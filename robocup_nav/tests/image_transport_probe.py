"""Domain181 full-size synthetic DDS images, separate producer/consumer processes.

No camera/VIO/flight imports or hardware topics. Default is describe-only.
This is a transport experiment, not cuVSLAM or physical-performance validation.
"""
import argparse
import hashlib
from array import array
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import uuid

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
from disarmed_ev_lifecycle import atomic_json

KINDS = ('infra1', 'infra2', 'depth')
N = 150
WARMUP = 30


def profile_path(profile):
    return ROOT/({'shm16m':'config/perception_dds_shm16m.xml',
        'shm512k':'tests/fixtures/fastdds_shm512k.xml',
        'udp':'tests/fixtures/fastdds_udp_only.xml'}[profile])


def mapped_segments():
    paths={line.split()[-1] for line in Path('/proc/self/maps').read_text().splitlines()
           if '/dev/shm/fastrtps_' in line and not line.endswith('(deleted)')}
    return {p:Path(p).stat().st_size for p in paths if Path(p).exists()}


def worker(a):
    # Never inherit a hardware domain or the PX4 DDS profile.
    os.environ.update(ROS_DOMAIN_ID='181', ROS_LOCALHOST_ONLY='1', RMW_IMPLEMENTATION='rmw_fastrtps_cpp')
    for name in ('FASTRTPS_DEFAULT_PROFILES_FILE', 'FASTDDS_DEFAULT_PROFILES_FILE',
                 'RMW_FASTRTPS_USE_QOS_FROM_XML'):
        os.environ.pop(name, None)
    if a.profile != 'builtin':
        path = profile_path(a.profile)
        os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = str(path)
        os.environ['FASTDDS_DEFAULT_PROFILES_FILE'] = str(path)
    import rclpy
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Image
    rclpy.init(domain_id=181)
    node = rclpy.create_node('image_transport_'+a.role, enable_rosout=False, start_parameter_services=False)
    topics = {k: '/robocup/isolated_image_probe/s_'+a.session+'/'+k for k in KINDS}
    started = time.monotonic()
    result = dict(role=a.role, domain=181, profile=a.profile, hardware=False, session=a.session)
    try:
        if a.role == 'sender':
            qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
            pubs = {k: node.create_publisher(Image, topics[k], qos) for k in KINDS}
            while time.monotonic()-started < 4 and any(p.get_subscription_count()!=1 for p in pubs.values()):
                rclpy.spin_once(node, timeout_sec=.02)
            assert all(p.get_subscription_count()==1 for p in pubs.values()), 'discovery did not complete'
            messages = {}
            for k in KINDS:
                m = Image(); m.width=640; m.height=480
                m.encoding = '16UC1' if k=='depth' else 'mono8'
                m.step=640*(2 if k=='depth' else 1)
                m.data = array('B', [0])*(m.step*m.height)
                messages[k] = m
            rows=[]; schedule=time.monotonic()
            for tick, index in enumerate(range(-WARMUP, N)):
                delay=schedule+tick/30-time.monotonic()
                if delay>0: time.sleep(delay)
                assert time.monotonic()<a.deadline, 'publisher exceeded budget'
                stamp=node.get_clock().now().to_msg()
                now=time.monotonic()
                for k, m in messages.items():
                    m.header.stamp=stamp; m.header.frame_id=str(index); m.data[0]=index%256
                    pubs[k].publish(m)
                rows.append(dict(seq=index, stamp_ns=stamp.sec*10**9+stamp.nanosec, at=now))
                rclpy.spin_once(node, timeout_sec=0)
            result.update(samples=rows, bytes_per_frame={k:len(m.data) for k,m in messages.items()},
                          mapped_segments=mapped_segments())
            atomic_json(a.out/'done.json', dict(at=time.monotonic(),count=len(rows)))
            until=time.monotonic()+.6
            while time.monotonic()<until: rclpy.spin_once(node,timeout_sec=.01)
        else:
            rows={k:[] for k in KINDS}
            def receive(k,m):
                assert len(m.data)==640*480*(2 if k=='depth' else 1), 'truncated image'
                seq=int(m.header.frame_id)
                assert m.data[0]==seq%256, 'corrupt payload marker'
                stamp=m.header.stamp.sec*10**9+m.header.stamp.nanosec
                rows[k].append(dict(seq=seq,stamp_ns=stamp,at=time.monotonic(),
                                   age_s=(time.time_ns()-stamp)/1e9))
            for k in KINDS:
                node.create_subscription(Image,topics[k],lambda m,k=k:receive(k,m),qos_profile_sensor_data)
            while time.monotonic()<a.deadline:
                rclpy.spin_once(node,timeout_sec=.01)
                if (a.out/'done.json').exists():
                    done=json.loads((a.out/'done.json').read_text())
                    if time.monotonic()>done['at']+.5:break
            result['samples']=rows
            result['mapped_segments']=mapped_segments()
        assert not any(t.startswith('/fmu/') for t,_ in node.get_topic_names_and_types()), 'isolation violated'
        result['completed']=True
    except Exception as exc:
        result.update(completed=False,error=repr(exc))
    finally:
        node.destroy_node();rclpy.try_shutdown()
        result['elapsed_s']=time.monotonic()-started
        atomic_json(a.out/(a.role+'.json'),result)
    return 0 if result['completed'] else 1


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run-isolated',action='store_true')
    p.add_argument('--profile',choices=['builtin','shm512k','shm16m','udp'],default='builtin')
    p.add_argument('--sender-container',action='store_true',help='synthetic sender only in existing isaac_ros_dev')
    p.add_argument('--receiver-container',action='store_true',help='synthetic receiver in same existing container')
    p.add_argument('--receiver-profile',choices=['builtin','shm512k','shm16m','udp'])
    p.add_argument('--role',choices=['sender','receiver'])
    p.add_argument('--out',type=Path)
    p.add_argument('--session')
    p.add_argument('--deadline',type=float)
    a=p.parse_args()
    if a.role:
        if not a.out or not a.session or not a.deadline or not 0<a.deadline-time.monotonic()<14:
            p.error('worker needs parent-owned output, session and bounded deadline')
        return worker(a)
    if not a.run_isolated:
        print('Describe only: explicit flag runs synthetic 640x480 IR1+IR2+depth at30Hz, domain181, <=14s.')
        return 0
    sid=uuid.uuid4().hex
    out=ROOT/'evidence'/('image_transport_'+time.strftime('%Y%m%d_%H%M%S')+'_'+a.profile+'_'+sid[:6])
    out.mkdir(); started=time.monotonic();deadline=started+12
    children=[];logs=[]
    report=dict(hardware=False,domain=181,profile=a.profile,fixture_completed=False,session=sid,
        scope='ordinary ROS Image transport only; no NITROS, GPU, synchronizer or physical camera',
        qos='publisher RELIABLE depth10; subscriber BEST_EFFORT depth5; explicit fixture, not live QoS readback')
    report.update(sender_location='container' if a.sender_container else 'host',
                  receiver_location='container' if a.receiver_container else 'host',
                  receiver_profile=a.receiver_profile or a.profile)
    report.update(warmup_frames_per_topic=WARMUP,measurement_frames_per_topic=N,
                  warmup_semantics='negative sequences are startup warmup, retained separately, not silently discarded',
                  script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    if a.profile!='builtin':
        report['profile_sha256']=hashlib.sha256(profile_path(a.profile).read_bytes()).hexdigest()
    if a.receiver_profile and a.receiver_profile!='builtin':
        report['receiver_profile_sha256']=hashlib.sha256(profile_path(a.receiver_profile).read_bytes()).hexdigest()
    try:
        for role in ('receiver','sender'):
            log=(out/(role+'.log')).open('w');logs.append(log)
            selected=(a.receiver_profile or a.profile) if role=='receiver' else a.profile
            script=str(Path(__file__).resolve());worker_out=str(out)
            prefix=[sys.executable]
            if (role=='sender' and a.sender_container) or (role=='receiver' and a.receiver_container):
                container_root=Path('/workspaces/ros2_ws/robocup_nav')
                script=str(container_root/'tests/image_transport_probe.py')
                worker_out=str(container_root/out.relative_to(ROOT))
                # Inner timeout owns the container process even if the docker client exits.
                prefix=['docker','exec','isaac_ros_dev','timeout','--signal=TERM','--kill-after=1','13',
                    'bash','-c','source /opt/ros/humble/setup.bash; exec "$@"',
                    'isolated_image_worker','/usr/bin/python3']
            children.append(subprocess.Popen(prefix+[script,
                '--role',role,'--out',worker_out,'--session',sid,'--deadline',str(deadline),'--profile',selected],
                stdout=log,stderr=log,cwd=out))
        for child in children:child.wait(timeout=max(.1,deadline+1-time.monotonic()))
        assert all(c.returncode==0 for c in children), 'child failed'
        s=json.loads((out/'sender.json').read_text());r=json.loads((out/'receiver.json').read_text())
        sent=[x for x in s['samples'] if x['seq']>=0];expected={x['seq'] for x in sent}
        report.update(fixture_completed=s['completed'] and r['completed'],submitted=len(sent),
            sender_max_interval_s=max(b['at']-a['at'] for a,b in zip(sent,sent[1:])),
            streams={},mapped_segments={role:data['mapped_segments'] for role,data in (('sender',s),('receiver',r))})
        for k,all_rows in r['samples'].items():
            rows=[x for x in all_rows if x['seq']>=0]
            unique={x['seq'] for x in rows}
            report['streams'][k]=dict(received=len(rows),unique=len(unique),
                warmup_received=sum(x['seq']<0 for x in all_rows),
                missing_sequences=sorted(expected-unique),
                max_source_gap_s=max(((b['stamp_ns']-a['stamp_ns'])/1e9 for a,b in zip(rows,rows[1:])),default=None),
                max_age_s=max((x['age_s'] for x in rows),default=None))
        report['all_images_delivered']=all(not x['missing_sequences'] for x in report['streams'].values())
    except Exception as exc:report['error']=repr(exc)
    finally:
        for c in children:
            if c.poll() is None:
                c.terminate()
                try:c.wait(timeout=.4)
                except subprocess.TimeoutExpired:c.kill();c.wait(timeout=.4)
        for log in logs:log.close()
        report['child_exit_codes']=[c.returncode for c in children]
        report['elapsed_s']=time.monotonic()-started
        errors=[role for role in ('sender','receiver') if 'XMLPARSER' in (out/(role+'.log')).read_text()]
        report['xml_error_logs']=errors
        if errors:report['fixture_completed']=False
        atomic_json(out/'report.json',report)
        print(json.dumps(dict(evidence=str(out),**report),indent=2))
    return 0 if report['fixture_completed'] else 1


if __name__=='__main__':raise SystemExit(main())
