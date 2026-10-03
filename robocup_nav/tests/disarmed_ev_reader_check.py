"""Container domain180 fake sensors/TF -> actual read-only worker pipe."""
import os
import fcntl
import argparse
os.environ.update(ROS_LOCALHOST_ONLY='1',ROS_DOMAIN_ID='180')
import json
from pathlib import Path
import select
import subprocess
import sys
import time
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped
from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
from rclpy.qos import qos_profile_sensor_data as qos
from tf2_ros import StaticTransformBroadcaster
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from disarmed_ev_lifecycle import atomic_json, request_stop, check_receipt, stop_reader


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--case',choices=['duplicate','stop','before-start','backpressure','stereo'],default='duplicate')
    args=parser.parse_args()
    root=Path(__file__).resolve().parents[1]
    out=root/'evidence'/(time.strftime('disarmed_ev_reader_%Y%m%d_%H%M%S')+'_'+args.case);out.mkdir()
    started=time.monotonic();report=dict(passed=False,hardware=False,domain=180)
    rclpy.init(domain_id=180)
    node=rclpy.create_node('synthetic_reader_fixture',enable_rosout=False,start_parameter_services=False)
    pose=node.create_publisher(Odometry,'/robocup/ev_reader_test/pose',qos)
    status=node.create_publisher(VisualSlamStatus,'/robocup/ev_reader_test/status',qos)
    images = [node.create_publisher(Image, '/robocup/ev_reader_test/'+k, qos)
              for k in ('infra1', 'infra2')] if args.case=='stereo' else []
    tf=StaticTransformBroadcaster(node)
    t=TransformStamped();t.header.frame_id='base_link';t.child_frame_id='camera_link'
    t.transform.translation.x=.196;t.transform.translation.y=.025;t.transform.translation.z=-.05
    t.transform.rotation.w=1.;tf.sendTransform(t)
    err=(out/'stderr.log').open('w')
    control=out/'control.json';receipt=out/'closed.json'
    atomic_json(control,dict(session='pipe-test',stop=False))
    if args.case=='before-start':request_stop(control,'pipe-test')
    child=subprocess.Popen([sys.executable,str(root/'scripts/disarmed_ev_sensor_reader.py'),
        '--isolated','--session','pipe-test','--deadline',str(started+9),
        '--control-file',str(control),'--receipt-file',str(receipt)]
        + (['--trace-image-inputs'] if images else []),stdout=subprocess.PIPE,stderr=err)
    if args.case=='backpressure':
        report['pipe_capacity_bytes']=fcntl.fcntl(child.stdout.fileno(),fcntl.F_SETPIPE_SZ,4096)
    os.set_blocking(child.stdout.fileno(),False);buffer=b'';packets=[];duplicate=None
    first_pipe_data=None
    try:
        while time.monotonic()-started<7 and child.poll() is None:
            stamp=node.get_clock().now().to_msg()
            m=Odometry();m.header.stamp=stamp;m.header.frame_id='odom';m.child_frame_id='base_link'
            m.pose.pose.orientation.w=1.;pose.publish(m)
            s=VisualSlamStatus();s.header.stamp=stamp;s.vo_state=1;status.publish(s)
            for publisher in images:
                i=Image();i.header.stamp=stamp;i.width=1;i.height=1;i.step=1;i.encoding='mono8';i.data=[0]
                publisher.publish(i)
            rclpy.spin_once(node,timeout_sec=.02)
            if args.case=='backpressure' and first_pipe_data is None and select.select([child.stdout],[],[],0)[0]:
                first_pipe_data=time.monotonic()
            if args.case=='backpressure' and first_pipe_data is not None and time.monotonic()-first_pipe_data>=1.2:
                stopped=time.monotonic()
                request_stop(control,'pipe-test')
                # Do NOT drain until the reader has exited. This reproduces the
                # old blocking-stdout stop hang with a real full OS pipe.
                child.wait(timeout=1)
                report['stop_without_drain_s']=time.monotonic()-stopped
                ack=check_receipt(receipt,'pipe-test',child.returncode,time.monotonic())
                assert ack['confirmed'],ack
                assert ack['receipt']['transport']['would_block']>0,ack
                while True:
                    chunk=os.read(child.stdout.fileno(),65536)
                    if not chunk:break
                    buffer+=chunk
                while b'\n' in buffer:
                    line,buffer=buffer.split(b'\n',1);packets.append(json.loads(line))
                break
            if args.case!='backpressure' and select.select([child.stdout],[],[],0)[0]:
                buffer+=os.read(child.stdout.fileno(),65536)
                while b'\n' in buffer:
                    line,buffer=buffer.split(b'\n',1);packets.append(json.loads(line))
            if len(packets)>40 and duplicate is None:
                if args.case=='duplicate':
                    duplicate=node.create_publisher(Odometry,'/robocup/ev_reader_test/pose',qos)
                elif args.case in ('stop', 'stereo'):
                    result=stop_reader(child,control,receipt,'pipe-test')
                    assert result['confirmed'],result
                    report['stop_handshake']=result
                    break
            if any(p['kind']=='fault' for p in packets):break
        child.wait(timeout=2)
        if args.case!='before-start':
            assert sum(p['kind']=='pose' for p in packets)>5,packets[:2]
            assert sum(p['kind']=='tracking' for p in packets)>5
        assert [p['seq'] for p in packets]==list(range(1,len(packets)+1))
        assert all(p['session']=='pipe-test' for p in packets)
        if args.case=='duplicate':assert any(p['kind']=='fault' and 'duplicate' in p['reason'] for p in packets)
        assert child.returncode==(1 if args.case=='duplicate' else 0),child.returncode
        ack=check_receipt(receipt,'pipe-test',child.returncode,time.monotonic())
        assert ack['confirmed'],ack
        if args.case=='before-start':assert not packets,packets
        if args.case=='stereo':
            for kind in ('infra1', 'infra2'):
                rows=[p for p in packets if p['kind']==kind]
                assert len(rows)>5,rows
                assert all(p['width']==1 and p['height']==1 and 'data' not in p for p in rows)
        assert not any(t.startswith('/fmu/') for t,_ in node.get_topic_names_and_types())
        report.update(passed=True,packets=len(packets),case=args.case,receipt=ack,
            duplicate_source_rejected=args.case=='duplicate',
            child_exit_code=child.returncode,scope='synthetic reader TF/topics and private pipe only')
    except Exception as exc:report['error']=repr(exc)
    finally:
        if child.poll() is None:child.kill();child.wait(timeout=1)
        node.destroy_node();rclpy.try_shutdown();err.close()
        report['elapsed_s']=time.monotonic()-started
        (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(dict(evidence=str(out),**report),indent=2))
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
