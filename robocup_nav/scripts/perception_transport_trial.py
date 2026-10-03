"""One explicit <=90s sensor-only transport rollout; no PX4 API or route goal.

Exact inspected container launchers are stopped, unrelated sensors/container stay.
Success retains only new perception/map/navigation candidate launchers; failure
stops those owned launch groups without retry. Default only describes the plan.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from bench_session_deadline import DeadlineAlarm
from disarmed_ev_lifecycle import atomic_json
from task_map_profile import mapper_profile


def identity(pid):
    try:
        # Start tick protects against PID reuse while stopping an inspected tree.
        return Path(f'/proc/{pid}/stat').read_text().rsplit(')',1)[1].split()[19]
    except FileNotFoundError:return None


def tree(roots):
    pairs=[tuple(map(int,line.split())) for line in subprocess.check_output(
        ['ps','-eo','pid=,ppid='],text=True,timeout=2).splitlines()]
    found=set(roots)
    while True:
        extra={p for p,parent in pairs if parent in found}-found
        if not extra:return {p:identity(p) for p in found}
        found.update(extra)


def group_alive(pid):
    try:os.killpg(pid,0);return True
    except ProcessLookupError:return False


def require_qualified_report(data):
    """Do not accept a stale observer version's bare passed=True."""
    if (data.get('passed') is not True or data.get('domain')!=176 or
            data.get('passive_only') is not True or data.get('flight_ready') is not False):
        raise RuntimeError('observer scope/pass mismatch')
    q=data.get('acceptance_window',{})
    start=q.get('measure_started');finish=q.get('finished')
    if (q.get('phase')!='PASSED' or q.get('passed') is not True or q.get('fault') is not None
            or q.get('required_measurement_s')!=12. or q.get('flight_ready') is not False
            or not all(type(x) in (float,int) and math.isfinite(x) for x in (start,finish))
            or start<0 or finish-start<12.):
        raise RuntimeError('observer lacks a complete qualified 12s window')


def require_ground_reference(data):
    ground=data.get('ground_reference') or {}
    xyz=ground.get('position',[]);spread=ground.get('max_displacement_m')
    tf=data.get('camera_tf',[])
    if (len(xyz)!=3 or not all(type(x) in (int,float) and math.isfinite(x) for x in xyz)
            or type(spread) not in (int,float) or not math.isfinite(spread) or not 0<=spread<=.05
            or ground.get('samples',0)<20):
        raise RuntimeError('no finite stable new ground reference')
    expected=[.196,.025,-.05,0,0,0,1]
    if (len(tf)!=7 or not all(type(x) in (int,float) and math.isfinite(x) and abs(x-y)<=1e-6
                             for x,y in zip(tf,expected))):
        raise RuntimeError('new camera installation TF mismatch/nonfinite')
    return xyz[2]


def require_same_ground_session(initial,final):
    require_ground_reference(final)
    first=initial.get('qualified_source_identity',{})
    second=final.get('qualified_source_identity',{})
    if any(not first.get(k) or first[k]!=second.get(k) for k in ('vio','depth','status')):
        raise RuntimeError('perception source changed after ground reference')
    a=initial['ground_reference']['position'];b=final['ground_reference']['position']
    if math.dist(a,b)>.05:raise RuntimeError('stationary ground reference moved between observations')


def verify_stopped(restore_support=False):
    """Read-only absence checks; invoked only inside an authorized rollout."""
    needles=('perception.launch.py','nvblox.launch.py','task_navigation.launch.py',
             'realsense2_camera_node','visual_slam','nvblox_node')
    if restore_support:
        needles+=('downward_camera_preview.launch.py','v4l2_camera_node','rplidar_node','h_target_preview.py')
    conflicts=[]
    for path in Path('/proc').glob('[0-9]*/cmdline'):
        try:cmd=path.read_bytes().decode(errors='replace').replace('\0',' ')
        except (FileNotFoundError,ProcessLookupError):continue
        if any(n in cmd for n in needles):conflicts.append((int(path.parent.name),cmd))
    if conflicts:raise RuntimeError('expected stopped; conflicting processes: '+repr(conflicts))
    import rclpy
    rclpy.init(args=[])
    node=rclpy.create_node('perception_stopped_audit',enable_rosout=False,start_parameter_services=False)
    topics=('/visual_slam/tracking/odometry','/camera/camera/depth/image_rect_raw',
            '/nvblox_node/static_map_slice','/robocup/task/navigation_command',
            '/robocup/task/navigation_goal')
    if restore_support:topics+=('/scan','/robocup/downward/image_raw','/robocup/downward/camera_info')
    try:
        until=time.monotonic()+2
        while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.05)
        counts={t:node.count_publishers(t) for t in topics}
        if any(counts.values()):raise RuntimeError('expected stopped; existing publishers: '+repr(counts))
        return counts
    finally:node.destroy_node();rclpy.try_shutdown()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--authorized-sensor-rollout',action='store_true')
    p.add_argument('--expect-stopped',action='store_true',help='verify stopped instead of replacing existing launchers')
    p.add_argument('--restore-support-sensors',action='store_true')
    p.add_argument('--session-deadline',type=float,help='shared host monotonic deadline including Docker startup')
    for name in ('perception','map','navigation'):p.add_argument('--old-'+name,type=int)
    a=p.parse_args()
    if not a.authorized_sensor_rollout:
        print('Describe only: one 90s sensor-only rollout; exact old PIDs OR verified --expect-stopped; 18s startup + 12s measurement per observer; keep new chain only on success.')
        return 0
    if not Path('/.dockerenv').exists():p.error('run inside inspected existing container')
    if a.restore_support_sensors and not a.expect_stopped:p.error('support restoration requires stopped mode')
    if a.session_deadline is not None and (not math.isfinite(a.session_deadline)
            or not 20<a.session_deadline-time.monotonic()<=90):p.error('invalid shared deadline')
    olds={a.old_perception:'perception.launch.py',a.old_map:'nvblox.launch.py',a.old_navigation:'task_navigation.launch.py'}
    if a.expect_stopped:
        if any(x is not None for x in (a.old_perception,a.old_map,a.old_navigation)):
            p.error('stopped mode cannot include old PIDs')
        olds={}
    elif len(olds)!=3 or any(pid is None or pid<=1 for pid in olds):p.error('three distinct inspected launch PIDs required')
    for pid,name in olds.items():
        cmd=Path(f'/proc/{pid}/cmdline').read_bytes().decode().split('\0')
        target=str(ROOT/'launch'/name)
        if target not in cmd or cmd[cmd.index(target)-1]!='launch':p.error('old launcher identity mismatch')
    os.environ.update(ROS_DOMAIN_ID='176',ROS_LOCALHOST_ONLY='1')
    os.environ['AMENT_PREFIX_PATH']=str(ROOT.parent/'build/uav_task/ament_cmake_index')+':'+os.environ.get('AMENT_PREFIX_PATH','')
    out=ROOT/'evidence'/time.strftime('perception_transport_rollout_%Y%m%d_%H%M%S');out.mkdir()
    started=time.monotonic();end=a.session_deadline if a.session_deadline is not None else started+90
    alarm=DeadlineAlarm(end-15)
    owned=[];logs=[];report=dict(passed=False,flight_ready=False,limit_s=90,hardware_retry=False,
        old_launchers=olds,profile='shm16m',px4_output=False,restore_support_sensors=a.restore_support_sensors,
        session_deadline_monotonic=end)
    def spawn(name,command):
        log=(out/(name+'.log')).open('w');logs.append(log)
        child=subprocess.Popen(command,
                               stdout=log,stderr=log,start_new_session=True)
        owned.append(child);return child.pid
    def launch(name,arguments):
        return spawn(name,['ros2','launch',str(ROOT/'launch'/name)]+arguments)
    def observe(*extra):
        record=dict(index=len(report.setdefault('observer_runs',[]))+1)
        report['observer_runs'].append(record)
        prefix='observer_'+str(record['index'])
        stdout='';stderr=''
        try:
            result=subprocess.run([sys.executable,str(ROOT/'tests/perception_recovery_observer.py'),'--qualified-window',*extra],
                                  text=True,stdout=subprocess.PIPE,stderr=subprocess.PIPE,
                                  timeout=min(33,max(.01,end-16-time.monotonic())))
            stdout=result.stdout;stderr=result.stderr;record['returncode']=result.returncode
        except subprocess.TimeoutExpired as exc:
            stdout=exc.stdout or '';stderr=exc.stderr or '';record['timeout']=True
            raise
        finally:
            for suffix,value in (('stdout',stdout),('stderr',stderr)):
                path=out/(prefix+'.'+suffix+'.log')
                path.write_text(value.decode(errors='replace') if isinstance(value,bytes) else value)
                record[suffix]=str(path)
        # No actuation/driver startup in this pre-existing observer.
        data=json.loads(result.stdout[result.stdout.index('{'):])
        record['observation']=data
        if result.returncode or not data['passed']:raise RuntimeError('observer failed: '+json.dumps(data))
        require_qualified_report(data)
        return data
    try:
        alarm.arm()
        if a.expect_stopped:report['stopped_audit']=verify_stopped(a.restore_support_sensors)
        old_tree=tree(olds)
        report['old_process_tree']=old_tree
        atomic_json(out/'report.json',report)
        for pid in olds:os.kill(pid,signal.SIGINT)
        until=time.monotonic()+9
        while time.monotonic()<until and any(identity(pid)==token for pid,token in old_tree.items() if token):
            time.sleep(.05)
        remaining=[pid for pid,token in old_tree.items() if token and identity(pid)==token]
        if remaining:raise RuntimeError('old descendants did not exit; no new startup: '+str(remaining))
        report['old_processes_exited']=True
        if a.restore_support_sensors:
            report['downward_pid']=launch('downward_camera_preview.launch.py',[])
            report['lidar_pid']=spawn('lidar',[str(ROOT.parent/'build/rplidar_ros/rplidar_node'),'--ros-args',
                '-p','channel_type:=serial','-p','serial_port:=/dev/rplidar','-p','serial_baudrate:=115200',
                '-p','frame_id:=laser','-p','inverted:=false','-p','angle_compensate:=true'])
            report['lidar_tf_pid']=spawn('lidar_tf',['ros2','run','tf2_ros','static_transform_publisher',
                '--x','0','--y','0','--z','0.1','--roll','0','--pitch','0','--yaw','0',
                '--frame-id','base_link','--child-frame-id','laser'])
        report['perception_pid']=launch('perception.launch.py',
            ['nvblox_depth:=true','imu_fusion:=false','publish_map_tf:=false','image_transport_profile:=shm16m'])
        initial=observe();report['initial_observation']=initial
        z=require_ground_reference(initial);profile=mapper_profile(z);report['mapper_profile']=profile
        report['map_pid']=launch('nvblox.launch.py',[k+':='+str(v) for k,v in profile.items()])
        report['navigation_pid']=launch('task_navigation.launch.py',
            ['center_z:='+str(profile['center_z']),'apf_executable:='+str(ROOT/'host_task_build/apf/legacy_apf_shadow')])
        final=observe('--navigation','--initial-z',str(z));report['final_observation']=final
        require_same_ground_session(initial,final)
        vio=final['streams']['vio'];depth=final['streams']['depth'];mapping=final['streams']['map']
        report['timing_passed']=bool(vio['max_source_gap_s']<=.3 and vio['max_gap_s']<=.3
            and vio['max_source_age_s']<=.25 and depth['max_source_age_s']<=.5
            and mapping['max_gap_s']<=.5 and mapping['max_source_age_s']<=.5)
        if not report['timing_passed']:raise RuntimeError('final input timing outside existing acceptance')
        if final['navigation_goal_publishers']!=0:raise RuntimeError('unexpected navigation goal writer')
        if any(child.poll() is not None for child in owned):raise RuntimeError('new launcher exited')
        report['passed']=True
    except Exception as exc:report['error']=repr(exc)
    finally:
        alarm.cancel()
        if not report['passed']:
            for sig,until in ((signal.SIGINT,time.monotonic()+5),(signal.SIGTERM,time.monotonic()+7),
                              (signal.SIGKILL,min(end-1,time.monotonic()+8))):
                for child in owned:
                    try:os.killpg(child.pid,sig)
                    except ProcessLookupError:pass
                while time.monotonic()<until and any(group_alive(child.pid) for child in owned):
                    for child in owned:child.poll()
                    time.sleep(.05)
        report.update(retained_new_pids=[c.pid for c in owned if c.poll() is None],
                      remaining_owned_groups=[c.pid for c in owned if group_alive(c.pid)],
                      elapsed_s=time.monotonic()-started,deadline_met=time.monotonic()<end)
        for log in logs:log.close()
        atomic_json(out/'report.json',report)
        print(json.dumps(dict(evidence=str(out),**report),indent=2))
    return 0 if report['passed'] and report['deadline_met'] else 1


if __name__=='__main__':raise SystemExit(main())
