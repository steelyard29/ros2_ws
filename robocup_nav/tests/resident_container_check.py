"""Actual CDR/container lifecycle, synthetic domains182/183 only, <=30s.

No real sensors/PX4, arm/mode/trajectory output, driver start or retry.
Default describes. Fixture publisher is only available with explicit isolation.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))


def source(deadline):
    if not 0<deadline-time.monotonic()<=25:raise ValueError('fixture budget')
    import rclpy
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from rclpy.qos import qos_profile_sensor_data
    os.environ['ROS_LOCALHOST_ONLY']='1'
    for key in ('FASTRTPS_DEFAULT_PROFILES_FILE','FASTDDS_DEFAULT_PROFILES_FILE'):
        os.environ.pop(key,None)
    rclpy.init(domain_id=183)
    node=rclpy.create_node('resident_container_synthetic_vio',enable_rosout=False,
                          start_parameter_services=False)
    pose=node.create_publisher(Odometry,'/robocup/runtime_test/input/vio',qos_profile_sensor_data)
    tracking=node.create_publisher(String,'/robocup/runtime_test/input/tracking',10)
    def tick():
        stamp=node.get_clock().now()
        m=Odometry();m.header.stamp=stamp.to_msg();m.header.frame_id='odom'
        m.child_frame_id='base_link';m.pose.pose.orientation.w=1.
        t=String();t.data=json.dumps(dict(stamp_ns=stamp.nanoseconds,vo_state=1))
        tracking.publish(t);pose.publish(m)
    node.create_timer(1/30,tick)
    try:
        while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.005)
    finally:node.destroy_node();rclpy.try_shutdown()


def check():
    from bench_session_deadline import DeadlineAlarm
    from disarmed_ev_lifecycle import atomic_json
    from resident_container_inputs import ContainerResidentInputs,container_path
    from resident_ev_node import create_node
    from flight_runtime_check import SyntheticPlant
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    start=time.monotonic();deadline=start+24;alarm=DeadlineAlarm(start+29)
    alarm.arm()
    out=ROOT/'evidence'/('resident_container_'+uuid.uuid4().hex);out.mkdir()
    report=dict(passed=False,hardware=False,flight_ready=False,host_domain=182,source_domain=183)
    manager=ContainerResidentInputs(out/'reader',isolated=True)
    fixture=fixture_log=executor=resident=plant=None
    try:
        os.environ.update(ROS_DOMAIN_ID='182',ROS_LOCALHOST_ONLY='1')
        for key in ('FASTRTPS_DEFAULT_PROFILES_FILE','FASTDDS_DEFAULT_PROFILES_FILE'):
            os.environ.pop(key,None)
        rclpy.init(domain_id=182)
        executor=SingleThreadedExecutor()
        plant=SyntheticPlant('position-slot');plant.operator_position=True
        resident=create_node(manager.session,isolated=True,perception_node=manager.port)
        executor.add_node(plant);executor.add_node(resident)
        fixture_log=(out/'source.log').open('w')
        fixture=subprocess.Popen(['docker','exec','-e','ROS_LOCALHOST_ONLY=1','isaac_ros_dev',
            'timeout','--signal=TERM','--kill-after=1','26','bash','-c',
            'source /opt/ros/humble/setup.bash; exec "$@"','synthetic_vio',
            'python3',container_path(Path(__file__)), '--run-isolated','--fixture-source',
            '--deadline',str(deadline)],stdout=fixture_log,stderr=subprocess.STDOUT)
        manager.start(deadline)
        while time.monotonic()<deadline-3 and resident.core.sent_count<60:
            executor.spin_once(timeout_sec=.002);manager.pump()
            if resident.core.fault:raise RuntimeError(resident.core.fault)
            if fixture.poll() is not None:raise RuntimeError('synthetic source exited early')
        report.update(sent=resident.core.sent_count,received=manager.receiver.received,
                      plant_ev=plant.counts['ev'],unique_ev_publishers=plant.count_publishers(resident.output),
                      commands=plant.commands,armed=plant.armed)
        assert resident.core.sent_count>=60,'EV delivery insufficient'
        assert report['unique_ev_publishers']==1 and not plant.commands and not plant.armed
        assert not any(n.startswith('/fmu/') for n,_ in plant.get_topic_names_and_types())
        report['delivery_passed']=True
    except Exception as exc:report['error']=repr(exc)
    finally:
        if resident:resident.close_output()
        manager.stop('isolated test finished')
        end=min(start+28,time.monotonic()+3)
        while time.monotonic()<end:
            cleanup=manager.poll_close()
            if not cleanup.get('running',False):break
            if executor:executor.spin_once(timeout_sec=.005)
            else:time.sleep(.005)
        report['reader_cleanup']=manager.poll_close()
        if executor:executor.shutdown(timeout_sec=.2)
        for node in (resident,plant):
            if node:node.destroy_node()
        rclpy.try_shutdown()
        # Fixture independently exits at its monotonic deadline; do not confuse
        # killing a docker client with confirmed container process shutdown.
        if fixture:
            try:report['source_exit']=fixture.wait(timeout=max(.1,start+28-time.monotonic()))
            except subprocess.TimeoutExpired:report['source_exit']=None
        if fixture_log:fixture_log.close()
        report['elapsed_s']=time.monotonic()-start
        report['passed']=bool(report.get('delivery_passed') and
            report['reader_cleanup'].get('confirmed') and report.get('source_exit')==0)
        alarm.cancel();atomic_json(out/'report.json',report)
        print(json.dumps(dict(evidence=str(out),**report),indent=2))
    return 0 if report['passed'] else 1


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run-isolated',action='store_true')
    p.add_argument('--fixture-source',action='store_true')
    p.add_argument('--deadline',type=float)
    a=p.parse_args()
    if not a.run_isolated:print(__doc__);return 0
    if a.fixture_source:
        if a.deadline is None:p.error('fixture needs deadline')
        source(a.deadline);return 0
    return check()


if __name__=='__main__':raise SystemExit(main())
