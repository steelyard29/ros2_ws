#!/usr/bin/env python3
"""Isolated generated DDS/guard shutdown check, not GPU/hardware validation."""
import os
os.environ.update(ROS_DOMAIN_ID='178', ROS_LOCALHOST_ONLY='1')
os.environ.pop('FASTRTPS_DEFAULT_PROFILES_FILE', None)
import hashlib
import json
from pathlib import Path
import signal
import subprocess
import sys
import time
import rclpy
from rclpy.qos import qos_profile_sensor_data as qos
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from nvblox_msgs.msg import DistanceMapSlice
from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus

ROOT=Path(__file__).resolve().parents[1]


def run():
    out=ROOT/'evidence'/time.strftime('guard_transport_%Y%m%d_%H%M%S')
    out.mkdir(parents=True,exist_ok=False)
    source=ROOT/'scripts/nvblox_guard.py'
    digest=lambda:hashlib.sha256(source.read_bytes()).hexdigest()
    report=dict(passed=False,hardware=False,container_started=False,domain=178,
        scope='generated DDS inputs and controlled SIGINT only',
        runtime='container' if Path('/.dockerenv').exists() else 'host',source_sha256=digest())
    rclpy.init(args=[]);node=rclpy.create_node('guard_transport_fixture')
    child=None;log=(out/'guard.log').open('w')
    labels=[];odometry=[];velocities=[];started=time.monotonic()
    try:
        node.create_subscription(String,'/robocup/nvblox/readiness',
            lambda m:labels.append((time.monotonic(),m.data)),10)
        node.create_subscription(Odometry,'/robocup/odom',lambda m:odometry.append(m),10)
        node.create_subscription(Twist,'/robocup/nav/cmd_vel_raw',
            lambda m:velocities.append((time.monotonic(),m.linear.x)),10)
        topics=[('/visual_slam/tracking/odometry',Odometry,qos),
                ('/visual_slam/status',VisualSlamStatus,qos),
                ('/camera/camera/depth/image_rect_raw',Image,qos),
                ('/nvblox_node/static_map_slice',DistanceMapSlice,10),
                ('/robocup/nvblox/cmd_vel_unchecked',Twist,10)]
        until=time.monotonic()+.5
        while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.02)
        assert not any(t.startswith('/fmu/') for t,_ in node.get_topic_names_and_types())
        assert all(node.count_publishers(t)==0 for t,_,_ in topics),'domain already in use'
        assert node.count_publishers('/robocup/nvblox/readiness')==0,'existing guard'
        pubs=[node.create_publisher(cls,topic,quality) for topic,cls,quality in topics]
        child=subprocess.Popen([sys.executable,str(source)],stdout=log,stderr=log,
                               start_new_session=True,env=os.environ.copy())
        until=time.monotonic()+4.;last_publish=0.
        while time.monotonic()<until:
            assert child.poll() is None,'guard exited while inputs active'
            now=time.monotonic()
            if now-last_publish>=.04:
                last_publish=now;stamp=node.get_clock().now().to_msg()
                o=Odometry();o.header.stamp=stamp;o.header.frame_id='odom'
                o.child_frame_id='base_link';o.pose.pose.orientation.w=1.
                s=VisualSlamStatus();s.header.stamp=stamp;s.vo_state=1
                d=Image();d.header.stamp=stamp;d.width=d.height=2
                d.encoding='16UC1';d.step=4;d.data=[0,1]*4
                m=DistanceMapSlice();m.header.stamp=stamp;m.header.frame_id='odom'
                m.width=m.height=20;m.resolution=.05;m.unknown_value=-1000.;m.data=[2.]*400
                v=Twist();v.linear.x=.1
                for pub,message in zip(pubs,(o,s,d,m,v)):pub.publish(message)
            rclpy.spin_once(node,timeout_sec=.005)
        assert odometry and any(s=='SHADOW_READY' for _,s in labels),'no healthy positive control'
        assert any(v>.09 for _,v in velocities),'no synthetic shadow command delivered'
        quiet=time.monotonic()
        while time.monotonic()-quiet<1.:
            assert child.poll() is None,'guard exited during silence'
            rclpy.spin_once(node,timeout_sec=.02)
        late=[v for t,v in velocities if t>quiet+.7]
        assert late and all(v==0. for v in late),'stale inputs did not inhibit shadow output'
        report['exit_code_before_stop']=child.poll()
        child.send_signal(signal.SIGINT);report['exit_code']=child.wait(timeout=3.)
        assert report['exit_code']==0,'guard failed controlled shutdown'
        assert digest()==report['source_sha256'],'guard changed during test'
        report.update(passed=True,odometry_samples=len(odometry),readiness_samples=len(labels),
                      stale_output_samples=len(late))
    except Exception as exc:report['error']=repr(exc)
    finally:
        if child is not None and child.poll() is None:
            os.killpg(child.pid,signal.SIGTERM)
            try:child.wait(timeout=2.)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid,signal.SIGKILL);child.wait(timeout=1.)
        report['child_exited']=child is None or child.poll() is not None
        report['elapsed_s']=time.monotonic()-started
        node.destroy_node();rclpy.try_shutdown();log.close()
        report['take_message_error_in_log']='Unable to convert call argument' in (out/'guard.log').read_text()
        (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(dict(evidence=str(out),**report),indent=2))
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(run())
