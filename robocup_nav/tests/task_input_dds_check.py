#!/usr/bin/env python3
"""Bounded real ROS message-delivery check on synthetic domain177, no hardware.

Exercises actual input adapter in the actual sole-writer runtime, WAIT only.
Not a full flight, real A* planner, PX4 SITL or map coverage acceptance.
"""
import os
os.environ.update(ROS_DOMAIN_ID='177',ROS_LOCALHOST_ONLY='1')
os.environ.pop('FASTRTPS_DEFAULT_PROFILES_FILE',None)
from pathlib import Path
import sys
import json
import time
import yaml
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import rclpy
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Image
from nvblox_msgs.msg import DistanceMapSlice
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from task_map_fixture import markers
from flight_runtime_check import SyntheticPlant,synthetic_aux_params
from flight_runtime import create_node
from task_flight_controller import TaskFlightCore


def run(wrong_height=False):
    root=Path(__file__).resolve().parents[1]
    cfg=yaml.safe_load((root/'config/roundtrip_task.yaml').read_text())
    # Explicit synthetic configuration ONLY; deployment YAML stays unchanged.
    cfg.update(alignment_reviewed=True,bounds_odom=[-1,4,-2,2],full_height_slice_reviewed=True,
               takeoff_column_reviewed=True)
    rclpy.init(args=[]);plant=runtime=None;ex=SingleThreadedExecutor()
    started=time.monotonic();report={'passed':False,'hardware':False,'full_flight':False,
                                   'wrong_map_height_fixture':wrong_height}
    try:
        plant=SyntheticPlant('nominal',use_aux=True);plant.operator_position=True
        core=TaskFlightCore(started,synthetic_aux_params(),cfg)
        runtime=create_node(core,isolated=True,exercise=False)
        pubs={k:plant.create_publisher(cls,runtime.task_input.topics[k],10) for k,cls in
              [('map',DistanceMapSlice),('bounds',Marker),('depth',Image),('navigation',String),('h',String)]}
        def sensors():
            stamp=plant.get_clock().now().to_msg()
            depth=Image();depth.header.stamp=stamp;depth.width=depth.height=1;depth.data=[1]
            pubs['depth'].publish(depth)
            m=DistanceMapSlice();m.header.stamp=stamp;m.header.frame_id='odom'
            m.width=m.height=100;m.resolution=.1;m.origin.x=m.origin.y=-5.
            m.origin.z=.6 if wrong_height else .86
            for bound in markers(stamp):pubs['bounds'].publish(bound)
            m.unknown_value=-1000.;m.data=[2.]*10000;pubs['map'].publish(m)
            mission=runtime.task_input.mission
            if mission:
                ns=stamp.sec*10**9+stamp.nanosec
                d=dict(schema=1,frame_id='odom',goal_id=mission.goal_id,stamp_ns=ns,
                       path_stamp_ns=ns,velocity_xy=[.1,0.])
                pubs['navigation'].publish(String(data=json.dumps(d)))
        plant.create_timer(.05,sensors)
        ex.add_node(plant);ex.add_node(runtime)
        while time.monotonic()-started<6:
            ex.spin_once(timeout_sec=.02)
        a=runtime.task_input
        assert a.mission is not None,a.report()
        assert a.command is not None,a.report()
        if wrong_height:
            assert not core.controller.scene.footprint_known_free,a.report()
            # Wrong cruise-map height inhibits lateral flight, not reviewed
            # vertical takeoff. This WAIT-only fixture never arms regardless.
            assert any('does not cover' in b for b in a.navigation_blockers),a.report()
        else:
            assert core.controller.scene.footprint_known_free,a.report()
        assert core.controller.state=='WAIT',core.report()
        assert not plant.armed and not plant.commands
        assert not any(t.startswith('/fmu/in/') for t,_ in runtime.get_topic_names_and_types())
        report.update(passed=True,scope='real generated types through DDS into WAIT-only task runtime',
            navigation_goal=a.command[0],runtime=runtime.report(),elapsed_s=time.monotonic()-started)
    except Exception as exc:
        report['error']=str(exc)
        raise
    finally:
        if runtime:runtime.destroy_node()
        if plant:plant.destroy_node()
        ex.shutdown();rclpy.try_shutdown()
        out=root/'evidence'/time.strftime('task_input_dds_%Y%m%d_%H%M%S')
        out.mkdir(parents=True,exist_ok=False)
        (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(dict(evidence=str(out),**report),indent=2))


if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--wrong-height',action='store_true')
    run(parser.parse_args().wrong_height)
