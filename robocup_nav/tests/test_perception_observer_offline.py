"""Run the actual observer main with in-memory ROS APIs and an analytic clock.

No DDS, subprocess, device or robot access. This is L1 orchestration evidence,
not a ROS transport simulation or hardware stability measurement.
"""
from contextlib import redirect_stdout
import importlib.util
import io
import json
import os
from pathlib import Path
import shutil
import sys
import tempfile
import types
import unittest
from unittest.mock import patch
import yaml
import numpy as np  # Load before sys.modules restoration; avoid repeated NumPy imports.

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from task_map_profile import mapper_profile
NS=types.SimpleNamespace


class ObserverFixture:
    def __init__(self,scenario):
        self.scenario=scenario;self.t=100.;self.step=0
        self.subs={};self.closed=False;self.shutdown=False
        self.k=yaml.safe_load((ROOT/'config/downward_camera_info.yaml').read_text())['camera_matrix']['data']
        self.profile=mapper_profile(0.)

    def create_subscription(self,kind,topic,callback,qos):self.subs[topic]=callback
    def destroy_node(self):self.closed=True
    def get_clock(self):return NS(now=lambda:NS(nanoseconds=round((1000+self.t)*1e9)))
    def get_topic_names_and_types(self):return [(t,[]) for t in self.subs]
    def get_publishers_info_by_topic(self,topic):
        changed=self.scenario=='owner_change' and self.t>=104 and topic.endswith('/odometry')
        name='mapper' if topic.startswith('/nvblox_node/') else 'fixture'
        owner=NS(node_name=name,node_namespace='/',endpoint_gid=[2 if changed else 1])
        if self.scenario=='duplicate_owner' and self.t>=104 and topic.endswith('/odometry'):
            return [owner,owner]
        return [owner]
    def count_publishers(self,topic):
        if topic.endswith('navigation_goal'):
            return int(self.scenario=='unexpected_goal' and self.t>=104)
        return 1

    def lookup_transform(self,*args):
        bad=self.scenario=='wrong_tf' and self.t>=104
        return NS(transform=NS(translation=NS(x=.25 if bad else .196,y=.025,z=-.05),
                               rotation=NS(x=0.,y=0.,z=0.,w=1.)))

    def message(self,topic):
        age=.01
        if self.scenario=='startup_transient' and self.t<100.8 and topic.endswith('/odometry'):age=.53
        ns=round((1000+self.t-age)*1e9)
        if self.scenario=='startup_transient' and self.t<101 and topic.endswith('static_map_slice'):ns=0
        m=NS(header=NS(stamp=NS(sec=ns//10**9,nanosec=ns%10**9),frame_id='odom'))
        m.child_frame_id='base_link'
        m.pose=NS(pose=NS(position=NS(x=0.,y=0.,z=0.),orientation=NS(x=0.,y=0.,z=0.,w=1.)))
        m.vo_state=0 if self.scenario=='tracking_loss' and self.t>=104 else 1
        if self.scenario=='nan_pose' and self.t>=104:m.pose.pose.position.x=float('nan')
        m.k=list(self.k)
        if self.scenario=='wrong_intrinsics' and self.t>=104:m.k[0]+=10
        m.origin=NS(z=self.profile['center_z']);m.data=[-1.,1.,-999.];m.unknown_value=-999.
        return m

    def spin_once(self,node,timeout_sec):
        self.step+=1;self.t=100+self.step/30
        for topic,callback in list(self.subs.items()):
            if topic.endswith('h_candidate'):continue  # Ground H is not required.
            if 'nvblox_node' in topic and self.step%6:continue
            if (topic in ('/scan','/robocup/legacy_apf/scan') or 'downward' in topic) and self.step%3:continue
            if self.t>=104 and ((self.scenario=='map_stopped' and topic.endswith('static_map_slice')) or
                    (self.scenario=='vio_stopped' and topic.endswith('/odometry'))):continue
            m=self.message(topic)
            if topic.endswith('esdf_slice_bounds'):
                for side,dz in [('bottom_height_limit',-self.profile['band_below']),
                                ('top_height_limit',self.profile['band_above'])]:
                    m.ns=side;m.type=11;m.action=0;m.id=0
                    m.points=[NS(z=self.profile['center_z']+dz)]*6
                    callback(m)
            else:callback(m)

    def modules(self):
        names={'rclpy.qos':['qos_profile_sensor_data'], 'nav_msgs.msg':['Odometry'],
               'sensor_msgs.msg':['Image','Imu','CompressedImage','CameraInfo','LaserScan'],
               'std_msgs.msg':['String'],'isaac_ros_visual_slam_interfaces.msg':['VisualSlamStatus'],
               'nvblox_msgs.msg':['DistanceMapSlice'],'visualization_msgs.msg':['Marker']}
        result={}
        for name,fields in names.items():
            package=name.split('.')[0]
            result.setdefault(package,types.ModuleType(package))
            mod=types.ModuleType(name)
            for field in fields:setattr(mod,field,object)
            result[name]=mod;setattr(result[package],name.split('.')[1],mod)
        ros=result['rclpy'];ros.init=lambda **kw:None
        ros.create_node=lambda *a,**kw:self;ros.spin_once=self.spin_once
        ros.try_shutdown=lambda:setattr(self,'shutdown',True)
        ros.time=NS(Time=lambda:None)
        tf=types.ModuleType('tf2_ros');tf.Buffer=lambda:self;tf.TransformListener=lambda *a:None
        result['tf2_ros']=tf
        return result


class ObserverOfflineTests(unittest.TestCase):
    def observe(self,scenario):
        f=ObserverFixture(scenario)
        with tempfile.TemporaryDirectory(prefix='observer_offline_') as d, \
                patch.dict(sys.modules,f.modules()),patch.dict(os.environ), \
                patch('subprocess.Popen',side_effect=AssertionError('no real process')), \
                patch('socket.socket',side_effect=AssertionError('no network')):
            root=Path(d);(root/'config').mkdir()
            shutil.copyfile(ROOT/'config/downward_camera_info.yaml',root/'config/downward_camera_info.yaml')
            spec=importlib.util.spec_from_file_location('observer_fixture',ROOT/'tests/perception_recovery_observer.py')
            m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
            m.ROOT=root;m.time=NS(monotonic=lambda:f.t,strftime=lambda fmt:'fixture')
            with patch.object(sys,'argv',['observer','--navigation','--initial-z','0','--qualified-window']), \
                    redirect_stdout(io.StringIO()) as output:
                code=m.main()
            report=json.loads(output.getvalue())
            raw=json.loads((root/'evidence/fixture/stream_timing.json').read_text())
            self.assertTrue(f.closed and f.shutdown)
            return code,report,raw

    def test_clean_full_observer_passes_no_ground_h(self):
        code,r,raw=self.observe('healthy')
        self.assertEqual(code,0,r)
        self.assertTrue(r['passed']);self.assertFalse(r['flight_ready'])
        self.assertEqual(r['stable_h'],0)
        self.assertGreaterEqual(r['acceptance_window']['finished']-r['acceptance_window']['measure_started'],12)
        self.assertEqual(r['ground_reference']['samples'],60)

    def test_startup_raw_failures_retained_measurement_clean(self):
        code,r,raw=self.observe('startup_transient')
        self.assertEqual(code,0,r)
        self.assertTrue(any(x[1]==0 for x in raw['map']))
        self.assertGreater(max(x[2] for x in raw['vio']),.5)
        self.assertEqual(r['streams']['map']['zero_stamps'],0)
        self.assertLess(r['streams']['vio']['max_source_age_s'],.25)
        self.assertGreater(r['startup_counts']['vio'],0)

    def test_real_observer_rejects_each_runtime_fault(self):
        for scenario in ('owner_change','duplicate_owner','unexpected_goal','wrong_tf',
                         'wrong_intrinsics','tracking_loss','nan_pose','map_stopped','vio_stopped'):
            with self.subTest(scenario=scenario):
                code,r,_=self.observe(scenario)
                self.assertEqual(code,1,r)
                self.assertFalse(r['passed'])
                self.assertEqual(r['acceptance_window']['phase'],'FAILED')
                self.assertLess(r['elapsed_s'],5.)


if __name__=='__main__':unittest.main()
