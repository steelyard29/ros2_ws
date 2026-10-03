import os
from pathlib import Path
import unittest
from unittest.mock import patch
from px4_runtime_transport import configure


class Px4TransportTests(unittest.TestCase):
    def test_host_profile_overrides_inherited_perception_settings(self):
        with patch.dict(os.environ,{'ROS_DOMAIN_ID':'176','ROS_LOCALHOST_ONLY':'1',
                                    'FASTRTPS_DEFAULT_PROFILES_FILE':'old',
                                    'FASTDDS_DEFAULT_PROFILES_FILE':'other'},clear=True):
            configure()
            self.assertEqual(os.environ['ROS_DOMAIN_ID'],'0')
            self.assertEqual(os.environ['ROS_LOCALHOST_ONLY'],'0')
            expected='/home/cfly/ros2_ws/config/fastdds_bridge.xml'
            self.assertEqual(os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'],expected)
            self.assertEqual(os.environ['FASTDDS_DEFAULT_PROFILES_FILE'],expected)

    def test_task_runtime_uses_one_host_profile_and_container_keeps_isolation(self):
        root=Path(__file__).resolve().parents[1]
        live=(root/'scripts/live_flight_runtime.py').read_text()
        self.assertIn('from px4_runtime_transport import configure',live)
        self.assertNotIn('Both contexts share one',live)
        reader=(root/'scripts/task_perception_reader.py').read_text()
        self.assertIn("domain=183 if a.isolated else 176",reader)
        self.assertIn("ROS_LOCALHOST_ONLY='1'",reader)
