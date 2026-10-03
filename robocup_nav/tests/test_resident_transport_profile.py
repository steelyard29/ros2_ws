"""No ROS/device access: worker must not inherit unrelated transport defaults."""
import os
from pathlib import Path
import sys
import unittest
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from resident_vio_worker import configure_transport


class ProfileTests(unittest.TestCase):
    def test_explicit_profile_replaces_inherited_bridge_profile(self):
        with patch.dict(os.environ,{'FASTRTPS_DEFAULT_PROFILES_FILE':'/wrong/bridge.xml',
                                    'RMW_IMPLEMENTATION':'wrong'}):
            path=configure_transport(176)
            self.assertTrue(Path(path).is_file())
            self.assertEqual(os.environ['ROS_DOMAIN_ID'],'176')
            self.assertEqual(os.environ['ROS_LOCALHOST_ONLY'],'1')
            self.assertEqual(os.environ['RMW_IMPLEMENTATION'],'rmw_fastrtps_cpp')
            for name in ('FASTRTPS_DEFAULT_PROFILES_FILE','FASTDDS_DEFAULT_PROFILES_FILE'):
                self.assertEqual(os.environ[name],path)
            self.assertIn('16777216',Path(path).read_text())

    def test_isolated_domain_remains_isolated(self):
        with patch.dict(os.environ):
            configure_transport(183)
            self.assertEqual(os.environ['ROS_DOMAIN_ID'],'183')
