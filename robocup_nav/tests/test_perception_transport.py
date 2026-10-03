import importlib.util
from pathlib import Path
import unittest
import subprocess
import sys
import xml.etree.ElementTree as ET
from launch import LaunchContext
from launch.utilities import perform_substitutions

ROOT=Path(__file__).resolve().parents[1]


class PerceptionTransportTests(unittest.TestCase):
    def test_cross_container_probe_default_has_no_execution(self):
        result=subprocess.run([sys.executable,str(ROOT/'tests/image_transport_probe.py'),
            '--sender-container','--receiver-container'],capture_output=True,text=True,timeout=3)
        self.assertEqual(result.returncode,0,result.stderr)
        self.assertIn('Describe only',result.stdout)

    def test_udp_fixture_does_not_enable_shared_memory(self):
        doc=ET.parse(ROOT/'tests/fixtures/fastdds_udp_only.xml')
        ns={'d':'http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles'}
        self.assertEqual([x.text for x in doc.findall('.//d:type',ns)],['UDPv4'])
        self.assertEqual(doc.find('.//d:useBuiltinTransports',ns).text,'false')

    def setup_launch(self, profile):
        spec=importlib.util.spec_from_file_location('perception_transport_test',ROOT/'launch/perception.launch.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        c=LaunchContext()
        c.launch_configurations.update(rgbd='false',nvblox_depth='true',publish_map_tf='false',
            imu_fusion='false',imu_unite_method='2',image_transport_profile=profile)
        return module,c

    def test_default_does_not_override_any_transport(self):
        m,c=self.setup_launch('unchanged');actions=m.nodes(c)
        self.assertIsNone(actions[0].additional_env)
        self.assertIsNone(actions[3].additional_env)
        declaration=next(a for a in m.generate_launch_description().entities
                         if getattr(a,'name',None)=='image_transport_profile')
        self.assertEqual(perform_substitutions(c,declaration.default_value),'unchanged')

    def test_candidate_applies_only_to_camera_and_vio(self):
        m,c=self.setup_launch('shm16m');actions=m.nodes(c)
        expected=str(ROOT/'config/perception_dds_shm16m.xml')
        for index in (0,3):
            env={perform_substitutions(c,k):perform_substitutions(c,v)
                 for k,v in actions[index].additional_env}
            self.assertEqual(env,dict(FASTRTPS_DEFAULT_PROFILES_FILE=expected,
                                     FASTDDS_DEFAULT_PROFILES_FILE=expected))
        self.assertIsNone(actions[2].additional_env)
        env={perform_substitutions(c,k):perform_substitutions(c,v) for k,v in actions[1].additional_env}
        self.assertEqual(env,dict(PYTHONUNBUFFERED='1'))

    def test_unknown_profile_rejected(self):
        m,c=self.setup_launch('arbitrary.xml')
        with self.assertRaises(RuntimeError):m.nodes(c)

    def test_control_changes_only_segment_size(self):
        large=ET.parse(ROOT/'config/perception_dds_shm16m.xml')
        small=ET.parse(ROOT/'tests/fixtures/fastdds_shm512k.xml')
        ns={'d':'http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles'}
        value=large.find('.//d:segment_size',ns)
        self.assertEqual(int(value.text),16*1024*1024)
        value.text=str(512*1024)
        self.assertEqual(ET.tostring(large.getroot()),ET.tostring(small.getroot()))


if __name__=='__main__':unittest.main()
