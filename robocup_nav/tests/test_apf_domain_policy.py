"""Compile and exercise the actual C++ domain policy without ROS or DDS."""
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import runpy
import os

ROOT = Path(__file__).resolve().parents[1]


class DomainPolicyTest(unittest.TestCase):
    def test_child_death_not_confused_with_running_launcher(self):
        detect = runpy.run_path(str(ROOT / 'tests/task_actual_navigation_replay.py'))['launch_child_failure']
        self.assertIsNone(detect('[INFO] [planner]: process started with pid [42]'))
        line = '[ERROR] [legacy_apf_shadow-6]: process has died [pid 42, exit code 64]'
        self.assertEqual(detect('[INFO] launch still running\n'+line), line)
        self.assertIn('exit code 64', detect((ROOT / 'evidence/task_actual_navigation_20260930_212447/navigation.log').read_text()))

    def test_nonlocal_test_rejected_before_ros_import(self):
        env = dict(os.environ, ROS_DOMAIN_ID='186', ROS_LOCALHOST_ONLY='0')
        result = subprocess.run([sys.executable, str(ROOT / 'tests/task_actual_navigation_replay.py'),
                                 '--run-isolated', '--apf', '/usr/bin/true'],
                                env=env, capture_output=True, text=True, timeout=5)
        self.assertEqual(result.returncode, 2)
        self.assertIn('ROS_LOCALHOST_ONLY=1', result.stderr)

    def test_policy_matrix_without_ros(self):
        source = r'''
#include "domain_policy.hpp"
int main() {
  const char *domains[] = {nullptr, "", "0", "176", "186", "185", "187"};
  for (const char *d : domains) for (bool real : {false, true})
    for (bool isolated : {false, true}) {
      const std::string v = d ? d : "";
      const bool expected = isolated ? (v == "186")
        : (v == "176" || (v == "0" && real));
      if (apf_domain::configured_allowed(d, real, isolated) != expected) return 1;
      if (expected && !apf_domain::entry_allowed(d)) return 2;
      if (apf_domain::entry_allowed(d) != (v == "0" || v == "176" || v == "186")) return 3;
    }
}
'''
        with tempfile.TemporaryDirectory(prefix='apf-domain-') as tmp:
            binary = str(Path(tmp) / 'policy')
            subprocess.run(['c++', '-std=c++17', '-x', 'c++', '-', '-I',
                            str(ROOT / 'legacy_apf_shadow'), '-o', binary],
                           input=source, text=True, check=True, capture_output=True, timeout=30)
            subprocess.run([binary], check=True, timeout=5)
        node = (ROOT / 'legacy_apf_shadow/node.cpp').read_text()
        self.assertIn('apf_domain::entry_allowed(domain)', node)
        self.assertIn('apf_domain::configured_allowed(domain,real_inputs_,isolated_task_test)', node)

    def test_replay_default_does_not_start_ros(self):
        result = subprocess.run([sys.executable, str(ROOT / 'tests/task_actual_navigation_replay.py')],
                                check=True, capture_output=True, text=True, timeout=5)
        self.assertIn('Describe only', result.stdout)
