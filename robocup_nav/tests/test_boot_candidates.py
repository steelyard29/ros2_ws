"""Pure checks of staged boot candidates; no systemd/docker/device execution."""
import subprocess
import sys
import unittest
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from boot_dds_agent import validate_start


class BootTests(unittest.TestCase):
    def test_unused_serial_accepts(self):validate_start(True,None,[])
    def test_existing_or_missing_rejected(self):
        for args in ((False,None,[]),(True,123,[]),(True,None,[123])):
            with self.assertRaises(RuntimeError):validate_start(*args)
    def test_default_description_no_execution(self):
        p=subprocess.run([sys.executable,str(ROOT/'scripts/boot_dds_agent.py')],
                         capture_output=True,text=True,timeout=3)
        self.assertEqual(p.returncode,0);self.assertIn('Describe only',p.stdout)
    def test_units_do_not_restart_or_launch_flight(self):
        for f in (ROOT/'systemd').glob('*-boot.service'):
            s=f.read_text()
            self.assertIn('Restart=no',s);self.assertIn('User=cfly',s)
            self.assertNotIn('flight_runtime',s)
            self.assertNotIn('disarmed_ev_session',s)
    def test_sensor_entry_reuses_bounded_recovery_and_clock_gate(self):
        s=(ROOT/'scripts/boot_sensor_stack.sh').read_text()
        self.assertIn('NTPSynchronized',s)
        self.assertIn('perception_reboot_session.py --authorized-sensor-recovery',s)
        self.assertNotIn('while ',s)


if __name__=='__main__':unittest.main()
