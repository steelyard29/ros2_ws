"""Keep the proven vertical-flight entrypoint separate from mission output.

Negative integration controls: passing means unsafe shortcut stays closed,
NOT that the new mission is integrated with READY/AUX/EV.
"""
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from flight_dispatch_contract import FlightDispatchContract
from flight_messages import build_messages
from flight_supervisor import FlightOutput, FlightSample
from flight_runtime import create_node
from test_sortie_shadow import Rig


class LegacyBoundaryTests(unittest.TestCase):
    def test_mission_state_not_silently_admitted_by_vertical_guard(self):
        output = FlightOutput('NAVIGATE', stream=True, position=(0., 0., -1.), yaw=0.)
        with self.assertRaisesRegex(ValueError, 'unknown state'):
            FlightDispatchContract().check(output, FlightSample(), now=1., stamp=1000000,
                state='NAVIGATE', origin=(0.,0.,0.,0.), height=1.1,
                switch_fresh=True, mode_slot=6, kill_switch=3, exclusive=True)

    def test_mixed_navigation_not_encoded_as_vertical_position(self):
        output = FlightOutput('HOVER', stream=True, position=(math.nan,math.nan,-1.), yaw=0.)
        with self.assertRaisesRegex(ValueError, 'invalid local setpoint'):
            build_messages(output, 1000000)

    def test_shadow_sortie_cannot_acquire_production_live_route(self):
        with self.assertRaisesRegex(ValueError, 'dedicated core and consumed release'):
            create_node(Rig().core, exercise=True, live_permit=object())


if __name__ == '__main__':
    unittest.main()
