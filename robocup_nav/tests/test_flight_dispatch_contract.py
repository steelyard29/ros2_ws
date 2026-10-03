from dataclasses import replace
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from flight_dispatch_contract import FlightDispatchContract
from flight_supervisor import FlightOutput, FlightSample


class DispatchContractTests(unittest.TestCase):
    def setUp(self):
        self.guard = FlightDispatchContract()
        self.sample = FlightSample(status_at=10, local_at=10, flags_at=10,
            land_at=10, rc_at=10, landed=True, nav_state=14, preflight_ok=True,
            rc_valid=True, input_ownership_ok=True, ev_ok=True, ev_position=True,
            ev_height=True, ev_yaw=True, baro_height=True, xy_valid=True,
            z_valid=True, v_xy_valid=True, v_z_valid=True,
            x=0., y=0., z=0., heading=0., vx=0., vy=0., vz=0.)
        self.output = FlightOutput('ARM', True, (0., 0., 0.), 0., 'ARM')

    def check(self, output=None, sample=None, **kwargs):
        output = output or self.output
        args = dict(now=10., stamp=10000000, state=output.state, origin=(0.,0.,0.,0.),
                    height=.5, switch_fresh=True, mode_slot=6, kill_switch=3, exclusive=True)
        args.update(kwargs)
        self.guard.check(output, sample or self.sample, **args)

    def test_nominal_arm(self):
        self.check()

    def test_unsafe_arm_snapshots_latch(self):
        for changes in ({'armed': True}, {'landed': False}, {'nav_state': 2},
                        {'preflight_ok': False}, {'rc_at': 0}, {'status_at': 0},
                        {'ev_ok': False}, {'failsafe': True}, {'manual_takeover': True}):
            self.guard = FlightDispatchContract()
            with self.assertRaises(ValueError):
                self.check(sample=replace(self.sample, **changes))
            with self.assertRaises(ValueError):
                self.check(now=10.1, stamp=10000001)

    def test_stream_allows_aux_debounce_while_px4_already_offboard(self):
        hold = FlightOutput('STREAM', True, (0., 0., 0.), 0., None)
        self.check(hold, mode_slot=1)

    def test_kill_ownership_and_switches(self):
        for changes in ({'kill_switch': 1}, {'exclusive': False},
                        {'mode_slot': 1}, {'switch_fresh': False}):
            self.guard = FlightDispatchContract()
            with self.assertRaises(ValueError):
                self.check(**changes)

    def test_bad_setpoints(self):
        for xyz in ((.01,0,0), (0,.01,0), (0,0,-.6), (0,0,float('nan')), (0,0,-.1)):
            self.guard = FlightDispatchContract()
            with self.assertRaises(ValueError):
                self.check(replace(self.output, position=xyz))

    def test_vision_loss_land_allowed_but_takeover_not(self):
        land = FlightOutput('ABORT', command='LAND')
        bad_ev = replace(self.sample, armed=True, landed=False, ev_ok=False, local_at=0)
        self.check(land, bad_ev)
        self.guard = FlightDispatchContract()
        self.check(land, replace(bad_ev, rc_at=0), switch_fresh=False)
        for changes in ({'nav_state': 2}, {'status_at': 0}, {'armed': False}):
            self.guard = FlightDispatchContract()
            with self.assertRaises(ValueError):
                self.check(land, replace(bad_ev, **changes))

    def test_quiet_stale_allowed_and_terminal_no_restart(self):
        self.check(FlightOutput('KILLED'), FlightSample())
        with self.assertRaises(ValueError):
            self.check(now=11., stamp=11000000)

    def test_wrong_state_command_and_clock(self):
        for output in (replace(self.output, command='DISARM'),
                       replace(self.output, state='HOVER'),
                       FlightOutput('KILLED', command='LAND')):
            self.guard = FlightDispatchContract()
            with self.assertRaises(ValueError):
                self.check(output)
        self.guard = FlightDispatchContract()
        self.check()
        with self.assertRaises(ValueError):
            self.check(now=11., stamp=10000000)

    def test_command_throttle(self):
        self.check()
        with self.assertRaises(ValueError):
            self.check(now=10.2, stamp=10200000)

    def test_runtime_sequences_with_aux_inputs(self):
        from test_flight_runtime import RuntimeRig
        from test_aux_switch_decoder import params
        scenarios = ({}, {'tracking': 2}, {'status': False}, {'mode': 1}, {'kill': 1})
        expected = ('DONE', 'STOPPED', 'ABORT', 'HANDOVER', 'KILLED')
        for injected, terminal in zip(scenarios, expected):
            with self.subTest(injected=injected):
                rig = RuntimeRig(aux_params=params())
                guard = FlightDispatchContract()
                for index in range(240):
                    # Synthetic landing response, not a real motor command.
                    if rig.output and rig.output.command == 'LAND':
                        rig.armed = False
                        rig.nav = 18
                    output = rig.tick(**(injected if index >= 100 else {}))
                    core = rig.c
                    guard.check(output, core.sample, now=rig.now,
                        stamp=int((rig.now+1)*1e6), state=core.controller.state,
                        origin=core.controller.origin, height=core.controller.height,
                        switch_fresh=core.switch_fresh(rig.now), mode_slot=core.mode_slot,
                        kill_switch=core.kill_switch, exclusive=core._owns(rig.now))
                self.assertEqual(output.state, terminal)
                self.assertIsNone(guard.fault)


if __name__ == '__main__':
    unittest.main()
