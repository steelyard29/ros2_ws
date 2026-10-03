import math
import sys
import unittest
from dataclasses import replace
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from competition_mission import CompetitionMission, Evidence, State


class MissionHarness:
    def __init__(self):
        self.m = CompetitionMission()
        self.now = 0.0
        self.e = Evidence(0, preflight_ok=True, localization_ok=True,
                          link_ok=True, control_owned=True)
        self.output = self.m.step(0, self.e, start=True)
        self.outputs = [self.output]

    def tick(self, **changes):
        self.now = round(self.now + 0.1, 3)
        self.e = replace(self.e, at=self.now, **changes)
        self.output = self.m.step(self.now, self.e)
        self.outputs.append(self.output)
        return self.output

    def ticks(self, count, **changes):
        for _ in range(count):
            self.tick(**changes)

    def airborne(self):
        self.ticks(103, landed=False, armed=True, height_m=1.2, stable=True)
        assert self.m.state == State.SEARCH

    def release(self, target='observed-1'):
        self.tick(target_id=target, target_class='red_cross', target_at=self.now + .1,
                  target_confidence=.95, completion_id='', release_ok=False)
        assert self.m.state == State.ALIGN
        for _ in range(12):
            self.tick(target_at=self.now + .1, completion_id=self.m.request_id,
                      alignment_ok=True)
        assert self.m.state == State.RELEASE


class MissionTests(unittest.TestCase):
    def test_missing_preflight_cannot_start(self):
        m = CompetitionMission()
        self.assertEqual(m.step(0, Evidence(0), start=True).kind, 'NONE')
        self.assertEqual(m.state, State.READY)

    def test_three_deliveries_and_direct_return(self):
        h = MissionHarness()
        h.airborne()
        for i in range(3):
            h.release(f'target-{i}')
            h.tick(release_ok=True, completion_id=h.m.request_id)
        self.assertEqual(h.m.state, State.RETURN)
        self.assertEqual([r['slot'] for r in h.m.released], [0, 1, 2])
        self.assertEqual(sum(o.kind == 'RELEASE_ONCE' for o in h.outputs), 3)
        h.tick(navigation_ok=True, completion_id=h.m.request_id)
        self.assertEqual(h.m.state, State.LAND)
        h.tick(landed=True, armed=False, height_m=0)
        self.assertEqual(h.m.state, State.DONE)
        self.assertEqual(h.tick().kind, 'NONE')

    def test_hover_must_be_continuous_and_above_one_metre(self):
        h = MissionHarness()
        h.ticks(110, landed=False, armed=True, height_m=1, stable=True)
        self.assertEqual(h.m.state, State.TAKEOFF)
        h.ticks(60, height_m=1.2)
        h.tick(stable=False)
        h.ticks(60, stable=True)
        self.assertEqual(h.m.state, State.TAKEOFF)
        h.ticks(43)
        self.assertEqual(h.m.state, State.SEARCH)

    def test_liftoff_deadline(self):
        h = MissionHarness()
        h.ticks(300)
        self.assertEqual(h.m.state, State.ABORT)
        self.assertIn('30 s', h.m.reason)

    def test_loss_latches_and_no_resume(self):
        for changes in ({'localization_ok': False}, {'link_ok': False},
                        {'control_owned': False}, {'height_m': math.nan},
                        {'height_m': 4.01}, {'collision': 'wall'},
                        {'collision': 'net'}, {'collision': 'ground'}):
            with self.subTest(changes=changes):
                h = MissionHarness()
                h.airborne()
                h.tick(**changes)
                self.assertEqual(h.m.state, State.ABORT)
                h.tick(localization_ok=True, link_ok=True, control_owned=True,
                       height_m=1.2, collision='')
                self.assertEqual(h.m.state, State.ABORT)
                h.tick(landed=True, armed=False, height_m=0)
                self.assertEqual(h.m.state, State.STOPPED)

    def test_manual_takeover_emits_no_land_or_control(self):
        h = MissionHarness()
        h.airborne()
        self.assertEqual(h.tick(manual_override=True).kind, 'NONE')
        self.assertEqual(h.m.state, State.HANDOVER)
        self.assertEqual(h.tick(manual_override=False).kind, 'NONE')

    def test_watchdog_and_stale_telemetry(self):
        for time, stamp in ((11, 11), (10.4, 9), (10.4, 11)):
            h = MissionHarness()
            h.airborne()
            h.m.step(time, replace(h.e, at=stamp))
            self.assertEqual(h.m.state, State.ABORT)

    def test_reject_clock_regression(self):
        h = MissionHarness()
        h.tick()
        with self.assertRaises(ValueError):
            h.m.step(0, h.e)

    def test_stale_or_low_confidence_target_not_used(self):
        h = MissionHarness()
        h.airborne()
        for changes in ({'target_at': 0}, {'target_confidence': .5},
                        {'target_class': 'unknown'}, {'target_confidence': math.nan}):
            args = dict(target_id='x', target_class='bridge',
                        target_at=h.now + .1, target_confidence=.95)
            args.update(changes)
            h.tick(**args)
            self.assertEqual(h.m.state, State.SEARCH)

    def test_release_has_no_retry_and_requires_correlated_confirmation(self):
        h = MissionHarness()
        h.airborne()
        h.release()
        h.ticks(31, release_ok=True, completion_id='old-command')
        self.assertEqual(h.m.state, State.RETURN)
        self.assertEqual(h.m.released, [])
        self.assertEqual(h.m.uncertain_slots, [0])
        self.assertEqual(sum(o.kind == 'RELEASE_ONCE' for o in h.outputs), 1)

    def test_alignment_loss_resets_dwell(self):
        h = MissionHarness()
        h.airborne()
        h.tick(target_id='x', target_class='tent', target_confidence=.9,
               target_at=h.now + .1)
        for _ in range(8):
            h.tick(target_at=h.now + .1, alignment_ok=True, completion_id=h.m.request_id)
        h.tick(alignment_ok=False)
        for _ in range(8):
            h.tick(target_at=h.now + .1, alignment_ok=True, completion_id=h.m.request_id)
        self.assertEqual(h.m.state, State.ALIGN)

    def test_vegetation_loses_avoidance_eligibility_only(self):
        h = MissionHarness()
        h.airborne()
        h.tick(collision='vegetation')
        self.assertFalse(h.m.avoidance_eligible)
        self.assertEqual(h.m.state, State.SEARCH)

    def test_return_reserve_and_hard_deadline(self):
        h = MissionHarness()
        h.airborne()
        while h.now < 540:
            h.tick()
        self.assertEqual(h.m.state, State.RETURN)
        h.tick(completion_id=h.m.request_id, navigation_ok=True)
        while h.now < 570:
            h.tick()
        self.assertEqual(h.m.state, State.LAND)
        # Start LAND later in a separate run to exercise the 600 s global cap.
        h = MissionHarness()
        h.airborne()
        while h.now < 579:
            h.tick()
        h.tick(completion_id=h.m.request_id, navigation_ok=True)
        while h.now < 600:
            h.tick()
        self.assertEqual(h.m.state, State.ABORT)
        self.assertEqual(h.m.reason, '600 s deadline')

    def test_search_exhaustion_requires_current_request(self):
        h = MissionHarness()
        h.airborne()
        h.tick(search_exhausted=True, completion_id='stale')
        self.assertEqual(h.m.state, State.SEARCH)
        h.tick(completion_id=h.m.request_id)
        self.assertEqual(h.m.state, State.RETURN)

    def test_abort_during_release_marks_inventory_uncertain(self):
        h = MissionHarness()
        h.airborne()
        h.release()
        h.tick(localization_ok=False, release_ok=True, completion_id=h.m.request_id)
        self.assertEqual(h.m.state, State.ABORT)
        self.assertEqual(h.m.uncertain_slots, [0])
        self.assertEqual(h.m.released, [])

    def test_old_navigation_ack_and_height_do_not_complete_landing(self):
        h = MissionHarness()
        h.airborne()
        h.tick(search_exhausted=True, completion_id=h.m.request_id)
        h.tick(navigation_ok=True, completion_id='old')
        self.assertEqual(h.m.state, State.RETURN)
        h.tick(completion_id=h.m.request_id)
        h.tick(height_m=0, landed=False, armed=True)
        self.assertEqual(h.m.state, State.LAND)
        h.tick(landed=True)
        self.assertEqual(h.m.state, State.LAND)
        h.tick(armed=False)
        self.assertEqual(h.m.state, State.DONE)


if __name__ == '__main__':
    unittest.main()
