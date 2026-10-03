"""Offline/shadow candidate for the reviewed PX4 1.16.2 AUX mirror contract.

No ROS, publishers, parameters writes or control actions. Derived observations
are explicitly not native manual_control_switches. Require a current export;
zero/unassigned AUX values alone never establish mapping validity.
"""
from dataclasses import dataclass
import math


def validate_contract(params):
    expected = {'RC_MAP_AUX1': 6, 'RC_MAP_AUX2': 5, 'RC_MAP_FLTMODE': 6,
                'RC_MAP_KILL_SW': 5, 'RC_MAP_OFFB_SW': 0, 'RC_MAP_FLTM_BTN': 0,
                'RC_KILLSWITCH_TH': .75, 'COM_RC_IN_MODE': 0,
                **{f'COM_FLTMODE{i}': 2 if i < 6 else 7 for i in range(1, 7)}}
    for name, value in expected.items():
        actual = params.get(name)
        if (type(actual) not in (int, float) or not math.isfinite(actual)
                or not math.isclose(actual, value, abs_tol=1e-6, rel_tol=0)):
            raise ValueError('AUX mirror contract missing/mismatch: '+name)


def px4_slot(value):
    # v1.16.2 RCUpdate::UpdateManualSwitches. Conservative boundary margin
    # below avoids relying on float32 rounding at an exact slot boundary.
    return min(6, int((((value+1.05)*6+1/6)/2.1)+1/6+1))


@dataclass(frozen=True)
class DerivedSwitches:
    mode_slot: int
    kill_switch: int
    timestamp_sample: int
    origin: str = 'derived_aux_mirror_not_native'


class AuxSwitchDecoder:
    def __init__(self, params):
        validate_contract(params)
        self.last_stamp = self.last_sample = None
        self.fault = None
        self.candidate = None
        self.candidate_since = None
        self.last_receipt = None
        self.last_good = None
        self.rejection = None

    def receive(self, *, stamp, sample, now, age, valid, source, aux1, aux2):
        self.rejection = None
        if self.fault:
            self.rejection = self.fault
            return None
        if (type(stamp) is not int or type(sample) is not int or sample <= 0
                or stamp <= 0 or not math.isfinite(now) or not math.isfinite(age)
                or not -.05 <= age <= .5 or not 0 <= stamp-sample <= 500_000
                or age+(stamp-sample)/1e6 > .5 or not valid or source != 1
                or not all(math.isfinite(v) and -1 <= v <= 1 for v in (aux1, aux2))):
            self.fault = self.rejection = 'invalid AUX source/value/time'
            return None
        if self.last_stamp is not None:
            if stamp < self.last_stamp or sample < self.last_sample or now < self.last_receipt:
                self.fault = self.rejection = 'AUX source clock reset'
                return None
            if stamp == self.last_stamp or sample == self.last_sample:
                self.rejection = 'duplicate sample; freshness not renewed'
                return None
            if now-self.last_receipt > .5 or sample-self.last_sample > 500_000:
                self.fault = self.rejection = 'AUX source gap; new session required'
                return None
        self.last_stamp, self.last_sample, self.last_receipt = stamp, sample, now
        # 0.02 normalized mode margin; 0.02 in [0,1] kill-threshold units.
        mode = px4_slot(aux1)
        if (px4_slot(max(-1., aux1-.02)) != mode
                or px4_slot(min(1., aux1+.02)) != mode or abs((aux2+1)/2-.75) <= .02):
            self.fault = self.rejection = 'ambiguous mode/kill boundary; new session required'
            return None
        kill = 1 if (aux2+1)/2 > .75 else 3
        candidate = (mode, kill)
        if candidate != self.candidate:
            self.candidate, self.candidate_since = candidate, sample
        # Kill assertion is never delayed. Normal stable state requires 0.1 s
        # of progressing RC sample time, not duplicated host receipt times.
        if kill != 1 and sample-self.candidate_since < 100_000:
            self.rejection = 'waiting for stable candidate'
            return None
        self.last_good = DerivedSwitches(mode, kill, sample)
        return self.last_good
