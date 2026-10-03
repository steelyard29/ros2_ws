"""Independent readonly observer source-clock validation; no ROS or I/O."""
import math
from copy import deepcopy


def diagnostic_value(value):
    """Keep invalid clocks visible while emitting strict, portable JSON."""
    if isinstance(value, float) and not math.isfinite(value):
        return repr(value)
    if isinstance(value, dict):
        return {k: diagnostic_value(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [diagnostic_value(v) for v in value]
    return value


def telemetry_observation(name, message):
    """Small raw-state view for diagnosis only; never fed into readiness."""
    fields = {
        'flags': ('cs_ev_pos', 'cs_ev_yaw', 'cs_ev_hgt', 'cs_ev_vel',
                  'cs_baro_hgt', 'cs_rng_hgt'),
        'local': ('xy_valid', 'z_valid', 'v_xy_valid', 'v_z_valid',
                  'x', 'y', 'z', 'heading'),
        'status': ('arming_state', 'nav_state', 'gcs_connection_lost', 'failsafe'),
        'land': ('landed',), 'rc': ('valid',),
        'ev': ('quality', 'pose_frame'),
    }
    return diagnostic_value({key: getattr(message, key)
                             for key in fields.get(name, ()) if hasattr(message, key)})


class SourceGuard:
    def __init__(self):
        self.stamps = {}
        self.receipts = {}
        self.duplicates = 0
        self.fault = None
        self.first_fault = None
        self.last_received = {}
        self.received_counts = {}

    def latch_fault(self, reason, name):
        """Save the first failure, including the offending clocks, exactly once."""
        if self.fault is None:
            self.fault = reason
            record = deepcopy(self.last_received.get(name, {}))
            record.update(reason=reason, topic=name,
                          previous_accepted_stamp_us=self.stamps.get(name),
                          previous_accepted_receipt_monotonic_s=self.receipts.get(name))
            self.first_fault = record
        return False

    def diagnostics(self, now):
        return diagnostic_value(deepcopy({
            'telemetry_frozen': self.fault is not None,
            'first_fault': self.first_fault,
            'last_accepted_stamp_us': self.stamps,
            'last_accepted_receipt_monotonic_s': self.receipts,
            'last_accepted_receipt_age_s': {k: now-v for k, v in self.receipts.items()},
            'received_counts': self.received_counts,
            'last_received_unvalidated': self.last_received,
            'semantics': 'latest telemetry is last accepted per topic, not necessarily current; '
                         'raw received observations cannot clear the latch or certify readiness',
        }))

    def accept(self, name, stamp, now, age, sample=None, sample_lead_us=0, observed=None):
        self.received_counts[name] = self.received_counts.get(name, 0) + 1
        record = diagnostic_value(dict(stamp_us=stamp, receipt_monotonic_s=now,
            source_age_s=age, sample_us=sample, sample_lead_us=sample_lead_us,
            source_age_limits_s=[-.05, .5], observed=observed,
            decision='rejected_latched' if self.fault else 'rejected'))
        self.last_received[name] = record
        if self.fault:
            return False
        stamps = {name: stamp}
        if sample is not None:
            # PX4 outputs keep a zero lead. EV may carry a VIO stamp up to the
            # publisher's existing 50 ms future allowance, plus truncation.
            if (type(sample_lead_us) is not int or not 0 <= sample_lead_us <= 60_000
                    or type(sample) is not int or sample <= 0 or type(stamp) is not int
                    or not -sample_lead_us <= stamp-sample <= 500_000):
                return self.latch_fault('invalid sample clock: '+name, name)
            stamps[name+'_sample'] = sample
            sample_age = age+(stamp-sample)/1e6
            record['sample_age_s'] = diagnostic_value(sample_age)
            if not math.isfinite(sample_age) or not -.05 <= sample_age <= .5:
                return self.latch_fault('stale sample: '+name, name)
        if (type(stamp) is not int or stamp <= 0 or not math.isfinite(now)
                or not math.isfinite(age) or not -.05 <= age <= .5):
            return self.latch_fault('invalid/stale source: '+name, name)
        if name in self.receipts and now < self.receipts[name]:
            return self.latch_fault('receipt clock reversed: '+name, name)
        for key, value in stamps.items():
            if key in self.stamps and value < self.stamps[key]:
                return self.latch_fault('source clock reset: '+key, name)
        if any(self.stamps.get(key) == value for key, value in stamps.items()):
            self.duplicates += 1
            record['decision'] = 'duplicate_not_renewed'
            return False
        self.stamps.update(stamps)
        self.receipts[name] = now
        record['decision'] = 'accepted'
        return True


def observer_passed(max_ready, final_ready, ever_armed, fault):
    return bool(max_ready >= 5 and final_ready and not ever_armed and fault is None)
