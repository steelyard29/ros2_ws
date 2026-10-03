"""Flight EV policy shared by shadow tests and the separately released live core.

Retains bench geometry, time, quality and no-resume guards. This does not
publish anything. Existing prop-off EvAdapter/EvBridge still stop on arming.
"""
from ev_adapter import EvAdapter


class FlightEvAdapter(EvAdapter):
    def __init__(self, started):
        super().__init__(started, allow_ev_hgt=True)
        self.seen_disarmed = False
        self.armed = False

    def on_vehicle(self, arming_state, now):
        if self.inhibit:
            return False
        if arming_state not in (1, 2):
            self.trip('Unknown arming state')
            return False
        if arming_state == 2 and (not self.seen_disarmed or self.publish_count == 0):
            self.trip('Flight EV cannot start or warm up airborne')
            return False
        self.vehicle_at = now
        self.disarmed = arming_state == 1
        self.seen_disarmed |= self.disarmed
        self.armed = arming_state == 2
        return True

    def vehicle_ready(self):
        return self.seen_disarmed and (self.disarmed or self.armed)
