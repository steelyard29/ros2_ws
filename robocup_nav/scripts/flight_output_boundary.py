"""Fixed output routing and fail-closed EV-only bench boundary. No ROS/devices.

There is deliberately no live-control mode. Even bench mode can only route EV
to PX4; every command/setpoint/heartbeat remains in a shadow namespace.
"""
from flight_supervisor import FlightSupervisor

OUTPUTS = ('vehicle_visual_odometry', 'offboard_control_mode',
           'trajectory_setpoint', 'vehicle_command')
EV_TOPIC = '/fmu/in/vehicle_visual_odometry'


def routes(*, isolated=False, bench_ev=False):
    if isolated and bench_ev:
        raise ValueError('EV-only bench cannot use isolated/synthetic inputs')
    prefix = '/robocup/runtime_test/output' if isolated else '/robocup/flight_runtime_shadow'
    mapping = {name: prefix+'/'+name for name in OUTPUTS}
    if bench_ev:
        mapping['vehicle_visual_odometry'] = EV_TOPIC
    return mapping


def ownership_ok(output_counts, real_inputs, *, bench_ev=False):
    expected = {EV_TOPIC: 1} if bench_ev else {}
    return (set(output_counts) == set(OUTPUTS) and all(v == 1 for v in output_counts.values())
            and real_inputs == expected)


class BenchEvGate:
    def __init__(self):
        self.sent = 0
        self.fault = None
        self.blockers = []

    def check(self, core, now):
        s = core.sample
        fresh = FlightSupervisor.fresh
        checks = {
            'disarmed_status': fresh(s.status_at, now, 1.5) and not s.armed,
            'landed': fresh(s.land_at, now, 2.) and s.landed,
            'valid_rc': fresh(s.rc_at, now, .5) and s.rc_valid,
            'position_slot': core.switch_fresh(now) and core.mode_slot in range(1, 6),
            'px4_position_mode': s.nav_state == 2,
            'kill_off': core.kill_switch == 3 and not core.kill_latched,
            'height_contract': fresh(s.flags_at, now, 2.) and s.baro_height
                               and not s.range_height and not s.ev_velocity,
            'exclusive_outputs': core._owns(now),
            'no_failsafe': not s.failsafe,
            'healthy_session': not core.fault and not core.ev.inhibit,
        }
        self.blockers = [name for name, ok in checks.items() if not ok]
        immediate = s.armed or core.kill_latched or core.ownership_fault or core.fault or core.ev.inhibit
        if self.blockers and (self.sent or immediate):
            self.fault = self.fault or 'EV-only bench gate: '+', '.join(self.blockers)
            core.trip(self.fault)
        return not self.fault and not self.blockers

    def sent_ev(self):
        self.sent += 1
