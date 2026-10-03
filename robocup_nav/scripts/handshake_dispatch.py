"""Fail-closed, disarmed-only dispatch contract. No ROS or device side effects.

Only the separate explicitly authorized bench entrypoint may select these
fixed routes. This is NOT a flight controller, and can never authorize ARM.
"""
import math

from disarmed_handshake import validate_handshake_output
from flight_output_boundary import OUTPUTS
from flight_supervisor import FlightSupervisor

SOURCE_SYSTEM = 1
SOURCE_COMPONENT = 191  # MAV_COMP_ID_ONBOARD_COMPUTER; reserved for this session.


def handshake_routes():
    return {name: '/fmu/in/'+name for name in OUTPUTS}


def handshake_ownership_ok(output_counts, real_inputs):
    return (set(output_counts) == set(OUTPUTS) and all(n == 1 for n in output_counts.values())
            and real_inputs == {t: 1 for t in handshake_routes().values()})


def validate_target(params):
    if any(type(params.get(k)) not in (int, float) or params[k] != 1
           for k in ('MAV_SYS_ID', 'MAV_COMP_ID')):
        raise ValueError('reviewed target must be MAV_SYS_ID=1, MAV_COMP_ID=1')


class HandshakeDispatchGuard:
    def __init__(self):
        self.command_count = 0
        self.last_stamp = None
        self.fault = None

    def check(self, output, core, now, stamp):
        try:
            if self.fault:
                raise ValueError(self.fault)
            validate_handshake_output(output)
            if getattr(core, 'fault_bench', False) and output.command:
                raise ValueError('fault bench forbids every command')
            if not getattr(core, 'staged', False) or not getattr(core, 'handshake_only', False):
                raise ValueError('staged disarmed handshake required')
            if type(stamp) is not int or stamp <= 0 or (self.last_stamp is not None and stamp <= self.last_stamp):
                raise ValueError('dispatch timestamp must advance')
            self.last_stamp = stamp
            if output.state != core.handshake.state:
                raise ValueError('output/state mismatch')
            if not output.stream and not output.command:
                return
            s = core.sample
            fresh = FlightSupervisor.fresh
            if (not math.isfinite(now) or core.ev_stopped or core.fault or core.command_fault
                    or not core.ev_allowed(now) or not core._owns(now)
                    or s.armed or not s.landed or not fresh(s.status_at, now, 1.5)
                    or not fresh(s.local_at, now, .5) or not fresh(s.land_at, now, 2.)
                    or not core.switch_fresh(now) or core.kill_switch != 3):
                raise ValueError('unsafe/stale dispatch snapshot')
            if core.handshake.origin is None or (tuple(output.position) != core.handshake.origin[:3]
                                                or output.yaw != core.handshake.origin[3]):
                raise ValueError('setpoint changed from captured ground pose')
            if output.command:
                if self.command_count or core.mode_slot != 6 or core.handshake.requested is None:
                    raise ValueError('duplicate or unsolicited mode request')
                self.command_count += 1
        except (ValueError, TypeError) as exc:
            self.fault = self.fault or str(exc)
            core.trip('dispatch boundary: '+self.fault)
            raise ValueError(self.fault) from exc


def build_handshake_messages(output, stamp):
    """Restricted serializer used only after the dispatch guard succeeds."""
    from flight_messages import build_messages
    validate_handshake_output(output)
    messages = build_messages(output, stamp)
    if 'vehicle_command' in messages:
        cmd = messages['vehicle_command']
        if cmd.command != 176 or cmd.param1 != 1. or cmd.param2 != 6.:
            raise ValueError('only standard OFFBOARD mode request is permitted')
        cmd.source_system, cmd.source_component = SOURCE_SYSTEM, SOURCE_COMPONENT
        cmd.target_system = cmd.target_component = 1
    return messages
