"""Ground -> takeoff -> navigation -> H -> touchdown, OFFLINE ONLY.

Reuses the original FlightSupervisor up to its successful hover completion.
Only this private subclass replaces normal LAND with an explicit handoff.
It is not LiveFlightCore and cannot obtain the existing real-flight route.
No EV bridge, READY/AUX runtime or real FlightPermit is implemented here.
"""
from dataclasses import dataclass
import math

from flight_supervisor import FlightSupervisor
from flight_dispatch_contract import FlightDispatchContract
from flight_messages import build_messages


@dataclass(frozen=True)
class TakeoffSpace:
    at: float
    column_verified: bool = False


class _TakeoffToMission(FlightSupervisor):
    def _enter(self, state, now, reason=''):
        if state == 'LAND' and self.state == 'HOVER' and not reason:
            state = 'MISSION_HANDOFF'
        super()._enter(state, now, reason)


class SortieShadow:
    TERMINAL = {'DONE', 'STOPPED', 'ABORT', 'HANDOVER', 'KILLED'}
    isolated_only = True

    def __init__(self, sequence, height=1.1, hover=3.):
        self.sequence = sequence
        self.takeoff = _TakeoffToMission(height, hover)
        self.takeoff_guard = FlightDispatchContract()
        self.state = 'WAIT'
        self.last = self.last_stamp = self.started = None
        self.handoff_at = None
        self.reason = ''
        self.history = []

    def stop(self, reason, state='ABORT'):
        if self.state not in self.TERMINAL:
            self.state, self.reason = state, reason
        return self.result({})

    def result(self, messages, **extras):
        return dict(state=self.state, reason=self.reason, messages=messages,
                    flight_authorized=False, **extras)

    def step(self, now, sample, scene, landing, space, *, timestamp_us,
             vio_session, start=False, accepted_commands=frozenset(),
             switch_fresh=False, mode_slot=-1, kill_switch=-1):
        if self.state in self.TERMINAL:
            return self.result({})
        try:
            if (not math.isfinite(now) or now < 0 or type(timestamp_us) is not int
                    or timestamp_us <= 0 or (self.last is not None and not 0 < now-self.last <= .3)
                    or (self.last_stamp is not None and timestamp_us <= self.last_stamp)):
                raise ValueError('sortie executive/timestamp deadline')
            self.last, self.last_stamp = now, timestamp_us
            if sample.kill_active or kill_switch == 1:
                return self.stop('kill asserted; no automatic recovery', 'KILLED')
            if sample.manual_takeover or scene.manual_override:
                return self.stop('manual takeover', 'HANDOVER')
            if self.started is not None and not sample.input_ownership_ok:
                return self.stop('output ownership lost', 'HANDOVER')
            alignment = self.sequence.navigation.alignment
            alignment.validate(vio_session, sample.reset_counters)
            fresh = FlightSupervisor.fresh
            if self.handoff_at is None:
                column_ok = bool(space is not None and space.column_verified and fresh(space.at, now, .25))
                if self.takeoff.state == 'WAIT' and not column_ok:
                    self.reason = 'takeoff column unverified; remain on ground'
                    return self.result({})
                if self.takeoff.state == 'WAIT' and start:
                    if (not fresh(scene.at, now, .25) or not scene.localization_ok
                            or len(scene.position) != 3 or not all(math.isfinite(v) for v in scene.position)):
                        self.reason = 'ground localization unverified; remain on ground'
                        return self.result({})
                    if abs(scene.position[2]+self.takeoff.height-self.sequence.navigation.mission.band.center) > 1e-5:
                        raise ValueError('takeoff height and initialized cruise map band disagree')
                if self.takeoff.state != 'WAIT' and not column_ok:
                    raise ValueError('takeoff column evidence lost')
                output = self.takeoff.step(now, sample, start=start,
                                           accepted_commands=accepted_commands)
                if self.started is None and self.takeoff.started is not None:
                    self.started = self.takeoff.started
                if output.state != 'MISSION_HANDOFF':
                    self.state, self.reason = output.state, output.reason
                    if self.state in self.TERMINAL:
                        # This isolated executive fences output on failure;
                        # actual PX4 fallback still needs separate validation.
                        return self.result({})
                    self.takeoff_guard.check(output, sample, now=now, stamp=timestamp_us,
                        state=self.takeoff.state, origin=self.takeoff.origin, height=self.takeoff.height,
                        switch_fresh=switch_fresh, mode_slot=mode_slot, kill_switch=kill_switch,
                        exclusive=sample.input_ownership_ok)
                    self.history.append((now, self.state))
                    return self.result(build_messages(output, timestamp_us))
                # The original vertical-only 0.3m departure/overshoot/ACK guards
                # were active through the final stable-hover sample above.
                self.handoff_at = now
                self.history.append((now, 'MISSION_HANDOFF'))

            if not fresh(sample.status_at, now, 1.5):
                raise ValueError('PX4 status stale')
            if sample.nav_state != FlightSupervisor.OFFBOARD or mode_slot != 6:
                return self.stop('Offboard ownership relinquished', 'HANDOVER')
            if (not switch_fresh or kill_switch != 3 or not sample.rc_valid
                    or not fresh(sample.rc_at, now, .5)):
                raise ValueError('RC/switch telemetry stale or invalid')
            if (not fresh(sample.local_at, now, .5) or not fresh(sample.flags_at, now, 2.)
                    or not fresh(sample.land_at, now, 2.) or sample.failsafe
                    or not FlightSupervisor.position_healthy(sample)):
                raise ValueError('PX4 estimator/land/EV health lost')
            touchdown = (sample.landed and scene.landed
                and self.sequence.descent.state in ('TERMINAL_DESCEND', 'TOUCHDOWN')
                and self.sequence.floor is not None and abs(scene.position[2]-self.sequence.floor) <= .03)
            if not sample.armed and not touchdown:
                raise ValueError('unexpected disarm before touchdown')
            if sample.reset_counters != self.takeoff.resets:
                raise ValueError('PX4 frame reset after takeoff')
            if now-self.started > 120.:
                raise ValueError('bounded offline sortie timeout')
            if not fresh(scene.at, now, .25) or not scene.localization_ok:
                raise ValueError('VIO/scene stale or invalid')
            paired = alignment.position(scene.position, vio_session, sample.reset_counters)
            # Commissioning discrepancy limit for this isolated integration;
            # NOT a measured VIO/EKF alignment uncertainty or flight clearance.
            if math.dist(paired, (sample.x, sample.y, sample.z)) > .10:
                raise ValueError('VIO/PX4 paired pose discrepancy')
            if (landing is not None and landing.armed != sample.armed
                    or scene.landed != sample.landed or scene.airborne == sample.landed):
                raise ValueError('conflicting flight/landing observations')
            output = self.sequence.step(now, scene, landing, timestamp_us=timestamp_us,
                vio_session=vio_session, reset_counters=sample.reset_counters,
                exclusive_writer_verified=sample.input_ownership_ok, yaw_rate_odom=0.)
            self.state = output['state']
            self.reason = output.get('reason', output.get('fault', ''))
            if 'vehicle_command' in output['messages']:
                raise ValueError('navigation must not issue ARM/mode/LAND commands')
            self.history.append((now, self.state))
            return self.result(output['messages'])
        except (ValueError, TypeError, IndexError, ArithmeticError) as exc:
            return self.stop(str(exc))


class SortieShadowWriter:
    """Single writer on a fixed synthetic namespace, no configurable live route.

The caller owns a domain-176 ROS node/executor. Failures latch and fence all
future batches. Stopping output does not prove physical braking or PX4 failsafe.
"""
    PREFIX = '/robocup/sortie_shadow/output/'

    def __init__(self, node, core):
        import os
        if os.environ.get('ROS_DOMAIN_ID') != '176' or type(core) is not SortieShadow:
            raise ValueError('dedicated offline core and domain 176 required')
        from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand
        self.node, self.core = node, core
        self.pubs = {name: node.create_publisher(cls, self.PREFIX+name, 10) for name, cls in (
            ('offboard_control_mode', OffboardControlMode), ('trajectory_setpoint', TrajectorySetpoint),
            ('vehicle_command', VehicleCommand))}
        self.counts = {name: 0 for name in self.pubs}
        self.fault = None
        self.last_stamp = None

    def owns(self):
        return (all(self.node.count_publishers(self.PREFIX+name) == 1 for name in self.pubs)
                and not any(name.startswith('/fmu/in/') for name, _ in self.node.get_topic_names_and_types()))

    def dispatch(self, result):
        if self.fault:
            return False
        try:
            if not self.owns():
                raise ValueError('shadow writer ownership conflict')
            messages = result['messages']
            if not messages:
                return True
            if self.core.state in self.core.TERMINAL:
                raise ValueError('terminal sortie cannot dispatch')
            if result['state'] != self.core.state or result.get('flight_authorized') is not False:
                raise ValueError('invalid shadow dispatch provenance')
            if not set(messages) <= set(self.pubs):
                raise ValueError('unknown output route')
            stamps = {int(msg.timestamp) for msg in messages.values()}
            if (len(stamps) != 1 or min(stamps) <= (self.last_stamp or 0)
                    or min(stamps) != self.core.last_stamp):
                raise ValueError('stale/replayed output batch')
            has_mode = 'offboard_control_mode' in messages
            if has_mode != ('trajectory_setpoint' in messages):
                raise ValueError('incomplete mode/setpoint batch')
            if has_mode:
                mode, sp = messages['offboard_control_mode'], messages['trajectory_setpoint']
                if not mode.position or any(getattr(mode, key, False) for key in
                    ('velocity', 'acceleration', 'attitude', 'body_rate', 'thrust_and_torque', 'direct_actuator')):
                    raise ValueError('unsupported Offboard mode')
                if not all(math.isfinite(v) for v in (sp.position[2], sp.yaw)):
                    raise ValueError('nonfinite height/yaw')
                origin = self.core.takeoff.origin
                if origin is None or not origin[2]-self.core.takeoff.height-1e-5 <= sp.position[2] <= origin[2]+1e-5:
                    raise ValueError('height setpoint outside sortie interval')
                if self.core.handoff_at is None:
                    if (not all(math.isfinite(v) for v in sp.position)
                            or math.dist(sp.position[:2], origin[:2]) > 1e-5
                            or not all(math.isnan(v) for v in sp.velocity)):
                        raise ValueError('takeoff dispatch must hold XY with position-only mode')
                elif (not all(math.isnan(v) for v in sp.position[:2])
                        or not math.isnan(sp.velocity[2])
                        or not all(math.isfinite(v) for v in sp.velocity[:2])
                        or math.hypot(*sp.velocity[:2]) > .15+2e-8):
                    # Two float32 ULP-scale rounding allowance only; no physical
                    # speed limit change from serialization representation.
                    raise ValueError('invalid mixed navigation setpoint')
            if 'vehicle_command' in messages:
                cmd = messages['vehicle_command']
                expected = {'OFFBOARD': 176, 'ARM': 400}.get(self.core.state)
                if (self.core.handoff_at is not None or cmd.command != expected
                        or not has_mode or (cmd.command == 400 and cmd.param1 != 1.)):
                    raise ValueError('command forbidden in this sortie phase')
            self.last_stamp = min(stamps)
            for name, msg in messages.items():
                self.pubs[name].publish(msg)
                self.counts[name] += 1
            return True
        except Exception as exc:
            self.fault = str(exc)
            self.core.stop('dispatch fenced: '+self.fault)
            return False
