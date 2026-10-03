"""Prop-off kill-stop qualification. Pure core; never injects a fault.

Uses the existing bounded Position prestream, but forbids every mode command.
Only a new validated AUX kill edge in the event window can qualify. This
certifies companion output cessation, NOT motor cutoff or in-air PX4 failsafe.
"""
from handshake_runtime_core import HandshakeRuntimeCore


class KillBenchCore(HandshakeRuntimeCore):
    fault_bench = True
    STREAMS = ('vehicle_visual_odometry', 'offboard_control_mode', 'trajectory_setpoint')

    def __init__(self, started, aux_params, switch_wait_s=60):
        super().__init__(started, aux_params=aux_params, staged=True,
                         require_gcs=True, switch_wait_s=switch_wait_s)
        self.window_at = None
        self.event = None
        self.terminal_at = None
        self.last_outputs = {}
        self.output_counts = {}
        self.failure = None
        self.restore_since = None
        self.clear_prompted = False
        self.last_observe = None

    def fail(self, reason):
        self.failure = self.failure or reason

    def trip(self, reason):
        # A kill legitimately trips the EV gate. Other faults cannot be
        # laundered into a successful kill test, even after the first latch.
        expected = False
        if self.event and reason.startswith('handshake EV gate: '):
            blockers = set(reason.split(': ', 1)[1].split(', '))
            expected = bool(blockers) and blockers <= {'switches', 'session'}
        if not expected:
            self.fail(reason)
        super().trip(reason)

    def switches(self, now, mode_slot, kill_switch):
        if (self.handshake.state not in ('WAIT', *self.handshake.TERMINAL)
                and mode_slot != self.handshake.baseline_slot):
            self.trip('kill bench requires unchanged Position slot; no Offboard request')
        super().switches(now, mode_slot, kill_switch)

    def aux_message(self, stamp, sample_stamp, now, age_s, valid, source, aux1, aux2):
        prior_kill = self.kill_switch
        eligible = (self.handshake.state == 'AWAIT_SWITCH' and self.window_at is not None
                    and 0 <= now-self.window_at < self.handshake.switch_wait_s
                    and prior_kill == 3 and not self.fault and not self.ev.inhibit
                    and not self.failure and not self.ownership_fault
                    and all(0 <= now-self.last_outputs.get(k, {}).get('at', -1e9) <= .15
                            for k in self.STREAMS))
        accepted = super().aux_message(stamp, sample_stamp, now, age_s, valid, source, aux1, aux2)
        if accepted and self.kill_switch == 1 and self.event is None:
            if eligible and not self.failure and self.mode_slot == self.handshake.baseline_slot:
                self.event = {'kind': 'kill', 'at': now, 'timestamp': stamp,
                              'timestamp_sample': sample_stamp,
                              'publication_age_s': age_s,
                              'sample_age_s': age_s+(stamp-sample_stamp)/1e6,
                              'source': 'validated_derived_aux_mirror',
                              'mode_slot': self.mode_slot, 'kill_switch': self.kill_switch,
                              'last_outputs_before_event': dict(self.last_outputs)}
            else:
                self.fail('kill was not a fresh edge in the active output window')
        if accepted and self.event and self.kill_switch == 3:
            if self.terminal_at is None or now-self.terminal_at < 2.:
                self.fail('kill cleared before two seconds of terminal silence')
        return accepted

    def tick(self, now, start=False):
        output = super().tick(now, start=start)
        if output.command:
            self.trip('fault bench forbids all commands')
            output = self.handshake.stop('ABORT', self.failure)
            self.ev_stopped = True
        if output.state == 'AWAIT_SWITCH' and self.window_at is None:
            self.window_at = now
        if output.state in self.handshake.TERMINAL and self.terminal_at is None:
            self.terminal_at = now
            if output.state != 'KILLED' or self.event is None:
                self.fail('expected observed kill, not '+output.state)
            elif not 0 <= now-self.event['at'] <= .1:
                self.fail('kill to terminal latency exceeds 100 ms')
        return output

    def note_output(self, name, now, stamp):
        # Called only AFTER publish() succeeds, for EV as well as controls.
        self.output_counts[name] = self.output_counts.get(name, 0)+1
        self.last_outputs[name] = {'at': now, 'timestamp': stamp}
        if name not in self.STREAMS:
            self.fail('forbidden output: '+name)
        if self.event is not None or self.terminal_at is not None:
            self.fail('output resumed after kill event or terminal')

    def operator_action(self, state):
        if state == 'AWAIT_SWITCH':
            return 'KILL TEST WINDOW: keep Position/disarmed/props removed; assert only kill now'
        if state == 'KILLED':
            return 'outputs stopped; keep kill asserted, Position/disarmed; await clear instruction'
        return self.handshake.operator_action(state)

    def observe_restore(self, now, restored, dispatch_fault=None):
        if dispatch_fault or self.ownership_fault or self.command_fault or self.sample.armed:
            self.fail('dispatch/ownership/command/arming fault')
        if self.terminal_at is None:
            return False
        previous = self.terminal_at if self.last_observe is None else self.last_observe
        if not 0 <= now-previous <= .3:
            self.fail('terminal observation clock/gap invalid')
        self.last_observe = now
        if (not self.gcs_connected or not self.controller.fresh(self.gcs_at, now, 1.5)
                or not self.controller.fresh(self.sample.status_at, now, 1.5)
                or not self.switch_fresh(now)):
            self.fail('terminal safety telemetry unavailable')
        self.restore_since = (now if self.restore_since is None else self.restore_since) if restored else None
        return (self.restore_since is not None and now-self.restore_since >= 2.
                and now-self.terminal_at >= 2.)

    def clear_ready(self, now):
        return (self.terminal_at is not None and now-self.terminal_at >= 2.
                and self.ev_stopped and not self.sample.armed
                and self.controller.fresh(self.sample.status_at, now, 1.5)
                and all(item['at'] <= self.terminal_at for item in self.last_outputs.values()))

    def qualification(self, now):
        restored = self.restore_since is not None and now-self.restore_since >= 2.
        fresh_observation = self.last_observe is not None and 0 <= now-self.last_observe <= .3
        passed = (self.event is not None and self.terminal_at is not None
                  and self.handshake.state == 'KILLED' and self.ev_stopped
                  and now-self.terminal_at >= 2. and restored and fresh_observation and not self.failure
                  and all(self.output_counts.get(k, 0) > 0 for k in self.STREAMS)
                  and self.output_counts.get('vehicle_command', 0) == 0)
        return {'scenario': 'kill', 'passed': bool(passed), 'failure': self.failure,
                'restore_observation_complete': restored,
                'fresh_terminal_observation': fresh_observation,
                'event': self.event, 'window_at': self.window_at,
                'terminal_at': self.terminal_at, 'restore_since': self.restore_since,
                'last_outputs': dict(self.last_outputs), 'output_counts': dict(self.output_counts),
                'motor_cutoff_verified': False, 'px4_airborne_failsafe_verified': False,
                'flight_approved': False}
