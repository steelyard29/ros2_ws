#!/usr/bin/env python3
"""Bounded, subscription-only AUX evidence collector. Never supplies switches
to a controller. AUX values are raw observations, NOT certified mode/kill.
"""
import argparse
from collections import Counter
import json
import math
import os
from pathlib import Path
import time
import uuid


class AuxEvidence:
    def __init__(self):
        self.last_stamp = self.last_sample = None
        self.latest = None
        self.counts = Counter()
        self.fault = None
        self.samples = []

    def receive(self, stamp, sample, age, now, valid, source, aux1, aux2):
        self.counts['received'] += 1
        record = dict(timestamp=stamp, timestamp_sample=sample, receipt=now,
                      publication_age_s=age, valid=valid, data_source=source,
                      aux1=aux1 if math.isfinite(aux1) else None,
                      aux2=aux2 if math.isfinite(aux2) else None)
        self.latest = record
        reason = None
        if not valid or source != 1:
            reason = 'invalid RC or non-RC source'
        elif (stamp <= 0 or sample <= 0 or not math.isfinite(age)
              or not -.05 <= age <= .5 or not 0 <= stamp-sample <= 500_000
              or age+(stamp-sample)/1e6 > .5):
            reason = 'invalid/stale source time'
        elif not all(math.isfinite(v) and -1 <= v <= 1 for v in (aux1, aux2)):
            reason = 'invalid AUX value'
        elif self.last_stamp is not None and (stamp < self.last_stamp or sample < self.last_sample):
            reason = 'source clock reset'
            self.fault = reason
        elif self.last_stamp is not None and (stamp == self.last_stamp or sample == self.last_sample):
            reason = 'duplicate source sample'
        if reason:
            self.counts[reason] += 1
        else:
            self.counts['fresh_rc_samples'] += 1
            self.last_stamp, self.last_sample = stamp, sample
        record['rejection'] = reason
        self.samples.append(record)
        return reason is None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=int, default=20)
    parser.add_argument('--label', choices=('baseline', 'position', 'kill', 'offboard', 'restored', 'sequence'),
                        default='baseline', help='operator label only, not proof of physical state')
    args = parser.parse_args()
    if not 10 <= args.duration <= 180:
        parser.error('duration must be 10..180 seconds')
    os.environ.update(ROS_DOMAIN_ID='0', ROS_LOCALHOST_ONLY='0',
                      FASTRTPS_DEFAULT_PROFILES_FILE='/home/cfly/ros2_ws/config/fastdds_bridge.xml')
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data as qos
    from px4_msgs.msg import ManualControlSetpoint, VehicleStatus
    rclpy.init(args=[])
    node = Node('robocup_aux_readonly', enable_rosout=False, start_parameter_services=False,
                use_global_arguments=False)
    evidence = AuxEvidence()
    status = {}
    ever_armed = False
    graph_fault = None
    started = time.monotonic()

    def rc(m):
        evidence.receive(int(m.timestamp), int(m.timestamp_sample),
                         (node.get_clock().now().nanoseconds/1000-m.timestamp)/1e6,
                         time.monotonic(), bool(m.valid), int(m.data_source), float(m.aux1), float(m.aux2))

    def vehicle(m):
        nonlocal ever_armed
        now = time.monotonic()
        age = (node.get_clock().now().nanoseconds/1000-m.timestamp)/1e6
        ever_armed |= m.arming_state == 2
        status.update(receipt=now, age_s=age, arming_state=int(m.arming_state),
                      nav_state=int(m.nav_state), preflight=bool(m.pre_flight_checks_pass),
                      gcs_connection_lost=bool(m.gcs_connection_lost))

    node.create_subscription(ManualControlSetpoint, '/fmu/out/manual_control_setpoint', rc, qos)
    node.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1', vehicle, qos)
    audit_at = log_at = -1.
    error = None
    try:
        while time.monotonic()-started < args.duration and not ever_armed and not graph_fault:
            rclpy.spin_once(node, timeout_sec=.05)
            now = time.monotonic()
            if now-audit_at >= .5:
                audit_at = now
                if any(t.startswith('/fmu/in/') and node.count_publishers(t)
                       for t, _ in node.get_topic_names_and_types()):
                    graph_fault = 'active flight-input publisher; stop observation'
                for t in ('/fmu/out/manual_control_setpoint', '/fmu/out/vehicle_status_v1'):
                    if node.count_publishers(t) > 1:
                        graph_fault = 'duplicate telemetry publishers'
            if now-log_at >= 2:
                log_at = now
                print(json.dumps({'elapsed': round(now-started, 1), 'counts': dict(evidence.counts),
                                  'latest': evidence.latest, 'status': status}), flush=True)
    except KeyboardInterrupt:
        error = 'operator interrupted'
    except Exception as exc:
        error = str(exc)
    finally:
        now = time.monotonic()
        last_good = next((s for s in reversed(evidence.samples) if s['rejection'] is None), None)
        healthy = (last_good is not None and now-last_good['receipt'] <= .5
                   and evidence.counts['fresh_rc_samples'] >= 20 and status.get('arming_state') == 1
                   and now-status.get('receipt', -1) <= 1.5 and -.05 <= status.get('age_s', 999) <= .5
                   and not ever_armed and not graph_fault and not error and not evidence.fault)
        report = dict(subscription_only=True, hardware_commands_sent=0, label=args.label,
                      telemetry_window_healthy=bool(healthy), switch_mapping_verified=False,
                      flight_approved=False, counts=dict(evidence.counts), latest=evidence.latest,
                      status=status, ever_armed=ever_armed, graph_fault=graph_fault,
                      source_fault=evidence.fault, error=error, duration_s=now-started)
        out = Path(__file__).resolve().parents[1]/'evidence'/(
            time.strftime('aux_readonly_%Y%m%d_%H%M%S_')+uuid.uuid4().hex[:6])
        out.mkdir(parents=True, exist_ok=False)
        (out/'report.json').write_text(json.dumps(report, indent=2)+'\n')
        (out/'samples.json').write_text(json.dumps(evidence.samples)+'\n')
        print(json.dumps({'evidence': str(out), **report}, indent=2), flush=True)
        node.destroy_node()
        rclpy.try_shutdown()
    return 0 if healthy else 1


if __name__ == '__main__':
    raise SystemExit(main())
