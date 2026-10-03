"""Actual DDS positive control and writer conflict/failure fencing, no devices."""
import os
os.environ['ROS_DOMAIN_ID'] = '176'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
import json
from pathlib import Path
import time
import rclpy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint
from test_sortie_shadow import Rig
from sortie_shadow import SortieShadowWriter


def run(node, scenario):
    rig = Rig()
    initial = rig.navigate()
    writer = SortieShadowWriter(node, rig.core)
    received = {'mode': 0, 'setpoint': 0}
    def count(key, msg):
        received[key] += 1
    subs = [node.create_subscription(cls, writer.PREFIX+name,
        lambda m, key=key: count(key, m), 10) for key, cls, name in (
            ('mode', OffboardControlMode, 'offboard_control_mode'),
            ('setpoint', TrajectorySetpoint, 'trajectory_setpoint'))]
    extra = None
    original_pub = writer.pubs['trajectory_setpoint']
    def spin(seconds):
        until = time.monotonic()+seconds
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.02)
    try:
        spin(.5)
        positive = writer.dispatch(initial)
        spin(.3)
        positive &= received['mode'] > 0 and received['setpoint'] > 0
        if scenario == 'duplicate-writer':
            extra = node.create_publisher(TrajectorySetpoint, writer.PREFIX+'trajectory_setpoint', 10)
            spin(.4)
            observed_conflict = node.count_publishers(writer.PREFIX+'trajectory_setpoint') == 2
            rejected = not writer.dispatch(rig.tick())
            node.destroy_publisher(extra)
            extra = None
        elif scenario == 'publisher-exception':
            class Broken:
                def publish(self, msg):
                    raise RuntimeError('synthetic publisher exception')
            writer.pubs['trajectory_setpoint'] = Broken()
            observed_conflict = True
            rejected = not writer.dispatch(rig.tick())
            writer.pubs['trajectory_setpoint'] = original_pub
        else:
            observed_conflict = True
            rejected = not writer.dispatch(initial)
        # Drain any already-published pre-fault DDS messages; they cannot be
        # retracted. Verify no NEW publication once the local fence is latched.
        spin(.3)
        counts, before = dict(writer.counts), dict(received)
        for _ in range(8):
            writer.dispatch(rig.tick())
            spin(.05)
        quiet = counts == writer.counts and before == received
        no_real = not any(n.startswith('/fmu/in/') for n, _ in node.get_topic_names_and_types())
        return dict(scenario=scenario, passed=bool(positive and observed_conflict and rejected and quiet
                    and rig.core.state == 'ABORT' and no_real), positive_control=bool(positive),
                    publication_fenced=quiet, fault=writer.fault, received=received,
                    final_state=rig.core.state, no_flight_inputs=no_real,
                    physical_stop_verified=False)
    finally:
        if extra is not None:
            node.destroy_publisher(extra)
        writer.pubs['trajectory_setpoint'] = original_pub
        for pub in writer.pubs.values():
            node.destroy_publisher(pub)
        for sub in subs:
            node.destroy_subscription(sub)
        spin(.3)


def main():
    rclpy.init()
    node = rclpy.create_node('sortie_writer_dds_check')
    results = []
    try:
        for _ in range(20):
            rclpy.spin_once(node, timeout_sec=.05)
        others = [name for name in node.get_node_names() if name != node.get_name()]
        if others:
            raise RuntimeError('domain 176 in use: '+str(others))
        for scenario in ('duplicate-writer', 'publisher-exception', 'replayed-batch'):
            results.append(run(node, scenario))
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    out = Path(__file__).resolve().parents[1]/'evidence'/time.strftime('sortie_writer_dds_%Y%m%d_%H%M%S')
    out.mkdir(exist_ok=False)
    report = dict(passed=all(r['passed'] for r in results) and len(results) == 3,
                  scenarios=results, flight_validation=False, px4_sitl=False)
    (out/'report.json').write_text(json.dumps(report, indent=2))
    print(out)
    print(json.dumps(report, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
