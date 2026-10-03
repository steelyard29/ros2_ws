#!/usr/bin/env python3
"""DDS serialization/handshake tests with a synthetic plant, never real PX4.

Hardcoded domain 177, localhost, and /robocup/flight_shadow/test/* topics.
Agent can remain up: this script has no /fmu topic publishers at all.
"""
import json
import math
import os
from pathlib import Path
import time

os.environ['ROS_DOMAIN_ID'] = '177'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ.pop('FASTRTPS_DEFAULT_PROFILES_FILE', None)
import rclpy
from rclpy.node import Node
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand
from flight_messages import build_messages
from flight_supervisor import FlightSupervisor, FlightSample


def scenario(name):
    node = Node('flight_contract_'+name.replace('-', '_'), enable_rosout=False,
                start_parameter_services=False)
    prefix = '/robocup/flight_shadow/test/'+name.replace('-', '_')+'/'
    pubs = {key: node.create_publisher(cls, prefix+key, 10) for key, cls in (
        ('offboard_control_mode', OffboardControlMode),
        ('trajectory_setpoint', TrajectorySetpoint), ('vehicle_command', VehicleCommand))}
    c = FlightSupervisor()
    plant = {'armed': False, 'mode': 2, 'z': .7, 'sp': .7,
             'land': False, 'heartbeat': -1.0, 'accepted': [], 'forced_disarm': False}
    sim = [0.0]
    received = dict.fromkeys(pubs, 0)
    def heartbeat(m):
        received['offboard_control_mode'] += 1
        if name != 'link-loss' or sim[0] < 5:
            plant['heartbeat'] = sim[0]
    def setpoint(m):
        received['trajectory_setpoint'] += 1
        if name == 'link-loss' and sim[0] >= 5:
            return
        assert all(math.isnan(v) for v in m.velocity)
        assert abs(m.position[0]-2) < 1e-6 and abs(m.position[1]+3) < 1e-6
        assert abs(m.yaw-.8) < 1e-6
        plant['sp'] = float(m.position[2])
    def command(m):
        received['vehicle_command'] += 1
        if name == 'link-loss' and sim[0] >= 5:
            return
        plant['accepted'].append(int(m.command))
        if m.command == VehicleCommand.VEHICLE_CMD_DO_SET_MODE:
            assert m.param1 == 1 and m.param2 == 6
            if name != 'mode-rejected':
                plant['mode'] = 14
        elif m.command == VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM:
            plant['forced_disarm'] |= m.param1 == 0 or m.param2 == 21196
            assert m.param1 == 1 and m.param2 == 0
            if plant['mode'] == 14:
                plant['armed'] = True
        elif m.command == VehicleCommand.VEHICLE_CMD_NAV_LAND:
            assert all(math.isnan(v) for v in (m.param4, m.param5, m.param6, m.param7))
            plant['land'], plant['mode'] = True, 18
    for cls, key, callback in ((OffboardControlMode, 'offboard_control_mode', heartbeat),
                               (TrajectorySetpoint, 'trajectory_setpoint', setpoint),
                               (VehicleCommand, 'vehicle_command', command)):
        node.create_subscription(cls, prefix+key, callback, 10)
    discovery_end = time.monotonic()+3
    while not all(node.count_subscribers(prefix+k) for k in pubs):
        if time.monotonic() > discovery_end:
            raise RuntimeError('isolated discovery timeout')
        rclpy.spin_once(node, timeout_sec=.05)
    manual_seen = False
    for tick in range(1400):
        now = tick*.05
        sim[0] = now
        loss = name == 'link-loss' and now >= 5
        if plant['armed'] and now-plant['heartbeat'] > 1:
            # Explicitly synthetic PX4 fallback, not a real parameter test.
            plant['land'], plant['mode'] = True, 18
        if name == 'manual-takeover' and now >= 5:
            plant['mode'], manual_seen = 2, True
        target = .7 if plant['land'] else plant['sp']
        if plant['armed']:
            plant['z'] += max(-.01, min(.01, target-plant['z']))
        if plant['land'] and abs(plant['z']-.7) < .005:
            plant['armed'] = False
        s = FlightSample(status_at=3 if loss else now, local_at=now, flags_at=now,
                         land_at=now, rc_at=now, armed=plant['armed'],
                         landed=abs(plant['z']-.7) < .005, nav_state=plant['mode'],
                         preflight_ok=True, rc_valid=True, manual_takeover=manual_seen,
                         input_ownership_ok=all(node.count_publishers(prefix+k) == 1 for k in pubs),
                         ev_ok=not (name == 'vision-loss' and now >= 5),
                         ev_position=True, ev_height=True, ev_yaw=True, baro_height=True,
                         xy_valid=True, z_valid=True, v_xy_valid=True, v_z_valid=True,
                         x=2, y=-3, z=plant['z'], heading=.8, vx=0, vy=0, vz=0,
                         reset_counters=(0, 0, 0, 0, 0))
        output = c.step(now, s, start=tick == 0)
        for key, msg in build_messages(output, node.get_clock().now().nanoseconds//1000).items():
            pubs[key].publish(msg)
        for _ in range(4):
            rclpy.spin_once(node, timeout_sec=.002)
        if c.state in c.TERMINAL:
            break
        if loss and c.state == 'ABORT' and not plant['armed']:
            break  # Controller cannot claim landing without fresh status.
    expected = {'nominal': 'DONE', 'vision-loss': 'STOPPED', 'link-loss': 'ABORT',
                'manual-takeover': 'HANDOVER', 'mode-rejected': 'STOPPED'}[name]
    result = {'scenario': name, 'passed': c.state == expected and not plant['forced_disarm'],
              'final_state': c.state, 'reason': c.reason, 'received_messages': received,
              'synthetic_plant_armed': plant['armed'], 'states': c.history,
              'real_aircraft': False, 'real_px4_sitl': False, 'fmu_input_publishers': 0}
    if name == 'mode-rejected':
        result['passed'] &= VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM not in plant['accepted']
    node.destroy_node()
    return result


def main():
    rclpy.init()
    try:
        results = [scenario(name) for name in
                   ('nominal', 'vision-loss', 'link-loss', 'manual-takeover', 'mode-rejected')]
    finally:
        rclpy.try_shutdown()
    out = Path(__file__).resolve().parents[1]/'evidence'/time.strftime('flight_dds_%Y%m%d_%H%M%S')
    out.mkdir(parents=True, exist_ok=False)
    report = {'passed': all(r['passed'] for r in results), 'flight_validation': False,
              'ros_domain': 177, 'synthetic_plant': True, 'scenarios': results}
    (out/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({'evidence': str(out), **report}, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
