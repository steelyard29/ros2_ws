#!/usr/bin/env python3
"""Enable D435i IMU only after infrared images are flowing.

Hard constraint: never start infra/depth and Motion Module together.
Starting Depth/IR and IMU in one step drops combined IMU. No EEPROM, no PX4.
"""
from __future__ import annotations

import argparse
import math
import time
from camera_stream_gate import StreamWindow, parameter_result_ok

import rclpy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu


def bool_param(name: str, value: bool) -> Parameter:
    return Parameter(name=name, value=ParameterValue(
        type=ParameterType.PARAMETER_BOOL, bool_value=value))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--camera-node', default='/camera/camera')
    parser.add_argument('--wait-topic', default='/camera/camera/infra1/image_rect_raw')
    parser.add_argument('--min-frames', type=int, default=15)
    parser.add_argument('--timeout', type=float, default=25.0)
    parser.add_argument('--settle', type=float, default=1.0)
    parser.add_argument('--request-timeout', type=float, default=5.0)
    parser.add_argument('--imu-timeout', type=float, default=15.0)
    parser.add_argument('--imu-topic', default='/camera/camera/imu')
    parser.add_argument('--enable-gyro', action='store_true')
    parser.add_argument('--enable-accel', action='store_true')
    args = parser.parse_args()
    if (not all(math.isfinite(v) and 0 < v <= 60 for v in
                (args.timeout, args.settle, args.request_timeout, args.imu_timeout))
            or not 2 <= args.min_frames <= 1000 or args.settle >= args.timeout):
        parser.error('invalid bounded startup timing/frame limits')
    if not (args.enable_gyro or args.enable_accel):
        return 0
    if not (args.enable_gyro and args.enable_accel):
        parser.error('combined IMU verification requires both gyro and accel')

    rclpy.init()
    node = rclpy.create_node('realsense_ir_then_imu', start_parameter_services=False)
    ir = StreamWindow(args.min_frames, args.settle, .2)
    imu = StreamWindow(200, 2., .1)
    imu_epoch = None

    def record(gate, msg, finite=True):
        stamp = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        age = (node.get_clock().now().nanoseconds-stamp)/1e9
        gate.observe(stamp, time.monotonic(), age, finite)

    def on_image(msg):
        record(ir, msg)

    def on_imu(msg):
        stamp = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        if imu_epoch is None or stamp < imu_epoch:
            return
        a, w = msg.linear_acceleration, msg.angular_velocity
        record(imu, msg, all(math.isfinite(v) for v in (a.x, a.y, a.z, w.x, w.y, w.z)))

    node.create_subscription(Image, args.wait_topic, on_image, qos_profile_sensor_data)
    node.create_subscription(Imu, args.imu_topic, on_imu, qos_profile_sensor_data)
    deadline = time.monotonic() + args.timeout
    ir_ready = False
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        now = time.monotonic()
        if now < deadline and ir.ready(now):
            ir_ready = True
            break
    if not ir_ready:
        msg = f'IR stability timeout: consecutive={ir.count}, rejected={ir.rejected}; IMU not enabled'
        print(msg, flush=True)
        node.get_logger().error(msg)
        node.destroy_node()
        rclpy.try_shutdown()
        return 1

    client = node.create_client(SetParameters, args.camera_node + '/set_parameters')
    if not client.wait_for_service(timeout_sec=5.0):
        print('Camera set_parameters service missing', flush=True)
        node.destroy_node()
        rclpy.try_shutdown()
        return 1
    if not ir.ready(time.monotonic()):
        print('IR no longer fresh before parameter request; IMU not enabled', flush=True)
        node.destroy_node()
        rclpy.try_shutdown()
        return 1
    request = SetParameters.Request()
    if args.enable_gyro:
        request.parameters.append(bool_param('enable_gyro', True))
    if args.enable_accel:
        request.parameters.append(bool_param('enable_accel', True))
    future = client.call_async(request)
    deadline = time.monotonic()+args.request_timeout
    while rclpy.ok() and not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not future.done() or time.monotonic() >= deadline:
        future.cancel()
        print('IMU parameter request timeout; camera state unknown, no automatic retry', flush=True)
        node.destroy_node()
        rclpy.try_shutdown()
        return 1
    try:
        result = future.result()
    except Exception as exc:
        print(f'IMU parameter request exception: {exc}', flush=True)
        node.destroy_node()
        rclpy.try_shutdown()
        return 1
    if not parameter_result_ok(result, len(request.parameters)):
        print(f'Failed to enable IMU: {result}', flush=True)
        node.destroy_node()
        rclpy.try_shutdown()
        return 1
    imu_epoch = node.get_clock().now().nanoseconds
    print(f'IMU enable acknowledged after {ir.total} IR frames; verifying combined messages', flush=True)
    deadline = time.monotonic()+args.imu_timeout
    verified = False
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.05)
        now = time.monotonic()
        if now < deadline and imu.ready(now) and ir.ready(now):
            verified = True
            break
    if not verified:
        print(f'Combined IMU/IR verification timeout: imu_frames={imu.count}, rejected={imu.rejected}', flush=True)
        node.destroy_node()
        rclpy.try_shutdown()
        return 1
    msg = (f'IMU startup verified: {imu.count} consecutive finite combined frames, '
           f'span={(imu.last_stamp-imu.first_stamp)/1e9:.3f}s; startup only, not calibration')
    print(msg, flush=True)
    node.get_logger().info(msg)
    # One-shot startup check; ongoing health is the VIO/EV supervisor's job.
    node.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
