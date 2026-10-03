"""Container camera/VIO supervisor with bounded lifetime and status relay."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import math
from bench_session_deadline import DeadlineAlarm
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
from std_msgs.msg import String

parser = argparse.ArgumentParser()
parser.add_argument('--duration', type=int, default=65)
parser.add_argument('--control-file', type=Path)
parser.add_argument('--visual-only-diagnostic', action='store_true')
parser.add_argument('--imu-copy-diagnostic', action='store_true')
parser.add_argument('--stop-at-monotonic', type=float)
args = parser.parse_args()
if not 10 <= args.duration <= 900:
    parser.error('duration outside 10..900')
if args.visual_only_diagnostic and args.imu_copy_diagnostic:
    parser.error('choose only one IMU diagnostic mode')
if args.stop_at_monotonic is not None and (not math.isfinite(args.stop_at_monotonic)
        or not 0 < args.stop_at_monotonic-time.monotonic() <= 60):
    parser.error('invalid or expired absolute camera deadline')
rclpy.init()
node = Node('alignment_status_relay')
pub = node.create_publisher(String, '/robocup/alignment/tracking', 10)
def status(msg):
    pub.publish(String(data=json.dumps({'vo_state': int(msg.vo_state),
        'stamp_ns': msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,
        'callback_s': float(msg.node_callback_execution_time),
        'track_s': float(msg.track_execution_time),
        'track_mean_s': float(msg.track_execution_time_mean),
        'track_max_s': float(msg.track_execution_time_max)})))
node.create_subscription(VisualSlamStatus, '/visual_slam/status', status, qos_profile_sensor_data)
root = Path(__file__).resolve().parents[1]
launch_cmd = ['ros2', 'launch', str(root/'launch/perception.launch.py')]
# Always pass the fusion flag so a launch-default change cannot silently
# re-enable D435i IMU inside cuVSLAM. Copy-mode remains the only fused path
# started from this supervisor.
if args.imu_copy_diagnostic:
    launch_cmd.append('imu_fusion:=true')
    launch_cmd.append('imu_unite_method:=1')
else:
    launch_cmd.append('imu_fusion:=false')
proc = None
alarm = DeadlineAlarm(math.inf if args.stop_at_monotonic is None else args.stop_at_monotonic)
try:
    alarm.arm()  # Refuse late startup; the host and container share CLOCK_MONOTONIC.
    proc = subprocess.Popen(launch_cmd, start_new_session=True)
    end = time.monotonic()+args.duration
    if args.stop_at_monotonic is not None:
        end = min(end, args.stop_at_monotonic)
    while time.monotonic() < end and proc.poll() is None:
        rclpy.spin_once(node, timeout_sec=.1)
        if args.control_file and args.control_file.exists():
            if json.loads(args.control_file.read_text()).get('stop'):
                break
finally:
    alarm.cancel()
    if proc is not None and proc.poll() is None:
        os.killpg(proc.pid, signal.SIGINT)
        try:
            proc.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGTERM)
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)  # Owned launch group only.
                proc.wait(timeout=2)
    node.destroy_node()
    rclpy.try_shutdown()
