#!/usr/bin/env python3
"""实时漂移诊断监控 - 1Hz 刷新关键指标"""
import math, sys, time, os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import (VehicleLocalPosition, VehicleAttitude, ActuatorMotors,
                           ActuatorOutputs, VehicleStatus, EstimatorStatusFlags,
                           VehicleControlMode)
from geometry_msgs.msg import PoseStamped

OUT = os.environ.get("DRIFT_LOG", "/tmp/drift_monitor.log")

class DriftMonitor(Node):
    def __init__(self):
        super().__init__('drift_monitor')
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=1)

        self.lp = {}; self.att = {}; self.motors = []; self.outputs = []
        self.pose = {}; self.eflags = {}; self.mode_str = ""
        self.armed = False; self.in_offboard = False
        self.lp_ts = 0; self.att_ts = 0; self.motor_ts = 0
        self.pose_ts = 0; self.eflags_ts = 0; self.mode_ts = 0

        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self._lp_cb, qos)
        self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self._att_cb, qos)
        self.create_subscription(ActuatorMotors, '/fmu/out/actuator_motors', self._motor_cb, qos)
        self.create_subscription(ActuatorOutputs, '/fmu/out/actuator_outputs', self._out_cb, qos)
        self.create_subscription(PoseStamped, '/uav/state/pose', self._pose_cb, 10)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1', self._status_cb, qos)
        self.create_subscription(EstimatorStatusFlags, '/fmu/out/estimator_status_flags', self._eflags_cb, qos)
        self.create_subscription(VehicleControlMode, '/fmu/out/vehicle_control_mode', self._mode_cb, qos)
        self.t0 = time.time()
        self.line_count = 0
        self.create_timer(0.5, self._tick)

    def _fmt(self, v): return f"{v:7.3f}" if math.isfinite(v) else "   n/a  "

    def _lp_cb(self, msg):
        self.lp = {'x': msg.x, 'y': msg.y, 'z': msg.z,
                   'vx': msg.vx, 'vy': msg.vy, 'vz': msg.vz,
                   'dist_bottom': msg.dist_bottom, 'dist_bottom_valid': msg.dist_bottom_valid,
                   'xy_valid': msg.xy_valid, 'z_valid': msg.z_valid, 'v_xy_valid': msg.v_xy_valid}
        self.lp_ts = time.time()

    def _att_cb(self, msg):
        q = msg.q
        if len(q) >= 4 and not any(math.isnan(float(v)) for v in q):
            w, x, y, z = (float(q[0]), float(q[1]), float(q[2]), float(q[3]))
            self.att = {'roll': math.degrees(math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))),
                        'pitch': math.degrees(math.asin(max(-1, min(1, 2*(w*y-z*x))))),
                        'yaw': math.degrees(math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z)))}
        self.att_ts = time.time()

    def _motor_cb(self, msg):
        self.motors = list(msg.control[:8]) if msg.control else []
        self.motor_ts = time.time()

    def _out_cb(self, msg):
        self.outputs = list(msg.output[:4]) if msg.output else []

    def _pose_cb(self, msg):
        self.pose = {'x': msg.pose.position.x, 'y': msg.pose.position.y, 'z': msg.pose.position.z}
        self.pose_ts = time.time()

    def _status_cb(self, msg):
        self.armed = msg.arming_state == 2

    def _eflags_cb(self, msg):
        self.eflags = {'cs_rng_hgt': msg.cs_rng_hgt, 'cs_ev_pos': msg.cs_ev_pos,
                       'cs_ev_hgt': msg.cs_ev_hgt, 'cs_ev_vel': msg.cs_ev_vel,
                       'cs_ev_yaw': msg.cs_ev_yaw,
                       'cs_gnss_hgt': msg.cs_gnss_hgt, 'cs_gnss_pos': msg.cs_gnss_pos}
        self.eflags_ts = time.time()

    def _mode_cb(self, msg):
        self.in_offboard = bool(msg.flag_control_offboard_enabled)
        self.mode_ts = time.time()

    def _tick(self):
        now = time.time()
        elapsed = now - self.t0
        self.line_count += 1

        # Stale checks
        s_lp = (now - self.lp_ts) > 1.0
        s_att = (now - self.att_ts) > 1.0
        s_motor = (now - self.motor_ts) > 1.0
        s_pose = (now - self.pose_ts) > 1.0
        s_eflags = (now - self.eflags_ts) > 3.0

        # Build motor string
        if self.motors and len(self.motors) >= 4:
            m = self.motors[:4]
            mstr = f"M1={m[0]:6.4f} M2={m[1]:6.4f} M3={m[2]:6.4f} M4={m[3]:6.4f}"
            m_avg = sum(m) / 4
            m_range = max(m) - min(m)
        else:
            mstr = "no_motor_data"
            m_avg = 0
            m_range = 0

        # Build PWM string
        if self.outputs and len(self.outputs) == 4:
            pstr = f"PWM1={self.outputs[0]:4.0f} PWM2={self.outputs[1]:4.0f} PWM3={self.outputs[2]:4.0f} PWM4={self.outputs[3]:4.0f}"
        else:
            pstr = "no_pwm_data"

        # EKF flags string
        if not s_eflags and self.eflags:
            ef = self.eflags
            ekf_str = (
                f"rng_hgt={ef['cs_rng_hgt']} ev_pos={ef['cs_ev_pos']} "
                f"ev_vel={ef['cs_ev_vel']} ev_yaw={ef.get('cs_ev_yaw')} "
                f"ev_hgt={ef['cs_ev_hgt']}"
            )
        else:
            ekf_str = "stale"

        # PX4 valid flags
        if not s_lp and self.lp:
            vf = f"xy_valid={self.lp.get('xy_valid',0)} z_valid={self.lp.get('z_valid',0)} dist_valid={self.lp.get('dist_bottom_valid',0)}"
        else:
            vf = "stale"

        # VehicleLocalPosition is NED; /uav/state/pose is ENU.
        # Convert PX4 to ENU before comparing the two views of the same EKF state.
        if not s_lp and not s_pose:
            px4_enu_x = self.lp['y']
            px4_enu_y = self.lp['x']
            px4_enu_z = -self.lp['z']
            dx = px4_enu_x - self.pose.get('x', 0)
            dy = px4_enu_y - self.pose.get('y', 0)
            dz = px4_enu_z - self.pose.get('z', 0)
            xy_diff = math.hypot(dx, dy)
            pos_diff = f"ΔXY(local→ENU-pose)={xy_diff:6.3f}m ΔZ={dz:+6.3f}m"
        else:
            pos_diff = "stale"

        # Dist bottom
        db = self.lp.get('dist_bottom', float('nan')) if not s_lp else float('nan')

        lines = []
        # Header every 30 lines
        if self.line_count % 30 == 1:
            lines.append(f"{'='*140}")
            lines.append(f"{'Time':>7s} | Arm | Offb | {'PX4_X':>8s} {'PX4_Y':>8s} {'PX4_Z':>8s} | {'SLAM_X':>8s} {'SLAM_Y':>8s} {'SLAM_Z':>8s} | {'Roll':>7s} {'Pitch':>7s} {'Yaw':>7s} | {'DistBot':>7s} | Motor[0:3] | {'EKF_Fusion':40s}")
            lines.append(f"{'='*140}")

        armed = "ARM" if self.armed else "DIS"
        offb = "OFFB" if self.in_offboard else "MAN"

        lines.append(
            f"{elapsed:7.1f} | {armed:3s} | {offb:4s} | "
            f"{self._fmt(self.lp.get('x', float('nan'))) if not s_lp else ' stale_lp'} "
            f"{self._fmt(self.lp.get('y', float('nan'))) if not s_lp else ''} "
            f"{self._fmt(self.lp.get('z', float('nan'))) if not s_lp else ''} | "
            f"{self._fmt(self.pose.get('x', float('nan'))) if not s_pose else ' stale_pose'} "
            f"{self._fmt(self.pose.get('y', float('nan'))) if not s_pose else ''} "
            f"{self._fmt(self.pose.get('z', float('nan'))) if not s_pose else ''} | "
            f"{self._fmt(self.att.get('roll', float('nan'))) if not s_att else ' st_att'} "
            f"{self._fmt(self.att.get('pitch', float('nan'))) if not s_att else ''} "
            f"{self._fmt(self.att.get('yaw', float('nan'))) if not s_att else ''} | "
            f"{self._fmt(db)} | "
            f"{mstr} | avg={m_avg:.4f} rng={m_range:.4f} | "
            f"{ekf_str} | {pos_diff} | {vf}"
        )

        with open(OUT, 'a') as f:
            f.write('\n'.join(lines) + '\n')

def main():
    rclpy.init()
    node = DriftMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
