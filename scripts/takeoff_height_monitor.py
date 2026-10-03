#!/usr/bin/env python3
"""Takeoff height monitor — prints AGL / ENU climb lines.

Used by takeoff_test.sh. Prefer redirecting stdout to a log file so the
interactive takeoff terminal is not flooded.
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

CENTER_OFFSET = 0.05  # PX4/CG above TFmini (m)
GROUND_SAMPLES = 6
GROUND_SETTLE_S = 1.0


def emit(line: str) -> None:
    print(line, flush=True)


class HeightMonitor(Node):
    def __init__(self, period: float):
        super().__init__('takeoff_height_monitor')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.dist_bottom = float('nan')
        self.pose_z = float('nan')
        self.range_at = 0.0
        self.pose_at = 0.0
        self.armed = False
        self.status_ok = False
        self.prev_armed = None

        self.ground_dist = float('nan')
        self.ground_enu = float('nan')
        self.zeroed = False
        self._disarm_since = 0.0
        self._ground_buf_dist: list[float] = []
        self._ground_buf_enu: list[float] = []
        self._collecting = False

        self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self._lp_cb,
            qos,
        )
        self.create_subscription(
            PoseStamped,
            '/uav/state/pose',
            self._pose_cb,
            10,
        )
        self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status_v1',
            self._status_cb,
            qos,
        )
        self.create_timer(period, self._tick)

    def _lp_cb(self, msg):
        self.dist_bottom = float(msg.dist_bottom)
        self.range_at = time.monotonic()

    def _pose_cb(self, msg):
        self.pose_z = float(msg.pose.position.z)
        self.pose_at = time.monotonic()

    def _status_cb(self, msg):
        self.status_ok = True
        self.armed = int(msg.arming_state) == 2  # ARMED

    def _fmt(self, value):
        return f'{value:5.2f}m' if math.isfinite(value) else '  n/a '

    def _start_ground_collect(self, reason):
        self._collecting = True
        self._ground_buf_dist = []
        self._ground_buf_enu = []
        self._disarm_since = time.monotonic()
        emit(f'[高度] 检测到{reason}，{GROUND_SETTLE_S:.0f}s 后贴地清零...')

    def _try_finish_ground_zero(self, now, raw_dist, raw_enu):
        if not self._collecting:
            return
        if now - self._disarm_since < GROUND_SETTLE_S:
            return
        if math.isfinite(raw_dist):
            self._ground_buf_dist.append(raw_dist)
        if math.isfinite(raw_enu):
            self._ground_buf_enu.append(raw_enu)
        if (len(self._ground_buf_dist) < GROUND_SAMPLES
                or len(self._ground_buf_enu) < GROUND_SAMPLES):
            return
        self.ground_dist = sum(self._ground_buf_dist) / len(self._ground_buf_dist)
        self.ground_enu = sum(self._ground_buf_enu) / len(self._ground_buf_enu)
        self.zeroed = True
        self._collecting = False
        emit(
            f'[高度] 已记录起飞基准: 测距={self.ground_dist:.2f}m '
            f'ENU_z={self.ground_enu:.2f}m（仅爬升量归零，AGL保留真实值）'
        )

    def _tick(self):
        now = time.monotonic()
        raw_dist = self.dist_bottom if (now - self.range_at) <= 1.0 else float('nan')
        raw_enu = self.pose_z if (now - self.pose_at) <= 1.0 else float('nan')

        if self.status_ok:
            if self.prev_armed is None and not self.armed:
                self._start_ground_collect('启动时已锁定')
            elif self.prev_armed is True and not self.armed:
                self.zeroed = False
                self._start_ground_collect('落地锁定')
            elif self.armed:
                self._collecting = False
            self.prev_armed = self.armed

        self._try_finish_ground_zero(now, raw_dist, raw_enu)

        sensor_agl = raw_dist
        center_agl = (
            raw_dist + CENTER_OFFSET if math.isfinite(raw_dist) else float('nan')
        )
        if self.zeroed and math.isfinite(self.ground_dist) and math.isfinite(raw_dist):
            range_climb = raw_dist - self.ground_dist
        else:
            range_climb = float('nan')
        if self.zeroed and math.isfinite(self.ground_enu) and math.isfinite(raw_enu):
            enu = raw_enu - self.ground_enu
        else:
            enu = float('nan')

        if self._collecting:
            mode = '清零中'
        elif self.zeroed:
            mode = '相对贴地'
        else:
            mode = '待清零'

        emit(
            f'[高度] EKF估测({mode}) '
            f'测距点AGL={self._fmt(sensor_agl)}  中心AGL≈{self._fmt(center_agl)}  '
            f'测距爬升={self._fmt(range_climb)}  ENU爬升={self._fmt(enu)}  '
            f'| raw {self._fmt(raw_dist)}/{self._fmt(raw_enu)}'
        )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--hz', type=float, default=2.0)
    args = parser.parse_args()
    period = 1.0 / max(args.hz, 0.2)

    rclpy.init()
    node = HeightMonitor(period)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
    return 0


if __name__ == '__main__':
    sys.exit(main())
