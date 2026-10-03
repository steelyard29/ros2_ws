#!/usr/bin/env python3
"""
PX4 一键起飞/降落/返航/紧急停机
═══════════════════════════════════════════════════════════════

基于 PX4 官方 offboard_control.py (px4_ros_com) 改写，通过 micro XRCE-DDS
直接向 PX4 飞控发送 OffboardControlMode / TrajectorySetpoint / VehicleCommand。

与官方示例的差异:
  - 增加了 NAV_TAKEOFF / NAV_LAND / RTL / ESTOP 命令行
  - 增加了 VehicleLocalPosition 反馈 (基于 VSLAM 融合后的位置)
  - 去掉了无人机到达目标高度后立即降落的逻辑，改为悬停等待
  - 坐标系: NED (x=北, y=东, z=下, 负=向上)

依赖:
  - px4_gateway_node 已在运行 (提供辅助心跳)
  - VSLAM 里程计已初始化 (EKF 能融合)

用法:
  python3 px4_fly.py takeoff [altitude]   # 一键起飞 (默认 2m, NED z=-2)
  python3 px4_fly.py land                 # 一键降落
  python3 px4_fly.py rtl                  # 一键返航
  python3 px4_fly.py arm                  # 仅解锁
  python3 px4_fly.py disarm               # 仅锁定
  python3 px4_fly.py estop                # 紧急停机 ★危险★
  python3 px4_fly.py status               # 查看飞控状态

参考:
  https://github.com/PX4/px4_ros_com/blob/main/src/examples/offboard_py/offboard_control.py
  https://docs.px4.io/main/en/flight_modes/offboard
"""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)


# ═══════════════════════════════════════════════════════════════
#  常量
# ═══════════════════════════════════════════════════════════════

# PX4 自定义飞行模式 (DO_SET_MODE 的 param2)
PX4_CUSTOM_MODE = {
    'MANUAL':       0,
    'ALTCTL':       1,
    'POSCTL':       2,
    'AUTO_MISSION': 3,
    'AUTO_LOITER':  4,
    'AUTO_RTL':     5,
    'ACRO':         10,
    'OFFBOARD':     6,   # ★ 注意: DO_SET_MODE 的 param2=6, 不是 nav_state=14
    'STAB':         15,
}

# NAV_STATE → 中文名
NAV_STATE_NAMES = {
    0:  'MANUAL',       1:  'ALTCTL',    2:  'POSCTL',
    3:  'AUTO_MISSION',  4:  'AUTO_LOITER', 5:  'AUTO_RTL',
    10: 'ACRO',          14: 'OFFBOARD',    15: 'STAB',
    17: 'AUTO_TAKEOFF',  18: 'AUTO_LAND',
}


# ═══════════════════════════════════════════════════════════════
#  PX4 飞行控制器
# ═══════════════════════════════════════════════════════════════

class PX4Controller(Node):
    """PX4 Offboard 控制器

    参考 px4_ros_com 官方示例的 QoS 和发布模式:
      - TRANSIENT_LOCAL durability (确保 PX4 DDS 客户端能匹配到)
      - 10Hz 定时器驱动主循环
      - 先发 OffboardControlMode + setpoint 建立信任，再切换模式
    """

    def __init__(self):
        super().__init__('px4_offboard_controller')

        # ── QoS: 与官方示例完全一致 ──
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── 发布者 ──
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos)

        # ── 订阅者 ──
        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self._local_pos_cb,
            qos,
        )
        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status_v1',
            self._status_cb,
            qos,
        )

        # ── 状态 ──
        self.local_pos = VehicleLocalPosition()
        self.status = VehicleStatus()
        self._tick = 0                     # 定时器计数

        # ── 定时器: 10Hz (官方示例用 0.1s) ──
        self.timer = self.create_timer(0.1, self._timer_cb)

        self.get_logger().info('PX4Controller 初始化完成')

    # ── 回调 ────────────────────────────────────────────────

    def _local_pos_cb(self, msg: VehicleLocalPosition):
        self.local_pos = msg

    def _status_cb(self, msg: VehicleStatus):
        self.status = msg

    # ── 属性 (便捷访问飞控状态) ──

    @property
    def armed(self) -> bool:
        return self.status.arming_state == 2

    @property
    def preflight_pass(self) -> bool:
        return self.status.pre_flight_checks_pass

    @property
    def failsafe(self) -> bool:
        return self.status.failsafe

    @property
    def nav_state(self) -> int:
        return self.status.nav_state

    @property
    def pos_z(self) -> float:
        """当前 NED Z 坐标 (负=向上)"""
        return self.local_pos.z

    @property
    def pos_xy(self) -> tuple:
        """当前 NED X, Y"""
        return (self.local_pos.x, self.local_pos.y)

    def nav_name(self, ns=None):
        if ns is None:
            ns = self.nav_state
        return NAV_STATE_NAMES.get(ns, f'UNKNOWN({ns})')

    # ── 定时器主循环 (仅 offboard 模式时使用) ──

    def _timer_cb(self):
        """10Hz 定时器: 持续发布 offboard 心跳 + setpoint"""
        self._tick += 1
        self._publish_offboard_heartbeat()

    # ── 发布方法 (与官方示例一致) ──

    def _publish_offboard_heartbeat(self):
        """OffboardControlMode 心跳 (声明位置控制)"""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self._now_us()
        self.offboard_mode_pub.publish(msg)

    def _publish_setpoint(self, x: float, y: float, z: float,
                          yaw: float = 0.0):
        """TrajectorySetpoint (NED 坐标系)"""
        msg = TrajectorySetpoint()
        msg.position = [float(x), float(y), float(z)]
        msg.yaw = float(yaw)
        # 速度/加速度设为 NaN 表示"不控制"
        msg.velocity = [float('nan')] * 3
        msg.acceleration = [float('nan')] * 3
        msg.timestamp = self._now_us()
        self.setpoint_pub.publish(msg)

    def _publish_cmd(self, command: int, **params):
        """VehicleCommand (与官方示例参数名一致)"""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get('param1', 0.0)
        msg.param2 = params.get('param2', 0.0)
        msg.param3 = params.get('param3', 0.0)
        msg.param4 = params.get('param4', 0.0)
        msg.param5 = params.get('param5', 0.0)
        msg.param6 = params.get('param6', 0.0)
        msg.param7 = params.get('param7', 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self._now_us()
        self.cmd_pub.publish(msg)

    def _now_us(self) -> int:
        """当前时间戳 (微秒)"""
        return self.get_clock().now().nanoseconds // 1000

    # ── 高级命令 ──

    def arm(self):
        """解锁"""
        self._publish_cmd(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm 命令已发送')

    def disarm(self):
        """锁定"""
        self._publish_cmd(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm 命令已发送')

    def engage_offboard(self):
        """切换到 Offboard 模式"""
        self._publish_cmd(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info('Offboard 模式切换命令已发送')

    def land_cmd(self):
        """发送 NAV_LAND 命令"""
        self._publish_cmd(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info('Land 命令已发送')

    def rtl_cmd(self):
        """发送 NAV_RTL 命令"""
        self._publish_cmd(VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH)
        self.get_logger().info('RTL 命令已发送')

    def estop_cmd(self):
        """紧急停机 (DO_FLIGHTTERMINATION)"""
        self._publish_cmd(
            VehicleCommand.VEHICLE_CMD_DO_FLIGHTTERMINATION, param1=1.0)
        self.get_logger().warn('⚠  紧急停机命令已发送! 电机将立即停转!')

    def set_mode_cmd(self, mode: int):
        """切换到指定自定义模式 (PX4_CUSTOM_MODE)"""
        self._publish_cmd(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,
            param2=float(mode),
        )
        self.get_logger().info(f'模式切换命令已发送 → mode={mode}')

    # ── 等待辅助 ──

    def wait_for_arming_change(self, target_armed: bool, timeout: float = 5.0):
        """等待解锁/锁定状态变化"""
        t0 = time.time()
        while time.time() - t0 < timeout:
            time.sleep(0.1)
            rclpy.spin_once(self, timeout_sec=0.01)
            if self.armed == target_armed:
                return True
        return False

    def wait_for_nav_state(self, target_state: int, timeout: float = 10.0):
        """等待导航状态变化"""
        t0 = time.time()
        while time.time() - t0 < timeout:
            time.sleep(0.1)
            rclpy.spin_once(self, timeout_sec=0.01)
            if self.nav_state == target_state:
                return True
        return False


# ═══════════════════════════════════════════════════════════════
#  起飞流程 (参考官方示例)
# ═══════════════════════════════════════════════════════════════

def do_takeoff(node: PX4Controller, altitude: float) -> bool:
    """Offboard 模式一键起飞

    与官方示例完全一致的流程:
      1. 先持续发心跳 + setpoint 建立信任
      2. 切换 Offboard 模式
      3. 解锁
      4. 发送爬升 setpoints
      5. 检测到达目标高度后保持悬停
    """
    target_z = -abs(float(altitude))  # NED: 负=向上

    print(f'\n═══ 一键起飞: 目标高度 {altitude}m (NED z={target_z:.2f}) ═══\n')

    # 安全检查
    if node.armed:
        print('✗ 飞控已解锁! 请先锁定: px4_fly.py disarm')
        return False
    if not node.preflight_pass:
        print('✗ 飞前检查未通过! 无法起飞。')
        return False

    # ── 第 1 步: 发心跳 + 地面 setpoints 建立信任 ──
    print('第 1 步: 发送 Offboard 心跳 + 地面 setpoints (建立信任)...')
    for i in range(30):  # 3 秒 @ 10Hz
        node._publish_offboard_heartbeat()
        node._publish_setpoint(x=0.0, y=0.0, z=0.0, yaw=0.0)
        time.sleep(0.1)
        rclpy.spin_once(node, timeout_sec=0.01)

    # ── 第 2 步: 切换 Offboard ──
    print('第 2 步: 切换 Offboard 模式...')
    for _ in range(5):
        node.engage_offboard()
        time.sleep(0.2)
    time.sleep(1.0)
    rclpy.spin_once(node, timeout_sec=0.5)
    print(f'  当前 nav_state = {node.nav_name()}')

    # ── 第 3 步: 解锁 ──
    print('第 3 步: 解锁...')
    for _ in range(5):
        node.arm()
        time.sleep(0.2)
    if not node.wait_for_arming_change(True, timeout=5.0):
        print('✗ 解锁超时!')
        return False
    print('  ✓ 解锁成功')

    # ── 第 4 步: 爬升 ──
    print(f'第 4 步: 爬升至 {altitude}m...')
    climb_rate = 0.5       # m/s 爬升速率
    dt = 0.1               # 10Hz
    current_z = node.pos_z  # NED 起始高度 (通常是 0 或接近 0)

    while True:
        # 计算下一个目标高度 (逐步爬升)
        if current_z > target_z:
            current_z -= climb_rate * dt
            if current_z < target_z:
                current_z = target_z
        else:
            current_z = target_z

        node._publish_offboard_heartbeat()
        node._publish_setpoint(x=0.0, y=0.0, z=current_z, yaw=0.0)
        time.sleep(dt)
        rclpy.spin_once(node, timeout_sec=0.001)

        # 安全检查
        if node.failsafe:
            print('✗ 飞控触发故障保护! 中止起飞。')
            return False
        if not node.armed:
            print('✗ 飞控意外锁定! 中止起飞。')
            return False

        # 到达目标高度
        if abs(current_z - target_z) < 0.01:
            break

        # 进度显示
        if int(current_z * 10) % 10 == 0:
            h = -current_z  # 转成正的高度显示
            print(f'  当前高度: {h:.1f}m')

    print(f'✓ 已到达目标高度 {altitude}m，悬停中')
    print('  按 CTRL+C 退出悬停并降落')
    print('  或在另一个终端执行: ./px4_fly.sh land')

    # ── 第 5 步: 持续悬停 ──
    try:
        while True:
            node._publish_offboard_heartbeat()
            node._publish_setpoint(
                x=node.local_pos.x,   # 保持当前 XY
                y=node.local_pos.y,
                z=target_z,
                yaw=0.0,
            )
            time.sleep(0.1)
            rclpy.spin_once(node, timeout_sec=0.001)
    except KeyboardInterrupt:
        print('\n收到中断，开始降落...')
        do_land(node)

    return True


# ═══════════════════════════════════════════════════════════════
#  降落流程
# ═══════════════════════════════════════════════════════════════

def do_land(node: PX4Controller) -> bool:
    """一键降落

    发送 NAV_LAND 命令 → PX4 自动降落 → 检测锁定后退出
    """
    print('\n═══ 一键降落 ═══\n')

    if not node.armed:
        print('飞控未解锁，无需降落')
        return True

    # 如果当前是 offboard 模式，先保持心跳让降落命令生效
    print('发送 NAV_LAND 命令...')
    for _ in range(5):
        node.land_cmd()
        time.sleep(0.2)

    # 等待飞控进入 AUTO_LAND
    print('等待飞控进入 AUTO_LAND...')
    if node.wait_for_nav_state(18, timeout=5.0):
        print('✓ 飞控已进入 AUTO_LAND')

    # 等待降落完成 (飞控自动锁定 = 检测到降落)
    print('等待降落完成...')
    t0 = time.time()
    while time.time() - t0 < 60.0:  # 最多等 60s
        time.sleep(0.2)
        rclpy.spin_once(node, timeout_sec=0.01)
        if not node.armed:
            print('✓ 降落完成，飞控已自动锁定!')
            return True
        if node.nav_state == 18:  # 仍在 LAND 模式
            h = -node.pos_z
            if h < 0.3:
                print(f'  距地面约 {h:.1f}m...')

    # 超时降落后强制锁定
    print('降落超时，强制锁定...')
    for _ in range(5):
        node.disarm()
        time.sleep(0.2)
    if node.wait_for_arming_change(False, timeout=3.0):
        print('✓ 锁定成功')
        return True
    else:
        print('✗ 强制锁定失败，请手动处理!')
        return False


# ═══════════════════════════════════════════════════════════════
#  返航
# ═══════════════════════════════════════════════════════════════

def do_rtl(node: PX4Controller) -> bool:
    """一键返航"""
    print('\n═══ 一键返航 (RTL) ═══\n')
    if not node.armed:
        print('✗ 飞控未解锁')
        return False

    for _ in range(5):
        node.rtl_cmd()
        time.sleep(0.2)
    print('✓ RTL 命令已发送，飞控将自动返航并降落')

    # 等待锁定
    t0 = time.time()
    while time.time() - t0 < 120.0:
        time.sleep(0.2)
        rclpy.spin_once(node, timeout_sec=0.01)
        if not node.armed:
            print('✓ 返航降落完成，已锁定!')
            return True
    return True


# ═══════════════════════════════════════════════════════════════
#  状态显示
# ═══════════════════════════════════════════════════════════════

def show_status(node: PX4Controller):
    """打印飞控状态"""
    print(f"""
╔════════════════════════════════════════════════╗
║              PX4 飞控状态                       ║
╠════════════════════════════════════════════════╣
║  解锁状态:  {'ARMED (已解锁)' if node.armed else 'DISARMED (锁定)':<30} ║
║  飞行模式:  {node.nav_name():<30} ║
║  飞前检查:  {'✓ 通过' if node.preflight_pass else '✗ 未通过!':<30} ║
║  故障保护:  {'⚠  触发!' if node.failsafe else '正常':<30} ║
║  NED 位置:  x={node.local_pos.x:.2f} y={node.local_pos.y:.2f} z={node.pos_z:.2f} ║
║  高度(ENU): {(-node.pos_z):.2f}m{'':<21} ║
╚════════════════════════════════════════════════╝
""")


# ═══════════════════════════════════════════════════════════════
#  CLI
# ═══════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(
        description='PX4 一键起飞/降落/返航/紧急停机',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  ./px4_fly.sh takeoff 2.5    # 起飞到 2.5m 高度 (offboard模式)
  ./px4_fly.sh land           # 一键降落
  ./px4_fly.sh rtl            # 一键返航
  ./px4_fly.sh arm            # 仅解锁
  ./px4_fly.sh disarm         # 仅锁定
  ./px4_fly.sh estop          # ★ 紧急停机 (电机立即停转!)
  ./px4_fly.sh status         # 查看飞控状态

起飞前请确认:
  1. ./run_slam_px4.sh --bg       # 启动 VSLAM + PX4 Bridge
  2. sleep 30                      # 等待 VSLAM 初始化
  3. ./run_slam_px4.sh --check-arm # 检查解锁条件
  4. ./px4_fly.sh takeoff 2.5     # 起飞
  5. ./px4_fly.sh land            # 降落
        """,
    )
    sub = subparsers = parser.add_subparsers(dest='cmd', help='命令')

    p = subparsers.add_parser('takeoff', help='一键起飞 (offboard模式)')
    p.add_argument('altitude', nargs='?', type=float, default=2.0,
                   help='目标高度 (米, 默认 2.0)')

    subparsers.add_parser('land', help='一键降落')
    subparsers.add_parser('rtl', help='一键返航')
    subparsers.add_parser('arm', help='仅解锁')
    subparsers.add_parser('disarm', help='仅锁定')
    subparsers.add_parser('estop', help='紧急停机 (电机立即停转!)')
    subparsers.add_parser('status', help='查看飞控状态')

    args = parser.parse_args()

    if args.cmd is None:
        parser.print_help()
        return 1

    # 初始化
    rclpy.init(args=sys.argv)
    node = PX4Controller()

    # 等待状态更新
    print('等待飞控状态...')
    for _ in range(50):
        time.sleep(0.1)
        rclpy.spin_once(node, timeout_sec=0.01)
        if node.status.arming_state != 0:  # 0 = 未初始化
            break

    if args.cmd == 'status':
        show_status(node)
        node.destroy_node()
        rclpy.shutdown()
        return 0

    if node.status.arming_state == 0:
        print('⚠  警告: 无法获取飞控状态 (DDS 链路不通?)')
        print('   请先启动: ./run_slam_px4.sh --bg')
        node.destroy_node()
        rclpy.shutdown()
        return 1

    success = False
    try:
        if args.cmd == 'takeoff':
            success = do_takeoff(node, args.altitude)

        elif args.cmd == 'land':
            success = do_land(node)

        elif args.cmd == 'rtl':
            success = do_rtl(node)

        elif args.cmd == 'arm':
            print('发送解锁命令...')
            for _ in range(5):
                node.arm()
                time.sleep(0.2)
            success = node.wait_for_arming_change(True, timeout=5.0)
            print('✓ 已解锁' if success else '✗ 解锁超时')

        elif args.cmd == 'disarm':
            print('发送锁定命令...')
            for _ in range(5):
                node.disarm()
                time.sleep(0.2)
            success = node.wait_for_arming_change(False, timeout=5.0)
            print('✓ 已锁定' if success else '✗ 锁定超时')

        elif args.cmd == 'estop':
            print('\n⚠⚠⚠  确认立即停转电机? 无人机会坠落!  ⚠⚠⚠')
            confirm = input('输入 YES 确认 (其他取消): ')
            if confirm == 'YES':
                for _ in range(10):
                    node.estop_cmd()
                    time.sleep(0.05)
                print('⚠  紧急停机已执行')
                success = True
            else:
                print('已取消')

    except KeyboardInterrupt:
        print('\n用户中断')
        success = True

    except Exception as e:
        print(f'\n✗ 异常: {e}')
        import traceback
        traceback.print_exc()

    finally:
        node.destroy_node()
        rclpy.shutdown()

    return 0 if success else 1


if __name__ == '__main__':
    sys.exit(main())
