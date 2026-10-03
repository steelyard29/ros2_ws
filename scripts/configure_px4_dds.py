#!/usr/bin/env python3
"""
PX4 参数配置脚本 — 通过 MAVLink 设置 uXRCE-DDS / 室内激光定高

默认室内融合:
  - TFmini 主高度 (HGT_REF=2, RNG_CTRL=2)，关气压/磁力计
  - EKF2_EV_CTRL=1 (仅水平位置；不开 EV 速度/高度/yaw)

环境变量:
  PX4_SERIAL_DEV=/dev/ttyTHS1     首选串口
  PX4_SERIAL_CANDIDATES=...       逗号分隔备选串口
  PX4_BAUD_RATES=57600,921600,...
  PX4_HEIGHT_ONLY=1               只写 TFmini/EKF 高度相关参数
  PX4_SKIP_REBOOT=1               写参后不重启飞控
  PX4_SKIP_SAVE=1                 写参后不保存到 flash

注意: TELEM1 若已被 uXRCE-DDS 占用，停掉 Agent 也不等于 MAVLink 可用。
      此时请用 USB MAVLink (临时打开 SYS_USB_AUTO / MAV_*_CONFIG) 或 QGC。
"""

import sys
import time
import os
from pymavlink import mavutil

SERIAL_DEV = os.environ.get("PX4_SERIAL_DEV", "/dev/ttyTHS1")
SERIAL_CANDIDATES = [
    p.strip()
    for p in os.environ.get(
        "PX4_SERIAL_CANDIDATES",
        f"{SERIAL_DEV},/dev/ttyUSB0,/dev/ttyACM0",
    ).split(",")
    if p.strip()
]
BAUD_RATES = [
    int(baud.strip())
    for baud in os.environ.get("PX4_BAUD_RATES", "57600,921600,115200").split(",")
    if baud.strip()
]

PARAM_INT32 = mavutil.mavlink.MAV_PARAM_TYPE_INT32
PARAM_REAL32 = mavutil.mavlink.MAV_PARAM_TYPE_REAL32
INTEGER_PARAM_TYPES = {
    mavutil.mavlink.MAV_PARAM_TYPE_UINT8,
    mavutil.mavlink.MAV_PARAM_TYPE_INT8,
    mavutil.mavlink.MAV_PARAM_TYPE_UINT16,
    mavutil.mavlink.MAV_PARAM_TYPE_INT16,
    mavutil.mavlink.MAV_PARAM_TYPE_UINT32,
    mavutil.mavlink.MAV_PARAM_TYPE_INT32,
}

HEIGHT_ONLY = os.environ.get(
    "PX4_HEIGHT_ONLY", "0"
).strip().lower() in {"1", "true", "yes", "on"}
SKIP_REBOOT = os.environ.get(
    "PX4_SKIP_REBOOT", "0"
).strip().lower() in {"1", "true", "yes", "on"}
SKIP_SAVE = os.environ.get(
    "PX4_SKIP_SAVE", "0"
).strip().lower() in {"1", "true", "yes", "on"}

# Fixed indoor contract: EV full DOF (HPOS+VPOS+VEL+YAW) + TFmini height.
# Matches SD card canonical parameters exactly.
HEIGHT_REFERENCE = 2
RANGE_FUSION = 2
EV_CTRL = 15

HEIGHT_PARAMS = [
    ("SENS_TFMINI_CFG", 102, PARAM_INT32),
    ("SENS_TFMINI_HW", 1, PARAM_INT32),
    ("EKF2_EV_CTRL", EV_CTRL, PARAM_INT32),
    ("EKF2_EV_QMIN", 50, PARAM_INT32),
    # The bridge already publishes the base_link pose.
    ("EKF2_EV_POS_X", 0.0, PARAM_REAL32),
    ("EKF2_EV_POS_Y", 0.0, PARAM_REAL32),
    ("EKF2_EV_POS_Z", 0.0, PARAM_REAL32),
    ("EKF2_EVA_NOISE", 0.25, PARAM_REAL32),
    ("EKF2_EVP_NOISE", 0.20, PARAM_REAL32),
    ("EKF2_EVV_NOISE", 0.15, PARAM_REAL32),
    # TFmini is 5 cm below the PX4/CG; PX4 body Z points down.
    ("EKF2_RNG_POS_Z", 0.05, PARAM_REAL32),
    ("EKF2_RNG_A_VMAX", 2.0, PARAM_REAL32),
    ("EKF2_RNG_GATE", 6.0, PARAM_REAL32),
    ("EKF2_RNG_K_GATE", 3.0, PARAM_REAL32),
    ("EKF2_RNG_NOISE", 0.15, PARAM_REAL32),
    ("EKF2_HGT_REF", HEIGHT_REFERENCE, PARAM_INT32),
    ("EKF2_BARO_CTRL", 0, PARAM_INT32),
    ("EKF2_MAG_TYPE", 5, PARAM_INT32),
    ("EKF2_RNG_CTRL", RANGE_FUSION, PARAM_INT32),
    ("MPC_ALT_MODE", 2, PARAM_INT32),
    ("COM_DISARM_LAND", 0.5, PARAM_REAL32),
]

FULL_PARAMS = [
    ("MAV_0_CONFIG", 0, PARAM_INT32),
    ("MAV_1_CONFIG", 0, PARAM_INT32),
    ("MAV_2_CONFIG", 0, PARAM_INT32),
    ("UXRCE_DDS_CFG", 101, PARAM_INT32),
    ("SER_TEL1_BAUD", 921600, PARAM_INT32),
    ("SYS_USB_AUTO", 2, PARAM_INT32),
    ("SENS_TFMINI_CFG", 102, PARAM_INT32),
    ("SENS_TFMINI_HW", 1, PARAM_INT32),
    ("EKF2_EV_CTRL", EV_CTRL, PARAM_INT32),
    ("EKF2_EV_QMIN", 50, PARAM_INT32),
    # Bridge publishes base_link pose, not camera focal-point pose. Applying
    # the camera lever arm again in EKF2 would create attitude-coupled XY error.
    ("EKF2_EV_POS_X", 0.0, PARAM_REAL32),
    ("EKF2_EV_POS_Y", 0.0, PARAM_REAL32),
    ("EKF2_EV_POS_Z", 0.0, PARAM_REAL32),
    ("EKF2_EVA_NOISE", 0.25, PARAM_REAL32),
    # Soften absolute EV position vs Isaac VSLAM climb slip (see flight_pose CSV).
    ("EKF2_EVP_NOISE", 0.20, PARAM_REAL32),
    ("EKF2_EVV_NOISE", 0.15, PARAM_REAL32),
    ("EKF2_RNG_POS_Z", 0.05, PARAM_REAL32),
    ("EKF2_RNG_A_VMAX", 2.0, PARAM_REAL32),
    ("EKF2_RNG_GATE", 6.0, PARAM_REAL32),
    ("EKF2_RNG_K_GATE", 3.0, PARAM_REAL32),
    ("EKF2_RNG_NOISE", 0.15, PARAM_REAL32),
    ("EKF2_HGT_REF", HEIGHT_REFERENCE, PARAM_INT32),
    ("EKF2_BARO_CTRL", 0, PARAM_INT32),
    # Indoor: mag usually harmful; keep off until outdoor / calibrated.
    ("EKF2_MAG_TYPE", 5, PARAM_INT32),
    ("EKF2_GPS_CTRL", 0, PARAM_INT32),
    ("EKF2_RNG_CTRL", RANGE_FUSION, PARAM_INT32),
    # No dedicated optical-flow sensor on this airframe yet.
    ("EKF2_OF_CTRL", 0, PARAM_INT32),
    # 0 = use EV timestamp (bridge applies timesync); set ~50–80 if still lagging.
    ("EKF2_EV_DELAY", 0, PARAM_REAL32),
    ("MPC_ALT_MODE", 2, PARAM_INT32),
    # Auto-disarm shortly after land detector; force-disarm path still needed if
    # cs_rng_hgt drops and land detector never fires.
    ("COM_DISARM_LAND", 0.5, PARAM_REAL32),
    ("COM_ARM_WO_GPS", 1, PARAM_INT32),
    ("SYS_HAS_GPS", 0, PARAM_INT32),
    # Keep the physical RC mode switches available while allowing MAVLink fallback.
    ("COM_RC_IN_MODE", 2, PARAM_INT32),
    ("CBRK_IO_SAFETY", 22027, PARAM_INT32),
    ("COM_POWER_COUNT", 1, PARAM_INT32),
]

PARAMS = HEIGHT_PARAMS if HEIGHT_ONLY else FULL_PARAMS


def param_set(mav, name, value, param_type=1):
    """Set a PX4 parameter via MAVLink PARAM_SET message."""
    mav.mav.param_request_read_send(
        mav.target_system, mav.target_component,
        name.encode('utf-8'), -1
    )
    msg = mav.recv_match(type='PARAM_VALUE', blocking=True, timeout=2)
    if msg and msg.param_id.rstrip('\x00') == name:
        current = msg.param_value
        if param_type in INTEGER_PARAM_TYPES:
            current = int(current)
        if current == value:
            print(f"  {name} = {value} (unchanged)")
            return True
        print(f"  {name}: {current} -> {value}")

    mav.mav.param_set_send(
        mav.target_system, mav.target_component,
        name.encode('utf-8'),
        float(value),
        param_type
    )
    msg = mav.recv_match(type='PARAM_VALUE', blocking=True, timeout=3)
    if msg and msg.param_id.rstrip('\x00') == name:
        new_val = (
            int(msg.param_value)
            if param_type in INTEGER_PARAM_TYPES
            else msg.param_value
        )
        print(f"  {name} = {new_val} ✓")
        return True
    print(f"  {name} = FAILED (no confirmation)")
    return False


def connect_mavlink():
    seen = set()
    for port in SERIAL_CANDIDATES:
        if port in seen or not os.path.exists(port):
            continue
        seen.add(port)
        for baud in BAUD_RATES:
            print(f"Trying {port} @ {baud} baud...")
            mav = None
            try:
                mav = mavutil.mavlink_connection(port, baud=baud)
                msg = mav.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
                if msg:
                    print(f"Connected on {port}! PX4 type={msg.type}, autopilot={msg.autopilot}")
                    return mav, msg, port
                mav.close()
            except Exception as e:
                print(f"  Failed: {e}")
                if mav:
                    try:
                        mav.close()
                    except Exception:
                        pass
    return None, None, None


def main():
    mav, msg, port = connect_mavlink()
    if mav is None:
        print("ERROR: Could not connect to PX4 via MAVLink!")
        print("Tried:", ", ".join(SERIAL_CANDIDATES))
        print("If TELEM1 is in uXRCE-DDS mode, use USB MAVLink or QGC once.")
        sys.exit(1)

    mav.target_system = msg.get_srcSystem()
    mav.target_component = msg.get_srcComponent()

    mode = "height-only" if HEIGHT_ONLY else "full DDS+EKF"
    print(f"\nSetting parameters ({mode}) on {port} "
          f"(sys={mav.target_system} comp={mav.target_component})...")
    print("Height mode: TFmini reference + EKF2_RNG_CTRL=2 (Always), "
          "BARO off, MAG off")
    print("EV mode: EKF2_EV_CTRL=1 (horizontal position only; "
          "velocity/vertical/yaw disabled), "
          "EKF2_EV_QMIN=50")
    print("-" * 50)

    success = 0
    failed = 0
    for name, value, ptype in PARAMS:
        try:
            if param_set(mav, name, value, ptype):
                success += 1
            else:
                failed += 1
        except Exception as e:
            print(f"  {name} = ERROR: {e}")
            failed += 1
        time.sleep(0.1)

    print("-" * 50)
    print(f"Results: {success} ok, {failed} failed")

    if failed > 0:
        print("Some parameters failed. You may need to set them manually via QGC.")
        mav.close()
        sys.exit(1)

    if not SKIP_SAVE:
        print("\nSaving parameters to flash...")
        mav.mav.command_long_send(
            mav.target_system, mav.target_component,
            mavutil.mavlink.MAV_CMD_PREFLIGHT_STORAGE,
            0,
            1,  # save all params
            0, 0, 0, 0, 0, 0
        )
        time.sleep(1)
        print("Parameters saved.")
    else:
        print("\nSkip flash save (PX4_SKIP_SAVE=1).")

    if not SKIP_REBOOT:
        print("\nRebooting PX4...")
        mav.mav.command_long_send(
            mav.target_system, mav.target_component,
            mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
            0,
            1,
            0, 0, 0, 0, 0, 0
        )
        time.sleep(0.5)
        print("Done! PX4 is rebooting with new parameters.")
        print("After reboot, restart:")
        print("  ~/ros2_ws/scripts/run_slam_px4.sh stop && sleep 3 && "
              "~/ros2_ws/scripts/run_slam_px4.sh --bg")
    else:
        print("\nSkip reboot (PX4_SKIP_REBOOT=1). EKF may need a few seconds to fuse.")

    mav.close()


if __name__ == '__main__':
    main()
