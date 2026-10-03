#!/usr/bin/env python3
"""Run librealsense rs-imu-calibration with a native frame queue.

The Jetson RSUSB Python binding enumerates and streams the D435i correctly via
``frame_queue``, while its Python callback overload receives no frames.  This
launcher preserves the upstream calibration implementation and replaces only
the sensor delivery mechanism.
"""

from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
import threading
import time


UPSTREAM = Path(
    "/workspaces/ros2_ws/third_party/librealsense/tools/"
    "rs-imu-calibration/rs-imu-calibration.py"
)


def load_upstream():
    spec = importlib.util.spec_from_file_location("rs_imu_calibration_upstream", UPSTREAM)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Cannot load upstream calibration script: {UPSTREAM}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def enable_imu_with_queue(module, wrapper, serial_no: str) -> bool:
    rs = module.rs
    devices = rs.context().query_devices()
    device = next(
        (
            item
            for item in devices
            if not serial_no
            or item.get_info(rs.camera_info.serial_number) == serial_no
        ),
        None,
    )
    if device is None:
        print("No matching RealSense device found.")
        return False

    active_profiles = {}
    wrapper.imu_sensor = None
    for sensor in device.sensors:
        for profile in sensor.get_stream_profiles():
            wanted_rate = {
                rs.stream.accel: 250,
                rs.stream.gyro: 200,
            }.get(profile.stream_type())
            if (
                wanted_rate is not None
                and profile.format() == rs.format.motion_xyz32f
                and profile.fps() == wanted_rate
            ):
                active_profiles[profile.stream_type()] = profile
                wrapper.imu_sensor = sensor
        if wrapper.imu_sensor:
            break

    if wrapper.imu_sensor is None:
        print("No IMU sensor found.")
        return False
    print(
        "\n".join(
            "FOUND %s with fps=%s" % (str(key).split(".")[1].upper(), profile.fps())
            for key, profile in active_profiles.items()
        )
    )
    profiles = list(active_profiles.values())
    if len(profiles) < 2:
        print("Not all IMU streams found.")
        return False

    wrapper.imu_sensor.open(profiles)
    wrapper.imu_start_loop_time = time.time()
    wrapper.frame_queue = rs.frame_queue(4096, True)
    wrapper.imu_sensor.start(wrapper.frame_queue)

    if wrapper.imu_sensor.supports(rs.option.enable_motion_correction):
        wrapper.imu_sensor.set_option(rs.option.enable_motion_correction, 0)

    def pump_frames() -> None:
        while not wrapper.is_done:
            frame = wrapper.frame_queue.poll_for_frame()
            if frame:
                wrapper.imu_callback(frame)
            else:
                time.sleep(0.001)

    wrapper.queue_thread = threading.Thread(target=pump_frames, daemon=True)
    wrapper.queue_thread.start()
    return True


def main() -> int:
    module = load_upstream()

    def patched_enable(wrapper, serial_no: str) -> bool:
        return enable_imu_with_queue(module, wrapper, serial_no)

    module.imu_wrapper.enable_imu_device = patched_enable
    result = module.main()
    return 0 if result is None else int(result)


if __name__ == "__main__":
    sys.exit(main())
