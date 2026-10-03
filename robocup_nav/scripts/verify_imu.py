#!/usr/bin/env python3
"""Read IMU intrinsics and collect 30 s via native frame_queue; no EEPROM command.

Run inside existing container with isolated pyrealsense2 Release on PYTHONPATH.
Writes a new evidence directory, never overwrites calibration data.
"""
from pathlib import Path
import json
import time
import numpy as np
import pyrealsense2 as rs

ROOT = Path(__file__).resolve().parents[1]
out = ROOT/'evidence'/time.strftime('imu_%Y%m%d_%H%M%S')
out.mkdir(parents=True, exist_ok=False)
report = {'eeprom_write_performed': False, 'duration_s': 30, 'streams': {}}
device = next(d for d in rs.context().query_devices() if d.get_info(rs.camera_info.serial_number) == '912112073953')
report['serial'] = device.get_info(rs.camera_info.serial_number)
report['firmware'] = device.get_info(rs.camera_info.firmware_version)
profiles = {}
sensor = None
for candidate in device.sensors:
    selected = {}
    for p in candidate.get_stream_profiles():
        rate = {rs.stream.accel: 250, rs.stream.gyro: 200}.get(p.stream_type())
        if p.fps() == rate and p.format() == rs.format.motion_xyz32f:
            selected[p.stream_type()] = p
    if len(selected) == 2:
        profiles, sensor = selected, candidate
        break
if sensor is None:
    raise RuntimeError('Both IMU streams unavailable')
for kind, profile in profiles.items():
    name = str(kind).split('.')[-1]
    try:
        intrinsics = profile.as_motion_stream_profile().get_motion_intrinsics()
        report['streams'][name] = {'intrinsics': np.asarray(intrinsics.data).tolist(),
                                   'noise_variances': list(intrinsics.noise_variances),
                                   'bias_variances': list(intrinsics.bias_variances)}
    except RuntimeError as exc:
        report['streams'][name] = {'intrinsics_error': str(exc)}
if sensor.supports(rs.option.enable_motion_correction):
    report['motion_correction_enabled'] = sensor.get_option(rs.option.enable_motion_correction)
queue = rs.frame_queue(2048)
samples = {rs.stream.accel: [], rs.stream.gyro: []}
sensor.open(list(profiles.values()))
started = False
try:
    sensor.start(queue)
    started = True
    start = time.monotonic()
    while time.monotonic()-start < 32:
        ok, frame = queue.try_wait_for_frame(200)
        if not ok or time.monotonic()-start < 2:
            continue
        kind = frame.profile.stream_type()
        if kind in samples:
            v = frame.as_motion_frame().get_motion_data()
            samples[kind].append([frame.get_timestamp(), v.x, v.y, v.z])
finally:
    if started:
        sensor.stop()
    sensor.close()
    for kind, values in samples.items():
        name = str(kind).split('.')[-1]
        stats = report['streams'][name]
        stats['count'] = len(values)
        if len(values) > 1:
            arr = np.asarray(values)
            np.savetxt(out/f'{name}.csv', arr, delimiter=',', header='timestamp_ms,x,y,z')
            delta = np.diff(arr[:, 0])
            norms = np.linalg.norm(arr[:, 1:], axis=1)
            stats.update({'hz': 1000*(len(arr)-1)/(arr[-1, 0]-arr[0, 0]),
                          'max_gap_ms': float(delta.max()), 'monotonic': bool((delta > 0).all()),
                          'mean': arr[:, 1:].mean(axis=0).tolist(),
                          'std': arr[:, 1:].std(axis=0).tolist(),
                          'norm_mean': float(norms.mean()), 'norm_std': float(norms.std())})
    report['both_streams_received'] = all(len(v) > 1 for v in samples.values())
    (out/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))
    print('Evidence:', out)
