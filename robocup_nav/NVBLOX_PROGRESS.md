# cuVSLAM + nvblox deployment, 2026-09-16

## Scope and preflight inventory

User requests replacing the mapping choice with cuVSLAM + nvblox and retaining Nav2 A* + DWB. Existing RTAB-Map prototype is retained. Host Jetson Orin Nano, 7.4 GiB visible RAM, Ubuntu22.04/L4T36.4.4; about 90 GiB disk free. Existing `isaac_ros_dev` restarted; no rebuild or deletion. Its inspected entrypoint only sources ROS and runs sleep infinity.

Container: Isaac ROS Visual SLAM3.2.6, Nav2 1.1.18. nvblox not previously installed. Apt simulated installation of nvblox-ros/nav2=3.2.5-0jammy adds only nvblox-msgs, nvblox-ros-common, nvblox-ros-python-utils and nvblox-rviz-plugin (six packages total), zero upgrades/removals. User deployment request authorizes this scoped installation. No firmware/EEPROM/PX4 changes or live flight outputs authorized.

## Design constraints

- cuVSLAM supplies continuous odometry and may provide a separate map correction / relocalization transform. nvblox integrates depth + poses into TSDF/ESDF; sparse visual landmarks are not Nav2 obstacle maps.
- nvblox 3.2 does not reintegrate historical dense voxels after pose-graph optimization automatically. Online baseline integrates in continuous odom, with navigation also in odom. Relocalization does not silently warp the dense map. Any actual odometry reset invalidates that mapping session and requires a fresh map before flight.
- Use installed-version official nvblox Nav2 layer, a bounded global costmap, a rolling local costmap and low-frequency depth integration. Mesh/color not required for navigation and disabled initially to conserve shared GPU/RAM.
- Foreground sensors may be tested under prior user authorization; no PX4 input/arming/actuators. Dimensions above/below reference and frame alignment remain unverified. Synthetic-test geometry is explicitly separate from live navigation geometry.

## Status

Installed all six nvblox packages at 3.2.5-0jammy in the existing container: 24.4 MB download, 176 MB installed, zero upgrades/removals. No host ROS package replacement. `container_env.sh` resolves GPU/RealSense shared libraries. Official release-3.2 source checked out for reference at vendor/isaac_ros_nvblox (7908a183acf84f4f1ab3fda7b6d6caf3eefc1f78), excluded from colcon via vendor/COLCON_IGNORE.

## Implementation and completed checks

- Added `nvblox.yaml`, mapper/navigation/combined launch files, `nvblox_guard.py`, bounded GPU/live/Navigation2 tests, and `run.sh nvblox*` commands. Existing perception launch gains optional native depth and map TF; defaults preserved.
- Native nvblox layer does not track stale input itself. Guard inhibits shadow commands on stale depth/ESDF/odom/status/command, bad tracking, height/tilt violation or latched abrupt odom jumps. Invalid map/frame/command data is rejected. No flight interface started.
- `evidence/nvblox_gpu_20260916_102520/report.json`: real GPU nvblox with synthetic depth, 129 ESDF messages (~4.88 Hz), 1720 known cells, 300 obstacle cells; passed.
- `evidence/nvblox_navigation_20260916_102931/report.json`: native nvblox/Nav2 layer and real NavFn A*/DWB, synthetic ESDF; lifecycle active, detour, commands, depth/slice/odom stale inhibition and recovery, blocked-path rejection, no /fmu/in topics. Passed again after guard hardening at `nvblox_navigation_20260916_103405`.
- Initial navigation test had an unnecessary px4_msgs import from the shared fixture. Made preview subscription optional (default unchanged for old tests); new nvblox test requires no PX4 package or flight bridge.
- `evidence/nvblox_live_20260916_103020/report.json`: 60 s actual camera/cuVSLAM/nvblox, depth25.00 Hz, IMU199.66 Hz, VIO26.16 Hz, ESDF4.86 Hz; known4393/obstacle1217 cells. VO state1 throughout observed statuses. VIO maximum gap200 ms, ESDF213 ms. Smoke passed, NOT motion accuracy or flight validation.
- `evidence/nvblox_live_20260916_103210/report.json`: 60 s actual full stack, all three Nav2 lifecycle states active, 40 global/94 local costmap messages, IMU199.53 Hz and VIO20.79 Hz (maximum gap267 ms). No goals sent; guard appropriately inhibits because command missing. One startup zero-stamp slice caused meaningless raw slice-rate statistic in this report; corrected future test statistics to separately count zero stamps and compute rate using positive timestamps. No old evidence overwritten.
- Startup Motion Module failure warning remains in both live runs, but IMU/VIO continued. No EEPROM/firmware changes attempted. Camera frame jitter persists under load; not a certified real-time system.
- Python compileall / shell syntax / combined launch argument expansion passed. Four existing geometry unit tests passed with scripts on PYTHONPATH.
- Sample during final full-stack run: system total7.4 GiB, used4.8 GiB, available2.3 GiB, swap51 MiB. This is a snapshot, not peak GPU memory or long-duration profiling.

## Competition review and boundaries

Re-read `/home/cfly/yun_ws/RoboCup规则.pdf`:10x10x4 m field,1.5 m corridor,~0.8 m doors, horizontal entry below1.5 m wall; takeoff score requires >1 m and >10 s stable;2026 targets have no approximate provided coordinates. This deployment covers online planar navigation components, not search/recognition/delivery/landing/whole mission. Sparse/dense map persistence and cross-session relocalization remain unvalidated. Full instructions and rollback in NVBLOX_README.md.

## Final live whole-stack retest

`evidence/nvblox_live_20260916_103426/report.json`, 45 s collection with corrected startup timestamp statistics:

- Depth22.59 Hz / max gap366.7 ms; IMU199.71 Hz /10.0 ms; VIO24.06 Hz /168.1 ms; ESDF4.87 Hz /214.7 ms.
- All positive timestamps monotonic; one initial zero-stamp ESDF recorded separately (rejected by guard).
- Known ESDF5121 cells, obstacle971 cells;29 global and67 local costmap messages with known cells.
- Planner/controller/BT lifecycle all active.915 observed VO statuses all1. No navigation goal submitted, no flight input topics.
- After initialization, guard reports only missing command (expected for no-goal test), without stale sensor/height/tilt/jump fault during this observation.
- Passed sensor/map/costmap smoke. This does not measure moving-pose accuracy, physical obstacle avoidance, actual flight braking, saved-map relocalization or long-duration reliability.
- Final docker top confirmed only docker-init and sleep infinity remain; all camera/VIO/mapping/Nav2 test processes stopped.
- Restored the previously stopped container to stopped state with `docker stop isaac_ros_dev`; packages and workspace files retained. Start it explicitly before using the new foreground command.
