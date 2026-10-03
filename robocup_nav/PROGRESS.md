# RoboCup navigation deployment

## 2026-09-17: static VIO/PX4 preview recovered

After D435i replug and PX4 power cycle, dual-source preview succeeded at
`evidence/alignment_preview_20260917_201641/`: IMU199.43Hz, VIO28.97Hz, PX4
attitude/local position~100Hz, all1521 tracking states1,1518 preview samples
and1517 attitude pairs. Static median preview-vs-PX4 RPY difference was
[1.42,1.82,-0.36]deg. This is a diagnostic only; preview remains off `/fmu/in`,
EV flags remain false, and no parameters or flight state changed. Next gate is
prop-off dynamic relative-motion and latency capture; see
`PX4_ALIGNMENT_PROGRESS.md`.

## 2026-09-17: slow dynamic relative-yaw preview passed

After PX4 power cycle and reducing nonessential high-rate JSON logging, bounded
90 s run `alignment_preview_20260917_203647` completed without preview fault.
PX4 attitude100.0 Hz, VIO29.0 Hz, IMU199.7 Hz;2407 timestamp-paired samples.
Slow fixture yaw produced preview/PX4 spans98.91/98.27 deg, correlation
0.999991, slope0.99568 and fitted RMSE0.148 deg. Relative yaw direction/scale
passes the preview gate. VIO position moved up to10.47 cm during rotation but
returned to1.31 cm; fixture-centre/lever-arm isolation remains unresolved.
Motion Module warning occurred while IMU/VIO continued. No EV input publisher,
parameter write, mode command or arming. Alignment/fusion remains unaccepted.

## 2026-09-17: dynamic yaw preview rejected by VIO discontinuity

After a full PX4 power cycle, `alignment_preview_20260917_202315` received
healthy-rate PX4 telemetry, camera IMU and VIO. During the requested fixture
yaw motion, consecutive VIO poses jumped 0.323 m in 33 ms; the preview guard
latched `VIO discontinuity; new session required` after 99 paired previews.
Raw VIO later diverged to tens of metres, so no post-jump data is accepted.
The valid pre-jump segment had only 0.15 deg yaw excitation and cannot test
relative yaw. No EV input publisher, parameter write, mode command or arming.
All temporary processes stopped. Do not retry until motion rate/scene and VIO
tracking robustness are addressed; alignment remains unaccepted.

## 2026-09-17: dynamic alignment attempt blocked by PX4 reconnect

`alignment_preview_20260917_202032` is not an alignment result. Camera IMU
199.9 Hz and cuVSLAM 29.7 Hz remained healthy enough to record, with 1570
tracking-state-1 samples and no preview fault, but the restarted serial Agent
received no PX4 attitude/status/local-position/timesync messages. Therefore
there are zero PX4/VIO pairs and the dynamic rotation cannot be evaluated.
No `/fmu/in` publisher, parameter write, mode command or arming occurred.
All temporary camera and serial processes stopped; ttyTHS1 is free. PX4 needs
a complete power cycle before one bounded dynamic retry.

## 2026-09-17: isolated alignment preview ready; acquisition blocked

Fixed local FRD preview and four geometry tests complete; see
`ALIGNMENT_PREVIEW_README.md`. Real60s run `alignment_preview_20260917_201231`
received no PX4 session and no IMU/VIO. Independent camera check:241/239 IR
frames,0 IMU in8s. No preview or fusion acceptance. All test processes stopped;
physical USB/PX4 recovery needed. FC parameters/EV input/arming unchanged.

## 2026-09-17: PX4 serial telemetry before VIO alignment

Confirmed TELEM1 link via ttyTHS1@921600, uXRCE-DDS domain0. Subscription-only
30s check received attitude/local position~100Hz, status~2Hz. PX4 disarmed;
yaw alignment and horizontal position validity false, all EV fusion flags false.
No flight-input publishers, parameter writes or EV injection. See
`PX4_ALIGNMENT_PROGRESS.md` and `evidence/px4_telemetry_20260917_195951/`.
Agent started for this probe was stopped after collection. VIO/PX4 alignment
is not yet complete; preserve live_flight_enabled=false.

## 2026-09-16: fixture yaw isolation, second run

Repeat `evidence/perception_20260916_134249/`, same prop-off stereo-only
gates. Peak yaw 92.9 deg (error 2.9 deg, pass). Height peak-to-peak 1.46 cm
(pass). Return now passes: endpoint yaw -1.42 deg, endpoint XY 2.19 cm.
Max XY from start 5.24 cm still exceeds the 5 cm isolation gate, so pure
in-place rotation is not passed. No `/fmu/in`; all 4289 VO states were 1.
Not PX4 EV or flight approval.

## 2026-09-16: fixture yaw isolation

Prop-off stereo+IMU+cuVSLAM only, ROS domain 174, 150 s,
`evidence/perception_20260916_133637/`. All 4292 VO states were 1; no `/fmu/in`.
Peak yaw 92.3 deg (error 2.3 deg, angle scale pass). Height peak-to-peak 1.28 cm
(pass vs 3 cm). Max XY from start 5.97 cm (fail vs 5 cm). Endpoint XY 4.95 cm
(pass) but endpoint yaw 5.00 deg (strict 5 deg gate fail). Pure in-place
isolation is not passed. This is far better than the handheld 35 cm / 9 cm
coupling, not flight or PX4 EV approval.

## 2026-09-16: SITL timeouts, estop, link-loss abort

Added airborne abort-land instead of in-air force-disarm. Plant now lands on
offboard heartbeat loss or `/robocup/sitl/estop`, and refuses Offboard/arm
while failsafe is latched. State machine timeouts are explicit; telemetry
loss, takeoff timeout, hover departure and estop all enter `ABORT_LAND` then
`ABORTED` only after the SITL vehicle is on the ground and disarmed.

`bash robocup_nav/run.sh sitl-safety` evidence
`evidence/sitl_safety_20260916_213108/`: all four isolated scenarios passed.
nominal DONE at 1.00 m; estop ABORTED from 0.70 m; link-loss ABORTED after
plant AUTO_LAND from 0.90 m; takeoff-timeout ABORTED at 0.12 m. No serial
agent, no `/fmu/in/vehicle_visual_odometry`, no real-aircraft arming.

## 2026-09-16: offline SITL takeoff/land (path B)

Handheld closed-loop/yaw isolation is still blocked on a fixture, so the next
software step is an isolated takeoff-hover-land state machine. Host command:
`bash robocup_nav/run.sh sitl-takeoff`. It uses ROS_DOMAIN_ID=175,
ROS_LOCALHOST_ONLY=1, a PX4 DDS-contract plant (not Gazebo, not a compiled
`px4_sitl` binary, not the real FC), and refuses to start if a serial
MicroXRCE/MAVLink agent is attached. No camera, cuVSLAM,
`/fmu/in/vehicle_visual_odometry`, serial, arming of the real aircraft, or
flight approval.

First run `evidence/sitl_takeoff_20260916_211536/`: passed in 9.7s.
WAIT_PLANT→STREAM→OFFBOARD→ARM→TAKEOFF→HOVER→LAND→DISARM→DONE. Peak/hover
height 0.9999 m, landed at 0.0 m and disarmed. EV fusion bits all false.
`/dev/ttyUSB0` was not held; no leftover SITL or agent processes. This is
software isolation, not first-flight approval.

## 2026-09-16: guided prop-off stationary baseline

User confirmed all propellers removed, aircraft placed still with clear camera view, independent power. Started only stereo+IMU+cuVSLAM in isolated domain174; no PX4, navigation, mapping or EEPROM writes. Added optional bounded duration/state rosbag recording and orientation-change measurement to smoke_perception.py (old45s default preserved), plus read-only rosbag static analyzer.

Evidence `evidence/perception_20260916_111837/`:75s collection including startup, ~68.9s valid sensor/VIO span,7.6MiB evidence. DualIR29.61/29.99Hz, IMU199.85Hz (maxgap10.0ms), VIO29.61Hz (maxgap133.4ms); timestamps monotonic and all2040 observed VO statuses1. Startup Motion Module failure warning occurred but IMU continued. No /fmu/in topics in isolated graph.

Last59.98s raw recorded VIO: maximum displacement6.83cm, endpoint6.76cm, maximum orientation change0.85deg. IMU acceleration norm9.879m/s2,std0.0282; gyro std[0.00475,0.00297,0.00611]rad/s. Static drift remains concerning; healthy streams are not localization/flight acceptance. Confirm no physical disturbance before stationary repeat/scene diagnosis; do not proceed as if motion accuracy is passed. State bag verified with19901 messages across5 topics. All sensor/recording processes stopped after test; no motor/arming commands. Container retained idle for next guided step.

Second stationary repeat after the user cleared the scene: `evidence/perception_20260916_122654/`, 75s collection and approximately 60s analyzed raw interval. Dual IR29.55/29.97Hz, IMU199.88Hz, VIO29.76Hz; all2023 VO states were1, timestamps monotonic, and no /fmu/in topics. Raw last-60s maximum displacement5.06mm, endpoint3.99mm, orientation change0.13deg; acceleration norm mean9.865m/s2/std0.0185, gyro axis std[0.00156,0.00218,0.00182]rad/s. This is a substantially stable static baseline and permits the next prop-off known-motion test, but is not flight approval. Container remains idle after the bounded collection.

Guided forward/return tests: `perception_20260916_123030` recorded an unsynchronized first one-way motion, max0.890m and no return. A phase-controlled run `perception_20260916_123808` recorded the forward leg to max1.019m but its 100s window ended during endpoint hold before return. A separate endpoint-to-start return run `perception_20260916_123958` recorded max displacement0.0587m and endpoint residual0.0587m, with all VO states1 and no /fmu/in topics. Forward scale passes the 10% preliminary target; return residual is borderline above the 5cm target, so repeat once slowly before flight-related decisions. These are handheld prop-off tests, not PX4 or flight validation.

Left and vertical guided tests: `perception_20260916_124539` recorded left displacement max1.001m; capture ended at y~0.544m so left-return closure was not recorded. `perception_20260916_124937` recorded vertical peak about0.493m, but horizontal peak-to-peak7.4cm and orientation change~10deg; height scale is good while pure vertical handling is not isolated. `perception_20260916_125313` recorded yaw peak94.27deg (within the preliminary 5deg angle target), but position peak~0.35m and z~0.09m changes, so pure in-place rotation is not passed. A rigid turntable/center fixture is required before using yaw/pose coupling as a flight gate. All runs were prop-off, sensor-only, with no PX4 inputs.

## 2026-09-16: cuVSLAM + nvblox + Nav2 A*/DWB

New independent shadow deployment is installed and tested. See `NVBLOX_PROGRESS.md` and
`NVBLOX_README.md` for exact versions, launch commands, safety boundaries and evidence.
Real GPU mapping, synthetic A*/DWB and stale-source tests, and live camera/VIO/nvblox/Nav2
costmap tests passed. Latest live report: `evidence/nvblox_live_20260916_103426/report.json`.
No flight approval, PX4 inputs, EEPROM writes or full competition task implementation.
The older RTAB-Map path and its failure evidence below are preserved, not silently repaired.

## 2026-09-14: scope and approvals

- User requested cuVSLAM localization, occupancy mapping, Nav2 A* and DWB horizontal navigation, with PX4 adaptation. Ask about unresolved hardware choices before live operation.
- User confirmed cleanup A+B+C. Delete only the four approved caches, the specified long recording database, and the OpenVINS archive below.
- Retain camera calibration, the other five-minute recording, logs/statistics, current source/build/install, existing Docker images/container/volumes, and old Isaac workspace archive.
- Physical footprint and payload vertical extent remain unknown. Live navigation/flight must remain disabled until measured and validated.
- No authorization to write camera EEPROM, alter PX4 parameters, feed external vision to PX4, arm, or fly.

## Cleanup inventory (before deletion)

Filesystem: 233 GiB total, 150 GiB used, 72 GiB available (rounded df -h).

| Target | Size | Recovery |
|---|---:|---|
| /home/cfly/.cache/vscode-cpptools | 2.1 GiB | Regenerate index |
| /home/cfly/.npm/_cacache | 626 MiB | Download again |
| /home/cfly/.cache/pip | 24 MiB | Download again |
| Docker build cache, unused records only | 2.447 GB reported | Rebuild/download layers; preserve images and container |
| /home/cfly/ros2_ws/isaac_vio_debug/evidence/20260911_113449_vio/bag_20260911_114334/bag_20260911_114334_0.db3 | 9,930,502,144 bytes | No verified external copy; irreversible local recording loss |
| /home/cfly/ros2_ws/backup/cleanup_20260914/openvins | 2.5 GiB | No verified external copy; local rollback removed |

No cpptools, npm or pip process was found by exact process name before cleanup. This is not a complete host-wide open-file audit.

## Calibration inspection (read-only)

Directory: `/home/cfly/ros2_ws/third_party/librealsense/tools/rs-imu-calibration`.
The September 14 output includes calibration.json/bin and accel_y.txt/gyro_y.txt. Six acceleration orientations have 1,000 samples each; gyro has 3,133 samples. Finite samples and increasing timestamps; stored gyro bias matches sample mean. Applying stored accelerometer fit gives orientation mean norms of 9.69–9.92 m/s². These are training-sample checks, not independent validation. EEPROM write/application and live IMU/VIO quality remain unverified.

## Deployment status

Cleanup A+B+C complete. Docker builder prune reported 2.447 GB reclaimed; images/container/volumes retained. Host deletion of B failed due to directory ownership; sudo was unavailable without a password. After inspecting the existing container entrypoint (sources ROS, then sleep infinity), started that container and deleted only the approved database via its existing workspace mount as root. Filesystem now reports 133 GiB used, 90 GiB available, about 18 GiB more free than before. OpenVINS archive and three local caches are absent. The long recording is no longer replayable; its metadata/statistics remain for provenance. No flight outputs started.

## Software deployment and isolated validation

- Added independent camera+cuVSLAM, external-odometry RTAB-Map and Nav2 launch files, A*/DWB configuration, explicit no-recovery behavior tree, geometry/depth projection bridge and preview-only PX4 message adapter.
- Preserved existing project entrypoints. No package upgrades, EEPROM writes or PX4 parameters changed.
- Four geometry tests passed. Real Nav2 plugins activated in ROS domain 173, produced an obstacle detour and DWB commands, rejected a fully blocked path, inhibited preview on stale odometry and exposed no /fmu/in topics. Evidence: evidence/integration.json. This is not flight dynamics simulation.
- After the first integration test, explicitly set DWB LimitedAccelGenerator sim_period=1/15 s to match controller frequency; revalidation required below.
- User supplied 0.45×0.45 m outer footprint, camera x=0.18/y=0/z=-0.04 m, lidar x=y=0/z=0.10 m, inherited orientations, 0.25–0.30 m total height. The endpoint of 0.15 m relative to PX4 and the base_link reference remain unclear; live geometry remains unverified.

## Independent IMU verification

User explicitly allowed a stationary 30 s sensor test. evidence/imu_20260914_130628/report.json:
- Camera serial 912112073953, FW 5.13.0.55. Motion correction enabled=1.
- SDK accelerometer matrix/bias match the saved calibration (matrix transpose per serialization convention).
- 7,488 accelerometer samples (~249.75 Hz), 5,864 gyro samples (~195.62 Hz); timestamps monotonic, maximum gaps ~50.9/50.0 ms.
- Acceleration norm mean 9.8266 m/s², std 0.2587; gyro axis std up to 0.1135 rad/s. Independent quiet-static quality NOT passed; motion versus noise not uniquely diagnosed.
- Gyro SDK bias equals saved JSON bias multiplied by pi/180, matching a conversion in local SDK parsers. Calibration script/SDK unit consistency remains to be audited; no corrective EEPROM write attempted.

## Joint RGB-D + stereo + IMU smoke test (failed)

evidence/perception_20260914_132042/ contains logs, map database shell and report.
- Actual profiles: dual IR/depth 640×480@30, RGB 640×480@15, accel250/gyro200.
- Driver reported Motion Module failure; received IR, RGB and aligned depth, but no IMU/VIO/status/map. RTAB-Map had no synchronized odometry input. This is NOT a usable map.
- IR subscriber rates ~28.4/27.0 Hz, RGB ~8.8 Hz, aligned depth ~14.0 Hz; gaps up to ~734 ms on RGB. These are observed rates during combined initialization/load, not certified sensor throughput.
- Bounded test terminated its own process groups. Container subsequently had only init and sleep, no ROS/camera/flight nodes.
- Asked user for physical D435i USB disconnect >=30 s and USB3 reconnect before the next minimum stereo+IMU test. Continue independent offline software checks while awaiting response.

## Minimum stereo + IMU retest after physical reconnect

- User confirmed D435i was unplugged for at least 30 s and reconnected to USB3.
- Evidence: `evidence/perception_20260914_132538/report.json`.
- Dual IR: right 29.99 Hz, left 29.46 Hz; IMU combined 199.83 Hz; both timestamps monotonic. cuVSLAM status 1 for 1,114 messages and odometry 29.36 Hz. Max observed static displacement 0.0274 m.
- The RealSense log still contains one `Motion Module failure` notification during startup, but IMU and VIO recovered and remained active. This reproduces the GitHub #6860 pattern: warning can be an activation hiccup rather than proof that the stream is unusable.
- RGB-D was intentionally excluded from this retest. Combined RGB-D+IMU test previously produced no IMU/VIO/map and must be retested separately after confirming the minimum chain.
- No EEPROM write, firmware update, PX4 input, arming, actuator, or flight action occurred.

## Step 2: RGB-D + stereo + IMU, no RTAB-Map

- User confirmed camera idle and authorized this step. Added `smoke --rgbd-no-map` so RGB-D can be tested without mapping (default smoke still starts RTAB-Map).
- Evidence: `evidence/perception_20260914_133323/report.json`. No `mapping.log`.
- Dual IR ~29.1/28.4 Hz; RGB ~14.1 Hz; aligned depth ~14.5 Hz; combined IMU ~199.6 Hz (max gap 31.6 ms); cuVSLAM odometry ~28.5 Hz; `vo_state=1` for 1,087 messages; static displacement 0.0038 m.
- `all_streams_received=true`; `/map` count 0 as intended; no `/fmu/in/*`.
- This log has no `Motion Module failure` string. Startup still restarts Depth/RGB/Motion once (driver `unite_imu_method` re-enable). cuVSLAM reports many frame deltas just above the 40 ms jitter threshold under RGB-D load; tracking stayed at `vo_state=1`.
- Compared with failed joint test `perception_20260914_132042` (IMU/VIO/map all zero), RGB-D no longer kills IMU/VIO when RTAB-Map is not started.
- Step 3 (RTAB-Map mapping) and step 4 (Nav2 costmaps) were not started. Camera processes were stopped after the 45 s collection. No EEPROM write, firmware update, PX4 input, arming, or flight.

## Step 3: RGB-D + stereo + IMU + RTAB-Map mapping (not confirmed)

- Evidence: `evidence/perception_20260914_134014/`. New database `map.db` (328 KiB), not an overwrite of production maps.
- Camera/VIO survived the extra load: dual IR ~28.2/27.4 Hz; RGB ~13.8 Hz (one 1.32 s gap); aligned depth ~14.5 Hz; IMU ~199.6 Hz (max gap 15.2 ms); cuVSLAM ~27.2 Hz; `vo_state=1` for 1,021 messages; static displacement 0.0055 m; no `/fmu/in/*`.
- Startup log again has `Motion Module failure` once; IMU/VIO continued afterward, same pattern as GitHub #6860 and the stereo retest.
- RTAB-Map subscribed to VIO + RGB-D, processed **one** node (`rtabmap (1)`), published `/map` once with 5,252 cells, then aborted:
  `Link.cpp:140::setInfMatrix()` — odometry angular information roll is `inf`. Process died (`exit -6`). Smoke therefore recorded `map.count=1` and `all_streams_received=false` (the script requires count > 1).
- This is **not** a usable mapping run: the grid is a single crashed snapshot, not a live map. Do not connect Nav2 costmaps.
- Root cause is cuVSLAM odometry covariance (zero/undefined roll variance → infinite information), not the earlier IMU-outage failure mode of `perception_20260914_132042`.
- Camera/mapping processes were stopped after the collection. No EEPROM write, firmware update, PX4 input, arming, or flight.

### 2026-09-17 dynamic alignment preview

Interactive bounded preview now has stage markers, operator stop and live status.
Cross-container host reception recovered with process-local discovery/UDP settings.
Active evidence: alignment_preview_20260917_230747; initial 215 valid previews,
tracking state 1, no fault. Waiting for guided known-distance motion; not a pass.
See PX4_ALIGNMENT_PROGRESS.md. No flight-controller writes or flight authorization.

Result: 0.70 m endpoint measured 0.7087 m forward, but 8.2 cm lateral and 3.8 cm
vertical coupling. During return, a 1.543 m VIO jump occurred in 33.34 ms near
the start point; vo_state stayed 1. Safety guard stopped collection. Dynamic
closure FAILED; do not enable PX4 external vision. Evidence retained in 230747.

Retry 231357 reproduced the return jump near the same 12 cm residual: 1.071 m
in 33.33 ms, only 0.0253 rad orientation change, vo_state still 1. Dynamic
closure remains FAILED. Far endpoint was 0.6352 m for measured 0.70 m (-9.3%).
Switched the cuVSLAM component to `component_container_mt`, explicitly disabled
localization/mapping, and extended status timing telemetry. Static retest is
required before any further manual motion. FC input remains prohibited.

Post-change static 232025: 1439 VIO samples at 28.05 Hz, all state 1; IMU
195.20 Hz; final/max static displacement 7.1/8.9 mm. Tracking execution
p50/p95/max 15.0/29.1/38.2 ms. Static acquisition passes, but 98 frame-gap
warnings remain (max 100.03 ms). Only a controlled dynamic reproduction is next.

Multi-thread reproduction 232216 failed differently: after reaching 0.70 m,
VIO stopped for >34 s while IMU/status stayed fresh and vo_state stayed 1;
cuVSLAM logged repeated IMU registration failures and Tracker Error 2. Run was
stopped before return and is invalid. Reverted to single-thread container. Added
a fail-closed >1 s established-VIO stale guard. External vision remains disabled.

Visual-only single-thread isolation 20260918_194847 PASSED the 0.70 m round trip:
far x=0.6363 m (-9.1%), final 3-D closure=9.79 mm, attitude residual=0.764 deg,
7091 continuous VIO samples at 29.30 Hz, max adjacent step=4.43 mm. This points
to the D435i IMU/fusion path, not scene geometry. Single-thread sampled IMU stamps
were monotonic; rejected multi-thread run had gross reorder. Added every-message
IMU timestamp statistics for the next probe. Fused dynamic acceptance still FAILS.

Calibration audit: accel training fit vector RMSE=0.1571 m/s^2 and corrected
norm=9.8051+/-0.0804 m/s^2. Same-version code exposes a gyro unit mismatch risk:
the Python tool stores motion-sample means directly while the D400 parser treats
EEPROM gyro bias as deg/s and multiplies pi/180. No EEPROM change. Fused IMU-copy
static run 200301 passed: 1450 state-1 VIO at 28.17 Hz, 10103 IMU at 195.21 Hz,
all stamps monotonic, dt 4.88--25.01 ms, final/max drift 1.35/1.66 cm. Dynamic
copy-mode validation remains required.

Gyro optical frame plus IMU-copy dynamic run 203457 completed the 0.70 m round
trip without a VIO jump: far x=0.6493 m (-7.3%), 3-D closure 9.77 mm, VIO
attitude closure 2.89 deg, maximum adjacent step 13.8 mm, and 8255 matched
previews all state 1. PX4 yaw remains invalid/uninitialized, so this is not PX4
EV alignment approval. Run used diagnostic copy mode; test gyro frame with
default interpolation mode 2 before selecting the production setting.

Final gyro-frame interpolation run 204301 completed the 0.70 m out/return with
no VIO jump: far x=0.6093 m (-13.0%), 3-D closure 12.97 mm, VIO attitude
closure 0.904 deg, max adjacent step 77.9 mm, 12197 matched previews all state
1. IMU stamps monotonic (4.89--50.02 ms; 308 gaps >20 ms). The PX4 heading is
not initialized/valid, so absolute RPY differences do not approve EV alignment.
Production launch keeps interpolation mode 2 and gyro optical frame; relative
yaw fixture is next.

Final gyro-frame interpolation yaw run 205215 failed rotational closure: the
left turn drifted about 24 cm, and the right-turn return produced a 1.227 m VIO
jump in 33.34 ms with only 0.0267 rad orientation change. The safety guard
stopped the run while vo_state remained 1. Fused translation passed, but fused
rotation closure does not. PX4 external vision remains prohibited.

Visual-only yaw run 210104 had no discontinuity: peak +91.29 deg and returned
yaw +1.28 deg. Manual rotation drifted 14.4 cm and ended 6.0 cm from the start,
so strict pure-rotation isolation failed by about 1 cm. It confirms yaw direction
and scale are plausible, but not camera-to-body/extrinsic acceptance. Visual-only
remains the safer candidate than fused D435i IMU; PX4 EV input is still disabled.

Production launch now defaults to `imu_fusion:=false` (D435i stereo visual-only;
PX4 IMU later). `camera_gyro_optical_frame` and interpolation mode 2 remain.
cuVSLAM still publishes `odom→base_link` from the inherited TF, so EKF2_EV_POS
must stay 0 to avoid a second lever-arm correction. Static PX4 read-only bench
with this default is the next gate; no EV injection.

Static visual-only plus PX4 read-only bench 211620 passed: Enable IMU Fusion
false, 1548/1549 state-1 previews at 29.18 Hz, IMU 194.08 Hz, PX4 attitude
99.74 Hz. No `/fmu/in` publishers. Live TF: `base_link→camera_link` is the
configured (0.18, 0, -0.04) m; `camera_imu_optical_frame` and
`camera_gyro_optical_frame` are identical; `odom→base_link` child frame is
body, not camera optical. PX4 remains disarmed, EV flags false, xy invalid,
yaw unaligned. Next physical gate is a mechanically centered visual-only yaw
fixture; do not write EV_CTRL or EV_POS.

Centered visual-only yaw run 211936 had no discontinuity: peak +89.60 deg
(error 0.40 deg) and returned yaw -1.24 deg. Endpoint XY 2.54 cm and final
hold 1.88 cm pass the 5 cm return gate; height peak-to-peak 1.66 cm. Max XY
from start was 9.23 cm, so strict in-place isolation still fails. This is
better than handheld 210104 (14.4 cm / 6.0 cm) but not extrinsic acceptance.
PX4 EV input remains disabled.

Tape update 2026-09-18: PX4 IMU/FC center to D435i. Lens glass +0.20 m,
body center z=-0.05 m, IR projector ~0.015 m to PX4 right, PX4 coincident
with the D435i module center. Converted to camera_link (left IR optical):
xyz=(0.196, 0.025, -0.05) m. Orientation unchanged. Extrinsics still
unverified until a centered yaw rerun; do not copy into EKF2_EV_POS.

New-extrinsic visual-only yaw 214135: no discontinuity. Peak +89.82 deg.
Left-turn max XY 4.08 cm (pass). At operator-confirmed return, yaw -0.62 deg
and endpoint XY 3.85 cm (pass); height ptp 8.1 mm. Full out/return max XY
6.02 cm fails the 5 cm isolation gate by 1 cm. Later wait motion to 12.8 cm
was excluded. PX4 EV remains disabled.

Visual-only left 0.70 m run 215519 passed: far y=0.702 m (+0.28%), 3-D
closure 4.70 cm, yaw residual 0.30 deg, max adjacent step 6.3 mm, no
discontinuity. Forward coupling on the outbound leg was 6.4 cm. PX4 EV
remains disabled; vertical isolation is next.

Visual-only up 0.45 m run 220009 had no discontinuity: peak z=0.452 m
(+0.4% vs the operator-corrected 45 cm cue), 3-D closure 3.91 cm, endpoint
yaw -1.30 deg, max adjacent step 4.0 mm. Handheld lift coupled 12.4 cm XY
and 5.3 deg yaw, so it is not a pure-vertical isolation pass. PX4 EV
remains disabled.

Visual-only yaw retry 220409: no discontinuity, IMU fusion false, no
`/fmu/in`. Peak yaw +91.49 deg (error 1.49 deg, pass). Left/full max XY
9.07 cm and endpoint XY 6.48 cm fail 5 cm; height ptp 4.25 cm fails 3 cm;
return yaw 1.54 deg pass. Worse than 214135 (left 4.08 cm / full 6.02 cm).
PX4 EV remains disabled.

Visual-only yaw retry 220958 passed isolation: no discontinuity, IMU
fusion false, no `/fmu/in`. Peak yaw +90.15 deg (error 0.15 deg). Left
max XY 3.11 cm; full out/return max XY 4.69 cm; endpoint XY 2.48 cm /
yaw 1.78 deg; height ptp 1.32 cm; max adjacent step 4.3 mm. This is
prop-off VIO body-origin evidence for the tape TF, not PX4 EV or flight
approval. `EKF2_EV_POS` stays 0. Next is a read-only EV injection plan.

Bounded prop-off EV bench `ev_inject_20260918_222612`: visual-only, no
param write, no arming. 4208 EV poses at quality 100 into
`/fmu/in/vehicle_visual_odometry`. `cs_ev_pos` and `cs_ev_yaw` stayed
true; `cs_ev_hgt`/`cs_ev_vel` stayed false; `xy_valid` true;
`heading_good_for_control` false (local FRD, not north). IMU fusion
false, all tracking state 1, no discontinuity. Not flight.

EV-on left 0.70 m `ev_inject_20260919_180507`: operator origin after hold.
Outbound far XY 69.6 cm (-0.6% vs 70 cm); PX4 local far 70.1 cm. Return
3-D closure 20.3 cm (VIO) / 20.2 cm (PX4) fails 5 cm; likely physical
miss, not a fusion drop. `cs_ev_pos`/`cs_ev_yaw`/`xy_valid` 632/632,
hgt/vel 0, no VIO jump, IMU fusion false. Not flight.

EV-on left 0.70 m retry `ev_inject_20260919_185742` passed: outbound far
XY 67.5 cm (-3.6%), 3-D closure 3.51 cm, yaw residual 0.31 deg, max
step 11 mm. PX4 local far 68.6 cm, return XY 2.94 cm. `cs_ev_pos`/
`cs_ev_yaw`/`xy_valid` 359/359, hgt/vel 0. IMU fusion false, no jump.
Not flight.

EV-on yaw `ev_inject_20260919_192538`: peak +91.11 deg, endpoint
-0.16 deg / 3.28 cm, z ptp 1.38 cm, no jump. Max XY 7.43 cm fails 5 cm
isolation (visual-only 220958 was 4.69 cm). EV pos/yaw/xy_valid
217/217, hgt/vel 0. Not flight.

EV-on yaw retry `ev_inject_20260919_192932` passed isolation with EV
held: peak +90.90 deg, left/full max XY 3.47 cm, endpoint 3.22 cm /
-1.11 deg, z ptp 0.88 cm, max step 5.9 mm. Flags 275/275 pos/yaw/
xy_valid, hgt/vel 0. IMU fusion false, no jump. Not flight.

EV-on up 0.45 m `ev_inject_20260919_193307` passed: peak z 43.8 cm
(-2.6%), 3-D closure 0.85 cm, yaw residual 1.34 deg, max step 3.3 mm.
Handheld lift coupled 7.8 cm XY (visual-only 220009 was 12.4 cm). PX4
local z ptp 6.2 cm while VIO rose 43.8 cm, so EKF did not take EV
height. PX4 XY followed VIO (far 7.9 cm, return 0.83 cm). `cs_ev_pos`/
`cs_ev_yaw`/`xy_valid` 417/417, hgt/vel 0. IMU fusion false, no jump.
Operator-complete. Not flight. Prop-off EV-on left/yaw/up gates are
now all passed.

Candidate EV publisher extracted: `scripts/ev_odometry.py` holds the
passed bench contract (local FRD, NaN velocity, ROS timestamps, no
range Z, no timesync offset). `scripts/ev_bridge.py` publishes only
`/fmu/in/vehicle_visual_odometry` with `--prop-off` and refuses
`live_flight_enabled`. Probe `--ev-inject` uses the same builder. Unit
tests in `tests/test_ev_odometry.py`. Old `vslam_odom_bridge` remains
prohibited. Covariance still quality 100 / unverified. Not flight.

Candidate `ev_bridge` 60 s static `ev_bridge_static_20260919_195132`
passed: 1691 EV poses, IMU fusion false, duration_complete.
`cs_ev_pos`/`cs_ev_yaw` 57/57, `cs_ev_hgt`/`cs_ev_vel` 0, `xy_valid`
5620/held, `heading_good_for_control` 0. Camera stopped; XRCE Agent
kept. Not flight.

Range-height `ev_inject_20260919_195848` FAIL for EKF use. Operator far
`dist_bottom` 10.2→56.8 cm (+46.7 cm vs 45 cm) and return closure
-0.15 cm, but `dist_bottom_valid` 0/2137 and sensor bitfield 0/2137.
Path peak 75.9 cm. PX4 local z ptp 1.31 m. EV pos/yaw 420/420, hgt/vel
0. VIO peak z 27.9 cm, 3-D closure 9.45 cm. Numeric lidar moved; PX4
never marked it valid. Not flight.

Ground visual-height `ev_inject_20260919_201427` passed with params
unchanged (EV height still off). Operator far VIO z 45.6 cm vs 45 cm
(+1.3%); path peak 46.8 cm (+4.0%); 3-D closure 2.30 cm; max step
6.7 mm. EV pos/yaw/xy_valid 251/251, hgt/vel 0. PX4 z ptp 0.92 m
(range still the height ref). IMU fusion false, no jump. Not flight.
Next is a user-approved param write: EV_CTRL 9→11 and HGT_REF 2→3.

2026-09-21 continuation: EV_BENCH_SUMMARY.md supersedes the older parameter
state above (Sept 19 EV_CTRL=11/HGT_REF=3 bench evidence). Prop-off occlusion
run ev_inject_20260921_183809 rejected a 1.219 m jump and stopped EV while
all tracking states still reported success. Independent observer recorded no
later EV for 30.52 s; first observed EV pos/yaw/hgt fusion-off flags at +0.5595 s,
with range fusion still active and final xy_valid=false, z_valid=true.
This passes the observed discontinuity-stop case only, not state-loss detection,
automatic recovery or flight failsafe. Camera/probe and observer stopped;
XRCE Agent preserved. Details in the run's OCCLUSION_RESULT.md. Next is
preview-only recovery after operator removes occluder, plus offline supervision.
2026-09-21 post-occlusion restart recovery:
`evidence/alignment_preview_20260921_184124` completed a 60 s bounded
preview-only run, with 53.12 s of valid VIO: 1567 messages at 29.51 Hz,
all vo_state=1, final/max drift 1.77/1.85 cm, max step 1.38 mm,
max source timestamp gap 100.02 ms. No fault, zero EV publication,
no discovered /fmu/in publishers. Camera/probe ended normally; persistent
XRCE Agent kept. New-session static recovery only, not same-session
relocalization, coordinate continuity, or airborne recovery acceptance.
2026-09-21 EV protection implementation and live height review:
See EV_SAFETY_REVIEW.md. Added ROS-free latched session gate, integrated into
candidate ev_bridge (still prop-off only), 15 unit tests passed, recorded-pose
replay rejects occlusion jump and never resumes; static replay has no fault.
New adapter has not been runtime flight/bench accepted. Read-only PX4 probe
184526 sees range-height active, baro/EV inactive, XY invalid. Current parameter
export is required; old Sep 17 export is stale. No FC writes or EV injection.
2026-09-21 fresh QGC export reviewed: see EV_SAFETY_REVIEW.md. Confirms
EV_CTRL=11/HGT_REF=3, BARO_CTRL=0/RNG_CTRL=2. Official v1.16.2 code can
synthesize valid 0.10 m ground range (EKF2_MIN_RNG), so today's valid range
flags are not proof of real near-ground ranging. Found Offboard-loss Position,
Offboard stick override disabled, FLTMODE/KILL both channel 5, RC-loss Return,
low-battery warning only. No writes; next raw range/barometer readouts and RC
mapping review required before proposing changes.
