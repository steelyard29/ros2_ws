# VIO / PX4 preview

Entry (host; needs running existing isaac_ros_dev container and unoccupied ttyTHS1):

```bash
source /opt/ros/humble/setup.bash
source /home/cfly/ros2_ws/install/setup.bash
python3 /home/cfly/ros2_ws/robocup_nav/scripts/probe_px4_telemetry.py --duration 60 --alignment-preview
```

This opens a bounded serial XRCE session and starts a separately bounded
container camera/VIO supervisor. PX4 parameters are not changed. It subscribes
to PX4 telemetry and publishes ONLY `/robocup/alignment/visual_odometry_preview`.
It rejects existing /fmu/in publishers and stops inspection when PX4 reports armed.
Both programs clean up their own processes; the container remains available.
Known device behavior: the PX4 XRCE client may fail to reconnect after agent stop.
If no session appears, inspect agent.log and request physical recovery; do not
write flight-controller parameters or force reboot automatically.

Output frame is world-fixed local FRD with initial visual yaw removed. It is not
BODY_FRD, not geographic NED, and is not aligned to an invalid PX4 heading. Raw
VIO height is retained, with no range substitution. Timestamps use the host ROS
clock and retain the VIO acquisition stamp (DDS SYNCT=1). Frame/stamp/pose and
tracking freshness are checked; discontinuities latch preview inhibition.

Preview quality=-1 and covariance/velocity are unspecified (NaN). This is a
geometry/time inspection artifact; do not relay it into PX4. No calibrated
covariance, extrinsic fit, actual fusion, or flight readiness is asserted.

Evidence directory contains agent/camera logs, raw telemetry+preview JSONL and
report.json. Angular comparisons pair latest PX4 attitude only within100ms of
the VIO stamp; this is approximate pairing, not latency estimation. Static
roll/pitch comparison and arbitrary initial yaw difference are diagnostics.
Dynamic hand-motion and initial extrinsic validation are separate gates.

Pure geometry tests:

```bash
python3 -m unittest discover -s /home/cfly/ros2_ws/robocup_nav/tests -p test_alignment_math.py
```

2026-09-17 first live attempt `alignment_preview_20260917_201231`: no PX4 client
session, no IMU/VIO; separate8s camera diagnosis received241 left/239 right images
and0 IMU messages. Motion Module failure in log. Zero previews. This is a failed
acquisition, not a frame-alignment result. Original report's error=null only
means no exception; empty streams and process exit1 indicate failure. Subsequent
runner versions explicitly report missing required telemetry.

Dynamic sessions: use `--alignment-preview --dynamic-session --duration 900`.
The evidence directory contains `control.json` (stage and stop) and continuously
updated `live_status.json`. Set stage labels at each operator-confirmed endpoint;
set stop=true only after the final stable return. The 900 s cap is a safety
timeout, not proof of completion. Check freshness and fault before every cue.
The XRCE Agent remains running after collection to preserve PX4 connectivity.
Host preview uses ROS_LOCALHOST_ONLY=0 and the existing UDP-only Fast DDS profile;
camera container stays localhost-only with the same UDP profile. These settings
are process-local, not global environment changes. Preview is never FC input.

Once at least five VIO messages have arrived, a VIO receipt age over 1 s is a
hard fault even if cuVSLAM still publishes `vo_state=1`. The D435i pipeline must
use the single-threaded component container here: the multi-threaded container
caused out-of-order IMU registration failures and stopped odometry in motion.

`--visual-only-diagnostic` keeps D435i IMU acquisition and PX4 read-only
telemetry active but starts cuVSLAM with IMU fusion disabled. After fused
rotation closure failed, this is the production candidate path
(D435i stereo + PX4 IMU later). `--ev-inject` publishes the shared
`ev_odometry.py` contract to `/fmu/in/vehicle_visual_odometry` only
(prop-off, visual-only). `run.sh ev-bridge --prop-off` is the same
contract without starting the camera supervisor; VIO must already be
up. Neither is flight approval. Do not launch `vslam_odom_bridge`.
Reports include every-message IMU timestamp
monotonicity, min/max interval, and >20 ms gap counts. The perception
launch default is now `imu_fusion:=false`; copy-mode remains the only
fused path started from the alignment supervisor.

`--imu-copy-diagnostic` keeps cuVSLAM IMU fusion enabled but starts the
RealSense merged IMU with copy mode (`unite_imu_method=1`) instead of linear
interpolation (`2`). This is reversible and does not change camera EEPROM.
