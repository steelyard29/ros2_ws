# VIO / PX4 alignment — 2026-09-17

User confirms PX4 powered, TELEM1 wired to Jetson, all propellers removed.
Authorized scope: link inspection, telemetry and preparation of alignment; no
EV injection, parameter changes, mode commands or arming.

## Live telemetry discovery

Evidence: `evidence/px4_telemetry_20260917_195951/{agent.log,report.json}`.
Host sandbox hides devices; host-level approved inspection found ttyTHS1,
ttyTHS2 and a CP2102 ttyUSB0. All unoccupied. Existing configuration points to
ttyTHS1 at921600; a bounded30s independent XRCE session successfully confirmed
that exact port/rate. No alternate ports or parameter-writing scripts run.

`probe_px4_telemetry.py` subscribes only, checks flight-input publishers before
and during the session, and stops its own agent afterward. No existing
flight-input publishers were observed. /fmu/in topics exist because the PX4
client creates subscribers; topic existence does not imply command publication.

- Vehicle attitude:2342 messages,100.19Hz.
- Local position:2340 messages,100.10Hz.
- Vehicle status_v1:47 messages,1.99Hz; last arming_state1(disarmed), failsafe false,
  pre_flight_checks_pass false.
- Estimator:tilt aligned, yaw NOT aligned; external-vision position/height/
  velocity/yaw fusion flags all false. These flags do not disclose configured
  EKF2_EV_CTRL bits; parameters have not been read.
- Local xy/velocity invalid; z/vz valid. heading_good_for_control false,
  inertial dead-reckoning active. Current horizontal origin/yaw cannot be treated
  as an independently validated alignment reference.
- Range-height fusion flag true, but last dist_bottom_valid false with numerical
  dist_bottom~0.099m. Do not accept this numeric reading merely because positive.
- DDS timesync status received; last RTT9225us. This is transport RTT, not VIO
  latency or proof of converged end-to-end synchronization.

## Existing bridge findings (not executed or changed)

`src/px4_interface/src/vslam_odom_bridge.cpp` defaults to publishing real EV input.
Its range validity test is `dist_bottom_valid || dist_bottom>0.01`, which would
accept the invalid positive reading observed here and substitute it for VIO z.
Preview should preserve raw VIO z and report range separately.

The bridge manually adds timesync offset to ROS stamps. The local PX4 XRCE
source passes time offsets into serialization/deserialization, and live outbound
timestamps are Unix-scale. Double time compensation is a concrete risk; validate
actual firmware/UXRCE_DDS_SYNCT behavior before any injection. Local source is not
proof of the exact flashed firmware version.

No persistent yaw/origin transform has been fitted or saved. Next: read actual
firmware/EV/yaw/time-sync parameters; capture simultaneous VIO/PX4 data for relative
attitude and temporal checks. Resolve yaw reference/fusion strategy before claiming
NED alignment. Any EV or parameter change still requires a concrete reviewed plan.

## Parameter export reviewed, 2026-09-17

User supplied `/home/cfly/param.params.txt`, unchanged. Header:PX4 Pro1.16.2,
Git Revision54f0455ffc000000. SHA256:
`c24f77ac8b863349195a8b6b0c92e3c9c9fb492d0db2977cb65b15286462963c`.
Export is configuration evidence, not a live re-read of every parameter.

Values:EKF2_EV_CTRL15(horizontal+vertical position+3D velocity+yaw),
EKF2_HGT_REF2(range), EKF2_RNG_CTRL2(enabled), EKF2_MAG_TYPE5(disabled),
EKF2_GPS_CTRL0, EKF2_BARO_CTRL0, EKF2_EV_DELAY0ms, EV_QMIN50,
EV_NOISE_MD0, EV_POS_X/Y/Z0. UXRCE_DDS_SYNCT1, SYNCC0, DOM_ID0,
CFG101, SER_TEL1_BAUD921600. Range lever arm Z0.05m.

Verified official v1.16.2 files directly from raw.githubusercontent.com:
- src/modules/uxrce_dds_client/module.yaml
- Tools/msg/templates/ucdr/msg.h.em
- src/modules/ekf2/EKF/aid_sources/external_vision/ev_yaw_control.cpp

Unlike the local PX4 checkout(v1.18-alpha), these references match the export's
release version; custom firmware differences remain possible. With SYNCT1, the
XRCE client adjusts timestamp and timestamp_sample on ingress/egress. The ROS
publisher should provide Agent/ROS-clock timestamps, not apply the estimated
boot-time offset again. EV_DELAY0 is a configured compensation, not measured
zero sensor latency.

Interpretation correction: absent yaw/XY validity before EV arrives is consistent
with the selected indoor vision-yaw setup. It does not require enabling a compass
or copying PX4's unaligned yaw into VIO. An independent, fixed, gravity-aligned
local FRD visual frame is appropriate for preview; it is NOT BODY_FRD and does not
rotate with the vehicle. v1.16.2 LOCAL_FRAME_FRD EV-yaw initialization can set
ev_yaw=true while keeping yaw_align=false (no geographic-north alignment).
Thus cs_yaw_align=true is not a universal acceptance condition for this setup.

## Reviewable next-stage plan (not executed)

1. No PX4 parameter changes for subscription-only dual-source observation or
   preview. Preserve raw VIO position/height; keep range separate. Use actual
   base_link/PX4 reference extrinsics, not duplicated camera offsets.
2. Preview in a fixed local FRD visual frame, establish origin/yaw once per
   session, and latch/reset on VIO discontinuity. Do not fit to an unaligned
   PX4 heading. Compare relative rotation and level attitude before EV injection.
3. Keep DDS SYNCT1, ROS-clock publication/sample stamps with real acquisition
   time retained, and measured latency before changing EV_DELAY. Covariance and
   quality must reflect actual tracking, not an unconditional quality100.
4. Proposed first fused bench stage: EV_CTRL15->9(horizontal position+yaw),
   retaining separate range height, only AFTER range validity/range limits are
   checked. This intentionally excludes EV height/velocity until independently
   verified. Current invalid dist_bottom reading is not sufficient to approve
   the height source. This is a proposal, not a write or flight authorization.
5. If approved later: preserve export, change only reviewed parameter(s), verify
   readback, then bounded prop-off EV input. Stop EV input on tracking/time/frame
   faults; assess horizontal validity, EV-pos/yaw flags, innovations and resets.
   Rollback:stop dedicated EV publisher, restore EV_CTRL15 from export with user
   approval; do not indiscriminately import the whole parameter file.

No bridge runtime code, exported parameters, EEPROM or PX4 state changed by this
review. The existing live bridge remains unsuitable for direct launch until the
timestamp/range/yaw contracts above are addressed and verified in preview.

## Static dual-source preview recovered, 2026-09-17

After the user physically replugged D435i and power-cycled PX4, the bounded
preview succeeded: `evidence/alignment_preview_20260917_201641/`. PX4 attitude
and local position were both about100Hz, IMU199.43Hz, VIO28.97Hz, tracking state1
for all1521 status messages. Preview emitted1518 samples with1517 approximate
PX4 attitude pairs; no VIO discontinuity fault. VIO receipt age p50/p95/max was
27.3/35.6/318.8ms; the max is an outlier and requires dynamic latency review.

Static median preview RPY was [0.165,-0.017,-0.024]deg; PX4 RPY was
[-1.266,-1.838,0.340]deg; differences were [1.422,1.822,-0.362]deg. These are
diagnostic attitude offsets, not a fitted extrinsic or yaw alignment. PX4 still
reported EV flags false, disarmed, yaw_align false and invalid horizontal
position because the preview is intentionally not published into `/fmu/in`.
No parameter or flight state changed. Static preview acquisition passes; alignment
acceptance remains false.

## Next authorized gate

Run a prop-off dynamic dual-source test with the preview still isolated: known
forward/left/vertical/yaw motions, simultaneous PX4 attitude/local-position and
VIO recording, and measured receipt/sample latency. Compare relative changes,
not absolute PX4 origin or its currently unaligned yaw. Reject any pose jump,
stale IMU/VIO, or timestamp outlier. Only after this report is reviewed should
we prepare a concrete minimal EV input plan; do not switch `EKF2_EV_CTRL` or
start the live bridge automatically.

## Preview implementation and blocked hardware acquisition

2026-09-17: Added `alignment_math.py`, `alignment_camera.py`, preview mode in
`probe_px4_telemetry.py`, four pure-geometry unit tests and
`ALIGNMENT_PREVIEW_README.md`. Tests verify FLU/FRD signs, fixed frame despite
initial yaw and later rotation, retained tilt, and invalid inputs. All passed.
No changes to old live bridge or FC parameters. Preview uses quality=-1 and
unspecified variance/velocity; not authorized or suitable for direct EV relay.

Live60s attempt `evidence/alignment_preview_20260917_201231/` FAILED acquisition:
agent opens ttyTHS1@921600 but no XRCE client session or PX4 messages; camera
reports Motion Module failure and no VIO/status. Separate8s subscriber check
confirmed241 left/239 right images,0 IMU. Report has0 previews/0 pose pairs.
No coordinate/extrinsic alignment can be inferred. Hardware recovery is required
before retry: physical camera USB power cycle, and user-controlled PX4 power
cycle for the unresponsive XRCE session. No automatic reboot requested/sent.
Sensor supervisor and serial agent have both stopped; container has only init
and sleep. After this failure, explicit missing-stream error reporting added
(the original report has empty streams/error null, exit1; evidence preserved).

## Static and dynamic preview results after hardware recovery

Static run `alignment_preview_20260917_201641` recovered both streams:
1518 previews and1517 pairs, VIO29.0Hz/PX4 attitude99.9Hz. Median preview-minus-
PX4 RPY was approximately(+1.42,+1.82,-0.36)deg. This supports the basic
FLU-to-local-FRD signs but is not dynamic alignment or calibration.

The next PX4 reconnect attempt `alignment_preview_20260917_202032` had no PX4
telemetry. After another power cycle, fast dynamic run
`alignment_preview_20260917_202315` was rejected correctly: VIO jumped0.323m
in33ms and later diverged to tens of metres. Only99 pre-fault pairs existed.

Nonessential raw IMU/local-position JSON logging was then decimated to reduce
probe load while preserving dense PX4 attitude, VIO and preview samples.
Slow90s fixture run `alignment_preview_20260917_203647` completed with no
preview fault:2407 pairs, yaw spans98.91deg(VIO preview)/98.27deg(PX4),
correlation0.999991, PX4-per-preview slope0.99568, fitted RMSE0.148deg.
Relative yaw direction and scale pass this preview gate. Position moved up to
10.47cm during fixture rotation and returned to1.31cm, so fixture centre versus
base_link/lever-arm isolation is not yet accepted. Motion Module warning
persisted despite continuous IMU/VIO. No EV injection, parameter write, mode
command or arming occurred. Full alignment, covariance and fusion remain
unaccepted.

## 2026-09-17 interactive dynamic session

Added stage/stop control and live freshness status to the bounded dynamic probe.
Four alignment-math unit tests pass. Runs 230137 and 230607 failed acquisition:
host received PX4 but no camera/VIO, although container tracking was state 1.
UDP-only alone did not fix this. A host subscription with localhost-only disabled
received tracking; process-local host ROS_LOCALHOST_ONLY=0 plus the existing UDP
profile restored all streams in alignment_preview_20260917_230747.
At readiness: 215 previews, all 215 tracking states=1, no fault, IMU/VIO/PX4
attitude receipt ages below 50 ms. Dynamic motion has not yet been accepted.
Session remains active for user-guided forward-out/return stages, capped at 900 s.
No PX4 params, external vision input, control commands, or EEPROM writes.

The same 230747 session was completed as a 0.70 m out/return test due to site
limits. At the measured far endpoint VIO was (0.7087, 0.0822, 0.0375) m: forward
scale was about +1.2%, but lateral/vertical coupling was 8.2/3.8 cm. On return,
the last plausible sample was (0.1214, 0.0250, 0.0075) m. The next frame 33.34 ms
later jumped to (-1.3522, 0.3470, 0.3330) m, a 1.543 m discontinuity, while
vo_state incorrectly remained 1. The guard stopped the session with
`VIO discontinuity; new session required`. Therefore forward scale is only a
partial observation; dynamic closure/alignment FAILED and EV input remains
prohibited. PX4 stayed disarmed and no FC input or parameter write occurred.

Retry `alignment_preview_20260917_231357` reproduced the failure. The 0.70 m
far endpoint was (0.6352, 0.0094, 0.0445) m, forward scale -9.3% (inside the
10% gate), with much smaller lateral coupling. On return the last plausible
sample was (0.1197, 0.0144, 0.0157) m; 33.33 ms later position jumped 1.071 m
to (-0.8935, 0.3582, 0.0647) m while orientation changed only 0.0253 rad and
vo_state remained 1. This repeatable position-only discontinuity makes dynamic
closure FAIL. The probe stopped safely; PX4 remained disarmed and isolated.

Before another motion run, `perception.launch.py` was changed to the official
multi-threaded component container and now explicitly sets
`enable_localization_n_mapping=false`. Status relay now records callback and
tracking timing. These changes are reversible and require a fresh static smoke
test; they do not enable FC input.

Post-change static preview `alignment_preview_20260917_232025` passed acquisition:
1439/1439 tracking states were 1, VIO 28.05 Hz and IMU 195.20 Hz. Over about
51 s of VIO output, final drift was 7.1 mm and max excursion 8.9 mm. Tracking
execution p50/p95/max was 15.0/29.1/38.2 ms. There were still 98 image-gap
warnings (max 100.03 ms), so the configuration is ready only for a controlled
reproduction run, not PX4 fusion.

The multi-threaded reproduction `alignment_preview_20260917_232216` is invalid.
At the 0.70 m endpoint the last pose was about (0.6439, 0.0096, 0.0394) m, then
odometry stopped for more than 34 s while IMU and status messages continued.
The camera log repeatedly reported `CUVSLAM has failed to register an IMU
measurement` and `Unknown Tracker Error 2`; vo_state misleadingly stayed 1.
The session was operator-stopped before return. `component_container_mt` was
therefore reverted to the original single-threaded `component_container`;
explicit `enable_localization_n_mapping=false` and timing telemetry remain.
The probe now fails closed if established VIO output is stale for over 1 s.

Single-thread visual-only isolation `alignment_preview_20260918_194847` passed a
complete measured 0.70 m out/return. Far endpoint was about
(0.6363, -0.0020, 0.0420) m (-9.1% forward scale); final position was
(-0.00377, 0.00860, 0.00276) m, giving 9.79 mm 3-D closure, and final attitude
residual was 0.764 deg. All 7091 odometry samples were continuous at 29.30 Hz;
the largest adjacent displacement was 4.43 mm. This strongly isolates the two
repeatable fused-run jumps to the D435i IMU/fusion path rather than visual scene
geometry. Decimated IMU stamps were monotonic in both single-thread fused runs
and visual-only; only the rejected multi-thread run had gross timestamp reorder
(54 non-monotonic saved intervals). Future probes now calculate every-message
IMU stamp min/max, non-monotonic and >20 ms gap counts. Fused dynamic alignment
remains FAILED and external vision remains prohibited.

Read-only calibration review: the September 14 six-face files contain exactly
1000 samples per orientation and the fitted acceleration vector RMSE is
0.1571 m/s^2; corrected norm is 9.8051+/-0.0804 m/s^2 on training data. The
live SDK matrix/bias previously matched this device calibration. However, the
official tool stores gyro means from pyrealsense motion samples directly, while
the same-version D400 parser documents EEPROM gyro bias as deg/s and multiplies
it by pi/180. No EEPROM rewrite was attempted.

Static fused copy-mode diagnostic `alignment_preview_20260918_200301` changed
only RealSense `unite_imu_method` from interpolation (2) to copy (1). It passed
with 1450/1450 state-1 VIO samples at 28.17 Hz and 10103 IMU samples at
195.21 Hz, without VIO or IMU-registration faults. Every-message IMU stamps
were monotonic, dt min/max 4.88/25.01 ms, with 27 gaps over 20 ms. Final/max
static displacement was 1.35/1.66 cm. Dynamic copy-mode validation is required.

Fused dynamic `alignment_preview_20260918_203457` used the reference
`camera_gyro_optical_frame` plus copy-mode IMU and completed the full 0.70 m
out/return without a discontinuity. Far endpoint x=0.6493 m (-7.3%); final
3-D closure=9.77 mm; final VIO attitude residual=2.89 deg; maximum adjacent
VIO displacement=13.8 mm. There were 8255 matched previews, all tracking state
1, and no stale/fault event. IMU stamps were monotonic (dt 4.86--50.00 ms;
210 gaps over 20 ms). PX4 alignment remains unaccepted because PX4 yaw is not
initialized/valid here and pairing is diagnostic only. A gyro-frame run with
interpolation mode 2 is required before selecting the production setting.

Final default-interpolation run `alignment_preview_20260918_204301` kept the
reference `camera_gyro_optical_frame` and used `unite_imu_method=2`. The 0.70 m
out/return completed without fault: far x=0.6093 m (-13.0% against the tape
mark), final 3-D closure=12.97 mm, final VIO attitude residual=0.904 deg,
maximum adjacent step=77.9 mm (below the 0.3 m guard). There were 12197 matched
previews, all state 1; every IMU stamp was monotonic (dt 4.89--50.02 ms; 308
gaps over 20 ms). PX4 yaw remains an arbitrary/uninitialized bench heading, so
the median absolute RPY differences are not an alignment acceptance; a relative
yaw fixture is the next gate. The production launch retains interpolation mode
2 and now uses the official gyro optical frame.

The RealSense ROS TF source shows the merged camera_imu frame is synthesized
from the gyro stream, while NVIDIA's same-version RealSense launch explicitly
sets cuVSLAM imu_frame to camera_gyro_optical_frame. The deployment had used
camera_imu_optical_frame; the launch now uses the reference gyro optical frame
for the next fused diagnostic. No sensor EEPROM or PX4 state was changed.

Current gyro-frame interpolation yaw run `alignment_preview_20260918_205215`
did not pass the rotational dynamic gate. The left 90 deg turn had no immediate
VIO fault but introduced about 24 cm translation, so it was not a pure rotation.
During the right-turn return, the pose near (0.0407,-0.0705,0.0319) m jumped
1.227 m in 33.34 ms to (0.6507,0.9863,-0.1008) m with only 0.0267 rad attitude
change. The guard stopped the session. All 7281 states still reported 1. Fused
IMU rotation closure remains FAILED; do not enable PX4 EV input.

Visual-only yaw isolation `alignment_preview_20260918_210104` had no VIO
discontinuity. Peak yaw was +91.29 deg (angle error 1.29 deg) and returned yaw
was +1.28 deg. However, manual rotation introduced 14.4 cm maximum translation
and 6.0 cm endpoint position residual, just outside the strict 5 cm endpoint
gate; it is a yaw-direction diagnostic, not a pure-rotation/extrinsic pass.
This supports visual-only as a safer software candidate than fused D435i IMU,
but does not authorize PX4 EV input or flight.

Candidate architecture is now encoded in launch defaults: D435i stereo
visual-only (`imu_fusion:=false`) plus later PX4 IMU EKF. Fused D435i IMU
rotation remains FAILED. Keep `camera_gyro_optical_frame` and
`unite_imu_method=2`. Because cuVSLAM already uses `base_frame=base_link` and
the inherited `base_link→camera_link` TF, `EKF2_EV_POS_X/Y/Z` must remain 0
until extrinsics are verified; copying `camera_xyz_rpy` into EV_POS would
double-count the lever arm. No PX4 parameters, EEPROM, or `/fmu/in` EV
publisher were changed. Next: static visual-only plus PX4 read-only bench,
then a mechanically centered yaw fixture; do not enable EV_CTRL writes.

Static visual-only PX4 read-only bench `alignment_preview_20260918_211620`
passed with the new default. cuVSLAM logged `Enable IMU Fusion: false`. There
were 1548 matched previews, all tracking state 1, VIO 29.18 Hz, IMU 194.08 Hz
(dt 4.88--90.03 ms, 46 gaps over 20 ms), PX4 attitude/local-position 99.74 Hz.
No flight-input publishers. Live TF confirmed `base_link→camera_link` =
(0.18, 0, -0.04) m, gyro/imu optical frames identical, and VIO child frame
`base_link`. Estimator: tilt aligned, yaw not aligned, `cs_ev_*` all false,
`xy_valid` false, `heading_good_for_control` false, range-height flag true but
`dist_bottom_valid` false. This accepts the software candidate at rest only.
A mechanically centered visual-only yaw fixture is still required before any
EV injection plan.

Centered visual-only yaw `alignment_preview_20260918_211936` had no VIO
discontinuity. Peak yaw +89.60 deg (angle error 0.40 deg); returned yaw
-1.24 deg. Endpoint XY 2.54 cm and post-return hold 1.88 cm pass the 5 cm
return gate; z peak-to-peak 1.66 cm; maximum adjacent step 15.5 mm. Max XY
from start 9.23 cm still fails the 5 cm isolation gate, so this is not a
pure-rotation/extrinsic pass. Improved versus handheld 210104 (14.4 cm max,
6.0 cm endpoint). 6258 matched previews, all state 1; IMU fusion remained
false; no `/fmu/in` publishers. Do not enable PX4 EV input.

Operator tape from PX4 center: front glass 0.20 m, D435i center 0.05 m
below, projector ~0.015 m to the right, PX4 = D435i module center. Launch
TF now uses camera_link (left IR optical) xyz=(0.196, 0.025, -0.05) m
after subtracting the 4.2 mm glass-to-depth origin and placing left IR
25 mm left of the 50 mm stereo midpoint. `sensor_extrinsics_verified`
remains false until the yaw isolation rerun. EKF2_EV_POS stays 0.

New-extrinsic yaw `alignment_preview_20260918_214135` had no VIO jump.
Peak yaw +89.82 deg. Left-turn max XY=4.08 cm. Operator-confirmed return
yaw=-0.62 deg, endpoint XY=3.85 cm, z ptp=8.1 mm, max step=9.5 mm. Full
out/return max XY=6.02 cm, so strict isolation still fails by 1 cm.
Post-report wait drifted to 12.8 cm and is not part of the gate. IMU
fusion false; no `/fmu/in`. Do not enable PX4 EV.

Visual-only lateral `alignment_preview_20260918_215519` completed the
0.70 m left out/return without a VIO jump. Far y=0.702 m (+0.28% vs tape);
final 3-D closure=4.70 cm; yaw residual=0.30 deg; max adjacent step=6.3 mm.
Outbound forward coupling was 6.4 cm. 6021 matched previews, fusion false,
no `/fmu/in`. Lateral scale/closure pass; not PX4 EV approval. Vertical
fixture is next.

Visual-only vertical `alignment_preview_20260918_220009` had no VIO jump.
Peak z=0.452 m against the operator-corrected 0.45 m cue (+0.4%); final 3-D closure=3.91 cm;
endpoint yaw=-1.30 deg; max adjacent step=4.0 mm. Maximum XY from start
was 12.4 cm with 5.3 deg yaw, so pure-vertical isolation is not passed.
Height direction/scale is usable as a diagnostic. No `/fmu/in`. Do not
enable PX4 EV.

Visual-only yaw retry `alignment_preview_20260918_220409` stopped on
operator confirm. 5674 matched previews, all tracking state 1, VIO
29.61 Hz, IMU 194.55 Hz, PX4 attitude 99.88 Hz, fusion false, no
`/fmu/in` publishers. Peak yaw +91.49 deg (error 1.49 deg). Left-turn
and full-path max XY=9.07 cm; endpoint XY=6.48 cm / yaw=1.54 deg;
z ptp=4.25 cm; max adjacent step=10.7 mm; no VIO jump. Isolation and
return XY fail; worse than 214135. Do not enable PX4 EV. Best remaining
yaw evidence is still 214135 (left 4.08 cm, full 6.02 cm).

Visual-only yaw retry `alignment_preview_20260918_220958` passed the
isolation gates. Operator-complete stop; 4583 matched previews, all
tracking state 1, VIO 29.66 Hz, IMU 194.74 Hz, PX4 attitude 99.91 Hz,
fusion false, no `/fmu/in` publishers. Peak yaw +90.15 deg (error
0.15 deg). Left-turn max XY=3.11 cm; full out/return max XY=4.69 cm;
endpoint XY=2.48 cm / yaw=1.78 deg; z ptp=1.32 cm; max adjacent
step=4.3 mm; no VIO jump. Tape TF `(0.196, 0.025, -0.05)` m is the
accepted VIO `odom→base_link` origin candidate. `EKF2_EV_POS` stays 0
so the lever arm is not applied twice. Still not EV injection or
flight. Next: read-only plan for later `EKF2_EV_CTRL` 15→9.

Bounded prop-off EV bench `ev_inject_20260918_222612` published 4208
FRD poses (quality 100, NaN velocity, no timesync offset, no range Z)
to `/fmu/in/vehicle_visual_odometry` only. Visual-only cuVSLAM; no
parameter write; arming_state remained 1. Estimator samples: ev_pos
265/265, ev_yaw 265/265, ev_hgt 0, ev_vel 0, xy_valid 265/265,
heading_good_for_control 0. `cs_yaw_align` stayed false, which matches
local FRD EV yaw. IMU fusion false; all 4238 tracking states 1; no VIO
jump. Session stopped by duration; publisher destroyed. Not flight.

EV-on lateral `ev_inject_20260919_180507` used the pose at operator
"start now" as origin. 10053 EV poses, all tracking state 1, fusion
false, arming unchanged. Outbound far XY=0.696 m (-0.6%); PX4 local
far XY=0.701 m. Return closure 20.3 cm VIO / 20.2 cm PX4, so EKF
followed VIO off the tape origin rather than losing EV. Flag samples:
ev_pos 632/632, ev_yaw 632/632, xy_valid 632/632, ev_hgt 0, ev_vel 0.
No discontinuity. Scale and EV hold pass; 5 cm closure does not. Not
flight.

EV-on lateral retry `ev_inject_20260919_185742` passed the translation
gates with EV held. Operator-complete; 5701 EV poses, all tracking
state 1, fusion false. Outbound far XY=0.675 m (-3.6% vs 0.70 m);
PX4 local far=0.686 m. VIO 3-D closure=3.51 cm / yaw=-0.31 deg; PX4
return XY=2.94 cm. Flag samples: ev_pos 359/359, ev_yaw 359/359,
xy_valid 359/359, ev_hgt 0, ev_vel 0. No VIO jump. Not flight. Next
prop-off gate is EV-on yaw isolation.

EV-on yaw `ev_inject_20260919_192538` held fusion through out/return.
3425 EV poses, all tracking 1. Peak yaw +91.11 deg (error 1.11 deg);
endpoint yaw=-0.16 deg / XY=3.28 cm; z ptp=1.38 cm; max step=2.5 mm.
Max XY=7.43 cm fails the 5 cm isolation gate. Flags: ev_pos/yaw/
xy_valid 217/217, ev_hgt 0, ev_vel 0. Angle, return, and EV hold pass;
strict in-place isolation does not. Not flight.

EV-on yaw retry `ev_inject_20260919_192932` passed isolation with EV
held. Operator-complete; 4364 EV poses, all tracking 1, fusion false.
Peak yaw +90.90 deg (error 0.90 deg). Left/full max XY=3.47 cm;
endpoint XY=3.22 cm / yaw=-1.11 deg; z ptp=0.88 cm; max step=5.9 mm.
Flags: ev_pos/yaw/xy_valid 275/275, ev_hgt 0, ev_vel 0. No VIO jump.
Better than 192538 (7.43 cm). Not flight. Next prop-off gate is EV-on
vertical 45 cm.

EV-on vertical `ev_inject_20260919_193307` passed with EV held.
Operator-complete; 6644 EV poses, all tracking 1, fusion false. Peak
z=0.438 m vs 0.45 m (-2.6%); 3-D closure=0.85 cm; endpoint yaw=1.34
deg; max step=3.3 mm. Handheld XY coupling 7.8 cm. PX4 local z ptp
6.2 cm (range height, `cs_ev_hgt` never true) while VIO z rose 43.8
cm; PX4 XY far=7.9 cm / return=0.83 cm, matching VIO. Flags:
ev_pos/yaw/xy_valid 417/417, ev_hgt 0, ev_vel 0. No VIO jump. Not
flight. Prop-off EV-on left 70 cm, yaw isolation, and up 45 cm are
complete. Production bridge and arming remain prohibited.

Candidate EV input is now a shared contract in `scripts/ev_odometry.py`
plus `scripts/ev_bridge.py`. Message fields copy the passed bench:
POSE_FRAME_FRD, VELOCITY_FRAME_FRD with NaN velocity, finite pose
variance, quality 100, ROS-clock timestamps with timesync offset
ignored, VIO Z retained. Inhibit on armed, extra `/fmu/in` publishers,
`cs_ev_hgt`, `cs_ev_vel`, and VIO discontinuity. `--prop-off` is
required; `live_flight_enabled` must stay false. Probe `--ev-inject`
calls the same builder so the harness cannot drift. Do not launch
`vslam_odom_bridge`. Next prop-off check is a static hold with the new
node while VIO is already up; not flight.

Candidate `ev_bridge` static hold `ev_bridge_static_20260919_195132`
passed. `--prop-off` 60 s; 1691 EV poses / 1720 previews; IMU fusion
false; duration_complete. Flag samples: ev_pos 57/57, ev_yaw 57/57,
ev_hgt 0, ev_vel 0, xy_valid 5620, heading_good_for_control 0. Own
publisher only; Agent 12316 kept. Not flight. User confirmed live
`EKF2_EV_CTRL=9`; the params export showing 15 is stale.

Range-height `ev_inject_20260919_195848` failed EKF usability.
Operator-complete; 6686 EV poses, fusion false, pos/yaw/xy_valid
420/420, hgt/vel 0. At rest `dist_bottom`~0.102 m, valid false,
bitfield 0, `cs_rng_hgt` true. Operator far: dist_bottom 56.8 cm
(+46.7 cm vs 45 cm) still valid false / bitfield 0; PX4 z delta
-35.9 cm; VIO z +26.6 cm. Return dist_bottom closure -0.15 cm; VIO
3-D closure 9.45 cm. Sample path peak dist_bottom 75.9 cm; PX4 z ptp
1.31 m. Numeric TFmini-like values moved with the lift, but
`dist_bottom_valid` never became true so range is not an accepted
height source. Not flight. Next: find why range is invalid (UART/
`SENS_TFMINI_CFG`, QGC distance, `dist_bottom_sensor_bitfield`),
without writing parameters in this step.

TFmini read-only: `SENS_TFMINI_CFG=102` is TELEM2; `UXRCE_DDS_CFG=101`
is TELEM1; `MAV_1_CONFIG=0` so TELEM2 is not also MAVLink. Hardware
must be on TELEM2 at 115200, not TELEM1. `SENS_TFMINI_HW=1` sets driver
min range 0.4 m (`hagl_min` 0.4); rest `dist_bottom`~0.10 m is below
that. PX4 1.16.2 sets `dist_bottom_valid` from terrain validity only
(`cs_rng_terrain` stayed false); bitfield follows `rng_terrain`, not
`rng_hgt`. Live `cs_rng_hgt` true, no rng fault; 6 s DDS saw 0
`/fmu/out/distance_sensor` samples (1.16 client does not publish that
topic). QGC MAVLink `DISTANCE_SENSOR` / `tfmini status` is the raw
check. Do not write parameters from this note.

Ground visual-height `ev_inject_20260919_201427` passed; parameters
unchanged. Operator-complete; 3988 EV poses; fusion false. Operator
far VIO z=0.456 m vs 0.45 m (+1.3%); path peak z=0.468 m (+4.0%);
3-D closure=2.30 cm; endpoint yaw=-0.55 deg; max step=6.7 mm.
Flags: ev_pos/yaw/xy_valid 251/251, ev_hgt 0, ev_vel 0. PX4 local z
ptp 0.92 m (still range height). No VIO jump. Not flight. Next
user-approved write: `EKF2_EV_CTRL` 9→11 (add EV height, keep
velocity off) and `EKF2_HGT_REF` 2→3 (vision). Then repeat the
ground lift with `cs_ev_hgt` expected true.
