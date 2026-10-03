# EV session protection and height review — 2026-09-21

## Fresh QGC export received (supersedes missing-export note below)

Source: `/home/cfly/.codex/attachments/942df7fd-702e-4a67-a3e2-5ad2da3b3def/param.params.txt`.
PX4 Pro 1.16.2, Git revision 54f0455ffc000000. SHA256
`96d1e682740a03bc32b919e89867c223034d54a7e27f73f428342822d13d4756`.
No parameter writes made.

Confirmed: EV_CTRL=11, HGT_REF=3, EV_POS_X/Y/Z=0, EV_QMIN=50,
EV_DELAY=0, BARO_CTRL=0, RNG_CTRL=2, GPS_CTRL=0, MAG_TYPE=5.
Thus vision is preferred height reference while range fusion is also enabled;
barometer is not a fused height fallback. Reference selection is not exclusive
source selection.

PX4 v1.16.2 official source was fetched directly (local checkout is 1.18-alpha
and was not used as the version authority):
- `src/modules/ekf2/EKF/aid_sources/range_finder/range_height_control.cpp`
  synthesizes ground range from rng_gnd_clearance and bypasses validity checks
  when range is unhealthy, regular data arrives and vehicle is on ground.
- `src/drivers/distance_sensor/tfmini/TFMINI.cpp` sets minimum range 0.4 m
  for hardware model 1. Export SENS_TFMINI_HW=1, EKF2_MIN_RNG=0.10 m.
  Ground 0.10 m valid telemetry is consistent with synthesized measurement;
  exact branch execution is not proved without raw sensor/EKF diagnostics.
- `src/modules/commander/commander_params.c` confirms enumerations below.
All source URLs use https://raw.githubusercontent.com/PX4/PX4-Autopilot/v1.16.2/.

Preflight issues requiring a reviewed plan, not immediate parameter writes:
- COM_OBL_RC_ACT=0: Offboard-loss configured response Position, not Land;
  COM_OF_LOSS_T=1 s. Invalid-position fallbacks also apply.
- COM_POSCTL_NAVL=0: manual Position navigation-loss fallback Altitude.
- COM_RC_OVERRIDE=1: auto-mode stick override only, not Offboard bit 1.
  Mode-switch takeover must be independently tested.
- RC_MAP_FLTMODE=5 and RC_MAP_KILL_SW=5: shared channel mapping; inspect
  actual switch thresholds/positions before interpreting or changing it.
- NAV_RCL_ACT=2: RC-loss Return; indoor return prerequisites/path unvalidated.
- COM_LOW_BAT_ACT=0: warning only, not automatic low-battery landing.
- MPC_TKO_SPEED=1.5 m/s, MPC_LAND_SPEED=0.7 m/s; review first-flight limits
  with actual command mode. These do not universally constrain Offboard commands.
- EKF2_RNG_POS_Z=+0.05 m (body down): verify against physical rangefinder;
  do not confuse this sensor with the separate reported upper-mounted lidar.

Next operator evidence: QGC MAVLink Console read-only
`listener distance_sensor -n 5` and `listener vehicle_air_data -n 5`, with
vehicle stationary on ground; sensor model and downward installation confirmed.
No lifting needed yet. These checks precede any proposal to enable barometer
fusion or change RC/failsafe settings. Full parameter audit complete; physical
fallback-height acceptance remains incomplete.

## Implemented, offline verified

`scripts/ev_session_guard.py` implements a ROS-free latched gate. Tracking
failure, duplicate/backward source timestamps, tracking/VIO receipt staleness
over 0.3 s, invalid pose frame/sample age, and timestamp disagreement latch a
fault. Startup missing data expires after 10 s. No reset method exists: a new
process/session and reviewed coordinate relationship are required. These are
bench defaults, not flight-certified timing limits.

`scripts/ev_bridge.py` now uses this gate, a 50 ms watchdog (graph audit remains
0.5 s), and requires fresh disarmed vehicle status and estimator flags before
publishing. Loss of these safety telemetry streams for >2 s after publishing
ends the session. Rejected EV quality ends the session. Existing discontinuity,
prop-off and live-flight prohibitions remain. A ROS executor stall can delay
watchdog execution; this is not an independent hardware failsafe.

15 unit tests passed (existing EV contract plus session guard tests). Source
compilation passed. Offline replay of measured pose sequences:

- Occlusion run `ev_inject_20260921_183809`: 1490 poses, first rejected pose
  index 1476 (zero based), discontinuity latched, zero subsequent acceptance.
- Restart static run `alignment_preview_20260921_184124`: 1567 poses, no fault.

Replay supplied ideal same-stamp tracking state 1, consistent with recorded
states, to test rejection even with nominal status. It does NOT reproduce DDS
callback ordering/latency or validate the ROS adapter under injected faults.
New bridge was not launched against the FC. Historical probe script retains
its earlier diagnostic implementation; it is not evidence for the new gate.
Still needed: prop-off runtime verification of `ev_bridge` under real
`vo_state` loss. Adapter fault-injection tests now cover tracking failure,
jumps while state stays 1, stale tracking, armed, extra `/fmu/in`, telemetry
timeout, recorded occlusion (no resume) and static replay (no fault).
This is not DDS callback-order certification. No in-air recovery approved.

## Live read-only height observation

`evidence/px4_telemetry_20260921_184526/report.json`, 20 s collection:
no /fmu/in publisher, disarmed. Final flags: range-height and terrain active,
baro-height inactive, EV position/yaw/height inactive, inertial dead reckoning
active. Final xy_valid=false, z_valid=true, dist_bottom approximately 0.100 m,
dist_bottom_valid=true. This is estimator status, not physical range accuracy.

Only parameter export found: `/home/cfly/param.params.txt`, Sep 17, already known
stale (EV_CTRL=15/HGT_REF=2). Sep 19 bench notes report EV_CTRL=11/HGT_REF=3,
but current parameter values cannot be proven from estimator flags. No known
live MAVLink parameter endpoint was found; TELEM1 is occupied by the persistent
XRCE Agent. Do not repurpose the unidentified ttyUSB0, reconfigure UARTs, or
stop the Agent to obtain parameters.

Needed from operator: QGC fresh parameter export saved as
`/home/cfly/param_current_20260921.params` (do not overwrite old export).
Review EKF2_HGT_REF, EV_CTRL, BARO_CTRL, RNG_CTRL, RNG_A_HMAX, EV_DELAY,
EV_POS_*, COM_OF_LOSS_T, COM_OBL_RC_ACT, COM_POSCTL_NAVL, COM_POS_LOW_ACT,
COM_RC_LOSS_T, COM_RCL_EXCEPT, NAV_RCL_ACT, COM_DL_LOSS_T, NAV_DLL_ACT.
Check exact PX4 1.16.2 semantics before proposing changes.

No parameter changes proposed yet. Range accuracy needs independent height
measurements inside documented sensor limits, sensor orientation/model and
installation offset confirmation. Neither numeric validity nor barometer
enablement alone proves a safe landing fallback. Flight remains disabled.
