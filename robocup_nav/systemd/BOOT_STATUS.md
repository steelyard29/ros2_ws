# Boot deployment status — 2026-10-01

## Latest resident-EV candidate (supersedes the historical missing-implementation notes below)

Latest bounded EV+independent-flags run (resident_joint_773e2f73a2fa4f77a7a8aafe24a30d30)
completed with service exit0, 1316 EV, 43.302s sampled continuous position/yaw/height
fusion and 1309 timestamp pairs. Both flags receivers had max gaps about1.011s.
PX4 operator status during this window showed RX3453 B/s and sync converged.
The service was deliberately stopped at the test deadline; this is not persistent
operation or cold-boot evidence. No new unit installation is confirmed. Earlier
failure descriptions below remain historical evidence, not the latest outcome.

`robocup-ev-boot.service` and `scripts/boot_resident_ev.sh` now exist and pass
shell/systemd static validation. Requires sensor and DDS units, starts only the
EV owner, uses ground/disarmed startup checks and the explicit 16MiB perception
profile. No arm/mode/trajectory/servo output; Restart=no; faults stay stopped.
Stop sends SIGTERM to the owner first (KillMode=mixed), allowing reader cleanup.
The service does not bypass clock synchronization or modify PX4 parameters.

NOT INSTALLED OR COLD-BOOT VERIFIED. Read-only checks still show sensor/DDS
enabled but inactive; their current manually started processes are separate.
This account requires local sudo authentication. Do not use --now or reboot
over the running stack. The latest real EV run produced 610 EV, 18.135 seconds
of sampled position/yaw/height fusion and 603 timestamp pairs, but stopped on
flags telemetry timeout after about 23.9 seconds. This candidate is therefore
not evidence of reliable unattended operation. It will fail closed on that fault.

If registering this fail-closed candidate for a later supervised ground boot,
the operator can execute locally (no service start now):

```bash
sudo systemctl enable /home/cfly/ros2_ws/robocup_nav/systemd/robocup-ev-boot.service
sudo systemctl daemon-reload
systemctl is-enabled robocup-sensors-boot.service robocup-dds-boot.service robocup-ev-boot.service
```

Rollback of future startup only (does not stop current localization):

```bash
sudo systemctl disable robocup-ev-boot.service
```

Automatic ground reference assumes stationary ground boot, as confirmed by
operator. New boot/session requires new task pairing; never reuse prior alignment.
Enabling this unit is not flight approval. Continuous fusion and supervised
cold-boot acceptance remain pending. The following sections preserve older history.

REGISTERED, NOT COLD-BOOT VALIDATED: operator installed/enabled both units on
2026-10-01. Agent read-only systemctl verification confirms both UnitFileState=enabled,
ActiveState=inactive, SubState=dead. No service start or reboot was performed.
No persistent EV service is implemented/enabled yet. Do not claim boot fusion
or flight readiness. Existing manually started processes are separate.

Operator confirmed every boot occurs disarmed, stationary on the ground, with
the boot location as the mission origin; no airborne Jetson reboot. This is an
operational constraint, not a physical ground detector. Existing VIO odom topics
are not arbitrarily zeroed; session transforms establish the task origin.

## Prepared layers

- `robocup-dds-boot.service`: foreground Agent on Telem1 `/dev/ttyTHS1`, 921600,
  domain 0, existing bridge profile. Refuses an existing Agent/occupied port.
  No restart loop, no mode/arming/EV outputs. Stopping an active Agent interrupts
  DDS; do not restart/stop it during flight. This does not configure PX4 firmware.
- `robocup-sensors-boot.service`: reuses the bounded 90-second recovery with
  D435i/visual-only cuVSLAM, downward/H candidates, lidar/TF, nvblox and A*/APF.
  Requires host clock synchronization and an inspected exited container.
  Startup success retains the sensors; failure stops only its inspected owned
  container, as in the existing recovery contract. No restart loop, EV or goals.
  `active (exited)` means startup completed, NOT continuous health verification.
  Stopping this oneshot unit does NOT stop the processes retained in Docker.
  To stop perception, inspect ownership and use the existing scoped shutdown;
  never blindly stop a shared container or restart it in flight.

## Still required for complete boot fusion / continuous EV

1. Resolve the observed VIO future timestamp (~1.955 seconds); do not relabel it
   as current or widen the guard. System clock currently reports synchronized,
   but that alone does not establish the camera timestamp mapping.
2. Implement/test a unique resident EV owner and task-only control handoff,
   including lifetime, failure latch, shutdown and session/reset binding.
   Existing task runtime still owns EV and MUST NOT run beside another EV writer.
3. System service installation needs local administrator authentication. This
   account has no passwordless sudo and `Linger=no`; user service enablement
   alone cannot be advertised as login-independent startup.
4. Test an actual cold boot later while disarmed on the ground; no reboot was
   performed as part of preparing these files. No automatic flight is included.

Do not install with `--now` over the currently running Agent/sensor stack.
Any later enablement must be recorded with exact units and rollback commands.
Disabling startup affects future boots only and must not terminate flight-time
localization. No reboot or automatic retry is authorized by this document.

## Optional staged installation (operator local terminal)

This registers ONLY the two prepared sensor/DDS layers for future boots. It
does not start them now, install a resident EV service, or prove cold-boot success.
Use only if accepting this partial deployment; no password belongs in chat.

```bash
sudo systemctl enable /home/cfly/ros2_ws/robocup_nav/systemd/robocup-sensors-boot.service /home/cfly/ros2_ws/robocup_nav/systemd/robocup-dds-boot.service
systemctl is-enabled robocup-sensors-boot.service robocup-dds-boot.service
```

Rollback future startup without stopping any current process:

```bash
sudo systemctl disable robocup-sensors-boot.service robocup-dds-boot.service
```

No `--now`, no reboot, and no promise of automatic fusion. Return actual command
results for the progress record; agent has not performed this installation.
