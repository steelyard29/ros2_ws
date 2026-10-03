#!/usr/bin/env bash
# Explicit foreground processes; Ctrl-C stops only this invocation.
set -eo pipefail
root_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
action="${1:-help}"
if [[ $# -gt 0 ]]; then shift; fi
case "$action" in
  flight-release)
    exec python3 "$root_dir/scripts/flight_release.py" "$@"
    ;;
  calibration-review)
    exec python3 "$root_dir/scripts/calibration_review.py" "$@"
    ;;
  disarmed-handshake-check)
    # Offline only: no ROS environment, serial port or GPIO.
    exec python3 "$root_dir/scripts/disarmed_handshake_check.py" "$@"
    ;;
  flight-switch-contract)
    exec python3 "$root_dir/scripts/flight_switch_contract.py" "$@"
    ;;
  flight-review)
    exec python3 "$root_dir/scripts/flight_readiness.py" "$@"
    ;;
  flight-test)
    exec python3 -m unittest discover -s "$root_dir/tests" -p 'test_flight*.py' -v
    ;;
  live-flight|live-flight-check|fault-bench-dds-check|fault-bench|handshake-bench|handshake-runtime|handshake-dds-check|flight-bench-check|flight-bench|aux-observe|flight-runtime|flight-runtime-check|flight-dds-check|flight-ev-shadow|preflight-observe)
    source /opt/ros/humble/setup.bash
    source /home/cfly/ros2_ws/install/setup.bash
    case "$action" in
      live-flight) exec python3 "$root_dir/scripts/live_flight_runtime.py" "$@" ;;
      live-flight-check) exec python3 "$root_dir/scripts/live_flight_check.py" "$@" ;;
      fault-bench) exec python3 "$root_dir/scripts/fault_bench_runtime.py" "$@" ;;
      fault-bench-dds-check) exec python3 "$root_dir/scripts/fault_bench_dds_check.py" "$@" ;;
      handshake-bench) exec python3 "$root_dir/scripts/handshake_bench_runtime.py" "$@" ;;
      handshake-runtime) exec python3 "$root_dir/scripts/handshake_runtime.py" "$@" ;;
      handshake-dds-check) exec python3 "$root_dir/scripts/handshake_dds_check.py" "$@" ;;
      flight-bench-check) exec python3 "$root_dir/scripts/flight_bench_check.py" "$@" ;;
      flight-bench) exec python3 "$root_dir/scripts/flight_bench_runtime.py" "$@" ;;
      aux-observe) exec python3 "$root_dir/scripts/aux_switch_observer.py" "$@" ;;
      flight-runtime) exec python3 "$root_dir/scripts/flight_runtime.py" "$@" ;;
      flight-runtime-check) exec python3 "$root_dir/scripts/flight_runtime_check.py" "$@" ;;
      flight-dds-check) exec python3 "$root_dir/scripts/flight_dds_check.py" "$@" ;;
      flight-ev-shadow) exec python3 "$root_dir/scripts/flight_ev_shadow.py" "$@" ;;
      preflight-observe) exec python3 "$root_dir/scripts/preflight_observer.py" "$@" ;;
    esac
    ;;
  mission-replay)
    # Pure Python evidence replay; never sources ROS or creates publishers.
    exec python3 "$root_dir/scripts/competition_replay.py" "$@"
    ;;
  mission-test)
    exec python3 -m unittest discover -s "$root_dir/tests" -p 'test_competition*.py' -v
    ;;
  offline-apf-h|offline-navigation|offline-apf-faults|offline-sortie-h|offline-ready-sortie-h)
    # Synthetic inputs only; never starts camera, Agent, GPIO or flight route.
    exec docker exec -i isaac_ros_dev bash /workspaces/ros2_ws/robocup_nav/run.sh "container-$action" "$@"
    ;;
  container-offline-apf-h|container-offline-navigation|container-offline-apf-faults|container-offline-sortie-h|container-offline-ready-sortie-h)
    source /workspaces/ros2_ws/isaac_vio_debug/scripts/container_env.sh
    export ROS_DOMAIN_ID=176 ROS_LOCALHOST_ONLY=1
    export PYTHONPATH="/workspaces/ros2_ws/build/px4_msgs/rosidl_generator_py:${PYTHONPATH:-}"
    export LD_LIBRARY_PATH="/workspaces/ros2_ws/install/px4_msgs/lib:${LD_LIBRARY_PATH:-}"
    if [[ "$action" == container-offline-ready-sortie-h ]]; then
      exec python3 "$root_dir/tests/sortie_ready_h_closed_loop.py" "$@"
    elif [[ "$action" == container-offline-sortie-h ]]; then
      exec python3 "$root_dir/tests/sortie_h_closed_loop.py" "$@"
    elif [[ "$action" == container-offline-apf-faults ]]; then
      exec python3 "$root_dir/tests/path_apf_fault_check.py" "$@"
    elif [[ "$action" == container-offline-apf-h ]]; then
      exec python3 "$root_dir/tests/legacy_apf_h_closed_loop.py" "$@"
    else
      exec python3 "$root_dir/tests/nvblox_closed_loop.py" "$@"
    fi
    ;;
  nvblox)
    if [[ ! -t 0 || ! -t 1 ]]; then
      echo 'nvblox requires an interactive terminal; use nvblox-smoke for a bounded test.' >&2
      exit 1
    fi
    exec docker exec -it isaac_ros_dev bash /workspaces/ros2_ws/robocup_nav/run.sh container-nvblox "$@"
    ;;
  nvblox-smoke|nvblox-test)
    exec docker exec -i isaac_ros_dev bash /workspaces/ros2_ws/robocup_nav/run.sh "container-$action" "$@"
    ;;
  container-nvblox|container-nvblox-smoke|container-nvblox-test)
    source /workspaces/ros2_ws/isaac_vio_debug/scripts/container_env.sh
    # Isolate shadow/test topics from the existing flight graph.
    export ROS_DOMAIN_ID=176 ROS_LOCALHOST_ONLY=1
    case "$action" in
      container-nvblox) exec ros2 launch "$root_dir/launch/nvblox_stack.launch.py" "$@" ;;
      container-nvblox-smoke) exec python3 "$root_dir/tests/nvblox_smoke.py" "$@" ;;
      container-nvblox-test)
        # Existing host symlink-install points at /home/cfly, not the container
        # mount. Use its actual generated Python tree for this isolated test.
        export PYTHONPATH="/workspaces/ros2_ws/build/px4_msgs/rosidl_generator_py:${PYTHONPATH:-}"
        export LD_LIBRARY_PATH="/workspaces/ros2_ws/install/px4_msgs/lib:${LD_LIBRARY_PATH:-}"
        exec python3 "$root_dir/tests/nvblox_navigation_test.py" "$@" ;;
    esac
    ;;
  perception)
    # Existing container only, inspected entrypoint. No PX4/actuator startup.
    if [[ "$(docker inspect -f '{{.State.Running}}' isaac_ros_dev)" != true ]]; then
      echo 'Start the existing isaac_ros_dev container before this command.' >&2
      exit 1
    fi
    if [[ ! -t 0 || ! -t 1 ]]; then
      echo 'perception requires an interactive terminal for Ctrl-C forwarding; use smoke for a bounded test.' >&2
      exit 1
    fi
    exec docker exec -it isaac_ros_dev bash /workspaces/ros2_ws/robocup_nav/run.sh container-perception "$@"
    ;;
  container-perception)
    source /workspaces/ros2_ws/isaac_vio_debug/scripts/container_env.sh
    exec ros2 launch "$root_dir/launch/perception.launch.py" "$@"
    ;;
  smoke)
    exec docker exec -i isaac_ros_dev bash /workspaces/ros2_ws/robocup_nav/run.sh container-smoke "$@"
    ;;
  container-smoke)
    source /workspaces/ros2_ws/isaac_vio_debug/scripts/container_env.sh
    exec python3 "$root_dir/scripts/smoke_perception.py" "$@"
    ;;
  sitl-takeoff)
    # Host-only PX4 DDS-contract SITL. Never docker-exec, never serial, never camera.
    source /opt/ros/humble/setup.bash
    source /home/cfly/ros2_ws/install/setup.bash
    export PYTHONPATH="$root_dir/scripts:${PYTHONPATH:-}"
    export ROS_DOMAIN_ID="${SITL_DOMAIN_ID:-175}"
    export ROS_LOCALHOST_ONLY=1
    exec python3 "$root_dir/scripts/sitl_takeoff_land.py" --offline-sitl "$@"
    ;;
  sitl-safety)
    source /opt/ros/humble/setup.bash
    source /home/cfly/ros2_ws/install/setup.bash
    export PYTHONPATH="$root_dir/scripts:${PYTHONPATH:-}"
    export ROS_DOMAIN_ID="${SITL_DOMAIN_ID:-175}"
    export ROS_LOCALHOST_ONLY=1
    exec python3 "$root_dir/tests/sitl_safety.py" --offline-sitl "$@"
    ;;
  mapping|navigation|geometry|preview|test|ev-bridge)
    source /opt/ros/humble/setup.bash
    source /home/cfly/ros2_ws/install/setup.bash
    export PYTHONPATH="$root_dir/scripts:${PYTHONPATH:-}"
    case "$action" in
      mapping) exec ros2 launch "$root_dir/launch/mapping.launch.py" "$@" ;;
      navigation) exec ros2 launch "$root_dir/launch/navigation.launch.py" "$@" ;;
      geometry) exec python3 "$root_dir/scripts/geometry_bridge.py" "$@" ;;
      preview) exec python3 "$root_dir/scripts/shadow_bridge.py" "$@" ;;
      test) exec python3 "$root_dir/tests/integration.py" "$@" ;;
      ev-bridge) exec python3 "$root_dir/scripts/ev_bridge.py" "$@" ;;
    esac
    ;;
  *)
    echo '       bash robocup_nav/run.sh offline-apf-h --controller path-apf --scene two-boxes --scan-beams 720 # SYNTHETIC ONLY, domain 176'
    echo '       bash robocup_nav/run.sh offline-navigation --controller path-apf --scene wall # SYNTHETIC ONLY, no cameras/PX4'
    echo '       bash robocup_nav/run.sh offline-apf-faults # SYNTHETIC ONLY, run separately in domain 176'
    echo '       bash robocup_nav/run.sh offline-sortie-h --controller path-apf --scene two-boxes --scan-beams 720 # GROUND-START SYNTHETIC DDS ONLY'
    echo '       bash robocup_nav/run.sh nvblox [center_z:=0.0] [navigation:=false] # SHADOW ONLY'
    echo '       bash robocup_nav/run.sh nvblox-smoke [--live --duration 60]'
    echo '       bash robocup_nav/run.sh nvblox-test'
    echo 'Usage: bash robocup_nav/run.sh perception [rgbd:=true]'
    echo '       bash robocup_nav/run.sh flight-test'
    echo '       bash robocup_nav/run.sh flight-release --template # blank, UNAPPROVED release; offline'
    echo '       bash robocup_nav/run.sh live-flight-check # synthetic domain 177, no hardware'
    echo '       bash robocup_nav/run.sh live-flight --help # REAL FLIGHT; requires completed release + interactive authorization'
    echo '       bash robocup_nav/run.sh calibration-review --params /absolute/current.params [--baseline /absolute/before.params] # offline only'
    echo '       bash robocup_nav/run.sh disarmed-handshake-check # offline only; no ROS/GPIO/PX4'
    echo '       bash robocup_nav/run.sh handshake-runtime --exercise-controller --aux-params /absolute/current.params # SHADOW ONLY'
    echo '       bash robocup_nav/run.sh handshake-dds-check # domain 177 synthetic, NO real control'
    echo '       bash robocup_nav/run.sh handshake-bench --help # REQUIRES SEPARATE AUTHORIZATION; disarmed mode only, NEVER ARM'
    echo '       bash robocup_nav/run.sh fault-bench --help # SEPARATE AUTHORIZATION; Position kill-stop only, NO commands'
    echo '       bash robocup_nav/run.sh fault-bench-dds-check # domain 177 synthetic; no hardware'
    echo '       bash robocup_nav/run.sh flight-bench --prop-off --aux-params /absolute/current.params # real EV only, NO control'
    echo '       bash robocup_nav/run.sh flight-bench-check --prop-off --params /absolute/current.params # bounded camera+EV-only'
    echo '       bash robocup_nav/run.sh aux-observe --duration 20 # subscription-only raw AUX evidence'
    echo '       bash robocup_nav/run.sh flight-switch-contract [--format patch] # read-only, no build/flash'
    echo '       bash robocup_nav/run.sh flight-runtime --duration 60  # integrated shadow-only runtime'
    echo '       bash robocup_nav/run.sh flight-runtime-check  # full-chain isolated DDS scenarios'
    echo '       bash robocup_nav/run.sh flight-dds-check  # domain 177, synthetic plant, shadow topics only'
    echo '       bash robocup_nav/run.sh flight-ev-shadow --duration 60  # VIO already running'
    echo '       bash robocup_nav/run.sh preflight-observe --duration 30  # subscription only'
    echo '       bash robocup_nav/run.sh flight-review --params /absolute/current.params'
    echo '       bash robocup_nav/run.sh mission-replay --demo nominal  # offline synthetic evidence'
    echo '       bash robocup_nav/run.sh mission-replay --input snapshots.jsonl  # no ROS/devices'
    echo '       bash robocup_nav/run.sh mission-test'
    echo '       bash robocup_nav/run.sh mapping database:=/absolute/NEW/map.db'
    echo '       bash robocup_nav/run.sh navigation [demo_geometry:=true OFFLINE ONLY]'
    echo '       bash robocup_nav/run.sh geometry [--ros-args -p demo_geometry:=true OFFLINE ONLY]'
    echo '       bash robocup_nav/run.sh preview'
    echo '       bash robocup_nav/run.sh ev-bridge --prop-off [--duration 60]  # VIO already up; not flight'
    echo '       bash robocup_nav/run.sh test'
    echo '       bash robocup_nav/run.sh sitl-takeoff  # offline takeoff/hover/land; no real FC'
    echo '       bash robocup_nav/run.sh sitl-safety   # estop/link-loss/timeout; no real FC'
    echo '       bash robocup_nav/run.sh smoke --stereo-only   # 45 s dual-IR+IMU+VIO, no RGB-D/map'
    echo '       bash robocup_nav/run.sh smoke --rgbd-no-map   # 45 s RGB-D+IMU+VIO, no RTAB-Map'
    echo '       bash robocup_nav/run.sh smoke                # 45 s RGB-D+IMU+VIO+mapping; no PX4'
    ;;
esac
