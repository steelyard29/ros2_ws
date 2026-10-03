#!/usr/bin/env bash
set -euo pipefail

container=isaac_ros_dev
container_script=/workspaces/ros2_ws/isaac_vio_debug/scripts/container_stack.sh
action="${1:-status}"

if ! docker ps -a --format '{{.Names}}' | grep -Fxq "$container"; then
  echo "Required existing container '$container' does not exist." >&2
  echo "This script never creates or rebuilds containers." >&2
  exit 1
fi

if ! docker ps --format '{{.Names}}' | grep -Fxq "$container"; then
  if [[ "$action" != start ]]; then
    echo "Container '$container' is stopped; only 'start' may restart it." >&2
    exit 1
  fi
  docker start "$container" >/dev/null
fi

docker exec "$container" bash "$container_script" "$@"

