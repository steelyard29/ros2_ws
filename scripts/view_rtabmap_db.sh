#!/bin/bash
# Open rtabmap.db with RTAB-Map Database Viewer (container 0.23.x).
# SLAM runs in isaac_ros_dev; the host ROS package is often older and cannot
# open databases created by the container.

set -e

CONTAINER_NAME="${CONTAINER_NAME:-isaac_ros_dev}"
WORKSPACE="${ROS2_WS:-$HOME/ros2_ws}"
DB_PATH="${1:-$WORKSPACE/maps/production/rtabmap.db}"
DISPLAY="${DISPLAY:-:0}"

if [ ! -f "$DB_PATH" ]; then
    echo "Database not found: $DB_PATH" >&2
    exit 1
fi

DB_PATH="$(realpath "$DB_PATH")"
WORKSPACE="$(realpath "$WORKSPACE")"
case "$DB_PATH" in
    "$WORKSPACE"/*)
        CONTAINER_DB="/workspaces/ros2_ws/${DB_PATH#"$WORKSPACE"/}"
        ;;
    *)
        echo "Database must be inside workspace $WORKSPACE: $DB_PATH" >&2
        exit 1
        ;;
esac

if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "Starting container ${CONTAINER_NAME}..."
    docker start "$CONTAINER_NAME" >/dev/null
    sleep 2
fi

xhost +local:docker >/dev/null 2>&1 || true

exec docker exec -it \
    -e DISPLAY="$DISPLAY" \
    -e QT_X11_NO_MITSHM=1 \
    "$CONTAINER_NAME" \
    bash -lc "source /opt/ros/humble/setup.bash && exec rtabmap-databaseViewer '$CONTAINER_DB'"
