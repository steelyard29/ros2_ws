#!/bin/bash
# Launch Isaac ROS dev container on Jetson Orin Nano
# Usage:
#   ./run_isaac_dev.sh          - start or re-enter persistent container
#   ./run_isaac_dev.sh reset    - delete old container and create fresh one
#   ./run_isaac_dev.sh reset -d - delete old container and start detached
#
# Inside the container, launch Visual SLAM + D435i with:
#   source /opt/ros/humble/setup.bash
#   ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam_realsense.launch.py

WORKSPACE_DIR="$HOME/ros2_ws"
CONTAINER_WS="/workspaces/ros2_ws"
YOLO_WORKSPACE_DIR="${YOLO_WORKSPACE_DIR:-$HOME/yolo_ws}"
CONTAINER_YOLO_WS="/workspaces/yolo_ws"
CONTAINER_NAME="isaac_ros_dev"
IMAGE="${ISAAC_ROS_IMAGE:-isaac_ros_dev-ros2ws:latest}"
DETACH=false

for arg in "$@"; do
    case "$arg" in
        reset)
            ;;
        -d|--detach)
            DETACH=true
            ;;
        *)
            WORKSPACE_DIR="$arg"
            ;;
    esac
done

if [ ! -t 0 ]; then
    DETACH=true
fi

if [ "${1:-}" = "reset" ]; then
    echo "Removing old container..."
    docker rm -f "$CONTAINER_NAME" 2>/dev/null
fi

# If container exists (stopped), restart and attach
if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "Starting existing container '${CONTAINER_NAME}'..."
    docker start "$CONTAINER_NAME" >/dev/null
    if [ "$DETACH" = false ]; then
      docker exec -it "$CONTAINER_NAME" /bin/bash
    fi
else
    echo "Creating new container '${CONTAINER_NAME}'..."
    DOCKER_TTY="-it"
    CONTAINER_CMD="/bin/bash"
    if [ "$DETACH" = true ]; then
      DOCKER_TTY="-d"
      CONTAINER_CMD="sleep infinity"
    fi
    DOCKER_YOLO_MOUNT=()
    if [ -d "$YOLO_WORKSPACE_DIR" ]; then
      DOCKER_YOLO_MOUNT=(-v "$YOLO_WORKSPACE_DIR":"$CONTAINER_YOLO_WS")
      echo "Mounting optional YOLO workspace: $YOLO_WORKSPACE_DIR -> $CONTAINER_YOLO_WS"
    else
      echo "YOLO workspace not found at $YOLO_WORKSPACE_DIR; vision detection will be unavailable."
    fi
    docker run $DOCKER_TTY \
      --name "$CONTAINER_NAME" \
      --init \
      --runtime nvidia \
      --network host \
      --privileged \
      -e DISPLAY=$DISPLAY \
      -v /tmp/.X11-unix/:/tmp/.X11-unix \
      -v "$WORKSPACE_DIR":"$CONTAINER_WS" \
      -v /dev:/dev \
      "${DOCKER_YOLO_MOUNT[@]}" \
      "$IMAGE" \
      $CONTAINER_CMD
fi
