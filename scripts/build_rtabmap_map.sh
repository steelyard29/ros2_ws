#!/bin/bash
# Build and promote RTAB-Map databases without starting PX4 bridge/control.
#
# 2026-09-14 起：建图只用 D435i RGB-D（视觉），不再接 2D 激光
# （激光 SLAM/scan_planar 已整体移除，RPLidar 仅保留给 /scan 避障）。

set -euo pipefail

ROS2_WS="${ROS2_WS:-$HOME/ros2_ws}"
CONTAINER_NAME="${CONTAINER_NAME:-isaac_ros_dev}"
CONTAINER_WS="${CONTAINER_WS:-/workspaces/ros2_ws}"
MAP_ROOT="$ROS2_WS/maps"
DRAFT_DIR="$MAP_ROOT/drafts"
PRODUCTION_DIR="$MAP_ROOT/production"
PRODUCTION_DB="$PRODUCTION_DIR/rtabmap.db"
LOG_DIR="${RTABMAP_BUILD_LOG_DIR:-/tmp/rtabmap_map_builder_logs}"
STATE_DIR="/tmp/rtabmap_map_builder_${UID}"
STATE_FILE="$STATE_DIR/state"
MIN_NODES="${RTABMAP_VALIDATE_MIN_NODES:-10}"
MAX_NEIGHBOR_STEP_M="${RTABMAP_VALIDATE_MAX_STEP_M:-2.0}"
ROS_DISCOVERY_SPIN_TIME="${ROS_DISCOVERY_SPIN_TIME:-3}"
# 建图时 rtabmap stereo_odometry 的 IMU 来源（none | d435 | px4），透传给容器 launch
RTABMAP_IMU_SOURCE="${RTABMAP_IMU_SOURCE:-none}"

mkdir -p "$DRAFT_DIR" "$PRODUCTION_DIR" "$LOG_DIR" "$STATE_DIR"
chmod 700 "$STATE_DIR"

log() { printf '[%s] %s\n' "$(date +'%H:%M:%S')" "$*"; }
die() { printf '[ERROR] %s\n' "$*" >&2; exit 1; }

source_ros2_env() {
    # ROS setup.bash references unset vars (e.g. AMENT_TRACE_SETUP_FILES).
    set +u
    # shellcheck disable=SC1091
    [ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash
    # shellcheck disable=SC1091
    [ -f "$ROS2_WS/install/setup.bash" ] && source "$ROS2_WS/install/setup.bash"
    set -u
}

usage() {
    cat <<EOF
Usage:
  $0 start                  Create and map into a new timestamped draft
  $0 stop                   Stop only the mapping workflow
  $0 force-stop             Force-kill mapping workflow if stop times out
  $0 status                 Show mapping state and latest draft
  $0 validate [draft.db]    Read-only database validation
  $0 promote [draft.db]     Validate, then atomically install production map

Environment:
  RTABMAP_IMU_SOURCE=$RTABMAP_IMU_SOURCE   建图时 stereo_odometry 的 IMU 源 (none|d435|px4)
  RTABMAP_VALIDATE_MIN_NODES=$MIN_NODES
  RTABMAP_VALIDATE_MAX_STEP_M=$MAX_NEIGHBOR_STEP_M
EOF
}

state_get() {
    local key="$1"
    [ -f "$STATE_FILE" ] || return 0
    awk -F= -v key="$key" '$1 == key {sub(/^[^=]*=/, ""); print; exit}' "$STATE_FILE"
}

latest_draft() {
    local latest=""
    latest=$(ls -1t "$DRAFT_DIR"/rtabmap_*.db 2>/dev/null | sed -n '1p' || true)
    printf '%s\n' "$latest"
}

container_mapping_running() {
    container_slam_stack_running
}

container_rtabmap_running() {
    docker ps --format '{{.Names}}' 2>/dev/null |
        grep -qx "$CONTAINER_NAME" || return 1
    docker exec "$CONTAINER_NAME" bash --noprofile --norc -c \
        "pgrep -f '^/opt/ros/humble/lib/rtabmap_slam/rtabmap ' >/dev/null" \
        2>/dev/null
}

container_slam_stack_running() {
    docker ps --format '{{.Names}}' 2>/dev/null |
        grep -qx "$CONTAINER_NAME" || return 1
    docker exec "$CONTAINER_NAME" bash --noprofile --norc -c \
        "pgrep -f '^/opt/ros/humble/lib/rtabmap_slam/rtabmap ' >/dev/null || \
         pgrep -f '^/opt/ros/humble/lib/rtabmap_odom/stereo_odometry ' >/dev/null || \
         pgrep -f '^/opt/ros/humble/lib/realsense2_camera/realsense2_camera_node ' >/dev/null || \
         pgrep -f '^/opt/ros/humble/lib/rclcpp_components/component_container ' >/dev/null" \
        2>/dev/null
}

stop_container_slam_stack() {
    local mode="${1:-term}"
    docker ps --format '{{.Names}}' 2>/dev/null | grep -qx "$CONTAINER_NAME" || return 0

    if [ "$mode" = "force" ]; then
        docker exec -u root "$CONTAINER_NAME" \
            bash /workspaces/ros2_ws/scripts/_cleanup_slam.sh >/dev/null 2>&1 || true
        return 0
    fi

    docker exec "$CONTAINER_NAME" bash --noprofile --norc -c "
        pkill -TERM -f 'vslam_(cuvslam|rtabmap).launch.py' 2>/dev/null || true
        pkill -TERM -f '/opt/ros/humble/lib/rtabmap_slam/rtabmap' 2>/dev/null || true
        pkill -TERM -f '/opt/ros/humble/lib/rtabmap_odom/stereo_odometry' 2>/dev/null || true
        pkill -TERM -f '/opt/ros/humble/lib/realsense2_camera/realsense2_camera_node' 2>/dev/null || true
        pkill -TERM -f '/opt/ros/humble/lib/rclcpp_components/component_container' 2>/dev/null || true
    " 2>/dev/null || true
}

topic_publisher_count() {
    local topic="$1"
    ros2 topic info "$topic" --no-daemon --spin-time "$ROS_DISCOVERY_SPIN_TIME" \
        2>/dev/null | awk '/Publisher count:/ {print $3; exit}'
}

wait_for_aligned_depth() {
    local timeout="${1:-60}" elapsed=0
    local depth_topic="/camera/camera/aligned_depth_to_color/image_raw"
    local color_topic="/camera/camera/color/image_raw"
    local depth_pub=0 color_pub=0
    source_ros2_env

    log "等待对齐深度/彩色 (最多 ${timeout}s)..."
    while [ "$elapsed" -lt "$timeout" ]; do
        if docker exec "$CONTAINER_NAME" bash --noprofile --norc -c \
            "grep -qE 'Depth stream start failure|Motion Module failure' /tmp/rtabmap_mapping.log 2>/dev/null"; then
            die "RealSense 流启动失败（常见原因: depth/infra 分辨率不一致）。" \
                "查看: docker exec $CONTAINER_NAME grep -E 'failure|Open profile' /tmp/rtabmap_mapping.log | tail -30"
        fi

        depth_pub=$(topic_publisher_count "$depth_topic")
        color_pub=$(topic_publisher_count "$color_topic")
        depth_pub=${depth_pub:-0}
        color_pub=${color_pub:-0}
        if [ "$depth_pub" -ge 1 ] && [ "$color_pub" -ge 1 ]; then
            if timeout 8 ros2 topic echo "$depth_topic" --once --no-daemon \
                    --spin-time "$ROS_DISCOVERY_SPIN_TIME" \
                    --qos-reliability best_effort --qos-durability volatile \
                    >/dev/null 2>&1; then
                log "对齐深度就绪 ($depth_topic)"
                return 0
            fi
            log "aligned_depth publisher 已出现，等待首帧... (${elapsed}s)"
        else
            log "等待 publisher: color=${color_pub} aligned_depth=${depth_pub} (${elapsed}s)"
        fi
        sleep 2
        elapsed=$((elapsed + 2))
    done
    die "建图前对齐深度未就绪；确认 vslam_rtabmap.launch.py 已设 align_depth.enable:=true。" \
        "容器日志: docker exec $CONTAINER_NAME tail -80 /tmp/rtabmap_mapping.log"
}

validate_db() {
    local db="$1"
    [ -f "$db" ] || die "数据库不存在: $db"
    [ -s "$db" ] || die "数据库为空: $db"

    DB_TO_VALIDATE="$db" \
    MIN_NODES_TO_VALIDATE="$MIN_NODES" \
    MAX_STEP_TO_VALIDATE="$MAX_NEIGHBOR_STEP_M" \
    python3 <<'PY'
import math
import os
import sqlite3
import struct
import sys

path = os.environ["DB_TO_VALIDATE"]
minimum_nodes = int(os.environ["MIN_NODES_TO_VALIDATE"])
maximum_step = float(os.environ["MAX_STEP_TO_VALIDATE"])
# Accept maps where the largest linked component is usable and the rest are
# degree-0 rehearsed orphans (common with older Mem/NotLinkedNodesKept=true).
min_connected_ratio = float(os.environ.get("RTABMAP_VALIDATE_MIN_RATIO", "0.85"))
errors = []
warnings = []

try:
    con = sqlite3.connect("file:{}?mode=ro".format(path), uri=True)
    con.execute("PRAGMA query_only=ON")
    integrity = [row[0] for row in con.execute("PRAGMA integrity_check")]
    if integrity != ["ok"]:
        errors.append("SQLite integrity_check: {}".format("; ".join(integrity)))

    tables = {row[0] for row in con.execute(
        "SELECT name FROM sqlite_master WHERE type='table'")}
    for required in ("Node", "Link"):
        if required not in tables:
            errors.append("缺少 RTAB-Map 表 {}".format(required))
    if errors:
        raise RuntimeError

    node_columns = {row[1] for row in con.execute("PRAGMA table_info(Node)")}
    if not {"id", "map_id"}.issubset(node_columns):
        errors.append("Node 表缺少 id/map_id")
        raise RuntimeError

    rows = list(con.execute("SELECT id, map_id FROM Node WHERE id > 0"))
    node_ids = {int(row[0]) for row in rows}
    map_ids = {int(row[1]) for row in rows}
    if len(node_ids) < minimum_nodes:
        errors.append("节点数 {} 小于最小值 {}".format(
            len(node_ids), minimum_nodes))
    if len(map_ids) != 1:
        errors.append("期望单一 map_id，实际 {}".format(sorted(map_ids)))

    link_columns = {row[1] for row in con.execute("PRAGMA table_info(Link)")}
    if not {"from_id", "to_id"}.issubset(link_columns):
        errors.append("Link 表缺少 from_id/to_id")
        raise RuntimeError

    select = ["from_id", "to_id"]
    select.append("type" if "type" in link_columns else "0 AS type")
    select.append("transform" if "transform" in link_columns else "NULL AS transform")
    links = list(con.execute("SELECT {} FROM Link".format(", ".join(select))))

    adjacency = {node_id: set() for node_id in node_ids}
    neighbor_adjacency = {node_id: set() for node_id in node_ids}
    bad_steps = []
    undecodable_neighbor_steps = 0
    for from_id, to_id, link_type, transform in links:
        from_id, to_id = int(from_id), int(to_id)
        if from_id in adjacency and to_id in adjacency:
            adjacency[from_id].add(to_id)
            adjacency[to_id].add(from_id)
        if int(link_type) not in (0, 6):
            continue
        if from_id in neighbor_adjacency and to_id in neighbor_adjacency:
            neighbor_adjacency[from_id].add(to_id)
            neighbor_adjacency[to_id].add(from_id)
        if not isinstance(transform, bytes) or len(transform) < 48:
            undecodable_neighbor_steps += 1
            continue
        values = struct.unpack("<12f", transform[:48])
        xyz = (values[3], values[7], values[11])
        step = math.sqrt(sum(value * value for value in xyz))
        if not math.isfinite(step) or step > maximum_step:
            bad_steps.append((from_id, to_id, step))

    components = []
    seen = set()
    for start in node_ids:
        if start in seen:
            continue
        pending = [start]
        component = set()
        while pending:
            node_id = pending.pop()
            if node_id in component:
                continue
            component.add(node_id)
            pending.extend(adjacency[node_id] - component)
        seen |= component
        components.append(component)
    components.sort(key=len, reverse=True)

    isolated = sorted(node_id for node_id in node_ids if not adjacency[node_id])
    linked_nodes = node_ids - set(isolated)
    linked_components = [c for c in components if len(c) > 1 or (
        len(c) == 1 and next(iter(c)) in linked_nodes)]
    # Degree-0 orphans are ignored for connectivity; require one linked graph.
    main = max(components, key=len) if components else set()
    if linked_nodes:
        linked_seen = set()
        pending = [next(iter(linked_nodes))]
        while pending:
            node_id = pending.pop()
            if node_id in linked_seen:
                continue
            linked_seen.add(node_id)
            pending.extend((adjacency[node_id] & linked_nodes) - linked_seen)
        main = linked_seen

    if len(main) < minimum_nodes:
        errors.append("最大连通分量仅 {} 节点，小于最小值 {}".format(
            len(main), minimum_nodes))
    if linked_nodes and main != linked_nodes:
        multi = sorted(
            (len(c) for c in components if len(c) > 1), reverse=True)
        errors.append(
            "存在多个非平凡连通分量 {}: 主分量无法覆盖全部有边节点".format(
                multi[:5]))
        errors.append(
            "请重新慢速建图；勿中途停顿/抱起，并确保回到起点闭环")
    elif isolated:
        warnings.append(
            "将在 promote 时裁剪 {} 个无边孤立节点，保留主分量 {}/{}".format(
                len(isolated), len(main), len(node_ids)))
    ratio = (len(main) / float(len(node_ids))) if node_ids else 0.0
    if ratio + 1e-9 < min_connected_ratio and isolated:
        warnings.append(
            "连通比例 {:.0%}（主分量 {}/{}）；新参数建图可避免孤立节点".format(
                ratio, len(main), len(node_ids)))

    if bad_steps:
        sample = ", ".join("{}→{}:{:.2f}m".format(*item)
                           for item in bad_steps[:5])
        # A couple of large neighbor steps usually mean a brief VIO jump; warn
        # unless there are many of them.
        if len(bad_steps) <= 3:
            warnings.append("相邻节点异常步长 > {:.2f}m: {}".format(
                maximum_step, sample))
        else:
            errors.append("相邻节点异常步长过多 ({}): {}".format(
                len(bad_steps), sample))
    if undecodable_neighbor_steps:
        warnings.append("有 {} 条相邻边无法解码位姿步长".format(
            undecodable_neighbor_steps))

except RuntimeError:
    pass
except Exception as exc:
    errors.append("只读验证异常: {}".format(exc))
finally:
    try:
        con.close()
    except NameError:
        pass

for warning in warnings:
    print("[WARN] " + warning)
if errors:
    for error in errors:
        print("[FAIL] " + error, file=sys.stderr)
    sys.exit(1)

print("[OK] SQLite integrity=ok")
print("[OK] map_id={} nodes={} links={} main_component={}".format(
    next(iter(map_ids)), len(node_ids), len(links), len(main)))
print("[OK] graph usable; neighbor step check done (max {:.2f}m)".format(
    maximum_step))
PY
}

start_mapping() {
    [ -f /opt/ros/humble/setup.bash ] || die "宿主机 ROS2 Humble 不存在"
    [ -f "$ROS2_WS/install/setup.bash" ] || die "工作区尚未编译"

    if ! docker ps --format '{{.Names}}' | grep -qx "$CONTAINER_NAME"; then
        log "启动容器 $CONTAINER_NAME"
        docker start "$CONTAINER_NAME" >/dev/null
        sleep 2
    fi
    docker ps --format '{{.Names}}' | grep -qx "$CONTAINER_NAME" ||
        die "容器无法启动: $CONTAINER_NAME"

    if docker exec "$CONTAINER_NAME" bash --noprofile --norc -c \
        "pgrep -f '[r]os2 launch .*/vslam_(cuvslam|rtabmap).launch.py|[r]tabmap_slam/rtabmap' >/dev/null" \
        2>/dev/null; then
        die "检测到已有 VSLAM/RTAB-Map；为避免误停定位或写错库，拒绝并行建图"
    fi

    local stamp draft container_draft
    stamp=$(date +'%Y%m%d_%H%M%S')
    draft="$DRAFT_DIR/rtabmap_${stamp}.db"
    [ ! -e "$draft" ] || die "草稿路径已存在: $draft"
    container_draft="$CONTAINER_WS/maps/drafts/$(basename "$draft")"

    {
        printf 'DB=%s\n' "$draft"
        printf 'STARTED=%s\n' "$(date -Is)"
    } >"$STATE_FILE"

    # 2026-09-14: 建图只用 D435i RGB-D（视觉里程计 + RTAB-Map），不再需要 RPLidar/scan_planar
    log "启动容器视觉 RTAB-Map 建图: $draft"
    docker exec -d "$CONTAINER_NAME" bash -lc "
        source /opt/ros/humble/setup.bash
        if [ -d /opt/realsense_rsusb/lib ]; then
            export LD_LIBRARY_PATH=/opt/realsense_rsusb/lib:\${LD_LIBRARY_PATH:-}
        fi
        exec ros2 launch '$CONTAINER_WS/launch/vslam_rtabmap.launch.py' \
            mode:=mapping \
            database_path:='$container_draft' \
            rtabmap_args:=--delete_db_on_start \
            imu_source:=${RTABMAP_IMU_SOURCE:-none} \
            rviz:=false \
            rtabmap_viz:=false \
            > /tmp/rtabmap_mapping.log 2>&1
    "

    sleep 3
    if ! container_mapping_running; then
        die "容器建图未保持运行: docker exec $CONTAINER_NAME tail -80 /tmp/rtabmap_mapping.log"
    fi

    # RealSense wiki + RabbitRobot both require aligned_depth before mapping.
    wait_for_aligned_depth 60

    log "建图运行中；数据库: $draft"
    log "操作要点(来自 RealSense/RabbitRobot 文档): 保持水平、慢走慢转，勿急停急转"
    log "完成采集后执行: $0 stop"
    log "随后验证: $0 validate '$draft'"
    log "注意: 本次建图已启用 align_depth；旧未对齐 depth 的 production 库不要混用"
}

stop_mapping() {
    local draft waited force="${1:-}"
    draft=$(state_get DB)

    if [ "$force" = "force" ]; then
        stop_container_slam_stack force
    else
        stop_container_slam_stack term
    fi

    waited=0
    while container_slam_stack_running && [ "$waited" -lt 20 ]; do
        sleep 1
        waited=$((waited + 1))
    done
    if container_slam_stack_running; then
        log "TERM 后仍有容器 SLAM 进程，执行强制清理..."
        stop_container_slam_stack force
        waited=0
        while container_slam_stack_running && [ "$waited" -lt 10 ]; do
            sleep 1
            waited=$((waited + 1))
        done
    fi
    if container_slam_stack_running; then
        die "容器建图未能安全停止；保留状态文件，禁止验证/提升。可试: $0 force-stop"
    fi
    rm -f "$STATE_FILE"

    log "建图流程已停止（未停止 PX4，因为本流程从未启动 PX4）"
    if [ -n "$draft" ] && [ -f "$draft" ]; then
        log "草稿: $draft ($(du -h "$draft" | awk '{print $1}'))"
        log "下一步: $0 validate '$draft'"
    else
        log "尚未生成数据库；请检查容器日志 /tmp/rtabmap_mapping.log"
    fi
}

show_status() {
    local draft
    draft=$(state_get DB)

    if container_mapping_running; then
        log "容器建图: RUNNING"
    else
        log "容器建图: STOPPED"
    fi
    [ -z "$draft" ] || log "当前草稿: $draft"
    draft=$(latest_draft)
    [ -z "$draft" ] || log "最新草稿: $draft ($(du -h "$draft" | awk '{print $1}'))"
    [ ! -f "$PRODUCTION_DB" ] ||
        log "生产地图: $PRODUCTION_DB ($(du -h "$PRODUCTION_DB" | awk '{print $1}'))"
}

promote_db() {
    local db="$1" archive_dir tmp backup_stamp
    if container_slam_stack_running; then
        die "RTAB-Map 仍在运行；必须先 stop，确保 SQLite 完整落盘"
    fi
    case "$(readlink -f "$db")" in
        "$(readlink -f "$DRAFT_DIR")"/*) ;;
        *) die "只能提升 maps/drafts 下的数据库: $db" ;;
    esac

    validate_db "$db"
    archive_dir="$PRODUCTION_DIR/archive"
    mkdir -p "$archive_dir"
    if [ -f "$PRODUCTION_DB" ]; then
        backup_stamp=$(date +'%Y%m%d_%H%M%S')
        cp -- "$PRODUCTION_DB" "$archive_dir/rtabmap_${backup_stamp}.db"
        chmod 0444 "$archive_dir/rtabmap_${backup_stamp}.db"
    fi

    tmp="$PRODUCTION_DIR/.rtabmap.db.promote.$$"
    trap 'rm -f "$tmp"' EXIT
    cp -- "$db" "$tmp"
    chmod u+w "$tmp"

    DB_TO_PRUNE="$tmp" python3 <<'PY'
import os
import sqlite3
import sys

path = os.environ["DB_TO_PRUNE"]
con = sqlite3.connect(path)
try:
    node_ids = {int(r[0]) for r in con.execute("SELECT id FROM Node WHERE id > 0")}
    adjacency = {n: set() for n in node_ids}
    for from_id, to_id in con.execute("SELECT from_id, to_id FROM Link"):
        from_id, to_id = int(from_id), int(to_id)
        if from_id in adjacency and to_id in adjacency:
            adjacency[from_id].add(to_id)
            adjacency[to_id].add(from_id)
    isolated = sorted(n for n in node_ids if not adjacency[n])
    if not isolated:
        print("[OK] no isolated nodes to prune")
        sys.exit(0)

    tables = {r[0] for r in con.execute(
        "SELECT name FROM sqlite_master WHERE type='table'")}
    placeholders = ",".join("?" for _ in isolated)
    # Keep Node/Data/Link consistent; ignore optional tables that may not exist.
    for table, column in (
        ("Link", "from_id"),
        ("Link", "to_id"),
        ("Data", "id"),
        ("Node", "id"),
    ):
        if table not in tables:
            continue
        con.execute(
            "DELETE FROM {} WHERE {} IN ({})".format(
                table, column, placeholders),
            isolated)
    con.commit()
    remaining = con.execute(
        "SELECT COUNT(*) FROM Node WHERE id > 0").fetchone()[0]
    print("[OK] pruned {} isolated nodes; remaining={}".format(
        len(isolated), remaining))
finally:
    con.close()
PY

    chmod 0444 "$tmp"
    mv -f -- "$tmp" "$PRODUCTION_DB"
    trap - EXIT
    log "已提升生产地图: $PRODUCTION_DB"
    log "保留草稿与旧版 $MAP_ROOT/rtabmap.db，未删除任何草稿/旧库"
}

command="${1:-start}"
case "$command" in
    start) start_mapping ;;
    stop) stop_mapping ;;
    force-stop) stop_mapping force ;;
    status) show_status ;;
    validate)
        db="${2:-$(latest_draft)}"
        [ -n "$db" ] || die "没有可验证的草稿数据库"
        validate_db "$db"
        ;;
    promote)
        db="${2:-$(latest_draft)}"
        [ -n "$db" ] || die "没有可提升的草稿数据库"
        promote_db "$db"
        ;;
    -h|--help|help) usage ;;
    *) usage; die "未知命令: $command" ;;
esac
