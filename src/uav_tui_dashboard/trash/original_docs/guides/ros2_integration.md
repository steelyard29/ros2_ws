# ROS2 数据源与数据流说明

本文档详细介绍 `uav-tui-dashboard` 在 ROS2 模式下的数据流、主题配置以及常见使用场景，帮助你快速接入真实无人机遥测数据。

## 总览

```
┌────────────────────────────┐
│  ROS2 图 (rclpy Context)   │
│                            │
│  ┌──────────────────────┐  │    ┌──────────────────────────┐
│  │ nav_msgs/msg/Odometry│──┼──▶│ Ros2DataSource._handle_… │
│  └──────────────────────┘  │    └──────────────────────────┘
│                            │                 │
│  ┌──────────────────────┐  │                 ▼ 线程安全缓存
│  │ sensor_msgs/msg/     │──┼──▶│ Ros2DataSource._handle_… │
│  │ BatteryState         │  │    └──────────────────────────┘
└────────────────────────────┘                 │
                                              ▼
                                      UAVStatus（位置、姿态、电池）
                                              │
                                              ▼
                                   Textual UI (`UAVDashboardApp`)
```

1. `Ros2DataSource` 使用 `rclpy` 创建节点，订阅里程计与电池主题。
2. 接收到的消息在独立线程中被解析，写入内部的 `UAVStatus` 缓存，并通过互斥锁保证线程安全。
3. Textual UI 每隔 `poll_interval` 秒调用 `fetch()`，读取最新缓存并刷新界面。

## 节点与执行器

- **节点名称**：默认 `uav_dashboard_listener`，可通过 CLI `--mode ros2 --ros-namespace <ns>` 配置命名空间。
- **执行器**：使用单线程执行器 (`SingleThreadedExecutor`)，在后台线程循环调用 `spin_once()`；
  这样 UI 主线程不会阻塞，但仍能及时消费 ROS 消息。
- **清理流程**：退出时销毁订阅、节点并关闭 `rclpy`，避免残留线程。

## 默认主题与消息

| 功能           | 默认主题           | 默认消息类型                |
|----------------|--------------------|-----------------------------|
| 位置 / 姿态    | `/uav/odometry`    | `nav_msgs/msg/Odometry`     |
| 电池状态       | `/uav/battery`     | `sensor_msgs/msg/BatteryState` |

- **可配置性**：所有主题与消息类型可通过 CLI 参数覆盖：
  - `--ros-odometry-topic` / `--ros-odometry-type`
  - `--ros-battery-topic` / `--ros-battery-type`
  - 将 `--ros-battery-topic ""` 设为空字符串即可关闭电池订阅。
- **消息解析策略**：
  - 里程计消息支持标准 `pose.pose` 结构，也兼容 PX4 `VehicleOdometry` 中的 `position`/`q` 列表。
  - 四元数通过 `atan2`/`asin` 计算欧拉角（单位：弧度）。
  - 电量优先读取 `percentage`（自动判断是 0-1 还是 0-100），其次回退到 `voltage` 或 `cell_voltage` 平均值。

## 运行前准备

1. 在终端中加载 ROS2 环境，例如 Humble：
   ```bash
   source /opt/ros/humble/setup.bash
   ```
2. 安装 Python 依赖：
   ```bash
    pip install -e .
   ```
    `ros2` 可选依赖现包含 `rclpy`、`PyYAML`、`jinja2`、`typeguard`、`lark`、`numpy` 等 ROS 插件/消息所需包。
  或使用发行版提供的 `rclpy`（Ubuntu: `sudo apt install ros-humble-rclpy`）。

3. 启动提供遥测数据的 ROS2 节点（例如 PX4 的 `px4_ros_com`、自建仿真等）。

## 启动与参数示例

  ```bash
  uav-dashboard --mode ros2 \
    --ros-namespace /uav1 \
    --ros-odometry-topic /uav1/nav/odom \
    --ros-battery-topic /uav1/power/battery \
    --poll-interval 0.5
  ```

- `--ros-arg` 可多次使用，用于传递额外的 rclpy 初始化参数（如 `--ros-arg --enclave /secure`）。
- 若需要更换消息类型（自定义消息），请提供完整的 Python 模块路径，例如：
  ```bash
  uav-dashboard --mode ros2 \
    --ros-odometry-type custom_msgs.msg.VehicleState
  ```
  对应模块需能被 `PYTHONPATH` 找到。

  ## 快照记录与导出

  除了实时表格，Textual UI 还提供右侧的 **Flight Snapshots** 队列，用于在调试或试飞时抓取关键时刻的数据快照。

  ### 捕获流程

  1. 当界面显示出想要记录的状态时，点击 **Capture** 按钮。
  2. 当前 `UAVStatus`（位置、姿态、电池、VehicleState 等）会被复制到队列末尾。
  3. 队列默认最多保留 20 条，若超过会自动丢弃最早的条目。可以在代码中调整 `FlightSnapshotQueue(maxlen=...)`。

  ### 导出与后处理

  - 点击 **Export** 将队列写出为 JSON 数组，默认输出到当前工作目录，文件名类似 `uav_snapshots_20251006-081530.json`。
  - JSON 数据包含 `captured_at` UTC 时间戳，以及 `status` 字段下的 `position`、`orientation`、`battery_level`、`voltage`、`vehicle_state` 等信息，方便导入 Pandas、Jupyter 或报告模板。
  - **Clear** 可快速重置队列，便于分阶段记录。

  #### 应用场景示例

  - **手动试飞记录**：手动捕捉起飞、巡航、返航三个阶段，导出到 JSON 用于飞行日志。
  - **异常排查**：当 UI 显示 failsafe 或电量异常时立即 Capture，多次采样后导出用于分析。
  - **与 rosbag 互补**：无需开启完整 rosbag，仍可收集关键帧数据；如需全量回放，可搭配 rosbag 使用。

## PX4 Interface 集成规划

> 目标：将 PX4 Interface 提供的缓存化遥测数据作为 TUI 的主要数据源，实现“PX4 Gateway → 缓存话题 → TUI 展示”的闭环，同时保持对模拟及其他 ROS2 数据源的兼容。

### 1. 依赖与环境准备

1. **ROS2 工作区**：在同一 `colcon` 工作区中构建 `px4_interface`（确保 `px4_msgs` 已可用），并确认节点能成功发布 `/cache/*` 话题。
2. **Python 依赖**：在 TUI 虚拟环境中安装 `px4_interface` 生成的 Python 消息（`pip install /path/to/install/lib/python*/site-packages` 或通过 `colcon build --symlink-install` 后 source `install/setup.bash`）。
3. **时间同步**：PX4 Interface 发布的消息携带时间戳校验逻辑，TUI 应使用 ROS 时钟；在仿真中建议启用 `/use_sim_time` 并保持时钟一致。

### 2. 主题与数据映射

| PX4 Interface 话题 | 自定义消息类型 | TUI 目标数据 | 说明 |
|--------------------|----------------|--------------|------|
| `/cache/vehicle_odometry` | `px4_interface/msg/PoseNED` | `UAVStatus.pose` & `UAVStatus.velocity` | 位置为 NED 坐标，姿态为四元数；需要转换为 ENU 或直接以 NED 展示（规划阶段需确定 UI 表达方式）。 |
| `/cache/battery_status` | `px4_interface/msg/BatteryStatus` | `UAVStatus.battery` | `voltage_v`、`current_a`、`remaining`（0-1）以及 `warning` 枚举；可扩展 UI 显示。 |
| `/cache/vehicle_status` | `px4_interface/msg/VehicleStatus` | `UAVStatus.health`（新字段） | 用于显示解锁状态、导航模式、failsafe 等；规划中需要定义 UI 呈现方案。 |

### 3. 代码修改计划

1. **扩展消息解析层** (`core/datasource.py`)
    - 添加 `parse_pose_ned`、`parse_battery_status`、`parse_vehicle_status` 函数，将自定义消息转换为现有 `UAVStatus` 结构。
    - 引入坐标与姿态转换工具（若 UI 继续使用 ENU，需要对 NED → ENU 做轴变换；否则需更新 UI 标签提示 NED）。
    - 允许 `ROS2DataSource` 同时管理多个解析器，支持在 CLI 中选择“No-Op/默认/px4”策略。
2. **调整默认配置** (`cli.py`)
    - 新增预设 `--ros-profile px4_interface`，一次性加载 `/cache/*` 话题与对应消息类型。
    - 保留现有自定义参数，允许在命令行覆盖 PX4 专用配置。
3. **UI 层适配** (`ui/app.py`)
    - 增加状态面板用于展示 `VehicleStatus`（arming/nav/failsafe）。
    - 根据坐标系决定文案与单位（NED ↔ ENU）。
4. **日志与监控** (`logging_config.py`)
    - 为 PX4 集成增加独立 logger，便于定位消息延迟或校验失败。

### 4. 测试与验证

| 阶段 | 目的 | 方法 |
|------|------|------|
| 单元测试 | 验证解析函数的 NED→内部模型转换正确 | 为 `parse_pose_ned` 等函数添加 pytest 用例，使用 px4_interface 消息样例。 |
| 集成测试 | 确认数据流通畅 | 启动 PX4 Interface 的 `px4_gateway`，模拟 PX4 消息输入（可使用录制的 `ros2 bag` 或测试脚本）。 |
| UI 验收 | 核对界面字段更新 | 运行 TUI，观察姿态、速度、电池及状态面板更新，检查边界值（无效消息、failsafe 告警等）。 |

### 5. 发布与部署策略

1. **运行时集成**：在终端中依次启动 PX4 Interface 与 TUI（`source install/setup.bash` 后执行 `uav-dashboard --mode ros2 --ros-profile px4_interface`）。
2. **可选 ROS2 包装**：若需与其他 ROS2 节点一起通过 launch 文件启动，可为 TUI 创建轻量级 `ament_python` 包装，在 `package.xml` 中声明对 `px4_interface` 的依赖，并提供 `launch` 脚本。
3. **监控与日志**：结合 `--log-level DEBUG` 与日志目录，记录 PX4 Gateway 缓存延迟、NED→ENU 转换结果以及 UI 渲染速率，为后续优化提供证据。

### 7. 本地模拟数据源

为方便离线演示，仓库提供 `scripts/mock_px4_cache_publisher.py`，可在未连接真实飞控时模拟发布 `/cache/vehicle_odometry`、`/cache/battery_status` 及 `/cache/vehicle_status` 数据。

```bash
# 终端 1：启动模拟发布器
colcon launch px4_interface mock_px4.launch.py
# 终端 2：启动 TUI（PX4 预设）
uav-dashboard --mode ros2 --ros-profile px4_interface --poll-interval 0.5 --log-level DEBUG
```

> ✅ **提示**：模拟发布器会周期性检查 `/cache/*` 话题是否已有其他发布者（例如真实 PX4 Gateway 或未关闭的旧实例）。
> 若检测到冲突，它会暂停输出并在终端打印错误日志，避免仪表盘在多个数据源之间跳变。
> 请先停止其他发布者，再重新启动脚本即可恢复模拟数据。

使用 `ros2 topic echo /cache/vehicle_odometry` 可验证话题是否持续更新。若需要自动化测试，可在 CI 中单独启动该脚本并运行 TUI 的集成测试。

### 6. 后续扩展路线

- 支持多无人机：以命名空间区分多个 PX4 Gateway 实例，动态生成面板。
- 增强 QoS 配置：允许在 CLI 中调整可靠性/历史策略，适应无线链路抖动。
- 安全模式指示：结合 `failsafe` 与 `warning` 字段触发 UI 高亮或弹窗提示。
- 与任务控制器联动：通过额外话题显示任务阶段、航点等高级信息。

## 常见问题

### 1. 启动时报错 “ROS2 模式需要安装 rclpy …”
说明当前 Python 环境无法导入 `rclpy`。请确认已安装 ROS 发行版并在同一终端激活环境，或通过 `pip install rclpy` 安装二进制包（需匹配系统和 ROS 版本）。

### 2. UI 没有更新数据
- 检查 ROS2 中是否有消息发布：
  ```bash
  ros2 topic echo /uav/odometry --once
  ```
- 若主题名称或消息类型不匹配，请使用 CLI 参数调整。
- 可以加上 `--log-level DEBUG` 查看数据源日志。

### 3. 想扩展成多话题或其他传感器？
你可以：
- 继承 `Ros2DataSource` 并覆盖 `_handle_*` 方法，或
- 使用构造函数传入自定义 `odometry_parser` / `battery_parser` 回调，实现特定解析逻辑。

## 相关源码位置

- `src/uav_tui_dashboard/core/datasource.py`：核心数据源实现与解析函数。
- `src/uav_tui_dashboard/cli.py`：命令行参数解析及数据源实例化。
- `tests/test_datasource.py`：关键解析函数的单元测试示例。

如需进一步定制（多无人机、复杂 QoS 配置等），欢迎基于以上内容扩展。