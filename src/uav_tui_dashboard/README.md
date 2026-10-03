# UAV TUI Dashboard

基于 Textual 的无人机状态监控终端用户界面 (TUI)，支持实时数据显示、快照队列和 TOML 配置文件管理。
## 版本
 - V0.1.0
## 功能

- 实时显示无人机位置、姿态、电池、电源以及飞行器健康状态（连接有效性、导航模式、Failsafe、预飞检查）
- 轻量级飞行状态快照队列：一键暂存当前状态、查看历史，并导出 JSON 文件
- 模拟数据源用于演示
- 可配置轮询间隔和日志级别
- 优雅关闭和日志记录
- TOML 配置文件支持，简化启动和配置管理
- Dracula 主题，提供现代化的视觉体验

## 安装

### 系统要求

- Python 3.8+
- Linux/macOS/Windows

### 安装步骤

1. 克隆仓库：
   ```bash
   git clone https://github.com/xmu795/uav-tui-dashboard.git
   cd uav-tui-dashboard
   ```

2. 创建虚拟环境：
   ```bash
   python3 -m venv venv
   source venv/bin/activate  # Linux/macOS
   # 或 venv\Scripts\activate  # Windows
   ```

3. 安装依赖：
   ```bash
   pip install -e .
   ```

4. （可选）启用 ROS2 支持：
   - 确保已安装 ROS 2 Humble 及以上版本，并在相同终端中 source 对应的环境（如 `source /opt/ros/humble/setup.bash`）。
   - 通过 pip 安装额外依赖：
     ```bash
     pip install -e ".[ros2]"
     ```
     或者直接使用系统包管理器安装 `rclpy`（例如 `sudo apt install ros-humble-rclpy`）。

## 运行

### 推荐：使用配置文件启动

项目推荐使用 TOML 配置文件进行启动，这可以避免每次运行时输入大量命令行参数。默认配置文件位于 `config/default.toml`，包含所有常用设置。

```bash
# 使用默认配置文件启动（推荐）
uav-dashboard

# 或指定自定义配置文件
uav-dashboard --config path/to/my/config.toml
```

配置文件示例（`config/default.toml`）：

```toml
[general]
mode = "ros2"  # 或 "sim"
poll_interval = 0.5

[logging]
level = "DEBUG"
dir = ""
config = ""
no_console = false

[ros2]
namespace = ""
odometry_topic = "/uav/odometry"
battery_topic = "/uav/battery"
odometry_type = "nav_msgs.msg.Odometry"
battery_type = "sensor_msgs.msg.BatteryState"
vehicle_status_topic = ""
vehicle_status_type = "px4_interface.msg.VehicleStatus"
args = []
profile = "px4_interface"
```

### 模拟模式

如果需要临时使用模拟数据，可以通过配置文件或命令行覆盖：

```bash
# 通过命令行覆盖
uav-dashboard --mode sim
```

### ROS2 模式

默认配置已设置为 ROS2 模式。如需自定义主题或设置，请编辑 `config/default.toml` 或使用命令行参数覆盖。

默认情况下，仪表盘会订阅下列主题：

- `/uav/odometry`，消息类型 `nav_msgs/msg/Odometry`
- `/uav/battery`，消息类型 `sensor_msgs/msg/BatteryState`

可通过配置文件自定义：

```toml
[ros2]
odometry_topic = "/custom/odometry"
battery_topic = "/custom/battery"
# 其他设置...
```

或使用命令行参数进行临时覆盖：

- `--ros-namespace`：节点命名空间
- `--ros-odometry-topic` 与 `--ros-odometry-type`：里程计主题与消息类型
- `--ros-battery-topic` 与 `--ros-battery-type`：电池主题与消息类型（留空主题可禁用订阅）
- `--ros-vehicle-status-topic` 与 `--ros-vehicle-status-type`：飞行器状态主题与消息类型（留空主题可禁用订阅）
- `--ros-arg`：传递给 `rclpy.init()` 的额外参数，可多次使用

#### 快照面板

应用右侧新增“Flight Snapshots”面板，适用于快速记录多个时间点的遥测状态，而无需整段录制。

- **Capture**：将当前 `UAVStatus` 冻结到队列中（默认上限 20 条，自动丢弃最旧项）。
- **Export**：把队列写出为 JSON 文件，默认保存到当前工作目录，命名格式 `uav_snapshots_YYYYMMDD-HHMMSS.json`。
- **Clear**：清空队列。

导出的 JSON 中包含捕获时间（UTC ISO8601）、位置/姿态/电池/状态等字段，方便交给后续工具或报告。

#### PX4 Interface 预设

- 如果工作区内运行了 [`px4_interface`](../px4_interface) 并发布 `/cache/*` 话题，可直接启用预设：

   ```bash
   uav-dashboard --mode ros2 --ros-profile px4_interface
   ```

   该命令会自动选择 `/cache/vehicle_odometry`、`/cache/battery_status`、`/cache/vehicle_status` 等主题，并使用 PX4 专用解析逻辑（含 NED→ENU 转换与飞行器状态解码）。命令行参数仍可覆盖预设值。

- 仅用于本地演示时，可启动项目内的模拟发布器（需要已 source ROS2 环境）：

   ```bash
   # 终端 1：发布模拟 PX4 缓存话题
   python /home/cfly/ros2_ws/scripts/mock_px4_cache_publisher.py

   # 终端 2：运行 TUI
   uav-dashboard --mode ros2 --ros-profile px4_interface --poll-interval 0.5
   ```

   可通过 `--log-level DEBUG` 查看解析日志，确认姿态、电量等数据在刷新。

> 📘 **深入了解数据流与扩展方式**：请阅读 [`docs/ros2_integration.md`](docs/ros2_integration.md)，了解订阅结构、消息解析策略、缓存机制及常见问题解答。

### 配置文件

支持通过 TOML 配置文件进行设置，无需在每次启动时输入大量命令行参数。默认配置文件位于 `config/default.toml`。

- 编辑默认配置：
  ```bash
  vim config/default.toml
  ```

- 使用自定义配置：
  ```bash
  uav-dashboard --config path/to/my/config.toml
  ```

配置优先级：命令行参数 > 配置文件 > 代码默认值。

详细配置选项请参考 `config/default.toml` 文件中的注释，或查看 [`docs/user_guide/config.md`](docs/user_guide/config.md)。

### 命令行参数（高级）

虽然推荐使用配置文件，但仍支持命令行参数进行临时覆盖或高级配置：

- `--mode`: 数据源模式（sim 或 ros2，默认 ros2）
- `--poll-interval`: 数据轮询间隔（秒，默认 0.5）
- `--log-level`: 日志级别（默认 DEBUG）
- `--log-dir`: 日志目录
- `--log-config`: 日志配置文件路径
- `--no-log-console`: 禁用控制台日志输出
- `--config`: 自定义配置文件路径（默认为 `config/default.toml`）

### 环境变量

- `UAV_TUI_LOG_DIR`: 日志目录
- `UAV_TUI_LOG_CONFIG`: 日志配置文件路径

## 开发

### 运行测试

```bash
pip install -e ".[dev]"
pytest
```

### 项目结构

```
src/uav_tui_dashboard/
├── cli.py              # 命令行接口
├── core/               # 数据模型与数据源
│   ├── base.py         # 基础数据源接口
│   ├── models.py       # 数据模型
│   ├── parsers.py      # 消息解析器
│   ├── ros.py          # ROS2 数据源
│   ├── sim.py          # 模拟数据源
│   └── datasource.py   # 兼容性 shim（已弃用）
├── ui/                 # Textual UI 实现
│   ├── app.py          # 主应用
│   └── dashboard.css   # 样式（Dracula 主题）
├── logging_config.py   # 日志配置
├── shutdown.py         # 优雅关闭工具
└── __main__.py         # 模块入口
config/
└── default.toml        # 默认配置文件
docs/                   # 文档
├── architecture.md     # 系统架构
├── changelog.md        # 更新日志
├── user_guide/         # 用户指南
└── developer_guide/    # 开发者指南
tests/                  # 测试套件
├── core/               # 核心模块测试
└── test_placeholder.py # 占位测试
trash/                  # 历史文件
├── legacy_requirements.txt
└── original_docs/
```

## 许可证

MIT
