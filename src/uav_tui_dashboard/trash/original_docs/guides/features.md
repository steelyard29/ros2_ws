# UAV TUI Dashboard 功能介绍

## 概述

UAV TUI Dashboard用于实时监控无人机状态信息。

## 核心功能

### 数据显示
- **位置信息**：显示无人机在三维空间中的坐标 (X, Y, Z)
- **姿态信息**：显示无人机的欧拉角 (Roll, Pitch, Yaw)
- **电池状态**：显示电池电量百分比和电压值
- **飞行器状态**：显示遥测有效性、时间戳、解锁状态、导航模式、Failsafe、预飞检查结果

### 数据源
- **模拟模式**：使用随机生成的数据进行演示
- **ROS2 模式**：预留接口，用于接入真实的 ROS2 节点（未来实现）

### 用户界面
- 基于 Textual 的现代化终端界面
- 实时数据表格显示
- 自动刷新机制

## 命令行参数

### 数据源配置
- `--mode {sim,ros2}`：选择数据源模式
  - `sim`：模拟模式（默认）
  - `ros2`：ROS2 模式（当前未实现）

### 性能配置
- `--poll-interval FLOAT`：设置数据轮询间隔（秒），默认 1.0

### 日志配置
- `--log-level LEVEL`：设置日志级别（DEBUG, INFO, WARNING, ERROR），默认 INFO
- `--log-dir PATH`：指定日志文件输出目录
- `--log-config PATH`：指定自定义日志配置文件（TOML/JSON 格式）
- `--no-log-console`：禁用控制台日志输出，仅写入文件

## 环境变量

- `UAV_TUI_LOG_DIR`：日志目录（可通过 --log-dir 覆盖）
- `UAV_TUI_LOG_CONFIG`：日志配置文件路径（可通过 --log-config 覆盖）

## 使用示例

### 基本运行
```bash
uav-dashboard
```

### 自定义轮询间隔
```bash
uav-dashboard --poll-interval 0.5
```

### 调试模式
```bash
uav-dashboard --log-level DEBUG
```

### 指定日志目录
```bash
uav-dashboard --log-dir ./logs
```

### 仅文件日志
```bash
uav-dashboard --no-log-console
```

## 退出方式

- 按 `Ctrl+Q` 进行关闭
- 发送 SIGTERM 或 SIGHUP 信号
- 应用会记录退出原因并确保日志完整写入