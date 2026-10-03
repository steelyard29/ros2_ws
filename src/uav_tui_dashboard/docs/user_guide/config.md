# Configuration Guide

UAV TUI Dashboard 支持通过 TOML 配置文件进行设置，无需在每次启动时输入大量命令行参数。

## 配置文件位置

默认配置文件位于 `config/default.toml`。您可以：

- 编辑此文件来设置默认值
- 使用 `--config` 参数指定自定义配置文件路径

## 配置优先级

配置值的优先级从高到低：

1. 命令行参数（覆盖配置文件）
2. 配置文件中的值
3. 代码中的硬编码默认值

## 快速开始

### 编辑默认配置

```bash
# 编辑配置文件
vim config/default.toml
```

### 使用自定义配置

```bash
# 指定自定义配置文件
uav-dashboard --config path/to/my/config.toml
```

### 命令行覆盖

```bash
# 配置文件中 mode="ros2"，但临时使用 sim 模式
uav-dashboard --mode sim
```

## 配置选项

所有配置选项及其详细说明已在 `config/default.toml` 文件中以注释形式提供。主要配置区域包括：

- **[general]**: 基本运行参数（模式、轮询间隔等）
- **[logging]**: 日志配置（级别、目录、控制台输出等）
- **[ros2]**: ROS2 相关设置（主题、消息类型、命名空间等）

## 注意事项

- 空字符串 (`""`) 表示使用系统默认值或禁用功能
- 配置文件不存在时将使用代码默认值
- TOML 格式错误会导致配置加载失败并回退到默认值
- 布尔值使用 `true`/`false`
- 数组使用 `[]` 语法（如 ROS 参数列表）

## 示例配置

查看 `config/default.toml` 获取完整示例和所有可用选项的详细说明。
