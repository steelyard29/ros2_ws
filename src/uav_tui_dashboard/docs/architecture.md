# 系统架构概览

本项目采用分层结构以保持终端 UI、数据源以及支撑工具之间的清晰边界。

## 顶层布局

```
├── src/
│   └── uav_tui_dashboard/
│       ├── cli.py              # 命令行入口
│       ├── logging_config.py   # 日志配置
│       ├── shutdown.py         # 优雅关闭工具
│       ├── core/               # 数据模型与数据源
│       └── ui/                 # Textual UI 实现
├── docs/                       # 标准化文档（当前目录）
├── tests/                      # Pytest 测试套件
├── trash/                      # 历史文档与测试归档
├── pyproject.toml              # 唯一依赖与元数据配置
└── README.md                   # 面向外部用户的快速指南
```

## 核心模块

- **core**
  - `models.py` 提供不可变数据模型与快照队列。
  - `datasource.py`（后续将拆分）定义数据源接口、模拟数据以及 ROS2 集成。
- **ui**
  - `app.py` 基于 Textual 渲染实时状态表格与快照面板。
  - `dashboard.css` 管理终端 UI 样式。
- **cli**
  - 解析命令行参数，安装日志配置并启动应用。

## 依赖策略

- 运行依赖与可选组件统一在 `pyproject.toml` 管理。
- 历史的 `requirements.txt` 已移至 `trash/legacy_requirements.txt`，不得再引用。

## 未来重构计划

Phase 0 结束后将进一步拆分 `core/datasource.py`，并配套新增测试，详见 `ROADMAP.md`。
