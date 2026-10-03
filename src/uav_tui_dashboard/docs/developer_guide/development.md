# 开发者指南

本指南描述如何在干净环境中启动开发工作。

## 1. 环境准备

1. 安装 Python 3.8–3.12。
2. 克隆仓库后创建虚拟环境：
   ```bash
   python -m venv .venv
   source .venv/bin/activate
   ```
3. 安装依赖：
   ```bash
   pip install -e .
   ```
   后续阶段会新增 `.[dev]`、`.[ros2]` 等可选依赖，届时以 `pyproject.toml` 为准。

## 2. 常用命令

| 任务 | 命令 |
| --- | --- |
| 运行仪表盘（模拟模式） | `uav-dashboard` |
| 查看 CLI 帮助 | `uav-dashboard --help` |
| 运行测试（占位） | `pytest` |

> 备注：当前测试目录为新建占位符，后续阶段将补充实际测试用例。

## 3. 代码风格与提交

- 使用 `ruff`、`mypy`、`pytest` 作为未来统一的本地检查工具。
- 提交信息遵循 Conventional Commits，例如 `feat: add ros2 parser`、`docs: update architecture overview`。
- 在提交或发起 PR 前，请完整阅读并遵守《[Git 工作流与提交规范](git_standards.md)》。
- 文档提交需符合《Documentation Guidelines》。

## 4. 目录约定

- 所有新文档写入 `docs/` 并登记在 `docs/README.md`。
- 历史资料保留在 `trash/`，禁止覆盖。
- 临时脚本或实验内容请使用 `sandbox/`（如有需要自行创建）。
