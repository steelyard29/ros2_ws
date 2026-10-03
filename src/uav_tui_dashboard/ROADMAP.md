# UAV TUI Dashboard 路线图

> 聚焦第一阶段仓库清理，确保开发环境可靠后再推进功能与集成。

## 阶段概览

| 阶段 | 目标 | 预计周期 | 状态 |
| --- | --- | --- | --- |
| Phase 0 | 仓库清理与基线修复 | 1 周内 | 🔄 进行中 |
| Phase 1 | 质量基线（测试、类型、CI） | Phase 0 完成后 1 周 | ⏳ 待启动 |
| Phase 2 | 功能巩固与文档运营 | Phase 1 完成后 1–2 周 | ⏳ 待启动 |
| Phase 3 | 进阶能力（ROS2、UI、生态） | 质量基线稳定后 | 📌 规划中 |

---

## Phase 0：仓库清理冲刺（当前优先级）

### 0.1 依赖与可选组件整理
- 以 `pyproject.toml` 为单一依赖来源，移除重复的 `requirements.txt` 或改为自动生成文件。
- 定义 `[project.optional-dependencies]`：`dev`（pytest、ruff、mypy 等）、`ros2`（仅保留 `rclpy` 等 ROS 相关项）、`docs`（如 mkdocs、sphinx）。
- 在 README 中更新安装指引，明确 `pip install -e .[dev]`、`pip install -e .[ros2]`、ROS 环境注意事项。

### 0.2 结构梳理与模块拆分
- 将 `core/datasource.py` 拆解为 `base.py`、`sim.py`、`ros.py`、`parsers.py` 等单元，降低文件复杂度。
- 检查并修正导入路径、`__all__` 导出以及 `__init__.py` 聚合。
- 为解析模块补充最小化单元测试覆盖（NED→ENU、百分比/电压边界、VehicleState 继承逻辑）。

### 0.3 文档同步与仓库卫生
- 对齐 `README.md`、`docs/guides/features.md`、`docs/guides/ros2_integration.md` 的状态描述，移除“ROS2 未实现”等过期段落。
- 更新 `docs/` 内引用，删除不存在的 `tests/test_datasource.py` 链接并补充正确路径。
- 检查 `.gitignore`、日志示例、截图/文档资源，确保仓库仅包含必要文件。

### 0.4 验收标准
- 新开发者可在干净环境中执行 `pip install -e .[dev]` 并成功运行 `pytest`（临时允许部分测试缺失，但需通过）。
- `uav-dashboard` 与 `python -m uav_tui_dashboard` 两种入口均正常运行模拟模式。
- README 与文档中的安装、运行、ROS 配置步骤与当前代码一致。

> 备注：Phase 0 完成后产出一次里程碑版本（tag），作为后续工作的稳定基线。

---

## Phase 1：质量基线
- 新增统一的 `pytest` 配置，移除测试中的 `sys.path` hack。
- 引入 `ruff`（lint + format）与 `mypy`，在 `pyproject.toml` 中集中配置并补齐必要类型注解。
- 建立 GitHub Actions：`lint`、`typecheck`、`test` 三条流水线；所有合并请求需通过。
- 提供本地开发脚本（Makefile / nox / uv），标准化 `fmt`、`lint`、`test` 指令。

## Phase 2：功能巩固与文档运营
- 为 `Ros2DataSource` 构建模拟消息或 fake `rclpy` 的集成测试，覆盖启动/停止流程。
- 借助 Textual pilot/pytest 方案补充 UI 层快照、导出、告警路径的端到端测试。
- 维护 `docs/`：新增开发者指南、贡献流程、FAQ；引入版本化或自动发布机制（如 mkdocs 或 sphinx）。
- 整合日志配置与调试案例，提供下载示例或脚本。

## Phase 3：进阶能力规划
- **ROS2 集成增强**：支持命名空间多无人机、动态 QoS、PX4 profile 自动检测、ROS 环境自检与回退。
- **UI 能力扩展**：图表/趋势面板、历史记录浏览、Failsafe 高亮策略、自定义布局。
- **生态建设**：pre-commit hooks、发行版发布流程（PyPI + tag 自动化）、CHANGELOG/towncrier、社区模板。

> Phase 1–3 的具体排期将在 Phase 0 验收后按照优先级重新评估。
