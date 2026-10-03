# Git 工作流与提交规范

## 摘要
- 统一团队在分支、提交信息、检查流程上的约定，确保仓库历史清晰可追踪。

## 1. 分支策略

- 主干：`master`（只允许合并通过审核的 Pull Request）。
- 功能分支命名：`feature/<scope>-<short-description>`，例如 `feature/ui-snapshot-export`。
- 修复分支命名：`fix/<scope>-<issue-id>` 或 `hotfix/<scope>-<short-description>`。
- 维护/杂项：`chore/<topic>-<short-description>`。
- 分支名一律使用小写英文字母、数字与连字符，不得包含空格或中文。

## 2. 提交信息格式

遵循 [Conventional Commits](https://www.conventionalcommits.org/zh-hans/v1.0.0/) 规范，格式：

```
<type>(<scope>): <subject>

<body>

<footer>
```

- **type（必填）**：`feat` / `fix` / `docs` / `test` / `refactor` / `chore` / `build` / `ci` / `perf` / `style`。
- **scope（可选）**：模块或目录，如 `core`、`ui`、`docs`、`cli`。
- **subject**：不超过 72 字符的动词短语，使用小写陈述语气，例如 `add ros2 flag validation`。
- **body**：必要时描述动机、实现方式或兼容性影响；每行不超过 80 字符。
- **footer**：引用 Issue/PR（例如 `Closes #42`）或注明重大变更（`BREAKING CHANGE:`）。

禁止使用如下提交：`WIP`、空 message、多个无关改动打包、二进制文件无解释。

## 3. 提交内容要求

- 每次提交聚焦单一主题，确保可以独立回滚。
- 对公共 API、行为或配置的变更必须同步更新文档与测试。
- 绝不在提交中包含敏感信息（密钥、密码、访问令牌）。
- 产出的新增文件须遵循项目目录约束（如文档位于 `docs/`）。

## 4. 提交前检查清单

在本地运行以下命令并处理所有失败：

1. `ruff check`（代码风格与 lint）
2. `mypy` 或 `pyright`（类型检查）
3. `pytest`（单元测试）
4. 如修改文档，运行 `markdownlint docs/`。

若某项工具当前尚未配置，可在阶段内说明「待补」，但必须在 Pull Request 描述中列出预计补齐时间。

## 5. Pull Request 规范

- 与目标分支保持同步并解决冲突后再发起 PR。
- PR 标题遵循提交规范，可追加简短说明，例如 `feat(ui): add snapshot export panel (Phase 1)`。
- PR 描述必须包含：
  - 变更概览与动机
  - 测试与检查结果（命令输出或说明）
  - 文档更新列表（若适用）
  - 风险与回滚方案
- 关联 Issue/Task：在描述或 footer 中使用 `Closes #ID` / `Refs #ID`。
- 至少一名维护者通过代码评审后方可合并；禁止自我合并。

## 6. 提交合并策略

- 默认使用「Squash and Merge」，保留单一提交历史，提交信息取自 PR 标题。
- 如需保留多个提交（例如大型功能分解），在评审阶段说明理由并改用「Rebase and Merge」。
- Merge Commit 仅用于同步上游或发布分支。

## 变更记录
- 2025-10-13：初版规范（@xmu795）
