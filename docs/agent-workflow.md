# 代理工作流

本仓库用 [Backlog.md](https://github.com/MrLesk/Backlog.md) 做可审阅的看板。CLI 是 npm 包 `backlog.md` 提供的 `backlog`。没有 quay，也不要安装或接入 quay。一块任务对应一个 PR。

实现前先读 `backlog instructions overview`。创建任务读 `backlog instructions task-creation`，动手改代码读 `backlog instructions task-execution`，收尾读 `backlog instructions task-finalization`。任务正文用 `backlog` 改，不要手改 `backlog/tasks/` 里的 Markdown。

## 角色

| 谁 | 做什么 |
| --- | --- |
| Ding | 项目负责人 / 管家。从 To Do 挑选下一块工作，标成 Ready。看板上同时只有一块 Ready 给实现。 |
| 钟离 | 编码。一次只领一块 Ready 任务：先把计划写进任务，再改代码、跑测试、开一个 PR。 |
| yale | 审阅。看 In Review 的 PR 和验收标准。 |
| Mike | 只处理 Needs Human。不实现 Ready 任务。 |

## 状态

列名固定为：`To Do`、`Ready`、`In Progress`、`In Review`、`Needs Human`、`Done`。

`Done` 是最后一列，表示已验证完成。新任务默认停在 `To Do`。编码代理不要把自己的任务从 `To Do` 拉到 `Ready`。

## 循环

1. **Ready 门。** 人或者 Ding 把一块 `To Do` 标成 `Ready`。没有 Ready 任务时，编码代理停下来，不要自己开工。
2. **计划写进任务。** 钟离只领这一块 Ready。读代码和 `docs/`，用 `backlog task edit` 把实现计划写进任务，然后再改代码。
3. **实现。** 一次一块任务，一个分支，一个 PR。不要顺手做下一块。
4. **测试。** Definition of Done 包含从仓库根目录执行：

   ```bash
   cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
   cmake --build build -j
   ctest --test-dir build --output-on-failure
   ```

   这是 `README.md` 里的项目命令。CI（`.github/workflows/ci.yml`）用同一条 `ctest --test-dir build --output-on-failure`，矩阵是 g++ / clang++ 和 Debug / Release。本地默认跑 Release 即可；任务碰到编译器差异时再对第二个编译器复跑。
5. **PR，然后 In Review。** 测试通过后开 PR，把任务标成 `In Review`，PR 链接写进任务。不要开第二个 PR，不要合并。
6. **审阅后 Done。** yale 审过、验收标准满足后，标成 `Done`。需要人做决定、缺数据或卡住时标 `Needs Human`，交给 Mike，并在任务里写清问题。不要在 `Needs Human` 上继续改代码。

## 不要做的事

- 不要安装、配置或依赖 quay。
- 不要一块 PR 里塞多块任务。
- 不要为了过看板去改 CI，除非任务写明要改 CI。
- 不要下载 KITTI 进 CI。真实序列仍按 `docs/KITTI_PIPELINE.md` 在本地可选运行。
