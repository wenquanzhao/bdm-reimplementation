# 合议敏捷

暂定名。Mike 可能改名。

合议敏捷是一套可复用的多机器人敏捷架构：Ding 负责，编码机器人实现，审阅机器人审阅，看板是 [Backlog.md](https://github.com/MrLesk/Backlog.md)（CLI 为 npm 包 `backlog.md` 的 `backlog`）。同一套角色和状态可以接到别的仓库。`bdm-reimplementation` 是第一个试点。这个仓库 `backlog/` 里的 `BDM-*` 任务是试点项目的工作，流程本身不叫 BDM。

没有 quay，也不要安装或接入 quay。一块任务对应一个 PR。

实现前先读 `backlog instructions overview`。创建任务读 `backlog instructions task-creation`，动手改代码读 `backlog instructions task-execution`，收尾读 `backlog instructions task-finalization`。任务正文用 `backlog` 改，不要手改 `backlog/tasks/` 里的 Markdown。

## 架构

| 角色 | 做什么 |
| --- | --- |
| Ding | 负责人 / 管家。从 To Do 挑选下一块工作，标成 Ready。看板上同时只有一块 Ready 给实现。 |
| 编码机器人 | 一次只领一块 Ready 任务：先把计划写进任务，再改代码、跑该仓库的测试、开一个 PR。 |
| 审阅机器人 | 看 In Review 的 PR 和验收标准。 |
| Backlog.md | 看板。列名在每个接入的仓库里保持一致。 |

人只在例外路径出现：Mike 只处理 Needs Human，不实现 Ready 任务。

在试点 `bdm-reimplementation` 上，编码机器人是钟离，审阅机器人是 yale。换仓库时换具体机器人，上面四个角色不变。

## 状态

列名固定为：`To Do`、`Ready`、`In Progress`、`In Review`、`Needs Human`、`Done`。

`Done` 是最后一列，表示已验证完成。新任务默认停在 `To Do`。编码机器人不要把自己的任务从 `To Do` 拉到 `Ready`。

## 循环

1. **Ready 门。** 人或者 Ding 把一块 `To Do` 标成 `Ready`。没有 Ready 任务时，编码机器人停下来，不要自己开工。
2. **计划写进任务。** 编码机器人只领这一块 Ready。读该仓库的代码和文档，用 `backlog task edit` 把实现计划写进任务，然后再改代码。
3. **实现。** 一次一块任务，一个分支，一个 PR。不要顺手做下一块。
4. **测试。** Definition of Done 包含跑该仓库自己的测试。命令写在该仓库的 Backlog 配置里，不要猜。
5. **PR，然后 In Review。** 测试通过后开 PR，把任务标成 `In Review`，PR 链接写进任务。不要开第二个 PR，不要合并。
6. **审阅后 Done。** 审阅机器人审过、验收标准满足后，标成 `Done`。需要人做决定、缺数据或卡住时标 `Needs Human`，交给 Mike，并在任务里写清问题。不要在 `Needs Human` 上继续改代码。

## 试点仓库的测试

`bdm-reimplementation` 的 Definition of Done 使用 `README.md` 里的命令。CI（`.github/workflows/ci.yml`）用同一条 `ctest --test-dir build --output-on-failure`，矩阵是 g++ / clang++ 和 Debug / Release。本地默认跑 Release 即可；任务碰到编译器差异时再对第二个编译器复跑。

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure
```

这个试点不要把 KITTI 下载进 CI。真实序列仍按 `docs/KITTI_PIPELINE.md` 在本地可选运行。

## 不要做的事

- 不要安装、配置或依赖 quay。
- 不要一块 PR 里塞多块任务。
- 不要为了过看板去改 CI，除非任务写明要改 CI。
- 不要把合议敏捷写成某个仓库的专名。项目任务留在该仓库的 `backlog/`。
