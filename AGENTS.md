
<!-- BACKLOG.MD GUIDELINES START -->
<!-- backlog.md-instructions-version: 1.55.3 -->
<CRITICAL_INSTRUCTION>

## Backlog.md Workflow

This project uses Backlog.md for task and project management.

**At the beginning of each conversation in this project, run `backlog instructions overview` before answering or taking action. Re-read it only if you have not read it yet in the current conversation.**

Use the overview to decide whether to search, read, create, or update Backlog tasks.

Before task lifecycle actions, read the matching detailed guide:
- `backlog instructions task-creation` before creating or splitting tasks
- `backlog instructions task-execution` before planning, changing status or assignee, adding a plan or implementation notes, or implementing task work
- `backlog instructions task-finalization` before checking acceptance criteria, writing final summaries, or moving tasks to terminal statuses

Use `backlog <command> --help` before running unfamiliar commands. Help shows options, fields, and examples.

Do not edit Backlog task, draft, document, decision, or milestone markdown files directly. Use the `backlog` CLI so metadata, relationships, and history stay consistent.

</CRITICAL_INSTRUCTION>
<!-- BACKLOG.MD GUIDELINES END -->

## BDM agent loop

Read `docs/agent-workflow.md` before implementing a task. Backlog.md is the board. Do not install or wire quay.

- Columns, exact names: To Do, Ready, In Progress, In Review, Needs Human, Done.
- Ding (or a human) moves work To Do → Ready. 钟离 implements one Ready task per PR. yale reviews In Review. Mike handles only Needs Human.
- Leave new tasks in To Do. Do not promote your own task to Ready.
- Definition of Done includes this command from the repo root. `.github/workflows/ci.yml` runs the same `ctest --test-dir build --output-on-failure` under g++ and clang++, Debug and Release.

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure
```
