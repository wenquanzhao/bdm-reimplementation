---
id: BDM-5
title: Map open Section 7 gaps to the tests that lock them
status: To Do
assignee: []
created_date: '2026-11-06T09:45:00Z'
labels: []
dependencies: []
references:
  - docs/INCREMENTAL.md
  - docs/VALIDATION.md
  - docs/ROADMAP.md
  - tests/test_incremental.cpp
priority: low
type: docs
ordinal: 5000
---

## Description

<!-- SECTION:DESCRIPTION:BEGIN -->
`docs/INCREMENTAL.md` "Not in this skeleton" and `docs/VALIDATION.md` "Missing" list behavior the Section 7 skeleton does not implement: hash `(1441·x + y) mod Q` and packed entries, projection axes other than z, a per-scan clamp instead of a clamp after each hit or miss, rays that miss the window, and the live-window lag in `globalMap()`. Some of those are locked by tests. `testStreamingSlideAndReentry` requires the lag until a slide. `testEndpointOutsideWindow` requires the endpoint to stay out of the local grid.

An agent that treats every bullet as a bug will fight those tests. Add a short table to `docs/INCREMENTAL.md` that maps each gap still open at the start of this task to the test that locks it, and to the roadmap line when there is one. Do not change `src/` or the test assertions. If BDM-1 or BDM-2 has already landed, do not re-open those rows as unfinished work.

Hash packing and `x±` / `y±` stay documented gaps. They are out of scope for this docs task.
<!-- SECTION:DESCRIPTION:END -->

## Acceptance Criteria
<!-- AC:BEGIN -->
- [ ] #1 docs/INCREMENTAL.md has a table of gaps that are still open when the work starts. Each row names the bullet from Not in this skeleton or docs/VALIDATION.md Missing, and the test function or CTest that locks today's behavior (or states that no test locks it).
- [ ] #2 Rows that BDM-1 or BDM-2 have already closed are omitted or marked done. This task does not change mapping code, CMake targets, or CI.
- [ ] #3 The table points at docs/ROADMAP.md where a checkbox exists. It does not claim Section 8 metrics, OctoMap parity, or hash-packing parity.
<!-- AC:END -->

## Definition of Done
<!-- DOD:BEGIN -->
- [ ] #1 From the repo root: cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j && ctest --test-dir build --output-on-failure
- [ ] #2 Docs that describe the changed behavior (README.md or docs/) match the code
<!-- DOD:END -->
