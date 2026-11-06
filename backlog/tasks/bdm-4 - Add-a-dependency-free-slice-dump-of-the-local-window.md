---
id: BDM-4
title: Add a dependency-free slice dump of the local window
status: To Do
assignee: []
created_date: '2026-11-06T09:45:00Z'
labels: []
dependencies: []
references:
  - src/incremental_demo.cpp
  - docs/INCREMENTAL.md
  - CMakeLists.txt
priority: medium
type: feature
ordinal: 4000
---

## Description

<!-- SECTION:DESCRIPTION:BEGIN -->
Reviewers of the Section 7 skeleton only see the text report from `src/incremental_demo.cpp` (`slides`, `global_boundaries`, `overlap_mismatched`, `incremental_commit`). Nothing prints the local window.

Add a dependency-free dump of one axis-aligned slice of local discrete states for `--synthetic`: occupied, free, and unknown counts, plus which slice was printed. Wire a CTest that checks the golden counts. Keep the current report fields. Say in `docs/INCREMENTAL.md` that this is a debug view for the sliding window, not a paper figure and not a memory or timing result.
<!-- SECTION:DESCRIPTION:END -->

## Acceptance Criteria
<!-- AC:BEGIN -->
- [ ] #1 bdm_incremental --synthetic can print one axis-aligned slice of local discrete states, including occupied, free, and unknown counts and the slice index. A CTest matches those counts to a fixed golden string.
- [ ] #2 The dump uses only the C++ already in this repo, or Python 3 with no third-party packages. No ROS 2, no plotting library, no quay.
- [ ] #3 docs/INCREMENTAL.md calls the dump a debug view, not a Section 8 figure. The existing incremental_synthetic, incremental_help, and incremental_kitti_mini pass conditions still hold.
<!-- AC:END -->

## Definition of Done
<!-- DOD:BEGIN -->
- [ ] #1 From the repo root: cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j && ctest --test-dir build --output-on-failure
- [ ] #2 Docs that describe the changed behavior (README.md or docs/) match the code
<!-- DOD:END -->
