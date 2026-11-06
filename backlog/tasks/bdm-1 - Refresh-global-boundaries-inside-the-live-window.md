---
id: BDM-1
title: Refresh global boundaries inside the live window
status: To Do
assignee: []
created_date: '2026-11-06T09:45:00Z'
labels: []
dependencies: []
references:
  - docs/INCREMENTAL.md
  - docs/ROADMAP.md
  - docs/VALIDATION.md
  - include/bdm/incremental_mapper.hpp
  - tests/test_incremental.cpp
priority: high
type: feature
ordinal: 1000
---

## Description

<!-- SECTION:DESCRIPTION:BEGIN -->
The Section 7 skeleton on main edits `BoundaryMap` only when the window slides. `docs/ROADMAP.md` (v0.3) and `docs/INCREMENTAL.md` leave this open: after a local edit that does not slide, `globalMap()` can still show the previous boundary while `query()` reads the local grid. `mergedBoundaryMap()` is only an overlap snapshot.

`tests/test_incremental.cpp` `testStreamingSlideAndReentry` locks the lag. Frame C must not rewrite the global columns. A later miss at `{-3,0,0}` must leave `globalMap().query` Occupied until the repair slide (`frame_edit` / `frame_repair`).

Close that roadmap item. When a voxel inside the live window changes discrete state, refresh its global boundary entry before the next slide. Keep Algorithm 2 slide-out set U (dirty voxels that leave, plus their 6-neighbors), slide-in fusion from equation (11), sorted columns, and `query()` reading the local grid inside the window. Do not add quay. Do not change the binary grid in `docs/KITTI_PIPELINE.md`.

If BDM-5 has already added a gap table, update the live-window row in the same change.
<!-- SECTION:DESCRIPTION:END -->

## Acceptance Criteria
<!-- AC:BEGIN -->
- [ ] #1 A case in tests/test_incremental.cpp changes a voxel inside the window without sliding, then globalMap() matches that voxel's local discrete state before any later slide.
- [ ] #2 testStreamingSlideAndReentry still covers slide-out, slide-in, and equation (11) re-entry (l_free or l_occ, not the accumulated log-odds). The assertions that a non-slide scan must leave global columns unchanged, and that {-3,0,0} stays Occupied in globalMap() until the repair slide, are updated to the new contract.
- [ ] #3 query() inside the window still reads the local grid. incremental_synthetic still prints incremental_commit: yes and overlap_mismatched: 0. testFreeVolumeRoundTrip still does not store an enclosed free cell as a boundary.
- [ ] #4 docs/INCREMENTAL.md and the v0.3 checkbox in docs/ROADMAP.md describe the refresh. They no longer say globalMap() lags inside the live window. The binary KITTI grid in docs/KITTI_PIPELINE.md is unchanged.
<!-- AC:END -->

## Definition of Done
<!-- DOD:BEGIN -->
- [ ] #1 From the repo root: cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j && ctest --test-dir build --output-on-failure
- [ ] #2 Docs that describe the changed behavior (README.md or docs/) match the code
<!-- DOD:END -->
