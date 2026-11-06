---
id: BDM-2
title: Persist rays that miss the sliding window
status: To Do
assignee: []
created_date: '2026-11-06T09:45:00Z'
labels: []
dependencies: []
references:
  - docs/INCREMENTAL.md
  - include/bdm/incremental_mapper.hpp
  - tests/test_incremental.cpp
priority: high
type: feature
ordinal: 2000
---

## Description

<!-- SECTION:DESCRIPTION:BEGIN -->
`IncrementalMapper` drops geometry that never lands in the current window. `docs/INCREMENTAL.md` lists "Casting a ray that misses the window into the global map" as not implemented. `include/bdm/incremental_mapper.hpp` says those rays are not written into the global map. The paper sizes the window to cover the sensor; this skeleton does not grow it (cap 32,000,000 voxels).

`testEndpointOutsideWindow` integrates `{10,0,0}` into a size-5 window. It checks `points_endpoint_outside == 1`, that the endpoint is not stored locally, and that the last inside voxel and the sensor voxel are free misses. Cells past the window are not applied to `globalMap()`.

Apply those out-of-window cells to the global `BoundaryMap`: the endpoint voxel becomes occupied, and miss voxels the ray would have painted outside the window become free, using the same hit/miss logits and clamp as the local grid. Do not write the local grid outside its bounds and do not grow the window. Leave `docs/KITTI_PIPELINE.md` and its binary hit/miss grid alone.

If BDM-1 has already refreshed in-window boundaries, keep that behavior. This task is only the out-of-window path. If BDM-5 has added a gap table, update that row in the same change. Do not add quay.
<!-- SECTION:DESCRIPTION:END -->

## Acceptance Criteria
<!-- AC:BEGIN -->
- [ ] #1 A unit test integrates a return whose endpoint is outside the window. The global BoundaryMap records that endpoint as occupied and the out-of-window miss voxels as free. The local grid still rejects the endpoint (local().contains is false), and the window size does not grow.
- [ ] #2 testEndpointOutsideWindow still checks points_endpoint_outside and the in-window misses (the last inside voxel and the sensor voxel stay free). It also states the new global-map side effects for the cells the ray does not store locally.
- [ ] #3 Hit, miss, and clamp use the same log-odds as the local grid for that run. The 32,000,000 voxel window cap in docs/INCREMENTAL.md stays. CI does not download KITTI. The binary OccupancyGrid path is unchanged.
- [ ] #4 docs/INCREMENTAL.md no longer lists casting a ray that misses the window as unimplemented. It says the update is not a Section 8 metric. --synthetic still prints incremental_commit: yes.
<!-- AC:END -->

## Definition of Done
<!-- DOD:BEGIN -->
- [ ] #1 From the repo root: cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j && ctest --test-dir build --output-on-failure
- [ ] #2 Docs that describe the changed behavior (README.md or docs/) match the code
<!-- DOD:END -->
