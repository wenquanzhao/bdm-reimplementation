# Validation contract

Independent static prototype, not the complete BDM algorithm.

## Required invariants
- Occupied input cells query as occupied.
- Free input cells query as free, including free-to-occupied interfaces.
- Missing cells query as unknown.
- Rebuild discards previous state.
- Negative coordinates and disconnected free intervals work.

## Validation strategy
- Dense-vs-sparse exhaustive randomized differential testing.
- Adversarial columns with occupied/free/unknown transitions.
- Sanitizers and GCC/Clang CI.
- Performance measurement with resident memory, throughput, and update time.

## Checked by ctest
The invariants above, both `z` search directions, interior-free presence after a free-region rebuild, and agreement with the dense grid on the scenes in `tests/test_boundary_map.cpp`. See `docs/VERIFICATION.md` for the paper comparison and the latest local result.

## KITTI static grid
`tests/test_kitti_grid.cpp`, `kitti_fixture`, and `kitti_help` cover the loader, the hit/miss ray grid, and agreement with `BoundaryMap` on a tiny synthetic sequence. They do not download KITTI and they are not the paper's benchmarks. A real sequence is an optional local run; the steps are in `docs/KITTI_PIPELINE.md`. That grid is binary. It is not the Section 7 log-odds model.

## Section 7 skeleton
`tests/test_incremental.cpp` streams posed scans into `IncrementalMapper`. It checks per-ray log-odds against the Section 8.2 defaults, that a miss can clear a hit, clamping, a ray that leaves the window, a discrete-state change inside the window that updates `globalMap()` before the next slide, a slide that edits the global `BoundaryMap` in place, overlap agreement of `mergedBoundaryMap` with the local grid along `z+` and `z-`, slid-out cells against a mapper that never slides, reconstruction of an enclosed free cell, and equation (11) re-entry. `incremental_synthetic` and `incremental_kitti_mini` run the CLI. None of these download KITTI or report a Section 8 metric. The surface and the gaps are in `docs/INCREMENTAL.md`.

## Missing
The paper's hash and packed-entry layout, projection axes other than `z`, measured average-`O(1)` query timings, ROS 2, and large-scale benchmarks against OctoMap. No memory or speed parity with the paper is claimed.
