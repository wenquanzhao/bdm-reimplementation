# Reproduction roadmap

## v0.1: static reference representation
- [x] Sparse 2D hash columns and sorted boundary entries.
- [x] 6-neighbor interface extraction and explicit occupied cells.
- [x] Binary-search occupancy query along `z+` and `z-` (Algorithm 1 predicates).
- [x] CTest coverage, including a dense-grid oracle on small random scenes.
- [x] Interior-free boundaries at free/unknown and free/occupied faces.

## v0.2: remaining paper-query gaps
- [x] Boundary-direction predicates for the `z` projection (`z+` and `z-`).
- [x] Unit tests for holes, touching occupied surfaces, disconnected free regions, negative indices.
- [ ] Paper hash `(P·x + y) mod Q` (`P = 1441`) and Int32 packing of `(z, type)`. The stored columns are behaviorally the boundary set; the exact layout is not.
- [ ] Projection axes other than `z`, and search directions `x±` / `y±`.

The average `O(1)` claim is a theorem about binary search over a whole map. It is not a separate lookup and is not benchmarked here.

## KITTI static occupancy hook
- [x] Load odometry velodyne bins, `calib.txt` `Tr`, and `poses/<seq>.txt`.
- [x] Accumulate a dense Free/Occupied/Unknown grid with a documented hit/miss ray cast (not Section 7 log-odds).
- [x] `BoundaryMap::rebuild` plus `z+`/`z-` queries compared with that grid. The CI fixture does not download KITTI. See `docs/KITTI_PIPELINE.md`.

## v0.3: incremental mapping
- [x] Probabilistic local occupancy grid and per-ray log-odds updates with the Section 8.2 default probabilities (Section 7.2.1 skeleton).
- [x] Fixed-size sliding window with a rolling index, slide-out boundary edits, and slide-in fusion (Sections 7.1 and 7.2.2–7.2.3 skeleton).
- [x] Streaming posed scans through `IncrementalMapper`. CI uses a synthetic scene. A KITTI root is optional and is not downloaded. See `docs/INCREMENTAL.md`.
- [ ] Refresh global boundaries inside the live window before the next slide. Until then, `globalMap()` can lag there; `query()` reads the local grid, and `mergedBoundaryMap()` is the overlap snapshot.
- [ ] Paper hash packing and projection axes other than `z` (still the open v0.2 items). Those are not required for this skeleton's `z` queries.

## v0.4: evaluation
- [ ] ROS2 adapter, benchmark datasets, memory/update/query benchmarks.
- [ ] Compare against OctoMap and voxel hashing at matched resolution.

## Status
Not an official HKU-MaRS implementation. Claims of IJRR-equivalent accuracy, memory usage, or speed are not made. Paper: https://arxiv.org/html/2603.21774
