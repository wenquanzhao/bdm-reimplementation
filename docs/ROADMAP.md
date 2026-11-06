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
- [ ] Probabilistic local occupancy grid and ray casting.
- [ ] Sliding local map; global-local boundary merge.
- [ ] Streaming LiDAR point cloud integration.

## v0.4: evaluation
- [ ] ROS2 adapter, benchmark datasets, memory/update/query benchmarks.
- [ ] Compare against OctoMap and voxel hashing at matched resolution.

## Status
Not an official HKU-MaRS implementation. Claims of IJRR-equivalent accuracy, memory usage, or speed are not made. Paper: https://arxiv.org/html/2603.21774
