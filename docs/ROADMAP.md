# Reproduction roadmap

## v0.1: static reference representation
- [x] Sparse 2D hash columns and sorted boundary entries.
- [x] 6-neighbor interface extraction and explicit occupied cells.
- [x] Binary-search occupation query.
- [x] Minimal CTest smoke tests.
- [ ] Verify exact equivalence to a dense grid on randomized 3D scenes.
- [ ] Resolve ambiguous FREE/UNKNOWN intervals in complex columns using paper algorithm.

## v0.2: exact paper query
- [ ] Reimplement the paper's boundary-direction/query predicates and optimized lookup.
- [ ] Unit tests for holes, touching occupied surfaces, disconnected free regions, negative indices.

## v0.3: incremental mapping
- [ ] Probabilistic local occupancy grid and ray casting.
- [ ] Sliding local map; global-local boundary merge.
- [ ] Streaming LiDAR point cloud integration.

## v0.4: evaluation
- [ ] ROS2 adapter, benchmark datasets, memory/update/query benchmarks.
- [ ] Compare against OctoMap and voxel hashing at matched resolution.

## Status
Not an official HKU-MaRS implementation. Claims of IJRR-equivalent accuracy, memory usage, or speed are not made. Paper: https://arxiv.org/html/2603.21774
