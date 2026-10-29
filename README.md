# BDM: Independent Paper Reimplementation

**Status:** v0.1 — static boundary-map core, not a complete reproduction of the IJRR experiments.

Independent C++17 implementation inspired by *Memory-Efficient Boundary Map for Large-Scale Occupancy Grid Mapping* (Tang et al., IJRR 2026).

- Paper: https://arxiv.org/html/2603.21774
- Original project: https://github.com/hku-mars/BDM

## Implemented

- Explicit occupied voxels and six-connected interior-free / exterior-unknown boundaries.
- Sparse (x,y) hash columns with sorted (z,boundary type) entries.
- Positive/negative Z occupancy queries using binary search.
- Tests for a free cube with an obstacle, disconnected intervals, occupied-only and empty maps.

## Build and test

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure
```

## Scope and limitations

This is **static reconstruction** from an existing occupancy grid, not the complete IJRR algorithm. It does not implement probabilistic ray casting, sliding local grid, incremental global-local updates, O(1) optimized query, ROS2 integration or paper-scale benchmarks. Column query is O(log k).

This is an independent implementation; the authors' repository currently says source release is pending. See docs/ROADMAP.md.
