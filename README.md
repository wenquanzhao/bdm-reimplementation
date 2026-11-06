# BDM: Independent Paper Reimplementation

**Status:** v0.1 — static boundary-map core, not a complete reproduction of the IJRR experiments.

Independent C++17 implementation inspired by *Memory-Efficient Boundary Map for Large-Scale Occupancy Grid Mapping* (Tang et al., IJRR 2026).

- Paper: https://arxiv.org/html/2603.21774
- Original project: https://github.com/hku-mars/BDM

## Implemented

- Occupied voxels, interior-free voxels (free with a 6-neighbor that is unknown or occupied), and exterior-unknown voxels (unknown with a free 6-neighbor), including unknown cells left out of the input.
- Sparse `(x, y)` columns in `std::unordered_map`, each a `(z, boundary type)` array sorted by `z`. This is not the paper's `(1441·x + y) mod Q` table or 32-bit packed entries.
- Queries along `z+` and `z-` by binary search, using Algorithm 1: nearest interior-free boundary means free; occupied is returned only when that nearest boundary is the query cell; otherwise the result is unknown.
- CTest coverage for a free volume with an obstacle, unstored interior free cells, disconnected free regions, occupied-only and empty maps, explicit unknown cells, negative indices, and dense-grid checks on small random scenes. Details and limits are in `docs/VERIFICATION.md`.
- A KITTI odometry loader and a binary hit/miss occupancy grid that feed `BoundaryMap::rebuild`, plus a `z+`/`z-` consistency report. CI uses a synthetic fixture. Real clips are optional. See `docs/KITTI_PIPELINE.md`.

## Build and test

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure
```

## Scope and limitations

This is **static reconstruction** from an existing occupancy grid, not the complete IJRR system. It does not implement the Section 7 probabilistic ray caster, the sliding local grid, incremental global-local updates, projection axes other than `z`, ROS 2, or the paper's experiments. A column query is a hash lookup plus a binary search, `O(log k)` in the number of boundaries in that column. The paper's average `O(1)` result is not re-measured here.

`docs/KITTI_PIPELINE.md` puts a binary Free/Occupied/Unknown grid in front of this map and checks that queries reproduce the grid. That grid is not the Section 7 log-odds model, and a match rate from it is not a Section 8 result.

This is an independent implementation. The upstream repository currently says source release is pending. See `docs/ROADMAP.md` and `docs/VERIFICATION.md`.
