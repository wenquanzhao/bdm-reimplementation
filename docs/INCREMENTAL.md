# Section 7 skeleton

This is the online-mapping path in front of `BoundaryMap`. It follows Section 7 of Tang et al. (arXiv:2603.21774v1) far enough to stream posed scans, keep a log-odds window on the sensor, and edit the global boundary map when a voxel inside that window changes discrete state and when the window slides.

It is not a Section 8 result. There is no OctoMap run, no memory or timing table, and no claim of parity with the paper. The binary KITTI grid in `docs/KITTI_PIPELINE.md` is a different sensor model and is unchanged.

## What a frame does

`IncrementalMapper::integrate` takes one scan in the sensor frame and a row-major 3×4 `world_from_sensor`. The sensor origin is the pose used for sliding. There is no separate vehicle frame.

1. The first pose centers the window on the sensor voxel and does not slide. The window is `[origin, origin + size)` in voxel indices.
2. Ray casting runs on that window (Algorithm 2, `NewScanRayCasting` before the slide test). The traversal is the same Amanatides–Woo stepper as the binary grid (`include/bdm/ray_cast.hpp`). A miss adds `l_miss`, a hit adds `l_hit`, and the value is clamped after that one update. Equations (7)–(9) are applied with `n_hit, n_miss ∈ {0, 1}` per ray, not as one summed delta for the whole scan.
3. Each voxel whose discrete state changed, plus its 6-neighbors, is written into the global `BoundaryMap` before the slide test. Occupancy for that set and one further 6-neighbor ring is snapshotted first; the outer ring is context for the tag and is not stored on its own. A log-odds change that stays in the same state does not rewrite a boundary. Columns stay sorted by `z`.
4. If the sensor is farther than `slide_threshold_m` from the window center, and recentering would move the origin, the window slides. The test is the Euclidean distance in Algorithm 2, `||x_k − o|| > t`, with `o` the geometric center from Section 7.1.
5. Slide-out: dirty voxels that leave the window, plus their 6-neighbors, are the set `U`. Their occupancy is snapshotted, then each voxel in `U` is erased or inserted in the global `BoundaryMap`. Columns stay sorted by `z`.
6. Slide-in: slots that the rolling index reuses are cleared to the prior `0`, then free and occupied voxels are rebuilt from the global columns and fused with equation (11): add `l_free` or `l_occ`, then clamp. Unknown voxels are not written.

The array index is `(x, y, z)` modulo the window size. A voxel that stays inside keeps its log-odds. The slide-in region aliases the slide-out slots, so those slots are cleared only after the slide-out snapshot.

`query` follows Section 7.3 for the `z` axis. Inside the window it returns the local discrete state. Outside, it uses Algorithm 1 along `z+` if the voxel is above the window and along `z−` if it is below. A voxel beside the window in `x` or `y`, with `z` inside the slab, uses `z+` because that column does not enter the window. `x±` and `y±` are not implemented.

## Log-odds

Natural log. Section 8.2 is the default:

| Probability | Value | Role |
| --- | --- | --- |
| `p_hit` | 0.8 | added on a hit |
| `p_miss` | 0.48 | added on a miss |
| `p_min` | 0.05 | clamp floor |
| `p_max` | 0.97 | clamp ceiling |
| `p_free` | 0.2 | free if `L <= l_free` |
| `p_occ` | 0.8 | occupied if `L >= l_occ` |

The prior is `0`, which is unknown. With these defaults one hit becomes occupied (`l_hit = l_occ`) and one miss stays unknown. The synthetic CLI sets `p_miss = 0.2` so one miss becomes free. That is a demo knob. It is not the Section 8.2 miss probability.

Equation (10) is the discrete state. Equation (11) runs when a voxel slides back in: the stored log-odds become the free or occupied threshold, not the value accumulated before the voxel left. Section 8.6 describes that loss. The tests check it.

## Two grids

| | Binary `OccupancyGrid` | Log-odds `LocalGrid` |
| --- | --- | --- |
| Where | `docs/KITTI_PIPELINE.md`, `bdm_kitti` | this page, `bdm_incremental` |
| Hit | endpoint voxel is occupied, and stays occupied | add `l_hit`, clamp |
| Miss | other crossed voxels become free unless already occupied | add `l_miss`, clamp |
| Unknown | a key that was never written | `l_free < L < l_occ`, including the prior |
| Extent | sparse hash, grows with the rays | fixed window; outside voxels are not stored |
| Map update | `BoundaryMap::rebuild` after the scans | `BoundaryMap` edits when a voxel inside the window changes discrete state, and on each slide |
| Consistency number | `match_rate` against that binary grid | overlap of `mergedBoundaryMap` against the local window |

A match on either path is agreement with the grid just built. It is not a number from the paper.

## Overlap snapshot

`globalMap()` is the Algorithm 2 store. A discrete state change inside the window rewrites that cell and its 6-neighbors before any later slide. `query` still reads the local grid inside the window.

`mergedBoundaryMap()` remains the overlap copy. It copies the global map and rewrites the current window plus its one-cell shell from the states in force now (local inside, global outside). Both `z+` and `z−` on that snapshot match the local discrete state for every voxel in the window. That is the overlap check. The copy is not a second online map, and building it walks the window surface. It is not the paper's update cost.

The slide-out edit covers every voxel in `U`, including a neighbor one cell outside the slide-out footprint. The removal loop printed in Algorithm 2 iterates the slide-out projection only. Updating all of `U` is what keeps a lateral neighbor from holding a stale type. Insertion is the same set the paper recomputes (Algorithm 2, lines 35–43).

## Run

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure
./build/bdm_incremental --synthetic
```

`--synthetic` is the default. It integrates two poses and requires the report to contain `incremental_commit: yes` (a slide happened, the global map gained boundaries, and the overlap matched). Sample:

```text
scope: section 7 skeleton; log-odds local grid; sliding window; incremental boundary update; not section 8 metrics
mode: synthetic
log_odds_preset: synthetic; p_miss=0.2 so one miss is free; not section 8.2 p_miss=0.48
frames: 2
slides: 1
points_read: 4
points_integrated: 4
points_rejected_range: 0
points_rejected_nonfinite: 0
points_endpoint_outside: 0
resolution_m: 1
window_voxels: 7 7 7
slide_threshold_m: 1
global_boundaries: 33
overlap_queries: 343
overlap_mismatched: 0
incremental_commit: yes
binary_grid: false
log_odds: true
```

Optional KITTI, still not downloaded by CI:

```bash
./build/bdm_incremental \
  --root data/kitti/dataset \
  --sequence 00 \
  --max-frames 5 \
  --resolution 1 \
  --size 32 \
  --slide-threshold 1
```

`--root` keeps the Section 8.2 probabilities. `--max-frames` defaults to 5. The layout is the same one `docs/KITTI_PIPELINE.md` describes. A full sequence at a fine resolution is outside what this skeleton is for: the window is capped at 32,000,000 voxels, and rays that miss the current window are not copied into the global map. The paper sizes the window to cover the sensor. This code does not grow it.

`incremental_kitti_mini` runs the two-frame fixture at 1 m in a 16³ window. The sensor moves toward the returns, so the slide-out slab is still the prior and `global_boundaries` stays 0. The check there is `overlap_mismatched: 0` only. The non-empty commit is the synthetic scene and `tests/test_incremental.cpp`.

Exit status: 0 when the overlap matches, 1 when it does not, 2 on usage or I/O errors.

## Tests

`tests/test_incremental.cpp` (`incremental_map`):

- Section 8.2 logits, and that one default miss stays unknown while the binary grid marks the same ray free.
- A later miss clears a hit. The binary grid keeps that hit.
- Clamping at `l_max`.
- An endpoint outside the window updates only the misses that still cross it.
- A multi-frame slide. A scan that changes a voxel inside the window writes that discrete state into `globalMap()` before the next slide. Slid-out states match a second mapper that never slides. An enclosed free cell is not stored and still queries as free. Coming back writes `l_free` or `l_occ`, not the accumulated value. A later local edit updates `query` and `globalMap()` together, and the next slide leaves that updated entry in place.

## Not in this skeleton

- Hash `(1441·x + y) mod Q` and the 32-bit packed entry. Columns are still `std::unordered_map` plus a sorted `vector`.
- Projection axes other than `z`, and search directions `x±` / `y±`.
- A boundary rewrite when log-odds move but the discrete state does not. A second hit on an occupied cell stays clamped and leaves `globalMap()` as it is. `dirty_` still keeps each flipped voxel until that voxel slides out, and the slide-out rewrite uses the state at that time.
- Casting a ray that misses the window into the global map.
- One clamp at the end of the scan instead of after each hit or miss.
- ROS 2, OctoMap, and the Section 8–9 tables. Memory and speed are not measured.
