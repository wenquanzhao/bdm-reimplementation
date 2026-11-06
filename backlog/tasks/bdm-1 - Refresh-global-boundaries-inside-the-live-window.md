---
id: BDM-1
title: Refresh global boundaries inside the live window
status: In Progress
assignee:
  - '@zhongli'
created_date: '2026-11-06T09:45:00Z'
updated_date: '2026-11-06T09:59:00Z'
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

## Implementation Plan

<!-- SECTION:PLAN:BEGIN -->
Implementation is deferred until yale accepts this plan. This checkpoint changes only the Backlog task.

## Where to refresh

Hook `IncrementalMapper::integrate` in `src/incremental_mapper.cpp` immediately after `local_.integrate` returns and before the `distanceFromCenter` slide test.

`LocalGrid::accumulate` (`src/local_grid.cpp`) inserts a voxel into the set passed to `integrate` only when `model_.state` changes (`before != after`). That is the live-window discrete change. Log-odds that move but stay in the same state (a second hit on an already occupied cell, clamping at `l_max`) must not rewrite a boundary.

Do not pass the persistent `dirty_` set into that `integrate` call. `dirty_` keeps every voxel that changed and has not yet slid out, so a membership diff misses a second flip of a voxel that is already dirty. Frame C does this to `{2,0,0}`: frame A marks it occupied, frame B slides but the cell stays inside (`origin.x == 0`, voxel x = 2), and frame C's miss changes it to unknown while it is still in `dirty_`.

```text
std::unordered_set<Voxel, VoxelHash> changed_now;
update.rays = local_.integrate(points_sensor, world_from_sensor, changed_now);
dirty_.insert(changed_now.begin(), changed_now.end());
refreshLiveBoundaries(changed_now);
// existing slide test and slideTo() stay after this
```

Add private `IncrementalMapper::refreshLiveBoundaries`. For each voxel in `changed_now` that `local_.contains`:

1. Build the update set as that voxel plus its 6-neighbors (`kNeighbors` / `shift`, same shape as Algorithm 2 set U in `slideTo`). A neighbor one cell outside the window can change between interior-free, exterior-unknown, and absent when this cell flips.
2. Snapshot occupancy for the update set and one further 6-neighbor ring so `boundaryTag` sees a full neighborhood. Inside the window use `local_.occupancy`. Outside use `queryGlobalAvoiding` with the current origin. This matches `slideTo`'s `remember` lambda, and the window has not moved yet.
3. For each voxel in the update set, call the existing file-local `boundaryTag`. `insertAt` when it returns a tag, `eraseAt` when it returns empty. Columns stay sorted because those two `BoundaryMap` methods already keep `z` order.

Reuse `boundaryTag`. An enclosed free cell (free with six free neighbors) returns empty and is erased. That is what `testFreeVolumeRoundTrip` locks for `{0,0,0}`. Do not store every free cell.

Leave `slideTo` in place after the refresh. It still builds U from dirty voxels that leave plus their 6-neighbors, snapshots before `place()` and the slide-in clear, and fuses slide-in with equation (11) via `fuseSlideIn` / `fuseVoxel`. A cell refreshed and then slid out in the same frame is written twice with the same tag; `insertAt` replaces and `eraseAt` drops, so the second write is idempotent. Do not refresh after `place()`.

Do not hook `fuseVoxel`. Slide-in copies the global state into the local grid. The comment there already records a clamp mismatch in `dirty_` for the next slide-out. `query()` stays `combinedState`: local occupancy inside the window, global map outside. `mergedBoundaryMap()` stays the overlap snapshot.

Update the class comment in `include/bdm/incremental_mapper.hpp` (the paragraph that says `globalMap()` is updated on slides, not on every ray). State that a discrete state change inside the window refreshes that cell and its 6-neighbors before any later slide, and that slide-out U and equation (11) are unchanged. Rays that never enter the window stay out of the global map (BDM-2).

## testStreamingSlideAndReentry

File: `tests/test_incremental.cpp`, function `testStreamingSlideAndReentry`. Keep the scene (window 7, threshold 1 m, strong miss `p_miss = 0.2`, poses A/B/C, return, edit, repair) and the log-odds checks.

- Frame A, the `boundaryCount() != 0` failure "frame A wrote the global map": the first scan does not slide and does change discrete state, so the global map must gain boundaries. Assert `slideCount() == 0`, `globalMap().query({-3,0,0}) == Occupied`, `globalMap().query({-2,0,0}) == Free`, and `expectSorted`. This is the AC #1 case together with frame C and `frame_edit`.
- Frame B: keep the slide (`slideCount() == 1`), `committed_voxels >= 3`, stored occupied entry at `{-3,0,0}`, sorted columns, origin, overlap log-odds for `{2,0,0}`, and `expectAgree` / `expectOverlap`. Keep copying `global_after_slide` only as a discarded control; nothing may require later non-slide frames to match it.
- Frame C, delete the `sameColumns(sliding.globalMap(), global_after_slide)` failure "a scan that does not slide rewrote the global boundary map". Assert `!frame_c.slid`, `slideCount() == 1`, `query({2,0,0}) == Unknown`, and `globalMap().query({2,0,0}) == Unknown` before the return slide. `{2,0,0}` was occupied and stays in the window; the miss flips it without a slide.
- Return: keep equation (11). `{-3,0,0}` odds equal `l_occ`, `{-2,0,0}` and `{-1,0,0}` equal `l_free`, `{-2,0,1}` stays `0`, odds are not the accumulated pre-slide value, `slide_in_fused >= 3`, origin x is `-3`, and slide-in does not delete the occupied global entry at `{-3,0,0}`.
- `frame_edit`: delete the failure "global entry should still be the occupied boundary until the next slide". After the outside-endpoint miss, local odds at `{-3,0,0}` are `0`, `query` is Unknown, and `globalMap().query({-3,0,0})` is Unknown before `frame_repair`. Expect an `ExteriorUnknown` entry there because `{-2,0,0}` is still free. `query()` remains the local read (`expectOccupancy` stays).
- `frame_repair`: still slides (`slideCount() == 3`). The entry is already `ExteriorUnknown` from `frame_edit`; the slide must leave `globalMap().query({-3,0,0})` Unknown and must not restore Occupied. Reword the failure so the slide is no longer described as the first replacement of a stale occupied boundary.

No new test file. `expectOverlap`, `testPaperMissStaysUnknown`, `testMissClearsOccupied`, and `testClamp` stay as they are. `incremental_demo.cpp` keeps `incremental_commit` as slides > 0 and `boundaryCount() > 0` and `overlap_mismatched == 0`.

## Docs

BDM-5 is still To Do and has not added a gap table. Do not add that table here. Drop the live-window lag from the open-gap lists so a later BDM-5 table does not record it as unfinished.

- `docs/INCREMENTAL.md`: "Two grids" map-update row; "Overlap snapshot" (global entries inside the window follow discrete changes; `query` still reads the local grid; `mergedBoundaryMap` remains the overlap copy); Tests bullet that says a non-slide scan leaves the global map unchanged and that `globalMap()` stays occupied until the next slide; "Not in this skeleton" bullet "Rewriting global boundaries on every ray...". After `./build/bdm_incremental --synthetic`, replace the sample block if `global_boundaries` or another line changes. The commit line stays `incremental_commit: yes` and `overlap_mismatched: 0`.
- `docs/ROADMAP.md`: check the v0.3 item "Refresh global boundaries inside the live window before the next slide" and remove the sentence that `globalMap()` can lag there.
- `docs/VALIDATION.md` "Missing": remove the clause that a live-window refresh is absent and that `globalMap()` lags until the next slide. Leave hash packing, other projection axes, timings, ROS 2, and OctoMap benchmarks in that paragraph.
- `docs/VERIFICATION.md` Section 7 row: remove "global entries inside the window can lag until the next slide".
- `README.md` (Definition of Done item 2): the two sentences that say the boundary map is edited when the window slides. Say it is also edited when a voxel inside the window changes discrete state.
- `docs/KITTI_PIPELINE.md`: leave the file unchanged. The binary grid is out of scope.

## Risks

- Refreshing only the changed voxel leaves a stale neighbor tag. The 6-neighbor update set is required. The extra ring is snapshot-only, matching `slideTo`.
- Refreshing after `place()` would snapshot cleared slide-in slots. The call stays before the slide test.
- Frame A will now publish boundaries that used to appear only at slide-out. Equation (11) on the return reads those columns. Sharing `boundaryTag` with slide-out keeps `{-3,0,0}` occupied and the ray free, so re-entry stays `l_occ` / `l_free`. If a tag differs, fix the shared classifier rather than special-casing the test.
- `mergedBoundaryMap` walks the window plus its shell. The refresh set covers neighbors of cells that actually changed. Overlap must stay `overlap_mismatched: 0`. If it does not, the snapshot used for `boundaryTag` is missing a neighbor, not a second map.
- A fresh `changed_now` set is required. Diffing against persistent `dirty_` drops frame C and any later flip of a voxel that has not left.
- `fuseVoxel` can disagree with the requested state and only marks `dirty_`. That path is unchanged. Call it out in review if a slide-in cell should also refresh immediately.
- The synthetic sample's `global_boundaries: 12` is likely to change because in-window cells such as `{2,0,0}` are stored before they leave. Update the sample from the command output. Do not change the CLI predicate to preserve 12.
- Out of scope for the implementation PR: quay, `docs/KITTI_PIPELINE.md`, hash packing, `x±` / `y±`, persisting rays that miss the window (BDM-2), and the BDM-5 gap table.
<!-- SECTION:PLAN:END -->

## Comments

<!-- COMMENTS:BEGIN -->
author: @zhongli
created: 2026-11-06T09:59:00Z

---

Plan checkpoint for yale. Status is In Progress. This change records the plan only; mapper, tests, and docs are not modified yet.

---
<!-- COMMENTS:END -->
