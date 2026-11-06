---
id: BDM-3
title: Harden the incremental KITTI fixture so a slide commits
status: To Do
assignee: []
created_date: '2026-11-06T09:45:00Z'
labels: []
dependencies: []
references:
  - docs/INCREMENTAL.md
  - docs/KITTI_PIPELINE.md
  - scripts/run_kitti_boundary.sh
  - tests/fixtures/kitti_mini
  - CMakeLists.txt
priority: medium
type: enhancement
ordinal: 3000
---

## Description

<!-- SECTION:DESCRIPTION:BEGIN -->
`incremental_kitti_mini` in `CMakeLists.txt` runs `tests/fixtures/kitti_mini` for two frames at 1 m in a 16³ window (`--max-frames 2 --slide-threshold 1`). `docs/INCREMENTAL.md` says the sensor moves toward the returns, so the slide-out slab is still the prior, `global_boundaries` stays 0, and the CTest only requires `overlap_mismatched: 0`. The non-empty commit is `--synthetic` and `tests/test_incremental.cpp`.

`scripts/run_kitti_boundary.sh` covers the binary `bdm_kitti` path only. There is no matching runner for `bdm_incremental`.

Add a fixture the CI can run, in the same on-disk layout as `tests/fixtures/kitti_mini`, where the sensor motion slides the window across occupied or free cells and `incremental_commit: yes` is true (`slides > 0`, `global_boundaries > 0`, `overlap_mismatched: 0`). Keep the current two-frame mini test. Add `scripts/run_kitti_incremental.sh` for an optional local KITTI drop: exit 2 when scans are absent, and do not download them. Document both in `docs/INCREMENTAL.md`. Do not add quay, and do not change `.github/workflows/ci.yml` unless a new CTest is the only workflow-visible change (CTest is picked up by the existing `ctest` step).
<!-- SECTION:DESCRIPTION:END -->

## Acceptance Criteria
<!-- AC:BEGIN -->
- [ ] #1 A checked-in fixture under tests/fixtures/ uses the KITTI layout (poses/, sequences/<seq>/calib.txt, velodyne/*.bin) and makes bdm_incremental slide. A CTest requires incremental_commit: yes and overlap_mismatched: 0. global_boundaries is greater than 0.
- [ ] #2 The existing incremental_kitti_mini test still passes. CI does not download KITTI and does not need data/.
- [ ] #3 scripts/run_kitti_incremental.sh runs build/bdm_incremental on a local drop and exits 2 when the velodyne directory is missing, with a pointer at the fetch command. It does not fetch scans itself. docs/INCREMENTAL.md documents the script, the fixture, and that a real sequence is still not a Section 8 result.
<!-- AC:END -->

## Definition of Done
<!-- DOD:BEGIN -->
- [ ] #1 From the repo root: cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j && ctest --test-dir build --output-on-failure
- [ ] #2 Docs that describe the changed behavior (README.md or docs/) match the code
<!-- DOD:END -->
