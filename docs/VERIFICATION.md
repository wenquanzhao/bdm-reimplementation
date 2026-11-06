# Verification of the static boundary-map core

This note compares the code in this repository with *Memory-Efficient Boundary Map for Large-Scale Occupancy Grid Mapping* (Tang et al., arXiv:2603.21774v1, 23 March 2026). The comparison uses the arXiv HTML page and the paper source file `main.tex` from the e-print. It is a check of this independent static core. It is not a run of the paper's experiments, and the upstream implementation was not available for a line-by-line diff (the project page says the source release is pending).

## Bug that was fixed

In `src/boundary_map.cpp`, the interior-free keep condition was on the same physical line as a `//` comment. The comment text contained the two characters `\` and `n`, not a newline, so the preprocessor never saw a new line and the `if` was comment text:

```text
// ... otherwise\n        // a vertical FREE run ...\n        if (state == State::Free && neighbor != State::Free) keep = true;
```

That was confirmed from the file bytes before the edit (`repr` of the line shows `\\n` inside the comment and a single real newline only after `keep = true;`).

`rebuild` still stored occupied cells, and unknown cells that touch a free cell. It did not store any free cell. `Boundary::InteriorFree` was therefore never written, and the free result of `query` was unreachable.

The smoke test already required a free result. Built from that pre-fix tree with

```bash
g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic -I include \
  src/boundary_map.cpp tests/test_boundary_map.cpp -o /tmp/bdm_test_before
/tmp/bdm_test_before
```

it aborted:

```text
tests/test_boundary_map.cpp:14: int main(): Assertion `map.query({2,2,3}) == State::Free' failed.
```

Process status 134 (abort). The occupied check on the previous line had passed. `(2,2,3)` is free and 6-connected to the occupied cell `(2,2,4)` in that fixture, which is exactly an interior-free voxel under the paper's definition.

The keep condition is now a real statement: a free voxel is retained when any 6-neighbor is not free (unknown or occupied). That matches the paper's boundary-interior definition, not only the free/unknown face.

## Query predicates

`query` previously returned free only for an exact interior-free entry, or when the query sat strictly between two interior-free entries in the same column. Anything else, including a nearer occupied or exterior-unknown boundary, fell through to unknown. Those are not the predicates in Algorithm 1.

Algorithm 1 (`main.tex`, "Determine Occupancy State") is:

1. `b_nn, r_min ← FindNearestInDirection(q, E)`.
2. If `b_nn` is null, return unknown.
3. If `b_nn` is an interior boundary, return free.
4. Otherwise, if `b_nn` is occupied and `r_min = 0`, return occupied.
5. Otherwise return unknown.

With `z` as the projection axis, Section 6.2 defines the nearest boundary as the first stored boundary with `z ≥ q^z` for `z+`, and the first with `z ≤ q^z` for `z-`. `r_min = |b_nn^z − q^z|`, so `r_min = 0` means the query cell is that boundary. A missing column, or no boundary in the chosen direction, is the null case (unknown).

`BoundaryMap::query(v)` uses `z+`. `query(v, SearchDirection::NegativeZ)` uses `z-`. Both use the predicates above. The old enclosing-pair rule is gone.

On a boundary set that matches Section 4, the paper's theorem says either direction returns the dense-grid state. The tests check that agreement. They do not re-prove the theorem.

## Paper ↔ code

| Paper (arXiv:2603.21774v1) | This repository |
| --- | --- |
| Section 4. Interior boundary `b_int`: free, with a 6-neighbor that is unknown or occupied. | `rebuild`: `state == Free && neighbor != Free`, stored as `Boundary::InteriorFree`. |
| Section 4. Exterior unknown `b_ukn`: unknown, with a free 6-neighbor. | Explicit unknown inputs, plus unknown cells omitted from the input (absence is treated as unknown). Stored as `Boundary::ExteriorUnknown`. |
| Section 4. Exterior occupied `b_occ`: every occupied voxel. | Every occupied input cell, stored as `Boundary::Occupied`. |
| Section 6.1. Project boundary voxels along `z` into `(x, y)` columns, sorted by `z`, with a type tag. | `std::unordered_map` of columns, `Entry{int z; Boundary boundary}`, sorted by `z` after `rebuild`. |
| Section 6.1. Hash `(P·x + y) mod Q` with `P = 1441`, and a 32-bit packing (30-bit `z`, 2-bit type). | Not implemented. The column key is hashed by `std::unordered_map`. `z` is an `int` and the type is an enum. |
| Section 6.2 and Algorithm 1. Nearest boundary in `z+` or `z-`, then the predicates above. | `query` / `query(v, direction)`. Binary search on the sorted column. |
| Search directions `x±` and `y±`, which need a different projection axis. | Not implemented. The map is projected on `z` only. |
| Theorem for `r_min > 0`: nearest interior boundary ⇒ free; nearest exterior boundary ⇒ unknown. Null search ⇒ unknown. | Behavior follows from Algorithm 1. Not a separate proof artifact. |
| Average query `O(1)` over a whole map (the binary search cost averaged over cells). | Not measured. One column query here is a hash lookup plus `O(log k)`. |
| Section 7. Local log-odds grid, ray casting, sliding window, incremental boundary update, region query. | Skeleton in `IncrementalMapper`. Log-odds ray casting, a fixed rolling window, slide-out boundary edits, and slide-in fusion are implemented. See `docs/INCREMENTAL.md` for the gaps (no other projection axis, no packed hash, global entries inside the window can lag until the next slide). |
| Sections 8–9. Benchmarks and real-world flights. | Not run. No claim of matching those memory, timing, or accuracy figures. |

## What the tests cover

`tests/test_boundary_map.cpp` is the CTest target `boundary_map`. Each scene is checked in two ways:

- Every stored boundary matches the Section 4 definition, every boundary the definition requires is stored, columns are strictly increasing in `z`, and `boundaryCount()` equals the number of stored entries.
- For every voxel in a window two cells outside the scene, and for two far-away voxels, both `z+` and `z-` equal the dense input (a missing key is unknown). `query(v)` is required to equal the `z+` result.

Scenes:

- Free `5×5×8` box with one occupied cell. The free cell beside that obstacle must be stored as interior free. A free cell with six free neighbors must not be stored, and must still query as free.
- Occupied-only points, including negative coordinates. No interior-free or exterior-unknown entries. A cell whose nearest boundary is occupied at `r_min > 0` queries as unknown.
- Two disconnected free boxes with an unknown gap. Each box center queries as free and is not stored. The gap queries as unknown.
- Free slab capped by occupied cells. The enclosed free cell queries as free via a non-zero distance to an interior-free boundary.
- Explicit unknown cells and negative indices on one column.
- Empty map, a single free cell, a single occupied cell, and unknown-only input (no boundaries).
- One adversarial column: occupied cells, free runs, unknown holes, and a lateral occupied neighbor.
- Fixed-seed random boxes (`8³`, `10³`, `14×12×8`, `6×16×12`), including seeds that write explicit unknown cells and seeds with a negative origin.
- `rebuild` of a new scene drops the previous one.

The random scenes are small and fixed. They are there so a regression fails in CI. They are not a sample of the paper's datasets.

## Build and test results

Local commands, from a clean configure, after the fix:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure
```

`/usr/bin/c++` is Clang 18.1.3. CMake 3.28.3. Result:

```text
1/1 Test #1: boundary_map .....................   Passed    0.01 sec

100% tests passed, 0 tests failed out of 1
```

The same Release configure with `g++` 13.3.0 (`-DCMAKE_CXX_COMPILER=g++`) also passed `boundary_map` in 0.01 s.

A Debug configure with `g++` and `-fsanitize=address,undefined -fno-omit-frame-pointer` passed `boundary_map` in 0.13 s. No ASan or UBSan report was printed.

Clang 18 ASan was not run: the link failed because `libclang_rt.asan-x86_64.a` (and the matching static/C++ runtime archives) are not installed in this environment. That is an environment gap, not a test failure.

Release and the g++ sanitizer build emitted no compiler diagnostics under `-Wall -Wextra -Wpedantic`.

GitHub Actions on `ubuntu-24.04` (`.github/workflows/ci.yml`) passed for commit `6bbf07d`: `g++` and `clang++`, each in Debug and Release, on both the push run and the pull-request run. The test step is `ctest --test-dir build --output-on-failure`. Status was read with `gh pr checks 1`. Pull-request run: https://github.com/wenquanzhao/bdm-reimplementation/actions/runs/42176201952. The job logs were not copied into this note; the claim is the check status, not a pasted CI transcript.

## Verdict

The v0.1 static core, after this fix, does what it claims for a **z-projected** boundary map:

- extraction follows the three boundary types in Section 4, including free/occupied faces;
- `query` along `z+` and `z-` follows Algorithm 1;
- on every scene in `tests/test_boundary_map.cpp`, both directions match the dense grid, and a free region actually produces interior-free entries.

That is the static representation and the static query. It is not a reproduction of the IJRR system. Still absent from the static core: the paper's hash and packed entry layout, projection axes other than `z`, any measured average-`O(1)` timing, ROS 2, and the Sections 8–9 experiments. A Section 7 skeleton is now separate from `rebuild`; what it covers and what it leaves out is in `docs/INCREMENTAL.md`. Nothing here should be read as matching the paper's memory, speed, or accuracy figures.

## KITTI

`docs/KITTI_PIPELINE.md` loads KITTI odometry scans into a dense binary grid and compares `rebuild` / `query` with that grid. The comparison checks this static map. It is not a Section 8 result, and that grid is not the Section 7 log-odds model. The log-odds path is `docs/INCREMENTAL.md`.

## Section 7 skeleton

`IncrementalMapper` is a later step than the static core checked above. It is not part of the historical run that produced the 1/1 `boundary_map` result. The local check for this step, from a clean configure, is:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER=g++
cmake --build build -j
ctest --test-dir build --output-on-failure
```

`g++` 13.3.0, CMake 3.28.3. Result: 8/8 tests passed (`boundary_map`, `kitti_grid`, `kitti_fixture`, `kitti_help`, `incremental_map`, `incremental_synthetic`, `incremental_help`, `incremental_kitti_mini`).

The same sources with `-Wall -Wextra -Wpedantic -Werror -fsanitize=address,undefined` also passed those 8 tests. No ASan or UBSan report was printed.

`incremental_map` streams synthetic scans, slides the window, and checks the overlap of `mergedBoundaryMap` against the local grid in both `z` directions. It also checks slid-out cells against a second mapper whose window never slides, including an enclosed free cell that is not stored as a boundary, and the equation (11) re-entry that replaces accumulated log-odds with the free or occupied threshold. `incremental_synthetic` requires a slide that actually inserts global boundaries. `incremental_kitti_mini` streams the two-frame fixture; at a 16³ window the slide-out slab is still prior-unknown, so the global map stays empty and the check is the overlap only.

This does not measure Section 8, and it does not claim the paper's memory or speed.
