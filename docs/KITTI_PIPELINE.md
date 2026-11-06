# KITTI odometry to a dense grid, then BoundaryMap

This is the first dataset hook in front of the static boundary map. It loads public KITTI odometry LiDAR and poses, paints a dense Free / Occupied / Unknown grid, calls `BoundaryMap::rebuild`, and checks `query` along `z+` and `z-` against that grid.

It is not the paper's Section 7 log-odds mapper (no odds, no clamping, no sliding window, no incremental boundary update). It is not a Section 8 experiment: there is no OctoMap comparison, no memory or timing table from the paper, and no accuracy claim against those figures. A match here means the static map agrees with the grid it was just built from.

The KITTI scans are not in this repository. CI runs a synthetic fixture in the same file layout. A short real clip is downloaded separately.

## Pipeline

1. Read `sequences/<seq>/calib.txt` and take the `Tr` row (velodyne to camera 0). `Tr_velo_to_cam` is accepted as an alias.
2. Read `poses/<seq>.txt` (or `sequences/<seq>/poses.txt`). Line `i` is the 3×4 world-from-camera-0 pose of frame `i`.
3. Read `sequences/<seq>/velodyne/NNNNNN.bin`. Each record is four little-endian float32 values: `x, y, z, reflectance`. Reflectance is ignored. The host must be little-endian. A file larger than 64 MiB, or whose size is not a multiple of 16 bytes, is rejected.
4. Transform each point with the KITTI devkit product. With `pose` the world-from-camera matrix and `Tr` the camera-from-velodyne matrix:

   `p_world = pose * Tr * p_velo`

   The sensor origin is that same product applied to `(0,0,0)`.
5. Keep returns whose range in the sensor frame is inside `[min_range, max_range]`. Other returns are ignored. They are not traced out to the range limit.
6. Ray-cast from the sensor voxel to the hit voxel (Amanatides and Woo). The hit voxel is Occupied. Every other voxel the ray crosses is Free unless it is already Occupied. Occupied is sticky for the rest of the run.
7. `BoundaryMap::rebuild` on that grid. Missing keys are Unknown, which is the same rule the map already uses.
8. Query the grid back. A voxel matches only when both `z+` and `z-` equal the dense state.

World axes are the published odometry frame, not a gravity-aligned velodyne frame: camera 0 at frame 0, x right, y down, z forward. `BoundaryMap` projects along that voxel z. Renaming axes is not part of this step, so a column is not a vertical column in the vehicle frame.

## Parameters

These are demo knobs. None of them is a Section 7 setting.

| Knob | Flag | Default | Effect |
| --- | --- | --- | --- |
| Voxel size | `--resolution` | `0.5` meters | `floor(world / resolution)` |
| Minimum range | `--min-range` | `1` meter | Drop closer returns (ego-vehicle filter is only this threshold) |
| Maximum range | `--max-range` | `30` meters | Drop farther returns, with no free-space ray in their place |
| Point stride | `--point-stride` | `1` | Keep points `0, N, 2N, ...` in file order. A speed knob, not a sensor model |
| Frame stride | `--stride` | `1` | Keep frame indices divisible by N |
| Frame cap | `--max-frames` | `0` | `0` uses every selected scan on disk |
| Query padding | `--padding` | `1` | Also query this many cells outside the occupied/free bounding box |
| Window cap | `--max-window-voxels` | `20000000` | Above this volume, compare stored cells and boundary entries only |

Hit policy: the voxel that contains an in-range return is Occupied.

Miss policy: other voxels crossed by that ray are Free. A later miss does not clear an earlier hit.

Out of range, non-finite points, and a zero-length return inside the sensor voxel: out-of-range and non-finite points are skipped. A return that falls in the sensor voxel is Occupied and does not paint a free ray.

When a ray crosses two voxel faces at the same time, the stepper advances x, then y, then z. A cell only touched at a corner may be skipped. The consistency check follows whatever the grid stored.

## What the report means

`bdm_kitti` prints one text report on stdout. Progress is on stderr. The fields that decide success:

- `queries`, `matched`, `mismatched`, `match_rate`
- `compare_scope`: `bounding_window` (every voxel in the padded box) or `stored_and_boundary` (the window was larger than `--max-window-voxels`)
- `mismatch:` lines, up to `--max-examples`, each with the dense state and both search directions

`match_rate` is `matched / queries`. It is printed as `1` when every query matched or when nothing was queried. Exit status is `0` when `mismatched` is 0, `1` when any query disagreed, and `2` on usage or I/O errors.

`free_voxels` and `occupied_voxels` count stored grid cells. Unknown cells are not stored. `boundary_interior_free`, `boundary_exterior_unknown`, and `boundary_occupied` are the Section 4 sets this repository already builds. They are counts, not a paper metric.

## Layout

```text
<root>/
  poses/00.txt
  sequences/00/calib.txt
  sequences/00/velodyne/000000.bin
```

`<root>` is the directory that contains `sequences/` and `poses/`. After the official zip files are unpacked, that directory is `dataset/`. The synthetic fixture uses the same shape at `tests/fixtures/kitti_mini`.

Public ground-truth poses exist for sequences `00` through `10` only. Sequences `11`–`21` have velodyne scans and no public pose file, so this tool cannot place them.

## Download

The pose zip is about 1.3 MB and the calibration zip is about 0.6 MB. The velodyne zip is one object (84,786,535,790 bytes, about 79 GiB, when this was checked on 2026-11-06). Its entries are stored uncompressed, so a short clip can be fetched with HTTP Range requests. The script reads the current `Content-Length`; it does not depend on that size staying the same.

From the repository root, with Python 3 and no extra packages:

```bash
python3 scripts/fetch_kitti_odometry.py --root data/kitti --sequence 00 --max-frames 200
```

That writes:

```text
data/kitti/dataset/poses/00.txt
data/kitti/dataset/sequences/00/calib.txt
data/kitti/dataset/sequences/00/velodyne/000000.bin
...
data/kitti/dataset/sequences/00/velodyne/000199.bin
```

`data/` is gitignored. Frames are `[--start-frame, --start-frame + --max-frames)`. The default start is 0 and the default length is 200. More than 2000 frames requires `--allow-large` (each scan is about 2 MB). `--sequence` can be repeated. `--dry-run` resolves zip entries and does not download scans. Sequence `02` is the same command with `--sequence 02`.

Official objects, also linked from the [KITTI odometry page](http://www.cvlibs.net/datasets/kitti/eval_odometry.php):

- http://www.cvlibs.net/datasets/kitti/eval_odometry.php
- https://s3.eu-central-1.amazonaws.com/avg-kitti/data_odometry_poses.zip
- https://s3.eu-central-1.amazonaws.com/avg-kitti/data_odometry_calib.zip
- https://s3.eu-central-1.amazonaws.com/avg-kitti/data_odometry_velodyne.zip

To unpack the full archives instead of a clip:

```bash
mkdir -p data/kitti
curl -L -o /tmp/data_odometry_poses.zip https://s3.eu-central-1.amazonaws.com/avg-kitti/data_odometry_poses.zip
curl -L -o /tmp/data_odometry_calib.zip https://s3.eu-central-1.amazonaws.com/avg-kitti/data_odometry_calib.zip
curl -L -o /tmp/data_odometry_velodyne.zip https://s3.eu-central-1.amazonaws.com/avg-kitti/data_odometry_velodyne.zip
unzip /tmp/data_odometry_poses.zip -d data/kitti
unzip /tmp/data_odometry_calib.zip -d data/kitti
unzip /tmp/data_odometry_velodyne.zip -d data/kitti
```

The velodyne unzip is the ~79 GiB path. The dataset is distributed by the KITTI authors for research use; this repository does not redistribute it. Cite Geiger, Lenz, and Urtasun, “Are we ready for Autonomous Driving? The KITTI Vision Benchmark Suite,” CVPR 2012.

If the fetch script cannot reach S3, create the layout above by hand and point `--root` at it. The C++ tool only reads the files.

## Run

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
./build/bdm_kitti \
  --root data/kitti/dataset \
  --sequence 00 \
  --max-frames 200 \
  --resolution 0.5 \
  --min-range 1 \
  --max-range 30
```

`scripts/run_kitti_boundary.sh [root] [sequence]` calls that binary. The default root is `data/kitti/dataset` and the default sequence is `00`. It exits 2 with the fetch command when the velodyne directory is missing. `BDM_RESOLUTION`, `BDM_MIN_RANGE`, `BDM_MAX_RANGE`, and `BDM_MAX_FRAMES` override the defaults. Without `BDM_MAX_FRAMES` the script uses every scan in the directory, so set the cap when a full sequence is on disk.

A full sequence at 0.5 m is a large hash map. For a first look, keep `--max-frames` in the low hundreds, or raise `--resolution` to `1` and `--point-stride` to `2` or `4`. Say which knobs you used when you quote `match_rate`. Those knobs change the grid. They are not comparable to the paper.

## Fixture and CI

`tests/fixtures/kitti_mini` is a two-frame synthetic sequence. `Tr` is the identity. Poses put the sensor at `(0.5, 0.5, 0.5)` and then `(2.5, 0.5, 0.5)` meters. Frame `000000.bin` contains sensor-frame points `(5,0,0)`, `(0,4,0)`, `(0,0,3)`, `(3,2,1)`. Frame `000001.bin` contains `(4,0,0)`, `(0,3,1)`. Reflectance in the files is `1` and is ignored.

At `--resolution 1 --min-range 0.1 --max-range 30` the report is:

```text
scope: dense-grid consistency; not section 7 log-odds; not section 8 metrics
sequence: 00
scans_on_disk: 2
frames_integrated: 2
first_frame: 0
last_frame: 1
points_read: 6
points_integrated: 6
points_rejected_range: 0
points_rejected_nonfinite: 0
resolution_m: 1
min_range_m: 0.1
max_range_m: 30
point_stride: 1
hit_policy: in-range endpoint voxel is occupied
miss_policy: other voxels crossed by that ray are free unless already occupied
range_policy: points nearer than min_range or farther than max_range are ignored
occupied_sticky: true
log_odds: false
world_frame: kitti odometry camera 0 at frame 0 (x right, y down, z forward)
world_min_m: 0.500 0.500 0.500
world_max_m: 6.500 4.500 3.500
free_voxels: 15
occupied_voxels: 6
boundary_entries: 65
boundary_interior_free: 15
boundary_exterior_unknown: 44
boundary_occupied: 6
compare_scope: bounding_window
window_voxels: 378
window_skipped: false
queries: 378
matched: 378
mismatched: 0
match_rate: 1
```

CTest runs three checks that do not download anything:

- `kitti_grid` (`tests/test_kitti_grid.cpp`): hit/miss voxels on axis-aligned, diagonal, and negative rays; sticky occupied; range limits; `pose * Tr` order; bin and calib parsing; the fixture at 1 m and 0.5 m.
- `kitti_fixture`: `bdm_kitti` on `tests/fixtures/kitti_mini`. The test fails unless the report contains `mismatched: 0`.
- `kitti_help`: the usage text states that this is not a Section 8 result.

`boundary_map` is unchanged in what it asserts. It still does not need KITTI.

## Local clip

This was run on the machine that added the pipeline, on 2026-11-06. It is not a Section 8 result, and the scans are not committed.

`scripts/fetch_kitti_odometry.py --root data/kitti --sequence 00 --max-frames 200` wrote poses, calibration, and frames `000000.bin` through `000199.bin` (about 388 MB of velodyne). The pose file for sequence 00 has 4541 lines; only the first 200 were integrated. Release `bdm_kitti`:

```bash
./build/bdm_kitti \
  --root data/kitti/dataset \
  --sequence 00 \
  --max-frames 200 \
  --resolution 0.5 \
  --min-range 1 \
  --max-range 30
```

The run took about 17 seconds. 24,228,694 points were read and 23,071,100 were integrated (1,157,594 rejected by range). The grid held 105,399 free voxels and 68,250 occupied voxels. `BoundaryMap` stored 195,154 boundary entries (71,995 interior free, 54,909 exterior unknown, 68,250 occupied). The compare used the bounding window (2,966,432 voxels): 2,966,432 matched, 0 mismatched, `match_rate: 1`.

Axis-aligned bounds of the sensor origins and the integrated hits, in the odometry camera frame, were x [-35.128, 82.452], y [-7.509, 11.351], z [-30.187, 120.434] meters. A separate two-frame run of the same sequence at the same resolution also matched (377,580 / 377,580).

Sequence 02, frames 0–39, at the same resolution and ranges, took about 5 seconds. The bounding window had 880,276 queries, 0 mismatches, and `match_rate: 1`.

The same four CTest targets passed here under g++ with `-Wall -Wextra -Wpedantic -Werror`, under that g++ plus AddressSanitizer and UndefinedBehaviorSanitizer, and under clang++ 18 with the same warnings. That is the local check of this step. It does not measure the paper.

## Limits

- Binary hits, not log-odds. One stray return occupies a voxel for the rest of the clip.
- No ray is cast for a point outside `[min_range, max_range]`.
- The map is rebuilt from scratch after the clip. There is no streaming update.
- Search directions are only `z+` and `z-` in the odometry camera frame.
- `match_rate` is agreement with this grid, not localization accuracy, not mesh quality, and not a number from the paper.
