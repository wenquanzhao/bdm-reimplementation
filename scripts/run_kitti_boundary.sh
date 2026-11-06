#!/usr/bin/env bash
# Run bdm_kitti on a local KITTI odometry drop. Exits 2 when the scans are absent.
# Override resolution and ranges with BDM_RESOLUTION, BDM_MIN_RANGE, BDM_MAX_RANGE.
# BDM_MAX_FRAMES, when set, is passed through. Otherwise every scan on disk is used.
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
DATA_ROOT="${1:-$ROOT_DIR/data/kitti/dataset}"
SEQUENCE="${2:-00}"
BIN="${BDM_KITTI_BIN:-$ROOT_DIR/build/bdm_kitti}"

if [[ ! -x "$BIN" ]]; then
  echo "bdm_kitti is not built at $BIN" >&2
  echo "From the repo root: cmake -S . -B build && cmake --build build -j" >&2
  exit 2
fi

if [[ ! -d "$DATA_ROOT/sequences/$SEQUENCE/velodyne" ]]; then
  echo "No velodyne directory at $DATA_ROOT/sequences/$SEQUENCE/velodyne" >&2
  echo "Fetch a short clip (poses, calib, and the first 200 frames of sequence 00):" >&2
  echo "  python3 scripts/fetch_kitti_odometry.py --root data/kitti --sequence 00 --max-frames 200" >&2
  echo "See docs/KITTI_PIPELINE.md" >&2
  exit 2
fi

args=(
  --root "$DATA_ROOT"
  --sequence "$SEQUENCE"
  --resolution "${BDM_RESOLUTION:-0.5}"
  --min-range "${BDM_MIN_RANGE:-1}"
  --max-range "${BDM_MAX_RANGE:-30}"
)
if [[ -n "${BDM_MAX_FRAMES:-}" ]]; then
  args+=(--max-frames "$BDM_MAX_FRAMES")
fi
exec "$BIN" "${args[@]}"
