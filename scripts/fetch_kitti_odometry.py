#!/usr/bin/env python3
"""Download a short KITTI odometry clip without the full velodyne archive.

Poses and calibration are small zip files. Velodyne scans live in one ~79 GiB
zip of stored (uncompressed) entries. This script reads that zip's central
directory with an HTTP Range request and downloads only the requested frames.

The data is written under --root and is not committed. See docs/KITTI_PIPELINE.md.
Sequences 00-10 have public ground-truth poses. 11-21 do not.
"""

from __future__ import annotations

import argparse
import math
import struct
import sys
import time
import urllib.request
import zipfile
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path

POSES_URL = "https://s3.eu-central-1.amazonaws.com/avg-kitti/data_odometry_poses.zip"
CALIB_URL = "https://s3.eu-central-1.amazonaws.com/avg-kitti/data_odometry_calib.zip"
VELODYNE_URL = "https://s3.eu-central-1.amazonaws.com/avg-kitti/data_odometry_velodyne.zip"
USER_AGENT = "bdm-kitti-fetch/0.1"


class ZipEntry:
    def __init__(self, name: str, method: int, comp_size: int, uncomp_size: int, local_off: int):
        self.name = name
        self.method = method
        self.comp_size = comp_size
        self.uncomp_size = uncomp_size
        self.local_off = local_off


def request(url: str, start: int | None = None, end: int | None = None, timeout: int = 180):
    headers = {"User-Agent": USER_AGENT}
    if start is not None:
        headers["Range"] = f"bytes={start}-{end}"
    last_error: Exception | None = None
    for attempt in range(4):
        try:
            req = urllib.request.Request(url, headers=headers)
            with urllib.request.urlopen(req, timeout=timeout) as resp:
                if start is not None and resp.status != 206:
                    raise RuntimeError(f"expected HTTP 206 for a range, got {resp.status}")
                data = resp.read()
            if start is not None and len(data) != (end - start + 1):
                raise RuntimeError(
                    f"short range read at {start}: got {len(data)} bytes, "
                    f"expected {end - start + 1}"
                )
            return data
        except Exception as exc:  # noqa: BLE001 - retry network failures
            last_error = exc
            time.sleep(1.5 * (attempt + 1))
    raise RuntimeError(f"GET {url} bytes={start}-{end} failed: {last_error}")


def content_length(url: str) -> int:
    req = urllib.request.Request(url, method="HEAD", headers={"User-Agent": USER_AGENT})
    with urllib.request.urlopen(req, timeout=60) as resp:
        return int(resp.headers["Content-Length"])


def download_file(url: str, dest: Path) -> None:
    dest.parent.mkdir(parents=True, exist_ok=True)
    if dest.exists() and dest.stat().st_size > 0:
        print(f"already present: {dest}")
        return
    print(f"downloading {url}")
    tmp = dest.with_suffix(dest.suffix + ".part")
    last_error: Exception | None = None
    for attempt in range(4):
        try:
            urllib.request.urlretrieve(url, tmp)
            tmp.replace(dest)
            return
        except Exception as exc:  # noqa: BLE001
            last_error = exc
            time.sleep(2 * (attempt + 1))
    raise RuntimeError(f"download failed for {url}: {last_error}")


def extract_zip(path: Path, root: Path) -> None:
    with zipfile.ZipFile(path) as archive:
        archive.extractall(root)


def apply_zip64(extra: bytes, comp: int, uncomp: int, local_off: int, disk: int):
    pos = 0
    while pos + 4 <= len(extra):
        header_id, header_size = struct.unpack_from("<HH", extra, pos)
        pos += 4
        block = extra[pos : pos + header_size]
        pos += header_size
        if header_id != 0x0001:
            continue
        cursor = 0
        if uncomp == 0xFFFFFFFF:
            uncomp = struct.unpack_from("<Q", block, cursor)[0]
            cursor += 8
        if comp == 0xFFFFFFFF:
            comp = struct.unpack_from("<Q", block, cursor)[0]
            cursor += 8
        if local_off == 0xFFFFFFFF:
            local_off = struct.unpack_from("<Q", block, cursor)[0]
            cursor += 8
        if disk == 0xFFFF and cursor + 4 <= len(block):
            disk = struct.unpack_from("<I", block, cursor)[0]
        break
    return comp, uncomp, local_off, disk


def parse_central_directory(data: bytes) -> list[ZipEntry]:
    entries: list[ZipEntry] = []
    offset = 0
    while offset + 46 <= len(data) and data[offset : offset + 4] == b"PK\x01\x02":
        (
            _ver_made,
            _ver_need,
            _flag,
            method,
            _time,
            _date,
            _crc,
            comp,
            uncomp,
            name_len,
            extra_len,
            comment_len,
            disk,
            _iattr,
            _eattr,
            local_off,
        ) = struct.unpack_from("<HHHHHHIIIHHHHHII", data, offset + 4)
        name_start = offset + 46
        name = data[name_start : name_start + name_len].decode("utf-8").replace("\\", "/")
        extra = data[name_start + name_len : name_start + name_len + extra_len]
        comp, uncomp, local_off, _disk = apply_zip64(extra, comp, uncomp, local_off, disk)
        if local_off == 0xFFFFFFFF or comp == 0xFFFFFFFF:
            raise RuntimeError(f"ZIP64 extra field missing for {name}")
        entries.append(ZipEntry(name, method, comp, uncomp, local_off))
        offset = name_start + name_len + extra_len + comment_len
    if not entries:
        raise RuntimeError("central directory contained no entries")
    return entries


def zip64_central_directory(url: str, file_size: int) -> tuple[int, int, bytes]:
    tail_len = min(file_size, 256 * 1024)
    tail_start = file_size - tail_len
    tail = request(url, tail_start, file_size - 1)
    eocd_rel = tail.rfind(b"PK\x05\x06")
    locator_rel = tail.rfind(b"PK\x06\x07")
    if eocd_rel < 0 or locator_rel < 0:
        raise RuntimeError("velodyne zip has no ZIP64 end-of-central-directory record")
    eocd64_off = struct.unpack_from("<Q", tail, locator_rel + 8)[0]
    if tail_start <= eocd64_off <= file_size - 56:
        eocd64 = tail[eocd64_off - tail_start : eocd64_off - tail_start + 56]
    else:
        eocd64 = request(url, eocd64_off, eocd64_off + 55)
    if eocd64[:4] != b"PK\x06\x06":
        raise RuntimeError("ZIP64 end record signature mismatch")
    _size_rec, _ver_made, _ver_need, _disk, _cd_disk, _n_disk, _n_total, cd_size, cd_off = (
        struct.unpack_from("<QHHIIQQQQ", eocd64, 4)
    )
    print(f"velodyne zip size {file_size} bytes, central directory {cd_size} bytes at {cd_off}")
    return cd_off, cd_size, request(url, cd_off, cd_off + cd_size - 1)


def load_central_directory(url: str, cache_dir: Path) -> list[ZipEntry]:
    file_size = content_length(url)
    cache_dir.mkdir(parents=True, exist_ok=True)
    meta_path = cache_dir / "velodyne_cd.meta"
    data_path = cache_dir / "velodyne_cd.bin"
    if meta_path.exists() and data_path.exists():
        size_text, off_text, len_text = meta_path.read_text().split()
        if int(size_text) == file_size:
            print(f"using cached central directory {data_path}")
            return parse_central_directory(data_path.read_bytes())
    cd_off, cd_size, blob = zip64_central_directory(url, file_size)
    data_path.write_bytes(blob)
    meta_path.write_text(f"{file_size} {cd_off} {cd_size}\n")
    return parse_central_directory(blob)


def extract_stored(url: str, entry: ZipEntry, dest: Path) -> None:
    if entry.method != 0:
        raise RuntimeError(f"{entry.name} is compressed (method {entry.method}); expected stored")
    if dest.exists() and dest.stat().st_size == entry.comp_size:
        return
    slack = 256
    # One range covers the local header plus the stored payload when the
    # local extra field is at most `slack` bytes (KITTI uses a short ZIP64 extra).
    guess = 30 + len(entry.name.encode("utf-8")) + slack + entry.comp_size
    blob = request(url, entry.local_off, entry.local_off + guess - 1)
    if blob[:4] != b"PK\x03\x04":
        raise RuntimeError(f"missing local header for {entry.name}")
    name_len, extra_len = struct.unpack_from("<HH", blob, 26)
    data_start = 30 + name_len + extra_len
    need = data_start + entry.comp_size
    if len(blob) < need:
        blob += request(url, entry.local_off + len(blob), entry.local_off + need - 1)
    payload = blob[data_start : data_start + entry.comp_size]
    if len(payload) != entry.comp_size:
        raise RuntimeError(f"truncated scan {entry.name}")
    if len(payload) % 16 != 0:
        raise RuntimeError(f"{entry.name} payload is not a multiple of 16 bytes")
    dest.parent.mkdir(parents=True, exist_ok=True)
    tmp = dest.with_suffix(dest.suffix + ".part")
    tmp.write_bytes(payload)
    tmp.replace(dest)


def sanity_check_scan(path: Path) -> None:
    data = path.read_bytes()
    count = len(data) // 16
    if count < 20000 or count > 300000:
        raise RuntimeError(f"{path} has {count} points, which is outside a normal KITTI scan")
    worst = 0.0
    for index in range(0, count, max(1, count // 32)):
        x, y, z, refl = struct.unpack_from("<ffff", data, index * 16)
        if not all(math.isfinite(v) for v in (x, y, z, refl)):
            raise RuntimeError(f"non-finite point in {path}")
        worst = max(worst, abs(x), abs(y), abs(z))
    if worst > 500:
        raise RuntimeError(f"implausible coordinate {worst} in {path}")
    print(f"scan {path.name}: {count} points, max |xyz| {worst:.2f} m")


def normalize_sequence(text: str) -> str:
    if len(text) == 1 and text.isdigit():
        text = "0" + text
    if len(text) != 2 or not text.isdigit():
        raise SystemExit(f"sequence must be two digits, got {text!r}")
    return text


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path("data/kitti"))
    parser.add_argument("--sequence", action="append", default=[], help="repeatable, default 00")
    parser.add_argument("--start-frame", type=int, default=0)
    parser.add_argument("--max-frames", type=int, default=200, help="frames per sequence, [start, start+max)")
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--skip-poses", action="store_true")
    parser.add_argument("--skip-velodyne", action="store_true")
    parser.add_argument("--allow-large", action="store_true", help="allow --max-frames above 2000")
    parser.add_argument("--dry-run", action="store_true", help="resolve entries, do not download scans")
    args = parser.parse_args()
    if args.start_frame < 0 or args.max_frames < 1:
        raise SystemExit("--start-frame must be >= 0 and --max-frames must be >= 1")
    if args.max_frames > 2000 and not args.allow_large:
        raise SystemExit("refusing more than 2000 frames without --allow-large (about 2 MB each)")
    sequences = [normalize_sequence(s) for s in (args.sequence or ["00"])]
    root: Path = args.root

    if not args.skip_poses:
        archive_dir = root / ".cache"
        poses_zip = archive_dir / "data_odometry_poses.zip"
        calib_zip = archive_dir / "data_odometry_calib.zip"
        download_file(POSES_URL, poses_zip)
        download_file(CALIB_URL, calib_zip)
        extract_zip(poses_zip, root)
        extract_zip(calib_zip, root)
        print(f"poses and calib extracted under {root / 'dataset'}")

    if args.skip_velodyne:
        return 0

    entries = load_central_directory(VELODYNE_URL, root / ".cache")
    by_name = {entry.name: entry for entry in entries}
    jobs: list[tuple[ZipEntry, Path]] = []
    for sequence in sequences:
        if int(sequence) > 10:
            print(
                f"warning: public odometry poses cover 00-10 only; sequence {sequence} "
                "will not have poses/<seq>.txt",
                file=sys.stderr,
            )
        for index in range(args.start_frame, args.start_frame + args.max_frames):
            name = f"dataset/sequences/{sequence}/velodyne/{index:06d}.bin"
            entry = by_name.get(name)
            if entry is None:
                raise SystemExit(f"frame not in the velodyne zip: {name}")
            jobs.append((entry, root / name))
    total_bytes = sum(entry.comp_size for entry, _path in jobs)
    print(f"selected {len(jobs)} scans, {total_bytes / 1e6:.1f} MB stored")
    if args.dry_run:
        first, first_path = jobs[0]
        last, last_path = jobs[-1]
        print(f"first {first_path} local_off={first.local_off} bytes={first.comp_size}")
        print(f"last  {last_path} local_off={last.local_off} bytes={last.comp_size}")
        return 0

    done = 0
    with ThreadPoolExecutor(max_workers=max(1, args.workers)) as pool:
        futures = [pool.submit(extract_stored, VELODYNE_URL, entry, dest) for entry, dest in jobs]
        for future in as_completed(futures):
            future.result()
            done += 1
            if done == len(jobs) or done % 10 == 0:
                print(f"downloaded {done}/{len(jobs)}")
    sanity_check_scan(jobs[0][1])
    dataset = root / "dataset"
    print("dataset root:", dataset)
    print("example:")
    print(
        "  ./build/bdm_kitti --root",
        dataset,
        "--sequence",
        sequences[0],
        "--max-frames",
        args.max_frames,
        "--resolution 0.5 --min-range 1 --max-range 30",
    )
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:  # noqa: BLE001
        print(f"error: {exc}", file=sys.stderr)
        sys.exit(1)
