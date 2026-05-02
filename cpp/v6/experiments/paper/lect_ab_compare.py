#!/usr/bin/env python3
"""
LECT storage A/B comparison:
  Parses every .lect file header and estimates byte savings from the
  dynamic n_channels optimization (IFK-sourced → 1 channel, full → 2).

Usage:
    python lect_ab_compare.py [--root DIR]

Output: per-file table + aggregated stats by (robot, scene_type).
"""
import struct
import os
import sys
import argparse
from pathlib import Path
from collections import defaultdict

MAGIC = b"SBF6LECT"
HEADER_SIZE = 128

# Header layout (from LectFileHeaderV5):
#  0: char[8]  magic
#  8: uint32   version
# 12: int32    n_nodes
# 16: int32    n_dims
# 20: int32    n_active_links
# 24: int32    ep_stride
# 28: int32    liaabb_stride
# 32: uint8    split_order
# 33: uint8    ep_source
# 34: uint8    env_type
# 35: uint8    n_channels
# 36: int32    capacity
# 40: uint64   robot_hash
# 48: uint8    grid_section
# 49: uint8    has_derived
# 50: uint8[2] reserved2
# 52: float    grid_delta
# 56: uint32   tree_record_bytes
# 60: uint32   reserved3
# 64: uint64   tree_section_off
# 72: uint64   ep_section_off
# 80: uint64   derived_off
# 88: uint64   trailer_off
# 96: uint64   grid_off
# 104-127: reserved

# 8+4+4+4+4+4+4+1+1+1+1+4+8+1+1+2+4+4+4+8+8+8+8+8+24 = 128
HDR_FMT = "<8sIiiiiibbbbiqbb2xfIIQQQQQ24s"

def parse_header(path: Path):
    import struct as _s
    sz = _s.calcsize(HDR_FMT)
    assert sz == 128, f"HDR_FMT gives {sz} bytes, expected 128"
    with open(path, "rb") as f:
        raw = f.read(HEADER_SIZE)
    if len(raw) < HEADER_SIZE:
        return None
    (magic, version, n_nodes, n_dims, n_active_links,
     ep_stride, liaabb_stride, split_order, ep_source, env_type, n_channels,
     capacity, robot_hash, grid_section, has_derived, grid_delta,
     tree_record_bytes, reserved3,
     tree_section_off, ep_section_off, derived_off, trailer_off, grid_off,
     _reserved) = struct.unpack(HDR_FMT, raw)
    if magic != MAGIC:
        return None
    return {
        "magic": magic,
        "version": version,
        "n_nodes": n_nodes,
        "n_dims": n_dims,
        "ep_stride": ep_stride,
        "ep_source": ep_source,
        "n_channels": n_channels,
        "capacity": capacity,
        "ep_section_off": ep_section_off,
        "derived_off": derived_off,
        "trailer_off": trailer_off,
    }


def ep_section_bytes(capacity: int, ep_stride: int, n_channels: int) -> int:
    ch = max(1, min(2, n_channels))
    return capacity * ep_stride * ch * 4  # sizeof(float) = 4


def classify(path: Path):
    """Return (robot, scene_type, is_ifk) from path components."""
    parts = path.parts
    # path pattern: .../lect_cache/<robot>_<difficulty>_<scene_id>/[ifk/]<name>.lect
    is_ifk = "ifk" in parts
    # find the lect_cache/<scene_dir> component
    scene_dir = ""
    for i, p in enumerate(parts):
        if p == "lect_cache" and i + 1 < len(parts):
            scene_dir = parts[i + 1]
            break
    if not scene_dir:
        # fallback: parent dir name (skip "ifk")
        for p in reversed(parts[:-1]):
            if p != "ifk":
                scene_dir = p
                break
    # split scene_dir into robot + difficulty
    tokens = scene_dir.rsplit("_", 2)  # robot_name_difficulty_id
    if len(tokens) >= 3:
        robot = "_".join(tokens[:-2])
        scene_type = tokens[-2]
    elif len(tokens) == 2:
        robot = tokens[0]
        scene_type = tokens[1]
    else:
        robot = scene_dir
        scene_type = "?"
    return robot, scene_type, is_ifk


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", default=str(
        Path(__file__).parent.parent / "results_paper" / "exp5_random_scenes" / "lect_cache"))
    args = parser.parse_args()

    root = Path(args.root)
    if not root.exists():
        print(f"[ERROR] root not found: {root}", file=sys.stderr)
        sys.exit(1)

    files = sorted(root.rglob("*.lect"))
    if not files:
        print(f"[ERROR] no .lect files under {root}", file=sys.stderr)
        sys.exit(1)

    print(f"Found {len(files)} .lect files under {root}\n")

    rows = []
    skipped = 0
    for p in files:
        hdr = parse_header(p)
        if hdr is None:
            skipped += 1
            continue
        robot, scene_type, is_ifk = classify(p)
        file_bytes = p.stat().st_size
        cap = hdr["capacity"]
        ep_stride = hdr["ep_stride"]
        n_ch_old = hdr["n_channels"]  # what's on disk now

        # EP section on disk
        ep_off = hdr["ep_section_off"]
        derived_off = hdr["derived_off"]
        ep_bytes_disk = int(derived_off) - int(ep_off) if derived_off > ep_off else 0

        # Theoretical new n_channels:
        # IFK files have no unsafe EP data → n_channels_new = 1
        # Full LECT files keep n_channels = 2 (conservative; unsafe may have data)
        n_ch_new = 1 if is_ifk else n_ch_old

        ep_bytes_old = ep_section_bytes(cap, ep_stride, n_ch_old)
        ep_bytes_new = ep_section_bytes(cap, ep_stride, n_ch_new)
        ep_saved = ep_bytes_old - ep_bytes_new
        # Adjust file size estimate
        file_new_est = file_bytes - ep_saved

        rows.append({
            "path": str(p.relative_to(root.parent.parent)),
            "robot": robot,
            "scene_type": scene_type,
            "is_ifk": is_ifk,
            "file_bytes": file_bytes,
            "ep_bytes_disk": ep_bytes_disk,
            "ep_bytes_old": ep_bytes_old,
            "ep_bytes_new": ep_bytes_new,
            "ep_saved": ep_saved,
            "file_new_est": file_new_est,
            "n_ch_old": n_ch_old,
            "n_ch_new": n_ch_new,
            "cap": cap,
            "ep_stride": ep_stride,
            "n_dims": hdr["n_dims"],
            "n_nodes": hdr["n_nodes"],
        })

    if skipped:
        print(f"[WARN] skipped {skipped} unreadable files\n")

    # ── print per-file table ─────────────────────────────────────────────
    print(f"{'Path':<60} {'FileKB':>7} {'EPdiskKB':>9} {'EPoldKB':>8} {'EPnewKB':>8} {'SavedKB':>8} {'Save%':>7} ch")
    print("-" * 120)
    for r in rows:
        pct = 100.0 * r["ep_saved"] / r["ep_bytes_old"] if r["ep_bytes_old"] > 0 else 0
        print(f"{r['path']:<60} {r['file_bytes']/1024:>7.1f} "
              f"{r['ep_bytes_disk']/1024:>9.1f} "
              f"{r['ep_bytes_old']/1024:>8.1f} "
              f"{r['ep_bytes_new']/1024:>8.1f} "
              f"{r['ep_saved']/1024:>8.1f} "
              f"{pct:>6.1f}% "
              f"{r['n_ch_old']}→{r['n_ch_new']}")

    # ── aggregate by (robot, scene_type, is_ifk) ────────────────────────
    agg = defaultdict(lambda: {"file_bytes": 0, "ep_bytes_old": 0, "ep_bytes_new": 0,
                                "ep_saved": 0, "file_new": 0, "count": 0})
    for r in rows:
        key = (r["robot"], r["scene_type"], "IFK" if r["is_ifk"] else "full")
        agg[key]["file_bytes"] += r["file_bytes"]
        agg[key]["ep_bytes_old"] += r["ep_bytes_old"]
        agg[key]["ep_bytes_new"] += r["ep_bytes_new"]
        agg[key]["ep_saved"] += r["ep_saved"]
        agg[key]["file_new"] += r["file_new_est"]
        agg[key]["count"] += 1

    print(f"\n{'Robot':<14} {'Type':<8} {'Src':<5} {'Files':>5} {'TotalMB':>8} {'EPoldMB':>8} {'EPnewMB':>8} {'SavedMB':>8} {'EP%':>6} {'File%':>6}")
    print("-" * 100)
    totals = {"file_bytes": 0, "ep_bytes_old": 0, "ep_bytes_new": 0,
              "ep_saved": 0, "file_new": 0, "count": 0}
    for key in sorted(agg):
        robot, scene_type, src = key
        v = agg[key]
        ep_pct = 100.0 * v["ep_saved"] / v["ep_bytes_old"] if v["ep_bytes_old"] > 0 else 0
        file_pct = 100.0 * (v["file_bytes"] - v["file_new"]) / v["file_bytes"] if v["file_bytes"] > 0 else 0
        print(f"{robot:<14} {scene_type:<8} {src:<5} {v['count']:>5} "
              f"{v['file_bytes']/1e6:>8.2f} "
              f"{v['ep_bytes_old']/1e6:>8.2f} {v['ep_bytes_new']/1e6:>8.2f} "
              f"{v['ep_saved']/1e6:>8.2f} {ep_pct:>5.1f}% {file_pct:>5.1f}%")
        for k in totals:
            if k != "count":
                totals[k] += v[k]
        totals["count"] += v["count"]

    print("=" * 100)
    ep_pct_tot = 100.0 * totals["ep_saved"] / totals["ep_bytes_old"] if totals["ep_bytes_old"] > 0 else 0
    file_pct_tot = 100.0 * (totals["file_bytes"] - totals["file_new"]) / totals["file_bytes"] if totals["file_bytes"] > 0 else 0
    print(f"{'TOTAL':<14} {'':<8} {'':<5} {totals['count']:>5} "
          f"{totals['file_bytes']/1e6:>8.2f} "
          f"{totals['ep_bytes_old']/1e6:>8.2f} {totals['ep_bytes_new']/1e6:>8.2f} "
          f"{totals['ep_saved']/1e6:>8.2f} {ep_pct_tot:>5.1f}% {file_pct_tot:>5.1f}%")

    # ── summary ─────────────────────────────────────────────────────────
    print(f"\n=== SUMMARY ===")
    print(f"Total files analyzed : {totals['count']}")
    print(f"Total disk (old)     : {totals['file_bytes']/1e6:.2f} MB")
    print(f"Total disk (est new) : {totals['file_new']/1e6:.2f} MB")
    print(f"EP section saved     : {totals['ep_saved']/1e6:.2f} MB  ({ep_pct_tot:.1f}% of EP)")
    print(f"Overall file saved   : {(totals['file_bytes']-totals['file_new'])/1e6:.2f} MB  ({file_pct_tot:.1f}% of total)")

    # IFK-only sub-total
    ifk_rows = [r for r in rows if r["is_ifk"]]
    if ifk_rows:
        ifk_file = sum(r["file_bytes"] for r in ifk_rows)
        ifk_ep_old = sum(r["ep_bytes_old"] for r in ifk_rows)
        ifk_saved = sum(r["ep_saved"] for r in ifk_rows)
        print(f"\nIFK-only ({len(ifk_rows)} files):")
        print(f"  EP saved           : {ifk_saved/1e6:.2f} MB  ({100*ifk_saved/ifk_ep_old:.1f}% of IFK EP)")
        print(f"  File size reduction: {ifk_saved/1e6:.2f} MB  ({100*ifk_saved/ifk_file:.1f}% of IFK total)")


if __name__ == "__main__":
    main()
