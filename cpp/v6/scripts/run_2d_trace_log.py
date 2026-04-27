#!/usr/bin/env python3
"""Run exp_2d_trace with replay-consistent config and auto-generate replay media.

Fixed consistency contract:
- GrowerMode: RRT (enforced by exp_2d_trace)
- EndpointSource: IFK or CritSample
- EnvelopeType: LinkIAABB / LinkIAABB_Grid / Hull16_Grid

Auto-generates MP4 video from log by default, which supports timeline scrubbing.
"""

import argparse
import json
import os
import re
import shlex
import subprocess
import sys
import threading
import time
from pathlib import Path


def _pick_binary(user_path: str | None) -> Path:
    candidates = []
    if user_path:
        candidates.append(Path(user_path))
    repo_root = Path(__file__).resolve().parents[1]
    candidates.extend([
        repo_root / "build" / "experiments" / "exp_2d_trace",
        repo_root.parent / "build" / "experiments" / "exp_2d_trace",
    ])
    for p in candidates:
        if p.exists() and os.access(p, os.X_OK):
            return p
    raise FileNotFoundError(
        "Cannot find executable exp_2d_trace. Build it first and/or pass --binary. "
        f"Tried: {[str(x) for x in candidates]}"
    )


def _load_replay_viewer(log_file: Path):
    """Load replay parser/viewer for media export."""
    import matplotlib
    matplotlib.use('Agg')

    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from viz_2d_build_replay import LogParser, Viewer2D

    print(f"\n[MEDIA] Loading log: {log_file}")
    try:
        parser = LogParser(str(log_file))
    except Exception as e:
        print(f"[MEDIA] ERROR parsing log: {e}", file=sys.stderr)
        return None, None

    print(f"[MEDIA] Frames: {len(parser.frames)}")

    viewer = Viewer2D(parser)
    viewer.setup_figure()
    return parser, viewer


def _pad_frame_for_video(frame, block_size: int = 16):
    """Pad frame to codec-friendly dimensions without rescaling content."""
    import numpy as np

    height, width = frame.shape[:2]
    padded_width = ((width + block_size - 1) // block_size) * block_size
    padded_height = ((height + block_size - 1) // block_size) * block_size
    if padded_width == width and padded_height == height:
        return frame, False

    padded = np.zeros((padded_height, padded_width, frame.shape[2]), dtype=frame.dtype)
    padded[:height, :width] = frame
    return padded, True


def _generate_mp4(log_file: Path, fps: int = 5) -> Path:
    """Generate MP4 video from log file at specified FPS."""
    import imageio.v2 as imageio
    import matplotlib.pyplot as plt
    import numpy as np

    parser, viewer = _load_replay_viewer(log_file)
    if parser is None or viewer is None:
        return None

    video_file = log_file.parent / f"{log_file.stem}.mp4"
    print(f"[VIDEO] Writing MP4: {video_file}")

    try:
        warned_about_padding = False
        with imageio.get_writer(
            str(video_file),
            fps=fps,
            codec="libx264",
            format="FFMPEG",
            pixelformat="yuv420p",
            ffmpeg_log_level="error",
            macro_block_size=None,
        ) as writer:
            for frame_idx in range(len(parser.frames)):
                viewer.state.current_frame_idx = frame_idx
                viewer._update_display()
                viewer.fig.canvas.draw()

                width, height = viewer.fig.canvas.get_width_height()
                frame = np.frombuffer(
                    viewer.fig.canvas.buffer_rgba(),
                    dtype=np.uint8,
                ).reshape(height, width, 4)[..., :3].copy()
                frame, was_padded = _pad_frame_for_video(frame)
                if was_padded and not warned_about_padding:
                    print(
                        f"[VIDEO] Padding frames from {width}x{height} to {frame.shape[1]}x{frame.shape[0]} for codec compatibility"
                    )
                    warned_about_padding = True
                writer.append_data(frame)

                if frame_idx == 0 or (frame_idx + 1) % 200 == 0 or frame_idx + 1 == len(parser.frames):
                    print(f"[VIDEO] Encoded {frame_idx + 1}/{len(parser.frames)} frames")

        print(f"[VIDEO] ✓ MP4 saved: {video_file}")
        return video_file
    except Exception as e:
        print(f"[VIDEO] ERROR writing MP4: {e}", file=sys.stderr)
        return None
    finally:
        plt.close(viewer.fig)


def _generate_gif(log_file: Path, fps: int = 5) -> Path:
    """Generate GIF from log file at specified FPS."""
    import matplotlib.pyplot as plt
    import matplotlib.animation as animation

    parser, viewer = _load_replay_viewer(log_file)
    if parser is None or viewer is None:
        return None

    def update_frame(frame_idx):
        viewer.state.current_frame_idx = frame_idx
        viewer._update_display()
        return list(viewer.fig.get_axes()[0].get_children())

    print(f"[GIF] Creating animation ({len(parser.frames)} frames, {fps} FPS)...")
    anim = animation.FuncAnimation(
        viewer.fig,
        update_frame,
        frames=len(parser.frames),
        interval=1000 // max(fps, 1),
        repeat=False,
        blit=False,
    )

    gif_file = log_file.parent / f"{log_file.stem}.gif"
    print(f"[GIF] Writing GIF: {gif_file}")
    try:
        anim.save(str(gif_file), writer='pillow', fps=fps, dpi=100)
        print(f"[GIF] ✓ GIF saved: {gif_file}")
        return gif_file
    except Exception as e:
        print(f"[GIF] ERROR writing GIF: {e}", file=sys.stderr)
        return None
    finally:
        plt.close(viewer.fig)

_FFB_SUCCESS_RE = re.compile(r"\[FFB\].*FREE -> RETURN", re.IGNORECASE)
_FFB_FAIL_RE = re.compile(r"\[FFB\].*(-> FAIL|reject\()", re.IGNORECASE)
_GRW_FINAL_RE = re.compile(
    r"\[GRW\] final: boxes=(\d+) success=(\d+) fail=(\d+) promo=(\d+)",
    re.IGNORECASE,
)
_DONE_RE = re.compile(
    r"\[2D-DONE\]\s+elapsed=([0-9.]+)s\s+boxes=(\d+)\s+edges=(\d+)\s+islands=(\d+)",
    re.IGNORECASE,
)


class _FFBGuardState:
    """Thread-safe accumulator for FFB outcomes scanned from the SBF log file."""

    def __init__(self, fail_limit: int):
        self.fail_limit = fail_limit
        self.consecutive = 0
        self.tripped = False
        self.lock = threading.Lock()

    def feed(self, line: str) -> bool:
        if _FFB_SUCCESS_RE.search(line):
            with self.lock:
                self.consecutive = 0
            return False
        if _FFB_FAIL_RE.search(line):
            with self.lock:
                self.consecutive += 1
                cur = self.consecutive
                if cur >= self.fail_limit and not self.tripped:
                    self.tripped = True
                    return True
            print(
                f"[FFB] Consecutive failures: {cur}/{self.fail_limit}",
                file=sys.stderr,
            )
        return False


def _tail_log_for_ffb(
    log_path: Path,
    state: _FFBGuardState,
    stop_event: threading.Event,
    on_trip,
    poll_interval: float = 0.05,
) -> None:
    """Tail an SBF log file as it is written and feed FFB lines into the guard."""
    while not log_path.exists() and not stop_event.is_set():
        time.sleep(poll_interval)
    if stop_event.is_set():
        return
    try:
        f = open(log_path, "r", errors="replace")
    except OSError as e:
        print(f"[FFB] Failed to open log for tailing: {e}", file=sys.stderr)
        return
    try:
        buf = ""
        while not stop_event.is_set():
            chunk = f.read()
            if not chunk:
                time.sleep(poll_interval)
                continue
            buf += chunk
            while "\n" in buf:
                line, buf = buf.split("\n", 1)
                if state.feed(line):
                    on_trip()
                    return
    finally:
        f.close()


def _run_with_ffb_guard(
    cmd: list[str],
    env: dict[str, str],
    log_file: Path,
    ffb_fail_limit: int,
    max_restarts: int,
) -> int:
    """Run command and auto-restart when consecutive FFB failures reach threshold.

    FFB log lines are written by the C++ binary into `log_file` (via SBF_LOG_FILE),
    not to stdout, so we tail the log file from a side thread.
    """
    restart_count = 0
    while True:
        attempt_no = restart_count + 1
        print(f"\n[RUN] Attempt {attempt_no}: {' '.join(shlex.quote(x) for x in cmd)}")

        # Truncate any existing log so the tail starts fresh.
        try:
            log_file.parent.mkdir(parents=True, exist_ok=True)
            with open(log_file, "w"):
                pass
        except OSError as e:
            print(f"[FFB] Could not pre-clear log file {log_file}: {e}", file=sys.stderr)

        state = _FFBGuardState(ffb_fail_limit)
        stop_event = threading.Event()
        terminated_by_guard = {"flag": False}

        proc = subprocess.Popen(
            cmd,
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )

        def on_trip() -> None:
            terminated_by_guard["flag"] = True
            print(
                f"[FFB] Reached failure limit ({ffb_fail_limit}), terminating and restarting...",
                file=sys.stderr,
            )
            try:
                proc.terminate()
            except ProcessLookupError:
                pass

        tail_thread = threading.Thread(
            target=_tail_log_for_ffb,
            args=(log_file, state, stop_event, on_trip),
            daemon=True,
        )
        tail_thread.start()

        assert proc.stdout is not None
        for line in proc.stdout:
            sys.stdout.write(line)
        proc.wait()

        # Stop tail thread immediately. Any remaining unread log lines are
        # post-exit chatter — the C++ planner has its own consecutive-FFB-fail
        # counter (max_consecutive_miss) so the Python tail is only a backup
        # for genuinely stuck processes.
        stop_event.set()
        tail_thread.join(timeout=1.0)

        if proc.returncode == 0 and not terminated_by_guard["flag"]:
            return 0

        if terminated_by_guard["flag"]:
            if restart_count >= max_restarts:
                print(
                    f"[FFB] Exceeded max restarts ({max_restarts}); aborting.",
                    file=sys.stderr,
                )
                return proc.returncode if proc.returncode else 1
            restart_count += 1
            continue

        # Non-FFB-limit failure: propagate the original failure code.
        return proc.returncode if proc.returncode is not None else 1


def _extract_log_stats(log_file: Path) -> dict:
    stats = {
        "grw_boxes": None,
        "grw_success": None,
        "grw_fail": None,
        "grw_promo": None,
        "elapsed_s": None,
        "boxes": None,
        "edges": None,
        "islands": None,
    }
    try:
        text = log_file.read_text(errors="replace")
    except OSError:
        return stats

    m1 = _GRW_FINAL_RE.search(text)
    if m1:
        stats["grw_boxes"] = int(m1.group(1))
        stats["grw_success"] = int(m1.group(2))
        stats["grw_fail"] = int(m1.group(3))
        stats["grw_promo"] = int(m1.group(4))

    m2 = _DONE_RE.search(text)
    if m2:
        stats["elapsed_s"] = float(m2.group(1))
        stats["boxes"] = int(m2.group(2))
        stats["edges"] = int(m2.group(3))
        stats["islands"] = int(m2.group(4))

    return stats


def main() -> int:
    ap = argparse.ArgumentParser(description="Batch-generate 2D replay logs/videos")
    ap.add_argument("--scene", choices=["simple", "narrow", "cluttered", "scattered", "random"], default="random")
    ap.add_argument("--max-boxes", type=int, default=40)
    ap.add_argument("--max-depth", type=int, default=100)
    ap.add_argument("--threads", type=int, default=1)
    ap.add_argument("--endpoint", choices=["ifk", "critsample"], default="ifk")
    ap.add_argument(
        "--envelope",
        choices=["linkiaabb", "linkiaabb_grid", "hull16_grid"],
        default="linkiaabb",
    )
    ap.add_argument("--binary", default=None, help="Path to exp_2d_trace binary")
    ap.add_argument("--log-file", default=None, help="Optional explicit output log path")
    ap.add_argument(
        "--ffb-fail-limit",
        type=int,
        default=20,
        help="Restart run if consecutive FFB failures reach this threshold",
    )
    ap.add_argument(
        "--vol-bonus-alpha",
        type=float,
        default=0.05,
        help="RRT nearest-box volume bonus coefficient (0 disables)",
    )
    ap.add_argument(
        "--random-obstacles",
        type=int,
        default=6,
        help="Number of random AABB obstacles (used by --scene random)",
    )
    ap.add_argument(
        "--obs-center-max-radius",
        type=float,
        default=2.0,
        help="Max radius of random obstacle centers from origin",
    )
    ap.add_argument(
        "--random-start-goal",
        action="store_true",
        default=True,
        help="Randomize start/goal then enforce FFB-success via exp_2d_trace",
    )
    ap.add_argument(
        "--runs",
        type=int,
        default=10,
        help="Number of runs to execute",
    )
    ap.add_argument(
        "--seed-base",
        type=int,
        default=20260423,
        help="Base scene seed; run i uses seed_base + i",
    )
    ap.add_argument(
        "--scene-retries",
        type=int,
        default=30,
        help="Max seed retries per run when random scene is infeasible",
    )
    ap.add_argument(
        "--max-restarts",
        type=int,
        default=10,
        help="Maximum auto-restarts triggered by FFB failure threshold",
    )
    ap.add_argument(
        "--media-format",
        choices=["mp4", "gif"],
        default="mp4",
        help="Replay media format to export after the run",
    )
    ap.add_argument(
        "--fps",
        type=int,
        default=10,
        help="Replay media frames per second",
    )
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    if (
        args.max_boxes <= 0
        or args.max_depth <= 0
        or args.threads <= 0
        or args.ffb_fail_limit <= 0
        or args.fps <= 0
        or args.runs <= 0
        or args.obs_center_max_radius <= 0
        or args.scene_retries <= 0
        or args.max_restarts < 0
    ):
        print(
            "ERROR: --max-boxes, --max-depth, --threads, --ffb-fail-limit, --fps, --runs, --obs-center-max-radius, --scene-retries must be > 0, and --max-restarts must be >= 0",
            file=sys.stderr,
        )
        return 2

    ts = time.strftime("%Y%m%d_%H%M%S")
    repo_root = Path(__file__).resolve().parents[1]
    out_dir = repo_root / "log"
    out_dir.mkdir(parents=True, exist_ok=True)
    run_dir = out_dir / f"2d_trace_batch_{args.scene}_{args.endpoint}_{args.envelope}_{ts}"
    run_dir.mkdir(parents=True, exist_ok=True)

    try:
        binary = _pick_binary(args.binary)
    except FileNotFoundError as e:
        print(str(e), file=sys.stderr)
        return 1

    env = os.environ.copy()

    print(f"[BATCH] run_dir={run_dir}")
    print(f"[BATCH] runs={args.runs} fps={args.fps} scene={args.scene}")

    if args.dry_run:
        return 0

    results = []
    failure_count = 0
    for run_idx in range(1, args.runs + 1):
        if args.log_file and args.runs == 1:
            log_file = Path(args.log_file)
        else:
            log_file = run_dir / f"run_{run_idx:02d}.log"

        rc = 1
        used_seed = None
        for seed_try in range(args.scene_retries):
            scene_seed = int(args.seed_base) + run_idx + seed_try * args.runs
            cmd = [
                str(binary),
                "--scene", args.scene,
                "--max-boxes", str(args.max_boxes),
                "--max-depth", str(args.max_depth),
                "--threads", str(args.threads),
                "--endpoint", args.endpoint,
                "--envelope", args.envelope,
                "--ffb-fail-limit", str(args.ffb_fail_limit),
                "--vol-bonus-alpha", str(args.vol_bonus_alpha),
                "--random-obstacles", str(args.random_obstacles),
                "--obs-center-max-radius", str(args.obs_center_max_radius),
                "--scene-seed", str(scene_seed),
            ]
            if args.random_start_goal:
                cmd.append("--random-start-goal")

            env["SBF_LOG_FILE"] = str(log_file)

            print("\n" + "=" * 60)
            print(
                f"[BATCH] Run {run_idx}/{args.runs} seed={scene_seed} "
                f"(try {seed_try + 1}/{args.scene_retries})"
            )
            print("Command:", " ".join(shlex.quote(x) for x in cmd))
            print("SBF_LOG_FILE=", log_file)

            rc = _run_with_ffb_guard(
                cmd=cmd,
                env=env,
                log_file=log_file,
                ffb_fail_limit=args.ffb_fail_limit,
                max_restarts=args.max_restarts,
            )
            if rc == 0:
                used_seed = scene_seed
                break
            if args.scene != "random":
                break
            print(
                f"[BATCH] run {run_idx} seed={scene_seed} failed (rc={rc}), retrying with new seed...",
                file=sys.stderr,
            )

        media_file = None
        if rc == 0:
            print("Done. Log:", log_file)
            if args.media_format == "mp4":
                media_file = _generate_mp4(log_file, fps=args.fps)
            else:
                media_file = _generate_gif(log_file, fps=args.fps)
        else:
            failure_count += 1
            print(f"[BATCH] run {run_idx} failed: rc={rc}", file=sys.stderr)

        row = {
            "run": run_idx,
            "scene_seed": used_seed,
            "return_code": rc,
            "log_file": str(log_file),
            "media_file": str(media_file) if media_file else None,
        }
        row.update(_extract_log_stats(log_file))
        results.append(row)

    summary = {
        "timestamp": ts,
        "scene": args.scene,
        "runs": args.runs,
        "failed_runs": failure_count,
        "fps": args.fps,
        "endpoint": args.endpoint,
        "envelope": args.envelope,
        "ffb_fail_limit": args.ffb_fail_limit,
        "vol_bonus_alpha": args.vol_bonus_alpha,
        "random_obstacles": args.random_obstacles,
        "obs_center_max_radius": args.obs_center_max_radius,
        "random_start_goal": bool(args.random_start_goal),
        "seed_base": int(args.seed_base),
        "run_dir": str(run_dir),
        "results": results,
    }
    summary_file = run_dir / "summary.json"
    summary_file.write_text(json.dumps(summary, ensure_ascii=False, indent=2))
    print("\n" + "=" * 60)
    print(f"[BATCH] Summary: {summary_file}")
    print(f"[BATCH] Success: {args.runs - failure_count}/{args.runs}")
    print("=" * 60)

    return 0 if failure_count == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
