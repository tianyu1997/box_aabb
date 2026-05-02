#!/usr/bin/env python3
"""
viz_2d_build_replay.py — 2D SBF Build Trace Interactive Replay Viewer

Features:
  - Parse multi-threaded trace log into frames
  - Interactive replay: space=pause/play, arrow keys=frame nav, q=quit
  - Matplotlib display: C-space boxes + workspace visualization
  - Optional: GIF/MP4 export

Usage:
  python3 viz_2d_build_replay.py log/2d_trace_simple_*.log [--video out.mp4 [--fps 15]]
"""

import argparse
import os
import re
import sys
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.gridspec import GridSpec

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_PY_BINDING_DIR = os.path.normpath(os.path.join(_THIS_DIR, '..', 'python'))
if _PY_BINDING_DIR not in sys.path:
    sys.path.insert(0, _PY_BINDING_DIR)
try:
    import sbf6
except Exception:
    sbf6 = None

# ─── Data Structures ─────────────────────────────────────────────────────────

@dataclass
class Interval:
    """1D interval [lo, hi]."""
    lo: float
    hi: float

    def center(self) -> float:
        return 0.5 * (self.lo + self.hi)

    def width(self) -> float:
        return self.hi - self.lo


@dataclass
class BoxInfo:
    """C-space box."""
    box_id: int
    intervals: List[Interval]
    seed: np.ndarray
    root_id: int
    parent_id: int
    tid: int
    frame_idx: int  # When created
    is_bridge: bool = False  # Created during bridge phase

    def get_rect(self) -> Tuple[float, float, float, float]:
        """Return (q1_lo, q1_hi, q2_lo, q2_hi) for matplotlib Rectangle."""
        if len(self.intervals) >= 2:
            return (self.intervals[0].lo, self.intervals[1].lo,
                    self.intervals[0].width(), self.intervals[1].width())
        return (0, 0, 0, 0)


@dataclass
class Frame:
    """A single replay frame."""
    idx: int
    kind: str  # 'metadata', 'ffb-begin', 'ffb-step', 'box-ok', 'rrt-sample', etc.
    tid: int  # Thread ID (-1 for serial)
    payload: dict = field(default_factory=dict)
    raw_line: str = ""


@dataclass
class FFBSession:
    """One FFB descent from root_iv toward a free leaf."""
    seed: np.ndarray
    root_iv: List[Interval]  # Full C-space domain at start
    steps: List[dict] = field(default_factory=list)  # Parsed step records
    complete: bool = False
    outcome: str = ''  # 'free' | 'fail' | ''

    def iv_at_step(self, n_steps: int) -> List[Interval]:
        """Reconstruct the C-space interval after applying the first n_steps splits."""
        ivs = [Interval(iv.lo, iv.hi) for iv in self.root_iv]
        for s in self.steps[:n_steps]:
            if s.get('action') == 'descend':
                d, val, side = s.get('split_dim', -1), s.get('split_val', 0.0), s.get('side', 'L')
                if 0 <= d < len(ivs):
                    if side == 'R':
                        ivs[d] = Interval(val, ivs[d].hi)
                    else:
                        ivs[d] = Interval(ivs[d].lo, val)
        return ivs


# ─── Log Parser ──────────────────────────────────────────────────────────────

class LogParser:
    """Parse 2D trace log into frames."""

    def __init__(self, logfile: str):
        self.logfile = logfile
        self.frames: List[Frame] = []
        self.metadata: dict = {}
        self.boxes_by_id: Dict[int, BoxInfo] = {}
        self._in_bridge_phase: bool = False
        self.parse()

    def parse(self):
        """Parse log file into frames."""
        with open(self.logfile, 'r') as f:
            for raw_line in f:
                raw_line = raw_line.rstrip('\n')
                if not raw_line:
                    continue

                # Extract TID if present
                tid = -1
                tid_match = re.search(r'\[tid=(\d+)\]', raw_line)
                if tid_match:
                    tid = int(tid_match.group(1))

                # Route by pattern
                if '[2D-META]' in raw_line:
                    self._parse_meta(raw_line)
                    self.frames.append(Frame(
                        idx=len(self.frames), kind='metadata', tid=tid,
                        payload={'line': raw_line}, raw_line=raw_line
                    ))
                elif '[GRW-FFB] OK' in raw_line:
                    payload = self._parse_box_ok(raw_line)
                    if payload:
                        bi = BoxInfo(
                            box_id=payload['box_id'],
                            intervals=payload['intervals'],
                            seed=payload['seed'],
                            root_id=payload['root_id'],
                            parent_id=payload['parent_id'],
                            tid=tid,
                            frame_idx=len(self.frames),
                            is_bridge=self._in_bridge_phase,
                        )
                        self.boxes_by_id[bi.box_id] = bi
                        payload['box'] = bi
                    self.frames.append(Frame(
                        idx=len(self.frames), kind='box-ok', tid=tid,
                        payload=payload, raw_line=raw_line
                    ))
                elif '[GRW-RRT]' in raw_line:
                    payload = self._parse_rrt_sample(raw_line)
                    self.frames.append(Frame(
                        idx=len(self.frames), kind='rrt-sample', tid=tid,
                        payload=payload, raw_line=raw_line
                    ))
                elif '[GRW-ROOT]' in raw_line:
                    payload = self._parse_root_seed(raw_line)
                    self.frames.append(Frame(
                        idx=len(self.frames), kind='root-seed', tid=tid,
                        payload=payload, raw_line=raw_line
                    ))
                elif '[FFB] begin' in raw_line and 'root_iv=' in raw_line:
                    # Only the tid-tagged begin line (not the duplicate plain one)
                    payload = self._parse_ffb_begin(raw_line)
                    self.frames.append(Frame(
                        idx=len(self.frames), kind='ffb-begin', tid=tid,
                        payload=payload, raw_line=raw_line
                    ))
                elif '[FFB]  step=' in raw_line:
                    payload = self._parse_ffb_step(raw_line)
                    self.frames.append(Frame(
                        idx=len(self.frames), kind='ffb-step', tid=tid,
                        payload=payload, raw_line=raw_line
                    ))
                elif '[BRG-ALL] round' in raw_line:
                    self._in_bridge_phase = True
                    m = re.search(r'round (\d+): (\d+) islands', raw_line)
                    brg_payload = {}
                    if m:
                        brg_payload = {'round_no': int(m.group(1)), 'n_islands': int(m.group(2))}
                    self.frames.append(Frame(
                        idx=len(self.frames), kind='bridge-round', tid=tid,
                        payload=brg_payload, raw_line=raw_line
                    ))
                elif '[BRG-ALL] island #' in raw_line and 'pair' in raw_line:
                    payload = self._parse_bridge_pair(raw_line)
                    self.frames.append(Frame(
                        idx=len(self.frames), kind='bridge-pair', tid=tid,
                        payload=payload, raw_line=raw_line
                    ))
                elif '[BRG-ALL] done' in raw_line:
                    self._in_bridge_phase = False
                    m = re.search(r'(\d+) islands remain, (\d+) bridge boxes', raw_line)
                    brg_payload = {}
                    if m:
                        brg_payload = {'islands_remain': int(m.group(1)), 'bridge_boxes': int(m.group(2))}
                    self.frames.append(Frame(
                        idx=len(self.frames), kind='bridge-done', tid=tid,
                        payload=brg_payload, raw_line=raw_line
                    ))

    def _parse_meta(self, line: str):
        """Extract [2D-META] info."""
        m = re.search(r'scene=(\w+)', line)
        if m:
            self.metadata['scene'] = m.group(1)
        m = re.search(r'dof=(\d+)', line)
        if m:
            self.metadata['dof'] = int(m.group(1))
        m = re.search(r'n_threads=(\d+)', line)
        if m:
            self.metadata['n_threads'] = int(m.group(1))
        m = re.search(r'max_boxes=(\d+)', line)
        if m:
            self.metadata['max_boxes'] = int(m.group(1))
        m = re.search(r'link_lengths=\[([\d\.\-,]+)\]', line)
        if m:
            vals = [float(x) for x in m.group(1).split(',')]
            self.metadata['link_lengths'] = vals
        m = re.search(r'endpoint=([A-Za-z0-9_]+)', line)
        if m:
            ep = m.group(1).strip().lower()
            if ep in ('ifk', 'critsample', 'crit'):
                self.metadata['endpoint'] = 'ifk' if ep == 'ifk' else 'critsample'
        m = re.search(r'envelope=([A-Za-z0-9_]+)', line)
        if m:
            env = m.group(1).strip().lower()
            env_map = {
                'linkiaabb': 'linkiaabb',
                'linkiaabb_grid': 'linkiaabb_grid',
                'grid': 'linkiaabb_grid',
                'hull16_grid': 'hull16_grid',
                'hull16': 'hull16_grid',
            }
            if env in env_map:
                self.metadata['envelope'] = env_map[env]

    def _extract_seed(self, line: str) -> Optional[np.ndarray]:
        """Extract `seed=[...]` from line."""
        m = re.search(r'seed=\[([\d\.\-,]+)\]', line)
        if m:
            vals = [float(x) for x in m.group(1).split(',')]
            return np.array(vals)
        return None

    def _parse_intervals(self, intervals_str: str) -> List[Interval]:
        """Parse interval string like '(lo,hi),(lo,hi)' or with leading [."""
        ivs = []
        # Remove outer brackets if present
        intervals_str = intervals_str.strip('[]')
        # Find all (lo, hi) pairs
        for match in re.finditer(r'\(([\d\.\-]+),([\d\.\-]+)\)', intervals_str):
            lo = float(match.group(1))
            hi = float(match.group(2))
            ivs.append(Interval(lo, hi))
        return ivs

    def _parse_box_ok(self, line: str) -> Optional[dict]:
        """Parse [GRW-FFB] OK line."""
        payload = {}
        m = re.search(r'box=(\d+)', line)
        if m:
            payload['box_id'] = int(m.group(1))
        m = re.search(r'root=(\d+)', line)
        if m:
            payload['root_id'] = int(m.group(1))
        m = re.search(r'parent=(-?\d+)', line)
        if m:
            payload['parent_id'] = int(m.group(1))
        m = re.search(r'seed=\[([\d\.\-,]+)\]', line)
        if m:
            vals = [float(x) for x in m.group(1).split(',')]
            payload['seed'] = np.array(vals)
        m = re.search(r'intervals=\[(.*?)\]', line)
        if m:
            payload['intervals'] = self._parse_intervals(m.group(1))
        return payload if 'box_id' in payload else None

    def _parse_rrt_sample(self, line: str) -> dict:
        """Parse [GRW-RRT] line: sample=, snap_seed=, nearest=."""
        payload = {}
        # sample= is the random target point
        m = re.search(r'sample=\[([\d\.\-,]+)\]', line)
        if m:
            payload['sample'] = np.array([float(x) for x in m.group(1).split(',')])
        # snap_seed= is the face-snapped FFB seed (projected onto nearest box)
        m = re.search(r'snap_seed=\[([\d\.\-,]+)\]', line)
        if m:
            payload['snap_seed'] = np.array([float(x) for x in m.group(1).split(',')])
        # nearest= box id
        m = re.search(r'nearest=(\d+)', line)
        if m:
            payload['nearest_box'] = int(m.group(1))
        # Legacy: q= field
        m = re.search(r'q=\[([\d\.\-,]+)\]', line)
        if m:
            payload['q'] = np.array([float(x) for x in m.group(1).split(',')])
        return payload

    def _parse_bridge_pair(self, line: str) -> dict:
        """Parse [BRG-ALL] island #N pair M/K line."""
        payload: dict = {}
        m = re.search(r'island #(\d+) pair (\d+)/(\d+)', line)
        if m:
            payload['island_id'] = int(m.group(1))
            payload['pair_no'] = int(m.group(2))
            payload['pair_total'] = int(m.group(3))
        m = re.search(r'box (\d+)->(\d+)', line)
        if m:
            payload['box_from'] = int(m.group(1))
            payload['box_to'] = int(m.group(2))
        m = re.search(r'dist=([\d\.]+)', line)
        if m:
            payload['dist'] = float(m.group(1))
        m = re.search(r'rrt=(\S+)', line)
        if m:
            payload['rrt_result'] = m.group(1).rstrip(',')
        payload['merged'] = 'merged=YES' in line
        return payload

    def _parse_root_seed(self, line: str) -> dict:
        """Parse [GRW-ROOT] line: multi-goal / endpoint start / goal."""
        payload = {}
        if 'multi-goal seed' in line:
            payload['kind'] = 'multi-goal'
            m = re.search(r'= \[([\d\.\-,]+)\]', line)
            if m:
                payload['seed'] = np.array([float(x) for x in m.group(1).split(',')])
        elif 'endpoint start' in line:
            payload['kind'] = 'start'
            m = re.search(r'= \[([\d\.\-,]+)\]', line)
            if m:
                payload['seed'] = np.array([float(x) for x in m.group(1).split(',')])
        elif 'endpoint goal' in line:
            payload['kind'] = 'goal'
            m = re.search(r'= \[([\d\.\-,]+)\]', line)
            if m:
                payload['seed'] = np.array([float(x) for x in m.group(1).split(',')])
        return payload

    def _parse_ffb_begin(self, line: str) -> dict:
        """Parse [FFB] begin line."""
        payload = {}
        seed = self._extract_seed(line)
        if seed is not None:
            payload['seed'] = seed
        m = re.search(r'root_iv=\[(.*?)\]', line)
        if m:
            payload['root_iv'] = self._parse_intervals(m.group(1))
        m = re.search(r'max_depth=(\d+)', line)
        if m:
            payload['max_depth'] = int(m.group(1))
        return payload

    def _parse_ffb_step(self, line: str) -> dict:
        """Parse [FFB]  step= line."""
        payload = {}
        for key, pat in [('step', r'step=(\d+)'), ('node', r'node=(\d+)'),
                         ('depth', r'depth=(\d+)')]:
            m = re.search(pat, line)
            if m:
                payload[key] = int(m.group(1))
        if 'descend' in line:
            payload['action'] = 'descend'
            m = re.search(r'split_dim=(\d+)', line)
            if m:
                payload['split_dim'] = int(m.group(1))
            m = re.search(r'split_val=([\d\.\-]+)', line)
            if m:
                payload['split_val'] = float(m.group(1))
            m = re.search(r'-> (L|R)', line)
            if m:
                payload['side'] = m.group(1)
        elif 'FREE' in line:
            payload['action'] = 'free'
        elif 'EXPAND' in line:
            payload['action'] = 'expand'
        else:
            payload['action'] = 'collide'
        return payload


# ─── Viewer State ────────────────────────────────────────────────────────────

@dataclass
class ViewerState:
    """Manages viewer state (current frame, paused, etc.)."""
    current_frame_idx: int = 0
    paused: bool = False
    boxes_created: Dict[int, BoxInfo] = field(default_factory=dict)
    # rrt_samples: random target points (gray)
    rrt_samples: List[np.ndarray] = field(default_factory=list)
    # snap_seeds: FFB snap points (orange x) + their paired sample
    snap_seeds: List[np.ndarray] = field(default_factory=list)
    snap_samples: List[np.ndarray] = field(default_factory=list)
    # root_seeds: start/goal/multi-goal (purple stars)
    root_seeds: List[Tuple[np.ndarray, str]] = field(default_factory=list)  # (pt, kind)
    # current highlighted points
    current_q: Optional[np.ndarray] = None
    current_snap: Optional[np.ndarray] = None
    current_sample: Optional[np.ndarray] = None
    # active FFB session and how many steps applied
    active_ffb: Optional[FFBSession] = None
    active_ffb_steps: int = 0  # Number of steps accumulated in current session
    completed_ffb_sessions: List[FFBSession] = field(default_factory=list)
    # Workspace envelope mode: 'linkiaabb' | 'linkiaabb_grid' | 'hull16_grid'
    envelope_mode: str = 'hull16_grid'
    # Endpoint IAABB source for diagnostics: 'IFK' | 'CritSample'
    endpoint_source: str = 'IFK'
    # Bridge phase tracking
    bridge_phase_active: bool = False
    active_bridge_pair: Optional[Tuple[int, int]] = None  # (box_from, box_to)


# ─── 2D Viewer ───────────────────────────────────────────────────────────────

class Viewer2D:
    """Interactive 2D SBF build replay viewer."""

    def __init__(self, parser: LogParser):
        self.parser = parser
        self.state = ViewerState()
        self.fig: Optional[plt.Figure] = None
        self.ax_cspace: Optional[plt.Axes] = None
        self.ax_workspace: Optional[plt.Axes] = None
        self.ax_log: Optional[plt.Axes] = None
        self.play_timer = None
        self.cspace_occ = None
        self.cspace_extent = None
        self.sbf6_robot = None
        self._last_env_diag = ''
        if sbf6 is not None:
            robot_json = os.path.normpath(os.path.join(_THIS_DIR, '..', 'data', '2dof_planar.json'))
            if os.path.exists(robot_json):
                try:
                    self.sbf6_robot = sbf6.Robot.from_json(robot_json)
                except Exception:
                    self.sbf6_robot = None

        # Auto-follow generation pipeline from [2D-META].
        ep = str(self.parser.metadata.get('endpoint', 'ifk')).lower()
        self.state.endpoint_source = 'IFK' if ep == 'ifk' else 'CritSample'
        env = str(self.parser.metadata.get('envelope', 'hull16_grid')).lower()
        if env in ('linkiaabb', 'linkiaabb_grid', 'hull16_grid'):
            self.state.envelope_mode = env

    @staticmethod
    def _parse_obstacles_from_meta(line: str) -> List[Tuple[float, float, float, float]]:
        # obstacles=[(x0,y0,x1,y1),(...)]
        m = re.search(r'obstacles=\[(.*?)\]', line)
        if not m:
            return []
        body = m.group(1)
        obs = []
        for mm in re.finditer(r'\(([\d\.-]+),([\d\.-]+),([\d\.-]+),([\d\.-]+)\)', body):
            x0 = float(mm.group(1))
            y0 = float(mm.group(2))
            x1 = float(mm.group(3))
            y1 = float(mm.group(4))
            obs.append((min(x0, x1), min(y0, y1), max(x0, x1), max(y0, y1)))
        return obs

    @staticmethod
    def _parse_limits_from_meta(line: str) -> List[Tuple[float, float]]:
        # limits=[(-3.1416,3.1416),(-3.1416,3.1416)]
        m = re.search(r'limits=\[(.*?)\]', line)
        if not m:
            return [(-np.pi, np.pi), (-np.pi, np.pi)]
        body = m.group(1)
        vals = []
        for mm in re.finditer(r'\(([\d\.-]+),([\d\.-]+)\)', body):
            vals.append((float(mm.group(1)), float(mm.group(2))))
        if len(vals) >= 2:
            return vals[:2]
        return [(-np.pi, np.pi), (-np.pi, np.pi)]

    @staticmethod
    def _point_in_rect(px: float, py: float, rect: Tuple[float, float, float, float]) -> bool:
        x0, y0, x1, y1 = rect
        return x0 <= px <= x1 and y0 <= py <= y1

    def _segment_hits_rect(self, p0: np.ndarray, p1: np.ndarray,
                           rect: Tuple[float, float, float, float]) -> bool:
        # Fast approximation by dense sampling along segment.
        ts = np.linspace(0.0, 1.0, 40)
        seg = p0[None, :] * (1.0 - ts[:, None]) + p1[None, :] * ts[:, None]
        x0, y0, x1, y1 = rect
        inside = (seg[:, 0] >= x0) & (seg[:, 0] <= x1) & (seg[:, 1] >= y0) & (seg[:, 1] <= y1)
        return bool(np.any(inside))

    def _fk(self, q: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        lengths = self.parser.metadata.get('link_lengths', [1.0, 1.0])
        if len(lengths) < 2:
            lengths = [1.0, 1.0]
        l1, l2 = float(lengths[0]), float(lengths[1])
        q1 = float(q[0])
        q2 = float(q[1]) if len(q) > 1 else 0.0
        p0 = np.array([0.0, 0.0])
        p1 = np.array([l1 * np.cos(q1), l1 * np.sin(q1)])
        p2 = p1 + np.array([l2 * np.cos(q1 + q2), l2 * np.sin(q1 + q2)])
        return p0, p1, p2

    def _in_collision(self, q: np.ndarray, obstacles: List[Tuple[float, float, float, float]]) -> bool:
        if len(q) < 2:
            return False
        p0, p1, p2 = self._fk(q)
        for rect in obstacles:
            if self._point_in_rect(p0[0], p0[1], rect):
                return True
            if self._point_in_rect(p1[0], p1[1], rect):
                return True
            if self._point_in_rect(p2[0], p2[1], rect):
                return True
            if self._segment_hits_rect(p0, p1, rect):
                return True
            if self._segment_hits_rect(p1, p2, rect):
                return True
        return False

    def _build_cspace_obstacle_map(self):
        if self.cspace_occ is not None:
            return
        if not self.parser.frames:
            return
        meta_line = ''
        for fr in self.parser.frames:
            if fr.kind == 'metadata':
                meta_line = fr.raw_line
                break
        obstacles = self._parse_obstacles_from_meta(meta_line)
        limits = self.parser.metadata.get('limits', [(-np.pi, np.pi), (-np.pi, np.pi)])
        q1_lo, q1_hi = limits[0]
        q2_lo, q2_hi = limits[1]

        n = 80
        q1_vals = np.linspace(q1_lo, q1_hi, n)
        q2_vals = np.linspace(q2_lo, q2_hi, n)
        occ = np.zeros((n, n), dtype=np.uint8)

        if obstacles:
            for i, q2 in enumerate(q2_vals):
                for j, q1 in enumerate(q1_vals):
                    q = np.array([q1, q2], dtype=float)
                    occ[i, j] = 1 if self._in_collision(q, obstacles) else 0

        self.cspace_occ = occ
        self.cspace_extent = [q1_lo, q1_hi, q2_lo, q2_hi]

    def setup_figure(self):
        """Setup matplotlib figure and axes."""
        self.fig = plt.figure(figsize=(14, 8))
        gs = GridSpec(2, 2, height_ratios=[4.0, 1.4], hspace=0.18, wspace=0.2)
        self.ax_cspace = self.fig.add_subplot(gs[0, 0])
        self.ax_workspace = self.fig.add_subplot(gs[0, 1])
        self.ax_log = self.fig.add_subplot(gs[1, :])

        self.fig.suptitle(f"2D SBF Build Replay: {self.parser.metadata.get('scene', 'unknown')}")

        meta = self.parser.metadata
        if 'limits' not in meta:
            # Infer limits from metadata or use defaults
            meta['limits'] = [(-np.pi, np.pi), (-np.pi, np.pi)]

        self._build_cspace_obstacle_map()

        # C-space plot
        limits = meta.get('limits', [(-np.pi, np.pi), (-np.pi, np.pi)])
        self.ax_cspace.set_xlim(limits[0])
        self.ax_cspace.set_ylim(limits[1] if len(limits) > 1 else limits[0])
        self.ax_cspace.set_xlabel('q1')
        self.ax_cspace.set_ylabel('q2')
        self.ax_cspace.set_title('C-space (configuration space)')
        self.ax_cspace.set_aspect('equal', adjustable='box')
        self.ax_cspace.grid(True, alpha=0.3)

        # Workspace plot
        self.ax_workspace.set_xlim(-2.5, 2.5)
        self.ax_workspace.set_ylim(-2.5, 2.5)
        self.ax_workspace.set_xlabel('x')
        self.ax_workspace.set_ylabel('y')
        self.ax_workspace.set_title('Workspace (robot)')
        self.ax_workspace.set_aspect('equal', adjustable='box')
        self.ax_workspace.grid(True, alpha=0.3)

        # Log panel
        self.ax_log.set_title('Log (synchronized)')
        self.ax_log.set_axis_off()

        self.play_timer = self.fig.canvas.new_timer(interval=160)
        self.play_timer.add_callback(self._on_timer)
        self.play_timer.start()

        # Setup keyboard events
        self.fig.canvas.mpl_connect('key_press_event', self._on_key_press)

    def _on_key_press(self, event):
        """Handle keyboard input."""
        if event.key == ' ':
            self.state.paused = not self.state.paused
            print(f"{'Paused' if self.state.paused else 'Playing'}")
        elif event.key == 'right' or event.key == '>':
            self.state.current_frame_idx = min(self.state.current_frame_idx + 1, 
                                               len(self.parser.frames) - 1)
            self._update_display()
        elif event.key == 'left' or event.key == '<':
            self.state.current_frame_idx = max(self.state.current_frame_idx - 1, 0)
            self._update_display()
        elif event.key == 'home':
            self.state.current_frame_idx = 0
            self._update_display()
        elif event.key == 'end':
            self.state.current_frame_idx = len(self.parser.frames) - 1
            self._update_display()
        elif event.key == 'q':
            plt.close(self.fig)
            sys.exit(0)

    def _on_timer(self):
        if self.state.paused:
            return
        if self.state.current_frame_idx < len(self.parser.frames) - 1:
            self.state.current_frame_idx += 1
            self._update_display()

    def _update_display(self):
        """Update display based on current frame."""
        if not self.parser.frames:
            return

        self.ax_cspace.clear()
        self.ax_workspace.clear()
        self.ax_log.clear()

        # Re-setup axes
        meta = self.parser.metadata
        limits = meta.get('limits', [(-np.pi, np.pi), (-np.pi, np.pi)])
        self.ax_cspace.set_xlim(limits[0])
        self.ax_cspace.set_ylim(limits[1] if len(limits) > 1 else limits[0])
        self.ax_cspace.set_xlabel('q1')
        self.ax_cspace.set_ylabel('q2')
        self.ax_cspace.set_title('C-space')
        self.ax_cspace.set_aspect('equal', adjustable='box')
        self.ax_cspace.grid(True, alpha=0.3)

        if self.cspace_occ is not None and self.cspace_extent is not None:
            self.ax_cspace.imshow(
                self.cspace_occ,
                origin='lower',
                extent=self.cspace_extent,
                cmap='Reds',
                alpha=0.22,
                interpolation='nearest',
                zorder=0,
            )

        lengths = self.parser.metadata.get('link_lengths', [1.0, 1.0])
        if len(lengths) < 2:
            lengths = [1.0, 1.0]
        reach = float(sum(lengths)) + 0.6
        self.ax_workspace.set_xlim(-reach, reach)
        self.ax_workspace.set_ylim(-reach, reach)
        self.ax_workspace.set_xlabel('x')
        self.ax_workspace.set_ylabel('y')
        self.ax_workspace.set_title('Workspace')
        self.ax_workspace.set_aspect('equal', adjustable='box')
        self.ax_workspace.grid(True, alpha=0.3)

        # Draw workspace obstacles from metadata.
        meta_line = ''
        for fr in self.parser.frames:
            if fr.kind == 'metadata':
                meta_line = fr.raw_line
                break
        obstacles = self._parse_obstacles_from_meta(meta_line)
        for (x0, y0, x1, y1) in obstacles:
            rect = mpatches.Rectangle(
                (x0, y0), x1 - x0, y1 - y0,
                linewidth=1.2, edgecolor='black', facecolor='#777777', alpha=0.45
            )
            self.ax_workspace.add_patch(rect)

        # Log panel setup.
        self.ax_log.set_axis_off()
        self.ax_log.set_title('Log (synchronized)', loc='left')

        self.state.boxes_created.clear()
        self.state.rrt_samples.clear()
        self.state.snap_seeds.clear()
        self.state.snap_samples.clear()
        self.state.root_seeds.clear()
        self.state.current_q = None
        self.state.current_snap = None
        self.state.current_sample = None
        self.state.bridge_phase_active = False
        self.state.active_bridge_pair = None

        # ── Accumulate state up to current frame ──────────────────────────
        for i in range(self.state.current_frame_idx + 1):
            frame = self.parser.frames[i]
            is_current = (i == self.state.current_frame_idx)

            if frame.kind == 'box-ok' and 'box' in frame.payload:
                box = frame.payload['box']
                self.state.boxes_created[box.box_id] = box
                q1, q2, w1, w2 = box.get_rect()
                # Bridge boxes: orange; grow boxes: blue
                if box.is_bridge:
                    fc = '#fff3cc' if not is_current else '#ffe080'
                    ec = 'darkorange'
                    ew = 1.5 if not is_current else 2.5
                else:
                    fc = 'lightblue' if not is_current else '#a0d0ff'
                    ec = 'blue'
                    ew = 1 if not is_current else 2.0
                rect = mpatches.Rectangle(
                    (q1, q2), w1, w2,
                    linewidth=ew, edgecolor=ec, facecolor=fc, alpha=0.35, zorder=2
                )
                self.ax_cspace.add_patch(rect)
                self.state.current_q = box.seed

            elif frame.kind == 'bridge-round':
                self.state.bridge_phase_active = True
                self.state.active_bridge_pair = None

            elif frame.kind == 'bridge-pair':
                bfrom = frame.payload.get('box_from', -1)
                bto = frame.payload.get('box_to', -1)
                if is_current:
                    self.state.active_bridge_pair = (bfrom, bto)

            elif frame.kind == 'bridge-done':
                if is_current:
                    self.state.active_bridge_pair = None
                    self.state.bridge_phase_active = False

            elif frame.kind == 'rrt-sample':
                p = frame.payload
                sample = p.get('sample')
                snap = p.get('snap_seed')
                if sample is not None and len(sample) >= 2:
                    self.state.rrt_samples.append(sample)
                    self.state.current_sample = sample
                if snap is not None and len(snap) >= 2:
                    self.state.snap_seeds.append(snap)
                    self.state.snap_samples.append(sample if sample is not None else snap)
                    self.state.current_snap = snap
                    self.state.current_q = snap

            elif frame.kind == 'root-seed':
                s = frame.payload.get('seed')
                k = frame.payload.get('kind', 'root')
                if s is not None and len(s) >= 2:
                    self.state.root_seeds.append((s, k))
                    self.state.current_q = s

            elif frame.kind == 'ffb-begin':
                seed = frame.payload.get('seed')
                root_iv = frame.payload.get('root_iv', [])
                # Archive previous session if any
                if self.state.active_ffb is not None:
                    self.state.completed_ffb_sessions.append(self.state.active_ffb)
                self.state.active_ffb = FFBSession(
                    seed=seed if seed is not None else np.zeros(2),
                    root_iv=root_iv
                )
                self.state.active_ffb_steps = 0
                if seed is not None and len(seed) >= 2:
                    self.state.current_q = seed

            elif frame.kind == 'ffb-step':
                if self.state.active_ffb is not None:
                    self.state.active_ffb.steps.append(frame.payload)
                    self.state.active_ffb_steps = len(self.state.active_ffb.steps)

        # ── Draw C-space seeds ────────────────────────────────────────────
        # 1. All past RRT random sample points (faint gray)
        if self.state.rrt_samples:
            pts = np.array(self.state.rrt_samples)
            self.ax_cspace.plot(pts[:, 0], pts[:, 1], '.', color='#aaaaaa',
                                markersize=4, zorder=3, label='RRT sample')

        # 2. All past snap seeds (orange ×, FFB trigger points)
        if self.state.snap_seeds:
            pt_snap = np.array(self.state.snap_seeds)
            pt_samp = np.array(self.state.snap_samples)
            self.ax_cspace.plot(pt_snap[:, 0], pt_snap[:, 1], 'x',
                                color='#e07000', markersize=7, mew=1.5,
                                zorder=4, label='snap seed (FFB)')
            # Draw arrows: sample → snap
            for s, t in zip(pt_samp, pt_snap):
                dx, dy = t[0] - s[0], t[1] - s[1]
                if abs(dx) + abs(dy) > 0.01:
                    self.ax_cspace.annotate(
                        '', xy=(t[0], t[1]), xytext=(s[0], s[1]),
                        arrowprops=dict(arrowstyle='->', color='#ccaa00',
                                        lw=0.8, alpha=0.55),
                        zorder=3
                    )

        # 3. Root seeds (start=green★, goal=red★, multi-goal=purple★)
        col_map = {'start': 'green', 'goal': 'red', 'multi-goal': '#9900cc', 'root': 'purple'}
        for (s, k) in self.state.root_seeds:
            c = col_map.get(k, 'purple')
            self.ax_cspace.plot(s[0], s[1], '*', color=c,
                                markersize=14, zorder=6)

        # 4. Current frame highlight: sample (big gray circle) and snap (big orange ×)
        if self.state.current_sample is not None:
            q = self.state.current_sample
            self.ax_cspace.plot(q[0], q[1], 'o', color='#888888',
                                markersize=10, zorder=7, alpha=0.8)
        if self.state.current_snap is not None:
            q = self.state.current_snap
            self.ax_cspace.plot(q[0], q[1], 'x', color='#ff5500',
                                markersize=14, mew=3, zorder=8)

        # 5. FFB descent boxes: nested C-space intervals from root → current step
        if self.state.active_ffb is not None and self.state.active_ffb.root_iv:
            session = self.state.active_ffb
            n_steps = self.state.active_ffb_steps
            # Draw from root_iv (pale) down to current step (bright)
            n_show = min(n_steps, 12)  # cap displayed layers to avoid clutter
            step_indices = list(range(0, n_steps + 1))
            if len(step_indices) > 13:
                # subsample but always include root and last
                step_indices = [0] + step_indices[max(1, n_steps - 11):]
            for rank, i in enumerate(step_indices):
                ivs = session.iv_at_step(i)
                if len(ivs) < 2:
                    continue
                frac = rank / max(len(step_indices) - 1, 1)
                alpha_f = 0.06 + 0.35 * frac
                ew_alpha = 0.3 + 0.7 * frac
                is_last = (i == n_steps)
                ec = '#ff2200' if is_last else '#ff8800'
                fc = '#ff5500' if is_last else '#ffcc44'
                lw = 2.0 if is_last else 0.7
                q1lo, q2lo = ivs[0].lo, ivs[1].lo
                w1, w2 = ivs[0].width(), ivs[1].width()
                rect = mpatches.Rectangle(
                    (q1lo, q2lo), w1, w2,
                    linewidth=lw, edgecolor=ec, facecolor=fc,
                    alpha=alpha_f, zorder=9,
                    linestyle='--' if not is_last else '-'
                )
                self.ax_cspace.add_patch(rect)
                # Mark the action on the current step's node
                if is_last and n_steps > 0:
                    step_info = session.steps[-1] if session.steps else {}
                    action = step_info.get('action', '')
                    cx = (ivs[0].lo + ivs[0].hi) / 2
                    cy = (ivs[1].lo + ivs[1].hi) / 2
                    self.ax_cspace.text(
                        cx, cy, action[:3].upper(),
                        ha='center', va='center', fontsize=7,
                        color='#cc1100', fontweight='bold', zorder=10
                    )

        # 5b. Workspace envelope from v6 library (IFK/CritSample + LinkIAABB/*_Grid)
        if self.state.active_ffb is not None and self.state.active_ffb.root_iv:
            session = self.state.active_ffb
            ivs = session.iv_at_step(self.state.active_ffb_steps)
            if len(ivs) >= 2 and sbf6 is not None and self.sbf6_robot is not None:
                sbf_ivs = [sbf6.Interval(float(iv.lo), float(iv.hi)) for iv in ivs]
                ep_cfg = sbf6.EndpointSourceConfig()
                ep_cfg.source = (sbf6.EndpointSource.IFK
                                 if self.state.endpoint_source == 'IFK'
                                 else sbf6.EndpointSource.CritSample)

                env_cfg = sbf6.EnvelopeTypeConfig()
                env_cfg.type = {
                    'linkiaabb': sbf6.EnvelopeType.LinkIAABB,
                    'linkiaabb_grid': sbf6.EnvelopeType.LinkIAABB_Grid,
                    'hull16_grid': sbf6.EnvelopeType.Hull16_Grid,
                }[self.state.envelope_mode]

                try:
                    ep_info = sbf6.compute_endpoint_iaabb_info(self.sbf6_robot, sbf_ivs, ep_cfg, None)
                    env_info = sbf6.compute_envelope_info(self.sbf6_robot, sbf_ivs, ep_cfg, env_cfg, None)

                    ep_boxes = np.asarray(ep_info['endpoint_iaabbs'], dtype=float).reshape((-1, 6))
                    link_boxes = np.asarray(env_info['link_iaabbs'], dtype=float).reshape((-1, 6))

                    # Draw link IAABB (xy projection) - this is the library output under current endpoint source.
                    palette = ['#3399ff', '#ff9933', '#44aa66', '#aa66ff']
                    for i, b in enumerate(link_boxes):
                        x0, y0, x1, y1 = b[0], b[1], b[3], b[4]
                        c = palette[i % len(palette)]
                        # For grid modes, use outline only; for non-grid modes, use filled boxes
                        if self.state.envelope_mode in ('linkiaabb_grid', 'hull16_grid'):
                            patch = mpatches.Rectangle(
                                (x0, y0), max(0.0, x1 - x0), max(0.0, y1 - y0),
                                linewidth=2.0, edgecolor=c, facecolor='none',
                                alpha=0.8, zorder=3, linestyle='-'
                            )
                        else:
                            patch = mpatches.Rectangle(
                                (x0, y0), max(0.0, x1 - x0), max(0.0, y1 - y0),
                                linewidth=1.4, edgecolor=c, facecolor=c,
                                alpha=0.16, zorder=3, linestyle='--'
                            )
                        self.ax_workspace.add_patch(patch)

                    # For Grid-based envelope types, draw voxel XY projection from C++ v6 grid.
                    if self.state.envelope_mode in ('linkiaabb_grid', 'hull16_grid'):
                        gxy = np.asarray(env_info.get('grid_xy_points', []), dtype=float)
                        if gxy.size >= 2:
                            gxy = gxy.reshape((-1, 2))
                            self.ax_workspace.scatter(
                                gxy[:, 0], gxy[:, 1],
                                s=10, color='#aa33ff' if self.state.envelope_mode == 'hull16_grid' else '#5555cc',
                                alpha=0.5, zorder=3.5
                            )

                    # Optional endpoint IAABB overlay (xy projection) for diagnostics.
                    if self.state.envelope_mode in ('linkiaabb_grid', 'hull16_grid'):
                        for b in ep_boxes:
                            x0, y0, x1, y1 = b[0], b[1], b[3], b[4]
                            patch = mpatches.Rectangle(
                                (x0, y0), max(0.0, x1 - x0), max(0.0, y1 - y0),
                                linewidth=0.8, edgecolor='#8844cc', facecolor='none',
                                alpha=0.45, zorder=4, linestyle=':'
                            )
                            self.ax_workspace.add_patch(patch)

                    self._last_env_diag = (
                        f"ep={ep_info['source']} ep_us={int(ep_info['ep_time_us'])} "
                        f"env_us={int(env_info['env_time_us'])} vol={float(env_info['volume']):.4f}"
                    )
                except Exception as ex:
                    self._last_env_diag = f"library envelope error: {ex}"

        # ── Bridge pair highlight ─────────────────────────────────────────
        if self.state.active_bridge_pair is not None:
            bfrom_id, bto_id = self.state.active_bridge_pair
            for bid, ec, label in [(bfrom_id, '#cc0000', 'src'), (bto_id, '#0033cc', 'dst')]:
                box = self.state.boxes_created.get(bid) or self.parser.boxes_by_id.get(bid)
                if box is not None:
                    q1, q2, w1, w2 = box.get_rect()
                    rect = mpatches.Rectangle(
                        (q1, q2), w1, w2,
                        linewidth=3.5, edgecolor=ec, facecolor='none',
                        alpha=0.95, zorder=11, linestyle='-'
                    )
                    self.ax_cspace.add_patch(rect)
                    cx = q1 + w1 / 2
                    cy = q2 + w2 / 2
                    self.ax_cspace.text(cx, cy, f'BRG-{label}\n#{bid}',
                                        ha='center', va='center', fontsize=7,
                                        color=ec, fontweight='bold', zorder=12)
            # Draw a line between box seeds if both exist
            box_f = self.state.boxes_created.get(bfrom_id) or self.parser.boxes_by_id.get(bfrom_id)
            box_t = self.state.boxes_created.get(bto_id) or self.parser.boxes_by_id.get(bto_id)
            if box_f is not None and box_t is not None:
                sf, st = box_f.seed, box_t.seed
                if len(sf) >= 2 and len(st) >= 2:
                    self.ax_cspace.annotate(
                        '', xy=(st[0], st[1]), xytext=(sf[0], sf[1]),
                        arrowprops=dict(arrowstyle='->', color='magenta', lw=2.0),
                        zorder=13
                    )

        # Legend
        from matplotlib.lines import Line2D
        legend_elements = [
            Line2D([0], [0], marker='.', color='w', markerfacecolor='#aaaaaa',
                   markersize=8, label='RRT sample'),
            Line2D([0], [0], marker='x', color='#e07000', markersize=8,
                   label='snap seed (FFB)', lw=0),
            Line2D([0], [0], marker='*', color='green', markersize=10, label='start', lw=0),
            Line2D([0], [0], marker='*', color='red', markersize=10, label='goal', lw=0),
            mpatches.Patch(facecolor='#fff3cc', edgecolor='darkorange', label='bridge box'),
        ]
        self.ax_cspace.legend(handles=legend_elements, fontsize=8,
                              loc='lower right', framealpha=0.9)

        # ── Draw workspace ────────────────────────────────────────────────
        # Past snap-seed robot poses (faint orange)
        for snap in self.state.snap_seeds[:-1]:
            p0, p1, p2 = self._fk(snap)
            self.ax_workspace.plot([p0[0], p1[0]], [p0[1], p1[1]],
                                   '-', color='#ffbb44', lw=1.5, alpha=0.3)
            self.ax_workspace.plot([p1[0], p2[0]], [p1[1], p2[1]],
                                   '-', color='#ffbb44', lw=1.5, alpha=0.3)

        # Current robot configuration
        if self.state.current_q is not None and len(self.state.current_q) >= 2:
            p0, p1, p2 = self._fk(self.state.current_q)
            self.ax_workspace.plot([p0[0], p1[0]], [p0[1], p1[1]], '-', color='#1f77b4', lw=4)
            self.ax_workspace.plot([p1[0], p2[0]], [p1[1], p2[1]], '-', color='#ff7f0e', lw=4)
            self.ax_workspace.plot([p0[0], p1[0], p2[0]], [p0[1], p1[1], p2[1]], 'ko', ms=5)
            label = 'snap' if self.state.current_snap is not None else 'q'
            self.ax_workspace.text(
                0.02, 0.98,
                f"{label}=[{self.state.current_q[0]:.3f}, {self.state.current_q[1]:.3f}]",
                transform=self.ax_workspace.transAxes,
                va='top', ha='left', fontsize=10,
                bbox=dict(boxstyle='round', facecolor='white', alpha=0.8, edgecolor='0.4')
            )
            if self._last_env_diag:
                self.ax_workspace.text(
                    0.02, 0.02,
                    self._last_env_diag,
                    transform=self.ax_workspace.transAxes,
                    va='bottom', ha='left', fontsize=8,
                    bbox=dict(boxstyle='round', facecolor='white', alpha=0.8, edgecolor='0.5')
                )

        # Sync log lines (latest + short history).
        i0 = max(0, self.state.current_frame_idx - 5)
        window = self.parser.frames[i0:self.state.current_frame_idx + 1]
        lines = []
        for fr in window:
            lines.append(f"[{fr.idx:04d}] tid={fr.tid:2d} {fr.raw_line}")
        if not lines:
            lines = ['(no frames)']
        log_text = "\n".join(lines)
        self.ax_log.text(
            0.01, 0.95, log_text,
            va='top', ha='left', family='monospace', fontsize=9,
            transform=self.ax_log.transAxes,
            bbox=dict(boxstyle='round', facecolor='#f8f8f8', edgecolor='0.8', alpha=1.0)
        )

        # Draw title with frame info
        frame = self.parser.frames[self.state.current_frame_idx]
        title = f"Frame {self.state.current_frame_idx + 1}/{len(self.parser.frames)} [{frame.kind}]"
        n_samples = len(self.state.rrt_samples)
        n_snaps = len(self.state.snap_seeds)
        bridge_tag = ''
        if self.state.bridge_phase_active:
            if self.state.active_bridge_pair is not None:
                bridge_tag = f' [BRIDGE: {self.state.active_bridge_pair[0]}\u2192{self.state.active_bridge_pair[1]}]'
            else:
                bridge_tag = ' [BRIDGE PHASE]'
        self.fig.suptitle(
            f"{title} | Boxes: {len(self.state.boxes_created)} | "
            f"Samples: {n_samples} | FFB seeds: {n_snaps} | "
            f"env={self.state.envelope_mode} | ep={self.state.endpoint_source} | "
            f"{'[PAUSED]' if self.state.paused else '[PLAYING]'}{bridge_tag}"
        )
        self.fig.canvas.draw_idle()

    def run(self):
        """Run interactive viewer."""
        self.setup_figure()
        self.state.paused = True
        self._update_display()
        print("Controls: Space=play/pause, ->/<-=next/prev, Home/End=jump, q=quit")
        plt.show()

    def export_video(self, output_path: str, fps: int = 10):
        """Export frames as MP4/GIF."""
        self.setup_figure()
        self.state.paused = True
        
        # Try to use imageio for MP4, fallback to imageio for GIF
        try:
            from PIL import Image
            import io
        except ImportError:
            print("ERROR: PIL required for video export. Install with: pip install pillow")
            return

        frames = []
        for i in range(len(self.parser.frames)):
            self.state.current_frame_idx = i
            self._update_display()
            
            # Render to buffer
            buf = io.BytesIO()
            self.fig.savefig(buf, format='png', dpi=100, bbox_inches='tight')
            buf.seek(0)
            frames.append(Image.open(buf).copy())
            
            if (i + 1) % 10 == 0:
                print(f"  Rendered {i + 1}/{len(self.parser.frames)} frames...")

        # Save as GIF
        if output_path.endswith('.gif'):
            frames[0].save(output_path, save_all=True, append_images=frames[1:],
                          duration=1000 // fps, loop=0)
        else:
            # Try MP4 with imageio
            try:
                import imageio
                imageio.mimsave(output_path, frames, fps=fps)
            except ImportError:
                print("Warning: imageio not available. Saving as GIF instead.")
                output_path = output_path.replace('.mp4', '.gif')
                frames[0].save(output_path, save_all=True, append_images=frames[1:],
                              duration=1000 // fps, loop=0)

        print(f"Video saved: {output_path}")
        plt.close(self.fig)


# ─── Main ────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="2D SBF Build Trace Replay Viewer")
    parser.add_argument('logfile', help='Path to 2d_trace_*.log file')
    parser.add_argument('--video', help='Export to video (MP4 or GIF)', default=None)
    parser.add_argument('--fps', type=int, default=10, help='Video FPS (default 10)')
    
    args = parser.parse_args()

    # Parse log
    print(f"Parsing {args.logfile}...")
    log_parser = LogParser(args.logfile)
    # Extract rich metadata from raw [2D-META] line.
    for fr in log_parser.frames:
        if fr.kind == 'metadata':
            log_parser.metadata['limits'] = Viewer2D._parse_limits_from_meta(fr.raw_line)
            log_parser.metadata['obstacles'] = Viewer2D._parse_obstacles_from_meta(fr.raw_line)
            break
    print(f"  Found {len(log_parser.frames)} frames")
    print(f"  Metadata: {log_parser.metadata}")

    # Create viewer
    viewer = Viewer2D(log_parser)

    # Run interactive or export video
    if args.video:
        print(f"Exporting video to {args.video}...")
        viewer.export_video(args.video, fps=args.fps)
    else:
        viewer.run()


if __name__ == '__main__':
    main()
