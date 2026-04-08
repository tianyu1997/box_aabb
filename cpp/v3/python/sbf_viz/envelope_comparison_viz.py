"""
Multi-method envelope comparison viewer — Plotly 3D with toggleable layers.

Shows all envelope methods simultaneously with interactive checkboxes:
  1. Full AABB (per-link)          — wireframe boxes
  2. Sub-AABB (n_sub subdivisions) — wireframe boxes
  3. Voxel Hull-16                 — scatter points (tightest)
  4. Voxel Sub-AABB                — scatter points
  5. Voxel Full-AABB               — scatter points (coarsest)
  + Robot arm at box center

Each method is a "legendgroup" so clicking its legend item toggles all
traces belonging to that method.
"""

import json
import re
import numpy as np
import plotly.graph_objects as go
from typing import Optional, Dict, Any, List

from .load_data import AABB


def _parse_rgba(s: str):
    """Parse 'rgba(r,g,b,a)' -> ('rgb(r,g,b)', float_a).  Passthrough for rgb."""
    m = re.match(r'rgba\((\d+),(\d+),(\d+),([\d.]+)\)', s.replace(' ', ''))
    if m:
        return f'rgb({m.group(1)},{m.group(2)},{m.group(3)})', float(m.group(4))
    return s, 1.0


# ── Colour scheme per method ─────────────────────────────────────────────────
# Each method gets a distinct hue: (fill_rgba, wire_rgb, scatter_rgba)
METHOD_STYLES = {
    "full_aabb": {
        "label":   "Full AABB",
        "fill":    "rgba(230,25,25,0.12)",      # vivid red
        "wire":    "rgb(230,25,25)",
        "scatter": "rgba(230,25,25,0.55)",
        "order":   1,
    },
    "sub_aabb": {
        "label":   "Sub-AABB",
        "fill":    "rgba(0,90,255,0.12)",        # electric blue
        "wire":    "rgb(0,90,255)",
        "scatter": "rgba(0,90,255,0.55)",
        "order":   2,
    },
    "voxel_hull16": {
        "label":   "Voxel Hull-16",
        "fill":    "rgba(0,190,0,0.45)",         # bright green — most opaque
        "wire":    "rgb(0,190,0)",
        "scatter": "rgba(0,190,0,0.60)",
        "diff":    "rgba(0,220,0,0.70)",         # set-difference colour
        "shrink":  0.88,
        "order":   3,
    },
    "voxel_sub_aabb": {
        "label":   "Voxel Sub-AABB",
        "fill":    "rgba(255,160,0,0.35)",       # bright orange — medium
        "wire":    "rgb(255,160,0)",
        "scatter": "rgba(255,160,0,0.60)",
        "diff":    "rgba(255,160,0,0.60)",
        "shrink":  0.88,
        "order":   4,
    },
    "voxel_full_aabb": {
        "label":   "Voxel Full-AABB",
        "fill":    "rgba(170,0,220,0.22)",       # vivid purple — most transparent
        "wire":    "rgb(170,0,220)",
        "scatter": "rgba(170,0,220,0.60)",
        "diff":    "rgba(170,0,220,0.45)",
        "shrink":  0.88,
        "order":   5,
    },
    "analytical_sub_aabb": {
        "label":   "Analytical Sub-AABB",
        "fill":    "rgba(0,200,200,0.14)",       # teal
        "wire":    "rgb(0,200,200)",
        "scatter": "rgba(0,200,200,0.55)",
        "order":   6,
    },
    "voxel_analytical": {
        "label":   "Voxel Analytical",
        "fill":    "rgba(255,60,160,0.35)",      # hot pink
        "wire":    "rgb(255,60,160)",
        "scatter": "rgba(255,60,160,0.60)",
        "diff":    "rgba(255,60,160,0.60)",
        "shrink":  0.88,
        "order":   7,
    },
    "voxel_hull16_analytical": {
        "label":   "Hull-16 ∩ Analytical",
        "fill":    "rgba(255,215,0,0.50)",       # gold — tightest, most opaque
        "wire":    "rgb(255,215,0)",
        "scatter": "rgba(255,215,0,0.70)",
        "diff":    "rgba(255,215,0,0.70)",
        "shrink":  0.90,
        "order":   8,
    },
}


# ── AABB trace helpers ───────────────────────────────────────────────────────

def _aabb_mesh(lo, hi, color: str, name: str,
               legendgroup: str, showlegend: bool = True) -> go.Mesh3d:
    x0, y0, z0 = lo
    x1, y1, z1 = hi
    vx = [x0, x0, x1, x1, x0, x0, x1, x1]
    vy = [y0, y1, y1, y0, y0, y1, y1, y0]
    vz = [z0, z0, z0, z0, z1, z1, z1, z1]
    i = [0, 0, 0, 0, 4, 4, 2, 2, 0, 0, 1, 1]
    j = [1, 2, 4, 5, 5, 6, 3, 7, 1, 3, 2, 6]
    k = [2, 3, 5, 6, 6, 7, 7, 6, 5, 4, 6, 5]
    return go.Mesh3d(
        x=vx, y=vy, z=vz, i=i, j=j, k=k,
        color=color, flatshading=True,
        name=name, legendgroup=legendgroup,
        showlegend=showlegend,
    )


def _aabb_wireframe(lo, hi, color: str, legendgroup: str,
                    width: float = 2) -> go.Scatter3d:
    x0, y0, z0 = lo
    x1, y1, z1 = hi
    nan = None
    xs = [x0,x1,nan, x0,x1,nan, x0,x1,nan, x0,x1,nan,
          x0,x0,nan, x1,x1,nan, x1,x1,nan, x0,x0,nan,
          x0,x0,nan, x1,x1,nan, x1,x1,nan, x0,x0]
    ys = [y0,y0,nan, y1,y1,nan, y0,y0,nan, y1,y1,nan,
          y0,y1,nan, y0,y1,nan, y0,y1,nan, y0,y1,nan,
          y0,y0,nan, y0,y0,nan, y1,y1,nan, y1,y1]
    zs = [z0,z0,nan, z0,z0,nan, z1,z1,nan, z1,z1,nan,
          z0,z0,nan, z0,z0,nan, z1,z1,nan, z1,z1,nan,
          z0,z1,nan, z0,z1,nan, z0,z1,nan, z0,z1]
    return go.Scatter3d(
        x=xs, y=ys, z=zs,
        mode="lines",
        line=dict(color=color, width=width),
        name="", legendgroup=legendgroup,
        showlegend=False,
        connectgaps=False,
    )


# ── Voxel cube-mesh helpers ──────────────────────────────────────────────────

def _voxel_cubes_mesh(centres: np.ndarray, delta: float, shrink: float,
                      color: str, name: str, legendgroup: str,
                      showlegend: bool = True,
                      visible: bool = True,
                      opacity: float | None = None) -> "go.Mesh3d | None":
    """Build ONE Mesh3d trace containing one cube per voxel centre.

    Unlike Scatter3d markers, cubes have proper 3D faces / lighting,
    making overlapping regions far easier to perceive.  A *shrink*
    factor < 1.0 leaves small gaps between cubes for extra clarity.

    If *opacity* is None, the alpha channel is parsed from *color*
    (rgba string) and set as a separate Mesh3d ``opacity`` property
    so that JS sliders can dynamically adjust it at runtime.
    """
    n = len(centres)
    if n == 0:
        return None
    half = delta * shrink * 0.5

    # Parse colour -> (rgb_str, alpha)
    rgb_str, alpha = _parse_rgba(color)
    if opacity is not None:
        alpha = opacity

    # 8 corner offsets of a cube centred at the origin
    cv = np.array([
        [-1, -1, -1], [+1, -1, -1], [+1, +1, -1], [-1, +1, -1],
        [-1, -1, +1], [+1, -1, +1], [+1, +1, +1], [-1, +1, +1],
    ], dtype=np.float64) * half                        # (8, 3)

    # 12 triangles — 2 per face × 6 faces
    ti = np.array([0, 0, 4, 4, 0, 0, 1, 1, 0, 0, 3, 3], dtype=np.int32)
    tj = np.array([1, 2, 5, 6, 1, 5, 2, 6, 4, 3, 4, 7], dtype=np.int32)
    tk = np.array([2, 3, 6, 7, 5, 4, 6, 5, 3, 7, 7, 4], dtype=np.int32)

    # Broadcast: centres (n,1,3) + cv (1,8,3) → (n,8,3) → (n*8, 3)
    all_v = (centres[:, None, :] + cv[None, :, :]).reshape(-1, 3)

    # Offset triangle indices per cube
    off = (np.arange(n, dtype=np.int32) * 8)[:, None]  # (n, 1)
    ai = (ti[None, :] + off).ravel()
    aj = (tj[None, :] + off).ravel()
    ak = (tk[None, :] + off).ravel()

    return go.Mesh3d(
        x=all_v[:, 0], y=all_v[:, 1], z=all_v[:, 2],
        i=ai, j=aj, k=ak,
        color=rgb_str, opacity=alpha, flatshading=True,
        name=name, legendgroup=legendgroup,
        showlegend=showlegend,
        visible=visible,
    )


def _voxel_set_keys(centres: np.ndarray, delta: float) -> set:
    """Snap voxel centres to integer grid keys for set operations."""
    if len(centres) == 0:
        return set()
    return set(map(tuple, np.round(centres / delta).astype(np.int64)))


def _keys_to_centres(keys, delta: float) -> np.ndarray:
    """Convert integer grid keys back to world-space centres."""
    if not keys:
        return np.empty((0, 3), dtype=np.float64)
    return np.array(sorted(keys), dtype=np.float64) * delta


# ── Custom JS slider builder ─────────────────────────────────────────────

def _build_slider_html(voxel_trace_indices: List[int]) -> str:
    """Return HTML+JS snippet for opacity and cube-size range sliders.

    The snippet creates two range-input sliders fixed at the bottom-right
    corner of the page.  They modify Plotly trace properties in-place via
    ``Plotly.restyle``.

    *   **Opacity** — directly restyles ``opacity`` on each voxel Mesh3d.
    *   **Cube size** — rescales every vertex around its cube centre
        (groups of 8 vertices) and restyles ``x`` / ``y`` / ``z``.
    """
    idx_js = json.dumps(voxel_trace_indices)
    return f"""
<div id="sbf-sliders" style="
  position:fixed; bottom:12px; right:12px; z-index:9999;
  background:rgba(255,255,255,0.92); border:1px solid #bbb;
  border-radius:8px; padding:10px 16px; font-family:sans-serif;
  font-size:13px; box-shadow:0 2px 8px rgba(0,0,0,0.15);
  display:flex; flex-direction:column; gap:6px; min-width:220px;
">
  <b style="margin-bottom:2px">Voxel Controls</b>
  <label>Opacity: <span id="sbf-op-val">0.40</span>
    <input id="sbf-op" type="range" min="0.05" max="1" step="0.05" value="0.40"
           style="width:140px; vertical-align:middle">
  </label>
  <label>Cube size: <span id="sbf-sz-val">1.00</span>
    <input id="sbf-sz" type="range" min="0.3" max="1.5" step="0.05" value="1.00"
           style="width:140px; vertical-align:middle">
  </label>
</div>
<script>
(function() {{
  var VIDX = {idx_js};

  /* ---- poll until Plotly has finished rendering ---- */
  var _attempts = 0;
  function _tryInit() {{
    _attempts++;
    var gd = document.querySelector('.js-plotly-plot') ||
             document.querySelector('.plotly-graph-div');
    if (!gd || !gd.data || !gd.data.length) {{
      if (_attempts < 50) setTimeout(_tryInit, 200);
      return;
    }}
    _initSliders(gd);
  }}

  function _initSliders(gd) {{
    /* ---- store original vertex arrays (safe copy via Array.from) ---- */
    var orig = {{}};
    VIDX.forEach(function(i) {{
      var d = gd.data[i];
      if (d && d.x) {{
        try {{
          orig[i] = {{x: Array.from(d.x), y: Array.from(d.y), z: Array.from(d.z)}};
        }} catch(e) {{}}
      }}
    }});

    /* ---- opacity slider ---- */
    var opSlider = document.getElementById('sbf-op');
    var opLabel  = document.getElementById('sbf-op-val');
    function onOpacity() {{
      var v = parseFloat(opSlider.value);
      opLabel.textContent = v.toFixed(2);
      VIDX.forEach(function(i) {{
        Plotly.restyle(gd, {{opacity: v}}, [i]);
      }});
    }}
    opSlider.addEventListener('input', onOpacity);
    opSlider.addEventListener('change', onOpacity);

    /* ---- cube-size slider ---- */
    var szSlider = document.getElementById('sbf-sz');
    var szLabel  = document.getElementById('sbf-sz-val');
    function onSize() {{
      var s = parseFloat(szSlider.value);
      szLabel.textContent = s.toFixed(2);
      VIDX.forEach(function(i) {{
        var o = orig[i];
        if (!o) return;
        var n = o.x.length;
        var nx = new Array(n), ny = new Array(n), nz = new Array(n);
        for (var c = 0; c < n; c += 8) {{
          var cx=0, cy=0, cz=0;
          for (var v=0; v<8; v++) {{ cx+=o.x[c+v]; cy+=o.y[c+v]; cz+=o.z[c+v]; }}
          cx/=8; cy/=8; cz/=8;
          for (var v=0; v<8; v++) {{
            nx[c+v] = cx + s*(o.x[c+v]-cx);
            ny[c+v] = cy + s*(o.y[c+v]-cy);
            nz[c+v] = cz + s*(o.z[c+v]-cz);
          }}
        }}
        Plotly.restyle(gd, {{x:[nx], y:[ny], z:[nz]}}, [i]);
      }});
    }}
    szSlider.addEventListener('input', onSize);
    szSlider.addEventListener('change', onSize);
  }}

  /* ---- kick off: try immediately, then poll ---- */
  if (document.readyState === 'complete') {{
    setTimeout(_tryInit, 300);
  }} else {{
    window.addEventListener('load', function() {{ setTimeout(_tryInit, 300); }});
  }}
}})();
</script>
"""


# ── Main comparison viewer ───────────────────────────────────────────────────

def load_envelope_comparison(path: str) -> Dict[str, Any]:
    """Load the envelope_comparison.json file."""
    with open(path) as f:
        return json.load(f)


def plot_envelope_comparison(data: Dict[str, Any],
                             title: str = "Envelope Method Comparison",
                             show: bool = True,
                             save_html: Optional[str] = None,
                             methods_filter: Optional[List[str]] = None) -> go.Figure:
    """
    Create an interactive figure comparing envelope methods.

    Each method's traces share a legendgroup, so clicking the legend
    entry toggles ALL traces of that method on/off.

    Parameters
    ----------
    data : dict
        Loaded from envelope_comparison.json.
    methods_filter : list of str, optional
        If given, only include methods whose key is in this list.
        When *None* (default) all methods are shown.
    """
    fig = go.Figure()
    all_methods = data.get("methods", {})
    if methods_filter is not None:
        methods = {k: v for k, v in all_methods.items() if k in methods_filter}
    else:
        methods = all_methods

    # Track which trace indices belong to each method (for button visibility)
    method_trace_ranges: Dict[str, List[int]] = {}
    trace_idx = 0

    # Track ALL voxel Mesh3d trace indices (for JS sliders)
    voxel_mesh_trace_indices: List[int] = []

    # Collect voxel data for set-difference (Δ) visualisation
    voxel_data_collected: Dict[str, tuple] = {}  # key → (centres, delta)

    # ── Robot arm ──
    arm = data.get("robot_arm", {})
    arm_start = trace_idx
    if arm and "link_positions" in arm:
        pos = np.array(arm["link_positions"])
        fig.add_trace(go.Scatter3d(
            x=pos[:, 0], y=pos[:, 1], z=pos[:, 2],
            mode="lines+markers",
            line=dict(color="#333333", width=6),
            marker=dict(size=4, color="#333333"),
            name="Robot Arm",
            legendgroup="robot_arm",
        ))
        trace_idx += 1
    method_trace_ranges["robot_arm"] = list(range(arm_start, trace_idx))

    # ── Process each method (sorted by order) ──
    sorted_methods = sorted(methods.items(),
                            key=lambda kv: METHOD_STYLES.get(kv[0], {}).get("order", 99))

    for method_key, method_data in sorted_methods:
        style = METHOD_STYLES.get(method_key, {
            "label": method_key, "fill": "rgba(128,128,128,0.1)",
            "wire": "rgb(128,128,128)", "scatter": "rgba(128,128,128,0.4)",
        })
        mtype = method_data.get("type", "")
        label = method_data.get("label", style.get("label", method_key))
        group = method_key
        start = trace_idx

        if mtype == "aabb":
            aabbs = method_data.get("aabbs", [])
            n = len(aabbs)
            first = True
            for ab in aabbs:
                lo = np.array(ab["lo"])
                hi = np.array(ab["hi"])
                link = ab.get("link", 0)
                seg  = ab.get("seg", -1)
                sub_label = f"{label} [L{link}]"
                if seg >= 0:
                    sub_label = f"{label} [L{link}.{seg}]"

                # Filled mesh
                fig.add_trace(_aabb_mesh(
                    lo, hi, style["fill"],
                    name=f"{label} ({n} boxes)" if first else "",
                    legendgroup=group,
                    showlegend=first,
                ))
                trace_idx += 1

                # Wireframe
                fig.add_trace(_aabb_wireframe(
                    lo, hi, style["wire"], legendgroup=group, width=1.5,
                ))
                trace_idx += 1
                first = False

        elif mtype == "voxel":
            centres = np.array(method_data.get("centres", []))
            n_occ = method_data.get("n_occupied", len(centres))
            delta = method_data.get("delta", 0.02)

            if len(centres) > 0:
                voxel_data_collected[method_key] = (centres, delta)
                shrink = style.get("shrink", 0.88)
                mesh = _voxel_cubes_mesh(
                    centres, delta, shrink, style["fill"],
                    name=f"{label} ({n_occ:,} cubes)",
                    legendgroup=group,
                )
                if mesh is not None:
                    fig.add_trace(mesh)
                    voxel_mesh_trace_indices.append(trace_idx)
                    trace_idx += 1

        method_trace_ranges[group] = list(range(start, trace_idx))

    # ── Set-difference voxel traces (hidden by default) ──────────────────
    # Each method shows ONLY voxels NOT in any tighter method, completely
    # eliminating overlap / colour mixing.
    _VOXEL_ORDER = ["voxel_hull16_analytical", "voxel_hull16", "voxel_analytical", "voxel_sub_aabb", "voxel_full_aabb"]
    diff_trace_ranges: Dict[str, List[int]] = {}
    tighter_union: set = set()
    for mk in _VOXEL_ORDER:
        if mk not in voxel_data_collected:
            continue
        centres_v, delta_v = voxel_data_collected[mk]
        keys = _voxel_set_keys(centres_v, delta_v)
        unique_keys = keys - tighter_union
        tighter_union |= keys
        if not unique_keys:
            continue
        unique_centres = _keys_to_centres(unique_keys, delta_v)
        st = METHOD_STYLES.get(mk, {})
        diff_color = st.get("diff", st.get("fill", "rgba(128,128,128,0.4)"))
        diff_label = st.get("label", mk)
        diff_mesh = _voxel_cubes_mesh(
            unique_centres, delta_v, 0.92, diff_color,
            name=f"{diff_label} \u0394 ({len(unique_keys):,})",
            legendgroup=f"{mk}_diff",
            visible=False,
        )
        if diff_mesh is not None:
            ds = trace_idx
            fig.add_trace(diff_mesh)
            voxel_mesh_trace_indices.append(trace_idx)
            trace_idx += 1
            diff_trace_ranges[mk] = list(range(ds, trace_idx))

    total_traces = trace_idx

    # ── Build toggle buttons ──
    # Each button toggles one method; also "All On" and "All Off"
    buttons = []

    # All On (hide Δ traces which are an alternative view)
    all_vis = [True] * total_traces
    for _dr in diff_trace_ranges.values():
        for _i in _dr:
            all_vis[_i] = False
    buttons.append(dict(
        label="✓ All",
        method="update",
        args=[{"visible": all_vis}],
    ))

    # Robot arm only
    robot_vis = [False] * total_traces
    for i in method_trace_ranges.get("robot_arm", []):
        robot_vis[i] = True
    buttons.append(dict(
        label="Arm Only",
        method="update",
        args=[{"visible": robot_vis}],
    ))

    # Per-method toggle presets: arm + one method
    for method_key, _ in sorted_methods:
        style = METHOD_STYLES.get(method_key, {})
        label = style.get("label", method_key)
        vis = [False] * total_traces
        # Always show arm
        for i in method_trace_ranges.get("robot_arm", []):
            vis[i] = True
        for i in method_trace_ranges.get(method_key, []):
            vis[i] = True
        buttons.append(dict(
            label=label,
            method="update",
            args=[{"visible": vis}],
        ))

    # AABB methods only (arm + full_aabb + sub_aabb)
    aabb_vis = [False] * total_traces
    for i in method_trace_ranges.get("robot_arm", []):
        aabb_vis[i] = True
    for i in method_trace_ranges.get("full_aabb", []):
        aabb_vis[i] = True
    for i in method_trace_ranges.get("sub_aabb", []):
        aabb_vis[i] = True
    buttons.append(dict(
        label="AABBs Only",
        method="update",
        args=[{"visible": aabb_vis}],
    ))

    # Voxel methods only
    voxel_vis = [False] * total_traces
    for i in method_trace_ranges.get("robot_arm", []):
        voxel_vis[i] = True
    for mk in ["voxel_hull16", "voxel_sub_aabb", "voxel_full_aabb"]:
        for i in method_trace_ranges.get(mk, []):
            voxel_vis[i] = True
    buttons.append(dict(
        label="Voxels Only",
        method="update",
        args=[{"visible": voxel_vis}],
    ))

    # Voxel set-difference view — each voxel rendered exactly ONCE
    if diff_trace_ranges:
        diff_vis = [False] * total_traces
        for i in method_trace_ranges.get("robot_arm", []):
            diff_vis[i] = True
        for _idxs in diff_trace_ranges.values():
            for i in _idxs:
                diff_vis[i] = True
        buttons.append(dict(
            label="Voxels \u0394",
            method="update",
            args=[{"visible": diff_vis}],
        ))

    # ── Summary annotation ──
    summary_lines = []
    for method_key, method_data in sorted_methods:
        mtype = method_data.get("type", "")
        label = method_data.get("label", method_key)
        if mtype == "aabb":
            n = len(method_data.get("aabbs", []))
            summary_lines.append(f"{label}: {n} boxes")
        elif mtype == "voxel":
            n = method_data.get("n_occupied", 0)
            summary_lines.append(f"{label}: {n:,} voxels")

    robot_name = data.get("robot_name", "Robot")
    n_joints = data.get("n_joints", "?")
    summary_text = (f"{robot_name} ({n_joints}-DOF)  |  " +
                    "  |  ".join(summary_lines))

    fig.update_layout(
        title=dict(text=f"{title}<br><sub>{summary_text}</sub>"),
        scene=dict(
            xaxis_title="X (m)",
            yaxis_title="Y (m)",
            zaxis_title="Z (m)",
            aspectmode="data",
        ),
        legend=dict(
            x=0.01, y=0.99,
            bgcolor="rgba(255,255,255,0.8)",
            bordercolor="rgba(0,0,0,0.3)",
            borderwidth=1,
            itemclick="toggle",
            itemdoubleclick="toggleothers",
        ),
        margin=dict(l=0, r=0, t=80, b=0),
        updatemenus=[
            dict(
                type="buttons",
                direction="left",
                x=0.01, y=-0.02,
                xanchor="left", yanchor="top",
                bgcolor="rgba(240,240,240,0.9)",
                bordercolor="rgba(0,0,0,0.3)",
                borderwidth=1,
                font=dict(size=11),
                buttons=buttons,
            ),
        ],
    )

    if save_html:
        html_str = fig.to_html(full_html=True, include_plotlyjs=True)
        slider_snippet = _build_slider_html(voxel_mesh_trace_indices)
        html_str = html_str.replace('</body>', slider_snippet + '\n</body>')
        with open(save_html, 'w', encoding='utf-8') as f:
            f.write(html_str)
    if show:
        fig.show()
    return fig
