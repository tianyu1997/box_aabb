"""Compare v3 vs v4 ForestGrower benchmark results."""
import pandas as pd
import numpy as np
import io

def load_grower_csv(path, version_prefix):
    """Load grower CSV, filtering out internal debug prints."""
    for enc in ["utf-16", "utf-8-sig", "utf-8"]:
        try:
            with open(path, 'r', encoding=enc) as f:
                lines = f.readlines()
            break
        except Exception:
            continue
    # Keep only header and data lines
    header = None
    data_lines = []
    for line in lines:
        line_s = line.strip()
        if line_s.startswith("version,"):
            header = line
        elif line_s.startswith(version_prefix + ','):
            data_lines.append(line)
    return pd.read_csv(io.StringIO(header + ''.join(data_lines)))

v3 = load_grower_csv("v3_grower_results.csv", "v3")
v4 = load_grower_csv("v4_grower_results.csv", "v4")

print(f"Rows — v3: {len(v3)}  v4: {len(v4)}")
print()

# ═══════════════════════════════════════════════════════════
# 1. Overall summary by robot × mode × n_obstacles
# ═══════════════════════════════════════════════════════════
for robot in ["iiwa14", "panda"]:
    for mode in ["Wavefront", "RRT"]:
        print(f"═══ {robot} / {mode} ═══")
        hdr = (f"{'nobs':>4}  {'v3_ms':>8}  {'v4_ms':>8}  {'v4/v3':>6}  "
               f"{'v3_box':>6}  {'v4_box':>6}  "
               f"{'v3_comp':>7}  {'v4_comp':>7}  "
               f"{'v3_sg%':>6}  {'v4_sg%':>6}  "
               f"{'v3_vol':>8}  {'v4_vol':>8}")
        print(hdr)
        for n in sorted(v3['n_obstacles'].unique()):
            d3 = v3[(v3['robot']==robot) & (v3['mode']==mode) & (v3['n_obstacles']==n)]
            d4 = v4[(v4['robot']==robot) & (v4['mode']==mode) & (v4['n_obstacles']==n)]
            
            t3, t4 = d3['grow_time_ms'].mean(), d4['grow_time_ms'].mean()
            b3, b4 = d3['n_boxes'].mean(), d4['n_boxes'].mean()
            c3, c4 = d3['n_components'].mean(), d4['n_components'].mean()
            sg3 = d3['start_goal_connected'].mean() * 100
            sg4 = d4['start_goal_connected'].mean() * 100
            vol3, vol4 = d3['total_volume'].mean(), d4['total_volume'].mean()
            ratio = t4 / t3 if t3 > 0 else float('inf')
            
            print(f"{n:>4}  {t3:>8.1f}  {t4:>8.1f}  {ratio:>6.3f}  "
                  f"{b3:>6.0f}  {b4:>6.0f}  "
                  f"{c3:>7.1f}  {c4:>7.1f}  "
                  f"{sg3:>5.0f}%  {sg4:>5.0f}%  "
                  f"{vol3:>8.3f}  {vol4:>8.3f}")
        print()

# ═══════════════════════════════════════════════════════════
# 2. Per-scene detail (successful runs only: n_boxes > 0)
# ═══════════════════════════════════════════════════════════
print("═══ Per-scene detail (successful runs only, n_boxes > 0) ═══")
for robot in ["iiwa14", "panda"]:
    for mode in ["Wavefront", "RRT"]:
        d3ok = v3[(v3['robot']==robot) & (v3['mode']==mode) & (v3['n_boxes']>0)]
        d4ok = v4[(v4['robot']==robot) & (v4['mode']==mode) & (v4['n_boxes']>0)]
        n3, n4 = len(d3ok), len(d4ok)
        print(f"  {robot:>6} {mode:>10}: v3 {n3} runs ok, v4 {n4} runs ok "
              f"(of {len(v3[(v3['robot']==robot)&(v3['mode']==mode)])} total)")
        if n3 > 0 and n4 > 0:
            t3m, t4m = d3ok['grow_time_ms'].median(), d4ok['grow_time_ms'].median()
            b3m, b4m = d3ok['n_boxes'].median(), d4ok['n_boxes'].median()
            print(f"         median time: v3={t3m:.1f}ms  v4={t4m:.1f}ms  ratio={t4m/t3m:.3f}")
            print(f"         median boxes: v3={b3m:.0f}  v4={b4m:.0f}")
print()

# ═══════════════════════════════════════════════════════════
# 3. Speed per box (ms per box created)
# ═══════════════════════════════════════════════════════════
print("═══ Speed: ms per box (successful runs, median) ═══")
for robot in ["iiwa14", "panda"]:
    for mode in ["Wavefront", "RRT"]:
        for label, df in [("v3", v3), ("v4", v4)]:
            ok = df[(df['robot']==robot) & (df['mode']==mode) & (df['n_boxes']>0)]
            if len(ok) == 0:
                print(f"  {robot:>6} {mode:>10} {label}: no successful runs")
                continue
            ms_per_box = ok['grow_time_ms'] / ok['n_boxes']
            print(f"  {robot:>6} {mode:>10} {label}: median {ms_per_box.median():.3f} ms/box  "
                  f"mean {ms_per_box.mean():.3f} ms/box")
    print()

# ═══════════════════════════════════════════════════════════
# 4. Start-goal connectivity rate
# ═══════════════════════════════════════════════════════════
print("═══ Start-goal connectivity rate ═══")
for robot in ["iiwa14", "panda"]:
    for mode in ["Wavefront", "RRT"]:
        for label, df in [("v3", v3), ("v4", v4)]:
            sub = df[(df['robot']==robot) & (df['mode']==mode)]
            rate = sub['start_goal_connected'].mean() * 100
            # Among runs with boxes
            ok = sub[sub['n_boxes'] > 0]
            rate_ok = ok['start_goal_connected'].mean() * 100 if len(ok) > 0 else 0
            print(f"  {robot:>6} {mode:>10} {label}: "
                  f"overall={rate:.0f}%  among_active={rate_ok:.0f}%")
    print()

# ═══════════════════════════════════════════════════════════
# 5. Raw data dump
# ═══════════════════════════════════════════════════════════
print("═══ Raw table — all runs ═══")
combined = pd.concat([v3, v4], ignore_index=True)
combined = combined.sort_values(['robot', 'mode', 'n_obstacles', 'scene_seed', 'version'])
cols = ['version', 'robot', 'n_obstacles', 'mode', 'scene_seed',
        'grow_time_ms', 'n_boxes', 'n_components', 'start_goal_connected', 'total_volume']
print(combined[cols].to_string(index=False))
