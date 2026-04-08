"""Compare v3 vs v4_flat (before lazy) vs v4_lazy (after lazy channel + Z4 short-circuit)."""
import pandas as pd, numpy as np

def load(path):
    for enc in ["utf-16", "utf-8-sig", "utf-8"]:
        try:
            return pd.read_csv(path, encoding=enc)
        except Exception:
            continue
    return pd.read_csv(path)

v3  = load("v3_ffb_results.csv")
v4f = load("v4_ffb_results_flat.csv")       # flat-buffer (before lazy)
v4l = load("v4_ffb_results_lazy.csv")       # lazy channel + Z4 short-circuit

NOB, SUCC, NN = 'n_obstacles', 'success', 'n_new_nodes'

print(f"Rows — v3: {len(v3)}  v4_flat: {len(v4f)}  v4_lazy: {len(v4l)}")
print()

# ═══════════════════════════════════════════════════════════
# 1. Wall-clock time & success rate
# ═══════════════════════════════════════════════════════════
for robot in ["iiwa14", "panda"]:
    print(f"═══ {robot} — Wall-clock time (us/query) & success rate ═══")
    hdr = f"{'nobs':>4}  {'v3':>8}  {'v4_flat':>8}  {'v4_lazy':>8}  {'flat/v3':>8}  {'lazy/v3':>8}  {'lazy/flat':>9}  {'v3%':>6}  {'flat%':>6}  {'lazy%':>6}"
    print(hdr)
    for n in sorted(v3[NOB].unique()):
        d3  = v3 [(v3 ['robot']==robot) & (v3 [NOB]==n)]
        df  = v4f[(v4f['robot']==robot) & (v4f[NOB]==n)]
        dl  = v4l[(v4l['robot']==robot) & (v4l[NOB]==n)]
        t3, tf, tl = d3['time_us'].mean(), df['time_us'].mean(), dl['time_us'].mean()
        s3, sf, sl = d3[SUCC].mean()*100, df[SUCC].mean()*100, dl[SUCC].mean()*100
        print(f"{n:>4}  {t3:>8.1f}  {tf:>8.1f}  {tl:>8.1f}  {tf/t3:>8.3f}  {tl/t3:>8.3f}  {tl/tf:>9.3f}  {s3:>5.1f}%  {sf:>5.1f}%  {sl:>5.1f}%")
    print()

# ═══════════════════════════════════════════════════════════
# 2. Median new nodes (tree growth per query)
# ═══════════════════════════════════════════════════════════
for robot in ["iiwa14", "panda"]:
    print(f"═══ {robot} — Median new nodes per query ═══")
    hdr = f"{'nobs':>4}  {'v3':>6}  {'v4_flat':>7}  {'v4_lazy':>7}"
    print(hdr)
    for n in sorted(v3[NOB].unique()):
        d3  = v3 [(v3 ['robot']==robot) & (v3 [NOB]==n)]
        df  = v4f[(v4f['robot']==robot) & (v4f[NOB]==n)]
        dl  = v4l[(v4l['robot']==robot) & (v4l[NOB]==n)]
        print(f"{n:>4}  {d3[NN].median():>6.0f}  {df[NN].median():>7.0f}  {dl[NN].median():>7.0f}")
    print()

# ═══════════════════════════════════════════════════════════
# 3. Per-FK cost (us per new node)
# ═══════════════════════════════════════════════════════════
print("═══ Per-FK cost (us / new_node, median) ═══")
for robot in ["iiwa14", "panda"]:
    for label, df in [("v3", v3), ("v4_flat", v4f), ("v4_lazy", v4l)]:
        sub = df[df['robot'] == robot]
        fk = sub['time_us'] / sub[NN].clip(lower=1)
        print(f"  {robot:>6} {label:>8}: median {fk.median():>6.1f}  mean {fk.mean():>6.1f}")
    print()

# ═══════════════════════════════════════════════════════════
# 4. Summary
# ═══════════════════════════════════════════════════════════
print("═══ v4_lazy vs v3 ═══")
for robot in ["iiwa14", "panda"]:
    for n in sorted(v3[NOB].unique()):
        t3 = v3 [(v3 ['robot']==robot) & (v3 [NOB]==n)]['time_us'].mean()
        tf = v4f[(v4f['robot']==robot) & (v4f[NOB]==n)]['time_us'].mean()
        tl = v4l[(v4l['robot']==robot) & (v4l[NOB]==n)]['time_us'].mean()
        print(f"  {robot:>6} n_obs={n:>2}: lazy/v3={tl/t3:.3f}  flat/v3={tf/t3:.3f}  lazy_speedup_vs_flat={tf/tl:.3f}x")
