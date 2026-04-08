"""Compare v3 vs v4_lazy vs v4_ifk_fast (IFK fast-path + partial inheritance)."""
import pandas as pd, numpy as np

def load(path):
    for enc in ["utf-16", "utf-8-sig", "utf-8"]:
        try:
            return pd.read_csv(path, encoding=enc)
        except Exception:
            continue
    return pd.read_csv(path)

v3  = load("v3_ffb_results.csv")
v4l = load("v4_ffb_results_lazy.csv")        # lazy channel + Z4
v4i = load("v4_ffb_results_ifk_fast.csv")    # IFK fast-path + partial inheritance

NOB, SUCC, NN = 'n_obstacles', 'success', 'n_new_nodes'

print(f"Rows — v3: {len(v3)}  v4_lazy: {len(v4l)}  v4_ifk: {len(v4i)}")
print()

# ═══════════════════════════════════════════════════════════
# 1. Wall-clock time & success rate
# ═══════════════════════════════════════════════════════════
for robot in ["iiwa14", "panda"]:
    print(f"═══ {robot} — Wall-clock time (us/query) & success rate ═══")
    hdr = (f"{'nobs':>4}  {'v3':>8}  {'v4_lazy':>8}  {'v4_ifk':>8}  "
           f"{'lazy/v3':>8}  {'ifk/v3':>7}  {'ifk/lazy':>9}  "
           f"{'v3%':>6}  {'lazy%':>6}  {'ifk%':>6}")
    print(hdr)
    for n in sorted(v3[NOB].unique()):
        d3  = v3 [(v3 ['robot']==robot) & (v3 [NOB]==n)]
        dl  = v4l[(v4l['robot']==robot) & (v4l[NOB]==n)]
        di  = v4i[(v4i['robot']==robot) & (v4i[NOB]==n)]
        t3, tl, ti = d3['time_us'].mean(), dl['time_us'].mean(), di['time_us'].mean()
        s3, sl, si = d3[SUCC].mean()*100, dl[SUCC].mean()*100, di[SUCC].mean()*100
        print(f"{n:>4}  {t3:>8.1f}  {tl:>8.1f}  {ti:>8.1f}  "
              f"{tl/t3:>8.3f}  {ti/t3:>7.3f}  {ti/tl:>9.3f}  "
              f"{s3:>5.1f}%  {sl:>5.1f}%  {si:>5.1f}%")
    print()

# ═══════════════════════════════════════════════════════════
# 2. Median new nodes per query
# ═══════════════════════════════════════════════════════════
for robot in ["iiwa14", "panda"]:
    print(f"═══ {robot} — Median new nodes per query ═══")
    hdr = f"{'nobs':>4}  {'v3':>6}  {'v4_lazy':>7}  {'v4_ifk':>7}"
    print(hdr)
    for n in sorted(v3[NOB].unique()):
        d3  = v3 [(v3 ['robot']==robot) & (v3 [NOB]==n)]
        dl  = v4l[(v4l['robot']==robot) & (v4l[NOB]==n)]
        di  = v4i[(v4i['robot']==robot) & (v4i[NOB]==n)]
        print(f"{n:>4}  {d3[NN].median():>6.0f}  {dl[NN].median():>7.0f}  {di[NN].median():>7.0f}")
    print()

# ═══════════════════════════════════════════════════════════
# 3. Per-FK cost (us per new node)
# ═══════════════════════════════════════════════════════════
print("═══ Per-FK cost (us / new_node, median) ═══")
for robot in ["iiwa14", "panda"]:
    for label, df in [("v3", v3), ("v4_lazy", v4l), ("v4_ifk", v4i)]:
        sub = df[df['robot'] == robot]
        fk = sub['time_us'] / sub[NN].clip(lower=1)
        print(f"  {robot:>6} {label:>8}: median {fk.median():>6.1f}  mean {fk.mean():>6.1f}")
    print()

# ═══════════════════════════════════════════════════════════
# 4. Summary
# ═══════════════════════════════════════════════════════════
print("═══ Summary: v4_ifk vs v3 & v4_lazy ═══")
for robot in ["iiwa14", "panda"]:
    for n in sorted(v3[NOB].unique()):
        t3 = v3 [(v3 ['robot']==robot) & (v3 [NOB]==n)]['time_us'].mean()
        tl = v4l[(v4l['robot']==robot) & (v4l[NOB]==n)]['time_us'].mean()
        ti = v4i[(v4i['robot']==robot) & (v4i[NOB]==n)]['time_us'].mean()
        pct_vs_v3   = (1 - ti/t3) * 100
        pct_vs_lazy = (1 - ti/tl) * 100
        print(f"  {robot:>6} n_obs={n:>2}: ifk/v3={ti/t3:.3f}  ifk/lazy={ti/tl:.3f}  "
              f"speedup_vs_v3={pct_vs_v3:+.1f}%  speedup_vs_lazy={pct_vs_lazy:+.1f}%")
