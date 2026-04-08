"""Analyze phase_compare_h6e.csv results — compare with phase_compare_h6.csv (pre H6-E)"""
import csv
from collections import defaultdict

def load_csv(fname):
    rows = []
    with open(fname, encoding='utf-8-sig') as f:
        rd = csv.DictReader(f)
        for r in rd:
            rows.append({k: float(v) for k, v in r.items()})
    return rows

def avg_by_width(rows, key):
    sums = defaultdict(lambda: [0.0, 0])
    for r in rows:
        w = round(r['width'], 3)
        sums[w][0] += r[key]
        sums[w][1] += 1
    return {w: s[0]/s[1] for w, s in sorted(sums.items())}

# Load new (H6-E) data
new = load_csv('phase_compare_h6e.csv')

# Load old (H6) data for comparison
try:
    old = load_csv('phase_compare_h6.csv')
    has_old = True
except:
    has_old = False

widths = sorted(set(round(r['width'], 3) for r in new))

# Phase keys
phases = ['A', 'B', 'C', 'D', 'E', 'F', 'G']

print("=== Per-width GCPC phase breakdown (H6-E) ===")
print(f"{'width':>6} | {'Total':>7} | " + " | ".join(f"{p:>6}" for p in phases) + " | Anal")
for w in widths:
    trials = [r for r in new if round(r['width'], 3) == w]
    n = len(trials)
    tot = sum(r['gcpc_total_ms'] for r in trials) / n
    anal = sum(r['anal_total_ms'] for r in trials) / n
    ph = {p: sum(r[f'gcpc_{p}_ms'] for r in trials) / n for p in phases}
    line = f"{w:6.3f} | {tot:7.2f} | " + " | ".join(f"{ph[p]:6.3f}" for p in phases) + f" | {anal:6.2f}"
    ratio = tot / anal if anal > 0 else 0
    line += f"  ratio={ratio:.3f}"
    if ratio > 1.0:
        line += " *** SLOWER"
    print(line)

# Overall averages
print("\n=== Overall Phase averages ===")
for p in phases:
    avg_new = sum(r[f'gcpc_{p}_ms'] for r in new) / len(new)
    print(f"  Phase {p}: {avg_new:.3f} ms")

tot_new = sum(r['gcpc_total_ms'] for r in new) / len(new)
anal_new = sum(r['anal_total_ms'] for r in new) / len(new)
print(f"\n  GCPC total: {tot_new:.3f} ms")
print(f"  Anal total: {anal_new:.3f} ms")
print(f"  Ratio: {tot_new/anal_new:.3f}")

if has_old:
    print("\n=== Comparison: H6 vs H6-E ===")
    print(f"{'width':>6} | {'GCPC_old':>8} {'GCPC_new':>8} {'Anal_old':>8} {'Anal_new':>8} | {'ratio_old':>9} {'ratio_new':>9} | {'A_old':>6} {'A_new':>6}")
    for w in widths:
        trials_new = [r for r in new if round(r['width'], 3) == w]
        trials_old = [r for r in old if round(r['width'], 3) == w]
        if not trials_old:
            continue
        gcpc_new = sum(r['gcpc_total_ms'] for r in trials_new) / len(trials_new)
        gcpc_old = sum(r['gcpc_total_ms'] for r in trials_old) / len(trials_old)
        anal_new = sum(r['anal_total_ms'] for r in trials_new) / len(trials_new)
        anal_old = sum(r['anal_total_ms'] for r in trials_old) / len(trials_old)
        a_new = sum(r['gcpc_A_ms'] for r in trials_new) / len(trials_new)
        a_old = sum(r['gcpc_A_ms'] for r in trials_old) / len(trials_old)
        r_old = gcpc_old / anal_old if anal_old > 0 else 0
        r_new = gcpc_new / anal_new if anal_new > 0 else 0
        note = ""
        if r_new > 1.0:
            note = " *** SLOWER"
        print(f"{w:6.3f} | {gcpc_old:8.2f} {gcpc_new:8.2f} {anal_old:8.2f} {anal_new:8.2f} | {r_old:9.3f} {r_new:9.3f} | {a_old:6.3f} {a_new:6.3f}{note}")

# Also check n_cache (number of Phase A cache hits)
print("\n=== Phase A cache hits per width ===")
for w in widths:
    trials = [r for r in new if round(r['width'], 3) == w]
    avg_cache = sum(r['gcpc_n_cache'] for r in trials) / len(trials)
    avg_a_ms = sum(r['gcpc_A_ms'] for r in trials) / len(trials)
    print(f"  w={w:.3f}: avg_cache={avg_cache:.0f}, A={avg_a_ms:.3f}ms")
