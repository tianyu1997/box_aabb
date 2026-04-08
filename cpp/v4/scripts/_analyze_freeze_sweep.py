import csv
from collections import defaultdict

f = open('exp_freeze_depth_sweep_output.csv', encoding='utf-16')
rows = list(csv.DictReader(f))
print(f'Total rows: {len(rows)}')

# Group by (source, width_frac, freeze_depth) and compute mean ratio, speedup
stats = defaultdict(lambda: {'ratio':[], 'speedup':[], 'vol':[], 'time_ms':[]})
for r in rows:
    key = (r['source'], r['width_frac'], int(r['freeze_depth']))
    stats[key]['ratio'].append(float(r['ratio']))
    stats[key]['speedup'].append(float(r['speedup']))
    stats[key]['vol'].append(float(r['vol']))
    stats[key]['time_ms'].append(float(r['time_ms']))

def avg(lst): return sum(lst)/len(lst) if lst else 0
def med(lst): 
    s = sorted(lst)
    n = len(s)
    return s[n//2] if n else 0

# ===== TABLE 1: Full sweep results =====
print()
hdr = f"{'Source':<12} {'Width':>6} {'FD':>3} {'MeanRatio':>10} {'MedRatio':>10} {'MeanSpeedup':>12} {'MedSpeedup':>11} {'MeanTime_ms':>12}"
print(hdr)
print('-' * len(hdr))

for source in ['IFK', 'CritSample', 'Analytical', 'GCPC']:
    for w in ['0.050', '0.100', '0.200', '0.300', '0.500', '0.700', '1.000']:
        for fd in range(7):
            key = (source, w, fd)
            if key in stats:
                d = stats[key]
                mr = avg(d["ratio"])
                mdr = med(d["ratio"])
                ms = avg(d["speedup"])
                mds = med(d["speedup"])
                mt = avg(d["time_ms"])
                print(f'{source:<12} {w:>6} {fd:>3} {mr:>10.4f} {mdr:>10.4f} {ms:>12.4f} {mds:>11.4f} {mt:>12.4f}')
        print()

# ===== TABLE 2: Optimal freeze depth per (source, width) =====
# Define "acceptable" as ratio <= threshold
for threshold in [1.5, 2.0, 3.0]:
    print(f"\n===== Optimal freeze depth (ratio threshold <= {threshold}) =====")
    print(f"{'Source':<12} {'Width':>6} {'OptFD':>6} {'Ratio@Opt':>10} {'Speedup@Opt':>12} {'Time@Opt_ms':>12}")
    print('-' * 70)
    
    for source in ['IFK', 'CritSample', 'Analytical', 'GCPC']:
        for w in ['0.050', '0.100', '0.200', '0.300', '0.500', '0.700', '1.000']:
            best_fd = 0
            best_ratio = 1.0
            best_speedup = 1.0
            best_time = 0.0
            for fd in range(7):
                key = (source, w, fd)
                if key not in stats: continue
                d = stats[key]
                r = med(d["ratio"])
                s = med(d["speedup"])
                t = avg(d["time_ms"])
                if r <= threshold:
                    best_fd = fd
                    best_ratio = r
                    best_speedup = s
                    best_time = t
            print(f'{source:<12} {w:>6} {best_fd:>6} {best_ratio:>10.4f} {best_speedup:>12.4f} {best_time:>12.4f}')
        print()

# ===== TABLE 3: Recommended min_interval_joints per source =====
print("\n===== Recommended min_interval_joints (D=7, ratio threshold <= 2.0) =====")
print(f"{'Source':<12} {'MaxOptFD':>9} {'min_ivj':>8} {'Notes'}")
print('-' * 60)

for source in ['IFK', 'CritSample', 'Analytical', 'GCPC']:
    max_opt_fd = 0
    for w in ['0.050', '0.100', '0.200', '0.300', '0.500', '0.700', '1.000']:
        for fd in range(7):
            key = (source, w, fd)
            if key not in stats: continue
            d = stats[key]
            if med(d["ratio"]) <= 2.0:
                max_opt_fd = max(max_opt_fd, fd)
    min_ivj = 7 - max_opt_fd
    note = ""
    if source == 'IFK':
        note = "freeze ≈ iFK, ratio always near 1.0"
    elif source == 'CritSample':
        note = "moderate inflation at high fd"
    elif source == 'Analytical':
        note = "similar inflation profile to CritSample"  
    elif source == 'GCPC':
        note = "similar to Analytical, huge speedup"
    print(f'{source:<12} {max_opt_fd:>9} {min_ivj:>8}   {note}')

# ===== TABLE 4: Width threshold analysis =====
print("\n===== Width threshold analysis (at which width does ratio exceed 2.0?) =====")
print(f"{'Source':<12} {'FD':>3} {'MaxWidth(ratio<=2)':>20} {'Suggested_threshold'}")
print('-' * 65)

for source in ['IFK', 'CritSample', 'Analytical', 'GCPC']:
    for fd in [2, 3, 4, 5, 6]:
        max_safe_width = "none"
        for w in ['1.000', '0.700', '0.500', '0.300', '0.200', '0.100', '0.050']:
            key = (source, w, fd)
            if key not in stats: continue
            d = stats[key]
            if med(d["ratio"]) <= 2.0:
                max_safe_width = w
                break
        print(f'{source:<12} {fd:>3} {max_safe_width:>20}')
    print()
