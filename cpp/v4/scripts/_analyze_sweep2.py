import csv
from collections import defaultdict

rows = list(csv.DictReader(open('exp_freeze_depth_sweep_output.csv', encoding='utf-16')))

stats = defaultdict(lambda: {'ratio':[], 'speedup':[], 'time_ms':[]})
for r in rows:
    key = (r['source'], r['width_frac'], int(r['freeze_depth']))
    stats[key]['ratio'].append(float(r['ratio']))
    stats[key]['speedup'].append(float(r['speedup']))
    stats[key]['time_ms'].append(float(r['time_ms']))

def med(lst):
    s = sorted(lst)
    return s[len(s)//2] if s else 0
def avg(lst):
    return sum(lst)/len(lst) if lst else 0

# ===== Ratio comparison across sources =====
print('=== Ratio comparison across sources (median) ===')
print(f'{"Width":>6} {"FD":>3} {"IFK":>8} {"Crit":>8} {"Anal":>8} {"GCPC":>8}')
for w in ['0.050', '0.200', '0.500', '1.000']:
    for fd in [0, 2, 4, 6]:
        vals = []
        for src in ['IFK', 'CritSample', 'Analytical', 'GCPC']:
            vals.append(med(stats[(src, w, fd)]['ratio']))
        print(f'{w:>6} {fd:>3} {vals[0]:>8.4f} {vals[1]:>8.4f} {vals[2]:>8.4f} {vals[3]:>8.4f}')
    print()

# ===== Speedup comparison =====
print('\n=== Speedup comparison (median) ===')
print(f'{"Width":>6} {"FD":>3} {"IFK":>8} {"Crit":>8} {"Anal":>8} {"GCPC":>8}')
for w in ['0.050', '0.200', '0.500', '1.000']:
    for fd in [0, 1, 2, 3, 4, 5, 6]:
        vals = []
        for src in ['IFK', 'CritSample', 'Analytical', 'GCPC']:
            vals.append(med(stats[(src, w, fd)]['speedup']))
        print(f'{w:>6} {fd:>3} {vals[0]:>8.2f} {vals[1]:>8.2f} {vals[2]:>8.2f} {vals[3]:>8.2f}')
    print()

# ===== Condensed recommendation table =====
print('\n=== Per-source recommendation (ratio <= 2.0 constraint) ===')
print(f'{"Source":<12} {"Best fd|w=0.05":>14} {"Best fd|w=0.20":>14} {"Best fd|w=0.50":>14} {"Best fd|w=1.00":>14}')
print('-' * 70)
for src in ['IFK', 'CritSample', 'Analytical', 'GCPC']:
    parts = []
    for w in ['0.050', '0.200', '0.500', '1.000']:
        best_fd = 0
        for fd in range(7):
            key = (src, w, fd)
            if key in stats and med(stats[key]['ratio']) <= 2.0:
                best_fd = fd
        # get speedup at best_fd
        sp = med(stats[(src, w, best_fd)]['speedup'])
        parts.append(f'fd={best_fd} ({sp:.1f}x)')
    print(f'{src:<12} {parts[0]:>14} {parts[1]:>14} {parts[2]:>14} {parts[3]:>14}')

# ===== Condensed recommendation table (ratio <= 3.0) =====
print(f'\n{"Source":<12} (ratio <= 3.0)')
print('-' * 70)
for src in ['IFK', 'CritSample', 'Analytical', 'GCPC']:
    parts = []
    for w in ['0.050', '0.200', '0.500', '1.000']:
        best_fd = 0
        for fd in range(7):
            key = (src, w, fd)
            if key in stats and med(stats[key]['ratio']) <= 3.0:
                best_fd = fd
        sp = med(stats[(src, w, best_fd)]['speedup'])
        parts.append(f'fd={best_fd} ({sp:.1f}x)')
    print(f'{src:<12} {parts[0]:>14} {parts[1]:>14} {parts[2]:>14} {parts[3]:>14}')

# ===== min_interval_joints derivation =====
print('\n=== Derived min_interval_joints (D=7) ===')
print(f'{"Source":<12} {"Threshold":>10} {"MaxOptFD":>9} {"min_ivj":>8} {"Rationale"}')
print('-' * 80)
for src in ['IFK', 'CritSample', 'Analytical', 'GCPC']:
    for thresh in [2.0, 3.0]:
        max_fd = 0
        for w in ['0.050', '0.100', '0.200', '0.300', '0.500', '0.700', '1.000']:
            for fd in range(7):
                key = (src, w, fd)
                if key in stats and med(stats[key]['ratio']) <= thresh:
                    max_fd = max(max_fd, fd)
        ivj = 7 - max_fd
        print(f'{src:<12} {thresh:>10.1f} {max_fd:>9} {ivj:>8}')
    print()

# ===== Absolute time comparison =====
print('\n=== Absolute time (ms, median) at representative points ===')
print(f'{"Source":<12} {"W":>5} {"fd=0":>8} {"fd=1":>8} {"fd=2":>8} {"fd=3":>8} {"fd=4":>8}')
print('-' * 60)
for src in ['IFK', 'CritSample', 'Analytical', 'GCPC']:
    for w in ['0.050', '0.200', '1.000']:
        parts = [f'{src:<12}', f'{w:>5}']
        for fd in range(5):
            key = (src, w, fd)
            t = med(stats[key]['time_ms']) if key in stats else 0
            parts.append(f'{t:>8.3f}')
        print(' '.join(parts))
    print()
