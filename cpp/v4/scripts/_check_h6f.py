import csv
from collections import defaultdict

rows = []
with open('phase_compare_h6f.csv', encoding='utf-8-sig') as f:
    for r in csv.DictReader(f):
        rows.append({k: float(v) for k, v in r.items()})

widths = sorted(set(round(r['width'], 3) for r in rows))

hdr = f"{'w':>6} | {'GCPC':>7} {'Anal':>7} {'ratio':>6} | {'A':>6} {'F':>7} {'G':>6}"
print(hdr)

for w in widths:
    t = [r for r in rows if round(r['width'], 3) == w]
    n = len(t)
    g = sum(r['gcpc_total_ms'] for r in t) / n
    a = sum(r['anal_total_ms'] for r in t) / n
    pa = sum(r['gcpc_A_ms'] for r in t) / n
    pf = sum(r['gcpc_F_ms'] for r in t) / n
    pg = sum(r['gcpc_G_ms'] for r in t) / n
    r2 = g / a if a > 0 else 0
    mark = ' ***' if r2 > 1 else ''
    print(f'{w:6.3f} | {g:7.2f} {a:7.2f} {r2:6.3f} | {pa:6.3f} {pf:7.2f} {pg:6.2f}{mark}')

overall_g = sum(r['gcpc_total_ms'] for r in rows) / len(rows)
overall_a = sum(r['anal_total_ms'] for r in rows) / len(rows)
print(f'Overall: GCPC={overall_g:.2f} Anal={overall_a:.2f} ratio={overall_g/overall_a:.3f}')
