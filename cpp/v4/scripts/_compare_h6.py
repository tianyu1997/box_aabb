import csv, statistics
from collections import defaultdict

def load(f):
    data = []
    with open(f, encoding='utf-8-sig') as fh:
        for row in csv.DictReader(fh):
            data.append({k: float(v) for k, v in row.items()})
    return data

old = load('phase_compare.csv')
new = load('phase_compare_h6.csv')

def m(rows, k): return statistics.mean([r[k] for r in rows])

byw_old = defaultdict(list)
byw_new = defaultdict(list)
for r in old: byw_old[r['width']].append(r)
for r in new: byw_new[r['width']].append(r)

print('=' * 130)
hdr = f'{"w":>5} | {"GCPC_old":>8} {"GCPC_new":>8} {"spdp":>5} | {"F_old":>7} {"F_new":>7} {"F_sp":>5} | {"Anal_old":>8} {"Anal_new":>8} {"A_sp":>5} | {"G/A_old":>7} {"G/A_new":>7} {"win?":>5}'
print(hdr)
print('-' * 130)
for w in sorted(byw_old.keys()):
    go = m(byw_old[w], 'gcpc_total_ms')
    gn = m(byw_new[w], 'gcpc_total_ms')
    fo = m(byw_old[w], 'gcpc_F_ms')
    fn = m(byw_new[w], 'gcpc_F_ms')
    ao = m(byw_old[w], 'anal_total_ms')
    an = m(byw_new[w], 'anal_total_ms')
    gs = go/gn if gn > 0 else 0
    fs = fo/fn if fn > 0 else 0
    asv = ao/an if an > 0 else 0
    ratio_o = go/ao if ao > 0 else 0
    ratio_n = gn/an if an > 0 else 0
    win = "YES" if ratio_n < 1.0 else "NO"
    print(f'{w:5.3f} | {go:8.2f} {gn:8.2f} {gs:5.2f}x | {fo:7.2f} {fn:7.2f} {fs:5.2f}x | {ao:8.2f} {an:8.2f} {asv:5.2f}x | {ratio_o:7.3f} {ratio_n:7.3f} {win:>5}')

print()
print('=== Overall Summary ===')
go_all = m(old, 'gcpc_total_ms')
gn_all = m(new, 'gcpc_total_ms')
fo_all = m(old, 'gcpc_F_ms')
fn_all = m(new, 'gcpc_F_ms')
ao_all = m(old, 'anal_total_ms')
an_all = m(new, 'anal_total_ms')
print(f'GCPC total:      {go_all:.2f} -> {gn_all:.2f} ms  ({go_all/gn_all:.2f}x speedup)')
print(f'Phase F:         {fo_all:.2f} -> {fn_all:.2f} ms  ({fo_all/fn_all:.2f}x speedup)')
print(f'Analytical:      {ao_all:.2f} -> {an_all:.2f} ms  ({ao_all/an_all:.2f}x speedup)')
print(f'GCPC/Anal ratio: {go_all/ao_all:.3f} -> {gn_all/an_all:.3f}')
print()

# Per-phase breakdown for H6
print('=== H6 GCPC Per-Phase Breakdown (all widths mean) ===')
for phase in ['A', 'B', 'C', 'D', 'E', 'F', 'G']:
    key = f'gcpc_{phase}_ms'
    vo = m(old, key)
    vn = m(new, key)
    pct_o = vo/go_all*100
    pct_n = vn/gn_all*100
    print(f'  Phase {phase}: {vo:6.3f} -> {vn:6.3f} ms  ({vo/vn:.2f}x)  [{pct_o:5.1f}% -> {pct_n:5.1f}%]')
