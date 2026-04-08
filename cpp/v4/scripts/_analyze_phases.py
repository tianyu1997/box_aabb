import csv, statistics
from collections import defaultdict

data = []
with open('phase_compare.csv') as f:
    for row in csv.DictReader(f):
        data.append({k: float(v) for k, v in row.items()})

by_width = defaultdict(list)
for r in data:
    by_width[r['width']].append(r)

def m(rows, k):
    return statistics.mean([r[k] for r in rows])

print('=== GCPC candidate counts (mean) ===')
print(f'{"w":>5} | {"cache":>5} {"kpi2":>5} {"atan2":>5} {"D":>5} {"E":>5} {"F":>5} {"G":>6} | {"FK":>6} {"AAprun":>6} {"AAchk":>6}')
for w in sorted(by_width.keys()):
    rows = by_width[w]
    print(f'{w:5.3f} | {m(rows,"gcpc_n_cache"):5.0f} {m(rows,"gcpc_n_kpi2"):5.0f} {m(rows,"gcpc_n_atan2"):5.0f} {m(rows,"gcpc_n_d_faces"):5.0f} {m(rows,"gcpc_n_e_pair1d"):5.0f} {m(rows,"gcpc_n_f_pair2d"):5.0f} {m(rows,"gcpc_n_g_interior"):6.0f} | {m(rows,"gcpc_n_fk"):6.0f} {m(rows,"gcpc_n_aa_pruned"):6.0f} {m(rows,"gcpc_n_aa_checks"):6.0f}')

print()
print('=== Analytical candidate counts (mean) ===')
print(f'{"w":>5} | {"P0":>5} {"P1":>5} {"P2":>5} {"P25a":>5} {"P25b":>5} {"P3":>6} | {"P1fk":>5} {"P2fk":>5} {"P25fk":>5} {"P3fk":>6}')
for w in sorted(by_width.keys()):
    rows = by_width[w]
    print(f'{w:5.3f} | {m(rows,"anal_n_p0"):5.0f} {m(rows,"anal_n_p1"):5.0f} {m(rows,"anal_n_p2"):5.0f} {m(rows,"anal_n_p25a"):5.0f} {m(rows,"anal_n_p25b"):5.0f} {m(rows,"anal_n_p3"):6.0f} | {m(rows,"anal_n_p1_fk"):5.0f} {m(rows,"anal_n_p2_fk"):5.0f} {m(rows,"anal_n_p25_fk"):5.0f} {m(rows,"anal_n_p3_fk"):6.0f}')

print()
print('=== Phase F deep analysis: F_ms vs n_f_pair2d ===')
for w in sorted(by_width.keys()):
    rows = by_width[w]
    f_ms = m(rows, 'gcpc_F_ms')
    n_f = m(rows, 'gcpc_n_f_pair2d')
    per_cand = f_ms/n_f*1000 if n_f>0 else 0
    print(f'w={w:.3f}: F={f_ms:7.2f}ms, n_f={n_f:6.0f}, per_candidate={per_cand:.1f}us')

print()
print('=== Phase G deep analysis: G_ms vs n_g_interior ===')
for w in sorted(by_width.keys()):
    rows = by_width[w]
    g_ms = m(rows, 'gcpc_G_ms')
    n_g = m(rows, 'gcpc_n_g_interior')
    per_cand = g_ms/n_g*1000 if n_g>0 else 0
    print(f'w={w:.3f}: G={g_ms:7.2f}ms, n_g={n_g:6.0f}, per_candidate={per_cand:.1f}us')

print()
print('=== Phase C+D (boundary analytical) total ===')
for w in sorted(by_width.keys()):
    rows = by_width[w]
    cd = m(rows,'gcpc_C_ms') + m(rows,'gcpc_D_ms')
    total = m(rows,'gcpc_total_ms')
    print(f'w={w:.3f}: C+D={cd:.2f}ms ({cd/total*100:.1f}%), E+F={m(rows,"gcpc_E_ms")+m(rows,"gcpc_F_ms"):.2f}ms ({(m(rows,"gcpc_E_ms")+m(rows,"gcpc_F_ms"))/total*100:.1f}%)')

print()
print('=== Overall summary ===')
all_gcpc = statistics.mean([r['gcpc_total_ms'] for r in data])
all_anal = statistics.mean([r['anal_total_ms'] for r in data])
all_F = statistics.mean([r['gcpc_F_ms'] for r in data])
all_G = statistics.mean([r['gcpc_G_ms'] for r in data])
all_C = statistics.mean([r['gcpc_C_ms'] for r in data])
all_D = statistics.mean([r['gcpc_D_ms'] for r in data])
all_E = statistics.mean([r['gcpc_E_ms'] for r in data])
all_A = statistics.mean([r['gcpc_A_ms'] for r in data])
all_B = statistics.mean([r['gcpc_B_ms'] for r in data])
print(f'GCPC mean: {all_gcpc:.2f}ms  (A={all_A:.3f} B={all_B:.3f} C={all_C:.3f} D={all_D:.3f} E={all_E:.3f} F={all_F:.2f} G={all_G:.2f})')
print(f'Anal mean: {all_anal:.2f}ms')
print(f'Ratio: {all_gcpc/all_anal:.3f}')
print(f'Phase F is {all_F/all_gcpc*100:.1f}% of GCPC total')
print(f'Phase G is {all_G/all_gcpc*100:.1f}% of GCPC total')
print(f'Phases E+F is {(all_E+all_F)/all_gcpc*100:.1f}% of GCPC total')
print(f'If Phase F were halved: {(all_gcpc-all_F/2):.2f}ms, ratio would be {(all_gcpc-all_F/2)/all_anal:.3f}')
print(f'If Phase F were zeroed: {(all_gcpc-all_F):.2f}ms, ratio would be {(all_gcpc-all_F)/all_anal:.3f}')
