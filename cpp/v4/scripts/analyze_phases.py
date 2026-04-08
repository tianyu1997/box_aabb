import csv, statistics
from collections import defaultdict

rows = []
with open('phase_compare.csv', encoding='utf-8-sig') as f:
    reader = csv.DictReader(f)
    for r in reader:
        rows.append(r)

print(f'Total rows: {len(rows)}')

widths = defaultdict(list)
for r in rows:
    w = float(r['width'])
    widths[w].append(r)

print()
print('=' * 130)
print('PER-WIDTH MEAN TIMING (ms)')
print('=' * 130)
hdr = f'{"Width":>6} | {"GCPC_tot":>9} | {"A":>7} | {"B":>7} | {"C":>7} | {"D":>7} | {"E":>7} | {"F":>7} | {"G":>7} | {"Anal_tot":>9} | {"Ratio":>6}'
print(hdr)
print('-' * 130)

for w in sorted(widths.keys()):
    rs = widths[w]
    gt = statistics.mean([float(r['gcpc_total_ms']) for r in rs])
    ga = statistics.mean([float(r['gcpc_A_ms']) for r in rs])
    gb = statistics.mean([float(r['gcpc_B_ms']) for r in rs])
    gc = statistics.mean([float(r['gcpc_C_ms']) for r in rs])
    gd = statistics.mean([float(r['gcpc_D_ms']) for r in rs])
    ge = statistics.mean([float(r['gcpc_E_ms']) for r in rs])
    gf = statistics.mean([float(r['gcpc_F_ms']) for r in rs])
    gg = statistics.mean([float(r['gcpc_G_ms']) for r in rs])
    at = statistics.mean([float(r['anal_total_ms']) for r in rs])
    ratio = gt / at if at > 0 else 0
    print(f'{w:>6.3f} | {gt:>9.3f} | {ga:>7.3f} | {gb:>7.3f} | {gc:>7.3f} | {gd:>7.3f} | {ge:>7.3f} | {gf:>7.3f} | {gg:>7.3f} | {at:>9.3f} | {ratio:>6.3f}')

gt_all = statistics.mean([float(r['gcpc_total_ms']) for r in rows])
at_all = statistics.mean([float(r['anal_total_ms']) for r in rows])
ga_all = statistics.mean([float(r['gcpc_A_ms']) for r in rows])
gb_all = statistics.mean([float(r['gcpc_B_ms']) for r in rows])
gc_all = statistics.mean([float(r['gcpc_C_ms']) for r in rows])
gd_all = statistics.mean([float(r['gcpc_D_ms']) for r in rows])
ge_all = statistics.mean([float(r['gcpc_E_ms']) for r in rows])
gf_all = statistics.mean([float(r['gcpc_F_ms']) for r in rows])
gg_all = statistics.mean([float(r['gcpc_G_ms']) for r in rows])
print('-' * 130)
print(f'{"ALL":>6} | {gt_all:>9.3f} | {ga_all:>7.3f} | {gb_all:>7.3f} | {gc_all:>7.3f} | {gd_all:>7.3f} | {ge_all:>7.3f} | {gf_all:>7.3f} | {gg_all:>7.3f} | {at_all:>9.3f} | {gt_all/at_all:>6.3f}')

# GCPC phase percentage
print()
print('=' * 80)
print('GCPC PHASE BREAKDOWN (% of total)')
print('=' * 80)
print(f'{"Width":>6} | {"A%":>6} | {"B%":>6} | {"C%":>6} | {"D%":>6} | {"E%":>6} | {"F%":>6} | {"G%":>6}')
print('-' * 80)
for w in sorted(widths.keys()):
    rs = widths[w]
    gt = statistics.mean([float(r['gcpc_total_ms']) for r in rs])
    ga = statistics.mean([float(r['gcpc_A_ms']) for r in rs])
    gb = statistics.mean([float(r['gcpc_B_ms']) for r in rs])
    gc = statistics.mean([float(r['gcpc_C_ms']) for r in rs])
    gd = statistics.mean([float(r['gcpc_D_ms']) for r in rs])
    ge = statistics.mean([float(r['gcpc_E_ms']) for r in rs])
    gf = statistics.mean([float(r['gcpc_F_ms']) for r in rs])
    gg = statistics.mean([float(r['gcpc_G_ms']) for r in rs])
    if gt > 0:
        print(f'{w:>6.3f} | {ga/gt*100:>5.1f}% | {gb/gt*100:>5.1f}% | {gc/gt*100:>5.1f}% | {gd/gt*100:>5.1f}% | {ge/gt*100:>5.1f}% | {gf/gt*100:>5.1f}% | {gg/gt*100:>5.1f}%')

print('-' * 80)
if gt_all > 0:
    print(f'{"ALL":>6} | {ga_all/gt_all*100:>5.1f}% | {gb_all/gt_all*100:>5.1f}% | {gc_all/gt_all*100:>5.1f}% | {gd_all/gt_all*100:>5.1f}% | {ge_all/gt_all*100:>5.1f}% | {gf_all/gt_all*100:>5.1f}% | {gg_all/gt_all*100:>5.1f}%')

# Per-width candidate counts
print()
print('=' * 130)
print('PER-WIDTH MEAN CANDIDATE COUNTS')
print('=' * 130)
print(f'{"Width":>6} | {"GCPC":>58} | {"Analytical":>58}')
print(f'{"":>6} | {"cache":>7} {"kpi2":>7} {"atan2":>7} {"D_face":>7} {"E_p1d":>7} {"F_p2d":>7} {"G_int":>7} | {"P0_vert":>7} {"P1_edge":>7} {"P2_face":>7} {"P25a":>7} {"P25b":>7} {"P3_int":>7}')
print('-' * 130)
for w in sorted(widths.keys()):
    rs = widths[w]
    gc_cache = statistics.mean([int(r['gcpc_n_cache']) for r in rs])
    gc_kpi2 = statistics.mean([int(r['gcpc_n_kpi2']) for r in rs])
    gc_atan2 = statistics.mean([int(r['gcpc_n_atan2']) for r in rs])
    gc_dface = statistics.mean([int(r['gcpc_n_d_faces']) for r in rs])
    gc_ep1d = statistics.mean([int(r['gcpc_n_e_pair1d']) for r in rs])
    gc_fp2d = statistics.mean([int(r['gcpc_n_f_pair2d']) for r in rs])
    gc_gint = statistics.mean([int(r['gcpc_n_g_interior']) for r in rs])
    an_p0 = statistics.mean([int(r['anal_n_p0']) for r in rs])
    an_p1 = statistics.mean([int(r['anal_n_p1']) for r in rs])
    an_p2 = statistics.mean([int(r['anal_n_p2']) for r in rs])
    an_p25a = statistics.mean([int(r['anal_n_p25a']) for r in rs])
    an_p25b = statistics.mean([int(r['anal_n_p25b']) for r in rs])
    an_p3 = statistics.mean([int(r['anal_n_p3']) for r in rs])
    print(f'{w:>6.3f} | {gc_cache:>7.0f} {gc_kpi2:>7.0f} {gc_atan2:>7.0f} {gc_dface:>7.0f} {gc_ep1d:>7.0f} {gc_fp2d:>7.0f} {gc_gint:>7.0f} | {an_p0:>7.0f} {an_p1:>7.0f} {an_p2:>7.0f} {an_p25a:>7.0f} {an_p25b:>7.0f} {an_p3:>7.0f}')

# FK calls comparison
print()
print('=' * 100)
print('PER-WIDTH MEAN FK CALLS')
print('=' * 100)
print(f'{"Width":>6} | {"GCPC_fk":>9} {"AA_prune":>9} {"AA_check":>9} | {"A_p1_fk":>9} {"A_p2_fk":>9} {"A_p25_fk":>9} {"A_p3_fk":>9} {"A_total":>9}')
print('-' * 100)
for w in sorted(widths.keys()):
    rs = widths[w]
    gc_fk = statistics.mean([int(r['gcpc_n_fk']) for r in rs])
    gc_prune = statistics.mean([int(r['gcpc_n_aa_pruned']) for r in rs])
    gc_check = statistics.mean([int(r['gcpc_n_aa_checks']) for r in rs])
    an_p1fk = statistics.mean([int(r['anal_n_p1_fk']) for r in rs])
    an_p2fk = statistics.mean([int(r['anal_n_p2_fk']) for r in rs])
    an_p25fk = statistics.mean([int(r['anal_n_p25_fk']) for r in rs])
    an_p3fk = statistics.mean([int(r['anal_n_p3_fk']) for r in rs])
    an_total = an_p1fk + an_p2fk + an_p25fk + an_p3fk
    print(f'{w:>6.3f} | {gc_fk:>9.0f} {gc_prune:>9.0f} {gc_check:>9.0f} | {an_p1fk:>9.0f} {an_p2fk:>9.0f} {an_p25fk:>9.0f} {an_p3fk:>9.0f} {an_total:>9.0f}')
