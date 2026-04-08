import pandas as pd, glob, os, sys

if len(sys.argv) > 1:
    csv_path = sys.argv[1]
else:
    dirs = sorted(glob.glob('results/exp_ep_iaabb_bench_*') + glob.glob('results/ep_iaabb_bench_*'))
    csv_path = os.path.join(dirs[-1], 'results.csv') if dirs else 'results/smoke_ep_iaabb/results.csv'
print(f'Data: {csv_path}')
df = pd.read_csv(csv_path)

def band(f):
    if f < 0.19: return 'narrow'
    if f < 0.45: return 'medium'
    return 'wide'

df['band'] = df['width_frac'].apply(band)
ORDER = ['IFK', 'CritSample', 'Analytical', 'GCPC']
n_trials = df['trial'].nunique()
print(f'n_trials={n_trials}  methods={ORDER}\n')

print('=== 全量 (mean / median / min / max / n_eval) ===')
print(f"{'方法':14s}  {'mean ms':>9s}  {'median ms':>10s}  {'min ms':>8s}  {'max ms':>8s}  {'n_eval':>8s}")
print('-' * 65)
for m in ORDER:
    s = df[df['source'] == m]
    if s.empty:
        continue
    print(f"{m:14s}  {s['time_ms'].mean():9.4f}  {s['time_ms'].median():10.4f}"
          f"  {s['time_ms'].min():8.4f}  {s['time_ms'].max():8.4f}  {s['n_eval'].mean():8.1f}")

print()
print('=== 按宽度类别细分 (mean time_ms) ===')
print(f"{'方法':14s}  {'narrow':>10s}  {'medium':>10s}  {'wide':>10s}")
print('-' * 48)
for m in ORDER:
    row = []
    for b in ['narrow', 'medium', 'wide']:
        s = df[(df['source'] == m) & (df['band'] == b)]
        row.append(f"{s['time_ms'].mean():10.4f}" if not s.empty else f"{'N/A':>10s}")
    print(f"{m:14s}  {'  '.join(row)}")

print()
print('=== 速度比（相对 IFK = 1.0x）===')
base = df[df['source'] == 'IFK']['time_ms'].mean()
for m in ORDER:
    s = df[df['source'] == m]
    if s.empty:
        continue
    ratio = s['time_ms'].mean() / base
    print(f"  {m:14s}: {ratio:.2f}x")
