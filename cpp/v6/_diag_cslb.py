import json, numpy as np, sys
sys.path.insert(0, 'scripts')
from gcs_query import load_forest_from_json, segment_in_box_union_exact

data = json.load(open('result/20260416_163139_b200000_rrt_bq_dijk_s5/gcs_query_results.json'))
forest = load_forest_from_json('result/20260416_163139_b200000_rrt_bq_dijk_s5/paths.json')
lo, hi = forest['lo'], forest['hi']
r = data['results'][2]
print(r.get('label', ''), 'wpts:', len(r['waypoints']), 'keys:', list(r.keys())[:8])
wp = np.array(r['waypoints'])
seg_lens = np.linalg.norm(np.diff(wp, axis=0), axis=1)
print(f'min={seg_lens.min():.4f} max={seg_lens.max():.4f} mean={seg_lens.mean():.4f}')
print('Top 15 longest:')
for i in np.argsort(seg_lens)[-15:]:
    cl = segment_in_box_union_exact(wp[i], wp[i+1], lo, hi)
    print(f'  seg[{i}]: len={seg_lens[i]:.4f} clean={cl}')
print('Sum of top-15:', seg_lens[np.argsort(seg_lens)[-15:]].sum())
print('Total:', seg_lens.sum())
