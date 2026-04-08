import json, numpy as np

with open('../examples/viz_output/envelope_comparison.json') as f:
    d = json.load(f)

print("=== Method Stats ===")
for k, v in d['methods'].items():
    if v['type'] == 'voxel':
        centres = np.array(v['centres'])
        lo = centres.min(axis=0)
        hi = centres.max(axis=0)
        vol = len(centres) * d['delta']**3
        print(f"{k:20s}  voxels={v['n_occupied']:6d}  bricks={v['n_bricks']:4d}  vol={vol:.6f} m3")
        print(f"  bbox: [{lo[0]:.3f},{lo[1]:.3f},{lo[2]:.3f}] → [{hi[0]:.3f},{hi[1]:.3f},{hi[2]:.3f}]")
    else:
        aabbs = v['aabbs']
        all_lo = np.array([a['lo'] for a in aabbs])
        all_hi = np.array([a['hi'] for a in aabbs])
        total_vol = sum((np.array(a['hi']) - np.array(a['lo'])).prod() for a in aabbs)
        print(f"{k:20s}  n_aabbs={len(aabbs):4d}  total_vol={total_vol:.6f} m3")
        print(f"  bbox: [{all_lo.min(0)[0]:.3f},{all_lo.min(0)[1]:.3f},{all_lo.min(0)[2]:.3f}] → [{all_hi.max(0)[0]:.3f},{all_hi.max(0)[1]:.3f},{all_hi.max(0)[2]:.3f}]")

print("\n=== Box Intervals ===")
for i, iv in enumerate(d['box_intervals']):
    print(f"  dim{i}: [{iv[0]:.4f}, {iv[1]:.4f}]  width={iv[1]-iv[0]:.4f}")
print(f"  delta = {d['delta']}")
