"""Per-link diagnostic for envelope method comparison."""
import json, numpy as np

with open('../examples/viz_output/envelope_comparison.json') as f:
    d = json.load(f)

delta = d['delta']
link_radii = d['link_radii']

# Get methods with voxel data
for mk in ['voxel_hull16', 'voxel_sub_aabb', 'voxel_full_aabb']:
    md = d['methods'][mk]
    centres = np.array(md['centres'])
    label = md['label']
    n = len(centres)
    
    if n > 0:
        lo = centres.min(axis=0)
        hi = centres.max(axis=0)
        span = hi - lo
        print(f"\n{label}: {n} voxels")
        print(f"  bbox: [{lo[0]:.4f},{lo[1]:.4f},{lo[2]:.4f}] → [{hi[0]:.4f},{hi[1]:.4f},{hi[2]:.4f}]")
        print(f"  span: [{span[0]:.4f},{span[1]:.4f},{span[2]:.4f}]")

# Compare AABB methods
for mk in ['full_aabb', 'sub_aabb']:
    md = d['methods'][mk]
    aabbs = md['aabbs']
    label = md['label']
    
    all_lo = np.array([a['lo'] for a in aabbs])
    all_hi = np.array([a['hi'] for a in aabbs])
    
    print(f"\n{label}: {len(aabbs)} boxes")
    for a in aabbs:
        lo = np.array(a['lo'])
        hi = np.array(a['hi'])
        vol = (hi - lo).prod()
        link = a.get('link', '?')
        seg = a.get('seg', -1)
        tag = f"L{link}" + (f".{seg}" if seg >= 0 else "")
        span = hi - lo
        if seg < 0 or seg == 0:  # Only print first segment for sub-aabb
            print(f"  {tag}: [{lo[0]:.4f},{lo[1]:.4f},{lo[2]:.4f}] → [{hi[0]:.4f},{hi[1]:.4f},{hi[2]:.4f}]  vol={vol:.6f}")

print(f"\n=== Summary ===")
print(f"{'Method':<20s} {'Voxels':>8s} {'Volume (m³)':>12s}")
for mk in ['full_aabb', 'sub_aabb', 'voxel_full_aabb', 'voxel_sub_aabb', 'voxel_hull16']:
    md = d['methods'][mk]
    if md['type'] == 'voxel':
        n = md['n_occupied']
        vol = n * delta**3
        print(f"{md['label']:<20s} {n:>8d} {vol:>12.6f}")
    else:
        aabbs = md['aabbs']
        total_vol = sum((np.array(a['hi']) - np.array(a['lo'])).prod() for a in aabbs)
        print(f"{md['label']:<20s} {len(aabbs):>8d} {total_vol:>12.6f}")

# Sanity: check if hull16 has voxels outside the full_aabb bbox
fa_aabbs = d['methods']['full_aabb']['aabbs']
fa_lo = np.min([a['lo'] for a in fa_aabbs], axis=0)
fa_hi = np.max([a['hi'] for a in fa_aabbs], axis=0)

hull_centres = np.array(d['methods']['voxel_hull16']['centres'])
half = delta / 2
outside = np.any(hull_centres - half < fa_lo - 1e-6, axis=1) | np.any(hull_centres + half > fa_hi + 1e-6, axis=1)
n_outside = outside.sum()
print(f"\nHull-16 voxels outside Full-AABB bbox: {n_outside} / {len(hull_centres)}")
if n_outside > 0:
    out_pts = hull_centres[outside]
    print(f"  Example outside voxel centres:")
    for i in range(min(5, len(out_pts))):
        print(f"    [{out_pts[i,0]:.4f}, {out_pts[i,1]:.4f}, {out_pts[i,2]:.4f}]")
    print(f"  Full-AABB bbox: [{fa_lo[0]:.4f},{fa_lo[1]:.4f},{fa_lo[2]:.4f}] → [{fa_hi[0]:.4f},{fa_hi[1]:.4f},{fa_hi[2]:.4f}]")
