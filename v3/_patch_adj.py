"""Patch pipeline.py: replace _build_adjacency_and_islands with vectorized numpy."""
import sys

FILEPATH = r"c:\Users\TIAN\Documents\box_aabb\v3\src\planner\pipeline.py"

NEW_FUNC = r'''def _build_adjacency_and_islands(boxes, period=None,
                                  _chunk: int = 256):
    """Vectorized overlap -> adjacency dict + UnionFind + islands.

    Two boxes are adjacent iff their intervals overlap (>= -eps) in ALL
    dimensions.  When *period* is not None, periodic wrapping is checked
    per dimension independently.

    Uses chunked numpy broadcasting so the peak memory stays bounded
    even for large forests (O(chunk^2 * D) instead of O(N^2 * D)).
    """
    ids = list(boxes.keys())
    n = len(ids)
    adj: Dict[int, Set[int]] = {bid: set() for bid in ids}
    uf = UnionFind(ids)

    if n < 2:
        return adj, uf, uf.components()

    # -- build (N, D) lo / hi arrays --
    n_dims = len(boxes[ids[0]].joint_intervals)
    lo_arr = np.empty((n, n_dims), dtype=np.float64)
    hi_arr = np.empty((n, n_dims), dtype=np.float64)
    for idx, bid in enumerate(ids):
        for d, (lo_v, hi_v) in enumerate(boxes[bid].joint_intervals):
            lo_arr[idx, d] = lo_v
            hi_arr[idx, d] = hi_v

    eps = 1e-9

    # -- chunked pairwise overlap (upper triangle only) --
    for i0 in range(0, n, _chunk):
        i1 = min(n, i0 + _chunk)
        lo_i = lo_arr[i0:i1]          # (Bi, D)
        hi_i = hi_arr[i0:i1]

        for j0 in range(i0, n, _chunk):
            j1 = min(n, j0 + _chunk)
            lo_j = lo_arr[j0:j1]      # (Bj, D)
            hi_j = hi_arr[j0:j1]

            # overlap_width[a, b, d] = min(hi_i[a,d], hi_j[b,d])
            #                        - max(lo_i[a,d], lo_j[b,d])
            ow = (np.minimum(hi_i[:, None, :], hi_j[None, :, :])
                  - np.maximum(lo_i[:, None, :], lo_j[None, :, :]))

            if period is not None:
                ow_r = (np.minimum(hi_i[:, None, :],
                                   hi_j[None, :, :] + period)
                        - np.maximum(lo_i[:, None, :],
                                     lo_j[None, :, :] + period))
                ow_l = (np.minimum(hi_i[:, None, :],
                                   hi_j[None, :, :] - period)
                        - np.maximum(lo_i[:, None, :],
                                     lo_j[None, :, :] - period))
                ow = np.maximum(ow, np.maximum(ow_r, ow_l))

            # adjacent iff every dim overlaps (ow >= -eps)
            is_adj = np.all(ow >= -eps, axis=2)       # (Bi, Bj)

            # mask to upper triangle (avoid double-counting & self)
            if i0 == j0:
                tri = np.triu(np.ones(is_adj.shape, dtype=bool), k=1)
                is_adj &= tri

            pairs = np.argwhere(is_adj)    # (K, 2) local indices
            for li, lj in pairs:
                bi = ids[i0 + int(li)]
                bj = ids[j0 + int(lj)]
                adj[bi].add(bj)
                adj[bj].add(bi)
                uf.union(bi, bj)

    islands = uf.components()
    return adj, uf, islands

'''

with open(FILEPATH, 'r', encoding='utf-8') as f:
    content = f.read()

start_marker = 'def _build_adjacency_and_islands(boxes, period=None):'
end_marker = '\ndef _add_bridge_to_adj('

start_idx = content.index(start_marker)
end_idx = content.index(end_marker, start_idx)

new_content = content[:start_idx] + NEW_FUNC + content[end_idx+1:]  # +1 to skip the \n

with open(FILEPATH, 'w', encoding='utf-8') as f:
    f.write(new_content)

print(f"Replaced {end_idx - start_idx} chars with {len(NEW_FUNC)} chars")
print("Done!")
