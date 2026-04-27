# Chapter 4: v7 Implementation Gap Plan

This document records the gap between the Chapter 4 design narrative and the
current v7 implementation. It is intended as an engineering follow-up plan,
not as part of the reviewer-facing paper text.

## 1. Current v7 LECT Core

The following items are already implemented in v7.

- Vector-backed KD tree over joint intervals.
- Zero-radius cached geometry with online radius inflation at collision time.
- Endpoint data stored first; link envelopes lazily materialised on demand.
- `snapshot()` deep copy for worker-local trees.
- `partition_for_seeds()` and `transplant_domain()` for parallel growth.
- `WIDEST_FIRST` split policy with root-width normalisation.
- Occupancy counters and subtree free-volume bookkeeping.

## 2. Items Required by the Chapter 4 Outline but Not Yet Implemented

### 2.1 Best-Tighten Split Criterion

Status: not implemented in v7.

Evidence:

- `cpp/v7/include/sbf/lect/lect.h` exposes only `ROUND_ROBIN` and `WIDEST_FIRST`.
- `cpp/v7/src/lect/lect.cpp` uses normalized widest-first in `pick_split_dim`.
- `cpp/v7/doc/migration/P3_lect.md` explicitly states that `BEST_TIGHTEN` was
  simplified away in the v7 minimal port.

Planned implementation:

1. Add `SplitOrder::BEST_TIGHTEN` to the public enum.
2. For each candidate split dimension, probe the two child envelopes and score
   them by post-split aggregate envelope volume.
3. Cache or approximate the score to keep the hot path affordable.
4. Benchmark against `WIDEST_FIRST` on the link-envelope pipeline and Marcucci
   cabinet workload.

Validation:

- Unit test: split choice is stable under identical intervals.
- Microbenchmark: average envelope volume decreases relative to widest-first.
- Planner benchmark: no regression in success rate or build time beyond the
  accepted threshold.

### 2.2 Parent-Envelope Union Refinement

Status: not implemented as a persistent cache rewrite in v7.

Evidence:

- `split_leaf_impl()` computes child envelopes but does not replace the parent
  representation by the union of child envelopes.
- Parent refinement currently happens only operationally through recursive
  descent to children.

Planned implementation:

1. Introduce a parent-side union representation, either as a sub-box list or as
   a grid handle.
2. After a split, promote the child union as the refined parent envelope.
3. Ensure collision queries can consume this union without collapsing it back to
   one coarse global AABB.

Validation:

- Synthetic test: refined parent volume is no larger than the original parent
  AABB envelope.
- Query test: parent collision queries remain conservative.
- Memory test: union storage growth stays within budget.

### 2.3 $Z_4$ Symmetry Cache

Status: not implemented in v7.

Evidence:

- `cpp/v7/include/sbf/lect/lect.h` comment explicitly says `no Z4/mmap/disk`.
- `cpp/v7/doc/migration/P3_lect.md` lists Z4 symmetry as removed from the
  minimal port.

Planned implementation:

1. Define a canonical sector for the base-joint interval.
2. Add interval canonicalisation and inverse rotation recovery.
3. Restrict rollout initially to robots with verified four-sector base symmetry.

Validation:

- Symmetry test: canonicalised and rotated-back envelopes match direct
  computation.
- Build benchmark: cold-cache speedup on IIWA/Panda scenes.

### 2.4 Memory-Mapped Persistence

Status: not implemented in v7.

Evidence:

- No `mmap`, persistence, or node-serialization path exists in the v7 LECT code.
- Migration notes explicitly defer disk-backed persistence to a later stage.

Planned implementation:

1. Define a binary on-disk layout for node arrays and metadata.
2. Add read-only memory mapping for existing cache files.
3. Preserve node-id stability across process restarts.

Validation:

- Restart test: a second process can reuse the same cache file correctly.
- Consistency test: mapped nodes match in-memory nodes byte-for-byte.
- Stress test: startup latency and resident-memory usage stay within target.

### 2.5 True Lazy Load with LRU

Status: not implemented in v7.

Clarification:

- Current v7 already performs lazy in-memory materialisation of link envelopes
  from endpoint data.
- What is still missing is disk-backed lazy node paging plus residency control.

Planned implementation:

1. Separate logical node ids from resident node blocks.
2. Add demand paging from the memory-mapped cache file.
3. Introduce a small LRU or clock-style eviction policy for node blocks.

Validation:

- Residency test: untouched nodes are never materialised.
- Memory-cap test: peak resident usage stays under the configured limit.
- Planner test: cache misses do not change geometric correctness.

### 2.6 Incremental Save

Status: not implemented in v7.

Planned implementation:

1. Track dirty node ranges or dirty pages.
2. Flush only newly added or modified nodes.
3. Use crash-atomic temp-file replacement or append-only generation markers.

Validation:

- Crash-recovery test: partial writes never corrupt the previous cache.
- Throughput test: incremental flush is materially cheaper than full rewrite.

## 3. Recommended Development Order

1. Best-tighten split.
2. Parent-envelope union refinement.
3. Z4 symmetry.
4. Memory-mapped persistence.
5. True lazy load with LRU.
6. Incremental save.

Rationale:

- Items 1 and 2 directly affect envelope quality and planner performance.
- Item 3 reduces duplicate work for symmetric robots.
- Items 4 to 6 are storage-system improvements and should build on the final
  node representation chosen after Items 1 and 2.

## 4. Paper-Writing Guidance

For the current v7 rewrite manuscript, the following wording is accurate.

- `best-tighten` is the design objective, but `widest-first` is the currently
  deployed v7 heuristic.
- Parent-envelope refinement by child union is a valid and desirable extension,
  but not yet a persisted cache feature in v7.
- Z4 symmetry, mmap persistence, true lazy load, and incremental save belong to
  the follow-up implementation plan rather than the current measured system.