# Chapter 3. Link Interval Envelopes

This chapter develops the geometric foundation behind link interval envelopes.
The key objective is to reduce the swept-volume certification of a rigid link
over a joint interval to a small number of reusable endpoint envelopes. The
presentation is intentionally split into three layers. Section 3.1 establishes
the geometric lemmas: what an envelope is, when erosion-dilation factorisation
is exact, and why endpoint convex envelopes are sufficient to outer-bound the
swept volume of a convex link. Section 3.2 then defines endpoint interval
envelopes and summarises the three endpoint sources used in this work. Finally,
Section 3.3 lifts endpoint envelopes to link envelopes and compares the storage
formats used later by the planner.

Throughout the chapter, let $I \subseteq \mathcal{Q}$ be a joint interval and
let $T(q)x = R(q)x + t(q)$ denote the rigid transform induced by configuration
$q \in I$. For a reference body $C_0 \subset \mathbb{R}^3$, its pose at
configuration $q$ is $C(q) = T(q) C_0$.

## 3.1 Geometric Foundation: From Point Envelopes to Convex Link Envelopes

### 3.1.1 Swept envelope, convex envelope, erosion, and dilation

**Definition 3.1 (Swept envelope).**
For a reference body $C_0$, the swept envelope of $C_0$ over interval $I$ is

$$
\mathcal{S}_I(C_0) := \bigcup_{q \in I} C(q).
$$

This is the exact occupied workspace set of the body while the joints vary over
$I$.

**Definition 3.2 (Convex envelope).**
The convex envelope of $C_0$ over interval $I$ is defined as

$$
\mathcal{C}_I(C_0) := \operatorname{conv}(\mathcal{S}_I(C_0)).
$$

The convex envelope is the smallest convex set containing the full swept
envelope. It is often easier to analyse and combine than the exact swept set,
and it is the correct object when the downstream representation is itself
convex, such as an AABB or a convex hull.

**Definition 3.3 (Morphological erosion and dilation).**
Let $B_2(r)$ be the closed Euclidean ball of radius $r$. For a compact set
$C \subset \mathbb{R}^3$, its radius-$r$ erosion and dilation are

$$
C \ominus B_2(r) := \{x \in \mathbb{R}^3 \mid x + B_2(r) \subseteq C\},
$$

$$
C \oplus B_2(r) := \{x + y \mid x \in C,\ y \in B_2(r)\}.
$$

The dilation $C \oplus B_2(r)$ is the standard Minkowski offset of $C$ by a
ball, while the erosion $C \ominus B_2(r)$ removes all points that are within
distance $r$ of the boundary.

The following point deserves emphasis because it affects the correctness of the
entire chapter. In general,

$$
(C \ominus B_2(r)) \oplus B_2(r) \neq C.
$$

The left-hand side is the morphological opening of $C$, which rounds off sharp
corners and may be strictly smaller than the original body. Therefore, the
statement "erode first, compute the envelope, and then dilate back to recover
the original envelope" is not exact for an arbitrary convex body.

It becomes exact only when the body is itself an exact radius-$r$ offset of a
smaller seed set.

**Definition 3.4 ($r$-offset body).**
A compact convex body $C$ is called an $r$-offset body if there exists a compact
convex seed set $S$ such that

$$
C = S \oplus B_2(r).
$$

Capsules are the most important example in this paper: a capsule is exactly the
offset of a line segment by a Euclidean ball.

**Lemma 3.1 (Radius lift for offset bodies).**
Let $C = S \oplus B_2(r)$ be an $r$-offset body. Then for any joint interval
$I$,

$$
\mathcal{S}_I(C) = \mathcal{S}_I(S) \oplus B_2(r),
$$

and consequently,

$$
\mathcal{C}_I(C) = \mathcal{C}_I(S) \oplus B_2(r).
$$

If $\widehat{\mathcal{E}}_I(S)$ is any conservative envelope satisfying
$\mathcal{S}_I(S) \subseteq \widehat{\mathcal{E}}_I(S)$, then

$$
\mathcal{S}_I(C) \subseteq \widehat{\mathcal{E}}_I(S) \oplus B_2(r).
$$

**Proof.**
For each fixed $q$, rigid transforms commute with ball offsets because rotations
preserve Euclidean balls:

$$
T(q)(S \oplus B_2(r)) = T(q)S \oplus B_2(r).
$$

Taking the union over $q \in I$ yields

$$
\mathcal{S}_I(C)
= \bigcup_{q \in I} \bigl(T(q)S \oplus B_2(r)\bigr)
= \left(\bigcup_{q \in I} T(q)S\right) \oplus B_2(r)
= \mathcal{S}_I(S) \oplus B_2(r).
$$

Because $B_2(r)$ is convex and fixed, convexification commutes with adding this
ball, which proves the second identity. The final inclusion follows immediately
from monotonicity of Minkowski addition. ∎

**Remark 3.1 (Why the capsule model matters).**
Lemma 3.1 is exact for capsules and, more generally, for any link geometry that
can be written as a rounded offset of a lower-dimensional seed. It is not exact
for a sharp-cornered convex polytope unless that polytope already has such an
offset representation. This is one of the main reasons why capsule models are
especially attractive in certified planning: they turn a 3D swept-volume problem
into a swept-seed problem plus a constant-radius lift.

### 3.1.2 Endpoint convex envelopes are sufficient for convex links

The next result explains why endpoint envelopes are enough to control the swept
volume of a whole convex link.

**Theorem 3.1 (Vertex-envelope lifting for convex polytopes).**
Let

$$
P = \operatorname{conv}\{v_1,\dots,v_m\}
$$

be a convex polytope in its reference frame. For each vertex $v_i$, define its
trajectory set over interval $I$ as

$$
\Gamma_i(I) := \{T(q) v_i \mid q \in I\}.
$$

Let $\widehat{\Gamma}_i(I)$ be any convex outer envelope of $\Gamma_i(I)$, i.e.,
$\Gamma_i(I) \subseteq \widehat{\Gamma}_i(I)$ and
$\widehat{\Gamma}_i(I)$ is convex. Then the swept polytope satisfies

$$
\mathcal{S}_I(P) \subseteq
\operatorname{conv}\left(\bigcup_{i=1}^m \widehat{\Gamma}_i(I)\right).
$$

Moreover, the convex envelope of the swept polytope satisfies

$$
\mathcal{C}_I(P) =
\operatorname{conv}\left(\bigcup_{i=1}^m \Gamma_i(I)\right).
$$

**Proof.**
Fix any $q \in I$. Because rigid transforms preserve convex combinations,

$$
T(q)P
= T(q)\operatorname{conv}\{v_1,\dots,v_m\}
= \operatorname{conv}\{T(q)v_1,\dots,T(q)v_m\}.
$$

Each point in $T(q)P$ can therefore be written as

$$
x = \sum_{i=1}^m \lambda_i T(q)v_i,
\qquad \lambda_i \ge 0,
\quad \sum_{i=1}^m \lambda_i = 1.
$$

Since $T(q)v_i \in \Gamma_i(I) \subseteq \widehat{\Gamma}_i(I)$ for every $i$,
the point $x$ lies in
$\operatorname{conv}(\cup_i \widehat{\Gamma}_i(I))$. Because $q$ was arbitrary,
the entire swept set $\mathcal{S}_I(P)$ is contained in that convex hull.

For the second statement, the first inclusion follows by choosing
$\widehat{\Gamma}_i(I)=\Gamma_i(I)$ and then taking the convex hull. The reverse
inclusion is immediate because every endpoint trajectory point already belongs
to the swept polytope of $P$. Therefore the convex hull of all endpoint
trajectory sets is exactly the convex envelope of the swept polytope. ∎

**Remark 3.2 (Extension beyond polytopes).**
Theorem 3.1 extends in a formal sense to any compact convex body $C$ by
replacing the finite vertex set with the set of extreme points
$\operatorname{ext}(C)$. Indeed,

$$
C = \operatorname{conv}(\operatorname{ext}(C)),
$$

so the same argument yields

$$
\mathcal{C}_I(C) =
\operatorname{conv}\left(\bigcup_{x \in \operatorname{ext}(C)}
\{T(q)x : q \in I\}\right).
$$

However, for a smooth convex body the set of extreme points is typically
infinite, so this extension is more of a geometric justification than a direct
computational recipe. In practice, a finite reduction is especially attractive
for two classes of shapes:

1. convex polytopes with a small number of vertices, and
2. rounded offset bodies, such as capsules, where Lemma 3.1 removes the radius
      analytically.

### 3.1.3 Capsule example and practical interpretation for robot links

Let a capsule link be written as

$$
K = [a,b] \oplus B_2(r),
$$

where $[a,b] = \operatorname{conv}\{a,b\}$ is the centre segment and $r$ is the
link radius. Combining Lemma 3.1 and Theorem 3.1 gives the following corollary.

**Corollary 3.1 (Capsule envelope from endpoint envelopes).**
Let $\widehat{\Gamma}_a(I)$ and $\widehat{\Gamma}_b(I)$ be convex outer envelopes
of the two endpoint trajectories. Then a conservative convex envelope of the
swept capsule is

$$
\operatorname{conv}\left(
\widehat{\Gamma}_a(I) \cup \widehat{\Gamma}_b(I)
\right) \oplus B_2(r).
$$

If the exact endpoint trajectory sets are used, then the exact convex envelope
of the swept capsule is

$$
\mathcal{C}_I(K)
= \operatorname{conv}\left(
\Gamma_a(I) \cup \Gamma_b(I)
\right) \oplus B_2(r).
$$

This corollary is the core justification for the modelling choice used in the
rest of the paper. Most manipulator links admit a conservative capsule model,
and once this model is adopted, the certification problem reduces to two moving
endpoints plus one scalar radius. The radius can be handled analytically, while
the remaining geometric work is concentrated entirely on endpoint envelopes.

**Suggested figure for this section.**
Figure 3 should illustrate the full capsule reduction pipeline in four panels:

1. a capsule link $K = [a,b] \oplus B_2(r)$ in the reference frame;
2. the centre segment $[a,b]$ obtained after removing the radius analytically;
3. the two endpoint trajectory envelopes over interval $I$ and the convex hull
      joining them;
4. the final radius lift obtained by dilating the centre-segment envelope by
      $B_2(r)$.

For readability, the figure should also explicitly contrast three objects: the
exact swept set, its convex envelope, and the AABB approximation used by the
planner. This avoids a common source of confusion in envelope papers, namely
mixing exact geometry, convex relaxation, and axis-aligned implementation.

## 3.2 Endpoint Interval Envelopes: Definition and Construction

Section 3.1 shows that link envelopes can be reduced to endpoint envelopes. We
now define the endpoint object used in the actual implementation.

**Definition 3.5 (Endpoint interval envelope).**
Let $p_k(q) \in \mathbb{R}^3$ be the position of endpoint $k$ under
configuration $q$. The exact endpoint trajectory set over interval $I$ is

$$
\Gamma_k(I) := \{p_k(q) \mid q \in I\}.
$$

An endpoint interval envelope is any set $E_k(I)$ satisfying
$\Gamma_k(I) \subseteq E_k(I)$.

In this paper, for computational simplicity and cache regularity, we choose the
endpoint envelope to be an axis-aligned bounding box:

$$
E_k(I) := \operatorname{bbox}(\Gamma_k(I)).
$$

This AABB choice is not theoretically mandatory. Other convex endpoint
envelopes, such as zonotopes, oriented boxes, or general polytopes, could be
used as drop-in replacements. The present paper adopts AABBs because they make
the downstream link constructions simple, storage-friendly, and fast to combine.

### 3.2.1 IFK with affine arithmetic

The default certified endpoint source is interval forward kinematics (IFK),
optionally enhanced with affine forms to reduce dependency loss.

Given an interval box

$$
I = [q_1^-, q_1^+] \times \cdots \times [q_n^-, q_n^+],
$$

we propagate interval or affine-transform bounds through the forward-kinematic
chain and obtain conservative coordinate ranges for each endpoint position
$p_k(q)$. This method is attractive for three reasons.

1. It is certified by construction: inclusion monotonicity guarantees that the
      true endpoint trajectory is never missed.
2. It is cheap enough for the planning hot path.
3. It reuses the same prefix transforms after interval splitting, which makes it
      naturally compatible with a hierarchical cache.

Its main weakness is the well-known wrapping effect: interval multiplication can
inflate the coordinate range when the joint interval becomes wide or when the
kinematic chain is long.

### 3.2.2 Critical-sample envelope

The second option is to estimate the endpoint AABB from a set of critical or
near-critical samples on the interval boundary and interior. The guiding idea is
that the extrema of each coordinate function often occur at a small number of
geometrically meaningful configurations. By explicitly evaluating those samples,
one can often obtain a much tighter box than raw IFK.

This source is useful when the objective is envelope tightness profiling or an
advisory envelope channel. However, unless the critical set is known to be
complete, critical-sample boxes should be treated as heuristic outer bounds
rather than as the sole safety certificate. For the current planner, IFK remains
the robust safe channel, while critical sampling is primarily a tool for
comparing the attainable tightness-runtime trade-off.

### 3.2.3 Analytical extrema and a global critical-point cache

The third option is to solve endpoint coordinate extrema analytically. For each
coordinate function of $p_k(q)$ over interval $I$, one can derive stationary
conditions, solve the candidate critical points, and evaluate the extrema exactly
or near-exactly. This route is typically the tightest among the three endpoint
sources, but it is also the most expensive if solved from scratch for every box.

The computational burden can be reduced by a global critical-point cache. The
cache stores previously discovered critical patterns keyed by the kinematic
structure and interval type, so later intervals can warm-start or partially
reuse earlier analytical solutions. This makes the analytical source compatible
with the broader philosophy of the paper: geometric evidence should be reused
whenever it depends primarily on robot kinematics rather than on the obstacle
scene.

## 3.3 Link Interval Envelopes: Definition, Construction, and Storage

We now lift endpoint envelopes to link envelopes.

Let link $\ell$ connect endpoint $p_{\ell-1}(q)$ and endpoint $p_\ell(q)$, and
let its capsule radius be $r_\ell$. The centre segment at configuration $q$ is

$$
L_\ell(q) := \{(1-\tau)p_{\ell-1}(q) + \tau p_\ell(q) \mid \tau \in [0,1]\}.
$$

The exact swept link volume is

$$
\mathcal{S}_I\bigl(L_\ell \oplus B_2(r_\ell)\bigr).
$$

By Lemma 3.1, it is sufficient to build an envelope for the swept centre segment
and then lift it by the radius.

**Definition 3.6 (Link interval envelope).**
A link interval envelope for link $\ell$ over interval $I$ is any set
$B_\ell(I)$ satisfying

$$
\mathcal{S}_I\bigl(L_\ell \oplus B_2(r_\ell)\bigr) \subseteq B_\ell(I).
$$

In implementation, it is convenient to separate the zero-radius centreline
envelope from the final radius lift. We therefore write

$$
B_\ell(I) = B_\ell^{\circ}(I) \oplus \mathcal{R}(r_\ell),
$$

where $B_\ell^{\circ}(I)$ is the centreline envelope and $\mathcal{R}(r_\ell)$ is
the chosen padding model. In the current AABB pipeline,
$\mathcal{R}(r_\ell) = B_\infty(r_\ell)$ because coordinate-wise padding is the
cheapest certified implementation.

### 3.3.1 AABB envelope from endpoint AABBs

Given endpoint AABBs $E_{\ell-1}(I)$ and $E_\ell(I)$, the simplest zero-radius
link envelope is

$$
B_\ell^{\circ}(I)
:= \operatorname{bbox}\bigl(E_{\ell-1}(I) \cup E_\ell(I)\bigr).
$$

The final padded envelope is then

$$
B_\ell(I) = B_\ell^{\circ}(I) \oplus B_\infty(r_\ell).
$$

This is the basic LinkIAABB representation. It is cheap, robust, and very easy
to test against obstacle AABBs. Its price is conservatism: diagonal empty space
inside the swept convex hull is inevitably filled in by the outer axis-aligned
box.

### 3.3.2 Subdivided AABB envelope

A tighter alternative is to subdivide the centre segment into $S$ subsegments in
the reference frame and envelope each subsegment separately. Let

$$
s_j = a + \frac{j}{S}(b-a), \qquad j = 0,\dots,S.
$$

The $j$-th subsegment is $[s_{j-1}, s_j]$. For each subsegment, we compute a
zero-radius AABB envelope from the endpoint envelopes of $s_{j-1}$ and $s_j$ and
store the family

$$
\{B_{\ell,j}^{\circ}(I)\}_{j=1}^S.
$$

After radius padding, the full envelope is represented as the union

$$
B_\ell^{(S)}(I) := \bigcup_{j=1}^S
\bigl(B_{\ell,j}^{\circ}(I) \oplus B_\infty(r_\ell)\bigr).
$$

This representation is strictly more informative than a single AABB because each
subsegment can adapt to a different local motion range.

**Important implementation note.**
The subdivision only improves tightness if the sub-envelopes are kept as a
union, or rasterised into a grid, or otherwise stored separately. If all
sub-envelopes are merged back into one global AABB, then the benefit largely
disappears; in fact, the result collapses to nearly the same box as the original
single-AABB construction. Therefore, a meaningful "subAABB" method must retain
the sub-box structure rather than immediately taking the AABB of their union.

### 3.3.3 Hull envelope from endpoint envelopes

When both endpoint envelopes are convex, a more geometry-aware zero-radius link
envelope is given by their convex hull:

$$
H_\ell^{\circ}(I) := \operatorname{conv}\bigl(E_{\ell-1}(I) \cup E_\ell(I)\bigr).
$$

The padded hull envelope is then

$$
H_\ell(I) := H_\ell^{\circ}(I) \oplus B_2(r_\ell),
$$

or, in an AABB-based implementation,

$$
\widehat{H}_\ell(I) := H_\ell^{\circ}(I) \oplus B_\infty(r_\ell).
$$

Compared with a single AABB, the hull retains much more of the directional
structure of the swept segment. In particular, it preserves diagonal occupancy
patterns that a single global box cannot express. This makes the hull variant a
natural bridge between purely box-based envelopes and voxelised shape-level
representations.

### 3.3.4 Storage as AABB unions or sparse grids

The link envelope can be stored in two main ways.

**AABB union.**
The envelope is stored as one AABB per link, or one AABB per subsegment. This
representation is fast to build, cache-friendly, and easy to test against AABB
obstacles. Its downside is conservatism: even a modestly curved swept shape may
occupy only a small fraction of the stored boxes.

**Sparse grid.**
The envelope is rasterised into occupied grid cells or voxels. This usually
provides a tighter description of complicated swept shapes and is especially
attractive when repeated union operations are required, since grid unions can be
implemented efficiently by bitwise-or or sparse-set merging. The cost is higher
memory usage and a more expensive construction stage.

This trade-off is important for the planner. AABB-based envelopes minimise
certificate construction cost and are therefore attractive for rapid expansion.
Grid-based envelopes are more expensive to build, but they become attractive
when many envelope unions, set merges, or repeated collision filters are needed
downstream.

## 3.4 Summary of the Chapter

The chapter establishes the logic underlying link interval envelopes.

1. For general convex links, the swept convex envelope can be reduced to the
      convex hull of endpoint envelopes if one tracks a finite generating set, such
      as the vertices of a polytope.
2. For capsules and other $r$-offset bodies, the radius can be separated
      exactly, so the main geometric task reduces to the centre segment.
3. This makes endpoint interval envelopes the central computational primitive.
4. Once endpoint envelopes are available, link envelopes can be represented as a
      single AABB, a union of subdivided AABBs, or a convex-hull-based shape that
      is later rasterised into a grid.

Because most manipulator links are well approximated by capsules, the remainder
of the paper focuses on the capsule case. This is not a loss of generality in a
practical sense: it captures the dominant industrial link model while preserving
an explicit path to more general convex geometries through Theorem 3.1.