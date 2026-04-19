# Proximity pair generation

The [BVH broadphase](00-bvh.md) reports candidate primitive pairs whose AABBs overlap within $\hat d$. Not all candidates are valid proximity pairs: some are adjacent primitives whose gap is identically zero by connectivity rather than by contact, and some are the wrong primitive-type combination for the point-triangle / edge-edge decomposition the barrier expects. Proximity pair generation is the filter between broadphase candidates and the narrow-phase per-pair distance evaluation.

## The two primitive-pair types

Per [§03-energy](../01-ipc-internals/03-energy.md), the barrier sum $B(x) = \sum_i b(d_i, \hat d)$ is indexed over two pair types for 3D triangle meshes:

- **Point-triangle** — surface vertex $v$ and a triangle $t$; gap $d$ is the shortest distance from $v$ to the closest point in $t$.
- **Edge-edge** — two edges $e_1, e_2$; gap $d$ is the shortest segment-to-segment distance.

The broadphase reports AABB-overlap candidates grouped by primitive type — vertex-against-triangle overlaps as point-triangle candidates, edge-against-edge overlaps as edge-edge candidates (whether implemented as separate trees per primitive type or as a single heterogeneous tree is a broadphase implementation detail). Other type combinations are not separately enumerated: vertex-vertex near-contact is covered by the point-triangle pathway through each vertex's incident triangles (as [§02-ccd](../01-ipc-internals/02-ccd.md) notes), and triangle-triangle closeness decomposes into its constituent point-triangle and edge-edge tests.

## Adjacency exclusions

Some candidate pairs have $d = 0$ trivially by primitive connectivity, not by physical contact:

- **Point-triangle with incident point.** If vertex $v$ is a vertex of triangle $t$, then $v$ lies in $t$'s closure, so $d(v, t) = 0$. Every interior vertex is a vertex of its six-or-so incident triangles, so every interior vertex would report six-or-so zero-distance point-triangle pairs — a count that scales with mesh size.
- **Edge-edge with shared vertex.** If edges $e_1$ and $e_2$ share a vertex, they touch at that vertex, so $d(e_1, e_2) = 0$. Every interior edge has roughly half a dozen edges sharing each of its two endpoints, so the same scaling applies.

Without these exclusions, the barrier function would diverge on every mesh's initial undeformed configuration — the barrier's divergence $b \to +\infty$ fires at every adjacency-induced zero, the total potential is $+\infty$, and the solver cannot take a single Newton step. Adjacency exclusion is not a performance optimization; it is a correctness requirement.

The exclusion rules are primitive-topological, not geometric: they depend on which vertex indices appear in which edge and triangle, and they are invariant under all mesh deformations. The filter can be precomputed once per topology.

## The adjacency predicate

[IPC Toolkit](../../appendices/00-references/00-ipc.md#ipc-toolkit)'s `CollisionMesh` class precomputes vertex-triangle incidence (which triangles contain each vertex) and vertex-edge incidence (which edges meet at each vertex), and exposes a `can_collide` predicate applied to each broadphase-reported candidate: the predicate returns `false` if the pair is incidence-excluded by the rules above and `true` otherwise. Candidates passing `can_collide` go to the narrow-phase distance computation; candidates failing it are dropped before any distance code runs.

The adjacency data is topology-only: computing it requires only the mesh connectivity, not vertex positions. For a static-topology mesh the adjacency structures are built once and reused across every timestep's broadphase query. Topology changes (remeshing, adaptive refinement) invalidate these structures and require rebuild — the same trigger that forces a BVH rebuild.

## Geometric degeneracies are not proximity-filter concerns

A point-triangle pair passing adjacency exclusion can still have geometric subcases: the point can project into the triangle's interior, onto one of the three edges, or coincide with one of the three vertices. An edge-edge pair can be parallel (infinite family of closest-point pairs) or have its closest-point parameters land outside the segments' extents (so the true closest-point pair is edge-vertex). The distance function $d(\cdot)$ handles these subcases in its piecewise definition ([§02-ccd](../01-ipc-internals/02-ccd.md) covers the classification for the CCD root-finding context; the static distance's subcase structure is analogous).

The proximity filter does not classify subcases. Its output is simply "point-triangle pair" or "edge-edge pair" with the primitive identifiers; the narrow-phase distance computation classifies the geometric subcase and returns the correct $d$.

## Self-contact uses the same filter

The adjacency exclusion rules depend on vertex indices and topological incidence; they do not inspect which mesh a primitive belongs to. A point-triangle candidate is excluded if the point's vertex index is in the triangle's vertex list, regardless of whether both primitives came from the same sleeve or from different surfaces. Two edges are excluded if they share a vertex index, regardless of mesh membership.

This closes the [§03-energy](../01-ipc-internals/03-energy.md) claim that "the barrier has no special case for self vs. cross" at the layer below the energy: the claim is true because the proximity filter has no such case either. Whatever branch self-contact might have needed — "these two primitives are on the same mesh, so check additionally whether they are in the same local neighborhood" — is subsumed by the adjacency predicate's vertex-index comparison.

## What this sub-leaf commits the book to

- **`sim-soft`'s proximity filter is a two-stage sharp filter** on top of the [BVH broadphase](00-bvh.md): (1) primitive-type grouping (vertex-triangle → point-triangle, edge-edge → edge-edge; drop other combinations), (2) adjacency exclusion (drop point-triangle pairs where the point is a vertex of the triangle; drop edge-edge pairs sharing a vertex).
- **Adjacency exclusion is a correctness requirement, not a performance optimization.** The barrier $b(0) = +\infty$ would fire on initial configurations without it. The filter is topology-only and invariant under deformations, so it can be precomputed per topology and reused across all timesteps until topology changes.
- **No self-vs-cross branch.** The adjacency predicate's vertex-index-based rule handles both cases identically, which is what makes the [§03-energy](../01-ipc-internals/03-energy.md) no-special-case claim hold one layer down.
- **Geometric subcase classification is downstream of the proximity filter**, not inside it. The filter emits (pair-type, primitive-ids); narrow-phase distance code classifies interior/edge/vertex for point-triangle and parallel/skew for edge-edge as part of distance computation.
