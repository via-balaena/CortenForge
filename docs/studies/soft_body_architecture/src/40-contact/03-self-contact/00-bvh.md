# Bounding volume hierarchies

Broadphase is the "deliver all primitive pairs with gap less than $\hat d$" query. [The IPC barrier](../01-ipc-internals/00-barrier.md) has compact support on $(0, \hat d]$ — pairs with $d \ge \hat d$ contribute identically zero energy, zero gradient, and zero Hessian — so the broadphase can cull them entirely and nothing downstream knows the difference. Whichever data structure makes the $\hat d$-radius query fast is the right broadphase.

[IPC Toolkit](../../appendices/00-references/00-ipc.md#ipc-toolkit) ships six broadphase implementations (BruteForce, HashGrid, SpatialHash, LBVH, SweepAndPrune, and a GPU SweepAndTiniestQueue variant) behind a common `BroadPhase` interface, with HashGrid as the CPU default. This sub-leaf covers the bounding volume hierarchy family — specifically the LBVH variant — because it is the data structure the [Part 4 Ch 05 real-time chapter](../05-real-time.md) forward-references as the wgpu compute-kernel target: hierarchical pruning that parallelizes cleanly on both CPU and GPU under a single algorithmic design, rather than a CPU-tuned spatial-grid bucket structure that needs re-architecting for GPU.

## Bounding volume choice: AABB

Axis-aligned bounding boxes. Tighter volumes (oriented bounding boxes, k-DOPs) fit deforming primitives more closely but cost more at every operation the broadphase does frequently — overlap testing is 6 float comparisons for AABB vs. 15 dot products plus separating-axis enumeration for OBB, and refitting is a min/max sweep for AABB vs. a full basis recomputation for OBB. Deforming meshes refit every Newton iteration (below); the refit cost dominates the per-node volume tightness. AABB is the canonical choice and is what IPC Toolkit's LBVH uses.

Dilating an AABB by $\hat d$ — the operation required to realize the "within-$\hat d$" query from a plain overlap query — is a trivial $\pm\hat d$ on each min/max face. OBB dilation has no such trivial form.

## Tree construction: LBVH via Morton codes

The LBVH (Linear BVH) construction in IPC Toolkit's `src/ipc/broad_phase/lbvh.{cpp,hpp}` is:

1. Compute the centroid of each surface primitive (vertex, edge, or triangle, depending on the query).
2. Map each centroid to normalized $[0, 1]^3$ space and interleave the $x, y, z$ bits into a single **Morton code** — a Z-order space-filling curve that places spatially close centroids at close integer positions.
3. Sort primitives by Morton code (parallel radix sort).
4. Build the binary tree top-down following the sorted order: each internal node's split is the position where the most significant differing bit of the Morton code changes, placing spatially close primitives as sibling leaves under a common parent.
5. Propagate leaf AABBs up to the root: each internal node's AABB is the union of its children's. Bottom-up fill is a parallel reduction.

Total construction is dominated by the sort ($O(n \log n)$); the tree topology build and bounds propagation are $O(n)$. Every step is a data-parallel primitive (sort, scan, reduction), which is what makes LBVH the natural GPU structure: the same algorithm ports to wgpu compute kernels without redesign.

## Static-proximity vs. CCD: same tree, different leaf bounds

Two query patterns share the same AABB-tree structure but differ in how leaf bounds are computed:

- **Static proximity** (barrier energy assembly at Newton iterate $x$): find pairs with current gap $d < \hat d$. Per-primitive AABB encloses the primitive's vertices at $x$; the "within-$\hat d$" query is realized either by dilating each leaf AABB by $\hat d$ at insertion time, or by testing overlap with a $\hat d$-dilated predicate during traversal — equivalent in output, an implementation choice only.
- **CCD broadphase** (line-search filter over step $[x_a, x_b]$, per [§02-ccd](../01-ipc-internals/02-ccd.md)): find pairs that could touch zero distance anywhere in $\alpha \in [0, 1]$. Per-primitive AABB is the **swept AABB** enclosing the primitive's vertices at both $x_a$ and $x_b$.

IPC Toolkit's `BroadPhase` base class has overloaded `build()` methods for the two cases — `build(vertices, edges, faces, inflation_radius)` for static, `build(vertices_t0, vertices_t1, edges, faces, inflation_radius)` for CCD — with the same tree structure and traversal logic downstream. Only the leaf-AABB construction differs.

## Update policy: refit within a solve, rebuild on topology change

Across Newton iterations within a single timestep's solve, mesh topology is invariant — vertex positions change but connectivity (which primitives exist, which are siblings in the tree) does not. Leaf AABBs are updated to the new vertex positions and the change is propagated up the tree in a single bottom-up pass; the tree topology is reused. This **refit** operation is $O(n)$.

When topology changes — [Ch 02 Sec 02 contact-driven h-refinement](../../30-discretization/02-adaptive/02-contact-driven.md) splits a tet and its incident surface triangles, for instance — the existing tree is invalidated (primitives have been added or removed, leaf count has changed) and a full rebuild is required. Rebuild is $O(n \log n)$ dominated by the Morton sort.

The typical pattern during a solve: rebuild at the start of the timestep if topology has changed since the previous step; refit between Newton iterations within the step. This split is standard practice in deforming-mesh collision detection; the cost of a single rebuild is amortized across many refit iterations, and refitting preserves the tree's balance well for small per-iterate vertex motions. When deformations grow large enough that the tree becomes significantly unbalanced (leaf AABBs overlap siblings' AABBs heavily), full rebuild recovers the pruning efficiency — the refit-vs-rebuild decision in pathological cases is a policy detail, not a correctness issue.

## Query traversal

With a built tree, the "all pairs with gap $< \hat d$" query is a simultaneous double-tree traversal. For self-contact, the tree is traversed against itself. At each internal-node pair, test whether the two AABBs overlap when dilated by $\hat d$; if not, prune the entire subtree product. If they overlap, recurse on the four child-pair combinations. At leaf-leaf pairs, emit the pair as a broadphase candidate for narrow-phase evaluation ([proximity filter next](01-proximity.md), then exact distance in the energy assembly).

The traversal is sub-quadratic because most internal-node pairs prune. Output size is roughly $O(N \cdot K)$ where $K$ is the average number of proximate primitives per primitive — a geometry-dependent constant in the 5–50 range for typical deforming contact configurations. The query's total cost is bounded by the output size plus the traversal overhead (logarithmic in $N$ per reported pair), giving several orders of magnitude pruning over the $O(N^2)$ naive enumeration at the canonical problem's resolution.

## What this sub-leaf commits the book to

- **`sim-soft` commits to the BVH family for broadphase, specifically an LBVH implementation.** HashGrid is the CPU default in [IPC Toolkit](../../appendices/00-references/00-ipc.md#ipc-toolkit), but the [Part 4 Ch 05 GPU lever](../05-real-time/00-gpu.md) requires a hierarchy-based structure (the Morton-sort + tree-construction + AABB-propagation pipeline is the GPU-parallel path); choosing HashGrid on CPU and something else on GPU doubles the broadphase maintenance. One structure across both.
- **AABB bounding volumes, Morton-code tree construction, refit-within-solve + rebuild-on-topology-change update policy.** The bounding-volume and construction choices match IPC Toolkit's `lbvh.{cpp,hpp}`; the update policy is standard deforming-mesh collision-detection practice. The resulting tree is a $O(n)$-memory parallel data structure, not an opaque black box.
- **Static-proximity and CCD broadphases share one tree structure**, differing only in leaf-AABB construction. [Part 8 Ch 00 kernel types](../../80-gpu/00-wgpu-layout.md) exploits this by sharing tree-traversal code between the two per-contact kernels.
- **The $\hat d$-radius query is the broadphase contract.** Downstream code ([barrier assembly](../01-ipc-internals/03-energy.md), [contact-driven mesh adaptivity](../../30-discretization/02-adaptive/02-contact-driven.md), [cross-surface interface tracking](../../30-discretization/03-interfaces/01-sliding.md)) calls this one broadphase query; the tree structure and Morton-code details stay inside the broadphase module.
