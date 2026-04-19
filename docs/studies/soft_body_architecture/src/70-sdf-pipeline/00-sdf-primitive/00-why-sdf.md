# Why SDFs over meshes

The [Ch 00 parent's Claim 1](../00-sdf-primitive.md) names three otherwise-independent concerns — geometry composition, material fields, differentiability — that all converge on the SDF abstraction. This leaf expands the argument with concrete failure modes the mesh-first alternative would incur in `sim-soft`, so that the "three concerns, one abstraction" economy in the parent is defended rather than asserted.

## Concrete failure modes of mesh-first composition

The [canonical problem](../../10-physical/00-canonical.md) mixes a small number of primitives — cylindrical cavity, probe, mount, optional layered sleeves — and allows the designer to sweep parameters (probe radius, cavity wall thickness, blend radii) across a range. A mesh-first pipeline would compose those primitives by boolean-ing polygon meshes, and each boolean operation would go through a library like [CGAL's `Polygon_mesh_processing::corefine_and_compute_difference`](https://doc.cgal.org/latest/Polygon_mesh_processing/) or an equivalent mesh-boolean kernel.

Three failure modes then become first-class concerns:

- **Self-intersection preconditions.** CGAL's mesh-boolean operations require that input meshes be closed, non-self-intersecting, and bound a volume. The manual flags these preconditions explicitly and fails the operation if they are violated. A designer who sweeps a probe radius across a range where the probe grazes the mount will produce self-intersecting intermediate meshes, and the boolean operation refuses to proceed until the upstream mesh is repaired. Repair is its own multi-step pipeline and not a drop-in fix.
- **Coplanar-face degeneracies.** When two primitive boundaries graze each other tangentially (a cylindrical cavity meets a cylindrical mount along a circular curve), the polygon meshes have near-coplanar triangles along that curve, and the boolean's segmentation of the triangles into "inside" and "outside" the other mesh is numerically fragile. Exact-predicate kernels make the topological decisions correct in principle, but the resulting mesh near the grazing curve contains sliver triangles that propagate into the tet mesh and become the aspect-ratio failures [Ch 01 Claim 3](../01-tet-strategies.md) targets.
- **T-junction artifacts.** Boolean operations produce new vertices where input edges are cut by other meshes; those new vertices are on existing edges of the other mesh, but the other mesh's triangulation is not updated to include them as explicit vertices. The result is a T-junction, which is geometrically consistent but FEM-unfriendly. Cleanup passes exist, but add another step whose failure modes compound with the boolean's.

Composing the same three primitives as SDFs requires evaluating $\min$, $\max$, or weighted blends of scalar fields at each query point. No mesh, no preconditions, no degeneracies, no T-junctions. A designer who sweeps a probe radius through a grazing configuration gets a continuous family of SDFs without a topology-cleanup step.

## Material fields compose through the same algebra

The [Part 1 Ch 03 thesis](../../10-physical/03-thesis.md) commits `sim-soft` to spatially-varying material parameters — stiffness ramped from skin to core, fiber direction following a braided pattern, Prony-relaxation times differing across a bonded multi-material. "Spatially varying" in practice means "scalar-valued (or vector-valued) function of reference-space position," which is the SDF formulation applied to material parameters rather than to geometry.

A designer who wants a stiff skin over a soft core writes: (a) a radial SDF for the skin's inner boundary, (b) a stiffness field that is `mu_stiff` outside the boundary and `mu_soft` inside, with an SDF-distance-weighted blend across a transition band. The second step uses the same operator algebra from [§01 operations](01-operations.md) that composed the geometric SDF. The same `cf-design` authoring machinery produces both, [Part 2 Ch 09 spatial-fields](../../20-materials/09-spatial-fields.md) consumes the material side, and [Ch 02 material-assignment](../02-material-assignment.md) samples it per-tet.

Having one abstraction for scalar (and typed) fields over the body is the architectural economy the parent's Claim 1 names — not because two abstractions are expensive to maintain, but because authoring, composition, and differentiation machinery can be shared. A carbon-black-loaded skin that is also stiffer than the substrate produces two fields that compose by the same rules as the geometry, sampled at the same reference-space points, differentiated through the same chain.

## Differentiability is localized to one boundary

[Part 6 Ch 05's differentiable-meshing open problem](../../60-differentiability/05-diff-meshing.md) names the structural obstruction: meshing operations (marching cubes, Delaunay, fTetWild) have piecewise-constant output topology and are not differentiable through topology change at the platform level. `sim-soft` commits to a three-part compromise — topology fixed per episode, between-episode re-mesh with warm-started state transfer, FD-wrapped gradients at topology boundaries — rather than solving the open problem.

The SDF abstraction is what makes that compromise localizable. The designer's edit is a scalar-field perturbation (a primitive-parameter slider move, a material-field blend-weight change), which is differentiable by construction. The non-differentiable step is the SDF-to-tet conversion inside [Ch 01's meshing pipeline](../01-tet-strategies.md), and the [FD wrapper](../../60-differentiability/05-diff-meshing.md) applies exactly there. [Part 6 Ch 02's implicit-function-theorem adjoint](../../60-differentiability/02-implicit-function.md) handles the forward solve's differentiability cleanly for all parameter-only edits.

If the design surface were mesh-valued, differentiability would bleed into every vertex-move (smooth path) and every topology edit (non-smooth path), interleaved across the same edit stream, and the FD wrapper would have no clean seam to attach to. The SDF abstraction gives the FD wrapper a single well-defined boundary — the SDF evaluator feeds the mesher; the mesher emits tets; everything before the mesher is differentiable by construction; everything through the mesher is FD-wrapped.

## What this sub-leaf commits the book to

- **Mesh-first composition carries a repair pipeline.** Self-intersection preconditions, coplanar-face degeneracies, and T-junctions are documented failure modes of polygon-mesh boolean libraries; each requires a cleanup step that compounds with the next operation. `sim-soft` does not ship a mesh-repair pipeline; the SDF abstraction is how that pipeline is avoided.
- **One scalar-field abstraction serves geometry and material.** The [`SdfField` and `MaterialField`](../00-sdf-primitive.md) types share authoring, composition, and per-element sampling machinery; the solver does not know which side of the boundary (geometry vs. material) a given field came from.
- **Differentiability is confined to one seam.** The FD wrapper from [Part 6 Ch 05](../../60-differentiability/05-diff-meshing.md) attaches at the SDF-to-tet conversion in [Ch 01](../01-tet-strategies.md); everywhere else in the pipeline, gradients flow by construction.
