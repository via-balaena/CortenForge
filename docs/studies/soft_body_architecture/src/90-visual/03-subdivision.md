# Subdivision surfaces

The `sim-soft` tet mesh is optimized for the solver, not the renderer. A 30k-tet canonical scene has a surface triangulation of ≈12k triangles — faceted if rendered directly, visibly polygonal on curved features like the probe's rim and the cavity's interior. The thesis commits to [the physics mesh being the render mesh](../10-physical/03-thesis.md), which seems at odds with "the render mesh needs to be smooth." The resolution: **subdivision surfaces** — the coarse sim-mesh is subdivided at render time into a smooth mesh, with the subdivision operation preserving the per-vertex physics attributes via interpolation.

| Section | What it covers |
|---|---|
| [Coarse sim mesh → smooth render mesh](03-subdivision/00-coarse-to-fine.md) | The pipeline: extract surface triangulation from the tet mesh, subdivide 1–2 levels, interpolate per-vertex physics attributes (stress, temperature, thickness, contact pressure) onto the subdivided mesh, shade |
| [Loop vs Catmull-Clark](03-subdivision/01-loop-vs-cc.md) | Loop (Charles Loop 1987) for triangle meshes, Catmull-Clark (Catmull and Clark 1978) for quad meshes. `sim-soft`'s surface extraction is triangle-based, so Loop is the default; Catmull-Clark is available if a surface is re-tessellated to quads |

Three claims.

**Loop subdivision is the default.** `sim-soft`'s surface extraction from a tet mesh produces a triangle triangulation natively (one triangle per boundary tet face). Loop subdivision — which refines each triangle into 4 smaller triangles per level while smoothing the position via a weighted stencil over neighbors — is the native fit. One level doubles vertex count; two levels quadruple. For the canonical scene (12k boundary triangles), one level gives ≈48k render-triangles, two levels ≈192k — both within GPU-rasterization budget.

**Physics attributes interpolate through subdivision, exactly.** Subdivision introduces new vertices whose positions are weighted averages of parent vertices. The same weighted average applies to per-vertex physics attributes — stress (interpolated linearly from parents), temperature, contact pressure, layer assignment (nearest-parent for layer ID). The interpolation is differentiable in the attribute values, so gradient flow through subdivision to rendered features is smooth. Gradients w.r.t. a designer's parameter propagate through subdivision into the shader outputs cleanly.

**Subdivision does not re-enter the physics.** The subdivided render mesh is a render-only artifact. The physics still runs on the coarse Tet4 mesh (or Tet10 in contact bands per [Part 3 Ch 00](../30-discretization/00-element-choice.md)); the subdivision is a post-physics, pre-rasterization step. This matters for the `sim-bevy` shader pipeline ([Ch 05](05-sim-bevy.md)) — the subdivided mesh is built once per physics step and re-used for that step's rendering, not re-subdivided per frame unless the physics mesh changed. Compute-level caching saves the per-frame subdivision cost when the simulation is paused or the physics step hasn't advanced.
