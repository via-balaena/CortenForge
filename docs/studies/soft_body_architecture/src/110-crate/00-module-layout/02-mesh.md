# mesh/

The `mesh/` module stores tetrahedral mesh topology, adjacency, quality metrics, and adaptive-refinement operations. It sits alongside [`material/`](00-material.md) as one of the two standalone modules in the crate — no internal `sim-soft` dependencies — and is shipped in Phase A of the [build order](../03-build-order.md#the-committed-order) for exactly that reason: a mesh library that compiles and passes its own tests before any solver machinery exists is the foundation [`element/`](01-element.md), [`contact/`](03-contact.md), and [`solver/`](04-solver.md) all build on.

## What the module owns

| Sub-area | Responsibility | Where it is specified |
|---|---|---|
| Tet mesh storage | Vertex positions, tet-vertex connectivity, rest-pose geometry, optional per-tet material-region tag | [Part 3 Ch 00](../../30-discretization/00-element-choice.md) |
| Adjacency | Tet–tet, vertex–tet, edge–tet, face–tet queries used by contact broadphase and refinement | [Part 3 Ch 02](../../30-discretization/02-adaptive.md) |
| Quality metrics | Aspect ratio, dihedral-angle histogram, volume consistency — inputs to the refinement trigger | [Part 3 Ch 01](../../30-discretization/01-mesh-quality.md) |
| Adaptive refinement | Stress-driven h-refinement, p-refinement, contact-proximity-driven subdivision; all operate on the stored mesh and emit a new one | [Part 3 Ch 02](../../30-discretization/02-adaptive.md) |
| Multi-material regions | Region tags shared with [`material/`](00-material.md)'s `Field` system for spatial material assignment | [Part 3 Ch 03](../../30-discretization/03-interfaces.md) |

## Three claims

**1. Tet-only. Hex rejected.** The module stores tetrahedra exclusively. Hex8 was considered and rejected at the element-choice layer ([Part 3 Ch 00 §02](../../30-discretization/00-element-choice/02-hex8.md)) because robust hex meshing from an arbitrary SDF input is an unsolved problem — and the [SDF-as-universal-primitive commitment in Part 7 Ch 00](../../70-sdf-pipeline/00-sdf-primitive.md) upstream of this module leaves tet meshes as the only realistic downstream geometry. The storage layout is therefore tet-specialized: tet-vertex connectivity as a flat `[u32; 4]` array, per-tet region tag as `u16`, vertex position as `Vec3<f64>`. Adding hex storage later is not impossible, but it is not a trait-variant upgrade — it would require duplicating the storage types and adjacency queries, which is exactly the complexity the Part 3 rejection buys us out of.

**2. Adaptive refinement is a mesh operation, not a solver operation.** Stress-driven h-refinement takes a stored mesh and a per-tet stress field and emits a new mesh with targeted tets subdivided; the solver is not involved beyond providing the field. This keeps the refinement trigger loop outside the Newton iteration: a refinement pass happens between timesteps (or between design-space evaluations), never inside one. The [Part 7 Ch 03 adaptive-refinement chapter](../../70-sdf-pipeline/03-adaptive-refine.md) names the criterion as "von Mises exceeds a per-region threshold" — a scalar field that [`readout/`](09-readout.md) produces as part of the standard observation extraction, consumed here through the `Field` trait, no tight coupling.

**3. Change detection lives here, not in [`sdf_bridge/`](08-sdf-bridge.md).** Hash-based mesh change detection — same topology, same vertices, same region tags? — is a mesh-level equality predicate. [`sdf_bridge/`](08-sdf-bridge.md)'s [live-re-mesh chapter](../../70-sdf-pipeline/04-live-remesh.md) uses this predicate to classify a design edit as `ParameterOnly` (mesh-identical warm start), `MaterialChanging` (mesh-identical, material-field differs), or `TopologyChanging` (mesh differs, requires re-mesh with state transfer). The classification logic is `sdf_bridge/`'s; the mesh-equality test it rests on is `mesh/`'s, because mesh identity is a mesh-module concept not an SDF-module concept.

## What the module does not carry

- **No materials.** Per-region tags are integer indices; the material itself lives in [`material/`](00-material.md) keyed by tag. A mesh of 1000 tets with three material regions has three `impl Material` instances at scene-construction time; the mesh stores only the integer tag per tet.
- **No SDF or primitive-tree knowledge.** The mesh is a concrete triangulation of a body. Where that triangulation came from — authored directly, imported from a file, emitted by [`sdf_bridge/`](08-sdf-bridge.md) — is not the mesh module's concern. The mesh is the handoff format, not the source.
- **No solver state.** No per-tet scratch, no sparse pattern, no factorization. The mesh is a static-between-timesteps object; dynamic state lives in [`solver/`](04-solver.md)'s arena.

## What this commits downstream

- **[`element/`](01-element.md) assumes per-region homogeneous element type.** Phase H Tet10-in-contact-band per [Part 3 Ch 00 §03](../../30-discretization/00-element-choice/03-mixed.md) stores the element-type choice per region via the existing tag mechanism; no new storage.
- **[`contact/`](03-contact.md)'s broadphase consumes mesh adjacency directly** for self-contact proximity queries. The BVH [`contact/`](03-contact.md) maintains is built over the mesh's surface-triangle projection and its adjacency, not over raw vertex positions — see [Part 4 Ch 03 §00](../../40-contact/03-self-contact/00-bvh.md).
- **[`sdf_bridge/`](08-sdf-bridge.md)'s warm-start path** transfers solver state across re-meshes using the stored mesh-equality predicate plus per-tet interpolation; the predicate is a mesh-module export, the interpolation is [Part 7 Ch 04 §02](../../70-sdf-pipeline/04-live-remesh/02-state-transfer.md)'s concern.
- **[Part 11 Ch 04 unit tests](../04-testing/00-unit.md)** for `mesh/` run in Phase A with no dependencies on any other `sim-soft` module — the 20-line gradcheck harness commitment from [Part 11 Ch 03](../03-build-order.md#the-committed-order) binds on this module as much as on [`material/`](00-material.md).
