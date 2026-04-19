# Frictional sliding interfaces

The [interfaces parent](../03-interfaces.md) named sliding as a Phase H deliverable, contrasting it with the [Phase B bonded default](00-bonded.md). This leaf writes down what "sliding" means at the [`mesh/`](../../../110-crate/00-module-layout/02-mesh.md) and [`contact/`](../../../110-crate/00-module-layout/03-contact.md) module level — the mesh has a double-layer of vertices on the interface, IPC handles the contact between layers, and friction lives in the IPC-friction integration from Part 4 — and identifies the Phase H scope and the regression-baseline gate.

## What "sliding" means at the mesh level

A frictional sliding interface between two material regions A and B is represented by a **double layer of vertices** on the interface: every interface point has two distinct vertex IDs, one belonging to the A-side mesh, one belonging to the B-side mesh. The two vertices share a position in the reference configuration but are independent DOFs in the global system. Tangential relative motion between A and B is not constrained — the two sides slide past each other, with friction handled by the [Part 4 Ch 02 IPC-friction](../../../40-contact/02-friction.md) machinery rather than by the mesh structure.

The data-structure picture differs from the [bonded sub-leaf](00-bonded.md) in three ways:

- **Vertex count doubled at the interface.** Each interface vertex appears twice in the global vertex array. The [`mesh/`](../../../110-crate/00-module-layout/02-mesh.md) module's interface-pair table records the (A-vertex, B-vertex) correspondence so that the [`contact/`](../../../110-crate/00-module-layout/03-contact.md) module can restrict its IPC contact-pair search to these pairs (the geometry near the interface is intentionally close, and an undirected proximity query would generate spurious contacts).
- **Per-tet material assignment is non-ambiguous at the interface.** A-side tets reference A-side vertices only; B-side tets reference B-side vertices only. There is no straddling tet, so the [Part 7 Ch 02](../../../70-sdf-pipeline/02-material-assignment.md) per-tet centroid sampling is unambiguous.
- **IPC contact-pair generation runs across the interface continuously.** The [Part 4 Ch 03 BVH](../../../40-contact/03-self-contact/00-bvh.md) detects contact between A-side surface triangles and B-side surface triangles; the interface-pair table tells IPC that these are *expected* contacts (not spurious self-contacts) and that they should be tracked across the simulation rather than treated as transient.

The result: a multi-layer cavity (a silicone outer sleeve with a softer silicone inner liner sliding inside it) is two meshes joined at the interface by IPC contact, with friction between layers determined by the [`FrictionalContact` material parameters](../../../40-contact/02-friction/00-coulomb.md).

## Why this is Phase H, not Phase B

Three reasons sliding is deferred to Phase H:

- **Mesh-generation complexity.** Producing a tet mesh with a double-layer of interface vertices from an SDF requires either (a) running [tet generation](../../../70-sdf-pipeline/01-tet-strategies.md) twice on the two regions and stitching the interface, or (b) post-processing a single-mesh output to duplicate interface vertices and re-tag tet-vertex references. Both are non-trivial extensions to the [`sdf_bridge/`](../../../110-crate/00-module-layout/08-sdf-bridge.md) ingestion path. The bonded case requires neither.
- **IPC active-set growth.** The IPC contact-pair search now has a permanent set of expected-active pairs at every interface, in addition to the transient external contacts. This grows the IPC active set proportionally to the interface area; the [Part 4 Ch 05 real-time IPC](../../../40-contact/05-real-time.md) chapter's cost analysis already includes this growth in its Phase H sizing.
- **Friction parameter authoring.** A bonded interface needs no parameters (continuity is structural); a sliding interface needs a Coulomb friction coefficient $\mu_c$ at minimum, and likely a smoothed-Coulomb regularization parameter ([Part 4 Ch 02](../../../40-contact/02-friction/01-smoothed.md)) per interface. Authoring these parameters, validating them, and exposing them through the [Part 7 Ch 02 material-assignment API](../../../70-sdf-pipeline/02-material-assignment.md) is non-trivial UX work the Phase H sliding-deliverable absorbs.

The sliding interface is the Phase H deliverable specifically tied to the multi-layer-cavity workload from [Part 4 Ch 04 multi-layer](../../../40-contact/04-multi-layer.md). Phase B does not need it.

## Composition with the existing IPC machinery

The friction model is not a new mechanism for sliding interfaces — it is the same [Part 4 Ch 02 IPC-friction](../../../40-contact/02-friction/02-ipc-friction.md) integration that handles all contact-friction in `sim-soft`. The interface-pair table is metadata that tells the IPC layer which contacts to expect; it does not change the friction model itself.

This is the architectural simplification: a sliding interface is "just contact, with metadata". No `slidingInterface/` module exists; the relevant code lives in [`contact/`](../../../110-crate/00-module-layout/03-contact.md) (the interface-pair table and the IPC active-set restriction), [`mesh/`](../../../110-crate/00-module-layout/02-mesh.md) (the double-layer vertex generation at ingest), and [`material/`](../../../110-crate/00-module-layout/00-material.md) (the friction-parameter authoring). The trait surfaces of these modules do not change.

## Regression-baseline gate

The Phase H sliding-deliverable certification requires regression tests against:

- **Analytic baselines where they exist.** Two-layer simple-shear with a known friction coefficient has analytic stress-strain curves; the sliding-interface implementation must match these to engineering tolerance.
- **MuJoCo flex baseline runs** for problems both can express. MuJoCo flex's contact-and-friction behavior is the [Part 0 Ch 02 SOTA reference](../../../00-context/02-sota/08-mujoco-flex.md) for this class of problem; matching its qualitative behavior on the multi-layer canonical-problem variant is the Phase H acceptance criterion.
- **Manufactured-solution tests** for cases without analytic or external baselines. The [Part 11 Ch 04 regression-test sub-leaf](../../../110-crate/04-testing/01-regression.md) gates the sliding deliverable on these alongside the per-impl unit tests for the interface-pair-table machinery.

## What this sub-leaf commits the book to

- **Sliding is double-layer-vertex + IPC-friction, not a new element type.** The mesh has duplicated interface vertices; the contact layer handles cross-interface contact; friction comes from the existing IPC-friction model. No `SlidingElement` or `JointPrimitive`.
- **Interface-pair table lives in [`contact/`](../../../110-crate/00-module-layout/03-contact.md).** Records the expected (A-vertex, B-vertex) correspondence; restricts IPC active-set search to expected pairs to avoid spurious self-contacts at the interface geometry.
- **Phase H scope is the multi-layer cavity workload only.** Other sliding-interface use cases (e.g., a soft-body rolling element on a rigid surface) are derivative — the same machinery, different parameters — and may ship with Phase H without re-architecting the layer.
- **Regression baseline is analytic + MuJoCo flex + manufactured-solution.** No single source is sufficient on its own; the trio gates the deliverable.
