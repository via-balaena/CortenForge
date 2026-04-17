# Element choice

FEM discretization starts with a question that every soft-body solver has to answer: **which finite element type do the stress, stiffness, and deformation live on?** The choice is among three mainstream families (linear tetrahedra, quadratic tetrahedra, trilinear hexahedra) plus compositional variants (mixed-element meshes). Each has well-understood tradeoffs in cost, accuracy, integration with contact, and compatibility with the [SDF → tet pipeline](../70-sdf-pipeline/00-sdf-primitive.md). `sim-soft` commits to **Tet4 as the Phase B default, Tet10 as the Phase H upgrade, Hex8 rejected, mixed-element deferred** — and this chapter is where that commitment is made.

| Section | What it covers |
|---|---|
| [Tet4 — linear tetrahedra](00-element-choice/00-tet4.md) | 4 nodes per element, linear shape functions, constant strain per tet. Cheapest per-element; industry standard. Phase B's element, shipped in every `sim-soft` milestone |
| [Tet10 — quadratic tetrahedra](00-element-choice/01-tet10.md) | 10 nodes per element (4 corners + 6 edge midpoints), quadratic shape functions, linear strain per element. 3–5× per-element cost of Tet4 but captures curved contact surfaces correctly. Phase H fidelity upgrade |
| [Hex8 — trilinear hexahedra](00-element-choice/02-hex8.md) | 8 nodes per element, trilinear shape functions. Lower error per-DOF than Tet4, but hex meshing from arbitrary SDFs is an unsolved robust-meshing problem. Rejected for `sim-soft`'s workflow |
| [Mixed element meshes](00-element-choice/03-mixed.md) | Tet4 in bulk, Tet10 in contact bands and stress-concentration zones. Natural fit for `sim-soft`'s targeting pattern but adds assembly complexity. Deferred beyond Phase H |
| [Cost/accuracy tradeoff](00-element-choice/04-tradeoff.md) | Quantitative comparison on the canonical problem: per-DOF error, per-element cost, conditioning, contact compatibility. The numbers behind the commitment above |

Four claims this chapter commits to.

## 1. Tet4 is the Phase B default because simplicity wins

The Tet4 element has 4 nodes (one per tetrahedron corner), linear shape functions, and therefore *constant* deformation gradient $F$, constant strain, and constant stress within each tet. The per-element stiffness assembly is a 12×12 matrix built from one deformation-gradient evaluation and one constitutive-law evaluation. The `element/` module's trait surface ([Part 11 Ch 00](../110-crate/00-module-layout/01-element.md)) is simplest at Tet4: no Gauss-point quadrature, no per-element shape-function inversion, no edge-midpoint nodes to track. The [`Material::tangent()`](../20-materials/00-trait-hierarchy/00-trait-surface.md) method is called once per tet, not once per quadrature point.

The downside is that constant-strain Tet4 suffers from two well-documented problems: **volumetric locking** under near-incompressibility (silicone is $\nu \approx 0.49$; Tet4 treats the volumetric constraint too stiffly and the mesh "locks" into an artificially rigid response) and **rim-deformation inaccuracy** (the constant-strain assumption cannot represent a tet that is bending around a contact edge, which is exactly the failure mode [Part 1 Ch 02](../10-physical/02-what-goes-wrong/04-rim.md) names). Both are addressed in later phases — [mixed u-p formulations](../20-materials/05-incompressibility/01-mixed-up.md) handle the incompressibility lock at the material layer in Phase H; Tet10 handles the rim-deformation problem at the element layer in the same phase. Phase B does not ship either of these; the canonical problem's Phase D deliverable is known to have visible Tet4-inherited errors on rim and under compression.

The Phase D deliverable ships anyway because the *differentiability* and *solver-correctness* properties of the stack are what Phase D is validating. Visual and quantitative fidelity on the rim and under compression are Phase H concerns. Shipping Phase D with Tet4-only is an explicit decision, not an oversight — the [`milestone — first working sim-soft` from build-order](../110-crate/03-build-order.md#the-committed-order) is end-to-end-gradient, not end-to-end-beautiful.

## 2. Tet10 is the Phase H upgrade because rim deformation demands it

Tet10 adds 6 midpoint nodes (one per edge) to the 4-corner Tet4, yielding 10 nodes per element and quadratic shape functions. The deformation gradient varies linearly across the element rather than being constant; strain and stress are therefore linear; bending modes are representable inside a single tet. The rim-deformation failure that Tet4 cannot represent — a glove wrapping around a finger edge with high curvature across a single element — becomes a first-class deformation mode in Tet10.

The cost is ≈3–5× Tet4 per-element (more nodes, more Gauss points, denser stiffness submatrix), and the stiffness matrix has ≈2.5× the nonzero count for the same tet count. The net wall-clock penalty depends on what dominates — per-element assembly or per-DOF sparse solve. For the canonical problem at Phase E resolution on GPU, Tet10 costs ≈3× the Tet4 runtime but delivers visible improvement at the rim that Phase H's fidelity targets demand.

**Gauss-point-based constitutive evaluation.** Tet10's strain is non-constant, so the material law is evaluated at multiple quadrature points per tet (typically 4 Gauss points for quadratic integration). The [`Material::tangent()`](../20-materials/00-trait-hierarchy/00-trait-surface.md) trait is called once per Gauss point rather than once per tet; this has consequences for [material-field sampling](../70-sdf-pipeline/02-material-assignment/00-sampling.md) (interpolation at Gauss points rather than centroid) and for [adaptive refinement](../70-sdf-pipeline/03-adaptive-refine.md) (stress evaluation is already per-quadrature-point, aligning naturally).

**Not mixing Tet4 and Tet10 globally in Phase H.** The Phase H commitment is that the mesh is homogeneously Tet10 in the contact band and Tet4 elsewhere, with a conforming interface at the band boundary. A globally-Tet10 mesh is wasteful (bulk regions don't need the precision); a fully-mixed mesh requires per-element element-type tagging and more careful stiffness assembly. The [mixed-element sub-chapter](00-element-choice/03-mixed.md) documents this targeting pattern.

## 3. Hex8 is rejected because you cannot robustly hex-mesh an arbitrary SDF

The linear hexahedral element (Hex8) has 8 nodes and trilinear shape functions. Per-DOF accuracy is generally better than Tet4 — hex elements suffer less from volumetric locking and require fewer DOFs for the same geometric-detail budget. Across the industrial-FEM world, Hex8 is the usual first pick for structured domains: blocky geometries, extruded profiles, pressure-vessel walls, anything that can be meshed by sweeping a 2D pattern along an axis. For those problems, Hex8 is the right element.

`sim-soft` rejects Hex8 because the workflow upstream of the element layer is **SDF → tet mesh** ([Part 7 Ch 01](../70-sdf-pipeline/01-tet-strategies.md)), and robust hexahedral meshing from an arbitrary input SDF is an unsolved problem. Tetrahedral meshing from SDF has mature tooling ([fTetWild](../70-sdf-pipeline/01-tet-strategies/00-ftetwild.md) is the default); hex meshing from SDF requires either (a) a designer manually laying out the hex block structure before meshing, which breaks [Part 7 Ch 00's SDF-as-primitive commitment](../70-sdf-pipeline/00-sdf-primitive.md); or (b) automated hex-dominant meshing, where the literature is a collection of heuristics that work on some inputs and fail on others, with no robust guarantees. Neither is acceptable for `sim-soft`'s SDF-driven design workflow.

Hex-dominant research (HexEx, Instant Meshes, et al.) is an active area, and a future `sim-soft` revision could add a Hex8 path once the robust-meshing problem has a clean answer. Pass 1 does not plan for it; the tet family covers the element-level needs for the problems `sim-soft` targets.

## 4. Mixed-element meshes are deferred beyond Phase H

The natural `sim-soft` targeting pattern is Tet4 in the bulk, Tet10 in contact and stress-concentration bands. Implementing this requires per-element element-type tagging, a transition-layer interface where Tet4 and Tet10 elements share edges (the Tet4 edge has 2 shape functions; the Tet10 edge has 3 shape functions because of the midpoint node; the interface requires constraint enforcement so that the shared edge's deformation is consistent across types), and careful handling in the sparse-Hessian assembly.

This is not architecturally hard, but it is a large increase in the `element/` module's complexity and regression-test surface. Phase H ships Tet10-in-band as a user-selected option (the user chooses which region gets Tet10 via an SDF mask); fully-automatic mixed-element refinement that adapts based on the [stress-gradient criterion from Part 7 Ch 03](../70-sdf-pipeline/03-adaptive-refine.md) is a post-Phase-I concern.

The deferral is explicit: Phase H is the fidelity-upgrade phase, and mixed-element is a fidelity upgrade whose complexity exceeds the single-phase budget once Tet10, mixed u-p, Prony viscoelasticity, and HGO anisotropy are also in it.

## What this commits downstream

- [Part 2 Ch 00 (material trait hierarchy)](../20-materials/00-trait-hierarchy.md)'s `Material` trait is called once per element in Tet4, once per Gauss point in Tet10. The trait surface does not change — the call-site pattern does.
- [Part 2 Ch 05 (incompressibility)](../20-materials/05-incompressibility.md) is the material-level companion to Tet10: mixed u-p formulations address the volumetric lock that Tet4 suffers from. Both land in Phase H.
- [Part 7 Ch 02 (material assignment)](../70-sdf-pipeline/02-material-assignment.md)'s centroid-sampling default is correct for Tet4; Tet10 requires per-Gauss-point sampling, which that chapter's [sampling sub-chapter](../70-sdf-pipeline/02-material-assignment/00-sampling.md) covers.
- [Part 8 Ch 01 (sparse matrix structures)](../80-gpu/01-sparse-matrix.md)'s BSR 3×3 blocking assumes per-vertex DOF grouping, which holds for both Tet4 and Tet10; the nonzero count per row changes but the block structure does not.
- [Part 11 Ch 00 element module](../110-crate/00-module-layout/01-element.md) ships Tet4 in Phase B and adds Tet10 in Phase H without changing the module's trait surface from [Ch 01](../110-crate/01-traits.md).
- [Part 11 Ch 04 (testing)](../110-crate/04-testing.md)'s regression suite validates against MuJoCo flex's Tet4 output in Phase D and adds Tet10 regression baselines in Phase H.

## What this does NOT commit

- **Hex-dominant future.** A post-Phase-I Hex8 or hex-dominant path is not ruled out; Pass 1 does not plan for it because the upstream SDF-meshing problem is unsolved.
- **Shell elements.** Thin-wall structures (e.g., balloon membranes) are sometimes modeled with shell or membrane elements. Not a current `sim-soft` workload; deferred without a commitment to ever add them.
- **Integration order for Tet10.** 4-Gauss-point is the Phase H default; reduced integration (1 Gauss point) is a known hex trick and has some Tet10 variants, but is not shipped. Sub-chapter work, not architectural.
