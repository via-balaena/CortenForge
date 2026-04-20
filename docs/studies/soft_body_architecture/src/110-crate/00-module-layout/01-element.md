# element/

The `element/` module carries shape functions, Gauss-point quadrature, and per-element stiffness assembly. It is where [`material/`](00-material.md)'s per-Gauss-point stress and tangent become per-tet residual and stiffness contributions — the layer that translates from "constitutive law at a point" to "stiffness block at a tet" and fans out to [`solver/`](04-solver.md)'s global assembly. The element choice commitments — **Tet4 in Phase B, Tet10 added in Phase H, Hex8 rejected, fully-mixed Tet4/Tet10 deferred post-Phase-H** — are [Part 3 Ch 00](../../30-discretization/00-element-choice.md)'s, and the module implements them without adding trait variants.

## What the module owns

| Sub-area | Responsibility | Where it is specified |
|---|---|---|
| Element trait | `Element` — shape functions $N_i(\xi)$, gradients $\nabla_\xi N_i$, Gauss-point weights, per-element DOF count | [Part 11 Ch 01](../01-traits.md) |
| Tet4 | Linear tet, 4 nodes, 1 Gauss point, constant strain. Phase B element | [Part 3 Ch 00 §00](../../30-discretization/00-element-choice/00-tet4.md) |
| Tet10 | Quadratic tet, 10 nodes, 4 Gauss points, linear strain. Phase H element | [Part 3 Ch 00 §01](../../30-discretization/00-element-choice/01-tet10.md) |
| Per-tet assembly | Strain-displacement matrix $B$, per-element residual and 12×12 / 30×30 stiffness block, PSD projection | [Part 5 Ch 00 Claim 2](../../50-time-integration/00-backward-euler.md) |
| FEM assembly VJP | Hand-written reverse-mode kernel against [`autograd/`](07-autograd.md), bypasses op-by-op tape recording | [Part 6 Ch 01 §01](../../60-differentiability/01-custom-vjps/01-fem-assembly.md) |

## Three claims

**1. Generic over `Material`, not over element type.** The per-element assembly loop takes `M: Material` as a single type parameter so the compiler monomorphizes the stress and tangent calls through the Gauss-point quadrature — one assembly-loop codepath per element type, but the constitutive-law surface is generic. This is the matching half of [Part 2 Ch 00 §00](../../20-materials/00-trait-hierarchy/00-trait-surface.md)'s trait-surface budget: [`material/`](00-material.md) commits to four items; `element/` commits to calling those four items with a deformation gradient derived from shape-function gradients and nodal positions. Tet4 and Tet10 are different concrete `impl Element` types, not trait variants on `Material`.

**2. PSD projection is an assembly-layer concern.** Per-element elastic-tangent Hessian blocks $K^e = B^T \cdot \text{tangent}(F) \cdot B$ can go indefinite under large compression, and the [Part 5 Ch 00 Claim 2](../../50-time-integration/00-backward-euler.md) commitment to an SPD global Hessian requires clamping the negative eigenvalues of each $K^e$ before global assembly. That projection happens here, not in [`material/`](00-material.md) (which returns the true fourth-order tangent, unfiltered) and not in [`solver/`](04-solver.md) (which consumes an SPD sparse matrix). The assembly layer is the right home because PSD projection needs the strain-displacement pull-back, not the 9×9 constitutive tangent.

**3. FEM assembly owns a hand-written VJP.** Reverse-mode autograd over the per-element assembly loop op-by-op would record tens of thousands of elementary multiplications per tet per Newton iteration — tape sizes that blow out memory before a 30k-tet scene finishes a single step. [Part 6 Ch 01 §01](../../60-differentiability/01-custom-vjps/01-fem-assembly.md) names the hand-written VJP as a fused kernel that the tape registers as one opaque node per tet-assembly call. `element/` implements both the forward assembly and its VJP, registered against [`autograd/`](07-autograd.md)'s VJP API. The same pattern governs [`contact/`](03-contact.md)'s IPC barrier VJP; both are fused because op-by-op autograd is the wrong tool for the shape of the compute.

## What the module does not carry

- **No element type tagging across the mesh.** Phase H ships Tet10-in-contact-band as a user-selected region via an SDF mask per [Part 3 Ch 00 §04](../../30-discretization/00-element-choice/04-tradeoff.md); the mesh holds one element type per region, and fully-adaptive mixed-element refinement is deferred past Phase I. `element/` assumes homogeneous regions and conforming interfaces at region boundaries.
- **No global-index knowledge.** Per-element indices are local (0..N for an $N$-node element); mapping local to global DOF indices is [`solver/`](04-solver.md)'s job via the mesh connectivity.
- **No sparse-matrix assembly.** The module returns dense 12×12 (Tet4) or 30×30 (Tet10) blocks; CSR pattern construction and scatter-add into the global Hessian are [`solver/`](04-solver.md)'s concern.

## What this commits downstream

- **[`material/`](00-material.md)'s hot path is called from here.** Stress and tangent are evaluated once per tet (Tet4) or once per Gauss point (Tet10, 4 GP); nothing outside `element/` calls `Material::first_piola` or `Material::tangent` on the Newton hot path.
- **[Part 6 Ch 01](../../60-differentiability/01-custom-vjps.md)'s registration API** is what `element/` binds its hand-written VJP against. The VJP registration mechanism itself lives in [`autograd/`](07-autograd.md); `element/` supplies the forward and backward closed-form expressions.
- **[Part 3 Ch 00 §01 Tet10](../../30-discretization/00-element-choice/01-tet10.md)'s Phase H landing** is additive at this module: a new `impl Element` alongside Tet4, without changing the trait surface or touching callers. The [Part 11 Ch 04 regression suite](../04-testing/01-regression.md) adds a Tet10 baseline against MuJoCo flex at the same phase.
