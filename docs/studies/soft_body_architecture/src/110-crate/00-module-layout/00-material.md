# material/

The `material/` module ships every constitutive law in [Part 2](../../20-materials/00-trait-hierarchy.md) — linear, corotational, neo-Hookean, Mooney-Rivlin, Ogden, HGO anisotropy, Prony viscoelasticity, and thermal couplings — as implementations of the four-item [`Material` trait from Part 2 Ch 00 §00](../../20-materials/00-trait-hierarchy/00-trait-surface.md). The parent [Ch 00 claim 2](../00-module-layout.md) says the module layout is "informed by the thesis, not by convention"; `material/` is the first module where that choice shows up — most FEM libraries collapse `material/`, [`element/`](01-element.md), and [`mesh/`](02-mesh.md) into one "fem" module, and `sim-soft` pays the separation cost here to buy composability in [`sdf_bridge/`](08-sdf-bridge.md) and the spatial-field machinery.

## What the module owns

| Sub-area | Responsibility | Where it is specified |
|---|---|---|
| Strain measures | $F$, $C$, $E$, invariants $I_1/I_2/I_3$, derived scalars (volumetric strain, von Mises) | [Part 2 Ch 01](../../20-materials/01-strain-measures.md) |
| Baseline laws | Linear elasticity, corotational | [Part 2 Ch 02–03](../../20-materials/02-linear.md) |
| Hyperelastic family | Neo-Hookean, Mooney-Rivlin, Ogden — the Phase B through Phase H material progression | [Part 2 Ch 04](../../20-materials/04-hyperelastic.md) |
| Near-incompressibility | Mixed u-p, F-bar, higher-order alternatives | [Part 2 Ch 05](../../20-materials/05-incompressibility.md) |
| Anisotropy | Fiber-reinforced, HGO, direction-field composition | [Part 2 Ch 06](../../20-materials/06-anisotropic.md) |
| Viscoelasticity | Prony series, Oldroyd-B, DMA protocol | [Part 2 Ch 07](../../20-materials/07-viscoelastic.md) |
| Thermal coupling | Temperature-dependent modulus, thermal expansion, dissipative heating | [Part 2 Ch 08](../../20-materials/08-thermal-coupling.md) |
| Spatial material fields | `Field<Output = f64>` / `Field<Output = Vec3>`, `DiffusionProfile` | [Part 2 Ch 09](../../20-materials/09-spatial-fields.md) |

Every item above is an `impl Material` or a composition decorator over one. No runtime feature-detection, no per-constitutive-law enum branches in the solver.

## Three claims

**1. The module has no dependencies inside `sim-soft`.** `material/` is pure math: strain measures, energy densities, their first and second derivatives, and the `Field` trait for spatial variation. It does not know what an element is, how a mesh is stored, whether the solver is on CPU or GPU, or whether the autograd tape is recording. This isolation is what makes the module's [gradcheck sub-chapter](../04-testing/03-gradcheck.md) a 20-line test harness that can run on [`material/`](00-material.md) alone, before [`element/`](01-element.md) and [`solver/`](04-solver.md) compile — exactly the [Phase A deliverable](../03-build-order.md#the-committed-order) from the build-order chapter.

**2. Rate-dependent, temperature-dependent, and anisotropic behavior are compositions, not trait variants.** The [Part 2 Ch 00 §01 composition rules](../../20-materials/00-trait-hierarchy/01-composition.md) commit to `Viscoelastic<M: Material>`, `Thermal<M: Material>`, and `Material` + `FiberDirection` associated type as the three decorator patterns. `material/` implements these as generic wrappers over a base material, so Prony-viscoelastic neo-Hookean with thermal softening is `Thermal<Viscoelastic<NeoHookean>>` — three structs composed, one `impl Material` at the outermost layer, generic in the inner type on every hot-path call site. The downstream hot path ([`element/`](01-element.md)'s per-Gauss-point stress evaluation) monomorphizes through the stack; no `dyn Material` dispatch inside the Newton loop.

**3. `MaterialField` carries both solver-consumed fields and renderer-only fields.** The [Part 7 Ch 00 §4](../../70-sdf-pipeline/00-sdf-primitive.md) `MaterialField` struct holds stiffness, density, fiber direction, Prony terms, and thermal conductivity (all solver-consumed), plus `layer_id` and `diffusion_profile: Option<Box<dyn Field<Output = DiffusionProfile>>>` (both renderer-only, no solver consumer). `DiffusionProfile` is a Phase-γ-locked type, sum-of-exponentials by default (Christensen–Burley 2015 lineage per [Part 9 Ch 00 §01](../../90-visual/00-sss/01-bssrdf.md)), with scene-ingest refit to sum-of-Gaussians when `layer_id` is present so multi-layer compounds via variance addition. `material/` owns the storage and composition; [`sim-bevy`](../02-coupling/02-bevy.md) owns the shader consumption. Single-material runs leave both fields `None` and pay no cost.

## What the module does not carry

- **No element-shape knowledge.** The `Material` trait sees a 3×3 deformation gradient, not a tet or a hex. Per-element shape-function pull-back lives in [`element/`](01-element.md).
- **No solver state.** No per-tet scratch, no assembly buffers, no factorization handles. Those are [`solver/`](04-solver.md)'s concern.
- **No autograd tape recording.** `material/` evaluates closed-form energy / stress / tangent; tape machinery is [`autograd/`](07-autograd.md)'s. The module is called by the tape, not the other way around.

## What this commits downstream

- **[Part 2](../../20-materials/00-trait-hierarchy.md) ships as one module.** Every constitutive law added post-Phase-H (e.g., new viscoelastic forms, electroactive-polymer models) lands here as an additional `impl Material` without changing the trait surface.
- **[Part 9 Ch 00 SSS](../../90-visual/00-sss.md) consumes `DiffusionProfile` from this module, not from a sibling renderer crate.** The storage-and-composition layer is `material/` because the spatial-field machinery already lives here; the visual-vs-physical merge from the [Part 1 Ch 03 thesis](../../10-physical/03-thesis.md) depends on this not being bolted on.
- **[Appendix 02's material-property database](../../appendices/02-material-db.md)** cashes out per-material numerical constants for every `impl Material` in this module — one row per real silicone compound with its hyperelastic fit, Prony spectrum, thermal parameters, and (post-γ) diffusion-profile measurements.
