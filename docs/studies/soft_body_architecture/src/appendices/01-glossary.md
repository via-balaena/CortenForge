# Glossary

Book-introduced terms, in definition-list form. Each entry is one to two lines — enough to fix the meaning, not enough to substitute for the chapter that introduces it. The "Introduced in" pointer on each entry names the chapter where the term first appears and is most fully treated. Terms from the wider continuum-mechanics, FEM, and ML-autograd literatures are not redefined here; the [reading guide](../00-context/04-reading-guide.md) names external resources for those.

## Architectural terms

**Compliant cavity–probe conformity**
: The book's canonical design problem — a soft cavity deformed to conform to a rigid probe under prescribed squeeze, with reward measuring pressure uniformity, contact coverage, peak-pressure bounds, and effective-stiffness bounds. A generic soft-robotics abstraction (seals, compliant grippers, catheter sheaths, wearables) that exercises every `sim-soft` subsystem. Introduced in [Part 1 Ch 00](../10-physical/00-canonical.md).

**Design mode**
: One of `sim-soft`'s two operating modes. Higher resolution (~30k tets), larger timestep, optimization-oriented. Paired with *experience mode*. Introduced in [Part 1 Ch 03 (thesis)](../10-physical/03-thesis.md).

**Experience mode**
: One of `sim-soft`'s two operating modes. Lower resolution (~5k tets), 60+ FPS, preview-oriented. Paired with *design mode*. Introduced in [Part 1 Ch 03](../10-physical/03-thesis.md).

**The book is the spec**
: Design-theme decision locked in [Part 11 Ch 03 (build order)](../110-crate/03-build-order.md). No separate RFC or Phase 0 precedes Phase A of the crate build; the Pass-3-complete mdbook is the specification Phase A compiles against.

## Physics and solver terms

**IPC barrier**
: The $C^2$-smooth non-penetration energy term $b(d) = -(d - \hat d)^2 \ln(d/\hat d)$ from Li et al. 2020 that replaces penalty and impulse contact in `sim-soft`. Diverges as gap $d \to 0^+$, vanishes outside tolerance $\hat d$. Introduced in [Part 4 Ch 01](../40-contact/01-ipc-internals.md).

**Factor-on-tape pattern**
: The autograd-tape convention by which the converged-step sparse Cholesky factor (CPU path) or the preconditioner plus warm-start vector (GPU path) is stored on the reverse-mode tape for the IFT adjoint's backward solve to re-use. Recovers a 10–30× speedup on both backends ("one setup, many backward solves"). Introduced in [Part 5 Ch 00](../50-time-integration/00-backward-euler.md) and [Part 6 Ch 02](../60-differentiability/02-implicit-function.md); extended to GPU in [Part 8 Ch 02](../80-gpu/02-sparse-solvers.md).

**Mesh IS render mesh**
: The commitment that the FEM tet mesh's surface (optionally subdivided for resolution) is the exact mesh the renderer rasterizes; no skinning layer, no secondary-motion module, no normal-map bakery. Per-vertex physics attributes (stress, temperature, thickness, contact pressure) are shader inputs. Introduced in [Part 1 Ch 03 (thesis)](../10-physical/03-thesis.md) and cashed out in [Part 9 Ch 05](../90-visual/05-sim-bevy.md).

## Differentiability terms

**Factor-on-tape**
: See above, under *physics and solver terms*.

**FD wrapper**
: Finite-difference wrapper — the gradient fallback used at non-smooth boundaries (topology-changing re-mesh, adaptive refinement events, adaptive-$\Delta t$ shrinks). Cost $2 n_\theta$ full forward solves per gradient evaluation; emits `GradientEstimate::Noisy { variance }` so the optimizer can weight the sample accordingly. Introduced in [Part 6 Ch 05](../60-differentiability/05-diff-meshing.md).

**Topology-fixed-per-episode**
: The three-part compromise `sim-soft` commits to for the [Part 6 Ch 05](../60-differentiability/05-diff-meshing.md) differentiable-meshing open problem. Inside one optimization episode the mesh topology is held fixed and all gradients flow cleanly; between episodes the mesh is re-derived with warm-started state transfer; gradient flow w.r.t. SDF parameters that cross topology boundaries is FD-wrapped.

## Types exposed at crate boundaries

**`SdfField`**
: The [`cf-design` → `sim-soft`](../70-sdf-pipeline/00-sdf-primitive.md) boundary for geometry. A trait object representing a signed-distance field over reference space; `sim-soft` does not inspect `cf-design`'s composition tree, only queries the trait. Introduced in [Part 7 Ch 00](../70-sdf-pipeline/00-sdf-primitive.md).

**`MaterialField`**
: The [`cf-design` → `sim-soft`](../70-sdf-pipeline/00-sdf-primitive.md) boundary for material properties. SDF-valued over reference space; each scalar material parameter (stiffness, density, fiber direction, Prony terms, diffusion profile) is sampled per tet. Introduced in [Part 7 Ch 00](../70-sdf-pipeline/00-sdf-primitive.md); extended with `DiffusionProfile` in [Part 9 Ch 00](../90-visual/00-sss.md).

**`DiffusionProfile`**
: Sub-field of `MaterialField` that stores a per-color-channel scattering-coefficient-vs-distance curve for the SSS shader. Measured via integrating-sphere spectrophotometry on a flat coupon. Introduced in [Part 9 Ch 00](../90-visual/00-sss.md).

**`SoftScene`**
: The top-level `sim-soft` scene description — an `SdfField` + `MaterialField` + boundary conditions + contact partners. Four fields cover the whole forward-map input. Introduced in [Part 1 Ch 03 (thesis)](../10-physical/03-thesis.md).

**`ForwardMap`**
: The trait `sim-soft` exposes to the optimization layer: `evaluate(theta, tape) → (reward, EditResult)` and `gradient(theta, tape) → (grad, GradientEstimate)`. The Part 10 contract. Introduced in [Part 10 Ch 00](../100-optimization/00-forward.md).

**`EditResult`**
: Return type from a design edit classifying the wall-time cost regime (warm-start, re-factor, full re-mesh) and carrying measured elapsed time. Introduced in [Part 7 Ch 04](../70-sdf-pipeline/04-live-remesh.md).

**`EditClass` — `ParameterOnly` / `MaterialChanging` / `TopologyChanging`**
: Three-valued enum the change detector produces classifying a design edit by its cost regime. `ParameterOnly` ≤50 ms, `MaterialChanging` ≤200 ms, `TopologyChanging` ≤500 ms on canonical design-mode resolution. Introduced in [Part 7 Ch 04](../70-sdf-pipeline/04-live-remesh.md).

**`GradientEstimate::Noisy { variance }`**
: Enum variant flagging gradient samples produced by FD wrappers or other non-smooth fallbacks. The `variance` field lets the outer optimizer weight the sample appropriately. Introduced in [Part 6 Ch 05](../60-differentiability/05-diff-meshing.md).

**`MeshSnapshot`**
: The per-physics-step record `sim-soft` produces for `sim-bevy` to render. Carries deformed vertices, normals, and per-vertex physics attributes (stress, temperature, thickness, contact pressure, layer ID, anisotropy direction). Double-buffered for writer-reader decoupling. Introduced in [Part 9 Ch 05](../90-visual/05-sim-bevy.md).

**`MeasurementReport`**
: The JSON schema for a physical-print measurement record (force curve, contact-pressure map, diffusion profile, effective stiffness at probe). Consumed by the residual-GP machinery in `sim-opt`. Scope: post-Phase-I per [Part 10 Ch 05](../100-optimization/05-sim-to-real.md) and [Part 12 Ch 07](../120-roadmap/07-open-questions.md).

**`SimSoftPlugin`**
: The single Bevy plugin a user adds to their `App` for the entire `sim-soft` ↔ Bevy integration. Registers the snapshot double-buffer, the background physics task, the GPU upload system, and the five-pass shader pipeline. Introduced in [Part 9 Ch 05](../90-visual/05-sim-bevy.md).

**`SimToRealCorrection`**
: Per-component residual-GP container (force, pressure-field, diffusion, stiffness) that corrects sim-side rewards against measured-print observations. Introduced in [Part 10 Ch 05](../100-optimization/05-sim-to-real.md).
