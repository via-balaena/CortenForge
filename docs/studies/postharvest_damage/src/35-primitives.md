# What the SDK has, and what it does not

> **The one-line version.** It is tempting to read this program as a consumer of existing primitives
> — we have a validated soft-body FEM, contact, and a co-design optimizer, so the gates look like
> assembly work.
>
> **They are not.** `sim-soft` is a *hyperelastic* solver, and every damage model in
> [Ch 3](30-gap.md) is elastic-plastic. Bridging that is a trait change, not an `impl`, and it is
> the single largest unpriced cost in the program.

## Inventory

Verified against the tree at `1bc69a85`, not from memory.

| Need | Primitive | Status |
|---|---|---|
| Soft-body FEM, backward-Euler | `sim/L0/soft` — BE-FEM, implicit, differentiable | ✅ exists |
| Hyperelastic constitutive models | `material/{neo_hookean,yeoh}.rs` | ✅ exists |
| **Elastic-plastic constitutive path** | — | ❌ **absent, and blocked by trait shape** |
| **Damage / failure criterion** | — | ❌ absent |
| Material-parameter gradients (∂P/∂p) | `Material::first_piola_param_grad` (keystone S5) | ✅ exists |
| Soft contact | `contact/{ipc,friction,penalty,rigid}.rs` | ✅ exists |
| Exact geometry → physics | `cf-mesh`, `sdf_bridge/` | ✅ exists |
| Soft ↔ rigid coupling, one tape | `sim/L1/coupling` | ✅ keystone complete |
| Inverse design over soft material params | `cf-codesign::SoftMaterialTarget`, `SoftMaterialTrajectoryTarget` | ✅ exists |
| Inverse design over geometry / routes | `cf-codesign::RouteTarget` (#661) | ✅ exists |
| **Stochastic / ensemble solve** | — | ❌ absent |

Four of those rows are the program. Two of them are cheap. One is not.

## ★ The blocking gap: sim-soft has no plasticity path

`sim/L0/soft/src/material/mod.rs` defines the constitutive contract:

```rust
pub trait Material: Send + Sync {
    fn energy(&self, f: &Matrix3<f64>) -> f64;
    fn first_piola(&self, f: &Matrix3<f64>) -> Matrix3<f64>;
    fn tangent(&self, f: &Matrix3<f64>) -> SMatrix<f64, 9, 9>;
    fn validity(&self) -> ValidityDomain;
}
```

Read the signature of `first_piola`. **Stress is a pure function of the deformation gradient `F`.**
There is no state argument, no history, no internal variable. That is a *path-independent*
contract — correct and complete for hyperelasticity, where stress derives from a strain-energy
density and the doc comment's validity example is literally `J > 0`.

Elastic-plasticity is **path-dependent**. Stress at a material point depends on accumulated plastic
strain, which requires:

- an internal state variable per quadrature point (plastic strain tensor, hardening variable),
- a **return-mapping** algorithm at each Newton iteration to project trial stress back onto the
  yield surface,
- a **consistent tangent** that differs from the elastic tangent once yielding,
- and state that persists and evolves across timesteps.

None of that fits behind `first_piola(&self, f)`. **Implementing plasticity means changing the
`Material` trait, not adding an `impl`** — and every existing material, the element assembly, and
the solver's tangent path are downstream of that signature.

### ✅ The adjoint question, answered

An earlier assessment left this open and warned that no gate should be costed as though the adjoint
survives plasticity. **It was investigated against the code, and the answer is favourable.** The
adjoint extends, by an established pattern, with one non-trivial term to derive.

**The through-time adjoint already exists.** Not via the `time_adjoint` trait method — that is a
deliberate stub. It is realized by composing per-step VJPs on the tape through `push_custom`, so one
`tape.backward` crosses step boundaries. `StateStepVjp` threads `(x_prev, v_prev)` as tape parents;
`TrajectoryStepVjp` already handles four or five parents depending on drift. **Plastic internal
state becomes another threaded parent — an existing pattern, not a new mechanism.**

**The real blocker is one sentence in `StateStepVjp`'s derivation:**

> the free-to-free block depends on `x_prev` only through the inertia diagonal (`f_int` depends on
> the unknown `x`, not on `x_prev`), so the `(M/Δt²)·λ` form is exact for it

**Plasticity breaks exactly that.** With plastic history, `f_int` *does* depend on previous state, so
`∂r/∂x_prev` stops being the clean diagonal `−(M/Δt²)·I` and acquires a `∂f_int/∂(plastic state)`
term. That is a real derivation and implementation — but it is one identified term in one primitive,
not an architectural rewrite.

**Symmetry is probably a non-issue and was anticipated.** The source already notes that "a future
asymmetric material would need `solve_transpose_in_place` here." Named, expected, small. And for
**associated J2 von Mises plasticity** — the model every validated paper in [Ch 3](30-gap.md) uses —
the consistent tangent is symmetric, so the happy-path Cholesky likely survives untouched.

**There is a template.** `MaterialStepVjp` already computes `∂r/∂p_k = ∂f_int/∂p_k` from the stress
derivative, contracting against the same factored tangent. A plastic-state VJP is structurally
identical: a computed RHS vector, same factor reuse.

> **Net effect on the program.** The scariest unknown turned out to be the most tractable.
> [Gate 4](40-program.md#gate-4--inverse-design) is no longer blocked on an open question. **Gate 0's
> cost is unchanged** — the `Material` trait change and the return mapping are still the real
> expense.

## Consequences for cost

| Gate | Earlier claim | Corrected |
|---|---|---|
| [Gate 0](40-program.md#gate-0--can-we-reproduce-a-published-number) | "pure software, existing primitives" | Needs a **new constitutive class + trait change**. Weeks-to-months, not days. |
| [Gate 1](40-program.md#gate-1--the-ablation-nobody-ran) | "a few weeks of work" | Cheap *given Gate 0*; it is a multi-material mesh run. Not independently cheap. |
| [Gate 2](40-program.md#gate-2--distributional-prediction) | "pure software" | Needs ensemble/Monte-Carlo driving, which is absent — but it is a harness, not solver surgery. |
| [Gate 4](40-program.md#gate-4--inverse-design) | "the flagship rung" | **Unblocked.** Adjoint extends by an existing pattern; one `∂f_int/∂(plastic state)` term to derive. |

## The honest alternative nobody costed

There is a cheaper path this study did not evaluate, and it should be on the table before any
plasticity work starts:

> **Use an existing FE package for Gates 0–2 and only bring the physics in-house at Gate 4.**

Abaqus, ANSYS and LS-DYNA already have bilinear isotropic hardening and would reproduce the
published results directly — that is what every validated paper in [Ch 3](30-gap.md) used. The
CortenForge-specific contribution is **inverse design and held-out validation discipline**, neither
of which requires that *we* own the forward solver until the optimization loop needs gradients.

Against that: a bought forward solver cannot be differentiated, so Gate 4 would eventually force the
in-house path anyway; and [Gate 5](40-program.md#gate-5--open-the-tooling-deliverable-not-a-gate)'s open-tooling
argument collapses entirely if the pipeline depends on a commercial licence.

**Which of those dominates depends on whether FE licence cost is actually a barrier for the
intended users — which three sweeps failed to establish.** It is
[open question 8](appendices.md#open-questions), and it now gates a build-vs-buy decision rather
than just a philanthropic talking point.

## Where this would live

If the program proceeds past Gate 2, the natural homes are:

- **Plasticity + damage criterion** → inside `sim/L0/soft` as a widened `Material` contract, not a
  new crate. A parallel `PlasticMaterial` trait is the obvious lower-risk alternative and should be
  weighed against widening the existing one.
- **Distributional harness** → `sim/L0/opt` or a thin tool crate; it is orchestration, not physics.
- **Produce-damage targets** → `tools/cf-codesign`, alongside `SoftMaterialTarget` and
  `RouteTarget`, following the pattern those established.

**No new L0 crate is obviously warranted.** That is a genuine point in the program's favour, and the
one sense in which "consumer of existing primitives" holds: the *architecture* absorbs this work
cleanly. It is the constitutive contract, not the crate layout, that has to change.
