# Unit tests

Unit tests are the narrowest, fastest, and most-local of the four test classes from [parent Ch 04](../04-testing.md). They exercise one trait impl on one small input and check a single numerical or structural property — the stress value at a known deformation gradient, the shape-function partition of unity at a Gauss point, the IPC barrier's value on an analytic gap, a mesh's element-quality histogram on a synthesized cube. They run in hundreds of milliseconds end-to-end under `cargo test`, they block every PR, and they are the first tests authored on every new module — [Phase A's commitment](../03-build-order.md#the-committed-order) that `material/` and `mesh/` ship with gradcheck + unit coverage before `solver/` compiles against them makes unit tests the crate's first line of defense.

## Scope — per-trait, in isolation

Each unit test targets one trait from the [Ch 01 seven-trait inventory](../01-traits/00-core.md) on a concrete impl, in isolation from other modules. The table below names the test classes against the minimal surface each trait commits:

| Trait | Test class | Representative assertion |
|---|---|---|
| `Material` ([`material/`](../00-module-layout/00-material.md)) | Constitutive law correctness at canonical $F$ | `energy(Matrix3::identity()) ≈ 0`; `first_piola` at a uniaxial stretch equals the closed-form neo-Hookean value; `tangent` is symmetric; `validity(F)` returns `OutOfDomain` on $\det F < 0$ |
| `Element<const N, const G>` ([`element/`](../00-module-layout/01-element.md)) | Shape-function partition of unity; gradient correctness | $\sum_{i=1}^{N} N_i(\xi) = 1$ at each Gauss point; `shape_gradients` columns sum to zero; per-tet assembled $B$ matrix agrees with its symbolic form on a reference tet |
| `Mesh` ([`mesh/`](../00-module-layout/02-mesh.md)) | Adjacency + quality + structural equality | Adjacency is symmetric; dihedral-angle histogram on a synthesized canonical cube matches the analytic bin counts; `equals_structurally` flips exactly on a single vertex-index permutation |
| `ContactModel` ([`contact/`](../00-module-layout/03-contact.md)) | IPC barrier values on analytic gaps | `energy(pair, positions)` at $d = \hat{d}/2$ matches the closed-form barrier; `gradient` at $d \to \hat{d}$ vanishes to machine epsilon; `ccd_toi` on a one-pair analytic collision returns the known time-of-impact to 10 digits |
| `Solver` ([`solver/`](../00-module-layout/04-solver.md)) | One-step convergence on analytic cases | `CpuNewtonSolver::step` on a single-tet cube with uniform traction converges in $\leq 3$ iterations; `convergence_tol` is honored; `NewtonStep` carries the `Llt<f64>` factor the [factor-on-tape claim](../00-module-layout/04-solver.md) commits to |
| `Differentiable` ([`autograd/`](../00-module-layout/07-autograd.md)) | Registration-surface sanity | `register_vjp` returns a key that `replay_vjp` accepts; `fd_wrapper` on an analytic scalar function agrees with hand-computed central differences to 5 digits — the gradient *magnitude* agreement is in [`03-gradcheck.md`](03-gradcheck.md), not here |
| `Observable` ([`readout/`](../00-module-layout/09-readout.md)) | Readout closed forms | `stress_field` on a uniaxial-stretch single-tet state equals the analytic first Piola; `reward_breakdown` on a hand-scored synthetic state matches the per-term values computed by hand |

Every entry is a test that runs in $< 50$ ms, on one thread, with no GPU dependency, and with no cross-module state. Cross-module integration is a [regression](01-regression.md), [visual-regression](02-visual.md), or [gradcheck](03-gradcheck.md) concern; at the unit layer, each trait impl is checked against its own spec.

## Per-trait determinism contracts

Every trait on the [Ch 01 inventory](../01-traits/00-core.md) commits implicitly to **same-input → bit-identical-output** — no hidden state, no RNG, no timestamp leakage, no environment-dependent floating-point modes. Unit tests make that commitment explicit by asserting it directly: repeated calls to `Material::energy(f)` at the same `f` return identical `f64` bit-patterns; `Element::shape_functions(xi)` is pure in `xi`; `ContactModel::energy(pair, positions)` depends only on its inputs; `Observable::stress_field(step)` is a closed-form read off the step.

The operational test is compact:

```rust
#[test]
fn material_energy_is_deterministic() {
    let m = NeoHookean::new(1.0e6, 0.3);
    let f = random_spd_f(seed = 42);
    assert_eq!(m.energy(&f).to_bits(), m.energy(&f).to_bits());
}
```

Per-trait determinism at the unit layer is the foundation the [Ch 00 readout claim 2 end-to-end invariant](../00-module-layout/09-readout.md) and the [Part 10 Ch 00 `ForwardMap` determinism-in-θ contract](../../100-optimization/00-forward.md) compose out of. `ForwardMap::evaluate(theta, tape)` can only be deterministic-in-θ modulo cache if every trait call on the forward path is deterministic; the full-composition test lives in the [gradcheck sub-leaf](03-gradcheck.md) §01 because it binds the γ-locked API, and the per-module building blocks are unit-tested here. One layer catches a trait impl that silently depends on a global counter; the other layer catches a composition that leaks stochasticity across trait boundaries. Both are needed.

## What each module's test suite looks like

Per [Phase A](../03-build-order.md#the-committed-order), `material/` and `mesh/` each ship with their own `tests/` directory before any consumer module compiles against them. The shape is:

- `tests/neo_hookean_values.rs` — energy, first Piola, tangent values at five canonical $F$ (identity, uniaxial stretch, simple shear, near-singular, inverted)
- `tests/neo_hookean_frame_indifference.rs` — rotating $F$ by $R$ rotates the stress by $R$ too (on a stack of random rotations)
- `tests/mesh_adjacency.rs` — on a 20-tet synthesized scene, per-tet adjacency is consistent with the face-incidence graph
- `tests/mesh_quality.rs` — the dihedral-angle histogram and the aspect-ratio histogram match their analytic bin counts on a canonical cube mesh

Each file is ~20–80 lines. No fixture files, no golden-data JSONs — the scenes are synthesized in code so failures point at one assertion rather than at an opaque binary.

As Phase B onward lands `element/`, `contact/`, `solver/`, `autograd/`, and `readout/`, each module adds its own `tests/` directory with the same structure. `solver/` unit tests check that `CpuNewtonSolver::step` reaches the analytic equilibrium on a single-tet cube under uniaxial load; `contact/` tests check barrier values and active-pair generation on two-cube analytic scenes; `readout/` tests check closed-form stress and reward-term readout against hand-computed values. The [gradcheck suite](03-gradcheck.md) is adjacent but orthogonal — it tests *derivatives* to 5 digits, this suite tests *values* to machine epsilon.

## Cadence

Unit tests run on every `cargo test` invocation and block every PR in CI. Per the [parent Ch 04 A-grade bar](../04-testing.md), a failing unit test blocks merge — the failing assertion is either a regression or a spec clarification, never a "flake" to be re-run. Flaky unit tests in CI are themselves a spec bug: either the assertion is too tight (tighten or loosen the spec), or the impl is non-deterministic against its trait contract (fix the impl), or the test harness has a bug (fix the harness).

The full workspace unit-test suite should run in under 30 seconds on a developer machine. If a per-module test file grows past that — a pathological case where a material test enumerates a high-dimensional parameter sweep, for instance — the slow cases get split into a release-mode `#[ignore]` pathway, not bundled into the default `cargo test` run.

## What unit tests do not cover

- **Cross-module composition.** A unit test that imports three modules and tests their interaction is already a small integration test and belongs with [regression](01-regression.md) or [gradcheck](03-gradcheck.md). The *whole* forward pipeline composition is tested at `ForwardMap::evaluate`-level in [gradcheck §01](03-gradcheck.md), not here.
- **Derivatives.** Value-level correctness is unit-tested; gradient-level correctness is [gradcheck](03-gradcheck.md)-tested. A unit test asserts `first_piola` at one $F$; gradcheck asserts $\partial P / \partial F$ matches FD to 5 digits over a distribution of $F$.
- **GPU parity.** CPU-vs-GPU 5-digit agreement is a [gradcheck §03 Phase E deliverable](03-gradcheck.md), not a unit-test concern. Unit tests run on CPU only; GPU unit coverage would duplicate what the CPU-vs-GPU regression already catches.
- **MuJoCo-flex baselines.** Cross-solver drift is [regression §01](01-regression.md)'s job. A unit test that "flex also gets this value" would couple two solvers at the unit layer, which defeats the isolation property this layer is built on.
- **Rendered output.** Per-vertex attribute streaming, shader output, visual composition — all [visual-regression §02](02-visual.md)'s job. The per-vertex attribute *values* are unit-tested at the `Observable` trait level; what the shader does with them is a [Phase I](../03-build-order.md#the-committed-order) concern.
- **Physical-print-loop regression.** [Part 10 Ch 05 `MeasurementReport` ingestion](../../100-optimization/05-sim-to-real.md) is [post-Phase-I per Part 12 Ch 07](../../120-roadmap/07-open-questions.md); no unit test in this suite binds against real-world measurement data. When the post-Phase-I test harness lands, it extends this suite rather than replacing it.

## What this sub-leaf commits downstream

- **[Phase A](../03-build-order.md#the-committed-order)'s start condition** is the `material/` and `mesh/` unit-test suites passing on every commit — no `element/` or `solver/` work lands before those suites are green.
- **[Ch 01 trait-surface changes are not local edits](../01-traits/00-core.md)** — a method addition to `Material` or `Element` ripples through this suite's per-trait test classes, and a cross-module audit cycle is triggered per the parent claim.
- **The [parent Ch 04 A-grade bar](../04-testing.md)** lands here as "every trait impl carries unit tests that check its spec directly." Per-module unit coverage is the first thing the workspace-level coverage audit reads; no module ships without it.
