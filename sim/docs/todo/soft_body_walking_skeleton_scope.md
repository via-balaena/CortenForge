# `sim-soft` Walking-Skeleton Scope

**Status:** Scope memo — authored before any Rust.
**Date:** 2026-04-21, post PR #210 (`be022f9e`).
**Follows:** [`project_soft_body_walking_skeleton_pivot.md`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/project_soft_body_walking_skeleton_pivot.md) — user directive to close the spec-to-code ratio after Pass 3 δ.
**Target:** ~500–1500 LOC Rust, single long-running branch `feature/soft-body-walking-skeleton`.

The book (`docs/studies/soft_body_architecture/`) is the spec. This memo does not re-argue architecture; it picks the minimum slice of that architecture required to operationally validate six load-bearing invariants, and it names the binary pass/fail criteria for a "green" skeleton.

## 1. The six load-bearing invariants

Each invariant is a claim the book makes that must be verified in code before further book expansion. Citations are to the book as it stands at `be022f9e`.

| # | Invariant | Book anchor | What green looks like |
|---|---|---|---|
| I-1 | Seven-trait core surface composes without ceremony explosion | [`110-crate/01-traits/00-core.md`](../../../docs/studies/soft_body_architecture/src/110-crate/01-traits/00-core.md) | All 7 traits (`Material`, `Element`, `Mesh`, `ContactModel`, `Solver`, `Differentiable`, `Observable`) defined with the pinned signatures; one concrete impl of each; the forward path compiles without phantom-type gymnastics, trait-object juggling, or `Box<dyn …>` ceremony beyond the one `dyn Solver` on the public boundary. |
| I-2 | `ForwardMap::evaluate → (RewardBreakdown, EditResult)` works in practice | [`100-optimization/00-forward.md`](../../../docs/studies/soft_body_architecture/src/100-optimization/00-forward.md) (γ-locked) | A single `SoftScene::one_tet_cube()` constructs a `ForwardMap`; `fm.evaluate(&theta, &mut tape)` returns a populated `RewardBreakdown` and an `EditResult` tagged `ParameterOnly`; repeated calls at the same θ return bit-identical `RewardBreakdown` (§01 determinism-in-θ). |
| I-3 | `faer::sparse::linalg::solvers::Llt<f64>` exposes factor-on-tape for IFT adjoint | [`50-time-integration/00-backward-euler.md`](../../../docs/studies/soft_body_architecture/src/50-time-integration/00-backward-euler.md), [`110-crate/03-build-order.md`](../../../docs/studies/soft_body_architecture/src/110-crate/03-build-order.md) Phase B | Forward Newton solve factors once; the `Llt<f64>` factor is stored on the tape as a first-class object; `fm.gradient(&theta, &tape)` retrieves the factor, applies `solve_in_place` to $-\partial r / \partial \theta$, returns a scalar gradient; no re-factorization. |
| I-4 | `sim-ml-chassis` autograd tape supports per-operation VJPs as Part 6 Ch 01 assumes | [`60-differentiability/01-custom-vjps/00-registration.md`](../../../docs/studies/soft_body_architecture/src/60-differentiability/01-custom-vjps/00-registration.md), [`60-differentiability/00-what-autograd-needs.md`](../../../docs/studies/soft_body_architecture/src/60-differentiability/00-what-autograd-needs.md) Claim 3 | `Differentiable::register_vjp` accepts a closure keyed on a `TapeNodeKey` from `sim-ml-chassis`'s existing CPU tape; forward-pass ops from `material/` / `element/` / `solver/` register VJPs once at construction; the IFT adjoint replays them against the cached factor. If `sim-ml-chassis`'s current tape cannot surface a `TapeNodeKey`-equivalent, the gap is flagged as a `sim-ml-chassis` amendment before skeleton code continues. |
| I-5 | Determinism-in-θ holds under realistic Newton convergence | [`110-crate/04-testing/03-gradcheck.md`](../../../docs/studies/soft_body_architecture/src/110-crate/04-testing/03-gradcheck.md) §01 | `forward_map_is_deterministic_in_theta` passes as a `cargo test` target on the 1-tet scene. Assertion is bit-equal on every field of `RewardBreakdown`. The Newton loop must converge to the tolerance reported by `Solver::convergence_tol` with no timestamp, counter, or RNG leakage into the fixed-point. |
| I-6 | Rust + wgpu build graph (no FFI) is tractable for soft-body FEM | Cross-cutting; foundational to [`110-crate/03-build-order.md`](../../../docs/studies/soft_body_architecture/src/110-crate/03-build-order.md) Phase-B/E story | `cargo build -p sim-soft` succeeds on macOS-arm64 and a Linux CI runner; all deps are pure Rust or Rust-bindgen (no C/C++ compile step added to the graph); faer integrates via `Cargo.toml` alone; `sim-soft` compiles `--release` in under the workspace's existing compile-time budget. `wgpu` is **not** pulled in yet — its graph tractability is validated when Phase E's GPU port begins; I-6 at skeleton scope validates only the CPU build graph claim. |

I-6 is intentionally narrowed from the pivot-memo version. Including wgpu in the skeleton conflates invariants and inflates LOC. The pivot memo's Phase-E blockers remain deferred; this skeleton validates *only* the CPU half of I-6, with the explicit note that the GPU half reactivates at Phase E start.

## 2. Minimal scene

**One tetrahedron.** Four vertices, one Tet4 element, one Gauss point, neo-Hookean material with hardcoded $(\mu, \lambda)$, Dirichlet boundary on one face (three fixed vertices), traction load on the fourth vertex via $\theta$.

Rationale:
- Smallest scene that exercises the Jacobian assembly, the sparse factorization, and the IFT back-substitution — there is no "smaller" scene on which the book's physics claims are testable.
- One Gauss point eliminates quadrature-weight bugs as a confound for I-5 determinism and I-3 factor-on-tape.
- Four vertices × 3 DOF = 12-DOF system, 9 free after Dirichlet; faer's `Llt<f64>` factors it in microseconds, and finite differences on a 1-D θ can match IFT to 6 digits under double-precision catastrophic-cancellation budget.
- No contact, no second element, no temperature coupling, no mesh topology — every deferral is citable to a specific book phase (below).

## 3. Material / contact / integrator combo

| Axis | Chosen | Alternatives considered | Rationale |
|---|---|---|---|
| Material | Neo-Hookean, hardcoded $(\mu, \lambda)$, Phase A form | Mooney-Rivlin, corotational | Neo-Hookean is the book's Phase A baseline ([`110-crate/03-build-order.md`](../../../docs/studies/soft_body_architecture/src/110-crate/03-build-order.md)); closed-form `first_piola` and `tangent` derivations are already in Part 2; no viscoelastic time-history to confound I-5. |
| Element | Tet4, $N=4$, $G=1$ | Tet10, Hex8 | Tet4 is Phase A; constant-strain element keeps the per-element stiffness 12×12 and analytic. Tet10 is Phase H. |
| Contact | **None** — `NullContact` impl of `ContactModel` | IPC barrier, penalty | IPC is Phase C and is the hardest single module; penalty contact is explicitly rejected in `contact/` Ch 00 §03 claim 1. A `NullContact { active_pairs: [] }` impl proves I-1 without committing to any contact model. The 1-tet scene uses Dirichlet + traction, no contact. |
| Integrator | Backward-Euler Newton, one time step | Forward-Euler, Runge-Kutta, static-equilibrium-only | Backward-Euler is the book's committed integrator ([`50-time-integration/00-backward-euler.md`](../../../docs/studies/soft_body_architecture/src/50-time-integration/00-backward-euler.md)); one step is sufficient to exercise the factor-on-tape and IFT adjoint. Static equilibrium alone would not force the time-stepping machinery to take shape; one step does, with $dt$ large enough that inertia is a real term. |
| Linear solver | faer `Llt<f64>` on sparse CSR | nalgebra dense, Eigen-via-FFI, suitesparse | faer's `Llt<f64>` is the first-class factor-on-tape object the book stakes I-3 on; pure Rust (no FFI) keeps I-6 clean. |
| Autograd tape | `sim-ml-chassis` CPU tape + VJP registration API | PyTorch via `tch-rs`, Burn, `dfdx` | `own every line` thesis ([`60-differentiability/00-what-autograd-needs.md`](../../../docs/studies/soft_body_architecture/src/60-differentiability/00-what-autograd-needs.md) Claim 3). Any gap in the existing chassis tape surfaces as an amendment PR to `sim-ml-chassis`, not a framework import. |
| Gradcheck | §01 determinism + §02 FD-vs-IFT at 5 digits | §03 CPU-vs-GPU | §03 is Phase E; skeleton is CPU-only. §01 + §02 together validate I-2, I-3, I-4, I-5 in one test binary. |

## 4. Crate and module layout

Proposed location: `sim/L0/soft/` — matches the existing `sim/L0/{types, simd, core, thermostat, gpu, mjcf, urdf, tests, ml-chassis, rl, opt}` sibling set.

```
sim/L0/soft/
├── Cargo.toml              # deps: sim-ml-chassis, sim-types, faer, nalgebra
├── src/
│   ├── lib.rs              # re-exports public surface + forward_map module
│   ├── material/
│   │   ├── mod.rs          # Material trait (per 110-crate/01-traits/00-core.md)
│   │   └── neo_hookean.rs  # hardcoded (μ, λ) impl + closed-form VJPs
│   ├── element/
│   │   ├── mod.rs          # Element<N, G> trait
│   │   └── tet4.rs         # impl Element<4, 1>
│   ├── mesh/
│   │   ├── mod.rs          # Mesh trait
│   │   └── tet_mesh.rs     # minimal TetMesh storage
│   ├── contact/
│   │   ├── mod.rs          # ContactModel trait
│   │   └── null.rs         # NullContact — empty active_pairs (validates I-1 without IPC)
│   ├── solver/
│   │   ├── mod.rs          # Solver trait (dyn-safe)
│   │   └── cpu_newton.rs   # CpuNewtonSolver<Tape = CpuTape>, backward-Euler, faer Llt
│   ├── autograd/
│   │   ├── mod.rs          # Differentiable trait + VJP registry
│   │   └── ift.rs          # ift_adjoint: factor.solve_in_place on cached Llt
│   ├── readout/
│   │   ├── mod.rs          # Observable trait
│   │   └── reward.rs       # minimal RewardBreakdown (one or two terms)
│   └── forward_map/
│       ├── mod.rs          # ForwardMap impl composing the above
│       └── scene.rs        # SoftScene::one_tet_cube()
└── tests/
    ├── determinism.rs      # §01 gradcheck — bit-equal RewardBreakdown
    └── gradcheck.rs        # §02 gradcheck — 5-digit FD-vs-IFT
```

Seven traits, seven concrete impls (one per trait, with `NullContact` the one that has no work to do), one scene constructor, one `ForwardMap` impl, two tests. No `coupling/`, `gpu/`, or `sdf_bridge/` modules — those reactivate at Phase F / E / G respectively.

Budget estimate: ~800 LOC without tests, ~1200 with tests. Sits in the middle of the pivot memo's 500–1500 band. Comes in lower if `sim-ml-chassis`'s tape already exposes enough of `TapeNodeKey`; comes in higher if I-4's amendment is needed.

## 5. Green-skeleton checklist

The skeleton is **green** when all of the following are true on `feature/soft-body-walking-skeleton`:

- [ ] `cargo build -p sim-soft` succeeds on macOS-arm64.
- [ ] `cargo build -p sim-soft --release` succeeds and completes within the workspace's compile-time budget (no new multi-minute FFI step).
- [ ] `cargo test -p sim-soft --release` runs two test binaries:
  - [ ] `determinism::forward_map_is_deterministic_in_theta` — bit-equal `RewardBreakdown` across two `fm.evaluate` calls at the same θ (validates I-2, I-5).
  - [ ] `gradcheck::ift_matches_central_difference_to_5_digits` — `assert_rel_err!(grad_ift, fd, rel_tol = 1e-5)` on the 1-tet scene (validates I-3, I-4).
- [ ] `cargo xtask grade sim-soft` returns **A** across all seven criteria. (User standard: "A-grade or it doesn't ship.")
- [ ] All seven traits from [`110-crate/01-traits/00-core.md`](../../../docs/studies/soft_body_architecture/src/110-crate/01-traits/00-core.md) are defined in `sim-soft` with the pinned signatures and at least one concrete impl each (validates I-1).
- [ ] Dependency audit: `cargo tree -p sim-soft` shows no C/C++ build step added to the workspace graph (validates I-6 CPU half).
- [ ] If `sim-ml-chassis`'s tape needed amendment for I-4: a sibling PR against `sim-ml-chassis` lands first, with its own gradcheck regression; skeleton PR depends on it.

Each unchecked item is a load-bearing claim from the book that did not survive contact with code — each one is a book-iteration trigger per the pivot memo's "Let skeleton findings drive subsequent book iterations."

## 6. What the skeleton does **not** validate

Deferred to the phase that first requires each, per [`110-crate/03-build-order.md`](../../../docs/studies/soft_body_architecture/src/110-crate/03-build-order.md):

| Deferred invariant / capability | Deferred to | Why not in skeleton |
|---|---|---|
| IPC contact barrier, CCD, friction | Phase C | Hardest single module; deserves its own focus. `NullContact` proves the trait surface. |
| Multiple materials / Mooney-Rivlin / viscoelasticity | Phase H | Additive to `Material` trait; does not affect skeleton invariants. |
| Tet10, Hex8, mixed u-p, adaptive refinement | Phase H | Same — additive to `Element` / `Mesh`. |
| GPU solver port, CpuTape-vs-GpuTape gradcheck (§03) | Phase E | wgpu build-graph claim is deferred with it per I-6 narrowing above. |
| `sim-core` / `sim-thermostat` coupling | Phase F | Sibling-crate coupling lives in `coupling/`, not skeleton. |
| `cf-design` / SDF → tet pipeline | Phase G | `sdf_bridge/` module is Phase G. |
| Visual layer, `sim-bevy` integration | Phase I | Physics substrate must be stable before shaders commit. |
| Time adjoint (multi-step rollout gradients) | Phase E+ | One-step Newton exercises I-3; multi-step exercises the time-adjoint machinery ([`60-differentiability/03-time-adjoint.md`](../../../docs/studies/soft_body_architecture/src/60-differentiability/03-time-adjoint.md)) which is not a skeleton invariant. |
| 100-tet neo-Hookean cube (Phase B deliverable) | Phase B proper | Skeleton is 1 tet; scaling to 100 tets is a Phase-B acceptance milestone after skeleton confirms the architecture. |
| Pass 2 / Pass 3 leaves not yet authored | Code-driven, JIT | Per pivot memo: "book iteration is code-driven post-skeleton, not spec-driven." |

## 7. Sequencing

1. **Memo review** (this document) — user reviews before any Rust lands.
2. **`sim-ml-chassis` tape audit** — quick read of its existing autograd surface to confirm I-4's `TapeNodeKey` assumption. If confirmed, skip step 3. If not, step 3 comes first.
3. (Conditional) **`sim-ml-chassis` amendment PR** — add `register_vjp` API surface, with its own regression. Must ship before skeleton PR touches `autograd/`.
4. **Skeleton crate scaffold** — `sim/L0/soft/` with `Cargo.toml`, seven trait definitions, seven empty impls, scene constructor stub. `cargo build` must pass.
5. **Material → Element → Mesh** — fill in neo-Hookean, Tet4, TetMesh. Each gets its own unit test before the next lands. `material/` and `mesh/` each ship their own isolated FD-vs-analytic gradient test per Phase A commitment.
6. **Solver → Autograd** — Newton loop, faer `Llt<f64>` factor-on-tape, IFT adjoint. Gradcheck §02 turns green at the end of this step.
7. **Readout → ForwardMap** — `RewardBreakdown` minimal (one or two terms), `ForwardMap::evaluate` composes the stack, `NullContact` locked in. Gradcheck §01 turns green.
8. **Green checklist pass** — all items in §5 verified.
9. **Book iteration pass** — every surprise encountered in steps 4–8 is recorded as a book-spec bug; fixes land as follow-up PRs to the book, not bundled into the skeleton PR.

All steps on one long-running branch. No merge until the green checklist in §5 passes end-to-end.

## 8. Open questions before coding starts

None blocking. Two to confirm with the user after memo review:

- **Q1:** Is `NullContact` acceptable for the skeleton, or should we push penalty contact into the skeleton anyway to stress-test the full trait surface? (Recommendation: `NullContact`. Rationale in §3.)
- **Q2:** One-step backward-Euler vs. static-equilibrium for the skeleton's canonical scene? (Recommendation: one-step backward-Euler with $dt$ large enough for inertia to be non-trivial. Rationale in §3.)
