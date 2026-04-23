# `sim-soft` Walking-Skeleton Scope

**Status:** Scope memo — post-refactor rewrite. Ready to stress-test before code lands.
**Date:** 2026-04-23, post PR #213 (`77751866`) — platform refactor soft-body-readiness MERGED.
**Supersedes:** the 2026-04-21 draft at commit `ee192303` (pre-refactor, marked SUPERSEDED at branch-tip).
**Follows:** [`project_soft_body_walking_skeleton_pivot.md`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/project_soft_body_walking_skeleton_pivot.md) — user directive to close the spec-to-code ratio after Pass 3 δ.
**Book spec:** [`docs/studies/soft_body_architecture/`](../../../docs/studies/soft_body_architecture/) at tip `77751866`.
**Target:** ~500–1500 LOC Rust, single long-running branch `feature/soft-body-walking-skeleton`.

The book is the spec. This memo does not re-argue architecture; it picks the minimum slice required to operationally validate six load-bearing invariants, names binary pass/fail criteria, and flags stress-test hooks the user should refine before coding starts.

## 0. Platform baseline — what PR #213 shipped

Refactor commits on main at `77751866`:

| Shipped | Not shipped |
|---|---|
| `Tensor<T>` generic, default `T = f32` (RL path), `Tensor<f64>` available for sim-soft | `sim_ml_chassis::gpu` submodule (B.5 ungated) |
| Vector-aware `Tape` — nodes carry `Tensor<f64>`, scalar API still lifts/unwraps rank-0 | GPU tensor type, chassis-GPU VJP path |
| `VjpOp` trait with `vjp(&self, cotangent: &Tensor<f64>, parent_cotans: &mut [Tensor<f64>])` | Any GPU solver or preconditioner |
| `Tape::push_custom` — the extension point sim-soft uses | Warm-start tensor stashing beyond scalar Var cotangents |
| 9 built-in elementwise primitives + 3 fused compositions (`add`/`sub`/`mul`/`neg`/`tanh`/`relu`/`square`/`ln`/`exp` + `affine`/`sum`/`mean`) | Broadcasting in primitive ops (strict same-shape) |

Skeleton invariant consequences:
- **I-4** ("chassis tape supports per-op VJPs") has the API it needs — the skeleton's job is to write a real physics VJP and verify it composes.
- **I-6** splits into two sub-claims: (6a) CPU build graph, tested implicitly by `cargo build -p sim-soft`; (6b) wgpu build graph + round-trip, tested by a feature-gated probe. Chassis-GPU (B.5) is a Phase-E prerequisite, explicitly out of skeleton scope.

## 1. The six load-bearing invariants

Each invariant is a book claim that must be verified in code before further book expansion. Citations are to the book at `77751866`.

| # | Invariant | Book anchor | What green looks like |
|---|---|---|---|
| I-1 | Seven-trait core surface composes without ceremony explosion | [`110-crate/01-traits/00-core.md`](../../../docs/studies/soft_body_architecture/src/110-crate/01-traits/00-core.md) | All 7 traits (`Material`, `Element<const N, const G>`, `Mesh`, `ContactModel`, `Solver`, `Differentiable`, `Observable` with `type Step`) defined with pinned signatures; one concrete impl each; forward path composes without phantom-type gymnastics or `Box<dyn _>` beyond what the chassis already uses inside `BackwardOp::Custom`. |
| I-2 | `ForwardMap::evaluate → (RewardBreakdown, EditResult)` works in practice | [`100-optimization/00-forward.md`](../../../docs/studies/soft_body_architecture/src/100-optimization/00-forward.md) (γ-locked) | `SoftScene::one_tet_cube()` constructs a `ForwardMap`; `fm.evaluate(&theta, &mut tape)` returns a populated `RewardBreakdown` (every field finite) and an `EditResult::ParameterOnly`; §01 determinism holds across repeated calls. |
| I-3 | `faer::sparse::linalg::solvers::Llt<f64>` exposes factor-on-tape for IFT adjoint | [`50-time-integration/00-backward-euler.md`](../../../docs/studies/soft_body_architecture/src/50-time-integration/00-backward-euler.md), [`110-crate/03-build-order.md`](../../../docs/studies/soft_body_architecture/src/110-crate/03-build-order.md) Phase B | Forward Newton factors once; `Llt<f64>` is stashed inside a `NewtonStepVjp: VjpOp`; `fm.gradient` retrieves it, applies `solve_in_place` to $-\partial r / \partial \theta$, returns a scalar gradient; zero re-factorization. |
| I-4 | Chassis tape per-operation VJPs support sim-soft physics | [`60-differentiability/01-custom-vjps/00-registration.md`](../../../docs/studies/soft_body_architecture/src/60-differentiability/01-custom-vjps/00-registration.md), [`60-differentiability/00-what-autograd-needs.md`](../../../docs/studies/soft_body_architecture/src/60-differentiability/00-what-autograd-needs.md) Claim 3 | `NewtonStepVjp` registered via `Tape::push_custom`; composes with chassis primitives in the reward-composition path; `tape.backward(reward_scalar)` runs; `parent_cotans` accumulation works at physics-tensor scale (shape `[12]` displacement, `[>=4]` θ). |
| I-5 | Determinism-in-θ holds under realistic Newton convergence | [`110-crate/04-testing/03-gradcheck.md`](../../../docs/studies/soft_body_architecture/src/110-crate/04-testing/03-gradcheck.md) §01 | Bit-equal `RewardBreakdown` (all fields `f64::to_bits`) across (a) two same-process `evaluate` calls at identical θ, (b) one same-process call + one cold-restart via a tiny `bin` helper. Armijo line-search selection, sparse-assembly accumulation order, and faer's solve all deterministic at fixed θ. |
| I-6 | Rust + wgpu build graph (no FFI) tractable for soft-body FEM | Cross-cutting; [`110-crate/03-build-order.md`](../../../docs/studies/soft_body_architecture/src/110-crate/03-build-order.md) Phase-B/E | (6a) `cargo build -p sim-soft --release` resolves cleanly on macOS-arm64 + Linux CI, no C/C++ compile step added to the graph, under workspace compile-time budget. (6b) `cargo test -p sim-soft --features gpu-probe -- gpu_probe` acquires adapter, round-trips a buffer through a trivial compute dispatch. |

**I-6 scope clarification:** the probe tests wgpu build-graph tractability and round-trip correctness only. It does NOT exercise `sim_ml_chassis::gpu` (B.5 ungated), GPU sparse linalg, GPU autograd, or any chassis↔GPU state sharing. Adding any of those mid-skeleton is out-of-scope creep — they are Phase E. If no adapter is available (headless CI), the probe test skips cleanly rather than failing.

## 2. Minimal scene

**One tetrahedron.** Four vertices, one Tet4 element, one Gauss point at the centroid, compressible neo-Hookean material with hardcoded $(\mu, \lambda)$, Dirichlet boundary on three vertices (fully pinned), traction load on the fourth vertex parameterized by $\theta$.

| Component | Spec |
|---|---|
| Reference vertices | Unit tet: $v_0 = (0,0,0)$, $v_1 = (1,0,0)$, $v_2 = (0,1,0)$, $v_3 = (0,0,1)$ |
| Element | one Tet4, $N = 4$, $G = 1$ (centroid Gauss point, weight = $V_{\text{ref}} = 1/6$) |
| Material | compressible NH: $\Psi(F) = (\mu/2)(I_1 - 3) - \mu \ln J + (\lambda/2)(\ln J)^2$; hardcoded $\mu = 10^5$ Pa, $\lambda = 4 \times 10^5$ Pa (Ecoflex-class, $\nu \approx 0.40$) |
| Density | $\rho = 1030 \ \text{kg/m}^3$ (silicone-class) |
| Boundary | Dirichlet on $v_0, v_1, v_2$ (all 9 DOF fixed); $v_3$ free |
| Load | Traction $\mathbf{t}(\theta) \in \mathbb{R}^3$ applied to $v_3$; $\theta \in \mathbb{R}^4$ parameterizes magnitude + 3-axis direction |
| Initial state | Reference configuration; $\dot{\mathbf{x}}_0 = \mathbf{0}$ |
| Time step | Single backward-Euler step, $\Delta t = 10^{-2}$ s (large enough that inertia is a real term, not so small that the problem collapses to quasi-static) |
| System size | 12 DOF, 3 free after Dirichlet (only $v_3$ moves) |

**Reward scalar** — minimal `RewardBreakdown` for a 1-tet case (full γ-struct defined but not all fields meaningful on one element):

| Field | 1-tet definition | Phase-A meaningful? |
|---|---|---|
| `pressure_uniformity` | not meaningful on 1 tet — set to NaN sentinel or `None` if Option | **gap** |
| `coverage` | not meaningful — no contact pair | **gap** |
| `peak_bound` | $\max$ principal Cauchy stress at the Gauss point, barrier-compared to $\sigma_{\max} = 10^6$ Pa | yes |
| `stiffness_bound` | $\mu$ itself, barrier-compared to $k_{\min} = 5 \times 10^4$ | yes |
| `composition` | scalar-composed via `RewardWeights` | yes |

**Finding if this plays out:** two of five fields are 1-tet-indeterminate. Expected. Feed back to Part 1 Ch 01: either (a) document that single-element scenes under-specify `RewardBreakdown` (spec clarification), or (b) add a 1-tet-safe reduction for those fields (spec change). Not a skeleton failure — a skeleton finding.

Rationale for scene choices:
- Smallest scene that exercises Jacobian assembly, sparse factorization, and IFT back-substitution simultaneously.
- One Gauss point eliminates quadrature-weight bugs as a confound.
- 3-DOF free system keeps FD sweep trivial for 5-digit gradcheck.
- Hand-rolled — no sim-mjcf / sim-core dependency, keeps (6a) dep-graph minimum per Part 11 Ch 02 §00 deferral.

## 3. Material / contact / integrator combo — confirmed decisions

| Axis | Chosen | Alternatives considered | Rationale |
|---|---|---|---|
| Material | NH compressible (above) | Mooney-Rivlin, corotational, Ogden | NH is Phase A baseline; closed-form $P$ and $\partial P / \partial F$ in Part 2 Ch 04; no time-history to confound I-5. |
| Element | Tet4, $N=4$, $G=1$ | Tet10, Hex8 | Tet4 is Phase A; Tet10 is Phase H per Part 3 Ch 00. |
| Contact | **None** — `NullContact` impl | IPC, penalty | IPC is Phase C; penalty is explicitly rejected in `contact/` Ch 00. `NullContact { active_pairs: [] }` proves I-1 without scope creep. Dirichlet + traction stands in for contact at 1-tet scale. |
| Integrator | Backward-Euler Newton + Armijo line-search, single step | Static equilibrium, Newton w/o line-search, strong-Wolfe | Backward-Euler is committed per Part 5 Ch 00; one step exercises the factor-on-tape pattern; Armijo is simplest compliant line-search. **Flag for I-5:** line-search selection branching is the leading determinism-risk source; test covers exactly this. |
| Linear solver | faer `Llt<f64>` on sparse | nalgebra dense, LDLᵀ (`Lblt`) | faer `Llt<f64>` is the I-3 target type; sparse is forward-compatible with Phase-B 100-tet scaling. **Risk:** 3-DOF free system is SPD only if the tangent is SPD at equilibrium — NH is SPD near the reference configuration; verify at the chosen load magnitude before committing, fall back to `Lblt` if not. |
| Autograd tape | chassis CPU tape + `VjpOp` + `Tape::push_custom` | `Burn`, `dfdx`, `tch-rs` | "Own every line" thesis. Refactor A.1/B.1 shipped the surface. |
| Gradcheck | §01 determinism + §02 FD-vs-IFT at 5 digits, central FD, $h = 1.5 \times 10^{-8}$ | §03 CPU-vs-GPU, forward FD, one-sided | §03 is Phase E (skeleton is CPU-only). Central FD is optimal at 5-digit bar. |
| GPU probe | minimal wgpu round-trip, feature-gated `gpu-probe` | full chassis-GPU path, no probe at all | Chassis-GPU is B.5-gated (not shipped). Minimal probe tests (6b) without committing to B.5 scope. |
| Mesh fixture | hand-rolled 1-tet (4 vertices hardcoded in `Mesh` impl) | sim-mjcf ingest, MJCF file | Skeleton stays self-contained; sim-mjcf coupling is Phase F. |

## 4. Crate and module layout

Location: `sim/L0/soft/` — sibling of `{types, simd, core, thermostat, gpu, mjcf, urdf, tests, ml-chassis, rl, opt}`.

```
sim/L0/soft/
├── Cargo.toml              # deps: sim-ml-chassis, sim-types, faer, nalgebra; optional wgpu behind gpu-probe
├── src/
│   ├── lib.rs              # pub re-exports
│   ├── material/
│   │   ├── mod.rs          # Material trait
│   │   └── neo_hookean.rs  # NeoHookean impl + closed-form P, ∂P/∂F
│   ├── element/
│   │   ├── mod.rs          # Element<const N, const G> trait
│   │   └── tet4.rs         # impl Element<4, 1>
│   ├── mesh/
│   │   ├── mod.rs          # Mesh trait
│   │   └── single_tet.rs   # hand-rolled 1-tet, 4 vertices hardcoded
│   ├── contact/
│   │   ├── mod.rs          # ContactModel trait
│   │   └── null.rs         # NullContact (empty active_pairs, zero potential)
│   ├── solver/
│   │   ├── mod.rs          # Solver trait (dyn-safe)
│   │   └── backward_euler.rs  # BackwardEulerNewton, Armijo LS, faer Llt
│   ├── differentiable/
│   │   ├── mod.rs          # Differentiable trait
│   │   └── newton_vjp.rs   # NewtonStepVjp: VjpOp, stashes Llt<f64> + ∂r/∂θ
│   ├── observable/
│   │   ├── mod.rs          # Observable trait + type Step
│   │   └── basic.rs        # BasicReadout impl
│   ├── readout/
│   │   ├── mod.rs          # ForwardMap impl
│   │   ├── reward_breakdown.rs  # RewardBreakdown struct + apply_residuals
│   │   ├── edit_result.rs  # EditResult enum (3 variants)
│   │   ├── reward_weights.rs   # RewardWeights
│   │   ├── gradient_estimate.rs  # GradientEstimate { Exact, Noisy { variance } }
│   │   └── scene.rs        # SoftScene::one_tet_cube()
│   └── gpu_probe/          # #[cfg(feature = "gpu-probe")]
│       ├── mod.rs          # adapter + buffer round-trip
│       └── probe.wgsl      # trivial compute shader (buf[i] *= 2.0)
├── tests/
│   ├── invariant_1_compose.rs      # 7-trait composition smoke
│   ├── invariant_2_forward.rs      # ForwardMap end-to-end
│   ├── invariant_3_factor.rs       # Llt<f64> lifetime + solve
│   ├── invariant_4_5_gradcheck.rs  # FD vs IFT, 5-digit, joint I-4 + I-5
│   ├── invariant_5_determinism.rs  # bit-equal + cold-restart via bin helper
│   └── invariant_6_gpu_probe.rs    # #[cfg(feature = "gpu-probe")]
└── bin/
    └── skeleton_run.rs             # cold-restart helper for I-5
```

**Dependency graph** (skeleton scope):

```
sim-ml-chassis  ──── Tape, VjpOp, Tensor<f64>
     │
     ▼
 sim-soft  ──┬── faer          (Llt<f64>, sparse CSR)
             ├── nalgebra      (Tet4 local 12×12 dense)
             └── wgpu (opt)    (probe, feature-gated)
```

Absent from skeleton (deferred by phase): `sim-core` (F), `sim-mjcf` (F), `sim-thermostat` (F/G), `sim-bevy` (I), `cf-design` (G), `sim-opt` (post-I).

**LOC budget:** ~800 production, ~400 tests, target total ~1200. Room at the top end (1500) for unavoidable growth in `NewtonStepVjp` + the Tet4 local-assembly routine if the closed-form NH derivatives expand beyond estimate.

## 5. Proposed new API — skeleton must define

Names locked by γ ([`project_soft_body_gamma_apis.md`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/project_soft_body_gamma_apis.md)) but not yet in Rust. Skeleton writes these — first contact with code is expected to surface refinements feeding back into Part 10 Ch 00.

- **`RewardBreakdown`** (γ) — struct with per-term fields per Part 1 Ch 01. 1-tet values: see §2 reward table. `apply_residuals(&self, residuals: &ResidualCorrections) -> Self` per δ Ch 00 readout §3.
- **`EditResult`** (γ) — three-variant enum: `ParameterOnly`, `TopologyPreserving`, `TopologyChanging`. 1-tet θ-only variation always yields `ParameterOnly`.
- **`GradientEstimate`** (γ) — `Exact` | `Noisy { variance: f64 }`. Skeleton always returns `Exact`; `Noisy` path activates at Phase G (stochastic adjoint).
- **`RewardWeights`** (γ) — scalar composition weights matching `RewardBreakdown` fields.
- **`ForwardMap` trait** (γ) — exactly the signature in `project_soft_body_gamma_apis.md`, including the determinism-in-θ docstring contract.
- **`NewtonStepVjp: VjpOp`** — the numerical heart of the skeleton. Stashes: converged $\mathbf{x}^*$, $\theta$, `Llt<f64>` factor of $\partial r / \partial \mathbf{x} \big|_{\mathbf{x}^*, \theta}$, sparse $\partial r / \partial \theta \big|_{\mathbf{x}^*, \theta}$. VJP formula: given cotangent $\bar{\mathbf{x}}^*$ (gradient flowing in from downstream reward-composition path), compute adjoint $\lambda = -(\partial r / \partial \mathbf{x})^{-\mathsf{T}} \bar{\mathbf{x}}^*$ via `factor.solve_in_place` on the transposed system (Cholesky is symmetric so transpose is identity), then $\partial_\theta = (\partial r / \partial \theta)^{\mathsf{T}} \lambda$ — accumulated into `parent_cotans[θ]`. **Sign convention check:** assumes residual $r(\mathbf{x}, \theta) = 0$ at equilibrium; if the skeleton writes $r$ as "residual = inertia + internal − external," the gradient carries the sign as written. **Verify against Part 6 Ch 02:13–15 before coding.**

## 6. Test harness

| Inv | Test file | Kind | Fixture | Runtime target |
|---|---|---|---|---|
| I-1 | `invariant_1_compose.rs` | integration | Construct `BackwardEulerNewton<Tet4, NeoHookean, NullContact, CpuTape>` via type alias; call each trait's contract method once | < 1 s |
| I-2 | `invariant_2_forward.rs` | integration | 1-tet scene, θ = `[f_magnitude, f_x, f_y, f_z]`, call `evaluate`; assert `peak_bound`/`stiffness_bound`/`composition` finite; flag `pressure_uniformity`/`coverage` gap; assert `EditResult::ParameterOnly` | < 2 s |
| I-3 | `invariant_3_factor.rs` | unit-style integration | Assemble 12×12 synthetic SPD matrix, `Llt::new`, stash in `NewtonStepVjp`, drop source matrix, `solve_in_place` on two different RHSes, assert consistency | < 1 s |
| I-4 + I-5 | `invariant_4_5_gradcheck.rs` | integration | Central FD with $h = 1.5 \times 10^{-8}$ vs IFT gradient; 4 θ components; relative error per component ≤ 1e-5 | < 10 s |
| I-5 | `invariant_5_determinism.rs` | integration | Two same-process `evaluate` calls → `f64::to_bits` equal on every field; spawn `bin/skeleton_run` subprocess, parse stdout JSON, compare bits | < 5 s |
| I-6a | (implicit) | build | `cargo build -p sim-soft --release` succeeds | — |
| I-6b | `invariant_6_gpu_probe.rs` | integration, feature-gated | `#[cfg(feature = "gpu-probe")]`; request adapter (`LowPower`); upload `[f32; 16]`; dispatch probe.wgsl doubling each element; read back; assert equality; skip cleanly on `RequestAdapterError` | < 5 s |

**Full skeleton test suite target:** `cargo test -p sim-soft --release` under 30 s; `cargo test -p sim-soft --release --features gpu-probe` under 35 s.

**No benches** in skeleton. Performance is Phase E+.

## 7. Green-skeleton checklist

Skeleton is **green** when all of the following hold on `feature/soft-body-walking-skeleton`:

- [ ] `cargo build -p sim-soft` + `cargo build -p sim-soft --release` succeed on macOS-arm64 + Linux CI.
- [ ] `cargo build -p sim-soft --features gpu-probe` succeeds on both platforms.
- [ ] `cargo test -p sim-soft --release` — all six non-GPU invariants green.
- [ ] `cargo test -p sim-soft --release --features gpu-probe` — I-6b green on a machine with a wgpu-compatible adapter; skipped cleanly on CI if none.
- [ ] `cargo xtask grade sim-soft` returns **A** across all 7 criteria. (User standard: A-grade or it doesn't ship.)
- [ ] All 7 traits from [`110-crate/01-traits/00-core.md`](../../../docs/studies/soft_body_architecture/src/110-crate/01-traits/00-core.md) defined with pinned signatures, one impl each (I-1).
- [ ] `cargo tree -p sim-soft` — no C/C++ build step; wgpu + faer + nalgebra + sim-ml-chassis pure-Rust chain (I-6a).
- [ ] No `unsafe` in sim-soft production code (grader enforces; sanity-check).
- [ ] Gradcheck relative error ≤ 1e-5 per component on all 4 θ axes (I-4 + I-5).
- [ ] `RewardBreakdown` 1-tet gap recorded as a book-iteration finding, not silently worked around.

## 8. What the skeleton does NOT validate

Deferred to the phase that first requires each:

| Deferred | Phase | Why not in skeleton |
|---|---|---|
| IPC contact, CCD, friction | C | Hardest single module; `NullContact` proves the trait surface. |
| Mooney-Rivlin / Ogden / corotational / viscoelasticity / anisotropy / thermal coupling / spatial fields | D/H | Additive to `Material`; no new trait surface needed. |
| Tet10, Hex8, mixed u-p, adaptive refinement, F-bar | H | Additive to `Element` / `Mesh`. |
| Chassis-GPU (`sim_ml_chassis::gpu`, B.5 ungated) | E | Refactor explicitly deferred; Part 8 Ch 04 Claim 3 prerequisite for Phase E. |
| GPU sparse CG, preconditioner-on-tape | E | Part 8 Ch 02; requires B.5 first. |
| `sim-core` / `sim-mjcf` coupling (wrench-only, `xfrc_applied`) | F | sim-soft is self-contained at skeleton scope per Part 11 Ch 02 §00. |
| `sim-thermostat` coupling (Langevin stochastic forcing) | F/G | Would break determinism-in-θ; excluded deliberately per γ Ch 00 contract. |
| `sim-bevy` visual layer (`MeshSnapshot`, `SimSoftPlugin`) | I | Physics substrate must be stable before shaders commit. |
| `cf-design` / SDF→tet pipeline / `SdfField` | G | `sdf_bridge/` module is Phase G. |
| Time adjoint (multi-step rollout gradients, Revolve checkpointing) | E+ | Single-step backward-Euler exercises I-3; multi-step is distinct machinery. |
| `SimToRealCorrection`, `MeasurementReport`, residual GPs, preference GPs | post-I | Physical-print loop; explicitly post-Phase-I per γ Ch 03/04/05/06. |
| `Acquisition` enum, `CostAwareAcquisition`, `DuelingBanditPolicy` | post-I | sim-opt / optimizer surface; not skeleton scope. |
| 100-tet Phase-B deliverable | B proper | Skeleton is 1 tet; scaling is a Phase-B acceptance milestone. |

## 9. Sequencing

1. **Memo stress-test** (this document) — user reviews §10 + §11, approves / refines / pushes back before any Rust lands.
2. **Crate scaffold** — `sim/L0/soft/` with `Cargo.toml`, seven trait definitions with pinned signatures, seven empty impls. `cargo build` passes. Validates I-1 compile layer.
3. **Material → Element → Mesh** — NeoHookean closed-form $P$ + $\partial P / \partial F$; Tet4 local assembly; single-tet mesh. Unit FD-vs-analytic test per trait before moving on.
4. **Solver** — Newton loop with Armijo, faer `Llt<f64>` on sparse assembly; convergence test at 1-tet scale. Verify SPD tangent at chosen load; fall back to `Lblt` if not.
5. **Differentiable (`NewtonStepVjp`)** — stash primal data, write VJP, verify sign convention against Part 6 Ch 02:13–15. Register with `Tape::push_custom`. Gradcheck §02 (I-4 + I-5 joint) turns green.
6. **Observable → readout → `ForwardMap`** — `RewardBreakdown`, `EditResult`, `RewardWeights`, `GradientEstimate` definitions; `ForwardMap::evaluate` composes the stack. Determinism test (I-5) turns green. `RewardBreakdown` 1-tet gap recorded as finding.
7. **GPU probe** — feature-gated wgpu adapter + round-trip. I-6b turns green.
8. **Green-checklist pass** — §7 verified end-to-end.
9. **Book iteration pass** — every surprise encountered in 2–8 becomes a book-spec PR, separate from the skeleton PR.

All steps on `feature/soft-body-walking-skeleton`. No merge until §7 passes.

## 10. Recommendations flagged for user check (recommend-first)

Per [`feedback_recommend_first_deep_specialist.md`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_recommend_first_deep_specialist.md): numerically-load-bearing choices below are Claude's call + rationale, flagged for user check against principles.

| # | Call | Rationale | User-check hook |
|---|---|---|---|
| R-1 | Full Newton + Armijo line-search, single backward-Euler step, $\Delta t = 10^{-2}$ | Simplest compliant choice matching Part 5 Ch 00; inertia non-trivial at $\Delta t = 10^{-2}$ with $\rho = 1030$. | Confirm $\Delta t$ isn't so large that Newton divergence is the bottleneck; alternatively $\Delta t \in [10^{-3}, 10^{-2}]$. |
| R-2 | faer `Llt<f64>` on sparse CSR | Directly tests I-3; forward-compatible with Phase-B 100-tet. | If NH tangent isn't SPD at the chosen load, fall back to `Lblt` (LDLᵀ) — verify at R-1 step. |
| R-3 | Central FD, $h = 1.5 \times 10^{-8}$ ($\approx \sqrt{\varepsilon_{f64}}$), 5-digit relative bar | Optimal for f64 precision; matches Part 11 Ch 04 §02 commitment. | Hard fail at per-component rel error > 1e-4; warn at > 1e-5. 5/6 components passing is NOT "almost working" — treat as hard fail. |
| R-4 | Minimal wgpu probe (~100–200 LOC, feature-gated) | Tests I-6b without committing to chassis-GPU (B.5 ungated). | User approved the probe; confirm scope guard (no chassis-GPU, no GPU solver, no shared tape state). |
| R-5 | `NewtonStepVjp` sign convention per residual form "inertia + internal − external" | Part 6 Ch 02:13–15 derivation; gradient sign follows. | **Verify Part 6 Ch 02:13–15 residual sign before coding VJP.** Single cheapest check that saves the most debugging time. |
| R-6 | θ = 4 scalars (traction magnitude + 3-axis direction) | Keeps FD sweep cheap; tests vector-cotangent accumulation at physics scale. | Is 4 the right count? Could narrow to 1 (magnitude only) for the very first gradcheck, widen to 4 after passing. |
| R-7 | Hand-rolled 1-tet in `mesh/single_tet.rs`, no sim-mjcf | User-confirmed. | — |
| R-8 | Crate name `sim-soft` at `sim/L0/soft/` | User-confirmed. | — |

## 11. Stress-test / review hooks (refine before coding)

Open questions the user should push on or confirm — these are where the spec is most likely wrong:

- **S-1 — `RewardBreakdown` gap on 1 tet.** Two of five fields (`pressure_uniformity`, `coverage`) are structurally undefined on a single element with no contact. Skeleton flags this as a finding. Acceptable? Or do we want the skeleton to include a 2-tet or tet-plus-rigid-wall scene to exercise all five fields and remove the gap? (Recommendation: accept the gap. Rationale: 2 tets ≈ 1.5× LOC for no additional invariant coverage; the finding itself is the point.)
- **S-2 — Determinism-under-line-search risk class.** Armijo's step-length selection branches on a sufficient-decrease inequality. If θ-perturbation changes the decrease quantity across the inequality threshold, the selected step length changes, propagating into non-smooth gradient. This would look like a gradcheck failure at 5 digits despite correct IFT. Is the skeleton gradcheck test the right place to expose this, or do we want a separate "line-search non-smoothness audit" test that holds θ fixed and varies the search-direction-norm to isolate the branch point? (Recommendation: the gradcheck test is sufficient; a failure maps to a book-level spec bug in Part 5 Ch 00 that deserves its own finding, and we don't need a dedicated test harness for it.)
- **S-3 — `faer::sparse::linalg::solvers::Llt<f64>` lifetime shape.** 80% probability it clones/moves cleanly across the VJP boundary; 20% it holds a lifetime to the source matrix we'd need to route around. 10-minute `cargo doc` spike before writing `NewtonStepVjp` would convert this to a known fact. Do that spike as the first step of skeleton coding, or commit to it in the memo?
- **S-4 — `parent_cotans` accumulation at physics scale.** Chassis contract says slots are "zero-initialized with shapes matching parent values." Refactor tests almost certainly exercised scalar + short-vector only. 1-tet θ is shape `[4]` or `[]+[3]`; displacement is shape `[12]` or `[4,3]`. Shape inference for custom VjpOps — does the tape actually know the parent's shape at `push_custom` time, or does it learn it at `backward` time? If the latter, the contract might not hold for shapes larger than scalar. Worth a code-read of `autograd.rs` push_custom path before writing the VJP.
- **S-5 — Cold-restart determinism via bin helper.** Is a subprocess-spawn test the right mechanism, or should the cold-restart path be exercised in a separate CI job that actually restarts the process? (Recommendation: subprocess spawn is faster and sufficient for the invariant; full CI restart isn't worth the infrastructure.)
- **S-6 — `EditResult` three-variant enum.** γ locked `ParameterOnly` / `TopologyPreserving` / `TopologyChanging` conceptually. Does the 1-tet θ-only case actually clarify the trichotomy, or does it just ever return `ParameterOnly` and leave the other two variants untested? (Answer: yes, only `ParameterOnly` fires. The other two reactivate at Phase G (SDF→mesh topology changes). Skeleton doesn't exercise them. Accept.)
- **S-7 — Nalgebra vs faer for Tet4 locals.** `Tet4::local_stiffness` returns a 12×12 dense matrix before scatter to sparse global. Using nalgebra for the local and faer for the global is two dense libraries — one extra dep. Acceptable, or collapse to faer-only dense? (Recommendation: keep nalgebra. Small 12×12 locals are faster in nalgebra's stack-allocated `SMatrix`; faer's strength is sparse. Two libraries with one-each role is cleaner than one library wearing both hats.)
- **S-8 — Feature-gate strategy for wgpu.** Should `gpu-probe` be the only wgpu entry, or should it also gate whatever Phase-E GPU code lands later? (Recommendation: `gpu-probe` is specifically the probe. Phase E adds `gpu` feature with its own gate for real GPU work. Two features. Keeps probe scope-locked.)
- **S-9 — Skeleton PR merge strategy.** One big PR at green-checklist, or staged PRs per invariant? (Recommendation: one PR, matches soft-body study PR pattern; pre-squash tag per `feedback_pre_squash_tag.md`.)
- **S-10 — Book-iteration cadence.** Every skeleton finding becomes a book-level PR (separate from the skeleton PR). Batched into one follow-up or dripped as each finding surfaces? (Recommendation: dripped. Surprise-finding at coding time produces freshest spec fix; batching loses context.)

## 12. Known unknowns with probability estimates

- **80% clean, 20% refactor needed:** faer `Llt<f64>` cleanly movable onto a VJP with owned lifetime. Mitigation: S-3 spike.
- **70% clean, 30% edge case:** chassis `push_custom` + `parent_cotans` accumulation works at `Tensor<f64>` shape `[12]` and `[4]`. Mitigation: S-4 code-read.
- **90% holds, 10% non-smooth:** Armijo line-search preserves determinism under θ-perturbation at gradcheck's $h = 1.5 \times 10^{-8}$ scale. Mitigation: the gradcheck test itself exposes this.
- **95% SPD, 5% indefinite:** NH compressible tangent stays SPD at $\Delta t = 10^{-2}$ with the chosen $(\mu, \lambda, \rho)$ and traction magnitude. Mitigation: fall back to `Lblt`; flag if it triggers.
- **99% clean, 1% surprise:** wgpu build graph resolves cross-platform (macOS-arm64 + Linux CI). Mitigation: the probe test is the check.

If any "refactor needed" scenario fires, the skeleton PR stays open and the refactor is a prerequisite commit on the same branch.

---

## Appendix: γ-locked API name registry (skeleton must cite verbatim)

From [`project_soft_body_gamma_apis.md`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/project_soft_body_gamma_apis.md):

- `ForwardMap` (trait) — `evaluate(theta, tape) → (RewardBreakdown, EditResult)` + `gradient(theta, tape) → (Tensor<f64>, GradientEstimate)`
- `RewardBreakdown` (struct) — per-term reward struct with `apply_residuals` method
- `EditResult` (enum) — 3 variants
- `RewardWeights` (struct) — scalar-composition weights
- `GradientEstimate` (enum) — `Exact | Noisy { variance: f64 }`
- `Material`, `Element<const N, const G>`, `Mesh`, `ContactModel`, `Solver`, `Differentiable`, `Observable` (with `type Step`) — the seven traits
- `faer::sparse::linalg::solvers::Llt<f64>` — Cholesky factor type (factor-on-tape)
- `VjpOp` (trait), `Tape::push_custom` (method) — chassis extension point (shipped in PR #213)
- `Tensor<f64>` — sim-soft precision (chassis default is f32 for RL)

Not cited in skeleton (post-skeleton API register for reference): `SimToRealCorrection`, `MeasurementReport`, `OnlineUpdateOutcome`, `PreferenceGP`, `BradleyTerryGP`, `GradientEnhancedGP`, `Acquisition` enum, `CostAwareAcquisition`, `DuelingBanditPolicy`, `MeasurementProtocol`, `DiffusionProfile`, `MjcfHandshake`, `ThermostatHandshake`, `MeshSnapshot`, `SimSoftPlugin`, `SdfField`, `MaterialField`.
