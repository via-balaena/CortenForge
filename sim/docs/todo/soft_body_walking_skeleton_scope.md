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

**Round 1 stress-test results (2026-04-23):** S-3 (faer `Llt` API), S-4 (chassis push_custom + backward at physics tensor scale), R-5 (NewtonStepVjp sign convention) all **CONFIRMED CLEAN**. Three book findings (BF-1, BF-2, BF-3) queued for Pass 2/3 — Cholesky type has two generic parameters not one, and Part 6 Ch 02 code sketches at lines 26 + 34 are pseudocode (correct math at lines 13-15).

**Round 2 stress-test results (2026-04-23):** R-1 (Δt + scene numerics) + R-2 (NH tangent SPD) + NH closed forms verified against [Part 2 Ch 04 NH sub-leaves](../../../docs/studies/soft_body_architecture/src/20-materials/04-hyperelastic/00-neo-hookean/). **One scene change:** tet edge 1 m → **0.1 m** — the 1-m unit tet was inertia-dominated by ~26× at $\Delta t = 10^{-2}$ (Newton would converge fast but NH nonlinearity wouldn't exercise); decimeter tet gives stiffness-dominated by ~4× with meaningful NH activation. Decimeter scale also matches canonical soft-robotics problem (compliant gripper / catheter sheath). **One API correction:** faer 0.24's `sparse::linalg::solvers` exposes only `Llt`, `Lu`, `Qr` — **no LDLᵀ variant**. Fallback for "NH tangent not SPD" is faer `Lu` (aligned with [Part 2 Ch 04 01-tangent.md:41](../../../docs/studies/soft_body_architecture/src/20-materials/04-hyperelastic/00-neo-hookean/01-tangent.md) already committing to LU fallback). Skeleton implements from the math + verified faer 0.24 API. See §11 and §13 for details.

## 1. The six load-bearing invariants

Each invariant is a book claim that must be verified in code before further book expansion. Citations are to the book at `77751866`.

| # | Invariant | Book anchor | What green looks like |
|---|---|---|---|
| I-1 | Seven-trait core surface composes without ceremony explosion | [`110-crate/01-traits/00-core.md`](../../../docs/studies/soft_body_architecture/src/110-crate/01-traits/00-core.md) | All 7 traits (`Material`, `Element<const N, const G>`, `Mesh`, `ContactModel`, `Solver`, `Differentiable`, `Observable` with `type Step`) defined with pinned signatures; one concrete impl each; forward path composes without phantom-type gymnastics or `Box<dyn _>` beyond what the chassis already uses inside `BackwardOp::Custom`. |
| I-2 | `ForwardMap::evaluate → (RewardBreakdown, EditResult)` works in practice | [`100-optimization/00-forward.md`](../../../docs/studies/soft_body_architecture/src/100-optimization/00-forward.md) (γ-locked) | `SoftScene::one_tet_cube()` constructs a `ForwardMap`; `fm.evaluate(&theta, &mut tape)` returns a populated `RewardBreakdown` (every field finite) and an `EditResult::ParameterOnly`; §01 determinism holds across repeated calls. |
| I-3 | `faer::sparse::linalg::solvers::Llt<I, f64>` exposes factor-on-tape for IFT adjoint | [`50-time-integration/00-backward-euler.md`](../../../docs/studies/soft_body_architecture/src/50-time-integration/00-backward-euler.md), [`110-crate/03-build-order.md`](../../../docs/studies/soft_body_architecture/src/110-crate/03-build-order.md) Phase B | Forward Newton factors once (symbolic + numeric, two-step per faer 0.24 API); `Llt<I, f64>` is stashed inside a `NewtonStepVjp: VjpOp` (owns its data, `Clone` + `Send + Sync` under `f64`); `fm.gradient` retrieves it, applies `solve_in_place_with_conj` to `-∂r/∂θ` via faer's `MatMut` bridging, returns a scalar gradient; zero re-factorization across backward. |
| I-4 | Chassis tape per-operation VJPs support sim-soft physics | [`60-differentiability/01-custom-vjps/00-registration.md`](../../../docs/studies/soft_body_architecture/src/60-differentiability/01-custom-vjps/00-registration.md), [`60-differentiability/00-what-autograd-needs.md`](../../../docs/studies/soft_body_architecture/src/60-differentiability/00-what-autograd-needs.md) Claim 3 | `NewtonStepVjp` registered via `Tape::push_custom`; composes with chassis primitives in the reward-composition path; `tape.backward(reward_scalar)` runs; `parent_cotans` accumulation works at physics-tensor scale (shape `[12]` displacement, `[>=4]` θ). |
| I-5 | Determinism-in-θ holds under realistic Newton convergence | [`110-crate/04-testing/03-gradcheck.md`](../../../docs/studies/soft_body_architecture/src/110-crate/04-testing/03-gradcheck.md) §01 | Bit-equal `RewardBreakdown` (all fields `f64::to_bits`) across (a) two same-process `evaluate` calls at identical θ, (b) one same-process call + one cold-restart via a tiny `bin` helper. Armijo line-search selection, sparse-assembly accumulation order, and faer's solve all deterministic at fixed θ. |
| I-6 | Rust + wgpu build graph (no FFI) tractable for soft-body FEM | Cross-cutting; [`110-crate/03-build-order.md`](../../../docs/studies/soft_body_architecture/src/110-crate/03-build-order.md) Phase-B/E | (6a) `cargo build -p sim-soft --release` resolves cleanly on macOS-arm64 + Linux CI, no C/C++ compile step added to the graph, under workspace compile-time budget. (6b) `cargo test -p sim-soft --features gpu-probe -- gpu_probe` acquires adapter, round-trips a buffer through a trivial compute dispatch. |

**I-6 scope clarification:** the probe tests wgpu build-graph tractability and round-trip correctness only. It does NOT exercise `sim_ml_chassis::gpu` (B.5 ungated), GPU sparse linalg, GPU autograd, or any chassis↔GPU state sharing. Adding any of those mid-skeleton is out-of-scope creep — they are Phase E. If no adapter is available (headless CI), the probe test skips cleanly rather than failing.

## 2. Minimal scene

**One tetrahedron.** Four vertices, one Tet4 element, one Gauss point at the centroid, compressible neo-Hookean material with hardcoded $(\mu, \lambda)$, Dirichlet boundary on three vertices (fully pinned), traction load on the fourth vertex parameterized by $\theta$.

| Component | Spec |
|---|---|
| Reference vertices | Decimeter tet: $v_0 = (0,0,0)$, $v_1 = (0.1,0,0)$, $v_2 = (0,0.1,0)$, $v_3 = (0,0,0.1)$ — edge length $L = 0.1$ m (10 cm, canonical soft-robotics scale: compliant gripper / catheter sheath). **Round-2 change from 1 m:** 1 m unit tet was inertia-dominated by ~26× at $\Delta t = 10^{-2}$; decimeter scale makes the system stiffness-dominated by ~4×, exercising NH nonlinearity at step 1 rather than a near-linear response. |
| Element | one Tet4, $N = 4$, $G = 1$ (centroid Gauss point, reference volume $V_\text{ref} = L^3 / 6 \approx 1.67 \times 10^{-4}$ m³) |
| Material | compressible NH per [Part 2 Ch 04 NH energy sub-leaf](../../../docs/studies/soft_body_architecture/src/20-materials/04-hyperelastic/00-neo-hookean/00-energy.md): $\Psi(F) = (\mu/2)(I_1 - 3) - \mu \ln J + (\lambda/2)(\ln J)^2$; $P(F) = \mu(F - F^{-T}) + \lambda (\ln J) F^{-T}$; hardcoded $\mu = 10^5$ Pa, $\lambda = 4 \times 10^5$ Pa (Ecoflex-class, $\nu \approx 0.40$) |
| Density | $\rho = 1030 \ \text{kg/m}^3$ (silicone-class) → tet mass $\approx 0.172$ kg, per-vertex lumped $\approx 0.043$ kg |
| Boundary | Dirichlet on $v_0, v_1, v_2$ (all 9 DOF fixed); $v_3$ free |
| Load | Traction $\mathbf{t}(\theta) \in \mathbb{R}^3$ applied to $v_3$; $\theta \in \mathbb{R}^4$ parameterizes magnitude + 3-axis direction. Reasonable magnitudes: $|\mathbf{t}| \sim 10$–$30$ N for ~5–15% strain per step. |
| Initial state | Reference configuration; $\dot{\mathbf{x}}_0 = \mathbf{0}$ |
| Time step | Single backward-Euler step, $\Delta t = 10^{-2}$ s. **Inertia/stiffness balance (Round-2 verified):** lumped $M/\Delta t^2 \approx 430$ N/m/axis per free DOF; Tet4 diagonal $K$ scale $\sim \mu V L^{-2} \approx 1.67 \times 10^3$ N/m → ratio $\approx 0.26$, stiffness-dominated by ~4× — Newton sees both terms, gradcheck exercises NH nonlinearity genuinely. |
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
| Material | NH compressible (above) | Mooney-Rivlin, corotational, Ogden | NH is Phase A baseline; closed-form $P$ and $\partial P / \partial F$ per [Part 2 Ch 04 NH energy + tangent sub-leaves](../../../docs/studies/soft_body_architecture/src/20-materials/04-hyperelastic/00-neo-hookean/); no time-history to confound I-5. **Round-2 verified** book closed forms: $P = \mu(F - F^{-T}) + \lambda(\ln J)F^{-T}$; tangent $\mathbb{C}_{ijkl} = \mu \delta_{ik}\delta_{jl} + (\mu - \lambda \ln J) F^{-T}_{il} F^{-T}_{kj} + \lambda F^{-T}_{ij} F^{-T}_{kl}$; symmetric by Clairaut (hyperelastic Hessian); SPD in admissible domain ($J > 0$, moderate stretch) per Part 2 Ch 04 01-tangent.md:19–23; flattened 9×9 per-Gauss-point, expanded to 12×12 per-element block via $B^T \mathbb{C} B$ integration. |
| Element | Tet4, $N=4$, $G=1$ | Tet10, Hex8 | Tet4 is Phase A; Tet10 is Phase H per Part 3 Ch 00. |
| Contact | **None** — `NullContact` impl | IPC, penalty | IPC is Phase C; penalty is explicitly rejected in `contact/` Ch 00. `NullContact { active_pairs: [] }` proves I-1 without scope creep. Dirichlet + traction stands in for contact at 1-tet scale. |
| Integrator | Backward-Euler Newton + Armijo line-search, single step | Static equilibrium, Newton w/o line-search, strong-Wolfe | Backward-Euler is committed per Part 5 Ch 00; one step exercises the factor-on-tape pattern; Armijo is simplest compliant line-search. **Flag for I-5:** line-search selection branching is the leading determinism-risk source; test covers exactly this. |
| Linear solver | faer `Llt<I, f64>` on sparse (with index type `I` = `u32` default) | faer `Lu` as indefinite fallback; nalgebra dense (1-tet would be tiny enough) | faer `Llt<I, f64>` is the I-3 target type; sparse is forward-compatible with Phase-B 100-tet scaling. **Round-2 SPD analysis:** NH compressible tangent is SPD in admissible domain ($J > 0$, moderate stretch) per [Part 2 Ch 04 01-tangent.md lines 19–23](../../../docs/studies/soft_body_architecture/src/20-materials/04-hyperelastic/00-neo-hookean/01-tangent.md); the 1-tet traction-on-$v_3$ case pulls $v_3$ outward → $J \geq 1$ → $\ln J \geq 0$ → tangent stays SPD. Compressive loads that drive $\ln J < 0$ by enough can indefinite a mode via the $(\mu - \lambda \ln J)$ coefficient, but are not on the skeleton's parameter path. **Fallback correction (Round 2):** faer 0.24 does NOT expose LDLᵀ / Bunch-Kaufman in `sparse::linalg::solvers` — only `Llt`, `Lu`, `Qr`. Fallback is faer `Lu` (general sparse LU, 2–3× Cholesky cost per [Part 2 Ch 04 01-tangent.md:41](../../../docs/studies/soft_body_architecture/src/20-materials/04-hyperelastic/00-neo-hookean/01-tangent.md)); SPD→LDLᵀ is not an available path. **faer API pattern (Round 1 verified):** construct `SymbolicLlt` once at assembly-pattern finalization, then `Llt::try_new_with_symbolic(symbolic, mat, side)` per-step for numeric refactor; `solve_in_place_with_conj(Conj::No, rhs_matmut)` for both forward and backward solves. |
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
│   │   └── newton_vjp.rs   # NewtonStepVjp: VjpOp, stashes Llt<I, f64> + ∂r/∂θ
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
│   ├── invariant_3_factor.rs       # Llt<I, f64> ownership + solve_in_place_with_conj
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
 sim-soft  ──┬── faer          (Llt<I, f64>, sparse CSR; **greenfield — not in workspace Cargo.toml as of `77751866`**; skeleton's first commit adds it at version 0.24+)
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
- **`NewtonStepVjp: VjpOp`** — the numerical heart of the skeleton. Stashes: converged $\mathbf{x}^*$, $\theta$, `Llt<I, f64>` factor of $A = \partial r / \partial \mathbf{x} \big|_{\mathbf{x}^*, \theta}$ (owned, `Clone`), sparse $\partial r / \partial \theta \big|_{\mathbf{x}^*, \theta}$. **Sign convention confirmed** against Part 6 Ch 02:13–15: IFT gives $\partial \mathbf{x}^* / \partial \theta = -A^{-1}\, \partial r / \partial \theta$ at equilibrium where $r(\mathbf{x}^*; \theta) = 0$, with residual written as "inertia + internal − external." **VJP formula** (equivalent forms, pick one):
  - Form A (minus in the adjoint solve): $\lambda = -A^{-\mathsf{T}} \bar{\mathbf{x}}^* = -A^{-1} \bar{\mathbf{x}}^*$ (A symmetric SPD), then $\text{parent\_cotans}[\theta] \mathrel{+}= (\partial r / \partial \theta)^{\mathsf{T}} \lambda$.
  - Form B (minus outside): $\lambda = A^{-1} \bar{\mathbf{x}}^*$, then $\text{parent\_cotans}[\theta] \mathrel{+}= -(\partial r / \partial \theta)^{\mathsf{T}} \lambda$.

  Skeleton uses Form A for symmetry with the forward solve (same sign of RHS). **Book bug flagged (Pass 2/3):** Part 6 Ch 02 line 34's code sketch `factor.solve_in_place(&mut minus_dr_dtheta.apply_t(&upstream))` is dimensionally inconsistent — $(\partial r / \partial \theta)^{\mathsf{T}}$ · upstream has shape $[n_\theta]$, which cannot be a RHS for an $[n_\text{dof} \times n_\text{dof}]$ factor solve. Pseudocode, not executable. The math at lines 13–15 is correct; the skeleton implements that, not the sketch.

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
| R-1 | Full Newton + Armijo line-search, single backward-Euler step, $\Delta t = 10^{-2}$, **tet edge $L = 0.1$ m** (Round-2 change from 1 m) | Simplest compliant choice matching Part 5 Ch 00; decimeter edge gives inertia/stiffness ratio ≈ 0.26 (stiffness-dominated by ~4×), exercising NH nonlinearity at step 1. Matches canonical soft-robotics scale. | **✓ Round 2 — verified.** Dimensional analysis: per-vertex $M \approx 0.043$ kg, $M/\Delta t^2 \approx 430$, Tet4 diagonal $K \sim \mu V L^{-2} \approx 1.67 \times 10^3$. Newton convergence: inertia-dominant diagonal + NH SPD tangent → quadratic convergence expected in 3–5 iterations from zero initial guess. |
| R-2 | faer `Llt<I, f64>` on sparse CSR | Directly tests I-3; forward-compatible with Phase-B 100-tet. | **✓ Round 2 — verified.** NH tangent stays SPD on our θ-path (traction pulls $v_3$ outward, $J \geq 1$). If SPD fails, fallback is faer `Lu` (not LDLᵀ — faer 0.24 doesn't expose it). |
| R-3 | Central FD, $h = 1.5 \times 10^{-8}$ ($\approx \sqrt{\varepsilon_{f64}}$), 5-digit relative bar | Optimal for f64 precision; matches Part 11 Ch 04 §02 commitment. | Hard fail at per-component rel error > 1e-4; warn at > 1e-5. 5/6 components passing is NOT "almost working" — treat as hard fail. |
| R-4 | Minimal wgpu probe (~100–200 LOC, feature-gated) | Tests I-6b without committing to chassis-GPU (B.5 ungated). | User approved the probe; confirm scope guard (no chassis-GPU, no GPU solver, no shared tape state). |
| R-5 | `NewtonStepVjp` sign convention per residual form "inertia + internal − external" | Part 6 Ch 02:13–15 derivation; gradient sign follows. | **✓ Round 1 — verified.** Part 6 Ch 02:13–15 gives $\partial x^* / \partial \theta = -A^{-1}\, \partial r / \partial \theta$ at $r = 0$; spec §5 VJP formula is mathematically correct. Book code sketch at Ch 02:34 is pseudocode-only (see BF-3). |
| R-6 | θ = 4 scalars (traction magnitude + 3-axis direction) | Keeps FD sweep cheap; tests vector-cotangent accumulation at physics scale. | Is 4 the right count? Could narrow to 1 (magnitude only) for the very first gradcheck, widen to 4 after passing. |
| R-7 | Hand-rolled 1-tet in `mesh/single_tet.rs`, no sim-mjcf | User-confirmed. | — |
| R-8 | Crate name `sim-soft` at `sim/L0/soft/` | User-confirmed. | — |

## 11. Stress-test / review hooks (refine before coding)

Open questions the user should push on or confirm — these are where the spec is most likely wrong:

- **S-1 — `RewardBreakdown` gap on 1 tet.** Two of five fields (`pressure_uniformity`, `coverage`) are structurally undefined on a single element with no contact. Skeleton flags this as a finding. Acceptable? Or do we want the skeleton to include a 2-tet or tet-plus-rigid-wall scene to exercise all five fields and remove the gap? (Recommendation: accept the gap. Rationale: 2 tets ≈ 1.5× LOC for no additional invariant coverage; the finding itself is the point.)
- **S-2 — Determinism-under-line-search risk class.** Armijo's step-length selection branches on a sufficient-decrease inequality. If θ-perturbation changes the decrease quantity across the inequality threshold, the selected step length changes, propagating into non-smooth gradient. This would look like a gradcheck failure at 5 digits despite correct IFT. Is the skeleton gradcheck test the right place to expose this, or do we want a separate "line-search non-smoothness audit" test that holds θ fixed and varies the search-direction-norm to isolate the branch point? (Recommendation: the gradcheck test is sufficient; a failure maps to a book-level spec bug in Part 5 Ch 00 that deserves its own finding, and we don't need a dedicated test harness for it.)
- **S-3 — `faer::sparse::linalg::solvers::Llt` lifetime shape.** **✓ RESOLVED 2026-04-23 (Round 1).** faer 0.24 API verified via docs.rs. Three findings: (a) type is `Llt<I, T>` with TWO generics (index + element), not `Llt<T>`; `I` defaults to `u32`-class, `T = f64` for us; spec updated. (b) `Llt` **owns its data** — no lifetime parameter on the struct — so stashing it on a `VjpOp: Send + Sync` works cleanly. `Clone` impl available. `Send + Sync` under `T: Send+Sync` + `I: Send+Sync`, which `f64` + `u32` both satisfy. (c) Constructor is two-step: `SymbolicLlt::try_new(pattern)` once (at assembly-pattern-finalization time), then `Llt::try_new_with_symbolic(symbolic, mat, side)` per re-factor. Book Ch 02 line 26's one-call `Llt::try_new_with_symbolic(&stiffness)` sketch is incomplete; flagged as Pass 2/3 finding. Solve API is `solve_in_place_with_conj(Conj::No, rhs: MatMut<'_, T>)` via the blanket `Solve` trait — takes faer's `MatMut` view, so `NewtonStepVjp` bridges `Tensor<f64>` → `MatMut` for RHS. **Net:** factor-on-tape pattern is viable, with two API clarifications over the book's pseudocode.
- **S-4 — `parent_cotans` accumulation at physics scale.** **✓ RESOLVED 2026-04-23 (Round 1).** Read `sim/L0/ml-chassis/src/autograd.rs:769-782` (the `BackwardOp::Custom(op)` arm inside `Tape::backward`). Slots ARE zero-initialized with parent's stored shape via `Tensor::zeros(self.values[p as usize].shape())` — the tape looks up the parent's shape from the value it was pushed with, so `param_tensor` / `push_custom` / elementwise ops all get shape-correct cotangent slots. Accumulation is `ew_add_assign` (matches `+=` contract). Fan-in via duplicate parent indices works (each slot contribution accumulates independently before flush to `self.grads[p]`). Backward loop `for i in (0..=out_idx).rev()` is deterministic — no RNG, no parallelism, no clock. **Minor note:** line 750 skips nodes with all-zero cotangent — optimization, not correctness issue, but means a custom op whose upstream cotangent happens to be identically zero silently won't run its VJP. Not a skeleton concern.
- **S-5 — Cold-restart determinism via bin helper.** Is a subprocess-spawn test the right mechanism, or should the cold-restart path be exercised in a separate CI job that actually restarts the process? (Recommendation: subprocess spawn is faster and sufficient for the invariant; full CI restart isn't worth the infrastructure.)
- **S-6 — `EditResult` three-variant enum.** γ locked `ParameterOnly` / `TopologyPreserving` / `TopologyChanging` conceptually. Does the 1-tet θ-only case actually clarify the trichotomy, or does it just ever return `ParameterOnly` and leave the other two variants untested? (Answer: yes, only `ParameterOnly` fires. The other two reactivate at Phase G (SDF→mesh topology changes). Skeleton doesn't exercise them. Accept.)
- **S-7 — Nalgebra vs faer for Tet4 locals.** `Tet4::local_stiffness` returns a 12×12 dense matrix before scatter to sparse global. Using nalgebra for the local and faer for the global is two dense libraries — one extra dep. Acceptable, or collapse to faer-only dense? (Recommendation: keep nalgebra. Small 12×12 locals are faster in nalgebra's stack-allocated `SMatrix`; faer's strength is sparse. Two libraries with one-each role is cleaner than one library wearing both hats.)
- **S-8 — Feature-gate strategy for wgpu.** Should `gpu-probe` be the only wgpu entry, or should it also gate whatever Phase-E GPU code lands later? (Recommendation: `gpu-probe` is specifically the probe. Phase E adds `gpu` feature with its own gate for real GPU work. Two features. Keeps probe scope-locked.)
- **S-9 — Skeleton PR merge strategy.** One big PR at green-checklist, or staged PRs per invariant? (Recommendation: one PR, matches soft-body study PR pattern; pre-squash tag per `feedback_pre_squash_tag.md`.)
- **S-10 — Book-iteration cadence.** Every skeleton finding becomes a book-level PR (separate from the skeleton PR). Batched into one follow-up or dripped as each finding surfaces? (Recommendation: dripped. Surprise-finding at coding time produces freshest spec fix; batching loses context.)

## 12. Known unknowns with probability estimates

- ~~**80% clean, 20% refactor needed:** faer `Llt<f64>` cleanly movable onto a VJP with owned lifetime.~~ **✓ Round 1 — confirmed clean** (Llt owns data, Clone, Send+Sync under f64 + u32 index); spec updated to `Llt<I, f64>`.
- ~~**70% clean, 30% edge case:** chassis `push_custom` + `parent_cotans` accumulation works at `Tensor<f64>` shape `[12]` and `[4]`.~~ **✓ Round 1 — confirmed clean** (autograd.rs:769-782).
- **90% holds, 10% non-smooth:** Armijo line-search preserves determinism under θ-perturbation at gradcheck's $h = 1.5 \times 10^{-8}$ scale. Mitigation: the gradcheck test itself exposes this.
- ~~**95% SPD, 5% indefinite:** NH compressible tangent stays SPD at $\Delta t = 10^{-2}$ with the chosen $(\mu, \lambda, \rho)$ and traction magnitude.~~ **✓ Round 2 — updated to 99%/1%.** NH tangent is SPD in admissible domain per Part 2 Ch 04 01-tangent.md; traction pulling $v_3$ outward keeps $J \geq 1$, so we don't cross the $(\mu - \lambda \ln J)$ sign flip that would indefinite a mode. Fallback is faer `Lu` (2–3× cost), not LDLᵀ (faer 0.24 doesn't expose LDLᵀ — finding from Round-2 docs.rs check). Dense-nalgebra LDLᵀ is available for 1-tet's 9 free DOFs if both sparse paths fail, but this is deep-fallback only.
- **99% clean, 1% surprise:** wgpu build graph resolves cross-platform (macOS-arm64 + Linux CI). Mitigation: the probe test is the check.
- **99% balanced, 1% surprise:** scene numerics (decimeter tet + $\Delta t = 10^{-2}$ + silicone density) produce well-conditioned Newton and meaningfully-nonlinear gradcheck. Mitigation: R-1 dimensional analysis verified; gradcheck tolerance 5-digit is slack vs FD precision (FD with $h = 1.5 \times 10^{-8}$ is good to ~8 digits under well-scaled $f$).

If any "refactor needed" scenario fires, the skeleton PR stays open and the refactor is a prerequisite commit on the same branch.

## 13. Book findings (Pass 2/3 feedback queue)

Surprises the skeleton stress-test surfaced in the book spec. Each is a future book-iteration PR, tracked separately from the skeleton PR per §11 S-10.

| # | Where | Finding | Class |
|---|---|---|---|
| BF-1 | γ API register ([`project_soft_body_gamma_apis.md`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/project_soft_body_gamma_apis.md)) + Part 6 Ch 02:8 + Part 5 Ch 00 + Part 8 Ch 02 + Part 10 Ch 02/03 | faer sparse Cholesky type is `Llt<I, T>` (generic over index + element), not `Llt<T>`. All book references to `Llt<f64>` should read `Llt<I, f64>` (or `Llt<u32, f64>` after fixing the default index). Affects: PreferenceGP, BradleyTerryGP, GradientEnhancedGP caches + backward-Euler factor-on-tape + sparse-solvers chapter. | API-signature drift |
| BF-2 | Part 6 Ch 02:26 code sketch | `Llt::try_new_with_symbolic(&stiffness)?` is incomplete — faer 0.24's constructor is `try_new_with_symbolic(symbolic: SymbolicLlt<I>, mat: SparseColMatRef<'_, I, T>, side: Side)`, requiring a pre-computed `SymbolicLlt` + explicit `Side`. Book sketch should show the symbolic-then-numeric two-step pattern. | API-signature drift |
| BF-3 | Part 6 Ch 02:34 code sketch | `factor.solve_in_place(&mut minus_dr_dtheta.apply_t(&upstream))` is dimensionally inconsistent — $(\partial r / \partial \theta)^{\mathsf{T}}$ · upstream has shape $[n_\theta]$, cannot be RHS for $[n_\text{dof} \times n_\text{dof}]$ solve. Sketch's order of operations is inverted. Correct sequence: (1) λ = A^{-1} · upstream via `solve_in_place`, (2) result = ±(∂r/∂θ)^T · λ. The math at lines 13-15 is fine; only the code sketch needs fixing. | Pseudocode correctness |

None of the three findings are skeleton-blocking — the skeleton implements from the underlying math + the verified faer API. They're book-edit backlog for whenever the PM memos or Pass 2/3 touch these chapters.

---

## Appendix: γ-locked API name registry (skeleton must cite verbatim)

From [`project_soft_body_gamma_apis.md`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/project_soft_body_gamma_apis.md):

- `ForwardMap` (trait) — `evaluate(theta, tape) → (RewardBreakdown, EditResult)` + `gradient(theta, tape) → (Tensor<f64>, GradientEstimate)`
- `RewardBreakdown` (struct) — per-term reward struct with `apply_residuals` method
- `EditResult` (enum) — 3 variants
- `RewardWeights` (struct) — scalar-composition weights
- `GradientEstimate` (enum) — `Exact | Noisy { variance: f64 }`
- `Material`, `Element<const N, const G>`, `Mesh`, `ContactModel`, `Solver`, `Differentiable`, `Observable` (with `type Step`) — the seven traits
- `faer::sparse::linalg::solvers::Llt<I, f64>` — Cholesky factor type (factor-on-tape). Round-1 correction: faer 0.24 uses two type parameters `<I, T>`; `I` is the index type (u32 default), `T` is the element type (f64 for sim-soft). `SymbolicLlt<I>` is the prerequisite symbolic factor; construct once per assembly-pattern lifetime. `solve_in_place_with_conj(Conj::No, rhs: MatMut<'_, T>)` is the solve entry point.
- `VjpOp` (trait), `Tape::push_custom` (method) — chassis extension point (shipped in PR #213)
- `Tensor<f64>` — sim-soft precision (chassis default is f32 for RL)

Not cited in skeleton (post-skeleton API register for reference): `SimToRealCorrection`, `MeasurementReport`, `OnlineUpdateOutcome`, `PreferenceGP`, `BradleyTerryGP`, `GradientEnhancedGP`, `Acquisition` enum, `CostAwareAcquisition`, `DuelingBanditPolicy`, `MeasurementProtocol`, `DiffusionProfile`, `MjcfHandshake`, `ThermostatHandshake`, `MeshSnapshot`, `SimSoftPlugin`, `SdfField`, `MaterialField`.
