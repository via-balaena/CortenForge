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

**Round 3 stress-test results (2026-04-23):** I-1 trait composition paper-sketched against [Part 11 Ch 01 00-core.md](../../../docs/studies/soft_body_architecture/src/110-crate/01-traits/00-core.md) + [01-composition.md](../../../docs/studies/soft_body_architecture/src/110-crate/01-traits/01-composition.md). **Seven trait surfaces compose cleanly** — ≤4 lines of `where` clause per impl, no generic pile-up, object-safety holds for `Solver` / `Observable` in `Box<dyn … Tape = CpuTape>` form. Full skeleton type alias: `CpuNewtonSolver<NeoHookean, Tet4, SingleTetMesh, NullContact, 4, 1>` — 6 generic parameters (4 type + 2 const), verbose but contained via one alias. **One real book finding (BF-4):** book's `Differentiable::register_vjp(forward_key: TapeNodeKey, vjp: Box<dyn VjpOp>)` assumes a key-based registry that doesn't match chassis's `Tape::push_custom(value, op)` model — `TapeNodeKey` isn't defined in chassis. Skeleton stubs `register_vjp` and creates VJPs at `push_custom` time instead. **Three skeleton implementation notes** (expected stub-on-trait-contract, not book bugs): `Solver::replay_step` is Phase-E checkpoint-replay infra (skeleton delegates to step without tape writes), `Differentiable::time_adjoint` + `fd_wrapper` are Phase E+/G respectively (`unimplemented!()`), `Observable::temperature_field` returns empty on skeleton's non-thermal scene. See §14 composition sketch + §13 BF-4.

**Round 4 stress-test results (2026-04-23):** book citation audit. **All 11 unique book-link paths resolve** — no dead links. Claim numbers cross-verified: Part 5 Ch 00 Claim 3 (`## 3. The factorization is a captured first-class object`), Part 6 Ch 00 Claim 3 ("Own every line" prose claim 3), Part 2 Ch 00 §00 4-item Material signature, Part 6 Ch 02:13–15 IFT derivation, Part 2 Ch 04 01-tangent.md:19–23 SPD + :41 LU fallback, Part 11 Ch 01 parent claims 1+2 — all present and correct. **Two spec-drift fixes** (not book bugs): (a) `RewardBreakdown` has **4 per-term fields**, not 5 per Part 10 Ch 00:47 — `composition` was described as a field but is actually a method call (`score_with(&weights)`); spec §2 + §5 + appendix corrected, 1-tet gap recounted from "two of five" to "two of four." (b) Line 218 "Part 8 Ch 04 Claim 3" was wrong — Part 8 Ch 04 cites "Part 6 Ch 00 Claim 3" as the upstream source; corrected to name both chapters with Part 6 Ch 00 as claim home. No new BF — the book is internally consistent; these were spec imprecision.

**Round 5 stress-test results (2026-04-23):** module layout + dep-graph FFI-leak audit. **S-7 nalgebra/faer division is mandatory, not preference** — the seven-trait surface in Part 11 Ch 01 uses nalgebra types (`Matrix3<f64>`, `SMatrix<f64, 9, 9>`, `SVector<f64, N>`) in `Material::tangent`, `Element::shape_functions`, etc.; faer can't express those. Sparse globals + `Llt<I, f64>` can't be expressed in nalgebra. The division is forced by trait signatures. **S-8 feature gate pattern:** sim-gpu makes wgpu a required dep (it IS the GPU crate); sim-soft's wgpu is optional/probe-only, so `gpu-probe` uses `dep:wgpu` + `optional = true` pattern. Phase-E GPU work lands as a separate `gpu` feature — `gpu-probe` is scope-locked to the build-graph round-trip test. **FFI-leak audit passes:** zero C/C++ source files in skeleton's transitive dep graph — `nalgebra 0.34.2` (already workspace, pure Rust, BLAS bindings off), `faer 0.24` default features all pure Rust (`private-gemm-x86` is Rust+intrinsics not C-BLAS FFI), `sim-ml-chassis` pure Rust, `wgpu` uses system Metal/Vulkan backends (no cargo-compiled C). I-6a holds. Concrete `Cargo.toml` sketch now in §4.

**Round 6 stress-test results (2026-04-23):** test harness realism. **S-5 subprocess cold-restart sufficient** (fresh heap + ASLR + thread-pool re-init cover what the skeleton needs to prove; cross-machine determinism is out of scope). **R-6 θ two-stage:** Stage 1 = 1 scalar (magnitude along +ẑ) for first-green, Stage 2 = 3 (full force vector) after Stage 1 passes; the Round-1 4-component polar parameterization was overcomplicated. **Gradcheck precision audit:** NH conditioning is tame (cond ~ 6 on 3-DOF tangent at our parameters), FD at $h = 1.5 \times 10^{-8}$ gives ~8 digits, 5-digit bar has ~3 digits of slack. Newton convergence tol set to 1e-10 to keep Newton output precision 5 digits below the gradcheck bar. **One structural I-5 risk caught and mitigated:** faer's default `rayon` feature work-steals floating-point reductions non-deterministically — **Cargo.toml sketch in §4 now sets `default-features = false, features = ["std", "sparse-linalg"]`** to keep skeleton single-threaded. For the 9-DOF system rayon gives no speedup anyway. **New §15** enumerates all 11 I-5 determinism risk classes with per-risk mitigation.

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
| Load | Traction $\mathbf{t}(\theta) \in \mathbb{R}^3$ applied to $v_3$. **Two-stage θ parameterization (Round-6 change from 4 components):** Stage 1 — $\theta \in \mathbb{R}^1$ = traction magnitude along hardcoded $+\hat z$, used for first gradcheck pass (minimal FD surface, 2 evaluations); Stage 2 — $\theta \in \mathbb{R}^3$ = full traction vector $(t_x, t_y, t_z)$, widened once Stage 1 passes. Reasonable magnitudes: $|\mathbf{t}| \sim 10$–$30$ N for ~5–15% strain per step. |
| Initial state | Reference configuration; $\dot{\mathbf{x}}_0 = \mathbf{0}$ |
| Time step | Single backward-Euler step, $\Delta t = 10^{-2}$ s. **Inertia/stiffness balance (Round-2 verified):** lumped $M/\Delta t^2 \approx 430$ N/m/axis per free DOF; Tet4 diagonal $K$ scale $\sim \mu V L^{-2} \approx 1.67 \times 10^3$ N/m → ratio $\approx 0.26$, stiffness-dominated by ~4× — Newton sees both terms, gradcheck exercises NH nonlinearity genuinely. |
| System size | 12 DOF, 3 free after Dirichlet (only $v_3$ moves) |

**Reward scalar** — minimal `RewardBreakdown` for a 1-tet case. Per [Part 10 Ch 00 00-forward.md:47](../../../docs/studies/soft_body_architecture/src/100-optimization/00-forward.md) the struct carries **four** per-term contributions (not five); scalar composition is via the `score_with(&weights: &RewardWeights) -> f64` method, not an additional field.

| Per-term field | 1-tet definition | Phase-A meaningful? |
|---|---|---|
| `pressure_uniformity` | not meaningful on 1 tet — set to NaN sentinel or `None` if Option | **gap** |
| `coverage` | not meaningful — no contact pair | **gap** |
| `peak_bound` | $\max$ principal Cauchy stress at the Gauss point, barrier-compared to $\sigma_{\max} = 10^6$ Pa | yes |
| `stiffness_bound` | $\mu$ itself, barrier-compared to $k_{\min} = 5 \times 10^4$ | yes |

Scalar composition: `breakdown.score_with(&RewardWeights { w_pu, w_cov, w_peak, w_stiff })` per [Part 1 Ch 01 composition rule](../../../docs/studies/soft_body_architecture/src/10-physical/01-reward.md). Skeleton sets weights with zeros on the gap fields so composed scalar only sees `peak_bound + stiffness_bound`.

**Finding if this plays out:** two of four per-term fields are 1-tet-indeterminate. Expected. Feed back to Part 1 Ch 01: either (a) document that single-element scenes under-specify `RewardBreakdown` (spec clarification), or (b) add a 1-tet-safe reduction for those fields (spec change). Not a skeleton failure — a skeleton finding.

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
| Integrator | Backward-Euler Newton + Armijo line-search, single step, **convergence tol = 1e-10** | Static equilibrium, Newton w/o line-search, strong-Wolfe | Backward-Euler is committed per Part 5 Ch 00; one step exercises the factor-on-tape pattern; Armijo is simplest compliant line-search. **Round-6 Newton tol set to 1e-10** — 5 digits below gradcheck's 1e-5 bar so FD precision isn't limited by Newton's own output precision. **Flag for I-5:** line-search selection branching is the leading determinism-risk source; test covers exactly this. |
| Linear solver | faer `Llt<I, f64>` on sparse (with index type `I` = `u32` default) | faer `Lu` as indefinite fallback; nalgebra dense (1-tet would be tiny enough) | faer `Llt<I, f64>` is the I-3 target type; sparse is forward-compatible with Phase-B 100-tet scaling. **Round-2 SPD analysis:** NH compressible tangent is SPD in admissible domain ($J > 0$, moderate stretch) per [Part 2 Ch 04 01-tangent.md lines 19–23](../../../docs/studies/soft_body_architecture/src/20-materials/04-hyperelastic/00-neo-hookean/01-tangent.md); the 1-tet traction-on-$v_3$ case pulls $v_3$ outward → $J \geq 1$ → $\ln J \geq 0$ → tangent stays SPD. Compressive loads that drive $\ln J < 0$ by enough can indefinite a mode via the $(\mu - \lambda \ln J)$ coefficient, but are not on the skeleton's parameter path. **Fallback correction (Round 2):** faer 0.24 does NOT expose LDLᵀ / Bunch-Kaufman in `sparse::linalg::solvers` — only `Llt`, `Lu`, `Qr`. Fallback is faer `Lu` (general sparse LU, 2–3× Cholesky cost per [Part 2 Ch 04 01-tangent.md:41](../../../docs/studies/soft_body_architecture/src/20-materials/04-hyperelastic/00-neo-hookean/01-tangent.md)); SPD→LDLᵀ is not an available path. **faer API pattern (Round 1 verified):** construct `SymbolicLlt` once at assembly-pattern finalization, then `Llt::try_new_with_symbolic(symbolic, mat, side)` per-step for numeric refactor; `solve_in_place_with_conj(Conj::No, rhs_matmut)` for both forward and backward solves. |
| Autograd tape | chassis CPU tape + `VjpOp` + `Tape::push_custom` | `Burn`, `dfdx`, `tch-rs` | "Own every line" thesis. Refactor A.1/B.1 shipped the surface. |
| Gradcheck | §01 determinism + §02 FD-vs-IFT at 5 digits, central FD, $h = 1.5 \times 10^{-8}$ | §03 CPU-vs-GPU, forward FD, one-sided | §03 is Phase E (skeleton is CPU-only). Central FD is optimal at 5-digit bar. |
| GPU probe | minimal wgpu round-trip, feature-gated `gpu-probe` | full chassis-GPU path, no probe at all | Chassis-GPU is B.5-gated (not shipped). Minimal probe tests (6b) without committing to B.5 scope. |
| Mesh fixture | hand-rolled 1-tet (4 vertices hardcoded in `Mesh` impl) | sim-mjcf ingest, MJCF file | Skeleton stays self-contained; sim-mjcf coupling is Phase F. |

## 4. Crate and module layout

Location: `sim/L0/soft/` — sibling of `{types, simd, core, thermostat, gpu, mjcf, urdf, tests, ml-chassis, rl, opt}`.

**Cargo.toml sketch (Round-5 verified):**

```toml
[package]
name = "sim-soft"
version.workspace = true
edition.workspace = true
# ... other workspace inheritances ...

[features]
default = []
# I-6b probe: opt-in wgpu round-trip. NOT a Phase-E GPU feature flag.
# (Phase-E will add a separate `gpu` feature for real GPU compute.)
gpu-probe = ["dep:wgpu", "dep:bytemuck", "dep:pollster"]

[dependencies]
# Chassis tape, VjpOp, Tensor<f64> — shipped in PR #213
sim-ml-chassis = { workspace = true }
# Dense small matrices for per-Gauss-point Material + per-element Tet4 locals
# (5 existing L0 consumers at 0.34.2 — not greenfield)
nalgebra = { workspace = true }
# Sparse SPD factor-on-tape (Llt<I, f64>) + sparse CSR assembly.
# Greenfield — this is the first workspace crate to depend on faer.
# Round-6 determinism: disable `rayon` to keep solves single-threaded
# (rayon work-stealing reorders floating-point reductions bit-non-
# deterministically, violating γ Ch 00 determinism-in-θ on I-5). For
# skeleton's 9-DOF system, rayon would provide no speedup anyway.
# `std` + `sparse-linalg` is the minimum that exposes Llt<I, f64> +
# solve_in_place_with_conj. All features remain pure Rust.
faer = { version = "0.24", default-features = false, features = ["std", "sparse-linalg"] }

# GPU probe deps — feature-gated, skeleton runs fine without them
wgpu      = { workspace = true, optional = true }
bytemuck  = { workspace = true, optional = true }
pollster  = { workspace = true, optional = true }

[dev-dependencies]
approx = { workspace = true }  # 5-digit gradcheck tolerance assertions

[lints]
workspace = true
```

```
sim/L0/soft/
├── Cargo.toml              # as above
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
 sim-soft  ──┬── faer 0.24         (Llt<I, f64>, sparse CSR; **greenfield — skeleton's first commit adds it to workspace Cargo.toml**; default features all pure Rust)
             ├── nalgebra 0.34.2   (Matrix3, SMatrix<9,9>, SVector<N>; already workspace, 5 existing L0 consumers)
             └── wgpu (opt)        (probe, feature-gated behind `gpu-probe`)
```

**Round-5 FFI-leak audit:** the skeleton compiles with **zero C/C++ source files** in its transitive dep graph.
- `nalgebra 0.34.2` — pure Rust; optional BLAS bindings are off by default.
- `faer 0.24` default features (`std`, `rayon`, `sparse-linalg`, `rand`, `npy`) — all pure Rust; `std` pulls `private-gemm-x86` which is Rust + platform intrinsics (not C-BLAS FFI); `sparse::linalg::solvers::Llt` is the Phase-B target type per Part 11 Ch 03 build-order.
- `sim-ml-chassis` — pure Rust (verified Round 1; no external crates beyond num-traits).
- `wgpu` (only when `gpu-probe` feature on) — system-library backends (Metal on macOS, Vulkan on Linux); no C source compiled by cargo.

I-6a CPU build graph commitment holds.

Absent from skeleton (deferred by phase): `sim-core` (F), `sim-mjcf` (F), `sim-thermostat` (F/G), `sim-bevy` (I), `cf-design` (G), `sim-opt` (post-I).

**LOC budget:** ~800 production, ~400 tests, target total ~1200. Room at the top end (1500) for unavoidable growth in `NewtonStepVjp` + the Tet4 local-assembly routine if the closed-form NH derivatives expand beyond estimate.

## 5. Proposed new API — skeleton must define

Names locked by γ ([`project_soft_body_gamma_apis.md`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/project_soft_body_gamma_apis.md)) but not yet in Rust. Skeleton writes these — first contact with code is expected to surface refinements feeding back into Part 10 Ch 00.

- **`RewardBreakdown`** (γ) — struct with **four** per-term fields per [Part 10 Ch 00 00-forward.md:47](../../../docs/studies/soft_body_architecture/src/100-optimization/00-forward.md) + [Part 1 Ch 01](../../../docs/studies/soft_body_architecture/src/10-physical/01-reward.md): `pressure_uniformity`, `coverage`, `peak_bound`, `stiffness_bound`. 1-tet values: see §2 reward table. Two methods: `score_with(&weights: &RewardWeights) -> f64` for scalar composition (used by downstream optimizers consuming scalar reward); `apply_residuals(&self, residuals: &ResidualCorrections) -> Self` per δ Ch 00 readout §3 (consumed by `SimToRealCorrection::correct`, post-Phase-I).
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
| I-2 | `invariant_2_forward.rs` | integration | 1-tet scene, θ = Stage-2 `[t_x, t_y, t_z]`, call `evaluate`; assert `peak_bound`/`stiffness_bound` finite + `pressure_uniformity`/`coverage` NaN-sentinel per 1-tet gap; scalar compose via `score_with(&weights)` with zeros on gap fields; assert `EditResult::ParameterOnly` | < 2 s |
| I-3 | `invariant_3_factor.rs` | unit-style integration | Assemble 12×12 synthetic SPD matrix, `Llt::new`, stash in `NewtonStepVjp`, drop source matrix, `solve_in_place` on two different RHSes, assert consistency | < 1 s |
| I-4 + I-5 | `invariant_4_5_gradcheck.rs` | integration | Central FD with $h = 1.5 \times 10^{-8}$ vs IFT gradient; **Stage 1: θ ∈ ℝ¹ (magnitude-only, 2 FD evals) for first green**, then **Stage 2: θ ∈ ℝ³ (full force vector, 6 FD evals)**; relative error per component ≤ 1e-5 | < 10 s |
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
| Chassis-GPU (`sim_ml_chassis::gpu`, B.5 ungated) | E | Refactor explicitly deferred; [Part 6 Ch 00 Claim 3 "Own every line"](../../../docs/studies/soft_body_architecture/src/60-differentiability/00-what-autograd-needs.md) commits the extension; [Part 8 Ch 04](../../../docs/studies/soft_body_architecture/src/80-gpu/04-chassis-extension.md) names the concrete shape as Phase-E prerequisite. |
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
| R-6 | θ two-stage: Stage-1 = 1 scalar (magnitude along +ẑ), Stage-2 = 3 (full force vector) | Stage 1 is minimal FD surface (2 evaluations) for first-green gradient confirmation. Stage 2 widens to test vector-cotangent accumulation on all 3 axes. | **✓ Round 6 — confirmed 2-stage is right.** 4-component polar-like parameterization was overcomplicated (singularity at $\|\mathbf{t}\|=0$, unit-vector constraint, FD awkwardness). Raw 3-force-vector is simpler and tests what actually matters. |
| R-7 | Hand-rolled 1-tet in `mesh/single_tet.rs`, no sim-mjcf | User-confirmed. | — |
| R-8 | Crate name `sim-soft` at `sim/L0/soft/` | User-confirmed. | — |

## 11. Stress-test / review hooks (refine before coding)

Open questions the user should push on or confirm — these are where the spec is most likely wrong:

- **S-1 — `RewardBreakdown` gap on 1 tet.** Two of five fields (`pressure_uniformity`, `coverage`) are structurally undefined on a single element with no contact. Skeleton flags this as a finding. Acceptable? Or do we want the skeleton to include a 2-tet or tet-plus-rigid-wall scene to exercise all five fields and remove the gap? (Recommendation: accept the gap. Rationale: 2 tets ≈ 1.5× LOC for no additional invariant coverage; the finding itself is the point.)
- **S-2 — Determinism-under-line-search risk class.** Armijo's step-length selection branches on a sufficient-decrease inequality. If θ-perturbation changes the decrease quantity across the inequality threshold, the selected step length changes, propagating into non-smooth gradient. This would look like a gradcheck failure at 5 digits despite correct IFT. Is the skeleton gradcheck test the right place to expose this, or do we want a separate "line-search non-smoothness audit" test that holds θ fixed and varies the search-direction-norm to isolate the branch point? (Recommendation: the gradcheck test is sufficient; a failure maps to a book-level spec bug in Part 5 Ch 00 that deserves its own finding, and we don't need a dedicated test harness for it.)
- **S-3 — `faer::sparse::linalg::solvers::Llt` lifetime shape.** **✓ RESOLVED 2026-04-23 (Round 1).** faer 0.24 API verified via docs.rs. Three findings: (a) type is `Llt<I, T>` with TWO generics (index + element), not `Llt<T>`; `I` defaults to `u32`-class, `T = f64` for us; spec updated. (b) `Llt` **owns its data** — no lifetime parameter on the struct — so stashing it on a `VjpOp: Send + Sync` works cleanly. `Clone` impl available. `Send + Sync` under `T: Send+Sync` + `I: Send+Sync`, which `f64` + `u32` both satisfy. (c) Constructor is two-step: `SymbolicLlt::try_new(pattern)` once (at assembly-pattern-finalization time), then `Llt::try_new_with_symbolic(symbolic, mat, side)` per re-factor. Book Ch 02 line 26's one-call `Llt::try_new_with_symbolic(&stiffness)` sketch is incomplete; flagged as Pass 2/3 finding. Solve API is `solve_in_place_with_conj(Conj::No, rhs: MatMut<'_, T>)` via the blanket `Solve` trait — takes faer's `MatMut` view, so `NewtonStepVjp` bridges `Tensor<f64>` → `MatMut` for RHS. **Net:** factor-on-tape pattern is viable, with two API clarifications over the book's pseudocode.
- **S-4 — `parent_cotans` accumulation at physics scale.** **✓ RESOLVED 2026-04-23 (Round 1).** Read `sim/L0/ml-chassis/src/autograd.rs:769-782` (the `BackwardOp::Custom(op)` arm inside `Tape::backward`). Slots ARE zero-initialized with parent's stored shape via `Tensor::zeros(self.values[p as usize].shape())` — the tape looks up the parent's shape from the value it was pushed with, so `param_tensor` / `push_custom` / elementwise ops all get shape-correct cotangent slots. Accumulation is `ew_add_assign` (matches `+=` contract). Fan-in via duplicate parent indices works (each slot contribution accumulates independently before flush to `self.grads[p]`). Backward loop `for i in (0..=out_idx).rev()` is deterministic — no RNG, no parallelism, no clock. **Minor note:** line 750 skips nodes with all-zero cotangent — optimization, not correctness issue, but means a custom op whose upstream cotangent happens to be identically zero silently won't run its VJP. Not a skeleton concern.
- **S-5 — Cold-restart determinism via bin helper.** **✓ Round 6 — subprocess approach confirmed sufficient.** Spawning `sim-soft-skeleton-run` as a child process tests: fresh heap, new process memory, OS ASLR re-randomization, fresh thread-pool init. What it doesn't test — cross-machine determinism (different CPU → different SIMD kernel dispatch in `private-gemm-x86`) — is outside skeleton scope; that's a separate Phase-B concern if 100-tet determinism needs to hold across dev-machine/CI. Full reboot-level restart would test nothing the subprocess doesn't.
- **S-6 — `EditResult` three-variant enum.** γ locked `ParameterOnly` / `TopologyPreserving` / `TopologyChanging` conceptually. Does the 1-tet θ-only case actually clarify the trichotomy, or does it just ever return `ParameterOnly` and leave the other two variants untested? (Answer: yes, only `ParameterOnly` fires. The other two reactivate at Phase G (SDF→mesh topology changes). Skeleton doesn't exercise them. Accept.)
- **S-7 — Nalgebra vs faer for Tet4 locals.** **✓ Round 5 — verified mandatory, not preference.** Trait signatures in Part 11 Ch 01 00-core.md pin nalgebra types (`Matrix3<f64>`, `SMatrix<f64, 9, 9>`, `SMatrix<f64, N, 3>`, `SVector<f64, N>`) — `Material::tangent`, `Element::shape_functions`, `Element::shape_gradients`. Faer can't express those. Sparse globals + `Llt<I, f64>` can't be expressed in nalgebra. Collapsing is impossible without editing the book's trait surface.
- **S-8 — Feature-gate strategy for wgpu.** **✓ Round 5 — confirmed two-feature plan.** Sim-gpu makes wgpu a required dep (crate IS the GPU); sim-soft's wgpu is optional via `gpu-probe` feature (`dep:wgpu` + `optional = true`). Phase-E real-GPU work lands as a separate `gpu` feature with its own dep set. Two features, scope-locked. Concrete Cargo.toml sketch in §4.
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

## 13. Book findings (Pass 2/3 feedback queue) and scaffold deferrals

Surprises the skeleton stress-test surfaced in the book spec (BF-N) plus scaffold-phase deferrals (SD-N) where the scaffold commit intentionally stopped short of §4's final shape. Book findings become future book-iteration PRs tracked separately from the skeleton PR per §11 S-10; scaffold deferrals close inside the skeleton PR sequence at the step named in the entry.

| # | Where | Finding | Class |
|---|---|---|---|
| BF-1 | γ API register ([`project_soft_body_gamma_apis.md`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/project_soft_body_gamma_apis.md)) + Part 6 Ch 02:8 + Part 5 Ch 00 + Part 8 Ch 02 + Part 10 Ch 02/03 | faer sparse Cholesky type is `Llt<I, T>` (generic over index + element), not `Llt<T>`. All book references to `Llt<f64>` should read `Llt<I, f64>` (or `Llt<u32, f64>` after fixing the default index). Affects: PreferenceGP, BradleyTerryGP, GradientEnhancedGP caches + backward-Euler factor-on-tape + sparse-solvers chapter. | API-signature drift |
| BF-2 | Part 6 Ch 02:26 code sketch | `Llt::try_new_with_symbolic(&stiffness)?` is incomplete — faer 0.24's constructor is `try_new_with_symbolic(symbolic: SymbolicLlt<I>, mat: SparseColMatRef<'_, I, T>, side: Side)`, requiring a pre-computed `SymbolicLlt` + explicit `Side`. Book sketch should show the symbolic-then-numeric two-step pattern. | API-signature drift |
| BF-3 | Part 6 Ch 02:34 code sketch | `factor.solve_in_place(&mut minus_dr_dtheta.apply_t(&upstream))` is dimensionally inconsistent — $(\partial r / \partial \theta)^{\mathsf{T}}$ · upstream has shape $[n_\theta]$, cannot be RHS for $[n_\text{dof} \times n_\text{dof}]$ solve. Sketch's order of operations is inverted. Correct sequence: (1) λ = A^{-1} · upstream via `solve_in_place`, (2) result = ±(∂r/∂θ)^T · λ. The math at lines 13-15 is fine; only the code sketch needs fixing. | Pseudocode correctness |
| BF-4 | Part 11 Ch 01 00-core.md:121 + 01-composition.md:66 | `Differentiable::register_vjp(forward_key: TapeNodeKey, vjp: Box<dyn VjpOp>)` assumes a key-indexed VJP registry that doesn't match the chassis API shipped in PR #213. Chassis's model is `Tape::push_custom(value: Tensor<f64>, op: Box<dyn VjpOp>) -> Var` — VJP is bundled with the node at forward-pass time, not registered-by-key-then-looked-up. No `TapeNodeKey` type exists in `sim-ml-chassis`. Also, register_vjp taking an `instance` (Box<dyn VjpOp>) rather than a factory/closure makes per-call primal-data capture awkward. **Fix options:** (a) drop `register_vjp` from the trait; Material/Element/ContactModel methods directly return configured `Box<dyn VjpOp>` instances that Solver::step passes to push_custom; (b) keep register_vjp but make it a factory registration with signature like `register_vjp(key: OpClassId, factory: Box<dyn Fn(&PrimalData) -> Box<dyn VjpOp>>)`. Skeleton picks (a) de facto by stubbing register_vjp and creating VJPs inline. | Trait-signature / platform-API mismatch |
| BF-5 | Part 2 Ch 04 01-tangent.md + 03-impl.md:57 | 9×9 tangent flattening convention is underspecified. 01-tangent.md asserts the tangent is "flattened 9×9 symmetric" but does not state the `(i,j,k,l) → (row,col)` index mapping. 03-impl.md:57 pseudocode comment says "row-major into a 9x9," which is ambiguous between F-flattening (contradicting scope §14's "column-major F-flattening per spec") and 9×9 storage layout (contradicting nalgebra's column-major `SMatrix`). Skeleton commits the scope-§14 convention `row = i + 3j, col = k + 3l` (column-major F-flattening, nalgebra-native 9×9 storage) in `sim-soft`'s `Material` trait docstring + `NeoHookean::tangent`, verified by the FD-vs-analytic test at `h = 1.5e-8`. Book-edit pass should state the index mapping explicitly in 01-tangent.md §"Per-element block sizes" and rewrite the 03-impl.md:57 comment. | Pseudocode / spec-gap |
| SD-1 | `sim/L0/soft/Cargo.toml` | Scaffold Cargo.toml omits §4's `[features]` table (`default = []`, `gpu-probe = ["dep:wgpu", "dep:bytemuck", "dep:pollster"]`) and the three optional deps. Closes with §9 step 7's `tests/invariant_6_gpu_probe.rs` commit, where building the feature without a probe to gate would be meaningless infrastructure. Green-checklist item 2 (`cargo build --features gpu-probe`) unreachable until then — non-blocking for steps 3–6 which are CPU-only. | Scaffold deferral |

BF-1..BF-5 are not skeleton-blocking — the skeleton implements from the underlying math + the verified faer/chassis APIs, stubbing book methods whose shape doesn't match shipped infrastructure. They're book-edit backlog for whenever Pass 2/3 touches these chapters. SD-1 closes inside the skeleton PR sequence.

---

## Appendix: γ-locked API name registry (skeleton must cite verbatim)

From [`project_soft_body_gamma_apis.md`](../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/project_soft_body_gamma_apis.md):

- `ForwardMap` (trait) — `evaluate(theta, tape) → (RewardBreakdown, EditResult)` + `gradient(theta, tape) → (Tensor<f64>, GradientEstimate)`
- `RewardBreakdown` (struct) — per-term reward struct with 4 fields (`pressure_uniformity`, `coverage`, `peak_bound`, `stiffness_bound`) + two methods: `score_with(&weights) -> f64` (scalar composition per Part 1 Ch 01 composition rule) + `apply_residuals(&residuals) -> Self` (sim-to-real correction, post-Phase-I)
- `EditResult` (enum) — 3 variants
- `RewardWeights` (struct) — scalar-composition weights
- `GradientEstimate` (enum) — `Exact | Noisy { variance: f64 }`
- `Material`, `Element<const N, const G>`, `Mesh`, `ContactModel`, `Solver`, `Differentiable`, `Observable` (with `type Step`) — the seven traits
- `faer::sparse::linalg::solvers::Llt<I, f64>` — Cholesky factor type (factor-on-tape). Round-1 correction: faer 0.24 uses two type parameters `<I, T>`; `I` is the index type (u32 default), `T` is the element type (f64 for sim-soft). `SymbolicLlt<I>` is the prerequisite symbolic factor; construct once per assembly-pattern lifetime. `solve_in_place_with_conj(Conj::No, rhs: MatMut<'_, T>)` is the solve entry point.
- `VjpOp` (trait), `Tape::push_custom` (method) — chassis extension point (shipped in PR #213)
- `Tensor<f64>` — sim-soft precision (chassis default is f32 for RL)

Not cited in skeleton (post-skeleton API register for reference): `SimToRealCorrection`, `MeasurementReport`, `OnlineUpdateOutcome`, `PreferenceGP`, `BradleyTerryGP`, `GradientEnhancedGP`, `Acquisition` enum, `CostAwareAcquisition`, `DuelingBanditPolicy`, `MeasurementProtocol`, `DiffusionProfile`, `MjcfHandshake`, `ThermostatHandshake`, `MeshSnapshot`, `SimSoftPlugin`, `SdfField`, `MaterialField`.

## 14. Trait composition sketch (Round 3)

Full paper-sketch of the skeleton's seven-trait composition. Goal: one monomorphized forward path + one `Box<dyn Solver<Tape = CpuTape>>` on the public boundary per Part 11 Ch 01 + 01-composition.md. All book signatures preserved except `Differentiable::register_vjp` (BF-4 — stubbed).

**Seven concrete types:**

```rust
// sim-soft/src/material/neo_hookean.rs
pub struct NeoHookean { pub mu: f64, pub lambda: f64 }
impl Material for NeoHookean {
    fn energy(&self, f: &Matrix3<f64>) -> f64 { /* Part 2 Ch 04 00-energy */ }
    fn first_piola(&self, f: &Matrix3<f64>) -> Matrix3<f64> { /* μ(F - F⁻ᵀ) + λ(ln J) F⁻ᵀ */ }
    fn tangent(&self, f: &Matrix3<f64>) -> SMatrix<f64, 9, 9> { /* Part 2 Ch 04 01-tangent */ }
    fn validity(&self) -> ValidityDomain { /* J > 0, moderate stretch */ }
}

// sim-soft/src/element/tet4.rs
pub struct Tet4;
impl Element<4, 1> for Tet4 {
    fn shape_functions(&self, xi: Vec3) -> SVector<f64, 4> { /* barycentric */ }
    fn shape_gradients(&self, xi: Vec3) -> SMatrix<f64, 4, 3> { /* constant */ }
    fn gauss_points(&self) -> [(Vec3, f64); 1] { [(Vec3::new(0.25, 0.25, 0.25), 1.0/6.0)] }
}

// sim-soft/src/mesh/single_tet.rs
pub struct SingleTetMesh {
    vertices: [Vec3; 4],    // hardcoded in SoftScene::one_tet_cube()
    adj: MeshAdjacency,     // trivial: single tet, no neighbours
    q: QualityMetrics,      // trivial: aspect ratio 1.0, etc.
}
impl Mesh for SingleTetMesh { /* 7 methods, mostly trivial on single-element mesh */ }

// sim-soft/src/contact/null.rs
pub struct NullContact;
impl ContactModel for NullContact {
    fn active_pairs(&self, _: &dyn Mesh, _: &[Vec3]) -> Vec<ContactPair> { vec![] }
    fn energy(&self, _: &ContactPair, _: &[Vec3]) -> f64 { 0.0 }
    fn gradient(&self, _: &ContactPair, _: &[Vec3]) -> ContactGradient { ContactGradient::zero() }
    fn hessian(&self, _: &ContactPair, _: &[Vec3]) -> ContactHessian { ContactHessian::zero() }
    fn ccd_toi(&self, _: &ContactPair, _: &[Vec3], _: &[Vec3]) -> f64 { f64::INFINITY }
}

// sim-soft/src/solver/backward_euler.rs
pub struct CpuNewtonSolver<M, E, Msh, C, const N: usize, const G: usize>
where
    M: Material, E: Element<N, G>, Msh: Mesh, C: ContactModel,
{
    material: M, element: E, mesh: Msh, contact: C,
    config: SolverConfig,
    // scratch: F⁻ᵀ cache per Gauss point, sparse pattern, SymbolicLlt
}

impl<M, E, Msh, C, const N: usize, const G: usize> Solver
    for CpuNewtonSolver<M, E, Msh, C, N, G>
where
    M: Material, E: Element<N, G>, Msh: Mesh, C: ContactModel,
{
    type Tape = CpuTape;
    fn step(&mut self, tape: &mut Self::Tape, x_prev: &Tensor<f64>, v_prev: &Tensor<f64>,
            theta: &Tensor<f64>, dt: f64) -> NewtonStep<Self::Tape> {
        // assemble residual + tangent, factor with Llt, Newton + Armijo,
        // push NewtonStepVjp onto tape via tape.push_custom, return NewtonStep
    }
    fn replay_step(&self, x_prev: &Tensor<f64>, v_prev: &Tensor<f64>,
                   theta: &Tensor<f64>, dt: f64) -> NewtonStep<Self::Tape> {
        // same as step but no &mut tape: rebuild factor on a scratch tape,
        // return NewtonStep whose factor+dr_dtheta the VJP consumes
        // (Phase-E checkpoint replay; skeleton implements as a thin wrapper
        // so the I-3 test can verify factor-on-tape survives a fresh solve)
    }
    fn current_dt(&self) -> f64 { self.config.dt }
    fn convergence_tol(&self) -> f64 { self.config.tol }
}

// sim-soft/src/differentiable/cpu.rs
pub type TapeNodeKey = u32;  // skeleton-local alias; chassis uses Var(u32)
pub struct CpuDifferentiable;
impl Differentiable for CpuDifferentiable {
    type Tape = CpuTape;
    fn register_vjp(&mut self, _key: TapeNodeKey, _vjp: Box<dyn VjpOp>) {
        // BF-4 stub: skeleton creates VJPs inline at push_custom time; this
        // method is kept for trait-contract compliance only. Book's registry
        // model is under-specified and doesn't match chassis's shipped API.
    }
    fn ift_adjoint(&self, tape: &CpuTape, step: &NewtonStep<CpuTape>,
                   upstream: &Tensor<f64>) -> Tensor<f64> {
        // Seed backward from upstream on the NewtonStep's output var, call
        // tape.backward, read tape.grad_tensor(theta_var). The IFT math is
        // inside NewtonStepVjp::vjp (Form A per §5).
    }
    fn time_adjoint(&self, _: &CpuTape, _: &[NewtonStep<CpuTape>],
                    _: &Tensor<f64>) -> Tensor<f64> {
        unimplemented!("time-adjoint is Phase E+ — single-step skeleton does not exercise")
    }
    fn fd_wrapper(&self, _: &dyn Fn(&Tensor<f64>) -> Tensor<f64>,
                  _: &Tensor<f64>) -> (Tensor<f64>, GradientEstimate) {
        unimplemented!("fd_wrapper is Phase G stochastic-adjoint path — skeleton is Exact-only")
    }
}

// sim-soft/src/observable/basic.rs
pub struct BasicObservable;
impl Observable for BasicObservable {
    type Step = NewtonStep<CpuTape>;
    fn stress_field(&self, step: &Self::Step) -> StressField { /* per Tet4 Gauss point */ }
    fn pressure_field(&self, step: &Self::Step) -> PressureField { /* -tr(σ)/3 */ }
    fn temperature_field(&self, _: &Self::Step) -> TemperatureField {
        TemperatureField::empty()  // no thermal coupling in skeleton
    }
    fn reward_breakdown(&self, step: &Self::Step, theta: &Tensor<f64>) -> RewardBreakdown {
        // peak_bound + stiffness_bound populated; pressure_uniformity +
        // coverage NaN-sentinel per §2 1-tet gap; scalar composition via
        // breakdown.score_with(&weights) is a separate call downstream
    }
}
```

**Skeleton type aliases:**

```rust
// sim-soft/src/lib.rs
pub type Vec3 = nalgebra::Vector3<f64>;
pub type SkeletonSolver = solver::CpuNewtonSolver<
    material::NeoHookean, element::Tet4, mesh::SingleTetMesh, contact::NullContact, 4, 1,
>;
```

**ForwardMap composition — the only `Box<dyn Solver<_>>`:**

```rust
// sim-soft/src/readout/mod.rs
pub struct SkeletonForwardMap {
    solver:       Box<dyn Solver<Tape = CpuTape>>,
    differential: CpuDifferentiable,
    observable:   BasicObservable,
    initial:      SceneInitial,  // x_0, v_0, mesh refs
}

impl ForwardMap for SkeletonForwardMap {
    fn evaluate(&mut self, theta: &Tensor<f64>, tape: &mut Tape) -> (RewardBreakdown, EditResult) {
        let theta_var = tape.param_tensor(theta.clone());
        let step = self.solver.step(tape, &self.initial.x_prev, &self.initial.v_prev,
                                    theta, self.solver.current_dt());
        let reward = self.observable.reward_breakdown(&step, theta);
        (reward, EditResult::ParameterOnly)
    }
    fn gradient(&mut self, theta: &Tensor<f64>, tape: &Tape)
        -> (Tensor<f64>, GradientEstimate) {
        // call self.differential.ift_adjoint(...), return Exact
    }
}
```

**Composition summary:**

- Public boundary: `Box<dyn Solver<Tape = CpuTape>>` — one runtime dispatch per step, per Part 11 Ch 01 §Solver.
- Hot path: `CpuNewtonSolver<NeoHookean, Tet4, SingleTetMesh, NullContact, 4, 1>` monomorphized; per-Gauss-point and per-iteration calls fully inlined.
- No `Box<dyn Material>` / `Box<dyn Element>` / `Box<dyn ContactModel>` on the hot path — all generic bounds.
- `Mesh` stays concrete (`SingleTetMesh`) on hot path; `&dyn Mesh` appears only in `ContactModel::active_pairs`, which is cold for NullContact (returns empty vec).
- `Observable::type Step = NewtonStep<CpuTape>` pins readout to the CPU backend type-level.
- `Differentiable::type Tape = CpuTape` pins VJP registry to the CPU backend type-level.

**`Box<dyn>` inventory for the full skeleton:** one — the public `Box<dyn Solver<Tape = CpuTape>>`. Plus the one that lives inside `sim-ml-chassis::Tape`'s `BackwardOp::Custom(Box<dyn VjpOp>)` per node. That's it. No additional type erasure.

**Ceremony cost:** 6 generic parameters on `CpuNewtonSolver`, 4-line where-clause. One type alias (`SkeletonSolver`) hides the noise. The parent Ch 01 claim 1 "trait generics on hot paths, trait objects on cold paths" split holds cleanly.

## 15. I-5 determinism risk classes (Round 6)

Enumeration of every place non-determinism could leak into `RewardBreakdown` bit-equality in the skeleton. Per-risk mitigation verified in-spec; I-5 test should pass by construction, not by accident.

| # | Risk class | Source | Mitigation (skeleton) |
|---|---|---|---|
| D-1 | Rayon work-stealing reorders FP reductions | faer 0.24's `rayon` feature, enabled by default, parallelizes sparse matvec / norm reductions. Thread assignment depends on runtime scheduler state → bit-different across runs. | **Disable faer's `rayon` feature** in Cargo.toml (§4). Single-threaded. 9-DOF system gains no speedup from parallelism anyway. |
| D-2 | Line-search branching on θ-threshold | Armijo's sufficient-decrease inequality: FD perturbation may cross the threshold, changing step length, propagating to x* bits. | Not mitigated structurally — gradcheck test is the detector (S-2). Failure → book-level finding in Part 5 Ch 00. |
| D-3 | HashMap iteration order | Rust's `HashMap` uses randomized Hasher per-process; iteration order differs across runs. | **Use `BTreeMap` or sorted `Vec<(key, val)>` for any numeric-path state.** Don't use `HashMap` in sparse-CSR build, triplet dedup, VJP primal stash. |
| D-4 | Sparse CSR triplet-insertion order | If assembly accumulates triplets in HashMap, order leaks into the built CSR's duplicate-merge sequence. | Insert triplets in sorted `(row, col)` order via `Vec` + sort, then pass to faer's sparse builder. Deterministic by construction. |
| D-5 | Chassis tape backward sweep | Linear reverse loop `for i in (0..=out_idx).rev()` — sequential, no parallelism. | **✓ Round-1 verified clean** (autograd.rs:744). |
| D-6 | `private-gemm-x86` SIMD kernel dispatch | Faer dispatches GEMM to AVX / AVX2 / AVX-512 kernel based on runtime CPU-feature detection. | Within-process: deterministic (feature set doesn't change mid-run). Cross-process on same machine: deterministic (same CPU). Cross-machine: **not** deterministic — out of skeleton scope; I-5 test runs on one machine via subprocess, not cross-machine. |
| D-7 | Uninitialized memory | Heap allocations contain arbitrary bytes until written. Reading before writing is UB and can produce bit-nondeterminism. | Rust enforces initialization before use at the type system level; `Tensor::zeros` always zero-inits. No risk with safe Rust. No `unsafe` in skeleton code. |
| D-8 | ASLR / address-dependent code | If any code path reads addresses (e.g., `ptr as usize`) and uses them in numeric computation, bits would differ per-run. | Don't do that. No address-dependent numerics in skeleton; pointer values are opaque. |
| D-9 | Wall-clock / RNG seeding | Any `SystemTime::now()` / `thread_rng()` / similar in a numeric path. | Forbidden in skeleton numeric code. RNG for initial guess: not needed (zero initial guess for Newton); if needed, seed deterministically from θ hash. |
| D-10 | Parallel `iter_mut().par_iter()` / rayon-style collectors | Any accidental use of parallel iterators in skeleton code. | **Skeleton's lint policy:** no `rayon::` imports outside feature-gated code. Code review before commit. |
| D-11 | `HashMap::iter().sum()` order in FP accumulation | Even without rayon, if any f64 sum iterates over HashMap entries, order is non-deterministic. | Subsumed by D-3 (no HashMap in numeric state). |

**Top-three in-the-skeleton exposure:** D-1 (rayon — mitigated via Cargo.toml), D-3/D-4 (HashMap/insert-order — mitigated by BTreeMap + sorted Vec discipline), D-2 (line-search non-smoothness — detected by gradcheck test, not mitigated structurally).

If the I-5 test fails, the diagnostic ladder is: (a) check for any HashMap in numeric path (grep), (b) check Cargo.toml rayon state (double-check faer features), (c) narrow to single-θ-component Stage 1 to see if failure is line-search-related (if Stage 1 passes but Stage 3 fails, line-search non-smoothness is the culprit → book-level Part 5 Ch 00 finding).
