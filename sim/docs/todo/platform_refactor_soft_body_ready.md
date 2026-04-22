# Platform refactor — soft-body readiness

**Status:** Scoping phase. Contents tonight are the **audit gameplan only** — the actual inventory, book-vs-code diff, breaking-change enumeration, and sequencing work begins next session. This memo is the parking position.

**Branch:** `feature/platform-refactor-audit`.

**Relationship to other memos:** This audit is *upstream* of [`soft_body_walking_skeleton_scope.md`](soft_body_walking_skeleton_scope.md), which is now marked SUPERSEDED pending this audit's completion. The walking skeleton's six invariants all assumed a soft-body-ready L0 platform that does not currently exist; this memo scopes the refactor that makes them true.

## Why this exists

`sim-soft` cannot be built on the L0 platform as it stands today. Known deficiencies surfaced in the 2026-04-21 session:

- `sim-ml-chassis::Tape` is scalar-only `f64` with a closed set of 9 primitives + 3 fused ops; no custom VJP registration; no vector cotangents.
- `sim-ml-chassis::Tensor` is `f32`; the book assumes `Tensor<f64>`.
- `faer::Llt<f64>` factor-on-tape pattern is asserted by the book but nothing in the current tape can carry a factorization as a tape node. **faer is not in workspace `Cargo.toml` at all** — greenfield dep, no version to preserve (confirmed Group A step 1, 2026-04-22).
- Cross-crate contracts (`ForwardMap`, `Differentiable`, `Solver`, `SdfField`) have no current home in the tree.
- These are symptoms of a broader "platform not soft-body-ready" state, not isolated bugs.
- **`sim-simd` is not actually SIMD** — the crate name and lib docs claim "portable SIMD intrinsics," but grep finds zero `std::simd` / `portable_simd` / `core::arch` usage. It's SoA layout (`Vec3x4`, `Vec3x8`, `#[repr(C, align(32))]`) + scalar loops relying on the auto-vectorizer. Only one live consumer (`find_max_dot` from `sim-core::gjk_epa`); the physics-batch surface (`ContactBatch4`, `batch_normal_force_4`, etc.) has no callers. Fate of this crate is a Group E/F question, not Group A (confirmed Group A step 2, 2026-04-22).

A patch-the-tape approach would leak into every other sim-soft dependency. Refactor the whole L0 platform for soft-body-readiness *first*, then build sim-soft on top. Aligned with "R34 GT-R overbuild chassis," "rocket platform — foundations before payload," and "own every line" principles.

**Prior art.** `sim-ml-chassis` has been refactored before — the ML chassis refactor study (PRs #188 `3d6f9ad8`, #190 `48f71ab3`, #192 `2835b4f4` — the chassis/RL split landed April 2026). That playbook is partially established; Group E migration decisions can cite that history rather than rediscovering the pattern, and Group B's tape-internals work has an existing precedent for "invasive chassis change shipped without regressing consumers."

## Scope is intentionally open

The 12 `sim/L0/*` crates are the **current** understanding of refactor targets, but scope is not frozen. Adjacent trees may enter scope as the audit surfaces dependencies:

- **`design/cf-design/`** — sibling tree at `design/cf-design/` (alongside `cf-geometry`, `cf-spatial`). Its boundary with sim-soft (`SdfField`, `MaterialField`, `EditResult` trait surface) is almost certainly in scope — those traits have to live somewhere Group D places, and cf-design migrates to match. Internals may or may not need changes; TBD during the walk.
- **`sim/L1/sim-bevy/`** — Layer 1 visualization. Consumes sim-soft's `Observable` trait at Phase I. Group D's `Observable` placement decision affects it. In scope as a future-consumer informing our trait decisions; internals (rendering code) almost certainly stay untouched.
- **`mesh/` crate family** — 10 crates at `mesh/` (`mesh`, `mesh-types`, `mesh-sdf`, `mesh-repair`, `mesh-offset`, `mesh-lattice`, `mesh-printability`, `mesh-shell`, `mesh-io`, `mesh-measure`). Several overlap our concerns: `mesh-types` with Group A (Vec3/face/vertex primitives shared with `sim-types`?); `mesh-sdf` with the Phase-G pure-Rust tet-mesher commitment; `mesh-repair` as probable home of libigl-lineage self-intersection repair. Fully in-scope-or-not determined during the walk; not pre-excluded.

No pre-commitments to "out of scope." If the audit surfaces work that needs to land in these trees to unblock soft-body-readiness, we do it. Scope discovers itself.

## Six groups the audit covers

All twelve `sim/L0/` crates map to exactly one primary group. Confirmed 2026-04-21 by crate-by-crate walk:

| Group | Scope | Primary crates |
|---|---|---|
| **A. Numerical/type foundations + numerical policy** | Types (Vec3, Quaternion, etc.), `sim-ml-chassis::Tensor` (f32 vs f64 tension), nalgebra + faer integration, IEEE754 mode + NaN/denormal handling, tolerance conventions, workspace-wide dependency alignment | `types`, `simd`, `ml-chassis` (Tensor half) |
| **B. Autograd platform** | `sim-ml-chassis::Tape` (scalar → vector, custom VJPs), factor-on-tape for faer `Llt`, `Differentiable` trait placement (chassis vs sim-soft), checkpointing + time adjoints, GPU-tape readiness | `ml-chassis` (tape half) |
| **C. Compute backends** | `sim-gpu` current state, wgpu integration patterns, CPU↔GPU parity discipline (the oracle question) | `gpu` |
| **D. Cross-crate contracts** | Trait placement (chassis vs sim-soft vs sim-core), `ForwardMap` + `Differentiable` + `Solver` locations, `SdfField` boundary, coupling patterns to `sim-core` / `sim-thermostat`, IO ingress | `core`, `thermostat`, `mjcf`, `urdf` |
| **E. Existing consumer compatibility** | What breaks in `sim-rl`, `sim-opt`, `sim-therm-env`, `autograd_policy`, `autograd_layers`; migration path for each | `rl`, `opt`, `therm-env` |
| **F. Build/test/CI infrastructure** | Workspace `Cargo.toml` hygiene, feature flag strategy, test matrix, `xtask grade` readiness for a new physics crate, new-crate onboarding checklist | `tests`, workspace config |

**Cross-cutting concern: determinism.** The book's `ForwardMap` determinism-in-θ contract threads through B, D, E, F — traced across those groups, not made its own group.

**Heads-up: Group B is the largest domain.** Likely reveals sub-groups as we walk it (tape internals vs higher-level autograd — IFT, time-adjoint, checkpointing — vs GPU-tape readiness). Hold the taxonomy loose and re-split if the walk demands it.

**Heads-up: Group E risk concentration.** Within Group E, `sim-opt` (the most autograd-dependent consumer — uses chassis tape via SA / richer-SA / PT / dual-metric analysis pipeline) and `sim-therm-env` (the most integration-heavy consumer — ThermCircuitEnv spans chassis + core + thermostat) are the likeliest to break hard under any chassis-B change. Eyes on those two first during the breaking-change enumeration step.

## Methodology per group

1. **Inventory** — catalog public API surface, list deps up + down.
2. **Book-vs-code diff** — every concrete API the book cites (`Tensor<f64>`, `Llt<f64>`, `register_vjp`, `SdfField`, `ForwardMap`, etc.) verified against code. Record hits, misses, stale-claims.
3. **Breaking-change enumeration** — for each proposed change: downstream consumers + what breaks for each.
4. **PR-sized chunking with dependency graph** — logical units that can ship independently, with dependencies drawn explicitly.
5. **Commit/defer decisions** — per chunk: "do now," "defer to post-skeleton," "defer to Phase X." Every defer is explicit with rationale.

Book edits emerge naturally from step 2:
- **Pure spec changes** (no code impact): land inline as we decide them.
- **Spec-matches-code changes**: land with the code change they document, not separately.
- **Not** a separate phase after the audit. Distributed through audit and refactor.

## Decisions recorded

### Group A

**A.1 — Precision strategy (2026-04-22).** Make `Tensor` generic: `Tensor<T>` where `T: Copy + Default + PartialEq + Debug + Send + Sync + num_traits::{Zero, One, Float}`. RL consumers use `Tensor<f32>`; sim-soft uses `Tensor<f64>`.

- **Why not status quo:** book's `Tensor<f64>` assumption wouldn't be satisfiable; sim-soft would need a second tensor type.
- **Why not Tensor-always-f64:** 2× memory on RL replay buffers, blocks future f32-GPU policy inference, punishes RL for sim-soft's needs.
- **Why generic beats two distinct types (`Tensor` + `TensorF64`):** follows nalgebra/ndarray pattern; one type family, no helper duplication; mixed-precision (bf16/f16) futureproofing is basically free.
- **Adjacent consequence:** `num-traits` promoted to explicit workspace dep (already transitive via nalgebra — trivial promotion, not a new dep decision).
- **Sub-decision B-2 — `TensorSpec` stays f32-only.** It's an RL-environment-declaration concept; sim-soft has no env, no bounded actions, no use for it. Avoids plumbing a type parameter through `Env` / `VecEnv` / `Policy` traits.
- **Sub-decision — `from_f64_slice` keeps its name.** `Tensor<f32>::from_f64_slice` is the "real" one (explicit sim/ML boundary cast); `Tensor<f64>::from_f64_slice` becomes a trivial generic-monomorphization artifact. Don't over-design.
- **Downstream work (Group E):** every RL consumer gains a type parameter at `Tensor` sites (`sim-rl`, `sim-opt`, `sim-therm-env`, `AutogradPolicy`/`AutogradValue` forward signatures). Mostly mechanical; scoped per-crate during Group E.
- **Downstream work (Group E) — post-walk reconciliation (2026-04-22).** Per-consumer outcomes verified across E.1–E.4 walks: sim-opt = zero `Tensor` sites in src + tests (E.1, `742af0f7`); sim-therm-env = 4 prod `Tensor` sites in `env.rs` + ~28 test-site references, all gaining explicit `<f32>` annotation under A.1 sub-decision B-2 (E.2, `4acf0062` — prediction holds); `AutogradPolicy` / `AutogradValue` = zero `Tensor` sites, forward signatures use `&[f32]` obs + `&[f64]` params/actions throughout (E.3, `9288d696`); sim-rl = 5 src `Tensor` sites all type-inferable via `action_data: Vec<f32>` → `env.step(&Tensor<f32>)` (A.1 sub-decision B-2), zero explicit annotation needed, worst-case 2 turbofish at `sac.rs:321` + `td3.rs:313` if Rust inference fails at chassis A.1 PR compile time (E.4, `1c6a94a9`). **Three overstatements + one hold across all four named consumers.** The "every RL consumer gains a type parameter at Tensor sites" framing above holds only for sim-therm-env; for sim-opt and autograd modules there are zero sites to annotate, and for sim-rl the 5 sites propagate `T = f32` without annotation. "Mostly mechanical; scoped per-crate" remains accurate in spirit — the mechanical surface is smaller than the prediction implied.

**A.2 — faer integration (2026-04-22).** Adopt faer as the CPU sparse linear algebra workhorse, confirming the book's existing commitment at the workspace level. MSRV 1.84 comfortably below our workspace floor of 1.85.

- **Version:** `faer = "0.24"` (latest on crates.io; matches book's `sparse::linalg::solvers::Llt` / `linalg::solvers::Solve` API paths; semver-compatible floating patch per the nalgebra pattern).
- **Features:** `default-features = false, features = ["std", "rayon", "sparse-linalg"]`. Excludes `npy` (no NumPy IO), `rand` (we already have workspace `rand`/`rand_distr`; faer's rand is random-matrix generation, not needed), `nightly`, `perf-warn`, `unstable`.
- **Placement deferred to sim-soft creation PR.** No existing L0 crate needs faer — RL baselines don't factor, `sim-opt` doesn't factor, `sim-core` uses small-matrix nalgebra math. Adding to `[workspace.dependencies]` now would be unused boilerplate. When the `sim-soft` crate is created, its `Cargo.toml` PR cites this decision and adds the workspace entry simultaneously.
- **`faer-ext` (nalgebra interop) deferred.** No concrete use case yet — Hessian assembly walks tet blocks, not per-body nalgebra matrices. Add if a Group D bridge surfaces; YAGNI until then.
- **Relationship to nalgebra:** coexist, different roles. nalgebra owns small-matrix geometry (`Vector3`, `Point3`, `Isometry3`, per-body inertia). faer owns large-scale linear algebra (Hessians, sparse Cholesky, Newton step solves). No interop contract at Group A.

**A.3 — nalgebra integration (2026-04-22).** Confirm status quo. No version/feature changes; two inline stale-reference fixes.

- **Version:** `nalgebra = "0.34.2"` — stays (already latest on crates.io).
- **Features:** `default` (= `std, macros`). `serde-serialize` continues to be opt-in per-consumer via `sim-types`'s `serde` feature (no change).
- **Re-export policy:** keep the per-crate subset pattern. `sim-types` re-exports `{Isometry3, Point3, UnitQuaternion, Vector3}`; `sim-core` re-exports `{DMatrix, DVector, Matrix3, Matrix6, UnitQuaternion, Vector3}`. No centralization, no expansion. When `sim-soft` needs `SMatrix<f64, 12, 12>` for per-tet blocks, it imports from nalgebra directly.
- **Sparse policy:** explicit non-adoption of `nalgebra-sparse`. **faer owns all sparse.** Temptations to pull `nalgebra-sparse` for quick sparse tests get redirected to faer for consistency.
- **No scalar alias.** `pub type Real = f64` is *not* introduced. Over-abstraction for zero-generic-code gain; precision decisions happen at type boundaries (`Tensor<T>` from A.1), not via a crate-level typedef.
- **Sub-decision — sim-soft hard-codes `f64`, no `<T: RealField>` generics.** A.1's `Tensor<T>` generic was justified by RL-at-f32 + sim-soft-at-f64. sim-soft's physics has no f32 use case: book commits to f64 end-to-end, GPU takes its f32 at the solver-hand-off boundary (§80 Ch 02). Generic-for-its-own-sake rejected.
- **Inline fixes landed with this decision:**
  - `Cargo.toml:325` — `rust-version = "1.85"` → `"1.87"` (nalgebra 0.34.2 floor; local rustc 1.95 was masking this).
  - `sim/docs/GPU_SDF_COLLISION_SPEC.md:883` — `nalgebra = "0.33"` → `"0.34"` (book-vs-code stale reference).

**A.4 — IEEE754 / NaN / denormal / tolerance policy (2026-04-22).** Codify existing de-facto convention as explicit policy. No behavior changes; future policy violations cite "A.4."

1. **NaN/Inf handling: validate at system boundaries, propagate internally.** External input (MJCF/URDF parsers, network deserialization, user-facing constructors) gets explicit `.is_finite()` checks returning typed errors (e.g., `SimError::InvalidTimestep`, `TensorError::*`). Internal cross-crate API calls trust the caller — Rust's type system is the contract. Within-crate arithmetic propagates IEEE. Never `panic!` on NaN; panic is reserved for programming errors, NaN is a domain signal.

2. **Denormals: IEEE-correct (no FTZ/DAZ).** Don't set flush-to-zero. Gradcheck correctness at small FD step sizes outweighs the rare denormal performance hit. Revisit *only* if a benchmark surfaces a real denormal-induced slowdown in a hot path.

3. **FMA contraction: off globally, on case-by-case.** Rust's default — do not fight it. `clippy::suboptimal_flops` allowed per-crate (already the pattern in `sim-types`). `.mul_add()` used only at explicit developer choice with benchmark justification.

4. **Determinism: algorithm-output, not bit-exact.** Same θ on same machine on sequential path is bit-reproducible as a side-effect of (1)+(3), but not a project guarantee. Parallel paths (rayon) accept float non-associativity. Cross-platform libm divergence (`sin`/`cos` differing by ULPs between x86-64 and ARM64) is accepted — falls within the 5-digit gradcheck tolerance. The `ForwardMap` determinism contract is satisfied by *"same θ → same output within engine tolerance"*, not by bit-identical byte streams.

5. **Tolerances: two-tier, not centralized.** (**Opinionated choice, not pure Rust idiom.** nalgebra + approx do centralize via traits; we deliberately don't.)
   - **Physical tolerances** live on domain config structs with units in docs (`SolverConfig::contact_tolerance: f64` in meters). Already the pattern.
   - **Numerical epsilons** are per-algorithm, per-crate. No shared `sim_types::tolerance` module. Centralization invites thoughtless reuse (`EPSILON_MEDIUM` used where the author should think about units).
   - **Project-wide exception:** gradcheck tolerance lives as `GRADCHECK_REL_TOL: f64 = 1e-5` in sim-soft's testing module when the crate arrives (per book §110 Ch 04's 5-digit commitment). Cited, not duplicated.

6. **Approx-equality: `approx` crate, per-site choice.** Test authors pick `relative_eq` vs `abs_diff_eq` based on what's being tested. `clippy::float_cmp` allowed in test modules only (via `#[allow]`). No workspace-level `#[deny(clippy::float_cmp)]` — risks triggering in generic serde/derive paths.

7. **Float ordering: per-site, prefer `total_cmp`.** Where total ordering is needed (RL reward ranking, best-tracker, fitness sorting), use `f64::total_cmp` with a one-line comment naming why total ordering is load-bearing (NaN sorting, ties, etc.).

### Group B

**Status:** B.1, B.1.a, B.2, B.3, B.4, B.5, B.5.a locked. Cross-cutting determinism audit complete (2026-04-22 walks B.1–B.5.a). 8 queued book edits bundled for Group B close; 1 deferred to Group D close (`Solver` docstring + `replay_step` signature); 3 GPU-cross-call-state pre-loads for Group C.

**Cadence shift mid-walk.** User flagged that Group B's autograd-internals depth exceeds their reps; switched to recommend-first cadence (Claude makes the call + compact why + alternatives, user checks against principles). Recorded as `feedback_recommend_first_deep_specialist.md`. Design-philosophy-adjacent calls (crate boundaries, what-belongs-where) stayed interactive.

**B.1 — Tape shape: vector-aware (2026-04-22).** `Tape` nodes carry `Tensor<f64>` values; custom VJPs handle non-primitive ops. Current 9 scalar primitives stay as the same nine, generalized to `Tensor<f64>` (a "scalar" is shape `[]`).

- **Why not scalar-only:** book's `faer::Llt<f64>` factor-on-tape requires representing a factorization as a tape node; no way to do that in a scalar tape without fabricating an opaque-node escape hatch (violates "own every line"). Hessian-vector cost also blows up scalarized.
- **Why not two tapes (scalar for RL, vector for sim-soft):** scope memo explicitly names this as leaky. Two sets of VJPs, two determinism contracts, `Differentiable` straddles. Rejected.
- **Compound with A.1:** A.1's `Tensor<T>` becomes structurally load-bearing — tape speaks Tensor, Tensor speaks precision-generic, decisions compound.

**B.1.a — `BackwardOp` representation (2026-04-22, amended during B.2).** Sum-type for built-in primitives + trait object for custom ops:

```rust
enum BackwardOp {
    Leaf,
    Unary { parent: u32, rule: UnaryRule },   // Neg, Tanh, Relu, Square, Ln, Exp
    Binary { lhs: u32, rhs: u32, rule: BinaryRule }, // Add, Sub, Mul
    Custom(Box<dyn VjpOp>),
}

pub trait VjpOp: Send + Sync {
    fn op_id(&self) -> &'static str;  // stable identity, futureproofs serialization
    fn vjp(&self, cotangent: &Tensor<f64>, parent_cotans: &mut [Tensor<f64>]);
    fn parents(&self) -> &[u32];
}
```

- **Why hybrid beats pure-`Box<dyn>`:** primitives stay allocation-free and inspectable; only custom ops pay one allocation per node.
- **Why hybrid beats pure-sum-type:** every future sim-soft op would force codebase-wide enum expansion; hybrid lets sim-soft add ops orthogonally.
- **Amendments from B.2 overbuild:** `name()` → `op_id()` (intent: stable identity), `Send + Sync` bounds added. Both are expensive to retrofit later across every op impl; adding now is ~free (sim-soft's custom ops carry tensors and faer factors that are Send+Sync naturally).

**B.2 — Custom VJP registration (2026-04-22).** Direct method on tape:

```rust
impl Tape {
    pub fn push_custom(&mut self, value: Tensor<f64>, op: Box<dyn VjpOp>) -> Var { ... }
}
```

- **No registry.** Mirrors existing API shape (every primitive is already a `Tape` method). Standard registry justifications don't apply: fusion/optimization explicitly rejected by chassis lib doc, cross-tape op sharing handled by `Arc<dyn VjpOp>` if ever needed.
- **Overbuild (post-user-pushback on R34 principle):** chassis carries `op_id` + `Send+Sync` now; everything else — global registry, serde bounds, wire protocol — is additive and bolts on later when concrete. "Serviceable for future pivots" ≠ "build speculatively."
- **Serialization flag:** chassis already serializes `PolicyArtifact` / `TrainingCheckpoint` (parameters + optimizer state, `sim-ml-chassis/src/artifact.rs`); tape is transient by design. No current need for tape graph serialization; `op_id` + Send/Sync leave the door open.

**B.3 — `Differentiable` trait placement (2026-04-22).** Placed in `sim-soft::autograd` per the book. No deferral to Group D needed.

- **Why not chassis:** trait methods reference sim-soft types (`NewtonStep<Self::Tape>`) and sim-soft semantics (IFT adjoints, time-adjoints, gradcheck); chassis placement would force chassis to know about soft-body physics (wrong direction of dependency).
- **Why not a `sim-autograd-traits` shim crate:** single-trait shim is premature abstraction (own-every-line).
- **Why not `sim-core`:** core is rigid-body; `Differentiable`'s IFT/time-adjoint are soft-body variational-dynamics concerns.
- **No rename of existing `DifferentiablePolicy`** (`sim-ml-chassis/src/policy.rs:80`): RL-specific, different crate, different purpose; the `Policy` suffix disambiguates. Brief doc note in both trait comments when the sim-soft crate lands.
- **Book edit queued** (pure spec change, lands inline at Group B close): `Differentiable::register_vjp` signature changes from closure-based (`vjp: F where F: Fn(&Self::Tape, &Tensor<f64>) -> Tensor<f64>`) to `Box<dyn VjpOp>`-based per B.2. Location: `docs/studies/soft_body_architecture/src/110-crate/01-traits/00-core.md` ~line 106.

**B.3 sub-decision — `CpuTape` as type alias (2026-04-22).** `pub type CpuTape = sim_ml_chassis::Tape;` in sim-soft (not a newtype wrapper).

- **Why not newtype:** concrete benefits (enforced invariants via types, orphan-rule escape for external traits, inherent methods) are all hypothetical. External sim-soft consumers interact via `Differentiable`/`ForwardMap`/`Observable`, not the tape type directly — curation has no audience. No real invariants to enforce; no external traits we need to impl on the tape; candidate "inherent methods" fit better as free functions or methods on physics-orchestrator types.
- **Why not direct-use (no alias):** alias gives sim-soft a vocabulary word. `&mut CpuTape` reads cleaner than `&mut sim_ml_chassis::Tape` in sim-soft contexts, and makes `CpuTape`/`GpuTape` parallelism at `Solver` impl sites legible.
- **Migration acknowledged:** alias → newtype is a focused PR (~200–500 lines, mostly mechanical) if a concrete trigger surfaces later. Migration cost scales with sim-soft size, so prompt migration on real triggers — not deferred-indefinitely.

**B.4 — Checkpointing + time adjoints (2026-04-22).** Mechanism locked; one downstream dependency queued for Group D. Checkpointing + time-adjoint machinery lives entirely in `sim-soft::autograd` — chassis `Tape` gains no rollout-awareness, no segment orchestrator, no checkpoint-schedule API.

Mechanism: each converged Newton step is one chassis `push_custom` node whose `VjpOp` impl is `CheckpointedStepVjp` — holds primal only (`(x_prev, v_prev, Δt)`); rebuilds the factor inside `vjp` via side-effect-free Newton replay (exact replay-capability surface queued to Group D), applies back-substitution, releases the factor before returning. RAII pushes the book's "tape drops → factor released" contract from tape-lifetime scope down to per-vjp-call scope. Phase D default matches the book's $k{=}1$ commitment (`60-differentiability/04-checkpointing/00-uniform.md`: "asymmetry drives optimum to $k{=}1$"). Adaptive-Δt, contact active-set determinism, and Brownian-path storage (Phase H only) are already covered by the book's primal-only payload contract — no B.4 surface additions.

`Differentiable::time_adjoint` consumes the chassis reverse walk at the per-step `VjpOp` boundary; the exact orchestration surface (direct `tape.backward` call vs. manual walk over the rollout slice vs. mix) pins at implementation time once B.5 clarifies tape semantics for the GPU variant. A future eager-step variant (holds `BackwardEulerState` across the forward→backward boundary) is additive if a short-rollout benchmark ever wants one — not shipped in Phase D.

- **Why not chassis-owned checkpointing.** Generic-activation-checkpointing (à la `jax.checkpoint`) is a different problem from Newton-replay: sim-soft's replay re-runs the physics Newton loop, not a feedforward pass. Bundling would force chassis to learn about physics (B.3's "wrong direction of dependency"). Book commits `autograd/` as the owner (`110-crate/00-module-layout/07-autograd.md` ¶1).
- **Why not tape-of-tapes / sub-tape per segment.** Doubles tape machinery for $k{=}1$'s zero benefit; re-introduces the two-tape anti-pattern B.1 rejected. Reopen only if Phase-E benchmarks show the single-tape reverse walk is a bottleneck at large $k$.
- **Phase-E Revolve extends, does not replace.** Revolve's $k>1$ segment replay needs a sim-soft orchestrator spanning multiple `CheckpointedStepVjp` nodes — cross-step coordination, not "pick a different `VjpOp`." Still above chassis `backward`, still no chassis changes. Benchmark-gated per `04-checkpointing/02-tradeoff.md`.

**B.4 → Group D dependency (queued).** `CheckpointedStepVjp::vjp` takes `&self` (per B.1.a). Current `Solver::step(&mut self, tape: &mut Self::Tape, ...)` (`docs/studies/soft_body_architecture/src/110-crate/01-traits/00-core.md` ~line 88) is `&mut self` — can't be called through a `VjpOp`'s `&self` handle. Candidate refinement: `replay_step(&self, x_prev, v_prev, theta, dt) -> BackwardEulerState` alongside `step`, matching "replay is deterministic given stored primal + Δt" (`04-checkpointing/02-tradeoff.md` §contact-active-set). Exact signature deferred to Group D, where the `Solver` trait's full consumer surface (including Phase-E GPU variant, B.5) gets walked. Book edit to Part 11 Ch 01 lands with Group D close.

**B.4 does not lock:**
- The `Solver` replay-method signature, `StepPrimal` layout, or solver-handle lifetime contract — all queued to Group D.
- Phase-E Revolve specifics beyond "still in sim-soft, still doesn't touch chassis."

**B.5 — GPU-tape readiness (2026-04-22).** Chassis acquires a GPU module behind a `gpu` feature flag. `sim_ml_chassis::gpu` owns `GpuDevice`, `GpuTensor<T: GpuScalar>`, `GpuTape<'device>`, `GpuTapeEntry`, `GpuTensorPool`, and three registries (`BindGroupLayoutRegistry`, `PipelineCache`, `VjpRegistry`). wgpu enters chassis's `[dependencies]` gated behind `gpu`. No generic `Tape<Device>` parameter, no `TapeBackend` trait — backend polymorphism lives at `SpMvOp` / `Preconditioner` trait level per book §80 Ch 04 §01.

- **Why chassis and not sim-soft** (Option A beats the sim-soft-owned Option C from scope memo). Initial recommend-first call leaned toward Option C; stress-test pushback established the "one conceptual API" architectural property the book fought for is a real regression target under Option C — sim-rl wanting GPU later would either inversely depend on sim-soft, duplicate machinery, or punt to CPU. User flipped the load-bearing assumption ("sim-rl should definitely be looking at using GPU sooner than later"), tipping EV from C to A. Also aligns with B.1's "chassis grows when shared concern justifies it" — cross-consumer substrate, not physics-domain machinery (the latter being what B.3/B.4 rightly kept out).
- **Why not a `TapeBackend` trait** (option B from scope memo): book rejects, rightly — generics thread through every consumer call site, trait-object dispatch at the tape level defeats static dispatch in hot paths.
- **Why feature-gated:** RL CI pipelines that don't need GPU don't pull wgpu's transitive graphics-driver load. `default-features = []` on chassis's `gpu`; consumers opt in with `features = ["gpu"]`.

**B.5.a — `GpuScalar` trait shape (2026-04-22).** Sealed marker trait with enum-discriminant associated const:

```rust
mod sealed { pub trait Sealed {} }

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum GpuScalarKind { F32, F64 /* F16 when shader-f16 use case lands */ }

pub trait GpuScalar:
    sealed::Sealed + bytemuck::Pod + Copy + Send + Sync + 'static
{
    const KIND: GpuScalarKind;
}

impl sealed::Sealed for f32 {}
impl GpuScalar for f32 { const KIND: GpuScalarKind = GpuScalarKind::F32; }
impl sealed::Sealed for f64 {}
impl GpuScalar for f64 { const KIND: GpuScalarKind = GpuScalarKind::F64; }
```

- **Why f64 admission at Phase E** (revised from f32-only-preimpl after book-wide grep surfaced `GpuTensor<f64>` in §80 Ch 02 sparse-solvers). f64 is a buffer-storage scalar for compensated-summation accumulators; arithmetic still happens at f32 per book §01 mixed-precision commitment. **Scalar admission ≠ kernel availability:** `PipelineCache::lookup` returning `None` for `(KernelId, F64)` is a runtime miss, not a type-system constraint. The trait defines which types can inhabit `GpuTensor<T>`; the registry defines which `(KernelId, GpuScalarKind)` pairs have dispatch support.
- **Why enum discriminant over `&'static str`:** pipelines compiled at chassis init per book §80 Ch 04 §01; no runtime WGSL templating. `HashMap<(KernelId, GpuScalarKind), ComputePipeline>` key wants exhaustive-matchable discriminant, not string compare. Future `F16` variant addition produces compile errors at every dispatch match site — right failure mode.
- **Why sealed:** new GPU scalars require chassis-level WGSL kernels shipping simultaneously; sealing reflects the workflow rather than creating a new restriction.
- **Why `bytemuck::Pod` bound:** `GpuTensor<T>` uploads via bit-cast to `&[u8]`; `Pod` is the actual correctness contract. `bytemuck` is already transitive via wgpu — direct dep behind `gpu` feature is free.

**B.5 also ratifies** (all per book, all implied by Option A):
- Registries are `GpuDevice`-owned with `'device`-lifetime borrowed by every `GpuTape`; not global statics.
- `vjp_coverage_test` in chassis asserts every registered forward `KernelId` has a matching `VjpKernelHandle`.
- Cross-backend tape (`AnyTapeEntry`) lives in chassis; exact enum layout is Phase-E implementation detail per book.
- Minimal chassis-shipped kernel set per book §01: `zero`, `copy`, `axpy`, `dot`, `elementwise_mul`. Physics / MLP kernels register on top from sim-soft / sim-rl at startup.
- Grad-of-grad out of scope on both CPU and GPU in Pass 1.
- wgpu version pin deferred to implementation PR (A.2-style "adopt latest at impl time").

**Queued book edits from B.5** (land at Group B close, bundled with B.2/B.3 queue):
- §80 Ch 04 §01 `01-gpu-backend.md` — spec `GpuScalar` trait with the shape above; document scalar-admission-vs-kernel-existence split.
- §80 Ch 04 §01 "Cross-backend interop" — add note that CPU-f64 → GPU-f32 marshal requires explicit precision-change cast (`GpuTensor::from_cpu_f64_as_f32`), not polymorphic.
- §80 Ch 03 `03-gpu-autograd.md:62` — reconcile `backward_pass` seed type with `01-playback.md:8` (parent says `GpuTensor<f64>`, sub-leaf says `GpuTensor<f32>`; sub-leaf is authoritative per Phase-E-f32-default commitment).
- §80 Ch 04 §00 `00-current-cpu.md` (from B.1/B.2 queue) — refresh "scalar-only f64, 12 public ops" to post-B.1 vector-aware tape with `Custom(Box<dyn VjpOp>)`.
- §80 Ch 04 §02 `02-vjp-api.md` (from B.2 queue) — CPU-side `CustomVjp` sketch → `Box<dyn VjpOp>` with `op_id`/`parents`/`vjp`.

**B.5 does not lock:**
- `AnyTapeEntry` exact enum layout — Phase E implementation.
- `Solver::replay_step` signature — Group D (unchanged by B.5: GPU solver has same `&self` VJP constraint as CPU; see B.4's queued item).
- wgpu version pin — implementation PR.
- Phase E build-sequence ordering beyond book-committed (chassis GPU → sim-rl adoption → sim-soft consumption → sim-opt Phase F) — Group E/F.
- Whether sim-rl picks up `gpu` feature in Phase A–I — Group E migration timing. User has flagged it should land sooner rather than later.

### Group C

**Status:** C.1, C.2, C.3, C.4 locked. Group C closed.

**C.1 — GPU VJP author contract placement (2026-04-22).** Dual-surface: Rust-side docstring on `sim_ml_chassis::gpu::VjpRegistry::register` + canonical book paragraph at §80 Ch 04 §02 `02-vjp-api.md` (already live per Group B close commit `3f3b3463`) + one-line cross-reference from §80 Ch 04 §01 `01-gpu-backend.md` → §02. Substance pre-locked by B.5 walk: deterministic within A.4 §4 tolerance; forbidden modes on GPU = non-deterministic subgroup shuffle (vendor-lane-dependent), vendor-state readback, uninitialized workgroup-shared reads, non-IEEE-deterministic intrinsics beyond the A.4 §4 tolerance band.

- **Why §02 is canonical and not §01.** §02's leaf topic is VJP registration at the chassis level; the author-determinism contract is a VJP-authoring concern, not a module-surface concern. Keeping canonical substance in §02 and cross-referencing from §01 avoids the duplication-drift failure mode that bit the canonical `00-registration.md` leaf pre-`5f94df22`.
- **Why a Rust-side docstring as well as the book paragraph.** Symmetry with CPU B.1.a/B.2: CPU `VjpOp` trait gets a Rust-side trait docstring (queued to chassis tape refactor PR); GPU `VjpRegistry::register` gets the parallel Rust-side anchor. A user browsing `cargo doc` on the chassis crate sees the same determinism contract on both backends — preserves "one conceptual API" (B.5 locked) at the `rustdoc` surface, not just the book surface. Rust docstring is a signpost (~4 lines: one-sentence contract + 4-bullet GPU-forbidden-modes compact list + cross-ref to §02 canonical); full enumeration stays in §02.
- **Why the §01 cross-reference.** §01's "No per-tape-instance `VjpOp` registration on GPU" callout points readers at Ch 03 §00 (recording mechanics) but not at §02 (author contract). One-sentence addition preserves navigation from the GPU-module-surface leaf to the VJP-authoring-contract leaf. Matches the existing §110 Ch 02 `03-ml-chassis.md:51` cross-reference convention (leaf-level link, no anchor fragment — sidesteps mdbook em-dash anchor uncertainty).
- **Pre-load location note.** B.5's pre-load phrased the candidate book location as §80 Ch 04 §01; substance actually landed in §80 Ch 04 §02 during Group B close. §02 is the right home (leaf topic match); the pre-load's §01 reference was imprecise.
- **Stress-test: does a Rust docstring on `register()` reach the right audience?** Partial — the register call site lives inside `register_builtin_vjps()`, touched briefly when a physics-op author adds a new `(KernelId, VjpKernelHandle)` pair alongside their WGSL kernel. Narrow but real audience; `cargo doc` broadens it. Recommendation stands at "both" with the Rust docstring framed as a signpost, not the canonical substance.
- **Stress-test declined:** docstring on `VjpRegistry` type itself (not just `::register`). Mild case; duplicates signpost without adding context. Revisit if Rust-surface navigation feedback surfaces a gap at Phase E.

**C.1 → Queued Rust-side docstring commitment.** `sim_ml_chassis::gpu::VjpRegistry::register` ships its determinism-contract docstring when the chassis GPU module is created (Phase E per B.5's build sequence). Not a new separate PR — lands with the module creation.

**C.1 → One queued book edit (lands at Group C close).** §80 Ch 04 §01 `01-gpu-backend.md` — suffix the existing "per-tape-instance `VjpOp` registration" callout with a one-sentence cross-reference to §02. **Landed inline as part of C.1 decision** (not deferred).

**C.1 does not lock:**
- Whether `VjpRegistry` itself (type-level, not method-level) carries a docstring. Leaning against; revisit if Rust-surface navigation feedback surfaces a gap at Phase E.
- Phase E gradcheck §03 GPU-variant deliverable — already flagged in B.5 queued edit #8 as a candidate Phase E deliverable, not a C.1 decision.

**C.2 — Preconditioner cross-call cache scope (2026-04-22).** Trait docstring on `sim_ml_chassis::gpu::Preconditioner` forbids output-affecting cross-call mutation. Intra-call state (AMG pattern cache across Newton iterations within one timestep, book-committed at `03-preconditioning.md` line 57 + line 106) is allowed and expected. State readable in a later `evaluate(θ)` call is forbidden; Phase-E cross-call cache wrapper is the escape hatch (separate `PatternCachedPreconditioner<P>` type, separate contract, own determinism argument).

- **Why not idempotent-re-setup (earlier recommendation, self-revised).** Initial recommend-first call allowed cross-call state conditional on "`setup(opA)` then `setup(opB)` is output-equivalent to fresh-construct-and-`setup(opB)`." Self-stress-test under user's "prioritize risk mitigation" steer surfaced three risks: (a) pattern-cache-drift in an impl that claims idempotent-re-setup but fails (cache keyed on nnz instead of sparsity pattern, invalidation misses a topology-change case) lands within §03's 5-digit band and is not caught — silent weakening; (b) third-category leakage toward "preserves conditional on impl discipline," adjacent to the banned "may weaken later" category; (c) departs from B.3/B.4's uniform "forbid cross-call output-affecting mutation" discipline without load-bearing reason beyond performance pre-admission.
- **Why not allow, rely on §03 gradcheck as safety net.** §03 compares CPU-vs-GPU at 5-digit tolerance; cannot distinguish "fresh rebuild per call" from "cached pattern that's mostly right." Blind spot at the correctness-vs-performance boundary. Contract-level enforcement closes the blind spot.
- **Why not defer to Phase E.** B.5.a pre-load flagged the temptation explicitly; deferring reintroduces the silent-footgun risk the pre-load was raised to surface.
- **Book commits intra-call only.** Line 57 "across Newton iterations within a timestep" + line 106 "pattern cached across Newton iterations, rebuilt on re-mesh" both scope to one `evaluate()` call. Cross-call pattern reuse is NOT book-committed — Phase-E temptation only. Forbidding at the trait contract preserves book as-is, no walk-back.
- **Escape hatch:** `PatternCachedPreconditioner<P>` wrapper — if Phase-E scene-scale benchmarks show per-`evaluate` full AMG rebuild unworkable, a wrapper type concentrates cross-call state in one auditable location with its own determinism argument. Pass 1 doesn't ship it; the trait contract holds pending Phase E benchmark. Same escape-hatch template as B.4's Phase-H Brownian extension — future extension, not pre-ratified.
- **Audit-habit symmetry.** B.3 `Differentiable` forbade cross-call mutation; B.4 `Solver::step` forbade output-affecting cross-call mutation; C.2 `Preconditioner` forbids output-affecting cross-call mutation. Uniform mutation-discipline pattern across chassis-owned + sim-soft-owned impl-side traits at trait-contract level.

**C.2 → Queued Rust-side docstring commitment.** `sim_ml_chassis::gpu::Preconditioner` trait docstring lands with chassis GPU module creation PR at Phase E per B.5's build sequence. Not a new separate PR.

**C.2 → One queued book edit (landed inline as part of C.2 decision).** §80 Ch 02 §03 `03-preconditioning.md` — new "Cross-call discipline" subsection between "The preconditioner contract" and "What this sub-leaf commits the book to," plus one summary bullet in the commitment list.

**C.2 does not lock:**
- Existence of a `PatternCachedPreconditioner<P>` wrapper at Phase E — benchmark-driven decision, not locked now. If built, wrapper carries its own determinism argument and this C.2 decision doesn't pre-ratify it.
- Specific AMG pattern-cache keying (on `op.pattern()`, on `(nnz, row_ptr_hash)`, etc.) — Phase-E implementation detail. Trait contract commits only to "intra-call pattern reuse is book-committed and allowed" without specifying cache structure.
- Whether other B.4 cross-cutting-audit-predicted "likely no docstring needed" traits (`Observable`, `SdfField`) actually need docstrings — per audit habit, surface findings per trait on its walk.

**C.3 — GpuTensorPool acquire-use contract (2026-04-22).** Hybrid API: `GpuTensorPool::acquire` is the default-safe zero-on-alloc path; `GpuTensorPool::acquire_uninit` is the explicit opt-out with a write-before-read kernel-author contract. Closes the B.5.a-flagged cross-call content-leakage risk at the API level — safe by default, explicit opt-in for hot-path write-every-element kernels.

- **Why not pure zero-on-alloc (Option A).** Real perf cost at scene scale: peak resident set × ≈50 acquires/Newton × 5–15 iter → 25–75% of Newton-step wall-clock at 300k-DOF scenes. Pure-write-output kernels (`copy`, fresh-output elementwise transforms, explicit `zero`) are trivially safe for uninit reuse; forcing them through zero-fill burns perf for no correctness gain.
- **Why not pure write-before-read (Option B).** Silent-footgun risk: a kernel reading uninitialized slot returns previous call's tenant data — if numerically small/zero-ish, gradcheck passes and the bug surfaces later when a high-magnitude previous value shows up. Gradcheck blind spot exactly parallel to C.2's pattern-cache-drift concern. Distributing safety-critical discipline across every kernel author fails the same "prioritize risk mitigation" steer that revised C.2.
- **Why not debug-build poison-pattern fill (Option D).** Re-opens the Layer 3 pattern-precedent risk declined at B.1.a/B.2 walk (chassis-runtime-checks-kernel-impls scope creep); NaN/0xDEADBEEF poison can mask real numerical issues if the pattern is not carefully chosen.
- **Why hybrid (Option C) wins.** Safe default closes the blind spot by construction; explicit opt-out is grep-able at the call site (`acquire_uninit` occurrences are the audit surface, not kernel internals). Matches Rust-idiomatic safe-default-plus-opt-out pattern (`Vec::new` / `Vec::with_capacity`, `MaybeUninit`). Two APIs map onto natural kernel taxonomy: accumulator / read-modify-write kernels (scatter-add, gradient buffers, atomic-add reductions, in-place `axpy`) use `acquire` (zeroed); pure-write-output kernels (`copy`, fresh-output elementwise transforms) use `acquire_uninit`.
- **Placement of the write-before-read kernel-author contract.** Lives alongside C.1's GPU VJP author contract at §80 Ch 04 §02 `02-vjp-api.md` — kernel authors read one location for all GPU determinism discipline. Pool API semantics documented at §80 Ch 03 §00 `00-recording.md`. Two-location placement symmetric to C.1's dual-surface approach (substance in one canonical location, cross-reference from the other).

**C.3 → Queued Rust-side docstring commitments.** Both acquire methods land with chassis GPU module creation PR at Phase E. `GpuTensorPool::acquire` — short docstring, default-safe semantics. `GpuTensorPool::acquire_uninit` — explicit write-before-read contract, cross-reference §80 Ch 04 §02.

**C.3 → Two queued book edits (landed inline as part of C.3 decision).**
- §80 Ch 03 §00 `00-recording.md` — new "Acquire-use contract" subsection between the existing "Tensor lifetime" section and "Dynamic-size tensors" section + one summary bullet in the commitment list.
- §80 Ch 04 §02 `02-vjp-api.md` — new GPU-specific bullet under the existing "author contract — determinism across calls" section, naming `acquire_uninit` reads without write-before-read as a forbidden mode.

**C.3 does not lock:**
- Whether `acquire_uninit`'s Rust API uses an `unsafe fn` signature (safety via runtime verification not possible; discipline is contract-documented). Leaning against `unsafe` — the contract is a determinism concern, not a memory-safety concern; conflating with Rust's `unsafe` semantics would dilute the meaning. Revisit if kernel-author misuse patterns surface at Phase E.
- Whether the pool's zero-fill is implemented via `wgpu::Queue::write_buffer` (CPU→GPU staging) or a compute-pass `zero` kernel dispatch. Phase-E implementation detail; latter is likely cheaper on device, former is simpler.

**C.4 — Current sim-gpu crate fate (2026-04-22).** Stays separate. sim-gpu remains a sim-core companion crate owning rigid-body GPU acceleration (SDF narrowphase + the GPU physics pipeline scaffold per `GPU_PHYSICS_PIPELINE_SPEC.md`); future `sim_ml_chassis::gpu` module (Phase E per B.5 build sequence) and future `sim_soft::gpu` module (Phase E per Part 4 Ch 05 §00) take their own homes. Three GPU concerns, three homes; no migration, no rename, no book edit required.

- **Why stays-separate.** sim-gpu's two subsystems (SDF-collision narrowphase via `GpuSdfCollision` trait dependency-inverted from `sim-core::sdf::gpu`, and the `pipeline/` FK/CRBA/RNE/integrate scaffold) are pure sim-core acceleration. No autograd surface, no `Tape`, no `ForwardMap` participation, no chassis dependency. The trait-in-core / impl-in-companion pattern (`GpuSdfCollision` defined at `sim-core::sdf::gpu::GpuSdfCollision`, implemented as `GpuSdfCollider` in sim-gpu) is identical to sim-mjcf / sim-urdf / sim-thermostat's relationship to sim-core — proven crate-split-by-concern pattern. R34 metaphor: sim-core is chassis, sim-gpu is one bolt-on; chassis-gpu (Phase E) and soft-gpu (Phase E) are different bolt-ons that don't displace it.
- **Why not rename `sim-gpu` → `sim-gpu-rigid` for clarity (Option B).** Premature. Disambiguation problem (three "GPU" things in the workspace) is hypothetical until chassis-gpu/soft-gpu exist; once they do (Phase E), namespace scoping already disambiguates — only sim-gpu lives at crate level with that name; the others are modules (`sim_ml_chassis::gpu`, `sim_soft::gpu`). Rename would touch Cargo.toml + ~24 files + CI + examples (`sim_gpu::GpuPhysicsPipeline` cited in `examples/sdf-physics/gpu/01-hockey/EXPECTED_BEHAVIOR.md`) for a problem nobody has hit. Cite-able revisit trigger: a reader at Phase E reaches for the wrong "GPU crate."
- **Why not collapse into `sim_ml_chassis::gpu` (Option C).** Hard rejection on layer/concern grounds. Chassis is autograd substrate; sim-gpu's rigid-body narrowphase has zero autograd story. Conflating them blends concerns: chassis would carry `wgpu::Device`-with-rigid-body-WGSL-kernels alongside `GpuTensor`/`GpuTape`/`VjpRegistry`, dragging two unrelated GPU concerns through one module. The chassis-already-imports-sim-core dep direction (`sim-ml-chassis/Cargo.toml:26`) doesn't help: chassis would inherit not just the kernel surface but the maintenance of `GpuPhysicsPipeline`'s scaffold-track and `GPU_PHYSICS_PIPELINE_SPEC.md`'s sim-core-physics roadmap — outside chassis's "algorithm chassis: traits, primitives, Competition runner" scope. Plus timing: sim-gpu ships today; chassis-gpu doesn't exist until Phase E. Migration would block working code on a non-existent module.
- **Why not freeze pending Phase E (Option D).** Violates the audit's explicit-defer-with-rationale rule. Three concerns / three homes is overdetermined by today's evidence: sim-gpu = rigid-body narrowphase + pipeline (sim-core companion, exists), `sim_ml_chassis::gpu` = autograd substrate (chassis module, Phase E), `sim_soft::gpu` = soft-body kernels (sim-soft module, Phase E). No information Phase E would surface that changes the call. Defer would silently leave sim-gpu in audit-untouched state with no cite-able trigger.
- **Triple-Device cost concern (stress-test, declined as blocker).** Worst case = Bevy + sim-gpu + chassis-gpu, three `wgpu::Device`s on one machine; cross-Device data exchange requires CPU readback (no GPU-GPU buffer share). Real but mitigated: per `110-crate/02-coupling/00-mjcf.md`, sim-soft ↔ sim-core coupling happens at the MJCF state-vector handshake (xfrc_applied, xpos), CPU-sync by design; the hot Newton loop stays single-Device per crate. sim-gpu's `context.rs:5-7` explicitly created its own Device to avoid Bevy contention — that design decision pre-dates chassis and remains correct. Document as a known design constraint; revisit only if a Phase E benchmark surfaces hot-loop multi-Device pain.
- **Determinism story does not propagate.** sim-gpu's rigid-body pipeline doesn't enter the soft-body autograd `ForwardMap(θ)` chain at all. Its determinism story is sim-core-step-deterministic (mujoco-equivalent), not per-`evaluate(θ)` autograd-deterministic. The B.5/B.5.a/C.1/C.2/C.3 GPU-cross-call-state pre-loads (chassis `VjpRegistry`, `Preconditioner`, `GpuTensorPool`) do not propagate to sim-gpu — separate determinism surface, no cross-reference needed at the trait-contract level. sim-gpu's existing CPU-vs-GPU parity tests live across `sim-gpu/src/collision.rs::trace_contacts_match_cpu` (narrowphase) and `sim-gpu/src/pipeline/tests.rs::t31_gpu_vs_cpu_trajectory` (full-pipeline trajectory), scoped to A.4 §4's tolerance band locally; `approx` is a sim-gpu dev-dep specifically for these per `sim-gpu/Cargo.toml:30`.
- **Stale "removed" claims (book-vs-code drift, inline-fixed with this decision).** Three engine docs predate the `b881b8be` (#143) re-introduction of sim-gpu via the GPU physics pipeline scaffold:
  - `sim/docs/ARCHITECTURE.md:572-576` — "GPU Acceleration (Removed)" header asserting "sim-gpu crate was removed in workspace trim (2026-03-19)."
  - `sim/docs/MUJOCO_REFERENCE.md:735-738` — "GPU path (removed): The sim-gpu crate was removed in workspace trim (2026-03-19)."
  - `sim/docs/MUJOCO_GAP_ANALYSIS.md:984-989` — "Implementation Notes: GPU Acceleration (Removed)" — though this one already cites `sim/L0/gpu/` and `GPU_PHYSICS_PIPELINE_SPEC.md` parenthetically, the prose still says "removed."
  All three contradict reality. Inline fix as part of C.4 follows A.3's precedent (engine-doc nalgebra-version drift fixed inline at decision time). Reword: drop the "(Removed)" header markers, restate as "removed in workspace trim 2026-03-19, then re-introduced via PR #143 with the GPU physics pipeline scaffold; current state per `GPU_PHYSICS_PIPELINE_SPEC.md`."

**C.4 → No queued docstring commitments.** sim-gpu carries no chassis-determinism trait surface; existing `GpuSdfCollision` trait in `sim-core::sdf::gpu` is already documented at the rigid-body-collision level and does not need a chassis-style cross-call-mutation docstring (no autograd participation). sim-gpu's `GpuContext` is a wgpu wrapper, not a determinism surface.

**C.4 → Three engine-doc inline fixes (landed inline as part of C.4 decision).**
- `sim/docs/ARCHITECTURE.md` §"GPU Acceleration (Removed)" — strike "(Removed)" header marker; reword to acknowledge re-introduction via PR #143.
- `sim/docs/MUJOCO_REFERENCE.md` §"GPU path (removed)" — same pattern.
- `sim/docs/MUJOCO_GAP_ANALYSIS.md` §"Implementation Notes: GPU Acceleration (Removed)" — same pattern.

**C.4 does not lock:**
- Whether sim-gpu's `pipeline/` module (Sessions 1–6 scaffold) ever ships to production or stays exploratory. `GPU_PHYSICS_PIPELINE_SPEC.md` is alive and tracked separately; that decision is downstream of sim-core's GPU-acceleration roadmap, not the soft-body refactor audit.
- Whether sim-bevy and sim-gpu eventually share a `wgpu::Device`. Bevy-render-contention investigation is out-of-audit-scope; sim-gpu's separate-Device design holds for now per its own commit's rationale.
- Whether sim-gpu's `GpuContext` and chassis-gpu's `GpuDevice` ever share infrastructure. Separate crates, separate concerns; unify only if Phase E or later benchmarks demand it. No pre-ratification.
- Future renames at Phase E if user-side disambiguation pain materializes — Option (B) revisit-trigger is not pre-ratified; if it fires, that PR carries its own naming-convention argument.

### Group D

**Status:** D.1, D.2, D.3, D.4 locked. D.5 close ratified (2026-04-22). Cross-cutting determinism audit complete across all D sub-decisions; post-commit sanity sweep for URDF API renames returned zero stale references. **Group D closed.**

**D.1 — `Solver` trait docstring + `replay_step` signature (2026-04-22).** Substance pre-locked by B.4; D.1 closes the signature and determinism-contract commitments as a single book edit at §110 Ch 01 `00-core.md`.

Signature:

```rust
fn replay_step(
    &self,
    x_prev: &Tensor<f64>,
    v_prev: &Tensor<f64>,
    theta: &Tensor<f64>,
    dt: f64,
) -> NewtonStep<Self::Tape>;
```

- **Why `NewtonStep<Self::Tape>` and not a new `BackwardEulerState`** (per B.4's sketched name). `NewtonStep` already carries `{x_n, factor, dr_dtheta}` per `50-time-integration/00-backward-euler/01-newton.md:55-59` — exactly the three fields a `CheckpointedStepVjp::vjp` back-substitutes against. Introducing a new named struct duplicates surface without payload difference; B.4's sketched name was "sketched", not canonical. The adjacent book reference at `03-time-adjoint/01-adjoint-state.md:55` to a non-canonical `BackwardEulerState` is pre-existing drift — separate post-D sweep candidate.
- **Why the `Tape` associated type parameter stays on the return type.** Keeps `step` / `replay_step` return type surface uniform. `replay_step`'s impl does not write a tape (no `&mut Tape` parameter), but the `NewtonStep<Tape>` return type nominally matches `step`'s for consumer ergonomics.
- **Trait docstring lands in three paragraphs matching `Differentiable`'s structure** (concrete-impls paragraph + `step` intra/cross-call mutation discipline + `replay_step` pure-function semantics). Cross-references: mutation-discipline symmetry with `Differentiable` (§below) and chassis `Preconditioner` (C.2); replay contract citation to `04-checkpointing/02-tradeoff.md`'s verbatim "Replay is bit-reproducible given the stored primal $(x, v, \Delta t)$"; RAII-scope framing to `02-implicit-function/01-linear-solve.md` tape-drop contract, refined to per-VJP-call scope per B.4's "tape-lifetime DOWN TO per-vjp-call" framing.

**D.1 → Revision-pass findings caught pre-commit (pattern #8 discipline).** Three technical-claim drift findings in the draft, all fixed before commit:
1. Quoted phrase `"replay is deterministic given stored primal + Δt"` was synthesized, not verbatim. Fixed to `02-tradeoff.md:49`'s "Replay is bit-reproducible given the stored primal $(x, v, \Delta t)$" + corrected path from `00-uniform.md` to `02-tradeoff.md`.
2. RAII-scope claim originally said replay's release "matches" `step`'s tape-drop. B.4 framed this as scope *refinement* (tape-lifetime → per-VJP-call), not matching. Fixed: "refined to per-VJP-call scope."
3. Tape-drop cite path originally pointed at `01-newton.md`; verbatim "released when the tape is dropped" lives at `01-linear-solve.md:30`. Fixed: redirected cite.

**D.1 → Book edit (landed inline as part of D.1 decision).** §110 Ch 01 `00-core.md`:
- Line 13 table entry (Solver row) updated to name `replay_step`.
- Solver section: `replay_step` added to trait code block; three new docstring paragraphs (two-impls retained + `step` mutation discipline + `replay_step` pure-function semantics).

**D.1 does not lock:**
- `BackwardEulerState` drift at `03-time-adjoint/01-adjoint-state.md:55` — separate finding; post-D sweep candidate.
- `NewtonStep<Self::Tape>` (trait surface, generic) vs `NewtonStep` (struct at `01-newton.md:55`, non-generic) drift — pre-existing book-internal inconsistency; post-D sweep candidate.
- Phase-E GPU `replay_step` variant details (GPU-resident factor handle vs on-device back-substitution before returning) — implementation detail, Phase E decision.

**D.2 — Trait-placement + mutation-discipline walk across remaining trait surfaces (2026-04-22).** Six trait surfaces walked; one book edit (D.2.a `ForwardMap` docstring); five no-docstring-needed findings (D.2.b–d).

**D.2.a — `ForwardMap` trait docstring (landed inline).** Book edit at `110-crate/02-coupling/03-ml-chassis.md` inserts two docstring paragraphs between the trait code block and the existing `EditResult` prose — same audit-habit pattern as B.3 `Differentiable`, D.1 `Solver`, C.2 `Preconditioner`. Paragraph 1 admits intra-call mutation (impl-field scratch, per-call tape-recording hooks, physics-rollout state; concrete "Solver / scene / Observable" example flagged as Phase D implementation choice, not trait-level commitment). Paragraph 2 forbids output-affecting cross-call mutation (warm-starts across θ evaluations, adaptive reward-scaling, accumulated per-call statistics); cites γ-locked determinism-in-θ contract + BayesOpt GP cache invariant + audit-habit symmetry across the four impl-side traits.

- **Closes a pre-existing book-vs-code gap, not just adds a new docstring.** `03-ml-chassis.md:61` already commits *"The chassis boundary surfaces this contract verbatim in the `ForwardMap` docstring."* — but the docstring did not exist until D.2.a. Fulfills a previously-unfulfilled book commitment.
- **Placement after line 41, not in §"Determinism-in-θ and the cached-tape contract" at line 57.** §"Determinism-in-θ" is the book-layer narrative contract; the new paragraphs are the rustdoc-layer type-surface commitment. Two different levels; §57 continues to elaborate (via the `(elaborated §below)` pointer).
- **Cross-references use file-level, no anchor fragments** per C.1's precedent (sidesteps mdbook em-dash anchor uncertainty).
- **Concrete impl example is illustrative, not ratified.** Paragraph 1 names "active `Solver`, scene/mesh, `Observable` readout" as expected impl fields; explicitly flagged as Phase D implementation choice, not a trait-level struct-layout commitment.

**D.2.b — `Observable` (no docstring needed).** `&self` only on all four methods (`stress_field`, `pressure_field`, `temperature_field`, `reward_breakdown`). Trait signature does not admit cross-call mutation. Interior-mutability-via-`RefCell` is a general Rust-idiom concern, not Observable-specific; catchable at the enclosing `ForwardMap` contract (D.2.a) because Observable is called from inside `ForwardMap::evaluate`. **Subtlety noted:** Observable is also consumed by `sim-bevy` (visualization) per `03-ml-chassis.md:67`; visualization has no determinism-in-θ requirement. If a future non-optimization consumer of Observable needs its own determinism contract, that's their concern — not Observable's trait-level contract. Finding holds: determinism-in-θ is an optimization-pipeline concern, transitively enforced at ForwardMap's boundary.

**D.2.c — `Material` / `Element` / `Mesh` / `ContactModel` (no docstring needed).** All four are `&self`-only data-evaluation traits (constitutive eval, shape-function eval, mesh connectivity queries, contact-pair eval). `Send + Sync` bounds signal immutable consumption. Same argument as Observable per B.4's audit-habit-not-template rule.

**D.2.d — `SdfField` (not a Rust trait; no placement decision).** Per `04-cf-design.md:9` + `:82`: `SdfField` is a struct composed of `Box<dyn Sdf>` + `Aabb3` + `f64`. The trait pair `Sdf` + `Field<Output = T>` lives in cf-design / cf-geometry, not sim-soft. Sim-soft consumes the boxed trait object; no D.2 trait-placement question.

**D.2 → Book edit (landed inline as part of D.2 decision).** `110-crate/02-coupling/03-ml-chassis.md` — two `ForwardMap` docstring paragraphs inserted between the trait code block (lines 22-38) and the existing `EditResult` prose (line 43, pre-edit).

**D.2 does not lock:**
- Concrete `ForwardMap` impl struct layout in sim-soft — Phase D implementation choice.
- Whether Observable's docstring needs strengthening if a future non-optimization consumer surfaces a determinism requirement. Per B.4 audit-habit-not-template rule, revisit per trait on its own walk, not pre-commit now.
- Whether `sim-soft::sdf_bridge/` introduces its own trait surface (distinct from cf-geometry's `Sdf`) — reserved to Phase G's `sdf_bridge/` impl scope, not a D.2 decision.

**D.3 — Coupling patterns walk across sim-soft ↔ sim-core and sim-soft ↔ sim-thermostat (2026-04-22).** Both couplings audit-correct as written; no book edits. Three findings + one post-D sweep candidate + one Phase E flag.

**D.3.a — `MjcfHandshake` no mutation-discipline docstring needed.** Trait signature at `02-coupling/00-mjcf.md:43-46` is asymmetric: `import_rigid_state(&mut self, &[RigidBoundary])` admits per-handshake-tick mutation by design (stores rigid BC for the next Newton solve); `export_contact_reactions(&self) -> Vec<ContactReaction>` is `&self`-only. The `&mut self` mutation is load-bearing across handshake ticks *within one `ForwardMap::evaluate(θ)` call* — not a candidate for "forbid cross-call mutation" phrasing because "cross-call" at method level (per-handshake-tick) is *allowed*, and "cross-call" at `evaluate` level is `ForwardMap`'s contract (D.2.a), not `MjcfHandshake`-local. Transitively inherits discipline per D.2.b's Observable argument. Book commits (line 28 + line 81 + Phase F commitment) that the handshake consumer is the ForwardMap-orchestrated outer coupling loop; no Phase A–I standalone consumer.

**D.3.b — `ThermostatHandshake` no mutation-discipline docstring needed.** Same shape as MjcfHandshake at `02-coupling/01-thermostat.md:36-40`: one `&mut self` importer (`import_temperature(TemperatureView)`) + one `&self` exporter (`export_dissipation() -> DissipationReport`). Same transitivity argument.

**D.3.c — Multi-Device CPU-sync claim from C.4 verified across both couplings.** Both coupling surfaces exchange CPU-resident boundary data at handshake ticks:
- MjcfHandshake: `RigidBoundary` (Pose + SpatialVec, CPU struct) in; `ContactReaction` (Wrench + centroid, CPU struct) out.
- ThermostatHandshake: `TemperatureView<'a>` (trait object `&'a dyn Field<Output = f64>`, CPU-resident) in; `DissipationReport` (`Vec<f64>`, CPU) out.

sim-thermostat is CPU-only today (no wgpu dep per inventory Cargo.toml verification, pattern #8 discipline). sim-core's rigid pipeline consumes sim-gpu's `wgpu::Device` when GPU-accelerated; sim-soft's Phase-E Newton consumes chassis-gpu's `wgpu::Device`. All cross-crate transit is CPU-resident. C.4's "hot Newton loop stays single-Device per crate" extends cleanly — no D.3 amendment to C.4.

**D.3 → Post-D sweep candidate (not a D.3 fix).** Part 5 Ch 03 §00 (`50-time-integration/03-coupling/00-mujoco.md:45-51`) has method-level `///` lifecycle docstrings on `MjcfHandshake` ("Called by the outer coupling loop before sim-soft advances"; "Called after sim-soft's Newton step converges..."). Part 11 Ch 02 §00 (`02-coupling/00-mjcf.md:43-46`) has none on the same trait. Book-internal docstring drift; candidate for the post-D sweep alongside `BackwardEulerState` drift (D.1) and `NewtonStep<Self::Tape>` generic-vs-non-generic drift (D.1). Not a D.3 fix because D.3 scope is mutation-discipline, not lifecycle-docstring consistency.

**D.3 → Phase E flag (not a D.3 concern).** `01-thermostat.md:42` commits "lazily per-Gauss-point during the `Thermal<M>` decorator's call chain" — a CPU access pattern. Phase-E GPU Newton needs a material-eval strategy for getting `Field` samples onto GPU (likely pre-sample at handshake time, upload scratch to GPU for the Newton loop). sim-soft material-eval concern, not coupling-trait concern; D.3 does not block.

**D.3 does not lock:**
- `sim-bevy` coupling walk — not in D.3 scope per audit doc's "sim-core / sim-thermostat" primary-crate naming. sim-bevy is an Observable-consumer (D.2.b), not a handshake partner; no coupling trait to audit. If a future sim-bevy surface surfaces a new cross-crate concern, walk then.
- Whether subcycling bookkeeping inside `coupling/` (time-averaging for faster-thermostat, time-integration for faster-soft-body) needs its own mutation-discipline audit at the sub-module level. Not a trait-level concern; module-level discipline enforced by code review during Phase F implementation.
- Fixed-point retry-cap behavior at `00-mjcf.md:24` (`handshake-uncontracted` signal triggering adaptive-Δt escape). Determinism preserved per line 74; behavior audit is a Phase F implementation concern.

**D.4 — IO ingress: URDF geometry NaN/Inf validation (2026-04-22).** A.4 §1's validate-at-system-boundary commitment reaffirmed; one code change inline per A.3 / A.4 §1 precedent. First code change of the audit (D.1–D.3 were all book edits). Substance pre-locked by Group D entry inventory.

**D.4.a — URDF geometry NaN/Inf gap closed inline.** `sim/L0/urdf/src/validation.rs` systematically checked mass at L209 (`!inertial.mass.is_finite()`) and inertia at L223-228 (all six tensor components), but geometry numeric fields (`Box.size`, `Cylinder.radius`/`length`, `Sphere.radius`, `Mesh.scale`) parsed into `UrdfGeometry` variants bypassed `.is_finite()` post-parse. MJCF parser was the comparison baseline (validates more at the boundary); URDF was the odd one out.

Pre-commit recon (run before inline-fix decision):
- Existing URDF fixture corpus (`sim/L0/tests/assets/mujoco_menagerie/google_barkour_{v0,vb}.urdf`, Google-authored): zero `NaN|Inf` tokens. Zero false-positive risk on existing inputs.
- Downstream coverage: `sim/L0/urdf/src/converter.rs` has no `.is_finite()` calls; `sim/L0/mjcf/src/validation.rs` checks option/mass/inertia fields but `validate_geom` / `geom.*finite` / `size.*finite` searches return zero matches. URDF geometry NaN today propagates past both validation boundaries unchecked.

Fix (inline, committed): new `InvalidGeometry { link_name, message }` variant in `sim-urdf::error` matching the `InvalidMass` / `InvalidInertia` pattern (not `InvalidAttribute`, which is parse-time semantic); new `validate_geometry` + `check_geometry_finite` functions in `sim-urdf::validation`, called from `validate()` after `validate_mass_properties`. 6 new unit tests (one per variant + visual-path detection + valid-geometry passes). Grader A across all automated criteria (Coverage 78.7%, Documentation / Clippy / Safety / Dependencies / Bevy-free all A).

Positivity (negative radius, zero box size) deferred per `validate_mass_properties` L239-242 leniency precedent: `if i.ixx + i.iyy < i.izz ... { /* This is a warning-level issue, not an error / Many URDFs have slightly invalid inertias */ }` — existing codebase consciously tolerates "technically invalid" geometry shapes in real-world URDFs. Bundling positivity risks newly-rejecting inputs that load today. Positivity becomes its own audit item if the fixture corpus grows to need it.

**D.4.b — Grader package-scoping bug surfaced (symptom fixed inline; root cause queued as F.1).** Initial `cargo xtask grade sim-urdf` reported F on clippy with 14 warnings despite sim-urdf's own source being clippy-clean. Root cause: grader at `xtask/src/grade.rs:834` invokes `cargo clippy -p {crate_name} --all-targets --all-features --message-format=json` and the JSON filter at L844-862 counts `compiler-message + warning-level + non-empty-spans` without checking the diagnostic's originating package. Transitive workspace dep warnings (cf-geometry 11, sim-core 1, sim-mjcf 2 on sim-urdf's dep chain; mesh-types 3 surfaced after the first pass) count against `-p sim-urdf`'s clippy criterion. Every workspace-member warning breaks every dep-chain crate's grade — fragile.

Symptom fix (separate hygiene commit): workspace clippy auto-fixes across 4 crates — `cf-geometry/src/{bvh,convex_hull,mesh}.rs` (11 × `missing_const_for_fn`), `sim-core/src/mesh.rs` (1 × `manual_is_multiple_of`), `sim-mjcf/src/builder/{composite,mesh}.rs` (2 × `manual_is_multiple_of`), `mesh-types/src/attributed.rs` (3 × `missing_const_for_fn`). Workspace compiles; 347 tests pass on affected crates. Adjacent-crate-flag fix per `feedback_adjacent_crate_flags.md`.

**F.1 queued (audit-driven, Group F entry).** Filter `grade_clippy`'s JSON parse by `message.spans[*].file_name` against the target package's `src/` path, so only diagnostics originating in the graded crate count. Natural home: Group F "build/test/CI infrastructure" scope, specifically `xtask grade readiness for a new physics crate` per the group's table-row. Not D.4 scope — D.4 fix-inline was for symptoms; F.1 is structural.

**D.4 → Post-D sweep candidate (MJCF sibling gap).** `sim-mjcf::validation` validates option / mass / inertia / gravity NaN/Inf systematically but NOT `MjcfGeom` size/radius/length numeric fields. Same class of gap as URDF's, one crate over. Candidate for follow-up PR or post-D sweep alongside the three already-queued book-internal drift items (`BackwardEulerState` cite drift, `NewtonStep<Self::Tape>` generic-vs-non-generic drift, `MjcfHandshake` lifecycle-docstring asymmetry).

**D.4 does not lock:**
- URDF geometry positivity (zero/negative radius etc.) — deferred per `validate_mass_properties` L239-242 leniency precedent; separate audit-item candidate if the fixture corpus grows.
- MJCF `MjcfGeom` NaN/Inf gap — post-D sweep candidate; orthogonal-crate.
- F.1 grader package-scoping fix implementation — queued for Group F proper.

### Group E

**Status:** E.1, E.2, E.3, E.4 locked. E.5 close ratified (2026-04-22). **Group E closed.** Cross-cutting determinism audit complete across E.1–E.4 consumer integrations; post-commit sanity sweep at E.4 close surfaced 1 finding (line 590 same-class drift, fixed in follow-up commit `d8f657fc`).

**Close-out summary.** Four consumer/chassis-internal walks placed across three PR-sequencing categories: **independent** (E.1 sim-opt at `742af0f7` + E.4 sim-rl at `1c6a94a9` — zero forcing function on chassis-refactor PR boundary), **bundled-consumer** (E.2 sim-therm-env at `4acf0062` — `impl Environment for ThermCircuitEnv`'s method signatures rename with chassis `Environment` trait at compile boundary), **bundled-internal** (E.3 autograd modules at `9288d696` — modules live inside the chassis being refactored). Cadence shifted by walk shape: argue-and-refine for placement decisions (E.1 / E.2 / E.4 — well-trodden CEM/REINFORCE/PPO/TD3/SAC surfaces); recommend-first for autograd-internals specialist depth (E.3); info-asymmetry override applied throughout. Option-B verification cadence (grade + broader inventory grep) applied before each lock, grep direction flipping by walk shape (consumer→upstream for E.1/E.2/E.4 external crates; upstream→downstream for E.3 chassis-internal walk). **Hygiene-commit-then-decision pattern firmly conditional on F-class trigger**: 3 of 5 Group-D-and-E walks applied (D.4.b + E.1 + E.2), 2 of 5 clean-grade (E.3 + E.4 both A across all 6 criteria). Pattern is conditional, not default-apply. A.1-line-78 prediction reconciled in the post-walk footnote at line 79 — three overstatements + one hold across all four named consumers; "mostly mechanical" remains accurate in spirit, mechanical surface smaller than predicted. **Implementation-PR-shipping-strategy decision queued as Group F sub-item** (per user direction at E.5 entry: audit branch ships as one squash PR locked; FUTURE implementation PR shape — one big bundle vs. split — decided during Group F walk when more information is available).

**E.1 — sim-opt breaking-change walk (2026-04-22).** sim-opt is **not affected by any locked A/B/C/D decision at its public API surface.** Zero code changes required for migration; ships independently of any future chassis refactor PR.

**Inventory** (Cargo.toml dep direction verified per pattern #8):
- Prod deps: `sim-ml-chassis`, `sim-thermostat`, `rand`, `serde`, `thiserror`.
- Dev deps: `sim-core`, `sim-mjcf`, `sim-rl`, `approx`.
- Chassis surfaces consumed (prod): `Algorithm`, `ArtifactError`, `Competition`, `CompetitionResult`, `CURRENT_VERSION`, `EnvError`, `EpochMetrics`, `Policy`, `PolicyArtifact`, `TaskConfig`, `TrainingBudget`, `TrainingCheckpoint`, `VecEnv`, `collect_episodic_rollout`. Dev-only adds (via sim-rl re-exports + chassis directly): `Activation`, `NetworkKind`, `PolicyDescriptor`, `RunResult`, `LinearPolicy`, `reaching_2dof`, `ActionSpace`, `ObservationSpace`, `Cem`, `CemHyperparams`. `sim-thermostat::prf::splitmix64` is the only non-chassis prod surface.

**Breaking-change intersection (explicit yes/no per locked decision):**

| Locked decision | Touches sim-opt? | Why |
|---|---|---|
| A.1 `Tensor<T>` generic | **No.** | Zero `Tensor` sites in sim-opt src + tests (narrow + broader grep against full chassis export list). All Tensor contact hidden behind `VecEnv` opacity + `collect_episodic_rollout`'s `&[f32] → Vec<f64>` closure boundary. A.1's sub-decision B-2 keeps `VecEnv`/`Policy`/`TensorSpec` f32-concrete (no type-parameter threading); insulation is structural, not coincidental. |
| A.2 faer | No | Gradient-free; no factor-on-tape; no faer dep candidate. |
| A.3 nalgebra | No | Zero nalgebra usage in sim-opt. |
| A.4 IEEE754 / NaN / tolerances | No surface changes | sim-opt already compliant: fitness = f64 scalar reductions, no NaN/Inf system boundary, no centralized tolerance leak. |
| B.1 vector-aware tape | No | Zero `Tape` usage (grep). Gradient-free. |
| B.1.a / B.2 `VjpOp` / `push_custom` | No | Zero `register_vjp` / `push_custom` usage. |
| B.3 `Differentiable` placement | No | Zero `Differentiable` / `DifferentiablePolicy` usage; SA / richer-SA / PT operate on `Policy` base trait only. |
| B.4 checkpointing / time adjoints | No | SA / PT have no time-adjoint surface; gradient-free Metropolis loop. |
| B.5 / B.5.a GPU tape / `GpuScalar` | No | CPU-only (no wgpu in deps). |
| C.1–C.4 GPU cross-call state | No | CPU-only. |
| D.1 `Solver::replay_step` | No | Zero `Solver` usage. |
| D.2.a `ForwardMap` purity | No | Zero `ForwardMap` usage. |

**Migration path:** zero code changes. Public API unchanged.

**PR sequencing:** independent. sim-opt does not require bundling with any chassis refactor PR. Can ship before, during, or after upstream A.1 refactor without consumer-side intervention.

**Pre-decision verification (Option B per E.1 risk-mitigation walk).**
- **Ground-truth grade baseline** via `cargo xtask grade sim-opt` (per `feedback_ground_truth_via_tool.md`): pre-hygiene F on Clippy (8 warnings — 2 sim-opt-own at `sim/L0/opt/src/analysis.rs:631` `manual_is_multiple_of` + 6 transitive-bleed via F.1 grader-scoping bug: `sim-ml-chassis/rollout.rs` len/is_empty `missing_const_for_fn`, `sim-ml-chassis/tensor.rs` ndim/len/is_empty `missing_const_for_fn`, `sim-rl/td3.rs:440` `manual_is_multiple_of`). Post-hygiene `f614cd77` re-grade: A across all automated criteria (Coverage 94.2% A+, Documentation A, Clippy A, Safety A, Dependencies A, Bevy-free A).
- **Broader inventory grep** against the full `sim_ml_chassis::lib.rs` re-export list (~50 identifiers): zero additional surfaces beyond the locked-decision keyword sweep. `Activation` and `NetworkKind` confirmed pure-enum (no Tensor/Tape exposure); used in `analysis.rs` `mock_run_result` test helper only. `ActionSpace` / `ObservationSpace` consumed via builder pattern in test fixtures; the Tensor-typed `.apply()` / `.extract()` methods are not called from sim-opt.

**E.1 → Group F queued item (F.2 candidate, audit-driven).** Chassis refactor PR pre-merge validation. When chassis A.1 (or any change touching `VecEnv` / `Policy` / `Algorithm` internals) ships, re-run sim-opt's `#[ignore]`-gated `d2c_sr_rematch{,_richer_sa,_pt}.rs` fixtures with `--release` to verify `(best_outcome, final_outcome)` classifications against the published Ch 51-55 study's pre-squash tag (`ml-chassis-post-impl-pre-squash`). Cost: ~30-60 min per fixture × 3 = ~1.5-3h on MBP. Catches silent RNG-consumption reorder that mechanical-rename refactor shouldn't introduce but could; option C from the E.1 risk-mitigation walk, deferred to natural validation timing rather than paid pre-emptively. Natural home: Group F build/test/CI infrastructure scope.

**E.1 → Audit-doc revision-pass finding (caught pre-commit, pattern #8 discipline).** A.1's "Downstream work (Group E)" paragraph at `platform_refactor_soft_body_ready.md` line 78 reads "every RL consumer gains a type parameter at `Tensor` sites (`sim-rl`, `sim-opt`, `sim-therm-env`, `AutogradPolicy`/`AutogradValue` forward signatures)." Overstates for sim-opt specifically: zero Tensor sites in sim-opt src + tests means the "type parameter at Tensor sites" claim has zero applicable sites in sim-opt. The remaining named consumers (sim-rl, sim-therm-env, `AutogradPolicy`/`AutogradValue`) are walked at E.2–E.4; A.1's prediction stands for those (TBD per their walks). Inline edit deferred to Group E close (E.5) — annotating one consumer's audit entry inline at A.1 is awkward; a single E-close clarification footnote is cleaner.

**E.1 does not lock:**
- E.2 sim-therm-env walk — separate; ThermCircuitEnv is integration-heavy with a different breaking-change profile.
- F.1 grader-scoping fix — Group F.
- F.2 chassis-refactor pre-merge fixture-regression validation — queued for Group F per the heads-up above.
- Any sim-opt internal refactoring (the crate is grade-A and the dual-metric pipeline shipped per memory PR #190; out of scope).

**E.2 — sim-therm-env breaking-change walk (2026-04-22).** sim-therm-env is **affected by A.1 only — mechanical rename at 4 prod `Tensor` sites + ~28 test-site references.** All other locked A/B/C/D/E.1 decisions: no. Migration bundles with the chassis A.1 refactor PR (compile-boundary forced). Two new audit-item findings surfaced during the walk (Coverage F structural, A.4 §1 builder-setter compliance gap); both queued as separate items outside E.2 decision scope.

**Inventory** (Cargo.toml dep direction verified per pattern #8):
- Prod deps: `sim-core`, `sim-mjcf`, `sim-thermostat`, `sim-ml-chassis`, `thiserror`.
- Dev deps: `approx`, `sim-rl`, `sim-opt`.
- Chassis surfaces consumed (prod): `Environment`, `SimEnv`, `VecEnv`, `ActionSpace`, `ObservationSpace`, `Tensor`, `StepResult`, `ResetError`, `EnvError`, `SpaceError`. Dev-only adds (via `sim-rl` + chassis directly + `sim-opt` fixtures): `Algorithm`, `TrainingBudget`, `Policy`, `LinearPolicy`, `LinearValue`, `OptimizerConfig`, `Cem`/`CemHyperparams`, `Ppo`/`PpoHyperparams`, `Sa`/`SaHyperparams`, `RicherSa`/`RicherSaHyperparams`. Non-chassis prod surfaces: `sim_core::{DVector, Data, Model, StepError}`, `sim_mjcf::{load_model, MjcfError}`, `sim_thermostat::{LangevinThermostat, PassiveComponent, PassiveStack}`.

**Breaking-change intersection (explicit yes/no per locked decision):**

| Locked decision | Touches sim-therm-env? | Why |
|---|---|---|
| A.1 `Tensor<T>` generic | **Yes — mechanical.** | 4 prod `Tensor` sites in `env.rs` (the `impl Environment for ThermCircuitEnv` block — `observe() -> Tensor`, `step(&Tensor) -> ...`, `reset() -> Result<Tensor, _>`) + ~28 test-site references across `tests/{phase2,phase3,experiment_1,experiment_4,ising_chain}.rs`. All become `Tensor<f32>` per A.1 sub-decision B-2 (chassis `Environment` trait stays f32-concrete; consumer impl signatures rename mechanically). No shape change, no new generics threaded through `ThermCircuitEnv`'s builder API. |
| A.2 faer | No | No linear-algebra factoring; no faer dep candidate. |
| A.3 nalgebra | No | nalgebra consumed only transitively via `sim_core::DVector` (one call site in `builder.prepare()` for `gamma_vec`); no direct nalgebra import. |
| A.4 IEEE754 / NaN / tolerances | No E.2-scope surface changes | Internal arithmetic (Langevin dynamics, reward/done/truncated closures) is within A.4 §1's "propagate internally" rule. Builder-setter `.is_finite()` compliance is a **separate** finding, queued below — not an E.2 migration-path change. |
| B.1 vector-aware tape | No | Zero `Tape` / `Var` usage. sim-therm-env is gradient-free at the env surface; tape concerns live at upstream RL/opt consumers. |
| B.1.a / B.2 `VjpOp` / `push_custom` | No | Zero custom-VJP surface. |
| B.3 `Differentiable` placement | No | Zero `Differentiable` / `DifferentiablePolicy` / `CpuTape` usage. |
| B.4 checkpointing / time adjoints | No | No time-adjoint surface; env is one-step-at-a-time via `Environment::step`. |
| B.5 / B.5.a GPU tape / `GpuScalar` | No | CPU-only; zero wgpu in prod or dev deps (verified via Cargo.toml, pattern #8). |
| C.1–C.4 GPU cross-call state | No | CPU-only (same as B.5). sim-thermostat also CPU-only per D.3.c verification; no C-surface propagates through the builder's physics wiring. |
| D.1 `Solver::replay_step` | No | Zero `Solver` usage. |
| D.2.a `ForwardMap` purity | No direct surface | sim-therm-env does not impl `ForwardMap`. Its `Environment::step`/`observe`/`reset` is consumed transitively inside `collect_episodic_rollout` called from sim-opt's `evaluate_fitness` and sim-rl's training loops (same transitive pattern as E.1 sim-opt). D.2.a's trait-surface commitment is enforced at upstream ForwardMap impl sites, not at the Environment impl. |
| D.2.b `Observable` | No | sim-therm-env does not impl `Observable`. If future visualization support adds one, that's its own walk. |
| D.3 coupling traits (`MjcfHandshake` / `ThermostatHandshake`) | No | sim-therm-env uses sim-thermostat's **internal** physics-wiring surfaces (`LangevinThermostat`, `PassiveComponent`, `PassiveStack`) inside its own env construction — distinct from the `ThermostatHandshake` sim-soft↔sim-thermostat coupling trait. Zero handshake-trait usage (grep). |
| D.4 IO ingress (URDF / MJCF NaN/Inf) | No direct | MJCF ingress is sim-mjcf's boundary; sim-therm-env is a user of `sim_mjcf::load_model`. D.4's URDF fix (shipped) + the queued post-D MjcfGeom sweep candidate cover the ingress-side. sim-therm-env's own user-facing builder setters are a **separate** finding, queued below. |
| E.1 sim-opt zero-changes | No | sim-opt is dev-only in sim-therm-env; sim-opt's own zero-change walk means zero propagated impact. |

**Migration path: mechanical rename only.** 4 prod sites in `src/env.rs` + ~28 test sites rename `Tensor` → `Tensor<f32>`. Builder API (`timestep`, `gamma`, `k_b_t`, `seed`, `with`, `with_ctrl_temperature`, `ctrl_range`, `sub_steps`, `episode_steps`, `reward`, `done`, `truncated`, `on_reset`, `build`, `build_vec`) untouched — no Tensor exposure. Internal `PreparedCircuit` struct untouched. No new generics threaded. No error-type changes. No closure-signature changes.

**PR sequencing: bundled with the chassis A.1 refactor PR (compile-boundary forced).** `impl Environment for ThermCircuitEnv`'s method signatures are pinned to chassis's `Environment` trait signature at the compile boundary. Splitting into separate PRs requires staged compat aliases on chassis that pay coordination cost for mechanical-rename gain. Contrast with E.1 sim-opt (independent, zero-change). E.3/E.4 RL walks likely face the same bundle-forced constraint if they impl chassis traits.

**Pre-decision verification (Option B, E.1-established cadence).**
- **Ground-truth grade baseline** via `cargo xtask grade sim-therm-env` (per `feedback_ground_truth_via_tool.md`): pre-hygiene F on Dependencies (8 deps unjustified — sim-therm-env-own Cargo.toml gap, not F.1 transitive bleed) + F on Coverage (0.0%, structural; see finding below). Post-hygiene `3a45d50f` re-grade: Dependencies A, Documentation A, Clippy A, Safety A, Bevy-free A. Coverage remains F (queued).
- **Broader inventory grep** against the full `sim_ml_chassis::lib.rs` re-export list (~50 identifiers) + sim-core/sim-mjcf/sim-thermostat export lists: **zero additional locked-decision surfaces beyond A.1's Tensor sites.** 10 chassis surfaces consumed in prod src total (`Environment`, `SimEnv`, `VecEnv`, `ActionSpace`, `ObservationSpace`, `Tensor`, `StepResult`, `ResetError`, `EnvError`, `SpaceError`); 9 of the 10 are infrastructure (envs, spaces, errors, step results) that don't intersect any locked A/B/C/D decision. Additional chassis surfaces in dev-test fixture suites (`Algorithm`, `TrainingBudget`, `Policy`, `LinearPolicy`, `LinearValue`, `OptimizerConfig`) are fixture-scoped, not prod; they don't widen the breaking-change intersection because sim-therm-env's prod migration path stays isolated from test-only consumption. `LinearPolicy` prod-src hit at `builder.rs:166` is a docstring reference, not a code import.

**E.2 → Two new audit-item findings (outside E.2 decision scope; queued).**

- **Coverage F structural (Group F candidate).** sim-therm-env has zero `#[cfg(test)]` modules in `src/` — unique among L0 crates (sim-core 50, sim-mjcf 23, sim-ml-chassis 22, sim-thermostat 16, sim-urdf 6, sim-rl 5, sim-opt 4, sim-types 4, **sim-therm-env 0**). All 31 passing tests live in `tests/*.rs` (integration). Grader uses `cargo llvm-cov --lib --release` by design per its own docstring at `xtask/src/grade.rs:640-642` (integration tests cost ~10× with instrumentation), so Coverage reports 0.0%. **Not an F.1-class grader bug** — grader behaves as documented; the gap is that sim-therm-env's test-organization pattern does not match the grader's assumptions. Candidate fix paths: (a) add inline unit tests to `src/env.rs` / `src/builder.rs` (~200–500 LOC code-authoring work); (b) relax grader to also count integration-test-derived coverage on a per-crate opt-in basis; (c) accept F-coverage structurally with an explicit carve-out for all-integration-test crates. Group F decision point (alongside F.1 grader package-scoping and F.2 fixture-regression validation).
- **A.4 §1 compliance gap at sim-therm-env builder setters.** `ThermCircuitEnvBuilder` setters (`timestep(f64)`, `gamma(f64)`, `k_b_t(f64)`, `ctrl_range(f64, f64)`) accept user-supplied floats without `.is_finite()` checks. These ARE A.4 §1 "user-facing constructors" per the locked policy. Same class as D.4 (URDF geometry NaN/Inf, fixed inline at D.4 close) and the queued post-D MjcfGeom sweep candidate. Scope: additive validation in `ThermCircuitEnvBuilder::prepare()` (already `Result`-returning via `build()`/`build_vec()`), new typed-error variants on `ThermCircuitError`, zero-API-break (setters stay `const fn`). ~50–100 LOC including unit tests. Natural home: post-E / pre-skeleton A.4 §1 compliance sweep PR, bundling sim-therm-env's builder gap with the MjcfGeom ingress gap. **Not** clustered with URDF's inline fix — URDF was D.4's primary decision scope; sim-therm-env + MjcfGeom are adjacent findings each with their own scope.

**E.2 → Audit-doc line-78 prediction update (A.1 downstream work).** A.1's "Downstream work (Group E)" paragraph named sim-therm-env among the consumers gaining a type parameter at `Tensor` sites. E.2 confirms: **prediction holds for sim-therm-env** (4 prod + ~28 test sites; mechanical). E.1 caught the overstatement for sim-opt specifically. Inline edit to line 78 still deferred to E.5 close (footnote clarification that the prediction is per-consumer, with sim-opt at zero sites and sim-therm-env at ~32 mechanical sites).

**E.2 does not lock:**
- Coverage F resolution path (a / b / c options above) — Group F decision; new-crate onboarding + grader-convention conversation.
- A.4 §1 builder-setter fix timing and scope bundling — queued as audit item; candidate PR is post-E / pre-skeleton A.4 §1 compliance sweep alongside MjcfGeom.
- E.3 `autograd_policy` / `autograd_layers` walk — likely heavier Tensor exposure than sim-therm-env (forward signatures named in A.1 line 78); recommend-first cadence per `feedback_recommend_first_deep_specialist.md`.
- E.4 sim-rl baselines walk — RL algos (Cem / REINFORCE / PPO / TD3 / SAC) have richer chassis-surface consumption than sim-therm-env; A.1 intersection expected wider; bundle-forced PR sequencing likely holds.
- Any sim-therm-env internal refactoring beyond the A.1 mechanical rename — the crate is architecturally sound; builder pattern is proven (31 fast-passing + 22 `#[ignore]`-gated heavy integration tests; 4 experiments validated per memory `project_therm_circuit_env.md`).

**E.3 — autograd_policy / autograd_value / autograd_layers breaking-change walk (2026-04-22).** Three chassis-internal modules (`sim-ml-chassis/src/autograd_policy.rs`, `autograd_value.rs`, `autograd_layers.rs`) — the concrete `AutogradPolicy` / `AutogradStochasticPolicy` / `AutogradValue` / `AutogradQ` types + `Activation` enum + tape-flavored free layer functions — **are NOT affected by any locked A/B/C/D/E.1/E.2 decision at their PUBLIC API surface (zero `Tensor` in forward signatures, zero custom VJPs, zero sim-soft trait participation).** The ONLY real intersection is B.1's vector-aware tape refactor touching internal tape call sites; under scalar-sugar-preserved B.1 implementation (recommended below for info-asymmetry reasons), the E.3 migration is **zero-code-change**. Under Tensor-first B.1 implementation, it's a mechanical wrap/unwrap rename at ~102 call sites across the 3 E.3 modules. PR sequencing is the new third category: **bundled-internal** — E.3 modules ship AS PART OF the chassis A.1+B.1 refactor PR itself (distinct from E.1's "independent" and E.2's "bundled-consumer").

**A.1-line-78 second overstatement.** A.1's "Downstream work (Group E)" paragraph at line 78 names `AutogradPolicy`/`AutogradValue` forward signatures as Tensor-site consumers. Reality verified by grep (`rg "Tensor" sim/L0/ml-chassis/src/autograd*.rs` → zero hits): **zero `Tensor` sites in any E.3 module** — forward signatures use `&[f32]` obs + `&[f64]` params/actions throughout. Second overstatement in the line-78 prediction sequence; E.2 held in between (E.1 sim-opt = zero overstate; E.2 sim-therm-env = partial hold at 4 prod + ~28 test; E.3 autograd = zero overstate). E.5-close footnote now covers all four named consumers.

**Inventory** (Cargo.toml dep direction verified per pattern #8; `sim-ml-chassis/Cargo.toml` carries prod deps nalgebra + rand + serde + serde_json + sim-core + sim-mjcf + thiserror + optional bevy_ecs, dev deps approx + criterion — grader reports 10 deps all justified. Zero faer, zero direct wgpu, no `gpu` feature flag yet per B.5's "chassis acquires a GPU module behind a `gpu` feature flag" commitment — Phase E work, not today).
- **`autograd_policy.rs`** — exports `AutogradPolicy` (impls `Policy` + `DifferentiablePolicy`) + `AutogradStochasticPolicy` (impls `Policy` + `DifferentiablePolicy` + `StochasticPolicy`). Builders: `new` / `new_with` / `new_xavier`. All forward/gradient methods internal: build fresh `Tape`, run forward, backward if needed, read grads, drop. Zero `Tensor`, zero `push_custom`, zero custom VJPs.
- **`autograd_value.rs`** — exports `AutogradValue` (impls `ValueFn`) + `AutogradQ` (impls `QFunction`). Same structural pattern as `autograd_policy.rs`. Zero `Tensor`, zero custom VJPs.
- **`autograd_layers.rs`** — exports `Activation` enum + free functions `linear_tanh` / `linear_relu` / `linear_raw` / `linear_hidden` / `mse_loss` / `mse_loss_batch`. Module-private (not re-exported): tape-flavored `gaussian_log_prob(&mut Tape, &[Var], &[Var], &[Var]) -> Var` (distinct from `sim_ml_chassis::stats::gaussian_log_prob` which is scalar-flavored `(&[f64], &[f64], &[f64]) -> f64` — consumed by sim-rl's sac.rs). All free functions take `&mut Tape + &[Var]` + shape dims, return `Vec<Var>` or `Var`. Zero `Tensor`.
- **Internal tape usage across the 3 E.3 modules (grep-counted):** 82 tape primitive / fused-op call sites (`tape.{param, constant, add, sub, mul, neg, tanh, relu, square, ln, exp, affine, sum, mean, backward}`) + 20 `tape.{value, grad}` reads — mixed prod + `#[cfg(test)]` FD-check scaffolding. Per-module split: `autograd_layers.rs` = 41 + 7 (heavy test-module FD scaffolding); `autograd_policy.rs` = 28 + 8; `autograd_value.rs` = 13 + 5. This is the B.1 migration surface **at E.3 scope** (the chassis `autograd.rs` source file itself has a further 29 tape-op sites + 15 value/grad reads in docstrings + tests, but `autograd.rs` IS the B.1 refactor target itself — the `Tape` source — not an E.3 module). Under B.1, `autograd.rs`'s own migration is the B.1 refactor PR's core work; E.3 modules' 82+20 sites are the downstream consumer-of-tape count that rides along.
- **Downstream consumers of E.3 public surface** (inverse-direction broader grep flipped for chassis-internal walk):
  - **sim-rl prod (`ppo.rs` / `sac.rs` / `td3.rs` / `reinforce.rs` / `cem.rs`):** zero direct E.3 concrete-type consumption. All RL baselines go through trait interface (`Box<dyn Policy>` / `Box<dyn DifferentiablePolicy>` / `Box<dyn StochasticPolicy>` / `Box<dyn ValueFn>` / `Box<dyn QFunction>`). `ppo.rs` carries its own local scalar `gaussian_log_prob` helper (line 188); `sac.rs` uses `sim_ml_chassis::stats::gaussian_log_prob` (scalar flavor, not the tape-flavored `autograd_layers` variant). Never direct `Tape` / `Var` / `Tensor` contact with E.3 internals.
  - **sim-rl tests** (`competition.rs` + `autograd_policy_reinforce_integration.rs`): concrete-type constructor sites only (`AutogradPolicy::new_xavier(obs_dim, hidden_dims, act_dim, obs_scale, Activation::Relu, &mut rng)` etc.). Zero `Tape` / `Var` / `Tensor` exposure.
  - **sim-opt:** zero consumption (confirms E.1's gradient-free finding — SA / richer-SA / PT operate on `Policy` base trait only).
  - **sim-therm-env:** zero consumption (confirms E.2's A.1-only-via-Environment walk; the `builder.rs:166` `LinearPolicy` hit was a docstring reference, not a code import).
  - **`examples/fundamentals/sim-ml/6dof/stress-test/src/main.rs`:** `AutogradStochasticPolicy::new_xavier` constructor + `Activation::Relu` arg — constructor-only surface. The `Tensor::zeros` / `Tensor::from_slice` sites in that example are A.1-environment-side, **not E.3-scope** (they belong to the broader examples walk or E.4 sim-rl).
  - **`artifact.rs`:** `NetworkKind::Autograd` enum variant discriminant — serialization tag, no Tensor/Tape participation.

**Breaking-change intersection (explicit yes/no per locked decision):**

| Locked decision | Touches E.3 modules? | Why |
|---|---|---|
| A.1 `Tensor<T>` generic | **No.** | Zero `Tensor` sites in any of the 3 modules (grep-verified). Trait signatures E.3 impls (`Policy::forward(&[f32]) -> Vec<f64>`, `ValueFn::forward(&[f32]) -> f64`, `QFunction::forward(&[f32], &[f64]) -> f64`, all `*_gradient` methods returning `Vec<f64>`) never crossed the Tensor boundary. A.1 line 78 overstates. |
| A.2 faer | No | No linear-algebra factoring; MLP forward/backward, not Newton solves. Zero faer in chassis Cargo.toml. |
| A.3 nalgebra | No | Zero nalgebra usage in any E.3 module. |
| A.4 IEEE754 / NaN / tolerances | No surface changes | Internal arithmetic (tanh / relu / exp / log-prob closures) stays IEEE-correct within A.4 §1's "propagate internally" rule. Constructor `obs_scale: &[f64]` args lack `.is_finite()` checks but this matches the **chassis-wide pattern** (verified: `MlpPolicy::new` / `LinearPolicy::new` / `LinearValue::new` / `LinearStochasticPolicy::new` all take `obs_scale: &[f64]` same shape). Programmatic constructors — RL orchestration + test fixtures, not user-text / user-JSON input parsing. Classified as A.4 §1 "internal cross-crate API" (trust-the-caller), not "user-facing constructors" (validate). Distinct from E.2's sim-therm-env builder setters (physical config values flowing into MJCF text → physics). **Not a gap; explicit decline.** |
| B.1 vector-aware tape | **Yes — implementation-path dependent.** | 82 tape primitive/fused-op call sites + 20 `tape.{value,grad}` reads across the 3 E.3 modules (per-module split in inventory above). Zero public-API change. Under scalar-sugar-preserved B.1 implementation: zero code change to E.3 modules. Under Tensor-first B.1 implementation: ~102-site mechanical wrap/unwrap rename at E.3 scope. Recommended path below. |
| B.1.a / B.2 `VjpOp` / `push_custom` | No | Zero `push_custom` / `register_vjp` usage in E.3. All autograd graphs compose the 9 scalar primitives + 3 fused ops (`affine` / `sum` / `mean`). `push_custom` is new API with no current E.3-side users. |
| B.3 `Differentiable` placement | No | Chassis `DifferentiablePolicy` (impl'd by `AutogradPolicy` + `AutogradStochasticPolicy`) is the RL gradient-in-params trait; sim-soft's `Differentiable` is the physics IFT/time-adjoint trait — explicitly disambiguated at B.3 line 169 ("No rename of existing `DifferentiablePolicy`"). The brief-doc-note addition at B.3 lands when sim-soft crate is created, not as E.3 work. |
| B.3 `CpuTape` alias | No | sim-soft-side alias; E.3 modules use `sim_ml_chassis::autograd::Tape` directly (not via alias). No visibility into sim-soft's aliasing. |
| B.4 checkpointing / time adjoints | No | No time-adjoint surface; E.3 autograd is one-step forward + one-step backward per tape lifetime. `CheckpointedStepVjp` is sim-soft `VjpOp` territory. |
| B.5 / B.5.a GPU tape / `GpuScalar` | No | CPU-only. E.3 modules do not use `sim_ml_chassis::gpu::GpuTape` / `GpuTensor` (gated behind chassis `gpu` feature per B.5). GPU-tape integration is Phase-E sim-soft + future sim-rl adoption, not E.3-today. |
| C.1 GPU VJP author contract | No | CPU-only. |
| C.2 Preconditioner cross-call cache | No | No preconditioner usage. |
| C.3 `GpuTensorPool` acquire-use | No | CPU-only. |
| C.4 sim-gpu crate fate | No | No sim-gpu dep. |
| D.1 `Solver::replay_step` | No | Zero `Solver` usage. |
| D.2.a `ForwardMap` purity | No direct surface | E.3 modules do not impl `ForwardMap`. Transitively: `Policy::forward` / `ValueFn::forward` / `QFunction::forward` are consumed by upstream `VecEnv`/training-loop code; purity-in-(params, obs) is the base `Policy::forward` contract, ratified at chassis trait level. No new ForwardMap surface introduced by E.3 modules. |
| D.2.b–d `Observable` / Material / Element / Mesh | No | E.3 modules do not impl any sim-soft trait. |
| D.3 coupling traits (`MjcfHandshake` / `ThermostatHandshake`) | No | No handshake-trait usage. |
| D.4 IO ingress (URDF / MJCF NaN/Inf) | No | E.3 modules consume obs from env-side `Environment::step` results, not parsed text. No ingress boundary. |
| E.1 sim-opt zero-changes | No | sim-opt does not consume E.3 public surfaces (orthogonal). |
| E.2 sim-therm-env bundle-forced | No | sim-therm-env does not consume E.3 public surfaces (orthogonal). |

**Migration path: zero code changes under scalar-sugar-preserved B.1 implementation (recommended below).** Public API of all three E.3 modules untouched: `AutogradPolicy::new` / `::new_with` / `::new_xavier` + `Policy::forward` + `DifferentiablePolicy::{log_prob_gradient, forward_vjp}` + `AutogradStochasticPolicy::*` + `StochasticPolicy::{forward_stochastic, log_prob_gradient_stochastic, entropy, reparameterized_vjp}` + `AutogradValue` / `AutogradQ` mirrors + `Activation` enum + `linear_*` / `mse_loss*` free functions — all unchanged. Internal 102 tape call sites at E.3 scope continue to compile because `tape.param(f64)` / `tape.value(var) -> f64` / `tape.grad(var) -> f64` + scalar-valued primitive methods remain the user-facing scalar API atop the internal `Tensor<f64>`-node representation.

**Scalar-sugar-preserved B.1 implementation (recommendation, info-asymmetry override applied).** B.1's locked text: "Current 9 scalar primitives stay as the same nine, generalized to `Tensor<f64>` (a 'scalar' is shape `[]`)." Cross-cutting determinism audit axis 3 on B.1: "B.1's 'the same nine, generalized' are all element-wise pointwise on shape-matched `Tensor<f64>` inputs." Reading this together: B.1 commits internal tape-node representation to `Tensor<f64>` but the "scalar is shape `[]`" phrasing reads as a mental model, not an API-level type-change forcing function. Recommendation: B.1 implementation PR preserves `Tape::param(f64)` / `tape.value(var) -> f64` / `tape.grad(var) -> f64` and the 9 scalar-valued primitive methods (`tape.add(Var, Var) -> Var`, etc.) as public API — scalar-valued sugar atop the internal tensor-node representation. Rationale:
- **Symmetry with A.1 sub-decision B-2.** A.1's `TensorSpec` sub-decision explicitly rejected type-parameter threading through `Env`/`VecEnv`/`Policy` traits ("avoids plumbing a type parameter through..."). Same R34 insulation principle applies at the chassis-autograd RL-side scalar API: don't force Tensor-wrap/unwrap through scalar-only autograd code just because sim-soft's vector VJPs need tensor-first primitives internally.
- **Load-bearing scope: 5 RL baselines + 4 E.3 concrete types + chassis scalar autograd tests + examples.** Preserving scalar sugar constrains sim-soft side zero (vector ops already flow through `Custom(Box<dyn VjpOp>)` per B.1.a/B.2); breaking it cascades ~102 E.3 sites + the chassis `autograd.rs` tape source + downstream wrap/unwrap noise across 5 sim-rl baselines, test fixtures, and examples.
- **No divergence between RL-side and sim-soft-side tape API.** Both sides use the same `Tape` type. Scalar users read `tape.add(v1, v2)`; vector users read `tape.push_custom(tensor_value, Box::new(MyOp))`. Different ops, same chassis, same determinism contract.

**Alternative: Tensor-first B.1 implementation (if scalar sugar is dropped).** E.3 migration becomes mechanical wrap/unwrap at ~102 sites: every `tape.param(val)` → `tape.param(Tensor::scalar(val))` (or equivalent), every `tape.value(var) -> f64` → `tape.value(var).to_scalar()` (or explicit helper), every primitive method's scalar-valued return type becomes `Tensor<f64>` that the caller reads as shape `[]`. Still deterministic, still bundles with chassis B.1 PR, zero shape changes, zero new generics threaded through E.3's public surfaces. **Flagged for completeness; not recommended** per the symmetry argument above. **This is an implementation-path proposal, not a B.1 spec modification.** The B.1 refactor PR decides.

**PR sequencing: bundled-internal with chassis A.1 + B.1 refactor PR(s).** E.3 modules live INSIDE the chassis being refactored — their code ships AS PART OF the A.1+B.1 chassis refactor PR itself, not as a downstream consumer bundle. Whether A.1 (Tensor generic, affects 4 tensor.rs sites + any `Tensor` user) and B.1 (vector-aware tape + `BackwardOp` enum + `VjpOp` trait + `push_custom` method) split into two PRs or ship as one bundle is a sequencing question this audit does not lock; flag that **under either sequencing, E.3 modules ride with the B.1 tape-side refactor** (not with A.1-only). Third PR-sequencing category alongside E.1 (independent) and E.2 (bundled-consumer).

**Pre-decision verification (Option B, E.1/E.2-established cadence).**
- **Ground-truth grade baseline** via `cargo xtask grade sim-ml-chassis` (per `feedback_ground_truth_via_tool.md`): **A across all 6 automated criteria — Coverage 93.3% A+, Documentation 0 warnings, Clippy 0 warnings, Safety 0 violations, Dependencies 10 deps all justified, Bevy-free confirmed.** 362 library tests pass; full grade run 43.3s. **No F-class findings — no hygiene-commit-then-decision prelude applies** (contrast E.1's transitive clippy bleed and E.2's Dependencies F + Coverage F). Significant: the 362-test corpus is the pre-merge regression floor for the chassis A.1+B.1 refactor PR; silent reorder / float-drift catches there before F.2's expensive `d2c_sr_rematch*` fixture validation (~1.5-3h).
- **Broader inverse-direction grep** (E.1/E.2 cadence flipped for chassis-internal walk): grepped all downstream consumers for E.3 public-surface identifiers (`AutogradPolicy` / `AutogradStochasticPolicy` / `AutogradValue` / `AutogradQ` / `Activation` / `linear_tanh` / `linear_relu` / `linear_raw` / `linear_hidden` / `mse_loss` / `mse_loss_batch`). 13 total file hits; code-relevant: 5 files (sim-rl tests + 1 example stress-test + chassis-internal `artifact.rs` + chassis-internal `lib.rs` re-exports). **Zero unexpected consumer paths.** All consumer surfaces are constructor-call + trait-interface shape — zero direct `Tape` / `Var` / `Tensor` contact with E.3 concrete internals. Module-private tape-flavored `gaussian_log_prob` at `autograd_layers.rs:195` confirmed NOT re-exported (lib.rs line 46-48 lists Activation + 4 linear_* + 2 mse_loss* only); the two `gaussian_log_prob`s (tape-flavored + scalar-flavored) live cleanly separated by module boundary.

**E.3 → Audit-driven observations (no new queued items beyond E.5-close footnote update).**
1. **A.1-line-78 second overstatement consolidated (E.5 close-time footnote).** Previously queued from E.1 (sim-opt zero); confirmed held for E.2 (sim-therm-env partial hold at 4 prod + ~28 test); now confirmed OVERSTATED for E.3 (zero). Remaining consumer: sim-rl at E.4 walk. E.5-close footnote now covers all four named consumers — sim-opt = zero sites, sim-therm-env = 4 prod + ~28 test mechanical, `AutogradPolicy`/`AutogradValue` = zero sites, sim-rl = TBD at E.4.
2. **F.2 scope expansion (carried from E.1, strengthened by E.3).** Chassis refactor PR pre-merge fixture validation now has three regression tiers: (a) 362 sim-ml-chassis library tests (minutes, first line of defense — catches RNG-reorder / API-shape regressions); (b) sim-rl + sim-therm-env test suites (minutes, catches consumer-side propagation); (c) sim-opt `#[ignore]`-gated `d2c_sr_rematch*` fixtures with `--release` (~1.5-3h, catches silent numerical drift vs. pre-squash baseline). F.2 methodology: tier (a) + (b) are blocking for PR merge; tier (c) is run once pre-merge and once post-merge tag-anchor ratifies (`ml-chassis-post-impl-pre-squash` precedent).

**E.3 does not lock:**
- E.4 sim-rl baselines walk — separate scope (RL prod training loops, replay-buffer shape, any tape construction inside algorithms, Environment interface boundary). A.1 intersection expected at Environment/VecEnv boundary (like E.2); B.1 intersection depends on whether sim-rl baselines construct tapes directly (grep required at E.4 entry). Bundle-forced PR sequencing likely holds.
- **B.1 implementation-path choice** (scalar sugar preserved vs. Tensor-first). Recommendation above; implementation PR decides. Either path bundles with chassis B.1 refactor; E.3 surface impact ranges from zero code change (scalar sugar) to ~102-site mechanical rename at E.3 scope (Tensor-first).
- Any internal refactoring of E.3 modules beyond B.1 migration. Grade A + 93.3% coverage + 362 passing tests is a clean baseline; no architectural changes indicated.
- `policy.rs:80` `DifferentiablePolicy` docstring addition (B.3-queued) — lands with sim-soft crate creation, not E.3.

**E.4 — sim-rl baselines breaking-change walk (2026-04-22).** sim-rl is **affected by A.1 only — minimal mechanical surface (5 src `Tensor` sites, all type-inferable; zero explicit type-parameter annotation needed under A.1).** All other locked A/B/C/D/E.1/E.2/E.3 decisions: no. PR sequencing: **independent** (E.1 calibration) — source change is potentially zero (max 2 turbofish if Rust inference fails at `Tensor::from_slice` call sites), Algorithm trait signature is A.1-invariant, zero workspace consumers of `sim_rl::Tensor` re-export. No new audit-item findings beyond the A.1-line-78 footnote consolidation at E.5 close + an inline correction to E.3's line 533 miscount (this commit).

**Inventory** (Cargo.toml dep direction verified per pattern #8):
- Prod deps: `rand`, `sim-ml-chassis`. Two prod deps total.
- Dev deps: `approx`, `serde_json`, `sim-mjcf`. Three dev deps. Five total — matches grade output.
- **Reverse-direction (consumers of sim-rl as dev-dep):** `sim-therm-env`, `sim-opt`, `sim-thermostat`. **Zero downstream prod consumers.** Workspace example consumer: `examples/fundamentals/sim-ml/persistence/train-then-replay/src/main.rs`.
- 5 algorithm structs (`Cem`/`Reinforce`/`Ppo`/`Sac`/`Td3`) + 5 `*Hyperparams` structs. **5 `impl Algorithm for X` blocks** (one per baseline, at `cem.rs:121` / `reinforce.rs:145` / `ppo.rs:203` / `td3.rs:222` / `sac.rs:229`). **Zero impls of any other chassis trait** — sim-rl does NOT impl `Environment`, `VecEnv`, `Competition`, `Policy`, `DifferentiablePolicy`, `StochasticPolicy`, `ValueFn`, or `QFunction`. All those traits are CONSUMED via `Box<dyn ...>`, never implemented.
- Chassis surfaces consumed (~30 identifiers across 13 module paths): `Algorithm` / `EpochMetrics` / `TrainingBudget`, `ArtifactError` / `NetworkSnapshot` / `PolicyArtifact` / `TrainingCheckpoint`, `BestTracker`, `Optimizer` / `OptimizerConfig`, `Policy` / `DifferentiablePolicy` / `StochasticPolicy`, `ReplayBuffer`, `compute_gae`, `collect_episodic_rollout` / `Trajectory`, `gaussian_log_prob` (scalar flavor), `Tensor`, `ValueFn` / `QFunction` / `soft_update`, `VecEnv`. Re-exports add `Competition` / `TrainingProvenance` / `Environment` / `SimEnv` / `StepResult` / `LinearPolicy` / `LinearQ` / `LinearStochasticPolicy` / `LinearValue` / `MlpPolicy` / `MlpQ` / `MlpValue` / `ActionSpace` / `ObservationSpace` / `TaskConfig` / `reaching_2dof` / `reaching_6dof`.

**Internal usage (grep-counted):**
- **5 src `Tensor` sites** — `lib.rs:44` (`pub use sim_ml_chassis::tensor::Tensor;` re-export) + `sac.rs:30` (`use sim_ml_chassis::tensor::Tensor;`) + `sac.rs:321` (`Tensor::from_slice(&action_data, &[n_envs, act_dim])`) + `td3.rs:26` (same `use` import) + `td3.rs:313` (same `from_slice` construction). **Zero test-side `Tensor` sites.**
- **Zero `Tape` / `Var` references** anywhere in sim-rl src + tests (broad grep `\bTape\b|\bVar\b|\bTensor\b` returned only the 5 `Tensor` sites listed above).
- **10 `NetworkSnapshot` construction sites** in src — `sac.rs` lines 593/598/603/608 (4 sites, q1/q2/target_q1/target_q2 critic snapshots), `ppo.rs` line 472 (1 site, single value-fn critic), `td3.rs` lines 554/559/564/569/574 (5 sites, q1/q2/target_q1/target_q2 + target_policy snapshot). 4+1+5 split. Plus 3 imports = 13 references total. Pure `Vec<f64>` params, no Tensor.
- **Heavy `Box<dyn ...>` consumption pattern:** `Policy` (cem), `DifferentiablePolicy` (reinforce, td3 ×2 actor + target, ppo), `StochasticPolicy` (sac), `QFunction` (sac ×4 = q1+q2+target_q1+target_q2, td3 ×4 same shape), `ValueFn` (ppo), `Optimizer` (reinforce ×1, ppo ×2 actor+critic, td3 ×3, sac ×3).
- **Workspace grep for `sim_rl::Tensor` consumers: zero hits.** lib.rs:44 re-export has no downstream users. The 13 `use sim_rl::` import sites across workspace (sim-rl's own 4 test files + sim-therm-env's 3 experiment fixtures + sim-opt's 3 d2c rematch fixtures + sim-thermostat's 3 D-series training tests + 1 example) consume algorithm + chassis-trait re-exports, never the `Tensor` re-export.

**Algorithm trait signature is invariant under A.1.** Five trait methods on `pub trait Algorithm: Send` (`sim/L0/ml-chassis/src/algorithm.rs:77-119`):
- `fn name(&self) -> &'static str`
- `fn train(&mut self, env: &mut VecEnv, budget: TrainingBudget, seed: u64, on_epoch: &dyn Fn(&EpochMetrics)) -> Vec<EpochMetrics>`
- `fn policy_artifact(&self) -> PolicyArtifact`
- `fn best_artifact(&self) -> PolicyArtifact`
- `fn checkpoint(&self) -> TrainingCheckpoint`

Zero `Tensor` references in any signature. Referenced structs all use `Vec<f64>` for parameters: `PolicyArtifact { params: Vec<f64>, ... }`, `NetworkSnapshot { params: Vec<f64>, ... }`, `TrainingCheckpoint { policy_artifact, critics: Vec<NetworkSnapshot>, optimizer_states, algorithm_state: BTreeMap<String, f64>, ... }`. sim-rl's 5 `impl Algorithm for X` blocks compile unchanged under A.1 — no method-signature forcing function.

**A.1-line-78 third overstatement (sim-rl); fourth-and-final consumer verification.** A.1's "Downstream work (Group E)" paragraph at line 78 names sim-rl as a consumer "gaining a type parameter at `Tensor` sites." Reality verified by grep: **5 src `Tensor` sites exist, but ALL are type-inferable.** `action_data: Vec<f32>` (per the algorithm code at `sac.rs:316` / `td3.rs:308` writing `a as f32` into the buffer); downstream `env.step(&action_tensor)` consumes `&Tensor<f32>` (chassis `VecEnv::step` signature stays f32-concrete per A.1 sub-decision B-2 — `vec_env.rs:132`); type inference propagates `T = f32` through `Tensor::from_slice(&action_data, &[n_envs, act_dim])`. The `pub use sim_ml_chassis::tensor::Tensor;` re-export at `lib.rs:44` syntactically resolves to whichever (non-generic or generic) `Tensor` chassis exports — zero source change. Zero workspace consumers of `sim_rl::Tensor`. **All 5 sites: zero explicit type-parameter annotation needed.** Worst case if Rust inference fails at chassis A.1 PR compile time (low probability — bidirectional inference is robust for associated functions with consuming context): 2 explicit turbofish (`Tensor::<f32>::from_slice(...)`) at `sac.rs:321` + `td3.rs:313`, 2 line additions, trivially shippable as a sim-rl follow-up commit (not blocking chassis A.1 PR merge). **The "gains a type parameter at Tensor sites" prediction overstates** in the same calibration as E.1 sim-opt (zero sites at all) and E.3 autograd modules (zero sites at all): sim-rl HAS sites but does NOT GAIN explicit type-parameter annotation at any of them. **Third overstatement in the line-78 prediction sequence; fourth-and-final consumer verification.** E.5-close footnote consolidates all four outcomes (sim-opt + autograd OVERSTATED with zero sites; sim-rl OVERSTATED with 5 type-inferable sites; sim-therm-env HOLDS with 4 prod + ~28 test sites that DO get explicit `<f32>` annotation).

**Breaking-change intersection (explicit yes/no per locked decision):**

| Locked decision | Touches sim-rl? | Why |
|---|---|---|
| A.1 `Tensor<T>` generic | **Yes — minimal mechanical (likely zero source change with type inference; max 2 turbofish if inference fails).** | 5 src `Tensor` sites: lib.rs:44 (`pub use ... Tensor;` re-export, syntactically unchanged), sac.rs:30 + td3.rs:26 (`use ... Tensor;` imports, syntactically unchanged), sac.rs:321 + td3.rs:313 (`Tensor::from_slice(&action_data, &[n_envs, act_dim])`, type-inferable from `action_data: Vec<f32>` flowing into `env.step(&Tensor<f32>)` per B-2). Zero test-side Tensor sites. Zero workspace `sim_rl::Tensor` consumers. Worst case 2 explicit turbofish at construction sites. |
| A.2 faer | No | Gradient-free at tape level (zero `Tape`); no factor-on-tape; no faer dep candidate. |
| A.3 nalgebra | No | Zero nalgebra usage in sim-rl src + tests (grep). |
| A.4 §1 user-facing constructors | No surface change (explicit decline) | All 5 algorithm constructors take pre-built `Hyperparams` struct + `Box<dyn Trait>` policy/value-fn + `OptimizerConfig`. None take raw f64 args. The five `*Hyperparams` structs have public f64 fields (`gamma`, `clip_eps`, `noise_std`, `tau`, etc.) but the **chassis-wide pattern** is programmatic-construction trust-the-caller (verified at E.3 for `MlpPolicy::new` / `LinearPolicy::new` / `LinearValue::new` / `LinearStochasticPolicy::new` / `AutogradPolicy::new` all taking `obs_scale: &[f64]` without `.is_finite()` checks). Same A.4 §1 "internal cross-crate API" classification as E.3 — RL hyperparameters are algorithm-author choice, not user-text / user-JSON parsing. Distinct from E.2's sim-therm-env builder setters (physical config values flowing into MJCF text → physics). **Not a gap; explicit decline matching E.3's chassis-wide-pattern finding.** |
| B.1 vector-aware tape | **No — zero direct tape construction in entire sim-rl crate.** | Zero `Tape` / `Var` references in sim-rl src + tests (grep-verified). All differentiable-policy work flows through `Box<dyn DifferentiablePolicy>` / `Box<dyn StochasticPolicy>` interface — sim-rl never constructs a tape. ppo.rs carries its own local scalar `gaussian_log_prob` helper (line 188, scalar f64 fold, no `Var`); sac.rs uses `sim_ml_chassis::stats::gaussian_log_prob` (scalar flavor, distinct from the tape-flavored `autograd_layers` variant per E.3's separation finding). Both are scalar API; tape lives chassis-side inside the boxed policy's gradient methods. **B.1 surface impact at sim-rl = zero; classifies sim-rl as gradient-free-at-tape-level.** Symmetric with E.1 sim-opt. |
| B.1.a / B.2 `VjpOp` / `push_custom` | No | Zero `register_vjp` / `push_custom` usage. |
| B.3 `Differentiable` placement | No | sim-rl consumes `Box<dyn DifferentiablePolicy>` (chassis trait, RL gradient-in-params). sim-soft's `Differentiable` (B.3-locked at sim-soft::autograd) is orthogonal; brief-doc-note addition lands when sim-soft crate is created, not in sim-rl. |
| B.3 `CpuTape` alias | No | Zero `Tape` usage means zero tape-aliasing surface. |
| B.4 checkpointing / time adjoints | No | No time-adjoint surface; algorithm `train` loops are one-step-at-a-time via `VecEnv::step`. Checkpoint state = per-algorithm `algorithm_state: BTreeMap<String, f64>` + `Vec<NetworkSnapshot>` with `params: Vec<f64>` (10 construction sites, 4+1+5 sac/ppo/td3 split). No tape-checkpoint. B.4's time-adjoint mechanism is sim-soft territory. |
| B.5 / B.5.a GPU tape / `GpuScalar` | No | CPU-only (no wgpu in deps). |
| C.1 GPU VJP author contract | No | CPU-only. |
| C.2 Preconditioner cross-call cache | No | No preconditioner usage. |
| C.3 `GpuTensorPool` acquire-use | No | CPU-only. |
| C.4 sim-gpu crate fate | No | No sim-gpu dep. |
| D.1 `Solver::replay_step` | No | Zero `Solver` usage. |
| D.2.a `ForwardMap` purity | No direct surface | sim-rl does not impl `ForwardMap`. Transitively: sim-rl impls `Algorithm`, which `train`s by calling `VecEnv::step` / `reset_all`; purity-in-(params, obs) is the consumed `Policy::forward` / `DifferentiablePolicy::log_prob_gradient` / `StochasticPolicy::*` / `ValueFn::forward` / `QFunction::forward` contract, ratified at chassis trait level. sim-rl introduces no new ForwardMap surface. Same transitive template as E.1 sim-opt + E.2 sim-therm-env + E.3 autograd modules. |
| D.2.b–d `Observable` / Material / Element / Mesh | No | sim-rl does not impl any sim-soft trait. |
| D.3 coupling traits (`MjcfHandshake` / `ThermostatHandshake`) | No | No handshake-trait usage. |
| D.4 IO ingress (URDF / MJCF NaN/Inf) | No | sim-rl consumes obs from env-side `VecEnv::step` results, not parsed text. No ingress boundary (sim-mjcf is dev-dep only for fixture loading). |
| E.1 sim-opt zero-changes | No | sim-opt has sim-rl as dev-dep (d2c rematch fixtures); orthogonal. sim-opt's own zero-change walk and sim-rl's own zero-change-modulo-inference walk are independently verifiable. |
| E.2 sim-therm-env bundle-forced | No | sim-therm-env has sim-rl as dev-dep (experiment fixtures); orthogonal. sim-rl's own A.1 mechanical surface is independent of sim-therm-env's `impl Environment` mechanical rename. |
| E.3 autograd modules bundled-internal | No (orthogonal) | E.3 modules are chassis-internal; sim-rl tests construct E.3 concrete types (`AutogradPolicy::new_xavier(...)`, `AutogradStochasticPolicy::new(...)`, `AutogradQ::new(...)`, `AutogradValue::new(...)` — heavily in `tests/competition.rs`, lightly in `tests/autograd_policy_reinforce_integration.rs`) but constructor signatures are `(usize, &[usize], usize, &[f64], Activation, &mut StdRng)`-shape with zero `Tensor` / `Tape` / `Var` exposure. E.3's bundled-internal status rides with the chassis B.1 refactor PR; sim-rl test fixtures consume E.3 outputs through the boxed-trait interface, A.1-invariant. |

**Migration path: zero source changes (modulo type inference).** All 5 `Tensor` src sites compile against EITHER pre-A.1 chassis (non-generic `Tensor`) OR post-A.1 chassis (generic `Tensor<T>` with `T = f32` inferred from `action_data: Vec<f32>` + `env.step(&Tensor<f32>)`). The `pub use sim_ml_chassis::tensor::Tensor;` re-export at lib.rs:44 syntactically resolves to whichever type chassis exports. Worst-case fallback if Rust bidirectional inference does not propagate through `Tensor::from_slice` (associated-function call with consuming context): explicit `Tensor::<f32>::from_slice(...)` at sac.rs:321 + td3.rs:313 — 2 line additions. Algorithm trait impls are A.1-invariant per the "Algorithm trait signature" paragraph above; Hyperparams structs are A.1-invariant; checkpoint/artifact structs are A.1-invariant.

**PR sequencing: independent (E.1 calibration).** sim-rl can ship before, during, or after the chassis A.1 refactor PR without consumer-side intervention. Distinct from E.2 sim-therm-env (bundled-consumer because `impl Environment for ThermCircuitEnv`'s method signatures rename mechanically with chassis Environment trait — that's the forcing function); E.4's Algorithm trait impls do not reference `Tensor` in their method signatures, so the A.1 forcing function does not apply. Distinct from E.3 autograd modules (bundled-internal because they live INSIDE the chassis being refactored). **Type-inference verification at chassis A.1 PR compile time:** if Rust inference fails to propagate `T = f32` through `Tensor::from_slice` at sac.rs:321 + td3.rs:313 (low-probability — bidirectional inference is robust for associated functions with consuming context), 2 explicit turbofish ship as a sim-rl follow-up commit (not blocking chassis A.1 PR merge). Three established PR sequencing categories now have all four E-walks placed: independent (E.1 sim-opt + E.4 sim-rl), bundled-consumer (E.2 sim-therm-env), bundled-internal (E.3 autograd).

**Pre-decision verification (Option B, E.1/E.2/E.3-established cadence).**
- **Ground-truth grade baseline** via `cargo xtask grade sim-rl` (per `feedback_ground_truth_via_tool.md`): **A across all 6 automated criteria — Coverage 97.7% A+, Documentation 0 warnings, Clippy 0 warnings, Safety 0 violations, Dependencies 5 deps all justified, Bevy-free confirmed.** 37 lib tests + 1 autograd_policy_reinforce_integration + 5 best_tracker_cem_integration + 1 custom_task = 44 tests passed; 13 competition tests + 5 doctests `#[ignore]`-gated as expected (multi-minute competition runs not part of standard grade); full grade run 957.3s (~16 min, dominated by Coverage pass 1/2's instrumented RL training tests at release). **No F-class findings — no hygiene-commit-then-decision prelude applies** (matches E.3's clean-grade pattern; second consecutive clean-grade walk; firmly establishes the hygiene-then-decision pattern as conditional on F-class trigger, not default-apply — 3 of 5 walks applied (D.4.b + E.1 + E.2), 2 of 5 did not (E.3 + E.4)).
- **Broader inventory grep** against the full `sim_ml_chassis::lib.rs` re-export list (~62 identifiers, including `Tape`, `Var`, all 4 Autograd concrete types, `Activation`, all `linear_*` / `mse_loss*`, all `*Builder`, all error types): **zero unexpected surfaces beyond the locked-decision keyword sweep.** Heavy test-side `AutogradPolicy` / `AutogradStochasticPolicy` / `AutogradQ` / `AutogradValue` / `Activation` / `obstacle_reaching_6dof` / `RunResult` consumption (~30+ sites in `competition.rs`, 4 in `autograd_policy_reinforce_integration.rs`) is constructor-only — already counted in E.3's downstream-consumer grep, all zero-Tensor / zero-Tape / zero-Var. `NetworkSnapshot` construction (10 src sites in sac/ppo/td3, 4+1+5 split) is pure `Vec<f64>` params, no Tensor. Consumer reverse-direction grep (`use sim_rl::`) returns 13 file hits across workspace: sim-rl's own 4 test files + sim-therm-env's 3 experiment fixtures + sim-opt's 3 d2c rematch fixtures + sim-thermostat's 3 D-series training tests + 1 example. Zero `sim_rl::Tensor` imports in any consumer.

**E.4 → Audit-driven observations (no new queued items beyond E.5-close consolidation + line 533 inline correction this commit).**

1. **A.1-line-78 fourth-and-final consumer verified (E.5 close-time footnote consolidated).** Status final across all four named consumers: sim-opt = OVERSTATED (zero Tensor sites at all); sim-therm-env = HOLDS (4 prod + ~28 test mechanical sites with explicit `<f32>` annotation); AutogradPolicy/AutogradValue = OVERSTATED (zero Tensor sites at all); sim-rl = OVERSTATED (5 sites that are type-inferable, zero explicit type-parameter annotation needed). **Three overstatements + one hold across all four named consumers.** E.5-close footnote at line 79 spells out the per-consumer outcomes; the prediction was directionally correct (sim-therm-env genuinely holds with explicit annotation at all 32 sites) but three-quarters of named consumers don't gain explicit type-parameter annotation in the way line 78 implies.

2. **Hygiene-then-decision pattern firmly conditional on F-class trigger (5-walk pattern observation).** Five Group-D-and-E walks: D.4.b applied (`d12a5e73` URDF clippy fix), E.1 applied (`f614cd77` workspace clippy hygiene), E.2 applied (`3a45d50f` Cargo.toml dep-justifications), E.3 NOT applied (grade A clean), E.4 NOT applied (grade A clean — sim-rl's 97.7% Coverage A+ is the second clean-grade walk in a row). **Pattern is firmly conditional, not default-apply.** F-class triggers are real but not universal; clean-grade walks proceed directly to decision commits without hygiene preludes.

3. **E.3 line 533 inline correction (audit-driven, this commit).** Audit-doc revision-pass during E.4 walk surfaced miscount in the E.3 entry: line 533 reads "Third consecutive overstatement in the line-78 prediction" but the actual count at E.3 close was 2 overstatements (E.1 + E.3) + 1 hold (E.2) — not "third consecutive." Header "A.1-line-78 third overstatement" carries the same drift. Corrected inline this commit to "A.1-line-78 second overstatement" + "Second overstatement in the line-78 prediction sequence; E.2 held in between" per pattern #8 discipline (technical claims about sequence counts must hold). E.4's count update lands in this commit's own A.1-line-78 paragraph above.

**E.4 does not lock:**
- E.5 Group E close — `platform_refactor_soft_body_ready.md` line 79 footnote consolidating all four named-consumer outcomes; updated Group E status line; review of any audit-driven items deferred to Group F or A.4 §1 sweep candidate. Single-session close expected.
- Group F build/test/CI infrastructure scope — F.1 grader package-scoping fix + F.2 three-tier chassis-refactor pre-merge fixture validation + F.3 candidate sim-therm-env Coverage F structural + workspace hygiene (mesh-measure / mesh-lattice / others outside L0 dep chain).
- A.4 §1 compliance sweep candidate — sim-therm-env builder setters + post-D MjcfGeom ingress gap. **NOT clustered with E.4** per chassis-wide programmatic-pattern explicit decline (matches E.3 finding).
- Any sim-rl internal refactoring beyond A.1's potential 2-turbofish — Grade A + 97.7% Coverage A+ + 44 passing tests is a clean baseline; no architectural changes indicated.
- B.1 implementation-path choice (scalar sugar preserved vs. Tensor-first per E.3 recommendation). Independent of sim-rl; sim-rl is gradient-free at tape level either way.

### Group F

**Status:** F.1, F.2, F.3, workspace hygiene placement locked (2026-04-22). Implementation-PR-shipping-strategy, Group F close + Audit close — remaining.

**F.1 — grader package-scoping fix (2026-04-22).** `grade_clippy` at `xtask/src/grade.rs:834-898` filters JSON `compiler-message + warning|error + non-empty-spans` diagnostics without checking whether any span originates in the target crate. Structural fix: add a disjunctive `any_in_crate` filter immediately after the empty-spans check at line 858, matching the `grade_coverage` line-721 convention (`if !filename.contains(crate_path) { continue; }`). **Scope locked; implementation ships on the new implementation branch after the audit PR merges**, not as a commit on the audit branch itself — the three precursor code commits already on this branch (`d12a5e73` / `f614cd77` / `3a45d50f`) were state-cleanup-to-enable-decisions; F.1 is new tooling with grader unit tests + Option-B sweep, appropriately scoped to its own implementation-branch work. Shape (standalone PR vs. part of a larger implementation bundle) is governed by the implementation-PR-shipping-strategy sub-item, still pending.

**Filter design (three calibration axes).**

1. *Include-not-exclude.* The existing bug surfaces as false positives (transitive warnings inflate graded counts) — visible pressure. The opposite failure (real warnings in graded-crate code silently filtered out) is harder to detect because grades look clean while issues persist. Risk-averse design errs toward including ambiguous diagnostics: a macro-generated warning with one span inside the crate + one outside gets counted, not dropped. Matches `grade_coverage` summing-loop semantics (any file with `filename.contains(crate_path)` contributes).
2. *Substring match.* `crate_path` is workspace-relative like `sim/L0/opt`; llvm-cov and rustc emit absolute paths. `.contains()` handles both without a normalization pass. Same choice as line 721.
3. *No expansion-tree recursion.* `message.spans[*].expansion` can nest further spans through macro expansions; recursing adds complexity without a current concrete failure case. YAGNI until a false-positive surfaces; widen then.

**Verification.** Three grader unit tests (positive: spans inside crate / negative: spans outside / mixed: 2 spans, one each) on synthesized JSON lines, sibling to the existing 44 grader unit tests. Plus an Option-B re-grade sweep across every L0 crate post-F.1 — each graded-crate's own-source clippy count must equal today's post-hygiene baseline (F.1 affects transitive bleed, not own-source warnings). Any delta = filter-semantics bug.

**Bundling.** F.1 commit is grader code + tests only. Workspace hygiene auto-fixes (`mesh-measure` 2 × `missing_const_for_fn`, `mesh-lattice` 16 × `missing_const_for_fn`, likely others outside the L0 dep chain) are a separate Group F sub-item (workspace hygiene placement, pending decision). Bundling them into F.1 would complicate review, revert, and grade-delta attribution. Risk-averse: unbundle.

**F.1 does not lock:**
- Workspace hygiene placement — separate Group F sub-item (fold into F.1 PR, separate post-audit hygiene PR, or additional audit-branch hygiene commit following the `f614cd77` precedent).
- sim-conformance-tests Coverage F resolution — structurally distinct from F.3 sim-therm-env scope (sim-conformance-tests has no lib target at all; sim-therm-env has a lib target but zero `#[cfg(test)]` modules in `src/`). Flag as F.3-adjacent; decided at F.3.
- Expansion-tree recursion widening — deferred pending a concrete false-positive case.

**F.2 — chassis A.1+B.1 refactor PR pre-merge validation protocol (2026-04-22).** Three-tier validation mechanism lives in this audit doc as a runnable protocol, reproduced in the implementation PR's description at merge time (tier outputs pasted as a commitment ritual). Not a CI gate, not a new xtask subcommand. Each tier catches a distinct regression class at a distinct compute cost.

**Per-tier protocol.**

1. **Tier (a) — sim-ml-chassis library tests.** `cargo test --release -p sim-ml-chassis` — 362 inline `src/#[cfg(test)]` tests (zero files in tests/, confirmed inventory). Minutes. Catches intra-chassis regressions: RNG-reorder, API-shape breakage, scalar-autograd FD gradcheck at `PARITY_TOL = 1e-10` / `FD_TOL = 1e-5`. Runs locally on any PR touching `sim/L0/ml-chassis/src/`.

2. **Tier (b) — consumer test suites.** `cargo test --release -p sim-rl` (44 fast tests; 13 `#[ignore]`-gated competition fixtures excluded per E.4 walk) + `cargo test --release -p sim-therm-env` (~31 fast tests across `phase2.rs` / `phase3.rs` / `experiment_1.rs` / `experiment_4.rs` / `ising_chain.rs` per E.2 walk). Minutes each. Catches consumer-side propagation through chassis trait surfaces: `VecEnv`/`Policy`/`Environment` impl signatures, `Tensor<T>` generics propagation per A.1 sub-decision B-2, `impl Environment for ThermCircuitEnv` mechanical rename at ~32 sites per E.2. Runs locally before opening any PR that touches chassis trait signatures.

3. **Tier (c) — sim-opt d2c_sr_rematch fixtures.** `cargo test --release -p sim-opt --ignored d2c_sr_rematch` — 3 `#[ignore]`-gated fixtures (`tests/d2c_sr_rematch.rs` uses `#[ignore = "..."]` attribute form; `_richer_sa.rs` / `_pt.rs` use bare `#[ignore]`). ~1.5-3h total on MBP. Catches silent numerical drift vs. the post-squash `ml-chassis-post-impl-pre-squash` tag baseline from PR #190. Runs **twice**: once pre-merge against the implementation-PR HEAD to confirm `Ok(outcome)` and the matched-complexity gate holds for all 3 fixtures; once post-squash against a fresh `<implementation-branch>-pre-squash` tag (per `feedback_pre_squash_tag.md` convention) to lock the determinism anchor for future reference.

**Placement decision: documentation-primarily (risk-averse call).** Protocol lives in this audit-doc entry; reproduced in implementation-PR description at merge time, including tier outputs as PR-description paste commitment. Rationale:
- *Single-use window.* Validation exists for the chassis A.1+B.1 refactor PR specifically. After that refactor lands, future chassis changes carry different risk profiles. New CI gate or xtask subcommand outlives its narrow-use window. YAGNI.
- *Tier (c) forces docs-primarily anyway.* ~1.5-3h compute can't cheaply run under GitHub free-tier CI; dedicated long-running-runner infrastructure is out of scope. Symmetry: keep all three tiers in the same place.
- *PR-description paste is the ritual record.* Engineer discipline + audit-doc protocol + PR-description paste is the enforcement mechanism. Visible commitment to reviewers: "tier (c) ran, here's the output." Reviewable at merge time.

**F.2 does not lock:**
- Tier (a) CI gating — coupled to the implementation-PR-shipping-strategy decision. If shipping is "one big implementation PR," tier (a) runs once and CI-gating adds little; if shipping splits by group, tier (a) runs per-PR and CI gating is material. Revisit when shipping-strategy resolves.
- xtask convenience wrapper (`cargo xtask validate-chassis-refactor` chaining all three tiers) — YAGNI; add trivially if implementation-PR author finds copy-paste friction material.
- Exact pre-squash tag name — implementation-branch-specific; decided when the branch is created.
- Tier-output artifact storage — implementation-PR author's discretion (PR-description paste is the minimum; additional artifact upload is optional).

**F.3 — sim-therm-env + sim-conformance-tests Coverage F resolution (2026-04-22).** Two structurally-F-gated crates under the current grader: **sim-therm-env** (E.2 finding — has `src/` lib target but zero `#[cfg(test)]` modules, `--lib` coverage reports 0.0%) and **sim-conformance-tests** (F.1 inventory finding — has no `src/` at all, integration-test-only with `[[test]]` paths and `publish = false`, `--lib` errors out). **Resolution: introduce `CrateProfile::IntegrationOnly` via Cargo.toml metadata opt-in; Coverage criterion returns `NotApplicable`. Other criteria apply normally.**

**Three alternatives declined.**

- **(a) Add inline `#[cfg(test)]` unit tests to sim-therm-env's `src/env.rs` / `src/builder.rs` (~200-500 LOC).** Environment impl and Builder wire sim-core / sim-thermostat / sim-mjcf physics; isolating wrapper code from the physics chain is artificial. Integration tests (5 files: `phase2.rs`, `phase3.rs`, `experiment_1.rs`, `experiment_4.rs`, `ising_chain.rs` — ~31 fast + 22 `#[ignore]`-gated heavy per memory `project_therm_circuit_env.md`) genuinely exercise the code paths. Forcing inline duplication adds maintenance burden without commensurate isolation benefit. **Partial exception:** the queued A.4 §1 compliance sweep for builder setters (`.is_finite()` checks) will naturally add ~50-100 LOC of unit-testable validation logic with accompanying tests; that lift is bonus coverage, not the F.3 mechanism.
- **(b) Grader fallback to `--tests` mode for sim-therm-env.** Integration-test instrumentation overhead is ~10× per grader docstring at `xtask/src/grade.rs:637-648`; sim-therm-env's fast tests under instrumentation run substantially slower than the `--lib` baseline. More fundamentally: sim-conformance-tests has no `src/` to measure at all — option (b) handles sim-therm-env but not the second F-case. Splitting into two fixes complicates the audit narrative.
- **(c) as originally stated — accept F with documentation carve-out.** Leaves a standing F on the automated grade table; reviewers have to remember which F is "real" vs. "structural." `NotApplicable` is the cleaner signal, consistent with how STANDARDS.md §1 already treats Example / Xtask crates (grader lines 607-618).

**Chosen path — `IntegrationOnly` profile with metadata opt-in.** Scope:
- New `CrateProfile::IntegrationOnly` variant at `xtask/src/grade.rs:139-154`, joining the existing Example / Xtask / BevyLayer1 / Layer0 set. Coverage returns `NotApplicable` for this profile (mirroring Example / Xtask behavior at lines 607-618). Other criteria untouched.
- `classify_crate` at `xtask/src/grade.rs:222-234` grows a Cargo.toml metadata read: `[package.metadata.cortenforge] grading_profile = "integration-only"`. Self-documenting — the annotation lives in the crate's own Cargo.toml where any reader encounters it.
- sim-therm-env and sim-conformance-tests add the opt-in annotation when the grader change ships. No other L0 crate needs the annotation today.
- Estimated implementation: ~30-50 LOC grader diff + 2 Cargo.toml annotations + 2 grader unit tests (classifier resolves metadata correctly; Coverage returns `NotApplicable` for IntegrationOnly). Ships on the implementation branch, natural bundle with F.1 (both grader infrastructure changes).

**sim-conformance-tests degenerate case.** No `src/` at all means Documentation (via `cargo doc --no-deps`) returns A trivially (nothing to warn about), Clippy post-F.1 returns A (no src/ warnings + transitive bleed filtered), Safety returns Manual with `"(no src/)"` at `grade.rs:931-938`. These are correct grader behaviors; IntegrationOnly only needs to address the Coverage criterion specifically. No additional special-casing required.

**F.3 does not lock:**
- Alternative metadata schema (per-criterion skip flags vs. profile enum) — chose profile enum for symmetry with Example / Xtask / BevyLayer1.
- Path-prefix auto-detection fallback — explicit opt-in preferred; no silent behavior change on `src/` reorganization.
- Future all-integration-test crates' opt-in — annotate if/when they emerge.
- A.4 §1 compliance sweep scope — separate queued item (post-E / pre-skeleton PR); incidentally lifts sim-therm-env's measurable coverage when it ships, but doesn't change F.3's mechanism.

**F — workspace hygiene placement (2026-04-22).** Three remaining workspace crates carry own-src clippy warnings (verified via `cargo clippy --workspace --all-targets --all-features` post-F.1-design and filtered by own-src file_name per F.1's proposed filter semantics): **mesh-lattice** (16 × `missing_const_for_fn`), **mesh-measure** (2 × `missing_const_for_fn`), **sim-gpu** (3 × `manual_is_multiple_of` + 2 × `missing_const_for_fn`). Total 23 warnings, all `cargo clippy --fix`-auto-fixable. **Placement: audit-branch hygiene commit, matching `d12a5e73` + `f614cd77` precedent.**

**sim-gpu expansion vs. pickup prediction.** Pickup memo at `project_next_session_pickup.md:63` predicted "mesh-measure + mesh-lattice + likely others outside sim-opt/sim-therm-env/sim-ml-chassis/sim-rl dep chain." Workspace sweep confirmed mesh-* counts (pickup's 1×/8× were stale; live 2×/16× per post-F.1 sanity-sweep `d7e31f25`) and surfaced **sim-gpu as a new case**: in L0, not in any E-series walk's dep chain (sim-opt/sim-therm-env/sim-ml-chassis/sim-rl don't depend on sim-gpu). Pre-fix: sim-gpu grades F on clippy. Post-fix: A. Same hygiene class, previously hidden from audit-walk visibility.

**Placement rationale (risk-averse, matches precedent with one notable difference).** Both existing audit-branch hygiene commits were **precursor hygiene** — `d12a5e73` cleaned state so D.4's sim-urdf grade-A baseline could be established; `f614cd77` cleaned state so E.1's sim-opt grade-A baseline could be established. The current fix is **not precursor** — Group F is the last group, no future walks need a baseline. It's workspace-quality-cleanup-at-audit-close, same commit shape with different motivation. Recording the distinction explicitly rather than treating the precedent as unconditional.

- *Single audit squash captures it.* User locked "one big PR" for audit close; audit-branch commit rolls into the squash rather than forcing a separate post-audit hygiene PR.
- *Mechanical and bounded.* All 23 warnings auto-fix via `cargo clippy --fix --allow-dirty --workspace`; no judgment calls. Reversible by single-commit revert.
- *Reset the baseline.* Workspace hygiene items accumulate when deferred. Post-fix state = "all workspace crates clippy-clean on own-src warnings, verified against the F.1-proposed filter semantics."

**Does not lock:**
- F.1 grader filter implementation — still ships post-audit; the F.1 filter will re-verify this hygiene-clean state on the implementation branch.
- Other workspace-quality audits (rustc lints beyond clippy, dep-justification sweeps, etc.) — out of F scope; separate audit items if surfaced later.
- Auto-fix review burden — `cargo clippy --fix` output reviewed pre-commit for any unexpected refactors; if autofix surfaces non-mechanical suggestions, manual review per-file.

### Cross-cutting determinism audit

Cross-cutting pass over each group's locked decisions against `ForwardMap`'s determinism-in-θ contract (`110-crate/02-coupling/03-ml-chassis.md` §"Determinism-in-θ and the cached-tape contract") and A.4 §4 ("algorithm-output, not bit-exact; same θ on same machine on sequential path bit-reproducible; parallel paths accept non-associativity; cross-platform libm divergence within 5-digit gradcheck tolerance"). Each entry lands in one of two authorized categories: *preserves determinism because X* or *weakens within tolerance Y*. No silent weakening; no third "may weaken later" category.

**B.1 — vector-aware tape.** Preserves determinism on three axes.

1. *ForwardMap cross-call purity (the load-bearing axis).* Vector-aware tape adds no hidden state, no statics, no RNG, no cross-call mutation — `Tape` is rebuilt per `evaluate(theta)` call, drops at end of scope. `evaluate` and `gradient` remain pure functions of `(theta, primal-trajectory)`. Cached BayesOpt and residual-GP training data per `§110/02-coupling/03-ml-chassis.md ¶57` stays valid across the optimization run.

2. *Within-walk bit-reproducibility, today.* Reverse walk is single sequential `for i in (0..=out_idx).rev()` (`sim/L0/ml-chassis/src/autograd.rs:369`). Post-B.1 generalized primitives are pointwise with shape-matched inputs (no broadcasting introduced — see axis 3); element-wise cotangent accumulation into parent `Tensor<f64>` slots is sequential per A.4 §4. **The audit does not pre-ratify any future slide.** B.1.a's `Send + Sync` bound is retrofit-cost hygiene (parallel to B.2's `op_id` hygiene for serialization) — admits *future* capability without committing to ship. If parallel-subtree reverse walk is ever proposed, that PR carries its own determinism argument (throughput gain vs. loss of bit-reproducible regression tests vs. A.4 tolerance band); it does not inherit a pre-baked OK from B.1.

3. *No new reductions in primitives.* Today's 9 scalar primitives (`add`, `sub`, `mul`, `neg`, `tanh`, `relu`, `square`, `ln`, `exp`) are all element-wise; B.1's "the same nine, generalized" are all element-wise pointwise on shape-matched `Tensor<f64>` inputs. No broadcasting in primitives, no reduction inside any primitive's forward or backward. Future broadcasting goes through `Custom(Box<dyn VjpOp>)` per B.1.a/B.2 — determinism is the user-impl's contract there (the B.2 audit question).

**B.1 → Group D flag (queued).** The 3 *fused* ops (`sum`, `mean`, `affine`) are today compositions of scalar primitives — `sum` is a left-to-right fold of `add` calls (`autograd.rs:285-292`), `mean` = sum × constant `1/n`, `affine` = per-row `mul`-then-`sum`-then-`add` (`autograd.rs:344-351`). Reduction order lives in tape topology, not in any node's backward, so today's `sum` is bit-deterministic. B.1 does not lock whether the fused ops stay compositions, become primitives with explicit forward reduction order, or migrate to sim-soft. Group D revisits placement; if any becomes a primitive with an internal forward reduction, the reduction order needs an explicit lock at that point.

**B.1.a / B.2 — `VjpOp` trait + `push_custom`.** Preserves determinism contractually. Chassis cannot enforce that user `VjpOp` impls are deterministic; it documents the contract and follows standard Rust trait-contract discipline (cf. `Hash`/`Eq` consistency, `Ord` totality — docstring + implementer's tests, no runtime enforcement). Two layers:

1. *Trait-level docstring on `VjpOp` (canonical, chassis).* Abstract umbrella + concrete examples + et-cetera, naming the tolerance band (cross-platform libm divergence and parallel-reduction non-associativity admitted within A.4 §4's 5-digit gradcheck tolerance) and the forbidden modes (RNG, wall-clock reads, mutable global state, captured `Rc<RefCell<...>>`, HashMap iteration order, env vars, file reads, etc. — anything that can vary across calls with the same inputs). One-line cross-reference on `Tape::push_custom`. Cites `ForwardMap`'s determinism-in-θ contract and the BayesOpt cache that depends on it.

2. *Authoritative paragraphs in the book.* §80 Ch 04 §02 `02-vjp-api.md` (architecture-book reader) and §110 Ch 02 `03-ml-chassis.md` (extends ¶57's determinism-in-θ paragraph to name the `VjpOp` substrate-side commitment). Both cite the chassis docstring as canonical.

*Layer 3 considered, declined.* A debug-build self-check inside `Tape::backward` (call each `Custom` VJP twice with identical inputs, assert within 5-digit tolerance) was proposed for maximum risk mitigation. Declined per the system-prompt rule "don't add validation for scenarios that can't happen; trust internal code; only validate at system boundaries" — `VjpOp` is an internal cross-crate trait, not a user-input boundary. Sim-soft's book-mandated gradcheck discipline (`110-crate/04-testing/01-regression.md`, demonstrated across PRs #188/#190/#192) already catches the high-magnitude failure modes; Layer 3's marginal coverage is a narrow band (sub-FD-noise non-determinism + cases-where-gradcheck-wasn't-written) at real iteration cost on sim-soft VJP authoring (doubled Newton-VJP cost in debug, where faer-Cholesky-touching impls dominate). Pattern-precedent risk: chassis-runtime-checks-user-impl scope-creeps to `Differentiable`, `Solver`, every future trait. **Revisit only if** a non-sim-soft VjpOp consumer ships without book-mandated gradcheck discipline (sim-rl custom losses, third-party wrappers) — at which point Layer 3 is justified by a real failure, not preemptive paranoia.

**B.1.a / B.2 → Queued book/code edits (audit-driven, lands at Group B close).** New sixth edit added to the existing B.5 queued bundle: `VjpOp` trait docstring lands in the chassis tape refactor PR; book §80 Ch 04 §02 and §110 Ch 02 paragraphs land alongside.

**B.3 — `Differentiable` trait + `CpuTape` alias.** Preserves determinism transitively, with one new contractual addition.

1. *No new arithmetic.* `Differentiable` is a placement decision (sim-soft, not chassis), not an arithmetic surface. Per-step VJP determinism inherits from B.1 + B.1.a/B.2; time-adjoint mechanism inherits from B.4 (separately audited below).

2. *`CpuTape` alias is identical at codegen.* `pub type CpuTape = sim_ml_chassis::Tape;` has no semantic difference from direct use; determinism is bit-identical. Bonus: Rust's orphan rule prevents sim-soft from adding inherent methods on the aliased type that could accidentally violate chassis tape invariants — *more* defensive against determinism leakage than newtype would be. Future newtype migration (B.3 sub-decision: on-trigger) would need to re-certify this surface.

3. *`register_vjp` signature change strengthens the contract.* Closure-based `register_vjp` (pre-B.2 spec) admitted captures of arbitrary mutable state with no documented contract. Post-B.2 `Box<dyn VjpOp>` carries the chassis `VjpOp` determinism docstring automatically; the contract follows the type.

4. **New — `Differentiable` trait docstring (audit-driven).** `forward(&mut self, ...)` / `backward(&mut self, ...)` admit per-call mutability, which is necessary for the expected sim-soft pattern: `forward` factors the Hessian and stashes it; `backward` reuses the factor for the IFT solve; tape lifetime drops it. *Intra-call* mutation is allowed and expected. But the trait signature cannot distinguish intra-call from *cross-call* mutation — an impl could silently cache between `evaluate` calls (warm-start a Newton iterate, accumulate statistics, hold a preconditioner "for performance") and break `ForwardMap`'s purity contract, invalidating the BayesOpt cache that `§110/02-coupling/03-ml-chassis.md ¶57` depends on. The chassis `VjpOp` docstring does not cover this — `Differentiable` impls live *above* the tape, in their own struct fields. The trait must carry a determinism docstring explicitly distinguishing intra-call (allowed: scratch, caches, factor stash that drops with tape lifetime) from cross-call (forbidden: state readable in a later call). Time-adjoint state-management discipline (B.4's mechanism) rolls into the same docstring — single source of truth, no split.

**B.3 placement flag (queued, not re-opening).** If sim-rl later picks up a higher-level differentiable consumer (custom value-function gradients, handcrafted policy-gradient terms beyond chassis primitives), the mitigation path is "promote `Differentiable` to a `sim-autograd-traits` shim crate," not "fork the trait into `sim_rl::Differentiable`." Recorded now to prevent drift between two parallel determinism docstrings if the situation ever materializes. Today's risk is hypothetical; shim crate stays premature per B.3's original rejection.

**B.3 → Queued book/code edits (audit-driven, lands at Group B close).** New seventh edit added to the queued bundle: `Differentiable` trait docstring lands when sim-soft autograd module is created; book §110 Ch 01 `00-core.md` carries the matching paragraph distinguishing intra-call from cross-call mutation discipline.

**B.4 — Newton replay + contact active-set.** Preserves determinism with explicit tolerance-band framing and one Group-D strengthening.

1. *Replay reproduces active set within A.4's tolerance band.* `04-checkpointing/02-tradeoff.md` §contact-active-set asserts "replay is bit-reproducible under this determinism assumption." That holds sequential same-machine same-θ per A.4 §4. Parallel paths (rayon for residual-norm reductions, per-tet stiffness assembly) fall under A.4's "parallel accepts non-associativity within 5-digit gradcheck tolerance" — active-set reproducibility is *within tolerance* for well-posed problems where pairs sit well within or outside the barrier $\hat d_k$. Near-boundary/ill-posed problems (pairs sitting near the barrier under ULP perturbations, topology changes) are out of B.4 scope and handled by `05-diff-meshing.md`'s topology-change machinery. Not a correction of the book — a citation-tightening so the tolerance band is explicit rather than hand-waved.

2. *RAII factor release is defensively determinism-safe.* `CheckpointedStepVjp::vjp` builds factor inside `&self` call, releases before return; primal-only payload means the factor is always *fresh from replay*, never carried across forward→backward→replay boundary. Eliminates a whole class of cross-call leaks a "store factor on tape" alternative would have admitted. Worth recording as a defensive property of the design, not just a memory choice.

3. *Phase H Brownian extension preserves the contract.* Phase D forward is deterministic in `(x, v, Δt, θ)`; Phase H extends to `(x, v, Δt, θ, Brownian-samples)` via payload extension per `00-uniform.md` §Brownian-path-storage. Same contract, extended input vector — no new determinism shape, just an additional input dimension.

**B.4 → Group D queued item strengthened.** Previously: "`Solver::replay_step(&self, ...)` signature deferred to Group D." Now: signature + determinism semantics + trait docstring. The signature alone is necessary but not sufficient — `&self` only enforces *call-site* immutability. The determinism contract requires that step output be a pure function of `(x_prev, v_prev, θ, Δt, Solver-immutable-config)`. Current `Solver::step(&mut self, ...)` admits per-call mutation; that mutation must be restricted to non-output-affecting concerns (logging, diagnostics, statistics) — never output-affecting state (adaptive tolerance learning across calls, cached line-search history influencing backtracking, warm-start iterate persisting across steps). Otherwise `step(&mut self, ...)` and `replay_step(&self, ...)` produce different results from the same physical inputs and replay-determinism is broken. Symmetric in shape to B.3's `Differentiable` finding.

*Audit habit, not template.* The walk has surfaced mutation-discipline gaps on two traits (`Differentiable` in B.3, `Solver` in B.4) by asking "is mutable state intra-call or cross-call, and does cross-call mutation affect output?" Groups C/D/E should apply the same *question* to traits they surface — `Observable` is read-only by design (likely no docstring needed), `SdfField` is likely `&self`-only (likely no docstring needed), `Preconditioner` is mixed `&mut self`/`&self` but build is intra-Newton-iteration (likely no cross-call concern), others TBD. The question is template-able; the docstring is per-trait. The audit does not pre-commit which remaining traits need their own determinism docstring — surface findings as you go, don't predict from n=2.

**B.4 → Queued book/code edits.** Eighth queued edit: `Solver` trait docstring at §110 Ch 01 `00-core.md`, landing with Group D close (alongside the previously-queued `replay_step` signature edit). Distinguishes output-affecting mutation (forbidden) from logging/diagnostics mutation (allowed) on `step`; commits `replay_step` to pure-function semantics.

**B.5 — GPU atomic-add f32 non-associativity.** Preserves determinism within A.4 §4's parallel-path tolerance band, operationally enforced by §110 Ch 04 §03, with explicit tolerance-band carry-over for the deferred f64-emulation variant.

1. *f32 atomic-add CAS is a parallel path per A.4 §4.* `atomic_add_f32` at `80-gpu/03-gpu-autograd/01-playback.md:61-71` is structurally parallel — GPU workgroup scheduling is hardware-determined, CAS retry order varies run-to-run even on identical hardware, and contributions from $k$ downstream consumers land in non-deterministic order. A.4 §4's parenthetical `(rayon)` is exemplification, not exhaustive enumeration; GPU workgroup scheduling falls under the same umbrella. Operational cite: §110 Ch 04 §03 line 92 is A.4 §4 re-expressed at the test level — *"Reduction order is part of the test: a GPU kernel that sums in a different order from CPU will agree to ~5 digits but not to 8 or more, and the 5-digit bar is tuned around that reality."* The tolerance band is not abstract; it's the bar the CPU-vs-GPU gradcheck uses.

2. *Contention cost profile does not change determinism shape.* Per `01-playback.md:74-77`: non-contended ≈3× native f32-add (first-iteration CAS succeeds); contended at mesh valence (≈8–15 tets/vertex) ≈3.5–4× native with ≈2–5% retry rate. Retries introduce additional non-associativity paths (each retry re-reads a different `old_bits` → different `new_val` summation order) but the same $k$ contributions land — only the accumulation order varies. Still within §03's 5-digit band for per-vertex gradient scatter, where retry rate is highest in the canonical forward graph.

3. *`atomic_add_f64` emulation (deferred) inherits the tolerance-band contract, not a pre-ratified implementation.* The `01-playback.md:79` note defers implementation: *"either a double-word compare-exchange (where supported) or a double-single Kahan-style split (where it is not). Phase E will benchmark both on target hardware; Pass 1 defers the specific choice to the implementation."* B.5 does not pre-ratify either path. B.5 does lock the contract: any f64-emulation implementation satisfies A.4 §4's tolerance band, operationally enforced by §03 on every PR at Phase E. The implementation PR carries its own impl-time determinism argument (same template as B.4's Phase H Brownian extension — extension, not re-litigation). CI's §03 gate catches a violating implementation directly; the deferred variant does not require a defensive marker at `01-playback.md:79`.

**B.5 → Group C pre-load (not a new B queued edit).** CPU VJP authors get a chassis `VjpOp` trait docstring per B.1.a/B.2 (queued edit #6). GPU VJP authors register a `VjpKernelHandle` keyed by `(KernelId, GpuScalarKind)` per B.5/B.5.a — there is no Rust-side trait object carrying a docstring. The symmetric-of-B.1.a/B.2 author contract has to live either on `VjpRegistry::register`'s Rust-side docstring or as a `01-gpu-backend.md` paragraph (or both). Substance pre-locked by this walk: deterministic within A.4 §4 tolerance; forbidden modes on GPU specifically — non-deterministic subgroup operations whose shuffle lanes depend on physical lane assignment, readback of vendor driver state, uninitialized workgroup-shared-memory reads, non-IEEE-deterministic intrinsics. Group C walks `01-gpu-backend.md` end-to-end and decides placement.

**B.5 → Queued book/code edit (audit-driven, lands at Group B close).** New eighth queued edit: `03-gradcheck.md` §01 closing prose — §01's bit-equal assertion scoped explicitly to CPU (GPU atomic-add non-associativity precludes bit-equal); GPU determinism-in-θ has two risk surfaces covered by (a) trait contracts (impl-side, primary, per B.3 docstring) and (b) §03's 5-digit bar (secondary, catches gross drift but runs one GPU forward only, so does not directly exercise GPU cross-call state); Phase-E GPU-side §01 counterpart (two GPU evaluations at same θ, 5-digit relative tolerance) flagged as candidate Phase E deliverable; infrastructure-side cross-call state (`GpuTensorPool` reuse, etc.) subject to separate acquire-use contract specified at Group C. Surfaced via stress-test pushback ("does §01's bit-equal hold on GPU?") which revealed §03's single GPU forward does not transitively cover GPU cross-call state the way §01 covers CPU cross-call state.

**B.5.a — `GpuScalar` mixed-precision boundary.** Preserves determinism within A.4 §4's tolerance band; precision gap framed via §110 Ch 04 §03 oracle, not re-derivation. Inherits B.5's f32 atomic-add contract directly at Phase E.

1. *CPU-f64 → GPU-f32 marshal is IEEE-deterministic on the cast; GPU-side arithmetic carries cross-driver divergence within A.4 §4's tolerance band.* The `from_cpu_f64_as_f32` cast (queued book edit #4) is Rust `as` = IEEE round-ties-to-even. Pure function of input f64 bytes; no CPU-side determinism surface. Downstream f32 WGSL arithmetic carries cross-driver divergence from two sources: (a) subnormal-FTZ behavior is WGSL-impl-defined per driver (A.4 §2's CPU-side IEEE-correct commitment does not transfer — Rust controls subnormal mode, WGSL does not), (b) transcendental ops (`sin` / `cos` / `exp` / `pow`) have up-to-2-ULP divergence per WGSL spec. Both bounded by §03's 5-digit tolerance.

2. *Storage-f64 / arithmetic-f32 CG reduction inherits B.5's f32 atomic-add contract directly at Phase E.* Per `80-gpu/02-sparse-solvers/00-cg.md:90`: CG runs f32 SpMV + f64 storage-only accumulators with Kahan compensated summation. At Phase E, f64 is storage-only per B.5.a's scalar-admission-vs-kernel-availability split — no f64 arithmetic kernels ship. The CG inner-product reduction executes as f32-atomic-add on GPU with the Kahan error term read/written in f64 buffer slots. B.5's f32 atomic-add determinism argument applies now, not when future f64-emulation lands. Carson-Higham 2018 iterative refinement recovers 5-digit precision; §03 enforces.

3. *§03 CPU-vs-GPU gradcheck is the operational oracle for the precision gap.* `03-gradcheck.md:92` tunes the 5-digit bar around reduction-order non-associativity explicitly (*"the 5-digit bar is tuned around that reality"*). A PR that pushes the precision gap outside 5 digits fails §03 directly. Scene-size scoping per `03-gradcheck.md:92` caps accumulation-density below the bar. The audit entry cites enforcement rather than re-deriving the precision argument.

4. *`GpuScalar` sealing preserves the contract at the scalar-admission boundary.* f16/bf16 admission requires chassis-level WGSL kernel pairing per B.5.a's sealed-trait design; PR review surfaces the precision decision naturally because the chassis-level code change is visible and sealing prevents silent admission. f16's precision gap is substantially wider than f32's — admission is a real decision, not rubber-stamp. Sealing IS the mechanism; the audit expectation of impl-time precision audit is policy prompted by the sealing structure, not a defensive marker.

**B.5.a → Group C pre-loads (accumulating from B.5).** Two additional GPU cross-call state concerns surfaced during walk, joining B.5's GPU VJP author contract pre-load:

- **GPU `Preconditioner` cache scope.** `03-preconditioning.md` governs the `Preconditioner` trait (chassis-owned per B.5's locked decision). Phase-E IFT has design pressure toward caching the preconditioner across `evaluate()` calls for warm-start on perturbed θ — a real performance temptation that would silently violate ForwardMap purity. Group C specifies whether the `Preconditioner` trait contract forbids cross-call caching (docstring per the B.1.a/B.2 / B.3 / B.4 mutation-discipline pattern, applying the audit-habit-not-template question).

- **`GpuTensorPool` cross-call reuse.** `GpuTensorPool` per B.5 is a shape-keyed free list. If buffers from call $N-1$ are returned to the pool and popped for call $N$ without explicit zero-init, kernels reading-before-write produce non-deterministic content leakage. Group C specifies the acquire-use contract (zero-on-alloc vs. write-before-read kernel-author contract).

All three pre-loads (B.5 GPU VJP author contract + these two) fall under "GPU cross-call state" — impl-side concerns (`Differentiable` / `ForwardMap`) handled by trait-contract-primary defense per B.3 audit; infrastructure-side concerns (these three) handled by Group-C-specified acquire-use / trait-contract patterns. Per queued book edit #8's framing: trait contract for impl-side, Group C specifies for infrastructure-side.

**B.5.a does not surface new queued edits for Group B close.** Book edit #4 (B.5 locked) already carries the `GpuScalar` spec + CPU-f64→GPU-f32 cast note.

**B.5 / C.1 — GPU VJP author contract.** Preserves `ForwardMap` determinism contract contractually, same model as B.1.a/B.2 (CPU `VjpOp` trait docstring) and B.3 (`Differentiable` trait docstring). Chassis cannot enforce kernel-internal determinism at the type level on GPU any more than on CPU; discipline is trait-contract-and-gradcheck, not runtime-checked. A.4 §4's tolerance band (≤2 ULP transcendentals inside band; vendor-lane-dependent subgroup shuffle outside band, forbidden) carries over from CPU to GPU. §110 Ch 04 §03 CPU-vs-GPU gradcheck is the operational oracle per B.5's walk.

Dual-surface placement (Rust docstring + book paragraph, §02 canonical, cross-references from §01 and from `VjpRegistry::register` docstring) matches the B.3 pattern: canonical source + cross-references, no duplication. Layer 3 runtime self-check was considered and declined at B.1.a/B.2 walk for CPU; same rationale applies on GPU (scope-creep risk + book-mandated gradcheck already catches high-magnitude failures). **No silent weakening.**

**C.2 — Preconditioner cross-call cache scope.** Preserves `ForwardMap` determinism contractually by forbidding output-affecting cross-call mutation at the trait level. Intra-call state (AMG pattern cache across Newton iterations, book-committed) is allowed and expected; state readable in a later `evaluate()` call is forbidden. Same mutation-discipline pattern as B.3 `Differentiable` and B.4 `Solver::step` at chassis-owned trait level — audit-habit symmetry preserved.

Pattern-cache-drift silent-weakening risk (a buggy impl claiming idempotent-re-setup but failing within §03's 5-digit band) is closed at contract level rather than impl discipline: a correct impl satisfies by construction (fresh instance per `evaluate()`); a drifting impl violates a simple contract that is catchable in code review. §03 gradcheck's correctness-vs-performance blind spot is not load-bearing — contract is.

Phase-E `PatternCachedPreconditioner<P>` wrapper is a *future* extension carrying its own determinism argument per the B.4 Phase-H Brownian extension template — extension, not re-litigation. Pass 1 does not ship; does not pre-ratify.

A.4 §4 tolerance band unchanged. §110 Ch 04 §03 CPU-vs-GPU gradcheck is the operational oracle for the trait contract, not for impl-level pattern-cache equivalence. **No silent weakening.**

**C.3 — GpuTensorPool acquire-use contract.** Preserves `ForwardMap` determinism at the pool API level via a hybrid safe-default-plus-opt-out split. `acquire` returns bit-identical-to-zero buffers regardless of pool history — cross-call content leakage closed by construction. `acquire_uninit` returns undefined content with a write-before-read kernel-author contract documented at §80 Ch 04 §02; violations are kernel-author bugs, catchable in gradcheck when stale content is numerically non-zero, *explicit at the acquire call site* (grep-able audit surface).

Same risk-mitigation pattern as C.2: safety-critical discipline moved from kernel-author-attention (distributed, gradcheck-blind-spot-prone) to API choice (concentrated, grep-able). Pure Option A (zero everything) declined for perf cost at scene scale; pure Option B (write-before-read always) declined for silent-footgun risk; Option D (debug-build poison-pattern) declined for Layer-3-pattern-precedent scope creep. Hybrid is the Rust-idiomatic safe-default-plus-opt-out pattern.

A.4 §4 tolerance band unchanged. §110 Ch 04 §03 gradcheck is the operational oracle for kernel-contract violations that produce within-5-digit drift; explicit `acquire_uninit` audit surface catches API-level misuse in code review. **No silent weakening.**

**C.4 — Current sim-gpu crate fate.** Preserves `ForwardMap` determinism contract trivially: sim-gpu does not participate in the contract at all. Its determinism surface is sim-core-step-deterministic (mujoco-equivalent rigid-body integration), not per-`evaluate(θ)` autograd-deterministic. The three GPU-cross-call-state pre-loads resolved at C.1/C.2/C.3 (chassis `VjpRegistry::register` author contract, chassis `Preconditioner` trait contract, chassis `GpuTensorPool::acquire`/`acquire_uninit` API contract) live on `sim_ml_chassis::gpu`'s future surface; sim-gpu's surface is orthogonal — different crate, different consumers, different oracle (CPU-vs-GPU parity tests at `sim-gpu/src/collision.rs::trace_contacts_match_cpu` for narrowphase and `sim-gpu/src/pipeline/tests.rs::t31_gpu_vs_cpu_trajectory` for full pipeline, scoped to A.4 §4 tolerance band locally; not §110 Ch 04 §03 which is the soft-body autograd gradcheck).

A.4 §4 tolerance band unchanged on the sim-gpu surface. Multi-Device design (sim-gpu's own `wgpu::Device` separate from Bevy-render and from chassis-gpu) is intentional per `sim-gpu/src/context.rs:5-7` and per `01-gpu-backend.md:107` (chassis ships its own `GpuDevice`); cross-Device coordination happens at the MJCF coupling boundary (CPU sync) per `110-crate/02-coupling/00-mjcf.md`, not in the hot Newton loop, so the multi-Device structure does not introduce new determinism surfaces beyond what each Device already satisfies independently. **No silent weakening.**

**D.1 — `Solver` trait docstring + `replay_step` signature.** Preserves `ForwardMap` determinism contract at two layers.

1. *`step` mutation discipline at the trait level.* Trait docstring forbids output-affecting cross-call mutation on `step(&mut self, ...)`; admits intra-call logging / diagnostic / telemetry mutation conditional on "must not affect returned `NewtonStep`." Audit-habit symmetry preserved with B.3 `Differentiable`, B.4 `Solver::step` (substance moved here from B.4 pre-load), C.2 `Preconditioner`.

2. *`replay_step` as pure function.* `&self` signature combined with docstring commitment that any `&self`-state reads are constructor-only (solver config, scene references) — nothing that depends on prior call history. Given the same `(x_prev, v_prev, theta, dt)`, `replay_step` produces the same `NewtonStep<Self::Tape>` as the forward `step` modulo A.4 §4 tolerance. The checkpointed-step VJP from B.4 rests on this purity; `02-tradeoff.md:49`'s bit-reproducibility commitment is the operational contract, and it transits through D.1 at the trait-surface level.

A.4 §4 tolerance band unchanged. §110 Ch 04 §03 gradcheck is the operational oracle for `replay_step` vs `step` bit-reproducibility divergence within tolerance (per `00-uniform.md:37`'s "replay reproduces the same trajectory byte-for-byte from the stored primal state"); trait-contract is the contractual enforcement. **No silent weakening.**

**D.2 — Trait-placement + mutation-discipline walk across remaining trait surfaces.** Preserves `ForwardMap` determinism contract at two layers.

1. *ForwardMap trait-surface commitment (D.2.a).* New trait-level docstring at `03-ml-chassis.md` forbids output-affecting cross-call `&mut self` mutation on `evaluate` and `gradient`; admits intra-call mutation conditional on non-persistence past the call. Audit-habit symmetry completes the four-trait pattern: B.3 `Differentiable` (sim-soft), D.1 `Solver` (sim-soft), C.2 `Preconditioner` (chassis), D.2.a `ForwardMap` (sim-soft) — uniform mutation discipline across chassis-owned + sim-soft-owned impl-side traits at trait-contract level. Closes the pre-existing book-vs-code gap at `03-ml-chassis.md:61` ("surfaces this contract verbatim in the `ForwardMap` docstring"), previously unfulfilled.

2. *`&self`-only traits surface no determinism leak via trait signature (D.2.b–d).* Observable / Material / Element / Mesh / ContactModel all `&self`-only; trait signatures do not admit cross-call mutation. Interior mutability via `RefCell` / `Mutex` is a general Rust-idiom concern catchable at the enclosing `ForwardMap` contract (now trait-docstring-enforced per D.2.a), not requiring per-trait docstrings. `SdfField` is a struct, not a trait — no trait-placement question. Matches B.4's audit-habit-not-template prediction ("Observable is read-only by design, likely no docstring needed"; "SdfField is likely `&self`-only"); finding confirmed per-trait rather than pre-committed.

A.4 §4 tolerance band unchanged. §110 Ch 04 §03 gradcheck is the operational oracle for violations of the `ForwardMap` determinism-in-θ contract; trait-surface docstring is the contractual enforcement. Observable's transitivity subtlety (sim-bevy consumer has no determinism-in-θ requirement; optimization-pipeline consumer does) does not introduce a new determinism surface — the requirement follows the consumer, not the trait. **No silent weakening.**

**D.3 — Coupling patterns walk.** Preserves `ForwardMap` determinism contract at two levels.

1. *Handshake-trait mutation discipline is ForwardMap-transitive (D.3.a, D.3.b).* Both `MjcfHandshake::import_rigid_state(&mut self)` and `ThermostatHandshake::import_temperature(&mut self)` admit per-handshake-tick mutation by design (store boundary data for the next Newton solve). Cross-evaluate-call mutation discipline is ForwardMap's contract (D.2.a); handshake traits inherit transitively. Same structural argument as Observable (D.2.b). Book commits (line 28 of `00-mjcf.md` + line 81 + Phase F; line 23 of `01-thermostat.md` + line 83) that handshake consumers are the ForwardMap-orchestrated outer coupling loop; no Phase A–I standalone consumer opens a leak path.

2. *Multi-Device CPU-sync claim extends cleanly (D.3.c).* All cross-crate data transit between sim-soft ↔ sim-core and sim-soft ↔ sim-thermostat is CPU-resident boundary data (plain Rust structs or CPU trait objects). sim-thermostat is CPU-only per Cargo.toml verification (pattern #8); sim-core's GPU acceleration and sim-soft's Phase-E GPU Newton each use their own `wgpu::Device`; no hot-loop cross-Device traffic. C.4's "hot Newton loop stays single-Device per crate" holds across both couplings without amendment.

Each crate's own deterministic integrator composed with deterministic fixed-point-iterate-count-from-initial-partner-state plus deterministic subcycled boundary-data reductions (time-averaging for thermostat-faster, time-integration for sim-soft-faster) equals composed determinism. `00-mjcf.md:73-75` and `01-thermostat.md:66-67` both commit this explicitly. No RNG enters either coupling loop. Fixed-point retry-cap behavior (`00-mjcf.md:24`'s adaptive-Δt escape hatch) is deterministic in initial partner state per line 74.

A.4 §4 tolerance band unchanged. §110 Ch 04 §03 gradcheck transitively covers coupled evaluations (the ForwardMap oracle doesn't distinguish single-crate from multi-crate evaluations). **No silent weakening.**

**D.4 — IO ingress: URDF geometry NaN/Inf validation.** Preserves `ForwardMap` determinism contract trivially by closing an earlier boundary. `validate_geometry` is a pure function of `&UrdfRobot` with zero cross-call state — no new determinism surface introduced. The gap it closes was not itself a determinism leak (NaN-propagating geometry would produce NaN gradients, catchable by gradcheck at the 5-digit bar) but an earlier-boundary hygiene win; rejecting at URDF ingress gives clearer error messages before downstream mesh/collision code produces confusing failures. Matches A.4 §1's validate-at-system-boundary commitment directly.

A.4 §4 tolerance band unchanged. D.4.b's workspace clippy hygiene (const-fn promotions, `is_multiple_of` substitutions) is determinism-neutral — `const fn` is a compile-time-evaluability assertion, no runtime semantic change; `n.is_multiple_of(3)` and `n % 3 == 0` are semantically equivalent for `usize` inputs with a nonzero divisor, which is all the hygiene sites satisfy. **No silent weakening.**

**E.1 — sim-opt as gradient-free consumer.** Preserves `ForwardMap` determinism contract trivially: sim-opt does not participate in the contract at all. Its determinism surface is Metropolis-accept/reject deterministic-in-RNG-seed (per chain, per epoch); fitness evaluation flows through `evaluate_fitness(env, policy, params, max_episode_steps)` (`sim/L0/opt/src/algorithm.rs:192-213`) which calls `collect_episodic_rollout` after `env.reset_all()`. Purity-in-θ at sim-opt's fitness-function level rests on `VecEnv::reset_all` being deterministic-for-seed and `Policy::set_params` + `Policy::forward` being pure-in-(params, obs) — both upstream-consumer contracts ratified at E.2/E.4 (sim-rl, sim-therm-env), not new sim-opt surface. sim-opt's pass-through consumption of chassis introduces no determinism surface of its own.

E.1's hygiene commit `f614cd77` (workspace clippy: 5 × `missing_const_for_fn` in sim-ml-chassis, 1 × `manual_is_multiple_of` in sim-rl, 1 × `manual_is_multiple_of` in sim-opt) is determinism-neutral by the same argument as D.4.b's parallel hygiene at `d12a5e73` — `const fn` is compile-time-evaluability, and `n.is_multiple_of(k)` ≡ `n % k == 0` for nonzero usize divisors (sim-opt's median uses constant `k = 2`; sim-rl's td3 policy-delay check uses `hp.policy_delay`, where `% policy_delay` would already be UB if `policy_delay == 0` so the carry-over preserves the existing precondition).

A.4 §4 tolerance band unchanged. **No silent weakening.**

**E.2 — sim-therm-env as env-surface consumer.** Preserves `ForwardMap` determinism contract transitively, same template as E.1.

1. *Non-participation in the ForwardMap contract directly.* sim-therm-env implements chassis `Environment`, not sim-soft `ForwardMap`. Its own determinism surface is the chassis `Environment::step` contract (deterministic-in-`(model, data, action, rng-seed)` via sim-core's integrator + sim-thermostat's `LangevinThermostat`). sim-therm-env is structurally upstream of ForwardMap participation; any RL consumer's ForwardMap-adjacent determinism story ratifies at its own walk (E.3 / E.4), not at sim-therm-env's Environment-impl surface.

2. *Transitive consumption via VecEnv/SimEnv opacity.* sim-therm-env's `Environment` impl is called from inside `VecEnv::step` (when built via `build_vec`) or via chassis `SimEnv` wrapping (when built via `build`) — both paths route through `collect_episodic_rollout` in upstream RL consumers. Tensor boundary stays f32-concrete per A.1 sub-decision B-2; no new cross-call-state surface is introduced by sim-therm-env's integration heaviness — the heaviness is compositional (wiring sim-core + sim-mjcf + sim-thermostat), not autograd-stateful.

3. *A.1 mechanical rename is determinism-neutral.* `Tensor<f32>` is bit-identical to today's `Tensor` at every operational level; rename is type-level only, zero arithmetic or control-flow change across the 4 prod + ~28 test sites.

4. *E.2 hygiene commit `3a45d50f` is determinism-neutral.* Cargo.toml `#`-comment additions only; zero code change, zero build-system change beyond grader-visible dep-justification compliance.

5. *Queued A.4 §1 builder-setter gap, when fixed, strengthens rather than weakens.* Rejecting NaN/Inf `timestep`/`gamma`/`k_b_t`/`ctrl_range` at construction replaces silent propagation (NaN flows into the MJCF text via `format!` → sim-mjcf parse → physics state → observation/reward, surfacing only as algorithm-level NaN failures in upstream consumers) with typed-error rejection at the A.4 §1 boundary. Closing an earlier boundary, per D.4's precedent framing.

A.4 §4 tolerance band unchanged. Queued Coverage F finding is a grader-visibility concern, orthogonal to determinism (grader measures line coverage; coverage is not a determinism property). **No silent weakening.**

**E.3 — autograd_policy / autograd_value / autograd_layers as chassis-internal ForwardMap-adjacent modules.** Preserves `ForwardMap` determinism contract transitively and structurally — these are chassis-autograd internals that consumer RL code uses via trait-object interfaces; they are ForwardMap-adjacent by construction (tape is the substrate ForwardMap's `gradient()` walks) yet do not themselves impl `ForwardMap`.

1. *Non-participation in the ForwardMap contract directly.* E.3 modules (`AutogradPolicy`, `AutogradStochasticPolicy`, `AutogradValue`, `AutogradQ`) impl chassis `Policy` / `DifferentiablePolicy` / `StochasticPolicy` / `ValueFn` / `QFunction` — RL gradient-in-params trait hierarchy. Own determinism surface is `Policy::forward` deterministic-in-(params, obs), `ValueFn::forward` deterministic-in-(params, obs), `QFunction::forward` deterministic-in-(params, obs, action). All five trait contracts inherit A.4 §4's "same θ on same machine on sequential path is bit-reproducible" modulo the IEEE propagation argument below. No ForwardMap `evaluate(θ)` surface; any RL consumer's ForwardMap-adjacent determinism story ratifies at E.4 (sim-rl baselines), not at E.3 modules' trait-impl surface.

2. *Tape-lifetime purity per call.* Every forward / log_prob_gradient / forward_vjp / mse_gradient / action_gradient method in E.3 builds a fresh `Tape` via `Tape::new()`, runs forward (builds the graph), runs `backward(output)` if gradients requested, reads `tape.grad(var)` per parameter, drops `Tape` at end of scope. Same per-call-scope tape-lifetime discipline as B.1's "`Tape` is rebuilt per `evaluate(theta)` call, drops at end of scope" (cross-cutting determinism section axis 1). No hidden state, no statics, no tape-instance reuse, no RNG in forward-pass (RNG enters only at construction via `new_xavier`, which produces deterministic params-for-seed via the Box-Muller `randn` helper at `autograd_policy.rs:63` / `autograd_value.rs:59` + Glorot-uniform / He-normal sampling per `Activation::Tanh` / `Activation::Relu` branch).

3. *B.1 migration is determinism-neutral under either implementation path.* **Scalar-sugar-preserved path:** zero code change, zero arithmetic change — bit-identical to today's f64 scalar arithmetic. **Tensor-first wrap/unwrap path:** ~102-site mechanical rename at E.3 scope; IEEE semantics of the scalar arithmetic operating on shape-`[]` tensors is bit-identical to f64 scalar arithmetic per B.1's element-wise-primitive commitment (cross-cutting determinism section axis 3: "all element-wise pointwise on shape-matched `Tensor<f64>` inputs"). No new reductions introduced at primitive level; no parallel paths admitted; no cross-call-state surface added. Either path: determinism-neutral.

4. *B.3 `DifferentiablePolicy` disambiguation from sim-soft `Differentiable`.* Chassis `DifferentiablePolicy` at `sim-ml-chassis/src/policy.rs:80` is the RL trait (`log_prob_gradient`, `forward_vjp`, `forward_vjp_batch` — gradient-in-params); sim-soft's `Differentiable` is the physics trait (`NewtonStep<Self::Tape>`, IFT adjoint, time-adjoint). Explicitly disambiguated at B.3 line 169 ("No rename of existing `DifferentiablePolicy`"). Both traits preserve cross-call mutation discipline at their respective trait levels: chassis `DifferentiablePolicy`'s methods take `&self` (read-only — no cross-call state admitted by signature), which is stronger than B.3's `Differentiable` `&mut self` audit-habit-intra-call-allowed discipline. E.3 modules inherit the stronger form trivially. **No new docstring addition needed at E.3** beyond B.3's already-queued brief doc note that lands with sim-soft crate creation.

5. *Non-participation in custom-VJP surface.* Zero `push_custom` / custom `VjpOp` calls in E.3 modules. All gradients compose from the 9 scalar primitives (`add` / `sub` / `mul` / `neg` / `tanh` / `relu` / `square` / `ln` / `exp`) + 3 fused ops (`affine` / `sum` / `mean`). Forward determinism inherits from composition of IEEE-correct primitives per A.4 §4. B.1.a/B.2's custom-VJP author contract (chassis `VjpOp` docstring, Group B close queued) is a new-surface contract with zero E.3-side users; it does not apply to E.3 migration determinism.

6. *FD gradcheck tests inside E.3 are the chassis-internal scalar determinism oracle.* Three E.3 `#[cfg(test)]` modules carry forward + gradient regression checks: `autograd_policy.rs` (forward parity against hand-coded MLP oracle + multi-layer FD gradient checks + stochastic policy FD checks + Xavier-init nonzero assertion) + `autograd_value.rs` (value / Q forward + MSE-gradient + action-gradient parity against `MlpValue` / `MlpQ` oracles) + `autograd_layers.rs` (`linear_{tanh,relu,raw}_gradient_matches_fd` + `mse_loss_*` + `gaussian_log_prob_gradient_matches_fd` + forward-parity against MlpPolicy) at `PARITY_TOL = 1e-10` for oracle parity and `FD_TOL = 1e-5` for FD cross-check. Together these constitute the scalar-autograd determinism regression corpus. Tighter than §110 Ch 04 §03's 5-digit CPU-vs-GPU gradcheck (that's sim-soft territory with parallel-reduction non-associativity); scalar autograd at `PARITY_TOL = 1e-10` holds because arithmetic is strictly sequential and IEEE-deterministic. The 362-test chassis library corpus (from Option-B grade baseline) includes these FD checks; F.2's tier (a) pre-merge chassis refactor validation runs them as the first regression-defense line.

A.4 §4 tolerance band unchanged. §110 Ch 04 §03 gradcheck not in play at E.3 (chassis-internal scalar autograd, CPU-only, no GPU parity). **No silent weakening.**

**E.4 — sim-rl baselines as algorithm consumers of chassis traits.** Preserves `ForwardMap` determinism contract transitively, same template as E.1 sim-opt and E.2 sim-therm-env.

1. *Non-participation in the ForwardMap contract directly.* sim-rl impls chassis `Algorithm` (5 baselines: CEM, REINFORCE, PPO, TD3, SAC), not sim-soft `ForwardMap`. Algorithm trait signature is invariant under A.1 (5 methods: `name`, `train`, `policy_artifact`, `best_artifact`, `checkpoint`; zero Tensor in any signature, all referenced structs use `Vec<f64>` for params). Own determinism surface is `Algorithm::train` deterministic-in-`(env, budget, seed, on_epoch)`: each baseline initializes its own `StdRng::seed_from_u64(seed)` for action sampling, replay-buffer indices, exploration noise, target-network polyak averaging — RNG progression is fully observable + reproducible. Inherits A.4 §4's "same θ on same machine on sequential path is bit-reproducible" via composition of `Policy::forward` / `DifferentiablePolicy::log_prob_gradient` / `StochasticPolicy::*` / `ValueFn::forward` / `QFunction::forward` deterministic-in-(params, obs[, action]) consumed contracts. No new ForwardMap `evaluate(θ)` surface introduced.

2. *Pass-through transitive consumption of VecEnv/SimEnv contracts.* sim-rl algorithm `train` loops call `env.step(&action_tensor)` (`sac.rs:321` + `td3.rs:313` explicitly; cem/reinforce/ppo via `sim_ml_chassis::rollout::collect_episodic_rollout`); `env.reset_all()` is called via the same rollout helper. Tensor boundary stays f32-concrete per A.1 sub-decision B-2; sim-rl introduces no cross-call-state surface in either prod algorithm code or test fixtures. Algorithm-level state (replay buffers, RNG, best-tracker, network snapshots, optimizer momentum) is intra-algorithm-instance, never cross-call-of-the-same-algorithm-evaluated-twice — purity-in-θ holds because `Algorithm::train` IS the multi-call composition (one `train` call evaluates many `(θ, obs)` pairs as part of its own training trajectory; cross-call meaning re-running `train` from the same `(env, budget, seed)` produces bit-identical outputs).

3. *A.1 mechanical surface is determinism-neutral under either path.* **Type-inference path (recommended/expected):** zero source change at all 5 sim-rl Tensor sites; arithmetic is bit-identical because `Tensor<f32>` IS today's `Tensor` operationally (rename is type-level only, monomorphizes to the same f32 path). **Explicit-turbofish fallback path:** 2-line addition at sac.rs:321 + td3.rs:313; same arithmetic, same control flow, same RNG consumption order. Either path: bit-identical algorithm output for same `(env, budget, seed)` inputs across the chassis A.1 PR boundary.

4. *Algorithm-output-determinism is the empirical floor at F.2 tier (b).* The 13 `#[ignore]`-gated `tests/competition.rs` competition fixtures (CEM/REINFORCE/PPO/TD3/SAC × Linear/MLP/Autograd1Layer/Autograd2Layer policy levels, plus sim-opt's `d2c_sr_rematch{,_richer_sa,_pt}.rs` fixtures at F.2 tier (c)) collectively gate the algorithm-output bit-equivalence story for chassis A.1 + B.1 refactor. F.2 tier (b)'s sim-rl test suite (44 fast tests) catches any silent RNG-reorder / API-shape regression at minutes-scale before tier (c)'s ~1.5-3h fixture validation.

5. *Non-participation in custom-VJP surface.* Zero `push_custom` / custom `VjpOp` calls in sim-rl. ppo.rs's local scalar `gaussian_log_prob` helper (line 188) is a fold of f64 `log` + `mul` + `add` — pure scalar function, no Tape contact. sac.rs's `sim_ml_chassis::stats::gaussian_log_prob` import is the scalar-flavored chassis helper (distinct from `autograd_layers::gaussian_log_prob` which is tape-flavored per E.3's separation finding). B.1.a/B.2's custom-VJP author contract (chassis `VjpOp` docstring, Group B close queued) is a new-surface contract with zero sim-rl-side users; it does not apply to sim-rl migration determinism.

A.4 §4 tolerance band unchanged. **No silent weakening.**

## Tomorrow's gameplan

1. **Start with Group A (numerical/type foundations + numerical policy).** Smallest domain; establishes the walk rhythm before harder groups. Expected coverage:
   - Inventory `sim/L0/types/` public API
   - Inventory `sim/L0/simd/` public API
   - Inventory `sim-ml-chassis::Tensor` surface
   - **Decision:** f32 vs f64 strategy — policy-network path stays f32? sim-soft exclusively f64? dual-backed tensor? some third thing?
   - **Decision:** faer integration state — version, features, workspace-dep convention
   - **Decision:** nalgebra integration state — same questions
   - **Decision:** IEEE754 / NaN / denormal / tolerance policy across the platform
2. **Then B, C, D, E, F in that order.** B and D are the load-bearing middle; E and F mostly fall out of the upstream decisions.
3. **Cadence:** interactive, step-by-step with user. User argues, Claude refines. Decisions captured in this document as we make them. Book edits land per methodology above.
4. **Exit condition for the audit:** a PR-sized-chunk dependency graph with explicit sequencing and deferrals — enough information to begin the refactor PRs in sensible order. **Not** complete implementation plans for each chunk; those are per-PR scope memos, downstream of this audit.

Expected cadence: day or two of review + edit work. No compression; "planning SOTA architecture is the work."

## Not tonight

- No inventory data pulled.
- No decisions made.
- No book edits.
- No refactor code.

Parking memo only. Pick up tomorrow at "Start with Group A" above.
