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

**Status:** C.1 locked. C.2, C.3, C.4 pending.

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
