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

**Status:** D.1, D.2, D.3 locked. Groups D.4–D.5 remaining.

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
