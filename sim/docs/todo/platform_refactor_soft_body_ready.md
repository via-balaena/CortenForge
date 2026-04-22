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

**Status:** Partial — B.1, B.1.a, B.2, B.3, B.4 locked (B.4 2026-04-22). B.5 (GPU-tape readiness) and the cross-cutting determinism audit remain. B.4 carries one queued dependency for Group D (`Solver` replay-method signature).

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
