# Platform refactor — soft-body readiness

**Status:** Scoping phase. Contents tonight are the **audit gameplan only** — the actual inventory, book-vs-code diff, breaking-change enumeration, and sequencing work begins next session. This memo is the parking position.

**Branch:** `feature/soft-body-walking-skeleton` (continuation).

**Relationship to other memos:** This audit is *upstream* of [`soft_body_walking_skeleton_scope.md`](soft_body_walking_skeleton_scope.md), which is now marked SUPERSEDED pending this audit's completion. The walking skeleton's six invariants all assumed a soft-body-ready L0 platform that does not currently exist; this memo scopes the refactor that makes them true.

## Why this exists

`sim-soft` cannot be built on the L0 platform as it stands today. Known deficiencies surfaced in the 2026-04-21 session:

- `sim-ml-chassis::Tape` is scalar-only `f64` with a closed set of 9 primitives + 3 fused ops; no custom VJP registration; no vector cotangents.
- `sim-ml-chassis::Tensor` is `f32`; the book assumes `Tensor<f64>`.
- `faer::Llt<f64>` factor-on-tape pattern is asserted by the book but nothing in the current tape can carry a factorization as a tape node.
- Cross-crate contracts (`ForwardMap`, `Differentiable`, `Solver`, `SdfField`) have no current home in the tree.
- These are symptoms of a broader "platform not soft-body-ready" state, not isolated bugs.

A patch-the-tape approach would leak into every other sim-soft dependency. Refactor the whole L0 platform for soft-body-readiness *first*, then build sim-soft on top. Aligned with "R34 GT-R overbuild chassis," "rocket platform — foundations before payload," and "own every line" principles.

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
