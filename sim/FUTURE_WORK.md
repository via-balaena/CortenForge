# Future Work

> Low-priority tasks that are out of scope for current development phases.
> These items are tracked here so they don't get lost, but are not blocking any features.

---

## Crate Consolidation

### Background

`sim-core` (`mujoco_pipeline.rs`) implements its own MuJoCo-aligned physics pipeline
end-to-end: forward kinematics, CRBA, RNE, collision detection, penalty-based joint
limits/equality constraints, and a custom `pgs_solve_contacts()` solver. Contact geometry
types (`ContactPoint`, `ContactForce`, `ContactManifold`) now live in `sim-core/src/contact.rs`
(consolidated from the deleted sim-contact crate in Phase 2). sim-core does **not** call
any solver or joint type from `sim-constraint`.

Current cross-crate usage (verified by grep):

| Type / API | Defined in | Actually called by | Notes |
|---|---|---|---|
| `PGSSolver`, `CGSolver`, `NewtonConstraintSolver`, `ConstraintSolver` | sim-constraint | **nobody** (only sim-core benchmarks) | Dead code in production. sim-core has its own PGS. |
| `Joint` trait, `RevoluteJoint`, `PrismaticJoint`, … | sim-constraint | **nobody** (only benchmarks + sim-physics re-export) | sim-core uses `MjJointType` enum instead. |
| `JointMotor`, `MotorMode`, `JointLimits`, `LimitState` | sim-constraint | sim-physics re-export + test | Instantiated in `test_joint_types_accessible` only. Never by pipeline. |
| `ConstraintIslands`, `SparseJacobian`, `JacobianBuilder` | sim-constraint | **nobody** | Unused infrastructure. |
| `ContactSolverConfig` | ~~sim-contact~~ | ~~sim-core (re-export only)~~ | **Deleted in Phase 1.** Was re-exported by sim-core → sim-bevy prelude. No actual callers. |
| `ContactPoint`, `ContactForce`, `ContactManifold` | ~~sim-contact~~ → **sim-core** | **sim-bevy** (`CachedContacts` for debug viz) | **Moved to sim-core in Phase 2.** Only real external consumer is sim-bevy. |
| `ContactModel`, `ContactParams`, `SurfaceMaterial` | ~~sim-contact~~ | ~~sim-physics re-export + test~~ | **Deleted in Phase 2.** Were instantiated in `test_contact_types_accessible` only. |
| `ContactSolver`, `BatchContactProcessor` | ~~sim-contact~~ | ~~nobody~~ | **Deleted in Phase 2.** Unused compliant solver + batch infrastructure. |
| `FrictionModel` trait, `FrictionCone`, etc. | ~~sim-contact~~ | ~~nobody~~ | **Deleted in Phase 2.** sim-core computes friction inline. |

Phantom Cargo.toml dependencies (declared but zero source-level imports):

| Crate | Declares dependency on | Why it exists | Can remove? |
|---|---|---|---|
| sim-mjcf | `sim-constraint` (with `muscle` feature) | Enables muscle feature on sim-constraint transitively; no types imported | Yes — sim-mjcf parses muscles with its own internal `MjcfActuatorType::Muscle`, not sim-constraint types |
| sim-urdf | `sim-constraint` | No types imported | Yes |
| sim-bevy | `sim-constraint` | No types imported | Yes |
| sim-conformance-tests | `sim-constraint`, ~~`sim-contact`~~ | No types imported in test source files | Yes (unless tests are added later). sim-contact removed in Phase 1. |

### Problem

Two crates (`sim-constraint`, `sim-contact`) carry substantial API surface that is
**entirely duplicated** by `sim-core`'s MuJoCo pipeline. They remain as dependencies
because (a) sim-physics re-exports their types as public API, (b) sim-bevy uses
`ContactPoint` for visualization, (c) benchmarks instantiate the standalone solvers,
and (d) four other crates declare phantom Cargo.toml dependencies with zero source-level
imports. This creates confusion about which types are canonical and inflates the
dependency graph.

### Phase 1 — Remove phantom and dead dependencies ✅

All items completed. Changes verified with `cargo check --workspace`,
`cargo build -p sim-core`, `cargo test -p sim-core`, `cargo bench -p sim-core --no-run`,
and `cargo test --workspace`.

**sim-core (dead dependencies):**

1. [x] Remove `pub use sim_contact::ContactSolverConfig;` from `sim-core/src/lib.rs`
2. [x] Remove feature forwards from sim-core `[features]`:
   - Removed `"sim-constraint/parallel"` from `parallel` feature
   - Removed `"sim-contact/serde"` and `"sim-constraint/serde"` from `serde` feature
3. [x] Move sim-constraint from `[dependencies]` to `[dev-dependencies]`
4. [x] Remove `sim-contact` from `sim-core/Cargo.toml` `[dependencies]` entirely
5. [x] Remove `pub use sim_core::ContactSolverConfig;` from sim-bevy prelude and
   delete the stale "Contact Solver Configuration" doc-comment section

**Phantom dependencies:**
- [x] Remove `sim-constraint` from `sim-mjcf/Cargo.toml`
- [x] Remove `sim-constraint` from `sim-urdf/Cargo.toml`
- [x] Remove `sim-constraint` from `sim-bevy/Cargo.toml`
- [x] Remove `sim-constraint` and `sim-contact` from `sim-conformance-tests/Cargo.toml`

### Phase 2 — Consolidate sim-contact into sim-core ✅

All items completed. All unused sim-contact types (ContactModel, ContactParams,
SurfaceMaterial, DomainRandomization, friction models, solver, batch processor) were
deleted. Only `ContactPoint`, `ContactForce`, and `ContactManifold` were preserved,
moved into `sim-core/src/contact.rs`. Changes verified with `cargo check --workspace`,
`cargo build --workspace`, and `cargo test --workspace`.

- [x] Move `ContactPoint`, `ContactForce`, and `ContactManifold` into
  `sim-core/src/contact.rs` — copied entire `contact.rs` (~1500 lines) including
  private helpers, collision methods, and ~450 lines of unit tests
- [x] Update sim-bevy to import `ContactPoint` from sim-core instead of sim-contact
- [x] Disposition of remaining sim-contact types: **all deleted** (unused by pipeline,
  duplicated by sim-core's MuJoCo-aligned solver)
- [x] Update sim-physics: removed `pub use sim_contact;`, updated prelude to source
  contact types from sim-core, deleted `test_contact_types_accessible` test
- [x] Remove `sim-contact` dependency from sim-physics and sim-bevy Cargo.tomls
- [x] Remove `"sim-contact/serde"` from sim-physics serde feature list
- [x] Update sim-physics package description
- [x] Remove `sim-contact` from workspace `Cargo.toml` (members and dependencies)
- [x] Update CI scripts: removed from WASM_CRATES and LAYER0_CRATES in both
  `local-quality-check.sh` and `quality-gate.yml`
- [x] Update documentation: README.md, VISION.md, sim/ARCHITECTURE.md,
  sim/L0/physics/README.md, sim/docs/MUJOCO_CONFORMANCE.md,
  sim/docs/SIM_BEVY_IMPLEMENTATION_PLAN.md, docs/MUJOCO_GAP_ANALYSIS.md
- [x] Delete the `sim-contact` crate directory

### Phase 3 ✅ — Reduce sim-constraint to types + CGSolver

**Decision:** Keep CGSolver (`cg.rs`) — it is planned for future integration into the
MuJoCo pipeline as an alternative solver for large-scale systems (see "CGSolver
Integration for Large-Scale Systems" in Physics Features below). CGSolver is fully
standalone — zero dependencies on any deleted module.

**Delete** (solvers, infrastructure, and supporting modules):

- [x] Delete `pgs.rs` — `PGSSolver`, `PGSSolverConfig`, `PGSSolverResult`, `PGSSolverStats`
- [x] Delete `newton.rs` — `NewtonConstraintSolver`, `NewtonSolverConfig`,
  `NewtonSolverResult`, `SolverStats`
- [x] Delete `sparse.rs` — `SparseJacobian`, `JacobianBuilder`, `SparseEffectiveMass`,
  `InvMassBlock`
- [x] Delete `islands.rs` — `ConstraintIslands`, `Island`, `IslandStatistics`
- [x] Delete `parallel.rs` — parallel island solving (only called from `newton.rs`)
- [x] Refactor `solver.rs`:
  - Extract `BodyState` (~120 lines, struct + 4 methods) and `JointForce` (~10 lines,
    struct) into `types.rs` — CGSolver imports these via `use crate::{BodyState, JointForce}`
  - Delete the rest of `solver.rs`: `ConstraintSolver` struct (~500 lines),
    `ConstraintSolverConfig` (~60 lines), `SolverResult` (~25 lines)
  - Delete the `solver.rs` file after extraction

**Keep** (types + CGSolver):

- [x] `cg.rs` — `CGSolver`, `CGSolverConfig`, `CGSolverResult`, `CGSolverStats`,
  `Preconditioner`, `PreconditionerData` (1,664 lines including Block Jacobi
  preconditioner and 11 tests). Imports from crate: `BodyState`, `ConstraintForce`,
  `Joint`, `JointForce`, `JointType` — all in kept modules.
- `joint.rs` — `Joint` trait, `RevoluteJoint`, `PrismaticJoint`, `FixedJoint`,
  `SphericalJoint`, `UniversalJoint`, `CylindricalJoint`, `PlanarJoint`,
  `FreeJoint`, `JointType` enum, `JointDof`
- `limits.rs` — `JointLimits`, `LimitState`, `LimitStiffness`
- `motor.rs` — `JointMotor`, `MotorMode`
- `types.rs` — `JointState`, `JointVelocity`, `ConstraintForce`, `BodyState`, `JointForce`
- `equality.rs` — all equality constraint types
- `actuator.rs` — all actuator types
- `muscle.rs` (optional) — `MuscleJoint`, `MuscleCommands`, `MuscleJointBuilder`

**Cargo.toml cleanup:**

- [x] Remove `nalgebra-sparse` (only used by `sparse.rs`; also removed from workspace deps)
- [x] Remove `hashbrown` optional dep (only used by `parallel` feature)
- [x] Remove `thiserror` (declared at Cargo.toml:16 but zero source-level imports —
  phantom dep even before this phase)
- [x] Remove `parallel` feature and `rayon` optional dep (Cargo.toml:18) — no
  parallel solvers remain (`parallel.rs` deleted)
- [x] Update description (line 3): "Joint types, motors, limits, and CGSolver for
  articulated body simulation"

**sim-core benchmark cleanup:**

- [x] Remove solver benchmarks from `collision_benchmarks.rs`:
  - Deleted solver benchmark section header and functions (lines 585–742)
  - Removed solver entries from `criterion_group!` macro
  - Removed `use sim_constraint::{...}` import block (lines 22–25)
  - Removed `sim-constraint` from `sim-core/Cargo.toml` `[dev-dependencies]`

**lib.rs module structure update:**

- [x] Remove `mod` declarations for deleted modules: `solver`, `pgs`, `newton`,
  `sparse`, `islands`
- [x] Remove `#[cfg(feature = "parallel")] pub mod parallel;`
- [x] Remove `pub use` re-exports for deleted types; update `BodyState`/`JointForce`
  to re-export from `types` module
- [x] Keep `pub use cg::{CGSolver, CGSolverConfig, CGSolverResult, CGSolverStats,
  Preconditioner};`
- [x] Rewrite crate-level documentation to mention only CGSolver as
  experimental/future-integration solver

**sim-physics updates:**

- [x] Remove `ConstraintSolver` and `ConstraintSolverConfig` from prelude re-exports.
  Added `BodyState` and `JointForce` re-exports.
- [x] Verified `test_joint_types_accessible` test still passes (187 tests pass)

**CI scripts:**

- [x] No `parallel` feature references found in CI scripts — no changes needed

**Documentation updates:**

- [x] `sim/ARCHITECTURE.md` — updated sim-constraint section and feature flags table
- [x] `sim/docs/MUJOCO_CONFORMANCE.md` — updated test mapping, divergence table, test dir
- [x] `sim/docs/SIM_BEVY_IMPLEMENTATION_PLAN.md` — no stale references found, no changes needed
- [x] `docs/MUJOCO_GAP_ANALYSIS.md` — annotated ~13 references to deleted files

**Name collisions (deferred to Phase 4):**

Three types share names between sim-types and sim-constraint with different definitions:
- `JointLimits` — sim-types (`types/src/joint.rs:111`) vs sim-constraint
  (`constraint/src/limits.rs:12`)
- `JointType` — sim-types (`types/src/joint.rs:45`) vs sim-constraint
  (`constraint/src/joint.rs:57`)
- `JointState` — sim-types (`types/src/joint.rs:238`) vs sim-constraint
  (`constraint/src/types.rs:14`)
- sim-physics prelude imports the sim-types versions. If merging (Phase 4),
  reconcile or rename the sim-constraint variants.

**Completed.** Deleted 6 files (pgs.rs, newton.rs, sparse.rs, islands.rs, parallel.rs,
solver.rs). Extracted `BodyState`/`JointForce` to types.rs. Removed `nalgebra-sparse`,
`thiserror`, `hashbrown`, `rayon` from sim-constraint deps. Removed solver benchmarks
from sim-core. Updated sim-physics prelude, ARCHITECTURE.md, MUJOCO_CONFORMANCE.md,
MUJOCO_GAP_ANALYSIS.md. Validated with `cargo check/build/test --workspace`,
`cargo test -p sim-constraint` (187 tests pass), `cargo bench -p sim-core --no-run`.
Changes verified with `cargo check --workspace` and `cargo test --workspace`.

### Phase 4 (optional) — Merge sim-constraint into sim-types

- [ ] Evaluate whether remaining sim-constraint types are pure data (move to sim-types)
  or carry significant behavior (keep separate). Key types to assess:
  - `JointLimits` (has `compute_force()` method — behavioral)
  - `JointMotor` (has `compute_force()` — behavioral)
  - `Joint` trait (defines `parent_anchor()`, `child_anchor()`, `axis()` etc. — trait
    with default impls, implementations in each joint struct carry geometric behavior)
  - `JointDof`, `JointState`, `JointVelocity`, `ConstraintForce` — likely pure data
- [ ] Resolve three name collisions (`JointLimits`, `JointType`, `JointState`)
  between sim-types and sim-constraint before merge (see Phase 3 for details)
- [ ] If pure data: merge into `sim-types/src/joint.rs`, `sim-types/src/motor.rs`, etc.
- [ ] If behavioral: keep sim-constraint as a thin crate
- [ ] If fully merged: remove sim-constraint from workspace `Cargo.toml`
  (`[workspace.members]` and `[workspace.dependencies]`), delete crate directory,
  update `README.md` (line 144) and `VISION.md`, update CI crate lists in
  `local-quality-check.sh` and `quality-gate.yml`
- [ ] Update sim-physics:
  - Remove `pub use sim_constraint;` (line 108), update prelude to source types from
    sim-types
  - Remove `sim-constraint` from `[dependencies]` (`Cargo.toml:14`)
  - Remove `"sim-constraint/serde"` from serde feature list (`Cargo.toml:33`)
  - Update package description (`Cargo.toml:3`) which lists sim-constraint
- Current state: After Phase 3, sim-constraint would contain only type definitions
  and trait implementations (no solvers).
- Target state: One fewer crate if types are pure data; no change if they carry logic
  that depends on nalgebra matrix operations.
- Risk: **Low.** Purely organizational. However, sim-types currently has minimal
  dependencies — adding nalgebra-heavy types would increase its compile footprint.
- Validation: `cargo build --workspace`, `cargo test --workspace`

### Dependency graph

Before Phase 1 (arrows = Cargo.toml `[dependencies]`, **bold** = has source-level imports):
```
sim-core    → sim-constraint          (benchmarks only, no lib imports)
            → sim-contact             (re-exports ContactSolverConfig, never read)
sim-physics → **sim-constraint**      (re-exports types via prelude)
            → **sim-contact**         (re-exports types via prelude)
            → sim-core
sim-bevy    → **sim-contact**         (imports ContactPoint for debug viz)
            → sim-constraint          (phantom — zero imports)
            → sim-core
sim-mjcf    → sim-constraint[muscle]  (phantom — zero imports)
sim-urdf    → sim-constraint          (phantom — zero imports)
sim-tests   → sim-constraint, sim-contact  (phantom — zero imports in .rs files)
```

After Phase 1 ✅:
```
sim-core    → (dev-dep only: sim-constraint for benchmarks)
sim-physics → **sim-constraint**      (re-exports types)
            → **sim-contact**         (re-exports types)
            → sim-core
sim-bevy    → **sim-contact**         (ContactPoint)
            → sim-core
```

Current (after Phase 2 ✅, sim-contact deleted):
```
sim-core    → (dev-dep only: sim-constraint for benchmarks)
sim-physics → **sim-constraint**      (re-exports types)
            → **sim-core**            (ContactPoint, ContactForce, ContactManifold)
sim-bevy    → **sim-core**            (ContactPoint moved here)
```

After Phase 3 ✅ (sim-constraint stripped to types + CGSolver):
```
sim-physics → sim-core                (contact types re-exported from here)
            → sim-constraint          (types + CGSolver, no PGS/Newton/sparse/islands)
sim-bevy    → sim-core                (ContactPoint)
```

Note: sim-core no longer depends on sim-constraint (dev-dep removed in Phase 3).

### Status

Phase 1 ✅ — Remove phantom and dead dependencies (commit ba1d729)
Phase 2 ✅ — Consolidate sim-contact into sim-core (commit 9ed88ea)
Phase 3 ✅ — Reduce sim-constraint to types + CGSolver (commit a5cef72)
Phase 4 — Optional, merge sim-constraint into sim-types (not started)

---

## Code Quality Issues

Identified during solref/solimp review. Items marked ✅ have been addressed.

### P0 — Correctness

| # | Issue | Status | Notes |
|---|-------|--------|-------|
| 1 | Hardcoded Baumgarte parameters — joint limits, equality constraints ignore solref/solimp | ✅ Fixed | `eq_solref` and `jnt_solref` now read via `solref_to_penalty()`. Hardcoded `10000/1000` retained as fallback when solref ≤ 0. `eq_solimp` consumed via `compute_impedance()` — position-dependent impedance scales penalty stiffness/damping in connect, weld, and joint equality constraints. Contact solver CFM also uses `compute_impedance()` with full solimp (previously only read `solimp[0]`). |
| 2 | Distance equality constraints missing | ✅ Fixed | `apply_distance_constraint` enforces `\|p1 - p2\| = target_distance` between geom centers using penalty+impedance (Baumgarte stabilization). Added `geom_name_to_id` mapping in model builder. Handles singularity when geoms coincide (+Z fallback direction). `eq_obj1id`/`eq_obj2id` store geom IDs; body IDs derived via `model.geom_body[]`. Auto-computes initial distance when `distance` attribute omitted. 7 integration tests cover normal operation, worldbody geom, geom2-omitted (world origin sentinel), auto-distance, zero target, error handling, and inactive constraint. |

### P1 — Performance / Quality

| # | Issue | Status | Notes |
|---|-------|--------|-------|
| 3 | Contact clone in hot path | ✅ Fixed | Borrow split via `std::mem::take` on `efc_lambda`; `pgs_solve_contacts` now takes `&Data` + `&mut HashMap`. See details below. |
| 4 | Cholesky clone for implicit integrator | ✅ Fixed | `std::mem::replace` swaps populated matrix out for Cholesky consumption; zero matrix left for next step. See details below. |
| 5 | Duplicate contact structures | ✅ Improved | Old `ContactPoint` in mujoco_pipeline.rs removed. Two distinct types remain: `Contact` (mujoco pipeline constraint struct) and `ContactPoint` (sim-core geometric contact, moved from sim-contact in Phase 2). These serve different purposes. |

#### Issue 3 — Contact clone in hot path

**Location:** `mj_fwd_constraint()` at `mujoco_pipeline.rs:8045`

```rust
let contacts: Vec<Contact> = data.contacts.clone();
```

**Cost:** O(n_contacts) allocation per step. `Contact` is ~216 bytes; with pre-allocated
capacity of 256 contacts this clones up to ~55 KB per simulation step.

**Root cause:** Borrow checker conflict — `data.contacts` cannot be borrowed while `&mut Data`
is passed to `pgs_solve_contacts()` (line 6539). The `&mut` prevents holding any shared
reference into `data` across the call.

**Key insight:** `pgs_solve_contacts` mostly reads from `data` (`qM_cholesky`,
`qfrc_applied`, `qfrc_actuator`, `qfrc_passive`, `qfrc_bias`) but **does** write to
`data.efc_lambda` (warmstart cache: clear + insert). The `&mut Data` cannot simply become
`&Data` without addressing `efc_lambda`.

**Fix (applied):** Split the borrow by extracting `efc_lambda` from `data` via
`std::mem::take` before the contact block. Pass it as a separate `&mut HashMap` parameter
to `pgs_solve_contacts` (signature changed to `data: &Data` + `efc_lambda: &mut HashMap`).
Restore `data.efc_lambda` after the call. This eliminates the borrow conflict —
`data.contacts` is borrowed as `&[Contact]` during the PGS call without cloning.

#### Issue 4 — Cholesky clone for implicit integrator

**Location:** `mj_fwd_acceleration_implicit()` at `mujoco_pipeline.rs:8351–8355`

```rust
let chol = data
    .scratch_m_impl
    .clone()
    .cholesky()
    .ok_or(StepError::CholeskyFailed)?;
```

**Cost:** O(nv²) allocation per step. For nv=30 DOFs this copies 900 f64s (~7.2 KB).
Larger models (nv=100+) copy ~80 KB per step.

**Root cause:** nalgebra's `.cholesky()` consumes the matrix (takes ownership). Since
`scratch_m_impl` lives inside `Data`, it cannot be moved out directly, so the code clones
it to produce an owned value for the API.

**Key insight:** `scratch_m_impl` is **never read after** the Cholesky call within this
function. It is a scratch buffer that gets completely overwritten via `copy_from(&data.qM)`
at the start of every step. The clone preserves data that is immediately discarded.

**Fix (applied):** Use `std::mem::replace` to swap the populated matrix out with a
same-sized zero matrix, then factorize the owned value. This still allocates O(nv²) for
the replacement zero matrix, but avoids copying the populated values — a cheaper zero-fill
instead of element-by-element clone. The zero matrix left behind is correctly sized for the
next step's `copy_from(&data.qM)` call (nalgebra's `copy_from` requires matching
dimensions).

**True zero-allocation alternative (not yet implemented):** Implement an in-place Cholesky
factorization that operates on `&mut DMatrix<f64>` without consuming it. This could wrap
LAPACK's `dpotrf` (which factorizes in place) or use a manual implementation. The factored
lower triangle would overwrite `scratch_m_impl` directly, and a thin wrapper would provide
the `solve` interface. This is more invasive but achieves O(0) allocation.

**Future optimization (out of scope):** For serial kinematic chains the mass matrix is
banded with O(1) bandwidth. A sparse L^T D L factorization (using the existing `qLD_diag` /
`qLD_L` fields already defined in `Data`) would reduce factorization from O(nv³) to O(nv).
Featherstone's ABA algorithm is another O(nv) alternative. These are tracked under
Physics Features.

### P2 — Minor

| # | Issue | Status | Notes |
|---|-------|--------|-------|
| 6 | Block Jacobi preconditioner placeholder in `cg.rs` | ✅ Fixed | See details below. |
| 7 | Various hardcoded numerical thresholds | ✅ Fixed | See details below. |

#### Issue 6 — Block Jacobi preconditioner placeholder

**Location:** `build_preconditioner()` at `sim/L0/constraint/src/cg.rs`

**Impact:** Users selecting `Preconditioner::BlockJacobi` (used by the `high_accuracy()`
and `large_system()` config presets) silently got scalar Jacobi instead. No warning was
emitted.

**Key finding: CGSolver is currently dead code.** It is exported from `sim-constraint`
(`lib.rs:122`) but has zero callers outside the crate. The active simulation pipeline uses
`pgs_solve_contacts` in `mujoco_pipeline.rs`, not the CGSolver.

**Fix (applied):** Implemented proper block Jacobi preconditioner:

1. Added `PreconditionerData` enum with `Diagonal` and `Block` variants — diagonal stores
   a `DVector<f64>` for element-wise multiply (None/Jacobi); Block stores inverted dense
   sub-blocks with offsets and sizes for block-wise matrix-vector solve.
2. `build_preconditioner()` now accepts `constraint_blocks: &[(usize, usize)]` describing
   `(row_offset, num_rows)` pairs. For BlockJacobi, it extracts each diagonal block
   `A[off..off+sz, off..off+sz]` and inverts via Cholesky (with scalar Jacobi fallback
   if Cholesky fails for a block). Falls back to scalar Jacobi when no block info provided.
3. `apply_preconditioner()` dispatches on the enum: Diagonal does `r.component_mul(diag)`,
   Block iterates over blocks computing `z[rows] = block_inv * r[rows]`.
4. `solve()` extracts block boundaries from `SolverConstraint::row_offset` and `num_rows`,
   passes them through `cg_solve()` to `build_preconditioner()`.
5. Two tests added: `test_block_jacobi_preconditioner` (correctness on 6×6 block-diagonal
   SPD system) and `test_block_jacobi_fewer_iterations_than_scalar` (verifies Block Jacobi
   converges in ≤ iterations vs scalar Jacobi on systems with strong intra-block coupling).

#### Issue 7 — Hardcoded numerical thresholds

Approximately 25+ hardcoded numerical thresholds existed in `mujoco_pipeline.rs`. They fall
into three categories:

**Category A — Numerical safety guards (NOT configurable, documented in-code):**

~12 values like `1e-10`, `1e-12`, `1e-15` used to prevent division by zero and detect
degenerate geometry. These are correct and remain fixed constants. Examples:
- `GEOM_EPSILON = 1e-10` — collision detection zero-guard
- `1e-10` in `build_tangent_basis()` — normalization check
- `1e-12` in PGS diagonal inversion — matrix conditioning
- `0.001` minimum solref timeconst clamp — prevents division by zero in Baumgarte
  stabilization; now documented with safety rationale
- `FRICTION_VELOCITY_THRESHOLD = 1e-12` — effectively zero; now documented as safety guard
- `.max(10)` PGS iteration floor — documented as safety clamp preventing under-resolution
- `.max(1e-8)` solver tolerance floor — documented as safety clamp preventing infinite loops

**Category B — Global solver parameters (already configurable):**

Already exposed in the `Model` struct:
- `solver_tolerance` (default `1e-8`) — PGS convergence criterion
- `solver_iterations` (default `100`) — max PGS iterations
- `impratio` (default `1.0`) — constraint impedance ratio
- `geom_solref`, `geom_solimp`, `geom_margin`, `geom_gap` — per-geometry contact params

**Category C — Fix (applied):**

6 new configurable fields added to `Model`, `ModelBuilder`, `MjcfOption`, and the MJCF
parser. Inline literals replaced with model field reads. All defaults match previous
hardcoded values, preserving backward compatibility.

| Field | Default | What it controls |
|-------|---------|------------------|
| `Model::regularization` | `1e-6` | PGS constraint softness. CFM now scales as `regularization + (1-d) * regularization * 100` (previously independent `1e-4` literal). |
| `Model::default_eq_stiffness` | `10000.0` | Fallback stiffness for equality constraints and joint limits without solref. |
| `Model::default_eq_damping` | `1000.0` | Fallback damping for equality constraints and joint limits without solref. |
| `Model::max_constraint_vel` | `1.0` | Maximum linear velocity change per timestep from constraint forces (m/s). |
| `Model::max_constraint_angvel` | `1.0` | Maximum angular velocity change per timestep from constraint forces (rad/s). |
| `Model::friction_smoothing` | `1000.0` | Sharpness of tanh friction transition. Replaced `FRICTION_SMOOTHING` const. |

Parseable from MJCF `<option>` via attributes: `regularization`, `default_eq_stiffness`,
`default_eq_damping`, `max_constraint_vel`, `max_constraint_angvel`, `friction_smoothing`.

---

## Physics Features

### Deformable Bodies

- [ ] Implement soft body simulation (FEM or position-based dynamics)
- [ ] Implement cloth simulation
- Current state: `sim-deformable` crate exists as stub only
- Use cases: Soft tissue simulation, cloth/fabric, pneumatic actuators

### Implicit Integrators

- [ ] Add RK4 (Runge-Kutta 4th order) integrator
- [ ] Add implicit Euler for stiff systems
- Current state: Semi-implicit Euler only
- Rationale: Higher-order integrators improve accuracy for stiff systems (high damping, high stiffness springs)

### CGSolver Integration for Large-Scale Systems

- [ ] Integrate CGSolver as alternative constraint solver in the MuJoCo pipeline
- Current state: CGSolver is fully implemented in `sim-constraint` (with Block Jacobi preconditioner) but has zero callers in the active pipeline. PGS is the only solver used in `mujoco_pipeline.rs`.
- Trigger: When the system needs to handle large constraint counts (100+ contacts) where PGS convergence becomes slow
- Rationale: CG with Block Jacobi preconditioning scales better than PGS for large systems — PGS convergence degrades with constraint count while preconditioned CG maintains stable iteration counts
- Prerequisites: Adapter layer mapping `mujoco_pipeline.rs` contact structures to CGSolver's `Joint` trait interface
- **Crate Consolidation Phase 3 complete:** CGSolver was preserved during Phase 3
  (commit a5cef72). It remains in `sim-constraint/src/cg.rs` ready for integration.

### Batched Simulation

- [ ] CPU-parallel multi-environment architecture for RL training
- [ ] Thread-per-env and/or SIMD-across-envs execution strategies
- [ ] Shared-nothing memory layout to avoid synchronization overhead
- [ ] Batched API surface (step N environments, return N observation tensors)
- Current state: Single-environment execution only
- Rationale: Prerequisite to GPU acceleration — validates the batching API, memory layout, and training loop integration on CPU before porting hot paths to GPU. Thousands of parallel humanoid envs are needed for policy learning; the batching architecture is independent of the compute backend.

### GPU Acceleration

- [ ] CUDA/Metal/WebGPU backend for parallel simulation
- [ ] Batch simulation for RL training (thousands of envs)
- Current state: CPU-only
- Use cases: Large-scale RL training, real-time multi-agent simulation
