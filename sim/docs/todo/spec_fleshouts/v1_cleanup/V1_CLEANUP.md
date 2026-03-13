# v1.0 Cleanup — Comprehensive Post-Ship Housekeeping

> **Status**: Draft — awaiting approval
> **Scope**: Every cleanup item found across all sim/ crates, docs, CI, examples, and workspace config
> **Principle**: A-grade or it doesn't ship. Now that v1.0 shipped, clean the house.

---

## Execution Model

Run **one batch per session**. Each session:

1. Read this spec — find the batch
2. Execute all tasks in the batch
3. Run the batch checkpoint command
4. Commit once (with permission)

**Starting prompt** (paste at the start of each new session):

```
Execute batch B<N> from the v1.0 cleanup spec at
sim/docs/todo/spec_fleshouts/v1_cleanup/V1_CLEANUP.md

Steps:
1. Read the spec — find the batch and its tasks
2. Read the relevant source files before making any changes
3. Implement each task in the batch exactly as specified
4. Run the batch checkpoint command
5. Ask before committing
```

Replace `<N>` with the batch number (e.g., `B1`, `B5`, `B9`).

---

## Progress

### B1 — Dead code removal (T01–T07)
**Checkpoint**: `cargo test -p sim-core -p sim-mjcf`
- [x] **T01** — Delete `project_elliptic_cone()` from noslip solver
- [x] **T02** — Delete `MJ_MINVAL` constant from linalg
- [x] **T03** — Delete `model_from_mjcf_with_plugins()` from mjcf builder
- [x] **T04** — Delete `ensure_bvh()` from mesh
- [x] **T05** — Delete `dact_dactdot` field from derivatives
- [x] **T06** — Delete `get_min_translational_mass()` and `get_min_rotational_inertia()` from equality constraints
- [x] **T07** — Delete plugin builder scaffolding (`resolve_and_assign_plugins()`, `assign_body_plugins()`, `with_nstate()`)

### B2 — Anti-patterns + visibility tightening (T08–T11)
**Checkpoint**: `cargo test -p sim-core -p sim-mjcf -p sim-conformance-tests`
- [x] **T08** — Replace `unwrap_or_else(|| unreachable!())` with `.expect()` in composite.rs and newton.rs
- [x] **T09** — Fix CG solver forced-read workaround
- [x] **T10** — Tighten Phase 8b leftovers in forward/position.rs
- [x] **T11** — Tighten `cost` field visibility in primal solver

### B3 — `#[allow]` audit (T12–T15)
**Checkpoint**: `cargo test -p sim-core -p sim-mjcf -p sim-gpu -p sim-conformance-tests` + `cargo clippy -p sim-core -p sim-mjcf -- -D warnings`
- [x] **T12** — Audit all `#[allow(dead_code)]` / `#[allow(unused_*)]` in sim-core (~20 annotations)
- [x] **T13** — Audit all `#[allow(dead_code)]` / `#[allow(unused_*)]` in sim-mjcf (~5 annotations)
- [x] **T14** — Audit all `#[allow(dead_code)]` in sim-gpu (2 annotations)
- [x] **T15** — Audit conformance test helpers in common.rs (23 annotations)

### B4 — Housekeeping: TODOs, workspace, feature flag (T16, T37–T40)
**Checkpoint**: `cargo build` + `cargo test -p sim-mjcf --features mjb`
- [x] **T16** — Resolve or convert all TODO comments to tracked deferred tasks
- [x] **T37** — Remove stale `lumen-geometry` path dependency from root Cargo.toml
- [x] **T38** — Remove stale "vision" domain from xtask grade.rs search locations
- [x] **T39** — Extract magic numbers in sim-mjcf (BENDING_COEFFS, MIN_VERTEX_MASS)
- [x] **T40** — Review `mjb` feature in sim-mjcf (add integration test or document)

### B5 — sim-mjcf function refactoring (T17–T23)
**Checkpoint**: `cargo test -p sim-mjcf -p sim-conformance-tests`
- [x] **T17** — Refactor `ModelBuilder::build()` (884 lines → extracted helpers)
- [x] **T18** — Refactor `process_actuator()` (545 lines → per-type handlers)
- [x] **T19** — Refactor `process_flex_bodies()` (454 lines → split mesh/body/integration)
- [x] **T20** — Refactor `ModelBuilder::new()` (309 lines → extract size computation)
- [x] **T21** — Refactor `validate_tendons()` (286 lines → per-constraint helpers)
- [x] **T22** — Refactor `process_body_with_world_frame()` (221 lines → split frame/body)
- [x] **T23** — Refactor `process_geom()` (208 lines → extract inertia logic)

### B6 — sim-core file splits, part 1 (T24–T27)
**Checkpoint**: `cargo test -p sim-core -p sim-conformance-tests`
- [x] **T24** — Split `derivatives.rs` (5,998 lines → fd_perturbations, hybrid_path, integration_derivs)
- [x] **T25** — Split `sdf.rs` (2,896 lines → primitive_sdf, operations, interpolation)
- [x] **T26** — Split `collision/flex_collide.rs` (2,753 lines → narrowphase_flex, flex_self_collision)
- [x] **T27** — Split `forward/muscle.rs` (2,497 lines → hill_dynamics, fiber_length)

### B7 — sim-core file splits, part 2 (T28–T31)
**Checkpoint**: `cargo test -p sim-core -p sim-conformance-tests`
- [x] **T28** — Split `constraint/assembly.rs` (1,903 lines → contact_assembly, equality_assembly)
- [x] **T29** — Split `collision/mod.rs` (1,539 lines → collision_pairs)
- [x] **T30** — Split `jacobian.rs` (1,532 lines → per-constraint-type builders)
- [x] **T31** — Split `sensor/mod.rs` (1,430 lines → per-sensor-type submodules)

### B8 — Documentation refresh (T32–T36)
**Checkpoint**: Visual review
- [x] **T32** — Update MUJOCO_CONFORMANCE.md (stale status markers, CI section)
- [x] **T33** — Update ARCHITECTURE.md (module counts, accuracy review)
- [x] **T34** — Clean stale references in phase spec docs (deleted crate names in test commands)
- [x] **T35** — Update example crate help text (references to deleted crates)
- [x] **T36** — Consolidate future_work_10b–10j.md into themed post-v1.0 roadmap

### B9 — Final verification (T41)
**Checkpoint**: Full suite (see T41 details)
- [ ] **T41** — Full verification pass (build, test, clippy, fmt, test count regression check)

---

## Task Details

---

### T01 — Delete `project_elliptic_cone()`

**File**: `sim/L0/core/src/constraint/solver/noslip.rs` (line ~25)
**Reason**: Marked "retained for potential future use" but PGS now uses two-phase
ray+QCQP projection. No call sites. No planned consumer.
**Action**: Delete function and its `#[allow(dead_code)]`.
**Checkpoint**: `cargo test -p sim-core`

---

### T02 — Delete duplicate `MJ_MINVAL` constant from linalg.rs

**File**: `sim/L0/core/src/linalg.rs` (line ~63)
**Reason**: Marked `#[allow(dead_code)]`. This is a private duplicate — the
authoritative `MJ_MINVAL` lives in `constraint/impedance.rs` (line ~35) and is
widely used across the crate. The `linalg.rs` copy has zero consumers.
**Action**: Delete the duplicate constant and its allow annotation. Do NOT touch
the `impedance.rs` definition.
**Checkpoint**: `cargo test -p sim-core`

---

### T03 — Delete `model_from_mjcf_with_plugins()`

**File**: `sim/L0/mjcf/src/builder/mod.rs` (line ~358)
**Reason**: Marked `#[allow(dead_code)]`. Phase 13 plugin scaffolding that was
never wired up. If plugin loading is implemented later, it will be written fresh
against the actual plugin API — not resurrected from dead scaffolding.
**Action**: Delete function and its allow annotation.
**Checkpoint**: `cargo test -p sim-mjcf`

---

### T04 — Delete `ensure_bvh()`

**File**: `sim/L0/core/src/mesh.rs` (line ~239)
**Reason**: Marked "Kept for future deserialization support." No call sites.
No deserialization pipeline exists.
**Action**: Delete method and its allow annotation.
**Checkpoint**: `cargo test -p sim-core`

---

### T05 — Delete `dact_dactdot` field

**File**: `sim/L0/core/src/derivatives.rs` (line ~2371)
**Reason**: Marked "Currently unused by hybrid path." Field is populated but
never read by any consumer.
**Action**: Delete the field from the struct, delete its computation wherever it
is populated. If a future dynamics path needs it, re-derive from the algorithm
spec rather than carrying dead state.
**Checkpoint**: `cargo test -p sim-core -p sim-conformance-tests`

---

### T06 — Delete debug-only mass/inertia helpers

**File**: `sim/L0/core/src/constraint/equality.rs` (lines ~109, ~127)
**Items**:
- `pub fn get_min_translational_mass()` — marked `#[allow(dead_code)]`, comment:
  "Kept for debugging/testing; hot path uses cached data"
- `pub fn get_min_rotational_inertia()` — same

**Reason**: Hot path uses `data.body_min_mass[]` / `data.body_min_inertia[]`.
These functions have zero call sites. Debugging helpers should live in test code,
not production code.
**Action**: Delete both functions and their allow annotations. If needed for
future debugging, re-add in a `#[cfg(test)]` block.
**Checkpoint**: `cargo test -p sim-core`

---

### T07 — Delete plugin builder scaffolding

**File**: `sim/L0/mjcf/src/builder/plugin.rs`
**Items**:
- `resolve_and_assign_plugins()` (line ~19) — `#[allow(dead_code)]`
- `assign_body_plugins()` (line ~138) — `#[allow(dead_code)]`
- `with_nstate()` (line ~192) — `#[allow(dead_code)]`, test-only builder method

**Reason**: Plugin system DT-170–174 is post-v1.0. When implemented, plugin
resolution will be written against the final plugin API. Dead scaffolding just
creates confusion about what's real.
**Action**: Delete all three items. If the file becomes empty, delete the file
and remove its `mod plugin;` declaration.
**Checkpoint**: `cargo test -p sim-mjcf`

---

### T08 — Replace `unreachable!()` anti-patterns with `.expect()`

**Files and locations**:

1. `sim/L0/mjcf/src/builder/composite.rs` — 3 instances in `validate_cable()`:
   - Line ~435: `unwrap_or_else(|| unreachable!())` on cable body chain lookup
   - Line ~450: same pattern on recursive body append
   - Line ~459: same pattern on cable geom extraction

2. `sim/L0/core/src/constraint/solver/newton.rs` — 2 instances:
   - Line ~240: sparse/dense path selection after conditional init
   - Line ~254: same pattern

**Reason**: `unreachable!()` produces a generic panic message with no context.
`.expect("descriptive message")` gives the same crash behavior but with a message
that explains what precondition was violated — invaluable for debugging.

**Action**: Replace each `unwrap_or_else(|| unreachable!())` with
`.expect("precondition: <what was expected>")`. Write messages that describe
the invariant, e.g., `.expect("cable body chain is non-empty after validate_cable()")`.

**Checkpoint**: `cargo test -p sim-core -p sim-mjcf`

---

### T09 — Fix CG solver forced-read workaround

**File**: `sim/L0/core/src/constraint/solver/cg.rs` (lines ~146–147)
**Current**:
```rust
// Force-read to silence unused_assignments warning.
let _ = (&grad_old, &mgrad_old);
```

**Context**: Polak-Ribiere CG algorithm initializes `grad_old` and `mgrad_old`
before the loop, but they aren't read until iteration ≥ 1. The compiler warns
about unused assignments on the first write.

**Action**: Restructure the loop so `grad_old` / `mgrad_old` are initialized
at the end of the loop body (as "previous iteration" state), eliminating the
need for the pre-loop initialization. If restructuring hurts algorithm clarity,
keep the workaround but replace the comment with a proper explanation of the
Polak-Ribiere two-iteration pattern.

**Checkpoint**: `cargo test -p sim-core -p sim-conformance-tests` (conformance
tests catch any numerical drift)

---

### T10 — Tighten Phase 8b leftovers

**File**: `sim/L0/core/src/forward/position.rs`
**Items** (all marked `#[allow(dead_code)]` with "consumers still import from
monolith; live after Phase 8b"):
- `pub fn aabb_from_geom()` (line ~228)
- `pub struct SweepAndPrune` (line ~419)
- `impl SweepAndPrune` (line ~427)
- `pub fn closest_point_segment()` (line ~517)
- `pub fn closest_points_segments()` (line ~538)

**Action**: Grep entire workspace for each item name.
- If no external consumer exists → change `pub` to `pub(crate)` and remove the
  `#[allow(dead_code)]` (crate-internal use is fine).
- If truly dead even within the crate → delete.
- If external consumers exist → remove the allow annotation and the stale comment.

**Checkpoint**: `cargo test -p sim-core`

---

### T11 — Tighten `cost` field visibility

**File**: `sim/L0/core/src/constraint/solver/primal.rs` (line ~140)
**Current**: `cost: f64` field marked `#[allow(dead_code)]` — "for diagnostics/
future cost monitoring."

**Action**: If the field is never read outside the struct (grep to verify):
- If only written, never read → delete the field and its computation entirely.
- If read only in debug builds → gate with `#[cfg(debug_assertions)]`.
- If read internally → remove the allow annotation.

**Checkpoint**: `cargo test -p sim-core`

---

### T12 — Audit `#[allow]` annotations in sim-core

**Scope**: All `#[allow(dead_code)]`, `#[allow(unused_imports)]`,
`#[allow(unused_variables)]`, `#[allow(unused_mut)]`, `#[allow(unused_assignments)]`
annotations in `sim/L0/core/src/`.

**Known locations** (~20 annotations after T01–T11 removals):
- `forward/mod.rs` — 8 `#[allow(unused_imports)]` on interior re-exports
- `forward/position.rs` — handled in T10
- `constraint/equality.rs` — handled in T06
- `constraint/solver/primal.rs` — handled in T11
- `island/mod.rs` — `#![allow(clippy::cast_possible_truncation, clippy::needless_range_loop)]`
  (MuJoCo engine_island.c convention — likely keep)

**Action**: For each annotation:
1. Check if the item is still dead (it may have gained consumers since annotated)
2. Item has consumers → remove annotation
3. Item is dead with documented future plan → keep, ensure comment cites the plan
4. Item is dead with no plan → delete the item

**Checkpoint**: `cargo test -p sim-core` + `cargo clippy -p sim-core -- -D warnings`

---

### T13 — Audit `#[allow]` annotations in sim-mjcf

**Scope**: All allow annotations in `sim/L0/mjcf/src/`.

**Known locations** (~5 after T03/T07 removals):
- `builder/asset.rs` (line ~13): `AssetKind::Texture` — `#[allow(dead_code)]`,
  comment: "Texture used when texture loading lands"
- Various clippy suppressions on parser functions (cast truncation, excessive bools,
  too-many-lines for Gauss-Kronrod constants, etc.)

**Action**: Same protocol as T12. Clippy suppressions with justification comments
are fine to keep. Dead code with no consumer and no near-term plan should go.

**Checkpoint**: `cargo test -p sim-mjcf` + `cargo clippy -p sim-mjcf -- -D warnings`

---

### T14 — Audit `#[allow]` annotations in sim-gpu

**File**: `sim/L0/gpu/src/pipeline.rs`
**Items**:
- Line ~48: `#[allow(dead_code)]` on `integrate_bind_group_layout: wgpu::BindGroupLayout`
- Line ~53: `#[allow(dead_code)]` on `params_buffer: wgpu::Buffer`

**Action**: Verify if these fields are needed for wgpu resource lifetime semantics
(GPU layouts/buffers must outlive their bind groups). If so, add a comment
explaining the ownership requirement and keep. If not → delete fields.

**Checkpoint**: `cargo test -p sim-gpu`

---

### T15 — Audit conformance test helpers

**File**: `sim/L0/tests/mujoco_conformance/common.rs`
**Scope**: 23 items marked `#[allow(dead_code)]` — tolerance constants
(`TOL_FK`, `TOL_CRBA`, `TOL_RNE`, etc.) and parse helpers (`parse_npy()`,
`parse_npy_i32()`, `load_reference_f64()`, `assert_array_eq()`,
`assert_quat_eq()`, etc.).

Conformance suite is 79/79 passing. Some of these helpers may have been for
test infrastructure that was later refactored.

**Action**:
1. Grep all files in `integration/` and `mujoco_conformance/` for each constant
   and function name
2. Remove genuinely unused items
3. Remove `#[allow(dead_code)]` from items that are actually used
4. Any helper used by only one test → consider inlining

**Checkpoint**: `cargo test -p sim-conformance-tests`

---

### T16 — Resolve or track all TODO/FIXME comments

**Known TODO comments** (4 total):

1. `sim/L0/core/src/collision/mesh_collide.rs:32`:
   `margin: f64, // TODO: thread margin into mesh helpers`

2. `sim/L0/core/src/collision/sdf_collide.rs:26`:
   `margin: f64, // TODO: thread margin into SDF helpers`

3. `sim/L0/urdf/src/converter.rs:355`:
   `// TODO: Add mesh asset support`

4. `sim/L1/bevy/src/mesh.rs:43`:
   `// TODO: Implement height field visualization`

**Action**: For each TODO:
- Assign a DT number (continuing from DT-174)
- Add to the post-v1.0 deferred task tracker
- Replace the code comment with `// DT-NNN: <brief description>`
  so the comment points to the tracking system rather than floating in code

**Checkpoint**: Visual review only.

---

### T17 — Refactor `ModelBuilder::build()` (884 lines)

**File**: `sim/L0/mjcf/src/builder/build.rs`
**Current**: Single 884-line function that orchestrates the entire model build.
**Problem**: Extremely difficult to navigate, review, or test individual stages.

**Action**: Extract 10–15 focused helper methods, one per logical stage of the
build (e.g., `build_kinematic_tree()`, `build_collision_geometry()`,
`build_constraints()`, `build_actuators()`, `build_sensors()`, etc.).
The top-level `build()` becomes a ~50-line orchestrator that calls helpers in order.

Each helper should be a `&self` or `&mut self` method on `ModelBuilder`.
No change to public API — `build()` signature stays the same.

**Checkpoint**: `cargo test -p sim-mjcf -p sim-conformance-tests`

---

### T18 — Refactor `process_actuator()` (545 lines)

**File**: `sim/L0/mjcf/src/builder/actuator.rs`
**Action**: Extract per-actuator-type handler functions. The main function becomes
a match dispatcher. Each handler is a standalone function that processes one
actuator type.

**Checkpoint**: `cargo test -p sim-mjcf -p sim-conformance-tests`

---

### T19 — Refactor `process_flex_bodies()` (454 lines)

**File**: `sim/L0/mjcf/src/builder/flex.rs`
**Action**: Split into mesh processing, body generation, and integration setup.
Extract `const BENDING_COEFFS: usize = 17` (currently a magic number).

**Checkpoint**: `cargo test -p sim-mjcf -p sim-conformance-tests`

---

### T20 — Refactor `ModelBuilder::new()` (309 lines)

**File**: `sim/L0/mjcf/src/builder/init.rs`
**Action**: Extract size computation into a separate `compute_model_sizes()` helper.
The `new()` method calls the helper and uses results for allocation.

**Checkpoint**: `cargo test -p sim-mjcf`

---

### T21 — Refactor `validate_tendons()` (286 lines)

**File**: `sim/L0/mjcf/src/validation.rs`
**Action**: Extract per-constraint-type validation helpers.

**Checkpoint**: `cargo test -p sim-mjcf`

---

### T22 — Refactor `process_body_with_world_frame()` (221 lines)

**File**: `sim/L0/mjcf/src/builder/body.rs`
**Action**: Split frame computation from body construction.

**Checkpoint**: `cargo test -p sim-mjcf -p sim-conformance-tests`

---

### T23 — Refactor `process_geom()` (208 lines)

**File**: `sim/L0/mjcf/src/builder/geom.rs`
**Action**: Extract inertia computation logic into a helper.

**Checkpoint**: `cargo test -p sim-mjcf -p sim-conformance-tests`

---

### T24 — Split `derivatives.rs` (5,998 lines)

**File**: `sim/L0/core/src/derivatives.rs`
**Action**: Extract into a `derivatives/` directory module:
- `derivatives/mod.rs` — public API, struct definitions, orchestration
- `derivatives/fd.rs` — finite-difference perturbation methods
- `derivatives/hybrid.rs` — hybrid analytical/FD path
- `derivatives/integration.rs` — integration-specific derivative logic

Maintain all existing tests. Move each test to the submodule it tests.

**Checkpoint**: `cargo test -p sim-core -p sim-conformance-tests`

---

### T25 — Split `sdf.rs` (2,896 lines)

**File**: `sim/L0/core/src/sdf.rs`
**Action**: Extract into `sdf/` directory:
- `sdf/mod.rs` — public types, API
- `sdf/primitives.rs` — sphere, box, capsule, ellipsoid distance queries
- `sdf/operations.rs` — union, intersection, transform
- `sdf/interpolation.rs` — trilinear/tricubic grid interpolation

**Checkpoint**: `cargo test -p sim-core`

---

### T26 — Split `collision/flex_collide.rs` (2,753 lines)

**File**: `sim/L0/core/src/collision/flex_collide.rs`
**Action**: Extract:
- `collision/flex_narrow.rs` — sphere-geom narrowphase tests
- `collision/flex_self.rs` — self-collision modes (narrow, BVH, SAP)

Keep `flex_collide.rs` as the dispatcher/entry point.

**Checkpoint**: `cargo test -p sim-core -p sim-conformance-tests`

---

### T27 — Split `forward/muscle.rs` (2,497 lines)

**File**: `sim/L0/core/src/forward/muscle.rs`
**Action**: Extract:
- `forward/hill.rs` — Hill-type muscle dynamics, force-velocity curves
- `forward/fiber.rs` — fiber length computation, gain functions

Keep `muscle.rs` as the entry point.

**Checkpoint**: `cargo test -p sim-core -p sim-conformance-tests`

---

### T28 — Split `constraint/assembly.rs` (1,903 lines)

**File**: `sim/L0/core/src/constraint/assembly.rs`
**Action**: Extract:
- `constraint/contact_assembly.rs` — contact constraint row assembly
- `constraint/equality_assembly.rs` — equality constraint row assembly

Keep `assembly.rs` as the orchestrator.

**Checkpoint**: `cargo test -p sim-core -p sim-conformance-tests`

---

### T29 — Split `collision/mod.rs` (1,539 lines)

**File**: `sim/L0/core/src/collision/mod.rs`
**Action**: Extract collision pair enumeration logic into
`collision/pairs.rs`. Keep `mod.rs` for module declarations and top-level dispatch.

**Checkpoint**: `cargo test -p sim-core`

---

### T30 — Split `jacobian.rs` (1,532 lines)

**File**: `sim/L0/core/src/jacobian.rs`
**Action**: Extract per-constraint-type Jacobian builders into `jacobian/` directory:
- `jacobian/mod.rs` — shared types, dispatch
- `jacobian/contact.rs` — contact Jacobian rows
- `jacobian/equality.rs` — equality constraint Jacobian rows
- `jacobian/tendon.rs` — tendon Jacobian rows (if applicable)

**Checkpoint**: `cargo test -p sim-core -p sim-conformance-tests`

---

### T31 — Split `sensor/mod.rs` (1,430 lines)

**File**: `sim/L0/core/src/sensor/mod.rs`
**Action**: Extract per-sensor-type dispatch into submodules. `sensor/position.rs`
and `sensor/velocity.rs` already exist — move remaining sensor types out of
`mod.rs` into focused files.

**Checkpoint**: `cargo test -p sim-core -p sim-conformance-tests`

---

### T32 — Update MUJOCO_CONFORMANCE.md

**File**: `sim/docs/MUJOCO_CONFORMANCE.md`

**Issues found**:
1. Lines ~59, ~141, ~310: Sections 1–4 claim status `[ ] Not started` — but
   Phase 12 created a 4-layer conformance suite with 79/79 tests passing
2. Lines ~405–412: Progress table says "Conformance tests | Not started" and
   "Trajectory comparison | Not started" — both complete
3. Lines ~372–401: CI section says `(PLANNED — not yet created)` — but Phase 12
   tests are already in quality-gate.yml
4. Lines ~134–139: Action items list Phase 7 work as `[ ]` (not done) — done

**Action**: Rewrite to reflect actual status. Replace "not started" markers with
completion notes referencing Phase 12. Replace CI "planned" section with a note
that conformance tests run in quality-gate.yml via `sim-conformance-tests`.

**Checkpoint**: Visual review.

---

### T33 — Update ARCHITECTURE.md

**File**: `sim/docs/ARCHITECTURE.md`

**Issues found**:
1. Claims "~40 focused modules" in sim-core — actual count is ~83 `.rs` files
2. Claims "~19 modules" in sim-mjcf builder — actual count is ~21
3. May still reference structural-refactor-era organization

**Action**: Review entire file for accuracy. Update module counts. Verify
dependency graph matches current state. Ensure no references to deleted crates.

**Checkpoint**: Visual review.

---

### T34 — Clean stale crate references in phase spec docs

**Files**:
- `sim/docs/todo/spec_fleshouts/phase13_remaining_core/SESSION_PLAN.md` — test
  command includes `-p sim-physics -p sim-constraint` (deleted crates)
- `sim/docs/todo/spec_fleshouts/s41_runtime_flags/S41_RUNTIME_FLAGS_SPEC.md` — same
- `sim/docs/todo/spec_fleshouts/s41_runtime_flags/S41_AUDIT_PLAN.md` — same
- `sim/docs/todo/spec_fleshouts/phase6_sensor_completeness/SPEC_D_RUBRIC.md` —
  references `sim-sensor` and `sim-physics` as live crates

**Action**: Update test commands to match current scope:
`cargo test -p sim-core -p sim-mjcf -p sim-conformance-tests -p sim-urdf -p sim-types -p sim-simd`

Add a note at the top of completed phase specs: `> Historical — Phase N completed <date>`.

**Checkpoint**: Visual review.

---

### T35 — Update example crate help text

**Files** (example `main.rs` files with `println!()` referencing deleted crates):
- `examples/phase-4-deformable/pneumatic-finger/src/main.rs` (line ~17) — references `sim-deformable`
- `examples/phase-4-deformable/peristaltic-crawler/src/main.rs` (line ~17) — references `sim-deformable`
- `examples/phase-4-deformable/variable-stiffness-pad/src/main.rs` (line ~17) — references `sim-deformable`

These reference `sim-deformable` in user-facing help text. (The phase-5 and
phase-6 example files were already cleaned by LEGACY_CRATE_CLEANUP.)

**Action**: Update help text to reference sim-core flex pipeline instead. Remove
mentions of `sim-deformable` which no longer exists.

**Checkpoint**: `cargo build -p <each-example-crate>` (verify they compile)

---

### T36 — Consolidate future_work files into themed post-v1.0 roadmap

**Current state**: 9 files (`future_work_10b.md` through `future_work_10j.md`)
tracking 119 deferred tasks, organized by the original phase structure.

**Problem**: Now that v1.0 is complete, the phase-based organization is confusing.
A reader has to know which phase a feature was deferred from to find its tracker.

**Action**: Create a new `POST_V1_ROADMAP.md` organized by theme:
- Performance optimizations (20 items)
- Advanced differentiation (3 items)
- CortenForge extensions beyond MuJoCo (27 items)
- Low-priority MuJoCo compatibility (47 items)
- Code quality (DT-117 + items from this cleanup spec)

Move all DT items into the new file. Archive the old `future_work_10b–10j.md`
files into an `archived/` subdirectory.

**Checkpoint**: Visual review. Verify all DT numbers are preserved.

---

### T37 — Remove stale `lumen-geometry` path dependency

**File**: Root `Cargo.toml` (line ~296)
**Current**: `lumen-geometry = { path = "geometry/lumen-geometry" }` defined in
`[workspace.dependencies]` but the directory does not exist. Workspace member is
commented out (line ~77).
**Action**: Delete the workspace dependency line. The member is already commented
out. If the crate is ever needed again, it can be re-added from git history.

**Checkpoint**: `cargo build` (verify workspace still compiles)

---

### T38 — Remove stale "vision" domain from xtask

**File**: `xtask/src/grade.rs` (lines ~288–289, ~630–631)
**Current**: Crate search locations include `"vision"` — but no `vision/` directory
exists in the repository.
**Action**: Remove `"vision"` from the search location arrays. Add `"sensor"` if
the sensor domain crates exist at that path.

**Checkpoint**: `cargo build -p xtask`

---

### T39 — Extract magic numbers in sim-mjcf

**Files**:
- `sim/L0/mjcf/src/builder/flex.rs` (line ~353): `17 * global_e` where 17 is
  documented as "coefficients per bending element" — extract to
  `const BENDING_COEFFS: usize = 17`
- `sim/L0/mjcf/src/builder/flex.rs`: `0.001` used as minimum vertex mass —
  extract to `const MIN_VERTEX_MASS: f64 = 0.001`

**Action**: Replace magic numbers with named constants at the top of the file.
Add brief doc comments explaining each constant's physical meaning.

**Checkpoint**: `cargo test -p sim-mjcf`

---

### T40 — Review `mjb` feature in sim-mjcf

**File**: `sim/L0/mjcf/Cargo.toml` and `sim/L0/mjcf/src/mjb.rs` (659 lines)
**Current**: `mjb` feature enables binary MuJoCo model format support via
serde + bincode. The module has a full implementation with 15 unit tests under
`#[cfg(test)]` (not feature-gated — tests compile regardless of feature flag).

**What's missing**: No integration-level test exercises the feature from
outside the crate, and CI doesn't run `--features mjb`. The feature compiles
and unit-tests pass, but it's not exercised end-to-end.

**Action**:
1. Run `cargo test -p sim-mjcf --features mjb` to verify the feature compiles
   and all 15 unit tests pass
2. If healthy → add a one-line note to the CI quality gate or a comment in
   `Cargo.toml` documenting that `mjb` is an opt-in feature with unit tests
3. If broken → either fix or remove and track as a DT item

**Checkpoint**: `cargo test -p sim-mjcf --features mjb`

---

### T41 — Final verification pass

Run after all other tasks are complete.

1. `cargo build` — full workspace compiles
2. `cargo test -p sim-core -p sim-mjcf -p sim-conformance-tests -p sim-types -p sim-simd -p sim-urdf` — all sim tests pass
3. `cargo clippy -p sim-core -p sim-mjcf -p sim-types -p sim-urdf -- -D warnings` — no warnings
4. `cargo fmt --all -- --check` — formatting clean
5. Verify 79/79 conformance tests still pass
6. Compare test count against baseline to confirm no regressions
7. `cargo doc -p sim-core -p sim-mjcf -p sim-types --no-deps` — docs build without warnings

---

## What we are NOT doing (and why)

| Approach | Why not |
|----------|---------|
| DT-117 (unwrap/expect mass cleanup — ~1,085 sites) | Separate project with its own spec. Most unwraps are in expert code (SDF, derivatives, convex hull) where internal invariants justify them. |
| New lint rules | Scope creep. The current clippy config is validated by 79/79 conformance. |
| GPU pipeline cleanup | sim-gpu is pre-v1.0 infrastructure. Clean when GPU work resumes. |
| Rewriting algorithms | v1.0 algorithms are conformance-tested. Refactoring is structural (file splits, helper extraction), not algorithmic. |
| Adding `#[must_use]` / `#[inline]` annotations | Performance/API polish — separate spec if desired. |
| Removing the `sensor/position.rs` + `sensor/velocity.rs` duplicate of `get_ref_pos_mat()` | Intentional — independent dispatch matching MuJoCo's `mj_sensorPos`/`mj_sensorVel` separation. Documented in velocity.rs. |

---

## Risk Assessment

| Risk | Likelihood | Mitigation |
|------|-----------|------------|
| Removing "dead" code that has external consumers | Very low — not published to crates.io | Git history preserves everything |
| Visibility change breaks downstream | Low — grep-verified per task | Each task has checkpoint build |
| File split changes module paths | Medium — public re-exports must be preserved | Split tasks must verify `pub use` in parent `mod.rs` |
| Large function refactoring changes behavior | Very low — pure structural | Conformance tests (79/79) catch any drift |
| Docs reorganization loses information | None — content preserved, files archived not deleted | Old files moved to `archived/` |

---

## Related specs

- [LEGACY_CRATE_CLEANUP.md](LEGACY_CRATE_CLEANUP.md) — complete. Removed 5 legacy
  crates + ~22 POC types (~21k lines deleted).
