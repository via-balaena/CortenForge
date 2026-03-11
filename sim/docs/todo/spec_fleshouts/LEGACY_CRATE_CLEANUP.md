# Legacy Crate Cleanup — Post-v1.0

> **Status**: Complete — all phases verified
> **Supersedes**: §44 in `future_work_11.md` (which proposed deprecation-in-place)
> **Principle**: Foundational fix. Remove dead code, don't decorate it with warnings.

---

## Execution Model

Run **one phase per session**. Each session:

1. Read this spec
2. Execute the next incomplete phase
3. Run that phase's checkpoint build
4. Commit (with permission)

**Starting prompt** (paste at the start of each new session):

> Continue executing the legacy crate cleanup spec at
> `sim/docs/todo/spec_fleshouts/LEGACY_CRATE_CLEANUP.md`.
> Pick up from the next incomplete phase. Run the checkpoint build after
> the phase. Ask before committing. One phase per session.

### Progress

- [x] **Phase A** — Remove downstream references to legacy crates
- [x] **Phase B** — Delete legacy crates
- [x] **Phase C** — Clean up sim-types
- [x] **Phase D** — Clean up sim-core re-exports (done in Phase C to maintain compilability)
- [x] **Phase E** — Update documentation and CI
- [x] **Phase F** — Verify

---

## Problem

During the pre-v1.0 refactor, the MuJoCo-conformant pipeline was built entirely
inside `sim-core` (constraint assembly, sensors, tendons, muscles, collision,
integration — all as internal modules). Four standalone crates from the original
proof-of-concept were never removed:

| Crate | Lines | sim-core equivalent | Depended on by pipeline? |
|-------|------:|---------------------|--------------------------|
| `sim-constraint` | 10,359 | `core/src/constraint/` (8,596 lines) | **No** |
| `sim-sensor` | 3,664 | `core/src/sensor/` (2,956 lines) | **No** |
| `sim-tendon` | 3,919 | `core/src/tendon/` (1,350 lines) | **No** |
| `sim-muscle` | 2,550 | `core/src/forward/actuation.rs` + Hill-type in Phase 5 | **No** |
| `sim-physics` | 439 | N/A (pure re-export umbrella, zero original code) | **No** |
| **Total** | **20,931** | | |

These crates:
- Are not called by any v1.0 pipeline code
- Confuse contributors ("which constraint solver?", "which sensor system?")
- Inflate compile times (~20k lines compiled for nothing)
- Create a false impression of coverage
- Are re-exported through `sim-physics`, making the public API misleading

Additionally, `sim-types` contains ~22 POC-era types that the v1.0 pipeline
does not use (Action, Observation, JointCommand, etc.), and `sim-physics` is
an umbrella re-exporter whose primary value was bundling these legacy crates.

---

## Scope

### In scope
1. Remove 4 legacy crates (`sim-constraint`, `sim-sensor`, `sim-tendon`, `sim-muscle`)
2. Clean up `sim-types` (remove unused POC types)
3. Clean up or remove `sim-physics` (umbrella crate)
4. Clean up example crate Cargo.toml dead dependencies
5. Update workspace members in root `Cargo.toml`
6. Update `ARCHITECTURE.md` and other docs

### Out of scope
- Any changes to v1.0 pipeline **behavior** (sim-core internals, sim-mjcf parsing, sim-urdf, sim-simd, sim-gpu, sim-bevy)
- Any changes to the conformance test suite (except removing dead sim-sensor import)

---

## Current Dependency Graph

```
                    ┌─────────────┐
                    │  sim-types  │  ← foundation (has POC types mixed in)
                    └──────┬──────┘
                           │
              ┌────────────┼────────────┐
              │            │            │
        ┌─────┴─────┐ ┌───┴───┐  ┌────┴────┐
        │ sim-simd  │ │       │  │sim-urdf │
        └─────┬─────┘ │       │  └────┬────┘
              │        │       │       │
        ┌─────┴────────┴───┐  │  ┌────┴────┐
        │    sim-core      │  │  │sim-mjcf │
        │ (v1.0 pipeline)  │◄─┘  │(parser) │
        └──────────────────┘     └─────────┘
              ▲
              │  gpu-internals feature
        ┌─────┴─────┐
        │  sim-gpu  │
        └───────────┘

  ═══════════════════════════════════════════════
  LEGACY (not depended on by anything above):

        sim-constraint ─── sim-types
        sim-sensor ─────── sim-types
        sim-tendon ─────── sim-types
        sim-muscle ─────── (standalone, no sim-types)

        sim-physics ─── re-exports ALL of the above
  ═══════════════════════════════════════════════
```

---

## Target Dependency Graph

```
                    ┌─────────────┐
                    │  sim-types  │  ← cleaned: only types used by sim-core
                    └──────┬──────┘
                           │
              ┌────────────┼────────────┐
              │            │            │
        ┌─────┴─────┐ ┌───┴───┐  ┌────┴────┐
        │ sim-simd  │ │       │  │sim-urdf │
        └─────┬─────┘ │       │  └────┬────┘
              │        │       │       │
        ┌─────┴────────┴───┐  │  ┌────┴────┐
        │    sim-core      │◄─┘  │sim-mjcf │
        │ (v1.0 pipeline)  │     │(parser) │
        └──────────────────┘     └─────────┘
              ▲
              │
        ┌─────┴─────┐
        │  sim-gpu  │
        └───────────┘
```

Gone: `sim-constraint`, `sim-sensor`, `sim-tendon`, `sim-muscle`, `sim-physics`.

---

## Blast Radius Audit

### What depends on legacy crates?

| Consumer | Legacy dep | Impact | Action |
|----------|-----------|--------|--------|
| **sim-core** | None | None | No change |
| **sim-mjcf** | None | None | No change |
| **sim-urdf** | None | None | No change |
| **sim-gpu** | None | None | No change |
| **sim-bevy** | None | None | No change |
| **sim-physics** | All 4 | Breaks | Remove crate |
| **sim-conformance-tests** | sim-sensor | 1 test file | Remove standalone sensor tests |
| **13 example crates** | Declared only | Dead deps | Remove from Cargo.toml |
| **mesh/ml/route/sensor domains** | None | None | No change |

### Key finding: zero runtime impact
The v1.0 pipeline (`sim-core` + `sim-mjcf` + `sim-types` + `sim-simd`) has
**zero dependencies** on any legacy crate. Removing them cannot change any
simulation behavior.

---

## Phases

> **Execution order**: A → B → C → D → E → F (alphabetical).
>
> **Why this order matters**: Phase A removes dead references from example crates
> and conformance tests *before* Phase B deletes the crate directories. If B ran
> first, `cargo build` would fail because 13 example `Cargo.toml` files and
> `sim-conformance-tests` still reference the deleted crates. Each phase leaves
> the workspace in a compilable state.

### Phase A — Remove downstream references to legacy crates

**A.1 — Example crates** (13 crates, `Cargo.toml` only — all are unimplemented stubs):

Remove `sim-constraint` and/or `sim-sensor` from each crate's `[dependencies]`.
No example crate depends on `sim-tendon`, `sim-muscle`, or `sim-physics`.

| Example crate | Remove |
|---------------|--------|
| `examples/phase-2-mechanism/prosthetic-finger` | `sim-constraint` |
| `examples/phase-2-mechanism/compliant-gripper` | `sim-constraint` |
| `examples/phase-2-mechanism/four-bar-walker` | `sim-constraint` |
| `examples/phase-3-perception/obstacle-avoiding-robot` | `sim-sensor` |
| `examples/phase-3-perception/gesture-controlled-arm` | `sim-constraint`, `sim-sensor` |
| `examples/phase-3-perception/leveling-platform` | `sim-constraint`, `sim-sensor` |
| `examples/phase-4-deformable/variable-stiffness-pad` | `sim-sensor` |
| `examples/phase-5-learning/tactile-adaptive-gripper` | `sim-sensor` |
| `examples/phase-5-learning/tendon-driven-hand` | `sim-constraint` |
| `examples/phase-5-learning/antagonistic-hopping-leg` | `sim-constraint`, `sim-sensor` |
| `examples/phase-6-capstone/bio-inspired-manipulator` | `sim-constraint`, `sim-sensor` |
| `examples/phase-6-capstone/soft-robotic-massage-device` | `sim-constraint`, `sim-sensor` |
| `examples/phase-6-capstone/adaptive-prosthetic-socket` | `sim-constraint`, `sim-sensor` |

**A.2 — sim-conformance-tests**:

- Remove `sim-sensor` dependency from `sim/L0/tests/Cargo.toml`
- Remove `sim/L0/tests/integration/sensors.rs` (tests standalone sensor crate, not pipeline)
- Remove `mod sensors;` declaration from `sim/L0/tests/integration/mod.rs`
- Pipeline sensor conformance is already covered by Layer B/C/D tests

**Checkpoint**: `cargo build` passes. No crate references the legacy crates anymore
(except `sim-physics`, which is deleted in Phase B).

### Phase B — Delete legacy crates

Delete directories:
- `sim/L0/constraint/` (entire directory)
- `sim/L0/sensor/` (entire directory)
- `sim/L0/tendon/` (entire directory)
- `sim/L0/muscle/` (entire directory)
- `sim/L0/physics/` (entire directory — umbrella crate)

Remove from `[workspace.members]` in root `Cargo.toml` (keep sim-mjcf, sim-urdf — they're v1.0):
- `"sim/L0/constraint"` (line 101)
- `"sim/L0/sensor"` (line 102)
- `"sim/L0/muscle"` (line 103)
- `"sim/L0/tendon"` (line 104)
- `"sim/L0/physics"` (line 107)

Remove from `[workspace.dependencies]` in root `Cargo.toml` (keep sim-mjcf, sim-urdf):
- `sim-constraint = { path = "sim/L0/constraint" }` (line 325)
- `sim-sensor = { path = "sim/L0/sensor" }` (line 326)
- `sim-muscle = { path = "sim/L0/muscle" }` (line 327)
- `sim-tendon = { path = "sim/L0/tendon" }` (line 328)
- `sim-physics = { path = "sim/L0/physics" }` (line 331)

**Checkpoint**: `cargo build` passes.

### Phase C — Clean up sim-types

Remove unused POC types from `sim/L0/types/src/`.

**Verified: types with 0 pipeline uses (grepped sim-core, sim-mjcf, sim-urdf, sim-bevy).**

**Files to remove entirely:**
- `observation.rs` — `Observation`, `ObservationType`, `SensorObservation`, `ContactInfo`,
  `ContactStats`, `PoseObservation`, `VelocityObservation` (0 uses in pipeline)

**File to gut (keep `Gravity` only, move it to `config.rs`):**
- `dynamics.rs` — remove `Action`, `ActionType`, `JointCommand`, `JointCommandType`,
  `ExternalForce` (0 uses in pipeline). **`Gravity` is defined here but actively used by
  sim-mjcf** (3 uses in `sim-mjcf/src/config.rs`). Migration sub-steps:
  1. Copy `Gravity` struct + `impl Default` + `impl Gravity` (6 methods) + `test_gravity`
     test from `dynamics.rs` into `config.rs`
  2. Remove `use crate::dynamics::Gravity;` from `config.rs` (line 6)
  3. Add `use nalgebra::Vector3;` to `config.rs` — `Gravity` uses `Vector3<f64>` and
     config.rs has no nalgebra import (its only imports are the Gravity re-import and serde)
  4. Preserve `#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]` on the struct
  5. Delete `dynamics.rs`
  6. Update `lib.rs`: change `pub use dynamics::Gravity` to `pub use config::Gravity`

**`error.rs` — KEEP (trimmed).** `SimError` is used by `config.rs`'s `validate()` methods:
- `SimulationConfig::validate()` → `SimError::InvalidTimestep`, `SimError::invalid_config()`
- `SolverConfig::validate()` → `SimError::invalid_config()`
- `pub type Result<T>` alias depends on `SimError`

Keep `error.rs` and `thiserror` dependency. **Trim to only the variants used by kept code.**
8 of 11 variants reference concepts being removed (bodies, joints, actions, mass properties,
divergence) — keeping them creates a false impression that sim-types still deals with these.

Keep:
- `InvalidTimestep(f64)`
- `InvalidConfig { reason: String }`
- `invalid_config()` helper
- `is_config_error()` helper

Remove:
- Variants: `InvalidBodyId`, `InvalidJointId`, `JointLimitViolation`, `Diverged`,
  `ActionTypeMismatch`, `BodyNotFound`, `JointNotFound`, `ParentRequired`,
  `InvalidMassProperties`
- Helpers: `diverged()`, `invalid_mass()`, `is_diverged()`
- Tests: replace with tests covering only `InvalidTimestep`, `InvalidConfig`,
  and the `invalid_config()`/`is_config_error()` helpers

**Types to remove from remaining files:**

From `body.rs` (keep `BodyId`, `Pose` — remove everything else):
- `RigidBodyState` (struct + 5 methods + Default impl) — 0 pipeline uses
- `MassProperties` (struct + 9 methods, including `validate()`) — 0 pipeline uses
- `Twist` (struct + 14 methods + Default impl) — 0 pipeline uses
- Update import line: `use nalgebra::{Isometry3, Matrix3, Point3, UnitQuaternion, Vector3};`
  → `use nalgebra::{Isometry3, Point3, UnitQuaternion, Vector3};` (`Matrix3` only used by removed types)
- Update module doc comment: "Rigid body state types" → "Body identity and pose types"
- Remove 6 tests that reference removed types:
  `test_twist_velocity_at_point`, `test_twist_kinetic_energy`,
  `test_mass_properties_sphere`, `test_mass_properties_box`,
  `test_mass_properties_validation`, `test_rigid_body_state_interpolation`
- Keep 7 tests: `test_body_id`, `test_pose_identity`, `test_pose_translation`,
  `test_pose_rotation`, `test_pose_inverse`, `test_pose_compose`, `test_pose_lerp`

From `joint.rs` (keep nothing — delete entire file):
- `JointId` — only re-exported by sim-core, 0 internal uses
- `JointType` — only re-exported by sim-core; pipeline uses `MjJointType` (187 uses in 23 files)
- `JointLimits` — only re-exported by sim-core, 0 internal uses
- `JointState` — 0 pipeline uses
- `JointAxis` — 0 pipeline uses
- `JointStateExtended` — 0 pipeline uses
- All 6 types + 10 tests confirmed unused. Delete `joint.rs` entirely.

**Other cleanup in sim-types:**
- Remove `glam` from `sim-types/Cargo.toml` dependencies
- Remove `"glam/serde"` from the `serde` feature in `sim-types/Cargo.toml`
  (current: `serde = ["dep:serde", "nalgebra/serde-serialize", "glam/serde"]`
  → becomes: `serde = ["dep:serde", "nalgebra/serde-serialize"]`)
- Update `Cargo.toml` description (currently: "Core types for physics simulation: state, action,
  observation, dynamics" → something like "Foundation types for sim-core: body identity, pose,
  simulation configuration")
- Update `lib.rs` module-level doc comment — currently references `RigidBodyState`, `Action`,
  `Observation`, and has a code example using `RigidBodyState`/`Twist` (all being removed).
  Rewrite to reference the kept types (`BodyId`, `Pose`, `SimulationConfig`, `Gravity`).
- Update `lib.rs` tests — `test_rigid_body_state` uses removed types (remove it);
  `test_pose_transform` uses only `Pose` (keep it)

**`lib.rs` final state** (module declarations + re-exports after all Phase C changes):

```rust
mod body;
mod config;
mod error;

pub use body::{BodyId, Pose};
pub use config::{Gravity, ParallelConfig, SimulationConfig, SolverConfig};
pub use error::SimError;

// Re-export math types for convenience
pub use nalgebra::{Isometry3, Point3, UnitQuaternion, Vector3};

/// Result type for simulation operations.
pub type Result<T> = std::result::Result<T, SimError>;
```

Removed from current state: `mod dynamics;`, `mod joint;`, `mod observation;`,
`pub use glam::{Quat, Vec3}`, and all re-exports of deleted types.

**Types to KEEP (verified active pipeline uses):**
- `BodyId` — ~80 uses in sim-core + 1 in sim-bevy
- `Pose` — ~255 uses in sim-core
- `Gravity` — 3 uses in sim-mjcf (move from dynamics.rs to config.rs)
- `SimulationConfig` — 8 uses in sim-mjcf
- `SolverConfig` — 2 uses in sim-mjcf
- `ParallelConfig` — 2 uses in sim-mjcf
- `SimError` — used by config.rs validate() methods
- `Result<T>` — type alias for `std::result::Result<T, SimError>`

**After cleanup, sim-types contains only:**
- `body.rs` — `BodyId`, `Pose` (+ 7 kept tests)
- `config.rs` — `SimulationConfig`, `SolverConfig`, `ParallelConfig`, `Gravity`
- `error.rs` — `SimError::InvalidTimestep`, `SimError::InvalidConfig`, `invalid_config()`,
  `is_config_error()` (2 variants, down from 11)
- `lib.rs` — re-exports + nalgebra convenience re-exports + `Result<T>` alias

### Phase D — Clean up sim-core re-exports

`sim-core/src/lib.rs:276-280` re-exports many now-removed types:

```rust
pub use sim_types::{
    Action, ActionType, BodyId, ExternalForce, Gravity, JointCommand, JointCommandType, JointId,
    JointLimits, JointState, JointType, MassProperties, Observation, ObservationType, Pose,
    RigidBodyState, SimError, SimulationConfig, SolverConfig, Twist,
};
```

Replace with only the types that sim-core actually uses internally:

```rust
pub use sim_types::{BodyId, Pose};
```

**Note**: Every other re-exported type — including `Gravity`, `SimulationConfig`,
`SolverConfig`, and all 15 POC types — is never used in sim-core's own source code
(only in the re-export block). `Gravity` appears in sim-core doc comments but not in
code; sim-core's `Model.gravity` field is `nalgebra::Vector3<f64>`, not `sim_types::Gravity`.
Consumers that need these types import them directly from `sim-types` (e.g., sim-mjcf
uses `use sim_types::{Gravity, SimulationConfig, SolverConfig}`). Removing them from
sim-core's re-exports is a public API change, but no code in the workspace uses them
via sim-core.

### Phase E — Update documentation and CI

**Documentation:**
- `sim/docs/ARCHITECTURE.md` — remove references to legacy crates, update directory listing
- `sim/docs/todo/future_work_11.md` — mark §44 as done (superseded by this cleanup)
- `sim/docs/ROADMAP_V1.md` — mark §44 as done in post-v1.0 section
- `sim/docs/todo/index.md` — update if it references legacy crates
- Any other docs that reference sim-constraint, sim-sensor, sim-tendon, sim-muscle, sim-physics

**CI/CD (hardcoded legacy crate references):**

Only `sim-constraint` and `sim-physics` appear in CI — sim-sensor, sim-tendon, and
sim-muscle are confirmed absent from both files.

- `.github/workflows/quality-gate.yml` — remove `sim-constraint` and `sim-physics` from:
  - WASM check crate array (lines 156, 158)
  - Bevy-free / Layer 0 crate array (lines 398, 400)
- `scripts/local-quality-check.sh` — remove `sim-constraint` and `sim-physics` from:
  - `check_wasm()` WASM_CRATES array (lines 187, 189)
  - `check_bevy_free()` LAYER0_CRATES array (lines 239, 241)

### Phase F — Verify

1. `cargo build` — workspace compiles
2. `cargo test -p sim-core -p sim-mjcf -p sim-types -p sim-conformance-tests` — pipeline tests pass
3. `cargo clippy -- -D warnings` — no new warnings
4. `cargo fmt --all -- --check` — formatting clean
5. Confirm test count hasn't regressed (minus the removed standalone sensor tests)

---

## What we are NOT doing (and why)

| Approach | Why not |
|----------|---------|
| Deprecate-in-place (`#[deprecated]`) | Leaves 20k lines of dead code. Warnings don't remove confusion. |
| Move to `sim-legacy/` directory | Still compiled, still confusing. Archive ≠ delete. |
| Keep sim-sensor "for hardware use" | v1.0 sensors are MuJoCo-conformant and more complete. Standalone sensor crate is a different abstraction that nobody uses. |
| Keep sim-physics umbrella | Its only remaining value would be re-exporting sim-core + sim-types + sim-mjcf, which users can import directly. Umbrella adds indirection without value. |

---

## Risk Assessment

| Risk | Likelihood | Mitigation |
|------|-----------|------------|
| External user depends on legacy crate | Very low — not published to crates.io | Git history preserves everything |
| Example crate needs legacy type later | Low — stubs will be rewritten against v1.0 API | Can add deps back when implementing |
| sim-types removal breaks sim-core | Low — sim-core re-exports 20 types, only 2 are used internally (BodyId, Pose) | Phase D cleans up the re-export block; grep-verified |
| Gravity migration breaks sim-mjcf | Very low — move within sim-types, same pub API | Explicit sub-steps in Phase C; sim-mjcf imports from `sim_types::Gravity` (path-independent) |
| SimError removal breaks config.rs | N/A — addressed in spec | error.rs is KEPT; config.rs validate() methods depend on SimError |
| Forgotten import somewhere | Low | Each phase has a `cargo build` checkpoint |

---

## Estimated Scope

- **Phase A**: ~30 min (remove legacy deps from 13 example crates + conformance tests)
- **Phase B**: ~15 min (delete 5 legacy crate dirs, update workspace Cargo.toml)
- **Phase C**: ~1 hr (move Gravity, trim sim-types + error.rs, update lib.rs/Cargo.toml)
- **Phase D**: ~15 min (clean up sim-core re-export block)
- **Phase E**: ~30 min (update docs + CI/CD scripts)
- **Phase F**: ~10 min (build + test + verify)

**Total: ~2.5 hrs of careful work.**
Net deletion: ~21,000+ lines (including ~20,931 from legacy crates + POC types + dead re-exports).
