# Future Work 11 — Phase 3A: Cleanup + Conformance Test Suite (Items #43–45)

Part of [Simulation Phase 3 Roadmap](./index.md). See [index.md](./index.md) for
priority table, dependency graph, and file map.

Items #43–#44 are the final correctness gaps before the conformance suite. Item #45
is the conformance test suite that validates all preceding correctness work (#19–#44).

---

### 43. Geom `shellinertia`
**Status:** Not started | **Effort:** S | **Prerequisites:** None

#### Current State

Not parsed or stored. All mesh geoms compute inertia treating the mesh as a solid
volume. Thin-walled objects (cups, shells, tubes) get incorrect inertia.

#### Objective

Parse `shellinertia` from `<geom>` and use it to compute mesh inertia as a hollow
shell instead of a solid volume.

#### Specification

1. **MJCF parsing**: Parse `shellinertia` (bool, default false) from `<geom>`.
   Also parse from `<default>` class.
2. **Model storage**: Add `geom_shellinertia: Vec<bool>` to `Model`.
3. **Effect on mesh inertia**: When `shellinertia="true"` on a mesh geom:
   - Compute inertia treating each triangle as a surface element (shell) rather
     than a volume element.
   - Shell inertia: for each triangle, mass is proportional to area (not volume).
     The inertia contribution is computed from the triangle's area, centroid, and
     the thin-shell inertia formula.
   - Total mass is distributed over the mesh surface area.
4. **Interaction with #23 (`exactmeshinertia`)**: `shellinertia` and
   `exactmeshinertia` are orthogonal — `exactmeshinertia` controls solid
   volume inertia accuracy, `shellinertia` switches to shell computation entirely.
5. **Non-mesh geoms**: `shellinertia` has no effect on primitive geoms (sphere,
   box, capsule, etc.) — they always use their analytical inertia formulas.

#### Acceptance Criteria

1. `shellinertia="false"` (default) preserves current solid inertia behavior.
2. `shellinertia="true"` on a spherical mesh shell matches the analytical hollow
   sphere inertia (`2/3 * m * r²`).
3. Non-mesh geoms ignore `shellinertia`.
4. Default class inheritance works for the attribute.

#### Files

- `sim/L0/mjcf/src/model_builder.rs` — parse shellinertia
- `sim/L0/core/src/mujoco_pipeline.rs` — Model field, shell inertia computation

---

### 44. Legacy Crate Deprecation
**Status:** Not started | **Effort:** S | **Prerequisites:** None

#### Current State

Four standalone crates duplicate functionality that is fully implemented in the
pipeline (`sim-core`):

| Crate | Lines | Pipeline equivalent | Used by pipeline? |
|-------|-------|--------------------|--------------------|
| `sim-constraint` | 10,359 | PGS/CG/Newton in `mujoco_pipeline.rs` | **No** |
| `sim-muscle` | 2,550 | MuJoCo FLV in `mj_fwd_actuation()` | **No** |
| `sim-tendon` | 3,919 | `mj_fwd_tendon_fixed/spatial()` | **No** |
| `sim-sensor` | — | 32 sensor types in `mj_sensor_*()` | **No** |

These crates are re-exported by `sim-physics` (the umbrella crate) but have zero
callers in the pipeline. They confuse contributors ("which PGS?"), inflate
compile times, and create a false impression of coverage.

#### Objective

Deprecate standalone crates that are fully superseded by pipeline implementations.

#### Specification

1. **sim-tendon**: Mark as `#[deprecated]`. Add top-level doc comment directing
   users to pipeline `mj_fwd_tendon_*()`. Remove from default workspace members.
2. **sim-muscle**: Mark as `#[deprecated]`. Note the standalone Hill model is
   richer than pipeline FLV but not MuJoCo-compatible. Keep available for
   biomechanics users who don't need MuJoCo conformance.
3. **sim-constraint**: Keep crate but audit public API. Remove re-exports of
   deleted types (PGSSolver, NewtonSolver, etc.). Document which types are
   standalone-only vs pipeline-compatible.
4. **sim-sensor**: Keep — provides standalone hardware sensor API independent of
   pipeline.

#### Acceptance Criteria

1. Deprecated crates emit compiler warnings on use.
2. `cargo doc` shows clear deprecation notices with migration guidance.
3. No change to pipeline behavior (regression).
4. Workspace compiles cleanly (no dead-code warnings in deprecated crates).

#### Files
- `sim/L0/tendon/src/lib.rs` — deprecation attributes
- `sim/L0/muscle/src/lib.rs` — deprecation attributes
- `sim/L0/constraint/src/lib.rs` — API audit
- `Cargo.toml` (workspace) — optional default-members adjustment

---

### 45. MuJoCo Conformance Test Suite
**Status:** Not started | **Effort:** XL | **Prerequisites:** #19–#44

#### Current State

Testing is ad-hoc: analytical FK/CRBA ground truth (9 tests in `validation.rs`),
finite-difference derivative checks (30+ in `derivatives.rs`), and hardcoded
MuJoCo 3.4.0 reference values for spatial tendons (18 in `spatial_tendons.rs`).
No systematic per-stage comparison, no trajectory-level validation, no
self-consistency or property-based tests.

#### Objective

Build a four-layer conformance test suite that validates CortenForge against
MuJoCo 3.4.0 at every pipeline stage, catches regressions via trajectory
comparison, and enforces physical invariants. The spec will be written fresh
once all 26 prerequisite items (#19–#44) are complete.

#### Specification

1. **Layer A — Self-consistency** (no MuJoCo dependency): forward/inverse
   equivalence, island/monolithic solver equivalence, determinism,
   integrator energy ordering, sparse/dense mass matrix equivalence.
2. **Layer B — Per-stage reference**: compare each pipeline stage output
   (FK, CRBA, RNE, passive, collision, constraint, actuation, sensors,
   tendons, integration) against MuJoCo 3.4.0 hardcoded values.
3. **Layer C — Trajectory comparison**: multi-step trajectory tests with
   per-step per-field diagnostics to isolate which stage first diverges.
4. **Layer D — Property/invariant tests** (no MuJoCo dependency): momentum
   conservation, energy conservation, quaternion normalization, contact
   force feasibility, mass matrix properties.
5. **Reference generation**: Python script pinned to `mujoco==3.4.0`,
   outputs checked into repo. No MuJoCo build dependency for Rust tests.
6. **Detailed spec deferred**: models, tolerances, test models, and
   acceptance criteria will be written against the post-fix codebase when
   all prerequisites land. Previous detailed spec was removed — it was
   written when only 14 of the current 26 gaps were known.

#### Acceptance Criteria

1. Self-consistency tests (Layer A) pass without MuJoCo installed.
2. Per-stage reference tests (Layer B) pass at algorithm-appropriate tolerances.
3. Trajectory tests (Layer C) pass for contact and non-contact scenarios.
4. Property tests (Layer D) verify conservation laws and invariants.
5. `cargo test -p sim-conformance-tests` works without MuJoCo installed.
6. No existing tests in `integration/` broken or moved.

#### Files

- `sim/L0/tests/conformance/` — test suite directory
- `sim/L0/tests/conformance/generate_references.py` — Python reference generator
- `sim/L0/tests/integration/` — unchanged (existing tests stay)
