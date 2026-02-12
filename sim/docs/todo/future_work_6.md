# Future Work 6 — Phase 3A: Foundation + Core Correctness (Items #18–22)

Part of [Simulation Phase 3 Roadmap](./index.md). See [index.md](./index.md) for
priority table, dependency graph, and file map.

Items are ordered foundationally: `<include>` unlocks loading production models,
the conformance suite unlocks verifying everything that follows, then core contact
correctness, then solver/actuator completeness.

---

### 18. `<include>` File Support + `<compiler>` Element
**Status:** Not started | **Effort:** L | **Prerequisites:** None

#### Current State
The MJCF parser returns `IncludeNotSupported` when encountering `<include file="..."/>`.
Many real-world MJCF models (MuJoCo Menagerie, DeepMind Control Suite) use `<include>`
for modular definitions. This blocks loading a significant fraction of production models
and is a prerequisite for testing most subsequent items against real-world scenes.

Additionally, the `<compiler>` element is completely skipped during parsing. This is
**critical** because:
- `angle="degree"` (MuJoCo's default) controls whether joint ranges, euler angles,
  and axis-angles are in degrees or radians. Without this, models are silently
  misinterpreted.
- `meshdir` / `texturedir` control asset path resolution, which is required for
  `<include>` to work correctly with relative paths.
- `coordinate`, `eulerseq`, `autolimits`, `inertiafromgeom`, `settotalmass`,
  `balanceinertia` all affect model construction.

These two features are tightly coupled — `<include>` needs `<compiler>` path
attributes, and both are prerequisites for loading production MJCF models.

#### Objective
Implement `<include>` element resolution and `<compiler>` element parsing.

#### Specification

**Part A — `<compiler>` element:**

1. **`angle`** (critical): Parse `"degree"` (default) or `"radian"`. When
   `angle="degree"`, convert all joint `range`, `ref`, body `euler`/`axisangle`,
   and geom `euler`/`axisangle` values from degrees to radians during model building.
2. **`meshdir` / `texturedir`**: Store directory paths. Resolve mesh/texture file
   references relative to these directories (falling back to model file directory).
3. **`coordinate`**: Parse `"local"` (default) or `"global"`. When `"global"`, body
   positions/orientations are absolute rather than parent-relative.
4. **`eulerseq`**: Parse Euler angle sequence (default `"xyz"`). Apply when
   converting `euler` orientation specifications.
5. **`autolimits`**: When `"true"`, infer `limited="true"` from the presence of
   `range` attributes (MuJoCo ≥2.3.3 behavior).
6. **`inertiafromgeom`**: Parse `"true"/"false"/"auto"`. The `compute_inertia_from_geoms()`
   function exists but is not gated by this attribute.
7. **`settotalmass`**: Rescale all body masses so total mass matches the specified value.
8. **`balanceinertia`**: Make all inertia matrices positive-definite (project
   eigenvalues).

**Part B — `<include>` files:**

1. **Pre-parse expansion**: Before DOM processing, recursively resolve `<include>`
   elements by reading the referenced file and inserting its XML children in place.
2. **Path resolution**: Relative paths resolve against the directory of the including
   file. Use `meshdir`/`texturedir` from `<compiler>` (Part A) for asset paths.
3. **Cycle detection**: Track include stack; error on circular includes.
4. **Nested includes**: Support includes within included files (recursive).

#### Acceptance Criteria
1. `<compiler angle="degree"/>` correctly converts joint ranges from degrees to radians.
2. `<compiler angle="radian"/>` preserves values as-is (matching current behavior).
3. `<compiler meshdir="assets/"/>` resolves mesh paths relative to specified directory.
4. MuJoCo Menagerie models with `<include>` load successfully.
5. Circular includes produce a clear error message.
6. Models without `<include>` or `<compiler>` are unaffected (regression).

#### Files
- `sim/L0/mjcf/src/lib.rs` — XML pre-processing / include expansion
- `sim/L0/mjcf/src/parser.rs` — `<compiler>` element parsing
- `sim/L0/mjcf/src/model_builder.rs` — angle conversion, path context, compiler settings

---

### 19. MuJoCo Conformance Test Suite
**Status:** Not started | **Effort:** XL | **Prerequisites:** None (benefits from #18)

#### Current State
Model loading is tested (MuJoCo Menagerie + DeepMind Control Suite all load
successfully). Numerical trajectory conformance is **not** tested — acceptance
criteria for all implemented features rely on ad-hoc verification rather than
systematic comparison against MuJoCo reference outputs.

This is foundational infrastructure: every subsequent physics item (#20–#28) should
be verified against MuJoCo ground truth via this harness.

#### Objective
Build an automated conformance harness that compares CortenForge simulation
trajectories against MuJoCo ground truth.

#### Specification

1. **Reference generation**: Run MuJoCo (C API via mujoco-sys or Python bindings)
   on a set of test models, recording `qpos`, `qvel`, `qacc`, `sensordata` at each
   step. Store as binary reference files (model + N steps + state vectors).
2. **Test harness**: For each reference file:
   - Load the same MJCF model in CortenForge
   - Step N times with identical `ctrl` inputs
   - Compare state vectors against reference at each step
   - Report per-DOF relative error, max error, and divergence step
3. **Tolerance tiers**:
   - **Exact** (< 1e-10): Same algorithm, same precision (e.g., FK, CRBA)
   - **Tight** (< 1e-6): Same algorithm, minor implementation differences
   - **Loose** (< 1e-2): Different algorithm or accumulation order (e.g., PGS
     iteration order, contact detection order)
4. **Test models** (minimum set):
   - `humanoid.xml` — complex articulated body, contacts, actuators
   - `ant.xml` — RL benchmark, 4 legs, ground contact
   - `cartpole.xml` — minimal joint chain, no contacts
   - `shadow_hand.xml` — high DOF, tendons, many contacts
   - `cloth_grid.xml` — deformable (when `<composite>` supported)
5. **CI integration**: Run as `cargo test -p sim-conformance-tests --features mujoco`
   (feature-gated to avoid requiring MuJoCo in default builds).

#### Acceptance Criteria
1. ≥5 reference models with 100-step trajectories.
2. Tight tolerance (< 1e-6) for FK, CRBA, RNE on all models.
3. Loose tolerance (< 1e-2) for constrained dynamics (contact forces are
   solver-order-dependent).
4. CI runs pass on every PR.

#### Files
- `sim/L0/tests/conformance/` — reference data + test harness
- `sim/L0/tests/conformance/generate_references.py` — MuJoCo reference generator

---

### 20. Contact Margin/Gap Runtime Effect
**Status:** Not started | **Effort:** S | **Prerequisites:** None

#### Current State
`geom_margin` and `gap` fields are parsed from MJCF and stored on `Contact` and
`Model.geom_margin`, but have **no runtime effect** on contact activation distance.
Comment at `mujoco_pipeline.rs:5664`: "margin/gap are NOT applied here."

In MuJoCo, `margin` widens the contact activation distance — contacts are generated
when `dist < margin` rather than `dist < 0`. The `gap` attribute further shifts the
reference distance for Baumgarte stabilization. Many MJCF models rely on margin for
smooth contact activation (avoiding sudden force spikes).

#### Objective
Wire `margin` and `gap` into the contact detection and constraint assembly pipeline
so contacts activate at the correct distance and stabilization uses the correct
reference.

#### Specification

1. **Contact activation** (`mj_collision` / narrow-phase): Generate contacts when
   `penetration_depth > -margin` (currently: `penetration_depth > 0`).
2. **Constraint assembly** (`assemble_contact_system`): Use `depth - gap` as the
   effective penetration for Baumgarte stabilization (currently: raw `depth`).
3. **Per-geom margin**: `margin = max(geom1.margin, geom2.margin)` (MuJoCo semantics).

#### Acceptance Criteria
1. A sphere resting 1mm above a plane with `margin="0.002"` generates a contact.
2. `gap` shifts the equilibrium penetration for compliant contacts.
3. Zero margin/gap reproduces current behavior (regression).

#### Files
- `sim/L0/core/src/mujoco_pipeline.rs` — contact generation + constraint assembly

---

### 21. Noslip Post-Processor
**Status:** Parsed, partially implemented | **Effort:** S | **Prerequisites:** None

#### Current State
`noslip_iterations` and `noslip_tolerance` are parsed from MJCF and stored in `Model`.
The Newton solver has a noslip code path, but PGS/CG solvers do not. MuJoCo's noslip
post-processor (`mj_solNoSlip`) is a secondary PGS sweep on friction rows only, using
`AR - R` (removing the regularizer) to suppress artificial slip.

#### Objective
Implement the noslip PGS post-processor for all solver types.

#### Specification

1. **When to run**: After the main constraint solve, if `noslip_iterations > 0`.
2. **What it solves**: Only friction-dimension constraint rows (tangent, torsional,
   rolling). Normal forces are fixed from the main solve.
3. **Modified diagonal**: Use `1 / A[i,i]` instead of `1 / (A[i,i] + R[i])` —
   removing the regularizer makes constraints "hard" (no softness).
4. **Projection**: Same friction cone projection as main PGS.
5. **Convergence**: `noslip_tolerance` for early termination.

#### Acceptance Criteria
1. `noslip_iterations=3` reduces tangential slip on a box-on-slope benchmark.
2. `noslip_iterations=0` matches current behavior (regression).
3. Normal forces are not modified by the noslip pass.

#### Files
- `sim/L0/core/src/mujoco_pipeline.rs` — new `mj_sol_noslip()` function, called
  after main solve

---

### 22. Body-Transmission Actuators (Adhesion)
**Status:** Not started | **Effort:** M | **Prerequisites:** None

#### Current State
`model_builder.rs:1671` returns an error: "uses body transmission which is not yet
supported." MuJoCo adhesion actuators use body-level transmission — the actuator
force is applied as a contact-modifying force on all contacts involving the specified
body, rather than through joint/tendon/site Jacobians.

#### Objective
Implement body transmission so adhesion actuators can load and function.

#### Specification

1. **Transmission type**: `TransmissionType::Body(body_id)` — identifies target body.
2. **Force application**: In `mj_fwd_constraint()` (or post-solve), for each adhesion
   actuator with body transmission:
   - Find all active contacts involving `body_id`
   - Add adhesion force to normal contact force: `F_n += gain * ctrl`
   - This allows negative normal force (sticking), bounded by `forcerange`
3. **MJCF parsing**: `<actuator><adhesion body="..." .../>` resolves body name to ID,
   sets `trntype="body"`.

#### Acceptance Criteria
1. Adhesion actuator with body transmission loads without error.
2. Positive control input increases normal contact force (pressing).
3. Negative contact forces allow body to "stick" to surfaces.
4. No adhesion actuator → existing behavior unchanged (regression).

#### Files
- `sim/L0/mjcf/src/model_builder.rs` — parsing + transmission resolution
- `sim/L0/core/src/mujoco_pipeline.rs` — force application in constraint solve
