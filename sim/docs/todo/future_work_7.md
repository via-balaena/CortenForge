# Future Work 7 — Phase 3B: Constraint + Physics Completeness (Items #23–27)

Part of [Simulation Phase 3 Roadmap](./index.md). See [index.md](./index.md) for
priority table, dependency graph, and file map.

Completes the constraint system and adds remaining physics subsystems. Ordered from
most foundational (constraint parameters) to most independent (fluid forces, format).

---

### 23. Tendon Equality Constraints
**Status:** Not started | **Effort:** M | **Prerequisites:** None

#### Current State
Pipeline emits a runtime warning and silently ignores `EqualityType::Tendon`
constraints. Any model using `<equality><tendon>` gets incorrect behavior — the
constraint is simply not enforced. Standalone `TendonConstraint`/`TendonNetwork`
exists in sim-constraint but is not wired into the pipeline.

#### Objective
Implement tendon equality constraints in the pipeline constraint assembly.

#### Specification

1. **Constraint type**: `tendon1.length - tendon2.length = polycoef(tendon1.length)`
   where `polycoef` is a polynomial (up to degree 4) mapping reference tendon length
   to target offset (MuJoCo semantics).
2. **Assembly**: In `mj_fwd_constraint()`, for each `EqualityType::Tendon`:
   - Compute constraint violation: `c = L1 - polycoef(L1) - L2`
   - Compute Jacobian: `J = J_tendon1 - dpolycoef/dL1 * J_tendon1 - J_tendon2`
   - Add row to equality constraint block (before contact constraints)
3. **Solver**: Equality constraints are unclamped in PGS/Newton (no projection needed).
4. **solref/solimp**: Apply per-constraint solver parameters for stabilization.

#### Acceptance Criteria
1. Two tendons coupled by `<equality><tendon>` maintain the specified length relationship.
2. `polycoef` polynomial evaluation matches MuJoCo for degree 0–4.
3. Removing the equality constraint reproduces uncoupled behavior (regression).

#### Files
- `sim/L0/core/src/mujoco_pipeline.rs` — `mj_fwd_constraint()`, equality block assembly

---

### 24. `solreffriction` Per-Direction Solver Parameters
**Status:** Not started | **Effort:** M | **Prerequisites:** None

#### Current State
`Contact` has a single `solref` field applied to all constraint rows. MuJoCo supports
`solreffriction` — separate solver parameters for friction directions (tangent,
torsional, rolling). Comment at `mujoco_pipeline.rs:5662`: "solreffriction is NOT
applied here."

#### Objective
Apply per-direction solver reference parameters to friction constraint rows.

#### Specification

1. **MJCF parsing**: Parse `solreffriction` from `<geom>`, `<pair>`, and `<default>`
   elements. Store on `Contact` as `solreffriction: [f64; 2]`.
2. **Constraint assembly**: When building the RHS vector `b`:
   - Normal row: use `solref` (existing behavior)
   - Friction rows (tangent, torsional, rolling): use `solreffriction` if specified,
     otherwise fall back to `solref`
3. **Default**: `solreffriction = [0, 0]` means "use solref" (MuJoCo convention —
   zero values indicate "inherit from solref").

#### Acceptance Criteria
1. `solreffriction="0.05 1"` produces different friction damping than `solref`.
2. `solreffriction="0 0"` matches current behavior (regression).
3. Per-direction stabilization is numerically stable.

#### Files
- `sim/L0/mjcf/src/model_builder.rs` — parse `solreffriction`
- `sim/L0/core/src/mujoco_pipeline.rs` — constraint assembly RHS

---

### 25. Fluid / Aerodynamic Forces
**Status:** Not started | **Effort:** L | **Prerequisites:** None

#### Current State
`density`, `viscosity`, and `wind` fields are parsed from MJCF `<option>` and stored
on `Model`, but no fluid drag or lift computation exists in the pipeline. Any model
that relies on these (swimming, flying, falling with drag) produces incorrect results.

MuJoCo computes viscous and inertial drag in `mj_passive()` using ellipsoid
approximations of geom shapes. The force model has two components:
- **Viscous drag**: `F = -6π μ r v` (Stokes drag, scaled by equivalent sphere radius)
- **Inertial drag**: `F = -½ ρ C_d A |v| v` (form drag with ellipsoid cross-section)

#### Objective
Implement MuJoCo-compatible fluid forces in the passive force computation.

#### Specification

1. **Equivalent ellipsoid**: For each geom, compute equivalent ellipsoid semi-axes
   based on geom type and size (MuJoCo uses `mju_geom2Ellipsoid()`).
2. **Viscous drag** (per-geom): `F_visc = -6π * viscosity * equiv_radius * vel_fluid`
   where `vel_fluid = vel_body - wind`.
3. **Inertial drag** (per-geom): `F_inertia = -½ * density * C_d * A_proj * |vel| * vel`
   with projected area `A_proj` from ellipsoid cross-section normal to velocity.
4. **Torque**: Both components produce torques via `r × F` from geom center to body COM.
5. **Application**: Add forces in `mj_passive()` before constraint solving.

#### Acceptance Criteria
1. A falling sphere in a viscous medium reaches terminal velocity.
2. Wind produces lateral force on a stationary body.
3. Zero density/viscosity/wind produces zero fluid force (regression).
4. Force magnitudes match MuJoCo within 1% for a reference test case.

#### Files
- `sim/L0/core/src/mujoco_pipeline.rs` — `mj_passive()` or new `mj_fluid()` helper

---

### 26. `<flex>` / `<flexcomp>` MJCF Deformable Body Parsing
**Status:** Not started | **Effort:** L | **Prerequisites:** None

#### Current State
The MJCF parser skips `<flex>` and `<flexcomp>` elements inside `<deformable>` — only
`<skin>` is parsed. MuJoCo's `<flex>` is the native MJCF way to define deformable
bodies (soft bodies, cloth, ropes), and `<flexcomp>` is its procedural counterpart
(similar to `<composite>` but for flex bodies).

CortenForge has a functional deformable pipeline (`XpbdSolver`, `DeformableBody`,
`mj_deformable_collision()`, `mj_deformable_step()`) wired into the main pipeline,
but it's only accessible via the programmatic API (`register_deformable()`). Models
that define deformable bodies through MJCF `<flex>` elements silently lose them.

#### Objective
Parse `<flex>` and `<flexcomp>` MJCF elements and wire them into the existing
deformable pipeline.

#### Specification

1. **`<flex>` parsing**: Parse `<flex>` elements from `<deformable>`:
   - `name`, `dim` (1=cable, 2=shell, 3=solid)
   - `<vertex>` — vertex positions
   - `<element>` — element connectivity (edges/triangles/tetrahedra)
   - `<body>` — which body each vertex belongs to
   - Material properties: `young`, `poisson`, `damping`, `thickness`
   - Collision: `selfcollide`, `radius`, `margin`
2. **`<flexcomp>` parsing**: Parse procedural flex definitions:
   - `type` (grid, box, cylinder, etc.)
   - `count`, `spacing` — procedural dimensions
   - Expand to equivalent `<flex>` before model building
3. **Wiring**: Convert parsed flex data to `DeformableBody` instances and register
   them with the pipeline's `deformable_solvers` during model construction.
4. **`<equality><flex>`**: Parse flex equality constraints (flex-flex coupling).

#### Acceptance Criteria
1. A `<flex dim="2">` cloth element loads and simulates correctly.
2. `<flexcomp type="grid">` generates the expected deformable mesh.
3. Programmatic `register_deformable()` path unchanged (regression).

#### Files
- `sim/L0/mjcf/src/parser.rs` — parse `<flex>`, `<flexcomp>` elements
- `sim/L0/mjcf/src/model_builder.rs` — convert to `DeformableBody`, register with pipeline

---

### 27. Ball / Free Joint Limits (Swing-Twist Cones)
**Status:** Not started | **Effort:** M | **Prerequisites:** None

#### Current State
Limit enforcement in `mj_fwd_constraint()` silently skips ball and free joints with
`_ => {}`. MuJoCo supports cone limits (swing-twist) for ball joints — the joint's
rotation is decomposed into swing (away from reference axis) and twist (around
reference axis), each independently limited. Models with `limited="true"` on ball
joints get no limit enforcement.

#### Objective
Implement swing-twist cone limits for ball joints and rotation limits for free joints.

#### Specification

1. **Ball joint limits**: MuJoCo parameterizes ball joint limits as:
   - `range[0]`: Maximum swing angle (rotation away from reference axis) in radians
   - `range[1]`: Maximum twist angle (rotation around reference axis) in radians
   - Constraint is `swing ≤ range[0]` and `|twist| ≤ range[1]`
2. **Swing-twist decomposition**: Given quaternion `q` representing joint rotation:
   - `twist = atan2(2*(q.w*q.z + q.x*q.y), q.w² + q.x² - q.y² - q.z²)`
   - `swing = acos(clamp(2*(q.w² + q.z²) - 1, -1, 1))`
   (or equivalent formulation matching MuJoCo's `mju_ball2Limit()`)
3. **Constraint assembly**: When limit is violated, add constraint row(s) to the
   equality/limit block with appropriate Jacobian (derivative of swing/twist w.r.t.
   angular velocity).
4. **Free joint limits**: Apply same logic to the quaternion DOFs of free joints
   (indices 3-6 of the 7-DOF free joint).

#### Acceptance Criteria
1. A ball joint with `range="30 45"` (degrees, if `<compiler angle="degree"/>`)
   limits swing to 30° and twist to 45°.
2. Limit forces push the joint back within the cone.
3. Unlimited ball joints (`limited="false"`) unchanged (regression).
4. Swing-twist decomposition matches MuJoCo's `mju_ball2Limit()` output.

#### Files
- `sim/L0/core/src/mujoco_pipeline.rs` — `mj_fwd_constraint()`, limit enforcement block
