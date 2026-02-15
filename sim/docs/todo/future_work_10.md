# Future Work 10 — Phase 3A: Constraint/Joint Features + Physics + Pipeline (Items #38–42)

Part of [Simulation Phase 3 Roadmap](./index.md). See [index.md](./index.md) for
priority table, dependency graph, and file map.

All items are prerequisites to #45 (MuJoCo Conformance Test Suite). This file
covers constraint/joint features (ball/free limits, tendon wrapping), physics
(fluid forces), pipeline flags, and deformable body MJCF parsing.

---

### 38. Ball / Free Joint Limits (Swing-Twist Cones)
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

---

### 39. `wrap_inside` Algorithm (Tendon Wrapping)
**Status:** Not started | **Effort:** M | **Prerequisites:** None

#### Current State

When a sidesite is inside a wrapping geometry, the code **panics** at
`mujoco_pipeline.rs:3848` with "The wrap_inside algorithm is not implemented."
This is a runtime crash on valid models.

#### Objective

Implement the `wrap_inside` resolution algorithm so that tendon wrapping handles
the sidesite-inside-geometry case gracefully instead of panicking.

#### Specification

1. **Detection**: The current code already detects the inside case (distance from
   sidesite to wrapping geometry center < radius). The `todo!()` panic needs to
   be replaced with the resolution algorithm.
2. **Algorithm (sphere wrapping)**: When a sidesite is inside the wrapping sphere:
   - Project the sidesite position onto the sphere surface along the line from
     center to sidesite.
   - Use the projected point as the effective sidesite for wrapping calculations.
   - If sidesite is exactly at center (degenerate), use the previous frame's
     wrap point or a default direction.
3. **Algorithm (cylinder wrapping)**: When a sidesite is inside the wrapping
   cylinder:
   - Project radially outward to the cylinder surface.
   - Maintain the axial component.
4. **MuJoCo reference**: MuJoCo's `mju_wrap` handles the inside case in
   `engine_core_smooth.c`. The key behavior: when inside, the tendon path goes
   through the wrapping geometry center (zero-length wrap segment) rather than
   crashing.
5. **No panic**: Replace `todo!()` with the computed wrap-inside path. Never
   panic on valid MJCF input.

#### Acceptance Criteria

1. A model with sidesite inside wrapping sphere does not panic.
2. A model with sidesite inside wrapping cylinder does not panic.
3. Tendon length is continuous as sidesite transitions from outside to inside
   the wrapping geometry.
4. Existing wrapping tests (18 spatial tendon tests) pass unchanged.
5. New test: sidesite deliberately placed inside wrap sphere, verify tendon
   length matches MuJoCo.

#### Files

- `sim/L0/core/src/mujoco_pipeline.rs` — replace `todo!()` at the wrap_inside
  call site with proper algorithm

---

### 40. Fluid / Aerodynamic Forces
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

### 41. `disableflags` — Runtime Disable Flag Effects
**Status:** Not started | **Effort:** M | **Prerequisites:** None

#### Current State

MJCF parser parses all 21 `<flag>` attributes. But only `DISABLE_ISLAND` has a
runtime constant and check in the pipeline. Flags like `contact="disable"`,
`gravity="disable"`, `limit="disable"`, `equality="disable"` are parsed but have
NO runtime effect.

#### Objective

Wire parsed disable flags into the pipeline so each flag conditionally skips its
corresponding stage.

#### Specification

1. **Flag constants**: Define constants for each disable flag (some may already
   exist as parsed values — wire them to runtime checks).
2. **Pipeline gates**: Add flag checks at each relevant pipeline stage:
   - `DISABLE_GRAVITY`: skip gravity contribution in `mj_rne`.
   - `DISABLE_CONTACT`: skip collision detection and contact constraint assembly.
   - `DISABLE_LIMIT`: skip joint/tendon limit enforcement.
   - `DISABLE_EQUALITY`: skip equality constraint assembly.
   - `DISABLE_FRICTIONLOSS`: skip friction loss passive force.
   - `DISABLE_PASSIVE`: skip all passive forces.
   - `DISABLE_ACTUATION`: skip actuator force computation.
   - `DISABLE_SENSOR`: skip sensor evaluation.
   - `DISABLE_REFSAFE`: skip reference safety clamping.
   - (remaining flags as documented in MuJoCo API reference)
3. **Enable flags**: Similarly wire enable flags (`ENABLE_ENERGY`, etc.) —
   `ENABLE_ENERGY` is already wired. Check others.

#### Acceptance Criteria

1. `<flag contact="disable"/>` prevents contact detection.
2. `<flag gravity="disable"/>` zeros gravitational contribution to bias forces.
3. `<flag limit="disable"/>` prevents joint limit enforcement.
4. All 21 parsed flags have runtime effect.
5. Default flag values match MuJoCo defaults (all disabled flags off, all
   enabled flags per MuJoCo spec).

#### Files

- `sim/L0/core/src/mujoco_pipeline.rs` — flag checks at each pipeline stage

---

### 42. `<flex>` / `<flexcomp>` MJCF Deformable Body Parsing
**Status:** Subsumed by §6B | **Effort:** L | **Prerequisites:** None

> **Note:** This item has been subsumed by [§6B Flex Solver Unification](./future_work_6b_precursor_to_7.md),
> which implements full `<flex>`/`<flexcomp>` parsing along with the unified flex
> constraint pipeline. See `future_work_6b_precursor_to_7.md` §P9 for the
> implemented parsing specification.

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
