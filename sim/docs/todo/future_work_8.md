# Future Work 8 — Phase 3C: Format Completeness + Edge-Case Physics (Items #28–32)

Part of [Simulation Phase 3 Roadmap](./index.md). See [index.md](./index.md) for
priority table, dependency graph, and file map.

Closes remaining model format gaps and edge-case physics features. Lower priority
than Phases 3A/3B but needed for full MuJoCo parity.

---

### 28. `<composite>` Element (Procedural Body Generation)
**Status:** Not started | **Effort:** XL | **Prerequisites:** #18 (`<include>` + `<compiler>`)

#### Current State
The MJCF parser does not handle the `<composite>` element. This is a macro-expansion
feature in MuJoCo that procedurally generates body/geom/joint/tendon trees from
templates (grid, rope, cable, cloth, box, cylinder, ellipsoid). It is widely used for
soft-body and deformable setups. Many composite models also use `<include>`, making
#18 a practical prerequisite.

#### Objective
Implement `<composite>` as a pre-processing expansion that generates equivalent
explicit MJCF elements before model building.

#### Specification

1. **Composite types** (MuJoCo supports 7):
   - `grid` — 2D grid of bodies (cloth, membranes)
   - `rope` / `cable` — 1D chain of bodies (ropes, cables)
   - `cloth` — 2D deformable mesh
   - `box` / `cylinder` / `ellipsoid` — 3D volumetric soft bodies
2. **Expansion**: For each `<composite>`, generate:
   - Body tree with specified count/spacing
   - Geoms (spheres or capsules) on each body
   - Joints (ball or hinge) connecting adjacent bodies
   - Tendons (if `tendon` sub-element specified)
   - Skin (if `skin` sub-element specified)
3. **Pre-processing**: Expand composites in the XML DOM before `model_builder`
   processes the tree. The builder never sees `<composite>` — only expanded elements.

#### Acceptance Criteria
1. `<composite type="rope" count="10"/>` generates a 10-body chain.
2. `<composite type="grid" count="5 5"/>` generates a 5×5 body grid.
3. Generated models match MuJoCo's composite expansion (body count, joint types,
   connectivity).
4. Models without `<composite>` are unaffected (regression).

#### Files
- `sim/L0/mjcf/src/composite.rs` — new module for composite expansion
- `sim/L0/mjcf/src/lib.rs` — pre-processing hook before model building

---

### 29. URDF Loader Completeness
**Status:** Not started | **Effort:** M | **Prerequisites:** None

#### Current State
URDF loading works for basic models but has known gaps:
- **Mesh geometry**: Parsed but not converted to collision shapes — mesh geoms are
  silently dropped.
- **Friction**: `<dynamics friction="...">` parsed but not applied to contacts.
- **Planar joint**: Approximated as a single hinge, losing 2 of 3 DOF.
- **`<mimic>` joints**: Not supported (coupled joint motion).
- **Kinematic loops**: Not supported (tree structures only).

#### Objective
Close the URDF→Model conversion gaps so standard ROS URDF models load faithfully.

#### Specification

1. **Mesh geometry**: Convert URDF mesh references (STL/OBJ/DAE file paths) to
   `GeomType::Mesh` with loaded vertex data. Reuse existing mesh loading
   infrastructure from MJCF path.
2. **Friction**: Map URDF `<dynamics friction="...">` to `geom.friction[0]`
   (sliding friction coefficient). Apply to generated contacts.
3. **Planar joint**: Map to 2 slide joints + 1 hinge (3 DOF) matching ROS
   convention: translation in XY plane + rotation about Z.
4. **`<mimic>` joints**: Parse `<mimic joint="..." multiplier="..." offset="..."/>`
   and generate `<equality><joint>` coupling constraints.

#### Acceptance Criteria
1. A URDF with mesh collision geoms generates contacts.
2. URDF friction values affect contact solver behavior.
3. Planar joint provides 3 DOF movement.
4. Simple URDF models (no meshes, no planar) unchanged (regression).

#### Files
- `sim/L0/urdf/src/` — URDF→Model conversion

---

### 30. Pyramidal Friction Cones
⚠️ **Migrated to [future_work_6_precursor_1.md](./future_work_6_precursor_1.md) #30.**
Full specification is now tracked there as a prerequisite to #19.

---

### 31. CCD (Continuous Collision Detection)
**Status:** Parsed only | **Effort:** L | **Prerequisites:** None

#### Current State
`ccd_iterations` is parsed from `<option>` and stored on `MjcfOption`.
`nativeccd` and `multiccd` enable flags are stored in `MjcfFlag`. But no CCD
implementation exists — these fields have no runtime effect.
Contacts are detected via discrete overlap testing only, which can miss fast-moving
thin objects (tunneling).

#### Objective
Implement conservative-advancement CCD for convex geom pairs.

#### Specification

1. **Algorithm**: Conservative advancement — iteratively advance the time-of-impact
   estimate using upper-bound velocity and minimum separation distance.
2. **Scope**: Convex-convex pairs only (sphere, capsule, box, ellipsoid, convex mesh).
   Non-convex pairs (mesh-mesh, hfield) use discrete detection.
3. **Integration**: After broad-phase, for pairs with relative velocity exceeding a
   threshold, run CCD to find earliest time-of-impact. Generate contact at TOI
   configuration.
4. **Parameters**: `ccd_iterations` (max bisection steps), `ccd_tolerance` (distance
   threshold for contact).

#### Acceptance Criteria
1. A fast-moving sphere does not tunnel through a thin wall.
2. `ccd_iterations=0` disables CCD (current behavior, regression).
3. CCD contacts produce forces consistent with discrete contacts at TOI.

#### Files
- `sim/L0/core/src/mujoco_pipeline.rs` — post-broadphase CCD pass
- `sim/L0/core/src/gjk_epa.rs` — GJK distance query for conservative advancement

---

### 32. Non-Physics MJCF Elements
**Status:** Not started | **Effort:** S | **Prerequisites:** None

#### Current State
Several MJCF elements are not parsed. None affect physics simulation, but their
absence causes warnings or limits interoperability:
- `<visual>` — rendering parameters (L1/sim-bevy concern)
- `<statistic>` — auto-computed model statistics (center, extent, meansize)
- `<custom>` — user-defined numeric/text data
- `<size>` — memory size hints (may not apply to Rust allocation model)

#### Objective
Parse and store these elements to suppress warnings and improve MJCF round-trip
fidelity. No simulation effect.

#### Specification

1. **`<statistic>`**: Parse `center`, `extent`, `meaninertia`, `meanmass`,
   `meansize`. Store on `Model`. Auto-compute from model geometry if not specified
   (MuJoCo behavior).
2. **`<custom>`**: Parse `<numeric>` and `<text>` children. Store in
   `Model.custom_numeric: HashMap<String, Vec<f64>>` and
   `Model.custom_text: HashMap<String, String>`.
3. **`<size>`**: Parse and store. No runtime effect (Rust manages memory
   dynamically).
4. **`<visual>`**: Parse top-level visual parameters (global, quality, headlight,
   map, scale, rgba). Store on `Model` for L1 consumption. Do not implement
   rendering (that's sim-bevy's job).

#### Acceptance Criteria
1. Models with these elements load without warnings.
2. Stored values accessible via `Model` API.
3. No simulation behavior change (regression).

#### Files
- `sim/L0/mjcf/src/model_builder.rs` — parse + store
- `sim/L0/core/src/mujoco_pipeline.rs` — `Model` struct fields (if needed)
