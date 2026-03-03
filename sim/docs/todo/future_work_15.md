# Future Work 15 — Phase 3D: Edge-Case Features (Items #60–64a)

Part of [Simulation Phase 3 Roadmap](./index.md). See [index.md](./index.md) for
priority table, dependency graph, and file map.

Lower-priority features that affect uncommon models or edge-case physics. Needed
for completeness but not blocking typical RL or robotics workflows.

---

### ~~60. `springinertia` — Joint Inertia-Spring Coupling~~
**Status:** DROPPED | **Effort:** — | **Prerequisites:** —

> **DROPPED (Phase 7 Spec B, EGT-1):** Verified nonexistent in MuJoCo.
> Zero GitHub search results for `springinertia` in the MuJoCo repository,
> no field in `mjmodel.h`, no XML attribute in MuJoCo's schema. The original
> future_work entry was based on incorrect assumptions. Conformance impact: none
> — the feature does not exist in MuJoCo C.

---

### 61. `slidercrank` Actuator Transmission
**Status:** Not started | **Effort:** M | **Prerequisites:** None

#### Current State

`ActuatorTransmission` enum has `Joint`, `Tendon`, `Site`, `Body` but no
`SliderCrank` variant. Item #36 covers adhesion (body-transmission for adhesion
actuators, now complete ✅) but not the slider-crank mechanism.

#### Objective

Implement slider-crank actuator transmission that maps actuator force through
a crank mechanism to a joint.

#### Specification

1. **MJCF parsing**: Parse `<general ... cranklength="..." slidersite="..."
   cranksite="..."/>` attributes. These define a slider-crank mechanism
   where actuator motion at the slider site drives rotation at the crank site.
2. **Transmission**: The slider-crank transmission maps linear actuator
   displacement to angular joint displacement through the crank geometry.
   The transmission ratio varies with crank angle.
3. **Force mapping**: `actuator_moment = d_length/d_angle * force`, where
   the derivative depends on the current crank geometry.
4. **MuJoCo reference**: MuJoCo computes the effective moment arm from the
   site-to-site distance derivative with respect to the joint angle.

#### Acceptance Criteria

1. A slider-crank actuator produces angle-dependent torque.
2. At crank TDC/BDC (top/bottom dead center), the transmission ratio
   approaches zero (singularity handled gracefully).
3. Models with slider-crank actuators load and simulate.

#### Files

- `sim/L0/mjcf/src/builder/` — parse crank attributes
- `sim/L0/core/src/forward/actuation.rs` — slider-crank transmission computation

---

### 62. Missing Sensor Types
**Status:** Not started | **Effort:** S per sensor | **Prerequisites:** None

#### Current State

Most sensor types are implemented. Confirmed missing:
- `clock` — simulation time (trivial)
- `jointactuatorfrc` — net actuator force on joint
- Newer MuJoCo 3.x types: `camprojection`, `geomdist`, `geompoint`, `geomnormal`

#### Objective

Implement missing sensor types.

#### Specification

1. **`clock`**: Returns `data.time`. Dimension: 1. Trivial.
2. **`jointactuatorfrc`**: Returns the net actuator force acting on a specific
   joint (sum of `qfrc_actuator` for that joint's DOFs). Dimension: 1 (hinge/slide)
   or 3 (ball).
3. **`camprojection`**: Projects a 3D point (body/site position) into camera
   image coordinates. Dimension: 2. Requires camera model parameters.
4. **`geomdist`**: Minimum distance between two geoms. Dimension: 1. Uses
   existing GJK distance query.
5. **`geompoint`**: Nearest point on a geom from a reference point. Dimension: 3.
6. **`geomnormal`**: Surface normal at nearest point. Dimension: 3.

#### Acceptance Criteria

1. `<sensor><clock/></sensor>` returns simulation time.
2. `<sensor><jointactuatorfrc joint="hip"/></sensor>` returns actuator force
   on the hip joint.
3. Models referencing missing sensor types no longer fail to parse.
4. Existing sensors unchanged (regression).

#### Files

- `sim/L0/sensor/src/` — sensor evaluation functions
- `sim/L0/mjcf/src/builder/` — parse new sensor types

---

### 63. `dynprm` Array Size (3 → 10)
**Status:** Not started | **Effort:** S | **Prerequisites:** None

#### Current State

`actuator_dynprm: Vec<[f64; 3]>`. MuJoCo uses `[f64; 10]`.

#### Objective

Expand `dynprm` to 10 elements to match MuJoCo's array size.

#### Specification

1. **Model storage**: Change `actuator_dynprm: Vec<[f64; 3]>` to
   `Vec<[f64; 10]>`.
2. **MJCF parsing**: Parse up to 10 `dynprm` values. Unspecified values
   default to 0.
3. **Runtime**: Current dynamics computation uses only `dynprm[0]` (time
   constant τ). Elements 1-9 are reserved for extended dynamics parameters
   (MuJoCo uses them for specific dyntype configurations).
4. **Backwards compatibility**: Existing models specifying 1-3 `dynprm`
   values continue to work (padded with zeros).

#### Acceptance Criteria

1. `dynprm="0.1 0 0 0 0 0 0 0 0 0"` parses without error.
2. Existing 3-element `dynprm` values continue to work.
3. No simulation behavior change for models using only `dynprm[0]`.

#### Files

- `sim/L0/core/src/types/model.rs` — array size change
- `sim/L0/mjcf/src/builder/` — parser update

---

### ~~64. Ball/Free Joint Spring Force and Energy~~
**Status:** Done | **Effort:** S | **Prerequisites:** Phase 7 Spec A (`qpos_spring` array)

> **Done (Phase 7 Spec B, commit `3f70616`).** Ball/free spring force in
> `passive.rs` via quaternion geodesic (`subquat()`), spring energy in
> `energy.rs`. Also fixed pre-existing energy bugs: DISABLE_SPRING gate,
> sleep filter, stiffness guard. 17 tests.

#### Current State

Spring force stub in `passive.rs`: ball/free cases in `mj_springdamper()` are
not implemented (hinge/slide work correctly via `qpos_spring`). Spring energy
stub in `energy.rs`: "Ball/Free joint springs would use quaternion distance —
Not commonly used, skip for now." Both `qfrc_spring` and `energy_potential`
are wrong for models with ball/free joint stiffness.

**Phase 7 Spec A dependency:** Spec A added `qpos_spring: Vec<f64>` to Model
(sized `nq`), populated from `qpos0` for ball/free joints. This provides the
quaternion/pose reference data that §64 will consume. The runtime consumers
(`passive.rs`, `energy.rs`) already read `qpos_spring` — §64 adds the
ball/free branches that use the quaternion reference.

#### Objective

Implement spring force and spring potential energy computation for ball and
free joints using quaternion distance.

#### Specification

1. **Ball joint spring force**: Compute `dif[3]` via `mji_subQuat(dif, quat,
   qpos_spring+padr)` — the 3D angular difference between current quaternion
   and spring reference. Apply `qfrc_spring[dadr+i] = -stiffness * dif[i]`.
   MuJoCo ref: `mj_springdamper()` in `engine_passive.c` (BALL case).
2. **Free joint spring force**: Translational: `F = -k * (qpos - qpos_spring)`
   for 3 position DOFs. Rotational: falls through to ball case above.
   MuJoCo ref: `mj_springdamper()` FREE case (falls through to BALL).
3. **Ball joint spring energy**: `E = 0.5 * stiffness * θ²`, where `θ` is the
   angle between the current quaternion and the spring reference quaternion.
   Compute `θ = 2 * arccos(|q_current · q_ref|)` (geodesic distance on S³).
4. **Free joint spring energy**: Translational: `0.5 * k * (x - x_ref)²`.
   Rotational: ball joint formula above.
5. **Integration**: Add force terms to `qfrc_spring` in `mj_springdamper()`.
   Add energy terms to `energy_potential` in `mj_energy_pos()`.

#### Acceptance Criteria

1. A ball joint with stiffness at its reference orientation has zero spring force and energy.
2. A ball joint rotated 90° from reference has `qfrc_spring = -k * dif` (3D) and
   energy `0.5 * k * (π/2)²`.
3. Free joint spring force includes both translational and rotational terms.
4. Free joint spring energy includes both translational and rotational terms.
5. Hinge/slide joint spring force and energy unchanged (regression).
6. `qpos_spring` from Spec A correctly provides reference data for ball/free.

#### Files

- `sim/L0/core/src/forward/passive.rs` — `mj_springdamper()` ball/free force
- `sim/L0/core/src/energy.rs` — `mj_energy_pos()` ball/free joint terms

---

### ~~64a. `jnt_margin` — Joint Limit Activation Margin~~
**Status:** Done | **Effort:** S | **Prerequisites:** None

> **Done (Phase 7 Spec B, commit `3f70616`).** `margin` parsed from `<joint>`,
> `jnt_margin: Vec<f64>` on Model, defaults cascade, 9 sites in `assembly.rs`
> replaced (3 counting + 3 assembly + 3 finalize_row! margin args). 4 tests.

#### Current State

MuJoCo activates joint limit constraints when `dist < jnt_margin[i]`, allowing
soft pre-activation before the limit surface is reached. Our code hardcodes
`margin = 0.0` for all joint limit types (hinge lower, hinge/slide upper, ball
cone). The `margin` attribute on `<joint>` is not parsed, not stored, and not
wired into the activation condition. This gap was identified and documented in
§38 (S6) as a pre-existing limitation affecting all joint types, not specific
to ball joints.

The `finalize_row!` macro already accepts a margin parameter (stored in
`efc_margin`, used for impedance computation via `compute_impedance`), but
joint limit callers always pass `0.0`.

#### MuJoCo Reference

In `engine_core_constraint.c`, `mj_instantiateLimit()`:
- Hinge/Slide: `if (dist < m->jnt_margin[i])` for both lower and upper limits
- Ball: `if (dist < m->jnt_margin[i])` for the cone limit

The `jnt_margin` value is also passed to the constraint row as the margin
parameter, affecting impedance computation (the soft transition zone width).

MuJoCo parses `margin` from `<joint margin="..."/>` (float, default `0.0`).
The attribute is also available in `<default><joint margin="..."/></default>`.

#### Objective

Parse `margin` from `<joint>`, store as `jnt_margin`, and wire it into the
joint limit activation condition and `finalize_row!` margin argument for all
three joint limit types.

#### Specification

1. **MJCF parsing**: Parse `margin` (float, default 0.0) from `<joint>` in
   `parse_joint_attrs()`. Also parse from `<default><joint>` in
   `parse_joint_defaults()`.
2. **Default resolution**: Apply class defaults in the standard cascade
   (joint-level overrides default-level).
3. **Model storage**: Add `jnt_margin: Vec<f64>` to `Model`. Populate in
   `builder/` during joint compilation.
4. **Constraint counting**: Replace `< 0.0` with `< model.jnt_margin[jnt_id]`
   in all 3 joint limit activation checks in the counting loop:
   - Hinge/Slide lower: `q - limit_min < margin`
   - Hinge/Slide upper: `limit_max - q < margin`
   - Ball cone: `limit - angle < margin`
5. **Constraint assembly**: Replace `< 0.0` with `< model.jnt_margin[jnt_id]`
   in all 3 assembly activation checks (must match counting exactly).
6. **`finalize_row!` margin argument**: Pass `model.jnt_margin[jnt_id]` instead
   of `0.0` as the margin argument to `finalize_row!` for all joint limit rows.
   This flows into `efc_margin` and `compute_impedance`, giving the correct
   soft transition zone.

#### Acceptance Criteria

1. `margin="0"` (default) produces identical behavior to current code
   (regression anchor: T22 from §38 must still pass).
2. `margin="0.01"` on a hinge joint activates the limit constraint 0.01 rad
   before the joint reaches the limit surface. Verify `nefc` increases when the
   joint is within the margin zone but not yet at the limit.
3. `margin="0.05"` on a ball joint activates the cone limit 0.05 rad early.
4. `efc_margin` for joint limit rows contains the actual margin value (not 0).
5. Impedance transition zone is correctly widened by margin (the soft region
   starts earlier).
6. `<default><joint margin="0.02"/></default>` applies to all joints without
   explicit margin.
7. Explicit `margin` on a joint overrides the default class value.

#### Files

- `sim/L0/mjcf/src/parser.rs` — `parse_joint_attrs()`, `parse_joint_defaults()`
- `sim/L0/mjcf/src/builder/` — `jnt_margin` storage + population
- `sim/L0/core/src/constraint/assembly.rs` — `assemble_unified_constraints()`:
  6 activation checks (3 counting + 3 assembly) + 3 `finalize_row!` margin args

---

### DT-120. `MjObjectType::Camera` — Frame Sensor Camera Support
**Status:** Not started | **Effort:** S | **Prerequisites:** Camera FK populating
`cam_xpos`/`cam_xmat`/`cam_quat` in `Data`
**Origin:** Phase 6 Spec A, DECISION 2

#### Current State

`MjObjectType::Camera` is not in the enum. `builder/sensor.rs` emits a warning
and falls back to the name heuristic when `objtype="camera"` is encountered.
`Data` does not have `cam_xpos`, `cam_xmat`, or `cam_quat` fields.

#### Objective

Add `Camera` variant to `MjObjectType`. Populate `cam_xpos`/`cam_xmat`/`cam_quat`
in `Data` during forward kinematics. Add Camera dispatch arms to all frame sensor
evaluation sites (position, velocity, acceleration).

#### Conformance Impact

Minor — camera-attached frame sensors are uncommon in RL/robotics models.

---

### DT-118. `mj_contactForce()` — Touch Sensor Force Reconstruction
**Status:** Not started | **Effort:** M | **Prerequisites:** None
**Origin:** Phase 6 Spec A, DECISION 3

#### Current State

Touch sensor reads `efc_force` directly (`acceleration.rs:183–193`). For
frictionless and elliptic contacts, this produces the correct normal force.
For pyramidal contacts (`dim=3`), summing facet projections produces ~75% of
the true normal force (MuJoCo: `mj_contactForce()` reconstructs the physical
force from the pyramidal basis, yielding ~33% higher values).

#### Objective

Implement `mj_contactForce()` equivalent that reconstructs physical normal force
from the constraint basis for all contact types (pyramidal, elliptic, frictionless).
Update the touch sensor to call this function instead of reading `efc_force`
directly.

#### Conformance Impact

Moderate — pyramidal is MuJoCo's default contact model. Touch sensor values are
~25% too low for all models using pyramidal contacts.

---

### DT-119. Ray-Geom Intersection Filter for Touch Sensor
**Status:** Not started | **Effort:** M | **Prerequisites:** DT-118
**Origin:** Phase 6 Spec A, DECISION 4

#### Current State

Touch sensor sums all contacts on the body without spatial filtering. MuJoCo
uses `mju_rayGeom()` to filter contacts outside the sensor site's geometric
volume. Without this filter, touch sensors over-report for small sensor sites
on large bodies (all contacts on the body contribute regardless of spatial
proximity to the site).

#### Objective

Implement MuJoCo-equivalent ray-geom intersection filter. For each contributing
contact, construct a ray from the contact point along the contact normal direction
(scaled by force, normalized, sign-flipped for `conbody[1]`). Only sum the
contact's force if the ray intersects the sensor site's geometric volume via
`mju_rayGeom()`.

#### Conformance Impact

Low to moderate — for typical models where the sensor site covers the full contact
surface, the difference is zero. Matters for precise tactile sensing with small
sensor sites on multi-geom bodies.

---

### DT-121. `InsideSite` Sensor (`mjSENS_INSIDESITE`)
**Status:** Not started | **Effort:** M | **Prerequisites:** None
**Origin:** Phase 6 Spec C, Scope Adjustment (R25/R74)

#### Current State

MuJoCo 3.5.0 has `mjSENS_INSIDESITE` — a sensor that tests whether a
specified point (site or geom) lies inside a reference site's geometric
volume. CortenForge does not implement this sensor type. It was not listed
in the umbrella spec and was discovered during Spec C rubric stress-testing
(R25).

#### Objective

Add `InsideSite` variant to `MjSensorType` and `MjcfSensorType`. Implement
geometric containment testing (point-in-geom for sphere, box, capsule,
cylinder, ellipsoid). Wire into position-stage evaluation.

#### Conformance Impact

Minor — `InsideSite` is uncommon in RL/robotics models. Primarily used for
spatial trigger detection.

---

### DT-122. Mesh/Hfield/SDF Geom Distance Support
**Status:** Not started | **Effort:** L | **Prerequisites:** None
**Origin:** Phase 6 Spec C, S3 (geom_distance algorithm)

#### Current State

`geom_distance()` (implemented in Spec C) supports convex primitives only
(Sphere, Box, Capsule, Cylinder, Ellipsoid). Non-convex geom types (Plane,
Mesh, Hfield, SDF) return `(cutoff, [0; 6])` with a warning. MuJoCo's
`mj_geomDistance()` supports all geom types via its collision backend.

#### Objective

Extend `geom_distance()` to support:
- Plane-geom distance (analytic for plane vs convex primitive)
- Mesh-geom distance (requires convex hull or GJK on mesh triangles)
- Hfield-geom distance (requires heightfield sampling)
- SDF-geom distance (requires signed distance field evaluation)

#### Conformance Impact

Gap for non-convex geometry distance queries. Acceptable for v1.0 since
common RL/robotics models use convex primitives. Becomes relevant for
environments with terrain (hfield) or complex objects (mesh).

---

### DT-125. `mj_setConst()` Runtime `qpos_spring` Recomputation
**Status:** Not started | **Effort:** M | **Prerequisites:** None
**Origin:** Phase 7 Spec B, Out of Scope bullet 4

#### Current State

`qpos_spring: Vec<f64>` is populated at build time from `qpos0`/`springref`
(Phase 7 Spec A). MuJoCo's `setSpring()` in `engine_setconst.c` recomputes
`qpos_spring` whenever `mj_setConst()` is called — this allows runtime updates
to spring reference poses (e.g., after modifying `qpos0`). CortenForge has no
`mj_setConst()` equivalent, so `qpos_spring` is static after model build.

#### Objective

Implement `mj_setConst()` (or equivalent) that recomputes `qpos_spring` and
other compile-time-derived fields at runtime. The `setSpring()` sub-routine
must run FK at `qpos0` to get body poses, then compute quaternion `qpos_spring`
for ball/free joints from the FK result.

#### Conformance Impact

Minor — most models don't call `mj_setConst()` at runtime. Matters for models
that programmatically modify `qpos0` or spring references during simulation.
