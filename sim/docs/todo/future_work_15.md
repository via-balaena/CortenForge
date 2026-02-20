# Future Work 15 — Phase 3D: Edge-Case Features (Items #60–64a)

Part of [Simulation Phase 3 Roadmap](./index.md). See [index.md](./index.md) for
priority table, dependency graph, and file map.

Lower-priority features that affect uncommon models or edge-case physics. Needed
for completeness but not blocking typical RL or robotics workflows.

---

### 60. `springinertia` — Joint Inertia-Spring Coupling
**Status:** Not started | **Effort:** S | **Prerequisites:** None

#### Current State

Completely absent. Not parsed, not stored, not enforced.

#### Objective

Parse `springinertia` from `<joint>` and add the inertia-spring coupling
term to the mass matrix.

#### Specification

1. **MJCF parsing**: Parse `springinertia` (float, default 0) from `<joint>`.
   Also parse from `<default>` class.
2. **Model storage**: Add to joint attributes on `Model`.
3. **Runtime effect**: In CRBA (`mj_crba`), add `springinertia` to the joint's
   diagonal mass matrix entry. This couples spring stiffness with apparent
   inertia — MuJoCo uses it to improve implicit integrator stability for
   stiff springs by making the effective inertia proportional to spring
   stiffness.
4. **Formula**: `M[dof,dof] += springinertia * stiffness[dof]`.

#### Acceptance Criteria

1. `springinertia="0"` (default) has no effect.
2. `springinertia="1"` with `stiffness="100"` adds 100 to the mass matrix
   diagonal for that DOF.
3. Improves implicit integrator stability for stiff springs.

#### Files

- `sim/L0/mjcf/src/model_builder.rs` — parse springinertia
- `sim/L0/core/src/mujoco_pipeline.rs` — CRBA diagonal modification

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

- `sim/L0/mjcf/src/model_builder.rs` — parse crank attributes
- `sim/L0/core/src/mujoco_pipeline.rs` — slider-crank transmission computation

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
- `sim/L0/mjcf/src/model_builder.rs` — parse new sensor types

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

- `sim/L0/core/src/mujoco_pipeline.rs` — array size change
- `sim/L0/mjcf/src/model_builder.rs` — parser update

---

### 64. Ball/Free Joint Spring Energy
**Status:** Not started | **Effort:** S | **Prerequisites:** None

#### Current State

Stub at `mujoco_pipeline.rs:7454`: "Ball/Free joint springs would use quaternion
distance — Not commonly used, skip for now." `energy_potential` is wrong for
models with ball/free joint stiffness.

#### Objective

Implement spring potential energy computation for ball and free joints using
quaternion distance.

#### Specification

1. **Ball joint spring energy**: The spring potential energy for a ball joint
   is `0.5 * stiffness * θ²`, where `θ` is the angle between the current
   quaternion and the spring reference quaternion. Compute `θ` using
   `θ = 2 * arccos(|q_current · q_ref|)` (the geodesic distance on S³).
2. **Free joint spring energy**: Free joints have 3 translational + 3
   rotational DOFs. Translational spring energy uses the standard
   `0.5 * k * (x - x_ref)²`. Rotational spring energy uses the ball joint
   formula above.
3. **Integration**: Add these terms to `energy_potential` in `mj_energy_pos()`.

#### Acceptance Criteria

1. A ball joint with stiffness at its reference orientation has zero spring energy.
2. A ball joint rotated 90° from reference has `0.5 * k * (π/2)²` spring energy.
3. Free joint spring energy includes both translational and rotational terms.
4. Hinge/slide joint spring energy unchanged (regression).

#### Files

- `sim/L0/core/src/mujoco_pipeline.rs` — `mj_energy_pos()` ball/free joint terms

---

### 64a. `jnt_margin` — Joint Limit Activation Margin
**Status:** Not started | **Effort:** S | **Prerequisites:** None

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
   `model_builder.rs` during joint compilation.
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
- `sim/L0/mjcf/src/model_builder.rs` — `jnt_margin` storage + population
- `sim/L0/core/src/mujoco_pipeline.rs` — `assemble_unified_constraints()`:
  6 activation checks (3 counting + 3 assembly) + 3 `finalize_row!` margin args
