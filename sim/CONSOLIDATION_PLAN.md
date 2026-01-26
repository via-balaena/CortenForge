# MuJoCo Alignment Consolidation Plan

> **Goal**: Align our physics engine strictly with MuJoCo's architecture. Demolish anything that doesn't fit.

## Executive Summary

We have two parallel architectures that don't talk to each other:

1. **Old architecture**: `World` + `Stepper` + separate solvers
2. **New MuJoCo architecture**: `ArticulatedSystem` + `PGSSolver` (in `mujoco_pipeline.rs`)

MuJoCo uses **generalized coordinates** (`qpos`/`qvel`) as the single source of truth. Body poses are *computed* from joint positions via forward kinematics. Our old `World` with independent body states is fundamentally incompatible.

**The plan**: Kill `World`, expand `mujoco_pipeline.rs` into a proper `Model`/`Data` architecture, and rewire everything to flow through generalized coordinates.

---

## What We Have (Current State)

### Completed Work (Phases 0-3.1 of old plan)

| Feature | Status | Notes |
|---------|--------|-------|
| sim-types | ✅ | Core data types |
| sim-simd | ✅ | SIMD batch operations |
| sim-core collision | ✅ | GJK/EPA, BVH, broad-phase, ray-casting |
| sim-contact | ✅ | Compliant contact model, friction cones |
| sim-constraint | ✅ | Joint types, multiple solvers (but wrong architecture) |
| sim-sensor | ✅ | IMU, F/T, touch, rangefinder, magnetometer |
| sim-muscle | ✅ | Hill-type muscles with activation dynamics |
| sim-tendon | ✅ | Fixed + spatial tendons with wrapping |
| sim-mjcf | ✅ | Parser with tendons, equality constraints, muscles |
| sim-urdf | ✅ | URDF parser |
| sim-deformable | ✅ | XPBD soft bodies (optional feature) |
| sim-bevy viz | ✅ | Muscle/tendon/sensor visualization |
| ArticulatedSystem | ✅ | CRBA/RNE for n-DOF chains |
| PGSSolver | ✅ | Warm-starting constraint solver |
| Pendulum examples | ✅ | Simple, double, n-link, spherical, constrained |

### The Problem

```
Current Flow (BROKEN):
  MJCF → World (separate body states) → Stepper → ???
                    ↓
          Bodies have independent poses
          Joints are constraints between bodies
          No unified qpos/qvel

MuJoCo Flow (CORRECT):
  MJCF → Model (static) → Data (qpos/qvel) → mj_step → FK → body poses
                              ↓
                    Single source of truth
                    Bodies computed from joints
```

---

## MuJoCo Architecture Reference

### Core Data Structures

```c
// MuJoCo's mjModel (STATIC - parsed from XML)
struct mjModel {
    int nq;           // number of generalized coordinates
    int nv;           // number of degrees of freedom
    int nbody;        // number of bodies
    int njnt;         // number of joints
    int ngeom;        // number of geoms

    // Tree structure
    int* body_parentid;    // parent body ID
    int* body_jntnum;      // number of joints for this body
    int* body_jntadr;      // start index in jnt arrays
    int* jnt_type;         // joint type (hinge, slide, ball, free)
    int* jnt_qposadr;      // start index in qpos
    int* jnt_dofadr;       // start index in qvel

    // Body properties
    mjtNum* body_mass;     // mass
    mjtNum* body_inertia;  // diagonal inertia in body frame
    mjtNum* body_ipos;     // COM position in body frame
    mjtNum* body_iquat;    // body frame orientation

    // Joint properties
    mjtNum* jnt_axis;      // joint axis (local)
    mjtNum* jnt_pos;       // joint anchor (local)
    mjtNum* jnt_range;     // joint limits
    mjtNum* jnt_stiffness; // spring stiffness
    mjtNum* jnt_damping;   // damping coefficient
};

// MuJoCo's mjData (DYNAMIC - changes each step)
struct mjData {
    // Generalized coordinates (THE source of truth)
    mjtNum* qpos;          // joint positions [nq]
    mjtNum* qvel;          // joint velocities [nv]
    mjtNum* qacc;          // joint accelerations [nv]

    // Computed from FK (READ-ONLY outputs)
    mjtNum* xpos;          // body positions [nbody x 3]
    mjtNum* xquat;         // body orientations [nbody x 4]
    mjtNum* xmat;          // body rotation matrices [nbody x 9]

    // Forces
    mjtNum* qfrc_applied;  // applied generalized forces [nv]
    mjtNum* qfrc_bias;     // Coriolis + gravity [nv]
    mjtNum* qfrc_constraint; // constraint forces [nv]

    // Inertia (computed by CRBA)
    mjtNum* qM;            // inertia matrix (sparse or dense)

    // Contacts
    mjContact* contact;    // active contacts
    int ncon;              // number of contacts
};
```

### Simulation Pipeline (`mj_step`)

```
mj_step(model, data):
    mj_checkPos(m, d)              // Validate positions (reset if NaN/Inf)
    mj_checkVel(m, d)              // Validate velocities
    mj_forward(m, d)               // All forward dynamics
    mj_checkAcc(m, d)              // Validate accelerations
    mj_Euler(m, d)                 // Integration (or mj_RungeKutta for RK4)

mj_forward(model, data):
    // Position stage (most expensive - FK + collision)
    mj_fwdPosition(m, d)           // FK: qpos → xpos, xquat, xmat
    mj_sensorPos(m, d)             // Position-dependent sensors
    mj_energyPos(m, d)             // Potential energy

    // Velocity stage
    mj_fwdVelocity(m, d)           // Velocity FK: qvel → cvel, cdof
    mj_sensorVel(m, d)             // Velocity-dependent sensors
    mj_energyVel(m, d)             // Kinetic energy

    // Acceleration stage
    mj_fwdActuation(m, d)          // ctrl → qfrc_actuator
    mj_fwdAcceleration(m, d)       // CRBA for qM, RNE for qfrc_bias
    mj_fwdConstraint(m, d)         // PGS solver → qfrc_constraint, qacc
    mj_sensorAcc(m, d)             // Acceleration-dependent sensors
```

**Key insight**: Body poses (`xpos`, `xquat`) are *outputs* of forward kinematics, not independent state.

### Critical Details

**nq vs nv**: Position dimension (`nq`) can exceed velocity dimension (`nv`) because:
- Hinge/Slide: nq=1, nv=1
- Ball: nq=4 (quaternion), nv=3 (angular velocity)
- Free: nq=7 (pos + quat), nv=6 (linear + angular velocity)

**Quaternion Integration**: Cannot simply do `qpos += qvel * dt` for quaternions. MuJoCo provides `mj_integratePos()` which properly handles the SO(3) manifold. We need `mj_differentiatePos()` for the inverse operation.

**Sparse Mass Matrix**: MuJoCo stores `qM` in a sparse L^T D L factorization that preserves kinematic tree sparsity. For initial implementation, dense is acceptable but sparse is needed for humanoid-scale models.

**Warm Starting**: `qacc_warmstart` stores previous accelerations to initialize constraint solver. Newton solver converges in 2-3 iterations so benefit is minimal, but PGS benefits significantly.

**Solver Choice**: MuJoCo default is Newton (exact 2nd derivatives, Cholesky factorization). PGS is available but slower to converge. For us, PGS with 10-20 iterations is sufficient for typical robotics.

---

## The Plan

### Phase 1: Define MuJoCo-Aligned Data Structures

Create `Model` and `Data` in `sim-core/src/mujoco_pipeline.rs`:

```rust
/// Static model definition (like mjModel)
/// Immutable after construction - all memory allocated upfront
pub struct Model {
    // ========== Dimensions ==========
    pub nq: usize,                 // generalized position coords (includes quaternions)
    pub nv: usize,                 // generalized velocity coords (DOFs, always <= nq)
    pub nbody: usize,              // number of bodies (including world body 0)
    pub njnt: usize,               // number of joints
    pub ngeom: usize,              // number of collision geometries
    pub nsite: usize,              // number of sites (attachment points)
    pub nu: usize,                 // number of actuators
    pub na: usize,                 // number of activation states

    // ========== Body Tree (indexed by body_id, 0 = world) ==========
    pub body_parent: Vec<usize>,   // parent body index (0 for root bodies)
    pub body_rootid: Vec<usize>,   // root body of kinematic tree
    pub body_jnt_adr: Vec<usize>,  // first joint index for this body
    pub body_jnt_num: Vec<usize>,  // number of joints for this body
    pub body_dof_adr: Vec<usize>,  // first DOF index for this body
    pub body_dof_num: Vec<usize>,  // number of DOFs for this body
    pub body_geom_adr: Vec<usize>, // first geom index for this body
    pub body_geom_num: Vec<usize>, // number of geoms for this body

    // Body properties
    pub body_pos: Vec<Vector3<f64>>,      // position in parent frame
    pub body_quat: Vec<UnitQuaternion<f64>>, // orientation in parent frame
    pub body_ipos: Vec<Vector3<f64>>,     // COM offset in body frame
    pub body_iquat: Vec<UnitQuaternion<f64>>, // inertial frame orientation
    pub body_mass: Vec<f64>,              // mass
    pub body_inertia: Vec<Vector3<f64>>,  // diagonal inertia (principal axes)
    pub body_name: Vec<Option<String>>,   // optional names for lookup

    // ========== Joints (indexed by jnt_id) ==========
    pub jnt_type: Vec<JointType>,         // FREE, BALL, SLIDE, HINGE
    pub jnt_body: Vec<usize>,             // body this joint belongs to
    pub jnt_qpos_adr: Vec<usize>,         // start index in qpos
    pub jnt_dof_adr: Vec<usize>,          // start index in qvel
    pub jnt_pos: Vec<Vector3<f64>>,       // anchor position in body frame
    pub jnt_axis: Vec<Vector3<f64>>,      // joint axis (for hinge/slide)
    pub jnt_limited: Vec<bool>,           // has joint limits?
    pub jnt_range: Vec<(f64, f64)>,       // limits [min, max]
    pub jnt_stiffness: Vec<f64>,          // spring stiffness (ref = qpos0)
    pub jnt_damping: Vec<f64>,            // damping coefficient
    pub jnt_armature: Vec<f64>,           // armature inertia (motor rotor)
    pub jnt_name: Vec<Option<String>>,

    // ========== DOFs (indexed by dof_id, one per velocity dimension) ==========
    pub dof_body: Vec<usize>,             // body for this DOF
    pub dof_jnt: Vec<usize>,              // joint for this DOF
    pub dof_parent: Vec<Option<usize>>,   // parent DOF in tree (None for root)
    pub dof_armature: Vec<f64>,           // armature inertia
    pub dof_damping: Vec<f64>,            // damping coefficient

    // ========== Geoms (indexed by geom_id) ==========
    pub geom_type: Vec<GeomType>,         // SPHERE, BOX, CAPSULE, CYLINDER, ELLIPSOID, MESH, etc.
    pub geom_body: Vec<usize>,            // parent body
    pub geom_pos: Vec<Vector3<f64>>,      // position in body frame
    pub geom_quat: Vec<UnitQuaternion<f64>>, // orientation in body frame
    pub geom_size: Vec<Vector3<f64>>,     // type-specific size parameters
    pub geom_friction: Vec<Vector3<f64>>, // [sliding, torsional, rolling]
    pub geom_contype: Vec<u32>,           // contact type bitmask
    pub geom_conaffinity: Vec<u32>,       // contact affinity bitmask
    pub geom_name: Vec<Option<String>>,

    // ========== Actuators (indexed by actuator_id) ==========
    pub actuator_trntype: Vec<ActuatorTransmission>, // JOINT, TENDON, SITE, etc.
    pub actuator_dyntype: Vec<ActuatorDynamics>,     // NONE, INTEGRATOR, FILTER, MUSCLE
    pub actuator_trnid: Vec<usize>,       // transmission target (joint/tendon/site id)
    pub actuator_gear: Vec<f64>,          // transmission gear ratio
    pub actuator_ctrlrange: Vec<(f64, f64)>, // control input limits
    pub actuator_forcerange: Vec<(f64, f64)>, // force output limits
    pub actuator_name: Vec<Option<String>>,

    // ========== Options ==========
    pub timestep: f64,
    pub gravity: Vector3<f64>,
    pub qpos0: DVector<f64>,              // default/reference position

    // Solver options
    pub solver_iterations: usize,         // max constraint solver iterations
    pub solver_tolerance: f64,            // early termination tolerance
    pub integrator: Integrator,           // EULER, RK4, IMPLICIT
}

/// Dynamic simulation state (like mjData)
/// All arrays pre-allocated - no heap allocation during simulation
pub struct Data {
    // ========== Generalized Coordinates (THE source of truth) ==========
    pub qpos: DVector<f64>,        // [nq] joint positions (includes quaternions)
    pub qvel: DVector<f64>,        // [nv] joint velocities
    pub qacc: DVector<f64>,        // [nv] joint accelerations (computed)
    pub qacc_warmstart: DVector<f64>, // [nv] warmstart for constraint solver

    // ========== Control / Actuation ==========
    pub ctrl: DVector<f64>,        // [nu] actuator control inputs
    pub act: DVector<f64>,         // [na] actuator activation states
    pub qfrc_actuator: DVector<f64>, // [nv] actuator forces in joint space

    // ========== Computed Body States (from FK - outputs, not inputs) ==========
    pub xpos: Vec<Vector3<f64>>,   // [nbody] body COM positions in world
    pub xquat: Vec<UnitQuaternion<f64>>, // [nbody] body orientations in world
    pub xmat: Vec<Matrix3<f64>>,   // [nbody] rotation matrices (cached for efficiency)
    pub xipos: Vec<Vector3<f64>>,  // [nbody] body inertial frame positions
    pub ximat: Vec<Matrix3<f64>>,  // [nbody] body inertial frame rotations

    // Geom positions (for collision detection)
    pub geom_xpos: Vec<Vector3<f64>>,  // [ngeom] geom positions in world
    pub geom_xmat: Vec<Matrix3<f64>>,  // [ngeom] geom rotations in world

    // ========== Velocities (computed from qvel) ==========
    pub cvel: Vec<SpatialVector>,  // [nbody] body velocities (angular, linear)
    pub cdof: Vec<SpatialVector>,  // [nv] DOF velocities in Cartesian space

    // ========== Forces in Generalized Coordinates ==========
    pub qfrc_applied: DVector<f64>,    // [nv] user-applied forces
    pub qfrc_bias: DVector<f64>,       // [nv] Coriolis + centrifugal + gravity
    pub qfrc_passive: DVector<f64>,    // [nv] spring + damping forces
    pub qfrc_constraint: DVector<f64>, // [nv] constraint forces (contacts + limits)

    // Cartesian forces (alternative input method)
    pub xfrc_applied: Vec<SpatialVector>, // [nbody] applied forces in world frame

    // ========== Mass Matrix ==========
    pub qM: DMatrix<f64>,          // [nv x nv] joint-space inertia (dense for now)
    // Future: sparse L^T D L factorization for efficiency

    // ========== Contacts ==========
    pub contacts: Vec<Contact>,    // active contacts (variable size, but pre-allocated capacity)
    pub ncon: usize,               // number of active contacts

    // ========== Solver State ==========
    pub solver_niter: usize,       // iterations used in last solve
    pub solver_nnz: usize,         // non-zeros in constraint Jacobian

    // ========== Energy (optional, for debugging) ==========
    pub energy_potential: f64,     // gravitational + spring potential
    pub energy_kinetic: f64,       // kinetic energy

    // ========== Time ==========
    pub time: f64,
}
```

### Phase 2: Implement Core Pipeline Functions

```rust
impl Model {
    /// Parse MJCF XML into Model
    pub fn from_mjcf(xml: &str) -> Result<Self, MjcfError>;

    /// Parse URDF into Model
    pub fn from_urdf(xml: &str) -> Result<Self, UrdfError>;

    /// Create initial Data for this model
    pub fn make_data(&self) -> Data;

    /// Get default qpos (reference configuration)
    pub fn qpos0(&self) -> DVector<f64>;
}

impl Data {
    /// Full simulation step (like mj_step)
    pub fn step(&mut self, model: &Model) {
        self.forward(model);           // FK + dynamics
        self.integrate(model);         // Euler step
    }

    /// Forward dynamics only (like mj_forward)
    pub fn forward(&mut self, model: &Model) {
        mj_fwd_position(model, self);      // FK
        mj_fwd_velocity(model, self);      // velocity FK
        mj_fwd_actuation(model, self);     // actuator forces
        mj_crba(model, self);              // mass matrix
        mj_rne(model, self);               // bias forces
        mj_fwd_constraint(model, self);    // contacts + constraints
        mj_fwd_acceleration(model, self);  // solve for qacc
    }

    /// Integration step (semi-implicit Euler, MuJoCo default)
    fn integrate(&mut self, model: &Model) {
        let h = model.timestep;

        // Update velocities first (semi-implicit)
        self.qvel += &self.qacc * h;

        // Update positions - CRITICAL: quaternions need special handling!
        mj_integrate_pos(model, self, h);

        // Normalize quaternions to prevent drift
        mj_normalize_quat(model, self);

        self.time += h;
    }
}

/// Proper position integration that handles quaternions on SO(3) manifold
fn mj_integrate_pos(model: &Model, data: &mut Data, h: f64) {
    for jnt_id in 0..model.njnt {
        let qpos_adr = model.jnt_qpos_adr[jnt_id];
        let dof_adr = model.jnt_dof_adr[jnt_id];

        match model.jnt_type[jnt_id] {
            JointType::Hinge | JointType::Slide => {
                // Simple scalar: qpos += qvel * h
                data.qpos[qpos_adr] += data.qvel[dof_adr] * h;
            }
            JointType::Ball => {
                // Quaternion: integrate angular velocity on SO(3)
                // quat_new = quat_old * exp(0.5 * h * omega)
                let omega = Vector3::new(
                    data.qvel[dof_adr],
                    data.qvel[dof_adr + 1],
                    data.qvel[dof_adr + 2],
                );
                let angle = omega.norm() * h;
                if angle > 1e-10 {
                    let axis = omega / omega.norm();
                    let dq = UnitQuaternion::from_axis_angle(&Unit::new_normalize(axis), angle);
                    let q_old = UnitQuaternion::from_quaternion(Quaternion::new(
                        data.qpos[qpos_adr],
                        data.qpos[qpos_adr + 1],
                        data.qpos[qpos_adr + 2],
                        data.qpos[qpos_adr + 3],
                    ));
                    let q_new = q_old * dq;
                    data.qpos[qpos_adr] = q_new.w;
                    data.qpos[qpos_adr + 1] = q_new.i;
                    data.qpos[qpos_adr + 2] = q_new.j;
                    data.qpos[qpos_adr + 3] = q_new.k;
                }
            }
            JointType::Free => {
                // Position: linear integration
                data.qpos[qpos_adr] += data.qvel[dof_adr] * h;
                data.qpos[qpos_adr + 1] += data.qvel[dof_adr + 1] * h;
                data.qpos[qpos_adr + 2] += data.qvel[dof_adr + 2] * h;

                // Orientation: quaternion integration (same as Ball)
                let omega = Vector3::new(
                    data.qvel[dof_adr + 3],
                    data.qvel[dof_adr + 4],
                    data.qvel[dof_adr + 5],
                );
                let angle = omega.norm() * h;
                if angle > 1e-10 {
                    let axis = omega / omega.norm();
                    let dq = UnitQuaternion::from_axis_angle(&Unit::new_normalize(axis), angle);
                    let q_old = UnitQuaternion::from_quaternion(Quaternion::new(
                        data.qpos[qpos_adr + 3],
                        data.qpos[qpos_adr + 4],
                        data.qpos[qpos_adr + 5],
                        data.qpos[qpos_adr + 6],
                    ));
                    let q_new = q_old * dq;
                    data.qpos[qpos_adr + 3] = q_new.w;
                    data.qpos[qpos_adr + 4] = q_new.i;
                    data.qpos[qpos_adr + 5] = q_new.j;
                    data.qpos[qpos_adr + 6] = q_new.k;
                }
            }
        }
    }
}

/// Normalize all quaternions in qpos to prevent numerical drift
fn mj_normalize_quat(model: &Model, data: &mut Data) {
    for jnt_id in 0..model.njnt {
        let qpos_adr = model.jnt_qpos_adr[jnt_id];
        match model.jnt_type[jnt_id] {
            JointType::Ball => {
                let norm = (data.qpos[qpos_adr].powi(2)
                    + data.qpos[qpos_adr + 1].powi(2)
                    + data.qpos[qpos_adr + 2].powi(2)
                    + data.qpos[qpos_adr + 3].powi(2))
                .sqrt();
                if norm > 1e-10 {
                    data.qpos[qpos_adr] /= norm;
                    data.qpos[qpos_adr + 1] /= norm;
                    data.qpos[qpos_adr + 2] /= norm;
                    data.qpos[qpos_adr + 3] /= norm;
                }
            }
            JointType::Free => {
                let norm = (data.qpos[qpos_adr + 3].powi(2)
                    + data.qpos[qpos_adr + 4].powi(2)
                    + data.qpos[qpos_adr + 5].powi(2)
                    + data.qpos[qpos_adr + 6].powi(2))
                .sqrt();
                if norm > 1e-10 {
                    data.qpos[qpos_adr + 3] /= norm;
                    data.qpos[qpos_adr + 4] /= norm;
                    data.qpos[qpos_adr + 5] /= norm;
                    data.qpos[qpos_adr + 6] /= norm;
                }
            }
            _ => {}
        }
    }

    /// Reset to initial state
    pub fn reset(&mut self, model: &Model) {
        self.qpos = model.qpos0();
        self.qvel = DVector::zeros(model.nv);
        self.time = 0.0;
        self.forward(model);
    }
}
```

### Phase 3: Implement Pipeline Stages

Each stage in detail:

#### 3.1 Forward Kinematics (`mj_fwd_position`)

```rust
/// Compute body positions and orientations from qpos
fn mj_fwd_position(model: &Model, data: &mut Data) {
    // Body 0 (world) is always at origin
    data.xpos[0] = Vector3::zeros();
    data.xquat[0] = UnitQuaternion::identity();
    data.xmat[0] = Matrix3::identity();

    // Process bodies in topological order (parent before child)
    for body_id in 1..model.nbody {
        let parent_id = model.body_parent[body_id];

        // Start with parent's frame
        let mut pos = data.xpos[parent_id];
        let mut quat = data.xquat[parent_id];

        // Apply each joint for this body
        let jnt_start = model.body_jnt_adr[body_id];
        let jnt_end = jnt_start + model.body_jnt_num[body_id];

        for jnt_id in jnt_start..jnt_end {
            let jnt_type = model.jnt_type[jnt_id];
            let qpos_adr = model.jnt_qpos_adr[jnt_id];

            match jnt_type {
                JointType::Hinge => {
                    let angle = data.qpos[qpos_adr];
                    let axis = model.jnt_axis[jnt_id];
                    let anchor = model.jnt_pos[jnt_id];

                    // Rotate around axis
                    let rot = UnitQuaternion::from_axis_angle(
                        &Unit::new_normalize(quat * axis),
                        angle
                    );
                    quat = rot * quat;
                    // Anchor point transforms
                    pos = pos + quat * anchor;
                }
                JointType::Slide => {
                    let displacement = data.qpos[qpos_adr];
                    let axis = model.jnt_axis[jnt_id];
                    pos = pos + quat * (axis * displacement);
                }
                JointType::Ball => {
                    // qpos stores quaternion [w, x, y, z]
                    let q = UnitQuaternion::from_quaternion(Quaternion::new(
                        data.qpos[qpos_adr],
                        data.qpos[qpos_adr + 1],
                        data.qpos[qpos_adr + 2],
                        data.qpos[qpos_adr + 3],
                    ));
                    quat = quat * q;
                }
                JointType::Free => {
                    // qpos stores [x, y, z, qw, qx, qy, qz]
                    pos = Vector3::new(
                        data.qpos[qpos_adr],
                        data.qpos[qpos_adr + 1],
                        data.qpos[qpos_adr + 2],
                    );
                    quat = UnitQuaternion::from_quaternion(Quaternion::new(
                        data.qpos[qpos_adr + 3],
                        data.qpos[qpos_adr + 4],
                        data.qpos[qpos_adr + 5],
                        data.qpos[qpos_adr + 6],
                    ));
                }
            }
        }

        // Apply body frame offset
        pos = pos + quat * model.body_ipos[body_id];
        quat = quat * model.body_iquat[body_id];

        data.xpos[body_id] = pos;
        data.xquat[body_id] = quat;
        data.xmat[body_id] = quat.to_rotation_matrix().into_inner();
    }
}
```

#### 3.2 CRBA (Composite Rigid Body Algorithm)

Already implemented in `ArticulatedSystem::inertia_matrix()`. Generalize for tree structures.

#### 3.3 RNE (Recursive Newton-Euler)

Already implemented in `ArticulatedSystem::bias_forces()`. Generalize for tree structures.

#### 3.4 Constraint Solver

Use existing `PGSSolver` with warm-starting. Add contact constraint generation.

### Phase 4: MJCF Parser → Model

Rewrite `sim-mjcf` to output `Model` directly:

```rust
// OLD API (to be deleted)
pub fn load_mjcf_into_world(xml: &str, world: &mut World) -> Result<SpawnedMjcf>;

// NEW API
pub fn load_mjcf(xml: &str) -> Result<Model, MjcfError>;

impl Model {
    pub fn from_mjcf(xml: &str) -> Result<Self, MjcfError> {
        sim_mjcf::load_mjcf(xml)
    }
}
```

The parser already extracts bodies, joints, geoms. Just need to build `Model` arrays instead of spawning into `World`.

### Phase 5: Delete Old Code

**DELETE entirely:**
- `sim-core/src/world.rs`
- `sim-core/src/stepper.rs`
- `sim-constraint/src/solver.rs` (basic Gauss-Seidel)
- `sim-constraint/src/newton.rs` (Newton-Raphson)
- `sim-constraint/src/cg.rs` (Conjugate Gradient)
- `sim-constraint/src/pgs.rs` (duplicate PGS)
- `sim-constraint/src/islands.rs` (constraint islands for old World)
- `sim-constraint/src/parallel.rs` (parallel for old World)

**KEEP but refactor:**
- `sim-constraint/src/joint.rs` - Joint type definitions (extract to Model)
- `sim-constraint/src/limits.rs` - Limit enforcement
- `sim-constraint/src/motor.rs` - Motor control
- `sim-contact/*` - Contact model (integrate into constraint solver)
- `sim-core/src/collision/*` - Collision detection (compute contacts for Data)

**KEEP as-is:**
- `sim-types/*`
- `sim-simd/*`
- `sim-sensor/*`
- `sim-muscle/*` (optional feature)
- `sim-tendon/*` (optional feature)
- `sim-deformable/*` (optional feature)

### Phase 6: Bevy Integration

```rust
// New Bevy resources
#[derive(Resource)]
pub struct PhysicsModel(pub Model);

#[derive(Resource)]
pub struct PhysicsData(pub Data);

// Sync system: data.xpos/xquat → Bevy transforms
fn sync_physics_to_bevy(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    mut query: Query<(&BodyIndex, &mut Transform)>,
) {
    for (body_idx, mut transform) in query.iter_mut() {
        let pos = data.0.xpos[body_idx.0];
        let quat = data.0.xquat[body_idx.0];

        // Z-up to Y-up conversion
        transform.translation = Vec3::new(pos.x as f32, pos.z as f32, pos.y as f32);
        transform.rotation = convert_quat_to_bevy(quat);
    }
}

// Step system
fn step_physics(
    model: Res<PhysicsModel>,
    mut data: ResMut<PhysicsData>,
) {
    data.0.step(&model.0);
}
```

### Phase 7: Examples

Rewrite all examples to use `Model`/`Data`:

```rust
// humanoid.rs
fn main() {
    let model = Model::from_mjcf(include_str!("assets/humanoid.xml"))
        .expect("Failed to load humanoid");
    let mut data = model.make_data();

    App::new()
        .add_plugins(DefaultPlugins)
        .insert_resource(PhysicsModel(model))
        .insert_resource(PhysicsData(data))
        .add_systems(Update, step_physics)
        .add_systems(PostUpdate, sync_physics_to_bevy)
        .run();
}
```

---

## Crate Structure After Consolidation

```
sim/L0/
├── types/          # KEEP - basic types (Pose, Twist, MassProperties)
├── simd/           # KEEP - SIMD utilities
├── core/           # EXPAND - Model, Data, pipeline, collision
│   └── src/
│       ├── lib.rs
│       ├── model.rs        # NEW: Model struct
│       ├── data.rs         # NEW: Data struct
│       ├── pipeline.rs     # RENAME from mujoco_pipeline.rs
│       ├── forward.rs      # NEW: FK, velocity FK
│       ├── inverse.rs      # NEW: CRBA, RNE
│       ├── constraint.rs   # NEW: contact + joint constraints
│       ├── integrate.rs    # NEW: Euler, implicit
│       ├── collision/      # KEEP
│       └── raycast.rs      # KEEP
├── mjcf/           # REFACTOR - outputs Model
├── urdf/           # REFACTOR - outputs Model
├── sensor/         # KEEP
├── physics/        # KEEP - re-export facade
│
├── contact/        # MERGE into core/constraint.rs
├── constraint/     # DELETE most, keep joint types
├── muscle/         # KEEP (optional feature)
├── tendon/         # KEEP (optional feature)
├── deformable/     # KEEP (optional feature)
```

---

## Migration Strategy

1. **Build new alongside old** - Model/Data coexists with World initially
2. **Create parallel MJCF loader** - `Model::from_mjcf()` next to old loader
3. **Write new examples** using Model/Data
4. **Validate against old examples** - same physics results
5. **Deprecate old API** with warnings
6. **Delete old code** after validation

---

## Success Criteria

### Correctness Tests

- [ ] **FK correctness**: Set qpos manually, verify xpos/xquat match analytical solution
- [ ] **CRBA correctness**: Compare mass matrix against known analytical solutions (2-link, 3-link)
- [ ] **RNE correctness**: Verify Coriolis/gravity terms match analytical solutions
- [ ] **Energy conservation**: Simple pendulum < 0.1% drift over 10s at 240Hz
- [ ] **Chaotic system**: Double pendulum qualitatively matches reference (not exact due to chaos)
- [ ] **Contact stability**: Ball stack remains stable for 10s without penetration > 1mm
- [ ] **Joint limits**: Limits enforced with < 1% overshoot

### Integration Tests

- [ ] **MJCF parsing**: Load DeepMind Control Suite models (cartpole, acrobot, humanoid)
- [ ] **URDF parsing**: Load standard URDF robots (Panda, UR5)
- [ ] **Actuators**: ctrl input produces expected joint torques
- [ ] **Sensors**: Sensor readings match analytical expectations

### API Tests

- [ ] `Model::from_mjcf()` parses humanoid.xml without error
- [ ] `model.make_data()` produces valid initial state
- [ ] `data.step(&model)` completes without NaN/Inf
- [ ] `data.reset(&model)` restores to qpos0

### Performance Tests

- [ ] Humanoid (20+ DOF): > 10,000 steps/second single-threaded
- [ ] Simple pendulum: > 100,000 steps/second

### Cleanup

- [ ] All old `World`/`Stepper` code deleted
- [ ] `sim-constraint` reduced to joint definitions only
- [ ] `sim-contact` merged into core
- [ ] CI passes with cleaner crate structure

---

## Timeline Estimate

| Phase | Description | Sessions |
|-------|-------------|----------|
| 1 | Define Model/Data structs | 1 |
| 2 | Core pipeline functions | 1-2 |
| 3.1 | Forward kinematics | 1 |
| 3.2-3.3 | CRBA/RNE (generalize existing) | 1 |
| 3.4 | Constraint solver integration | 1-2 |
| 4 | MJCF → Model parser | 1 |
| 5 | Delete old code | 1 |
| 6 | Bevy integration | 1 |
| 7 | Examples + validation | 1-2 |

**Total: ~10-12 focused sessions**

---

## Appendix: MuJoCo Numerical Details (Todorov-Approved)

### Contact Model

MuJoCo uses **soft contacts** (complementarity can be violated). Key parameters:

```
Contact force model:
  f_normal = -k * penetration - b * v_normal    (spring-damper)

Where stiffness/damping derived from impedance parameter d (0 < d < 1):
  R = (1-d)/d × Â    (Â = approximate end-effector inertia)

Friction cone:
  |f_tangent| ≤ μ × f_normal

  Elliptic (default, more physical):  f_t1² + f_t2² ≤ (μ f_n)²
  Pyramidal (faster):                 |f_t1| + |f_t2| ≤ μ f_n
```

Our `sim-contact` already implements this correctly. Key is integrating it into the constraint solver.

### Constraint Solver

MuJoCo's default solver is **Newton** (not PGS). However:
- Newton: 2-3 iterations, exact Hessian, Cholesky factorization
- PGS: 10+ iterations, no matrix factorization, easier to implement

For typical robotics, **PGS with 10-20 iterations and warm-starting** is sufficient. This is what we have in `mujoco_pipeline.rs`.

### Joint Limit Constraints

Joint limits are inequality constraints with reference acceleration:
```
a_ref = -b × (J × v) - k × violation
```
Where `b` is damping and `k` is stiffness. The constraint is:
- Active when joint is at limit AND moving into limit
- Uses Baumgarte stabilization to prevent drift

### Energy Conservation

For conservative systems (no damping, no contacts), semi-implicit Euler conserves energy to O(h²). Key validation:
- Simple pendulum: < 0.1% energy drift over 10 seconds at 240Hz
- Double pendulum: < 1% energy drift (chaotic, more sensitive)

### Numerical Tolerances

From MuJoCo defaults:
- Solver tolerance: 1e-8 (relative change in cost)
- Quaternion normalization: every step
- Position/velocity validation: reset if > 1e10 or NaN

---

## Appendix: Completed Work from Previous Plan

### Phase 0 (Foundational) ✅

- Tendon integration (fixed + spatial with wrapping)
- Equality constraints (weld, joint, distance)
- Muscle actuator integration (HillMuscle)
- Rangefinder ray-casting

### Phase 1 (Wire into sim-physics) ✅

- sim-mjcf, sim-muscle, sim-tendon, sim-sensor added to sim-physics
- sim-deformable behind feature flag

### Phase 2 (Integration Tests) ✅

- 37 integration tests: URDF pipeline, MJCF pipeline, musculoskeletal, sensors

### Phase 3.1 (sim-bevy visualization) ✅

- Muscle visualization (activation heat map)
- Tendon visualization (cable paths)
- Sensor visualization (IMU frames, force arrows, touch, rangefinder)

---

## Appendix: MuJoCo vs Our Current Implementation

| Aspect | MuJoCo | Our Current | Action |
|--------|--------|-------------|--------|
| State representation | qpos/qvel (generalized) | Body states (Cartesian) | **Change to qpos/qvel** |
| Body poses | Computed from FK | Independent state | **Compute from FK** |
| Model vs Data | Separate structs | Mixed in World | **Split Model/Data** |
| Constraint solver | PGS with warm-start | Multiple solvers | **Keep PGS only** |
| Integration | Semi-implicit Euler | Multiple integrators | **Default to semi-implicit** |
| Contact model | Compliant (spring-damper) | Compliant | **Keep** ✅ |
| Collision detection | Built-in | GJK/EPA, BVH | **Keep** ✅ |

---

---

## Out of Scope (For Now)

These MuJoCo features are explicitly NOT in this plan:

| Feature | Reason |
|---------|--------|
| **Newton solver** | PGS sufficient for typical robotics; Newton requires Hessian computation |
| **Sparse mass matrix** | Dense is fine for < 50 DOF; sparse needed for humanoid-scale |
| **RK4 integrator** | Semi-implicit Euler is MuJoCo default; RK4 only for conservative systems |
| **Kinematic loops** | Tree-only for now; loops require special constraint handling |
| **Flex (FEM deformables)** | We have XPBD which is different; keep as optional feature |
| **Composite objects** | Auto-generation of geoms from primitives |
| **Heightfields** | Terrain collision; lower priority |
| **User callbacks** | Control callbacks during RK4 substeps |
| **Visualization in sim-core** | Keep visualization in sim-bevy only |

These can be added later but are not required for MuJoCo-compatible core physics.

---

## References

- [MuJoCo Documentation](https://mujoco.readthedocs.io/)
- [MuJoCo Computation](https://mujoco.readthedocs.io/en/stable/computation/index.html)
- [MuJoCo Simulation Pipeline](https://mujoco.readthedocs.io/en/stable/programming/simulation.html)
- [MuJoCo XML Reference](https://mujoco.readthedocs.io/en/stable/XMLreference.html)
- Featherstone, R. (2008). "Rigid Body Dynamics Algorithms"
- Todorov, E. (2014). "Convex and analytically-invertible dynamics"
