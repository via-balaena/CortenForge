# Phase 7 Completion Plan: MuJoCo-Aligned Implementation

> **Goal**: Complete all Phase 7 validation requirements before proceeding to Phase 8.
> **Approach**: Adhere strictly to MuJoCo's principles - generalized coordinates as truth,
> computed body poses, constraint-based dynamics.

## Executive Summary

Three gaps must be addressed:

| Gap | Current State | Target | Approach |
|-----|---------------|--------|----------|
| Contact Physics | Infrastructure exists, not wired | Ball stack stable 10s, <1mm penetration | Wire collision detection to Model/Data pipeline |
| URDF → Model | Uses old World API | Load Panda/UR5 into Model | Add `load_urdf_model()` function |
| Performance | Humanoid 1.7k, Pendulum 40k | Humanoid >10k, Pendulum >100k | Optimize CRBA/RNE with sparse algorithms |

---

## Part 1: Contact Physics Integration

### 1.1 MuJoCo Contact Architecture

MuJoCo's contact pipeline follows this strict order:

```
mj_step:
  mj_forward:
    mj_fwdPosition:      qpos → xpos, xquat, geom_xpos, geom_xmat
    mj_collision:        geom_xpos × geom_xpos → contacts[]
    mj_fwdVelocity:      qvel → cvel (body velocities)
    mj_fwdActuation:     ctrl → qfrc_actuator
    mj_fwdAcceleration:
      mj_crba:           mass matrix M
      mj_rne:            bias forces c(q,qdot)
      mj_fwdConstraint:  contacts → constraint Jacobians → PGS → qfrc_constraint
  mj_Euler:              qpos, qvel ← integrate(qacc)
```

**Key Insight**: Collision detection happens AFTER FK updates `geom_xpos` but BEFORE
velocity/dynamics computation. This is critical - contacts use position-only information.

### 1.2 Current Infrastructure

**Available in sim-core/mujoco_pipeline.rs:**
- `Data::contacts: Vec<Contact>` - pre-allocated contact array
- `Data::ncon: usize` - active contact count
- `ContactPoint` struct for constraint generation
- `PGSSolver` with warm-starting
- `contact_constraints()` method to build Jacobians
- `mj_fwd_constraint()` placeholder (currently only handles joint limits)

**Available in sim-core/collision/:**
- `BroadPhaseDetector` (SweepAndPrune / BruteForce)
- GJK/EPA narrow-phase
- All shape types supported

**Available in sim-contact/:**
- `ContactModel` with MuJoCo-style soft contacts
- Elliptic friction cones
- `ContactSolver` with fixed iterations

### 1.3 Implementation Plan

#### Step 1: Add Collision Detection to Model/Data

Create new function `mj_collision()` that:
1. Queries `data.geom_xpos` and `data.geom_xmat` for world-space poses
2. Runs broad-phase on geometry AABBs
3. Runs narrow-phase (GJK/EPA) on candidate pairs
4. Populates `data.contacts` with `Contact` structs

```rust
// In mujoco_pipeline.rs

/// Collision detection: populate contacts from geometry pairs.
fn mj_collision(model: &Model, data: &mut Data) {
    data.contacts.clear();
    data.ncon = 0;

    // Build AABBs from geom_xpos + geom sizes
    // Run broad-phase to get candidate pairs
    // For each candidate pair:
    //   - Get shapes from model.geom_type, model.geom_size
    //   - Transform to world frame using data.geom_xpos, data.geom_xmat
    //   - Run GJK/EPA to get contact point, normal, depth
    //   - If contact: push to data.contacts, increment data.ncon
}
```

#### Step 2: Integrate into Forward Dynamics

Modify `Data::forward()`:

```rust
pub fn forward(&mut self, model: &Model) {
    // Position Stage
    mj_fwd_position(model, self);
    mj_collision(model, self);           // NEW: detect contacts
    mj_sensor_pos(model, self);
    mj_energy_pos(model, self);

    // Velocity Stage
    mj_fwd_velocity(model, self);
    mj_sensor_vel(model, self);

    // Acceleration Stage
    mj_fwd_actuation(model, self);
    mj_crba(model, self);
    mj_rne(model, self);
    mj_energy_vel(model, self);
    mj_fwd_passive(model, self);
    mj_fwd_constraint(model, self);      // Now handles contacts + limits
    mj_fwd_acceleration(model, self);
    mj_sensor_acc(model, self);
}
```

#### Step 3: Wire Contacts to Constraint Solver

Modify `mj_fwd_constraint()` to handle contacts:

```rust
fn mj_fwd_constraint(model: &Model, data: &mut Data) {
    data.qfrc_constraint.fill(0.0);

    // ========== Joint Limit Constraints (existing) ==========
    // ... existing code ...

    // ========== Contact Constraints (NEW) ==========
    if data.ncon > 0 {
        // Build ArticulatedSystem for PGS
        let sys = ArticulatedSystem::new(/* ... */);

        // Convert contacts to ContactPoint format
        let contact_points: Vec<ContactPoint> = data.contacts.iter().map(|c| {
            ContactPoint {
                position: c.pos,
                normal: c.normal,
                depth: c.depth,
                body_a: Some(model.geom_body[c.geom1]),
                body_b: if c.geom2 < model.ngeom { Some(model.geom_body[c.geom2]) } else { None },
                friction: c.friction,
            }
        }).collect();

        // Generate constraints
        let baumgarte_k = 100.0;  // Position stabilization
        let baumgarte_b = 10.0;   // Velocity damping
        let constraints = sys.contact_constraints(&contact_points, baumgarte_k, baumgarte_b);

        // Compute smooth acceleration (without constraints)
        let m_inv = data.qM.clone().try_inverse().unwrap_or_else(DMatrix::identity);
        let qfrc_smooth = &data.qfrc_applied + &data.qfrc_actuator + &data.qfrc_passive - &data.qfrc_bias;
        let qacc_smooth = &m_inv * &qfrc_smooth;

        // Solve with PGS
        let mut solver = PGSSolver::new(model.solver_iterations);
        let result = solver.solve(&m_inv, &qacc_smooth, &constraints);

        // Apply constraint forces
        for (i, c) in constraints.iter().enumerate() {
            let jt_lambda = c.jacobian.transpose() * result.forces[i];
            data.qfrc_constraint += jt_lambda;
        }
    }
}
```

#### Step 4: Geom Pose Computation

Add `mj_fwd_geom()` to compute geometry world poses:

```rust
fn mj_fwd_geom(model: &Model, data: &mut Data) {
    for geom_id in 0..model.ngeom {
        let body_id = model.geom_body[geom_id];

        // Transform geom pose from body frame to world frame
        data.geom_xpos[geom_id] = data.xpos[body_id] + data.xquat[body_id] * model.geom_pos[geom_id];
        let geom_quat = data.xquat[body_id] * model.geom_quat[geom_id];
        data.geom_xmat[geom_id] = geom_quat.to_rotation_matrix().into_inner();
    }
}
```

This should be called at the end of `mj_fwd_position()`.

### 1.4 Testing Strategy

The `test_contact_ball_stack_stability` test is already written. It will pass once:
1. `mj_collision()` detects sphere-sphere and sphere-plane contacts
2. `mj_fwd_constraint()` generates normal + friction constraints
3. PGS solver resolves constraint forces
4. Integration maintains contact stability

---

## Part 2: URDF → Model Conversion

### 2.1 MuJoCo Model Structure

The URDF → Model conversion must produce the same `Model` structure as MJCF:

```rust
pub struct Model {
    // Dimensions
    pub nq: usize,              // Generalized position coordinates
    pub nv: usize,              // Generalized velocity coordinates (DOFs)
    pub nbody: usize,           // Bodies (including world body 0)
    pub njnt: usize,            // Joints
    pub ngeom: usize,           // Geometries

    // Body tree (indexed by body_id)
    pub body_parent: Vec<usize>,
    pub body_pos: Vec<Vector3<f64>>,
    pub body_quat: Vec<UnitQuaternion<f64>>,
    pub body_mass: Vec<f64>,
    pub body_inertia: Vec<Vector3<f64>>,
    pub body_jnt_adr: Vec<usize>,
    pub body_jnt_num: Vec<usize>,

    // Joints (indexed by jnt_id)
    pub jnt_type: Vec<MjJointType>,
    pub jnt_body: Vec<usize>,
    pub jnt_qpos_adr: Vec<usize>,
    pub jnt_dof_adr: Vec<usize>,
    pub jnt_axis: Vec<Vector3<f64>>,
    pub jnt_pos: Vec<Vector3<f64>>,
    pub jnt_limited: Vec<bool>,
    pub jnt_range: Vec<(f64, f64)>,
    pub jnt_damping: Vec<f64>,
    // ...
}
```

### 2.2 URDF Joint Type Mapping

| URDF Type | MuJoCo Type | nq | nv | Notes |
|-----------|-------------|----|----|-------|
| fixed | (none) | 0 | 0 | No joint - bodies welded |
| revolute | Hinge | 1 | 1 | Limited rotation |
| continuous | Hinge | 1 | 1 | Unlimited rotation (limited=false) |
| prismatic | Slide | 1 | 1 | Limited translation |
| floating | Free | 7 | 6 | 6-DOF floating base |
| planar | Planar | 3 | 3 | XY translation + Z rotation |

### 2.3 Implementation Plan

#### Step 1: Add `load_urdf_model()` Function

In `sim-urdf/src/lib.rs`:

```rust
/// Load URDF into MuJoCo-aligned Model structure.
pub fn load_urdf_model(urdf: &str) -> Result<sim_core::Model, UrdfError> {
    let robot = parse_urdf_str(urdf)?;
    model_from_urdf(&robot)
}

/// Convert parsed URDF robot to Model.
fn model_from_urdf(robot: &UrdfRobot) -> Result<sim_core::Model, UrdfError> {
    let mut builder = ModelBuilder::new();

    // Build kinematic tree from URDF parent-child relationships
    let root_link = find_root_link(robot)?;

    // Recursively process links and joints
    process_link(&mut builder, robot, &root_link, 0 /* parent_body_id */)?;

    builder.build()
}
```

#### Step 2: Create `ModelBuilder`

Add helper to construct Model from URDF:

```rust
struct ModelBuilder {
    // Body data
    body_parent: Vec<usize>,
    body_pos: Vec<Vector3<f64>>,
    body_quat: Vec<UnitQuaternion<f64>>,
    body_mass: Vec<f64>,
    body_inertia: Vec<Vector3<f64>>,
    body_jnt_adr: Vec<usize>,
    body_jnt_num: Vec<usize>,
    body_name: Vec<Option<String>>,

    // Joint data
    jnt_type: Vec<MjJointType>,
    jnt_body: Vec<usize>,
    jnt_axis: Vec<Vector3<f64>>,
    jnt_pos: Vec<Vector3<f64>>,
    jnt_limited: Vec<bool>,
    jnt_range: Vec<(f64, f64)>,
    jnt_damping: Vec<f64>,
    jnt_name: Vec<Option<String>>,

    // Geometry data
    geom_type: Vec<GeomType>,
    geom_body: Vec<usize>,
    geom_pos: Vec<Vector3<f64>>,
    geom_quat: Vec<UnitQuaternion<f64>>,
    geom_size: Vec<Vector3<f64>>,

    // Running counters
    nq: usize,
    nv: usize,
}

impl ModelBuilder {
    fn new() -> Self {
        let mut builder = Self::default();
        // Add world body (body 0)
        builder.add_world_body();
        builder
    }

    fn add_body(&mut self, name: &str, parent: usize, pos: Vector3<f64>,
                quat: UnitQuaternion<f64>, mass: f64, inertia: Vector3<f64>) -> usize {
        let body_id = self.body_parent.len();
        self.body_parent.push(parent);
        self.body_pos.push(pos);
        self.body_quat.push(quat);
        self.body_mass.push(mass);
        self.body_inertia.push(inertia);
        self.body_jnt_adr.push(self.jnt_type.len());
        self.body_jnt_num.push(0);
        self.body_name.push(Some(name.to_string()));
        body_id
    }

    fn add_joint(&mut self, body_id: usize, jnt_type: MjJointType,
                 axis: Vector3<f64>, limited: bool, range: (f64, f64),
                 damping: f64, name: &str) {
        self.jnt_type.push(jnt_type);
        self.jnt_body.push(body_id);
        self.jnt_axis.push(axis);
        self.jnt_pos.push(Vector3::zeros()); // Joint at body origin
        self.jnt_limited.push(limited);
        self.jnt_range.push(range);
        self.jnt_damping.push(damping);
        self.jnt_name.push(Some(name.to_string()));

        // Update body joint count
        self.body_jnt_num[body_id] += 1;

        // Update nq/nv
        self.nq += jnt_type.nq();
        self.nv += jnt_type.nv();
    }

    fn build(self) -> Result<Model, UrdfError> {
        // Compute jnt_qpos_adr, jnt_dof_adr
        // Build final Model structure
        // ...
    }
}
```

#### Step 3: Recursive Link Processing

```rust
fn process_link(builder: &mut ModelBuilder, robot: &UrdfRobot,
                link_name: &str, parent_body_id: usize) -> Result<(), UrdfError> {
    let link = robot.links.iter().find(|l| l.name == link_name)
        .ok_or(UrdfError::LinkNotFound(link_name.to_string()))?;

    // Extract mass properties
    let (mass, inertia) = if let Some(inertial) = &link.inertial {
        (inertial.mass, Vector3::new(inertial.inertia.ixx, inertial.inertia.iyy, inertial.inertia.izz))
    } else {
        (0.001, Vector3::new(1e-6, 1e-6, 1e-6)) // Minimal mass for stability
    };

    // For root: pos/quat relative to world (identity)
    // For children: pos/quat from joint origin
    let (pos, quat) = if parent_body_id == 0 {
        (Vector3::zeros(), UnitQuaternion::identity())
    } else {
        // Find joint connecting parent to this link
        let joint = find_joint_to_child(robot, link_name)?;
        let pos = Vector3::new(joint.origin.xyz.x, joint.origin.xyz.y, joint.origin.xyz.z);
        let quat = rpy_to_quat(joint.origin.rpy);
        (pos, quat)
    };

    // Add body
    let body_id = builder.add_body(link_name, parent_body_id, pos, quat, mass, inertia);

    // Add joint if not root
    if parent_body_id != 0 {
        let joint = find_joint_to_child(robot, link_name)?;
        let mj_type = urdf_to_mujoco_joint_type(&joint.joint_type);
        let limited = joint.limit.is_some() && joint.joint_type != UrdfJointType::Continuous;
        let range = joint.limit.as_ref()
            .map(|l| (l.lower, l.upper))
            .unwrap_or((-3.14159, 3.14159));
        let damping = joint.dynamics.as_ref().map(|d| d.damping).unwrap_or(0.0);

        if mj_type.nv() > 0 {
            builder.add_joint(body_id, mj_type, joint.axis, limited, range, damping, &joint.name);
        }
    }

    // Add collision geometries
    for collision in &link.collisions {
        builder.add_geom(body_id, &collision.geometry, &collision.origin);
    }

    // Recursively process children
    for joint in &robot.joints {
        if joint.parent == link_name {
            process_link(builder, robot, &joint.child, body_id)?;
        }
    }

    Ok(())
}
```

### 2.4 Testing Strategy

Update `test_urdf_model_data_pipeline` to load actual URDFs:

```rust
#[test]
fn test_urdf_model_data_pipeline() {
    // Simple two-link arm
    let urdf = r#"
        <robot name="two_link">
            <link name="base">
                <inertial><mass value="1.0"/><inertia ixx="0.1" iyy="0.1" izz="0.1"/></inertial>
            </link>
            <link name="link1">
                <inertial><mass value="0.5"/><inertia ixx="0.01" iyy="0.01" izz="0.01"/></inertial>
            </link>
            <joint name="j1" type="revolute">
                <parent link="base"/><child link="link1"/>
                <origin xyz="0 0 0.5"/><axis xyz="0 1 0"/>
                <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
            </joint>
        </robot>
    "#;

    let model = load_urdf_model(urdf).expect("should load");
    assert_eq!(model.nbody, 3); // world + base + link1
    assert_eq!(model.njnt, 1);
    assert_eq!(model.nq, 1);
    assert_eq!(model.nv, 1);

    let mut data = model.make_data();
    data.qpos[0] = 0.5;
    data.forward(&model);

    // Verify FK computes correct poses
    assert!(data.xpos[2].z > 0.4);  // link1 should be above origin
}
```

---

## Part 3: Performance Optimization

### 3.1 Performance Analysis

Current bottlenecks (profiled):

1. **CRBA**: O(n³) due to triple-nested loops + `contains()` calls
2. **RNE**: O(n²) due to nested loops with ancestor lookups
3. **Dense matrix operations**: Full nv×nv matrices even for sparse trees

MuJoCo achieves high performance via:
- Featherstone's O(n) algorithms
- Sparse L^T D L factorization
- Cache-friendly data layout

### 3.2 Optimization Strategy

#### Phase A: Algorithm Improvements (High Impact)

**A1: Replace CRBA with Featherstone's Algorithm**

Instead of computing M[i,j] = Σ J^T I J for all pairs, use the recursive formula:

```
// Backward pass: accumulate composite inertias
for body = n down to 1:
    I_composite[body] = I_body + Σ_children I_composite[child]

// Forward pass: compute M columns
for body = 1 to n:
    F = I_composite[body] * S[body]  // S = motion subspace
    M[dof[body], dof[body]] = S^T * F

    // Propagate to ancestors
    parent = body
    while parent != world:
        F = φ * F  // φ = spatial transform to parent
        M[dof[parent], dof[body]] = S[parent]^T * F
        M[dof[body], dof[parent]] = M[dof[parent], dof[body]]
        parent = parent_of(parent)
```

This is O(n) for the backward pass + O(n×depth) for the forward pass = O(n) for trees.

**A2: Replace RNE with Recursive Implementation**

```
// Forward pass: compute velocities and accelerations
for body = 1 to n:
    v[body] = φ * v[parent] + S * qdot
    a[body] = φ * a[parent] + S * qddot + v × (S * qdot)

// Backward pass: compute forces
for body = n down to 1:
    f[body] = I * a[body] + v × (I * v)  // Newton-Euler
    τ[dof[body]] = S^T * f[body]
    f[parent] += φ^T * f[body]  // Propagate to parent
```

This is strictly O(n).

#### Phase B: Data Structure Improvements (Medium Impact)

**B1: Pre-compute Ancestor Lists at Model Construction**

Instead of building ancestor lists every step:

```rust
// In Model construction
pub body_ancestors: Vec<Vec<usize>>,  // Pre-computed ancestor body IDs
pub body_ancestor_joints: Vec<Vec<usize>>,  // Pre-computed ancestor joint IDs
```

**B2: Use Sparse Mass Matrix Storage**

For tree-structured robots, M has O(n×depth) non-zeros, not O(n²).

```rust
// Store M in compressed sparse row format
pub struct SparseMassMatrix {
    values: Vec<f64>,
    col_indices: Vec<usize>,
    row_ptrs: Vec<usize>,
}
```

#### Phase C: Low-Level Optimizations (Lower Impact)

**C1: SIMD for Vector Operations**

Use `sim-simd` for batched vector operations in FK/CRBA/RNE.

**C2: Cache-Friendly Layout**

Ensure body data is stored contiguously for tree traversal:
- Bodies sorted in topological order (already done)
- Hot data (mass, inertia) packed together

### 3.3 Implementation Priority

For Phase 7 completion (hitting 10k humanoid, 100k pendulum):

1. **Immediate**: Pre-compute ancestor lists (easy, significant speedup)
2. **Short-term**: Implement Featherstone CRBA (moderate effort, large speedup)
3. **Medium-term**: Implement recursive RNE (moderate effort, moderate speedup)
4. **Long-term**: Sparse mass matrix (significant effort, essential for humanoid+)

### 3.4 Performance Targets

| Model | Current | Target | After Optimization |
|-------|---------|--------|-------------------|
| Simple pendulum (1 DOF) | 40k steps/s | 100k steps/s | ~150k steps/s expected |
| Humanoid (25 DOF) | 1.7k steps/s | 10k steps/s | ~15k steps/s expected |
| 15-link pendulum (15 DOF) | 20 steps/s | 1k steps/s | ~3k steps/s expected |

---

## Implementation Order

### Week 1: Performance (Highest Priority)

1. **Pre-compute ancestors** in Model construction
2. **Implement Featherstone CRBA** to replace O(n³) algorithm
3. **Verify** pendulum hits 100k, humanoid approaches 10k

### Week 2: Contact Physics

1. **Add `mj_collision()`** function
2. **Wire to PGS solver** in `mj_fwd_constraint()`
3. **Test** ball stack stability

### Week 3: URDF Support

1. **Add `load_urdf_model()`** function
2. **Create `ModelBuilder`** helper
3. **Test** with simple robots

### Week 4: Integration & Polish

1. **Run all Phase 7 tests**
2. **Remove `#[ignore]` from contact/URDF tests**
3. **Update documentation**
4. **Prepare for Phase 8**

---

## Success Criteria

Phase 7 is complete when:

- [ ] `test_contact_ball_stack_stability` passes (not ignored)
- [ ] `test_urdf_model_data_pipeline` passes (not ignored)
- [ ] `test_performance_humanoid` prints "✓ MEETS SPEC" (>10k steps/s)
- [ ] `test_performance_simple_pendulum` prints "✓ MEETS SPEC" (>100k steps/s)
- [ ] All 76+ integration tests pass
- [ ] Clippy passes with no warnings

---

## References

- Featherstone, R. (2008). "Rigid Body Dynamics Algorithms"
- MuJoCo Documentation: Computation chapter
- Todorov, E. (2014). "Convex and analytically-invertible dynamics"
