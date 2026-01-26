# Phase 7 Completion: Todorov Quality Specification

> **Standard**: Every implementation choice must align with MuJoCo's architecture.
> If there's a shortcut that "makes tests pass" but diverges from Todorov's principles, it's wrong.

## Current State Assessment

| Component | Tests Pass? | Todorov Quality? | Gap |
|-----------|-------------|------------------|-----|
| FK (mj_fwd_position) | ✓ | ✓ | None |
| CRBA (mass matrix) | ✓ | ✗ | O(n³) not O(n) |
| RNE (bias forces) | ✓ | ✗ | O(n²) not O(n) |
| Joint limits | ✓ | ✓ | None |
| Contact detection | ✓ | ✗ | O(n²) broad-phase, missing primitives |
| Contact forces | ✓* | ✗ | Penalty-based, free joints only |
| URDF loading | ✓ | ✓ | Single path via MJCF (correct) |
| Performance | ✓* | ✗ | Threshold lowered to pass |

*Tests pass due to band-aids, not correctness.

---

## Part 1: Collision Detection (mj_collision)

### 1.1 Current Problems

1. **O(n²) all-pairs check** - No broad-phase culling
2. **Missing analytical primitives** - Only sphere-*, capsule-capsule implemented
3. **Special case for world body** - Should use contype/conaffinity uniformly

### 1.2 MuJoCo Architecture

```
mj_collision:
  1. Compute geom AABBs from geom_xpos + geom_size
  2. Broad-phase: spatial hash or sweep-and-prune → candidate pairs
  3. For each candidate pair (g1, g2):
     a. Check contype[g1] & conaffinity[g2] || contype[g2] & conaffinity[g1]
     b. Check body exclusion (parent-child, same body)
     c. Narrow-phase: analytical function based on geom_type pair
     d. If penetrating: add to contacts[]
```

### 1.3 Required Implementation

#### Step 1.3.1: Add Broad-Phase Filtering

```rust
fn mj_collision(model: &Model, data: &mut Data) {
    data.ncon = 0;

    // Pre-compute AABBs for all geoms
    let aabbs: Vec<AABB> = (0..model.ngeom).map(|g| {
        compute_geom_aabb(model, data, g)
    }).collect();

    // Spatial hash or sweep-and-prune for candidate pairs
    let candidates = broad_phase_candidates(&aabbs);

    for (g1, g2) in candidates {
        // Check collision affinity
        if !check_collision_affinity(model, g1, g2) {
            continue;
        }

        // Narrow-phase
        if let Some(contact) = narrow_phase(model, data, g1, g2) {
            if data.ncon < MAX_CONTACTS {
                data.contacts[data.ncon] = contact;
                data.ncon += 1;
            }
        }
    }
}

fn check_collision_affinity(model: &Model, g1: usize, g2: usize) -> bool {
    let body1 = model.geom_body[g1];
    let body2 = model.geom_body[g2];

    // Same body - no collision
    if body1 == body2 { return false; }

    // Parent-child - no collision (unless explicitly enabled)
    // Note: This is the ONLY place parent-child check happens
    if model.body_parent[body1] == body2 || model.body_parent[body2] == body1 {
        return false;
    }

    // contype/conaffinity check
    let c1 = model.geom_contype[g1];
    let a1 = model.geom_conaffinity[g1];
    let c2 = model.geom_contype[g2];
    let a2 = model.geom_conaffinity[g2];

    (c1 & a2) != 0 || (c2 & a1) != 0
}
```

#### Step 1.3.2: Implement Missing Analytical Primitives

| Pair | Algorithm | Complexity |
|------|-----------|------------|
| sphere-plane | Point-plane distance | O(1) |
| sphere-sphere | Center distance | O(1) |
| sphere-capsule | Point-segment distance | O(1) |
| sphere-box | Closest point on box | O(1) |
| capsule-capsule | Segment-segment distance | O(1) |
| capsule-plane | Segment-plane distance | O(1) |
| capsule-box | Segment-box distance | O(1) |
| box-box | SAT (15 axes) | O(1) |
| box-plane | Vertex enumeration | O(1) |
| cylinder-* | Approximate as capsule | O(1) |

**Priority order**: sphere-*, capsule-*, box-* (covers 90% of use cases)

```rust
fn narrow_phase(model: &Model, data: &mut Data, g1: usize, g2: usize) -> Option<Contact> {
    let t1 = model.geom_type[g1];
    let t2 = model.geom_type[g2];

    // Ensure t1 <= t2 for canonical ordering
    let (g1, g2, t1, t2) = if t1 <= t2 {
        (g1, g2, t1, t2)
    } else {
        (g2, g1, t2, t1)
    };

    match (t1, t2) {
        (GeomType::Plane, GeomType::Sphere) => collide_plane_sphere(model, data, g1, g2),
        (GeomType::Plane, GeomType::Capsule) => collide_plane_capsule(model, data, g1, g2),
        (GeomType::Plane, GeomType::Box) => collide_plane_box(model, data, g1, g2),
        (GeomType::Sphere, GeomType::Sphere) => collide_sphere_sphere(model, data, g1, g2),
        (GeomType::Sphere, GeomType::Capsule) => collide_sphere_capsule(model, data, g1, g2),
        (GeomType::Sphere, GeomType::Box) => collide_sphere_box(model, data, g1, g2),
        (GeomType::Capsule, GeomType::Capsule) => collide_capsule_capsule(model, data, g1, g2),
        (GeomType::Capsule, GeomType::Box) => collide_capsule_box(model, data, g1, g2),
        (GeomType::Box, GeomType::Box) => collide_box_box(model, data, g1, g2),
        // Cylinder approximated as capsule
        (GeomType::Cylinder, _) | (_, GeomType::Cylinder) => {
            // Approximate cylinder as capsule with same height/radius
            collide_as_capsule(model, data, g1, g2)
        }
        // Fallback to GJK/EPA for mesh-* pairs
        _ => collide_gjk_epa(model, data, g1, g2),
    }
}
```

#### Step 1.3.3: Spatial Hashing Broad-Phase

```rust
struct SpatialHash {
    cell_size: f64,
    cells: HashMap<(i32, i32, i32), Vec<usize>>,
}

impl SpatialHash {
    fn new(cell_size: f64) -> Self {
        Self { cell_size, cells: HashMap::new() }
    }

    fn insert(&mut self, geom_id: usize, aabb: &AABB) {
        let min_cell = self.world_to_cell(aabb.min);
        let max_cell = self.world_to_cell(aabb.max);

        for x in min_cell.0..=max_cell.0 {
            for y in min_cell.1..=max_cell.1 {
                for z in min_cell.2..=max_cell.2 {
                    self.cells.entry((x, y, z)).or_default().push(geom_id);
                }
            }
        }
    }

    fn query_candidates(&self) -> Vec<(usize, usize)> {
        let mut pairs = HashSet::new();
        for geoms in self.cells.values() {
            for i in 0..geoms.len() {
                for j in (i+1)..geoms.len() {
                    let (a, b) = if geoms[i] < geoms[j] {
                        (geoms[i], geoms[j])
                    } else {
                        (geoms[j], geoms[i])
                    };
                    pairs.insert((a, b));
                }
            }
        }
        pairs.into_iter().collect()
    }

    fn world_to_cell(&self, pos: Vector3<f64>) -> (i32, i32, i32) {
        (
            (pos.x / self.cell_size).floor() as i32,
            (pos.y / self.cell_size).floor() as i32,
            (pos.z / self.cell_size).floor() as i32,
        )
    }
}
```

### 1.4 Success Criteria

- [ ] Broad-phase reduces candidate pairs by >90% for typical scenes
- [ ] All analytical primitives implemented (no GJK/EPA fallback for primitives)
- [ ] Performance test passes at 1000+ steps/sec in debug mode
- [ ] No special cases for world body in hot loop

---

## Part 2: Contact Force Application (mj_fwd_constraint)

### 2.1 Current Problems

1. **Penalty-based forces** - Not constraint-based like MuJoCo
2. **Free joints only** - Articulated bodies don't respond to contacts
3. **No Jacobian transpose** - Direct force application instead of J^T λ

### 2.2 MuJoCo Architecture

```
mj_fwd_constraint:
  1. Build constraint Jacobians for each contact
  2. Build constraint residuals (penetration depth + velocity)
  3. Solve for constraint forces λ via PGS/Newton
  4. Apply qfrc_constraint = J^T * λ
```

The key insight: **contact forces are mapped to generalized coordinates via the contact Jacobian**.

### 2.3 Required Implementation

#### Step 2.3.1: Contact Jacobian Computation

For a contact between geom g1 (body b1) and geom g2 (body b2) at world point p with normal n:

```rust
/// Compute contact Jacobian row for a single contact.
///
/// J maps generalized velocities to contact-frame velocity:
///   v_contact = J * qdot
///
/// Returns a 3×nv matrix (normal + 2 tangent directions).
fn contact_jacobian(model: &Model, data: &Data, contact: &Contact) -> DMatrix<f64> {
    let nv = model.nv;
    let mut J = DMatrix::zeros(3, nv);

    let b1 = contact.body1;
    let b2 = contact.body2;
    let p = contact.pos;       // World-space contact point
    let n = contact.normal;    // World-space normal (b2 → b1)

    // Tangent directions (any orthonormal basis in contact plane)
    let (t1, t2) = orthonormal_basis(n);

    // Contribution from body 1 (positive)
    if b1 != 0 {  // Not world body
        add_body_jacobian_contribution(&mut J, model, data, b1, p, n, t1, t2, 1.0);
    }

    // Contribution from body 2 (negative - opposite direction)
    if b2 != 0 {
        add_body_jacobian_contribution(&mut J, model, data, b2, p, n, t1, t2, -1.0);
    }

    J
}

fn add_body_jacobian_contribution(
    J: &mut DMatrix<f64>,
    model: &Model,
    data: &Data,
    body: usize,
    point: Vector3<f64>,
    n: Vector3<f64>,
    t1: Vector3<f64>,
    t2: Vector3<f64>,
    sign: f64,
) {
    // Walk up the kinematic tree from body to root
    let mut current_body = body;

    while current_body != 0 {
        // Get joints for this body
        let jnt_adr = model.body_jnt_adr[current_body];
        let jnt_num = model.body_jnt_num[current_body];

        for j in jnt_adr..(jnt_adr + jnt_num) {
            let jnt_type = model.jnt_type[j];
            let dof_adr = model.jnt_dof_adr[j];

            match jnt_type {
                MjJointType::Hinge => {
                    // Hinge: rotation around axis
                    let axis = data.xmat[current_body] * model.jnt_axis[j];
                    let r = point - data.xpos[current_body];
                    let v = axis.cross(&r);  // Linear velocity from rotation

                    J[(0, dof_adr)] += sign * n.dot(&v);
                    J[(1, dof_adr)] += sign * t1.dot(&v);
                    J[(2, dof_adr)] += sign * t2.dot(&v);
                }
                MjJointType::Slide => {
                    // Slide: translation along axis
                    let axis = data.xmat[current_body] * model.jnt_axis[j];

                    J[(0, dof_adr)] += sign * n.dot(&axis);
                    J[(1, dof_adr)] += sign * t1.dot(&axis);
                    J[(2, dof_adr)] += sign * t2.dot(&axis);
                }
                MjJointType::Ball => {
                    // Ball: 3 rotational DOFs
                    let r = point - data.xpos[current_body];

                    // x-rotation
                    let v_x = Vector3::x().cross(&r);
                    J[(0, dof_adr)] += sign * n.dot(&v_x);
                    J[(1, dof_adr)] += sign * t1.dot(&v_x);
                    J[(2, dof_adr)] += sign * t2.dot(&v_x);

                    // y-rotation
                    let v_y = Vector3::y().cross(&r);
                    J[(0, dof_adr + 1)] += sign * n.dot(&v_y);
                    J[(1, dof_adr + 1)] += sign * t1.dot(&v_y);
                    J[(2, dof_adr + 1)] += sign * t2.dot(&v_y);

                    // z-rotation
                    let v_z = Vector3::z().cross(&r);
                    J[(0, dof_adr + 2)] += sign * n.dot(&v_z);
                    J[(1, dof_adr + 2)] += sign * t1.dot(&v_z);
                    J[(2, dof_adr + 2)] += sign * t2.dot(&v_z);
                }
                MjJointType::Free => {
                    // Free: 3 translation + 3 rotation
                    let r = point - data.xpos[current_body];

                    // Translation DOFs (0-2)
                    J[(0, dof_adr)] += sign * n.x;
                    J[(0, dof_adr + 1)] += sign * n.y;
                    J[(0, dof_adr + 2)] += sign * n.z;

                    J[(1, dof_adr)] += sign * t1.x;
                    J[(1, dof_adr + 1)] += sign * t1.y;
                    J[(1, dof_adr + 2)] += sign * t1.z;

                    J[(2, dof_adr)] += sign * t2.x;
                    J[(2, dof_adr + 1)] += sign * t2.y;
                    J[(2, dof_adr + 2)] += sign * t2.z;

                    // Rotation DOFs (3-5)
                    let v_x = Vector3::x().cross(&r);
                    let v_y = Vector3::y().cross(&r);
                    let v_z = Vector3::z().cross(&r);

                    J[(0, dof_adr + 3)] += sign * n.dot(&v_x);
                    J[(0, dof_adr + 4)] += sign * n.dot(&v_y);
                    J[(0, dof_adr + 5)] += sign * n.dot(&v_z);

                    J[(1, dof_adr + 3)] += sign * t1.dot(&v_x);
                    J[(1, dof_adr + 4)] += sign * t1.dot(&v_y);
                    J[(1, dof_adr + 5)] += sign * t1.dot(&v_z);

                    J[(2, dof_adr + 3)] += sign * t2.dot(&v_x);
                    J[(2, dof_adr + 4)] += sign * t2.dot(&v_y);
                    J[(2, dof_adr + 5)] += sign * t2.dot(&v_z);
                }
            }
        }

        current_body = model.body_parent[current_body];
    }
}
```

#### Step 2.3.2: Constraint Solver (PGS)

```rust
fn mj_fwd_constraint(model: &Model, data: &mut Data) {
    data.qfrc_constraint.fill(0.0);

    // ========== Joint Limit Constraints ==========
    apply_joint_limit_constraints(model, data);

    // ========== Contact Constraints ==========
    if data.ncon == 0 {
        return;
    }

    let nv = model.nv;
    let ncon = data.ncon;

    // Build constraint system
    // Each contact has 1 normal + 2 friction constraints = 3 rows
    let nc = ncon * 3;
    let mut J = DMatrix::zeros(nc, nv);
    let mut b = DVector::zeros(nc);
    let mut lo = DVector::zeros(nc);
    let mut hi = DVector::zeros(nc);

    for i in 0..ncon {
        let contact = &data.contacts[i];
        let Ji = contact_jacobian(model, data, contact);

        // Copy Jacobian rows
        for row in 0..3 {
            for col in 0..nv {
                J[(i * 3 + row, col)] = Ji[(row, col)];
            }
        }

        // Constraint residual
        // b = -depth / dt - velocity_term
        let dt = model.timestep;
        let erp = 0.2;  // Error reduction parameter
        let cfm = 0.0;  // Constraint force mixing

        // Baumgarte stabilization: correct position error over time
        let depth_correction = contact.depth * erp / dt;

        // Contact-frame velocity
        let v_contact = &Ji * &data.qvel;

        b[i * 3] = -depth_correction - v_contact[0];  // Normal
        b[i * 3 + 1] = -v_contact[1];  // Tangent 1
        b[i * 3 + 2] = -v_contact[2];  // Tangent 2

        // Constraint bounds
        // Normal: λ_n >= 0 (can only push)
        lo[i * 3] = 0.0;
        hi[i * 3] = f64::INFINITY;

        // Friction: |λ_t| <= μ * λ_n (Coulomb cone)
        // Handled during iteration
        lo[i * 3 + 1] = f64::NEG_INFINITY;
        hi[i * 3 + 1] = f64::INFINITY;
        lo[i * 3 + 2] = f64::NEG_INFINITY;
        hi[i * 3 + 2] = f64::INFINITY;
    }

    // Compute effective mass: A = J * M^{-1} * J^T
    let M_inv = data.qM.clone().try_inverse().unwrap_or(DMatrix::identity(nv, nv));
    let A = &J * &M_inv * J.transpose();

    // Solve for constraint forces λ using PGS
    let mut lambda = DVector::zeros(nc);

    for _iter in 0..model.solver_iterations {
        for i in 0..ncon {
            let i_n = i * 3;      // Normal index
            let i_t1 = i * 3 + 1; // Tangent 1 index
            let i_t2 = i * 3 + 2; // Tangent 2 index

            // Normal constraint
            let delta_n = (b[i_n] - A.row(i_n).dot(&lambda)) / A[(i_n, i_n)];
            lambda[i_n] = (lambda[i_n] + delta_n).max(0.0);

            // Friction constraints with Coulomb limit
            let mu = data.contacts[i].friction;
            let friction_limit = mu * lambda[i_n];

            let delta_t1 = (b[i_t1] - A.row(i_t1).dot(&lambda)) / A[(i_t1, i_t1)];
            lambda[i_t1] = (lambda[i_t1] + delta_t1).clamp(-friction_limit, friction_limit);

            let delta_t2 = (b[i_t2] - A.row(i_t2).dot(&lambda)) / A[(i_t2, i_t2)];
            lambda[i_t2] = (lambda[i_t2] + delta_t2).clamp(-friction_limit, friction_limit);
        }
    }

    // Apply constraint forces: qfrc_constraint = J^T * λ
    data.qfrc_constraint = J.transpose() * &lambda;
}
```

### 2.4 Success Criteria

- [ ] Contact forces work for articulated bodies (humanoid standing on ground)
- [ ] Jacobian transpose method used (no direct acceleration modification)
- [ ] PGS solver with proper Coulomb friction cone
- [ ] Ball stack test passes with correct physics (not just stable)

---

## Part 3: Performance (CRBA/RNE)

### 3.1 Current Problems

1. **O(n³) CRBA** - Triple nested loops with ancestor lookups
2. **O(n²) RNE** - Ancestor lists rebuilt every step
3. **Dense matrices** - Full nv×nv even for sparse kinematic trees

### 3.2 MuJoCo Architecture

MuJoCo uses Featherstone's algorithms:
- **Composite Rigid Body Algorithm (CRBA)**: O(n) + O(n×depth) = O(n) for trees
- **Recursive Newton-Euler (RNE)**: O(n)
- **Sparse factorization**: L^T D L with O(n×depth) non-zeros

### 3.3 Required Implementation

#### Step 3.3.1: Pre-compute Ancestor Data

At Model construction time:

```rust
impl Model {
    pub fn compute_ancestors(&mut self) {
        self.body_ancestor_joints = vec![vec![]; self.nbody];
        self.body_ancestor_dofs = vec![vec![]; self.nbody];

        for body in 1..self.nbody {
            let mut current = body;
            while current != 0 {
                // Add joints for this body
                let jnt_adr = self.body_jnt_adr[current];
                let jnt_num = self.body_jnt_num[current];

                for j in jnt_adr..(jnt_adr + jnt_num) {
                    self.body_ancestor_joints[body].push(j);

                    let dof_adr = self.jnt_dof_adr[j];
                    let nv = self.jnt_type[j].nv();
                    for d in dof_adr..(dof_adr + nv) {
                        self.body_ancestor_dofs[body].push(d);
                    }
                }

                current = self.body_parent[current];
            }
        }
    }
}
```

#### Step 3.3.2: Featherstone CRBA

```rust
fn mj_crba(model: &Model, data: &mut Data) {
    let nbody = model.nbody;
    let nv = model.nv;

    // Initialize composite inertias with body inertias
    let mut Ic: Vec<SpatialInertia> = (0..nbody)
        .map(|b| SpatialInertia::from_body(model, data, b))
        .collect();

    // Backward pass: accumulate composite inertias
    for body in (1..nbody).rev() {
        let parent = model.body_parent[body];
        if parent != 0 {
            // Transform child composite inertia to parent frame and add
            let X = spatial_transform(data, body, parent);
            Ic[parent] += X.transform_inertia(&Ic[body]);
        }
    }

    // Forward pass: compute mass matrix columns
    data.qM.fill(0.0);

    for body in 1..nbody {
        let jnt_adr = model.body_jnt_adr[body];
        let jnt_num = model.body_jnt_num[body];

        for j in jnt_adr..(jnt_adr + jnt_num) {
            let dof_adr = model.jnt_dof_adr[j];

            // Motion subspace S for this joint
            let S = joint_motion_subspace(model, data, j);

            // F = Ic * S
            let F = Ic[body].apply(&S);

            // Diagonal: M[dof, dof] = S^T * F
            data.qM[(dof_adr, dof_adr)] = S.dot(&F);

            // Off-diagonal: propagate up the tree
            let mut F_prop = F;
            let mut current = model.body_parent[body];

            while current != 0 {
                // Transform F to parent frame
                let X = spatial_transform(data, body, current);
                F_prop = X.transform_force(&F_prop);

                // Add contributions from parent joints
                let p_jnt_adr = model.body_jnt_adr[current];
                let p_jnt_num = model.body_jnt_num[current];

                for pj in p_jnt_adr..(p_jnt_adr + p_jnt_num) {
                    let p_dof_adr = model.jnt_dof_adr[pj];
                    let S_parent = joint_motion_subspace(model, data, pj);

                    let m = S_parent.dot(&F_prop);
                    data.qM[(p_dof_adr, dof_adr)] = m;
                    data.qM[(dof_adr, p_dof_adr)] = m;  // Symmetric
                }

                current = model.body_parent[current];
            }
        }
    }
}
```

#### Step 3.3.3: Recursive Newton-Euler

```rust
fn mj_rne(model: &Model, data: &mut Data) {
    let nbody = model.nbody;

    // Spatial velocities and accelerations
    let mut v: Vec<SpatialVector> = vec![SpatialVector::zero(); nbody];
    let mut a: Vec<SpatialVector> = vec![SpatialVector::zero(); nbody];

    // Gravity as base acceleration (negative because we're computing in world frame)
    a[0] = SpatialVector::from_linear(-model.gravity);

    // Forward pass: compute velocities and accelerations
    for body in 1..nbody {
        let parent = model.body_parent[body];
        let X = spatial_transform(data, parent, body);

        // Transform parent velocity/acceleration to this frame
        v[body] = X.transform_motion(&v[parent]);
        a[body] = X.transform_motion(&a[parent]);

        // Add joint contributions
        let jnt_adr = model.body_jnt_adr[body];
        let jnt_num = model.body_jnt_num[body];

        for j in jnt_adr..(jnt_adr + jnt_num) {
            let dof_adr = model.jnt_dof_adr[j];
            let S = joint_motion_subspace(model, data, j);

            let qdot = data.qvel[dof_adr];
            let qddot = 0.0;  // We compute bias forces with zero acceleration

            v[body] += S * qdot;
            a[body] += S * qddot;

            // Velocity-dependent acceleration (Coriolis)
            a[body] += v[body].cross(&(S * qdot));
        }
    }

    // Spatial forces
    let mut f: Vec<SpatialVector> = vec![SpatialVector::zero(); nbody];

    // Backward pass: compute forces
    for body in (1..nbody).rev() {
        let I = SpatialInertia::from_body(model, data, body);

        // Newton-Euler equation: f = I*a + v × (I*v)
        f[body] = I.apply(&a[body]) + v[body].cross(&I.apply(&v[body]));

        // Add external forces (negated because they reduce required actuation)
        // f[body] -= external_force[body];

        // Propagate to parent
        let parent = model.body_parent[body];
        if parent != 0 {
            let X = spatial_transform(data, body, parent);
            f[parent] += X.transform_force(&f[body]);
        }
    }

    // Project forces onto joint axes to get bias forces
    data.qfrc_bias.fill(0.0);

    for body in 1..nbody {
        let jnt_adr = model.body_jnt_adr[body];
        let jnt_num = model.body_jnt_num[body];

        for j in jnt_adr..(jnt_adr + jnt_num) {
            let dof_adr = model.jnt_dof_adr[j];
            let S = joint_motion_subspace(model, data, j);

            data.qfrc_bias[dof_adr] = S.dot(&f[body]);
        }
    }
}
```

### 3.4 Success Criteria

- [ ] CRBA is O(n) for tree-structured robots
- [ ] RNE is O(n) with no rebuilding of ancestor lists
- [ ] Humanoid achieves >10,000 steps/sec in release mode
- [ ] Pendulum achieves >100,000 steps/sec in release mode
- [ ] Debug mode achieves >1,000 steps/sec (remove lowered threshold)

---

## Part 4: Testing and Validation

### 4.1 Test Modifications Required

1. **Restore debug threshold** in `test_performance_humanoid`:
```rust
// Remove:
#[cfg(debug_assertions)]
let min_threshold = 50.0;

// Replace with:
let min_threshold = 1_000.0;  // Same for debug and release
```

2. **Add articulated contact test**:
```rust
#[test]
fn test_articulated_body_contact() {
    // Double pendulum swinging down to hit ground
    // Verifies contact forces propagate through kinematic tree
}
```

3. **Add humanoid standing test**:
```rust
#[test]
fn test_humanoid_standing_stability() {
    // Humanoid standing on ground plane
    // Should remain stable with contact forces on feet
}
```

### 4.2 Acceptance Criteria

| Test | Current | Target |
|------|---------|--------|
| `test_contact_ball_stack_stability` | ✓ (penalty) | ✓ (constraint-based) |
| `test_urdf_model_data_pipeline` | ✓ | ✓ |
| `test_performance_humanoid` | ✓ (50 threshold) | ✓ (1000 threshold) |
| `test_performance_simple_pendulum` | ✓ | ✓ |
| `test_articulated_body_contact` | N/A | ✓ (new) |
| `test_humanoid_standing_stability` | N/A | ✓ (new) |

---

## Implementation Order

### Week 1: Collision Detection
1. Implement spatial hashing broad-phase
2. Add sphere-box, capsule-box analytical collision
3. Remove world body special case from filtering
4. Verify performance threshold can be restored to 1000

### Week 2: Contact Forces
1. Implement contact Jacobian computation
2. Implement PGS solver with friction cone
3. Replace penalty forces with constraint-based
4. Add articulated contact test

### Week 3: CRBA/RNE Optimization
1. Add spatial algebra types (SpatialVector, SpatialInertia)
2. Implement Featherstone CRBA
3. Implement recursive RNE
4. Benchmark and verify performance targets

### Week 4: Integration
1. Run full test suite
2. Add humanoid standing test
3. Remove all band-aids and thresholds
4. Document completion

---

## Definition of Done

Phase 7 is complete **to Todorov's standards** when:

- [ ] All collision detection uses broad-phase (no O(n²) all-pairs)
- [ ] All analytical primitive pairs implemented (no GJK fallback for primitives)
- [ ] Contact forces use Jacobian transpose (works for any joint type)
- [ ] PGS solver with proper Coulomb friction cone
- [ ] CRBA is O(n) using Featherstone's algorithm
- [ ] RNE is O(n) using recursive formulation
- [ ] Debug threshold is 1,000 steps/sec (no band-aid lowering)
- [ ] Humanoid with ground contact remains stable
- [ ] No special cases or hacks in hot paths
- [ ] All tests pass without `#[ignore]` annotations
