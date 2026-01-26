# MuJoCo Reference Architecture

**Purpose**: This document defines the EXACT architecture we must follow. No deviations.

## Core Principle

MuJoCo's physics pipeline is carefully designed with each stage depending on previous stages. We must implement it **exactly** as specified, not approximate it.

---

## Computation Pipeline (Exact Order)

```
mj_step:
  1. mj_fwdPosition    - Forward kinematics, collision detection, mass matrix
  2. mj_fwdVelocity    - Velocity-dependent quantities, passive forces, bias forces
  3. mj_fwdActuation   - Actuator forces from controls
  4. mj_fwdAcceleration - Constraint solving, compute qacc
  5. mj_Euler          - Semi-implicit integration
```

---

## Stage 1: Forward Position (`mj_fwdPosition`)

### 1.1 Forward Kinematics (`mj_kinematics`)
Computes global position and orientation of all bodies from joint positions.

For each body in tree order:
```
X_world[i] = X_world[parent[i]] * X_joint[i]
```

Where `X_joint[i]` is the spatial transform from joint position `qpos[i]`.

### 1.2 Composite Rigid Body Algorithm (`mj_makeM`)
Computes joint-space inertia matrix M.

**Algorithm (from Featherstone):**
```python
# Initialize composite inertias
for i in range(n_bodies):
    Ic[i] = I_body[i]  # 6x6 spatial inertia

# Backward pass: accumulate composite inertias
for i in range(n_bodies-1, 0, -1):
    Ic[parent[i]] += X_lambda[i].T @ Ic[i] @ X_lambda[i]

# Build inertia matrix
for i in range(n_joints):
    # Diagonal
    H[i,i] = S[i].T @ Ic[i] @ S[i]

    # Off-diagonal: traverse ancestor chain
    j = i
    while parent[j] != 0:
        fh = X_lambda[j].T @ fh
        j = parent[j]
        H[i,j] = S[j].T @ fh
        H[j,i] = H[i,j]
```

Where:
- `Ic[i]` = composite spatial inertia of subtree rooted at body i
- `X_lambda[i]` = spatial transform from body i to parent
- `S[i]` = joint motion subspace (6×dof matrix)
- `fh` = force propagated up the tree

### 1.3 Factor Mass Matrix (`mj_factorM`)
Computes L^T D L factorization of M for efficient solving.

### 1.4 Collision Detection (`mj_collision`)
- Broad phase: sweep-and-prune
- Mid phase: AABB BVH traversal
- Near phase: detailed geometry tests

**Output**: Contact list with points, normals, penetration depths.

### 1.5 Build Constraints (`mj_makeConstraint`)
Instantiates constraint Jacobians for all active constraints.

---

## Stage 2: Forward Velocity (`mj_fwdVelocity`)

### 2.1 Recursive Newton-Euler for Bias Forces (`mj_rne`)

**Forward pass** (velocities):
```python
for i in range(1, n_bodies):
    # Velocity of body i
    v[i] = X_lambda[i] @ v[parent[i]] + S[i] @ qdot[i]

    # Acceleration bias (Coriolis term)
    a_bias[i] = X_lambda[i] @ a_bias[parent[i]] + crossM(v[i]) @ S[i] @ qdot[i]
```

**Forward pass** (forces):
```python
for i in range(1, n_bodies):
    # Force = I*a + v×(I*v) - external forces
    f[i] = Ic[i] @ a_bias[i] + crossF(v[i]) @ Ic[i] @ v[i] - f_ext[i]
```

**Backward pass** (joint forces):
```python
for i in range(n_bodies-1, 0, -1):
    qfrc_bias[i] = S[i].T @ f[i]
    f[parent[i]] += X_lambda[i].T @ f[i]
```

### 2.2 Passive Forces (`mj_passive`)
```
qfrc_passive[i] = -stiffness[i] * (qpos[i] - springref[i]) - damping[i] * qvel[i]
```

---

## Stage 3: Forward Actuation (`mj_fwdActuation`)

For each actuator:
```python
# Activation dynamics (for muscles, filters)
if dyntype == INTEGRATOR:
    act_dot = ctrl
elif dyntype == FILTER:
    act_dot = (ctrl - act) / tau
elif dyntype == MUSCLE:
    # Muscle activation dynamics

# Force generation
force = gain * act + bias

# Map to joint space via transmission
qfrc_actuator += moment.T @ force
```

---

## Stage 4: Forward Acceleration (`mj_fwdAcceleration`)

### 4.1 Compute Total Applied Force
```
qfrc_smooth = qfrc_passive + qfrc_actuator + qfrc_applied - qfrc_bias
```

### 4.2 Unconstrained Acceleration
```
qacc_smooth = M^(-1) @ qfrc_smooth
```

Solved via back-substitution using L^T D L factorization.

### 4.3 Constraint Jacobian Structure

**For revolute joint limit:**
```python
# Jacobian is sparse: 1 at the joint DOF, 0 elsewhere
J[constraint_row, joint_dof] = -sign  # sign depends on which limit
```

**For connect constraint (ball joint):**
```python
# 3 rows for position constraint
# Jacobian is difference of position Jacobians at attachment points
J = jac_body1(point1) - jac_body2(point2)
```

**For weld constraint:**
```python
# 6 rows: 3 position + 3 orientation
J_pos = jac_body1(point1) - jac_body2(point2)
J_rot = rotation_jacobian_difference(body1, body2)
```

### 4.4 Constraint Solver (PGS)

**The Optimization Problem:**
```
minimize: (1/2) λ^T (A + R) λ + λ^T b
subject to: λ ∈ Ω
```

Where:
- `A = J @ M^(-1) @ J.T` (constraint-space inverse inertia)
- `R = diag(regularization)` (softness)
- `b = J @ qacc_smooth - aref` (velocity error)
- `aref = -B * (J @ qvel) - K * constraint_violation` (Baumgarte stabilization)

**PGS Algorithm (from MuJoCo source):**
```python
def pgs_solve(A, R, b, bounds, max_iter, tolerance):
    H = A + R
    H_diag_inv = 1.0 / diag(H)
    force = warmstart  # from previous timestep

    for iteration in range(max_iter):
        improvement = 0

        for i in range(n_constraints):
            # Compute residual for constraint i
            residual = H[i,:] @ force + b[i]

            # Save old force
            old_force = force[i]

            # Gauss-Seidel update
            force[i] -= residual * H_diag_inv[i]

            # Project onto constraint bounds
            force[i] = project(force[i], bounds[i])

            # Track improvement
            improvement += abs(force[i] - old_force)

        if improvement < tolerance:
            break

    return force
```

**Projection rules:**
| Constraint Type | Projection |
|----------------|------------|
| Equality | none (λ ∈ ℝ) |
| Friction loss | clamp to [-η, η] |
| Joint limit | max(0, λ) |
| Contact normal | max(0, λ) |
| Contact friction | clamp to [-μλ_n, μλ_n] |

### 4.5 Final Acceleration
```
qacc = qacc_smooth + M^(-1) @ J.T @ efc_force
```

---

## Stage 5: Integration (`mj_Euler`)

**Semi-implicit Euler (default):**
```python
# Update velocity FIRST (using current acceleration)
qvel_new = qvel + dt * qacc

# Update position using NEW velocity (semi-implicit)
qpos_new = integrate_pos(qpos, qvel_new, dt)

# For quaternions, use proper SO(3) integration
# qpos_new = qpos * exp(qvel_new * dt / 2)
```

**Critical**: Position update uses the NEW velocity, not the old one. This is what makes it semi-implicit and stable.

**With damping (implicit in velocity):**
```python
# Modified mass matrix
H = M + dt * diag(damping)

# Solve for acceleration with implicit damping
qacc = H^(-1) @ (qfrc_smooth + qfrc_constraint)

# Then integrate as above
```

---

## Key Data Structures

### Joint-Space (dimension: nv)
| Name | Description |
|------|-------------|
| `qpos` | Generalized positions |
| `qvel` | Generalized velocities |
| `qacc` | Generalized accelerations |
| `qfrc_bias` | Bias forces (Coriolis + gravity) |
| `qfrc_passive` | Passive forces (springs, dampers) |
| `qfrc_actuator` | Actuator forces |
| `qfrc_applied` | User-applied forces |
| `qfrc_smooth` | Sum of above minus bias |
| `qM` | Inertia matrix (nv × nv) |

### Constraint-Space (dimension: nefc)
| Name | Description |
|------|-------------|
| `efc_J` | Constraint Jacobian (nefc × nv) |
| `efc_force` | Constraint forces (Lagrange multipliers) |
| `efc_aref` | Reference acceleration (for Baumgarte) |
| `efc_AR` | A + R matrix diagonal |

---

## What We Got Wrong Previously

1. **Penalty-based constraints**: MuJoCo uses Lagrange multipliers (impulse-based)
2. **Missing proper Jacobian**: Didn't compute J correctly - need body Jacobians
3. **Wrong integration order**: Semi-implicit uses NEW velocity for position
4. **No inertia matrix**: Missing Composite Rigid Body algorithm
5. **Missing bias forces**: No Recursive Newton-Euler for Coriolis terms
6. **No warmstart**: PGS should reuse forces from previous timestep
7. **Wrong force combination**: Applied - bias, not applied + bias

---

## Implementation Plan

### Phase 1: Single Pendulum (NO CONSTRAINTS)
- One revolute joint connecting fixed world to swinging body
- Implement: forward kinematics, M (scalar for 1-DOF), RNE bias, gravity
- Integration: semi-implicit Euler
- **Visual test**: Pendulum must swing correctly under gravity

### Phase 2: Add Position Constraint
- Add constraint to keep joint attached (prevents drift)
- Implement: Jacobian for revolute joint, PGS solver
- **Visual test**: Pendulum stays attached, doesn't drift

### Phase 3: Two-Link Chain
- Add second revolute joint
- Full 2×2 inertia matrix via CRBA
- **Visual test**: Double pendulum swings chaotically but stays connected

### Phase 4: Three+ Bodies
- Validate CRBA and solver scale correctly
- **Visual test**: Chain of bodies stays intact

### Phase 5: Ball Joints
- Add spherical joint support (3-DOF)
- Quaternion integration
- **Visual test**: Humanoid limbs move correctly

### Phase 6: Contact
- Add contact constraints
- Friction cone projection
- **Visual test**: Objects rest on ground, don't fall through

---

## References

- [MuJoCo Computation](https://mujoco.readthedocs.io/en/stable/computation/index.html)
- [MuJoCo Source - engine_forward.c](https://github.com/google-deepmind/mujoco/blob/main/src/engine/engine_forward.c)
- [MuJoCo Source - engine_solver.c](https://github.com/google-deepmind/mujoco/blob/main/src/engine/engine_solver.c)
- [RBDL Library](https://github.com/rbdl/rbdl) - Reference implementation of Featherstone algorithms
- Featherstone, R. (2008). Rigid Body Dynamics Algorithms. Springer.
- Todorov, E. (2014). Convex and analytically-invertible dynamics with contacts and constraints.
