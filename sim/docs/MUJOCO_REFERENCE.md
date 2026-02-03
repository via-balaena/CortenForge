# MuJoCo Reference Architecture

This document defines the physics pipeline architecture implemented in
`sim/L0/core/src/mujoco_pipeline.rs`. The pipeline follows MuJoCo's computation
model: `forward()` (8 sub-stages) then either `integrate()` (Euler/ImplicitSpringDamper)
or `mj_runge_kutta()` (RK4) run each timestep, operating on a static `Model` and
mutable `Data`.

---

## Computation Pipeline

```
Data::step():
  1. forward()
     a. mj_fwd_position    — Forward kinematics
        mj_fwd_tendon       — Tendon lengths + Jacobians (fixed tendons)
        mj_collision        — Broad/narrow phase collision detection
     b. mj_fwd_velocity    — Body + tendon velocities from joint velocities
     c. mj_fwd_actuation   — Actuator forces from controls (joint + tendon transmission)
     d. mj_crba            — Mass matrix (Composite Rigid Body Algorithm)
     e. mj_rne             — Bias forces (Recursive Newton-Euler)
     f. mj_fwd_passive     — Spring, damper, friction loss forces (joints + tendons)
     g. mj_fwd_constraint  — Joint/tendon limits, equality constraints, contact PGS
     h. mj_fwd_acceleration — Solve M*qacc = f (explicit) or implicit velocity update
  2a. integrate()          — Semi-implicit Euler or implicit position update
                              (for Euler / ImplicitSpringDamper integrators)
  2b. mj_runge_kutta()     — True 4-stage RK4 with Butcher tableau
                              (for RungeKutta4 integrator)
```

---

## Stage 1: Forward Position (`mj_fwd_position`)

### 1.1 Forward Kinematics

Computes global position and orientation of all bodies from joint positions.

For each body in tree order:
```
X_world[i] = X_world[parent[i]] * X_joint[i]
```

Where `X_joint[i]` depends on joint type:
- **Hinge**: rotation about body-frame axis by `qpos[i]`
- **Slide**: translation along body-frame axis by `qpos[i]`
- **Ball**: rotation by quaternion `qpos[i..i+4]`
- **Free**: translation `qpos[i..i+3]` + rotation `qpos[i+3..i+7]`

### 1.2 Collision Detection (`mj_collision`)

- Broad phase: sweep-and-prune on AABBs
- Affinity filter: contype/conaffinity bitmasks, parent-child exclusion
- Narrow phase: per-pair geometry tests

Narrow phase dispatches analytically for common pairs (sphere-sphere,
sphere-capsule, box-box via SAT, etc.) and falls back to GJK/EPA for
remaining convex pairs. Mesh collisions use BVH-accelerated triangle tests.

Output: contact list with position, normal, penetration depth, and geom IDs.

---

## Stage 2: Forward Velocity (`mj_fwd_velocity`)

Computes body spatial velocities from joint velocities.

For each body in tree order:
```python
v[i] = v[parent[i]] + omega[parent[i]] x r[parent->i] + S[i] @ qdot[i]
```

Where:
- `v[i]` is the 6D spatial velocity (angular, linear) of body i
- `S[i]` is the joint motion subspace (6 x ndof matrix)
- `r[parent->i]` is the position vector from parent to child (lever arm)

The lever arm cross-product `omega x r` is critical for correct Coriolis forces.

---

## Stage 3: Forward Actuation (`mj_fwd_actuation`)

Maps control inputs to generalized forces.

For each actuator:
```python
if transmission == Joint:
    qfrc_actuator[dof] += ctrl * gear

if transmission == Tendon:
    force = ctrl * gear
    for w in tendon_wrap_range:
        qfrc_actuator[wrap_objid[w]] += wrap_prm[w] * force  # J^T mapping

if transmission == Site:
    # Not yet implemented (requires spatial tendon support)
```

Joint-transmission actuators apply force directly to the joint DOF scaled by
the gear ratio. Tendon-transmission actuators map the force through the tendon
Jacobian transpose to all coupled joints.

---

## Stage 4: Dynamics

### 4.1 Composite Rigid Body Algorithm (`mj_crba`)

Computes the joint-space inertia matrix M using Featherstone's algorithm.

**Phase 1 — Initialize composite inertias:**
```python
for i in range(nbody):
    Ic[i] = I[i]   # Body's own 6x6 spatial inertia
```

**Phase 2 — Backward pass (accumulate subtree inertias):**
```python
for i in range(nbody-1, 0, -1):
    Ic[parent[i]] += Ic[i]
```

**Phase 3 — Build mass matrix:**
```python
for i in range(njnt):
    body_i = jnt_body[i]

    # Diagonal: project composite inertia through joint subspace
    M[i,i] = S[i].T @ Ic[body_i] @ S[i]

    # Off-diagonal: propagate force up kinematic chain
    F = Ic[body_i] @ S[i]
    j = body_i
    while parent[j] != 0:
        # Spatial force transform via lever arm
        r = xpos[j] - xpos[parent[j]]
        F_angular += r x F_linear
        j = parent[j]
        for each joint k on body j:
            M[k,i] = S[k].T @ F
            M[i,k] = M[k,i]
```

**Phase 4 — Add armature inertia:**
```python
M[dof,dof] += armature[dof]
```

**Phase 5 — Sparse L^T D L factorization (`mj_factor_sparse`):**

Exploits kinematic tree sparsity via `dof_parent` for O(Σ depth(i)²)
factorization vs O(nv³) dense Cholesky. Stores result in `qLD_diag` (diagonal D)
and `qLD_L` (unit lower triangular L as sparse rows). Reused by
`mj_fwd_acceleration_explicit` and `pgs_solve_contacts` via `mj_solve_sparse`.

```python
# Phase 1: Copy M's sparse entries into qLD working storage
for i in range(nv):
    qLD_diag[i] = M[i,i]
    qLD_L[i] = [(j, M[i,j]) for j in ancestors(i)]  # via dof_parent chain

# Phase 2: Eliminate from leaves to root
for i in range(nv-1, -1, -1):
    di = qLD_diag[i]
    for (j, val) in qLD_L[i]:
        qLD_L[i][j] = val / di          # Normalize L entries
    for (j, lij) in qLD_L[i]:
        qLD_diag[j] -= lij * lij * di   # Update ancestor diagonal
        for (k, lik) in qLD_L[i] where k < j:
            qLD_L[j][k] -= lij * lik * di  # Update ancestor off-diagonal
```

### 4.2 Recursive Newton-Euler (`mj_rne`)

Computes bias forces `qfrc_bias` (gravity + Coriolis + centrifugal + gyroscopic).

**Gravity (O(n) using precomputed subtree mass and COM):**
```python
for each joint i on body b:
    F_grav = -subtree_mass[b] * gravity
    if Hinge:
        tau = ((subtree_com[b] - jnt_pos) x F_grav) . axis
    if Slide:
        tau = F_grav . axis
    if Ball:
        tau = R_body^-1 * ((subtree_com[b] - jnt_pos) x F_grav)
    if Free:
        tau_lin = F_grav
        tau_ang = (subtree_com[b] - jnt_pos) x F_grav
    qfrc_bias[dof] += tau
```

**Gyroscopic terms (Ball and Free joints only):**
```python
omega = angular velocity of body in world frame
tau_gyro = omega x (I_body @ omega)
qfrc_bias[angular_dofs] += S.T @ tau_gyro
```

**Coriolis and centrifugal — Forward pass (bias accelerations):**
```python
for i in range(1, nbody):
    a_bias[i] = a_bias[parent[i]]
    for each joint on body i:
        v_joint = S @ qdot
        # Velocity-product acceleration (centripetal/Coriolis)
        # Uses parent velocity because v[i] x_m v_joint = v[parent] x_m v_joint
        # (since v_joint x_m v_joint = 0)
        a_bias[i] += v[parent[i]] x_m v_joint
```

**Coriolis and centrifugal — Forward pass (bias forces):**
```python
for i in range(1, nbody):
    f_bias[i] = I[i] @ a_bias[i] + v[i] x* (I[i] @ v[i])
```

**Coriolis and centrifugal — Backward pass (project to joint space):**
```python
for i in range(nbody-1, 0, -1):
    f_bias[parent[i]] += f_bias[i]

for each joint i on body b:
    qfrc_bias[dof] += S[i].T @ f_bias[b]
```

Where:
- `x_m` is the spatial motion cross-product
- `x*` is the spatial force cross-product
- `I[i]` is the body's own spatial inertia (not the composite)

### 4.3 Passive Forces (`mj_fwd_passive`)

```
qfrc_passive[dof] = -stiffness * (qpos - springref)
                    - damping * qvel
                    - frictionloss * tanh(qvel * friction_smoothing)
```

Where:
- `springref` is the spring equilibrium position (distinct from initial `qpos0`)
- `frictionloss` is Coulomb friction magnitude (velocity-independent)
- `tanh(qvel * 1000)` smoothly approximates `sign(qvel)` to avoid discontinuity

In implicit integration mode, spring and damper terms are handled in the
implicit solve (section 4.7). Only friction loss is computed here.

**Tendon passive forces** follow the same pattern, mapped through J^T:
```
ten_force[t] = -stiffness * (ten_length - lengthspring)
               - damping * ten_velocity
               - frictionloss * tanh(ten_velocity * friction_smoothing)

qfrc_passive[dof] += coef * ten_force    # for each wrap entry
```
Note: tendon spring/damper forces are skipped in implicit mode (non-diagonal
coupling cannot be absorbed into the diagonal implicit modification).

### 4.4 Constraint Forces (`mj_fwd_constraint`)

Four constraint types, applied in order:

1. **Joint limits** — penalty-based with Baumgarte stabilization
2. **Tendon limits** — penalty-based (same as joint limits, mapped through J^T)
3. **Equality constraints** — penalty-based with Baumgarte stabilization
4. **Contact forces** — PGS solver with friction cones

### 4.5 Joint Limits

Penalty method with configurable stiffness derived from `solref`/`solimp`.

When joint position violates a limit (`q < qmin` or `q > qmax`):
```python
violation = qmin - q   # or q - qmax

# Stiffness/damping from solref [timeconst, dampratio]
# Note: solref_to_penalty takes (solref, default_k, default_b, dt), not effective_mass
k, b = solref_to_penalty(solref, default_k, default_b, dt)

# Constraint force (no impedance scaling — unlike contacts and equality constraints)
qfrc_constraint[dof] += k * violation + b * violation_velocity
```

> **Implementation note:** Joint limits do NOT use `compute_impedance()`. The
> impedance sigmoid is applied for contacts and equality constraints, but joint
> limits use a direct penalty without impedance modulation.

### 4.6 Equality Constraints

All equality constraints use penalty enforcement:

| Type | DOF removed | Description |
|------|-------------|-------------|
| Connect | 3 | Two body points coincide (translation only) |
| Weld | 6 | Two body frames identical (translation + rotation) |
| Joint | varies | Polynomial coupling: `q2 = c0 + c1*q1 + c2*q1^2 + ...` |
| Tendon | varies | Polynomial coupling between tendon lengths (**not yet implemented** — ignored with warning) |
| Distance | 1 | Fixed distance between two geom centers |

Stiffness and damping derive from per-constraint `solref`/`solimp` parameters,
identical to the joint limit mechanism.

### 4.7 Contact Solver (PGS)

**The optimization problem:**
```
minimize:   (1/2) lambda^T (A + R) lambda + lambda^T b
subject to: lambda in Omega
```

Where:
- `A = J M^-1 J^T` (constraint-space inverse inertia)
- `R = diag(regularization)` (CFM, derived from impedance)
- `b = J qacc_smooth + aref` (velocity error + Baumgarte stabilization)
- `aref` incorporates ERP derived from per-contact `solref`

**Contact Jacobian construction:**

Each contact produces 3 constraint rows (normal + 2 tangent directions).
The Jacobian maps joint velocities to contact-frame relative velocity:

```python
for each body in kinematic chain from contact to root:
    r = contact_point - body_position
    for each joint on body:
        if Hinge: J_col = axis x r          (angular)
        if Slide: J_col = axis              (linear)
        if Ball:  J_col = basis vectors x r  (3 angular DOFs)
        if Free:  J_lin = I, J_ang = [.] x r (6 DOFs)
```

Signs are positive for body1 and negative for body2.

**PGS iteration:**
```python
H = A + R
H_diag_inv = 1.0 / diag(H)
lambda = warmstart   # from previous timestep via contact correspondence

for iteration in range(max_iter):
    max_delta = 0

    for c in range(n_contacts):        # iterate per-contact (3 rows each)
        base = c * 3
        # Gauss-Seidel update for all 3 rows (normal + 2 friction)
        for j in [0, 1, 2]:
            residual = H[base+j,:] @ lambda + b[base+j]
            lambda[base+j] -= residual * H_diag_inv[base+j]

        # Project: normal >= 0, then rescale 2D friction vector
        lambda[base] = max(0, lambda[base])
        friction_mag = sqrt(lambda[base+1]^2 + lambda[base+2]^2)
        max_friction = mu * lambda[base]
        if friction_mag > max_friction:
            scale = max_friction / friction_mag
            lambda[base+1] *= scale
            lambda[base+2] *= scale

        max_delta = max(max_delta, ...)

    if max_delta < tolerance:
        break
```

**Projection rules:**

| Constraint | Projection |
|------------|------------|
| Contact normal | `max(0, lambda)` |
| Contact friction | Rescale 2D friction vector: if `\|lambda_t\| > mu * lambda_n`, scale to `mu * lambda_n` (circular cone, no per-axis clamp) |

**Warmstart:** Contact forces are cached by canonical geom pair
`(min(g1,g2), max(g1,g2))`. When the same pair reappears next frame,
previous lambda values seed the iteration.

### 4.8 Forward Acceleration

**Explicit path** (Euler and RK4 — RK4 re-evaluates this at each of stages 1–3 via `forward_skip_sensors()`):
```
qfrc_total = qfrc_applied + qfrc_actuator + qfrc_passive + qfrc_constraint - qfrc_bias
qacc = M^-1 @ qfrc_total
```

Solved via `mj_solve_sparse` using the sparse L^T D L factorization from CRBA.
The solve applies L^T, D, then L in three passes — each O(nv) for tree-sparse L.

**Implicit path:**

Solves the implicit system for new velocity directly:
```
(M + h*D + h^2*K) v_new = M*v_old + h*f_ext - h*K*(q - q_eq)
```

Where:
- `D = diag(damping)` per joint DOF
- `K = diag(stiffness)` per joint DOF
- `q_eq = springref` per joint
- `f_ext = applied + actuator + passive(friction only) + constraint - bias`

The modified matrix `M + h*D + h^2*K` is SPD when `M` is SPD and `D, K >= 0`.
After solving, `qacc = (v_new - v_old) / h` for consistency.

---

## Stage 5: Integration

**Semi-implicit Euler (default, `Integrator::Euler`):**
```python
# 1. Update velocity FIRST using current acceleration
qvel += qacc * h

# 2. Update position using NEW velocity (semi-implicit)
for each joint:
    if Hinge or Slide:
        qpos += qvel * h
    if Ball:
        # Quaternion integration on SO(3) via exponential map
        qpos = qpos * quat_exp(omega * h / 2)
    if Free:
        qpos[0:3] += qvel[0:3] * h          # linear
        qpos[3:7] = quat_integrate(qvel[3:6], h)  # angular

# 3. Normalize quaternions to prevent drift
```

Position update uses the NEW velocity (step 1 output). This is what makes it
semi-implicit and provides energy stability that plain Euler lacks.

**Implicit (`Integrator::ImplicitSpringDamper`):**

Velocity was already updated in `mj_fwd_acceleration_implicit`. Integration
only updates positions using the new velocity, identical to step 2 above.

**Runge-Kutta 4 (`Integrator::RungeKutta4`):**

True 4-stage RK4 via `mj_runge_kutta()`. Does not call `integrate()` at all.
Uses Butcher tableau weights `[1/6, 1/3, 1/3, 1/6]`.

```python
# Stage 0: use results from initial forward() call in step()
rk4_qvel[0] = qvel                     # initial velocity
rk4_qacc[0] = qacc                     # initial acceleration
qpos_saved = qpos.copy()
efc_lambda_saved = efc_lambda.copy()

# Stages 1, 2, 3: re-evaluate dynamics at intermediate points
A = [[0.5, 0, 0], [0, 0.5, 0], [0, 0, 1.0]]
for i in [1, 2, 3]:
    # Weighted velocity/acceleration from all previous stages (A row selects)
    dX_vel = sum(A[i-1][j] * rk4_qvel[j] for j in 0..3)
    dX_acc = sum(A[i-1][j] * rk4_qacc[j] for j in 0..3)

    # Advance state from saved initial position
    qpos = mj_integrate_pos_explicit(qpos_saved, dX_vel, h)
    rk4_qvel[i] = rk4_qvel[0] + h * dX_acc   # velocity at this stage
    qvel = rk4_qvel[i]                        # set Data.qvel

    forward_skip_sensors()               # full pipeline minus sensors
    rk4_qacc[i] = qacc                   # acceleration from dynamics

# Weighted combination: B = [1/6, 1/3, 1/3, 1/6]
qvel = rk4_qvel[0] + h * sum(B[i] * rk4_qacc[i] for i in 0..4)
qpos = mj_integrate_pos_explicit(qpos_saved, sum(B[i] * rk4_qvel[i]), h)
efc_lambda = efc_lambda_saved            # restore warmstart
```

Key details:
- Stage 0 uses the results from the initial `forward()` call in `step()`;
  only stages 1–3 call `forward_skip_sensors()`
- `forward_skip_sensors()` is identical to `forward()` but omits sensor
  evaluation (matching MuJoCo's `mj_forwardSkip(m, d, mjSTAGE_NONE, 1)`)
- Position integration uses `mj_integrate_pos_explicit()` which handles
  quaternion exponential map for ball/free joints
- `efc_lambda` (warmstart cache) is saved before RK4 and restored afterward
  so intermediate stage contact solutions don't corrupt the warmstart

---

## Key Data Structures

### Model (static, immutable after loading)

| Field | Type | Description |
|-------|------|-------------|
| `nq, nv` | `usize` | Position coordinates, velocity DOFs |
| `nbody, njnt, ngeom` | `usize` | Body, joint, geometry counts |
| `body_parent[i]` | `usize` | Parent body index |
| `body_pos[i], body_quat[i]` | `Vector3, UnitQuat` | Local frame offset from parent |
| `body_mass[i], body_inertia[i]` | `f64, Vector3` | Mass properties |
| `body_subtreemass[i]` | `f64` | Precomputed subtree mass |
| `jnt_type[i]` | `MjJointType` | Hinge, Slide, Ball, or Free |
| `jnt_axis[i], jnt_pos[i]` | `Vector3` | Joint axis and anchor in body frame |
| `jnt_stiffness[i], jnt_damping[i]` | `f64` | Spring-damper parameters |
| `jnt_springref[i]` | `f64` | Spring equilibrium position |
| `jnt_solref[i]` | `[f64; 2]` | [timeconst, dampratio] for limits |
| `jnt_solimp[i]` | `[f64; 5]` | [d0, d_width, width, midpoint, power] |
| `timestep` | `f64` | Simulation step size |
| `gravity` | `Vector3` | Gravity vector |
| `integrator` | `Integrator` | Euler, RungeKutta4, or ImplicitSpringDamper |
| `solver_iterations` | `usize` | Max PGS iterations (default 100) |
| `solver_tolerance` | `f64` | PGS convergence threshold (default 1e-8) |

### Data (dynamic, mutable, pre-allocated)

**State (source of truth):**

| Field | Type | Description |
|-------|------|-------------|
| `qpos[nq]` | `DVector` | Joint positions |
| `qvel[nv]` | `DVector` | Joint velocities |
| `time` | `f64` | Simulation time |

**Computed quantities:**

| Field | Type | Description |
|-------|------|-------------|
| `xpos[nbody]` | `Vec<Vector3>` | World-frame body positions |
| `xquat[nbody]` | `Vec<UnitQuat>` | World-frame body orientations |
| `qacc[nv]` | `DVector` | Joint accelerations |
| `qM[nv,nv]` | `DMatrix` | Mass matrix from CRBA |
| `qfrc_bias[nv]` | `DVector` | Gravity + Coriolis + centrifugal |
| `qfrc_passive[nv]` | `DVector` | Springs + dampers + friction loss |
| `qfrc_actuator[nv]` | `DVector` | Actuator forces |
| `qfrc_applied[nv]` | `DVector` | User-applied forces |
| `qfrc_constraint[nv]` | `DVector` | Contact + limit + equality forces |
| `contacts[ncon]` | `Vec<Contact>` | Active contact points |
| `efc_lambda` | `HashMap` | Warmstart cache (keyed by geom pair) |

---

## Solver Parameters

### solref — Solver Reference Parameters

`[timeconst, dampratio]` — controls constraint stiffness and damping.

- `timeconst` (default 0.02): Response time in seconds. Lower = stiffer.
- `dampratio` (default 1.0): Damping ratio. 1.0 = critically damped.

Converted to penalty stiffness `k` and damping `b` via `k = 1/timeconst²`,
`b = 2*dampratio/timeconst`, with a stability clamp `k ≤ 1/dt²`.
(MuJoCo scales by effective mass; our implementation uses unit mass.)

### solimp — Solver Impedance Parameters

`[d0, d_width, width, midpoint, power]` — controls force scaling with violation.

```python
x = clamp(violation / width, 0, 1)
y = sigmoid(x, power, midpoint)     # smooth power curve
impedance = d0 + y * (d_width - d0)
```

Defaults: `[0.9, 0.95, 0.001, 0.5, 2.0]`

This provides soft initial contact (impedance = 0.9) that stiffens as
penetration increases (impedance -> 0.95), over a transition zone of 0.001 m.

---

## References

- [MuJoCo Computation](https://mujoco.readthedocs.io/en/stable/computation/index.html)
- [MuJoCo Source — engine_forward.c](https://github.com/google-deepmind/mujoco/blob/main/src/engine/engine_forward.c)
- [MuJoCo Source — engine_solver.c](https://github.com/google-deepmind/mujoco/blob/main/src/engine/engine_solver.c)
- Featherstone, R. (2008). *Rigid Body Dynamics Algorithms*. Springer.
- Todorov, E. (2014). "Convex and analytically-invertible dynamics with contacts and constraints."
