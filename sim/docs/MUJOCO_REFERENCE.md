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
     [sleep] mj_wake            — Wake sleeping bodies (user forces, contacts, tendons, equality)
             mj_sleep            — Sleep state machine (countdown → sleep transition)
             mj_island           — Island discovery (DFS flood-fill over constraints)
     a. mj_fwd_position    — Forward kinematics (skips sleeping bodies)
        mj_fwd_tendon       — Tendon lengths + Jacobians (fixed tendons)
        mj_collision        — Broad/narrow phase collision detection (skips sleeping pairs)
        mj_transmission_body_dispatch — Body transmission moment arms (§36, requires contacts)
     b. mj_fwd_velocity    — Body + tendon velocities (skips sleeping DOFs)
        mj_actuator_length  — Actuator length/velocity from transmission state
     c. mj_fwd_actuation   — Activation dynamics (act_dot) + gain/bias force + clamping
     d. mj_crba            — Mass matrix (selective CRBA, skips sleeping subtrees)
     e. mj_rne             — Bias forces (Recursive Newton-Euler)
     f. mj_fwd_passive     — Spring, damper, friction loss forces (skips sleeping DOFs)
     g. mj_fwd_constraint  — Joint/tendon limits, equality, contact PGS
        mj_fwd_constraint_islands — Per-island block-diagonal solving (when islands > 1)
     h. mj_fwd_acceleration — Solve M*qacc = f (explicit) or implicit velocity update
  2a. integrate()          — Activation integration + semi-implicit Euler or implicit
                              (for Euler / ImplicitSpringDamper integrators)
                              (skips sleeping joints for position/velocity integration)
  2b. mj_runge_kutta()     — True 4-stage RK4 with Butcher tableau, including
                              activation state (for RungeKutta4 integrator)
                              (sleep disabled for RK4; warning emitted)
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

Two-mechanism architecture:

**Mechanism 1 (automatic pipeline):**
- Broad phase: sweep-and-prune on AABBs
- Filters (in order): body-pair excludes (`contact_excludes`), explicit pair-set
  suppression (`contact_pair_set`), same-body, parent-child,
  contype/conaffinity bitmasks
- Narrow phase: per-pair geometry tests

**Mechanism 2 (explicit `<pair>` pipeline):**
- Iterates `contact_pairs` (from `<contact><pair>`)
- Bypasses all kinematic and bitmask filters
- Bounding-sphere distance cull (rbound + margin)
- Narrow phase via `collide_geoms`
- `apply_pair_overrides` applies per-pair condim/friction/solref/solimp

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

## Stage 2b: Actuator Length/Velocity (`mj_actuator_length`)

Computes actuator-space length and velocity from transmission state.
Called after `mj_fwd_velocity()` (which populates `ten_velocity`).

```python
for i in range(nu):
    gear = actuator_gear[i]
    if transmission[i] == Joint:
        jid = actuator_trnid[i]
        if jnt_type[jid] in (Hinge, Slide):   # scalar DOF only
            actuator_length[i] = gear * qpos[qpos_adr[jid]]
            actuator_velocity[i] = gear * qvel[dof_adr[jid]]
    elif transmission[i] == Tendon:
        tid = actuator_trnid[i]
        actuator_length[i] = gear * ten_length[tid]
        actuator_velocity[i] = gear * ten_velocity[tid]
    elif transmission[i] == Site:
        (sid0, sid1) = actuator_trnid[i]    # site pair
        actuator_length[i] = mj_transmission_site(model, data, i, gear)
        actuator_velocity[i] = gear . (J @ qvel)  # via site Jacobian
    elif transmission[i] == Body:
        # Length/moment already set by mj_transmission_body (runs after collision)
        actuator_velocity[i] = actuator_moment[i] . qvel
```

---

## Stage 3: Forward Actuation (`mj_fwd_actuation`)

Three-phase function: computes activation derivatives, actuator forces, and
generalized forces. Does NOT integrate activation — that is the integrator's
responsibility (matching MuJoCo's `mjData.act_dot` architecture).

### Phase 1: Activation Dynamics (`act_dot`)

For each actuator, compute the time-derivative of activation state without
modifying `data.act`:

```python
for i in range(nu):
    ctrl_clamped = clamp(ctrl[i], ctrlrange[i])   # enforce ctrllimited

    if dyntype[i] == None:
        input = ctrl_clamped          # direct passthrough, no activation state

    elif dyntype[i] == Muscle:
        act_dot[act_adr[i]] = muscle_activation_dynamics(ctrl_clamped, act[act_adr[i]], dynprm[i])
        input = act[act_adr[i]]       # use CURRENT activation for force

    elif dyntype[i] in (Filter, FilterExact):
        act_dot[act_adr[i]] = (ctrl_clamped - act[act_adr[i]]) / max(tau, 1e-10)
        input = act[act_adr[i]]
        # Filter uses Euler integration, FilterExact uses exact discrete integration.
        # Both compute the same act_dot here; the difference is in integrate().

    elif dyntype[i] == Integrator:
        act_dot[act_adr[i]] = ctrl_clamped
        input = act[act_adr[i]]
```

**Muscle activation dynamics** (Millard et al. 2013) with activation-dependent
time constants:

```python
def muscle_activation_dynamics(ctrl, act, dynprm):
    ctrl_c = clamp(ctrl, 0, 1)
    act_c  = clamp(act, 0, 1)

    tau_act   = dynprm[0] * (0.5 + 1.5 * act_c)    # slower at high activation
    tau_deact = dynprm[1] / (0.5 + 1.5 * act_c)    # faster at high activation
    tausmooth = dynprm[2]

    dctrl = ctrl_c - act

    if tausmooth < 1e-10:
        tau = tau_act if dctrl > 0 else tau_deact    # hard switch
    else:
        tau = tau_deact + (tau_act - tau_deact) * sigmoid(dctrl / tausmooth + 0.5)

    return dctrl / max(tau, 1e-10)
```

Where `sigmoid(x) = x³(6x² - 15x + 10)` is the quintic C2-continuous
smoothstep matching MuJoCo's `mju_sigmoid`.

Default `dynprm` for muscles: `[tau_act=0.01, tau_deact=0.04, tausmooth=0.0]`.

### Phase 2: Force Generation (Gain/Bias)

```python
    # Gain dispatch (on gaintype, not dyntype)
    if gaintype[i] == Fixed:
        gain = gainprm[i][0]
    elif gaintype[i] == Affine:
        gain = gainprm[i][0] + gainprm[i][1]*length[i] + gainprm[i][2]*velocity[i]
    elif gaintype[i] == Muscle:
        prm = gainprm[i]
        L0 = (lengthrange[i][1] - lengthrange[i][0]) / max(prm[1] - prm[0], 1e-10)
        norm_len = prm[0] + (actuator_length[i] - lengthrange[i][0]) / max(L0, 1e-10)
        norm_vel = actuator_velocity[i] / max(L0 * prm[6], 1e-10)  # prm[6] = vmax

        FL = muscle_gain_length(norm_len, lmin=prm[4], lmax=prm[5])
        FV = muscle_gain_velocity(norm_vel, fvmax=prm[8])
        gain = -F0 * FL * FV

    # Bias dispatch (on biastype, not dyntype)
    if biastype[i] == None:
        bias = 0.0
    elif biastype[i] == Affine:
        bias = biasprm[i][0] + biasprm[i][1]*length[i] + biasprm[i][2]*velocity[i]
    elif biastype[i] == Muscle:
        FP = muscle_passive_force(norm_len, lmax=prm[5], fpmax=prm[7])
        bias = -F0 * FP

    force = gain * input + bias   # unified for all types
```

**Force-Length curve** (`muscle_gain_length`): Piecewise-quadratic bump.
Peak 1.0 at `L = 1.0`, zero at `lmin` and `lmax`. Four segments with
midpoints `a = (lmin+1)/2`, `b = (1+lmax)/2`.

**Force-Velocity curve** (`muscle_gain_velocity`): Piecewise-quadratic.
Zero at `V = -1` (max shortening), 1.0 at `V = 0` (isometric), plateau
at `fvmax` for `V ≥ fvmax - 1` (eccentric).

**Passive Force** (`muscle_passive_force`): Zero below `L = 1.0`.
Quadratic onset from 1.0 to midpoint `b = (1+lmax)/2`, then linear
with `fpmax * 0.5` at midpoint.

These match MuJoCo's `mju_muscleGain` and `mju_muscleBias` in
`engine_util_misc.c` exactly.

**F0 (peak isometric force):** Auto-computed at model build time when
`gainprm[2] < 0`: `F0 = scale / acc0`, where `acc0 = ‖M⁻¹J‖₂` is the
acceleration produced by unit actuator force, computed via the sparse
L^T D L factorization from CRBA at `qpos0`.

### Phase 3: Transmission (Force → Generalized Forces)

```python
    force = clamp(force, forcerange[i])   # enforce forcelimited
    actuator_force[i] = force

    gear = actuator_gear[i]
    if transmission[i] == Joint:
        qfrc_actuator[dof_adr] += gear * force
    elif transmission[i] == Tendon:
        for w in tendon_wrap_range:
            qfrc_actuator[wrap_objid[w]] += gear * wrap_prm[w] * force  # J^T
    elif transmission[i] == Site:
        J = mj_jac_site(model, data, site_id)   # 6×nv Jacobian
        qfrc_actuator += gear . J^T * force      # 6D wrench projection
    elif transmission[i] == Body:
        for dof in range(nv):
            if actuator_moment[i][dof] != 0:
                qfrc_actuator[dof] += actuator_moment[i][dof] * force
```

### Model Build: `compute_muscle_params()`

Called after `compute_ancestors()` and `compute_implicit_params()`, for each
muscle actuator:

1. **`actuator_lengthrange`** — computed from tendon limits (if limited) or
   joint range sums (fixed tendons), gear-scaled with sign awareness.
2. **`actuator_acc0`** — forward pass at `qpos0` → CRBA → sparse solve:
   `acc0 = ‖M⁻¹J‖₂` where J is the gear-scaled transmission moment vector.
3. **F0 resolution** — when `gainprm[2] < 0` (auto-compute):
   `F0 = gainprm[3] / acc0` (scale / acc0). `biasprm[2]` synced to match.

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
factorization vs O(nv³) dense Cholesky. Uses flat CSR storage matching
MuJoCo's `mj_factorI`: immutable sparsity metadata in Model
(`qLD_rowadr`, `qLD_rownnz`, `qLD_colind`) and factored values in Data
(`qLD_data` with unified diagonal layout, `qLD_diag_inv` for precomputed
reciprocals).

**Unified diagonal layout:** The diagonal element D[i,i] is stored as the
last element of each CSR row: `qLD_data[rowadr[i] + rownnz[i] - 1]`.
Off-diagonal L entries occupy `qLD_data[rowadr[i]..rowadr[i]+rownnz[i]-1]`.
During factorization, `qLD_diag_inv[i] = 1.0 / D[i,i]` is precomputed so
that the solve phase uses multiply (`x[i] *= diag_inv[i]`) instead of
divide, avoiding per-DOF division in the hot path.

The inner loop uses bulk row updates (MuJoCo's `mju_addToScl` pattern)
instead of per-element column search, exploiting the ancestor row superset
property: if DOF `j` is ancestor of DOF `i`, then `ancestors(j) ⊆ ancestors(i)`
and `row_i[0..nnz_j]` has identical column indices to `row_j[0..nnz_j]`.

When sleep is enabled with sleeping trees, `mj_factor_sparse_selective`
dispatches to partial factorization that skips sleeping DOFs entirely
(C3b, §16.29.5). Tree independence guarantees no cross-contamination.

```python
# CSR metadata (immutable, computed once at model build):
# rowadr[i] = start address of row i in qLD_data
# rownnz[i] = total entries in row i (off-diagonal L entries + 1 diagonal)
# colind[rowadr[i]..rowadr[i]+rownnz[i]] = ancestor column indices
#
# Unified diagonal layout:
#   off-diagonal L entries: qLD_data[rowadr[i] .. rowadr[i]+rownnz[i]-1]
#   diagonal D[i,i]:        qLD_data[rowadr[i] + rownnz[i] - 1]

# Phase 1: Copy M's sparse entries into qLD working storage
for i in range(nv):
    # Off-diagonal entries
    for k in range(rownnz[i] - 1):
        qLD_data[rowadr[i]+k] = M[i, colind[rowadr[i]+k]]
    # Diagonal as last element
    qLD_data[rowadr[i] + rownnz[i] - 1] = M[i,i]

# Phase 2: Eliminate from leaves to root (bulk row update)
for i in range(nv-1, -1, -1):
    diag_i = rowadr[i] + rownnz[i] - 1       # index of D[i,i] in qLD_data
    di = qLD_data[diag_i]
    inv_di = 1.0 / di
    diag_inv[i] = inv_di                       # precompute for solve phase
    n_offdiag = rownnz[i] - 1                  # number of off-diagonal entries
    for k in range(n_offdiag):
        qLD_data[rowadr[i]+k] *= inv_di        # Normalize L entries
    for a in range(n_offdiag-1, -1, -1):       # deep-to-shallow
        j = colind[rowadr[i]+a]
        lij = qLD_data[rowadr[i]+a]
        diag_j = rowadr[j] + rownnz[j] - 1
        qLD_data[diag_j] -= lij * lij * di     # Update D[j,j]
        # Bulk update: row_j[0..n_offdiag_j] += scale * row_i[0..a]
        # (ancestor superset property guarantees aligned columns)
        scale = -lij * di
        for k in range(a):
            qLD_data[rowadr[j]+k] += scale * qLD_data[rowadr[i]+k]
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
# Deadband spring (lengthspring is a [lower, upper] pair):
if ten_length > upper:    frc_spring = stiffness * (upper - ten_length)
elif ten_length < lower:  frc_spring = stiffness * (lower - ten_length)
else:                     frc_spring = 0    # deadband region

# When lower == upper, this reduces to classical: F = -k * (L - ref)

ten_force[t] = frc_spring
               - damping * ten_velocity
               - frictionloss * tanh(ten_velocity * friction_smoothing)

qfrc_passive[dof] += coef * ten_force    # for each wrap entry
```
Note: tendon spring/damper forces are skipped in implicit mode (non-diagonal
coupling cannot be absorbed into the diagonal implicit modification).

### 4.4 Constraint Forces (`mj_fwd_constraint`)

Four constraint types, applied in order:

1. **Joint limits** — penalty-based with Baumgarte stabilization (PGS/CG path);
   solver-based constraint rows (Newton path)
2. **Tendon limits** — same as joint limits, mapped through J^T
3. **Equality constraints** — penalty-based with Baumgarte stabilization (PGS/CG
   path); solver-based constraint rows (Newton path)
4. **Contact forces** — PGS/CG/Newton solver with friction cones

> **⚠️ Divergence:** MuJoCo uses solver-based constraint rows for ALL four types
> in ALL solver modes. CortenForge's PGS/CG penalty path for types 1–3 is a
> known divergence. Migration to unified solver-based constraints is tracked as
> [#30](todo/future_work_8.md) (PGS/CG Unified Constraints).

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
lambda = warmstart   # from previous timestep via WarmstartKey correspondence

for iteration in range(max_iter):
    max_delta = 0

    for c in range(n_contacts):
        base = efc_offsets[c]           # variable offset per contact
        dim = contacts[c].dim           # 1, 3, 4, or 6
        mu = contacts[c].mu             # [sliding1, sliding2, torsional, rolling1, rolling2]

        # Gauss-Seidel update for all rows of this contact
        for j in range(dim):
            residual = H[base+j,:] @ lambda + b[base+j]
            lambda[base+j] -= residual * H_diag_inv[base+j]

        # Project onto elliptic friction cone (two-step)
        # Step 1: Unilateral constraint — release if separating
        if lambda[base] < 0:
            lambda[base:base+dim] = 0
            continue

        # Step 2: Scale friction to cone boundary if exceeded
        # s = sqrt(sum((lambda[i]/mu[i-1])^2 for i in 1..dim))
        # if s > lambda[base]: scale all friction by lambda[base]/s

        max_delta = max(max_delta, ...)

    if max_delta < tolerance:
        break
```

**Projection rules:**

| Constraint | Projection |
|------------|------------|
| Contact normal (condim ≥ 1) | `max(0, lambda_n)` — unilateral constraint, release if separating |
| Contact friction (condim ≥ 3) | Elliptic cone: if `‖(λ_i/μ_i)‖ > λ_n`, scale friction to boundary |
| Torsional friction (condim ≥ 4) | Included in elliptic cone projection with `mu[2]` |
| Rolling friction (condim = 6) | Included in elliptic cone projection with `mu[3..5]` |

**Variable condim:** Contact dimension determines constraint size:
- condim 1: Normal only (frictionless) — 1 constraint row
- condim 3: Normal + 2D tangential — 3 constraint rows
- condim 4: + torsional friction — 4 constraint rows
- condim 6: + rolling friction — 6 constraint rows

**Warmstart:** Contact forces are cached by `WarmstartKey` combining canonical geom pair
`(min(g1,g2), max(g1,g2))` with 1cm spatial grid cell. Stored as `Vec<f64>` (variable length
for different condim). When the same key reappears next frame, previous lambda values seed
the iteration (with dimension check).

### 4.8 Forward Acceleration

**Explicit path** (Euler and RK4 — RK4 re-evaluates this at each of stages 1–3 via `forward_skip_sensors()`):
```
qfrc_total = qfrc_applied + qfrc_actuator + qfrc_passive + qfrc_constraint - qfrc_bias
qacc = M^-1 @ qfrc_total
```

Solved via `mj_solve_sparse` using the sparse L^T D L factorization from CRBA.
The solve applies L^T, D^{-1}, then L in three passes — each O(nv) for tree-sparse L.
The D^{-1} pass uses the precomputed `diag_inv[i]` via multiply (`x[i] *= diag_inv[i]`)
rather than divide, since the reciprocals were cached during factorization.

```python
# mj_solve_sparse(rowadr, rownnz, colind, qLD_data, diag_inv, x):
#
# Phase 1: L^T solve (forward, i = nv-1 down to 0)
for i in range(nv-1, -1, -1):
    n_offdiag = rownnz[i] - 1
    for k in range(n_offdiag):
        j = colind[rowadr[i]+k]
        x[j] -= qLD_data[rowadr[i]+k] * x[i]

# Phase 2: D^{-1} solve (multiply by precomputed reciprocal)
for i in range(nv):
    x[i] *= diag_inv[i]       # NOT x[i] /= D[i] — division precomputed

# Phase 3: L solve (backward, i = 0 up to nv-1)
for i in range(nv):
    n_offdiag = rownnz[i] - 1
    for k in range(n_offdiag):
        j = colind[rowadr[i]+k]
        x[i] -= qLD_data[rowadr[i]+k] * x[j]
```

**Implicit path (ImplicitSpringDamper — legacy diagonal):**

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

**Implicit path (ImplicitFast — full Jacobian, Cholesky):**

Assembles the full velocity-derivative Jacobian `D = qDeriv = ∂(qfrc_smooth)/∂(qvel)` via:
1. `mjd_passive_vel()` — DOF damping + tendon damping J^T B J
2. `mjd_actuator_vel()` — actuator velocity derivatives (Affine gain/bias)

Symmetrizes D, then solves `(M − h·D) · qacc = f` via dense Cholesky factorization.
Skips Coriolis derivatives (`mjd_rne_vel`). Returns `StepError::CholeskyFailed` if
`M − h·D` is not positive definite (e.g., strong positive velocity feedback).

**Implicit path (Implicit — full Jacobian, LU):**

Same as ImplicitFast but also includes Coriolis velocity derivatives via `mjd_rne_vel()`.
Does NOT symmetrize D (Coriolis terms break symmetry). Uses LU factorization with
partial pivoting instead of Cholesky. Returns `StepError::LuSingular` if any pivot
magnitude is below `1e-30`.

Both ImplicitFast and Implicit compute `qacc` (not `v_new` directly), then velocity
is updated in the integration step via `qvel += h * qacc`, matching MuJoCo's approach.

---

## Stage 5: Integration

All integrators integrate activation state before velocity/position.
Activation uses `act_dot` (computed by `mj_fwd_actuation()` without modifying
`act`), matching MuJoCo's order: **activation → velocity → position**.

**Semi-implicit Euler (default, `Integrator::Euler`):**
```python
# 0. Integrate activation (per actuator, then clamp muscles)
for i in range(nu):
    for k in range(actuator_act_num[i]):
        j = act_adr[i] + k
        if dyntype[i] == FilterExact:
            tau = max(dynprm[i][0], 1e-10)
            act[j] += act_dot[j] * tau * (1 - exp(-h / tau))   # exact discrete
        else:
            act[j] += h * act_dot[j]                            # Euler
    if dyntype[i] == Muscle:
        act[act_adr[i]:act_adr[i]+act_num[i]] = clamp(act[...], 0, 1)

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

Muscle activations are clamped to `[0, 1]` after integration to enforce the
physiological range.

**GPU path (`sim-gpu`, Phase 10a):** When using `GpuBatchSim`, step 1
(`qvel += qacc * h`) is replaced by a wgpu compute shader dispatch operating
on all environments in parallel. Steps 0 (activation), 2 (position), and 3
(quaternion normalization) remain on CPU via `Data::integrate_without_velocity()`.
See [future_work_3 #10](../todo/future_work_3.md).

**ImplicitSpringDamper (`Integrator::ImplicitSpringDamper`):**

Activation integration is identical to Euler (step 0 above). Velocity was
already updated in `mj_fwd_acceleration_implicit`. Integration only updates
positions using the new velocity, identical to step 2 above.

**ImplicitFast / Implicit (`Integrator::ImplicitFast`, `Integrator::Implicit`):**

Activation integration is identical to Euler (step 0 above). Velocity is
updated in the integration step via `qvel += h * qacc` (same as Euler).
Position update uses the new velocity, identical to step 2 above.

**Runge-Kutta 4 (`Integrator::RungeKutta4`):**

True 4-stage RK4 via `mj_runge_kutta()`. Does not call `integrate()` at all.
Uses Butcher tableau weights `[1/6, 1/3, 1/3, 1/6]`. Integrates `act`
alongside `qpos`/`qvel` using the same RK4 weights.

```python
# Stage 0: use results from initial forward() call in step()
rk4_qvel[0] = qvel                     # initial velocity
rk4_qacc[0] = qacc                     # initial acceleration
qpos_saved = qpos.copy()
act_saved = act.copy()                  # save activation state
rk4_act_dot[0] = act_dot.copy()        # collect stage-0 derivatives
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

    # Advance activation trial state from saved initial activation
    for each actuator act_i:
        for each activation slot a owned by act_i:
            weighted = sum(A[i-1][j] * rk4_act_dot[j][a] for j in 0..3)
            if dyntype[act_i] == FilterExact:
                tau = max(dynprm[act_i][0], 1e-10)
                act[a] = act_saved[a] + weighted * tau * (1 - exp(-h / tau))
            else:
                act[a] = act_saved[a] + h * weighted
    clamp_muscle_activations(act, 0, 1)

    forward_skip_sensors()               # full pipeline minus sensors
    rk4_qacc[i] = qacc                   # acceleration from dynamics
    rk4_act_dot[i] = act_dot.copy()      # collect activation derivatives

# Weighted combination: B = [1/6, 1/3, 1/3, 1/6]
qvel = rk4_qvel[0] + h * sum(B[i] * rk4_qacc[i] for i in 0..4)
qpos = mj_integrate_pos_explicit(qpos_saved, sum(B[i] * rk4_qvel[i]), h)

# Activation final combination
for each actuator act_i:
    for each activation slot a owned by act_i:
        combined = sum(B[j] * rk4_act_dot[j][a] for j in 0..4)
        if dyntype[act_i] == FilterExact:
            tau = max(dynprm[act_i][0], 1e-10)
            act[a] = act_saved[a] + combined * tau * (1 - exp(-h / tau))
        else:
            act[a] = act_saved[a] + h * combined
clamp_muscle_activations(act, 0, 1)

efc_lambda = efc_lambda_saved            # restore warmstart
```

Key details:
- Stage 0 uses the results from the initial `forward()` call in `step()`;
  only stages 1–3 call `forward_skip_sensors()`
- `forward_skip_sensors()` is identical to `forward()` but omits sensor
  evaluation (matching MuJoCo's `mj_forwardSkip(m, d, mjSTAGE_NONE, 1)`)
- Position integration uses `mj_integrate_pos_explicit()` which handles
  quaternion exponential map for ball/free joints
- Activation is integrated with the same Butcher tableau as `qpos`/`qvel`,
  reconstructed from `act_saved` at each trial stage (never accumulated)
- Muscle activations clamped to `[0, 1]` at each trial stage and final
- `efc_lambda` (warmstart cache) is saved before RK4 and restored afterward
  so intermediate stage contact solutions don't corrupt the warmstart
- All RK4 activation buffers (`rk4_act_saved`, `rk4_act_dot`) are pre-allocated
  Data fields to avoid per-step heap allocation

---

## Stage 6: Derivatives (optional, after `forward()`)

Implemented in `sim-core/src/derivatives.rs` (~1685 lines). Four modes:

### 6.1 Pure Finite-Difference: `mjd_transition_fd()`

Linearizes the transition `x_{t+1} = f(x_t, u_t)` around the current state.

```
state x = [dq (nv tangent), qvel (nv), act (na)]    dim = 2*nv + na
control u = ctrl                                      dim = nu

centered FD (O(ε²) error):
  for each state dimension i:
    x⁺ = step(x + ε·eᵢ)
    x⁻ = step(x − ε·eᵢ)
    A[:, i] = (x⁺ − x⁻) / (2·ε)
  for each control dimension j:
    x⁺ = step(x, u + ε·eⱼ)
    x⁻ = step(x, u − ε·eⱼ)
    B[:, j] = (x⁺ − x⁻) / (2·ε)

Position perturbations: mj_integrate_pos_explicit() (tangent → coordinate)
Position differences:   mj_differentiate_pos()      (coordinate → tangent)
```

Cost: `2·(2·nv + na + nu)` step() calls (centered). Handles any integrator
including RK4. Captures contact transitions naturally.

### 6.2 Analytical Velocity Derivatives: `mjd_smooth_vel()`

Computes `∂(qfrc_smooth)/∂qvel` analytically, stored in `Data.qDeriv`:

```
qfrc_smooth = qfrc_passive + qfrc_actuator − qfrc_bias

qDeriv = ∂(passive)/∂v + ∂(actuator)/∂v − ∂(bias)/∂v

mjd_passive_vel():   diagonal −damping[i] + tendon rank-1 −b·J^T·J
mjd_actuator_vel():  affine gain/bias velocity terms via transmission
mjd_rne_vel():       chain-rule derivative propagation through kinematic tree
                     Forward pass: Dcvel, Dcacc (6×nv per body)
                     Backward pass: Dcfrc accumulation + projection to joint space
                     + direct gyroscopic derivative for Ball/Free joints
```

Cross-product derivative signs (critical):
- `d(a × b)/d(a) = −[b]×`  (negative skew of second argument)
- `d(a × b)/d(b) = [a]×`   (positive skew of first argument)

MuJoCo correspondence: `mjd_smooth_vel` → `mjd_smooth_vel`, `Data.qDeriv` →
`mjData.qDeriv` (sparse in MuJoCo, dense here).

### 6.3 Quaternion Integration Jacobians: `mjd_quat_integrate()`

Computes SO(3) Jacobians for quaternion integration `q_new = q_old ⊗ exp(ω·h/2)`:

```
Returns (dpos_dpos, dpos_dvel):
  dpos_dpos: ∂(q_new_tangent)/∂(q_old_tangent) — adjoint exp(-ω·h) via Rodrigues
  dpos_dvel: ∂(q_new_tangent)/∂ω — h × right Jacobian of SO(3)

Right Jacobian: J_r(θ) = I - (1-cos‖θ‖)/‖θ‖² · [θ]× + (‖θ‖-sin‖θ‖)/‖θ‖³ · [θ]×²
```

Used by `compute_integration_derivatives()` for Ball and Free joints. Hinge/Slide
joints use simple scalar chain rules (identity + h·I).

### 6.4 Hybrid FD+Analytical: `mjd_transition_hybrid()`

Combines analytical velocity/activation columns with FD position columns:

```
Velocity columns (analytical):
  Euler:    ∂v⁺/∂v = I + h · M⁻¹ · qDeriv   (sparse LDL solve)
  Implicit: ∂v⁺/∂v = (M+hD+h²K)⁻¹ · (M + h·(qDeriv+D))  (Cholesky solve)

Activation columns (analytical):
  DynType::None:        no activation → skip
  Filter/FilterExact:   ∂act⁺/∂act = exp(-h/τ) or 1-h/τ; force via gain·moment
  Integrator:           ∂act⁺/∂act = 1; no force-through-act derivative
  Muscle:               FD fallback (FLV curve gradients too complex)

Position columns: FD (captures contact transitions, implicit spring ∂v/∂q)
B matrix: analytical for DynType::None, FD for actuators with dynamics

Cost: ~nv FD step() calls (position columns only) vs 2·(2nv+na+nu) for pure FD
```

### 6.5 Public Dispatch: `mjd_transition()`

Dispatches to `mjd_transition_fd()` or `mjd_transition_hybrid()` based on
`DerivativeConfig.use_analytical`. Also available as `Data::transition_derivatives()`.

### 6.6 Validation Utilities

- `validate_analytical_vs_fd()`: compares hybrid vs pure FD, returns max error
- `fd_convergence_check()`: verifies FD at ε and ε/10 converge (ratio test)
- `max_relative_error()`: element-wise max relative error between two matrices

---

## Key Data Structures

### Model (static, immutable after loading)

| Field | Type | Description |
|-------|------|-------------|
| `nq, nv` | `usize` | Position coordinates, velocity DOFs |
| `nu, na` | `usize` | Actuator count, total activation dimension |
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
| `actuator_dyntype[i]` | `ActuatorDynamics` | None, Filter, FilterExact, Integrator, or Muscle |
| `actuator_trntype[i]` | `ActuatorTransmission` | Joint, Tendon, Site, or Body |
| `actuator_gear[i]` | `f64` | Transmission gear ratio |
| `actuator_gaintype[i]` | `GainType` | Fixed, Affine, or Muscle — dispatches gain computation |
| `actuator_biastype[i]` | `BiasType` | None, Affine, or Muscle — dispatches bias computation |
| `actuator_dynprm[i]` | `[f64; 3]` | Dynamics params (Muscle: [τ_act, τ_deact, τ_smooth]; Filter/FilterExact: [τ, 0, 0]) |
| `actuator_gainprm[i]` | `[f64; 9]` | Gain params (per-type: Motor=[1,0,...], Position=[kp,0,...], Damper=[0,0,-kv,...], Muscle=[range0..1, F0, scale, lmin..vmax, fpmax, fvmax]) |
| `actuator_biasprm[i]` | `[f64; 9]` | Bias params (per-type: Position=[0,-kp,-kv,...], Velocity=[0,0,-kv,...], Muscle=shared with gainprm) |
| `actuator_lengthrange[i]` | `(f64, f64)` | Transmission length extremes (gear-scaled) |
| `actuator_acc0[i]` | `f64` | ‖M⁻¹J‖ at qpos0 (for F0 auto-computation) |
| `actuator_ctrlrange[i]` | `(f64, f64)` | Control input limits (gated by ctrllimited) |
| `actuator_forcerange[i]` | `(f64, f64)` | Force output limits (gated by forcelimited) |
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
| `act[na]` | `DVector` | Activation states (muscles, filters, integrators) |
| `ctrl[nu]` | `DVector` | Control inputs |
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
| `qfrc_actuator[nv]` | `DVector` | Actuator forces in joint space |
| `qfrc_applied[nv]` | `DVector` | User-applied forces |
| `qfrc_constraint[nv]` | `DVector` | Contact + limit + equality forces |
| `actuator_length[nu]` | `Vec<f64>` | Gear × transmission length |
| `actuator_velocity[nu]` | `Vec<f64>` | Gear × transmission velocity |
| `actuator_moment[nu]` | `Vec<DVector(nv)>` | Transmission moment arm vectors (Site/Body transmissions; used for force projection and velocity) |
| `actuator_force[nu]` | `Vec<f64>` | Scalar actuator force (after gain/bias/clamp) |
| `act_dot[na]` | `DVector` | Activation time-derivative (integrated by Euler/RK4) |
| `contacts[ncon]` | `Vec<Contact>` | Active contact points |
| `efc_lambda` | `HashMap` | Warmstart cache (keyed by geom pair) |
| `qDeriv[nv,nv]` | `DMatrix` | Analytical ∂(qfrc_smooth)/∂qvel (populated by `mjd_smooth_vel`) |
| `deriv_Dcvel[nbody]` | `Vec<DMatrix(6,nv)>` | Per-body ∂(cvel)/∂(qvel) scratch Jacobians |
| `deriv_Dcacc[nbody]` | `Vec<DMatrix(6,nv)>` | Per-body ∂(cacc)/∂(qvel) scratch Jacobians |
| `deriv_Dcfrc[nbody]` | `Vec<DMatrix(6,nv)>` | Per-body ∂(cfrc)/∂(qvel) scratch Jacobians |

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

## Sleeping / Body Deactivation

Tree-based sleeping system that deactivates stationary bodies to reduce
computation. Matches MuJoCo's `mj_checkSleep` / `mj_island` architecture.

### Tree Enumeration

Bodies are partitioned into kinematic trees (connected components of
`body_parent`). Model stores per-tree metadata:

```
ntree                    — number of kinematic trees
tree_body_adr[t]         — first body index for tree t
tree_body_num[t]         — number of bodies in tree t
tree_dof_adr[t]          — first DOF index for tree t
tree_dof_num[t]          — number of DOFs in tree t
body_treeid[b]           — tree index for body b
dof_treeid[d]            — tree index for DOF d
```

### Sleep Policy Resolution

Each tree gets a `SleepPolicy` resolved at model build time:

```python
for tree in range(ntree):
    if tree has actuators or multi-tree tendons:
        policy = AutoNever       # cannot sleep (actuation coupling)
    else:
        policy = AutoAllowed     # may sleep
    # User overrides (from MJCF body/@sleep attribute):
    if any body in tree has sleep="never":   policy = Never
    if any body in tree has sleep="allowed": policy = Allowed
    if any body in tree has sleep="init":    policy = Init  # starts asleep
```

### Sleep State Machine (`mj_sleep`)

```python
for tree in range(ntree):
    if not can_sleep(tree):
        continue
    # Check if all DOFs are below threshold
    all_slow = True
    for d in tree_dofs(tree):
        if abs(qvel[d]) > sleep_tolerance * dof_length[d]:
            all_slow = False
            break
    if all_slow:
        tree_asleep[tree] += 1            # advance countdown toward -1
        if tree_asleep[tree] >= -1:
            sleep_trees(model, data, [tree])  # transition to sleep
    else:
        tree_asleep[tree] = -(1 + MIN_AWAKE)  # reset countdown
```

`sleep_trees()` zeros: `qvel`, `qacc`, `cvel`, `cacc_bias`, `cfrc_bias`,
`qfrc_bias`, `qfrc_passive`, `qfrc_constraint`, `qfrc_actuator` for all
DOFs/bodies in the tree.

### Island Discovery (`mj_island`)

DFS flood-fill over tree-tree adjacency graph. Two trees are adjacent if
they share a contact, tendon coupling, or equality constraint:

```python
# Build adjacency from active constraints
for each contact between body_a, body_b:
    tree_a, tree_b = body_treeid[body_a], body_treeid[body_b]
    if tree_a != tree_b:
        adjacency[tree_a].add(tree_b)
        adjacency[tree_b].add(tree_a)
# Similar for tendons and equality constraints

# DFS flood-fill to assign island IDs
island_id = 0
for tree in range(ntree):
    if not visited[tree]:
        dfs_assign(tree, island_id)
        island_id += 1
```

Island arrays: `tree_island[t]`, `island_ntree[i]`, `dof_island[d]`,
`contact_island[c]`, etc.

### Wake Detection

```python
# mj_wake: check user forces on sleeping bodies
for b in sleeping_bodies:
    if xfrc_applied[b] != 0 or qfrc_applied[dofs_of(b)] != 0:   # bytewise
        wake_island(island_of(tree_of(b)))

# mj_wake_collision: check contacts between sleeping/awake bodies
for contact in contacts:
    if one_sleeping(contact) and one_awake(contact):
        wake_island(island_of(sleeping_tree))

# mj_wake_tendon: active limited tendons coupling sleeping ↔ awake trees
# mj_wake_equality: equality constraints to awake trees
# qpos change: tree_qpos_dirty flag set by mj_kinematics1()
```

### Pipeline Skip Logic

When `ENABLE_SLEEP` is set, pipeline stages use awake-index indirection
(`body_awake_ind`, `dof_awake_ind`, `parent_awake_ind`) for O(awake) iteration:

| Stage | Skip behavior |
|-------|---------------|
| FK (`mj_fwd_position`) | Sleeping bodies: poses frozen, not recomputed |
| Collision (`mj_collision`) | Both geoms asleep: skip narrow-phase |
| Velocity (`mj_fwd_velocity`) | Sleeping DOFs: spatial velocity not updated |
| Passive forces (`mj_fwd_passive`) | Sleeping DOFs: spring/damper/friction skipped |
| Integration | Sleeping joints: qpos/qvel not updated |
| Sensors | Sleeping bodies: return frozen values (not zeroed) |

### Selective CRBA + Partial LDL (Phase C)

`mj_crba` with sleep filter:
- Phase 2 (backward pass): only accumulates composite inertia for awake bodies
- Phase 3 (build M): only computes mass matrix entries for awake DOFs
- Sleeping DOFs retain their last-awake `qM` entries

`mj_factor_sparse_selective`: dispatches to partial factorization:
```python
if nv_awake < nv:
    # Only factorize awake DOF blocks
    for i in awake_dofs (leaves to root):
        # Same elimination as mj_factor_sparse, but restricted to awake DOFs
        # Tree independence guarantees no cross-contamination
else:
    mj_factor_sparse(model, data)  # full factorization
```

### Per-Island Constraint Solving

When `nisland > 1`, `mj_fwd_constraint_islands` replaces the global solver:

```python
for island in range(nisland):
    # Gather island-local DOFs, contacts, constraints
    # Build small island-local Delassus matrix (island_nv × island_nv)
    # Solve independently via PGS/CG
    # Scatter forces back to global arrays
```

Single-island scenes use the global solver path (no overhead).

---

## References

- [MuJoCo Computation](https://mujoco.readthedocs.io/en/stable/computation/index.html)
- [MuJoCo Source — engine_forward.c](https://github.com/google-deepmind/mujoco/blob/main/src/engine/engine_forward.c)
- [MuJoCo Source — engine_solver.c](https://github.com/google-deepmind/mujoco/blob/main/src/engine/engine_solver.c)
- [MuJoCo Source — engine_util_misc.c](https://github.com/google-deepmind/mujoco/blob/main/src/engine/engine_util_misc.c) (muscle FLV curves, activation dynamics)
- Featherstone, R. (2008). *Rigid Body Dynamics Algorithms*. Springer.
- Todorov, E. (2014). "Convex and analytically-invertible dynamics with contacts and constraints."
- Millard, M. et al. (2013). "Flexing Computational Muscle: Modeling and Simulation of Musculotendon Dynamics." *Journal of Biomechanical Engineering*. (Activation dynamics with activation-dependent time constants)
