# MuJoCo Reference Architecture

**Purpose**: This document defines the EXACT architecture we must follow. No deviations.

## Core Principle

MuJoCo's physics pipeline is carefully designed with each stage depending on previous stages. We must implement it **exactly** as specified, not approximate it.

## Computation Pipeline (Exact Order)

```
mj_step:
  1. mj_fwdPosition    - Forward kinematics
  2. mj_fwdVelocity    - Velocity-dependent quantities
  3. mj_fwdActuation   - Actuator forces
  4. mj_fwdAcceleration - Constraint solving
  5. mj_Euler/implicit  - Integration
```

### Stage 1: Forward Kinematics (`mj_fwdPosition`)

Computes:
- Global position and orientation of all bodies
- Joint axes in world coordinates
- Collision detection (broad → mid → narrow phase)

**Output**: Body poses, contact list

### Stage 2: Forward Velocity (`mj_fwdVelocity`)

Computes:
- Bias forces using Recursive Newton-Euler (RNE) with zero acceleration
- `qfrc_bias = c(q, v)` (Coriolis + centrifugal + gravity)
- Passive forces: `qfrc_passive` (joint springs, dampers)

**Passive force formula**:
```
τ_passive = -stiffness * (q - springref) - damping * qdot
```

### Stage 3: Forward Actuation (`mj_fwdActuation`)

Computes:
- Actuator forces from control inputs
- `qfrc_actuator`

### Stage 4: Forward Acceleration (`mj_fwdAcceleration`)

This is the constraint solver. **Most critical stage.**

#### 4.1 Compute Applied Forces
```
τ = qfrc_passive + qfrc_actuator + qfrc_applied - qfrc_bias
```

#### 4.2 Compute Unconstrained Acceleration
```
a_unconstrained = M^(-1) * τ
```

Where M is the joint-space inertia matrix (computed via Composite Rigid Body algorithm).

#### 4.3 Build Constraint Jacobian

For each constraint, compute the Jacobian row J_i that maps joint velocities to constraint space:
```
c_i = J_i * qdot
```

Constraint types (in order):
1. **Equality constraints** (weld, joint, distance, connect)
2. **Friction loss** (joint friction)
3. **Limit constraints** (joint limits)
4. **Contact constraints** (collision response)

#### 4.4 Solve Constraint Optimization

**The Dual Problem**:
```
f = argmin_λ { (1/2) λ^T (A + R) λ + λ^T (a_unconstrained - a_ref) }
subject to: λ ∈ Ω
```

Where:
- `A = J M^(-1) J^T` (inverse inertia in constraint space)
- `R` = diagonal regularization (softness)
- `a_ref = -b * (J * v) - k * r` (reference acceleration with Baumgarte)
- `Ω` = constraint bounds (depends on type)

**Constraint bounds Ω**:
| Type | Bounds |
|------|--------|
| Equality | λ ∈ ℝ (unconstrained) |
| Friction loss | \|λ\| ≤ η |
| Limit | λ ≥ 0 |
| Contact normal | λ_n ≥ 0 |
| Contact friction | \|λ_t\| ≤ μ * λ_n |

#### 4.5 Compute Constrained Acceleration
```
qacc = M^(-1) * (τ + J^T * f)
```

### Stage 5: Integration

**Semi-implicit Euler** (default):
```
qvel_new = qvel + dt * qacc
qpos_new = qpos + dt * qvel_new  // Note: uses NEW velocity
```

**Implicit** (for stiff systems):
Solves implicit equation including velocity-dependent forces.

## Key Data Structures

### Joint-Space Quantities (dimension: nv)
- `qpos` - generalized positions
- `qvel` - generalized velocities
- `qacc` - generalized accelerations
- `qfrc_bias` - bias forces (Coriolis + gravity)
- `qfrc_passive` - passive forces (springs, dampers)
- `qfrc_actuator` - actuator forces
- `qfrc_applied` - user-applied forces

### Constraint-Space Quantities (dimension: nc)
- `efc_J` - constraint Jacobian (nc × nv)
- `efc_force` - constraint forces
- `efc_aref` - reference acceleration

### Inertia Matrix
- `qM` - joint-space inertia (nv × nv), stored as L^T D L factorization

## PGS Solver Algorithm

```python
def pgs_solve(A, R, b, bounds, max_iter):
    """
    Solve: min (1/2) λ^T (A+R) λ + λ^T b
    subject to: λ ∈ bounds
    """
    λ = zeros(nc)
    H = A + R  # Hessian

    for iteration in range(max_iter):
        for i in range(nc):
            # Compute gradient for constraint i
            grad_i = H[i,:] @ λ + b[i]

            # Gauss-Seidel update
            λ_new = λ[i] - grad_i / H[i,i]

            # Project onto constraint bounds
            λ[i] = project(λ_new, bounds[i])

    return λ
```

## What We Got Wrong

1. **Penalty-based constraints**: MuJoCo uses impulse-based (Lagrange multipliers), not penalty forces
2. **Missing proper Jacobian**: We didn't compute J correctly for all constraint types
3. **Wrong integration order**: Semi-implicit uses NEW velocity for position update
4. **No proper inertia matrix**: We didn't implement Composite Rigid Body algorithm
5. **Missing bias forces**: Coriolis and centrifugal forces weren't computed via RNE
6. **Passive forces timing**: Springs/dampers go in `qfrc_passive`, computed in Stage 2

## Implementation Plan

### Phase 1: Single Pendulum (Visual Validation)
- One revolute joint, two bodies
- Implement: kinematics, M, bias forces, semi-implicit integration
- NO constraints yet (just gravity)
- **Must see pendulum swing correctly**

### Phase 2: Add Joint Constraints
- Implement constraint Jacobian for revolute joint
- Implement PGS solver
- **Must see pendulum stay connected under gravity**

### Phase 3: Multi-Body Chain
- Extend to 3+ body chain
- Validate constraint solver handles multiple joints
- **Must see chain swing without exploding**

### Phase 4: Full Humanoid
- Add all joint types
- Add contact constraints
- **Must see humanoid stand/fall realistically**

## References

- [MuJoCo Computation](https://mujoco.readthedocs.io/en/stable/computation/index.html)
- [MuJoCo Modeling](https://mujoco.readthedocs.io/en/stable/modeling.html)
- Todorov, E. (2014). Convex and analytically-invertible dynamics with contacts and constraints.
