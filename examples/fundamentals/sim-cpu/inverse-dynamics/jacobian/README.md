# Jacobian — End-Effector Velocity Mapping

Demonstrates the analytical Jacobian: how joint velocities map to
Cartesian end-effector velocity. The Jacobian `J` is the matrix that
converts joint-space velocities to task-space velocities: `v = J * qdot`.

## What you see

Two identical arms swing side by side under gravity (with light damping):

- **Left arm (dim)** — uses a **stale Jacobian**, computed once at t=0
  and never updated. The green prediction arrow diverges from the red
  actual-velocity arrow as the arm moves away from its initial pose.
- **Right arm (solid)** — uses the **correct Jacobian**, recomputed at
  every physics step. The green arrow covers the red arrow perfectly.

Each arm has two arrows at its end-effector tip:
- **Green** = Jacobian prediction (`J * qdot`)
- **Red** = finite-difference actual velocity

**Key insight:** On the right arm you only see green (correct Jacobian
matches reality). On the left arm you see red poking out from under
green (stale Jacobian predicts the wrong velocity). This shows why
the Jacobian must be recomputed at each configuration — it depends on
the arm's current pose.

## Physics

The site Jacobian at the end-effector is a 3x2 matrix (3 Cartesian axes
x 2 DOFs). Each column represents the velocity contribution of one
joint:

```
jacp = [ d(ee_x)/dq1   d(ee_x)/dq2 ]     (3 x nv)
       [ d(ee_y)/dq1   d(ee_y)/dq2 ]
       [ d(ee_z)/dq1   d(ee_z)/dq2 ]
```

For hinge joints, each column is `axis x r` where `r` is the lever arm
from the joint to the end-effector.

At zero configuration (arm hanging straight down):
```
shoulder column: axis x r = [0,1,0] x [0,0,-0.7] = [-0.7, 0, 0]
elbow column:    axis x r = [0,1,0] x [0,0,-0.3] = [-0.3, 0, 0]
```

The shoulder column is longer because it has a longer lever arm (the
full arm length 0.7m vs just the forearm 0.3m).

| Parameter | Value |
|-----------|-------|
| Upper arm | 0.4 m, 2.0 kg |
| Lower arm | 0.3 m, 1.0 kg |
| Joint damping | 0.3 N·m·s/rad |
| Initial shoulder | 1.0 rad (57°) |
| Initial elbow | -0.5 rad (-29°) |
| Arrow scale | 0.3x (velocity to visual length) |
| Integrator | RK4, dt = 2 ms |

## Expected console output

```
  Jacobian at zero config:
    jacp shoulder col: [-0.7000, 0.0000, 0.0000]
    expected:          [-0.7000, 0, 0]
    error: 0.00e0
    jacp dims: 3x2 (expect 3x2)

t=  1.0s  shoulder=-0.094  elbow=-0.144
...

=== Jacobian (t=6s) ===
  Jacobian dims (3 x nv):       3x2 (expect 3x2)  PASS
  Zero-config analytical form:  err=0.00e0  PASS
  Correct J*qdot vs FD < 1%:    max rel err=0.28%  PASS
```

## Validation

Three automated checks at t=6s (validated on the correct arm only):

| Check | Expected | Threshold |
|-------|----------|-----------|
| **Jacobian dimensions** | jacp is 3 x nv (3x2) | exact |
| **Zero-config form** | Shoulder column matches `[-(L1+L2), 0, 0]` | < 1e-6 |
| **Velocity match** | Correct `J*qdot` matches finite-difference velocity | < 1% relative |

## Run

```
cargo run -p example-inverse-jacobian --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
