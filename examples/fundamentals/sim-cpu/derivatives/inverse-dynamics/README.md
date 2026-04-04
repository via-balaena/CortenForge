# Inverse Dynamics Derivatives — DfDq, DfDv, DfDa

`mjd_inverse_fd()` computes finite-difference Jacobians of the inverse dynamics
map: given joint positions, velocities, and accelerations, inverse dynamics
returns the generalized forces `qfrc = M*qacc + bias(qpos, qvel)`. The three
Jacobians tell you how those forces change when you perturb each input:

- **DfDq** — how forces change with position (gravity, Coriolis geometry)
- **DfDv** — how forces change with velocity (Coriolis, damping)
- **DfDa** — how forces change with acceleration (the mass matrix M)

The key insight: because inverse dynamics is linear in `qacc`, the derivative
`DfDa` exactly recovers the mass matrix M.

## What you see

- **Double pendulum** (two hinge joints, Y-axis, light damping) at
  q=[0.3, -0.5] rad with qvel=[1.0, -0.5] rad/s
- Frozen for 3 seconds showing DfDq, DfDv, DfDa, and M side by side
- Released to swing freely — the HUD shows the instantaneous DfDa vs M error
  and **per-swing averages** (detected via joint 1 velocity zero-crossings,
  4 crossings = 1 full swing)
- Error *decreases* over time as damping reduces swing amplitude — smaller
  arcs mean less mass-matrix curvature within each FD perturbation step
- The mass matrix changes shape as the pendulum reconfigures — the off-diagonal
  coupling between joints depends on the relative angle

## Physics

The inverse dynamics equation is `qfrc = M(q)*qacc + c(q, qvel)` where `c`
captures Coriolis, centrifugal, and gravitational terms. Differentiating:

- `DfDa = dqfrc/dqacc = M` (linear in acceleration)
- `DfDq = dqfrc/dqpos = dM/dq * qacc + dc/dq` (configuration-dependent)
- `DfDv = dqfrc/dqvel = dc/dqvel` (velocity-dependent)

For a double pendulum, all three are 2x2 matrices. DfDq captures how gravity
torques shift with joint angles. DfDv captures how Coriolis forces couple the
two joints. DfDa is the 2x2 inertia tensor in joint space.

| Parameter | Value |
|-----------|-------|
| DOF | 2 (hinge + hinge, damping=0.005) |
| Actuators | 0 |
| Link mass | 1.0 kg each (CoM at 0.25 m) |
| Link length | 0.5 m each |
| Initial qpos | [0.3, -0.5] rad |
| Initial qvel | [1.0, -0.5] rad/s |
| FD method | centered, eps = 1e-6 |

## Validation

Four automated checks at startup:

| Check | Expected | Threshold |
|-------|----------|-----------|
| **DfDq, DfDv, DfDa each 2x2** | nv x nv | exact |
| **DfDa matches M within 1e-5** | d(M*a+bias)/da = M | 1e-5 rel |
| **DfDq not all zeros** | gravity depends on position | any nonzero |
| **DfDv not all zeros** | Coriolis depends on velocity | any nonzero |

## Run

```
cargo run -p example-derivatives-inverse-dynamics --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
