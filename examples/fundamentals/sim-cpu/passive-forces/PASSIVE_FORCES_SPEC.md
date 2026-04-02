# Passive Forces Examples Spec

**Status:** Draft
**Date:** 2026-04-02
**Engine code:** `sim/L0/core/src/forward/passive.rs` (1,023 lines)
**Tests:** 133 passing (26 passive + 8 conformance + 99 fluid/derivatives)

## Scope

Passive forces are configuration/velocity-dependent forces the engine computes
automatically — no actuator needed. The pipeline (`mj_fwd_passive`) produces
five component vectors:

| Component | Array | Source |
|-----------|-------|--------|
| Joint spring | `qfrc_spring` | `-stiffness * (q - springref)` |
| Joint damper | `qfrc_damper` | `-damping * qvel` |
| Fluid drag | `qfrc_fluid` | Inertia-box (body) or ellipsoid (per-geom) model |
| Gravity comp | `qfrc_gravcomp` | RNE-based, DOFs without actuator routing |
| Aggregated | `qfrc_passive` | `spring + damper [+ gravcomp] [+ fluid]` |

Joint spring/damper and gravity compensation are already demonstrated in
`slide-joint/`, `urdf-loading/`, and `inverse-dynamics/`. This directory
covers the **remaining** passive force features: fluid drag (both models),
spring-damper response characterization, and wind interaction.

## Prior Art (already covered elsewhere)

- Spring stiffness + damping on slide joint (`slide-joint/horizontal`)
- Gravity compensation (`inverse-dynamics/gravity-comp`)
- Friction loss (solver constraint, not passive — `contact-tuning/friction-slide`)
- Tendon spring/damper (`tendons/fixed-coupling`)

## Examples

### 1. `fluid-drag` — Terminal Velocity and Drag Models

Three spheres falling through a viscous medium side by side. Each sphere has
different mass, so they reach different terminal velocities. Demonstrates the
legacy inertia-box fluid model (body-level, automatic when no per-geom fluid
params are set).

**Physics:**
- Without drag: free-fall v = g*t (unbounded).
- With drag: terminal velocity where drag force = weight, net acceleration = 0.
- Inertia-box model: equivalent box from body inertia, quadratic + viscous
  drag. Viscous term ~ beta * v (Stokes), quadratic term ~ rho * v^2.
- Equivalent box dimensions from inertia: `bx = sqrt((Iy+Iz-Ix)/m * 6)` etc.
  For a sphere (Ix=Iy=Iz=2mr^2/5): `bx = by = bz = r * sqrt(12/5)`.
- Z-axis quadratic drag: `F_drag = 0.5 * rho * bx * by * v^2` (implicit Cd=1).
- Terminal velocity: `v_t = sqrt(2*m*g / (rho * bx * by))`.

**MJCF sketch:**
```xml
<mujoco model="fluid-drag">
  <option gravity="0 0 -9.81" timestep="0.002"
          density="1.2" viscosity="1.5e-5"/>
  <worldbody>
    <!-- Three free-falling spheres, different masses -->
    <body name="light" pos="-1 0 10">
      <freejoint/>
      <geom type="sphere" size="0.15" mass="0.5"/>
    </body>
    <body name="medium" pos="0 0 10">
      <freejoint/>
      <geom type="sphere" size="0.15" mass="2.0"/>
    </body>
    <body name="heavy" pos="1 0 10">
      <freejoint/>
      <geom type="sphere" size="0.15" mass="8.0"/>
    </body>
  </worldbody>
</mujoco>
```

**What you see:** Three spheres released from the same height. The light
sphere quickly reaches terminal velocity and drifts down slowly. The heavy
sphere accelerates much longer before drag catches up. The medium sphere
is in between. HUD shows velocity, drag force, and weight for each.

**Validation (in ValidationHarness):**
- Light sphere velocity plateaus (|dv/dt| < 0.1 m/s^2 for 2+ seconds)
- Terminal velocity ordering: v_light < v_medium < v_heavy
- At plateau: |qfrc_fluid_z + m*g| < 5% of m*g (drag balances weight)

### 2. `ellipsoid-drag` — Per-Geom Advanced Fluid Model

Demonstrates the 5-component ellipsoid fluid model with per-geom
`fluidshape="ellipsoid"` and `fluidcoef`. Three bodies with different shapes
(sphere, long capsule, flat cylinder) fall through a medium. Shape affects
drag coefficient — the flat cylinder has highest frontal area and decelerates
fastest.

**Physics:**
- Ellipsoid model activated by `fluidshape="ellipsoid"` on any child geom.
- 5 force components: added mass (gyroscopic), Magnus lift, Kutta lift,
  linear drag, angular drag.
- Per-geom coefficients: `fluidcoef="C_blunt C_slender C_ang C_Kutta C_Magnus"`.
- Default coefficients (from `fluid.rs:109`): `[0.5, 0.25, 1.5, 1.0, 1.0]`.
- Shape matters: semi-axes determine projected area, drag scales with it.

**MJCF sketch:**
```xml
<mujoco model="ellipsoid-drag">
  <option gravity="0 0 -9.81" timestep="0.002"
          density="50.0" viscosity="0.1"/>
  <worldbody>
    <body name="sphere" pos="-1.5 0 10">
      <freejoint/>
      <geom type="sphere" size="0.15" mass="2.0"
            fluidshape="ellipsoid"/>
    </body>
    <body name="capsule" pos="0 0 10">
      <freejoint/>
      <geom type="capsule" size="0.08 0.4" mass="2.0"
            fluidshape="ellipsoid"/>
    </body>
    <body name="cylinder" pos="1.5 0 10">
      <freejoint/>
      <geom type="cylinder" size="0.3 0.05" mass="2.0"
            fluidshape="ellipsoid"/>
    </body>
  </worldbody>
</mujoco>
```

**What you see:** Three shapes released from the same height with equal mass.
The streamlined capsule falls fastest (low frontal area along fall axis). The
flat cylinder falls slowest (high frontal area). HUD shows velocity and
per-component fluid forces.

**Validation:**
- All three reach terminal velocity within simulation time
- Capsule terminal velocity > sphere terminal velocity > cylinder terminal velocity
- qfrc_fluid != 0 for all bodies when density > 0

### 3. `wind` — Wind Force on Bodies

Bodies in a crosswind field. A free-falling sphere experiences lateral drift
from wind. A hanging pendulum deflects from vertical under wind load.
Demonstrates `<option wind="vx vy vz"/>` — wind subtracts from body velocity
before computing drag, so a stationary body in wind experiences the same force
as a body moving at -wind in still air.

**Physics:**
- Wind is global: `model.wind = Vector3(vx, vy, vz)`.
- In fluid computation, effective velocity = body_velocity - wind.
- Stationary body in wind: effective velocity = -wind, drag opposes wind.
- Pendulum equilibrium angle: tan(theta) = F_drag / (m * g).

**MJCF sketch:**
```xml
<mujoco model="wind">
  <option gravity="0 0 -9.81" timestep="0.002"
          density="1.2" viscosity="1.5e-5" wind="5 0 0"/>
  <worldbody>
    <!-- Free sphere drifts in wind -->
    <body name="drifter" pos="0 0 5">
      <freejoint/>
      <geom type="sphere" size="0.2" mass="1.0"/>
    </body>
    <!-- Pendulum deflects in wind -->
    <body name="pivot" pos="3 0 3">
      <joint name="hinge" type="hinge" axis="0 1 0" damping="0.5"/>
      <geom type="capsule" fromto="0 0 0 0 0 -1.5" size="0.05" mass="2.0"/>
      <body name="bob" pos="0 0 -1.5">
        <geom type="sphere" size="0.15" mass="3.0"/>
      </body>
    </body>
  </worldbody>
</mujoco>
```

**What you see:** The free sphere drifts sideways as it falls — parabolic
trajectory instead of straight down. The pendulum swings toward the wind and
settles at an angle from vertical. HUD shows wind velocity, body velocity,
effective velocity (body - wind), and fluid force.

**Validation:**
- Drifter has nonzero x-velocity after 2 seconds (wind accelerates it)
- Pendulum reaches steady-state angle > 0 (deflected from vertical)
- With wind = [0,0,0], qfrc_fluid for stationary body = 0

### 4. `spring-damper-tuning` — Underdamped, Critically Damped, Overdamped

Three identical hinge pendulums displaced to the same initial angle, each with
different stiffness and damping combinations producing three distinct damping
regimes. This is the classic second-order response characterization.

**Physics:**
- **Zero gravity** — eliminates the nonlinear `m*g*L*sin(q)` gravity torque
  that would couple with the spring. Pure torsional spring-damper system,
  exactly linear: `I * q'' + c * q' + k * (q - q_ref) = 0`.
- Natural frequency: omega_n = sqrt(k / I_eff)
- Damping ratio: zeta = c / (2 * sqrt(k * I_eff))
- zeta < 1: underdamped (oscillates, decays)
- zeta = 1: critically damped (fastest return, no overshoot)
- zeta > 1: overdamped (exponential decay, no oscillation)

**MJCF sketch:**
```xml
<mujoco model="spring-damper-tuning">
  <compiler angle="radian"/>
  <option gravity="0 0 0" timestep="0.001"/>
  <worldbody>
    <!-- A: Underdamped (zeta = 0.1) -->
    <body name="arm_a" pos="-1 0 0">
      <joint name="j_a" type="hinge" axis="0 1 0"
             stiffness="20" damping="COMPUTED" springref="0"/>
      <geom type="capsule" fromto="0 0 0 0 0 -1" size="0.04" mass="1.0"/>
      <geom type="sphere" pos="0 0 -1" size="0.08" mass="0.5"/>
    </body>
    <!-- B: Critically damped (zeta = 1.0) -->
    <body name="arm_b" pos="0 0 0">
      <joint name="j_b" type="hinge" axis="0 1 0"
             stiffness="20" damping="COMPUTED" springref="0"/>
      <geom type="capsule" fromto="0 0 0 0 0 -1" size="0.04" mass="1.0"/>
      <geom type="sphere" pos="0 0 -1" size="0.08" mass="0.5"/>
    </body>
    <!-- C: Overdamped (zeta = 3.0) -->
    <body name="arm_c" pos="1 0 0">
      <joint name="j_c" type="hinge" axis="0 1 0"
             stiffness="20" damping="COMPUTED" springref="0"/>
      <geom type="capsule" fromto="0 0 0 0 0 -1" size="0.04" mass="1.0"/>
      <geom type="sphere" pos="0 0 -1" size="0.08" mass="0.5"/>
    </body>
  </worldbody>
</mujoco>
```

Note: Damping values are set programmatically after model load. Read effective
inertia from `data.qM[(dof_adr, dof_adr)]` (the CRBA-computed diagonal,
includes body inertia + parallel axis + armature), then set
`model.jnt_damping[jid] = 2 * zeta * sqrt(k * I_eff)` for each target zeta.

**What you see:** Three pendulums displaced to ~45 degrees and released. The
underdamped one oscillates with decaying amplitude. The critically damped one
returns smoothly without overshoot. The overdamped one returns sluggishly.
HUD shows angle, angular velocity, natural frequency, and damping ratio for
each.

**Validation:**
- Underdamped: at least 3 zero-crossings in 10 seconds
- Critically damped: no zero-crossing after release (no overshoot)
- Overdamped: no zero-crossing after release, slower return than critical
- All three converge to springref (|q - springref| < 0.01 rad by t=15s)
- Oscillation frequency (underdamped): omega_d = omega_n * sqrt(1 - zeta^2),
  measured period within 5% of analytical T = 2*pi/omega_d

### 5. `stress-test` — Headless Validation

Headless (no Bevy). Multiple MJCF models, each testing a specific passive
force invariant. Target: 15 checks minimum.

**Checks:**

*Fluid drag — inertia-box model:*
1. **Terminal velocity magnitude** — sphere in density=1.2, viscosity=0:
   compute bx, by from `model.body_inertia` and `model.body_mass`, then
   `v_t = sqrt(2*m*g / (rho * bx * by))`. Measured within 5% after 20s.
2. **Drag proportional to v^2** — at half terminal velocity, drag force is
   ~25% of weight (quadratic). Verify ratio within 10%.
3. **Zero medium = zero drag** — density=0, viscosity=0: qfrc_fluid == 0
   for all DOFs after stepping.
4. **Viscous vs quadratic regime** — pure viscosity (density=0, viscosity=1.0):
   qfrc_fluid is linear in velocity (double v, double force). Verify ratio
   within 5%.

*Fluid drag — ellipsoid model:*
5. **Ellipsoid activates on fluidshape** — body with `fluidshape="ellipsoid"`:
   qfrc_fluid != 0 when density > 0.
6. **Shape dependence** — sphere vs flat cylinder with equal mass in same
   medium: cylinder has higher drag at same speed (larger projected area).
7. **Interaction coefficient scaling** — after loading, set
   `model.geom_fluid[gid][0] = 0.5` (not settable from MJCF — always 1.0
   for `fluidshape="ellipsoid"`). Produces half the force vs 1.0 (within 1%).

*Wind:*
8. **Wind on stationary body** — body at rest with wind=[10,0,0]: qfrc_fluid
   has nonzero x-component.
9. **Wind cancellation** — set `model.wind = [W, 0, 0]` and give the body
   initial `qvel` matching W in x. After `mj_forward`, `qfrc_fluid` x-component
   is ~0 (within 1e-6). Effective velocity = body_vel - wind = 0.
10. **Wind direction** — wind=[0,10,0]: fluid force has y-component, not x.

*Spring mechanics:*
11. **Spring restoring force** — slide joint with stiffness=100, displaced by
    0.1 from springref: qfrc_spring = -100 * 0.1 = -10.0 (within 1e-10).
12. **Springref != qpos0** — springref=0.5, qpos0=0: spring pulls toward 0.5,
    not toward 0. Verify force direction.
13. **Spring + gravity equilibrium** — vertical slide with stiffness=k, mass=m:
    equilibrium at q = springref + mg/k (within 1%).

*Damper mechanics:*
14. **Damper dissipation** — hinge with damping, no spring: total energy
    monotonically decreases every 100 steps.
15. **Damper force proportional to velocity** — qfrc_damper = -b * qvel
    (within 1e-10 at a single timestep, read before step corrupts it).

*Integration mode:*
16. **ImplicitSpringDamper suppression** — with
    `<option integrator="implicitspringdamper"/>`: qfrc_spring and qfrc_damper
    remain zero (force application gated on `!implicit_mode` — forces handled
    implicitly via K/D matrices in `mj_fwd_acceleration_implicit`). qfrc_passive
    still aggregates fluid + gravcomp contributions.

*Disable flags:*
17. **DISABLE_SPRING** — `<option><flag spring="disable"/></option>`:
    qfrc_spring = 0 for all DOFs, but qfrc_damper still computed.
18. **DISABLE_DAMPER** — `<option><flag damper="disable"/></option>`:
    qfrc_damper = 0 for all DOFs, but qfrc_spring still computed.

## Directory Structure

```
passive-forces/
  README.md
  PASSIVE_FORCES_SPEC.md          # This file
  stress-test/
    Cargo.toml                     # example-passive-stress-test
    README.md
    src/main.rs
  fluid-drag/
    Cargo.toml                     # example-passive-fluid-drag
    README.md
    src/main.rs
  ellipsoid-drag/
    Cargo.toml                     # example-passive-ellipsoid-drag
    README.md
    src/main.rs
  wind/
    Cargo.toml                     # example-passive-wind
    README.md
    src/main.rs
  spring-damper-tuning/
    Cargo.toml                     # example-passive-spring-damper-tuning
    README.md
    src/main.rs
```

## Cargo.toml Naming

| Directory | Package name |
|-----------|-------------|
| stress-test | `example-passive-stress-test` |
| fluid-drag | `example-passive-fluid-drag` |
| ellipsoid-drag | `example-passive-ellipsoid-drag` |
| wind | `example-passive-wind` |
| spring-damper-tuning | `example-passive-spring-damper-tuning` |

## Dependencies

- **Visual examples:** `sim-core`, `sim-mjcf`, `sim-bevy`, `bevy`
- **Stress-test:** `sim-core`, `sim-mjcf`, `nalgebra`

## Implementation Order

1. `stress-test` — validates engine correctness first (18 checks)
2. `fluid-drag` — simplest visual (inertia-box model, just density/viscosity)
3. `ellipsoid-drag` — advanced fluid model (per-geom fluidshape)
4. `wind` — wind interaction (builds on fluid understanding)
5. `spring-damper-tuning` — classical dynamics (no fluid, pure spring/damper)

Stress-test first: if any check fails, we fix the engine before building
visual examples on top of it. Visual examples in order of increasing
conceptual complexity.
