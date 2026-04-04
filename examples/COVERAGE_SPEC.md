# Examples Coverage Spec

**Status:** Track 1A complete, Track 1B in progress
**Date:** 2026-04-02
**Goal:** 100% coverage of codebase capabilities

## Principle

Examples should mirror the distribution of code in the codebase. Every major
feature should have at least one dedicated example. Examples also serve as
integration tests — they find bugs that unit tests miss.

## Current State (2026-04-02)

- 232K LOC codebase
- sim-core + sim-mjcf + sim-urdf → 139 examples (Track 1A + Track 1B layers 1–13)
  - Joint types: hinge, slide, ball, free (4/4 — all covered)
  - Sensors: 31/31 types covered (Track 1A: 16, tendons: 3, joint-limits: 1, sensors-advanced: 11+)
  - Actuators: 10 examples, Muscles: 5 examples
  - Integrators: 8 examples, Solvers: 5 examples
  - Equality constraints: 8 examples
  - Contact tuning: 7 examples
  - Inverse dynamics: 5 examples, Energy-momentum: 5 examples
  - URDF loading: 10 examples
  - Mocap bodies: 4 examples, 12 stress-test checks
  - Tendons: 8 examples, 16 stress-test checks
  - Sensors-advanced: 7 examples, 25 stress-test checks
  - Passive forces: 5 examples, 18 stress-test checks
- Track 1B in progress: 20 subdirectories, ~70 examples — no stone unturned.
  **13/20 subdirectories done:**
  - free-joint: 4 examples, 12 stress-test checks
  - keyframes: 3 examples, 12 stress-test checks
  - mocap-bodies: 4 examples, 12 stress-test checks
  - contact-filtering: 4 examples, 12 stress-test checks
  - joint-limits: 4 examples, 12 stress-test checks
  - tendons: 8 examples, 16 stress-test checks
  - sensors-advanced: 7 examples, 25 stress-test checks
  - passive-forces: 5 examples, 18 stress-test checks
  - sleep-wake: 4 examples, 18 stress-test checks
  - raycasting: 4 examples, 20 stress-test checks
  - derivatives: 8 examples (7 visual + 1 stress-test), 61 checks
  - composites: 4 examples (3 visual + 1 stress-test), 15 checks
  - batch-sim: 3 examples (2 visual + 1 stress-test), 18 checks
  Remaining 7: mesh collision, heightfield terrain,
  Hill muscle, adhesion, flex bodies, collision
  pairs, and plugins.
- cf-design → 3 examples
- mesh-* → 1 example
- sim-gpu → 0 working examples
- SDF-CPU ladder: 8 working, 7 stubs, 1 blocked

## Gap Analysis

### Joint Types (4 types, 4 covered)
- Hinge → covered (simple-pendulum, double-pendulum, actuator-force, sensor examples)
- Free → **dedicated directory** (tumble, projectile, spinning-toss, stress-test) + used in energy-momentum, SDF physics
- Slide → covered (horizontal, vertical, geom-distance)
- Ball → covered (spherical-pendulum, conical-pendulum, cone-limit, frame-pos-quat, gyro-velocimeter)

### Geometry Types (8 types, ~6 covered)
- Plane, Sphere, Capsule, Cylinder, Box, Ellipsoid → covered
- **Mesh (explicit convex, 2,090 LOC) → Track 1B**
- **Hfield (height field terrain, 441 LOC) → Track 1B**
- Sdf → extensively covered

### Actuator System (10 standard + 5 muscle examples)
- Transmission: Joint ✓, Tendon ✓, Site ✓, SliderCrank ✓, **Body (adhesion) → Track 1B**, JointInParent (≈Joint, skip)
- Dynamics: None ✓, Filter ✓, Integrator ✓, Muscle ✓, **HillMuscle → Track 1B**, User (plugin)
- Gain: Fixed ✓, Affine ✓, Muscle ✓, **HillMuscle → Track 1B**, User (plugin)
- Bias: None ✓, Affine ✓, Muscle ✓, **HillMuscle → Track 1B**, User (plugin)
- Muscle-specific: Activation dynamics ✓, Force-length ✓, Cocontraction ✓, Forearm flexion ✓

### Sensors (31 types, 16 covered in 10 examples)
- Position: JointPos ✓, FramePos ✓, FrameQuat ✓, SubtreeCom ✓, Clock ✓
- Velocity: JointVel ✓, Gyro ✓, Velocimeter ✓
- Force: Touch ✓, ActuatorFrc ✓, JointActuatorFrc ✓
- Spatial: Accelerometer ✓, GeomDist ✓, GeomNormal ✓, GeomFromTo ✓
- Ball: BallQuat ✓, BallAngVel ✓ (in ball-joint/spherical-pendulum)
- Momentum: SubtreeAngMom ✓ (in energy-momentum/free-flight)
- **Uncovered → Track 1B:** TendonPos, TendonVel, Rangefinder, FrameLinVel,
  FrameAngVel, FrameLinAcc, FrameAngAcc, Force, Torque, JointLimitFrc,
  TendonLimitFrc, SubtreeLinVel, ActuatorPos, ActuatorVel

### Collision (partially covered)
- SDF-plane ✓, SDF-SDF ✓, analytical convex ✓
- Convex-convex (GJK/EPA) ✓ (geom-distance sensor example — also fixed a GJK bug)
- Friction tuning ✓ (friction-slide, condim-compare — 2 examples)
- Solref/solimp (bounce) ✓ (solref-bounce — 1 example)
- Contact parameter override ✓ (pair-override — 1 example)
- Solimp impedance curve ✓ (solimp-depth — 1 example)
- Margin/gap activation ✓ (margin-gap — 1 example)
- **contype/conaffinity bitmask filtering → Track 1B**
- **Collision pair isolation (all primitive pairs) → Track 1B**
- **Mesh-mesh, mesh-plane → Track 1B**
- **Height field → Track 1B**

### Solvers & Integration
- Newton ✓, PGS ✓, CG ✓ (comparison + comparison-visual + 3 per-solver)
- Euler ✓, RK4 ✓, Implicit ✓, ImplicitFast ✓, ImplicitSpringDamper ✓ (comparison + comparison-visual + 5 per-integrator)

### Constraint System
- Contact ✓
- Equality (weld, connect, distance, joint coupling) ✓ (8 examples)
- **Joint limits (dedicated demo) → Track 1B**
- Friction loss ✓ (urdf-loading/damping-friction)
- **Tendon limits → Track 1B (in tendons/)**

### Model Loading
- MJCF (inline string) ✓
- **MJCF (file with <include>) ✗**
- URDF ✓ (10 examples: revolute, prismatic, continuous, fixed, mimic, geometry, inertia, damping-friction, error-handling, stress-test)

### Inverse Dynamics & Jacobians
- Inverse dynamics ✓ (gravity-comp, torque-profile, forward-replay, stress-test — 4 examples)
- Jacobians (mj_jac_site, velocity mapping) ✓ (jacobian example — stale vs correct comparison)

### Tendon System (0% covered — 1,350 LOC)
- **Fixed tendons → Track 1B**
- **Spatial tendons → Track 1B**
- **Wrapping (sphere, cylinder) → Track 1B**
- **Pulleys → Track 1B**
- **Tendon limits → Track 1B**
- **TendonPos/TendonVel sensors → Track 1B**

### State Management (0% covered)
- **Keyframes (reset-to-keyframe) → Track 1B** ✓ (3 examples, 12 stress-test checks)
- **Mocap bodies (kinematic input) → Track 1B**

### Passive Forces (fully covered)
- Joint spring/damper ✓ (slide-joint, urdf-loading, spring-damper-tuning)
- Gravity compensation ✓ (inverse-dynamics)
- Fluid drag ✓ (fluid-drag, ellipsoid-drag — both inertia-box and ellipsoid models)
- Wind ✓ (wind — on/off demonstration with pendulum + drifter)
- Spring-damper response ✓ (spring-damper-tuning — underdamped/critical/overdamped)
- ImplicitSpringDamper suppression ✓ (stress-test)
- Disable flags ✓ (stress-test)

### Advanced Features
- Energy conservation tracking ✓ (4 visual + 12-check stress-test)
- Sleep / wake / islands ✓ (4 examples, 18 stress-test checks)
- Raycasting ✓ (4 examples, 20 stress-test checks)
- Derivatives ✓ (8 examples, 61 checks: FD linearization, LQR, hybrid vs FD, sensor Jacobians, inverse dynamics, convergence, V-curve)
- **Cable composites → Track 1B**
- Batch simulation ✓ (3 examples, 18 checks: parameter-sweep, soft-landing reset_where, stress-test)
- **Flex bodies (2,797 LOC, beta) → Track 1B**
- **Plugin system (24,398 LOC) → Track 1B**
- **Collision pair isolation (2,600+ LOC) → Track 1B**
- **HillMuscle (1,894 LOC) → Track 1B**
- **Body adhesion (276 LOC) → Track 1B**

### GPU Pipeline
- Full pipeline (GpuPhysicsPipeline::step()) → ZERO examples
- Half-GPU (enable_gpu_collision) → was in old hockey, now cleaned up
- Mocap bodies on GPU → not yet implemented in orchestrator
- Batch/multi-env → architecture ready, not exposed

## Proposed Example Structure

### Track 1A: sim-core fundamentals — COMPLETE (82 examples)

```
fundamentals/
  sim-cpu/
    pendulum-sim/           # DONE — MJCF hinge, energy
    hinge-joint/            # DONE — simple + double pendulum
    slide-joint/            # DONE — horizontal + vertical
    ball-joint/             # DONE — spherical, conical, cone-limit (4 examples)
    sensors/                # DONE — 9 examples, 15+ sensor types, GJK bug fixed
    actuators/              # DONE — 10 examples: motor, servos, damper, filter, cylinder, integrator, gear, site, slider-crank
    muscles/                # DONE — stress-test (51 checks), forearm-flexion, activation, cocontraction, force-length
    solvers/                # DONE — comparison + comparison-visual + 3 per-solver
    integrators/            # DONE — comparison + comparison-visual + 5 per-integrator
    equality-constraints/   # DONE — weld, connect, distance, joint coupling (8 examples)
    contact-tuning/         # DONE — friction, condim, solref, solimp, margin/gap, pair override (7 examples)
    inverse-dynamics/       # DONE — gravity-comp, torque-profile, forward-replay, jacobian, stress-test (5 examples)
    energy-momentum/        # DONE — 4 visual + stress-test (12 checks)
    urdf-loading/           # DONE — 10 examples: all joint types, geometry, mimic, inertia, dynamics, errors
```

---

#### Track 1A Example Descriptions

##### 1. `slide-joint/` — Prismatic Joint, Linear Motion

A mass on a frictionless horizontal rail (slide joint along the X axis) connected
to a spring (joint stiffness) with damping. The mass is displaced from equilibrium
and released. The example visualizes the oscillating block and prints position,
velocity, and energy each second. A second body hangs from a vertical slide joint
(gravity-loaded spring) to show the equilibrium offset `mg/k`. Both joints have
limits enabled — the horizontal slider hits its range boundary and bounces back
via the limit constraint.

**Concepts covered:** `type="slide"`, joint axis, stiffness, damping, joint
limits with `solref`/`solimp`, armature (reflected inertia), linear motion
subspace (S = [0; axis]).

**MJCF sketch:** Two bodies — (A) horizontal slider with `range="-0.5 0.5"`,
`stiffness="20"`, `damping="0.5"`; (B) vertical slider with `stiffness="50"`,
gravity load. Geoms: box on rail, sphere on vertical rod.

**Pass/fail:**
- Horizontal oscillation period within 2% of analytical `T = 2π√(m/k)` (account
  for armature: effective mass = m + armature).
- Vertical equilibrium position within 1% of `mg/k`.
- Energy (KE + PE_spring + PE_gravity) conserved to <0.5% over 10 seconds (with
  low damping). With higher damping, energy monotonically decreases.
- Joint limit activated: position never exceeds range ± solver penetration tolerance.

##### 2. `ball-joint/` — Spherical Joint, 3-DOF Rotation

A heavy sphere on a rigid rod attached to the world via a ball joint — a 3D
spherical pendulum. The sphere is displaced from vertical in both pitch and roll,
then released. It traces a Lissajous-like path on the ground plane. The example
visualizes the swinging sphere and prints the ball joint quaternion (`BallQuat`
sensor), angular velocity (`BallAngVel` sensor), and total energy each second.
A second ball joint has `limited="true"` with `range="0 45"` (45° cone limit) —
the rod is displaced beyond 45° and the cone constraint pushes it back.

**Concepts covered:** `type="ball"`, quaternion representation (4 qpos, 3 qvel),
exponential-map integration, ball joint limits (rotation cone), `BallQuat` and
`BallAngVel` sensors.

**MJCF sketch:** Two bodies — (A) unlimited ball joint, rod + sphere;
(B) limited ball joint (`range="0 45"` degrees), rod + sphere. Geoms: capsule
rods, sphere tips.

**Pass/fail:**
- Quaternion norm stays within 1e-10 of 1.0 (integration preserves unit quaternion).
- Unlimited pendulum: energy conserved to <0.5% over 10 seconds (low damping).
- Limited pendulum: maximum deviation from vertical never exceeds 45° ± solver
  penetration margin after limit activates.
- BallQuat sensor matches `data.qpos[qpos_adr..qpos_adr+4]` exactly.
- BallAngVel sensor matches `data.qvel[qvel_adr..qvel_adr+3]` exactly.

##### 3. `sensors/` — Sensor Gallery

A simple articulated model (hinge arm + ball-joint wrist + free-floating target)
instrumented with one sensor from each major category. The simulation runs for a
few seconds, and the example prints a formatted table of sensor readings every
0.5s. The goal is to show every sensor pipeline stage (position, velocity,
acceleration) and verify readings against manually computed values.

**Sensors demonstrated:**
- **Position stage:** `JointPos`, `FramePos`, `FrameQuat`, `BallQuat`, `Clock`,
  `SubtreeCom`
- **Velocity stage:** `JointVel`, `Gyro`, `Velocimeter`, `FrameLinVel`,
  `FrameAngVel`, `BallAngVel`
- **Acceleration stage:** `Accelerometer`, `Touch`, `Force`, `Torque`,
  `ActuatorFrc`
- **Geometric:** `GeomDist` (between arm tip geom and target geom)

**MJCF sketch:** Hinge arm with site at tip (carries accelerometer, gyro,
velocimeter, touch). Ball-joint wrist. Free-floating target sphere. Contact
between arm tip and ground plane (activates touch sensor). A motor actuator on
the hinge (activates ActuatorFrc). Sites for frame sensors.

**Pass/fail:**
- `Clock` sensor equals `data.time` exactly.
- `JointPos` matches `data.qpos[joint_qpos_adr]`.
- `JointVel` matches `data.qvel[joint_dof_adr]`.
- `FramePos` of tip site matches `data.site_xpos[site_id]`.
- `Accelerometer` at rest reads `[0, 0, +9.81]` (gravity in sensor frame) ± 1%.
- `Gyro` on a spinning body matches analytical angular velocity ± 1%.
- `Touch` sensor > 0 when arm contacts ground, == 0 when in free flight.
- `GeomDist` matches Euclidean distance between geom centers ± solver tolerance.
- All sensor dimensions match expected (1D, 3D, 4D) per type.

##### 4. `actuators/` — Motor Types, Dynamics, and Transmission

Three side-by-side single-hinge arms driven by different actuator types:
(A) `motor` — direct torque control, (B) `position` — PD servo with
`FilterExact` activation dynamics, (C) `velocity` — velocity servo. Each arm
starts horizontal and receives a control signal: motor gets constant torque,
position gets target angle, velocity gets target angular velocity. The example
prints actuator force, activation state (for position servo), and joint angle.

**Concepts covered:** Motor (gain=fixed, bias=none), Position (gain=fixed,
bias=affine, dynamics=FilterExact with timeconst), Velocity (gain=fixed,
bias=affine). Demonstrates `data.ctrl`, `data.act`, `data.actuator_force`,
`data.actuator_length`. Shows how `kp`, `kv`, `dampratio`, and `timeconst`
parameters shape the response.

**MJCF sketch:** Three independent hinge arms.
- Arm A: `<motor joint="j1" gear="1"/>`
- Arm B: `<position joint="j2" kp="50" dampratio="1" timeconst="0.1"/>`
- Arm C: `<velocity joint="j3" kv="10"/>`

**Pass/fail:**
- Motor arm: constant torque → constant angular acceleration `α = τ/I` ± 2%.
- Position servo: reaches target angle within 5% after 5× time constant.
  Activation `data.act` follows first-order filter `(1 - e^(-t/τ))`.
- Velocity servo: reaches target velocity within 5% in steady state.
- `actuator_force[i]` = gain × activation + bias, verified against manual calc.
- All three arms produce physically distinct motion profiles.

##### 5. `muscles/` — DONE (5 examples)

Five examples covering the full muscle actuator system — both MuJoCo `<muscle>`
(conformant) and HillMuscle (extension). Spec: `muscles/MUSCLE_EXAMPLES_SPEC.md`.

1. **stress-test** — Headless validation: 51 checks across 6 groups (activation
   dynamics, MuJoCo curve pinning, Hill curve pinning, full-pipeline force,
   HillMuscle pipeline, dynamic behavior). All pass.
2. **forearm-flexion** — MuJoCo `<muscle>` lifts forearm against gravity.
   3D tendon mesh colored by tension (blue→red). Flex/release cycle.
3. **activation** — Three arms with different time constants (5/20, 10/40,
   50/200 ms). Arms contract together, release at different rates —
   demonstrates Millard et al. activation-dependent deactivation asymmetry.
4. **cocontraction** — Agonist-antagonist pair (gear=±1). Cycles through
   cocontraction (both red, joint stiff), agonist-only, antagonist-only.
   Symmetric swings, gravity restores to vertical.
5. **force-length** — Two arms swept sinusoidally: wide FL range (lmin=0.5,
   lmax=1.6) vs narrow (lmin=0.8, lmax=1.2). Same motion, different force
   output visible as tendon color difference.

##### 6. `solvers/` — PGS vs CG vs Newton Comparison

Three identical scenes (a box resting on a plane with a second box stacked on
top) simulated with different constraint solvers: PGS, CG, and Newton. The
example runs each for 2 seconds, then prints solver statistics: iterations to
convergence, final cost, constraint violation, and wall-clock time. Visualizes
all three side by side.

**Concepts covered:** `solver="PGS|CG|Newton"` in `<option>`, solver iterations,
tolerance, line search (Newton/CG only), `SolverStat` (improvement, gradient,
nactive, nchange), warmstart. Demonstrates that Newton converges in 2–3
iterations vs PGS needing 20+.

**MJCF sketch:** `<option solver="PGS" iterations="100" tolerance="1e-8"/>`
(and CG/Newton variants). Two stacked boxes on a ground plane. Moderate friction
(`condim="3"`, `friction="0.5"`).

**Pass/fail:**
- All three solvers produce a stable stack (no drift > 1mm over 2 seconds).
- Newton converges in ≤5 iterations per step on average.
- PGS converges in ≤50 iterations per step (or hits max with acceptable residual).
- CG converges faster than PGS but slower than Newton.
- Final constraint violation (penetration depth) < 1mm for all solvers.
- Solver statistics printed match `data.solver_stat` fields.

##### 7. `integrators/` — Euler vs RK4 vs Implicit Comparison

A single undamped pendulum simulated with four integrators: Euler, RK4,
ImplicitFast, and Implicit. Each runs for 20 seconds with energy tracking
enabled. The example prints total energy at 1-second intervals for each
integrator, showing energy drift characteristics. Euler drifts visibly, RK4
stays tight, Implicit variants stay stable.

**Concepts covered:** `integrator="Euler|RK4|implicit|implicitfast"` in
`<option>`, `ENABLE_ENERGY` flag, quaternion normalization during integration,
energy conservation as accuracy metric, computational cost tradeoffs (RK4 = ~4×
Euler cost).

**MJCF sketch:** Single hinge pendulum (identical to existing `pendulum-sim`
but single link, zero damping). `<option timestep="0.005"/>` to make drift
visible on Euler.

**Pass/fail:**
- RK4: energy drift < 0.01% over 20 seconds.
- Euler: energy drift > 0.1% over 20 seconds (demonstrably worse than RK4).
- ImplicitFast: energy drift < 0.1% over 20 seconds (stable, slight dissipation).
- Implicit: energy drift < 0.1% over 20 seconds.
- All integrators: quaternion norm (for any ball/free joints) within 1e-10 of 1.0.
- Period of oscillation within 1% of analytical `T = 2π√(L/g)` for all
  integrators.

##### 8. `equality-constraints/` — Weld, Connect, Distance

Three constraint demos in one scene: (A) two bodies welded together (6-DOF lock)
— push one, both move as a unit; (B) two bodies connected at a point (3-DOF
ball-socket) — they share a pivot but rotate independently; (C) two free-floating
spheres with a distance constraint — they maintain fixed separation like a rigid
rod. All bodies have free joints. The example applies external forces and
visualizes the constrained motion.

**Concepts covered:** `<equality>` element with `<weld>`, `<connect>`,
`<distance>`. Constraint solver reference parameters (`solref`), constraint
force extraction (`efc_force`). Shows the difference between removing 6 DOF
(weld), 3 DOF (connect), and 1 DOF (distance).

**MJCF sketch:** Three pairs of free-floating bodies above a ground plane.
- Pair A: `<weld body1="a1" body2="a2" relpose="0 0.3 0 1 0 0 0"/>`
- Pair B: `<connect body1="b1" body2="b2" anchor="0 0 0"/>`
- Pair C: `<distance geom1="c1_geom" geom2="c2_geom" distance="0.5"/>`

**Pass/fail:**
- Weld: relative pose between bodies stays constant (position error < 0.1mm,
  rotation error < 0.01 rad) over 5 seconds.
- Connect: anchor points on both bodies coincide within 0.1mm; bodies rotate
  independently (angular velocities differ).
- Distance: Euclidean distance between geom centers stays within 1% of target
  distance over 5 seconds.
- Constraint forces (`efc_force`) are non-zero when constraints are active.

##### 9. `contact-tuning/` — Contact Parameter Tuning (7 examples)

One example per concept, each isolating a single contact parameter:

1. **friction-slide** — Boxes on 15° tilted plane with μ=0.1, 0.5, 1.0.
   Low friction slides (μ < tan(15°)), high friction holds.
2. **condim-compare** — Spheres with condim=1 (frictionless), 3 (rolling),
   6 (rolling resistance) on tilted plane.
3. **solref-bounce** — Spheres dropped from 0.5m with same K=5000, different
   damping B=10/30/500 (bouncy/moderate/absorbing).
4. **pair-override** — Two identical boxes, one with `<pair>` friction
   override. Demonstrates override replaces auto-combined values.
5. **solimp-depth** — Heavy balls with different impedance curves (d0=0.9
   vs d0=0.1). Low d0 sinks ~19mm (foam), high d0 barely penetrates (steel).
6. **margin-gap** — Balls with margin=0/0.02/0.05. Larger margin = ball
   floats higher above the surface.
7. **stress-test** — Headless validation of all parameters (26 checks).

##### 10. `inverse-dynamics/` — DONE (5 examples)

Five examples covering `data.inverse()` and `mj_jac_site()`, all using the
same two-link planar arm (shoulder + elbow hinge, motor gear=1, no contacts):

1. **gravity-compensation** — Static holding torques (`qacc=0`, `qvel=0`).
   Verifies `qfrc_inverse == qfrc_bias`, arm holds pose < 0.001 rad over 5s.
2. **torque-profile** — Dynamic inverse for sinusoidal trajectory. Verifies
   smooth torque profile, shoulder peak > elbow peak.
3. **forward-replay** — Round-trip: replay inverse torques in forward sim.
   Tracking < 0.02 rad over 5s. Torques disengage at t=5s, arm falls.
4. **jacobian** — `mj_jac_site` velocity mapping. Two arms: stale Jacobian
   (left) diverges, correct Jacobian (right) matches FD within 0.3%.
5. **stress-test** — Headless validation: 14 checks covering formula,
   round-trip, free-fall, gravity-comp, tracking, Jacobian, torque profile,
   and body accumulators. All pass.

##### 11. `energy-momentum/` — DONE (5 examples)

Four visual examples + stress-test (12 checks), one concept per example:

1. **free-flight** — Zero-gravity free body (asymmetric inertia). KE, linear
   momentum, angular momentum all conserved to machine precision (~1e-17).
   Angular velocity precesses (torque-free precession) but |L| is constant.
2. **pendulum-energy** — Undamped hinge pendulum. PE ↔ KE exchange with
   total energy flat. KE/PE_drop = 1.0 at bottom swing.
3. **elastic-bounce** — Ball dropped onto elastic plane. Energy dips during
   contact (contact spring PE untracked), recovers at apex. Restitution 0.97.
4. **damped-decay** — Damped pendulum (damping=0.05). Energy monotonically
   decreasing (0 violations). KE decays from 0.459 J to 0.000 J.
5. **stress-test** — 12 headless checks: free-flight (4), pendulum (2),
   bounce (2), damped (2), flag disabled (1), multi-body (1). All pass.

##### 12. `urdf-loading/` — DONE (10 examples)

One concept per example, 8 visual (Bevy) + 1 headless error-handling + 1
headless stress-test (31 checks). Covers the full `sim_urdf` pipeline:

1. **revolute** — Hinge pendulum, period matches analytical compound-pendulum
   formula. `track_period` + `track_energy` validation.
2. **prismatic** — Slide joint with spring stiffness (MJCF augmentation).
   Period matches T = 2pi*sqrt(m/k). Zero-crossing period measurement.
3. **continuous** — Unlimited revolute wheel. Constant torque via
   `qfrc_applied`, velocity ramps linearly (alpha = tau/I).
4. **fixed** — Three URDF links, one fixed joint. `fusestatic` merges the
   fixed link — model has fewer bodies than URDF links.
5. **mimic** — Two arms coupled 2:1 via `<mimic>` → `<equality><joint
   polycoef>`. Follower tracks 2*leader + 0.1, max error < 0.0001 rad.
6. **geometry** — Box + cylinder + sphere on one arm. Verifies URDF
   full-extents → MJCF half-extents size conversion.
7. **inertia** — Two free-floating boxes (combined MJCF): diagonal
   (`diaginertia`) vs off-diagonal (`fullinertia`). Different precession
   patterns from same initial angular velocity.
8. **damping-friction** — Three colored pendulums (combined MJCF): no-loss
   (blue, swings forever), damped (green, decays smoothly), friction (red,
   decays then stops). URDF `<dynamics friction>` → MJCF `frictionloss`.
9. **error-handling** — Headless. 7 checks: missing robot, unknown joint
   type, undefined link, duplicate link, malformed XML, empty string, error
   message context.
10. **stress-test** — Headless. 31 checks: structural equivalence, dynamic
    equivalence, all joint types, geometry sizes, inertia, limits, damping,
    frictionloss, mimic tracking, planar DOF, mesh conversion, error variants.

### Track 1B: sim-core foundation layer 2 (~70 examples)

The next layer of sim-cpu fundamentals. No stone unturned — every implemented
subsystem gets dedicated coverage. Every feature below has a real implementation
in sim-core (LOC noted). Examples double as integration tests that find bugs
unit tests miss.

Ordered ground-up by dependency: basic concepts first, advanced features last.

```
fundamentals/
  sim-cpu/
    # --- Layer 1: Missing fundamentals ---
    free-joint/             # DONE — 4 examples, 12 stress-test checks
    keyframes/              # DONE — 3 examples, 12 stress-test checks
    mocap-bodies/           # DONE — 4 examples, 12 stress-test checks

    # --- Layer 2: Filtering and limits ---
    contact-filtering/      # DONE — 4 examples, 12 stress-test checks
    joint-limits/           # DONE — 4 examples, 12 stress-test checks

    # --- Layer 3: Tendons ---
    tendons/                # DONE — 8 examples, 16 stress-test checks

    # --- Layer 4: Sensors completion ---
    sensors-advanced/       # DONE — 7 examples, 25 stress-test checks

    # --- Layer 5: Geometry types ---
    mesh-collision/         # Explicit convex mesh collision (2,090 LOC)
    heightfield/            # Terrain collision (441 LOC)

    # --- Layer 6: Passive forces and dynamics ---
    passive-forces/         # DONE — 5 examples, 18 stress-test checks
    sleep-wake/             # DONE — 4 examples, 18 stress-test checks

    # --- Layer 7: Muscles and actuators (remaining) ---
    hill-muscle/            # HillMuscle extension (1,894 LOC)
    adhesion/               # Body transmission — adhesive/magnetic actuation (276 LOC)

    # --- Layer 8: Deformable bodies ---
    flex-bodies/            # Soft bodies, cloth, cables, self-collision (2,797 LOC)

    # --- Layer 9: Advanced ---
    raycasting/             # DONE — 4 examples, 20 stress-test checks
    collision-pairs/        # All primitive pair functions isolated (2,600+ LOC)
    derivatives/            # DONE — 8 examples (7 visual + 1 stress-test), 61 checks
    composites/             # DONE — 4 examples (3 visual + 1 stress-test), 15 checks
    batch-sim/              # DONE — 3 examples (2 visual + 1 stress-test), 18 checks
    plugins/                # Plugin system — custom sensors/forces (24,398 LOC)
```

---

#### Track 1B Example Descriptions

##### 1. `free-joint/` — 6-DOF Floating Body

The only joint type without a dedicated example. A free joint gives a body full
translational and rotational freedom — 7 qpos (3 position + 4 quaternion), 6
qvel (3 linear + 3 angular). This is the joint type used for every untethered
object in the engine.

**Examples:**

1. **tumble** — A box with asymmetric inertia (Ixx ≠ Iyy ≠ Izz) launched with
   angular velocity in zero gravity. Demonstrates torque-free precession —
   angular velocity vector precesses around the angular momentum vector, but
   |L| is exactly conserved. Visualizes the body tumbling and prints quaternion,
   angular velocity, and angular momentum each second.

2. **projectile** — A sphere launched at 45° in gravity. Parabolic trajectory
   with exact analytical solution. Demonstrates the 3-DOF translational
   subspace of a free joint. Prints position vs analytical trajectory.

3. **coupled-rotation** — Two free-floating bodies connected by a weld
   constraint. Push one, both move as a rigid unit. Demonstrates that free
   joints + equality constraints compose correctly.

4. **stress-test** — Headless validation (12+ checks):
   - Quaternion norm preserved to 1e-10 over 10s
   - Linear momentum conserved in zero-g (< 1e-12)
   - Angular momentum conserved in zero-g (< 1e-12)
   - Projectile apex height within 0.1% of v²sin²θ/(2g)
   - Projectile range within 0.1% of v²sin(2θ)/g
   - qpos dimension = 7, qvel dimension = 6
   - Free joint has no limits (unlimited always)
   - Gravity: linear momentum changes at rate mg, angular unchanged
   - Multiple free bodies don't interact without contact
   - Quaternion integration matches exponential map

**Concepts covered:** `type="free"`, 7-DOF qpos (pos + quat) vs 6-DOF qvel
(lin + ang), quaternion integration on SO(3), torque-free precession,
conservation laws, exponential map.

##### 2. `keyframes/` — State Snapshots and Reset

Keyframes are named state snapshots stored in the model (`<key>` elements in
MJCF). They let you save and restore complete simulation states — position,
velocity, activation, control, and mocap state.

**Examples:**

1. **save-restore** — A pendulum swings for 2 seconds, then resets to a named
   keyframe (arm horizontal, zero velocity). The arm snaps back to the saved
   pose and restarts. Cycles through 3 keyframes: rest, horizontal, inverted.
   Prints state before/after each reset.

2. **multi-keyframe** — A robot arm with 3 keyframes representing different
   poses (home, reach-left, reach-right). Cycles through them every 2 seconds.
   Each reset is exact — qpos matches keyframe qpos to machine precision.
   Demonstrates keyframe with `ctrl` and `act` fields populated.

3. **stress-test** — Headless validation (10+ checks):
   - qpos matches keyframe qpos exactly after reset
   - qvel matches keyframe qvel exactly after reset
   - act matches keyframe act exactly after reset
   - ctrl matches keyframe ctrl exactly after reset
   - time resets to keyframe time
   - Derived quantities (qacc, cvel) cleared after reset
   - Multiple keyframes indexed correctly
   - Keyframe with zero-length fields handled gracefully
   - Reset mid-simulation produces clean state
   - Simulation runs normally after reset

**Concepts covered:** `<key>` MJCF element, `Data::reset_to_keyframe()`,
qpos/qvel/act/ctrl state vectors, named poses, simulation checkpoint/restart.

##### 3. `mocap-bodies/` — Kinematic Input Bodies

Mocap (motion capture) bodies are world-attached bodies whose pose is set
directly via `data.mocap_pos` and `data.mocap_quat` rather than computed by
the integrator. They're the mechanism for user input, teleop, animation
playback, and VR controllers.

**Examples:**

1. **drag-target** — A mocap sphere moves on a sinusoidal path. A free-floating
   box is connected to the mocap body via a weld constraint with compliance
   (soft weld). The box follows the target with spring-like lag. Demonstrates
   the basic mocap → constraint → dynamic body pipeline.

2. **push-object** — A mocap paddle sweeps across the scene and pushes a
   free-floating ball through contact. The mocap body has geometry (capsule)
   that generates contacts with dynamic bodies. Shows that mocap bodies
   participate in collision but not in dynamics.

3. **stress-test** — Headless validation (8+ checks):
   - Mocap body position tracks mocap_pos exactly (zero error)
   - Mocap body quaternion tracks mocap_quat exactly
   - Mocap body is always world-child (parent = 0)
   - Mocap body generates contacts with dynamic bodies
   - Mocap body not affected by gravity or contact forces
   - Keyframe reset restores mocap_pos and mocap_quat
   - Multiple mocap bodies independent of each other
   - Mocap body with zero mass still works (FK override)

**Concepts covered:** `mocap="true"` in MJCF body, `data.mocap_pos`,
`data.mocap_quat`, kinematic vs dynamic bodies, weld-to-mocap for soft
tracking, mocap + contact for pushing/interaction.

##### 4. `contact-filtering/` — Collision Bitmasks and Exclusion

Controls which geometry pairs can collide. Three mechanisms: bitmask filtering
(`contype`/`conaffinity`), parent-child exclusion (automatic for jointed
bodies), and explicit pair exclusion (`<exclude>`).

**Examples:**

1. **bitmask** — Four spheres falling onto a plane. Spheres have different
   contype/conaffinity bitmasks: (A) collides with everything, (B) collides
   with plane only, (C) collides with other spheres only, (D) collides with
   nothing. The bitmask rule: contact if `(c1 & a2) != 0 || (c2 & a1) != 0`.
   Sphere D falls through the ground.

2. **exclude-pairs** — Two overlapping boxes that would normally collide, but
   an `<exclude body1="a" body2="b"/>` suppresses their contact. A third box
   with no exclusion collides normally. Shows explicit pair exclusion vs
   automatic parent-child exclusion.

3. **ghost-layers** — A multi-layer scene: "solid" layer (contype=1,
   conaffinity=1) and "sensor" layer (contype=2, conaffinity=2). Solid objects
   collide with each other and with sensor objects, but sensor objects pass
   through each other. Demonstrates using bitmasks for selective physics layers.

4. **stress-test** — Headless validation (10+ checks):
   - Bitmask AND rule verified for all 16 combinations of 4-bit types
   - Parent-child exclusion: jointed bodies don't self-collide
   - World body (body 0) geoms exempt from parent-child exclusion
   - Explicit exclude suppresses contact force to zero
   - Exclude is bidirectional (body1/body2 order doesn't matter)
   - contype=0 body collides with nothing
   - conaffinity=0 body collides with nothing
   - Default contype=conaffinity=1 collides with everything
   - Explicit `<pair>` bypasses bitmask filtering
   - Contact count matches expected after filtering

**Concepts covered:** `contype`, `conaffinity`, `<exclude>`, parent-child
auto-exclusion, `<pair>` override, bitmask AND rule, collision layers.

##### 5. `joint-limits/` — Limit Constraints with Solver Tuning

Joint limits constrain the range of motion. When a joint reaches its limit, the
solver generates a one-sided constraint. The stiffness and damping of the limit
response are tuned via `solref` and `solimp` — the same solver parameters used
for contacts.

**Examples:**

1. **hinge-limits** — A pendulum with `range="-45 45"` degrees. Released from
   60° (beyond limit). The limit constraint pushes it back. Three variants
   side by side: stiff limit (high solref K), soft limit (low K, visible
   penetration), and overdamped limit (high B, no bounce).

2. **slide-limits** — A mass on a rail with `range="-0.3 0.3"`. Driven by a
   motor that pushes it into the limit. Shows limit force via `JointLimitFrc`
   sensor. Motor torque increases but position stays clamped.

3. **ball-cone** — A ball joint with `range="0 30"` (30° cone limit). Rod
   is pushed beyond the cone in different directions. The cone constraint is
   rotationally symmetric — same limit force regardless of azimuthal angle.

4. **stress-test** — Headless validation (10+ checks):
   - Position never exceeds range ± solver penetration tolerance
   - JointLimitFrc > 0 when at limit, == 0 when interior
   - Limit force increases with penetration depth
   - Solref stiffness scales limit force linearly
   - Solimp width controls penetration depth at equilibrium
   - Hinge, slide, and ball limits all activate correctly
   - Limit is one-sided (no constraint when interior)
   - Limit + motor: motor cannot push past limit
   - Symmetric ball cone: limit force magnitude independent of azimuth
   - Zero-width range (locked joint) holds position exactly

**Concepts covered:** `limited="true"`, `range`, `jnt_solref`, `jnt_solimp`,
`JointLimitFrc` sensor, one-sided constraints, limit + motor interaction,
ball joint cone limits.

##### 6. `tendons/` — Routing, Wrapping, and Limits

The tendon system (1,350 LOC) provides two mechanisms: fixed tendons (linear
combinations of joint positions) and spatial tendons (3D paths routed through
sites with optional wrapping around geometry). Tendons can have limits,
spring-damper dynamics, and serve as actuator transmission.

**Examples:**

1. **fixed-coupling** — Two hinge joints coupled by a fixed tendon with
   coefficients [1.0, -0.5]. When joint A moves 1 rad, joint B is pulled
   0.5 rad in the opposite direction. Tendon length = coef1*q1 + coef2*q2.
   Print TendonPos sensor vs manual calc. Demonstrates the simplest tendon.

2. **spatial-path** — A spatial tendon routed through 3 sites on a 2-link
   arm. As the arm moves, the tendon length changes. Visualize the 3D tendon
   path (line segments between sites). TendonPos tracks the path length.
   Compare against manual distance calculation.

3. **sphere-wrap** — A spatial tendon that wraps around a sphere (representing
   a bone condyle or pulley wheel). The tendon takes the shortest path that
   doesn't penetrate the sphere. As the joint angle changes, the wrap arc
   length changes, producing a moment arm that varies with configuration.
   Visualize wrap points on the sphere surface.

4. **cylinder-wrap** — Same concept as sphere-wrap but wrapping around a
   cylinder (representing a tendon sheath or muscle routing). The tendon
   follows a geodesic on the cylinder surface. Demonstrates the difference
   between sphere wrap (great-circle arc) and cylinder wrap (helix).

5. **tendon-limits** — A spatial tendon with `limited="true"` and
   `range="0.1 0.5"`. When the arm configuration would stretch the tendon
   beyond 0.5, the limit constraint activates and resists further motion.
   When compressed below 0.1, the lower limit activates. Print TendonLimitFrc
   sensor readings.

6. **pulley** — A spatial tendon with a pulley (divisor > 1). Two branches
   of the tendon contribute to length with different mechanical advantage.
   Demonstrates the `<pulley divisor="2"/>` element.

7. **tendon-actuator** — A motor with `<general tendon="t1">` transmission.
   The actuator drives the tendon, which in turn drives the joints. Compare
   against joint-transmission equivalent. Shows that tendon transmission
   creates configuration-dependent effective gear ratio for spatial tendons.

8. **stress-test** — Headless validation (16+ checks):
   - Fixed tendon length = Σ coef_i * q_i exactly
   - Fixed tendon velocity = Σ coef_i * dq_i exactly
   - TendonPos sensor matches data.ten_length
   - TendonVel sensor matches data.ten_velocity
   - Spatial tendon length matches point-to-point distance (no wrapping)
   - Sphere wrap: tendon doesn't penetrate sphere (distance ≥ radius)
   - Cylinder wrap: tendon follows geodesic
   - Tendon limit activates when length exceeds range
   - TendonLimitFrc > 0 at limit, == 0 interior
   - Pulley divisor scales branch contribution correctly
   - Tendon-driven actuator produces correct force
   - Tendon spring/damper passive force matches analytical
   - Fixed tendon Jacobian is constant (configuration-independent)
   - Spatial tendon Jacobian varies with configuration
   - Multiple tendons on same joint compose correctly
   - Spatial tendon moment arm varies with configuration (Jacobian check)

**Concepts covered:** `<tendon><fixed>`, `<tendon><spatial>`, `<site>` routing,
sphere/cylinder wrapping, `<pulley>`, tendon limits, `TendonPos`/`TendonVel`/
`TendonLimitFrc` sensors, tendon-driven actuators, moment arms, Jacobian.

##### 7. `sensors-advanced/` — DONE (7 examples, 25 stress-test checks)

All remaining sensor types not covered in Track 1A `sensors/`. One concept
per example, each with analytical validation. Tendon sensors (TendonPos,
TendonVel, TendonLimitFrc) and JointLimitFrc omitted here — already covered
by `tendons/` and `joint-limits/`. ActuatorPos/Vel added as a replacement.

1. **frame-velocity** — Spinning rod (ω=2π, R=0.5m). FrameLinVel |v|=π,
   FrameAngVel ω_z=2π. Both at 0.000% error.
2. **frame-acceleration** — Pendulum held 3s then released. FrameLinAcc
   at rest = [0,0,+g]. FrameAngAcc sign tracks cos(θ).
3. **force-torque** — Static beam (m=1kg, L=0.8m). Force |F|=mg at 0.1%,
   Torque |τ|=mgL/2 at 0.2%.
4. **rangefinder** — Oscillating body above ground. Distance tracks height
   at zero error. Side sensor reads −1 every frame.
5. **subtree-velocity** — 3-link chain free-fall. SubtreeLinVel v_z = −gt
   at 0.3%. 2-second hover before drop.
6. **subtree-angmom** — Single body spinning in zero-g. SubtreeAngMom |L|
   conserved to 3.2e-16 (machine epsilon).
7. **actuator-pos-vel** — Gear=2 servo. ActuatorPos = 2×JointPos at exact
   zero error.
8. **stress-test** — 25 headless checks: FrameLinVel (3), FrameAngVel (2),
   FrameLinAcc (3), FrameAngAcc (2), Force/Torque (3), Rangefinder (3),
   SubtreeLinVel (2), SubtreeAngMom (1), ActuatorPos/Vel (2), Frame*Axis (1),
   Magnetometer (1), sensor dimensions (1).

**Concepts covered:** FrameLinVel, FrameAngVel, FrameLinAcc, FrameAngAcc,
Force, Torque, Rangefinder, SubtreeLinVel, SubtreeAngMom, ActuatorPos,
ActuatorVel, FrameXAxis, FrameYAxis, FrameZAxis, Magnetometer.

##### 8. `passive-forces/` — Fluid Drag and Damping

Passive forces are configuration/velocity-dependent forces computed
automatically by the engine — no actuator needed. Joint springs and gravity
compensation are already demonstrated. This covers the remaining passive force
types.

**Examples:**

1. **fluid-drag** — A sphere falling through a medium with density (viscosity).
   Without drag: free-fall acceleration = g. With drag: terminal velocity
   = sqrt(2mg/(ρCdA)). Three spheres with different drag coefficients
   side by side — heavy one falls fast, light one reaches terminal velocity
   quickly. Demonstrates `<option density="1.2" viscosity="1.5e-5"/>` and
   per-geom fluid interaction.

2. **spring-damper-tuning** — Three identical pendulums with different
   joint stiffness and damping combinations: (A) stiff spring, low damping
   (fast oscillation, slow decay), (B) soft spring, high damping (slow,
   overdamped), (C) critical damping (ζ=1, fastest return without overshoot).
   Print natural frequency and damping ratio. Verify against analytical
   ω_n = √(k/I), ζ = c/(2√(kI)).

3. **stress-test** — Headless validation (10+ checks):
   - Terminal velocity matches analytical within 5%
   - Drag force proportional to v² at high Re
   - Zero density/viscosity = zero drag force
   - Spring restoring force = k*Δq
   - Damper dissipative force = c*dq
   - Critical damping: no overshoot, reaches 98% in 4/ω_n seconds
   - Underdamped: oscillation frequency = ω_n√(1-ζ²)
   - Overdamped: exponential decay, no oscillation
   - Gravity compensation: body holds pose with zero actuator force
   - Spring + gravity: equilibrium offset = mg/(k)

**Concepts covered:** `<option density="" viscosity=""/>`, per-geom fluid
parameters, joint `stiffness` and `damping`, critical/under/overdamped
response, natural frequency, damping ratio, terminal velocity.

##### 9. `sleep-wake/` — Island-Based Deactivation ✅

The sleep system (1,415 LOC) groups bodies into constraint islands and
deactivates islands whose velocities fall below a threshold. Sleeping bodies
skip integration and collision narrowphase — essential for large scenes with
many resting objects.

**Examples (4 — all implemented):**

1. **sleep-settle** — 5 boxes drop from staggered heights and turn blue one by
   one as they settle below the velocity threshold and sleep. Pure sleep
   mechanism: threshold + countdown + transition. (4 checks)

2. **wake-on-contact** — A box with `sleep="init"` sits on a plane (frozen,
   blue). A ball drops from above — contact wakes the box (turns orange).
   Demonstrates init-sleep freeze and contact-triggered wake. (5 checks)

3. **island-groups** — Two stacks of 3 boxes, 3 m apart. Both settle and sleep.
   An impulse disturbs one stack — only that stack wakes. The other stays
   asleep. Demonstrates island independence. (5 checks)

4. **stress-test** — Headless validation (18 checks):
   - Sleep threshold, qvel/qacc/qfrc zeroing, countdown duration
   - Wake on contact, equality, xfrc_applied, cascade
   - Island singletons, shared islands, island count, selective wake
   - Countdown reset, sleep policy Never, ENABLE_SLEEP disabled
   - Narrowphase skip, nbody_awake bookkeeping

**Concepts covered:** `SleepPolicy`, sleep threshold, countdown timer,
constraint islands, wake-on-contact, wake-on-equality, island membership,
`tree_asleep`, `nisland`, `nbody_awake`, `sleep="init"`, narrowphase skip.

##### 10. `raycasting/` — Ray Queries and Shape Intersection ✅

The raycast module supports ray intersection queries against all geometry
types. Used internally by the Rangefinder sensor but also available as a
direct API for visibility checks, distance queries, and custom sensors.

**Examples (4 — all implemented):**

1. **stress-test** ✅ — Headless validation (20 checks):
   - Per-shape analytical intersection: sphere, plane, box, capsule,
     cylinder, ellipsoid, heightfield, convex mesh (10 checks)
   - Surface normal properties: unit-length, faces ray, perpendicular (3)
   - Scene-level queries: nearest hit, body exclude, geom group filter,
     scene miss (4)
   - Edge cases: inside sphere, max distance clamp, zero max distance (3)

2. **basic-shapes** ✅ — 6 primitives (sphere, box, capsule, cylinder,
   ellipsoid, plane) struck by downward rays. Green hit dots + yellow
   normal arrows. HUD shows distance and normal per shape. 5 checks.

3. **scene-query** ✅ — LIDAR fan of 36 rays from a central eye. Cyan hit
   lines, red miss lines. Body exclusion demonstrated. HUD shows hit/miss
   counts, nearest/farthest geom. 4 checks.

4. **heightfield** ✅ — 8×8 ray grid onto sinusoidal terrain. MJCF hfield
   loaded and replaced with `HeightFieldData::from_fn`. Dot cloud traces
   the surface. 3 checks (hit rate, z-accuracy, gradient normals).

**Concepts covered:** `raycast_shape`, `raycast_scene`, `RaycastHit`,
`SceneRayHit`, analytical intersection (sphere, plane, box, capsule,
cylinder, ellipsoid), ray marching (heightfield), convex mesh hull test,
body exclusion, geom group filtering, surface normals.

##### 11. `derivatives/` — Finite-Difference Linearization

The derivatives module (6,029 LOC) computes the linearized dynamics
A, B, C, D matrices via finite-difference perturbation or hybrid
analytical/FD methods. These matrices are the foundation for LQR, MPC,
system identification, and any control design workflow.

**Examples:**

1. **linearize-pendulum** — A single hinge pendulum. Compute A, B matrices
   at the upright equilibrium. The A matrix eigenvalues reveal the unstable
   mode (positive real eigenvalue = √(g/L)). Print the 2×2 A matrix and
   verify eigenvalues match analytical. Demonstrates the simplest use of
   `mjd_transition_fd()`.

2. **control-design** — A cart-pole (slide + hinge). Compute A, B at the
   unstable equilibrium. Design an LQR controller from the linearization
   (K = lqr(A, B, Q, R)). Run the closed-loop simulation — the cart-pole
   balances. Without control, it falls. Demonstrates the full pipeline:
   linearize → design → simulate.

3. **hybrid-vs-fd** — Compare pure FD vs hybrid derivatives on a 3-link arm.
   Both should produce the same A, B matrices (within perturbation tolerance).
   Print timing: hybrid is ~2× faster because it uses analytical velocity
   derivatives instead of perturbing every DOF (`mjd_transition_hybrid()`).

4. **stress-test** — Headless validation (12+ checks):
   - A matrix eigenvalues match analytical for simple pendulum
   - B matrix matches gain for motor-driven joint
   - FD convergence: halving epsilon halves the error (O(ε²) for centered)
   - Hybrid matches FD within 1e-6
   - A matrix for stable system has all eigenvalues with negative real part
   - A matrix for unstable system has positive eigenvalue
   - Sensor derivatives (C, D) match FD perturbation
   - Free joint: A is 12×12 (6 pos + 6 vel), B matches
   - Ball joint: quaternion chain rule handled correctly
   - Zero-control linearization matches passive dynamics
   - Multiple actuators: B has correct column count
   - Transition matrices A*dt + I approximate one-step evolution

**Concepts covered:** `mjd_transition_fd()`,
`mjd_transition_hybrid()`, A/B/C/D state-space matrices, eigenvalue
analysis, LQR control design, perturbation convergence, quaternion chain
rules, computational cost comparison.

##### 12. `composites/` — Cable Composite Bodies — DONE

The composite system expands a single `<composite>` MJCF element into a chain
of bodies with joints, geoms, and contact exclusions. Cable composites generate
a chain of capsule bodies connected by ball joints — useful for ropes, cables,
and flexible tubes.

**Examples (4 total, 15 checks):**

1. **stress-test** — Headless validation (11 checks): body count, joint
   count/types, contact exclusions, gravity hang, convergence, length
   preservation, cosine curve shape, multi-cable independence.

2. **hanging-cable** — Three cables (5/10/20 segments) pinned at one end,
   hanging under gravity. Resolution comparison — more segments = smoother
   catenary. 5 checks.

3. **cable-catenary** — Gold cable spanning two pylons, pinned at both ends
   (ball joint left, connect constraint right). Half-sine initial bulge for
   slack, sags into catenary. 4 checks.

4. **cable-loaded** — Cyan cable spanning two pylons with 5N downward force
   at midpoint via `xfrc_applied`. V-shaped sag deeper than passive catenary.
   Red sphere tracks the load point. 4 checks.

**Concepts covered:** `<composite type="cable">`, segment count, joint
parameters, contact exclusion generation, catenary shape, `xfrc_applied`,
equality connect constraints, curve types (line, sine).

##### 13. `batch-sim/` — Parallel Multi-Environment Simulation — DONE

The batch system (600 LOC) runs N independent copies of the same model in
parallel using shared-nothing `Data` instances. Essential for reinforcement
learning, Monte Carlo sampling, and parameter sweeps.

**Examples (3 examples, 18 checks):**

1. **parameter-sweep** (visual, 4 checks) — 8 pendulums with different
   ctrl-based damping (0.0 to 0.32). All step in parallel via `step_all()`.
   Undamped pendulum conserves energy (RK4); highest damping settles quickly.

2. **reset-subset** (visual, 4 checks) — "Soft Landing": 12 landers on
   vertical slide joints descend under gravity with different thrust levels.
   Too little → crash, too much → hover, just right → soft landing (green).
   `reset_where(mask)` resets failures; thrust adapts each generation.
   All 12 converge over ~30 seconds.

3. **stress-test** (headless, 10 checks) — Construction, independent state,
   parallel stepping, cross-contamination isolation, bitwise determinism,
   `reset(i)`, `reset_where`, `reset_all`, shared model, single-env parity.

**Concepts covered:** `BatchSim`, `step_all()`, `reset_where()`, `reset(i)`,
`reset_all()`, shared Model, independent Data, parameter sweeps, selective
reset with adaptation, `sync_batch_geoms` infrastructure.

##### 14. `mesh-collision/` — Explicit Convex Mesh Collision

The mesh module (2,090 LOC) provides BVH-accelerated triangle mesh collision
against all primitive types. Mesh geoms are used for realistic object shapes
imported from CAD or 3D modeling tools — robot links, furniture, terrain
features, anything that isn't a sphere/box/capsule/cylinder.

**Examples:**

1. **mesh-on-plane** — A convex mesh (tetrahedron defined by 4 triangles)
   dropped onto a ground plane. The mesh rests stably on one face. Compare
   rest height against analytical (centroid height of the resting face).
   Demonstrates `type="mesh"` geom with inline vertex/face data.

2. **mesh-on-mesh** — Two convex meshes (a wedge and a block) dropped onto
   each other. The mesh-mesh narrow phase uses BVH queries for triangle pair
   candidates, then triangle-triangle contact generation. Verify stable
   stacking — no interpenetration beyond solver tolerance.

3. **mesh-primitives** — A mesh pyramid colliding with sphere, capsule, and
   box side by side. Each pair exercises a different narrow-phase function
   (`mesh_sphere_contact`, `mesh_box_contact`, `mesh_capsule_contact`).
   All three should rest stably.

4. **stress-test** — Headless validation (12+ checks):
   - Mesh-plane: rest height matches face centroid within 1mm
   - Mesh-mesh: penetration < solver tolerance after settling
   - Mesh-sphere: contact normal points away from mesh surface
   - Mesh-capsule: contact point on capsule surface
   - Mesh-box: stable rest on flat face
   - BVH queries return correct triangle candidates
   - Non-convex mesh: concave bowl catches a ball (ball stays in bowl)
   - Contact count ≥ 1 for each resting pair
   - Zero contacts when meshes separated
   - Mesh with single triangle: degeneracy handled
   - Large mesh (100+ triangles): performance reasonable
   - Mesh + friction: object doesn't slide on flat face

**Concepts covered:** `type="mesh"`, vertex/face data, BVH construction,
`mesh_sphere_contact()`, `mesh_box_contact()`, `mesh_capsule_contact()`,
`mesh_mesh_contact()`, convex vs non-convex mesh, triangle-level contact
generation.

##### 15. `heightfield/` — Terrain Collision

The heightfield module (441 LOC) provides terrain collision via grid-based
elevation data. Objects roll and slide over procedural terrain. Used for
wheeled robots, walking robots, and any ground that isn't a flat plane.

**Examples:**

1. **sphere-on-terrain** — A sphere rolling down a sinusoidal terrain
   (heightfield with `sin(x)*cos(z)` elevation). The sphere accelerates
   on downslopes and decelerates on upslopes. Track the sphere's x-position
   over time — verify it follows the terrain contour.

2. **box-on-hills** — A box sliding down a terrain with gentle hills. The
   box contacts the terrain on its bottom face. Different friction values
   control slide speed. The heightfield uses `heightfield_box_contact()`.

3. **terrain-walk** — Three capsules (representing legs) placed on a terrain
   at different elevations. Each rests at the local terrain height. Apply
   downward force — capsules push into terrain and are supported. Demonstrates
   `heightfield_capsule_contact()`.

4. **stress-test** — Headless validation (10+ checks):
   - Sphere rests at correct terrain elevation (within 1mm)
   - Box rests flat on flat terrain region
   - Capsule contacts at both endcap and cylinder
   - Contact normal matches terrain gradient direction
   - Flat terrain behaves identically to plane geom
   - Heightfield boundary: objects at grid edge handled safely
   - Zero-height terrain = flat plane
   - High-frequency terrain: contact generation stable
   - Object rolls downhill under gravity (x-position changes)
   - Multiple objects on same terrain don't interfere

**Concepts covered:** `type="hfield"`, elevation grid data, grid cell lookup,
`heightfield_sphere_contact()`, `heightfield_box_contact()`,
`heightfield_capsule_contact()`, terrain gradient, boundary handling.

##### 16. `hill-muscle/` — Hill-Type Muscle Model

The HillMuscle system (1,894 LOC in fiber.rs) is a CortenForge extension
beyond MuJoCo's quadratic muscle model. It uses physiologically accurate
curves: Gaussian active force-length, hyperbolic force-velocity, and
exponential passive force-length. These match published Hill-type muscle
models (Zajac 1989, Millard 2013).

**Examples:**

1. **force-curves** — Side-by-side comparison of MuJoCo quadratic FL/FV
   curves vs HillMuscle Gaussian/hyperbolic curves. Sweep a joint through
   its range and plot actuator force vs joint position for both models.
   The Hill curves are smoother at the extremes and match experimental
   muscle data more closely.

2. **twitch-response** — A single HillMuscle actuator receives a brief
   control pulse (0.1s). Track the force output over time — it rises with
   activation dynamics, peaks, then decays. Compare the twitch shape against
   the MuJoCo muscle's response. The Hill model's hyperbolic FV curve
   produces a different peak force at high shortening velocities.

3. **eccentric-loading** — A HillMuscle holding a load that is then increased.
   The muscle lengthens under load (eccentric contraction). The hyperbolic FV
   curve predicts higher force during lengthening than shortening — this
   asymmetry is a key feature of real muscle that the Hill model captures.

4. **stress-test** — Headless validation (12+ checks):
   - Gaussian FL: peak force at optimal length (FL = 1.0 at l_opt)
   - Gaussian FL: force → 0 at extremes (l < lmin, l > lmax)
   - Hyperbolic FV: force = 1.0 at zero velocity (isometric)
   - Hyperbolic FV: force > 1.0 during eccentric (lengthening)
   - Hyperbolic FV: force < 1.0 during concentric (shortening)
   - Exponential passive FL: zero below slack, exponential above
   - Total force = activation * active_FL * FV + passive_FL
   - HillMuscle activation dynamics match standard first-order
   - HillMuscle force at optimal length/zero velocity matches gain
   - Eccentric/concentric asymmetry ratio matches published values
   - HillMuscle with muscle actuator transmission works correctly
   - Full pipeline: ctrl → activation → fiber force → joint torque

**Concepts covered:** `GainType::HillMuscle`, `BiasType::HillMuscle`,
`DynType::HillType`, `hill_active_fl()`, `hill_force_velocity()`,
`hill_passive_fl()`, Gaussian vs quadratic FL, hyperbolic vs linear FV,
eccentric/concentric asymmetry, physiological muscle modeling.

##### 17. `adhesion/` — Body Transmission (Adhesive Actuation)

Body transmission (276 LOC) creates adhesive/attractive forces via contact
normals. The actuator iterates all contacts for a body and applies a force
along the average contact normal — pulling the body toward whatever it's
touching. Used for magnetic grippers, suction cups, climbing robots, and
any scenario requiring adhesion.

**Examples:**

1. **magnetic-gripper** — A free-floating body above a ground plane with a
   body-transmission actuator. When `ctrl > 0`, the adhesion force pulls
   the body down into the plane (attraction). When `ctrl = 0`, gravity is
   the only force. Vary ctrl to demonstrate force scaling. The gripper can
   hold objects against gravity.

2. **wall-climb** — A body on a vertical wall with adhesion. Gravity pulls
   it down, adhesion holds it up. Increase adhesion ctrl until the body
   sticks. Decrease until it slides off. Find the critical ctrl value
   where adhesion balances gravity.

3. **stress-test** — Headless validation (8+ checks):
   - Adhesion force = 0 when ctrl = 0
   - Adhesion force scales linearly with ctrl
   - Adhesion force direction matches average contact normal
   - No adhesion when no contacts (body in free flight)
   - Adhesion can hold body against gravity (equilibrium)
   - Multiple contacts: force averages normals correctly
   - Adhesion + friction: body sticks on tilted surface
   - Body transmission gear ratio affects force magnitude

**Concepts covered:** `MjTrnType::Body`, `mj_transmission_body()`, contact
normal averaging, adhesive force generation, `data.actuator_force` for
body transmission, grip strength vs gravity.

##### 18. `flex-bodies/` — Deformable Objects (Beta)

The flex system (2,797 LOC across dynamics and collision) implements
deformable bodies — cloth, cables, and soft objects. Bodies are connected
by edge constraints with stiffness and damping. Two bending models are
available: Cotangent (Wardetzky/Garg, MuJoCo-conformant) and Bridson
(dihedral angle springs). Self-collision uses specialized algorithms
(brute-force, BVH, or sweep-and-prune).

**Note:** Flex bodies are beta — core algorithms work but edge cases may
need refinement. These examples validate the current state and will catch
regressions as the implementation matures.

**Examples:**

1. **hanging-cloth** — A rectangular flex body (grid of vertices, quad
   elements) fixed along the top edge, hanging under gravity. The cloth
   sags into a U-shape. Bending stiffness controls how stiff the drape is.
   Vary Young's modulus: high = stiff sheet, low = droopy fabric.

2. **cable-drop** — A 1D flex body (chain of vertices, edge elements)
   dropped onto a plane. The cable coils and settles. Edge stiffness
   prevents excessive stretching. Demonstrates flex-rigid contact via
   `flex_collide.rs`.

3. **bending-modes** — Same cloth, two bending models side by side:
   Cotangent (curvature-based, smoother) vs Bridson (dihedral angle,
   stiffer at creases). Apply the same perturbation — the deformation
   patterns differ. Demonstrates `FlexBendingType::Cotangent` vs
   `FlexBendingType::Bridson`.

4. **self-collision** — A cloth dropped onto itself (folded). Self-collision
   prevents interpenetration. Three variants: `FlexSelfCollide::None`
   (passes through itself), `FlexSelfCollide::Narrow` (brute-force
   detection), `FlexSelfCollide::BVH` (accelerated). Only Narrow and BVH
   prevent self-interpenetration.

5. **stress-test** — Headless validation (12+ checks):
   - Edge lengths stay within tolerance of rest length
   - Hanging cloth: bottom edge below top edge (gravity works)
   - Young's modulus scaling: higher = less deformation
   - Bending stiffness: higher = larger bending radius
   - Flex-rigid contact: flex vertices don't penetrate plane
   - Self-collision: vertex-vertex distance ≥ minimum gap
   - Cotangent bending energy = ∫κ² dA (curvature integral)
   - Bridson bending energy = Σ k*(θ - θ_rest)² (dihedral angles)
   - Damping: flex oscillations decay over time
   - Thickness parameter affects contact margin
   - Friction: flex slides on tilted plane at correct angle
   - Poisson ratio: lateral contraction under longitudinal stretch

**Concepts covered:** `<flex>`, vertices, edges, elements, `Young`,
`Poisson`, `damping`, `thickness`, `FlexBendingType` (Cotangent, Bridson),
`FlexSelfCollide` (None, Narrow, BVH, SAP), flex-rigid contact,
deformable dynamics, edge constraint assembly.

##### 19. `collision-pairs/` — All Primitive Pair Functions

The collision module has specialized narrow-phase functions for every
geometry pair type (~2,600 LOC across pair_convex.rs, pair_cylinder.rs,
plane.rs). Each pair function handles the geometry-specific math — SAT for
boxes, analytic for spheres, clipping for cylinders. These are exercised
indirectly by other examples but never isolated.

**Examples:**

1. **sphere-pairs** — Sphere-sphere, sphere-capsule, sphere-box, sphere-
   cylinder, sphere-plane. Drop each pair, verify contact normal points
   from shape A to shape B, contact depth matches analytical overlap.

2. **box-pairs** — Box-box (SAT + polygon clipping), box-capsule (polyhedral
   clipping), box-plane (4-corner contacts), box-cylinder. The complex
   cases: box-box at a tilted angle generates edge-edge contacts. Verify
   stable stacking for each.

3. **capsule-cylinder** — Capsule-capsule (segment-segment, up to 2
   contacts, near-parallel special case), capsule-cylinder, cylinder-
   cylinder. These are the most geometrically complex pair functions.
   Verify contact count and normals.

4. **ellipsoid** — Ellipsoid-plane, ellipsoid-sphere, ellipsoid-ellipsoid.
   Ellipsoids use the general GJK/EPA fallback for most pairs. Verify
   contacts match (EPA supports these natively).

5. **stress-test** — Headless validation (16+ checks):
   - Sphere-sphere: contact depth = r1 + r2 - dist(c1, c2)
   - Sphere-plane: contact depth = radius - height
   - Box-plane: 1-4 contacts depending on orientation
   - Box-box: SAT separating axis correct
   - Capsule-capsule: up to 2 contacts (end-end, end-side, side-side)
   - Capsule-capsule near-parallel: correct fallback
   - Contact normal is unit length (|n| = 1.0 ± 1e-10)
   - Contact normal points from geom1 to geom2
   - Separated pair: zero contacts
   - Touching pair: contact depth ≈ 0
   - Deeply overlapping: contact depth > 0
   - All primitive-plane pairs: normal = plane normal
   - Cylinder-sphere: contact on cylinder surface
   - Box-capsule: clipping produces correct contact patch
   - GJK/EPA fallback agrees with analytical for sphere-sphere
   - Each pair function exercised at least once

**Concepts covered:** `collide_sphere_sphere()`, `collide_capsule_capsule()`,
`collide_sphere_capsule()`, `collide_sphere_box()`, `collide_cylinder_sphere()`,
`collide_cylinder_capsule()`, `collide_capsule_box()`, `collide_box_box()`,
`collide_with_plane()`, GJK/EPA fallback, SAT (separating axis theorem),
polygon clipping, contact normal convention, contact depth.

##### 20. `plugins/` — Custom Sensors and Forces

The plugin system (24,398 LOC) provides a trait-based framework for
extending the engine with custom computation. Plugins can define new sensor
types and custom force generators that integrate into the standard forward
pipeline.

**Examples:**

1. **custom-sensor** — Register a plugin that computes a custom sensor:
   the distance between two sites raised to a power (nonlinear distance).
   Wire it into the sensor pipeline via `SensorType::Plugin`. Read the
   sensor value and verify it matches manual computation. Demonstrates
   the sensor plugin callback interface.

2. **custom-force** — Register a plugin that applies a custom force: a
   velocity-dependent drag force (F = -c*v²*v̂) that isn't covered by
   the built-in fluid model. Wire it into the actuation pipeline. Drop
   a ball — it reaches terminal velocity. Verify terminal velocity
   matches analytical sqrt(mg/(c)).

3. **stress-test** — Headless validation (8+ checks):
   - Custom sensor returns correct value
   - Custom sensor dimension matches declared ndata
   - Custom force modifies qfrc_applied correctly
   - Plugin state persists across timesteps
   - Multiple plugins coexist without interference
   - Plugin callback called once per step
   - Plugin with zero state: works without allocation
   - Unregistered plugin type: graceful error

**Concepts covered:** Plugin trait, sensor plugin callbacks, force plugin
callbacks, `SensorType::Plugin`, plugin state, plugin registration,
custom computation in the forward pipeline.

---

### Track 2: SDF-CPU ladder (complete the 7 stubs)

Steps 04, 07–11 now working. Remaining stubs:

```
sdf-physics/
  cpu/
    05-drop/       # implement: restitution + bounce
    06-slide/      # implement: friction on ground
    12-hinge-wall/ # implement: articulated external contact
    13-hinge-stop/ # implement: parent-child SDF contact
    14-damped-hinge/ # implement: damping parameter tuning
    15-concave-stop/ # implement: concave parent geometry
    16-socket/     # blocked: full socket/condyle
```

### Track 3: GPU baby-step ladder

```
sdf-physics/
  gpu/
    00-freefall/   # GPU init, step(), readback, Bevy wiring
    01-rest/       # SDF-plane contact on GPU
    02-drop/       # dynamic contact + restitution
    03-slide/      # friction
    04-pair/       # SDF-SDF contact
    05-stack/      # multi-body constraint graph
    06-strike/     # momentum transfer (free body → free body)
    07-hockey/     # mocap stick + puck + goal (capstone)
```

### Track 4: design + mesh (already proportional, minor additions)

```
fundamentals/
  design/
    (existing 3 are good — maybe add optimization demo later)
  mesh/
    (existing 1 is good — maybe add mesh-collision demo)
```

## Priority

Build from the ground up — the foundation must be bulletproof before moving
to GPU or advanced features:

1. **Track 1A** — COMPLETE (82 examples). Basic sim-core fundamentals.
2. **Track 1B** IN PROGRESS — sim-core foundation layer 2 (~70 examples).
   No stone unturned — every implemented subsystem gets dedicated coverage.
   **3/20 subdirectories done** (free-joint, keyframes, mocap-bodies).
   Next: contact filtering.
3. **Track 3** after Track 1B proves the engine — GPU ladder
4. **Track 2** as needed — SDF-CPU stubs (most SDF work targets GPU)
5. **Track 4** as needed — design + mesh

## Notes

- Each example should be ~200–400 LOC, standalone, with pass/fail checks
- Examples double as integration tests — they find bugs
- Baby-step philosophy: one new concept per example
- MuJoCo as reference: verify behavior matches
