# Examples Coverage Spec

**Status:** Track 1 fleshed out — ready for review
**Date:** 2026-03-25
**Goal:** 100% coverage of codebase capabilities

## Principle

Examples should mirror the distribution of code in the codebase. Every major
feature should have at least one dedicated example. Examples also serve as
integration tests — they find bugs that unit tests miss.

## Current State (2026-03-30)

- 232K LOC codebase
- sim-core + sim-mjcf + sim-urdf → 82 examples (8 joint types, 9 sensors, 10 actuators, 5 muscles, 8 integrators, 5 solvers, 8 equality constraints, 7 contact tuning, 5 inverse dynamics, 5 energy-momentum, 10 urdf-loading, 2 legacy)
- cf-design → 3 examples
- mesh-* → 1 example
- sim-gpu → 0 working examples
- SDF-CPU ladder: 8 working, 7 stubs, 1 blocked

## Gap Analysis

### Joint Types (4 types, 4 covered)
- Hinge → covered (simple-pendulum, double-pendulum, actuator-force, sensor examples)
- Free → covered (accelerometer, touch, all SDF physics)
- Slide → covered (horizontal, vertical, geom-distance)
- Ball → covered (spherical-pendulum, conical-pendulum, cone-limit, frame-pos-quat, gyro-velocimeter)

### Geometry Types (8 types, ~6 covered)
- Plane, Sphere, Capsule, Cylinder, Box, Ellipsoid → covered
- **Mesh (explicit convex) → ZERO dedicated examples**
- **Hfield (height field terrain) → ZERO examples**
- Sdf → extensively covered

### Actuator System (10 standard + 5 muscle examples)
- Transmission: Joint ✓, Tendon ✓, Site ✓, SliderCrank ✓, **Body ✗, JointInParent ✗**
- Dynamics: None ✓, Filter ✓, Integrator ✓, Muscle ✓, **HillMuscle (stress-test only), User ✗**
- Gain: Fixed ✓, Affine ✓, Muscle ✓, **HillMuscle (stress-test only), User ✗**
- Bias: None ✓, Affine ✓, Muscle ✓, **HillMuscle (stress-test only), User ✗**
- Muscle-specific: Activation dynamics ✓, Force-length ✓, Cocontraction ✓, Forearm flexion ✓

### Sensors (40+ types, 16 covered in 10 examples)
- Position: JointPos ✓, FramePos ✓, FrameQuat ✓, SubtreeCom ✓, Clock ✓
- Velocity: JointVel ✓, Gyro ✓, Velocimeter ✓
- Force: Touch ✓, ActuatorFrc ✓, JointActuatorFrc ✓
- Spatial: Accelerometer ✓, GeomDist ✓, GeomNormal ✓, GeomFromTo ✓
- Ball: BallQuat ✓, BallAngVel ✓ (in ball-joint/spherical-pendulum)
- Momentum: SubtreeAngMom ✓ (in energy-momentum/free-flight)
- **Uncovered:** TendonPos, TendonVel, ActuatorPos, FrameLinVel, FrameAngVel,
  Force, Torque, JointLimitFrc, RangeFinder, User, Plugin

### Collision (partially covered)
- SDF-plane ✓, SDF-SDF ✓, analytical convex ✓
- Convex-convex (GJK/EPA) ✓ (geom-distance sensor example — also fixed a GJK bug)
- Friction tuning ✓ (friction-slide, condim-compare — 2 examples)
- Solref/solimp (bounce) ✓ (solref-bounce — 1 example)
- Contact parameter override ✓ (pair-override — 1 example)
- Solimp impedance curve ✓ (solimp-depth — 1 example)
- Margin/gap activation ✓ (margin-gap — 1 example)
- **Mesh-mesh ✗, mesh-plane ✗**
- **Height field ✗**

### Solvers & Integration
- Newton ✓, PGS ✓, CG ✓ (comparison + comparison-visual + 3 per-solver)
- Euler ✓, RK4 ✓, Implicit ✓, ImplicitFast ✓, ImplicitSpringDamper ✓ (comparison + comparison-visual + 5 per-integrator)

### Constraint System
- Contact ✓
- Equality (weld, connect, distance, joint coupling) ✓ (8 examples)
- **Joint limits (dedicated demo) ✗**
- Friction loss ✓ (urdf-loading/damping-friction)
- **Tendon limits ✗**

### Model Loading
- MJCF (inline string) ✓
- **MJCF (file with <include>) ✗**
- URDF ✓ (10 examples: revolute, prismatic, continuous, fixed, mimic, geometry, inertia, damping-friction, error-handling, stress-test)

### Inverse Dynamics & Jacobians
- Inverse dynamics ✓ (gravity-comp, torque-profile, forward-replay, stress-test — 4 examples)
- Jacobians (mj_jac_site, velocity mapping) ✓ (jacobian example — stale vs correct comparison)

### Advanced Features (mostly uncovered)
- Energy conservation tracking ✓ (4 visual + 12-check stress-test)
- **Sleep / wake / islands ✗**
- **Keyframes ✗**
- **Flex bodies ✗**
- **Plugin system / callbacks ✗**
- **Domain randomization ✗**

### GPU Pipeline
- Full pipeline (GpuPhysicsPipeline::step()) → ZERO examples
- Half-GPU (enable_gpu_collision) → was in old hockey, now cleaned up
- Mocap bodies on GPU → not yet implemented in orchestrator
- Batch/multi-env → architecture ready, not exposed

## Proposed Example Structure

### Track 1: sim-core fundamentals (fill the 90K LOC gap)

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

#### Track 1 Example Descriptions

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

Build from the ground up:
1. **Track 1** first — sim-core fundamentals (the foundation)
2. **Track 2** in parallel — finish the CPU stubs
3. **Track 3** after Track 1 proves the engine — GPU ladder
4. **Track 4** as needed

## Notes

- Each example should be ~200–400 LOC, standalone, with pass/fail checks
- Examples double as integration tests — they find bugs
- Baby-step philosophy: one new concept per example
- MuJoCo as reference: verify behavior matches
