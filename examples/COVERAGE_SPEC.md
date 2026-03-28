# Examples Coverage Spec

**Status:** Track 1 fleshed out — ready for review
**Date:** 2026-03-25
**Goal:** 100% coverage of codebase capabilities

## Principle

Examples should mirror the distribution of code in the codebase. Every major
feature should have at least one dedicated example. Examples also serve as
integration tests — they find bugs that unit tests miss.

## Current State (2026-03-27)

- 232K LOC codebase
- sim-core + sim-mjcf → 42 examples (8 joint types, 9 sensors, 10 actuators, 8 integrators, 5 solvers, 2 legacy)
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

### Actuator System (10 examples — standard actuators covered, muscles remaining)
- Transmission: Joint ✓, Tendon ✓, Site ✓, SliderCrank ✓, **Body ✗, JointInParent ✗**
- Dynamics: None ✓, Filter ✓, Integrator ✓, **Muscle ✗, HillMuscle ✗, User ✗**
- Gain: Fixed ✓, Affine ✓, **Muscle ✗, HillMuscle ✗, User ✗**
- Bias: None ✓, Affine ✓, **Muscle ✗, HillMuscle ✗, User ✗**

### Sensors (40+ types, 15 covered in 9 examples)
- Position: JointPos ✓, FramePos ✓, FrameQuat ✓, SubtreeCom ✓, Clock ✓
- Velocity: JointVel ✓, Gyro ✓, Velocimeter ✓
- Force: Touch ✓, ActuatorFrc ✓, JointActuatorFrc ✓
- Spatial: Accelerometer ✓, GeomDist ✓, GeomNormal ✓, GeomFromTo ✓
- Ball: BallQuat ✓, BallAngVel ✓ (in ball-joint/spherical-pendulum)
- **Uncovered:** TendonPos, TendonVel, ActuatorPos, FrameLinVel, FrameAngVel,
  Force, Torque, JointLimitFrc, RangeFinder, SubtreeAngMom, User, Plugin

### Collision (partially covered)
- SDF-plane ✓, SDF-SDF ✓, analytical convex ✓
- Convex-convex (GJK/EPA) ✓ (geom-distance sensor example — also fixed a GJK bug)
- **Mesh-mesh ✗, mesh-plane ✗**
- **Height field ✗**
- **Friction tuning ✗** (06-slide is stub)
- **Restitution ✗** (05-drop is stub)
- **Contact parameter override ✗**

### Solvers & Integration
- Newton ✓, PGS ✓, CG ✓ (comparison + comparison-visual + 3 standalones)
- Euler ✓, RK4 ✓, Implicit ✓, ImplicitFast ✓, ImplicitSpringDamper ✓ (comparison + comparison-visual + 5 standalones)

### Constraint System
- Contact ✓
- **Equality (weld, connect, distance) ✗**
- **Joint limits (dedicated demo) ✗**
- **Friction loss ✗**
- **Tendon limits ✗**

### Model Loading
- MJCF (inline string) ✓
- **MJCF (file with <include>) ✗**
- **URDF ✗** (entire sim-urdf crate, zero examples)

### Advanced Features (all uncovered)
- Inverse dynamics
- Derivatives / Jacobians
- Sleep / wake / islands
- Keyframes
- Flex bodies
- Plugin system / callbacks
- Energy conservation tracking
- Domain randomization

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
    muscles/                # TODO — Hill muscle model, activation dynamics
    solvers/                # DONE — comparison + comparison-visual + 3 standalones
    integrators/            # DONE — comparison + comparison-visual + 5 standalones
    equality-constraints/   # TODO — weld, connect, distance
    contact-tuning/         # TODO — friction, restitution, solref/solimp
    inverse-dynamics/       # TODO — compute required torques
    energy-momentum/        # TODO — conservation tracking
    urdf-loading/           # TODO — load URDF, compare with MJCF
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

##### 5. `muscles/` — Hill Muscle Model and Activation Dynamics

A forearm model: upper arm fixed to world, elbow hinge joint, forearm body with
a Hill muscle actuator spanning the joint. The muscle has realistic parameters:
optimal fiber length, tendon slack length, pennation angle, peak isometric force.
The control signal ramps activation from 0 → 1 over 0.5s, then holds. The
example prints muscle activation, normalized fiber length, force-length curve
value, force-velocity curve value, and total muscle force.

**Concepts covered:** `HillMuscle` dynamics (asymmetric activation rise/fall),
`HillMuscle` gain (FL × FV × cos(α) active force), `HillMuscle` bias (passive
force-length), `gainprm` layout (F0, optimal fiber length, tendon slack,
pennation angle, max contraction velocity). Demonstrates the CortenForge
extension beyond standard MuJoCo muscle model.

**MJCF sketch:** Two-body forearm. `<general joint="elbow" dyntype="hillmuscle"
gaintype="hillmuscle" biastype="hillmuscle" .../>` with explicit `gainprm` and
`dynprm`. A second standard `<muscle>` actuator on a parallel arm for comparison.

**Pass/fail:**
- Activation rises with time constant τ_act ≈ 10ms and falls with τ_deact ≈ 40ms
  (verify within 10% of expected exponential).
- At optimal fiber length (L_norm=1.0), FL curve ≈ 1.0.
- Passive force is zero for L_norm ≤ 1.0, positive and increasing for L_norm > 1.0.
- Peak isometric force matches `gainprm[2]` (F0) within 5% at optimal length,
  zero velocity, full activation.
- Pennation angle reduces force by cos(α) — verify against zero-pennation case.

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

##### 9. `contact-tuning/` — Friction, Restitution, and Solver Parameters

A series of balls dropped onto a tilted plane to demonstrate contact parameter
tuning. Row 1: three balls with different friction coefficients (μ=0.1, 0.5, 1.0)
— low friction slides, high friction sticks. Row 2: three balls with different
`solref` (stiff vs soft contact) — stiff bounces sharply, soft sinks slightly.
Row 3: three balls with different `condim` (1=frictionless, 3=2D friction,
6=full friction+rolling) — frictionless slides off, rolling friction stops
quickly.

**Concepts covered:** `<pair>` contact overrides, five-element friction
(`friction="sliding torsional rolling"`), `condim` (1/3/4/6), `solref`
(timeconst + dampratio → contact stiffness/damping), `solimp` (impedance curve),
`margin` and `gap`. Shows the three-level parameter hierarchy: explicit pair →
geom combination → geom defaults.

**MJCF sketch:** Tilted ground plane (15°). 9 spheres in a 3×3 grid.
`<contact>` section with explicit `<pair>` overrides for each sphere-plane
interaction. Different friction, solref, condim per row.

**Pass/fail:**
- Low friction (μ=0.1): ball slides down plane, velocity increasing.
- High friction (μ=1.0): ball remains stationary on 15° slope (friction force
  > gravity component: μ > tan(15°) ≈ 0.27).
- Frictionless (condim=1): ball accelerates down slope at `g·sin(θ)` ± 5%.
- Stiff solref: bounce height > 50% of drop height.
- Soft solref: bounce height < 10% of drop height (energy absorbed).
- Rolling friction: ball stops rotating faster than sliding-only friction.

##### 10. `inverse-dynamics/` — Compute Required Torques

A two-link robot arm follows a prescribed trajectory (sinusoidal joint angles).
At each timestep, the example sets `qpos` and `qacc` to the desired trajectory,
runs `inverse()`, and reads `qfrc_inverse` — the torques required to produce
that motion. It then runs a second simulation with a motor actuator applying
those computed torques and verifies the arm follows the same trajectory. Prints
the torque profile and tracking error.

**Concepts covered:** `data.inverse(model)`, `qfrc_inverse` output, the
relationship `qfrc_inverse = M·qacc + bias - passive - constraint`. Jacobian
computation (`mj_jac`). Round-trip verification: inverse → forward reproduces
the trajectory.

**MJCF sketch:** Two-link planar arm (two hinge joints in the Y-Z plane). Motor
actuators on both joints. Gravity enabled.

**Pass/fail:**
- Inverse dynamics torques are smooth and physically reasonable (no spikes).
- Forward simulation driven by inverse-computed torques tracks the desired
  trajectory with position error < 0.1° per joint over 5 seconds.
- `qfrc_inverse` satisfies the identity: `qfrc_inverse ≈ qfrc_applied +
  qfrc_actuator` when those are the only external forces.
- Jacobian (`mj_jac`) at end-effector site has correct dimensions (3×nv) and
  maps joint velocities to Cartesian velocity within 1%.

##### 11. `energy-momentum/` — Conservation Tracking

An undamped, frictionless system: a free-floating rigid body (no joints, no
contacts, no gravity) with initial linear and angular velocity. With no external
forces, linear momentum, angular momentum, and kinetic energy must all be
conserved exactly. The example tracks these quantities every timestep and prints
max deviation. A second scene adds gravity + a perfectly elastic bounce
(solref tuned for restitution ≈ 1) to show total energy conservation through
a collision.

**Concepts covered:** `ENABLE_ENERGY` flag, `data.energy_kinetic`,
`data.energy_potential`, `data.total_energy()`, momentum computation from
`data.qvel` and mass matrix, `DISABLE_GRAVITY` flag, elastic collision
via `solref` tuning.

**MJCF sketch:** Scene A: `<option gravity="0 0 0"/>`, single free body with
`<key qvel="0.5 0 0 0 0.3 0.2"/>`. Scene B: gravity enabled, ball dropped onto
plane with tuned solref for elastic bounce.

**Pass/fail:**
- Scene A: kinetic energy drift < 1e-10 over 10 seconds (machine precision).
- Scene A: linear velocity constant to machine precision (no forces).
- Scene A: angular velocity constant (symmetric inertia) or angular momentum
  conserved (asymmetric inertia) to < 1e-8.
- Scene B: total energy (KE + PE_gravity) conserved to < 1% through bounce
  (with elastic solref).
- Scene B: bounce height within 5% of drop height (near-perfect restitution).

##### 12. `urdf-loading/` — Load URDF, Compare with MJCF

Load a simple robot arm defined in both URDF and MJCF format, run identical
simulations, and verify the dynamics match. The URDF uses standard elements:
revolute joints, prismatic joint, fixed joint, box/sphere/cylinder primitives,
inertia tensors. Prints body count, joint count, DOFs, and compares qpos after
1 second of freefall.

**Concepts covered:** `sim_urdf::load_urdf_model()`, URDF→MJCF conversion
pipeline, supported joint types (revolute, continuous, prismatic, fixed,
floating), supported geometries (box, sphere, cylinder), inertia specification,
limitations (no mesh loading, no mimic joints, tree structures only).

**MJCF sketch:** Equivalent MJCF and URDF for a 3-DOF arm: revolute shoulder,
revolute elbow, prismatic gripper. Fixed base link. Box and cylinder geoms.

**Pass/fail:**
- URDF and MJCF produce identical `model.nbody`, `model.njnt`, `model.nv`.
- After 1 second of identical simulation: `qpos` matches within 1e-6.
- `qvel` matches within 1e-6.
- Joint axes and limits match between URDF and MJCF models.
- URDF with unsupported features (mesh geometry) produces a clear error message,
  not a crash.

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
