# CortenForge Examples

## Directory Structure

```
examples/
  fundamentals/
    design/           Pure cf-design (implicit surfaces, bio-inspired geometry)
    mesh/             Pure mesh-* (repair, lattice, shell, printability)
    sim-cpu/          CPU physics fundamentals (MJCF, joints, sensors)
    sim-gpu/          GPU physics fundamentals (future)
  integration/        Cross-domain pipelines (design → sim → mesh → print)
  sdf-physics/
    cpu/              SDF collision proof ladder (CPU pipeline, 16 baby steps)
    gpu/              GPU physics demos (hockey, future: batch-rl, vr)
```

## fundamentals/design/

Pure `cf-design` examples — implicit surfaces, no physics backend.

| Example | Package | Status | Notes |
|---------|---------|--------|-------|
| `hello-solid` | `example-hello-solid` | Working | Implicit surface basics |
| `bio-shapes` | `example-bio-shapes` | Working | Bio-inspired geometry library |
| `finger-design` | `example-finger-design` | Needs work | cf-design + sim visualization (dynamics not tuned) |

## fundamentals/mesh/

Pure `mesh-*` examples — mesh processing, no physics.

| Example | Package | Status | Notes |
|---------|---------|--------|-------|
| `mesh-pipeline` | `example-mesh-pipeline` | Working | Mesh repair, lattice, shell, printability |

## fundamentals/sim-cpu/

CPU physics fundamentals — joints, sensors, MJCF, sim-core.

### Joint types

| Example | Package | Status | Notes |
|---------|---------|--------|-------|
| `hinge-joint/simple-pendulum` | `example-hinge-joint-pendulum` | Working | Single hinge, energy conservation |
| `hinge-joint/double-pendulum` | `example-hinge-joint-double-pendulum` | Working | Chaotic two-link chain |
| `slide-joint/horizontal` | `example-slide-joint-horizontal` | Working | Spring-loaded slider, oscillation |
| `slide-joint/vertical` | `example-slide-joint-vertical` | Working | Gravity-loaded spring, equilibrium |
| `ball-joint/spherical-pendulum` | `example-ball-joint-pendulum` | Working | 3-DOF spherical pendulum |
| `ball-joint/conical-pendulum` | `example-ball-joint-conical` | Working | Steady conical orbit, precession |
| `ball-joint/cone-limit` | `example-ball-joint-cone-limit` | Working | Ball joint with cone constraint |
| `ball-joint/cone-limit-orbit` | `example-ball-joint-cone-orbit` | Working | Cone limit with orbital motion |
| `free-joint/tumble` | `example-free-joint-tumble` | Working | Torque-free precession, asymmetric inertia |
| `free-joint/projectile` | `example-free-joint-projectile` | Working | 45-degree launch, parabolic trajectory |
| `free-joint/spinning-toss` | `example-free-joint-spinning-toss` | Working | Full 6-DOF: translation + rotation decoupled |
| `free-joint/stress-test` | `example-free-joint-stress-test` | Working | Headless validation (12 checks) |

### Keyframes

| Example | Package | Status | Notes |
|---------|---------|--------|-------|
| `keyframes/save-restore` | `example-keyframes-save-restore` | Working | Pendulum cycles 3 named keyframes (rest, horizontal, inverted) |
| `keyframes/multi-body` | `example-keyframes-multi-body` | Working | Two-link arm with ctrl state, 3 keyframes |
| `keyframes/stress-test` | `example-keyframes-stress-test` | Working | Headless validation (12 checks) |

### Mocap Bodies

| Example | Package | Status | Notes |
|---------|---------|--------|-------|
| `mocap-bodies/stress-test` | `example-mocap-bodies-stress-test` | Working | Headless validation (12 checks) |
| `mocap-bodies/drag-target` | `example-mocap-bodies-drag-target` | Working | Soft weld tracking — box follows mocap sphere |
| `mocap-bodies/push-object` | `example-mocap-bodies-push-object` | Working | One-way contact — paddle pushes ball |
| `mocap-bodies/tilt-drop` | `example-mocap-bodies-tilt-drop` | Working | Orientation-driven — tilting platform slides ball off |

### Contact Filtering

| Example | Package | Status | Notes |
|---------|---------|--------|-------|
| `contact-filtering/stress-test` | `example-contact-filtering-stress-test` | Working | Headless validation (12 checks: bitmask AND/OR, parent-child, exclude, pair bypass) |
| `contact-filtering/bitmask` | `example-contact-filtering-bitmask` | Working | 4 spheres with different contype/conaffinity — two rest, two fall through |
| `contact-filtering/exclude-pairs` | `example-contact-filtering-exclude-pairs` | Working | Mocap platform side-by-side — excluded sphere passes through, normal lands |
| `contact-filtering/ghost-layers` | `example-contact-filtering-ghost-layers` | Working | Solid/ghost collision layers — ghosts rest on solids, pass through each other |

### Joint Limits

| Example | Package | Status | Notes |
|---------|---------|--------|-------|
| `joint-limits/stress-test` | `example-joint-limits-stress-test` | Working | Headless validation (12 checks: hinge/slide/ball activation, sensor, solref, solimp, motor, locked, penetration, symmetry) |
| `joint-limits/hinge-limits` | `example-joint-limits-hinge-limits` | Working | 3 pendulums comparing stiff/default/soft solref — different bounce and penetration |
| `joint-limits/slide-limits` | `example-joint-limits-slide-limits` | Working | Box on rail pushed by motor into limit — limit force balances motor force |
| `joint-limits/ball-cone` | `example-joint-limits-ball-cone` | Working | Ball joint 30° cone limit — tip orbits boundary then spirals to rest (coin-funnel) |

### Sensors

| Example | Package | Sensors | Status |
|---------|---------|---------|--------|
| `sensors/clock` | `example-sensor-clock` | Clock | Working |
| `sensors/joint-pos-vel` | `example-sensor-joint-pos-vel` | JointPos, JointVel | Working |
| `sensors/frame-pos-quat` | `example-sensor-frame-pos-quat` | FramePos, FrameQuat | Working |
| `sensors/subtree-com` | `example-sensor-subtree-com` | SubtreeCom | Working |
| `sensors/gyro-velocimeter` | `example-sensor-gyro-velocimeter` | Gyro, Velocimeter | Working |
| `sensors/accelerometer` | `example-sensor-accelerometer` | Accelerometer | Working |
| `sensors/touch` | `example-sensor-touch` | Touch | Working |
| `sensors/actuator-force` | `example-sensor-actuator-force` | ActuatorFrc, JointActuatorFrc | Working |
| `sensors/geom-distance` | `example-sensor-geom-distance` | GeomDist, GeomNormal, GeomFromTo | Working |

### Sensors Advanced

| Example | Package | Sensors | Status |
|---------|---------|---------|--------|
| `sensors-advanced/stress-test` | `example-sensor-adv-stress-test` | All 15 types (25 checks) | Working |
| `sensors-advanced/frame-velocity` | `example-sensor-adv-frame-velocity` | FrameLinVel, FrameAngVel | Working |
| `sensors-advanced/frame-acceleration` | `example-sensor-adv-frame-acceleration` | FrameLinAcc, FrameAngAcc | Working |
| `sensors-advanced/force-torque` | `example-sensor-adv-force-torque` | Force, Torque | Working |
| `sensors-advanced/rangefinder` | `example-sensor-adv-rangefinder` | Rangefinder | Working |
| `sensors-advanced/subtree-velocity` | `example-sensor-adv-subtree-velocity` | SubtreeLinVel | Working |
| `sensors-advanced/subtree-angmom` | `example-sensor-adv-subtree-angmom` | SubtreeAngMom | Working |
| `sensors-advanced/actuator-pos-vel` | `example-sensor-adv-actuator-pos-vel` | ActuatorPos, ActuatorVel | Working |

### Integrators

| Example | Package | Status | Notes |
|---------|---------|--------|-------|
| `integrators/comparison` | `example-integrator-comparison` | Working | Headless energy drift table, 7 checks |
| `integrators/comparison-visual` | `example-integrator-comparison-visual` | Working | 3 double pendulums side by side |
| `integrators/euler` | `example-integrator-euler` | Working | Semi-implicit Euler, visible drift |
| `integrators/rk4` | `example-integrator-rk4` | Working | 4th-order Runge-Kutta, near-perfect |
| `integrators/implicit` | `example-integrator-implicit` | Working | Full implicit, L-stable |
| `integrators/implicit-fast` | `example-integrator-implicit-fast` | Working | Symmetric D, no Coriolis |
| `integrators/implicit-spring-damper` | `example-integrator-implicit-spring-damper` | Working | Diagonal spring/damper path |

### Solvers

| Example | Package | Status | Notes |
|---------|---------|--------|-------|
| `solvers/comparison` | `example-solver-comparison` | Working | Headless convergence table, 7 checks |
| `solvers/comparison-visual` | `example-solver-comparison-visual` | Working | 3 stacks side by side |
| `solvers/pgs` | `example-solver-pgs` | Working | Projected Gauss-Seidel, ~47 iter |
| `solvers/cg` | `example-solver-cg` | Working | Conjugate Gradient, ~31 iter |
| `solvers/newton` | `example-solver-newton` | Working | Full Newton, ~0 iter |

### Equality Constraints

| Example | Package | Status | Notes |
|---------|---------|--------|-------|
| `equality-constraints/connect-to-world` | `example-equality-connect-to-world` | Working | Ball-and-socket pendulum |
| `equality-constraints/connect-body-to-body` | `example-equality-connect-body-to-body` | Working | Double pendulum via constraints |
| `equality-constraints/weld-to-world` | `example-equality-weld-to-world` | Working | Body frozen in space |
| `equality-constraints/weld-body-to-body` | `example-equality-weld-body-to-body` | Working | Two bodies rigidly glued |
| `equality-constraints/distance` | `example-equality-distance` | Working | Rigid rod between spheres |
| `equality-constraints/joint-mimic` | `example-equality-joint-mimic` | Working | 1:1 joint coupling |
| `equality-constraints/joint-gear` | `example-equality-joint-gear` | Working | 2:1 gear ratio |
| `equality-constraints/stress-test` | `example-equality-stress-test` | Working | Headless validation |

### Contact Tuning

| Example | Package | Status | Notes |
|---------|---------|--------|-------|
| `contact-tuning/friction-slide` | `example-contact-friction-slide` | Working | Friction coefficient on tilted plane |
| `contact-tuning/condim-compare` | `example-contact-condim-compare` | Working | condim=1 vs 3 vs 6 comparison |
| `contact-tuning/solref-bounce` | `example-contact-solref-bounce` | Working | Contact stiffness and bounce |
| `contact-tuning/pair-override` | `example-contact-pair-override` | Working | Explicit `<pair>` override demo |
| `contact-tuning/solimp-depth` | `example-contact-solimp-depth` | Working | Impedance curve and penetration depth |
| `contact-tuning/margin-gap` | `example-contact-margin-gap` | Working | Contact activation distance |
| `contact-tuning/stress-test` | `example-contact-stress-test` | Working | Headless parameter sweep (26 checks) |

### Inverse Dynamics

| Example | Package | Status | Notes |
|---------|---------|--------|-------|
| `inverse-dynamics/gravity-compensation` | `example-inverse-gravity-compensation` | Working | Static holding torques via inverse() |
| `inverse-dynamics/torque-profile` | `example-inverse-torque-profile` | Working | Dynamic torques for sinusoidal trajectory |
| `inverse-dynamics/forward-replay` | `example-inverse-forward-replay` | Working | Round-trip: replay inverse torques in forward sim |
| `inverse-dynamics/jacobian` | `example-inverse-jacobian` | Working | Velocity mapping: stale vs correct Jacobian |
| `inverse-dynamics/stress-test` | `example-inverse-stress-test` | Working | Headless validation (14 checks) |

### Energy-Momentum

| Example | Package | Status | Notes |
|---------|---------|--------|-------|
| `energy-momentum/free-flight` | `example-energy-free-flight` | Working | Zero-gravity conservation (KE, momentum, angular momentum) |
| `energy-momentum/pendulum-energy` | `example-energy-pendulum-energy` | Working | PE/KE exchange, total energy constant |
| `energy-momentum/elastic-bounce` | `example-energy-elastic-bounce` | Working | Energy conservation through contact, restitution 0.97 |
| `energy-momentum/damped-decay` | `example-energy-damped-decay` | Working | Monotonic energy dissipation from damping |
| `energy-momentum/stress-test` | `example-energy-stress-test` | Working | Headless validation (12 checks) |

### Muscles

| Example | Package | Status | Notes |
|---------|---------|--------|-------|
| `muscles/stress-test` | `example-muscle-stress-test` | Working | Headless validation (51 checks: curves, dynamics, pipeline) |
| `muscles/forearm-flexion` | `example-muscle-forearm-flexion` | Working | Muscle lifts forearm against gravity, 3D tendon mesh |
| `muscles/activation` | `example-muscle-activation` | Working | Three time constants, rise/fall asymmetry |
| `muscles/cocontraction` | `example-muscle-cocontraction` | Working | Agonist-antagonist pair, joint stiffening |
| `muscles/force-length` | `example-muscle-force-length` | Working | Wide vs narrow FL range, swept joint position |

### URDF Loading

| Example | Package | Status | Notes |
|---------|---------|--------|-------|
| `urdf-loading/revolute` | `example-urdf-revolute` | Working | Revolute pendulum, period matches analytical |
| `urdf-loading/prismatic` | `example-urdf-prismatic` | Working | Slide joint, spring oscillation period |
| `urdf-loading/continuous` | `example-urdf-continuous` | Working | Unlimited revolute, alpha = tau/I |
| `urdf-loading/fixed` | `example-urdf-fixed` | Working | Fixed joint fusion, body count reduction |
| `urdf-loading/mimic` | `example-urdf-mimic` | Working | Leader-follower 2:1 coupling via equality |
| `urdf-loading/geometry` | `example-urdf-geometry` | Working | Box, sphere, cylinder size conversion |
| `urdf-loading/inertia` | `example-urdf-inertia` | Working | Diagonal vs full inertia, precession |
| `urdf-loading/damping-friction` | `example-urdf-damping-friction` | Working | Three colored pendulums: no-loss, damped, friction (6 checks) |
| `urdf-loading/error-handling` | `example-urdf-error-handling` | Working | Headless: invalid URDFs → correct error variants (7 checks) |
| `urdf-loading/stress-test` | `example-urdf-stress-test` | Working | Headless validation (31 checks) |

### Tendons

| Example | Package | Status | Notes |
|---------|---------|--------|-------|
| `tendons/stress-test` | `example-tendon-stress-test` | Working | Headless validation (16 checks) |
| `tendons/fixed-coupling` | `example-tendon-fixed-coupling` | Working | Two pendulums coupled by fixed tendon [1.0, -0.5] |
| `tendons/spatial-path` | `example-tendon-spatial-path` | Working | 3-site tendon on 2-link arm, starts at -90° |
| `tendons/sphere-wrap` | `example-tendon-sphere-wrap` | Working | Tendon wraps around sphere via great-circle arc |
| `tendons/cylinder-wrap` | `example-tendon-cylinder-wrap` | Working | Tendon wraps around cylinder via helical geodesic |
| `tendons/tendon-limits` | `example-tendon-limits` | Working | Pendulum tethered by limited tendon |
| `tendons/pulley` | `example-tendon-pulley` | Working | 2:1 mechanical advantage, resistance-band ribbon |
| `tendons/tendon-actuator` | `example-tendon-actuator` | Working | Tendon vs joint transmission comparison |

### Passive Forces

| Example | Package | Status | Notes |
|---------|---------|--------|-------|
| `passive-forces/stress-test` | `example-passive-stress-test` | Working | Headless validation (18 checks: fluid drag, ellipsoid model, wind, springs, dampers, implicit mode, disable flags) |
| `passive-forces/fluid-drag` | `example-passive-fluid-drag` | Working | 3 spheres (different mass) in dense medium — terminal velocity separation (5 checks) |
| `passive-forces/ellipsoid-drag` | `example-passive-ellipsoid-drag` | Working | 3 shapes (capsule/sphere/cylinder), equal mass — shape determines drag (4 checks) |
| `passive-forces/wind` | `example-passive-wind` | Working | Wind on/off: sphere drifts, pendulum deflects then recovers (3 checks) |
| `passive-forces/spring-damper-tuning` | `example-passive-spring-damper-tuning` | Working | Underdamped/critical/overdamped — 3 classical damping regimes (6 checks) |

### Sleep/Wake

| Example | Package | Status | Notes |
|---------|---------|--------|-------|
| `sleep-wake/stress-test` | `example-sleep-wake-stress-test` | Working | Headless validation (18 checks: threshold, countdown, wake triggers, islands, policy flags, narrowphase skip) |
| `sleep-wake/sleep-settle` | `example-sleep-wake-settle` | Working | 5 boxes drop and turn blue as they settle and sleep (4 checks) |
| `sleep-wake/wake-on-contact` | `example-sleep-wake-contact` | Working | Init-sleep box wakes on ball impact (5 checks) |
| `sleep-wake/island-groups` | `example-sleep-wake-islands` | Working | Two stacks — poke one, other stays asleep (5 checks) |

### Raycasting

| Example | Package | Status | Notes |
|---------|---------|--------|-------|
| `raycasting/stress-test` | `example-raycasting-stress-test` | Working | Headless validation (20 checks: all shapes, normals, scene queries, edge cases) |
| `raycasting/basic-shapes` | `example-raycasting-basic-shapes` | Working | 6 primitives struck by downward rays — hit dots + normal arrows (5 checks) |
| `raycasting/scene-query` | `example-raycasting-scene-query` | Working | LIDAR fan of 36 rays, body exclusion, geom group filter (4 checks) |
| `raycasting/heightfield` | `example-raycasting-heightfield` | Working | Ray marching on sinusoidal terrain, dot cloud tracing surface (3 checks) |

### Other

| Example | Package | Status | Notes |
|---------|---------|--------|-------|
| `pendulum-sim` | `example-pendulum-sim` | Working | Original raw MJCF demo |
| `multi-scene-test` | `example-multi-scene-test` | Working | Multi-scene infrastructure test |

## integration/

Pipeline composition: proving domains work together end-to-end.
All use `Mechanism::to_model()` (SDF collision + visual mesh, no MJCF round-trip).

| Example | Package | Pipeline | Status |
|---------|---------|----------|--------|
| `design-to-sim` | `example-design-to-sim` | design → simulate → visualize | Working |
| `design-to-print` | `example-design-to-print` | design → mesh → print | Working |
| `sim-informed-design` | `example-sim-informed-design` | design → simulate → stress-graded lattice | Working |
| `full-pipeline` | `example-full-pipeline` | design → simulate → stress → lattice → print | Working |

## sdf-physics/cpu/

Baby-step ladder proving the SDF collision thesis on the CPU pipeline.
**One new variable per step.** Each step depends on every previous step working.

| Step | Example | Package | What it proves | Status |
|------|---------|---------|---------------|--------|
| 01 | `01-sdf-grid` | `example-sdf-cpu-01-sdf-grid` | SdfGrid from solid sphere | Working |
| 02 | `02-thin-grid` | `example-sdf-cpu-02-thin-grid` | Thin-wall grid fidelity | Working |
| 03 | `03-freefall` | `example-sdf-cpu-03-freefall` | to_model() + gravity | Working |
| 04 | `04-rest` | `example-sdf-cpu-04-rest` | sdf_plane_contact | Working |
| 05 | `05-drop` | `example-sdf-cpu-05-drop` | Dynamic contact + restitution | Stub |
| 06 | `06-slide` | `example-sdf-cpu-06-slide` | Friction / tangential forces | Stub |
| 07 | `07-pair` | `example-sdf-cpu-07-pair` | sdf_sdf_contact | Working |
| 08 | `08-stack` | `example-sdf-cpu-08-stack` | Multi-body stacking | Working |
| 09 | `09-cube-in-box` | `example-sdf-cpu-09-cube-in-box` | Concave containment | Working |
| 10 | `10-ball-in-bowl` | `example-sdf-cpu-10-ball-in-bowl` | Curved concave SDF contact | Working |
| 11 | `11-hinge-free` | `example-sdf-cpu-11-hinge-free` | Joints with SDF bodies | Working |
| 12 | `12-hinge-wall` | `example-sdf-cpu-12-hinge-wall` | Articulated external contact | Stub |
| 13 | `13-hinge-stop` | `example-sdf-cpu-13-hinge-stop` | Parent-child SDF contact (convex) | Stub |
| 14 | `14-damped-hinge` | `example-sdf-cpu-14-damped-hinge` | Damping + parameter sensitivity | Stub |
| 15 | `15-concave-stop` | `example-sdf-cpu-15-concave-stop` | Concave parent-child contact | Stub |
| 16 | `16-socket` | `example-sdf-cpu-16-socket` | Full socket/condyle | Blocked |

## sdf-physics/gpu/

GPU physics demos — full wgpu compute pipeline (broadphase → narrowphase → solver → integration).
Free joints only, nv ≤ 60.

| Step | Example | Package | What it proves | Status |
|------|---------|---------|---------------|--------|
| 01 | `01-hockey` | `example-sdf-gpu-01-hockey` | Actuated impact, momentum transfer via GPU SDF | In progress |

## Key Architecture: `Mechanism::to_model()`

`cf-design` can build a `sim-core::Model` directly from a `Mechanism`,
bypassing the MJCF XML round-trip. Each part produces two geoms:

- **SDF geom** (collision): `SdfGrid` from the implicit surface — O(1)
  distance queries with exact distance semantics.
- **Mesh geom** (visual): triangle mesh from the same `Solid` — high-res
  rendering in Bevy.

Both derive from the identical mathematical object. What you see is what
physics simulates.
