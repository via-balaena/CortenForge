# CortenForge Examples

## Directory Structure

```
examples/
  fundamentals/
    design/           Pure cf-design (implicit surfaces, bio-inspired geometry)
    mesh/             Pure mesh-* (repair, lattice, shell, printability)
    sim-cpu/          CPU physics fundamentals (MJCF, joints, sensors)
    sim-ml/           ML boundary layer (spaces, environments, learning algorithms)
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
| `integrators/stress-test` | `example-integrator-stress-test` | Working | Headless energy drift table, 7 checks (validator) |
| `integrators/comparison-visual` | `example-integrator-comparison-visual` | Working | 3 double pendulums side by side |
| `integrators/euler` | `example-integrator-euler` | Working | Semi-implicit Euler, visible drift |
| `integrators/rk4` | `example-integrator-rk4` | Working | 4th-order Runge-Kutta, near-perfect |
| `integrators/implicit` | `example-integrator-implicit` | Working | Full implicit, L-stable |
| `integrators/implicit-fast` | `example-integrator-implicit-fast` | Working | Symmetric D, no Coriolis |
| `integrators/implicit-spring-damper` | `example-integrator-implicit-spring-damper` | Working | Diagonal spring/damper path |

### Solvers

| Example | Package | Status | Notes |
|---------|---------|--------|-------|
| `solvers/stress-test` | `example-solver-stress-test` | Working | Headless convergence table, 7 checks |
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
| `urdf-loading/stress-test` | `example-urdf-stress-test` | Working | Headless validation (35 checks, incl. error-variant handling) |

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

### Composites

| Example | Package | Status | Notes |
|---------|---------|--------|-------|
| `composites/stress-test` | `example-composite-stress-test` | Working | Headless validation (11 checks: body count, joints, exclusions, convergence, length, curves, independence) |
| `composites/hanging-cable` | `example-composite-hanging-cable` | Working | 3 cables (5/10/20 segments) pinned at one end, resolution comparison (5 checks) |
| `composites/cable-catenary` | `example-composite-cable-catenary` | Working | Both-ends-fixed gold cable, catenary sag (4 checks) |
| `composites/cable-loaded` | `example-composite-cable-loaded` | Working | Midpoint 5N force, V-shape sag, red load indicator (4 checks) |

### Batch Simulation

| Example | Package | Status | Notes |
|---------|---------|--------|-------|
| `batch-sim/parameter-sweep` | `example-batch-sim-parameter-sweep` | Working | 8 pendulums, different damping, parallel via `step_all()` (4 checks) |
| `batch-sim/reset-subset` | `example-batch-sim-reset-subset` | Working | 12 landers converging on soft landing via `reset_where()` (4 checks) |
| `batch-sim/stress-test` | `example-batch-sim-stress-test` | Working | Headless validation (10 checks: construction, independence, determinism, resets, parity) |

### Raycasting

| Example | Package | Status | Notes |
|---------|---------|--------|-------|
| `raycasting/stress-test` | `example-raycasting-stress-test` | Working | Headless validation (20 checks: all shapes, normals, scene queries, edge cases) |
| `raycasting/basic-shapes` | `example-raycasting-basic-shapes` | Working | 6 primitives struck by downward rays — hit dots + normal arrows (5 checks) |
| `raycasting/scene-query` | `example-raycasting-scene-query` | Working | LIDAR fan of 36 rays, body exclusion, geom group filter (4 checks) |
| `raycasting/heightfield` | `example-raycasting-heightfield` | Working | Ray marching on sinusoidal terrain, dot cloud tracing surface (3 checks) |

### Derivatives

| Example | Package | Status | Notes |
|---------|---------|--------|-------|
| `derivatives/linearize-pendulum` | `example-derivatives-linearize-pendulum` | Working | `mjd_transition_fd` — A, B matrices, eigenvalue analysis (4 checks) |
| `derivatives/control-design` | `example-derivatives-control-design` | Working | LQR from linearization — A, B to closed-loop cart-pole balance (4 checks) |
| `derivatives/hybrid-vs-fd` | `example-derivatives-hybrid-vs-fd` | Working | `mjd_transition_hybrid` vs FD — accuracy + timing race (4 checks) |
| `derivatives/sensor-jacobians` | `example-derivatives-sensor-jacobians` | Working | C, D sensor derivative matrices — predicted vs actual (5 checks) |
| `derivatives/inverse-dynamics` | `example-derivatives-inverse-dynamics` | Working | `mjd_inverse_fd` — DfDq, DfDv, DfDa, verify DfDa = M (4 checks) |
| `derivatives/convergence` | `example-derivatives-convergence` | Working | Centered O(eps^2) vs forward O(eps) — clock-hand sweep (4 checks) |
| `derivatives/epsilon-vcurve` | `example-derivatives-epsilon-vcurve` | Working | Optimal epsilon — truncation vs roundoff V-curve (4 checks) |
| `derivatives/stress-test` | `example-derivatives-stress-test` | Working | Headless validation (32 checks) |

### Mesh Collision

| Example | Package | Status | Notes |
|---------|---------|--------|-------|
| `mesh-collision/mesh-on-plane` | `example-mesh-collision-plane` | Working | Tetrahedron on ground plane, `collide_mesh_plane()` |
| `mesh-collision/mesh-sphere` | `example-mesh-collision-sphere` | Working | Sphere on mesh slab, BVH `mesh_sphere_contact()` |
| `mesh-collision/mesh-box` | `example-mesh-collision-box` | Working | Box on mesh slab, GJK/EPA + MULTICCD |
| `mesh-collision/mesh-capsule` | `example-mesh-collision-capsule` | Working | Sideways capsule on mesh slab, BVH `mesh_capsule_contact()` |
| `mesh-collision/mesh-cylinder` | `example-mesh-collision-cylinder` | Working | Upright cylinder, GJK/EPA + MULTICCD (branch highlight) |
| `mesh-collision/mesh-ellipsoid` | `example-mesh-collision-ellipsoid` | Working | Oblate disc, GJK/EPA exact support (branch highlight) |
| `mesh-collision/mesh-on-mesh` | `example-mesh-collision-mesh` | Working | Wedge on mesh platform, hull-hull GJK/EPA + MULTICCD |
| `mesh-collision/stress-test` | `example-mesh-collision-stress-test` | Working | Headless validation (21 checks: all 7 pairs + edge cases) |

### Other

| Example | Package | Status | Notes |
|---------|---------|--------|-------|
| `pendulum-sim` | `example-pendulum-sim` | Working | Original raw MJCF demo |
| `multi-scene-test` | `example-multi-scene-test` | Working | Multi-scene infrastructure test |

## fundamentals/sim-ml/

ML boundary layer — observation/action spaces, environments, learning algorithms.

### Vec-Env Learning Algorithms

| Example | Package | Status | Notes |
|---------|---------|--------|-------|
| `vec-env/stress-test` | `example-ml-vec-env-stress-test` | Working | Headless VecEnv correctness (11 checks) |
| `vec-env/parallel-step` | `example-ml-vec-env-parallel-step` | Working | 8 pendulums, different actions |
| `vec-env/auto-reset` | `example-ml-vec-env-auto-reset` | Working | ★ 50 reaching arms + CEM (8 tests) |
| `vec-env/reinforce` | `example-ml-vec-env-reinforce` | Working | ★ 50 reaching arms + REINFORCE (7 tests) |
| `vec-env/ppo` | `example-ml-vec-env-ppo` | Working | ★ 50 reaching arms + PPO (9 tests) |
| `vec-env/td3` | `example-ml-vec-env-td3` | Working | ★ 50 reaching arms + TD3 (5 tests) |
| `vec-env/sac` | `example-ml-vec-env-sac` | Working | ★ 50 reaching arms + SAC (5 tests) |

## sim-soft/

Soft-body FEM examples — Neo-Hookean elasticity on linear tetrahedra, BCC + Labelle-Shewchuk SDF meshing, multi-material partitioning, penalty contact with rigid primitives. See [`sim/L0/soft/EXAMPLE_INVENTORY.md`][sim-soft-inv] for the full 3-PR plan + tier breakdown.

[sim-soft-inv]: ../sim/L0/soft/EXAMPLE_INVENTORY.md

### Tier 1 — SDF primitives + meshing

| Example | Package | Status | Notes |
|---------|---------|--------|-------|
| `sdf/stress-test` | `example-sdf-stress-test` | Working (validator) | SDF validation superset, rows 1–3 folded (one domain → one stress-test): module `sphere_eval` (`Sdf` trait contract on `SphereSdf`, 11³ grid + FD-Eikonal), `hollow_shell` (`DifferenceSdf` hollow-body composition, z=0 slice with `signed_distance` + `active_branch`), `sdf_to_tet` (`SdfMeshedTetMesh::from_sdf` BCC + Isosurface Stuffing, 1224 faces, Euler χ = 2, bimodal `boundary_residual`) |

*Row 4 (`single-tet-stretch`) folded into the `stretch/stress-test` validator (module `single_tet`) alongside rows 5–6 — see Tier 2.*

### Tier 2 — Constitutive + multi-element

| Example | Package | Status | Notes |
|---------|---------|--------|-------|
| `stretch/stress-test` | `example-stretch-stress-test` | Working (validator) | Uniaxial-stretch validation superset on the canonical compressible NH baseline (`μ = 1e5`, `Λ = 4e5` Pa), rows 4–6 folded (one domain → one stress-test; a solver → constitutive → assembly ladder): module `single_tet` (row 4 — `SkeletonSolver::step` on `SoftScene::one_tet_cube`, `θ = 10 N`, 3 iters, `dz ≈ 9.69e-4 m`, 12 IV-1 bit-pins), `neo_hookean` (row 5 — direct-eval `Material::first_piola` / `energy` vs closed form across a 12-point traction-free sweep with inner Newton on `λ_t`, `ValidityDomain` check, 48 captured-bit self-pins, optional `uv run plot.py` panel), `multi_element` (row 6 — Phase 2 multi-element FEM assembly on a 27-vertex / 48-tet hex grid at `λ = 1.20` with one interior vertex free, per-tet `F` uniform `diag(λ, 1, 1)` across all 48 tets, quasi-static via `cfg.density = 0`, 10 sparse-tier bit-pins). JSON-only traces under `out/<module>/` |

### Tier 3 — Multi-material spatial fields

| Example | Package | Status | Notes |
|---------|---------|--------|-------|
| `scalar-field/stress-test` | `example-scalar-field-stress-test` | Working (validator) | `ScalarField` validation superset, rows 8–9 folded (one domain → one stress-test): module `layered` (3-shell `LayeredScalarField` sharp CSG step, categorical `material_layer_id` ∈ {0,1,2}, exact per-shell counts, `interface_flags`-all-false contract) + `blended` (`BlendedScalarField` cubic-Hermite smoothstep, monotone μ gradient, bit-exact s=0/s=1 snap, positive IV-6 `interface_flag` book rule; scalars `interface_flag`/`material_mu`/`smoothstep_weight`) |
| `bonded/stress-test` | `example-bonded-stress-test` | Working (validator) | Bonded multi-material validation superset — shared-vertex (C⁰, no-slip) multi-material bodies validated against closed-form elasticity, rows 10–11 folded (one domain → one stress-test): module `bilayer_beam` (row 10 — Phase 4 IV-3 bonded bilayer cantilever beam, 1701 verts / 7680 tets, inline `HalfSpaceField` region A `1×` / region B `2×`; tip displacement matches Euler-Bernoulli composite-beam analytic within 30 % at h/2 (Tet4 sub-`O(h²)` per IV-3); IV-2-lens-β strict-between-uniform-bounds discriminator; first IV-2 shared-vertex continuity gate at production scale; y-slab PLY, `DISPLACEMENT_SCALE = 20×`) + `lame_shells` (row 11 — IV-5 three-shell concentric hollow silicone sphere, BCC + Labelle-Shewchuk on a `DifferenceSdf` of two `SphereSdf`s, `R_OUTER = 0.10 m` / `R_CAVITY = 0.04 m`, 6456 tets, 3-shell `MaterialField` `1× / 2× / 1×`; radially-outward pressure traction `LoadAxis::FullVector` on cavity, fixed Dirichlet outer; four Saint-Venant radial-displacement readouts match piecewise-Lamé closed-form within 30 % at h/2; IV-2-lens-β strict-between gate runs three solver passes; z-slab PLY, `DISPLACEMENT_SCALE = 50×`) |

### Tier 4 — Penalty contact (PR2 shipped at main `55cc3a42`)

| Example | Package | Status | Notes |
|---------|---------|--------|-------|
| `soft-drop-on-plane` | `example-sim-soft-soft-drop-on-plane` | Working | ★ Soft sphere released above `RigidPlane`, settles into quiescence; first sim-bevy-soft consumer (CF_VISUAL=1) |
| `hertz-sphere-plane` | `example-sim-soft-hertz-sphere-plane` | Working | ★ Soft sphere quasi-statically pressed; contact-patch radius vs Hertz analytic (V-3); cf-view colormap PLY + matplotlib pressure-profile plot |
| `compressive-block` | `example-sim-soft-compressive-block` | Working | ★ Soft cube quasi-statically compressed by descending plate; per-refinement F_R two-bound bracket [F_us, F_strain] + Cauchy convergence (V-3a); cf-view colormap PLY + matplotlib F-vs-ε scatter |
| `contact-force-readout` | `example-sim-soft-contact-force-readout` | Working | ★ Per-active-pair contact readout via `PenaltyRigidContact::per_pair_readout` (foundation patch `995fb0bf`); same V-3a scene as `compressive-block`; accessor-vs-manual consistency at 1e-12 rel; cf-view pressure-colormap PLY + matplotlib top-down patch scatter |

### Tier 5 — Bridges + extensions (PR3 substance complete on `dev`, awaiting squash-merge to main)

| Example | Package | Status | Notes |
|---------|---------|--------|-------|
| `sdf-bridge/stress-test` | `example-sdf-bridge-stress-test` | Working (validator) | SDF-bridge validation superset — both directions of the `cf_design::Sdf` trait interop spine, rows 15–16 folded (one domain → one stress-test): module `mesh_scan` (row 15 — scan→design: `mesh_sdf::Signed<TriMeshDistance, PseudoNormalSign>` satisfies `cf_design::Sdf` (PR3 F2); 12-tri programmatic cube round-tripped through a runtime-written STL, closed-form L∞-ball anchors via `&dyn cf_design::Sdf`, 17³ = 4913 bulk grid PLY (`signed_distance` + `inside_raycast`), the `PseudoNormalSign` inside-set proven equal to the closed cube by per-point set-equality (`raycast_inside = 9³ = 729`, divergence `= 729 − 343 = 386` as closed-form identities)) + `solid_to_sim` (row 16 — design→sim: typed `cf_design::Solid` CSG body drives `SdfMeshedTetMesh::from_sdf` via the F1+F3 bridge; HEADLINE A bit-exact bridge-equivalence vs `DifferenceSdf<SphereSdf>` baseline, single-material Lamé cavity-wall readout within 30 %, mesher-version-robust structural count invariants — absolute counts unpinned, like row 11) |
| `material/stress-test` | `example-material-stress-test` | Working (validator) | Material-reference validation superset (row 19 relocated into the standardization layout — singleton `material` domain, module `material_table`): engineering-grade Smooth-On platinum-cure silicone Lamé pairs + density via PR3 F4's `silicone_table.rs` const module. Iterates 7 `pub const SiliconeMaterial` entries (`{Ecoflex 00-10/20/30/50, Dragon Skin 10A/20A/30A}`), dispatches each via `SiliconeMaterial::to_neo_hookean()` (`const` bridge into the `Material` trait), probes at `F = diag(2.0, 1, 1)` (the `σ_100 = 100% engineering strain` data-sheet anchor); 7 verify groups + 21 captured-bit self-pins; JSON-only, no solver/mesh |

### Tier 6 — Synthesis (PR3 final row)

| Example | Package | Status | Notes |
|---------|---------|--------|-------|
| `layered-silicone-device` | `example-sim-soft-layered-silicone-device` | Working | ★ The PR3 Tier 6 synthesis row — composes every PR3 foundation piece (F1–F5) into one end-to-end relative-comparison sim of the layered silicone device cavity-fit. Scan SDF (F2) → typed `Solid::from_sdf` heterogeneous CSG (F5) → `SdfMeshedTetMesh::from_sdf` (F1 + F3) → 3-shell `MaterialField` from F4 (`ECOFLEX_00_30` outer + inner / `DRAGON_SKIN_10A` middle as the conductive-composite proxy) → static fit pose with the same scan SDF as `PenaltyRigidContact` rigid indenter (first non-plane consumer post-PR2 trait unification). Outputs `layered_silicone_device.json` (fit-pose scalars + materials + per-active-pair detail) + `device_zslab.ply` (categorical `material_id` + sequential `displacement_magnitude`, z-slab per pattern (aa) for hollow geometry) |

### Visual-mode convention: `CF_VISUAL=1`

Tier 4 examples (`soft-drop-on-plane` + `hertz-sphere-plane` + `compressive-block` + `contact-force-readout`) ship a **headless harness + opt-in Bevy windowed visualization**. Default invocation (no env var) runs all `verify_*` asserts and emits the static PLY artifact — no display / winit / OpenGL required, suitable for CI. Setting `CF_VISUAL=1` (any non-empty value) additionally spawns a Bevy app via [`sim_bevy_soft::SoftBodyVisualPlugin`][sbsp] that renders the captured trajectory or static state (depending on the row — `soft-drop-on-plane` replays a 1000-frame freefall trajectory; `hertz-sphere-plane` / `compressive-block` / `contact-force-readout` render single quasi-static settled frames):

```text
cargo run -p example-sim-soft-soft-drop-on-plane --release           # headless asserts + PLY
CF_VISUAL=1 cargo run -p example-sim-soft-soft-drop-on-plane --release # + Bevy windowed visualization
```

The replay clock is per-entity — a `ReplayEpoch` component captures the wall-clock at the first `step_replay` tick (so `DefaultPlugins` startup time doesn't consume playback budget) and `step_replay` thereafter computes frame index against `(now - epoch)`. Press `R` to clear all entities' epochs and replay from frame 0. Default replay rate is `1×` wall-clock; consumers can override per spawn (row 12 ships at `10×` slow-motion since its `~89 ms` freefall is blink-and-miss-it at real-time). Replay clamps at end (no looping). Pause / scrub controls are out of scope — defer to a future row per the [`step_replay`][step-replay] docs.

[sbsp]: ../sim/L1/sim-bevy-soft/src/plugin.rs
[step-replay]: ../sim/L1/sim-bevy-soft/src/trajectory.rs

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
