# Examples Coverage Spec

**Status:** Draft — flesh out in next session
**Date:** 2026-03-25
**Goal:** 100% coverage of codebase capabilities

## Principle

Examples should mirror the distribution of code in the codebase. Every major
feature should have at least one dedicated example. Examples also serve as
integration tests — they find bugs that unit tests miss.

## Current State (2026-03-25)

- 232K LOC codebase, 4.3K LOC examples (1.9%)
- sim-core (60K LOC) + sim-mjcf (30K LOC) = 40% of codebase → 1 example
- cf-design (29K LOC) → 3 examples (proportional)
- mesh-* (19K LOC) → 1 example (proportional)
- sim-gpu (8K LOC) → 0 working examples
- SDF-CPU ladder: 10 working, 6 stubs, 1 blocked

## Gap Analysis

### Joint Types (4 types, 2 covered)
- Hinge → covered (pendulum, finger-design, hinges 11–14)
- Free → covered (all SDF physics)
- **Slide → ZERO examples**
- **Ball → ZERO examples**

### Geometry Types (8 types, ~6 covered)
- Plane, Sphere, Capsule, Cylinder, Box, Ellipsoid → covered
- **Mesh (explicit convex) → ZERO dedicated examples**
- **Hfield (height field terrain) → ZERO examples**
- Sdf → extensively covered

### Actuator System (mostly uncovered)
- Transmission: Joint ✓, Tendon ✓, **Site ✗, Body ✗, SliderCrank ✗, JointInParent ✗**
- Dynamics: None ✓, **Filter ✗, FilterExact ✗, Integrator ✗, Muscle ✗, HillMuscle ✗, User ✗**
- Gain: Fixed ✓, **Affine ✗, Muscle ✗, HillMuscle ✗, User ✗**
- Bias: None ✓, **Affine ✗, Muscle ✗, HillMuscle ✗, User ✗**

### Sensors (40+ types, ZERO examples)
- Position: JointPos, TendonPos, ActuatorPos, FramePos, FrameQuat, SubtreeCom...
- Velocity: JointVel, TendonVel, FrameLinVel, FrameAngVel, Gyro, Velocimeter...
- Force: Touch, Force, Torque, ActuatorFrc, JointActuatorFrc, JointLimitFrc...
- Spatial: Accelerometer, RangeFinder, GeomDist, GeomNormal...
- Other: Clock, BallQuat, BallAngVel, SubtreeAngMom...

### Collision (partially covered)
- SDF-plane ✓, SDF-SDF ✓, analytical convex ✓
- **Mesh-mesh ✗, mesh-plane ✗, convex-convex (GJK/EPA) ✗**
- **Height field ✗**
- **Friction tuning ✗** (06-slide is stub)
- **Restitution ✗** (05-drop is stub)
- **Contact parameter override ✗**

### Solvers & Integration
- Newton ✓ (default)
- **PGS ✗, CG ✗** (zero examples)
- Euler ✓ (default)
- **RK4 ✗, Implicit ✗, ImplicitFast ✗, ImplicitSpringDamper ✗**

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
    pendulum-sim/           # existing — MJCF hinge, energy
    slide-joint/            # NEW — prismatic joint, linear motion
    ball-joint/             # NEW — spherical joint, 3DOF rotation
    mjcf-loading/           # NEW — load from file, <include>, complex models
    urdf-loading/           # NEW — load URDF, compare with MJCF
    sensors/                # NEW — sensor gallery (touch, gyro, accel, frame)
    actuators/              # NEW — motor types, dynamics, transmission
    muscles/                # NEW — Hill muscle model, activation dynamics
    solvers/                # NEW — PGS vs CG vs Newton comparison
    integrators/            # NEW — Euler vs RK4 vs Implicit
    equality-constraints/   # NEW — weld, connect, distance
    contact-tuning/         # NEW — friction, restitution, solref/solimp
    inverse-dynamics/       # NEW — compute required torques
    energy-momentum/        # NEW — conservation tracking
```

### Track 2: SDF-CPU ladder (complete the 6 stubs)

```
sdf-physics/
  cpu/
    05-drop/       # implement: restitution + bounce
    06-slide/      # implement: friction on ground
    12-hinge-wall/ # implement: articulated external contact
    13-hinge-stop/ # implement: parent-child SDF contact
    14-damped-hinge/ # implement: damping parameter tuning
    15-concave-stop/ # implement: concave parent geometry
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
