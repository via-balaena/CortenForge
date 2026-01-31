# CortenForge Vision

> Industrial-grade foundation, unlimited application.

---

## What CortenForge Is

CortenForge is a **Rust-native simulation and fabrication SDK** for building physical things that move, sense, and act in the world.

It is not:
- A game engine (though it runs on one)
- A CAD program (though it has CAD-grade geometry)
- A physics simulator (though it simulates physics)
- An ML framework (though it trains models)

It is all of these composed into a coherent system where **the same code runs in simulation and on hardware**.

---

## The Core Insight

The gap between simulation and reality is the central problem of robotics, autonomous vehicles, and smart manufacturing. Teams build in one environment, deploy to another, and spend months bridging the gap.

CortenForge eliminates this by design:

```
┌─────────────────────────────────────────────────────────────────┐
│                      SAME TYPES                                  │
│                      SAME TRAITS                                 │
│                      SAME CODE                                   │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│     SIMULATION                          HARDWARE                │
│     ┌─────────────┐                    ┌─────────────┐          │
│     │ Bevy World  │                    │ Real World  │          │
│     │             │                    │             │          │
│     │ SimCamera   │ ←─ CameraFrame ──→ │ RealCamera  │          │
│     │ SimIMU      │ ←─ ImuReading ───→ │ RealIMU     │          │
│     │ SimLiDAR    │ ←─ LidarScan ────→ │ RealLiDAR   │          │
│     │ SimMotor    │ ←─ JointState ───→ │ RealMotor   │          │
│     │             │                    │             │          │
│     └─────────────┘                    └─────────────┘          │
│            │                                  │                 │
│            └──────────┬───────────────────────┘                 │
│                       ▼                                         │
│              ┌─────────────────┐                                │
│              │  Your Control   │                                │
│              │     Code        │                                │
│              │  (runs both)    │                                │
│              └─────────────────┘                                │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

A `sensor_fusion::StreamSynchronizer` doesn't care if its inputs come from Bevy render passes or USB drivers. A `route_pathfind::VoxelAStar` doesn't care if the occupancy grid came from simulated LiDAR or real point clouds. The ML model trained on synthetic data runs inference on real frames without modification.

---

## Architecture Philosophy

### Layer 0: Pure Rust Foundation

**Zero Bevy dependencies. Zero framework lock-in.**

Every Layer 0 crate compiles to:
- Native binaries
- WASM for browsers
- Python modules via PyO3
- Embedded targets
- Other game engines

This is non-negotiable. Layer 0 is the **permanent foundation**. Bevy could be replaced tomorrow and Layer 0 would remain intact.

```
Layer 0 (Pure Rust)
├── mesh/*           27 crates - Industrial mesh processing
├── geometry/*       Curves, surfaces, solids
├── sensor/*         Hardware-agnostic sensor types
├── ml/*             Burn-native ML pipeline
├── routing/*        Path planning and optimization
├── spatial/*        Voxel grids, occupancy maps
└── sim/*            13 L0 crates - MuJoCo-aligned physics (NO BEVY)
```

### Layer 1: Bevy Integration

**ECS-native, not wrapped legacy code.**

Layer 1 is plugins that wire Layer 0 into Bevy's ECS. Components hold data. Systems process it. Resources manage state. This is Bevy's paradigm, fully embraced.

```
Layer 1 (Bevy SDK)
└── cortenforge/
    ├── CfMeshPlugin      Asset loading, visualization
    ├── CfSensorPlugin    Simulated sensors, readback
    ├── CfMlPlugin        Async inference scheduling
    ├── CfSimPlugin       Physics stepping, world state
    └── CfUiPlugin        Debug visualization, overlays
```

When Bevy breaks (and it will), Layer 1 adapts. Layer 0 is untouched.

### The Simulation Question

**Simulation lives in BOTH layers.**

```
sim/ (Layer 0 - Pure Rust)
├── sim-types        RigidBodyState, Pose, Twist, MassProperties
├── sim-simd         SIMD batch operations (Vec3x4/Vec3x8)
├── sim-core         MuJoCo-aligned pipeline: Model/Data, FK, CRBA, RNE, PGS
├── sim-contact      Compliant contact model, friction, domain randomization
├── sim-constraint   Joint types, motors, limits, equality constraints
├── sim-sensor       IMU, F/T, touch, rangefinder, magnetometer
├── sim-deformable   XPBD soft bodies (ropes, cloth, volumes)
├── sim-muscle       Hill-type muscle model
├── sim-tendon       Cable/tendon routing and actuation
├── sim-mjcf         MuJoCo XML/MJB format parser
├── sim-urdf         URDF parser, kinematic tree validation
├── sim-physics      Unified L0 API re-exporting all sim crates
└── sim-conformance-tests  MuJoCo conformance tests

cortenforge/ (Layer 1 - Bevy)
├── sim-bevy         Model/Data sync, debug gizmos, coordinate conversion
├── CfSimPlugin      Bevy <-> sim-core bridge (planned)
└── SimulationState  Pause, step, reset controls (planned)
```

Why both? Because simulation math is pure. `F = ma` doesn't need a game engine. But *visualizing* simulation, *interacting* with it, *recording* it - that's where Bevy excels.

A headless training run uses `sim-core` directly. A visualization demo uses `CfSimPlugin`. Same physics. Different interfaces.

---

## Domain Coverage

### What We Build vs. What We Use

| Domain | Strategy | Rationale |
|--------|----------|-----------|
| **Mesh Processing** | Build | Core competency. 27 crates, A-grade. |
| **Geometry** | Build + Truck | Curves/surfaces ourselves, BREP via truck |
| **Sensors** | Build | Must own for sim/real parity |
| **ML Pipeline** | Build + Burn | Types ourselves, Burn for tensors |
| **Routing** | Build + pathfinding | Core algorithms ourselves, graph utils from crate |
| **Spatial** | Build + kiddo | Voxels ourselves, KD-trees from crate |
| **Collision** | Build | MuJoCo-aligned: analytical + GJK/EPA + BVH mesh |
| **Rigid Physics** | Build | MuJoCo-aligned: Model/Data, CRBA, RNE, PGS solver |
| **Soft Physics** | Build | XPBD (sim-deformable), Hill-type muscles (sim-muscle) |
| **Rendering** | Use Bevy | Not our domain |
| **UI** | Use bevy_egui | Not our domain |

### Domain Roadmap

```
COMPLETE (52 crates):
├── Mesh Domain (27)
├── Sensor Domain (2)
├── ML Domain (4)
├── Spatial Domain (1)
├── Routing Domain (3)
├── Geometry Domain (1)
└── Simulation Domain (13 L0 + 1 L1)

NEXT PHASE:
├── surface-types      NURBS surfaces, patches
├── PyO3 bindings      Python access to Layer 0
└── cortenforge        Bevy SDK umbrella (CfSimPlugin, etc.)

FUTURE:
├── solid-types        BREP solids via truck
├── constraint-solver  Assembly constraints
└── Hardware bridges   ROS2, custom drivers
```

---

## Quality Standard

**A-grade or it doesn't ship.**

This is not marketing. This is policy enforced by CI:

1. **Zero `unwrap()`/`expect()` in library code** - Errors are values
2. **≥75% test coverage (target: 90%)** - Measured and enforced
3. **Zero Clippy warnings** - pedantic + nursery enabled
4. **Doc examples for all public APIs** - Tested in CI
5. **Conventional commits** - Machine-readable history
6. **Semver compliance** - cargo-semver-checks on every PR
7. **Supply chain verified** - cargo-vet, cargo-audit, cargo-deny

Why this strict? Because:
- Robots hurt people when software fails
- Medical devices kill when software fails
- Vehicles crash when software fails

CortenForge is built for domains where failure has consequences.

---

## The Behemoth Comparison

### TensorFlow Extended (TFX)
What they have: End-to-end ML pipelines, data validation, model serving, continuous training.

What we have: `ml-types`, `ml-dataset`, `ml-models`, `ml-training`. The primitives.

What we need: Pipeline orchestration, model registry, A/B deployment infrastructure.

### Siemens NX / Solid Edge
What they have: Full parametric CAD, assembly constraints, simulation integration, 40 years of development.

What we have: Mesh processing, curves, basic BREP via truck.

What we need: Parametric history, constraint solver, assembly management.

We're not trying to replace Siemens. We're building the foundation that could *interoperate* with it. Import STEP, manipulate, export. The industrial pipeline.

### MuJoCo
What they have: 15 years of physics research, soft contacts, implicit integration, GPU acceleration, massive adoption in RL research.

What we have: A MuJoCo-aligned pipeline (14 crates) implementing Model/Data, FK, CRBA, RNE, PGS contact solver, implicit integration, MJCF/URDF loading, compliant contacts, Hill-type muscles, XPBD deformables.

What we need: GPU acceleration, broader actuator coverage, tendon/site transmissions, full MuJoCo XML parity.

The honest assessment: We can match TFX patterns in months. We can get useful CAD interop in a year. Full MuJoCo parity (GPU, all actuators, all constraint types) is ongoing work. See `sim/ARCHITECTURE.md` and `sim/MUJOCO_REFERENCE.md`.

---

## Target Applications

### Humanoid Robots
- Mesh: Body shells, gripper designs, sensor housings
- Simulation: Walking, manipulation, contact-rich tasks
- ML: Imitation learning, reinforcement learning
- Sensors: Vision, proprioception, force sensing

### Autonomous Vehicles
- Mesh: Chassis design, sensor mounts
- Simulation: Traffic, sensors, weather
- ML: Perception, prediction, planning
- Routing: Path planning, trajectory optimization

### Medical Devices
- Mesh: Implant design, surgical tools
- Simulation: Tissue interaction, device mechanics
- ML: Image analysis, outcome prediction
- Geometry: Patient-specific modeling from scans

### Smart Manufacturing
- Mesh: Part design, fixture generation
- Simulation: Process validation
- ML: Defect detection, process optimization
- Routing: Tool paths, robot motion

All four domains share:
- Mesh processing
- Simulation
- ML pipelines
- Sensor integration
- Path planning

CortenForge is the shared foundation.

---

## Growth Model

### Architecture That Welcomes Change

The crate structure is designed for AI-assisted development:

1. **Small, focused crates** - Each crate fits in context
2. **Clear boundaries** - Dependencies are explicit in Cargo.toml
3. **Comprehensive tests** - AI can verify changes work
4. **Documentation in code** - Doc comments are the source of truth

When AI capabilities improve:
- Larger context = more crates considered together
- Better reasoning = more complex algorithms implemented correctly
- Faster iteration = more experiments, faster convergence

The CI/CD pipeline is the immune system. It catches regressions regardless of who (or what) wrote the code.

### Breaking Changes Are Welcome

Bevy 0.16 will break things. Bevy 1.0 will break things. This is fine.

Layer 0 is untouched. Layer 1 adapts. The architecture absorbs change because the boundaries are clean.

When we need to rebuild Layer 1 from scratch? The AI does it in a day. The tests verify it works. Ship it.

### The Rebuild Scenario

"My AI literally just rebuilt everything ever written on my local home lab."

This is coming. The question is: what survives a rebuild?

**What survives:**
- Type definitions (they encode domain knowledge)
- Test cases (they encode expected behavior)
- Documentation (it encodes intent)
- Architecture decisions (INFRASTRUCTURE.md, this document)

**What gets rebuilt:**
- Implementation details
- Boilerplate
- Anything that can be derived from types + tests

CortenForge is structured so the *permanent* artifacts (types, tests, docs) are maximally information-dense, and the *transient* artifacts (implementations) can be regenerated.

---

## What "Done" Means

There is no "done." Cathedrals are never finished; they're maintained and extended.

But there are milestones:

### Milestone 1: Simulation Foundation (Largely Complete)
- 14 sim crates implemented (MuJoCo-aligned pipeline)
- Model/Data architecture with forward kinematics, dynamics, contact solver
- sim-bevy provides Bevy visualization with debug gizmos
- Headless training loop works via sim-core directly

### Milestone 2: Hardware Bridge
- Real sensor data flows through `sensor-types`
- Control commands go to real actuators
- Same code runs in sim and on hardware

### Milestone 3: Full Pipeline
- Design in CAD (external) → import STEP
- Simulate in CortenForge
- Train policies in simulation
- Deploy to hardware
- Iterate

### Milestone 4: Community
- Documentation complete enough for external contributors
- Examples for each major use case
- Python bindings stable
- First external production use

---

## Principles

1. **Layer 0 purity is sacred.** No exceptions. Ever.

2. **Tests are documentation.** If behavior isn't tested, it's not guaranteed.

3. **Types are contracts.** Get them right first. Implementation follows.

4. **Simple > Clever.** The next person reading this code might be an AI that needs to modify it.

5. **Fail fast, fail loud.** No silent corruption. Errors are values. Handle them.

6. **Own the core, use the rest.** We build what defines us. We use what doesn't.

7. **Quality over velocity.** A-grade takes longer. It's worth it.

---

## For Contributors

(Future section - when we're ready for external contributions)

The bar is high. This is intentional.

Before contributing:
1. Read STANDARDS.md
2. Read CONTRIBUTING.md
3. Read INFRASTRUCTURE.md
4. Run `cargo xtask check`
5. Understand why the rules exist

We're building something that will run on robots, in vehicles, in medical devices. The stakes justify the standards.

---

*This document is the north star. When in doubt, check here.*

*Last updated: 2026-01-30*
