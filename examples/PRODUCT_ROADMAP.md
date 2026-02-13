# CortenForge Product Roadmap

> **18 products across 6 phases.** Each phase adds new CortenForge domains. Each product is a real, physical artifact that validates the SDK through manufacturing.

> **Philosophy:** Three products per phase to stress-test every crate across different geometries, constraints, and manufacturing methods. By Phase 6, every domain has been battle-hardened in at least three different contexts.

---

## Overview

```
Phase 1: Mesh                          -> Shoe, Insole, Tool Handle
Phase 2: +Sim +Geometry                -> Gripper, Prosthetic Finger, Walker
Phase 3: +Sensors +Routing             -> Leveling Platform, Mobile Robot, Gesture Arm
Phase 4: +Deformables                  -> Pneumatic Finger, Variable Pad, Soft Crawler
Phase 5: +Muscles +Tendons +ML         -> Tendon Hand, Hopping Leg, Tactile Gripper
Phase 6: Everything                    -> Prosthetic Socket, Massage Device, Tentacle Arm
```

---

## Phase 1: Mesh Pipeline (Scan -> Design -> Manufacture)

**New domains:** mesh-scan, mesh-repair, mesh-boolean, mesh-offset, mesh-shell, mesh-lattice, mesh-printability, mesh-io, curve-types

**Goal:** Prove the full scan-to-manufacture pipeline across three different product geometries and manufacturing methods.

| Product | Description | Manufacturing |
|---------|-------------|---------------|
| [Custom Shoe](./phase-1-mesh/custom-shoe/SPEC.md) | 3D foot scan -> parametric sole -> lattice midsole -> printable shoe | FDM/SLS 3D print |
| [Orthotic Insole](./phase-1-mesh/orthotic-insole/SPEC.md) | Foot scan + pressure map -> variable-density lattice insole | FDM/SLS 3D print or CNC |
| [Ergonomic Handle](./phase-1-mesh/ergonomic-handle/SPEC.md) | Hand scan -> grip cavity -> offset walls -> ergonomic tool handle | FDM 3D print |

**Crate coverage:**

| Crate | Shoe | Insole | Handle |
|-------|------|--------|--------|
| mesh-scan | x | x | x |
| mesh-repair | x | x | x |
| mesh-boolean | x | | x |
| mesh-offset | x | | x |
| mesh-shell | x | x | |
| mesh-lattice | x | x | |
| mesh-printability | x | x | x |
| mesh-io | x | x | x |
| mesh-slice | | x | |
| mesh-morph | | | x |
| mesh-from-curves | x | | x |
| curve-types | x | | x |

---

## Phase 2: Mechanism Design + Simulation

**New domains:** sim-types, sim-core, sim-constraint, sim-mjcf, sim-urdf, mesh-assembly

**Goal:** Introduce rigid body simulation. Design parametric mechanisms, simulate kinematics and dynamics, verify against physical prototypes.

| Product | Description | Manufacturing |
|---------|-------------|---------------|
| [Compliant Gripper](./phase-2-mechanism/compliant-gripper/SPEC.md) | Two-finger flexure gripper with material compliance | FDM (flexible filament) |
| [Prosthetic Finger](./phase-2-mechanism/prosthetic-finger/SPEC.md) | Bio-inspired finger with tendon-routed actuation | FDM + cable |
| [Four-Bar Walker](./phase-2-mechanism/four-bar-walker/SPEC.md) | Passive walking toy (Strandbeest-style linkage) | FDM + metal pins |

**Crate coverage:**

| Crate | Gripper | Finger | Walker |
|-------|---------|--------|--------|
| sim-types | x | x | x |
| sim-core | x | x | x |
| sim-constraint | x | x | x |
| sim-mjcf or sim-urdf | x | x | x |
| curve-types | x | x | x |
| mesh-from-curves | x | x | x |
| mesh-assembly | x | x | x |
| mesh-printability | x | x | x |

---

## Phase 3: Perception + Control Loop

**New domains:** sensor-types, sensor-fusion, route-types, route-pathfind, sim-sensor

**Goal:** Close the loop. Sense the world, decide what to do, act. Same control code runs in simulation and on hardware.

| Product | Description | Hardware |
|---------|-------------|----------|
| [Leveling Platform](./phase-3-perception/leveling-platform/SPEC.md) | IMU-fused self-leveling platform with two servos | IMU + 2 servos |
| [Obstacle-Avoiding Robot](./phase-3-perception/obstacle-avoiding-robot/SPEC.md) | Wheeled robot with range sensors and path replanning | Range sensors + motors |
| [Gesture-Controlled Arm](./phase-3-perception/gesture-controlled-arm/SPEC.md) | IMU glove teleoperating a 3-DOF robot arm | IMU glove + 3-DOF arm |

**Crate coverage:**

| Crate | Platform | Robot | Arm |
|-------|----------|-------|-----|
| sensor-types | x | x | x |
| sensor-fusion | x | x | x |
| sim-sensor | x | x | x |
| route-types | | x | x |
| route-pathfind | | x | x |
| cf-spatial | | x | |
| sim-core | x | x | x |
| sim-constraint | x | | x |

---

## Phase 4: Deformable Simulation + Soft Fabrication

**New domains:** sim-deformable, mesh-lattice (advanced), mesh-shell, mesh-slice

**Goal:** Simulate and fabricate soft structures. Validate XPBD deformable simulation against physical silicone/elastomer prototypes.

| Product | Description | Manufacturing |
|---------|-------------|---------------|
| [Pneumatic Finger](./phase-4-deformable/pneumatic-finger/SPEC.md) | Silicone bending actuator with internal air chambers | 3D-printed mold + silicone casting |
| [Variable-Stiffness Pad](./phase-4-deformable/variable-stiffness-pad/SPEC.md) | Pad with pneumatic chambers for tunable stiffness | 3D-printed mold + silicone casting |
| [Peristaltic Crawler](./phase-4-deformable/peristaltic-crawler/SPEC.md) | Worm-like robot with sequential soft segment inflation | 3D-printed mold + silicone casting |

**Crate coverage:**

| Crate | Finger | Pad | Crawler |
|-------|--------|-----|---------|
| sim-deformable | x | x | x |
| mesh-lattice | x | x | |
| mesh-shell | x | x | x |
| mesh-slice | x | x | x |
| mesh-boolean | x | x | x |
| sim-core | x | x | x |
| sensor-types | | x | |
| sim-sensor | | x | |

---

## Phase 5: Muscles + Tendons + ML

**New domains:** sim-muscle, sim-tendon, sim-gpu, ml-types, ml-models, ml-training, ml-dataset

**Goal:** Train learned policies in simulation (GPU-batched), deploy to hardware. Full sim-to-real transfer pipeline.

| Product | Description | Hardware |
|---------|-------------|----------|
| [Tendon-Driven Hand](./phase-5-learning/tendon-driven-hand/SPEC.md) | Three-finger hand with Hill-type muscles, RL grasping policy | Servos + cables + force sensors |
| [Antagonistic Hopping Leg](./phase-5-learning/antagonistic-hopping-leg/SPEC.md) | Single leg with muscle pairs, learned hopping gait | Servos + IMU |
| [Tactile-Adaptive Gripper](./phase-5-learning/tactile-adaptive-gripper/SPEC.md) | Soft gripper with tactile sensing and learned grip strategy | Force sensors + servos |

**Crate coverage:**

| Crate | Hand | Leg | Gripper |
|-------|------|-----|---------|
| sim-muscle | x | x | |
| sim-tendon | x | x | |
| sim-gpu | x | x | x |
| ml-types | x | x | x |
| ml-models | x | x | x |
| ml-training | x | x | x |
| ml-dataset | x | x | x |
| sim-deformable | | | x |
| sensor-types | x | x | x |
| sensor-fusion | x | x | x |

---

## Phase 6: Full Integration Capstone

**New domains:** None. Every domain exercised simultaneously.

**Goal:** Integrate all domains into single artifacts. Each product combines scan-to-custom geometry, deformable simulation, embedded sensing, tendon/muscle actuation, thermal management, and ML-driven adaptive control.

| Product | Description | Manufacturing |
|---------|-------------|---------------|
| [Adaptive Prosthetic Socket](./phase-6-capstone/adaptive-prosthetic-socket/SPEC.md) | Custom-fit socket with pressure sensors, active bladders, ML adaptation | Scan + mold + silicone + electronics |
| [Soft Robotic Massage Device](./phase-6-capstone/soft-robotic-massage-device/SPEC.md) | Body-conforming wearable with pneumatics, force sensing, learned patterns | Scan + mold + silicone + electronics |
| [Bio-Inspired Manipulator](./phase-6-capstone/bio-inspired-manipulator/SPEC.md) | Continuum arm (tentacle/trunk) with distributed sensing and learned reaching | 3D print + silicone + sensors |

**Crate coverage:** All 52+ crates exercised across the three capstone products.

---

## Cumulative Domain Progression

```
                          Phase: 1    2    3    4    5    6
                                 |    |    |    |    |    |
mesh-scan                   ████ ████ ████ ████ ████ ████
mesh-repair                 ████ ████ ████ ████ ████ ████
mesh-boolean                ████ ████ ████ ████ ████ ████
mesh-offset                 ████ ████ ████ ████ ████ ████
mesh-shell                  ████ ████ ████ ████ ████ ████
mesh-lattice                ████ ████ ████ ████ ████ ████
mesh-printability           ████ ████ ████ ████ ████ ████
mesh-io                     ████ ████ ████ ████ ████ ████
curve-types                 ████ ████ ████ ████ ████ ████
mesh-from-curves            ████ ████ ████ ████ ████ ████
mesh-assembly                    ████ ████ ████ ████ ████
sim-types                        ████ ████ ████ ████ ████
sim-core                         ████ ████ ████ ████ ████
sim-constraint                   ████ ████ ████ ████ ████
sim-mjcf / sim-urdf              ████ ████ ████ ████ ████
sensor-types                          ████ ████ ████ ████
sensor-fusion                         ████ ████ ████ ████
sim-sensor                            ████ ████ ████ ████
route-types                           ████ ████ ████ ████
route-pathfind                        ████ ████ ████ ████
cf-spatial                            ████ ████ ████ ████
sim-deformable                             ████ ████ ████
mesh-slice                                 ████ ████ ████
sim-muscle                                      ████ ████
sim-tendon                                      ████ ████
sim-gpu                                         ████ ████
ml-types                                        ████ ████
ml-models                                       ████ ████
ml-training                                     ████ ████
ml-dataset                                      ████ ████
```

---

## Running Examples

Each product is a standalone Cargo binary in the workspace:

```bash
# Run a specific product example
cargo run -p example-custom-shoe

# List all product examples
cargo metadata --format-version 1 | jq '.packages[] | select(.name | startswith("example-")) | .name'
```

---

## Status

| Phase | Products | Status |
|-------|----------|--------|
| Phase 1: Mesh | 3 | Spec |
| Phase 2: Mechanism | 3 | Spec |
| Phase 3: Perception | 3 | Spec |
| Phase 4: Deformable | 3 | Spec |
| Phase 5: Learning | 3 | Spec |
| Phase 6: Capstone | 3 | Spec |
| **Total** | **18** | |

---

*Last updated: 2026-02-12*
