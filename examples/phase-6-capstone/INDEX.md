# Phase 6: Full Integration Capstone

> **Prerequisites:** Phases 1-5 complete (all domains battle-tested across 15 prior products).

## Goal

Pure integration. Every domain combined into single artifacts. No new crates to debug -- just composition. Each product combines scan-to-custom geometry, deformable simulation, embedded sensing, tendon/muscle actuation, and ML-driven adaptive control.

## New Domains Introduced

None. Every existing domain is exercised simultaneously.

## Products

| Product | Description | Manufacturing |
|---------|-------------|---------------|
| [Adaptive Prosthetic Socket](./adaptive-prosthetic-socket/SPEC.md) | Custom-fit socket with pressure sensors, active bladders, ML adaptation | Scan + mold + silicone + electronics |
| [Soft Robotic Massage Device](./soft-robotic-massage-device/SPEC.md) | Body-conforming wearable with pneumatics, force sensing, learned patterns | Scan + mold + silicone + electronics |
| [Bio-Inspired Manipulator](./bio-inspired-manipulator/SPEC.md) | Continuum arm (tentacle/trunk) with distributed sensing and learned reaching | 3D print + silicone + sensors |

## Crate Coverage

All 52+ crates exercised across the three capstone products. Every domain from Phases 1-5 is represented:

- **Mesh pipeline:** mesh-scan, mesh-repair, mesh-boolean, mesh-offset, mesh-shell, mesh-lattice, mesh-slice, mesh-printability, mesh-io, mesh-from-curves, mesh-assembly
- **Simulation:** sim-types, sim-core, sim-constraint, sim-mjcf, sim-urdf, sim-deformable, sim-muscle, sim-tendon, sim-sensor, sim-gpu
- **Perception:** sensor-types, sensor-fusion
- **Routing:** route-types, route-pathfind
- **ML:** ml-types, ml-models, ml-training, ml-dataset
- **Geometry:** curve-types, cf-spatial

---

## Running

```bash
# Adaptive Prosthetic Socket
cargo run -p example-adaptive-prosthetic-socket

# Soft Robotic Massage Device
cargo run -p example-soft-robotic-massage-device

# Bio-Inspired Manipulator
cargo run -p example-bio-inspired-manipulator
```

---

## Completion Criteria

- [ ] Each capstone product exercises domains from every prior phase in a single pipeline
- [ ] Adaptive prosthetic socket: scan-to-fit pipeline produces patient-specific geometry, active bladders respond to ML-driven pressure redistribution
- [ ] Soft robotic massage device: body-conforming surface adapts pneumatic patterns via learned policy, force sensing prevents over-pressure
- [ ] Bio-inspired manipulator: continuum arm reaches target poses using distributed sensing and RL policy trained in GPU-batched sim
- [ ] No new crate bugs discovered -- all failures are integration-level, not domain-level
- [ ] Full sim-to-real pipeline executes end-to-end for at least one capstone product
- [ ] All 52+ crates compile and pass tests when exercised together

---

## Shared Learnings

Things we expect to discover and codify during Phase 6:

- Cross-domain pipeline composition overhead: where serialization boundaries between mesh, sim, ML, and sensor crates create bottlenecks
- Scan-to-sim-to-real end-to-end latency budget and which pipeline stages dominate wall-clock time
- Integration failure modes that individual domain tests cannot catch (e.g., mesh-to-tet quality degrading ML training stability)
- Patient/user-specific adaptation: how much per-individual scan data is needed before learned policies generalize
- Multi-physics coupling stability when rigid body, deformable, muscle, and contact solvers all run in the same timestep
- Minimum viable sensor suite for closed-loop adaptive control in each capstone product (which sensors can be dropped without policy degradation)

---

*See also: [Product Roadmap](../PRODUCT_ROADMAP.md) | [Phase 5: Muscles + Tendons + ML](../phase-5-learning/INDEX.md)*
