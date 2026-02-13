# Phase 5: Muscles + Tendons + ML

> **Prerequisites:** Phases 1-4 complete (mesh pipeline, rigid-body simulation, perception + control loop, deformable simulation all validated).

## Goal

Train learned policies in GPU-batched simulation, deploy to hardware without modification. Full sim-to-real transfer pipeline.

## New Domains Introduced

| Crate | Purpose |
|-------|---------|
| `sim-muscle` | Hill-type muscle models |
| `sim-tendon` | Cable routing and tendon dynamics |
| `sim-gpu` | GPU-batched RL training environments |
| `ml-types` | Core ML data types and interfaces |
| `ml-models` | Policy and value network architectures |
| `ml-training` | Training loops, PPO/SAC, curriculum |
| `ml-dataset` | Replay buffers, dataset management |

## Products

| Product | Description | Hardware |
|---------|-------------|----------|
| [Tendon-Driven Hand](./tendon-driven-hand/SPEC.md) | Three-finger hand with Hill-type muscles, RL grasping policy | Servos + cables + force sensors |
| [Antagonistic Hopping Leg](./antagonistic-hopping-leg/SPEC.md) | Single leg with muscle pairs, learned hopping gait | Servos + IMU |
| [Tactile-Adaptive Gripper](./tactile-adaptive-gripper/SPEC.md) | Soft gripper with tactile sensing and learned grip strategy | Force sensors + servos |

## Crate Coverage

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

## Running

```bash
# Tendon-Driven Hand
cargo run -p example-tendon-driven-hand

# Antagonistic Hopping Leg
cargo run -p example-antagonistic-hopping-leg

# Tactile-Adaptive Gripper
cargo run -p example-tactile-adaptive-gripper
```

---

## Completion Criteria

- [ ] Hill-type muscle model produces physiologically plausible force-length-velocity curves
- [ ] Tendon routing simulation matches cable tension measurements on physical prototype
- [ ] GPU-batched environment runs at least 4096 parallel instances
- [ ] RL policy for tendon-driven hand achieves stable grasp on 5+ object geometries in simulation
- [ ] Antagonistic hopping leg learns periodic gait with energy-efficient muscle coordination
- [ ] Tactile-adaptive gripper modulates grip force in response to slip detection
- [ ] Trained policies transfer to physical hardware without policy modification (sim-to-real gap < 20%)
- [ ] Training pipeline reproduces from checkpoint: deterministic seeds, logged hyperparameters

---

## Shared Learnings

Things we expect to discover and codify during Phase 5:

- Hill-type muscle parameter ranges that produce stable training without reward hacking or degenerate gaits
- Tendon routing topology constraints that prevent cable self-intersection in simulation and on hardware
- GPU-batched environment scaling behavior: memory per instance, minimum batch size for stable PPO updates
- Sim-to-real gap sources ranked by impact: actuator deadband, sensor latency, contact model fidelity, muscle damping
- Reward shaping strategies that transfer across morphologies (hand grasp vs. leg hop vs. gripper squeeze)
- Deterministic training reproducibility requirements: seed management, floating-point ordering, checkpoint round-trip fidelity

---

*See also: [Product Roadmap](../PRODUCT_ROADMAP.md) | [Phase 4: Deformable](../phase-4-deformable/INDEX.md) | [Phase 6: Full Integration](../phase-6-capstone/INDEX.md)*
