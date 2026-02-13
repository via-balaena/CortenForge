# Antagonistic Hopping Leg — Phase 5 Product Spec

Single leg (hip + knee + ankle) with antagonistic muscle pairs. Train a hopping
gait via RL in GPU-batched simulation. Exercises muscle dynamics (co-contraction,
force-velocity curves), tendon elasticity, and RL on cyclical locomotion.

## Pipeline

```
Leg MJCF model ─► Antagonistic muscle pairs ─► Tendon elasticity setup
    ─► GPU-batched RL training ─► Policy export ─► Hardware deployment
```

1. **Leg model** — Load single-leg geometry from MJCF with hip, knee, and ankle
   revolute joints and appropriate link masses/inertias.
2. **Antagonistic muscle pairs** — Attach flexor/extensor Hill-type muscle pairs
   (sim-muscle) to each joint, modeling co-contraction, force-length, and
   force-velocity relationships.
3. **Tendon elasticity** — Configure series elastic tendons (sim-tendon) on each
   muscle-tendon unit, modeling energy storage and return during hopping.
4. **GPU-batched RL training** — Train a hopping policy via RL (sim-gpu +
   ml-training) across thousands of parallel environments, optimizing for stable
   periodic hopping at a target height.
5. **Policy export** — Export trained policy weights for hardware inference.
6. **Hardware deployment** — Deploy policy to physical leg (3 servos + IMU +
   springs for tendon compliance).

## CortenForge Crates Used

| Crate | Purpose in this product |
|---|---|
| `sim-types` | Body, joint, and muscle state representations |
| `sim-core` | Physics simulation engine with ground contact |
| `sim-constraint` | Joint limits and ground contact constraints |
| `sim-muscle` | Hill-type antagonistic muscle pair models |
| `sim-tendon` | Series elastic tendon dynamics for energy storage |
| `sim-gpu` | GPU-batched parallel simulation for RL training |
| `sim-mjcf` | Load leg model from MJCF format |
| `sim-sensor` | Simulated IMU for orientation and acceleration |
| `sensor-types` | IMU data types (accelerometer, gyroscope) |
| `sensor-fusion` | Fuse IMU readings into body state estimate |
| `ml-types` | Policy network and reward signal data types |
| `ml-models` | Policy network architecture definition |
| `ml-training` | RL training loop for locomotion policy |
| `ml-dataset` | Experience buffer and trajectory storage |
| `mesh-types` | Leg geometry mesh data structures |
| `mesh-io` | Load/export leg geometry meshes |

## Input

- Leg geometry (MJCF model with hip, knee, ankle joints).
- Muscle attachment points and Hill-type parameters per muscle.
- Target hop height (e.g., 10 cm).

## Output

- Trained hopping policy (model weights file).
- Hardware-deployable binary (same control code, real IMU input).

## Hardware

- **Actuators:** 3 servos (hip, knee, ankle).
- **Sensing:** 6-axis IMU mounted on the thigh link.
- **Compliance:** Compression springs in series with each servo for tendon-like
  energy storage and return.
- **Controller:** Microcontroller running exported policy at 200 Hz.

## Acceptance Criteria

1. Stable hopping for 30+ consecutive cycles in simulation.
2. Hop height within 25% of simulated prediction on hardware.
3. Co-contraction dynamics visible in muscle activation profiles (antagonist
   pairs co-activate during stance phase).
4. Training converges within 5M environment steps on GPU.
5. Energy-efficient gait: tendon elastic recoil contributes > 20% of positive
   work per cycle.

## Status

**Spec** — not yet implemented.
