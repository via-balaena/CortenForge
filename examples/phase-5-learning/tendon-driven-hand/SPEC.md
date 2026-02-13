# Tendon-Driven Hand — Phase 5 Product Spec

Three-finger hand with cable tendons and Hill-type muscle models. Train grasping
policy via RL on GPU-batched simulation (sim-gpu), deploy to hardware without
modification. The full sim-to-real integration test.

## Pipeline

```
Hand MJCF model ─► Hill-type muscle setup ─► Tendon routing
    ─► GPU-batched RL training ─► Policy export ─► Hardware deployment
```

1. **Hand model** — Load three-finger hand geometry from MJCF, defining joints,
   link masses, and collision geometry for each finger segment.
2. **Hill-type muscle setup** — Attach Hill-type muscle actuators (sim-muscle) to
   each finger joint, modeling force-length and force-velocity relationships.
3. **Tendon routing** — Route cable tendons (sim-tendon) from muscle actuators
   through guide pulleys to fingertip attachment points, with realistic friction
   and elasticity.
4. **GPU-batched RL training** — Run thousands of parallel grasp episodes on GPU
   (sim-gpu + ml-training), training a policy network to control muscle
   activations for stable grasps across a diverse object set.
5. **Policy export** — Export the trained policy as model weights, ready for
   inference on the target hardware controller.
6. **Hardware deployment** — Deploy the policy to hardware (servos + fishing line
   tendons + force sensors) with zero retraining.

## CortenForge Crates Used

| Crate | Purpose in this product |
|---|---|
| `sim-types` | Body, joint, and actuator state representations |
| `sim-core` | Physics simulation engine |
| `sim-constraint` | Joint limits and contact constraints for finger articulation |
| `sim-muscle` | Hill-type muscle actuator models |
| `sim-tendon` | Cable tendon routing with friction and elasticity |
| `sim-gpu` | GPU-batched parallel simulation for RL training |
| `sim-deformable` | Deformable contact surfaces on fingertips |
| `sim-mjcf` | Load hand model from MJCF format |
| `sensor-types` | Force sensor data types for fingertip feedback |
| `sensor-fusion` | Fuse force sensor readings into grasp state estimate |
| `ml-types` | Policy network and reward signal data types |
| `ml-models` | Policy network architecture definition |
| `ml-training` | RL training loop (PPO or similar) |
| `ml-dataset` | Experience buffer and trajectory storage |
| `mesh-types` | Hand geometry mesh data structures |
| `mesh-io` | Load/export hand geometry meshes |

## Input

- Hand geometry (MJCF model with finger segments, joints, and collision shapes).
- Muscle parameters (peak force, optimal fiber length, tendon slack length per actuator).
- Object set for grasping (sphere, cylinder, cube, irregular shapes, varying mass).

## Output

- Trained grasping policy (model weights file).
- Hardware-deployable binary (same control code, real sensor input).

## Hardware

- **Actuators:** 6 micro servos (2 per finger: flexion + extension).
- **Tendons:** Braided fishing line routed through PTFE guide tubes.
- **Sensors:** FSR (force-sensitive resistor) pads on each fingertip.
- **Controller:** Microcontroller running exported policy at 100 Hz.

## Acceptance Criteria

1. Policy trained 100% in simulation grasps 5 different objects on real hardware
   without retraining.
2. Grasp success rate > 80% across the 5-object test set on hardware.
3. Sim-to-real grip force prediction error < 30%.
4. Training converges within 10M environment steps on GPU.
5. Hardware control loop runs at >= 100 Hz with policy inference.

## Status

**Spec** — not yet implemented.
