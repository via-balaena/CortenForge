# Tactile Adaptive Gripper — Phase 5 Product Spec

Two-finger soft gripper with embedded force/tactile sensors. ML model learns to
classify object properties (soft/hard, round/flat, heavy/light) from tactile
feedback during grasp, then adapts grip strategy in real-time. Full ML pipeline:
synthetic tactile data in sim -> classifier training -> real-time inference.

## Pipeline

```
Object set ─► Simulated grasps (sim-gpu) ─► Synthetic tactile dataset
    ─► Classifier training (ml-training) ─► Adaptive grip policy ─► Hardware deployment
```

1. **Object set definition** — Define a diverse set of objects with varying
   properties: stiffness (soft/hard), shape (round/flat), and mass (heavy/light).
2. **Simulated grasps** — Run GPU-batched grasp simulations (sim-gpu) with the
   soft gripper (sim-deformable), recording tactile sensor readings (sim-sensor)
   at each fingertip contact point.
3. **Synthetic tactile dataset** — Collect labeled tactile time-series data from
   simulation into a structured dataset (ml-dataset) for supervised learning.
4. **Classifier training** — Train a tactile classifier (ml-models + ml-training)
   to predict object properties from raw tactile sensor streams.
5. **Adaptive grip policy** — Combine classifier output with a grip strategy
   selector: soft objects get gentle grasps, heavy objects get firm grasps,
   round objects get enveloping grasps.
6. **Hardware deployment** — Deploy classifier + adaptive policy to hardware with
   real FSR sensors, running inference in real-time.

## CortenForge Crates Used

| Crate | Purpose in this product |
|---|---|
| `sim-types` | Body and contact state representations |
| `sim-core` | Physics simulation engine |
| `sim-deformable` | Soft finger deformation during contact |
| `sim-gpu` | GPU-batched parallel simulation for dataset generation |
| `sim-sensor` | Simulated force/tactile sensors on fingertips |
| `sensor-types` | Force sensor data types and tactile array layout |
| `sensor-fusion` | Fuse multi-point tactile readings into contact features |
| `ml-types` | Classifier input/output data types |
| `ml-models` | Tactile classifier network architecture |
| `ml-training` | Supervised training loop for classifier |
| `ml-dataset` | Labeled tactile dataset storage and batching |
| `mesh-types` | Gripper and object geometry data structures |
| `mesh-io` | Load/export gripper and object meshes |

## Input

- Object set with varying properties (stiffness, shape, mass).
- Tactile sensor layout (FSR positions on each fingertip).

## Output

- Trained tactile classifier (model weights file).
- Adaptive grip policy (classifier + strategy selector).

## Hardware

- **Gripper:** Two-finger soft gripper (from Phase 4 soft-body example).
- **Sensors:** 4 FSR (force-sensitive resistor) pads per fingertip (8 total).
- **Controller:** Microcontroller running classifier inference at 50 Hz.
- **Actuator:** Single servo per finger for open/close with variable force.

## Acceptance Criteria

1. Object classification accuracy > 85% on real objects using a model trained
   on synthetic data only.
2. Classifier correctly distinguishes all 3 property axes (stiffness, shape,
   mass) independently.
3. Adaptive grip strategy reduces object damage rate by > 50% compared to
   fixed-force grasping on soft objects.
4. Real-time inference runs at >= 50 Hz on the target microcontroller.
5. Synthetic-to-real tactile signal correlation > 0.7 (sensor readings in sim
   qualitatively match hardware readings for the same object).

## Status

**Spec** — not yet implemented.
