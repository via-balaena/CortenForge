# Soft Robotic Massage Device — Phase 6 Capstone Product Spec

Body-conforming wearable soft robotic massage system built scan-to-fit. Pneumatic
actuation for deep tissue manipulation, distributed force sensors for pressure
control, tendon-driven shape adaptation, and ML-learned massage patterns that
adapt to user feedback. Wellness and medical rehabilitation framing exercising
every single CortenForge domain.

## Pipeline

```
Body region scan (PLY/STL)
    ─► Mesh cleanup + anatomical region segmentation
    ─► Custom-fit wearable shell
    ─► Pneumatic massage chamber design (lattice)
    ─► Tendon routing for shape adaptation
    ─► Force sensor layout (spatial optimization)
    ─► Deformable simulation (tissue interaction)
    ─► ML training (massage patterns from user feedback)
    ─► Manufacturing export (3MF)
    ─► Hardware deployment
    ─► Continuous pattern adaptation
```

1. **Scan ingestion** — Load 3D body region scan (shoulder, back, calf) from PLY or STL via mesh-scan. Register against anatomical atlas using cf-spatial for muscle group identification.
2. **Mesh cleanup** — Repair non-manifold edges, fill holes, smooth noise via mesh-repair. Produce watertight surface for downstream operations.
3. **Wearable shell geometry** — Generate inner contact surface from cleaned scan using mesh-offset. Create outer structural shell via mesh-shell. Boolean-unite shell components via mesh-boolean. Define edge profiles and closure curves using curve-types and mesh-from-curves.
4. **Pneumatic chamber design** — Design pneumatic massage chambers as a lattice structure within the shell via mesh-lattice. Chamber geometry varies by muscle group (large chambers for broad strokes, small chambers for trigger-point work).
5. **Tendon routing** — Route shape-adaptation tendons through the device shell via route-pathfind. Tendons pull the device into tighter contact for deep-tissue work or release for lighter massage. Optimize paths to avoid chamber interference using route-types.
6. **Force sensor layout** — Use cf-spatial to optimally distribute force sensors across the inner contact surface. Define sensor specifications (range, sensitivity) via sensor-types.
7. **Deformable simulation** — Simulate device-tissue interaction using sim-deformable (XPBD soft body). Model tissue as layered viscoelastic body (skin, fascia, muscle). Apply pneumatic actuation as muscle activation via sim-muscle. Model tendon-driven shape changes via sim-tendon. Constrain device-body interface via sim-constraint. Step physics via sim-core. Read simulated force and pressure via sim-sensor.
8. **GPU-batched simulation** — Run large ensembles of massage patterns under varied body morphologies via sim-gpu for rapid pattern optimization.
9. **MJCF model export** — Export simulation scene as MJCF for cross-validation with MuJoCo reference via sim-mjcf.
10. **ML massage pattern learning** — Collect force sensor data paired with user comfort ratings as ml-dataset. Train massage pattern policy via ml-training using ml-models. Policy learns pneumatic actuation sequences that maximize user comfort rating for each body region and user preference.
11. **Sensor fusion** — Fuse force + IMU data via sensor-fusion to estimate tissue state (muscle tension, user posture, device fit quality).
12. **Printability validation** — Validate device mesh for 3D printing via mesh-printability. Check wall thickness, chamber connectivity, overhang angles.
13. **Slicing** — Generate toolpath slices via mesh-slice for manufacturing preview.
14. **Export** — Write final device mesh as 3MF via mesh-io.

## CortenForge Crates Used

| Crate | Purpose in this product |
|---|---|
| `mesh-types` | Core mesh data structures for device geometry |
| `mesh-scan` | Ingest 3D body region scan |
| `mesh-repair` | Repair scan artifacts, produce watertight surface |
| `mesh-boolean` | Boolean union of shell components |
| `mesh-offset` | Inner contact surface offset from scan |
| `mesh-shell` | Outer structural shell generation |
| `mesh-lattice` | Pneumatic massage chamber lattice design |
| `mesh-slice` | Toolpath slicing for manufacturing preview |
| `mesh-printability` | Validate printability constraints |
| `mesh-io` | Read/write PLY, STL, 3MF |
| `mesh-from-curves` | Generate edge profiles and closure surfaces from curves |
| `curve-types` | B-spline curves for edge and closure profiles |
| `sim-types` | Simulation data structures |
| `sim-core` | Physics simulation stepping |
| `sim-constraint` | Device-body interface contact constraints |
| `sim-deformable` | XPBD soft body for tissue modeling |
| `sim-muscle` | Pneumatic actuation modeled as muscle activation |
| `sim-tendon` | Tendon-driven shape adaptation modeling |
| `sim-gpu` | GPU-batched massage pattern optimization |
| `sim-sensor` | Simulated force and pressure sensors |
| `sim-mjcf` | MJCF export for MuJoCo cross-validation |
| `sensor-types` | Force sensor data types |
| `sensor-fusion` | Multi-modal sensor fusion (force + IMU) |
| `ml-types` | ML data types for training pipeline |
| `ml-models` | Neural network architecture for massage pattern policy |
| `ml-dataset` | Force + comfort rating dataset collection |
| `ml-training` | Training loop for massage pattern policy |
| `route-types` | Tendon routing path representations |
| `route-pathfind` | Optimal tendon routing through device shell |
| `cf-spatial` | KD-tree sensor placement, anatomical registration |

## Input

- 3D body region scan as PLY or STL mesh.
- User preferences (pressure intensity, target muscle groups, sensitivity areas).

## Output

- Custom-fit massage device mesh exported as 3MF, ready for printing.
- Trained ML massage pattern policy weights.
- Sensor placement map and tendon routing schematic.

## Manufacturing

- **Primary:** Multi-material FDM (rigid PETG structural shell + flexible TPU pneumatic chambers).
- **Alternative:** SLS 3D printing (Nylon PA12 shell) + cast silicone pneumatic chambers.

## Acceptance Criteria

1. Device adapts massage pattern based on real-time force feedback — detected force anomalies (> 2x target pressure) trigger automatic pressure reduction within 200ms.
2. ML-learned massage patterns achieve user comfort rating > 4/5 averaged across a panel of 5 simulated user profiles.
3. Tendon-driven shape adaptation achieves at least 15% increase in contact area under deep-tissue mode vs. relaxed mode.
4. Output mesh is watertight with all regions passing printability checks.
5. Sensor fusion correctly estimates muscle tension state from force data with > 85% accuracy.
6. Tendon routing paths are collision-free and do not interfere with pneumatic chambers.
7. MJCF export loads and runs in MuJoCo without errors.
8. Pipeline completes without panics on a reference body scan dataset.

## Status

**Spec** — not yet implemented.
