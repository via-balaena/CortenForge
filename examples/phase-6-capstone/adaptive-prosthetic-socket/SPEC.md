# Adaptive Prosthetic Socket — Phase 6 Capstone Product Spec

Adaptive prosthetic socket custom-molded from a residual limb scan. Embedded
pressure sensors, active pneumatic bladders for pressure redistribution, thermal
management for comfort, and ML-driven fit adaptation over time. High-impact
medical application exercising every single CortenForge domain.

## Pipeline

```
Residual limb scan (PLY/STL)
    ─► Mesh cleanup + anatomical landmarking
    ─► Custom socket geometry (shell + offset)
    ─► Internal pneumatic bladder lattice
    ─► Pressure sensor placement (spatial optimization)
    ─► Tendon-routed valve actuation paths
    ─► Deformable simulation (socket under load)
    ─► ML training (fit adaptation policy)
    ─► Manufacturing export (3MF)
    ─► Hardware deployment
    ─► Continuous adaptation loop
```

1. **Scan ingestion** — Load 3D residual limb scan from PLY or STL via mesh-scan. Register against anatomical atlas using cf-spatial for landmark identification.
2. **Mesh cleanup** — Repair non-manifold edges, fill holes, smooth noise artifacts via mesh-repair. Produce watertight surface suitable for downstream operations.
3. **Socket geometry** — Generate inner socket wall from cleaned scan using mesh-offset. Create outer structural shell via mesh-shell. Boolean-unite socket components with mesh-boolean. Define profile curves for socket brim and trim lines using curve-types and mesh-from-curves.
4. **Pneumatic bladder lattice** — Fill designated pressure zones with a compliant lattice structure via mesh-lattice. Lattice density varies by region (higher compliance at bony prominences, stiffer at load-bearing surfaces).
5. **Pressure sensor placement** — Use cf-spatial KD-tree queries to optimally distribute pressure sensor locations across the inner socket wall. Define sensor specifications via sensor-types.
6. **Tendon routing** — Route pneumatic valve control tendons from bladder zones to external actuators via route-pathfind. Optimize routing paths through the socket wall using route-types.
7. **Deformable simulation** — Simulate socket-limb interaction under walking loads using sim-deformable (XPBD soft body). Model residual limb tissue as viscoelastic body. Apply muscle activation via sim-muscle. Constrain socket-limb interface via sim-constraint. Step physics via sim-core. Read simulated pressure via sim-sensor.
8. **GPU-batched simulation** — Run large ensembles of socket designs under varied loading conditions via sim-gpu for rapid design-space exploration.
9. **MJCF model export** — Export simulation scene as MJCF for cross-validation with MuJoCo reference via sim-mjcf.
10. **ML fit adaptation** — Collect pressure sensor data as ml-dataset. Train fit adaptation policy via ml-training using ml-models. Policy learns to command bladder pressures that minimize peak pressure and maximize contact uniformity.
11. **Sensor fusion** — Fuse pressure + temperature + IMU data via sensor-fusion to estimate limb state (volume changes, gait phase, thermal comfort).
12. **Printability validation** — Validate socket mesh for 3D printing via mesh-printability. Check wall thickness, overhang angles, lattice connectivity.
13. **Slicing** — Generate toolpath slices via mesh-slice for manufacturing preview.
14. **Export** — Write final socket mesh as 3MF via mesh-io.

## CortenForge Crates Used

| Crate | Purpose in this product |
|---|---|
| `mesh-types` | Core mesh data structures for socket geometry |
| `mesh-scan` | Ingest 3D residual limb scan |
| `mesh-repair` | Repair scan artifacts, produce watertight surface |
| `mesh-boolean` | Boolean union of socket shell components |
| `mesh-offset` | Inner wall offset from scan surface |
| `mesh-shell` | Outer structural shell generation |
| `mesh-lattice` | Pneumatic bladder lattice fills |
| `mesh-slice` | Toolpath slicing for manufacturing preview |
| `mesh-printability` | Validate printability constraints |
| `mesh-io` | Read/write PLY, STL, 3MF |
| `mesh-from-curves` | Generate brim and trim-line surfaces from curves |
| `curve-types` | B-spline curves for socket brim profiles |
| `sim-types` | Simulation data structures |
| `sim-core` | Physics simulation stepping |
| `sim-constraint` | Socket-limb interface contact constraints |
| `sim-deformable` | XPBD soft body for residual limb tissue |
| `sim-muscle` | Residual limb muscle activation modeling |
| `sim-tendon` | Tendon-driven valve actuation modeling |
| `sim-gpu` | GPU-batched design-space exploration |
| `sim-sensor` | Simulated pressure and temperature sensors |
| `sim-mjcf` | MJCF export for MuJoCo cross-validation |
| `sensor-types` | Pressure sensor data types |
| `sensor-fusion` | Multi-modal sensor fusion (pressure + thermal + IMU) |
| `ml-types` | ML data types for training pipeline |
| `ml-models` | Neural network architecture for adaptation policy |
| `ml-dataset` | Pressure dataset collection and management |
| `ml-training` | Training loop for fit adaptation policy |
| `route-types` | Tendon routing path representations |
| `route-pathfind` | Optimal tendon routing through socket wall |
| `cf-spatial` | KD-tree sensor placement, anatomical landmarking |

## Input

- 3D residual limb scan as PLY or STL mesh.
- Patient weight, activity level, and socket usage profile.

## Output

- Custom-fit prosthetic socket mesh exported as 3MF, ready for printing.
- Trained ML adaptation policy weights.
- Sensor placement map and tendon routing schematic.

## Manufacturing

- **Primary:** SLS 3D printing (Nylon PA12 for structural shell, TPU for compliant bladder zones).
- **Alternative:** Multi-material FDM (rigid PETG shell + flexible TPU bladder inserts).

## Acceptance Criteria

1. Socket automatically redistributes pressure to maintain comfort over a simulated 1-hour wear test (peak pressure remains below 60 kPa threshold).
2. ML adaptation policy converges within 3 simulated wear sessions — pressure distribution uniformity improves by at least 30% from session 1 to session 3.
3. Output mesh is watertight with all regions passing printability checks.
4. Sensor fusion correctly estimates gait phase from pressure + IMU data with > 90% accuracy.
5. Tendon routing paths are collision-free and satisfy minimum bend-radius constraints.
6. MJCF export loads and runs in MuJoCo without errors.
7. Pipeline completes without panics on a reference limb scan dataset.

## Status

**Spec** — not yet implemented.
