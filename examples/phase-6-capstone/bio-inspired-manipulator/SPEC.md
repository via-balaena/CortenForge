# Bio-Inspired Manipulator — Phase 6 Capstone Product Spec

Multi-segment continuum arm inspired by octopus tentacle and elephant trunk
biomechanics. Scan-derived custom mounting, XPBD deformable body simulation,
tendon-driven shape control, distributed tactile sensing along the full length,
and ML policy for reaching and grasping in cluttered environments. Pure robotics
research framing — publishable at top venues (RSS, CoRL, ICRA).

## Pipeline

```
Base mount scan (PLY/STL)
    ─► Mesh cleanup + mounting interface design
    ─► Continuum arm segment geometry (lattice + shell)
    ─► Tendon routing through segments
    ─► Distributed tactile sensor placement
    ─► XPBD simulation (whole-arm deformation)
    ─► GPU-batched RL training (reaching + grasping)
    ─► MJCF export for cross-validation
    ─► Manufacturing export (3MF)
    ─► Hardware deployment
    ─► Sim-to-real transfer
```

1. **Mount scan ingestion** — Load 3D scan of robot base or mounting surface from PLY or STL via mesh-scan. Register against workspace coordinate frame using cf-spatial.
2. **Mesh cleanup** — Repair non-manifold edges, fill holes, smooth noise via mesh-repair. Produce watertight surface for mounting interface design.
3. **Mounting interface** — Generate mounting flange and adapter geometry from cleaned scan using mesh-offset and mesh-shell. Boolean-unite mount components via mesh-boolean. Define flange profile curves using curve-types and mesh-from-curves.
4. **Continuum arm geometry** — Design multi-segment continuum arm as a series of compliant lattice sections via mesh-lattice. Each segment has a deformable lattice core surrounded by a thin structural shell via mesh-shell. Segment geometry tapers from base to tip (trunk/tentacle biomimicry).
5. **Tendon routing** — Route actuation tendons through each segment via route-pathfind. Each segment has 3+ tendons for omnidirectional bending. Optimize routing paths for minimum friction and maximum control authority using route-types.
6. **Tactile sensor placement** — Use cf-spatial to distribute tactile sensors along the arm surface with higher density at the distal tip (grasping zone). Define sensor specifications (normal force, shear force, proximity) via sensor-types.
7. **XPBD simulation** — Simulate whole-arm deformation using sim-deformable (XPBD soft body). Model each segment as a deformable volume with position-based constraints. Apply tendon forces via sim-tendon. Model antagonistic muscle pairs for stiffness control via sim-muscle. Enforce segment connectivity via sim-constraint. Step physics via sim-core. Read tactile feedback via sim-sensor.
8. **GPU-batched RL training** — Run thousands of parallel simulation instances via sim-gpu. Train reaching and grasping policy via ml-training using ml-models. Collect trajectory and reward data as ml-dataset. Policy learns tendon activation patterns for target reaching in cluttered environments and compliant grasping of varied objects.
9. **MJCF model export** — Export simulation scene as MJCF via sim-mjcf for cross-validation with MuJoCo reference and comparison with published baselines.
10. **Sensor fusion** — Fuse tactile + proprioceptive (tendon tension) + IMU data via sensor-fusion to estimate arm configuration and contact state.
11. **Printability validation** — Validate arm segment meshes for 3D printing via mesh-printability. Check lattice connectivity, wall thickness, overhang angles.
12. **Slicing** — Generate toolpath slices via mesh-slice for manufacturing preview.
13. **Export** — Write final arm and mount meshes as 3MF via mesh-io.

## CortenForge Crates Used

| Crate | Purpose in this product |
|---|---|
| `mesh-types` | Core mesh data structures for arm geometry |
| `mesh-scan` | Ingest 3D base mount scan |
| `mesh-repair` | Repair scan artifacts, produce watertight surface |
| `mesh-boolean` | Boolean union of mount and segment components |
| `mesh-offset` | Mounting interface offset geometry |
| `mesh-shell` | Structural shell for arm segments and mount |
| `mesh-lattice` | Compliant lattice core for each arm segment |
| `mesh-slice` | Toolpath slicing for manufacturing preview |
| `mesh-printability` | Validate printability constraints |
| `mesh-io` | Read/write PLY, STL, 3MF |
| `mesh-from-curves` | Generate flange and segment profile surfaces |
| `curve-types` | B-spline curves for segment cross-section profiles |
| `sim-types` | Simulation data structures |
| `sim-core` | Physics simulation stepping |
| `sim-constraint` | Segment connectivity and joint constraints |
| `sim-deformable` | XPBD soft body for continuum arm deformation |
| `sim-muscle` | Antagonistic muscle pairs for stiffness control |
| `sim-tendon` | Tendon-driven bending actuation |
| `sim-gpu` | GPU-batched parallel RL training environments |
| `sim-sensor` | Simulated tactile and proprioceptive sensors |
| `sim-mjcf` | MJCF export for MuJoCo cross-validation |
| `sensor-types` | Tactile sensor data types (normal, shear, proximity) |
| `sensor-fusion` | Multi-modal fusion (tactile + proprioceptive + IMU) |
| `ml-types` | ML data types for RL training pipeline |
| `ml-models` | Policy and value network architectures |
| `ml-dataset` | Trajectory and reward dataset collection |
| `ml-training` | RL training loop (PPO or SAC) |
| `route-types` | Tendon routing path representations |
| `route-pathfind` | Optimal tendon routing through arm segments |
| `cf-spatial` | KD-tree sensor placement, workspace registration |

## Input

- 3D scan of robot base or mounting surface as PLY or STL mesh.
- Target workspace dimensions and clutter environment specification.

## Output

- Multi-segment continuum arm mesh exported as 3MF, ready for printing.
- Trained RL reaching and grasping policy weights.
- Tactile sensor placement map and tendon routing schematic.

## Manufacturing

- **Primary:** Multi-material FDM (rigid PETG structural elements + flexible TPU lattice core + conductive TPU sensor traces).
- **Alternative:** SLS 3D printing (Nylon PA12 shell) + cast silicone deformable segments.

## Acceptance Criteria

1. Arm reaches target positions in cluttered environment with > 75% success rate in simulation across 100 randomized trials.
2. RL policy trained in sim transfers to hardware with < 20% performance degradation (sim-to-real gap).
3. Whole-arm XPBD simulation runs at > 100 Hz for real-time control.
4. Tendon routing achieves omnidirectional bending (> 90 degrees in any direction) for each segment.
5. Distributed tactile sensing detects contact location within 5mm accuracy along the arm length.
6. Sensor fusion correctly estimates arm configuration from proprioceptive data with < 3 degree joint angle error.
7. Output meshes are watertight with all segments passing printability checks.
8. MJCF export loads and runs in MuJoCo without errors.
9. Pipeline completes without panics on a reference mount scan dataset.

## Status

**Spec** — not yet implemented.
