# Compliant Gripper — Phase 2 Product Spec

Two-finger compliant gripper -- flexure hinges, no bearings, material compliance only.
Design linkage from parametric curves, simulate grip force vs deflection, verify range
of motion. Print and test.

## Pipeline

```
Gripper parameters ─► Parametric linkage design ─► Mesh generation
    ─► Assembly ─► Rigid body simulation ─► Grip force analysis ─► Export (STL)
```

1. **Parametric linkage design** — Define finger geometry, flexure hinge locations, and
   material thickness using parametric curves and dimension parameters.
2. **Mesh generation** — Generate triangle meshes for each finger segment and the
   gripper base from the parametric curve profiles.
3. **Assembly** — Assemble finger segments into a two-finger gripper with flexure
   hinge joints defined at the compliance points.
4. **Rigid body simulation** — Simulate gripper closure under applied actuation force,
   modeling flexure compliance as joint stiffness constraints.
5. **Grip force analysis** — Extract grip force vs. displacement curve from simulation
   results across the full range of motion.
6. **Export** — Write the printable gripper assembly as STL, ready for slicing.

## CortenForge Crates Used

| Crate | Purpose in this product |
|---|---|
| `sim-types` | Rigid body and joint state representations |
| `sim-core` | Rigid body dynamics engine for grasp simulation |
| `sim-constraint` | Flexure hinge joint constraints with stiffness |
| `sim-urdf` | Export gripper model to URDF for MuJoCo verification |
| `curve-types` | Parametric curve definitions for finger profiles |
| `mesh-from-curves` | Generate finger segment meshes from curve profiles |
| `mesh-assembly` | Assemble multi-body gripper with joint definitions |
| `mesh-printability` | Validate wall thickness and overhang constraints |
| `mesh-io` | Export final STL mesh |
| `mesh-types` | Core mesh data structures |

## Input

- Gripper parameters: finger length, flexure thickness, material stiffness, actuation force range.

## Output

- Printable gripper assembly exported as STL, ready for slicing.
- Simulation results: grip force vs. displacement curve.

## Manufacturing

- **Primary:** FDM 3D printing with flexible filament (TPU) for flexure hinges.
- **Assembly:** Single-print monolithic design; no fasteners or bearings required.

## Acceptance Criteria

1. Simulated grip force is within 20% of physically measured grip force.
2. Full range of motion achieved in simulation without constraint violation.
3. Output mesh is watertight and passes printability validation.
4. Gripper closes around a test object in simulation without interpenetration.
5. Exported STL is loadable by at least one mainstream slicer.

## Status

**Spec** — not yet implemented.
