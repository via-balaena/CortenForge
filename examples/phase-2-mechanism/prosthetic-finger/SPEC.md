# Prosthetic Finger — Phase 2 Product Spec

Single bio-inspired finger with tendon-routed actuation through printed channels.
Simulate curl trajectory, verify against human finger kinematics. Exercises cable
routing geometry, basic joint chain, and forward kinematics.

## Pipeline

```
Finger dimensions ─► Joint chain definition ─► Tendon routing geometry
    ─► Mesh generation ─► Boolean channels ─► Assembly ─► Simulation ─► Export (STL + MJCF)
```

1. **Joint chain definition** — Define phalanx segment lengths, joint axes, and range
   of motion limits based on human finger anthropometry.
2. **Tendon routing geometry** — Compute tendon channel paths through each phalanx,
   with routing pulleys at joint locations for smooth cable travel.
3. **Mesh generation** — Generate triangle meshes for each phalanx segment from
   parametric curve cross-sections.
4. **Boolean channels** — Subtract tendon channel geometry from phalanx bodies to
   create internal routing paths for the actuation cable.
5. **Assembly** — Assemble phalanx segments into a kinematic chain with revolute
   joints at each interphalangeal joint.
6. **Simulation** — Simulate finger curl trajectory under tendon pull force, computing
   joint angles over time via forward kinematics.
7. **Export** — Write printable finger as STL and simulation model as MJCF for
   MuJoCo verification.

## CortenForge Crates Used

| Crate | Purpose in this product |
|---|---|
| `sim-types` | Joint chain and body state representations |
| `sim-core` | Rigid body dynamics for tendon-driven flexion |
| `sim-constraint` | Revolute joint constraints with range limits |
| `sim-mjcf` | Export finger model to MJCF for MuJoCo verification |
| `curve-types` | Parametric curve definitions for phalanx cross-sections |
| `mesh-from-curves` | Generate phalanx meshes from curve profiles |
| `mesh-boolean` | Subtract tendon channel geometry from phalanx bodies |
| `mesh-assembly` | Assemble multi-body finger with joint definitions |
| `mesh-printability` | Validate wall thickness around tendon channels |
| `mesh-io` | Export final STL and MJCF files |
| `mesh-types` | Core mesh data structures |

## Input

- Finger dimensions: phalanx lengths, joint angles, tendon routing path, cable diameter.

## Output

- Printable finger assembly exported as STL, ready for slicing.
- Simulation model exported as MJCF for MuJoCo verification.
- Simulation results: curl trajectory (joint angles vs. tendon displacement).

## Manufacturing

- **Primary:** FDM 3D printing (PLA or PETG for rigid phalanx segments).
- **Assembly:** Fishing line or thin cable routed through printed channels; pin joints
  at interphalangeal joints.

## Acceptance Criteria

1. Simulated curl trajectory matches human finger kinematics within 10% RMS joint angle error.
2. Full flexion/extension range achieved in simulation without constraint violation.
3. Tendon channels have sufficient clearance for cable routing (min 1.5 mm diameter).
4. Output mesh is watertight and passes printability validation.
5. Exported MJCF loads and simulates in MuJoCo without errors.

## Status

**Spec** — not yet implemented.
