# Four-Bar Walker — Phase 2 Product Spec

Passive walking toy -- Strandbeest-style four-bar linkage, gravity-driven gait.
Design linkage from parametric curves, simulate gait cycle, optimize link lengths for
smooth walking. Exercises multi-body dynamics, closed kinematic chains, and equality
constraints.

## Pipeline

```
Link lengths + joint positions ─► Linkage geometry ─► Mesh generation
    ─► Assembly ─► Gait simulation ─► Link length optimization ─► Export (STL + MJCF)
```

1. **Linkage geometry** — Define four-bar linkage topology with parametric link lengths,
   joint pivot positions, and ground contact points.
2. **Mesh generation** — Generate triangle meshes for each link from parametric curve
   cross-sections with appropriate pin-hole geometry at joints.
3. **Assembly** — Assemble links into a closed kinematic chain with revolute joints and
   equality constraints to enforce the loop closure.
4. **Gait simulation** — Simulate passive walking under gravity on a flat surface,
   computing foot trajectories and gait phase timing.
5. **Link length optimization** — Iterate link lengths to minimize gait asymmetry and
   maximize step length for stable passive walking.
6. **Export** — Write printable linkage assembly as STL and simulation model as MJCF
   for MuJoCo verification.

## CortenForge Crates Used

| Crate | Purpose in this product |
|---|---|
| `sim-types` | Multi-body state and joint representations |
| `sim-core` | Rigid body dynamics for gravity-driven gait simulation |
| `sim-constraint` | Revolute joints + equality constraints for loop closure |
| `sim-mjcf` | Export walker model to MJCF for MuJoCo verification |
| `curve-types` | Parametric curve definitions for link cross-sections |
| `mesh-from-curves` | Generate link meshes from curve profiles |
| `mesh-assembly` | Assemble multi-body linkage with joint definitions |
| `mesh-printability` | Validate joint clearances and wall thickness |
| `mesh-io` | Export final STL and MJCF files |
| `mesh-types` | Core mesh data structures |

## Input

- Link lengths, joint pivot positions, gravity vector.

## Output

- Printable linkage assembly exported as STL, ready for slicing.
- Simulation model exported as MJCF for MuJoCo verification.
- Gait cycle animation data (foot trajectories, joint angles over time).

## Manufacturing

- **Primary:** FDM 3D printing (PLA or PETG for rigid links).
- **Assembly:** Metal pins (e.g., nail or piano wire) inserted through printed pivot holes
  at each joint.

## Acceptance Criteria

1. Walker achieves stable passive walking in simulation for at least 10 gait cycles.
2. Foot trajectory follows a smooth, repeatable path without chaotic divergence.
3. Closed kinematic chain constraints are satisfied within solver tolerance throughout
   the gait cycle (equality constraint residual < 1e-6).
4. Simulated gait matches physical prototype walking behavior within stated tolerances.
5. Output mesh is watertight and passes printability validation.
6. Exported MJCF loads and simulates in MuJoCo without errors.

## Status

**Spec** — not yet implemented.
