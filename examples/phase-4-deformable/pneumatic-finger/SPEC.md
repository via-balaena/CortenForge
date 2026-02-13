# Pneumatic Finger — Phase 4 Product Spec

Single silicone bending actuator with internal air chambers. Design chamber geometry with mesh-lattice, simulate inflation/deflection with XPBD (sim-deformable), cast from 3D-printed mold. The standard soft robotics benchmark.

## Pipeline

```
Chamber dimensions + wall thickness + silicone material properties
    ─► Chamber geometry design (mesh-lattice)
    ─► Mold generation (mesh-shell, mesh-boolean)
    ─► Parting line analysis (mesh-slice)
    ─► XPBD inflation/deflection simulation (sim-deformable)
    ─► Export mold STL (mesh-io)
    ─► Cast silicone ─► Test
```

1. **Chamber geometry design** — Define internal air chamber geometry using mesh-lattice to create repeating chamber cells within the finger body envelope.
2. **Mold generation** — Generate mold halves from the finger exterior using mesh-shell for wall offsets and mesh-boolean for cavity subtraction.
3. **Parting line analysis** — Use mesh-slice to determine optimal mold parting lines for clean demolding.
4. **XPBD simulation** — Simulate inflation of the chambers at varying pressures using sim-deformable XPBD solver. Compute tip deflection angle as a function of input pressure.
5. **Export mold STL** — Write mold halves as STL files ready for 3D printing.
6. **Cast and test** — 3D-print the mold, cast with Dragon Skin silicone, and measure physical deflection for validation.

## CortenForge Crates Used

| Crate | Purpose in this product |
|---|---|
| `sim-types` | Simulation data structures (materials, bodies) |
| `sim-core` | Simulation engine orchestration |
| `sim-deformable` | XPBD solver for soft-body inflation/deflection |
| `mesh-types` | Core mesh data structures |
| `mesh-lattice` | Internal air chamber geometry generation |
| `mesh-shell` | Mold wall-thickness offset generation |
| `mesh-slice` | Parting line analysis for mold halves |
| `mesh-boolean` | Boolean subtraction for mold cavity creation |
| `mesh-io` | Export mold halves as STL |
| `mesh-printability` | Validate mold printability constraints |

## Input

- Chamber dimensions (length, width, number of chambers, chamber spacing).
- Wall thickness for silicone body and mold.
- Silicone material properties (Young's modulus, Poisson's ratio, density).

## Output

- 3D-printable mold halves exported as STL.
- Simulated deflection curve: pressure vs. tip angle.

## Manufacturing

- **Mold:** FDM 3D printing (PLA or PETG for rigid mold halves).
- **Actuator:** Dragon Skin silicone casting into 3D-printed mold.
- **Actuation:** Pneumatic tubing + syringe or pressure regulator.

## Key Technical Validations

- **`mesh-lattice` chamber generation:** Confirms that the lattice generator can produce repeating internal void geometry suitable for pneumatic actuation.
- **`sim-deformable` XPBD inflation:** Validates that the XPBD solver correctly models pressure-driven deformation of a hyperelastic silicone body.
- **`mesh-boolean` mold subtraction:** Ensures clean boolean difference between mold block and finger exterior produces watertight mold cavities.

## Acceptance Criteria

1. Mold halves are watertight with zero non-manifold edges.
2. Mold halves pass printability checks (wall thickness >= 1.0 mm, no unsupported overhangs > 45 deg).
3. XPBD simulation produces monotonically increasing tip deflection with increasing pressure.
4. Simulated deflection is within 15% of measured physical deflection on the cast actuator.
5. Pipeline completes without panics on reference input parameters.

## Status

**Spec** — not yet implemented.
