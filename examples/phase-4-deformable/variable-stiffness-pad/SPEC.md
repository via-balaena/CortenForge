# Variable Stiffness Pad — Phase 4 Product Spec

Flat pad with embedded pneumatic chambers of varying size and spacing. Inflating different zones changes local stiffness. Exercises XPBD on distributed loading, mesh-lattice for variable-density internal structure, and mesh-slice for mold parting-line analysis.

## Pipeline

```
Pad dimensions + chamber layout + pressure per zone
    ─► Variable-density chamber geometry (mesh-lattice)
    ─► Mold generation (mesh-shell, mesh-boolean)
    ─► Parting line analysis (mesh-slice)
    ─► XPBD distributed-load simulation (sim-deformable)
    ─► Sensor zone mapping (sim-sensor, sensor-types)
    ─► Export mold STL (mesh-io)
    ─► Cast silicone ─► Test
```

1. **Variable-density chamber geometry** — Use mesh-lattice to generate internal chambers with spatially varying size and spacing across the pad, creating distinct stiffness zones.
2. **Mold generation** — Create mold halves using mesh-shell for wall offsets and mesh-boolean for cavity subtraction.
3. **Parting line analysis** — Use mesh-slice to determine optimal parting lines for the multi-chamber mold.
4. **XPBD distributed-load simulation** — Simulate the pad under distributed surface loading at varying zone pressures using sim-deformable. Compute local deflection per zone.
5. **Sensor zone mapping** — Map pressure sensor zones onto the pad surface using sim-sensor and sensor-types to correlate input pressure with measured local stiffness.
6. **Export mold STL** — Write mold halves as STL files for 3D printing.
7. **Cast and test** — 3D-print the mold, cast with silicone, and measure stiffness variation across zones.

## CortenForge Crates Used

| Crate | Purpose in this product |
|---|---|
| `sim-types` | Simulation data structures (materials, bodies) |
| `sim-core` | Simulation engine orchestration |
| `sim-deformable` | XPBD solver for distributed-load deformation |
| `sim-sensor` | Pressure sensor zone simulation |
| `sensor-types` | Sensor data structures for zone mapping |
| `mesh-types` | Core mesh data structures |
| `mesh-lattice` | Variable-density internal chamber generation |
| `mesh-shell` | Mold wall-thickness offset generation |
| `mesh-slice` | Parting line analysis for mold halves |
| `mesh-boolean` | Boolean subtraction for mold cavity creation |
| `mesh-io` | Export mold halves as STL |

## Input

- Pad dimensions (length, width, thickness).
- Chamber layout (zone count, chamber sizes per zone, spacing).
- Pressure per zone (independent pressure input for each zone).

## Output

- 3D-printable mold halves exported as STL.
- Stiffness map: pressure vs. local deflection per zone.

## Manufacturing

- **Mold:** FDM 3D printing (PLA or PETG for rigid mold halves).
- **Pad:** Silicone casting into 3D-printed mold (Ecoflex or Dragon Skin).
- **Actuation:** Independent pneumatic lines per zone.

## Key Technical Validations

- **`mesh-lattice` variable-density API:** Confirms that the lattice generator produces spatially varying chamber densities matching the zone layout specification.
- **`sim-deformable` distributed loading:** Validates that the XPBD solver correctly resolves independent zone pressures and produces per-zone deflection differences.
- **`sim-sensor` zone mapping:** Confirms sensor zone overlay correctly maps pressure inputs to spatial regions on the pad mesh.

## Acceptance Criteria

1. Mold halves are watertight with zero non-manifold edges.
2. Chamber density varies measurably across zones (cell count ratio matches specified layout).
3. XPBD simulation shows distinct deflection per zone under independent pressures.
4. Stiffness variation ratio >= 3:1 between minimum and maximum pressure zones.
5. Pipeline completes without panics on reference input parameters.

## Status

**Spec** — not yet implemented.
