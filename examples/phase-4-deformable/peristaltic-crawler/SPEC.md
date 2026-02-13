# Peristaltic Crawler — Phase 4 Product Spec

Worm-like robot with three alternating soft segments. Sequential inflation produces forward locomotion with zero rigid parts. Exercises deformable simulation for dynamic locomotion (not just static loading), including contact between a deformable body and ground.

## Pipeline

```
Segment dimensions + inflation sequence timing + friction coefficients
    ─► Segment geometry (mesh-shell)
    ─► Mold generation (mesh-boolean)
    ─► Parting line analysis (mesh-slice)
    ─► XPBD dynamic locomotion simulation (sim-deformable)
    ─► Export mold STL (mesh-io)
    ─► Cast silicone + pneumatic tubing ─► Test
```

1. **Segment geometry** — Define the three-segment worm body using mesh-shell to create hollow inflatable segments with controlled wall thickness.
2. **Mold generation** — Generate mold halves using mesh-boolean for cavity subtraction from a mold block.
3. **Parting line analysis** — Use mesh-slice to determine optimal parting lines for demolding the multi-segment body.
4. **XPBD dynamic locomotion simulation** — Simulate the sequential inflation/deflation cycle using sim-deformable. Model deformable-body-to-ground contact with friction. Compute forward displacement over time and generate gait animation data.
5. **Export mold STL** — Write mold halves as STL files for 3D printing.
6. **Cast and test** — 3D-print the mold, cast with silicone, attach pneumatic tubing, and measure physical locomotion.

## CortenForge Crates Used

| Crate | Purpose in this product |
|---|---|
| `sim-types` | Simulation data structures (materials, bodies, contacts) |
| `sim-core` | Simulation engine orchestration and time-stepping |
| `sim-deformable` | XPBD solver for dynamic soft-body locomotion and ground contact |
| `mesh-types` | Core mesh data structures |
| `mesh-shell` | Hollow segment geometry with controlled wall thickness |
| `mesh-slice` | Parting line analysis for multi-segment mold |
| `mesh-boolean` | Boolean subtraction for mold cavity creation |
| `mesh-io` | Export mold halves as STL |

## Input

- Segment dimensions (length, diameter, wall thickness per segment).
- Inflation sequence timing (phase offsets, inflation/deflation durations).
- Friction coefficients (segment-to-ground static and dynamic friction).

## Output

- 3D-printable mold halves exported as STL.
- Locomotion simulation results: distance vs. time curve and gait animation data.

## Manufacturing

- **Mold:** FDM 3D printing (PLA or PETG for rigid mold halves).
- **Body:** Silicone casting into 3D-printed mold (Ecoflex for high elongation).
- **Actuation:** Pneumatic tubing per segment + sequencing valve or syringe array.

## Key Technical Validations

- **`sim-deformable` dynamic contact:** Validates that the XPBD solver handles time-varying inflation loads combined with deformable-to-rigid ground contact and Coulomb friction.
- **`sim-deformable` sequential actuation:** Confirms that phased inflation/deflation of independent segments produces net forward displacement (peristaltic gait).
- **`mesh-boolean` multi-segment mold:** Ensures clean boolean operations on the multi-chamber geometry produce printable mold halves.

## Acceptance Criteria

1. Mold halves are watertight with zero non-manifold edges.
2. XPBD simulation produces net forward locomotion under the peristaltic inflation sequence.
3. Forward locomotion > 1 body length per 10 inflation cycles in both simulation and physical test.
4. Simulated gait pattern qualitatively matches physical observation (alternating segment expansion/contraction).
5. Pipeline completes without panics on reference input parameters.

## Status

**Spec** — not yet implemented.
