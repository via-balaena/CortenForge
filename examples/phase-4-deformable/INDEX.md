# Phase 4: Deformable Simulation + Soft Fabrication

> **Prerequisites:** Phases 1-3 complete (mesh pipeline, rigid-body simulation, perception + control loop all validated).

## Goal

Simulate and fabricate soft structures. Validate XPBD deformable simulation against physical silicone/elastomer prototypes. Simulated deflection within 15% of measured.

## New Domains Introduced

| Crate | Purpose |
|-------|---------|
| `sim-deformable` | XPBD soft body simulation |
| `mesh-lattice` | Advanced internal channels and lattice structures |
| `mesh-shell` | Mold design and wall generation |
| `mesh-slice` | Cross-section analysis and parting line detection |

## Products

| Product | Description | Manufacturing |
|---------|-------------|---------------|
| [Pneumatic Finger](./pneumatic-finger/SPEC.md) | Silicone bending actuator with internal air chambers | 3D-printed mold + silicone casting |
| [Variable-Stiffness Pad](./variable-stiffness-pad/SPEC.md) | Pad with pneumatic chambers for tunable stiffness | 3D-printed mold + silicone casting |
| [Peristaltic Crawler](./peristaltic-crawler/SPEC.md) | Worm-like robot with sequential soft segment inflation | 3D-printed mold + silicone casting |

**Manufacturing method:** 3D-printed molds + silicone casting for all three products.

## Crate Coverage

| Crate | Finger | Pad | Crawler |
|-------|--------|-----|---------|
| sim-deformable | x | x | x |
| mesh-lattice | x | x | |
| mesh-shell | x | x | x |
| mesh-slice | x | x | x |
| mesh-boolean | x | x | x |
| sim-core | x | x | x |
| sensor-types | | x | |
| sim-sensor | | x | |

---

## Running

```bash
# Pneumatic Finger
cargo run -p example-pneumatic-finger

# Variable-Stiffness Pad
cargo run -p example-variable-stiffness-pad

# Peristaltic Crawler
cargo run -p example-peristaltic-crawler
```

---

## Completion Criteria

- [ ] XPBD simulation converges for all three product geometries
- [ ] Simulated deflection of pneumatic finger matches physical prototype within 15% error
- [ ] Variable-stiffness pad simulation predicts correct stiffness gradient across pressure range
- [ ] Peristaltic crawler achieves forward locomotion in simulation with sequential inflation
- [ ] Mold geometry exports as watertight, printable STL for all three products
- [ ] Parting line analysis (mesh-slice) produces valid two-part molds
- [ ] Physical silicone castings match simulated deformation profiles

---

## Shared Learnings

Things we expect to discover and codify during Phase 4:

- XPBD iteration counts and substep strategies needed for stable soft body convergence across different silicone shore hardnesses
- Internal chamber geometry constraints that are both simulatable (clean tet mesh) and moldable (no undercuts, uniform wall thickness)
- Parting line placement heuristics for two-part molds of complex pneumatic geometries
- Material parameter fitting workflow: mapping physical silicone tensile tests to XPBD constraint stiffness values
- Coupling boundary between rigid body sim (sim-core) and deformable sim (sim-deformable) at attachment points
- Mold draft angle and shrinkage compensation values that produce dimensionally accurate silicone castings from FDM molds

---

*See also: [Product Roadmap](../PRODUCT_ROADMAP.md) | [Phase 3: Perception](../phase-3-perception/INDEX.md) | [Phase 5: Muscles + Tendons + ML](../phase-5-learning/INDEX.md)*
