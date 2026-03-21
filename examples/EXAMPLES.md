# CortenForge Examples

## Directory Structure

```
examples/
  fundamentals/       Domain-isolated demos (one domain each)
  integration/        Pipeline composition demos (design → sim → mesh → print)
  sdf-physics/        SDF === collision proof ladder (planned)
```

## Status

### fundamentals/

Each example demonstrates a single CortenForge domain in isolation.

| Example | Domain | Status | Notes |
|---------|--------|--------|-------|
| `hello-solid` | cf-design | Working | Implicit surface basics |
| `bio-shapes` | cf-design | Working | Bio-inspired geometry library |
| `mesh-pipeline` | mesh-* | Working | Mesh repair, lattice, shell, printability |
| `pendulum-sim` | sim-core | Working | Raw MJCF, pure physics (no cf-design) |
| `finger-design` | cf-design + sim | Needs work | Uses `to_model()` but dynamics aren't tuned — animates but motion is too aggressive. Socket/condyle geometry is visual only (parent-child collision filtered). |

### integration/

Pipeline composition: proving domains work together end-to-end.
All three use `Mechanism::to_model()` (SDF collision + visual mesh, no MJCF round-trip).

| Example | Pipeline | Status |
|---------|----------|--------|
| `design-to-sim` | design → simulate → visualize | Working |
| `sim-informed-design` | design → simulate → stress-graded lattice | Working |
| `full-pipeline` | design → simulate → stress → lattice → print | Working |
| `design-to-print` | design → mesh → print | Working |

### sdf-physics/ (planned — not yet created)

Baby-step ladder proving the SDF === collision thesis. Each step has a
clear pass/fail and depends on the previous step working.

| Step | Example | What it proves | Engine status |
|------|---------|---------------|---------------|
| 01 | `01-drop` | Solid falls onto ground plane. SDF vs plane collision. | Should work (sdf_plane_contact exists) |
| 02 | `02-pair` | Two Solids collide. SDF vs SDF, non-parent-child. | Should work (sdf_sdf_contact exists) |
| 03 | `03-hinge` | Parent-child SDF collision. Arm hits base on a hinge. | Blocked — `sdf_sdf_contact` returns 1 contact, needs multi-contact for stability |
| 04 | `04-socket` | Concave constraint. Condyle rotates inside socket. | Blocked on 03 + concave multi-contact |

## Key Architecture: `Mechanism::to_model()`

`cf-design` can build a `sim-core::Model` directly from a `Mechanism`,
bypassing the MJCF XML round-trip. Each part produces two geoms:

- **SDF geom** (collision): `SdfGrid` from the implicit surface — O(1)
  distance queries with exact distance semantics.
- **Mesh geom** (visual): triangle mesh from the same `Solid` — high-res
  rendering in Bevy.

Both derive from the identical mathematical object. What you see is what
physics simulates.

### What works now

- SDF collision for non-parent-child geom pairs (ground planes, obstacles)
- Visual mesh perfectly matches collision boundary (same Solid)
- No MJCF XML round-trip — `Mechanism` → `Model` directly
- Joint limits, tendons, actuators all functional

### What doesn't work yet

- **Parent-child SDF collision** (`DISABLE_FILTERPARENT`): `sdf_sdf_contact`
  generates only 1 contact point. Concave constraints (socket/condyle) need
  multiple simultaneous contacts to prevent penetration from all directions.
  This is the primary blocker for sdf-physics steps 03–04.
- **Gravity at mm scale**: cf-design geometry is in mm, but `Model::empty()`
  sets gravity to 9.81 (m/s²). In a mm-world this is ~1000× too weak.
  Needs either unit-aware gravity or a scaling convention.
