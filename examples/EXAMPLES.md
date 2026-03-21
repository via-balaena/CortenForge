# CortenForge Examples

## Directory Structure

```
examples/
  fundamentals/       Domain-isolated demos (one domain each)
  integration/        Pipeline composition demos (design → sim → mesh → print)
  sdf-physics/        SDF === collision proof ladder (stub implementations)
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

### sdf-physics/ (stubs — implementation is the next milestone)

Baby-step ladder proving the SDF === collision thesis. **One new variable
per step.** Each step has a clear pass/fail and depends on every previous
step working. If a step breaks, you know exactly what broke.

| Step | Example | What it proves | New concept | Status |
|------|---------|---------------|-------------|--------|
| 01 | `01-sdf-grid` | `SdfGrid` from solid sphere at 1.0 mm cells. Accuracy < cell size. | SdfGrid construction | Working |
| 02 | `02-thin-grid` | `SdfGrid` on thin walls (0.6 mm shell at 0.5 mm cells). Void preserved, wall resolved. | Thin-wall grid fidelity | Stub |
| 03 | `03-freefall` | `to_model()` mass/inertia. Free-fall matches `z₀ − ½gt²`. | to_model() + gravity | Stub |
| 04 | `04-rest` | Body settles on ground plane. Near-zero velocity, no penetration. | sdf_plane_contact | Stub |
| 05 | `05-drop` | Drop from height → impact → bounce → settle. No tunneling. | Dynamic contact + restitution | Stub |
| 06 | `06-slide` | Lateral velocity on ground. Friction decelerates to rest. | Friction / tangential forces | Stub |
| 07 | `07-pair` | One SDF body dropped onto another. SDF-vs-SDF, non-parent-child. | sdf_sdf_contact | Stub |
| 08 | `08-stack` | Three cubes stacked on ground. Stable multi-body contact. | Multi-body stacking | Stub |
| 09 | `09-hinge-free` | Revolute joint + SDF bodies, free-swinging. No collision. | Joints with SDF bodies | Stub |
| 10 | `10-hinge-wall` | Hinged arm swings into separate wall body. Non-parent-child. | Articulated external contact | Stub |
| 11 | `11-hinge-stop` | Arm hits parent body's flat stop. `DISABLE_FILTERPARENT`. | Parent-child SDF contact (convex) | Stub |
| 12 | `12-damped-hinge` | Three damping/solref configs on step 11 geometry. Tuning knobs. | Damping + parameter sensitivity | Stub |
| 13 | `13-concave-stop` | Arm tip enters concave pocket on parent. Designed failure point. | Concave parent-child contact | Stub |
| 14 | `14-socket` | Condyle rotates inside concave socket. Multi-contact needed. | Full socket/condyle | Blocked |

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
  Single contact may suffice for step 11 (flat stop surface). Step 13
  (concave half-socket) is a designed failure point that will reveal
  whether multi-contact is needed. Step 14 (full socket) requires it.
- **Gravity at mm scale**: cf-design geometry is in mm, but `Model::empty()`
  sets gravity to 9.81 (m/s²). In a mm-world this is ~1000× too weak.
  Step 03 (freefall) will expose this directly.
