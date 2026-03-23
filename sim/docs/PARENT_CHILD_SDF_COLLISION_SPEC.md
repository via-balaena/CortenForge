# Geometry-Driven Joint Collision Spec

## Goal

Enable SDF-SDF collision between bodies so that joints emerge from
**physical geometry** (pin-in-socket, etc.) rather than abstract
mathematical constraints. The geometry should be something that could
be 3D printed in place and perform similarly.

This is foundational to CortenForge's value: the physics emerges from
the actual printable geometry.

## Architecture: siblings, not parent-child

**Critical finding from stress test:** Free joint FK is world-absolute
by design (MuJoCo convention). Free joints overwrite the parent frame
with raw qpos values (`forward/position.rs:100-114`). A Free joint child
of a non-world body produces undefined behavior.

The correct topology for geometry-driven joints:

```
world → socket (welded, child of world)
world → pin (Free joint, child of world)
```

Both bodies are **siblings** (children of world body 0). Sibling collision
is already enabled by default — only parent-child pairs are filtered.
`DISABLE_FILTERPARENT` is **not needed**.

This is the same topology used in examples 01-08 (stacking). The only
new element is **concave SDF geometry**.

## Infrastructure status

All infrastructure exists and is verified:

| Component | Status | Verified by |
|-----------|--------|-------------|
| `ShapeConcave` → multi-contact fallback | Working | Phase 1-3 tests |
| Inner surface normals (concave) | Correct | Phase 1c, 2c |
| Radial constraint (bore walls) | Working | Phase 2a, 2c, 3a |
| Axial constraint (flanges + caps) | Working | Phase 3b |
| Rotation freedom (bore axis) | Working | Phase 3c |
| Margin-zone guard contacts | Correct | Phase 2b |
| `Solid::subtract()` → concave SDF | Working | All phases |

## Phase results

### Phase 1 — Bowl captures sphere (PASS)

Bowl (sphere shell with top removed) + small sphere inside.
- 1a: 137 contacts detected on inner surface
- 1b: exterior sphere correctly rejected (0 contacts)
- 1c: 575 multi-contacts, net force pushes sphere toward center

### Phase 2 — Cylindrical constraint (PASS)

Tube (cylinder.subtract(cylinder)) + pin cylinder.
- 2a: 1619 contacts, net force strongly radial (-0.997 in offset direction)
- 2b: centered pin has zero penetration contacts (margin-zone guards only)
- 2c: all 4 azimuthal offsets produce correct restorative forces

### Phase 3 — Full hinge (PASS)

Socket with caps + flanged pin. Cell size = 0.3mm.
- 3a: 5362 contacts, net force (-1.000, -0.002, -0.021) — pure radial
- 3b: 7099 contacts (975 penetrating), net force z=-0.995 — strong axial
- 3c: 6305 contacts, **0** with penetration > 0.1 at 45° rotation — free

## Resolution requirements

The flange-to-bore clearance must be >> cell_size * 0.5 (the contact margin):

| cell_size | Contact margin | Min clearance | Quality |
|-----------|---------------|---------------|---------|
| 1.0mm     | 0.5mm         | 1.0mm+        | Marginal |
| 0.5mm     | 0.25mm        | 0.5mm+        | Adequate |
| 0.3mm     | 0.15mm        | 0.3mm+        | Good |

Phase 3 uses 0.3mm cell size with 0.3mm flange-to-bore clearance and
1.0mm flange-to-cap clearance. Zero grid artifacts at 45° rotation.

## Next: Phase 4 — Example 09 visual demo

Replace the abstract `JointKind::Revolute` example with a geometry-driven
hinge. Socket welded at origin, arm with pin on Free joint. Both children
of world. Gravity creates torque, arm swings on the hinge. SDF collision
is the **only** constraint.

## Contact count concern

Phase 3 generates 5000-7000 contacts per pair. In simulation, this means
5000-7000 constraint rows per step. If solver performance is an issue,
contact clustering (keeping only the highest-penetration contact per
spatial cell) may be needed. This is a performance optimization, not a
correctness concern.
