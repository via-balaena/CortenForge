# Example 09 — Rewrite: Door Hinge

## Decision

The spinning-pin-in-a-box mechanism was rejected — it looks like magic.
You can't see the constraint happening. Replacing with a **door hinge**:
the most universally understood geometry-driven joint.

## What to build (2 parts)

**Frame** (fixed, heavy, gravcomp):
- Flat plate (cuboid) with two hinge knuckles + a pin running through them
- Knuckles: cylinders welded to the plate edge, spaced apart along the
  hinge axis (Y), with a solid pin cylinder connecting them
- The pin is part of the frame geometry (not a separate body)
- Attached to world via Free joint + gravcomp, doesn't move

**Door** (free joint, light):
- Flat plate (cuboid) with one hinge knuckle that has a bore
- The door's knuckle interleaves between the frame's two knuckles
- The bore slides over the frame's pin — this is the constraint
- Bore constrains radially (XZ), knuckle faces constrain axially (Y)
- Exactly 1 free DOF: rotation around the pin (Y axis)

**Gravity** pulls the door to swing. Start it displaced from vertical
and let it pendulum.

## Why this works

- Everyone knows what a door hinge looks like
- The constraint is VISIBLE — you see the door's knuckle wrapping around
  the frame's pin, held between the frame's knuckles
- The geometry is simple CSG: cylinders + cuboids + unions + subtracts
- It uses only what's proven: bore radial constraint + analytical normals
- Zero magic — the shape is obviously why it moves that way

## CSG geometry sketch

```
Frame:
  plate:     cuboid(40, 5, 60)            — tall flat plate
  knuckle1:  cylinder(R=4, h=5) at y=-7   — lower knuckle
  knuckle2:  cylinder(R=4, h=5) at y=+7   — upper knuckle
  pin:        cylinder(R=2.0, h=20) along Y — solid pin through both knuckles
  → union all (smooth_union for clean seams)

Door:
  plate:     cuboid(40, 5, 60) offset in X — the swinging door
  knuckle:   cylinder(R=4, h=5) at y=0     — interleaves between frame knuckles
             subtract bore(R=2.5, h=6)      — bore slides over frame's pin
  → union plate + knuckle (smooth_union)

Clearance: bore R=2.5 − pin R=2.0 = 0.5mm radial
Axial gap:  knuckle spacing − door knuckle width ≈ 1mm per side

Hinge axis: Y (horizontal). Gravity: -Z. Door swings in XZ plane.
```

## What we need from the engine

All proven:
- Concave SDF collision (Tier 3 grid path + per-shape Newton refinement)
- Analytical CSG normals via AnalyticalShape
- PGS solver with frictionless bore contacts (condim=1)
- body_gravcomp to fix the frame
- Smooth unions/subtracts for clean mesh seams

## Camera

Side view (looking along Y / bore axis) so the door swing is obvious.
Orbit camera centered on the hinge point.
