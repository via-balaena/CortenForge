# Example 09 — Rewrite: Door Hinge

## Decision

The spinning-pin-in-a-box mechanism was rejected — it looks like magic.
You can't see the constraint happening. Replacing with a **door hinge**:
the most universally understood geometry-driven joint.

## What to build

**Frame** (fixed, heavy, gravcomp):
- Flat plate (cuboid) with two hinge knuckles
- Knuckles: half-cylinders with bores, spaced apart along the hinge axis
- Attached to world, doesn't move

**Door** (free joint, light):
- Flat plate (cuboid) with one hinge knuckle
- The door's knuckle interleaves between the frame's two knuckles
- Shares the same bore axis — interlocking cylinders constrain rotation

**No separate pin part.** The interlocking knuckle geometry IS the
constraint. The bore walls constrain radially, the knuckle faces
constrain axially. Exactly 1 free DOF: rotation around the hinge axis.

**Gravity** pulls the door to swing. Start it displaced from vertical
and let it pendulum.

## Why this works

- Everyone knows what a door hinge looks like
- The constraint is VISIBLE — you see knuckles wrapping around each other
- The geometry is simple CSG: cylinders + cuboids + subtracts
- It uses only what's proven: bore radial constraint + analytical normals
- Zero magic — the shape is obviously why it moves that way

## CSG geometry sketch

```
Frame plate:    cuboid(40, 5, 60)   — tall flat plate
Frame knuckle1: cylinder(R=4, h=5) at y=-7, subtract bore(R=2.5, h=6)
Frame knuckle2: cylinder(R=4, h=5) at y=+7, subtract bore(R=2.5, h=6)

Door plate:     cuboid(40, 5, 60)   — same size, offset in X by ~80mm
Door knuckle:   cylinder(R=4, h=5) at y=0, subtract bore(R=2.5, h=6)
                (interleaves between frame knuckles)

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
