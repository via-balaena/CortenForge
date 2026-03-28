# Weld Body-to-Body — Rigid Glue

Two free-floating bodies — a box and a capsule — welded together by a `<weld>`
equality constraint. They fall under gravity as a single rigid unit and land on
the ground. The weld locks all 6 DOFs of relative motion, so the pair behaves
as one solid object.

See also: [Weld to World](../weld-to-world/) — a body frozen in space.

## What you see

- **Orange box** (top) welded to a **steel capsule** (bottom) — they fall
  together as one object
- The pair drops from z=1.5, lands on the ground plane, and comes to rest
- The relative pose between the two bodies stays constant throughout —
  they never flex or separate, even on impact
- A slight bounce on landing shows the penalty stiffness at work

## Physics

The `<weld>` constraint locks the **full relative pose** (position +
orientation) between two bodies. Unlike connect (which allows rotation), weld
freezes everything — the pair is rigidly glued.

During freefall, both bodies accelerate identically under gravity. The
constraint does no work here — it only activates when something tries to
change the relative pose (like ground contact forces on impact).

| Parameter | Value |
|-----------|-------|
| Box mass | 1.0 kg |
| Capsule mass | 0.5 kg |
| Box size | 0.24 x 0.16 x 0.16 m |
| Capsule length | 0.3 m |
| Initial separation | 0.3 m (z-axis) |
| solref | 0.005, 1.0 |
| Ground | Plane at z=0 |

## Validation

Three automated checks at t=5s:

| Check | Expected | Threshold |
|-------|----------|-----------|
| **Relative pos locked** | Constant offset between bodies | < 2 mm |
| **Relative angle locked** | No relative rotation | < 0.05 rad |
| **Pair falls together** | Z-velocities match during freefall | < 5% relative difference |

## Run

```
cargo run -p example-equality-weld-body-to-body --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
