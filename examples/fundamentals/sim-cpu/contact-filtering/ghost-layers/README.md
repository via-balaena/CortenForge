# Ghost Layers — Collision Layers via Bitmasks

Demonstrates how to use contype/conaffinity bitmasks to create physics layers
where some objects interact with others selectively.

## Concept

**Collision layers.** Two layers are defined using bit positions:

| Layer | contype | conaffinity | Meaning |
|-------|---------|-------------|---------|
| Solid | 1 (bit 0) | 3 (bits 0+1) | Collides with everything |
| Ghost | 2 (bit 1) | 1 (bit 0) | Collides with solids, not with other ghosts |

The math:
- **Solid vs Solid:** `(1&3)||(1&3)` = true — solids collide with each other.
- **Solid vs Ghost:** `(1&1)||(2&3)` = true — ghosts interact with solids.
- **Ghost vs Ghost:** `(2&1)||(2&1)` = false — ghosts pass through each other.

This pattern is useful for sensor volumes, trigger zones, or any object that
needs to detect solid surfaces without interfering with other sensors.

## What you're seeing

- **Blue box (solid):** Sits on the ground. Part of the solid layer.

- **Gold box (solid):** Stacks on top of the blue box. Solid-solid contact
  works normally.

- **Two green spheres (ghost, translucent):** Dropped from above onto the
  solid stack. They rest on the top solid surface (ghost-solid contact works)
  but pass through each other (ghost-ghost contact disabled). Both end up at
  the same height, side by side — not stacked.

## What to look for

- The two solid boxes stack normally.
- Both ghost spheres land on the solid stack at the same Z height.
- The HUD shows `ghost<>ghost: 0 contacts` throughout the simulation.
- If the ghosts were solid, one would stack on top of the other. Instead,
  they overlap at the same elevation.

## Run

```
cargo run -p example-contact-filtering-ghost-layers --release
```
