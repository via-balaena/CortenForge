# Bitmask — Contype/Conaffinity Filtering

Demonstrates the bitmask rule that governs which geometry pairs can collide.
Four spheres with different `contype`/`conaffinity` values fall onto a ground
plane with default values (`contype=1`, `conaffinity=1`).

## Concept

**The bitmask AND/OR rule.** Two geoms collide if
`(g1.contype & g2.conaffinity) != 0 || (g2.contype & g1.conaffinity) != 0`.
This is an OR — if *either* side of the bitwise AND matches, contact happens.
Both sides must fail for filtering to reject the pair.

## What you're seeing

Four colored spheres dropped from the same height:

- **Blue "full" (1,1):** Both contype and conaffinity match the ground.
  `(1&1)||(1&1)` = true. **Rests on ground.**

- **Green "layer2" (2,2):** On a completely different bitmask layer.
  `(2&1)||(1&2)` = false. **Falls through.**

- **Red "off" (0,0):** Both fields zeroed — matches nothing.
  `(0&1)||(1&0)` = false. **Falls through.** Rendered translucent to
  visually signal its "ghost" status.

- **Gold "type" (1,0):** contype matches but conaffinity is zero.
  `(1&1)||(1&0)` = true — contype alone is enough because of the OR.
  **Rests on ground.**

## What to look for

- Blue and gold land on the ground and stay.
- Green and red pass straight through the ground and disappear below.
- The HUD shows RESTING/FALLING status and the bitmask values for each sphere.

## Run

```
cargo run -p example-contact-filtering-bitmask --release
```
