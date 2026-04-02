# Fluid Drag — Terminal Velocity

Three spheres of different mass fall through a dense medium. Each reaches a
different terminal velocity where drag force exactly balances weight.

## What it demonstrates

- **Inertia-box fluid model**: body-level drag computed from equivalent box
  dimensions derived from body inertia. No per-geom fluid params needed.
- **Terminal velocity**: lighter objects reach terminal velocity faster and
  fall slower. Heavier objects accelerate longer before drag catches up.
- **Drag-weight balance**: at terminal velocity, net acceleration is zero
  because drag force = weight.

## What you see

Three colored spheres (blue=light, gold=medium, red=heavy) released from the
same height. They separate as each reaches its own terminal velocity. The HUD
shows speed and drag/weight percentage for each sphere, plus medium properties.

## Run

```
cargo run -p example-passive-fluid-drag --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
