# Ellipsoid Drag — Shape-Dependent Terminal Velocity

Three shapes of equal mass fall through a dense medium. Each uses the advanced
5-component ellipsoid fluid model (`fluidshape="ellipsoid"`). Shape determines
drag through projected area and semi-axis geometry.

## What it demonstrates

- **Per-geom ellipsoid model**: activated by `fluidshape="ellipsoid"` on any
  child geom. Computes 5 force components: added mass, Magnus lift, Kutta
  lift, linear drag, angular drag.
- **Shape-dependent drag**: a streamlined capsule (small frontal area) falls
  fastest. A flat cylinder (large disc face into the flow) falls slowest.
  A sphere is in between.
- **Equal mass comparison**: all three shapes are 2 kg — only shape differs.

## What you see

Three shapes (green capsule, blue sphere, orange disc) released from the same
height. The capsule pulls ahead quickly while the disc lags behind. The HUD
shows speed and drag/weight percentage for each shape.

## Run

```
cargo run -p example-passive-ellipsoid-drag --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
