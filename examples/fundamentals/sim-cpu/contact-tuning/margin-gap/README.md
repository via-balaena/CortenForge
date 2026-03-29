# Margin / Gap — Contact Activation Distance

**Three balls at different heights above a plane — larger margin = ball floats higher.**

See also: [solimp-depth](../solimp-depth/) | [solref-bounce](../solref-bounce/)

## What you see

- Green ball (margin=0) sits right on the surface
- Yellow ball (margin=0.02) floats ~20mm above the surface
- Red ball (margin=0.05) floats ~50mm above the surface

The balls appear to hover because the contact constraint activates before
the geom physically touches the plane.

## Physics

`margin` creates a buffer zone around a geom. The contact constraint
activates when the distance between surfaces is less than `margin`,
before physical penetration occurs.

`gap` subtracts from margin: `includemargin = margin - gap`. A geom
with margin=0.05 and gap=0.03 behaves like effective margin=0.02.

The constraint equilibrium shifts outward by approximately the margin
value, making the ball float above the surface.

**Use cases:**
- Prevent fast-moving objects from tunneling through thin surfaces
- Model soft contact zones (foam padding, rubber bumpers)
- Create clearance in mechanical assemblies

## Parameters

| Parameter | Green | Yellow | Red |
|-----------|-------|--------|-----|
| margin | 0 | 0.02 m | 0.05 m |
| gap | 0 | 0 | 0 |
| Mass | 0.5 kg | 0.5 kg | 0.5 kg |
| Radius | 0.05 m | 0.05 m | 0.05 m |

## Validation

| Check | Expected | Threshold |
|-------|----------|-----------|
| Zero-margin near surface | z < 0.055 m | PASS at t=5s |
| Mid-margin floats higher | z_mid > z_zero + 5mm | PASS |
| High-margin floats highest | z_high > z_mid + 5mm | PASS |

## Run

```sh
cargo run -p example-contact-margin-gap --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
