# Condim Compare — Contact Dimensionality

**Three spheres on a 15-degree ramp with different contact dimensionality.**

See also: [friction-slide](../friction-slide/) | [pair-override](../pair-override/)

## What you see

- Red sphere (condim=1) slides frictionlessly — no tangential forces at all
- Green sphere (condim=3) rolls without slipping — friction prevents sliding but
  allows rolling
- Blue sphere (condim=6) rolls with resistance — rolling friction decelerates it

## Physics

`condim` controls how many constraint rows are generated per contact:

| condim | Rows | Effect |
|--------|------|--------|
| 1 | 1 | Normal force only — frictionless sliding |
| 3 | 3 | + 2 tangential friction — Coulomb cone |
| 6 | 6 | + torsional + 2 rolling — full friction model |

For a solid sphere rolling without slipping on an incline (condim=3):
`a = (5/7) * g * sin(theta)`

This is slower than frictionless sliding (`a = g * sin(theta)`) because
rotational inertia absorbs some of the gravitational potential energy.

Uses `<pair>` overrides to set different condim per sphere-plane contact.

## Parameters

| Parameter | Value |
|-----------|-------|
| Slope angle | 15 deg |
| Sphere radius | 0.05 m |
| Sphere mass | 0.5 kg |
| condim=1 friction | 0 (frictionless) |
| condim=3 friction | 1.0 sliding |
| condim=6 friction | 1.0 sliding, 1.0 rolling |

## Validation

| Check | Expected | Threshold |
|-------|----------|-----------|
| condim=1 has velocity | > 1.0 m/s at t=5s | PASS |
| condim=3 matches rolling | (5/7)*g*sin(theta)*t +/- 15% | PASS |
| condim=6 slower than condim=3 | rolling friction decelerates | PASS |

## Run

```sh
cargo run -p example-contact-condim-compare --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
