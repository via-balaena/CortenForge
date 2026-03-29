# Friction Slide — Coulomb Friction on a Tilted Plane

**Three boxes on a 15-degree ramp with different friction coefficients.**

See also: [condim-compare](../condim-compare/) | [pair-override](../pair-override/)

## What you see

- Red box (mu=0.1) slides down the ramp — friction is too low to resist gravity
- Green box (mu=0.5) stays put — friction exceeds the gravity component
- Blue box (mu=1.0) stays put easily

The critical friction threshold is `tan(15deg) = 0.268`. Below this, the
gravity component along the slope exceeds the maximum static friction force.

## Physics

Boxes are used instead of spheres because boxes can't roll. This isolates
the sliding friction effect.

The floor has `friction="0 0 0"`. Since CortenForge combines friction via
element-wise MAX, the effective friction equals the box's friction:
`MAX(0, box_mu) = box_mu`.

For a box on a slope at angle theta with friction mu:
- Gravity component along slope: `F_parallel = m*g*sin(theta)`
- Maximum static friction: `F_friction = mu * m*g*cos(theta)`
- Slides when: `mu < tan(theta)`

## Parameters

| Parameter | Value |
|-----------|-------|
| Slope angle | 15 deg (0.2618 rad) |
| tan(15 deg) | 0.268 |
| Box half-extent | 0.04 m |
| Box mass | 0.5 kg |
| Low friction | 0.1 (slides) |
| Med friction | 0.5 (holds) |
| High friction | 1.0 (holds) |

## Validation

| Check | Expected | Threshold |
|-------|----------|-----------|
| Low-mu slides | displacement > 0.1m after settling | PASS at t=5s |
| Low-mu peak velocity | > 0.5 m/s | PASS at t=5s |
| Med-mu holds | vel < 0.05 m/s | PASS at t=5s |
| High-mu holds | vel < 0.05 m/s | PASS at t=5s |

## Run

```sh
cargo run -p example-contact-friction-slide --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
