# Solref Bounce — Contact Stiffness via Solver Reference

**Three spheres dropped from 0.5m with the same stiffness but different damping.**

See also: [friction-slide](../friction-slide/) | [solimp-depth](../solimp-depth/)

## What you see

- Red sphere (B=10) bounces many times — very underdamped
- Green sphere (B=30) bounces once or twice — slightly underdamped
- Blue sphere (B=500) stops dead on impact — very overdamped

## Physics

In MuJoCo's constraint-based contact, there is no restitution coefficient.
Bounce behaviour comes entirely from the contact impedance parameters.

All three balls use direct-mode `solref=[-K, -B]` with the same stiffness
K=5000 but different damping B. This isolates the damping effect:

- Low B relative to critical → underdamped → oscillates (bounces)
- B near critical → slight bounce, settles quickly
- High B → overdamped → no oscillation, stops immediately

All three use `condim=1` (frictionless) to isolate the bounce effect.

## Parameters

| Parameter | Bouncy (red) | Moderate (green) | Absorbing (blue) |
|-----------|-------------|-----------------|-----------------|
| solref | [-5000, -10] | [-5000, -30] | [-5000, -500] |
| Stiffness K | 5000 | 5000 | 5000 |
| Damping B | 10 | 30 | 500 |

| Parameter | Value |
|-----------|-------|
| Drop height | 0.5 m (center at 0.55m, radius 0.05m) |
| Sphere mass | 0.5 kg |
| Timestep | 0.5 ms |

## Validation

| Check | Expected | Threshold |
|-------|----------|-----------|
| Bouncy bounces high | max z > 0.15m after contact | PASS |
| Bouncy > moderate | bouncy bounce height > moderate | PASS |
| Moderate < bouncy | moderate bounce height < bouncy | PASS |

## Run

```sh
cargo run -p example-contact-solref-bounce --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
