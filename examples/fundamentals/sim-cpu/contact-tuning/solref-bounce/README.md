# Solref Bounce — Contact Stiffness via Solver Reference

**Three spheres dropped from 0.5m with different contact stiffness/damping.**

See also: [friction-slide](../friction-slide/) | [pair-override](../pair-override/)

## What you see

- Red sphere (stiff) bounces multiple times — high stiffness, low damping
- Green sphere (default) barely bounces — critically damped
- Blue sphere (soft) doesn't bounce at all — overdamped, absorbs energy

## Physics

In MuJoCo's constraint-based contact, there is no restitution coefficient.
Bounce behaviour comes entirely from the contact impedance parameters:

**solref** controls stiffness (K) and damping (B):

| Mode | Format | K | B |
|------|--------|---|---|
| Standard | `[timeconst, dampratio]` | `1/(dmax^2 * tc^2 * dr^2)` | `2/(dmax * tc)` |
| Direct | `[-K, -B]` | `-solref[0]/dmax^2` | `-solref[1]/dmax` |

When K is high and B is low relative to critical damping, the contact
is underdamped — it oscillates (bounces). When B is at critical damping
or above, the contact absorbs energy monotonically.

All three balls use `condim=1` (frictionless) to isolate the bounce effect.

## Parameters

| Parameter | Value |
|-----------|-------|
| Drop height | 0.5 m (center at 0.55m, radius 0.05m) |
| Sphere mass | 0.5 kg |
| Timestep | 0.5 ms (finer for bounce accuracy) |
| Stiff solref | [-5000, -10] (direct mode, underdamped) |
| Default solref | [0.02, 1.0] (standard, critically damped) |
| Soft solref | [0.2, 2.0] (standard, overdamped) |

## Validation

| Check | Expected | Threshold |
|-------|----------|-----------|
| Stiff bounces high | max z > 0.15m after first contact | PASS |
| Stiff > default | stiff bounce height > default | PASS |
| Default doesn't bounce | max z < 0.12m after contact | PASS |

## Run

```sh
cargo run -p example-contact-solref-bounce --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
