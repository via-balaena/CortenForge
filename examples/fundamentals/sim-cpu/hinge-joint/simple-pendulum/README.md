# Simple Pendulum — Hinge Joint

A rigid rod with a tip mass, hanging from a hinge joint. Displaced 30° from
vertical and released, it swings back and forth — the simplest system that
demonstrates a **hinge (revolute) joint**.

## What you see

- **Red tip sphere** — the 1 kg point mass at the end of the rod
- **Steel rod** — rigid capsule connecting the pivot to the mass
- **Dark socket** — the hinge pivot point
- **Metallic bracket** — visual mount at the top

## Physics

The hinge joint restricts a body to **rotation about a single axis** (1 DOF).
It is specified in MJCF as `type="hinge"` with an `axis` vector. Unlike a ball
joint (3 DOF, quaternion), the hinge uses a **scalar angle** for both qpos and
qvel.

This is a **physical pendulum** — the period depends on the moment of inertia
about the pivot, not just the rod length:

```
T₀ = 2π√(I_pivot / (m·g·d))
```

where `I_pivot = I_cm + m·d²` (parallel axis theorem) and `d` is the distance
from pivot to center of mass. With the mass concentrated at the tip (d=1.0 m,
I_cm=0.01), `I_pivot ≈ 1.01` and the period is very close to the textbook
point-mass formula `T = 2π√(L/g)`.

At 30° initial amplitude, the period is ~1.7% longer than the small-angle
approximation (correction: `T ≈ T₀(1 + θ₀²/16)`).

| Parameter | Value |
|-----------|-------|
| Mass | 1.0 kg |
| Rod length | 1.0 m |
| Diaginertia | 0.01 (isotropic) |
| Damping | 0 (undamped) |
| Integrator | RK4 |
| Initial angle | 30° |

## Validation

The example runs two automated checks at t=15s:

| Check | Expected | Threshold |
|-------|----------|-----------|
| **Period** | `T ≈ 2.051 s` (physical pendulum + amplitude correction) | < 2% error |
| **Energy** | Conserved (undamped, no contacts) | < 0.5% drift |

## Run

```
cargo run -p example-hinge-joint-pendulum --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
