# Slide Joint — Prismatic Spring-Mass Oscillator

A block rides a frictionless rail, held between two springs attached to the
walls at each end. Displaced from center and released, it oscillates back and
forth as a damped harmonic oscillator — the simplest system that demonstrates
a **slide (prismatic) joint**.

## What you see

- **Red block** — the 1 kg mass, free to slide along one axis
- **Two metallic coil springs** — one on each side of the block, connecting
  it to the walls. They stretch and compress in real time as the block moves.
- **Grey walls** — the joint range limits at ±1.0 m
- **Steel rail** — the axis of motion (collision disabled, visual only)

## Physics

The slide joint restricts a body to **linear motion along a single axis**
(1 DOF). It is specified in MJCF as `type="slide"` with an `axis` vector.

The joint spring provides a restoring force `F = -k * x` where `x` is the
displacement from the rest position. With damping, this becomes a classic
damped harmonic oscillator:

```
m_eff * x'' + c * x' + k * x = 0
```

The effective mass includes **armature** (reflected rotor inertia):
`m_eff = m + armature = 1.0 + 0.1 = 1.1 kg`.

| Parameter | Value |
|-----------|-------|
| Mass | 1.0 kg |
| Armature | 0.1 kg |
| Stiffness | 20 N/m |
| Damping | 0.2 Ns/m |
| Range | ±1.0 m |
| Initial displacement | 0.8 m |

## Validation

The example runs three automated checks at t=15s:

| Check | Expected | Threshold |
|-------|----------|-----------|
| **Period** | `T = 2pi * sqrt(1.1 / 20) = 1.4735 s` | < 2% error |
| **Energy** | Monotonically decreasing (damping only dissipates) | < 1e-6 J increase |
| **Limits** | Position stays within ±1.0 m | < 1 mm violation |

## Run

```
cargo run -p example-slide-joint-horizontal --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
