# Vertical Slide Joint — Gravity-Shifted Spring-Mass Oscillator

A block rides a vertical rail, suspended by a spring from an overhead mount.
Gravity pulls it down; the spring pulls it back up. Released below equilibrium,
it oscillates as a damped harmonic oscillator around a **gravity-shifted rest
position** — the key difference from the horizontal sibling example.

## What you see

- **Blue block** — the 1 kg mass, free to slide along the vertical axis
- **Coil spring above** — connects the mount to the block top, stretches and
  compresses in real time as the block oscillates
- **Overhead mount** — metallic box at the top of the rail
- **Steel rail** — vertical axis of motion (collision disabled, visual only)
- **Ground plane** — visual reference at the bottom

## Physics

The slide joint restricts a body to **linear motion along a single axis**
(1 DOF). Here the axis is vertical, so gravity acts along it.

The equation of motion is:

```
m_eff * x'' + c * x' + k * x = -m * g
```

Gravity shifts the equilibrium from `x = 0` to:

```
x_eq = -m * g / k = -1.0 * 9.81 / 40 = -0.2453 m
```

The oscillation period is **unchanged** by gravity — it only shifts *where*
the block oscillates, not *how fast*:

```
T = 2pi * sqrt(m_eff / k)
```

| Parameter | Value |
|-----------|-------|
| Mass | 1.0 kg |
| Armature | 0.1 kg |
| Stiffness | 40 N/m |
| Damping | 0.3 Ns/m |
| Range | -1.5 to +1.0 m (asymmetric) |
| Initial displacement | -0.5 m |
| Equilibrium | -0.2453 m |

## Validation

The example runs four automated checks at t=15s:

| Check | Expected | Threshold |
|-------|----------|-----------|
| **Period** | `T = 2pi * sqrt(1.1 / 40) = 1.0416 s` | < 2% error |
| **Energy** | Monotonically decreasing (damping only dissipates) | < 1e-6 J increase |
| **Limits** | Position stays within [-1.5, 1.0] m | < 1 mm violation |
| **Equilibrium** | Mean position → -0.2453 m (trailing 5s window) | < 5% error |

The equilibrium check is new vs. the horizontal example — it validates that
the engine correctly computes the gravity-shifted rest position.

## Run

```
cargo run -p example-slide-joint-vertical --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
