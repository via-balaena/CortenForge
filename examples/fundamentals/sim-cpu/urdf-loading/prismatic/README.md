# Prismatic Joint Slider

A box sliding back and forth on a horizontal axis, driven by a spring
(joint stiffness injected into the MJCF since URDF lacks a stiffness
attribute).

## What you see

A box oscillating left and right. The HUD shows position, velocity, and
spring force. The ValidationHarness measures the period via zero-crossings.

## What it tests

The URDF `prismatic` joint converts to an MJCF `slide` with the correct
axis mapping and position limits.

## Physics

```
T = 2*pi*sqrt(m/k) = 2*pi*sqrt(2/50) = 1.257s
```

## Validation

| Check | Source |
|-------|--------|
| Prismatic → slide | `print_report` |
| Axis maps to X | `print_report` |
| Period matches T=2pi*sqrt(m/k) | `print_report` zero-crossing (2% tolerance) |

## Run

```
cargo run -p example-urdf-prismatic --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
