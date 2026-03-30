# Damped Decay — Energy Dissipation

A pendulum with joint damping, released from 45 degrees. Unlike the
undamped pendulum (example 2), this one loses energy with every swing.
The amplitude shrinks steadily until the pendulum comes to rest hanging
straight down.

## What you should see

A red-bobbed pendulum swinging with decreasing amplitude. Each swing is
visibly smaller than the last. After ~10 seconds the pendulum is barely
moving — nearly all kinetic energy has been dissipated by the damper.

The key property: energy **never increases**. Not even for a single
timestep. Damping is a one-way valve — it only removes energy, never
adds it.

## What to watch for in the HUD

| Field | Expected |
|-------|----------|
| KE | Oscillates but with shrinking peaks, approaches 0 |
| total E | Steadily decreasing (never increases) |
| KE_max | Captured peak KE (~0.109 J) |
| KE % of max | Drops toward 0% over time |
| violations | 0 (energy never increases) |
| angle | Oscillation amplitude shrinks toward 0 |

## Contrast with example 2

The undamped pendulum (pendulum-energy) swings forever with constant
amplitude and constant total energy. This example adds one thing —
damping — and the behavior changes fundamentally: energy decays to
zero.

## Run

```
cargo run -p example-energy-damped-decay --release
```
