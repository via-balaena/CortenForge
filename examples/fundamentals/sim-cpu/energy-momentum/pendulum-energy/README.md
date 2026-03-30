# Pendulum Energy — Potential-Kinetic Exchange

An undamped pendulum released from 45 degrees. As it swings, energy
flows back and forth between two forms:

- **Potential energy (PE)** — stored in height. Maximum at the top of
  each swing, when the pendulum momentarily stops.
- **Kinetic energy (KE)** — stored in speed. Maximum at the bottom of
  the swing, when the pendulum is moving fastest.

The total energy (KE + PE) stays perfectly constant — energy is never
created or destroyed, only transformed between forms.

## What you should see

A pendulum swinging back and forth indefinitely. The amplitude never
changes because there is no damping — no friction, no air resistance.

In the HUD, watch KE and PE oscillate in complementary patterns: when
one is high, the other is low. Their sum (total) stays flat.

## What to watch for in the HUD

| Field | Expected |
|-------|----------|
| KE | Oscillates 0 ↔ ~0.575 J |
| PE | Oscillates ~-1.962 ↔ ~-1.387 J |
| total | Constant at ~-1.387 J |
| drift % | ~0% (< 0.5%) |
| KE/PE_drop | Peaks at ~1.0 at bottom of swing |
| angle | Oscillates between ±0.785 rad |

The negative PE values are because the pendulum's center of mass is
below the coordinate origin — the PE reference point is arbitrary.
What matters is that the total never changes.

## Run

```
cargo run -p example-energy-pendulum-energy --release
```
