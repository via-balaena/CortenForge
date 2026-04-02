# Wind — Global Wind Field

Two demonstrations of the global wind field (`<option wind="vx vy vz"/>`).
Wind subtracts from body velocity before computing drag, so a stationary body
in wind experiences the same force as a body moving at -wind in still air.

## What it demonstrates

- **Free sphere drift**: a sphere released from rest drifts sideways in a
  crosswind while falling — parabolic trajectory instead of straight down.
- **Pendulum deflection**: a hinged arm with a bob deflects from vertical
  under steady wind load, then recovers when wind stops.
- **Wind on/off**: 3-second calm, gravity at 3s, wind at 3.25s, wind off
  at 6s. Shows both the wind-driven response and the recovery.

## What you see

A blue sphere falls and drifts rightward in wind, then decelerates after wind
stops. An orange pendulum bob swings to ~55 degrees under wind, then swings
back toward vertical when wind cuts out. The HUD shows velocity, pendulum
angle, and wind state.

## Run

```
cargo run -p example-passive-wind --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
