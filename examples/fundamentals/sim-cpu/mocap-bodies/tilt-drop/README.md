# Tilt Drop — Orientation-Driven Mocap Interaction

Demonstrates `mocap_quat` driving contact interaction with dynamic bodies
through geometry orientation change.

## Concept

**Orientation-driven mocap interaction.** A mocap platform (orange box) tilts
slowly via scripted `mocap_quat` updates (rotating about Y). A ball sitting
on top stays in place at shallow angles but slides off once the tilt exceeds
the friction limit, falling to the ground.

## What you're seeing

- **Orange box:** The mocap platform. Its orientation is set directly via
  `data.mocap_quat` each frame — it tilts linearly from 0 to ~35 degrees
  over 4 seconds. No physics can resist the tilt.

- **Blue sphere:** A free-floating dynamic ball resting on the platform.
  Gravity pulls it straight down. As the platform tilts, the contact normal
  rotates, and the gravity component along the surface grows until friction
  can no longer hold the ball.

## What to look for

- The platform slowly tilts while the ball stays on it (friction holds).
- At a critical angle the ball begins to slide.
- The ball slides off the edge and falls to the ground.
- The platform continues tilting at the same rate — unaffected by the ball's
  departure or weight.

## Run

```
cargo run -p example-mocap-bodies-tilt-drop --release
```
