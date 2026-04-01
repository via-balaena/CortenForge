# Push Object — Mocap Contact Interaction

Demonstrates one-way contact between a mocap body and a dynamic body. The
mocap body pushes things without being pushed back.

## Concept

**Mocap contact interaction.** A mocap paddle (red capsule) sweeps linearly
across the scene at constant speed. A dynamic ball (blue sphere) sits on the
ground plane in the paddle's path. When the paddle reaches the ball, contact
forces push the ball; the paddle continues unaffected.

## What you're seeing

- **Red capsule:** The mocap paddle. Its position is set directly via
  `data.mocap_pos` each frame — it moves at a constant 1 m/s along +X
  regardless of what it hits.

- **Blue sphere:** A free-floating dynamic ball resting on the ground plane.
  When the paddle reaches it, the ball is pushed along continuously.

- **The asymmetry:** The paddle does not slow down, deflect, or react to the
  ball in any way. Contact forces are computed, but they only act on the
  dynamic body. This is the defining property of mocap bodies.

## What to look for

- The ball sits still until the paddle arrives.
- On contact, the paddle bulldozes the ball forward at its own speed.
- The paddle sweeps at constant speed — no stutter, no recoil.
- The HUD shows `contacts > 0` while the paddle pushes the ball.

## Run

```
cargo run -p example-mocap-bodies-push-object --release
```
