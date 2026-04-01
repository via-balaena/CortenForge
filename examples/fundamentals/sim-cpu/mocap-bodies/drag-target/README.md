# Drag Target — Soft Weld Tracking

Demonstrates how a mocap body drives a dynamic body through a compliant weld
constraint — the standard pattern for teleop, VR controllers, and interactive
dragging.

## Concept

**Weld-to-mocap tracking.** A translucent green mocap sphere moves on a
scripted sinusoidal path. A solid orange box is connected to the mocap body
via a weld equality constraint with compliance (`solref="0.02 1.0"`). The
box follows the target with visible spring-like lag.

## What you're seeing

- **Green sphere (translucent):** The mocap target. Its position is set
  directly via `data.mocap_pos` each physics step. It glides on a smooth
  circular path and is never affected by physics — it goes exactly where the
  code puts it.

- **Orange box (solid):** A free-floating dynamic body. The weld constraint
  exerts a spring-damper force pulling it toward the mocap target. It chases
  the sphere but always lags behind because the constraint is compliant, not
  rigid.

- **The gap between them:** This is the weld constraint violation — visible
  proof that the constraint is soft. A stiffer `solref` would shrink the gap;
  a softer one would widen it.

## What to look for

- The box tracks the sphere but never quite catches it — there's always a
  visible separation during motion.
- When the sphere slows (at the turning points of the sine wave), the box
  closes the gap.
- The sphere itself moves perfectly smoothly — no jitter, no physics
  influence.

## Run

```
cargo run -p example-mocap-bodies-drag-target --release
```
