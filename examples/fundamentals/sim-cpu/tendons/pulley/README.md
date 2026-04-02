# Pulley — Mechanical Advantage via Tendon Branch Scaling

Two masses on parallel horizontal rails connected through a pulley. A motor
drives the top mass left and right. The bottom mass follows — but at exactly
half the rate. The pulley's `divisor="2"` creates a 2:1 mechanical advantage.

## Concept

A `<pulley divisor="2"/>` in a spatial tendon path splits the tendon into two
branches. Branch 1 (top mass → anchor) contributes its full length. Branch 2
(anchor → bottom mass) contributes `length / 2`. So when the top mass moves
by ±X, the bottom mass only needs to move by ±X/2 to keep the tendon the same
length. This is the same principle as a block-and-tackle: trade distance for
force.

## What you see

- **Blue box (top):** motor-driven, slides left and right.
- **Red box (bottom):** passive, follows at half the rate.
- **Yellow sphere:** the fixed pulley anchor between the two rails.
- Yellow tendon lines show the two branches connecting through the pulley.
- The HUD shows both positions and their ratio (should hover around 2.0×).

## Run

```
cargo run -p example-tendon-pulley --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
