# Pulley — 2:1 Mechanical Advantage via Tendon Branch Scaling

Two masses on parallel vertical rails connected through a pulley. A motor
drives the bottom mass (pull side) up and down. The top mass (load) follows
— but at exactly half the rate. The pulley's `divisor="2"` creates a 2:1
mechanical advantage.

## Concept

A `<pulley divisor="2"/>` in a spatial tendon path splits the tendon into two
branches. Branch 1 (top mass → anchor) contributes its full length. Branch 2
(anchor → bottom mass) contributes `length / 2`. So when the bottom mass moves
by ±X, the top mass only needs to move by ±X/2 to keep the tendon the same
length. This is the same principle as a block-and-tackle: trade distance for
force.

## What you see

- **Red box (bottom):** motor-driven, slides up and down on a vertical rail.
- **Blue box (top):** passive load, follows at half the rate.
- **Yellow sphere:** the fixed pulley anchor between the two rails.
- Both masses are spring-centered (stiffness=8) so they return to rest.
- Ruler tick marks along each rail (every 5 cm) make the 2:1 ratio visible.
- 4-color tendon lines show branch state:
  - **Red / Orange:** stretching (top / bottom branch).
  - **Blue / Teal:** contracting (top / bottom branch).
  - **Gray:** near zero velocity.
- The HUD shows positions, travel ranges, and the pull-to-load ratio
  (should converge to ~2.0×).

## Run

```
cargo run -p example-tendon-pulley --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
