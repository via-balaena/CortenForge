# Elastic Bounce — Conservation Through Contact

A ball dropped from 0.5 meters onto a hard floor. The contact parameters
(`solref`) are tuned for near-perfect elasticity — the ball should bounce
back to almost the same height it was dropped from.

## What you should see

A blue ball falls, hits the floor, and bounces back up repeatedly. Each
bounce reaches nearly the same height as the previous one. Over many
bounces, the height slowly decreases as small amounts of energy are lost
to the contact solver's damping.

## The contact energy puzzle

Watch the "total E" and "E loss %" values in the HUD. During the brief
instant the ball is in contact with the floor, total energy appears to
drop sharply. This is not a bug — it's because the contact spring stores
elastic energy that `energy_potential` doesn't track (it only tracks
gravitational and joint spring PE). Once the ball separates from the
floor, the energy comes back.

The true conservation test is the energy measured at the apex of each
bounce, when the ball is in free flight with zero velocity — no contact
spring artifacts.

## What to watch for in the HUD

| Field | Expected |
|-------|----------|
| height | Starts at 0.55, bounces back to ~0.535 |
| KE | Zero at apex, peaks during fall/rise |
| PE | Tracks height (gravitational) |
| total E | ~5.40 J in free flight, dips during contact |
| E loss % | < 3% at apex, spikes during contact |
| restitution | ~0.97 (bounce height / drop height) |
| bounces | Counting up over time |

## Run

```
cargo run -p example-energy-elastic-bounce --release
```
