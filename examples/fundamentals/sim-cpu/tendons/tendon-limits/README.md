# Tendon Limits — Range Constraints on Tendon Length

A pendulum tethered by a spatial tendon with a maximum length limit. When
the tendon reaches its limit, the solver generates a one-sided constraint
that stops the motion — like a resistance band going taut.

## Concept

A **tendon limit** constrains the tendon length to stay within a range. When
the length hits the upper bound, a constraint force resists further extension
(like a cable reaching maximum length). The `TendonLimitFrc` sensor reports
the constraint force — zero when slack, positive when taut.

## What you see

- A pendulum starts horizontal (3 o'clock), holds briefly, then releases.
- A tendon runs from a fixed red anchor above the pivot to the pendulum tip.
- As the pendulum swings away from the anchor, the tendon stretches —
  color shifts from **green** (slack) through **yellow** to **red** (at limit).
- When the limit catches, the pendulum visibly stops and bounces back —
  like a ball on a leash.
- Reduced gravity (4.0 m/s²) for a slower, more dramatic swing.

## Run

```
cargo run -p example-tendon-limits --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
