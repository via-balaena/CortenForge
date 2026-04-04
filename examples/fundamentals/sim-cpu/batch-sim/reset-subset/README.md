# Reset Subset — Selective Reset with `reset_where(mask)`

Sixteen pendulums in a row, each with a different initial angular velocity.
Every 0.25s of sim time, any pendulum whose angle exceeds +/-90 deg gets
reset via `BatchSim::reset_where(mask)`. Low-velocity pendulums never cross
the threshold; high-velocity ones get reset frequently.

This is the RL "done" pattern: selectively reset finished environments
while others continue uninterrupted.

## What you see

- **Sixteen pendulums** in a row, color gradient: blue (low velocity) to
  orange (high velocity)
- Blue pendulums oscillate gently and never reset
- Orange pendulums swing wide, snap back to start on reset, swing again
- HUD shows per-env angle, reset count, and total resets

## Physics

Each pendulum starts at 45 deg tilt with velocity `v_i = i * 0.5 rad/s`.
A pendulum needs omega >= ~5.2 rad/s to swing from 45 deg past 90 deg
(energy conservation). Envs 0-10 lack the energy; envs 11-15 cross easily.

| Env group | Initial omega (rad/s) | Crosses +/-90 deg? | Gets reset? |
|-----------|----------------------|-------------------|-------------|
| 0-4 | 0.0-2.0 | No | Never |
| 5-10 | 2.5-5.0 | Borderline | Rarely |
| 11-15 | 5.5-7.5 | Yes | Frequently |

## Key API demonstrated

```rust
// Build mask: which envs crossed the threshold?
let mask: Vec<bool> = (0..16)
    .map(|i| batch.env(i).unwrap().qpos[0].abs() > PI/2)
    .collect();

// Reset only those envs — others continue uninterrupted
batch.reset_where(&mask);

// Re-apply per-env initial conditions on reset envs
for i in 0..16 {
    if mask[i] {
        let env = batch.env_mut(i).unwrap();
        env.qpos[0] = initial_angle;
        env.qvel[0] = initial_velocity(i);
    }
}
```

## Validation

Four automated checks at t=10s:

| Check | Expected |
|-------|----------|
| Low-velocity envs never reset | envs 0-4 have zero resets |
| High-velocity envs reset | envs 13-15 have reset at least once |
| Reset restores state | total resets > 0 |
| Non-reset envs untouched | envs 0-4 have continuous time near 10s |

## Run

```
cargo run -p example-batch-sim-reset-subset --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
