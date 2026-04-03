# Sleep Settle — Velocity Threshold and Sleep Transition

The fundamental sleep mechanism: bodies go to sleep when their velocity drops
below the threshold for long enough. No disturbances, no wake triggers — just
gravity, contact, and the countdown timer.

## What you see

- **Five orange boxes** drop from staggered heights (1–3 m) onto a grey plane
- The lowest box lands first, bounces briefly, then turns **steel blue** as it
  sleeps — its velocity has been below `sleep_tolerance` for 10 consecutive
  timesteps
- Each subsequent box follows — a blue wave ripples from left to right as boxes
  settle at different times
- The HUD counts down `awake 5/5` → `4/5` → ... → `0/5`

## Physics

The sleep decision is per-tree, checked every timestep after integration:

```
for each DOF in tree:
    if |qvel[dof]| > sleep_tolerance * dof_length[dof]:
        reset countdown to -(1 + MIN_AWAKE)
        break

if countdown == -1:
    sleep the tree (zero qvel, qacc, qfrc)
else:
    countdown += 1
```

With `sleep_tolerance = 0.05` and `dof_length ≈ 1.0` for translational DOFs,
the threshold is 0.05 m/s. Once all 6 DOFs of a free joint stay below that for
10 steps (20 ms at dt=0.002), the body sleeps. Sleeping zeroes velocity and
acceleration bitwise — not approximately, exactly.

| Parameter | Value |
|-----------|-------|
| Boxes | 5, mass 1 kg each, 0.3 m cubes |
| Drop heights | 1.0, 1.5, 2.0, 2.5, 3.0 m |
| Sleep tolerance | 0.05 (velocity threshold) |
| MIN_AWAKE | 10 timesteps (20 ms) |
| Contact | solref = [0.005, 1.5] (high damping, fast settle) |
| Integrator | Euler, dt = 2 ms |

**Key distinction:** sleep is not "velocity is small" — it's "velocity has been
small for at least 10 consecutive steps." A single spike resets the countdown.

## Validation

Four automated checks at t=15s:

| Check | Expected |
|-------|----------|
| All awake at start | nbody_awake == 6 at t=0.1 |
| All asleep by t=6 | nbody_awake == 1 (world only) |
| Sleeping qvel = 0 | Bitwise zero for all sleeping DOFs |
| Sleeping qacc = 0 | Bitwise zero for all sleeping DOFs |

## Run

```
cargo run -p example-sleep-wake-settle --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
