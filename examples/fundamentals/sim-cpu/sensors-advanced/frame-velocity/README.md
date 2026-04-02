# Frame Velocity Sensors

Linear and angular velocity at a site on a spinning rod.

## What it demonstrates

- `<framelinvel>` gives the 3D linear velocity at the site origin (world frame)
- `<frameangvel>` gives the 3D angular velocity of the parent body (world frame)
- Both are velocity-stage sensors (read from `cvel`)
- For circular motion: |v| = ωR (tip speed = angular velocity times radius)

## Expected visual behavior

A horizontal rod spins smoothly at 1 revolution per second about a vertical
axis. A blue sphere marks the tip. The rod stays in a horizontal plane —
no wobble, no vertical drift.

## Expected console output

```
t=  2.0s  |v|=3.1416  ω_z=6.2832
t=  3.0s  |v|=3.1416  ω_z=6.2832
...
```

After the 2-second spin-up, both readings are rock-steady.

## Pass/fail criteria

| Check | Condition | Tolerance |
|-------|-----------|-----------|
| AngVel Z = 2π | mean ω_z after spin-up | < 0.5% |
| AngVel XY ≈ 0 | max off-axis angular velocity | < 0.01 rad/s |
| |LinVel| = π | mean tip speed after spin-up | < 0.5% |
| LinVel Z ≈ 0 | max vertical velocity | < 0.01 m/s |

## Run

```bash
cargo run -p example-sensor-adv-frame-velocity --release
```
