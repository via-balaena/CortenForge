# JointPos + JointVel Sensors

Direct state readback — the simplest "real" sensor pair.

## What it demonstrates

- `<jointpos>` reads `qpos` for a hinge joint (1D scalar, radians)
- `<jointvel>` reads `qvel` for a hinge joint (1D scalar, rad/s)
- Both are position-stage and velocity-stage pass-throughs respectively

## Expected visual behavior

A hinge pendulum displaced 45° from vertical swings back and forth. Zero damping,
RK4 integrator — the pendulum swings indefinitely with no energy loss.

## Expected console output

```
t=  1.0s  pos=+0.XXX rad  vel=+0.XXX rad/s  sensor_err=0.00e0
t=  2.0s  pos=-0.XXX rad  vel=-0.XXX rad/s  sensor_err=0.00e0
...
```

Position oscillates between approximately ±0.785 rad (±45°). Velocity changes
sign at each turning point.

## Pass/fail criteria

| Check | Condition | Tolerance |
|-------|-----------|-----------|
| JointPos == qpos | sensor matches backing state every frame | < 1e-14 |
| JointVel == qvel | sensor matches backing state every frame | < 1e-14 |
| Pos range > 1 rad | pendulum actually swings (not stuck at zero) | range > 1.0 |
| Vel sign changes | velocity reverses at each swing apex | >= 10 in 15s |
| Energy conservation | undamped system (harness tracker) | < 0.5% drift |

## Run

```bash
cargo run -p example-sensor-joint-pos-vel --release
```
