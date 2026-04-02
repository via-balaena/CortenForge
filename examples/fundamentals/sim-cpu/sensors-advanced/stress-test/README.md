# Stress Test — Headless Advanced Sensor Validation

Exhaustive headless validation of every advanced sensor type. No window,
no Bevy — pure sim-core assertions against analytical expectations.

## What it validates

| # | Check | Invariant |
|---|-------|-----------|
| 1 | FrameLinVel spinning tip | \|v\| = ωR to 0.5% |
| 2 | FrameLinVel stationary | \|v\| < 1e-6 m/s |
| 3 | FrameLinVel free-fall | v_z = -gt to 0.5% |
| 4 | FrameAngVel constant spin | ω_z = 2π, ω_xy ≈ 0 |
| 5 | FrameAngVel at rest | \|ω\| < 1e-6 rad/s |
| 6 | FrameLinAcc at rest | a = [0, 0, +g] (gravity pseudo-acceleration) |
| 7 | FrameLinAcc free fall | \|a\| ≈ 0 (weightless) |
| 8 | FrameLinAcc centripetal | \|a\| > g at bottom of swing |
| 9 | FrameAngAcc constant ω | α ≈ 0 (no angular acceleration) |
| 10 | FrameAngAcc torque pulse | α = τ/I (reasonable range) |
| 11 | Force static beam | \|F\| = mg to 2% |
| 12 | Torque static beam | \|τ\| = mgL/2 to 2% |
| 13 | Force/Torque free zero-g | \|F\|, \|τ\| < 1e-8 |
| 14 | Rangefinder distance | reading = height above ground to 0.5% |
| 15 | Rangefinder no-hit | reading = -1 exactly |
| 16 | Rangefinder self-exclusion | ray through own geoms → -1 |
| 17 | SubtreeLinVel free-fall | v_z = -gt to 0.5% |
| 18 | SubtreeLinVel linear | v(1s)/v(0.5s) = 2.0 to 0.5% |
| 19 | SubtreeAngMom conserved | \|ΔL\|/\|L₀\| < 1e-8 over 5s |
| 20 | ActuatorPos = gear × JointPos | exact to 1e-10 |
| 21 | ActuatorVel = gear × JointVel | exact to 1e-10 |
| 22 | Frame*Axis = site_xmat columns | exact to 1e-10 |
| 23 | Magnetometer frame rotation | \|B\| preserved, R_site^T correct |
| 24 | Sensor dimensions | 29 sensor types, all dimensions correct |

## Expected output

```
TOTAL: 25/25 checks passed
ALL PASS
```

## Run

```
cargo run -p example-sensor-adv-stress-test --release
```
