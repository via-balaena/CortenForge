# Subtree Velocity Sensor

Composite center-of-mass velocity of a free-falling multi-body chain.

## What it demonstrates

- `<subtreelinvel>` gives the linear velocity of the subtree's COM
- Aggregates momentum across all bodies rooted at the sensor body
- In free fall: v_z = −g×t regardless of internal joint motion
- The links flex and tumble but the COM velocity is a clean linear ramp

## Expected visual behavior

A 3-link chain (grey + blue + red capsules) hovers at z=3 for 2 seconds,
then drops. The links are initially bent and flex during the fall. The
camera is zoomed out and tilted down to keep the chain in frame as it
accelerates. Report prints at t=5s (sim time).

## Expected console output

```
t=  0.5s  v_z=-4.9010  expected=-4.9050  err=0.082%
t=  1.0s  v_z=-9.8002  expected=-9.8100  err=0.100%
t=  1.5s  v_z=-14.6993 expected=-14.7150 err=0.107%
t=  2.0s  v_z=-19.5984 expected=-19.6200 err=0.110%
```

v_z ramps linearly, matching −g×t to ~0.1%.

## Pass/fail criteria

| Check | Condition | Tolerance |
|-------|-----------|-----------|
| v_z = −g×t at t=2s | v_z ≈ −19.62 m/s | < 0.5% |
| v_xy ≈ 0 | no horizontal COM velocity | < 0.01 m/s |
| COM descends | com_z(3s) < com_z(0s) | verified |

## Run

```bash
cargo run -p example-sensor-adv-subtree-velocity --release
```
