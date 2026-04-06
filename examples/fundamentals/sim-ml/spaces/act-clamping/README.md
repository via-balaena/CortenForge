# Ctrl Clamping and Action Bounds

Pendulum with ctrlrange=[-1, 1]. A ramping action sweeps from -3 to +3
over time. The HUD shows the requested action value alongside the actual
clamped ctrl, with a CLAMPED/OK indicator.

## What you see

- A pendulum driven by a slowly ramping torque
- HUD showing requested (f32) vs actual ctrl (f64)
- CLAMPED indicator turns on when the requested value exceeds [-1, 1]

## Expected behavior

- Action ramps linearly from -3 to +3 over 10 seconds, then repeats
- When the requested value is within [-1, 1], ctrl matches exactly
- When outside, ctrl is clamped to exactly -1.0 or +1.0
- TensorSpec low/high match the ctrlrange

## Validation

| Check | Expected | Tolerance |
|-------|----------|-----------|
| ctrl within range | never exceeds [-1, 1] | exact |
| Within-range pass-through | ctrl == action (as f64) | f32 epsilon |
| Boundary values | ctrl == exactly +/-1.0 | exact |
| spec bounds | low=[-1], high=[1] | exact |

## Run

```
cargo run -p example-ml-spaces-act-clamping --release
```
