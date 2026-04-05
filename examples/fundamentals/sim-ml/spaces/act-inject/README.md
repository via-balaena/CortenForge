# Action Injection

Pendulum driven by a sinusoidal action through an `ActionSpace` with
clamped control range [-2, 2]. The HUD shows the full chain: action
tensor value, the resulting `data.ctrl[0]`, and the actuator force.

## What you see

- A pendulum oscillating under a sinusoidal torque
- HUD showing action (f32) → ctrl (f64) → actuator force (f64)
- TensorSpec bounds displayed (should match ctrlrange)

## Expected behavior

- Pendulum swings back and forth, driven by the sinusoidal input
- Action value sweeps smoothly between -2.0 and +2.0
- ctrl tracks the action value exactly (within f32 precision)
- Actuator force matches ctrl (gain=1 motor)
- Pendulum velocity changes sign multiple times (oscillation)

## Validation

| Check | Expected | Tolerance |
|-------|----------|-----------|
| act_space.dim() | 1 | exact |
| ctrl == action (as f64) | match | f32 epsilon |
| spec bounds | low=[-2], high=[2] | exact |
| Oscillation | qvel sign changes >= 4 | — |

## Run

```
cargo run -p example-ml-spaces-act-inject --release
```
