# Inverse Dynamics

Five examples covering inverse dynamics (`data.inverse()`) and the
analytical Jacobian (`mj_jac_site`). Each example demonstrates one
concept, building from the simplest use case to a comprehensive
headless validation.

All examples use the same two-link planar arm (shoulder + elbow hinge
joints, motor actuators with gear=1, no contacts).

## Examples

| Example | Concept | Visual |
|---------|---------|--------|
| [gravity-compensation](gravity-compensation/) | Static holding torques — what torques keep the arm still? | Arm frozen at a non-vertical pose |
| [torque-profile](torque-profile/) | Dynamic inverse — torques for a prescribed sinusoidal trajectory | Arm following trajectory, HUD shows torques |
| [forward-replay](forward-replay/) | Round-trip verification — replay inverse torques through forward sim | Two arms: ghost (prescribed) vs solid (motor-driven) |
| [jacobian](jacobian/) | Velocity mapping — `v = J * qdot`, stale vs correct Jacobian | Two arms with green/red velocity arrows |
| [stress-test](stress-test/) | Headless validation of all concepts (14 checks) | Console only |

## Key APIs

```rust
// Inverse dynamics: what forces produce the current accelerations?
data.inverse(&model);
// Output: data.qfrc_inverse (length nv)

// Site Jacobian: 3×nv translational + 3×nv rotational
let (jacp, jacr) = sim_core::mj_jac_site(&model, &data, site_id);

// Cartesian velocity from Jacobian
// v_linear = jacp * qvel
```

## Run

```bash
cargo run -p example-inverse-gravity-compensation --release
cargo run -p example-inverse-torque-profile --release
cargo run -p example-inverse-forward-replay --release
cargo run -p example-inverse-jacobian --release
cargo run -p example-inverse-stress-test --release
```
