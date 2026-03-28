# Hinge Joint Examples

Revolute (hinge) joints — the simplest joint type. One rotational DOF,
scalar angle and angular velocity.

| Example | What it shows |
|---------|---------------|
| [`simple-pendulum/`](simple-pendulum/) | Single rod + tip mass, undamped. Validates oscillation period against analytical formula and energy conservation. |
| [`double-pendulum/`](double-pendulum/) | Two-link chain, chaotic dynamics. Validates energy conservation over 30s of non-periodic motion. |

## Key concepts

- `type="hinge"` — 1 qpos (angle), 1 qvel (angular velocity)
- `axis="0 1 0"` — rotation axis in parent frame
- `damping="0"` — undamped for energy conservation testing
- RK4 integrator at dt=1ms for high-accuracy reference

## Run

```
cargo run -p example-hinge-joint-pendulum --release
cargo run -p example-hinge-joint-double-pendulum --release
```
