# Ball Joint Examples

Spherical (ball) joints — 3 rotational DOF represented as unit quaternions.
4 qpos (quaternion), 3 qvel (angular velocity in body frame).

| Example | What it shows |
|---------|---------------|
| [`spherical-pendulum/`](spherical-pendulum/) | Unlimited ball joint, precessing elliptical path. Validates quaternion norm, energy conservation, BallQuat/BallAngVel sensor accuracy. |
| [`conical-pendulum/`](conical-pendulum/) | Circular orbit via initial angular velocity. Trail visualization shows 3D trajectory impossible with a hinge. |
| [`cone-limit/`](cone-limit/) | 45-degree cone limit, displaced beyond limit. Validates constraint enforcement, energy dissipation, quaternion norm. |
| [`cone-limit-orbit/`](cone-limit-orbit/) | Sliding along the cone boundary surface. Trail shows the constraint is a true 3D surface, not a 1D angular limit. |

## Key concepts

- `type="ball"` — quaternion representation (4 qpos, 3 qvel)
- Exponential map integration on SO(3) — no gimbal lock
- `limited="true" range="0 0.7854"` — cone limit in radians
- Cone limit is a 3D surface constraint, not per-axis limits
- Initial angular velocity requires body-frame decomposition

## Run

```
cargo run -p example-ball-joint-pendulum --release
cargo run -p example-ball-joint-conical --release
cargo run -p example-ball-joint-cone-limit --release
cargo run -p example-ball-joint-cone-orbit --release
```
