# Sensor Examples

CPU physics sensor fundamentals — one example per sensor type (or natural pair).

| Example | Sensors | Pipeline Stage | Model | Status |
|---------|---------|---------------|-------|--------|
| `clock/` | Clock | Position | Hinge pendulum | PASS |
| `joint-pos-vel/` | JointPos, JointVel | Position + Velocity | Hinge pendulum | PASS |
| `frame-pos-quat/` | FramePos, FrameQuat | Position | Conical pendulum (ball joint) | PASS |
| `subtree-com/` | SubtreeCom | Position | Two-link arm (2× hinge) | PASS |
| `gyro-velocimeter/` | Gyro, Velocimeter | Velocity | Conical pendulum (ball joint) | PASS |
| `accelerometer/` | Accelerometer | Acceleration | Free-joint box drop | Pending |
| `touch/` | Touch | Acceleration | Free-joint sphere drop | Pending |
| `actuator-force/` | ActuatorFrc, JointActuatorFrc | Acceleration | Hinge + motor | Pending |
| `geom-distance/` | GeomDist, GeomNormal, GeomFromTo | Position (GJK) | Slide joint + spring | Pending |

## Not covered here

- **BallQuat, BallAngVel** — see [`ball-joint/spherical-pendulum`](../ball-joint/spherical-pendulum/)

## Spec

See [SENSOR_EXAMPLES_SPEC.md](SENSOR_EXAMPLES_SPEC.md) for full design rationale and
validation criteria.
