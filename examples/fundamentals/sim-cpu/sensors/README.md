# Sensor Examples

CPU physics sensor fundamentals — one example per sensor type (or natural pair).

| Example | Sensors | Pipeline Stage | Status |
|---------|---------|---------------|--------|
| `clock/` | Clock | Position | Pending |
| `joint-pos-vel/` | JointPos, JointVel | Position + Velocity | Pending |
| `frame-pos-quat/` | FramePos, FrameQuat | Position | Pending |
| `subtree-com/` | SubtreeCom | Position | Pending |
| `gyro-velocimeter/` | Gyro, Velocimeter | Velocity | Pending |
| `accelerometer/` | Accelerometer | Acceleration | Pending |
| `touch/` | Touch | Acceleration | Pending |
| `actuator-force/` | ActuatorFrc, JointActuatorFrc | Acceleration | Pending |
| `geom-distance/` | GeomDist, GeomNormal, GeomFromTo | Position (GJK) | Pending |

## Not covered here

- **BallQuat, BallAngVel** — see [`ball-joint/spherical-pendulum`](../ball-joint/spherical-pendulum/)

## Spec

See [SENSOR_EXAMPLES_SPEC.md](SENSOR_EXAMPLES_SPEC.md) for full design rationale and
validation criteria.
