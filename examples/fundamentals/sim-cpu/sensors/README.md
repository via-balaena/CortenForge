# Sensor Examples

CPU physics sensor fundamentals — one example per sensor type (or natural pair).
Each example runs for 15 seconds and prints a validation report to the console.

| Example | Sensors | Pipeline Stage | Model |
|---------|---------|---------------|-------|
| `clock/` | Clock | Position | Hinge pendulum |
| `joint-pos-vel/` | JointPos, JointVel | Position + Velocity | Hinge pendulum |
| `frame-pos-quat/` | FramePos, FrameQuat | Position | Conical pendulum (ball joint) |
| `subtree-com/` | SubtreeCom | Position | Two-link arm (2× hinge) |
| `gyro-velocimeter/` | Gyro, Velocimeter | Velocity | Conical pendulum (ball joint) |
| `accelerometer/` | Accelerometer | Acceleration | Free-joint box drop |
| `touch/` | Touch | Acceleration | Free-joint sphere drop |
| `actuator-force/` | ActuatorFrc, JointActuatorFrc | Acceleration | Hinge + motor |
| `geom-distance/` | GeomDist, GeomNormal, GeomFromTo | Position (GJK) | Slide joint + spring |

## Not covered here

- **BallQuat, BallAngVel** — see [`ball-joint/spherical-pendulum`](../ball-joint/spherical-pendulum/)
