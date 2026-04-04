# sim-cpu — CPU Physics Fundamentals

Standalone visual examples covering every major subsystem of the CortenForge
CPU physics engine. Each example runs in a Bevy window with an orbit camera
and prints a validation report to the console. 140 examples across 25 domains.

## Domains

| Domain | Examples | What it covers |
|--------|----------|---------------|
| [`hinge-joint/`](hinge-joint/) | 2 | Revolute joints — simple + double pendulum |
| [`ball-joint/`](ball-joint/) | 4 | Spherical joints — unlimited, cone limit, orbits |
| [`slide-joint/`](slide-joint/) | 2 | Prismatic joints — horizontal + vertical spring-mass |
| [`free-joint/`](free-joint/) | 4 | 6-DOF bodies — projectile, tumble, spinning toss |
| [`joint-limits/`](joint-limits/) | 4 | Hinge, slide, ball-cone limits |
| [`sensors/`](sensors/) | 9 | One example per sensor type (or natural pair) |
| [`sensors-advanced/`](sensors-advanced/) | 8 | Force-torque, frame vel/acc, rangefinder, subtree |
| [`actuators/`](actuators/) | 10 | One example per actuator concept |
| [`integrators/`](integrators/) | 7 | All 5 integrators + comparison benchmarks |
| [`solvers/`](solvers/) | 5 | All 3 constraint solvers + comparison benchmarks |
| [`equality-constraints/`](equality-constraints/) | 8 | Weld, connect, distance, joint coupling |
| [`contact-tuning/`](contact-tuning/) | 7 | Friction, condim, solref, solimp, margin/gap, pair override |
| [`contact-filtering/`](contact-filtering/) | 4 | Bitmask, exclude pairs, ghost layers |
| [`inverse-dynamics/`](inverse-dynamics/) | 5 | Gravity-comp, torque-profile, forward-replay, jacobian |
| [`energy-momentum/`](energy-momentum/) | 5 | Free-flight conservation, PE/KE exchange, bounce, damping |
| [`muscles/`](muscles/) | 5 | Activation dynamics, force-length, cocontraction |
| [`tendons/`](tendons/) | 8 | Fixed coupling, spatial path, cylinder/sphere wrap, pulleys |
| [`passive-forces/`](passive-forces/) | 5 | Spring-damper tuning, fluid/ellipsoid drag |
| [`derivatives/`](derivatives/) | 8 | FD linearization, LQR, hybrid vs FD, sensor Jacobians, inverse dynamics, convergence, V-curve |
| [`raycasting/`](raycasting/) | 4 | Basic shapes, heightfield, scene query |
| [`keyframes/`](keyframes/) | 3 | Save-restore, multi-body |
| [`mocap-bodies/`](mocap-bodies/) | 4 | Drag target, push object, tilt-drop |
| [`sleep-wake/`](sleep-wake/) | 4 | Sleep settle, wake on contact, island groups |
| [`composites/`](composites/) | 4 | Cable composites — hanging, catenary, loaded, stress-test |
| [`urdf-loading/`](urdf-loading/) | 10 | URDF→Model pipeline: all joint types, geometry, mimic, errors |
| [`multi-scene-test/`](multi-scene-test/) | 1 | Multi-scene infrastructure test |

## How to use

Each example is a standalone Cargo binary:

```
cargo run -p example-hinge-joint-pendulum --release
cargo run -p example-sensor-accelerometer --release
cargo run -p example-actuator-motor --release
cargo run -p example-derivatives-linearize-pendulum --release
```

Start with the joint examples (simplest physics), then work through sensors,
actuators, integrators, and solvers. Derivatives and raycasting are more
advanced topics.

Orbit: left-drag | Pan: right-drag | Zoom: scroll
