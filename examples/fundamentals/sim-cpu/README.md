# sim-cpu — CPU Physics Fundamentals

Standalone visual examples covering every major subsystem of the CortenForge
CPU physics engine. Each example runs in a Bevy window with an orbit camera
and prints a validation report to the console.

## Domains

| Domain | Examples | What it covers |
|--------|----------|---------------|
| [`hinge-joint/`](hinge-joint/) | 2 | Revolute joints — simple + double pendulum |
| [`ball-joint/`](ball-joint/) | 4 | Spherical joints — unlimited, cone limit, orbits |
| [`slide-joint/`](slide-joint/) | 2 | Prismatic joints — horizontal + vertical spring-mass |
| [`sensors/`](sensors/) | 9 | One example per sensor type (or natural pair) |
| [`actuators/`](actuators/) | 10 | One example per actuator concept |
| [`integrators/`](integrators/) | 7 | All 5 integrators + comparison benchmarks |
| [`solvers/`](solvers/) | 5 | All 3 constraint solvers + comparison benchmarks |
| [`multi-scene-test/`](multi-scene-test/) | 1 | Multi-scene infrastructure test |

## How to use

Each example is a standalone Cargo binary:

```
cargo run -p example-hinge-joint-pendulum --release
cargo run -p example-sensor-accelerometer --release
cargo run -p example-actuator-motor --release
```

Start with the joint examples (simplest physics), then work through sensors,
actuators, integrators, and solvers.

Orbit: left-drag | Pan: right-drag | Zoom: scroll
