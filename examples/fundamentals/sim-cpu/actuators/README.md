# Actuator Examples

CPU physics actuator fundamentals — one example per actuator concept. Each
example runs for 15-17 seconds and prints a validation report to the console.

| Example | Concept | Gain / Bias / Dynamics | Transmission |
|---------|---------|----------------------|--------------|
| `motor/` | Direct torque | Fixed gain, no bias | Joint |
| `position-servo/` | Position tracking | Fixed gain, affine bias (kp) | Joint |
| `velocity-servo/` | Velocity tracking | Fixed gain, affine bias (kv) | Joint |
| `damper/` | Viscous braking | Affine gain (-kv), no bias | Joint |
| `activation-filter/` | Smooth activation | Fixed gain, no bias, Filter | Joint |
| `cylinder/` | Pneumatic piston | Fixed gain, affine bias | Joint |
| `integrator/` | Integrated control | Fixed gain, affine bias, Integrator | Joint |
| `gear-and-limits/` | Gear ratio + clamping | Fixed gain, no bias | Joint (gear) |
| `site-transmission/` | Cartesian wrench | Fixed gain, no bias | Site |
| `slider-crank/` | Mechanical linkage | Fixed gain, no bias | SliderCrank |

## Progression

Start with **motor** (simplest — ctrl maps directly to torque) and work down.
Each example introduces one new concept:

1. **motor** — constant torque, no shaping
2. **position-servo** — bias term creates a spring (kp)
3. **velocity-servo** — bias term creates a velocity target (kv)
4. **damper** — affine gain makes force velocity-dependent
5. **activation-filter** — dynamics filter smooths the control signal
6. **cylinder** — affine bias models a pneumatic actuator
7. **integrator** — dynamics integrate the control signal over time
8. **gear-and-limits** — gear ratio scales force, clamps limit output
9. **site-transmission** — force at a site, configuration-dependent moments
10. **slider-crank** — linkage geometry, varying moment arm, dead centers

## Run

```
cargo run -p example-actuator-motor --release
cargo run -p example-actuator-position-servo --release
cargo run -p example-actuator-velocity-servo --release
cargo run -p example-actuator-damper --release
cargo run -p example-actuator-activation-filter --release
cargo run -p example-actuator-cylinder --release
cargo run -p example-actuator-integrator --release
cargo run -p example-actuator-gear-and-limits --release
cargo run -p example-actuator-site-transmission --release
cargo run -p example-actuator-slider-crank --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
