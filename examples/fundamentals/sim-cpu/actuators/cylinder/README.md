# Cylinder — Double-Acting Pneumatic Piston

A pneumatic cylinder that extends and retracts under pressure control. Positive
pressure pushes the piston out; vacuum (negative pressure) sucks it back. No
springs — just air pressure, viscous damping from the fluid, and the physical
end-of-stroke limits of the cylinder bore.

## What you see

- **Semi-transparent hollow housing** with solid end caps — the cylinder bore
- **Green piston barrel** sliding inside the housing
- **Silver rod** extending through a hole in the right end cap
- Pressure alternates between +1 (extend) and -1 (retract) every 3 seconds
- The piston pumps back and forth, with visible delay from the activation filter
- Damping smooths the motion — the piston decelerates as it approaches each end

## Physics

The `<cylinder>` shortcut combines all three actuator components:

```
force = area * activation + bias[0] + bias[1]*position + bias[2]*velocity
      = 0.196 * act       + 0       + 0               - 0.5 * vel
```

- **Gain**: `GainType::Fixed(area)` — force = pressure * piston area.
  Area is computed from diameter: `pi/4 * 0.5^2 = 0.196 m^2`.
- **Bias**: `BiasType::Affine(0, 0, -0.5)` — viscous fluid damping only.
  No spring (bias[1] = 0). Damping resists motion proportional to velocity.
- **Dynamics**: `ActuatorDynamics::Filter(tau=0.5)` — Euler-approximated
  first-order filter. Pressure doesn't arrive instantly; activation ramps
  toward the command with time constant 0.5s.

The Filter dynamics here uses Euler integration (discrete step), distinct from
the FilterExact (analytical solution) in example 5 (activation-filter). Same
physics, different numerical method — this validates the Euler code path.

| Parameter | Value |
|-----------|-------|
| Cylinder diameter | 0.5 m |
| Piston area | 0.196 m^2 |
| Filter time constant | 0.5 s |
| Viscous damping | 0.5 N*s/m |
| Piston mass | 1.0 kg |
| Stroke limits | +/- 0.25 m |
| Pressure cycle | +1/-1 square wave, 6s period |
| Integrator | RK4, dt = 1 ms |

**Key distinction from activation-filter:** that example uses a position servo
with FilterExact. This one uses a cylinder with raw pressure force, Euler
Filter, and affine bias — testing three different code paths simultaneously.

## Expected console output

```
t=  1.0s  act=0.866  x=0.0375m  F=0.1237
t=  3.0s  act=0.931  x=0.2000m  F=0.1828
t=  5.0s  act=-0.966 x=0.0718m  F=-0.1042
t=  8.0s  act=0.965  x=-0.0344m F=0.0972
```

Activation lags the pressure command by ~0.5s. Position oscillates between the
stroke limits as pressure and vacuum alternate.

## Validation

Four automated checks at t=15s:

| Check | Expected | Threshold |
|-------|----------|-----------|
| **Activation exists** | `model.na == 1` | exact |
| **Filter dynamics** | act(tau) = 0.632 | < 5% |
| **Force formula** | force = area*act + bias terms | < 2% |
| **Area from diameter** | gainprm = pi/4 * d^2 | < 1e-10 |

## Run

```
cargo run -p example-actuator-cylinder --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
