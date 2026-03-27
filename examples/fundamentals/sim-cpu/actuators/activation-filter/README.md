# Activation Filter — Position Servo with Delayed Response

A position servo that doesn't respond instantly. Adding `timeconst` introduces
a first-order activation filter between the command and the force computation.
The servo sees `activation`, not `ctrl` — and activation lags behind the
command with exponential convergence.

This is the first actuator type with internal state (`data.act`). All previous
examples (motor, position-servo, velocity-servo, damper) have no dynamics —
ctrl maps to force immediately. The activation filter models real-world delays:
muscle activation, hydraulic valve response, motor driver ramp-up.

## What you see

- **Metal rod** with a **gold tip sphere** — a single-link arm viewed head-on
- The arm hangs for 1 second, then starts swinging side to side
- The target alternates between +45 and -45 degrees every 3 seconds (square
  wave), but the arm moves in **smooth curves**, not sharp jumps
- Watch the HUD: `target` jumps instantly, `activation` ramps smoothly — that
  gap is the filter in action
- The arm never reaches the full +/-45 degrees because the target flips before
  activation catches up — the filter attenuates the square wave

## Physics

The `<position>` actuator with `timeconst=1.0` adds `FilterExact` dynamics:

```
act_dot = (ctrl - act) / tau       (first-order ODE)
act(t)  = ctrl * (1 - e^(-t/tau))  (step response)
```

The servo force is then computed from activation, not ctrl:

```
force = kp * (activation - theta) - kd * velocity
```

This is a **low-pass filter** applied to the command signal. High-frequency
changes in ctrl are smoothed out. The time constant tau controls how fast
activation tracks the command:

- At t = tau (1.0s): activation reaches 63% of the step
- At t = 3*tau (3.0s): activation reaches 95% of the step

With the square wave toggling every 3 seconds, activation reaches ~95% before
each flip — close but never quite catching up. The arm traces smooth S-curves
while the command makes sharp steps.

| Parameter | Value |
|-----------|-------|
| Time constant (tau) | 1.0 s |
| Target amplitude | 45 degrees |
| Square wave period | 6.0 s (3s per half) |
| Position gain (kp) | 100 |
| Damping ratio | 1.0 (critically damped) |
| Body mass | 1.0 kg |
| Arm length | 0.5 m (CoM at 0.25 m) |
| Integrator | RK4, dt = 1 ms |

**Key distinction from position-servo:** the position-servo snaps to its target
in ~0.3 seconds. This filtered version takes ~3 seconds because activation has
to ramp through the filter first. Same servo, same gains — the only difference
is the activation dynamics.

## Expected console output

```
t=  1.0s  act=0.0°   theta=0.0°   lag=0.0°
t=  2.0s  act=28.6°  theta=27.0°  lag=1.6°
t=  5.0s  act=-13.5° theta=-11.4° lag=-2.1°
t=  8.0s  act=14.5°  theta=12.5°  lag=2.1°
```

Activation ramps smoothly while target jumps. The arm oscillates continuously —
the demo never settles.

## Validation

Four automated checks at t=15s:

| Check | Expected | Threshold |
|-------|----------|-----------|
| **Activation exists** | `model.na == 1`, `data.act` populated | exact |
| **Filter response at tau** | act(1s) = 63.2% of target | < 5% |
| **Filter response at 3*tau** | act(3s) = 95.0% of target | < 3% |
| **act_dot correct** | act_dot = (ctrl - act) / tau | < 2% |

## Run

```
cargo run -p example-actuator-activation-filter --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
