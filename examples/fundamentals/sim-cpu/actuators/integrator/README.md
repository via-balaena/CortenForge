# Integrator — Position Control from Velocity Input

A general actuator with integrator dynamics: the control signal is a rate
command, and the activation is its integral over time. Combined with a
position-servo bias, this gives position control from a velocity-like input —
you steer the arm by commanding angular rate, and it holds wherever you stop.

This is the third dynamics type (after FilterExact in example 5 and Filter in
example 6). It also uses the `<general>` actuator element directly, testing the
most flexible actuator specification path.

## What you see

- **Metal rod** with a **purple tip sphere** — arm viewed head-on, with gravity
- Three distinct phases:
  1. **Ramp up** (0-3s): ctrl=+1, activation climbs linearly, arm swings to +90 degrees
  2. **Hold** (3-8s): ctrl=0, activation freezes — the arm stays at 90 degrees
     for 5 full seconds with zero input. This is integrator memory.
  3. **Ramp down** (8-13s): ctrl=-1, activation decreases, arm swings to -90 degrees
- The arm holds ~3 degrees below the target — that's gravity pulling on it,
  balanced by the position servo

## Physics

The `<general>` actuator with `dyntype="integrator"`:

```
act_dot = ctrl                         (integrator: activation = integral of ctrl)
force   = 50 * act - 50 * theta - 5 * velocity   (position servo bias)
        = 50 * (act - theta) - 5 * vel
```

The activation acts as a position target. The gain (50) and bias spring (-50)
create a position servo, while the bias damping (-5) prevents oscillation.
The integrator dynamics mean you command the *rate of change* of the target,
not the target itself.

With ctrl=1 for t seconds: act = t (linear ramp). The arm follows act as its
target position. When ctrl returns to 0, act freezes — the integral holds its
value. This is the key property: **integrator memory**.

The actrange [-1.57, 1.57] rad (= +/-90 degrees) clamps the activation,
preventing unbounded growth.

| Parameter | Value |
|-----------|-------|
| Gain | 50 (position spring) |
| Bias | [0, -50, -5] (spring + damping) |
| Dynamics | Integrator (act_dot = ctrl) |
| Activation range | [-1.57, 1.57] rad (+/-90 deg) |
| Body mass | 1.0 kg |
| Arm length | 0.5 m (CoM at 0.25 m) |
| Gravity | 9.81 m/s^2 |
| Integrator | RK4, dt = 1 ms |

**Key distinction from filter examples:** filters track ctrl with exponential
lag. The integrator accumulates ctrl over time — fundamentally different
dynamics. A filter forgets; an integrator remembers.

## Expected console output

```
t=  1.0s  act=57.5  theta=49.8  [ramp]
t=  3.0s  act=90.0  theta=87.1  [hold]
t=  5.0s  act=90.0  theta=87.1  [hold]
t=  9.0s  act=30.7  theta=34.6  [down]
t= 12.0s  act=-90.0 theta=-87.1 [down]
```

Activation ramps linearly during phases 1 and 3. During the hold phase,
activation is rock-solid at 90.0 degrees — zero drift.

## Validation

Four automated checks at t=15s:

| Check | Expected | Threshold |
|-------|----------|-----------|
| **Linear ramp** | act(1s) = 1.0 rad | < 5% |
| **Hold at zero ctrl** | act(4s) = act(7s) | diff < 0.01 |
| **Activation clamping** | act in [-1.57, 1.57] | exact |
| **act_dot == ctrl** | Integrator dynamics exact | < 1e-10 |

## Run

```
cargo run -p example-actuator-integrator --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
