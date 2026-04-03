# Control Design — LQR from Linearization

The "why" of linearization: produce A and B matrices, feed them to a standard
control design algorithm, and get a controller that balances an unstable
system. A cart-pole is linearized at the upright equilibrium, discrete-time LQR
solves the Riccati equation for optimal gains K, and the closed-loop controller
keeps the pole vertical.

## What you see

- **Yellow cart** on a rail, **blue pole** with **red tip** pointing upward
- Frozen at exact upright for 3 seconds while you read the K gains on the HUD
- Released with a 0.2 rad (~11 deg) tilt — the cart immediately starts
  correcting, sliding left/right to keep the pole balanced
- The pole stays vertical indefinitely — the LQR controller works
- HUD shows K gains, pole deviation, state error norm, and control force

## Physics

Linearization at the upright equilibrium gives:

```
dx_{t+1} = A * dx_t + B * du_t     (A: 4x4, B: 4x1)
```

State vector: `x = [cart_pos, pole_angle, cart_vel, pole_vel]` (upright = 0).
Control: `u = cart_force` (motor on the slide joint, gear=1).

The discrete algebraic Riccati equation (DARE) finds the optimal gain:

```
K = (R + B^T P B)^{-1} B^T P A
```

where P satisfies the DARE with cost matrices Q (state penalty) and R (control
penalty). At each timestep: `ctrl = -K * x_error`.

| Parameter | Value |
|-----------|-------|
| DOF | 2 (slide + hinge) |
| Actuators | 1 (motor on slide) |
| State dim | 4 |
| Cart mass | 1.0 kg |
| Pole mass | 0.5 kg (CoM at 0.5 m) |
| Cart damping | 1.0 N·s/m |
| Timestep | 2 ms |
| Q | diag(1, 100, 1, 10) |
| R | 0.01 |
| Initial tilt | 0.2 rad (~11 deg) |

The Q matrix heavily penalizes pole angle (100x) because that's what we care
about most. R is cheap (0.01) — we're happy to use force to keep the pole up.

## Validation

Four automated checks at startup (headless, before visual sim):

| Check | Expected | Threshold |
|-------|----------|-----------|
| **A is 4x4, B is 4x1** | 2*nv x 2*nv, 2*nv x nu | exact |
| **Balanced after 500 steps** | pole within +/-0.1 rad under LQR | 0.1 rad |
| **K has 4 elements** | one gain per state dimension | exact |
| **Falls without control** | pole exceeds pi/4 in 100 steps (ctrl=0) | pi/4 |

## Run

```
cargo run -p example-derivatives-control-design --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
