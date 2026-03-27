# Gear & Limits — Gear Ratio and Force Clamping

Two side-by-side arms driven by the same control signal, comparing the effect
of gear ratio and force limits. Arm A (blue tip) has gear=1. Arm B (orange tip)
has gear=5 with a force clamp at 3.0.

The gear ratio is a torque multiplier: `joint_torque = gear * actuator_force`.
Same motor, longer lever. The force limit caps the actuator force before the
gear multiplies it — like a torque limiter or slip clutch.

## What you see

- Two arms spinning in zero gravity with viscous damping (terminal velocity)
- **Blue arm (A)**: gear=1, baseline — spins at a steady rate
- **Orange arm (B)**: gear=5 — visibly faster, 5x the torque for the same input

Two phases:

1. **Phase 1 (0-5s)**: ctrl=1.0 for both. B spins 5x faster (gear advantage).
   HUD shows vel ratio = 5.0x.
2. **Phase 2 (5-15s)**: ctrl=5.0 for both. A's force = 5.0 (unclamped). B's
   force = 3.0 (clamped at forcerange limit). B's torque = 5*3 = 15, A's
   torque = 1*5 = 5. The vel ratio drops from 5.0x to 3.0x — the clamp
   cut B's advantage.

## Physics

Both arms use `<motor>` actuators with identical inertia (0.0825 kg*m^2):

```
Arm A: torque = gear * force = 1 * ctrl     (unclamped)
Arm B: torque = gear * force = 5 * min(ctrl, 3.0)  (clamped at forcerange)
```

With joint damping (1.0 N*m*s/rad), each arm reaches terminal velocity:

```
terminal_vel = torque / damping
```

Phase 1 (ctrl=1.0):
- A: torque = 1, vel = 1.0 rad/s
- B: torque = 5, vel = 5.0 rad/s
- Ratio: 5.0x (pure gear advantage)

Phase 2 (ctrl=5.0):
- A: torque = 5, vel = 5.0 rad/s
- B: torque = 5 * 3.0 = 15, vel = 15.0 rad/s  (force clamped to 3.0)
- Without clamp, B would have: torque = 25, vel = 25.0 rad/s
- Ratio: 3.0x (clamp reduced B's advantage from 5x to 3x)

| Parameter | Arm A | Arm B |
|-----------|-------|-------|
| Gear ratio | 1 | 5 |
| Force range | unlimited | [-3, 3] |
| Joint damping | 1.0 N*m*s/rad | 1.0 N*m*s/rad |
| Effective inertia | 0.0825 kg*m^2 | 0.0825 kg*m^2 |
| Tip color | Blue | Orange |

**Key insight:** gear amplifies force, but the force limit applies BEFORE the
gear. So a gear=5 actuator with forcerange=3 produces max torque of 15 — not
the 25 it would produce without the clamp.

## Expected console output

```
t=  1.0s  fA=1.000  fB=1.000  vA=1.000  vB=5.000
t=  5.0s  fA=5.000  fB=3.000  vA=2.010  vB=7.524
t=  6.0s  fA=5.000  fB=3.000  vA=5.000  vB=15.000
```

Phase transition at t=5: force_B snaps from 1.0 to 3.0 (clamped), not 5.0.
Velocities settle within one time constant (~0.08s).

## Validation

Four automated checks at t=15s:

| Check | Expected | Threshold |
|-------|----------|-----------|
| **Gear scaling** | vel_B / vel_A = 5.0 at t=1s | < 5% |
| **Force identity** | force_A == ctrl (gain=1, no clamp) | < 1e-10 |
| **Force clamping** | force_B == 3.0 when ctrl=5 | < 1e-10 |
| **Torque = gear * force** | qfrc_B / qfrc_A = 3.0 in phase 2 | < 1% |

## Run

```
cargo run -p example-actuator-gear-and-limits --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
