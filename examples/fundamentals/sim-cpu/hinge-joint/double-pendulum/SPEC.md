# Double Pendulum — Spec

## Overview

Two rigid links chained by hinge joints, released from a dramatic initial
angle. The system exhibits **deterministic chaos** — extreme sensitivity to
initial conditions. No closed-form period exists; energy conservation is the
primary validation target.

## What it teaches (beyond simple pendulum)

1. **Multi-body kinematic chain** — two bodies, two hinge joints, 2 DOF
2. **Chaotic dynamics** — visually dramatic, scientifically important
3. **Energy conservation under chaos** — the integrator must conserve energy
   even as the motion is wildly non-periodic

## Layout (Z-up in MuJoCo, Y-up in Bevy)

```
    ╔═══╗        ← pivot bracket (visual)
    ╚═╤═╝
      │          ← link 1 (capsule + tip sphere)
      │
      ●──┐
         │       ← link 2 (capsule + tip sphere)
         │
         ●
```

## MJCF Model

```xml
<mujoco model="double_pendulum">
    <compiler angle="radian"/>
    <option gravity="0 0 -9.81" timestep="0.001" integrator="RK4"/>

    <default>
        <geom contype="0" conaffinity="0"/>
    </default>

    <worldbody>
        <!-- Pivot bracket (visual) -->
        <geom name="bracket" type="box" size="0.06 0.04 0.03"
              pos="0 0 0" rgba="0.45 0.45 0.48 1"/>

        <!-- Link 1 -->
        <body name="link1" pos="0 0 0">
            <joint name="hinge1" type="hinge" axis="0 1 0" damping="0"/>
            <inertial pos="0 0 -0.5" mass="1.0" diaginertia="0.01 0.01 0.01"/>
            <geom name="socket1" type="sphere" size="0.035"
                  pos="0 0 0" rgba="0.35 0.33 0.32 1"/>
            <geom name="rod1" type="capsule" size="0.025"
                  fromto="0 0 0  0 0 -1.0" rgba="0.50 0.50 0.53 1"/>
            <geom name="joint_ball" type="sphere" size="0.03"
                  pos="0 0 -1.0" rgba="0.35 0.33 0.32 1"/>

            <!-- Link 2 -->
            <body name="link2" pos="0 0 -1.0">
                <joint name="hinge2" type="hinge" axis="0 1 0" damping="0"/>
                <inertial pos="0 0 -0.35" mass="0.5" diaginertia="0.005 0.005 0.005"/>
                <geom name="rod2" type="capsule" size="0.02"
                      fromto="0 0 0  0 0 -0.7" rgba="0.48 0.48 0.50 1"/>
                <geom name="mass2" type="sphere" size="0.05"
                      pos="0 0 -0.7" rgba="0.15 0.45 0.82 1"/>
            </body>
        </body>
    </worldbody>
</mujoco>
```

### Parameter choices

| Parameter | Value | Rationale |
|-----------|-------|-----------|
| Link 1 mass | 1.0 kg | Heavier upper link for interesting dynamics |
| Link 1 length | 1.0 m | Same as simple pendulum for comparison |
| Link 2 mass | 0.5 kg | Lighter lower link — more dramatic flipping |
| Link 2 length | 0.7 m | Shorter — visually distinct from link 1 |
| Damping | 0 (both) | Undamped — energy conservation test |
| Integrator | RK4 | Best for energy conservation |
| Initial angles | 120°, 90° | Large displacement — triggers chaotic regime quickly |

## Visual Design

- **Red link 1** — heavier upper rod with larger capsule radius
- **Blue tip sphere on link 2** — signals "this is the chaos" end
- **Dark joint balls** — cast iron at both pivot points
- **Metallic bracket** — brushed metal at the top mount
- Camera: side view, pulled back enough to see full extent of chaotic swings

## Validation Checks (at t=30s)

| Check | Expected | Threshold |
|-------|----------|-----------|
| **Energy** | Conserved (undamped, RK4) | < 0.5% drift |

Only one check — there is no analytical period for a double pendulum. Energy
conservation over 30 seconds of chaotic motion is a strong integrator test.

Longer validation window (30s vs 15s) because chaotic motion exercises more
of the integrator's state space.

## File Structure

```
examples/fundamentals/sim-cpu/hinge-joint/
├── BUGS.md
├── simple-pendulum/    ← existing
└── double-pendulum/
    ├── Cargo.toml      ← example-hinge-joint-double-pendulum
    ├── src/main.rs
    └── README.md
```

## Decisions

- **No period check** — chaotic systems have no repeating period
- **30s validation window** — longer than other examples to stress the integrator
- **Undamped** — matches simple pendulum; damped double pendulum could be a
  future sub-example
