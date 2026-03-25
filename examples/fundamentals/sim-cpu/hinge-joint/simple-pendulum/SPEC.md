# Simple Pendulum — Spec

## Overview

A single rigid rod with a tip mass, hanging from a hinge joint. Displaced from
vertical and released, it swings as a simple pendulum. This is the **hinge
joint** counterpart to the ball-joint and slide-joint example families.

The simplest rotational system: 1 body, 1 hinge, 1 DOF.

## What it teaches

1. **Hinge joint** — `type="hinge"`, scalar angle (1 qpos, 1 qvel), axis vector
2. **Small-angle period** — `T ≈ 2π√(L/g)` for small oscillations; measurable
   deviation at larger amplitudes
3. **Energy conservation** — undamped system, RK4 integrator should conserve
   energy to high precision

## Layout (Z-up in MuJoCo, Y-up in Bevy)

```
    ╔═══╗        ← pivot bracket (visual box)
    ║   ║
    ╚═╤═╝
      │          ← rigid rod (capsule)
      │
      ●          ← tip mass (sphere)
```

## MJCF Model

```xml
<mujoco model="simple_pendulum">
    <compiler angle="radian"/>
    <option gravity="0 0 -9.81" timestep="0.001" integrator="RK4"/>

    <default>
        <geom contype="0" conaffinity="0"/>
    </default>

    <worldbody>
        <!-- Pivot bracket (visual) -->
        <geom name="bracket" type="box" size="0.06 0.04 0.03"
              pos="0 0 0" rgba="0.45 0.45 0.48 1"/>

        <!-- Pendulum arm -->
        <body name="arm" pos="0 0 0">
            <joint name="hinge" type="hinge" axis="0 1 0"
                   damping="0"/>
            <inertial pos="0 0 -1.0" mass="1.0" diaginertia="0.01 0.01 0.01"/>
            <geom name="rod" type="capsule" size="0.02"
                  fromto="0 0 0  0 0 -1.0" rgba="0.50 0.50 0.53 1"/>
            <geom name="mass" type="sphere" size="0.06"
                  pos="0 0 -1.0" rgba="0.82 0.22 0.15 1"/>
        </body>
    </worldbody>
</mujoco>
```

### Parameter choices

| Parameter | Value | Rationale |
|-----------|-------|-----------|
| Mass | 1.0 kg | Standard reference mass |
| Rod length | 1.0 m | Clean analytical period |
| Inertial pos | 0 0 -1.0 | CoM at tip — approximates ideal point-mass pendulum |
| Diaginertia | 0.01 isotropic | Small (compact sphere), keeps effective L ≈ 1.0 m |
| Damping | 0 | Undamped — tests energy conservation |
| Axis | 0 1 0 (Y in MuJoCo) | Swings in the XZ plane |
| Integrator | RK4 | Best energy conservation for undamped |
| Initial angle | 30° (π/6) | Large enough to see, small enough for period check |

### Derived quantities

The inertial is at the tip (d=1.0 m) with small diaginertia (0.01), making
this a near-ideal point-mass pendulum:

- `I_pivot = I_cm + m*d² = 0.01 + 1.0*1.0² = 1.01`
- Physical pendulum period: `T₀ = 2π√(I_pivot/(m*g*d)) = 2π√(1.01/9.81) = 2.0161 s`
- With amplitude correction for θ₀=30°: `T ≈ T₀(1 + θ₀²/16) = 2.0506 s`
- Compare textbook point mass: `T = 2π√(1.0/9.81) = 2.0061 s` (0.5% shorter)

We use the physical pendulum formula (not the point-mass approximation) as the
validation target — it accounts for the finite diaginertia.

## Visual Design

- **Red tip sphere** — matches the horizontal slide-joint color palette
- **Steel rod** — capsule, polished steel material
- **Metallic bracket** — brushed metal at the pivot point
- **Pivot sphere** — dark ball-and-socket marker at the hinge
- Camera: slightly offset side view to see the swing plane

## Validation Checks (at t=15s)

| Check | Expected | Threshold |
|-------|----------|-----------|
| **Period** | `T ≈ 2.051 s` (physical pendulum + amplitude correction) | < 2% error |
| **Energy** | Conserved (undamped, RK4) | < 0.5% drift |

## File Structure

```
examples/fundamentals/sim-cpu/hinge-joint/
├── BUGS.md
└── simple-pendulum/
    ├── Cargo.toml      ← example-hinge-joint-pendulum
    ├── src/main.rs
    └── README.md
```

Leaves room for future sub-examples (e.g. `double-pendulum/`, `damped/`).

## Migration

1. Delete `pendulum-sim/` (old, no validation, uses deprecated API patterns)
2. Remove `pendulum-sim` from workspace `Cargo.toml`
3. Add `hinge-joint/simple-pendulum` to workspace `Cargo.toml`

## Implementation Notes

1. **Period measurement**: The `PeriodTracker` measures zero crossings of the
   angular position. For a pendulum starting at +30°, it crosses zero twice
   per period — the tracker already handles this correctly (it counts
   positive→negative crossings only = one per period).

2. **Amplitude-corrected period**: For a pendulum with initial angle θ₀, the
   exact period is `T = T₀ · K(sin(θ₀/2)) · 2/π` where K is the complete
   elliptic integral. For θ₀=30°, the correction is small (~0.4%).
   We can use the series approximation `T ≈ T₀(1 + θ₀²/16)` which is
   accurate to ~0.01% at 30°.

3. **No springs to animate** — simpler than the slide-joint examples. Just
   MJCF geoms + pivot marker.
