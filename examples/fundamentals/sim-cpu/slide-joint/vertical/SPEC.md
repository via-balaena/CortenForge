# Vertical Slide Joint — Spec

## Overview

A mass on a vertical spring — the gravitational counterpart to the existing
horizontal slide-joint example. A block slides along a vertical rail, suspended
by a spring from an overhead mount. Gravity pulls it down; the spring pulls it
back up. Released from rest below equilibrium, it oscillates as a damped
harmonic oscillator around the gravity-shifted equilibrium point.

## What it teaches (beyond the horizontal example)

The horizontal example has zero gravity influence (motion perpendicular to g).
This example shows:

1. **Gravity shifts the equilibrium** — rest position is `x_eq = -mg/k` below
   the natural length, not at qpos=0
2. **Period is unchanged** — gravity shifts *where* it oscillates, not *how fast*
   (`T = 2π√(m_eff/k)`, same formula)
3. **Joint stiffness as a spring** — the MJCF `stiffness` attribute models the
   spring restoring force; gravity is handled by the engine

## Layout (Z-up in MuJoCo, Y-up in Bevy)

```
    ┌─────┐  ← overhead mount (visual box)
    │█████│
    └──┬──┘
       │      ← spring (coil visual, stretches/compresses)
       │
    ┌──┴──┐
    │     │  ← sliding block (mass)
    │     │
    └─────┘
       │
       │      ← vertical rail (capsule, visual only)
       │
    ───┴───   ← ground plane (visual reference)
```

## MJCF Model

```xml
<mujoco model="vertical_slide_joint">
    <compiler angle="radian"/>
    <option gravity="0 0 -9.81" timestep="0.001" integrator="RK4"/>

    <default>
        <geom contype="0" conaffinity="0"/>
    </default>

    <worldbody>
        <!-- Vertical rail along Z axis -->
        <geom name="rail" type="capsule" size="0.02"
              fromto="0 0 -0.5  0 0 2.0" rgba="0.50 0.50 0.53 1"/>

        <!-- Overhead mount at top of rail -->
        <geom name="mount" type="box" size="0.10 0.10 0.02"
              pos="0 0 2.0" rgba="0.45 0.45 0.48 1"/>

        <!-- Ground plane (visual reference) -->
        <geom name="ground" type="plane" size="1.0 1.0 0.01"
              pos="0 0 -0.5" rgba="0.35 0.35 0.38 1"/>

        <!-- The sliding block -->
        <body name="block" pos="0 0 1.0">
            <joint name="slide" type="slide" axis="0 0 1"
                   stiffness="40" damping="0.3" armature="0.1"
                   limited="true" range="-1.5 1.0"/>
            <inertial pos="0 0 0" mass="1.0" diaginertia="0.01 0.01 0.01"/>
            <geom name="block" type="box" size="0.08 0.08 0.08"
                  rgba="0.15 0.45 0.82 1"
                  contype="1" conaffinity="1"/>
        </body>
    </worldbody>
</mujoco>
```

### Parameter choices

| Parameter | Value | Rationale |
|-----------|-------|-----------|
| Mass | 1.0 kg | Same as horizontal for comparison |
| Armature | 0.1 kg | Same as horizontal |
| Stiffness | 40 N/m | Higher than horizontal (20) so equilibrium shift `mg/k = 0.245 m` is moderate, not extreme |
| Damping | 0.3 Ns/m | Slightly more than horizontal (0.2) — visible decay within ~10s |
| Axis | 0 0 1 (Z in MuJoCo → Y in Bevy) | Vertical |
| Body pos | 0 0 1.0 | Natural anchor point (qpos=0 means block at z=1.0) |
| Range | -1.5 to 1.0 m | Asymmetric — block can drop further than it can rise |
| Initial qpos | -0.5 m | Below equilibrium, gives a visible oscillation |

### Derived quantities

- Effective mass: `m_eff = 1.0 + 0.1 = 1.1 kg`
- Equilibrium displacement: `x_eq = -m*g/k = -1.0*9.81/40 = -0.2453 m`
- Period: `T = 2π√(m_eff/k) = 2π√(1.1/40) = 1.0416 s`
- Initial offset from equilibrium: `x₀ - x_eq = -0.5 - (-0.2453) = -0.2547 m`

## Visual Design

- **Blue block** — contrasts with red horizontal block; signals "different example"
- **Single spring above** — coil from mount down to block top, stretches/compresses
- **Vertical rail** — capsule along Y (Bevy), visual only
- **Overhead mount** — small metallic box at top
- **Ground plane** — flat reference at bottom
- Camera: slightly elevated side view, offset to see the full vertical extent

## Validation Checks (at t=15s)

| Check | Expected | Threshold |
|-------|----------|-----------|
| **Period** | `T = 2π√(1.1/40) = 1.0416 s` | < 2% error |
| **Energy** | Monotonically decreasing | < 1e-6 J increase |
| **Limits** | qpos stays within [-1.5, 1.0] | < 1 mm violation |
| **Equilibrium** | Mean position converges to `x_eq = -0.2453 m` | < 5% error (measured over last 5s) |

The 4th check is new vs. the horizontal example — it validates that the engine
correctly computes the gravity-shifted equilibrium.

## File Structure

Restructured to match ball-joint pattern (`horizontal/` + `vertical/` subdirs):

```
examples/fundamentals/sim-cpu/slide-joint/
├── BUGS.md             ← shared (existing, stays at root)
├── horizontal/
│   ├── Cargo.toml      ← example-slide-joint-horizontal (renamed from example-slide-joint)
│   ├── src/main.rs     ← existing code, unchanged
│   └── README.md       ← existing, unchanged
└── vertical/
    ├── Cargo.toml      ← example-slide-joint-vertical
    ├── src/main.rs
    └── README.md
```

### Migration steps (horizontal)

1. `git mv` existing `Cargo.toml`, `src/`, `README.md` into `horizontal/`
2. Rename package `example-slide-joint` → `example-slide-joint-horizontal`
3. Update workspace `Cargo.toml` member path
4. Verify `cargo build -p example-slide-joint-horizontal` works

## Implementation Notes

1. **Spring visual**: single spring from mount to block top. Reuse `SpringCoilParams`
   and `spring_coil()` from `sim-bevy::mesh`. Spring extends along Y in Bevy
   (rotated 90° from horizontal example which extends along X).

2. **`EquilibriumTracker` in `sim-core::validation`**: reusable tracker that
   accumulates samples over a trailing window and compares mean position to an
   expected equilibrium value. API sketch:
   ```rust
   pub struct EquilibriumTracker { ... }
   impl EquilibriumTracker {
       pub fn new(window_duration: f64) -> Self;
       pub fn sample(&mut self, value: f64, time: f64);
       pub fn check(&self, expected: f64, tolerance_pct: f64) -> ValidationCheck;
   }
   ```
   `window_duration` controls how much trailing history to average over (e.g. 5s).
   The check computes `|mean - expected| / |expected|` and compares to
   `tolerance_pct / 100`.

3. **Z-up → Y-up**: MuJoCo Z axis maps to Bevy Y axis. The `sync_geom_transforms`
   system handles this automatically. Spring visual code needs to work in Bevy
   coordinates (Y-axis).

## Decisions

- ~~Restructure to subdirs?~~ **Yes** — matches ball-joint pattern.
- ~~Equilibrium tracker location?~~ **`sim-core::validation`** — reusable.
