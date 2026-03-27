# Integrators Example Spec

**Status:** Done — implemented 2026-03-27
**Date:** 2026-03-27
**Parent:** `examples/COVERAGE_SPEC.md` Track 1, item 7

## Goal

Exercise all 5 integrator implementations on the simplest possible scene
(single undamped pendulum), measuring energy drift as the primary accuracy
metric. No contacts, no actuators, no solver — the only variable is the
integrator.

## Why This Matters

Every existing example uses RK4. Euler, Implicit, ImplicitFast, and
ImplicitSpringDamper have **zero** visual examples. This is the first time
4 of the 5 integrators get exercised outside unit tests.

## Scene

Single-link pendulum: one hinge joint, one rod + tip mass, gravity,
zero damping, zero friction. Released from horizontal (θ₀ = π/2).

```
             ○ pivot (world)
             |
             | rod (capsule, L=0.5m)
             |
             ● tip (sphere, m=1.0kg)
```

Analytical reference: simple pendulum with period T = 2π√(L/g).
With L = 0.5m, g = 9.81 m/s²: T ≈ 1.419s.

## Integrators Under Test

| # | MJCF string             | Enum variant            | Method                              |
|---|-------------------------|-------------------------|-------------------------------------|
| 1 | `Euler`                 | `Euler`                 | Semi-implicit Euler + Eulerdamp     |
| 2 | `RK4`                   | `RungeKutta4`           | Classic 4-stage Runge-Kutta         |
| 3 | `implicit`              | `Implicit`              | Full implicit, Coriolis + LU        |
| 4 | `implicitfast`          | `ImplicitFast`          | Symmetric D, no Coriolis, Cholesky  |
| 5 | `implicitspringdamper`  | `ImplicitSpringDamper`  | Diagonal spring/damper, Cholesky    |

## Example Structure

**One example per integrator** — each independently runnable and debuggable.
Plus a headless comparison that runs all 5 and prints the results table.

```
integrators/
  euler/                    # cargo run -p example-integrator-euler
  rk4/                      # cargo run -p example-integrator-rk4
  implicit/                 # cargo run -p example-integrator-implicit
  implicit-fast/            # cargo run -p example-integrator-implicit-fast
  implicit-spring-damper/   # cargo run -p example-integrator-implicit-spring-damper
  comparison/               # cargo run -p example-integrator-comparison (headless table)
```

Each visual example is ~150 LOC, standalone, one Bevy window, one pendulum.
If an integrator blows up, you debug it in isolation — no 5-model tangle.

The `comparison/` example runs all 5 headless (no window), steps each for
15s, and prints the formatted comparison table + pass/fail checks. This is
the one that validates ordering and ratios.

### MJCF (identical for all, only integrator= differs)

Timestep `0.005` (deliberately coarse to make Euler drift visible).

```xml
<mujoco model="integrator-{name}">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.005" integrator="{name}">
    <flag energy="enable" contact="disable"/>
  </option>
  <default>
    <geom contype="0" conaffinity="0"/>
  </default>
  <worldbody>
    <body name="arm" pos="0 0 0">
      <joint name="hinge" type="hinge" axis="0 1 0" damping="0"/>
      <inertial pos="0 0 -0.25" mass="1.0" diaginertia="0.01 0.01 0.01"/>
      <geom name="rod" type="capsule" size="0.02"
            fromto="0 0 0  0 0 -0.5"/>
      <geom name="tip" type="sphere" size="0.05" pos="0 0 -0.5"/>
    </body>
  </worldbody>
  <sensor>
    <jointpos name="jpos" joint="hinge"/>
    <jointvel name="jvel" joint="hinge"/>
  </sensor>
</mujoco>
```

Initial condition: `qpos[0] = π/2` (horizontal), `qvel[0] = 0`.

### Timestep Choice

`dt = 0.005` is deliberate:
- Coarse enough that Euler shows visible energy drift over 20 seconds
- Fine enough that all implicit integrators remain stable
- RK4 stays excellent at this timestep (4th-order)

### Simulation Duration

20 seconds (~14 pendulum periods). Long enough to clearly separate
integrator accuracy tiers. Validation checks at t = 15s.

## Per-Example Visual

Each visual example (euler, rk4, implicit, implicit-fast, implicit-spring-damper)
shows:
- One swinging pendulum
- HUD with: integrator name, current energy, energy drift %, sim time
- Distinct color per integrator (metal preset)

### HUD (per example)

```
Integrator: RK4
Energy₀:    4.905 J
Energy:     4.905 J
Drift:      +0.00%
Time:       12.3s
```

## Comparison Example (headless)

The `comparison/` example is headless — no Bevy window. It:
1. Creates 5 Model+Data pairs (one per integrator)
2. Steps each for 15 seconds
3. Tracks energy and period zero-crossings
4. Prints formatted table + runs validation checks

### Console Output

```
=== Integrator Comparison (t = 15.0s, dt = 0.005) ===

  Integrator            Energy₀     Energy_now   Drift(%)   Period(s)  T_err(%)
  ─────────────────────────────────────────────────────────────────────────────
  Euler                  4.905       4.952       +0.96%      1.416     -0.21%
  RK4                    4.905       4.905       +0.00%      1.419     +0.00%
  Implicit               4.905       4.903       -0.04%      1.418     -0.07%
  ImplicitFast           4.905       4.903       -0.04%      1.418     -0.07%
  ImplicitSpringDamper   4.905       4.904       -0.02%      1.418     -0.07%

  Drift ordering: RK4 < ImplicitFast < Implicit < ImplSpDmp < Euler  ✓
  RK4/Euler ratio: 96.0x  ✓ (> 10x)

  PASS: 7/7 checks passed
```

## Pass/Fail Criteria

All criteria evaluated at t = 15s.

### Energy Conservation (primary metric)

| Integrator            | Max energy drift | Rationale                          |
|-----------------------|------------------|------------------------------------|
| Euler                 | > 0.5%           | Must show visible drift (that's the point) |
| RK4                   | < 0.01%          | 4th-order, gold standard           |
| Implicit              | < 0.5%           | Stable, slight dissipation OK      |
| ImplicitFast          | < 0.5%           | Stable, slight dissipation OK      |
| ImplicitSpringDamper  | < 0.5%           | Stable (no springs → reduces to Euler-like, but implicit damping path) |

Note: Euler energy drift is expected to be *positive* (energy gain for
semi-implicit Euler on a pendulum). The test checks `|drift| > 0.5%` to
confirm Euler is measurably worse than the implicit methods.

### Period Accuracy

All integrators: measured oscillation period within 2% of analytical
T = 2π√(L_eff/g) where L_eff accounts for the mass distribution.

### Ordering

The example must demonstrate the accuracy hierarchy:
```
RK4 >> Implicit ≈ ImplicitFast ≈ ImplicitSpringDamper >> Euler
```

Specifically: `|drift_RK4| < |drift_Implicit| < |drift_Euler|`
(with RK4 being at least 10× better than Euler).

## Validation Checks (7 total)

1. **Euler drifts visibly**: `|drift_euler| > 0.5%`
2. **RK4 near-perfect**: `|drift_rk4| < 0.01%`
3. **Implicit stable**: `|drift_implicit| < 0.5%`
4. **ImplicitFast stable**: `|drift_implicitfast| < 0.5%`
5. **ImplicitSpringDamper stable**: `|drift_implicitspringdamper| < 0.5%`
6. **RK4 >> Euler**: `|drift_rk4| < |drift_euler| / 10`
7. **All periods within 2%**: every integrator's measured T within 2% of analytical

## Implementation Notes

- Use `sim_mjcf::load_model_from_xml()` to build each model
- Set `data.qpos[0] = FRAC_PI_2` before first step
- Call `data.forward()` to initialize kinematics before first energy read
- Energy: `data.total_energy()` (requires `ENABLE_ENERGY` flag)
- Period measurement: track zero-crossings of `qpos[0]` (sign changes),
  compute average period from intervals between same-direction crossings
- Visual examples use standard sim-bevy infrastructure (PhysicsModel,
  PhysicsData, spawn_model_geoms, etc.)
- Comparison example uses sim-core + sim-mjcf only (no Bevy dependency)

## Non-Goals

- No spring/damper tuning (save for contact-tuning example)
- No multi-joint systems (single hinge is sufficient to expose drift)
- No actuators (pure passive dynamics)
- No contacts (pure free swinging)
