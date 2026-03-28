# Integrators — Energy Drift Comparison

Runs all 5 integrators on the same undamped pendulum and compares energy
conservation. This is the definitive test: identical physics, identical
initial conditions, only the time-stepping method differs.

## What it does

Creates 5 independent simulations — one per integrator — and steps each
for 15 seconds (3000 steps at dt = 0.005). Measures energy drift and
oscillation period, then prints a comparison table.

No window — headless analysis. The output is the table.

## The 5 integrators

| Integrator | Method | Order |
|-----------|--------|-------|
| **Euler** | Semi-implicit (symplectic) Euler | 1st |
| **RK4** | Classic 4-stage Runge-Kutta | 4th |
| **Implicit** | Full implicit with Coriolis, LU factorization | 1st (L-stable) |
| **ImplicitFast** | Symmetric D, no Coriolis, Cholesky | 1st (L-stable) |
| **ImplicitSpringDamper** | Diagonal spring/damper, Cholesky | 1st (L-stable) |

## What to expect

```
=== Integrator Comparison (t = 15s, dt = 0.005) ===
  Drift normalized by m*g*d = 2.4525 J

  Integrator              Drift(%mgd)
  ───────────────────────────────────
  Euler                    +0.24%      ← visible drift (energy gain)
  RK4                      +0.00%      ← near-perfect conservation
  Implicit                 +0.24%      ← same as Euler (zero K/D)
  ImplicitFast             +0.24%      ← same as Euler (zero K/D)
  ImplSpDmp                +0.36%      ← slightly worse (different path)
```

**Key insight:** On this zero-damping, zero-stiffness pendulum, the three
implicit integrators degenerate to Euler-like behavior because there are no
springs or dampers to treat implicitly. The implicit methods shine on stiff
systems (strong springs, high damping) — not on a free pendulum. RK4 is the
clear winner for accuracy on smooth problems.

## Why drift is normalized by m*g*d

The pendulum starts horizontal (theta = 90 degrees), so the center of mass
is at z = 0 and total mechanical energy E_0 is approximately zero. Dividing
drift by E_0 would give infinity. Instead we normalize by the characteristic
energy scale: `m * g * d = 1.0 * 9.81 * 0.25 = 2.4525 J` — the maximum
potential energy swing as the pendulum drops from horizontal to vertical.

## Scene

```
         o pivot (world)
         |
         | rod (0.5 m, capsule)
         |
         * tip (1.0 kg sphere)
```

Single hinge joint, zero damping, zero friction, gravity only. Released
from horizontal. Timestep dt = 0.005 (deliberately coarse to amplify Euler
drift over 15 seconds).

## Validation (7 checks)

| # | Check | Criterion |
|---|-------|-----------|
| 1 | Euler drifts visibly | \|drift\| > 0.1% of m*g*d |
| 2 | RK4 near-perfect | \|drift\| < 0.001% of m*g*d |
| 3 | Implicit stable | \|drift\| < 0.5% of m*g*d |
| 4 | ImplicitFast stable | \|drift\| < 0.5% of m*g*d |
| 5 | ImplSpDmp bounded | \|drift\| < 5% of m*g*d |
| 6 | RK4 >> Euler | RK4 drift < Euler drift / 10 |
| 7 | All periods match | Within 2% of RK4's measured period |

## Per-integrator examples

Each integrator has its own visual (Bevy window) example with algorithm
documentation. Run any in isolation:

```
cargo run -p example-integrator-euler --release
cargo run -p example-integrator-rk4 --release
cargo run -p example-integrator-implicit --release
cargo run -p example-integrator-implicit-fast --release
cargo run -p example-integrator-implicit-spring-damper --release
```

## Run

```
cargo run -p example-integrator-comparison --release
```
