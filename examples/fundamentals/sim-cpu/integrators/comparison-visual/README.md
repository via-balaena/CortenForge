# Integrator Comparison — Visual (Double Pendulum)

Three identical double pendulums side by side, each using a different
integrator. The double pendulum is a chaotic system — extreme sensitivity
to initial conditions means that tiny numerical differences from the
integrator compound exponentially. Within seconds, three simulations that
started identically are doing completely different things.

## What you see

- **Euler** (orange tip) — semi-implicit Euler, first-order
- **RK4** (blue tip) — 4th-order Runge-Kutta, gold standard
- **ImplicitSpringDamper** (gold tip) — diagonal implicit, different code path

All three start from the same initial angles (120 deg + 90 deg) and begin
swinging in perfect unison. Within 5-10 seconds they desync completely —
different positions, different velocities, completely different motion.

## Why they diverge

The double pendulum has two coupled rotating joints. Each integrator makes
slightly different numerical errors at each timestep:

- **Euler** is first-order: local error O(h^2), global error O(h)
- **RK4** is fourth-order: local error O(h^5), global error O(h^4)
- **ImplicitSpringDamper** takes a different code path through the solver

On a non-chaotic system (single pendulum) these differences are sub-pixel.
On a chaotic system the differences grow exponentially — the Lyapunov
exponent of the double pendulum is positive, meaning any perturbation
(including numerical error) doubles roughly every second.

After ~10 doublings the trajectories are completely uncorrelated. This
happens in about 10 seconds at dt = 0.005.

## What to watch in the HUD

- **E** — total mechanical energy (should be conserved in the true solution)
- **dE** — energy drift from initial value

RK4's dE stays closest to zero. Euler and ImplSpDmp drift more because
their lower-order errors accumulate faster. The *trajectories* diverge
from chaos, but the *energy drift* tells you which integrator is most
faithful to the true physics.

## Parameters

| Parameter | Value |
|-----------|-------|
| Link 1 | mass=1.0 kg, length=1.0 m |
| Link 2 | mass=0.5 kg, length=0.7 m |
| Damping | 0 (undamped) |
| Initial angles | 120 deg, 90 deg |
| Timestep | 0.005 s |

## Why not 5 pendulums?

On this scene, Euler, Implicit, and ImplicitFast produce nearly identical
trajectories because the implicit methods degenerate to Euler-like behavior
with zero stiffness/damping. Only RK4 and ImplicitSpringDamper are visually
distinct. Three pendulums keeps the scene clean and each one tells a
different story.

The headless `comparison` example runs all 5 integrators and prints the
full numerical table.

## Run

```
cargo run -p example-integrator-comparison-visual --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
