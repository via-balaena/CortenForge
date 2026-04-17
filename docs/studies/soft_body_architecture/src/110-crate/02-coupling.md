# Coupling boundaries

`sim-soft` sits at the center of five coupling boundaries, each with a sibling crate on the other side. This chapter enumerates the boundaries and names what flows across each one. The [module layout](00-module-layout.md) groups these into a single `coupling/` module at the filesystem level; this chapter elaborates the semantics of each coupling.

| Boundary | Direction | What crosses |
|---|---|---|
| [With `sim-mjcf`](02-coupling/00-mjcf.md) | bidirectional | **In:** rigid contact partners (probes, mandrels), pin constraints. **Out:** reaction forces from the soft body, constraint Jacobians for `sim-mjcf`'s solver |
| [With `sim-thermostat`](02-coupling/01-thermostat.md) | bidirectional | **In:** temperature field, temperature-dependent material parameter modulation. **Out:** dissipative heating rate from viscoelastic deformation |
| [With `sim-bevy`](02-coupling/02-bevy.md) | out only | Deformed mesh + per-vertex attributes (stress, temperature, thickness, contact pressure) for shader consumption — no state flows back into physics |
| [With `sim-ml-chassis`](02-coupling/03-ml-chassis.md) | bidirectional | **In:** design parameters $\theta$ from the optimizer. **Out:** scalar reward + gradient $\partial R / \partial \theta$. Chassis is extended with a GPU tensor backend and a VJP registration API to support custom solver-level gradients |
| [With `cf-design`](02-coupling/04-cf-design.md) | in only | SDF geometry spec + SDF-valued material field spec, re-sent on every design edit. Re-meshing and warm-started re-solve handled by [`sdf_bridge/`](00-module-layout/08-sdf-bridge.md) |

Two claims Ch 02 rests on:

1. **These are boundaries, not couplings.** Each sibling crate develops on its own schedule and tests independently; each coupling is a thin trait-based API at the boundary, not a shared internal datastructure. The consequence is that any sibling can be swapped (a different rigid-body solver stepping in for `sim-mjcf`, a different shader layer stepping in for `sim-bevy`) without forcing changes in `sim-soft`. This is how the [Ch 03 thesis](../10-physical/03-thesis.md) avoids the games-vs-science tooling lock-in — by refusing to let any one sibling reach into `sim-soft`'s internals.
2. **The boundary set is closed.** Five crates, not more. A sixth coupling (say, a learned policy network that drives the probe trajectory) is not a new boundary — it composes on top of the `sim-ml-chassis` boundary, because that is where "control input from a differentiable source" already lives. Keeping the boundary set closed is what lets [Part 12's roadmap](../120-roadmap/00-dependencies.md) commit to a finite integration schedule rather than an open-ended one.
