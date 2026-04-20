# coupling/

The `coupling/` module holds the co-simulation interfaces between `sim-soft` and its sibling crates — [`sim-core`](../02-coupling/00-mjcf.md) for rigid-body partners and [`sim-thermostat`](../02-coupling/01-thermostat.md) for the temperature field. It is the thin boundary layer that keeps sibling-crate concerns from contaminating [`solver/`](04-solver.md) while still letting each timestep exchange state with its neighbors. Wider crate-boundary design — [`sim-bevy`](../02-coupling/02-bevy.md), [`sim-ml-chassis`](../02-coupling/03-ml-chassis.md), [`cf-design`](../02-coupling/04-cf-design.md) — lives in [Part 11 Ch 02](../02-coupling.md); this module's scope is specifically **the two siblings that exchange per-timestep state inside the sim loop**.

## What the module owns

| Sub-area | Responsibility | Where it is specified |
|---|---|---|
| `sim-core` coupling | Rigid-probe geometry in, contact-force accumulation out; co-simulated timestep via fixed-point iteration | [Part 5 Ch 03 §00](../../50-time-integration/03-coupling/00-mujoco.md), [Part 11 Ch 02 §00](../02-coupling/00-mjcf.md) |
| `sim-thermostat` coupling | Temperature field in, dissipative heating power out; subcycling if thermostat timestep differs from mechanical step | [Part 5 Ch 03](../../50-time-integration/03-coupling.md), [Part 11 Ch 02 §01](../02-coupling/01-thermostat.md) |
| Fixed-point iteration | The [Part 5 Ch 03 §01](../../50-time-integration/03-coupling/01-fixed-point.md) scheme — each substep is a full Newton-converged `sim-soft` solve against the current sibling state, iterated until sibling-state changes fall below tolerance | [Part 5 Ch 03 §01](../../50-time-integration/03-coupling/01-fixed-point.md) |
| Shared ECS handle | Read-only access to sibling state via a small per-coupling handle (no raw `sim-core` / `sim-thermostat` types leak into `solver/`) | [Part 11 Ch 02 §00 / §01](../02-coupling/00-mjcf.md) |

## Three claims

**1. Coupling is outside the Newton loop.** Each `sim-soft` Newton solve runs against a frozen snapshot of sibling state — [`sim-core`](../02-coupling/00-mjcf.md)'s current rigid-body configuration, [`sim-thermostat`](../02-coupling/01-thermostat.md)'s current temperature field — and fixed-point iteration across siblings happens at the substep level, not inside the Newton loop. [Part 5 Ch 03 §01](../../50-time-integration/03-coupling/01-fixed-point.md) commits to this structure because the alternative (sibling state updated inside Newton) makes the Hessian no longer local to `sim-soft`'s DOFs and breaks the factor-on-tape contract [`solver/`](04-solver.md) rests on. One Newton solve, one coupling step. Convergence across siblings comes from fixed-point iteration, not from inner-loop tangles.

**2. Only sibling crates that exchange per-step state live here.** [`sim-core`](../02-coupling/00-mjcf.md) and [`sim-thermostat`](../02-coupling/01-thermostat.md) co-simulate with `sim-soft` per timestep — rigid bodies and temperature both tick at the sim clock. [`sim-bevy`](../02-coupling/02-bevy.md) reads per-frame state but does not feed any back; [`sim-ml-chassis`](../02-coupling/03-ml-chassis.md) and [`sim-opt`](../02-coupling/03-ml-chassis.md) sit *outside* the sim loop, operating per-evaluation not per-step; [`cf-design`](../02-coupling/04-cf-design.md) edits the scene between evaluations, not during one. The per-step / per-frame / per-evaluation / per-edit distinction is what makes this module's scope narrow by design.

**3. `coupling/` owns the time-scale bridge; siblings own their own physics.** When `sim-thermostat` ticks faster than the mechanical step (thermal diffusion on a fine spatial grid needs small $\Delta t$), `coupling/` subcycles: freeze the `sim-soft` state, tick the thermostat several times, use the time-averaged temperature field as the boundary condition for the next `sim-soft` step. The inverse case — thermostat coarser than mechanical step — is handled symmetrically: tick `sim-soft` several times on the same temperature field, integrate dissipative heating, hand it to the thermostat. The subcycling logic belongs here because neither `sim-soft` nor `sim-thermostat` should know about the other's clock rate.

## What the module does not carry

- **No wider boundary concerns.** [`sim-bevy`](../02-coupling/02-bevy.md)'s mesh-streaming API ([Part 9 Ch 05](../../90-visual/05-sim-bevy.md)) is a read-only data export handled in [`readout/`](09-readout.md); it does not need per-step feedback and is not a `coupling/` concern. [`cf-design`](../02-coupling/04-cf-design.md)'s SDF edits are handled in [`sdf_bridge/`](08-sdf-bridge.md), not here.
- **No rigid-body physics.** `sim-core` owns the rigid-body integration; `coupling/` only exchanges state. Per the [β.1-followup audit closure](../02-coupling/00-mjcf.md), `sim-core` is the rigid-body simulator (architecturally the dominant half of the coupling) and `sim-mjcf` is the MJCF loader that builds `sim-core` scenes. Coupling commitments are `sim-core` commitments; loader specifics are `sim-mjcf` specifics.
- **No temperature field storage.** The temperature field lives in `sim-thermostat`; `coupling/` reads a view and passes it to [`material/`](00-material.md)'s thermal decorators ([Part 2 Ch 08](../../20-materials/08-thermal-coupling.md)).

## What this commits downstream

- **[`solver/`](04-solver.md) sees coupling-boundary state as frozen during a Newton iteration.** The Hessian stays local, the factor-on-tape contract holds, IFT backward works — all because the coupling does not intrude.
- **[Part 11 Ch 02 §00 and §01](../02-coupling/00-mjcf.md)** describe the same interfaces from the cross-crate-architecture angle, where `sim-soft`'s `coupling/` is one side and `sim-core` / `sim-thermostat` are the other. This module implements the `sim-soft` side; the sibling crates implement their sides.
- **[Part 11 Ch 03 Phase F deliverable](../03-build-order.md#the-committed-order)** is when this module closes — all phases A–E can test `sim-soft` in isolation; Phase F is where the two-crate canonical-problem configuration becomes executable.
