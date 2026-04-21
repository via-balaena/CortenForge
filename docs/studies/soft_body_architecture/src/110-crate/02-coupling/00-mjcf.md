# With sim-core and sim-mjcf

The `sim-core` coupling is the rigid-body half of the canonical problem: a compliant cavity (`sim-soft`) contacted by a rigid probe whose kinematics live in a separate crate. `sim-core` is the rigid-body simulator — a MuJoCo-pipeline-aligned pure-Rust runtime that advances the rigid state through time. `sim-mjcf` is the MJCF loader that parses `.xml` model files at scene construction and populates `sim-core`'s `Model` / `Data`. The per-step handshake is between `sim-soft` and `sim-core`; `sim-mjcf` is upstream of the handshake and is not invoked per-tick. This sub-leaf specifies the cross-crate architecture that [Part 5 Ch 03](../../50-time-integration/03-coupling.md) translates into time-integration terms and that [`coupling/` §05](../00-module-layout/05-coupling.md) implements on the `sim-soft` side.

The split between the two crates is preserved per the [β.1-followup audit closure](../00-module-layout/05-coupling.md): architecture commitments are `sim-core` commitments, loader specifics are `sim-mjcf` specifics, and the sub-leaf title keeps both names because every cross-crate obligation this book names touches exactly one of them.

## What crosses the boundary

Four fields cross at each handshake tick; the tick period is set by the coarser native timestep per the [fixed-point sub-leaf](../../50-time-integration/03-coupling/01-fixed-point.md).

| Field | Direction | Representation | Required? |
|---|---|---|---|
| Rigid pose + spatial velocity | `sim-core` → `sim-soft` | $(R, t) \in SO(3) \times \mathbb{R}^3$ plus $(\omega, v) \in \mathbb{R}^3 \times \mathbb{R}^3$; read from `Data.xpos` / `Data.xquat` / `Data.cvel` | yes |
| Contact wrench at rigid body | `sim-soft` → `sim-core` | per-body $(f, \tau) \in \mathbb{R}^3 \times \mathbb{R}^3$ applied via MuJoCo-convention `Data.xfrc_applied` | yes |
| Pin-constraint reactions | `sim-soft` → `sim-core` | same `xfrc_applied` channel, summed with contact wrench before export | opt-in per scene config |
| Temperature / heat flux at interface | via [`sim-thermostat`](01-thermostat.md) | out-of-band; not on the `sim-core` handshake | opt-in per scene config |

The committed surface is wrench-only on `xfrc_applied`, plus the optional pin-reaction contribution on the same channel. [Part 5 Ch 03 §00](../../50-time-integration/03-coupling/00-mujoco.md) commits explicitly that "no intermediate Newton residuals, Hessian blocks, or constraint Jacobians are shared" — constraint Jacobians stay inside each simulator's own solver, and nothing that looks like a shared constraint representation crosses the boundary.

## The coupling is bidirectional at the handshake, one-way inside each solver

Each simulator advances its native step with the partner's boundary data held constant, then the boundary data is exchanged and the outer fixed-point iterates. Neither simulator writes into the other's internal state — no shared sparse Hessian, no shared constraint solver. `sim-core`'s MuJoCo pipeline projects `xfrc_applied` through the body Jacobian (`qfrc_smooth += J^T * xfrc_applied`) during `mj_fwdAcceleration`, advancing the rigid body as if the contact wrench were an externally commanded force; `sim-core`'s own constraint solver never sees the soft-rigid pair. Symmetrically, `sim-soft`'s Newton loop solves against the current rigid pose/velocity as a frozen boundary condition (the [`coupling/` §05 "coupling is outside the Newton loop"](../00-module-layout/05-coupling.md) commitment). This is the partitioned-analysis pattern [Felippa and Park 1980](../../50-time-integration/03-coupling.md) frames for coupled-system integration: black-box simulators, outer-loop handshake.

The fixed-point iterate count is 1–3 in steady state; [the fixed-point sub-leaf](../../50-time-integration/03-coupling/01-fixed-point.md) commits a retry cap plus an adaptive-$\Delta t$ escape hatch that fires via a `handshake-uncontracted` signal when the cap is hit without convergence.

## The `sim-soft` side — `coupling/` trait `MjcfHandshake`

`sim-soft::coupling/` exports an `MjcfHandshake` trait that the outer coupling loop calls. The trait owns the boundary-data shape and keeps raw `sim-core` types from leaking into [`solver/`](../00-module-layout/04-solver.md) — consistent with [`coupling/` §05](../00-module-layout/05-coupling.md)'s commitment that the module is a bridge, not a dependency.

```rust
pub struct RigidBoundary {
    pub body_id:     BodyId,        // sim-core body index (from sim-mjcf load)
    pub pose:        Pose,          // SE(3): rotation + translation
    pub spatial_vel: SpatialVec,    // MuJoCo cvel: world-aligned axes, linear at COM
}

pub struct ContactReaction {
    pub body_id:          BodyId,
    pub wrench:           Wrench,   // (force, torque) on the rigid body
    pub contact_centroid: Vec3,     // diagnostic; not used by sim-core
}

pub trait MjcfHandshake {
    fn import_rigid_state(&mut self, boundaries: &[RigidBoundary]);
    fn export_contact_reactions(&self) -> Vec<ContactReaction>;
}
```

The contact wrench is the integral of the IPC barrier's per-vertex contact pressure over each rigid body's contact region, re-expressed as a net force plus net torque about the body's COM; [Part 5 Ch 03 §00](../../50-time-integration/03-coupling/00-mujoco.md) fixes the representation, unit convention, and reference-point choice. The contact-region centroid is attached as a diagnostic for the outer-loop log and does not feed `sim-core`.

The boundary-data types above are the only `sim-core`-adjacent types `sim-soft::coupling/` exposes. Nothing inside `solver/`, `contact/`, or `readout/` touches `sim-core::Data` directly; the bridge module owns the read-through and the write-through. This is what keeps the crate dependency graph a DAG — `sim-soft` depends on `sim-core` only through `coupling/`, and a different rigid-body runtime (a future non-MuJoCo solver) would plug in by implementing `MjcfHandshake`'s consumer side without forcing changes upstream. Single-direction dependency. Thin boundary.

## The `sim-mjcf` side — loader only

`sim-mjcf` parses `.xml` MJCF model files at scene construction, populates `sim-core::Model` (immutable after load) and the initial `sim-core::Data` (which `sim-core` advances through time), and exits the call stack. It is not involved in the per-step exchange. The one piece of setup-time state that crosses into `sim-soft::coupling/`'s construction is the `BodyId` set — which rigid bodies in the loaded scene are participants in soft-rigid contact — plus any pin-constraint declarations keyed on `BodyId` plus per-vertex references in the soft body's initial configuration.

Scene ingest is a one-shot call:

```rust
pub fn load_scene(xml_path: &Path) -> (CoreScene, Vec<ParticipantBodyId>);
```

`ParticipantBodyId` is a newtype marking which rigid bodies cross the handshake. Non-participants (scene dressing, decorative rigid bodies) pay no per-step cost on the boundary. The discipline is that `sim-mjcf`'s contract ends at the `(CoreScene, participants)` pair; any state that mutates per step belongs to `sim-core::Data`, and the handshake reads it from there.

## Timestep mismatch and what the handshake owns

`sim-core` defaults to MuJoCo's 2 ms rigid-body step; contact-heavy scenes reduce this toward 0.5–1 ms per the [MuJoCo XML reference](https://mujoco.readthedocs.io/en/stable/XMLreference.html). `sim-soft` runs 16 ms in experience mode or 1 ms in design mode. [The fixed-point sub-leaf](../../50-time-integration/03-coupling/01-fixed-point.md) resolves the mismatch: the `sim-soft` step is the handshake period, and `sim-core` advances 8–32 consecutive native steps within one handshake in experience mode (one step per handshake in design mode with `sim-core`'s step tuned to match). Between handshakes, `sim-core` holds the last-received contact wrench piecewise-constant across its native steps — the zeroth-order coupling payload from [Felippa and Park 1980](../../50-time-integration/03-coupling.md), with local truncation error bounded by $O(\dot{\mathbf{f}}\,\Delta t^2)$ in transferred impulse per handshake.

The subcycling logic lives in [`coupling/` §05](../00-module-layout/05-coupling.md) on the `sim-soft` side, which is the [`coupling/` "time-scale bridge" claim 3](../00-module-layout/05-coupling.md) made concrete for `sim-core`: neither `sim-core` nor `sim-soft` should know the other's clock rate, and the bridge is the piece that does.

## Determinism-in-θ implication

The [`ForwardMap` γ-locked contract](../../100-optimization/00-forward.md) commits that repeated `evaluate` calls at the same $\theta$ produce the same `RewardBreakdown` modulo cache state. The `sim-core` coupling preserves this because (a) `sim-core`'s rigid-body integrator is deterministic given fixed input state and timestep, (b) the fixed-point iteration's retry cap is a deterministic function of the initial partner state, and (c) `sim-mjcf`'s scene load is a pure function of the XML content. No RNG, no wall-clock sensitivity, no non-reproducible dispatch. This is what lets [Part 10 Ch 02's BayesOpt GP cache](../../100-optimization/02-bayesopt.md) treat coupled-simulation evaluations the same as uncoupled ones for the purpose of cached training data.

Any future coupling change that introduces stochastic partner kinematics — for example, a rigid-body forcing schedule driven by an external sensor stream — would have to either key its seed off `theta` (preserving determinism conditional on seed) or explicitly update the `ForwardMap` contract in coordination with [Part 10 Ch 00](../../100-optimization/00-forward.md) and [Part 11 Ch 00 `readout/`](../00-module-layout/09-readout.md). Silently breaking determinism is the failure mode this sub-leaf is naming out loud so that it does not land accidentally.

## What this sub-leaf commits the crate to

- **`sim-core` coupling is a wrench-only bidirectional handshake via `xfrc_applied`.** No constraint Jacobians, no Hessian blocks, no shared Newton residuals; each simulator keeps its own, per [Part 5 Ch 03 §00](../../50-time-integration/03-coupling/00-mujoco.md).
- **`sim-mjcf` is a setup-time loader and nothing else.** No per-step state touches `sim-mjcf`; its contract is `(CoreScene, ParticipantBodyIds)` at scene construction.
- **`coupling/`'s `MjcfHandshake` trait is the only `sim-core`-adjacent surface `sim-soft` exposes.** `solver/`, `contact/`, `readout/` do not include `sim-core` types. Swapping out the rigid-body runtime is a `coupling/`-only edit.
- **Timestep subcycling is `coupling/`'s responsibility.** 8–32 `sim-core` steps per `sim-soft` handshake in experience mode; one step per handshake in design mode. Neither crate encodes the other's rate.
- **Determinism-in-θ is preserved across the coupling.** No RNG, no wall-clock sensitivity; [`ForwardMap`](../../100-optimization/00-forward.md)'s contract holds on coupled evaluations.
- **Phase F is when this coupling closes.** Phases A–E run `sim-soft` in isolation against hardcoded SDF probes; [Phase F](../03-build-order.md#the-committed-order) is when the `MjcfHandshake` trait's consumer side comes up and the canonical problem becomes a multi-crate simulation.
