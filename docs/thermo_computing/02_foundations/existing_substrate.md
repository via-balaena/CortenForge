# Existing Substrate

> Extracted from `MASTER_PLAN.md` §3 *What already exists in CortenForge*,
> *What does not yet exist (in code)*, and *Adjacent capabilities to lean on*
> subsections during the 2026-04-09 doc-tree refactor. Slow-changing
> substrate reference, positive AND negative space.
>
> The "What does not yet exist" subsection was added to this file during
> pre-execution discovery (refactor plan §3.6 #3) — the original §3.1
> mapping had omitted it.

## What already exists in CortenForge

- **External force buffers** in `sim/L0/core/src/types/data.rs`:
  `qfrc_applied` (per-DOF generalized forces), `xfrc_applied` (per-body
  Cartesian forces). Folded into `qfrc_smooth` in
  `sim/L0/core/src/constraint/mod.rs` before the constraint solve, matching
  MuJoCo's `mj_fwdAcceleration` ordering.
- **Damping** as a first-class force term in `sim/L0/core/src/forward/passive.rs`:
  `dof_damping`, `jnt_damping`, `tendon_damping`, `flex_damping`,
  `flex_edgedamping`, `flex_bend_damping`. Damping force `τ = −b·v` flows into
  `qfrc_damper`.
- **Integrator family**: explicit Euler, semi-implicit, RK4, implicit-fast
  (Newton-based). Implicit-fast is the production workhorse.
- **Sleep / island infrastructure** already inspects `qfrc_applied` and
  `xfrc_applied` to decide wake-up — meaning anything we write into those
  buffers will Just Work with islanding.
- **ml-bridge autograd** (`sim/L0/ml-bridge/`, Phases 1–6c complete). Spec at
  `sim/L0/ml-bridge/AUTOGRAD_SPEC.md`.
- **ml-bridge RL** algorithms with policy persistence + best-policy tracking
  (recently merged: `4260a58`, `228d044`).
- **cf-design** (Phases 1–5 complete) with implicit surface composition,
  mechanism library, and finite-difference design optimization with
  analytic ∂f/∂θ at the SDF field level only (full pipeline is not
  end-to-end differentiable; see [Q5 in `open_questions.md`](./open_questions.md)
  for the finding and build-order implications).

## What does *not* yet exist (in code)

*Many items below are designed in the chassis doc but not yet
implemented. The chassis is a paper artifact; Phase 1 will turn the
first slice of it into shipping code.*

- **No code** at `sim/L0/thermostat/`. The crate directory itself
  doesn't exist. Designed in chassis Decisions 1-7; implementation
  starts after the Phase 1 spec is drafted.
- Any concrete `LangevinThermostat`, `BrownianRatchet`, or other
  stochastic component implementation.
- Any sampling validation test in `sim-conformance-tests` or
  `sim-thermostat`'s own `tests/` directory. Test conventions
  catalogued in part 9; helpers designed in chassis Decision 5.
- Any energy-based model training pathway *bound to a thermo
  component*. ml-bridge autograd exists; the EBM-on-thermostat
  binding does not.
- Any bistable mechanical example or "physical p-bit" reference
  design. Phase 3.
- A bridge to THRML or any block-Gibbs sampler. Q3 RESOLVED for
  Phase 6 — three options on the table, leading direction is
  native implementation.
- The Phase 1 spec itself (`PHASE_1_LANGEVIN_THERMOSTAT_SPEC.md`).

## Adjacent capabilities to lean on

- The visual example infrastructure (sim-bevy + museum-plaque READMEs).
  Bistable switching is a great visual story — "watch the bit flip."
- The conformance test harness (`sim/L0/tests/`) for validation against
  MuJoCo ground truth. Equipartition tests fit this pattern.
- The grading system (`cargo xtask grade`). Anything new must pass A-grade
  per project policy.
