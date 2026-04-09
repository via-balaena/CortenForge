# 2026-04-09 — Initial scaffold

> Extracted from `MASTER_PLAN.md` §7 part 00a during the 2026-04-09 doc-tree refactor.

- **Trigger**: User shared a 5-layer thermodynamic computing proposal from a
  prior Claude session. Original proposal started at atomic-scale Langevin MD
  (`sim-langevin` as a new crate) and worked up through coarse-graining,
  rigid-body bridge, THRML, and RL.
- **Reframe**: After clarifying that the user's interest is *thermodynamic
  computing* specifically (not molecular dynamics), shifted the entire stack
  from atomic scale to **device scale**. Mechanical p-bits instead of
  transistors or atoms. This aligns with the existing CortenForge initiatives
  (cf-design, bio-inspired mechatronics, design → 3D print).
- **Recon findings**:
  - `qfrc_applied` and `xfrc_applied` already exist as MuJoCo-style applied
    force buffers, projected into `qfrc_smooth` before the constraint solve
    in `sim/L0/core/src/constraint/mod.rs` (lines 78–87).
  - Damping is already first-class across many domains: `dof_damping`,
    `jnt_damping`, `tendon_damping`, `flex_damping`, `flex_edgedamping`,
    `flex_bend_damping` (see `sim/L0/core/src/forward/passive.rs`).
  - This means **half of the FDT machinery already exists**. The only missing
    piece is the stochastic force term.
- **Decision**: Do *not* create a `sim-langevin` crate. Add a `Thermostat`
  abstraction as a small extension to sim-core's forward step instead. The
  proposal's "plug into sim-core integrator pattern" framing was wrong about
  the scale (should be at the forward-step level, not a parallel L0 crate).
- **Decision**: Phase 1 minimum viable test = 1-DOF damped harmonic
  oscillator equipartition. Smallest possible foothold; everything else gates
  on it.
- **Open questions raised**: Q1 (constraint projection), Q2 (implicit
  integrator interaction), Q3 (`thrml-rs` existence), Q4 (zero default
  damping), Q5 (cf-design differentiability), Q6 (reward signal choice).
- **Next action**: Read the forward step + integrator code carefully and
  draft the Phase 1 spec (`docs/thermo_computing/PHASE_1_LANGEVIN_THERMOSTAT_SPEC.md`).

