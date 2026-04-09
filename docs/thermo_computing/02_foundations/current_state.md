# Current State (the starting point)

> Extracted from `MASTER_PLAN.md` §3 *Current state of the thermo line*
> subsection during the 2026-04-09 doc-tree refactor.

*Snapshot — last updated 2026-04-09 (after doc review pass). The Recon
Log at the bottom of this document is append-only history; **this section
is the canonical "where are we right now"** and is rewritten as the
state evolves. Re-verify before relying on these claims; the codebase
moves.*

## Current state of the thermo line (where we are *right now*)

*Thermo-computing initiative on `feature/thermo-computing` (now merged
to main as `143c2fc`) plus `feature/thermo-doc-review` for the doc
review pass.*

- **Recon round COMPLETE.** Eight Phase-1-blocking items resolved
  (parts 2-10 of the Recon Log); the substrate (`cb_passive`,
  `qfrc_passive`, `Model::set_passive_callback`, `ChaCha8Rng`,
  `model.timestep` semantics, test conventions, sibling-crate
  layout) is fully understood.
- **Chassis design round COMPLETE.** Seven decisions resolved in
  [`chassis_design.md`](./chassis_design.md):
  1. `PassiveComponent` trait (broad, callback-shaped, type-enforced
     write target via `&mut DVector<f64>` per M5)
  2. `PassiveStack::builder().with(...).build().install(&mut model)`
  3. `install_per_env(prototype, n, build_one)` + defensive clear
     resolves the `Model::clone()` callback-sharing footgun
  4. `Diagnose` trait with `diagnostic_summary -> String` (minimal)
  5. `WelfordOnline` (with `reset` + `merge` per M4) +
     `assert_within_n_sigma` + `sample_stats` test utilities
  6. Flat ml-bridge-style crate layout at `sim/L0/thermostat/`
  7. `Stochastic` orthogonal trait + `disable_stochastic()` RAII guard
     for FD/autograd contexts (Decision 7 added by doc review M2)
- **Doc review pass COMPLETE for must-fixes**. M1 (sampling-error
  tolerance wording), M2 (Decision 7 stochastic gating), M3 (Q5
  escalated to active recon), M4 (Welford reset + merge), M5 (trait
  write-target contract type-enforced) all landed. Should-fix and
  nit items are in flight on `feature/thermo-doc-review`.
- **Phase-1 implementation: NOT YET STARTED.** No code, no
  Cargo.toml, no `sim/L0/thermostat/` directory. The chassis is a
  paper artifact; everything is by design — Phase 1 implementation
  starts with the spec, not direct code.
- **Crate to be created**: `sim/L0/thermostat/` (package
  `sim-thermostat`). Deps: `sim-core`, `nalgebra`, `rand`,
  `rand_chacha`, `rand_distr`. Estimated Phase 1 footprint:
  ~890 LOC across 8 files (per Decision 6 + M4 + M2 inventory).
  **sim-core stays rand-free in production** (item 8 + Decision 6
  constraint).
- **Open questions status**:
  - Q1 (constrained Langevin) — UNRESOLVED, gates Phase 2.
    Reference: Lelièvre, Rousset, Stoltz.
  - Q2 (implicit integrator interaction) — RESOLVED part 2.
  - Q3 (`thrml-rs` existence) — RESOLVED 2026-04-09 doc review S1.
    Yes with caveats; two community Rust ports on GitHub, neither
    on crates.io, neither officially maintained. Three Phase 6
    options (depend / native / vendor); leading direction is native.
  - Q4 (zero default `dof_damping`) — RESOLVED part 2.
  - Q5 (cf-design end-to-end differentiability) — RESOLVED 2026-04-09
    ([recon log part 13](../04_recon_log/2026-04-09_part_13_q5_cf_design.md)):
    **NO.** cf-design has analytic ∂f/∂θ at the SDF field level only;
    mesh extraction (marching cubes + dual contouring) is topologically
    discrete, mass properties are unwired to `param_gradient`, and
    sim-core's forward step has no autograd handshake. The shipping
    Phase 5 "differentiable design optimization" is actually centered
    finite differences over the full `θ → SDF → mesh → Model → simulate
    → J` pipeline. D3 (co-design) demoted from headline experiment to
    "blocked on cf-design foundation, revisit when D2/D4 force the
    question." Phases 1-4 unaffected.
  - Q6 (Phase 7 reward signal) — deferred until Phase 6 results.
- **Next action**: draft
  [`../03_phases/01_langevin_thermostat.md`](../03_phases/01_langevin_thermostat.md)
  against the now-finalized chassis. The doc review pass and Q5 recon
  are both complete.
