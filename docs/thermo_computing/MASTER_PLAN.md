# Thermodynamic Computing — Master Plan

> **Status**: Draft. Initial scaffold, populated from one round of recon (2026-04-09).
> **Branch**: `feature/thermo-computing`
> **Owner**: Jon

This is a *living* document. It captures the starting point, the estimated
endpoint, and the gap between them. As recon and implementation progress, both
ends move toward the middle. Every meaningful change should leave a dated entry
in the **Recon Log** at the bottom so the reasoning is preserved.

Child specs live next to this file in `docs/thermo_computing/`. They are
spawned from individual phases of **The Gap** as those phases mature from
"sketch" to "implementable."

---

## 2. Research Directions

The Vision above is abstract. **These are the concrete experiments that
realize it.** They are not the build order — Phase 1 still has to ship
first — but they are the *why*. The Master Plan exists to keep these alive
through implementation.

Five experiments + one foundational connection, ordered from earliest
payoff to longest horizon. Each direction states: concept, physics
grounding, the experiment, where it fits in the build, and why it matters.

#### Status legend (added by doc review N3, 2026-04-09)

Each direction (and the priority ladder at the bottom of §2) carries
a `Status: idea / <reach> / <novelty>` line. The categories:

- **Reach**: how far the experiment is from currently-shipping
  CortenForge. `low` = achievable in the current branch with one
  more phase shipped. `early` = within 1-2 phases. `mid` = within
  3-4 phases. `long-term` = 5+ phases out, or has external
  dependencies (printer, fab, measurement rig). `moonshot` = an
  open research problem that may not work even with the full stack.
- **Novelty**: how much of this would be new in the literature.
  `low` = textbook physics, well-trodden experiment. `med` = a
  variant of a published result, with at least one new ingredient.
  `high` = a publishable result on its own. `extreme` = paper or
  thesis material; either no prior work has the full stack to do
  it, or the result would be a first-of-kind demonstration.
  `gate` = not a research result itself but blocks other work
  (e.g., the Phase 1-2 thermostat shipping).

These are the author's calibration; treat them as priors that should
be revised as the build progresses and the literature is read more
carefully.

### D1 — The agent learns to *harvest* thermal noise (flashing ratchet)

**Concept**: A flashing-ratchet Brownian motor controlled by an RL policy.
A particle in an asymmetric periodic potential plus thermal noise can be
made to drift directionally if the potential is switched on and off at the
right rate. With no policy, diffusion is symmetric. With the right policy,
the system rectifies thermal motion into directed transport.

**Physics**: Astumian, Magnasco, Reimann (review article). Real and
well-grounded. Biology already does this — kinesin walking on microtubules,
ATP synthase as a rotary ratchet.

**Experiment**: 1-DOF particle in an asymmetric periodic potential, single
binary actuator (potential on/off), thermal bath at fixed `T`. Reward = net
displacement per unit time. Train CEM or REINFORCE. The agent should
discover the resonant flashing frequency without being told it exists.

**Stack fit**: Achievable after Phase 3. Bistable element + thermostat +
simple actuator + existing RL is enough.

**Why it matters**: First experiment that produces *both* a stunning visual
("the agent learned to harvest heat") and a publishable result. Direct
analogue to molecular motors. Earliest payoff for the entire research line.

**Status**: idea / mid-term reach / very high novelty.

### D2 — Stochastic resonance, RL-tuned

**Concept**: A sub-threshold periodic signal that *cannot* be detected
deterministically can be detected by adding the *right amount* of noise.
Below the optimum, switches are too rare; above, switches are too random;
at the peak, switches lock to the signal. The RL agent's job is to find
the peak.

**Physics**: Stochastic resonance (Benzi, Wiesenfeld, Moss; Gammaitoni
review). Used in neuroscience, climate modeling, MEMS sensors.

**Experiment**: A buckling-beam bistable element from cf-design with a
sub-threshold periodic load. The agent controls a parameter that
effectively sets the noise level (thermostat `T`, or coupling stiffness).
Reward = mutual information between input signal and observed switching
events. The agent should converge to the SR peak.

**Stack fit**: Phase 4. Requires the coupled-element infrastructure.

**Why it matters**: Counter-intuitive result with deep physics. The 3D
print version is the kicker — print the beam, drive it on a shaker table,
verify the optimum holds physically. This is novel even within the SR
literature once the physical realization is included.

**Status**: idea / mid-term reach / very high novelty.

### D3 — Co-design: geometry × policy × temperature (the triple-product)

**Concept**: With cf-design + autograd + ml-bridge + thermostat, four
things become parameters in a single optimization:

- the *shape* of the bistable element (cf-design)
- the *coupling* between elements (cf-design + sim-core)
- the *control policy* on the actuators (RL)
- the *operating temperature / noise spectrum* (thermostat)

Joint objective: task reward, energy budget, robustness to noise. This is
morphological computation (Pfeifer, Iida) made *thermodynamically explicit*
— the body does part of the computation, and the energetic cost is
quantified in `k_B·T` units.

**Experiment**: Pick a task ("walk forward in a thermal bath" or "sample
from distribution X"). Co-optimize all four parameter sets jointly. Compare
against fixed-geometry, fixed-temperature, and fixed-policy baselines. The
joint optimum should *use* noise differently — softer mechanism, lower
control authority, lower energy.

**Stack fit**: Phase 5+. Requires the differentiable cf-design → sim-core
parameter pipeline (Q5).

**Why it matters**: This is the headline experiment. The cleanest
demonstration that all four CortenForge subsystems multiply. The
`project_cf_design_vision.md` charter extended one floor up. Nobody else
has the full stack to do this.

**Status**: idea / mid-term reach / extremely high novelty. Likely paper
material on its own.

### D4 — Sim-to-real on a 3D-printed thermodynamic device

**Concept**: The killer experiment for the entire pipeline. Train a
coupled bistable array as an EBM in simulation to match a target
distribution. Freeze parameters. **3D print the device.** Measure its
physical sample distribution (high-speed video + state classification).
Compare against simulation.

**Experiment**: Pick a small target distribution (4-bit binary, or a 2D
Gaussian mixture). Train a printable mechanical sampler in cf-design + sim
+ thermostat. Print. Measure. Quantify the sim-to-real gap.

**Stack fit**: After Phase 5. Hardware loop is real but fully owned — no
fab-house dependency.

**External dependencies** (added by doc review S5, 2026-04-09 — these
are real engineering effort, not afterthoughts):
- **3D printer with reproducible mechanical properties**. FDM is
  cheap and accessible but has poor layer-to-layer consistency,
  anisotropic stiffness, and visible buckling-mode artifacts. SLA
  / DLP gives smoother surfaces and more uniform stiffness but
  needs post-cure handling and material-specific calibration. The
  print process directly affects barrier height and coupling
  stiffness — both first-order parameters in the EBM.
- **Print material consistency**. PLA stiffness varies batch-to-batch
  by ~10% (humidity, age, supplier); resin stiffness varies with
  cure time and temperature. The same digital design printed twice
  can produce devices with measurably different sample distributions.
  The Phase 1+ thermo line treats `γ` and `kT` as known parameters;
  D4 must measure them per-print, not assume them.
- **High-speed video + state classification infrastructure**. To
  measure a printed device's *actual* sample distribution, the
  pipeline needs (a) a high-speed camera capable of resolving the
  switching events (typical bistable mechanical timescale: 10-1000 Hz),
  (b) lighting and contrast that survive the print's surface finish,
  (c) a state classifier (probably a small CV pipeline or ML model)
  that maps frame → discrete state with high enough accuracy that
  classification noise doesn't dominate the measured distribution.
  None of this exists in CortenForge yet; it's a parallel build.
- **Per-device calibration loop**. The simulated device's `T`, `γ`,
  barrier height *will not* exactly match the printed device's
  effective values. D4 needs a calibration step: measure the
  printed device's switching rate as a function of (something
  user-controllable, e.g., shaker amplitude or temperature),
  back out the effective Kramers parameters, and compare against
  the simulation under matched effective parameters. The
  sim-to-real gap is then a *meaningful* quantity, not a hardware-
  vs-software calibration error.

These dependencies don't kill D4 — every one is achievable with
off-the-shelf hardware and a few weeks of measurement-rig build —
but they should be planned as real engineering effort, not
hand-waved as "we have a printer." The "no fab dependency" claim
is true at the *fab-house* level (we don't need an Extropic or
Intel); it is *not* true at the engineering-effort level.

**Why it matters**: Closes the design → simulate → fabricate loop for a
probabilistic computing device. **No software-only stack can do this. No
fab-based stack can do it cheaply.** This is the moat. Most aligned with
the bio-inspired mechatronics + 3D print core vision.

**Status**: idea / longer-horizon reach / paradigm-defining if it works.

### D5 — Brownian computer near the Landauer bound (moonshot)

**Concept**: Bennett 1982 — computation can in principle be done at
arbitrarily low energy cost by a system that diffuses through a chain of
states corresponding to a computation, with the energy landscape biased
just enough to overcome thermal fluctuation. Landauer's bound says erasing
one bit costs at least `k_B·T·ln 2`. A Brownian computer asymptotically
saturates this.

**Experiment**: Design (RL + cf-design) the energy landscape of a
mechanical lattice such that under Langevin dynamics it computes a target
function — say a 3-bit logic gate — and measure how close to the Landauer
bound the energy cost can be pushed.

**Stack fit**: Phase 6+.

**Why it matters**: There is no concrete simulated demonstration of a
near-Landauer Brownian computer in any existing software stack. If it
works, it is foundational physics — textbook material. If it half-works,
it still produces measurable bounds on a real mechanical lattice.

**Status**: moonshot / longest-horizon reach / foundational. Don't attempt
unless D1–D4 are working.

### Foundational connection — energy-based RL meets thermodynamic devices

The deepest version of the sim × RL × thermo intersection: a Boltzmann
policy `π(a|s) ∝ exp(−E(s,a)/T)` is *already* an energy-based model. The
maximum-entropy RL lineage (Haarnoja & Levine, soft actor-critic) trains
exactly this. The same energy function `E(s,a)` can simultaneously be:

- the policy of an RL agent (max-ent RL)
- the energy of a physical thermodynamic device (this stack)
- the gradient target of the autograd engine (already shipped)

If these three roles can be made to share *one* `E(s,a)` function, the
policy the agent learns and the energy landscape the physical sampler
embodies are *the same object*. The autograd engine you already have
(Phases 1–6c) is the lever.

This needs a real lit review before any novelty claim — search not yet
done — but it is the deepest possible coupling between the existing ml-bridge
and the new thermo layer, and it should be on the radar from day one.
Tracking item: open a Recon Log entry when the lit review happens.

### Synthesis

> **CortenForge becomes the only physics + ML stack where the same energy
> function specifies the geometry of a fabricable device, the dynamics of
> its simulation, the gradient of its training, and the policy of its
> control — and where you can print the result and measure whether reality
> agrees.**

That sentence, backed by even one of the experiments above, is a paper.
With three, it is a thesis. With four, it is a research program.

### Priority ladder

| Priority | Item                              | Earliest phase | Reach     | Novelty |
| -------- | --------------------------------- | -------------- | --------- | ------- |
| 1        | Phase 1–2 (thermostat works)      | now            | low       | gate    |
| 2        | D1 — Brownian motor               | after Phase 3  | low-med   | high    |
| 3        | D3 — Co-design                    | after Phase 5  | medium    | extreme |
| 4        | D2 — Stochastic resonance         | after Phase 4  | medium    | high    |
| 5        | D4 — Sim-to-real on printed device| after Phase 5  | high      | extreme |
| 6        | D5 — Brownian computer            | after Phase 6  | very high | extreme |

The build order is still driven by **The Gap**. The Research Directions
inform *which* experiments to prioritize once each phase opens up. They are
the destination; the Gap is the road.

---

## 3. Current State (the starting point)

*Snapshot — last updated 2026-04-09 (after doc review pass). The Recon
Log at the bottom of this document is append-only history; **this section
is the canonical "where are we right now"** and is rewritten as the
state evolves. Re-verify before relying on these claims; the codebase
moves.*

### Current state of the thermo line (where we are *right now*)

*Thermo-computing initiative on `feature/thermo-computing` (now merged
to main as `143c2fc`) plus `feature/thermo-doc-review` for the doc
review pass.*

- **Recon round COMPLETE.** Eight Phase-1-blocking items resolved
  (parts 2-10 of the Recon Log); the substrate (`cb_passive`,
  `qfrc_passive`, `Model::set_passive_callback`, `ChaCha8Rng`,
  `model.timestep` semantics, test conventions, sibling-crate
  layout) is fully understood.
- **Chassis design round COMPLETE.** Seven decisions resolved in
  [`THERMO_CHASSIS_DESIGN.md`](./THERMO_CHASSIS_DESIGN.md):
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
  - Q5 (cf-design end-to-end differentiability) — ESCALATED 2026-04-09
    doc review M3 to active foreground recon, parallel with Phase 1
    spec drafting. Gates Phase 5 and the D3 headline experiment.
  - Q6 (Phase 7 reward signal) — deferred until Phase 6 results.
- **Next action**: complete the doc review pass (S2-S6, N1-N4),
  resolve Q5 in parallel, then draft
  `PHASE_1_LANGEVIN_THERMOSTAT_SPEC.md` against the now-finalized
  chassis.

### What already exists in CortenForge

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
  mechanism library, and differentiable design optimization.

### What does *not* yet exist (in code)

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

### Adjacent capabilities to lean on

- The visual example infrastructure (sim-bevy + museum-plaque READMEs).
  Bistable switching is a great visual story — "watch the bit flip."
- The conformance test harness (`sim/L0/tests/`) for validation against
  MuJoCo ground truth. Equipartition tests fit this pattern.
- The grading system (`cargo xtask grade`). Anything new must pass A-grade
  per project policy.

---

## 4. The Gap (working both ways toward the middle)

Phases ordered by dependency. Each phase has a validation gate; subsequent
phases do not start until the previous gate is green. This implements the
"baby steps for new physics" + "fix gaps before continuing" rules.

| #   | Phase                              | Status   | Validation gate                                                       | Spec |
| --- | ---------------------------------- | -------- | --------------------------------------------------------------------- | ---- |
| 1   | Langevin thermostat (1-DOF)        | sketch   | Equipartition `⟨½mv²⟩ = ½k_B·T` on damped harmonic oscillator         | —    |
| 2   | Thermostat on free + articulated   | pending  | Equipartition holds on 3-DOF free body; on a constrained chain        | —    |
| 3   | Single bistable element            | pending  | Double-well switching rate matches Kramers' formula                   | —    |
| 4   | Coupled bistable array             | pending  | Joint distribution matches analytical Ising / CPU Gibbs ground truth  | —    |
| 5   | Differentiable EBM via cf-design   | pending  | Trained array matches a 2D Gaussian mixture target                    | —    |
| 6   | THRML bridge or native Gibbs       | pending  | Physical sampler vs. software sampler on the same energy function    | —    |
| 7   | RL on the sampler                  | pending  | Agent improves sample quality (ESS / autocorrelation / KL to target)  | —    |

### Phase 1 — Langevin thermostat (1-DOF)

> **Revised 2026-04-09 (part 4)** — superseded the original `qfrc_applied`
> design after recon found `cb_passive`. See Recon Log part 4 for the
> trade-off table and full reasoning.

The minimum viable first move. Implement a `LangevinThermostat` struct
that wraps in a `PassiveStack` (chassis Decision 2) and installs as a
`cb_passive` callback on the model. Each step, the thermostat contributes
both an explicit damping force *and* a stochastic force to its per-DOF
scratch buffer (chassis M5 contract); the stack folds the buffer into
`qfrc_passive` once per step:

```
qfrc_out[i] += −γ_i · qvel[i]  +  sqrt(2 · γ_i · k_B · T / h) · randn()
                └── damping ──┘   └────── FDT-paired noise ──────┘
```

After every component in the stack has run, the stack does
`data.qfrc_passive += qfrc_out` once. The thermostat itself never sees
or touches `data.qfrc_passive` directly.

where `γ_i` is the thermostat damping coefficient on DOF `i`, owned by the
thermostat instance (not by the model — see Q4).

**Scheme**: Explicit Langevin (Euler-Maruyama). Damping and noise are
both *custom passive forces*, sharing the existing `qfrc_passive`
aggregation and projection pipeline. No BAOAB splitting, no integrator
bypass, no model mutation, no split-step API needed. The discretization
temperature error scales as `O(h · γ / M)` and is below the validation
test's sampling-error tolerance at the chosen parameters (see Recon Log,
2026-04-09 part 2). Upgrade path to BAOAB / Eulerdamp is clear if higher
accuracy is later needed.

**Why `cb_passive` and not `qfrc_applied`**: Recon item 3 first finding
(2026-04-09 part 4). `cb_passive` is the documented hook for "custom
passive forces (e.g., viscous drag, spring models)" — exactly the
category Langevin damping + FDT-paired noise belongs to. `qfrc_passive`
is auto-cleared every step at the start of `mj_fwd_passive()`
(`passive.rs:368`), so accumulation is safe by construction — no silent
compounding possible. The thermostat is conceptually a passive thermal
force, so this is also the correct ontological category. Item 2's
original Option A (sole-writer overwrite of `qfrc_applied`) is superseded
and the forward-looking Option C (dedicated `qfrc_thermostat` field) is
no longer needed — `cb_passive` already achieves composability with RL /
controller writes to `qfrc_applied`.

**Where it lives**: A small `thermostat` submodule of sim-core (likely
alongside `forward/passive.rs`, since the integration point is in
`mj_fwd_passive`). A `LangevinThermostat` struct + an
`install(self: Arc<Self>, model: &mut Model)` method that wires up the
callback. Opt-in, default off (the model's `cb_passive` field defaults
to `None`).

**RNG ownership**: `cb_passive` is `Fn`, not `FnMut`, so the closure
cannot capture mutable RNG state directly. The `LangevinThermostat`
struct owns its RNG behind interior mutability (`Mutex<PRNG>`); the
callback closure captures `Arc<LangevinThermostat>` and dereferences it
each call. This gives thread-safety (matches the `Send + Sync` bound on
the callback), reproducibility (RNG is per-instance, seedable), and
local state (no `Data` mutation needed). PRNG choice is recon item 4
(still open) — placeholder.

**API sketch**:
```rust
let thermostat = Arc::new(LangevinThermostat::new(
    /* gamma  */ DVector::from_element(model.nv, 0.1),
    /* k_b_t  */ 1.0,
    /* seed   */ 42,
));
thermostat.install(&mut model);
loop { data.step(&model)?; }   // plain step(), no split-step needed
```

**Model setup**: Set `dof_damping = 0` on the thermostatted DOFs (the
thermostat owns damping; model damping would compound). Use
`Integrator::Euler`. The thermostat is integrator-agnostic in principle —
ImplicitFast / Implicit will work via the same `qfrc_passive` path —
but Phase 1 only validates against Euler.

**Validation** (sketch — *the Phase 1 spec will revise the parameter set*):
1-DOF damped harmonic oscillator with `M=1`, `k_spring=1`, `γ=0.1`,
`k_B·T=1`, `h=0.001`. Measure `⟨½ M qvel²⟩` and assert it equals
`½ k_B·T` within sampling-error tolerance. Then sweep
`γ ∈ {0.01, 0.1, 1.0}` and `k_B·T ∈ {0.5, 1.0, 2.0}` to confirm
linear scaling in `T` and γ-independence of the stationary
temperature. Passing this is the gate to all of Phase 2+.

> **Correction (chassis design Decision 5, 2026-04-09; tolerance
> wording corrected 2026-04-09 doc review M1)**: the original
> "10⁵ steps gives ~10⁻² sampling tolerance" calculation from
> recon log part 2 was *too optimistic by ~20×*. It assumed
> independent samples; actual samples from the Markov chain are
> correlated. For γ=0.1, M=1, h=0.001, the velocity-squared
> autocorrelation time is `τ_int ≈ M/(2γh) = 5000 steps`, giving
> an *effective* sample count of `N_eff ≈ 10⁵ / (1 + 2·5000) ≈ 10`
> for a 10⁵-step run. With `½Mv²` chi-squared distributed (mean
> `½kT`, standard deviation `(½kT)·√2`), the standard error of
> the sample mean over `N_eff = 10` effective samples is
> `(½kT)·√2/√10 ≈ 0.45 · (½kT)` — about **±45% relative to the
> expected mean ½kT**, not ±2%. **Tolerance convention** (locked
> in by this correction): in this document and all downstream
> validation specs, sampling-error tolerances are always expressed
> as a fraction of the expected mean (here `½kT`), not as a
> fraction of `kT`. The earlier "±22%" wording confused the two
> by a factor of 2. The Phase 1 spec will choose between three
> fixes: **(α)** ~100× longer trajectories, **(β)** ~100
> *independent* trajectories of 10⁵ steps each averaged across
> runs — avoids autocorrelation analysis entirely; weak lean;
> brings the std error to ~4.5% of `½kT`, **(γ)** loosen tolerance
> to ~5-10% of `½kT`. See `THERMO_CHASSIS_DESIGN.md` Decision 5
> for the full reasoning.

**Phase 5+ caveats** (flagged here, addressed in later phases):
1. **Derivatives / FD perturbation** — `cb_passive` fires inside
   `forward_skip()` because `mj_fwd_passive` is called from `forward_acc`,
   which `forward_skip` calls. **RESOLVED at the chassis level by
   doc review M2 → Decision 7 (2026-04-09)**: `PassiveStack` exposes
   `disable_stochastic() -> StochasticGuard<'_>`, an RAII guard that
   sets every `Stochastic` component to deterministic-mode for the
   guard's lifetime and restores prior state on drop. Phase 5 FD
   loops wrap their perturbation block in the guard; the perturbed
   and baseline runs both produce zero noise, so the FD difference
   recovers `∂F_det/∂qpos` exactly. See `THERMO_CHASSIS_DESIGN.md`
   Decision 7 for the full reasoning and the Scheme A vs. Scheme B
   trade-off.
2. **Plugin passive forces fire after `cb_passive`** (`passive.rs:723-731`).
   If a model has plugins contributing passive forces, the thermostat's
   noise will be present when the plugin runs. Not a Phase 1 issue (no
   plugins) but worth being aware of.
3. **BatchSim parallel envs** need *one* `LangevinThermostat` instance
   per env, not a shared instance. Avoids lock contention on the RNG
   mutex and keeps RNG streams independent across envs. **Resolved at
   the chassis level by Decision 3** (`install_per_env` factory +
   defensive clear).

### Phases 2–7

Sketches only at this point. Each will be fleshed into a child spec when the
preceding gate is green.

- **Phase 2**: Test the thermostat on a free body (3 translational + 3
  rotational DOF) and on a 2-link articulated chain with a hinge. The
  articulated case is where constraint-projection effects on noise become
  real (see Open Question Q1).
- **Phase 3**: Build a single bistable element in cf-design — start with a
  buckling beam or a parameterized double-well potential well. Measure
  switching rates as a function of barrier height and `T`; compare against
  Kramers' escape-rate formula.
- **Phase 4**: Couple 4–8 bistable elements via elastic linkages. Measure the
  joint state distribution; compare against analytical Ising or against a CPU
  Gibbs sampler on the same energy.
- **Phase 5**: Make the geometry/coupling differentiable through cf-design and
  hook the autograd engine to it. Train the array as an EBM to match a target
  distribution (start with a 2D Gaussian mixture). **Gates on Q5** (cf-design
  end-to-end differentiability) — escalated to active foreground recon by doc
  review M3, scheduled in parallel with the Phase 1 spec drafting so the
  build order past Phase 1 is informed by the answer rather than blind to it.
- **Phase 6**: Bridge to THRML or implement native. **Q3 RESOLVED 2026-04-09
  (doc review S1)**: original `extropic-ai/thrml` (JAX) exists; two community
  Rust ports exist on GitHub (`SashimiSaketoro/thrml-rs`,
  `Pingasmaster/thrml-rs`), neither officially maintained by Extropic, neither
  apparently on crates.io. Three Phase 6 options now in scope: (A) depend on
  a community port, (B) implement a minimal native block-Gibbs sampler in
  Rust (leading direction, ~few hundred LOC, sharpen-the-axe consistent),
  (C) vendor / fork a port into a `sim-thrml` sibling crate. See Q3 in §5
  for the full trade-off.
- **Phase 7**: Wrap the sampler as an ml-bridge environment. Reward = sample
  quality. Train a controller to improve mixing.

---

## 5. Open Questions / Unknowns

These need answers before the relevant phase can start. Numbered for
referenceability.

- **Q1 — Noise vs. constraint projection.** When stochastic forces are
  written into `qfrc_applied` and projected by the constraint solver, what is
  the effective temperature on constrained DOFs? There is real literature on
  constrained Langevin dynamics (Lelièvre, Stoltz, *Free Energy Computations*)
  — needs a half-day read before Phase 2 can claim a meaningful equipartition
  test on articulated bodies.
- **Q2 — Implicit integrator interaction.** **RESOLVED 2026-04-09 (part 2).**
  Recon of `forward/mod.rs` and `integrate/mod.rs` showed that the canonical
  force-injection point is `qfrc_applied` between `step1()` and `step2()`,
  not a velocity update outside the integrator. Forces written there are
  folded into `qfrc_smooth` and projected by the constraint solver before
  `qacc` is computed; the `integrate()` step then propagates the noise through
  `qvel`. This is integrator-agnostic (Euler, ImplicitFast, Implicit all work;
  RK4 is excluded by the split-step API itself, which is fine for Langevin).
  No BAOAB or integrator bypass needed for Phase 1. See Recon Log entry
  2026-04-09 part 2 for the full trace.
- **Q3 — Does `thrml-rs` exist?** **RESOLVED 2026-04-09 (doc review S1)
  via web search.** Answer is *"yes, but with caveats."*
  - **Original THRML exists**: `extropic-ai/thrml` on GitHub — a JAX
    library, "Thermodynamic Hypergraphical Model Library," focused on
    block Gibbs sampling on sparse heterogeneous graphs and discrete
    EBMs. Confirms the original prompt's premise.
  - **Two community Rust ports exist**, both on GitHub, *neither
    apparently published to crates.io*:
    1. `SashimiSaketoro/thrml-rs` — described as a pure Rust
       implementation of GPU-accelerated sampling for PGMs, with a
       CPU/GPU/hybrid runtime abstraction (`GpuFast`, `CpuPrecise`,
       `Adaptive` routing modes).
    2. `Pingasmaster/thrml-rs` — "1:1 safe Rust rewrite of the JAX
       version. Compatibility is NOT guaranteed. Speed ~2x better on
       CPU, way worse on GPU."
  - **Neither port is officially maintained by Extropic.** Both are
    community ports of unknown maturity. Pingasmaster's explicit
    "compatibility not guaranteed" warning is a red flag for
    correctness-critical use.

  **Implication for Phase 6**: the original "bridge to THRML or
  implement native" framing remains correct, but the trade-off is
  richer than "exists / doesn't exist." Three options for Phase 6:
  - **(A) Depend on one of the community ports** — fastest path to a
    working bridge, but inherits unknown maintenance status and the
    "compatibility not guaranteed" risk. Pingasmaster's port is
    likely more stable based on the description; Sashimi's has the
    GPU story.
  - **(B) Implement a minimal native block-Gibbs sampler** in Rust
    (~few hundred LOC) — most consistent with the sharpen-the-axe
    discipline ("best plans, best materials"). No external dep risk,
    natively integrates with the `sim-thermostat` trait shape, full
    control. Slower to ship.
  - **(C) Vendor / fork one of the ports into a sibling crate**
    (`sim/L0/thrml/` → `sim-thrml`) — middle ground. Inherits the
    initial implementation, owns maintenance going forward, can
    adapt to the chassis (e.g., implement `PassiveComponent` /
    `Stochastic` directly).

  **No commitment yet**, but the leading direction is **(B)** —
  consistent with the same "narrow crates, full control, no external
  silent-failure dependencies" reasoning that drove items 4 and 8 of
  the recon round (`ChaCha8Rng` over `StdRng`, `sim-thermostat`
  sibling crate over sim-core integration). Phase 6 spec will decide.

  **What this changes about the build order**: nothing for Phases
  1-5. Q3 was a Phase 6 question and remains a Phase 6 question; the
  early resolution just sharpens the trade-off the Phase 6 spec will
  face. The "5-minute web search" was the right move because deferring
  it preserved an artificial "exists / doesn't exist" binary that
  hid the real "three options" structure.
- **Q4 — Default `dof_damping` is zero.** **RESOLVED 2026-04-09 (part 2) →
  option (a).** Thermostat carries its own `γ_thermostat[i]` parameter and
  writes both `−γ·qvel` *and* the FDT-paired noise into `qfrc_applied`. Reason:
  `model.implicit_damping` is the canonical per-DOF damping vector populated
  from `jnt_damping` at model init (`model_init.rs:911`) and consumed by
  Eulerdamp, the constraint solver, the implicit-fast `qacc` computation, and
  the derivatives engine. It is model-owned state. Mutating it per step would
  conflate physical damping (joint friction) with thermodynamic damping (FDT
  pairing). Thermostat-owned `γ` keeps the two cleanly separated.
- **Q5 — Is the cf-design → sim-core parameter pipeline already
  differentiable end-to-end?** Phases 5+ depend on it, *and* the
  D3 co-design experiment (the headline Research Direction) does
  too. **Status (updated 2026-04-09 by doc review M3)**: escalated
  from "deferred until Phase 5" to **active foreground recon,
  scheduled in parallel with the Phase 1 spec drafting**. Target
  resolution: before the Phase 1 spec is finalized, so the
  Phase 1+ build order is informed by the answer rather than
  committing to it blind.

  **Why escalated**: the asymmetric risk argument. If Q5 turns
  out "yes, cf-design is fully differentiable end-to-end," the
  cost of the early recon was a half-day and the build order
  is unchanged — no harm done. If Q5 turns out "no" (e.g.,
  non-differentiable booleans, FD-only past a certain layer,
  no autograd hookup at all), the build order *changes
  substantially*. Plausible reactions to a "no":
  1. Build the differentiable layer earlier — potentially
     before D1 (Brownian motor), reordering the priority ladder.
  2. Re-prioritize the Research Directions away from D3 toward
     D1+D2+D4, which need much less differentiability.
  3. Use a surrogate model (e.g., a small neural net trained
     on cf-design output) and explicitly accept the boundary.
  4. Implement the differentiable layer as a sim-thermostat-style
     sibling crate that wraps cf-design with custom autograd.

  Discovering this *after* months of Phases 1-4 commitment is
  the kind of avoidable surprise sharpen-the-axe forbids. The
  cost of doing the recon now is small; the cost of doing it
  late is potentially months.

  **Recon scope** (what to read, ~half-day):
  - `crates/cf-design/src/` — locate the autograd integration
    point (if any). Look for `Tensor`, `Variable`, `grad`,
    `backward`, or interop with `sim-ml-bridge`'s autograd
    engine.
  - The cf-design Phase 5 commit history (per project memory:
    "cf-design Phases 1–5 complete (including differentiable
    design optimization)") — read the spec for the
    differentiable design optimization phase.
  - SDF library composition: are booleans (union, difference,
    intersection) differentiable, or do they introduce
    non-differentiable kinks at the boundary?
  - Mesh extraction: marching cubes or a smooth alternative?
    Marching cubes is non-differentiable at the topology
    boundary.
  - Existing examples or tests that exercise the
    cf-design → sim-core parameter flow with gradients.

  **Recon log entry**: will be opened when the recon starts,
  named "2026-04-XX (part N) — Q5: cf-design end-to-end
  differentiability". This question is the next item on the
  recon queue after the Phase 1 spec is in flight.
- **Q6 — What's the right reward signal for Phase 7?** ESS, integrated
  autocorrelation time, KL divergence to target, wall-clock to convergence?
  Different choices give different agents. Defer the decision to Phase 6
  results.

---

## 6. Spec Index

Child specs spawned from phases of the Gap will be linked here as they are
written.

- [`THERMO_CHASSIS_DESIGN.md`](./THERMO_CHASSIS_DESIGN.md) — bolt-pattern
  design document for the `sim-thermostat` crate. Defines the
  `PassiveComponent` trait, the `PassiveStack` builder + composer, the
  clone-footgun resolution via `install_per_env`, the orthogonal
  `Diagnose` trait, the `test_utils` chassis (Welford's algorithm),
  and the on-disk crate layout. Six decisions (1-6), all RESOLVED.
  Read this *before* the Phase 1 spec — every Phase 1 implementation
  detail bolts into the chassis defined here.
- *(Phase 1 spec — `PHASE_1_LANGEVIN_THERMOSTAT_SPEC.md` — is the next
  artifact to produce, drafted in a fresh session against the chassis
  design above.)*

---

## 7. Recon Log

Dated entries documenting what we learned and how the plan changed. Append,
don't rewrite. This is the record of *why* the plan looks the way it does.

### 2026-04-09 — Initial scaffold

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

### 2026-04-09 — Research Directions added

- **Trigger**: User asked whether the existing sim + RL stack could combine
  with the proposed thermo layer to do interesting research.
- **Result**: Section 2 (Research Directions) added, capturing five
  concrete experiments + one foundational connection. Sections 3–7
  renumbered accordingly.
- **Why these five**: Each is grounded in real published physics
  (Astumian/Magnasco for ratchets, Benzi/Wiesenfeld for SR, Pfeifer/Iida
  for morphological computation, Bennett/Landauer for reversible
  computation), each is achievable with the planned stack, and each is
  visually demonstrable. D3 (co-design) is the headline because it is the
  cleanest demonstration that all four CortenForge subsystems multiply.
- **Why the priority ladder**: Earliest payoff first (D1 — Brownian motor
  produces a stunning visual after only Phase 3). Headline experiment
  second (D3). Moat experiment (D4 — sim-to-real on a printed device) is
  highest stakes, longer horizon. Moonshot (D5) only after D1–D4 work.
- **Foundational connection flagged**: max-ent RL energy function +
  physical thermodynamic device energy function may be unifiable as a
  single shared `E(s,a)`. Needs a lit review before any novelty claim.
  Tracking item, not yet a phase.
- **No change to build order**: Phases 1–7 in The Gap are unchanged. The
  Research Directions inform *which experiments* to run within each phase,
  not the phase order itself.
- **Next action unchanged**: still draft the Phase 1 spec.

### 2026-04-09 (part 2) — Forward step + integrator code recon

- **Trigger**: Read `forward/mod.rs`, `integrate/mod.rs`, `integrate/euler.rs`,
  and traced `implicit_damping` usage across the crate to resolve Q2 and
  tighten Q4 before drafting the Phase 1 spec.
- **Pipeline structure**: `step()` → `forward()` → `integrate()`. The forward
  pipeline is split into `forward_pos_vel()` (FK, collision, velocity FK,
  energy, sensors) and `forward_acc()` (actuation, RNE, passive, constraint
  solve, qacc). `cb_control` fires at the boundary between them.
- **Documented force-injection API**: `step1()` runs `forward_pos_vel()` +
  `cb_control`; `step2()` runs `forward_acc()` + `integrate()`. The user is
  *expected* to write into `ctrl`, `qfrc_applied`, or `xfrc_applied` between
  step1 and step2. The doc comment on `step1` literally calls this out as
  the RL force-injection point. The thermostat is just another consumer of
  this hook. RK4 is excluded by design (multi-stage substeps recompute
  `forward()` internally) — fine, since Langevin + RK4 is mathematically
  problematic anyway.
- **Force flow trace**: `qfrc_applied[i] += …` → folded into `qfrc_smooth` in
  `constraint/mod.rs:78-87` (`qfrc_smooth = qfrc_applied + qfrc_actuator +
  qfrc_passive − qfrc_bias`, plus `xfrc_applied` projection) → constraint
  solver computes `qfrc_constraint` → `mj_fwd_acceleration` computes `qacc`
  with `M_hat = M − h·∂f/∂v` for implicit integrators → `integrate()` does
  `qvel += qacc * h` (or the Eulerdamp `(M + h·D)·qacc_new = rhs` solve when
  `implicit_damping > 0`). Position integration `qpos += qvel * h` then runs
  in `mj_integrate_pos` with proper SO(3) handling for ball/free joints, and
  `mj_normalize_quat` cleans up quaternion drift.
- **`implicit_damping` is model-owned state**: Populated from `jnt_damping`
  at model init (`model_init.rs:911`). Consumed by Eulerdamp
  (`integrate/mod.rs:85,94,136`), the constraint solver
  (`constraint/mod.rs:149,221`), implicit-fast acceleration
  (`forward/acceleration.rs:103`), and the derivatives engine
  (`derivatives/hybrid.rs:78`). Mutating it per step would couple thermostat
  damping to model damping in ways that break separation of concerns.
- **Free wake-up bonus**: `island/sleep.rs:132-150,522-539` already inspects
  both `qfrc_applied` and `xfrc_applied` for wake detection. **A sleeping
  body receiving thermal noise wakes up automatically.** No thermostat-
  specific logic needed for islanded simulations. Correctness win for free.
- **Q2 RESOLVED**: Use `qfrc_applied` injection between step1/step2. No
  BAOAB, no integrator bypass. Integrator-agnostic across Euler /
  ImplicitFast / Implicit.
- **Q4 RESOLVED (option a)**: Thermostat owns its own `γ_thermostat[i]` and
  writes both `−γ·qvel` and the FDT noise into `qfrc_applied`. Model damping
  stays untouched.
- **Phase 1 algorithm sharpened**: Explicit Langevin (Euler-Maruyama). For
  `M=1, k_spring=1, γ=0.1, k_B·T=1, h=0.001`: natural frequency `ω=1`,
  `h·ω=0.001`, damping ratio `ζ=γ/(2√(kM))=0.05` (underdamped, healthy
  mixing), discretization temperature error `O(h·γ/M) ≈ 10⁻⁴` — well below
  the `~10⁻²` sampling-error tolerance for 10⁵ samples. Equipartition test
  should pass with margin.
- **Did NOT resolve**: Q1 (constraint projection — irrelevant for 1-DOF
  Phase 1, gates Phase 2), Q3 (`thrml-rs` existence — gates Phase 6),
  Q5 (cf-design end-to-end differentiability — gates Phase 5), Q6 (Phase 7
  reward signal — defer until Phase 6 results inform the choice).
- **Phase-1-blocking recon items still open** (small, listed in the post-
  recon report): PRNG conventions, `qfrc_applied` lifecycle / auto-clearing,
  existing usage patterns for writing into `qfrc_applied`, `model.timestep`
  variability, statistical-test infrastructure conventions, where the
  thermostat module should live in the crate, public mutability of
  `Data::qfrc_applied`.
- **Next action**: Answer the Phase-1-blocking recon items (small focused
  recon round), *then* draft the Phase 1 spec.

### 2026-04-09 (part 3) — Phase-1-blocking item 2: `qfrc_applied` lifecycle

- **Trigger**: First item in the Phase-1-blocking recon round, taken under
  the sharpen-the-axe discipline (one item at a time, fully logged before
  moving on). Picked first because it is the most correctness-critical of
  the open items and it shapes the thermostat API.
- **Question**: Does the forward pipeline auto-clear `qfrc_applied` at the
  start of each step, or does the user own clearing? Answer determines
  whether the thermostat does `=` (overwrite) or `+=` (accumulate), and
  shapes the API for composing the thermostat with future writers.
- **Finding**: `qfrc_applied` is **user-owned and persistent across steps**.
  The forward pipeline never clears it. It is only zeroed in two places,
  both reset functions:
  - `Data::reset()` at `data.rs:1123` (full reset, in the "force vectors —
    zero" block)
  - `Data::reset_to_keyframe()` at `data.rs:1245` (keyframe reset, with
    explicit doc-comment: *"Clears derived quantities (...) and user-applied
    forces (qfrc_applied, xfrc_applied) — matching the convention of
    Data::reset()"*)

  Initialization at `model_init.rs:540` is `DVector::zeros(self.nv)`. No
  per-step clear anywhere in `forward/`, `integrate/`, or `step()`.
- **Convention is documented loudly** in the batch-sim API
  (`batch.rs:106-110`): *"Use this to set ctrl, qfrc_applied, xfrc_applied,
  or any other input field before calling step_all()."* And
  `batch.rs:171-175`: *"[reset_one] does **not** zero qfrc_applied /
  xfrc_applied (see Data::reset documentation). Callers must zero these
  explicitly if needed."* `inverse.rs:23-25` confirms the categorization:
  `qfrc_applied + qfrc_actuator + J^T*xfrc_applied = "the total
  user-supplied generalized forces"`. This matches MuJoCo's convention
  exactly: external forces persist to model constant external loads; user
  owns the lifecycle.
- **Implication**: Thermal noise is i.i.d. per step. Accumulating across
  steps without clearing is wrong physics — `qfrc_applied` would random-walk
  and the equilibrium temperature would drift upward over time (variance
  compounding). The thermostat must either overwrite, clear-then-accumulate,
  be the only writer, or use a separate field.
- **Two schemes compared** (per sharpen-the-axe rule "Two schemes, then
  choose"):
  - **Option A — Sole-writer overwrite**: Thermostat does
    `data.qfrc_applied[i] = -γ·qvel + noise`. Documented contract:
    *"LangevinThermostat is the sole writer of qfrc_applied while active.
    For additional forces, use xfrc_applied or ctrl."*
    - **Pros**: Foolproof — no silent compounding possible by construction.
      A-grade enforceable (debug-assert invariant in tests). Zero cost for
      Phase 1 validation (no other forces in the test). Failure mode (a
      user trying to compose) is **loud and immediate** — they see their
      force vanish.
    - **Cons**: Doesn't compose with controllers / RL policies that also
      want to write `qfrc_applied`. Future RL/controller users must use
      `xfrc_applied` or `ctrl` instead, or wait for an upstream redesign.
  - **Option B — Additive, caller clears**: Thermostat does `+=`. Caller
    must `data.qfrc_applied.fill(0.0)` each step before calling `apply()`.
    - **Pros**: Composable with other writers. Matches MuJoCo's "user owns
      lifecycle" convention exactly.
    - **Cons**: **Silent correctness failure** if caller forgets to clear.
      Early steps would still pass equipartition (per-step variance is
      right), but temperature would drift upward as the random walk in
      `qfrc_applied` accumulates. Failure mode is invisible, delayed, and
      exactly the hardest kind of bug to debug.
- **Decision: Option A for Phase 1.** Reasoning:
  1. Option B's failure mode (silent, delayed, wrong physics that passes
     early validation) is precisely what sharpen-the-axe is meant to
     prevent. Option A makes that failure mode impossible by construction.
  2. Phase 1 has zero composability requirement — the 1-DOF damped harmonic
     oscillator validation has no other forces. Designing the more flexible
     API for a feature we don't yet need is premature optimization, *and
     the flexibility is itself the source of the bug*.
  3. Composability is a Phase 4-7 concern; we'll know what the right
     interface looks like by then. Designing it now based on guesses is
     speculative scaffolding.
  4. Migration A → C (see below) is a focused localized change. Migration
     B → A would be a breaking change after users have written code against
     additive semantics — much more painful.
  5. Option A's downside is loud and immediate; Option B's is silent and
     delayed. Loud failures are cheap, silent failures are expensive.
- **Forward-looking — Option C, NOT for Phase 1**: The eventual right
  answer for Phase 4+ composability is probably a dedicated
  `qfrc_thermostat: DVector<f64>` field on `Data`, summed alongside
  `qfrc_applied` in the canonical aggregation in `constraint/mod.rs:78-87`.
  This preserves `qfrc_applied` for user/RL writes while giving the
  thermostat its own clean slate. Invasive (modifies the canonical
  force-sum, requires upstream sim-core support, touches the constraint
  module). Explicitly out of scope for Phase 1. Flag in the Phase 1 spec
  as the eventual end-state; revisit when Phase 4 (coupled bistable
  arrays) starts running into composition needs.
- **Item 2 RESOLVED**: Phase 1 thermostat uses sole-writer overwrite
  semantics. Documented contract, debug-assert invariant in tests, Option
  C flagged as future work.
- **What item 2 leaves open for item 3**: Whether a `cb_passive` (or
  similar passive-force) callback hook exists. If it does, the thermostat
  might more idiomatically write into `qfrc_passive` instead, sharing the
  existing damping/spring infrastructure — which would change the Option
  A/C tradeoff entirely. This question naturally belongs to item 3
  (existing usage patterns for writing applied forces). If found, revisit
  item 2's decision before drafting the Phase 1 spec.
- **Next action**: Phase-1-blocking recon item 3 — existing usage patterns
  for writing into `qfrc_applied`, including the `cb_passive` hook
  question.

### 2026-04-09 (part 4) — Item 3 first finding: `cb_passive` exists, item 2 REVISED

- **Trigger**: Started item 3 (existing usage patterns for writing into
  `qfrc_applied`) under sharpen-the-axe. The very first searches for
  `cb_passive` returned hits, exposing a documented hook that materially
  changes item 2's decision. Stopped item 3 immediately to surface the
  finding, per the "stop and ask when uncertain" principle.
- **What `cb_passive` is** — a documented user passive-force callback hook
  on `Model`, analogous to `cb_control`. Citations:
  - `model.rs:1002` — `pub cb_passive: Option<super::callbacks::CbPassive>`
  - `model.rs:1242, 1247` — setter and clearer methods
  - `model_init.rs:396` — initialized to `None` (opt-in by default)
  - `callbacks.rs:37-41` — type definition + contract
  - `forward/passive.rs:719` — invocation site

  Type: `Callback<dyn Fn(&Model, &mut Data) + Send + Sync>` — `Fn` (not
  `FnMut`), thread-safe, `Send + Sync` for cross-thread sharing in
  BatchSim.

  Documented contract (`callbacks.rs:37-41`): *"Passive force callback:
  called at end of `mj_fwd_passive()`. Use to inject custom passive forces
  (e.g., viscous drag, spring models). The callback may modify
  `data.qfrc_passive` or any other Data field."*
- **What `qfrc_passive` is** — engine-owned, **auto-cleared every step**.
  In `forward/passive.rs`:
  - Line 368 — `mj_fwd_passive()` *starts* by zeroing `qfrc_passive`,
    `qfrc_spring`, `qfrc_damper`, `qfrc_fluid`, `qfrc_gravcomp`
    (sleep-aware: only awake DOFs cleared if sleeping).
  - Lines 388–665 — spring, damper, tendon, flex, bending, fluid, and
    gravcomp forces accumulate into per-component arrays.
  - Line 681 — aggregation: `qfrc_passive = qfrc_spring + qfrc_damper`
    (assignment, not accumulation).
  - Lines 683–706 — gravcomp and fluid added optionally.
  - Line 719 — `cb_passive` fires *after* aggregation; the callback sees
    the populated `qfrc_passive` and can `+=` into it freely.
  - Lines 723–731 — plugin passive forces fire *after* `cb_passive`.

  `qfrc_passive` then flows into `qfrc_smooth` via
  `constraint/mod.rs:78-87` — same path as `qfrc_applied`, same projection
  by the constraint solver, same wake-up support.
- **Why this changes item 2**: Item 2's Option A (sole-writer overwrite of
  `qfrc_applied`) was forced by the fact that `qfrc_applied` is user-owned
  and persistent across steps — any additive scheme could silently compound
  noise. `qfrc_passive` is the opposite: engine-owned, auto-cleared. The
  thermostat can `+=` into it with zero risk of compounding because the
  next step starts from zero by construction. **The Option A vs B
  trade-off dissolves entirely.**
- **And the ontology is correct**: Langevin damping + FDT-paired stochastic
  forcing is *literally a passive thermal force*. The right sim-core
  categorization is:
  - `qfrc_actuator` — controlled actuator forces (motors, muscles)
  - `qfrc_passive` — passive forces (springs, damping, fluid drag,
    **thermal bath**)
  - `qfrc_applied` — external user-applied forces (RL action injection,
    manual loads)
  - `qfrc_constraint` — constraint reaction forces

  The thermostat belongs in `qfrc_passive`. Item 2 was solving the right
  problem in the wrong category.
- **Trade-off comparison** (cb_passive vs item 2 Option A):

  | Concern | Option A (qfrc_applied overwrite) | cb_passive + qfrc_passive |
  |---|---|---|
  | Compounding-safe? | Yes (overwrite) | Yes (auto-cleared) |
  | Composes with RL writers of `qfrc_applied`? | No | Yes (different field) |
  | Composes with multiple thermostats / passive plugins? | N/A | Yes |
  | Requires split-step API? | Yes (manual hook) | No (`step()` works) |
  | Idiomatic? | Hijacks user-input field | Uses the documented hook for exactly this purpose |
  | Forward-looking Option C still needed? | Yes, eventually | No — composability achieved already |

  cb_passive dominates on every axis except one — the RNG constraint —
  which is solvable.
- **The RNG constraint and its solution**: `cb_passive` is `Fn`, not
  `FnMut`. The closure cannot capture mutable state directly. Solution:
  the `LangevinThermostat` struct owns its state behind interior
  mutability, the closure captures `Arc<LangevinThermostat>`:

  ```rust
  pub struct LangevinThermostat {
      gamma: DVector<f64>,
      k_b_t: f64,
      rng: Mutex<ChaCha8Rng>,  // PRNG choice = recon item 4
  }

  impl LangevinThermostat {
      pub fn install(self: Arc<Self>, model: &mut Model) {
          let me = Arc::clone(&self);
          model.set_passive_callback(move |m, d| me.apply(m, d));
      }

      fn apply(&self, model: &Model, data: &mut Data) {
          let h = model.timestep;
          let mut rng = self.rng.lock().unwrap();
          for i in 0..self.gamma.len() {
              let g = self.gamma[i];
              let damping = -g * data.qvel[i];
              let noise = (2.0 * g * self.k_b_t / h).sqrt()
                  * standard_normal(&mut *rng);
              data.qfrc_passive[i] += damping + noise;
          }
      }
  }
  ```

  Properties: thread-safe (matches `Send + Sync` bound), reproducible
  (RNG is per-instance, seedable), local (no `Data` mutation needed).
  PRNG choice is recon item 4 — placeholder.
- **Caveats for the spec** (NOT Phase 1 blockers; flagged for later):
  1. **Derivatives / FD perturbation** — `cb_passive` *will* fire inside
     `forward_skip()` because `mj_fwd_passive` is called from
     `forward_acc`, which `forward_skip` calls. For deterministic
     derivative computation in Phase 5, the thermostat needs gating or
     RNG snapshot/restore. Phase 1 doesn't compute derivatives.
  2. **Plugin passive forces fire after `cb_passive`** — not a Phase 1
     issue (no plugins), but worth flagging in the spec.
  3. **Mutex lock contention** — one lock per step is negligible for
     single-env sims. For BatchSim parallel envs, each env needs its
     OWN thermostat instance with its own RNG, not a single shared
     thermostat. The API should make the wrong choice hard.
- **DECISION REVISED**: Item 2's Option A is **superseded**. The Phase 1
  thermostat lives in `cb_passive` and writes into `qfrc_passive` with
  accumulation semantics. The forward-looking Option C (dedicated
  `qfrc_thermostat` field) is no longer needed — `cb_passive` already
  achieves the composability goal. Item 2's log entry remains for
  historical reasoning; the decision-trail is the value, not just the
  current decision.
- **Phase 1 section in The Gap UPDATED in this same commit** to reflect
  the new algorithm. The old text described the `qfrc_applied` approach
  and is no longer accurate. Reading the master plan top-to-bottom now
  yields a consistent design.
- **Item 3 status**: First finding extracted (cb_passive exists — the
  most consequential bit). Item 3 is **otherwise still open**: we still
  need to read existing patterns for writing into `qfrc_applied`
  (`ml-bridge/src/space.rs`, `tests/integration/split_step.rs`,
  `examples/coupled_pendulums.rs`) to understand how RL controllers
  inject forces, even though the thermostat doesn't go through that
  path — the knowledge is relevant for Phase 7 composition. That recon
  happens in a fresh session.
- **Next action (fresh session)**: Continue item 3 — existing
  `qfrc_applied` usage patterns in `ml-bridge/src/space.rs`,
  `tests/integration/split_step.rs`, and `examples/coupled_pendulums.rs`.
  Then proceed to items 4–7 (PRNG conventions, `Data` field public
  mutability, `model.timestep` variability, statistical-test
  conventions, thermostat module location). After all Phase-1-blocking
  items are answered, draft `PHASE_1_LANGEVIN_THERMOSTAT_SPEC.md`.

### 2026-04-09 (part 5) — Item 3 closed: existing `qfrc_applied` write patterns

- **Trigger**: Continuation of recon item 3, the *rest* of it. Part 4
  closed the `cb_passive` sub-question and revised the Phase 1 design;
  this entry closes the remainder by reading the three files cataloging
  how RL controllers and example code currently inject forces:
  `sim/L0/ml-bridge/src/space.rs`, `sim/L0/tests/integration/split_step.rs`,
  `sim/L1/bevy/examples/coupled_pendulums.rs`. The thermostat itself
  doesn't go through `qfrc_applied` anymore, so this is forward-looking
  recon for Phase 7 composition with RL — *not* Phase 1 design input.
- **Pattern 1 — ml-bridge `ActionSpace::apply`**
  (`ml-bridge/src/space.rs:599-768`, call site `env.rs:133-146`):
  - **Cadence**: action injection happens *before* `data.step()`, with
    plain `step()`. Not split-step. The `SimEnv::step()` body is literally
    `act_space.apply(action, &mut self.data, &self.model); for _ in 0..sub_steps { self.data.step(...)?; … }`.
  - **Semantics**: pure overwrite. `Injector::QfrcApplied` does
    `data.qfrc_applied[i] = f64::from(val)` (line 644). Same shape for
    `XfrcApplied` (line 651), `Ctrl` (line 639, with clamp), `MocapPos`,
    `MocapQuat`. There is no `+=` anywhere in the injector path.
  - **Range coverage**: each injector overwrites only the indices in its
    declared `Range<usize>`. **Indices outside the declared range are not
    cleared.** Safe today because `Data::reset()` zeros `qfrc_applied` and
    `SimEnv::reset()` calls `Data::reset()`, but the injector contract is
    "I own my range," not "I own the field."
  - **Five disjoint slots**: ctrl, qfrc_applied, xfrc_applied, mocap_pos,
    mocap_quat. ml-bridge composes multiple injectors *only* via field
    disjointness — never two writers to the same slot.
- **Pattern 2 — `split_step.rs` integration test**
  (`sim/L0/tests/integration/split_step.rs:87-121`):
  - The *only* in-tree consumer of the documented split-step force-injection
    cadence. Writes `data.qfrc_applied[0] = 5.0` between `step1()` and
    `step2()` and asserts the trajectory diverges from a baseline.
  - It is a **contract test** for the split-step API, not a usage
    cookbook. It does not represent the dominant idiom in the codebase.
- **Pattern 3 — `coupled_pendulums.rs` Bevy example**
  (`sim/L1/bevy/examples/coupled_pendulums.rs:359-394`):
  - **Cadence**: identical to ml-bridge — write before plain `step()`,
    no split-step. Inside the substep loop:
    `data.qfrc_applied[0] = coupling; data.qfrc_applied[1] = -coupling;
    data.step(&model)?;`
  - **Semantics**: overwrite with state-dependent value
    (`coupling = -KAPPA * (theta1 - theta2)`).
  - **Reset path is manual**: the restart handler does
    `qpos.fill(0.0); qvel.fill(0.0); qfrc_applied.fill(0.0); time = 0.0`
    explicitly, *bypassing* `Data::reset()`. The author *knows*
    `qfrc_applied` is persistent across steps and clears it explicitly.
    Confirms the cultural awareness of MuJoCo "user owns lifecycle."
- **Synthesis — three observations that matter for the thermostat:**
  1. **Both real-world writers (ml-bridge and coupled_pendulums) use
     plain `step()`, not split-step.** The `step1`/`step2` API is
     documented + tested but the codebase's actual writers don't reach
     for it; they write to `qfrc_applied` *before* `step()` and rely on
     overwrite the next iteration. **This validates the cb_passive
     design choice from part 4** — the thermostat will use plain
     `step()` too, joining the dominant convention. The split-step API
     remains a tool in the toolbox but is not the idiom.
  2. **Both real-world writers use `=`, not `+=`.** Each owns the index
     it cares about and overwrites it each step. Multiple writers to the
     *same* index would clobber each other; the codebase has no
     established protocol for that case because it has never needed one.
     **Field disjointness is the de facto composition idiom.**
  3. **The thermostat (`qfrc_passive`) and ml-bridge `ActionSpace`
     (`qfrc_applied`/`xfrc_applied`/`ctrl`) live in different fields.
     They cannot clobber each other.** Composition with RL is automatic
     and trivial — no API design needed in Phase 1, and Phase 7 inherits
     it for free. Document this in the Phase 1 spec under "Composition
     with other writers" so future readers understand it was *designed
     in*, not stumbled into.
- **Two surprises (neither blocks Phase 1, both flagged for the user):**
  1. **Doc/code drift on `step1()`'s doc-comment**. Part 2's recon noted
     that the `step1()` doc-comment "literally calls out [the
     step1/step2 hook] as the RL force-injection point." That doc is
     accurate as a *capability* statement but does not match the *actual
     ml-bridge code path*, which writes before plain `step()`. Not a
     bug, not a design issue, but the doc reads as if split-step is the
     RL idiom when in fact it isn't. Worth a doc fixup eventually; out
     of scope for this branch.
  2. **The thermostat will be the workspace's first `cb_passive`
     consumer.** `callbacks.rs:37-41` documents the API; nothing in
     `sim/`, `examples/`, or any test currently uses it. (Verified by
     the absence of any hits in the three target files and by the fact
     that Phase 1 design searches in part 4 found only the type
     definition + initialization sites, not call sites.) **Implication
     for the spec**: tests need to establish the precedent for
     callback-RNG reproducibility patterns. A small dedicated test —
     "the callback fires once per `step()`, RNG advances by exactly the
     expected count" — should be in the Phase 1 validation suite, not
     just the equipartition test.
- **Latent footgun noted (NOT Phase 1 relevant)**: The
  `Injector::QfrcApplied` contract is "I overwrite my range, I do not
  clear unrelated indices." Safe today via `Data::reset()`. If a user
  ever combines an `ActionSpace` with the escape-hatch `data_mut()` and
  bypasses reset, stale `qfrc_applied` values outside the action range
  would persist across episodes. Not introduced by anything thermostat-
  related; flagging for ml-bridge maintainers as a docs improvement
  opportunity. Out of scope for this branch.
- **Implications for the Phase 1 spec** (concrete, will appear in the
  spec when it gets drafted):
  - **Composition section**: state explicitly that the thermostat lives
    in `qfrc_passive` via `cb_passive`, that ml-bridge writers live in
    `qfrc_applied`/`xfrc_applied`/`ctrl`, and that field disjointness
    makes composition automatic. Cite the three patterns in this entry.
  - **Test section**: include a callback-firing-count test in addition
    to the equipartition test. Establishes the `cb_passive` test
    precedent for the workspace.
  - **Cadence section**: the canonical user pattern is plain `step()`.
    No split-step required. State this explicitly so a beginner reading
    the spec doesn't reach for `step1()`/`step2()` thinking it's
    necessary.
- **Item 3 RESOLVED.** Both halves are now closed: the `cb_passive`
  sub-question (part 4, design-changing) and the existing-pattern survey
  (this entry, design-confirming). No revisions to the Phase 1 design
  result from this entry — only documentation hooks for the spec.
- **Next action**: Phase-1-blocking recon item 4 — PRNG conventions in
  CortenForge. Question: which PRNG crate is the workspace standard
  (`rand` + `rand_chacha`? something else? a custom in-tree generator?),
  what seeding patterns are already used in tests/examples, and is
  there a documented thread-safe-RNG idiom. Answer determines the
  concrete type behind `Mutex<PRNG>` in the `LangevinThermostat` struct.

### 2026-04-09 (part 6) — Item 4: PRNG conventions

- **Trigger**: Recon item 4 under sharpen-the-axe. The placeholder
  `Mutex<ChaCha8Rng>` in part 4's API sketch needs to either match an
  existing convention or be deliberately chosen against one. Until that
  is settled, the Phase 1 spec cannot name a concrete RNG type.
- **Workspace declarations** (root `Cargo.toml:375-376`):
  ```toml
  rand = "0.9"
  rand_distr = "0.5"
  ```
  Plus `STANDARDS.md:316`: *"No duplicate functionality (e.g., both
  `rand` and `fastrand`)"*. The `rand` family is the canonical and only
  blessed RNG ecosystem in the workspace. `fastrand` appears only as a
  transitive dep of build tools (tempfile, etc.), never as a workspace
  dep. No custom in-tree PRNG.
- **Existing seeding pattern — single, uniform**: every seeded RNG in
  the codebase uses `rand::rngs::StdRng` with `seed_from_u64`. Hits:
  - `sim/L0/ml-bridge/src/cem.rs:14,139` — `StdRng::seed_from_u64(seed)`
  - `sim/L0/ml-bridge/src/sac.rs:20,262` — same
  - `sim/L0/ml-bridge/src/ppo.rs:16,219` — same
  - `sim/L0/ml-bridge/src/td3.rs:17,235` — same
  - `sim/L0/ml-bridge/src/reinforce.rs:17,163` — same
  - `sim/L0/ml-bridge/src/replay_buffer.rs:183,236` — same
  - `sim/L0/ml-bridge/src/autograd_policy.rs:1017,1038` — same
  - `sim/L0/ml-bridge/tests/competition.rs` — 9 seeded sites, all same
  - Bench files (`sim/L0/simd/benches/...`, `sim/L0/core/benches/...`)
    use `rand::rng()` (unseeded thread-local) — appropriate for
    benches, irrelevant to validation.
  - **Zero hits for `ChaCha8Rng`, `SmallRng`, `Pcg`, `Xoshiro`,** or
    any explicitly-named PRNG algorithm anywhere in `sim/`. The
    workspace is 100% `StdRng`.
- **The `StdRng` reproducibility hazard — already known to maintainers**.
  `sim/L0/ml-bridge/DEFERRED.md:47-53` explicitly flags this:

  > **Pin RNG algorithm** — Replace `StdRng` with an explicit algorithm
  > (e.g., `ChaCha8Rng`) across all algorithms and builders. `StdRng`
  > is documented as not reproducible across `rand` versions.
  > `Cargo.lock` pins it in practice, but an explicit algorithm guards
  > against silent result changes on dependency bumps. Mechanical
  > find-and-replace.

  In other words: ml-bridge already concluded that `StdRng` is wrong,
  proposed `ChaCha8Rng` as the replacement, and parked the migration as
  deferred work. The thermostat lands into a codebase that has the
  *intention* but not yet the *implementation* of the right convention.
- **`rand_chacha` is already a transitive dep**: `Cargo.lock` shows
  `rand_chacha 0.9.0` (and an old `0.3.1` from a build tool). It is
  pulled in transitively by `rand 0.9` itself. **Adding it as a
  top-level workspace dep is a no-cost organizational change**, not a
  new compile-time dep.
- **`rand_distr` is declared but unused** in production code. The only
  hit in `sim/` is a *negative* citation in
  `sim/L0/ml-bridge/POLICY_PERSISTENCE_SPEC.md:1237`:
  *"No `rand`/`rand_distr` dependency — CEM handles its own RNG
  internally..."*. The workspace declares `rand_distr = "0.5"` but no
  crate currently consumes it. **The thermostat will be the workspace's
  first `rand_distr` consumer** (for `StandardNormal`).
- **Thread-safe RNG idiom — none documented, none needed yet**. Every
  existing RNG owner in the codebase passes `&mut rng` through internal
  call chains within a single thread. ml-bridge gives each algorithm
  its own owned `StdRng`; replay buffers, policies, and tests all hold
  exclusive `&mut` references. There is no `Mutex<Rng>` anywhere in the
  workspace. The thermostat's `Mutex<PRNG>` (forced by `cb_passive`
  being `Fn` not `FnMut`) is **a new pattern for the codebase**.
- **`sim-core`'s rand dependency surface — important structural finding**:
  - `sim-core/Cargo.toml:24` has `rand = { workspace = true }` *only in
    `[dev-dependencies]`*. Production sim-core does **not** depend on
    `rand` at all.
  - `sim-core` does not depend on `rand_chacha` (transitively or
    directly).
  - `sim-core` does not depend on `rand_distr`.

  **If the thermostat lives inside sim-core**, all three will become
  production dependencies of sim-core. This is a non-trivial change to
  sim-core's surface area: a 100% deterministic physics core gains a
  stochastic RNG dep tree. Not a blocker — `rand` is a high-quality
  small dep — but it interacts with item 8 (thermostat module
  location). If item 8 ends up with the thermostat in a sibling L0
  crate (e.g., `sim-thermostat`), sim-core's deps stay clean and the
  RNG deps live where they are used. **This is the most consequential
  finding from item 4** and the user should make the call deliberately
  rather than have it follow from a default choice on item 8.

- **Two schemes compared** (per "Two schemes, then choose"):
  - **Scheme A — Inherit `StdRng`**. Use `rand::rngs::StdRng` with
    `seed_from_u64`, matching every other seeded RNG in the workspace.
    - **Pros**: Zero new deps. Consistent with the existing codebase
      pattern. Drop-in find/replace migration when ml-bridge eventually
      does its DEFERRED.md cleanup.
    - **Cons**: `StdRng` is documented as **not reproducible across
      `rand` versions**. The Phase 1 equipartition test is a
      *numerical validation gate* — the temperature must hit
      `½ k_B·T` to within `~10⁻²`. If a future `rand` bump silently
      changes `StdRng`'s underlying algorithm, the equipartition test
      result will shift. The shift may be small enough that the test
      still passes, but the *meaning* of "passing" will have changed
      between dependency bumps. Failure mode is **silent and delayed**
      — exactly the class sharpen-the-axe is meant to prevent (cf.
      part 3's reasoning against Option B for item 2).
  - **Scheme B — Use `ChaCha8Rng` directly**. Use
    `rand_chacha::ChaCha8Rng` with `seed_from_u64`. Add
    `rand_chacha = "0.9"` to workspace deps (organizational only —
    already in `Cargo.lock`).
    - **Pros**: Explicit algorithm = bit-stable across `rand` /
      `rand_chacha` versions = reproducibility guaranteed. Aligns with
      the documented future direction in `ml-bridge/DEFERRED.md`. The
      thermostat sets a clean precedent that the eventual ml-bridge
      migration follows rather than maintaining two patterns. ChaCha8
      is non-cryptographic-strength but has excellent statistical
      properties — overkill is fine, the cost is one extra ALU
      operation per draw which is negligible compared to a full
      `step()` of physics.
    - **Cons**: Adds `rand_chacha` to top-level workspace deps (no new
      compile dep, only a declaration). Thermostat is the first
      explicit consumer in the workspace — sets a precedent that ml-
      bridge has not yet adopted, which produces a brief two-pattern
      window until DEFERRED.md is done.
- **Recommendation: Scheme B (`ChaCha8Rng`)**. Reasoning:
  1. Reproducibility is non-negotiable for the Phase 1 validation
     gate. The equipartition test is the *foundation* the rest of the
     thermo line builds on; a silent reproducibility hazard at the
     foundation is exactly the failure mode sharpen-the-axe forbids.
  2. ml-bridge maintainers have already concluded `ChaCha8Rng` is the
     right answer; choosing it for the thermostat does not introduce a
     novel decision, only completes one already made.
  3. `rand_chacha` is already in `Cargo.lock` as a transitive dep, so
     the marginal compile cost is zero. The "new dep" is purely
     organizational.
  4. The "first explicit consumer" framing is actually a positive: the
     thermostat lands clean and the eventual ml-bridge migration has
     one pattern to copy, not two patterns to reconcile.
  5. ChaCha8 (8 rounds) is the right cipher choice for non-cryptographic
     statistical use — `ChaCha20Rng` is overkill, `ChaCha12Rng` adds
     half the cost for no statistical gain at this scale, and the
     DEFERRED.md note already names ChaCha8 as the target.
- **`rand_distr` choice (sub-decision)**: The thermostat needs Gaussian
  samples for the noise term. Two ways to get them in `rand_distr 0.5`:
  - `rand_distr::StandardNormal` — zero-parameter unit-variance normal,
    `(2γkT/h)^{1/2}` is applied as a multiplicative scale outside the
    sample. Cleaner: the thermostat owns the scale, the distribution
    just produces standard normals.
  - `rand_distr::Normal::new(0.0, sigma).unwrap()` — a parameterized
    distribution. Slightly more allocation per call (the distribution
    object) but lets `rand_distr` compute the scaled sample directly.

  **Choice**: `StandardNormal`. The scale `(2γkT/h)^{1/2}` is
  per-DOF (γ is per-DOF) and is recomputed per step (h depends on
  `model.timestep`, which is item 6's open question — could vary). A
  single `StandardNormal` distribution + per-DOF/per-step scale is the
  cleanest factoring. Document in the spec.
- **Concrete API update** for the part 4 sketch:
  ```rust
  use rand_chacha::ChaCha8Rng;
  use rand::SeedableRng;
  use rand_distr::{StandardNormal, Distribution};

  pub struct LangevinThermostat {
      gamma: DVector<f64>,
      k_b_t: f64,
      rng: Mutex<ChaCha8Rng>,
  }

  impl LangevinThermostat {
      pub fn new(gamma: DVector<f64>, k_b_t: f64, seed: u64) -> Self {
          Self {
              gamma,
              k_b_t,
              rng: Mutex::new(ChaCha8Rng::seed_from_u64(seed)),
          }
      }

      fn apply(&self, model: &Model, data: &mut Data) {
          let h = model.timestep;
          let mut rng = self.rng.lock().unwrap();
          for i in 0..self.gamma.len() {
              let g = self.gamma[i];
              let damping = -g * data.qvel[i];
              let z: f64 = StandardNormal.sample(&mut *rng);
              let noise = (2.0 * g * self.k_b_t / h).sqrt() * z;
              data.qfrc_passive[i] += damping + noise;
          }
      }
  }
  ```
- **Cross-item interaction surfaced — affects item 8**: Whether the
  thermostat lives inside sim-core or in a new sibling L0 crate
  (`sim-thermostat`) is item 8's question, not item 4's. But item 4
  determines that **wherever it lives, that crate gains `rand`,
  `rand_chacha`, and `rand_distr` as production deps**. If sim-core,
  that's a sim-core API surface change. If a sibling crate, the deps
  stay isolated and sim-core remains rand-free in production. Item 8
  should be answered with this trade-off explicitly in mind, not as a
  follow-on default.
- **DECISION (user confirmed)**: **Scheme B — `ChaCha8Rng`**. Concrete
  PRNG type for the thermostat is `rand_chacha::ChaCha8Rng`, seeded
  via `seed_from_u64`, wrapped in `Mutex<ChaCha8Rng>` for the
  `cb_passive` `Fn` constraint. Gaussian draws via
  `rand_distr::StandardNormal`, with the `(2γkT/h)^{1/2}` scale applied
  outside the sample (per-DOF γ, per-step h). The recommendation in
  this entry is now the answer.
- **DECISION (user confirmed) on item 8 lookahead**: **"Keep sim-core
  rand-free" enters item 8 as a heavy constraint, not a tiebreaker.**
  Item 8 is not resolved here, but the leading candidate is a sibling
  L0 crate (`sim-thermostat`) that depends on sim-core and bolts in
  via `cb_passive`. Reasoning: sim-core's identity is "deterministic
  physics anchor"; the readability-first / narrow-crates / R34
  bolt-on-aftermarket philosophy points the same direction; technical
  cost of one more crate is low (the workspace already has 7 sim-L0
  crates); and the dep-graph property "if you don't depend on
  sim-thermostat, your simulation is rand-free" is exactly the kind
  of architecture-from-types property that pays off for beginners
  reading the codebase. Item 8 will weigh this against any
  counter-evidence found during its own recon, but the default
  direction is now set.
- **Item 4 RESOLVED.** PRNG conventions catalogued, two schemes
  compared, Scheme B selected, item-8 constraint set. No code
  written, no Cargo.toml touched yet — those happen during Phase 1
  spec implementation, not recon.
- **Next action**: Phase-1-blocking recon item 5 — `Data::qfrc_passive`
  public mutability. Verify the field is `pub` so `cb_passive`'s
  `&mut Data` can write into it directly. Should be a quick check.

### 2026-04-09 (part 7) — Item 5: `Data::qfrc_passive` public mutability

- **Trigger**: Recon item 5, the smallest of the Phase-1-blocking items.
  Listed in part 3 as "probably fine since other passive fields are
  pub, but verify before drafting the spec." Single-question check.
- **Question**: Is `Data::qfrc_passive` declared `pub` so the
  `cb_passive` callback (which receives `&mut Data`) can write into it
  directly via `data.qfrc_passive[i] += ...`, or is it crate-private
  with no public mutator?
- **Finding**: **Yes — `pub qfrc_passive: DVector<f64>`** at
  `sim/L0/core/src/types/data.rs:158`. Direct field-access mutation is
  supported and is the only intended access path; there is no
  setter/mutator method shadowing it.
- **Bonus context — the entire "Forces in Generalized Coordinates"
  block is uniformly public** (`data.rs:152-172`):
  - `pub qfrc_applied`     (line 154)
  - `pub qfrc_bias`        (line 156)
  - `pub qfrc_passive`     (line 158)
  - `pub qfrc_spring`      (line 161)
  - `pub qfrc_damper`      (line 164)
  - `pub qfrc_fluid`       (line 167)
  - `pub qfrc_gravcomp`    (line 170)
  - `pub qfrc_constraint`  (line 172)

  The codebase convention for `Data` is "fields are public, callers
  read and write directly." This is consistent with the MuJoCo origin
  (mjData fields are all public C struct members) and with the
  `Injector::QfrcApplied` pattern in ml-bridge (item 3 part 5), which
  writes `data.qfrc_applied[i] = ...` directly. The thermostat writing
  `data.qfrc_passive[i] += ...` matches this convention exactly.
- **Sub-observation — the spring/damper/fluid/gravcomp split**: All
  the *component* passive fields (`qfrc_spring`, `qfrc_damper`,
  `qfrc_fluid`, `qfrc_gravcomp`) are also `pub`. They are aggregated
  into `qfrc_passive` at `forward/passive.rs:681` *before* `cb_passive`
  fires (`passive.rs:719`, per part 4). The thermostat writes into
  the *aggregated* `qfrc_passive` field after the engine has finished
  populating it from springs/dampers/fluid/gravcomp — never into a
  component field. Writing into a component field would be a category
  error (component fields have specific physical meanings) and would
  also be wrong on the timing (the aggregation has already happened).
  Part 4's design choice is reaffirmed; no surprise here.
- **No surprises**. Item 5 was expected to be a quick yes-or-no check
  and it was. The only thing worth flagging is that the spec should
  state explicitly *which* field the thermostat writes into
  (`qfrc_passive`, the aggregated post-spring/damper field) and *why*
  (cb_passive fires after aggregation — the aggregated field is the
  only correct write target).
- **Implication for the Phase 1 spec**: One sentence in the API
  section ("Writes via `data.qfrc_passive[i] += ...`; field is
  declared `pub` at `data.rs:158`; no setter required.") and one
  sentence in the design rationale ("Writes the *aggregated*
  `qfrc_passive` field, not the component spring/damper/fluid/gravcomp
  fields, because `cb_passive` fires after aggregation
  (`passive.rs:681,719`).") That's it.
- **Item 5 RESOLVED.** No design changes; a documentation hook for the
  spec is the only output.
- **Next action**: Phase-1-blocking recon item 6 — `model.timestep`
  variability across steps. Question: is `model.timestep` ever changed
  during a simulation, or is it set once at model init and held
  constant? The thermostat reads `h = model.timestep` inside
  `cb_passive` to scale the noise as `(2γkT/h)^{1/2}`. If `h` can
  change between steps, the thermostat is correct. If it can't, the
  `(2γkT/h)^{1/2}` factor could be precomputed once and cached at
  install time, eliminating a sqrt per step per DOF. Performance is
  negligible at Phase 1's nv=1, but the answer also affects the
  `LangevinThermostat` constructor signature (does it take `h`, or
  read it from the model?).

### 2026-04-09 (part 8) — Item 6: `model.timestep` variability across steps

- **Trigger**: Recon item 6 under sharpen-the-axe. Affects whether the
  thermostat reads `h = model.timestep` fresh inside `cb_passive` every
  step or precomputes the scale at install time.
- **Field declaration**: `pub timestep: f64` at
  `sim/L0/core/src/types/model.rs:828`. Plain public field, no setter
  shadowing it. (A second `timestep` field exists at `model.rs:1105`
  on `LengthRangeOpt`, but that's a length-range computation option
  unrelated to the main `Model::timestep`.)
- **Catalog of every write site in the workspace** (categorized):
  1. **Defaults / construction** (set once, never touched again):
     - `sim/L0/types/src/config.rs:32,56,65,76` — Config presets
       (240Hz, 60Hz, 1000Hz, 30Hz).
     - `sim/L0/core/src/types/model.rs:1123` —
       `LengthRangeOpt::default` (different field).
     - `sim/L0/core/src/types/model_init.rs:330` — Default model init
       value (500Hz, 0.002s).
     - `sim/L0/core/src/types/model_factories.rs:132,240,331` — Model
       factory presets (240Hz).
     - `sim/L0/mjcf/src/types.rs:513` — MJCF default.
     - `sim/L0/mjcf/src/builder/init.rs:144` — MJCF builder init.
     - `sim/L0/mjcf/src/builder/mod.rs:772` — Builder copying option
       into model.
     - `sim/L0/mjcf/src/parser.rs:246` — MJCF parser writing into
       option struct.
  2. **Test fixtures setting timestep before any stepping starts**:
     - `sim/L0/core/src/forward/fiber.rs` × 6 (all
       `model.timestep = 0.001`).
     - `sim/L0/core/src/constraint/jacobian.rs` × 2.
     - `sim/L0/core/src/jacobian/mod.rs` × 7.
  3. **Validation tests setting an invalid value to assert rejection**:
     - `sim/L0/types/src/config.rs:485,488,491` — `-0.01`, `0.0`, NaN.
     - `sim/L0/mjcf/src/validation.rs:804,811,818,825` — same pattern.
     - `sim/L0/tests/integration/split_step.rs:126` —
       `model.timestep = 0.0` to test the `step1()` error path
       (already seen in item 3 part 5).
  4. **Internal sub-component sync** (NOT `Model::timestep` itself):
     - `sim/L0/core/src/forward/muscle.rs:134` — `lr_model.timestep =
       opt.timestep;` writes a temporary length-range solver model,
       not the user's main model.
- **Crucial finding**: **There is no site in the codebase where
  `Model::timestep` is mutated *between* steps during a running
  simulation.** Every write is either init/default, test-fixture
  pre-setup, or validation-test invalid-value injection. The field is
  *de facto constant after model init*. The codebase convention is
  "set once, leave alone."
- **No compile-time enforcement, however**. The field is `pub` and
  freely mutable. Any user with `&mut Model` can rewrite it between
  steps. The codebase simply doesn't do this. There is no documented
  policy *because there is no usage pattern that needs one*.
- **Implication for the thermostat**: The cb_passive callback receives
  `&Model` (immutable) and `&mut Data`. It can read `model.timestep`
  for free every step. Two schemes for how to use it:
  - **Scheme A — read fresh every step**: thermostat reads
    `h = model.timestep` inside `cb_passive`, computes
    `(2γkT/h)^{1/2}` per draw. Current API sketch matches this.
    - Pros: Robust to any user manipulation (legal but
      unconventional). Constructor stays simple — no `h` parameter.
      Matches the cb_passive idiom (the callback already gets
      `&Model`; using it costs nothing). One extra `f64` read +
      sqrt per DOF per step.
    - Cons: One sqrt per DOF per step that *could* be precomputed.
  - **Scheme B — cache at install time**: thermostat captures `h` in
    its constructor (or `install()`) and stores the per-DOF scale
    `sigma_i = (2γ_i kT/h)^{1/2}` as a `DVector<f64>`. The hot path
    becomes one multiply per DOF per step, no sqrt.
    - Pros: One sqrt per DOF saved per step.
    - Cons: Becomes silently wrong if the user mutates
      `model.timestep` after install. Constructor needs `h` as input,
      OR `install()` needs to read it from the model — adds an
      `install`-time read that has its own ordering hazard ("what if
      the user installs the thermostat *before* setting timestep?").
- **Performance accounting**: Phase 1 is `nv=1` and runs 10⁵ steps.
  At Scheme A's cost of one sqrt + one read per DOF per step, that's
  10⁵ sqrts total — order of microseconds on any modern CPU,
  negligible compared to the constraint solver + RNG draws. Even at
  Phase 4 (8 coupled bistable elements, maybe `nv ~ 8-16`), the cost
  is 10⁶-ish sqrts over a 10⁵-step run — still negligible. The
  precompute is a true micro-optimization at every plausible phase
  scale.
- **Recommendation: Scheme A — read fresh every step**. Reasoning:
  1. The cost is negligible at every plausible phase scale (Phase
     1–7). Optimizing it is premature; the cost of being wrong is
     not.
  2. Robustness against user mutation is free at this cost. A user
     who legally rewrites `model.timestep` between steps gets correct
     physics from the thermostat, not silently wrong physics.
  3. Constructor stays minimal: `LangevinThermostat::new(gamma,
     k_b_t, seed)` — three parameters, no `h`. Simpler API,
     simpler tests.
  4. No install-order hazard. The thermostat doesn't care when the
     timestep is set, only that it's set when `cb_passive` fires.
  5. Matches the cb_passive idiom: the callback gets `&Model` for a
     reason; using it is the correct pattern. Caching would defeat
     the purpose of the callback receiving the model.
  6. **Loud failure beats silent failure** — same reasoning as item
     2 part 3 and item 4 part 6. Scheme B's failure mode (silent
     wrong physics after a `model.timestep` mutation) is exactly
     the class sharpen-the-axe forbids.
- **Confidence**: high. This is a low-stakes decision (the costs of
  both schemes are well below noise floor), but the principled
  choice still points to A.
- **Implication for the spec**: Document that the thermostat reads
  `h = model.timestep` inside `cb_passive` every step, and that this
  is by design — it's robust to legal-but-unusual user mutation of
  `model.timestep` between steps, and the cost is negligible. State
  that the codebase convention is "set once, leave alone" (no
  in-tree write site mutates timestep mid-simulation), but that the
  thermostat doesn't depend on the convention — it would still be
  correct if the convention were broken.
- **DECISION (user confirmed)**: **Scheme A — read fresh every step.**
  User cited Scheme B's silent-failure potential as the deciding
  factor — same reasoning as item 2 part 3, item 4 part 6, and the
  reasoning in this entry. Loud failure beats silent failure
  consistently across the recon line.
- **Item 6 RESOLVED.** Thermostat reads `h = model.timestep` inside
  `cb_passive` every step. Constructor signature is
  `LangevinThermostat::new(gamma, k_b_t, seed)` — no `h` parameter.
  No code, no spec, no Cargo.toml touched.
- **Next action**: Phase-1-blocking
  recon item 7 — statistical-test infrastructure conventions in
  `sim/L0/tests/`. Question: how does the existing test suite
  structure conformance/numerical tests, and what assertion
  patterns + tolerance idioms are in use? Determines where the
  Phase 1 equipartition test lives, what test crate it joins, and
  what shape its assertions take.

### 2026-04-09 (part 9) — Item 7: test infrastructure conventions

- **Trigger**: Recon item 7 under sharpen-the-axe. Determines test
  crate location, file structure, model-construction idiom, naming
  convention, tolerance idiom, and where the new equipartition test
  lives.
- **Test crate**: `sim-conformance-tests` at `sim/L0/tests/`
  (`sim/L0/tests/Cargo.toml`). One workspace test crate, two
  `[[test]]` targets:
  - `mujoco_conformance` (path `mujoco_conformance/mod.rs`) — golden
    conformance tests against MuJoCo reference outputs.
  - `integration` (path `integration/mod.rs`) — every other sim
    integration test (collision, equality constraints, integrators,
    sensors, actuators, callbacks, etc.).

  Deps: `sim-mjcf`, `sim-urdf`, `sim-core`, `sim-types`, `nalgebra`,
  `approx`. **No `rand` / `rand_chacha` / `rand_distr` in the test
  crate's deps.** Adding statistical tests will need to add these
  (or pull them transitively via the `sim-thermostat` crate from
  item 8).
- **Convention 1 — model construction is by MJCF XML, not direct Rust
  construction**. Every callback test (`tests/integration/callbacks.rs`)
  and almost every passive test (`tests/integration/passive_forces.rs`)
  builds models like:
  ```rust
  let xml = r#"
  <mujoco>
    <option gravity="0 0 0"/>
    <worldbody>...</worldbody>
  </mujoco>"#;
  let mut model = sim_mjcf::load_model(xml).expect("load");
  ```
  This is the dominant idiom; direct `Model::n_link_pendulum(...)`
  construction (used in `split_step.rs`) is the exception. The
  thermostat's 1-DOF damped harmonic oscillator should be expressed
  as MJCF XML to match convention.
- **Convention 2 — naming**. `^fn test_` prefix is dominant: 582
  matches across 23 files in `tests/integration/`. The non-prefixed
  style (`fn passive_callback_adds_force()` in callbacks.rs) is an
  outlier in a small subset of files. **Thermostat tests should use
  `test_` prefix** to match the dominant convention.
- **Convention 3 — tolerance assertions use `approx::assert_relative_eq!`**.
  20+ files import `approx::assert_relative_eq` and use the
  `epsilon = X` form:
  ```rust
  assert_relative_eq!(data.qpos[0], 0.5, epsilon = 0.05);
  ```
  Some files mix in plain `(actual - expected).abs() < epsilon`
  patterns for one-off tolerance checks (e.g., `callbacks.rs:39`).
  Both are accepted; `assert_relative_eq!` is the dominant idiom.
- **Convention 4 — doc comments per test explaining intent**. Every
  test in `callbacks.rs` and `passive_forces.rs` has a `///` doc
  comment explaining what is being verified, often including the
  MuJoCo physics reference being tested. Convention.
- **Adjacent test files for the thermostat — natural neighbors**:
  - `tests/integration/callbacks.rs` (DT-79) — already tests
    `cb_passive`, `cb_control`, `cb_contactfilter`, `cb_sensor`. The
    natural home for *infrastructure* tests on the thermostat
    (callback fires once per step, RNG advances by expected count,
    install/clear lifecycle, etc.).
  - `tests/integration/passive_forces.rs` — tests existing passive
    physics (springs, damping, friction loss). The natural physics
    neighborhood; the equipartition test could conceptually live
    here, though it's substantial enough to deserve its own file.
  - **Recommended new file**: `tests/integration/langevin_thermostat.rs`,
    registered in `tests/integration/mod.rs` as
    `pub mod langevin_thermostat;`. Holds the equipartition test +
    callback-firing-count test + reproducibility-from-seed test +
    parameter sweeps. Big enough as a test surface to deserve its
    own file; small enough to live as one file rather than a
    subdirectory.
- **CORRECTION to part 5 (item 3)**: Part 5 claimed *"The thermostat
  will be the workspace's first `cb_passive` consumer ... nothing in
  `sim/`, `examples/`, or any test currently uses it."* That was
  **partially wrong**. `tests/integration/callbacks.rs` already has:
  - `passive_callback_adds_force` (line 11) — unit test that
    `cb_passive` fires and writes to `qfrc_passive`.
  - `clone_with_callbacks` (line 148) — unit test that cloning a
    Model preserves the passive callback.
  - `none_callbacks_no_effect` (line 117) — sanity check.

  These are **unit tests of the cb_passive API itself**, not
  production consumers. The corrected claim: the thermostat will be
  the workspace's **first stateful `cb_passive` consumer**, and its
  first production consumer. The unit tests in callbacks.rs prove
  the API works; they do not exercise the *stateful* + *Mutex<RNG>*
  patterns the thermostat needs. Worth flagging for accuracy in
  future references back to part 5.
- **Genuine new finding 1 — no existing statistical-sampling tests
  in the workspace**. Verified by greps for `n_samples`,
  `sample_mean`, `0..100000`, `mean +/=`, `chi-square`, etc. The
  `0..10000` loops in `passive_forces.rs:358,672`,
  `model_data_pipeline.rs:280`, `validation.rs:1480`, and
  `collision_primitives.rs:1743` are all *time-evolution* loops
  (run the simulation forward), not Monte Carlo sample loops.
  No test in `sim/L0/tests/` averages a quantity over many samples
  and compares against a theoretical statistical value. **The
  thermostat's equipartition test will be the workspace's first
  statistical-sampling validation test.**
  - Implication: there is **no convention to inherit** for
    statistical assertions. The thermostat must establish one. The
    Phase 1 spec needs to think about whether `approx::assert_relative_eq!`
    is the right tool (probably not — it tests relative error, not
    sampling error) or whether to roll a small helper like
    `assert_within_sampling_error!(actual, expected, n_samples,
    expected_relative_std)` that captures the statistical-tolerance
    intuition. This is a Phase 1 spec design question, not a recon
    question.
  - Implication: the thermostat test sets the *style* for all
    future statistical validation in the thermo line — Phase 2
    (free + articulated body equipartition), Phase 3 (Kramers
    rate), Phase 4 (Ising/Gibbs joint distribution), Phase 5 (EBM
    target match). All of them will need similar statistical
    assertion infrastructure. Worth designing it deliberately at
    Phase 1, not stumbling into it.
- **Genuine new finding 2 — `Model::clone()` preserves passive
  callbacks AND shares captured state across clones**. This is the
  most important finding from item 7 and it surfaces a Phase 1 spec
  API design question. **`callbacks.rs::clone_with_callbacks`
  (lines 148-186) proves it explicitly**:
  ```rust
  let called = Arc::new(AtomicBool::new(false));
  let called_clone = Arc::clone(&called);
  model.set_passive_callback(move |_model, data| {
      called_clone.store(true, Ordering::SeqCst);
      data.qfrc_passive[0] += 1.0;
  });
  let model2 = model.clone();           // ← clone here
  data.forward(&model2).expect("forward");
  assert!(called.load(Ordering::SeqCst));  // ← shared Arc fired
  ```
  The captured `Arc<AtomicBool>` is observed to be set via the
  cloned model's callback firing. Therefore the closure is shared
  (or at least its captured state is) between the original and the
  clone. **Implication for the thermostat**: a user who builds a
  Model, installs a `LangevinThermostat` (which captures
  `Arc<LangevinThermostat>`), then clones the Model N times for N
  parallel envs gets:
  - **Shared `Arc<LangevinThermostat>` across all clones**.
  - **Shared `Mutex<ChaCha8Rng>` across all clones**.
  - **Shared RNG stream** — sample 1 goes to env 0, sample 2 to
    env 5, sample 3 to env 2, etc., based on whatever order the
    parallel envs happen to call the callback. **This is wrong
    physics** — each env's noise stream should be statistically
    independent, not interleaved with all the others.
  - **Mutex contention** on every `cb_passive` invocation across
    every parallel env. Performance disaster at high env counts.
- **Why this is a Phase 1 spec question, not a Phase 1 *implementation*
  problem**: Phase 1 itself is single-env (1-DOF DHO equipartition).
  No clone, no BatchSim, no problem. But the API the thermostat
  exposes in Phase 1 will set the pattern that BatchSim users in
  Phase 2+ will follow. If we ship a `LangevinThermostat::install`
  that silently breaks under clone, BatchSim users will hit it
  cold. Sharpen-the-axe says we surface this *now* and design the
  API with the answer in mind, even if Phase 1 doesn't exercise it.
- **Three possible API responses (sketches only — not deciding
  here)**:
  - **(A)** Document loudly that the user must install a *fresh*
    `LangevinThermostat` on each cloned model, *after* cloning.
    Pure documentation, no code change. Risk: silent footgun,
    exactly the failure mode sharpen-the-axe forbids.
  - **(B)** Provide a paired API:
    `LangevinThermostat::install_per_env(Arc<Model>, n_envs: usize,
    seed_base: u64) -> Vec<Arc<Model>>` that builds N models from
    one prototype, installs N independent thermostats with seeds
    derived from `seed_base`, returns the N models. Forces correct
    usage at the API level. Cost: more API surface, requires Arc
    rewrap.
  - **(C)** Provide a `LangevinThermostat::for_clone(seed: u64) ->
    LangevinThermostat` helper that the user calls explicitly after
    each clone, plus loud documentation. Middle ground.
  - **(D)** Investigate whether `set_passive_callback` could be
    redesigned to take a *factory* (`impl Fn() -> Box<dyn Fn...>`)
    that runs at clone time and produces a fresh closure per clone.
    Most invasive — touches sim-core's callback infrastructure.
    Probably out of scope for Phase 1; flag for the thermostat-and-
    BatchSim spec interaction.

  This list is *not* a decision; it's the option space the spec
  needs to consider. Item 8 (where the thermostat lives) and the
  Phase 1 spec design will weigh these against each other.

  > **Update (chassis design Decision 3, 2026-04-09)**: This
  > clone-footgun option space is now **resolved at the chassis
  > level**, not at the Phase 1 spec level. The chassis design
  > chose **option (B)** — `PassiveStack::install_per_env(prototype,
  > n, build_one)` — as the named API for BatchSim setups, with a
  > defensive `model.clear_passive_callback()` inside the loop
  > that makes the API bulletproof against the install-then-batch
  > order. Option (D) (modify sim-core) is named in the chassis
  > design as the *eventual* right answer if a future user
  > demands truly impossible-to-misuse semantics, but is
  > Phase-1-out-of-scope. See `THERMO_CHASSIS_DESIGN.md`
  > Decision 3 for the full trade-off analysis.
- **Caveat (caveat 3) from part 4 status updated**: Previously
  framed as "BatchSim parallel envs need their own
  `LangevinThermostat` instance — not a shared one — to keep RNG
  streams independent and avoid lock contention. The API should
  make the wrong choice hard." After item 7's finding, it's
  upgraded from "user discipline issue" to "easy-to-trigger
  silent footgun under the natural BatchSim workflow (clone the
  model)." The Phase 1 spec must address it explicitly, not just
  mention it.
- **Item 7 RESOLVED on its actual question** (test conventions
  catalogued, file structure clear, naming and tolerance idioms
  documented, the thermostat tests have a clear home). **Two
  side-findings flagged**: the statistical-test precedent void,
  and the clone-preserves-callback footgun. Neither blocks Phase
  1 implementation; both are inputs to the Phase 1 spec design.
- **Did NOT yet draft**: any code, any spec, any Cargo.toml
  changes.
- **Next action**: Phase-1-blocking recon item 8 — where the
  thermostat module lives (sim-core submodule vs. sibling L0 crate
  vs. inside a new crate). With item 4's "keep sim-core rand-free"
  constraint already set as heavy, the leading candidate is a new
  L0 crate (`sim-thermostat`). Item 8 will weigh this against any
  counter-evidence (e.g., does sim-core's `cb_passive` API expose
  enough surface for an external crate to install a callback?
  needs verification).

### 2026-04-09 (part 10) — Item 8: thermostat module location

- **Trigger**: Recon item 8 under sharpen-the-axe. Item 4 already
  set "keep sim-core rand-free" as a heavy constraint and named the
  leading candidate (a sibling L0 crate `sim-thermostat`). Item 8
  exists to verify the public-API compatibility, name the location
  precisely, and surface any counter-evidence.
- **Question**: Does sim-core's `cb_passive` API expose enough
  *public* surface for an *external* crate to install a callback,
  with no need to access sim-core-private types?
- **Verification — every required type and method is `pub`**:
  - `Model::set_passive_callback<F>(&mut self, f: F)` at
    `sim/L0/core/src/types/model.rs:1237-1243` — public, generic,
    bound `F: Fn(&Self, &mut Data) + Send + Sync + 'static`. The
    setter constructs the internal `Callback(Arc<dyn Fn...>)`
    wrapper itself; **external crates never need to mention
    `CbPassive` or `Callback` types directly**.
  - `Model::clear_passive_callback(&mut self)` at `model.rs:1246`
    — public.
  - `Model` re-exported at `sim/L0/core/src/lib.rs:219` →
    `sim_core::Model`.
  - `Data` re-exported at `lib.rs:192` → `sim_core::Data`.
  - `DVector<f64>` re-exported via
    `pub use nalgebra::{DMatrix, DVector, ...}` at `lib.rs:257` →
    `sim_core::DVector`.
  - `Data::qfrc_passive` is `pub` (item 5 part 7).
  - `Model::timestep` is `pub` (item 6 part 8).

  **An external `sim-thermostat` crate can do everything it needs
  through `sim_core`'s public API with no friction.** No
  sim-core-private types are required, no sim-core code changes
  are required, no `pub(crate)` widening is required.

- **Type-level confirmation of the closure bound**: The thermostat's
  `Arc<LangevinThermostat>` (where `LangevinThermostat` contains
  `gamma: DVector<f64>`, `k_b_t: f64`, `rng: Mutex<ChaCha8Rng>`) is:
  - `Send + Sync`: `DVector<f64>` is Send + Sync; `f64` is
    Send + Sync; `Mutex<ChaCha8Rng>` is Send + Sync because
    `ChaCha8Rng: Send` (its state is plain bytes). ✓
  - `'static`: it owns all its state, no borrows. ✓
  - The closure `move |m, d| me.apply(m, d)` (where `me:
    Arc<LangevinThermostat>`) therefore satisfies
    `Fn(&Model, &mut Data) + Send + Sync + 'static`. ✓
- **Wrapper-type confirmation of the clone footgun**: Looking at
  `set_passive_callback`'s body:
  ```rust
  self.cb_passive = Some(super::callbacks::Callback(std::sync::Arc::new(f)));
  ```
  The closure is wrapped in `Arc<dyn Fn...>` *inside* sim-core. So
  `Model::clone()` clones the outer `Callback` struct, which clones
  the inner `Arc`, which is a pointer-copy — both Models point at
  the same closure object. The closure's `move`-captured
  `Arc<LangevinThermostat>` is therefore physically shared between
  original and clone, which means the `Mutex<ChaCha8Rng>` is shared.
  **The clone footgun from item 7 is now confirmed at the type
  level**, not just observed via the test in `clone_with_callbacks`.
- **Existing sim/L0 sibling layout** (verified by directory listing):
  ```
  sim/L0/types       → sim-types
  sim/L0/simd        → sim-simd
  sim/L0/core        → sim-core
  sim/L0/gpu         → sim-gpu
  sim/L0/mjcf        → sim-mjcf
  sim/L0/urdf        → sim-urdf
  sim/L0/tests       → sim-conformance-tests
  sim/L0/ml-bridge   → sim-ml-bridge
  ```
  **8 existing sim/L0 sibling crates**. Naming convention is
  single-word lowercase (`types`, `simd`, `core`, `gpu`, `mjcf`,
  `urdf`) with hyphen for compound names (`ml-bridge`,
  `conformance-tests`). Package names are `sim-{dirname}`.
  **A new `sim/L0/thermostat/` → `sim-thermostat` fits cleanly.**
- **Two schemes compared** (per "Two schemes, then choose"):
  - **Scheme α — sim-core submodule**: Add a `thermostat`
    submodule at `sim/L0/core/src/thermostat.rs` (or alongside
    `forward/passive.rs`). Promote `rand`, `rand_chacha`,
    `rand_distr` from dev-deps to production deps in
    `sim/L0/core/Cargo.toml`. Re-export `LangevinThermostat` from
    `sim/L0/core/src/lib.rs`.
    - **Pros**: One fewer crate. Colocation with the integration
      point (`cb_passive`, `qfrc_passive`, `forward/passive.rs`).
      Discoverable: a reader of sim-core sees the thermostat
      naturally.
    - **Cons**: Promotes `rand` + `rand_chacha` + `rand_distr` to
      production deps of sim-core. **sim-core's identity changes
      from "deterministic physics anchor" to "physics anchor with
      stochastic infrastructure."** Every consumer of sim-core
      (sim-mjcf, sim-urdf, sim-ml-bridge, sim-conformance-tests,
      sim-bevy, every example, every downstream user) inherits the
      `rand` dep tree transitively, *whether or not they use the
      thermostat*. Violates item 4's heavy constraint.
  - **Scheme β — sibling L0 crate `sim-thermostat`**: New crate at
    `sim/L0/thermostat/` with `Cargo.toml` declaring `sim-core`,
    `nalgebra`, `rand`, `rand_chacha`, `rand_distr` as deps.
    `LangevinThermostat::install` takes `&mut sim_core::Model` and
    calls `model.set_passive_callback(...)`. sim-core stays
    rand-free in production.
    - **Pros**: sim-core's identity is preserved
      (deterministic-by-default, rand-free). Dep tree property
      "if you don't depend on sim-thermostat, your sim is rand-free"
      is observable from the dep graph alone — the kind of
      readability-from-types property that beginners benefit from.
      Aligns with the workspace's existing 8-crate sibling layout
      and the user's stated "narrow crates over monoliths"
      preference. Future companion features (Phase 2 free-body
      thermostat, Phase 3 bistable element library, Phase 4
      coupled-array thermostat) get a natural home alongside
      `LangevinThermostat`. Item 4's constraint is honored.
    - **Cons**: One more crate to maintain (Cargo.toml, lib.rs,
      lints, dev-deps). Discoverability slightly worse — a reader
      of sim-core won't see the thermostat without browsing
      siblings. (Mitigated by a one-line "see also" comment in
      sim-core's `forward/passive.rs:719` near the `cb_passive`
      invocation site, and by the spec / master plan.)
- **Recommendation: Scheme β — `sim-thermostat` sibling crate**.
  Confidence: high. Reasoning condensed:
  1. Item 4's heavy constraint is unambiguous: keep sim-core
     rand-free. Putting the thermostat in sim-core directly
     violates it.
  2. The public-API surface verification proves there is **zero
     technical friction** to the sibling-crate approach. No
     `pub(crate)` widening, no callback infrastructure changes,
     no sim-core code edits required.
  3. The R34 / narrow-crates / readability-from-types philosophy
     points the same direction (item 4 part 6 already documented
     this).
  4. The "one fewer crate" pro of Scheme α is real but small —
     the workspace already has 8 sibling sim-L0 crates, so the
     marginal cost of a 9th is well-amortized.
  5. The discoverability con of Scheme β is mitigable with one
     "see also" comment in sim-core and proper documentation in
     the master plan + Phase 1 spec.
- **Concrete location**: New crate at `sim/L0/thermostat/` with
  package name `sim-thermostat`. Files at Phase 1 implementation
  time:
  ```
  sim/L0/thermostat/
  ├── Cargo.toml         # deps: sim-core, nalgebra, rand,
  │                      #       rand_chacha, rand_distr
  └── src/
      └── lib.rs         # pub struct LangevinThermostat
                         # impl LangevinThermostat { new, install,
                         #                           clear, ... }
  ```
  **One file is enough at Phase 1 scale**. Phase 2+ may add
  submodules (`free_body.rs`, `articulated.rs`) as needed; the
  crate-as-collection-point gives them a home.
- **Test location — flagged for the Phase 1 spec, not decided here**:
  Three options for where the thermostat tests live:
  - **(α)** Inside `sim-thermostat/tests/` as integration tests of
    the sibling crate. Pro: tests with code, deps isolated. Con:
    diverges from the "all sim integration tests in
    sim-conformance-tests" convention.
  - **(β)** In `sim-conformance-tests` as
    `tests/integration/langevin_thermostat.rs` (per item 7
    recommendation). Pro: matches convention. Con:
    `sim-conformance-tests` gains `sim-thermostat` as a dep (and
    transitively `rand`/`rand_chacha`/`rand_distr`).
  - **(γ)** Hybrid: API/lifecycle/seeding tests in
    `sim-thermostat/tests/`, equipartition + cross-physics
    integration tests in `sim-conformance-tests`. Pro: each test
    lives where it makes most sense. Con: two homes, slightly
    worse discoverability.

  This is a Phase 1 spec design question, not a recon question.
  Item 8 is resolved on the *code* location; the *test* location
  goes in the spec.
- **DECISION (user confirmed)**: **Scheme β — `sim-thermostat`
  sibling crate at `sim/L0/thermostat/`**. Item 4's "keep sim-core
  rand-free" constraint is honored; the public-API surface is
  sufficient (no sim-core changes required); the sibling layout
  matches the existing 8 sim/L0 crates.
- **Item 8 RESOLVED.** Thermostat code will live in
  `sim/L0/thermostat/` as a new sibling L0 crate (`sim-thermostat`),
  depending on `sim-core`, `nalgebra`, `rand`, `rand_chacha`,
  `rand_distr`. sim-core stays rand-free in production.
- **Did NOT yet draft**: any code, any spec, any Cargo.toml
  changes, any new directories.
- **Phase-1-blocking recon round complete.** Item
  list status:
  - Item 2 — RESOLVED (part 3, then superseded by part 4).
  - Item 3 — RESOLVED (part 4 + part 5).
  - Item 4 — RESOLVED (part 6).
  - Item 5 — RESOLVED (part 7).
  - Item 6 — RESOLVED (part 8).
  - Item 7 — RESOLVED (part 9).
  - Item 8 — RESOLVED (this entry).

  Once item 8 is confirmed, the next action is **drafting the
  Phase 1 spec** (`docs/thermo_computing/PHASE_1_LANGEVIN_THERMOSTAT_SPEC.md`).
  All inputs are now collected. The spec needs to address:
  1. The base algorithm (Euler-Maruyama Langevin) and the
     equipartition validation gate.
  2. Composition section (field disjointness with RL writers,
     per part 5).
  3. Test section (callback-firing-count + equipartition + seed
     reproducibility, per part 7).
  4. **Statistical-test infrastructure design** — how the
     equipartition test asserts sample-mean-within-tolerance, and
     what shape the helper macro/fn takes (per part 9 finding 1).
  5. **Clone footgun resolution** — which of options A-D from
     part 9 finding 2 the spec adopts, and how the API enforces
     it (per part 9 finding 2).
  6. Cadence section (plain `step()`, no split-step required,
     per part 5).
  7. PRNG choice (`ChaCha8Rng + StandardNormal`, per part 6) and
     the constructor signature (`new(gamma, k_b_t, seed)`, per
     part 8).
  8. Module location (`sim/L0/thermostat/`, per this entry).
  9. Phase 5+ caveats from part 4 (FD perturbation gating,
     plugin ordering, BatchSim per-env instances).
  10. The validation parameter set (M=1, k_spring=1, γ=0.1,
      k_B·T=1, h=0.001, 10⁵ steps after burn-in) and the
      γ + T sweep grid.

  > **Update (chassis design round, 2026-04-09)**: Items 1-3
  > and 6-10 above are now answered by `THERMO_CHASSIS_DESIGN.md`
  > (Decisions 1-6) and the existing recon log entries. Items 4
  > and 5 — the two "new design surfaces" — are also resolved at
  > the chassis level: item 4 by Decision 5 (`WelfordOnline` +
  > `assert_within_n_sigma`), item 5 by Decision 3
  > (`PassiveStack::install_per_env` with defensive clear). The
  > Phase 1 spec inherits answers to all 10 items from the
  > chassis design round; the only design decision left for the
  > spec is choosing the validation parameter set fix
  > (α/β/γ — see the corrected Phase 1 validation paragraph in
  > §The Gap above and Decision 5 in the chassis doc).

### 2026-04-09 (part 11) — Chassis design round complete

- **Trigger**: After the Phase-1-blocking recon round (items 2-8)
  closed in part 10, the user reframed the next step away from a
  monolithic Phase 1 spec and toward a *bolt-pattern design*
  document defining the swappable chassis of the `sim-thermostat`
  crate. The framing was R34-explicit: "we might decide to redo
  the entire thing, as long as the bolt patterns are the same,
  we can swap it." The reference architecture was named
  explicitly as `sim-ml-bridge`. A new artifact was opened:
  `docs/thermo_computing/THERMO_CHASSIS_DESIGN.md`.
- **Cadence**: Same sharpen-the-axe rhythm as the recon round
  (one decision at a time, two schemes per decision,
  recommendation + reasoning, user confirmation, log + commit),
  applied at the *design layer* instead of the code-reading
  layer.
- **Six chassis decisions resolved**:

  | # | Decision                  | Resolution                                                            |
  |---|---------------------------|-----------------------------------------------------------------------|
  | 1 | Core trait shape          | `PassiveComponent` — broad, callback-shaped, no physics in the trait  |
  | 2 | Composition idiom         | `PassiveStack::builder().with(...).build().install(&mut model)`       |
  | 3 | Clone footgun resolution  | `install_per_env(prototype, n, build_one)` + defensive clear inside loop |
  | 4 | `Diagnose` trait surface  | Minimal: `diagnostic_summary -> String` only                          |
  | 5 | Public test utilities     | `WelfordOnline` (Welford 1962) + `assert_within_n_sigma` + `sample_stats` |
  | 6 | Crate layout              | Flat ml-bridge style: 5 source files + 1 test file at `sim/L0/thermostat/` |

  Full reasoning for each decision lives in
  [`THERMO_CHASSIS_DESIGN.md`](./THERMO_CHASSIS_DESIGN.md). The
  recon log entries here capture the *meta* — that the round
  happened, what it decided, what it found.
- **Side findings surfaced during chassis design**:
  - **Decision 5 — autocorrelation correction**: The recon log
    part 2 claim of "10⁻² sampling tolerance for 10⁵ samples"
    was off by ~10× because samples from the Markov chain are
    correlated. Effective N ≈ 10, not 10⁵. The `§The Gap Phase 1
    Validation` paragraph above now carries the corrected
    calculation and a note that the Phase 1 spec will choose
    between three fixes (α/β/γ).
  - **Decision 3 — clone footgun resolved at chassis level**:
    Item 7 / part 9 flagged the `Model::clone()` callback-
    sharing footgun as needing Phase 1 spec resolution. Now
    resolved at the chassis level (not the spec level) via
    `PassiveStack::install_per_env`. The part 9 entry above
    now carries an "update" note pointing to Decision 3.
  - **Total chassis surface estimate**: ~790 LOC across 8
    files (6 source + 1 integration test + 1 Cargo.toml).
    Comparable to a small ml-bridge algorithm + tests.
    Manageable, mechanical, swappable.
- **Why the chassis design round was a separate phase from
  spec drafting**: The user named the principle directly —
  "the implementation may be wrong, that's OK, AS LONG AS the
  bolt patterns are right." Designing the bolt patterns
  *first*, then implementing one component against them,
  produces a chassis that survives implementation surprises.
  The Phase 1 implementation is now allowed to inform a chassis
  revision if needed; the chassis is small enough (~320 LOC of
  trait + composer + diagnose + test_utils, per Decision 6's
  file inventory: 30 + 120 + 20 + 150) that revising it is a
  focused operation, not a rewrite.
- **Memory entry created during this round**:
  [Genuine agreement, not passive](feedback_genuine_agreement.md)
  — feedback memory established when the user noted that their
  string of confirmed recommendations was genuine agreement,
  not passive acceptance. Keep being decisive; don't water down
  recommendations to extract artificial pushback.
- **Did NOT yet draft**: any code, any Cargo.toml, any new
  directories. Six decisions are designed, none are
  implemented. The chassis is a paper artifact at the close of
  this entry.
- **Next action — IN A NEW SESSION**: draft the Phase 1 spec
  (`PHASE_1_LANGEVIN_THERMOSTAT_SPEC.md`) against the now-
  finalized chassis. The spec is small because the chassis
  answered everything except the validation parameter fix
  (α/β/γ). Estimated size ~300-500 lines, half-day of focused
  drafting. The new session should get a tailored prompt that
  references:
  - This master plan (with the chassis design round complete)
  - `THERMO_CHASSIS_DESIGN.md` for the bolt patterns
  - Decision 5's autocorrelation finding and the three fix
    options for the validation parameter set
  - The remaining sharpen-the-axe discipline for spec design

### 2026-04-09 (part 12) — Doc review pass: M1, M4, M5, M2, M3, S1

- **Trigger**: Reviewer pass (Claude Opus 4.6, 1M context) over
  `MASTER_PLAN.md` + `THERMO_CHASSIS_DESIGN.md`. Produced a
  15-item checklist (5 must-fix, 6 should-fix, 4 nits) recorded
  at `docs/thermo_computing/DOC_REVIEW_2026-04-09.md`. Items
  applied one at a time as separate commits on the branch
  `feature/thermo-doc-review`.
- **Cadence**: Same sharpen-the-axe rhythm — one item per
  commit, two-schemes-then-choose for design changes (M5 and
  M2), explicit acceptance criteria, user confirmation for the
  load-bearing chassis-altering items.
- **Items closed in this entry**:
  - **M1 — Phase 1 sampling-error tolerance wording fix**.
    The original `0.22 · (½kT) ≈ ±22%` phrasing was off by
    a factor of 2. Corrected to `(½kT)·√2/√10 ≈ 0.45·(½kT)`,
    i.e. ±45% of the expected mean for one trajectory; option
    (β) with 100 trajectories brings this to ±4.5% of `½kT`.
    Locked in a tolerance convention: tolerances are always
    expressed as a fraction of the expected mean, not as a
    fraction of `kT`. Commit `75d90de`.
  - **M4 — `Welford::reset()` and `merge()` added to chassis
    Decision 5**. Without `reset()`, the streaming-Welford
    burn-in story is broken. Without `merge()`, option (β)
    has to materialize per-trajectory means and re-Welford
    them. Both shipped in Phase 1; `merge` uses the
    Chan/Pébay parallel formula. `WelfordOnline` gains
    `#[derive(Clone)]`. Commit `031f777`.
  - **M5 — Type-enforce `PassiveComponent` write target**.
    Trait revised from `apply(&self, &Model, &mut Data)` to
    `apply(&self, &Model, &Data, &mut DVector<f64>)`. Data
    is now read-only; `qfrc_out` (length nv) is the only legal
    write target. `PassiveStack::install` allocates a scratch
    buffer per step, runs each component into it, and folds
    the result into `data.qfrc_passive` once at the end. The
    breaking change is free *right now* because no code has
    been written; after Phase 1 ships, the same change would
    be ripple-through painful. Same "loud over silent" line
    as items 2, 4, 6. Commit `9bc030b`.
  - **M2 — Decision 7: stochastic gating for FD/autograd**.
    The Phase 5+ caveat about `cb_passive` firing inside
    `forward_skip()` was previously punted. Doc review M2
    escalated it to a chassis-level decision because
    retrofitting trait shape at Phase 5 is breaking-change
    territory. Two-scheme analysis: Scheme A (orthogonal
    `Stochastic` opt-in trait + RAII `disable_stochastic()`
    guard) vs Scheme B (RNG snapshot/restore on
    `PassiveStack`). Scheme A confirmed. Every stochastic
    component on the roadmap has state-independent noise, so
    "disable noise during FD" gives the exactly-correct
    derivative `∂F_det/∂qpos`. Scheme B is documented as the
    additive future direction (`RngSnapshot` orthogonal trait)
    if a state-dependent noise component ever lands. Commit
    `034d9c8`.
  - **M3 — Q5 (cf-design end-to-end differentiability)
    escalated**. Q5 was previously a single-sentence
    deferred-to-Phase 5 entry. Doc review M3 escalated it to
    active foreground recon, scheduled in parallel with the
    Phase 1 spec drafting, on the asymmetric-risk argument.
    Discovering "no" after months of Phase 2-4 commitment is
    exactly the avoidable surprise sharpen-the-axe forbids.
    Recon scope named (cf-design autograd integration, SDF
    boolean differentiability, marching cubes vs smooth
    alternative, the cf-design Phase 5 differentiable-
    optimization spec). Recon log entry name reserved for when
    the recon actually starts. Commit `6ccc0ff`.
  - **S1 — Q3 (`thrml-rs` existence) resolved**. 5-minute web
    search. Answer is *"yes, but with caveats"*: original
    `extropic-ai/thrml` (JAX) exists, two community Rust
    ports exist on GitHub (`SashimiSaketoro/thrml-rs`,
    `Pingasmaster/thrml-rs`), neither officially maintained
    by Extropic, neither apparently on crates.io. Phase 6
    options refined from "exists / doesn't" binary to (A)
    depend on a port, (B) implement minimal native block-Gibbs
    (~few hundred LOC, leading direction, sharpen-the-axe
    consistent), (C) vendor / fork into a `sim-thrml` sibling
    crate. No commitment yet — Phase 6 spec will decide. The
    "5-minute web search" was the right move because deferring
    it preserved an artificial binary that hid the real
    three-option structure. This commit.
- **Aggregate effect on the chassis**:
  - Trait shape (Decision 1) revised twice: M5 added the
    scratch buffer signature, M2 added the `as_stochastic`
    introspection hook. Both additive and consistent.
  - Stack API (Decision 2) revised twice: M5 added the
    scratch-buffer install body, M2 added
    `set_all_stochastic` and `disable_stochastic` RAII guard.
  - Test utilities (Decision 5) gained `Welford::reset()` and
    `merge()` (M4).
  - File inventory: `component.rs` 30→70 LOC (Stochastic
    trait), `stack.rs` 120→160 LOC (gating + guard),
    `test_utils.rs` 150→170 LOC (reset + merge), total Phase
    1 footprint 790→890 LOC.
  - Decision count: 6→7. The chassis design round complete
    table now has 7 rows.
- **Open follow-ons after this entry** (the should-fix /
  nit items still on the doc review checklist):
  - S2 — Rewrite §3 Current State to reflect chassis-design-
    round outcome (or split master plan into two files).
  - S3 — Reconsider `install_per_env` return type
    (`Vec<Model>` vs `EnvBatch`).
  - S4 — Add "passive forces only" framing paragraph to
    chassis §0.
  - S5 — Call out D4's external dependencies (3D printer
    accuracy, measurement infrastructure).
  - S6 — Note thermostat persistence as a future direction.
  - N1-N4 — polish pass.
  - **Then**: draft the Phase 1 spec.
- **Did NOT yet draft**: any code, any Cargo.toml, any new
  directories. Doc review pass is paper-only by design.

### 2026-04-09 (part 13) — Q5: cf-design end-to-end differentiability — RESOLVED (NO, with caveats)

- **Trigger**: Doc review M3 (part 12) escalated Q5 from "deferred
  to Phase 5" to active foreground recon, on the asymmetric-risk
  argument: discovering "no" after months of Phase 2-4 commitment
  is exactly the avoidable surprise sharpen-the-axe forbids. This
  is the recon that closes the question. Recon scope was named
  in part 12: cf-design autograd integration, SDF boolean
  differentiability, marching cubes vs smooth alternative, the
  cf-design Phase 5 spec, and existing end-to-end gradient tests.
- **Verdict**: **NO**. The cf-design → sim-core parameter
  pipeline is *not* end-to-end differentiable. It is differentiable
  *up to* the SDF field level (analytically) and *no further*.
  Three concrete breaks downstream. The shipping Phase 5 design-
  through-physics optimization uses **finite differences over the
  full pipeline**, and the documentation says so explicitly.
- **What works** — analytic ∂f/∂θ at the SDF level:
  - `cf-design/src/param.rs` defines `ParamStore` (Arc-shared
    `RwLock<Vec<f64>>`) and `ParamRef` for naming scalar design
    variables. ~216 LOC, zero ML deps. Plain mutable f64 store.
  - `cf-design/src/param_gradient.rs` (~273 LOC) implements
    hand-rolled forward-mode AD over the `FieldNode` enum.
    Smooth booleans (`SmoothUnion`, `SmoothSubtract`,
    `SmoothIntersect`, `SmoothUnionAll`) carry full analytic
    chain-rule derivatives in θ, including the softmax-weighted
    `∂f/∂k` for the variable blend radius. Hard booleans
    (`Union`, `Subtract`, `Intersect`) follow the active child
    (correct subgradient, undefined exactly at the seam).
    Transforms (translate/rotate/scale/mirror/twist/bend/
    repeat) propagate via inverse pull-back. Tested against
    centered FD oracle to ε=1e-6 across 8+ unit tests
    (`param_gradient.rs:294-end`).
  - `cf-design/src/gradient.rs` (separate file) implements
    `∇f(p)` — the *spatial* gradient w.r.t. (x,y,z), used by
    dual contouring for vertex placement. Distinct from
    `param_gradient`; both are analytic, both forward-mode.
- **First break** — mesh extraction is topologically discrete:
  - `cf-design/src/mesher.rs:1` — "Marching cubes mesher with
    interval arithmetic pruning." Standard MC over a regular
    voxel grid; vertex/edge/face count changes discretely as θ
    crosses critical values. No smooth surrogate.
  - `cf-design/src/dual_contouring.rs:1-7` — manifold dual
    contouring with QEF vertex placement on the same regular
    voxel grid. Sharper features than MC, but still discrete
    sign-change topology. `adaptive_dc.rs` (1286 LOC) is the
    octree-adaptive variant — same fundamental break.
  - There is no differentiable marching cubes (DiffMC), no
    neural implicit surface extractor, no soft-occupancy mesher
    in the codebase. The cf-geometry::IndexedMesh that comes
    out the other side has no notion of θ-derivatives.
- **Second break** — mass properties are unwired:
  - `cf-design/src/mechanism/mass.rs:1-22` — `mass_properties`
    integrates `density · interior_indicator` over a regular
    grid with linear-fraction boundary weighting. Returns plain
    `f64` mass + `Point3<f64>` COM + `[f64; 6]` inertia tensor.
    No `param_gradient` propagation. The chain rule *could* be
    applied here in principle (the boundary integral has a
    well-defined θ-derivative via the divergence theorem on the
    smooth field), but it currently isn't.
  - This is the **fixable** break. Doable in a few hundred LOC,
    independent of the mesh-extraction question, would unlock
    analytic ∂(mass, COM, inertia)/∂θ. The Phase 5 spec already
    flagged it: `optim.rs:9-19` calls out "future hybrid
    approaches (analytic mass/inertia gradients + FD for the
    sim boundary)."
- **Third break** — the sim-core forward step is opaque to AD:
  - sim-core's `mj_step` is a hand-coded forward integrator with
    contact, constraints, and implicit damping. No autograd
    handshake. ml-bridge's autograd never touches `Model`/`Data`
    fields — it operates on f64 scalars in policy networks
    (`sim/L0/ml-bridge/src/autograd.rs`).
  - This is the same problem MuJoCo has been chasing for ~5
    years (MuJoCo MJX is the JAX port that solves it by
    rewriting the entire forward step in XLA). No shortcut.
- **The two AD systems are completely disconnected**:
  - `cf-design/Cargo.toml` does NOT depend on `sim-ml-bridge`.
  - `sim-ml-bridge/Cargo.toml` does NOT depend on `cf-design`.
  - Verified by direct read and by `grep`-ing for `cf_design`,
    `ParamStore`, `FieldNode` across `sim/L0/ml-bridge/src/` —
    zero matches.
  - cf-design's `param_gradient` is **forward-mode** analytic AD
    over a hand-rolled `FieldNode` enum (~273 LOC, scalar f64).
  - sim-ml-bridge's `autograd::Tape` is **reverse-mode** tape AD
    over scalar f64 ops (`add/sub/mul/neg/tanh/relu/square/ln/
    exp` + `affine/sum/mean`), explicitly RL-policy-oriented
    ("CPU f64 only," "no GPU tensors," "no batched tape").
  - There is no shared `Variable`/`Tensor` type, no shared scalar
    abstraction, and the two engines use opposite gradient
    styles. Fusing them is not a one-day port.
- **The shipping "differentiable design optimization"**:
  - `cf-design/src/optim.rs` is `minimize_fd` — centered finite-
    difference gradient descent over a user-provided objective
    closure. Lines 9-19 of the docstring are the smoking gun:
    > "For design-through-physics optimization, the end-to-end
    > gradient is `∂J/∂θ where θ → SDF(θ) → mesh → Model →
    > simulate → x_T → J(x_T)`. The middle of this chain
    > (`mesh → Model`) is not analytically differentiable, so
    > we use finite differences over the full pipeline.
    > Session 25's analytic `∂f/∂θ` enables future hybrid
    > approaches."
  - The Phase 5 integration test
    (`mechanism/integration.rs:328-457`,
    `phase5_parameterized_grasp_optimization`) drives a
    parameterized sphere onto a ground plane, measures contact
    force, and uses `minimize_fd` to find the radius that
    maximizes it. **Three** FD iterations, learning rate `1e-9`,
    `fd_eps=0.5`. End-to-end FD, full pipeline rebuild every
    eval (mechanism rebuild → MJCF emit → `sim_mjcf::load_model`
    → 500 sim steps → contact force sum → return scalar). No
    analytic gradients exercised anywhere in the test.
  - **There is no test, anywhere in the workspace, that
    exercises an end-to-end analytical θ-gradient from cf-design
    through sim-core**. `param_gradient` is exercised only
    against the field value itself, not against any downstream
    physics quantity.
- **The "Phase 5 = differentiable design optimization" framing
  is overstated**: Both the project-memory entry ("cf-design
  Phases 1–5 complete (including differentiable design
  optimization)") and §3 of this master plan ("cf-design
  (Phases 1–5 complete) with implicit surface composition,
  mechanism library, and **differentiable design optimization**")
  describe Phase 5 as if the gradient pipeline closes. It does
  not. The honest description is: **finite-difference design
  optimization through simulation, with analytic ∂f/∂θ at the
  SDF field level as a future hybrid lever.** This is a memory/
  doc nit, not a chassis nit; flagging here for future reference.
- **What this means for the build order** (the reason Q5 was
  escalated):
  - **Phases 1-4 are unaffected.** The Langevin thermostat
    chassis lives entirely on the sim-core side. None of the
    seven chassis decisions touches cf-design or its parameter
    pipeline. Phase 1 spec drafting can proceed unchanged.
  - **D3 (co-design) drops several rungs.** D3 was priority 3
    on the Research Directions ladder, ahead of D2 (stochastic
    resonance) and D4 (sim-to-real on a printed device). With
    the Q5 NO, D3 requires *either* a multi-month differentiable-
    forward-step research thread, *or* a surrogate-model
    workaround that crosses a fidelity boundary. Neither is
    sharpen-the-axe-cheap. **Recommend reshuffling**: D1 → D2
    → D4 → D3 → D5. D3 stays on the roadmap as the long-horizon
    "after we can differentiate through `mj_step`" item, not as
    the headline experiment.
  - **D1, D2, D4 are unblocked.** None of them require end-to-
    end gradients through cf-design. D1 (Brownian motor) and
    D2 (stochastic resonance) are pure sim experiments with
    fixed geometry, no design loop. D4 (sim-to-real on printed
    device) needs the design → print path, but the *training*
    happens via RL (sim-ml-bridge already shipped) on a fixed
    parameterization — no cf-design gradients required for the
    EBM training itself, only for *post-hoc* design refinement
    which is exactly what `minimize_fd` already does.
  - **The fixable second break (mass properties) is the cheapest
    capability gain.** Wiring `param_gradient` through
    `mass_properties` would give analytic ∂(mass, COM, inertia)/
    ∂θ for free, which is the part of the Model that benefits
    most from gradients (mass and inertia drive the dynamics
    linearly via `M` and `M⁻¹`). This is *not* on the thermo
    line's critical path, but it's worth flagging upstream to
    the cf-design team as a small, well-scoped follow-on.
- **Plausible reactions revisited** (from the Q5 entry in §5):
  1. **Build the differentiable layer earlier** — large surface
     area: differentiable mesh extraction (DiffMC or smooth
     surrogate) AND a differentiable forward step. Multi-month.
     Not recommended pre-Phase-1.
  2. **Re-prioritize D3 down the ladder** — RECOMMENDED. Cheap,
     consistent with sharpen-the-axe ("foundations over
     scaffolding"), preserves option to revisit later.
  3. **Surrogate model** — viable for D3 specifically, crosses a
     fidelity boundary. Defer the decision until D2/D4 results
     show whether D3's headline framing is still load-bearing.
  4. **Sibling crate wraps cf-design with custom autograd** —
     same surface area as option 1, just packaged differently.
     Not a free win.
- **Recon scope coverage** (from part 12 M3):
  - [x] cf-design autograd integration point — does not exist
    (no sim-ml-bridge dep); custom forward AD lives in
    `param_gradient.rs`.
  - [x] cf-design Phase 5 spec — `optim.rs` docstring is the
    authoritative description; no separate spec file
    (`docs/SDF_NATIVE_PHYSICS_SPEC.md` is the only doc and is
    about a different topic).
  - [x] SDF boolean differentiability — smooth booleans fully
    analytic; hard booleans subgradient-correct away from the
    seam.
  - [x] Mesh extraction — marching cubes + dual contouring,
    both topologically discrete, no smooth alternative.
  - [x] End-to-end gradient tests — none. `param_gradient` is
    tested against FD oracle on field values; the Phase 5
    integration test uses `minimize_fd` (full-pipeline FD)
    with no analytic gradient exercised.
- **Time spent**: ~30 minutes of focused reading. The answer
  was visible in the first file (`optim.rs:9-19`) but warranted
  the corroborating recon across `param_gradient.rs`,
  `mesher.rs`, `dual_contouring.rs`, `mass.rs`,
  `integration.rs`, both Cargo.toml files, and
  `ml-bridge/src/autograd.rs`. Asymmetric-risk argument from
  part 12 was correct: a half-day's recon would have been
  worth it even on a "yes," and the actual answer is "no" with
  build-order consequences.
- **What did NOT happen this session**: no code, no chassis
  edits, no Phase 1 spec drafting, no priority-ladder edit in
  §2.4 (the recommendation above is a recommendation, not a
  commit — pending user confirmation). The recon log entry is
  the only artifact.
- **Open follow-ons**:
  - Update §3 "Q5 status" line from "ESCALATED" to "RESOLVED
    (NO)" with a one-sentence summary pointer to part 13.
  - Update §5 Q5 entry to reflect resolution and link to this
    part.
  - Decide whether to reshuffle the §2.4 priority ladder
    (D1 → D2 → D4 → D3 → D5 recommended).
  - Decide whether to soften the "differentiable design
    optimization" phrasing in §3 and in the cf-design memory
    entry (`project_thermo_computing.md`, `MEMORY.md`) to
    "finite-difference design optimization with analytic
    ∂f/∂θ at the SDF level."
  - Optional cross-team flag: wiring `param_gradient` through
    `mass_properties` is a small, well-scoped cf-design follow-
    on that would unlock analytic mass/inertia gradients.
    Independent of the thermo line's critical path; surface to
    cf-design when convenient.
