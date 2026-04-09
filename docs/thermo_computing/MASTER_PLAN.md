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

## 1. Vision (the endpoint)

Build a unified **design → simulate → fabricate** pipeline for *mechanical
thermodynamic computing devices* — physical systems whose probabilistic
behavior is the computation. The same parameters drive the geometry (cf-design),
the physics (sim-core + a Langevin thermostat), the energy-based model
(ml-bridge autograd), and the controller (ml-bridge RL).

The probabilistic computing element is **not** a transistor. It is a bistable
mechanical structure — a buckling beam, a snap-through latch, a Brownian
ratchet — whose state is set by thermal noise plus local field. These elements
are real, they're studied seriously in stochastic thermodynamics (Landauer,
Bennett, Sekimoto), and **they are 3D-printable**. Extropic builds p-bits out
of transistors because they have a fab. CortenForge has cf-design and a 3D
printer. That's a different — and arguably more interesting — angle.

### Why CortenForge is the right substrate

- **sim-core** already does articulated rigid-body dynamics with constraint
  solving at the device scale. This is where the bistable elements live.
- **External force injection** (`qfrc_applied`, `xfrc_applied`) and
  **damping infrastructure** (`dof_damping`, `jnt_damping`, etc.) already
  exist and are MuJoCo-conformant. The fluctuation-dissipation machinery is
  half-built without anyone planning for it.
- **cf-design** parameterizes the geometry of bistable elements with
  implicit surfaces and mechanism primitives. The same parameters can be made
  differentiable.
- **ml-bridge autograd** (Phases 1–6c) gives us a Rust-native gradient engine
  for training energy functions.
- **ml-bridge RL** (CEM/TD3/REINFORCE + policy persistence) can consume the
  resulting stochastic environment.
- **3D printing** closes the loop in a way no software-only stack can.

### Headline research claim (aspirational)

> A unified Rust-native pipeline in which the same set of parameters
> simultaneously specifies the *geometry*, the *physics simulation*, the
> *energy-based model*, and the *control policy* of a mechanical thermodynamic
> computing device, and where the simulated device matches its 3D-printed
> physical counterpart within validated tolerances.

Nobody has this. LAMMPS can't design. cf-design can't sample. PyTorch EBMs
aren't physical. Extropic can't fab cheaply. The wedge is real.

---

## 2. Research Directions

The Vision above is abstract. **These are the concrete experiments that
realize it.** They are not the build order — Phase 1 still has to ship
first — but they are the *why*. The Master Plan exists to keep these alive
through implementation.

Five experiments + one foundational connection, ordered from earliest
payoff to longest horizon. Each direction states: concept, physics
grounding, the experiment, where it fits in the build, and why it matters.

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
fab dependency.

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

*Recon snapshot — last updated 2026-04-09. Re-verify before relying on these
claims; the codebase moves.*

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

### What does *not* yet exist

- Any stochastic force term (no thermostat of any kind).
- Any explicit notion of temperature (`k_B·T`) or thermodynamic ensemble.
- Any sampling validation infrastructure (equipartition tests, RDF tests).
- Any energy-based model training pathway.
- Any bistable mechanical example or "physical p-bit" reference design.
- A bridge to THRML or any block-Gibbs sampler.

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

The minimum viable first move. Add a `LangevinThermostat` that, each step,
writes a stochastic force into the existing `qfrc_applied` buffer:

```
qfrc_applied[i] += sqrt(2 * γ_i * k_B * T / dt) * randn()
```

where `γ_i` is the thermostat damping coefficient on DOF `i` (initially equal
to `dof_damping[i]`, later configurable independently of structural damping).

**Splitting strategy**: BAOAB-style — apply the stochastic Ornstein-Uhlenbeck
update to velocities in closed form *outside* the existing integrator, so
that implicit-fast doesn't see the noise and can stay numerically clean. This
sidesteps the "implicit step + naive √dt noise → wrong invariant distribution"
problem.

**Where it lives**: Not a new crate. A small extension to sim-core's forward
step. A `Thermostat` trait + `LangevinThermostat` impl, opt-in, default off.

**Validation**: 1-DOF damped harmonic oscillator (mass on a spring), 10⁵
steps, measure `⟨½mv²⟩`. Must equal `½k_B·T` to within sampling error. Then
sweep `γ` and `T` to confirm scaling.

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
  distribution (start with a 2D Gaussian mixture).
- **Phase 6**: Bridge to THRML if `thrml-rs` exists (verify first, see Q3); if
  not, implement a minimal block-Gibbs sampler in Rust as the comparison point.
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
- **Q2 — Implicit integrator interaction.** Implicit-fast does a Newton solve
  on the smooth dynamics. BAOAB splitting (apply OU step outside the
  integrator) should sidestep this, but needs verification on a real model
  before Phase 1 ships. Concretely: does the integrator interface allow a
  "velocity update from the outside" between steps without breaking
  invariants?
- **Q3 — Does `thrml-rs` exist?** The original prompt asserted it does. Not
  verified. Resolution: a 5-minute web search before Phase 6 is committed. If
  it doesn't exist, fall back to a native Rust block-Gibbs sampler (a few
  hundred lines).
- **Q4 — Default `dof_damping` is zero.** A thermostat that scales noise by
  damping gives zero noise on undamped DOFs, which is wrong for the user's
  intent. Need to decide: (a) thermostat carries its own damping coefficient
  independent of model damping, or (b) thermostat requires the model to set
  damping. Tentatively (a) — separation of concerns.
- **Q5 — Is the cf-design → sim-core parameter pipeline already
  differentiable end-to-end?** Phases 5+ depend on it. Recon needed.
- **Q6 — What's the right reward signal for Phase 7?** ESS, integrated
  autocorrelation time, KL divergence to target, wall-clock to convergence?
  Different choices give different agents. Defer the decision to Phase 6
  results.

---

## 6. Spec Index

Child specs spawned from phases of the Gap will be linked here as they are
written.

- *(none yet — Phase 1 spec is the next artifact to produce)*

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
