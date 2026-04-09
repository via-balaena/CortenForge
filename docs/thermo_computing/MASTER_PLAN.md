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

## 0. Working Principles — Sharpen the Axe

This initiative is run on the *sharpen-the-axe* discipline. **We do not
rush.** The goal is not to ship fast; it is to ship work we are proud of,
on a foundation that doesn't have to be redone.

The house-building analogy: best plans, best materials, best subcontractors.
Pride, vision, wisdom. Apply these principles before any tactical decision
on this branch:

- **Recon depth before commitment.** Take an extra round of investigation
  rather than commit to a scheme on intuition. When a question has a
  literature answer (constrained Langevin, BAOAB error analysis, Landauer
  bound, stochastic resonance), read the literature first. The cost of an
  extra recon round is hours; the cost of building on a wrong foundation
  is months.
- **Validation must pass with margin.** Every phase has a validation gate
  in The Gap. The gate must pass with *margin*, not at threshold. If a
  test passes by 1% on a metric where the theory predicts 0%, that is a
  finding to investigate, not a green light.
- **Two schemes, then choose.** When proposing an algorithm, propose two
  approaches and compare honestly before committing. Single-option
  proposals are sketches, not decisions.
- **Document the *why*, not just the *what*.** Every meaningful decision
  goes in the Recon Log with its reasoning. Future-us — or a beginner
  reading the code months from now — should be able to reconstruct *why*
  we chose what we chose without having to re-derive it.
- **No batching.** One phase fully shipped, validated, and reviewed before
  the next opens. No batching of validation runs across phases. No
  batching of recon items across phases. One thing at a time, with
  attention.
- **Foundations over scaffolding.** When a shortcut creates technical debt,
  pay the debt now. The A-grade bar is not negotiable on this line. If a
  fix is foundational, take it even if it's breaking — bad foundations
  compound across phases more than anywhere else in the project.
- **Stop and ask when uncertain.** If a recon item turns up something
  surprising or contradictory, stop the implementation cadence and surface
  it. Surprises are information; ignoring them costs more than addressing
  them.

These principles apply with strength to *every* phase of this initiative,
including the early ones. Phase 1 is not a "warm-up" — it is the
foundation, and a sloppy Phase 1 will undermine every phase above it.

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

> **Revised 2026-04-09 (part 4)** — superseded the original `qfrc_applied`
> design after recon found `cb_passive`. See Recon Log part 4 for the
> trade-off table and full reasoning.

The minimum viable first move. Implement a `LangevinThermostat` struct
that installs itself as a `cb_passive` callback on the model and writes
both an explicit damping force *and* a stochastic force into `qfrc_passive`:

```
qfrc_passive[i] += −γ_i · qvel[i]  +  sqrt(2 · γ_i · k_B · T / h) · randn()
                   └── damping ──┘   └────── FDT-paired noise ──────┘
```

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

**Validation**: 1-DOF damped harmonic oscillator with `M=1`, `k_spring=1`,
`γ=0.1`, `k_B·T=1`, `h=0.001`. Run 10⁵ steps after a burn-in. Measure
`⟨½ M qvel²⟩`. Must equal `½ k_B·T` to within sampling-error tolerance
(target: well below `±2%` at this sample count — per sharpen-the-axe
"validation must pass with margin"). Then sweep `γ ∈ {0.01, 0.1, 1.0}`
and `k_B·T ∈ {0.5, 1.0, 2.0}` to confirm linear scaling in `T` and
γ-independence of the stationary temperature. Passing this is the gate
to all of Phase 2+.

**Phase 5+ caveats** (flagged here, addressed in later phases):
1. **Derivatives / FD perturbation** — `cb_passive` fires inside
   `forward_skip()` because `mj_fwd_passive` is called from `forward_acc`,
   which `forward_skip` calls. For deterministic derivative computation
   in Phase 5, the thermostat needs gating or RNG snapshot/restore.
2. **Plugin passive forces fire after `cb_passive`** (`passive.rs:723-731`).
   If a model has plugins contributing passive forces, the thermostat's
   noise will be present when the plugin runs. Not a Phase 1 issue (no
   plugins) but worth being aware of.
3. **BatchSim parallel envs** need *one* `LangevinThermostat` instance
   per env, not a shared instance. Avoids lock contention on the RNG
   mutex and keeps RNG streams independent across envs. The API should
   make the wrong choice hard.

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
- **Q3 — Does `thrml-rs` exist?** The original prompt asserted it does. Not
  verified. Resolution: a 5-minute web search before Phase 6 is committed. If
  it doesn't exist, fall back to a native Rust block-Gibbs sampler (a few
  hundred lines).
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
