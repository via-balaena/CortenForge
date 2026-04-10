# Research Directions

> Extracted from `MASTER_PLAN.md` §2 during the 2026-04-09 doc-tree refactor.
>
> **The only EDIT in the refactor**: each direction now carries a
> `Foundation status:` field (D1, D2, D4 viable; D3 BLOCKED on Q5
> per recon log part 13; D5 NOT RECON'D). The §2 *Synthesis*
> subsection moved to [`synthesis.md`](./synthesis.md). The §2
> *Priority ladder* subsection was DELETED — build-order
> sequencing lives in [`../03_phases/overview.md`](../03_phases/overview.md)
> and per-direction stack-fit fields, not as a global ranking.

The Vision above is abstract. **These are the concrete experiments that
realize it.** They are not the build order — Phase 1 still has to ship
first — but they are the *why*. The Master Plan exists to keep these alive
through implementation.

Five experiments + one foundational connection, ordered from earliest
payoff to longest horizon. Each direction states: concept, physics
grounding, the experiment, where it fits in the build, and why it matters.

### Status legend (added by doc review N3, 2026-04-09)

Each direction carries a `Status: idea / <reach> / <novelty>` line.
The categories:

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

A second per-direction field, **Foundation status**, was added
during the 2026-04-09 doc-tree refactor. It records whether the
direction's substrate (the layers it depends on) is currently in
place, blocked by an open question, or has not yet had a foundation
recon round. `viable` means the direction can be attempted once
its `Stack fit` phase is reached. `BLOCKED on QN` means a specific
open question must be resolved (or worked around) before the
direction is reachable. `NOT RECON'D` means no foundation recon
round has been done yet — the direction is a candidate, not a
green light.

## D1 — The agent learns to *harvest* thermal noise (flashing ratchet)

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

**Foundation status**: viable. Phase 1 (Langevin thermostat) is the
gating precondition; everything else (asymmetric potential, binary
actuator, RL training loop) is achievable on the substrate already
in place. No open-question blockers.

## D2 — Stochastic resonance, RL-tuned

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

**Foundation status**: viable. Buckling-beam from cf-design (already
shipped) + thermostat (Phase 1) + RL (already shipped) is sufficient.
No Q-class blockers; the open work is integration, not foundation.

## D3 — Co-design: geometry × policy × temperature (the triple-product)

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

**Foundation status**: **BLOCKED on Q5.** Recon log part 13
(2026-04-09) resolved Q5 with answer **NO**: cf-design ships analytic
∂f/∂θ at the SDF level only; the full design → sim → loss pipeline
is not end-to-end differentiable. D3 as originally framed (joint
gradient-based optimization across geometry × policy × temperature)
cannot ship until either the differentiable layer is rebuilt past
the current SDF-only boundary, or the experiment is reframed around
a surrogate model and the boundary is explicitly accepted. See
[`../04_recon_log/2026-04-09_part_13_q5_cf_design.md`](../04_recon_log/2026-04-09_part_13_q5_cf_design.md)
for the three breaks and the disposition options.

## D4 — Sim-to-real on a 3D-printed thermodynamic device

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

**Foundation status**: viable. Differentiability is not on D4's critical
path — the EBM is trained in simulation, parameters are frozen, then the
device is printed. The external dependencies above are real engineering
effort (printer, calibration, measurement rig) but they are *parallel
builds*, not foundation gaps in CortenForge itself.

## D5 — Brownian computer near the Landauer bound (moonshot)

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

**Foundation status**: **NOT RECON'D.** No foundation-level recon
round has been done for D5. Stack-fit is plausible at Phase 6+ but
the specific questions — whether mechanical-lattice energy
landscapes can be designed to embody a target computation,
whether near-Landauer operation is achievable in this stack, what
the validation gate even looks like — have not been investigated.
Recon when D1–D4 have closed (per the "don't attempt unless
D1–D4 are working" guidance above). Tracked as Q7 in
[`../02_foundations/open_questions.md`](../02_foundations/open_questions.md).

## Foundational connection — energy-based RL meets thermodynamic devices

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
