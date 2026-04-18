# The full design-print-rate loop

Parts 1–9 described a physics engine and a visual renderer. Parts 10 Ch 00–05 described the optimizer infrastructure that runs against them. This chapter is where everything composes into one loop — the north-star workflow that the whole platform exists to serve. An inherent leaf, no sub-chapters: the loop is too tightly coupled to decompose into independent pieces, and the structural diagram + time budgets + phase-staged rollout belong together.

The loop is simple to state and load-bearing in its details. A designer declares an SDF-authored geometry and a reward function in `cf-design`. `sim-soft` evaluates the reward against physics, BayesOpt proposes the next design, the loop iterates for a batch of fast simulation steps, then converges on a small shortlist of candidates. Those candidates go to the printer. The printer produces parts, the designer measures and rates them, and the ratings flow back into two updates: the sim-to-real residual GP and the preference-learned reward-composition weights. Then the sim-side loop starts again with a corrected reward and a corrected simulator.

## The loop, diagrammatically

```text
    ┌──────────────────────────  OUTER LOOP (weeks) ───────────────────────────┐
    │                                                                          │
    │    ┌──────────────  INNER LOOP (minutes to hours)  ──────────────┐       │
    │    │                                                             │       │
    │    │   cf-design  ──▶  SdfField + MaterialField ── θ_i           │       │
    │    │        ▲                    │                               │       │
    │    │        │                    ▼                               │       │
    │    │        │             sim-soft forward map ── R_sim          │       │
    │    │        │                    │                               │       │
    │    │        │                    ▼                               │       │
    │    │        │          sim-to-real correction ── R̂_real(θ_i)    │       │
    │    │        │                    │                               │       │
    │    │        │                    ▼                               │       │
    │    │        │             BayesOpt posterior update              │       │
    │    │        │                    │                               │       │
    │    │        └─── next θ_{i+1} ◀──┘                               │       │
    │    │                                                             │       │
    │    │                 after N_sim iterations:                     │       │
    │    │           shortlist K candidates (top-K under R̂_real)      │       │
    │    │                                                             │       │
    │    └─────────────────────────────┬───────────────────────────────┘       │
    │                                  │                                       │
    │                                  ▼                                       │
    │                    physical print K designs (batch)                      │
    │                                  │                                       │
    │                                  ▼                                       │
    │               measure (force, pressure, diffusion, stiffness)            │
    │                                  │                                       │
    │                                  ▼                                       │
    │               designer rates pairs (A vs B haptic / visual)              │
    │                                  │                                       │
    │                                  ▼                                       │
    │       update sim-to-real residual GP + preference GP over weights        │
    │                                  │                                       │
    │                                  ▼                                       │
    │                          restart INNER LOOP                              │
    │                                                                          │
    └──────────────────────────────────────────────────────────────────────────┘
```

Two nested loops at different time scales. The inner loop ("minutes to hours") is the BayesOpt-over-sim loop of [Ch 02](02-bayesopt.md) running against `sim-soft` with sim-to-real correction from [Ch 05](05-sim-to-real.md); wall time is dominated by `sim-soft` forward evaluations at 50–500 ms each. The outer loop ("weeks") is the active-learning-over-prints loop of [Ch 04](04-active-learning.md) and the preference-learning loop of [Ch 03](03-preference.md); wall time is dominated by physical printing and designer rating sessions.

## Budget scales

A working intuition for the loop's cost structure, given the canonical problem's numbers from [Ch 00](00-forward.md) and the active-learning discussion from [Ch 04](04-active-learning.md):

| Loop element | Frequency | Per-unit cost | Budget per week |
|---|---|---|---|
| `sim-soft` forward + gradient | innermost, $\sim 10^4$–$10^5$ per shortlist | 70–250 ms (90% warm-start) | $\sim 10^5$ evaluations |
| BayesOpt posterior update | per sim sample | $\sim 10$ ms at $n \lesssim 500$ | free (amortized by sim cost) |
| sim-to-real correction application | per sim sample | <1 ms (GP evaluation) | free |
| Shortlist → physical print batch | 1–2 per week | 4–8 designs per batch, hours of print time | 5–20 prints |
| Physical measurement | per print | $\sim 15$ min / print on instrumented bench | 3–5 hrs/wk |
| Pairwise rating | per batch | $\sim 2$ min per pair, $\binom{K}{2} \sim 20$ pairs per 8-design batch | 40 min/wk |
| Residual GP + preference GP update | per batch | $<1$ s | free |

The ratios: $\sim 10^5$ sim evaluations per week, 5–20 prints per week. The inner loop is roughly four orders of magnitude busier than the outer loop. The whole architecture is structured around this ratio — every sim evaluation is essentially free compared to every physical print, every designer-minute is a scarce resource compared to every CPU-minute.

The corollary: optimization-algorithm choices that trade sim evaluations for print evaluations are nearly always correct. BayesOpt's sample efficiency *on the inner loop* matters less than people think — a 2× change in sim sample count is a 2× change in something that is already four orders of magnitude cheaper than the binding constraint. Efficiency on the *outer* loop is what the whole optimization stack is really designed for, and this is why the cost-aware active learning of [Ch 04](04-active-learning.md) is load-bearing rather than a refinement.

## What each inner-loop pass actually does

Given a current state — a residual GP with $n_\text{real}$ physical measurements, a preference GP with $n_\text{pref}$ pairwise ratings, an inner BayesOpt GP with $n_\text{sim}$ sim evaluations for this geometry — one inner-loop pass does:

```rust
use sim_ml_chassis::{Tape, Tensor};
use sim_soft::{SoftScene, ForwardMap, EditResult};
use sim_opt::{BayesOpt, AcquisitionConfig};

pub fn inner_loop_step(
    scene: &mut SoftScene,
    forward: &mut dyn ForwardMap,
    bayesopt: &mut BayesOpt,
    sim_to_real: &SimToRealCorrection,
    preferences: &PreferenceModel,
    tape: &mut Tape,
) -> InnerStepOutcome {
    let theta_next = bayesopt.propose_next(&AcquisitionConfig::default());

    let (r_sim, edit) = forward.evaluate(&theta_next, tape);
    let (grad_r, grad_est) = forward.gradient(&theta_next, tape);

    let r_hat_real = sim_to_real.correct(&theta_next, &r_sim);
    let weights    = preferences.current_weights();
    let r_scored   = r_hat_real.score_with(&weights);

    bayesopt.observe(theta_next.clone(), r_scored, grad_r, grad_est);

    InnerStepOutcome { theta: theta_next, edit, gradient_est: grad_est }
}
```

This composition closes the picture from [Ch 00's ForwardMap trait](00-forward.md) through every other chapter in Part 10: `forward.evaluate` is Ch 00, `bayesopt.propose_next` is Ch 02, `sim_to_real.correct` is Ch 05, `preferences.current_weights` is Ch 03, and `bayesopt.observe` with the gradient flag is where Ch 02's noise-tolerant acquisition design pays off. Every chapter's API surface is a line or two here.

The outer loop (after $N_\text{sim}$ inner passes) selects the top-$K$ designs under the scored, corrected reward and emits them as a print batch:

```rust
pub fn outer_loop_shortlist(
    bayesopt: &BayesOpt,
    k: usize,
) -> Vec<Tensor<f64>> {
    bayesopt.top_k_by_posterior_mean(k)        // diversity-promoted via determinantal sampling
}
```

Diversity promotion (determinantal point process, or a simple clustering + selection scheme) prevents the shortlist from collapsing to $k$ nearly-identical designs. This detail matters because the physical-print batch is expensive per slot, and wasting a slot on a near-duplicate of another slot forfeits most of that slot's information contribution to the residual GP.

## The phase-staged build

The full loop does not arrive in one deliverable. Per [Part 11 Ch 03's committed order](../110-crate/03-build-order.md#the-committed-order) and [Part 12's milestones](../120-roadmap/06-optimization.md), it lands in four stages:

- **Phase D** ships the inner loop on a *synthetic* 2-parameter design space with no sim-to-real correction, no preference learning, and hard-coded reward weights. This is the "first working `sim-soft`" milestone. The inner loop proves end-to-end gradient flow and BayesOpt convergence; the outer loop is a unit test with synthetic measurement data.
- **Phase G** makes `cf-design` authoring live, extends the inner loop to the real design space (10–50 parameters), and adds the change-detection-driven warm-start path from [Part 7 Ch 04](../70-sdf-pipeline/04-live-remesh.md). Still no physical printing — the sim-to-real correction is dormant (zero residual) and the reward weights are hard-coded.
- **Phase I** adds the visual layer: rendered preview matches the designer's mental model well enough that "looks right" becomes a legitimate inner-loop signal alongside "computes well." The preference GP comes online to learn weights against visual-match ratings over rendered designs; no physical printing yet.
- **Post-Phase-I (not on the Phase A–I roadmap)** closes the physical-print loop. `MeasurementReport` ingestion, residual GP online updates, batch-rating workflow, cross-print drift detection. This is when the platform stops being a simulator and starts being the engineering-workflow tool the thesis promised.

The physical-print loop is not scheduled inside the Phase A–I roadmap because it requires a specific printer, sensors, and measurement protocol that are out-of-scope for the simulation crate. [Part 12 Ch 07](../120-roadmap/07-open-questions.md) names it as the roadmap's highest-priority post-Phase-I deliverable.

## What "convergence" means

The loop does not converge to a point in the classical sense. It converges to a *steady-state distribution* over designs that the designer rates as satisfactory. Three measures that make this concrete:

- **Shortlist stability.** Two consecutive outer-loop iterations produce overlapping top-$K$ shortlists by $\ge 80\%$. No new design beats the existing shortlist's top members in the corrected posterior.
- **Residual GP saturation.** The residual GP's posterior entropy on the relevant design region falls below a threshold; new prints do not meaningfully update the bias model.
- **Preference stability.** The preference GP's inferred weight vector has converged modulo slow drift attributable to designer drift (which is its own signal: maybe the requirements changed).

All three converging together is the signal that the design problem — *this* design, under *this* reward — has been solved. The loop then runs at a maintenance tempo, prints occasional calibration samples to keep the residual GP fresh, and waits for the next design problem.

## What this loop is not

It is not a black-box optimizer hiding behind a fit button. Every element — the sim-side reward, the gradient, the correction, the weights, the rating — is inspectable and overridable by the designer. The whole platform is built on the thesis that the right design tool exposes the physics, not hides it; the full loop inherits that commitment. A designer who wants to fix the weights, disable the residual correction, or turn off preference learning does so through well-defined API toggles. The default loop is the sophisticated one; the escape hatches are explicit.

It is also not a replacement for the designer's judgment. The loop proposes designs and corrects the reward model from ratings; it does not decide what to design. Goal setting, constraint declaration, and acceptance criteria remain the designer's call. The tool is an optimizer, an oracle, and a calibrator — not an auteur.

## Where this lands in the roadmap

[Part 12 Ch 00](../120-roadmap/00-dependencies.md) shows the dependency graph; this full loop sits at the top of it, with every other milestone as a prerequisite. [Part 12 Ch 06](../120-roadmap/06-optimization.md) is the optimization-specific milestone — it consumes Ch 06 of this Part as the capability specification. The roadmap answers *when*; this chapter answers *what the thing is*.

When the roadmap concludes through Phase I and the post-Phase-I physical-print loop comes online, the platform's original question — "can we build a SOTA soft-body stack that serves real engineering workflows?" — reduces to "does this loop run as advertised on a real printer with real materials, producing parts the designer accepts?" If yes, the book's thesis is demonstrated. If no, the gap is named, scoped, and fed back into the next revision of the stack. The book does not promise the first answer; it promises the instrumentation to find out.
