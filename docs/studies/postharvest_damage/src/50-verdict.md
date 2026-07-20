# The fork

This chapter states the decision and leaves it open. It is the head engineer's to make.

## What three sweeps actually changed

The study began with a tonnage argument and a mechanism. Both failed.

| Going in | Coming out |
|---|---|
| ~1/7 of food lost; mechanical damage a large share | Share **cannot be derived** — FAO's model has no cause axis. Retracted. |
| Bruising initiates spoilage, so attribution understates it | Strong form **refuted** (0-3 on one paper, 1-2 on another). Mechanism plausible, unquantified anywhere. |
| Bruise mechanics is literature-ready for calibration | **No force-time data surfaced.** But the targeted repository search never ran — coverage gap, not searched negative. |
| The field is primitive | **Wrong** — a real frontier exists at KU Leuven. |
| No study reports damage-extent error | **Wrong, retracted** — kiwifruit 9.63%, ginseng 0.4–2.2%. |
| Validation is weak | **Survives, narrowed**: held-out status not established anywhere. |
| Simulation angle may be crowded | **Open** on inverse design, held-out validation, distributional prediction — at *medium* confidence, from ~7 papers. |

A later cold read added two more, both cutting against the program: that
[`sim-soft` cannot host plasticity](35-primitives.md) without a trait change, and that the
[micronutrient motivation](10-loss.md#what-survives-as-motivation) was never submitted to the sweeps
at all. Full record in the [appendix](appendices.md#revision-history).

**Note the asymmetry, because it is the most useful thing this table says.** Of the five
sweep-driven retractions, four left the program *more* attractive — the field being less primitive
upgraded the finding; damage-extent errors existing created a reproduction target. Both
review-driven corrections made it *less* attractive.

**Self-correction that only ever improves the case is not self-correction.** Weigh the retraction
count accordingly: it is evidence that this study was checked, not that it was right.

## The case for

**The gap is real, and it is shaped like our strengths.**

Four things the field does not do — [bridge sophisticated ↔ validated](30-gap.md#the-disjointness),
[validate held-out](20-mechanics.md#but-held-out-status-is-not-established),
[inverse design](30-gap.md#nobody-does-inverse-design),
[distributional prediction](20-mechanics.md#-the-variability-that-should-reshape-the-deliverable) —
are four things CortenForge is placed to do.

The differentiating asset is not the solver. Consider what passes for validation in this
literature: a headline 4.87% error that is a regression intercept compared against a bio-yield
measured from the same specimens; a 12% error from a method that prescribes the measured force
history as its input; an "external" validation dataset sharing three of five authors and supplying
the material parameters; a paper comparing its stress plots to another paper's stress plots. The
covenant this repo runs on — that a claim stays unverified until a committed, re-runnable
measurement against the real object is green — is not standard practice here.

Against that, one honest limit: **a process norm is not a technical moat.** Anyone can adopt
held-out validation; it requires discipline, not capability. The genuinely CortenForge-specific
contribution is [Gate 4](40-program.md#gate-4--inverse-design) — differentiable inverse design over
exact geometry — and that gate is currently blocked on an
[unresolved adjoint question](35-primitives.md#the-adjoint-question-which-is-worse).

## The case against

**The motivation is thin and partly unverified.** The tonnage argument is dead. What replaced it —
the micronutrient overlap — never entered verification and is flagged as author reasoning.

**The evidence base is small and unchecked.** ~7 papers per sweep. Adversarial counter-search never
ran. Wageningen's modern group, UC Davis, Cranfield, Washington State, Michigan State and USDA ARS
were **never reached**. Any could be doing this work.

**There is a named, unexamined counter-example.** [Van Zeebroeck's DEM bruise-depth
work](30-gap.md#-the-known-candidate-counter-example) on 'Jonagold' apples could sit in both columns
of the disjointness table. Nobody read it.

**Gate 0 costs more than advertised.** Plasticity in `sim-soft` is a trait change, not an `impl`,
and the adjoint may not survive it.

**Gate 3 is a research program, not a feature.** Months of bench work — and its justifying premise
rests on a search that never ran.

**It competes with the flagship.** The co-design arc is mid-flight: `RouteTarget` landed at #661,
conduit-radius next, real-anatomy L4 SDF after. This forks that arc rather than extending it.

**The philanthropic payoff is unverified**, and now also gates a build-vs-buy decision. If FE
licence cost is not a real barrier, [buying the forward solver](35-primitives.md#the-honest-alternative-nobody-costed)
for Gates 0–2 is likely cheaper than building plasticity.

## Why this is philanthropy and not a product

**The market will solve harvest robotics for high-value crops in wealthy countries** whether or not
we help. California strawberries and Dutch greenhouse tomatoes are crowded, well-funded commercial
spaces. Marginal contribution there is near zero.

**The market will not produce an open, validated, distributional produce-damage model**, because the
people bearing the losses cannot pay, and the fix is often a better crate, a padded chute, or a
changed handling protocol rather than a machine. Nobody funds that R&D because there is no product
at the end of it.

That is the gap philanthropy is for — conditional on the premise in
[Gate 5](40-program.md#gate-5--open-the-tooling-deliverable-not-a-gate) holding, which is unchecked.

## The decision

Four defensible positions. This study does not choose between them.

**A. Bounded spike, Gates 0–2.** Produces at least one publishable result. But it is **no longer
weeks** — Gate 0 needs a plasticity path, and Gate 2 probably needs bench data. Re-cost before
committing.

**B. Cheap-first: run the unattempted checks before any code.** Search the open-data repositories,
read Van Zeebroeck, email MeBioS, ask CGIAR whether licences are the barrier. Days of work,
resolves four load-bearing unknowns, and could kill or reshape the program before a line is
written. **Lowest regret of the four**, and it dominates A on cost.

**C. Full program including Gate 3.** Multi-year with a bench component. Justified only if the
philanthropic case is load-bearing for you personally; the technical case does not require it.

**D. Bank the study and walk.** The recon is durable. The four open lanes will still be open in a
year. The flagship has momentum.

## What we still do not know

Recorded so the next person does not re-run three sweeps to learn the same thing.

- **The competitive landscape is entirely unmapped.** Zero surviving claims across three sweeps on
  tooling, licence costs, development-sector practice, or patents. Probably not on the open web.
- **Whether the open-data repositories actually lack produce force-displacement data.** Never
  searched. Gates 3 and 5 both lean on the assumption that they do.
- **What Van Zeebroeck's DEM work actually validated.**
- **Whether the MeBioS 2019 calibration-data gap persists.**
- **What the locked paper says** — *Postharvest Biol. Technol.* 213, 112930, eluded three sweeps.
- **Whether the adjoint survives plasticity.** Answerable in-house, and blocks Gate 4.
- **Whether major unreached groups are already doing this.**
- **Whether ASABE S368 mandates force-deformation curve reporting.**

**The cheapest way to close most of these is not another sweep.** It is a handful of emails and one
repository search — which is option B, and why option B exists.
