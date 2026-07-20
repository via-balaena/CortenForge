# The fork

This chapter states the decision and leaves it open. It is the head engineer's to make.

## What three sweeps actually changed

The study began with a tonnage argument and a mechanism. Both failed.

| Going in | Coming out |
|---|---|
| ~1/7 of food lost; mechanical damage a large share | Share **cannot be derived** — FAO's model has no cause axis. Retracted. |
| Bruising initiates spoilage, so attribution understates it | Strong form **refuted** (0-3 on one paper, 1-2 on another). Mechanism plausible, unquantified anywhere. |
| Bruise mechanics is literature-ready for calibration | **Half right, established later.** Apple-tissue force curves *do* exist openly; force↔damage pairing does not. |
| The field is primitive | **Wrong** — a real frontier exists at KU Leuven. |
| No study reports damage-extent error | **Wrong, retracted** — kiwifruit 9.63%, ginseng 0.4–2.2%. |
| Validation is weak | **Survives, narrowed**: held-out status not established anywhere. |
| Simulation angle may be crowded | **Open** on inverse design, held-out validation, distributional prediction — at *medium* confidence, from ~7 papers. |

A later cold read added two more, both cutting against the program: that
[`sim-soft` cannot host plasticity](35-primitives.md) without a trait change, and that the
[micronutrient motivation](10-loss.md#what-survives-as-motivation) was never submitted to the sweeps
at all.

**Note the asymmetry, because it is the most useful thing this table says.** Of the five
sweep-driven retractions, three left the program *more* attractive — the field being less primitive
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
exact geometry — and that gate is
[no longer blocked](35-primitives.md#-the-adjoint-question-answered): the through-time adjoint
already exists and extends to plastic state by an established pattern.

There is also now a **physics** case, not just a methodological one. Diels (2019) measured the
viscoelastic contact law at R² 0.71 ± 0.20 against 0.90 ± 0.13 for visco-elastoplastic on 'Jonagold'
impact profiles, and concluded plastic dissipation *"can probably not be neglected"* at bruise scale.
**Conditionally** — below 0.3 m/s the plastic model is the worse of the two (0.51 vs 0.74). The case
holds in the bruising regime and reverses at gentle impacts.
That is the field's own leading group establishing why the sophisticated side needs plasticity — the
capability [Ch 4](35-primitives.md) says we would have to build.

## The case against

**The motivation is thin and partly unverified.** The tonnage argument is dead. What replaced it —
the micronutrient overlap — never entered verification and is flagged as author reasoning.

**The evidence base is small and unchecked.** ~7 papers per sweep. Adversarial counter-search never ran in
sweeps 2–3 (budget exhausted). Wageningen's modern group, UC Davis, Cranfield, Washington State, Michigan State and USDA ARS
were **never reached**. Any could be doing this work.

**Gate 0 costs more than advertised.** Plasticity in `sim-soft` is a trait change, not an `impl` —
the `Material` contract makes stress a pure function of `F`, with nowhere to put plastic history.
That cost is unchanged by anything the cheap checks found.

**Gate 3 has narrowed but not vanished.** Constitutive calibration data now exists openly; the
force↔damage pairing still does not, so a bench component survives unless it can be obtained by
asking the people who already measured it.

**A numeric result may still be hiding.** Van Zeebroeck's 2006 validation paper is closed at Elsevier
and absent from Lirias. Its abstract-level claim is qualitative, but a figure cannot be excluded.

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
weeks** — Gate 0 needs a plasticity path in `sim-soft`. Better founded than it was: real calibration
data now exists, and Gate 4 is unblocked.

**B. Finish the cheap checks.** Three are done (below). Two remain, both emails: **MeBioS** — is the
2019 gap fully closed, what else is unpublished — and **CGIAR/GIZ/PEF** on whether FE licence cost is
a real barrier, which decides build-vs-buy. Plus one ten-minute item: ask Hussein and
Scheffler for their unreleased force↔damage data. **Still the lowest-regret
next move**, though it is now most of the way done.

**C. Full program including Gate 3.** Multi-year with a bench component. Justified only if the
philanthropic case is load-bearing for you personally; the technical case does not require it.

**D. Bank the study and walk.** The recon is durable. The four open lanes will still be open in a
year. The flagship has momentum.

## What option B has already resolved

Three of the cheap checks were run. **All three moved the picture, and two moved it in the
program's favour.**

| Check | Result |
|---|---|
| **Search the open-data repositories** | **Partially closes Gate 3.** Apple-tissue force curves + DVC strain fields + microstructure now exist openly (`10.48804/GNBFGU`, MeBioS, May 2026, CC-BY-NC-SA). Force↔**damage** pairing still absent, and the structural reason is now known. |
| **Read Van Zeebroeck** | **Finding survives**, at reduced confidence — the thesis was never obtained and the characterization is second-hand. Separately, Diels 2019 (verified) gives a *conditional* physics case for plasticity: better above 0.3 m/s, worse below. |
| **Does the adjoint survive plasticity?** | **Yes.** Through-time adjoint already exists via tape-composed per-step VJPs; plastic state becomes another threaded parent. One `∂f_int/∂(plastic state)` term to derive. Gate 4 unblocked. |

Two are outstanding, and both are yours to send: **email MeBioS** (is the 2019 gap now closed — the
new dataset suggests partly, but ask what else is unpublished) and **ask CGIAR/GIZ/PEF whether FE
licence cost is actually a barrier**, which gates build-vs-buy.

## What we still do not know

- **The competitive landscape is entirely unmapped.** Zero surviving claims across three sweeps on
  tooling, licence costs, development-sector practice, or patents. Probably not on the open web.
- **Whether Du et al. 2019 is the origin of the withdrawn orthotropic table.** The Gate 0 table
  itself is settled; only the provenance of the discarded one is open.
- **Whether a numeric validation figure exists inside Van Zeebroeck's 2006 PBT paper** — closed at
  Elsevier and absent from Lirias. Cannot be excluded, only not found where it would be advertised.
- **What the locked paper says** — *Postharvest Biol. Technol.* 213, 112930, eluded three sweeps.
- **Whether Hussein's and Scheffler's unreleased force↔damage data can be obtained by asking.**
- **Whether major unreached groups are already doing this.**
- **Whether ASABE S368 mandates force-deformation curve reporting.**
- **Baheri 1997** (KU Leuven, potato mechanical damage, 301 pp) — a strong lead that is not digitally
  indexed anywhere.
