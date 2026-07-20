# The fork

This chapter states the decision honestly and leaves it open. It is the head engineer's to make.

## What three sweeps actually changed

The study began with a tonnage argument and a mechanism. Both failed.

| Going in | Coming out |
|---|---|
| ~1/7 of food lost; mechanical damage a large share | Share **cannot be derived** — FAO's model has no cause axis. Retracted. |
| Bruising initiates spoilage, so attribution understates it | Strong form **refuted 0-3**. Mechanism plausible, unquantified anywhere. |
| Bruise mechanics is literature-ready for calibration | **No force-time data exists** in accessible form. Confirmed ×3. |
| The field is primitive | **Wrong** — a real frontier exists at KU Leuven. |
| No study reports damage-extent error | **Wrong, retracted** — kiwifruit 9.63%, ginseng 0.4–2.2%. |
| Validation is weak | **Survives, narrowed**: held-out status not established anywhere. |
| Simulation angle may be crowded | **Open** on inverse design, held-out validation, distributional prediction. |

Five substantive positions this study advanced were killed by its own verification. That is the
process working, and it is why the [claim ledger](appendices.md#claim-ledger) records refutations
alongside confirmations rather than quietly dropping them.

## The case for

**The gap is real, and it is shaped like our strengths.**

Four things the field does not do — [bridge sophisticated ↔ validated](30-gap.md#the-disjointness),
[validate held-out](20-mechanics.md#what-survives-the-correction),
[inverse design](30-gap.md#nobody-does-inverse-design),
[distributional prediction](20-mechanics.md#-the-variability-that-should-reshape-the-deliverable) —
are four things CortenForge is unusually well-placed to do. Not because our solver is better;
because our **discipline** is different.

Consider what passes for validation in this literature: a headline 4.87% error that is a
regression intercept compared against a bio-yield measured from the same specimens; a 12% error
from a method that prescribes the measured force history as its input; an "external" validation
dataset sharing three of five authors and supplying the material parameters; a paper whose
validation is comparing its stress plots to *another paper's stress plots*. Meanwhile the
covenant this repo runs on is that
[a claim stays unverified until a committed, re-runnable measurement against the real object is
green](../../../../MISSION.md).

**That covenant is the contribution.** The solver is ordinary here. The discipline is not.

And the entry cost is lower than first assessed: [Gate 0](40-program.md#gate-0--can-we-reproduce-a-published-number)
through [Gate 2](40-program.md#gate-2--distributional-prediction) are **pure software**, use
published parameters, and each can kill the direction cheaply. Gate 1 alone — the skin ablation
nobody has run — would be a real contribution for a few weeks of work.

## The case against

Stated as strongly as it deserves.

**The motivation is thinner than it looked.** What survives is the
[micronutrient-overlap argument](10-loss.md#what-actually-survives-as-motivation): what bruises is
what people are nutritionally short of. That is real, but it is not a tonnage claim, and it will
not survive contact with anyone who has read the FAO methodology if you overstate it.

**The evidence base is small and unchecked.** Roughly seven papers per sweep. Adversarial
counter-search never ran — budget exhausted in two of three sweeps. Wageningen's modern group,
UC Davis, Cranfield, Washington State, Michigan State and USDA ARS were **never reached**. Any of
them could be doing exactly this work.

**Gate 3 is a genuine research program, not a feature.** Generating a calibration dataset is
months of bench work, specimens, a cold room, and a protocol. It is not a detour on the way to
something else — it *is* the thing, for a while.

**It competes with the flagship.** The
[co-design arc](../../../../MISSION.md) is mid-flight: `RouteTarget` landed at #661, conduit-radius
is the next rung, real-anatomy L4 SDF after that. This direction does not extend that arc; it
forks it. Two years of produce mechanics is two years not spent on the exo capstone.

**The philanthropic payoff is unverified.** [Gate 5](40-program.md#gate-5--open-the-tooling) rests
on the assumption that FE license cost is a binding constraint for the people bearing these
losses. **Three sweeps produced zero evidence for that.** It may be true. It is currently an
assumption, and the honest thing is to check it before it justifies effort.

## Why this is philanthropy and not a product

Worth stating precisely, because the distinction determines what to build.

**The market will solve harvest robotics for high-value crops in wealthy countries** whether or
not we help. California strawberries and Dutch greenhouse tomatoes are crowded, well-funded
commercial spaces. Marginal contribution there is near zero.

**The market will not produce an open, validated, distributional produce-damage model**, because
the people bearing the losses cannot pay, and the fix is often a better crate, a padded chute, or
a changed handling protocol rather than a machine. Nobody funds that R&D because there is no
product at the end of it.

That is the gap philanthropy is for. It is also, notably, **less fun to build** than a harvesting
robot — and that asymmetry is exactly why it stays open.

## The decision

Three defensible positions. This study does not choose between them.

**A. Take Gates 0–2 as a bounded spike.** Pure software, weeks not months, uses existing
primitives, produces at least one publishable result (the skin ablation), and either validates or
kills the direction on measured evidence rather than argument. Does not commit to bench work.
Does not fork the flagship. **This is the lowest-regret option and the one this study's structure
implies.**

**B. Commit to the full program including Gate 3.** Accepts that this becomes a multi-year
research direction with a bench component. Justified only if the philanthropic case is
load-bearing for you personally, because the technical case alone does not require it.

**C. Bank the study and walk.** The recon is done and durable. The four open lanes will still be
open in a year. The flagship co-design arc has momentum, and
[foundational ordering](../../../../MISSION.md) is a real principle. Nothing is lost by not starting.

## What we still do not know

Recorded so the next person does not re-run three sweeps to learn the same thing.

- **The competitive landscape is entirely unmapped.** Three sweeps, zero surviving claims on
  open-source tooling, license costs, development-sector simulation practice, or patents. This
  information is probably **not on the open web** — it needs direct contact.
- **Whether the MeBioS 2019 data gap persists.** Has anyone published the same-batch dataset they
  said was missing? This single question decides whether independent constitutive calibration is
  currently possible at all.
- **What the locked paper says.** *Postharvest Biol. Technol.* 213, 112930 eluded three sweeps and
  is the source most likely to overturn the "no consensus criterion" reading.
- **Whether major unreached groups are already doing this.** Wageningen (modern), UC Davis,
  Cranfield, Washington State, Michigan State, USDA ARS.
- **Whether ASABE S368 mandates force-deformation curve reporting**, and whether a calibration
  corpus could therefore be assembled from standards-compliant published work.

**The cheapest way to close most of these is not another sweep.** It is an email to the MeBioS
group asking what the field's unpublished state is. Academics generally answer, and they would
know things no paper says.
