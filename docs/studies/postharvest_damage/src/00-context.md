# Why this study exists

> ## Program status — read this first
>
> This book is the head engineer's working recon for a possible CortenForge expansion:
> **validated, differentiable contact mechanics for mechanical damage (bruising) in produce**,
> pursued as philanthropic work on postharvest loss.
> It is optimized for fast recall by whoever picks the program up, not for a general audience.
>
> | | |
> |---|---|
> | **Status** | Literature recon complete. **No code written. No decision made.** |
> | **★ Finding** | **The sophisticated constitutive work and the quantitatively-validated work are disjoint sets** — the cellular models are unvalidated on damage magnitude, the validated models are bilinear elastic-plastic, and nobody bridges. See [Ch 3](30-gap.md). |
> | **Case for** | Four things the field does not do, all of which we are placed to do. |
> | **Case against** | Evidence base is ~7 papers per sweep, adversarial counter-search never ran, six major groups unreached, competitive landscape unmapped, and the philanthropic premise is unverified. |
> | **✅ Since the sweeps** | Three cheap checks run — [repository search](30-gap.md#-the-repository-search-run), [Van Zeebroeck](30-gap.md#the-candidate-counter-example-examined), [the adjoint](35-primitives.md#-the-adjoint-question-answered). Finding **survived and strengthened**; Gate 3 narrowed; Gate 4 unblocked. Detail → [Ch 6](50-verdict.md#what-option-b-has-already-resolved). |
> | **The bet** | Not "simulate fruit" — people do that. **Held-out validation + inverse design + distributional prediction.** *(That the field does none of these is a medium-confidence absence-of-evidence finding, not an established fact — see [Ch 3](30-gap.md#nobody-does-inverse-design).)* |
> | **Load-bearing unknown** | Whether our solver can reproduce a *published* damage-extent error at all. This is [Gate 0](40-program.md#gate-0--can-we-reproduce-a-published-number). |
> | **⚠ Largest unpriced cost** | **`sim-soft` is hyperelastic and cannot express plasticity without a trait change.** No gate is free. See [Ch 4](35-primitives.md#-the-blocking-gap-sim-soft-has-no-plasticity-path). |
> | **First buildable rung** | Reproduce the kiwifruit bruise-area result. Needs a plasticity material path first; the constitutive parameters themselves are published. |
> | **Scope line** | By *coupling to the figure of merit*. **In**: soft contact, tissue constitutive models, damage criteria, geometry, inverse design. **Out**: microbial spoilage, ripening biochemistry, cold chain, logistics, CFD, post-hoc bruise detection. |
> | **Honest ceiling** | Damage-onset energy varies **~7×** across specimens *(medium confidence; Parke 1963, as reported in Molema 1999)*. The correct deliverable is *distributional*, not deterministic. See [Ch 2](20-mechanics.md#-the-variability-that-should-reshape-the-deliverable). |
>
> **Where to go:** if you read one chapter, read [Ch 3](30-gap.md) — it carries the finding.
> Then [Ch 4](35-primitives.md) for what we'd have to build and [Ch 6](50-verdict.md) for the decision.
> Full path: [Ch 1](10-loss.md) · [Ch 2](20-mechanics.md) · [Ch 3](30-gap.md) ·
> [Ch 4](35-primitives.md) · [Ch 5](40-program.md) · [Ch 6](50-verdict.md) · [appendix](appendices.md).

## The question

CortenForge's transferable core is **differentiable mechanical multiphysics + geometry +
gradient-based co-design**. The flagship loop is body ↔ device co-design. This study asks
whether that loop transfers to a philanthropic problem by substituting one term:

> body ↔ device co-design → **produce ↔ container co-design**

Same optimizer, same soft-contact physics, same covenant recorded in `MISSION.md` — that a proxy
geometry lies about where the load concentrated, which is precisely how a bruise appears where the
model said it would not.

The narrow question:

> Is mechanical damage to fruits and vegetables a problem where **CortenForge's specific edge is
> load-bearing** — or one where our engine is incidental and the binding constraint is elsewhere?

## What this book is not

It is **not** a claim that simulation solves hunger. Most food insecurity is logistics,
economics, cold chain, and conflict; the historical wins came from seeds, fertilizer, and
policy. A physics engine has nothing useful to say about any of that. The scope here is
confined to the one place inside food systems where mechanical physics is plausibly the
binding constraint.

It is also **not** a business case. Harvest robotics for high-value crops in wealthy countries
is a crowded, well-funded commercial space where marginal contribution is near zero. The
philanthropic argument, if one survives, has to rest somewhere else — see
[Ch 6](50-verdict.md#why-this-is-philanthropy-and-not-a-product).

## How this book is ordered

[Chapter 3](30-gap.md) carries the finding and [Chapter 4](35-primitives.md) carries the cost. A
reader short on time should start there and treat Chapters 1–2 as supporting evidence.

The written order is nonetheless chronological — loss accounting, then mechanics, then the gap —
for one reason: **Chapter 1 is where this study's original motivation died**, and a reader who
meets the gap before the retraction will over-weight it. The retraction is short and comes first
in that chapter.

1. [**Chapter 1**](10-loss.md) — what the loss statistics can and cannot support. Opens with a
   retraction of the argument that motivated the study.
2. [**Chapter 2**](20-mechanics.md) — constitutive models, damage criteria, parameter values, and
   what the validation actually looks like when you read primary text instead of abstracts.
3. [**Chapter 3**](30-gap.md) — the finding.
4. [**Chapter 4**](35-primitives.md) — SDK inventory: what exists, what is missing, and the one
   gap that changes the cost of everything downstream.
5. [**Chapter 5**](40-program.md) — a gated program with explicit NO-GO branches.
6. [**Chapter 6**](50-verdict.md) — the fork, left open.

## Epistemic status

Everything here comes from three adversarial research sweeps run in July 2026. Method, full
ledger, and refuted claims are in the [appendix](appendices.md).

| | Sweep 1 | Sweep 2 | Sweep 3 | Total |
|---|---|---|---|---|
| Sources fetched | 28 | 22 | 24 | **74** |
| Claims extracted | 108 | 80 | 77 | **265** |
| Claims verified | 25 | 25 | 25 | **75** |
| Confirmed | 14 | 17 | 19 | **50** |
| Refuted | 11 | 8 | 6 | **25** |

**Read that table carefully, because it is easy to over-read.**

- **190 of 265 extracted claims were never adjudicated.** Exactly 25 per sweep reached
  verification — a budget cap, not a risk-ranked selection. The verified set is a 28% subsample
  chosen by an unstated rule.
- Of the 75 that were adjudicated, **25 were killed** — but that ratio describes the subsample,
  not the evidence base.
- **The single most important fact about this evidence base is not the kill rate.** It is that
  **adversarial counter-search never ran** in sweeps 2 and 3 (budget exhausted at 200/200), so the
  "no credible source disputes this" leg of verification is missing throughout.

Three standing caveats, repeated where they bite:

- **No counter-search.** Absence of contradicting evidence is a tool artifact, not a verified null.
- **Paywalls shaped the evidence set.** ScienceDirect, Wiley, and MDPI HTML returned 403
  consistently. Sweep 1's conclusion that the field was primitive turned out to be a
  **reachability artifact**, corrected only when sweep 3 routed through institutional
  repositories. Some decision-relevant papers were never reached at all.
- **The competitive landscape is unmapped.** Three sweeps produced *zero* surviving claims on
  tooling, licensing, development-sector practice, or patents.

Where a claim was verified from full text, this book says so. Where only an abstract was reached,
it says that. Where a sweep produced nothing, that is recorded as a negative result — with one
distinction this study got wrong on its first pass and now enforces: **"we searched and found
nothing" and "we never searched" are different claims**, and the appendix marks which is which.

### On this study's own retractions

Seven positions this study advanced were later killed — five by its own verification, two by a
subsequent cold read. They are tabulated in
[Ch 6](50-verdict.md#what-three-sweeps-actually-changed); the per-defect record is in
`git log docs/studies/postharvest_damage/`.

**Four of the five sweep-driven retractions left the program more attractive; both review-driven
ones made it less so.** Each correction is individually sound, but a study that only ever corrects
in its own favour is not correcting. Weigh the retraction count as evidence that this was checked,
not that it was right.

One caution carries into the body: the argument this study fell back on when the tonnage case died —
the micronutrient overlap in [Ch 1](10-loss.md#what-survives-as-motivation) — **was never submitted
to the sweeps.** It is author reasoning, and it is marked as such where it appears.
