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
> | **Status** | Literature recon complete (3 adversarial sweeps, 323 agents). **No code written. No decision made.** |
> | **Verdict** | The field has a real structural gap and it is shaped like our strengths. The tonnage argument that motivated the study **does not survive** and has been retracted. |
> | **★ The finding** | **The sophisticated constitutive work and the quantitatively-validated work are disjoint sets.** Nobody bridges them. See [Ch 3](30-gap.md). |
> | **The bet** | Not "simulate fruit" — people do that. **Held-out validation + inverse design + distributional prediction**, none of which the field currently does. |
> | **Load-bearing unknown** | Whether our solver can reproduce a *published* damage-extent error at all. Unproven, and cheap to falsify. This is [Gate 0](40-program.md#gate-0--can-we-reproduce-a-published-number). |
> | **First buildable rung** | Reproduce the ginseng or kiwifruit bruise-area result from published constitutive parameters. **Pure software — no bench work.** See [Ch 4](40-program.md). |
> | **Scope line** | By *coupling to the figure of merit*. **In**: soft contact, tissue constitutive models, damage criteria, geometry, inverse design. **Out**: microbial spoilage, ripening biochemistry, cold chain, logistics, CFD. |
> | **Honest ceiling** | Damage-onset energy varies **~7× within a single lot**. The correct deliverable is *distributional*, not deterministic. See [Ch 2](20-mechanics.md#-the-variability-that-should-reshape-the-deliverable). |
>
> **Where to go:** what the loss numbers can't say → [Ch 1](10-loss.md) · what's known about the mechanics → [Ch 2](20-mechanics.md) ·
> the gap → [Ch 3](30-gap.md) · the gated program → [Ch 4](40-program.md) ·
> the fork → [Ch 5](50-verdict.md) · claim ledger & parameters → [appendix](appendices.md).

## The question

CortenForge's transferable core is **differentiable mechanical multiphysics + geometry +
gradient-based co-design**. The flagship loop is body ↔ device co-design. This study asks
whether that loop transfers to a philanthropic problem by substituting one term:

> body ↔ device co-design → **produce ↔ container co-design**

Same optimizer, same soft-contact physics, same covenant that
[exact geometry *is* the exact physics](../../../../MISSION.md) — a proxy fruit geometry lies about
where the load concentrated, which is precisely how a bruise appears where the model said it
would not.

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
[Ch 5](50-verdict.md#why-this-is-philanthropy-and-not-a-product).

## How this book is ordered

The order is deliberate and matches how every CortenForge arc is built —
**measure before you architect**, and *retract before you build*.

1. **The loss accounting first**, in [Chapter 1](10-loss.md), because that is what motivated the
   study and it is where the argument was weakest. The chapter leads with a retraction.
2. **Then the mechanics**, in [Chapter 2](20-mechanics.md): what constitutive models, damage
   criteria, and parameter values actually exist in the accessible literature, and how good the
   validation really is when you read the primary text rather than the abstract.
3. **Then the gap**, in [Chapter 3](30-gap.md). This is the load-bearing chapter and the reason
   the study was worth running.
4. **Then the ladder**, in [Chapter 4](40-program.md): a gated program whose first three rungs
   are pure software, each one capable of killing the direction cheaply.
5. **Then the fork**, in [Chapter 5](50-verdict.md), which is a decision for the head engineer
   and is deliberately left open.

## Epistemic status

Everything in this book comes from three adversarial research sweeps run in July 2026. The
method, the full claim ledger, and the refuted claims are in the [appendix](appendices.md).
Summary of the evidence base:

| | Sweep 1 | Sweep 2 | Sweep 3 | Total |
|---|---|---|---|---|
| Agents | 111 | 105 | 107 | **323** |
| Sources fetched | 28 | 22 | 24 | **74** |
| Claims extracted | 108 | 80 | 77 | **265** |
| Claims verified | 25 | 25 | 25 | **75** |
| **Confirmed** | 14 | 17 | 19 | **50** |
| **Refuted** | 11 | 8 | 6 | **25** |

Each verified claim was subjected to 3-vote adversarial verification; a claim needed 2 of 3
refutations to be killed. **One third of all claims that reached verification were killed.**
That ratio is the single most important fact about this evidence base, and several of the
casualties were claims this study's own author had advanced.

Three standing caveats apply to everything below, and are repeated where they bite:

- **Search budget was exhausted (200/200) during verification in sweeps 2 and 3.** Adversarial
  counter-searches could not be run. Absence of contradicting evidence is a tool artifact, not a
  verified null.
- **Paywalls held throughout.** ScienceDirect, Wiley, and MDPI HTML returned 403 consistently.
  Verification succeeded only via institutional repositories, PMC mirrors, `mdpi-res.com` PDF
  deploys, and local `pdftotext`. Some decision-relevant papers were never reached.
- **The competitive landscape is unmapped.** Three sweeps produced *zero* surviving claims on
  tooling, licensing, development-sector practice, or patents. See
  [Ch 5](50-verdict.md#what-we-still-do-not-know).

Where a claim was verified from full text, this book says so. Where only an abstract was
reached, it says that too. Where a sweep produced nothing, that is recorded as a negative
result rather than silently omitted — because in this study, absence of evidence has repeatedly
turned out to be decision-relevant.
