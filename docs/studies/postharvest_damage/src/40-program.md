# What CortenForge would build, gated

This chapter derives a program from [the four open lanes](30-gap.md#four-open-lanes). It is
sequenced by the CortenForge covenant: **cheapest falsification first**, each rung capable of
killing the direction, no code written until the rung below it is green.

> **Correction carried forward.** An earlier reading of this evidence concluded that the entry
> cost was bench work — a materials tester, specimens, months before a line of solver code paid
> off. **That was wrong.** The ginseng and kiwifruit papers publish constitutive parameters *and*
> a damage-extent error, which makes them reproduction targets. **The first three gates are pure
> software.** Bench work enters at Gate 3, and only if the software gates are green.

## What already exists in the SDK

Inventory before architecture. The relevant primitives are already built and validated:

| Need | Existing primitive | Status |
|---|---|---|
| Soft-tissue FEM | `sim/L0/soft` — BE-FEM + IFT | ✅ validated (Ecoflex uniaxial 5.8% RMS) |
| Exact contact on real geometry | `cf-core` contact, mesh-SDF | ✅ MuJoCo-conformant |
| Geometry → physics, differentiably | `cf-mesh`, SDF pipeline | ✅ |
| Gradient-based co-design | `cf-codesign` | ✅ `RouteTarget` landed (#661) |
| Route/geometry design variables | `cf-routing` | ✅ |
| Coupled soft ↔ rigid, one tape | `sim/L1/coupling` | ✅ keystone complete |

Nothing in the first three gates requires a new engine. That is the point — this program is a
consumer of existing primitives, not a pretext to build new ones.

## Gate 0 — Can we reproduce a published number?

**The load-bearing unknown.** Everything downstream is worthless if this fails.

**Target.** The American ginseng root pendulum-impact study (2025) is the better first target,
because its constitutive parameters came from **independent uniaxial compression tests** on
cortex and cambium rather than from a citation — so reproducing it is not circular. Kiwifruit
(*Foods* 2024, 9.63% bruise area) is the backup, with full tissue-specific parameter tables in
the [appendix](appendices.md#recovered-constitutive-parameters).

**The task.** Implement bilinear isotropic strain-hardening elastic-plastic tissue in `sim/L0/soft`,
apply the von Mises > bio-yield criterion, run the published impact condition, and compare bruise
area against the published value.

**Pass.** Our bruise-area prediction lands within the published error band using only published
parameters — no tuning.

**Fail → stop.** If we cannot reproduce a bilinear elastic-plastic result that a commercial FE
package produced, the problem is our solver, not the field, and this direction is not ours.

> **Honest note on what passing means.** Reproducing a mean-vs-mean number at one condition is a
> *conformance* check, not a validation of the underlying science. It proves our engine belongs in
> the conversation. It proves nothing about whether the criterion is right — that is Gate 1.

## Gate 1 — The ablation nobody ran

**The question the field left open.** Skin is 4–8× stiffer than flesh where it is modelled, and is
usually not modelled at all. **Nobody has tested whether that changes bruise-volume prediction.**

**The task.** Run the Gate 0 geometry twice — homogeneous vs. explicit skin/flesh/core layers —
and report the delta in predicted bruise volume, not just contact pressure and contact area.

**Why this rung.** It is cheap, it is pure software, it uses parameters already recovered, and
**either outcome is publishable**. If skin omission barely moves bruise volume, the field's
homogeneous models are vindicated and a lot of complexity is unnecessary. If it moves it a lot,
a large fraction of the published literature is built on a disqualifying simplification.

**Pass.** A defensible number with a stated uncertainty, either direction.

> This is the first rung that produces **new knowledge** rather than conformance. It is also the
> cheapest possible contribution to the field, and it would be a reasonable place to stop and
> publish if the program goes no further.

## Gate 2 — Distributional prediction

**The constraint from [Ch 2](20-mechanics.md#-the-variability-that-should-reshape-the-deliverable).**
Damage-onset energy varies ~7× within a single lot. Deterministic prediction of an individual
specimen is close to a category mistake, and **no stochastic or Monte Carlo FEM on produce exists
anywhere in the evidence base.**

**The task.** Propagate parameter distributions — seeded from published standard deviations where
they exist — through the Gate 1 model, and predict a **distribution** of bruise volume rather
than a point value.

**Pass.** A predicted distribution whose spread is commensurate with the reported experimental
scatter. Not a matched mean — a matched *spread*.

**Why it matters for the deliverable.** The use case is designing a crate that will handle ten
thousand apples, not predicting one apple. A distributional objective — *P(bruise > threshold)*
— is both the honest output and the more useful one. It also has to be designed in here, at the
solver interface, rather than bolted on after Gate 4.

> **Risk flagged.** The parameter distributions this gate needs may not exist in published form.
> Every parameter recovered in three sweeps is a **point estimate with no reported scatter**. If
> seeding fails, this gate becomes the first one that genuinely requires bench data, and Gate 3
> moves ahead of it.

## Gate 3 — Our own calibration data

**The first bench rung, and it is unavoidable eventually.**
[The calibration data does not exist](30-gap.md#the-calibration-data-does-not-exist) — confirmed
across three sweeps and by the frontier group's own admission. No archived force-displacement
curves paired with damage outcomes, nothing in any open data repository.

**The task.** Generate what the field is missing: force-displacement curves under ASABE S368-style
compression, with full specimen metadata (cultivar, ripeness, turgor, temperature, storage
duration, contact geometry, loading rate), **paired per-specimen with measured damage** by the
Molema slicing protocol or a modern imaging equivalent.

**Then publish it openly.** No such dataset exists in Zenodo, Dryad, Figshare, Mendeley, or
4TU.ResearchData.

**Pass.** A dataset sufficient to calibrate a constitutive model *and* fit a damage criterion,
with enough specimens to characterize scatter.

> **★ This is the rung where held-out validation becomes possible for the first time.** With our
> own specimens we control the train/test split — the thing
> [no paper in the evidence base documents](20-mechanics.md#what-survives-the-correction). That is
> a larger contribution than the solver.

**Cost honesty.** This is a materials tester, a few hundred specimens, a cold room, and months.
It is the real fork in this program and it is the subject of [Chapter 5](50-verdict.md).

## Gate 4 — Inverse design

**The flagship rung**, and the one where CortenForge's specific machinery becomes load-bearing
rather than incidental.

**The task.** Take a validated, distributional damage model as an **objective function** and
optimize a design variable against it. The natural first target is container geometry — crate wall
profile, liner compliance, divider spacing, chute curvature — using the same `cf-codesign`
machinery that `RouteTarget` uses for route control points against a body SDF.

The substitution is exact:

> body ↔ device co-design → **produce ↔ container co-design**

**Pass.** A geometry the optimizer found, that measurably reduces damage against a hand-designed
baseline, verified experimentally.

> **Why this is genuinely novel.** [Nobody does inverse design](30-gap.md#nobody-does-inverse-design)
> in this field — confirmed at medium confidence across sweep 2. Every study is forward analysis;
> optimization is deferred to future work where mentioned at all. This gate is the one that would
> be a real methodological contribution rather than an accessibility one.

## Gate 5 — Open the tooling

**The philanthropic payoff**, and deliberately last, because it is worthless before Gate 3.

Publish the validated model, the parameter database, the calibration dataset, and the optimizer
as an **open, free, documented tool** — so that a researcher, NGO, or cooperative without an
Abaqus license can evaluate a crate design.

> **Unverified premise, flagged.** Whether commercial FE license cost is *actually* a binding
> constraint for the people bearing these losses **was not established by any sweep**. Three
> attempts produced zero surviving claims on tooling, licensing, or development-sector practice.
> This gate rests on an assumption, and the assumption should be checked — by talking to CGIAR,
> GIZ, or the Postharvest Education Foundation — **before** it is used to justify effort.

## Sequence and scope line

```
Gate 0  reproduce published number        pure software    ← falsifies the solver
Gate 1  skin ablation                     pure software    ← first new knowledge
Gate 2  distributional prediction         pure software    ← fixes the deliverable shape
─────────────────────────────────────────────────────────  ← the fork (Ch 5)
Gate 3  own calibration dataset           BENCH            ← enables held-out validation
Gate 4  inverse design                    software         ← the flagship
Gate 5  open tooling                      software + eng   ← the philanthropic payoff
```

**Scope line, by coupling to the figure of merit** — not by category:

| In scope | Out of scope |
|---|---|
| Soft-tissue contact mechanics | Microbial spoilage, pathogen entry |
| Tissue constitutive models | Ripening biochemistry, ethylene kinetics |
| Damage criteria and thresholds | Cold chain, refrigeration, CFD |
| Exact geometry → contact | Logistics, routing, market economics |
| Distributional/stochastic prediction | Post-hoc bruise *detection* (crowded ML field) |
| Gradient-based geometry optimization | Harvest robotics for high-value crops (crowded, commercial) |

The exclusions are deliberate and several were argued for explicitly. Post-hoc bruise detection
via hyperspectral and deep learning is an active, crowded field — three sweeps surfaced Mask R-CNN
strawberries, SWIR apples, 61-cultivar blueberry YOLO, MRI/ANN pear volume. That is *measurement
after the fact*, orthogonal to *prediction before the design*. Do not drift into it.
