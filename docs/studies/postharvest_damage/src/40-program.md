# What CortenForge would build, gated

This chapter derives a program from [the four open lanes](30-gap.md#four-open-lanes), sequenced by
the CortenForge covenant: **cheapest falsification first**, each rung capable of killing the
direction, no code written until the rung below it is green.

> **Read [Chapter 4](35-primitives.md) first.** `sim-soft` is hyperelastic and cannot express
> plasticity without a trait change, so **none of the gates below is free** — the costs here are
> stated on that assumption.
>
> Every gate carries an explicit **NO-GO** condition. Where one genuinely cannot fail, it is labelled
> a *deliverable* rather than a gate.

## Gate 0 — Can we reproduce a published number?

**The load-bearing unknown.** Everything downstream is worthless if this fails.

**Target: kiwifruit cv. 'Xuxiang'** — Zhu et al., *Foods* 2024, 13(21):3523,
`10.3390/foods13213523`. One paper, verified from the PDF. It supplies **both** the constitutive
table *and* the 9.63% bruise-area result, so target and oracle are the same study.

Its Table 3 gives skin / flesh / core Young's modulus, tangent modulus, bio-yield stress, density and
Poisson ratio — **with SDs on modulus, bio-yield and density** — plus the collision-surface properties (steel,
PVC, neoprene) needed to set up the drop. Transcribed in the
[appendix](appendices.md#kiwifruit--xuxiang--the-gate-0-target-table--verified-from-the-pdf).

> **⚠ Ginseng looks like the better target and is not usable.** The ginseng study (2025) is
> methodologically stronger — its parameters came from independent uniaxial compression tests rather
> than a citation, so reproducing it would not be circular. But **no ginseng parameter values exist
> anywhere in this study's evidence base**; only the model class was recovered. Extracting them is
> unattempted work, not a starting point.
>
> **⚠ Ignore any "Xu et al. 2024 kiwifruit" parameter table.** No such paper exists in Crossref, and
> the orthotropic values circulated under that name appear nowhere in the Zhu PDF —
> [detail in the appendix](appendices.md#kiwifruit--xuxiang--the-gate-0-target-table--verified-from-the-pdf).

**Prerequisite.** A plasticity path in `sim-soft`: widened or parallel `Material` contract, internal
state per quadrature point, return mapping, consistent tangent. This is the real cost of Gate 0 and
it is unestimated.

**The task.** Bilinear isotropic strain-hardening elastic-plastic tissue, von Mises > bio-yield
criterion, published impact condition, compare predicted bruise area against published value.

**PASS.** Bruise-area prediction within the published error band using only published parameters, no
tuning.

**NO-GO.** If we cannot reproduce a bilinear elastic-plastic result that a commercial FE package
produced, the problem is our solver and this direction is not ours. **Stop.**

**ALSO NO-GO — the cost branch.** If the plasticity trait change proves to require restructuring the
solver's tangent path or breaks the existing adjoint, stop and reconsider
[the buy-not-build alternative](35-primitives.md#the-honest-alternative-nobody-costed) before
continuing.

> Reproducing a mean-vs-mean number at one condition is a **conformance check**, not a validation of
> the underlying science. It proves our engine belongs in the conversation. It says nothing about
> whether the criterion is right — that is Gate 1.

## Gate 1 — The ablation nobody ran

**The question the field left open.** Skin is 6.8× stiffer than flesh where it is modelled, and is
often not modelled at all. Nobody has tested whether that changes bruise-volume prediction.

> **Both premises are medium-confidence.** The 6.8× ratio is one cultivar of one commodity, from a single isotropic table; the
> "usually ignored" pattern holds across three of four papers and the field-wide version was
> **refuted 0-3**. See [Ch 2](20-mechanics.md#skin-is-often-ignored-and-where-measured-it-is-68-stiffer).
> If either premise fails on wider reading, this gate loses its point before it starts.

**The task.** Run the Gate 0 geometry twice — homogeneous vs. explicit skin/flesh/core — and report
the delta in predicted bruise **volume**, not just contact pressure and area.

**PASS.** A defensible delta with stated uncertainty, either direction.

**NO-GO.** If the with/without-skin delta is *inside* the specimen-to-specimen scatter measured at
Gate 2, the ablation is unresolvable at this precision and publishing it would be noise. Stop and
reorder: Gate 2 first, Gate 1 after, or drop Gate 1.

> **This gate is cheap *given Gate 0*** — a multi-material mesh run on machinery that by then
> exists. It is **not** independently cheap: pricing it alone, without the plasticity prerequisite,
> understates it by the entire cost of Gate 0.

## Gate 2 — Distributional prediction

**The constraint.** Damage-onset energy varies ~7× across specimens *(medium confidence, as of
Molema 1999)*. Deterministic prediction of an individual specimen is close to a category mistake,
and no stochastic or Monte Carlo FEM on produce surfaced in any sweep.

**The task.** Propagate parameter distributions through the Gate 1 model; predict a **distribution**
of bruise volume rather than a point value.

**PASS.** A predicted distribution whose *spread* is commensurate with reported experimental
scatter. Not a matched mean — a matched spread.

**NO-GO.** If the predicted spread cannot be made commensurate with experiment — or if published
scatter turns out too sparse to score against — then damage prediction is not distributionally
tractable from public data, and Gate 4's objective function has no honest form. Stop and reconsider
whether the deliverable is a model at all.

> **✅ Seeding does not require bench data.** The Gate 0 target table reports **mean ± SD** for
> Young's modulus, tangent modulus, bio-yield and density across all three tissues — enough to seed a
> first parameter distribution directly.
>
> Two limits. Those SDs describe **specimen-to-specimen scatter within one cultivar at one maturity**,
> so they will understate the ~7× damage-onset spread seen across harvest dates and ripeness. And the
> *damage-outcome* scatter needed to score the predicted spread is still thin. **Gate 2 can start on
> published data but may not finish on it.**

## Gate 3 — Our own calibration data

**The first bench rung — and its scope has narrowed since the repository search ran.**

> **✅ Force curves exist; the damage pairing does not.** The
> [repository search](30-gap.md#-the-repository-search-run) found `10.48804/GNBFGU` (Van Cauteren et
> al., **MeBioS**, CC-BY-NC-SA, May 2026): force–displacement curves for apple cortex under in-situ
> compression, with DVC strain fields and per-cell morphology on the same specimens, all CSV.
>
> **So do not generate constitutive calibration data — that now exists.** What is still missing is
> force curves paired per-specimen with *damage outcomes*, and the search identified why: damage gets
> published per specimen, force curves get reduced to scalars before publication.
>
> **Ask before measuring.** Hussein (2019, Opara group) recorded per-fruit force-deformation curves
> on a 10 kg load cell and published only mean ± SE — the curves exist and were never released.
> Scheffler (2018) has per-specimen peak-force-vs-bruise scatter that is digitizable. An email is
> cheaper than a cold room.

**The task.** Force-displacement curves under ASABE S368-style compression with full specimen
metadata (cultivar, ripeness, turgor, temperature, storage duration, contact geometry, loading
rate), paired per-specimen with measured damage. Then publish openly.

**PASS.** A dataset sufficient to calibrate a constitutive model *and* fit a damage criterion, with
enough specimens to characterize scatter.

**NO-GO.** If Hussein, Scheffler or another group will release force↔damage data on request,
**skip this gate** — it becomes redundant and its cost should not be paid. Ask before building a rig.

> **★ This is where held-out validation becomes possible for the first time.** With our own specimens
> we control the train/test split — the thing
> [no paper in the evidence base documents](20-mechanics.md#but-held-out-status-is-not-established).

## Gate 4 — Inverse design

**The flagship rung**, and where CortenForge's machinery becomes load-bearing rather than incidental.

> **✅ No longer blocked.** The through-time adjoint already exists (per-step VJPs composed on the
> tape), plastic state becomes another threaded parent by an established pattern, and associated J2
> plasticity keeps the tangent symmetric. **But this is not a one-term change** — the return map is a
> nested implicit solve, the yield switch is non-smooth, and the state is a tensor per quadrature
> point. See [Ch 4](35-primitives.md#-the-adjoint-question-answered).

**The task.** A validated distributional damage model as an **objective function**, optimizing a
design variable — crate wall profile, liner compliance, divider spacing, chute curvature — via the
same `cf-codesign` machinery `RouteTarget` uses against a body SDF.

> body ↔ device co-design → **produce ↔ container co-design**

**PASS.** A geometry the optimizer found that measurably reduces damage against a hand-designed
baseline, **verified experimentally**.

**NO-GO.** If gradients through the damage physics prove unusable — non-smooth, or dominated by
specimen noise — fall back to surrogate or Bayesian optimization and re-evaluate whether a
*differentiable* engine was ever the right tool. That fallback would remove most of this program's
CortenForge-specific justification, and should be treated as a serious negative result rather than a
detour.

> **This gate is bench-dependent**, despite reading as a software rung. Its pass criterion requires
> experimental verification, and the sequence diagram below marks it accordingly.

## Gate 5 — Open the tooling *(deliverable, not a gate)*

Publish the validated model, parameter database, calibration dataset, and optimizer as an open,
free, documented tool.

This **cannot fail on technical grounds** and is therefore not a gate. It is the philanthropic
payoff, and it is contingent, not conditional.

> **⚠ Its premise is unverified and now also load-bearing for build-vs-buy.** Whether commercial FE
> licence cost is *actually* a binding constraint for the intended users was **not established by any
> sweep** — three attempts, zero surviving claims. If licences are not the barrier, this deliverable
> is worth little, *and* the
> [buy-not-build alternative](35-primitives.md#the-honest-alternative-nobody-costed) becomes
> substantially more attractive for Gates 0–2.
>
> Check it — by asking CGIAR, GIZ, or the Postharvest Education Foundation — **before** it justifies
> effort. This is [open question 8](appendices.md#open-questions).

## Sequence and scope line

```
Gate 0  reproduce a published number     NEEDS PLASTICITY   ← falsifies the solver
Gate 1  skin ablation                    software           ← first new knowledge
Gate 2  distributional prediction        software*          ← *likely needs Gate 3 data
─────────────────────────────────────────────────────────── ← the fork (Ch 6)
Gate 3  own calibration dataset          BENCH              ← enables held-out validation
Gate 4  inverse design                   software + BENCH   ← flagship; adjoint resolved
Gate 5  open tooling                     deliverable        ← cannot fail; premise unverified
```

**Scope line, by coupling to the figure of merit** — not by category:

| In scope | Out of scope |
|---|---|
| Soft-tissue contact mechanics | Microbial spoilage, pathogen entry |
| Tissue constitutive models incl. plasticity | Ripening biochemistry, ethylene kinetics |
| Damage criteria and thresholds | Cold chain, refrigeration, CFD |
| Exact geometry → contact | Logistics, routing, market economics |
| Distributional / stochastic prediction | Post-hoc bruise *detection* |
| Gradient-based geometry optimization | Harvest robotics for high-value crops |

Post-hoc bruise detection via hyperspectral imaging and deep learning is an active, crowded field —
sweep 1 surfaced Mask R-CNN strawberries, SWIR apples, a 61-cultivar blueberry YOLO, and MRI/ANN
pear volume. That is *measurement after the fact*, orthogonal to *prediction before the design*. Do
not drift into it.
