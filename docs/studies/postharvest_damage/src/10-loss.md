# What the loss numbers can and cannot say

> **Retraction, up front.** This study was motivated by the claim that roughly one seventh of
> food produced is lost between harvest and retail, and that mechanical damage is a large and
> under-attributed share of it. The first half is real. **The second half cannot be supported,
> and the specific mechanism proposed — that bruising initiates spoilage which then gets
> misattributed away from mechanical causes — was adversarially refuted.** The tonnage argument
> is retracted. A different and narrower argument survives; it is at the
> [end of this chapter](#what-actually-survives-as-motivation).

## The headline figure is real, and it is modelled

The canonical number is FAO's **Food Loss Index** (SDG indicator 12.3.1a): **13.3% globally in
2023**, up from 13.0% in 2015 when monitoring began. Verified verbatim against the FAO SDG data
portal, unanimous 3-0.

Scope matters and is routinely misquoted. The FLI covers on-farm, transport, storage, wholesale
and processing — it **excludes retail and consumer waste**, which are the separate UNEP Food
Waste Index (12.3.1b). Conflating the two is how "one third of all food" enters conversation.

The 0.3 percentage-point movement over eight years is almost certainly inside model noise and
should not be read as a measured increase. What it does say is that there has been **no
progress** against SDG 12.3's halving target.

### By commodity

| Commodity group | 2023 | 2015 |
|---|---|---|
| **Fruits & vegetables** | **25.4%** | 23.2% |
| Meat & animal products | 14.0% | — |
| Roots, tubers, oil-bearing crops | 12.3% | — |
| Cereals & pulses | 8.4% | — |

Fruits and vegetables lose at **3.02×** the rate of cereals. FAO attributes the gap to "high
perishability and handling requirements."

> **Artefact caveat, and it is a real one.** FAO's imputation model explicitly "imputes losses by
> food group, assuming loss levels are similar among products of a similar nature," using these
> exact clusters. The cross-commodity contrast is therefore **partly a product of the model's own
> grouping structure**, not a purely measured contrast. It is *consistent with* the mechanical
> fragility of soft high-moisture produce. It is not *evidence for* it.

### By region

| Region | 2023 |
|---|---|
| Sub-Saharan Africa | 23.0% |
| Least Developed Countries | 19.9% |
| Small Island Developing States | 19.0% |
| Northern America & Europe | 10.0% |

A 2.3× spread. Note that the FLI is a value-weighted aggregate over each country's top
commodities, so part of the gap reflects commodity mix and general infrastructure rather than
handling and transport specifically.

## The FLI is an imputation, not a measurement

This is stated plainly in FAO's own methodology working paper, and it is the reason the rest of
this chapter exists:

> "estimates for country-product pairs … can be very sensitive to changes in the method or the
> data, implying a high level of uncertainty on such detailed results, even at the country level.
> The results have proven more stable at the aggregated regional or global level … all results
> presented must be considered with caution."

Supporting facts, all verified:

- FAO "relies on auxiliary data used as proxy variables to estimate losses."
- Loss records were **~7% of reported FAOSTAT records** over 1990–2016; only ~4.4% of country-year
  loss observations are official reported data.
- The FLI is **not published at country level**.
- Indicator 12.3.1 was originally classified Tier III — the tier reserved for indicators with no
  established methodology.

The estimator specification has itself changed between versions (random-effects in the cited
document; a later variant described as fixed-effects with random-forest selection over 200+
variables). The load-bearing points hold across versions, but **cite the vintage**.

## ★ The pivotal negative finding: there is no cause axis

This is the finding that dismantled the study's original framing, and it was verified 3-0 by
direct extraction of the full 52-page FAO working paper.

> **FAO's model contains no physical-cause attribution axis. A mechanical-damage share cannot be
> extracted from it, because the model does not have one.**

Causes enter only through the HLPE micro/meso/macro *socio-economic* taxonomy. Mishandling is
named exactly once, as a micro-level cause — "the decision to harvest at a non-optimal moment or
mishandling of produce" — and **never as a model variable**.

The enumerated variable list is exclusively socio-economic, price, and infrastructure:

- energy and fertilizer prices
- electricity access
- credit to agriculture
- GDP, trade, education and extension spending
- metal prices, as a storage proxy
- World Bank Logistics Performance Index

**No impact energy. No bruise incidence. No drop height. No vibration variable.** Anywhere.

Two honest qualifications:

1. Weather and climate *do* enter, via the World Bank Climate Knowledge Portal. So "no physical
   causes" is strictly true for **mechanical** causes only.
2. FAO's separate **Food Loss and Waste Database** (37,000+ data points across 700+ publications)
   *does* record narrative causes including mechanical damage — but as a literature repository,
   not as part of the estimation model. Whether it is minable for a mechanical share is an
   [open question](appendices.md#open-questions).

The consequence for this study is direct: **any mechanical-damage share must be built bottom-up
from primary literature.** Asking the headline statistics to decompose was asking a question the
data structurally cannot answer.

## Three measurement regimes that disagree

Independent evidence diverges sharply from the modelled figures, in both directions.

**Field-measurement review.** Ambuko et al. (2025), *Frontiers in Horticulture*, finds a median
whole-supply-chain fruit-and-vegetable loss of **16.2%**, against FAO-lineage figures of 44%
(2011) and 22% (2019 harvest-to-retail). The authors attribute the gap to "assumptions made when
modeling FLW estimates using limited data points."

> Caveats that matter: this is a literature review, not a PRISMA systematic review; 16.2% is an
> *unweighted median over heterogeneous case studies* with differing commodities, boundaries and
> methods, not commensurate with a production-weighted modelled global estimate; and the 44%/22%
> comparators are SOFA/Gustavsson-lineage loss-*and*-waste figures, related to but **not identical
> with** the ~13.3% FLI. Extend the critique to the FLI via FAO's own methodology admissions,
> not via these numbers.

Regional medians from the same review — North Africa ~34%, sub-Saharan Africa ~25%, Europe ~22%,
Latin America/Caribbean ~19%, Oceania ~17%, Asia ~12%, North America ~8% — survived verification
only 2-1 and rest on very thin, likely selection-biased evidence (North America n=7, North Africa
n=8, Europe n=13, against Asia n=78 and sub-Saharan Africa n=66). The Asia figure of 12% sits
notably below most FAO-lineage estimates and is the clearest internal tension. **Present these as
literature medians with heterogeneous methods, never as measured regional loss rates.**

**Household survey data.** Nationally representative LSMS-ISA self-reports put national maize
harvest and postharvest loss at **1.4% (Malawi) to 5.9% (Uganda)** — far below FAO's 8% for the
same stage and APHLIS's 10–12%. Underlying source is Kaminski & Christiaensen (2014), *Food
Policy*.

> Two qualifications, and one disqualification. Self-reports likely **understate**: Ambler,
> de Brauw & Godlonton (2018), using a detailed loss module across 1,200 Malawi households rather
> than a single aggregate question, found far more farmers experiencing losses, with conditional
> losses of 5% maize / 8% soya / 12% groundnut. The surveys are also ~15 years old (2008/09–2010/11).
>
> **The disqualification: this is maize.** A dry grain whose loss mechanisms are storage and pest
> driven. It carries essentially no information about mechanical bruise damage in fruits and
> vegetables and **must not be used to argue anything about the mechanical share in produce.**
> It is included here only to show that a third independent measurement regime exists and
> disagrees with the other two.

**Do not present any single number as the truth.** Three regimes, materially different answers,
overlapping scopes.

## ★ The coupling hypothesis: partially supported, strong form killed

The original argument ran: mechanical damage breaches the skin, which admits pathogens and
spikes respiration, so fruit that was *mechanically* damaged is ultimately recorded as lost to
*spoilage* — meaning direct attribution understates the mechanical share.

### What survived (3-0)

Controlled pendulum impacts produce a **dose-dependent physiological response**, not merely
cosmetic injury:

- **Pomegranate** cv. 'Helow' (*Foods* 2023, 12(6):1122): pendulum impacts at 45° = 1.18 J and
  65° = 2.29 J, absorbed energy computed as $E_a = mg(h_1 - h_2)$ from rebound height, n=24/group,
  28 days at 5 °C and 22 °C. Bruise area and volume "increased significantly" with impact level;
  respiration and ethylene were "significantly (p<0.05) dependent on bruising impact level,
  storage temperature, and storage duration."
- **Banana** cv. Malindi (*Current Research in Food Science* 2023, n=219): peak ethylene
  production **3.85×10⁻³ mg/kg/h** on day 2 in high-impact (0.273 J) fruit at 22 °C.

### What was refuted (0-3 and 1-2)

**The actual load-bearing claim.** That impacted fruit is ultimately lost through a *microbial*
endpoint which cause-attribution would misclassify as spoilage rather than mechanical damage —
refuted at the source level, because the source papers do not support that reading as stated.

Also refuted, and listed so they do not leak back in: the specific respiration-spike figure
(14.02 mg/kg/h), the weight-loss figure (20.39%), the day-21 ethylene peak, and the potato
50–100 g damage-threshold range.

### Limits on what survived

Even the surviving finding is weaker than it first reads:

- In the pomegranate study, storage **temperature dominates** respiration and ethylene magnitude
  (~5× higher CO₂ at 22 °C than 5 °C), and pairwise 1.18 vs 2.29 J p-values for those variables
  are not isolated. The dose effect is **temperature-confounded**.
- Banana is **climacteric**, so the impact effect is superimposed on an autocatalytic ethylene
  rise. It must not be generalised to non-climacteric produce.
- Both papers measure ethylene, **not a ripening-rate model**.

**Verdict on the coupling hypothesis:** the mechanism is plausible and is standard postharvest
physiology. It is *not established* by this evidence base, and **no study anywhere in the
evidence base quantifies the coupled share.** It cannot carry an argument.

## What actually survives as motivation

Strip out everything above that failed, and one argument is left standing. It is not about
tonnage.

**The overlap between what bruises and what people are nutritionally short of is nearly total.**

Loss is concentrated in fruits, vegetables, roots and tubers — the perishable, high-moisture,
mechanically fragile commodities. Those are also precisely the micronutrient-dense foods whose
absence defines hidden hunger. Cereals, which barely bruise, are where calories come from and
are not the problem.

That argument does not require a mechanical-damage share, does not depend on the coupling
hypothesis, and is unaffected by the FLI's methodology. It is narrower than the one this study
started with, and it is the only one that survived three sweeps.

Whether it is *sufficient* motivation is a judgement call, made in [Chapter 5](50-verdict.md).
