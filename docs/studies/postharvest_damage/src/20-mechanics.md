# What is actually known about bruise mechanics

This chapter surveys the accessible literature on modelling mechanical damage in produce:
constitutive practice, damage criteria, parameter values, validation quality, and specimen
variability. It is deliberately concrete — every number below was verified against primary text
unless flagged otherwise.

The short version: **there is more usable material here than the field's reputation suggests,
and the validation is worse than its abstracts suggest.**

## Constitutive practice: bilinear elastic-plastic

Four independent groups across four commodities converge on **elastic-plastic** models in
commercial FE software. Not viscoelastic as a primary model, not hyperelastic, and — in the
quantitatively-validated literature — **never poroelastic or turgor-based**.

| Study | Commodity | Constitutive choice |
|---|---|---|
| Szyjewicz et al. 2021 | Apple (drop impact) | Abaqus *Crushable Foam*, volumetric hardening |
| Xu et al. 2024 | Kiwifruit | E + tangent modulus + bio-yield, per tissue |
| Soheilsarv et al. 2024 | Persimmon (pendulum) | "elastic-plastic … non-linear explicit procedure" |
| Xia et al. 2021 | Carrot | Elastic-plastic + 2-term Prony viscoelastic series |

Viscoelastic Prony terms appear as an occasional *addition*. Hyperelastic, poroelastic and
turgor-based models are **absent from every source that reports a damage-extent error**.

> **Geographic caveat, load-bearing.** This evidence set is dominated by Chinese, Polish and
> Iranian applied ag-engineering groups. "Best practice" here means *what passes peer review in
> the applied journals*, not the research frontier. The frontier is
> [Chapter 3](30-gap.md).

### Parameters are frequently borrowed, not measured

The clearest case, verified from full text via `mdpi-res.com` PDF deploy:

> "The mechanical parameters of the apples were set as follows: Poisson's ratio 0.35 and
> elasticity modulus 5.0 MPa **[27]**"
>
> "Differences in peel and flesh were ignored to simplify the model, by assuming that the
> materials were all homogeneous **[27,30]**"

The bracketed citations are direct evidence of provenance-by-citation. This paper ran an
extensive 3-cultivar × 5-distance × 5-velocity × 5-temperature compression campaign, and its own
texture-profile data shows hardness varying with depth (0.90248 N/mm² at 5 mm vs 0.58272 N/mm²
at 25 mm) — **and still took two elastic constants from a citation and assumed homogeneity.**

Full parameter tables recovered across the sweeps are in the
[appendix](appendices.md#recovered-constitutive-parameters). They are real and usable. They are
also point estimates with **no reported scatter**, and one recovered table lists density in
g/mm³ at physically implausible values (~1000× off) — a quality signal about table-level rigor
that should temper any wholesale reuse.

### Skin is usually ignored, and it is 4–8× stiffer

Where skin *is* modelled separately, the stiffness contrast is large:

- Kiwifruit 'Xuxiang': skin E = 10.233 MPa against flesh E = 2.305 MPa axial / 1.346 MPa radial
  → **4.44× to 7.60×**.

Against that, the apple drop study states "The material was homogeneous throughout the entire
volume of the model," and its own authors concede that modelling "of individual layers (skin,
flesh tissue, and seminal nest) were often omitted," explicitly flagging a three-layer model as
future work. The carrot study models flesh and core with no skin, epidermis or periderm layer
anywhere.

> **★ Nobody has run the ablation.** Whether modelling skin as a separate stiffer membrane
> materially changes **bruise volume** prediction — as opposed to contact pressure and contact
> area — was answered by *no* verified source. No study in the evidence base ran the
> with-skin/without-skin comparison. This is a cheap, publishable, pure-software experiment and
> it is [Gate 1](40-program.md#gate-1--the-ablation-nobody-ran) of the proposed program.

## The damage criterion

### What the field actually uses

The de facto criterion is a **critical-stress threshold: von Mises equivalent stress exceeding a
measured bio-yield stress.** The carrot study states it plainly:

> "the von Mises failure criterion … was used to analyze the equivalent stress of the carrot. In
> other words, the region where the equivalent stress exceeded the bio-yield stress was judged as
> bruised"

Bruise extent is read as the *instantaneous spatial region* over threshold — the damaged region
grows from 1.9 ms to 2.2 ms and then stops. **There is no accumulation variable and no dose
integration.** No damage-accumulation law appeared in any verified source across three sweeps.

Energy appears in that paper only as a solver sanity check (hourglass energy < 5% of internal
energy), not as a criterion.

> This is **convention-by-repetition, not demonstrated consensus.** No verified source shows the
> field agreeing, and every claim asserting a systematic multi-criterion comparison was refuted.

### Absorbed energy is falsified as a simple criterion

Sweep 1 established this and sweep 2 confirmed it. Bruise susceptibility — defined as bruise
volume per unit impact energy — is **itself energy-dependent**:

| Impact level | Bruise susceptibility |
|---|---|
| Medium (0.160 J) | 8.37×10⁴ mm³/J |
| High (0.273 J) | 1.04×10⁵ mm³/J |

*(Banana cv. Malindi, day 12, 22 °C. Max bruise volume 2.09×10⁴ mm³.)*

Bruise volume is **superlinear** in energy, so a simple linear energy criterion is falsified by
the source's own data. Corroborating work on Korla pear via CT shows susceptibility varying with
both energy *and* impact count, with a threshold effect as damaged tissue loses absorption
capacity.

> **Two structural problems with energy as a criterion.** First, **partial circularity**:
> susceptibility is *defined* as bruise volume ÷ impact energy, so reporting it is not
> independent evidence that energy is the right predictor. Second, and decisive for our purposes,
> these datasets report bruise volume, area and susceptibility — **no force-time histories, no
> tissue constitutive parameters, no contact geometry.** They can validate an output curve. They
> **cannot calibrate a material model.** See [Ch 3](30-gap.md#the-calibration-data-does-not-exist).

### The one criterion discrimination test

Ghasemi et al. (2015), *Journal of Agricultural Machinery*, Ferdowsi University of Mashhad, open
access. Apple cv. 'Golab kohanz', Abaqus, viscoelastic, quasi-static indentation. This is the
closest thing in the entire evidence base to a head-to-head criterion test.

Measured shear strength: **0.23 MPa**.

| Deformation | FEM max shear | Bruising observed? |
|---|---|---|
| 2 mm | 0.195 MPa | No |
| 3 mm | 0.24 MPa | Yes |
| 4 mm | 0.26 MPa | Yes |

The threshold is bracketed by experiment. Meanwhile "at the three deformation levels of 2, 3, and
4 mm, the maximum generated normal stress inside the apple was above the point of failure" while
bruising "was almost zero" at 1–2 mm — **the max-normal criterion fires where no damage occurs.**

**Result: max-shear correct, max-normal produces false positives.**

> **Limits that matter.** Quasi-static only — this does *not* generalize to impact. Single
> cultivar, single group, one paper, coarse 3-level bracketing, no held-out validation. This is a
> consistency and bracketing check on the calibration specimens; "validated" overstates it. The
> paper itself notes the model overpredicts stress beyond bio-yield. And the **spatial** half of
> the result — that the FEM max-shear *location* matched the observed subsurface bruise location —
> was **refuted 0-3** and does not survive.

## Validation quality

This is the field's weakest link, and reading the primary text rather than the abstract changes
the picture substantially.

### The correction

An earlier draft of this study asserted that **no** published work reports a quantitative
predicted-vs-measured damage-extent error. **That was wrong, and is retracted.** Sweep 3
overturned it:

| Study | Quantity | Reported error |
|---|---|---|
| Kiwifruit drop-impact (*Foods* 2024, 13(21):3523) | Bruised surface area — 592.46 mm² sim vs 540.38 mm² measured (hyperspectral) | **9.63%** |
| American ginseng root, pendulum impact (2025) | Exterior bruise area / internal | **0.4–2.1% / 0.76–2.2%** |
| Apple drop (Southwest Univ., *Comput. Electron. Agric.* 2024) | "drop velocity and bruise volume" | **max 2.51%** |

The kiwifruit and ginseng results were verified from **PMC full text**. The ginseng constitutive
parameters came from **independent uniaxial compression tests** on cortex and cambium with a
yield-stress bruise criterion — so it is *not* circular in-sample curve fitting. The apple paper
was verified to **abstract level only**; its 2.51% is a pooled maximum across *both* validated
quantities, not specifically bruise volume.

> **Do not confuse two 2024 Xu/Liu apple papers.** The one reached is
> *Comput. Electron. Agric.* `10.1016/j.compag.2024.109024`. The one that eluded all three sweeps
> is *Postharvest Biol. Technol.* 213, `10.1016/j.postharvbio.2024.112930` — "Evaluation of bruise
> volume quantification methods using finite element analysis for apple." That second paper is the
> single most decision-relevant unread source in this study.

### What survives the correction

The **narrower** claim, and it is the one that matters:

> **Held-out status is not established anywhere.** No paper states that validation specimens were
> disjoint from the specimens used for constitutive calibration. All comparisons are mean-vs-mean
> at a single or few conditions, with no per-specimen scatter, no coefficient of variation, and no
> train/test split.

Specifics:

- **Kiwifruit**: 60 fruits used for tensile/compression calibration; geometry reverse-engineered
  from **one** "average-size" representative fruit; the 9.63% figure covers a **single condition**
  (0.5 m drop, steel plate, 0°) out of 27 simulated scenarios — the other 26 are simulation-only
  response-surface work. Specimen independence is never stated.
- **Ginseng**: "Five sets of tests were conducted for each drop angle," no specimen-level
  separation stated. Sub-1% error on a biological specimen is implausible as a *per-specimen*
  predictive error and should be read as mean-vs-mean.
- **Apple**: abstract silent on protocol.

> **★ Epistemic discipline, worth preserving.** A verifier attempt to assert that the ginseng
> validation was *definitively in-sample* was **refuted 0-3**. The defensible position is
> **"held-out status not established"** — never "established as in-sample." This distinction is
> load-bearing and should survive into any external write-up.

### Where the headline numbers come from

Several ostensibly strong validation numbers weaken sharply on inspection.

**The carrot 4.87% is calibration compared against calibration.** The "experimental" 43.1 N is
not measured — it is the zero-damage intercept of a logarithmic regression of damage rate against
impact force:

$$P = 0.76\ln(F) - 2.86, \quad R^2 = 0.9275, \quad n = 300$$

Setting $P = 0$ gives $\ln F = 3.763$, hence $F = 43.1$ N. And the FEM's 45.2 N is the instant
simulated equivalent stress reaches the bio-yield stress **measured from the same specimen
population**.

**The persimmon 12% contact-force error is partially circular**, because the force-time method
*prescribes the measured force history as the input boundary condition*. The genuinely predictive
setup — initial velocity — gives **21% force and 52% deformation**. The honest predictive numbers
are the bad ones.

**The apple drop study's "external" validation dataset** (Komarnicki et al. 2017, *Food Bioprocess
Technol.* 10:1479–1494) **shares three of five authors** with the FEM paper, and also supplied its
material parameters. Same-lab data used both to parameterize and to validate.

### And some studies report no validation at all

The 2022 apple compression paper, verified by `pdftotext` over the full 1597-line document:
**zero hits** for `error`, `validat*`, `deviat*`, `RMSE`. Its validation is one sentence of visual
comparison —

> "The stress distribution was similar to the shape of the apple damage, in a funnel shape"

— and it compares its stress plots to **other papers' stress plots** rather than to measurement:

> "The obtained stress distribution diagrams for static loads were similar to the impact load
> stress distribution diagrams of Celik et al."

The paper's impressive R² values (0.99904, 0.99982) are **polynomial fits to experimental data
only**, not sim-vs-experiment agreement. A skimming reader would be misled. The paper *does*
quantify bruise area experimentally — it simply never scores the FE model against it.

## ★ The variability that should reshape the deliverable

Damage-onset energy varies by roughly **7× within comparable material**:

- Parke (1963): one sample damaged at **0.09 J**; another absorbed **0.6 J** with no damage.
- Ghadge (1988): onset spanning **0.4 to 0.7 J**.

Molema (Wageningen PhD, 1999) states the consequence directly:

> "Therefore tissue discolouration resistance is variable (Hyde et al, 1993), also within the same
> lot… Consequently, it is rather complicated to derive a generally valid subcutaneous tissue
> discolouration threshold. Additional research is needed…"

Corroborated by a dedicated *Research needs / Damage threshold* section, by "No minimum threshold
energy level has been reported," by "Variation in tissue discolouration within treatments was
considerable," and by the thesis's formal proposition #7: *the mechanical load above which tubers
sustain damage is not known.*

> Date-stamp this as **"as of Molema 1999."** The still-open status could not be re-checked —
> search budget was exhausted. The biological-variability core is not time-sensitive; the
> "nobody has solved it" framing is.

**No stochastic or Monte Carlo FEM on produce surfaced in any sweep.**

The implication is a design constraint on any deliverable, not a footnote. With a 7× within-lot
spread, deterministic prediction of an individual specimen's damage is close to a category
mistake. **The correct output is a distribution.** That is fine — arguably better — for the actual
use case: you are designing a crate that will handle ten thousand apples, not predicting one
apple. But it means a deterministic solver alone is the **wrong deliverable**, and that has to be
designed in from the start rather than bolted on.

## What the empirical tradition looked like before FEM

Two threads worth knowing, because they are the field's actual foundation.

**Instrumented spheres.** The main tool for locating damage in a supply chain, and seriously
limited. Praeger et al., *Sensors* 2013, 13(6):7140 (Leibniz-ATB): on a 10 cm drop onto steel, a
Mikras sensor in a **real potato** read ~110 g while the same sensor in a polyurethane dummy, plus
IRD and TuberLog, each read ~200 g. Dummy density was deliberately matched (1.14 vs 1.08 g/cm³),
isolating the cause to **casing stiffness** — "the synthetic casing material was in general more
firm than the real tuber material." On PVC foam all devices converged (~50 g ± 20 g), so the bias
applies specifically to **stiff-contact events** (steel rollers, grading lines), not compliant
ones.

And the prediction limit, verbatim and verified 3-0:

> "A prediction of produce bruising probability based on measurements with these devices requires
> specific determination of relationships between impact acceleration and produce damage."

A 2022–2025 search returned only **post-hoc detection** methods (Mask R-CNN strawberries, SWIR
apples, 61-cultivar blueberry YOLO, MRI/ANN pear bruise volume). **None predict bruise from chain
acceleration alone.**

**Damage-boundary correlations.** Pang (Massey PhD, 1993 — *abstract only*) built a lumped
"Bruise Factor" correlated to handling damage, plus a threshold boundary curve in velocity-change
vs peak-acceleration space across Gala/Splendour/Fuji/Braeburn/Granny Smith and five contact
surfaces. Notably, he found that boundary **hyperbolic** rather than the linear form described in
prior studies. The IS-100/ASABE damage-boundary approach is the standardized descendant — and
that literature exists *because* thresholds must be measured empirically, per commodity.
