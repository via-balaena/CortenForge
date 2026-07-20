# Sophisticated and validated are disjoint sets

This chapter carries the finding.

Two sweeps produced a picture of a primitive field: bilinear elastic-plastic models, borrowed
parameters, validation by eyeballing stress contours against photographs. That picture was
**wrong** — or rather, it was an artifact of which papers a web crawler can reach. Sweep 3 routed
around the paywalls through institutional repositories and found the frontier.

The frontier is real. It is also **disjoint from the validated work**, and that disjointness is
the finding.

## The frontier exists

**KU Leuven MeBioS** — Diels, Nicolai, Ramon, Wang & Smeets — published a discrete-element model
of tomato tissue at cellular scale: deformable visco-elastoplastic triangulated cell meshes under
turgor pressure, with explicit **cell-wall rupture at a critical stretch ratio** and **brittle
inter-cellular debonding**.

Verified from full text via the Lirias open post-print (6.9 MB PDF; `WebFetch` could not render
it, local `pdftotext` was required). The abstract opens:

> "Bruise damage in fruit results from cell wall failure and inter-cellular separation."

This settles a question this study had wrong. The working hypothesis going in was that MeBioS's
mechanics work was aimed at gas and moisture transport and texture, with damage a side interest.
**It is not.** The work is squarely damage-oriented, and it is genuinely sophisticated — the class
of model that would let bruise formation emerge from microstructure rather than be declared by a
threshold.

## The frontier is unvalidated on mechanics

The **only** quantitative error that model reports is **geometric**: microstructure NRMSE < 3%
against cesium contrast-enhanced micro-CT, comparing 649 real cells against 712 virtual ones.

That number is also **partly circular** — virtual cell volumes were sampled from the CT list, and
anisotropy was tuned by an imposed 1.25 stretch factor.

Mechanical validation is qualitative curve agreement plus literature range-overlap. And the model
does not agree well:

> "Quantitatively, the elastic modulus, failure strain and failure stress reasonably agree with
> experimental results from literature… **For compression loading, the values of elastic modulus
> and failure stress reported in literature are significantly higher than the simulated ones.**"

The comparison to prior work is stated as *range overlap with no error metric*. A grep for
`NRMSE`, `RMSE`, `R2`, `error` across the full text returns only the geometric NRMSE plus an
internal sensitivity figure. The single-cell curve was **fitted** — turgor calibrated to 4.3 kPa —
so it is not held-out.

The string `validat` appears **twice in the entire paper**.

## ★ The field says so itself

Two pieces of testimony against interest — the strongest available form of evidence for a claim
about a field's own gaps, because they come from the groups best positioned to deny it.

**First: the calibration data does not exist, per the people who needed it.**

The MeBioS authors state outright what experiments would be required and are not available:
tissue tension and compression tests with failure-propagation imaging (SEM or high-resolution
micro-CT), on specimens for which single-cell compression, pressure-probe, and 3D microstructure
data are *also* available — **all from the same batch of fruit.**

And they say what they did instead:

> "Due to a lack of experimental information on inter-cellular adhesion we assumed an
> uncomplicated debonding model and performed a parameter study."

**A parameter sweep substituted for calibration, because the data was absent.**

> Time-stamped ~2019 (latest reference 2018). This does **not** prove the gap persists in 2026 —
> see [open questions](appendices.md#open-questions).

**Second: constitutive laws are under-researched, per the field's leading review.**

A review coauthored by **Saeys and Nicolai** (KU Leuven MeBioS) and **Opara** (Stellenbosch
SARChI Postharvest) — precisely the groups hypothesized to be the sophisticated frontier — states:

> "However, fruit are complex materials and the constitutive laws that govern their mechanical
> behavior have **not received much attention**. More research in this area is, hence, required."

The passage directly follows the review's own survey of apple, pomelo, orange, pear, tomato and
kiwifruit FEM work. It is the review's assessment of the mechanics literature, not an aside.

## The disjointness

Plotted against the two axes that matter — how physical the constitutive model is, and whether the
damage prediction was scored against measurement — the literature occupies two corners and leaves
the one that matters empty.

```
                  quantitative damage-extent error reported
                         no                        yes
                 ┌──────────────────────┬──────────────────────┐
     cellular /  │  MeBioS DEM (tomato) │                      │
     micro-      │  turgor, rupture,    │                      │
     structural  │  debonding           │        E M P T Y     │
                 │                      │                      │
                 │  Diels 2019          │   ← what a validated │
                 │  NRMSE<3% geometry   │     damage model     │
                 │  only                │     would occupy     │
                 ├──────────────────────┼──────────────────────┤
     bilinear    │                      │  Kiwifruit    9.63%  │
     elastic-    │  Apple 2022          │  Ginseng   0.4–2.2%  │
     plastic     │  (no validation)     │  Apple*       2.51%  │
     (macro)     │                      │                      │
                 ├──────────────────────┼──────────────────────┤
     regression  │                      │  Van Zeebroeck       │
     (not a      │                      │  "sufficient         │
     constitutive│                      │   accuracy",         │
     model)      │                      │  no metric found     │
                 └──────────────────────┴──────────────────────┘
                                                    * abstract only
```

The top-right cell is the finding. Everything else is detail.

Now the same picture as a table, with the constitutive class made explicit.

| | Constitutive sophistication | Quantitative damage-extent error |
|---|---|---|
| **MeBioS cellular DEM** (tomato) | Turgor, visco-elastoplastic cell walls, rupture, debonding | ✗ — geometric NRMSE only; mechanics qualitative; under-predicts modulus |
| **Kiwifruit** (*Foods* 2024) | ✗ — isotropic bilinear elastic-plastic | ✓ 9.63% bruise area |
| **Ginseng root** (2025) | ✗ — bilinear isotropic strain-hardening | ✓ 0.4–2.2% bruise area |
| **Apple** (*Comput. Electron. Agric.* 2024) | ✗ (abstract only) | ✓ "max 2.51%" pooled |
| **Carrot** (2021) | Elastic-plastic + Prony | ✗ — its 4.87% is a **critical-force** error, not damage extent |

> **Carrot does not belong in the right-hand column**, though it is often read that way. Its 4.87%
> is a single scalar comparison of critical *force* — 45.2 N simulated against 43.1 N
> "experimental" — and [Ch 2](20-mechanics.md#where-the-headline-numbers-come-from) shows that 43.1 N
> is itself a regression intercept, not a measurement. Counting it would put the number of studies
> with a validated damage-extent error at four rather than three.

Every study that reports a damage-extent error is **bilinear elastic-plastic in commercial FE**.
No poroelasticity, no turgor, no microstructural cellular mechanics appears in *any* of them.

This holds **even for papers whose titles say "multiscale."** The kiwifruit study's "multiscale"
means three concentric macro tissue layers; its authors explicitly state that "seeds, tissue
fluids, and fibers… were ignored," and its material definition is
"the skin, pulp, and core were regarded as isotropic elastic-plastic materials" — an
E + tangent modulus + yield-stress triple, which **is** bilinear isotropic hardening in Abaqus.
The ginseng study is an "idealized bilinear isotropic strain-hardening elastic-plastic model" for
cortex with linear elastic cambium.

> **★ The accurate characterization is not "the field is primitive."**
> It is: **the sophisticated constitutive work and the quantitatively-validated work are disjoint
> sets, and nobody bridges them.**

### The candidate counter-example, examined

**Van Zeebroeck — DEM modelling of apple impact damage.** The review surveyed in this chapter cites
it as its bruise-prediction success story, and it was the one piece of work that might have sat in
*both* columns: discrete-element rather than bilinear elastic-plastic, applied to a damage-extent
quantity. It was retrieved and read.

**It does not bridge, and it is not in the same category.** Verified from the KU Leuven PhD thesis
(M. Van Zeebroeck, 2005, Ramon supervising):

- **One DEM particle is one whole apple** — the thesis states it directly: *"a particle is, for
  example, a fruit."* This is bulk-handling DEM for fruit-to-fruit and fruit-to-wall contact in
  crates and bins, not tissue microstructure.
- **The contact law is viscoelastic**, not elastoplastic: Kuwabara–Kono normal, a viscoelastic
  Mindlin–Deresiewicz tangential, parameters identified on a rheometer.
- **Damage is not in the constitutive model at all.** The thesis says the bruise models *"are not
  inherent to DEM"* — they are multiple linear and non-linear **regressions** mapping peak contact
  force and absorbed energy plus covariates (ripeness, stiffness, temperature, curvature, harvest
  date) onto bruise volume. No plasticity, no failure criterion, no cell rupture.

So it belongs in the validated-but-unsophisticated column, with a regression damage layer rather
than a physical one.

> **Correction to a widely-repeated detail.** The 2022 review describes this as predicting "bruise
> depth." The thesis reports bruise **volume**. The "depth" phrasing appears to originate in that
> one review sentence.

**Reported validation:** apples in boxes on an electro-hydraulic shaker against DEM with matched
initial conditions, concluded to model vibration bruise damage *"with sufficient accuracy."* **No
numeric error metric appears in any reachable source**, and the 2022 review characterizes it only as
having "showed that it was possible to predict bruise damage." The one paper where a number would
live — *Postharvest Biol. Technol.* 41(2):85–91 (2006) — is closed at Elsevier and **absent from
Lirias**, so a figure cannot be excluded, only not found where it would be advertised.

### ★ The same lab hit this gap twice

This is stronger evidence than the absence of a counter-example, because it is the field's leading
group documenting the gap in its own words.

**Diels (2019) measured the inadequacy directly.** Fitting 'Jonagold' impact profiles, the
Kuwabara–Kono viscoelastic law reaches **R² = 0.71 ± 0.20** against **0.90 ± 0.13** for her
visco-elastoplastic model, and KK *"does not succeed in fitting the data"* even tuned to a single
specimen. Her stated reason is this chapter's thesis:

> In case of large impacts, potentially resulting in bruise damage, this **plastic energy
> dissipation can probably not be neglected and thus a viscoelastic model will be insufficient.**

That is independent quantitative support for the claim that bruise-scale mechanics requires
plasticity — and it is the physics justification for
[Gate 0](40-program.md#gate-0--can-we-reproduce-a-published-number).

**And Diels' own work fails the bridge test in mirror image.** Her cell-scale model is validated
*structurally* — NRMSE < 3% on tissue geometry against micro-CT — while her actual bruise-damage
prediction remains **empirical regression**. Sophisticated side validated qualitatively; quantitative
side is regression. Same split, one generation later, inside the lab best positioned to close it.

The 2022 review states the position outright: the multiscale mechanical models *"need to be
incorporated into these DEMs."* As of 2022, from that institute, they are not.

## The calibration data does not exist

Three sweeps asked whether force-time or force-displacement curves paired with damage outcomes
exist in accessible form. This is the fact that decides whether independent constitutive
calibration is even possible.

**The answer is no.**

The closest thing found is the Molema PhD thesis (Wageningen, defended 30 November 1999), and it
is genuinely good work. It pairs controlled mechanical loading with measured damage **on the same
potato tubers**:

- Splitting 0.6 J over 9 impacts at one site cut discoloured volume by **64%** and depth by **33%**
  versus a single impact.
- Spherical impact bodies **doubled** both versus a flat one.
- n = 45 per treatment, crossed with 2 potassium levels and 3 storage periods (3/5/7 months).
- Damage assay is real and per-specimen: 4 days at 20 °C post-impact, ~1.5 mm calibrated slicing,
  discoloured area summed across consecutive slices.

The apparatus is well-documented — Chapter 4 is literally titled "A computer-controlled pendulum
to impact potato tubers precisely." Energies 0.600 / 0.200 / 0.067 J, three impact-body curvatures
(flat / R30 / R15), delivered-energy accuracy −3 to +8%, tubers absorbing ~70–80% of delivered
energy.

**And it is still not enough.**

> What is paired per-specimen is a **scalar energy dose** — joules, number of impacts — **not a
> force-time or force-displacement curve.** Instrumented acceleration traces exist only for one
> treatment-level subset, and what is archived is **means and standard deviations**
> (e.g. B∞ @ 0.600 J = 286.9 (14.2) g and 2.85 (0.12) ms; B15 @ 0.067 J = 64.3 (3.56) g and
> 4.00 (0.13) ms), **not raw traces.**

You cannot fit a constitutive model to a scalar energy dose.

### ★ The repository search, run

The three sweeps never actually searched the open-data repositories — that question was recorded in
their caveats as a coverage gap, not a negative finding. **It has since been run.** The result
splits the question in two, and only one half is still a gap.

**Force–displacement curves for apple tissue DO exist, openly and machine-readably.**

> **`10.48804/GNBFGU`** — Van Cauteren, Dequeker, Ty, Ongenae, Van Doorselaer, Soete, **Verboven,
> Smeets, Nicolai**. *Replication Data for: In-situ visualisation of the micromechanical deformation
> of apple tissue using 4D X-ray computed tomography with digital volume correlation.* KU Leuven RDR,
> available 2026-05-28, **CC-BY-NC-SA-4.0**.
>
> Contains (1) **force–displacement curves** from in-situ uniaxial compression of apple cortex cubes
> (4×4×4 mm³) during dynamic X-ray CT — three harvest stages (May/July/September) × two orientations
> (radial/tangential) × four replicates; (2) **internal 3D strain fields** by global finite-element
> digital volume correlation, as CSV; (3) **per-cell morphological measurements** from
> contrast-enhanced high-resolution CT. ~700 files, overwhelmingly CSV, with a Snakemake/Python
> pipeline reproducing all 12 publication figures. Raw CT volumes are archived separately.

Note the author list. **This is MeBioS** — the same group whose ~2019 paper is quoted above saying
the required experiments did not exist. It is substantially the same-batch dataset they specified:
tissue compression, internal deformation, and microstructure on the same specimens. It is three
months old at the time of writing.

**What still does not exist is the damage pairing.** That dataset characterizes micromechanics; it
does not carry bruise outcomes. No archived dataset was found anywhere pairing force curves
per-specimen with measured damage.

**And the search surfaced why**, which is more useful than the negative itself:

> Across ten theses read in full text, the consistent pattern is that **damage outcomes get published
> per specimen, while force curves get reduced to scalars before publication.** The two halves are
> routinely measured on the same fruit and then separated at the point of writing up.

Best partial sources for the pairing, all requiring work:

| Source | What it has | Catch |
|---|---|---|
| **Scheffler 2018** (Stellenbosch) | Per-specimen peak force vs bruise area/depth, n≈13 per impactor | Scatter *plots*, not arrays — digitizable |
| **Heap 1994** (Massey) | ~152 pp, one row per numbered apple: bruise depth, volume, energy absorbed; thousands of specimens | **No load cell in the thesis** — pairs with drop *energy*, never force. Needs re-OCR |
| **Hussein 2019** (Opara group) | Per-fruit force-deformation curves, TA-XT Plus, 10 kg load cell | Published only mean ± SE. **The curves exist and were never released** |

That last row is the actionable one: the data is held by identifiable people who demonstrably
measured it. Asking is cheaper than re-measuring.

> **Access gaps in this search, recorded honestly.** MDPI `/s1` supplementary endpoints 403'd, so
> "no supplementary material exists" for those papers is inference from absent in-text citation
> rather than direct confirmation. `research.wur.nl` is Cloudflare-blocked and its OAI endpoint
> returns 500, so two Wageningen theses were judged from titles alone. **Baheri 1997** (KU Leuven,
> potato mechanical damage, 301 pp) is a strong lead that is not digitally indexed anywhere.

One more finding worth carrying, because it shows how cleanly the two traditions are separated:
the Molema thesis — the deepest full-text produce-damage thesis reached — **contains no finite
element or computational mechanics whatsoever.** A `pdftotext` pass over 259,717 characters /
7,633 lines returns **zero hits** for `finite element`, `modulus`, `Young`, `stiffness`,
`constitutive`, `Poisson ratio`, `Hertz`, and `numerical model`. Its 24 hits for `elastic` are all
narrative prose or citations to other authors, and the elasticity discussion applies to the
instrumented *sphere* as a physical object, not to tissue. Its only models are generalized linear
mixed models fitted in GenStat. All three hits for `simulation` refer to hardware.

In this evidence base the empirical damage tradition and the computational mechanics tradition do
not touch — though "this evidence base" is two theses (1993, 1999) against a modelling literature
from 2019–2025, so the separation is *observed across* those sources rather than established as a
continuous historical fact.

## Nobody does inverse design

From sweep 2, and the reason this study matters to CortenForge specifically.

**Every study examined is forward simulation or method benchmarking.** No gradient-based, adjoint,
surrogate, Bayesian, or topology optimization of packaging, crates, liners, chutes, conveyors, or
grippers against a produce-damage objective surfaced anywhere.

The sharpest data point is the carrot paper, because it is already in an equipment-design context
— a root-stem separation device — and **still defers**:

> "Through kinematic analysis and dynamic analysis, the relationship between the speed of the pull
> rod and carrot damage can be established, so as to complete the structural optimization and
> working parameters optimization of root-stem separation" — framed as *"After that … can be
> carried out."*

No design variables, no objective function, no sensitivity or adjoint, no surrogate, no geometry
search anywhere in it. The one apparent counter-example was checked and dismissed: its
"three-factor and three-level orthogonal test" optimizes the **material-measurement protocol**
(compression rate, deformation, hold time), not the device.

> **Confidence: medium, and the scope is narrow.** This is absence-of-evidence from ~7 papers,
> none of which were selected to search for optimization work. It supports *"inverse design did
> not show up where you would most expect it."* It does **not** establish that no such work
> exists.

## Four open lanes

Collecting what the three sweeps established, the field currently does not:

1. **Bridge sophisticated ↔ validated.** The cellular models are unvalidated on damage magnitude;
   the validated models are bilinear elastic-plastic. Nobody has taken a microstructural
   constitutive model and scored it against measured bruise volume.
2. **Validate held-out.** No paper documents specimen independence between calibration and
   validation. Every reported error is mean-vs-mean at one condition.
3. **Do inverse design.** All forward analysis and parameter studies. Optimization deferred to
   future work where mentioned at all.
4. **Predict distributionally.** ~7× specimen-to-specimen variability in damage-onset energy
   *(medium confidence, as of Molema 1999)*, and no stochastic or Monte Carlo FEM on produce
   surfaced **in any sweep** — noting that sweep 2 recorded zero verified claims on the variability
   question, so this is thin rather than settled.

Each of those is a gap that a validation-disciplined, differentiable, geometry-exact engine is
unusually well-placed to close. That is the case for the program in
[Chapter 4](40-program.md).

## The honest counter-case

Stated plainly, because it should be argued against rather than buried:

- **The evidence base is small.** Pattern-level conclusions about "the field" rest on roughly
  seven papers per sweep. Individual numbers are solid — verified verbatim against primary text —
  but the generalizations are directional extrapolations.
- **Adversarial counter-search did not run.** Budget hit 200/200 during verification in sweeps 2
  and 3. The "no credible source disputes it" leg is **unchecked throughout**. Absence of
  contradiction is a tool artifact.
- **Major groups were never reached.** Wageningen's modern group, UC Davis, Cranfield, Washington
  State (Karkee), Michigan State, and USDA ARS were not examined at all. Their absence here is
  **unexamined, not evidence of absence.**
- **The single most decision-relevant paper stayed locked.** *Postharvest Biol. Technol.* 213,
  112930 — "Evaluation of bruise volume quantification methods using finite element analysis for
  apple" — eluded all three sweeps. From title and topic tags it is method evaluation rather than
  inverse design, which is the weak negative this study relies on. Its actual contents are
  **unknown**, and it is the paper most likely to overturn the "no consensus criterion" reading.
- **A second locked paper could carry a number.** Van Zeebroeck's 2006 validation paper
  (*Postharvest Biol. Technol.* 41(2):85–91) is the one place a quantitative bruise-prediction error
  for that work would live. Closed at Elsevier, **absent from Lirias**, no abstract in OpenAlex or
  Crossref. The thesis abstract says only "with sufficient accuracy," which is where a number would
  be advertised if one existed — but it cannot be excluded.
- **Source-age asymmetry, and it cuts both ways.** The deepest evidence on calibration data and
  specimen variability is 1993–1999; the modelling evidence is 2019–2026. Do not read the old theses
  as statements about the current frontier.
  **And the 2019 MeBioS data-gap admission is no longer current** — that group published much of the
  missing data themselves in May 2026 (`10.48804/GNBFGU`). A three-month-old dataset from an active
  line is a warning that other parts of this chapter may be similarly overtaken. **Check the
  publication dates before relying on any "nobody has done X" claim here.**
