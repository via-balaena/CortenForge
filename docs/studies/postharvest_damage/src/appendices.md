# Claim ledger, parameters, glossary, sources, method

## Method

Three sequential adversarial research sweeps, run July 2026 via the `deep-research` workflow
harness. Each sweep: decompose into search angles → parallel web search per angle → URL-dedup and
fetch → extract falsifiable claims → **3-vote adversarial verification** (2 of 3 refutations kills
a claim) → synthesize with confidence ratings.

| | Sweep 1 | Sweep 2 | Sweep 3 |
|---|---|---|---|
| **Focus** | Loss accounting; causal coupling | FEM maturity; forward vs inverse | Frontier; calibration data; landscape |
| Sources fetched | 28 | 22 | 24 |
| Claims extracted | 108 | 80 | 77 |
| Claims verified | 25 | 25 | 25 |
| Confirmed | 14 | 17 | 19 |
| Refuted | 11 | 8 | 6 |
| Post-synthesis findings | 11 | 9 | 14 |

### ⚠ How to read the ledger below

Three things an earlier draft of this appendix got wrong, corrected here:

1. **The confirmed tables list 34 rows, not 50.** Those 34 are the **post-synthesis findings** —
   the harness merges semantically duplicate confirmed claims before reporting. The 50 confirmed
   claims are upstream of that merge and are not individually enumerable from the output. Nothing
   was dropped; the two numbers count different objects.
2. **Sweep 2 has no vote records at all.** Its findings carry only a `confidence` rating. An earlier
   draft printed "3-0" for sweep-2 rows, which **manufactured vote counts that do not exist**. The
   tables below use a single **Provenance** column that states what each source actually recorded.
3. **The refuted table is now complete at 25** — an earlier draft listed 19 and silently omitted
   six, three of which cut against this study's thesis.

### Method limitations, load-bearing

- **Verifier search budget exhausted (200/200) in sweeps 2 and 3.** The "no credible source disputes
  this" leg could not run. Verification rested on direct primary-text reading — adequate for claims
  purely descriptive of one paper's contents, **a real weakness for any field-level generalization.**
- **Paywalls shaped the evidence set.** Sweep 1's "the field is primitive" conclusion was a
  **reachability artifact**, corrected only when sweep 3 routed around publishers.
- **Selection bias toward what is open.** Recovered evidence skews Chinese, Polish and Iranian
  applied ag-engineering. "Practice" means *modal practice in the accessible literature*.
- **"Not found" vs "not searched."** Several questions (open-data repositories, ASABE standards,
  tooling, development-sector practice, patents) produced **zero verified claims** — recorded in the
  source caveats as *coverage gaps, not negative findings*. This distinction is marked throughout.

### Routing lessons (reusable)

- `mdpi-res.com` direct PDF deploys succeed where MDPI HTML 403s.
- PMC mirrors succeed for MDPI-published *Foods* articles.
- Lirias (KU Leuven) and WUR eDepot serve open post-prints, but return PDFs `WebFetch` **cannot
  render** — download plus local `pdftotext` is required.
- DOAJ v2, Crossref, OpenAlex and Semantic Scholar APIs work where publisher HTML does not.
- **Theses are the highest-yield open source** for parameter tables, raw scatter, and honest negative
  results that journal papers compress out.

## Claim ledger

### ✅ Confirmed — loss accounting *(sweep 1: vote records exist)*

| Claim | Provenance |
|---|---|
| FLI = 13.3% global harvest-to-retail, 2023; 13.0% in 2015; excludes retail/consumer | 3-0, high |
| F&V worst commodity group 25.4%; cereals & pulses 8.4%; roots/tubers/oil 12.3%; meat 14.0% | 3-0, high |
| Regional: sub-Saharan Africa 23.0%, LDCs 19.9%, SIDS 19.0%, N. America & Europe 10.0% | 3-0, high |
| FLI is a **modelled imputation**, not a measurement; FAO states results unreliable below aggregate | 3-0, high |
| **★ FAO's model contains no physical-cause axis**; mechanical damage not extractable | 3-0, high |
| Field-measurement review median whole-chain F&V loss 16.2% vs FAO-lineage 22–44% | 3-0, high |
| Regional F&V medians (N. Africa ~34% … N. America ~8%) — thin, selection-biased | **2-1, medium** |
| LSMS-ISA maize 1.4–5.9% vs FAO 8% / APHLIS 10–12% — third disagreeing regime | 3-0, high |
| Impact produces dose-dependent bruise growth **and** elevated ethylene (pomegranate, banana) | 3-0, high |
| Absorbed energy is a first-order correlate only; susceptibility superlinear | **2-1, medium** |
| Instrumented-fruit devices disagree at stiff impacts; cannot predict without per-commodity calibration | 3-0 / **2-1** merged |

### ✅ Confirmed — mechanics *(sweep 2: **no vote records exist**; confidence only)*

| Claim | Provenance |
|---|---|
| Bilinear elastic-plastic is the working constitutive standard as of 2024 | no vote; **high** |
| Parameters are point estimates, frequently borrowed rather than measured | no vote; **high** |
| Skin modelled separately is 4–8× stiffer; separate modelling not standard *(3 of 4 papers; field-wide version refuted 0-3)* | no vote; **medium** |
| De facto criterion = von Mises > bio-yield; no damage-accumulation law | no vote; **high** |
| Max-shear beats max-normal (Ghasemi 2015) — quasi-static, single cultivar, n=1 | no vote; **medium** |
| No verified consensus criterion; no verified systematic multi-criterion comparison | no vote; **medium** |
| **Validation quality is poor**: correlations, single scalars, or qualitative eyeballing | no vote; **high** |
| Headline numbers are fitted-to-same-data, circular, or regression intercepts | no vote; **high** |
| **All verified work is forward simulation**; no inverse design surfaced | no vote; **medium** ⚠ absence-of-evidence from ~7 papers |

### ✅ Confirmed — the gap *(sweep 3: vote records exist)*

| Claim | Provenance |
|---|---|
| **★ MeBioS frontier is real**: cellular DEM, turgor, visco-elastoplastic walls, rupture, debonding | 3-0, high |
| **★ That frontier is unvalidated on mechanics**; only geometric NRMSE <3%, partly circular | 3-0 / **2-1**, high |
| **★ First-party admission**: the required same-batch calibration data does not exist *(as of ~2019)* | 3-0, high |
| **★ Nicolai/Saeys/Opara review**: constitutive laws "have not received much attention" | 3-0, high |
| Damage-extent errors **do** exist (kiwifruit 9.63%, ginseng 0.4–2.2%, apple 2.51% abstract-only) | 3-0, high |
| **Held-out status not established** in any study | 3-0 / **2-1**, high |
| Validated studies uniformly bilinear elastic-plastic — including papers titled "multiscale" | 3-0, high |
| Some published FE work does not calibrate from its own data at all | 3-0, high |
| That work reports **no** quantitative damage validation — compares stress plots to other papers' | 3-0, high |
| Loading-paired damage data exists (Molema) but at **scalar energy resolution**, not force curves | 3-0, high |
| Calibration apparatus documented, but only means and SDs archived — no raw traces | 3-0, high |
| **★ ~7× specimen-to-specimen variability** in damage-onset energy *(as of Molema 1999)* | 3-0, **medium** |
| Massey/NZ tradition was empirical impact-correlation; boundary hyperbolic *(abstract only)* | 3-0, **medium** |
| **Negative**: the deepest produce-damage thesis contains **zero** FE or constitutive mechanics | 3-0, high |

### ❌ Refuted — complete, all 25

**Sweep 1 (11)**

| Refuted claim | Vote |
|---|---|
| FLI 13.2% in 2021 with F&V 31.2% and cereals 7.2% | 1-2 |
| Regional: sub-Saharan Africa 19.9–20%, N. America & Europe 9.2% | 0-3 |
| Losses concentrate in post-harvest handling and market stages (~15% median each) | 0-3 |
| FAO ~37% of food lost in sub-Saharan Africa; cereals 20.5%; **8% for handling and storage alone** | 0-3 |
| FAO figures methodologically weak and **likely upward-biased** | 0-3 |
| High-impact fruit lost 20.39% weight by day 28 | 0-3 |
| Ethylene spike peaking at day 21 in high-impact fruit | 0-3 |
| Late-storage respiration peaks interpreted by authors as microbial infestation | 1-2 |
| Bruised bananas reached 14.02 mg/kg/h respiration on day 2 | 0-3 |
| High-impact bananas unmeasurable by day 12 via ripening + **microbial infestation** | 0-3 |
| Potato black-spot damage threshold ~50–100 g, size-dependent | 0-3 |

> The fourth row matters for [Ch 1](10-loss.md): the refuted claim **included** the "8% for
> post-harvest handling and storage alone" clause that the chapter cites as fact. The evidence base
> is internally inconsistent here — a separate confirmed finding uses the same 8%. Treat that figure
> as contested.

**Sweep 2 (8)**

| Refuted claim | Vote |
|---|---|
| 2022 apple study modelled fruit as homogeneous *(as evidence of **field-wide** practice)* | 0-3 |
| Energy-threshold criterion (~100 N·mm) in active use as damage-onset criterion | 0-3 |
| FEM max-shear **location** coincided with observed subsurface bruise location | 0-3 |
| Constitutive model was linear viscoelastic with constant bulk modulus; skin not separate | 0-3 |
| Current practice uses bilinear elasto-plastic with **tissue-specific anisotropic** parameters | 0-3 |
| Damage onset predicted where equivalent stress reaches bio-yield **at ~5% strain** | 1-2 |
| Xu 2024 compares five named methods (BTM, EM, UVRM, EVM, FDM) with condition-dependent ranking | 0-3 |
| Xu 2024 sweeps six drop heights (600–2100 mm) and four contact materials | 0-3 |

> The last two were refuted **for unreachability**, not for being wrong — ScienceDirect 403,
> Semantic Scholar 429, OpenAlex null abstract. See
> [Ch 2](20-mechanics.md#what-the-field-actually-uses) for why that cannot be used as evidence of
> absence.

**Sweep 3 (6)**

| Refuted claim | Vote |
|---|---|
| MeBioS's mechanics line is multiscale cell-based modelling positioned as a building block | 1-2 |
| **Design optimization via these models is aspiration, not practice** — i.e. the frontier is forward analysis | 1-2 |
| **Contact-law knowledge is scarce; Van Zeebroeck's DEM bruise-depth work was qualitative-only** | 1-2 |
| Massey thesis pairs measured impact kinematics with bruise area per specimen | 1-2 |
| The ginseng validation was **definitively in-sample** | 0-3 |
| The MDPI RSM-surrogate paper performs no experimental validation of bruise predictions | 1-2 |

> **★ The two rows that cut against this study's thesis.**
>
> The second row is the closest thing in the ledger to a refutation of
> [Ch 3's "nobody does inverse design"](30-gap.md#nobody-does-inverse-design) — a claim that the
> frontier is forward-analysis-only was *not* accepted by verifiers.
>
> The third names **Van Zeebroeck's DEM bruise-depth work on 'Jonagold' apples** and refuses the
> characterization that it was qualitative-only — which cuts *toward* it being a genuine
> counter-example to "nobody bridges." An earlier draft omitted both rows.
>
> The fifth row cuts the other way and is why this book says "held-out status **not established**"
> rather than "established as in-sample."

## Recovered constitutive parameters

Usable for [Gate 0](40-program.md#gate-0--can-we-reproduce-a-published-number). **All are point
estimates with no reported scatter.** Verify against primary sources before relying on them.

### Apple — 'Golden Delicious' (whole-fruit homogeneous)

| Property | Value |
|---|---|
| Young's modulus E | 4.069 MPa |
| Poisson ratio ν | 0.32 |
| Density ρ | 845 kg/m³ |
| Crushable-foam yield | 0.3 and 3 *(units inferred MPa — **not stated in source**)* at uniaxial plastic strains 0 and 1 |
| Yield-stress ratios | 0.1 / 0.5 at plastic strain rates 0 and 1 |
| Geometry | Idealized 71.7 mm sphere, 163 g (from 300 apples) |

> E and ν taken from Komarnicki et al.; **only density was measured.**

### Kiwifruit — 'Xuxiang' (Xu et al. 2024) — the **Gate 0 target table**

| Tissue | E (MPa) | Tangent (MPa) | Bio-yield (MPa) | ν |
|---|---|---|---|---|
| Skin | 10.233 | — | 0.514 | 0.30 |
| Core | 4.499 | 1.381 | 1.306 | 0.30 |
| Flesh (axial) | 2.305 | 0.967 | 0.491 | 0.40 |
| Flesh (radial) | 1.346 | 0.642 | 0.292 | 0.40 |

> **⚠ Different study from the 9.63% result.** Zhu et al., *Foods* 2024, 13(21):3523 — which
> produced the 9.63% bruise-area error — reports skin E=10.69 / yield 0.53; pulp E=1.57 / tangent
> 0.92 / yield 0.26; core E=5.11 / tangent 0.83 / yield 1.12. **No Poisson ratios, no density.**
> Reproducing the 9.63% specifically requires Zhu's full set, which is not recovered here.
>
> **⚠ Quality flag:** one recovered kiwifruit table lists density in g/mm³ at physically implausible
> values (~1000× off). Treat table-level rigor with caution.

### Carrot — 'New Red Carrot' (22 ± 1 °C, 40–60% RH, ASAE S368.4 at 2.5 mm/min, n=10)

| Tissue | E (MPa) | Bio-yield (MPa) | Tangent (MPa) | ν |
|---|---|---|---|---|
| Flesh | 4.45 | 0.806 | 3.35 | 0.477 |
| Core | 4.63 | 0.821 | 3.42 | 0.479 |

**Prony series (2-term):**

| Tissue | g₁ | g₂ | τ₁ (s) | τ₂ (s) |
|---|---|---|---|---|
| Flesh | 0.218 | 0.233 | 1.684 | 26.31 |
| Core | 0.238 | 0.237 | 1.623 | 30.3 |

### Apple — 2022 compression study

ν = 0.35, E = 5.0 MPa — **both literature-cited, not measured.** No density, yield stress, or Prony
terms disclosed.

### ⚠ Not recovered: ginseng

The American ginseng study (2025) is methodologically the strongest reproduction target — its
parameters came from independent uniaxial compression on cortex and cambium — but **no numerical
values were recovered by any sweep.** Only the model class is known: "idealized bilinear isotropic
strain-hardening elastic-plastic" cortex, linear elastic cambium.

## Impact-energy reference values

| Source | Commodity | Energies |
|---|---|---|
| Pomegranate 'Helow' | Pendulum, 45° / 65° | 1.18 J / 2.29 J |
| Banana cv. Malindi | Pendulum, low/med/high | 0.074 / 0.160 / 0.273 J |
| Molema (potato) | Computer-controlled pendulum | 0.600 / 0.200 / 0.067 J |

Molema apparatus: impact-body curvatures flat / R30 / R15; delivered-energy accuracy −3 to +8%;
tubers absorb ~70–80% of delivered energy. Table 4.5 exemplars: B∞ @ 0.600 J = 286.9 (14.2) g,
2.85 (0.12) ms; B15 @ 0.067 J = 64.3 (3.56) g, 4.00 (0.13) ms.

Absorbed energy convention: $E_a = mg(h_1 - h_2)$, computed from rebound height.

Bruise susceptibility, banana, day 12 / 22 °C: **8.37×10⁴ → 1.04×10⁵ mm³/J** (medium → high impact);
max bruise volume 2.09×10⁴ mm³.

**Damage-onset scatter:** Parke (1963) — one sample damaged at 0.09 J, another absorbed 0.6 J
undamaged. Ghadge (1988) — onset spanning 0.4–0.7 J. *(Both as reported in Molema 1999.)*

## Glossary

Domain terms used without definition elsewhere in this book.

| Term | Meaning |
|---|---|
| **Bio-yield** | Stress at which plant tissue first shows irreversible cellular failure — the produce analogue of a yield stress. The de facto bruise threshold in this literature. |
| **Bruise susceptibility (BS)** | Bruise volume divided by impact energy (mm³/J). Note it is a *ratio*, so reporting it is not independent evidence that energy predicts bruising. |
| **Turgor** | Internal hydrostatic pressure of a plant cell pressing against its wall. What makes produce firm; drops as fruit ages. |
| **Climacteric** | Fruit that ripens via an autocatalytic ethylene burst after harvest (banana, apple, tomato). Non-climacteric fruit does not. Matters because impact effects superimpose on that burst. |
| **Poroelastic** | Constitutive model treating tissue as a porous solid saturated with fluid, so stress depends on fluid migration. Absent from all validated work here. |
| **Prony series** | Sum-of-exponentials representation of viscoelastic relaxation; $g_i$ are relative moduli, $\tau_i$ relaxation times. |
| **Crushable foam** | Abaqus material model for cellular solids that compact under pressure, with volumetric hardening. Used for whole-apple drop models. |
| **DEM** | Discrete Element Method — bodies modelled as interacting discrete particles rather than a continuum mesh. Used by MeBioS at cell scale. |
| **NRMSE** | Normalized root-mean-square error. |
| **Cortex / cambium** | Outer storage tissue and the thin dividing layer beneath it, in root vegetables. Modelled as separate materials in the ginseng study. |
| **BE-FEM** | Backward-Euler finite element method — implicit time integration, the `sim-soft` solver family. |
| **IFT** | Implicit Function Theorem — how `sim-soft` differentiates through an implicit solve. |
| **Return mapping** | Algorithm projecting a trial elastic stress back onto the yield surface in plasticity. The thing `sim-soft` has no place to put. |
| **HLPE** | High Level Panel of Experts (on Food Security and Nutrition) — source of FAO's socio-economic loss-cause taxonomy. |
| **APHLIS** | African Postharvest Losses Information System. |
| **LSMS-ISA** | Living Standards Measurement Study – Integrated Surveys on Agriculture (World Bank household survey programme). |
| **SIDS** | Small Island Developing States. |
| **IS-100 / ASABE S368** | Instrumented-sphere device family; and the ASABE standard for compression testing of food materials. Formerly ASAE S368. |
| **SWIR** | Short-wave infrared imaging, used for post-hoc bruise detection. |

## Key sources

### Frontier / constitutive

- Diels, Nicolai, Ramon, Wang & Smeets — "A discrete element approach to model rupturable 3D
  visco-elastoplastic tomato cells and their inter-cellular contact mechanics," *Soft Matter*
  (~2019). KU Leuven BIOSYST-MeBioS. **Open post-print via Lirias.** *Verified full text.*
- Zhejiang University with MeBioS (Saeys, Nicolai) and Stellenbosch (Opara) — "Mechanical damages and
  packaging methods along the fresh fruit supply chain: a review" (~2022–23). **Open post-print.**
  *Verified full text.*

### Validated FEM

- Zhu et al., *Foods* 2024, 13(21):3523 — kiwifruit drop-impact, 9.63% bruise area. *PMC full text.*
- American ginseng root pendulum impact, 2025, PMC12179082 — 0.4–2.2% bruise area, independent
  constitutive tests. Northeast Agricultural University, Harbin. *PMC full text.*
- Xu, Liu, Huang, Li — *Comput. Electron. Agric.* 2024, `10.1016/j.compag.2024.109024` — apple drop,
  "max error 2.51%." *Abstract only.*
- Xia et al. 2021, `10.3390/agriculture11060471` — carrot, Prony + von Mises criterion. *Full text.*
- Szyjewicz et al. 2021, `10.3390/app11167579` — apple drop, Crushable Foam. *Full text.*
- Soheilsarv et al. 2024, *J. Food Process Eng.* — persimmon pendulum. *Abstract level only.*
- 2022 apple compression, PMC9265796 — the no-validation case. *Full text via `mdpi-res.com`.*
- Ghasemi et al. 2015, *J. Agric. Machinery*, `10.22067/jam.v5i2.28262` — max-shear vs max-normal.
  **Open access.**

### Empirical tradition

- Molema, G.-J. — PhD thesis, Wageningen, defended 30 Nov 1999. **WUR eDepot, open.**
  *Verified full text (259,717 chars / 7,633 lines).*
- Pang, W. — PhD thesis, Massey University, 1993 — "Bruise Factor," hyperbolic damage boundary.
  *Abstract only, Massey Research Online.*
- Praeger, Surdilovic, Truppel, Herold & Geyer — "Comparison of Electronic Fruits for Impact
  Detection on a Laboratory Scale," *Sensors* 2013, 13(6):7140. Leibniz-ATB. *Full text.*

### Loss statistics

- FAO SDG Data Portal, indicator 12.3.1a — Food Loss Index. *Primary.*
- FAO methodology working paper (52 pp), `openknowledge.fao.org` — the no-cause-axis finding.
  *Verified full text.*
- Ambuko et al. 2025, *Frontiers in Horticulture*, `10.3389/fhort.2025.1529040` — 16.2% median.
- Kaminski & Christiaensen 2014, *Food Policy* — LSMS-ISA maize. Via World Bank.
- Ambler, de Brauw & Godlonton 2018 — detailed loss module, 1,200 Malawi households.
- Pomegranate: *Foods* 2023, 12(6):1122, PMC10048388. Banana: *Current Research in Food Science* 2023,
  `10.1016/j.crfs.2023.100640`, PMC10696235. Both *full text*.

### Named but never read

- **Van Zeebroeck** — DEM prediction of bruise depth in 'Jonagold' apples. Cited by the MeBioS-coauthored
  review as its bruise-prediction success story. **The strongest known threat to this study's central
  finding, and nobody read it.**
- **Xu, Liu, Wang, Guan, Tang, Li (2024)** — "Evaluation of bruise volume quantification methods using
  finite element analysis for apple," *Postharvest Biol. Technol.* 213,
  `10.1016/j.postharvbio.2024.112930`. Confirmed to exist via Crossref and OpenAlex (~10 citations).
  **ScienceDirect 403 ×3.**
- **Dintwa (2011)** — earlier MeBioS multiscale cell-based work, referenced but not retrieved.
- **Mohsenin**, *Physical Properties of Plant and Animal Materials* — cited throughout the field;
  current standard-reference status **never verified**.

## Open questions

1. **Do the open-data repositories actually lack produce force-displacement data?** Zenodo, Dryad,
   Figshare, Mendeley, 4TU were **never successfully searched**. Gates 3 and 5 both assume they do.
   Cheapest unattempted action in the program.
2. **What did Van Zeebroeck's DEM work actually validate**, and does it bridge sophisticated ↔
   validated?
3. **Does the MeBioS ~2019 calibration-data gap persist?** Has anyone published the same-batch
   dataset — tissue tension/compression with failure-propagation imaging, plus single-cell mechanics,
   pressure-probe, and 3D microstructure?
4. **Is the max-shear-over-max-normal result reproducible under impact loading** and in other
   cultivars? Currently one paper, one cultivar, quasi-static, three deformation levels.
5. **What does the locked paper conclude** about which bruise-volume quantification method wins?
   CNKI, Chinese institutional repositories, and a thesis version remain untried.
6. **Does skin modelling materially change bruise volume**, or only contact pressure and area? No
   study ran the ablation → [Gate 1](40-program.md#gate-1--the-ablation-nobody-ran).
7. **What are the coefficients of variation** for fruit mechanical properties, and has anyone run
   stochastic or Monte Carlo produce FEM? Every parameter recovered here is a point estimate.
8. **Is FE licence cost actually a binding constraint** for low-income-country researchers and
   development-sector actors? Zero evidence after three sweeps — and it now gates a **build-vs-buy**
   decision, not just [Gate 5](40-program.md#gate-5--open-the-tooling-deliverable-not-a-gate).
9. **Is FAO's Food Loss and Waste Database minable** for a mechanical-damage share? It records
   narrative causes across 700+ publications and 37,000+ data points.
10. **Are the unreached groups already doing this?** Wageningen (modern), UC Davis, Cranfield,
    Washington State (Karkee), Michigan State, USDA ARS.
11. **Does the `sim-soft` adjoint survive path-dependent plastic state?** Answerable in-house, and it
    blocks [Gate 4](40-program.md#gate-4--inverse-design).
12. **Does ASABE S368 mandate force-deformation curve reporting**, and do benchmark datasets exist?
