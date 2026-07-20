# Claim ledger, parameters, sources, method

## Method

Three sequential adversarial research sweeps, run July 2026 via the `deep-research` workflow
harness. Each sweep: decompose the question into search angles → parallel web search per angle →
URL-dedup and fetch sources → extract falsifiable claims → **3-vote adversarial verification**
(a claim needed 2 of 3 verifiers to *refute* it to be killed) → synthesize with confidence
ratings and vote records.

| | Sweep 1 | Sweep 2 | Sweep 3 |
|---|---|---|---|
| **Focus** | Loss accounting; causal coupling | FEM maturity; forward vs inverse | Frontier; calibration data; landscape |
| Agents | 111 | 105 | 107 |
| Sources fetched | 28 | 22 | 24 |
| Claims extracted | 108 | 80 | 77 |
| Claims verified | 25 | 25 | 25 |
| Confirmed | 14 | 17 | 19 |
| Refuted | 11 | 8 | 6 |
| Subagent tokens | 3.10 M | 3.10 M | 3.27 M |

**Totals: 323 agents, 74 sources, 265 claims extracted, 75 verified, 50 confirmed, 25 refuted,
~9.5 M subagent tokens.**

### Method limitations, load-bearing

- **Verifier search budget was exhausted (200/200) in sweeps 2 and 3.** The "no credible source
  disputes this" leg of verification could not run. Verification rested on direct primary-text
  reading — adequate for claims that are purely descriptive of one paper's contents (which other
  sources cannot contradict), **a real weakness for any field-level generalization.**
- **Paywalls shaped the evidence set.** ScienceDirect/Elsevier, Wiley, and MDPI HTML returned 403
  consistently. Sweep 1's conclusion that "the field is primitive" was substantially a
  **reachability artifact**, corrected only when sweep 3 routed around publishers.
- **Selection bias toward what is open.** The recovered evidence skews Chinese, Polish and Iranian
  applied ag-engineering. Statements about "practice" mean *modal practice in the accessible
  literature*.

### Routing lessons (reusable)

Recorded because they cost two sweeps to learn:

- `mdpi-res.com` direct PDF deploys succeed where MDPI HTML 403s.
- PMC mirrors succeed for MDPI-published *Foods* articles.
- Lirias (KU Leuven) and WUR eDepot serve open post-prints, but return PDFs that `WebFetch`
  **cannot render** — download plus local `pdftotext` is required.
- DOAJ API v2, Crossref, OpenAlex and Semantic Scholar APIs work where publisher HTML does not.
- **Theses are the highest-yield open source** for parameter tables, raw scatter, and honest
  negative results that journal papers compress out.

## Claim ledger

### ✅ Confirmed — loss accounting

| Claim | Vote |
|---|---|
| FLI = 13.3% global harvest-to-retail, 2023; 13.0% in 2015; excludes retail/consumer | 3-0 |
| F&V worst commodity group at 25.4%; cereals & pulses 8.4%; roots/tubers/oil 12.3%; meat 14.0% | 3-0 |
| Regional: sub-Saharan Africa 23.0%, LDCs 19.9%, SIDS 19.0%, N. America & Europe 10.0% | 3-0 |
| FLI is a **modelled imputation**, not a measurement; FAO states results unreliable below aggregate | 3-0 |
| **★ FAO's model contains no physical-cause axis**; mechanical damage not extractable | 3-0 |
| Field-measurement review median whole-chain F&V loss 16.2% vs FAO-lineage 22–44% | 3-0 |
| Regional F&V medians (N. Africa ~34% … N. America ~8%) — thin, selection-biased | 2-1 |
| LSMS-ISA maize 1.4–5.9% vs FAO 8% / APHLIS 10–12% — third disagreeing regime | 3-0 |

### ✅ Confirmed — mechanics

| Claim | Vote |
|---|---|
| Impact produces dose-dependent bruise growth **and** elevated ethylene (pomegranate, banana) | 3-0 |
| Absorbed energy is a first-order correlate only; susceptibility superlinear (8.37e4 → 1.04e5 mm³/J) | 2-1 |
| Instrumented-fruit devices disagree with each other and with real produce at stiff impacts | 2-1 |
| Electronic fruits cannot predict bruising without commodity-specific calibration | 3-0 |
| Bilinear elastic-plastic is the working constitutive standard as of 2024 | 3-0 |
| Parameters are point estimates, frequently borrowed rather than measured | 3-0 |
| Skin modelled separately is 4–8× stiffer than flesh; separate modelling is **not** standard | medium |
| De facto criterion = von Mises > bio-yield; **no damage-accumulation law anywhere** | 3-0 |
| Max-shear beats max-normal (Ghasemi 2015) — quasi-static, single cultivar | medium |
| **Validation quality is poor**: correlation coefficients, single scalars, or qualitative eyeballing | 3-0 |
| Headline numbers are fitted-to-same-data, circular, or regression intercepts | 3-0 |
| Some published FE work does not calibrate from its own data at all | 3-0 |
| That work reports **no** quantitative damage validation — compares stress plots to other papers' | 3-0 |

### ✅ Confirmed — the gap

| Claim | Vote |
|---|---|
| **★ MeBioS frontier is real**: cellular DEM, turgor, visco-elastoplastic walls, rupture, debonding | 3-0 |
| **★ That frontier is unvalidated on mechanics**; only geometric NRMSE <3%, partly circular | 3-0 / 2-1 |
| **★ First-party admission**: the required same-batch calibration data does not exist | 3-0 |
| **★ Nicolai/Saeys/Opara review**: constitutive laws "have not received much attention" | 3-0 |
| A3 partially overturned: quantitative damage-extent errors **do** exist (9.63%, 0.4–2.2%, 2.51%) | 3-0 |
| **A3 survives in strong form**: held-out status not established in any study | 3-0 / 2-1 |
| Validated studies are uniformly bilinear elastic-plastic — **including papers titled "multiscale"** | 3-0 |
| Loading-paired damage data exists (Molema) but at **scalar energy resolution**, not force curves | 3-0 |
| Calibration apparatus documented, but only means and SDs archived — no raw traces | 3-0 |
| **★ ~7× within-lot variability** in damage-onset energy; distributional treatment required | medium |
| Massey/NZ tradition was empirical impact-correlation, not simulation; boundary hyperbolic | medium |
| **Negative**: the deepest produce-damage thesis contains **zero** FE or constitutive mechanics | 3-0 |
| **All verified work is forward simulation**; no inverse design surfaced | medium |

### ❌ Refuted — do not let these leak back in

| Refuted claim | Vote |
|---|---|
| Impact damage leads to a microbial-spoilage endpoint misattributed away from mechanical causes | 0-3 |
| High-impact fruit lost 20.39% weight by day 28 (post-impact moisture loss) | 0-3 |
| Ethylene spike peaking at day 21 in high-impact fruit | 0-3 |
| Late-storage respiration peaks interpreted by authors as microbial infestation | 1-2 |
| Bruised bananas reached 14.02 mg/kg/h respiration on day 2 | 0-3 |
| High-impact bananas became unmeasurable by day 12 via ripening + microbial infestation | 0-3 |
| Potato black-spot damage threshold ~50–100 g, size-dependent | 0-3 |
| FAO figures are methodologically weak and **likely upward-biased** | 0-3 |
| FAO ~37% of food lost in sub-Saharan Africa; cereals 20.5% | 0-3 |
| Losses concentrate in post-harvest handling and market stages (~15% median each) | 0-3 |
| FLI 13.2% in 2021 with F&V 31.2% and cereals 7.2% | 1-2 |
| Regional: sub-Saharan Africa 19.9–20%, N. America & Europe 9.2% | 0-3 |
| Energy-threshold criterion (~100 N·mm) still in active use as damage-onset criterion | 0-3 |
| FEM max-shear **location** coincided with observed subsurface bruise location | 0-3 |
| Xu 2024 compares five named methods (BTM, EM, UVRM, EVM, FDM) with condition-dependent ranking | 0-3 |
| Xu 2024 sweeps six drop heights (600–2100 mm) and four contact materials | 0-3 |
| The ginseng validation was **definitively in-sample** | 0-3 |
| 2022 apple compression study modelled fruit as single homogeneous material *(as field-wide evidence)* | 0-3 |
| Per-specimen kinematics-to-bruise-area pairing in Pang 1993 constitutes calibration-grade data | 1-2 |

> **★ Note the last two rows.** Refutation cut in *both* directions — against claims that would
> have flattered the "field is primitive" thesis as well as against claims that would have
> supported the coupling hypothesis. The verifier also killed an overreach asserting the ginseng
> validation was definitively in-sample, forcing the weaker and correct
> **"held-out status not established."**

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

### Kiwifruit — 'Xuxiang' (tissue-specific)

| Tissue | E (MPa) | Tangent (MPa) | Bio-yield (MPa) | ν |
|---|---|---|---|---|
| Skin | 10.233 | — | 0.514 | 0.30 |
| Core | 4.499 | 1.381 | 1.306 | 0.30 |
| Flesh (axial) | 2.305 | 0.967 | 0.491 | 0.40 |
| Flesh (radial) | 1.346 | 0.642 | 0.292 | 0.40 |

> A second kiwifruit study (*Foods* 2024, the 9.63% result) reports skin E=10.69 / yield 0.53;
> pulp E=1.57 / tangent 0.92 / yield 0.26; core E=5.11 / tangent 0.83 / yield 1.12.
>
> **⚠ Quality flag:** one recovered kiwifruit table lists density in g/mm³ at physically
> implausible values (~1000× off). Treat table-level rigor with caution.

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

ν = 0.35, E = 5.0 MPa — **both literature-cited, not measured.** No density, yield stress, or
Prony terms disclosed.

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

**Damage-onset scatter:** Parke (1963) — one sample damaged at 0.09 J, another absorbed 0.6 J
undamaged. Ghadge (1988) — onset spanning 0.4–0.7 J. *(As of Molema 1999.)*

## Key sources

### Frontier / constitutive

- Diels, Nicolai, Ramon, Wang & Smeets — "A discrete element approach to model rupturable 3D
  visco-elastoplastic tomato cells and their inter-cellular contact mechanics," *Soft Matter*
  (~2019). KU Leuven BIOSYST-MeBioS. **Open post-print via Lirias.** *Verified full text.*
- Zhejiang University with MeBioS (Saeys, Nicolai) and Stellenbosch (Opara) — "Mechanical damages
  and packaging methods along the fresh fruit supply chain: a review" (~2022–23). **Open
  post-print.** *Verified full text.*

### Validated FEM

- Zhu et al., *Foods* 2024, 13(21):3523 — kiwifruit drop-impact, 9.63% bruise area. *PMC full text.*
- American ginseng root pendulum impact, 2025, PMC12179082 — 0.4–2.2% bruise area, independent
  constitutive tests. Northeast Agricultural University, Harbin. *PMC full text.*
- Xu, Liu, Huang, Li — *Comput. Electron. Agric.* 2024, `10.1016/j.compag.2024.109024` — apple drop,
  "max error 2.51%." *Abstract only.*
- Xia et al. 2021, `10.3390/agriculture11060471` — carrot, Prony + von Mises criterion. *Full text.*
- Szyjewicz et al. 2021, `10.3390/app11167579` — apple drop, Crushable Foam. *Full text.*
- Soheilsarv et al. 2024, *J. Food Process Eng.* — persimmon pendulum. 
- 2022 apple compression, PMC9265796 — the no-validation case. *Full text via `mdpi-res.com`.*
- Ghasemi et al. 2015, *J. Agric. Machinery*, `10.22067/jam.v5i2.28262` — max-shear vs max-normal.
  **Open access.**

### Empirical tradition

- Molema, G.-J. — PhD thesis, Wageningen, defended 30 Nov 1999. **WUR eDepot, open.**
  *Verified full text (259,717 chars).*
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
- Pomegranate: *Foods* 2023, 12(6):1122, PMC10048388. Banana: *Current Research in Food Science*
  2023, `10.1016/j.crfs.2023.100640`, PMC10696235. Both *full text*.

### Never reached

- **Xu, Liu, Wang, Guan, Tang, Li (2024)** — "Evaluation of bruise volume quantification methods
  using finite element analysis for apple (*Malus pumila* Mill.)," *Postharvest Biol. Technol.* 213,
  `10.1016/j.postharvbio.2024.112930`. Confirmed to exist via Crossref and OpenAlex (~10 citations).
  **ScienceDirect 403 ×3, Semantic Scholar 429, OpenAlex null abstract.** The single most
  decision-relevant unread source in this study.
- Mohsenin, *Physical Properties of Plant and Animal Materials* — cited throughout the field
  (Mohsenin 1986 appears in Molema's references); its current standard-reference status was
  **never verified**.

## Open questions

Carried from all three sweeps. See also
[what we still do not know](50-verdict.md#what-we-still-do-not-know).

1. **Does the MeBioS 2019 calibration-data gap persist?** Has anyone published the same-batch
   dataset — tissue tension/compression with failure-propagation imaging, plus single-cell
   mechanics, pressure-probe, and 3D microstructure? *This single question decides whether
   independent constitutive calibration is currently possible at all.*
2. **What does the locked paper conclude** about which bruise-volume quantification method wins?
   CNKI, Chinese institutional repositories, the authors' lab page, and a thesis version all
   remain untried.
3. **Does skin modelling materially change bruise volume**, or only contact pressure and area?
   No study ran the ablation. → [Gate 1](40-program.md#gate-1--the-ablation-nobody-ran).
4. **Is the max-shear-over-max-normal result reproducible under impact loading** and in other
   cultivars? Currently one paper, one cultivar, quasi-static, three deformation levels.
5. **Does any force-time dataset with paired damage outcomes exist** in accessible form?
   Unresolved after three attempts.
6. **What are the coefficients of variation** for fruit mechanical properties, and has anyone run
   stochastic or Monte Carlo produce FEM? Every parameter recovered here is a point estimate.
7. **Do ASABE S368 or any impact/bruise standard mandate force-deformation curve reporting**, and
   do benchmark datasets exist? Entirely unexamined.
8. **Is FE license cost actually a binding constraint** for low-income-country researchers and
   development-sector actors? Zero evidence after three sweeps —
   [Gate 5](40-program.md#gate-5--open-the-tooling) rests on this.
9. **Is FAO's Food Loss and Waste Database minable** for a mechanical-damage share? It records
   narrative causes across 700+ publications and 37,000+ data points, but as a repository rather
   than a model input. What are its coding conventions and biases?
10. **Are the unreached groups already doing this?** Wageningen (modern), UC Davis, Cranfield,
    Washington State (Karkee), Michigan State, USDA ARS.
