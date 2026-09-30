# Soft-contact architecture

**Status:** the plan, 2026-09-24, amended as the build goes. Build step 1 is merged (#965). Step 2 is designed in
§16; 2a (#968) and 2b's G2 (#969) are merged, and 2b's remaining runs, with the material damping they led to, are
§16p (#970), and 2c, K6, is §16q (#971). 2d, the product's budget and the stop rule, is §16r (#972). K5,
with D1's readings diagnosed and replaced, is §16s (#973). Fit plan U3, why the rigid path asks for room and the
path step 7 runs, is §16t. Step 6's obstacle bake is §16u, the wall's canal surface §16v, and the lowering §16w.
Step 7's first run is §16x (#978), the element collapsing at its seated tip §16y (#979), and D1's element size §16z
(#980). The GPU steps are §17, step 3 first (§17a).
- **Research:** §1–§10.
- **Code architecture and the crate layout:** §11–§14.
- **The first experiment and its kill criteria:** §15.
- **Build step 2's design, and what its PRs measured:** §16 (§16m–§16s); U3, §16t; step 6's bake, §16u; the wall's
  canal surface, §16v; the lowering, §16w; step 7's first run, §16x; the element collapsing at the seated tip, §16y;
  D1's element size, §16z.
- **The GPU steps, 3–5:** §17.

The code architecture, crate layout and first experiment were checked by cold review, against criteria
written beforehand (§14e, §15i). The research sections were not. Jon's direction:
- *"take a step back architecturally and think this out and be extremely honest about things before we
  pour a bunch of time into stuff"*;
- *"the planning phase is crucial. we must exhaust the planning/architecting phase."*

**How the numbers are sourced.**
- **✓** marks a number I re-read at its source, derived myself, or the repo measured (with its referent).
- Unmarked claims come from ten research surveys, which are not in the repo. Their links are given where
  they were kept. They were not re-checked.
- The PolyFEM numbers were measured on a build that has since been deleted. Their record is the fit
  plan's Speed section.
- A number with neither is arithmetic, and is labelled as such.
- ⚠ **The session's web-search allowance (200) ran out.** Several surveys were cut short, and their gaps
  are listed in §10.

---

## 1. What we need

- **The fit test.** A scanned body slides into a silicone sleeve whose cavity is ~5 mm smaller. It
  reports:
  - push force along the path;
  - contact pressure;
  - stretch.

  Target: at most **5 minutes per run on an Apple M4 Pro** (fit plan D4). Both the visuals and the numbers must be
  realistic.
- **The class, at CortenForge's peak** (Jon, 2026-09-24):
  - surgical insertion (needle, catheter, endoscope);
  - footwear and garment fit;
  - seal and O-ring fitting;
  - soft-robot grasping.
- **The platform.** wgpu on Metal and Vulkan (fit plan D6), which means **f32 on the GPU**: Metal has no
  f64. An outside physics engine comes inside only if it is Rust (fit plan D7).
- **The material is uncertain, and that uncertainty is the input** (Jon: *"figure out how to properly
  model that variety/operate within the ends of the variety spectrums"*).

## 2. What the fit test really is

**A comparison instrument, judged against published limits.**
- Its verdict compares readings with limits. Jon, 2026-09-24, *"i dont really want to do the manual
  calibrations as im not a lab. we can source calibration/measurements from the internet"*, so those
  limits come from **published measurements** (§8), not from casts.
- **That changes what bias costs.** Nothing per-design now absorbs a systematic bias. The absolute
  accuracy of both the solver and the material data matters, and so does **validation against published
  experiments** (§7).
- **The honest answer is a range** across the material spread (§5), not one number.
- **Noise is still the enemy.** A readout that jumps with the mesh cannot be trusted at all. The measured
  example is PolyFEM's single rim node carrying 42× the next ✓.
- **What matters most:** the solver must rank designs consistently and respond **smoothly** to the inset,
  material and lip.
- **What to report:**
  - **ranges** across material and friction corners;
  - pressure as area-weighted percentiles, not a single-node peak *(2026-09-26, §16s: the seated reading is the
    most-loaded 1 cm² patch, with the percentile shown beside it)*;
  - push force split into its geometric and friction parts (§6).

## 3. Why the current approach failed — measured

- **We built the most demanding formulation:**
  - quasi-static equilibrium at every step;
  - guaranteed non-penetration (IPC);
  - tight tolerances;
  - a fine quadratic mesh (Tet10);
  - a CPU direct solver.
- **A mature library of exactly that kind** (PolyFEM, built and validated here, since deleted) took about
  **7 minutes per load step** on `base_mold`, one step measured, at first contact ✓
  (`docs/INSERTION_SIM_FIT_TEST_PLAN.md`, Speed):
  - ~30 Newton iterations at first contact ✓;
  - ~15 s per iteration, for one sparse factorization ✓;
  - changing the step size or the mesh made it worse ✓ (fit plan, Speed).
- **So it is about 200× over budget** (arithmetic: 5 min / (~130 steps × ~30 iterations) ≈ 77 ms per
  iteration, against 15 s).
- **The "real-time" squishy-cube demos are replays.** `example-integration-two-way-striker-viewer`
  computes 220 steps in **21.9 s** ✓, then plays them back.

## 4. How practitioners do it

| Domain | Method | Speed | Validation |
|---|---|---|---|
| **Seal mounting** | LS-DYNA **explicit**, mass scaling, penalty contact. *"the implicit solver is generally very difficult to successfully handle the analysis. However, for explicit solver like LS-DYNA, this problem can be trivial."* ✓ | not reported | *"… very well"* against tests ✓ ([Shi](https://lsdyna.ansys.com/wp-content/uploads/attachments/Session_4-4.pdf)) |
| **Compression stockings** | Abaqus/**Explicit**, S4 **shells** on a rigid leg | not reported | within 20 % of measured pressure ([PMC12920784](https://pmc.ncbi.nlm.nih.gov/articles/PMC12920784/)) |
| **Prosthetic socket donning** (closest analogue) | Abaqus **implicit, static**, 40 272 quadratic tets, displacement-driven from 20 mm ✓ | **~30 min per run** on a CPU ✓ | a Kriging surrogate trained on 150 runs answers in **1.6 ms** (NRMSE 4 %), checked against the FE runs, not experiment ✓ ([Steer 2019](https://pmc.ncbi.nlm.nih.gov/articles/PMC7423807/)) |
| **Surgical, GPU** (TLED: Taylor; Miller, Joldes, Wittek) | **explicit total-Lagrangian, f32 GPU**; dynamic relaxation (DR) for quasi-static | a 125 292-element brain in **19.95 s** on a 2007 Tesla C870 (543 s on CPU) ✓; 1 250–3 300 DR steps per equilibrium ✓ ([PMC3003932](https://pmc.ncbi.nlm.nih.gov/articles/PMC3003932/)) | 2.5 % reaction-force error against Abaqus **without contact**; displacement-only against experiment ([PMC2783832](https://pmc.ncbi.nlm.nih.gov/articles/PMC2783832/)) |
| **Surgical, real-time** (SOFA) | corotational **linear** tets, one linearization per step, LCP contact | 18–40 FPS on the surgical cases (7 680 and 8 596 tets) | 1.7 % mean stress error against Abaqus ([PMC6485523](https://pmc.ncbi.nlm.nih.gov/articles/PMC6485523/)) |
| **Fast GPU research** (VBD, AVBD, GPU IPC, ppf) | vertex Gauss–Seidel, or Newton–PCG with a barrier | seconds per dynamic frame at 10⁵ tets | the survey found **none** that validated contact force or pressure against engineering FEM or experiment |

**What it adds up to.**
- **TLED is the precedent for the architecture recommended here.**
  - It runs explicit in f32 on the GPU. Joldes 2010: *"performing the operations in single precision does
    not have a high impact on convergence for the accuracy usually required in our simulations"* ✓
    ([PMC3003932](https://pmc.ncbi.nlm.nih.gov/articles/PMC3003932/)).
  - Per-element force buffers gathered at the nodes need **no atomics** ([NiftySim](https://pmc.ncbi.nlm.nih.gov/articles/PMC4488488/)).
  - The average-nodal-pressure (ANP) tetrahedron handles near-incompressibility.
- **What TLED does not cover, which is therefore what we add** (Jon, 2026-09-24, *"these would be what
  we would need to add to it"*):
  - **friction**, since every dynamic-relaxation contact the surveys found is frictionless;
  - **silicone-level incompressibility**, since nothing was validated above ν = 0.49;
  - **force validated with contact.**

  Its code is not maintained either: NiftySim is CUDA and unchanged since 2017. We build our own.
- **The survey found no one simulating sleeve fit** (no paper, vendor or patent).

## 5. The material, and what it forces

### 5a. Stiffness varies across labs, and that sets the accuracy worth paying for

Stress at 100 % strain across sources, from raw data (round 1:
[Marechal](https://github.com/LucMarechal/Soft-Robotics-Materials-Database),
[Roels](https://zenodo.org/records/14983287), datasheets):

| Material | Range | Spread |
|---|---|---|
| Ecoflex 00-30 | 29.7–101 kPa | 3.4× |
| Dragon Skin 10 | 113–152 kPa | 1.3× (two sources only) |
| Dragon Skin 20 / 30 | 318–593 / 437–833 kPa | 1.9× |
| Within one lab (Ecoflex 00-20, n = 10) | 74–100 kPa | ±15 % |

- **For Ecoflex 00-30, the datasheet σ₁₀₀ is ~2.3× Marechal's measured value** ✓
  (`sim/L0/soft/src/material/silicone_table.rs:103-107`).
- **The product blend (Dragon Skin 10 + Slacker) has no published modulus.** It is rated only by Shore
  hardness. Shore-to-modulus conversions are unvalidated this soft
  ([Gent](https://en.wikipedia.org/wiki/Shore_durometer)). No home lab will measure it (§9 decision 5), so
  the nearest measured grade is used, with a wider range, and flagged (§8).
- **Silicone softens after its first stretch** (the Mullins effect), and about 60 % of that recovers within
  a month ([Liao 2020](https://cronfa.swan.ac.uk/Record/cronfa53571)). A tested cast carries its own
  history.

### 5b. Near-incompressibility — our biggest modelling gap

- **Our model is far more compressible than silicone.** It uses λ = 4μ: Poisson's ratio ν = 0.4,
  K/μ = 4.7.
- **What silicone actually is:**
  - Measured Sylgard 184: **ν = 0.4950 ± 0.0010** ([Müller 2019](https://www.ebi.ac.uk/europepmc/webservices/rest/search?query=DOI:10.1039/c8sm02105h&format=json&resultType=core)).
  - Abaqus puts unfilled elastomers at K/μ 1 000–10 000 ([Abaqus](https://abaqus-docs.mit.edu/2017/English/SIMACAEMATRefMap/simamat-c-hyperelastic.htm)).
  - Every explicit code's default is far above ours: Abaqus/Explicit K/μ = 20 (ν 0.475); LS-DYNA
    recommends ν 0.49–0.5; Radioss defaults to 0.495.
- **What it costs.** Arithmetic ✓: explicit steps scale with √(λ+2μ). Against ν = 0.4:

  | ν | 0.49 | 0.495 | 0.499 |
  |---|---|---|---|
  | steps × | 2.92 | 4.10 | 9.14 |

- **How much it matters depends on confinement.** From the tube oracle (§15,
  `docs/soft_contact/thick_tube_reference.py`), p/μ at λ_a 1.1, B/A 2 ✓. The first two columns have
  free ends; the last has all axial motion held:

  | ν | Free wall | Cased, material escapes axially | Cased, no escape |
  |---|---|---|---|
  | 0.49 | 0.1235 | 0.446 | 4.14 |
  | 0.495 | 0.1237 | 0.449 | 8.04 |
  | 0.4995 | 0.1238 | 0.452 | 78.3 |

  - With a free wall, or with axial escape, ν barely matters. Only full confinement scales with K.
  - Abaqus says of confined rubber in explicit: *"it may not be feasible to obtain accurate results"*.
- ⛔ **The current simulation (`cf-sim-research`) pins the sleeve's entire outer skin, which confines
  it.**
  - **Decided (Jon, 2026-09-24): "probably an option for either".** The outer boundary becomes a design
    option: free, cased, or bonded.
  - Which regime each application sits in is in §15h.
- **Element choice.** In explicit codes, the plain 10-node tet:
  - lumps badly (row-sum gives corner masses of −1/20 of the element's ✓: ∫N_corner dV = 2∫L² − ∫L = V/5 − V/4 = −V/20);
  - gives zero corner loads under face pressure (*"Second-order tetrahedra are not suitable for the
    analysis of contact problems"*,
    [Abaqus](https://abaqus-docs.mit.edu/2017/English/SIMACAETHERefMap/simathe-c-tritetwedge.htm)).

  Practice uses:
  - the ANP linear tet (LS-DYNA ELFORM 13, up to +25 % cost);
  - or reduced-integration hexahedra with hourglass control;
  - or modified 10-node tets.

  **This means leaving our Tet10 investment** for the explicit solver.

### 5c. Friction probably dominates push force, and it is the least known input

*Re-sourced 2026-09-27 by a research round that kept each source's text (fit plan U2). What it changed from the
round-2 table is listed below the new one.*

Silicone on the forearm's skin unless stated. *Onset* is the peak as sliding starts; *sliding* is the value after it.

| State | μ | Source | Conditions |
|---|---|---|---|
| Dry, onset | 0.94 and 1.14 | [Masen 2020](https://doi.org/10.1371/journal.pone.0239363) (one subject; read from its Fig. 3), [Yap 2021](https://doi.org/10.1038/s41598-021-91119-0) (7 subjects; printed on its Fig. 2b) | 20 and 14 kPa |
| Dry, sliding | 0.61 ± 0.21, over six sites and ten subjects | [Zhang & Mak 1999](https://doi.org/10.3109/03093649909071625) | a liner silicone of unstated grade |
| Dry, held | a tacky silicone pad held a shear of twice its load for 30 min without slipping | [Klaassen 2018](https://doi.org/10.3990/1.9789036546072) | 2.4 kPa; the pad was chosen not to slide |
| Water only | none on skin found. On analogs: 0.84 (a steel ball on water-wet PDMS); 1.09 onset and 1.47 sliding (a tacky silicone dressing wetted with saline, on a silicone skin simulant) | [Lee 2022](https://doi.org/10.3390/ma15093262), [Gefen 2026](https://doi.org/10.1111/iwj.70860) | |
| Water-based gel, fresh | 0.18 onset; 0.104–0.145 sliding over seven gels (an endoscope's fluoropolymer tube on skin post mortem) | Masen 2020 (read from its Fig. 3); [Watanabe 2024](https://doi.org/10.1038/s44172-024-00177-5) | thin water-based liquids read 0.40–0.59 sliding in Watanabe 2024 |
| Water-based gel, later | 0.96 onset at 5 min: back at the dry value | Masen 2020 (one subject; about 2 mg/cm², applied once) | |
| Silicone lubricant | 0.30 onset on application, 1.21 at 20 min | Masen 2020 (read from its Fig. 3) | |

- **Onset and sliding differ.** In one trace of each, sliding ran at about 0.8 of the onset peak dry; lubricated it
  fell from about 0.8 to 0.55 of it over the trace (Yap 2021, read from its Fig. 2a). The solver's contact takes one
  μ_f; whether a pairing gives it the onset or the sliding value is not settled.

**What changed from the round-2 table** (dry 0.4–1.0, nominal 0.6; water 0.15–2.0; water-based 0.05–0.5, 0.15
fresh and 0.3 depleted; silicone lubricant 0.05–0.3, unmeasured):
- Dry friction reaches 0.9–1.1 at onset. The old nominal 0.6 matches only the sliding value.
- In the one time course of silicone on skin, a water-based gel was back at the dry value within 5 min, not at 0.3.
- Silicone lubricant has that time course too, above. Masen 2020's text gives the two lubricants' late rises the
  other way round from its figure; either way both are at or above dry within 5–20 min.
- The old water-based row's source ([Cooper 2018](https://pmc.ncbi.nlm.nih.gov/articles/PMC6227966/)) is latex on a
  skin-like polyurethane at 78 kPa, and its two numbers come from two tests: 0.159 is one articulation, and "greater
  than 0.30" is seconds 600–900 of a 1000-articulation run, "a roughly 40% increase".

- **Decided (Jon, 2026-09-24): "different things can have different lubricants, so plan for that".**
  Friction is a **library of contact pairings**, each with a sourced range: surface × surface ×
  lubricant, with fresh and depleted states. It extends beyond the sleeve (for example sock on skin,
  leather on sock, oiled rubber on steel, a hydrophilic catheter on mucosa). The fit test picks the
  pairing, and reports the range.
- **Friction's rise as a lubricant goes differs between sources:** ×1.4 over 15 min (Cooper 2018) and ×5 within
  5 min (Masen 2020). No study on skin that was found varied the dose or reapplied it.
- **Pressure and speed.** The silicone-on-skin numbers above are at 2.4–20 kPa and up to 18 mm/s (Zhang & Mak 1999
  states 80 kPa, which its own load and probe put at 8, arithmetic); no sliding measurement was found at 1–5 or
  30–50 kPa. Pooled over studies of skin against rigid probes, wet friction falls with pressure (as p^b,
  b = −0.53, standard error 0.37), and the dry trend is uncertain
  ([Derler & Gerhardt 2012](https://doi.org/10.1007/s11249-011-9854-y)).
- **No friction measurement on the relevant skin was found.**
- **Silicone oil swells silicone.** Silicone oils of 50–1000 cP swelled solvent-extracted Sylgard 184 to 1.13–1.41
  times its mass, less for the more viscous ([Li 2026](https://arxiv.org/abs/2605.12125), a preprint, read from its
  Fig. 2A). Nothing was measured on Dragon Skin, Ecoflex or a Slacker-softened part.
- **The consequence (arithmetic on the table):** measured on skin, the pairings span about 0.1 (a fresh gel,
  sliding) to about 1.2 (a silicone lubricant gone, at onset). That does not bound the top: a tacky silicone held
  twice its load on skin without slipping, and a tacky dressing read 4.38 at onset and 2.59 sliding dry, 1.09 and
  1.47 wet, on a skin simulant (Gefen 2026); Smooth-On lists Dragon Skin 10 with the least Slacker it tabulates as
  not tacky. The list for steps 6–9 holds friction above μ_f 0.3 untrusted until its Coulomb push is checked there
  and the damping is settled (the damped tube fails it at 0.6, §16p), so most of this range sits there.
- **Where the force comes from, in catheter insertion:** the leading edge dominates the first ~30 mm,
  then friction takes over ([PMC10809236](https://pmc.ncbi.nlm.nih.gov/articles/PMC10809236/)).
- **For a straight section, arithmetic:** contact normals point sideways, so push force there is
  friction. The simulation can measure the geometric share directly by running with μ = 0.

### 5d. Consequence

- **The inputs are uncertain by** 1.3–3.4× in stiffness (at 100 % strain), **>10× in friction**, and an unknown factor in
  bulk stiffness and boundary condition.
- **A solver error of a few percent is plenty.** The answer is an **interval across corners**:
  - {soft, stiff};
  - {friction scenario};
  - {virgin, Mullins-conditioned}.
- **A shortcut to test (arithmetic).** With one material scaled uniformly, a prescribed motion, and exact
  contact, every force scales with μ, so a modulus corner costs no extra run. It breaks with friction
  smoothing, with a penalty stiffness that does not scale, and where the curve *shape* differs between
  sources.

## 6. The architecture, consolidated

**Three tiers**, with one physics definition behind them.

**Tier 1: an instant estimate while the user edits (microseconds).**
- Along the scan, slice by slice, apply the exact incompressible neo-Hookean thick-tube result
  (Haughton & Ogden, via [arXiv 2203.04110](https://arxiv.org/pdf/2203.04110)):

  > P/μ = (1/λ_z)·ln(λ_a/λ_b) + (1/(2λ_z²))·(λ_b⁻² − λ_a⁻²), with λ_a²λ_z − 1 = (B/A)²(λ_b²λ_z − 1)

  ✓ derived and checked by hand: 0.313 at λ_a = 1.3, B/A = 2, plane strain.
- **Push force** is μ_f·Σ P·perimeter·Δz, plus a geometric term.
- **Simpler formulas are badly wrong here** (arithmetic): at 30 % stretch and B/A 2, the small-strain
  Lamé formula overshoots by 44 % ✓ (36–80 % across B/A 2.8–1.05), and the thin-wall garment formula
  (Laplace) by roughly 2×.
- **Error against the full simulation on a real scan: unknown.** Measure it before trusting it.

**Tier 2: the full simulation (seconds to minutes, target ≤ 5 min): explicit dynamics on wgpu, in f32,
built TLED-style.**
- **Formulation:** total Lagrangian with precomputed shape-function derivatives, central differences, and
  lumped mass.
- **Element:** the linear tetrahedron with *selective* average-nodal-pressure (ANP): only the stiff
  volumetric term is node-averaged (§15c). ν 0.475–0.495, swept, plus 0.4995 once as a check (15d.5).
  Hexahedra with hourglass control
  remain the alternative if the sleeve meshes that way.
- **GPU layout:**
  - one thread per element writes its forces to its own slot, and nodes gather them (no float atomics);
  - displacements are stored rather than positions, and J − 1 is computed by expansion. The survey found
    f32 position storage gives 0.8–18 % spurious pressure at high K/μ.
- **Quasi-static:** a slow insertion with mass scaling. Kinetic energy must stay below 5–10 % of internal
  energy ([Abaqus](https://ceae-server.colorado.edu/v2016/books/gsa/ch13s04.html)).
  - This is not dynamic relaxation along the path: friction depends on the path taken, and DR's
    contact is frictionless in the literature.
  - DR may still settle the **seated** state.
- **Contact:**
  - The scan is an SDF.
  - Penalty contact with Coulomb friction (the LS-DYNA pattern), or kinematic projection (the TLED
    pattern). Projection has no friction (§15c), so the product uses the penalty, and projection stays
    a diagnostic (16d).
  - The penalty's gap biases pressure about 1 % low on the benchmark tube (§15c). It is measured and
    corrected for.
  - *Amended 2026-09-25 (§16o): the product uses the kinematic predictor/corrector with kinematic
    Coulomb friction, and the penalty is deleted. On the tube its penetration against the grid is at most
    0.2 µm, so G2 on the product rests mostly on the baked grid's own error (16j).*
- **Boundary options:** a free outer wall, a rigid case, or bonding to a stiffer outer layer (§5b).
- **Friction:** from the pairing library (§5c), swept over its range.
- **Readouts:**
  - push force along the path, with its μ = 0 geometric share;
  - pressure maps as area-weighted percentiles *(2026-09-26, §16s: the seated reading is the most-loaded 1 cm²
    patch, with the percentile shown beside it)*;
  - stretch.
- **Size of it (arithmetic):**
  - about 29–32k steps for the 100k-tet benchmark at ν 0.49, which leaves 3.7–4.1 ms per step within
    the 2-minute budget (§15c) *(2026-09-25: that assumed T = 10 T_s and no material damping; see §15c's
    note)*;
  - our rigid-body GPU pipeline's whole step at n_env 1 is about 0.74 ms ✓ (1/1.35k steps per second,
    `sim/L0/gpu-benches/PERF_BASELINE.md`).

**Tier 3: a surrogate for sweeps (later).** Trained on Tier 2 runs across insets and materials, as the
socket-donning study did. Reduced-order bases do not survive a sliding contact ✓
(`SIM_SOFT_REALTIME_RECON.md` §2l).

**Fallback for Tier 2:** AVBD (vertex Gauss–Seidel with an augmented Lagrangian). It shares the element
and contact kernels.

**Forward only.** The explicit solver runs forward only, and gradients stay with the implicit solver
(§11). The product scene therefore has no gradient path today. Design sweeps use a few parameters by
finite differences, or Tier 3.

## 7. Validation ladder

We have to climb it ourselves: the survey found no fast method with a published force validation.

1. **An exact answer.**
   - A long neo-Hookean tube on a rigid, frictionless mandrel.
   - The reference is the same-material oracle, `docs/soft_contact/thick_tube_reference.py` ✓. It
     reduces to Haughton–Ogden as ν → 0.5 and to Lamé at small interference, and asserts both.
   - This is our problem in its simplest form.
2. **Standard contact benchmarks.**
   - Hertz sphere-on-flat.
   - NAFEMS R0081, including CGS-10 (interference between cylinders; paid).
   - Frictional **ironing** at μ = 0.2 against published curves ([arXiv 1903.05859](https://arxiv.org/pdf/1903.05859)).
     Reported, not a kill criterion: its limits as a reference are in 16b.
   - **Cattaneo–Mindlin partial slip** in plane strain: K6 (16b).
3. **In-house f64 reference:** sim-soft's CPU Newton solver on small meshes.
   - For Tet4 at ν 0.49 its F-bar option over-softens by about 21 % against Lamé ✓
     (`sim/L0/soft/src/solver/backward_euler/config.rs:308-311`).
   - Rung 1's oracle solves exactly the solver's energy function W (§15b), so it is the first
     experiment's reference.
4. **A self-consistency check on every run:** the solver's reaction force against μ_f∫p dA from its own
   pressures. Also kinetic energy against internal energy.
5. **Published experiments, not home measurements** (Jon, 2026-09-24).
   - Reproduce studies that report measured forces or pressures on an elastomer fit.
   - For example: compression-stocking pressures on printed legs
     ([PMC12920784](https://pmc.ncbi.nlm.nih.gov/articles/PMC12920784/)), lip-seal radial force (18.3 N
     measured, [Engin 2019](http://przyrbwn.icm.edu.pl/APP/PDF/135/app135z5p57.pdf)), catheter insertion
     force in tissue ([PMC10809236](https://pmc.ncbi.nlm.nih.gov/articles/PMC10809236/)), and needle
     insertion into silicone gel ([PMC7755224](https://pmc.ncbi.nlm.nih.gov/articles/PMC7755224/)).
   - Each needs its geometry and materials recovered from the paper, which is work per case.

## 8. Sourcing measurements (no home lab)

**Decided (Jon, 2026-09-24):** *"id rather lean on already existing measurements from professionals good at
measuring."*

| Input | Published source | Gap |
|---|---|---|
| **Silicone stress–strain** | raw curves from Marechal 2021 ([repo](https://github.com/LucMarechal/Soft-Robotics-Materials-Database)) and Roels ([Zenodo](https://zenodo.org/records/14983287)), plus Smooth-On datasheets | **The Slacker blends are unmeasured.** Use the nearest measured grade by Shore, with a **wider** range, and flag it |
| **Bulk modulus / ν** | Sylgard 184, ν = 0.4950 ± 0.0010 ([Müller 2019](https://www.ebi.ac.uk/europepmc/webservices/rest/search?query=DOI:10.1039/c8sm02105h&format=json&resultType=core)) | Ecoflex and Dragon Skin are unmeasured. Sweep ν over 0.475–0.4995 |
| **Friction pairings** | §5c, re-sourced 2026-09-27 | silicone on skin with a lubricant: measured on the forearm, over time in one subject; none on the relevant skin, and none wet. Use the nearest analog and a wide range |
| **Comfort and pain limits** | the axial-rigidity convention (below); for stockings, *"Self-prescription is reasonably safe assuming that the compression gradient is 15–20 mmHg"* (≈ 2.0–2.7 kPa, [Wikipedia](https://en.wikipedia.org/wiki/Compression_stockings)); *(2026-09-27)* the relevant tissue's pressure-pain threshold, measured with a 1 cm² tip (fit plan U1) | No discomfort threshold of the relevant tissue, no pressure held on it for longer than a ramp, and none in its state of use (fit plan U1) |
| **Damping (loss)** | Ecoflex 00-30: four fractional fits and a DMA (fit plan U15) | *(2026-09-27)* Dragon Skin 10, and any Slacker-softened silicone: none found (fit plan U15) |
| **Validation experiments** | §7, rung 5 | each case needs its geometry recovered from the paper |

**Where no measurement exists, the answer is a wider range, labelled as such.** Nothing gets invented to
fill a gap.

**A physiological push-force ceiling.** A clinical convention treats an axial rigidity of about **550 g
(≈5.4 N)** as adequate for insertion ([Allen 1993](https://doi.org/10.1016/s0022-5347(17)36363-2)).
- It is a clinic convention, not a measured tolerance.
- It gives fit plan D1's push-force limit a real-world anchor: a sleeve that needs more push than the
  user's buckling force will not go in.

## 9. Decisions (Jon, 2026-09-24)

1. **The outer boundary:** *"probably an option for either"*. Free, cased and bonded are all design options
   (§5b).
2. **The first experiment: approved**: the tube on a mandrel. §15 details it.
3. **Lubricants:** *"different things can have different lubricants, so plan for that"*. A pairing library
   (§5c).
4. **Calibration:** no manual calibration. Limits and validation come from published measurements (§2, §7,
   §8).
5. **The measurement kit:** not pursued. We lean on professional measurements (§8).
6. **`ppf-contact-solver`: not used** (engineering call, 2026-09-24; Jon delegated engineering
   decisions).
   - The validation ladder (§7) already covers what it would.
   - Its Metal backend is days old and "slow" by its authors' account.
   - Another mixed-language install would repeat PolyFEM's cost.
   - Revisit only if a specific validation gap appears that an f32 barrier oracle alone closes.
7. **Leaving Tet10** for the explicit solver: ok.
8. **PolyFEM: cut** (*"I was pretty underwhelmed"*). The repo, cache and outputs were deleted from the
   machine.
   - The exporter and the oracle-only lip option were removed before landing.
   - They survive as `9f448e72` and `371f4a54` in a local tag, `fit-test-oracle-and-flow-pre-squash`.
   - The tag is not pushed, because that history carries scan-derived geometry.
9. **Soft-on-soft contact.** Jon, 2026-09-24: *"right now the hole is an offset of the scan, so nothing
   touches"*, but *"with a soft enough material they can"*, and *"soft on soft contact is definetely
   something i want in the future though, if not from the start"*.
   - **Designed in from the start; built second** (engineering call). The first experiment and today's
     product have no touching walls, so the first build does not need it.
   - Not designing it out: the lowered data carries the surface triangles, the phase-level trait has
     room for broad-phase and soft-contact phases, and the contact-list GPU tools are extracted as
     shared infrastructure (§14) *(2026-09-29, §17a: when soft-on-soft contact is built, after step 8's design)*.
   - Building it at once would put a broad phase into the first build before the base solver has passed
     its kill criteria.
   - When it is built, it gets its own research round (how practitioners do explicit soft self-contact
     on the GPU) and its own validation case, as §4 and §7 did for the base solver.

10. **The product's outer boundary** (Jon, 2026-09-24): *"right now it has no shell, its outside is free.
    but in the future i might add a shell/bond it to a shell. also sometimes there are multiple silicone
    shells layered."* So today's regime is the free wall (§15h). The confined case becomes a gate
    before any shell or bond design is simulated (15g step 2). *(2026-09-27, §16w: layered shells wait on the
    interface rule's PR, and the lowering refuses a wall where two materials meet until then.)*
11. **How the device is held** (Jon, 2026-09-24): *"it really depends, it could be in a shell, connected
    to something like a robotic arm, or just held in the hand."*
    - How it is held is a design option, like the outer boundary: in a shell, mounted to an arm, or held
      in the hand.
    - A shell, or a mount at the closed end, confines the material near it. So the confined case gates
      those designs too.
    - *(2026-09-27, §16w)* The lowering holds a wall by a mount or a rigid shell bonded to it. A case it slides along
      and the hand wait on a PR of their own, before any verdict that uses them. Step 7's first run is mounted.
12. **Speed against quality** (Jon, 2026-09-24): *"i just mean i want a fast simulation. but i dont want
    to sacrifice quality."*
    - D4's 5 minutes is a target, measured per press (one verdict), and driven down.
    - The quality gates (K2–K6) are never loosened to meet it.

**Engineering decisions** (Jon delegated them):
- the three-tier architecture, and AVBD as Tier 2's fallback (§6);
- the single-source translator (§13);
- the physics' wgpu, decoupled from Bevy's (§13e);
- the explicit solver runs forward only, and gradients stay with the implicit solver (§6, §11);
- the crate layout (§14);
- the first experiment's detailed design (§15);
- how a range becomes a verdict, and the Mullins state (fit plan U11);
- the penetration bound, G2 (fit plan §5);
- *(2026-09-27)* step 6's done-when re-barred to the bake's exact distance (§15g step 6, §16u), and step 7's wall
  meshed from the scan's exact distance with its cut points located on it (fit plan U17, §16v).
- *(2026-09-27, §16w)* the case moved to a PR of its own; the run's start clear of the wall by §15b's gap; the
  path's sampling held to G2's floor at the points it reads; the band's rule; and the product's band of eight fine
  cells.
- *(2026-09-28, §16x, labelling #977's calls)* the mount's plane, through the seated tip and normal to the
  centreline there; the band rule's factor of two; the dry pairing's mean ± its spread; and the sampling gate
  loosened after #977's build, to within 1 % of its bar at an interval it never read (mine).
- *(2026-09-28, §16x)* step 7's first run's rules: the loading ladder, the element size, ν's convergence in K (the
  choice of ν stays Jon's), the confined case held on the surface, f32 against f64 at K3's bar, the seated window, the
  band's trigger on a coarse correction, where G1 and G2 are read and G1's tolerance at G2's bar, the contact work over
  the hold, the free-scan test, stiffness scaling on the product, the push read over 1 mm of travel, the mass damping
  carried from the tube, and the executor's two new monitors.
- *(2026-09-28, §16y)* the collapse's rules: a run that fails with the loop's re-estimate made again every 50 steps;
  the collapse read by stabilizing the collapsing elements alone, cut at half their nodes' averaged J, 2μ to start,
  doubled up to λ, at most four masked runs, decided at four times h_K2's elements; energy sampling as the
  stabilization's form; and tracking the step every step not taken. Which element the product runs: the element as
  it is, left to me by Jon after the macro review's correction (fit plan U20, §16y).
- *(2026-09-29, §16z)* D1's element size read again: an eight-times wall and a replicate at four times; each doubling
  read against its replicate's scatter, and the size the coarsest from which every doubling passes; each comparison
  read at one re-estimate interval, the step control's own effect barred at K3's 0.5 %; the masked comparison doubled
  up to 25μ; the loading check, the masked comparison and G6 at the size picked, or else the finest whose runs stood;
  and G6 under the loop's re-run and a fixed 50 steps.
- *(2026-09-29, §17a)* step 3: a shared recorder that submits by a count of compute passes, at every read and before
  every host write, with each step's values in a uniform ring; the rigid pipeline recording through it; reads that
  stop on a failure and wait without a timeout; the pipeline tests under the adapter policy; and the contact-list
  tools and a rigid pipeline on a caller's device left to their users.

## 10. What the research could not see

- **Cut short by the search limit:**
  - glove, sock, finger-ring and consumer-device insertion force;
  - catheter and endoscope insertion *simulation* models;
  - Stribeck curves for skin lubricants;
  - measured ν or K for Ecoflex and Dragon Skin;
  - O-ring and seal ν-sensitivity studies;
  - LS-DYNA and Radioss GPU efforts;
  - any WebGPU explicit FEM;
  - pressure-pain thresholds for the relevant tissue *(2026-09-27: one study found, fit plan U1)*.
- **Paywalled:** Taylor 2008 (TMI) and Strbac 2015 (single against double precision), Nedoluha 2025 (ν
  measurement methods), and the NAFEMS R0081 references.
- **Unmeasured anywhere we looked:**
  - silicone on skin with a lubricant *(2026-09-27: measured on the forearm, §5c)*;
  - Prescale on sliding silicone;
  - DIC at large stretch on silicone;
  - a validated fast GPU contact force.

## 11. Code architecture: sharing the CPU groundwork

Jon asked:
- *"is it going to pretty much have to be like a rewrite? or can we do something like burn where we just
  specify if the backend is cpu/gpu?"*
- *"i want the gpu jump to use as much of the cpu groundwork as possible without taking an[y] sort of
  performance hit, and … the code itself to be as efficient/non redundant as possible while still having
  pretty intuitive architecture."*

**It is not a rewrite of the stack. It is a new solver.** A backend flag cannot move today's solver to the
GPU:
- **The algorithm itself changes.** `sim-soft`'s CPU solver is implicit Newton with a sparse direct
  factorization, and the round-1 survey found no GPU sparse direct solver in Rust or wgpu. The GPU solver is **explicit**.
- **A Burn-style switch works where the algorithm is the same**, which is the explicit solver on the CPU
  and on the GPU.

**The industry pattern is one model and several solvers.** Abaqus has Standard and Explicit, and LS-DYNA
has both, each over one model definition.

| Layer | Contents | CPU groundwork |
|---|---|---|
| **Model** | scene, SDF→tet meshing, material parameters, boundary options, contact pairings (§5c), readouts, viewer, studio integration | reused as it is |
| **Physics math** | constitutive P(F) per material, element internal force, ANP pressure averaging, contact and friction laws | the formulas carry over; the code is decided below |
| **Integrators** | **implicit Newton** (existing, CPU) and **explicit** (new) | implicit **stays**: it is the f64 reference, it handles small precise problems, and it gives differentiable co-design its gradients by the implicit function theorem (`docs/studies/soft_body_architecture/src/60-differentiability/`) |
| **Executors** (explicit only) | **CPU** (rayon; f32, and f64 as the precision check) and **GPU** (wgpu, f32) | the Burn-style switch: the same algorithm on both |

**The layout.**
- The explicit solver uses one flat, data-parallel layout: arrays per field, one force slot per element,
  and a gather at the nodes.
- It is TLED's GPU layout (§4). Its cost against hand-tuned layouts is not measured on either side.
- The implicit solver keeps its own sparse layout.

**Rigid and soft together on the GPU.**
- Explicit rigid and explicit soft dynamics step with the same small Δt. Coupling is a per-step contact
  exchange that can stay on the GPU.
- Keeping it there matters: reading back every step made our rigid pipeline 3–5× slower at n_env 256
  (`sim/L0/gpu-benches/PERF_BASELINE.md:89-90`).
- **Fit test:** the scan is a kinematic pose per step, which is trivial.
- **The class** (grasping, an exo on tissue) needs articulated rigid bodies on the GPU.
- The soft mesh is what earns the GPU. A small rigid system rides along so the data never leaves it.

**`sim-gpu` is not a constraint.** Jon, 2026-09-24: *"sim gpu is pretty old/not as much thought put into it
… you can do as much demo as you want/strip it to the studs as much as you want before you build it up."*
- **Redesign** the GPU foundation for rigid **and** soft.
- **Keep what has earned it:**
  - device setup;
  - chunked submits (long command buffers hung the readback, `pipeline/orchestrator.rs:28-37` at `df00dea8`, round 1)
    *(2026-09-29, §17a: measured, what blocks is compute passes, inside `finish` on Metal, not the readback; now
    `sim_gpu::submit::Recorder`)*;
  - the atomic contact append, extracted as shared GPU infrastructure for soft-on-soft contact (§9
    decision 9). The CAS float-add is extracted too, if scatter is chosen over per-pair slots and a
    gather. The first experiment needs neither *(2026-09-29, §17a: both are extracted when soft-on-soft contact is
    built, after step 8's design)*;
  - the CPU-conformance harnesses.

**The key design choice is to write the physics math once.** The repo has measured what writing it twice
costs.
- `sim-gpu`'s shaders are a hand-written WGSL copy of `sim-core`, validated only GPU-vs-CPU. They **silently
  lagged CPU fixes** (`sim/L0/gpu/src/pipeline/conformance_tests.rs:7-10`).
- **Decided (§13): a translator from a loop-free subset of plain Rust to WGSL.** It beat:
  - hand-written copies, whose drift has been measured;
  - CubeCL, whose CPU backend needs LLVM;
  - rust-gpu, which needs a pinned nightly and C++ to regenerate shaders.

  Orchestration stays per backend, because rayon loops and GPU dispatches differ.

**Constraints the design must meet** (`SIM_SOFT_REALTIME_RECON.md:3006-3011`, ✓):
- **The GPU executor is its own L0-io crate**, for F1's grading reasons, not a feature behind
  `sim-soft`'s `gpu-probe` door. That door is what `SIM_SOFT_REALTIME_RECON.md:3006-3011` prescribed.
- no C toolchain;
- wasm32 must build;
- `grade` stays A.

**Jon's standard for it** (2026-09-24): *"I want the highest ceiling possible performance wise, and I
love good, effcient architecture. I value sharpening the sword and checking the whole blade carefully
before swinging the axe."*

## 12. Repo facts that force decisions

From a read-only survey of the repo. The referents were re-checked in the PR #964 review.

| # | Fact | Referent | Forces |
|---|---|---|---|
| F1 | **L0 bans wgpu; L0-io allows it** (bans only `bevy*`, `winit`) ✓. A `tier_up_feature` applies **only under `--all-features`** ✓. Coverage runs **default features only** ✓, and so does the doc check. | `xtask/src/grade.rs:4323`, `:4370-4379`, `:4709-4730`; `xtask/src/coverage_run.rs:609` | **The GPU executor is its own L0-io crate, not a `sim-soft` feature.** Code behind a feature is not coverage-measured or doc-checked, so it would be the least-graded code in the solver |
| F2 | **`[build-dependencies]` are invisible to the L0 dependency count and ban checks**, but still need a justification comment. | `grade.rs:4786-4825`, `:3992-4119` | **If we generate code, generate it into `src/` and commit it**, with a freshness test that regenerates and diffs |
| F3 | **`sim-soft`'s surfaces are all f64/nalgebra.** `Material` is per-point (`energy`, `first_piola`, `tangent`, `validity`). **`Solver` and `ContactModel` are Newton-shaped** (`Tape`, `NewtonStep`, `energy/gradient/hessian/ccd_toi`). | `sim/L0/soft/src/material/mod.rs:35-71`, `solver/mod.rs:202-298`, `contact/mod.rs:367-442` | **The explicit solver does not implement `Solver`**; it needs its own integrator interface. It needs only P(F) and validity, which makes a smaller **constitutive-kernel layer** (float-generic, no dynamic dispatch) the physics-math layer. The existing `Material` impls are conformance-tested against it |
| F4 | **Contact primitives are `dyn Sdf`, so they cannot be uploaded.** `cf_geometry::SdfGrid` is dense (values, w/h/d, cell, origin; z slowest) and **`sim-gpu` already uploads exactly this layout, with a WGSL trilinear lookup**. `Solid::sdf_grid_at` takes its cell size **in mm**; `sim-soft` works in metres. | `design/cf-geometry/src/sdf.rs:147-171`; `sim/L0/gpu/src/pipeline/model_buffers.rs:198-219`; `sdf_sdf_narrow.wgsl:97-136`; `design/cf-design/src/solid/query.rs:378` | **GPU contact geometry is a baked `SdfGrid`.** Reuse the layout and lookup. Guard the mm/m unit seam with a test |
| F5 | **`StaggeredCoupling` builds a fresh `CpuNewtonSolver` every step and assumes implicit steps.** The rigid side steps at `model.timestep`, and nothing ties it to the soft `dt`. Its gradients use `sim-soft`'s implicit-function-theorem adjoints. | `sim/L1/coupling/src/step.rs:52-118`, `:106-108`, `lib.rs:8-32`, `construct.rs:55-56` | **Explicit coupling needs subcycling**: many soft steps per rigid step. That makes it a new coupling path. The gradient paths stay with the implicit solver |
| F6 | **`sim-gpu` is rigid-only** (free joints, nv ≤ 60), with 13 hand-written WGSL shaders **and GPU structs hand-mirrored in WGSL** (`pipeline/types.rs`). The CPU-conformance harnesses and `CF_REQUIRE_GPU` + lavapipe CI are worth keeping. | `sim/L0/gpu/src/pipeline/orchestrator.rs:141-148`; `pipeline/types.rs:95-248`; `test_support.rs:1-60`; `.github/workflows/quality-gate.yml:589-623` | **Single-sourcing must cover struct layouts, not only functions.** Keep the conformance and CI patterns |
| F7 | **The repo has no code generation, proc-macro crate or direct naga use.** Only `xtask/build.rs` does anything at build time. | `xtask/build.rs` is the only build script; no crate is a proc-macro or depends on `naga` directly (greps) | **Any single-source approach is new infrastructure.** Its cost counts in the spike |
| F8 | **Docs disagree on the L0 dependency cap** (80/100 in STANDARDS, 60 in the plan, 100/120 in code; the code is enforced). "sim-soft is the only tier-up declarer" is stale (`cf-device-types` has `{ bevy = "L1" }`). | STANDARDS.md:731, `:981`; `design/cf-device-types/Cargo.toml:18` | Cleanup for later; out of scope |

## 13. Writing the physics once: the single-source spike

**The candidates:**
- **CubeCL** is out without a run: its CPU backend needs LLVM (`cubecl-cpu` → `cubecl-llvm` →
  `llvm-sys` + `cc`) ✓, which breaks the no-C-toolchain rule.
- **rust-gpu** compiles plain Rust to SPIR-V through a nightly rustc backend. It is *"not yet
  production-ready"*, and dimforge's `nexus` uses it.
- **A translator** from a loop-free subset of plain Rust to WGSL.

**The test.** One material kernel, compressible Yeoh P(F) (9 floats in, 9 out), written once and run on
an Apple M4 Pro against hand-written WGSL and an f64 reference. The spike was throwaway, outside the
repo.

**The instrument.** wgpu timestamps: 50 back-to-back dispatches per pass, and every kernel interleaved
over 10 rounds after a warm-up. Two instruments were replaced:
- a per-dispatch one, which read 0 ms;
- a one-kernel-per-process one, where hand WGSL alone moved between two levels. What causes the two
  levels has not been isolated.

### 13a. Results

Two interleaved runs, each against its own hand-WGSL baseline. Ratios are to hand WGSL.

| Kernel | 100k | 300k | 1M | Max rel err vs f64 |
|---|---|---|---|---|
| Hand WGSL, run 1 / run 2 | 0.0196 / 0.0199 ms | 0.0505 / 0.0501 ms | 0.2954 / 0.2934 ms | 5.3e-6 |
| rust-gpu, math written with loops (run 1) | 1.75× | 1.81× | 1.07× | 7.3e-6 |
| rust-gpu, loop-free, run 1 / run 2 | 1.00× / 0.995× | 1.01× / 1.020× | 1.00× / 1.000× | 7.2e-6 |
| rust-gpu, loop-free, no bounds checks (run 1) | 0.99× | 1.00× | 1.00× | 7.2e-6 |
| **The translator's WGSL** (run 2) | 1.003× | 1.000× | 1.000× | 7.2e-6 |

- **Loops are the whole gap.** Both loops were removed together, so which one costs more has not been
  isolated.
- **Bounds checks cost nothing measurable.**

### 13b. The translator

- **How it works.** It reads the same Rust source the CPU compiles, with `syn`, and writes a WGSL
  function. naga must parse and validate the output before the file is written.
- **The subset.**
  - Allowed: `let` bindings, array destructuring, arithmetic, comparisons, math methods (`ln` → `log`,
    `sqrt`, `min`, …), calls, fixed-size arrays, `#[repr(C)]` structs of scalars, and `if`/`else`
    expressions (emitted as `select`).
  - Refused: loops, mutation, `self`, and anything else. The refusal was made to fire twice: on the
    looped Yeoh, and on a function whose only violation is a `for`.
- **Entry points stay hand-written per backend.** They cover storage indexing, gathers and dispatch
  shape, and they call the generated functions.
- **Size:** 148 lines on stable Rust (130 excluding blanks and comments), in the spike (deleted). Its
  dependencies are `syn`, `quote` and `naga`; no build dependency compiles C.
- **Struct layouts (F6).** An array in a uniform buffer needs a 16-byte stride, and naga's validator
  **rejects** a violation rather than laying it out wrongly. `array<f32, 3>` in a uniform failed with
  `ArrayStride { stride: 4, alignment: 16 }`; in a storage buffer it validated. `vec3` stays out of the
  subset.

### 13c. Verdict: the translator

| Pre-registered bar | Translator | rust-gpu |
|---|---|---|
| GPU within 10 % of hand WGSL | pass: 1.00× | pass: 1.00×, only if loop-free, which nothing enforces |
| f32 agreement with the f64 reference (no threshold was pre-registered) | max rel err 7.2e-6 | max rel err 7.2e-6 |
| No C toolchain | pass | fail: regenerating shaders needs `spirv-tools-sys` (C++, through `cc`) and a pinned nightly with `rustc-dev` |
| Readable definition | pass: plain Rust | pass: plain Rust, and a much larger subset |

- **rust-gpu also broke twice on first contact:** a backend library that would not load on macOS 27,
  and an unbounded `glam` requirement.
- **On the one kernel measured (M4 Pro, Metal), neither choice cost speed.** The shared math ran at
  1.00× hand WGSL both ways.
  Orchestration, where GPU performance work happens, stays in hand-written WGSL with the whole language
  available.
- **What the translator gives up:** writing orchestration in Rust, and generics or traits in the shared
  math.
- **What it costs:** new infrastructure (F7), meaning a translator that grows with the subset, plus a
  freshness test that regenerates the WGSL and diffs it (F2).

### 13d. Design rules from the spike

1. **Write the shared math loop-free.** Loops over private arrays cost 1.07–1.81× (13a). The translator
   enforces this by refusing loops.
2. **Never rely on NaN or infinity inside a shader.** The WGSL spec (Candidate Recommendation Draft,
   21 September 2026): *"Implementations may assume that overflow, infinities, and NaNs are not present
   during shader execution."* Inversion must be detected by an explicit `J ≤ 0` comparison before
   `ln(J)`, and blow-up by explicit bounds, not by NaN propagation.
3. **No `vec3` in shared structs.** Uniform arrays need a 16-byte stride, and naga rejects violations at
   validation (13b).
4. **The physics has its own wgpu version** (13e).

### 13e. wgpu version: the physics side is decoupled from Bevy's

Jon, 2026-09-24: *"maybe bevy shouldnt keep us from upgrading wgpu on the backend/phsycis."* Decided on
these facts:
- **The workspace's `wgpu = "27"` is not Bevy's.** Only `sim-gpu` and `sim-soft`'s `gpu-probe` use it;
  `bevy_render` declares its own `wgpu ^27` (`cargo tree -i wgpu`).
  - Bevy 0.18 (ours) pins wgpu 27, 0.19.1 pins 29, and 0.20.0-rc.1 pins 30.
  - wgpu 30.0.0 shipped 2026-07-01 (crates.io).
- **Two wgpu majors in one binary work on Metal.** A spike binary (deleted) built with wgpu 27 and 30 held a live
  device from each at once on the M4 Pro. The second copy added 14.7 s wall (121 s CPU) to a release
  build. The all-platform graph has no `links` conflict: only `rayon-core` and `wasm-bindgen-shared`
  declare one. **Not tested: both at once on Vulkan.**
- **The price.**
  - Separate devices, so physics buffers never reach Bevy's renderer zero-copy. The viewer reads a CPU
    snapshot per displayed frame: 100k nodes × 3 f32 = 1.2 MB.
  - A binary that links both compiles wgpu twice.
  - Nothing forbids duplicates: `deny.toml` has no `[bans]` section.
- **Consequence.** The GPU executor crate (F1) takes its own wgpu entry and moves on its own schedule. It
  never shares a device with Bevy, and the renderer consumes snapshots. Zero-copy is revisited only if a
  measured frame budget demands it.

## 14. The crate layout

Jon handed the sign-off to me (2026-09-24: *"you may be in charge of the design/deciding is 14 v2 is
solid"*).
- **The crate boundaries, dependency edges and tiers** were signed off after two cold reviews (14e).

### 14a. The layout

| Crate | Tier | Status | Holds |
|---|---|---|---|
| **`sim-soft-explicit`** | L0 | new | **The explicit solver, minus the GPU.** The executor trait. The explicit model and state data layout (flat arrays; `#[repr(C)]` parameter blocks with no `vec3`). The shared math (14b), written once in the loop-free subset and compiled at f32 and f64, with its committed generated WGSL and a freshness test. The **CPU executor** (rayon on native, sequential on wasm32, as `newton.rs` does). The **stepping loop**, which owns the order of phases within a step, batching, the stable time step and mass scaling, and the energy monitors and stop rule, over any executor. A `test-fixtures` feature with small lowered meshes, as `sim-core` has *(replaced in step 2's design by a public module, 16f)*. |
| **`sim-wgsl-gen`** | L0 | new | The §13 translator: `syn` (with `proc-macro2` for source positions), plus `naga` to validate its output, on the physics side's naga version. A `write` command regenerates the committed WGSL, and the freshness test names that command when it fails. A dev-dependency of `sim-soft-explicit`. |
| **`sim-soft`** | L0 | grows | The model as today, plus **lowering** it to `sim-soft-explicit`'s data, including resampling the insertion path evenly in time *(2026-09-27, §16t: the path is the fitted pose)*. **Baking the obstacle SDF from its triangle mesh** (flood-fill sign and the Gaussian pre-smooth, moved from `tools/cf-sim-research`) *(2026-09-27, §16u: a new bake, not moved: the parity of a ray's crossings for the sign, no pre-smooth, and a fine grid in bricks near the surface)*. The **scenarios and readouts in model terms** (contact pressure by region) *(2026-09-26, §16s: D1's readings landed in `sim-soft-explicit`'s `readings`, over the solver's snapshots, so `sim-soft` calls them and does not build a second set)* *(2026-09-27, §16w: nothing in `sim-soft` calls them yet; step 7's runs read them from the tool)*. The test of its `Material` impls against the shared math (F3). The implicit Newton solver stays as it is. |
| **`sim-gpu`** | L0-io | rebuilt | **The GPU executors.** It *extracts* shared infrastructure from today's rigid code: the device context (`context.rs`), and chunked submission, which today sits inside the rigid `step()` (`pipeline/orchestrator.rs:28-37`) *(2026-09-29, §17a: extracted as `sim_gpu::submit::Recorder`; the lines cited are `df00dea8`'s)*, and the contact-list tools (the atomic append; the CAS float-add if scatter is chosen) *(2026-09-29, §17a: extracted when soft-on-soft contact is built, after step 8's design)*. It adds `soft`, the explicit executor, whose hand-written entry points fetch, gather and scatter around the generated WGSL. It holds the **GPU-vs-CPU conformance tests** against `sim-soft-explicit`'s CPU executor. The rigid pipeline stays as it is until its own redesign, keeping the parts only it uses. It depends on `sim-soft-explicit` and `sim-core`, not on `sim-soft`, and has its own wgpu version (13e). |
| `sim-coupling` | L1 | later | Two-way explicit rigid–soft coupling on the CPU (subcycling, F5). **The fit test does not need it**: the scan is a kinematic pose, applied in the contact law. GPU rigid–soft exchange lives in `sim-gpu`, on one device. |
| `sim-bevy-soft`, the studio, `tools/cf-sim-research` | L1 / App / tool | consumers | Pick the executor (CPU or GPU), and show results from CPU snapshots (13e). |
| `sim-gpu-benches` | L1 | grows | Benchmarks for both executors. L0 bans `criterion`, even as a dev-dependency. |

**The Burn-style switch is the executor trait.** The shape follows Burn's:
- the trait and a reference CPU backend in one crate;
- the GPU backend in another;
- model-side code generic over the trait.

`sim-soft` (L0) never depends on `sim-gpu` (L0-io).

### 14b. What the shared math holds

Everything both executors must compute identically, as pure per-element or per-node functions:
- **Constitutive:** P(F) per material, and the strain energy Ψ, which the internal-energy monitor needs.
  *(Amended 2026-09-25, §16p: and the material's viscous stress, from the element's velocities.)*
- **Validity:** the `J ≤ 0` check, as an explicit comparison (§13d rule 2).
- **Element force:** internal force from P and the shape gradients.
- **ANP:** the per-element and per-node pieces of pressure averaging. The gather between them is
  orchestration.
- **Contact:** the contact and friction laws per pairing (§5c). Obstacle contact first. Soft-on-soft
  contact laws join the same shared math when it is built (§9 decision 9).
- **The SDF query,** with the clamped semantics the product uses (`distance_clamped` and
  `gradient_clamped`, `insertion_sim.rs:1619, 1623`). That means the value, and a finite-difference
  gradient at ±cell/2: seven trilinear lookups over a fixed neighbourhood of at most 32 grid values.
  - The shared math owns the indices, the clamping, the weights and the combination. Each executor
    only fetches the neighbourhood.
  - The analytic trilinear gradient (8 values) is an option to *measure* (15d.9), not an assumption.
    It is discontinuous at cell faces, and the product's pre-smooth was tuned against the
    finite-difference gradient.
  - *Amended 2026-09-25 (§16o): the query is a tricubic (Catmull–Rom) interpolant of 64 grid values
    and its exact gradient, the grid extended linearly past its faces; `sdf_grid_index` keeps the index
    flattening in the shared math. The pre-smooth was tuned for the trilinear lookup. Whether tricubic
    still needs it, and its surface bias against G2 (the code's own estimate is σ²κ/2 at a 3 mm grid:
    about 0.11 mm on a 40 mm radius and 0.9 mm on 5 mm-radius features, `insertion_sim.rs:1686–1690`,
    against G2's 0.05 mm on `base_mold`), are 2d's to measure.* *(Measured, not settled: §16r.)* *(2026-09-27,
    §16u: §16r's grids had the flood fill's sign. Signed by parity, without the pre-smooth, a 0.25 mm grid's own
    error meets G2 at `base_mold`'s 5 mm inset; the bake does not pre-smooth.)*
- **Time integration:** the per-node explicit update (velocity, position, damping, mass scaling,
  kinematic boundary conditions), and the per-element stable time-step estimate.
- **The obstacle's pose** between two time samples, by interpolation. Lowering resamples the path
  evenly in time, so no search is needed. Today it is a polyline by arc length
  (`point_along_polyline_at_arc_distance`, `insertion_sim.rs:3509`).

Orchestration stays per backend: storage indexing, gathers and scatters, reductions, and dispatch shape.
The boundary is guarded in both directions:
- the translator's subset keeps storage access out of the shared math;
- per-phase conformance tests keep physics out of the orchestration (14d).

### 14c. Why this shape: the facts it rests on

- **Tier caps** (`xtask/src/grade.rs:4438-4453`).
  - L0: 100 release / 120 test. L0-io: 200 / 220.
  - `sim-soft` carries a recorded override to 200.
  - `syn`, `quote` and `naga` are allowed in L0: the ban list has the `wgpu` prefix, not `naga`
    (`:4323-4368`).
- **Dependency counts,** as `grade` counts them (`cargo tree -e normal --prefix none`, root included,
  wgpu 27).
  - `sim-soft` 110 (145 with all features), `sim-gpu` 73, `sim-core` 25.
  - A `sim-gpu` that depended on `sim-soft` would reach 146. It would pull in 73 crates it lacks today
    (`sim-soft` itself, faer, gemm, parry3d, rand, serde, …), most of which a GPU executor does not use.
- **Baking the obstacle SDF in `sim-soft` costs no new crate.**
  - `mesh-sdf` is already in its graph, via `mesh-offset` (`cargo tree -p sim-soft -i mesh-sdf`).
  - `cf-design` would cost one (110 → 111, measured by the second reviewer), and it is not on the
    fit-test path. *(2026-09-27, §16v, §16w: the fit test's wall is built with `cf-design`, by the lowering's
    caller; `sim-soft` still does not depend on it.)*
- **The CPU executor sits beside the contract, not beside the model,** because it consumes only lowered
  data. That:
  - lets `sim-gpu`'s conformance tests use it as an ordinary dependency;
  - keeps explicit-solver work out of `sim-soft`'s long test suite;
  - keeps faer and the rest out of the GPU crate.
- **The SDF lookup is written twice today, and the two copies disagree,** so it must be single-sourced.
  Both use the finite-difference gradient at ±cell/2 (`sdf.rs:507-508`, `wgsl:147-153`).

  | Behaviour | CPU: `cf_geometry::SdfGrid` (`design/cf-geometry/src/sdf.rs:421-557`) | GPU (`sim/L0/gpu/src/shaders/sdf_sdf_narrow.wgsl:103-170`) |
  |---|---|---|
  | Unclamped, outside the grid | `distance` returns `None` | `src_trilinear` returns `1e6` |
  | **Clamped** (the product's path), at or beyond the far face | `distance_clamped` returns the face value (`:480-494`) | `src_trilinear_clamped` clamps onto the face, but `src_trilinear` rejects the face's index, so it returns `1e6` whenever the clamped point rounds to that index |
  | Interpolation | fused `mul_add` | `mix` |
  | Gradient fallback threshold | `1e-10` (`sdf.rs:524`) | `1e-5` (`wgsl:165`) |

  The WGSL `SdfMeta` also holds a `vec3` (`wgsl:34`).
- **The wgpu 30 migration of `sim-gpu` is small,** measured in a throwaway worktree: 30 type errors from
  5 API changes.
  - `push_constant_ranges` was removed (12), and bind-group layouts are now `Option` (12).
  - `get_mapped_range` returns a `Result` (3), `InstanceDescriptor` lost `Default` and is passed by
    value (2), and `RequestAdapterOptions` gained a field (1).
  - The same bump breaks `sim-soft`'s `gpu-probe` test on the same APIs
    (`sim/L0/soft/tests/invariant_6_gpu_probe.rs:57, 62, 122-125, 170`). It migrates in the same PR.
  - All 13 existing shaders parse and validate under naga 30.0.1, measured in the PR #964 review.
    Backend translation and pipeline creation are not tested.
- **One source serves both precisions.** Each shared-math file is written against a type alias `R` and
  included twice: `mod f32 { type R = f32; include!(…) }`, and the same with f64.
  - Verified in a scratch crate (deleted) under the workspace lint levels (clippy all, pedantic and nursery,
    `missing_docs`, `-D warnings`): clean. An f32-vs-f64 agreement test passed, and failed when
    tightened past f32.
  - One precedent in the repo: `xtask/build.rs:13`.
  - **Not built yet:** the translator's mapping of `R` to `f32`, and the same trick applied to the CPU
    executor, so it can also run at f64 as a check that f32 is adequate.
- **The translator builds for wasm32 with naga.** A scratch crate (deleted) with `syn`, `quote` and `naga`
  (`wgsl-in`) passed grade's wasm command (`cargo check --target wasm32-unknown-unknown
  --no-default-features`).
- **CI runs a crate's tests only if a list names it.**
  - `grade` runs in CI with `--skip-coverage`, which runs no tests. *"a crate absent from these lists
    and from `tests-release` has its unit tests run in NO CI context at all"*
    (`.github/workflows/quality-gate.yml:406-408, 471-475`).
  - So both new crates join tests-debug shard 3 (`:590`) in the PR that creates them, and the freshness
    test is made to fail once in CI.
  - Coverage works differently: the weekly job walks every crate, and it runs `sim-gpu` on lavapipe
    (`scheduled.yml:220-284`).
- **GPU tests run on lavapipe (Vulkan) in the tests job as well,** with `sim-gpu` in shard 3 and
  `CF_REQUIRE_GPU` set (`quality-gate.yml:589-623`).

### 14d. The executor trait's shape (decided), and what is not decided

**The trait is phase-level.**
- The stepping loop calls one method per phase, in the fixed order of 15f. **The order is written
  once**, in the loop. Soft-on-soft contact adds a broad-phase and a soft-contact phase to that list. It does not change
  any crate boundary. The lowered data carries the surface triangles from the start. *(Amended
  2026-09-25, §16o: the kinematic law predicts each node's step from the forces before contact, the
  elastic and (since §16p) viscous forces today. A soft-contact phase, or any other force phase, must feed that
  prediction, and a node touching two surfaces needs a joint correction: part of step 8's design.)*
- The GPU executor records each phase as compute passes, submits in chunks, and **never reads back**
  except on an explicit read call. The pose samples stream to the device a batch at a time. *(2026-09-30, §17b: the
  pose is interpolated on the host and carried in each step's values, and a step is one pass.)*
- The monitors (kinetic and internal energy, contact force) are reduced on the executor, and read
  every k steps.
- **What phase-level costs:**
  - the loop is generic over the trait, so calls are statically dispatched, which adds nothing;
  - per-phase kernel launches are a cost that fusing phases would remove. It is not measured.
- **Conformance is per phase,** as `sim-gpu`'s harness already works stage by stage
  (`sim/L0/gpu/src/pipeline/conformance_tests.rs:12-16`). That is also the guard against physics creeping into the
  orchestration.
- Reading back every step made the rigid pipeline 3–5× slower at n_env 256 (§11).

**Not decided here:**
- The exact method signatures, which are settled in the build.
- Fusing adjacent phases into one GPU kernel, as a later optimization. It must keep per-phase
  conformance checkable.
- **Parameter blocks:** derive `bytemuck::Pod` (a few more dependencies), or cast from plain arrays.
- **The rigid pipeline's redesign,** and whether the retired `gpu-probe` door is removed.
- **Routing the implicit Newton `Material` impls through the f64 shared math.** It changes their
  floating-point order, so it is a behaviour change, tested downstream when it is done. The Newton path
  would also need the tangent dP/dF in the shared math.
- **The `cf-design` mm seam (F4)** belongs to whichever consumer bakes from a `Solid`. The fit test bakes
  from the scan mesh, in metres.

### 14e. How the layout was checked

Two cold reviews, each against criteria written beforehand.
- The first review moved the CPU executor beside the contract, and restored the translator's naga
  validation.
- The second, limited to structure, found no problem with any crate boundary, dependency edge or tier.
- **Checked by no one:**
  - two wgpu versions at once on Vulkan;
  - how coverage scores code that is included twice;
  - whether `sim-gpu` grades A today *(2026-09-29, §17a: it does, at `df00dea8`)*;
  - the spike timings;
  - `sim-soft-explicit`'s own dependency count and wasm32 build, since it does not exist yet. L0's cap
    is 100 release / 120 test.

## 15. The first experiment

**What §15 is built on:**
- §14's layout;
- a code survey of `sim-soft` (file:line referents below);
- a method-research pass. Items it could not source are marked UNSOURCED;
- **the oracle,** `docs/soft_contact/thick_tube_reference.py`. Run it with `uv run`; it asserts its own
  checks.
- **the whole-plan review** (15i): two cold reviewers, one of whom built the element in a scratch model
  that was not kept
  and measured it.

Arithmetic is labelled as such.

### 15a. The question, and the kill criteria (pre-registered)

**The question:** can an explicit solver on §14's layout, in f32, reproduce the exact tube-on-mandrel
contact pressure within 5 % at ν ≥ 0.49? And can it run a 100k-tet insertion on the M4 Pro within
2 minutes?

| | Criterion | Measured as |
|---|---|---|
| **K1 speed** | a 100k-tet insertion at ν = 0.49, GPU executor, **≤ 2 min** | wall-clock from setup to the last readback, including loading, hold and the measurement window. A first bar on the tube, not derived from D4; build step 2 derives the product's budget |
| **K2 accuracy** | band pressure within **5 %** of the oracle, **both raw and gap-corrected** (15d.1) | same material, free ends, frictionless, the pinned SDF (15c). At ν 0.49 and 0.495, for (λ_a, B/A) = (1.1, 2) and (1.3, 2), on the 100k mesh |
| **K3 precision** | CPU f32 against CPU f64, same executor: band pressure within **0.5 %** (frictionless, 50k), and the Coulomb push's reaction within **0.5 %** (μ_f 0.3, 10k). *Amended 2026-09-24 (PR #965 review):* the band pressure also within 0.5 % at every pair-averaged ring level (15d.1), not only in the mean, since D1's 95th-percentile reading depends on the local values. *2026-09-26 (§16s): D1's seated reading is now the 1 cm² patch; f32 and f64 read it the same to five decimals at 50k and μ_f 0.3* | step 2 of the build (15g), before any GPU code. This is the fit plan's *"precision spike on contact before any GPU contact code"* |
| **K4 robustness** | J > 0 in every element at every step of every valid run | explicit check (§13d rule 2). Any J ≤ 0 in a valid run is a failure. A run that breaks a validity gate is invalid, and K4 does not judge it. *(2026-09-28, §16y rule 1: a run that goes non-finite, fails a gate or inverts an element with the loop's re-estimate every 500 steps is made again every 50 steps, and an inversion in a valid run there is K4 failing. On the product, the frictionless run at twice h_K2's elements went non-finite at 500 with or without the collapse resisted; its gates could not be read, so K4 did not judge it; at 50 it stood)* |
| **K5 product readings** | the peak push force during entry and the seated 95th-percentile pressure (fit plan D1's readings) change ≤ 5 % from 50k to 100k. *2026-09-26 (§16s): the percentile was decided by one or two rings (a diagnostic); D1's seated reading is now the most-loaded 1 cm² patch, and the μ = 0 push is read over 10 mm of travel. K5 passes on them from 50k to 100k; the element size they need on the product is open (step 7)* | the tube's entry is a sharp edge, like the product's mouth. **A gate on the verdict's design, not on the solver:** if it fails, D1's readings or the lip radius are revisited before step 7 |
| **K6 friction** | *Amended 2026-09-24, in step 2's design and before any data (16b):* Cattaneo–Mindlin partial slip in plane strain, a rigid cylinder on the block. The stick zone's half-width within 0.03a of the closed form while the tangential load rises to 0.8·μ_f·P, and the retained stick zone's within 0.03a while it falls back. It replaced frictional ironing, whose published curves could not carry a 5 % gate (16b) | CPU, build step 2. Friction's only external reference: the Coulomb push (15d.7) checks consistency only |

**Why two corrections to K2:**
- **Requiring both raw and gap-corrected results:** the penalty's gap biases pressure low by about 1 %
  (15c), and the element biases it high by 0.7–1.65 % at 50k–100k (15c). A raw pass could come from
  the two cancelling. The gap-corrected number isolates the element. *(Since 2026-09-25 the kinematic
  law leaves no penalty gap, and raw and gap-corrected agree to the lookup's error, §16o.)*
- **Judging at 100k:** 10k fails on the element alone at one corner (+5.4 %).

**Validity gates.** A run that breaks one is invalid, not failed:
- kinetic energy stays ≤ 5 % of internal energy over the measurement window (the strict end of Abaqus's
  5–10 %). *Amended 2026-09-24 in step 2's design, before any data (16e):* over every phase a readout
  is taken from, and with an energy balance added;
- the band's axial stretch is within 0.5 % of the oracle's λ_z. K2 then compares against the oracle at
  the band's measured λ_z (15d.1), since 0.5 % of λ_z moves pressure by up to 2.1 %.

**What a failure points at:**
- K1 → dispatch overhead first (per-phase GPU time against fixed cost), then AVBD (§6).
- K2 → the element (15e), the contact variant, or the SDF (compare with the analytic SDF, 15d.9).
- K3 → f32 storage or accumulation (§6's displacement storage).
- K4 → the element or the loading time.
- K6 → the friction law or its stick state.

### 15b. The case

- **Geometry (SI).**
  - Inner radius A = 10 mm, outer B = 20 mm, length L = 120 mm.
  - A rigid mandrel of radius a = 11 or 13 mm (λ_a 1.1, 1.3), with a hemispherical nose of radius a,
    inserted 100 mm from the free entry.
  - All positions are in **reference** coordinates, measured from the entry.
  - **The band is pre-registered at z ∈ [20, 60] mm.** That is two wall thicknesses from the entry,
    about 27 mm clear of the nose's contact region (z ≈ 87–100 mm), with 20 mm of tube ahead of the
    nose. Its flatness is reported (15d.1), not used to find it.
- **Material:** `sim-soft`'s compressible neo-Hookean, Ψ = μ/2(I₁−3) − μ ln J + λ/2(ln J)²
  (`sim/L0/soft/src/material/neo_hookean.rs:5-7`). That is **exactly the oracle's W**.
  - μ = 23 kPa and ρ = 1070 kg/m³ (`ECOFLEX_00_30`).
  - λ comes from ν through `from_lame`. `from_young_poisson` asserts ν < 0.45 (`neo_hookean.rs:70-78`).
  - `NeoHookean::validity()` declares ν ≤ 0.45 (`neo_hookean.rs:110`). The shared math's validity is the
    J > 0 check alone, with no Poisson bound.
  - The density comes from the material, not the implicit solver's global `SolverConfig.density`
    (`solver/backward_euler/config.rs:253-255`).
- **Boundary conditions:** the far end is held, the entry end is free, and the outer wall is free.
  - A free body from the entry to any section in the band carries only radial contact traction. So the
    axial force is zero, and **the free-ends oracle applies**. This is a statics argument; the decay
    lengths are measured, not assumed.
  - It matters. At ν 0.49, free ends against plane strain moves pressure by 2.3 % and 5.7 % in K2's two
    geometries (9.3 % at B/A 1.5). ν 0.49 against 0.495 moves K2's free-ends pressure only 0.11–0.12 %.
- **Oracle values** (p/μ, free wall, free ends):

  | | ν 0.49 | ν 0.495 |
  |---|---|---|
  | λ_a 1.1, B/A 2 | 0.12352 (λ_z 0.98523) | 0.12365 (λ_z 0.98511) |
  | λ_a 1.3, B/A 2 | 0.30507 (λ_z 0.95779) | 0.30544 (λ_z 0.95747) |

  The compressible answer sits 0.40 % below the incompressible Haughton–Ogden formula at ν 0.49, and
  0.20 % below at 0.495 (plane strain, λ_a 1.3). K2 is judged against the oracle, not the formula.
- **Friction:** μ_f = 0 for K2, and μ_f = 0.3 for the Coulomb push.
- **Loading.** The mandrel's nose starts 5 mm before the entry.
  - Its speed ramps up over the first 10 % of the loading time T, holds constant for 80 %, and ramps
    down over the last 10 %.
  - Then comes a hold of 0.2 s, about two axial-shear periods, where T_s = 4L/c_s and c_s = √(μ/ρ).
    Arithmetic: T_s = 0.1035 s.
  - The measurement window is the hold's last 0.1 s, time-averaged.

### 15c. Discretization and solver

- **Mesh: a structured annulus.**
  - An n_r × n_θ × n_z hex grid, split 6 tets per hex by the Coxeter–Freudenthal–Kuhn pattern of
    `hand_built.rs:238-242`. Every cell uses the same body diagonal, so shared faces match "without
    parity flips", across the wrap in θ too.
  - Nodes sit exactly on the circles. The facet error at n_θ = 64 is 0.12 % of radius (arithmetic).
  - The tube builder is the fixture's own, about 60 lines of the same pattern. A dev-dependency on
    `sim-soft` would put 110+ crates into an L0 test graph capped at 120.
  - Why not the SDF mesher: its BCC surfaces sit inside the true surface (96 of 110 cavity nodes, on a
    sphere, `tests/tet10_lame_decision.rs:166-170`).
  - **The ladder is pre-registered**, because stability depends on the split (15i):

    | Grid (n_r × n_θ × n_z) | Tets |
    |---|---|
    | 3 × 32 × 17 | 9,792 |
    | 5 × 48 × 35 | 50,400 |
    | 6 × 64 × 43 | 99,072 |

    A reviewer's model (not kept) found 6 × 64 × 43 stable in the deformed band state (ωΔt 1.846), and
    6 × 47 × 59 not.
- **Element: Tet4 with *selective* ANP.**
  - Only the stiff term λ/2(ln J)² is node-averaged:
    - nodal volume ratio J_a = v_a/V_a, with V_a = Σ V_e/4 and v_a = Σ v_e/4;
    - nodal pressure p_a = U′(J_a) = λ ln J_a / J_a;
    - element pressure p̄_e = ¼ Σ p_a;
    - volumetric force p̄_e v_e ∇ₓN_a.
  - The μ terms stay per element: V_e P_μ(F_e) ∇_X N_a.
  - **Why selective:** Bonet–Burton ANP assumes a split energy Ψ̂(F̂) + U(J) (Joldes, Wittek & Miller
    2009, eqs. 9–13, PMC4477870). Ours is not split. Selective averaging leaves the material exactly
    `sim-soft`'s and the oracle's.
  - **Measured by a reviewer's static periodic-slab model, which was not kept,** so these numbers cannot
    be re-run from the repo (the element alone; dynamics, the nose and the contact law not included):
    - the force is the exact gradient of the energy (6e-16 against autodiff);
    - the stiffness at rest has 6 zero modes;
    - the error against the oracle is **+3.6–5.4 % at 10k, +1.2–1.65 % at 50k, +0.72–1.07 % at 100k**,
      against plain Tet4's +8.6–39 %. It converges at about O(h^1.7–2), and barely moves from ν 0.49 to
      0.495, which is what an element that does not lock does;
    - averaging J_a before U′, instead of U′ at each node, is off by 1.5e-2, so the order matters.
  - **Known artifact:** ring-averaged pressure alternates between adjacent z-levels. The amplitude is
    ±1.1–1.6 % at 10k, ±0.3–0.7 % at 50k and ±0.07–0.6 % at 100k. The alternating state has lower
    energy than the uniform one. It is the checkerboard that Pires et al. 2004 call *"considerable …
    hydrostatic pressure fluctuations"*.
  - *(2026-09-28, §16y)* A motion that keeps every node's volume gets no stiffness from the averaged λ term; only the
    element's μ terms resist it. Under the product's mount an element collapses at the seated tip (§16x). A
    stabilization taking part of the λ term at each element's own volume is built, off by default
    (`ExplicitModel::with_volumetric_stabilization`).
- **Mass:** lumped, ρV/4 per node (the implicit solver's rule, `construct.rs:607-619`).
- **Time step:** Δt = 0.9 · 2/ω_max, with ω_max from power iteration on M⁻¹K through the executor's
  own force phases, **penalty stiffness included**. *(Amended 2026-09-25, §16o: the kinematic law
  replaced the penalty and adds nothing to the step, so Δt = 0.9 · 2/ω_el. Amended again, §16p: with the
  material's viscosity, Δt = 0.9 · 2/ω (√(1 + ξ²) − ξ), where ω² and ξ are the stiffness quotient and the
  viscous damping ratio of the top vector of M⁻¹(K + βC), not the mass damping's ξ below. Computed since 2c
  as 0.9 · 4/(γ + √(γ² + 4ω²)), γ = 2ξω, which holds at ω² = 0 and, where it has a root, below (§16q).)*
  - **The iteration is re-run during loading**, every 500 steps, each from the same fixed start
    *(amended in 2a, 16m: warm-started, it stalled on a lower mode once loaded)*. The step never grows
    by more than 5 % at a time. *(2026-09-28, §16y rule 1: a run that fails at 500 is made again every 50 steps.)*
  - A reviewer's model (not kept) measured, on 6 × 47 × 59: deformation cut the limit to 0.912× its
    rest value, and a Δt fixed at rest with the penalty gave ωΔt = 2.041, which is unstable.
  - The altitude estimate is a cross-check only. The method research measured it 4.4× loose on a
    jittered mesh (not kept).
- **Loading time T:** set by a convergence ladder. Halve T from about 10 axial-shear periods (10·T_s) until the
  band pressure moves by more than 0.5 %, or KE/IE exceeds 5 %.
  - Time scaling stands in for mass scaling. They are equivalent for rate-independent material and
    friction (Abaqus *Getting Started* §13; DERIVED), so the physical density is kept. *(Amended
    2026-09-25, §16p: the material now has a viscosity, so it is not rate-independent; the ladder records
    what loading faster does to the push force and the seated readings.)*
  - **Arithmetic** (100k, ν 0.49):
    - Δt at 0.9 of the rest limit is 42.2 µs on the pre-registered 6 × 64 × 43 mesh. That comes from a
      reviewer's model (not kept).
    - T is 1.04 s plus the 0.2 s hold: about 29k steps at that Δt. It rises to about 32k if loading cuts
      Δt to 0.912×, as on the other mesh.
    - That leaves **3.7–4.1 ms per step within K1**. *(Amended 2026-09-25, §16p: at 10 T_s the material's
      damping takes ×1.23–1.39 the steps at 100k; the ladder's rung is now 0.625 T_s. K1's per-step
      budget is re-derived when step 5 sets K1's loading time.)* *(2d, §16r, sets it for now at the rung:
      2 minutes over the damped 100k tube's steps there.)*
    - The rigid pipeline's whole step is about 0.74 ms at n_env 1 (§6).
- **Damping:** mass-proportional damping α_D·M while loading (NiftySim, Johnsen et al. 2015), with
  α_D = 2·ξ·ω₀, where ξ = 0.05 and ω₀ = 2π/T_s. *(Amended 2026-09-25, §16p: plus the silicone's own
  loss, a deviatoric Kelvin–Voigt viscosity. With mass damping alone the frictional tube fluttered.)*
  - It drags a translating body with force c·m·v (DERIVED). Holding the tube and driving the mandrel
    keeps that small.
  - No dynamic relaxation in any run that K1 or K2 judges.
- **Contact variant (a): nodal-mass penalty** k = s·m_a/Δt², s = 0.5, **primary**. *Superseded
  2026-09-25 (§16n, §16o): its gap could not meet G2 on the tube, and the kinematic predictor/corrector
  replaced it, with friction (Jon's call after the A/B).*
  - Stable only together with the in-loop Δt above.
  - **The gap biases pressure low by about 1 %** (arithmetic, from a reviewer's inner-node
    V_a/A_a ≈ 0.88 mm). Its actual size is measured (15d.1).
  - The gap scales with Δt², so it roughly halves at ν 0.495 and changes along the mesh ladder.
  - **It also grows with contact pressure.** The plan's own inputs give a stiffness of about 264 kPa/mm
    per unit area (arithmetic). That means a gap of about 0.03 mm on the free tube, but about 0.36 mm in
    the confined case (against 1 mm of interference). The confined case is therefore judged in step 2.
  - LS-DYNA's SOFT=1 belongs to the same family; its exact formula is UNSOURCED.
- **Contact variant (b): kinematic projection,** frictionless K2 runs only.
  - A penetrating node is moved to the surface, Δu = −g·n (NiftySim for analytic surfaces; Joldes). Its
    normal velocity is set so it does not re-penetrate.
  - Its contact force for the readout is the time-averaged projection impulse m_a·Δu/Δt².
  - Joldes justified it under dynamic relaxation, which damps high frequencies. A lightly damped
    insertion lacks that, so (b) is measured, not assumed stable.
  - **Friction is not defined for (b)**; the Coulomb push uses (a). *(2026-09-25: the kinematic
    predictor/corrector, (b)'s family, defines it, and is now the law; §16o.)*
  - The rule: (b) replaces (a) only if it meets K2 with less scatter at equal cost. *Amended
    2026-09-24 in step 2's design (16d): (b) cannot replace (a) in the product, since a verdict's
    μ = 0 run must use its frictional runs' law. It stays a K2 diagnostic (15e).*
- **Pressure readout:** force per node over its tributary area, a third of each incident deformed
  boundary triangle (`sim-soft`'s convention, `mesh/mod.rs:433`), area-weighted over the band as
  ΣF_n/ΣA.
- **Friction (the Coulomb push and K6):** Coulomb, with an elastic-slip stick state: a tangential penalty with
  the same k, and a return map on a per-node anchor, as in Abaqus's penalty friction. *Superseded
  2026-09-25 (§16o): friction is kinematic, a sticking node held at its anchor with no elastic slip.*
  - It is rate-independent, so time scaling stays valid.
  - The fallback is viscous regularization, with v_ε ≥ μ_f·f_n·Δt/m to avoid chatter (DERIVED).
    *(Not used, 2026-09-25: 2b's shortfall was flutter of the tube, and the material's own damping
    removed it, §16p.)*
- **The SDF for K2 is pinned:** the mandrel baked into a grid at cell A/20, clamped, with the
  finite-difference gradient (the product's path, §14b). *Amended 2026-09-25 (§16o): the lookup is a
  tricubic interpolant of the grid and its exact gradient. The trilinear lookup's error on the mandrel,
  up to 5.7 µm at A/20, fed energy into the kinematic contact in a long hold; tricubic reads the
  mandrel to 0.053 µm, and 0.9 µm across the seam where its nose meets its shank
  (`tests/sdf_lookup.rs`).*
  - A reviewer's model (not kept) found a grid at A/10 reads the true surface +7.6 µm off, 0.76 % of the
    interference. Trilinear error goes as h², so A/20 should give about a quarter of that (arithmetic).
  - The analytic SDF is the diagnostic (15d.9).

### 15d. Measurements (instruments pre-registered)

1. **Band pressure (K2).** The area-weighted mean over the pre-registered band.
   - **Raw:** against the oracle at radius a.
   - **Gap-corrected:** against the oracle at a − ḡ, where ḡ is the band's mean gap to the *true*
     mandrel surface. That covers the penalty gap and the SDF bias together. *(Since 2026-09-25 only
     the SDF's error is left, §16o.)*
   - Both use the oracle at the band's measured λ_z. The golden values carry p, ∂p/∂a and ∂p/∂λ_z at
     each case, and the corrected reference is their linearization at the measured point.
   - Also reported:
     - flatness, as ring means averaged over pairs of adjacent levels, since single levels alternate
       (15c);
     - the node-to-node scatter;
     - the band's axial stretch against the oracle's λ_z (the validity gate).
2. **Time (K1):** wall-clock end to end, plus per-phase GPU time with §13's instrument (batched passes,
   interleaved configurations).
3. **Energies:** kinetic; internal, ΣΨV from the shared math; external and contact work. KE/IE ≤ 5 %.
4. **Precision (K3):**
   - CPU f32 against CPU f64, in build step 2: band pressure on the 50k mesh (its mean, and each
     pair-averaged ring level, as for flatness in 15d.1), and the Coulomb push on the 10k mesh;
   - later, the GPU f32 against the CPU f64, as a conformance check;
   - TLED's experience: single precision did not hurt convergence, *"no accumulation of errors"* in
     total Lagrangian form (Joldes et al., PMC3003932).
5. **ν sweep:** 0.475, 0.49, 0.495, plus 0.4995 once at 50k as a check (about 4.4× the steps, 15h).
   Record the steps and the error against the oracle.
6. **Mesh ladder:** the three pre-registered meshes. Record error against element size, and its trend.
7. **Coulomb push (μ_f = 0.3):** the mandrel's axial reaction **minus the same run's frictionless
   reaction**, against μ_f·Σp·A over the contact, within 5 %, averaged over the constant-speed phase.
   - The subtraction removes the nose's geometric push. A reviewer's estimate puts it at about 2 % and 6 % of
     the friction force at λ_a 1.1 and 1.3, which is enough to break 5 % unaided.
   - *Amended 2026-09-25 (§16p): the subtraction assumes the geometric push is the same with and without
     friction. It is not quite: on the undamped solver the difference read −1.0 % and −1.6 % of μ_f Σf_n at
     10k; on the damped one it has not been split out.*
8. **The confined stress case:** cased outer wall, all axial motion held, λ_a 1.1, B/A 2, ν 0.49. The
   oracle gives p/μ = 4.1417. This is the regime TLED never validated. The product is free today (§9
   decision 10), so in step 2 it is reported. It becomes a gate, against G2 and 5 %, before any shell or
   bond design is simulated.
9. **The SDF comparison:** the analytic mandrel, against grids at A/10 and A/20, with the
   finite-difference and the analytic trilinear gradients. Record band error and scatter. *(Amended
   2026-09-25: the lookup is tricubic with its exact gradient, §16o; the comparison is the analytic
   mandrel against the tricubic grid at A/10 and A/20.)*
10. **Stiffness scaling** (§5d's shortcut): the same run at μ and 2μ, frictionless and at μ_f 0.3.
    Record whether every force scales by 2 within 2 %. This decides how many runs a verdict needs
    (15h). *(Amended 2026-09-25, §16p: the viscosity scales with μ, as η/μ is held.)*

### 15e. If K2 fails

- **Localize first:**
  - raw against gap-corrected;
  - penalty against kinematic *(moot since 2026-09-25: the penalty is deleted, §16o)*;
  - the mesh-ladder trend: slow convergence points at the element, a plateau at contact or the SDF;
  - analytic SDF against the grid.
- **Element fallbacks, in order.** Each fits the shared math plus a gather phase.
  1. The averaged nodal deformation gradient (Bonet, Marriott & Hassan 2001), or F-bar-patch
     (de Souza Neto, Pires & Owen 2005).
  2. Cyclic J smoothing, which *"suppresses the pressure oscillation"* (Onishi et al. 2017).
  3. Split-energy ANP. This would need the oracle rerun with the split W. *(2026-09-28, §16y: by a research
     report's inference it has no element-level barrier to a change of volume, so it is not taken as a fix for the
     collapse §16x found at the product's seated tip; the stabilization §16y built is energy sampling.)*
- **IANP is not a fallback for the one-material tube:** it changes the rule only where materials meet.
  Which interface rule layered products use is decided in step 6 (15g step 1).

### 15f. The executor trait's phases

The stepping loop calls, per step and in this order:
1. current element volumes;
2. the nodal-volume gather;
3. nodal pressures;
4. element forces;
5. the nodal-force gather;
6. contact, with the pose sample;
7. integration;
8. boundary conditions.

Also on the trait:
- **setup:** upload, plus a force-of-displacement entry that the power iteration uses at setup and every
  500 steps;
- **monitors:** kinetic and internal energy and contact force, reduced on the device and read every k
  steps;
- **snapshots:** a state read for the viewer.

The signatures are settled in the build (§14d).

### 15g. Build order

Each item is one PR with its own tests and a done-when.

1. **`sim-wgsl-gen` and the `sim-soft-explicit` skeleton.**
   - The data layout, with **material parameters per element** (bonded layers need it).
   - The shared math of §14b: the selective-ANP pieces, Ψ, the J check, the Tet4 force, the per-node
     update, the SDF query, the contact law and pose interpolation. **Both neo-Hookean and Yeoh**
     (`base_mold` is Yeoh).
   - Where materials meet: **decided in the build, provisionally.** Each node's pressure uses the
     rest-volume-weighted λ of the elements around it (`ExplicitModel::node_lambdas`).
     - Inside one material that is selective ANP exactly. Across materials the forces stay the exact
       gradient of an energy, to 2e-9 relative on a two-material block (`tests/elasticity.rs`).
     - It is not Joldes' IANP. IANP (PMC4477870, eq. 18) averages each element's own pressure, at its
       own J, into one nodal pressure. The paper's volumetric law is linear in J; ours is not, and for
       ours eq. 18 taken literally would not reduce to selective ANP inside one material.
     - Choosing between them needs a two-layer reference, so it moves to step 6 (bonded layers).
   - **Displacements, as §6 requires.** The shared math takes nodal displacements, computes J − 1 by
     expansion in the displacement gradient, and gathers volume changes, not volumes. It evaluates
     ln(1 + x) as a series of its own, so both backends compute the same expression.
     - Measured on a block 0.12 m from the origin at the band's pressure: the largest nodal f32
       difference is at most 2.0e-5 of that pressure up to ν 0.4995. With f32 positions the same test
       reads 4.6e-3 at ν 0.49 (`tests/elasticity.rs`,
       `f32_nodal_pressure_holds_far_from_the_origin_at_high_nu`).
     - This was caught by the PR #965 review; step 1 had been built on positions.
   - **Poses are interpolated linearly and renormalized, not spherically.** The gap to spherical
     interpolation is 4.0e-6 rad at a 0.1 rad step and scales with the step cubed (`tests/motion.rs`).
   - Compiled at f32 and f64. The freshness test, made to fail once.
   - A conformance test of the shared SDF lookup against `cf-geometry`'s `distance_clamped` and
     `gradient_clamped` in f64. The largest difference over the test points must be ≤ 1e-9 × the largest
     value. The degenerate-gradient threshold it adopts is
     recorded here. *Superseded 2026-09-25 (§16o): the shared lookup no longer matches `cf-geometry`'s
     trilinear one; `tests/sdf_lookup.rs` checks it against the exact mandrel and a quadratic field.*
     - **Adopted: 1e-10**, the CPU path's (`sdf.rs:524`), at both precisions.
     - Measured on two grids. `cargo test -p sim-soft-explicit --test sdf_conformance -- --nocapture`
       prints the margins.
       - An offset grid, over 4,567 points inside the grid and on and past every face: the largest
         distance difference is 1.7e-18 against a bar of 1.3e-11, and the largest normal difference
         3.6e-15.
       - A grid where `cf-geometry`'s clamp, done in world units, rounds past its far faces. Of 4,324
         points, 1,240 get its largest value and +z, and 1,171 more a skewed normal (one gradient probe
         rounds past). All lie within half a cell of a far face or past one. The shared lookup reads the
         face there, as `cf-geometry`'s own doc says a clamped point should. Elsewhere: 2.8e-17 for the
         distance and 1.4e-15 for the normal.
       - Fixing `cf-geometry`'s rounding is a separate change: it moves the fit test's current CPU path
         and the spinal-unit model's.
   - Both crates added to tests-debug shard 3.
   - *Done when:* CI runs the new tests, and the freshness test has failed once on a deliberate edit.
2. **The oracle as golden values, the tube fixture, the CPU executor and the stepping loop.** Designed in
   §16, which splits it into four PRs.
   - The oracle becomes a golden generator (the `sim/L0/mjcf/tests/conformance/gen_golden.py` pattern).
   - **K3 runs here**, including a CPU Coulomb push at 10k. K2 runs on the CPU at 10k and 50k, and so
     does K5's convergence (10k to 50k).
   - **K6 runs on the CPU:** Cattaneo–Mindlin partial slip. It replaced frictional ironing in step 2's
     design (16b): the ironing reference is two deformable bodies, and its published curves could not
     carry a 5 % gate.
   - **Step 2 adds per-direction kinematic constraints to the shared math.** The confined case needs a
     radial constraint and an axial hold, and reproducing a 2D reference in 3D needs an out-of-plane
     condition; step 1 holds whole nodes only.
   - **One Yeoh case** runs, with the oracle extended by the C₂ term.
   - **The confined case (15d.8) and a product-level contact pressure** run on the CPU, and record the
     raw error and the largest gap.
     - The product is free today (§9 decision 10). Before any shell or bond design is simulated, the
       contact law must pass G2 and 5 % there.
     - The fallbacks: a gap-offset penalty, an augmented-Lagrangian update, or a larger s at a smaller
       Δt. *(Settled 2026-09-25, §16n–§16o: none of them; the kinematic predictor/corrector.)*
   - **The product's budget:**
     - mesh `base_mold` locally (the scan stays outside the repo) at the resolution K2 needs;
     - compute its stable Δt with the CPU executor's power iteration;
     - derive its per-run time at K1's per-step cost;
     - measure its surface bias, as the fraction of canal nodes inside the true surface, with and
       without projection, and its element count at that resolution.
     - An explicit step is set by the worst element, so the product mesh's quality sets this number,
       not the tube's.
     - Report the per-press time against D4's 5-minute target (§9 decision 12).
     - If it misses, the speed plan is revised before any GPU work. The quality gates are not
       loosened. *(2026-09-29, §17: GPU work started on Jon's call with this revision still owed; when it is due is
       his.)*
   - **The stop rule (pre-registered; amended 2026-09-24 in 16i, before any data):**
     - Proceed if the 50k gap-corrected error is ≤ 5 % at every corner.
     - Proceed with a flag if it is 5–7 %, and the extrapolation reaches ≤ 5 % at 100k at every corner.
       The model: h = (mean tet volume)^(1/3), and e = C·h^p, with C and p fitted per corner from 10k
       and 50k. Where a CPU run at 100k exists, its measured error decides instead, in every band (16i).
     - ⛔ **Stop before any GPU work** otherwise, or if K3, K4, K6 or the Yeoh case (16h) fails.
   - A reviewer's model (not kept) put the element alone at +1.2–1.65 % at 50k.
   - *Done when:* the stop rule has been applied, with its numbers written here. *(Applied 2026-09-26, §16r:
     proceed.)* *(Jon, 2026-09-26: the quality items come before steps 3–5, K5 first of them; §16s.)* *(Jon, 2026-09-29: steps 3–5
     now, the items still open afterwards or on the GPU; §17. The step control, mine, still comes before step 4.)*
3. **`sim-gpu`: the shared GPU infrastructure is extracted:** the context, chunked submission and the
   contact-list tools.
   - It stays on the workspace's wgpu (27) until a need for a newer version is named. The physics' own
     wgpu entry (§13e) lets it move without Bevy.
   - When it moves:
     - `gpu-probe` migrates;
     - the shaders are validated under the new naga;
     - a binary holding both versions runs on lavapipe (Vulkan) in CI.
   - *Done when:* `sim-gpu`'s suite passes on Metal and in CI.
   - *(2026-09-29, §17a: built; the contact-list tools are extracted when soft-on-soft contact is built, after step
     8's design.)*
4. **`sim-gpu`'s soft executor,** with per-phase conformance against the CPU executor, on lavapipe in CI.
   - *Done when:* every phase's outputs agree, GPU f32 against CPU f32. Per output, the largest
     difference must be ≤ 1e-5 × the largest magnitude.
   - *(2026-09-27, §16u)* The obstacle's fine grid, at the product's 0.0625 mm: two more buffers; a storage binding
     above wgpu's default, so the executor requests the adapter's limit, as `sim-gpu`'s context does (the M4 Pro
     grants 4 GiB; lavapipe's grant is not checked); and a step in the distance where its bricks end, which the
     conformance tests must allow. The done-when above does not yet say how. *(2026-09-27, §16w: at the product's
     band of eight fine cells the fine values are more than four times the default at f32; and the executor gains two
     monitors, the deepest predicted point and the corrections the coarse grid answered, which the GPU must reduce
     too.)* *(2026-09-28, §16x: two more, the contact forces' moment about the obstacle's posed origin and, each step,
     the work the obstacle's motion does against them, its move and turn over a step taken from the pose track at f64;
     the GPU, which runs f32 only, must reduce the one and accumulate the other. The CPU executor, at f32 too, forms
     both at f64: each node's arm from its widened position, the step's sums, and the work, with the move and turn
     from the pose track it keeps at f64 (`contact` in `src/cpu/executor.rs`). Whether sums at f32 would meet the
     done-when's 1e-5 is not measured.)* *(2026-09-28, §16y: a volumetric stabilization, off by default, adds one
     per-element value, κ_e; each node's λ_a − κ_a is formed at f64; and phase 4 reads the element's own dilation,
     which phase 1 writes, so on the GPU it gains a binding or recomputes J. While the product runs the element as it
     is (fit plan U20), the GPU needs κ = 0 only, and its per-phase conformance is at κ = 0; the stabilization stays a
     CPU instrument.)*
   - *(2026-09-30, §17c: built and gated on Metal; the lavapipe run is the PR's CI. G6 on the GPU misses D4, which
     goes to Jon.)*
5. **The experiment on the GPU:** K1, K2 at 100k, the ν sweep, the ladder, the Coulomb push, the stress
   case, the SDF comparison and stiffness scaling. *Stiffness scaling runs first on the CPU, in step 2
   (16i), because the product's budget depends on it.*
   - *Added 2026-09-26 (§16q): f32 does not resolve K6's partial slip (0.56 against 0.03), and K3 has
     compared f32 with f64 only on the frictionless band and on steady sliding. Before a verdict reads a
     frictional seated state in f32, step 5 compares f32 with f64 on the tube's frictional seated p95
     (50k, μ_f 0.3), and step 7 repeats it on `base_mold`.* *(Run in 2d on the CPU, §16r: f32 passes K3's
     0.5 % there; step 7's comparison on `base_mold` stands.)* *(2026-09-26, §16s: and on the seated reading
     that replaced the p95, the 1 cm² patch; step 7 repeats it on that.)*
   - *Done when:* K1–K6 are decided and the results are in this document with their commands.
**Steps 6–9 are an outline.** They are designed in detail after step 2's results, which set the
product's mesh, budget and contact law. Three macro reviews found what that design must settle:
- the per-press time against D4's target (§9 decision 12), which follows from the runs per verdict
  *(2026-09-26, §16r: 0.29–0.61 of D4 at K1's rate with the wall as meshed; projected, more, fit plan U17)*
  *(2026-09-27, §16v: step 7's wall projects nothing; a viscous press at ν 0.49 takes 0.42–0.56 of D4 at K1's rate
  over the insets measured, and 0.59–0.79 at the budget's worst corner)* *(2026-09-27, §16w: before the first press,
  a scan's bake takes about three tenths of D4 at the product's band, once per scan and band)* *(2026-09-28, §16x:
  those figures took the budget's loading; step 7's first run set it at four times that. At h_K2 a press then takes
  0.38 of D4 on the CPU, and 3.8 at four times its elements, which D1's readings may need; a device at K1's per-step
  budget would be slower than the CPU)* *(2026-09-29, §16z: 11.8 at eight times, the size used and not picked, and
  27.5 re-estimating every 50 steps)*;
- the pairing's nominal corner, which D3 judges at: add it as a run, or show push force is linear in
  μ_f *(2026-09-27: §5c's re-sourcing found no source for a nominal; which value D3 judges at is open)*
  *(2026-09-28, §16x: on `base_mold` the frictional push less the frictionless, at 0.18 over at 0.104, reads 1.686
  against 1.731: nearly linear)*;
- Tier 1's accuracy against the solver, and what D3 does if it is poor *(2026-09-27, §16t: Tier 1 reads slice by
  slice, with no rigid path, and the solver runs the fitted pose; step 7 reports the path's own share, the fitted
  pose against the slide, beside Tier 1's error)*;
- modelling each way of holding the device (§9 decision 11): a shell, a mount, or a hand as a soft,
  distributed support. A held closed end is the "no escape" row of §15h *(2026-09-27, §16w: the lowering holds the
  wall by a mount or by a rigid shell bonded to it; a case it slides along, the hand, and bonded layers' interface
  rule move to a PR of their own, before any verdict that uses them)*;
- the product mesher's surface bias and element count (measured in step 2) *(2026-09-26, §16r: the canal nodes
  sit up to 1.25 elements off the true surface, the 5th and 95th percentiles at −0.50 and +0.32; projecting them
  costs the step; fit plan U17)* *(2026-09-27, §16v: meshed from the scan's exact distance with the cut points
  located on it, every canal node lies on the true surface, the step within 0.86–1.03 of the old wall's at the same
  inset, the element size 0.99–1.00 of h_K2)*;
- *(2026-09-26, §16r)* G2 on `base_mold`: no scan grid measured meets it, down to 0.25 mm; step 6's bake sets
  the grid's spacing and pre-smooth against it (fit plan U18) *(2026-09-27, §16u: that was the flood fill's sign;
  signed by parity, with no pre-smooth, the grid's own error meets it at the 5 mm inset at 0.25, 0.125 and
  0.0625 mm. Jon set the bar at smaller insets: 1 % of the inset, never below 0.02 mm, and the product's fine grid
  at 0.0625 mm, fit plan U18)*;
- U3's outcome, and the fact that a contact-guided intruder would need rigid–soft coupling *(2026-09-27, §16t:
  a rigid pose fitted to the slide along the canal asks about a quarter of the room the path as written asks; Jon
  chose the fitted pose, which needs no coupling, and the contact-guided intruder moved to the fit plan's Later)*;
- *(2026-09-25, §16p)* K5's failure: both of D1's readings failed to converge on the tube, so D1's readings or
  the lip radius are revisited before step 7 (§15a; fit plan U16) *(2026-09-26, §16s: diagnosed; Jon replaced
  the seated reading with the most-loaded 1 cm² patch, and K5 passes from 50k to 100k. The lip radius stays with
  the fit plan's Later)*;
- *(2026-09-26, §16s)* the element size D1's readings need on the product: on the tube the frictionless patch and
  the geometric share do not converge from 10k to 50k, and `base_mold` at h_K2 is about the 10k tube's element
  size, though not built in rings. Step 7
  measures D1's readings' convergence on `base_mold`; until then D4's figures at h_K2 (§16r) rest on K2 alone
  *(2026-09-28, §16x: measured and open: four times h_K2's elements or finer, with the element as it is)*
  *(2026-09-29, §16z: with an eight-times wall, each doubling read against its replicate's scatter, not picked: twice
  h_K2's elements or finer, taking h_K2's scatter for twice's, and four times or finer with one as small as four
  times'; from four to eight times only the frictionless patch cannot tell, +5.06 % against 0.71 %. Read at 5 % flat,
  eight times or finer)*;
- *(2026-09-27, #973's review)* D1's push at a low friction. K5 judged the push's peak with friction only at μ_f
  0.3. Without friction the peak moves with the mesh, which ripples it (§16s): at 100k and λ_a 1.1 it reads 30 %
  above the 10 mm reading, and on the mesh 4× finer along the tube 8 % above that mesh's (arithmetic on §16s's
  tables). How the peak moves at a low μ_f is not measured. Step
  7's convergence check of D1's push includes the lowest μ_f of the pairing library, and D1's push is decided with
  that number (Jon, 2026-09-26) *(2026-09-28, §16x: at μ_f 0.104 the peak moves 8.4 % over the first doubling of
  h_K2's elements and 2.2 % over the second, read over 1 mm of travel; for Jon)* *(2026-09-29, §16z: −1.12 % over the
  third)*;
- *(2026-09-27, §16t's review)* **D1's push on a turning path.** The fit plan defines the push as −Σ fᵢ · (dxᵢ/ds).
  On a path that turns the scan, that takes the twist on the scan as well as the force. The monitors reduce the
  resultant contact force but not its moment, and the tube probe reads the resultant's component along the tube's
  axis. Whether the push is reduced in the executor (and so on the GPU, steps 3–5) or computed from the snapshots'
  per-node forces is not decided *(2026-09-28, §16x: the executor, which accumulates the obstacle's work each step)*;
- *(2026-09-27, §16t)* **What reading of the fitted pose brings the contact-guided scan forward.** Step 7 prints
  the sideways force and twist the wall puts on the scan; no reading of them is yet set that would. Step 7's design
  sets one before its runs *(2026-09-28, §16x rule 10 set it: a scan free to move changing D1's reading by more than
  5 %. On `base_mold` it changed it by −1.0 and −2.8 %, so no recommendation; the turn about the path was held)*;
- *(2026-09-25, §16p)* friction above μ_f 0.3: the damped tube's Coulomb push fails 15d.7 at μ_f 0.6, and §5c's ranges
  reach 2.0 *(re-sourced 2026-09-27: on skin to about 1.2, tacky analogs above 2; §5c)*. Before a verdict is trusted
  at such a corner, its Coulomb push is checked there (§7 rung 4's self-consistency check), and the material's damping
  (fit plan U15) is settled. Above f ≈ 1.0 the half-space's own sliding is unstable at ν 0.49 (§16p), so there a finer
  mesh need not converge;

- *(2026-09-28, §16x)* ν under the mount: each of D1's readings' change per doubling of K on the product (ν 0.49,
  0.495 and 0.4975), the silicone's own K being unknown. Which ν verdicts read at, and whether ν becomes a corner as
  μ_f is, trades speed against quality, so it is Jon's call;
- *(2026-09-28, §16x)* the confined case for a hold on the skin alone: the tube with its outer wall held whole and
  its ends on frictionless plates, the mandrel through it, against the confined oracle. The mount holds less and has
  no oracle of its own *(§16x: the tube passes, which settles the shell's side; the mount's stays open, since rule 2,
  the check on the product's own confinement, did not settle, and the element collapsing at the tip appears under
  the mount)* *(2026-09-29, §16z: still open; the size rule picked no size)*;
- *(2026-09-28, §16x)* the element collapsing near the product's seated tip under the mount: the most-compressed
  element reaches 9–16 % of its volume while its nodes' averaged volume, from which selective ANP takes the pressure,
  stays within 12 % of its rest, at every size run, and the step falls to 0.06–0.40 of the rest step. With
  the loop's re-estimate every 500 steps (§15c) one run inverted an element and went non-finite; every 50 it ran
  through, at a cost not measured. Which element limits the step, whether the collapse moves D1's readings and so the
  element size they need, and what the re-estimate interval should be are not measured. My recommendation to Jon:
  settle this before D1's element size is read again *(2026-09-28, §16y: on the frictionless runs the step is set at
  the most-compressed element at h_K2 and twice its elements, and at one near it at four times. Resisting the
  collapsing elements alone moves the frictional readings at four times h_K2's elements by at most 1.5 %; the
  frictionless patch moved +6.44 % at twice h_K2's elements, and at four times its comparison did not stand by the
  cut, its last two runs reading +5.06 and +5.07 %. The element as it is stays (fit plan U20, §16y's call); the next
  PR reads D1's size with it and re-reads rule 2 there. Where the loop's re-estimate every 500 steps fails, a run is
  made again every 50 steps, §16y rule 1)* *(2026-09-29, §16z: at eight times h_K2's elements every corner's collapse
  cleared with the masked runs, each deciding reading's masked change within 5 %, at most +4.11 %: U20 stands there,
  the size used and not picked; at twice h_K2's elements §16y's +6.44 % was past the bar, and at four times it was not
  judged)*;
- *(2026-09-28, §16y)* the product loop's step control: rule 1's re-run every 50 steps lives only in the probe, and
  the solver's loop re-estimates every 500 steps with no retry. Before step 4, the product loop needs one of a retry,
  a fixed 50 steps, or tracking the step, costed in G6 *(2026-09-29, §16z: at eight times h_K2's elements no run needed
  the retry, and a fixed 50 steps took 2.34 times the time, 27.5 of D4 against 11.8; tracking not costed)*
  *(2026-09-29, §17a: step 3's recorder assumes a step's values are known on the host when it is recorded, so tracking
  the step on the device would need another design)* *(2026-09-29, §17b: set, the loop's re-estimate every 500 steps
  and §16y rule 1's re-run, which stays with the code that runs a press)*;

**Starting now, in parallel with steps 1–2, needing no solver:**
- U3's geometric check *(done, §16t)*;
- friction sourcing for the product's pairings (§5c: the dominant input, uncertain by more than 10×) *(a round
  done 2026-09-27, §5c)*;
- the comfort-limit research (step 9) *(a round done 2026-09-27, fit plan U1)*.

6. **`sim-soft`: lowering, obstacle baking, and the pairing library.**
   - Lowering and obstacle baking come from `cf-sim-research`. *(2026-09-27, §16u: the bake is new, in
     `sim_soft::obstacle`, and replaces `cf-sim-research`'s rather than moving it.)*
   - The pairing library covers surface × surface × lubricant, with fresh and depleted ranges (§5c).
   - The boundary options (§9 decisions 10–11):
     - free;
     - cased, as a radial kinematic constraint;
     - bonded, through per-element materials, with the interface rule chosen against a two-layer
       reference (step 1);
     - mounted at the closed end;
     - hand-held, as a soft, distributed support.

     *(2026-09-27, §16w: Jon moved the hand-held support and the bonded layers' interface rule to a PR of their
     own, before any verdict that uses them: the first needs a hand's stiffness, which is not sourced, and the second
     a two-layer reference. Step 7's first run is mounted at the closed end. The case joins them, an engineering
     call: as a radial kinematic constraint it let the cased tube turn and its case open (§16n).)*
   - The F3 conformance test of `Material` against the shared math lands here, since `sim-soft` takes
     the dependency. *(2026-09-27: step 6 lands as three PRs: the bake and U18's grid (§16u, #975), which took
     the dependency; the wall's canal surface (fit plan U17); and the
     lowering, which carries the boundary options, the pairing library, this test and the items carried from
     Phase 1.)* *(2026-09-27, §16v: the wall's canal surface is settled. Step 7's wall is meshed from the scan's
     exact distance with its cut points located on that distance, which the lowering builds; and the mesher's
     Parity Rule is fixed.)* *(2026-09-27, §16w: the lowering meshes and lowers it; its caller builds its body.)*
   - **U3** (fit plan) is settled geometrically before step 7: with no inset, the rigid path already
     asks 8.3 mm of room. Its swept volume is checked against the cavity. If the path is at fault, step
     7's verdicts would measure that and not the fit. The prescribed path is then replaced by an intruder
     that is force- or velocity-driven and guided by contact (the replaced plan's Phase 4 item).
     *(2026-09-27, §16t: checked as the room the path asks at the cavity wall's nodes over 64 poses. The path is
     at fault, and Jon chose a prescribed pose instead: each pose is the rigid motion closest, in least squares,
     to sliding along the canal. The contact-guided intruder moved to the fit plan's Later. Step 6's lowering
     builds the fitted pose, and settles what the measuring copy does not: the pose before any of the scan is
     inside the device, where the fit has nothing to fit (the probe starts a 64th of the way in); and one fit per
     scan, since the path does not depend on the inset.)*
   - Also carried from the fit plan's Phase 1:
     - the t = 0 intrusion and its pre-roll *(measured on the old path; again on the fitted pose, §16t)*;
     - the outer-skin pin's 646 interior vertices;
     - the path's time sampling *(of the fitted pose, against a bar, §16t)*.
   - *Done when:* the bake matches `cf-sim-research`'s within 1 % of a grid cell at every grid point, and
     each boundary option has a test. *(2026-09-26, §16r: the old bake is coarser than every grid 2d
     measured, and none of those meets G2 on `base_mold`; fit plan U18.)* *(2026-09-27, §16u: the old bake's sign
     is wrong near the surface, so the bar is instead that the bake reads each sample's exact signed distance (its
     tests) and, on `base_mold`, its own error at the scan's points is within G2's bar
     (`the_bake_on_the_product_scan`); that bar is an engineering call, and Jon delegates these.)*
     *(2026-09-27, §16t: and the lowered
     fitted pose passes the measuring copy's known-answer tests, and on `base_mold` reproduces the probe's room at
     each pose, checked locally as the 8.285 mm was.)* *(2026-09-27, §16u: the probe's room reads the old path's
     scan grid, so that check reads the probe and the lowering through the same grid; and the lowering sets the
     bake's band and bakes the fine grid at 0.0625 mm (Jon, 2026-09-27). Deeper than the band, as at the t = 0
     intrusion, the coarse grid answers, and its error there is not measured.)* *(2026-09-28, §16x: done, on the bars
     as re-barred. The bake reads each sample's exact signed distance, and on `base_mold` its own error at the scan's
     points is within G2's bar (§16u); the lowered fitted pose passes the known-answer tests and reproduced the
     probe's room at each pose through one grid (§16w); and the options this step kept each have a test, the wall
     held by nothing, by a mount and by a rigid shell (§16w). The options it moved are step 6b.)*
   - **6b** *(2026-09-28, §16x)*: the hand as a soft, distributed support, bonded layers' interface rule, and a case
     the wall slides along, moved out of step 6 (§16w), are a PR of their own. It lands before any verdict on a design
     that uses one of them; step 7's first run uses none.
7. **`base_mold` on the new solver.**
   - G6 is 5 minutes per run, where a run is one simulation (fit plan D4). `base_mold` stays outside
     the repo. *(D4 is per press, §9 decision 12 and fit plan U13; §16r reports it so.)*
   - The **lip radius** (fit plan "Later"): the lipped cavity built in `cf-design`, then sharp against
     rounded on the same scan.
   - **Tier 1** (the per-slice estimate, §6) is built and checked against the solver on `base_mold`. D3's
     search uses it to pick candidate insets, so full verdicts run at only 1–2 insets. *(2026-09-27, §16t: beside
     Tier 1's error, the path's own share: the fitted pose against the slide.)*
   - *(Added 2026-09-27.)* Beside D1's readings, each run prints:
     - the loaded surface area inside the winning 1 cm² patch. The patch is a ball in space, so it also holds any
       other loaded surface within its radius of its centre (§16s: 1.0103 cm² on a bore of 10 mm radius);
     - the sideways force and the twist the wall puts on the scan, which show how far the walls would push it off
       the fitted path (§16t);
     - which of the obstacle's grids answered each surface node's lookups, and the deepest predicted point; and on
       the frictionless run, the work the contact does over the hold (§16u). G2 is judged in the run, against the
       scan's exact signed distance (the bake's distance and sign) and G2's floor: the executor's penetration
       monitor reads the grid, so it cannot see the grid's own error. At which steps the run reads it is step 7's
       design *(2026-09-28, §16x rule 8: at every monitor read, over every surface node the grid puts inside the scan
       or less than the band outside it)*.
   - *(2026-09-27, §16w.)* The first run is mounted at the closed end (Jon), which confines the material there and
     moves it toward §15h's cased rows, where ν matters: step 7 reads its verdict's readings at ν 0.49 and 0.495
     *(2026-09-28, §16x: and 0.4975; which ν verdicts read at is Jon's call)*.
     Step 7 sets the loading time and the mass damping (§16w's run took the tube's rules), and re-reads the travel,
     which the join, past the centreline's end, lengthens. G2 is judged against the pose the executor ran. A run whose
     corrections read the coarse grid says so, and a run whose deepest predicted point passes half the band re-sets
     it by §16w's rule and bakes again before its readings stand *(2026-09-28, §16x: a run whose corrections all read
     the fine grid stands; past half the band is a warning, and a correction the coarse grid answered re-sets the
     band)*. The fitted pose is the first obstacle that turns, so
     step 7's G2 also reads §16o's frame carry, which no test isolates. D1's push on the fitted pose, which turns,
     needs the twist on the scan; whether the executor reduces it or the snapshots' forces give it is step 7's to
     decide (the list for steps 6–9) *(2026-09-28, §16x: the executor)*. The confined case that gates a mount (§9 decision 11) passed with the tube's
     outer wall and every node's axial motion held (§16n); whether that covers a mount holding the skin alone is step
     7's to settle before its first verdict *(2026-09-28, §16x: a tube held on its surface alone, its outer wall
     whole and its ends on frictionless plates, meets the confined oracle, which settles a shell. The mount's side stays
     open: rule 2, its check on the product, did not settle, and the element collapsing at the tip appears under the
     mount)* *(2026-09-29, §16z: still open; read again with an eight-times wall, the size rule picked no size)*.
   - *(Carried here from earlier sections, 2026-09-27.)* Step 7 reads the room on its own wall (§16v); reads the
     product's own seated window (§16r); repeats f32 against f64 on the 1 cm² patch (step 5's note); and the old
     path, `slide_pose_at` with it, retires after it (§16u, §16w).
   - *Done when:* the fit plan's G1–G3 and G6 have numbers on `base_mold`. *(2026-09-27: and D1's readings'
     convergence there, at the product's element size and at the lowest μ_f; the list above.)* *(2026-09-28, §16x:
     G1–G3 and G6 have numbers at the 5 mm inset; D1's readings' convergence is open, four times h_K2's elements or
     finer, so step 7 is not done.)* *(2026-09-29, §16z: read again with an eight-times wall, D1's size is not picked:
     twice h_K2's elements or finer by the size rule, taking h_K2's scatter for twice's, and eight times or finer read at
     5 % flat. G6 at eight times is 11.8
     of D4 on the CPU. Step 7 is not done.)*
8. **The soft-on-soft contact design** (§9 decision 9): a design step with its own research round, not
   code.
9. **Validation and limits:** §7's rungs 2 and 5, and the comfort limits (fit plan U1, §8). The limits
   research does not depend on the solver, so **it starts now**, in parallel with steps 1–2. The 5.4 N
   anchor limits getting in at all; it is not a comfort limit.
   - *Done when:* rung 2's benchmarks are within their published tolerances, at least one rung-5
     experiment is reproduced within its measured scatter, and each comfort limit is either sourced or
     has a fallback Jon has decided (fit plan U1).
   - Until then, the fit test's verdict is labelled unvalidated. D2 already makes it advisory.

**Where the runs live in CI:**
- tests-debug runs a short sanity run on the 10k mesh (a few hundred steps: finite energies, J > 0). It
  makes no accuracy claim.
- K2 at 10k goes in tests-release, **named in its explicit list**, because a release-only test runs in
  no CI job otherwise. It asserts an error ≤ 7 %; the element alone measured up to 5.4 % there.
- K1 and the 50k/100k runs are recorded experiment commands, not CI. `#[ignore]`d runs have skipped
  silently here before, so their results are kept with their commands.

### 15h. Consequences for the product, and what is not known yet

- **Runs per verdict.** A run is one simulation. §2 and §5d want an interval across friction,
  stiffness and Mullins corners.
  - **Decided (fit plan U11):** verdicts use the virgin state, the stiffest and so the conservative one.
    The Mullins-conditioned state is reported once per design, not run for every verdict.
  - **If stiffness scaling holds** (15d.10), a verdict is **3 runs**: the pairing's low and high μ, plus
    μ = 0 for the geometric share of push force (§2). That is about 6 minutes at K1's rate. *(2026-09-26, §16r:
    on `base_mold` as meshed at h_K2, a press takes 0.29–0.61 of D4 at K1's rate.)* *(2026-09-28, §16x: at the
    loading D1's readings need, four times the budget's, a press takes 0.38 of D4 on the CPU at h_K2 and 3.8 at four
    times its elements; stiffness scaling holds on the product, so a verdict stays three runs.)* *(2026-09-29, §16z:
    11.8 at eight times its elements, the size used and not picked.)*
  - **A D3 search** of 3–4 full verdicts would take 18–24 minutes, against D4's 15 (arithmetic). Tier 1 therefore pre-filters the search (step 7), so full verdicts run at 1–2 insets. *(2026-09-26, §16r: on `base_mold` as meshed at h_K2, 3–4 verdicts take 4–12 minutes at K1's rate, within 15; with the canal nodes projected at a floor of 0.5 and the viscosity, 15–21 minutes (fit plan U17); arithmetic.)* *(2026-09-27, §16v: step 7's wall projects nothing; at ν 0.49 with Ecoflex's viscosity, a press takes 0.42–0.56 of D4 at K1's rate over the insets measured, so 3–4 verdicts take 6–11 minutes, and at the budget's worst corner 9–16; with Tier 1 picking 1–2 insets, 3–8 at the worst corner; arithmetic.)* *(2026-09-28, §16x: at four times the budget's loading, full verdicts at 1–2 insets take 0.13–0.25 of the search's 15 minutes on the CPU at h_K2, and 1.3–2.6 at four times its elements.)* *(2026-09-29, §16z: 3.9–7.9 at eight times its elements, the size used and not picked.)*
  - **If stiffness scaling fails,** each stiffness corner doubles the friction runs: 5 runs.
- **The regime per application** (from the oracle's confinement table, §5b amended):

  | Application | Outer wall / axial escape | Regime |
  |---|---|---|
  | The sleeve, free wall | free | ν barely matters |
  | The sleeve, cased with an open entry | the material escapes axially | ν matters mildly |
  | Near a closed, cased base | no escape | pressure ∝ K |
  | An O-ring in its gland | fully confined | pressure ∝ K. ν must come from the measured K; Abaqus's K/μ 1 000–10 000 is ν 0.4995–0.49995 |
  | Garment, footwear, grasping | free, or thin | ν barely matters |
  | Surgical insertion (needle, catheter) | tissue around it | not assessed |
  | `base_mold`, the product, today | free outer wall (§9 decision 10); held in a shell, on an arm mount or in the hand, by design (decision 11) | free or hand-held: ν barely matters. A shell or a mount at the closed end moves toward the cased rows *(2026-09-27, §16w: step 7's first run is mounted; the hand and a sliding case wait on a PR of their own)* |

  At ν 0.4995 an explicit run needs about 4.4× K1's steps (√(1001/51), arithmetic), roughly 9 minutes
  at K1's rate. **The O-ring class is outside K1's sizing**, and needs its own budget or a mixed
  treatment when that application is reached.
- **Not known yet:**
  - whether the alternating pressure pattern persists in a damped explicit run *(2026-09-28: not measured on the
    product; §16x records a different, element-level observation there, an element collapsing at the seated tip)*;
  - the GPU's per-step cost at 100k;
  - the in-loop Δt's cost in steps *(2026-09-25: measured on the tube, §16p: the loaded step factor 0.977,
    and the damping's ×1.06–1.39)*;
  - the LS-DYNA formula marked UNSOURCED. No choice here depends on it.



### 15i. How the plan was checked

Two cold reviewers worked against criteria written beforehand.
- **One reviewed the physics and numerics.** It built the element in a scratch JAX model: exact Hessians,
  Lanczos eigenvalues, a static periodic slab.
- **One reviewed the plan and its fit to Jon's requirements.**

Their findings were checked and folded in above. The physics reviewer's model was not kept, so its
numbers are labelled and cannot be re-run from the repo.

**The D7 search** looked for a Rust engine that could come inside. The web was not searched, because the
session's search budget was spent. A keyword search of crates.io found nothing that is a validated GPU
explicit-FEM solver with contact:
- `fenris` is CPU-only, last released in 2023;
- `gizmo-physics-soft` uses wgpu, but is a game-engine FEM/XPBD component;
- `oxiphysics` and `tpt-fem` are CPU-only.

## 16. Build step 2: the design

Written 2026-09-24, before any step-2 code. §15g step 2 lists what step 2 must do. This section says how,
and records the decisions made in designing it. How it was checked is in 16k.

### 16a. What constrains it

Step 1 was built against its bullet list and missed §6's rule on displacements (15g step 1). So these are
the sections that bind step 2, and the cold review (16k) checks the design against each of them:
- **§6:** displacements are the state, J − 1 is computed by expansion, and forces go to per-element slots
  gathered at the nodes, with no atomics.
- **§13d:** the shared math stays loop-free, never relies on NaN, and puts no `vec3` in shared structs.
- **§14a, §14b:** the CPU executor uses rayon on native and runs sequentially on wasm32. The stepping loop
  owns the phase order, the stable step, the monitors and the stop rule. Kinematic boundary conditions
  are shared math.
- **§14c, F1, F2:** L0's dependency caps (100 release, 120 test); wasm32 must build; CI runs a crate's
  tests only if a list names it; code behind a feature is neither coverage-measured nor doc-checked.
- **§14d, §15f:** the trait is phase-level, and the order is written once, in the loop. The GPU never
  reads back except on an explicit read. Monitors are reduced on the executor and read every k steps.
- **§15a–§15d:** the case, the kill criteria, the validity gates, the discretization and the instruments.
- **§15g steps 3–5:** what the GPU executor will need from the trait.

### 16b. K6: Cattaneo–Mindlin partial slip (amended 2026-09-24)

*Amended 2026-09-25 (§16p): the tube's material now has a viscosity, which the elastic closed forms do
not; 2c sets K6's viscous time, or shows its loading slow enough for it not to matter.*

*Amended 2026-09-25 (§16q, 2c as built): K6 runs elastic, η = 0; the stick zone is read from the friction
deficit μ_f f_n − f_t, not the ratio below; the legs start and stop over the block's shear period, with a hold
after each; and the press ends near a, not at it. K6's runs are §16q's. Three of the rules below were applied
as follows, the first two written after the data:*
- *the rate ladder stops when c and m move by at most the budget table's 0.005a for the rate, not by the unit
  test's 0.00076a, which the ladder's text names;*
- *companions and rungs are compared at the same load fraction while the load moves, and within 0.02 of each
  leg's largest judged fraction by each row's mean error: matched by fraction there, the same pair read 0.0055
  or 0.0080, as a tie fell;*
- *R = 200a at the plan's speeds differs by 0.0103, so K6 is judged on it too (0.0218). Its legs travel half
  as far, so at those speeds it is also a faster loading; at half its rate it differs by 0.0089.*

*Amended 2026-09-25 (§16o): the contact law is now kinematic, with no elastic slip, so the penalty's
compliance drops out of the error budget. The reason for f64 does not: the stick test compares one
step's tangential drift with μ_f times one step's normal correction, and that correction is
s·(1.8/1.655)² ≈ 0.59× the penalty's gap (arithmetic), so f32 anchors are coarser against it than
before (16m). 2c re-derives both.* *(2c, §16q: the law's compliance is gone from the budget, and f64
stays: f32 reads K6's stick zone 0.56 off.)*

**Why ironing was replaced** (Jon approved the swap, 2026-09-24). Read at the source, arXiv 1903.05859
§5.1:
- The die is 1000× stiffer than the slab (E 1000 against 1 N/mm², ν 0.3 for both; Fig. 5).
- The force histories are published only as plots, and only on the coarsest mesh (m1, Fig. 8). Read
  off the plot, the smoothest curve's horizontal force swings about ±3 %, the basic curve's about ±14 %.
  A 5 % gate could pass or fail on how the plot is read.
- The paper states plane strain for its other two examples, not for this one.
- While the die slides, horizontal over vertical force reads about 0.20, which is μ_f. That is full
  slip, which the Coulomb push (15d.7) already checks. The stick state lasts a few load steps.

Ironing stays in §7 rung 2, as a step-9 benchmark reported with these limits.

**The reference.** A rigid cylinder is pressed into an elastic block, then pushed sideways with less
than μ_f·P, in plane strain. Part of the contact sticks and part slips.
- **Loading:** the stick zone's half-width is c = a·(1 − Q/(μ_f P))^½
  ([Lorez & Pundir, arXiv 2412.14972](https://arxiv.org/pdf/2412.14972), eq. 33 ✓).
- **Unloading** by ΔQ from the peak: the zone that has not slipped back has half-width
  m = a·(1 − ΔQ/(2μ_f P))^½ ✓. This is Mindlin and Deresiewicz's rule in the half-plane form of
  [Andresen & Hills, arXiv 1911.07789](https://arxiv.org/pdf/1911.07789), eqs. 10–12, with the
  half-plane's a ∝ P^½. It tests the anchors' memory.
- **Plane, not the 3D sphere:**
  - *"The Cattaneo–Mindlin solution is exact only when applied to the plane form of the contact"*
    (Dini & Hills, J. Tribol. 131, 2009 ✓). The 3D form neglects a transverse slip. They call the
    error small *"for contacts where the contacting materials have only low or moderate Poisson's
    ratios"*, which does not include ours.
  - In the plane form, the tangential displacement is defined only up to a constant (Popov et al.,
    *Handbook of Plane Contact Mechanics*, eq. 2.7). So the observable is the stick zone, not a
    force–displacement curve.
- **The coupling.** The solution assumes the normal and tangential problems are uncoupled. A rigid body
  on an incompressible one is such a case (Popov, Heß & Willert, *Handbook of Contact Mechanics*, 2019,
  eq. 4.3 ✓).
  - At ν 0.49 a rigid body leaves a Dundurs β = (1 − 2ν)/(2(1 − ν)) = 0.0196 (derived), so
    μ_f/β = 15 at μ_f 0.3.
  - No published source found quantifies the error there. The only coupled number found is at
    μ_f/β = 1: gross slip at 0.9463·μ_f·P instead of μ_f·P (Wang et al., Tribol. Lett. 70:98, 2022, a
    rigid sphere).
  - **Measured instead ✓:** `uv run docs/soft_contact/cattaneo_mindlin_reference.py`, a plane-strain
    boundary-element solution of this contact with the coupling included. At ν 0.49 the coupling moves
    the stick zone's half-width by at most 0.005a while loading and 0.0025a while unloading, on two
    grids (a/200 and a/400). It also shifts the zone off-centre by up to 0.05a. The script checks
    itself: it reproduces both closed forms at β = 0, and at ν 0.475 the effect grows to 0.020a.

**The case.**
- **The block:** §15b's neo-Hookean (μ = 23 kPa), ν 0.49, μ_f 0.3.
  - It is 20a wide and 10a deep, with the bottom held and the sides free.
  - It has one element layer in y, with every node held in y (plane strain, 16d). Holding y does not
    make the front and back rows of nodes agree (the Kuhn split is not symmetric front to back), so
    both rows are read, and the worse one is judged.
  - A depth of 10a meets the half-space rule in Hojjati-Talemi et al. 2012 ✓ (half-thickness ≥ 10a).
    They cite Fellows et al.: at 3a the edge stress rises by up to 20 %.
- **The cylinder:** rigid, R = 100a, baked into a grid at cell h/2. Its pose carries a fixed 30°
  rotation about its own axis. That changes nothing physical, and it puts the contact law's frame
  rotations under test, which no other friction gate does.
  - Its peak pressure is p₀/μ = a/(R(1 − ν)) = 0.020 (arithmetic, plane Hertz with E* = 2μ/(1 − ν)), so
    strains are about 2 %. The reference is small-strain.
- **The mesh:** element size h = a/50 over |x| ≤ 1.5a and the top 0.5a, graded to about a/2 at the
  far boundaries. Hojjati-Talemi et al. used 51 elements across the half-width (5 µm on a = 254 µm),
  and their a came within 0.39 % ✓.
- **Precision: f64, with the world and body frames' origins at the first point of contact.** A
  slipping node moves about 1e-8·a to 1e-7·a per step, below f32's spacing even for coordinates of
  order a (1.2e-7·a) (arithmetic). f32 is K3's question, asked at the product's scale. On the tube a
  slipping node moves about 4.7 µm per step (0.112 m/s × 42.2 µs), about 630 of f32's 7.5 nm spacings
  at 0.12 m (arithmetic). An f32 run of K6 is reported, not judged.
- **Loading:** displacement-driven:
  1. press until the contact half-width is a, and hold;
  2. move the cylinder sideways until Q = 0.8·μ_f·P;
  3. move it back until Q = 0.
- **The rate is set by its own ladder,** not by KE/IE. The indentation dominates the internal energy,
  and the tangential phases add only about 6 % to it (arithmetic, a reviewer's estimate), so KE/IE ≤ 5 %
  would allow kinetic energy as large as the whole signal. The rate is halved until c and m move by no
  more than the readout's resolution (below), at every sample.
- **Readouts,** from each node's normal and tangential contact forces averaged over each monitor
  interval (16e's `accumulate`):
  - P and Q, the forces' resultants.
  - a: each edge of the contact is where the averaged normal force squared, extrapolated linearly,
    reaches zero. Hertz pressure goes as the square root of the distance to the edge.
  - **The stick zone's edges** are read from the ratio f_t/(μ_f f_n). f_t is the component along the
    load's direction, and along its reverse while unloading. A slipping node's ratio is exactly 1, and
    a sticking node's approaches 1 as the square root of its distance to the edge.
    - **The rule is settled in 2c by a unit test,** not here. *(Settled, §16q: the friction deficit.)* The test feeds the closed forms, sampled
      at the mesh's own nodes over many mesh offsets and load fractions, to the readout. The largest
      error it reads is the readout's resolution, and it must be ≤ 0.005a.
    - The candidate is to extrapolate (1 − ratio)² linearly to zero from the last two sticking nodes,
      as a is found. Interpolating the ratio itself was checked on the closed forms by a reviewer and
      read up to 0.02a wide.
    - If no rule reaches 0.005a, the budget below is revised before K6 runs.
    - c (and m) is half the zone's width. The zone may sit off-centre (by up to 0.05a at ν 0.49, as
      above), so a width is read, not a distance from x = 0.
  - Reading the anchors instead, so that a node sticks when its anchor did not move, was rejected. At
    f32 a slipping node's drag is below one spacing, so it reads as sticking. At f64 a node one element
    inside the edge is within about 2.5e-5·h of the cone (a reviewer's estimate), and any vibration
    flips it. And an anchor
    that is not dragged reads as sticking everywhere.

**The criterion (pre-registered):**
- **Loading:** at every sample with 0.2 ≤ Q/(μ_f P) ≤ 0.8, |c/a − (1 − Q/(μ_f P))^½| ≤ 0.03.
- **Unloading:** at every sample with 0.2 ≤ ΔQ/(μ_f P) ≤ 0.8, |m/a − (1 − ΔQ/(2μ_f P))^½| ≤ 0.03.
- **What the 0.03 must cover:**

  | Source | Size | Referent |
  |---|---|---|
  | The coupling at ν 0.49 | ≤ 0.005a | the boundary-element referent above ✓ |
  | The penalty's compliance | ≤ 0.002–0.004a | a reviewer's superposition, with k/A from §15c's stiffness (not kept) |
  | The readout | ≤ 0.005a | 2c's unit test (above) |
  | The loading rate | ≤ 0.005a | the rate ladder |
  | Finite strain | measured | the companion run below |
  | The finite domain | measured | the companion runs below |
  | The element | not known | 2c also runs a/h = 25 and records the difference |

  *Amended 2026-09-25 (§16q): the penalty's compliance went with the penalty (§16o). The readout reads the closed
  forms, sampled at the nodes, to 0.00076a (2c's unit test). A run's nodal forces are not those samples: a review's
  half-plane model with contact held at the nodes read 0.0056a at a/h 50 through the same readout (not kept), and a
  run's readout cannot be separated from the element's error. The a/h 25 companion measures the two together.*

- **Two companion runs, each changing one thing:**
  - R = 200a (strains about 1 %), for finite strain;
  - a block 30a wide and 15a deep, for the finite domain.

  Each must agree with the main run within 0.01a at every sample. If one does not, that effect is not
  negligible, and K6 is judged on that companion.
- **The gate must fail on purpose first** (2c's done-when). Three mutations of the contact law:
  - an anchor that never releases;
  - an anchor that is not dragged while slipping;
  - a friction limit 10 % high.

  Arithmetic for two of them, at Q/(μ_f P) = 0.8: an anchor that never releases gives c/a = 1, not 0.45.
  A 10 % limit error gives 0.52 against 0.45.
  - An anchor that is not dragged gives the correct forces while the load rises, by construction, so
    only unloading can catch it. Its nodes stay at the limit in the old direction and never slip back.
  - If a mutation does not fail K6, the gate is revised before K6 is judged, and the revision is
    recorded here.
  - *2c (§16q): each fails K6, but not as expected here. The never-releasing anchor reads 0.32 off, not
    c/a = 1: the deficit readout counts a node held past its limit as slipping. The undragged anchor fails
    while loading too: the push never reaches its peak, holding near 0.47 μ_f P.*

**In CI:** a coarse version (a/h = 12, f64, tolerance 0.1) joins tests-release. It must fail under the
two anchor mutations. At a/h = 12 it cannot see a friction limit 10 % off (a 0.075 shift). That is
covered at the law's level by
`the_friction_force_is_the_tangential_part_and_reaches_the_cone_only_when_slipping` (the kinematic law's,
since 2026-09-25). The full runs are recorded commands.

**What K6 does not test:**
- *added 2c (§16q):* the material's viscosity (K6 runs elastic; the product does not);
- *added 2c:* f32 (it does not resolve K6);
- *added 2c:* the readout on a run apart from the element;
- sliding at large deformation: the Coulomb push covers full slip in 3D, against the solver's own
  pressures;
- a tangential load whose direction turns (3D stick–slip);
- soft-on-soft friction (§9 decision 9).

### 16c. Four PRs

Step 2 is split into four PRs. 2a holds the solver and its trait, which steps 3–5 build on, so that
code is reviewed on its own. 2b–2d add fixtures, readouts and runs, and 2d adds a local lowering in
`cf-sim-research`.

| PR | Holds | Done when |
|---|---|---|
| **2a. The CPU solver runs the tube** | Per-direction constraints and the readout pieces in the shared math (16d). The executor trait, the CPU executor and the stepping loop (16e). The tube's fixture and its neo-Hookean golden values (16f, 16g) | CI runs the sanity run and K2 at 10k (16i), and each has failed once on a deliberate mutation. The power iteration meets its accuracy bar (16e). The crate's coverage run is timed (16i) |
| **2b. The tube experiment on the CPU** | The oracle's Yeoh extension (16g). The runs of 16i, their results and commands written into this document | Every 16i run has its numbers: K2, K3, K5, the loading-time ladder, the Coulomb push, the Yeoh and confined cases, stiffness scaling, the gap record, and the loaded step factor. 2d reads the last four |
| **2c. K6** | The Cattaneo–Mindlin block and its readouts (16b, 16f), and its runs | K6 has failed once under each of 16b's three mutations, and is then decided |
| **2d. The product's budget, and the stop rule** | The `base_mold` measurements of §15g step 2, run locally (16j) | The stop rule has been applied, with its numbers written here |

*Amended 2026-09-25 (§16o): 2b's "gap record" was the penalty's (gap = F/k). Under the kinematic law the
penetration against the grid is at most 0.2 µm on the tube, so 2b records G2 against the grid and
against the true surface on the ladder, and 2d measures both on the product.* *(§16r: 2d measured the grid
against the scan; G2 against the grid needs a run on the product, step 7.)*

*2b's runs are in §16p, 2026-09-25, on the damped solver it led to. 2c's are in §16q, and 2d's in §16r.*

### 16d. What the shared math gains

- **Per-direction constraints.** Each node holds up to two constraint directions, orthonormal, or zero
  when unused. Phase 8 removes the displacement's and the velocity's components along them.
  - This covers the confined case (a radial direction on the outer wall, and z on every node), the
    out-of-plane condition for a 2D reference, and symmetry planes.
  - A fully held node stays `held` (inverse mass 0), as in step 1.
  - The directions are fixed at rest. So a constraint is a plane, not a curved surface a node slides
    along. That is exact for the axisymmetric tube, where no node moves circumferentially. *(Amended
    2026-09-25, §16n: the cased tube did move circumferentially. Nothing held its rotation, and a radial
    hold's fixed direction let its case open; its outer wall is now held whole.)*
  - The lumped mass is the same in every direction, so removing a component is the mass-orthogonal
    projection. No mass correction is needed.
- **The readout's tributary area:** a third of each incident deformed boundary triangle (`sim-soft`'s
  convention, §15c). The triangle's area is shared math; summing it at the nodes is orchestration.
- **The internal-energy density of the averaged λ term** at a node, for the monitor. Step 1 already has
  the per-element μ terms (`tet4_energy_mu_terms`).
- **Kinematic projection is dropped from step 2** (amended 2026-09-24, before any data). It has no
  friction (§15c), and a verdict's μ = 0 run gives push force's geometric share only if it uses the
  same contact law as its frictional runs (§15h). So (b) cannot replace (a) in the product. It stays a
  diagnostic, built only if K2 fails and 15e's localization needs it.

### 16e. The executor and the stepping loop

**The trait** has one method per phase of §15f, in that order. It also has:
- **`accumulate`:** adds each node's contact normal force and tangential force to per-node sums. It is
  called only inside a window (the tube's measurement window; each of K6's monitor intervals), so a
  time-averaged readout never reads back each step.
- **Per-node running values, updated every step inside the phases** and reduced only when read:
  - the largest penetration so far, since G2 applies at any step (fit plan §5);
  - the work done by the contact forces, and the energy mass damping removed (15d.3).
- **Monitors,** reduced on the executor and read every 100 steps:
  - kinetic and internal energy;
  - the kinetic energy of the nodes in contact, a watch for friction flutter (below);
  - the contact force's resultant, as its mean over the steps since the last read, so a peak read from
    the monitors is not a single step's chatter;
  - a sticky count of inverted elements (K4);
  - the reductions of the running values.
- **Snapshot:** the displacements and the accumulated sums, read on request.
- **The power iteration's pieces** (below). Its vector stays on the executor; the host reads one scalar
  per iteration. *(2026-09-30, §17b: on the GPU the whole estimate is one read.)*

**The CPU executor is one source file, compiled at f32 and f64** by the same `include!` pattern as the
shared math (§14c). K3 compares the two. It loops with rayon on native, and sequentially on wasm32
(`newton.rs`'s `cfg` pattern). Gathers loop over each node's incident element slots in a fixed order,
so no two threads add into one place. A 2a test checks the consequence: the forces are bitwise equal on
one thread and on many.

**The stable step.** *(Amended 2026-09-25, §16o: the kinematic law adds nothing to the step, so
Δt = 0.9 · 2/ω_el; the penalty bound below, and the flutter it names, were the penalty's. Amended
again, §16p: the frictional tube did flutter under the kinematic law, the material was given its own
viscosity, and the step is 0.9 · 2/ω (√(1 + ξ²) − ξ) for the top vector of M⁻¹(K + βC). In 2c, §16q: it
is computed as 0.9 · 4/(γ + √(γ² + 4ω²)), γ = vᵀCv/vᵀMv, which holds at ω² = 0 and, where it has a root,
below; and the start estimates twice whatever the material.)*
- The elastic part comes from a power iteration on M⁻¹K. K·v is the finite difference of the elastic
  force phases (1–5) in the direction v, at the current state.
- The penalty is added as a bound, not iterated. On a node in contact, the normal penalty and the
  sticking tangential penalty have stiffness k·I, with k = s·m/Δt² (§15c). So, by Weyl's inequality
  applied to M^(−½)KM^(−½) (arithmetic):

      ω_max² ≤ ω_el² + s/Δt²

  §15c's Δt = 0.9 · 2/ω_max then gives **Δt = √(3.24 − s)/ω_el = 1.655/ω_el at s = 0.5**.
- The bound holds whichever nodes are in contact, including nodes that touch between two re-estimates.
  It costs 8 % of the step against 1.8/ω_el, which would ignore the penalty (arithmetic).
- **A slipping node's stiffness is not symmetric.** In the (normal, slip direction) basis it is
  k·[[1, 0], [−μ_f, 0]]: its eigenvalues are real and lie in [0, k], and its symmetric part's largest
  eigenvalue is k(1 + √(1 + μ_f²))/2 (derived). Using that in place of k bounds the real parts:
  - **Δt = √(3.24 − s(1 + √(1 + μ_f²))/2)/ω_el**, which is 1.652/ω_el at μ_f 0.3 and 1.559/ω_el at
    μ_f 2, the top of §5c's widest range (arithmetic).
  - The loop uses this form, with the run's largest μ_f.
- **What no bound covers is flutter.** Coupled with the elastic stiffness, a non-symmetric stiffness
  can have complex eigenvalues. Central differences amplify those at any step, and mass damping removes
  only α_D/2 of their growth rate. Nothing gates it: the contact nodes' kinetic energy is reported, and
  whether flutter occurs here is not known.
- The iteration is re-run every 500 steps, each from the same fixed start (16m). The step grows by at
  most 5 % per re-run (§15c) and shrinks at once.
- **Its accuracy is checked, not its precision.** A Rayleigh quotient never exceeds the largest
  eigenvalue, so agreement between f32 and f64 says nothing about convergence. 2a's done-when: at the
  iteration count the loop uses, the estimate of ω_el² is within 5 % of a converged f64 reference on
  the 10k tube. That is a fifth of the 27.7 % margin that 0.9 · 2 leaves below the stability limit in
  ω_el² (arithmetic: (4 − s)/(3.24 − s) − 1 at s = 0.5). 2b repeats the check on the 50k tube, and 2d
  on the product's mesh, before either relies on the step.

**Loading.**
- The mandrel's pose is sampled every T/1000 and interpolated (step 1's `pose_sample_span` and
  `pose_interpolate`). Interpolating the ramp's quadratic segments linearly is off by at most a·τ²/8:
  0.15 µm at T = 1.04 s, against a penalty gap of about 30 µm (arithmetic: 105 mm of travel, a =
  1.08 m/s², τ = 1.04 ms).
- The speed profile, the hold and the window are §15b's.
- Mass damping (§15c) stays on through the hold, where nothing translates, so its drag on a moving
  body (§15c) does not arise there.

**The loop checks the generic validity gates,** and reports every run with them (§15a, amended
2026-09-24 in step 2's design, before any data):
- KE/IE ≤ 5 % over the interval each readout is averaged over, as that interval's mean KE over its mean
  IE: the constant-speed phase for the Coulomb push and K5's entry peak, and the measurement window for
  K2. It was the window only. A ratio per sample would be 0/0 before first contact, which comes inside
  the constant-speed phase.
- **The energy balance:** the contact forces' work equals internal plus kinetic energy plus what damping
  removed, within 1 % of the run's peak internal energy. The work is summed as f·(u⁺ − u⁻)/2 per step:
  undamped, central differences change the half-step kinetic energy by exactly that (derived). It
  catches energy the
  integrator creates, such as a step past the stability limit, and not flutter, whose energy comes in
  as contact work. The 1 % is an engineering call, not sourced.
- K4's count.

The band's λ_z against the oracle's is the tube's own check, and belongs to its readout.

### 16f. The fixtures: a plain public module, not a feature

- The tube (§15c's structured annulus, its three meshes and its Kuhn split), the mandrel's grid and the
  golden values (16g) live in a public `fixtures` module, from 2a. K6's block joins it in 2c.
- §14a planned a `test-fixtures` feature. F1 says code behind a feature is neither coverage-measured
  nor doc-checked, and these fixtures define the experiment's geometry. So they are graded like the
  solver.
- **The mandrel's grid** is its exact distance (a cylinder of radius a with a hemispherical nose),
  sampled at A/20 over the region the tube can reach (§15c).
- `sim-gpu`'s conformance tests (step 4) and the benchmarks use the same module.

### 16g. Golden values

- `thick_tube_reference.py` gains a golden mode that writes a Rust source file of constants into the
  `fixtures` module. It holds p/μ, λ_z, ∂p/∂a and ∂p/∂λ_z for each K2 case (15d.1) and the confined
  case (2a), and the Yeoh cases (16h, 2b).
  - A source file, not JSON: step 5's GPU runs in `sim-gpu` read the same values through the public
    module, and no JSON parser is needed.
- The oracle's existing self-checks run before anything is written. The Yeoh extension (2b) adds its
  own: C₂ = 0 reproduces the neo-Hookean values, and at small interference the pressure is Lamé's with
  λ + 8C₂ in place of λ (the stiffness `dilatational_wave_speed` already uses).
- As with `gen_golden.py`, a regenerated file is a reviewed change.
- K6's reference is two closed-form expressions, written in its test with their sources (16b) *(2c put them
  in the fixture, which the probe and the CI check share, §16q)*. The
  coupling's size comes from `cattaneo_mindlin_reference.py`, which is a referent, not golden data.

### 16h. The Yeoh case is judged on its increment ✓

At the C₂/μ of every datasheet anchor in `sim-soft`'s table (0.086–0.094; the measured Ecoflex fits are
0.014 and 0.017), the Yeoh term moves the band pressure very little. From a scratch run of the oracle extended by
the C₂ term, at ν 0.49, B/A 2, free ends:

| Material | C₂/μ | λ_a 1.1 | λ_a 1.3 |
|---|---|---|---|
| `ECOFLEX_00_30` | 0.089 | +0.56 % | +4.22 % |
| `DRAGON_SKIN_10A` | 0.087 | +0.55 % | +4.15 % |

Its C₂ = 0 values reproduce the oracle's table (0.12352, 0.30507). K2's 5 % would pass with the Yeoh term
missing. So:
- the Yeoh case runs at λ_a 1.3, ν 0.49, on the 50k mesh, with `ECOFLEX_00_30`'s C₂ (2 050 Pa) on the
  tube's material;
- it is judged on the **increment**, the solver's p_Yeoh − p_NH on the same mesh against the oracle's
  (4.22 % of p_NH), within 25 % of that increment, which is 1.05 % of p_NH (arithmetic).
  - A missing or doubled C₂ term is off by about 100 % of the increment (arithmetic).
  - The 25 % allows the element's error to differ between the two runs by up to 1.05 % of p. Whether
    it does is not known before 2b;
- the absolute K2 criterion applies too;
- **a failure stops before any GPU work,** as K3, K4 and K6 do, since `base_mold` is Yeoh (15g step 1).

### 16i. The runs, and where they live

**In CI:**
- tests-debug: a sanity run on the 10k tube, a few hundred steps, asserting finite energies and J > 0.
  It makes no accuracy claim (§15g).
- tests-release: K2 on the 10k tube at λ_a 1.1, ν 0.49, asserting both the raw and the gap-corrected
  error ≤ 7 % (§15g). If 2b finds another corner worse at 10k, the test moves to it. *(Moved
  2026-09-25, §16p: λ_a 1.1 at ν 0.495 is the worst 10k corner.)*
  `sim-soft-explicit` joins a tests-release shard's explicit list in 2a.
- tests-release, from 2c: the coarse K6 (16b).
- **These release-only tests live in their own integration-test binaries,** named in the crate's
  `coverage_skip_binaries`. The weekly coverage job builds a crate's tests in release and runs every
  binary instrumented (`xtask/src/coverage_run.rs:609`), at 164–1 226× the uninstrumented time on the
  suites that file measured (`:690`). The precedent is `sim/L0/soft/Cargo.toml:77`.
- **The crate's other tests still run instrumented, on rayon,** and the job does not pin
  `RAYON_NUM_THREADS` (`.github/workflows/scheduled.yml`). How much the instrumentation slows rayon code
  here is not known. 2a times the crate's coverage run locally, at the CI runner's thread count and at
  one thread. If the first is much slower, the executor's tests run on a one-thread pool.

**Recorded commands, in 2b** (results and commands written into this document):
- the loading-time ladder (§15c), at 10k;
- K2 at 10k and 50k, at every corner, raw and gap-corrected;
- K3: f32 against f64 at 50k (the band's mean and each pair-averaged ring level), and the Coulomb push at
  10k;
- K5: the peak entry push (the largest of the monitor's 100-step means) and the seated
  95th-percentile pressure, 10k against 50k *(2026-09-26, §16s: the seated reading is now the most-loaded
  1 cm² patch; from 10k to 50k the frictionless patch and the geometric share do not converge)*;
- the Coulomb push (15d.7) and the Yeoh case (16h);
- the confined case (15d.8), and a product-level run: the free tube at λ_a 1.3, ν 0.49, with
  `DRAGON_SKIN_10A`'s μ. Each records its raw error and its largest gap (§15g).
  - Every run also records its largest gap next to p/(λ + 2μ) and its element size. At a fixed s, the
    penalty's gap depends on those and on element shape, not on μ alone (arithmetic: gap = F/k, with
    k = s·m/Δt² and Δt ∝ h/c_d). That record is what carries G2 from the tube to the product's mesh
    *(superseded 2026-09-25: 16c's note)*;
- stiffness scaling (15d.10) at 10k. It moves here from step 5, because 2d's budget depends on it: it
  decides whether a verdict is 3 runs or 5 (§15h);
- the loaded step factor: the smallest in-run step over the rest step, which 2d applies to the product.

**The CPU may reach 100k.** Measured on the M4 Pro (a scratch benchmark of the shared math, not kept):
the element phases alone (1 and 4) take 0.29 ms per step at 50k and 0.49 ms at 100k on 12 threads, f32.
The gathers, contact and integration are not in that number, and the whole step is measured in 2a.
- If a 100k run fits in 10 minutes on the CPU, K2 runs at 100k there too.
- **Stop rule, amended 2026-09-24, before any data:**
  - K2 is defined at 100k (§15a). So where a measured 100k error exists, it decides: proceed if it is
    ≤ 5 % at every corner, and stop otherwise. The 50k rule and its extrapolation apply only where it
    does not.
  - The Yeoh case (16h) joins K3, K4 and K6 in stopping GPU work when it fails.

### 16j. The product's budget (2d)

*Amended 2026-09-26 (§16r, 2d as built): 2d writes ratios and verdicts here, not the product's counts, steps or
times (Jon): an element count at a known element size gives the wall's volume, and a step count at a known step
and speed the insertion's length. K1's per-step budget is 2 minutes over the damped 100k tube's steps at the
ladder's rung.*

*Amended 2026-09-25 (§16p): the solver now carries the material's viscosity, and 2d's inputs change with it:*
- *the product's η: none is known for Dragon Skin 10A (fit plan U15), and the step, the loading rung and the
  frictional seated state depend on it. The tube's runs used Ecoflex 00-30's η/μ throughout;*
- *the loading speed: the ladder's rung, 0.625 T_s (v/c_s 0.39), holds for Ecoflex's η/μ (undamped it was
  2.5 T_s), and the frictionless push peak moved up to 12 % across the rungs (the push with friction, 1.5 %);*
- *the loaded step factor: 0.977, the smallest in-run step over the rest step on the damped tube;*
- *the per-step cost: ×1.36–1.67 a step (§16p's times; corrected in 2c, §16q), and ×1.06–1.39 the steps, against the
  undamped solver;*
- *the steps' factor depends on the element size: integrated explicitly, the viscosity cuts the step 19× on 83 µm
  elements and 80× on 20 µm (2c, §16q), against 12–28 % on the tube. 2d measures the product's step with the
  viscosity on and off;*
- *the damping's form (Kelvin–Voigt, or a Maxwell branch) is not decided, and moves both costs.*

- **Where it runs:** a command in `cf-sim-research`, since the scan never enters the repo. It meshes
  `base_mold`'s wall with the tool's current Tet4 mesher (`SdfMeshedTetMesh`) and lowers the mesh into
  `ExplicitModel` with per-element materials.
  - Lowering moves to `sim-soft` in step 6. This copy measures only.
  - It fails loudly when the scan is missing, instead of skipping.
- **What it measures** (§15g step 2):
  - the element count when the wall's elements have size h_K2: the largest h, as the stop rule defines
    it ((mean tet volume)^⅓), at which K2's gap-corrected error is ≤ 5 % at every corner. It is
    measured where a run exists at that h, and taken from the stop rule's extrapolation otherwise.
    Whether h_K2 gives the product the tube's accuracy is checked on the product itself in step 7;
  - the stable step from the CPU executor's power iteration, at rest, times 2b's loaded step factor;
  - the loading time at the tube's converged speed as a fraction of the shear wave speed, v/c_s. This
    is an assumption, not a derived rule: step 7 checks it with a ladder on the product;
  - the per-run time from those steps at K1's per-step budget, scaled by element count;
  - the per-press time, with 3 or 5 runs per verdict, as stiffness scaling says;
  - the surface bias (the fraction of canal nodes inside the true surface), with and without
    projecting the boundary nodes onto it, and what projecting does to the stable step;
  - G2's margin on the product: the baked scan grid's error against the scan, and with it whether the
    pre-smooth is still needed (14b's note) *(amended 2026-09-25, §16o)*.
- **The per-press time is reported against D4's 5 minutes** (§9 decision 12). If it misses, the speed
  plan is revised before any GPU work, and the quality gates are not loosened (§15g).
- Only ratios and verdicts are written here *(amended 2026-09-26, §16r; it said counts, steps and times)*.
  No geometry leaves the machine.

### 16k. How the design was checked

Two cold reviewers, against ten criteria written beforehand (16a's list among them), at `7ea0b73e`:
- **physics and numerics:** re-derived the bounds, reran the Yeoh oracle and wrote a boundary-element
  solution of K6's contact;
- **fit to the whole plan:** read every section, mapped §15g step 2's bullets to the four PRs, and
  checked the CI, grade and coverage claims at their referents.

**Sixteen findings, all checked before fixing.** The ones that changed the design:
- **K6's precision and readout.** Both reviewers found it:
  - the elastic-slip window is a few f32 spacings wide, so K6 runs at f64 with its frames at the
    contact;
  - reading edges node by node used 0.02a of the 0.03a tolerance, so the edges are read from
    window-averaged forces;
  - an anchor that is not dragged could not be seen while loading.
- **The coupling was measured,** not companion-run. The boundary-element solution was hardened,
  made to fail once per check, and committed (16b).
- **Monitors:** G2's largest penetration at any step, the work and damping sums, an energy balance,
  and the resultant's mean between reads.
- **KE/IE** now covers every phase a readout is taken from. K6 has its own rate ladder.
- **Kinematic projection dropped** from step 2 (16d).
- **The power iteration's bar** is accuracy against a converged reference, not f32 against f64.
- **Slipping friction** is bounded in its real part. Flutter is named as unbounded and watched.
- **2d's inputs pinned:** h_K2, the loading speed, the loaded step factor.
- **Consequences given:** to the Yeoh case, to a measured 100k error, and to the CI K6.
- **Coverage:** the release-only tests are kept out of it.

**What the method missed.** 16a named §6's rule, that displacements are the state, and applied it to
the element. The friction anchors, stored as absolute positions, were never checked against it. That
is the same class of miss as step 1's, one level down. A constraint list is only as good as the
places it is checked against.

**A second pass read only the fixes** (`7ea0b73e..821c3323`) with one fresh reviewer. It found 4
problems, and all 4 sat in text the fixes wrote:
- interpolating the stick ratio still read 0.02a wide;
- KE/IE per sample is 0/0 before first contact;
- v/c_s dropped the strain it scales with;
- neither flutter guard could see flutter.

They were fixed by cutting: the readout rule moved to a 2c unit test with a bar, flutter is reported
and not gated, and the loading speed is marked as an assumption. The load-bearing design (the four
PRs, K6's reference, the trait, the stop rule) was not touched by pass 2. What changed was detail the
build will measure. So no third pass was run on the prose. The readout rule and the energy balance's
1 % are settled by 2c's and 2a's tests.

**Settled only by running,** and where each is measured:
- finite strain and the finite domain's effect on K6's stick zone: 2c's two companion runs *(§16q: each differs by
  just over 0.01a, so K6 is judged on each too, and passes; R = 200a at half its rate differs by 0.0089)*;
- whether flutter occurs: every 2b and 2c run reports the contact nodes' kinetic energy, and nothing
  gates it *(2026-09-25, §16p: it occurred at μ_f 0.3, found through the Coulomb push's shortfall; a
  linearized analysis finds growing structural modes there, and growing modes at μ_f 0.1 too, where runs
  show none)*;
- the power iteration's accuracy beyond the 10k tube: 2b on the 50k tube, 2d on the product's mesh *(done,
  §16r)*;
- the coverage cost of the new tests: 2a, timed at two thread counts.

Not scheduled: how coverage counts code `include!`d twice.

### 16l. Not decided here

- The trait's exact signatures, settled in 2a (§14d).
- The power iteration's finite-difference size and iteration count, set in 2a against its accuracy bar
  (16e).

### 16m. 2a, as built (2026-09-25)

2a holds what 16c gave it. Four cold reviewers read it against criteria written beforehand (the
engine, the loop and fixtures, the tests and this record, and the whole plan). They found 17
distinct problems, and each was checked before it was fixed.

**Decided in the build or by the review:**
- **The friction anchors are state.** They start in the obstacle's body frame, as the contact law reads
  them. `snapshot` carries them, and `set_state` takes them back, or re-anchors each node where it sits.
  - Started from rest positions in the world frame instead, a floor moved along its own plane gave a
    different run.
  - They stay absolute body-frame positions (§16b sends K6 to f64). On the tube, at μ_f 0.3 and a 28 µm
    gap, the elastic-slip window is about 1 100 f32 spacings at 0.1 m (arithmetic). At μ_f 0.05 and a
    10 µm gap it would be 67 spacings, each 1.5 % of the Coulomb limit.
  - Step 4 decides between absolute anchors and a stored elastic slip before it fixes the WGSL layout.
    *(Amended 2026-09-25, §16o: the kinematic law has no elastic slip; the windows above become about
    650 and 40 spacings, each 2.5 % of the limit (arithmetic, ×0.59), and step 4 sets the anchors'
    precision against that. 2c, §16q: in f32, K6's stick zone reads 0.56 off, and §15g step 5 compares f32
    with f64 on a frictional seated state.)*
- **The trait gains:**
  - `set_poses`, for a new track mid-run (§14d's batches; K6's legs that end on a force);
  - `phase_outputs`, for step 4's per-phase conformance;
  - the contact law's s and μ_f. The loop reads them, so the stable step bounds the law in use. *(Both
    removed 2026-09-25: the kinematic law adds nothing to the step, §16o.)*
- **Every stable-step estimate starts cold,** at 100 power iterations. Its finite-difference step is
  √ε of the executor's precision times the shortest rest edge, on the largest nodal component
  (`Stepper::estimate`).
  - Warm-started at 20, the estimate stayed on a lower mode once the tube was loaded. It read 3.7–3.9 %
    low through the hold (the review's measurement, not kept). A test now checks that an estimate
    depends only on the state.
- **The band reads λ_z, the gap and its areas at the window's mean state,** as the pressure is a window
  mean. At the last instant, λ_z swings about 0.4 % across the window, and K2's reference with it.
- **The loop:**
  - it takes a final read after the last step, so K4 and G2 see every step;
  - it stops on a non-finite read;
  - the gates refuse one.
- **The golden values are generated Rust constants** in `fixtures::golden`. `thick_tube_reference.py
  --golden` writes them after its checks and a cross-check against §15b's table.
- **`fixtures::tube::TubeRun` runs the tube on any executor,** so 2b's runs and step 5's use one path.
  It returns the final snapshot, for K5's per-node pressures. It takes Yeoh's C₂, and the monitors carry
  Σf_n for the Coulomb push.
- **The executor's tests do not run on a one-thread pool.** Measured warm with `cargo xtask grade
  sim-soft-explicit`, the crate's coverage pass takes 18.0 s at CI's 4 threads (`RAYON_NUM_THREADS=4`),
  22.3 s at 1, and 37.2 s at 12.

**Measured** (the tests print `MARGIN` lines):
- **K2 on the 10k tube,** λ_a 1.1, ν 0.49, T = 10 T_s, f32 on the CPU: raw **+1.67 %**, gap-corrected
  **+4.42 %**, over 13 799 steps.
  - The validity gates: λ_z −0.04 %, KE/IE 0.16 %, energy balance 0.04 %, and no inverted element.
  - `cargo test --release -p sim-soft-explicit --test tube_release -- --nocapture`.
- **The penalty's gap at this corner is −28 µm,** which biases pressure 2.6 % low (∂p/∂a from the
  oracle, arithmetic).
  - §15c's "about 1 %" came from a reviewer's inner-node V_a/A_a of 0.88 mm on the 100k mesh.
  - The gap scales with element size (16i), so 2b records it on the ladder. *(The penalty's; moot since
    §16o.)*
- **G2 is not met on the tube at 10k.** The deepest penetration is 45 µm, 4.5 % of the 1 mm
  interference, against 1 %. §15g step 2 settles the contact law against G2 in 2b.
- **The power iteration at the loop's 100 cold iterations,** against a converged f64 run on the 10k
  tube: −0.21 % at rest, and −0.35 % (f64) and −0.32 % (f32) loaded at K2's end. On a small block it is −1.1 %
  against a dense eigensolve (`tests/executor.rs`).
- **The energy balance's own error** is 0.28 % of the peak internal energy, on a pressed block at α 500
  (`tests/executor.rs`), against the 1 % gate.

**Not recorded in 2a; 2b records both on the ladder, with `RAYON_NUM_THREADS` set:**
- **The whole step's cost,** which §16i said 2a measures.
- **The share of it the cold estimates take.** 2d adds that share to its per-run time *(§16r: K1's
  wall-clock carries it; the CPU's timed steps include one re-estimate per 1 000 steps, a run two)*. The iteration
  count can come down against the 5 % bar, since the loaded estimate reads −0.35 % at 100.

A reviewer's scratch timing (release, f32, M4 Pro, not kept) gives 2b its starting point:
- 0.30–0.78 ms per step at 10k (4 and 12 threads);
- 0.95–1.5 ms at 50k, and 1.5–1.8 ms at 100k, both before contact;
- 12 threads were slower than 4 at 10k and 50k;
- the cold estimates added 8.6 % to K2's 10k run.

**Each CI check failed once on purpose,** and a check that could not fail was changed:
- **The damping-balance test** passed with the damping loss doubled at α 50, where that loss is the size
  of the balance's own error. It now runs at α 500, and fails that mutation.
- **The band test** checked the area against the readout's own output. It now checks the closed form,
  and an area halved or read at the last instant fails it.
- **Every check added in the review** fails its named mutation:
  - anchors in the world frame;
  - `set_state` ignoring given anchors;
  - `set_poses` a no-op;
  - no stop on a non-finite read;
  - gates that accept one;
  - window sums that overwrite;
  - no re-estimate;
  - a gap correction on the cased tube;
  - a halved volume-change gather.
- **K2 at 10k** fails with the λ term removed (raw −26.5 %), and the sanity run fails with the elastic
  force reversed.
- **`release-gates`** failed before `sim-soft-explicit` joined tests-release.

### 16n. 2b, G2 first (2026-09-25)

**The question:** G2 (no node deeper than 1 % of the inset, at any step) failed on the 10k tube in 2a
(16m). The fit plan's rule is that the gate stands, and if the contact law cannot meet it, the law
changes. *(2026-09-26, §16r measured the scan grid's own error against the bar on `base_mold`; fit plan U18.)* On the tube the inset is the interference: 1 mm at λ_a 1.1 (bar 10 µm), 3 mm at λ_a 1.3
(bar 30 µm). Stage 1 measured the current law, and the one fallback of §15g step 2 that needs no new
code (a larger s at a smaller Δt). The predictions were written before any run, and are scored below.

**The instrument,** as these runs used it (`c7d67eab`–`96087aa2`; the command changed later, §16o):
`cargo run --release -p sim-soft-explicit --example tube -- <10k|50k|100k> <case> <s> <μ_f> <f32|f64>`, where `case` indexes `fixtures::golden::THICK_TUBE` (0: λ_a 1.1 free, 2: λ_a 1.3
free, 4: cased). It prints one line: the deepest penetration over all steps (against the A/20 grid),
the deepest at the end against the true surface and the grid, the band's gap, K2's errors, the validity
gates and the run's cost. All runs below are f32 on the CPU, frictionless, `RAYON_NUM_THREADS=4`, M4 Pro.
The probe reproduces 2a's K2 run exactly (+1.67 %, +4.42 %, 45.3 µm, 13 799 steps).

**The cased tube was not confined.** Its first run read 3 % of the oracle's pressure (K2 raw
−96.8 %) on every mesh. The outer wall's radial hold was a fixed direction per node, so a node sliding
around the tube moved along its tangent line, and so outward, and nothing held the tube's rotation. A
diagnostic (not kept) measured every ring rotated about 10° and the outer ring at 20.51 mm instead of
20, which leaves the annulus its rest area: π(20.51² − 10.97²) ≈ π·300 mm². The fix holds the outer wall
whole (`Walls::Cased`), the same problem under axisymmetry. The fixture test fails on the old hold. The
pre-fix reading reproduces at `c7d67eab`.

**The current law, s = 0.5** (case 4 after the fix):

| Mesh (h) | Case | Deepest, grid, all steps | Of the inset | Band gap | K2 raw / gap-corrected |
|---|---|---|---|---|---|
| 10k (2.26 mm) | λ_a 1.1 free | 45.3 µm | 4.5 % | −27.5 µm | +1.67 % / +4.42 % |
| 10k | λ_a 1.3 free | 116.2 µm | 3.9 % | −69.1 µm | +1.50 % / +3.31 % |
| 10k | cased | 422.7 µm | 42.3 % | −413.7 µm | −44.9 % |
| 50k (1.31 mm) | λ_a 1.1 free | 35.1 µm | 3.5 % | −16.9 µm | −0.17 % / +1.47 % |
| 50k | λ_a 1.3 free | 78.2 µm | 2.6 % | −41.8 µm | +0.06 % / +1.13 % |
| 50k | cased | 329.0 µm | 32.9 % | −324.4 µm | −35.6 % |
| 100k (1.05 mm) | λ_a 1.1 free | 28.1 µm | 2.8 % | −13.3 µm | −0.33 % / +0.95 % |
| 100k | λ_a 1.3 free | 65.2 µm | 2.2 % | −34.4 µm | −0.15 % / +0.73 % |
| 100k | cased | 258.8 µm | 25.9 % | −255.9 µm | −28.4 % |

- Every validity gate holds in these runs (λ_z within 0.1 %, KE/IE ≤ 0.2 %, energy balance ≤ 0.04 %, no
  inversion).
- On the free tube the deepest penetration is reached while loading (t = 0.19–0.72 s of 1.24), and is
  1.6–2.1× the band's seated gap. At the end, the deepest node sits 10–15 mm behind the tip.
- **The grid reads shallower than the true surface by up to 5.7 µm** at A/20 (every run, at the end
  state). On the λ_a 1.1 tube that is half of G2's 10 µm before the contact law contributes.
- **In the cased tube the penalty acts in series with the wall.** The wall's stiffness per area is the
  oracle's p over the interference, about 95 kPa/mm; the penalty's is p over the free tube's gap, 103
  kPa/mm at 10k, 168 at 50k and 214 at 100k (arithmetic). Their ratio predicts −48 %, −36 % and −31 %;
  the runs read −44.9 %, −35.6 % and −28.4 %.

**A larger s** (at 10k; the cased runs in this sweep came before the fixture fix, so only the free
cases count): s = 1 cuts the λ_a 1.1 tube's deepest penetration to 20.5 µm (2.1 %) and its
gap to −12.7 µm. On the λ_a 1.3 tube the gap halves (−28.5 µm), but the deepest penetration does not
fall (119.4 µm, reached in the hold), and the hold rings (KE/IE 1.45 %, balance 0.86 %). At s = 2 the
contact chatters (KE/IE 61–62 %, balance 6.7–7.2 %, deepest 128 µm and 359 µm). At s = 3 both stop on a
non-finite read (steps 24 700 and 17 100), though Weyl's bound for the linear problem is 3.24. Why the
contact loses stability below that bound has not been isolated.

**So the penalty cannot meet G2 on the tube.** Its gap scales with element size: at λ_a 1.1 the band
gap falls 10k → 50k → 100k as 1 : 0.61 : 0.48, against h's 1 : 0.58 : 0.46. The deepest penetration
falls more slowly (1 : 0.77 : 0.62). The confined case misses by 42× at
10k, 33× at 50k and 26× at 100k.

**Predictions, scored:**

| | Predicted (before any run) | Measured |
|---|---|---|
| P1 | gap λ_a 1.3 / 1.1 ≈ 2.5 (∝ p) | 2.51 at 10k ✓ |
| P2 | cased: series, 30–50 % low at 10k | −44.9 % at 10k ✓; −35.6 % and −28.4 % at 50k and 100k (after the fixture fix) |
| P3 | gap ∝ h | ✓ for the band gap; the deepest node falls more slowly |
| P4 | gap ∝ (3.24 − s)/s; steps ×1.10, 1.49, 3.4 at s = 1, 2, 3 | band gap ✓ at s = 1 (0.46× and 0.41× against 0.41×); ✗ at s ≥ 2, which chatters or blows up |
| P5 | grid bias 2–3 µm | ✗: up to 5.7 µm |
| P6 | G2 fails at s = 0.5 everywhere | ✓ |

**The cost** (the §16m items): a whole step, estimates excluded, takes 0.29–0.30 ms at 10k, 0.84–0.86
ms at 50k and 1.47–1.49 ms at 100k. One cold estimate takes 19–27 ms, 68 ms and 126 ms. The estimates
are 11–16 % of a run's wall time. A K2 run takes about 5 s, 23 s and 52 s. So K2 runs at 100k on the
CPU (16i's 10-minute test).

**Not decided yet: the law that replaces the penalty.** §15g step 2 listed a gap-offset penalty, an
augmented-Lagrangian update, or a larger s; the last is ruled out above. G2 holds "at any step", and
the deepest penetration arrives while the mandrel moves. A multiplier converges over steps; whether it
keeps up while the mandrel moves has not been measured. Abaqus/Explicit's default for contact pairs is a kinematic predictor/corrector, which "has no
influence on the stable time increment": each node gets "the force which, had it been applied during
the increment, would have caused the slave node to exactly contact the master surface" (Abaqus Analysis
User's Manual 6.11, §36.2.3). Its friction has *"an infinite sticking stiffness, in which case the elastic
slip is always zero"* (§35.1.5). Its cost is that *"impact is
plastic"*: a node's normal kinetic energy is lost on contact. That removes §15c's objection to variant
(b), that friction is not defined for it. The choice is stage 2.

### 16o. 2b, G2 stage 2: the contact law's A/B (2026-09-25)

**The candidates,** each a function in the shared math (so the test is what the GPU would run),
chosen in the order the rule below was written, before any run:
- **P:** the penalty, s = 0.5 (the control; it reproduces §16n exactly).
- **K:** the kinematic predictor/corrector (`shared::kinematic_contact`). The node's position at the end of
  the step is predicted without contact; if it lands inside, the node gets the force m(1 + αΔt/2)·g/Δt²
  that puts it on the surface, along the normal made free of its constraints. Friction is kinematic: a
  sticking node is held at its anchor; a slipping one moves back by at most μ_f·g along the surface, in
  its free directions, and its anchor goes with it. It adds no penalty term to the stable step.
- **A10, A50:** the penalty plus a per-node multiplier λ, the normal force max(0, λ + k·p), with
  λ ← max(0, λ + k·p̄) every 10 or 50 steps (p̄ the mean penetration since the last update). An update
  every step was excluded before running: for one node under central differences the characteristic
  polynomial is z³ + (s − 3)z² + (3 − s + βs)z − 1, whose roots multiply to 1, so two grow by about
  √(1 + β) per step, and the mass damping (αΔt ≈ 5e-4) cannot hold a useful gain β (arithmetic).

**The rule, written first:** a law is eligible if every run finishes and keeps the validity gates; it
must meet G2 (grid, all steps) on every case at 10k and 50k; K2 within 7 % at 10k; the Coulomb push
within 5 % where another law meets it; then fewer steps, less scatter, fewer knobs. If none meets G2
everywhere, report and decide nothing.

**The runs** (`examples/tube.rs <mesh> <case> <law> <μ_f> f32 <A/n>`, `RAYON_NUM_THREADS=4`;
`kinematic`, `penalty:0.5`, `augmented:0.5:10`, `augmented:0.5:50`):

| Law | G2, free (10k, 50k) | Cased, frictionless | Validity gates | |
|---|---|---|---|---|
| P | 2.6–4.5 % | pressure −45 % / −36 % | hold | fails G2 |
| K | **0.02–0.04 %** (0.2–0.9 µm) | diverged (10k, 50k); runs, −0.34 % / −0.31 %, once its direction was fixed (below) | hold | meets the rule |
| A10 | 26–41 % | diverges | fail: KE/IE 73–80 %, balance 16–26 % | fails |
| A50 | 2.0–10 % | pressure −0.4 %, G2 11–16 % | fail at 10k: balance 1.1–2.0 % | fails |

- **K, free tube:** the deepest penetration against the grid is 0.2–0.9 µm at 10k, 50k and 100k. Against
  the true surface at the end it is the grid's own bias, 3.8–5.6 µm at A/20 and 0.8–1.4 µm at A/40 (10k,
  50k). K2: +4.19 % / +4.41 % (λ_a 1.1) and +3.25 % / +3.29 % (λ_a 1.3) at 10k; +1.28 % / +1.45 % and
  +1.09 % / +1.13 % at 50k; **+0.75 % / +0.94 % and +0.69 % / +0.73 % at 100k.** Raw and gap-corrected
  now agree, as the gap is gone. KE/IE ≤ 0.73 %, balance ≤ 0.46 %.
- **K takes 0.920× P's steps on every mesh** (1.8/ω_el against the frictionless penalty's 1.655/ω_el).
  Each step costs more (a second grid sample and the prediction): 0.345, 0.964–0.967 and 1.661–1.665 ms
  at 10k, 50k and 100k, against P's 0.300–0.316 and 0.864–0.871 ms in the A/B's control runs and
  1.489 ms at 100k (§16n). A run takes about P's wall time (4.9 s, 23.7 s, 52.5 s against 4.9 s, 23.5 s,
  51.9 s).
- **K, cased, frictionless, diverged** at t = 0.40 s (10k) with the step unchanged: the contact nodes at
  the tube's entry grew a motion around the tube, its kinetic energy growing 4–17× per 100 steps. Not the
  cause, measured: the step (the loop's ω² at step 3 500 is within 0.05 % of a converged f64 estimate, and
  safety 0.8, 0.7 and 0.5 only delay it, to t = 0.41, 0.43 and 0.51 s); the grid (A/40 and A/80 diverge
  too); the precision (f64 diverges too).
  - **The cause: the correction's direction.** K moved the predicted point back out along the normal
    *at the predicted point*. The step's inward push, a = F·Δt²/m, leaves that point at a smaller radius
    of the round mandrel, where the node's sideways travel is a larger angle; moving it back out along
    its own normal keeps that angle, so the node lands further around than it went. The sliding grows by
    about 1 + a/R a step, and on the confined tube a/R ≈ 0.05 (arithmetic). Friction stopped it by
    holding the sliding.
  - Shown three ways. A one-bead model (a circle pushed on by a constant force) grows ×1.04704 a step
    against 1 + a/R = 1.04666, and ×1.00000 with the normal taken where the node is now. A one-tetrahedron
    solver test (`kinematic_contact_does_not_feed_sliding_around_a_curved_obstacle`, a/R ≈ 0.05) grows its
    energy 35× in 400 steps with the old direction and stays bounded with the new. And the frictionless
    confined tube, with the new direction, runs to the end and reads −0.34 %, −0.31 % and −0.24 % at 10k,
    50k and 100k, G2 2.2–2.6 µm.
  - **K now takes the depth at the predicted point and the normal where the node is now.** The free-tube
    and friction numbers above were measured with the old direction. Rerun with the new one, K2 moves by
    at most 0.03 % on the frictionless runs (10k, 50k, 100k) and 0.09 % on the μ_f 0.3 run.
- **K's extra kinetic energy on the free tube goes with the grid.** At A/20 its KE/IE and balance read
  0.63 % and 0.47 % (10k, λ_a 1.1) against P's 0.16 % and 0.04 %; at A/40 they read 0.16 % and 0.04 %,
  and at 50k 0.03 % and 0.01 %. K follows the baked surface exactly, and the mechanism beyond that
  association has not been isolated.
- **A second, slower instability remains: the frictionless confined tube in a long hold, at 50k.** With a
  1.0 s hold (§15b's is 0.2 s), K's kinetic energy grows about 400× after the mandrel stops and levels off
  near KE/IE 0.5 %, and the balance reads 1.11 % (A/20) and 3.42 % (A/40), against the 1 % gate. Not the
  contact law: A50 diverges in the same run. Not the grid's coarseness: A/40 is worse. Any friction
  (μ_f 0.05) removes it, and 10k decays. Its fastest nodes include inner-ring nodes beyond the mandrel's
  tip, out of contact. **What drives it has not been isolated.** In §15b's 0.2 s hold it stays inside the
  gates (KE/IE 0.03–0.06 %, balance 0.16–0.23 % at 50k and 100k).
- **K, cased, with friction, runs clean, and reads the confined oracle:** −0.34 %, −0.31 % and −0.24 %
  at 10k, 50k and 100k (μ_f 0.05), against P's −45 %, −36 % and −28 %. μ_f 0.3 reads the same pressure
  (−0.35 %, −0.32 %), so friction carries nothing at the seated equilibrium. G2: 2.2–4.4 µm (0.22–0.44 %).
  - An earlier version of K kept the friction step off the node's constraints, and at μ_f 0.3 its cased
    G2 read 1.4–1.9 %. The step is now kept to the node's free directions and the surface; a test fails
    on the old step.
- **The Coulomb push** (15d.7) reads 0.87 for P and 0.89 for K at 10k: neither meets 5 %, so it separates
  nothing here. Why both read 11–13 % low has not been isolated; 2b's Coulomb push takes it up.

**Predictions, scored:**

| | Predicted | Measured |
|---|---|---|
| Q1 | K's G2 (grid, all steps) < 1 µm everywhere | ✓ free (0.2–0.9 µm); ✗ cased with friction (2.2–4.4 µm, still ≤ 0.44 %) |
| Q2 | K's steps × 0.918 | 0.920 on every mesh ✓ |
| Q3 | K's cased K2 single-digit | −0.24 to −0.35 % ✓ (frictionless, once its direction was fixed) |
| Q4 | K's true G2 = the grid bias: ≤ 5.7 µm at A/20, ≤ 1.5 µm at A/40 | 3.8–5.6 µm, 0.8–1.4 µm ✓ |
| Q5 | A: seated gap ~0, the all-steps maximum above 1 % | all-steps ✓; seated ✗ (A10 rings; A50 up to 115 µm) |
| Q6 | an A may ring or blow up | A10 both ✓ |
| Q7 | K's scatter and contact-node KE above P's | scatter ✓ (e.g. 2.26 % against 1.67 %); contact KE mostly ✓ |
| Q8 | Coulomb within 5 % for K and P | ✗ both (0.87, 0.89) |

**By the rule, K is the only law that meets it**, once its correction direction was fixed: it meets G2
on every case at 10k, 50k and 100k, keeps the validity gates in §15b's run, reads the confined oracle
within 0.35 %, and costs the same wall time as P. Every A-law fails. Open: the long-hold instability of
the frictionless confined tube at 50k, and why both laws' Coulomb push reads 11–13 % low. The adoption
is Jon's call (2026-09-25).

**The long-hold instability, chased (2026-09-25).** The frictionless confined tube at 50k with a 1.0 s
hold, under K. Each row changed one thing in the executor's obstacle lookup (a diagnostic, not kept):

| What the contact reads | Kinetic energy through the hold | Contact work in the hold |
|---|---|---|
| the A/20 grid (trilinear distance, finite-difference normal) | grows ~400×, then levels near KE/IE 0.5 % | rises 9 mJ |
| the exact mandrel | decays to 4e-9 J | flat |
| the exact distance, the grid's normal | decays to 4e-9 J | flat |
| the grid's distance, its exact trilinear gradient | grows from t ≈ 1.4 s | rises |
| the exact distance plus smooth bumps, 5 µm at the cell spacing | grows | rises about 30 mJ |
| the same, 1 µm and 0.2 µm | decays | flat |
| a tricubic (Catmull–Rom) interpolant of the same A/20 grid, and its gradient | decays to 9e-9 J | flat |

- **The energy comes from the contact:** the mandrel is stationary and frictionless in the hold, yet the
  contact does positive work, 9 mJ over the hold (3 % of the stored energy). The balance gate cannot see
  that part, which enters both sides (16e); the 1.11 % it read is energy beyond the contact's work, made
  by the integrator. KE/IE stayed near 0.5 %. **No gate is built to catch energy fed in as contact work**
  (open).
- **The cause is the grid's distance, not its normal and not the law:** the trilinear interpolant of a
  curved surface is off by up to 5.7 µm here, and bumps that large feed the contact energy; bumps of 1 µm
  and less do not. The step, the normal, the precision and the contact law are ruled out above. The
  adopted lookup is off by 0.9 µm across the mandrel's nose–shank seam (below), so the calm runs include
  an error near 1 µm; where between 1 and 5 µm the pumping starts is not measured.
- **Not isolated:** how a stationary, bumpy surface feeds energy through the kinematic correction. A
  one-bead model pressed onto a bumpy floor gains none, so it takes something the one-bead model lacks.
  Why A/40 (bumps a quarter the size) pumps harder than A/20 is not explained either.
- **The loop's step estimate reads 1.46 % low** in this confined state at 50k (100 iterations against a
  converged f64 estimate), so the step is at 0.906 of the limit instead of 0.9. Recorded; it is not this
  instability's cause.

**A tricubic interpolant of the grid** (Catmull–Rom, 64 grid values per lookup, its exact gradient), on
the standard runs:
- G2 against the true surface falls from 3.8–5.6 µm to 0.0–0.9 µm (10k, 50k). The lookup's error on this
  mandrel at A/20 is 0.053 µm away from the nose–shank seam and 0.9 µm across it
  (`tests/sdf_lookup.rs`); the probe puts the 50k runs' deepest node 10.9 mm behind the tip, at the seam.
- K's extra kinetic energy on the free tube goes: KE/IE and balance 0.16 % and 0.04 % at 10k (P's level).
- The band's node scatter falls from 1.01 % to 0.15 % (10k) and 2.23 % to 0.05 % (50k): the grid's bumps
  were most of it.
- The confined K2 moves closer to the oracle: −0.18 % and −0.12 % at 10k and 50k.
- Its cost was not measured by the diagnostic, which read an environment variable at every lookup; it is
  measured on the adopted code below. The trilinear lookup fetched 56 grid values (seven probes);
  tricubic fetches 64.
- It departs from step 1's decision that the shared lookup matches `cf-geometry`'s clamped trilinear
  lookup (§15g step 1, the conformance test). Adopting it is Jon's call (2026-09-25).

**Adopted (Jon, 2026-09-25): the kinematic law, with the tricubic lookup.** The penalty and the
augmented Lagrangian are deleted, with the law's choice (`ContactLaw`) and the penalty's term in the
stable step (now Δt = 0.9 · 2/ω_el). §15c, §15g step 1, §16b and §16e carry dated notes. The probe's
command is now `tube <mesh> <case> <μ_f> <f32|f64> <A/n> <hold>`; the runs above that name a law ran at
`d7f4822b`–`69805cbf`, before the law was fixed.

On the adopted code (`5e6b7281`; frictionless, A/20, §15b's 0.2 s hold unless noted):

| | 10k | 50k | 100k |
|---|---|---|---|
| G2, deepest over all steps, against the grid | 0.0–0.2 µm | 0.0–0.1 µm | 0.0–0.1 µm |
| G2 at the end, against the true surface | 0.0–0.2 µm | 0.0–0.9 µm | 0.0–0.6 µm |
| K2, λ_a 1.1 and 1.3 (raw = gap-corrected) | +4.41 %, +3.30 % | +1.47 %, +1.13 % | +0.95 %, +0.73 % |
| The confined case | −0.18 % | −0.12 % | −0.06 % |
| KE/IE and balance, largest | 0.16 %, 0.04 % | 0.03 %, 0.01 % | 0.01 %, 0.01 % |
| The band's node scatter | 0.01–0.15 % | 0.00–0.06 % | 0.00–0.05 % |
| Per step (4 threads), estimates excluded | 0.33–0.36 ms | 0.98–1.00 ms | 1.68–1.70 ms |

- The 50k confined tube in a 1.0 s hold reads KE/IE 0.00 % and balance 0.00 %.
- A step costs what the kinematic law cost on the trilinear lookup (1.68–1.70 against 1.66–1.67 ms at
  100k), and more than the penalty's (above). The penalty took 9 % more steps, so a run takes about as
  long: 53.3 s against 51.9 s at 100k.
- With friction (μ_f 0.3, 10k, λ_a 1.1), at `77f51dc6`: G2 0.0 µm, balance 0.50 %, and the Coulomb push
  0.893, still 11 % low; on λ_a 1.3 it reads 0.786 (at `dfc759d6`), so the shortfall is 11–21 % and
  varies with the case (open, 2b). The band reads +3.38 % against the frictionless oracle with λ_z
  −1.75 %: K2's λ_z gate (0.5 %) is for frictionless runs, so this is not a K2 reading. The confined case
  reads −0.18 % with friction too.
- **A friction run is sensitive to a change far below its readings.** Dividing the Coulomb limit by the
  reach (`b40eb773`) changes it only by n·n's rounding on a free node, yet the band moved from +3.56 % to
  +3.38 % and λ_z from −1.80 % to −1.75 %. The frictionless and cased runs did not move. Why the run
  amplifies it has not been isolated; it bears on comparing friction runs across versions and on step
  4's CPU–GPU conformance.
- `tube_release` passes: K2 at 10k +4.41 % (bar 7 %), its deepest penetration 13 nm; the power
  iteration −0.21 % at rest and −0.33 % (f64), −0.30 % (f32) loaded.
- **By the stop rule** (16i), the two corners run at 100k read +0.95 % and +0.73 %, against 5 %; the ν
  0.495 corners are 2b's.

**Open, for later steps:**
- **The GPU executor (step 4):** the law takes two lookups per surface node per step (the current
  position, for the normal and G2; the predicted one, for the depth), 128 grid values, and
  `sdf_tricubic` takes its 64 values by value. Its GPU cost is not measured. The contact phase also
  reads the integrator's state (each node's velocity, elastic and, since §16p, viscous force, inverse
  mass, constraints, and α)
  and keeps a per-surface-node sample buffer, which bears on step 4's bind layout and on whether contact
  is fused with the integration.
- **The product (2d):** G2 there is the scan grid's own error, and the pre-smooth is sized against it
  (14b's note). The verdict's frictionless run (§15h) is the configuration in which the long-hold
  pumping appeared on the tube, and bumps in the grid's own samples (the scan's facets) are not removed
  by the lookup. Unexamined. *(2d, §16r, measured the grid's error against the scan and the pre-smooth.
  G2 against the grid, and the pumping, need a run on the product, so they move to step 7.)* *(2026-09-27, §16u:
  §16r's grids had the flood fill's sign; re-measured with parity's, with no pre-smooth. The pumping check is
  step 7's, §16u.)*
- **Other force phases:** the prediction uses the elastic force alone (14d's note). *(Since §16p, the
  elastic and viscous forces.)*
- **The frame carry:** the current normal is carried from the pose at t to the pose at t + Δt; no test
  reaches a rotating obstacle closely enough to see an error in it.

**How this was checked (2026-09-25).**
- **The criteria came first,** with ten priors (the defects I already suspected) kept from the reviewers.
- **Round 1:** four cold reviewers raised about 20 distinct findings. Five priors hit outright and two in
  part. The reviewers covered:
  - the engine;
  - the tests and fixtures;
  - this record, reproducing its numbers;
  - the whole plan.
- **Round 1's largest findings:**
  - the lookup's error across the mandrel's nose–shank seam, found by three reviewers;
  - the lookup wrong in the outermost cell of every face;
  - the readouts of constrained nodes;
  - no test on G2;
  - the passages amended above;
  - G2 and the pre-smooth on the product.
- **Round 2:** one fresh reviewer read only round 1's fixes and found 5 problems, 4 created by those
  fixes. The 4 were two over-claims, stale friction numbers, and a test grid too symmetric to catch a
  swapped index. The fifth was a fix left incomplete. All five were cut or measured, not rewritten.
- **Every code fix has a test** that failed on its defect first.
- **No pass looked at:** the GPU, the product scan, K6's block, or flutter.

### 16p. 2b, the rest: friction's flutter, the silicone's damping, and the runs (2026-09-25)

**The Coulomb push first**, since K3 compares it and §16o left its shortfall open (0.893 at λ_a 1.1 and 0.786 at
λ_a 1.3, 10k). Several diagnostics (not kept) looked at it on the undamped solver, over the constant-speed phase.

- **What it is made of.** The ratio is friction's magnitude over μ_f Σf_n, times its axial share, less the change in
  the nose's geometric push between the run and its frictionless companion (10k):

  | | λ_a 1.1 | λ_a 1.3 |
  |---|---|---|
  | Σ\|f_t\| / μ_f Σf_n | 0.918 | 0.829 |
  | Σf_t,z / Σ\|f_t\| | 0.984 | 0.968 |
  | The geometric push's change, over μ_f Σf_n | −0.010 | −0.016 |
  | The ratio | 0.893 | 0.786 |

- **Sticking made most of the magnitude's loss.** 13 % and 29 % of the contact node-steps stuck, at 55–58 % of
  μ_f f_n. A sticking node was moving with the mandrel (its axial velocity within 4 % of the mandrel's), for a median
  of 3 steps.
- **What moved it, one thing at a time** (λ_a 1.1 unless noted):
  - the mesh: 0.893, 0.845 and 0.841 at 10k, 50k and 100k; at λ_a 1.3, 0.786 and 0.708 at 10k and 50k (no 100k run);
  - the step: 0.845, 0.824 and 0.820 at Δt, Δt/2 and Δt/4 (50k);
  - the precision: f64 read 0.886 (10k);
  - the loading time: 0.962, 0.893 and 0.840 at 5, 10 and 20 T_s (10k), and 0.915 at 5 T_s (50k);
  - five times the mass damping: 0.906 (10k);
  - the friction: 0.942, 0.959, 0.883 and 0.845 at μ_f 0.05, 0.1, 0.2 and 0.3 (50k);
  - Poisson's ratio (50k, μ_f 0.3): 0.726, 0.779, 0.816, 0.698 and 0.845 at ν 0.3, 0.4, 0.45, 0.475 and 0.49, with
    more sticking at lower ν (34 % at 0.3, 16 % at 0.49).
- **Two of those point different ways.** Written before running: a limit set by one step's friction impulse
  (μ_f f_n Δt/m, larger at lower ν, where the step is longer) would stick more at lower ν and less at a smaller step.
  Lower ν stuck more; a smaller step lowered the ratio 2.5 % (its sticking was not counted). What sets the sticking
  is not isolated.
- **The inner surface oscillated with friction.** At μ_f 0.3 the shank's mean axial velocity carried 0.28–0.35 V rms
  (V the mandrel's speed) between 120 and 300 Hz on the 10k and 50k meshes, peaking at 189–195 Hz, and 0.17 V at
  100k, with more above 300 Hz. At μ_f 0.1, and without friction at λ_a 1.1, that band carried 0.02–0.03 V (0.10 V
  on λ_a 1.3's frictionless 10k run).
- **Friction following the normal force drove it.** A diagnostic took friction's limit from a normal force smoothed
  over τ steps. At τ = 3 the band fell from about 0.35 to 0.12 V at 10k and stayed at 0.29 V at 50k; at 10 it read
  0.03 and 0.08 V; at 30, 0.03 V on both, and the ratio 0.975 and 0.981. Smoothing filters both the structure's
  modes and the mesh's, so which coupling it broke is not isolated.
- **The mesh's rings force the tube too.** Without friction, at λ_a 1.1, the shank rings at the rate the nose
  crosses the mesh's rings (V/Δz: 16.0 Hz at 10k, 32.9 Hz at 50k). At 10k and T = 10 T_s that rate sits on the
  tube's axial mode (about 16 Hz): 0.60 V rms, and 0.35 V at T = 20 T_s on its second harmonic, against 0.06–0.07 V
  off it. At λ_a 1.3 the 50k tube rang at its axial mode too (0.38 V at 17 Hz), off the ring rate; what excites it
  there, and whether the 10k ringing moved the 10k ratios, is not isolated.
- None of the plan's gates saw any of it: KE/IE over the constant-speed phase read 0.7–4.3 %.

**Is it the model's, or the discretization's?** (Jon asked for the chase before a fix.)
- **A second discretization reads it too.** The deleted penalty law (elastic-slip friction, trilinear lookup;
  `69805cbf`) read 0.868, 0.817 and 0.799 at 10k, 50k and 100k.
- **The half-space puts it outside the continuum's local problem.** The linearized sliding of an elastic half-space
  on a rigid flat, i a z² + f (1 + b² − 2ab) = 0 (z = c/c_s, a = √(1 − κz²), b = √(1 − z²), κ = c_s²/c_d²; derived for
  this record, not kept, and re-derived in review), has no growing root at ν 0.49 for f ≤ 1.0, and one above it (to
  f 3, in review). It
  reproduces the published boundaries: Renardy's f < 1, and Martins et al.'s flutter region as Yastrebov (2016,
  arXiv 1507.07334) draws it.
- **The finite tube's linearized sliding has growing modes.** A complex-eigenvalue analysis (linearized about the
  frictionless state at 0.5 T; contact nodes held on the mandrel; friction μ_f λ_n along the slip; a diagnostic, not
  kept):
  - at μ_f 0.3, an unstable pair at 135 Hz (11.1–11.3 and 10.4–10.8 /s net of mass damping, on the 10k and a
    4 × 40 × 23 tube), and 233 unstable modes below 500 Hz on the 10k tube, from 68 Hz up (177 Hz at 9.0 /s, and
    214 Hz at 10.1 /s in review; 21.3 /s at 190 Hz on the finer tube);
  - mesh-scale modes, the fastest at 0.48–0.60 of each mesh's top frequency (990, 1747 and 2315 Hz; up to 174 /s),
    which the half-space's result rules out for the continuum;
  - **and growing modes at μ_f 0.1 as well** (572 on the 10k tube; 42.5 /s at 1745 Hz; below 500 Hz up to 5.9 /s),
    where a run shows no band (0.02 V) and reads 0.965. So growth in this analysis does not by itself predict what a
    run does. What limits the growth in a run is not isolated.
- **Settled, and not.** The shortfall is converged in the step, and in the mesh at λ_a 1.1; a second discretization
  shares it; and the finite tube's linearized sliding is unstable at structural frequencies, as steady Coulomb sliding
  of finite bodies "generically" is (Nguyen 2003; Yastrebov's finite layer slides by stick-slip pulses at ν 0.49 from
  f 0.5, his lowest). No continuum solution of the tube was computed, and the ν sweep's disagreement stands.
- The model's only damping was mass-proportional: about 0.25 % of critical at 190 Hz.

**Decided (Jon, 2026-09-25): give the silicone its own damping.**
- **The data** (Ecoflex 00-30, fractional Kelvin–Voigt μ(ω) = μ₀[1 + (iωτ)ⁿ]):
  - Delory et al. (arXiv 2310.11396, plate-plate rheometry, over a range the text does not state): E₀ 69 kPa,
    τ 330 µs, n 0.32 *(2026-09-27: its data run from 0.1 to about 100 Hz, read off its Fig. 1B by a research
    round, so 190 Hz extrapolates the fit)*;
  - Croquette et al. 2026 (arXiv 2604.27722, guided waves over 2–300 Hz, against an Anton Paar MCR501): μ₀ 22.6 kPa,
    n 0.20 ± 0.04, τ 2.9 ± 1.3 ms, and a τ that differs from their rheometry's by up to 10×.
  - Their loss moduli at 190 Hz are 0.36 and 0.40 μ₀ (arithmetic). **Dragon Skin 10A: no data found.**
- **The model:** a deviatoric Kelvin–Voigt viscosity, σ_v = 2η dev D with D = sym(Ḟ F⁻¹), P_v = σ_v cof F, per
  element from the half-step velocities (`first_piola_viscous`, `tet4_viscous_forces`). It vanishes at rest, for a
  rigid spin and for a pure change of volume (`tests/viscosity.rs`).
- **η = 7 Pa·s** for the tube (`ECOFLEX_00_30_VISCOUS_TIME` = η/μ): the loss modulus at 190 Hz over ω, between the
  two fits' 6.9 and 7.5 Pa·s (arithmetic). What that choice gets wrong, against the fits (arithmetic):
  - it keeps the static storage modulus, so its loss tangent at 190 Hz is 0.36, against the fits' 0.22 and 0.18;
  - its loss grows as ω, theirs as ω^0.2–0.32: 41–51 % of theirs at 68 Hz, and 5–6× at 2 kHz, which damps the mesh's
    top modes hardest;
  - Croquette's error bars on n and τ put η at 5.2–10.3 Pa·s (arithmetic); Delory's fit gives 6.9.
- **The stable step.** Central differences with the damping force at the lagging half-step velocity are stable while
  4M − Δt²K − 2ΔtC is positive definite (derived; re-derived in review). So the loop estimates the top vector of
  M⁻¹(K + βC) at β = 2/Δt, and takes Δt = 0.9 · 2/ω (√(1 + ξ²) − ξ) with ω² = vᵀKv/vᵀMv and ξ = vᵀCv/(2ω vᵀMv)
  *(computed in the damping quotient's form since 2c, §16q)*.
  - The elastic top mode was not enough: a block at 150 Pa·s (ξ 0.38 there) blew up on the step it gives; at
    β = 2/Δt the iteration finds a vector (ξ 4.7) that binds first, and the loop's step is 0.535 of the elastic top
    mode's (`tests/viscosity.rs`).
  - The loop's step reads 0.9031–0.9038 of a dense critical step on a block at 0, 20 and 150 Pa·s
    (`tests/viscosity.rs`), and 0.9008–0.9103 over 60 blocks in review.
  - On the 10k tube (`tests/tube_release.rs`) the loop's step is +0.49 % above 0.9 of its own estimate's converged
    critical step at rest, and +0.07 % loaded, against a 2 % bar (a fifth of the 11 % margin 0.9 leaves, as §16e's
    bar was); review measured +0.12 % and +0.36 % at rest on 50k and 100k. With the loop's step taken without ξ,
    the same check reads +11 % and fails. On this tube the elastic top mode is nearly the binding one (+0.96 %
    above, with the start's second estimate removed), so the block tests carry that case.
  - No test pins β's factor of 2: at β = 1/Δt the step moves under 1 % (review).
  - A scratch run on the 10k tube: stable at 0.97 of the loop's estimated limit, marginal at 1.00 (balance 181 %),
    blown up at 1.03, as undamped.

**What the damping does to the Coulomb push** (10k, λ_a 1.1, μ_f 0.3, T = 10 T_s; the probe below, with η/μ):

| η (Pa·s) | 0 | 2 | 3.85 (the fits' tan δ at 190 Hz) | 5 | 7 |
|---|---|---|---|---|---|
| The Coulomb push | 0.893 | 0.935 | 0.957 | 0.976 | 0.982 |

At 50k it reads 0.986 at η 3.85, 5 and 7, and at 10k and λ_a 1.3, 0.958 at η 5 and 0.957 at 7 (the review's runs;
0.893 and 0.957 at 10k reproduced). **With the damping, the linearized analysis** (predicted before it ran) finds no
growing mode at μ_f 0.1, 0.3 or 0.6 on the 2 × 24 × 12 tube, and none at 0.3 on the 10k tube; on the 10k tube at
μ_f 0.6 it finds three (973 Hz at +41 /s, two at 56 Hz at +3–4 /s).

**The runs, on the damped solver** (`d2d5de3f`): `cargo run --release -p sim-soft-explicit --example tube --
<mesh> <case> <μ_f> <f32|f64> 20 0.2 <T/T_s> <stiffness> [η/μ]`, f32 unless noted, A/20, §15b's 0.2 s hold,
T = 10 T_s unless noted, `RAYON_NUM_THREADS=4`, M4 Pro. η/μ defaults to Ecoflex 00-30's; 0 reproduces the undamped
solver exactly (K2 +4.41 % in 12 694 steps at 10k, as before). The readings were first measured at `5ca8053d`; the
later commits change no reading.

| | 10k | 50k | 100k |
|---|---|---|---|
| K2, λ_a 1.1, ν 0.49 / 0.495 | +4.41 % / **+5.29 %** | +1.47 % / +1.60 % | +0.95 % / +1.00 % |
| K2, λ_a 1.3, ν 0.49 / 0.495 | +3.30 % / +3.78 % | +1.13 % / +1.20 % | +0.73 % / +0.75 % |
| The confined case | −0.18 % | −0.12 % | −0.06 % |
| The Yeoh case: K2; increment over neo-Hookean (oracle +4.22 %) | +4.71 %; +5.60 % | +1.66 %; **+4.75 %** | +1.10 %; +4.59 % |
| Coulomb push, λ_a 1.1 / 1.3 (μ_f 0.3) | 0.982 / 0.957 | 0.986 / 0.962 | 0.985 / 0.961 |
| Coulomb push, λ_a 1.1 / 1.3 (μ_f 0.6) | **0.934** / 0.963 | | |
| G2, deepest over all steps (grid); at the end (true surface) | 0.0–0.1 µm; 0.0–0.1 µm | 0.0–0.1 µm; 0.0–0.9 µm | 0.0 µm; 0.0–0.6 µm |

- **At T = 10 T_s the validity gates hold in every damped run, and K4 finds no inverted element** (the ladder's
  faster rungs are below). Over the 24 frictionless runs: λ_z within 0.23 %, KE/IE at most 0.05 %, the energy
  balance at most 0.03 %. Over the 11 friction runs, μ_f 0.6's included: KE/IE at most 0.005 % and the balance at
  most 0.05 % (λ_z moves 1.0–14.9 % with friction, so a friction run is not a K2 reading).
- **The damping moved no frictionless seated reading:** every K2, the confined case and the Yeoh case read as on the
  undamped solver to the printed 0.01 point; the 10k band's p/μ moved 0.128504 → 0.128529.
- **It moves the frictional seated state**, which depends on the path the tube took (10k, λ_a 1.1, μ_f 0.3; band
  p/μ and seated p95/μ):

  | η (Pa·s) | 0 | 0.875 | 1.75 | 3.5 | 3.85 | 7 | 14 | 28 |
  |---|---|---|---|---|---|---|---|---|
  | Band p/μ | 0.1184 | 0.1153 | 0.1145 | 0.1126 | 0.1124 | 0.1106 | 0.1106 | 0.1105 |
  | Seated p95/μ | 0.1679 | 0.1523 | 0.1516 | 0.1359 | 0.1336 | 0.1345 | 0.1345 | 0.1347 |

  Undamped against damped at 50k: band 0.1165 against 0.1080, p95 0.1374 against 0.1435; 3.5 and 7 Pa·s agree to
  0.05 % there (the review's runs; η 0, 3.85 and 7 at 10k reproduced). What in the path changes it is not isolated.
- **By the stop rule's measured 100k errors,** all four corners are within 5 % (+0.95, +1.00, +0.73, +0.75 %). 2d
  applies the rule.
- **K2's CI check moves to λ_a 1.1 at ν 0.495,** the worst 10k corner (+5.29 %, against 7 %), as 16i directs. At 10k,
  going from ν 0.49 to 0.495 moves K2 +0.88 and +0.48 points; at 50k and 100k, 0.02–0.13.
- **The Yeoh case passes 16h's rule:** at 50k the increment is off by 0.53 points of p_NH, 13 % of the increment,
  against 25 % (1.05 points). It is off by 1.38 points at 10k and 0.37 at 100k. Its absolute K2 is within 5 % at 50k
  and 100k.
  - *Added in 2c (§16q):* the table's increment is the band pressure's, p_Yeoh/p_NH − 1 on the same mesh, not
    corrected for the two runs' λ_z. Corrected to the oracle's λ_z through the two K2 errors,
    (1 + K2_Yeoh)/(1 + K2_NH) × 1.0422 − 1, it reads 5.64, 4.77 and 4.60 % (arithmetic on the table), and the verdict
    is the same: 13 % of the increment at 50k.
- **The product-level run** (`DRAGON_SKIN_10A`'s μ, 51 kPa, as a stiffness scale of 2.2174 with Ecoflex's η/μ;
  λ_a 1.3, ν 0.49, frictionless) reads +3.30 % and +1.13 % at 10k and 50k, as at 23 kPa.

**K3 passes.** f32 against f64:
- the band at 50k: the mean and all 11 pair-averaged ring levels agree to the printed 6 digits, both corners;
- the Coulomb push's reactions at 10k (μ_f 0.3, λ_a 1.1): 2.172312 N against 2.172310 N with friction, and 0.119912 N
  in both frictionless (the probe prints them). Undamped, f32 and f64 had differed by 0.65 %, and a rounding-level
  change of step had moved f32 by 0.25 %.

**The Coulomb push passes 15d.7's 5 % at μ_f 0.3** on every mesh and case above, and at μ_f 0.1 (0.985, 50k). At 10k
and λ_a 1.1, where the diagnostic looked, no node-step sticks now. **At μ_f 0.6 it fails at λ_a 1.1** (0.934).

**K5 fails on both readings.** §15a defines K5 as 50k → 100k, §16i as 10k → 50k:

| | Push peak, frictionless | Push peak, μ_f 0.3 | Seated p95, frictionless | Seated p95, μ_f 0.3 |
|---|---|---|---|---|
| λ_a 1.1, 10k → 50k | −40 % | −0.1 % | −16.4 % | +6.7 % |
| λ_a 1.1, 50k → 100k | −14.1 % | −1.3 % | +8.6 % | +7.9 % |
| λ_a 1.3, 10k → 50k | −25.5 % | −3.1 % | −2.5 % | +11.2 % |
| λ_a 1.3, 50k → 100k | −3.2 % | −1.0 % | +12.4 % | −2.2 % |

- The push with friction converges. The frictionless push is the nose's geometric push alone (0.15–1.3 N), the
  share a verdict's μ = 0 run reads (§15h); on the undamped solver it rose and fell as the nose crossed each ring of
  nodes at 10k and 50k, and at 10k fell to zero between rings (a diagnostic, not kept).
- On the undamped solver the most-squeezed 5 % of the contact area was one or two rings at the nose–shank seam,
  10.7–15 mm behind the tip, so the 95th percentile is whichever ring sits at the 5 % boundary (the same diagnostic).
- §15a makes K5 a gate on the verdict's design: D1's readings, or the lip radius, are revisited before step 7. On
  the undamped solver the frictionless reading's spot was the mandrel's seam, not the tube's entry edge the lip
  radius was named for; where the damped frictional reading fails is not measured. §15g's list for steps 6–9 and fit
  plan U16 carry it. *(2026-09-26, §16s, a diagnostic: on the damped solver, the frictionless percentile sits at
  the seam and the frictional one at the seam and the entry edge, and the frictionless push ripples as the nose
  crosses each ring. K5 passes from 50k to 100k on the readings that replaced them.)*

**The loading-time ladder** (§15c; 10k, frictionless, halving from 10 T_s):
- The band moved at most 0.04 points down to 0.156 T_s, and KE/IE stayed at most 1.9 %.
- **The energy balance stopped it:** at λ_a 1.1 it read 1.60 % at 0.3125 T_s and 4.71 % at 0.156 T_s, against 1 %.
  The last valid rung is **0.625 T_s** (balance 0.63 %): the mandrel at 1.80 m/s, v/c_s 0.39 (arithmetic).
- On the undamped solver KE/IE stopped it at 1.25 T_s (5.18 %), so the rung was 2.5 T_s: the rung holds for
  Ecoflex's η/μ.
- The push is not in the ladder's rule. With friction (μ_f 0.3, λ_a 1.1) the push peak read 4.519–4.601 N from
  10 T_s to 0.625 T_s, and the seated p95 moved at most 0.7 %. Frictionless, the push peak moved up to 12 % over the
  same rungs, then rose to 0.40 N and 1.07 N at 0.3125 and 0.156 T_s (λ_a 1.1).

**Stiffness scaling holds** (15d.10; 10k, μ and 2μ with η/μ held): K2 and the seated p95 are unchanged, and the push
peak scales by 2 × (1 + 1.64 %) and 2 × (1 + 0.80 %) frictionless, and 2 × (1 + 0.89 %) at μ_f 0.3 (p95 +0.36 %).
So by §15h a verdict is 3 runs.

**The loaded step factor** (the smallest step in a run over its rest step): **0.977**–1.000 over the damped runs
(0.946 on the undamped solver). 2d applies 0.977.

**The power iteration's elastic accuracy** (§16e's re-check at 50k, and 100k; a diagnostic not kept): the loop's 100
iterations read ω_el² −0.2 % to −0.8 % against a converged f64 estimate, at rest and loaded, on 10k, 50k and 100k. The
damped step's accuracy is above.

**The cost of the damping** (idle machine, 4 threads):
- Steps: ×1.06–1.22 at 10k, ×1.09–1.22 at 50k, ×1.23–1.39 at 100k. The rest step shrinks 12.5 %, 19 % and 28 % (λ_a
  1.1): Kelvin–Voigt's damping grows with frequency, and so does the top of a finer mesh.
- A step costs 0.49, 1.56 and 2.78–2.80 ms at 10k, 50k and 100k, against 0.33–0.36, 0.98–1.00 and 1.68–1.70 ms
  (§16o): ×1.36–1.67. The viscous phase evaluates every element a second time whatever η is: at η = 0 a 10k step costs 0.46 ms.
  *(Amended in 2c, §16q: the CPU executor now skips the viscous phases when no material is viscous. The 0.46 ms was
  read on an idle machine; a reviewer read 0.49 ms on a loaded one.)*
- So a 100k K2 run takes 124.5 s, against 53.3 s (§16o). The estimates take 12.6–15.8 % of a run.
- §15c's K1 arithmetic (29k steps, 3.7–4.1 ms per step) assumed 10 T_s and no damping; both changed (§15c's note).

**Open, for later steps:**
- **Friction above μ_f 0.3.** The damped 10k tube fails 15d.7 at μ_f 0.6 (0.934), and §5c's ranges reach 1.0 (dry) and
  2.0 (water alone) *(re-sourced 2026-09-27: on skin to about 1.2, tacky analogs above 2; §5c)*. No gate catches
  flutter: the Coulomb push's shortfall led to it, and nothing reads the contact nodes' kinetic energy against a bar.
  §15g's list for steps 6–9 carries it.
- **The damping's form and value.** Kelvin–Voigt's loss grows as ω; a Maxwell branch in parallel (one stored stress
  per element) would bound it above its rate, and match the fits' stiffening better. One fit's error bars span
  5.2–10.3 Pa·s, and the frictional seated state moves with η below 7 Pa·s. Not decided; step 4 (the GPU layout) and 2d
  (the budget) are where its cost matters.
- **The viscous phase is a second pass** over the elements; fusing it with the elastic forces (they share F, J and
  cof F) is step 4's.
- **Dragon Skin 10A's loss is not known**, and the product is Dragon Skin (fit plan U15).
- **The fits' storage stiffening** at insertion rates (12–25 % and 43–68 % from 1 to 10 Hz, arithmetic) is not in the
  model (fit plan U15).
- **2c (K6)** sets its viscous time (§16b's note); 2d's inputs are in §16j's note. *(2c: η = 0, §16q.)*

**How this was checked.**
- **The criteria came first,** with twelve priors kept from the reviewers.
- **Round 1:** four cold reviewers (the engine and the step; the tests and the oracle; this record, reproducing its
  numbers; the whole plan) raised about 30 findings. Five priors hit and three in part.
- **Round 1's largest findings:**
  - the frictional seated state moves with the damping (this record had said no seated reading did);
  - the linearized analysis quoted only where it agreed, and the ν sweep left out;
  - the damped μ_f 0.6 push (0.934), which this record had inferred from the analysis alone;
  - the CI check of the power iteration testing an estimate the loop no longer made;
  - K4 and the validity gates unreported;
  - K5 read on one reading;
  - §16j, §14 and §16o passages the damping made stale;
  - η's uncertainty and its loss tangent.
- **Every code fix has a test** that failed on its defect first. The review's own runs are cited as the review's;
  where they were reproduced, the text says so.
- **Round 2:** one fresh reviewer read only round 1's fixes and found 11 problems, 7 of them created by those
  fixes (an overclaimed gate sentence, a test bar tightened past what correct code reads, a step bar reused
  without re-deriving it, K5's location and remedy restated, η's range stretched past its source, a one-mesh
  sensitivity, and a misread step result). They were cut or re-measured, not rewritten; with round 2 finding
  mostly what round 1's prose wrote, no third pass was run.

### 16q. 2c: K6 (2026-09-25)

2c holds what 16c gave it: the Cattaneo–Mindlin block and its readouts as a fixture (`fixtures::partial_slip`), its
runs, and a coarse K6 in CI. It also takes #970's six follow-ups.

**K6 passes.** On §16b's block at a/h 50, the worse row's stick zone is within 0.0135a of Cattaneo–Mindlin while the
load rises and 0.0076a of Mindlin–Deresiewicz while it falls, against 0.03. Both companions differ from it by just
over 16b's 0.01a, so K6 is judged on them too: 0.0218 at R = 200a, and 0.0122 on the larger block. The worst judged
reading is 0.0218. Every input of the stop rule (§15g step 2, 16i) is now measured and passes: K2 at 100k
(within 5 % at every corner), K3, K4 and the Yeoh case (§16p), and K6. K4 reads no inverted element in any run §16p
and this section report, and none on the loading-time ladder's valid rungs either (5, 2.5, 1.25 and 0.625 T_s; 10k,
λ_a 1.1 and 1.3 frictionless and λ_a 1.1 at μ_f 0.3; `cargo run --release -p sim-soft-explicit --example tube --
10k <0|2> <0|0.3> f32 20 0.2 <T/T_s> 1`, rerun in 2c at `d6ad283f`). 2d applies the rule, and reports
the budget against D4, which revises the speed plan rather than stopping it.

**Decided in the build:**
- **The stick zone is read from the friction deficit,** μ_f f_n − f_t, not the ratio f_t/(μ_f f_n) 16b proposed.
  Each edge is where the deficit squared, extrapolated linearly from the zone's last two nodes, reaches zero; it
  moves at most to the next node out.
  - The unit test 16b asked for feeds the closed forms, sampled at the nodes over 25 mesh offsets and 25 load
    fractions, to the readout (`tests/partial_slip.rs`). At a/h 50 the deficit reads within 0.00076a loading and
    0.00046a unloading, against the bar's 0.005a; the ratio reads 0.0049a and 0.014a. At a/h 25 the deficit reads
    0.0030a and 0.0018a, at a/h 12 0.012a and 0.0079a.
  - That is the rule's resolution on point samples of the closed forms, not a run's (the budget's note, 16b).
- **A node counts as sticking when its deficit is above 5 % of its own limit** (`STICKING_DEFICIT`). A node in the
  slip zone that sticks on some of an interval's steps reads a small deficit, not none: with a floor of 10⁻⁴ of the
  row's largest limit, the first a/h 12 run read the whole contact as sticking at every load (a scratch run, not
  kept).
  - The deficits do not fall into two groups: on a coarse K6 run a review counted node readings in every percent
    band from 1 % to 10 % of the limit. Moving the floor from 0.01 to 0.10 left that run's largest errors as they
    were, and moved single unloading readings by up to 0.015 (the review's, not kept).
- **The loading** (`PartialSlipRun::plan`), in the block's first shear period T = 4·depth/c_s (8.6 ms):
  - the press at 0.004a/T, the push and the return at 0.0015a/T;
  - each leg reaches its speed over T and stops over T, then holds for 2T;
  - a leg ends on a force read every monitor interval. Its stop starts when the travel the stop takes would reach
    the end, at the rate the reading grew over the last half T (the square of the half-width while pressing). A
    rate over one interval stopped the press at 0.92a at a/h 12 (a review's run), as the contact's edge moves a node
    at a time; it now stops at 1.002a there and 1.004a at a/h 50;
  - mass damping ξ 0.05 at the block's first shear frequency, as the tube's at its own.
- **K6 runs elastic, η = 0.** The closed forms are elastic. And integrated explicitly, the viscosity's damping grows
  as 1/h²: with Ecoflex 00-30's η/μ the stable step at the start falls 19.1× at a/h 12 and 80.0× at a/h 50
  (`k6_release`'s measurement), so a viscous K6 at a/h 50 would take about 35 hours (arithmetic: 80 times the
  main run's 26 minutes of steps). A viscous companion at a/h 12 measures what η does to the stick zone at this loading (below).
- **The closed forms are in the fixture** (`stick_while_loading`, `stick_while_unloading`, with their sources), not
  only in the test (16g): the probe and the CI check judge by them too.
- **The cylinder's grid** holds the strip of body-frame points within 0.1a of the plane tangent at its lowest point,
  and within √(0.1aR) of that point along it. Past a face the lookup reads the face, so the box is widened on the
  faces where that moves a point towards the cylinder, until every point moved there lies 0.1a below the plane. It
  holds while the block's top stays within 0.05a of the plane beyond the strip (K6 presses 0.014a), and for turns
  under 90°. The test checks every node at 30°, ±10° and 60°, and fails with any one of the three widenings removed.
- `fixtures::grid::bake` bakes any distance function; the mandrel and the cylinder share it.
- **The CPU executor got faster where K6 needed it,** with identical output over a whole a/h 12 run (measured then,
  4 threads, not kept):
  - the grid's 64 values are fetched a row at a time: 205 s → 187 s;
  - the viscous passes are skipped when no material is viscous: 187 s → 142 s. At η = 0 each re-estimate's 100
    iterations had also evaluated the viscous forces.
  - A review's bitwise A/B against `main`'s executor agreed on every state and step size, at f32 and f64, elastic and
    viscous (not kept).

**The runs** (`cargo run --release -p sim-soft-explicit --example partial_slip -- <a/h> <f32|f64> <R/a> <block/a>
<η/μ> <slowdown> table`, f64 and the plan's loading unless noted, `RAYON_NUM_THREADS=6`, M4 Pro, two runs at a time,
at `df957bec`; the two marked † at `2590634a`, which changes only a leg that does not end. `… -- compare <table>
<table>` gives the largest difference from the reference run: in c/a while the load moves, at the same load
fraction; and within 0.02 of each leg's largest judged fraction, in each row's mean error from the closed form):

| Run | K6, rows 0/1: loading; unloading | Against | While the load moves | At the peaks |
|---|---|---|---|---|
| **The plan's, a/h 50** (`50 f64 100 10 0 1`) | **0.0135/0.0079; 0.0076/0.0042** | | | |
| Half the rate (`… 0 2`) | 0.0113/0.0036; 0.0074/0.0044 | the plan's | 0.0033; 0.0024 | 0.0029; 0.0010 |
| R = 200a (`50 f64 200 10 0 1`) | 0.0218/0.0123; 0.0113/0.0041 | the plan's | 0.0103; 0.0073 | 0.0050; 0.0044 |
| R = 200a, half the rate† (`50 f64 200 10 0 2`) | 0.0128/0.0070; 0.0115/0.0041 | the plan's | 0.0089; 0.0067 | 0.0033; 0.0032 |
| | | R = 200a | 0.0049; 0.0045 | 0.0083; 0.0032 |
| A 30a × 15a block (`50 f64 100 15 0 1`) | 0.0122/0.0082; 0.0096/0.0023 | the plan's | 0.0111; 0.0040 | 0.0037; 0.0023 |
| a/h 25 | 0.0194/0.0088; 0.0185/0.0072 | the plan's | 0.0133; 0.0132 | 0.0031; 0.0099 |
| a/h 12 (CI's mesh) | 0.0322/0.0281; 0.0406/0.0173 | | | |
| a/h 12, half the rate‡ | 0.0324/0.0292; 0.0435/0.0197 | a/h 12 | 0.0130; 0.0159 | 0.0094; 0.0055 |
| a/h 12, the step 19× smaller‡ (`… safety=0.04712`) | 0.0322/0.0286; 0.0440/0.0201 | a/h 12 | 0.0157; 0.0162 | 0.0113; 0.0068 |
| a/h 12, Ecoflex 00-30's η (`12 f64 100 10 0.00030434782608695654 1`, 7/23 000) | 0.0332/0.0357; 0.0483/0.0252 | the step 19× smaller | 0.0180; 0.0179 | 0.0106; 0.0102 |
| a/h 12, f32‡ | 0.0823/0.0596; 0.0667/0.0835 | a/h 12 | 0.0830; 0.0931 | 0.0220; 0.0121 |
| f32, a/h 50 | 0.5586/0.5620; 0.2376/0.2394 | | | |

‡ At `027ab9dc`, which adds the step's safety fraction to the run; the plan's run is unchanged by it. `compare` also
prints each row's mean signed difference while the load moves; the bullets below quote it.

- **Every f64 run is valid by the generic gates:** KE/IE at most 1.54e-4 over the push and return, the energy
  balance at most 6.9e-5, no inverted element, the deepest penetration at most 5.2e-12a. The contact nodes' kinetic
  energy, the watch for flutter, peaks at 7.7e-16 J (a/h 12).
- **The press ends at 1.001–1.004a** (1.021a at R = 200a, 0.990a with the viscosity), 0.0135–0.0139a deep at
  R = 100a (0.0159a on the larger block), and the push at 0.786–0.805 μ_f P. The return's hold settles just past
  the band (at 0.8035 on the plan's run), so its judged readings are those on the approach.
- **The rate ladder stops at the plan's rate:** half the rate moves c/a by at most 0.0033, inside the 0.005a the
  budget gives the rate (16b's note says why that bar). Matched by fraction over the holds as well, the difference
  had read 0.0055 or 0.0080 for the same pair, as a tie fell; a quarter-rate run was queued on the 0.0055 and cancelled
  once the holds were compared apart. One halving bounds the change from one rung to the next, not the rate's own
  error, whose form is not known.
- **Both companions differ by more than 16b's 0.01a, so K6 is judged on each too, and passes on each.**
  - R = 200a at the plan's speeds differs by up to 0.0103, and K6 reads 0.0218 on it. Its legs travel about half as
    far, so it pushes to 0.78 in 3.1T against the plan's 5.3T: a faster loading as well as a smaller strain. At half
    its rate (6.2T) it differs by up to 0.0089.
  - At R = 200a, halving the rate moves c/a by 0.0049 while the load moves and 0.0083 at the peaks: finite strain is
    not separated from rate there.
  - The 30a × 15a block, whose legs take as long as the plan's (5.6T), moves c/a by up to 0.0111, evenly while loading
    (0.0075–0.0111 in every 0.05 band of the load; each row's mean by +0.005) and by up to 0.0040 while unloading. K6
    reads 0.0122 on it. Its mass
    damping is set at its own first shear period, two thirds of the plan's coefficient; that change's share is not
    isolated.
- **The element:** a/h 25 reads 0.0133 from a/h 50 while the load moves. The worse row's errors, loading and
  unloading, are 0.0322 and 0.0406 at a/h 12, 0.0194 and 0.0185 at a/h 25, and 0.0135 and 0.0076 at a/h 50.
- **Row 0 (y = 0) reads low throughout** (−0.009 on average while loading at a/h 50, row 1 −0.003). The Kuhn split is
  not symmetric front to back (16b); what makes row 0 lower is not isolated.
- **The press leaves a tangential load:** Q/(μ_f P) = −0.0052 at a/h 50 and −0.0122 at a/h 12 before any push, where
  symmetry would give none. The push's fraction is measured from zero, as
  16b defines it. Measured from the press's load instead (arithmetic on the tables), the loading errors read
  0.0078/0.0086 at a/h 50 (from 0.0135/0.0079) and 0.0213/0.0413 at a/h 12 (from 0.0322/0.0281): the rows trade
  places, and the verdict is the same.
- **The viscosity, at a/h 12, moves one row by about 0.01.** With η the stable step is 19× smaller, so the viscous
  run is compared with the elastic one at the same step. At a/h 12, halving the rate or cutting the step alone moves
  single readings by 0.013–0.016 and each row's mean by at most 0.004. Against its own step, η moves single readings
  by up to 0.018, row 1's mean by +0.0096 while loading and +0.0053 while unloading, and row 0's by at most 0.002. So
  K6's loading is not slow enough for η to vanish on every row; K6 runs elastic, as its closed forms are. The a/h 50
  mesh was not run with η. The product's frictional states are rate dependent through η on §16p's own evidence
  (the seated readings, fit plan U15) *(2026-09-26, §16s: those were the 95th percentile's; how the 1 cm² patch
  depends on η is not measured)*.
- **f32 cannot resolve K6:** 0.56 while loading at a/h 50, with the deepest penetration 5.9e-7a against f64's
  2.6e-13a. §16b predicted it: a slipping node moves 1e-8a to 1e-7a per step, below f32's spacing at coordinates of
  order a. At a/h 12, f32 reads 0.084, twice f64's 0.041. The tube passes K3 (§16p),
  which compared f32 with f64 only frictionless and in steady sliding; §15g step 5 now compares them on a frictional
  seated state.

**The mutations** (16b's three, one at a time, each an edit of `kinematic_contact` in `src/shared/contact.rs`:
`sticking` always true; the anchor kept while slipping; the limit times 1.1; then the same commands):

| Mutation | a/h 50: rows 0/1, loading; unloading | CI (a/h 12, bar 0.1): the worse row |
|---|---|---|
| An anchor that never releases | 0.2989/0.3208; 0.2135/0.1918 | 0.389 |
| An anchor not dragged while slipping† | 0.8797/0.8813; none (the push never reached its peak) | 0.797 |
| A friction limit 10 % high | 0.0649/0.0672; 0.0164/0.0241 | 0.075 (passes) |

- Each fails K6 at a/h 50. The CI check fails under both anchor mutations and cannot see the friction limit, as 16b
  expected (§16b's arithmetic: a 0.075 shift). That is covered at the law's level.
- **16b's expectations were partly wrong:**
  - the anchor that never releases does not read c/a = 1: the readout counts a node held past its limit as
    slipping (its deficit is negative), and the zone it reads fails by 0.32;
  - the undragged anchor fails while loading too, which 16b thought only unloading could catch. At a/h 50 the push
    never reaches its peak: Q rises to 0.54 μ_f P, falls back and holds near 0.47 while the cylinder moves on, until
    the leg is given up after 20T, and the energy balance reads 1.2 %. Why is not isolated.
- **The CI check** is `tests/k6_release.rs`, in tests-release and outside coverage. It also requires the press to end
  within 0.99–1.02a and the push to reach 0.78 μ_f P: the one-interval stop rule fails the first (0.921a) and the
  undragged anchor the second (0.475). It took 74 s alone at 6 threads, and 315–343 s beside two a/h 50 runs.
  `tests/partial_slip.rs` runs K6's whole path at a/h 4 in the debug and coverage set.

**#970's follow-ups:**
1. The start's second estimate was gated on the elastic top vector's ξ > 0, not on the material; the start now always
   estimates twice (`Stepper::estimate`). For an elastic material the two agree, and the executor skips the viscous
   iteration. No test builds a viscous material whose elastic top vector has no damping; the estimate count
   catches the old gate.
2. `TopMode` carries the viscous damping quotient γ = vᵀCv/vᵀMv instead of ξ, and the step is
   0.9 · 4/(γ + √(γ² + 4ω²)): the same as 0.9 · 2/ω (√(1 + ξ²) − ξ) where ω² > 0; 0.9 · 2/γ at ω² = 0, where the old
   form gave no step; and where γ² + 4ω² < 0, where the vector sets no limit at all, none, which stops the loop
   (`tests/executor.rs`).
3. §16p's per-step cost is ×1.36–1.67, from its own times (§16j's note said ×1.4–1.65).
4. §16p's Yeoh increment is the band pressure's; corrected to the oracle's λ_z it reads 5.64, 4.77 and 4.60 %, and
   the verdict is the same (§16p's note).
5. `the_energy_balance_holds_with_viscosity` ran at a viscous loss of 2.1 % of the peak internal energy, so a 40 %
   error in the viscous work passed. At 300 Pa·s the loss is 30.8 %, and with the viscous work 5 % short (an edit of
   the executor) the balance read 1.54 %, which fails it.
6. §16p's 0.46 ms at η = 0 is annotated: read idle (a reviewer read 0.49 ms loaded), and the executor now skips the
   viscous phases there.

**Open, for later steps:**
- **f32 and friction.** f32 does not resolve K6's partial slip. Step 4's GPU runs f32; §15g step 5 now compares f32
  with f64 on a frictional seated state, on the tube and then on `base_mold`. *(On the tube f32 passes, §16r.)*
- **The viscosity and friction.** At K6's loading, Ecoflex's η moves one row's mean stick zone by about 0.01a at
  a/h 12, and integrated explicitly it shrinks the step as 1/h² (fit plan U15). The damping's form is still §16p's
  open item.
- **The readout on a run** is not separated from the element: the a/h 25 companion measures the two together.
- **Not isolated:** why row 0 reads lower than row 1; why the press leaves a tangential load; why the undragged
  anchor fails while loading.

**How this was checked.**
- **The criteria came first,** with twelve priors kept from the reviewers.
- **Round 1:** four cold reviewers (the engine and fixture; the readouts and tests; this record, reproducing its
  numbers; the whole plan) raised about 34 findings. Five priors hit and three in part.
- **Round 1's largest findings:**
  - the probe ran a block twice §16b's width, so every recorded run was redone;
  - the press's stop fired on a one-interval rate;
  - the readout's resolution holds on sampled closed forms, not a run;
  - the edge reader's clamp and its run around the peak were pinned by no test;
  - R = 200a had to be judged by 16b's rule;
  - three of 16b's rules were applied differently from their text;
  - the viscous companion changed the step as well;
  - f32's friction question had no home in the plan.
- **Every code fix has a test** that failed on its defect first.
- **Round 2:** one fresh reviewer read only round 1's fixes and found 13 problems, 12 of them created by those fixes:
  prose that claimed more than its referent, a column described as what it was not, a rule said to be pre-registered
  that was written after the data, citations to this section for evidence it did not hold, and one gap in the CI
  check (the press's end and the push's peak, now asserted). The prose was cut, not rewritten. Every number and
  verdict it re-derived reproduced; with round 2 finding mostly what round 1's prose wrote, no third pass was run.


### 16r. 2d: the product's budget, and the stop rule (2026-09-26)

2d holds what 16c gave it: `base_mold`'s measurements of §15g step 2, run locally (16j), and the stop rule
applied. It also runs the K6 rung that §16b's ladder text left due (§16b's 2c note), and §15g step 5's f32
comparison on the tube's frictional seated state, on the CPU.

**What is written here** (Jon, 2026-09-26): ratios and verdicts. An element count at a known element size gives
the wall's volume, and a step count at a known step and speed gives the insertion's length. So the element count,
the step, the step counts and the loading time stay on the machine that ran them, in place of 16j's "counts,
steps and times".

**The stop rule: proceed to step 3.**
- K2 is measured at 100k, so that decides (16i): within 5 % at every corner, +0.95, +1.00, +0.73 and +0.75 %
  (§16p).
- K3, K4 and the Yeoh case pass (§16p). K6 passes (§16q), and at a quarter of the plan's rate (below).
- D4 is met as meshed, below. Had it missed, the speed plan would be revised before any GPU work (§15g step 2,
  16j); the quality gates are not loosened for it (§9 decision 12).

**The K6 rung.** §16b's ladder text stops the rate when a halving moves c and m by no more than the readout's
resolution, which 2c's unit test put at 0.00076a; 2c stopped at the budget's 0.005a, a bar chosen after the data
(§16b's 2c note). Pre-registered before the runs: K6 passes at this rung if its worst reading is within 0.03; a
quarter against half the rate is read against both bars; no further rung unless that moves more than 0.005a and
the rung's K6 is within 0.005 of its bar. At a quarter of the plan's rate (`cargo run --release -p
sim-soft-explicit --example partial_slip -- 50 f64 100 10 0 4 table`, at `edf09a09`, `RAYON_NUM_THREADS=6`,
beside the half-rate rerun):
- **K6 passes:** 0.0109/0.0034 while loading and 0.0072/0.0047 while unloading (rows 0/1), within 0.03. KE/IE
  3.6e-6, the balance 4.1e-6, no inverted element.
- **Against half the rate** (`… -- compare <half> <quarter>`), c/a moves by at most 0.0022 while loading and
  0.0027 while unloading as the load moves, and by 0.0011 and 0.0002 at the peaks: inside the budget's 0.005a,
  not inside 0.00076a. So 16b's rate line holds from half the rate on; the ladder's own text would halve again.
  By the pre-registration, no further rung.
- The half-rate run, rerun at `edf09a09`, reproduces 2c's table row for row.

**f32 on the tube's frictional seated state** (§15g step 5's note, run here on the CPU). Pre-registered before the
runs: K3's 0.5 % on the seated 95th percentile. 50k, μ_f 0.3, Ecoflex 00-30's η/μ, at λ_a 1.1 and 1.3, each at
10 T_s (K3's loading) and at the ladder's 0.625 T_s (`cargo run --release -p sim-soft-explicit --example tube --
50k <0|2> 0.3 <f32|f64> 20 0.2 <10|0.625> 1`, `RAYON_NUM_THREADS=4`, at `edf09a09`):
- f32 and f64 print the same p95 to five decimals in three of the four, and 0.39628 against 0.39629 in the fourth
  (λ_a 1.3 at 10 T_s). The band pressure
  agrees to its six printed decimals in all four, and the push with friction to 4e-5 relative. Every run is valid:
  KE/IE at most 0.00 %, the balance at most 0.10 %, no inverted element.
- **f32 passes** here, where it failed K6's partial slip (0.56 against 0.03, §16q, where §16b's arithmetic had
  put a slipping node's step below f32's spacing). It passes with the damping as it stands; undamped, f32 and f64
  had differed by 0.65 % on the Coulomb push (§16p), so a change of the damping's form or value (fit plan U15)
  repeats it. Step 7 still repeats the comparison on `base_mold`.

**The product's budget** (`RAYON_NUM_THREADS=4 cargo test --release -p cf-sim-research explicit_budget --
--ignored --nocapture` on an idle machine, the scan local, at `3af4024c`; later commits change only the module's
tests and comments; `tools/cf-sim-research/src/insertion_sim/explicit_budget.rs`) *(2026-09-27, §16v: since, the
mesher's Parity Rule fix changes the wall's connectivity, and the canal truth is signed by parity; the old wall's
ν 0.49 press re-reads 0.28 / 0.42 elastic / viscous)*:
- **h_K2 = 2.20 mm**, set by λ_a 1.1 at ν 0.495 (+5.29 % at 10k and +1.60 % at 50k, §16p's table) by the stop
  rule's model fitted per corner. It lies between the 10k and 50k meshes (h 2.26 and 1.31 mm).
- **The wall** is the old path's `build_insertion_geometry`: the scan decimated to 2 500 faces, a flood-filled
  grid at 0.75 of the lattice pre-smoothed by a cell, and isosurface stuffing, at the lattice that gives h within
  2 % of h_K2 (0.989 h_K2). Its canal is smooth: the poured plug's ridges and texture are not in `SimDesign`.
- **The material:** Dragon Skin 10A at 25 % Slacker resolves to Shore 00-30, which the catalog gives Ecoflex
  00-30's μ and C₂. The catalog's λ is 4μ (ν 0.4, §5b), so the lowering sets λ from μ at ν 0.49 and at 0.495,
  where K2 was judged. Each element keeps its catalog density; no node is held (the product's boundary options
  are step 6's).
- **K1's per-step budget:** 2 minutes over the damped 100k tube's steps at the ladder's rung (0.625 T_s, then §15b's
  0.2 s hold), from its rest step and the loaded step factor 0.977: 14.4 ms a step. A run at that rung took 2.7 %
  fewer steps (arithmetic; its smallest step was 0.998 of the rest step; `tube -- 100k 0 0 f32 20 0.2 0.625 1`),
  so the budget errs short. §15c's 3.7–4.1 ms assumed 10 T_s.
- **The loading:** the tube's rung speed, v/c_s 0.389, in the product's innermost material, over its insertion path
  and §15b's 5 mm start gap, then §15b's 0.2 s hold (on the tube at the rung, 76 % of a run; arithmetic).
- **The step's accuracy** (§16e, in §16p's form): the loop's step within 0.001 of 0.9 of the converged critical
  step on every model checked (as meshed and projected, elastic and viscous, ν 0.49), against 0.02.

A press is 3 runs (stiffness scaling, §16p). Its time over D4, at K1's per-step budget scaled by element count,
and on the CPU: the f32 executor on 4 threads, timed over 1 000 steps from rest with the scan 1 m clear, so no node
in contact, and one of the loop's re-estimates (a run makes one every 500 steps):

| | ν 0.49, at K1 | ν 0.49, on the CPU | ν 0.495, at K1 |
|---|---|---|---|
| Elastic | 0.29 | 0.045 | 0.40 |
| Ecoflex 00-30's η/μ × 0.74 | 0.39 | | 0.49 |
| Ecoflex 00-30's η/μ | 0.43 | 0.097 | 0.53 |
| Ecoflex 00-30's η/μ × 1.47 | 0.53 | | 0.61 |

- **As meshed, a press takes 0.29–0.61 of D4 at K1's rate.** *(2026-09-27, §16v: read before the mesher's Parity Rule
  fix. Step 7's wall, at the 5 mm inset: 0.420 viscous at ν 0.49, 0.594 at ν 0.495 with the viscosity × 1.47.)* The η range is
  Croquette's error bars' 5.2–10.3 Pa·s for Ecoflex (fit plan U15); Dragon Skin 10A's is not known.
- The viscosity cuts the step to 0.672 of the elastic one at ν 0.49 and 0.759 at 0.495. The tube's 10k mesh, in
  the same material at the same η/μ and about the product's h, reads 0.875 (§16p). What in the meshes makes the
  difference is not isolated.
- **At the 50k tube's h** (1.31 mm; D1's readings did not converge on the tube, §16p's K5; *§16s: the readings
  as now read move at most 1.3 % from 50k to 100k, and the frictionless patch and geometric share up to 38 % from
  10k to 50k, so the element size they need on the product is open*), a press takes 2.3 of D4 elastic and 4.8
  viscous at K1's rate, and 0.26 and 0.84 on the CPU (ν 0.49, Ecoflex's η/μ).
- **What the D4 verdict rests on:** the wall as meshed (projected at a floor of 0.5, a viscous press takes 1.03 of D4
  at K1's rate, below; U17) *(2026-09-27, §16v: step 7's wall projects nothing)*; the GPU meeting K1 exactly (measured in step 5), with its per-step cost scaling with
  element count, which a GPU's fixed cost a step need not; the tube's v/c_s, loaded step factor, hold and start
  gap, where the product's own seated window is D1's (step 7); Ecoflex's η/μ; 3 runs a press (a run at the
  pairing's nominal corner would make 4, §15g's outline); no node held; and h_K2, set by K2 alone. *(2026-09-27,
  §16w: step 7's first run is mounted. Held nodes cannot shorten the stable step: its condition, `4M − Δt²K − 2ΔtC`
  positive definite, holds on a principal submatrix whenever it holds on the whole, and on `base_mold` the mounted
  rest step is the unheld one's. The mount's confinement is step 7's to read.)*
- **K1's own tube on the CPU:** the 100k run at the rung above took 65 s of wall-clock (f32, 4 threads, contact and
  the loop's estimates included, on a machine running other jobs), against K1's 2 minutes. At 10 T_s the same
  mesh took 124.5 s (§16p), so it depends on the loading time step 5 sets for K1. The product's CPU figures above
  leave contact out; what contact adds on the product is not measured.
- The CPU figures bear on whether steps 3–5, the GPU, come before the quality items 2d measured (fit plan U15,
  U17, U18) and K5 (U16). That is Jon's call; the stop rule proceeds to step 3 either way. *(Jon, 2026-09-26:
  the quality items first, K5 first of them; §16s.)*

**The surface bias.** *(2026-09-27, §16v: re-read against a parity-signed truth, the as-meshed percentiles below
read the same; what sets them is isolated there, and step 7's wall is meshed so that no node needs projecting, so the
projections below are not its cost.)* The canal nodes are the wall's boundary nodes within two element sizes of the true canal
surface, the cap-stripped scan's exact distance at the inset (the mesher offsets the cap-stripped scan near the
mouth), and more than one element from a cap plane. No boundary node away from the caps lies two to three
element sizes off. Offsets are in element sizes, negative into the canal:
- **As meshed:** 49 % sit inside the true surface by more than 0.01 h and 47 % outside; mean −0.04 h, 5th
  percentile −0.50 h, 95th +0.32 h, worst 1.25 h. At h_K2, half an element is about a fifth of the inset
  (arithmetic).
- **Not the decimation:** meshed at the same lattice from the undecimated scan, the 5th and 95th percentiles are
  −0.50 h and +0.32 h again, the worst 1.23 h. Between the grid, its pre-smooth and the stuffing, what sets them is
  not isolated.
- **Projected** onto the true surface (steps along the distance's gradient until on it), each node only as far as
  keeps every incident element above the floor's share of its rest volume (`SdfMeshedTetMesh::with_projected_nodes`):

  | Floor | Canal nodes within 0.01 h | Worst left | The step, elastic / viscous | A press over D4 at K1, elastic / viscous | On the CPU, elastic / viscous |
  |---|---|---|---|---|---|
  | As meshed | | 1.25 h | 1 / 1 | 0.29 / 0.43 | 0.045 / 0.097 |
  | 0.5 | 95.0 % | 0.90 h | 0.71 / 0.42 | 0.41 / 1.03 | 0.063 / 0.23 |
  | 0.1 | 98.7 % | 0.78 h | 0.27 / 0.050 | 1.08 / 8.6 | 0.17 / 1.95 |

  So projecting at a floor of 0.5 misses D4 at K1's rate with the viscosity, and a floor of 0.1 misses it on the
  CPU too.

**G2's margin on the product.** *(Superseded 2026-09-27, §16u: these grids were signed by the flood fill, whose
sign is wrong within a quarter cell of the surface; signed by parity, the grid's own error meets G2 at the 5 mm
inset at 0.25, 0.125 and 0.0625 mm.)*
The obstacle grid's distance at the scan's points (the vertices its faces name and
its face centroids), which lie on the true surface; the grid is baked from the full-resolution scan and signed by
a flood fill. Where it reads positive the grid's surface lies inside the scan, and a node the contact law holds on
the grid's surface sits that deep in it; where it reads negative, a node stops short. G2's bar is 1 % of the inset.
Read off the cap discs (the faces `dome_wall_only_mesh` strips; in brackets, over every point):

| Grid | Pre-smooth | Points past the bar | Penetration over the bar: p95 / p99 / worst | Shortfall over the bar: p95 / worst |
|---|---|---|---|---|
| 1 mm | none | 7.8 % (8.2 %) | 2.0 / 5.2 / 9.5 | 1.6 / 8.7 |
| 1 mm | 1 cell | 28.0 % (28.6 %) | 2.0 / 4.4 / 13.3 | 0.71 / 7.1 |
| 0.5 mm | none | 4.9 % (5.5 %) | 0.97 / 2.6 / 4.9 | 0.92 / 4.9 |
| 0.5 mm | 1 cell | 1.7 % (2.6 %) | 0.64 / 1.3 / 6.9 | 0.34 / 3.5 |
| 0.25 mm | none | 1.8 % (1.9 %) | 0.47 / 1.3 / 2.3 | 0.42 / 2.4 |
| 0.25 mm | 1 cell | 0.34 % (0.66 %) | 0.25 / 0.54 / 3.2 | 0.14 / 1.8 |

- **No grid measured meets G2,** which is judged at the deepest node. The nearest, 0.25 mm without the pre-smooth,
  lets a node sit 2.3 bars deep. Without the pre-smooth the worst reading halved with each halving of the spacing
  (9.5, 4.9 and 2.3 bars at 1, 0.5 and 0.25 mm), as did the 99th percentile; by that trend, not measured, it
  reaches the bar near 0.1 mm, about 100× the 0.5 mm grid's samples (arithmetic). What sets the remaining error is
  not isolated.
- **The pre-smooth trades:** at 0.5 and 0.25 mm it cuts the share past the bar by about 3× and 5× and lowers the
  99th percentile, and raises the worst reading, which sits beside the cap's rim at every spacing. Leaving out the
  band within h_K2 of the cap plane as well, the smoothed 0.25 mm grid read 0.01 % past the bar and a worst of 1.24
  (an earlier run of the same grids, at `807fc703`). Step 6's bake chooses with this table.
- The old path baked its grid from the 2 500-face decimation at 0.75 of a 4 mm lattice. A 0.25 mm grid holds 8×
  the samples of a 0.5 mm one (arithmetic).

**Moved to step 7:** G2 against the grid on the product, and whether the scan's facets pump energy into the
frictionless run (§16o), both need a run on the product; so does the product's own seated window (D1). The one
attempt to step the product, with the scan at the path's start, went non-finite by step 100; why is not isolated
(a run's pre-roll is step 6's).

**What the instrument's unit tests pin:** the lowering (two materials, λ from ν), h_K2 and its corner, the element
size, the budget arithmetic, the bias statistics, the canal selection on a synthetic wall, the projection onto the
level, the grid's layout and G2's sign; each failed once under a mutation. Not pinned: the step-accuracy
reference, the CPU timing, the product's loading, the wall's secant and the per-model cost line. The ignored run
itself asserts nothing.

**How this was checked.**
- **The criteria came first,** with twelve priors kept from the reviewers.
- **Round 1:** four cold reviewers (the instrument, mutating it in a worktree of its own; this record, reproducing its
  numbers; a hunt for the product's figures, with controls on `main`; the whole plan) raised about 50 findings.
  Seven priors hit, and two in part.
- **Round 1's largest findings:** the product had been lowered at the catalog's ν 0.4, not K2's (two reviewers);
  §16e's step check on the product had been skipped; most of the instrument had no unit test, and nine mutations
  applied together passed; the canal selection and the G2 exclusion cut in the wrong places; the CPU already
  meets D4; and more places where the product's figures came back by arithmetic.
- **Every code fix has a unit test** that failed on a mutation of it.
- **Round 2:** one fresh reviewer read only round 1's fixes and found 18 problems, 16 of them created by those
  fixes: prose that claimed more than its referent (the CPU against D4, the pre-smooth, a cross-reference), a
  wrong comment on the timing, and two leaks the fixes narrowed but did not close. They were cut, not rewritten;
  every number it re-derived reproduced. With round 2 finding mostly what round 1's prose wrote, no third pass was
  run.

### 16s. K5: D1's readings, diagnosed and replaced (2026-09-26)

§16p's K5 failed on both of D1's readings. §15a sends that back to D1's readings or the lip radius, and D1 is
Jon's, so this step diagnosed why each reading moved, measured candidates and recommended; Jon accepted the
recommendations. It comes before steps 3–5 on Jon's call (2026-09-26: *"k5 is up next right?"*, on the
recommendation that the quality items come first, K5 first of them).

**Why the readings moved.** A scratch probe dumped each run's push history and seated window, and the dumps were
analysed outside the repo (a diagnostic, not kept). It reproduced §16p's K5 table to the printed digit. Beside the
planned meshes, it refined the tube 2× and 4× along its length at the 50k and 100k cross-sections (at 4×, rings of
nodes 0.86 and 0.70 mm apart), frictionless and at μ_f 0.3, λ_a 1.1 and 1.3:
- **The push with friction converges** (§16p), and kept converging under refinement.
- **The frictionless push carries the mesh's ripple.** On every mesh it ripples with a wavelength equal to the
  spacing of the rings, from 7.06 mm down to 0.70 mm: the nose crossing each ring. At λ_a 1.1 the ripple's rms is
  71, 45 and 28 % of the mean push at 10k, 50k and 100k; it fades under refinement. How much of the peak it makes
  is in the recorded table below.
- **The seated 95th percentile is decided by one or two rings.** The most-pressed 5 % of the contact is about
  4.5 mm of the tube's length (arithmetic: 5 % of the contact over the bore's circumference). It lies on the
  tube's two pressure concentrations: frictionless, the ring where the tube leaves the mandrel at its nose–shank
  seam; with friction, that ring and the entry edge. The 50k and 100k meshes put one or two rings there, so the
  percentile is whichever ring straddles the 5 % line.
- **With friction, the entry edge's pressure does not settle.** Over the entry ring's inner face it rose 25–33 %
  with each halving of the spacing, at both cross-sections and both λ_a. The product's mouth is the same kind of
  sharp edge (§15a).
- **A readout error at the edge.** A node's tributary area, a third of each incident boundary triangle (§15c), gave
  the entry ring a share of the tube's end face, which the mandrel never touches: its area came to 1.5–1.6 times its
  inner face's share at 50k and 100k, and about 3 times at 0.70 mm, so the entry's pressure read low by a third or
  more. K2's band holds no such node.

**The decision** (Jon, 2026-09-26, accepting the recommendations; fit plan D1):
- **Seated:** the contact force on the most-loaded 1 cm² patch, over 1 cm². The pressure-pain studies in a 2021
  review all used flat algometer tips of 0.5–2 cm², most of them 1 cm² by its tables
  ([Trueba-Perdomo 2021](https://www.scielo.org.mx/scielo.php?script=sci_arttext&pid=S0188-95322021000200203)), so
  the reading and the limit it will be judged against can be taken over the same area. The patch's size follows
  whichever data calibrates D1 (§8 has not found it) *(2026-09-27: one study at 1 cm² was found, fit plan U1)*. The 95th percentile and the peak are shown beside it.
- **Getting it in:** the peak push, as before. The geometric share, the μ = 0 run's push (§15h), is read as its
  largest mean over 10 mm of travel. 10 mm has no outside source: it is the shortest window the diagnostic tried
  (2, 5, 10 and 20 mm) that moved less than 5 % from 50k to 100k at both λ_a.

**The readings** (`src/readings.rs`; the tube probe prints them, and takes any cell counts as `RxCxA`):
- **A node's contact area** counts each incident boundary triangle that turns toward the obstacle: a third of it
  times the cosine between its outward normal and the obstacle's normal turned inward. A face at right angles to
  the obstacle, or turned away, adds nothing. The tube's end face tilts toward the mandrel and still adds 3.7 %
  (λ_a 1.1) and 10 % (λ_a 1.3) of an entry node's area at 50k and μ_f 0.3 (the diagnostic's dumps). A contact node
  with no facing area at all keeps its force, over a third of every incident triangle. Near side-on a node's
  pressure grows as one over the cosine, without bound, so the peak and percentile shown beside the patch are
  unbounded at a side-on contact; the patch keeps the force at any angle.
- **The patch.** Each node's force is spread over its contact area, so each triangle carries a uniform pressure.
  The surface inside a ball of the patch's radius (5.64 mm for 1 cm²) is integrated in sub-triangles, at least 16
  to a radius along each longest edge, each faded in across the rim over its own size. The patch is centred on
  every contact node and on the centroid of every triangle that carries force, so it reads the most-loaded of those
  centres; a search 8× denser found up to 0.6 % more on the 50k runs (a review's measurement, not kept). On a bore
  of 10 mm radius the ball holds 1.03 % more than 1 cm² of surface, which the unit test computes and the reading
  reproduces.
- **The push over travel** is the work over a 10 mm window, over 10 mm, taken exactly at the samples' boundaries;
  a hold adds nothing. 10 mm is 2.9 and 3.6 ring spacings on the 50k and 100k tubes, over which a sinusoidal ripple
  keeps 3 % and 9 % of its amplitude (arithmetic; a unit test checks the formula). At 50k (λ_a 1.1, frictionless)
  the reading moved +2.2 and −2.3 % between windows of 9, 10 and 11 mm (a review's measurement), as much as its
  50k → 100k change: the ripple the window keeps is part of what K5 reads there.
- **Pinned by 21 unit tests** (`tests/readings.rs`), each on a case with a known answer, among them a stretched
  window, a window turned from rest, an obstacle posed and tilted at the window's time, and a side-on plane reached
  through posed turns. Of 39 single mutations, all but one made a test fail; the survivor cuts the sub-triangles
  by `floor` rather than `ceil`, which only coarsens the integration (a mutation run, not kept).

**K5 passes on the new readings, from 50k to 100k.** The bar is §15a's, 5 % from 50k to 100k. The readings were
chosen after the diagnosis had seen every mesh below, so none is held out.
`cargo run --release -p sim-soft-explicit --example tube -- <mesh> <0|2> <0|0.3> f32 20 0.2 10 1`,
`RAYON_NUM_THREADS=4`, at `9a802965`; `1f84dbb8`, `393b35ba` and `9e25fb90` (before rounds 1, 2 and 3's fixes)
printed the same in every field but the timings. The 100k cross-section refined along the tube is `6x64x86` and
`6x64x172`:

| D1's readings | Case | 10k | 50k | 100k | 50k → 100k | 2× / 4× along | 100k against 4× |
|---|---|---|---|---|---|---|---|
| 1 cm² patch / μ | λ_a 1.1, frictionless | 0.1450 | 0.1665 | 0.1681 | **+0.97 %** | 0.1704 / 0.1718 | −2.2 % |
| | λ_a 1.3, frictionless | 0.3679 | 0.4019 | 0.4054 | **+0.87 %** | 0.4160 / 0.4206 | −3.6 % |
| | λ_a 1.1, μ_f 0.3 | 0.1444 | 0.1374 | 0.1367 | **−0.55 %** | 0.1358 / 0.1352 | +1.1 % |
| | λ_a 1.3, μ_f 0.3 | 0.3559 | 0.3461 | 0.3444 | **−0.48 %** | 0.3420 / 0.3410 | +1.0 % |
| Push peak (N) | λ_a 1.1, μ_f 0.3 | 4.532 | 4.526 | 4.467 | **−1.31 %** | 4.465 / 4.472 | −0.11 % |
| | λ_a 1.3, μ_f 0.3 | 13.23 | 12.82 | 12.70 | **−0.98 %** | 12.68 / 12.68 | +0.13 % |
| Geometric share, over 10 mm (N) | λ_a 1.1, frictionless | 0.1841 | 0.1136 | 0.1122 | **−1.24 %** | 0.1102 / 0.1107 | +1.4 % |
| | λ_a 1.3, frictionless | 1.067 | 0.9191 | 0.9118 | **−0.80 %** | 0.9081 / 0.9089 | +0.31 % |

- **The frictionless patch is still moving along the tube:** at λ_a 1.3 it rose 2.6 % from 100k to 2× and 1.1 % from
  2× to 4×, and 100k reads 3.6 % below the 4× mesh, more than its 50k → 100k change.
- **From 10k to 50k** (§16i's reading of K5) the frictionless patch moves +14.8 and +9.2 % and the geometric share
  −38 and −14 %, past the bar; the patch with friction moves −4.8 and −2.8 % and the push peak −0.1 and −3.1 %
  (arithmetic on the table).
- **So the element size D1's readings need on the product is open.** `base_mold` is meshed at h_K2 = 2.20 mm, about
  the 10k tube's element size (2.26 mm, §16r), and D4's figures there rest on K2 alone. Its mesh is not built in
  rings; whether it resolves the readings as the 10k tube does, or as the 50k does, is not measured. Step 7 measures
  it on `base_mold` before a verdict is trusted (§15g's list for steps 6–9).
- The patch sits 12.7–13.5 mm behind the tip frictionless from 50k on, over the seam (11 and 13 mm behind it). With
  friction it is centred about 5 mm inside the entry ring (whose position the diagnostic read), within its radius
  of the edge.
- **f32 against f64** (K3's 0.5 %, pre-registered): at 50k and μ_f 0.3 the patch reads the same to five decimals at
  both λ_a (0.13742 and 0.34610; `… -- 50k <0|2> 0.3 f64 20 0.2 10 1`, `RAYON_NUM_THREADS=4`; 6 threads printed the
  same).
- Every run is valid: λ_z within 0.18 % on the frictionless runs, KE/IE and the balance at most 0.05 %, no inverted
  element; the Coulomb push reads 0.985 and 0.961 at 100k, as in §16p.

**What is shown beside them:**

| | Case | 50k → 100k | 100k's cross-section, 1× / 2× / 4× along |
|---|---|---|---|
| 95th percentile / μ | λ_a 1.1, frictionless | +8.5 % | 0.1544 / 0.1500 / 0.1487 |
| | λ_a 1.3, frictionless | +12.4 % | 0.4786 / 0.4499 / 0.4565 |
| | λ_a 1.1, μ_f 0.3 | −5.1 % | 0.1365 / 0.1580 / 0.1464 |
| | λ_a 1.3, μ_f 0.3 | −11.8 % | 0.3515 / 0.3897 / 0.3928 |
| Peak / μ | λ_a 1.1, μ_f 0.3 | +5.1 % | 0.2367 / 0.2844 / 0.3439 |
| | λ_a 1.3, μ_f 0.3 | +5.9 % | 0.6005 / 0.7004 / 0.7926 |
| Push, 100-step peak, μ = 0 (N) | λ_a 1.1 | −14.1 % | 0.1464 / 0.1244 / 0.1200 |
| | λ_a 1.3 | −3.2 % | 0.9657 / 0.9299 / 0.9311 |

- The percentile moves 5–12 % from 50k to 100k; from 2× to 4× it moves 0.8–7.4 %.
- **The peak with friction rises 13–21 % with each doubling along the tube**, where the entry edge's pressure does
  not settle (above).
- **The frictionless 100-step push peak** reads 22 % (λ_a 1.1) and 3.7 % (λ_a 1.3) above the 4× mesh's at 100k. It
  moves 3.5 % and 0.1 % from 2× to 4×, and at 4× reads 8 % and 2 % above the 10 mm reading, which spreads the peak
  over its window.

**Not covered:** refinement through the wall or around it alone; loading times other than 10 T_s (the budget's
runs are at 0.625 T_s, §16r); viscosities other than Ecoflex 00-30's; the product, where its most-loaded square
centimetre is, the element size its readings need (above) and how its mesh reads the percentile beside them
(step 7); a lip radius; patch sizes other than 1 cm² (0.5 and 2 cm² moved at most 2 % from 50k to 100k in the
diagnostic). Fit plan U10, Jon's account that the sharp mouth is fine in Ecoflex 00-30 and a comfort issue in
Dragon Skin 10A, is the one report of seated comfort from use; whether the patch reading agrees with it waits on
D1's limits and a run on the product.

**How this was checked.**
- **The criteria came first,** with 17 priors kept from the reviewers.
- **Round 1:** three cold reviewers (the code, mutating it in a worktree of its own; this record, reproducing its
  numbers from the committed probe; the whole plan) raised 30 findings, four of them twice. The largest:
  - K5 had been read only from 50k to 100k; from 10k to 50k two of the new readings do not converge, and
    `base_mold`'s mesh is about the 10k tube's element size (above);
  - the tests never read a deformed window or a posed obstacle, so four mutations of which state the geometry is
    read in passed all of them, moving the readings by 1.4–46 %;
  - a face turned away from the obstacle counted, and a node's force could be lost;
  - the end face's share was stated inverted, the cited review said less than claimed, and Jon's build-order call
    was cited where it was not written.

  Five priors hit, and two in part.
- **Round 2:** one fresh reviewer read only round 1's fix diff and found 10 problems, 9 of them in code or text the
  fixes wrote: the side-on threshold sat at rounding, so a side-on plane reached through a posed turn missed it; a
  face's normal read at rest passed every test; "from 10k to 50k the readings do not converge" held for two of
  the four; and smaller slips in the prose. The code got tests, and the prose was corrected.
- **Round 3:** one fresh reviewer read only round 2's fix diff and found 5 problems, all in what round 2 wrote. The
  largest: the new threshold's "far above rounding" held for f64 windows, not the f32 ones the probe runs, where
  a constructed side-on window missed the fallback. The threshold was cut rather than moved again: the fallback
  now takes only a node with no facing area, and the pointwise pressures' growth near side-on is stated, not
  bounded. Every round's findings sat mostly in the previous round's fixes, and the threshold drew one in each;
  the review stopped there.

### 16t. U3: why the rigid path asks for room, and the path step 7 runs (2026-09-27)

Fit plan U3 asked why the old path asks the cavity for 8.3 mm of room with no inset, near the entrance (the Tet10
recon's `THE SLIDING MODEL ON THE PRODUCT SCAN`). Step 6 settles it before step 7, because step 7's verdicts would
otherwise measure the path and not the fit (§15g). No solver runs here:
`tools/cf-sim-research/src/insertion_sim/path_room.rs`.

**The path as written** (`slide_pose_at`) walks the scan's tip back along the centreline, and turns the whole scan
about the tip by the rotation between the centreline's tangent at the seated tip and its tangent where the tip now
is.
- On a circular arc in a plane, that carries the rest of the scan along the curve
  (`on_a_planar_arc_the_pose_as_written_is_the_slide`). It reads the seated tip's tangent off the first segment
  alone, half a segment's turn from the curve's, so the two differ by that angle times the reach.
- Where the curvature changes, it does not: on an S of two opposite arcs, the scan's far end lands where the turn
  at the tip puts it, off the curve, as the closed form says
  (`where_the_curvature_changes_the_pose_as_written_turns_the_far_end_off_the_curve`).

**Two other motions, on the same measure.** The measure is the Tet10 recon's "as written" room: the moved contact's
signed distance at the undeformed cavity wall's nodes, over 64 poses, with a contact that is the cavity itself when
seated. Room between the wall's corner nodes, or between poses, is not seen.
- **The slide** carries every point of the scan along the centreline by the tip's walk, at the same place in the
  centreline's parallel-transported frame. It bends the scan to follow the curve, so it is not rigid. The room it
  asks is the room the scan's own cross-sections ask of the ones they pass.
- **The fitted pose** is the rigid motion closest to the slide: least squares over the scan's surface that the slide
  puts inside the device, each vertex weighted by its area (Kabsch).

On a straight centreline the three are one translation, and a taper asks its own room under all three
(`on_a_straight_centreline_the_three_motions_agree_and_a_taper_asks_its_own_room`).

**On `base_mold`** (`why_the_rigid_path_asks_room_on_the_product_scan`; the rooms as rough ratios, since the slide's
room is a size of the scan's shape; the figures stay local):

| Motion | Most room, against the path as written's | Where |
|---|---|---|
| Path as written | 1 | Late in the travel, near the entrance: the Tet10 recon's 8.285 mm, reproduced to its printed digit |
| Fitted pose | About a quarter | About halfway through the travel, deeper in |
| Slide (not rigid) | About three quarters of the fitted pose's | The same pose and node as the fitted pose |

- The path as written asks room from about an eighth of the travel on, and the other two from about a quarter.
- Between poses, a vertex of the scan inside the device moves at most 1.08 of the tip's arc step under the fitted
  pose, and 1.55 under the path as written.
- The most room understates what the fitted pose keeps of the path. From 0.6 to 0.9 of the travel it asks 1.5–2.2
  times the slide's room at each pose. It asks more than the old bridge's d̂ (1.2 mm) of 0.12 of the wall's nodes,
  against 0.02 under the slide and 0.30 under the path as written. *(2026-09-27, §16v: those fractions are of the old
  wall's boundary nodes, read before the mesher's Parity Rule fix, which could expose nodes inside the wall on its
  boundary; not re-read.)*
- The fitted pose is the closest rigid motion to the slide, not a close one: over the scan's surface inside the
  device it misses the slide by up to 1.8 times the slide's most room.
- The headline rests on fitting over the part of the scan the slide puts inside the device. Fitted over the whole
  scan, the fitted pose asks about two fifths of the path's most room (a review's mutation of the probe, not
  kept).
- The slide is built from projections onto the centreline, so a node slid there and back misses itself by up to
  0.04 of the slide's most room. Its room is the seated contact's distance at the point the slide carries to each
  node, not a distance to the bent scan; the difference was not measured.

**Verdict:** most of the room the path as written asks is the path's own. A rigid pose exists that asks about a
quarter of it, so step 7 on the path as written would have measured the path.

**Decided (Jon, 2026-09-27, accepting the recommendation):** step 6 builds the fitted pose as the scan's path. It
stays a prescribed pose, applied in the contact law, so no rigid–soft coupling is needed (§14a). Step 7 prints the
sideways force and the twist the wall puts on the scan, which show how far the walls would push it off this path;
what reading of them would bring the contact-guided scan forward is not yet set (the list for steps 6–9). The
contact-guided scan moves to the fit plan's Later. The fitted pose's code here is a measuring copy; step 6's
lowering builds it in `sim-soft`, and samples it in time. Jon decided on the most-room ratios; the per-pose and
extent figures above came from review afterwards, and were reported to him before this PR was pushed.

**Not settled here:**
- The fitted pose is a least-squares fit to the slide, not the rigid pose that asks the least room. The least room
  any rigid path asks is not measured.
- What in the path as written makes its excess was not isolated: the curvature changing, the centreline leaving its
  plane (the pose as written is tested only on planar curves), or the tip's tangent read off one segment.
- The slide asks room of its own. That is the scan's shape, carried by a rigid scan (fit plan U9); step 7 measures
  what the wall does with it.

**Also found.** nalgebra's `Rotation3::rotation_between` takes its angle as the `acos` of the unit vectors' dot
product, which rounding can push one ulp past 1 while their cross product is above its cut-off. Two directions about
1e-10 rad apart then give a NaN rotation. `slide_pose_at` called it, and the probe's frame did. Both now call
`turn_between` (`atan2`), and `turn_between_is_finite_where_rotation_between_is_not` fails on the old call.

**How this was checked.**
- **The criteria came first,** with eleven priors kept from the reviewers.
- **Round 1:** three cold reviewers (the code, mutating it in a worktree of its own; this record, rerunning the
  probe, which printed the same in every field; the whole plan) raised 27 findings, four of them twice. None
  changed the verdict. The largest:
  - the tests pinned neither the frame's transport, the fit's reflection fix, its weights and areas, nor which
    points it fits: eleven mutations of those passed every test. Each now fails one, the transport on a helix;
  - step 7's print of the sideways force and twist had no reading that would act on it, and D1's push on a turning
    path needs the twist, which the executor does not reduce (the list for steps 6–9);
  - the most room understated what the fitted pose keeps of the path (above), and this record's pointer to §16s's
    patch area was wrong by a factor.

  Seven priors hit, one in part.
- The probe printed the same after the fixes.

### 16u. Step 6: the obstacle bake (2026-09-27)

Step 6 bakes the obstacle in `sim-soft` and sets its grid against G2 (fit plan U18). Measuring that grid
below §16r's finest overturned §16r's reading of G2: its grids' sign was wrong at the surface.

**How it is measured.** A probe (`tools/cf-sim-research/src/insertion_sim/obstacle_grid.rs`) reads the scan's
exact distance only at the samples of the tricubic stencils around G2's points (the vertices the scan's faces name,
and its face centroids), through the solver's own `sdf_tricubic`, so it reads grids too fine to build whole. At a
spacing a dense grid can be built, it reads what that grid reads (`the_band_reads_what_the_dense_grid_reads`).
Signed by the flood fill as §16r's grids were, at 0.25 mm it reads §16r's row off the cap discs to its printed
digits: 1.8 % past the bar, penetration p95 / p99 / worst 0.47 / 1.28 / 2.34 bars, shortfall 0.42 / 2.41.

**The sign.** Three, compared at the 0.25 mm band's samples
(`g2_against_the_grids_spacing_on_the_product_scan`, `where_the_pseudo_normal_sign_fails_on_the_product_scan`):
- **The flood fill's** (§16r's grids, and the old path's) disagrees with both others at 2.5 % of the samples, every
  one within a quarter cell of the surface. `mesh-sdf` documents it as unreliable within a cell of the surface
  (`FloodFillSign`).
- **The pseudo-normals'** (the welded scan's) disagrees with both others at about 3 samples in 100 000 a quarter
  cell or more from the surface (up to about three cells), all in one region. What makes it wrong there is not
  isolated.
- **Parity**, the majority of three rays' crossings (`mesh_sdf::ParitySign`), agrees with the flood fill at every
  sample a quarter cell or more from the surface, and with the pseudo-normals at all but a handful of samples
  within it; which is right at those few is not measured. Its tests: a sphere against the analytic answer, a cube's
  edges and corners either way it is wound, and one miscounting ray outvoted. It needs a closed surface, and it
  counts even-odd: a region enclosed twice reads outside (`a_region_enclosed_twice_reads_outside`). On `base_mold`
  the bake's coarse grid agrees in sign with a flood fill of its spacing at every sample a flood cell or more from
  the surface (`the_bake_on_the_product_scan`); an overlap thinner than that is not compared.

**G2 signed by parity, with no pre-smooth** (off the cap discs; readings over the bar):

| Grid | Points past the bar | Penetration: p95 / p99 / worst | Shortfall: p95 / worst |
|---|---|---|---|
| 0.25 mm | none | 0.047 / 0.18 / 0.86 | 0.019 / 0.71 |
| 0.125 mm | none | 0.024 / 0.065 / 0.53 | 0.008 / 0.24 |
| 0.0625 mm | none | 0.012 / 0.045 / 0.30 | 0.004 / 0.17 |

- At `base_mold`'s 5 mm inset the grid's own error meets G2 at every spacing measured, 0.25 mm included. Over every
  point, the discs included, none is past the bar either, and the worst readings are the same.
- At every spacing the worst point lies beside a cap's rim.

**The bake** (`sim_soft::obstacle`; the fine grid is `sim_soft_explicit::executor::FineGrid`):
- **The distance** is exact, to the mesh's triangles; **the sign** is parity. The bake welds exactly coincident
  vertices and refuses a mesh that is not closed once welded (`a_mesh_that_is_not_closed_is_refused`); its winding
  does not matter (`a_box_wound_inward_bakes_to_the_same_grids`). Nothing is smoothed.
- **Two grids.** The obstacle's own grid is the coarse one. The fine one is stored in bricks of 8 samples a side,
  named by a brick map. A lookup reads the fine grid when its point is on the fine lattice and every brick its
  stencil touches has a slot, and the coarse grid otherwise. The rule is shared math (`sdf_stencil_bricks`,
  `sdf_fine_present`, `sdf_fine_index`), used by the obstacle's lookup and the CPU executor's alike
  (`where_a_brick_is_missing_the_grid_answers`, `the_executors_fine_lookup_is_the_obstacles_on_and_off_its_bricks`).
- **Which bricks.** A lookup reads only samples within 2√3 cells of its point, so a point within the bake's band of
  the surface reads only samples within the band and 2√3 cells of it. A brick is kept exactly when one of its samples
  is that near, and every point within the band reads the fine grid; both are tested on a box turned off the grid's
  axes (`a_brick_is_kept_exactly_when_a_sample_is_near_the_surface`,
  `every_point_within_the_band_reads_the_fine_grid`).
- **On `base_mold`** (`the_bake_on_the_product_scan`): a coarse grid at 0.5 mm, a fine one at 0.25 mm and at
  0.0625 mm, a band of one fine cell. Every point on the scan reads the fine grid, and reads exactly what the band
  signed by parity reads at the same spacing (asserted). At 0.0625 mm the bake takes about a tenth of D4; at 0.25 mm, under a fiftieth *(2026-09-27, §16w: at the
  product's band of eight fine cells, about three tenths)*. At
  0.0625 mm the fine grid's values at f32 are more than twice wgpu's default storage-binding limit (128 MiB); at
  0.25 mm they are well under it. Its sizes and times stay on the machine that ran it.

**What this corrects.** §16r's claim that no grid meets G2 supported:
- §16r's G2 bullets (no grid meets it; the trend to 0.1 mm; the pre-smooth's trade) and fit plan U18: read with the
  flood fill's sign, and superseded;
- the list for steps 6–9's G2 item, §14a's `sim-soft` row and §14b's pre-smooth note;
- step 6's done-when, that the bake match the old one within 1 % of a grid cell: the bar is instead that the bake
  reads each sample's exact signed distance (its tests) and, on `base_mold`, its own error at the scan's points is
  within G2's bar (above).

**Not re-read here:**
- **U17's canal offsets** were measured against a flood-fill-signed grid (`explicit_budget.rs`, `Truth`), on a
  wall meshed from the old path's grid, flood-filled and pre-smoothed; neither is re-read with parity's sign. D3's
  search goes down to 0 mm, where the canal is the scan's own surface. So the wall's PR (fit plan U17) measures
  its option at a small inset too, against a reference signed by parity. *(2026-09-27: done, §16v.)*
- **U3's room** (§16t) was read through the old path's scan grid (decimated and flood-filled). Step 6's check that
  the lowering reproduces the probe's room reads both through the same grid.
- The tube's G2 (§16o) is not affected: its mandrel is baked from its exact distance. The old path's contact read
  flood-filled grids too; it retires after step 7.

**G2 against the inset (Jon, 2026-09-27).** G2's bar was 1 % of the inset: 0.05 mm at 5 mm, 0.01 mm at 1 mm, and
none at 0 mm, where no grid meets it; D3's search goes down to 0 mm. Jon set it to 1 % of the inset, but never below
0.02 mm. The worst readings above are, in millimetres, 0.043, 0.026 and 0.015 at 0.25, 0.125 and 0.0625 mm
(arithmetic on the table), and the obstacle is the scan whatever the inset: so a 0.0625 mm grid meets the bar at
every inset, a 0.125 mm grid from 3 mm, and a 0.25 mm grid from 5 mm. Jon chose 0.0625 mm for the product's fine
grid (2026-09-27), and `the_bake_on_the_product_scan` asserts it meets the floor.

**Carried to later steps:**
- **Step 6's lowering** bakes the fine grid at 0.0625 mm and sets the band from how far a node's predicted position
  can reach past the surface in a step (the contact law reads the grid there, §16o). Where a node sits deeper than
  the band, as at the t = 0 intrusion, the coarse grid answers, and its error there is not measured. *(2026-09-27,
  §16w: the band is eight fine cells, from the product run's deepest predicted point; two monitors read it and the
  coarse grid's corrections in every run; and the run starts clear of the wall.)*
- **Step 7** judges G2 in a run: the table reads the grid at the scan's own points, not where the contact law holds
  the wall's nodes. It reads each surface node against the scan's exact signed distance and G2's floor: the
  executor's penetration monitor reads the grid, so it cannot see the grid's own error. At which steps it reads is
  step 7's design. Each run prints which grid answered each surface node's lookups and the deepest predicted
  point; and on the frictionless run, the work the contact does over the hold, for the facets' pumping (§16o).
- **Step 4** (the GPU): the rule is already in the generated WGSL. Where the fine grid's bricks end, the distance
  steps between the two grids, which the conformance tests must allow. The brick map and the bricks are two more
  buffers to bind. The fine grid at 0.0625 mm needs a storage binding above wgpu's default (above): `sim-gpu`'s
  context requests the adapter's limit, and the M4 Pro grants 4 GiB; lavapipe's grant is not checked. An f64
  executor holds its own copy of the fine grid beside the obstacle's.

**How this was checked.**
- **The probes** run locally, on the product scan:
  `cargo test -p cf-sim-research --release --bin cf-sim-research -- insertion_sim::obstacle_grid --ignored --nocapture`.
- **Mutations:** the new checks the tests reach each failed once under a mutant of the code they guard, and so did
  the bake probe's G2 and sign asserts. No test reaches the bake's refusals of counts past a u32 (vertices, fine
  values, brick slots). Two mutants survive: the fine lattice's origin moved by a cell, which is another valid lattice, and `<=` for `<` at the keep threshold, which
  differs only on a tie. One mutant died of an overflow instead of its check; that found four grid-size products
  that could overflow, now fixed.
- **Round 1:** three cold reviewers of the first version, signed by the pseudo-normals. They found that sign wrong
  away from the surface, and this record claiming more than its probes showed.
- **Round 2:** a code and a record reviewer on round 1's fixes: 18 findings, 14 of them created by those fixes,
  among them parity's even-odd limit and its unbounded bins.
- **Round 3:** one reviewer on round 2's fixes: 6 findings, 5 of them created by those fixes, all prose or test gaps;
  the bake's grids were bit-identical across round 2's restructure. The rounds stopped there.
- **After the push:** a completeness review against the plan and the session's pending list found several items
  carried only in conversation; a macro review found the floor's consequences unrecorded. The
  owners above, and this paragraph, came from them.

### 16v. Step 6: the wall's canal surface (2026-09-27)

Fit plan U17: the old path's wall puts its canal nodes off the true canal surface (§16r: the 5th and 95th percentiles
at −0.50 and +0.32 of an element, the worst at 1.25), and step 6 chooses among a quality floor, a finer grid under the
mesher, and a mesher that places the surface nodes itself. Measured here by changing one thing at a time, on
`base_mold` at h_K2, at the same lattice throughout, at the 5 mm inset and at 1 mm and 0 mm (D3's search reaches
0 mm, where the canal is the scan's surface): `tools/cf-sim-research/src/insertion_sim/canal_surface.rs`,
`where_the_canal_nodes_offsets_come_from_on_the_product_scan`.

**How it is measured.**
- **The wall** is `build_insertion_geometry`'s, built from its parts (`wall_body`, `wall_material_field`,
  `mesh_wall`), so that one input can change at a time. At the 5 mm inset the probe asserts that it rebuilds the
  old path's wall: the same vertex positions and tetrahedra.
- **The chain**, at each inset, each wall changing one input from the one before: the old path (the scan decimated,
  a flood-filled grid at 0.75 of the lattice, pre-smoothed by a cell); no pre-smooth; the scan as loaded, not
  decimated; the grid signed by parity, not the flood fill (the same samples with parity's sign); the scan's exact
  distance, not the grid; and the cut points located on that distance, not interpolated.
- **The truth** is the cap-stripped scan's exact distance, signed by parity (§16u); the budget probe's `Truth` is now
  signed so too, where it was signed by a 1 mm flood fill.
- **Each canal node's offset** splits into how far the node sits off the mesher's own zero set (the wall body's
  value at the node) and the rest, the field's error at the node. The split holds on the canal side of the scan's
  surface, where the cavity's distance is `pinned_floor_shell`'s rind; at 1 mm some canal nodes lie outside the scan,
  where it does not (4 % on the old path, none on the last two walls). At 0 mm there is no rind.
- **The columns:** offsets in element sizes, negative into the canal; "in / out" are the canal nodes more than
  0.01 h inside and outside the true surface; the step is the rest step at ν 0.49 over the old path's at the same
  inset, elastic / viscous; the press is a viscous one at ν 0.49 over D4 at K1's rate (§16r's arithmetic).

**At the 5 mm inset:**

| The wall | In / out | Offset p5 / p95 / worst | Off its own zero set p5 / p95 | Field's error p5 / p95 | Step | Press / D4 |
|---|---|---|---|---|---|---|
| Old path | 49.4 % / 46.9 % | −0.504 / +0.322 / 1.252 | −0.009 / +0.160 | −0.532 / +0.271 | 1 / 1 | 0.424 |
| No pre-smooth | 73.0 % / 16.5 % | −0.213 / +0.048 / 0.537 | −0.058 / +0.048 | −0.181 / +0.041 | 0.97 / 0.98 | 0.444 |
| The scan as loaded | 69.4 % / 18.4 % | −0.198 / +0.052 / 0.533 | −0.060 / +0.048 | −0.168 / +0.037 | 1.05 / 1.06 | 0.408 |
| The grid signed by parity | 76.7 % / 12.6 % | −0.207 / +0.036 / 0.533 | −0.063 / +0.022 | −0.170 / +0.037 | 1.05 / 1.06 | 0.408 |
| The exact distance | 37.5 % / 7.8 % | −0.069 / +0.016 / 0.266 | −0.069 / +0.016 | 0 / 0 | 0.92 / 0.96 | 0.451 |
| Cut points located on it | 0 / 0 | 0 / 0 / 0.000 | 0 / 0 | 0 / 0 | 0.95 / 1.03 | 0.420 |

- **The pre-smooth carries most of the spread:** without it the 5th percentile goes from −0.50 to −0.21.
- **The sign does not matter here:** signed by parity, the grid reads about the same, and parity, a 1 mm flood fill
  and the welded scan's pseudo-normals agree at every canal node of every wall.
- **The grid's own sampling carries most of the rest:** the exact distance takes the 5th percentile from −0.21 to
  −0.07. The decimation hardly matters, as §16r found.
- **What remains is the mesher's:** a cut point placed where the line through its lattice edge's two samples crosses
  zero sits off a curved zero set. Located on the distance itself (`sim_soft::CutPoints::Root`), every canal node
  lies on the true surface to the printed digits. Labelle and Shewchuk's own implementation locates cut points by
  bisection; they note that linear interpolation keeps the angle guarantee and loses the others (their §3.1).
- With the exact distance the truth is the mesher's own field, so the field's error is zero by construction; what
  checks that field is §16u's parity against the flood fill, and here parity against the welded pseudo-normals.

**At 1 mm:**

| The wall | In / out | Offset p5 / p95 / worst | Mean | Off its own zero set p5 / p95 | Step | Press / D4 |
|---|---|---|---|---|---|---|
| Old path | 15.5 % / 82.1 % | −0.164 / +0.441 / 1.163 | +0.156 | −0.051 / +0.012 | 1 / 1 | 0.460 |
| No pre-smooth | 3.4 % / 95.0 % | +0.010 / +0.427 / 0.888 | +0.214 | −0.262 / +0.154 | 0.86 / 0.88 | 0.525 |
| The scan as loaded | 2.6 % / 95.9 % | +0.017 / +0.434 / 0.897 | +0.220 | −0.263 / +0.142 | 0.85 / 0.88 | 0.521 |
| The grid signed by parity | 20.2 % / 73.2 % | −0.077 / +0.319 / 0.466 | +0.108 | −0.013 / +0.341 | 0.85 / 0.88 | 0.522 |
| The exact distance | 7.1 % / 79.0 % | −0.015 / +0.346 / 0.416 | +0.146 | −0.015 / +0.346 | 0.86 / 0.88 | 0.523 |
| Cut points located on it | 0 / 0 | 0 / 0 / 0.000 | 0 | 0 / 0 | 0.86 / 0.88 | 0.530 |

- **Here the sign matters:** signed by parity, the grid's mean offset halves (+0.220 to +0.108) and so does its worst
  (0.90 to 0.47).
- Without the pre-smooth the canal moves further into the wall (the mean from +0.156 to +0.214) while its 5th
  percentile comes up; the two errors are not separated.
- **Linear cuts on the exact distance sit into the wall** (the mean +0.146). The cavity's distance there is the rind,
  the scan's unsigned distance less the inset, kinked at the scan's surface one inset from the canal, and linear cuts
  sit off a kinked distance's zero set (`root_cuts_find_a_kinked_distances_zero_set`); whether that kink is what
  sets the offset on the product is not isolated. Located cuts remove it.

**At 0 mm:**

| The wall | In / out | Offset p5 / p95 / worst | Mean | Off its own zero set p5 / p95 | Step | Press / D4 |
|---|---|---|---|---|---|---|
| Old path | 95.2 % / 3.6 % | −0.614 / −0.016 / 1.723 | −0.296 | −0.051 / +0.011 | 1 / 1 | 0.539 |
| No pre-smooth | 78.8 % / 12.8 % | −0.176 / +0.060 / 0.697 | −0.059 | −0.070 / +0.038 | 0.98 / 0.98 | 0.549 |
| The scan as loaded | 76.1 % / 14.5 % | −0.160 / +0.081 / 0.683 | −0.048 | −0.089 / +0.039 | 0.95 / 0.97 | 0.554 |
| The grid signed by parity | 81.2 % / 8.2 % | −0.137 / +0.023 / 0.443 | −0.048 | −0.051 / +0.017 | 0.95 / 0.97 | 0.553 |
| The exact distance | 34.3 % / 6.7 % | −0.046 / +0.013 / 0.166 | −0.009 | −0.046 / +0.013 | 0.88 / 0.91 | 0.591 |
| Cut points located on it | 0 / 0 | 0 / 0 / 0.000 | 0 | 0 / 0 | 0.91 / 0.96 | 0.558 |

- **The pre-smooth carries most of it here too:** the mean goes from −0.30 to −0.06. Parity's sign takes the worst
  from 0.68 to 0.44, and the exact distance the 5th percentile from −0.14 to −0.05.
- **The signs at the canal nodes:** parity and the welded pseudo-normals disagree at no more than one canal node on
  any wall at 1 mm and 0 mm, except the located-cut wall at 0 mm, whose nodes lie on the surface, where the sign is
  arbitrary and the offset is zero either way (about half disagree). A 1 mm flood fill disagrees with parity at up
  to about half the canal nodes at 0 mm: they lie within a cell of the surface, where §16u found it wrong.

**The cost of step 7's wall** (the last row at each inset; beside it, the old path's):

| Inset | h / h_K2 | Step accuracy (§16e, ν 0.49), elastic / viscous | Press / D4 at ν 0.49 | At the budget's worst corner (ν 0.495, the viscosity × 1.47) | The old path's, worst corner |
|---|---|---|---|---|---|
| 5 mm | 0.992 | +0.0000 / +0.0002 | 0.420 | 0.594 | 0.598 |
| 1 mm | 0.997 | +0.0000 / +0.0001 | 0.530 | 0.748 | 0.647 |
| 0 mm | 0.995 | +0.0000 / +0.0002 | 0.558 | 0.787 | 0.759 |

- At ν 0.49 the step accuracy is within §16e's bar of ±0.02 at every inset; at the worst corner it is not read.
- Each wall is built, its fields and its mesh, once per inset, in about a hundredth of D4.
- At the worst corner, 3–4 full verdicts would take 9–16 minutes (arithmetic), the upper end past D4's 15; with Tier
  1 picking 1–2 insets for full verdicts (step 7), 3–8 minutes.

**A mesher defect it found.** Before the fix below, with the exact distance and located cuts, a few canal nodes still
sat off the surface (0.5 % at 5 mm, the worst 0.72; 0.2 % at 1 mm), and so did boundary nodes on 9 of 600 random
ellipsoids and shells (a search not kept). Each was a lattice vertex inside the wall, exposed where two BCC tetrahedra
split the quadrilateral they share along different diagonals, so neither half was matched. The Parity Rule (Labelle
and Shewchuk §3.3) chooses a diagonal from the face's long edge and its third lattice vertex. The predicate read only
the long edge, so it could not tell the long edge's two faces apart: the `(2 in, 2 out)` stencil gave them opposite
diagonals by the order of its outside slots, which the lattice's tetrahedron table does not fix, and the other
stencils gave both the same one.
- **Fixed:** `diagonal_from_a` is the paper's predicate computed from integer lattice indices, and every stencil that
  splits such a face uses it.
- **It changes the mesher's default output:** the vertex positions are the same and the connectivity changes
  wherever the two predicates differ. On the old wall at the 5 mm inset a press re-reads 0.28 / 0.42 of D4 at K1
  (elastic / viscous, ν 0.49), where §16r's table read 0.29 / 0.43 before the fix. The solver's crate changed between
  the two readings too (#973, #975), and which change moved it is not isolated. `contact_drop_rest` now runs 2 s,
  not 1 s: its sphere rocks on its lowest vertex after landing, and settles later on the fixed mesh (measured on
  both; the test's docstring).

**The choice** (an engineering call, 2026-09-27; Jon delegates these, §9): step 7's wall is meshed from the scan's
exact distance, signed by parity, with its cut points located on that distance. No node is projected, so no quality
floor is set; §16r's projection cost does not apply. A finer grid under the mesher is not measured: the exact
distance needs no grid.

**Not measured here:**
- The canal between its nodes: a face between canal nodes on a curved surface lies off it. The contact law holds
  nodes; the faces between them are not read.
- The CPU's time: the probe times the build, not a step.
- Insets between 1 and 5 mm; the step ratio is not monotone in the inset.
- The element size the readings need (§16s): the wall stays at h_K2.
- The caps' floors, the mouth within an element of a cap plane, and the outer skin.
- Whether the cast's canal matches step 7's at small insets: `cf-cast-cli` signs the scan by a flood fill.

**Carried:**
- **Where step 7's wall is built:** `wall_body` needs `cf-design`'s `pinned_floor_shell` and the cap planes, which
  `sim-soft` does not depend on. Step 6's lowering decides where the wall is meshed, and meshes it as the probe does;
  with more than one layer, the material field and the per-tetrahedron layer (keyed on the old grid in
  `build_insertion_geometry`) read the exact distance too.
- **U3's room** (§16t) was read on the old wall's nodes, whose canal offsets are of the same order as the room.
  Step 6's check that the lowering reproduces the probe's room reads both on one wall, and step 7 reads the room on
  its own wall.
- **The mesher before the fix** may have given the old wall boundary faces inside it. Whether that played a part in
  §16r's non-finite attempt to step the product, or in the old path's surface touching itself (fit plan, Phase 1),
  is not measured; the pre-roll runs on the fixed mesher.
- **Figures read before the fix** carry notes where they stand: §16r's budget table and fit plan U15's note (the old
  wall's ν 0.49 press re-read above), and §16t's node fractions (not re-read).

**How this was checked.**
- **The probe** runs locally, on the product scan:
  `cargo test -p cf-sim-research --release --bin cf-sim-research -- insertion_sim::canal_surface --ignored --nocapture`.
- **Located cuts** (`sim/L0/soft/tests/sdf_cut_points.rs`): every boundary vertex on a sphere's and on a kinked
  shell's zero set to 1e-12 of the radius, where linear cuts are measurably off; Theorem 1's angle bounds; and a
  non-finite value met while locating a cut reported as `MeshingError::NonFiniteSdfOnEdge`. In
  `sim/L0/soft/src/sdf_bridge/stuffing.rs`'s unit tests: a steep and a repeated root, each found to 1e-12, the steep
  one in fewer evaluations than bisection takes.
- **The Parity Rule:** `stuffing_a_random_sign_field_leaves_no_face_unshared_off_the_surface` (400 random sign
  fields; it failed on the previous predicate), `the_parity_rule_is_the_papers` (worked cases from the paper's text),
  `a_long_edges_two_faces_take_opposite_diagonals`, and
  `every_stencil_splits_a_whole_long_face_along_the_parity_rules_diagonal`.
- **Mutations:** each of those checks failed under a mutation of the code it guards, among them the diagonal flipped
  in every stencil at once, which no angle test sees. The regula falsi's fallback to the midpoint when the secant
  leaves the bracket runs in no test.
- **The default path**, before the Parity Rule fix, hashed the same as the previous code's on six meshes (a comparison
  not kept).
- **Downstream of the mesher:** the release suites of `sim-soft`, `cf-sim-research`, `cf-fsu-model`, `sim-bevy-soft`,
  `cf-studio-engine` and `sim-coupling`, and the ten `sim-soft` validators CI runs, pass. Two examples CI does not run,
  `soft-drop-on-plane` and `hertz-sphere-plane`, pin values the fix moved; each was diagnosed and re-captured, with
  the diagnosis beside its constants.
- **Round 1:** three cold reviewers (the mesher with 23 mutations; the measurement and this record; the whole plan)
  raised 33 findings. One was a code defect: the located-cut search could stop far from a repeated root, and now
  bisects every fourth step. Most of the rest were settled by measuring (the parity-signed grid, all three insets,
  the worst corner, the step's accuracy) or by tests (the error path, each stencil's diagonal, a repeated root), and
  the others by correcting or cutting this record's explanations.
- **Round 2:** one fresh reviewer on round 1's fixes: 17 findings, 15 of them created by those fixes, most of them
  prose, several wrong when measured (what moved the Hertz example's values, the drop example's residual motion); and
  the probe's located-cut rows re-read at the final code. They were corrected or cut, and the rounds stopped there.

### 16w. Step 6: the lowering (design, 2026-09-27)

The last of step 6's three PRs (§15g step 6's note): the explicit solver's model, holds, path and obstacle, built in
`sim-soft` from the copies `cf-sim-research` measured with (§16j, §16t), and the items step 6 carries. This is the
design, written before the code and revised after its review (below). The results are added below it.

**Scope (Jon, 2026-09-27).**
- Two of step 6's boundary options move to a PR of their own, which lands before any verdict that uses them. The
  hand, as a soft distributed support, needs a hand's stiffness, and none is sourced. Bonded layers need a two-layer
  reference to choose the interface rule (§15g step 1), and `base_mold` has one layer. Step 7's first run needs
  neither.
- Step 7's first run holds the wall by a mount at its closed end (fit plan U14).
- *(An engineering call, after the review.)* The case the wall slides along joins them. A per-node constraint is a
  fixed direction, so a node slides along its tangent plane and off a curved case: on the cased tube that let the
  tube turn about 10° and its case open, and the pressure read 3 % of the oracle's (§16n). A case needs the wall's
  outer skin in contact with a second rigid surface, and the executor takes one obstacle.

**Where it lives.**
- `sim_soft::lowering` holds the model, the holds and the fitted path with its start and its sampling in time.
  `sim_soft::pairing` holds the friction library.
- `sim_soft::obstacle` gains the scan's exact signed distance as a value (`SignedDistance`), so the bake, the path's
  start and step 7's G2 read one definition.
- **The wall's body stays with its caller.** `cf-sim-research` builds it with `cf-design`'s `pinned_floor_shell` over
  the scan's exact distance, as §16v's probe did. `sim-soft` meshes it with its cut points located (`CutPoints::Root`)
  and lowers it.
  - `sim-soft` does not depend on `cf-design` (§14c).
  - Moving the product scene out of the tool, whose crate the app cannot depend on, is the fit plan's Phase 5
    (its §2c).
  - The material field and each element's layer read the field the wall is meshed from, as §16v's probe did;
    `build_insertion_geometry` keys them on the old grid.

**The model.** `explicit_budget.rs`'s measuring copy, moved. It keeps the vertices the elements name and the elements
as meshed. Per element it takes the mesher's μ and C₂, λ from ν, η = τμ, and the caller's density.
- It is the model §16r and §16v measured.
- It refuses a wall where two materials meet: their pressure rule waits on the deferred PR above.

**The holds** are sets of vertices held whole. They read the wall's outer skin: the boundary faces whose centroid
lies within half a lattice cell of the outer surface's level.
- Only boundary faces are read, so no vertex inside the wall is taken; the old path's pin took 646 (fit plan,
  Phase 1).
- The field is the outer surface's own: the scan's signed distance less the outer offset. The outer shell's field
  closes the mouth with a floor, and the faces beside the mouth read near zero on it, the canal's included
  (`the_skin_needs_the_outer_surfaces_own_field`).
- Where the outer surface ends at the mouth's rim, the skin also takes vertices in the mouth's plane near the rim:
  within a cell of it on the tests' cup. Which faces bring them in, faces bevelling the rim or flat ones beside it, is
  not separated.
- **Mounted:** the skin beyond a plane. Step 7's mount is the plane through the centreline's first point, normal to
  it there; that point is the seated tip (`slide_pose_at`'s convention). An engineering call (§9, labelled
  2026-09-28).
- **Bonded to a rigid shell:** the whole skin.
- **Nothing held:** for measuring the model alone, as the budget's step does; a scan pressing an unheld wall carries
  it off.
- The confined tube passed with its outer wall held whole and every node's axial motion held (§16n–§16p). The mount
  and the shell hold only the skin's vertices, which that case does not test (§9 decision 11's gate).

**The fitted path.** `path_room.rs`'s measuring copy, moved (§16t).
- **What it holds:**
  - the centreline's frame, carried by parallel transport;
  - the slide;
  - the rigid motion closest to the slide, in least squares over the scan's surface that the slide puts inside the
    device, each vertex weighted by its area.
- It computes each vertex's arc and place in the frame once; the copy did so for every pose.
- The copy's known-answer tests move with it. The three that compare with the pose as written stay with
  `slide_pose_at`, which retires after step 7, and call the moved code.
- **The walk** is the tip's arc from its seat. It grows outward.
- **The device** is the intersection of the cap planes' inner sides; a vertex is inside it when it lies on the inner
  side of every one.
- **Where the fit has nothing to fit.**
  - The fit is used while at least three of the scan's vertices with area lie inside the device and the
    cross-covariance the fit decomposes has a second singular value above a millionth of its first.
  - Walks are checked outward from the seat on a 0.5 mm grid. The join is the last one before the first where the
    fit is not used. Past it, the pose keeps the join's rotation and moves along the centreline's direction there.
    With no cap planes, the fit is used everywhere and there is no join.
- **The start.**
  - The run starts at the first grid walk, from the join outward, at which every boundary node of the wall lies at
    least 5 mm outside the scan by the scan's exact signed distance: §15b's start gap, taken here as a clearance.
  - So the scan starts with at most its tip inside the device, where the fit last has anything to fit, and clear of
    the wall. No node starts inside it, as nodes did at the old path's t = 0 (fit plan, Phase 1), where the contact
    law reads the coarse grid (§16u).
  - The search stops at the join plus the diagonal of the scan's box and the clearance, and fails there.
  - The travel is longer than the centreline and 5 mm, which §16r's budget took; step 7 re-reads it.
- **One fit per scan:** the fitted pose does not depend on the wall. Its start does, through the clearance.
- **In time:** the walk goes from the start to 0 over the loading time with §15b's profile, then holds at the seat.
  The speed ramps up over the first tenth of the loading time and down over the last. Step 7 sets the loading time.
  The profile's shape does not depend on the loading time, so neither do the sampled poses.
- **Sampled evenly in time**, as the obstacle takes it; the executor interpolates between samples (§15g step 1).
  - The count of intervals doubles from 64, to at most 4 096, until the interpolated pose lies within G2's floor
    (0.02 mm) of the fitted pose at every scan vertex the fitted pose puts inside the device. It is read at a
    quarter, a half and three quarters of every interval. Past 4 096, the sampling fails with the error it reached.
  - The fitted pose jumps where a vertex crosses a cap plane, and a jump does not shrink with the interval; the
    doubling ends only if every jump is below about the bar. The jumps on `base_mold` are measured, as the error at
    each doubling.
  - The fitted pose's own distance from the slide is millimetres (§16t), so the sampling stays below both it and
    what G2 tolerates.
  - Step 7 judges G2 against the pose the executor ran.

**The band.**
- A node's predicted position reaches past the surface by how far the node and the scan close in a step, and by
  the step's inward push, `F Δt²/m`, which the contact takes back each step. On the confined tube the push is about
  a twentieth of the mandrel's radius (§16o); held at the seat, it is the whole depth.
- Neither is known on the product before a run. The band starts at four fine cells (0.25 mm), and the product's
  frictionless run on step 7's wall reads the deepest predicted point. If that is past half the band, the band
  becomes twice the deepest predicted point, rounded up to whole fine cells. The factor of two is an engineering call
  (§9, labelled 2026-09-28).
- Two monitors make it a reading in every run: the deepest predicted point, and the corrections that took their
  depth from the coarse grid. A run with any says so.
- The band is the bake's, so one bake serves every corner and inset of a scan, until a run reads past half of it;
  then the band is re-set by the same rule and the scan baked again (§15g step 7). *(2026-09-28, §16x: re-set only
  when a correction reads the coarse grid; past half the band is a warning.)* The grids are §16u's: coarse at
  0.5 mm with a 4 mm margin, and fine at 0.0625 mm (Jon).

**The obstacle on the path:** the bake's grids, the sampled poses from time 0, and the run's μ_f.

**The pairing library** (`sim_soft::pairing`).
- It holds §5c's rows as data: a pairing is what slides on what, in which lubricant and state (fresh, or a time
  after), with each measurement it rests on, its phase (onset or sliding), its source and what the source measured.
- A lubricant's states are separate pairings, so a verdict picks one; the later ones show where use takes it.
- Where a source gives a mean and a spread (dry sliding, 0.61 ± 0.21), the pairing keeps the mean ± the spread, not
  the data's range; the dry pairing's low corner is that lower end. An engineering call (§9, labelled 2026-09-28).
- §5c's water-alone row has no measurement on skin, and its tacky pad bounds friction from below without a value;
  neither is a pairing.
- A verdict's friction corners are μ_f 0 and the pairing's lowest and highest values (§15h). They span onset and
  sliding, so a verdict's interval covers either; which one the solver should take is not settled (§5c).
- No nominal is sourced, so the library has none; which value D3 judges at stays open (the list for steps 6–9).
- A corner above μ_f 0.3 is marked unchecked: before a verdict there is trusted, its Coulomb push is checked there
  and the damping (fit plan U15) is settled, and above about 1.0 a finer mesh need not converge (the list).

**F3** (`sim/L0/soft/tests/material_conformance.rs`).
- It compares `sim-soft`'s `Yeoh` and `NeoHookean`, energy and first Piola stress, against the shared math's at f64,
  both the whole stress and the split the executor runs: the μ terms per element and the λ term's pressure.
- The deformation gradients run from near rest to principal stretches below 0.2 and above 2 (asserted). The bar is
  1e-12 of the moduli, scaled by
  `(1 + |F|²)²`: `sim-soft` forms `I₁ − 3` and `ln det F` directly, so near rest its rounding is the moduli's.
- It also compares the shared validity check, `J ≤ 0`, against the determinant.

**#976's merge review.** Its two low findings: §16v's "moves" for a press read before and after the Parity Rule fix
with other changes between, and `soft-drop-on-plane`'s protocol naming a toolchain its re-capture note does not.

**Done when** (set before the code; revised with the design):
- **In CI:**
  - the lowering keeps every element, names each node's vertex and material, and refuses what it cannot lower;
  - the skin is the boundary on the outer surface and vertices in the mouth's plane near the rim, and each hold keeps
    its vertices still
    through a press on a cup;
  - the fitted path passes the known-answer tests, and the fit uses the points the slide puts inside;
  - on a curved path where 64 intervals are not enough, the sampling meets its bar at the quarters it reads, and
    lies within 1 % of it at 16 times an interval that it did not read (revised after the build, 2026-09-27, my
    engineering call: first set as within the bar); and the start is clear by its clearance;
  - the two monitors read the corrections' own depths, and count every coarse correction and no fine one;
  - the pairing library gives its corners and marks the unchecked ones;
  - F3 passes.

  Each new check fails once under a mutation of the code it guards.
- **Locally, on `base_mold`** (figures stay local; ratios and verdicts go here):
  - `path_room`'s probe reads the same room at each pose through `sim-soft`'s fitted pose as through its copy, to
    its printed digits, on one wall and through one grid (§16u, §16v);
  - step 7's wall at h_K2, lowered and mounted: it is one connected piece; how many vertices inside it the old pin's
    rule would take;
  - whether its surface touches itself, the fit plan's Phase 1 finding, now on the fixed mesher (§16v): its boundary
    edges not on exactly two boundary faces, and its boundary vertices whose faces form more than one fan;
  - the path's join, its rotation about the join, its start's clearance, and the sampling's error at each doubling;
  - the band and the bake;
  - a frictionless run from the start through the hold stays finite with no element inverted, and reads the deepest
    predicted point and the coarse corrections;
  - a diagnostic: the same wall and obstacle, with the scan held still at the old path's start pose as §16r's timed
    run held it, for 100 steps. My prior, written first: it goes non-finite. If it does, the start's intrusion is
    enough to do it on the new wall; §16r's own wall, mesher and grid are not re-run.
- **The measuring copies are deleted,** and their probes call `sim-soft`.

**Not decided here (step 7):** the loading time; at which steps G2 is read; the reading of the sideways force and
twist (§16t); D1's push on a turning path.

**How the design was checked.**
- **Priors first:** twelve defects I suspected, kept from the reviewers.
- **Two cold reviewers** read the first version, one against the code it moves or reads and one against the plan's
  text. Between them they raised about 30 distinct findings, the case twice. What changed:
  - the case repeated §16n's measured defect; none of the priors named it;
  - the band left out the step's inward push, which sets the depth at the seat, so it now starts from a
    measurement, with a monitor in every run;
  - the sampling had no cap, and the fitted pose's jumps could keep it from finishing;
  - the skin's field was not named, and the outer shell's own would take the canal's rim;
  - F3 compared a stress the executor does not run;
  - the mount's consequences, the handoffs to step 7, the friction check's second condition, and three citations
    were missing or wrong.
- About half the priors hit, most in part.

**What was built and measured (2026-09-27).**

*In CI*, each check made to fail once under a mutation of the code it guards:
- `sim/L0/soft/tests/lowering_holds.rs`, the model and the holds, on a cup meshed with its cut points located:
  - the skin is every boundary vertex on the outer surface, plus vertices in the mouth's plane within a cell of the
    rim, and no vertex inside the wall;
  - read against the outer shell's field, which closes the mouth, the skin would take the canal's rim;
  - the old pin's rule takes vertices inside that cup's wall;
  - held vertices stay still through a press, while an unheld cup is carried off;
  - the lowering names every element's vertices and its material, C₂ included, and refuses what it cannot lower,
    a material that is not finite as such.
- `sim/L0/soft/tests/lowering_path.rs`, the path:
  - the copy's known-answer tests, and a planar arc's slide shown to be one rigid motion;
  - the join and the start where a straight tube's geometry puts them, the join also past the centreline's end;
  - past the join on an arc, the join's rotation moving along the centreline's direction there;
  - the start searched from the join, and a clearance that is not finite refused;
  - the fit over the points the slide puts inside, and none over points on one line;
  - a short segment along a centreline, as a trim leaves, turns nothing; one repeated a nanometre to the side turns
    the tangent 45°, as the centreline's doc says;
  - on an arc, the sampling read at 16 times an interval that it never read, within 1 % of its bar, and its first
    error recomputed; the cap failing; the time profile equal to the tube's; the obstacle on the path.
- `sim/L0/soft/tests/material_conformance.rs` (F3): principal stretches from below 0.2 to above 2. The worst difference
  is about a two-hundredth of the bar (printed with `--nocapture`). A mutation of the executor's pressure term fails
  it once the split is compared; the whole stress alone did not see it.
- `sim/L0/soft/tests/pairing_library.rs`: every pairing's corners, and the checked mark.
- `sim/L0/soft-explicit/tests/executor.rs`: the deepest prediction equals the corrections' own depths, and leaves out
  a held node the floor passes through; every correction without a fine grid is a coarse one, and none with one.

*The moves were exact* (comparisons not kept):
- the moved centreline and fit agreed with the copy bit for bit at 7 395 comparisons over three synthetic curves;
- U3's probe prints the same output through `sim-soft`'s fitted pose as through its copy, timing aside: the
  done-when's reproduction, on one wall and through one grid;
- after the lowering moved, the U17 probe reads every measured line of #976's archived run;
- after the executor kept its predictions in a pass of their own, the frictional tube reads the same line at f32 and
  f64, wall-clock aside; a review's hash of a tube run's state (a check not kept) agreed with the previous executor's
  at both precisions, with and without a fine grid;
- a bake hashed the same before and after `SignedDistance` came out of it.

*On `base_mold`* (`insertion_sim::product_lowering`; ratios and verdicts here, the rest local):
- **The wall:** step 7's wall at h/h_K2 0.991 is one piece. Its surface does not touch itself on the fixed mesher:
  no boundary edge lies on other than two triangles, and no node's triangles form more than one fan. The fit plan's
  Phase 1 finding was on the old wall. The old pin's rule takes vertices inside this wall too; the skin takes none
  (counted).
- **The path:**
  - the join lies at about 1.1 of the centreline's length, past its end;
  - at 2, 1 and 0.5 mm before the join, the fitted rotation is within 0.0003° of the join's;
  - the start is the join itself: where the fit last has something to fit, every wall node already lies the
    clearance outside the scan;
  - so the travel is the join's walk, longer than the centreline and 5 mm that §16r's budget took.
- **The sampling:** 128 intervals meet the sampling's bar (G2's floor): 2.43 of it at 64 and 0.89 at 128. The error
  fell 2.7 times at the doubling, not the 4 of a smooth path, and did not stall; what sets that rate is not isolated.
- **The run**, frictionless, from the start through §15b's 0.2 s hold:
  - ν 0.49 with Ecoflex's η/μ, and the tube's mass damping (ξ 0.05 at the shear wave's period along the centreline),
    at §16r's loading speed;
  - at a band of four fine cells (the probe's run before the band changed): finite, with no element inverted, no
    correction taken from the coarse grid, and the deepest predicted point at 0.91 of the band. That is past half, so by the rule the band became eight cells
    (0.5 mm);
  - re-run at eight cells: the same run, the deepest predicted point at 0.455 of the band;
  - at both, the grid's deepest penetration was 0.046 of G2's bar at the 5 mm inset. That is the grid's own reading;
    G2 against the scan's exact distance is step 7's.
- **The mount** leaves the rest step as it was: mounted over unheld, 1.0000. The stability condition,
  `4M − Δt²K − 2ΔtC` positive definite (§16p), holds on the free nodes' principal submatrix whenever it holds on the
  whole, so holding nodes cannot shorten the stable step.
- **The bake** at eight cells takes about three tenths of D4, once per scan and band. Its fine values at f32 are more
  than four times wgpu's default
  storage binding.
- **The diagnostic:** the old start pose, held still for 100 steps with no mass damping, puts nodes inside the scan
  by under a millimetre. As §16r's timed runs held it (unheld, on the f32 executor, elastic and viscous), and mounted
  at f32 and f64, it stayed finite with nothing inverted. My prior, that it would go non-finite, was wrong. So on the
  new wall and bake, neither the start's intrusion nor the hold, the precision or the viscosity does it in 100 steps.
  What made §16r's attempt go non-finite is not isolated; its wall, mesher and grid remain different.

*Not measured here:* where on the wall the deepest predicted point lies; the band's reading at any other corner (step
7's runs read it at each, and re-bake past half); the fitted pose's jumps apart from the doubling, which showed no
plateau at 64 and 128; the mount's confinement against ν. The product's mass damping and loading speed are the
tube's rules carried over, and step 7 sets them.

**How this was checked.**
- **The design** (above): priors first, then two cold reviewers of its first version.
- **The build, round 1:** three cold reviewers of the code with 43 mutations, of this record, and of the whole plan
  from its text raised about 40 findings; ten priors were kept from them, and three hit, in part.
  - Two survivors mattered: the lowering's C₂ was untested (the cup had none, and every catalog silicone has one), and
    the join's search past the centreline's end, where the product's lies, was untested. Six more survivors of the
    review's and one of mine were pinned; each now fails a test.
  - `start` never returned for a clearance that was not finite; it is refused. A centreline point repeated all but
    exactly turns the tangent 45°; `base_mold`'s centreline has no such point, and the limit is documented.
  - Two reviewers found that three of this record's ratios gave back a length of the scan by arithmetic. The record
    now states the travel against the centreline as a comparison, not a ratio.
  - The rest: this record's claims against the probe (it now counts what it states), the diagnostic's differences
    from §16r (now run as §16r ran), and handoffs to step 7 and annotations in the fit plan that were missing.
- **Round 2,** one reviewer on round 1's fixes: 10 findings, 9 of them created by those fixes. One was a code defect:
  round 1's refusal of a centreline with a very short segment also refused the short end segments a trim leaves, and
  it is removed (the limit is documented and tested instead). The rest were this record's prose, two ratios that gave
  back local figures (now bounds), and the confined case's gate for a skin-only hold (handed to step 7). The rounds
  stopped there.

### 16x. Step 7's first run (design, 2026-09-28)

The first of step 7's runs on `base_mold` (§15g step 7), the last PR of the arc Jon set on 2026-09-26: step 6 and
step 7's first run, ending at his call on whether the GPU (steps 3–5) comes next. That call rests on what a press
costs on the CPU at the element size and loading time D1's readings need on the product, neither of which is measured
yet (§16s, §16j). This is the design, written before the code and revised after its review (below). The results are
added below it.

**Scope.**
- One press on `base_mold` at its 5 mm inset, with the measurements that decide the element size and the loading time
  its readings need, and so its cost; the prints §15g step 7 asks of every run; and #977's merge-review items.
- Not here: Tier 1, and the path's own share beside it (the fitted pose against the slide); the lip radius; D1's
  limits, and so a verdict; other insets and D3's search; the hand, bonded layers and a case the wall slides along
  (§16w); the pairing's nominal corner (open); friction above μ_f 0.3. The old path and `slide_pose_at` retire after
  step 7's last PR, not this one.

**The press.**
- **The wall:** step 7's wall (§16v), lowered and mounted at its closed end (§16w), at ν 0.49 with Ecoflex 00-30's
  η/μ and the catalog's density, as §16r and §16w lowered it.
- **The obstacle:** the fitted path from its start (§16w), baked at `PRODUCT_BAKE`.
- **The pairing:** silicone on skin, water-based gel, fresh, whose corners are μ_f 0, 0.104 and 0.18. It is one of
  the library's two pairings whose corners are all checked (`FRICTION_CHECKED_TO`), and the one that holds the
  library's lowest μ_f, at which Jon decides D1's push (§15g's list for steps 6–9).
- **Mass damping:** ξ 0.05 at `ω₀ = 2π/T_s`, with `T_s = 4ℓ/c_s` and ℓ the centreline's length: §15c's rule, the
  tube's fixed–free formula applied to the product, as §16w's run applied it. The product's own lowest period is not
  measured. An engineering call; rule 1 measures the damping's effect on the readings together with the viscosity's.
- **The hold:** §15b's 0.2 s, read in quarters (rule 6).
- **The executor:** the CPU at f32, as the budget timed it (§16r); f64 where f32 is compared with it.

**The instruments.**
- **Two monitors join the executor's**, reduced as the resultant is:
  - the moment of the contact forces about the obstacle's body origin as posed at the step's time, `Σ (xᵢ − p) × fᵢ`
    with xᵢ the node's position at the start of the step, averaged over the steps since the last read;
  - cumulatively, the work the obstacle's motion does against the contact forces: over each step, `F · Δp + M · φ`,
    with F and M that step's resultant and moment, Δp the body origin's move over the step and φ the world-frame
    rotation vector of the obstacle's turn over it, both from the pose track at f64.

  An engineering call. The push needs the work integrated every step, which only the executor sees; read from the
  monitors' means instead, it depended on how far a read reached, and a read at the budget's speed reached 13 mm on
  the tube (below). Both go to the GPU too (step 4).
- **D1's push readings** are windowed means over travel of the work's change, per unit of the walk's change (from
  `travelled`): the peak push is the largest mean over 1 mm, and the geometric share the largest over 10 mm on the
  μ_f 0 run (§16s). The 1 mm is an engineering call that pins the instrument without changing D1's definition, which
  is Jon's; the tube's reads at 10 T_s reached about 0.8 mm (arithmetic on the review's 13 mm at a sixteenth of that
  loading time). Reads come often enough that one reaches at most a quarter of a millimetre at the top speed, and the
  work is interpolated between them. The peak over 0.5 and 2 mm is printed beside it.
- **Along the path** is the direction the posed seated tip moves per unit of walk over the sampled path's interval
  that holds the read's time (the pose the executor ran; in the hold, the last interval). The sideways force is the
  resultant less its part along the path; the twist is the moment about the posed seated tip.
- **At a pose held still,** the push is the static one: `F · dp/ds + M · dφ/ds` over the same interval.

**What every run prints** (§15g step 7):
- D1's readings: the peak push (1 mm) at each friction, the geometric share (10 mm) at μ_f 0, and the seated patch;
  beside them the pointwise pressure's area-weighted 95th percentile and peak, the loaded surface area inside the
  winning patch, and where the patch sits, as its centre's arc along the centreline over the centreline's length.
- The sideways force and the twist at every read, over the push and over the sum of the normal forces.
- G1 and G2 against the scan's exact signed distance (rule 8), and the grid's own every-step monitor.
- The deepest predicted point over the band, and the corrections the coarse grid answered (rule 7).
- The validity gates, over every phase a reading is taken from (§15a as amended): kinetic over internal energy over
  the loading from the first contact and over the window, and the energy balance; and K4, no element inverted
  *(corrected 2026-09-28: first listed inversion among the validity gates, which §15a keeps apart)*.
- On a frictionless run, the contact work over each quarter of the hold (rule 9).
- Steps, the time of the stepping alone and of the probe's instruments, and the loop's estimates.
- Once: the travel against the centreline and §15b's 5 mm, as a comparison (§16w); and the push's linearity in μ_f,
  the frictional push less the frictionless, at 0.18 over at 0.104, against 0.18/0.104 (§15g's list: the
  alternative to a nominal corner).

**The rules, set before the runs.** Each is an engineering call unless it names Jon. Where a rule borrows K5's 5 % or
K3's 0.5 %, it says so; the bars do not add up to a verdict's error.
1. **The loading time: a ladder on the product** (§16j). At h_K2, μ_f 0 and 0.18, at the budget's loading `T_r` (v/c_s
   0.389, §16r) and at 2, 4, 8 and 16 `T_r`. The readings are the peak push at 0.18, the geometric share at 0, and the
   patch at both.
   - The loading time is the first rung T from which both doublings, T/2 → T and T → 2T, move every reading by at
     most 5 % (K5's bar). One pair is not enough: on the tube, the frictionless 10 mm push is not monotone in the
     loading time (below).
   - If no rung up to 8 `T_r` meets it, the loading time is open, and D4 is read at 16 `T_r`, marked as a bound from
     below.
   - The chosen rung and the next are run again at the element size rule 2 picks; if a reading moves more than 5 %
     there, rule 1 is read again at that size.
2. **The element size.** At that loading, the three corners at h_K2 and at two and four times its element count
   (each wall meshed as step 7's is, at the lattice the same secant finds; the achieved counts' ratios printed), and
   a replicate at h_K2 on a lattice shifted by half a cell, whose difference from h_K2 is the meshes' scatter.
   - **Deciding readings:** the patch at every corner, and the geometric share. The peak push at μ_f 0.104 and 0.18 is
     reported beside them for Jon, whose decision it is how the peak is read at a low friction (fit plan D1).
   - D1's readings need the coarsest size from which doubling the element count moves every deciding reading by at
     most 5 % (K5's bar and its doubling, 50k to 100k). If the scatter is 5 % or more, the rule cannot tell, and says
     so.
   - Per reading, `e = C·hᵖ` is fitted over the three sizes (the stop rule's model, §15g step 2), and the remaining
     error it extrapolates at the chosen size is printed: over a doubling of elements h shrinks only by 2^(−1/3), so a
     5 % change can leave more than 5 % to go (§16s: 100k read 3.6 % below the 4× mesh after a 0.87 % step).
   - If neither doubling passes, the size is open: four times or finer, since four times is not judged without eight
     *(corrected 2026-09-28: first written "finer than four times")*; D4 is read at the size the fit extrapolates to,
     labelled so.
   - Under the mount this is also the discretization's check on the product's own confinement.
3. **ν under the mount** (#977's review). At h_K2 and the loading time, the three corners at ν 0.495 and 0.4975 too;
   each step doubles K roughly (K/μ about 50, 100 and 200). The silicone's K is not known: §5b's sources put rubbers at
   K/μ 1 000–10 000, beyond any step here, and on the tube with its outer wall held the band moved +32 % and then +21 %
   a doubling (below). The printout is each reading's change per doubling. Readings converge in K here if the
   second change is within 5 % (K5's bar) and at most half the first. Which ν verdicts read at, and whether ν becomes
   a corner as μ_f is, trades speed against quality, so it goes to Jon with these numbers (§9 decision 12).
4. **The confined case, held on the surface only** (§9 decision 11; #977's review). 15d.8's case (λ_a 1.1, B/A 2,
   ν 0.49, frictionless) in a new fixture, `Walls::Shell`: the outer wall held whole, both end faces held axially
   (frictionless end plates), every other node free, and the mandrel driven through the whole tube, its nose 5 mm
   past the far end, so it fills the bore end to end. Then the confined oracle's state (p/μ 4.1417) is the exact
   solution of the whole fixture: no axial motion anywhere, which the end plates allow, the outer wall still, and the
   bore at a.
   - The gate is 15d.8's: G2, and the band's raw error within 5 %. The fixture gives no gap-corrected error for a
     held wall (`TubeCase::errors`).
   - Validity: the band's axial stretch within 0.15 % of 1. Here `∂(p/μ)/∂λ_z` is −56.8 (`golden.rs`), so 0.15 %
     moves the reference by 2.1 %, as K2's 0.5 % does on the free tube (§15a).
   - At 50k and 100k, with the command recorded.
   - This covers a shell, which holds the whole skin. The mount holds less, and has no oracle of its own: taking it
     as covered between this case and the free tube that K2 passed is an engineering call. Rule 2's convergence is
     the check on the product itself.
5. **f32 against f64** (§15g step 5's note): the patch at μ_f 0.18 and h_K2, within K3's 0.5 %. The GPU runs f32 only
   (§1), so a failure bears directly on Jon's call, and is reported as such.
6. **The seated window.** The hold is read in four windows of 0.05 s; D1's seated reading is the last two together,
   §15b's last 0.1 s. The product's own window starts at the first quarter from which every later quarter's patch
   lies within 1 % of the last quarter's, a fifth of K5's bar. If the last two quarters differ by more than 1 %, the
   corner is run again with a 0.4 s hold and its readings come from that run. A shorter hold is a lever on the
   budget, reported, not taken here.
7. **The band** (#977's review: §16w's trigger at half the band had about a tenth of the band's headroom on the
   product). A run's readings stand if no correction read the coarse grid; a deepest predicted point past half the
   band is printed as a warning. A run with a coarse correction is run again after a re-bake at twice its deepest
   predicted point, in whole fine cells, and D4's figures then carry the re-bake. This replaces §16w's trigger, which
   re-baked runs whose corrections had all read the fine grid.
8. **G1 and G2**, at every monitor read, over every surface node the grid puts inside the scan or less than the band
   outside it, so that no node inside is missed, against the scan's exact signed distance (`SignedDistance`, the
   bake's own) at the pose the executor ran then; and at the end over every surface node. G2's bar is 1 % of the
   inset, 0.05 mm; G1's tolerance is the same. Between reads the grid's every-step monitor stands in. The exact
   reads are a CPU instrument, timed apart from the run. **G3,** the full seat, is met by construction: the path
   ends at the seat, so any valid run through the hold reaches it.
9. **Contact work over the hold** (§16o, §16u), on each frictionless run, per quarter of the hold. A scan held still
   does no work on a frictionless wall, while the kinematic law's corrections take work out as the wall settles, so
   the whole hold's net work can hide energy pumped in late (§16o's signature was a late rise). The bar is on the
   last half of the hold: work gained there at most 1 % of the internal energy at the seat, the energy balance's bar.
10. **When the contact-guided scan comes forward** (§16t: no reading was set). The question is whether a scan free to
    move would read a different D1 reading. On the frictionless run at h_K2, at the seat after the window, and on a
    frictionless run stopped at the centre of its largest 10 mm push window and held until rule 6's 1 % holds:
    - four degrees of freedom: two translations across the path and two turns about axes across it through the
      posed seated tip. The turn about the path's own direction is left out, and its moment is printed *(corrected
      2026-09-28: first justified as a turn a nearly round scan barely resists, which was not measured)*;
    - a stiffness matrix from central differences, ±δ and ±δθ in each, each probe moved over 0.05 s on §15b's ramp,
      held 0.1 s and read over a further 0.05 s, with the change of the force between that read's halves printed as
      its noise *(as built; first written "read over the last 0.05 s" of the hold)*;
    - up to three steps with that matrix toward zero sideways force and zero twist across the path, stopping when both
      are within a tenth of the fitted pose's; the residual is printed;
    - D1's reading there: the patch at the seat, the static push at the peak's pose. **If it differs from the
      fitted pose's by more than 5 %, the recommendation to Jon is that the contact-guided scan comes forward**; Jon
      placed it under the fit plan's Later.

    δ is 0.1 mm, 2 % of the inset, and δθ turns the scan's farthest point inside the device by δ. Friction is left
    out: a scan free to move would stick and slip, which this does not model; with friction the prints stand, with no
    rule.
11. **Stiffness scaling on the product** (15d.10; §15h's runs per verdict rest on it, checked only on the 10k tube at
    10 T_s, §16p). At h_K2 and the loading time, μ_f 0 and 0.18 at μ and 2μ, η/μ held: every force within 2 % of
    twice, 15d.10's bar. If it holds, a verdict is 3 runs; if not, 5 (§15h).

**The cost (G6).**
- At the loading time and element size the rules set, each corner is timed on the f32 CPU executor at 4 threads
  (§16r's), on an idle machine: the executor's setup, the loop's estimates, and every step and read. The probe's own
  instruments (the exact G1 and G2, the snapshots, the windows' readings) are timed apart and reported beside it. One
  corner is timed again at 8 threads, the M4 Pro's performance cores.
- A press is rule 11's 3 or 5 runs, reported over D4; with ν as a corner (rule 3), twice that at ν's own cost, read
  from its runs. The η range §16r took, Ecoflex's η/μ × 0.74 and × 1.47, is read as the rest step's factor.
- Beside it: the bake, once per scan and band, and a re-bake if rule 7 triggers one; and D4's search, 15 minutes over
  the full verdicts at the 1–2 insets Tier 1 would pick (arithmetic; Tier 1 is not built).
- **K1's per-step budget**, scaled by element count, is the bar the GPU must meet for D4 at K1's rate (§15a, §16r), not
  a projection of it: no GPU step has been timed, and a GPU's fixed cost a step need not scale with element count.

**The room on step 7's wall** (§16v): §16t's room, the posed scan's exact signed distance at a canal node plus the
inset, since the canal lies the inset inside the scan (first written "less the inset"; corrected with the results),
over §16t's 64 fitted poses, read through `SignedDistance` on both step 7's wall's canal nodes and the old wall's
(`sliding_product_scene`), so the ratio of their most room changes the wall alone. The ratio is public; both rooms stay
local.

**#977's merge review.**
1. ν's rule and the confined case for a skin-only hold: rules 3 and 4, which also go into §15g's list for steps 6–9.
2. The band's headroom: rule 7, with dated notes where §15g step 7 and §9 state the old trigger.
3. Step 6's done-when is closed in §15g with what met each bar, and the PR that §16w deferred (the hand, bonded
   layers' interface rule, and a case the wall slides along) gets a place in the build order, before any verdict on a
   design that uses them.
4. The mount's plane (through the seated tip, normal to the centreline there) is labelled an engineering call in §9.
   The probe prints the mount's extent locally; a length ratio would give back the scan's length from the public
   inset and wall, so the record takes only a rough bound on the share of the skin it holds.
5. §16w's other calls are labelled in §9: the start gap taken as a clearance, the band rule's factor of two, and the
   dry pairing's mean ± its spread; the sampling gate loosened after the build is attributed to me.
6. The pairing library carries its two surfaces as data, not only in a pairing's name.
7. The count of the old pin's interior vertices, now in `hold.rs` and §16w as in the fit plan: Jon's call. *(Jon,
   2026-09-28: keep it.)*

Step 4's note gains the two monitors.

**The code.**
- `sim-soft-explicit`: `Monitors::{contact_moment, obstacle_work}` on the CPU executor; in `readings`, the push over
  travel from the work, the direction along a sampled path, the sideways force, the twist, the static push, and the
  loaded area inside a patch; `Walls::Shell` in the tube fixture, and the tube probe takes the walls and the depth.
- `sim-soft`: the pairing's surfaces.
- `cf-sim-research`: the probe `insertion_sim::step7_first_run` (ignored; the scan stays local), and a lattice shift
  for the wall's replicate.

**Done when:**
- **In CI**, each new check failing once under a mutation of the code it guards:
  - the moment monitor equals `Σ (xᵢ − p) × fᵢ` from the phase outputs and the state before the step, at f32 and f64,
    on an obstacle that moves and turns enough in a step that the start and end of the step read differently;
  - the work monitor equals the work each node's contact force does over the obstacle's exact rigid motion of the
    point, summed over the steps, relative to within the largest turn in a step (the linearization's order), at f64,
    on an obstacle that translates and turns from a turned start (so a body-frame φ reads wrong);
  - the push over travel on a translating obstacle equals the resultant along it; the sideways force, the twist and
    the static push on known cases;
  - the loaded area on §16s's bore equals the ball's share of the cylinder;
  - `Walls::Shell` holds the outer wall whole and the end faces axially, and nothing else;
  - every pairing's surfaces.
- **On the tube:** rule 4 at 50k and 100k, run and recorded with its command.
- **Locally, on `base_mold`** (figures stay local; ratios and verdicts go here): every rule applied, with its numbers;
  G1, G2 and G6 at the 5 mm inset; D1's readings' convergence, the peak push's at μ_f 0.104 among them; the room on
  step 7's wall.

**How the design was checked.**
- **Priors first:** twelve defects I suspected in the design, kept from the reviewers.
- **Two cold reviewers** read the first version, one against the code and the physics, one against the plan's text.
  Between them they raised about 30 findings. The code reviewer measured three of its own on the tube, with a
  harness outside the repo (not kept) that reproduced §16o's confined figure, −0.18 % at 10k:
  - **The first rule 4 was wrong.** It held the outer wall alone, argued that far from the ends that is the confined
    state, and took §15b's tube. The band read p/μ 2.338 at both 10k and 50k, 44 % below the oracle, with its axial
    stretch 3.4–3.5 % above 1: the material escapes toward the free entry and the empty bore past the nose, and the
    tube is not long enough for the argument. The rule now holds the ends with plates and fills the bore.
  - **The first rule 1 read the monitors' 100-step means.** At the tube's rung a read reached 13.1 mm, eight reads
    over the insertion, so each rung changed the push's resolution as well as the speed: at that rung the frictional
    peak read 4.505 N from 100-step reads, 4.789 N from 10-step reads and 6.21 N from every step. Read every step,
    the frictionless 10 mm push fell from 0.319 N at the rung to 0.150 N at 8 times the loading time and rose to
    0.187 N at 16, so a single pair's change is not the error left. Hence the work monitor and two pairs.
  - **On that outer-wall-held tube, ν moved the band +32 % and then +21 % a doubling of K** (0.49, 0.495, 0.4975): one
    pair of ν cannot say ν barely matters. A third point was added, and the choice goes to Jon.
  - The rest: the contact-guided test probed one direction at a time, where a sideways move also turns the scan; the
    push and the sideways force were undefined at a held pose; rule 2's 5 % over a doubling bounds less than it
    reads, and required a reading whose definition is Jon's; the exact G2 reads missed nodes deeper than the band and
    their cost went unstated; K1's rate was put as the GPU's; the press's cost left out ν's branch, stiffness
    scaling, the viscosity's range and D4's search; a mount extent ratio would have given back the scan's length; and
    handoffs from the plan (the travel, the patch's place, the dated notes) were missing.
- About half the priors hit, most in part. None named the first rule 4's failure, which I had argued rather than
  measured.

**What was built and measured (2026-09-28).**

*In CI*, each new check made to fail under a mutation of the code it guards (each mutant's run printed a failing test
result, not a compile error):
- `sim/L0/soft-explicit/tests/executor.rs`, on a floor that starts turned, then rises, slides and turns about another
  axis:
  - the moment monitor equals `Σ (xᵢ − p) × fᵢ` from the phase outputs and the state before each step, to 1e-12 at
    f64 and 1e-5 at f32, and the origin posed a step later reads more than a thousand times the f64 difference away;
  - the obstacle's work equals the work each node's force does over the exact rigid motion of its point, within the
    largest turn in a step, and a turn taken in the body frame misses by more than ten times that;
  - `rigid_motion` on a turned pose, with the quaternion's sign flipped and with no motion; and `Monitors::finite`
    reads both new monitors.

  Seven mutations of the executor and `rigid_motion`, each failing a test.
- `sim/L0/soft-explicit/tests/readings.rs`: the push over travel from the work, on a force stepping from 2 to 5 N, with
  a pause mid-path and a hold; the direction along a path, the force across it and the moment about a point, on known
  cases; the static push against the exact rigid motion, the moment carried to the interval's origin beating it left
  where it was; the loaded area on §16s's bore and on a half-pressed plate. Nine mutations, each failing a test; the
  zero-travel guard survived until the pause mid-path was added.
- `tests/fixtures.rs`: `Walls::Shell`'s holds (two mutations); `sim/L0/soft/tests/pairing_library.rs`: the surfaces
  (one). *(2026-09-28, after the merge: #978's description counted 21 mutations of ours; this list counts 19.)*
- Round 1's code reviewer ran 31 mutations of its own. Two survived these checks: `set_poses` not updating the f64
  track, and the obstacle's move taken from the narrowed poses at f32. Two tests now fail on them: a new pose track
  moves the moment's origin and a still obstacle does no work; and f32 reads the obstacle's work within 1e-4 of f64's
  on a fixture about 0.35 m from the origin. A third survivor, keying `work_peak`'s windows at an interval's start,
  changed a push read on uneven reads by less than a tenth of a percent, and stands.

*On the tube, rule 4* (`cargo run --release -p sim-soft-explicit --example tube -- <mesh> 4 0 f32 20 0.2 10 1
0.00030434782608695654 <cased|shell>`, `RAYON_NUM_THREADS=4`, at `3256d838`):

| Walls | Mesh | Band against 4.1417 | Band's axial stretch | G2 on the grid, every step | KE/IE, balance |
|---|---|---|---|---|---|
| Cased (the check) | 10k | −0.18 % | +0.00 % | 0.1 µm | 0.00 %, 0.01 % |
| Shell | 10k | −0.20 % | +0.01 % | 0.8 µm | 0.00 %, 0.02 % |
| Shell | 50k | −0.11 % | +0.00 % | 0.2 µm | 0.00 %, 0.01 % |
| Shell | 100k | −0.06 % | +0.00 % | 0.1 µm | 0.00 %, 0.01 % |

The cased 10k run reproduces §16o's −0.18 %, a check on the instrument. **Rule 4 passes** at 50k and 100k: held on
its surface alone, with its ends on frictionless plates, the confined tube reads the oracle. That settles the shell's
side of §9 decision 11's gate; the mount's stays open (rule 2, below).

*On `base_mold`* (`insertion_sim::step7_first_run`, the scan local; ratios and verdicts here, the figures on the
machine that ran them; a ratio over D4 gives a compute time back, which Jon allowed, 2026-09-28). Each stage is `RAYON_NUM_THREADS=4 STEP7_LOADING=<rung> cargo test --release -p
cf-sim-research --bin cf-sim-research -- insertion_sim::step7_first_run::<stage> --ignored --nocapture`, with
`STEP7_LOADING=4` for every stage but `step7_ladder` and, for `step7_cost`, `STEP7_SIZE` 0 or 2. Unless a line says
otherwise: step 7's wall at h_K2 (h/h_K2 0.991), mounted, ν 0.49, Ecoflex 00-30's η/μ with the tube's mass damping
(§15c's rule, carried; rule 1 measures its effect with the viscosity's), f32, and rule 1's loading. Every run behind
rules 3, 5 and 11, and the run rule 10 starts from at the seat, passes the validity gates (kinetic over internal energy
at most 0.60 %, the balance 0.01 %) and K4; rule 10's probe holds and its run to the peak's pose are not gated.

- **Rule 1, the loading time: four times the budget's** (v/c_s about 0.097). Each reading's change from the rung before
  (`step7_ladder`, at `5ffacb6a`):

  | Rung | Peak push, μ_f 0.18 | Geometric share | Patch, μ_f 0 | Patch, μ_f 0.18 |
  |---|---|---|---|---|
  | ×2 | (−1.13 %, from ×1, not valid) | −10.17 % | −0.04 % | (+1.20 %, from ×1, not valid) |
  | ×4 | +1.71 % | −4.27 % | 0.00 % | +0.66 % |
  | ×8 | +0.06 % | −1.71 % | +0.05 % | +0.19 % |
  | ×16 | −0.47 % | −1.79 % | +0.10 % | +0.22 % |

  The budget's own rung fails a validity gate with friction (kinetic over internal energy over the loading, 5.73 %).
  Only the geometric share moves with the speed past K5's bar; at four times it lies within 3.6 % of sixteen times'.
  Rule 1's check at the size rule 2 picks did not run, since rule 2 picked none.
- **Rule 2, the element size: open, four times h_K2's elements or finer** (`step7_sizes`, at `3256d838`). Meshed at
  0.795 and 0.633 of h_K2, 1.94 and 3.85 times the elements *(2026-09-29, §16z: read against h_K2's replicate's
  scatter, as §16z's size rule reads a doubling, the second doubling cannot tell, and with an eight-times wall D1's
  size is not picked)*:

  | Reading | ×1 → ×2 | ×2 → ×4 | ×1 → ×4 | Replicate against ×1 |
  |---|---|---|---|---|
  | Patch, μ_f 0 | not valid | not valid | +13.66 % | −2.67 % |
  | Patch, μ_f 0.104 | +14.53 % | −2.35 % | +11.85 % | −3.01 % |
  | Patch, μ_f 0.18 | +18.84 % | −5.55 % | +12.24 % | −2.73 % |
  | Geometric share | not valid | not valid | +6.58 % | +0.20 % |
  | Peak push, μ_f 0.104 (for Jon) | +8.36 % | +2.19 % | +10.73 % | +0.27 % |
  | Peak push, μ_f 0.18 (for Jon) | +8.41 % | +1.46 % | +10.00 % | +0.28 % |

  - The replicate, h_K2's wall on a lattice shifted half a cell, moves each reading by at most 3.01 %, so the rule can
    tell 5 %.
  - h_K2 fails the doubling; the twice-refined wall fails it on the frictional patch (−5.55 %) and has no valid
    frictionless run (below); the four-times wall is not judged, since that needs a run at eight times. No order can
    be fitted. All of it is with the element as it is (below).
  - The peak push, which Jon decides at a low friction, moves 8.4 % over the first doubling and 1.5–2.2 % over the
    second, within K5's bar from twice h_K2's elements. It is read as its largest mean over 1 mm of travel; its
    largest means over 0.5 and 2 mm read 1.04–1.07 and 0.89–0.95 times that in every valid run.
  - The four-times wall's element size is about the 50k tube's (§16r), from which the tube's readings converge (§16s).
- **At the seated tip, under the mount, an element collapses** (`step7_blow_up` at `aa9681a7`; `step7_stiffening` at
  `1c271024`, frictionless, the step re-estimated every 50 steps so each runs through the loading):

  | Wall | Smallest step over the rest step | The most-compressed element's J, at the read of about the smallest step | Its nodes' averaged J | Where |
  |---|---|---|---|---|
  | h_K2 | 0.398 | 0.098 | 0.934–1.000 | 0.06 of the centreline from the seated tip |
  | ×2 | 0.060 | 0.163 | 0.888–0.994 | 0.03 |
  | ×4 | 0.181 | 0.089 | 0.888–0.989 | 0.04 |
  | ×2, nothing held | 0.925 | 0.705 | 0.980–0.996 | past the centreline's end |

  - At every size the most-compressed element, near the seated tip, is at 9–16 % of its volume while its nodes'
    averaged volume, from which the pressure is taken (selective ANP, §15g step 1), is within 12 % of its rest. The
    element is read at a monitor read whose step is within 5 % of the run's smallest; which element limits the step,
    and how many elements collapse, are not read.
  - With the loop's re-estimate every 500 steps (§15c), the twice-refined wall's frictionless run went non-finite at
    0.951 of the loading at f32, and at 0.936 at f64, which first inverted an element at 0.935; re-estimated every 50
    steps it ran through with no element inverted. The f64 run's first inverted element lay on the canal at the tip, a
    third of the mean volume. Under
    §15a an inversion in a valid run fails K4; its validity gates could not be read, since its later reads were not
    finite, so whether it fails K4 or was invalid first is not read. The probe counted an inversion as a failed gate;
    it now reports K4 apart (below). h_K2's and the four-times wall's runs stayed finite, with no element inverted, at
    500.
  - The seated patch centres at the seated tip in every valid run, near the collapsed elements. Whether the collapse
    moves the patch, and so rule 2's changes, is not measured; nor whether it relates to §15h's alternating pressure,
    which is a reading of the tube's rings.
  - Frictionless runs took more steps than frictional ones at the same loading (on h_K2's wall, 1.19 times the μ_f
    0.18 run's), and the twice-refined wall's run at μ_f 0.104 took 1.7 times its μ_f 0.18 run's. What sets that is
    not isolated.
- **Rule 3, ν under the mount: open; Jon's call** (`step7_at_h_k2`, at `5ffacb6a`). Each reading's change per doubling
  of K:

  | Reading | ν 0.49 → 0.495 | ν 0.495 → 0.4975 |
  |---|---|---|
  | Patch, μ_f 0 / 0.104 / 0.18 | +3.81 / +4.28 / +5.29 % | +2.91 / +3.22 / +3.65 % |
  | Geometric share | +1.96 % | +1.41 % |
  | Peak push, μ_f 0.104 / 0.18 | +2.66 / +2.41 % | +1.90 / +2.14 % |

  The second change is more than half the first for every reading, so by the rule the readings do not converge in K
  here; §5b's sources put rubbers at K/μ 1 000–10 000, several doublings past these.
- **Rule 5: f32 against f64** on the patch at μ_f 0.18: +0.002 %, within K3's 0.5 %.
- **Rule 6, the seated window:** in every valid run the patch settled from the hold's first quarter, each quarter
  within 1 % of the last; a shorter hold is a lever on the budget, not taken.
- **Rule 7, the band:** no correction read the coarse grid in any valid run, and the deepest predicted point reached
  at most 0.45 of the band.
- **Rule 8, G1, G2 and G3:** against the scan's exact signed distance, the deepest node over the reads lay at most
  0.096 of G2's bar inside the scan, and at the end over every surface node at most 0.019; G1, read the same way with
  G2's bar as its tolerance, is the same. On the grid, every step, at most 0.045. G3 is met by construction. The
  twice-refined wall's frictionless run reached 1.9 bars before it stopped.
- **Rule 9, contact work over the hold:** the work gained over the hold's last half was at most about 5e-6 of the
  internal energy at the seat.
- **Rule 10, the free scan: within 5 %; no recommendation.** At the seat the scan free to move would move about a
  tenth of the inset and turn under two degrees, and the patch changes −1.00 %; held at the centre of the frictionless
  run's largest 10 mm window, the static push changes −2.79 %. The steps took the sideways force and twist to at most
  3.1 % of the fitted pose's; the probes' noise, read on the force, was at most 0.16 %. The scan was free in four of
  the five freedoms the path leaves it: at the fitted pose the moment about the path's own direction, which was held,
  read 0.085 of the twist across it at the seat and 0.22 at the peak's pose; whether freeing that turn moves a reading
  is not measured.
- **Rule 11: stiffness scaling holds on the product:** at 2μ each reading is within 0.36 % of twice, so a verdict is
  three runs. The mass damping was held at μ's; at 2μ the tube's rule would raise it by √2, which was not run.
- **Beside them, over every valid run:**
  - the push is nearly linear in μ_f: the frictional push less the frictionless, at 0.18 over at 0.104, reads 1.686
    against 1.731;
  - the loaded surface inside the winning patch is 1.01–1.11 cm²; the pointwise pressure's 95th percentile reads
    0.49–0.69 of the patch and its peak 1.6–3.0 times it;
  - the sideways force is 0.8–2.3 % of the normal forces' sum at the seat, and 0.2–2.1 % of the push at the read where
    the push is largest; its largest share of the normal forces over the loading, 0.67–0.94, is not located; the
    twist about the seated tip is 0.003–0.006 of the normal forces' sum times the centreline's length;
  - the travel is longer than the centreline and §15b's 5 mm, and the start is the join, on every wall; the mount
    holds under a tenth of the skin's vertices;
  - **the room on step 7's wall** (`step7_room`, at `aa9681a7`): through the exact distance, step 7's wall's canal
    nodes ask 1.17 times the room the old wall's do at §16t's 64 fitted poses; 96 % of its canal nodes lie within
    1 µm of the inset.
- **G6, the cost** (`step7_cost`, at `3256d838`, the probe's instruments off, an idle machine; a press is rule 11's
  three runs at rule 1's loading):

  | Wall | A press over D4, on the CPU | A search of full verdicts at 1 and 2 insets, over its 15 min | A press at K1's per-step budget, over D4 |
  |---|---|---|---|
  | h_K2 | 0.38 (4 threads), 0.39 (8 threads) | 0.13 and 0.25 | 1.3 |
  | Four times h_K2's elements | 3.8 (4 threads) | 1.3 and 2.6 | 17 |

  - Eight threads did not speed the stepping up; the bake went from 0.29 to 0.16 of D4. Why the stepping does not
    scale past four threads is not isolated. In the stages' runs the probe's instruments took at most 11 % of the
    stepping's time.
  - With ν a corner, a press is twice the runs: from the ν runs' steps (1.10 and 1.27 times the ν 0.49 press's at
    0.495 and 0.4975), 0.80–0.86 of D4 at h_K2, and 1.0–1.1 at the viscosity's high end, taking the time as the steps
    (arithmetic). Across the viscosity's range (Ecoflex's η/μ × 0.74 to × 1.47) the rest step's factor moves the steps
    0.87–1.27 times at h_K2 and 0.83–1.39 at four times its elements.
  - The four-times press cost 10.1 times h_K2's, for 3.85 times the elements; their count and the element size's ratio
    alone give about 6. The steps were taken with the collapse at the tip present; how much of the gap it accounts for
    is not measured.
  - The CPU as timed runs 3.5 and 4.4 times faster than a device at K1's per-step budget, so K1's budget is no longer
    a bar that a GPU meeting it would clear: meeting D4 at four times h_K2's elements takes a device at least 3.8 times
    this CPU, about 17 times K1's budget (arithmetic). No GPU step has been timed. Steps 3–5 would run the element as it
    is (step 4 checks every phase against the CPU executor).
  - **For Jon's call:** at the element size K2 needs, the CPU meets D4 with room at ν 0.49 (0.38, and the bake once per
    scan), and about at D4 with ν as a corner at the viscosity's high end. D1's readings need four times that size's
    elements or finer, with the element as it is; at four times, a press takes 3.8 of D4 on the CPU. Before that size
    is read again, the element collapsing at the seated tip bears on it, and on the cost. Jon's rule stands that the
    quality items come before steps 3–5 (§15g step 2's note). *(2026-09-29, §16z: D1's size is not picked; at eight
    times h_K2's elements, the size used, a press takes 11.8 of D4 on the CPU.)*

*The commits the runs name* are pre-squash commits kept only on the machine that ran them; the merged probe is #978's
(`240844a3`). *Since the runs,* the probe's code changed only in which runs it lets feed the rules: K4 apart from the
validity gates, rules 6 and 7 enforced, the frictionless corner of `step7_at_h_k2` gated as the others are, and a rule
whose inputs include a run that did not stand printing "not judged". In `step7_cost` a rule-6 re-run would time the
re-run. No recorded run met any of those cases, so the recorded readings stand.

*Found by the runs, not by the rules:* the probe read an exact depth from a state that had gone non-finite and
panicked in the distance query; it now leaves such a read out. Rule 2 first computed its changes from the invalid
run's partial readings; a run whose gates fail now reads nothing for the rules. The room first read nodes at plus the
inset, off the canal, as the design first worded it; it reads the canal now.

*Not measured:* where the frictionless push peaks; where the sideways force's largest share falls; the order of
convergence; the eight-times wall; which element limits the step at the tip, and whether the collapse moves D1's
readings; whether a formulation without the collapsing mode, or with it stabilized, converges; what the step
re-estimated every 50 steps costs a press; what makes frictionless runs take more steps. *(2026-09-28, §16y: which
element limits the step is measured; whether the collapse moves D1's readings is open, fit plan U20; re-estimated every
50 steps a run evaluates the forces about three times per step, not timed; and the frictionless cells of rule 2 at
twice h_K2's elements are filled, the verdict unchanged.)*

*My priors, scored:* the loading at two or four times the budget's (hit); the frictional patch within 5 % from h_K2
to twice its elements (miss: +14.5 % and +18.8 %); ν's per-doubling change under 5 % (miss: +5.29 % on the patch at
0.18); G2 under half its bar (hit); the deepest predicted point shrinking with h, with no coarse correction (hit);
f32 = f64 (hit); the shell within 1 % (hit); no pumping (hit); the free scan within 5 % (hit, its move about my guess);
the hold settled by its third quarter (hit, the first); the patch at the mouth (miss: at the tip); a press at the size
D1 needs within D4 on the CPU (miss as far as measured: 3.8 of D4 at four times h_K2's elements). None named the
collapsing element.

**How the build was checked.**
- **Round 1:** three cold reviewers, of the code (31 mutations in a worktree of its own), of this record against the
  runs' outputs and for the scan's figures, and of the whole plan from its text, raised about 28 findings. The largest:
  - rule 2's verdict and the call for Jon claimed more than rule 2 found: four times h_K2's elements was never judged,
    and at the size it picks the cost is not known; the design's own wording ("finer than four times") carried it;
  - the bar a GPU must meet was put as K1's per-step budget, which the CPU already beats; it is now against the CPU;
  - explanations with no measurement behind them, of what sets the step and of §15h's pattern, were cut;
  - the probe gated all but one corner, counted an inversion as a failed validity gate where §15a keeps K4 apart, and
    printed rules 6 and 7 without enforcing them; two mutations of the executor survived its tests;
  - two exact node counts from the product mesh had reached this record, and several ranges were read from h_K2 alone.

  None of my twelve design priors named the collapsing element; round 1's findings were mostly about my own
  statements of the results.
- **Round 2:** one fresh reviewer read only round 1's fixes and found 12 problems, 10 of them created by those fixes,
  among them a verdict a rule would print from a run that did not stand, a blow-up point given to the wrong precision,
  and a figure the fix deleted while the fit plan still cited it. They were cut or corrected, and the rounds stopped
  there.

### 16y. The element collapsing at the seated tip (design, 2026-09-28)

Jon's call after §16x (2026-09-28): settle the element that collapses at the seated tip, finding which element limits
the step and whether the collapse moves D1's readings; then read D1's element size again; his GPU call comes after.
This is the design, written after the exploratory runs below, revised after two rounds of review (below), and set
before the deciding runs. Their results are added below it. #978's five merge-review items are folded in: §16x's rule 10 and its
record, the fit plan's note on it, step 4's note, the run commits and the mutation count, and the tube example's walls.

**Scope.**
- Which element limits the step at the seated tip (measured below).
- The step control: what a run that fails with the loop's re-estimate does (rule 1).
- Whether the collapse moves D1's readings (rule 2), with a volumetric stabilization built as the instrument, and as
  the candidate element if it does.
- Next, not here: D1's element size read again with the element rule 2 leaves, and G6 there, for Jon's call.
  - If the stabilized element goes forward, it first needs its own h_K2, K3, the Yeoh case (16h), the confined case,
    K5, K6, and CI at its weight. §16r's stop rule derives h_K2 for an element, and on the 10k tube at c = 2 K2's first
    corner reads +7.01 % (below).
  - ν under the mount (fit plan U19) stays Jon's call; its figures are the element as it is, and rule 2 was read at
    ν 0.49 only.

**Measured before the rules** (exploratory; the commits named are this PR's pre-squash commits, kept locally).

- **What limits the step** (`step7_stiffening` at `d61412c6`: frictionless, loading four times the budget's, the step
  re-estimated every 50 steps). At the read of the smallest step, within about 5 % of it, the power iteration's vector
  (`CpuExecutor::top_mode_and_vector`), by its mass-weighted size at each node:

  | Wall | Smallest step over the rest step | Share on the top node | Share on the most-compressed element's nodes | The vector's damping ratio | The elastic top mode's step over its own at rest |
  |---|---|---|---|---|---|
  | h_K2 | 0.398 | 0.93 | 0.998 | 1.17 | 0.68 |
  | ×2 | 0.060 | 0.96 | 1.000 | 3.90 | 0.28 |
  | ×4 | 0.181 | 0.83 | 0.000 | 2.15 | 0.41 |
  | ×2, nothing held | 0.925 | 0.92 | 0.000 | 0.82 | 1.01 |

  - At h_K2 and ×2 the step is set at the most-compressed element. At ×4 it is set at another element near it, at J
    0.45 against its nodes' averaged 0.84–0.99.
  - Elements under half their nodes' averaged J: about 1 to 2 in 10⁴ of the wall's elements, all within 0.06 of the
    centreline's length from the seated tip; none with nothing held.
- **The research round** (two reports, kept in the local archive with their sources):
  - With the λ term averaged over nodes, a motion that leaves every node's volume unchanged gets no stiffness from λ.
    Only the element's μ terms resist it: a bulk of about 2μ/3, plus 8C₂ for Yeoh. On one report's linear model, not in
    the repo (8³ cubes, bottom held, a point load), the worst element's volume change under selective ANP barely fell
    as λ rose (as λ^−0.19, against λ^−0.77 for the nodes').
  - Split-energy ANP (Bonet–Burton, Joldes' IANP, LS-DYNA's formulation 13) has no element-level barrier to a change
    of volume, by the report's inference. The element as it is keeps the μ terms' −μ ln J, so a split form is not
    taken as a fix.
  - The sources' stabilizations give each element back some volumetric stiffness of its own:
    - Krysl's energy sampling (ESNICE-T4, `FinEtoolsDeforLinear.jl`), the source of this stabilization's form: a
      material of ν 0.395 at ν 0.49, weighted 0.46 at a regular tet, a bulk of about 2.2μ (arithmetic), of which about
      1.5μ is beyond the 2μ/3 the μ terms here already keep per element;
    - Puso 2005: a penalty on the element's strain less its nodes', weighted 0.05, with ν 0.4 in its material;
    - Puso et al. 2008, as Ortiz-Bernardin et al. quote it: the penalty's λ capped at 25μ;
    - Sierra/SM's node-based tet: 0.01 of the element's bulk stress, about 0.5μ at ν 0.49 (arithmetic).

    No source was found that measures the locking against the weight at ν 0.49–0.4975.
  - Production codes track the step every cycle from a cheap estimate, and re-run a global one on a schedule or when
    the cheap one moves. Sierra/SM's power method does so every 50 steps, or on a 10 % change. The loop here
    re-estimates every 500 steps and tracks nothing in between.
- **The stabilization, built** (`ExplicitModel::with_volumetric_stabilization`, and per element
  `with_element_stabilizations`): energy sampling, in the log form.
  - Each element's κ_e (from the weight, min(λ_e, c μ_e)) moves from the averaged term to the element's own volume:
    `E = Σ_e V_e (Ψ_μ(F_e) + κ_e/2 (ln J_e)²) + Σ_a V_a (λ_a − κ_a)/2 (ln J_a)²`, with κ_a weighted at the nodes as
    λ_a is.
  - Where every element around a node has one J, the two terms add up to the λ term. With the same κ on every element
    around each node, while J stays below e, the energy otherwise rises by each patch's gap (`tests/elasticity.rs`).
    Where κ differs around a node, as on a mask's edge or where materials meet, it can fall: the design review
    measured a two-material compression with less energy at 4μ than at none (not in the repo).
  - No pass is added: the node pass uses λ_a − κ_a, and the element pass adds κ_e ln J_e / J_e
    (`sampled_element_pressure`, in the WGSL).
  - c = 0 is the element as it was.
- **On the tube** (the example at `5cfa84b0`: `<mesh> 0 0 f32 20 0.2 10 1 0.00030434782608695654 free <c>`, K2's first
  corner, the mandrel at 1.1 times the bore and ν 0.49, frictionless, at 10 T_s), K2's raw error:

  | c | 10k | 50k | 100k |
  |---|---|---|---|
  | 0 | +4.41 % | +1.47 % | +0.95 % |
  | 0.5 | | +1.72 % | +1.13 % |
  | 1 | | +1.98 % | +1.31 % |
  | 2 | +7.01 % | +2.47 % | +1.67 % |
  | 4 | +9.44 % | +3.44 % | +2.36 % |
  | 8 | +13.90 % | | |
  | 25 | +28.49 % | | |

  - The tube's worst element, over every read, stays above 0.93 of its nodes' averaged J at 10k and 50k (the design
    review's measurement, not in the repo), so there the tube's change with c is the stabilization's own, with no
    collapse; at 100k it was not read.
  - On K2's band it adds about linearly in c up to 4, and less as the mesh refines. At 2μ it adds 2.60, 1.00 and 0.72
    points: of order 1.7, then 1.5, in the element size (arithmetic, the size taken as the cube root of the element
    count).
  - On the tube's D1 readings at c = 2 (the design review's measurement, not in the repo), the patch moves +5.04, +2.51
    and +1.84 % at 10k, 50k and 100k, shrinking about as K2's change does; the 10 mm push +1.53, +3.76 and +0.68 %, and
    the 1 mm peak +2.30, +1.31 and +5.99 %, do not shrink in step.
- **On the product at h_K2** (`step7_stabilized` at `e057dfa0`: loading ×4; its presses at every corner and its
  collapse diagnostic frictionless, all with the loop's re-estimate every 500 steps), against c = 0:

  | c | The most-compressed J at the smallest step | The step's vector on the most-compressed element's nodes | Smallest step over the model's own rest step | Patch, μ_f 0 / 0.104 / 0.18 | Geometric share | Peak push, 0.104 / 0.18 |
  |---|---|---|---|---|---|---|
  | 0 | 0.10 | 0.998 | 0.43 | | | |
  | 2 | 0.29 | 0.000 | 0.59 | +6.3 / +6.9 / +7.9 % | +4.3 % | +4.5 / +4.6 % |
  | 4 | 0.36 | 0.000 | 0.59 | +11.4 / +11.7 / +13.1 % | +7.7 % | +7.6 / +7.7 % |
  | 8 | 0.53 | 0.000 | 0.59 | +18.6 / +19.8 / +21.9 % | +13.8 % | +12.9 / +13.7 % |
  | 25 | 0.63 | 0.000 | 0.58 | +48.3 / +48.6 / +52.7 % | +34.7 % | +30.6 / +30.7 % |

  - From 2μ on, the most-compressed element no longer sets the step. Each row's step is over its own model's rest step,
    so the rows are not step gains.
  - The readings rise with c at every rung, with no plateau.
- **The design review's public case** (a ball pressed into a block held at its base; `tests/collapse_release.rs`,
  which the review found):
  - The element as it is drives two elements under half their nodes' averaged J, the least at 0.386, with the step
    re-estimated every 50 steps. κ 2μ everywhere lifts the least to 0.551, and on those two elements alone to 0.547.
  - The seated vertical contact force at the last read moves +10.14 % stabilized everywhere and +1.26 % on the two
    alone (the test prints both). On the review's finer blocks, with no element under half at c = 0, 2μ everywhere
    still moved it +4.27 and +2.41 % (not in the repo). So rule 2 stabilizes the collapsing elements alone, and its
    change includes the mask's own.
  - With the step re-estimated every 500 steps, the element as it is goes non-finite (the test pins it); every 50
    steps it stands. On the review's case with a smaller ball, runs at 500 went non-finite at c = 0, 2 and 4, and
    those it ran again at 50 (c = 0 and 2) stood; at 8μ the run finished at 500 with an element inverted (not in the
    repo). A failure at 500 that stands at 50 is what rule 1 re-runs; what fails is not isolated.
  - Stabilized everywhere at 2μ, the collapse cleared in two of the six public cases where the element as it is had
    one (the review's measurement, not in the repo).

**The rules, set before the deciding runs.** Engineering calls unless one names Jon.
1. **The step control.** A run that goes non-finite, inverts an element (K4) or fails a validity gate with the loop's
   re-estimate every 500 steps is run again with the step re-estimated every 50 steps, the interval Sierra/SM's power
   method uses by default. If it fails again, it does not stand. Each run prints the interval it ran at, and its steps
   and estimates, for the next PR's G6.
2. **Whether the collapse moves D1's readings** (`step7_masked`: at ×1, ×2 and ×4 h_K2's elements, every corner,
   loading ×4, rule 1 applied):
   - the element as it is; its mask is every element it drives under half its nodes' averaged J at some read;
   - the same run with κ on the mask alone, 2μ to start. An element under half in a masked run has its κ doubled, up
     to λ, or joins the mask at 2μ, and the run is made again: at most four masked runs. If an element is still under
     half after them, or those under half are at λ already, the corner's comparison does not stand;
   - each deciding reading's masked change, masked over as it is, printed at each size. The deciding readings are
     §16x rule 2's, the patch at every corner and the geometric share; the peak push is printed beside them for Jon;
   - the verdict is at ×4 h_K2's elements, the finest size run and the coarsest D1 could need (§16x); ×1 and ×2 show
     the trend. If every deciding reading's masked change is within 5 % (K5's bar), resisting the collapse does not
     move D1's readings there by more than that bar; otherwise it moves them by the largest change;
   - the masked change includes the mask's own change of stiffness, whose sign on the product is not known (κ then
     differs around the nodes on the mask's edge) and which is not measured apart. The mask is cut at half its nodes'
     averaged J; how the change depends on that cut is not measured on the product. On the public case, masks cut
     at 0.7 and 0.8 moved the reaction +4.65 and +5.82 %, against +1.25 % at half (the design review's measurement,
     not in the repo);
   - 2μ to start, the doubling and the four runs are engineering calls; 2μ is within the sources' span, 0.5μ to 25μ.
3. **What follows.**
   - If rule 2 finds every deciding reading's masked change within 5 % at ×4, the element stays as it is, with rule
     1's step control, and the next PR reads D1's size with it; its ×4 is judged by an ×8 wall.
   - If a masked change is beyond 5 %, resisting the collapse moves D1's readings by that much at ×4. The stabilized
     element everywhere is the candidate; at h_K2 its change with 2μ everywhere was +4–8 %, the collapse's part and its
     own together (above). Which element the product runs is Jon's call, with both elements' readings and costs at the
     sizes read in the next PR.
   - If rule 2 is not judged at ×4, the element stays open, and that goes to Jon with the runs' readings.

**Done when:** rule 2's runs are in and read, rule 1 applied to them; the record says which runs were run again by
rule 1; the public case runs in CI (the tests-release job); and the element's state, and what the next PR must do, are
written into §15g's list and the fit plan. The next PR waits on the call (fit plan U20) *(made 2026-09-28: the element
as it is, below)*.

**The code.**
- `sim-soft-explicit`:
  - `ExplicitModel::with_volumetric_stabilization` and `with_element_stabilizations`;
  - `sampled_element_pressure`, shared and in the WGSL, and the executor's node and element passes and energy;
  - `CpuExecutor::top_mode_and_vector`, `TubeRun::stabilization` and `TubeRun::model`;
  - the tube example's eleventh argument;
  - `tests/collapse_release.rs`, release-only and out of coverage.
- The probe:
  - `diagnose` and `locate`, the vector and the collapse;
  - `step7_stabilized` (exploratory) and `step7_masked` (rule 2);
  - every run prints its re-estimate interval and its collapse, and `press` re-runs by rule 1.

**Beside the rules.**
- The step control is not redesigned; rule 1 reads a run's own failure (an engineering call). Tracking the step every
  step from a cheap estimate, as production codes do, needs a bound for this element; the research round's derivation
  puts one 2 to 5 times loose on a model it built (not measured here). Where it would run is not decided: step 4's
  done-when compares every phase with the CPU executor, which has none.
- The GPU (step 4): κ_e is one more per-element value, and each node's λ_a − κ_a is formed at f64 before it is
  narrowed. Phase 4 reads the element's own dilation, which phase 1 writes: on the GPU that is a binding more, or J
  recomputed from the displacements phase 4 already reads.

**Results** (`step7_masked` at `3ee38040`, a pre-squash commit kept locally; loading four times the budget's, every
corner). Each deciding reading's masked change, masked over as it is:

| Reading | ×1 h_K2's elements | ×2 | ×4 |
|---|---|---|---|
| Patch, μ_f 0 | −0.26 % | +6.44 % | not judged |
| Patch, μ_f 0.104 | +0.03 % | +1.80 % | +1.45 % |
| Patch, μ_f 0.18 | +2.04 % | +0.28 % | +0.88 % |
| Geometric share (μ_f 0) | +1.18 % | +0.83 % | not judged |
| Peak push, μ_f 0.104 / 0.18 (for Jon) | +0.75 / +1.54 % | +0.10 / +0.21 % | +0.28 / +0.35 % |

- **Rule 2 at ×4: not judged, by the cut.** In the frictionless run an element was under half its nodes' averaged J
  after each of the four masked runs, κ up to 16μ; after the last its least read 0.495 against the cut at 0.5. So the
  frictionless patch and the geometric share, both read on that run, have no verdict. The masked runs at 8μ and
  16μ read the frictionless patch +5.06 and +5.07 % and the share +0.68 and +0.70 %, those at 2μ and 4μ +3.38 and
  +4.47 %; none cleared the collapse. The two frictional corners stood, every change within 1.5 %; the μ_f 0.18
  corner's least read 0.500.
- **By rule 3, the element stays open, and goes to Jon** with these readings (below).
- The masks held under 3 in 10⁴ of the wall's elements. At ×1 and ×2 every corner cleared within the four runs, with
  κ up to 16μ; at ×4 the frictional corners cleared at 16μ.
- **Rule 1:** at ×2 the frictionless run went non-finite with the loop's re-estimate every 500 steps, at 0.94–0.95 of
  the loading, with the element as it is and with each of its three masks. Each ran again every 50 steps and stood.
  Every other run stood at 500. Why the ×2 frictionless runs fail at 500 is not isolated. Re-estimated every 50 steps,
  a run evaluates the forces about three times per step, against 1.2 at 500 (arithmetic: 100 iterations per estimate),
  after the run that failed; not timed.
- **At 500 on the public cases:** on the test's case the element as it is stops and 2μ everywhere finishes
  (`tests/collapse_release.rs`); on the design review's case with a smaller ball, 2μ everywhere stopped too (above). On
  the product, 2μ everywhere at twice h_K2's elements was not run.
- **Steps,** the element as it is over masked, at μ_f 0 / 0.104 / 0.18: ×1 1.23 / 1.20 / 1.10; ×2 1.90 / 1.34 / 1.00
  (the frictionless pair both at 50); ×4 1.12 / 1.00 / 1.03 (the frictionless mask not accepted). They include the
  mask's own change of stiffness.
- **§16x rule 2's missing cells.** The element as it is, frictionless at ×2 (run again at 50 by rule 1), fills them:
  its patch moves +16.86 % over the first doubling and −2.74 % over the second, and the geometric share +6.00 % and
  +0.55 %; §16x's changes from ×1 to ×4, +13.66 and +6.58 %, are reproduced. §16x's rule 2 verdict stands: ×2's
  doubling moves the patch at μ_f 0.18 by −5.55 %, so D1's size stays open at four times h_K2's elements or finer
  *(2026-09-29, §16z: read against h_K2's replicate's scatter, that doubling cannot tell; D1's size is not picked)*.
- **Resisted, the first doubling moves the patch as much:** the masked patch moves +24.7, +16.6 and +16.8 % from ×1
  to ×2 at μ_f 0, 0.104 and 0.18, against +16.9, +14.5 and +18.8 % as it is; from ×2 to ×4, −2.7 and −5.0 % at the
  frictional corners, against −2.4 and −5.6 %.
- *Not measured:* the mask's own change of stiffness on the product, of unknown sign; how the change depends on the
  cut at half, on the product (on the public case a looser cut moved more, above); why the frictionless ×2 runs fail
  at 500 with the collapse resisted.
- **For Jon's call.** Resisted at its collapsing elements, the element as it is reads (the masked change):
  - at ×4, the frictional patches +1.45 and +0.88 %, and the peak push, which Jon decides at a low friction, +0.28
    and +0.35 %;
  - on the frictionless patch, −0.26 % at ×1 and +6.44 % at ×2, where the comparison stood; at ×4, +5.06 % at 8μ
    and +5.07 % at 16μ, in runs that did not clear the cut.

  The options, each with rule 1's re-runs, which the ×2 frictionless runs needed with the collapse resisted too:
  - keep the element as it is, and read D1's size with it in the next PR;
  - stabilize every element: it first needs its own h_K2, gates and CI. On the product at h_K2, 2μ everywhere left
    an element collapsed; at 8μ, where no element was under half at the read of the smallest step (frictionless, the
    loading only), the readings moved +13 to +22 %. On the tube it moves D1's pushes by amounts that do not all shrink
    with the mesh. Its cost is not measured;
  - before choosing, a fifth masked run at ×4's frictionless corner, at 32μ, past the sources' 25μ: if it clears the
    collapse, it settles whether that corner's change passes 5 %; at 8μ and 16μ the runs the rule did not accept read
    +5.06 and +5.07 %.

  **The call: the element as it is** (fit plan U20). Rule 3's third branch sent it to Jon. He first chose it on a
  summary that gave ×4's frictionless change as "+3.4 to +5.1 %, rising"; told of the correction, he left the call to
  me (2026-09-28), and I keep it.
  - With the collapse resisted (the table above), the μ_f 0.18 patch, the top of U11's interval, moves +2.04, +0.28
    and +0.88 % at ×1, ×2 and ×4; the frictionless patch, its bottom, −0.26 and +6.44 % at ×1 and ×2, and at ×4 it is
    not judged. The element as it is reads the top lower at every size and the bottom lower at ×2, so a *fits* within
    about 2 % of a limit could be a wrong verdict.
  - The stabilized element everywhere at 8μ moved every reading +13 to +22 % at h_K2, and first needs its own h_K2,
    gates and CI.
  - Every masked change includes the mask's own change of stiffness, of unknown sign, and is at the cut at half; on
    the public case a looser cut moved the reading several times more.

  Carried forward: the next PR re-reads rule 2 at the size it picks, at every corner; a ν above 0.49 re-opens U20
  (fit plan U19); the product loop's step control (§15g's list); and the GPU carries the element as it is while U20
  stands (§15g step 4).

*Since the runs,* the probe changed only where no recorded run reached it: a masked comparison whose run as it is did
not stand now masks nothing, and a masked run that does not stand ends its corner's comparison (neither happened);
G6 counts the attempts rule 1 and rule 6 replace (`step7_cost` was not run); a run with its instruments off prints
its collapse as not read; and `diagnose` takes the viscosity's share at rest from its own executor.

*My priors, scored* (written before the first run):
- the vector that sets the step on the most-compressed element at every size: hit at h_K2 and twice its elements,
  miss at four times;
- the elastic part setting it: miss, the vector's damping ratio 1.2–3.9;
- few elements, near the tip: hit;
- a stabilization of 1–2.5μ everywhere clearing the collapse: miss, 2μ left an element collapsed at h_K2;
- K2 on the 50k tube moving under 1 %: at the edge, +1.00 at 2μ;
- the collapse moving the patch more than 5 % at h_K2: miss at ×1, −0.26 to +2.04 % masked, where 2μ everywhere had
  moved it +6.3 to +7.9 %;
- resisted, the patch's first doubling under 5 %: miss, +16.6 to +24.7 %;
- the step at least half the rest step, stabilized, and fewer steps at ×4: hit at h_K2 (0.59); at ×4 the masked runs
  took 0–11 % fewer;
- the ×2 failure at 500 gone with the collapse resisted: miss;
- the peak push within 5 %: hit.

None of my eight design priors (kept locally) named h_K2 belonging to the element, the step control's instrument, or
the mask's sign.

**How the design was checked.**
- **Round 1:** two cold reviewers, of the physics and code, and of the whole plan. The first measured on a public case
  (above).
  - A stabilization everywhere moved the public case's reaction +10.14 %, against +1.26 % on its two collapsing
    elements alone, so the first rule 4, comparing the elements through the sizes, could not answer its question. It
    is now rule 2, the masked comparison.
  - The first rule 1 judged the element by runs that fail with the loop's step control, whatever the element.
  - h_K2 belongs to an element, and the stabilized element needs its own (above). That moved the stabilized element's
    own gates, and D1's size, to the next PR.
  - The CI check of K2 would fail at c = 2: on the 10k tube its corner reads +7.87 % against the test's 7 % (the
    review's measurement).
  - Also: claims with no referent, the plan's own annotations, and #978's follow-ups done by halves.
- **Round 2:** one fresh reviewer of the revision. It found the step's overshoot, an instrument the revision had
  added, reading at most 1 on the public runs that failed and above 1 on runs whose readings matched the 50-step run's,
  so it was cut, and rule 1 reads the run's own failure. It found that the mask's own change of stiffness has no known
  sign, that no route followed a corner that does not stand, and several claims with no referent. Its findings are
  kept in the local archive, not in the repo.

**How the build was checked.**
- **Round 1:** three cold reviewers: of the code (19 mutations, in a worktree of its own), of this record against the
  runs' outputs and the repo for the scan's figures, and of the whole plan.
  - The summary sent to Jon left out the one accepted frictionless comparison past the bar (+6.44 % at ×2), and costed
    the options unequally: rule 1's re-runs apply to either element, 2μ everywhere left the collapse at h_K2, and the
    fifth run is a measurement, not an element.
  - Rule 1 read an inversion at 500 as a re-run, which changes how K4 is read; §15a now says so.
  - An attribution of the failures at 500 to the step control was cut: on the public cases they depend on κ and on
    the case.
  - Three mutations survived: the tube's model dropping its stabilization, its run bypassing that model, and the
    damping quotient taken of the next iterate. Each now fails a test.
  - The cells §16x rule 2 left open, and the prior on the masked patch's first doubling, were read from the runs.
  - The privacy sweep, with planted figures as its positive controls, found no product figure in any added line or
    commit message.
- **Round 2:** one fresh reviewer of round 1's fixes found eight problems, all eight written by those fixes: the
  options costed unequally again, K4's note wider than §15a allows, "cleared" read from one read, the fifth run
  promising more than it can, and an attribution the archive could not back. They were cut, and the rounds stopped
  there: the results and the verdict did not move in either round.
- **Macro review** (Jon's request, one fresh reviewer of the whole diff at `b3613c25`): every figure it checked
  matched the runs' outputs. It found that the summary Jon decided on gave ×4's frictionless change as "+3.4 to
  +5.1 %, rising", where the rule rejected it by 0.005 on its cut and the last two runs read +5.06 and +5.07 %; that
  the cut's effect had been measured on the public case; that "known bias" sat inside Jon's decision; and that ν, the
  product loop's step control and the GPU's κ were not carried forward. Jon left the call to me; the record above now
  states it, and those items are carried.


### 16z. D1's element size, with the element as it is (design, 2026-09-28)

Jon's call after §16x: settle the element collapsing at the seated tip, then read D1's element size again; his GPU call
comes after. §16y settled the element (the element as it is, fit plan U20) and carried forward what this PR does: judge
four times h_K2's elements with an eight-times wall, read the collapse's change again at the size picked, at every
corner, and time §16y rule 1's re-runs. This is the design, set before the runs and revised after its review (below);
the results are added after.

**Scope.**
- §16x rule 2 read again with an eight-times wall, so that four times h_K2's elements is judged. Under the mount it is
  also the discretization's check on the product's own confinement (§16x rule 2), the mount's side of §9 decision 11's
  gate, open until it settles (§15g step 7's note).
- §16y rule 2, the masked comparison, read again at every corner at the size used, with U20's reading and the two sides
  of U11's verdict it bears on.
- §16x rule 1's check of the loading, and G6, at the size used. G6 runs under two step controls, which times §16y rule
  1's re-runs and costs one of the options §15g's list sets for the product loop before step 4.
- Not here: ν under the mount (fit plan U19, Jon's; everything here is at ν 0.49); choosing the product loop's step
  control; the stabilized element's own h_K2, gates and cost; other insets, D1's limits and a verdict.

**The runs.** As §16x's and §16y's: step 7's wall mounted at its closed end, ν 0.49, Ecoflex 00-30's η/μ with the tube's
mass damping, f32, the pairing's corners μ_f 0, 0.104 and 0.18, the loading four times the budget's (§16x rule 1), the
element as it is, and §16y rule 1: a run that fails with the loop's re-estimate every 500 steps is run again every 50.

**The rules, set before the runs.** Engineering calls unless one names Jon. They are named, since §16x's and §16y's
rules carry the same numbers.
- **The size rule** (§16x rule 2, with one wall more). Walls at one, two, four and eight times h_K2's element count,
  each meshed as step 7's is at the lattice the secant finds, the counts' ratios printed, every wall built and checked
  before the first run; and a replicate on a lattice shifted half a cell at h_K2, as §16x's, and at four times.
  - The deciding readings, the bar and the peak push beside them for Jon are §16x rule 2's.
  - Each doubling is read against the scatter of the replicate at its coarser size, or the nearest coarser (h_K2's for
    the first two doublings). A deciding reading fails if it moves past 5 % by more than that scatter, passes if it
    stays within 5 % by that scatter, and otherwise cannot tell. A doubling fails if a reading fails; otherwise it is
    not judged if a run behind a reading did not stand, cannot tell if a reading cannot, and passes if all pass. *(§16x
    read a doubling at 5 % exactly, the scatter deciding only whether the rule could tell at all; read against the
    scatter, its −5.55 % on the μ_f 0.18 patch at twice h_K2's elements cannot tell against the replicate's 2.73 % on
    that reading.)*
  - D1's readings need the coarsest size from which every doubling passes *(§16x's probe took the coarsest whose own
    doubling passed; the probe now reads "from which")*. If none does and the finest doubling fails, the size is open,
    eight times or finer; if the finest doubling after the last that failed has no verdict, no size is picked, and the
    rule says why.
  - `r = r∞ + C hᵖ` is fitted over the finest three sizes, and its remaining error at four and eight times printed. No
    size is extrapolated from it *(replacing §16x rule 2's "D4 at the size the fit extrapolates to": on the design review's synthetic
    readings, that size went from eight times to none over half a point of the finest doubling)*.
  - **The size used** by the masked rule, the loading check and G6 is the size picked, or else the finest whose runs
    all stood, labelled as not picked.
  - The executor is deterministic (four times h_K2's elements at μ_f 0.18 read the same at #978's and #979's commits),
    so the runs at one, two and four times must reproduce §16x's and §16y's readings; one that does not stops the
    record.
- **The interval rule.** A doubling's two runs, a replicate and its size's run, a masked run and its run as it is, and
  the loading check's two runs are read at one re-estimate interval: where §16y rule 1 ran one every 50 steps and the
  other stood at 500, the other is run again every 50 steps, and the change that makes to a run standing at both is
  printed. If it moves a deciding reading by more than K3's 0.5 %, D1's readings depend on the step control, which then
  goes to Jon beside D1's size. On the design review's public cases (the ball-on-block and the 10k tube, not in the
  repo) it moved the seated readings at most 0.001 % and the pushes 0.25 %. The fit reads each size's own run, and says
  so when its sizes stood at different intervals.
- **The masked rule** (§16y rule 2) at the size used, at every corner:
  - an element still under half after a masked run has its κ doubled, or joins the mask at 2μ, up to 25μ, the top of
    the sources' span (§16y), or λ if lower, over up to five masked runs at ν 0.49, enough to take 2μ to 25μ. §16y
    stopped at four, at 16μ, its four-times frictionless run leaving an element at 0.495 of its nodes against the cut
    at 0.5;
  - the masked runs start at the interval the run as it is was kept at (the interval rule);
  - every round's change over as it is is printed, and the κ at which each corner cleared. On the design review's
    public cases the change kept rising with κ after the collapse had cleared, on one with no plateau up to λ (+5.12 %
    at 2μ, +13.83 % at λ; not in the repo);
  - U20 stands if every deciding reading's masked change is within 5 % (K5's bar). It goes back to Jon if one exceeds
    5 %, or if a corner is not judged, with the note that the stabilized element's own h_K2, gates and cost are not
    measured here;
  - U11's two sides (fit plan: *fits* needs the corners' top under the limit, *too tight* their bottom over it): the
    corners' top and bottom, resisted over as it is. Read lower as it is, a *fits* within that change of a limit could
    be false and a *too tight* missed; read higher, a *fits* could be missed and a *too tight* false. The push's sides
    are printed beside, for Jon, from the geometric share at μ_f 0 (D1's push there) and the peak push with friction;
  - the masked change keeps §16y's limits: it includes the mask's own change of stiffness, whose sign on the product
    is not known, and is taken at the cut at half.
- **The loading check** (§16x rule 1's) at the size used: the loading and twice it at μ_f 0 and 0.18. If a reading
  moves more than 5 %, the loading is open at that size, and the size rule and the masked rule hold at four times the
  budget's loading only.
- **G6** (§16x's, `step7_cost`) at the size used, at 4 threads on an idle machine with the probe's instruments off,
  under two step controls: the loop's re-estimate every 500 steps with §16y rule 1's re-run, the attempts it replaced
  counted with their share of the time and the kept run's time per step over theirs; and a fixed re-estimate every 50
  steps. Beside them, ν's and the viscosity's factors on the steps (§16x).
- **K4.** An element inverting in a run whose validity gates hold, with no re-run left to §16y rule 1 (every 50 steps),
  fails K4, one of §15g step 2's stop criteria, and §15a sends it to the element or the loading time. Each stage names
  such runs of the element as it is.

**For Jon,** for his GPU call: D1's size or why there is none, with the fit's remaining error there; G6 there under both
step controls, at ν 0.49 with ν's and the viscosity's factors; the masked comparison, U20's reading and U11's sides
there; the loading check; K4; and the interval rule's reading. Still ahead of steps 3–5 under his rule (§15g step 2's
note), from §15g's list: ν (U19), the product loop's step control, the mount's side of the confined case unless the
size rule settles it, and how the peak push is read at a low friction (fit plan D1, his).

**Done when:** the rules' runs are in and read; the record names the runs §16y rule 1 and the interval rule made again;
§15g's list and step 7's note, §9 (these rules' engineering calls, dated) and the fit plan (D1, G6, U20) carry the
outcome; and the probe's new logic is pinned by tests, each failing under a mutation of the code it guards.

**The code.** The probe (`insertion_sim::step7_first_run`): `SIZES` gains eight times and `REPLICATES` four; `Cell`,
`Mounted`, `at_one_interval`, `needs_one_interval` and `interval_line` (the interval rule); `outcome`, `size_verdict`,
`scatter_for` and `Fit` (the size rule); `step7_masked` to 25μ, with `next_stiffening`, `masked_rounds`, `sides_of` and
`side`; `step7_cost` under two step controls; `k4_fails` and `k4_line`; fourteen unit tests.


**Results** (`step7_sizes` at `ed24f709`; `step7_masked` and `step7_cost` at eight times h_K2's elements, at
`cf601cb0`; both pre-squash commits kept locally; loading four times the budget's, every corner). Since `ed24f709` the
probe changed only its summary lines (design round 2's fixes); every run's own lines are the same. After the runs,
build round 1 added K4's reading of a run that stopped, G6's standing and K4 line, and the pure functions the tests
pin; no run was made with them.

- **Reproduction:** the runs at one, two and four times h_K2's elements, h_K2's replicate, and twice h_K2's
  frictionless run every 50 steps read exactly §16x's and §16y's readings.
- **The size rule: no size.** The eight-times wall is meshed at 0.506 of h_K2, the counts 1.94, 3.85 and 7.55 times
  h_K2's. Each deciding reading's change per doubling, with how the rule reads it against the scatter beside it:

  | Reading | ×1 → ×2 | ×2 → ×4 | ×4 → ×8 | Replicate at h_K2 | Replicate at ×4 |
  |---|---|---|---|---|---|
  | Patch, μ_f 0 | +16.86 %, fails | −2.74 %, cannot tell | +5.06 %, cannot tell | −2.67 % | −0.71 % |
  | Patch, μ_f 0.104 | +14.53 %, fails | −2.35 %, cannot tell | +2.45 %, passes | −3.01 % | −0.14 % |
  | Patch, μ_f 0.18 | +18.84 %, fails | −5.55 %, cannot tell | +1.68 %, passes | −2.73 % | −0.05 % |
  | Geometric share | +5.90 %, fails | +0.57 %, passes | −0.90 %, passes | +0.20 % | −0.43 % |
  | Peak push, μ_f 0.104 / 0.18 (for Jon) | +8.36 / +8.41 % | +2.19 / +1.46 % | −1.12 / −0.96 % | +0.27 / +0.28 % | −0.57 / −0.50 % |

  - No size is picked. The first doubling fails on every reading, so D1's size is twice h_K2's elements or finer; the
    second cannot tell against h_K2's scatter, and the third cannot tell on the frictionless patch, which moves
    +5.06 % against its replicate's 0.71 %. Over the third, every other deciding reading, and the peak push, moves at
    most 2.45 %. Eight times is the size used, as not picked.
  - The second doubling is read against h_K2's scatter, standing in for twice's, which was not measured. Against a
    scatter under 0.55 % the −5.55 % fails, and the size would be four times or finer (arithmetic on the rule); four
    times' replicate moves the patches 0.05–0.71 %.
  - Read at 5 % flat, as §16x read a doubling, the second and third doublings fail (−5.55 % and +5.06 %), and the size
    would be open, eight times or finer.
  - The replicate at four times moves the patches 0.05–0.71 %, against 2.67–3.01 % at h_K2.
  - No fit: every deciding reading changes direction over the finest three sizes, so no order and no remaining error
    are read.
  - The share's first two doublings read +5.90 and +0.57 %, where §16y read +6.00 and +0.55 %: here both runs are
    every 50 steps (the interval rule), and h_K2's and four times' frictionless runs moved their 10 mm push +0.099 and
    +0.020 % between intervals.
  - Under the mount, the size rule is the check on the product's own confinement, so the mount's side of §9 decision
    11's gate stays open.
- **The interval rule:** h_K2's and four times' frictionless runs were run again every 50 steps to match twice h_K2's
  (which failed at 500 and stood at 50, §16y rule 1). Between the intervals the patch moved 0.000 % and the geometric
  share at most 0.099 %, under K3's 0.5 %, so here D1's readings do not depend on the step control; the peak push, not
  a deciding reading, moved at most 0.40 %. At eight times no run needed the rule, as every run stood at 500; there
  G6's two step controls read the deciding readings within 0.01 % of each other.
- **The loading check at eight times:** twice the loading moves the peak push at μ_f 0.18 +0.27 %, the geometric share
  −2.60 %, and the patches at 0 and 0.18 −0.01 and +0.22 %: the loading holds there.
- **The masked rule at eight times, the size used and not picked: U20 stands there.** Each deciding reading's masked
  change, masked over as it is:

  | Reading | Masked change | The collapse cleared at |
  |---|---|---|
  | Patch, μ_f 0 | +4.11 % | 16μ, the fifth run |
  | Patch, μ_f 0.104 | +1.56 % | 8μ, the third |
  | Patch, μ_f 0.18 | +0.31 % | 8μ, the third |
  | Geometric share (μ_f 0) | +0.61 % | 16μ |
  | Peak push, μ_f 0.104 / 0.18 (for Jon) | +0.70 / +0.63 % | 8μ |

  - Every corner cleared; the masks held about 1 in 10⁴ of the wall's elements. Every deciding reading's masked change
    is within 5 %, so U20 stands at eight times. At twice h_K2's elements, which the size rule does not exclude, §16y's
    masked change on the frictionless patch was +6.44 %, and at four times it was not judged; U20 is read again at the
    size picked.
  - The frictionless corner cleared on the last run the rule allows, its least element at 0.510 of its nodes against
    the cut at 0.5; no element reached more than 16μ. Over its five runs the
    patch's change read +3.23, +4.32, +4.00, +4.49 and +4.11 %, and the least element 0.259, 0.330, 0.478, 0.416 and
    0.510: neither rose steadily over the runs.
  - **U11's two sides,** resisted over as it is: the corners' top, the patch at μ_f 0.18, +0.31 %: as it is reads it
    lower, so a *fits* within 0.31 % of a limit could be false; their bottom, the patch at μ_f 0, +4.11 %: as it is
    reads it lower, so a *too tight* within 4.11 % of a limit could be missed. The push's (for Jon): its top +0.63 %
    and its bottom, the geometric share, +0.61 %, each lower as it is. Each includes the mask's own change of
    stiffness, of unknown sign on the product.
  - Every run stood at 500.
- **K4** held in every kept run of the element as it is, in the three stages; every kept run stood.
- **The tests:** each of the fourteen fails under a mutation of the helper it guards (the build review's and mine after
  its fixes). The re-runs' plumbing (`at_fifty`, `Cell::press`) and each stage's own inline logic cannot fail a unit
  test; the runs exercised them as they stood at `ed24f709` and `cf601cb0`.
- **G6 at eight times** (`step7_cost`: 4 threads, an idle machine, the probe's instruments off; a press is §16x rule
  11's three runs):

  | Step control | A press over D4, on the CPU | Full verdicts at 1 and 2 insets, over their 15 min | The same steps at K1's per-step budget, over D4 |
  |---|---|---|---|
  | Every 500 steps, with §16y rule 1's re-run | 11.8 | 3.9 and 7.9 | 53 |
  | Every 50 steps | 27.5 | 9.2 and 18.4 | 53 |

  - No run at eight times needed §16y rule 1's re-run, so there the retry cost nothing; re-estimating every 50 steps
    took the same steps, within 0.2 %, and 2.34 times the time.
  - A press cost 3.1 times four times h_K2's (3.8 of D4, §16x) for 1.96 times the elements (arithmetic).
  - The rest step's factor on the steps is 1.07 and 1.23 at ν 0.495 and 0.4975, and 0.80–1.39 across the viscosity's
    range (Ecoflex's η/μ × 0.74 to × 1.47); no ν or viscosity run was made at eight times. With ν a corner, a press is
    twice the runs (§16x), about 24–26 of D4 at the loop's interval, and up to about 37 at the viscosity's high end
    (arithmetic, taking the steps as that factor and the time as the steps).
  - With the loop's interval, the CPU as timed runs 4.5 times faster than a device at K1's per-step budget. Meeting D4
    at eight times h_K2's elements takes a device at least 11.8 times this CPU (arithmetic). No GPU step has been
    timed.
- **For Jon's call:**
  - **D1's element size is not picked.** The size rule excludes h_K2 alone, taking h_K2's scatter for twice's; with a
    scatter at twice as small as four times', it would be four times or finer; read at 5 % flat, eight times or finer.
    From four to eight times every deciding reading passes but the frictionless patch, which moves +5.06 % against its
    replicate's 0.71 %. That patch is the bottom of U11's interval, on which a *too tight* rests. The
    frictional patches move +2.45 and +1.68 %, the geometric share −0.90 %, and the peak push, his to read at a low
    friction, −1.12 and −0.96 %. No fit says how much is left: every deciding reading changed direction over the finest
    three sizes.
  - At eight times, the size used and not picked: the loading holds; G6's two step controls read the deciding readings
    within 0.01 % of each other; K4 holds; and the collapse's masked change is at most +4.11 %, so U20 stands there.
    So U20, and with it a GPU carrying κ = 0 only (§15g step 4), stands at eight times; at twice h_K2's elements §16y's
    masked change was +6.44 %, past its bar, and at four times it was not judged.
  - G6 there: 11.8 of D4 on the CPU with the loop's interval, 27.5 re-estimating every 50 steps; the bake 0.29 of D4
    once per scan and band. At four times it was 3.8 (§16x); twice was not timed as G6. A press at sixteen times would
    take about 37 of D4 on this CPU, by the last doubling's growth (arithmetic).
  - Still ahead of steps 3–5 under his rule (§15g step 2's note): D1's size and, with it, the mount's side of the
    confined case, both the size rule's (judging eight times takes a sixteen-times wall); ν (U19, his); and how the
    peak push is read at a low friction (fit plan D1, his). Before step 4: the product loop's step control, mine to
    choose from G6 (here the re-run cost nothing, and a fixed 50 steps 2.34 times the time).

*My priors, scored* (written before the first run, kept locally):
- the runs at one to four times h_K2's elements reproducing §16x's and §16y's: hit;
- four times passing its doubling, so D1's size four times: miss, the frictionless patch +5.06 %, and the frictional
  patches moving −2 to −3 %: miss, +2.45 and +1.68 %;
- eight times' runs standing at 500: hit;
- the interval's own effect under 0.5 % on the patch: hit, 0.000 %;
- the replicate at four times moving less than h_K2's: hit, 0.05–0.71 % against 2.67–3.01 %;
- the masked frictionless corner at four times clearing by 32μ or λ, its change +4.8 to +5.3 %: not run at four times;
  at eight times it cleared by 16μ, at +4.11 %;
- the loading check at four times holding, the share moving most: run at eight times, where it held and the share
  moved most, −2.60 %;
- G6 at four times reproducing 3.8 of D4: not run; a press at eight times about 12 of D4: hit, 11.8;
- a fit's order between 1 and 2: miss, no fit.

None of my ten suspected design defects named reading a doubling against its scatter, K4 at the last interval, the
mount's side, or costing a fixed 50 steps.

**How the design was checked.** Round 1: two cold reviewers, of the physics and code, and of the whole plan; the first
measured on the public cases with a harness outside the repo, since deleted. About 19 findings; the largest:
- a doubling was passed or failed at 5 % inside the scatter its replicate measures; each is now read against it;
- on the public cases the masked change kept rising with κ after the collapse cleared, and the extension to λ went
  past the sources' span with nothing depending on it; κ now stops at 25μ, every round is printed, and a corner not
  judged sends U20 back to Jon;
- the fit's extrapolated size moved with noise, and was cut;
- a K4 failure every 50 steps read as "not judged"; the mount's side of the confined case, the fixed-50 step control's
  cost, the push's bottom side (the geometric share, not the 1 mm peak) and the rules' clashing numbers were missing
  or wrong; a size verdict was printed where the rule could not tell;
- a stored spec and three prints did not match their runs, `sides` had no test, and a replicate wall was built only
  after every run.

Round 2: one fresh reviewer of round 1's fixes found nine problems, eight of them written by those fixes, among them:
the interval rule's reading left out the loading check's and the masked rule's runs; a sentence gave the mask's own
stiffening a sign the next called unknown; the fit moved by the scatter came out undefined where a size passes; the K4
line was wider than its check, and counted the masked model's runs; and new logic had no test. They were fixed in the
code or cut, and the design rounds stopped there.

**How the build was checked.** Round 1: three cold reviewers, of the code (42 mutations in a worktree of its own), of
this record against the runs' outputs and the repo for the scan's figures (about 130 numbers checked; a sweep of the
runs' local figures, with planted ones as its positive controls, found none in any added line or commit message), and
of the whole plan. The largest findings:
- "U20 stands" was written without its condition, eight times being the size used and not picked, while the size rule
  does not exclude twice h_K2's elements, where §16y's masked change was past the bar;
- "twice h_K2's elements or finer" stood alone, resting on h_K2's scatter in place of twice's, which was not measured;
  the replicate at four times reads its own far smaller;
- the step control's independence at eight times was credited to the interval rule, which made no runs there (its
  line read 0.000 % over no runs); G6's paired runs are the evidence;
- about half the new logic had no test, the deciding readings among it; the helpers now have tests, and the rest was
  exercised only by the runs;
- the rest step's factor was reported as steps, older cost statements read as current, and three prints could say
  more than their checks (G6's standing, the loading check's order, K4 in a run that stopped).

Round 2: one fresh reviewer of round 1's fixes found seven problems, five of them written by those fixes: the masked
rule's κ described two ways, a "not read" line saying more than its check, the record crediting the runs with logic
changed after them, a figure finer than its prints, and a count no test pinned. They were fixed or cut, and the rounds
stopped there; none moved a verdict.

## 17. The GPU steps

Jon, 2026-09-29, after §16z: *"let's do it on metal first, on this laptop"*. The GPU executor is built now, §15g steps
3–5, first on Metal on the M4 Pro, D4's machine (fit plan D4).
- **D4.** At eight times h_K2's elements, the size used and not picked, a press takes 11.8 of D4 on this CPU (§16z), so a
  device must be at least 11.8 times as fast there. No step yet measures a device against that: at K1's per-step budget,
  step 5's speed gate, a press there would take 53 of D4 (§16z). The revision of the speed plan that §15g's rule asks for
  before GPU work (step 2) is still owed; when it is due is Jon's, raised with him on 2026-09-29. The quality gates are
  not loosened.
- **The order.** This reverses Jon's order of 2026-09-26 (§15g step 2's note), which put the quality items first. Those
  still open stay open, to run afterwards or on the GPU: the items §16z's "For Jon's call" lists, and fit plan U15, the
  damping's form. The product loop's step control, mine, still comes before step 4 (§16z); on the GPU each re-estimate's
  power iteration reads one scalar per iteration (§16e), and each read is a submit (§17a). *(2026-09-30, §17b: the step
  control is set, and an estimate is one read.)*

### 17a. Step 3: recording and reading, shared (2026-09-29)

**What step 4's executor needs,** read from the loop and the trait in `sim/L0/soft-explicit/src`:
- It is driven one phase at a time and runs to a time, not a count (`stepping.rs:179-206`, `:233-244`). Reads cut into
  the run: the monitors every 100 steps (§16e), the re-estimate, snapshots and phase outputs; so do host writes,
  `set_state` and `set_poses` (`executor.rs:341-408`). The GPU executor records phases as compute passes and reads back
  only on an explicit read (§14d).
- Each step's own values: its phases take the step's time, dt and damping (`stepping.rs:194-196`). What else a step
  carries, the obstacle's pose interpolated on the device (§14b) from samples streamed a batch at a time (§14d) and the
  work's move and turn from the f64 track (§15g step 4's note), is step 4's design. *(2026-09-30, §17b: the pose is
  interpolated on the host and carried in the step's values.)* This assumes a step's values are
  known on the host when the step is recorded; tracking the step on the device (not costed, §16z) would need another
  design.

**Two facts about wgpu 27 that it rests on.**
- A `queue.write_buffer` runs "just before the explicitly submitted commands" of the next submit (wgpu 27.0.1,
  `src/api/queue.rs:114-116`), so a write issued while work is recorded but not submitted overtakes all of it. Two tests
  assert it: `values_written_between_encodes_collapse_to_the_last` and `a_bare_write_overtakes_the_steps_before_it`
  (`sim/L0/gpu/src/submit/tests.rs`).
- On Metal, one command buffer of 2 048 compute passes blocks for good inside `CommandEncoder::finish`, and 2 047
  complete (measured on the M4 Pro, a debug build, one dispatch a pass). wgpu-core opens two Metal command buffers for
  each compute pass (`wgpu-core-27.0.3/src/command/compute.rs:543-547`, `:770-799`), wgpu-hal gives the queue 4 096
  (`wgpu-hal-27.0.4/src/metal/adapter.rs:30`, `:54`), and none is committed before the submit. One more is outstanding
  than two a pass account for; which one is not isolated. The rigid pipeline's hang was this, not the readback §11
  names: T38 with its chunk bound removed blocked in `finish`, at a Metal command-buffer allocation (its stack sampled).
  T38's model opens 24 passes a substep, and the collision stage opens one per dispatch (`pipeline/collision.rs:476`),
  so a bound in substeps is not one in passes.

**What was built** (`sim/L0/gpu/src/submit.rs`).
- `Recorder` holds the pending encoder and opens every compute pass (`Recording::pass`), counting them; a pass opened on
  the encoder directly escapes the count. A submit holds at most `PASS_CAP`, 1 024 passes. With a run of other commands
  between each two passes, which opens one more Metal command buffer (`command/mod.rs:1352`, `:635-643`), that is at
  most about 3 072 of the queue's 4 096.
- The recorder submits before a step and at its end once `STEP_PASS_CAP`, 512, are pending. A step carrying values
  cannot split across submits, since its values sit in one submit's ring: it may open 512 and stops with a message past
  that. A step carrying none is also submitted inside itself, at the cap.
- Each step's values go into a uniform ring, one slot a step at `min_uniform_buffer_offset_alignment`, staged on the
  host, written at the submit, and bound by a dynamic offset, as `fk.rs` binds its per-level parameters
  (`pipeline/fk.rs:75`, `:302`). It needs no device feature, and the context requests none (`context.rs:39`). Push
  constants would serve on Metal and Vulkan too (a native feature in wgpu 27, renamed immediates from 28); they are not
  used, so the device stays featureless.
- `write` submits what was recorded before it. `read` and `read_many` copy the buffers named into staging, submit once,
  map each and wait once, with no timeout: the wait covers everything submitted before it, whose length the caller
  sets. A read comes in 4-byte units, as wgpu copies. A failed wait or map stops with its cause, since the trait's reads
  return values, not errors (`executor.rs:401-408`). Reads, writes and submits inside a step stop with a message.
- The rigid pipeline records through it. Its stages open passes through `Recording` (a bare encoder in tests), and
  `step()` records one step a substep, with no values, and reads qpos and qvel in one read; its uploads and uniform
  writes go straight to the queue before it creates its recorder, so they land at the recorder's first submit. A
  substep past the cap now runs, split across submits; at `df00dea8` a model of about 524 passes a substep ran calls of
  up to three substeps and hung from `step(4)`, the first to pass 2 047 in one chunk (measured in the build's review).
  `SUBSTEP_CHUNK`, the staging buffers, `map_staging_f32` and `fk.rs`'s readback
  helpers are gone; tests read through `submit::read_buffer`, and read counts as u32 rather than through f32 bits.
- The test policy: eleven pipeline tests (twelve places; T28 has two) returned on `NoGpu` without consulting
  `test_support`. `NoGpu` now carries the context's `GpuError`, and they go through the policy (`pipeline_or_skip`). The
  context's test asserts the backend, Metal on macOS and Vulkan elsewhere, and that the device got the adapter's buffer
  limits.

**Not here.**
- **The contact-list tools** (the atomic append and the CAS float-add; §11, §14a). Step 4 need not use them, and today
  they are WGSL written out in each rigid shader that uses them (the append in `shaders/sdf_sdf_narrow.wgsl:340-351` and
  `shaders/sdf_plane_narrow.wgsl:240-251`; the CAS add in `crba.wgsl`, `rne.wgsl` and `newton_solve.wgsl`). They are
  extracted when soft-on-soft contact is built, after step 8's design (§9 decision 9), where they have a user (Jon
  agreed, 2026-09-29).
- **A rigid pipeline built on a caller's device,** for §14a's rigid–soft exchange on one device: no user before that
  exchange. The pipeline creates its own (`pipeline/orchestrator.rs:137`).
- **For step 4:** its host writes go through the recorder (the queue stays public for the rigid pipeline's writes
  before recording); more storage buffers per stage than the context's 16, wgpu features, and lavapipe's granted binding
  size are its to name (§15g step 4's note).
- **wgpu stays at 27** (§15g step 3).

**The gates, each made to fail once, at the code of `1fcd5289` (T41 at `217a2e8d`)** (the recorder's in `sim/L0/gpu/src/submit/tests.rs`,
the pipeline's in `sim/L0/gpu/src/pipeline/tests.rs`; each thread-bound test fails after a minute rather than hang):

| Gate | Made to fail by |
|---|---|
| `each_step_reads_its_own_values`: 40 steps through a ring of 8 slots, so it fills and its slots are reused, and a read after step 21; the cells start unwritten | every step's values staged into one slot |
| `a_write_waits_for_the_steps_before_it` | a write that does not submit first |
| `a_submit_at_the_cap_with_a_clear_between_passes_completes`: 1 024 passes, each after a clear | the cap raised to 1 400, which blocked: with the clears, about 4 200 Metal command buffers |
| `a_step_without_values_past_the_cap_submits_itself`: one step of 2 148 passes, each counted | no submit at the cap, which blocked |
| `a_step_carrying_values_past_its_pass_cap_stops_with_a_message` | the submit cap in place of the step cap |
| `a_step_after_passes_outside_steps_keeps_its_budget`: 600 passes outside steps, then a step of 512 | no submit at the step's start |
| `a_read_not_in_four_byte_units_stops_with_a_message` | no check on the read's size |
| `reads_writes_and_submits_inside_a_step_stop` | a read allowed inside a step |
| T38: one `step(150)` byte-identical to 150 `step(1)`, within a minute | a read that drops the pending submit; unbounded, it blocked (above) |
| T40: `step(0)` reads the state back, rounded to f32 | an early return at zero substeps |
| T41: a free sphere beside 1 024 static ones, two passes an SDF pair, so a substep passes 2 047; one `step(2)` byte-identical to two `step(1)`, within a minute | `step()` on a recorder with values, which stopped past 512; no submit at the cap, which blocked |
| the test policy | `GpuContext::new` finding no adapter under `CF_REQUIRE_GPU=1`: 3 tests pass and 63 fail; at `df00dea8` the same change let 14 of 55 pass, 11 of them pipeline tests that never ran |
| the context's test | asking for wgpu's default buffer size: 268 435 456 bytes granted against the adapter's 14 302 248 960 |

Not made to fail: a failed wait or map, since no way was found to make one fail on demand.

Measured once, not a test: sixteen submits of 512 passes back to back, with more GPU work in each than recording it
takes, completed on Metal: recorded in 2.2 s and done in 3.1 s, where recording the same passes with no work takes 0.3 s,
so the recording waited on the GPU and did not block. What it waited on is not isolated. It was a test until the build's
review, and was cut because nothing could make it fail.

**Before and after** (a one-off; the probe and its outputs are local). Four rollouts, run ten times at `df00dea8` and ten
times at `1fcd5289`: T38's free fall, a sphere resting on a plane, implicit damping, and four environments on a plane.
Free fall and damping repeat bit for bit and are bit-for-bit identical before and after. The two with contact do not
repeat run to run: x, y and the velocities differ by up to 2.3e-18 between runs of the same code, with height and time
exact, and before to after by at most 2.1e-18. What differs between their runs is not isolated.

**Done when:** `sim-gpu`'s suite passes on Metal on the M4 Pro with `CF_REQUIRE_GPU=1`, a local run, since no CI job
runs `sim-gpu` on Metal (the macOS job tests other crates, `.github/workflows/quality-gate.yml:942-982`); it passes on
lavapipe in `tests-debug` shard 3 (`quality-gate.yml:589-637`); `sim-gpu` grades A; and `sim-gpu-benches`, the one crate
that uses it, passes.

**Results** (the code at `217a2e8d`):
- On Metal, `CF_REQUIRE_GPU=1 cargo test -p sim-gpu`: 67 of 67 pass, on the Apple M4 Pro.
- `cargo xtask grade sim-gpu`: A on every automated criterion, coverage 97.9 %. A run before the build's review was F on
  Clippy, for an `#[allow(clippy::panic)]` with no `//` justification, which the pre-commit hook's clippy does not check.
- `sim-gpu-benches`: its test passes.
- lavapipe, in CI's `tests-debug` shard 3: at the push.

**Before the change** (`df00dea8`): 55 of 55 tests pass on Metal, the adapter reporting Apple M4 Pro on Metal, in
4.0 s; `cargo xtask grade sim-gpu` is A on every automated criterion, with coverage 97.9 %. (§14e had left whether
`sim-gpu` grades A unchecked.)

**How the design was checked.** Round 1: a reviewer of the code, who measured on Metal with a probe outside the repo
(since deleted), and a reviewer of the whole plan. Their findings that changed the design:
- the loop never hands the executor a total, and reads and host writes cut into it, so a split of a known count became
  a recorder that submits at reads and before writes;
- what hangs is compute passes, blocking in `finish`, not the readback (measured by the reviewer, then again here, and
  traced to the Metal queue's count), so the cap counts passes;
- the obstacle's pose was said to come from the host's f64 track, which only the work's move and turn do;
- the contact-list tools were sent to step 8, which is a design step;
- D4's rule on GPU work, the step control's place before step 4, and the open items' owners were missing from §17;
- push constants were ruled out on a removed field, not a removed capability;
- no gate made T38's no-hang half fail, and none compared the rigid pipeline with itself before the change.

Round 2: one fresh reviewer of the revision found nine problems, seven of them written by the revision: §17 called the
GPU work D4's revision though nothing could fail; the write rule held only for writes through the recorder; the ring's
wrap, a failed wait and the before-and-after comparison had no failing arm; the cap was argued one submit at a time,
though submits run back to back; §11 still named the readback; a step's pass budget depended on what was pending; and the
no-timeout reason did not hold for capped submits. Two were older: "twelve tests" counted places, and the fit plan had
no note. They were fixed in the code and its tests, measured, or cut, and the design rounds stopped there. The eleven
tests were found by a mutation run before this round reported them.

The build's review, round 1: three cold reviewers, of the recorder, of the rigid pipeline's migration, and of the whole
plan; the first two measured with probes outside the repo (since deleted). All three found the same regression: the step
cap stopped rigid substeps past 512 passes, which had run. The fix splits steps where no values can split rather than
raising the cap. The rest: passes opened outside steps shrank a step's budget; reads not in 4-byte units failed with
wgpu's message; T38 had no time bound, and the time bound reported a panic as a timeout; the stages' dispatch wrappers
took a recorder but wrote straight to the queue; the cap's margin with clears between passes was argued, not tested; one
gate could not fail; the speed plan's revision had no owner; and stale text in the spec, the benches, §11, §14a and §15g.
All were fixed or cut, and every gate was made to fail again at the fixed code. Two of these reopened what round 2 had
recorded as fixed: a step's budget still depended on passes opened outside steps, which no test covered, and the
back-to-back test that answered round 2 could not fail.

A fix-diff pass by one fresh reviewer found five problems, three written by those fixes, all in prose ("instead" where
every recorder also submits at step boundaries, "under" where a submit holds exactly the cap, two `# Panics` sections
missing the 4-byte case, and this paragraph not saying what was reopened), and two older: the regression's fix had no
gate in the pipeline, since `step()` on a recorder with values passed every test (T41 now fails it), and one bench
sentence still said one submit. They were fixed, and the build's review stopped there; the prose of those last fixes
has not been reviewed.

### 17b. Step 4: the soft executor on the GPU (design, 2026-09-29)

**The step control** (mine, before step 4; the list after §15g's steps 6–9). The loop re-estimates every 500 steps,
and §16y rule 1 runs a press that goes non-finite, fails a validity gate or inverts an element again from its start,
re-estimating every 50 steps; a press that fails again does not stand.
- At eight times h_K2's elements no run needed the re-run, and re-estimating every 50 steps instead took 2.34 times
  the time for deciding readings within 0.01 % (§16z). Where the re-run fires, that corner costs its attempt plus a
  run every 50 steps; at twice h_K2's elements it fired (§16y), untimed. If D1's size moves there, it is costed again.
- Tracking the step is not taken: it is not costed, its bound is not measured for this element (§16y), and the
  recorder takes a step's values from the host (§17a).
- The rule stays with the code that runs a press, today the probe (`step7_first_run.rs:1076-1112`, with rule 6's
  longer hold). `Stepper` is unchanged.

**Two measurements on Metal** (the M4 Pro, wgpu 27; the probes are kept locally).
- **Fast math.** wgpu-hal does not set Metal's fast-math option, and wgpu 27 has none
  (`wgpu-hal-27.0.4/src/metal/device.rs:208-213`). TwoSum's error term and Kahan's compensation of 1 + 1e-8 came back
  0 (IEEE: 1e-8), and a·b + c was rounded once. The design's review measured that a sum with a NaN in it is NaN, that
  `max`, `min` and `select` drop a NaN, and that `x != x` is false for one. So the device keeps no compensated sum,
  and long sums go to the host at f64.
- **Passes.** 960 dispatches recorded as 960 passes took 19.8–22.7 ms to record and run, and as one pass 1.4–8.6 ms:
  12–22 µs a pass. One pass per phase would spend roughly a fifth to a third of a GPU step's share of D4 at eight
  times h_K2's elements on passes (arithmetic on §16z's local figures). So a step is one pass.

**The design.** `sim-gpu` gains `soft`, an `Executor` over `sim-soft-explicit`'s model and obstacle, with entry points
around `SHARED_WGSL` (§14a). `sim-gpu` depends on `sim-soft-explicit` (73 crates to 80; L0-io allows 200), and
`cf-sim-research` on `sim-gpu`. The device runs f32; the trait carries f64 (`executor.rs:5-7`).
- **The phases** are the CPU executor's, one entry point each, over the same arrays.
  - Each gather sums a node's incidence list in its order (`cpu/executor.rs:139`, `:186`); no atomic adds a float.
  - The counts, inverted elements and coarse corrections, are u32 atomics into a 64-bit pair with a carry.
  - The internal energy at a read (phases 1–2) and the estimate (phases 1–5) run on arrays of their own and do not
    count (`:217-270`, `:796-802`).
  - κ_e is taken as the CPU takes it; conformance is at κ = 0 (§15g step 4), and one fixture at κ = 2μ (§17c).
  - The per-node constants share one array and each surface node's contact state another, so each entry point binds
    at most 16 storage buffers (`context.rs:49`).
- **The pose, on the host.** `contact` interpolates the pose at the step's start and end with the shared f32 math, as
  the CPU executor does (`cpu/executor.rs:527-539`), and passes both in the step's values, so `set_poses` writes
  nothing to the device.
  - The same call keeps the step's origin, move and turn from the f64 track (`:904-917`).
  - `set_state`'s default anchors are formed on the host the same way.
  - This replaces §14d's pose samples streamed to the device; a pose formed on the device (§14a's rigid–soft
    exchange) would need another design.
- **A step's values** (§17a's ring): dt, damping, the two poses, the step's two log rows, and an estimate's
  perturbation and β, since a recorder has one type. In WGSL the poses and what follows them are aligned to 16 bytes,
  and the Rust type is padded to match (`submit.rs:169`, `:183`).
- **One pass a step.** Each phase appends its dispatches to a pending list, held in the loop's order: 1–8, then
  `accumulate`. The list is recorded as one pass, in one recorder step, when a phase does not follow the last one
  pending, when dt or damping changes, and before any read, write or `clear_accumulators`. In the loop that is once a
  step.
- **The step log.** Each `contact` writes a row: the contact forces' resultant, their moment about the rest
  positions' centroid c, and the normal sum. Each `boundary_conditions` writes a row of its own, the contact work and
  damping loss; the two are counted apart, as on the CPU (`:936`, `:965-973`). Rows are read back only at the trait's
  reads (`executor.rs:316-319`), and a full log grows on the device.

  | At f64 on the CPU | On the GPU |
  |---|---|
  | The resultant, moment and normal sum: per step, then over steps | A fixed f32 tree per step, into the row; the host adds rows at f64 |
  | The moment's arm, from the f64 track's origin p | About c: each rest position less c at f64, narrowed, and the displacement added on the device; the host moves it, M = M_c + (c − p) × F |
  | The obstacle's work, each step | On the host at f64, from each row and its kept origin, move and turn |
  | Contact work and damping loss, per node | A fixed f32 tree per step, into the row; the host adds rows at f64 |
  | Inverted elements and coarse corrections, u64 | 64-bit atomic pairs; exact |
  | The deepest penetration and prediction, per node | Per node at f32, maxed at a read; exact |
  | Kinetic, contact kinetic and internal energy, at a read | Workgroup partials by a fixed f32 tree, added at f64 on the host |
  | The window sums, per node | Per node at f32 between reads, added into f64 on the host at a read |

- **The power iteration** follows `top_mode_and_vector` (`cpu/executor.rs:631-732`), with one read an estimate.
  - Its two maxima are `max` trees, which on Metal drop a NaN as Rust's `max` does, into a buffer the next dispatch reads.
  - Its breaks (`:677`, `:715-718`) set a flag that idles the later iterations, and keep the vector and quotient the
    CPU keeps.
  - The damping quotient takes one viscous evaluation after the loop (`:723-729`).
  - The quotients' partials are read once and added at f64.
  - The start vector is formed on the host at f32, as the CPU forms it.
  - An estimate is one recorder step, one pass.
- **Reads and writes** go through the recorder (§17a). Each trait read is one read; `monitors` and `snapshot` also
  take the log and, when a window has added steps, the window sums. `set_state` is three writes, with no submit between them. No wgpu feature is
  needed. The product's fine grid is above wgpu's default binding size and within Metal's 4 GiB (§15g step 4); CI's
  fixtures stay under the default.

**The fixtures,** each set on both executors from one state, deformed 1–10 % unless named:
- the tube, frictional;
- a block of two materials, at ν 0.49 and 0.4975;
- a viscous block;
- nodes held whole, and in one or two directions;
- an obstacle with a fine grid;
- an obstacle whose track moves and turns;
- friction sticking, and friction slipping;
- an element compressed past |J − 1| = 0.25, where `ln_1p` takes the backend's `log` (`shared.wgsl:225-226`).

Each asserts, from the CPU's state, that it stays clear of where an output jumps: a node starting to touch or turning
from sticking to slipping (its anchor, `shared/contact.rs:119-124`), the movable reach (`KINEMATIC_MIN_REACH`), the
fine grid's edge, and J = 0. No node is left out of a comparison.

**The gates,** set before the build. Each is made to fail once by the change named. They run on Metal locally and on
lavapipe in CI's `tests-debug` shard 3, unless named.

| Gate | Made to fail by |
|---|---|
| Per-phase conformance, GPU f32 against CPU f32, on every fixture, the phases run in order up to the one read; by the bars below | for each phase, a term dropped from its kernel (a corner swapped in a gather; the projection skipped in `boundary_conditions`) |
| Each fixture's margins | a fixture moved onto one |
| The obstacle that moves and turns | its two poses swapped; the moment moved with the wrong sign |
| `set_poses` between two steps: the obstacle's work is the same with a read before it as without one (the GPU against itself) | the origin, move and turn formed at the read |
| With one element inverted well past J = 0: the phase outputs and the count read after `monitors` and after an estimate equal those read before | the energy's phase 1 on the step's arrays; its phase 1 counting; the estimate on the step's arrays |
| A step recorded as one pass is byte-identical to it recorded pass by pass | the pending list replayed out of order |
| The estimate's ω² and damping quotient within 1e-3 of the CPU f32 executor's, the CPU f32's distance from f64 printed beside; a break at the first iteration leaves ω² at 0; one read an estimate | the free-direction projection dropped from K v; ω² as 0/0; a read each iteration |
| On Metal, three runs of 1 000 tube steps through `Stepper` repeat bit for bit | a gather written as a scatter that races |
| With no window open, a run read every step and the same run read every 100 steps, its log growing between reads, give identical cumulative monitors and final state | the rows dropped when the log grows |
| A still state's window sums over 1 000 steps, read every 100, are each within 1e-5 of 1 000 times the state; the test shows the state's f32 sum over all 1 000 steps misses 1e-5 | no window sums added at a read |
| A NaN in one node's velocity, read before any step, makes the monitors not finite, and `run_until` stops, on the GPU as on the CPU | the kinetic energy's tree dropping a NaN |
| A count set just below 2³² and counted past it reads the exact total | no carry |

**The bars.** §15g step 4 holds each output to 1e-5 of its largest magnitude. Two are amended here, before the build
(mine):
- a sum is held to 1e-5 of its terms' summed magnitudes (for the moment, Σ|x − c||f| + |c − p|Σ|f|), since the
  device's f32 tree is within ⌈log₂ n⌉ · u · Σ|terms| and keeps no compensation;
- phase 6's contact and normal forces are held to the larger of 1e-5 and twice the CPU f32 executor's distance from
  f64 on the same fixture, since f32's rounding of the grid coordinate sets their precision. The design's review
  measured the CPU f32 2.9e-5 from f64 on the 10k tube with the nodes on the mandrel, and 6.9e-6 with the bore
  0.5 mm inside it.

Maxima are held to 1e-5 of themselves and counts exactly. The estimate is held to 1e-3, the bar the repo holds f32's
ω² to against f64 (`tests/executor.rs:686`).

**If a gate misses,** it is not re-tuned; a gate its named change cannot make fail is deleted, and the record says
so.
- Conformance: the record gives the GPU's distance from the CPU f32 executor beside the CPU f32's from f64, and step 4
  is not done until the bar is met or Jon changes it.
- G6 on the GPU: deciding readings beyond K3's 0.5 %, or a corner that stands on one executor only, leave step 4
  undone and go to Jon. A corner that rule 1 runs again on one executor only is compared as each ran.
- A D4 miss leaves step 4 done, and goes to Jon.

**D4: a proposal for Jon** (§17). The revision §15g step 2's rule asks for is owed; its date is Jon's.
- **The measurement.** G6 on the GPU: a press on `base_mold` at eight times h_K2's elements, the three corners under
  the step control above, through the probe's `press` on Metal. It is reported over D4 and over the CPU (four
  threads, §16z), with its deciding readings against the CPU's at K3's 0.5 %. It runs locally under the memory
  watchdog, since the wall took about 4 GB; no reviewer runs it.
- **The bar.** At least 11.8 times the CPU, at ν 0.49 and the nominal viscosity. With ν as a corner it is about 24–26,
  and about 37 at the viscosity's high end or at sixteen times h_K2's elements (§16z). It would replace K1's per-step
  budget, which is 53 of D4 there, so it cannot fail a GPU that misses D4. That replacement is Jon's.
- **A miss.** The quality gates and the element size stay (§9 decision 12, §16z). The options, for Jon: a faster
  executor (fused phases, §14d; local stepping, not costed); or D4's machine or budget (a faster device through the
  same code; fit plan D4: *"revisit the budget rather than cut the physics"*).

**Carried, and not here.**
- §17a's failed wait or map: the build tries a device destroyed before a read. If the read reaches the recorder, it
  becomes §17a's gate.
- Not here: soft-on-soft contact; fused kernels; a faster device; K1–K6 on the GPU (step 5). The Metal facts above
  are wgpu 27's.

**Done when:**
- every gate passes where the table runs it;
- `sim-gpu` grades A, and `sim-soft-explicit`, `sim-gpu-benches` and `cf-sim-research` pass and grade as before;
- G6 on the GPU is recorded here;
- the notes §17b changes point here: §14d's and §17a's pose on the device, §16e's and §17's read per power
  iteration, §15g's step-control item, fit plan U20, and `set_poses`' doc (`executor.rs:349-353`).

**How the design was checked.** Two rounds of cold review (three reviewers; the reviews are kept locally).
- Round 1, a code reviewer who measured on Metal and a whole-plan reviewer, changed the design:
  - the energy read and the estimate on their own arrays;
  - the obstacle's work formed at `contact`;
  - the log read only at the trait's reads;
  - G6's consequence;
  - a turning obstacle and ν per fixture;
  - gates that could not fail.
- Round 2, one fresh reviewer, found 16 problems, 11 written by round 1's fixes and all in gates or prose.
- The section was then cut to this. Round 2's fixes and the cut have not been reviewed.

### 17c. Step 4, as built (2026-09-30)

**Where.** `sim/L0/gpu/src/soft.rs`, the executor; `soft/kernels.rs`, each entry point's bindings, pipeline and
dispatch; `soft/log.rs`, the step log's rows added at f64; `soft/soft.wgsl`, 21 entry points after the shared math;
`soft/tests/`, the fixtures, the conformance gate and the other gates. The rigid pipeline's wgpu helpers moved to the
crate root for both; `ExplicitModel` gained `shortest_edge`, which both executors read; the recorder counts its reads
under test. The probe's `step7_cost` takes `STEP7_GPU=1`, and times a press under the decided step control alone.

**On Metal** (the M4 Pro, wgpu 27; each figure is its test's output at `7649a1bc`, the record kept locally):
- Conformance on nine fixtures, §17b's and one at κ = 2μ, the GPU's distance from the CPU at f32 against each output's
  bar:
  - phases 1–5 at most 4.6e-7 of the largest magnitude;
  - phase 6's contact forces 4.8e-6 against 1.5e-5, and its normal forces 5.0e-6 against 1.3e-5, both on the tube,
    each bar twice the CPU at f32's distance from f64;
  - phases 7–8 at most 7.5e-6 against 1e-5, on the tube, where the CPU at f32 is 8.3e-6 from f64;
  - the window's sums of one step as their phases' outputs. The friction's takes phase 6's amended bar, which §17b
    does not list: at most 3.1e-3 against 6.0e-3, where the CPU at f32 is 3.0e-3 from f64;
  - each sum at most 6.6e-7 of its terms' magnitudes; the maxima equal to the digits printed; the counts exact.
- The estimate: ω² within 1.6e-6 and the damping quotient within 6.8e-6 of the CPU at f32 (bar 1e-3); the CPU at f32
  is 8.6e-5 to 2.7e-4 from f64 in ω², and 5.8e-4 in the damping quotient.
- Every gate failed under its named change, on its own assertion, and each fixture under a change taking away what
  it is for (49 changes at `ed965b2c`; the script and its log, with each failure's place, kept locally). Three were
  not the first tried:
  - phase 6's first, the damping dropped from the prediction, did not fail; the step's advance dropped did.
  - the repeat gate's race, a gather written as a scatter, blows the runs up; they stop at different times. The gate
    now compares runs that stop, every value by its bits, and a race that leaves them finite, the kinetic energy added
    into one term, fails it too.
  - the margins': at `J − 1 = −1` the pressures missed their bar (1.0e-4 against 1e-5) while `J` cleared the margin
    then, 1e-3. The margin is 0.1, and `J − 1 = −0.95` fails it.
- A reduction past 256 blocks and past 65 535 workgroups (16.8 million items) has its gate. Each of these changes
  passes every gate: the per-item index past 65 535 workgroups (4.2 million items); held nodes left out of
  the deepest prediction; the projection in the estimate's damping; the damping in phase 6's prediction; a race left
  in a reduction's tree, which three Metal runs did not show. No gate is written for the power iteration's second
  break, a scaled vector of zeros.
- §17a's failed wait or map: a device destroyed before a read fails in wgpu's validation at `map_async` (the staging
  buffer is invalid), before the recorder's wait or map (a probe, kept locally). It is not the recorder's path, so no
  gate follows.
- Between reads the window sums add at f32 on the device. The still state's gate reads every 100 steps, and its own
  check shows f32 missing 1e-5 over 1 000 steps without a read. The probe reads by travel; its interval at eight times
  h_K2's elements is not recorded.
- On lavapipe the gates are CI's (`tests-debug` shard 3); the repeat gate runs on Metal only.

**Grades** (`RAYON_NUM_THREADS=1`): `sim-gpu` A at `ed965b2c`; `sim-gpu-benches` and `cf-sim-research` A at
`9f13719f`; `sim-soft-explicit` A at `b62acd5b`, its code unchanged since.

**On the product** (`base_mold`, through the probe's press; the logs kept locally):
- At h_K2's elements and the budget's loading, the GPU and the CPU took the same steps in every run, D1's readings
  within 0.01 % of each other, and the same corner (μ_f 0.18) fell short of the same validity gate on both.
- **G6 at eight times h_K2's elements** (`step7_cost` at `b62acd5b`; the loading four times the budget's, as §16z's;
  the loop's interval with §16y rule 1's re-run; the GPU, then the CPU at four threads, from one binary):

  | Executor | A press over D4 | Full verdicts at 1 and 2 insets, over their 15 min |
  |---|---|---|
  | GPU (Metal) | 2.30 | 0.77 and 1.53 |
  | CPU (4 threads) | 12.4 | 4.1 and 8.2 |

  - The GPU is 5.4 times the CPU (arithmetic). The CPU took 12.4 of D4 here and 11.8 at `cf601cb0` (§16z), over the
    same steps in each corner; what differs between the two CPU runs has not been isolated.
  - Every corner stood on both, and neither ran rule 1's re-run. The steps agree within 0.011 % a corner, and the
    deciding readings within 0.002 %, against K3's 0.5 %.
  - Making the device (`GpuContext::new`) is outside the timer, and not measured.
- **D4 is missed:** meeting it takes 2.3 times this GPU's speed (arithmetic). Under §17b's rule the miss does not
  undo step 4; D4 goes to Jon with §17b's options.

**How the build was checked.** One round of cold review, four reviewers (the kernels, the executor, the gates, the
whole plan), then one fresh reviewer on that round's fixes; their findings kept locally. No reviewer found a finite
state in which an output of the executor differs from the CPU's; after a run had blown up, one found the GPU's
contact outputs finite where the CPU's were zero, the loop stopping on both. The first round found gates that could
not fail for a path they claimed (the window sums and their clear, the fine grid, `set_poses`, array lengths, the
repeat gate's fields), paths no gate covered (κ, reductions past 256 blocks, the fixtures' purposes), and figures and
claims in this record that its logs did not carry. The second found 10 more, 7 made by the first's fixes: κ had gone
onto the viscous fixture, taking viscosity at κ = 0 with it, and the widened comparisons and the purposes had not been
made to fail. A third, narrow, on those fixes found 2: three fixtures asserted nothing of what they are for, and
the one-pass gate's later comparisons had never failed. The mutation record above was taken after all three; the
third round's fixes have not been reviewed.
