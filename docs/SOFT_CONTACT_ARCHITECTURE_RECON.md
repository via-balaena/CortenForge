# Soft-contact architecture

**Status:** the plan, 2026-09-24. Build step 1 is merged (#965). Step 2 is designed in §16, and not yet built.
- **Research:** §1–§10.
- **Code architecture and the crate layout:** §11–§14.
- **The first experiment and its kill criteria:** §15.
- **Build step 2's design:** §16.

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
  - pressure as area-weighted percentiles, not a single-node peak;
  - push force split into its geometric and friction parts (§6).

## 3. Why the current approach failed — measured

- **We built the most demanding formulation:**
  - quasi-static equilibrium at every step;
  - guaranteed non-penetration (IPC);
  - tight tolerances;
  - a fine quadratic mesh (65 293 Tet10);
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

Round-2 scenario ranges, silicone sleeve on skin, from the surveys. The water-based-lubricant row rests on
[a latex-coating study](https://pmc.ncbi.nlm.nih.gov/articles/PMC6227966/): COF 0.159 fresh, above 0.30 at
600–900 s.

| Scenario | Range | Nominal |
|---|---|---|
| Dry | 0.4–1.0 | 0.6 |
| Water only | 0.15–2.0 | ~1.0 |
| Water-based lubricant | 0.05–0.5 | 0.15 fresh, 0.3 depleted |
| Silicone lubricant | 0.05–0.3 (estimate; no direct measurement) | ~0.15 |

- **Decided (Jon, 2026-09-24): "different things can have different lubricants, so plan for that".**
  Friction is a **library of contact pairings**, each with a sourced range: surface × surface ×
  lubricant, with fresh and depleted states. It extends beyond the sleeve (for example sock on skin,
  leather on sock, oiled rubber on steel, a hydrophilic catheter on mucosa). The fit test picks the
  pairing, and reports the range.
- **Friction roughly doubles within 2–15 minutes** as a water-based lubricant is rubbed away.
- **Silicone oil swells silicone**, so silicone lubricant may not suit a silicone sleeve. This was not
  researched further.
- **No measurement exists for silicone on skin with a lubricant.**
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
- **Boundary options:** a free outer wall, a rigid case, or bonding to a stiffer outer layer (§5b).
- **Friction:** from the pairing library (§5c), swept over its range.
- **Readouts:**
  - push force along the path, with its μ = 0 geometric share;
  - pressure maps as area-weighted percentiles;
  - stretch.
- **Size of it (arithmetic):**
  - about 29–32k steps for the 100k-tet benchmark at ν 0.49, which leaves 3.7–4.1 ms per step within
    the 2-minute budget (§15c);
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
| **Friction pairings** | the round-2 survey table (§5c) and its sources | silicone on skin with a lubricant is unmeasured. Use the nearest analog and a wide range |
| **Comfort and pain limits** | the axial-rigidity convention (below); for stockings, *"Self-prescription is reasonably safe assuming that the compression gradient is 15–20 mmHg"* (≈ 2.0–2.7 kPa, [Wikipedia](https://en.wikipedia.org/wiki/Compression_stockings)) | **Pressure-pain thresholds for the relevant tissue were not found** (search cut short). Needs another research round |
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
     shared infrastructure (§14).
   - Building it at once would put a broad phase into the first build before the base solver has passed
     its kill criteria.
   - When it is built, it gets its own research round (how practitioners do explicit soft self-contact
     on the GPU) and its own validation case, as §4 and §7 did for the base solver.

10. **The product's outer boundary** (Jon, 2026-09-24): *"right now it has no shell, its outside is free.
    but in the future i might add a shell/bond it to a shell. also sometimes there are multiple silicone
    shells layered."* So today's regime is the free wall (§15h). The confined case becomes a gate
    before any shell or bond design is simulated (15g step 2).
11. **How the device is held** (Jon, 2026-09-24): *"it really depends, it could be in a shell, connected
    to something like a robotic arm, or just held in the hand."*
    - How it is held is a design option, like the outer boundary: in a shell, mounted to an arm, or held
      in the hand.
    - A shell, or a mount at the closed end, confines the material near it. So the confined case gates
      those designs too.
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
- the penetration bound, G2 (fit plan §5).

## 10. What the research could not see

- **Cut short by the search limit:**
  - glove, sock, finger-ring and consumer-device insertion force;
  - catheter and endoscope insertion *simulation* models;
  - Stribeck curves for skin lubricants;
  - measured ν or K for Ecoflex and Dragon Skin;
  - O-ring and seal ν-sensitivity studies;
  - LS-DYNA and Radioss GPU efforts;
  - any WebGPU explicit FEM;
  - pressure-pain thresholds for the relevant tissue.
- **Paywalled:** Taylor 2008 (TMI) and Strbac 2015 (single against double precision), Nedoluha 2025 (ν
  measurement methods), and the NAFEMS R0081 references.
- **Unmeasured anywhere we looked:**
  - silicone on skin with a lubricant;
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
  - chunked submits (long command buffers hung the readback, `pipeline/orchestrator.rs:28-37`, round 1);
  - the atomic contact append, extracted as shared GPU infrastructure for soft-on-soft contact (§9
    decision 9). The CAS float-add is extracted too, if scatter is chosen over per-pair slots and a
    gather. The first experiment needs neither;
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
| **`sim-soft`** | L0 | grows | The model as today, plus **lowering** it to `sim-soft-explicit`'s data, including resampling the insertion path evenly in time. **Baking the obstacle SDF from its triangle mesh** (flood-fill sign and the Gaussian pre-smooth, moved from `tools/cf-sim-research`). The **scenarios and readouts in model terms** (contact pressure by region). The test of its `Material` impls against the shared math (F3). The implicit Newton solver stays as it is. |
| **`sim-gpu`** | L0-io | rebuilt | **The GPU executors.** It *extracts* shared infrastructure from today's rigid code: the device context (`context.rs`), and chunked submission, which today sits inside the rigid `step()` (`pipeline/orchestrator.rs:28-37`), and the contact-list tools (the atomic append; the CAS float-add if scatter is chosen). It adds `soft`, the explicit executor, whose hand-written entry points fetch, gather and scatter around the generated WGSL. It holds the **GPU-vs-CPU conformance tests** against `sim-soft-explicit`'s CPU executor. The rigid pipeline stays as it is until its own redesign, keeping the parts only it uses. It depends on `sim-soft-explicit` and `sim-core`, not on `sim-soft`, and has its own wgpu version (13e). |
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
    fit-test path.
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
  any crate boundary. The lowered data carries the surface triangles from the start.
- The GPU executor records each phase as compute passes, submits in chunks, and **never reads back**
  except on an explicit read call. The pose samples stream to the device a batch at a time.
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
  - whether `sim-gpu` grades A today;
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
| **K3 precision** | CPU f32 against CPU f64, same executor: band pressure within **0.5 %** (frictionless, 50k), and the Coulomb push's reaction within **0.5 %** (μ_f 0.3, 10k). *Amended 2026-09-24 (PR #965 review):* the band pressure also within 0.5 % at every pair-averaged ring level (15d.1), not only in the mean, since D1's 95th-percentile reading depends on the local values | step 2 of the build (15g), before any GPU code. This is the fit plan's *"precision spike on contact before any GPU contact code"* |
| **K4 robustness** | J > 0 in every element at every step of every valid run | explicit check (§13d rule 2). Any J ≤ 0 in a valid run is a failure. A run that breaks a validity gate is invalid, and K4 does not judge it |
| **K5 product readings** | the peak push force during entry and the seated 95th-percentile pressure (fit plan D1's readings) change ≤ 5 % from 50k to 100k | the tube's entry is a sharp edge, like the product's mouth. **A gate on the verdict's design, not on the solver:** if it fails, D1's readings or the lip radius are revisited before step 7 |
| **K6 friction** | *Amended 2026-09-24, in step 2's design and before any data (16b):* Cattaneo–Mindlin partial slip in plane strain, a rigid cylinder on the block. The stick zone's half-width within 0.03a of the closed form while the tangential load rises to 0.8·μ_f·P, and the retained stick zone's within 0.03a while it falls back. It replaced frictional ironing, whose published curves could not carry a 5 % gate (16b) | CPU, build step 2. Friction's only external reference: the Coulomb push (15d.7) checks consistency only |

**Why two corrections to K2:**
- **Requiring both raw and gap-corrected results:** the penalty's gap biases pressure low by about 1 %
  (15c), and the element biases it high by 0.7–1.65 % at 50k–100k (15c). A raw pass could come from
  the two cancelling. The gap-corrected number isolates the element.
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
- **Mass:** lumped, ρV/4 per node (the implicit solver's rule, `construct.rs:607-619`).
- **Time step:** Δt = 0.9 · 2/ω_max, with ω_max from power iteration on M⁻¹K through the executor's
  own force phases, **penalty stiffness included**.
  - **The iteration is re-run during loading**, every 500 steps, each from the same fixed start
    *(amended in 2a, 16m: warm-started, it stalled on a lower mode once loaded)*. The step never grows
    by more than 5 % at a time.
  - A reviewer's model (not kept) measured, on 6 × 47 × 59: deformation cut the limit to 0.912× its
    rest value, and a Δt fixed at rest with the penalty gave ωΔt = 2.041, which is unstable.
  - The altitude estimate is a cross-check only. The method research measured it 4.4× loose on a
    jittered mesh (not kept).
- **Loading time T:** set by a convergence ladder. Halve T from about 10 axial-shear periods (10·T_s) until the
  band pressure moves by more than 0.5 %, or KE/IE exceeds 5 %.
  - Time scaling stands in for mass scaling. They are equivalent for rate-independent material and
    friction (Abaqus *Getting Started* §13; DERIVED), so the physical density is kept.
  - **Arithmetic** (100k, ν 0.49):
    - Δt at 0.9 of the rest limit is 42.2 µs on the pre-registered 6 × 64 × 43 mesh. That comes from a
      reviewer's model (not kept).
    - T is 1.04 s plus the 0.2 s hold: about 29k steps at that Δt. It rises to about 32k if loading cuts
      Δt to 0.912×, as on the other mesh.
    - That leaves **3.7–4.1 ms per step within K1**.
    - The rigid pipeline's whole step is about 0.74 ms at n_env 1 (§6).
- **Damping:** mass-proportional damping α_D·M while loading (NiftySim, Johnsen et al. 2015), with
  α_D = 2·ξ·ω₀, where ξ = 0.05 and ω₀ = 2π/T_s.
  - It drags a translating body with force c·m·v (DERIVED). Holding the tube and driving the mandrel
    keeps that small.
  - No dynamic relaxation in any run that K1 or K2 judges.
- **Contact variant (a): nodal-mass penalty** k = s·m_a/Δt², s = 0.5, **primary**.
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
  - **Friction is not defined for (b)**; the Coulomb push uses (a).
  - The rule: (b) replaces (a) only if it meets K2 with less scatter at equal cost. *Amended
    2026-09-24 in step 2's design (16d): (b) cannot replace (a) in the product, since a verdict's
    μ = 0 run must use its frictional runs' law. It stays a K2 diagnostic (15e).*
- **Pressure readout:** force per node over its tributary area, a third of each incident deformed
  boundary triangle (`sim-soft`'s convention, `mesh/mod.rs:433`), area-weighted over the band as
  ΣF_n/ΣA.
- **Friction (the Coulomb push and K6):** Coulomb, with an elastic-slip stick state: a tangential penalty with
  the same k, and a return map on a per-node anchor, as in Abaqus's penalty friction.
  - It is rate-independent, so time scaling stays valid.
  - The fallback is viscous regularization, with v_ε ≥ μ_f·f_n·Δt/m to avoid chatter (DERIVED).
- **The SDF for K2 is pinned:** the mandrel baked into a grid at cell A/20, clamped, with the
  finite-difference gradient (the product's path, §14b).
  - A reviewer's model (not kept) found a grid at A/10 reads the true surface +7.6 µm off, 0.76 % of the
    interference. Trilinear error goes as h², so A/20 should give about a quarter of that (arithmetic).
  - The analytic SDF is the diagnostic (15d.9).

### 15d. Measurements (instruments pre-registered)

1. **Band pressure (K2).** The area-weighted mean over the pre-registered band.
   - **Raw:** against the oracle at radius a.
   - **Gap-corrected:** against the oracle at a − ḡ, where ḡ is the band's mean gap to the *true*
     mandrel surface. That covers the penalty gap and the SDF bias together.
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
8. **The confined stress case:** cased outer wall, all axial motion held, λ_a 1.1, B/A 2, ν 0.49. The
   oracle gives p/μ = 4.1417. This is the regime TLED never validated. The product is free today (§9
   decision 10), so in step 2 it is reported. It becomes a gate, against G2 and 5 %, before any shell or
   bond design is simulated.
9. **The SDF comparison:** the analytic mandrel, against grids at A/10 and A/20, with the
   finite-difference and the analytic trilinear gradients. Record band error and scatter.
10. **Stiffness scaling** (§5d's shortcut): the same run at μ and 2μ, frictionless and at μ_f 0.3.
    Record whether every force scales by 2 within 2 %. This decides how many runs a verdict needs
    (15h).

### 15e. If K2 fails

- **Localize first:**
  - raw against gap-corrected;
  - penalty against kinematic;
  - the mesh-ladder trend: slow convergence points at the element, a plateau at contact or the SDF;
  - analytic SDF against the grid.
- **Element fallbacks, in order.** Each fits the shared math plus a gather phase.
  1. The averaged nodal deformation gradient (Bonet, Marriott & Hassan 2001), or F-bar-patch
     (de Souza Neto, Pires & Owen 2005).
  2. Cyclic J smoothing, which *"suppresses the pressure oscillation"* (Onishi et al. 2017).
  3. Split-energy ANP. This would need the oracle rerun with the split W.
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
     recorded here.
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
       Δt.
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
       loosened.
   - **The stop rule (pre-registered; amended 2026-09-24 in 16i, before any data):**
     - Proceed if the 50k gap-corrected error is ≤ 5 % at every corner.
     - Proceed with a flag if it is 5–7 %, and the extrapolation reaches ≤ 5 % at 100k at every corner.
       The model: h = (mean tet volume)^(1/3), and e = C·h^p, with C and p fitted per corner from 10k
       and 50k. Where a CPU run at 100k exists, its measured error decides instead, in every band (16i).
     - ⛔ **Stop before any GPU work** otherwise, or if K3, K4, K6 or the Yeoh case (16h) fails.
   - A reviewer's model (not kept) put the element alone at +1.2–1.65 % at 50k.
   - *Done when:* the stop rule has been applied, with its numbers written here.
3. **`sim-gpu`: the shared GPU infrastructure is extracted:** the context, chunked submission and the
   contact-list tools.
   - It stays on the workspace's wgpu (27) until a need for a newer version is named. The physics' own
     wgpu entry (§13e) lets it move without Bevy.
   - When it moves:
     - `gpu-probe` migrates;
     - the shaders are validated under the new naga;
     - a binary holding both versions runs on lavapipe (Vulkan) in CI.
   - *Done when:* `sim-gpu`'s suite passes on Metal and in CI.
4. **`sim-gpu`'s soft executor,** with per-phase conformance against the CPU executor, on lavapipe in CI.
   - *Done when:* every phase's outputs agree, GPU f32 against CPU f32. Per output, the largest
     difference must be ≤ 1e-5 × the largest magnitude.
5. **The experiment on the GPU:** K1, K2 at 100k, the ν sweep, the ladder, the Coulomb push, the stress
   case, the SDF comparison and stiffness scaling. *Stiffness scaling runs first on the CPU, in step 2
   (16i), because the product's budget depends on it.*
   - *Done when:* K1–K6 are decided and the results are in this document with their commands.
**Steps 6–9 are an outline.** They are designed in detail after step 2's results, which set the
product's mesh, budget and contact law. Three macro reviews found what that design must settle:
- the per-press time against D4's target (§9 decision 12), which follows from the runs per verdict;
- the pairing's nominal corner, which D3 judges at: add it as a run, or show push force is linear in
  μ_f;
- Tier 1's accuracy against the solver, and what D3 does if it is poor;
- modelling each way of holding the device (§9 decision 11): a shell, a mount, or a hand as a soft,
  distributed support. A held closed end is the "no escape" row of §15h;
- the product mesher's surface bias and element count (measured in step 2);
- U3's outcome, and the fact that a contact-guided intruder would need rigid–soft coupling.

**Starting now, in parallel with steps 1–2, needing no solver:**
- U3's geometric check;
- friction sourcing for the product's pairings (§5c: the dominant input, uncertain by more than 10×);
- the comfort-limit research (step 9).

6. **`sim-soft`: lowering, obstacle baking, and the pairing library.**
   - Lowering and obstacle baking come from `cf-sim-research`.
   - The pairing library covers surface × surface × lubricant, with fresh and depleted ranges (§5c).
   - The boundary options (§9 decisions 10–11):
     - free;
     - cased, as a radial kinematic constraint;
     - bonded, through per-element materials, with the interface rule chosen against a two-layer
       reference (step 1);
     - mounted at the closed end;
     - hand-held, as a soft, distributed support.
   - The F3 conformance test of `Material` against the shared math lands here, since `sim-soft` takes
     the dependency.
   - **U3** (fit plan) is settled geometrically before step 7: with no inset, the rigid path already
     asks 8.3 mm of room. Its swept volume is checked against the cavity. If the path is at fault, step
     7's verdicts would measure that and not the fit. The prescribed path is then replaced by an intruder
     that is force- or velocity-driven and guided by contact (the replaced plan's Phase 4 item).
   - Also carried from the fit plan's Phase 1:
     - the t = 0 intrusion and its pre-roll;
     - the outer-skin pin's 646 interior vertices;
     - the path's time sampling.
   - *Done when:* the bake matches `cf-sim-research`'s within 1 % of a grid cell at every grid point, and
     each boundary option has a test.
7. **`base_mold` on the new solver.**
   - G6 is 5 minutes per run, where a run is one simulation (fit plan D4). `base_mold` stays outside
     the repo.
   - The **lip radius** (fit plan "Later"): the lipped cavity built in `cf-design`, then sharp against
     rounded on the same scan.
   - **Tier 1** (the per-slice estimate, §6) is built and checked against the solver on `base_mold`. D3's
     search uses it to pick candidate insets, so full verdicts run at only 1–2 insets.
   - *Done when:* the fit plan's G1–G3 and G6 have numbers on `base_mold`.
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
    μ = 0 for the geometric share of push force (§2). That is about 6 minutes at K1's rate.
  - **A D3 search** of 3–4 full verdicts would take 18–24 minutes, against D4's 15 (arithmetic). Tier 1 therefore pre-filters the search (step 7), so full verdicts run at 1–2 insets.
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
  | `base_mold`, the product, today | free outer wall (§9 decision 10); held in a shell, on an arm mount or in the hand, by design (decision 11) | free or hand-held: ν barely matters. A shell or a mount at the closed end moves toward the cased rows |

  At ν 0.4995 an explicit run needs about 4.4× K1's steps (√(1001/51), arithmetic), roughly 9 minutes
  at K1's rate. **The O-ring class is outside K1's sizing**, and needs its own budget or a mixed
  treatment when that application is reached.
- **Not known yet:**
  - whether the alternating pressure pattern persists in a damped explicit run;
  - the GPU's per-step cost at 100k;
  - the in-loop Δt's cost in steps;
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
    - **The rule is settled in 2c by a unit test,** not here. The test feeds the closed forms, sampled
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

**In CI:** a coarse version (a/h = 12, f64, tolerance 0.1) joins tests-release. It must fail under the
two anchor mutations. At a/h = 12 it cannot see a friction limit 10 % off (a 0.075 shift). That is
covered at the law's level by `a_small_slip_sticks_and_a_large_one_slides_at_the_coulomb_limit`. The
full runs are recorded commands.

**What K6 does not test:**
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

### 16d. What the shared math gains

- **Per-direction constraints.** Each node holds up to two constraint directions, orthonormal, or zero
  when unused. Phase 8 removes the displacement's and the velocity's components along them.
  - This covers the confined case (a radial direction on the outer wall, and z on every node), the
    out-of-plane condition for a 2D reference, and symmetry planes.
  - A fully held node stays `held` (inverse mass 0), as in step 1.
  - The directions are fixed at rest. So a constraint is a plane, not a curved surface a node slides
    along. That is exact for the axisymmetric tube, where no node moves circumferentially.
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
  per iteration.

**The CPU executor is one source file, compiled at f32 and f64** by the same `include!` pattern as the
shared math (§14c). K3 compares the two. It loops with rayon on native, and sequentially on wasm32
(`newton.rs`'s `cfg` pattern). Gathers loop over each node's incident element slots in a fixed order,
so no two threads add into one place. A 2a test checks the consequence: the forces are bitwise equal on
one thread and on many.

**The stable step.**
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
- K6's reference is two closed-form expressions, written in its test with their sources (16b). The
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
  error ≤ 7 % (§15g). If 2b finds another corner worse at 10k, the test moves to it.
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
  95th-percentile pressure, 10k against 50k;
- the Coulomb push (15d.7) and the Yeoh case (16h);
- the confined case (15d.8), and a product-level run: the free tube at λ_a 1.3, ν 0.49, with
  `DRAGON_SKIN_10A`'s μ. Each records its raw error and its largest gap (§15g).
  - Every run also records its largest gap next to p/(λ + 2μ) and its element size. At a fixed s, the
    penalty's gap depends on those and on element shape, not on μ alone (arithmetic: gap = F/k, with
    k = s·m/Δt² and Δt ∝ h/c_d). That record is what carries G2 from the tube to the product's mesh;
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
  - G2's margin at the product's element size, from 2b's gap record.
- **The per-press time is reported against D4's 5 minutes** (§9 decision 12). If it misses, the speed
  plan is revised before any GPU work, and the quality gates are not loosened (§15g).
- Only counts, steps and times are written here. No geometry leaves the machine.

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
- finite strain and the finite domain's effect on K6's stick zone: 2c's two companion runs;
- whether flutter occurs: every 2b and 2c run reports the contact nodes' kinetic energy, and nothing
  gates it;
- the power iteration's accuracy beyond the 10k tube: 2b on the 50k tube, 2d on the product's mesh;
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
- **The trait gains:**
  - `set_poses`, for a new track mid-run (§14d's batches; K6's legs that end on a force);
  - `phase_outputs`, for step 4's per-phase conformance;
  - the contact law's s and μ_f. The loop reads them, so the stable step bounds the law in use.
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
  - The gap scales with element size (16i), so 2b records it on the ladder.
- **G2 is not met on the tube at 10k.** The deepest penetration is 45 µm, 4.5 % of the 1 mm
  interference, against 1 %. §15g step 2 settles the contact law against G2 in 2b.
- **The power iteration at the loop's 100 cold iterations,** against a converged f64 run on the 10k
  tube: −0.21 % at rest, and −0.35 % (f64) and −0.32 % (f32) loaded at K2's end. On a small block it is −1.1 %
  against a dense eigensolve (`tests/executor.rs`).
- **The energy balance's own error** is 0.28 % of the peak internal energy, on a pressed block at α 500
  (`tests/executor.rs`), against the 1 % gate.

**Not recorded in 2a; 2b records both on the ladder, with `RAYON_NUM_THREADS` set:**
- **The whole step's cost,** which §16i said 2a measures.
- **The share of it the cold estimates take.** 2d adds that share to its per-run time. The iteration
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
changes. On the tube the inset is the interference: 1 mm at λ_a 1.1 (bar 10 µm), 3 mm at λ_a 1.3
(bar 30 µm). Stage 1 measured the current law, and the one fallback of §15g step 2 that needs no new
code (a larger s at a smaller Δt). The predictions were written before any run, and are scored below.

**The instrument:** `cargo run --release -p sim-soft-explicit --example tube -- <10k|50k|100k> <case>
<s> <μ_f> <f32|f64>`, where `case` indexes `fixtures::golden::THICK_TUBE` (0: λ_a 1.1 free, 2: λ_a 1.3
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
falls more slowly (1 : 0.77 : 0.62, about h^0.62). Extrapolating that trend, 1 % at λ_a 1.1 needs h
about 5× smaller than 100k's, about 125× its elements (arithmetic). The confined case misses by 42× at
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
| K | **0.02–0.04 %** (0.2–0.9 µm) | **diverges** (10k, 50k) | hold on the free cases | fails "every run finishes" |
| A10 | 26–41 % | diverges | fail: KE/IE 73–80 %, balance 16–26 % | fails |
| A50 | 2.0–10 % | pressure −0.4 %, G2 11–16 % | fail at 10k: balance 1.1–2.0 % | fails |

- **K, free tube:** the deepest penetration against the grid is 0.2–0.9 µm at 10k, 50k and 100k. Against
  the true surface at the end it is the grid's own bias, 3.8–5.6 µm at A/20 and 0.8–1.4 µm at A/40 (10k,
  50k). K2: +4.19 % / +4.41 % (λ_a 1.1) and +3.25 % / +3.29 % (λ_a 1.3) at 10k; +1.28 % / +1.45 % and
  +1.09 % / +1.13 % at 50k; **+0.75 % / +0.94 % and +0.69 % / +0.73 % at 100k.** Raw and gap-corrected
  now agree, as the gap is gone. KE/IE ≤ 0.73 %, balance ≤ 0.46 %.
- **K takes 0.920× P's steps on every mesh** (1.8/ω_el against 1.652/ω_el). Each step costs 11–15 % more
  (a second grid sample and the prediction): 0.345, 0.965 and 1.665 ms at 10k, 50k and 100k. A run takes
  the same wall time as P's (4.9 s, 23.7 s, 52.5 s).
- **K, cased, frictionless, diverges** at t = 0.40 s (10k) with the step unchanged; the contact nodes at
  the tube's entry grow a motion around the tube, about 7× in kinetic energy per 100 steps. Measured, not
  the cause: the step (the loop's ω² at step 3 500 is within 0.05 % of a converged f64 estimate, and
  safety 0.8, 0.7 and 0.5 only delay it, to t = 0.41, 0.43 and 0.51 s); the grid (A/40 and A/80 diverge
  too); the precision (f64 diverges too). **What drives it has not been isolated.**
- **K, cased, with friction, runs clean, and reads the confined oracle:** −0.34 %, −0.31 % and −0.24 %
  at 10k, 50k and 100k (μ_f 0.05), against P's −45 %, −36 % and −28 %. μ_f 0.3 reads the same pressure
  (−0.35 %, −0.32 %), so friction carries nothing at the seated equilibrium. G2: 2.2–4.4 µm (0.22–0.44 %).
  - An earlier version of K kept the friction step off the node's constraints, and at μ_f 0.3 its cased
    G2 read 1.4–1.9 %. The step is now kept to the node's free directions and the surface; a test fails
    on the old step.
- **The Coulomb push** (15d.7) reads 0.87 for P and 0.89 for K at 10k: neither meets 5 %, so it separates
  nothing here. Why both read about 11 % low has not been isolated; 2b's Coulomb push takes it up.

**Predictions, scored:**

| | Predicted | Measured |
|---|---|---|
| Q1 | K's G2 (grid, all steps) < 1 µm everywhere | ✓ free (0.2–0.9 µm); ✗ cased with friction (2.2–4.4 µm, still ≤ 0.44 %) |
| Q2 | K's steps × 0.918 | 0.920 on every mesh ✓ |
| Q3 | K's cased K2 single-digit | −0.24 to −0.35 %, with friction ✓; frictionless diverges ✗ |
| Q4 | K's true G2 = the grid bias: ≤ 5.7 µm at A/20, ≤ 1.5 µm at A/40 | 3.8–5.6 µm, 0.8–1.4 µm ✓ |
| Q5 | A: seated gap ~0, the all-steps maximum above 1 % | all-steps ✓; seated ✗ (A10 rings; A50 up to 115 µm) |
| Q6 | an A may ring or blow up | A10 both ✓ |
| Q7 | K's scatter and contact-node KE above P's | scatter ✓ (e.g. 2.26 % against 1.67 %); contact KE mostly ✓ |
| Q8 | Coulomb within 5 % for K and P | ✗ both (0.87, 0.89) |

**By the rule, nothing is decided:** K meets G2 on every run that finishes, and on the cased tube as soon
as it has any friction; its frictionless cased run diverges. Every A-law fails. The decision, and what
to do about the divergence, is Jon's (2026-09-25).
