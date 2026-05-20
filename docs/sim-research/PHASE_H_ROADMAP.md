# Phase-H roadmap — sim-research leg-by-leg

> **Status**: ACTIVE, drafted 2026-05-19 LATE-NIGHT as B2 of the
> program gameplan reframe. This is the **sim-research planning
> source of truth**. The product pipeline plans live elsewhere
> (cf-* arc memos via `MEMORY.md`); sim research is on its own
> quarter-pace cadence and this doc is where the legs are
> sequenced, sized, and gated.
>
> **Predecessors**:
> - `docs/PROGRAM_GAMEPLAN.md` §5 — the long-horizon sketch this
>   doc fleshes out (5 legs: IPC → mixed-u-p → friction → Tet10 +
>   h-ref → Prony)
> - `docs/SIM_ARCHITECTURE_AUDIT.md` — the foundational audit that
>   surfaced the Phase-C-without-IPC + Phase-H-class-problem gap
> - `docs/studies/soft_body_architecture/` — the design book; the
>   architectural source of truth each leg points back into
>
> **Authority chain**: design book (the spec) → gameplan §5 (the
> reframe) → this doc (per-leg planning) → per-leg design docs
> (land when each leg's work begins).
>
> **Posture**: science, not fire-fighting. Time-equity per gameplan
> §1.3. Quarter-paced. No leg blocks any product-pipeline arc.
> Workshop iteration carries the product forward; sim plugs in
> only when literature-validated AND physical-cast-validated.
>
> This doc is cold-readable.

---

## TL;DR

Five legs, in this order:

| Leg | Deliverable | Horizon |
|----:|---|:---:|
| L1 | IPC barrier replacing penalty (+ CCD + BVH broadphase) | 1-2 quarters |
| L2 | Mixed u-p `<Yeoh>` for ν → 0.499 silicone | 1 quarter |
| L3 | Smoothed Coulomb friction on top of IPC | 1 quarter |
| L4 | Tet10 in contact band + stress-gradient h-refinement | 1-2 quarters |
| L5 | Prony viscoelasticity | 1 quarter |

**Total**: 5-7 quarters of focused sim-research work; roughly
1-2 years. **Not a commitment to a calendar** — gameplan §1.3
explicitly trades schedule certainty for time-equity. Estimates
exist to size honestly, not to lock dates.

Each leg ships with:
- A design doc rooted in a book chapter
- One or more literature-benchmark gates (analytical or canonical-
  problem comparison)
- A regression suite extension
- A retrospective vs the prior leg's output on canonical scenes

The 3-way **sim ↔ physical cast ↔ design intent** validation
harness goes live after L1 + L2 + L3 ship AND cast iter-1+ data
exists. Gap drives prioritization of L4 + L5.

---

## 1. Sequencing rationale

The gameplan's leg ordering (IPC first, then mixed-u-p, then
friction, then Tet10 + h-refinement, then Prony) supersedes the
audit's tentative sub-order (F-bar first, IPC second). Two reasons
the reframe re-sequenced:

### 1.1 IPC is the foundational replacement, not the last fix

`docs/SIM_ARCHITECTURE_AUDIT.md` §4.8 names `contact/` as one of
two highest-leverage gaps and quotes `sim/L0/soft/src/lib.rs`'s
own framing: penalty is a "named-removed-at-Phase-H stepping stone"
that has not yet been removed. Every Newton convergence pathology
the H4/F3/F4/H2 recon arcs chased
(persistent-indefinite-pivot stagnation, N_STEPS resonance, F3 LM
saturation) traces back to penalty's discontinuous Hessian per
`40-contact/00-why-ipc/00-penalty.md`:

> The Hessian contribution from a contact pair is therefore
> discontinuous across the contact/no-contact boundary. **This
> one fact is every downstream failure.**

Until IPC ships, no other Phase-H leg can be validated against
canonical contact problems without the penalty noise floor
contaminating the measurement. **IPC first** is a measurement
prerequisite, not just a fix.

### 1.2 Mixed u-p before friction before Tet10/h-ref

After L1, the stack has stable Newton convergence on a single-
material near-incompressible problem with frictionless contact.

L2 (mixed u-p) addresses the **volumetric locking** that
`20-materials/05-incompressibility/00-locking.md` documents and
the H4-2-C asymmetric-bound surface palliates. The book is
unambiguous about why Ecoflex/DS20A at ν ≈ 0.499 needs full mixed
u-p, not F-bar: F-bar covers ν ≤ 0.49; mixed u-p covers the
limit. With L1 + L2, the canonical scene (sock-on-probe with
silicone material) becomes literature-validatable without
either contact or material noise floors.

L3 (smoothed Coulomb friction) plugs into IPC's barrier
infrastructure (`40-contact/02-friction/02-ipc-friction.md`).
It's small in scope (~500-1000 LOC) but needs IPC to exist.
Pre-L1 there's no infrastructure for it to sit on.

L4 (Tet10 + stress-gradient h-refinement) addresses rim-band
fidelity per `10-physical/02-what-goes-wrong/04-rim.md`. Doing
this on a stack that still has penalty or locking would
multiply failure modes — refining a locked mesh under penalty
gives a finer-grained locked mesh under penalty. L4 third is
the book's order in `70-sdf-pipeline/03-adaptive-refine.md`:
refinement *after* contact + material are correct.

L5 (Prony viscoelasticity) is last because its primary validation
gate is **physical-cast data**, not analytical. The 3-way
validation harness needs L1 + L2 + L3 already shipped before
viscoelasticity adds another degree of freedom to the
sim-vs-physical comparison.

### 1.3 The legs are additive, not interleaved

Per `110-crate/03-build-order.md` Phase H:

> Each extends `material/` or `element/` without changing the
> trait surface from Ch 01, so they are additive.

L1 adds `contact::ipc` alongside `contact::penalty`. L2 adds
`MixedUP<M>` decorator. L3 adds `SmoothedCoulomb` decorator. L4
adds `Tet10` element + `red_green::subdivide` operator. L5 adds
`Prony<M>` decorator. **None of them rewrites prior legs.** This
is why ordering is permissive within constraints — the gate is
each leg's own benchmark, not its compatibility with later legs.

---

## 2. The legs

Each section below is a placeholder; the per-leg design doc
fleshes out the implementation strategy + commit-by-commit
sub-ladder + regression set when work begins.

### L1 — IPC barrier replacing penalty

**Scope sketch**. Full Li 2020 barrier formulation: $C^2$ smooth
log-barrier with compact support; adaptive $\kappa$ schedule;
continuous collision detection (CCD) filter; BVH broadphase for
pair generation. Replaces `contact/penalty.rs` (868 LOC) as the
production contact baseline. Penalty stays in tree as a
research/regression surface but is no longer the default.

**Estimated size**. ~3000-6000 LOC across the new `contact/ipc.rs`
module, BVH infrastructure (`broadphase.rs`), CCD primitives
(`ccd.rs`), the adaptive-$\kappa$ schedule, and Phase-D regression
extensions.

**Design-doc placeholder**: `docs/sim-research/L1_IPC_DESIGN.md`
— lands when work begins.

**Literature-benchmark gates** (all must pass):

- **G1.1 Hertz contact pressure**. Rigid sphere onto soft
  half-space (Neo-Hookean, ν=0.3 to avoid coupling with L2).
  Peak contact pressure $p_0$ matches Hertz analytical
  $p_0 = (6FE^{*2}/\pi^3 R^2)^{1/3}$ to 4 significant digits.
- **G1.2 Non-penetration invariant**. Per
  `40-contact/00-why-ipc/02-ipc-guarantees.md` and Li 2020,
  the IPC barrier guarantees that for any timestep accepting the
  step, $\min_{i,j} d(\text{geom}_i, \text{geom}_j) > 0$.
  Regression test asserts this across every accepted step on
  the canonical sock-on-probe scene.
- **G1.3 Newton convergence on the persistent-pivot scene**.
  Re-run the H2 Q3 baseline (cavity 6 mm, N=16, sock-on-probe).
  Residual must drop monotonically below
  $\|r\|_\infty < 10^{-6}$ within 25 Newton iterations, with
  no vertex pivoting ≥ 10 consecutive iterations. (Direct
  empirical falsification of penalty's persistent-pivot
  signature.)
- **G1.4 Adaptive $\kappa$ schedule bounded**. Per
  `40-contact/01-ipc-internals/01-adaptive-kappa.md`, $\kappa$
  grows monotonically per timestep, bounded above by
  $\kappa_{\max}$ tied to mesh-edge length scale. Schedule
  saturates at $\kappa_{\max}$ on the canonical scene within
  the design timestep range.

**Time horizon**. 1-2 quarters of focused work. Gameplan §5.1
notes "probably its own multi-quarter research arc."

**Design-book pointers**:
- `40-contact/00-why-ipc/{00-penalty,01-impulse,02-ipc-guarantees}.md`
- `40-contact/01-ipc-internals/{00-barrier,01-adaptive-kappa,02-ccd,03-energy}.md`
- `40-contact/03-self-contact/{00-bvh,01-proximity}.md`
- `appendices/00-references/00-ipc.md` — paper list

---

### L2 — Mixed u-p `<Yeoh>` for ν → 0.499 silicone

**Scope sketch**. `MixedUP<M>` decorator wrapping any `Material`
impl, exposing the saddle-point u-p form
$\min_u \max_p \mathcal{L}(u, p)$ where $p$ is the per-tet
pressure. Schur-complement-based linear solve on the augmented
system. Validity-domain composition per
`20-materials/00-trait-hierarchy/02-validity.md`.

**Estimated size**. ~1500-3000 LOC across `material/mixed_up.rs`,
the Schur-complement Newton sub-solve, the pressure-field state
vector extension, the gradient-descent through the augmented
system.

**Design-doc placeholder**: `docs/sim-research/L2_MIXED_UP_DESIGN.md`.

**Literature-benchmark gates**:

- **G2.1 Cantilevered incompressible Tet4 beam**. Per
  `20-materials/05-incompressibility/00-locking.md` and audit
  §1.4. Slender Tet4 cantilever (aspect ratio ~10:1), Ecoflex
  00-30 Yeoh parameters, ν=0.499, tip-load configuration. Tip
  deflection matches Euler-Bernoulli analytical
  $\delta = FL^3 / (3EI)$ to within 5%. Bare Tet4 + Yeoh fails
  this test (locks); `MixedUP<Yeoh>` must pass.
- **G2.2 Cook's membrane patch test**. Standard skewed cantilever
  test ([Simo & Rifai 1990]). Plane-strain analog; ν=0.499.
  Tip displacement converges with mesh refinement to within 2%
  of the reference solution.
- **G2.3 Composition rule preservation**. `MixedUP<Yeoh>`'s
  `ValidityDomain` is the same as `Yeoh`'s minus a
  pressure-bound caveat. Composition unit tests pass per
  `20-materials/00-trait-hierarchy/01-composition.md`.

**Time horizon**. ~1 quarter.

**Note on F-bar vs mixed u-p**. The audit's §5.1 suggested F-bar
first (smaller surface). The roadmap commits to mixed u-p because
the product uses Ecoflex/DS20A at ν → 0.499, outside F-bar's
ν ≤ 0.49 validity. F-bar can ship later as an additional
decorator for less-incompressible materials (carbon-black-loaded
composites), but is not on the path for the canonical scene.

**Design-book pointers**:
- `20-materials/05-incompressibility/{00-locking,01-mixed-up,02-f-bar}.md`
- `20-materials/04-hyperelastic/{00-neo-hookean,01-mooney-rivlin,02-ogden}.md` —
  for parallel constitutive laws if mixed u-p generalization needs them
- `20-materials/00-trait-hierarchy/{00-trait-surface,01-composition,02-validity}.md`

---

### L3 — Smoothed Coulomb friction on top of IPC

**Scope sketch**. `SmoothedCoulomb` friction model decorating IPC
contact. Per `40-contact/02-friction/01-smoothed.md`, replaces the
discontinuous stick/slip transition with a smoothed transition
over a small sliding-velocity scale $\epsilon_v$. Integrates
with IPC's barrier energy per `40-contact/02-friction/02-ipc-friction.md`.
Plugs into the BVH broadphase from L1.

**Estimated size**. ~500-1000 LOC. Smallest of the five legs.
Most of the implementation cost is in the IPC infrastructure L1
already shipped.

**Design-doc placeholder**: `docs/sim-research/L3_FRICTION_DESIGN.md`.

**Literature-benchmark gates**:

- **G3.1 Inclined-plane Coulomb test**. Soft block on rigid
  incline at angle $\theta$; static friction coefficient $\mu_s$.
  Block remains at rest for $\tan\theta < \mu_s$; slides for
  $\tan\theta > \mu_s$. Predicted slide-onset angle matches
  analytical $\theta_c = \arctan\mu_s$ to within 1°.
- **G3.2 Smoothed-vs-hard energy regime**. As $\epsilon_v \to 0$,
  the smoothed-Coulomb energy dissipation matches the hard-
  Coulomb (Stribeck) dissipation on a steady-sliding test. The
  smoothed model is consistent in the $\epsilon_v$ limit.
- **G3.3 Sock-on-probe sliding ramp regression**. Re-run the
  sliding-intruder canonical scene
  ([[project-sliding-intruder-contact-recon-shipped]]) with
  friction enabled. Per-step gripping force consistent with
  $|F_t| \le \mu |F_n|$ at every contact pair.

**Time horizon**. ~1 quarter.

**Design-book pointers**:
- `40-contact/02-friction/{00-coulomb,01-smoothed,02-ipc-friction}.md`
- `10-physical/02-what-goes-wrong/03-no-friction.md` — the
  pathology this leg cures

---

### L4 — Tet10 in contact band + stress-gradient h-refinement

**Scope sketch**. Two coupled deliverables per the book's
treatment of them as complementary
(`70-sdf-pipeline/03-adaptive-refine.md`: "[p- and h-refinement]
are complementary, not alternatives"):

- **Tet10 element**. Quadratic shape functions, 4-point Gauss,
  ~3-5× per-element FLOPs. New `Tet10` impl alongside `Tet4`.
  Midpoint-insertion post-pass converts Tet4 mesh to Tet10 in
  a contact-band selection.
- **Stress-gradient h-refinement**. Red-green subdivision of
  Tet4 elements (Bey 1995) between Newton solves. Trigger:
  stress-gradient-across-neighbors threshold on the **converged**
  iterate, per `30-discretization/02-adaptive/00-h-refinement.md`.
  Topology-preserving state transfer between refinement steps.
  **NOT contact-proximity-driven** — that primitive is rejected
  in `30-discretization/02-adaptive/02-contact-driven.md`.

**Estimated size**. ~3000-5500 LOC combined. Tet10 ~1500-2500 LOC;
h-refinement ~1500-3000 LOC.

**Design-doc placeholder**: `docs/sim-research/L4_TET10_HREF_DESIGN.md`
(may split into L4a + L4b once the work begins).

**Literature-benchmark gates**:

- **G4.1 Tet10 convergence rate**. Per
  `30-discretization/00-element-choice/04-tradeoff.md`, Tet10
  on a coarse mesh matches Tet4 on an ~8× finer mesh for
  bending-dominated problems. Cantilevered-beam test: Tet10 with
  N elements converges to the same tip deflection as Tet4 with
  8N elements, to within 2%.
- **G4.2 L-bracket reentrant-corner stress concentration**.
  Standard h-refinement validation problem. Stress field on a
  red-green-subdivided coarse mesh (3-4 refinement passes near
  the reentrant corner) matches a uniform fine-mesh reference
  within 5% at the corner singularity, with 10-20× fewer DOFs.
- **G4.3 Refinement topology invariants**. Red-green
  subdivision (Bey 1995) preserves: (a) tet quality bound
  (minimum dihedral angle), (b) mesh conformity (no T-junctions),
  (c) state-transfer continuity (no spurious energy jumps).
  Regression tests assert each invariant across the refinement
  history of the canonical scene.
- **G4.4 Stress-gradient trigger validity**. The trigger fires
  on converged iterates only (asserted in regression). Refining
  a non-converged iterate is documented as undefined behavior.

**Time horizon**. 1-2 quarters.

**Design-book pointers**:
- `30-discretization/00-element-choice/{00-tet4,01-tet10,03-mixed,04-tradeoff}.md`
- `30-discretization/02-adaptive/{00-h-refinement,01-p-refinement,02-contact-driven}.md`
  (the last as the **rejected** primitive — do not implement)
- `70-sdf-pipeline/03-adaptive-refine.md`
- `10-physical/02-what-goes-wrong/04-rim.md` — the pathology
  this leg cures

---

### L5 — Prony viscoelasticity

**Scope sketch**. `Prony<M>` decorator. Discrete relaxation
spectrum $\sigma(t) = \int_0^t G(t-\tau) \dot\epsilon(\tau)\,d\tau$
with $G(t) = G_\infty + \sum_i G_i e^{-t/\tau_i}$. Internal-
state-variable form (per
`20-materials/07-viscoelastic/00-prony.md`) avoids the history
integral. Time-dependent tangent stiffness per
`20-materials/07-viscoelastic/02-tangent.md`.

**Estimated size**. ~800-1500 LOC. `material/prony.rs` decorator
+ internal-state extension on the mesh state vector + tangent-
stiffness time-dependence threading.

**Design-doc placeholder**: `docs/sim-research/L5_PRONY_DESIGN.md`.

**Literature-benchmark gates**:

- **G5.1 Single-element step-strain relaxation**. Single Tet4
  element, step-strain input, N-term Prony series. Time-resolved
  stress response matches analytical $\sigma(t) = \epsilon_0
  (G_\infty + \sum_i G_i e^{-t/\tau_i})$ to 4 significant digits.
- **G5.2 DMA-derived Prony fit reproduction**. Per
  `10-physical/04-material-data/00-ecoflex/02-viscoelastic.md`
  and `20-materials/07-viscoelastic/03-dma.md`. Ecoflex 00-30
  DMA storage/loss modulus curves at the design frequency
  range (~0.1-10 Hz) reproduce within 10% of measured.
- **G5.3 (Physical-cast gate, deferred)**. Time-resolved
  indentation curve on a physical Ecoflex cast (measured via
  workshop-iter-N+) compared to sim prediction. **Tolerance
  driven by the 3-way validation harness's calibration**, not
  pre-specified here. This gate fires when cast data exists,
  not on a calendar.

**Time horizon**. ~1 quarter.

**Design-book pointers**:
- `20-materials/07-viscoelastic/{00-prony,01-oldroyd,02-tangent,03-dma}.md`
- `10-physical/04-material-data/00-ecoflex/02-viscoelastic.md`
- `10-physical/04-material-data/01-dragonskin/02-viscoelastic.md`

---

## 3. Validation cadence

### 3.1 Per-leg

Each leg ships with:

- A design doc citing the book chapter it descends from
- A literature-benchmark test suite (the G*.x gates above)
- A regression-suite extension on the canonical scene
- A retrospective comparison against the prior leg's output

The retrospective is important: **L2 must show that the L1-only
output on the canonical scene is unchanged** (mixed u-p is dormant
under `MixedUP<Yeoh>` when not opted into), preserving L1's
literature gates. Same pattern for L3 over L1+L2, etc.

The retrospective enforces the additivity contract from §1.3:
each leg extends without changing prior trait surfaces.

### 3.2 Cross-leg — the 3-way harness

When **L1 + L2 + L3 have shipped AND cast iter-1+ has happened**,
the 3-way validation harness goes live:

```
sim prediction          design intent (cf-device-design)
       \                  /
        physical cast measurement
```

The gap between any two pairs of these drives subsequent
prioritization:

- Sim vs cast gap → tells us which Phase-H legs to ship next
- Design-intent vs cast gap → tells us which cf-device-design
  geometry assumptions to revise
- Sim vs design-intent gap → tells us where the sim's modeling
  assumptions diverge from the geometry tool's

The gameplan §7.2 frames the promotion gate: **when 3-way
agreement falls within product-acceptable tolerance on the
canonical sleeve-on-probe scene, the sim is promoted from
research-mode-flag to product-default**. Not before; not on
a timeline; "when the data says so."

---

## 4. Open question — Phase E (GPU) sequencing

Gameplan §5.4 + audit §7 flag this:

> The book's Phase E (GPU port) sits between current state and
> Phase H. Open question: do we port the current penalty solver
> to GPU first (to make Phase H development tractable on bigger
> meshes), or skip straight to Phase H on CPU?

**Resolution**: bank as open until L1's first-quarter wall-time
tells us. Three reasons:

1. **GPU porting is itself a multi-quarter arc** per Phase E in
   `110-crate/03-build-order.md`. Committing to GPU before any
   Phase-H leg has measured wall-clock would be a
   multi-quarter-scope decision based on extrapolation, not
   data.
2. **IPC's wall-clock cost is unknown in our context**. Li 2020
   reports IPC scales well in their benchmarks, but those used
   different mesh sizes, different stencils, and different
   broadphase implementations. Until L1 runs on our canonical
   scene, the "wall-clock-bound vs not" question is unanswered.
3. **CPU is the spec's default through the design book**. Phase E
   is named as a Phase-E deliverable, not a Phase-H prerequisite.
   The book's "first working" milestone in
   `120-roadmap/02-first-working.md` ships on CPU.

**Trigger condition** to re-evaluate: after L1 ships, if the
canonical scene's per-step wall-clock exceeds **30 minutes**
(an arbitrary but useful threshold: long enough that a designer
iteration takes a full workday, short enough that automated
regression suites still close in CI), open a Phase-E spike
arc to estimate GPU port scope. If wall-clock stays below that,
defer Phase E to after L4 (when adaptive refinement may inflate
mesh sizes).

This decision is **reversible** — Phase E can land at any
inter-leg boundary. The order does not affect the legs'
correctness, only their wall-clock.

---

## 5. What's explicitly NOT on this roadmap

Per gameplan §6, these are paused or shelved:

- **H2 mesh-refinement implementation** as a tactical recon
  arc. The stress-gradient h-refinement work it would have done
  is folded into L4; the H2 bookmark + recon findings stand as
  audit trail.
- **Further N_STEPS / κ / Yeoh-validity-bound tuning** in the
  current FEM stack. Symptom-level levers exhausted.
- **F4 / F5 / Fx warmup arcs.** Same reasoning.
- **F3 LM extension to deeper cavities.** Same reasoning.
- **Tactical mesh-quality tweaks** to make cavity > 5 mm work
  in the current stack. Quote audit §3: this regime is the
  Phase-H-class problem the current stack was never built for.

What stays alive on the sim-research surface:

- Q1 (uniform 2 mm spike, background) — data feeds future
  L4 design doc
- F3 LM in dormant-default-off state — preserved as a
  measurement aid
- Existing recon docs — audit trail
- The current FEM stack — kept as research surface per
  gameplan's decoupling commitment (§2 / §3 A1)

Anything outside the 5 legs is **out of scope for sim-research
Phase H**. New ideas surface a new leg (L6+) or a back-port
into an existing leg, with a design doc and a gate.

---

## 6. Pointers

### Predecessors (read these for context)
- `docs/PROGRAM_GAMEPLAN.md` — the program reframe; §5 sketches
  this roadmap, §6 defines the paused set
- `docs/SIM_ARCHITECTURE_AUDIT.md` — foundational audit; §1
  enumerates book commitments, §3 maps pathologies to cures,
  §5 enumerates pivot options
- `docs/H2_RECON_FINDINGS.md` — Q3 pivot-position data + Q1
  (when complete) feeding L1 + L4 design-doc evidence base
- `MEMORY.md` — full project index

### Design book (sim-research spec)
- `docs/studies/soft_body_architecture/src/SUMMARY.md` — book
  index
- `docs/studies/soft_body_architecture/src/110-crate/03-build-order.md` —
  Phase ladder; Phase H is bundled additive deliverables
- Per-leg book cross-refs in each §2 section above

### Per-leg design-doc placeholders (lands when work begins)
- `docs/sim-research/L1_IPC_DESIGN.md`
- `docs/sim-research/L2_MIXED_UP_DESIGN.md`
- `docs/sim-research/L3_FRICTION_DESIGN.md`
- `docs/sim-research/L4_TET10_HREF_DESIGN.md`
- `docs/sim-research/L5_PRONY_DESIGN.md`

### Crate locations (research cluster, gameplan §7.1)
- `sim/L0/soft` — FEM solver, materials, contact, elements
  (where L1-L5 ship)
- `sim/L0/soft/src/sdf_bridge` — BCC + stuffing meshers (L4
  extends here)
- `mesh/mesh-sdf` — SDF oracle (no Phase-H change expected)

---

End of roadmap. Read this before opening any per-leg design doc.
If the roadmap is wrong — leg ordering shifts, a new pathology
surfaces, the 3-way harness produces unexpected data — update
this doc rather than silently routing work around it.
