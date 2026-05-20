# sim-soft / sdf_bridge / mesh-sdf — architectural audit

> **Authored 2026-05-19 LATE-NIGHT.** Foundational audit of the FEM
> stack, written during the H2 mesh-refinement recon wait while Q1
> (uniform 2 mm spike at cavity 6 mm) runs in the background.
>
> Scope, per user direction: FEM/sim stack only. **sim-soft** (solver
> + element + material + contact), **sdf_bridge** (BCC + Sommerville
> tets + stuffing), **mesh-sdf** (SDF oracle). cf-device-design as a
> consumer of these is in scope only at the boundary.
>
> Audit drivers:
> - Cascading tactical recons (H4-2-C, F3 LM, F4, H2) each patching
>   one symptom of a deeper architecture-fit issue.
> - H2's ~500 LOC implementation estimate revealed (in audit
>   [[project-h2-bcc-audit]]) as off by 4-5× because the existing BCC
>   mesher's Sommerville-tet quality invariants don't admit variable
>   cell size sub-routinely.
> - Each recon arc has shipped real value but hasn't moved the
>   underlying failure surface.
>
> **TL;DR.** The current stall is *not a tactical bug*. It's the
> book-predicted failure mode of running **Tet4 + near-incompressible
> Yeoh + penalty contact + uniform-coarse BCC mesh** in a regime that
> combination was explicitly architected to be removed at Phase H by
> a **{Tet10-in-band | F-bar | IPC | stress-gradient adaptive
> refinement}** bundle. We're shipping Phase-C-without-IPC FEM
> against a Phase-H-class problem. The four "Phase H fidelity
> upgrades" the book specifies cure all four symptoms in one
> coordinated wave; each tactical recon arc cures one symptom
> partially in isolation.
>
> Recommendation: pause the tactical-recon thread; commit to a
> ~3-month Phase-H arc OR narrow the user-facing problem to the
> regime the current stack handles. **Both are reasonable; the choice
> is product/timeline, not technical.**

---

## 0. Reading guide

- §1 — what the design book (`docs/studies/soft_body_architecture`)
  says the production architecture is.
- §2 — what's actually shipped, organized by the book's phase ladder.
- §3 — the FEM-stall pathology mapped onto the book's failure-mode
  catalog.
- §4 — layer inventory with lifecycle stage + load-bearing role.
- §5 — concrete pivot options with cost/benefit + risk profile.
- §6 — the asks (decisions the user makes; what the assistant drives).

This memo is cold-readable. It assumes familiarity with the recon
arc landscape (the [[MEMORY.md]] index) but not with any individual
recon doc.

---

## 1. What the design book commits to

The `docs/studies/soft_body_architecture/` mdbook is the architectural
source of truth — 100+ leaf chapters specifying the production target.
The most load-bearing claims for this audit:

### 1.1 The build-order ladder (`110-crate/03-build-order.md`)

| Phase | Deliverable | Status |
|------:|---|:---:|
| A | `material/` + `mesh/` standalone math (Tet4-only mesh, NH baseline) | ✓ shipped |
| B | `element/` + `solver/` (Tet4 + backward-Euler Newton + faer sparse Cholesky, no contact) | ✓ shipped |
| **C** | **`contact/` = IPC barrier + CCD + smoothed Coulomb friction + BVH** | ✗ **PENALTY shipped instead of IPC** |
| D | `readout/` + ml-chassis boundary ("**first working sim-soft**") | ✓ shipped (under penalty, not IPC) |
| E | GPU port of solver hot paths via wgpu | ✗ not started |
| F | `coupling/` — sim-core + sim-thermostat boundaries | ✗ not started |
| G | `sdf_bridge/` — SDF→tet pipeline + live remesh | partial: BCC + Sommerville stuffing shipped; **fTetWild path not shipped**; live-remesh not shipped |
| **H** | **Tet10 + mixed u-p / F-bar + Prony viscoelasticity + HGO + stress-gradient adaptive refinement** | ✗ **not started** |
| I | `sim-bevy` visual layer | partial (cf-device-design viz coupling) |

The "first working" milestone (end of Phase D) is supposed to ship
**IPC, not penalty**. Penalty is named in `40-contact/00-why-ipc/00-penalty.md`
as a "deliberate, named-removed-at-Phase-H stepping stone for exactly
the validation scope §00 §00 names: rigid↔soft co-sim plumbing,
first-time `ContactModel` wiring into the Newton hot path, and an
analytic-comparison scientific gate (Hertz)."

### 1.2 The four Phase-H fidelity upgrades

`110-crate/03-build-order.md` Phase H roadmap:

> **Tet10 elements, mixed u-p incompressibility (Mooney-Rivlin,
> Ogden), Prony viscoelasticity, HGO anisotropy, adaptive
> refinement driven by stress feedback. Each extends `material/`
> or `element/` without changing the trait surface from Ch 01, so
> they are additive.**

The four bundled together is not accidental — each addresses one
named pathology in `10-physical/02-what-goes-wrong/`:

| Phase-H deliverable | Pathology it cures (book reference) |
|---|---|
| F-bar / mixed u-p | Volumetric locking (`20-materials/05-incompressibility/00-locking.md`) |
| IPC barrier (already named in Phase C) | Newton oscillation at penalty discontinuity (`40-contact/00-why-ipc/00-penalty.md`) |
| Tet10 in contact band | Rim-deformation failure (`10-physical/02-what-goes-wrong/04-rim.md`) |
| Stress-gradient adaptive refinement | Same rim failure + internal stress concentrations (`70-sdf-pipeline/03-adaptive-refine.md`) |

### 1.3 Penalty's explicit failure modes (`40-contact/00-why-ipc/00-penalty.md`)

The book is direct:

> The Hessian contribution from a contact pair is therefore
> discontinuous across the contact/no-contact boundary. **This one
> fact is every downstream failure.**
>
> ... The Newton iteration sees two different linear systems at
> two consecutive iterates and oscillates without converging.
> Line search cannot fix this because the Newton direction itself
> is computed with a Hessian that is wrong on the other side of
> the boundary.

This is *exactly* the persistent-pivot stagnation pattern the H4 / F3
recons spent weeks isolating. The book documented this in 2026-04
or earlier; the recon arc rediscovered it empirically through
H4-2-C falsification + F3 LM falsification + F4 falsification.

### 1.4 Volumetric locking under Tet4 (`20-materials/05-incompressibility/00-locking.md`)

> The Lamé pair at ν → 0.5 has λ = Eν / [(1+ν)(1−2ν)] diverging
> while μ stays bounded. For silicone at ν = 0.499, λ/μ ≈ 500 —
> the volumetric-penalty stiffness dominates the deviatoric-shear
> stiffness by three orders of magnitude.
>
> ... The twelve-DOF Tet4 cannot, in general, configure itself so
> that tr ε = 0 holds per tet while the deviatoric modes match an
> arbitrary boundary condition. ... the deviatoric modes (the
> bending, the isochoric shape changes) are suppressed to keep the
> dominant volumetric constraints nearly met.

The H4 arc invented "asymmetric one-sided bound" to keep Yeoh from
panicking when the compressive principal stretch crossed the
validity gate. That's a *symptom-management* fix: the locked
deformation itself is what produces the unphysical compressive
stretches that the validity gate then trips on. The actual cure
the book names is **F-bar wrapping the Yeoh material**, which
relaxes the per-tet volumetric constraint to per-patch averaging.

### 1.5 Adaptive refinement is stress-gradient-driven, not
proximity-driven (`30-discretization/02-adaptive/02-contact-driven.md`)

The H2 bookmark was about to spend a multi-session arc on
distance-banded (cavity-proximity) refinement. The book *explicitly
rejects* this as a primary trigger:

> Contact-driven refinement is rejected as `sim-soft`'s primary
> refinement trigger.
> ... over-refine on glancing contact, under-refine on internal
> stress concentration. Both are workload-relevant for the
> canonical problem and not theoretically resolvable within the
> contact-proximity criterion family.

The book's adaptive primitive uses **stress-gradient-across-neighbors
on the converged Newton iterate**, refined via **red-green
subdivision (Bey 1995)** between solves. Topology-preserving,
state-transferable, bounded quality loss. *Not* the distance-banded
BCC-cell-size approach H2 was about to build.

### 1.6 Material model commitment (`20-materials/04-hyperelastic.md` family)

The book's hyperelastic family: **Neo-Hookean baseline + Mooney-Rivlin /
Ogden as Phase-H additions, all wrappable with `FBar<M>` or
`MixedUP<M>`.** Yeoh is *not in the book* — it was added to sim-soft
during the cf-device-design arc because the silicone material data
(Ecoflex 00-30, DS 20A) demanded a curve-fit form richer than
Neo-Hookean.

This is fine in principle (per the trait-surface composition rules,
new materials extend `Material` without changing the surface), but
it means the F-bar / mixed-u-p decorators *don't exist yet* —
`FBar<Yeoh>` is not implemented because `FBar` itself isn't
implemented. The Yeoh arc shipped material breadth ahead of the
book's planned material depth.

---

## 2. What's actually shipped — by phase

### Phase A — material + mesh ✓

- `material/` — Neo-Hookean + Yeoh + multi-material composition via
  `MaterialField` (Yeoh is ahead of book's NH-only baseline; F-bar /
  mixed-u-p / Mooney-Rivlin / Ogden / viscoelasticity not shipped)
- `mesh/` — Tet4 storage + adjacency + quality metrics

### Phase B — element + solver ✓

- `element/` — Tet4 + 1-point Gauss (Tet10 banked per Phase H)
- `solver/` — Backward Euler + Newton + faer Llt-first + LU fallback
- F3 LM regularization (Marquardt λ retry) shipped on top of the
  Phase-B surface — this is a *Phase-D-late* feature, added during
  the F3 recon arc, *not* a Phase-B deliverable. Off by default; the
  bit-equal-when-dormant contract preserves Phase-B numerics.

### Phase C — contact ✗ (penalty shipped instead of IPC)

- `contact/penalty.rs` — 868 LOC, full penalty implementation
- `contact/rigid.rs` — RigidPlane primitive
- `contact/null.rs` — null contact (the Phase-B compile gate)
- **No IPC.** No barrier function, no CCD, no Coulomb friction
  decorator, no BVH broadphase.

This is the most consequential book divergence. The decision to ship
penalty was deliberate (per the `lib.rs` doc-comment cited above —
"penalty is a stepping stone, a future contact upgrade replaces
penalty as the production baseline") but the upgrade hasn't happened.

### Phase D — readout + chassis ✓ (under penalty, not IPC)

- `readout/` — stress extraction, contact-pair readouts, reward
  breakdown, gradient estimate (with the documented
  `GradientEstimate::Noisy { variance }` shape for mesh-topology
  events)
- ml-chassis coupling closed
- 5-digit FD gradcheck passing on the penalty path (per
  `tests/contact_grad_hook.rs`)

The "first working" milestone shipped on the penalty stepping stone.
Every reward gradient computed since rests on penalty's piecewise-C²
total-potential surface, with the documented gradient-validity
restriction to robustly-in-contact / robustly-out-of-contact bands.

### Phase E — GPU ✗

Not started. Solver runs CPU-single-threaded via faer (rayon
explicitly disabled for determinism). Workshop iter-1 wall-clock
budgets are growing — the H4 + F3 recons hit 30-60 min cells, Q1's
2-mm spike is 1-6 hr.

### Phase F — coupling ✗

cf-design integration *does* exist (sim-soft consumes `cf_design::Sdf`)
but sim-core / sim-mjcf / sim-thermostat boundaries are not wired.
Rigid-body co-sim is not on the path; the cf-device-design arc has
been operating without it. Likely fine for the current product;
relevant only if soft-rigid coupling becomes a workshop requirement.

### Phase G — SDF bridge ✓ partial

- `sdf_bridge/` — BCC + Sommerville-tet stuffing pure-Rust path
  shipped (`lattice.rs` 789 LOC, `stuffing.rs` 1708 LOC,
  `sdf_meshed_tet_mesh.rs` 354 LOC)
- **fTetWild path is not shipped** — the book's Phase-H+ target
  generator (`70-sdf-pipeline/01-tet-strategies/00-ftetwild.md`)
- **Live re-mesh + warm-start across topology changes is not
  shipped** — `70-sdf-pipeline/04-live-remesh.md`. cf-device-design
  rebuilds-from-scratch on each design-edit event.

The BCC + stuffing path is a Phase-G *placeholder* per the book's
phrasing in `lib.rs`:

> pure-Rust SDF→tet bridge via BCC plus Labelle-Shewchuk Isosurface
> Stuffing (Phase 3) ...

Phase 3 is sim-soft-internal phase numbering, distinct from the
book's Phase G. The placeholder works for the cf-device-design
workload as long as the mesh quality is good enough — which is
exactly what the H2 audit revealed it *isn't* at cavity > 5 mm.

### Phase H — fidelity upgrades ✗

All four bundled deliverables are unshipped:
- ✗ Tet10
- ✗ F-bar / mixed u-p / Mooney-Rivlin / Ogden / viscoelasticity / HGO
- ✗ IPC barrier (also named in Phase C and missing there too)
- ✗ Stress-gradient adaptive refinement

### Phase I — visual ✗ in-crate; ✓ via cf-device-design

sim-soft's `viz/` (4034 LOC) ships boundary surface + slab cut +
design-scene helpers for AttributedMesh consumers. cf-device-design
runs the Bevy + egui visual layer on top. The book's `sim-bevy` /
SSS / micro-wrinkle visual layer is not shipped, but the
cf-device-design viz scope is enough for the workshop workflow.

---

## 3. The FEM-stall pathology mapped to the book

Every observed pathology in the H4 / F3 / F4 / H2 recon arc maps
directly onto a documented failure mode in `10-physical/02-what-goes-wrong/`
or its referencing sub-leaves.

| Observed pathology | Book reference | Documented cure |
|---|---|---|
| **Persistent-indefinite-pivot Newton stall** at specific vertices, residual floor at ~0.5 (terminal r_norm 0.536 at cavity 6 mm N=16, dominated by v5114 57× consecutive iters per H2 Q3) | `40-contact/00-why-ipc/00-penalty.md` — "Newton iteration sees two different linear systems at two consecutive iterates and oscillates" | **IPC barrier** ($C^2$, no discontinuity) |
| **Yeoh validity panic** at compressive principal stretches under deep cavity loading; cured palliatively by H4-2-C "asymmetric one-sided bound" dropping the compressive gate | `20-materials/05-incompressibility/00-locking.md` — Tet4 over-constrained system trades deviatoric residual against per-tet volumetric constraint; unphysical compressive stretches are *the locking signature* | **F-bar `<Yeoh>`** patch-averages $\bar J$ to one constraint per patch, allowing Tet4 to satisfy both volumetric + deviatoric without unphysical strain spikes |
| **N_STEPS resonance** — N=12 covers cavity 5 mm, N=20 covers cavity 6+7 mm, no single N covers all 5; per-cavity tuning is scan-specific tactical | `40-contact/00-why-ipc/00-penalty.md` — penalty's stiffness-vs-stability tradeoff: "increasing κ_pen ... buys either smaller timesteps (explicit) or more inner iterations per Newton step (implicit) ... at finite κ_pen there remains a finite penetration depth at any finite load" | **IPC's adaptive $\kappa$ schedule** (`01-adaptive-kappa.md`) — barrier stiffness grows only when gap-closure is observed, warm-starts across timesteps |
| **Rim-band stress concentration** at the cavity wall near intruder slide-pose tip (H2 Q3: v5114, v6645 cluster in lower-body z-band r=29–36 mm) | `10-physical/02-what-goes-wrong/04-rim.md` (referenced by `30-discretization/02-adaptive/02-contact-driven.md` failure mode 1) | **Tet10 in the contact band + stress-gradient h-refinement** (red-green subdivision, Bey 1995) |
| **Uniform-cell-size mesh strain at workshop scale** — 72k tets at 4 mm with 2 vertices owning 69 % of pivot fires; Q1's 580k tets at 2 mm is 1-6 hr wall-clock | `70-sdf-pipeline/03-adaptive-refine.md` — uniform refinement explicitly rejected: "8× larger than adaptive refinement for the same worst-case accuracy" | **Stress-gradient adaptive h-refinement only on converged Newton iterates**, NOT contact-proximity (rejected as failure-mode-1 trigger) |
| **F3 LM saturation at cavity > 5 mm** — Newton extends to 57 iters under LM, then re-stalls at the *same* r_norm floor (Gate B 1.784 → 1.784 across LM seeding); the indefinite eigenmode is κ-invariant | `20-materials/05-incompressibility/00-locking.md` — locked-system over-constraint is *geometric*, not stiffness-related: "the minimizer is a configuration where neither set is fully satisfied" | F-bar relaxes the constraint scale; LM regularizes the *symptom*, not the *cause* |

**Pattern**: every tactical fix in the recon arc has been correctly
diagnosed-and-palliated *one symptom at a time*, while the
book-documented cure is *one coordinated Phase-H wave* that removes
all the symptoms by changing the underlying formulation.

H4-2-C added Yeoh asymmetric bound — palliates locking, doesn't cure.
F3 LM added Marquardt regularization — palliates indefinite-tangent
from contact discontinuity, doesn't cure. F4 added warmup against
contact-onset — palliates same, doesn't cure. H2 was about to add
distance-banded BCC refinement — but the book explicitly rejects
contact-proximity refinement as the primary trigger.

Each recon arc *was* the right thing to ship for its scope —
empirical falsification of the wrong mental model is itself
high-information per [[feedback-implement-measure-revert-pattern]],
and the F-bar / IPC / Tet10 / adaptive-h-refinement bundle is too
big to ship in one move. But the *cumulative pattern* of "tactical
recon arcs that each refine the diagnosis without moving the
underlying failure surface" is the architectural-fit signal this
audit was asked to surface.

---

## 4. Layer inventory + lifecycle

For each layer of the FEM stack, where it sits on the
placeholder → production → research-grade axis.

### 4.1 `material/`

- **Trait surface**: production-grade. `Material` + `MaterialField` +
  `ValidityDomain` + composition rules.
- **NeoHookean impl**: production-grade.
- **Yeoh impl**: production-grade *as a constitutive law*. Validity-domain
  gates are well-defined; the H4-2-C asymmetric-bound surface is a
  workaround for locking-induced strain pathologies, not a Yeoh-specific
  bug.
- **Mooney-Rivlin / Ogden / Prony viscoelasticity / HGO**: not shipped.
- **F-bar / mixed u-p / TVF decorators**: not shipped. Most load-bearing
  Phase-H gap — wraps any `<M: Material>`.

Lifecycle stage: **production-grade for what's shipped; Phase-H
decorators completely absent.**

### 4.2 `element/`

- **Tet4**: production-grade. 31 LOC trait + 44 LOC impl, well-tested.
- **Tet10**: not shipped. Required for in-band rim fidelity.
- **Hex8**: not shipped (and explicitly *rejected* in the book —
  `30-discretization/00-element-choice/02-hex8.md` cites "no robust
  SDF → hex meshing exists").

Lifecycle stage: **Tet4 is production-grade but is structurally the
bottom of the book's element ladder.** Tet10 is the in-band upgrade
the book commits to.

### 4.3 `mesh/`

- **HandBuiltTetMesh / SingleTetMesh**: production-grade for
  regression-net scenes. Phase-B baseline.
- **Mesh trait + adjacency + quality metrics**: production-grade.

Lifecycle stage: **production-grade.** Phase-G `SdfMeshedTetMesh`
wraps this surface; nothing in `mesh/` needs to change for Phase H.

### 4.4 `contact/`

- **NullContact**: production-grade compile-gate.
- **PenaltyRigidContact**: 868 LOC, production-grade *as a penalty
  implementation*, but penalty is structurally wrong for sim-soft per
  `40-contact/00-why-ipc/00-penalty.md`.
- **IPC**: NOT SHIPPED. The book's production contact baseline.

Lifecycle stage: **the "stepping stone" — `lib.rs` explicitly names
penalty as named-removed-at-Phase-H.** The most load-bearing Phase-C
gap in the entire stack. Every current Newton convergence pathology
the recon arc has been chasing traces to penalty's discontinuous
Hessian. **This is the single highest-leverage replacement.**

### 4.5 `solver/`

- **CpuNewtonSolver**: production-grade Newton + Armijo backtrack +
  faer Cholesky + LU fallback.
- **F3 LM regularization**: production-grade *for what it does*, but
  what it does is palliate penalty's discontinuity-induced indefinite
  tangent. Disabled-by-default preserves bit-equal Phase-B numerics.
- **`try_step` / `SolverFailure` graceful-failure surface**: shipped
  during F3.3, production-grade.
- **Implicit-function adjoint via faer factor-on-tape**: production-grade.
- **GPU port**: not shipped (Phase E).

Lifecycle stage: **production-grade.** The solver is well-architected;
its convergence pathologies trace to penalty (Phase C) and locking
(Phase H material decorators), not to anything wrong in the solver
itself. *Solver-side changes have hit diminishing returns.*

### 4.6 `sdf_bridge/`

- **BCC + Sommerville-tet stuffing**: 3309 LOC, production-grade
  *as a uniform-cell-size pure-Rust mesher*. Closed-form vertex IDs,
  Sommerville-tet quality invariants (edge lengths exactly
  cell_size + cell_size·√3/2; dihedrals exactly 60° + 90°), passes
  quality unit tests.
- **mesh-sdf parry-accel**: production-grade, shipped 2026-05-17.
- **Adaptive BCC mesher with variable cell size**: not shipped, and
  per the H2 audit, **not a small extension** — would require
  T-junction handling or conforming refinement breaking the Sommerville
  quality invariants. Research-grade scope.
- **fTetWild path**: not shipped.
- **Live remesh + warm-start across topology changes**: not shipped.

Lifecycle stage: **placeholder per `lib.rs`'s own framing
("Phase 3 placeholder mesher").** Production-grade for what it does;
the gap is the *generator capability*, not the implementation
quality.

### 4.7 `readout/` + `differentiable/` + `viz/`

- **readout/**: production-grade. Stress field, contact-pair readout,
  reward breakdown, gradient estimate with mesh-topology-event noise
  attribution.
- **differentiable/**: production-grade. CpuTape + NewtonStepVjp +
  IFT adjoint.
- **viz/**: production-grade for cf-device-design's needs.

Lifecycle stage: **production-grade.** These layers don't need
Phase-H upgrades; they consume whatever the solver/contact pair
produces.

### 4.8 Summary

| Layer | Lifecycle | Phase-H gap |
|---|---|---|
| `material/` | production | F-bar/mixed-u-p decorators, viscoelasticity, anisotropy |
| `element/` | production-grade Tet4; Tet10 absent | Tet10 in contact band |
| `mesh/` | production | none |
| `contact/` | **STEPPING STONE — penalty must be replaced** | IPC + CCD + Coulomb friction + BVH |
| `solver/` | production | none (GPU port is Phase E, orthogonal) |
| `sdf_bridge/` | placeholder (Phase 3 framing) | stress-gradient h-refinement + fTetWild + live remesh |
| `readout/` + `differentiable/` + `viz/` | production | none |

**The two highest-leverage gaps are `contact/` (IPC replacing
penalty) and `sdf_bridge/` (stress-gradient adaptive h-refinement
with red-green subdivision).** Both are Phase-H deliverables that
the book bundles together because they cure interacting pathologies.

`material/`'s F-bar / mixed-u-p decorator is the third Phase-H leg.
It's smaller in surface area than IPC (per the book's "no extra
unknowns" framing in `20-materials/05-incompressibility/02-f-bar.md`)
and is conceptually independent of the contact + refinement work.

`element/`'s Tet10 is the fourth Phase-H leg. The book treats it as
**complementary, not alternative, to h-refinement** —
`70-sdf-pipeline/03-adaptive-refine.md`: "[p- and h-refinement] are
complementary, not alternatives — the canonical problem benefits
from both."

---

## 5. Pivot options

Three categorically-distinct paths forward, each with its own risk
profile and time horizon.

### 5.1 Option A — commit to Phase H

Build the four-deliverable bundle the book commits to, in some
sensible sub-order:

**A1. F-bar `<Yeoh>` decorator (4-8 weeks?)**
- Wraps `Material` trait surface; no extra unknowns
- Cures locking up to ν ≤ 0.49 (covers Ecoflex-class silicone but
  not DS20A's near-incompressible regime)
- Smallest surface of the four Phase-H legs
- Validity-domain composition rules per `20-materials/00-trait-hierarchy/02-validity.md`
- **Estimate, gut**: 600-1200 LOC, 2-3 sessions, low conceptual risk
- Per the book's framing: "the cheap Tet4 hack"
- *Mixed u-p* is the sibling that covers ν → 0.499 properly;
  bigger surface (saddle-point solve, Schur complement, additional
  pressure field), but the book commits to it for production

**A2. IPC barrier replacing penalty (8-16 weeks?)**
- Full Li 2020 barrier formulation: $C^2$ smooth, compact support
- Adaptive $\kappa$ schedule (`40-contact/01-ipc-internals/01-adaptive-kappa.md`)
- CCD filter to prevent step-over-into-penetration
- BVH broadphase (`40-contact/03-self-contact/00-bvh.md`)
- Smoothed Coulomb friction decorator (`40-contact/02-friction/`)
- **Estimate, gut**: 3000-6000 LOC, 8-12 sessions, high conceptual risk
- Single biggest item in Phase H. Could be its own multi-month arc.
- Reference: IPC Toolkit (C++, not directly usable but the math is
  documented chapter by chapter in the book)

**A3. Stress-gradient adaptive h-refinement (4-8 weeks?)**
- Red-green subdivision (Bey 1995) of Tet4 mesh between solves
- Stress-gradient-across-neighbors trigger criterion
- State transfer + warm-start through subdivision
- Quality-floor abort backstop
- **Estimate, gut**: 1500-3000 LOC, 4-8 sessions, medium conceptual risk
- Independent of IPC + F-bar; can ship alongside

**A4. Tet10 in contact band (4-8 weeks?)**
- Quadratic shape functions; 4-point Gauss; ~3-5× per-element FLOPs
- New `Tet10` element impl; mesh-to-Tet10 midpoint-insertion post-pass
- Phase-D regression suite extends with Tet10 baselines
- **Estimate, gut**: 1500-2500 LOC, 4-6 sessions, medium conceptual risk
- The book treats this as *complementary* to A3 (h-refinement) —
  Tet10 for in-band fidelity, h-refinement for stress-gradient zones

**Total Phase H scope**: order-of-12-30 sessions; probably 3-6 months
of focused work. Each leg is additive (the book's "extends material/
or element/ without changing the trait surface" framing).

**Sub-order rationale**: A1 first (smallest, cures the locking-induced
Yeoh panics that the H4 arc has been palliating). A2 second (biggest;
cures every penalty-discontinuity pathology). A3 third (cures rim-
band-resolution AFTER the locking + contact pathologies are out of
the way — refining a locked mesh under penalty would just multiply
the failure modes). A4 fourth (in-band fidelity upgrade on top of
the refined mesh).

**Risk**: 3-6 months of architecture work before the cf-device-design
arc gains workshop-iter-2 capability. Workshop iter-1 cast (per
[[project-v2-1-cf-cast-arc-complete]]) is still on the path; physical
validation against the current FEM is the next inflection point.
**A Phase-H commitment defers physical-cast iteration.**

### 5.2 Option B — narrow the user-facing problem

Restrict cf-device-design's user surface to the regime the current
stack handles cleanly. Specifically:

- Cap the cavity_inset slider at 3-5 mm (the H4 arc envelope)
- Document the constraint in the UI + the prep.toml
- Accept that workshop-iter-2 may want cavity > 5 mm and revisit
  Phase H then
- Ship the cf-device-design iter-1 workflow with the current FEM;
  measure physical-cast outcomes; let workshop iteration drive the
  next Phase-H commitment

**Scope**: 1-2 days of UI work + memo writing.

**Risk**: low. The physical-cast iter-1 is the gate; if iter-1
reveals that cavity > 5 mm is a workshop must-have, we know Phase H
is unavoidable and can budget for it.

**Trade-off**: deferred fidelity. We're shipping a tool whose
recommended-use-band is narrower than its theoretical-use-band.
Some users may want to push the slider past 5 mm; the tool will
honestly say "not validated in this regime."

This is the option [[feedback-strip-the-knob-when-default-works]]
pattern would endorse — *if* the cf-device-design product needs
cavity ≤ 5 mm only, strip the rest of the slider range. The user's
verbal feedback during the workshop-iter-1 design ([[project-cf-device-design-cavity-pinned-floor-redesign-spec]])
was about cavity *floors*, not deep cavities.

### 5.3 Option C — pivot to external solver

Drop sim-soft FEM, use FEBio / PolyFEM / MuJoCo flex as the simulator
backend. sim-soft becomes a thin wrapper that ingests SDF + materials
+ contact spec and forwards to the external solver.

**Pros**:
- Production-grade IPC + Tet10 + F-bar / mixed-u-p available today
- The current 30+ session recon arc would have been unnecessary
- Frees the architecture team to focus on cf-device-design product
  surface, not FEM internals

**Cons**:
- Loses the book's load-bearing differentiability (none of the named
  external solvers ship a 5-digit reverse-mode gradient through
  contact)
- Loses the GPU port path (Phase E)
- Loses the live-remesh + warm-start (Phase G)
- C++ build-graph dependency (FEBio) or Julia (PolyFEM) breaks the
  pure-Rust commitment
- Tooling cost — sim-soft has 18k LOC of trait surfaces, regression
  tests, gradcheck infrastructure — none of it directly portable

**Estimate**: 4-8 weeks of integration work just to reach feature
parity with the current penalty path; longer to add the
differentiability sim-soft already has.

**Risk**: this is a *strategic pivot*, not a *tactical decision*.
The book exists because sim-soft's differentiable in-house FEM is
load-bearing for the optimization loop ([Part 10 — full design-print-rate loop](../docs/studies/soft_body_architecture/src/100-optimization/06-full-loop.md)).
External solvers don't support that.

**Verdict**: Option C is on the table for completeness but is the
weakest path absent a hard strategic re-prioritization. Not
recommending it.

### 5.4 Option D — hybrid: ship Option B now, queue A1 + A3 for the
post-cast-iter-1 window

A pragmatic middle path:

1. **This week**: ship Option B's cavity-slider cap (1-2 days).
   Releases workshop-iter-1 physical-cast unblocked.
2. **Q1 (3-6 months)**: physical-cast iter-1 in the workshop;
   collect real measurements; revise the FEM spec based on cast
   outcomes.
3. **Q2 onward**: commit to A1 (F-bar `<Yeoh>`) + A3 (stress-gradient
   h-refinement) as the *first half* of Phase H. Defer A2 (IPC) and
   A4 (Tet10) until the cast-iter-1 measurements say we need them.

**Estimate**: Option-B work + ~3-6 months for A1 + A3 sub-arc.

**Risk**: depends on whether cast-iter-1 reveals issues the current
FEM stack can't model AT ALL (vs issues it models inaccurately).
Inaccurate-but-shipping is the OK state; can't-model-at-all is the
bad state.

**Recommendation**: this is the path I'd bet on. Workshop-iter-1
physical-cast is the *real* validation gate; spending months on FEM
architecture before that gate fires is premature optimization.

---

## 6. The asks

### 6.1 Decisions for the user

**D1**. Pick a pivot path: A (full Phase H), B (narrow problem),
C (external solver), or D (hybrid: narrow now, A1+A3 later).

If D is the choice: confirm the cavity-slider cap value (5 mm? 4 mm?)
and the post-cast-iter-1 trigger condition for A1+A3.

If A is the choice: confirm the sub-order (A1 → A2 → A3 → A4 is the
recommended one), and which of A1's two variants (F-bar vs mixed
u-p) to ship first.

**D2**. Decide what to do with H2 specifically.
- *Continue* with H2's distance-banded BCC refinement is **rejected
  by the book** as a primary refinement trigger. Even if Q1 confirms
  uniform 2 mm fixes the stall, the distance-banded approach H2
  would have built is the wrong design.
- *Abandon* H2 and revert the diagnostic instrumentation. Q1 still
  runs to completion as data.
- *Repurpose* H2's recon outputs as the seed for a future A3
  (stress-gradient h-refinement) arc. The H2 Q3 data (v5114 + v6645
  dominate at the cavity-wall band) is the *prerequisite measurement*
  A3 would consume.

**D3**. Decide what to do with the existing recon-arc commitments
(F4 already-falsified, F3 already-shipped-but-Gate-B-stalled, H4
already-shipped-with-cavity-6+7-gap). The recon-arc-pattern of "one
session for bookmark + one for recon + one for implementation"
([[feedback-bookmark-when-surface-levers-exhaust]]) is *correctly
sized* for the symptom-level work but is *under-sized* for the
architectural pivots the book commits to. **Phase H deliverables
are 8-12 sessions, not 3.**

### 6.2 Drives for the assistant

If D1 = B or D: ship the cavity-slider cap as a small bookmark +
implementation arc (2 sessions max), document in the cf-device-design
UI + memo the constraint, and continue the cf-device-design workshop
arc per the existing roadmap.

If D1 = A or D's-A1-prefix: open a fresh design doc for the chosen
sub-order; commit-by-commit log of decisions; expect 8-12 sessions
per Phase-H leg. The book is the spec — every leg's design doc
should cite the book chapter it descends from.

If D1 = C: this is a strategic decision; the assistant doesn't drive
it without explicit re-authorization.

In ALL cases:
- The H2 diagnostic instrumentation in `sim/L0/soft/src/solver/backward_euler.rs`
  should be reverted before any non-recon commit lands (search
  `H2-Q3 DIAG` for the sites).
- The `h2_recon_cavity_6mm_n16` test should be either deleted or
  refactored into a regression sentinel.
- Q1's run completes regardless; its data goes into the recon
  findings doc (`docs/H2_RECON_FINDINGS.md`) as audit trail.

---

## 7. Open questions for future audit passes

Not blocking the D1 decision, but worth noting:

- **Should sim-soft adopt Mooney-Rivlin or Ogden as the primary
  hyperelastic baseline instead of (or alongside) Yeoh?** The book
  commits to MR/Ogden as Phase H; Yeoh is sim-soft-specific. If we
  build F-bar / mixed u-p anyway, the question of which constitutive
  law it wraps becomes design-space-relevant.
- **Is Phase E (GPU port) on the path before Phase H (fidelity)?**
  The book's order is E before H, but the cf-device-design workload
  is wall-clock-bound today and Phase H's compute cost will be worse
  before it's better. A GPU port of the current penalty solver might
  unblock more workshop iteration than Phase H's fidelity upgrades.
- **What's the cf-device-design product surface in 6-12 months?**
  This audit assumes workshop-iter-1 is the next gate; iter-2 might
  pivot the product enough that the FEM requirements change.
- **Is sim-soft's autograd / chassis coupling load-bearing for
  cf-device-design?** If cf-device-design never invokes the 5-digit
  reverse-mode gradient (and currently it doesn't — there's no
  optimization loop wired through the UI), the differentiability
  argument for in-house FEM (Option C's main blocker) weakens.

---

## 8. Pointers

- `docs/studies/soft_body_architecture/src/SUMMARY.md` — book index
- `docs/studies/soft_body_architecture/src/110-crate/03-build-order.md` — Phase ladder
- `docs/studies/soft_body_architecture/src/40-contact/00-why-ipc/00-penalty.md` — why penalty must go
- `docs/studies/soft_body_architecture/src/20-materials/05-incompressibility/{00-locking,01-mixed-up,02-f-bar}.md` — locking + cures
- `docs/studies/soft_body_architecture/src/30-discretization/00-element-choice/{00-tet4,01-tet10,04-tradeoff}.md` — element commitment
- `docs/studies/soft_body_architecture/src/70-sdf-pipeline/{01-tet-strategies,03-adaptive-refine}.md` — refinement strategy
- `sim/L0/soft/src/lib.rs` — actual public surface
- `docs/H2_RECON_FINDINGS.md` — Q3 data + Q1 (pending) data feeding this audit
- `docs/H2_MESH_REFINEMENT_BOOKMARK.md` — the recon bookmark that triggered this audit

---

End of audit. Awaiting D1/D2/D3.
