# Program gameplan — sim is research, product pipeline ships

> **Status**: ACTIVE, set 2026-05-19 LATE-NIGHT after the
> H2 recon arc audit revealed that the FEM tactical-recon cadence
> has been treating research-scale problems as product blockers.
> This doc supersedes the implicit "recon → fix" cadence of the
> last several months. It is the program-level reframe.
>
> **Predecessors**:
> - `docs/SIM_ARCHITECTURE_AUDIT.md` — the foundational audit that
>   surfaced the architectural-fit gap
> - `docs/H2_RECON_FINDINGS.md` — Q3 data showing the persistent-
>   pivot stagnation pattern is book-predicted (`40-contact/00-why-ipc/00-penalty.md`)
> - `docs/studies/soft_body_architecture/` — the design book; the
>   sim-research spec
>
> This doc is cold-readable. Future sessions should READ THIS FIRST
> before resuming any recon arc or sim work.

---

## TL;DR

Two parallel tracks at different time horizons:

1. **Product pipeline** (scan → layer → mold) ships and iterates on
   workshop-cast time scale. cf-device-design + cf-cast-cli +
   cf-scan-prep + supporting cf-* crates. Workshop iteration IS the
   engineering loop. **Pace: days to weeks per arc.**

2. **Sim research** (sim-soft + sdf_bridge + mesh-sdf) is a
   state-of-the-art softbody-FEM project on its own roadmap. Treated
   as science with literature-benchmark gates per Phase-H leg. Plugs
   into the product pipeline only when literature-validated AND
   physical-cast-validated. **Pace: quarters to years per leg.**

The recon arc's mistake was conflating these. Sim convergence problems
became "blockers" because the insertion-sim panel was wired into
cf-device-design as if predictive. Decoupling kills the false
urgency. Sim becomes interesting research questions, not
on-fire problems.

---

## 1. The reframe

### 1.1 What we were doing (implicitly)

- Insertion-sim panel was a load-bearing surface of cf-device-design
- Recon arcs treated FEM convergence failures as product blockers
- Three-session pattern (bookmark / recon / implementation) was
  right-sized for symptoms but mis-sized for architectural pivots
- F3 LM, F4 warmup, H4-2-C asymmetric bound, H2 mesh refinement —
  each a real ship but each palliating one symptom of a four-symptom
  bundle the design book commits to curing together at Phase H
- "Pro-tier" simulation goal was being pursued under product-cadence
  pressure that made the architectural decisions impossible to size
  honestly

### 1.2 What we're doing now

- **Decouple the insertion-sim from cf-device-design.** The product
  ships as a CAD tool (scan ingestion → layer parameterization →
  cavity geometry → mold output). Workshop iteration is the
  engineering loop.
- **Move sim work to its own cadence.** No more tactical-recon-fix
  cycles forcing sim issues into product time. Phase H is a
  multi-quarter research arc with per-leg literature gates.
- **Workshop iteration = the heuristic.** Physical castings are the
  ground truth. Designer prints v1, casts in silicone, fits, evaluates,
  iterates. cf-device-design supports this loop as a geometry tool;
  it does not predict behavior.
- **Long-term sim ROI** = avoid $100s–$1000s of physical iteration
  per cycle. Worth multi-year research investment to reach. Not
  worth tactical-fix urgency to fake.

### 1.3 The principle — time equity

> "Treat it as science, and organize this program so we give
> ourselves a lot of time equity."

Time equity = not feeling rushed, having budget to do things right.
The rigid-body sim was already built on this principle; the
softbody sim deserves the same posture. Recon-arc pressure created
false urgency that distorted the design space.

---

## 2. Decoupling commitment

The insertion-sim is removed from cf-device-design as a load-bearing
surface. Two implementation paths to consider:

**Path A — feature-flag the panel.** Insertion-sim panel + SimMode
resource stay in tree but only build/expose under a `--features
research-mode` flag. Default builds of cf-device-design ship with
no sim surface.

**Path B — extract to a separate research tool.** Move
`tools/cf-device-design/src/insertion_sim*.rs` + supporting modules
to a new `tools/cf-sim-research/` crate. cf-device-design has no
dependency on sim-soft. Sim research happens in the separate tool.

**Default recommendation**: Path A first (cheap, ~1 session). Path B
later if research tooling grows enough to warrant its own crate.

### 2.1 What the decoupled cf-device-design looks like

- Scan ingestion (via cf-scan-prep)
- Centerline + cap-plane handling
- Layer parameterization (cavity inset, per-layer thickness,
  material + Slacker ratio)
- Cavity SDF + per-layer mesh extraction
- Heat-map viz (currently driven by FEM strain field — switches to
  pure geometry-derived overlays or removed)
- Mold-geometry output (via cf-cast-cli)

What's removed/hidden:

- Insertion-sim panel
- SimMode resource + sliding/growing-intruder dispatcher
- Per-step scalar fields tied to FEM outputs
- Sim-mode-specific UI controls

The Bevy + egui visual layer stays; the FEM-coupled overlays go.

---

## 3. This-week actions

### A1 — Decouple insertion-sim from cf-device-design ✓ COMPLETE 2026-05-20

**SCOPE REVISED 2026-05-19 LATE-NIGHT** after A1's in-session
scoping revealed the sim is woven into layer/cavity/intruder
rendering. Feature-flag approach (~1 session) was the wrong
estimate; the correct refactor is structural extraction.

**SHIPPED 2026-05-20** on `dev` directly per
[[feedback-omnibus-pr-single-branch]] (5 phases + Phase 2.5
mini-arc, 6 sessions, ~10 000 LOC moved across 4 crates).
Full plan + per-phase ship records: `docs/SIM_DECOUPLE_REFACTOR_PLAN.md`.

**As-built outcome**:
- cf-device-design ships as CAD-only tool (35 tests, ~10 000 LOC
  deleted including `insertion_sim.rs` 5762 LOC + `insertion_sim_ui.rs`
  2275 LOC + 7 orphan deps: mesh-repair / meshopt / mesh-sdf /
  cf-design / sim-soft / sim-ml-chassis / nalgebra). Intruder mesh
  + helpers deleted per Q4.1. `cargo build --no-default-features`
  green confirms zero sim-soft dep.
- `tools/cf-sim-research/` carries the FEM sim + heat-map + deformed-
  cavity + slide-intruder (99 tests / 12 ignored). iter-1 sock smoke
  converged 16/16 to 83.35 mm full depth, ~3.0 N peak.
- Shared crates: `design/cf-device-types/` (24 tests — scan / design /
  slacker / sim modules + design_toml + LAYER_SURFACE_PALETTE) +
  `design/cf-device-geometry/` (50 tests — sdf_layers + bevy_mesh +
  cavity + clip_plane). cf-cast-cli is a future 3rd consumer of
  cf-device-geometry.

Workshop loop unchanged (scan → cf-device-design → save TOML →
cf-cast-cli → workshop print); sim research loop is now independent
(load design TOML + scan in cf-sim-research → run sim).

Sequencing: B1 + B2 + A1 all SHIPPED. Next inflection points are
Phase H sub-leg L1 (sim-research, quarter-pace per
[[project-b2-sim-research-roadmap]]) and workshop iter-1 cast
(cf-cast-cli, workshop-pace).

### A2 — Revert H2 diagnostic instrumentation [N/A — never landed]

**Resolved 2026-05-19 LATE-NIGHT**: A2's intended cleanup is a no-op
on the current branch. The H2-Q3 DIAG instrumentation described in
`docs/H2_RECON_FINDINGS.md` (x_curr threading through the try_*
functions + pivot-vertex eprintln) was authored in-session during
H2 recon but never committed to a branch reachable from main / the
current `sim-arc/sl-4-intruder-render`. `grep -rn "H2-Q3" sim/
tools/` finds zero code references and the named test
`h2_recon_cavity_6mm_n16` does not exist in
`tools/cf-device-design/src/insertion_sim.rs`. Audit trail in
`docs/H2_RECON_FINDINGS.md` stands.

Q1 still runs to completion in the background; its log is data,
not action.

### A3 — Stop the tactical FEM-convergence recon thread

No further work on H-anything, F-anything, future-N-knob-anything as
product-cadence recon arcs. The audit + this gameplan capture the
findings; future FEM work moves to the sim-research roadmap (§5).

### A4 — Update auto-memory to point at this gameplan

`MEMORY.md` adds a top-level pointer so future sessions find this
doc before resuming any recon arc.

---

## 4. Short-term work (next 1-3 sessions)

### B1 — FDM-mold-optimization arc in cf-cast-cli

**Problem**: current molds are cubic blocks around the negatives.
Wastes FDM print time + plastic. Should be walls around the
negatives.

**Goal**: replace cube-minus-negative with negative-expand-then-
subtract-negative (uniform-offset shell). Print time + plastic
budget drops significantly; structural integrity for pouring +
demolding preserved.

**Three-session pattern**:
- **Bookmark**: characterize current mold geometry, target shell
  geometry, structural constraints (pour-gate access, alignment-pin
  surfaces, demolding clearance, wall thickness for plastic stiffness).
- **Recon**: cheapest correct construction. Options: uniform-offset
  SDF + boolean cleanup; mesh-offset directly on the negative;
  ribbed-shell variant for very thin layers.
- **Implementation**: ship per the recon outcome.

Worth its own arc because it's a real user-facing win and exercises
cf-cast-cli's geometry surface independently of any sim work.

### B2 — Sim research roadmap doc

`docs/sim-research/PHASE_H_ROADMAP.md` (or `sim/L0/soft/docs/...`).
Structure:

- Sequencing rationale (IPC first → mixed-u-p → friction → Tet10
  + h-refinement → Prony)
- Per-leg design-doc placeholders (each leg gets its own design
  doc when work begins)
- Literature-benchmark gates per leg (e.g., Hertz contact pressure
  matches analytical to N digits; IPC self-intersection invariants
  per Li 2020 etc.)
- Time-horizon estimates at quarter granularity (NOT week or session)
- Pointers into the design book for each leg's spec

This doc separates sim research planning from product planning.
Both happen in parallel without one starving the other.

### B3 — Product strategy memo (optional, if helpful)

Currently the product strategy is implicit in cf-* arc memos
(workshop iter-1, cast-iter-1, scan-prep, etc.). A consolidated
product memo at the top level might help orient — but only if it
adds clarity beyond what's already in MEMORY.md's project index.
Defer unless surfacing a specific gap.

---

## 5. Long-horizon sim research roadmap (Phase H)

The sim-research roadmap doc (B2 above) will be the source of
truth; this section is the sketch.

### 5.1 The leg-by-leg target

Per the audit's §5.1 sequencing:

1. **IPC barrier replacing penalty** — without it, nothing else
   converges cleanly. ~3000-6000 LOC. Literature gate: Li 2020
   barrier formulation; Hertz contact + canonical-problem benchmarks.
   *Probably its own multi-quarter research arc.*
2. **Mixed u-p `<Yeoh>`** — for Ecoflex 00-30 at ν ≈ 0.499. Cures
   locking properly. ~1500-3000 LOC. Literature gate: cantilevered
   incompressible Tet4 beam matches analytical bending to N digits.
3. **Smoothed Coulomb friction** — sliding-intruder physics. Plugs
   into IPC infrastructure once IPC ships. ~500-1000 LOC.
4. **Tet10 in contact band + stress-gradient h-refinement** — rim
   fidelity. Complementary, not alternative. Adaptive h-refinement
   via red-green subdivision (Bey 1995). Each is ~1500-3000 LOC.
5. **Prony viscoelasticity** — time-resolved silicone response. Only
   if cast-iter+ data shows it matters. ~800-1500 LOC.

### 5.2 Validation cadence

Each leg ships with:
- A design doc citing the relevant book chapter
- A literature-benchmark test gate (analytical or canonical-problem
  comparison)
- A regression suite extension
- A retrospective comparison against the prior leg's output on
  the canonical scenes

When AT LEAST IPC + mixed-u-p + friction have shipped AND cast
iter-1+ has happened, the **3-way physical-cast validation harness**
goes live:

```
sim prediction (with current shipped legs) → physical cast measurement
```

The gap drives prioritization of remaining legs.

### 5.3 Pace expectation

Quarter granularity per leg, not session. IPC alone could be 1-2
quarters of focused work. Total Phase-H bundle: probably 1-2 years
of sim-research work. **This is fine.** Workshop iteration carries
the product forward in the meantime.

### 5.4 Optional Phase E (GPU) sequencing

The book's Phase E (GPU port) sits between current state and Phase H.
Open question: do we port the current penalty solver to GPU first
(to make Phase H development tractable on bigger meshes), or skip
straight to Phase H on CPU? Resolve when the sim-research roadmap
doc is written — depends on whether early Phase H legs are
wall-clock-bound on CPU.

---

## 6. What's explicitly NOT being done

These were on the path under the old cadence; they're paused or
shelved under this reframe.

- **H2 mesh-refinement implementation.** The bookmark + recon
  findings stand as audit trail. The work itself moves into the
  sim-research roadmap as part of leg 4 (stress-gradient
  h-refinement, not distance-banded). DO NOT resume H2 as a
  tactical product-cadence arc.
- **Further N_STEPS / κ / Yeoh-validity-bound tuning** in the
  current FEM stack. Diminishing returns; the surface is exhausted.
- **F4 / F5 / Fx warmup arcs.** Same reasoning.
- **F3 LM "Gate B" extension to deeper cavities.** Same reasoning.
- **Tactical mesh-quality tweaks** to make cavity > 5 mm work in
  the current stack. Quote the audit: this regime is the
  Phase-H-class problem the current stack was never built for.

What stays alive:

- Q1 (uniform 2 mm spike) — finishes in background, log is data
- F3 LM in dormant default-off state — bit-equal-when-disabled
  preserves Phase-B numerics
- Existing recon docs — audit trail
- Existing FEM-stack code — kept as-is; the decouple makes it
  research surface, not product surface

---

## 7. Architectural commitments

### 7.1 Crate map

**Research cluster** (own roadmap, quarter-pace):
- `sim/L0/soft` (sim-soft) — FEM solver, materials, contact, elements
- `sim/L0/soft/src/sdf_bridge` — SDF→tet bridge (BCC + stuffing)
- `mesh/mesh-sdf` — SDF oracle + flood-fill primitives

**Product pipeline** (workshop-iter pace):
- `cf/cf-design` — design surface (`Sdf` trait, Solid, primitives)
- `cf/cf-geometry` — mesh primitives, IndexedMesh, bbox, transforms
- `cf/cf-scan-prep` — scan ingestion, cleaning, capping, centerline
- `cf/cf-cap-planes` — cap-plane primitives shared between cf-scan-prep / cf-device-design / cf-cast
- `tools/cf-device-design` — layer-engineering UI (decoupled from sim per §3)
- `cf/cf-cast` — mold geometry library
- `tools/cf-cast-cli` — mold-output CLI tool

**Spec docs**:
- `docs/studies/soft_body_architecture/` — sim research spec (the
  design book)
- Per-arc memos in `docs/` and the auto-memory index — product spec

### 7.2 The bridge — when sim plugs back in

Each Phase-H leg ships into sim-soft. cf-device-design optionally
gains a "validate with sim" mode that uses the leg-by-leg shipped
sim — gated behind the research-mode feature flag UNTIL the sim is
validated against physical casts. Default cf-device-design build
stays sim-free.

**The gating condition** for promoting sim from research-feature-
flag to product-default: 3-way validation (sim prediction + physical
cast + design intent) closing to within product-acceptable tolerance
on the canonical sleeve-on-probe scene. Not on a session timeline;
on a "when the data says so" timeline.

---

## 8. Decision rationale (for future cold-readers)

If you're reading this and wondering whether to resume tactical
FEM recon work, here's the reasoning chain that led to this
gameplan:

1. The recon arcs (H4-2-C, F3 LM, F4, H2) each shipped real value
   but each palliated one symptom of a multi-symptom bundle.
2. The audit (`docs/SIM_ARCHITECTURE_AUDIT.md`) mapped each
   pathology onto a book-documented failure mode with a documented
   Phase-H cure. The current FEM stack is Phase C without IPC
   running into Phase-H-class problems.
3. The book bundles four Phase-H deliverables (IPC + mixed-u-p +
   Tet10 + adaptive refinement) because they cure interacting
   pathologies; partial fixes hit diminishing returns.
4. Phase H is multi-quarter scope; recon cadence forces it into
   week-scope, which sizes wrong.
5. The cf-device-design tool's actual product value is CAD
   surfacing (scan → layer → mold), not behavior prediction. The
   sim was wired in as if predictive but never validated against
   physical casts.
6. Workshop iteration carries the product forward today on physical
   feel + measurement. Sim plugs in when it can match physical;
   not before.
7. Time equity (treating sim as science, not as fire-fighting) is
   the operational principle.

If new evidence contradicts any step in this chain, update this
doc rather than silently resuming the old cadence.

---

## 9. Pointers

### Foundational docs
- `docs/SIM_ARCHITECTURE_AUDIT.md` — full audit of the FEM stack vs
  the design book
- `docs/H2_RECON_FINDINGS.md` — Q3 pivot-position data + Q1 result
  (when complete); the empirical finding that triggered the audit
- `docs/H2_MESH_REFINEMENT_BOOKMARK.md` — the bookmark that
  triggered the recon; superseded by this gameplan but retained as
  audit trail

### Design book (sim-research spec)
- `docs/studies/soft_body_architecture/src/SUMMARY.md` — index
- `docs/studies/soft_body_architecture/src/110-crate/03-build-order.md` — Phase ladder
- `docs/studies/soft_body_architecture/src/40-contact/00-why-ipc/00-penalty.md` — why penalty must go
- `docs/studies/soft_body_architecture/src/20-materials/05-incompressibility/{00-locking,01-mixed-up,02-f-bar}.md` — locking + cures
- `docs/studies/soft_body_architecture/src/30-discretization/00-element-choice/{00-tet4,01-tet10,04-tradeoff}.md` — element commitment
- `docs/studies/soft_body_architecture/src/70-sdf-pipeline/{01-tet-strategies,03-adaptive-refine}.md` — meshing + refinement

### Product spec (implicit, lives in arc memos)
- `MEMORY.md` index of arc memos
- `docs/CURVE_FOLLOWING_DESIGN.md` — v2 architecture
- The cf-device-design + cf-cast arc memos as referenced from
  MEMORY.md

### To-be-written
- `docs/sim-research/PHASE_H_ROADMAP.md` (or
  `sim/L0/soft/docs/PHASE_H_ROADMAP.md`) — sim-research planning;
  drafted as part of B2
- `docs/PRODUCT_STRATEGY.md` — only if a gap surfaces

---

End of gameplan. Read this before resuming any recon arc or sim
work. If the gameplan is wrong, update the gameplan; do not
silently revert to the old cadence.
