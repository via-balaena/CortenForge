# cf-cast Sharp-Feature Extractor — Research Recon (S2 / future)

**Status: RESEARCH BOOKMARK, not scheduled.** Seeded 2026-05-30 from the
§MA-14 R2/DC feasibility spike (`docs/CF_CAST_MESHING_ARCHITECTURE_RECON.md`).
This doc exists so the spike's *measured requirements* aren't re-discovered the
hard way when a bespoke extractor is eventually built. It is a problem
statement + requirements + open questions — **not** a design.

Per [[feedback-correctness-over-speed]].

---

## §SX-1 Why this might exist

cf-cast meshes every part with **marching cubes on a baked dense `ScalarGrid`**
(`mesher::solid_to_mm_mesh`). MC has two structural limits the workshop has hit:

1. **It rounds sharp features** by construction — the cavity-floor↔socket↔seam
   junction (Defect 1, §MA-S1b) and the flange faces. We work around this today
   with **exact post-MC mesh-CSG** (the floor slab, the mating features) and a
   **post-MC weld** (§MA-14 R1 flange slivers) — both are *repairs of MC output*,
   not better extraction.
2. **It triangulates tilted flat faces with slivers** — the flange-perimeter
   "vertical cracks" (Defect 2): ~20 µm thin triangles where a flat face crosses
   the axis-aligned grid at a shallow (fitted-seam) tilt.

cf-design already has `dual_contouring::mesh_field_dc` (QEF, sharp-feature) +
`adaptive_dc`, **both unused by cf-cast.** Switching the extractor is the
recon's standing S2 lever (options B/B2 "grid-DC", C "adaptive", D
"mesh-direct"). The question is whether a *purpose-built* extractor could
subsume the floor-CSG + sliver-weld workarounds **and** improve organic cup-wall
fidelity (S2) in one architecture.

---

## §SX-2 What the spike measured (the load-bearing data)

§MA-14 R2/DC feasibility spike — cf-design's **uniform** `Solid::mesh_dc` vs MC
on the same tilted-sphere cup field (`piece::tests::ma14_dc_feasibility`):

| mesher | slivers<1e-3 | watertight (boundary edges) | near-seam scatter (rms) |
|---|---|---|---|
| MC 3 mm (today) | 2 | 0 ✓ | **3.6 µm** |
| DC tol 6 mm | 0 | **40 ✗** | 9.0 µm |
| DC tol 3 mm (matched) | **3** | 0 ✓ | **13.4 µm** |
| DC tol 1.5 mm | 10 | 0 ✓ | 8.4 µm |

**The out-of-the-box DC is worse than MC on every axis that matters here.** A
bespoke extractor must therefore *beat* these, not merely swap the algorithm.

---

## §SX-3 Measured requirements a bespoke extractor MUST meet

1. **Seam-plane vertex snapping (the make-or-break).** The seam mating face is
   the surface that *seals the mold*; §F-4 holds it flat to **1 µm** because MC
   linearly interpolates the linear halfspace SDF onto the plane exactly. DC's
   QEF minimization scattered seam vertices to **~13 µm rms** (3–4× MC). A
   bespoke extractor must detect vertices on a planar (halfspace / cap-plane)
   cut and snap them onto that plane — otherwise it regresses the most
   important surface to fix a cosmetic one.
2. **Planar-face sliver avoidance.** Dual vertex placement on tilted flat faces
   sheds its *own* thin triangles (DC made 3–10 vs MC's 2). The extractor needs
   explicit planar-region handling (regular grid-aligned triangulation on flat
   spans) — *or* it still pairs with the §MA-14 R1 weld, in which case DC buys
   nothing for Defect 2. Strike: this is intrinsic to dual placement; don't
   assume "DC ⇒ no slivers."
3. **Guaranteed watertight + manifold output.** Uniform DC emitted 40 boundary
   edges (holes) at coarse resolution — would fail F4. Non-negotiable: every
   edge shared by exactly two faces, at every resolution the workshop uses.
4. **Composes with the post-MC mesh-CSG transforms.** The mating features
   (`apply_mating_transforms`: floor slab, plug-lock socket, pour bore, bolt +
   dowel holes) run as manifold3d booleans on the extractor output and require
   a clean manifold input. Whatever the extractor emits must round-trip through
   `indexed_mesh_to_manifold` and the booleans without the §MA-14 S3d failure
   mode (welding/merging near tight feature walls → non-manifold edges +
   self-intersections). *(Note: §MA-14 R1 sidesteps this by welding the RAW MC
   mesh BEFORE the booleans — a bespoke extractor should likewise produce its
   clean mesh upstream of the CSG stage.)*
5. **Bounded cost on the silhouette `UserFn` field.** The flange `Silhouette2d`
   extraction is a high-Lipschitz `UserFn` (S0 profile: ~70 % of cup-piece
   time; the adaptive-DC "Lipschitz cost cliff" = 235 s). `Solid::mesh_dc` sets
   `cell = tolerance / lipschitz`, so a high Lipschitz explodes the cell count.
   A bespoke extractor must cap or decouple cell size from the field's global
   Lipschitz estimate, or the fine-cell case is unaffordable.
6. **Deterministic.** Reproducible byte-for-byte across runs (the project gates
   on byte-identity for no-op flags + regression diffing). DC's `HashMap`-keyed
   cell vertices must yield a deterministic emission order.

---

## §SX-4 Open questions (carried from §MA-9 #5)

- Can a **grid-based** DC (B2) meet all six §SX-3 requirements, or is an
  **adaptive octree** (C) needed for cost (req 5) — and does adaptivity worsen
  watertightness (req 3) at level boundaries?
- Is seam-snapping (req 1) cleanest as (a) a post-extraction projection of
  near-plane vertices, (b) a QEF constraint that pins planar-cut cells, or
  (c) keeping the seam as an exact post-MC/DC CSG cut (so the extractor never
  meshes it)? Option (c) mirrors how the floor + features are already handled
  and may make req 1 moot.
- Does a sharp-feature extractor make the §MA-S1b floor-CSG slab and the
  §MA-14 R1 weld **redundant** (subsume both), or do they remain as
  belt-and-suspenders? (If redundant, retiring them is part of the payoff.)
- **mesh-direct (D)** alternative: for the cup-wall *organic* surface, skip
  extraction entirely and offset/clip the cf-scan-prep scan mesh — does that
  beat any extractor on fidelity (req: S2 cosmetic) while the flat features
  stay SDF/CSG?

---

## §SX-5 Decision rule before scheduling

Do **not** schedule this as the Defect-2 fix — the spike proved DC doesn't
solve Defect 2 (req 2) and regresses the seam (req 1); §MA-14 R1 (weld) is the
correct, validated, *reversible* near-term fix. Schedule this extractor work
only when (a) the cup-wall organic fidelity (S2) genuinely needs sub-3 mm cells
the silhouette tax can't afford, AND (b) a spike shows a candidate extractor
clears req 1 + req 3 on a production part. Until then the MC + post-MC-CSG +
weld stack is the architecture. When/if this lands, the R1 weld becomes a no-op
and is deleted (it's scaffolding, not debt).

Related: [[project-cf-cast-q4-mating-face-quality-active]] (§MA arc),
[[project-cf-cast-sdf-meshcsg-paradigm-boundary]] (the CSG composition rules
req 4 must respect), [[feedback-correctness-over-speed]].
