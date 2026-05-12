# Casting Roadmap — Track F: Cast Tooling for the Layered Silicone Device v1.0

**Status:** active (opened 2026-05-12)
**Predecessor doc:** [`SIM_SOFT_ROADMAP.md`](SIM_SOFT_ROADMAP.md)

---

## Strategic context

This roadmap covers **physical-cast tooling**, parallel to `SIM_SOFT_ROADMAP.md`'s focus on simulation. Both target the same v1.0 device: a multi-layer silicone body with a scan-shaped internal cavity, designed via SDF, sanity-checked in sim, cast physically for subjective tightness calibration.

Why now: the sim track has reached its MVP-image done-criteria (see [`SIM_SOFT_ROADMAP.md`](SIM_SOFT_ROADMAP.md) §Convergence target) as of the 2026-05-12 C2.2 ship. Materials + equipment are in hand. The active decision point is "cast v1" — which requires mold STLs and a procedure spec that don't yet exist. CortenForge has all the primitive geometry/mesh capabilities to generate these; what's missing is the composition layer.

For v1.0, the cast's purpose is calibration data, not a polished product. The right move is to ship the tooling with sensible defaults and let physical iter-1 surface what to improve.

---

## Target deliverable

A **cast v1 layered silicone device in the user's hand**, produced from:

1. **Per-layer mold STLs** — one cup per layer (3 total), plus one printable plug for the innermost layer's scan-cavity shape (1 total). **4 STL files**, not 6 — layers 2 and 3 use the previously-cured layer as the plug, not a printed object.
2. **Pour volume spec per layer** — grams of each silicone, sanity-checked against the 2 lb/silicone budget.
3. **Procedure spec markdown** — ordered layer-by-layer instructions referencing the SiliconeMaterial table for mix ratios and cure times.
4. **Printability gate** — every exported STL validated against the chosen printer technology before write.

No quantitative gate on the cast itself; tightness is subjectively calibrated after demolding.

---

## Workflow

The artifact-generation side (steps 2-3 below) and the slicer-import side are well-understood. Workshop chemistry — vacuum-degas timing, the copper-mesh embed strategy for the middle layer, layer-to-layer adhesion behavior — is user-domain knowledge that gets folded into F3's procedure-spec template when that leaf is built.

End-to-end flow (CortenForge produces artifacts for steps 2-3; everything else is workshop / external):

1. **Design** — body SDF in `cf-design::Solid` (cavity scan, 3 concentric layers). Sim already runs sanity checks via rows 20-25.
2. **Generate molds** (F1) — 4 STLs land on disk.
3. **Generate spec + volumes** (F2 + F3) — procedure markdown + pour-volume summary.
4. **3D print molds** — user, external (slicer → printer).
5. **Mix + pour layer 1** — workshop; silicone-chemistry details are user-domain.
6. **Demold layer 1**.
7. **Cast layer 2** — pour around the layer-1 plug (with the copper-mesh + carbon-black middle-layer treatment).
8. **Cast layer 3**.
9. **Finish + measure**.

---

## Current state inventory

### Already in place (existing crates the cast pipeline composes)

- ✅ `cf-design::Solid` — CSG primitives (sphere, cuboid, from_sdf, union, difference, intersection)
- ✅ `cf-design::Sdf` trait — uniform SDF surface for arbitrary geometries
- ✅ `mesh-sdf::SignedDistanceField` — SDF from triangle mesh; impls `cf-design::Sdf`
- ✅ `mesh-offset::offset_mesh` — positive/negative offset via SDF + marching cubes
- ✅ `mesh-offset::marching_cubes` — iso-surface extraction from a `ScalarGrid`
- ✅ `mesh-printability::validate_for_printing` — FDM / SLA / SLS / MJF printability gates
- ✅ `mesh-printability::find_optimal_orientation` — auto-orient for print
- ✅ `mesh-measure` — AABB, OBB, cross-section, dimensions
- ✅ `mesh-io::save_stl` — STL binary + ASCII export
- ✅ `sim_soft::SiliconeMaterial` const table — density (kg/m³) + Shore + Yeoh + provenance for Ecoflex / DragonSkin / DS_15 (Smooth-On data)

### Missing for v1 cast

- ❌ Mold-cup STL extraction from a layered body SDF (bounding region ∖ layer-solid → marching cubes)
- ❌ Plug STL extraction for the innermost layer (scan-cavity geometry → marching cubes)
- ❌ Pour channel geometry (CSG-subtract a cylinder from the mold cup at the +z extremum; manual placement params for v1)
- ❌ Per-layer pour-volume integration (grid-sample the layer's solid SDF → sum negative voxels × voxel_volume → grams via `SiliconeMaterial::density`)
- ❌ Procedure spec markdown generator (template + computed values)
- ❌ Printability gate wiring at STL-write time
- ❌ A reference cast-v1 example crate that ties everything together against the `layered-silicone-device` geometry

---

## Code organization decision

**Recommendation: new `design/cf-cast` crate + example wrapper.**

Aligns with the existing `design/cf-*` family (cf-design, cf-geometry, cf-spatial). Public API ~3 entry points:
- `CastSpec` struct — holds layer SDFs + material assignments + procedure params.
- `CastSpec::export_molds(out_dir, printer_config) -> Result<MoldExportReport>` — writes STLs, runs printability gate.
- `CastSpec::write_procedure(path) -> Result<()>` — writes the markdown procedure spec including per-layer pour volumes.

Example crate `examples/cast/layered-silicone-device-v1/` is a thin wrapper that constructs a `CastSpec` against the device's layered body geometry and calls the two methods. Total v1 example ≈ ~60-100 LOC.

Why crate-first over example-first: standing project preference is foundational architecture over quick-patch examples, and the public API is small enough that library-up-front costs little extra. Refactoring an example crate to a library later is friction worth avoiding.

---

## Track F leaves

| # | Leaf | v1 MVP? | Notes |
|---|------|---------|-------|
| F1 | Mold + plug STL exporter | ✅ | For each layer: bounding region in `cf-design::Solid` ∖ layer-solid ∖ pour-channel → marching cubes → `mesh-io::save_stl`. Plug: scan-cavity SDF → marching cubes → STL. 4 STLs total. Default demolding axis = +z (cavity opens up); single-piece molds (flexible silicone tolerates demolding from a one-piece cup). |
| F2 | Per-layer pour volume calculator | ✅ | Grid-sample the layer's solid SDF on a `mesh-offset::ScalarGrid`, sum negative-SDF voxels × voxel_volume → m³, multiply by `SiliconeMaterial::density` → grams. Sanity-check against 2 lb/silicone budget; error if any layer exceeds. |
| F3 | Procedure spec markdown exporter | ✅ | Template generator with per-layer masses, mix ratios + cure times from `SiliconeMaterial`, layer ordering, vacuum-degas reminder, demold checkpoints. Generic Smooth-On guidance for workshop chemistry; user revises against bench reality. |
| F4 | Printability gate at STL-write time | ✅ | Chain `mesh-printability::validate_for_printing` on each generated STL; refuse to write (or write-with-warning) on hard failures. Configurable; default = `PrinterConfig::fdm_default()`. |
| F5 | Algorithmic pour channel + vent placement | ⏳ defer | For v1, manual params. Algorithmic placement (e.g., topology analysis for highest-point auto-vent) only after iter-1 surfaces real issues. |
| F6 | Multi-piece mold support (>2 parts) | ⏳ defer | The open-mouth cavity may need a split mold for the innermost layer's plug. v1 assumes one-piece molds work; revisit on iter-1. |
| F7 | Undercut / draft-angle analysis | ⏳ defer | Use `mesh-printability` overhang detector against demolding direction. Flexible silicones tolerate mild undercuts; defer until iter-1 demolding shows real issues. |

---

## v1 MVP scope

**F1 + F2 + F3 + F4**, wired end-to-end. Each leaf independently committable; v1 MVP slice ships when all four pass the user's standing gates against the layered-silicone-device geometry.

**v1 MVP done-criteria:**

1. Running the v1 example crate writes 4 STL files + 1 procedure spec markdown + 1 pour-volume summary to a known output directory (e.g., `examples/cast/layered-silicone-device-v1/out/`).
2. Every STL passes the configured `mesh-printability` gate without hard errors (warnings on minor overhangs OK).
3. The procedure spec is human-readable; per-layer masses sum within the 2 lb budget per silicone.
4. User can take the STLs to a slicer (Bambu Studio / Cura / PrusaSlicer), print, and follow the procedure markdown end-to-end to a cast in hand. (Steps 5-9 of the workflow are workshop work — sim/code success criterion stops at "user can execute the procedure.")

**Out of MVP scope:** F5-F7 above; any sim-side work beyond what's already shipped.

**Effort estimate:** F1 ≈ 3-4 hours focused; F2 ≈ 1-2 hours; F3 ≈ 1-2 hours; F4 ≈ 30 min. **Total ~6-8 hours of focused arc work**, which at the user's patient cadence (eyes-on-pixels per leaf, two-pass review, ask-before-commit) realistically spreads across **3-7 calendar days**. Both framings are true; the elapsed time is the cadence, not the work content.

---

## Open architectural decisions (need user input before F1)

Most parameters default cleanly inside F-leaf implementations; these two are real architectural choices the user is better-positioned to make.

- **Q1 Print technology assumption**: v1 default = FDM PLA at 0.2mm layer height, since `mesh-printability::PrinterConfig::fdm_default()` is pre-built and the user has an FDM printer. Alternative: SLA resin if tighter tolerances matter. FDM is more accessible; SLA produces finer features at higher cost.
- **Q2 Scan-cavity geometry for the innermost mold's plug**: use the existing `layered-silicone-device` row's programmatic cube fixture as a stand-in (CortenForge ships the workflow; user swaps in a real scan STL when ready), or wire real-scan-STL import as part of v1 MVP (one extra leaf)? Both flow through `mesh_sdf::SignedDistanceField` → `Solid::from_sdf`; the difference is whether v1's first cast uses a placeholder or actually fits the scanned reference.

Other parameters (mold wall thickness, pour-channel placement, mold release strategy, layer keying, demolding axis) belong in F-leaf implementation specs, not the strategic roadmap; they surface in-context when each leaf is being built.

---

## PR shape (horizontal slicing)

Single long-running `dev` branch per the standing strategy; PR shape decided at ship time. Stages:

| Stage | Leaves | Outcome |
|-------|--------|---------|
| **Stage 1 — Single-layer mold prototype** | F1 (single-layer subset) + F4 | First mold + plug STL pair on disk; printability gate passes; visually inspect in slicer preview. Validates the bounding-region ∖ layer-solid CSG logic on a known-simple body before adding multi-layer complexity. |
| **Stage 2 — Full v1 MVP** | F1 (full 3-layer) + F2 + F3 wired through `cf-cast` + example crate | All 4 STLs + procedure markdown + mass summary against the actual layered-silicone-device geometry. End-to-end pipeline. |
| **Stage 3 — Physical cast iter-1** (workshop, not a code slice) | Print + pour + cure + demold + measure | First physical artifact. May surface F5-F7 gaps; if so, those prioritize for v1.1. |
| **Stage 4+ — long tail** | F5-F7 + any refactor demanded by iter-1 reality | Polish + algorithmic placement + multi-piece support if needed. |

---

## Risks + uncertainty

Worth surfacing now so failures aren't surprises:

- **Workshop chemistry is user-domain.** Vacuum-degas timing, copper-mesh embed approach, layer-to-layer adhesion behavior — F3's procedure-spec template will start from generic Smooth-On guidance and the user revises against bench reality.
- **The bounding-region ∖ layer-solid approach may produce a mold cup with thin features at the cavity-opening edges.** The printability gate catches catastrophic cases; eyes-on-pixels review catches subtle ones. May need to inflate the bounding region or add explicit chamfers.
- **Unit boundary**: sim/cf-design uses meters; mesh-shell + mesh-printability use mm. F1 needs explicit conversion at the boundary. Easy bug source if missed.
- **Flexible-silicone demolding from a one-piece mold is an assumption.** If the open-mouth cavity has bad undercuts, F6 (multi-piece mold) gets promoted from "defer" to "v1 critical." First iter-1 cast tells us.
- **Mass-budget over-run** is a hard 2 lb/silicone constraint; F2 errors on overrun rather than warns. Catching this at generation time means redesigning layer thicknesses before any silicone is poured.

---

## Update protocol

Mirrors `SIM_SOFT_ROADMAP.md`:

1. Move leaves from ❌ to ✅ in the inventory on ship.
2. Add one-line entries to the slice ship log below with date + commit SHA.
3. If a leaf splits or new sub-leaves emerge, edit the track table and note in the slice log.
4. Mark dropped leaves with 🚫 + one-line reason; preserve for audit.

Each commit must pass:
- **A-grade-or-it-doesn't-ship** — `cargo xtask grade` clean across affected crates.
- **Eyes-on-pixels** — generated STLs inspected in a mesh viewer or slicer preview before declaring a leaf complete.
- **Ask-before-commit** — standing rule.

When Stage 2 ships (v1 MVP), update the [target deliverable](#target-deliverable) section with paths to the generated artifacts. When Stage 3 ships (physical cast), update with a photo or descriptive note.

Archive trigger: move to `docs/archive/` after v1 has been cast + measured + iter-2 begins on a new roadmap.

### Slice ship log

_(empty — first entry lands when Stage 1 ships)_
