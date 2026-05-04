# CortenForge Visual-Review Viewer — Design Plan

**Status:** SEED — to iterate across sessions before execution.

**Purpose:** Design the unified visual-review viewer that supersedes per-example tool chains (MeshLab + filter dialogs for sim-soft; f3d for mesh examples). One command, one window, every static-field artifact in the workspace. Living document — delete after the viewer ships and the workspace has migrated to it, per `feedback_code_speaks`.

---

## Originating signal

**2026-05-04, sim-soft examples arc PR1 commit 1 — `93b4d5a4` (sphere-sdf-eval).** Visual pass surfaced a gap the iter-2 visualization-convention round didn't anticipate:

- **MeshLab** opened the 1331-point PLY but required a 2-filter chain per example to render `extras["signed_distance"]` (`Filters → Quality Measure and Computations → Per Vertex Quality Function` with `func q = signed_distance`, then `Filters → Color Creation and Processing → Colorize by Vertex Quality`). Per-example friction × 10 PR1 rows = a lot of clicking, every review pass.
- **f3d** ignores per-vertex custom scalars entirely — it would just render 1331 white dots in a grid. f3d is geometry-only.
- **The user separately flagged that f3d had already been underwhelming for the mesh-v1.0 examples** — even though those examples' content IS geometric (which f3d nominally handles), f3d's rendering wasn't living up to the visual review the user wanted.

The PLY-vs-Bevy split locked in the sim-soft iter-2 round conflated two cases. **Mesh-v1.0's content is geometric** (shape, winding, surface, lattice density) — f3d nominally fits but disappoints. **Sim-soft Tier 1's content is field-over-space** (a scalar over a sampled domain) — no native viewer in the workspace renders that well. The fix is workspace-level: a unified viewer that handles both, replacing both f3d and MeshLab.

Per `feedback_examples_drive_gap_fixes` — examples drive engine fixes; this gap surfaced exactly when an example exercised it. Per `feedback_fix_gaps_before_continuing` — sim-soft PR1 paused at commit 1; viewer ships first.

---

## Anchoring goals

1. **Single tool** — one binary serves mesh, sim-soft, cf-design, integration, casting-domain, and future-domain static-artifact examples.
2. **Zero-friction visual pass** — `<binary> <path>` opens a colormapped, oriented, navigable view. No filter chains, no menu spelunking, no per-example viewer setup.
3. **Field-aware** — auto-detects per-vertex scalars (per Q4 lock + stress-test E1; per-face deferred to mesh-io extension out of v1) and colormaps them appropriately (divergent for signed, sequential for unsigned, categorical for integer IDs). User toggles between scalars if multiple are present.
4. **Geometry-faithful** — meshes render with correct winding and shading; point clouds render with adjustable point size; lines render with controllable widths. The viewer does NOT silently re-orient or re-wind input data. **Geometry-only mode is dominant, not a corner case** — stress-test N3 confirmed 22 of 25 mesh-v1.0 example PLYs are geometry-only (no custom scalars); cf-viewer's geometry-only path must be polished (sane default lighting + camera framing + axis gizmo) because it's what most retrofit consumers exercise.
5. **Scriptable / reproducible** — viewer accepts a config file or CLI flags so a saved view (camera angle, scalar choice, colormap range) can be replayed for screenshots / regression-style visual review.

---

## Scope boundaries

**In scope (v1):**

- Static-field examples — sim-soft Tier 1-3 + Tier 5 rows 1-3, 8-11, 15-16, 19 (10 examples).
- Static-geometry examples — mesh-v1.0 examples (eventual retrofit; see open question 7).
- Format support: PLY (binary + ASCII), with custom per-vertex / per-face scalars.
- Cross-platform Bevy app with orbit camera, scalar selector, colormap selector.

**Out of scope (v1):**

- Dynamic playback — Tier 4 + Tier 6 (drop-and-rest, Hertz, compressive block, silicone device E2E) handled by full per-example Bevy apps because they need custom interaction (scrubbing, force arrows, contact-pressure overlay).
- Time-series / scrubbing / animation. Static-only for v1.
- Vector fields (gradient arrows, displacement vectors) — could extend the viewer's surface but no immediate consumer.
- JSON force-stretch / displacement traces — matplotlib post-hoc; not the viewer's job.
- Interactive editing or parameter sweeping — viewer is read-only.
- Headless / CI rendering — desktop tool only; CI builds it via `cargo build` for integrity but doesn't run it.

---

## Open questions — answers (iter 1)

All 8 walked + locked as head-architect calls per `feedback_master_architect_delegation`. Recon read-pass: Bevy 0.18 is workspace-pinned (`Cargo.toml:466`, `default-features = false, features = ["x11"]`); `sim-bevy` at `sim/L1/bevy/` is a substantial existing Bevy library (`camera.rs`, `mesh.rs`, `materials.rs`, `multi_scene.rs`, etc.) used by 4 integration examples; `sim-ml-chassis-bevy` is a sibling adapter; the workspace has **no `default-members`** so every member builds in CI today, and Bevy is already a transitive CI cost via sim-bevy. `mesh-io::load_ply_attributed` exists and is the symmetric counterpart to `save_ply_attributed` used by every existing PLY-emitting example.

### Q1 — Crate location: `cf-viewer/` at workspace root.

Matches `xtask/` precedent (workspace tools live flat at root, not under a categorical dir). Matches the `cf-` namespace convention (`cf-design`, `cf-geometry`, `cf-spatial`). Avoids creating a new top-level `tools/` directory for a single tool. **No `[package.metadata.cortenforge]` tier annotation** — corrected from initial iter-1 lock per stress-test E2: workspace tools (xtask) and binary crates (every example) carry no tier metadata; only L0/L1 *library crates* under `sim/` do. cf-viewer is a binary tool, matches xtask precedent.

### Q2 — Binary name: `cf-view`.

Pairs with crate name `cf-viewer` (crate is the library + binary, binary entry point is `cf-view`). Short, no obvious conflicts (`cv` collides with curriculum-vitae tooling in some shells), follows the `cf-*` namespace, easy to type. Usage: `cf-view <path-to-ply>`. CLI flags layered on top.

### Q3 — Input formats: PLY-only for v1. CLI binary only.

Every existing example in the workspace emits PLY (mesh-v1.0, sim-soft sphere-sdf-eval). PLY natively carries per-vertex / per-face scalars — the whole point of the viewer. OBJ + STL don't carry scalars and would dilute the v1 anchoring goal. Library API (in-process consumption) deferred to v2 unless a real consumer asks. PLY parsing reuses `mesh_io::load_ply_attributed` — no greenfield parser.

### Q4 — Scalar detection: auto-discover at load + alphabetical-first-pick + UI dropdown. **Per-VERTEX only.**

On PLY load, the viewer enumerates all per-vertex scalar properties — everything in `AttributedMesh.extras` (`BTreeMap<String, Vec<f32>>`) beyond the geometry-required `x/y/z`. Auto-selects the first by alphabetical name. Exposes a UI dropdown for the user to switch in multi-scalar PLYs. CLI flag `--scalar=<name>` available for scripted / reproducible workflows. Zero-scalar case: render the geometry only — this IS the mesh-v1.0 retrofit case (22 of 25 mesh-v1.0 examples have no custom scalars; see anchoring-goal note below).

**Per-face scalars: NOT supported in v1.** Corrected from initial iter-1 lock per stress-test E1: mesh-io's `AttributedMesh.extras` is per-vertex only (validated against `vertex_count`), and PLY writes extras under `element vertex`. The workspace has no per-face scalar API today. Adding one is a mesh-io extension out of v1 scope; defer until a real consumer asks.

### Q5 — Colormap policy: auto-detect by value distribution + CLI override.

**Detection rules:**
- If the scalar has any negative value → **divergent** (cool→white→warm), centered at 0.
- Otherwise, if all values cast bit-equal to their integer cast AND there are < 16 unique values → **categorical** (tab10 or similar).
- Otherwise → **sequential** (viridis).

**CLI override:** `--colormap=divergent|sequential|categorical` for explicit control when the heuristic mis-classifies (rare but possible; e.g., a signed scalar that happens to be all-positive in the PLY's bbox).

**Three colormap library v1:** divergent (`coolwarm` or `RdBu`), sequential (`viridis`), categorical (`tab10`). Hard-coded for v1; pluggable colormaps deferred.

### Q6 — Up-axis convention: `+Z` default + `--up=+X|+Y|+Z` CLI flag.

Matches the mesh-v1.0 convention (`f3d --up=+Z`) which exists because the printability + lattice examples use the build-plate orientation. Sim-soft sphere-sdf-eval is rotation-symmetric so any up-axis works; `+Z` default doesn't hurt. Orbit camera lets the user re-orient interactively post-startup. Future Bevy-based dynamics examples (Tier 4 / Tier 6) inherit the same default.

### Q7 — Mesh-v1.0 retrofit story: followup PR, after viewer stabilizes against 1-2 sim-soft consumers.

Sequencing:
1. Viewer ships against ONE real consumer (sphere-sdf-eval) — proves the basic shape end-to-end.
2. Sim-soft PR1 resumes; row 2 (`hollow-shell-sdf`) and row 3 (`sdf-to-tet-sphere`) author against the viewer, refining the API as edge cases surface.
3. **Then** mesh-v1.0 retrofit lands as its own PR — 9 README updates (`f3d --up=+Z` → `cf-view <path>`), tested against the now-stable viewer.

Not "same PR" because: bundling 9 example retrofits with the viewer's first ship would obscure where bugs originate, and the viewer hasn't yet been exercised on real consumers when the retrofit goes in. Not "as touched" because: that creates inconsistency over time — some examples on viewer, some still on f3d, no clean migration date. Followup PR is the clean cut.

### Q8 — CI handling: workspace member, no special gating.

Bevy is already a transitive CI cost via `sim-bevy` and the 4 integration examples. Adding `cf-viewer` doesn't materially change CI build time. No `default-members` exclusion needed (workspace doesn't use `default-members` today). No feature flag — viewer is the crate's purpose; gating Bevy behind a feature for a Bevy-native viewer is silly.

**Tests:** v1 ships with **no integration tests that run a Bevy window** (CI runners don't have a display server; spinning up the full app would require headless EGL setup that's overkill for v1). v1 DOES ship with **unit tests on the colormap detection logic + scalar-extraction logic + CLI parsing** — those are pure-function deterministic and exercise the brain of the viewer. The window itself goes uncovered until v2 adds headless rendering for screenshot regression tests.

`xtask grade` likely treats `cf-viewer` similarly to `sim-bevy` — opt-in coverage, structurally-pinned-low coverage with a documented tier rationale (Bevy app code is mostly resource setup + system wiring, hard to unit-test without a running app). Match `sim-bevy`'s `[package.metadata.cortenforge]` shape.

---

## PR shape + commit segmentation (iter 1)

**Branch:** `feature/cf-viewer-arc` off `main`. Sim-soft branch (`feature/sim-soft-examples-arc`) rebases after viewer arc merges.

**Commit segmentation (target ~7-8 commits):**

1. **Scaffolding** — `cf-viewer/Cargo.toml`, `src/main.rs` Bevy app skeleton (window + clear color + exit-on-Esc), workspace registration.
2. **PLY loading** — `load_input(path) -> ViewerInput` via `mesh_io::load_ply_attributed`; data structures for the loaded scalars + geometry; unit tests on a known PLY fixture.
3. **Geometry rendering** — spawn `Mesh3d` entities from the loaded `AttributedMesh` (faces case) or point-cloud entities (faces-empty case). Default material, no scalar coloring yet.
4. **Colormap pipeline** — detect distribution category (divergent/sequential/categorical) per Q5 rules, compute per-vertex / per-face RGBA, attach as a vertex-color attribute on the Bevy mesh. Unit tests on the detection logic.
5. **Orbit camera + UI** — orbit-zoom-pan camera (likely lift from `sim-bevy::camera` or write a minimal one), scalar-selector dropdown for multi-scalar PLYs, colormap-selector dropdown.
6. **CLI flags** — `--scalar=<name>`, `--colormap=<kind>`, `--up=<axis>` via `clap`. Unit tests on flag parsing.
7. **Sphere-sdf-eval consumer** — update `examples/sim-soft/sphere-sdf-eval/README.md`'s "Visuals" section to point at `cf-view <path>` (replaces the MeshLab two-filter chain). End-to-end smoke test.
8. **(Optional)** — README at `cf-viewer/README.md` documenting the binary's CLI surface + supported PLY shapes.

PR size precedent: comparable to mesh-v1.0 PR #224 (8 mesh examples, 8 commits). Reviewable in one sitting.

**No pre-squash tag** per `feedback_pre_squash_tag` — viewer arc doesn't touch architecture book chapters; this is tooling, not architecture.

---

## Iter-1 still-open items (not blocking; surfaces during code authoring)

1. **Shared Bevy utilities — locked: NO sim-bevy dep.** Stress-test N1 confirmed sim-bevy's transitive deps (sim-types, sim-core, sim-mjcf, sim-urdf) are physics-specific and have zero overlap with cf-viewer's concerns. **Decision: write the orbit camera + AttributedMesh→Bevy-mesh conversion inline in cf-viewer (~150 LOC).** Factor out to a `cf-bevy-common` sibling crate ONLY if a second consumer emerges. cf-viewer's only Bevy-related deps in v1 are `bevy` itself, `bevy_egui` (likely; see #3), and `mesh-types` / `mesh-io` for the PLY surface.
2. **Axis conversion (PLY-z up → Bevy-y up)** — Bevy is internally Y-up; mesh-v1.0 examples use Z-up build-plate convention (hence f3d's `--up=+Z`). Two implementation paths: (a) rotate input mesh at load to align user-space-up with Bevy-y (sim-bevy's `convert.rs` does this), or (b) configure camera + lighting + grid to interpret the chosen axis as up without rotating data. **Default for now:** option (a) per sim-bevy's precedent; reconsider during commit 5 if (b) ends up cleaner for the orbit-camera shape.
3. **UI library choice** — egui (mature, common Bevy companion via `bevy_egui`) vs Bevy's built-in UI. Decide during commit 5 (orbit camera + UI). egui is likely cleaner for dropdowns; Bevy UI is leaner but rougher for forms. **Default for now:** lean toward `bevy_egui` unless workspace already uses Bevy UI primitives elsewhere.
4. **Scalar value range** — should the viewer expose a CLI flag / UI slider for the colormap range (clamping outliers)? Out of v1 scope; flag if it surfaces.

---

## Adjacent / related decisions (not gating but worth tracking)

- **Sim-soft Bevy adapter** for deforming tet meshes (boundary-face re-extraction per frame from `x_current`) is the Tier-4-and-Tier-6 dynamic-playback path. **Not the unified viewer's responsibility** — those are per-example Bevy apps. The adapter could live in the same crate as the viewer if there's natural reuse (mesh rendering primitives, colormap utilities, camera setup), or in `sim/L1/sim-bevy/` if the dynamic-playback story diverges.
- **Casting-domain / cf-design future examples** — the viewer should anticipate them. cf-design likely emits the same PLY shape (geometry + per-vertex scalars). Casting domain may want time-series support eventually — but that's v2, not v1.

---

## Iteration log

_(append session-by-session; date-stamped; what changed and why)_

- **2026-05-04 (seed):** Initial structure + originating signal + anchoring goals + scope + 8 open questions. No design decisions yet — that's iter 1's work. Sim-soft examples arc paused at PR1 commit 1; viewer ships first per `feedback_fix_gaps_before_continuing`.
- **2026-05-04 (iter 1 — head-architect calls + PR shape):** Recon read-pass: Bevy 0.18 workspace-pinned with `default-features = false, features = ["x11"]`; existing `sim-bevy` at `sim/L1/bevy/` with 14-file Bevy library (camera + mesh + materials + multi-scene); 4 integration examples already pull Bevy into CI; workspace has no `default-members` so every member builds in CI; `mesh_io::load_ply_attributed` is the symmetric counterpart to the `save_ply_attributed` every example uses. Locked all 8 open questions:

  **Q1 (location):** `cf-viewer/` at workspace root — matches `xtask/` precedent + `cf-*` namespace. **Q2 (binary):** `cf-view` — pairs with crate name, no shell conflicts. **Q3 (formats):** PLY-only v1 via `mesh_io::load_ply_attributed`; CLI binary only, no library API. **Q4 (scalar detection):** auto-discover at load + alphabetical-first-pick + UI dropdown for multi-scalar; CLI flag `--scalar=<name>` for scripted use. **Q5 (colormap):** auto-detect by value distribution (any negative → divergent; integer-valued + < 16 unique → categorical; else sequential); CLI override via `--colormap=<kind>`; three colormap library v1 (`coolwarm`/`viridis`/`tab10`). **Q6 (up-axis):** `+Z` default + `--up=+X|+Y|+Z` CLI flag, matches mesh-v1.0's `f3d --up=+Z` precedent. **Q7 (mesh retrofit):** followup PR after viewer ships and is exercised on 1-2 sim-soft consumers — clean cut, no "as touched" inconsistency. **Q8 (CI):** workspace member, no special gating; Bevy is already a transitive CI cost via sim-bevy + integration examples; v1 unit-tests the colormap + scalar + CLI logic but does NOT run a Bevy window in CI (no headless display).

  **PR shape locked:** 7-8 commits on `feature/cf-viewer-arc` (off main): scaffolding → PLY loading → geometry rendering → colormap pipeline → orbit camera + UI → CLI flags → sphere-sdf-eval consumer (README update from MeshLab to `cf-view`) → optional crate README. Comparable size to mesh-v1.0 PR #224. No pre-squash tag (tooling, not architecture).

  **Still-open iter-2 items (not blocking):** (1) shared Bevy utilities — depend on sim-bevy vs factor out `cf-bevy-common` sibling; decide during commit 5 against actual reuse profile. (2) tier annotation — `L1` likely matches sim-bevy; verify xtask's tier shape during scaffolding. (3) UI library — egui via `bevy_egui` is the default lean; reconsider only if workspace uses Bevy UI primitives elsewhere. (4) scalar-value-range CLI clamp — out of v1 scope; flag if it surfaces.

  **Plan is now executable.** Next session: branch off main as `feature/cf-viewer-arc`, ship commit 1 (scaffolding). Cold-start should read this doc end-to-end first. After viewer arc merges, sim-soft branch rebases, sphere-sdf-eval README updates, sim-soft PR1 resumes at row 2.

  **Next iteration entry point:** scaffolding commit on `feature/cf-viewer-arc` — `cf-viewer/Cargo.toml` + `src/main.rs` Bevy app skeleton + workspace registration. Recon `xtask/Cargo.toml` for tier-annotation precedent before authoring.

- **2026-05-04 (iter 1.1 — stress-test pass + revisions):** User-initiated stress test of the iter-1 locked decisions against the codebase per `feedback_thorough_review_before_commit` + `feedback_risk_mitigation_review`. Recon batch on the high-risk assumptions: mesh-io PLY surface, AttributedMesh shape, sim-bevy dep tree, xtask precedent, mesh-v1.0 PLY content distribution, Bevy 0.18 mesh API. Two spec errors + three nuances surfaced.

  **E1 (per-face scalars):** iter-1 Q4 promised "auto-discover per-vertex AND per-face scalars." Mesh-io has no per-face scalar API — `AttributedMesh.extras` is `BTreeMap<String, Vec<f32>>` validated against vertex count; PLY writes extras under `element vertex` only. Q4 revised to per-vertex only; per-face deferred to mesh-io extension out of v1 scope.

  **E2 (tier annotation):** iter-1 Q1 locked `tier = "L1"` for cf-viewer. Wrong — recon shows xtask, every example crate (mesh-v1.0 and our just-shipped sphere-sdf-eval), and every binary in the workspace carry NO `[package.metadata.cortenforge]` block. Tier is for L0/L1 *library crates* under `sim/` only. Q1 revised: cf-viewer carries no tier annotation, matches xtask precedent.

  **N1 (sim-bevy dep direction):** iter-1 still-open #1 said "depend on sim-bevy if reuse is clean." Sim-bevy's transitive deps include sim-types, sim-core, sim-mjcf, sim-urdf — physics-specific crates with zero overlap with cf-viewer's concerns. Decision locked early (no longer iter-2 deferred): **never depend on sim-bevy.** Write orbit camera + AttributedMesh→Bevy-mesh conversion inline in cf-viewer (~150 LOC).

  **N2 (axis conversion):** new still-open item added — Bevy is Y-up internally; mesh-v1.0 examples use Z-up build-plate; cf-viewer must handle the conversion either by rotating input data on load (sim-bevy's `convert.rs` precedent, default) or configuring Bevy camera/lighting/grid for the chosen axis. Decide during commit 5.

  **N3 (retrofit calculus — anchoring goal sharpened):** of 25 mesh-v1.0 example crates, only 3 carry per-vertex scalars (attributed-mesh-basics, ply-with-custom-attributes, mesh-sdf-distance-query); 22 are geometry-only PLYs. **Geometry-only mode is the dominant retrofit case, not a corner case.** Anchoring goal 4 sharpened to reflect this — cf-viewer's geometry-only path must be polished because most retrofit consumers exercise it.

  **Positive confirmations:** `load_ply_attributed -> IoResult<AttributedMesh>` ✓; Bevy 0.18's `Mesh3d` + `MeshMaterial3d` + `VertexAttributeValues` API matches sim-bevy's usage ✓; PLY writes `extras` as `property float <name>` under `element vertex` (matches sphere-sdf-eval's actual PLY) ✓; f3d is referenced in 9+ mesh-v1.0 example READMEs (retrofit scope confirmed) ✓.

  **Plan still executable; no PR-shape changes.** Stress-test corrected two spec errors and sharpened three deferred items in place; commit segmentation and branch strategy unchanged.

  **Next iteration entry point:** unchanged — scaffolding commit on `feature/cf-viewer-arc` (off main). Verified: cf-viewer/Cargo.toml has NO tier annotation, depends on bevy + mesh-io + mesh-types + clap + anyhow (and likely bevy_egui pending iter-2 still-open #3).

---

## Cross-references

- **Originating arc:** [`sim/L0/soft/EXAMPLE_INVENTORY.md`](../sim/L0/soft/EXAMPLE_INVENTORY.md) — sim-soft examples arc; visualization convention section flags this doc as the supersession point.
- **Existing viewer-pattern reference:** [`examples/mesh/README.md`](../examples/mesh/README.md) — current f3d convention; mesh retrofit target (open question 7). f3d's `--up=+Z` flag and per-example viewer-instruction blocks are the precedent the unified viewer replaces.
- **Bevy precedent in workspace:** `examples/fundamentals/sim-cpu/` — 60+ Bevy examples with the headless-asserts-pre-render + visual-mode-default pattern. The unified viewer is simpler than these (no simulation, just rendering) but inherits their build-time + dep-weight cost profile.
- **Cadence memos:**
  - [`feedback_examples_drive_gap_fixes`](../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_examples_drive_gap_fixes.md) — examples are *just as much* for fixing gaps as for showcasing.
  - [`feedback_fix_gaps_before_continuing`](../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_fix_gaps_before_continuing.md) — when examples reveal engine gaps, stop and fix the engine first.
  - [`feedback_planning_as_therapy`](../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_planning_as_therapy.md) — for ambitious personal projects, default to ceiling-level architecture; planning depth IS the product.
  - [`feedback_master_architect_delegation`](../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_master_architect_delegation.md) — during architectural execution with documented plan, Claude makes tech decisions; brings only project-scope / UX / blocker calls.
  - [`feedback_no_allergy_to_bold_choices`](../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_no_allergy_to_bold_choices.md) — don't pre-weight to minimum-diff / battle-tested options.
  - [`feedback_simplify_examples`](../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_simplify_examples.md) — viewer's simplicity reduces per-example complexity.
  - [`feedback_code_speaks`](../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_code_speaks.md) — delete this doc after the viewer ships and the workspace has migrated.

---

## Resume-here for next session

**Iter 1 complete (2026-05-04). All 8 open questions locked. PR shape locked. Plan is executable — next step is code, not more planning.**

1. Read this file end-to-end (especially the locked Q1-Q8 + PR shape sections + still-open iter-2 items).
2. Branch off `main` as `feature/cf-viewer-arc`. (The seed planning doc rode `feature/sim-soft-examples-arc` because it was prose-only; viewer code ships its own PR per the locked branch strategy.)
3. Recon `xtask/Cargo.toml` for any tier-annotation / category metadata precedent that should propagate to `cf-viewer/Cargo.toml`.
4. Author **commit 1 — scaffolding**: `cf-viewer/Cargo.toml`, `src/main.rs` (Bevy app skeleton — window + clear color + Esc-to-exit, nothing more), workspace member registration in root `Cargo.toml`. Build clean, run clean, exit clean.
5. Pause for user review per the per-commit cadence; iterate per `feedback_one_at_a_time_review`.
6. Continue down the 7-8-commit segmentation; the iter-1 PR shape is the executable checklist.

If anything in the locked decisions feels wrong on a re-read, redirect — locked ≠ frozen, and the planning doc is the place for second thoughts before code makes them expensive to undo.
