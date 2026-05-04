# CortenForge Visual-Review Viewer — Design Plan

**Status:** Commits 1 (scaffolding `02bceb9c`) + 2 (PLY load + `ViewerInput` `093f0410`) + 3 (geometry rendering `79b97339`) + commit-3 cold-read fixes (`b46d0b99`) + 4 (colormap pipeline `0c3ab996`) + commit-4 visual-review corrections (`8b53f296` — TwoSlopeNorm + unlit + saturated bwr) shipped on `dev`. Plan iter-1 locked + iter-1.3 (commit-3 head-architect calls) + iter-1.4 (commit-4 point-cloud coloring lock) + iter-1.5/1.6/1.7 (commit-4 visual-review corrections) all banked. **Commit 4 visually reviewed + passed 2026-05-04** — sphere-sdf-eval renders with the deep-blue inside-sphere region (~6% of points) clearly distinct from the white isosurface ring and the saturated red outer corners. Branch strategy revised mid-commit-1 to single-branch flow per `feedback_single_active_branch`. **Next step: commit 5 (orbit camera + UI).**

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
- Format support: PLY (binary + ASCII), with custom per-vertex scalars (per-face deferred — see Q4).
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

Every existing example in the workspace emits PLY (mesh-v1.0, sim-soft sphere-sdf-eval). PLY natively carries per-vertex scalars (per-face is a PLY-format feature but not exposed by mesh-io's `AttributedMesh` today; see Q4) — the whole point of the viewer. OBJ + STL don't carry scalars and would dilute the v1 anchoring goal. Library API (in-process consumption) deferred to v2 unless a real consumer asks. PLY parsing reuses `mesh_io::load_ply_attributed` — no greenfield parser.

### Q4 — Scalar detection: auto-discover at load + alphabetical-first-pick + UI dropdown. **Per-VERTEX only.**

On PLY load, the viewer enumerates all per-vertex scalar properties — everything in `AttributedMesh.extras` (`BTreeMap<String, Vec<f32>>`) beyond the geometry-required `x/y/z`. Auto-selects the first by alphabetical name. Exposes a UI dropdown for the user to switch in multi-scalar PLYs. CLI flag `--scalar=<name>` available for scripted / reproducible workflows. Zero-scalar case: render the geometry only — this IS the mesh-v1.0 retrofit case (22 of 25 mesh-v1.0 examples have no custom scalars; see anchoring-goal note below).

**Per-face scalars: NOT supported in v1.** Corrected from initial iter-1 lock per stress-test E1: mesh-io's `AttributedMesh.extras` is per-vertex only (validated against `vertex_count`), and PLY writes extras under `element vertex`. The workspace has no per-face scalar API today. Adding one is a mesh-io extension out of v1 scope; defer until a real consumer asks.

### Q5 — Colormap policy: auto-detect by value distribution + CLI override.

**Detection rules:**
- If the scalar has any negative value → **divergent** (cool→white→warm), centered at 0 via `TwoSlopeNorm` (see Normalization below).
- Otherwise, if all values cast bit-equal to their integer cast AND there are < 16 unique values → **categorical** (tab10 or similar).
- Otherwise → **sequential** (viridis).

**Normalization (locked iter 1.5):**
- **Divergent — `TwoSlopeNorm` (asymmetric):** value 0 always pins to `t = 0.5` (white center); negatives stretch to `t ∈ [0, 0.5]` (deep blue → white) using their own extent; positives stretch to `t ∈ [0.5, 1.0]` (white → deep red) using theirs. Both extremes always saturate. Matches matplotlib's `TwoSlopeNorm` and ParaView's default for asymmetric divergent data. The naive symmetric `(-m, m)` alternative (matplotlib's default `Normalize`) wastes half the colormap when data is asymmetric — sphere-sdf-eval (min=-1, max=+2.46) only reaches t=0.297 on the cool side under symmetric normalization, washing out the inside-sphere blue under bright lighting + tonemapping. See iteration log entry "iter 1.5" for the full rationale.
- **Sequential:** `(data_min, data_max)` linear normalization to `t ∈ [0, 1]`.
- **Categorical:** integer value indexed `mod 10` into the tab10 table.

**CLI override:** `--colormap=divergent|sequential|categorical` for explicit control when the heuristic mis-classifies (rare but possible; e.g., a signed scalar that happens to be all-positive in the PLY's bbox).

**Three colormap library v1:** divergent (saturated bwr-style — endpoints `(0,0,1)` / `(1,0,0)`, white center; iter 1.7 lock chose this over matplotlib-coolwarm's perceptually-uniform-but-medium-saturation endpoints because for visual review the negative/positive split must be unmistakable), sequential (`viridis`), categorical (`tab10`). Hard-coded for v1; pluggable colormaps deferred. The const name in `colormap.rs` is still `COOLWARM` — that's the divergent-bin name, not the matplotlib-palette name.

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

`xtask grade` treatment: cf-viewer carries no `[package.metadata.cortenforge]` block per Q1 (matches xtask, not sim-bevy). xtask itself is excluded from `xtask grade` by virtue of its untiered status; cf-viewer inherits the same exclusion. If xtask grade ever needs to gate the viewer (e.g., for clippy / fmt), do it via the existing path-based filtering rather than retrofitting tier metadata.

---

## PR shape + commit segmentation (iter 1)

**Branch:** single long-running `dev` branch (renamed from `feature/sim-soft-examples-arc` 2026-05-04 after retiring the two-branch plan — see iteration log entry "iter 1.2"). Branch ≠ PR; PR shape is a ship-time decision off `dev`.

**Commit segmentation (target ~7-8 commits):**

1. **Scaffolding** — `cf-viewer/Cargo.toml`, `src/main.rs` Bevy app skeleton (window + clear color + exit-on-Esc), workspace registration.
2. **PLY loading** — `load_input(path) -> ViewerInput` via `mesh_io::load_ply_attributed`; data structures for the loaded scalars + geometry; unit tests on a known PLY fixture.
3. **Geometry rendering** — spawn `Mesh3d` entities from the loaded `AttributedMesh` (faces case) or point-cloud entities (faces-empty case). Default material, no scalar coloring yet.
4. **Colormap pipeline** — detect distribution category (divergent/sequential/categorical) per Q5 rules, compute per-vertex RGBA, attach as a vertex-color attribute on the Bevy mesh. Unit tests on the detection logic.
5. **Orbit camera + UI** — orbit-zoom-pan camera (likely lift from `sim-bevy::camera` or write a minimal one), scalar-selector dropdown for multi-scalar PLYs, colormap-selector dropdown.
6. **CLI flags** — `--scalar=<name>`, `--colormap=<kind>`, `--up=<axis>` via `clap`. Unit tests on flag parsing.
7. **Sphere-sdf-eval consumer** — update `examples/sim-soft/sphere-sdf-eval/README.md`'s "Visuals" section to point at `cf-view <path>` (replaces the MeshLab two-filter chain). End-to-end smoke test.
8. **(Optional)** — README at `cf-viewer/README.md` documenting the binary's CLI surface + supported PLY shapes.

PR size precedent: comparable to mesh-v1.0 PR #224 (8 mesh examples, 8 commits). Reviewable in one sitting.

**No pre-squash tag** per `feedback_pre_squash_tag` — viewer arc doesn't touch architecture book chapters; this is tooling, not architecture.

---

## Iter-1 still-open items (not blocking; surfaces during code authoring)

1. **Shared Bevy utilities — locked: NO sim-bevy dep.** Stress-test N1 confirmed sim-bevy's transitive deps (sim-types, sim-core, sim-mjcf, sim-urdf) are physics-specific and have zero overlap with cf-viewer's concerns. **Decision: write the orbit camera + AttributedMesh→Bevy-mesh conversion inline in cf-viewer (~150 LOC).** Factor out to a `cf-bevy-common` sibling crate ONLY if a second consumer emerges. cf-viewer's only Bevy-related deps in v1 are `bevy` itself, `bevy_egui` (likely; see #3), and `mesh-types` / `mesh-io` for the PLY surface.
2. **Axis conversion (PLY-z up → Bevy-y up) — locked at commit 3: option (a) inline.** Bevy is internally Y-up; mesh-v1.0 examples use Z-up build-plate convention (hence f3d's `--up=+Z`). Implementation: copy-adapt sim-bevy's `convert.rs` swap inline in cf-viewer's mesh-conversion path — `[x, z, y]` on positions and normals, `(v0, v2, v1)` winding flip on triangle indices. Locked because (b) "configure camera/lighting for axis without rotating data" would force every downstream consumer (orbit camera, gizmos, light placement) to re-derive the swap; doing it once at conversion time keeps Bevy's Y-up assumption intact through the whole render path. The `--up=<axis>` CLI flag (commit 6) parameterizes which input axis maps to Bevy-y; Z-up is the default per Q6.
3. ~~**UI library choice**~~ — RESOLVED at iter 1.8 (commit 5): `bevy_egui = "0.39.1"`. Workspace has no Bevy UI primitive precedent (`grep -rE "use bevy::ui::" --include="*.rs"` empty); bevy_egui 0.39.1's depends-on `bevy ^0.18.0` matches the workspace pin; cleaner forms/dropdowns than Bevy UI. UI systems run in `EguiPrimaryContextPass` schedule; dropdowns use `egui::ComboBox`. EguiPlugin added via `app.add_plugins(EguiPlugin::default())`.
4. **Scalar value range** — should the viewer expose a CLI flag / UI slider for the colormap range (clamping outliers)? Out of v1 scope; flag if it surfaces.
5. **Crease-angle splitting for the geometry-only path — DEFERRED to a polish commit before the Q7 mesh-v1.0 retrofit PR.** sim-bevy's `triangle_mesh_from_indexed` does crease-angle vertex splitting (cos 30° threshold) so cuboid lattices shade as flat surfaces and curved SDF surfaces shade smoothly. cf-viewer commit 3 ships the simpler smooth-normals path (use `mesh.normals` if `Some` else `geometry.compute_vertex_normals()`) — visually fine for the sphere-sdf-eval consumer but renders mesh-v1.0 cuboid faces with noticeable corner blurring. Cost to land splitting now: ~80 extra LOC + extra tests. Per `feedback_baby_steps`, kept commit 3 small; mesh-v1.0 retrofit is its own followup PR (Q7) and a polish commit can land between now and then once a real mesh consumer is exercised.
6. **Point-cloud rendering — locked at commit 3: option (b) instanced spheres.** PR shape said "spawn `Mesh3d` entities (faces case) or point-cloud entities (faces-empty case)." Walked the four candidates: (a) `PrimitiveTopology::PointList` with default size (Bevy 0.18 renders 1px points; not viable at orbit-camera distances); (b) instanced small spheres / cubes per vertex; (c) custom point-cloud shader; (d) defer faces-empty entirely until commit 4. Locked (b) — sphere-sdf-eval (1331 verts, the only commit-3 consumer of this path) is well within the entity-spawn budget for a static viewer; one shared `Sphere` mesh handle + one shared `StandardMaterial` handle keeps it cheap. Sphere radius = `bbox.diagonal() * 0.005` (empirically tuned to render discrete dots without occluding neighbors at the 11³-grid scale; user can refine in commit 4 once colormap shading provides a second visibility cue). Custom shader (c) is over-engineering for v1; defer (d) would block sphere-sdf-eval, which IS the commit-3 acceptance fixture.
7. **Point-cloud coloring — locked at commit 4: option (A) per-vertex `StandardMaterial`.** PR-shape commit 4 said "compute per-vertex RGBA, attach as a vertex-color attribute on the Bevy mesh" — fine for the faces case (one mesh, vertex colors baked in), but the faces-empty path needs a separate decision because each sphere entity shares ONE `Sphere` mesh handle (per #6 lock). Three candidates: **(A)** per-vertex `StandardMaterial` (1331 materials for sphere-sdf-eval; wasteful but simple); **(B)** combined-mesh: one `Mesh` containing N translated copies of an icosahedron template, vertex-colored per-sample, single entity (~50 LOC, asset-cost clean); **(C)** custom material with per-instance color override (custom shader; over-engineering at v1). Locked (A) — preserves the iter 1.3 single-shared-sphere-mesh-handle lock, smallest deviation from commit 3's spawn pattern, ~10 LOC delta at the spawn site. 1331 materials at ~100 bytes each is ~130 KB asset memory — negligible. Per `feedback_baby_steps`: ship the simplest thing first; (B) lands as a polish commit if a future point-cloud consumer pushes past ~10K verts (none on the immediate roadmap — hollow-shell-sdf and sdf-to-tet-sphere are similar 11³-30³ grid scales). Bevy 0.18 PBR shader behavior verified against `bevy_pbr-0.18.1/src/render/pbr_fragment.wgsl:54-56`: `pbr_input.material.base_color = in.color;` — the vertex color OVERWRITES the material's `base_color` (not modulates), so the shared-template grey fallback "just works" when no vertex colors are present.

---

## Adjacent / related decisions (not gating but worth tracking)

- **Sim-soft Bevy adapter** for deforming tet meshes (boundary-face re-extraction per frame from `x_current`) is the Tier-4-and-Tier-6 dynamic-playback path. **Not the unified viewer's responsibility** — those are per-example Bevy apps. The adapter **cannot live in cf-viewer** per stress-test N1 (cf-viewer has no sim-soft / sim-types / sim-core deps); it lives in `sim/L1/sim-bevy/` (or a new `sim/L1/sim-bevy-soft/` sibling) where the physics deps already exist. cf-viewer and the sim-soft Bevy adapter can independently extract any truly-shared bits (orbit camera, AttributedMesh→Bevy-mesh) into a `cf-bevy-common` crate later if duplication crosses a threshold; v1 keeps them separate.
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

- **2026-05-04 (iter 1.2 — branch strategy retired during commit 1):** Mid-commit-1 redirect from user: "two parallel branches = recipe for disaster." Original plan had `feature/cf-viewer-arc` (viewer code, off main) running parallel to `feature/sim-soft-examples-arc` (planning + paused sphere-sdf-eval). Resolution: cherry-picked the viewer-scaffolding commit onto sim-soft branch, deleted `feature/cf-viewer-arc`, renamed sim-soft branch to `dev`. Going forward: single long-running `dev` branch; branch ≠ PR; PR segmentation decided at ship time. Banked at [`feedback_single_active_branch`](../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_single_active_branch.md). PR-shape commit segmentation in this doc unchanged — only the branching changed. **Bevy-0.18 API correction during the same commit:** `EventWriter` was renamed to `MessageWriter` in 0.18; `app.run()` returns `AppExit` (not `()`); use `MessageWriter<AppExit>` + `AppExit::Success` to exit cleanly. (Captured here so commit-2-onwards code authoring inherits the pattern.)

  **Commit 1 shipped:** `02bceb9c feat(cf-viewer): scaffolding — Bevy app skeleton + workspace member`. 4 files / 87 insertions. cargo build / clippy -D warnings / cargo doc all clean. `examples/sim-soft/` untracked-files concern during the prior two-branch attempt was just gitignored test output (`out/sdf_grid.ply`) — zero leak risk. Pre-commit hook clean.

  **Next iteration entry point:** commit 2 (PLY load + ViewerInput) on `dev` branch — see Resume-here block.

- **2026-05-04 (iter 1.3 — commit-3 head-architect calls: axis conversion + point cloud + crease-angle deferred):** Three architectural calls made at commit-3 authoring time per `feedback_master_architect_delegation`:

  **(a) Axis conversion locked option (a) inline** — moved iter-2 still-open #2 from "default for now" to "locked." Z-up → Y-up swap (`[x, z, y]` on positions + normals, `(v0, v2, v1)` on triangle indices) lives in cf-viewer's mesh-conversion path. Justification: option (b) would force every downstream consumer (orbit camera, gizmos, light placement) to re-derive the swap; once at conversion time keeps Bevy's Y-up assumption intact through the whole render path.

  **(b) Point-cloud rendering locked option (b) instanced spheres** — resolved iter-2 still-open #6. Sphere radius = `bbox.diagonal() * 0.005`, one shared mesh handle + one shared material handle. PointList (option a) renders 1px on Bevy 0.18; custom shader (option c) is over-engineering; defer (option d) would block sphere-sdf-eval as the commit-3 acceptance fixture.

  **(c) Crease-angle splitting deferred** — added iter-2 still-open #5. Commit 3 ships smooth normals only (`mesh.normals` if `Some` else `compute_vertex_normals()`); crease-angle splitting (sim-bevy precedent) lands as a polish commit before the Q7 mesh-v1.0 retrofit PR. Per `feedback_baby_steps` to keep commit 3 reviewable in one sitting.

  Recon batch before authoring: sim-bevy's `triangle_mesh_from_indexed` + `triangle_mesh_from_attributed` (sim/L1/bevy/src/mesh.rs:255-403) and `convert.rs` swap functions (sim/L1/bevy/src/convert.rs:251-283); `AttributedMesh::compute_normals` (mesh/mesh-types/src/attributed.rs:187) + `Bounded` impl (line 248) re-exported from `mesh_types`; `cf_geometry::Aabb` center/diagonal/is_empty (design/cf-geometry/src/aabb.rs); Bevy 0.18 `GlobalAmbientLight` (sim-bevy's `scene.rs:101` — `AmbientLight` was demoted to a per-camera component in 0.18). All four recon hits informed the implementation; no further surprises during authoring.

  **Next iteration entry point:** commit 4 (colormap pipeline) on `dev` after user reviews commit 3 by running `cargo run -p cf-viewer -- examples/sim-soft/sphere-sdf-eval/out/sdf_grid.ply` and a mesh-v1.0 fixture.

- **2026-05-04 (iter 1.4 — commit-4 head-architect call: point-cloud coloring option A):** Resolved iter-2 still-open #7 (point-cloud coloring) per `feedback_master_architect_delegation`. Locked option (A) — per-vertex `StandardMaterial`, one clone per sphere with `base_color` set to the colormapped value. Preserves iter 1.3's single-shared-`Sphere`-mesh-handle lock; smallest deviation from commit-3's spawn pattern; ~10 LOC delta. (B) combined-mesh and (C) custom shader queued as polish-commit options if a future consumer pushes past ~10K point-cloud verts.

  Recon batch before authoring: Bevy 0.18 `Mesh::ATTRIBUTE_COLOR` is `VertexFormat::Float32x4` (`bevy_mesh-0.18.1/src/mesh.rs:316`); PBR fragment shader OVERWRITES `pbr_input.material.base_color` from `in.color` when the `VERTEX_COLORS` shader def is on (`bevy_pbr-0.18.1/src/render/pbr_fragment.wgsl:54-56`); the shader def is auto-pushed when the mesh's vertex layout contains `ATTRIBUTE_COLOR` (`bevy_pbr-0.18.1/src/render/mesh.rs:2415-2417`). Workspace has no colormap crate (`grep -E "^(colorous|colorgrad|palette)" Cargo.toml` empty); 9-point inline tables for viridis + coolwarm, 10-color tab10 array.

  **Next iteration entry point:** commit 5 (orbit camera + UI) on `dev` after user reviews commit 4 by running `cargo run -p cf-viewer -- examples/sim-soft/sphere-sdf-eval/out/sdf_grid.ply` (signed_distance is divergent → coolwarm; inside-sphere blue, outside red, isosurface white-grey) and a mesh-v1.0 PLY (no scalars → grey fallback unchanged).

- **2026-05-04 (iter 1.5 — commit-4 visual-review correction: divergent normalization → `TwoSlopeNorm`):** Visual review of `sphere-sdf-eval` after the initial commit 4 (`0c3ab996`) showed all-warm coloring with no visible blue inside-sphere region — the math was correct (origin at `v=-1` mapped to `(0.537, 0.609, 0.933)` light blue, verified via diagnostic prints), but the `(-m, m)` symmetric range with `m = max(|min|, |max|) = 2.46` only reached `t = 0.297` on the cool side. Combined with the bright directional + ambient lighting and `TonyMcMapface` tonemap, light blue washed out to near-white while the warm half (1250/1331 points) saturated visibly. **Locked: switch divergent normalization from symmetric `(-m, m)` to `TwoSlopeNorm` (asymmetric two-half).** Negatives map `[min, 0] → [0, 0.5]`; positives map `[0, max] → [0.5, 1.0]`; value 0 always pins to `t = 0.5`. Both extremes saturate independent of magnitude asymmetry. Matches matplotlib's `TwoSlopeNorm` + ParaView's default for asymmetric divergent data. The "centered at 0" Q5 semantic is preserved either way; the change is in *how* each half stretches.

  User-decided UX call per `feedback_master_architect_delegation` (UX/visual-review effectiveness is a project-scope call, not a tech detail). Followup commit on `dev` after `0c3ab996`.

  **Bevy 0.18 lighting note worth banking:** the bright lighting setup (DirectionalLight illuminance 12_000 + GlobalAmbientLight brightness 1_200) compresses subtle hue differences in the highlight region after PBR + tonemap. If subtle colormap variations need to be visible in future commits, consider easing the lighting OR ensuring the colormap saturates near both extremes (which `TwoSlopeNorm` does for divergent). **Iter 1.6 (below) supersedes this with the cleaner fix.**

  **Next iteration entry point:** unchanged — commit 5 (orbit camera + UI) after user re-reviews the corrected commit 4.

- **2026-05-04 (iter 1.6 — commit-4 visual-review second correction: `unlit = true` when colormap data present):** The iter-1.5 `TwoSlopeNorm` fix mathematically saturated both colormap extremes (origin v=-1 → t=0 → COOLWARM[0] = (0.230, 0.299, 0.754) deep blue), but the second visual review still showed at most a faint lavender hint in the inner region — no visibly blue points. Root cause: PBR shading × bright lights × `TonyMcMapface` tonemap desaturate the deep-blue base_color into near-grey by the time it hits the display. The pipeline `sRGB(0.230, 0.299, 0.754) → linear(~0.043, ~0.073, ~0.527) → × illuminance/ambient → tonemap → display sRGB` over-drives into the highlight-desaturation regime of TonyMcMapface.

  **Locked: set `unlit = true` on `StandardMaterial` when colormap data is present** (faces with vertex colors OR point cloud with per-vertex materials). For a visualization tool the color IS the data; PBR shading actively fights the colormap. The `unlit` flag bypasses lighting entirely so the material's `base_color` (or vertex color via `Mesh::ATTRIBUTE_COLOR`) renders faithfully on every fragment. **Geometry-only path keeps `unlit = false`** — mesh-v1.0 examples need proper shading to read surface form. Single rule at material creation: `let unlit = vertex_colors.is_some()`.

  Two-line fix (one assignment + one struct field). User-decided UX call per `feedback_master_architect_delegation`. Same followup commit as iter 1.5 (rolling both fixes into one commit on `dev` since iter 1.5 alone was insufficient and the visual-review cycle is what's gating).

  **Iter 1.6 supersedes the iter-1.5 lighting-easing speculation:** rather than dim the lights (which would compromise the geometry-only path), simply opt out of lighting per-material when it's not needed. Cleaner, no global setting trade-off.

  **Bevy 0.18 design pattern banked:** for data-encoded color (visualization, debug overlays, custom UI on PBR meshes), `StandardMaterial { unlit: true, .. }` is the right escape hatch. PBR is for *physical surfaces*; data colors aren't physical surfaces.

  **Next iteration entry point:** unchanged — commit 5 (orbit camera + UI) after user re-reviews the corrected commit 4 (now with iter 1.5 + iter 1.6 fixes).

- **2026-05-04 (iter 1.7 — commit-4 visual-review third correction: divergent palette → saturated bwr):** Iter-1.5 + iter-1.6 fixed normalization + lighting; the rendered colors now matched matplotlib coolwarm faithfully. But coolwarm's endpoints `(0.230, 0.299, 0.754)` blue and `(0.706, 0.016, 0.150)` red are *intentionally* medium-saturation (matplotlib's design — perceptual uniformity > saturation), and at point-cloud-marker scale on a dark background the deep blue read as "lavender hint" rather than "unmistakably blue." For a workspace visual-review tool the negative/positive split needs to be obvious at a glance — saturation trumps perceptual uniformity for THIS use case (matplotlib's coolwarm is optimal for static figures where exact luminance matters; cf-viewer is for "is the inside-sphere region negative? yes/no"). **Locked: replace the 9-point matplotlib-coolwarm approximation with a saturated bwr-style ramp** — endpoints `(0, 0, 1)` deep blue and `(1, 0, 0)` deep red, white center `(1, 1, 1)`, linear interpolation through 9 control points. Constant name `COOLWARM` retained (it's the divergent-bin colormap implementation; "coolwarm" is the bin name even though the specific palette is now bwr). Doc comment in `colormap.rs` records the choice + rationale.

  User-decided UX call per `feedback_master_architect_delegation` (subjective saturation preference for visual review). Same followup commit as iter 1.5 + 1.6; the visual-review cycle is what's gating the commit.

  **Future flexibility:** commit 6's `--colormap=<kind>` flag can grow to also accept palette overrides (e.g., `--palette=coolwarm|bwr|rdbu`) if a downstream consumer needs matplotlib-faithful coolwarm specifically. Out of v1 scope.

  **Bevy-related lesson reinforced:** the iter-1.5 + iter-1.6 + iter-1.7 chain shows how easily "the math is right" can mask "the rendered output is wrong." For visualization commits, the visual review IS the test — three correction cycles before the colors actually communicated the data. Per `feedback_thorough_review_before_commit`, N+1 always finds something — but for visual output, N+1 has to mean an actual eye-on-pixels pass, not just code re-reads.

  **Next iteration entry point:** unchanged — commit 5 (orbit camera + UI) after user re-reviews commit 4 with all three iter 1.5/1.6/1.7 fixes applied.

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

**Commits 1 (scaffolding `02bceb9c`) + 2 (PLY load + `ViewerInput` `093f0410`) + 3 (geometry rendering `79b97339`) + commit-3 cold-read fixes (`b46d0b99`) + 4 (colormap pipeline `0c3ab996`) + commit-4 visual-review corrections (`8b53f296` — TwoSlopeNorm + unlit + saturated bwr) shipped on `dev`. Branch strategy revised mid-commit-1 to single-branch flow per `feedback_single_active_branch`. Commit 4 visually reviewed + passed 2026-05-04.**

1. Verify current state: on `dev`, commit `8b53f296` is the most recent viewer commit. Working tree should be clean. `cargo test -p cf-viewer` should report 22/22 passing.
2. Author **commit 5 — orbit camera + UI**: orbit-zoom-pan camera (write minimal inline per iter-1 still-open #1 — NO sim-bevy dep) replacing the static placeholder camera in `setup_scene`. Scalar-selector dropdown for multi-scalar PLYs + colormap-selector dropdown via `bevy_egui` (iter-2 still-open #3 default lean — confirm during authoring). Resolves the orbit-camera + UI surface that's been deferred since the static placeholder shipped in commit 3.
3. Pause for user review; per-commit cadence per `feedback_one_at_a_time_review`. Visual review steps: (a) orbit/zoom/pan with mouse on the sphere-sdf-eval fixture, (b) confirm the dropdown is wired + switching between scalars in a multi-extra PLY actually re-colors the scene.
4. Continue down the 7-8-commit segmentation in the PR shape section above; commit 6 is CLI flags, commit 7 is the sphere-sdf-eval README retrofit.

**Bevy 0.18 API gotchas banked so far:**
- Commit 1: `EventWriter` → `MessageWriter`; emit `AppExit::Success` to exit cleanly.
- Commit 3: `AmbientLight` is now a per-camera component override; world-wide ambient is `GlobalAmbientLight` (sim-bevy `scene.rs:101` precedent).
- Commit 4: vertex colors via `Mesh::ATTRIBUTE_COLOR` use `VertexFormat::Float32x4`; the PBR fragment shader OVERWRITES `material.base_color` from `in.color` (not modulates), so a shared template `StandardMaterial` "just works" as the fallback when no vertex colors are present.
- Commit 4 visual-review (iter 1.5/1.6/1.7): bright lighting (DirectionalLight 12_000 + GlobalAmbientLight 1_200) + `TonyMcMapface` tonemap compress subtle hue differences in highlights — pale colormap colors (e.g. light blue at `t = 0.3` of coolwarm) wash to near-white. **Three corrections needed:** (a) for divergent maps prefer `TwoSlopeNorm` over symmetric `(-m, m)` so both extremes always saturate; (b) set `unlit = true` on `StandardMaterial` whenever the color encodes data rather than a physical surface — PBR is for physical surfaces, data colors aren't; (c) prefer saturated palettes (bwr-style endpoints `(0,0,1)`/`(1,0,0)`) over matplotlib-coolwarm's medium-saturation endpoints when the visual goal is unmistakable category split rather than perceptual luminance uniformity.
- Commit 4 also: `cargo fmt --all -- --check` is stricter than `cargo fmt -p` (per `project_git_and_hook_gotchas.md`) — bit again at commit 4 with a multi-line `assert_eq!` reflow.

If anything in the locked decisions feels wrong on a re-read, redirect — locked ≠ frozen, and the planning doc is the place for second thoughts before code makes them expensive to undo.
