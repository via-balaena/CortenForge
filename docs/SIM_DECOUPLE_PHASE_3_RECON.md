# Sim-decouple Phase 3 — recon

> **STATUS — RECON COMPLETE; PHASE 2.5 MINI-ARC RECOMMENDED.**
> 2026-05-19. Per [[feedback-bookmark-when-surface-levers-exhaust]]
> three-session pattern, this is the recon between the plan (which
> serves as the bookmark) and implementation. Recon ran on branch
> `refactor/sim-decouple-phase-3-recon` (off `refactor/sim-decouple-phase-2`,
> origin `de397b4a`).
>
> **Predecessors**:
> - `docs/SIM_DECOUPLE_REFACTOR_PLAN.md` — parent plan; Phase 1 + 2
>   SHIPPED, Phase 3 is "move sim code" with one open scope question
>   the plan flagged at its closing paragraph.
> - `tools/cf-sim-research/` (Phase 2 ship `d4695892` + `7ea5a995`) —
>   the binary skeleton Phase 3 fills in.

---

## TL;DR

The recon settles the plan's "do `sdf_layers` + `design_toml` lift
to a shared crate, or fold into Phase 3?" question by surfacing
**four** lift candidates (not the two the plan named), then
recommending a **Phase 2.5 mini-arc** with a 4-sub-leaf ladder so
Phase 3 itself stays scoped to the sim move.

Lift candidates the recon surfaced:

| Module | LOC | Verdict | Destination |
|--------|-----|---------|-------------|
| `sdf_layers.rs` | 2091 | CLEAN LIFT | new `cf-device-geometry` crate |
| `design_toml.rs` | 666 | CLEAN LIFT | existing `cf-device-types` |
| `LAYER_SURFACE_PALETTE` const | 1 line | CLEAN LIFT | existing `cf-device-types` |
| `clip_plane.rs` | 729 | LIFT REQUIRED | new `cf-device-geometry` crate |

The fourth (`clip_plane`) is the recon's load-bearing surprise: the
original plan didn't flag it because the plan was scoped against the
sim-coupled Bevy *systems*, not their material dependencies. Three
sim spawn/update systems mount entities with `ClipPlaneMaterial`,
which is `pub(crate)` to cf-device-design. cf-sim-research can't
reach in (that's a backwards dep — the exact thing the refactor
fights). Options: lift to a shared crate, widen visibility +
add a cf-sim-research → cf-device-design dep (forbidden), or
parameterize the spawn helpers over the material type (heavy
generics, ugly). Lift wins.

After Phase 2.5 ships, Phase 3 becomes purely:
**copy `insertion_sim.rs` + `insertion_sim_ui.rs` + 4 sim-coupled
Bevy systems (`spawn_intruder_mesh` / `update_cavity_mesh` /
`update_layer_meshes` / `update_intruder_mesh`) + `LayerMeshKey` /
`CavityMeshKey` / `IntruderMeshKey` + `pose_to_bevy_transform` +
`visible_pose_for_intruder` into cf-sim-research, wire the
`InsertionSimPlugin` into its `run_render_app`.** `spawn_cavity_mesh`
+ `build_bevy_mesh_from_indexed*` are shared via cf-device-geometry
(both binaries call them). Phase 4 then strips cf-device-design's
sim-coupled copies.

**Honest estimate**: Phase 2.5 (1-2 sessions) + revised Phase 3
(2 sessions) = 3-4 sessions total, matching the original plan's
Phase 3 high end. Phase 2.5 does not BANK session work; it
**derisks** Phase 3 by landing the cross-crate moves + lift
decisions upstream, so Phase 3 itself is a pure copy + wire job
with no architectural decisions mid-arc.

---

## 1. The recon evidence

Four sibling agents ran in parallel against:

- `tools/cf-device-design/src/sdf_layers.rs` (2091 LOC)
- `tools/cf-device-design/src/design_toml.rs` (666 LOC)
- `tools/cf-device-design/src/insertion_sim.rs` (5762 LOC) +
  `tools/cf-device-design/src/insertion_sim_ui.rs` (2275 LOC)
- The 3 sim-coupled Bevy systems + supporting cast in
  `tools/cf-device-design/src/main.rs` (the Phase 3 move surface)

### 1.1 `sdf_layers` — lift to new `cf-device-geometry`

**Public surface** (all `pub(crate)` today; lift widens to `pub`):
`struct CapPlanes`, `struct CachedScanSdf`, `const SDF_SOURCE_TARGET_FACES`,
`const LAYER_PREVIEW_CELL_SIZE_M`, `const LAYER_GRID_MARGIN_M`,
`fn build_cached_scan_sdf`, `fn extract_layer_surface`,
`fn sample_sdf_into_cached_template`, `fn marching_cubes_at_iso`.

**Intra-crate reaches**: ZERO `use crate::*` imports. Fully
self-contained against external workspace crates (`bevy` —
Resource derive only, see below; `mesh-sdf`, `mesh-offset`,
`cf-cap-planes`, `cf-design`, `mesh-types`, `mesh-repair`,
`meshopt`, `nalgebra`, `anyhow`).

**Bevy coupling**: `Resource` derive on `CachedScanSdf` + `CapPlanes`
only. `bevy = { workspace = true, default-features = false }` is in
the workspace dep set already; cf-device-geometry carries the same
minimal feature set as cf-device-types.

**Why not `cf-device-types`**: too heavy. Phase 1's `cf-device-types`
is small shared types; `CachedScanSdf` carries `ScalarGrid` +
`Signed<TriMeshDistance, ...>` and pulls in
mesh-sdf/mesh-offset/meshopt/cf-cap-planes — that's a real compute
surface, not types-only.

**Why new `cf-device-geometry`**: clean separation of concerns
(`cf-device-types` = pure data; `cf-device-geometry` = compute +
rendering primitives both binaries need). Also natural home for
`clip_plane` (see §1.4).

### 1.2 `design_toml` — lift to existing `cf-device-types`

**Public surface**: schema structs (`DesignToml`, `DesignMetaBlock`,
`ScanRefBlock`, `CavityBlock`, `LayerBlock`),
`DESIGN_TOML_SCHEMA_VERSION` const, and 6 functions
(`resolve_design_toml_path`, `build_design_toml`, `save_design_toml`,
`load_design_toml`, `validate_design_toml`, `apply_design_toml`).

**Intra-crate reaches**: ZERO. Only `use cf_device_types::{...}` for
the schema types it serializes (`CavityState`, `LayerSpec`,
`LayersState`) PLUS the `LAYER_COUNT_MAX` const + `LAYER_MATERIALS`
table that `validate_design_toml` uses (layer-count bound + anchor-
key catalog check).

**Bevy coupling**: NONE. Doesn't import `bevy::*`, doesn't derive
`Resource`, doesn't touch ECS. `apply_design_toml(&DesignToml,
&mut CavityState, &mut LayersState)` is a pure mutating transform
on two `&mut` refs — Bevy systems own the `ResMut` lifecycle around
it.

**Cost of lift**: add `serde` + `toml` + `anyhow` to
`cf-device-types`'s deps. All three are workspace pins — no new
third-party intake.

**Tests**: ~300 LOC + 15 cases move with the file unchanged.

### 1.3 `LAYER_SURFACE_PALETTE` — lift to `cf-device-types`

A `&[(f32, f32, f32)]` 5-entry slice currently defined in
`tools/cf-device-design/src/main.rs:744`. Used by:
- `main.rs:1059` (palette tint in non-sim layer render)
- `insertion_sim_ui.rs:37` (`use crate::LAYER_SURFACE_PALETTE`) for
  the sim panel's layer-row tinting
- `tools/cf-sim-research/src/main.rs:64` (duplicated by-value at
  Phase 2 ship per [[project-sim-decouple-refactor-plan]] §2 banked
  posture)

Three consumers, one canonical source candidate. Lifts cleanly to
`cf-device-types` alongside `LAYER_MATERIALS` (both are
material/styling data the design tool + sim panel both read).

### 1.4 `clip_plane` — the recon SURPRISE

**Why the plan missed it**: the plan was scoped against the
sim-coupled Bevy *systems*, not their *material types*. The
clip-plane visualization is independently configurable (slider in
the panel), looks like a CAD feature, easy to overlook. But
`ClipPlaneMaterial` is the `ExtendedMaterial` wrapping every
opaque-mesh entity in the scene — including the cavity, layer
shells, and intruder mesh that the sim systems own.

**Concrete coupling sites** (post-Phase-3 move):
- `spawn_cavity_mesh` (line 778) — `ResMut<Assets<ClipPlaneMaterial>>`
- `update_layer_meshes` (line 950) — same
- `update_cavity_mesh` (line 849) — same (queries it on entities)
- `spawn_intruder_mesh` (line 1128) — same
- `update_intruder_mesh` (line 1275) — same (queries it on entities)

**Module structure** (729 LOC, all `pub(crate)`):
- `ClipPlaneMaterial = ExtendedMaterial<StandardMaterial, ClipPlaneExt>`
  type alias
- `struct ClipPlaneExt` — extended-material data (AsBindGroup)
- `struct ClipPlanePlugin` — Bevy plugin registering the
  ExtendedMaterial<...>Plugin + the per-frame uniform push system
- `struct ClipPlaneState` — Bevy `Resource`, panel-edited
- `fn render_clip_plane_section` — egui section (cf-device-design
  panel-side)
- `fn resolve_plane` — math
- `const DEFAULT_T`, `const DEFAULT_ROLL_RAD`,
  `const WORLD_Z_DEGENERATE_DEG`

**Decision matrix**:
| Option | Cost | Verdict |
|--------|------|---------|
| (a) cf-sim-research depends on cf-device-design | Forbidden — backwards dep, kills the refactor | NO |
| (b) Widen visibility, both binaries import the type from cf-device-design | Same as (a) | NO |
| (c) Lift `ClipPlaneMaterial` + `ClipPlaneExt` + `ClipPlanePlugin` + `ClipPlaneState` + the math + constants to new `cf-device-geometry` shared crate. Keep `render_clip_plane_section` egui UI in cf-device-design (calls the shared types). | Lift + visibility widen | YES |
| (d) Parameterize spawn helpers over `<M: Material>` so each binary picks its own material type | Generics-heavy, breaks the simple `Assets<ClipPlaneMaterial>` pattern, and cf-sim-research still wants clip-plane behavior | NO |

**Picked (c)**. The split-of-concerns mirrors Phase 1's pattern:
data + plugin go shared; UI sections stay binary-local.

`render_clip_plane_section` stays in cf-device-design (a panel
concern); a similar section for cf-sim-research's sim panel could
be added later (out of scope for Phase 3 — defer to a Phase 5
follow-up if the sim viewer ends up wanting independent clipping).

### 1.5 `insertion_sim.rs` — fully self-contained, moves as-is

Zero `use crate::*` / `use super::*` imports at module scope. All
deps are workspace crates that follow the move. Only `use super::*`
inside `#[cfg(test)] mod tests`, which resolves inside the file.

One doc-comment-only `crate::insertion_sim_ui` cross-ref (line 45)
that rewrites trivially on move.

The 5762 LOC are internal complexity, not coupling surface.

### 1.6 `insertion_sim_ui.rs` — 3 coupling points, all addressable

Three `use crate::*` imports at the module scope:

1. `use crate::LAYER_SURFACE_PALETTE;` (line 37) — resolved by §1.3
   lift to cf-device-types.
2. `use crate::insertion_sim::{...};` (lines 39-43) — sibling sim
   module that moves alongside.
3. `use crate::sdf_layers::{CachedScanSdf, CapPlanes};` (line 44) —
   resolved by §1.1 lift to cf-device-geometry.

After Phase 2.5 lands, all three imports rewrite to external
workspace paths. No further coupling.

### 1.7 main.rs — what stays / what moves

**Move to cf-sim-research** (sim-coupled):
- `fn update_cavity_mesh` (line 849) + `struct CavityMeshKey`
- `fn update_layer_meshes` (line 950) + `struct LayerMeshKey`
- `fn spawn_intruder_mesh` (line 1128)
- `fn update_intruder_mesh` (line 1275) + `struct IntruderMeshKey`
- `fn pose_to_bevy_transform` + `fn visible_pose_for_intruder` (helpers)
- `struct LayerSurfaceEntity` + `struct IntruderEntity` markers +
  `const INTRUDER_COLOR` + `const LAYER_SLAB_ALPHA`.

**Duplicate** (needed by both binaries — the cf-device-geometry
home is the cleanest landing, see footnote):
- `fn spawn_cavity_mesh` (line 778) — rest-frame iso extraction, no
  `InsertionSimState`. Parent plan §4 Phase 4 step 4 keeps "cavity
  always rest-frame SDF iso" in cf-device-design post-strip, so the
  CAD-side spawner survives Phase 4. cf-sim-research also needs to
  spawn a cavity entity (which `update_cavity_mesh` then drives).
  Two options: (i) lift `spawn_cavity_mesh` to cf-device-geometry
  alongside `sdf_layers` (it consumes `CachedScanSdf` + `CapPlanes`
  + `ClipPlaneMaterial`, all shared post-2.5), or (ii) duplicate by
  copy + cold-read at Phase 3.3 time. **Lean: (i)** — single source
  of truth, both binaries call it from the Startup chain.
- `fn build_bevy_mesh_from_indexed` + `fn build_bevy_mesh_from_indexed_with_colors`
  (lines 685, 700) — pure `IndexedMesh → bevy::Mesh` adapters with
  optional per-vertex colors. Used by `spawn_cavity_mesh` (stays
  CAD-side AND moves sim-side per above), `update_cavity_mesh`
  (sim-side), `update_layer_meshes` (sim-side). **Lean: lift to
  cf-device-geometry** alongside the cavity spawner so both binaries
  consume the same helper.
- `struct CavityEntity` marker — both binaries spawn a cavity
  entity. Lifts to cf-device-geometry alongside `spawn_cavity_mesh`.
- `const CAVITY_COLOR` — paired with `spawn_cavity_mesh`. Lifts.

**Stay in cf-device-design** (sim-agnostic):
- `fn setup_render_scene` (line 1315) — spawns scan mesh + camera +
  lighting. Cf-sim-research already has its own copy at
  `tools/cf-sim-research/src/main.rs:317`. Phase 3.3 swaps that
  copy's `spawn_face_mesh` for the `ClipPlaneMaterial` path (so the
  scan participates in clipping like cavity/layers do).
- `fn draw_reference_overlays` (line 1366) — axis arrows + centerline
  overlay; cf-sim-research has its own copy too.
- `fn apply_scan_mesh_visibility` (line 1101) — same.
- `fn block_orbit_input_when_over_egui` (line 1701) — same.
- `fn exit_on_esc` (line 1715) — same.
- `fn device_design_panel` (line 1740) — the panel shell; calls
  `insertion_sim_ui::render_insertion_sim_section` as a cross-crate
  fn (the same shape `clip_plane::render_clip_plane_section` will
  use after the lift).
- `fn compute_validations` + `struct DeviceValidations` +
  `struct LayerValidation` + `fn grade_budget` + `enum BudgetStatus`
  + `fn signed_volume_m3` (~lines 380-606) — sim-agnostic
  validations machinery; consumed by `render_validations_section`
  (panel) + `render_layers_section`. Stays. **Note**:
  `compute_validations` reads `&CachedScanSdf` + `&CapPlanes` and
  calls `extract_layer_surface`, so cf-device-design's `run_render_app`
  **keeps** the `CachedScanSdf` + `CapPlanes` resource inserts (and
  the `build_cached_scan_sdf` call at startup) post-Phase-4 — they
  feed validations, not just sim. ~324 ms cf-device-design startup
  cost survives Phase 4.
- `fn render_*_section` UI functions for Scan / Cavity / Layers /
  Validations / Save (the non-sim panel sections).
- `fn triangle_mesh_flat_shaded` — re-exported from `cf-bevy-common`,
  consumed by `setup_render_scene`. Already shared via cf-bevy-common
  — no lift needed.

**System ordering** (no surprises):
- 3 sim systems can run unordered relative to each other (separate
  marker components, separate Local<> keys, separate Query
  filters).
- Startup chain `(setup_render_scene, spawn_cavity_mesh,
  spawn_intruder_mesh).chain()` is load-bearing — `setup_render_scene`
  must run first (creates camera + lights). After move,
  cf-sim-research's `InsertionSimPlugin::build` registers spawns
  with `.after(setup_render_scene)`.

### 1.8 Bevy resource deltas

Phase 2 inserted: `UpAxis`, `RenderScale`, `ScanMesh`, `ScanFilePath`,
`ScanInfo`, `Centerline`, `CavityState`, `LayersState`,
`ScanMeshVisible`.

Phase 3 adds (delta): `CachedScanSdf` (from cf-device-geometry),
`CapPlanes` (from cf-device-geometry), `ClipPlaneState` (from
cf-device-geometry — binds the clip-plane material the sim
systems mount), `InsertionSimState` (registered by `InsertionSimPlugin`).
No conflicts — all new.

cf-device-design's `run_render_app` resource set after Phase 4
strip: **`CachedScanSdf` + `CapPlanes` + `ClipPlaneState` stay**
(needed by `compute_validations` + the CAD-side rest-frame cavity
render + the clip-plane visualization), only `InsertionSimState` +
`InsertionSimPlugin` get dropped. The ~324 ms `build_cached_scan_sdf`
startup cost survives Phase 4 for the validations panel.

**Two-binary cost note**: with cf-sim-research wired in Phase 3 +
cf-device-design still building the SDF for validations, opening
both viewers against the same scan pays the build cost twice. The
cache is in-memory per process, no on-disk artifact to share.
Acceptable per [[project-mesh-sdf-perf-pattern]] (decimation is the
dominant cost, not user-facing).

**Empty `CapPlanes::default()` failure mode**: `CapPlanes::default()`
returns `planes: vec![]`. Calling `build_cached_scan_sdf` with an
empty cap-planes vec is fine (the no-caps path is the optimized
fast path); calling `extract_layer_surface` with `cap_planes:
&[]` resolves to a closed-scan offset iso (no open-mouth carve).
cf-sim-research's existing Phase-2 behavior — discard parsed caps
after logging the count — would put it on this empty-caps path,
which may not match a user expectation that "ran cf-device-design
with prep.toml" → "cf-sim-research sees same cavity opening." Phase
3.3 must propagate the parsed caps into the `CapPlanes` resource
(not discard them). Cheap fix at impl time.

---

## 2. Phase 2.5 sub-leaf ladder

Each sub-leaf ends with a green workspace build. Order minimizes
churn: smallest leaves first, biggest lift last.

**Migration-safety invariant**: after each sub-leaf, `cargo build`
+ `cargo test` are green on the whole workspace, **including
cf-device-design with no half-state import drift**. This means
Phase 2.5 can ship and sit indefinitely if Phase 3 stalls (workshop
iter-1 cast intervenes, etc.) without leaving the CAD binary in a
broken state.

### 2.5.a — Lift `design_toml` + `LAYER_SURFACE_PALETTE` to `cf-device-types`

**Scope**: move `tools/cf-device-design/src/design_toml.rs` →
`design/cf-device-types/src/design_toml.rs`. Add `serde` + `toml` +
`anyhow` to `cf-device-types/Cargo.toml`. Add `pub const
LAYER_SURFACE_PALETTE: &[(f32, f32, f32)]` to
`cf-device-types/src/design.rs` (alongside `LAYER_MATERIALS`). Drop
the duplicated copy from `tools/cf-sim-research/src/main.rs:64`
(import from cf-device-types). Update cf-device-design's
`main.rs:744` (delete the const, add import) and `insertion_sim_ui.rs:37`.

**Tests**: existing 15 cases move unchanged.

**Estimate**: 30-60 min (mostly mechanical).

### 2.5.b — Create `cf-device-geometry` crate

**Scope**: new `design/cf-device-geometry/` directory with
`Cargo.toml` + `src/lib.rs`. Workspace member entry. xtask grader
exemption (Bevy-using, mirrors cf-device-types posture).
`bevy = { workspace = true, default-features = false }` minimal
feature set (`Resource` derive only at this stage — 2.5.d adds
`bevy/bevy_pbr` + `bevy/bevy_render` for ExtendedMaterial). Lenient
lint posture per [[feedback-lint-posture-on-extract]].

Empty lib at this stage — populated by 2.5.c + 2.5.d.

**Estimate**: 20 min.

### 2.5.c — Lift `sdf_layers` + sharing helpers to `cf-device-geometry`

**Scope**: move `tools/cf-device-design/src/sdf_layers.rs` →
`design/cf-device-geometry/src/sdf.rs` (or keep `sdf_layers.rs`
name). Widen `pub(crate)` → `pub` on the public surface. Add
`bevy` + mesh-sdf + mesh-offset + cf-cap-planes + cf-design +
mesh-types + mesh-repair + meshopt + nalgebra + anyhow deps to
cf-device-geometry/Cargo.toml. `Resource` derives on `CachedScanSdf`
+ `CapPlanes` stay un-gated (Bevy is a hard dep of this crate).

**Also lift** (per §1.7 "Duplicate"): `fn spawn_cavity_mesh` +
`fn build_bevy_mesh_from_indexed` + `fn build_bevy_mesh_from_indexed_with_colors`
+ `struct CavityEntity` + `const CAVITY_COLOR`. These are
two-binary consumers; lifting alongside `sdf_layers` keeps the
shared rest-frame cavity rendering surface in one place.
`spawn_cavity_mesh`'s `Res<ClipPlaneMaterial>` parameter resolves
after 2.5.d's clip-plane lift — order matters here, so 2.5.d lands
before the consumer-site updates in 2.5.c's tail, OR 2.5.c does
the mesh-builder lift first and defers `spawn_cavity_mesh` to a
2.5.e sub-leaf. Implementer's choice at recon-doc level; the
material-type ordering is the only constraint.

Update cf-device-design's ~40 consumer sites (`sdf_layers::*` →
`cf_device_geometry::*`). Delete the now-empty `sdf_layers` module
declaration from `lib.rs` / `main.rs`.

**Tests**: 35 tests in `sdf_layers.rs` move with the file.

**Estimate**: 2-3 hours (~40 call-site updates + dep wiring + the
shared rendering helpers).

### 2.5.d — Lift `clip_plane` (full module) to `cf-device-geometry`

**Scope**: lift the **entire** `clip_plane.rs` module:
`ClipPlaneMaterial` + `ClipPlaneExt` + `ClipPlanePlugin` +
`ClipPlaneState` + `fn resolve_plane` + `struct ResolvedPlane` +
the 3 constants + **`fn render_clip_plane_section`** (the egui
panel section). The section is pure `&mut ClipPlaneState` + `bool`
input → egui output; no cf-device-design-internal state. Lifting
it gives cf-sim-research a clipping slider out-of-the-box (instead
of deferring to a Phase 5 follow-up — the original recon called
that out as R5 then walked it back here).

Widen `pub(crate)` → `pub`. Add `bevy/bevy_pbr` + `bevy/bevy_render`
to cf-device-geometry's Bevy feature set (for ExtendedMaterial
machinery — see [[project-cf-device-design-clip-plane-arc]] for the
Bevy-0.18 `ExtendedMaterial` shader-def gotcha).

Update cf-device-design's `main.rs` + delete the local `clip_plane`
module declaration; replace imports with `use cf_device_geometry::clip_plane::*`.

**Tests**: existing clip_plane tests (math + plane resolution) move
with the module.

**Estimate**: 1-2 hours.

### 2.5 totals

**Surface area**: ~3500 LOC moved across 4 sub-leaves (`design_toml`
666 + `sdf_layers` 2091 + `clip_plane` 729 + `LAYER_SURFACE_PALETTE`
+ the cavity-rendering helpers ~150 LOC ≈ 3636).

**Estimate**: 4-7 hours, **1-2 sessions** (revised up from 3-5
hours / 1 session — the additional cavity-render-helper lifts in
2.5.c + the full clip_plane module in 2.5.d add load, and each
sub-leaf gets its own brief cold-read pass per
[[feedback-cold-read-review-post-ship]] applied to non-trivial
diffs).

**Branches**: single branch `refactor/sim-decouple-phase-2.5`,
sub-leaf commits.

---

## 3. Phase 3 (revised) scope

After Phase 2.5 lands, Phase 3 is purely the sim move:

### 3.1 Copy sim modules to cf-sim-research

- `tools/cf-device-design/src/insertion_sim.rs` →
  `tools/cf-sim-research/src/insertion_sim.rs` (verbatim;
  doc-comment cross-ref edit only).
- `tools/cf-device-design/src/insertion_sim_ui.rs` →
  `tools/cf-sim-research/src/insertion_sim_ui.rs` (rewrite 3
  import lines to use cf-device-types + cf-device-geometry).
- Add deps: `sim-soft`, `sim-ml-chassis`, `cf-design`,
  `cf-device-geometry`, `mesh-sdf`, `mesh-offset` to
  `tools/cf-sim-research/Cargo.toml`.

### 3.2 Copy sim-coupled Bevy systems + helpers

Per §1.7 "Move to cf-sim-research" list — the 4 sim-coupled
systems (`update_cavity_mesh`, `update_layer_meshes`,
`spawn_intruder_mesh`, `update_intruder_mesh`) + the 2 sim-side
helpers (`pose_to_bevy_transform`, `visible_pose_for_intruder`) +
3 key structs (`CavityMeshKey`, `LayerMeshKey`, `IntruderMeshKey`)
+ 2 marker components (`LayerSurfaceEntity`, `IntruderEntity`) + 2
consts (`INTRUDER_COLOR`, `LAYER_SLAB_ALPHA`). `spawn_cavity_mesh`
+ `CavityEntity` + `CAVITY_COLOR` + `build_bevy_mesh_from_indexed*`
lifted in Phase 2.5.c, so they're already in `cf-device-geometry`
when Phase 3 starts. All copy verbatim; only path imports change.

### 3.3 Wire into cf-sim-research's run_render_app

- Insert `CachedScanSdf` + `CapPlanes` resources (built at startup
  the way cf-device-design does today — call
  `cf_device_geometry::build_cached_scan_sdf(scan, cap_planes,
  LAYER_PREVIEW_CELL_SIZE_M, LAYER_GRID_MARGIN_M)` in `main`).
- Register `InsertionSimPlugin`.
- Register `ClipPlanePlugin` (since the sim entities mount
  `ClipPlaneMaterial`).
- Replace `setup_render_scene`'s `spawn_face_mesh` call with the
  ClipPlaneMaterial path that cf-device-design's
  `setup_render_scene:1315` uses (mounts the scan with a
  `ClipPlaneMaterial`, not a `StandardMaterial`, so the clip plane
  affects it).
- Startup chain: `(setup_render_scene, spawn_cavity_mesh,
  spawn_intruder_mesh).chain()`.
- Update set: add `update_cavity_mesh, update_layer_meshes,
  update_intruder_mesh` to the existing tuple.
- Egui pass: replace Phase 2's read-only `sim_research_panel` with
  a panel that includes Scan Info + Cavity (now editable via
  `update_cavity_mesh`) + Layers + the sim section
  (`insertion_sim_ui::render_insertion_sim_section`).
- `.design.toml` ingest: now possible since `design_toml` lifted
  in 2.5.a. Wire `load_design_toml` + `apply_design_toml` into
  `main` between `default_for_scan` and `run_render_app` (the
  Phase-2 polish commit's [[feedback-cold-read-review-post-ship]]
  pre-positioned this splice point).

### 3.4 Verify

- `cargo build -p cf-sim-research` green.
- `cargo build -p cf-device-design` green (cf-device-design still
  has its sim code — duplication intentional through Phase 4).
- `cargo test -p cf-sim-research --release` green.
- Visual smoke: load iter-1 sock, run the sim panel, verify
  heat-map + sliding intruder render.
- Workshop loop unaffected (cf-device-design's panel still works).

### 3.5 Phase 3 estimate

**Surface area**: ~10 000 LOC copied + ~500 LOC of wiring.
**Sessions**: 2.

**Honest framing**: Phase 2.5 (1-2 sessions) + revised Phase 3
(2 sessions) = **3-4 total sessions**, matching or slightly
exceeding the original Phase 3 estimate's 2-3 session range. Phase
2.5 does NOT bank session work — it **derisks Phase 3** by landing
the cross-crate moves in a state where Phase 3 itself is a pure
copy + wire job with no architectural decisions remaining.
Cross-arc surprises (the clip_plane material coupling, the
cavity-spawner classification, the validations SDF-dependency)
land in 2.5 instead of mid-Phase-3, which is the actual win.

---

## 4. Risks + mitigations

**R1 — `ClipPlaneMaterial` ExtendedMaterial dep is heavier than
expected.** The `bevy_pbr` + `bevy_render` features pull in real
weight to cf-device-geometry. Mitigation: gate behind a `clip-plane`
feature flag in cf-device-geometry so consumers that only want
SDF compute (e.g., a future cf-cast-cli SDF consumer) don't take
the rendering deps.

**R2 — `LAYER_SURFACE_PALETTE` consumer drift between binaries.**
Phase 2's cf-sim-research has a by-value duplicate that 2.5.a
deletes; if a developer touches it during the Phase 2.5 work they
might forget to update both. Mitigation: 2.5.a deletes the duplicate
in the same commit that adds the cf-device-types const — atomic.

**R3 — `apply_design_toml` ResMut lifecycle changes between
cf-device-design and cf-sim-research.** Phase 3.3 wires
`apply_design_toml` into cf-sim-research's `main`, but
cf-device-design already has it threaded through a slightly
different lifecycle (load on Bevy startup, save on panel button
press). Mitigation: the function is pure on `&mut CavityState +
&mut LayersState`; each binary chooses its own lifecycle.

**R4 — Bevy feature-set drift between cf-device-geometry and the
binaries that consume it.** If cf-device-geometry pins a feature
the binaries don't carry, downstream rebuilds churn. Mitigation:
cf-device-geometry uses `default-features = false` and explicit
feature additions; binary Cargo.toml features always superset the
lib's. Mirror cf-device-types' precedent.

**R5 — clip_plane UI parity** ~~(originally: defer cf-sim-research
clipping to Phase 5)~~ **— resolved at the recon-doc stage.**
Updated 2.5.d lifts `render_clip_plane_section` to
cf-device-geometry alongside the rest of the module, so both
binaries get an editable clipping slider without further work. The
section is pure `&mut ClipPlaneState` + `bool` → egui — no
cf-device-design-internal coupling.

**R6 — Workshop iter-1 cast mid-arc.** Phase 2.5 + Phase 3 don't
touch cf-cast-cli or the workshop loop. Phase 4 is where breaking
risk lives (cf-device-design loses sim panel). Mitigation: Phase 4
defers until after iter-1 cast lands.

**R7 — cf-cast-cli is the third consumer of `cf-device-geometry`.**
cf-cast-cli already has open SDF-coupling work pending (see
[[project-mesh-sdf-oracle-decomposition-spec]] D.5 — workshop unblock
TODO + [[project-cf-cast-plug-layer-0-watertight-discovery]]).
Whenever cf-cast-cli's SDF arc resumes, it will want
`build_cached_scan_sdf` + `extract_layer_surface` too — a 3rd
consumer of cf-device-geometry, not a hypothetical future. R1's
`clip-plane` feature flag (gate the Bevy rendering deps behind a
non-default feature) lets cf-cast-cli consume the SDF compute path
without paying the rendering weight. Mitigation: confirmed R1
already addresses this.

---

## 5. Open questions for implementation

**Q5.1 — `cf-device-geometry` package name**. Final? Alternatives:
`cf-device-render`, `cf-device-sdf`, `cf-scan-geometry`. The lean is
`cf-device-geometry` because it captures both `sdf_layers` (SDF
compute) and `clip_plane` (rendering) under one "device-side
geometric primitives" umbrella. Confirm at implementation time.

**Q5.2 — clip-plane feature flag on cf-device-geometry**. Per R1
mitigation, gate the clip-plane sub-module behind a `clip-plane`
feature. Both binaries enable it. Confirm at 2.5.d time.

**Q5.3 — `build_bevy_mesh_from_indexed*` helper destination —
RESOLVED at recon-doc revision time**. The original recon said
"sim-coupled in practice, move to cf-sim-research" — falsified by
the cold-read pass: `spawn_cavity_mesh` is rest-frame (no
`InsertionSimState`) and STAYS in cf-device-design per parent plan
§4 Phase 4 step 4, so a cf-device-design caller of these helpers
survives Phase 4. New verdict: lift to **cf-device-geometry**
alongside `spawn_cavity_mesh` (per §1.7 Duplicate block + §2.5.c).
Both binaries consume from the shared crate; no duplication.

**Q5.4 — Save panel for cf-sim-research**. Phase 3 ingests
`.design.toml` (read-only — sim viewer doesn't edit designs per
[[project-sim-decouple-refactor-plan]] §3 Q2 default lean). No save
button in cf-sim-research. If the lean changes later, the
`design_toml` shared crate already has `save_design_toml`. Confirm
at implementation.

---

## 6. Pointers

- Plan: `docs/SIM_DECOUPLE_REFACTOR_PLAN.md` §4 Phase 3.
- Phase 2 ship: `refactor/sim-decouple-phase-2` branch, origin
  `de397b4a`.
- Phase 1 output: `design/cf-device-types/` (lift destination for
  `design_toml` + `LAYER_SURFACE_PALETTE`).
- New crate destination: `design/cf-device-geometry/` (to be
  created in 2.5.b).
- Sim modules: `tools/cf-device-design/src/insertion_sim.rs` +
  `insertion_sim_ui.rs`.
- Sim-coupled Bevy systems: `tools/cf-device-design/src/main.rs`
  at `update_layer_meshes` @ 950, `update_cavity_mesh` @ 849,
  `update_intruder_mesh` @ 1275, `spawn_cavity_mesh` @ 778,
  `spawn_intruder_mesh` @ 1128, `LayerMeshKey` @ 659.
- Recon evidence: 4 sibling-agent reports, summarized in §1.
  Logs not retained (synthesized into this doc).

---

End of recon. **Phase 2.5 mini-arc next** — 4 sub-leaves, ~3500
LOC moved, 1-2 sessions. Then Phase 3 sim move (2 sessions). Then
Phase 4 strip from cf-device-design (1-2 sessions). Then Phase 5
verify + document (1 session). **Total remaining: 5-7 sessions
post-Phase-2**, matching the parent plan's "4-5 remaining"
estimate (which counted Phase 2.5 as part of Phase 3's range).
