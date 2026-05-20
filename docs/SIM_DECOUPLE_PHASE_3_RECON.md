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
**copy `insertion_sim.rs` + `insertion_sim_ui.rs` + 7 Bevy systems
(`spawn_cavity_mesh` / `spawn_intruder_mesh` /
`update_cavity_mesh` / `update_layer_meshes` / `update_intruder_mesh`
+ the two `build_bevy_mesh_from_indexed*` helpers + `LayerMeshKey`
+ `pose_to_bevy_transform` + `visible_pose_for_intruder`) into
cf-sim-research, wire the `InsertionSimPlugin` into its
`run_render_app`.** Phase 4 then strips cf-device-design's copies.

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
self-contained against external workspace crates (`mesh-sdf`,
`mesh-offset`, `cf-cap-planes`, `cf-design`, `mesh-types`,
`mesh-repair`, `meshopt`, `nalgebra`, `anyhow`).

**Bevy coupling**: `Resource` derive on `CachedScanSdf` + `CapPlanes`
only. Trivially gated behind a `bevy` feature flag.

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
the schema types it serializes.

**Bevy coupling**: NONE. Doesn't import `bevy::*`, doesn't derive
`Resource`, doesn't touch ECS. `apply_design_toml(&DesignToml,
&mut CavityState, &mut LayersState)` is a pure mutating transform
on two `&mut` refs — Bevy systems own the `ResMut` lifecycle around
it.

**Cost of lift**: add `serde` + `toml` + `anyhow` to
`cf-device-types`'s deps. All three are workspace pins — no new
third-party intake.

**Tests**: ~300 LOC + 11 cases move with the file unchanged.

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
- `fn spawn_cavity_mesh` (line 778)
- `fn update_cavity_mesh` (line 849) + `struct CavityMeshKey`
- `fn update_layer_meshes` (line 950) + `struct LayerMeshKey`
- `fn spawn_intruder_mesh` (line 1128)
- `fn update_intruder_mesh` (line 1275) + `struct IntruderMeshKey`
- `fn pose_to_bevy_transform` + `fn visible_pose_for_intruder` (helpers)
- `fn build_bevy_mesh_from_indexed` + `fn build_bevy_mesh_from_indexed_with_colors`
  (sim-coupled in practice — after the move, no cf-device-design
  call site remains)
- `struct CavityEntity`, `struct LayerSurfaceEntity`,
  `struct IntruderEntity` markers + `const INTRUDER_COLOR` +
  `const CAVITY_COLOR` + `const LAYER_SLAB_ALPHA`

**Stay in cf-device-design** (sim-agnostic):
- `fn setup_render_scene` (line 1315) — spawns scan mesh + camera +
  lighting. Cf-sim-research already has its own copy at
  `tools/cf-sim-research/src/main.rs:317`.
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
  (panel) + `render_layers_section`. Stays.
- `fn render_*_section` UI functions for Scan / Cavity / Layers /
  Validations / Save (the non-sim panel sections).

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
cf-device-geometry — gated to bind the clip-plane material the sim
systems mount), `InsertionSimState` (registered by `InsertionSimPlugin`).
No conflicts — all new.

cf-device-design's `run_render_app` resource set after Phase 4
strip becomes: drop `CachedScanSdf` / `CapPlanes` / `InsertionSimState`
inserts (or rather, the `CachedScanSdf` + `CapPlanes` inserts stay
for the validations panel — see §1.7 "stay" — but no
`InsertionSimPlugin`). TBD at Phase 4 design-time.

---

## 2. Phase 2.5 sub-leaf ladder

Each sub-leaf ends with a green workspace build. Order minimizes
churn: smallest leaves first, biggest lift last.

### 2.5.a — Lift `design_toml` + `LAYER_SURFACE_PALETTE` to `cf-device-types`

**Scope**: move `tools/cf-device-design/src/design_toml.rs` →
`design/cf-device-types/src/design_toml.rs`. Add `serde` + `toml` +
`anyhow` to `cf-device-types/Cargo.toml`. Add `pub const
LAYER_SURFACE_PALETTE: &[(f32, f32, f32)]` to
`cf-device-types/src/design.rs` (alongside `LAYER_MATERIALS`). Drop
the duplicated copy from `tools/cf-sim-research/src/main.rs:64`
(import from cf-device-types). Update cf-device-design's
`main.rs:744` (delete the const, add import) and `insertion_sim_ui.rs:37`.

**Tests**: existing 11 cases move unchanged.

**Estimate**: 30-60 min (mostly mechanical).

### 2.5.b — Create `cf-device-geometry` crate

**Scope**: new `design/cf-device-geometry/` directory with
`Cargo.toml` + `src/lib.rs`. Workspace member entry. xtask grader
exemption (Bevy-using, mirrors cf-device-types posture).
`bevy = { workspace = true, default-features = false }` minimal
feature set (`Resource` derive only — no rendering deps yet). Lenient
lint posture per [[feedback-lint-posture-on-extract]].

Empty lib at this stage — populated by 2.5.c + 2.5.d.

**Estimate**: 20 min.

### 2.5.c — Lift `sdf_layers` to `cf-device-geometry`

**Scope**: move `tools/cf-device-design/src/sdf_layers.rs` →
`design/cf-device-geometry/src/sdf.rs` (or keep `sdf_layers.rs`
name). Widen `pub(crate)` → `pub` on the public surface. Add
mesh-sdf / mesh-offset / cf-cap-planes / cf-design / mesh-types /
mesh-repair / meshopt / nalgebra / anyhow deps to
cf-device-geometry/Cargo.toml. Bevy-feature-gate the `Resource`
derives on `CachedScanSdf` + `CapPlanes` (currently un-gated; gate
is cosmetic — Bevy-feature is on by default in cf-device-geometry
since both binaries need it).

Update cf-device-design's 30+ consumer sites (`sdf_layers::*` →
`cf_device_geometry::*`). Delete the now-empty `sdf_layers` module
declaration from `lib.rs` / `main.rs`.

**Tests**: ~10 tests in `sdf_layers.rs` move with the file.

**Estimate**: 1-2 hours (30+ call-site updates + dep wiring).

### 2.5.d — Lift `clip_plane` to `cf-device-geometry`

**Scope**: lift the non-UI portion of `clip_plane.rs`:
`ClipPlaneMaterial` + `ClipPlaneExt` + `ClipPlanePlugin` +
`ClipPlaneState` + `fn resolve_plane` + `struct ResolvedPlane` +
the 3 constants. Keep `fn render_clip_plane_section` in
cf-device-design (panel UI). Widen `pub(crate)` → `pub`. Add
Bevy-feature-flagged `bevy/bevy_pbr` + `bevy/bevy_render` to
cf-device-geometry (for ExtendedMaterial machinery).

Update cf-device-design's `main.rs` + `clip_plane.rs` to import the
moved primitives (`use cf_device_geometry::clip_plane::*`).

**Tests**: existing clip_plane tests move with the math.

**Estimate**: 1-2 hours.

### 2.5 totals

**Surface area**: ~3500 LOC moved across 4 sub-leaves.
**Estimate**: 3-5 hours, 1 session.
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

Per §1.7 "Move" list — ~9 functions + 3 marker components + 4 key
structs + 4 consts. All copy verbatim; only path imports change.

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
**Sessions**: 2 (was 2-3 in the original plan; one session banked
by Phase 2.5 doing the lifts upstream).

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

**R5 — clip_plane's `render_clip_plane_section` cross-crate
double-call.** Phase 3 wires cf-sim-research to call the same
section in its panel (so the sim viewer also gets clipping). But
the function lives in cf-device-design (per §1.4). cf-sim-research
can't reach in — same backwards-dep problem. Mitigation: defer
clipping in cf-sim-research's panel to a Phase 5 follow-up; for
Phase 3, the clip-plane works on cf-sim-research's scan/cavity/
layer/intruder shells via the `ClipPlanePlugin` (the plugin's
per-frame uniform push system runs without any UI), it just has no
slider in the cf-sim-research panel. Adequate for sim research.

**R6 — Workshop iter-1 cast mid-arc.** Phase 2.5 + Phase 3 don't
touch cf-cast-cli or the workshop loop. Phase 4 is where breaking
risk lives (cf-device-design loses sim panel). Mitigation: Phase 4
defers until after iter-1 cast lands.

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

**Q5.3 — `build_bevy_mesh_from_indexed*` helper destination**. The
audit (§1.7) says they're sim-coupled in practice (only consumed
by sim-coupled systems). Move to cf-sim-research. Phase 4 confirms
no cf-device-design caller remains.

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
LOC moved, 1 session. Then Phase 3 sim move (2 sessions). Then
Phase 4 strip from cf-device-design (1-2 sessions). Then Phase 5
verify + document (1 session).
