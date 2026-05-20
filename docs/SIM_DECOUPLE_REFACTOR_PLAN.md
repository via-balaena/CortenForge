# Sim-decouple refactor — full plan

> **Status**: ACTIVE — Phase 1 + Phase 2 + Phase 2.5 SHIPPED
> 2026-05-19 on branches `refactor/sim-decouple-phase-1` +
> `refactor/sim-decouple-phase-2` + `refactor/sim-decouple-phase-2.5`
> respectively; Phase 3 (move sim code) is next. Originally
> bookmarked 2026-05-19 LATE-NIGHT after A1's
> in-session scoping revealed the sim is woven deeply into
> cf-device-design's layer/cavity/intruder rendering. The original
> gameplan §3 A1 "~1 session feature-flag" estimate was wrong; this
> doc sequences the real arc.
>
> **Goal** (from [[project-program-gameplan]]): cf-device-design
> ships as a CAD-only tool (scan → layer → mold geometry). Sim
> research lives in its own crate with its own roadmap. Default
> cf-device-design build has zero sim-soft dependency.
>
> **Predecessors**:
> - `docs/PROGRAM_GAMEPLAN.md` §3 A1 — the gameplan ask that
>   sized this wrong
> - `docs/SIM_ARCHITECTURE_AUDIT.md` — the audit that motivated
>   the decouple

---

## TL;DR

Extract insertion-sim from `tools/cf-device-design/` to a new
`tools/cf-sim-research/` crate. Move the FEM-coupled rendering
systems (heat-map projection, deformed-shells path, intruder slide)
out of `tools/cf-device-design/src/main.rs`. Multi-session arc —
realistically **6-9 sessions across 5 phases**, calendar 1-2 weeks
if focused.

---

## 1. Why the gameplan's ~1-session estimate was wrong

A1 was scoped as "feature-flag the panel" — cfg-gate `mod
insertion_sim` + plugin registration. In practice the sim is
deeply intertwined with the layer rendering surface:

- `update_layer_meshes` (main.rs:950) — reads `InsertionSimState`
  for `heat_map_on`, `scalar_mode`, `last_run_generation`,
  `displayed_step`, `show_deformed`, and `last_run` (the FEM
  results). Applies heat-map projection per-MC-vertex; falls
  through to deformed-shells path when `show_deformed` is on; falls
  through to rest-frame SDF iso when both are off.
- `update_cavity_mesh` (main.rs:849) — reads same state for the
  deformed-cavity path.
- `update_intruder_mesh` (main.rs:1275) — reads same state for the
  slide-pose-at-displayed-step rendering.
- `device_design_panel` (main.rs:1740) — calls
  `render_insertion_sim_section` inside the panel render system.
- `LayerMeshKey` struct (main.rs:659) — carries
  `scalar_mode: insertion_sim_ui::ScalarMode` as a diff field for
  layer-mesh cache invalidation.
- `run_render_app` (main.rs:2032) — registers
  `insertion_sim_ui::InsertionSimPlugin` which inserts the
  `InsertionSimState` resource and runs the FEM-poll task.

(Line numbers refreshed post-Phase-1 extraction, 2026-05-19. They
will drift again under cf-device-design's churn; treat as "find by
function name, not line.")

Feature-gating these via `#[cfg(feature = "research-mode")]` would
require ~10-15 cfg gates and would create runtime-resource-missing
panics (the layer-render systems require `InsertionSimState` to
exist; without the plugin, the resource doesn't get registered).

The CORRECT refactor is structural: separate the CAD surface from
the sim surface into distinct binaries, with shared data types in
a lib crate.

---

## 2. Target architecture

```
┌─────────────────────────┐    ┌──────────────────────────┐
│  cf-device-design       │    │  cf-sim-research         │
│  (CAD tool, default)    │    │  (sim tool, --features?  │
│                         │    │   or default-on for      │
│  Scan + Layers + Mold   │    │   sim-research builds)   │
│  No sim-soft dep        │    │                          │
│                         │    │  Scan + Layers + Sim     │
│  - egui panel UI        │    │  Depends on sim-soft     │
│  - palette-tint layers  │    │                          │
│  - rest-frame cavity    │    │  - egui panel UI         │
│  - static intruder      │    │  - heat-map layers       │
│  - mold output via      │    │  - deformed-cavity path  │
│    cf-cast-cli          │    │  - slide intruder        │
│                         │    │  - FEM polling           │
│                         │    │  - sim panel             │
└──────────┬──────────────┘    └──────────┬───────────────┘
           │                              │
           │      ┌────────────────┐      │
           └─────►│ cf-device-types├◄─────┘
                  │  (shared data) │
                  │                │
                  │  CavityState   │
                  │  LayersState   │
                  │  SimDesign     │
                  │  Centerline    │
                  │  CapPlanes     │
                  │  ScalarMode    │
                  │  …             │
                  └────────────────┘
                          ▲
                          │
                  ┌───────┴────────┐
                  │ cf-viewer-     │
                  │   layer-render │
                  │  (shared scene │
                  │   helpers)     │
                  │                │
                  │  build_layer_  │
                  │    mesh()      │
                  │  build_cavity_ │
                  │    mesh()      │
                  │  …             │
                  └────────────────┘
```

**Key decisions** (some open at bookmark stage):

- **Shared types crate**: `cf-device-types` (or similar) carries
  CavityState, LayersState, ScalarMode, etc. Both binaries depend
  on it.
- **Shared rendering helpers**: a small lib (could be in
  cf-device-types or a separate `cf-layer-render`) provides
  build-layer-mesh / build-cavity-mesh helpers that BOTH binaries
  use. cf-device-design calls these with `heat_map: None`;
  cf-sim-research calls them with `heat_map: Some(scalars)`.
- **TOML I/O contract**: cf-sim-research reads cf-device-design's
  design TOML (the same one already shipped) as input. No live
  state sharing; the two tools communicate via files. Workshop
  loop: open cf-device-design → tweak layers → save TOML → open
  cf-sim-research → load TOML → run sim → inspect.
- **Bevy resource overlap**: cf-sim-research has its own
  CavityState/LayersState resources (loaded from TOML, possibly
  editable in the sim-research UI for layer-tweaking-with-live-sim
  workflow). cf-device-design's resources are isolated.

---

## 3. Architecture questions (Q1 + Q3 RESOLVED by Phase 2; Q2 + Q4 still open)

**Q1 — does cf-sim-research have a UI, or is it headless?** ✓ RESOLVED by Phase 2 — Option C

Phase 2 shipped a minimal Bevy + egui viewport (scan + centerline +
read-only cavity/layer-stack readout panel). Phase 3 layers the sim
panel onto this same minimal viewport rather than going headless or
mirroring cf-device-design's full UI surface.

Original options (preserved for audit trail):
- Option A: full Bevy + egui UI mirroring cf-device-design's
  layout, with sim panel added. Heavy weight; lots of duplicated
  rendering code.
- Option B: headless CLI tool — `cf-sim-research --design design.toml
  --scan scan.stl` runs sim, writes outputs to files (heat-map
  STLs, force traces, etc.). Lightweight; sim researchers can
  visualize via existing tools.
- Option C: minimal Bevy UI with a sim-results-only viewport (load
  prep.toml + design.toml + scan, render sim outputs read-only).

**Q2 — should cf-sim-research enable layer-tweaking-with-live-sim?** (still open)

A future "research mode" might want the workflow: tweak layers in
cf-sim-research's UI, see sim re-run live (subject to wall-clock).
This is what the current cf-device-design panel supports.

- Option A (recommended): cf-sim-research is read-only WRT design.
  Layer tweaking stays in cf-device-design. If the researcher
  wants a new design, they go back to cf-device-design, edit,
  save, reopen in cf-sim-research.
- Option B: cf-sim-research has its own (duplicated) layer
  controls, doubling the UI surface area. More flexible but heavier.

Default lean: **Option A**. Read-only sim viewer; design happens
elsewhere.

**Q3 — shared rendering helpers — extract to lib crate or duplicate?** ✓ RESOLVED by Phase 2 — neither, for now

Phase 2 consumed cf-viewer's existing `spawn_face_mesh` directly
(no new crate, no duplication). Phase 3 brings the sim-coupled
layer-mesh + cavity-mesh helpers in via copy-then-dedupe (Phase 4
strips cf-device-design's copies); only then will the
extract-to-lib-vs-keep-in-cf-viewer question become live again. If
the helpers stay binary-local through Phase 4's strip, no new
crate is needed; if they need to be shared with a third binary,
extract then.

Original options (preserved for audit trail):
- Option A: extract to a new `cf-layer-render` crate or expand
  `cf-viewer`. Both binaries depend on it. DRY.
- Option B: duplicate the helpers between the two binaries.
  Simpler at extraction time, technical debt long-term.

**Q4 — feature-flag the sim within cf-sim-research, or always-on?** (still open)

Once extracted, cf-sim-research could itself have a feature flag
for the sim, or always include it (since sim is its raison d'être).

- Option A: always-on. cf-sim-research IS the sim tool;
  feature-flagging within it is redundant.
- Option B: feature-flag for cargo-doc-no-sim convenience.

Default lean: **Option A**.

---

## 4. Migration sequence

Five phases. Each phase ends with a green workspace build.

### Phase 1 — extract shared types to `cf-device-types` ✓ SHIPPED 2026-05-19

**Status**: SHIPPED on branch `refactor/sim-decouple-phase-1` in a
single session — mostly mechanical, no type-coupling surprises beyond
the slacker module's catalog test (which moved with the data via
`crate::LAYER_MATERIALS` after re-export).

**Outcome**:
- New crate at `design/cf-device-types/` (workspace `design/cf-*`
  convention, not `lib/`); Bevy-using because the moved types
  include Bevy `Resource`s; xtask grader exempts it via the
  cf-bevy-common precedent (`xtask/src/grade.rs::applies_to_crate`).
- Four topical submodules: `scan` (ScanMesh / ScanFilePath /
  ScanMeshVisible / ScanInfo / Centerline), `design` (CavityState /
  LayerSpec / LayersState + LAYER_MATERIALS + LAYER_COUNT_MAX +
  material_density + CAVITY_DEFAULT_INSET_M), `slacker` (TB curve
  data + Support / Point / ShoreHardness / ShoreScale / Tack +
  resolve_slacker_fraction), `sim` (SimDesign / SimLayer /
  SlackerResolution / ScalarMode / SimMode).
- cf-device-design imports from `cf_device_types::*`; intra-crate
  references to moved items (`crate::CavityState` etc.) updated
  across main.rs / insertion_sim.rs / insertion_sim_ui.rs /
  design_toml.rs / clip_plane.rs.
- Two redundant slacker-resolution tests removed from main.rs (now
  covered in `cf-device-types/src/slacker.rs::tests`).
- 9 tests in cf-device-types + 164 in cf-device-design (release,
  zero regressions). cargo fmt + clippy clean (workspace-noise
  pedantic warnings carried over from the pre-move code; no new
  noise).

**Posture banked**:
- No transitional re-export shim — one-commit import swap was cheap
  and keeps the diff understandable.
- `slacker` module re-exported as a module (not flattened) so
  consumers write `slacker::Support` / `slacker::ShoreScale::A` etc.
  — preserves the original call-site shape.

**Sub-leg estimate confirmed**: 1 session (mostly mechanical).

### Phase 2 — create `tools/cf-sim-research/` skeleton ✓ SHIPPED 2026-05-19

**Status**: SHIPPED on branch `refactor/sim-decouple-phase-2` in a
single session. The new binary loads a cleaned scan + `.prep.toml`
centerline + cap-plane count, then opens a Bevy + egui window
showing the scan mesh, axis-arrow + centerline overlay, and a
right-side panel listing the default cavity + layer stack with
palette-tinted swatches per layer row. No sim wiring yet (Phase 3);
no 3D layer shells yet (the per-iso MC extraction needs mesh-sdf +
mesh-offset, which Phase 3 brings in).

**Outcome**:
- New crate at `tools/cf-sim-research/` (Cargo.toml + src/main.rs,
  ~690 LOC including 12 unit tests); workspace member entry added
  after cf-device-design's block; xtask grader exempts the new
  crate via the cf-device-design / cf-cast-cli precedent
  (`xtask/src/grade.rs::applies_to_crate`).
- CLI shape mirrors cf-device-design: positional `cleaned_stl` +
  optional `--design <PATH>` + optional `--prep-toml <PATH>`.
  Path-resolution helpers (`resolve_prep_toml_path` /
  `resolve_design_toml_path`) duplicate cf-device-design's
  stem-stripping convention so both viewers point at the same
  companions from the same cleaned STL.
- Deps: `cf-device-types` (Phase 1 output), `cf-viewer`,
  `cf-bevy-common`, `mesh-io`, `mesh-types`, `nalgebra`,
  `cf-cap-planes` (parse-only subset for now), `bevy` + `bevy_egui`
  with the same feature set + version pin as cf-device-design.
  Sim-side deps (`sim-soft`, `mesh-sdf`, `mesh-offset`,
  `sim-ml-chassis`, `cf-design`) intentionally absent — Phase 3
  adds them.
- App-builder mirrors cf-device-design's `run_render_app` shape:
  `DefaultPlugins` + `EguiPlugin` + `OrbitCameraPlugin`; resources
  `UpAxis` / `RenderScale` / `ScanMesh` / `ScanFilePath` /
  `ScanInfo` / `Centerline` / `CavityState` / `LayersState` /
  `ScanMeshVisible`; Startup `setup_render_scene` (cf-viewer's
  `spawn_face_mesh` for the scan); Update `block_orbit_input_when_over_egui`
  + `draw_reference_overlays` + `apply_scan_mesh_visibility` +
  `exit_on_esc`; `EguiPrimaryContextPass` panel system.
- 12 tests in cf-sim-research (release, all green); 3 xtask
  `applies_to_crate_*` tests still green after the exemption arm +
  test-assert parallel-pin.
- Lint posture matches cf-device-design's (lenient — `unwrap_used` /
  `expect_used` / `panic` denied via `[lints.clippy]`, not workspace
  pedantic) per [[feedback-lint-posture-on-extract]].

**Posture banked**:
- `.design.toml` ingest deferred to Phase 3 — the loader lives
  behind cf-device-design's private `design_toml` module and would
  need a Phase-2.5 lift-to-shared-crate mini-arc to be reusable
  here. Phase 2 logs a notice if a sibling design TOML exists; the
  viewer always renders the `default_for_scan` cavity + layer stack.
- Cap planes parsed via `cf_cap_planes::parse_cap_planes` but
  discarded after the count is logged — there's no cavity/layer
  iso-extraction here to anchor them to in Phase 2. Phase 3
  inserts them as a Bevy resource alongside the cached SDF.
- `LAYER_SURFACE_PALETTE` (5 entries) duplicated by-value from
  cf-device-design's binary because the source binary doesn't
  export the constant; Phase 4 consolidates when the layer-mesh
  helpers lift to a shared crate.
- Layer "palette-tint" is a 14×14 px swatch in the panel row, NOT
  a 3D shell tint — Phase 3's mesh-sdf + mesh-offset wiring brings
  the on-mesh tinting in.
- Post-cold-read polish bundled into the same Phase-2 commit:
  doc-lie tightening on the lib-level docstring + CLI flag docs +
  workspace-member comment + cf-cap-planes dep comment; cavity +
  layers construction moved INSIDE `run_render_app` so Phase 3
  splices `apply_design_toml` there without rewriting the call
  site; non-empty-layer-stack regression test added.

**Sub-leg estimate confirmed**: 1 session (mostly mechanical;
clean lift across the binary boundary via cf-device-types).

### Phase 2.5 — lift shared modules ✓ SHIPPED 2026-05-19

**Status**: SHIPPED on branch `refactor/sim-decouple-phase-2.5`
(off `refactor/sim-decouple-phase-3-recon` `4b0605b9`), 4 commits
in execution order a → b → d → c per the recon's §2.5.c swap-
order authorization (clip_plane is self-contained; lifting it
first means 2.5.c can include `spawn_cavity_mesh` in a single
commit). Single session.

**Outcome**:
- `18300bf6` (2.5.a) — Lift `design_toml` (666 LOC + 15 tests) +
  `LAYER_SURFACE_PALETTE` const to cf-device-types. Drops the
  cf-sim-research by-value duplicate atomically. `serde` + `toml`
  + `anyhow` workspace deps added to cf-device-types. 24
  cf-device-types tests / 149 cf-device-design tests / 10
  cf-sim-research tests (2 duplicates retired) — all green.
- `b72c179b` (2.5.b) — New empty `design/cf-device-geometry/`
  crate with stub `lib.rs` + xtask grader exemption + matching
  test assertion. Lenient lint posture per
  [[feedback-lint-posture-on-extract]]. Bevy minimal feature set
  (`bevy_asset` + `bevy_pbr` + `bevy_render` — 2.5.d's
  ExtendedMaterial machinery).
- `fe9b80e3` (2.5.d) — Lift `clip_plane.rs` (729 LOC + 15 tests)
  + `clip_plane.wgsl` shader. Module docstring + WGSL header
  comment rephrase the prepass posture against cf-viewer's camera
  setup (consumed by both binaries) rather than the host binary
  by name. Shader-path const switched to
  `embedded://cf_device_geometry/...`. Q5.2 (`clip-plane` feature
  flag) NOT taken per [[feedback-strip-the-knob-when-default-works]]
  — add when cf-cast-cli (R7) actually consumes the crate.
- `d897e6c6` (2.5.c) — Lift `sdf_layers.rs` (2091 LOC + 35
  tests) + the 4 cross-binary cavity-render surfaces
  (`spawn_cavity_mesh`, `build_bevy_mesh_from_indexed*`,
  `CavityEntity`, `CAVITY_COLOR`). 17 `pub(crate)` → `pub`
  widenings. New cf-device-geometry deps: `mesh-sdf` +
  `mesh-offset` + `cf-cap-planes` + `cf-design` + `mesh-repair` +
  `meshopt` + `nalgebra` + `anyhow`. Final module organization:
  `sdf_layers` (compute) + `bevy_mesh` (adapters) + `cavity`
  (spawner + marker + color) + `clip_plane` (already shipped).
  50 cf-device-geometry tests (15 + 35) / 99 cf-device-design
  tests / 10 cf-sim-research tests — all green.

**Posture banked**:
- Sub-leaf execution order swap is OK when explicitly authorized
  by the recon — keeps commit count down (no separate 2.5.e for
  the deferred `spawn_cavity_mesh`).
- `pub(crate)` → `pub` bulk swap on lifted modules works cleanly
  when the consumer is the only crate accessing the surface;
  oversharing across the crate boundary doesn't cost anything
  at extraction time and tightening is easy if a surface
  surprise surfaces in Phase 3.
- Lift-with-deletion is atomic: the recon's R2
  ("`LAYER_SURFACE_PALETTE` consumer drift") was mitigated by
  bundling the cf-sim-research duplicate's deletion into 2.5.a's
  same commit as the cf-device-types const addition.

**Sub-leg estimate confirmed**: 1 session (1-2 was the recon's
range; landed at the low end via the swap-order optimization).

### Phase 3 — move sim code

**Goal**: relocate insertion_sim.rs (5835 LOC) +
insertion_sim_ui.rs (2331 LOC) + the 5 sim-coupled Bevy systems
from cf-device-design/main.rs into cf-sim-research.

**Steps**:
1. Copy insertion_sim.rs + insertion_sim_ui.rs to cf-sim-research
2. Copy update_layer_meshes, update_cavity_mesh, update_intruder_mesh
   (the FEM-coupled versions) to cf-sim-research's main.rs
3. Copy LayerMeshKey + cache logic
4. Wire InsertionSimPlugin in cf-sim-research's App builder
5. Replace cf-sim-research's stub UI with the real sim panel
6. Verify cargo build -p cf-sim-research green
7. Verify cargo test -p cf-sim-research green
8. cf-device-design still has duplicate copies of these — Phase 4
   removes them

**Estimate**: 2-3 sessions. Big copy job + adapter wiring. The
sim-soft dep moves to cf-sim-research at this phase.

### Phase 4 — refactor cf-device-design rendering, remove sim

**Goal**: strip the FEM-coupled rendering paths from
cf-device-design. Layers render with palette tint only; cavity
renders rest-frame SDF iso only; intruder renders at static pose
or is removed.

**Steps**:
1. Remove `insertion_sim*.rs` files from cf-device-design
2. Strip InsertionSimState references from update_layer_meshes —
   layers always palette-tint, never heat-map, never deformed
3. Strip ScalarMode + heat_map_on + show_deformed + displayed_step
   + last_run_generation from LayerMeshKey
4. Strip InsertionSimState references from update_cavity_mesh —
   cavity always rest-frame SDF iso
5. Strip InsertionSimState references from update_intruder_mesh —
   intruder at static cap-mouth pose, or remove if no longer needed
   (workshop iter-1 uses cf-cast-cli for mold; intruder visualization
   was sim-specific)
6. Remove `render_insertion_sim_section` call from
   `device_design_panel`
7. Remove `InsertionSimPlugin` registration from `run_render_app`
8. Remove sim-soft from cf-device-design Cargo.toml
9. Verify cargo build + test green
10. Verify cf-device-design UI works as expected (manual smoke
    test: scan loads, layers display, mold output works)

**Estimate**: 1-2 sessions. Lots of small deletions + system
signature changes.

### Phase 5 — verify + document

**Goal**: confirm both tools work end-to-end + update
documentation.

**Steps**:
1. cf-device-design smoke test: load scan, tweak cavity + layers,
   save design TOML, generate mold via cf-cast-cli
2. cf-sim-research smoke test: load scan + design TOML, render
   layers, run sim panel
3. Update gameplan §3 A1 → mark complete with refactor outcomes
4. Update auto-memory → add cf-sim-research as the new research
   crate cluster member
5. Update `docs/studies/soft_body_architecture/` SUMMARY index if
   it referenced cf-device-design as the sim host
6. Commit messages capture the architecture decision (the WHY)

**Estimate**: 1 session.

---

## 5. Total estimate + sequencing

**Sessions**: 6-9 total (Phase 1: 1-2, Phase 2: 1, Phase 2.5: 1-2,
Phase 3: 2, Phase 4: 1-2, Phase 5: 1). **3 banked** (Phase 1 +
Phase 2 + Phase 2.5 each shipped in 1 session, the low end of
their range); **3-4 remaining** for Phases 3-5.

**Calendar**: 1-2 weeks if focused single-task; 2-4 weeks if
interleaved with workshop-iter and other product work.

**Per [[feedback-bookmark-when-surface-levers-exhaust]]
three-session pattern**: this arc breaks the pattern (5+ sessions),
which is fine — refactors of this size aren't pattern-matched to
the three-session shape. Each phase ends with a green build so the
arc is incrementally shippable; if interrupted mid-arc, the
already-shipped phases stand.

---

## 6. What stays at parity

**Phase 1 + Phase 2 (shipped)**: cf-device-design's user-facing
surface is unchanged. cf-device-types is a pure type-share lift;
cf-sim-research is additive (a new binary that doesn't yet touch
the sim).

**Phases 3-4** (cf-device-design + cf-sim-research coexist with
duplicate sim code from Phase 3's copy until Phase 4 strips
cf-device-design's copy):
- All existing tests pass
- All workshop-iter functionality preserved
- The duplicate sim code is intentional; clean up in Phase 4

**Post-arc**:
- cf-device-design's user-facing surface: scan + layers + mold
  output, no sim panel
- cf-sim-research's surface: read-only design + sim panel + heat-map
  + deformed-cavity + slide-intruder
- cf-device-design build: no sim-soft dep
- cf-sim-research build: sim-soft dep + all the FEM machinery
- Workshop loop unchanged: scan → cf-device-design (layer design) →
  save TOML → cf-cast-cli (mold geometry) → workshop print
- Sim research loop: load design TOML + scan in cf-sim-research →
  run sim → visualize / export results

---

## 7. Risks + mitigations

**R1 — Phase 1 type extraction surprises.** Types in main.rs
reference internal helpers (e.g., default_for_scan methods that
read material data). Some helpers will need to move too. Mitigation:
do the type extraction breadth-first (move type, follow compile
errors, move what compile demands).

**R2 — Bevy resource/system wiring breaks.** Moving systems
between binaries means restructuring App builders. Mitigation:
Phase 2's stub binary establishes the App pattern before sim
moves; Phase 3 just slots systems into the existing App.

**R3 — Shared rendering helpers underestimated.** cf-viewer has
some helpers; we'll likely need to expand it OR duplicate. Phase
3's "copy first" approach defers this decision to Phase 4
cleanup. Mitigation: explicit Phase 4 step for rendering-helper
DRY.

**R4 — `cf-viewer` itself becomes a target of refactor.** If we
need to lift more helpers into it, that's its own micro-arc.
Mitigation: scope-creep guard — defer cf-viewer refactor to a
separate post-arc bookmark if it surfaces.

**R5 — Mid-arc workshop iter-1 cast happens.** Workshop loop must
keep working. Mitigation: Phase 1 + 2 don't touch cf-device-design's
working surface; Phase 4 is where the breaking change happens.
If iter-1 cast is mid-Phase-3, defer Phase 4 until after.

---

## 8. Sequencing relative to other work — HISTORICAL

The original plan suggested B1 → Phase 1+2 → B2 — keep the workshop
loop unblocked, then do the refactor, then write the roadmap.
Shipped order matched: B1 (`948bfb8d`) → Phase 1 (`a8567f24` +
`7cef74fe`) → B2 (`10d10db9`) → Phase 2 (`d4695892` + `7ea5a995`).
Phase 3 is the next inflection point; no remaining cross-arc
ordering constraints.

[Original suggested order preserved below for audit trail.]

- **B1 (FDM-mold-optimization in cf-cast-cli)** is INDEPENDENT of
  this refactor; can ship before, during, or after. Probably
  better to ship B1 first since it's a concrete user-facing win
  and doesn't touch cf-device-design or sim code.
- **B2 (sim-research roadmap doc)** is dependent on this
  refactor's outcome — the doc lives in cf-sim-research's
  directory or `docs/sim-research/`. Write it after Phase 5
  completes.

Suggested order:
1. **Now**: B1 bookmark + recon + impl (cf-cast-cli mold-wall
   optimization). Concrete user-facing win.
2. **Then**: this refactor (Phase 1 → 5).
3. **Then**: B2 sim-research roadmap doc.

This sequencing keeps the workshop loop unblocked (B1 ships
product value) while the refactor and roadmap are longer-arc
infrastructure work.

---

## 9. Pointers

- `docs/PROGRAM_GAMEPLAN.md` — parent gameplan; this doc realizes
  §3 A1's intent at honest scope
- `docs/SIM_ARCHITECTURE_AUDIT.md` — why decouple matters
- `tools/cf-device-design/src/main.rs` — current host of sim
  wiring (`update_layer_meshes` @ 950, `update_cavity_mesh` @ 849,
  `update_intruder_mesh` @ 1275, `device_design_panel` @ 1740,
  `run_render_app` @ 2032 + `insertion_sim.rs` 5762 LOC +
  `insertion_sim_ui.rs` 2275 LOC). Numbers as of 2026-05-19; find
  by name, not line.
- `cf-viewer` — already a workspace lib (not "if exists"); Phase 2
  consumed `spawn_face_mesh` from it directly. Candidate home for
  any further shared rendering helpers that surface mid-Phase-3.
- `design/cf-device-types/` — Phase 1 output (SHIPPED 2026-05-19,
  branch `refactor/sim-decouple-phase-1` on origin).
- `tools/cf-sim-research/` — Phase 2 output (SHIPPED 2026-05-19,
  branch `refactor/sim-decouple-phase-2`).

---

End of refactor plan. **Phase 1 + Phase 2 + Phase 2.5 SHIPPED
2026-05-19** on branches `refactor/sim-decouple-phase-1` +
`refactor/sim-decouple-phase-2` + `refactor/sim-decouple-phase-2.5`
respectively; next session picks up at Phase 3, which per §4 step
list copies `insertion_sim.rs` + `insertion_sim_ui.rs` + the 4
sim-coupled Bevy systems (`update_layer_meshes` /
`update_cavity_mesh` / `update_intruder_mesh` / `spawn_intruder_mesh`)
into cf-sim-research. All cross-crate decisions landed at Phase
2.5; Phase 3 is pure copy + wire.

Phase 4 then strips cf-device-design's copies; Phase 5 verifies +
documents.
