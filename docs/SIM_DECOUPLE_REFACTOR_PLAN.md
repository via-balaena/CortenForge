# Sim-decouple refactor — full plan

> **Status**: BOOKMARK, set 2026-05-19 LATE-NIGHT after A1's
> in-session scoping revealed the sim is woven deeply into
> cf-device-design's layer/cavity/intruder rendering. The original
> gameplan §3 A1 "~1 session feature-flag" estimate was wrong.
> This doc sequences the real arc.
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

- `update_layer_meshes` (main.rs:1630) — reads `InsertionSimState`
  for `heat_map_on`, `scalar_mode`, `last_run_generation`,
  `displayed_step`, `show_deformed`, and `last_run` (the FEM
  results). Applies heat-map projection per-MC-vertex; falls
  through to deformed-shells path when `show_deformed` is on; falls
  through to rest-frame SDF iso when both are off.
- `update_cavity_mesh` (main.rs:1529) — reads same state for the
  deformed-cavity path.
- `update_intruder_mesh` (main.rs:1955) — reads same state for the
  slide-pose-at-displayed-step rendering.
- `device_design_panel` (main.rs:2437) — calls
  `render_insertion_sim_section` inside the panel render system.
- `LayerMeshKey` struct (main.rs:1339) — carries
  `scalar_mode: insertion_sim_ui::ScalarMode` as a diff field for
  layer-mesh cache invalidation.
- `run_render_app` (main.rs:2729) — registers
  `insertion_sim_ui::InsertionSimPlugin` which inserts the
  `InsertionSimState` resource and runs the FEM-poll task.

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

## 3. Open architecture questions (resolve at recon stage)

**Q1 — does cf-sim-research have a UI, or is it headless?**

- Option A: full Bevy + egui UI mirroring cf-device-design's
  layout, with sim panel added. Heavy weight; lots of duplicated
  rendering code.
- Option B: headless CLI tool — `cf-sim-research --design design.toml
  --scan scan.stl` runs sim, writes outputs to files (heat-map
  STLs, force traces, etc.). Lightweight; sim researchers can
  visualize via existing tools.
- Option C: minimal Bevy UI with a sim-results-only viewport (load
  prep.toml + design.toml + scan, render sim outputs read-only).
  Lightweight, still visual.

Default lean: **Option C** (minimal Bevy viewport). UI is for
visualizing sim results, not for layer tweaking. Layer tweaking
happens in cf-device-design; sim runs against the locked design.

**Q2 — should cf-sim-research enable layer-tweaking-with-live-sim?**

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

**Q3 — shared rendering helpers — extract to lib crate or duplicate?**

The current `build_bevy_mesh_from_indexed`, `build_bevy_mesh_from_indexed_with_colors`,
`spawn_face_mesh`, palette code, etc. would be needed by both tools.

- Option A: extract to a new `cf-layer-render` crate or expand
  `cf-viewer`. Both binaries depend on it. DRY.
- Option B: duplicate the helpers between the two binaries.
  Simpler at extraction time, technical debt long-term.

Default lean: **Option A**. cf-viewer already has scene helpers
(per its existing role); expanding it is natural.

**Q4 — feature-flag the sim within cf-sim-research, or always-on?**

Once extracted, cf-sim-research could itself have a feature flag
for the sim, or always include it (since sim is its raison d'être).

- Option A: always-on. cf-sim-research IS the sim tool;
  feature-flagging within it is redundant.
- Option B: feature-flag for cargo-doc-no-sim convenience.

Default lean: **Option A**.

---

## 4. Migration sequence

Five phases. Each phase ends with a green workspace build.

### Phase 1 — extract shared types to `cf-device-types`

**Goal**: lift CavityState, LayersState, ScalarMode (+ supporting
types) out of cf-device-design's main.rs and insertion_sim*.rs into
a new lib crate that both future binaries depend on.

**Steps**:
1. Create `lib/cf-device-types/` (or under `cf/` per workspace
   convention)
2. Move types: CavityState, LayersState, LayerState, MaterialField
   (the device-side, not the sim-side), SlackerSelection,
   ScalarMode, SimDesign, Centerline, ScanInfo, ScanMesh,
   ScanFilePath, plus supporting structs
3. Update cf-device-design imports + Cargo.toml dep
4. Verify cargo build green
5. Verify cargo test green

**Estimate**: 1-2 sessions. Mostly mechanical; risk = type-coupling
surprises (some types reference internal cf-device-design helpers
that also need moving).

### Phase 2 — create `tools/cf-sim-research/` skeleton

**Goal**: stand up the new binary with a no-op-sim placeholder UI.
Doesn't actually move sim code yet; just establishes the binary
exists and compiles.

**Steps**:
1. Create `tools/cf-sim-research/` with Cargo.toml + main.rs
   stub
2. Workspace member entry in root Cargo.toml
3. Depend on cf-device-types (Phase 1 output)
4. Bevy + egui dep
5. Minimal "load scan + display layers in palette tint" UI
   (no sim yet — just to verify the binary stands up)
6. Verify cargo build green

**Estimate**: 1 session.

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

**Sessions**: 6-9 (Phase 1: 1-2, Phase 2: 1, Phase 3: 2-3, Phase 4:
1-2, Phase 5: 1).

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

**During the arc** (cf-device-design + cf-sim-research coexist
with duplicate sim code in Phases 3-4):
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

## 8. Sequencing relative to other work

Per `docs/PROGRAM_GAMEPLAN.md`:

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
  wiring (1630, 1529, 1955, 2437, 2729 + 5835 + 2331 LOC of
  sim modules)
- `tools/cf-viewer/` (if exists) — candidate home for shared
  rendering helpers
- Future `lib/cf-device-types/` — Phase 1 output
- Future `tools/cf-sim-research/` — Phase 2+ output

---

End of refactor plan. Next session: Phase 1 (extract shared
types). Or, if B1 (cf-cast-cli mold-wall) is the priority, do
that first; this refactor waits.
