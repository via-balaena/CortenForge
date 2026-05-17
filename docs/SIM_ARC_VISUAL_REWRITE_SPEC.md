# cf-device-design Sim Arc — Visual Rewrite Spec (S11)

**Status**: RECON SHIPPED 2026-05-18 LATE. This spec replaces the bookmark sketch at `docs/SIM_ARC_VISUAL_REWRITE_BOOKMARK.md` §3 with concrete file locations, function signatures, and a sub-leaf ladder. Next session = implementation per the three-session pattern ([[feedback-bookmark-when-surface-levers-exhaust]]).

**Reading order**: §1 problem recap (one paragraph) → §2 resolved decisions (D-Vis1..D-Vis6) → §3 architectural shape (data flow + APIs) → §4 sub-leaf ladder (S11.1/.2/.3, each with concrete diffs + visual gate + tests) → §5 pre-implementation gates → §6 cross-references.

---

## 1. Problem recap (one paragraph)

S1-S4 (`docs/SIM_ARC_RECON.md` §6) shipped the visual story but the user-verified visual gate on iter-1 (2026-05-18) surfaced three architectural issues that need a rewrite, not patches (full detail in bookmark §2): (F1) S4 intruder is the full scan rigid-translated, but the FEM intruder is an SDF-offset that grows from cavity-sized at step 1 to full-scan at step 16 → user reads a phantom "air gap"; (F2) layer shells render as outer-face thin shells, not slabs → user can't see the silicone volume between layers; (F3) opaque outer shells occlude inner geometry → user has to manually toggle visibilities. This spec resolves all three by sampling the cached scan SDF for an FEM-accurate per-step intruder, adding inner-face triangles to each layer entity for slab volumes, applying translucent alpha, and hiding the redundant cavity entity in `show_deformed` mode.

---

## 2. Resolved decisions

Resolved in this recon session per [[feedback-autonomous-architecture]] (defaults from bookmark §4 stand unless noted).

| Decision | Resolution | Rationale |
|---|---|---|
| **D-Vis1** intruder color | **Teal `(0.20, 0.75, 0.85)`** | Cool tone contrasts the warm coral cavity (`0.95, 0.55, 0.45`) so the intruder stays distinguishable from the cavity surface in contact zones. Magenta candidate not preferred — sits visually too close to the lavender layer-palette entry. |
| **D-Vis2** layer alpha | **`0.35`** | Default from bookmark; reads as a noticeable translucent band without losing the inner layers behind it. Implementation lands a single constant `LAYER_SLAB_ALPHA` that the visual gate can A/B 0.25/0.35/0.45 trivially if needed. |
| **D-Vis3** in-place vs new sub-leaf | **In-place rewrite** | Per [[feedback-strip-the-knob-when-default-works]]: the S4 translation visual proxy did not earn its keep on the visual gate. Replacing the path is cleaner than gating on a knob. The S4 commit message `9a55cc2a` already documented it as a visual proxy explicitly. |
| **D-Vis4** cavity entity in show_deformed | **Hide** | L0's slab inner face = the same cavity surface triangles (`cavity_boundary_faces`), so rendering both would double-draw the same geometry. Hiding the cavity entity when `show_deformed && last_run.is_some()` keeps the rest-cavity view available by toggling show_deformed off. |
| **D-Vis5** cap-rim closure tier-2 | **Defer to follow-up** | Adds geometry complexity (annular triangulation per layer) that the iter-1 visual gate has not flagged as needed. Easy to fold into a later commit if iter-2 surfaces the need. |
| **D-Vis6** sub-leaf ladder | **3 sub-leaves S11.1 / S11.2 / S11.3** | Each individually revertable per [[feedback-implement-measure-revert-pattern]]. Per-sub-leaf visual gate is short + scoped (single feature per commit). |

**SDF source for per-step intruder MC** (resolved in recon, ahead of implementation): **`main.rs::cached_scan_sdf`** via `sdf_layers::extract_layer_surface(cached_sdf, &caps, iso)`, NOT the FEM-internal `InsertionGeometry::intruder` GridSdf. Same SDF + same MC pipeline as the cavity + layer renders → visually consistent; no API change to `InsertionGeometry`. Trade-off: ~sub-mm geometric difference from what the FEM actually solved against (FEM uses `decimate_for_sdf` sloppy at `0.75 * cell_size_m = 3 mm` grid; cached uses `decimate_scan_for_sdf` topology-preserving at `LAYER_PREVIEW_CELL_SIZE_M = 5 mm` grid). Difference is swamped by the BCC analysis-mesh chunkiness (4 mm tet faces) that drives the deformed cavity render — the visual story remains FEM-truthful.

---

## 3. Architectural shape

### 3a. Data-flow diagram (post-rewrite)

```
kick_off_simulation        ┌── cached_scan_sdf (Res, cloned at kickoff) ──┐
   spawns async task ──────┤                                              │
                           ├── design (from CavityState + LayersState) ───┤
                           └── cap_planes (Res, cloned at kickoff) ───────┤
                                                                          ▼
                                              run_sim_pipeline(scan, design, cap_planes,
                                                               cached_sdf, n_steps)
                                                                          │
                              ┌── build_insertion_geometry + run_ramp ────┤
                              │                                            │
                              └── per_step_intruder_meshes: NEW ───────────┤
                                  (16 IndexedMeshes from cached_sdf MC)   │
                                                                          ▼
                                              InsertionSimOutputs {
                                                  ... existing ...,
                                                  per_step_intruder_meshes,   // S11.1
                                              }
                                                                          │
                ┌─────────────────────────────────────────────────────────┤
                │                                                          │
                ▼                                                          ▼
       update_intruder_mesh (NEW, S11.1)            update_layer_meshes (S11.2 modified)
       (renamed from update_intruder_transform)     - uses deformed_layer_slab_mesh_at
       - snapshot-and-compare on                     (NEW helper, inner + outer faces)
         (displayed_step, generation, show_deformed) - translucent material
       - rebuilds Mesh3d on change                   (alpha = LAYER_SLAB_ALPHA = 0.35)
       - per-step iso = step.interference_m
                       + cavity_offset_m
       - uses cached_scan_sdf via
         extract_layer_surface
                                                    update_cavity_mesh (S11.3 modified)
                                                    - when show_deformed && last_run:
                                                      force Visibility::Hidden
                                                      (skip mesh rebuild — wasted work)
```

### 3b. New / modified APIs (concrete signatures)

#### `tools/cf-device-design/src/insertion_sim_ui.rs`

```rust
pub struct InsertionSimOutputs {
    // ... existing fields ...
    /// S11.1 — pre-computed per-step intruder MC meshes for the
    /// FEM-accurate intruder render. Outer index is converged-step
    /// index (matches `ramp.steps`); each `IndexedMesh` is the
    /// cached scan SDF iso surface at `step.interference_m +
    /// cavity_offset_m`. Built once in `run_sim_pipeline` from the
    /// CachedScanSdf cloned into the async task at kickoff. Memory:
    /// 16 × ~5 K vertices × 24 B + ~10 K faces × 12 B ≈ 4 MB
    /// per run (dwarfed by `per_step_scalar_fields`).
    pub per_step_intruder_meshes: Vec<IndexedMesh>,
}

impl InsertionSimOutputs {
    /// S11.1 — borrow the pre-computed intruder mesh for the
    /// requested step, or `None` if out of converged range. Returns
    /// a reference (no clone — the caller `update_intruder_mesh`
    /// passes the borrow through `build_bevy_mesh_from_indexed` which
    /// only reads positions + faces to upload to a Bevy Mesh asset).
    #[must_use]
    pub fn intruder_mesh_at(&self, step: usize) -> Option<&IndexedMesh> {
        self.per_step_intruder_meshes.get(step)
    }

    /// S11.2 — combine layer `layer_idx`'s INNER + OUTER triangles
    /// with the step's deformed positions. Inner triangles =
    /// `cavity_boundary_faces` for `layer_idx == 0`, else
    /// `per_layer_outer_faces[layer_idx - 1]` (the prior layer's
    /// outer face IS this layer's inner face — same BCC interface
    /// triangles, shared vertex layout). Outer triangles =
    /// `per_layer_outer_faces[layer_idx]`. Returns `None` for
    /// out-of-range layer or step.
    ///
    /// Inner and outer triangles reference disjoint BCC vertex
    /// subsets (cavity-side vs outer-side of the layer band), so
    /// smooth-normal computation accumulates correctly per surface
    /// without interference between the two skins.
    #[must_use]
    pub fn deformed_layer_slab_mesh_at(
        &self,
        layer_idx: usize,
        step: usize,
    ) -> Option<IndexedMesh>;
}
```

#### `tools/cf-device-design/src/insertion_sim_ui.rs::run_sim_pipeline` — new parameter

```rust
fn run_sim_pipeline(
    scan: IndexedMesh,
    design: SimDesign,
    cap_planes: Vec<CapPlane>,
    cached_sdf: CachedScanSdf,             // S11.1 NEW — cloned at kickoff
    n_steps: usize,
) -> Result<InsertionSimOutputs>;
```

Implementation: after `run_insertion_ramp` returns, iterate `ramp.steps`, compute per-step iso = `step.interference_m + cavity_offset_m` (where `cavity_offset_m = -design.cavity_inset_m`), call `sdf_layers::extract_layer_surface(&cached_sdf, &cap_planes, iso)` for each, collect into `Vec<IndexedMesh>`. Cost: ~sub-ms per step × 16 ≤ 20 ms total inside the async task (FEM ramp dominates at 30-60 s).

#### `tools/cf-device-design/src/insertion_sim_ui.rs::kick_off_simulation` — new parameter

```rust
pub fn kick_off_simulation(
    scan: Option<Res<ScanMesh>>,
    cavity: Res<CavityState>,
    layers: Res<LayersState>,
    cap_planes: Res<CapPlanes>,
    cached_sdf: Res<CachedScanSdf>,        // S11.1 NEW
    mut state: ResMut<InsertionSimState>,
);
```

Clone `cached_sdf.clone()` into the task spawn just like `cap_planes_clone`. CachedScanSdf is `Resource, Clone` (sdf_layers.rs:181); the clone is ~700 KB at iter-1 (two ScalarGrids at ~44 k cells × 8 B + Arc-wrapped trait objects that are clone-by-ref).

#### `tools/cf-device-design/src/main.rs::update_intruder_transform` — renamed + reshaped

```rust
// RENAMED from `update_intruder_transform`.
// Replaces transform-translation math with mesh-asset rebuild.
fn update_intruder_mesh(
    sim_state: Res<insertion_sim_ui::InsertionSimState>,
    up: Res<UpAxis>,
    render_scale: Res<RenderScale>,
    mut last_key: Local<Option<IntruderMeshKey>>,  // NEW snapshot
    mut meshes: ResMut<Assets<Mesh>>,
    mut q: Query<(&mut Mesh3d, &mut Visibility), With<IntruderEntity>>,
);

#[derive(Debug, Clone, PartialEq)]
struct IntruderMeshKey {
    displayed_step: usize,
    last_run_generation: u64,
    show_deformed: bool,
}
```

Same snapshot-and-compare posture as `update_cavity_mesh` (main.rs:1455). On change: pull `sim_state.last_run.intruder_mesh_at(displayed_step)`, build Bevy mesh via `build_bevy_mesh_from_indexed`, swap the mesh handle, set visibility. On no run / no `show_deformed`: hide. **No transform mutation** — the intruder mesh sits at world origin (the scan's natural frame); the per-step geometry change IS the animation.

`spawn_intruder_mesh` (main.rs:1712) simplifies: spawn with an empty placeholder mesh handle + Visibility::Hidden + IntruderEntity marker; `update_intruder_mesh` fills the mesh on the first frame `last_run` becomes `Some`. Drop the `triangle_mesh_flat_shaded(&scan.0, None, *up)` initial mesh build — the rest scan is owned by `ScanMeshEntity`, the IntruderEntity is sim-only.

**Color change** (S11.3): `INTRUDER_COLOR` constant changes from `(0.95, 0.55, 0.20)` warm orange to `(0.20, 0.75, 0.85)` teal.

#### `tools/cf-device-design/src/main.rs::update_layer_meshes` — slab rendering

```rust
// (function signature unchanged)
// CHANGE: when `show_deformed && last_run.is_some()`, use the
//         slab mesh helper instead of the outer-only helper.
let layer_indexed = deformed_layers_run
    .and_then(|run| run.deformed_layer_slab_mesh_at(i, sim_state.displayed_step))  // S11.2
    .unwrap_or_else(|| {
        sdf_layers::extract_layer_surface(&cached_sdf, &cap_planes.planes, safe_offset_m)
    });
```

**Material change** (S11.2): the per-layer `StandardMaterial` gets `alpha_mode: AlphaMode::Blend` + `base_color` carries alpha. Two cases:

- Heat-map ON: `base_color = Color::srgba(1.0, 1.0, 1.0, LAYER_SLAB_ALPHA)`. Vertex-color attribute multiplies through unchanged.
- Heat-map OFF: `base_color = Color::srgba(r, g, b, LAYER_SLAB_ALPHA)` for the palette tint.

Constant: `const LAYER_SLAB_ALPHA: f32 = 0.35;` next to `LAYER_SURFACE_PALETTE`.

Heat-map note: `project_layer_heat_map` already operates on `layer_indexed.vertices`. The slab's combined vertex array IS the full BCC vertex layout (same as the outer-only path); heat-map projection iterates `mc_vertices` (i.e. the full `.vertices`), so the nearest-tet sampling per inner-face vertex Just Works. Inner-face vertices on the cavity surface get colored by the in-layer tet nearest to them — for L0 those are the layer-0 cavity-side tets, which is correct (the pressure on the cavity wall IS the layer-0 contact pressure).

#### `tools/cf-device-design/src/main.rs::update_cavity_mesh` — hide on show_deformed

```rust
// EARLY-EXIT addition near the top of update_cavity_mesh:
//   when show_deformed && last_run.is_some(), force the cavity
//   entity hidden (L0 slab inner face owns the cavity surface)
//   AND skip the mesh-rebuild work below. The CavityMeshKey
//   already captures show_deformed, so the snapshot-and-compare
//   path will re-fire when the toggle flips back.
if sim_state.show_deformed && sim_state.last_run.is_some() {
    for (_mesh_handle, mut visibility) in &mut q {
        *visibility = Visibility::Hidden;
    }
    return;
}
```

Place this **after** the `current_key` snapshot + change-check so we don't re-hide every frame. Actually — the simpler shape is to fold "should hide" into the existing logic at the bottom:

```rust
let force_hidden = sim_state.show_deformed && sim_state.last_run.is_some();
// ... existing mesh rebuild ...
for (mut mesh_handle, mut visibility) in &mut q {
    mesh_handle.0 = mesh_asset.clone();
    *visibility = if state.visible && !force_hidden {
        Visibility::Visible
    } else {
        Visibility::Hidden
    };
}
```

This skips the rebuild waste too — actually no, the rebuild work still runs to keep the mesh handle warm for when `show_deformed` flips off. Probably fine (rebuild is cheap, sub-millisecond per main.rs:736).

### 3c. Why no spawn-time changes

S11 keeps `setup_render_scene` + `spawn_cavity_mesh` + `spawn_intruder_mesh` shapes. `spawn_intruder_mesh` simplifies but stays a startup system. No new ECS systems beyond renaming `update_intruder_transform` → `update_intruder_mesh` (Plugin chain at main.rs:2588-2597 needs the rename).

---

## 4. Sub-leaf ladder

### S11.1 — FEM-accurate intruder (~2 hr)

**Touches**: `insertion_sim_ui.rs` (struct field + helper + `run_sim_pipeline` + `kick_off_simulation`), `main.rs` (`update_intruder_transform` → `update_intruder_mesh` rewrite, `spawn_intruder_mesh` simplification, Plugin chain rename).

**Diff sketch**:
1. Add `per_step_intruder_meshes: Vec<IndexedMesh>` to `InsertionSimOutputs`.
2. Add `intruder_mesh_at` accessor.
3. Add `cached_sdf: CachedScanSdf` parameter to `run_sim_pipeline`; after the ramp, compute `cavity_offset_m = -design.cavity_inset_m`, then iterate `ramp.steps` collecting `extract_layer_surface(&cached_sdf, &cap_planes, step.interference_m + cavity_offset_m)`.
4. Add `cached_sdf: Res<CachedScanSdf>` parameter to `kick_off_simulation`; clone into task at spawn.
5. Rename `update_intruder_transform` → `update_intruder_mesh`; replace transform math with snapshot-and-compare mesh-asset rebuild via `IntruderMeshKey`.
6. Simplify `spawn_intruder_mesh` to spawn with placeholder empty mesh.
7. Update Plugin chain rename at main.rs:2597.
8. Update doc comments on the moved/renamed code to reflect the FEM-literal semantics (replacing the "Visual proxy, not FEM-literal" note).

**Tests** (add to insertion_sim_ui.rs tests module):
- `per_step_intruder_meshes_grow_from_step0_to_stepN_minus_1` — assert that `outputs.intruder_mesh_at(0).vertices.len() < outputs.intruder_mesh_at(n-1).vertices.len()` (the iso moves from negative-inside toward 0, growing the iso surface envelope). Fixture: a synthetic InsertionSimOutputs with hand-built 2-step meshes is fine; an integration test through `run_sim_pipeline` would tie this to a real ramp and is too slow for the unit-test layer. Pick the synthetic path.
- `per_step_intruder_meshes_count_matches_ramp_steps` — assert `per_step_intruder_meshes.len() == ramp.steps.len()`.
- `intruder_mesh_at_returns_none_past_converged_range` — assert `outputs.intruder_mesh_at(n).is_none()` where `n == ramp.steps.len()`.

**Visual gate** (user-verified on iter-1):
- Boot cf-device-design + run Simulate.
- Scrub playback slider 1 → 16.
- At step 1: intruder is a small lump roughly inside the cavity volume (shrunken scan, ~2.81 mm offset inside).
- At step 16: intruder = full scan, coincides with the deformed cavity surface in contact zones (no visible "air gap" between intruder and cavity).
- Intermediate steps: intruder grows uniformly, cavity deforms outward to track it.
- Intruder is teal (post S11.3 — at S11.1 it's still orange; that's fine for the S11.1 gate).

**Cumulative test count**: 145 → 148 (+3).

---

### S11.2 — Layer slab rendering with translucency (~2-3 hr)

**Touches**: `insertion_sim_ui.rs` (`deformed_layer_slab_mesh_at` helper), `main.rs` (`update_layer_meshes` slab call + `LAYER_SLAB_ALPHA` constant + material alpha + AlphaMode change).

**Diff sketch**:
1. Add `deformed_layer_slab_mesh_at(layer_idx, step) -> Option<IndexedMesh>` to `InsertionSimOutputs`.
   - For `layer_idx == 0`: inner_faces = `cavity_boundary_faces`.
   - For `layer_idx > 0`: inner_faces = `per_layer_outer_faces[layer_idx - 1]`.
   - outer_faces = `per_layer_outer_faces[layer_idx]`.
   - Concatenate inner + outer; same shared vertex layout as the existing helpers; positions = `ramp.steps[step].x_final` chunked to `Point3<f64>`.
2. Add `const LAYER_SLAB_ALPHA: f32 = 0.35;` next to `LAYER_SURFACE_PALETTE` (main.rs:1361).
3. In `update_layer_meshes` (main.rs:1556), swap `deformed_layer_mesh_at` for `deformed_layer_slab_mesh_at`.
4. In the same loop, modify the per-layer material's `base_color` (the per-layer materials are `ExtendedMaterial<StandardMaterial, ClipPlaneExt>`, so the `base_color` + `alpha_mode` live under `.base.*`):
   - Heat-map ON: `base.base_color = Color::srgba(1.0, 1.0, 1.0, LAYER_SLAB_ALPHA)`.
   - Heat-map OFF: `base.base_color = Color::srgba(r, g, b, LAYER_SLAB_ALPHA)`.
   - Set `base.alpha_mode = AlphaMode::Blend` in both cases.
5. Doc update on `deformed_layer_mesh_at` — note it's the legacy outer-only helper kept for SDF-iso fallback; the slab helper is the deformed render's primary path.

**Tests** (add to insertion_sim_ui.rs tests):
- `deformed_layer_slab_mesh_at_layer_0_uses_cavity_boundary_for_inner` — synthetic outputs with `cavity_boundary_faces = vec![[0,1,2]]` and `per_layer_outer_faces = vec![vec![[3,4,5]]]`; assert slab mesh for layer 0 has 2 faces == cavity face + outer face.
- `deformed_layer_slab_mesh_at_layer_i_uses_prior_outer_for_inner` — synthetic 2-layer outputs; assert layer 1's slab faces = `per_layer_outer_faces[0]` ++ `per_layer_outer_faces[1]`.
- `deformed_layer_slab_mesh_at_returns_none_for_out_of_range_layer` — assert `layer_idx = 2` returns None on a 2-layer outputs (indices 0, 1 valid).
- `deformed_layer_slab_mesh_at_returns_none_for_out_of_range_step` — assert step past `ramp.steps.len()` returns None.

**Visual gate** (user-verified on iter-1):
- 2-layer iter-1 design: scrub to step 16.
- Each layer reads as a translucent thick band of silicone — inner surface + outer surface visible through alpha; the volume between is recognizably the silicone wall.
- Layer 0 (Ecoflex amber) inner face = the cavity surface; outer face = the L0/L1 interface.
- Layer 1 (Dragon Skin sky-blue) inner face = the L0/L1 interface (rendered twice: amber slab outer + blue slab inner; user sees a blended color at the interface — pedagogically clear, "this is the boundary between materials").
- Through the clip plane: cavity coincides with intruder at contact; L0 wall thickness visually readable; L1 wall thickness visually readable.
- Heat-map ON: per-vertex colors still drive the gradient under the translucent material — confirm the gradient reads clearly through alpha.

**Cumulative test count**: 148 → 152 (+4).

---

### S11.3 — Cavity hide + intruder recolor (~30 min)

**Touches**: `main.rs` (`update_cavity_mesh` force-hidden branch, `INTRUDER_COLOR` constant change).

**Diff sketch**:
1. In `update_cavity_mesh` (main.rs:1455), add a `force_hidden = sim_state.show_deformed && sim_state.last_run.is_some()` check; OR into the final visibility computation so the entity hides when the L0 slab inner face owns the cavity surface.
2. Change `INTRUDER_COLOR` constant from `(0.95, 0.55, 0.20)` → `(0.20, 0.75, 0.85)` (teal).
3. Update the doc comment on `INTRUDER_COLOR` to explain the contrast rationale ("cool teal distinguishes the intruder from the warm coral cavity in contact zones where the two surfaces coincide").

**Tests**: no new unit tests. Both changes are surface-only (a visibility branch + a constant tweak); the S11.1 + S11.2 regression tests cover the data layer.

**Visual gate** (user-verified on iter-1):
- show_deformed ON + last_run present: cavity entity is hidden (no separate coral mesh — the L0 slab amber-translucent inner face owns the cavity surface).
- show_deformed OFF (toggle the checkbox in the Insertion Sim panel): cavity reappears as the standalone coral entity at the rest position (pre-S11 behavior preserved).
- Intruder is teal — visibly distinct from any other entity in any color mode (rest scan grey, cavity coral, layer palette amber/blue/...).
- Heat-map mode still works: cavity surface (now part of L0 slab) carries the heat gradient.

**Cumulative test count**: 152 → 152 (+0).

---

### Bundling option for S11.3

Per [[feedback-cold-read-review-post-ship]]: after S11.1 + S11.2 ship + their visual gates pass, do a cumulative cold-read pass on both diffs; fold the S11.3 cavity-hide + intruder-color changes PLUS any findings from the cold-read into a single polish commit. Default S11.3 lands tiny enough that the cold-read polish can sit alongside it cleanly; the polish commit is the natural "ship the arc" boundary.

---

## 5. Pre-implementation gates (next session = implementation)

1. **Cold-read this spec end-to-end** before starting S11.1. Especially §3b (concrete signatures) and §4 (per-sub-leaf diff sketches). The naming `update_intruder_mesh` vs `update_intruder_transform` is load-bearing — the Plugin chain rename at main.rs:2597 + any test imports.

2. **Re-validate empirical baseline** if more than ~24 hr have passed since this recon (2026-05-18 LATE):
   ```
   cargo run -p cf-device-design --release -- \
       ~/scans/sock_over_capsule.cleaned.stl \
       --design ~/scans/sock_over_capsule.design.toml \
       --prep-toml ~/scans/sock_over_capsule.prep.toml
   ```
   Confirm 16/16 ramp + S1-S4 + 2 fixes still hold (per bookmark §6 step 2). If anything regressed, investigate before continuing.

3. **Pre-implementation test snapshot**: `cargo test -p cf-device-design --release` should report **145 passed**. `cargo clippy -p cf-device-design --release --tests -- -D warnings` should be clean. This recon session confirmed both 2026-05-18 LATE.

4. **Implementation cadence per [[feedback-implement-measure-revert-pattern]]**: implement S11.1 → run tests + clippy + visual gate → if green, commit; if red, revert and recon. Same for S11.2 + S11.3.

5. **Post-arc cumulative cold-read** per [[feedback-cold-read-review-post-ship]]: fresh-eyes pass on the three commits as a unit. Findings + S11.3 bundle into the polish commit (or a separate cold-read polish commit if S11.3 already shipped).

6. **Memo + MEMORY.md update** at end of arc: this spec becomes SHIPPED; the bookmark stays as the visual-gate audit trail; the sim-arc memo gets a "S11 SHIPPED" row.

---

## 6. Sub-leaf grade gate (each sub-leaf)

Per [[feedback-cf-cast-tests-use-release]] + [[feedback-long-running-commands-use-file-redirect]]:

```
cargo test -p cf-device-design --release 2>&1 | tail -5
cargo clippy -p cf-device-design --release --tests -- -D warnings 2>&1 | tail -5
cargo run -p xtask -- grade cf-device-design > /tmp/s11_grade.log 2>&1 &
# (wait for completion notification; then `tail /tmp/s11_grade.log`)
```

All three must be green per sub-leaf before commit. The grade run is ~few minutes; redirect to file per the durable feedback.

---

## 7. Open risks (surface for user input if hit during implementation)

| Risk | Trigger | Mitigation |
|---|---|---|
| Per-step MC cost much higher than estimated | If iter-1 cached_sdf MC at iso ≠ 0 is slower than the ~sub-ms estimate (e.g., the with-caps composition path adds significantly to the 16-call total) | Switch to lazy-cache-on-scrub: keep `per_step_intruder_meshes: Vec<Option<IndexedMesh>>` and lazily fill in `update_intruder_mesh` on the scrub frame. Spec the change in-session if it hits. |
| Smooth normals on slab mesh wrong at shared vertices | If the BCC inner face triangles and outer face triangles SHARE any vertex (they shouldn't — inner is cavity-side, outer is outer-side, disjoint subsets) | Verify with a `slab_inner_outer_vertices_disjoint` test on iter-1 outputs. If they share, split inner + outer into two Bevy mesh assets per layer (two entities per layer) instead of one combined mesh. |
| Translucent material draws not z-sorted properly | Bevy's standard alpha blending depth-sorts at the entity level, not the triangle level — for the slab case (inner + outer faces on the same entity), the triangles inside one entity can render in submission order, producing flicker/order artifacts at certain camera angles | Hit-test on iter-1 visual gate. If artifacts surface: split inner + outer faces into TWO entities per layer (each opaque-OR-translucent independently), giving Bevy per-entity depth-sort granularity. OR fall back to opaque outer + translucent inner (loses the "see the slab volume" affordance but keeps z-order clean). |
| Heat-map gradient washed out under translucent material | Vertex color × base color × light × alpha may make the gradient illegible | A/B 0.45 vs 0.35 alpha for the heat-map mode specifically; OR force heat-map mode to opaque (alpha = 1.0), losing slab-volume readability when the user wants pressure scoring. Hit-test before committing S11.2. |
| Cap-rim "open tube" reads as broken | iter-1 viewing from below the cap plane shows the slab annulus open | Tier-2 cap-rim closures (D-Vis5 deferred) can be added in a follow-up sub-leaf S11.4 if visual gate flags it. |

---

## 8. Cross-references

- `docs/SIM_ARC_VISUAL_REWRITE_BOOKMARK.md` — the bookmark this spec executes against. §2 findings + §3 sketch + §4 decisions all flow into this spec.
- `docs/SIM_ARC_RECON.md` — the original recon doc. S1-S4 shipped per its §6; this spec is the S11 follow-up. §10 cold-read findings disposition: items 2 + 3 (doc rot + accessor consistency) bundle into the S11.3 polish; item 1 (intruder ignores ScanMeshVisible) is resolved by the S11.1 rewrite (the IntruderEntity is sim-only now, with `show_deformed` as its toggle); items 4 + 5 + 6 stay banked.
- [[project-cf-device-design-sim-arc-recon]] — headline memo; update at end of arc to point at this spec as SHIPPED.
- [[feedback-bookmark-when-surface-levers-exhaust]] — three-session pattern; this spec is the recon output.
- [[feedback-autonomous-architecture]] — drove the D-Vis defaults.
- [[feedback-strip-the-knob-when-default-works]] — drove D-Vis3 in-place rewrite default.
- [[feedback-implement-measure-revert-pattern]] — drove D-Vis6 sub-leaf ladder shape.
- [[feedback-cold-read-review-post-ship]] — drives the cumulative cold-read + S11.3 polish bundle.
- [[feedback-cf-cast-tests-use-release]] + [[feedback-long-running-commands-use-file-redirect]] — gate cadence per sub-leaf.
- [[project-bevy-is-changed-footgun]] — pattern for `IntruderMeshKey` snapshot-and-compare in `update_intruder_mesh`.
- [[project-cf-device-design-sdf-layers]] — the SDF + MC machinery powering the intruder MC path.
- `tools/cf-device-design/src/insertion_sim_ui.rs:128` — `InsertionSimOutputs` definition; new field lands here.
- `tools/cf-device-design/src/insertion_sim_ui.rs:447` — `kick_off_simulation`; new `Res<CachedScanSdf>` parameter.
- `tools/cf-device-design/src/insertion_sim_ui.rs:555` — `run_sim_pipeline`; new `cached_sdf` parameter + per-step MC loop.
- `tools/cf-device-design/src/insertion_sim.rs:587` — `InsertionGeometry` definition (unchanged — no FEM-intruder snapshot needed per resolved SDF-source decision).
- `tools/cf-device-design/src/sdf_layers.rs:686` — `extract_layer_surface` (the reusable MC primitive; no changes needed).
- `tools/cf-device-design/src/main.rs:419` — `INTRUDER_COLOR` constant; recolored in S11.3.
- `tools/cf-device-design/src/main.rs:1361` — `LAYER_SURFACE_PALETTE`; `LAYER_SLAB_ALPHA` constant lands nearby.
- `tools/cf-device-design/src/main.rs:1455` — `update_cavity_mesh`; S11.3 force-hidden branch.
- `tools/cf-device-design/src/main.rs:1556` — `update_layer_meshes`; S11.2 slab-mesh swap + material alpha.
- `tools/cf-device-design/src/main.rs:1712` — `spawn_intruder_mesh`; simplified in S11.1.
- `tools/cf-device-design/src/main.rs:1764` — `update_intruder_transform` → `update_intruder_mesh` (renamed + reshaped in S11.1).
- `tools/cf-device-design/src/main.rs:2588` + `:2597` — Plugin chain; rename `update_intruder_transform` to `update_intruder_mesh`.
