# Scan Prep Design — `cf-scan-prep` GUI Tool

**Status:** active (opened 2026-05-12)
**Predecessor docs:** [`CASTING_ROADMAP.md`](CASTING_ROADMAP.md), [`SIM_SOFT_ROADMAP.md`](SIM_SOFT_ROADMAP.md)
**Implementation track:** Stage 2.5 (between Track F.2 v1 MVP and Stage 3 physical cast iter-1)

---

## Strategic context

Track F.2's v1 MVP shipped a capsule-fallback example that demonstrates the cast pipeline end-to-end. The capsule was the [Q2 fallback](CASTING_ROADMAP.md#architectural-decisions); the **scan-derived primary** path was deferred to iter-1 prep, where a real scanned reference geometry drives the cavity shape.

Real scans don't drop into `cf_cast::CastSpec` directly. They need preprocessing decisions that the capsule fallback didn't surface:

- **Simplify (decimate)** — modern 3D scanners routinely produce 1M-10M+ face meshes. cf-cast samples SDF on a 2 mm grid; mesh detail finer than the cell size is wasted. A 3 M-face scan adds ~30× compute to cf-cast's `SignedDistanceField` build + queries, ~60× memory pressure on Bevy rendering, and produces cleaned STLs the slicer chokes on. Boundary-preserving quadric edge collapse (via `meshopt-rs`) reduces a typical scan to 50k-200k faces with no perceptible quality loss for cast purposes. **Load-bearing for any scan-driven cf-cast workflow.**
- **Reorient** — scanner-frame ≠ cast-frame; the scan's local axes must be rotated so the demolding direction is `+z`
- **Recenter** — translate the scan so the cavity centroid sits at origin (offsets are then symmetric)
- **Trim** — drop scanner noise (floor, fixture, extraneous geometry) below a clip plane
- **Cap open boundaries** — body-part scans are inherently open (you stop scanning where the limb continues, leaving a cup-like cross-section). cf-cast's `mesh_sdf::SignedDistanceField` evaluator requires a **watertight** input; without closing the boundary, the scan's "inside" is undefined and SDF queries return garbage. **This makes capping a load-bearing feature, not a polish item.**
- **Mouth handling** — extend `+z` headroom past the scan's natural top so the mold cup opens cleanly
- **Inspect / validate** — sanity-check size vs print volume, surface mesh-quality issues (non-manifold edges, multi-shell topology)

These decisions need **visual feedback**. The user has tried scan-shell workflows before and knows from experience that programmatic-only preprocessing (rotation matrices in a config file, no preview) is intractable — the user can't tell whether the orientation is right without seeing the scan rendered in 3D with axis gizmos.

`cf-scan-prep` is the GUI tool that provides that feedback. Output: a cleaned STL with transforms baked + a TOML config that documents the preprocessing decisions for reproducibility. The cleaned STL is the input to `examples/cast/layered-silicone-device-v1-scan/`, which mirrors the v1 capsule example's structure but consumes the scan-derived plug.

---

## Target deliverable

A **scan preprocessed and ready for cast tooling**, produced by:

1. **Run** `cargo run -p cf-scan-prep -- /path/to/scan.stl`
2. **Bevy window opens** with the scan rendered, axis gizmos at origin, transformed-AABB wireframe, ground grid
3. **User adjusts** rotation / translation / clip / mouth via the right-side egui panels until the scan is oriented correctly with the demolding direction = `+z` and the cavity floor near `z=0`
4. **User clicks Save** — `<stem>.cleaned.stl` (transforms baked) + `<stem>.prep.toml` (config + provenance) land in the input scan's parent directory
5. **User runs** `CF_CAST_SCAN_STL_PATH=<stem>.cleaned.stl cargo run --release -p example-cast-layered-silicone-device-v1-scan` to generate the 4 mold STLs + procedure.md against the actual scan geometry

The cast example (Stage 2.6) produces the workshop artifacts; cf-scan-prep produces the cast example's input.

---

## Architectural decisions

Resolved 2026-05-12 by user:

- **Tool home** → **extend cf-viewer via library refactor + new `tools/cf-scan-prep` binary**. cf-viewer is already `[lib] + [[bin]]` with the heaviest reusable primitives (`OrbitCameraPlugin`) already in `cf-bevy-common`. The refactor lifts `RenderScale`, `setup_camera_and_lighting`, and `spawn_face_mesh` from `cf-viewer/src/main.rs` to `cf-viewer/src/lib.rs` so cf-scan-prep can import them. cf-view binary stays PLY-centric ("static visual review"), cf-scan-prep is STL-centric ("preprocessing"), each tool one job. ~2 hours active refactor + the new tool.
- **Rotation input** → **sliders + axis-snap buttons + quaternion readout**. Three Euler-angle sliders (intrinsic XYZ, displayed in degrees), 4 axis-snap buttons for common scanner-frame → cast-frame rotations, read-only quaternion display for advanced users. Internal state stored as `UnitQuaternion<f64>` to avoid gimbal-lock corruption. **No 3D drag-handle gizmo** in MVP — defer to Stage 2.6 if the slider UX proves insufficient.
- **Print-volume warnings** → **CLI-configurable build volume** (`--build-volume-mm W,D,H` flag). Default warns when cleaned AABB exceeds `250 × 210 × 210 mm` (Prusa MK3S baseline). User overrides for their specific printer.
- **Save behavior** → **single button, overwrites without prompt**. Iterative tuning means saving often; confirm dialogs add friction. Output paths are predictable from input filename; status bar flashes "Saved at HH:MM:SS" on success.
- **Scan asset path** → **CLI positional argument** (`cargo run -p cf-scan-prep -- /path/to/scan.stl`). Scan stays out of the repo per the CASTING_ROADMAP.md sanitization directive.
- **Cap open boundaries** → **auto-detect boundary loops + per-loop approve + manual plane override**. Tool runs `mesh-repair` boundary-loop detection, fits a least-squares plane to each loop, displays the proposed cap as a translucent polygon. User approves per-loop via checkboxes (multi-shell scans sometimes have intentional holes); per-loop manual override (axis dropdown + offset slider) for cases where auto-fit's R² is poor. Triangulation via inline ear-clipping (no new dep). **Rejected alternative**: line-drawing in 3D (geometric ambiguity from 2D click → 3D position; single line doesn't uniquely define a plane; positioning friction fights with orbit camera). Auto-detect handles the common case (planar boundary, R² > 0.85) with one click.
- **STL units convention** → **assume millimeters at load + CLI flag override**. STL files don't carry unit metadata; industry convention is mm. cf-cast and the rest of the workspace work in meters. cf-scan-prep divides loaded vertex coordinates by `1000` to convert mm → m, then the in-tool sliders/AABB readouts/saved STL all stay in meters. CLI flag `--stl-units mm|m|inch` lets users override (default `mm`). Scan Info panel displays the conversion (`"STL units: mm (vertex × 0.001 → meters)"`) so the assumption is visible — catches the most common user-confusion failure mode (loading a meter-scale STL with default mm assumption produces a 1000× too-small mesh; loading a mm-scale STL with `--stl-units m` produces a 1000× too-big mesh that cf-cast can't print).
- **Clip cut style** → **true plane intersection (not vertex-drop)**. For each triangle straddling the clip plane, compute the intersection edge, generate new vertices on the plane, retesselate the triangle into one above-plane (kept) and one or two below-plane (discarded). Result: clean planar cut along `z = clip_z`. **Rejected alternative**: vertex-drop (drop any triangle whose all 3 vertices are below the plane). Vertex-drop is simpler (~30 LOC) but produces jagged ragged edges that don't lie on the clip plane; the resulting open boundary has low plane-fit R² and degrades the auto-cap quality downstream. True intersection costs ~150 LOC of triangle-clipping but makes the clip-then-cap workflow clean (the resulting boundary lies exactly on the clip plane, R² = 1.0).
- **Cap normal orientation** → **winding-determined outward via mesh-side-of-plane heuristic**. The triangulated cap's vertex winding determines its normal direction; the cap must face outward (away from the mesh interior) so `mesh_sdf` computes the right inside/outside. Heuristic: project the loop centroid + boundary onto the auto-fit plane, walk counterclockwise as seen from the mesh-side of the plane (the side with more original mesh geometry). Fallback if heuristic is ambiguous (loop straddles both sides ~50/50): use the average outward normal of the boundary edges' adjacent faces and orient the cap-plane normal to match.
- **Save atomicity** → **`.tmp` + atomic rename**. Both files (`.cleaned.stl` and `.prep.toml`) write to `<stem>.cleaned.stl.tmp` + `<stem>.prep.toml.tmp` first, then atomic-rename to final names only after both writes succeed. If either write fails, neither final file lands and the previous version (if any) is preserved. Status bar surfaces the FS error in red.
- **Simplify (decimate) algorithm** → **`meshopt-rs` workspace dep, boundary-preserving quadric edge collapse**. New workspace dep (Rust crate wrapping Arseny Kapoulkine's `meshoptimizer` C library — battle-tested in Unity, Unreal, Bevy itself; MIT licensed). Boundary preservation is **required** (NOT optional): if decimation collapses edges along the open boundary loop, the cap polygon shape changes and may become non-planar from edge merges, breaking downstream cap detection. `meshopt::simplify` (NOT `simplify_sloppy`) preserves boundaries by setting border-edge collapse cost to ∞. **Rejected alternatives**: inline Garland-Heckbert quadric edge collapse (~500-800 LOC, risk of subtle quadric-math bugs, may not match meshopt's quality tuning); vertex clustering / grid snap (~100 LOC, fast but doesn't preserve boundaries cleanly, uneven triangle distribution). Default target: 200k faces (10× over the 50k surface-continuity floor; well below the 500k "may be sluggish" threshold); user-configurable via slider 1k-1M.

---

## Workflow

End-to-end, scan to mold:

```
[scanner / photogrammetry / etc.]
       ↓ (raw STL)
[user pre-trims to single shell if needed; external tool — MeshLab / Blender]
       ↓ (single-shell STL)
[cf-scan-prep GUI]
   1. Load STL via CLI arg (default --stl-units mm; vertex × 0.001 → meters)
   2. Inspect — vertex/face count, AABB, mesh-repair diagnostics, units assumption
   3. Simplify — decimate to target face count (default 200k; auto-banner at >500k); meshopt boundary-preserving
   4. Reorient — rotate so demolding direction = +z
   5. Recenter — pick one: `[Center origin]` translates centroid to origin, OR `[Floor → z=0]` translates cavity floor to z=0
   6. Optionally clip — drop scanner-noise floor below z=clip_z
   7. Cap open boundaries — auto-detect open loops, fit planes, approve per loop
   8. Set mouth extension — +z headroom for cup opening
   9. Save — emits <stem>.cleaned.stl (watertight, simplified) + <stem>.prep.toml
       ↓
[examples/cast/layered-silicone-device-v1-scan/]
   - reads cleaned STL via CF_CAST_SCAN_STL_PATH env var
   - reads .prep.toml for mouth_extension value
   - builds Solid::from_sdf(scan_sdf, scan_bounds) plug
   - cumulative shells via plug.offset(thickness).subtract(plug)
   - cf-cast::CastSpec → 4 mold STLs + procedure.md
       ↓
[workshop: 3D print → pour → cure → demold]
```

cf-scan-prep handles steps 1-9. Steps before (raw scan acquisition + multi-shell trim) and after (workshop) are user-domain.

---

## Window layout

```
┌─────────────────────────────────────────────────────────────────┐
│                                              ┌──────────────────┤
│                                              │ ▼ Scan Info      │
│                                              │   File: scan.stl │
│                                              │   Vertices: 18.4k│
│                                              │   Faces: 24.5k   │
│                                              │   Raw AABB:      │
│         3D Bevy viewport (central)           │     W: 80.3 mm   │
│         ─────────────────                    │     D: 60.1 mm   │
│         · scan mesh (transformed)            │     H: 120.7 mm  │
│         · axis gizmos at origin              │   Volume: 12.4cm³│
│         · transformed AABB (wireframe)       │                  │
│         · clip plane (translucent, opt)      │ ▼ Simplify       │
│         · cap visualization (translucent)    │   Current: 24.5k │
│         · mouth extension (translucent, opt) │   Target: ──○──  │
│         · ground grid (subtle, z=0)          │   200k faces     │
│                                              │   [Apply]        │
│         (orbit camera; mouse-drag)           │   [Reset orig]   │
│                                              │                  │
│                                              │ ▼ Reorient       │
│                                              │   Roll  ──○──    │
│                                              │   Pitch ──○──    │
│                                              │   Yaw   ──○──    │
│         (orbit camera; mouse-drag)           │   [Snap +Z up]   │
│                                              │   [Snap -Z up]   │
│                                              │   [Snap +Y -> +Z]│
│                                              │   [Snap -Y -> +Z]│
│                                              │   [Reset rot]    │
│                                              │   q = (w,x,y,z)  │
│                                              │                  │
│                                              │ ▼ Recenter       │
│                                              │   X / Y / Z      │
│                                              │   [Center origin]│
│                                              │   [Floor → z=0]  │
│                                              │   [Reset trans]  │
│                                              │                  │
│                                              │ ▼ Clip floor     │
│                                              │   ☐ Enabled      │
│                                              │   Z ──○──  mm    │
│                                              │   Drops X% verts │
│                                              │                  │
│                                              │ ▼ Cap boundaries │
│                                              │   [Scan]         │
│                                              │   Found N loop(s)│
│                                              │   ☑ Loop A R²=… │
│                                              │   ☐ Manual plane │
│                                              │   [Apply caps]   │
│                                              │                  │
│                                              │ ▼ Mouth ext.     │
│                                              │   +Z ──○── mm    │
│                                              │   ☐ Preview      │
│                                              │                  │
│                                              │ ▼ Cleaned AABB   │
│                                              │   W / D / H      │
│                                              │   ⚠ warnings     │
│                                              │                  │
│                                              │ ▼ Save           │
│                                              │   Output dir     │
│                                              │   File previews  │
│                                              │   [Save]         │
│                                              │   [Reset all]    │
└──────────────────────────────────────────────┴──────────────────┘
│ Status: loaded scan.stl • 6 issues: 4 holes, 2 non-manifold edges │
└─────────────────────────────────────────────────────────────────┘
```

- **Central viewport** — Bevy 3D, transformed scan + overlays
- **Right panel** — collapsible egui sidebar, ~320 px wide, all controls
- **Bottom status bar** — ~30 px, load info + diagnostic counts + save confirmations

---

## Visualization layer

**Always-on overlays in the viewport:**

| Element | Purpose |
|---------|---------|
| Axis gizmos at origin (colored arrows) | Show `+x` / `+y` / `+z` reference; `+z` labeled "demolding direction" |
| Subtle ground grid at `z=0` | Anchors the eye; reference for `[Floor → z=0]` button |
| Transformed AABB wireframe | Shows post-rotation+translation bounding box, updates live on slider change |

**Toggleable overlays (per-section checkbox):**

| Element | Default | Trigger |
|---------|---------|---------|
| Clip plane (translucent disc at `z = clip_z`) | OFF | `Clip floor → Enabled` checkbox |
| Open boundary loop highlights (red linestrips along open edges) | ON after `Scan` | `Cap boundaries → [Scan]` button populates the loop list |
| Per-loop cap polygon (translucent fill on the proposed cap plane, color per loop for disambiguation) | ON for checked loops | Each `Loop X` checkbox toggles its own cap-overlay rendering |
| Cap-stale dim overlay (gray transparency over loop list area + viewport boundary highlights) | ON when transform changes after a `Scan` | Auto-applied; clears on next `[Scan]` click |
| Mouth extension overlay (translucent box from `transformed_aabb.z_max` extending `+z` by `mouth_z` mm) | OFF | `Mouth ext. → Preview` checkbox |
| Cleaned-result preview (semi-transparent body extruded by the device outer-shell thickness from the cf-cast CastSpec) | OFF | Future Stage 2.6 — banked, not in MVP |

**Camera behavior:** orbit camera (cf-bevy-common `OrbitCamera`), framed on initial load to scan's transformed AABB. **Position preserved across transform tweaks** — re-framing on every slider change would be jarring. A `[Reframe]` button in the side panel lets the user manually re-center the view if needed.

---

## Panel specifications

### 1. Scan Info (read-only)

Static fields (file path, STL units, raw AABB) loaded at startup; vertex/face counts update when the Simplify panel applies a decimation pass (commit #5):
- File path (from CLI arg)
- Vertex count, face count
- **STL units assumption** — `"mm (vertex × 0.001 -> meters)"` by default; `"m (no scale)"` if `--stl-units m` was supplied; `"inch (vertex × 0.0254 -> meters)"` if `--stl-units inch`. Arrow is ASCII `->` (not Unicode `→`) because egui's default `ProggyClean` font ships with Latin-1 + basic-punctuation coverage only; U+2192 falls outside the glyph set and renders as `□` (missing-glyph box). `×` (U+00D7, Latin-1 Supplement) is in coverage so it stays. Surfaces the load-time conversion so users notice mismatches immediately.
- Raw AABB dimensions in mm (W × D × H) — displayed in mm regardless of `--stl-units` (workshop measurements are in mm; consistency with the user's mental model)
- Estimated volume in cm³ (via `mesh-measure`)

### 2. Simplify (decimate)

Reduces scan face count via boundary-preserving quadric edge collapse so all downstream panels operate on a tractable mesh size. Critical for scans > 500k faces (rotating-table scans of a 100mm device routinely produce 1-5M faces).

- **Read-only header**: `"Current: <face_count> (<file_size_mb> MB)"`
- **Target slider**: 1k - 1M faces, default **200k** (10× over the 50k surface-continuity floor; well below the 500k "may be sluggish" threshold; matches cf-cast's 2 mm-cell SDF sampling density × surface-continuity headroom)
- **Algorithm**: `meshopt::simplify` (NOT `simplify_sloppy`) — boundary-preserving quadric edge collapse via the `meshopt-rs` crate
- `[Apply simplify]` button — runs decimation, swaps the displayed Bevy mesh entity, updates Scan Info panel's vertex/face counts. Status bar reports actual achieved face count: `"Reduced 3.35M -> 198,432 faces in 4.2s"` (ASCII `->`, not Unicode `→` — same egui font glyph constraint as the STL units row per [`StlUnits::panel_label`](#stl-units-convention)).
- `[Reset to original]` button — restores the loaded mesh; user can re-target and re-apply iteratively
- **Boundary preservation is non-optional**: if the open-boundary loop's vertices get collapsed during decimation, the cap polygon shape changes (and may become non-planar from edge merges), breaking downstream cap detection. `meshopt::simplify` enforces this by setting border-edge collapse cost to ∞.
- **Auto-suggest banner** at load: when the scan's face count exceeds 500k, the status bar surfaces `"Scan has <N>M faces. Recommended: simplify to ~200k for performance. Open the Simplify panel to decimate."` Non-blocking; user can ignore if they have a reason to keep full resolution.
- **Position in workflow**: panel #2, BEFORE Reorient, so all subsequent panels (Reorient/Recenter/Clip/Cap/Mouth/AABB/Save) operate on the simplified mesh. Decimating after transforms would re-walk the larger mesh through every transform tick.

### 3. Reorient

Rotation state stored internally as `UnitQuaternion<f64>`; Euler conversion happens at the slider boundary so gimbal-lock state corruption is avoided.

- Three sliders for **roll / pitch / yaw** (intrinsic XYZ Euler, displayed in degrees, range −180° to +180°)
- Five shortcut buttons for common scanner-frame → cast-frame rotations:
  - `[Snap +Z up]` — identity (default; matches `1` keyboard shortcut)
  - `[Snap -Z up]` — 180° flip about the X axis (matches `2` keyboard shortcut)
  - `[Snap +Y -> +Z]` — 90° about X (the "scanner had Y up" case; matches `3` keyboard shortcut)
  - `[Snap -Y -> +Z]` — −90° about X
  - `[Snap +X -> +Z]` — −90° about Y (the "scanner had X up" case, e.g. horizontal scanners)

  Snap button labels use ASCII `-` / `->` rather than Unicode `−` / `→` because egui's default `ProggyClean` font lacks the U+2212 (minus) and U+2192 (rightwards arrow) glyphs — same constraint as the §1 STL-units row. The narrative angle values below (e.g. "−90° about X") stay Unicode since they're rendered by markdown viewers, not egui.
- `[Reset rotation]` button → identity quaternion
- Read-only quaternion display (4 components: `w`, `x`, `y`, `z`) for advanced users

### 4. Recenter

- Three sliders for **X / Y / Z translation** (in mm, range derived from scan AABB ±2× max dimension)
- `[Center origin]` button → translate by `−transformed_aabb.center()`
- `[Floor → z=0]` button → translate by `−transformed_aabb.min.z * Vec3::Z`. Useful default for cast prep (cavity floor sits at `z=0`)
- `[Reset translation]` button → zero

### 5. Clip floor (optional)

- Checkbox `Enabled` (default off)
- Z slider for plane height (range derived from transformed AABB; default at `transformed_aabb.min.z + 0.1×height`)
- Live readout: `"Drops X% of vertices below z=Y mm"` — sanity check before saving destroys mesh
- When enabled, the translucent disc visualization renders in the viewport
- **Cut style: true plane intersection** — for each triangle straddling the clip plane, compute the intersection edge, generate new vertices on the plane, retesselate the triangle into one above-plane (kept) and one or two below-plane (discarded). Result: clean planar cut along `z = clip_z`, suitable for capping without R² loss. Implementation cost: ~150 LOC of triangle-clipping (commit #7); rejected vertex-drop alternative produces jagged boundaries that degrade auto-cap quality. See [Architectural decisions](#architectural-decisions) §Clip cut style for the full rationale.
- Edge case: if `>95%` of vertices drop, status bar warns `"Clip removes nearly all geometry — verify Z value"` (catches above-mesh / inverted-axis user errors)

### 6. Cap open boundaries

Closes open boundary loops with a flat planar cap so the saved mesh is watertight (required by `mesh_sdf::SignedDistanceField` downstream in cf-cast).

- `[Scan]` button — runs `mesh-repair` boundary-loop detection on the **transformed** mesh (post-rotation/recenter/clip; ensures the cap planes are computed in cast-frame). Cheap; ~10ms for 50k-face meshes.
- **Loop list** — one entry per detected open boundary, showing:
  - Checkbox to include/exclude this loop in the cap operation (default: checked if vertex count ≥ 8 — small loops are usually acceptable holes or scanner artifacts)
  - Vertex count
  - Plane-fit R² (least-squares plane through the loop's vertices)
  - Cap area in mm² (derived from the proposed cap polygon)
- **Per-loop manual override** (collapsible sub-section, off by default):
  - Axis dropdown: `+X`, `−X`, `+Y`, `−Y`, `+Z`, `−Z`
  - Offset slider in mm (range derived from loop's AABB ±2× extent)
  - When enabled, replaces the auto-fit plane for that loop
  - Useful for low-R² loops or when the user wants the cap slightly above/below the auto-fit
- `[Apply caps]` button — bakes the selected caps into the mesh on save (triangulation via inline ear-clipping; new vertices + faces added). Status bar confirms: `"Capped N boundary loops"` 
- After capping, a re-`[Scan]` will detect zero open loops (sanity confirmation)
- **Mesh-repair gate**: the status bar warning that surfaced at load (`"6 issues: 4 holes, 2 non-manifold edges"`) updates after capping — `holes` count drops to zero for the loops that were capped
- **Cap state invalidation on transform change**: any reorient / recenter / clip change invalidates the loop list (planes were fit to the previous transformed mesh; new transform → planes need re-fit). When invalidated, the loop list dims to gray with overlay text `"Transform changed — re-Scan to refresh"`. User clicks `[Scan]` again to recompute. Don't auto-rescan on every slider tick (defeats the per-loop checkbox state the user already set + re-runs `mesh-repair` for nothing).
- **Cap normal orientation**: triangulated cap winding determines its outward normal direction. Cap must face away from the mesh interior so `mesh_sdf::SignedDistanceField::from_mesh` computes correct inside/outside. Heuristic: walk the boundary loop counterclockwise as seen from the mesh-majority side of the auto-fit plane (the side containing the majority of original mesh geometry). Fallback (loop straddles both sides ~50/50): use the average outward normal of the boundary edges' adjacent faces and orient the cap-plane normal to match. Verify post-cap by sampling a small SDF probe at `aabb.center()` — should be negative (inside) for a watertight cleaned mesh.

### 7. Mouth extension

- `+Z` slider in mm (range 0–50 mm, default 15 mm — matches the v1 capsule example's headroom)
- Checkbox `Preview` — when checked, translucent box renders from `transformed_aabb.max.z` extending upward by `+Z` mm
- The mouth extension is **NOT baked into the cleaned STL** — it's a cf-cast bounding-region hint passed through the TOML

### 8. Cleaned AABB (read-only)

Live-updates as user drags sliders:
- Width / depth / height after rotation + translation + clip (in mm)
- Inline warnings if dimensions exceed CLI-configured build volume (default `250 × 210 × 210 mm`):
  - Per-axis warning: `"⚠ exceeds build volume on Z axis (135.7 > 210 mm)"`
  - Multi-axis warning if multiple exceeded
- All warnings advisory; save proceeds regardless

### 9. Save

- Output directory display (default: parent of input scan; CLI flag `--output-dir <path>` overrides for read-only-input-dir cases)
- Filename preview (`<input_stem>.cleaned.stl` + `<input_stem>.prep.toml`)
- `[Save]` button — single click, overwrites without prompt; status bar flashes `"Saved at HH:MM:SS"` on success
- `[Reset all]` button — resets transforms (rotation/translation back to identity), clip off, mouth = 15 mm, no caps applied. **Does NOT undo simplification** — the simplified mesh is preserved (re-decimating on every `[Reset all]` click would cost 4+ seconds for large scans). To undo simplify, use the Simplify panel's own `[Reset to original]` button.
- **Atomic write contract**: both files write to `<stem>.cleaned.stl.tmp` + `<stem>.prep.toml.tmp` first, then atomic-rename to the final names ONLY after both writes succeed. If either write fails, neither final file lands; previous versions (if any) are preserved. Filesystem failures (read-only mount, disk full, permission denied) surface as a red status-bar message: `"Save failed: <fs_error>"` for ~5 seconds. App stays open for retry; no panic.
- **Pre-save validation**: reject save with red status-bar error if any transform value is non-finite (`NaN` / `±Inf`). Defensive against future slider changes; current sliders only emit valid f64.

---

## Output format

### `<input_stem>.prep.toml`

Caller-readable config + provenance record:

```toml
# Generated by cf-scan-prep <tool_version>
# Applied transforms in order: rotation → translation → clip (if enabled).

[scan_prep]
source_stl = "/abs/path/to/scan.stl"
tool_version = "0.1.0"
generated_at = "2026-05-12T16:30:00Z"

[simplify]
# Decimation applied to the loaded scan before any other preprocessing.
# Boundary preservation enabled so the cap loop topology survives.
applied = true
algorithm = "meshopt_quadric_edge_collapse"
algorithm_version = "0.4.1"  # meshopt-rs version
target_face_count = 200000
achieved_face_count = 198432
original_face_count = 3352068
boundary_preserved = true

[transform.rotation]
# UnitQuaternion (w, x, y, z) applied to scan vertices.
quaternion = [0.7071, 0.0, 0.0, 0.7071]

[transform.translation]
# Translation in meters, applied after rotation.
m = [0.0, 0.0, -0.05]

[clip]
# Drops vertices below z = horizontal_plane_z_m (transformed frame).
# If `enabled = false`, no clip is applied.
enabled = true
horizontal_plane_z_m = -0.005

[caps]
# Boundary loops capped during preprocessing. Each entry records the
# detected loop's auto-fit plane (so the saved STL's new triangles can
# be reconstructed / audited) plus whether the user overrode it
# manually. After capping, the saved STL is watertight.
applied = true
[[caps.loops]]
loop_index = 0
vertex_count = 87
plane_fit_r_squared = 0.94
plane_normal = [0.02, -0.01, 0.999]
plane_offset_m = 0.121
manual_override = false
new_face_count = 85  # ear-clipped triangles added

[mouth_extension]
# Headroom above the scan's transformed +z extent. Used by cf-cast's
# bounding-region sizing; not applied to the cleaned STL itself.
plus_z_m = 0.015

[output]
cleaned_stl = "scan.cleaned.stl"

[output.aabb_m]
# AABB of the cleaned mesh (post-transform, post-clip), in meters.
min = [-0.040, -0.030, -0.060]
max = [ 0.040,  0.030,  0.060]
```

Schema versioned implicitly via `tool_version`; future TOML schema changes bump the version and document migration.

### `<input_stem>.cleaned.stl`

STL with rotation + translation + clip + caps baked into vertex positions. Mouth extension is **NOT** baked (it's a bounding-region hint, not a mesh modification).

After capping, the cleaned STL is **watertight** — each cap loop's auto-fit (or user-overridden) plane is triangulated via ear-clipping and added as new faces. This is required for `mesh_sdf::SignedDistanceField::from_mesh` to produce valid SDF queries; an open scan would yield undefined inside/outside topology and corrupt cf-cast's mold-cup CSG.

The cast example (`examples/cast/layered-silicone-device-v1-scan/`) consumes only the cleaned STL for geometry; optionally reads the TOML for the `mouth_extension.plus_z_m` value to size the bounding region.

---

## Validation rules

**At load:**
- Run `mesh-repair` diagnostics on the loaded scan (closed manifold? non-manifold edges? holes? disconnected shells?)
- Display issue count in status bar (`"6 issues: 4 holes, 2 non-manifold edges"`)
- Allow user to continue — diagnostic is **informational, not blocking**. Open boundaries surfaced here are addressed via the Cap panel; uncapped loops at save time fire a yellow warning so the workflow oversight is visible. Non-manifold-edge / disconnected-shell counts surface for awareness but aren't auto-fixed (out-of-scope per [MVP scope](#mvp-scope-vs-banked-items) §9).
- **Face-count auto-suggest banner** — if loaded scan has > 500k faces, status bar surfaces `"Scan has <N>M faces. Recommended: simplify to ~200k for performance. Open the Simplify panel to decimate."` Non-blocking; user can ignore if they have a reason to keep full resolution. Banner clears once Simplify is applied.

**Live (per-frame):**
- "Drops X% of vertices below z=Y mm" under clip plane control (updates on slider change)
- Cleaned-AABB dimensions update on every slider change
- Per-loop R² readout updates on `[Scan]` button click in the Cap panel (boundary detection re-runs after transform changes)

**At save:**
- If cleaned AABB exceeds CLI-configured build volume on any axis → red warning in side panel
- If clip drops `>50%` of vertices → yellow warning (`"clip is removing most of the mesh — verify"`)
- If any open boundary loop remains uncapped at save time → yellow warning (`"N open boundary loops uncapped — cf-cast SDF will produce undefined topology, verify"`). Doesn't block save (user may want to inspect a non-watertight STL externally), but bold so the workflow oversight is visible.
- All warnings are advisory; save proceeds regardless
- **Filesystem error pattern** — read-only mount, disk full, permission denied, or atomic-rename failure surfaces as a **red status-bar message** (`"Save failed: <fs_error>"`) for ~5 seconds, then auto-clears. The app stays open for retry — no panic, no quit. Pre-save validation rejects non-finite transform values (`NaN` / `±Inf`) before any I/O attempt with the same red status-bar pattern.
- **STL writer dual-mesh support** — the cleaned STL contains the transformed-clipped scan PLUS the cap triangles concatenated in. `mesh-io::save_stl` writes a flat triangle list, so the cap polygon's triangulated faces append to the same STL without needing multi-solid file structure. Recon during commit #12 to confirm `mesh-io::save_stl`'s contract handles this.

---

## Keyboard shortcuts

| Key | Action |
|-----|--------|
| `Esc` | Quit |
| `Ctrl+S` | Save |
| `R` | Reset all transforms |
| `C` | Center origin |
| `F` | Floor → z=0 |
| `A` | Apply caps (acts on currently-detected loops; no-op if none) |
| `1` / `2` / `3` | `Snap +Z up` / `Snap -Z up` / `Snap +Y -> +Z` (matches the first three Reorient panel buttons; ASCII per §3) |
| Mouse drag | Orbit camera |
| Scroll | Zoom |

---

## MVP scope vs banked items

**In MVP:**
- All 9 panels above (Scan Info, **Simplify**, Reorient, Recenter, Clip floor, Cap open boundaries, Mouth ext., Cleaned AABB, Save)
- All visualization elements above (axis gizmos, transformed-AABB wireframe, ground grid, optional clip plane disc, open boundary loop highlights, per-loop translucent caps, mouth-extension overlay)
- All keyboard shortcuts above
- Output: cleaned STL (simplified to target face count + watertight after capping) + TOML config (with simplify provenance + capped-loop provenance)
- Validation: load-time mesh-repair diagnostic + face-count auto-suggest banner if > 500k faces + save-time AABB warning + clip-% live readout + uncapped-loop warning + per-loop R² readout + FS error red-status-bar pattern + pre-save non-finite-transform rejection

**Banked (deferred to Stage 2.6+):**

1. **Live mold-cup preview** — translucent overlay showing what cf-cast will produce. Requires running cf-cast inline on every transform change; complexity worth deferring.
2. **Multiple clip planes** — only horizontal-Z plane in MVP
3. **Arbitrary-plane clip** (non-horizontal normal) — same
4. **Painted trim region** (Boolean brush) — substantial new UI; defer
5. **Auto-PCA initial orientation guess** — useful but adds heuristic surface area
6. **Multi-shell selection** (pick one of disconnected meshes) — out of scope; assume user pre-trims to single shell externally
7. **Undo/redo** — state-management complexity; user can edit the TOML by hand if they need to back out
8. **Wireframe / shaded toggle** — visualization polish
9. **Mesh repair (auto-fix)** — only diagnose for MVP, don't modify the mesh
10. **3D rotation gizmo (drag-the-mesh handles)** — see "Architectural decisions" §Rotation input; defer if slider UX proves sufficient

---

## Implementation arc structure

14-commit plan, **~28-37 hours** active across multiple sessions (sum of per-commit estimates with the per-commit ranges):

| # | Commit | Scope | Est |
|---|--------|-------|----:|
| 1 | `docs(scan-prep): design spec` | This document | ~30 min |
| 2 | `refactor(cf-viewer): lift RenderScale + setup helpers to lib` | Extract `RenderScale`, `compute_render_scale`, `scale_aabb`, `setup_camera_and_lighting`, `spawn_face_mesh` from main.rs to lib.rs; cf-view binary thinned; per-crate gates + xtask grade | ~2 hr + 25 min grader |
| 3 | `feat(tools): cf-scan-prep crate scaffold + STL load with units` | New `tools/cf-scan-prep/` workspace member, deps (cf-viewer + bevy + bevy_egui + mesh-io + mesh-measure + mesh-repair + meshopt-rs + nalgebra + serde + toml + anyhow + clap), blank Bevy app loading scan via CLI with `--stl-units mm\|m\|inch` flag (default mm; vertex × 0.001 → meters), rendering it via cf-viewer helpers with `OrbitCamera::UpAxis = +Z` (cast-frame demolding axis convention; orbit camera frames the raw AABB at load), error-overlay pattern for load failures | ~1.5 hr |
| 4 | `feat(scan-prep): Scan Info panel` | First egui panel, validates the wiring; surfaces vertex/face counts + AABB in mm + units assumption | ~1 hr |
| 5 | `feat(scan-prep): Simplify (decimate) panel + meshopt-rs integration` | New panel #2; target-face-count slider (1k-1M, default 200k); `[Apply simplify]` runs `meshopt::simplify` (boundary-preserving) and swaps Bevy mesh entity; `[Reset to original]` restores loaded mesh; status bar reports achieved face count + elapsed; auto-suggest banner if loaded scan > 500k faces; updates Scan Info panel face counts; TOML schema gains `[simplify]` block | ~3-4 hr |
| 6 | `feat(scan-prep): Reorient panel` | Sliders + snap buttons + quaternion readout. **Live transforms via Bevy `Transform` component**, not mesh rebuild — slider drags update one component value, no per-tick vertex-buffer copy. Mesh entity spawns once at load; transformed AABB updates per-frame from `Transform * raw_aabb`. Critical for >50k-face perf even after simplify (still 50k-200k faces typical). | ~3-4 hr |
| 7 | `feat(scan-prep): Recenter panel` | Translation sliders + center + floor → z=0 buttons. `[Floor → z=0]` uses post-clip min.z if clip enabled, raw transformed min.z otherwise. | ~1-2 hr |
| 8 | `feat(scan-prep): Clip floor panel + true plane intersection` | Toggle, slider, translucent disc, drop-% live readout. **True plane intersection** (~150 LOC of triangle-clipping): each straddling triangle generates intersection vertices on the plane, retesselates into above-kept + below-discarded sub-triangles. Result: clean planar cut suitable for capping. Rejected vertex-drop alternative would produce jagged ragged boundaries that degrade auto-cap R². | ~3-4 hr |
| 9 | `feat(scan-prep): Cap open boundaries panel + auto-detect + planar cap triangulation` | `mesh-repair` boundary loop extraction (or inline edge-walk if mesh-repair only flags edges); least-squares plane fit (SVD via nalgebra); inline ear-clipping triangulation (~200 LOC) with fan-fallback for degenerates; per-loop checkboxes + manual override sub-panel; red boundary linestrip overlay; translucent cap polygon overlay; **cap-stale invalidation on transform change** (loop list dims with "re-Scan to refresh" overlay); **outward cap normal via mesh-side-of-plane heuristic + boundary-edge-adjacency fallback** | ~4-6 hr |
| 10 | `feat(scan-prep): Mouth extension panel + preview overlay` | Slider, translucent box from transformed `z_max` | ~1-2 hr |
| 11 | `feat(scan-prep): Cleaned AABB + CLI build-volume thresholds` | Read-only panel, configurable via `--build-volume-mm W,D,H` flag | ~1-2 hr |
| 12 | `feat(scan-prep): Save panel + TOML + cleaned STL writer + atomic write` | Save button, output naming, status-bar feedback, uncapped-loop warning. **Atomic write**: `.tmp` files first, atomic rename only on success. **FS error pattern**: red status-bar `"Save failed: <fs_error>"` for ~5s, app stays open. **Pre-save validation**: reject non-finite transform values. **`--output-dir <path>` CLI flag** for read-only-input-dir cases. | ~2.5-3.5 hr |
| 13 | `feat(scan-prep): mesh-repair diagnostics + keyboard shortcuts` | Load-time diagnostic in status bar; Esc/Ctrl+S/R/C/F/A/1-2-3 shortcuts | ~1 hr |
| 14 | `feat(examples): cast/layered-silicone-device-v1-scan` | New example consuming `<stem>.cleaned.stl` via `CF_CAST_SCAN_STL_PATH` env var; mirrors v1 capsule's structure | ~2 hr |

Plus the slice-ship-log update + auto-memory at the end — usually inline with the final ship commit.

---

## Risks + uncertainty

Worth surfacing now so failures aren't surprises:

- **Bevy 0.18 + bevy_egui interactions** — the workspace is on Bevy 0.18; bevy_egui 0.39.1 is the first release that pins `bevy ^0.18.0`. Should be stable but version churn possible.
- **Translucent overlay rendering** — Bevy's PBR pipeline doesn't always handle translucent materials cleanly with mesh occlusion. The clip-plane disc and mouth-extension box may need custom material setup or a separate render pass. Banked as risk; investigate during commit #8 / #9.
- **Quaternion-to-Euler-to-quaternion round-trip drift** — three slider sources of truth → one quaternion → display back as Euler can drift after multiple updates. Mitigation: the slider VALUES are the source of truth (not the displayed quaternion); each slider change re-derives the quaternion from the three current slider values. The quaternion display is computed from the quaternion-of-record at render time.
- **Mesh-repair diagnostic semantics** — `mesh-repair` returns a structured issue list, but mapping that to a human-readable status-bar string ("6 issues: 4 holes, 2 non-manifold edges") needs verification of which issue types it actually surfaces. Recon during commit #13.
- **Boundary-loop extraction** — commit #9 needs to either consume `mesh-repair`'s boundary-edge output or implement loop-walking inline (~50 LOC). If `mesh-repair` doesn't expose loop-grouping (only edge-flagging), inline implementation is needed. Recon at commit #9 start.
- **Ear-clipping edge cases** — degenerate boundary polygons (self-intersecting after projection to plane, near-collinear vertices, very thin sliver triangles) can break naive ear-clipping. Mitigation: pre-validate the projected loop is simple (no self-intersections); fall back to fan-triangulation from centroid if ear-clipping fails (less optimal triangle quality but always succeeds for star-shaped polygons, which boundary loops typically are).
- **Cap quality vs scan resolution** — high-resolution scans produce dense boundary loops (hundreds of vertices); ear-clipping is O(n²) so 500-vertex loops cost ~250k operations (still <100ms, fine). Pathological scans with 10k-vertex boundary loops would need a faster triangulation (Delaunay, ~O(n log n)) but are unusual for body-part scans.
- **Bevy `Transform` component for live transforms** — commits #5 and #6 update the mesh entity's `Transform` component on slider changes (rotation + translation), NOT rebuild the mesh's vertex buffer. Bevy applies `Transform` per-frame on GPU; per-tick CPU cost is ~zero. Without this, dragging a slider on a 100k-face scan would rebuild the vertex buffer 60×/sec and lag the UI noticeably. Spec'd explicitly because it's easy to miss when first wiring egui slider → Bevy mesh updates.
- **STL units mismatch is the most common load-time user error** — loading a meter-scale STL with default `--stl-units mm` produces a 1000× too-small mesh; loading mm-scale with `--stl-units m` produces a 1000× too-big mesh. Mitigation: Scan Info panel surfaces the assumption in panel text; if the displayed AABB dimensions read absurd (e.g., 0.08 mm or 80,000 mm) the user should suspect the units flag. No way to fully auto-detect since STL carries no metadata.
- **True plane intersection edge cases** — triangles parallel to the clip plane (lying ON the plane) need careful handling: include in kept-side if their average z is >= clip_z, else discard. Triangles with one vertex EXACTLY on the plane should treat that vertex as above-plane (keep the triangle) to avoid degenerate sub-triangles. Implementation notes for commit #8.
- **Filesystem path edge cases** — input scan path with no parent (e.g., `scan.stl` invoked from cwd) → output dir defaults to cwd. Path with non-ASCII characters → handled by `PathBuf` natively. Symlinks → `std::fs::canonicalize` to resolve; output uses canonicalized parent dir.
- **Boundary preservation under decimation** — `meshopt::simplify` (NOT `simplify_sloppy`) preserves border edges by setting their collapse cost to ∞. If the implementation accidentally uses the sloppy variant, the open-boundary loop's vertex positions get collapsed, the cap polygon shape changes, and the auto-fit plane becomes wrong (R² drops, the cap might no longer be planar). Verify in commit #5 that the meshopt-rs API call uses the non-sloppy variant + that border-preservation is enabled (it's the default but worth pinning in a unit test that runs decimation on a known open mesh and asserts the boundary loop topology is preserved).
- **`meshopt-rs` is the first new workspace dep added to this arc** — wraps Arseny Kapoulkine's `meshoptimizer` C library. Brings a small build-time C-toolchain dependency (cargo handles via `cc` build script). Battle-tested in production engines (Unity, Unreal, Bevy itself); MIT-licensed. If meshopt-rs proves unavailable / blocked, fallback is inline Garland-Heckbert quadric edge collapse (~500-800 LOC, ~2-3× the commit-#5 estimate). Recon at commit #5 start to confirm the latest meshopt-rs version pins to a Bevy-0.18-compatible toolchain.
- **Decimation-then-cap interaction** — the workflow order (Simplify → Cap) means cap detection runs on the SIMPLIFIED boundary. The boundary loop's vertex count drops from (e.g.) 800 → 60 after a 16× decimation. Ear-clipping cost drops from ~640k ops to ~3.6k ops — strictly faster. Cap polygon's plane-fit R² should be UNCHANGED (boundary points remain coplanar; meshopt preserves their positions). Verify in commit #9 with a unit test that pins R² ≥ original-mesh's R².
- **Workshop iteration cadence** — the user may discover during real-scan work that some MVP design decisions don't hold (e.g., horizontal-clip-only is too restrictive, or the mouth-extension-preview-as-translucent-box is hard to read against the scan). v2 polish will follow iter-1.

---

## Update protocol

Mirrors `CASTING_ROADMAP.md`:

1. Move panels from "spec'd" to "shipped" in the slice ship log on each commit.
2. Add one-line entries to the slice ship log below with date + commit SHA.
3. If a panel splits or new sub-leaves emerge, edit the panel-specifications table and note in the slice log.
4. Mark deferred items with 🚫 + one-line reason; preserve for audit.

Each commit must pass:
- **A-grade-or-it-doesn't-ship** — `cargo xtask grade` clean across affected crates (cf-viewer for the refactor commit; cf-scan-prep for new-crate commits).
- **Eyes-on-pixels** — for UI commits, user inspects the rendered Bevy window before declaring the commit complete. Per the auto-memory rule, this is user-side; Claude relays screenshots.
- **Ask-before-commit** — standing rule.

When the MVP ships (commit #13), update the [target deliverable](#target-deliverable) section with the actual scan asset details (what scanner / what shape / what cast outcome) once iter-1 produces them.

Archive trigger: move to `docs/archive/` after iter-1 cast is in hand and v2 prep work begins on a new spec.

### Slice ship log

- **2026-05-12** — Commit #1 (design spec) shipped (`119ac2f2`). 394-line initial spec, 7 panels, ~12 commits estimated.
- **2026-05-12** — Cap-feature addendum to commit #1 (`b5cd83cd`). Adds **Cap open boundaries** as panel #5 of 8, makes the spec watertight-by-construction (required for cf-cast SDF queries), bumps implementation arc to 13 commits / ~19-31 hours. Auto-detect + per-loop approve + manual override; rejected line-drawing alternative for geometric-ambiguity reasons.
- **2026-05-12** — Edge-case review pass folds 7 spec updates from a workflow-walkthrough cold-read (`2666fc0a`). Adds: STL units handling (`--stl-units mm\|m\|inch` flag, default mm, vertex × 0.001 -> meters, displayed in Scan Info); clip = true plane intersection (~150 LOC, rejects vertex-drop alt for cap-quality reasons); cap state invalidation on transform change (loop list dims with re-Scan overlay); cap normal orientation (mesh-side-of-plane heuristic + boundary-edge-adjacency fallback); Bevy `Transform`-component live transforms (commit #5/#6 perf for 100k-face scans); save atomicity (`.tmp` + atomic rename); FS error pattern (red status-bar, no panic). Bumps implementation arc to ~21-34 hours (commits #3 +0.5 hr, #7 +1 hr, #11 +0.5 hr). Risks expanded with edge-case enumeration (Transform-component rationale, units-mismatch user-error pattern, true-intersection plane-parallel triangle handling, FS path edge cases).
- **2026-05-12** — Final-pass cold-read polish (`272c98b9`). 5 drift items + 4 minor wording fixes from reading the spec end-to-end with fresh eyes: stale "commit #12" reference (→ #13); load-time validation parenthetical that contradicted strategic context (broken scans CAN produce valid SDFs → tightened to point at Cap panel + uncapped-loop warning); MVP scope validation list missing FS error + non-finite-rejection items; keyboard shortcut `1`/`2`/`3` claimed +X/+Y/+Z but Reorient panel only had 4 buttons (added `[Snap +X → +Z]` button + clarified shortcut mapping); arc total estimate 21-34 hr → 24-32 hr (matches per-commit sum); workflow step 4 wording (centroid-or-floor as alternatives); "14 mm offset" generic wording (→ "device outer-shell thickness from cf-cast CastSpec"); new keyboard shortcut `A` for Apply caps (most-impactful Cap-panel operation); commit #3 scope adds `OrbitCamera::UpAxis = +Z` cast-frame convention. No design changes; spec is implementation-ready.
- **2026-05-12** — Simplify (decimate) panel addendum (`8d35d563`). Adds **Simplify** as panel #2 of 9, between Scan Info and Reorient. Bumps implementation arc to 14 commits / ~28-37 hours. Triggered by iter-1 test fixture sock_over_capsule.stl (3.35M faces, 168 MB) revealing that ANY scan-driven cf-cast workflow without decimation drags 1M-10M+ face meshes through Bevy rendering + cf-cast `SignedDistanceField` build + slicer ingest, all of which choke at that scale. Decimation reduces to ~200k faces (cf-cast's 2 mm-cell SDF sampling density × surface-continuity headroom) with no quality loss for cast purposes; 30× downstream speedup typical. Algorithm: `meshopt-rs` (boundary-preserving quadric edge collapse, NOT `simplify_sloppy` — boundary preservation required so cap topology survives decimation). Rejected alternatives: inline Garland-Heckbert (~500-800 LOC, risk of subtle quadric-math bugs); vertex clustering (~100 LOC but doesn't preserve boundaries cleanly). Auto-suggest banner at load if scan > 500k faces. New `[simplify]` block in TOML schema captures algorithm + version + target/achieved/original face counts. Risks added: boundary-preservation verification (use non-sloppy variant + unit test pinning topology preservation); meshopt-rs as first new workspace dep with C-toolchain build-time dep; decimation-then-cap interaction (R² should be preserved, verify in commit #9).
- **2026-05-12** — Post-Simplify cold-read polish (`f3dfe2ca`). 4 drift items left over from the Simplify panel insertion: (a) ASCII mockup had duplicated viewport-overlay annotations (`· mouth extension` and `· ground grid`) at lines 118-119 from copy-paste during the panel insert — removed; (b) 4 stale commit numbers in Risks section from the 13→14 renumber (translucent overlay #7/#8 → #8/#9; mesh-repair semantics #12 → #13; boundary-loop extraction commit #8 → #9 (two refs); true-intersection edge cases #7 → #8); (c) 1 stale commit number in Validation rules (STL writer dual-mesh recon #11 → #12); (d) Save §9 `[Reset all]` ambiguity about simplify state — clarified that `[Reset all]` resets transforms + cap state but does NOT un-simplify (re-decimating on every reset would cost 4+ seconds for large scans; users use the Simplify panel's `[Reset to original]` button to un-simplify). No design changes; spec is implementation-ready.
- **2026-05-12** — STL units assumption strings: Unicode `→` (U+2192) swapped to ASCII `->` (`ecff1ca4`). Discovered during commit #4 eyes-on-pixels — egui's default `ProggyClean` font ships with Latin-1 + basic-punctuation coverage only, so `→` rendered as `□` (missing-glyph box) in the Scan Info panel's "STL units" row. `×` (U+00D7, Latin-1 Supplement) is in coverage and stays. Two strings amended in §1 (`mm (vertex × 0.001 -> meters)` + `inch (vertex × 0.0254 -> meters)`); slice-log line 509's descriptive mention also updated to match. The cf-scan-prep code's `StlUnits::panel_label` strings + the 3 spec-string pin tests + the `scan_info_from_loaded_populates_all_fields` test's assertion update in lockstep. No design changes; the spec's other `→` arrows (section-header transition arrows + Reorient panel snap-button names like `[Snap +Y → +Z]`) remain Unicode since they're documentation-level (rendered in markdown viewers, not egui) — only the literal egui-rendered strings get ASCII'd.
- **2026-05-12** — Commit #5 (Simplify panel + meshopt-rs integration) ships (`4f4f8952` + `9cc08aa5` docstring polish + `eb594cdc` SIGABRT guard + `317568df` cold-read polish). Two spec amendments folded in: (a) §1 Scan Info wording "Loaded at startup, never changes" → "Static fields (file path, STL units, raw AABB) loaded at startup; vertex/face counts update when the Simplify panel applies a decimation pass (commit #5)" — the original "never changes" wording predated the Simplify panel's design and is now factually wrong since `[Apply simplify]` mutates the displayed vertex/face counts; (b) §2 Simplify example string `"Reduced 3.35M → 198,432 faces in 4.2s"` → `"Reduced 3.35M -> 198,432 faces in 4.2s"` (same egui ProggyClean `→`-glyph constraint as commit #4's STL units fix; the live status bar emits ASCII `->`). New workspace dep `meshopt = "0.6.2"` (boundary-preserving quadric edge collapse via FFI to `meshoptimizer`); new cf-scan-prep dep `mesh-repair` (`weld_vertices` pre-simplify since STL load produces 3N unshared vertices that meshopt can't see as collapsible edges). `SimplifyOptions::LockBorder` flag pins open-boundary loop vertices in place — load-bearing for downstream Cap detection at commit #9 seeing the same loop topology after decimation. SIGABRT followup (eyes-on-pixels iter-1): drag slider above current face count + Apply triggered `meshopt_simplifyEdge` assertion (`target_index_count <= index_count`) — added defensive guards in `simplify_mesh` + `handle_simplify_actions` that surface a `[Reset to original]` workflow hint instead of aborting.
- **2026-05-12** — Commit #6 (Reorient panel) ships (`a2cead3f`). One spec amendment folded in: §3 Reorient + window-layout mockup + §Keyboard shortcuts snap-button labels swap Unicode `−` / `→` → ASCII `-` / `->`. The commit #4 slice-log entry predicted these "remain Unicode since they're documentation-level" — that was wrong; commit #6 makes the buttons real egui-rendered widgets that hit the same `ProggyClean` glyph constraint as the STL-units row. Narrative angle values in spec text (e.g. "−90° about X") stay Unicode since they're rendered by markdown viewers, not egui. New cf-scan-prep dep `nalgebra` (workspace pin) for `UnitQuaternion::from_euler_angles` derivation. Source-of-truth choice (per spec §Risks §"Quaternion-to-Euler-to-quaternion round-trip drift"): three slider degrees are the source of truth, quaternion is derived at read time — avoids the gimbal-lock round-trip corruption that a quaternion-first storage would accumulate. Bevy `Transform.rotation` updated per Update tick via `apply_reorient_to_transform`; physics-frame quaternion converted to bevy-frame via a scalar `(w, x, y, z) -> (w, -x, -z, -y)` formula (`physics_quat_to_bevy_for_plus_z`) derived from the matrix conjugation `R_bevy = M_pb · R_physics · M_pb^-1` where M_pb is the parity-flipping physics→bevy swap. Idempotent (per-entity equality check before writing) so steady-state is effectively zero work. 6 new tests pin Euler convention + axis-flip semantics. Implementation arc unchanged at 14 commits / ~28-37 hours; commit #6 actual ~3 hr inclusive of recon + spec amendments.
- **2026-05-12** — Always-on viewport overlays (`f0a65b0f`). Eyes-on-pixels on the iter-1 fixture revealed the Reorient panel was un-usable without visual references — the user could see the mesh tilt under slider drags but had no way to tell when it was axis-aligned. Spec §Visualization layer always-on overlays (axis gizmos + AABB wireframe + ground grid) had been spec'd from commit #1 but never implemented at commit #3 scaffold time. This commit lands two of three: axis gizmos at world origin (red/green/blue = cast X/Y/Z, with the post-`UpAxis::PlusZ`-swap on-screen directions: red right, blue up, green into screen — blue is the +Z demolding-direction reference) and a transformed AABB wireframe that follows the live `ReorientState` quaternion in lockstep with the mesh entity. Ground grid deferred to a future commit: the iter-1 fixture's natural origin sits inside the scan, so a `z=0` grid wouldn't be usefully visible until commit #7's Recenter panel translates the cavity floor to the origin. New `OverlayLengths` resource pre-computes the axis arrow length (`0.6 × scaled_diagonal`) + Bevy-frame bbox center/dims once at startup; new `draw_reference_overlays` Update system renders via Bevy's `Gizmos` API (`arrow` + `cube`). 1 new test pins the up-axis-swap-and-scale formula. No spec amendments needed (overlays were already spec'd; this is just implementation catching up).
- **2026-05-12** — Commit #7 (Recenter panel) ships (this commit). Three translation sliders + `[Center origin]` + `[Floor -> z=0]` + `[Reset translation]` buttons, mirroring Reorient's pattern. State stored in **physics-frame millimeters** (mm to match the AABB display convention; physics-frame because the saved STL bakes the translation into vertex positions directly at commit #12). Slider range per spec: `±2 × max(raw_aabb)` (mm) — scan-derived, pre-computed at startup in `build_overlay_lengths`. `[Center origin]` and `[Floor -> z=0]` are click-time computations: walk the 8 corners of the raw AABB through the current `ReorientState` quaternion, recompute the rotated bbox bounds in physics mm via new helper `rotated_aabb_physics_mm`, set slider values to the negation of (center / min.z). If the user changes rotation after clicking, the centering becomes stale — they re-click to re-center (spec confirms these are buttons, not continuous behaviors). The button label "Floor -> z=0" uses ASCII `->` matching the commit #4 / #6 glyph-coverage pattern. New `apply_recenter_to_transform` Update system mirrors `apply_reorient_to_transform` — writes `Transform.translation` per tick, idempotent equality-check, automatic re-apply after `handle_simplify_actions` respawn. `draw_reference_overlays` updated to compose translation + rotation in the wireframe's transform so the wireframe stays glued to the mesh through both Reorient and Recenter edits. 4 new tests pin RecenterState default, the physics→bevy translation projection (swap + scale), the `rotated_aabb_physics_mm` identity case, and the `rotated_aabb_physics_mm` 90°-about-X axis-extent swap. `OverlayLengths` gained two fields (`raw_aabb_m` cached for click-handler math; `recenter_slider_range_mm` for the slider extent); the existing overlay test extended to pin both. cf-scan-prep tests now 34; xtask grade A. No spec amendments. Implementation arc unchanged at 14 commits / ~28-37 hours; commit #7 actual ~1.5 hr inclusive of design preview + spec slice-log writeup.
