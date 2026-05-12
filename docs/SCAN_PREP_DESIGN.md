# Scan Prep Design — `cf-scan-prep` GUI Tool

**Status:** active (opened 2026-05-12)
**Predecessor docs:** [`CASTING_ROADMAP.md`](CASTING_ROADMAP.md), [`SIM_SOFT_ROADMAP.md`](SIM_SOFT_ROADMAP.md)
**Implementation track:** Stage 2.5 (between Track F.2 v1 MVP and Stage 3 physical cast iter-1)

---

## Strategic context

Track F.2's v1 MVP shipped a capsule-fallback example that demonstrates the cast pipeline end-to-end. The capsule was the [Q2 fallback](CASTING_ROADMAP.md#architectural-decisions); the **scan-derived primary** path was deferred to iter-1 prep, where a real scanned reference geometry drives the cavity shape.

Real scans don't drop into `cf_cast::CastSpec` directly. They need preprocessing decisions that the capsule fallback didn't surface:

- **Reorient** — scanner-frame ≠ cast-frame; the scan's local axes must be rotated so the demolding direction is `+z`
- **Recenter** — translate the scan so the cavity centroid sits at origin (offsets are then symmetric)
- **Trim** — drop scanner noise (floor, fixture, extraneous geometry) below a clip plane
- **Mouth handling** — extend `+z` headroom past the scan's natural top so the mold cup opens cleanly
- **Inspect / validate** — sanity-check size vs print volume, surface mesh-quality issues (holes, non-manifold edges, multi-shell topology)

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

---

## Workflow

End-to-end, scan to mold:

```
[scanner / photogrammetry / etc.]
       ↓ (raw STL)
[user pre-trims to single shell if needed; external tool — MeshLab / Blender]
       ↓ (single-shell STL)
[cf-scan-prep GUI]
   1. Load STL via CLI arg
   2. Inspect — vertex/face count, AABB, mesh-repair diagnostics
   3. Reorient — rotate so demolding direction = +z
   4. Recenter — translate cavity centroid to origin (or floor → z=0)
   5. Optionally clip — drop scanner-noise floor below z=clip_z
   6. Set mouth extension — +z headroom for cup opening
   7. Save — emits <stem>.cleaned.stl + <stem>.prep.toml
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

cf-scan-prep handles steps 1-7. Steps before (raw scan acquisition + multi-shell trim) and after (workshop) are user-domain.

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
│         · clip plane (translucent, opt)      │ ▼ Reorient       │
│         · mouth extension (translucent, opt) │   Roll  ──○──    │
│         · ground grid (subtle, z=0)          │   Pitch ──○──    │
│                                              │   Yaw   ──○──    │
│         (orbit camera; mouse-drag)           │   [Snap +Z up]   │
│                                              │   [Snap −Z up]   │
│                                              │   [Snap +Y → +Z] │
│                                              │   [Snap −Y → +Z] │
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
| Mouth extension overlay (translucent box from `transformed_aabb.z_max` extending `+z` by `mouth_z` mm) | OFF | `Mouth ext. → Preview` checkbox |
| Cleaned-result preview (semi-transparent body extruded by 14 mm offset) | OFF | Future Stage 2.6 — banked, not in MVP |

**Camera behavior:** orbit camera (cf-bevy-common `OrbitCamera`), framed on initial load to scan's transformed AABB. **Position preserved across transform tweaks** — re-framing on every slider change would be jarring. A `[Reframe]` button in the side panel lets the user manually re-center the view if needed.

---

## Panel specifications

### 1. Scan Info (read-only)

Loaded at startup, never changes:
- File path (from CLI arg)
- Vertex count, face count
- Raw AABB dimensions in mm (W × D × H)
- Estimated volume in cm³ (via `mesh-measure`)

### 2. Reorient

Rotation state stored internally as `UnitQuaternion<f64>`; Euler conversion happens at the slider boundary so gimbal-lock state corruption is avoided.

- Three sliders for **roll / pitch / yaw** (intrinsic XYZ Euler, displayed in degrees, range −180° to +180°)
- Four shortcut buttons for common scanner-frame → cast-frame rotations:
  - `[Snap +Z up]` — identity (default)
  - `[Snap −Z up]` — 180° flip about the X axis
  - `[Snap +Y → +Z]` — 90° about X (the "scanner had Y up" case)
  - `[Snap −Y → +Z]` — −90° about X
- `[Reset rotation]` button → identity quaternion
- Read-only quaternion display (4 components: `w`, `x`, `y`, `z`) for advanced users

### 3. Recenter

- Three sliders for **X / Y / Z translation** (in mm, range derived from scan AABB ±2× max dimension)
- `[Center origin]` button → translate by `−transformed_aabb.center()`
- `[Floor → z=0]` button → translate by `−transformed_aabb.min.z * Vec3::Z`. Useful default for cast prep (cavity floor sits at `z=0`)
- `[Reset translation]` button → zero

### 4. Clip floor (optional)

- Checkbox `Enabled` (default off)
- Z slider for plane height (range derived from transformed AABB; default at `transformed_aabb.min.z + 0.1×height`)
- Live readout: `"Drops X% of vertices below z=Y mm"` — sanity check before saving destroys mesh
- When enabled, the translucent disc visualization renders in the viewport

### 5. Mouth extension

- `+Z` slider in mm (range 0–50 mm, default 15 mm — matches the v1 capsule example's headroom)
- Checkbox `Preview` — when checked, translucent box renders from `transformed_aabb.max.z` extending upward by `+Z` mm
- The mouth extension is **NOT baked into the cleaned STL** — it's a cf-cast bounding-region hint passed through the TOML

### 6. Cleaned AABB (read-only)

Live-updates as user drags sliders:
- Width / depth / height after rotation + translation + clip (in mm)
- Inline warnings if dimensions exceed CLI-configured build volume (default `250 × 210 × 210 mm`):
  - Per-axis warning: `"⚠ exceeds build volume on Z axis (135.7 > 210 mm)"`
  - Multi-axis warning if multiple exceeded
- All warnings advisory; save proceeds regardless

### 7. Save

- Output directory display (default: parent of input scan)
- Filename preview (`<input_stem>.cleaned.stl` + `<input_stem>.prep.toml`)
- `[Save]` button — single click, overwrites without prompt; status bar flashes `"Saved at HH:MM:SS"` on success
- `[Reset all]` button — returns to load-time state (identity transform, clip off, mouth = 15 mm)

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

STL with rotation + translation + clip baked into vertex positions. Mouth extension is **NOT** baked (it's a bounding-region hint, not a mesh modification).

The cast example (`examples/cast/layered-silicone-device-v1-scan/`) consumes only the cleaned STL for geometry; optionally reads the TOML for the `mouth_extension.plus_z_m` value to size the bounding region.

---

## Validation rules

**At load:**
- Run `mesh-repair` diagnostics on the loaded scan (closed manifold? non-manifold edges? holes? disconnected shells?)
- Display issue count in status bar (`"6 issues: 4 holes, 2 non-manifold edges"`)
- Allow user to continue — diagnostic is **informational, not blocking** (broken scans CAN produce valid SDFs if the holes are small)

**Live (per-frame):**
- "Drops X% of vertices below z=Y mm" under clip plane control (updates on slider change)
- Cleaned-AABB dimensions update on every slider change

**At save:**
- If cleaned AABB exceeds CLI-configured build volume on any axis → red warning in side panel
- If clip drops `>50%` of vertices → yellow warning (`"clip is removing most of the mesh — verify"`)
- Both warnings are advisory; save proceeds regardless (user might intentionally want a partial clip for testing)

---

## Keyboard shortcuts

| Key | Action |
|-----|--------|
| `Esc` | Quit |
| `Ctrl+S` | Save |
| `R` | Reset all transforms |
| `C` | Center origin |
| `F` | Floor → z=0 |
| `1` / `2` / `3` | Snap +X / +Y / +Z up |
| Mouse drag | Orbit camera |
| Scroll | Zoom |

---

## MVP scope vs banked items

**In MVP:**
- All 7 panels above
- All visualization elements above
- All keyboard shortcuts above
- Output: cleaned STL + TOML config
- Validation: load-time diagnostic + save-time AABB warning + clip-% live readout

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

12-commit plan, ~15-25 hours active across multiple sessions:

| # | Commit | Scope | Est |
|---|--------|-------|----:|
| 1 | `docs(scan-prep): design spec` | This document | ~30 min |
| 2 | `refactor(cf-viewer): lift RenderScale + setup helpers to lib` | Extract `RenderScale`, `compute_render_scale`, `scale_aabb`, `setup_camera_and_lighting`, `spawn_face_mesh` from main.rs to lib.rs; cf-view binary thinned; per-crate gates + xtask grade | ~2 hr + 25 min grader |
| 3 | `feat(tools): cf-scan-prep crate scaffold` | New `tools/cf-scan-prep/` workspace member, deps (cf-viewer + bevy + bevy_egui + mesh-io + mesh-measure + mesh-repair + nalgebra + serde + toml + anyhow + clap), blank Bevy app loading scan via CLI, rendering it via cf-viewer helpers | ~1 hr |
| 4 | `feat(scan-prep): Scan Info panel` | First egui panel, validates the wiring | ~1 hr |
| 5 | `feat(scan-prep): Reorient panel` | Sliders + snap buttons + quaternion readout; rotation applied to displayed mesh; transformed AABB updates live | ~3-4 hr |
| 6 | `feat(scan-prep): Recenter panel` | Translation sliders + center + floor → z=0 buttons | ~1-2 hr |
| 7 | `feat(scan-prep): Clip floor panel + plane viz` | Toggle, slider, translucent disc, drop-% live readout | ~2-3 hr |
| 8 | `feat(scan-prep): Mouth extension panel + preview overlay` | Slider, translucent box from transformed `z_max` | ~1-2 hr |
| 9 | `feat(scan-prep): Cleaned AABB + CLI build-volume thresholds` | Read-only panel, configurable via `--build-volume-mm W,D,H` flag | ~1-2 hr |
| 10 | `feat(scan-prep): Save panel + TOML + cleaned STL writer` | Save button, output naming, status-bar feedback | ~2-3 hr |
| 11 | `feat(scan-prep): mesh-repair diagnostics + keyboard shortcuts` | Load-time diagnostic in status bar; Esc/Ctrl+S/R/C/F/1-2-3 shortcuts | ~1 hr |
| 12 | `feat(examples): cast/layered-silicone-device-v1-scan` | New example consuming `<stem>.cleaned.stl` via `CF_CAST_SCAN_STL_PATH` env var; mirrors v1 capsule's structure | ~2 hr |

Plus the slice-ship-log update + auto-memory at the end — usually inline with the final ship commit.

---

## Risks + uncertainty

Worth surfacing now so failures aren't surprises:

- **Bevy 0.18 + bevy_egui interactions** — the workspace is on Bevy 0.18; bevy_egui 0.39.1 is the first release that pins `bevy ^0.18.0`. Should be stable but version churn possible.
- **Translucent overlay rendering** — Bevy's PBR pipeline doesn't always handle translucent materials cleanly with mesh occlusion. The clip-plane disc and mouth-extension box may need custom material setup or a separate render pass. Banked as risk; investigate during commit #7 / #8.
- **Quaternion-to-Euler-to-quaternion round-trip drift** — three slider sources of truth → one quaternion → display back as Euler can drift after multiple updates. Mitigation: the slider VALUES are the source of truth (not the displayed quaternion); each slider change re-derives the quaternion from the three current slider values. The quaternion display is computed from the quaternion-of-record at render time.
- **Mesh-repair diagnostic semantics** — `mesh-repair` returns a structured issue list, but mapping that to a human-readable status-bar string ("6 issues: 4 holes, 2 non-manifold edges") needs verification of which issue types it actually surfaces. Recon during commit #11.
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

When the MVP ships (commit #12), update the [target deliverable](#target-deliverable) section with the actual scan asset details (what scanner / what shape / what cast outcome) once iter-1 produces them.

Archive trigger: move to `docs/archive/` after iter-1 cast is in hand and v2 prep work begins on a new spec.

### Slice ship log

_(empty — first entry lands when commit #1 ships)_
