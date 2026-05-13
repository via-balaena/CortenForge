# Engineering Suite Design — `tools/cf-device-design/`

**Status**: design draft, not yet implemented (2026-05-13 EOD).

The engineering suite is the design + testing surface for a layered
silicone device. User-described as "probably the hardest part to
build, but the most important part — this IS the meat of the
program. The casting is just the casting for the layers we decide
on in here."

This doc captures the v1 MVP scope distilled from 4 rounds of
requirements gathering on 2026-05-13. Code-side implementation is a
multi-session arc; this doc is the design surface to align on before
the first commit lands.

## Pipeline placement

```
raw scan
   │
   ▼
cf-scan-prep            (clean STL + .prep.toml; SHIPPED on dev)
   │
   ▼
cf-device-design        (THIS DOC — engineering suite)
   │
   ├──► <scan>.design.toml + <scan>.cleaned.stl
   ▼
cf-cast-cli             (mold pieces + plugs + procedure.md;
                         SCAFFOLDED on dev. Will be re-pointed to
                         consume design.toml instead of auto-deriving
                         layer geometry from the scan.)
   │
   ▼
physical workshop cast
```

## Sanitization

Per [[project_layered_silicone_device]] and
[[project_layered_silicone_device_engineering_suite]] in
auto-memory: all tracked files (this doc, code, commit messages,
crate names, test names) refer to the device as a "layered silicone
device" or "layered elastomeric device" with "scanned reference
geometry" inputs. No anatomical or product-category language in
repo-visible content. Reviewers reading the source should see a
generic mechanical design tool for layered compliant payloads with
scan-derived inner cavities.

## Architectural foundation

The suite composes existing CortenForge primitives — does NOT
invent new geometric primitives. The heavy lifting:

- **`cf_design::Solid`** — typed SDF expression tree. Outer
  envelope, cavity, layer bodies, features all live here.
  Compose via `Solid::union / subtract / intersect`.
- **`cf_design::Solid::pipe(centerline, radius)`** — constant-
  radius pipe along a polyline. The curve-following outer capsule
  IS this primitive.
- **`cf_design::Solid::from_sdf(mesh_sdf_signed_distance_field,
  aabb)`** — wraps a scan SDF into the typed kernel for boolean
  composition with primitives.
- **`mesh_sdf::SignedDistanceField`** — the scan SDF source. Loads
  from the cleaned STL exactly as `cf-cast-cli::scan` does.
- **`cf_cast::Ribbon`** — curve-following 2-piece mold split.
  Already handles up to 120° of cumulative tangent rotation; the
  user's scans top out around 30° so cf-cast handles all expected
  cases.
- **`cf_cast::CastSpec`** + **`export_molds_v2`** — once the
  engineering suite has produced a `(plug, layer_bodies)` pair,
  this is what cf-cast-cli's main pipeline consumes downstream.
- **`mesh_printability::validate_for_printing`** — F4 gate for
  print-volume + sliver-geometry checks during design-time
  validation.
- **`sim-soft`** — the FEM crate. Engineering-readout integration
  is its own slice within this arc (post-MVP geometric build-out).
- **`cf-viewer`** + **`cf-bevy-common`** — shared Bevy plumbing
  (orbit camera, scene helpers, `UpAxis::PlusZ`). Matches the
  cf-scan-prep precedent.

## v1 MVP scope (locked from requirements rounds)

### Inputs

- **Cleaned scan**: path to `<scan>.cleaned.stl` (from cf-scan-prep).
- **Prep TOML**: path to `<scan>.cleaned.prep.toml` (carries the
  centerline polyline; cf-cast-cli already parses this).

### Outer envelope

- **Curve-following capsule** along the scan centerline.
- **Constant radius** along the arc (one slider in the GUI).
  Taper / variable-radius is deferred to v2.
- Implementation: `Solid::pipe(scan_centerline, outer_radius)`.

### Inner cavity

- **User-picked insertable length** measured from the tip along
  the scan centerline (one slider, units: mm).
- The scan portion from arc-length 0 to that distance becomes
  the cavity former. The remaining scan portion is not
  represented in the device.
- **Compression-fit inset** for the cavity surface (one slider,
  units: mm, typically negative). Cavity surface = scan offset
  INWARD by `inset_m`. Silicone stretches over the inserted
  object → grip / compression / seal.
- Implementation: clip the scan SDF to the insertable arc range,
  apply the inset offset, subtract from the layer-0 body.

### Layers

- **Variable count: 1 – 6 layers**, innermost-first.
- Each layer carries:
  - `thickness_m`: silicone shell thickness above the previous
    boundary (the scan's compression-inset surface for layer 0;
    layer N-1's outer surface for N > 0).
  - `material`: one of the cf-cast cure anchors (ECOFLEX_00_30,
    DRAGON_SKIN_10A, etc.) or a custom (display_name +
    density_kg_m3) override.
  - `features`: array of per-layer features (see below).
- Defaults: 3 layers, 6 mm Ecoflex / 4 mm Dragon Skin 10A /
  4 mm Ecoflex (matches MEMORY.md's v1 stack).

### Features (per layer)

All four feature types were selected in requirements gathering;
they ship in priority order with the first one feature-complete
before the next slice begins.

#### 1. Inner-cavity surface texture (FIRST feature)

Procedural patterns parametrized in the design TOML. The texture
modifies the cavity SDF as an analytic perturbation; the cavity
remains an exact SDF for composition into the rest of the tree.

v1 pattern library:

- **Ring of ridges**: count, spacing (mm along centerline arc),
  height (mm normal to surface).
- **Dimple grid**: rows + columns (count along arc × count around
  circumference), radius (mm), depth (mm).
- **Spiral ridge**: pitch (mm per full rotation), height (mm).

Schema sketch:

```toml
[[layers.features.texture]]
kind = "ring_of_ridges"
count = 8
spacing_m = 0.005
height_m = 0.0015
```

User dials these via GUI sliders; live preview updates the cavity
surface.

#### 2. Holes / through-channels (SECOND feature)

CSG cylinders subtracted from one specific layer body. Each hole
is positioned by a click-to-place anchor (in the GUI) and dialed
by radius + axis-orientation + depth.

Schema sketch:

```toml
[[layers.features.hole]]
position_m = [0.020, 0.015, -0.030]  # world-frame
axis = [0.0, 0.0, 1.0]              # unit vector
radius_m = 0.002
depth_m = 0.012
```

#### 3. Embedded component pockets (THIRD feature)

Bounded cavities sized for off-the-shelf components (motors,
sensors). Geometry-only; electrical routing always deferred.

Schema sketch:

```toml
[[layers.features.pocket]]
shape = "cylinder"      # "cylinder" | "cuboid" | "capsule"
position_m = [...]
orientation_axis = [...]
dimensions_m = { radius = 0.004, height = 0.010 }
```

#### 4. Variable thickness within a layer (FOURTH feature)

One layer's thickness varies along the centerline arc (or
circumferentially). Probably via a 1D thickness curve along arc
length for v1; full 2D (arc × circumference) is v2.

Schema sketch:

```toml
[[layers.features.thickness_curve]]
control_points = [
    { arc_t = 0.0, thickness_m = 0.006 },
    { arc_t = 0.5, thickness_m = 0.004 },
    { arc_t = 1.0, thickness_m = 0.006 },
]
interpolation = "linear"
```

### Engineering readouts (live during design)

Three readout categories. The first ships with the MVP; the others
follow as named slices.

#### A. Geometric validations (SHIPS WITH MVP)

Cheap, every-tick computation:

- **Per-layer pour mass** in grams, computed via
  `cf_cast::compute_pour_volumes` against the in-flight design.
- **Mass budget gate**: each layer's pour mass < the configured
  budget (`DEFAULT_MASS_BUDGET_KG` = 2 lb). Red if exceeded.
- **F4 printability per piece**: post-export check on the mold
  piece STLs (run on-demand via a `[Validate F4]` button, not
  every tick — F4 needs marching cubes).
- **Mold-piece bounding box vs FDM print volume**: yellow/red if
  any piece exceeds `PrinterConfig::fdm_default` extents.

#### B. Visual fit / clearance check (SLICE 2 of engineering readouts)

Pure geometry, no FEM:

- Heatmap on the cavity surface showing the local distance from
  cavity to outer envelope. Red where wall thickness drops below
  the minimum (per cure-anchor recommendation, e.g. 3 mm for
  Ecoflex 00-30).
- Per-segment-along-centerline summary readout (min wall, max
  wall, average).
- Surfaces the issue from "30° scan + small outer radius" before
  the user commits to a cast.

#### C. sim-soft FEM preview (SLICE 3, biggest)

Run a small FEM solve on the cured-silicone body with a rigid
scan-shaped indenter pushing through the cavity. Surface:

- Stretch heatmap on the silicone shell (where it's pulled
  tightest).
- Stress concentration peaks.
- Force-displacement curve from the insertion sweep.

Requires the sim-soft FEM bridge to be wired into this tool —
substantial cross-crate plumbing. Out of v1; the slice plan
below stages it.

### Output: `<scan>.design.toml`

A single file the engineering suite reads and writes, carrying:

- Scan reference (paths to cleaned STL + prep TOML).
- Outer envelope parameters (radius).
- Cavity parameters (insertable length, compression inset).
- Layer array (thickness + material + features per layer).
- Cast spec defaults (mesh cell size, mass budget, output dir,
  printer config) — same defaults as cf-cast-cli's CastDefaults.
- Provenance (tool version, generated_at timestamp).

cf-cast-cli is then extended to accept either a `cast.toml`
(current auto-derive path, for synthetic fixtures and
first-principles validation) OR a `design.toml` (engineering-
suite output). Same downstream pipeline; different upstream
authoring.

## GUI layout

Same window-layout pattern as cf-scan-prep:

```
+-----------------------------------------------------+--------------------+
|                                                     |  Scan Info         |
|                                                     |    file, vertex... |
|                                                     +--------------------+
|                                                     |  Outer Envelope    |
|                                                     |    radius (mm)     |
|                  3D viewport                        +--------------------+
|                                                     |  Cavity            |
|   - cleaned scan mesh (wireframe + flat shade)      |    length (mm)     |
|   - outer envelope wireframe (live)                 |    inset (mm)      |
|   - cavity surface (live)                           +--------------------+
|   - per-layer outer surfaces (live, transparent)    |  Layers            |
|   - feature overlays (texture, holes, pockets)      |    [+ Add Layer]   |
|   - validation heatmap toggle                       |    Layer 0: 6 mm   |
|                                                     |    Layer 1: 4 mm   |
|                                                     |    Layer 2: 4 mm   |
|                                                     +--------------------+
|                                                     |  Validations       |
|                                                     |    mass: 451 g ✓   |
|                                                     |    budget: 907 g   |
|                                                     |    F4: [Validate]  |
|                                                     +--------------------+
|                                                     |  Save              |
|                                                     |    <scan>.design.toml
+-----------------------------------------------------+--------------------+
|  Status bar — load / save / validation messages                          |
+--------------------------------------------------------------------------+
```

Live preview semantics: every panel parameter writes to the
in-memory design state; a Bevy Update system rebuilds + re-meshes
the affected solids and updates the viewport. Marching cubes
runs at a coarse cell size for live preview (e.g. 5 mm) and a
finer cell size on `[Validate F4]` / `[Save]`.

## Implementation slice plan

The arc breaks into ~14 commits, ~30–40 hours active across
multiple sessions. Each slice ships with per-leaf
`cargo test --release` + clippy + xtask grade A.

| # | Commit | Scope | Est |
|---|--------|-------|----:|
| 1 | `docs(device-design): design spec` | This document | shipped |
| 2 | `feat(tools): cf-device-design crate scaffold + STL/prep load` | New `tools/cf-device-design/` workspace member. Loads `<scan>.cleaned.stl` + `<scan>.cleaned.prep.toml` from CLI args. Blank Bevy app renders scan mesh + centerline overlay. | 2–3 hr |
| 3 | `feat(device-design): Outer Envelope panel` | Outer capsule along centerline at constant radius; slider drives `Solid::pipe(centerline, r)`. Live wireframe preview. | 2 hr |
| 4 | `feat(device-design): Cavity panel + compression-fit inset` | Insertable-length slider + cavity-inset slider. Computes the cavity former: scan SDF clipped to the insertable arc range, offset inward by `inset_m`. Live surface preview. | 3–4 hr |
| 5 | `feat(device-design): Layers panel (variable 1–6 layers)` | Add / remove / reorder layers; per-layer thickness + material dropdown. Live per-layer outer-surface preview. Default state = 3-layer Ecoflex / Dragon Skin / Ecoflex. | 3–4 hr |
| 6 | `feat(device-design): geometric validations live readout` | Per-layer pour mass (via `compute_pour_volumes`); mass-budget red/yellow/green; mold-piece-AABB-vs-FDM-print-volume check. Sub-second update on every parameter change. | 2 hr |
| 7 | `feat(device-design): inner-cavity surface texture (procedural)` | Texture pattern library: ring-of-ridges, dimple grid, spiral ridge. SDF perturbation on the cavity surface. Live preview. | 4–6 hr |
| 8 | `feat(device-design): Save / Export panel + design.toml schema` | Atomic write to `<scan>.design.toml`. Schema = serde structs mirroring this doc's spec. Pre-save validation. | 2 hr |
| 9 | `feat(cf-cast-cli): accept design.toml input` | cf-cast-cli extended to consume `<scan>.design.toml` (engineering-suite output) in addition to the current `cast.toml` (auto-derive default). Geometry construction reads from the design spec instead of auto-deriving from scan AABB / centerline. | 3 hr |
| 10 | `feat(device-design): holes / through-channels feature` | Click-to-place hole anchors in the viewport; per-hole radius + axis + depth sliders. CSG cylinder subtractions per-layer. | 4 hr |
| 11 | `feat(device-design): embedded component pockets feature` | Bounded cavity placement (cylinder / cuboid / capsule). Same click-to-place pattern. | 3–4 hr |
| 12 | `feat(device-design): variable thickness curve per layer` | 1D thickness curve along centerline arc; control-point editor. Modifies a layer's outer surface as a per-arc-fraction radius offset. | 4–5 hr |
| 13 | `feat(device-design): visual fit / clearance heatmap` | Per-cavity-vertex wall-thickness colormap. Min-wall threshold red. Live (coarse) + on-demand (fine). | 3–4 hr |
| 14 | `feat(device-design): sim-soft FEM preview` | Insertion-sweep FEM solve via sim-soft. Stretch + stress readouts. Force-displacement curve. Biggest single slice. | 8–12 hr |

Commits 1–9 = MVP that produces a workshop-iter-1-ready design.
Commits 10–13 = feature build-out (deferred until v1 round-trips
through a real cast). Commit 14 = the sim-engineering payoff.

## Open questions (deferred to implementation time)

These don't block this design doc but will need resolution during
the relevant slice:

- **Centerline sampling for outer envelope**: cf-scan-prep's
  centerline polyline has ~30 points; `Solid::pipe` may need
  smoother input for clean preview. Resample / spline-fit during
  load?
- **Cavity insertable-length clipping**: easiest implementation is
  to clip the scan SDF with a plane perpendicular to the centerline
  at the insertable-length arc position. Edge case: centerline
  tangent at that arc might not be axis-aligned → need a tangent-
  aligned clip plane. Slice 4 detail.
- **Texture-on-curved-surface UV math**: ring-of-ridges along a
  curved centerline needs centerline-arc-aware ridge spacing
  (not Cartesian Z spacing). Procedural-pattern formulas need
  to be in centerline-arc coordinates, not world. Slice 7 detail.
- **Bevy entity churn under live preview**: rebuilding mesh
  entities on every slider tick will lag. The Bevy Transform
  approach used by cf-scan-prep (mesh entity once, Transform
  per tick) doesn't apply here because the GEOMETRY changes,
  not just the transform. Need a debounced rebuild or partial
  update strategy.
- **Undo / redo**: not in v1 MVP. User edits the design.toml
  manually to back out if needed (matches cf-scan-prep posture).

## Out of scope for v1

- 3D rotation gizmo for outer envelope orientation (centerline
  is already in cast frame post-cf-scan-prep; no orientation
  needed).
- Multi-device-per-file library (one design.toml per device).
- Tapered or variable-radius outer envelope (constant only for
  v1).
- Hand-sketched textures or imported height-map textures
  (procedural-only for v1).
- Per-layer geometry that differs from the centerline-aligned
  shell (e.g. asymmetric layers, partial layers).
- Material gradients within a layer.
- 3+ mold pieces (Ribbon stays at 2 pieces for v1).
- Manufacturing-time process simulation (curing dynamics, mold
  release, etc.).

## See also

- [[project_layered_silicone_device]] — base sanitization +
  v1 stack memo.
- [[project_layered_silicone_device_engineering_suite]] —
  banked requirements + architectural correction memo for
  cf-cast-cli's Option A.
- [[project_scan_to_cast_bridge_design]] — cf-cast-cli
  scaffolding decisions (will need a follow-up entry noting
  the design.toml input mode added in slice 9).
- [[project_v2_1_cf_cast_arc_complete]] — cf-cast v2.1
  architecture (Ribbon, per-layer plugs, detachable shells)
  this suite produces specs for.
- `docs/SCAN_PREP_DESIGN.md` — cf-scan-prep design spec
  (precedent for tool delivery + GUI layout).
- `docs/CURVE_FOLLOWING_DESIGN.md` — cf-cast v2 curve-following
  architecture.
