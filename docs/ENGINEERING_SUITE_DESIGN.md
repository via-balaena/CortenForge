# Engineering Suite Design — `tools/cf-device-design/`

> ⚠️ **STALE — pre-implementation draft. Trust the code, not this doc.**
> This document was written before `tools/cf-device-design/` was built
> (2026-05-13) and the implementation diverged hard from it mid-arc —
> there is no "outer envelope" concept (it was absorbed into the layer
> stack), surfaces are solid meshes (not wireframes), every layer is
> user-dialed, and the slice ladder was reprioritized (FEM insertion
> sim pulled forward). The as-built architecture lives in the code's
> module docstrings. For the **insertion-sim subsystem** specifically
> (slices 7.0 → 7.3b.1, as-built + open recon question), see
> [`INSERTION_SIM_STATE.md`](INSERTION_SIM_STATE.md). This doc is
> retained only for the original requirements-gathering record.

**Status**: superseded by the as-built code (slices 2–6.5 shipped).
Original status at authoring: design draft, not yet implemented
(2026-05-13 EOD).

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
- **(Optional) existing design TOML**: path to a previously-saved
  `<scan>.design.toml` to reopen + iterate on. When supplied via
  CLI flag (`--design <path>`) or `[File > Open]`, the suite
  pre-populates every panel from the file; when absent, panels
  start at defaults.

### Geometry semantics — the load-bearing piece

The product geometry composes three components: a **scan-derived
plug** (cavity former), a **user-designed outer envelope** (pipe
along centerline), and **silicone layer surfaces** that partition
the volume between them. This section makes each one explicit
because the layer surfaces are subtle.

#### Plug

The cavity former. Built from:

1. Load the scan SDF via `mesh_sdf::SignedDistanceField`, wrap
   with `Solid::from_sdf(scan_sdf, scan_aabb)`.
2. Clip to the insertable arc range — intersect with a half-space
   whose plane is centered at the centerline point at arc-length
   `insertable_length_m` from the tip, normal = the centerline
   tangent at that arc position. Everything past that plane (the
   non-insertable scan portion) is discarded.
3. Offset INWARD by `compression_inset_m` (typically negative,
   e.g. -0.0005 m = 0.5 mm undersized → silicone stretches over
   the inserted object).

The result is `spec.plug` for the downstream `cf_cast::CastSpec`.

#### Outer envelope

`Solid::pipe(scan_centerline, R_outer)` — uniform-radius capsule
following the centerline polyline. `R_outer` is dialed by one
slider.

#### Layer surfaces (load-bearing convention)

A multi-layer cast has `N` layers (innermost-first). Each layer
needs a `body` Solid for `cf_cast::CastSpec` — the cumulative
SOLID outer-surface positive after that pour. The convention this
suite uses:

- **Layers 0 to N-2 (inner + middle)**: scan-shaped surfaces
  offset outward from the plug.
  - `layer[i].body = plug.offset(sum(t[0..=i]))`
  - Each inner layer has a uniform-by-offset thickness `t[i]` —
    e.g. layer 0 is `t[0]` thick of silicone uniformly around the
    plug surface.

- **Layer N-1 (outermost)**: equals the outer envelope.
  - `layer[N-1].body = Solid::pipe(centerline, R_outer)`
  - The outer layer's thickness is NOT user-dialed; it's the
    variable distance from `plug.offset(sum t[0..=N-2])` to the
    outer envelope. The suite's validations panel shows the
    actual thickness range so the user can sanity-check it.
  - Schema convention: layer N-1 carries `material` only — no
    `thickness_m` field. Saving / loading is symmetric.

This convention matches how the silicone actually gets cast in
the workshop:

- Cast layer 0 inside a `plug.offset(t[0])` mold → uniform `t[0]`
  shell over the plug.
- Cast layer 1 inside `plug.offset(t[0]+t[1])` mold → another
  uniform shell over the previous.
- ...
- Cast layer N-1 inside the outer-envelope mold → fills the
  remaining (variable) gap from `plug.offset(sum t[0..=N-2])` out
  to the pipe.

For the single-layer case (N=1), there are no inner layers; the
sole layer's body = outer envelope, and the silicone fills
everything from plug to envelope. Edge case is well-defined.

The CastSpec `(plug, layer_bodies)` pair this produces flows into
`cf_cast::CastSpec::export_molds_v2` exactly as for any other
geometry.

### Outer envelope (UI)

- **Constant radius** along the arc (one slider in the GUI).
  Taper / variable-radius is deferred to v2.
- Range typically `R_plug_max + safety` to `~3 × R_plug_max`. UI
  bounds derived from scan AABB at load time.

### Inner cavity (UI)

- **User-picked insertable length** measured from the tip along
  the scan centerline (one slider, units: mm).
  - Range: 0 to total centerline arc length, defaulted to the
    full arc length (insertable = entire scan).
- **Compression-fit inset** (one slider, units: mm; typically
  small negative for compression fit).
  - Range: ~-3 mm to +3 mm; default 0 mm (exact-fit cavity).
- Surface preview updates live as either slider moves.

### Layers (UI)

- **Variable count: 1 – 6 layers**, innermost-first.
- Per-layer controls:
  - `thickness_m` (inner + middle layers only — N-1 layer slot
    omits this).
  - `material` — dropdown of cf-cast cure anchors (ECOFLEX_00_30,
    DRAGON_SKIN_10A, etc.) plus a "Custom..." entry that opens a
    sub-form for display_name + density_kg_m3 override.
  - `features` — collapsing sub-section per layer with [+ Add]
    buttons for each feature type.
- `[+ Add Layer]` / `[- Remove Layer]` buttons; max 6.
- Defaults: 3 layers, 6 mm Ecoflex / 4 mm Dragon Skin 10A /
  Ecoflex (no thickness on the outermost slot — derived from
  R_outer and the inner thicknesses).

### Feature coordinate convention

All features that have a spatial location (holes, pockets,
textures) use **centerline-relative coordinates**, not world-frame
coordinates:

- `arc_t ∈ [0, 1]` — normalized arc-length along the centerline
  (0 = tip / cavity-entrance side, 1 = base / far end).
- `theta_rad ∈ [-π, π]` — angle around the centerline tangent at
  that arc position (0 = the ribbon's split-normal direction;
  positive = right-hand rule around tangent).
- `depth_from_outer_m` / `radial_offset_m` — distance from the
  outer envelope surface inward (or from the centerline outward,
  as appropriate per feature).

This means features stay attached to the device geometry when the
user changes `R_outer`, insertable length, or even the scan. The
suite resolves centerline-relative coordinates to world points at
mesh-build time using the same centerline-sampling helpers
`cf_cast::Ribbon` already exposes.

### Plug features (cavity)

#### Inner-cavity surface texture (FIRST feature to wire)

Procedural patterns parametrized in the design TOML. Texture
modifies the PLUG SDF as an analytic perturbation (the plug
surface IS the cavity-facing silicone surface, so texturing the
plug = texturing the cavity). Plug remains an exact SDF for
composition into the rest of the tree.

v1 pattern library:

- **Ring of ridges**: count, spacing (mm along centerline arc),
  height (mm — ridge protrusion outward from the plug surface).
  Ridges run around the centerline (perpendicular to tangent).
- **Dimple grid**: rows (count along arc) × columns (count
  around circumference), radius (mm), depth (mm — into the plug,
  so ridges-into-cavity).
- **Spiral ridge**: pitch (mm per full rotation around centerline),
  height (mm).

Each pattern's formula operates in centerline-arc coordinates
(`arc_t`, `theta_rad`) — NOT world Cartesian. A spiral on a
30°-curved centerline still wraps cleanly around the tangent at
every arc position.

Schema sketch:

```toml
[[cavity.textures]]
kind = "ring_of_ridges"
count = 8
spacing_m = 0.005        # arc-length spacing between ridges
height_m = 0.0015
# optional: arc_t_range = [0.0, 1.0] to limit ridges to a
# sub-range of the cavity. Defaults to the full insertable length.
```

User dials each pattern via GUI sliders; live preview updates the
plug surface (and therefore the inner cavity of every layer).

### Layer features

#### 1. Holes / through-channels

CSG cylinders subtracted from one specific layer's body. Each
hole is positioned by a click-to-place anchor on the layer's
outer surface in the GUI and stored in centerline-relative
coordinates.

Schema sketch:

```toml
[[layers.features.hole]]
arc_t = 0.55              # along centerline
theta_rad = 0.0           # around tangent (0 = +split_normal direction)
radius_m = 0.002
depth_m = 0.012           # inward from the layer's outer surface
# axis direction is computed at build time = -outward_normal at (arc_t, theta_rad)
```

#### 2. Embedded component pockets

Bounded cavities sized for off-the-shelf components (motors,
sensors). Geometry-only; electrical routing always deferred.

Schema sketch:

```toml
[[layers.features.pocket]]
shape = "cylinder"           # "cylinder" | "cuboid" | "capsule"
arc_t = 0.3                  # along centerline
theta_rad = 1.57             # around tangent (90° from +split_normal)
radial_offset_m = -0.004     # negative = below the layer's outer surface
dimensions_m = { radius = 0.004, height = 0.010 }
# orientation axis = tangent direction at (arc_t) by default; rotation override optional
```

#### 3. Variable thickness within a layer

A layer's nominal thickness varies along the centerline arc. v1
ships a 1D arc-only curve; per-circumference variation is v2.

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

When a layer has a thickness curve, its `thickness_m` field is
ignored in favor of the curve. Variable thickness only applies
to layers `0..=N-2` (the outermost is always the outer envelope).

### Engineering readouts (live during design)

Three readout categories. The first ships with the MVP; the others
follow as named slices.

#### A. Geometric validations (SHIPS WITH MVP)

Cheap, every-tick (or every-parameter-change-debounced) computation:

- **Per-layer pour mass** in grams, computed via
  `cf_cast::compute_pour_volumes` against the in-flight design.
- **Mass budget gate**: each layer's pour mass < the configured
  budget (`DEFAULT_MASS_BUDGET_KG` = 2 lb). Red if exceeded.
- **F4 printability per piece**: post-export check on the mold
  piece STLs (run on-demand via a `[Validate F4]` button, not
  every tick — F4 needs marching cubes). Should run async (spawn
  background task; UI shows "F4: running…" with a spinner). On
  completion, results land in the validations panel with green
  / yellow / red per piece.
- **Mold-piece bounding box vs FDM print volume**: yellow/red if
  any piece exceeds `PrinterConfig::fdm_default` extents. Cheap
  to compute live (just the outer envelope AABB + bounding margin).

#### Health checks (always-on)

In addition to the validations, the suite shows persistent
HEALTH WARNINGS for design states that produce invalid geometry:

- **No centerline in .prep.toml**: the cf-scan-prep save didn't
  include a `[centerline]` block (Cap panel wasn't applied
  before save). Red banner: "Reopen the scan in cf-scan-prep,
  apply [Cap]+ centerline computation, and re-save."
- **Insertable length exceeds total scan arc**: slider clamps;
  warn at the slider.
- **Layer thicknesses overspecified vs available radial gap**:
  if `sum(t[0..=N-2]) > (R_outer - max(plug_local_radius))`, the
  inner layers wouldn't fit inside the outer envelope. Yellow:
  "Layer 0 thickness reduced to fit"; red if even after reducing
  layer 0 to zero the stack doesn't fit.
- **Compression inset self-intersects plug**: large negative
  `inset_m` can shrink the plug to a self-intersecting blob
  (especially on a noisy scan). Red: "Compression inset too
  large; cavity geometry is invalid at arc positions [X, Y, Z]."
- **Outer-layer thickness goes negative**: if the inner layer
  stack reaches the outer envelope before layer N-1, layer N-1
  would have negative thickness. Red.

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
+-----------------------------------------------------+----------------------+
|                                                     |  Scan Info           |
|                                                     |    file, vertex...   |
|                                                     +----------------------+
|                                                     |  Outer Envelope      |
|                                                     |    radius (mm)       |
|                                                     +----------------------+
|                                                     |  Cavity              |
|                  3D viewport                        |    length (mm)       |
|                                                     |    inset (mm)        |
|   - cleaned scan mesh (wireframe + flat shade)      |    [+ Add Texture]   |
|   - outer envelope wireframe (live)                 |      Ring × 8 …      |
|   - cavity (plug) surface (live, with texture)      +----------------------+
|   - per-layer outer surfaces (live, transparent)    |  Layers              |
|   - feature overlays (holes, pockets)               |    [+ Add Layer]     |
|   - validation heatmap toggle                       |    ▼ Layer 0:        |
|   - centerline polyline overlay (cyan)              |       6 mm Ecoflex   |
|                                                     |       [+ Hole]       |
|                                                     |       [+ Pocket]     |
|                                                     |       [+ Thickness…] |
|                                                     |    ▶ Layer 1: 4 mm…  |
|                                                     |    ▶ Layer 2: outer  |
|                                                     +----------------------+
|                                                     |  Validations         |
|                                                     |    mass: 451 g ✓     |
|                                                     |    budget: 907 g     |
|                                                     |    F4: [Validate]    |
|                                                     +----------------------+
|                                                     |  Health              |
|                                                     |    ✓ centerline OK   |
|                                                     |    ✓ stack fits      |
|                                                     +----------------------+
|                                                     |  Save / Load         |
|                                                     |    [Save Design]     |
|                                                     |    [Open Design…]    |
+-----------------------------------------------------+----------------------+
|  Status bar — load / save / validation messages                            |
+----------------------------------------------------------------------------+
```

Live preview semantics: every panel parameter writes to the
in-memory design state; a Bevy Update system rebuilds + re-meshes
the affected solids and updates the viewport. Marching cubes
runs at a coarse cell size for live preview (e.g. 5 mm) and a
finer cell size on `[Validate F4]` / `[Save]`. Two performance
hedges:

- **Debounced rebuild**: parameter changes set a "dirty" flag;
  the next frame rebuilds only the affected solid(s) — not the
  entire mesh. Slider drags coalesce into one rebuild per ~100 ms.
- **Per-solid mesh entities**: plug + each layer + each feature
  overlay gets its own Bevy entity, so only the changed entities
  need to re-mesh on any given tick. Texture changes re-mesh the
  plug; a thickness slider on layer 2 re-meshes only layer 2.

CLI invocation:

```
cf-device-design <scan>.cleaned.stl              # fresh design
cf-device-design <scan>.cleaned.stl --design <existing>.design.toml
```

The first form starts with default panel values. The second form
loads + pre-populates from the saved design and lets the user
iterate. `[Open Design…]` in the GUI is the in-session
equivalent.

## Implementation slice plan

The arc breaks into ~14 commits, ~30–40 hours active across
multiple sessions. Each slice ships with per-leaf
`cargo test --release` + clippy + xtask grade A.

| # | Commit | Scope | Est |
|---|--------|-------|----:|
| 1 | `docs(device-design): design spec` | This document | shipped |
| 2 | `feat(tools): cf-device-design crate scaffold + STL/prep load` | New `tools/cf-device-design/` workspace member. Loads `<scan>.cleaned.stl` + `<scan>.cleaned.prep.toml` from CLI args. Blank Bevy app renders scan mesh + centerline overlay. | 2–3 hr |
| 3 | `feat(device-design): Outer Envelope panel` | Outer capsule along centerline at constant radius; slider drives `Solid::pipe(centerline, r)`. Live wireframe preview. | 2 hr |
| 4 | `feat(device-design): Cavity panel + plug construction` | Insertable-length slider + cavity-inset slider + health-check (centerline present? length in range? inset non-self-intersecting?). Constructs the plug: scan SDF clipped to the insertable arc range via tangent-perpendicular plane, offset inward by `inset_m`. Live plug-surface preview. | 4–5 hr |
| 5 | `feat(device-design): Layers panel (variable 1–6 layers)` | Add / remove layers; per-layer thickness + material dropdown. Per-section convention: layers 0..=N-2 are user-thickness'd (plug-offset); layer N-1 has material only (its body = outer envelope). Live per-layer outer-surface preview. Default state = 3-layer 6/4 + outer Ecoflex / Dragon Skin / Ecoflex. | 3–4 hr |
| 6 | `feat(device-design): geometric validations + health checks` | Per-layer pour mass (via `compute_pour_volumes` on the live CastSpec); mass-budget red/yellow/green; mold-piece-AABB-vs-FDM-print-volume check; stack-fits + cavity-non-self-intersecting + outer-layer-thickness-non-negative health checks. Debounced live update. Async `[Validate F4]` button. | 3 hr |
| 7 | `feat(device-design): inner-cavity surface texture (procedural)` | Texture pattern library: ring-of-ridges, dimple grid, spiral ridge. SDF perturbation on the PLUG surface (in centerline-arc coords). Live preview. Multiple textures composable on one plug. | 4–6 hr |
| 8 | `feat(device-design): Save / Open + design.toml schema` | Atomic write to `<scan>.design.toml` + Open path (CLI flag + `[Open Design…]` button). Schema = serde structs mirroring this doc's spec, with serde-`deny_unknown_fields` for forward-compat catches. Pre-save validation. | 2–3 hr |
| 9 | `feat(cf-cast-cli): accept design.toml input + layer-body convention` | cf-cast-cli extended to consume `<scan>.design.toml` (engineering-suite output) in addition to `cast.toml` (auto-derive default). Implements this doc's layer-body convention: layers 0..=N-2 = `plug.offset(sum t[0..=i])`, layer N-1 = outer envelope. Outer envelope = `Solid::pipe(scan_centerline, R_outer)`. Cavity texture applied to plug. Per-layer features applied as CSG (holes / pockets / variable thickness curve). | 4–5 hr |
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
  load? Slice 3 detail.
- **Cavity insertable-length clip plane robustness**: at the
  endpoint of the insertable arc, the centerline tangent is
  computed from the two surrounding polyline points. If the
  centerline has noise near the cut point, the tangent direction
  jitters and the cut plane wobbles. May need to smooth the
  tangent estimate (e.g. average over a small arc window). Slice
  4 detail.
- **Live mass calc performance on a 3.3M-face scan SDF**:
  `compute_pour_volumes` does a Riemann-sum SDF integration. The
  scan SDF eval is a mesh-sdf distance query — slow on the iter-1
  fixture. Probably need a coarser SDF approximation (downsampled
  mesh) for the live readout, with the fine SDF reserved for
  on-demand validation. Slice 6 detail.
- **Bevy entity churn under live preview**: the doc's "debounced
  rebuild + per-solid entities" strategy is the planned approach;
  may need refinement after slice 3 surfaces the actual mesh-
  rebuild latency.
- **Per-piece mold split for scan-shape-to-pipe-shape transition
  at layer N-1**: cf-cast's Ribbon already handles this — the
  mold-piece composition `bounding ∖ layer_body ∩ ribbon_side`
  doesn't care whether `layer_body` is scan-shaped or pipe-
  shaped. But the printability F4 check might fire on thin walls
  near the transition (where the gap from layer N-2 outer to
  outer envelope narrows). Slice 9 + slice 13 detail.
- **Undo / redo**: not in v1 MVP. User edits the design.toml
  manually to back out if needed (matches cf-scan-prep posture).
- **Cavity clip "non-insertable scan portion"**: at slice 4, the
  scan portion past the insertable-length cut is discarded. But
  some products may want that portion present as solid-silicone
  (forming a base / handle). Out of v1 per Round 1 answer; bank
  as v2 candidate ([[project_layered_silicone_device_engineering_suite]]
  has the requirements rounds banked for follow-up).

## Out of scope for v1

- 3D rotation gizmo for outer envelope orientation (centerline
  is already in cast frame post-cf-scan-prep; no orientation
  needed).
- Multi-device-per-file library (one design.toml per device).
- Tapered or variable-radius outer envelope (constant only for
  v1).
- Hand-sketched textures or imported height-map textures
  (procedural-only for v1).
- **Arbitrary asymmetric layers** — partial-circumference layers,
  per-θ varying layer shape, layers that protrude beyond the outer
  envelope (e.g. flanges, ridges on one side only). The variable-
  thickness-curve feature (slice 12) covers 1D arc-length variation;
  per-circumference + arbitrary shape is v2 / v3.
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
