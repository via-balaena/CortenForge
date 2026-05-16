# Centerline Computation — Implementation Spec

**Status**: SPEC. Recon arc output, 2026-05-16. Implementation arc is the next
session.

**Source-of-truth recon brief**: `docs/CENTERLINE_RECON_BOOKMARK.md` —
problem statement, the five failed iterations, root-cause analysis, and the
seven research directions this spec selected from.

**Predecessor research reports** (Phase 1, in-session): MCF (Tagliasacchi
2012), Voronoi MAT, iterative per-slab, voxel skeletonization (Lee-Kashyap-
Chu), L1-medial skeleton (Huang 2013). Summaries inline below; full reports
in the session transcript.

---

## 1. Problem statement (one paragraph)

cf-scan-prep emits a 30-point centerline polyline into `.prep.toml`
`[centerline].points_m`. cf-device-design consumes it for per-vertex radial
direction (nearest-centerline-index lookup → radial = `vertex −
nearest_centerline_pt`). cf-cast consumes it for curve-following mold paths.
The current `compute_centerline_polyline` (AABB-center + spine_hint, straight
line) produces a **smooth** centerline but **off-center** for asymmetric
scans. Off-center centerline → asymmetric per-vertex radials → asymmetric
layer dome in cf-device-design even when the underlying scan dome is
geometrically symmetric. **Worked geometry** (re-derived 2026-05-16):
for a dome vertex on the ±x side with body shifted by Δ from the prescriptive
axis,

```
x_layer = (Δ ± r)(1 − t/m_±)        with m_± = sqrt((Δ ± r)² + h²)
```

Δ = 0 → symmetric layer dome. Δ ≠ 0 → asymmetric. This is the iter-1 cast
blocker.

---

## 2. Recommended approach — algorithmic centerline + wire-handle endpoint overrides

**One sentence**: the centerline is computed by a **density-independent
per-slab algorithm** (area-weighted polygon centroid from slab-mesh
intersection) — this is the **primary** path; the wire is rendered with
draggable handle balls at tip and floor, and the user can click a
handle to anchor that endpoint manually as an **override** when the
algorithm's auto-derived endpoint is wrong.

**Why algorithmic-primary, not manual-primary**: per the user's design
direction 2026-05-16, "the centers/points really should be
mathematically derived" — manual placement cannot achieve the precision
the algorithm can in the interior of the body, and forcing the user to
click every scan would be friction without payoff. The tip handle's
manual override is the safety net for "the certain body part that will
be being scanned here, [where] there a clear tip point to mark." The
floor handle override is also available but less commonly needed.

**XYZ-independence**: every coordinate is in scan-mesh world space
(post-load auto-PCA + auto-center). The algorithm uses **spine_hint
or the (auto_floor − auto_tip) line** as the body axis, NOT world Z.
Auto-PCA + auto-center reduce the residual tilt but don't zero it; the
design absorbs the residual naturally because the slab-perpendicular
direction comes from the body, not from world axes. See [[scans-axis-
orientation]].

### 2.1 Why this approach

- **Mathematically-derived centers > user-clicked centers in the body's
  interior.** Per-slab area-weighted polygon centroid is the
  geometrically correct center of each cross-section; a click on a
  curved surface can only get within `body_radius_at_z` of the true
  axis. The algorithm wins on precision in the interior; the user
  wins on judgment at the endpoints (where small cross-sections make
  the algorithm noise-dominated).
- **Area-weighted polygon centroid escapes the failure mode of the
  five prior iterations.** Failures #1, #2, #4 were all vertex-
  density-biased statistics (Kasa, vertex centroid). Failure #5 was
  extreme-point-biased (AABB midpoint). The area-weighted polygon
  centroid is **density-INDEPENDENT by construction** — it depends
  on the surface geometry (the slab-mesh intersection polygon), not
  on how the surface was sampled. This is the C-direction-done-right
  the Phase 1 research agent flagged as "only viable if paired with
  area-weighted polygon centroid statistic" (and which we hadn't
  tried).
- **Endpoint override absorbs the algorithm's known failure mode.**
  Dome tips have small / noise-dominated cross-sections — exactly
  where any per-slab statistic struggles. User-anchored tip handle
  is the safety net (per user direction: "for the certain body part
  that will be being scanned here, there a clear tip point to
  mark"). Floor handle override symmetric for edge cases (e.g., pre
  vs. post reconstruct state changes).
- **No manual spline / no mid-wire bend.** Per user direction 2026-
  05-16. Interior of the wire is algorithm-only; the user cannot
  bend the wire manually. G.s3 stretch goal from prior spec
  iteration is **dropped**.
- **XYZ-independent by construction.** Slab planes are perpendicular
  to the (spine_hint OR override-derived) axis direction, not world
  Z.
- **Bevy 0.18 ships `MeshPickingPlugin` built-in.** No new external
  dep for the handle UI.
- **Reuses `.prep.toml` `[centerline]` polyline format.** Downstream
  consumers (cf-device-design, cf-cast) unchanged.

### 2.2 Trade-offs

- **Slab-mesh intersection is more complex than vertex statistics.**
  ~150 LOC for plane-triangle intersection + polygon centroid vs.
  ~10 LOC for a vertex sum. Worth it because the statistic is
  density-independent — the simpler statistics are the ones that
  failed.
- **Algorithm result still feeds into downstream radial-direction
  compute.** If the algorithm produces a wiggly polyline (per-slab
  centroids deviate slab-to-slab), the cf-device-design nearest-
  centerline-index lookup will produce wiggly radials. *Mitigation*:
  smoothing pass on the polyline (Taubin / moving-average, ~30 LOC)
  before serialization. Same pattern as the centerline-trim polyline
  smoothing already in the codebase.
- **Dome-tip slabs have ill-defined centroids.** Cross-sections shrink
  to a point; polygon area → 0; centroid becomes noise-dominated or
  undefined. *Mitigation*: tip handle override + truncate the
  algorithm's polyline to skip degenerate slabs (last 2-3 slabs at
  each end); interpolate from the smoothing pass.
- **`MeshPickingPlugin` × `OrbitCameraPlugin` interaction.** Bevy
  picking might fire on click during a camera orbit drag.
  *Mitigation*: Bevy's `Pointer<Click>` already distinguishes click
  (no drag) from drag. Verify during implementation; gate picking
  on `armed_handle.is_some() OR ev.target() is HandleTag` if
  conflict.
- **Pre/post-reconstruct floor handle behavior.** When the user
  toggles Reconstruct, the floor geometry changes under the floor
  handle. The handle's world-coordinate anchor stays valid (still a
  3D point) but may no longer sit on the mesh surface. The
  ALGORITHM's auto-derived floor may also shift because the body
  extent changed. *Mitigation*: handle position is independent of
  mesh — ball renders at the stored position; algorithm re-runs on
  the current mesh state. User can re-drag if displeased.

### 2.3 UI flow

1. Cap step runs the **per-slab area-weighted polygon centroid
   algorithm** (§2.4) to produce the auto-derived centerline polyline.
2. The 3D viewport shows the polyline as a wire. Two handle balls
   render at the wire's endpoints (small spheres, ~3mm radius in world
   units, colored distinctly e.g. cyan tip / magenta floor for left/
   right disambiguation). Handles are visible always — they're how the
   user knows where the algorithm placed the endpoints.
3. Default state: handles render at the algorithm's `polyline[0]` and
   `polyline[n-1]`. No user action required; the algorithm-derived
   wire is the active centerline.
4. **Arming a handle (override mode)**: user clicks a handle ball. The
   handle visually highlights (brighter shade / scale up 1.2×). Now
   armed.
5. **Re-positioning the armed handle**: user clicks anywhere on the
   scan mesh. The armed handle's anchor moves to the click's hit
   position. **The algorithm re-runs with the anchored endpoint(s) as
   constraint(s)**. Wire updates. Handle disarms.
6. **Cancel arm**: user clicks the same handle again (toggle off) OR
   presses Escape OR clicks somewhere not on the scan mesh.
7. Existing "Centerline" panel section gains an "Endpoint overrides"
   sub-section (collapsed by default). Inside:
   - Per-handle status: `tip: auto` / `tip: user-anchored (...) mm`.
   - `[Apply overrides]` button — commits picked anchors into
     `CenterlineAxisState.applied_*`.
   - `[Reset tip]` / `[Reset floor]` buttons — reverts the
     corresponding handle to the algorithm's auto-derived default.
8. Save writes the algorithm's final polyline (with any overrides
   applied) into `.prep.toml` `[centerline].points_m`, plus per-handle
   anchor state into the `[centerline_axis]` block (see §4.2).

### 2.4 Wire computation algorithm — per-slab area-weighted polygon centroid

Given the input mesh, the `spine_hint`, the number of slices `N=30`,
and per-handle overrides `H_tip`, `H_floor: Option<Point3<f64>>`,
compute the polyline:

```text
INPUT:
  mesh: IndexedMesh             // cleaned scan, watertight, ~169k faces
  spine_hint: Vector3<f64>      // initial axis estimate (cap loop normal)
  n_slices: usize = 30
  H_tip: Option<Point3<f64>>    // user override or None
  H_floor: Option<Point3<f64>>  // user override or None

STEP 1 — Determine axis direction:
  If H_tip is Some AND H_floor is Some:
    axis_dir = (H_floor − H_tip).normalize()        // user-anchored axis
  Else:
    axis_dir = spine_hint.normalize()                // algorithm default

STEP 2 — Determine axis depth range:
  depths = mesh.vertices.map(|v| axis_dir.dot(&v.coords))
  d_min = depths.min()                               // body's min depth on axis
  d_max = depths.max()                               // body's max depth on axis
  If H_tip is Some:    d_tip = axis_dir.dot(H_tip.coords)
  Else:                d_tip = d_min                 // algorithm picks extremes
  If H_floor is Some:  d_floor = axis_dir.dot(H_floor.coords)
  Else:                d_floor = d_max

STEP 3 — Per-slab centroid for interior:
  For i in 0..n_slices:
    t = (i + 0.5) / n_slices                         // evenly-spaced
    d_i = d_tip + t * (d_floor − d_tip)              // depth at slab i
    plane_i = perpendicular to axis_dir at depth d_i
    polygon_i = intersect plane_i with mesh.faces    // ~150 LOC subroutine
    If polygon_i.area > MIN_SLAB_AREA:
      centroid_i = area_weighted_centroid(polygon_i) // ~30 LOC subroutine
    Else:
      centroid_i = null                              // degenerate slab (dome tip)

STEP 4 — Fill in degenerate slabs:
  For each null centroid_i: linearly interpolate from nearest non-null neighbors.
  Endpoint degenerate cases: use H_tip / H_floor if provided; else linearly
  extrapolate from interior centroids.

STEP 5 — Endpoint pinning:
  If H_tip is Some:   centroid_0 = H_tip
  If H_floor is Some: centroid_(n-1) = H_floor

STEP 6 — Smoothing:
  Apply iterated 3-tap moving average over interior points (existing
  smooth_polyline subroutine), endpoints pinned. 3-5 iterations.
  Smooths slab-to-slab noise without flattening overall curvature.

STEP 7 — Output:
  polyline = [centroid_0, centroid_1, ..., centroid_(n-1)]
  Each point in world coordinates (post auto-PCA + auto-center).
```

**Key subroutine**: `intersect_plane_with_mesh(plane, mesh) → Polygon`
walks the mesh faces, intersects each triangle with the plane, collects
intersection edges, sorts them into a closed polygon (assuming
watertight mesh produces a closed cross-section). ~100 LOC. Standard
algorithm; well-known edge cases (degenerate triangles, vertices on
plane, multiple closed loops for non-convex bodies — for body-part
scans expect single convex-ish loop per slab).

**Key subroutine**: `area_weighted_centroid(polygon) → Point3<f64>`.
For a 2D polygon in the slab plane: standard centroid formula
`(1 / 6A) * Σ (xᵢ + xᵢ₊₁) * (xᵢyᵢ₊₁ − xᵢ₊₁yᵢ)`. For our 3D polygon
in the slab plane, project to local 2D frame, compute, project back.
~30 LOC.

**Iteration?** Not in MVP. With the axis fixed at step 1 (from either
spine_hint or user anchors), one pass of per-slab centroids
approximates the body axis well for straight bodies. Smoothing in
step 6 absorbs slab-to-slab noise. If a curved body surfaces, add
iteration loop: re-orient slabs perpendicular to local polyline
tangent and re-fit; iterate to convergence. ~50 LOC, deferred until
needed.

### 2.5 Pseudocode

```rust
// State, slots into AppState alongside CenterlineTrimState / CapState.
#[derive(Resource, Default, Clone, Copy, Debug)]
struct CenterlineAxisState {
    // Currently armed handle (None = nothing armed; click-mesh
    // events are ignored).
    armed: Option<Handle>,
    // Per-handle user-anchored positions. None = use Cap-derived
    // default for that handle.
    anchor_tip:   Option<Point3<f64>>,
    anchor_floor: Option<Point3<f64>>,
    // Applied versions — what gets serialized at Save. Diverge from
    // anchor_* if user has dragged a handle but not yet [Apply axis].
    applied_tip:   Option<Point3<f64>>,
    applied_floor: Option<Point3<f64>>,
    // Action queue for [Apply axis] / [Reset tip] / [Reset floor].
    pending_action: Option<AxisAction>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
enum Handle { Tip, Floor }

enum AxisAction { Apply, ResetTip, ResetFloor }

/// Compute the 30-point polyline via per-slab area-weighted polygon
/// centroid (§2.4), with optional endpoint overrides.
///
/// **This REPLACES the iteration-5 `compute_centerline_polyline`** —
/// see §10 References for the migration path. The function name is
/// retained to minimize call-site churn; signature gains optional
/// override params.
fn compute_centerline_polyline(
    mesh: &IndexedMesh,
    spine_hint: Vector3<f64>,
    n_slices: usize,
    h_tip:   Option<Point3<f64>>,        // NEW — endpoint override
    h_floor: Option<Point3<f64>>,        // NEW — endpoint override
) -> Vec<Point3<f64>> {
    if mesh.vertices.is_empty() || n_slices == 0 { return Vec::new(); }

    // STEP 1: axis direction (handle overrides if both present)
    let axis_dir = match (h_tip, h_floor) {
        (Some(t), Some(f)) => {
            let d = f.coords - t.coords;
            if d.norm_squared() < f64::EPSILON { spine_hint.normalize() }
            else { d.normalize() }
        }
        _ => {
            if spine_hint.norm_squared() < f64::EPSILON { return Vec::new(); }
            spine_hint.normalize()
        }
    };

    // STEP 2: axis depth range (endpoints from overrides OR mesh extremes)
    let mesh_depths: Vec<f64> = mesh.vertices.iter()
        .map(|v| axis_dir.dot(&v.coords)).collect();
    let d_min = mesh_depths.iter().copied().fold(f64::INFINITY, f64::min);
    let d_max = mesh_depths.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    if d_max - d_min < f64::EPSILON { return Vec::new(); }
    let d_tip   = h_tip  .map_or(d_min, |p| axis_dir.dot(&p.coords));
    let d_floor = h_floor.map_or(d_max, |p| axis_dir.dot(&p.coords));

    // STEP 3: per-slab area-weighted polygon centroid
    let mut centroids: Vec<Option<Point3<f64>>> = Vec::with_capacity(n_slices);
    for i in 0..n_slices {
        let t = (i as f64 + 0.5) / (n_slices as f64);
        let depth_i = d_tip + t * (d_floor - d_tip);
        let plane = Plane { normal: axis_dir, depth: depth_i };
        let polygon = intersect_plane_with_mesh(&plane, mesh);        // ~100 LOC
        centroids.push(if polygon.area() > MIN_SLAB_AREA_M2 {
            Some(area_weighted_centroid(&polygon))                    // ~30 LOC
        } else {
            None    // degenerate (dome tip / floor truncation)
        });
    }

    // STEP 4: fill in degenerate slabs (linear interpolation between
    // nearest non-null neighbors)
    let mut polyline: Vec<Point3<f64>> = fill_null_by_interpolation(&centroids);

    // STEP 5: endpoint pinning (overrides if present)
    if let Some(t) = h_tip   { polyline[0]            = t; }
    if let Some(f) = h_floor { polyline[n_slices - 1] = f; }

    // STEP 6: smoothing (iterated 3-tap moving average, endpoints pinned).
    // Existing `smooth_polyline` subroutine, 3-5 iterations.
    smooth_polyline(&mut polyline, /*iters=*/ 5);

    polyline
}

// Subroutines (new):
fn intersect_plane_with_mesh(plane: &Plane, mesh: &IndexedMesh) -> Polygon { /* ~100 LOC */ }
fn area_weighted_centroid(polygon: &Polygon) -> Point3<f64> { /* ~30 LOC */ }
fn fill_null_by_interpolation(centroids: &[Option<Point3<f64>>]) -> Vec<Point3<f64>> { /* ~40 LOC */ }

const MIN_SLAB_AREA_M2: f64 = 1e-8;  // 0.01 mm² — below this slab is noise

/// Bevy click-handler — fires on Pointer<Click> events.
/// Disambiguates handle entities vs. scan-mesh entity.
fn handle_axis_click(
    mut events: EventReader<Pointer<Click>>,
    mut axis_state: ResMut<CenterlineAxisState>,
    handle_query: Query<&HandleTag>,    // marker on the two handle balls
    scan_mesh_query: Query<Entity, With<ScanMeshTag>>,
) {
    let scan_entity = match scan_mesh_query.single() {
        Ok(e) => e, Err(_) => return,
    };
    for ev in events.read() {
        // Case 1: clicked a handle ball → arm it (or toggle off).
        if let Ok(tag) = handle_query.get(ev.target()) {
            axis_state.armed = if axis_state.armed == Some(tag.0) {
                None  // toggle off
            } else {
                Some(tag.0)
            };
            continue;
        }
        // Case 2: clicked the scan mesh while a handle is armed → move it.
        if ev.target() == scan_entity {
            if let Some(handle) = axis_state.armed {
                let hit: Vec3 = ev.hit.position
                    .expect("MeshPickingPlugin always provides position");
                let p = Point3::new(hit.x as f64, hit.y as f64, hit.z as f64);
                match handle {
                    Handle::Tip   => axis_state.anchor_tip   = Some(p),
                    Handle::Floor => axis_state.anchor_floor = Some(p),
                }
                axis_state.armed = None;  // disarm after move
            }
            continue;
        }
        // Case 3: clicked elsewhere with handle armed → cancel arm.
        if axis_state.armed.is_some() {
            axis_state.armed = None;
        }
    }
}

/// UI panel — renders status + Apply/Reset buttons.
fn render_centerline_axis_section(
    ui: &mut egui::Ui,
    state: &mut CenterlineAxisState,
) {
    egui::CollapsingHeader::new("Adjust axis")
        .default_open(false)
        .show(ui, |ui| {
            // Per-handle status display
            for (label, handle, anchor, applied) in [
                ("tip",   Handle::Tip,   state.anchor_tip,   state.applied_tip),
                ("floor", Handle::Floor, state.anchor_floor, state.applied_floor),
            ] {
                ui.horizontal(|ui| {
                    let status = match anchor {
                        None => "auto",
                        Some(p) => &format!("user ({:.1}, {:.1}, {:.1}) mm",
                                            p.x * 1000.0, p.y * 1000.0, p.z * 1000.0),
                    };
                    let armed_str = if state.armed == Some(handle) { " [ARMED]" } else { "" };
                    ui.label(format!("{label}: {status}{armed_str}"));
                    if ui.button(format!("Reset {label}")).clicked() {
                        state.pending_action = Some(match handle {
                            Handle::Tip   => AxisAction::ResetTip,
                            Handle::Floor => AxisAction::ResetFloor,
                        });
                    }
                });
            }

            // Drift indicator
            let drifted = state.anchor_tip != state.applied_tip
                || state.anchor_floor != state.applied_floor;
            if drifted {
                ui.colored_label(
                    egui::Color32::from_rgb(240, 200, 80),
                    "⚠ Pending: handle moved but not applied — click [Apply axis]",
                );
            }
            if ui.add_enabled(drifted, egui::Button::new("Apply axis")).clicked() {
                state.pending_action = Some(AxisAction::Apply);
            }
            ui.small(
                "Click a handle ball in the viewport to arm it, then click \
                 the scan mesh to re-position. Click [Apply axis] to commit. \
                 Reset reverts a handle to the Cap-derived default.",
            );
        });
}

/// Centerline overlay update — runs each frame state changes.
/// Calls the algorithm with endpoint overrides applied.
fn update_centerline_overlay(
    axis_state: Res<CenterlineAxisState>,
    cap_state: Res<CapState>,
    /* gizmo / line renderer + handle ball positions */
) {
    let points = compute_centerline_polyline(
        &cap_state.cleaned_mesh,     // algorithm needs mesh (per-slab intersection)
        cap_state.spine_hint,
        30,
        axis_state.anchor_tip,       // None → algorithm picks d_min extreme
        axis_state.anchor_floor,     // None → algorithm picks d_max extreme
    );
    // Re-render wire from `points`. Re-position handle balls at
    // points[0] (tip) and points[n-1] (floor) — handles always
    // render at the actual endpoint positions, whether algorithm-
    // derived or user-anchored.
}
```

### 2.6 Bevy MeshPickingPlugin integration

One-time setup in `main()`:
```rust
App::new()
    .add_plugins(DefaultPlugins.set(WindowPlugin { /* existing */ }))
    .add_plugins(MeshPickingPlugin)                  // NEW
    .add_plugins(EguiPlugin::default())
    /* existing */
    .add_systems(Update, handle_axis_click)         // NEW
    /* existing */
```

Scan-mesh entity (in the existing mesh-update path):
```rust
commands.spawn((
    Mesh3d(scan_mesh_handle),
    /* existing materials, transforms */
    RayCastPickable,                                  // NEW
    ScanMeshTag,                                      // existing marker
));
```

Handle ball entities (spawned per-frame from `update_centerline_overlay`,
OR persistent with position updates per-frame):
```rust
commands.spawn((
    Mesh3d(ball_mesh_handle),       // small sphere mesh, ~3mm radius
    MeshMaterial3d(tip_material),   // distinct color
    Transform::from_translation(tip_world_pos),
    RayCastPickable,
    HandleTag(Handle::Tip),
));
// same for floor
```

The egui-captures-pointer guard (line 4842 of main.rs,
`AccumulatedMouseMotion`/`Scroll` zero-out) already prevents 3D-
viewport pointer events from firing when the cursor is over the egui
panel area.

### 2.7 Stretch goals (deferred unless iter-1 surfaces a real need)

- **G.s1 — Snap-to-cross-section-centroid on click.** When user clicks
  the mesh to anchor a handle, instead of recording the surface hit
  point directly, project the hit point onto a perpendicular slab,
  compute the area-weighted centroid of that slab's intersection
  polygon, and use THAT as the anchor. Turns a single click into an
  axis-centered anchor automatically. **Lower priority than in the
  previous spec iteration** because the algorithm already does this
  for the interior — the override only needs to be approximately
  correct since the algorithm constrains the rest. Cost: ~30 LOC
  (reuses the algorithm's slab-intersection + polygon-centroid
  subroutines already required for §2.4). Add if iter-1 shows raw
  click precision is insufficient.
- **G.s2 — Algorithm iteration for curved bodies.** Add an outer loop
  to §2.4 step 3: re-orient slabs perpendicular to the local polyline
  tangent (not the global axis) and re-fit; iterate to convergence
  (5-15 iters per the Direction C research). Handles curved limbs
  that the straight-axis MVP cannot. Cost: ~50 LOC. Defer until a
  curved-body fixture surfaces.
- **G.s3 — Drag gesture for handle re-position.** Replace "click
  handle, click mesh" with "mouse-down on handle, drag, mouse-up on
  mesh." Faster for power users. Bevy `Pointer<Drag>` events would
  drive this. Cost: ~50 LOC. Defer unless click-then-click feels
  slow in workshop use.

**Explicitly dropped from prior spec iteration**: manual mid-wire
bend control (Bézier / Catmull-Rom from a third handle). Per user
direction 2026-05-16: "i dont think we should do a manual spline.
manual will not be able to get it perfect."

---

## 3. Alternative algorithmic option — Direction E (L1-medial skeleton)

**When to revisit**: if the user requires no per-scan UI interaction (full
automation), OR if curved-body fixtures surface and a straight prescriptive
axis is insufficient.

### 3.1 Summary

Huang, Wu, Cohen-Or, Gong, Zhang, Chen, *L1-Medial Skeleton of Point
Clouds* (SIGGRAPH 2013). Iteratively project sample points to their local
L1-medians via a Weiszfeld-style fixed-point update with a repulsion term
between samples and a `1/local_density` weight on cloud points
(precisely targeting our root failure source). The σ-PCA-directionality
threshold gates the skeleton trace.

### 3.2 Why E over D (voxel skeletonization)

D and E tied at 15/20 in triage. E wins on:

- **LOC**: ~400 vs. ~1500.
- **Single-branch tubular case dodges the branch-extraction phase** that
  sank the Python port — we trace ONE branch from the σ-largest seed and
  ignore bridging logic entirely (cuts the gnarliest ~150 LOC down to ~60).
- **Point-cloud-native**: mesh hygiene issues don't matter.
- **`1/local_density` weight** directly antidotes the scanner-viewpoint
  asymmetry that broke five descriptive iterations.

D's case (over E) would be: if we need provable topology preservation
guarantees (Lee-Kashyap-Chu has them via the Euler LUT) or if the
voxel-quantization noise filter turns out to be necessary on a
particularly noisy scan. Defer to a later evaluation if it comes up.

### 3.3 Why E over A (MCF)

A is the gold-standard algorithm in the literature but costs 1500-2500
LOC + sparse Cholesky + half-edge remesh + closed-mesh requirement. Only
worth it if E proves insufficient on real scans AND the descriptive (vs.
prescriptive G) axis turns out to be load-bearing for the design.

### 3.4 E pseudocode (high level)

```rust
fn l1_medial_skeleton(
    cloud: &[Point3<f64>],        // ~85k from 169k-face mesh
    n_samples: usize,             // ~500-2000
    bbox_diag: f64,
) -> Vec<Point3<f64>> {
    let mut samples = farthest_point_sample(cloud, n_samples);
    let kd = kiddo::KdTree::<f64, 3>::from(cloud);
    let h0 = 2.0 * bbox_diag / (n_samples as f64).cbrt();
    let mut h = h0;
    let mu = 0.35;
    let n_outer_steps = 20;

    for _ in 0..n_outer_steps {
        for s in samples.iter_mut() {
            // Inner Weiszfeld L1-median update (eq. 4):
            //   x_i_new = (Σ_j α_ij q_j + μ σ_i Σ_i' β_ii' (x_i - x_i'))
            //             / (Σ_j α_ij + μ σ_i Σ_i' β_ii')
            // where α_ij = θ(||x_i - q_j||) / (||x_i - q_j|| · d_j),
            //       β_ii' = θ(||x_i - x_i'||) / ||x_i - x_i'||²,
            //       d_j = 1 + Σ θ(||q_j - q_k||)  (local density),
            //       σ_i from weighted PCA over h-neighborhood,
            //       θ(r) = exp(-r²/(h/2)²).
            // Iterate to fixed point (~5-10 inner iters).
            *s = weiszfeld_step(s, cloud, &samples, h, mu, &kd);
        }
        h += h0 / 2.0;
    }

    // Single-branch trace (degenerates from the paper's full branching
    // tracing for our tubular case):
    let seed = samples.iter().max_by(|a, b| sigma(a, cloud, h, &kd)
        .partial_cmp(&sigma(b, cloud, h, &kd)).unwrap()).unwrap();
    let branch = trace_branch_along_pca(seed, &samples, &kd);
    resample_to_polyline(&branch, 30)
}
```

### 3.5 E dependency surface

- `kiddo` v5.3.1 (k-d tree, ~3kLOC, pure Rust, MIT/Apache, 570k dl/mo,
  actively maintained). One new workspace dep.
- `nalgebra` (already present) for `SymmetricEigen` weighted-PCA.

### 3.6 E performance

Paper: ~1 min for 100k points on 2011 i7-2700K. Modern Rust + kiddo:
estimated 5-15s for 85k pts. Comfortably within file-save budget; would
NOT fit a per-slider-tick interactive budget (so user-adjustable
offsetting on top is unworkable).

---

## 4. Data structures and `.prep.toml` schema

### 4.1 In-memory (G variant — wire-handle)

- **`CenterlineAxisState`** (new) — see §2.5. Holds armed-handle state,
  per-handle picked anchors `Option<Point3<f64>>` (tip + floor),
  applied anchors, and pending action queue.
- **`HandleTag(Handle)`** marker component — distinguishes the two
  handle ball entities for the click-handler's entity dispatch.

### 4.2 `.prep.toml` (G variant)

The serialized polyline format **does not change**. cf-device-design and
cf-cast continue to consume the same `[centerline].points_m` array.

A new `[centerline_axis]` provenance block records the user's handle
anchors for round-trip + audit. Per-handle independence: tip can be
user-anchored while floor stays auto, or vice versa.

```toml
[centerline_axis]
# Per-handle anchor positions in meters (world coordinates after auto-
# PCA + auto-center). Each field is OPTIONAL — absence means the user
# did not anchor that handle (it stayed at the Cap-derived default).
# The polyline in [centerline].points_m already incorporates whichever
# anchors are present, so consumers don't need to re-apply.
applied_tip_m   = [0.001, 0.002, 0.075]   # optional
applied_floor_m = [0.000, 0.000, -0.075]  # optional
```

**Backward compatibility**: existing `.prep.toml` files without the
`[centerline_axis]` block load with both fields = `None` (default =
no user override = Cap-derived auto polyline). cf-device-design /
cf-cast see the same polyline shape they saw before — no breakage.
Old saves with the auto-derived polyline already in
`[centerline].points_m` continue to work uninterpreted.

### 4.3 Schema cross-references to keep in sync

- `tools/cf-scan-prep/src/main.rs::PrepCenterlineTrimBlock` (existing).
  Add `PrepCenterlineAxisBlock` adjacent. Plumb through `save_prep_toml`.
- `tools/cf-device-design/src/main.rs::PrepDoc` — already only reads
  `[centerline]`. No change needed.
- `tools/cf-cast-cli/.../prep.rs` — same. No change needed.

---

## 5. Performance budget

| Path | Operation | Budget | Cost estimate |
|---|---|---|---|
| Handle re-position (interactive) | Re-run algorithm → re-render wire + handle balls | <100ms (responsive) | Algorithm dominates: 30 slices × 169k face-plane intersections = 5M ops. ~50-200ms single-threaded; rayon-parallel across slices brings to <50ms. |
| `[Apply overrides]` click | Bake anchors into applied state | <100ms | trivial |
| Save | Write `[centerline_axis]` + serialize polyline | <500ms (mesh-I/O-dominated) | trivial |
| Cap step | First-time algorithm run after scan-detection | <500ms (one-time, save-time-equivalent) | Same 5M ops as handle re-position; rayon-parallel. |

**Algorithm hotspot**: `intersect_plane_with_mesh` is O(N_faces) per
slab. 30 slabs × 169k faces = 5M triangle-plane intersections per
algorithm invocation. Each intersection is a few flops; total well
under 100ms even single-threaded. Slab-loop is embarrassingly parallel
(rayon `par_iter` over the 30 slabs). If performance bites, decimate
mesh for the algorithm (use the existing `compute_envelope_proxy_mesh`
output at 1500 faces — algorithm cost drops to 45k ops).

---

## 6. Test plan

### 6.1 Unit tests (added to `tools/cf-scan-prep/src/main.rs` `#[cfg(test)]`)

**Algorithm correctness (per-slab area-weighted polygon centroid)**:

1. **`algorithm_axisymmetric_cylinder_centerline_along_axis()`** — synthetic axisymmetric cylinder along `spine_hint = +Z`, vertex-uniformly-sampled. Algorithm output: 30 points on the cylinder's central axis within ε = 1e-6 m. Contract test.
2. **`algorithm_offset_capsule_centerline_at_body_axis()`** — synthetic capsule shifted by `Δx = 5mm` from origin (the iter-1 failure-mode mock). Algorithm output: polyline x-coords ≈ 5mm at all slabs except dome tip (degenerate). **The regression test that the area-weighted centroid escapes the AABB+spine_hint bias.**
3. **`algorithm_density_independent()`** — synthetic capsule where one side has 10× the vertex density of the other (mocks scanner-viewpoint bias, the documented failure source). Algorithm output: centerline does NOT drift toward the dense side; same result within ε as the uniform-density baseline. **Density-independence regression test — the load-bearing property.**
4. **`algorithm_degenerate_slabs_filled_by_interpolation()`** — synthetic capsule. Force MIN_SLAB_AREA_M2 high enough that 2-3 dome slabs are null. Verify interpolation fills them smoothly between non-null neighbors.
5. **`algorithm_xyz_independent()`** — input mesh rotated 30° around Y so body axis is NOT along world Z. `spine_hint` updated to match. Algorithm output: polyline runs along the rotated body axis, NOT along world Z. **See [[scans-axis-orientation]].**

**Endpoint override behavior**:

6. **`no_overrides_uses_mesh_extremes()`** — `compute_centerline_polyline(mesh, spine_hint, n, None, None)` — endpoints at `d_min` / `d_max` projections of mesh on spine_hint.
7. **`tip_override_pins_first_point()`** — `h_tip = Some(p_t)`, `h_floor = None` — polyline[0] == p_t; polyline[n-1] at d_max extreme.
8. **`both_overrides_define_axis_and_endpoints()`** — both Some, polyline runs from h_tip to h_floor along (h_floor - h_tip).normalize().

**Handle state machine**:

9. **`armed_handle_then_mesh_click_moves_anchor()`** — drive `armed = Some(Tip)` → simulate Pointer<Click> on scan mesh → verify `anchor_tip` updated AND `armed = None`.
10. **`armed_handle_then_same_handle_click_toggles_off()`** — drive `armed = Some(Tip)` → click tip handle entity → verify `armed = None`.
11. **`apply_overrides_action_copies_picked_to_applied()`** — `pending_action = Some(Apply)` → handler → `applied_* == anchor_*`.
12. **`reset_tip_clears_only_tip()`** — `ResetTip` → `anchor_tip = applied_tip = None`; floor unchanged. Symmetric test for floor.

**Serialization**:

13. **`save_prep_toml_round_trips_per_handle_anchors()`** — write + read-back. Variants: both anchored, only tip, only floor, neither. Backward-compat: old `.prep.toml` (no block) → both default `None`.

### 6.2 Integration test (cf-device-design side)

Add to `tools/cf-device-design/src/main.rs` `#[cfg(test)]`:

14. **`layer_dome_symmetric_when_centerline_algorithmically_derived()`** — synthetic offset-capsule fixture: body shifted by `Δx = 5mm` from origin. Two cases:
    - Centerline from old AABB+spine_hint algorithm (iteration #5 behavior): layer dome asymmetric (asserted as baseline failure mode).
    - Centerline from new per-slab area-weighted polygon centroid algorithm: layer dome symmetric within ε. **Regression test for the iter-1 failure mode — verifies the algorithm change alone (no user clicks) fixes the lopsidedness.**

### 6.3 Visual / fixture verification

15. **iter-1 fixture, zero clicks** — `~/scans/sock_over_capsule.stl` through the new pipeline: load → Cap (algorithm auto-runs) → Save. Re-open in cf-device-design. **Expected**: layer dome visually symmetric on both lateral axes WITHOUT any user handle interaction. This is the primary success criterion — algorithm-only fix.

16. **iter-1 fixture, with tip override** — same fixture: load → Cap → click tip handle → click body's dome point → wire updates → `[Apply overrides]` → Save. Re-open in cf-device-design. **Expected**: layer dome even more centered than test #15 (if dome tip was the algorithm's weak spot). Demonstrates the override mechanism works without breaking anything.

17. **Pre/post-reconstruct handle persistence** — fixture: scan → drag floor handle to position P → apply → toggle Reconstruct → verify floor handle ball still renders at P (anchor preserved across reconstruct toggle). User can re-drag if they want it on the new reconstructed surface, but the anchor doesn't auto-clear.

---

## 7. Migration / backward compatibility

- **Existing `.prep.toml` files** load with `applied_tip_m = applied_floor_m
  = None` (no `[centerline_axis]` block). The centerline polyline in
  `[centerline].points_m` is honored as-is (whatever was baked in by the
  iteration-5 AABB+spine_hint algorithm). Downstream consumers see the
  same shape they saw before. No re-prep required for old files; users
  who want the new algorithm's output need to re-prep through cf-scan-
  prep.
- **cf-device-design** unchanged — reads the polyline; doesn't care
  whether it was offset.
- **cf-cast-cli** unchanged — same.
- **Workshop iter-1 fixture** (`sock_over_capsule.cleaned.stl`) — once
  the implementation lands, the user will re-prep through cf-scan-prep
  with the new axis-refine panel, replacing the existing cleaned STL +
  `.prep.toml` pair. This is the unblock event for the iter-1 cast.

---

## 8. Open questions / risks

1. **(R1) — How often does the algorithm need an override?**
   Best case: algorithm alone fixes the iter-1 failure mode and 90%+
   of scans; handle overrides are an unused safety net. Worst case:
   dome tips consistently need user override (the algorithm's known
   weak spot is small/null cross-sections at the apex). Iter-1 visual
   test #15 (zero clicks) is the primary success criterion; #16 (with
   tip override) is the fallback. **If #15 fails, the path forward is
   the override habit AND consider G.s1 (snap-to-centroid on click)
   to make the override more precise.**

2. **(R2) — Click precision on curved surfaces vs. body geometric axis.**
   A click lands on the body's SURFACE, not its geometric center. The
   recorded anchor is offset from the true axis by
   ~`body_radius_at_handle_depth`. For dome clicks: small (~5-10mm
   offset for a 50mm capsule). For floor clicks: larger. The wire's
   visible offset from the body's perceived axis becomes the user's
   visual feedback — they re-click to refine if needed. **Stretch
   §2.7 G.s1 (snap-to-centroid)** turns clicks into axis-centered
   anchors automatically — implement if iter-1 shows raw clicks are
   too imprecise.

3. **(R3) — Visual feedback for axis verification.**
   User mentally verifies "wire sits on body axis" via the 3D
   viewport. Rotational ambiguity (cylinder seen from one side looks
   fine even when axis is offset along the un-viewed direction).
   *Mitigation*: orbit camera in place; user can rotate to verify from
   multiple angles. The wire + handle balls are always visible inside
   the scan mesh (semi-transparent body material if needed).

4. **(R4) — `MeshPickingPlugin` × `OrbitCameraPlugin` interaction.**
   Bevy 0.18's `Pointer<Click>` distinguishes click (no drag) from
   drag (mouse-down → motion → mouse-up). Need to verify in
   implementation. *Mitigation if conflict*: gate the picking-handler
   system on `armed_handle.is_some() OR ev.target() is HandleTag` —
   handle clicks always armed/disarm; mesh clicks only register when a
   handle is armed.

5. **(R5) — Direction F.a complementary path.**
   The bookmark's "reconstruct asymmetry feedback loop" (bias source #3)
   isn't addressed by G alone. Once a scan's centerline is user-aligned
   via G, a subsequent re-prep that re-runs reconstruct will still build
   asymmetric floor geometry around an asymmetric body. **For iter-1 ship,
   not blocking** (user-aligned centerline + smoothed reconstruct via the
   Taubin pass already shipped is workshop-acceptable). **Bank F.a as a
   followup** — `~300 LOC` to make the reconstruct algorithm produce a
   body-of-revolution slug around the (now reliably-placed) prescriptive
   centerline. Add to `MEMORY.md` followups.

6. **(R6) — Curved body support.**
   G's straight-line constraint matches our target use case
   `[[project_cf_scan_prep_target_use_case]]`. Genuinely curved limbs
   (bent finger, flexed arm) would need Direction E (L1-medial) or A
   (MCF) added later. No code is being written that BLOCKS that future
   path — `.prep.toml` schema can accept curved polylines today.

7. **(R7) — Handle anchor survival across pipeline step changes.**
   Workflow: user drags handles → Apply → re-runs Scan / Trim /
   Reconstruct upstream. Do anchors survive? Spec answer: anchors are
   world-space points; they survive any upstream change that doesn't
   re-transform the mesh frame (auto-PCA / auto-center happen at load,
   not at re-Scan). Re-Cap is the only step that conceptually
   invalidates anchors (it re-derives the Cap-time defaults from
   scratch). Pick during implementation: preserve anchors across re-
   Cap (UX nicety, user re-applies if displeased) OR zero them (safer,
   user re-drags). Test #13 covers reconstruct-toggle persistence.

8. **(R8) — Synthetic test fixtures to author.**
   Tests #2 (offset capsule), #3 (density-asymmetric capsule), #5
   (rotated capsule), and #14 (cf-device-design layer regression) all
   need new synthetic fixtures. ~100 LOC total across fixture
   builders. Existing tests have a tapered cylinder
   (`centerline_tapered_cylinder_tip_stays_on_axis`) — that's it. The
   density-asymmetric capsule is the most novel; the others are
   variants of the offset-capsule base. **Implementation arc authors
   #2 + #3 FIRST** (red tests → impl → green) — these gate the
   algorithm correctness contract.

9. **(R9) — Plane-mesh intersection edge cases.**
   `intersect_plane_with_mesh` has well-known edge cases: vertices
   exactly on the plane, plane tangent to a face, plane intersecting
   multiple closed loops (non-convex bodies). For body-part scans, the
   single-convex-loop assumption holds in practice but the
   implementation should be defensive: return all loops, pick the
   loop with max area for centroid computation (heuristic for "main
   body" loop). Cost: a few extra LOC in the intersection subroutine.

10. **(R10) — Handle render scale across body sizes.**
    Handle balls at fixed ~3mm world radius work for typical body-part
    scans (50-150mm) but become too small / too large for outliers.
    *Mitigation*: scale handle radius proportional to body AABB diag
    (e.g., `radius = 0.02 * diag`, clamped). Pick during implementation
    after visual feedback on the iter-1 fixture.

---

## 9. Implementation arc shape (for the next-next session)

Out of scope for this recon. Suggested ladder (not authoritative).
**The algorithm ships first; UI layers on top.** If the algorithm alone
fixes iter-1 (test #15 passes), the handle UI is a polish item — could
even be deferred to a separate arc.

1. **Red tests (algorithm)**: synthetic offset-capsule + density-asymmetric-capsule + rotated-capsule fixtures + tests #2, #3, #5. All fail with iteration #5's `compute_centerline_polyline`.
2. **Algorithm core**: `intersect_plane_with_mesh` + `area_weighted_centroid` + `fill_null_by_interpolation` subroutines. Rewrite `compute_centerline_polyline` to use them (signature gains optional `h_tip` / `h_floor` params, callers pass `None, None`). Tests #1-#8 green.
3. **Red test (cf-device-design)**: test #14 (layer dome symmetric with new algorithm). Initially fails until step 2 lands.
4. **iter-1 fixture, zero clicks (visual test #15)**: re-prep `sock_over_capsule.stl` through the new algorithm. **This is the primary success gate.** If #15 passes → handle UI is nice-to-have; if it fails → handle UI becomes load-bearing for the workshop ship.
5. **Handle UI**: `CenterlineAxisState` + `MeshPickingPlugin` + click handler + Apply/Reset/Reset-tip/Reset-floor + handle ball rendering + overlay update. Tests #9-#12 green.
6. **Schema**: `[centerline_axis]` block + round-trip test #13.
7. **iter-1 with overrides (visual test #16)**: verify the override mechanism works on the real fixture.
8. **Grade + clippy + ship**.

Estimated arc length: 6-10 commits. Test count: cf-scan-prep +13,
cf-device-design +1. Larger than the original two-click variant (the
algorithm itself is ~300 LOC + handle UI ~250 LOC = ~550 LOC) but
correspondingly more value — the algorithm fixes the iter-1 failure
mode whether or not the user touches the handles.

---

## 10. References

- `docs/CENTERLINE_RECON_BOOKMARK.md` — recon brief (problem, 5
  failures, 7 directions).
- `tools/cf-scan-prep/src/main.rs::compute_centerline_polyline` —
  iteration #5 (AABB+spine_hint) — **to be replaced** by the new
  per-slab algorithm; same function name, signature gains
  `h_tip` / `h_floor` optional params.
- `tools/cf-scan-prep/src/main.rs::CenterlineTrimState`,
  `render_centerline_trim_section` — slider+Apply pattern the handle
  UI panel follows for layout consistency.
- `tools/cf-scan-prep/src/main.rs::smooth_polyline` (line ~2150) —
  existing 3-tap moving-average smoother with endpoint pinning;
  reused by the algorithm's step 6.
- `tools/cf-device-design/src/main.rs::compute_envelope_proxy_mesh` —
  per-vertex radial direction consumer + decimated mesh source if
  algorithm performance bites.
- L1-medial skeleton paper (deferred to Direction E alternative path):
  Huang et al. 2013,
  [SFU mirror](https://www.cs.sfu.ca/~haoz/pubs/huang_sig13_l1skel.pdf).
- Polygon centroid formula:
  [Wikipedia — Centroid of a polygon](https://en.wikipedia.org/wiki/Centroid#Of_a_polygon).
- Bevy 0.18 `MeshPickingPlugin` for the handle UI click dispatch.
