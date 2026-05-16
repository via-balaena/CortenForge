# cf-device-design Clip Plane (Fit-Viz Rung 1) — Spec

**Status**: **SHIPPED on `dev` 2026-05-16** in 5 implementation
commits (`46421011` → `799e478f`) + this docs commit. cf-device-
design at 118 tests / clippy clean / release build clean. Visual
gate passed on iter-1 `sock_over_capsule.cleaned.stl` (user-
confirmed). Arc-end memo at
`memory/project_cf_device_design_clip_plane_arc.md`. This spec
stays as the design-of-record.

**One spec drift discovered at implementation**: the WGSL snippet
used `@group(2)` for the material bind group; that was Bevy 0.17.
Bevy 0.18 moved the material bind group to index 3
(`bevy_pbr::MATERIAL_BIND_GROUP_INDEX = 3`). Fixed by switching to
the `#{MATERIAL_BIND_GROUP}` shader-def in the embedded WGSL —
self-updating against any further renumbering. Caught by the
first sub-leaf-3 smoke run's `wgpu` validation panic; cost ~5
minutes.

**Original status**: SPEC 2026-05-16 (post-SDF-layers ship
`320d6dcd`, post-fit-viz bookmark `e786dbc9`). Implementation
deferred to the next session.

**Parent**: [fit-viz program bookmark](./CF_DEVICE_DESIGN_FIT_VIZ_BOOKMARK.md).
**Rung order**: 1 of 6 — cheapest, composes forward into rungs 2
(per-step playback) and 3 (scan-as-intruder).

## Goal

A toggleable clip plane that lets the user peek inside the cavity +
nested layer surfaces, anchored to the centerline so the controls
read as "where along the device, and which way to split lengthways."

Visible only when the loaded scan has a centerline (i.e., the
`.prep.toml`'s `[centerline]` block was populated by cf-scan-prep).
Disabled with a tooltip otherwise.

## User-pinned decisions (2026-05-16, in-session)

Two open UX questions from the bookmark were settled before drafting
this spec:

1. **Plane orientation**: **lengthways only** (plane contains the
   centerline tangent at the slide position). Two controls — position
   along centerline + roll around tangent. Cross-section discs and
   free-axis gizmo are explicit non-goals for rung 1; either can be
   a follow-up sub-leaf later.
2. **Cut face**: **hollow shell** (per-fragment `discard`). The
   layer meshes are already double-sided + cull-mode None, so the
   user sees the inside surface where the plane sliced through.
   Filled cap (CT-scan-style colored interior) is explicitly
   deferred — see "Open risks / followups" below.

## Approach

**`ExtendedMaterial<StandardMaterial, ClipPlaneExt>`** with a custom
fragment shader that calls `discard` on the kept-out side of the
plane. Bevy 0.18 supports this directly — see
`bevy-0.18.1/examples/shader/extended_material.rs`.

Why this approach won over the alternatives in the bookmark:

| Approach | Cost | Composability w/ rungs 2-3 | Filled cap? |
|---|---|---|---|
| **A. Per-mesh fragment-discard (ExtendedMaterial)** | 1 WGSL + 1 uniform + plugin reg | ✓ free | ✗ (deferred) |
| B. Custom Material from scratch | larger, loses PBR | ✓ | ✗ |
| C. CPU-side mesh trim / MC re-extract bounded grid | rebuild every slider tick | (would need per-frame re-extract) | ✓ (cap geom) |

Approach A is cheapest AND composes — the same material plugin and
uniform serve any future mesh (deformed-per-step in rung 2, scan-
intruder in rung 3). Filled cap (only available via C or a stencil
follow-up pass) is left deferred; the visual gate will tell us
whether hollow reads well enough.

## Centerline-anchored plane resolution

The plane has two DOFs, both relative to the centerline polyline
(in physics-frame meters, same convention as `Centerline.points_m`):

- `t ∈ [0, 1]` — parametric position along the centerline polyline
  (arc-length normalized).
- `roll ∈ [0°, 360°)` — rotation around the centerline tangent at
  point `P(t)`.

Resolves to world `(origin, normal)` as follows:

1. **Origin**: linearly interpolate the polyline at arc-length
   parameter `t`. Centerline of length `L`, find segment `i` where
   `Σ_{j<i} |Δ_j| ≤ t·L < Σ_{j≤i} |Δ_j|`, lerp inside that segment.
2. **Tangent `T`**: forward difference of the chosen segment's
   endpoints, normalized. At segment boundaries, use the segment
   that contains `P(t)`. (Smoothing across segments is overkill for
   rung 1 — iter-1 centerline is straight; for curved centerlines
   the discontinuity at each polyline vertex is small and
   user-imperceptible.)
3. **Reference up `U_ref`**: project world Z onto the plane
   perpendicular to `T`, normalize. If `T` is within 1° of world Z,
   fall back to world X (avoids degeneracy).
4. **Binormal `B`** = `T × U_ref` (right-handed).
5. **Plane normal `N`** = `cos(roll)·U_ref + sin(roll)·B`. This is a
   unit vector perpendicular to `T`, so the plane CONTAINS the
   tangent direction by construction → longways split.
6. **Flip half** (button): negate `N` if `state.flip` is set.

Render-space conversion: apply the same `UpAxis::PlusZ` swap +
`render_scale.0` lift the existing scene uses (`draw_reference_overlays`
is the precedent for centerline-frame → render-frame conversion).

WGSL discard test: `if (dot(world_pos.xyz, N_render) - dot(origin_render, N_render) < 0.0) discard;` — i.e., kept half is the positive side of the plane.

## Data model

```rust
// New module: tools/cf-device-design/src/clip_plane.rs

/// Clip-plane state. Anchored to the centerline; meaningless without one.
#[derive(Resource, Debug, Clone, Copy, PartialEq)]
pub struct ClipPlaneState {
    pub enabled: bool,
    /// Arc-length-normalized position along centerline, [0, 1].
    pub t: f64,
    /// Rotation around centerline tangent, radians, [0, 2π).
    pub roll_rad: f64,
    /// Negate the plane normal (swap which half is kept).
    pub flip: bool,
}

impl Default for ClipPlaneState {
    fn default() -> Self {
        Self { enabled: false, t: 0.5, roll_rad: 0.0, flip: false }
    }
}

/// Resolved plane in PHYSICS-FRAME meters (NOT render-frame).
/// `None` when `state.enabled == false` OR centerline is empty / has < 2 points.
pub struct ResolvedPlane { pub origin_m: Point3<f64>, pub normal: Vector3<f64> }

pub fn resolve_plane(state: &ClipPlaneState, centerline: &Centerline) -> Option<ResolvedPlane>;
```

The `ExtendedMaterial` extension carries the resolved plane as a
single `vec4<f32>` uniform (`xyz` = normal in render-frame, `w` =
`dot(origin_render, normal_render)`) plus a `u32` enabled flag:

```rust
#[derive(Asset, AsBindGroup, Reflect, Debug, Clone, Default)]
pub struct ClipPlaneExt {
    /// (normal.xyz, plane_offset) packed in render-frame.
    #[uniform(100)]
    pub plane: Vec4,
    /// Non-zero = clipping enabled.
    #[uniform(100)]
    pub enabled: u32,
    // WebGL2 alignment padding only if `webgl2` feature ever lands;
    // not needed for native target.
}

impl MaterialExtension for ClipPlaneExt {
    fn fragment_shader() -> ShaderRef { CLIP_PLANE_SHADER_HANDLE.into() }
    fn prepass_fragment_shader() -> ShaderRef { CLIP_PLANE_SHADER_HANDLE.into() }
}
```

## WGSL (sketch)

```wgsl
#import bevy_pbr::{
    pbr_fragment::pbr_input_from_standard_material,
    pbr_functions::{alpha_discard, apply_pbr_lighting, main_pass_post_lighting_processing},
}

struct ClipPlane {
    // .xyz = normal (render-frame), .w = dot(origin_render, normal)
    plane: vec4<f32>,
    enabled: u32,
}
@group(2) @binding(100) var<uniform> clip: ClipPlane;

@fragment
fn fragment(
    in: VertexOutput,
    @builtin(front_facing) is_front: bool,
) -> FragmentOutput {
    if (clip.enabled != 0u) {
        let d = dot(in.world_position.xyz, clip.plane.xyz) - clip.plane.w;
        if (d < 0.0) { discard; }
    }
    var pbr_input = pbr_input_from_standard_material(in, is_front);
    pbr_input.material.base_color = alpha_discard(pbr_input.material, pbr_input.material.base_color);
    var out: FragmentOutput;
    out.color = apply_pbr_lighting(pbr_input);
    out.color = main_pass_post_lighting_processing(pbr_input, out.color);
    return out;
}
```

Exact imports + prepass-fragment will be confirmed against
`extended_material.wgsl` (the Bevy example) at implementation time —
the published crate doesn't ship the asset, but the example file is
upstream-canonical and the snippets above match its pattern.

## Sub-leaf ladder

| # | Slice | Description | Test posture |
|---|---|---|---|
| 1 | `mod clip_plane` + `ClipPlaneState` + `resolve_plane` | Data model + plane math. Pure-function unit tests. | unit (straight + curved centerlines, edge cases) |
| 2 | `MaterialPlugin<ExtendedMaterial<StandardMaterial, ClipPlaneExt>>` + embedded WGSL | Plugin registration + shader handle bootstrap. | compile-clean + plugin add doesn't panic |
| 3 | Swap cavity + layer + scan spawn sites | Replace `MeshMaterial3d<StandardMaterial>` → `MeshMaterial3d<ExtendedMaterial<…>>` on the 3 spawn sites. Inline scan-spawn helper (avoid forking cf-viewer). | per-crate clippy + visual smoke (user side) |
| 4 | `update_clip_plane_uniform` system | Per-frame: resolve plane from state + centerline, push to all `ClipPlaneExt`'s on the changed-key path (`Local<>` snapshot — `is_changed` footgun [[project-bevy-is-changed-footgun]]). | manual visual + plane-equation unit (separate, with synthetic state) |
| 5 | Egui "Clip plane" UI panel | Toggle, two sliders, flip button. Disabled when centerline empty (tooltip: "needs `.prep.toml` centerline"). | manual UI walk on iter-1 fixture |
| 6 | Eyes-on visual gate + docs commit | User-side check that drag-along-centerline + roll behave sanely on `sock_over_capsule.cleaned.stl`. Mark bookmark RESOLVED for rung 1, leave rungs 2-6 open. | user visual gate |

Estimated 3–6 commits, matches bookmark range. If sub-leaf 2 (WGSL
embedding macro) turns out trickier than the example suggests, may
split sub-leaf 2 into 2a (plugin scaffold) + 2b (uniform push +
visual verify on a single cube) before sub-leaf 3 touches the real
mesh spawn sites.

## Constants

| Name | Value | Why |
|---|---|---|
| `DEFAULT_T` | `0.5` | Mid-centerline. Workshop case: user typically wants to peek at the middle of the device first. |
| `DEFAULT_ROLL_RAD` | `0.0` | Plane normal aligned with the projected-world-up reference — looks "horizontal" relative to scan orientation by default. |
| `ROLL_MIN_DEG` / `ROLL_MAX_DEG` | `0.0` / `360.0` | One full revolution. UI may wrap. |
| `WORLD_Z_DEGENERATE_DEG` | `1.0` | If tangent is within 1° of world Z, fall back to world X as the projection reference. Avoids `U_ref ≈ 0`. |
| `CLIP_PLANE_SHADER_HANDLE` | Stable UUID via `weak_handle!` macro | Bevy 0.18 idiom for embedded internal shaders (vs file-path asset loading; cf-device-design has no `assets/` directory). Exact macro confirmed at implementation. |

## Test plan

**Unit (`mod clip_plane::tests`):**

1. `resolve_plane_straight_centerline_default` — 3-point centerline along world Z, default state (t=0.5, roll=0, flip=false): origin = midpoint, normal ⟂ Z (specifically, world-X-ish based on the `U_ref` fallback rule), tangent ⊥ normal.
2. `resolve_plane_roll_90_rotates_normal_90_around_tangent` — same centerline, roll = π/2: normal rotates 90° around tangent.
3. `resolve_plane_flip_negates_normal` — flip = true: normal is negated.
4. `resolve_plane_curved_centerline_picks_local_segment` — kinked polyline (Z then X), confirm `t = 0.5` lands in the right segment with the right tangent.
5. `resolve_plane_tangent_near_world_z_uses_x_fallback` — centerline along world Z, default reference: `U_ref` should be world-X-ish (not zero or NaN). Sanity-check the degeneracy guard.
6. `resolve_plane_empty_centerline_returns_none` — 0 points → `None`.
7. `resolve_plane_single_point_centerline_returns_none` — 1 point → `None` (no tangent definable).
8. `resolve_plane_disabled_returns_none` — `enabled = false` → `None`.
9. `resolve_plane_t_clamped` — `t = -0.5` or `t = 1.5` returns plane at endpoint, not panic.

Optional 10: arc-length parameterization correctness — non-uniform polyline (segments of unequal length), `t = 0.5` lands halfway by arc length, not by index.

**Integration / visual gate (user-side):**

- Iter-1 fixture (`sock_over_capsule.cleaned.stl`): toggle on, slide t from 0 to 1, confirm the clip plane sweeps along the scan's length. Roll through 0° → 90° → 180° → 270°, confirm the plane rotates around the centerline like a windshield wiper. Flip button swaps kept half.
- All meshes (cavity + 6 layers + scan) clip in lockstep.
- Heat-map (when on) stays consistent: only the kept half is colored.

No headless integration test — fragment-discard is hard to test outside a real render pass, and cf-device-design's slice ladder has set the precedent of leaning on user-side visual gates for rendering changes (slice 9 sub-leaf 4 was the load-bearing example).

## Open risks / followups

1. **Embedded-shader macro for Bevy 0.18.** Two viable patterns: (a) `load_internal_asset!` (older, still works in 0.18 last I saw); (b) the newer `Shader::from_wgsl` + manual `Assets<Shader>` insert at `Plugin::build`. Either way the WGSL string is embedded in the binary; no `assets/` directory needed. Pin exact macro at sub-leaf 2 implementation time after reading `bevy_pbr/src/lib.rs` for current idiom. Risk: LOW — both patterns are well-trodden.

2. **Prepass shader override.** Default forward rendering in cf-device-design (no `DeferredPrepass` on the camera), but shadows + the depth prepass DO run when `bevy_pbr` is on. Without overriding `prepass_fragment_shader`, clipped fragments would still cast shadows + write depth, producing ghost shadows on the kept half. Sub-leaf 4's uniform push must reach the prepass; the prepass shader gets a slimmer fragment signature than forward (no full `pbr_input`). Sub-leaf 2 should land BOTH forward + prepass shader overrides together (single WGSL with two entry points, or two files).

3. **`spawn_face_mesh` touchpoint.** cf-viewer's `spawn_face_mesh` hardcodes `StandardMaterial`. Sub-leaf 3 will inline a local equivalent in cf-device-design (uses the same `triangle_mesh_flat_shaded` from cf-viewer, but the local helper spawns with the extended material). This keeps cf-viewer generic and avoids the leaked-asset hack of inserting a replacement `MeshMaterial3d` after spawn.

4. **Filled cap (deferred).** The bookmark's "filled cross-section cap or hollow shell" question landed on hollow. If the visual gate reads as hard to interpret with 6 nested shells, follow up with EITHER:
   - **Screen-space stencil cap**: render a full-screen quad behind the scene, mask by stencil written during the clipped pass — paints the cut interior in the layer color. Single extra pass.
   - **CPU-side cap geometry**: close each layer's cut boundary polygon at the plane (essentially MC re-extract on a clipped grid, then triangulate the boundary). Heavier; matches Approach C from the bookmark's recon questions. Both touch parts of the system not in scope for rung 1.

5. **Centerline-curve plane discontinuity at segment boundaries.** A polyline tangent is constant within a segment + jumps at vertices. For iter-1 (straight centerline) this is invisible. For curved centerlines, the plane normal will visibly snap as `t` crosses a segment boundary. Smoothing via Catmull-Rom or finite-difference-over-neighbors is overkill for rung 1; flag as a polish item if iter-2+ surfaces it.

6. **`is_changed` footgun ([[project-bevy-is-changed-footgun]]).** `ClipPlaneState` is mutated by egui every frame the user touches the panel, but the uniform push system shouldn't actually re-upload every frame regardless. Sub-leaf 4 uses snapshot-and-compare via `Local<Option<UniformKey>>` rather than gating on `ResMut::is_changed`. Pattern is well-established in this crate (`update_layer_meshes`, `invalidate_on_geometry_change`).

7. **No-centerline UX.** If the scan was loaded without `.prep.toml` or its `[centerline]` block was empty, `resolve_plane` returns `None` and the clip plane is silently inert. UI must disable the toggle + show a tooltip explaining how to get a centerline (run cf-scan-prep on the scan first). Sub-leaf 5 owns this.

## Compositon with later rungs (sanity check)

- **Rung 2 (per-step playback)**: clip plane operates on whatever mesh is mounted on the layer entities at the moment. If rung 2 swaps the layer meshes per playback step, clip plane still works — the plane is in world frame, the meshes are in world frame. ✓
- **Rung 3 (scan-as-intruder)**: a new mesh entity (the deformable scan intruder) needs to use the same `ExtendedMaterial<StandardMaterial, ClipPlaneExt>` to inherit clipping. Trivial — the material plugin is registered once. ✓
- **Rung 4 (pressure scoring)**: a per-cavity-surface scalar overlay. If implemented as per-vertex COLOR on the cavity mesh, clipping behavior is independent. ✓

## When to start implementation

Next session. Cold-read prompt: this spec is self-contained — start
at sub-leaf 1, work down. Pre-implementation, double-check that:

1. `bevy-0.18.1/examples/shader/extended_material.rs` still
   represents the canonical pattern (drift is unlikely in a patch
   release but cheap to verify).
2. `Centerline.points_m` is still `Vec<Point3<f64>>` (it was at the
   time of this spec).
3. No PR has shipped between this spec and implementation that
   changes the mesh-spawn sites in main.rs.

## Related memory

- [[project-cf-device-design-fit-viz-bookmark]] — parent (the
  bookmark this spec implements rung 1 of).
- [[project-cf-device-design-sdf-layers]] — the arc that established
  the current SDF-based cavity + layer mesh pipeline this clips.
- [[project-bevy-is-changed-footgun]] — load-bearing for sub-leaf 4.
- [[project-centerline-spec]] — the centerline algorithm this plane
  anchors to (per cf-scan-prep PR #248).
