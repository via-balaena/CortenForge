//! Clip-plane (fit-viz rung 1, `docs/CF_DEVICE_DESIGN_CLIP_PLANE_SPEC.md`).
//!
//! Lengthways-only clip plane anchored to the centerline polyline. Two
//! user-dialed DOFs:
//!
//! - `t ∈ [0, 1]` — arc-length-normalized position along the
//!   centerline.
//! - `roll ∈ [0, 2π)` — rotation around the centerline tangent at the
//!   slide position.
//!
//! Resolves to a world-space `(origin, normal)` in PHYSICS-FRAME
//! meters. Render-frame conversion (the `UpAxis::PlusZ` swap +
//! `RenderScale` lift) is the uniform-push system's responsibility
//! (sub-leaf 4); this module is pure math + the resource declaration.
//!
//! Sub-leaf 2 layers on:
//!
//! - the [`ClipPlaneExt`] `MaterialExtension` (a `vec4` plane + a
//!   `u32` enabled flag, packed into a single uniform at binding 100)
//!   plus the WGSL fragment-discard shader (embedded next to this
//!   file, loaded via Bevy's `embedded_asset!` macro);
//! - the [`ClipPlanePlugin`] that registers the shader asset, the
//!   `MaterialPlugin<ExtendedMaterial<StandardMaterial, …>>`, and the
//!   [`ClipPlaneState`] resource.
//!
//! Sub-leaves 4 + 5 add the per-frame uniform push system and the
//! egui panel.
//!
//! **Prepass posture**: cf-device-design's camera does NOT add
//! `DepthPrepass` / `MotionVectorPrepass` / `NormalPrepass` (see
//! `cf-viewer::setup_camera_and_lighting`) and its directional light
//! has `shadows_enabled: false`, so the depth + shadow prepasses
//! never run for these meshes — i.e., no "ghost shadows on the kept
//! half" risk the spec called out (open risk #2). To stay defensive
//! against a future arc enabling shadows or prepass without also
//! reaching into this module, we explicitly set
//! [`MaterialExtension::enable_prepass`] → `false` below, which
//! prevents the prepass pipeline from being built for our extended
//! material. If a future rung needs shadows, the fix is to add a
//! prepass fragment shader override (and flip `enable_prepass` back
//! to `true`); the spec's open-risk #2 covers the followup.
//!
use bevy::{
    asset::embedded_asset,
    pbr::{ExtendedMaterial, MaterialExtension, MaterialPlugin},
    prelude::*,
    render::render_resource::AsBindGroup,
    shader::ShaderRef,
};
use bevy_egui::egui;
use cf_bevy_common::axis::UpAxis;
use cf_viewer::RenderScale;
use mesh_types::{Point3, Vector3};

use crate::Centerline;

/// Embedded-asset URI of the clip-plane WGSL shader (sub-leaf 2).
///
/// Resolves to `tools/cf-device-design/src/clip_plane.wgsl`, embedded
/// at compile time by [`ClipPlanePlugin::build`]'s `embedded_asset!`
/// call. The crate-name segment uses Cargo's underscored bin name
/// (`cf_device_design`), matching `module_path!()` — see
/// `bevy_asset::io::embedded::_embedded_asset_path` for the exact
/// resolution rule.
const CLIP_PLANE_SHADER_PATH: &str = "embedded://cf_device_design/clip_plane.wgsl";

/// Type alias for the concrete extended material used to clip the
/// cavity + per-layer + scan meshes (sub-leaf 3 swaps the spawn
/// sites).
pub(crate) type ClipPlaneMaterial = ExtendedMaterial<StandardMaterial, ClipPlaneExt>;

/// `ExtendedMaterial` extension that adds the clip-plane uniform +
/// fragment-discard shader to a `StandardMaterial` base. Both fields
/// share binding slot 100 — `AsBindGroup` packs them into a single
/// uniform struct matching `struct ClipPlane` in the WGSL.
///
/// Render-frame meters (NOT physics-frame). The uniform-push system
/// in sub-leaf 4 applies the `UpAxis::PlusZ` swap + `RenderScale`
/// lift before writing this field.
#[derive(Asset, AsBindGroup, Reflect, Debug, Clone, Default)]
pub(crate) struct ClipPlaneExt {
    /// `.xyz` = unit plane normal (render-frame), `.w = dot(origin,
    /// normal)` — the plane offset along the normal. Kept half is
    /// `dot(world_pos, normal) - w >= 0`.
    #[uniform(100)]
    pub plane: Vec4,
    /// Non-zero → the WGSL fragment shader runs the discard test.
    /// Zero → pass-through (the entire world is kept). The Rust-side
    /// gate ([`ClipPlaneState::enabled`] AND a non-empty centerline)
    /// is the source of truth; the uniform flag is a render-side
    /// mirror so toggling doesn't require a material asset rebuild.
    #[uniform(100)]
    pub enabled: u32,
}

impl MaterialExtension for ClipPlaneExt {
    fn fragment_shader() -> ShaderRef {
        ShaderRef::from(CLIP_PLANE_SHADER_PATH)
    }

    fn enable_prepass() -> bool {
        // See the module docstring — cf-device-design's camera +
        // lighting setup means the depth/shadow prepasses never run
        // for our meshes today; opting out here prevents the prepass
        // pipeline being built for our material if a future arc adds
        // `DepthPrepass` without first authoring a prepass shader
        // override.
        false
    }
}

/// Plugin that embeds the clip-plane WGSL asset + registers the
/// `MaterialPlugin` for [`ClipPlaneMaterial`] + inserts the
/// [`ClipPlaneState`] resource + the per-frame uniform-push system.
/// Add AFTER `DefaultPlugins` (the `EmbeddedAssetRegistry` resource
/// the `embedded_asset!` macro writes into is set up by Bevy's
/// `AssetPlugin`, which lives in `DefaultPlugins`).
pub(crate) struct ClipPlanePlugin;

impl Plugin for ClipPlanePlugin {
    fn build(&self, app: &mut App) {
        embedded_asset!(app, "clip_plane.wgsl");
        app.init_resource::<ClipPlaneState>();
        app.add_plugins(MaterialPlugin::<ClipPlaneMaterial>::default());
        app.add_systems(Update, update_clip_plane_uniform);
    }
}

// ============================================================
// Sub-leaf 4 — per-frame uniform push.
// ============================================================
//
// Resolves the clip plane from `ClipPlaneState` + `Centerline` +
// `UpAxis` + `RenderScale` into a single `(plane: Vec4, enabled:
// u32)` tuple in RENDER FRAME, then pushes that tuple to every
// `ClipPlaneMaterial` asset whose `extension` differs from it.
//
// Two early-out guards keep per-frame work cheap:
//
// 1. **Snapshot-and-compare on the target uniform** via
//    `Local<Option<UniformKey>>` — same posture as
//    `update_layer_meshes` + `invalidate_on_geometry_change` (and
//    the [[project-bevy-is-changed-footgun]] memo). Avoids re-pushing
//    when state + centerline + camera unchanged.
// 2. **AssetEvent::Added watcher** for `ClipPlaneMaterial`: catches
//    new materials spawned mid-arc (layer-rebuild spawns fresh
//    materials with default zero-uniforms) so they get pushed even
//    when the key was already current. Without this, a layer added
//    while clipping is on would render unclipped until the next
//    state-change tick.

/// The render-frame uniform tuple the WGSL `ClipPlane` struct
/// consumes. Pure data so `Local<>` snapshotting is a trivial
/// equality check (no float-trap; computed deterministically from
/// the same inputs each frame).
#[derive(Debug, Clone, Copy, PartialEq)]
struct UniformKey {
    plane: Vec4,
    enabled: u32,
}

/// Project the resolved plane into render frame + pack into the
/// `(vec4, u32)` uniform layout. `None`-input (no centerline, plane
/// disabled, or polyline degenerate) collapses to `enabled = 0` and
/// a zero plane vector — the WGSL fragment shader's pass-through
/// path.
fn compute_uniform_key(
    state: &ClipPlaneState,
    centerline: &Centerline,
    up: UpAxis,
    render_scale: f32,
) -> UniformKey {
    let Some(resolved) = resolve_plane(state, centerline) else {
        return UniformKey {
            plane: Vec4::ZERO,
            enabled: 0,
        };
    };
    // Origin: physics-frame point → Bevy frame (UpAxis swap) →
    // render-frame (uniform scale). Matches the conversion
    // `draw_reference_overlays` applies to the centerline polyline,
    // so the clip plane stays anchored to the centerline at any
    // render scale.
    let origin_bevy = Vec3::from_array(up.to_bevy_point(&resolved.origin_m));
    let origin_render = origin_bevy * render_scale;
    // Normal: directions are scale-invariant. The UpAxis swap is
    // identical to the point swap (cf. `UpAxis::to_bevy_normal` doc);
    // we re-normalize defensively because the f64 → f32 cast can
    // introduce tiny rounding.
    let normal_bevy = Vec3::from_array(up.to_bevy_normal(&resolved.normal));
    let normal_render = normal_bevy.normalize_or_zero();
    let offset = origin_render.dot(normal_render);
    UniformKey {
        plane: Vec4::new(normal_render.x, normal_render.y, normal_render.z, offset),
        enabled: 1,
    }
}

// ============================================================
// Sub-leaf 5 — egui panel.
// ============================================================

/// Render the Clip Plane egui section. Surfaces the four user
/// controls (toggle, position along centerline, roll around tangent,
/// flip), all greyed-out + tooltipped when the loaded scan has no
/// centerline (i.e., `.prep.toml` was missing or its `[centerline]`
/// block was empty).
///
/// Caller passes whether the centerline is present so the disabled
/// state survives the slider being mutated without the resource
/// itself: a panel that read `centerline.points_m.is_empty()`
/// directly would still let the user touch the disabled controls;
/// the `enabled_section` guard around each widget is the actual
/// gate.
pub(crate) fn render_clip_plane_section(
    ui: &mut egui::Ui,
    state: &mut ClipPlaneState,
    centerline_available: bool,
) {
    egui::CollapsingHeader::new("Clip plane")
        .default_open(false)
        .show(ui, |ui| {
            if !centerline_available {
                ui.label("(needs a centerline — re-run cf-scan-prep on the source scan to emit a `.prep.toml`)");
                ui.add_enabled(false, egui::Checkbox::new(&mut state.enabled, "Enable clip plane"));
                return;
            }
            ui.checkbox(&mut state.enabled, "Enable clip plane");
            ui.add_enabled_ui(state.enabled, |ui| {
                let mut t_pct = state.t * 100.0;
                if ui
                    .add(egui::Slider::new(&mut t_pct, 0.0..=100.0).text("position along (%)"))
                    .changed()
                {
                    state.t = (t_pct * 0.01).clamp(0.0, 1.0);
                }
                let mut roll_deg = state.roll_rad.to_degrees();
                // Wrap into [0, 360) for the slider so the user sees a
                // single full revolution. The state stays in radians.
                let roll_max = 360.0_f64;
                roll_deg = roll_deg.rem_euclid(roll_max);
                if ui
                    .add(egui::Slider::new(&mut roll_deg, 0.0..=roll_max).text("roll (°)"))
                    .changed()
                {
                    state.roll_rad = roll_deg.to_radians();
                }
                if ui.button("Flip kept half").clicked() {
                    state.flip = !state.flip;
                }
                ui.label("(plane slides along centerline; rolls around it)");
            });
        });
}

#[allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value.
fn update_clip_plane_uniform(
    state: Res<ClipPlaneState>,
    centerline: Res<Centerline>,
    up: Res<UpAxis>,
    render_scale: Res<RenderScale>,
    mut materials: ResMut<Assets<ClipPlaneMaterial>>,
    mut last_key: Local<Option<UniformKey>>,
    mut asset_events: MessageReader<AssetEvent<ClipPlaneMaterial>>,
) {
    let target = compute_uniform_key(&state, &centerline, *up, render_scale.0);
    let new_assets = asset_events
        .read()
        .any(|event| matches!(event, AssetEvent::Added { .. }));
    if !new_assets && last_key.as_ref() == Some(&target) {
        return;
    }
    *last_key = Some(target);
    // Collect IDs first so the `iter()` borrow is released before
    // the `get_mut()` writes. With ≤ 8 materials in the scene
    // (cavity + ≤ 6 layers + scan), the allocation is trivial.
    let ids: Vec<_> = materials.iter().map(|(id, _)| id).collect();
    for id in ids {
        if let Some(material) = materials.get_mut(id) {
            material.extension.plane = target.plane;
            material.extension.enabled = target.enabled;
        }
    }
}

/// Clip-plane state. Anchored to the centerline; meaningless without
/// one (the slider and toggle disable themselves when
/// [`Centerline::points_m`] is empty — see sub-leaf 5's UI).
#[derive(Resource, Debug, Clone, Copy, PartialEq)]
pub(crate) struct ClipPlaneState {
    /// Master enable. `false` → [`resolve_plane`] returns `None`
    /// regardless of the other fields.
    pub enabled: bool,
    /// Arc-length-normalized position along the centerline polyline,
    /// clamped to `[0, 1]` by [`resolve_plane`]. The default
    /// [`DEFAULT_T`] picks the middle of the device — workshop case
    /// is "peek inside the middle first."
    pub t: f64,
    /// Rotation around the centerline tangent at `P(t)`, radians.
    /// Sub-leaf 5's UI exposes this as degrees in `[0°, 360°)`.
    pub roll_rad: f64,
    /// Swap which half of the world the plane keeps. Negates the
    /// plane normal before the WGSL fragment test.
    pub flip: bool,
}

impl Default for ClipPlaneState {
    fn default() -> Self {
        Self {
            enabled: false,
            t: DEFAULT_T,
            roll_rad: DEFAULT_ROLL_RAD,
            flip: false,
        }
    }
}

/// Default arc-length position along the centerline (midpoint).
pub(crate) const DEFAULT_T: f64 = 0.5;

/// Default roll around the tangent — plane normal aligned with the
/// projected-world-up reference, so the plane reads as "horizontal"
/// relative to the scan orientation by default.
pub(crate) const DEFAULT_ROLL_RAD: f64 = 0.0;

/// If the centerline tangent at `P(t)` is within this angle (degrees)
/// of world Z, fall back to world X as the up-reference for the
/// in-plane basis. Avoids `U_ref ≈ 0` when the tangent is nearly
/// parallel to world Z. 1° is generous; the iter-1 scan's centerline
/// runs along `+Z` so this branch is the common case.
pub(crate) const WORLD_Z_DEGENERATE_DEG: f64 = 1.0;

/// Resolved clip plane in PHYSICS-FRAME meters (NOT render-frame).
///
/// The render-frame conversion (the same `UpAxis::PlusZ` swap +
/// `RenderScale` lift `draw_reference_overlays` applies to the
/// centerline overlay) happens at the uniform-push boundary in
/// sub-leaf 4 — this struct stays in the same coordinate system as
/// [`Centerline::points_m`] so the math is easy to test.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct ResolvedPlane {
    pub origin_m: Point3<f64>,
    pub normal: Vector3<f64>,
}

/// Resolve a clip plane from `state` + the loaded `centerline`.
///
/// Returns `None` when:
///
/// - `state.enabled == false`.
/// - `centerline.points_m.len() < 2` (no tangent is definable).
///
/// `state.t` is clamped to `[0, 1]` before the arc-length lookup, so
/// callers don't need to pre-validate slider input.
///
/// The plane's normal is perpendicular to the centerline tangent at
/// `P(t)` BY CONSTRUCTION, which means the plane CONTAINS the tangent
/// — the longways-split UX the spec pinned. `state.roll_rad` rotates
/// the normal around the tangent. `state.flip` negates it (swaps
/// which half of the world the WGSL fragment shader keeps).
pub(crate) fn resolve_plane(
    state: &ClipPlaneState,
    centerline: &Centerline,
) -> Option<ResolvedPlane> {
    if !state.enabled {
        return None;
    }
    let pts = centerline.points_m.as_slice();
    if pts.len() < 2 {
        return None;
    }

    let total_arc = pts
        .windows(2)
        .map(|pair| (pair[1] - pair[0]).norm())
        .sum::<f64>();
    if total_arc <= 0.0 {
        // Defensive: a polyline with duplicate points has no defined
        // tangent direction. Treat the same as "too few points."
        return None;
    }
    let t_clamped = state.t.clamp(0.0, 1.0);
    let target_arc = t_clamped * total_arc;

    // Walk the polyline accumulating arc-length until `target_arc`
    // lies inside the current segment, then lerp within that segment
    // for the plane origin and take the segment's tangent.
    let mut traveled = 0.0_f64;
    let mut origin_m = pts[pts.len() - 1];
    let mut tangent = (pts[pts.len() - 1] - pts[pts.len() - 2]).normalize();
    for pair in pts.windows(2) {
        let seg = pair[1] - pair[0];
        let seg_len = seg.norm();
        if seg_len <= 0.0 {
            continue;
        }
        if traveled + seg_len >= target_arc {
            let local = (target_arc - traveled) / seg_len;
            origin_m = pair[0] + seg * local;
            tangent = seg / seg_len;
            break;
        }
        traveled += seg_len;
    }

    // In-plane basis perpendicular to the tangent. World Z is the
    // first-choice up-reference; if the tangent is within
    // [`WORLD_Z_DEGENERATE_DEG`] of world Z, world X takes over.
    let world_z = Vector3::new(0.0, 0.0, 1.0);
    let world_x = Vector3::new(1.0, 0.0, 0.0);
    let degenerate_cos = WORLD_Z_DEGENERATE_DEG.to_radians().cos();
    let raw_reference = if tangent.dot(&world_z).abs() < degenerate_cos {
        world_z
    } else {
        world_x
    };
    // Gram-Schmidt: project the reference onto the plane
    // perpendicular to the tangent, then normalize.
    let u_ref = (raw_reference - tangent * tangent.dot(&raw_reference)).normalize();
    let binormal = tangent.cross(&u_ref);

    let mut normal = u_ref * state.roll_rad.cos() + binormal * state.roll_rad.sin();
    if state.flip {
        normal = -normal;
    }
    Some(ResolvedPlane { origin_m, normal })
}

#[cfg(test)]
mod tests {
    // `unwrap()` + `expect()` are denied at the crate level for
    // production safety; allow them inside tests so assertions can
    // pull values out of `Option` / `Result` returns without
    // multi-line `match` ceremony. Matches the posture in
    // `main.rs::tests`.
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use super::*;

    const TOL: f64 = 1e-9;

    fn centerline_straight_z() -> Centerline {
        Centerline {
            points_m: vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(0.0, 0.0, 0.5),
                Point3::new(0.0, 0.0, 1.0),
            ],
        }
    }

    fn enabled_state() -> ClipPlaneState {
        ClipPlaneState {
            enabled: true,
            ..ClipPlaneState::default()
        }
    }

    #[test]
    fn resolve_plane_straight_centerline_default() {
        let plane = resolve_plane(&enabled_state(), &centerline_straight_z()).unwrap();
        // Origin at the midpoint by arc length.
        assert!(
            (plane.origin_m - Point3::new(0.0, 0.0, 0.5)).norm() < TOL,
            "origin = {:?}",
            plane.origin_m
        );
        // Tangent points along +Z → U_ref falls back to world X.
        // Roll = 0, flip = false → normal == U_ref == (1, 0, 0).
        assert!(
            (plane.normal - Vector3::new(1.0, 0.0, 0.0)).norm() < TOL,
            "normal = {:?}",
            plane.normal
        );
        // Plane normal is perpendicular to tangent.
        let tangent = Vector3::new(0.0, 0.0, 1.0);
        assert!(plane.normal.dot(&tangent).abs() < TOL);
    }

    #[test]
    fn resolve_plane_roll_90_rotates_normal_90_around_tangent() {
        let state = ClipPlaneState {
            roll_rad: std::f64::consts::FRAC_PI_2,
            ..enabled_state()
        };
        let plane = resolve_plane(&state, &centerline_straight_z()).unwrap();
        // U_ref = world X, binormal = Z × X = +Y. roll = +π/2 →
        // normal = cos(π/2)·X + sin(π/2)·Y = (0, 1, 0).
        assert!(
            (plane.normal - Vector3::new(0.0, 1.0, 0.0)).norm() < TOL,
            "normal = {:?}",
            plane.normal
        );
    }

    #[test]
    fn resolve_plane_flip_negates_normal() {
        let baseline = resolve_plane(&enabled_state(), &centerline_straight_z()).unwrap();
        let state = ClipPlaneState {
            flip: true,
            ..enabled_state()
        };
        let flipped = resolve_plane(&state, &centerline_straight_z()).unwrap();
        assert!((flipped.normal + baseline.normal).norm() < TOL);
        // Origin is the same either way.
        assert!((flipped.origin_m - baseline.origin_m).norm() < TOL);
    }

    #[test]
    fn resolve_plane_curved_centerline_picks_local_segment() {
        // Kinked polyline: 1 m along +Z, then 1 m along +X (total arc = 2).
        // `t = 0.6` → arc = 1.2, lands 0.2 m INTO the second segment
        // (along +X). Both origin (clearly past the joint) and
        // tangent (= +X, not +Z) come from the second segment, so
        // the discontinuity at the joint is exercised cleanly.
        let centerline = Centerline {
            points_m: vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(0.0, 0.0, 1.0),
                Point3::new(1.0, 0.0, 1.0),
            ],
        };
        let state = ClipPlaneState {
            t: 0.6,
            ..enabled_state()
        };
        let plane = resolve_plane(&state, &centerline).unwrap();
        assert!(
            (plane.origin_m - Point3::new(0.2, 0.0, 1.0)).norm() < TOL,
            "origin = {:?}",
            plane.origin_m
        );
        // Tangent along +X → world Z is NOT degenerate, so U_ref = Z.
        // roll = 0 → normal = U_ref = (0, 0, 1).
        assert!(
            (plane.normal - Vector3::new(0.0, 0.0, 1.0)).norm() < TOL,
            "normal = {:?}",
            plane.normal
        );
    }

    #[test]
    fn resolve_plane_at_segment_joint_picks_first_containing_segment() {
        // Document the tie-breaking at exact segment boundaries: when
        // `t·L` lands exactly on the joint between two segments, the
        // FIRST segment that contains the arc-length point wins
        // (`traveled + seg_len >= target_arc` triggers on the first
        // segment when sum equals target). This matches the spec's
        // "polyline tangent jumps at vertices" note — pick a
        // well-defined branch and document it.
        let centerline = Centerline {
            points_m: vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(0.0, 0.0, 1.0),
                Point3::new(1.0, 0.0, 1.0),
            ],
        };
        // `t = 0.5` → target_arc = 1.0 = exact joint.
        let plane = resolve_plane(&enabled_state(), &centerline).unwrap();
        assert!((plane.origin_m - Point3::new(0.0, 0.0, 1.0)).norm() < TOL);
        // Tangent inherits from the FIRST segment (+Z) — degeneracy
        // branch fires, U_ref = world X, normal = +X.
        assert!(
            (plane.normal - Vector3::new(1.0, 0.0, 0.0)).norm() < TOL,
            "normal = {:?}",
            plane.normal
        );
    }

    #[test]
    fn resolve_plane_tangent_near_world_z_uses_x_fallback() {
        // The straight-Z centerline above hits the degeneracy branch.
        // Confirm the resulting U_ref is world X (not zero, not NaN).
        let plane = resolve_plane(&enabled_state(), &centerline_straight_z()).unwrap();
        let n = plane.normal;
        assert!(n.x.is_finite() && n.y.is_finite() && n.z.is_finite());
        assert!(n.norm() > 0.99 && n.norm() < 1.01);
        // roll = 0 + tangent ≈ +Z + fallback U_ref = +X → normal = +X.
        assert!(n.x > 0.99, "normal = {n:?}");
    }

    #[test]
    fn resolve_plane_empty_centerline_returns_none() {
        let empty = Centerline::default();
        assert!(resolve_plane(&enabled_state(), &empty).is_none());
    }

    #[test]
    fn resolve_plane_single_point_centerline_returns_none() {
        let one = Centerline {
            points_m: vec![Point3::origin()],
        };
        assert!(resolve_plane(&enabled_state(), &one).is_none());
    }

    #[test]
    fn resolve_plane_disabled_returns_none() {
        let state = ClipPlaneState {
            enabled: false,
            ..enabled_state()
        };
        assert!(resolve_plane(&state, &centerline_straight_z()).is_none());
    }

    #[test]
    fn resolve_plane_t_clamped_below_zero() {
        let state = ClipPlaneState {
            t: -0.5,
            ..enabled_state()
        };
        let plane = resolve_plane(&state, &centerline_straight_z()).unwrap();
        // Clamped to t = 0 → origin at the first polyline point.
        assert!(
            (plane.origin_m - Point3::new(0.0, 0.0, 0.0)).norm() < TOL,
            "origin = {:?}",
            plane.origin_m
        );
    }

    #[test]
    fn resolve_plane_t_clamped_above_one() {
        let state = ClipPlaneState {
            t: 1.5,
            ..enabled_state()
        };
        let plane = resolve_plane(&state, &centerline_straight_z()).unwrap();
        // Clamped to t = 1 → origin at the last polyline point.
        assert!(
            (plane.origin_m - Point3::new(0.0, 0.0, 1.0)).norm() < TOL,
            "origin = {:?}",
            plane.origin_m
        );
    }

    #[test]
    fn compute_uniform_key_disabled_packs_zero_plane() {
        let key = compute_uniform_key(
            &ClipPlaneState::default(),
            &centerline_straight_z(),
            UpAxis::PlusZ,
            1.0,
        );
        assert_eq!(key.enabled, 0);
        assert_eq!(key.plane, Vec4::ZERO);
    }

    #[test]
    fn compute_uniform_key_applies_up_axis_swap() {
        // Straight-Z centerline (physics frame), enabled, default
        // roll. UpAxis::PlusZ swaps Y↔Z, so the physics-Z tangent
        // becomes Bevy-Y; the physics-X up reference stays at
        // Bevy-X; therefore normal stays Bevy-X = (1, 0, 0).
        let key = compute_uniform_key(
            &enabled_state(),
            &centerline_straight_z(),
            UpAxis::PlusZ,
            1.0,
        );
        assert_eq!(key.enabled, 1);
        // Plane normal (.xyz) = +Bevy-X.
        assert!((key.plane.x - 1.0).abs() < 1e-6, "key = {key:?}");
        assert!(key.plane.y.abs() < 1e-6);
        assert!(key.plane.z.abs() < 1e-6);
        // Origin (physics 0,0,0.5) → Bevy (0,0.5,0). Plane offset
        // `w = dot(origin_render, normal_render) = 0`.
        assert!(key.plane.w.abs() < 1e-6, "key = {key:?}");
    }

    #[test]
    fn compute_uniform_key_render_scale_lifts_offset() {
        // Straight-Z centerline (origin spans 0..1m). Shift the
        // origin off the plane by giving the slider a non-X plane
        // (use roll = π/2 so normal becomes Bevy-Z, the swapped
        // physics-Y) and a 10× scale. The plane equation's offset
        // `w = dot(origin_render, normal_render)` must scale with
        // the origin lift.
        let state = ClipPlaneState {
            roll_rad: std::f64::consts::FRAC_PI_2,
            ..enabled_state()
        };
        // Centerline offset from origin so the lift is visible: a
        // straight-Z polyline shifted by +0.2 m along physics Y.
        let centerline = Centerline {
            points_m: vec![Point3::new(0.0, 0.2, 0.0), Point3::new(0.0, 0.2, 1.0)],
        };
        // physics tangent = +Z, U_ref = +X (fallback), binormal = Z×X = +Y.
        // roll = π/2 → normal_physics = cos(π/2)·X + sin(π/2)·Y = +Y.
        // UpAxis::PlusZ swap on +Y normal → Bevy (0, 0, 1) = +Z.
        // origin physics (0, 0.2, 0.5) → Bevy (0, 0.5, 0.2).
        // With render_scale = 10, origin_render = (0, 5, 2).
        // w = dot((0,5,2), (0,0,1)) = 2.
        let key_1x = compute_uniform_key(&state, &centerline, UpAxis::PlusZ, 1.0);
        let key_10x = compute_uniform_key(&state, &centerline, UpAxis::PlusZ, 10.0);
        // Normals are scale-invariant.
        assert!(
            (key_1x.plane.truncate() - key_10x.plane.truncate()).length() < 1e-6,
            "normal diverged across scales: {key_1x:?} vs {key_10x:?}"
        );
        // Offsets scale linearly with render_scale.
        assert!(
            (key_10x.plane.w - 10.0 * key_1x.plane.w).abs() < 1e-5,
            "offset did not scale: {key_1x:?} vs {key_10x:?}"
        );
        // Spot-check the 10× absolute offset.
        assert!((key_10x.plane.w - 2.0).abs() < 1e-5, "key = {key_10x:?}");
    }

    #[test]
    fn resolve_plane_arc_length_parameterization_lands_by_arc_not_index() {
        // Three points, two segments of UNEQUAL length: a short
        // 0.25 m hop along +Z, then a long 0.75 m hop along +Z. Total
        // arc = 1.0; `t = 0.5` should land at arc = 0.5 m → 2/3 of
        // the way into the SECOND segment (not the midpoint by
        // index, which would be the joint at z = 0.25).
        let centerline = Centerline {
            points_m: vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(0.0, 0.0, 0.25),
                Point3::new(0.0, 0.0, 1.0),
            ],
        };
        let plane = resolve_plane(&enabled_state(), &centerline).unwrap();
        assert!(
            (plane.origin_m - Point3::new(0.0, 0.0, 0.5)).norm() < TOL,
            "origin = {:?}",
            plane.origin_m
        );
    }
}
