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
//! The actual fragment-discard material extension + WGSL embedding
//! land in sub-leaf 2; the per-frame uniform push system in sub-leaf
//! 4; the egui panel in sub-leaf 5.
//!
//! Module-level `#![allow(dead_code)]` matches the `sdf_layers`
//! precedent: the resource + resolver land in sub-leaf 1 but the
//! consumers (the uniform-push system + the panel) only un-shade them
//! later. The allowance keeps the bin's compile clean at the in-flight
//! sub-leaf boundary.
#![allow(dead_code)]

use bevy::prelude::Resource;
use mesh_types::{Point3, Vector3};

use crate::Centerline;

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
