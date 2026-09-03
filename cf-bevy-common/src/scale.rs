//! Render-side scale lift for sub-meter scenes.
//!
//! [`OrbitCamera::framing_for_aabb`] clamps the AABB diagonal to `max(1.0)`, so
//! every scene 1 m or smaller is framed from one distance — `1.5·√3 ≈ 2.6 m`.
//! A 5 cm object there spans about 2° of the view.
//!
//! ⚠ The lift does **not** move the camera; `framing_for_aabb` returns the same
//! distance either way (`the_lift_does_not_move_the_camera` asserts it). What
//! changes is the geometry's size within that fixed frame.
//!
//! ⚠ **These functions only derive numbers.** The caller applies the factor to
//! the geometry — `Transform::from_scale(Vec3::splat(f))` — and to anything else
//! drawn in the same frame (gizmos, clip planes, pushed uniforms), which is what
//! [`RenderScale`] is for. Deriving without applying leaves a correctly-framed
//! camera over an unscaled mesh: the original defect.
//!
//! ⚠ Rendering only. A lifted value sits in a different frame from the mesh it
//! came from: never write one to a file or report it to the user.
//!
//! Pattern from sim-soft `EXAMPLE_INVENTORY` — iter-11 (row 12) pivoted to it,
//! iter-12 applied it in cf-view.
//!
//! [`OrbitCamera::framing_for_aabb`]: crate::camera::OrbitCamera::framing_for_aabb

use bevy::prelude::Resource;
use mesh_types::{Aabb, Point3};

/// The factor applied uniformly to all spawned geometry, so systems that
/// draw into the rendered frame can apply the same lift the mesh got.
///
/// `1.0` for scenes already at metre scale or larger — a true no-op.
#[derive(Resource, Clone, Copy, Debug)]
pub struct RenderScale(pub f32);

/// Compute the render scale from the raw bbox diagonal: lift sub-meter
/// scenes to a 1 m target diagonal; meter+ scenes render at native scale.
/// Degenerate (zero / non-finite) diagonals fall back to `1.0` so the
/// downstream framing helper's own clamp handles them.
#[must_use]
pub fn compute_render_scale(raw_diagonal: f32) -> f32 {
    const TARGET_DIAGONAL: f32 = 1.0;
    if !raw_diagonal.is_finite() || raw_diagonal <= 0.0 || raw_diagonal >= TARGET_DIAGONAL {
        1.0
    } else {
        TARGET_DIAGONAL / raw_diagonal
    }
}

/// Apply a uniform scale factor to an [`Aabb`]'s corners. Used to compute
/// the camera-framing AABB at render scale (the rendered geometry's
/// bbox), distinct from the loaded mesh's physics-scale AABB.
#[must_use]
pub fn scale_aabb(raw: &Aabb, scale: f32) -> Aabb {
    let s = f64::from(scale);
    Aabb::from_corners(
        Point3::new(raw.min.x * s, raw.min.y * s, raw.min.z * s),
        Point3::new(raw.max.x * s, raw.max.y * s, raw.max.z * s),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::axis::UpAxis;
    use crate::camera::OrbitCamera;

    /// Sub-meter scenes lift to the 1 m target: `0.05 m → 20×`.
    #[test]
    fn compute_render_scale_lifts_sub_meter_to_one() {
        assert!((compute_render_scale(0.05) - 20.0).abs() < 1e-6);
        assert!((compute_render_scale(0.5) - 2.0).abs() < 1e-6);
    }

    /// Meter+ scenes pass through unchanged. The `1 m` boundary itself is
    /// included in the no-lift regime (the impl gates on `>= TARGET`).
    #[test]
    fn compute_render_scale_passthrough_at_meter_plus() {
        assert_eq!(compute_render_scale(1.0), 1.0);
        assert_eq!(compute_render_scale(2.5), 1.0);
        assert_eq!(compute_render_scale(100.0), 1.0);
    }

    /// Degenerate inputs (zero, negative, NaN, Inf) fall back to `1.0` so
    /// the downstream framing helper's own `max(1.0)` clamp can handle the
    /// scene without dividing by zero here.
    #[test]
    fn compute_render_scale_falls_back_to_one_on_degenerate() {
        assert_eq!(compute_render_scale(0.0), 1.0);
        assert_eq!(compute_render_scale(-1.0), 1.0);
        assert_eq!(compute_render_scale(f32::NAN), 1.0);
        assert_eq!(compute_render_scale(f32::INFINITY), 1.0);
    }

    /// `scale = 1.0` is the identity (load-bearing for meter+ scenes
    /// where `compute_render_scale` returns `1.0`).
    #[test]
    fn scale_aabb_identity_at_unit_scale() {
        let raw = Aabb::from_corners(Point3::new(-0.1, -0.2, -0.3), Point3::new(0.4, 0.5, 0.6));
        let scaled = scale_aabb(&raw, 1.0);
        assert_eq!(scaled.min, raw.min);
        assert_eq!(scaled.max, raw.max);
    }

    /// Uniform scaling is corner-wise linear: `min × s` and `max × s`
    /// reach the rendered-frame bbox.
    #[test]
    fn scale_aabb_scales_corners_linearly() {
        let raw = Aabb::from_corners(
            Point3::new(-0.05, -0.05, -0.05),
            Point3::new(0.05, 0.05, 0.05),
        );
        let scaled = scale_aabb(&raw, 10.0);
        assert!((scaled.min.x - -0.5).abs() < 1e-9);
        assert!((scaled.max.z - 0.5).abs() < 1e-9);
        // Diagonal scales by the same factor.
        assert!((scaled.diagonal() - raw.diagonal() * 10.0).abs() < 1e-9);
    }

    /// The claim the module doc rests on, and the reason the lift is not a
    /// camera change: `framing_for_aabb`'s `max(1.0)` clamp pins a 5 cm scene
    /// and that same scene lifted to one distance. Only the geometry's size in
    /// that fixed frame changes.
    #[test]
    // f64 diagonal → the f32 the real call sites pass.
    #[allow(clippy::cast_possible_truncation)]
    fn the_lift_does_not_move_the_camera() {
        let raw = Aabb::from_corners(
            Point3::new(-0.025, -0.025, -0.025),
            Point3::new(0.025, 0.025, 0.025),
        );
        let lifted = scale_aabb(&raw, compute_render_scale(raw.diagonal() as f32));
        assert!(
            (lifted.diagonal() - 1.0).abs() < 1e-6,
            "the lift targets a 1 m diagonal",
        );

        let before = OrbitCamera::framing_for_aabb(&raw, UpAxis::PlusZ).distance;
        let after = OrbitCamera::framing_for_aabb(&lifted, UpAxis::PlusZ).distance;
        assert!(
            (before - after).abs() < 1e-4,
            "the clamp pins both to one distance ({before} vs {after}); if this \
             ever fails the clamp has changed, and whether this module is still \
             needed should be re-asked",
        );
    }
}
