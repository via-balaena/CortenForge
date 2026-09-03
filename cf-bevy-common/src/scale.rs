//! Render-side scale lift for sub-meter scenes.
//!
//! [`OrbitCamera::framing_for_aabb`] clamps the AABB diagonal to `max(1.0)`, so
//! every scene 1 m or smaller is framed from one distance — `1.5·√3 ≈ 2.6 m`,
//! where a 5 cm object spans about 2° of the view. Scaling the *rendered*
//! geometry to a 1 m diagonal fills that fixed frame; the camera never moves.
//!
//! ⚠ Rendering only. A lifted value is in a different frame from the mesh it
//! came from: never write one to a file or report it to the user.
//!
//! [`OrbitCamera::framing_for_aabb`]: crate::camera::OrbitCamera::framing_for_aabb

use bevy::prelude::{Resource, Transform, Vec3};
use mesh_types::{Aabb, Point3};

/// The factor every spawned entity and rendered-frame system must apply.
/// `1.0` for metre-scale scenes — a true no-op.
#[derive(Resource, Clone, Copy, Debug)]
pub struct RenderScale(pub f32);

/// The factor that lifts `raw_diagonal` to the 1 m target; `1.0` at metre scale
/// or on a degenerate diagonal, leaving the framing helper's own clamp to cope.
#[must_use]
pub fn compute_render_scale(raw_diagonal: f32) -> f32 {
    const TARGET_DIAGONAL: f32 = 1.0;
    if !raw_diagonal.is_finite() || raw_diagonal <= 0.0 || raw_diagonal >= TARGET_DIAGONAL {
        1.0
    } else {
        TARGET_DIAGONAL / raw_diagonal
    }
}

/// The camera-framing AABB at render scale — distinct from the mesh's
/// physics-scale AABB.
#[must_use]
pub fn scale_aabb(raw: &Aabb, scale: f32) -> Aabb {
    let s = f64::from(scale);
    Aabb::from_corners(
        Point3::new(raw.min.x * s, raw.min.y * s, raw.min.z * s),
        Point3::new(raw.max.x * s, raw.max.y * s, raw.max.z * s),
    )
}

/// The transform that applies the lift to a spawned entity.
///
/// Deriving a scale without applying it leaves a correctly-framed camera over
/// an unscaled mesh — the defect this module exists for.
#[must_use]
pub fn render_transform(scale: f32) -> Transform {
    Transform::from_scale(Vec3::splat(scale))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::axis::UpAxis;
    use crate::camera::OrbitCamera;

    #[test]
    fn compute_render_scale_lifts_sub_meter_to_one() {
        assert!((compute_render_scale(0.05) - 20.0).abs() < 1e-6);
        assert!((compute_render_scale(0.5) - 2.0).abs() < 1e-6);
    }

    /// The `1 m` boundary is in the no-lift regime — the impl gates on `>=`.
    #[test]
    fn compute_render_scale_passthrough_at_meter_plus() {
        assert_eq!(compute_render_scale(1.0), 1.0);
        assert_eq!(compute_render_scale(2.5), 1.0);
        assert_eq!(compute_render_scale(100.0), 1.0);
    }

    #[test]
    fn compute_render_scale_falls_back_to_one_on_degenerate() {
        assert_eq!(compute_render_scale(0.0), 1.0);
        assert_eq!(compute_render_scale(-1.0), 1.0);
        assert_eq!(compute_render_scale(f32::NAN), 1.0);
        assert_eq!(compute_render_scale(f32::INFINITY), 1.0);
    }

    /// Load-bearing for metre-scale scenes, where the factor is `1.0`.
    #[test]
    fn scale_aabb_identity_at_unit_scale() {
        let raw = Aabb::from_corners(Point3::new(-0.1, -0.2, -0.3), Point3::new(0.4, 0.5, 0.6));
        let scaled = scale_aabb(&raw, 1.0);
        assert_eq!(scaled.min, raw.min);
        assert_eq!(scaled.max, raw.max);
    }

    #[test]
    fn scale_aabb_scales_corners_linearly() {
        let raw = Aabb::from_corners(
            Point3::new(-0.05, -0.05, -0.05),
            Point3::new(0.05, 0.05, 0.05),
        );
        let scaled = scale_aabb(&raw, 10.0);
        assert!((scaled.min.x - -0.5).abs() < 1e-9);
        assert!((scaled.max.z - 0.5).abs() < 1e-9);
        assert!((scaled.diagonal() - raw.diagonal() * 10.0).abs() < 1e-9);
    }

    /// The factor reaches the geometry as a uniform scale on all three axes.
    #[test]
    fn render_transform_scales_uniformly() {
        assert_eq!(render_transform(11.5).scale, Vec3::splat(11.5));
        assert_eq!(render_transform(1.0).scale, Vec3::ONE);
    }

    /// `framing_for_aabb`'s `max(1.0)` clamp pins a 5 cm scene and that same
    /// scene lifted to one distance. Only the geometry's size in that fixed
    /// frame changes.
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
