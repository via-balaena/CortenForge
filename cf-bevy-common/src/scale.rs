//! Render-side scale lift for sub-meter scenes.
//!
//! [`OrbitCamera::framing_for_aabb`] clamps the AABB diagonal to `max(1.0)`, so
//! every scene 1 m or smaller is framed from one distance. Scaling the
//! *rendered* geometry to a 1 m diagonal fills that fixed frame; the camera
//! never moves.
//!
//! Everything here traffics in [`RenderScale`] rather than a bare `f32`, so a
//! length from the physics frame cannot be passed in by mistake.
//!
//! [`OrbitCamera::framing_for_aabb`]: crate::camera::OrbitCamera::framing_for_aabb

use bevy::prelude::{Resource, Transform, Vec3};
use mesh_types::{Aabb, Point3};

/// A render-frame scale factor. `1.0` for metre-scale scenes — a true no-op.
///
/// ⚠ The inner value is the *rendered* frame's factor. Multiplying a
/// physics-frame length by it produces a rendered length, and the two must not
/// be mixed: never write a lifted value to a file or report one to the user.
#[derive(Resource, Clone, Copy, Debug)]
pub struct RenderScale(pub f32);

impl core::fmt::Display for RenderScale {
    /// Forwards to the inner `f32`, so `{:.2}`-style precision works.
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        self.0.fmt(f)
    }
}

impl RenderScale {
    /// The lift that brings `raw_diagonal` to the 1 m target; `1.0` at metre
    /// scale or on a degenerate diagonal, leaving the framing helper's own
    /// clamp to cope.
    #[must_use]
    pub fn for_diagonal(raw_diagonal: f32) -> Self {
        const TARGET_DIAGONAL: f32 = 1.0;
        if !raw_diagonal.is_finite() || raw_diagonal <= 0.0 || raw_diagonal >= TARGET_DIAGONAL {
            Self(1.0)
        } else {
            Self(TARGET_DIAGONAL / raw_diagonal)
        }
    }

    /// The camera-framing AABB at this scale — distinct from the mesh's
    /// physics-scale AABB.
    #[must_use]
    pub fn framing_aabb(self, raw: &Aabb) -> Aabb {
        let s = f64::from(self.0);
        Aabb::from_corners(
            Point3::new(raw.min.x * s, raw.min.y * s, raw.min.z * s),
            Point3::new(raw.max.x * s, raw.max.y * s, raw.max.z * s),
        )
    }

    /// The transform that applies this lift to a spawned entity.
    ///
    /// Deriving a scale without applying it leaves a correctly-framed camera
    /// over an unscaled mesh — the defect this module exists for.
    #[must_use]
    pub fn transform(self) -> Transform {
        Transform::from_scale(Vec3::splat(self.0))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::axis::UpAxis;
    use crate::camera::OrbitCamera;

    #[test]
    fn for_diagonal_lifts_sub_meter_to_one() {
        assert!((RenderScale::for_diagonal(0.05).0 - 20.0).abs() < 1e-6);
        assert!((RenderScale::for_diagonal(0.5).0 - 2.0).abs() < 1e-6);
    }

    /// The `1 m` boundary is in the no-lift regime — the impl gates on `>=`.
    #[test]
    fn for_diagonal_passes_through_at_meter_plus() {
        assert_eq!(RenderScale::for_diagonal(1.0).0, 1.0);
        assert_eq!(RenderScale::for_diagonal(2.5).0, 1.0);
        assert_eq!(RenderScale::for_diagonal(100.0).0, 1.0);
    }

    #[test]
    fn for_diagonal_falls_back_to_one_on_degenerate() {
        assert_eq!(RenderScale::for_diagonal(0.0).0, 1.0);
        assert_eq!(RenderScale::for_diagonal(-1.0).0, 1.0);
        assert_eq!(RenderScale::for_diagonal(f32::NAN).0, 1.0);
        assert_eq!(RenderScale::for_diagonal(f32::INFINITY).0, 1.0);
    }

    /// Load-bearing for metre-scale scenes, where the factor is `1.0`.
    #[test]
    fn framing_aabb_is_identity_at_unit_scale() {
        let raw = Aabb::from_corners(Point3::new(-0.1, -0.2, -0.3), Point3::new(0.4, 0.5, 0.6));
        let scaled = RenderScale(1.0).framing_aabb(&raw);
        assert_eq!(scaled.min, raw.min);
        assert_eq!(scaled.max, raw.max);
    }

    #[test]
    fn framing_aabb_scales_corners_linearly() {
        let raw = Aabb::from_corners(
            Point3::new(-0.05, -0.05, -0.05),
            Point3::new(0.05, 0.05, 0.05),
        );
        let scaled = RenderScale(10.0).framing_aabb(&raw);
        assert!((scaled.min.x - -0.5).abs() < 1e-9);
        assert!((scaled.max.z - 0.5).abs() < 1e-9);
        assert!((scaled.diagonal() - raw.diagonal() * 10.0).abs() < 1e-9);
    }

    #[test]
    fn transform_scales_uniformly() {
        assert_eq!(RenderScale(11.5).transform().scale, Vec3::splat(11.5));
        assert_eq!(RenderScale(1.0).transform().scale, Vec3::ONE);
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
        let lifted = RenderScale::for_diagonal(raw.diagonal() as f32).framing_aabb(&raw);
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
