//! Render-side scale lift for sub-meter scenes.
//!
//! Bevy 0.18's rendering defaults — the `0.1 m` camera near plane,
//! [`OrbitCamera::framing_for_aabb`]'s internal `.max(1.0)` clamp on the
//! AABB diagonal, `AmbientLight` brightness — are tuned for human-scale
//! scenes. Geometry much smaller than a metre falls outside that regime:
//! the framing clamp treats a 5 cm scene as if it were 1 m and parks the
//! camera 1.5 m away, so it renders as a dot.
//!
//! The fix is uniform and cheap: scale the *rendered* geometry so its
//! diagonal reaches ~1 m, frame the camera on the scaled AABB, and leave
//! the physics-scale mesh untouched. [`compute_render_scale`] derives the
//! factor, [`scale_aabb`] produces the framing AABB, and [`RenderScale`]
//! carries the factor to every system that must apply the same lift —
//! gizmo overlays, clip planes, and pushed uniforms all live in the
//! rendered frame, not the physics frame.
//!
//! ⚠ **The lift is a rendering concern only.** Nothing written back to a
//! file, measured, or reported to the user may pass through it. A value
//! that has been scaled is in a different frame from the mesh it came
//! from, and mixing the two silently reports the wrong size.
//!
//! Banked as sim-soft `EXAMPLE_INVENTORY` iter-11 pattern (b); the same
//! root cause as sim-bevy-soft's row-12/row-13 `RENDER_SCALE` policy.
//!
//! [`OrbitCamera::framing_for_aabb`]: crate::camera::OrbitCamera::framing_for_aabb

use bevy::prelude::Resource;
use mesh_types::{Aabb, Point3};

/// The factor applied uniformly to all spawned geometry, so systems that
/// draw into the rendered frame can apply the same lift the mesh got.
///
/// `1.0` for scenes already at metre scale or larger — the common case for
/// mesh-v1.0 examples, and a true no-op.
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

    /// The pairing that matters: a lifted scene's framing AABB reaches the
    /// 1 m target, so `framing_for_aabb`'s clamp is no longer what decides
    /// the camera distance. Without the lift a 0.05 m scene and a 1 m scene
    /// frame identically — which is the defect this module exists for.
    #[test]
    // f64 diagonal → the f32 the real call sites pass.
    #[allow(clippy::cast_possible_truncation)]
    fn lifting_a_sub_meter_scene_clears_the_framing_clamp() {
        let raw = Aabb::from_corners(
            Point3::new(-0.025, -0.025, -0.025),
            Point3::new(0.025, 0.025, 0.025),
        );
        let scale = compute_render_scale(raw.diagonal() as f32);
        let lifted = scale_aabb(&raw, scale);
        // f32 tolerance, not f64: the scale factor is an `f32` because
        // that is what `RenderScale` and Bevy's `Transform` carry, so the
        // round trip through it is only good to ~1e-7.
        assert!(
            (lifted.diagonal() - 1.0).abs() < 1e-6,
            "the lift targets a 1 m diagonal, where the clamp is inert",
        );
    }
}
