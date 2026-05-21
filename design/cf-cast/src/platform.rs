//! Workshop platform geometry for casts with T-bar-locked plug pins.
//!
//! When the ribbon's plug-pin kind has `include_t_bar = true`, the
//! plug's T-bar at the pin tip protrudes below the cup outer face
//! (see [`crate::plug::PlugPinSpec::include_t_bar`] for the
//! lock-vs-plastic-economy trade-off). [`build_platform_solid`]
//! generates a complementary printable slab with a rectangular
//! through-hole sized to clear the T-bar — workshop user sets the
//! assembled mold on the platform's top face; the T-bar hangs
//! through the hole below the slab; the cup outer face rests flat
//! on the slab top.
//!
//! ## Geometry
//!
//! - **Slab**: cuboid sized to the bounding region's XY AABB plus
//!   [`PLATFORM_XY_SLACK_M`] on each side; Z-thickness
//!   [`PLATFORM_THICKNESS_M`]. Positioned with the top face just
//!   below the bounding region's outer bottom face.
//! - **Through-hole**: cuboid sized to the T-bar's footprint plus
//!   [`PLATFORM_HOLE_LATERAL_SLACK_M`] on each side. Long axis
//!   aligned with the T-bar's axis (= ribbon binormal at the
//!   plug-pin anchor); short axis perpendicular within the slab
//!   plane. Cuboid Z-extent exceeds slab thickness so the hole
//!   passes completely through.
//! - **Final solid**: `slab.subtract(through_hole)`.

use cf_design::Solid;
use nalgebra::{UnitQuaternion, Vector3};

use crate::plug::pour_end_t_bar_geometry;
use crate::ribbon::Ribbon;

/// XY slack added to the platform's footprint relative to the
/// bounding region's XY AABB.
///
/// 5 mm of platform material extends past the cup outer face on
/// every side, giving the workshop user thumb room when placing
/// the assembled mold + a margin against cup-wall-MC stair-step
/// extending into the slab top.
pub const PLATFORM_XY_SLACK_M: f64 = 0.005;

/// Platform slab thickness (m). 10 mm gives FDM-printable rigidity
/// without excessive plastic. The slab is single-layer — workshop
/// can scale up post-print if it warps.
pub const PLATFORM_THICKNESS_M: f64 = 0.010;

/// Gap between the platform's top face and the bounding region's
/// outer bottom face (m). 0.5 mm — well above FDM stair-step,
/// keeps the platform from cosmetically merging with the cup
/// outer face under MC.
pub const PLATFORM_TOP_GAP_M: f64 = 0.0005;

/// Lateral slack between the T-bar's bounding box and the
/// platform's through-hole (m).
///
/// 2 mm of horizontal play on each side of the T-bar so workshop
/// user can drop the assembled mold into the platform without
/// fighting tight tolerance. The slack also absorbs FDM print
/// shrink (~0.2 mm) + any ribbon-binormal mis-alignment from a
/// not-quite-vertical cap-plane normal.
pub const PLATFORM_HOLE_LATERAL_SLACK_M: f64 = 0.002;

/// Extra Z-extent added to the through-hole beyond the slab's
/// thickness so the hole reliably passes all the way through (m).
///
/// 5 mm of overshoot above + below the slab guarantees MC produces
/// a clean through-hole regardless of slab-position MC stair-step.
pub const PLATFORM_HOLE_Z_OVERSHOOT_M: f64 = 0.005;

/// Build the workshop platform [`Solid`] for the given cast spec +
/// ribbon, or `None` if the ribbon's plug-pin kind doesn't
/// produce a T-bar protrusion (no platform needed).
///
/// The platform's through-hole is sized to the T-bar's bounding
/// footprint plus [`PLATFORM_HOLE_LATERAL_SLACK_M`] on each side,
/// aligned with the T-bar's axis (ribbon binormal at the pin
/// anchor). The hole spans the full slab thickness so the T-bar
/// hangs through the platform during workshop pour + cure.
///
/// Returns `None` when:
/// - The ribbon's plug-pin kind is
///   [`crate::plug::PlugPinKind::None`].
/// - `include_t_bar = false` on the ribbon's plug-pin spec.
/// - The bounding region's AABB is unbounded (the platform footprint
///   can't be computed).
/// - The T-bar's axis is degenerate (pin direction parallel to
///   `split_normal` — caught upstream by `Ribbon::new`).
#[must_use]
pub fn build_platform_solid(bounding_region: &Solid, ribbon: &Ribbon) -> Option<Solid> {
    let (t_bar_center, t_bar_axis, t_bar_radius_m, t_bar_half_length_m) =
        pour_end_t_bar_geometry(ribbon)?;

    let bounding_aabb = bounding_region.bounds()?;
    let bound_size = bounding_aabb.size();
    let bound_center = bounding_aabb.center();

    // Slab cuboid: bounding XY + slack, fixed Z thickness.
    let slab_half_x = bound_size.x / 2.0 + PLATFORM_XY_SLACK_M;
    let slab_half_y = bound_size.y / 2.0 + PLATFORM_XY_SLACK_M;
    let slab_half_z = PLATFORM_THICKNESS_M / 2.0;

    // Slab Z position: top face just below the bounding region's
    // outer bottom face.
    let slab_top_z = bounding_aabb.min.z - PLATFORM_TOP_GAP_M;
    let slab_z_center = slab_top_z - slab_half_z;

    let slab = Solid::cuboid(Vector3::new(slab_half_x, slab_half_y, slab_half_z))
        .translate(Vector3::new(bound_center.x, bound_center.y, slab_z_center));

    // Through-hole cuboid: local frame has Y-axis along T-bar axis
    // (long direction), X-axis perpendicular within the slab plane
    // (short direction = T-bar diameter + slack), Z-axis through
    // the slab (with overshoot for clean MC through-hole).
    let hole_half_x = t_bar_radius_m + PLATFORM_HOLE_LATERAL_SLACK_M;
    let hole_half_y = t_bar_half_length_m + PLATFORM_HOLE_LATERAL_SLACK_M;
    let hole_half_z = slab_half_z + PLATFORM_HOLE_Z_OVERSHOOT_M;

    let hole_local = Solid::cuboid(Vector3::new(hole_half_x, hole_half_y, hole_half_z));
    // Rotate the cuboid's local Y-axis onto the T-bar axis. Cuboid
    // is symmetric under 180° rotation around any of its principal
    // axes, so even if `rotation_between` resolves an antipodal
    // input (e.g., binormal = -Y in iter-1's frame) via its
    // arbitrary-perpendicular-axis fallback, the resulting cuboid
    // shape is identical.
    let rotation = UnitQuaternion::rotation_between(&Vector3::y_axis().into_inner(), &t_bar_axis)
        .unwrap_or_else(UnitQuaternion::identity);
    // Hole z-center aligned with slab z-center so the cuboid
    // straddles the slab evenly + the overshoot extends past both
    // faces.
    let hole = hole_local.rotate(rotation).translate(Vector3::new(
        t_bar_center.x,
        t_bar_center.y,
        slab_z_center,
    ));

    Some(slab.subtract(hole))
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

    use super::*;
    use crate::plug::{PlugPinKind, PlugPinSpec};
    use crate::ribbon::{Ribbon, SplitNormal};
    use nalgebra::Point3;

    fn iter1_like_ribbon() -> Ribbon {
        let centerline = vec![
            Point3::new(0.0, 0.0, 0.073),
            Point3::new(0.0, 0.0, 0.020),
            Point3::new(0.0, 0.0, -0.013),
        ];
        let split = SplitNormal::new(Vector3::new(1.0, 0.0, 0.0)).unwrap();
        let cap_centroid = Point3::new(0.0, 0.0, -0.054);
        let cap_normal = Vector3::new(0.0, 0.0, -1.0);
        Ribbon::new(centerline, split)
            .unwrap()
            .with_pour_end_hint(cap_centroid, cap_normal)
            .with_plug_pins(PlugPinKind::Axial(PlugPinSpec::iter1()))
    }

    #[test]
    fn build_platform_solid_returns_none_when_plug_pins_none() {
        let centerline = vec![Point3::new(-0.050, 0.0, 0.0), Point3::new(0.050, 0.0, 0.0)];
        let split = SplitNormal::new(Vector3::new(0.0, 1.0, 0.0)).unwrap();
        let ribbon = Ribbon::new(centerline, split).unwrap();
        let bounding = Solid::cuboid(Vector3::new(0.040, 0.040, 0.030));
        assert!(build_platform_solid(&bounding, &ribbon).is_none());
    }

    #[test]
    fn build_platform_solid_returns_none_when_include_t_bar_false() {
        let spec = PlugPinSpec {
            include_t_bar: false,
            ..PlugPinSpec::iter1()
        };
        let ribbon = {
            let centerline = vec![Point3::new(0.0, 0.0, 0.073), Point3::new(0.0, 0.0, -0.013)];
            let split = SplitNormal::new(Vector3::new(1.0, 0.0, 0.0)).unwrap();
            let cap_centroid = Point3::new(0.0, 0.0, -0.054);
            let cap_normal = Vector3::new(0.0, 0.0, -1.0);
            Ribbon::new(centerline, split)
                .unwrap()
                .with_pour_end_hint(cap_centroid, cap_normal)
                .with_plug_pins(PlugPinKind::Axial(spec))
        };
        let bounding = Solid::cuboid(Vector3::new(0.061, 0.061, 0.090));
        assert!(build_platform_solid(&bounding, &ribbon).is_none());
    }

    #[test]
    fn build_platform_solid_returns_some_for_iter1_like_fixture() {
        let ribbon = iter1_like_ribbon();
        let bounding = Solid::cuboid(Vector3::new(0.061, 0.061, 0.090))
            .translate(Vector3::new(0.0, 0.0, -0.020));
        let platform = build_platform_solid(&bounding, &ribbon).expect("platform should build");
        let aabb = platform.bounds().expect("platform solid has finite bounds");
        // Slab Z thickness 10 mm; top at bounding.min.z - 0.5 mm.
        // Bounding min.z = -0.020 - 0.090 = -0.110 → slab top at
        // -0.1105 m, slab spans z ∈ [-0.1205, -0.1105].
        assert!(
            aabb.max.z <= -0.110,
            "platform top should sit below bounding min.z = -0.110; got max.z = {}",
            aabb.max.z,
        );
        assert!(
            aabb.min.z >= -0.122,
            "platform bottom should not extend below z = -122 mm; got min.z = {}",
            aabb.min.z,
        );
        // Footprint extends past bounding XY (66 mm half-extent vs
        // 61 mm bounding half-extent).
        assert!(aabb.min.x <= -0.065);
        assert!(aabb.max.x >= 0.065);
    }

    #[test]
    fn build_platform_solid_hole_carves_t_bar_footprint() {
        let ribbon = iter1_like_ribbon();
        let bounding = Solid::cuboid(Vector3::new(0.061, 0.061, 0.090))
            .translate(Vector3::new(0.0, 0.0, -0.020));
        let platform = build_platform_solid(&bounding, &ribbon).expect("platform should build");

        // T-bar geometry: for iter-1-like ribbon, pour_end_t_bar
        // returns center at cap_centroid + pin_length * cap_normal
        // = (0, 0, -0.054) + 0.020*(0, 0, -1) = (0, 0, -0.074),
        // axis = cap_normal × split = -Z × +X = -Y. The platform
        // slab sits at z ∈ [-0.1205, -0.1105] — way below the T-bar
        // at z=-0.074. The hole is positioned at the slab z-center,
        // and its XY center matches the T-bar's XY (= (0, 0)).
        //
        // Probe at slab center XY (0, 0) within slab z range: with
        // the through-hole cut, the platform SDF should be > 0
        // (outside the platform — i.e., inside the hole).
        let q_hole_center = Point3::new(0.0, 0.0, -0.115);
        assert!(
            platform.evaluate(&q_hole_center) > 0.0,
            "platform SDF at hole center should be > 0 (outside platform = inside hole); \
             got {}",
            platform.evaluate(&q_hole_center),
        );

        // Probe far from the hole (corner of slab) should be INSIDE
        // the platform (SDF < 0).
        let q_far_corner = Point3::new(0.060, 0.060, -0.115);
        assert!(
            platform.evaluate(&q_far_corner) < 0.0,
            "platform SDF at far corner of slab should be < 0 (inside platform); got {}",
            platform.evaluate(&q_far_corner),
        );
    }
}
