//! Workshop platform support slab for assembled molds.
//!
//! Pre-S4 (post-S6 of the prior mating-features arc) the assembled
//! mold's plug carried a T-bar lock at the pour-end pin tip
//! protruding **through** the cup wall at the cap-plane. The
//! `platform.stl` carried a complementary blind pocket sized to
//! clear the T-bar so the cup outer face could rest flat on the
//! slab top during pour + cure.
//!
//! Recon-1 §G-1 (2026-05-24) redesigned the plug retention
//! mechanism as a truncated-pyramid press-fit INTERNAL to the mold
//! cavity (the new lock lives recessed in the cup-piece cap-plane
//! interior surface — no through-hole, no protrusion past the cup
//! outer face). With no T-bar protrusion, the platform's blind
//! pocket has no load-bearing role to play. S4 of the FDM-friendly
//! geometry implementation arc retires the pocket and keeps the
//! platform as a **flat support slab** — workshop user sets the
//! assembled mold on it during pour + cure for level + spill
//! catchment; the cup outer face rests flat on the slab top with
//! no special geometry on either side.
//!
//! Keeping the platform STL (rather than retiring the artifact
//! entirely) preserves the 11-STL count + workshop muscle memory
//! (the platform-on-bench step survives unchanged) while costing
//! ~one extra second per regen and ~10 g of plastic per print
//! across the cast.
//!
//! ## Geometry
//!
//! - **Slab**: cuboid sized to the bounding region's XY AABB plus
//!   [`PLATFORM_XY_SLACK_M`] on each side (5 mm thumb-room).
//!   Thickness fixed at [`PLATFORM_MIN_THICKNESS_M`] (10 mm — FDM-
//!   printable rigidity without excessive plastic). Top face just
//!   below the bounding region's outer bottom face (with a small
//!   [`PLATFORM_TOP_GAP_M`] separation to keep the slab from
//!   cosmetically merging with the cup outer face under MC).
//! - **Final solid**: bare slab (no pocket carve).

use cf_design::Solid;
use nalgebra::Vector3;

use crate::mesh_csg::MatingTransform;
use crate::plug::PlugPinKind;
use crate::ribbon::Ribbon;

/// XY slack added to the platform's footprint relative to the
/// bounding region's XY AABB.
///
/// 5 mm of platform material extends past the cup outer face on
/// every side, giving the workshop user thumb room when placing
/// the assembled mold + a margin against cup-wall-MC stair-step
/// extending into the slab top.
pub const PLATFORM_XY_SLACK_M: f64 = 0.005;

/// Platform slab thickness (m).
///
/// 10 mm gives FDM-printable rigidity without excessive plastic.
/// Pre-S4 this was a floor (auto-bumped when needed to host the
/// blind T-bar pocket); post-S4 there is no pocket so the slab
/// ships at this thickness directly.
pub const PLATFORM_MIN_THICKNESS_M: f64 = 0.010;

/// Gap between the platform's top face and the bounding region's
/// outer bottom face (m).
///
/// 0.5 mm — well above FDM stair-step, keeps the platform from
/// cosmetically merging with the cup outer face under MC. The
/// assembled mold's cup-floor outer face rests on the platform's
/// top face (with the 0.5 mm of "floating" being negligible at
/// workshop scale).
pub const PLATFORM_TOP_GAP_M: f64 = 0.0005;

/// Build the workshop platform [`Solid`] for the given ribbon, or
/// `None` if the ribbon doesn't enable plug pins (and so doesn't
/// need a platform).
///
/// Returns `None` when:
/// - `ribbon.plug_pins` is [`PlugPinKind::None`] (no plug-floor
///   lock → workshop user hand-positions the plug, no need for a
///   slab).
/// - The bounding region's AABB is unbounded.
///
/// Post-S4 the platform is a bare flat slab — no pocket carve, no
/// dependency on the plug-lock geometry. The returned
/// `Vec<MatingTransform>` is always empty (no post-MC mesh-CSG ops).
#[must_use]
pub fn build_platform_solid(
    bounding_region: &Solid,
    ribbon: &Ribbon,
) -> Option<(Solid, Vec<MatingTransform>)> {
    // Pre-S4 the platform's purpose was to clear the T-bar
    // protrusion; post-S4 the lock is internal to the cavity so
    // there's no protrusion. The platform now serves only as a
    // workshop bench-protect slab — emit it whenever plug pins are
    // enabled (preserves the 11-STL count + workshop muscle
    // memory); skip it for ribbons without plug pins (the workshop
    // doesn't need a platform-shaped cup-floor reference if the
    // plug isn't being installed).
    if matches!(ribbon.plug_pins, PlugPinKind::None) {
        return None;
    }

    let bounding_aabb = bounding_region.bounds()?;
    let bound_size = bounding_aabb.size();
    let bound_center = bounding_aabb.center();

    // Slab top: just below the bounding region's outer bottom face,
    // so the cup-floor outer surface rests on the slab top with a
    // negligible 0.5 mm gap.
    let slab_top_z = bounding_aabb.min.z - PLATFORM_TOP_GAP_M;
    let slab_half_x = bound_size.x / 2.0 + PLATFORM_XY_SLACK_M;
    let slab_half_y = bound_size.y / 2.0 + PLATFORM_XY_SLACK_M;
    let slab_half_z = PLATFORM_MIN_THICKNESS_M / 2.0;
    let slab_z_center = slab_top_z - slab_half_z;

    let slab = Solid::cuboid(Vector3::new(slab_half_x, slab_half_y, slab_half_z))
        .translate(Vector3::new(bound_center.x, bound_center.y, slab_z_center));

    Some((slab, Vec::new()))
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

    use super::*;
    use crate::plug::{PlugPinKind, PlugPinSpec};
    use crate::ribbon::{Ribbon, SplitNormal};
    use nalgebra::Point3;

    fn iter1_like_ribbon_with_plug_lock() -> Ribbon {
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

    /// Post-S4 the platform emits as soon as plug pins are enabled
    /// — no `include_t_bar`-style sub-toggle anymore (the T-bar
    /// mechanism is gone; the platform is a workshop bench slab,
    /// not a T-bar pocket).
    #[test]
    fn build_platform_solid_emits_when_plug_pins_enabled() {
        let ribbon = iter1_like_ribbon_with_plug_lock();
        let bounding = Solid::cuboid(Vector3::new(0.061, 0.061, 0.090));
        let (platform, transforms) =
            build_platform_solid(&bounding, &ribbon).expect("plug pins enabled → platform emits");
        assert!(
            transforms.is_empty(),
            "post-S4 platform emits no mesh-CSG ops (pocket retired with the T-bar)",
        );
        // Slab body sanity: a probe at the slab's z-center +
        // bounding XY center reports interior.
        let probe = Point3::new(
            0.0,
            0.0,
            -0.090 - PLATFORM_TOP_GAP_M - PLATFORM_MIN_THICKNESS_M / 2.0,
        );
        assert!(
            platform.evaluate(&probe) < 0.0,
            "platform SDF at slab z-center must be interior; got {}",
            platform.evaluate(&probe),
        );
    }

    /// Post-S4 the platform is a bare slab — a probe ANYWHERE below
    /// the slab top within the bounded XY footprint must report
    /// interior (no pocket carve to remove material).
    #[test]
    fn build_platform_solid_is_a_bare_slab_with_no_pocket_carve() {
        let ribbon = iter1_like_ribbon_with_plug_lock();
        let bounding = Solid::cuboid(Vector3::new(0.061, 0.061, 0.090));
        let (platform, _) = build_platform_solid(&bounding, &ribbon).expect("platform must build");

        let slab_top_z = -0.090 - PLATFORM_TOP_GAP_M;
        // Probe at the slab's vertical midpoint directly under the
        // cap-plane centroid — pre-S4 this was the pocket interior
        // (SDF > 0); post-S4 it's solid slab material (SDF < 0).
        let probe_under_cap = Point3::new(0.0, 0.0, slab_top_z - PLATFORM_MIN_THICKNESS_M / 2.0);
        assert!(
            platform.evaluate(&probe_under_cap) < 0.0,
            "post-S4 slab must report interior under the cap-plane (no pocket carve); \
             got SDF = {}",
            platform.evaluate(&probe_under_cap),
        );
        // Above-slab probe (slab top + 1 mm) is OUTSIDE the slab.
        let probe_above = Point3::new(0.0, 0.0, slab_top_z + 0.001);
        assert!(
            platform.evaluate(&probe_above) > 0.0,
            "platform SDF just above slab top must report exterior; got {}",
            platform.evaluate(&probe_above),
        );
        // Below-slab probe (slab bottom - 1 mm) is OUTSIDE the slab.
        let probe_below = Point3::new(0.0, 0.0, slab_top_z - PLATFORM_MIN_THICKNESS_M - 0.001);
        assert!(
            platform.evaluate(&probe_below) > 0.0,
            "platform SDF just below slab bottom must report exterior; got {}",
            platform.evaluate(&probe_below),
        );
    }
}
