//! Inter-piece registration features for v2 multi-piece molds.
//!
//! v2 iter-1 surfaced a workshop pain point: hand-clamping two
//! ribbon-cut mold pieces with rubber bands works but is fiddly,
//! and the pieces can shift along the seam during pour.
//! Registration features — geometry CSG'd into the mold pieces —
//! lock the pieces in alignment.
//!
//! Step 9 of `docs/CURVE_FOLLOWING_DESIGN.md` ships **cylindrical
//! pins** (vs the design-doc-§Open-questions §1 alternatives of
//! dovetails or magnets): simplest SDF (one cylinder per pin),
//! industry-standard for printed molds, easy to print + insert +
//! remove, gravity-held. Pin geometry:
//!
//! - Position: at `centerline_sample(arc_fraction) + offset *
//!   split_normal` so the pin sits in the cup wall (outside the
//!   body cavity, inside the bounding region).
//! - Axis: along the ribbon's binormal at that centerline position
//!   (perpendicular to the cutting plane).
//! - Length: 10 mm total (5 mm half-length each side of the ribbon)
//!   for the default — pin spans both pieces, half in each.
//! - Diameter: 3 mm (1.5 mm radius) for the default — beefy enough
//!   for FDM printing without supports, small enough to insert by
//!   hand with no tooling.
//! - Count: 2 per layer-piece-pair, at arc-length fractions
//!   `0.25` and `0.75` — prevents rotation around the centerline.
//!
//! ## Composition
//!
//! Pin cylinders are unioned into the [`crate::ribbon::PieceSide::Negative`]
//! piece's geometry (gain protrusions) and subtracted from the
//! [`crate::ribbon::PieceSide::Positive`] piece's geometry (gain
//! matching holes). The [`crate::piece::compose_piece_solid`]
//! function consults [`crate::ribbon::Ribbon::registration`] and
//! does the CSG inline; callers don't manage pin geometry
//! explicitly.
//!
//! ## Default off
//!
//! [`Ribbon::new`] sets `registration = RegistrationKind::None`
//! for backward compatibility — Steps 5-8 of the v2 arc all
//! pre-date this module, and no v2 test fixture or example crate
//! is expected to opt into pins without an explicit
//! [`crate::ribbon::Ribbon::with_registration`] call. The Step 11
//! example will flip this on.
//!
//! [`Ribbon::new`]: crate::ribbon::Ribbon::new

use cf_design::Solid;
use nalgebra::{UnitQuaternion, Vector3};

use crate::ribbon::Ribbon;

/// Cylindrical-pin registration spec. All dimensions in meters.
#[derive(Debug, Clone, PartialEq)]
pub struct PinSpec {
    /// Pin radius (m). Default `0.0015` = 1.5 mm = 3 mm diameter.
    pub pin_radius_m: f64,
    /// Half-length of each pin along the binormal axis (m).
    /// Default `0.005` = 5 mm → total pin length 10 mm.
    pub pin_half_length_m: f64,
    /// Offset from the centerline along the ribbon's split-normal
    /// direction (m).
    ///
    /// The pin's center sits at
    /// `centerline_sample(arc_fraction) + offset_from_centerline_m * split_normal_vec`.
    /// Default `0.025` = 25 mm — puts the pin in the cup wall for
    /// typical body radii (under 25 mm).
    pub offset_from_centerline_m: f64,
    /// Arc-length fractions along the centerline where pins are
    /// placed. Default `vec![0.25, 0.75]` — 2 pins per
    /// layer-piece-pair, evenly spaced, no rotation possible.
    pub arc_fractions: Vec<f64>,
}

impl PinSpec {
    /// v2 iter-1 defaults: 3 mm diameter × 10 mm long pins at 25% +
    /// 75% of centerline arc length, offset 25 mm from the
    /// centerline.
    #[must_use]
    pub fn iter1() -> Self {
        Self {
            pin_radius_m: 0.0015,
            pin_half_length_m: 0.005,
            offset_from_centerline_m: 0.025,
            arc_fractions: vec![0.25, 0.75],
        }
    }
}

impl Default for PinSpec {
    fn default() -> Self {
        Self::iter1()
    }
}

/// What registration mechanism, if any, the [`Ribbon`] uses.
///
/// `None` is the v1/v2-pre-Step-9 default — pieces clamp by hand
/// with rubber bands. `Pins(PinSpec)` enables Step 9's cylindrical
/// pins.
#[derive(Debug, Clone, Default, PartialEq)]
pub enum RegistrationKind {
    /// No registration features. Workshop hand-clamps the pieces;
    /// the v2 procedure.md surfaces the rubber-band approach.
    #[default]
    None,
    /// Cylindrical pins per [`PinSpec`].
    Pins(PinSpec),
}

/// Build the combined pin Solid for the ribbon's registration kind,
/// or `None` if registration is `None` or the pin spec has no arc
/// fractions.
///
/// Returns `Some(solid)` whose SDF is the union of all individual
/// pin cylinders (one cylinder per arc fraction in
/// [`PinSpec::arc_fractions`]). Each cylinder is positioned at
/// `centerline_sample(t) + offset * split_normal_vec` with axis
/// aligned to the ribbon's binormal at that centerline position.
///
/// The cylinders are deliberately NOT rotated to align with the
/// centerline's tangent — they're perpendicular to the seam, so
/// they extend equally into both [`crate::ribbon::PieceSide`] halves.
#[must_use]
pub fn build_registration_solid(ribbon: &Ribbon) -> Option<Solid> {
    let spec = match &ribbon.registration {
        RegistrationKind::None => return None,
        RegistrationKind::Pins(spec) => spec,
    };
    if spec.arc_fractions.is_empty() {
        return None;
    }
    let split_vec = ribbon.split_normal.as_vector();
    let mut cylinders = Vec::with_capacity(spec.arc_fractions.len());
    for &t in &spec.arc_fractions {
        let (center, _tangent, binormal) = ribbon.sample_at_arc_fraction(t)?;
        let pin_center = center + spec.offset_from_centerline_m * split_vec;
        let rotation = UnitQuaternion::rotation_between(&Vector3::z_axis().into_inner(), &binormal)
            .unwrap_or_else(UnitQuaternion::identity);
        let cyl = Solid::cylinder(spec.pin_radius_m, spec.pin_half_length_m)
            .rotate(rotation)
            .translate(pin_center.coords);
        cylinders.push(cyl);
    }
    // Union all pin cylinders. `Solid::union` is binary; chain via
    // fold. (No `union_all` helper in cf-design today; trivial loop.)
    let mut combined = cylinders.pop()?;
    for c in cylinders {
        combined = combined.union(c);
    }
    Some(combined)
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::panic, clippy::expect_used)]

    use super::*;
    use crate::ribbon::{Ribbon, SplitNormal};
    use nalgebra::Point3;

    fn straight_x_ribbon() -> Ribbon {
        let centerline = vec![Point3::new(-0.050, 0.0, 0.0), Point3::new(0.050, 0.0, 0.0)];
        let split = SplitNormal::new(Vector3::new(0.0, 1.0, 0.0)).unwrap();
        Ribbon::new(centerline, split).unwrap()
    }

    #[test]
    fn pin_spec_iter1_has_workshop_defaults() {
        let s = PinSpec::iter1();
        assert!((s.pin_radius_m - 0.0015).abs() < f64::EPSILON);
        assert!((s.pin_half_length_m - 0.005).abs() < f64::EPSILON);
        assert!((s.offset_from_centerline_m - 0.025).abs() < f64::EPSILON);
        assert_eq!(s.arc_fractions, vec![0.25, 0.75]);
    }

    #[test]
    fn registration_kind_default_is_none() {
        assert_eq!(RegistrationKind::default(), RegistrationKind::None);
    }

    #[test]
    fn build_registration_solid_none_returns_none() {
        let ribbon = straight_x_ribbon(); // default registration = None
        assert!(build_registration_solid(&ribbon).is_none());
    }

    #[test]
    fn build_registration_solid_pins_returns_some_with_finite_bounds() {
        let ribbon =
            straight_x_ribbon().with_registration(RegistrationKind::Pins(PinSpec::iter1()));
        let pins = build_registration_solid(&ribbon).expect("pin spec should yield a solid");
        let aabb = pins
            .bounds()
            .expect("pin cylinders should have finite bounds");
        // 2 pins along +X at 25%/75% of 100 mm centerline → x ∈
        // [-0.025, +0.025] (centers). Each pin offset +0.025 in y
        // and centered at z=0 with half-length 5 mm.
        // Pin radius 1.5 mm so x extent ~[-0.025-0.0015, +0.025+0.0015]
        assert!(aabb.min.x < 0.0); // first pin at -25 mm
        assert!(aabb.max.x > 0.0); // second pin at +25 mm
        // y around +25 mm (offset from centerline along +Y split-normal)
        assert!(aabb.min.y > 0.020);
        assert!(aabb.max.y < 0.030);
        // z extends pin_half_length each side
        assert!(aabb.min.z <= -0.005 + 1e-9);
        assert!(aabb.max.z >= 0.005 - 1e-9);
    }

    #[test]
    fn build_registration_solid_empty_arc_fractions_returns_none() {
        let mut spec = PinSpec::iter1();
        spec.arc_fractions.clear();
        let ribbon = straight_x_ribbon().with_registration(RegistrationKind::Pins(spec));
        assert!(build_registration_solid(&ribbon).is_none());
    }

    #[test]
    fn pin_cylinders_are_inside_only_at_pin_positions() {
        // Pin at arc-fraction 0.25 of straight +X centerline (-0.05 → +0.05):
        //   centerline_sample = (-0.025, 0, 0)
        //   pin position      = (-0.025, +0.025, 0)
        //   axis              = +Z (binormal of straight +X with +Y split)
        //   radius 1.5 mm, half-length 5 mm
        // Query the pin Solid at the pin center: SDF < 0 (inside).
        // Query far away: SDF > 0 (outside).
        let ribbon =
            straight_x_ribbon().with_registration(RegistrationKind::Pins(PinSpec::iter1()));
        let pins = build_registration_solid(&ribbon).unwrap();

        let pin_center = Point3::new(-0.025, 0.025, 0.0);
        assert!(
            pins.evaluate(&pin_center) < 0.0,
            "pin SDF at pin center should be negative; got {}",
            pins.evaluate(&pin_center),
        );

        let far = Point3::new(0.100, 0.100, 0.100);
        assert!(
            pins.evaluate(&far) > 0.0,
            "pin SDF far from pins should be positive; got {}",
            pins.evaluate(&far),
        );
    }
}
