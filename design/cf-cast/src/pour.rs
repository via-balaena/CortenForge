//! Pour-gate + air-vent geometry for v2 multi-piece molds.
//!
//! Step 10 of `docs/CURVE_FOLLOWING_DESIGN.md` adds two cylindrical
//! channels CSG'd off the v2 mold pieces:
//!
//! - **Pour gate** at `centerline[0]` (the base end of the
//!   centerline polyline) — silicone enters the cavity through
//!   this channel.
//! - **Air vent** at `centerline.last()` (the tip end) — trapped
//!   air escapes through this channel during pour + cure.
//!
//! Both channels run along the centerline tangent at their
//! respective endpoints. They're cylinders centered on the
//! endpoint, oriented along the tangent, sized to pierce the
//! bounding region wall once placed inside (real cf-scan-prep
//! output has centerline endpoints at the body's surface, just
//! inside the bounding region; the cylinder extends both inward
//! into the body cavity — harmless, already excluded by
//! `subtract(layer_body)` — and outward through the cup wall to
//! the outside world).
//!
//! ## Composition
//!
//! [`build_pour_gate_solid`] returns the union of the gate +
//! (optional) vent cylinders, or `None` if [`PourGateKind::None`].
//! [`crate::piece::compose_piece_solid`] subtracts this solid
//! from the base piece geometry, applying the same channel to
//! BOTH [`crate::ribbon::PieceSide`]s so the pour/vent paths are
//! centered on the ribbon seam (split equally between the two
//! pieces — workshop user pours straight along the centerline,
//! doesn't matter which piece is "up").
//!
//! ## Default off
//!
//! [`Ribbon::new`] sets `pour_gate = PourGateKind::None` for
//! backward compatibility with Steps 5-9 of the v2 arc. The
//! Step 11 example crate is expected to opt into
//! [`PourGateKind::Default`] via
//! [`crate::ribbon::Ribbon::with_pour_gate`].
//!
//! [`Ribbon::new`]: crate::ribbon::Ribbon::new

use cf_design::Solid;
use nalgebra::{UnitQuaternion, Vector3};

use crate::ribbon::Ribbon;

/// Cylindrical pour-gate + air-vent geometry spec. All dimensions
/// in meters.
#[derive(Debug, Clone, PartialEq)]
pub struct PourGateSpec {
    /// Pour-gate channel radius (m). Default `0.003` = 3 mm =
    /// 6 mm diameter — enough for low-viscosity silicone flow
    /// without splashing at typical workshop pour rates.
    pub gate_radius_m: f64,
    /// Air-vent channel radius (m). Default `0.0015` = 1.5 mm =
    /// 3 mm diameter — smaller than the pour gate; air escapes
    /// easily through small cross-sections, and silicone won't
    /// flow out of vents this size at workshop pour pressures
    /// (silicone surface tension + low pressure differential).
    pub vent_radius_m: f64,
    /// Half-length of each channel cylinder along its tangent
    /// axis (m). Total channel length is `2 * channel_half_length_m`.
    /// Default `0.020` = 20 mm half-length → 40 mm total channel.
    ///
    /// Cylinder is centered on the centerline endpoint, so the
    /// channel extends `channel_half_length_m` outward into the
    /// world AND `channel_half_length_m` inward toward the body
    /// cavity. The inward half is mostly redundant (the body
    /// cavity is already excluded by `subtract(layer_body)`), but
    /// it provides cell-size-tolerant overlap so marching cubes
    /// doesn't produce a thin annular wall at the body-channel
    /// interface.
    pub channel_half_length_m: f64,
    /// Whether to include the air-vent cylinder at
    /// `centerline.last()` (tip end). Default `true`. Disable for
    /// short straight molds where air escape through the gate is
    /// sufficient.
    pub include_vent: bool,
}

impl PourGateSpec {
    /// v2 iter-1 defaults: 6 mm Ø pour gate, 3 mm Ø air vent,
    /// 40 mm total channel length, vent enabled.
    #[must_use]
    pub const fn iter1() -> Self {
        Self {
            gate_radius_m: 0.003,
            vent_radius_m: 0.0015,
            channel_half_length_m: 0.020,
            include_vent: true,
        }
    }
}

impl Default for PourGateSpec {
    fn default() -> Self {
        Self::iter1()
    }
}

/// What pour-gate + vent geometry, if any, the [`Ribbon`] uses.
///
/// `None` is the v1/v2-pre-Step-10 default — no integral pour
/// gate or vent; the workshop user pours through the seam where
/// pieces meet and drills vent holes post-print as needed.
/// `Default(PourGateSpec)` enables Step 10's integrated channels.
#[derive(Debug, Clone, Default, PartialEq)]
pub enum PourGateKind {
    /// No pour-gate or vent channels.
    #[default]
    None,
    /// Pour gate at centerline base + optional vent at tip, per
    /// the supplied [`PourGateSpec`].
    Default(PourGateSpec),
}

/// Build the combined pour-gate + vent Solid for the ribbon's
/// pour-gate kind, or `None` if [`PourGateKind::None`].
///
/// Returns `Some(solid)` whose SDF is the union of the pour-gate
/// cylinder (always present in `Default`) and the air-vent
/// cylinder (present if `spec.include_vent`). Both cylinders are
/// positioned at their respective centerline endpoints, oriented
/// along the local tangent.
///
/// The cylinder axis matches the centerline tangent at each
/// endpoint — i.e., the pour gate runs ALONG the centerline at
/// the base end, not perpendicular to it. This makes the channel
/// a natural extension of the body's "axial" direction for
/// curved scans: silicone flows in along the body's spine.
#[must_use]
pub fn build_pour_gate_solid(ribbon: &Ribbon) -> Option<Solid> {
    let spec = match &ribbon.pour_gate {
        PourGateKind::None => return None,
        PourGateKind::Default(spec) => spec,
    };

    let first_seg = ribbon.segments.first()?;
    let last_seg = ribbon.segments.last()?;

    let gate_cylinder = channel_cylinder(
        first_seg.start,
        first_seg.tangent,
        spec.gate_radius_m,
        spec.channel_half_length_m,
    );

    if !spec.include_vent {
        return Some(gate_cylinder);
    }

    let vent_cylinder = channel_cylinder(
        last_seg.end,
        last_seg.tangent,
        spec.vent_radius_m,
        spec.channel_half_length_m,
    );

    Some(gate_cylinder.union(vent_cylinder))
}

/// Build a single channel cylinder centered at `center` with the
/// given `tangent` axis, `radius`, and `half_length`. Used by
/// [`build_pour_gate_solid`] for both gate + vent.
fn channel_cylinder(
    center: nalgebra::Point3<f64>,
    tangent: Vector3<f64>,
    radius: f64,
    half_length: f64,
) -> Solid {
    let rotation = UnitQuaternion::rotation_between(&Vector3::z_axis().into_inner(), &tangent)
        .unwrap_or_else(UnitQuaternion::identity);
    Solid::cylinder(radius, half_length)
        .rotate(rotation)
        .translate(center.coords)
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
    fn pour_gate_spec_iter1_has_workshop_defaults() {
        let s = PourGateSpec::iter1();
        assert!((s.gate_radius_m - 0.003).abs() < f64::EPSILON);
        assert!((s.vent_radius_m - 0.0015).abs() < f64::EPSILON);
        assert!((s.channel_half_length_m - 0.020).abs() < f64::EPSILON);
        assert!(s.include_vent);
    }

    #[test]
    fn pour_gate_kind_default_is_none() {
        assert_eq!(PourGateKind::default(), PourGateKind::None);
    }

    #[test]
    fn build_pour_gate_solid_none_returns_none() {
        let ribbon = straight_x_ribbon(); // default pour_gate = None
        assert!(build_pour_gate_solid(&ribbon).is_none());
    }

    #[test]
    fn build_pour_gate_solid_default_returns_some_with_finite_bounds() {
        let ribbon =
            straight_x_ribbon().with_pour_gate(PourGateKind::Default(PourGateSpec::iter1()));
        let solid = build_pour_gate_solid(&ribbon).expect("Default kind should yield a solid");
        let aabb = solid
            .bounds()
            .expect("channel cylinders should have finite bounds");
        // Gate at (-0.05, 0, 0) along +X tangent with half-length
        // 0.020 → x ∈ [-0.07, -0.03]. Vent at (+0.05, 0, 0) →
        // x ∈ [+0.03, +0.07]. Combined union x ∈ [-0.07, +0.07].
        assert!(aabb.min.x <= -0.069);
        assert!(aabb.max.x >= 0.069);
        // y, z extents bounded by gate radius (3 mm).
        assert!(aabb.min.y >= -0.0035);
        assert!(aabb.max.y <= 0.0035);
    }

    #[test]
    fn build_pour_gate_solid_no_vent_is_smaller_than_with_vent() {
        let mut spec_no_vent = PourGateSpec::iter1();
        spec_no_vent.include_vent = false;
        let ribbon_no_vent =
            straight_x_ribbon().with_pour_gate(PourGateKind::Default(spec_no_vent));
        let ribbon_with_vent =
            straight_x_ribbon().with_pour_gate(PourGateKind::Default(PourGateSpec::iter1()));
        let no_vent = build_pour_gate_solid(&ribbon_no_vent).unwrap();
        let with_vent = build_pour_gate_solid(&ribbon_with_vent).unwrap();
        // No-vent solid spans only the gate region (negative x).
        // With-vent solid spans both gate + vent (negative + positive x).
        assert!(no_vent.bounds().unwrap().max.x < 0.0);
        assert!(with_vent.bounds().unwrap().max.x > 0.0);
    }

    #[test]
    fn pour_gate_solid_is_inside_at_channel_axis() {
        // Pour gate at (-0.05, 0, 0) along +X tangent, radius 3 mm,
        // half-length 20 mm. Query on the axis 10 mm outward
        // (toward -X): (-0.060, 0, 0). Inside the cylinder.
        let ribbon =
            straight_x_ribbon().with_pour_gate(PourGateKind::Default(PourGateSpec::iter1()));
        let solid = build_pour_gate_solid(&ribbon).unwrap();
        let q = Point3::new(-0.060, 0.0, 0.0);
        assert!(
            solid.evaluate(&q) < 0.0,
            "pour gate SDF on channel axis should be negative; got {}",
            solid.evaluate(&q),
        );
    }
}
