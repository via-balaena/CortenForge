//! Pour-gate + air-vent geometry for v2 multi-piece molds.
//!
//! Step 10 of `docs/CURVE_FOLLOWING_DESIGN.md` added two cylindrical
//! channels CSG'd off the v2 mold pieces; v2.1 sub-leaves 2 + 3
//! relocate both off the centerline endpoints so the endpoints can
//! host plug-anchor pin sockets without overlap and so the
//! workshop "roof" hosts the vent for natural air escape:
//!
//! - **Pour gate** at the centerline **midpoint** (arc fraction
//!   `0.5`), axis along the ribbon's local binormal. The channel
//!   cylinder's inner tip is at the centerline (so it overlaps the
//!   body cavity by a small redundant amount — MC tolerance
//!   benefit) and extends outward along binormal by
//!   `2 * gate_half_length_m`, well past the typical bounding-
//!   region outer surface. Workshop user pours into the channel
//!   opening on the side face of the assembled mold.
//! - **Air vent** at the centerline's **argmax-z vertex** (the
//!   highest point on the polyline; workshop user orients the
//!   assembled mold with `+Z` up, so the apex is the physical
//!   highest point on the cured silicone body — where trapped
//!   air collects). Axis is `+Z` (world up); cylinder rises
//!   straight from the apex up through the cup wall to the
//!   bounding region's `+Z` outer face.
//!
//! ## Composition
//!
//! [`build_pour_gate_solid`] returns the union of the gate +
//! (optional) vent cylinders, or `None` if [`PourGateKind::None`].
//! [`crate::piece::compose_piece_solid`] subtracts this solid
//! from the base piece geometry, applying the same channel to
//! BOTH [`crate::ribbon::PieceSide`]s. The gate cylinder is
//! oriented perpendicular to the ribbon seam (along binormal) so
//! each piece carves a half-cylinder cross-section that combines
//! into the full channel when the pieces close.
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
    /// Air-vent channel radius (m). Default `0.001` = 1 mm =
    /// 2 mm diameter — small enough for silicone surface tension
    /// to hold against the workshop pour-pressure differential
    /// while letting air freely escape. v2.1 sub-leaf 3 shrank
    /// the default from the prior 1.5 mm to reduce silicone
    /// surface-tension hold concerns now that the vent is on the
    /// workshop "roof" (apex of the centerline arc, oriented
    /// `+Z`-up).
    pub vent_radius_m: f64,
    /// Half-length of the **side-mounted pour-gate** cylinder
    /// (m). Total channel length is `2 * gate_half_length_m`.
    /// Default `0.045` = 45 mm half-length → 90 mm total channel.
    ///
    /// v2.1 sub-leaf 2 anchors the gate cylinder so its **inner
    /// tip is at the centerline midpoint** and the cylinder
    /// extends outward along the local ribbon binormal by
    /// `2 * gate_half_length_m`. For a typical workshop bounding
    /// region (cuboid with ≤80 mm half-extent along the binormal
    /// axis from the centerline midpoint), the default 45 mm
    /// half-length puts the outer tip 10 mm past the bounding-
    /// region outer surface — the channel reliably punches all
    /// the way through the cup wall to the outside.
    ///
    /// Workshop users with larger bounding regions tune this up.
    /// Keep the redundant outside-bounding-region portion of the
    /// cylinder small: it's a no-op (it doesn't intersect any mold
    /// piece's `bounding ∖ layer_body ∩ ribbon_side` composition)
    /// but does enlarge the gate cylinder's AABB, which slows
    /// cf-design's octree-pruning interval arithmetic.
    pub gate_half_length_m: f64,
    /// Half-length of the **apex vent** cylinder along its `+Z`
    /// axis (m). Total channel length is `2 * vent_half_length_m`.
    /// Default `0.040` = 40 mm half-length → 80 mm total channel.
    ///
    /// v2.1 sub-leaf 3 anchors the vent cylinder so its **inner
    /// tip is at the centerline's argmax-z vertex** and the
    /// cylinder extends straight up `+Z` by `2 * vent_half_length_m`.
    /// For a typical workshop bounding region (cuboid with ≤80 mm
    /// half-extent in `+Z` from the centerline apex), the default
    /// 40 mm half-length puts the outer tip ≥80 mm above the apex
    /// — the channel reliably punches all the way through the
    /// cup wall to the workshop "roof".
    ///
    /// Workshop users with apex-far-from-`+Z`-face geometries
    /// tune this up. The redundant outside-bounding-region portion
    /// of the cylinder is a no-op for the mold piece composition.
    pub vent_half_length_m: f64,
    /// Whether to include the apex air-vent cylinder (placed at the
    /// centerline's argmax-z vertex along `+Z` per v2.1 sub-leaf 3).
    /// Default `true`. Disable for short straight molds where air
    /// escape through the side-mounted pour gate is sufficient.
    pub include_vent: bool,
}

impl PourGateSpec {
    /// v2.1 iter-1 defaults: 6 mm Ø pour gate (side-mounted at
    /// centerline midpoint along binormal, 90 mm total channel),
    /// 2 mm Ø apex vent (at centerline's argmax-z vertex along
    /// `+Z`, 80 mm total channel), vent enabled.
    #[must_use]
    pub const fn iter1() -> Self {
        Self {
            gate_radius_m: 0.003,
            vent_radius_m: 0.001,
            gate_half_length_m: 0.045,
            vent_half_length_m: 0.040,
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
    /// Side-mounted pour gate at the centerline midpoint along the
    /// ribbon binormal + optional apex air vent at the centerline's
    /// argmax-z vertex along `+Z`, per the supplied
    /// [`PourGateSpec`]. (v2 Step 10 originally placed the gate at
    /// the centerline base along tangent and the vent at the tip
    /// end along tangent; v2.1 sub-leaves 2 + 3 relocated both off
    /// the centerline endpoints so the endpoints can host plug-
    /// anchor pin sockets and the workshop "roof" hosts the vent.)
    Default(PourGateSpec),
}

/// Build the combined pour-gate + vent Solid for the ribbon's
/// pour-gate kind, or `None` if [`PourGateKind::None`].
///
/// Returns `Some(solid)` whose SDF is the union of the pour-gate
/// cylinder (always present in `Default`) and the air-vent
/// cylinder (present if `spec.include_vent`).
///
/// - The **pour gate** is side-mounted: the cylinder is centered
///   at `centerline_midpoint + gate_half_length_m * binormal_unit`
///   with axis along the binormal at the midpoint, so the inner
///   tip rests at the centerline (overlapping the body cavity for
///   MC tolerance) and the outer tip extends past the typical
///   bounding-region outer surface. Workshop user pours into the
///   channel opening on the assembled mold's side face. Reuses
///   [`Ribbon::sample_at_arc_fraction(0.5)`] to derive midpoint +
///   binormal.
/// - The **vent** is at the centerline's **argmax-z vertex**
///   (the highest point on the polyline; workshop user orients
///   the assembled mold with `+Z` up, so the apex vertex is the
///   physical highest point on the cured silicone body, where
///   trapped air collects). Axis is `+Z` (world up), so the vent
///   channel rises straight from the centerline apex up through
///   the cup wall to the bounding region's `+Z` outer face.
///
/// Returns `None` when [`Ribbon::sample_at_arc_fraction(0.5)`]
/// fails to resolve (degenerate centerline). Per the
/// `crate::ribbon::Ribbon` invariants this only occurs for
/// hand-constructed Ribbons that bypass the public constructor;
/// the public path always has at least one segment.
///
/// [`Ribbon::sample_at_arc_fraction(0.5)`]: crate::ribbon::Ribbon::sample_at_arc_fraction
#[must_use]
pub fn build_pour_gate_solid(ribbon: &Ribbon) -> Option<Solid> {
    let spec = match &ribbon.pour_gate {
        PourGateKind::None => return None,
        PourGateKind::Default(spec) => spec,
    };

    let (midpoint, _tangent, binormal) = ribbon.sample_at_arc_fraction(0.5)?;
    let gate_center = midpoint + binormal * spec.gate_half_length_m;
    let gate_cylinder = channel_cylinder(
        gate_center,
        binormal,
        spec.gate_radius_m,
        spec.gate_half_length_m,
    );

    if !spec.include_vent {
        return Some(gate_cylinder);
    }

    let apex = apex_point(&ribbon.points)?;
    let up = Vector3::z_axis().into_inner();
    let vent_center = apex + up * spec.vent_half_length_m;
    let vent_cylinder =
        channel_cylinder(vent_center, up, spec.vent_radius_m, spec.vent_half_length_m);

    Some(gate_cylinder.union(vent_cylinder))
}

/// Return the centerline vertex with the maximum z coordinate (the
/// "apex" of the polyline in the workshop's `+Z = up` frame).
///
/// Trapped air during pour + cure collects at the highest point of
/// the cavity. v2.1 sub-leaf 3 places the air vent here so the
/// channel rises straight from the cavity ceiling to the bounding
/// region's `+Z` outer face — workshop user orients the assembled
/// mold with `+Z` up, sees the vent as the small hole on the
/// "roof".
///
/// Returns `None` only for an empty polyline (impossible for a
/// public-API `Ribbon` since `Ribbon::new` rejects fewer than 2
/// vertices).
fn apex_point(points: &[nalgebra::Point3<f64>]) -> Option<nalgebra::Point3<f64>> {
    points
        .iter()
        .copied()
        .max_by(|a, b| a.z.partial_cmp(&b.z).unwrap_or(std::cmp::Ordering::Equal))
}

/// Build a single channel cylinder centered at `center` with the
/// given `axis`, `radius`, and `half_length`. Used by
/// [`build_pour_gate_solid`] for the side-mounted gate (where
/// `axis = binormal`) and the apex vent (where `axis = +Z`).
fn channel_cylinder(
    center: nalgebra::Point3<f64>,
    axis: Vector3<f64>,
    radius: f64,
    half_length: f64,
) -> Solid {
    let rotation = UnitQuaternion::rotation_between(&Vector3::z_axis().into_inner(), &axis)
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
        assert!((s.vent_radius_m - 0.001).abs() < f64::EPSILON);
        assert!((s.gate_half_length_m - 0.045).abs() < f64::EPSILON);
        assert!((s.vent_half_length_m - 0.040).abs() < f64::EPSILON);
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

    /// Straight +X centerline with +Y split-normal → binormal = +Z.
    /// Midpoint = (0, 0, 0). Side-mounted gate extends from
    /// midpoint along +Z by `2 * gate_half_length_m` = 90 mm at
    /// the iter1 default. Apex vent rises straight +Z from the
    /// polyline's max-z vertex (here both vertices tie at z=0;
    /// `apex_point`'s `max_by` returns the last-encountered max so
    /// apex = `(+0.050, 0, 0)`); vent spans z ∈ [0, +0.080] at
    /// x=+0.050.
    #[test]
    fn build_pour_gate_solid_default_returns_some_with_finite_bounds() {
        let ribbon =
            straight_x_ribbon().with_pour_gate(PourGateKind::Default(PourGateSpec::iter1()));
        let solid = build_pour_gate_solid(&ribbon).expect("Default kind should yield a solid");
        let aabb = solid
            .bounds()
            .expect("channel cylinders should have finite bounds");
        // Side-mounted gate reaches z ≥ +0.089 (45 mm × 2 from
        // z=0). Apex vent reaches z ≥ +0.079 (40 mm × 2 from z=0).
        // Combined max.z dominated by gate.
        assert!(
            aabb.max.z >= 0.089,
            "gate cylinder should reach z ≥ +0.089 m; got max.z = {}",
            aabb.max.z
        );
        // Gate inner tip + vent inner tip both at z=0 → min z
        // bounded by ~gate-radius slack on the -z side.
        assert!(
            aabb.min.z >= -0.0035,
            "channels should start near z = 0; got min.z = {}",
            aabb.min.z
        );
        // Apex vent at x = +0.050 → max.x bounded by apex x plus
        // ~vent radius (1 mm).
        assert!(
            aabb.max.x >= 0.049,
            "apex vent at x = +0.050 should reach max.x ≥ +0.049; got max.x = {}",
            aabb.max.x
        );
    }

    /// With vent disabled, only the side-mounted gate remains —
    /// AABB spans z up high (gate) but x stays near zero (no vent
    /// reaching out to x = -0.050).
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
        // No-vent solid is just the side-mounted gate centered on
        // the centerline midpoint x = 0 — x stays within gate
        // radius (~3 mm). With-vent solid additionally has the
        // apex vent at x = +0.050, so max.x extends out to ~+0.051.
        assert!(
            no_vent.bounds().unwrap().max.x < 0.010,
            "no-vent gate-only solid should not extend far in +x; got max.x = {}",
            no_vent.bounds().unwrap().max.x,
        );
        assert!(with_vent.bounds().unwrap().max.x > 0.040);
    }

    /// Side-mounted gate is centered at `(0, 0, +half_length)` with
    /// axis +Z. Query on the channel axis at z = +0.045 (midpoint
    /// of the cylinder) should be inside (SDF < 0).
    #[test]
    fn pour_gate_solid_is_inside_at_channel_axis() {
        let ribbon =
            straight_x_ribbon().with_pour_gate(PourGateKind::Default(PourGateSpec::iter1()));
        let solid = build_pour_gate_solid(&ribbon).unwrap();
        let q = Point3::new(0.0, 0.0, 0.045);
        assert!(
            solid.evaluate(&q) < 0.0,
            "side-mounted gate SDF on channel axis at z=+0.045 should be negative; got {}",
            solid.evaluate(&q),
        );
    }

    /// Inner tip of the side-mounted gate sits at the centerline
    /// midpoint (z=0 for the straight-X test ribbon): the cylinder
    /// extends from there outward along +binormal, so a query at
    /// the midpoint along the cylinder axis is on the cylinder's
    /// face. SDF should be close to zero (numerical floor inside
    /// the cylinder, due to the cell-size-tolerant overlap
    /// described in [`PourGateSpec::gate_half_length_m`]'s
    /// docstring).
    #[test]
    fn pour_gate_solid_inner_tip_is_at_centerline_midpoint() {
        let ribbon =
            straight_x_ribbon().with_pour_gate(PourGateKind::Default(PourGateSpec::iter1()));
        let solid = build_pour_gate_solid(&ribbon).unwrap();
        // Just inside the cylinder's bottom face along the axis.
        let q_inside = Point3::new(0.0, 0.0, 0.001);
        assert!(
            solid.evaluate(&q_inside) < 0.0,
            "gate SDF just inside the inner tip should be negative; got {}",
            solid.evaluate(&q_inside),
        );
        // 4 mm below the midpoint along -binormal — outside the
        // gate cylinder (cylinder starts at z=0).
        let q_below = Point3::new(0.0, 0.0, -0.004);
        assert!(
            solid.evaluate(&q_below) > 0.0,
            "gate SDF below the inner tip (outside cylinder) should be positive; got {}",
            solid.evaluate(&q_below),
        );
    }
}
