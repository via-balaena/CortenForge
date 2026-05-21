//! Pour-gate + air-vent geometry for v2 multi-piece molds.
//!
//! v2 Step 10 + v2.1 sub-leaves 2/3 placed two cylindrical channels
//! off the v2 mold pieces. Workshop iter-1 visual-pass review
//! (2026-05-21) surfaced two problems with that layout for sock-
//! shaped scans:
//!
//! - The side-mounted pour gate exited on the cup wall's curved
//!   binormal face — no flat surface for a funnel to rest on
//!   during pour.
//! - The apex vent at the centerline's argmax-z vertex landed on
//!   roughly the same dome region as the workshop-natural pour
//!   end, but the gate was somewhere else entirely.
//!
//! v2.1 sub-leaf 4 (this module, 2026-05-21) reshapes the channels
//! into a **V at the dome end** of the centerline — opposite the
//! plug-anchor pin / cap plane. Both channels share the V apex
//! (the centerline endpoint farthest from
//! [`crate::ribbon::Ribbon::pour_end_hint`]'s centroid) and splay
//! apart in the plane containing the local outward-axis + the
//! local segment's `binormal`, with a 30° half-angle. The pour leg
//! lands on the [`crate::ribbon::PieceSide::Positive`] side
//! (`+binormal` half-space); the vent leg lands on the
//! [`crate::ribbon::PieceSide::Negative`] side (`-binormal`
//! half-space). Workshop user orients the assembled mold `+Z` up
//! and sees two distinct holes at the top:
//!
//! - **Pour leg** (`gate_radius_m`, default 5 mm = 10 mm Ø) —
//!   sized for honey-thick workshop silicones (Dragon Skin 10A /
//!   20A at ~20-23k cps) so per-layer pour completes in seconds
//!   without trapping air. Mates with the printable
//!   [`crate::funnel::build_funnel_solid`] funnel.
//! - **Vent leg** (`vent_radius_m`, default 3 mm = 6 mm Ø) — wide
//!   enough to mesh cleanly at cf-cast's default 3 mm cells, narrow
//!   enough to be workshop-visually distinct from the pour leg.
//!
//! ## Composition
//!
//! [`build_pour_gate_solid`] returns the union of the two cylinders,
//! or just the pour cylinder if `spec.include_vent = false`, or
//! `None` if [`PourGateKind::None`].
//! [`crate::piece::compose_piece_solid`] subtracts this solid from
//! BOTH [`crate::ribbon::PieceSide`]s — since the legs lie on
//! opposite sides of the seam (one on `+binormal`, one on
//! `-binormal`), each piece carves the leg on its own side and
//! leaves the other leg untouched (modulo the small
//! [`crate::piece::RIBBON_PIECE_OVERLAP_M`] seam-overlap slice).
//!
//! ## Why splay along binormal, not split-normal
//!
//! The ribbon's seam zero-set is the plane perpendicular to the
//! local `binormal`. Splaying the V along `±binormal` puts the
//! pour leg on `+binormal` (Positive piece) and the vent leg on
//! `-binormal` (Negative piece) — each leg lives entirely on one
//! piece. Splaying along `±split_normal` would keep both legs in
//! the SEAM plane (since `split_normal` is perpendicular to
//! `binormal`), so both legs would straddle the seam and each
//! piece would carve a half-cylinder of each — workshop user can't
//! tell pour from vent at the seam. Binormal splay gives the
//! cleaner per-piece geometry.
//!
//! ## Why opposite the cap plane
//!
//! The plug-anchor pin lives at the cap-plane centroid (per
//! [`crate::ribbon::Ribbon::pour_end_hint`]). The cap plane is the
//! body's natural open end and sits at the BOTTOM of the assembled
//! mold under `+Z` up orientation. Putting the pour channels at the
//! cap-plane end would put the pour openings on the floor of the
//! mold — silicone would pour DOWN through the cup wall and out
//! the bottom rather than INTO the cavity.
//!
//! Anchoring at the OPPOSITE centerline endpoint (the body's closed
//! dome, which sits on top when the mold is `+Z` up) puts the pour
//! openings at the top of the assembly where gravity feeds silicone
//! into the cavity.
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
use nalgebra::{Point3, UnitQuaternion, Vector3};

use crate::ribbon::Ribbon;

/// V half-angle in radians — 30° (= π/6). Each leg of the V splays
/// from the body's outward axis at the dome end by this angle
/// toward `±binormal`; total V opening is 60°.
///
/// Chosen so the two outer-surface punctures are visibly separate
/// on the bounding-region outer surface while keeping each leg
/// close enough to axial that workshop pour ergonomics aren't
/// awkward. Not user-tunable; iter-1 visual gate accepted the
/// default.
const V_HALF_ANGLE_RAD: f64 = std::f64::consts::FRAC_PI_6;

/// Cylindrical pour-gate + air-vent geometry spec. All dimensions
/// in meters.
#[derive(Debug, Clone, PartialEq)]
pub struct PourGateSpec {
    /// Pour-leg channel radius (m). Default `0.005` = 5 mm =
    /// 10 mm diameter.
    ///
    /// Sized for honey-thick workshop silicones (Dragon Skin 10A /
    /// 20A / 30A at ~20-23k cps): 10 mm Ø roughly doubles the
    /// pour-area vs the pre-iter-1 6 mm Ø, dropping per-layer pour
    /// time from minutes to seconds. The matching pour funnel
    /// ([`crate::build_funnel_solid`]) sizes its nipple to
    /// `gate_radius_m × 2 − 0.2 mm` slack so it slides into the
    /// pour leg hole.
    pub gate_radius_m: f64,
    /// Vent-leg channel radius (m). Default `0.003` = 3 mm =
    /// 6 mm diameter — same as the pour leg.
    ///
    /// Equal-radius legs ship together because at cf-cast's default
    /// 3 mm mesh-cell size, a 2 mm Ø vent (the pre-V-shape default)
    /// captures as only 0.67 cells radially and MC produces a
    /// partial / fragmented through-hole — workshop iter-1 visual
    /// gate falsified the small-vent variant. The pour leg's 6 mm Ø
    /// is the minimum that meshes cleanly at 3 mm cells (1 cell
    /// radius); matching the vent to it guarantees the vent leg
    /// also resolves. Workshop user tells pour from vent by position
    /// (procedure.md surfaces the +/-binormal side mapping), not by
    /// hole diameter.
    ///
    /// Trade-off: silicone surface tension can't hold against the
    /// pour-pressure differential on a 6 mm Ø vent as reliably as
    /// the prior 2 mm Ø. Workshop iter-1 accepts that — the pour
    /// leg is the main filler; the vent's role is post-fill air
    /// release as the pour decelerates.
    pub vent_radius_m: f64,
    /// Half-length of the **pour leg** cylinder (m). Total channel
    /// length is `2 * gate_half_length_m`. Default `0.045` = 45 mm
    /// half-length → 90 mm total channel.
    ///
    /// The pour leg anchors at the V apex (centerline endpoint
    /// farthest from [`crate::ribbon::Ribbon::pour_end_hint`])
    /// and extends along the pour-leg axis (outward at apex,
    /// tilted by the module-private `V_HALF_ANGLE_RAD` (30°)
    /// toward `+binormal`, where
    /// `binormal` is the local segment's cached binormal at the
    /// apex endpoint).
    /// For a typical workshop bounding region (cuboid with ≤80 mm
    /// half-extent along the body axis past the dome), the default
    /// 45 mm half-length puts the outer tip past the bounding-
    /// region outer surface — the channel reliably punches all the
    /// way through the cup wall to the outside.
    ///
    /// Workshop users with larger bounding regions tune this up.
    pub gate_half_length_m: f64,
    /// Half-length of the **vent leg** cylinder (m). Total channel
    /// length is `2 * vent_half_length_m`. Default `0.040` = 40 mm
    /// half-length → 80 mm total channel.
    ///
    /// The vent leg anchors at the same V apex as the pour leg and
    /// extends along the vent-leg axis (outward at apex, tilted by
    /// the module-private `V_HALF_ANGLE_RAD` (30°) toward
    /// `-binormal`).
    pub vent_half_length_m: f64,
    /// Whether to include the vent leg. Default `true`. Disable for
    /// short straight molds where air escape back through the pour
    /// leg is sufficient (silicone fills bottom-up; air bubbles
    /// rise out the same channel).
    pub include_vent: bool,
}

impl PourGateSpec {
    /// v2.1 iter-1 defaults: 10 mm Ø pour leg (90 mm total), 6 mm Ø
    /// vent leg (80 mm total), vent enabled. Both legs anchored at
    /// the V apex on the body's dome end (opposite the cap plane),
    /// splayed ±30° in the (outward + binormal) plane.
    ///
    /// Pour leg is bigger than the vent: honey-thick workshop
    /// silicones (Dragon Skin 10A / 20A at ~20-23k cps) pour
    /// tediously through smaller channels and trap air bubbles.
    /// 10 mm Ø roughly doubles the pour-area vs the original
    /// 6 mm Ø, dropping per-layer pour time from minutes to
    /// seconds. Vent stays at 6 mm Ø — it only needs to release
    /// air (no flow-rate constraint) and is workshop-visually
    /// distinct from the larger pour hole. Both Øs mesh cleanly at
    /// cf-cast's default 3 mm cells (10 mm = 1.67 cells radial,
    /// 6 mm = 1 cell radial).
    #[must_use]
    pub const fn iter1() -> Self {
        Self {
            gate_radius_m: 0.005,
            vent_radius_m: 0.003,
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
/// `Default(PourGateSpec)` enables the V-at-dome channels.
#[derive(Debug, Clone, Default, PartialEq)]
pub enum PourGateKind {
    /// No pour-gate or vent channels.
    #[default]
    None,
    /// V-at-dome pour-leg + (optional) vent-leg per the supplied
    /// [`PourGateSpec`]. Both legs share an apex at the centerline
    /// endpoint opposite the cap plane (per
    /// [`crate::ribbon::Ribbon::pour_end_hint`]) and splay ±30°
    /// in the (outward + `binormal`) plane.
    Default(PourGateSpec),
}

/// Build the combined pour-leg + vent-leg Solid for the ribbon's
/// pour-gate kind, or `None` if [`PourGateKind::None`].
///
/// Returns `Some(solid)` whose SDF is the union of the pour-leg
/// cylinder (always present in `Default`) and the vent-leg
/// cylinder (present when `spec.include_vent`).
///
/// V geometry:
/// - **Apex** at `v_apex_anchor(ribbon)` — the centerline endpoint
///   FARTHER from `ribbon.pour_end_hint`'s centroid, or
///   `centerline[0]` when the hint is unset (cf-scan-prep's
///   tip→base convention puts the dome at `centerline[0]`).
/// - **Outward axis at apex** — the local-segment tangent pointing
///   AWAY from body interior at that endpoint (`-first.tangent` if
///   apex = `centerline[0]`, `+last.tangent` if apex =
///   `centerline.last()`).
/// - **Pour leg axis** — `cos(θ) * outward + sin(θ) * binormal`,
///   normalized. Lands on the [`crate::ribbon::PieceSide::Positive`]
///   side (the `+binormal` side of the ribbon seam).
/// - **Vent leg axis** — `cos(θ) * outward - sin(θ) * binormal`,
///   normalized. Lands on the [`crate::ribbon::PieceSide::Negative`]
///   side.
///
/// Returns `None` only when the ribbon has no segments — impossible
/// for ribbons built via the public [`Ribbon::new`] constructor
/// (which rejects fewer than 2 vertices).
///
/// [`Ribbon::new`]: crate::ribbon::Ribbon::new
#[must_use]
pub fn build_pour_gate_solid(ribbon: &Ribbon) -> Option<Solid> {
    let spec = match &ribbon.pour_gate {
        PourGateKind::None => return None,
        PourGateKind::Default(spec) => spec,
    };

    let (apex, outward, binormal) = v_apex_anchor(ribbon)?;

    let cos = V_HALF_ANGLE_RAD.cos();
    let sin = V_HALF_ANGLE_RAD.sin();
    let pour_leg_axis = (cos * outward + sin * binormal).normalize();
    let vent_leg_axis = (cos * outward - sin * binormal).normalize();

    let pour_leg = leg_cylinder(
        apex,
        pour_leg_axis,
        spec.gate_radius_m,
        spec.gate_half_length_m,
    );

    if !spec.include_vent {
        return Some(pour_leg);
    }

    let vent_leg = leg_cylinder(
        apex,
        vent_leg_axis,
        spec.vent_radius_m,
        spec.vent_half_length_m,
    );
    Some(pour_leg.union(vent_leg))
}

/// Resolve the V apex `(point, outward_axis, binormal)` for this
/// ribbon — the centerline endpoint opposite the
/// [`crate::ribbon::Ribbon::pour_end_hint`]'s centroid, with the
/// local-segment tangent pointing outward (away from body interior)
/// and the local-segment cached binormal threaded through so the
/// V's leg splay aligns with the ribbon seam at the apex.
///
/// Hint resolution mirrors `crate::plug::build_plug_pin_solid`'s
/// pour/dome split: when a hint is set, the V apex is the centerline
/// endpoint FARTHER from the hint centroid (the hint marks the cap
/// plane / plug-anchor side; the V apex is on the opposite end).
/// When no hint is set, the apex falls back to `centerline[0]`
/// (per cf-scan-prep's tip→base convention, `centerline[0]` is the
/// closed dome).
///
/// The returned `binormal` is the segment's cached binormal at the
/// apex endpoint: `first.binormal` if apex = `first.start`,
/// `last.binormal` if apex = `last.end`. Both are unit-length per
/// `Ribbon::new`'s invariant.
fn v_apex_anchor(ribbon: &Ribbon) -> Option<(Point3<f64>, Vector3<f64>, Vector3<f64>)> {
    let first = ribbon.segments.first()?;
    let last = ribbon.segments.last()?;
    let start_anchor = (first.start, -first.tangent, first.binormal);
    let end_anchor = (last.end, last.tangent, last.binormal);
    match ribbon.pour_end_hint {
        Some((centroid, _)) => {
            let d_start = (centroid - first.start).norm_squared();
            let d_end = (centroid - last.end).norm_squared();
            if d_start > d_end {
                Some(start_anchor)
            } else {
                Some(end_anchor)
            }
        }
        // No hint: fall back to `centerline[0]` per cf-scan-prep's
        // tip→base convention (closed dome lives at `centerline[0]`).
        None => Some(start_anchor),
    }
}

/// Build a single leg cylinder of the V — anchored at `apex` with
/// its inner tip at the apex point and extending outward along
/// `axis` by `2 * half_length_m`. The unit `axis` must already be
/// normalized.
fn leg_cylinder(apex: Point3<f64>, axis: Vector3<f64>, radius: f64, half_length: f64) -> Solid {
    let center = apex + axis * half_length;
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
        assert!(
            (s.gate_radius_m - 0.005).abs() < f64::EPSILON,
            "gate_radius_m should be 5 mm (10 mm Ø) for honey-thick \
             silicone flow",
        );
        assert!(
            (s.vent_radius_m - 0.003).abs() < f64::EPSILON,
            "vent_radius_m should be 3 mm (6 mm Ø) — smaller than \
             pour gate; visually distinct + meshes cleanly at 3 mm cells",
        );
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

    /// Straight +X centerline with +Y split-normal → binormal +Z.
    /// With no hint set, V apex falls back to
    /// `centerline[0] = (-0.050, 0, 0)` with outward axis =
    /// `-first.tangent = -X`. Pour leg tilted +30° toward
    /// `+binormal = +Z` from `-X` (so pour leg axis
    /// ≈ `(-cos30°, 0, +sin30°)` = `(-0.866, 0, +0.5)`); vent leg
    /// tilted -30° toward `-binormal = -Z` (so vent axis
    /// ≈ `(-0.866, 0, -0.5)`). Each leg extends `2 * half_length`
    /// from apex.
    #[test]
    fn build_pour_gate_solid_default_returns_some_with_finite_bounds() {
        let ribbon =
            straight_x_ribbon().with_pour_gate(PourGateKind::Default(PourGateSpec::iter1()));
        let solid = build_pour_gate_solid(&ribbon).expect("Default kind should yield a solid");
        let aabb = solid
            .bounds()
            .expect("channel cylinders should have finite bounds");
        // Pour leg of length 90 mm at ~0.866 in -X direction reaches
        // x ≈ -0.050 - 90 mm × 0.866 ≈ -0.128 m.
        assert!(
            aabb.min.x <= -0.120,
            "V pour leg should reach x ≤ -120 mm; got min.x = {}",
            aabb.min.x,
        );
        // Pour leg's +Z extent: apex + 90 mm × 0.5 = +0.045 m (plus
        // radius slack ~3 mm).
        assert!(
            aabb.max.z >= 0.040,
            "V pour leg's +binormal extent should reach z ≥ +40 mm; got max.z = {}",
            aabb.max.z,
        );
        // Vent leg's -Z extent: apex + 80 mm × (-0.5) = -0.040 m
        // (plus vent radius ~1 mm).
        assert!(
            aabb.min.z <= -0.035,
            "V vent leg's -binormal extent should reach z ≤ -35 mm; got min.z = {}",
            aabb.min.z,
        );
    }

    /// With vent disabled, only the pour leg remains — AABB extends
    /// in +binormal (=+Z) but NOT in -binormal (=-Z).
    #[test]
    fn build_pour_gate_solid_no_vent_omits_negative_binormal_extent() {
        let mut spec_no_vent = PourGateSpec::iter1();
        spec_no_vent.include_vent = false;
        let ribbon_no_vent =
            straight_x_ribbon().with_pour_gate(PourGateKind::Default(spec_no_vent));
        let ribbon_with_vent =
            straight_x_ribbon().with_pour_gate(PourGateKind::Default(PourGateSpec::iter1()));
        let no_vent = build_pour_gate_solid(&ribbon_no_vent).unwrap();
        let with_vent = build_pour_gate_solid(&ribbon_with_vent).unwrap();
        // With-vent has the -binormal leg → AABB min.z well below 0.
        let with_vent_min_z = with_vent.bounds().unwrap().min.z;
        assert!(with_vent_min_z <= -0.035);
        // No-vent has only the +binormal pour leg → AABB min.z is
        // bounded by the pour-leg radius slack (~3 mm).
        let no_vent_min_z = no_vent.bounds().unwrap().min.z;
        assert!(
            no_vent_min_z >= -0.010,
            "no-vent solid should not extend into -z; got min.z = {no_vent_min_z}",
        );
    }

    /// Pour leg's inner tip sits at the V apex. A query just inside
    /// the cylinder along the pour-leg axis should be inside (SDF
    /// < 0).
    #[test]
    fn pour_gate_solid_is_inside_at_pour_leg_axis() {
        let ribbon =
            straight_x_ribbon().with_pour_gate(PourGateKind::Default(PourGateSpec::iter1()));
        let solid = build_pour_gate_solid(&ribbon).unwrap();
        // V apex at centerline[0] = (-0.050, 0, 0). Pour leg axis
        // ≈ (-cos30°, 0, +sin30°) (binormal at apex = +Z for this
        // ribbon). Step 20 mm along the leg from apex:
        let pour_axis = Vector3::new(-V_HALF_ANGLE_RAD.cos(), 0.0, V_HALF_ANGLE_RAD.sin());
        let on_leg = Point3::new(-0.050, 0.0, 0.0) + pour_axis * 0.020;
        assert!(
            solid.evaluate(&on_leg) < 0.0,
            "pour-leg SDF along leg axis should be negative; got {}",
            solid.evaluate(&on_leg),
        );
    }

    /// V apex sits at the centerline endpoint opposite the
    /// `pour_end_hint`. Set the hint near `centerline.last()` to
    /// confirm the apex flips to `centerline[0]`; set near
    /// `centerline[0]` to confirm the apex flips to
    /// `centerline.last()`.
    #[test]
    fn v_apex_flips_to_endpoint_opposite_pour_end_hint() {
        // Hint near `centerline.last()` (= (+0.050, 0, 0)) → V apex
        // at `centerline[0]` (= (-0.050, 0, 0)). Pour leg extends
        // outward in -X-ish.
        let ribbon = straight_x_ribbon()
            .with_pour_end_hint(Point3::new(0.060, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0))
            .with_pour_gate(PourGateKind::Default(PourGateSpec::iter1()));
        let solid = build_pour_gate_solid(&ribbon).unwrap();
        assert!(
            solid.bounds().unwrap().min.x < -0.050,
            "V apex at centerline[0]: pour leg should extend into -X past -50 mm",
        );

        // Hint near `centerline[0]` → V apex at `centerline.last()`.
        // Pour leg extends outward in +X-ish.
        let ribbon = straight_x_ribbon()
            .with_pour_end_hint(Point3::new(-0.060, 0.0, 0.0), Vector3::new(-1.0, 0.0, 0.0))
            .with_pour_gate(PourGateKind::Default(PourGateSpec::iter1()));
        let solid = build_pour_gate_solid(&ribbon).unwrap();
        assert!(
            solid.bounds().unwrap().max.x > 0.050,
            "V apex at centerline.last(): pour leg should extend into +X past +50 mm",
        );
    }

    /// Iter-1-shaped fixture: trimmed centerline (tip at z=+0.073,
    /// trimmed end at z=-0.013), cap centroid 40 mm past trimmed
    /// end at z=-0.053. V apex must land at `centerline[0]` (the
    /// closed dome at z=+0.073), NOT at `centerline.last()` (the
    /// trimmed body interior).
    #[test]
    fn v_apex_at_dome_for_iter1_trimmed_centerline_with_cap_hint() {
        let centerline = vec![
            Point3::new(0.0, 0.0, 0.073), // closed dome
            Point3::new(0.0, 0.0, 0.020),
            Point3::new(0.0, 0.0, -0.013), // trimmed end (40 mm above cap)
        ];
        let split = SplitNormal::new(Vector3::new(1.0, 0.0, 0.0)).unwrap();
        let cap_centroid = Point3::new(0.0, 0.0, -0.053);
        let cap_normal = Vector3::new(0.0, 0.0, -1.0);
        let ribbon = Ribbon::new(centerline, split)
            .unwrap()
            .with_pour_end_hint(cap_centroid, cap_normal)
            .with_pour_gate(PourGateKind::Default(PourGateSpec::iter1()));
        let solid = build_pour_gate_solid(&ribbon).expect("Default kind should yield a solid");
        let aabb = solid.bounds().unwrap();
        // V apex at z=+0.073, outward axis = -first.tangent ≈ +Z
        // (first.tangent goes from z=+0.073 to z=+0.020 = -Z).
        // Pour-leg + vent-leg both go in roughly +Z direction (with
        // ±split offsets).
        assert!(
            aabb.max.z > 0.10,
            "V at iter-1 dome should reach high +Z; got max.z = {}",
            aabb.max.z,
        );
        // Both legs are entirely above z=+0.060 — the apex is at
        // +0.073 and legs extend outward (+Z) with only the radius
        // slack reaching below the apex.
        assert!(
            aabb.min.z >= 0.060,
            "V at iter-1 dome should not extend below z=+60 mm; got min.z = {}",
            aabb.min.z,
        );
    }
}
