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
//! - Position: at the **midpoint of the cup-wall annulus** along
//!   the ribbon's split-normal direction at each
//!   `centerline_sample(arc_fraction)` so the pin sits in the cup
//!   wall (outside the body cavity, inside the bounding region).
//!   The offset is **derived per-(layer × ribbon-frame)** by
//!   ray-marching the layer body + bounding-region SDFs along the
//!   split-normal from the centerline sample; no fixed
//!   `offset_from_centerline_m` knob.
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
//! ## Why the offset is derived, not fixed
//!
//! Pre-mold-wall-arc, the bounding region was a cuboid envelope
//! whose half-extent comfortably exceeded any per-layer body half-
//! extent, so a fixed 25 mm offset reliably landed the pin in
//! cup-wall territory regardless of the body's width along the
//! split-normal. After `54df41cb` (2026-05-19) the bounding region
//! is body-relative (`outermost_body.offset(wall_thickness_m)`), so
//! the "is the pin in the cup wall?" geometry now depends on body
//! radius along the split-normal at the pin's centerline z — a
//! quantity the fixed offset can't see. The iter-1 workshop visual
//! gate falsified the fixed default for sock-over-capsule
//! (body half-extent ~36 mm vs the 25 mm offset → pin inside the
//! body → free-floating sliver in the Negative-piece mesh +
//! invisible Positive-piece hole). Ray-marching the layer body +
//! bounding-region SDFs makes the pin position track whatever
//! per-layer cup-wall annulus geometry the spec produces.
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
use nalgebra::{Point3, UnitQuaternion, Vector3};

use crate::ribbon::Ribbon;

/// Cylindrical-pin registration spec. All dimensions in meters.
#[derive(Debug, Clone, PartialEq)]
pub struct PinSpec {
    /// Pin radius (m). Default `0.0015` = 1.5 mm = 3 mm diameter.
    pub pin_radius_m: f64,
    /// Half-length of each pin along the binormal axis (m).
    /// Default `0.005` = 5 mm → total pin length 10 mm.
    pub pin_half_length_m: f64,
    /// Arc-length fractions along the centerline where pins are
    /// placed. Default `vec![0.25, 0.75]` — 2 pins per
    /// layer-piece-pair, evenly spaced, no rotation possible.
    pub arc_fractions: Vec<f64>,
}

impl PinSpec {
    /// v2 iter-1 defaults: 3 mm diameter × 10 mm long pins at 25% +
    /// 75% of centerline arc length. Pin offset from the centerline
    /// along the ribbon's split-normal is derived per-layer in
    /// [`build_registration_solid`] (midpoint of the cup-wall
    /// annulus between the layer body and the bounding region),
    /// not stored on the spec — see the module docstring for the
    /// "derived, not fixed" rationale.
    #[must_use]
    pub fn iter1() -> Self {
        Self {
            pin_radius_m: 0.0015,
            pin_half_length_m: 0.005,
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

/// Maximum distance the surface-ray-march walks outward from the
/// centerline before giving up. 1 m comfortably covers every
/// workshop-scale device (typical body part scans fit in a
/// ≤200 mm cube); a probe that runs out of body geometry past this
/// surface returns `None` and the offending pin is dropped.
const PIN_RAY_MAX_REACH_M: f64 = 1.0;

/// Hard cap on exponential-bracket iterations inside
/// [`surface_distance_along_ray`]. Float comparisons drive the
/// outer loop, so this cap guarantees termination on pathological
/// inputs (NaN-tinted SDFs, etc.); the bracket reaches
/// [`PIN_RAY_MAX_REACH_M`] in ≤ 14 doublings from the 5 mm start,
/// so 32 is comfortably above the legitimate bound.
const PIN_RAY_BRACKET_MAX_ITERS: usize = 32;

/// Build the combined pin Solid for the ribbon's registration kind,
/// or `None` if registration is `None` or the pin spec has no arc
/// fractions.
///
/// Returns `Some(solid)` whose SDF is the union of all individual
/// pin cylinders (one cylinder per arc fraction in
/// [`PinSpec::arc_fractions`]). Each cylinder is positioned at
/// `centerline_sample(t) + pin_offset(t) * split_normal_vec` with
/// axis aligned to the ribbon's binormal at that centerline
/// position. `pin_offset(t)` is the midpoint of the cup-wall
/// annulus at that arc fraction — i.e., halfway between the layer
/// body's outer surface and the bounding region's outer surface
/// along the split-normal ray from the centerline sample.
///
/// `layer_body` and `bounding_region` are the same per-layer
/// [`Solid`]s the caller passes to
/// [`crate::piece::compose_piece_solid`]; the cup-wall annulus is
/// `bounding_region ∖ layer_body` at this layer.
///
/// The cylinders are deliberately NOT rotated to align with the
/// centerline's tangent — they're perpendicular to the seam, so
/// they extend equally into both [`crate::ribbon::PieceSide`] halves.
///
/// Returns `None` for any arc fraction whose split-normal ray runs
/// past [`PIN_RAY_MAX_REACH_M`] without crossing the
/// bounding-region surface; this signals a degenerate caller
/// (unbounded bounding region) and the surrounding piece
/// composition will produce a pin-less mold piece for this layer.
#[must_use]
pub fn build_registration_solid(
    ribbon: &Ribbon,
    layer_body: &Solid,
    bounding_region: &Solid,
) -> Option<Solid> {
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
        let body_dist = surface_distance_along_ray(layer_body, center, split_vec)?;
        let bounding_dist = surface_distance_along_ray(bounding_region, center, split_vec)?;
        let pin_offset = f64::midpoint(body_dist, bounding_dist);
        let pin_center = center + pin_offset * split_vec;
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

/// Walk `dir` outward from `origin` to find the distance `d ≥ 0` at
/// which `solid.evaluate(origin + d * dir) > 0` — i.e., the ray's
/// crossing of the solid's outer surface in this direction.
///
/// Exponential bracket + bisection. The short-circuit fires only
/// for an `origin` STRICTLY OUTSIDE the solid (`sd_origin > 0` with
/// a tiny tolerance for FP slack); a centerline sample that lands
/// exactly ON the solid's surface in some non-ray direction (e.g.,
/// the test fixture where the centerline endpoint touches the
/// body's `+X` face but the body still extends in `+Y`) falls
/// through to the loop, where the exponential bracket detects the
/// surface crossing along the ray and bisection nails it down.
///
/// Returns `None` only if the ray fails to exit the solid within
/// [`PIN_RAY_MAX_REACH_M`] (degenerate input — e.g., an unbounded
/// [`cf_design::Solid::plane`] passed as the bounding region).
fn surface_distance_along_ray(
    solid: &Solid,
    origin: Point3<f64>,
    dir: Vector3<f64>,
) -> Option<f64> {
    const OUTSIDE_SLACK: f64 = 1e-9;
    let sd_origin = solid.evaluate(&origin);
    if sd_origin > OUTSIDE_SLACK {
        return Some(0.0);
    }
    let mut lo = 0.0_f64;
    let mut hi = 0.005_f64;
    let mut found_outside = false;
    // Exponential bracket. Capped at PIN_RAY_BRACKET_MAX_ITERS so
    // floats never drive an infinite loop; max reach
    // `PIN_RAY_MAX_REACH_M` is reached well before the cap fires.
    for _ in 0..PIN_RAY_BRACKET_MAX_ITERS {
        if hi > PIN_RAY_MAX_REACH_M {
            break;
        }
        let p = origin + hi * dir;
        if solid.evaluate(&p) > OUTSIDE_SLACK {
            found_outside = true;
            break;
        }
        lo = hi;
        hi *= 2.0;
    }
    if !found_outside {
        return None;
    }
    for _ in 0..60 {
        let mid = f64::midpoint(lo, hi);
        let p = origin + mid * dir;
        if solid.evaluate(&p) <= OUTSIDE_SLACK {
            lo = mid;
        } else {
            hi = mid;
        }
        if (hi - lo) < 1e-7 {
            break;
        }
    }
    Some(f64::midpoint(lo, hi))
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

    /// Reference body/bounding pair for ray-march tests. Cuboid body
    /// with half-extent 60 mm in X (covers `straight_x_ribbon`'s
    /// `[-0.050, +0.050]` centerline span) and 10 mm in Y/Z;
    /// bounding cuboid 60 mm × 30 mm × 30 mm half-extents. The
    /// cup-wall annulus along the `+Y` split-normal spans
    /// `y ∈ [+10 mm, +30 mm]` and the pin should land at
    /// `y = +20 mm` (annulus midpoint).
    fn reference_body_and_bounds() -> (Solid, Solid) {
        let body = Solid::cuboid(Vector3::new(0.060, 0.010, 0.010));
        let bounds = Solid::cuboid(Vector3::new(0.060, 0.030, 0.030));
        (body, bounds)
    }

    #[test]
    fn pin_spec_iter1_has_workshop_defaults() {
        let s = PinSpec::iter1();
        assert!((s.pin_radius_m - 0.0015).abs() < f64::EPSILON);
        assert!((s.pin_half_length_m - 0.005).abs() < f64::EPSILON);
        assert_eq!(s.arc_fractions, vec![0.25, 0.75]);
    }

    #[test]
    fn registration_kind_default_is_none() {
        assert_eq!(RegistrationKind::default(), RegistrationKind::None);
    }

    #[test]
    fn build_registration_solid_none_returns_none() {
        let ribbon = straight_x_ribbon(); // default registration = None
        let (body, bounds) = reference_body_and_bounds();
        assert!(build_registration_solid(&ribbon, &body, &bounds).is_none());
    }

    #[test]
    fn build_registration_solid_pins_returns_some_with_finite_bounds() {
        let ribbon =
            straight_x_ribbon().with_registration(RegistrationKind::Pins(PinSpec::iter1()));
        let (body, bounds) = reference_body_and_bounds();
        let pins = build_registration_solid(&ribbon, &body, &bounds)
            .expect("pin spec should yield a solid");
        let aabb = pins
            .bounds()
            .expect("pin cylinders should have finite bounds");
        // 2 pins along +X at 25%/75% of 100 mm centerline → x ∈
        // [-0.025, +0.025] (centers). Each pin offset along +Y by
        // the annulus midpoint (10 + 30)/2 = 20 mm, centered at z=0
        // with half-length 5 mm. Pin radius 1.5 mm so x extent
        // ~[-0.025-0.0015, +0.025+0.0015]
        assert!(aabb.min.x < 0.0); // first pin at -25 mm
        assert!(aabb.max.x > 0.0); // second pin at +25 mm
        // y around +20 mm (annulus midpoint, NOT the legacy
        // hardcoded 25 mm offset).
        assert!(
            aabb.min.y > 0.018,
            "pin y_min should sit in the annulus around +20 mm; got {}",
            aabb.min.y,
        );
        assert!(
            aabb.max.y < 0.022,
            "pin y_max should sit in the annulus around +20 mm; got {}",
            aabb.max.y,
        );
        // z extends pin_half_length each side
        assert!(aabb.min.z <= -0.005 + 1e-9);
        assert!(aabb.max.z >= 0.005 - 1e-9);
    }

    #[test]
    fn build_registration_solid_empty_arc_fractions_returns_none() {
        let mut spec = PinSpec::iter1();
        spec.arc_fractions.clear();
        let ribbon = straight_x_ribbon().with_registration(RegistrationKind::Pins(spec));
        let (body, bounds) = reference_body_and_bounds();
        assert!(build_registration_solid(&ribbon, &body, &bounds).is_none());
    }

    #[test]
    fn pin_cylinders_are_inside_only_at_pin_positions() {
        // Pin at arc-fraction 0.25 of straight +X centerline (-0.05 → +0.05):
        //   centerline_sample = (-0.025, 0, 0)
        //   body surface along +Y: y = +0.010
        //   bounding surface along +Y: y = +0.030
        //   pin_offset = (0.010 + 0.030) / 2 = +0.020
        //   pin position      = (-0.025, +0.020, 0)
        //   axis              = +Z (binormal of straight +X with +Y split)
        //   radius 1.5 mm, half-length 5 mm
        let ribbon =
            straight_x_ribbon().with_registration(RegistrationKind::Pins(PinSpec::iter1()));
        let (body, bounds) = reference_body_and_bounds();
        let pins = build_registration_solid(&ribbon, &body, &bounds).unwrap();

        let pin_center = Point3::new(-0.025, 0.020, 0.0);
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

    /// Wider body fixture: half-extent 36 mm body + 61 mm bounding —
    /// mirrors iter-1 sock-over-capsule's `+X` geometry (body extent
    /// at layer 0 ≈ 41 mm, bounding ≈ 61 mm; cup-wall annulus
    /// `x ∈ [41 mm, 61 mm]`). Pin should land at the annulus
    /// midpoint, NOT at the legacy 25 mm fixed offset — the original
    /// iter-1 visual-gate falsification mode. Per `recon §F`.
    #[test]
    fn pin_position_stays_in_cup_wall_for_wide_body_iter1_regression() {
        let body = Solid::cuboid(Vector3::new(0.036, 0.036, 0.036));
        let bounds = Solid::cuboid(Vector3::new(0.061, 0.061, 0.061));
        // Centerline along +X, split-normal +X so the pin extends
        // along +X (the iter-1 production direction). +Y split would
        // require the cuboid to be wider in +Y than in +X; we keep
        // the cuboid symmetric and switch the split direction.
        let centerline = vec![Point3::new(0.0, 0.0, -0.050), Point3::new(0.0, 0.0, 0.050)];
        let split = SplitNormal::new(Vector3::new(1.0, 0.0, 0.0)).unwrap();
        let ribbon = Ribbon::new(centerline, split)
            .unwrap()
            .with_registration(RegistrationKind::Pins(PinSpec::iter1()));
        let pins = build_registration_solid(&ribbon, &body, &bounds)
            .expect("pin spec should yield a solid for wide-body fixture");
        let aabb = pins
            .bounds()
            .expect("pin cylinders should have finite bounds");
        // Annulus midpoint along +X = (36 + 61) / 2 = 48.5 mm. Pin
        // cylinder spans 48.5 ± pin_radius (1.5 mm) in x.
        assert!(
            aabb.min.x > 0.046,
            "pin AABB x_min should sit inside cup-wall annulus (>46 mm), \
             not at the legacy 25 mm offset; got {}",
            aabb.min.x,
        );
        assert!(
            aabb.max.x < 0.051,
            "pin AABB x_max should sit inside cup-wall annulus (<51 mm); got {}",
            aabb.max.x,
        );
        // Pin t=0.25 is centered at (+0.0485, 0, -0.025) (annulus
        // midpoint along +X, sample z at 25% of [-0.050, +0.050]).
        // Pin axis is the local binormal +Y; cylinder extends
        // ±5 mm along Y. Query at pin center → inside pin (SDF < 0);
        // also OUTSIDE the body and INSIDE the bounding region.
        let pin_center_t025 = Point3::new(0.0485, 0.0, -0.025);
        assert!(
            body.evaluate(&pin_center_t025) > 0.0,
            "pin axis should be OUTSIDE the body (in the cup wall)",
        );
        assert!(
            bounds.evaluate(&pin_center_t025) < 0.0,
            "pin axis should be INSIDE the bounding region (in the cup wall)",
        );
        assert!(
            pins.evaluate(&pin_center_t025) < 0.0,
            "pin SDF at the annulus-midpoint pin center should be negative; got {}",
            pins.evaluate(&pin_center_t025),
        );
    }

    /// Origin strictly OUTSIDE the solid (`sd_origin > 0`) →
    /// short-circuit to 0 (the ray has already exited). Origin ON
    /// the surface in a non-ray axis (`sd_origin == 0` but the body
    /// extends further along the ray) → ray-march detects the true
    /// surface crossing.
    #[test]
    fn surface_distance_short_circuits_only_for_strictly_outside_origin() {
        let body = Solid::cuboid(Vector3::new(0.010, 0.010, 0.010));
        let bounds = Solid::cuboid(Vector3::new(0.030, 0.030, 0.030));
        // (a) Origin slightly OUTSIDE body in +Y (y = +11 mm): body
        // SDF > 0 → short-circuit to body_d = 0.
        let outside = Point3::new(0.0, 0.011, 0.0);
        let dir = Vector3::new(0.0, 1.0, 0.0);
        let body_d_outside = surface_distance_along_ray(&body, outside, dir).unwrap();
        assert!(
            (body_d_outside - 0.0).abs() < 1e-12,
            "body_d_outside = {body_d_outside}",
        );
        // Bounding surface along +Y from outside = 0.030 - 0.011 = 0.019.
        let bounds_d_outside = surface_distance_along_ray(&bounds, outside, dir).unwrap();
        assert!(
            (bounds_d_outside - 0.019).abs() < 1e-6,
            "bounds_d_outside = {bounds_d_outside} (expected ≈0.019)",
        );
        // (b) Origin ON body's +X face (`x = +0.010`, y = z = 0):
        // body SDF = 0 (on the surface in x), but the body extends
        // further along +Y (it's a cube). Ray-march should still
        // find the +Y surface crossing at y = 0.010.
        let on_face = Point3::new(0.010, 0.0, 0.0);
        let body_d_on_face = surface_distance_along_ray(&body, on_face, dir).unwrap();
        assert!(
            (body_d_on_face - 0.010).abs() < 1e-6,
            "body_d_on_face = {body_d_on_face} (expected ≈0.010 — body extends \
             +Y from y=0 to y=0.010 along the ray)",
        );
    }
}
