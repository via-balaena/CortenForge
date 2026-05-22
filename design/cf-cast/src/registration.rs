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
//! dovetails or magnets): simplest geometry (one cylinder per pin —
//! pre-S5 an SDF [`Solid::cylinder`], post-S5 a manifold3d cylinder
//! primitive via [`crate::mesh_csg::build_cylinder_along_axis`]),
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
//! - Count: 4 per layer-piece-pair — at arc-length fractions
//!   `0.25` and `0.75` along the centerline, mirrored across the
//!   split-normal axis (one pin at `+split_normal` offset and one
//!   at `-split_normal` offset per arc fraction). The four pins
//!   give each cup piece two lateral interlock columns instead of
//!   one, doubling the alignment constraint against rotation +
//!   parting-plane shear.
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
//! Pin cylinders are emitted as
//! [`crate::mesh_csg::MatingTransform::UnionCylinder`] on the
//! [`crate::ribbon::PieceSide::Negative`] piece (gain protrusions)
//! and as [`crate::mesh_csg::MatingTransform::SubtractCylinder`] on
//! the [`crate::ribbon::PieceSide::Positive`] piece (gain matching
//! sockets, slightly inflated radius + depth per the per-piece
//! clearance budget). [`crate::piece::compose_piece_solid`] consults
//! [`crate::ribbon::Ribbon::registration`] and appends the pin
//! transforms to the returned `Vec<MatingTransform>` so the post-MC
//! mesh-CSG stage materializes the geometry; callers don't manage
//! pin geometry explicitly.
//!
//! S5 of `docs/CF_CAST_MATING_FEATURES_PLAN.md` migrated this path
//! from an SDF [`Solid::union`] / [`Solid::subtract`] (MC-resolved at
//! marching-cubes grid resolution) to the post-MC mesh-CSG stage —
//! exact cylinder primitives via [`crate::mesh_csg::build_cylinder_along_axis`]
//! replace MC-discretized pin geometry. The pin and socket of a
//! registration pair share `(center, axis)` derived from the same
//! arc-fraction sample; only `half_length` (extended by
//! [`PinSpec::axial_clearance_m`] / 2) and radius (extended by
//! [`PinSpec::diametral_clearance_m`] / 2) differ between the two.
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
use nalgebra::{Point3, UnitVector3, Vector3};

use crate::mesh_csg::{CylinderParams, CylinderParent, MatingTransform};
use crate::ribbon::{PieceSide, Ribbon};

/// Polygonal facet count around the pin cylinder's circumference.
///
/// 32-segment circles are the workshop default for 3 mm Ø pins
/// (recon §2): the chord error at radius 1.5 mm is
/// `r(1 - cos(π/32)) ≈ 7 µm`, comfortably below FDM bead width
/// (~0.4 mm). Part of the determinism contract — same
/// [`CylinderParent`] + same radius + same `PIN_SEGMENTS` → bit-equal
/// output across calls and across pieces.
pub(crate) const PIN_SEGMENTS: u32 = 32;

/// Cylindrical-pin registration spec. All dimensions in meters.
#[derive(Debug, Clone, PartialEq)]
pub struct PinSpec {
    /// Pin radius (m). Default `0.0015` = 1.5 mm = 3 mm diameter.
    pub pin_radius_m: f64,
    /// Half-length of each pin along the binormal axis (m).
    /// Default `0.005` = 5 mm → total pin length 10 mm.
    pub pin_half_length_m: f64,
    /// Arc-length fractions along the centerline where pins are
    /// placed. Default `vec![0.25, 0.75]`. Each arc fraction yields
    /// TWO pins — one on each lateral side of the centerline
    /// (at `+split_normal` and `-split_normal` offsets). With the
    /// default two fractions, total pin count is 4 per
    /// layer-piece-pair, giving each cup piece two lateral
    /// interlock columns + axial pin pairs.
    pub arc_fractions: Vec<f64>,
    /// Diametral clearance between pin and socket (m). The socket's
    /// radius is `pin_radius_m + diametral_clearance_m / 2` so the
    /// pin-vs-socket diameter difference equals
    /// `diametral_clearance_m`. v2 iter-1 default `0.00020` =
    /// 0.20 mm matches FDM precision; recon §9 baseline for M2
    /// pins (positional sliding fit).
    pub diametral_clearance_m: f64,
    /// Axial socket-bottom relief (m). The socket parent's
    /// `half_length` is extended by `axial_clearance_m / 2` so the
    /// socket *cylinder primitive* runs `axial_clearance_m / 2` past
    /// the pin cylinder on each axial face (symmetric per the
    /// `diametral_clearance / 2 → radius` convention). Per-piece
    /// `SeamTrim` removes the near-seam half of each cylinder, so
    /// the workshop-meaningful component is the AWAY-from-seam
    /// extension — a modest bottom-of-pocket relief beyond a
    /// pin-half-length-only socket. Workshop assembly engagement is
    /// dominated by [`crate::piece::RIBBON_PIECE_OVERLAP_M`] (the
    /// pin's protrusion past the seam plane after trim), not by this
    /// field; the iter-1 0.50 mm baseline is preserved for spec
    /// completeness + future asymmetric-cylinder migrations. Recon
    /// §9 baseline for M2 pins.
    pub axial_clearance_m: f64,
}

impl PinSpec {
    /// v2 iter-1 defaults: 3 mm diameter × 10 mm long pins at 25% +
    /// 75% of centerline arc length, 0.20 mm diametral × 0.50 mm
    /// axial clearance (recon §9 M2 baseline — positional sliding
    /// fit). Pin offset from the centerline along the ribbon's
    /// split-normal is derived per-layer in
    /// [`build_registration_transforms`] (midpoint of the cup-wall
    /// annulus between the layer body and the bounding region), not
    /// stored on the spec — see the module docstring for the
    /// "derived, not fixed" rationale.
    #[must_use]
    pub fn iter1() -> Self {
        Self {
            pin_radius_m: 0.0015,
            pin_half_length_m: 0.005,
            arc_fractions: vec![0.25, 0.75],
            diametral_clearance_m: 0.00020,
            axial_clearance_m: 0.00050,
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
/// [`PIN_RAY_MAX_REACH_M`] in ≤ 8 doublings from the 5 mm start
/// (`0.005 × 2^8 = 1.28 m > 1.0 m`), so 32 is comfortably above the
/// legitimate bound.
const PIN_RAY_BRACKET_MAX_ITERS: usize = 32;

/// Build the per-piece pin/socket mating transforms for the ribbon's
/// registration kind.
///
/// Returns a `Vec<MatingTransform>` containing one
/// [`MatingTransform::UnionCylinder`] per pin position for
/// [`PieceSide::Negative`], or one
/// [`MatingTransform::SubtractCylinder`] per pin position for
/// [`PieceSide::Positive`]. Pin and socket of a registration pair
/// share `(center, axis)` derived from the same arc-fraction sample
/// (and lateral mirror sign); the socket's `half_length` is extended
/// by [`PinSpec::axial_clearance_m`] / 2 and its `radius_m` by
/// [`PinSpec::diametral_clearance_m`] / 2 so the pin seats with
/// symmetric per-side clearance (matching the `/2` convention the
/// diametral budget uses).
///
/// Returns an empty `Vec` when:
/// - `ribbon.registration` is [`RegistrationKind::None`];
/// - the spec has no arc fractions;
/// - any pin's split-normal ray runs past the module-private
///   [`PIN_RAY_MAX_REACH_M`] (1 m) without crossing the
///   bounding-region surface (signals a degenerate caller).
///
/// Each pin is positioned at
/// `centerline_sample(t) + pin_offset(t) * sign * split_normal_vec`
/// with axis aligned to the ribbon's binormal at that centerline
/// position. `pin_offset(t)` is the midpoint of the cup-wall annulus
/// at that arc fraction — halfway between the layer body's outer
/// surface and the bounding region's outer surface along the
/// split-normal ray from the centerline sample.
///
/// `layer_body` and `bounding_region` are the same per-layer
/// [`Solid`]s the caller passes to
/// [`crate::piece::compose_piece_solid`]; the cup-wall annulus is
/// `bounding_region ∖ layer_body` at this layer.
///
/// The cylinders are deliberately NOT rotated to align with the
/// centerline's tangent — they're perpendicular to the seam, so
/// they extend equally into both [`PieceSide`] halves (later
/// bisected by the cup-piece [`MatingTransform::SeamTrim`] in
/// recon §5's transform order: cylinder ops before seam trim).
#[must_use]
pub fn build_registration_transforms(
    ribbon: &Ribbon,
    layer_body: &Solid,
    bounding_region: &Solid,
    side: PieceSide,
) -> Vec<MatingTransform> {
    let spec = match &ribbon.registration {
        RegistrationKind::None => return Vec::new(),
        RegistrationKind::Pins(spec) => spec,
    };
    if spec.arc_fractions.is_empty() {
        return Vec::new();
    }
    let split_vec = ribbon.split_normal.as_vector();
    // For each arc fraction, build TWO pins — one on each lateral
    // side of the centerline along ±split_normal. The split-normal
    // direction defines the mold's lateral "left/right" axis (the
    // axis the cup pieces open along); putting pins on both lateral
    // sides gives each cup piece two interlock columns instead of
    // one, doubling the alignment constraint. For asymmetric scans
    // (body wider on one side of the centerline than the other),
    // each pin's offset is computed independently — the +side and
    // -side annulus midpoints can differ.
    let mut transforms = Vec::with_capacity(spec.arc_fractions.len() * 2);
    for &t in &spec.arc_fractions {
        let Some((center, _tangent, binormal)) = ribbon.sample_at_arc_fraction(t) else {
            return Vec::new();
        };
        // `binormal` is unit-length by ribbon construction
        // (`tangent × N_split` normalized); renormalize defensively
        // so a degenerate (zero) binormal vector returns the
        // Z-fallback rather than panicking on UnitVector3 invariants.
        let axis = UnitVector3::new_normalize(binormal);
        for &sign in &[1.0_f64, -1.0_f64] {
            let ray_dir = sign * split_vec;
            let Some(body_dist) = surface_distance_along_ray(layer_body, center, ray_dir) else {
                return Vec::new();
            };
            let Some(bounding_dist) = surface_distance_along_ray(bounding_region, center, ray_dir)
            else {
                return Vec::new();
            };
            let pin_offset = f64::midpoint(body_dist, bounding_dist);
            let pin_center = center + pin_offset * ray_dir;
            transforms.push(pin_transform_for_side(spec, pin_center, axis, side));
        }
    }
    transforms
}

/// Build the per-side cylinder transform for one pin location.
///
/// Negative side gets [`MatingTransform::UnionCylinder`] at the pin
/// radius and pin half-length. Positive side gets
/// [`MatingTransform::SubtractCylinder`] with a socket that's
/// inflated by [`PinSpec::diametral_clearance_m`] / 2 radially and
/// [`PinSpec::axial_clearance_m`] / 2 axially (symmetric `/2`
/// convention — the socket *cylinder primitive* extends past the
/// pin cylinder on each axial face). Per-piece `SeamTrim` removes
/// the near-seam half of each cylinder, so only the AWAY-from-seam
/// extension survives in the printed STL — a modest socket-bottom
/// relief; see [`PinSpec::axial_clearance_m`] for the workshop-
/// engagement caveat.
fn pin_transform_for_side(
    spec: &PinSpec,
    pin_center: Point3<f64>,
    axis: UnitVector3<f64>,
    side: PieceSide,
) -> MatingTransform {
    match side {
        PieceSide::Negative => MatingTransform::UnionCylinder {
            params: CylinderParams {
                parent: CylinderParent {
                    center_m: pin_center,
                    axis,
                    half_length_m: spec.pin_half_length_m,
                },
                radius_m: spec.pin_radius_m,
                segments: PIN_SEGMENTS,
            },
        },
        PieceSide::Positive => MatingTransform::SubtractCylinder {
            params: CylinderParams {
                parent: CylinderParent {
                    center_m: pin_center,
                    axis,
                    half_length_m: spec.pin_half_length_m + spec.axial_clearance_m / 2.0,
                },
                radius_m: spec.pin_radius_m + spec.diametral_clearance_m / 2.0,
                segments: PIN_SEGMENTS,
            },
        },
    }
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

    /// Extract `CylinderParams` from a [`MatingTransform`] for test
    /// inspection. Both Union and Subtract cylinder variants carry
    /// the same payload type; tests focus on the params rather than
    /// the variant when checking pin position / radius / clearance.
    fn params_of(t: &MatingTransform) -> &CylinderParams {
        match t {
            MatingTransform::UnionCylinder { params }
            | MatingTransform::SubtractCylinder { params } => params,
            MatingTransform::SeamTrim { .. } => {
                panic!("registration transforms should never include SeamTrim")
            }
        }
    }

    #[test]
    fn pin_spec_iter1_has_workshop_defaults() {
        let s = PinSpec::iter1();
        assert!((s.pin_radius_m - 0.0015).abs() < f64::EPSILON);
        assert!((s.pin_half_length_m - 0.005).abs() < f64::EPSILON);
        assert_eq!(s.arc_fractions, vec![0.25, 0.75]);
        // S5 clearance defaults — recon §9 M2 baseline (positional
        // sliding fit). 0.20 mm diametral × 0.50 mm axial.
        assert!((s.diametral_clearance_m - 0.00020).abs() < f64::EPSILON);
        assert!((s.axial_clearance_m - 0.00050).abs() < f64::EPSILON);
    }

    #[test]
    fn registration_kind_default_is_none() {
        assert_eq!(RegistrationKind::default(), RegistrationKind::None);
    }

    #[test]
    fn build_registration_transforms_none_returns_empty() {
        let ribbon = straight_x_ribbon(); // default registration = None
        let (body, bounds) = reference_body_and_bounds();
        assert!(
            build_registration_transforms(&ribbon, &body, &bounds, PieceSide::Negative).is_empty()
        );
        assert!(
            build_registration_transforms(&ribbon, &body, &bounds, PieceSide::Positive).is_empty()
        );
    }

    #[test]
    fn build_registration_transforms_pins_returns_one_per_pin_per_side() {
        // 4 pins total: 2 arc fractions × 2 lateral mirror sides.
        let ribbon =
            straight_x_ribbon().with_registration(RegistrationKind::Pins(PinSpec::iter1()));
        let (body, bounds) = reference_body_and_bounds();
        let neg = build_registration_transforms(&ribbon, &body, &bounds, PieceSide::Negative);
        let pos = build_registration_transforms(&ribbon, &body, &bounds, PieceSide::Positive);
        assert_eq!(
            neg.len(),
            4,
            "4 pins per piece (2 arc fractions × 2 lateral mirrors)"
        );
        assert_eq!(
            pos.len(),
            4,
            "Positive side emits the same pin count as Negative"
        );
        // Negative emits UnionCylinder; Positive emits SubtractCylinder.
        for t in &neg {
            assert!(
                matches!(t, MatingTransform::UnionCylinder { .. }),
                "Negative side should emit UnionCylinder; got {t:?}",
            );
        }
        for t in &pos {
            assert!(
                matches!(t, MatingTransform::SubtractCylinder { .. }),
                "Positive side should emit SubtractCylinder; got {t:?}",
            );
        }
    }

    #[test]
    fn build_registration_transforms_empty_arc_fractions_returns_empty() {
        let mut spec = PinSpec::iter1();
        spec.arc_fractions.clear();
        let ribbon = straight_x_ribbon().with_registration(RegistrationKind::Pins(spec));
        let (body, bounds) = reference_body_and_bounds();
        assert!(
            build_registration_transforms(&ribbon, &body, &bounds, PieceSide::Negative).is_empty()
        );
    }

    /// The pin position invariant: each Negative-side pin's
    /// `CylinderParent::center_m` lands at the annulus midpoint, NOT
    /// inside the body (where pre-C1 iter-1 fixture made the pin
    /// free-float). Replaces the pre-S5 pin-SDF inside-only assertion
    /// — post-S5 the pin geometry is mesh-CSG, but the COMPOSITION
    /// position invariant lives in the transform parameters and is
    /// directly inspectable without a manifold3d round-trip.
    #[test]
    fn pin_transforms_position_each_pin_at_annulus_midpoint() {
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
        let neg = build_registration_transforms(&ribbon, &body, &bounds, PieceSide::Negative);

        // Expect 4 pin centers — 2 arc fractions × 2 lateral sides.
        let centers: Vec<Point3<f64>> = neg.iter().map(|t| params_of(t).parent.center_m).collect();
        let expected = [
            Point3::new(-0.025, 0.020, 0.0),  // t=0.25, +Y
            Point3::new(-0.025, -0.020, 0.0), // t=0.25, -Y
            Point3::new(0.025, 0.020, 0.0),   // t=0.75, +Y
            Point3::new(0.025, -0.020, 0.0),  // t=0.75, -Y
        ];
        for e in &expected {
            let matched = centers.iter().any(|c| {
                (c.x - e.x).abs() < 1e-6 && (c.y - e.y).abs() < 1e-6 && (c.z - e.z).abs() < 1e-6
            });
            assert!(
                matched,
                "expected pin center at {e:?}, got centers {centers:?}"
            );
        }
        // Pin radius + half-length unchanged from spec.
        for t in &neg {
            let p = params_of(t);
            assert!((p.radius_m - 0.0015).abs() < f64::EPSILON);
            assert!((p.parent.half_length_m - 0.005).abs() < f64::EPSILON);
        }
    }

    /// Positive-side socket params carry the diametral + axial
    /// clearance inflation; pin and socket share `center_m` + `axis`
    /// per pin position. The `mesh_csg::tests` math invariant builds
    /// parents inline, so without this test a regression in
    /// `pin_transform_for_side`'s Positive arm (e.g., a stray `* 2.0`
    /// or dropped `/ 2.0`) could land without any test surfacing it.
    #[test]
    fn pin_transforms_positive_socket_params_inflate_per_spec() {
        let ribbon =
            straight_x_ribbon().with_registration(RegistrationKind::Pins(PinSpec::iter1()));
        let (body, bounds) = reference_body_and_bounds();
        let neg = build_registration_transforms(&ribbon, &body, &bounds, PieceSide::Negative);
        let pos = build_registration_transforms(&ribbon, &body, &bounds, PieceSide::Positive);
        assert_eq!(
            neg.len(),
            pos.len(),
            "Negative and Positive emit one transform per pin"
        );

        let spec = PinSpec::iter1();
        let expected_socket_radius = spec.pin_radius_m + spec.diametral_clearance_m / 2.0;
        let expected_socket_half_length = spec.pin_half_length_m + spec.axial_clearance_m / 2.0;

        // Each Positive socket pairs with the same-position Negative
        // pin via shared center+axis. Match Positive transforms back
        // to Negative by `parent.center_m` (per-position bijection).
        for (i, pos_t) in pos.iter().enumerate() {
            let pos_params = params_of(pos_t);
            let neg_match = neg
                .iter()
                .find(|n| {
                    let np = params_of(n);
                    np.parent.center_m == pos_params.parent.center_m
                })
                .unwrap_or_else(|| {
                    panic!("Positive transform {i} has no Negative match by center_m")
                });
            let neg_params = params_of(neg_match);

            // Shared center + axis.
            assert_eq!(neg_params.parent.center_m, pos_params.parent.center_m);
            assert_eq!(neg_params.parent.axis, pos_params.parent.axis);
            // Negative side carries pin geometry verbatim.
            assert!((neg_params.radius_m - spec.pin_radius_m).abs() < f64::EPSILON);
            assert!(
                (neg_params.parent.half_length_m - spec.pin_half_length_m).abs() < f64::EPSILON
            );
            // Positive side carries the symmetric `/2` inflation.
            assert!(
                (pos_params.radius_m - expected_socket_radius).abs() < f64::EPSILON,
                "Positive socket radius should be pin + diametral/2 = {expected_socket_radius}; \
                 got {}",
                pos_params.radius_m,
            );
            assert!(
                (pos_params.parent.half_length_m - expected_socket_half_length).abs()
                    < f64::EPSILON,
                "Positive socket half_length should be pin + axial/2 = {expected_socket_half_length}; \
                 got {}",
                pos_params.parent.half_length_m,
            );
            // Determinism contract — same segments across sides.
            assert_eq!(neg_params.segments, pos_params.segments);
        }
    }

    /// Wider body fixture: half-extent 36 mm body + 61 mm bounding —
    /// mirrors iter-1 sock-over-capsule's `+X` geometry (body extent
    /// at layer 0 ≈ 41 mm, bounding ≈ 61 mm; cup-wall annulus
    /// `x ∈ [41 mm, 61 mm]`). Each pin's `CylinderParent::center_m`
    /// should land at the annulus midpoint, NOT at the legacy 25 mm
    /// fixed offset — the original iter-1 visual-gate falsification
    /// mode (workshop iter-1 mold recon §F). Pre-S5 the test
    /// asserted the pin-SDF inside-only; post-S5 the equivalent
    /// invariant inspects the [`CylinderParent::center_m`] each
    /// transform carries directly.
    #[test]
    fn pin_transforms_position_stays_in_cup_wall_for_wide_body_iter1_regression() {
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
        let neg = build_registration_transforms(&ribbon, &body, &bounds, PieceSide::Negative);
        assert_eq!(neg.len(), 4, "wide-body fixture should still emit 4 pins");

        // For each pin, the parent center must be:
        // - OUTSIDE the body (cup-wall material, body.evaluate > 0),
        // - INSIDE the bounding region (cup-wall material, bounds.evaluate < 0),
        // - At |x| ≈ (36 + 61) / 2 = 48.5 mm (annulus midpoint along ±X).
        for t in &neg {
            let c = params_of(t).parent.center_m;
            assert!(
                body.evaluate(&c) > 0.0,
                "pin center {c:?} must be OUTSIDE the body (in cup wall); body.evaluate = {}",
                body.evaluate(&c),
            );
            assert!(
                bounds.evaluate(&c) < 0.0,
                "pin center {c:?} must be INSIDE the bounding region (in cup wall); \
                 bounds.evaluate = {}",
                bounds.evaluate(&c),
            );
            assert!(
                (c.x.abs() - 0.0485).abs() < 1e-6,
                "pin center {c:?} should land at the annulus midpoint |x| ≈ 48.5 mm",
            );
        }
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
