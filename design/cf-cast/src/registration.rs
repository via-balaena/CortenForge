//! Inter-piece registration features for v2 multi-piece molds.
//!
//! v2 iter-1 surfaced a workshop pain point: hand-clamping two
//! ribbon-cut mold pieces with rubber bands works but is fiddly,
//! and the pieces can shift along the seam during pour.
//! Registration features — geometry composed into the mold pieces —
//! lock the pieces in alignment.
//!
//! # Architectural history
//!
//! Step 9 of `docs/CURVE_FOLLOWING_DESIGN.md` originally shipped
//! **cylindrical pins** (vs the design-doc-§Open-questions §1
//! alternatives of dovetails or magnets) — pre-S5 an SDF
//! [`Solid::cylinder`], post-S5 a manifold3d cylinder primitive via
//! `crate::mesh_csg::build_cylinder_along_axis`. Workshop iter-2
//! printed cleanly on Bambu Lab calibrated FDM but the consumer-FDM
//! target floor (Bambu A1 + default + Jayo per the FDM-friendly
//! geometry recon §G-3) struggles with cylinder-on-cylinder facet-
//! vs-facet contact + first-layer elephant foot. Recon-1
//! (`docs/CF_CAST_FDM_FRIENDLY_GEOMETRY_RECON.md`) §G-2 picked
//! trapezoidal / truncated-pyramid pins (sharp angles, flat faces,
//! straight lines — what FDM gantry kinematics produce cleanly)
//! over the cylindrical design.
//!
//! S3 of the FDM-friendly geometry implementation arc migrates this
//! module from the mesh-CSG cylinder primitive
//! (`MatingTransform::UnionCylinder` / `SubtractCylinder` consuming
//! a shared [`crate::mesh_csg::CylinderParent`]) to the SDF-side
//! [`crate::PrismaticPin`][`crate::prismatic_pin`] primitive
//! composed pre-MC via [`Solid::union`] / [`Solid::subtract`]. The
//! mesh-CSG `MatingTransform` variants stay for the cup pour-gate
//! carve (S7 of the prior mating-features arc) but no longer carry
//! the cup-pin payload.
//!
//! # Paradigm-boundary placement
//!
//! The S1 probe-spike characterised mesh-CSG `Manifold::union` of a
//! `Manifold::hull_pts` truncated-pyramid pin against an SDF→MC
//! curved-shell host (`design/cf-cast/tests/g7_g9_prismatic_pin_probe.rs`,
//! commit `a218ddfb`). Outcome at both production (3 mm) and
//! over-resolved (1 mm) cell sizes: the union introduces 2
//! disconnected components AND a `SelfIntersecting` F4 Critical
//! issue at the boolean junction (§G-7 BRANCH B + BRANCH C BOTH
//! fire). The §G-12 #2 bail-out (pin composed pre-MC into the host
//! SDF via [`Solid::union`]) PASSES the BRANCH-A criteria — 1
//! connected component, no new F4 Critical types at the junction.
//! See [`crate::prismatic_pin`] module docstring for the full
//! probe outcome + paradigm-boundary framing (recon-4 (P) "bulk-
//! welded SDF/MC × fine mesh-CSG" boundary).
//!
//! # Pin pose convention
//!
//! Each cup-pin's pose is derived per-(layer × ribbon-frame × arc-
//! fraction × lateral-mirror) by ray-marching the layer body +
//! bounding region SDFs from each `centerline_sample(arc_fraction)`:
//!
//! - **`center_m` = annulus midpoint** of the cup-wall: halfway
//!   along the ribbon's split-normal between the layer body's outer
//!   surface and the bounding region's outer surface. Pre-recon-2 /
//!   recon-4 (P) restored this from recon-3 (α)'s bounds-anchored
//!   variant — the annulus-midpoint pin is contained in cup-wall
//!   material with `RIBBON_PIECE_OVERLAP_M`-sized overlap into the
//!   Negative half-shell, so SDF union absorbs it without
//!   disconnection.
//! - **`axis_unit` = ribbon binormal** at the centerline sample
//!   (perpendicular to the seam plane). The pin extends
//!   `±half_length_m` along binormal symmetrically across the seam
//!   plane; the `-binormal` half lives inside the Negative half-
//!   shell material (provides SDF-union connectivity) and the
//!   `+binormal` half protrudes past the seam face as the workshop-
//!   visible registration ridge.
//! - **`lateral_unit` = ribbon split-normal**. The
//!   [`crate::PrismaticPinPose`] requires a deterministic lateral
//!   axis even for square-base pins (where the rotation about
//!   `axis_unit` is geometrically immaterial). Split-normal is
//!   picked over tangent because it's the lateral mirror axis along
//!   which sibling pins are placed — same direction the workshop
//!   user perceives as the cup's "left/right" — and because
//!   split-normal ⊥ binormal by ribbon construction
//!   (`binormal = tangent × split_normal_unit`), so the orthogonality
//!   contract on [`crate::PrismaticPinPose::new`] holds without an
//!   extra Gram-Schmidt step.
//!
//! # Composition
//!
//! Pin SDFs are built via [`crate::build_prismatic_pin_sdf`] and
//! returned as a `Vec<Solid>` per piece side by
//! [`build_registration_sdf_ops`]. [`crate::piece::compose_piece_solid`]
//! consumes the returned solids:
//!
//! - [`PieceSide::Negative`] receives PIN solids — each unioned into
//!   the per-piece half-shell `Solid` pre-MC. The pin's `-binormal`
//!   half overlaps cup-wall material (SDF-union connectivity);
//!   `+binormal` half protrudes past the half-shell as the workshop-
//!   visible ridge.
//! - [`PieceSide::Positive`] receives SOCKET solids (same pose, each
//!   extent inflated per [`crate::PrismaticPinSpec`]'s symmetric
//!   `/2` clearance convention) — each subtracted from the per-piece
//!   half-shell `Solid` pre-MC. The socket's `+binormal` half
//!   carves a matching cavity from Positive cup-wall material;
//!   `-binormal` half extends past the half-shell as a no-op
//!   (subtraction outside the kept material doesn't affect the
//!   result).
//!
//! # Bit-precise fit invariant
//!
//! Pin (Negative side) and socket (Positive side) of a registration
//! pair derive from the **same** [`crate::PrismaticPinPose`] (per-pin
//! `center_m`, `axis_unit`, `lateral_unit` triple), so the SDF input
//! layer is bit-identical between the two sides modulo the socket's
//! inflated extents. [`crate::PrismaticPinSpec::pin_params`] and
//! [`crate::PrismaticPinSpec::socket_params`] share the same pose
//! input and diverge only by the symmetric `/2` extent inflation —
//! the bit-precise fit invariant (analog to S5's cylinder-primitive
//! `pin_and_socket_fit_invariant`) is preserved at the spec layer
//! and exercised by `tests::cup_prismatic_pin_and_socket_fit_invariant`
//! (private module-level test in this file).
//!
//! # First-layer chamfer
//!
//! [`crate::PrismaticPinSpec::cup_pin_default`] sets a 0.6 mm
//! base-end chamfer band per recon-1 §G-6 / §G-8. The chamfer band
//! lives at the pin's `-half_length` axial end (deep inside the
//! Negative cup wall under the symmetric-across-seam pose). Print
//! orientation discipline that places the chamfer band at the bed-
//! adjacent face is a §G-4 procedure.rs concern — the SDF emission
//! is pose-symmetric and emits the chamfer band regardless of which
//! end the workshop user prints down.
//!
//! # Default off
//!
//! [`Ribbon::new`] sets `registration = RegistrationKind::None`
//! for backward compatibility — Steps 5-8 of the v2 arc all
//! pre-date this module, and no v2 test fixture or example crate
//! is expected to opt into pins without an explicit
//! [`crate::ribbon::Ribbon::with_registration`] call. The Step 11
//! example + cf-cast-cli `[registration_pins].enabled = true`
//! turn it on.
//!
//! [`Ribbon::new`]: crate::ribbon::Ribbon::new

use cf_design::Solid;
use nalgebra::{Point3, UnitVector3, Vector3};

use crate::prismatic_pin::{PrismaticPinPose, PrismaticPinSpec, build_prismatic_pin_sdf};
use crate::ribbon::{PieceSide, Ribbon};

/// Cup-pin registration spec — wraps the SDF-side
/// [`PrismaticPinSpec`] primitive with the per-cup-side arc-fraction
/// placement list.
///
/// `pin_spec` is the per-pin geometry (rectangular base half-extents,
/// taper to flat tip, optional base-end chamfer band, diametral +
/// axial clearance for the matching socket) — shared between every
/// pin on the cup-piece. `arc_fractions` lists where along the
/// centerline pins are placed; each fraction yields TWO pins (one
/// per lateral-mirror side along `±split_normal`), so a default
/// `vec![0.25, 0.75]` gives 4 pins per cup-piece pair.
#[derive(Debug, Clone, PartialEq)]
pub struct PinSpec {
    /// Per-pin SDF-side primitive spec. Default
    /// `PrismaticPinSpec::cup_pin_default()` per recon-1 §G-6 /
    /// §G-8: 1.5 mm half-extent square base, 1.2 mm half-extent
    /// square tip, 3 mm half-length, 0.6 mm chamfer, 0.30 mm
    /// diametral, 0.50 mm axial clearance.
    pub pin_spec: PrismaticPinSpec,
    /// Arc-length fractions along the centerline where pins are
    /// placed. Each fraction yields TWO pins — one on each lateral
    /// side of the centerline (at `+split_normal` and `-split_normal`
    /// offsets). With the default two fractions, total pin count is
    /// 4 per layer-piece-pair (2 arc fractions × 2 lateral mirrors),
    /// giving each cup-piece two lateral interlock columns + axial
    /// pin pairs.
    pub arc_fractions: Vec<f64>,
}

impl PinSpec {
    /// v2 iter-3 defaults (post-S3): cup-pin geometry per
    /// [`PrismaticPinSpec::cup_pin_default`] (1.5 mm half-extent
    /// base, taper to 1.2 mm tip, 3 mm half-length, 0.6 mm base
    /// chamfer, 0.30 mm diametral × 0.50 mm axial clearance) at
    /// arc fractions `[0.25, 0.75]` — 4 pins per cup-piece-pair.
    #[must_use]
    pub fn iter1() -> Self {
        Self {
            pin_spec: PrismaticPinSpec::cup_pin_default(),
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
/// with rubber bands. `Pins(PinSpec)` enables Step 9's registration
/// pins (cylindrical pre-S3 of the FDM-friendly geometry arc;
/// truncated-pyramid post-S3).
#[derive(Debug, Clone, Default, PartialEq)]
pub enum RegistrationKind {
    /// No registration features. Workshop hand-clamps the pieces;
    /// the v2 procedure.md surfaces the rubber-band approach.
    #[default]
    None,
    /// Trapezoidal / truncated-pyramid pins per [`PinSpec`].
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
/// `PIN_RAY_MAX_REACH_M` in ≤ 8 doublings from the 5 mm start
/// (`0.005 × 2^8 = 1.28 m > 1.0 m`), so 32 is comfortably above the
/// legitimate bound.
const PIN_RAY_BRACKET_MAX_ITERS: usize = 32;

/// Build the per-piece cup-pin SDF composition list for the ribbon's
/// registration kind.
///
/// Returns a `Vec<Solid>` of [`crate::PrismaticPin`][`crate::prismatic_pin`]
/// solids:
///
/// - [`PieceSide::Negative`] returns **pin** solids (per
///   [`PrismaticPinSpec::pin_params`]) — callers UNION each into the
///   per-piece [`Solid`] pre-MC.
/// - [`PieceSide::Positive`] returns **socket** solids (per
///   [`PrismaticPinSpec::socket_params`], same pose, extents inflated
///   by the symmetric `/2` clearance convention) — callers SUBTRACT
///   each from the per-piece [`Solid`] pre-MC.
///
/// `compose_piece_solid` (the only production caller) consumes both
/// modes uniformly per [`PieceSide`].
///
/// Returns an empty `Vec` when:
/// - `ribbon.registration` is [`RegistrationKind::None`];
/// - the spec has no arc fractions;
/// - any pin's split-normal ray runs past the module-private
///   `PIN_RAY_MAX_REACH_M` (1 m) without crossing the
///   bounding-region surface (signals a degenerate caller).
///
/// Each pin's pose is built per the §"Pin pose convention" module-
/// docstring section: `center_m` at the annulus midpoint
/// `(body_dist + bounding_dist) / 2` along `sign · split_normal`;
/// `axis_unit` is the ribbon binormal; `lateral_unit` is the ribbon
/// split-normal (orthogonal to binormal by construction). Pin
/// extends `±half_length_m` along binormal, symmetric across the
/// seam plane — the `-binormal` half overlaps cup-wall material for
/// SDF-union connectivity under the recon-4 (P) seam architecture,
/// the `+binormal` half protrudes as the workshop-visible ridge.
///
/// `layer_body` and `bounding_region` are the same per-layer
/// [`Solid`]s the caller passes to
/// [`crate::piece::compose_piece_solid`]; the cup-wall annulus is
/// `bounding_region ∖ layer_body` at this layer.
#[must_use]
pub fn build_registration_sdf_ops(
    ribbon: &Ribbon,
    layer_body: &Solid,
    bounding_region: &Solid,
    side: PieceSide,
) -> Vec<Solid> {
    let spec = match &ribbon.registration {
        RegistrationKind::None => return Vec::new(),
        RegistrationKind::Pins(spec) => spec,
    };
    if spec.arc_fractions.is_empty() {
        return Vec::new();
    }
    let split_vec = ribbon.split_normal.as_vector();
    let lateral_unit = UnitVector3::new_unchecked(split_vec);
    let mut solids = Vec::with_capacity(spec.arc_fractions.len() * 2);
    for &t in &spec.arc_fractions {
        let Some((center, _tangent, binormal)) = ribbon.sample_at_arc_fraction(t) else {
            return Vec::new();
        };
        // `binormal` is unit-length by ribbon construction
        // (`tangent × N_split` normalized); renormalize defensively
        // so a degenerate (zero) binormal vector returns the
        // Z-fallback rather than panicking on UnitVector3 invariants.
        let axis_unit = UnitVector3::new_normalize(binormal);
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
            let pose = PrismaticPinPose::new(pin_center, axis_unit, lateral_unit);
            let params = match side {
                PieceSide::Negative => spec.pin_spec.pin_params(pose),
                PieceSide::Positive => spec.pin_spec.socket_params(pose),
            };
            solids.push(build_prismatic_pin_sdf(&params));
        }
    }
    solids
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
/// `PIN_RAY_MAX_REACH_M` (degenerate input — e.g., an unbounded
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
// `float_cmp`: fit-invariant + chamfer-equality assertions compare
// f64 fields by exact value — these are determinism contracts on
// emission where any bit-difference would indicate a real regression
// (not measurement noise). The `assert_abs_diff_eq!` machine-epsilon
// variants live alongside, used where decimal-mm input values are
// not f64-exact (matches the convention in `crate::prismatic_pin::tests`).
#[allow(
    clippy::unwrap_used,
    clippy::panic,
    clippy::expect_used,
    clippy::float_cmp
)]
mod tests {

    use super::*;
    use crate::piece::RIBBON_PIECE_OVERLAP_M;
    use crate::ribbon::{Ribbon, SplitNormal};
    use approx::assert_abs_diff_eq;
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
    /// `y ∈ [+10 mm, +30 mm]` and the pin lands at the annulus
    /// midpoint `y = +20 mm` ((10 + 30) / 2).
    fn reference_body_and_bounds() -> (Solid, Solid) {
        let body = Solid::cuboid(Vector3::new(0.060, 0.010, 0.010));
        let bounds = Solid::cuboid(Vector3::new(0.060, 0.030, 0.030));
        (body, bounds)
    }

    #[test]
    fn pin_spec_iter1_has_cup_pin_default_and_two_arc_fractions() {
        let s = PinSpec::iter1();
        assert_eq!(s.pin_spec, PrismaticPinSpec::cup_pin_default());
        assert_eq!(s.arc_fractions, vec![0.25, 0.75]);
    }

    #[test]
    fn registration_kind_default_is_none() {
        assert_eq!(RegistrationKind::default(), RegistrationKind::None);
    }

    #[test]
    fn build_registration_sdf_ops_none_returns_empty() {
        let ribbon = straight_x_ribbon(); // default registration = None
        let (body, bounds) = reference_body_and_bounds();
        assert!(
            build_registration_sdf_ops(&ribbon, &body, &bounds, PieceSide::Negative).is_empty()
        );
        assert!(
            build_registration_sdf_ops(&ribbon, &body, &bounds, PieceSide::Positive).is_empty()
        );
    }

    #[test]
    fn build_registration_sdf_ops_pins_returns_one_solid_per_pin_per_side() {
        // 4 pins total: 2 arc fractions × 2 lateral mirror sides.
        let ribbon =
            straight_x_ribbon().with_registration(RegistrationKind::Pins(PinSpec::iter1()));
        let (body, bounds) = reference_body_and_bounds();
        let neg = build_registration_sdf_ops(&ribbon, &body, &bounds, PieceSide::Negative);
        let pos = build_registration_sdf_ops(&ribbon, &body, &bounds, PieceSide::Positive);
        assert_eq!(
            neg.len(),
            4,
            "4 pins per piece (2 arc fractions × 2 lateral mirrors)"
        );
        assert_eq!(
            pos.len(),
            4,
            "Positive side emits the same socket count as Negative pin count"
        );
    }

    #[test]
    fn build_registration_sdf_ops_empty_arc_fractions_returns_empty() {
        let mut spec = PinSpec::iter1();
        spec.arc_fractions.clear();
        let ribbon = straight_x_ribbon().with_registration(RegistrationKind::Pins(spec));
        let (body, bounds) = reference_body_and_bounds();
        assert!(
            build_registration_sdf_ops(&ribbon, &body, &bounds, PieceSide::Negative).is_empty()
        );
    }

    /// §G-10 #1 bit-precise fit invariant: at the spec layer,
    /// socket extents inflate by exactly `clearance / 2` per axis
    /// from pin extents, within machine epsilon. SDF-side analog of
    /// S5's mesh-CSG `pin_and_socket_fit_invariant`. The contract
    /// lives at the SDF input layer (per
    /// [`crate::prismatic_pin`] module docstring) — the SDF kernel
    /// composes the primitive lazily, so a determinism contract on
    /// the SDF input bytes implies bit-equal SDF evaluation across
    /// pin / socket sides modulo the clearance inflation.
    ///
    /// Extends the [`crate::prismatic_pin`] module's
    /// `prismatic_pin_pair_extents_match_spec_clearance_within_machine_epsilon`
    /// test to the cup-pin call path (verifies the registration
    /// module's spec consumption — not just the primitive in
    /// isolation).
    #[test]
    fn cup_prismatic_pin_and_socket_fit_invariant() {
        let ribbon =
            straight_x_ribbon().with_registration(RegistrationKind::Pins(PinSpec::iter1()));
        let (body, bounds) = reference_body_and_bounds();

        let pin_spec = PrismaticPinSpec::cup_pin_default();
        let half_diametral = pin_spec.diametral_clearance_m / 2.0;
        let half_axial = pin_spec.axial_clearance_m / 2.0;

        // Pose comes from the same ray-march per arc fraction +
        // mirror; verifying both sides give the same pose triple by
        // reaching the same per-pin annulus midpoint in both `side`
        // calls. Then check that the BUILT params per
        // PrismaticPinSpec respect the clearance inflation.
        let pose = PrismaticPinPose::new(
            Point3::new(-0.025, 0.020, 0.0),
            UnitVector3::new_unchecked(Vector3::new(0.0, 0.0, 1.0)),
            UnitVector3::new_unchecked(Vector3::new(0.0, 1.0, 0.0)),
        );
        let pin_params = pin_spec.pin_params(pose.clone());
        let socket_params = pin_spec.socket_params(pose);

        assert_abs_diff_eq!(
            socket_params.base_half_extents_m.x - pin_params.base_half_extents_m.x,
            half_diametral,
            epsilon = 1.0e-15,
        );
        assert_abs_diff_eq!(
            socket_params.base_half_extents_m.y - pin_params.base_half_extents_m.y,
            half_diametral,
            epsilon = 1.0e-15,
        );
        assert_abs_diff_eq!(
            socket_params.tip_half_extents_m.x - pin_params.tip_half_extents_m.x,
            half_diametral,
            epsilon = 1.0e-15,
        );
        assert_abs_diff_eq!(
            socket_params.tip_half_extents_m.y - pin_params.tip_half_extents_m.y,
            half_diametral,
            epsilon = 1.0e-15,
        );
        assert_abs_diff_eq!(
            socket_params.half_length_m - pin_params.half_length_m,
            half_axial,
            epsilon = 1.0e-15,
        );
        assert_eq!(socket_params.base_chamfer_m, pin_params.base_chamfer_m);

        // Spot-check via the full call path that pin + socket sides
        // produce different solids (different counts of solids would
        // signal a missing arc fraction; identical Vec lengths but
        // different contained geometry is what we want).
        let neg = build_registration_sdf_ops(&ribbon, &body, &bounds, PieceSide::Negative);
        let pos = build_registration_sdf_ops(&ribbon, &body, &bounds, PieceSide::Positive);
        assert_eq!(neg.len(), pos.len(), "pin + socket Vec lengths must match");
    }

    /// §G-10 SDF position invariant: each pin's centre (interior
    /// reading) lies at the annulus midpoint of the cup-wall along
    /// `±split_normal`. Replaces the mesh-CSG-era
    /// `pin_transforms_position_each_pin_at_annulus_midpoint` (which
    /// inspected `CylinderParent::center_m` directly); the SDF-side
    /// equivalent samples the pin SDF at the expected pose centre
    /// and asserts interior.
    #[test]
    fn cup_pin_sdf_centre_lands_at_annulus_midpoint() {
        // Pin at arc-fraction 0.25 of straight +X centerline (-0.05 → +0.05):
        //   centerline_sample = (-0.025, 0, 0)
        //   body surface along +Y: y = +0.010
        //   bounding surface +Y:   y = +0.030
        //   annulus midpoint   = (10 + 30) / 2 = +0.020
        //   pin centre         = (-0.025, +0.020, 0)
        //   axis               = +Z (binormal: tangent +X × split +Y = +Z)
        let ribbon =
            straight_x_ribbon().with_registration(RegistrationKind::Pins(PinSpec::iter1()));
        let (body, bounds) = reference_body_and_bounds();
        let neg = build_registration_sdf_ops(&ribbon, &body, &bounds, PieceSide::Negative);
        assert_eq!(neg.len(), 4, "4 pins expected from default iter1 spec");

        let expected_centres = [
            Point3::new(-0.025, 0.020, 0.0),  // t=0.25, +Y
            Point3::new(-0.025, -0.020, 0.0), // t=0.25, -Y
            Point3::new(0.025, 0.020, 0.0),   // t=0.75, +Y
            Point3::new(0.025, -0.020, 0.0),  // t=0.75, -Y
        ];
        for expected in &expected_centres {
            let matched = neg.iter().any(|pin| pin.evaluate(expected) < 0.0);
            assert!(
                matched,
                "expected interior reading at pin centre {expected:?}; \
                 no pin SDF in the Vec reads negative there"
            );
        }
    }

    /// §G-10 wide-body regression (iter-1 sock-over-capsule scope) —
    /// each pin's pose lands in the cup-wall annulus (OUTSIDE body,
    /// INSIDE bounding) at `|x| ≈ 48.5 mm = (36 + 61) / 2`.
    /// SDF-side equivalent of the post-S5 mesh-CSG-era
    /// `pin_transforms_position_stays_in_cup_wall_for_wide_body_iter1_regression`.
    #[test]
    fn cup_pin_sdf_centre_stays_in_cup_wall_for_wide_body() {
        let body = Solid::cuboid(Vector3::new(0.036, 0.036, 0.036));
        let bounds = Solid::cuboid(Vector3::new(0.061, 0.061, 0.061));
        let centerline = vec![Point3::new(0.0, 0.0, -0.050), Point3::new(0.0, 0.0, 0.050)];
        let split = SplitNormal::new(Vector3::new(1.0, 0.0, 0.0)).unwrap();
        let ribbon = Ribbon::new(centerline, split)
            .unwrap()
            .with_registration(RegistrationKind::Pins(PinSpec::iter1()));
        let neg = build_registration_sdf_ops(&ribbon, &body, &bounds, PieceSide::Negative);
        assert_eq!(neg.len(), 4, "wide-body fixture should still emit 4 pins");

        // Sample expected centres: arc fractions 0.25 + 0.75 along
        // ±Z centerline (z = -0.025 and z = +0.025), each mirrored
        // along ±X split-normal at offset = (36 + 61) / 2 = 48.5 mm.
        let expected_centres = [
            Point3::new(0.0485, 0.0, -0.025),
            Point3::new(-0.0485, 0.0, -0.025),
            Point3::new(0.0485, 0.0, 0.025),
            Point3::new(-0.0485, 0.0, 0.025),
        ];
        for expected in &expected_centres {
            let matched = neg.iter().any(|pin| pin.evaluate(expected) < 0.0);
            assert!(
                matched,
                "expected interior reading at annulus-midpoint pin centre {expected:?} \
                 (wide-body iter-1 regression); no pin SDF reads negative there. \
                 If a pin's centre is INSIDE the body, the pre-C1 fixed-offset bug \
                 has regressed."
            );
            // Belt-and-suspenders: the centre must lie OUTSIDE the
            // body (cup-wall material) and INSIDE the bounding
            // region (cup-wall material). Holds independently of the
            // SDF emission path.
            assert!(
                body.evaluate(expected) > 0.0,
                "pin centre {expected:?} must be OUTSIDE the body"
            );
            assert!(
                bounds.evaluate(expected) < 0.0,
                "pin centre {expected:?} must be INSIDE the bounding region"
            );
        }
    }

    /// §G-10 pin-extends-across-seam check: the pin's `+binormal`
    /// half protrudes past the seam plane (= z = 0 for this
    /// fixture) by ~`half_length_m`. Confirms the symmetric-across-
    /// seam pose convention from the module docstring — the
    /// workshop-visible registration ridge is produced when this
    /// pin SDF is unioned with the Negative half-shell (the half-
    /// shell's `+binormal` boundary is at `+RIBBON_PIECE_OVERLAP_M`,
    /// so any pin material past that becomes the ridge).
    #[test]
    fn cup_pin_extends_symmetrically_across_seam_plane() {
        let ribbon =
            straight_x_ribbon().with_registration(RegistrationKind::Pins(PinSpec::iter1()));
        let (body, bounds) = reference_body_and_bounds();
        let neg = build_registration_sdf_ops(&ribbon, &body, &bounds, PieceSide::Negative);

        // For the first pin (centre at (-0.025, +0.020, 0)), sample
        // at:
        //   - 1 µm past the +binormal tip → exterior (z = +half_length + 1e-6)
        //   - 1 mm inside the -binormal tip → interior (z = -half_length + 1e-3)
        let half_length = PrismaticPinSpec::cup_pin_default().pin_half_length_m;
        let first_pin = &neg[0];
        let first_centre = Point3::new(-0.025, 0.020, 0.0);

        let interior_neg_z = first_centre + Vector3::new(0.0, 0.0, -half_length + 1.0e-3);
        let interior_pos_z = first_centre + Vector3::new(0.0, 0.0, half_length - 1.0e-3);
        let exterior_pos_z = first_centre + Vector3::new(0.0, 0.0, half_length + 1.0e-4);

        assert!(
            first_pin.evaluate(&interior_neg_z) < 0.0,
            "pin interior at -binormal half (z = -{} + 1 mm) must be inside; SDF = {}",
            half_length,
            first_pin.evaluate(&interior_neg_z),
        );
        assert!(
            first_pin.evaluate(&interior_pos_z) < 0.0,
            "pin interior at +binormal half (z = +{} - 1 mm) must be inside; \
             confirms symmetric extent across seam; SDF = {}",
            half_length,
            first_pin.evaluate(&interior_pos_z),
        );
        assert!(
            first_pin.evaluate(&exterior_pos_z) > 0.0,
            "pin 0.1 mm past +binormal tip must be exterior; SDF = {}",
            first_pin.evaluate(&exterior_pos_z),
        );
    }

    /// §G-10 + recon-1 §G-12 #2 paradigm-boundary precondition: the
    /// pin's `-binormal` half overlaps the Negative half-shell
    /// material by at least `RIBBON_PIECE_OVERLAP_M`. The half-
    /// shell's `+binormal` boundary sits at
    /// `+RIBBON_PIECE_OVERLAP_M` (per
    /// `crate::piece::compose_piece_solid`'s seam halfspace inward
    /// bias); the pin centre at `binormal = 0` has its `-binormal`
    /// half spanning `[-half_length, 0]`. So the overlap region
    /// `[-half_length, +RIBBON_PIECE_OVERLAP_M]` lies inside the
    /// half-shell — providing the SDF-union connectivity that the
    /// recon-3 §R3-3 / recon-4 §F-3 architectural-correction
    /// pattern relies on.
    ///
    /// This is the lib-test analog of the §G-7 probe's BRANCH-A
    /// outcome (`§G-12 #2 bail-out (SDF-side pin, 3 mm)` →
    /// 1 component). The probe characterises full SDF→MC at
    /// production cell sizes; this test characterises the SDF input
    /// pre-MC (geometric overlap exists, which is the SDF-union
    /// connectivity precondition).
    #[test]
    fn cup_pin_overlaps_negative_half_shell_for_sdf_union_connectivity() {
        let ribbon =
            straight_x_ribbon().with_registration(RegistrationKind::Pins(PinSpec::iter1()));
        let (body, bounds) = reference_body_and_bounds();
        let neg = build_registration_sdf_ops(&ribbon, &body, &bounds, PieceSide::Negative);

        let first_pin = &neg[0];
        let first_centre = Point3::new(-0.025, 0.020, 0.0);
        // Probe at z = -RIBBON_PIECE_OVERLAP_M (just inside the
        // Negative half-shell's seam boundary): the pin must report
        // interior here so SDF union with the half-shell forms a
        // connected component.
        let overlap_probe = first_centre + Vector3::new(0.0, 0.0, -RIBBON_PIECE_OVERLAP_M);
        assert!(
            first_pin.evaluate(&overlap_probe) < 0.0,
            "pin must extend into the Negative half-shell's seam-overlap region \
             (z = -RIBBON_PIECE_OVERLAP_M from centre) for SDF-union connectivity; \
             pin SDF = {}",
            first_pin.evaluate(&overlap_probe),
        );
    }

    /// Bisect for the SDF zero-crossing along `lateral_unit` from
    /// `base_world`. `base_world` is the world-frame point at the
    /// axial coordinate of interest (`= center + axial * axis_unit`
    /// at lateral = 0); the bisection walks `+lateral_unit` and
    /// returns the lateral magnitude at which the SDF crosses 0.
    ///
    /// World-coord analog of `crate::prismatic_pin::tests::find_x_zero_crossing`
    /// (which operates in pin-local coords against the identity pose).
    /// The half-open interior predicate (`sdf <= +1e-12`) absorbs
    /// sub-ulp positive noise on the chamfer-band cap planes — same
    /// rationale as the primitive-layer helper's docstring.
    ///
    /// MIRROR: an identical copy lives in `crate::plug::tests`; any
    /// change to one MUST mirror the other (or the helper should be
    /// promoted to a shared test-util module).
    fn find_lateral_zero_crossing(
        sdf: &Solid,
        base_world: Point3<f64>,
        lateral_unit: Vector3<f64>,
        lo: f64,
        hi: f64,
    ) -> f64 {
        let mut a = lo;
        let mut b = hi;
        for _ in 0..64 {
            let m = f64::midpoint(a, b);
            if sdf.evaluate(&(base_world + m * lateral_unit)) <= 1.0e-12 {
                a = m;
            } else {
                b = m;
            }
        }
        f64::midpoint(a, b)
    }

    /// §G-10 S5 chamfer band production-call-path gate: the chamfer
    /// band that S2's [`crate::build_prismatic_pin_sdf`] composes at
    /// the primitive layer must hold geometrically when emitted
    /// through [`build_registration_sdf_ops`]'s pose-rotation +
    /// world-translation, on the production cup-pin call path
    /// (symmetric-across-seam pose, not the synthetic identity pose
    /// used in
    /// `crate::prismatic_pin::tests::chamfer_band_lateral_extents_match_spec_across_g6_range`).
    ///
    /// First pin (arc fraction 0.25, `+split_normal` mirror) sits at
    /// `center = (-0.025, +0.020, 0)` with `axis_unit = +Z` (binormal)
    /// and `lateral_unit = +Y` (`split_normal`). The pin-local `-Y`
    /// axial end (= chamfer-band base; per the module docstring this
    /// sits at the deepest-in-cup-wall side under the symmetric-
    /// across-seam pose, **not necessarily bed-adjacent** — the §G-4
    /// chamfer-band ↔ workshop-bed reconciliation is deferred to S6
    /// procedure.rs) projects to world `z = -half_length_m`; the
    /// chamfer-top axial coord projects to world
    /// `z = -half_length_m + base_chamfer_m`. Lateral extents along
    /// pin-local `+X` map to world `+Y`.
    ///
    /// Verifies (a) the chamfer-band-base lateral extent equals
    /// `(base - chamfer)` to ~1 µm; (b) the chamfer-top lateral
    /// extent equals `base` to ~1 µm. Paired with a chamfer-disabled
    /// bare baseline per [[feedback-load-bearing-test-fixtures]]: a
    /// `PrismaticPinSpec { base_chamfer_m: 0.0, .. }` override emits
    /// a single-frustum pin whose chamfer-band-base extent equals
    /// `base` exactly — the delta between bare-baseline and with-
    /// chamfer base extents must equal `base_chamfer_m`. If chamfer
    /// emission breaks at the pose-rotation layer, the extent would
    /// either stay at `base` (no chamfer applied) or invert; the bare
    /// baseline guarantees the test detects fixture breakage rather
    /// than passing for the wrong reason.
    #[test]
    fn cup_pin_chamfer_band_holds_through_sdf_ops() {
        let pin_spec = PrismaticPinSpec::cup_pin_default();
        let ribbon =
            straight_x_ribbon().with_registration(RegistrationKind::Pins(PinSpec::iter1()));
        let (body, bounds) = reference_body_and_bounds();
        let neg = build_registration_sdf_ops(&ribbon, &body, &bounds, PieceSide::Negative);
        let pin = &neg[0];

        let center = Point3::new(-0.025, 0.020, 0.0);
        let axis_unit = Vector3::new(0.0, 0.0, 1.0);
        let lateral_unit = Vector3::new(0.0, 1.0, 0.0);
        let bed_axial = -pin_spec.pin_half_length_m;
        let bed_base = center + bed_axial * axis_unit;
        let bed_expected = pin_spec.pin_base_half_extents_m.x - pin_spec.base_chamfer_m;
        let bed_extent = find_lateral_zero_crossing(pin, bed_base, lateral_unit, 0.0, 0.01);
        assert_abs_diff_eq!(bed_extent, bed_expected, epsilon = 1.0e-6);

        let top_axial = -pin_spec.pin_half_length_m + pin_spec.base_chamfer_m;
        let top_base = center + top_axial * axis_unit;
        let top_expected = pin_spec.pin_base_half_extents_m.x;
        let top_extent = find_lateral_zero_crossing(pin, top_base, lateral_unit, 0.0, 0.01);
        assert_abs_diff_eq!(top_extent, top_expected, epsilon = 1.0e-6);

        // Bare-baseline pairing: chamfer-disabled spec emits a
        // single-frustum pin; bed-face extent equals `base` (NOT
        // `base - chamfer`). Confirms the chamfer band emission is
        // load-bearing through the production cup-pin call path —
        // not silently degenerate.
        let bare_spec = PinSpec {
            pin_spec: PrismaticPinSpec {
                base_chamfer_m: 0.0,
                ..pin_spec
            },
            arc_fractions: vec![0.25, 0.75],
        };
        let bare_ribbon = straight_x_ribbon().with_registration(RegistrationKind::Pins(bare_spec));
        let bare_neg =
            build_registration_sdf_ops(&bare_ribbon, &body, &bounds, PieceSide::Negative);
        let bare_pin = &bare_neg[0];
        let bare_bed_extent =
            find_lateral_zero_crossing(bare_pin, bed_base, lateral_unit, 0.0, 0.01);
        assert_abs_diff_eq!(
            bare_bed_extent,
            pin_spec.pin_base_half_extents_m.x,
            epsilon = 1.0e-6,
        );
        // The chamfered bed extent is inset from the bare-baseline
        // bed extent by exactly `base_chamfer_m`.
        assert_abs_diff_eq!(
            bare_bed_extent - bed_extent,
            pin_spec.base_chamfer_m,
            epsilon = 1.0e-6,
        );
    }

    /// §G-10 S5 socket-side chamfer gate: at the production cup-pin
    /// emission, the Positive-side socket's chamfer-band cross-section
    /// **at the socket's own bed face** is the pin's chamfer-band
    /// bed-face cross-section inflated by `diametral_clearance_m / 2`
    /// per lateral side. Same invariant at the chamfer-top boundary
    /// (widest section).
    ///
    /// Why each side's **own** bed face (not a shared world-coord):
    /// [`PrismaticPinSpec::socket_params`] inflates `half_length_m`
    /// by `axial_clearance_m / 2`, so the socket's bed face sits
    /// `axial_clearance_m / 2` axially deeper than the pin's. Probing
    /// at a shared world-axial coord would land midway through the
    /// socket's chamfer band's linear interpolation (lateral delta
    /// would be larger than `half_diametral`, conflating chamfer-band
    /// emission with the axial-clearance offset). The bit-precise fit
    /// invariant is on the spec layer's `base_half_extents_m` +
    /// `pin_base_half_extents_m - chamfer` extents — i.e., extents at
    /// each side's chamfer-band endpoints.
    ///
    /// Paired with a chamfer-disabled bare baseline per
    /// [[feedback-load-bearing-test-fixtures]]: chamfer disabled, the
    /// socket-vs-pin lateral delta at each side's bed face still
    /// equals `diametral_clearance_m / 2` — confirms the socket
    /// extents emission path is chamfer-independent (a chamfer-only
    /// bug would surface as with-chamfer delta breakage while the
    /// bare baseline holds).
    ///
    /// Parallel test on the plug-lock call path:
    /// `crate::plug::tests::plug_lock_socket_chamfer_matches_lock_pin`
    /// — mirrored structure (same probe-at-each-side's-own-bed-face
    /// pattern; same chamfer-disabled bare-baseline pairing). Any
    /// future change to the socket inflation convention or chamfer
    /// emission MUST update both tests in lockstep.
    #[test]
    fn cup_pin_socket_chamfer_matches_pin() {
        let pin_spec = PrismaticPinSpec::cup_pin_default();
        let half_diametral = pin_spec.diametral_clearance_m / 2.0;
        let half_axial = pin_spec.axial_clearance_m / 2.0;
        let ribbon =
            straight_x_ribbon().with_registration(RegistrationKind::Pins(PinSpec::iter1()));
        let (body, bounds) = reference_body_and_bounds();
        let neg = build_registration_sdf_ops(&ribbon, &body, &bounds, PieceSide::Negative);
        let pos = build_registration_sdf_ops(&ribbon, &body, &bounds, PieceSide::Positive);

        let center = Point3::new(-0.025, 0.020, 0.0);
        let axis_unit = Vector3::new(0.0, 0.0, 1.0);
        let lateral_unit = Vector3::new(0.0, 1.0, 0.0);

        // Pin bed face: axial = -pin_half_length; extent = base -
        // chamfer = 0.9 mm. Socket bed face: axial = -(pin_half_length
        // + half_axial); extent = (base + half_diametral) - chamfer =
        // 1.05 mm. Delta = half_diametral.
        let pin_bed_base = center + (-pin_spec.pin_half_length_m) * axis_unit;
        let socket_bed_base = center + (-(pin_spec.pin_half_length_m + half_axial)) * axis_unit;
        let pin_bed = find_lateral_zero_crossing(&neg[0], pin_bed_base, lateral_unit, 0.0, 0.01);
        let socket_bed =
            find_lateral_zero_crossing(&pos[0], socket_bed_base, lateral_unit, 0.0, 0.01);
        assert_abs_diff_eq!(socket_bed - pin_bed, half_diametral, epsilon = 1.0e-6);

        // Pin chamfer-top: axial = -pin_half_length + chamfer; extent
        // = base = 1.5 mm. Socket chamfer-top: axial =
        // -(pin_half_length + half_axial) + chamfer; extent = base +
        // half_diametral = 1.65 mm. Delta = half_diametral.
        let pin_top_base =
            center + (-pin_spec.pin_half_length_m + pin_spec.base_chamfer_m) * axis_unit;
        let socket_top_base = center
            + (-(pin_spec.pin_half_length_m + half_axial) + pin_spec.base_chamfer_m) * axis_unit;
        let pin_top = find_lateral_zero_crossing(&neg[0], pin_top_base, lateral_unit, 0.0, 0.01);
        let socket_top =
            find_lateral_zero_crossing(&pos[0], socket_top_base, lateral_unit, 0.0, 0.01);
        assert_abs_diff_eq!(socket_top - pin_top, half_diametral, epsilon = 1.0e-6);

        // Bare-baseline (chamfer disabled): same socket-vs-pin delta
        // at each side's bed face. Chamfer-independence sanity check.
        let bare_spec = PinSpec {
            pin_spec: PrismaticPinSpec {
                base_chamfer_m: 0.0,
                ..pin_spec
            },
            arc_fractions: vec![0.25, 0.75],
        };
        let bare_ribbon = straight_x_ribbon().with_registration(RegistrationKind::Pins(bare_spec));
        let bare_neg =
            build_registration_sdf_ops(&bare_ribbon, &body, &bounds, PieceSide::Negative);
        let bare_pos =
            build_registration_sdf_ops(&bare_ribbon, &body, &bounds, PieceSide::Positive);
        let bare_pin_bed =
            find_lateral_zero_crossing(&bare_neg[0], pin_bed_base, lateral_unit, 0.0, 0.01);
        let bare_socket_bed =
            find_lateral_zero_crossing(&bare_pos[0], socket_bed_base, lateral_unit, 0.0, 0.01);
        assert_abs_diff_eq!(
            bare_socket_bed - bare_pin_bed,
            half_diametral,
            epsilon = 1.0e-6,
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
