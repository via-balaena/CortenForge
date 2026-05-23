//! Workshop pour-funnel geometry.
//!
//! Honey-thick workshop silicones (Dragon Skin 10A / 20A / 30A at
//! ~20-23k cps) take minutes per layer to pour through bare
//! 10 mm Ø pour-gate holes without splashing or trapping air.
//! [`build_funnel_solid`] generates a printable funnel STL with a
//! self-aligning **nipple** that slides into the pour-gate hole +
//! a **flange** that rests flat on the cup outer surface around the
//! hole + a tapered hollow **cone** widening to a workshop-friendly
//! mouth for ladle-pouring.
//!
//! ## Geometry
//!
//! Funnel sits with its axis along world `+Z`; bottom at `z=0`
//! (nipple tip), top at `z = nipple_height + flange_height +
//! cone_height` (workshop pour mouth).
//!
//! - **Nipple** at the bottom: cylinder outer Ø = pour-gate Ø −
//!   [`NIPPLE_DIAMETRAL_CLEARANCE_M`] (slides into the pour-leg
//!   hole). Post-S7 lives entirely as a post-MC mesh-CSG primitive
//!   ([`MatingTransform::UnionCylinder`]) so the OD is bit-precise
//!   to f64 rather than MC-quantized. Inner Ø = outer Ø −
//!   2 × [`FUNNEL_NIPPLE_WALL_M`].
//! - **Flange** above the nipple: cylinder outer Ø = 2 ×
//!   [`FUNNEL_FLANGE_OUTER_RADIUS_M`]. Rests flat on the cup wall
//!   outer surface around the pour-gate hole.
//! - **Cone** above the flange: truncated cone widening from
//!   flange Ø at the bottom to 2 × [`FUNNEL_TOP_OUTER_RADIUS_M`]
//!   at the top.
//! - **Hollow interior**: single continuous tapered cavity. The
//!   SDF carves the flange + cone region; the post-MC
//!   [`MatingTransform::SubtractCylinder`] lumen carves through the
//!   added nipple so the lumen passes through end-to-end.
//!
//! ## Recon §9 axial standoff — DEFERRED
//!
//! `docs/CF_CAST_MATING_FEATURES_RECON.md` §9 proposes a
//! `FLANGE_AXIAL_STANDOFF_M` of 0.5 mm to hold the broad flange off
//! the curved + MC-jittery cup outer surface. Implementation was
//! left open (S0 bookmark §M5: "Recon decides whether the standoff
//! lives as a physical lip on the flange or whether the cup-outer-
//! face mesh gets a local-flat patch under the flange footprint" —
//! never resolved in recon §9). S7 explored a sub-mm contact-ring
//! lip in mesh-CSG, but that thickness is below
//! [`crate::spec::FUNNEL_MAX_CELL_SIZE_M`] (1 mm) and the resulting
//! mesh fails F4 `ThinWall` checks. Deferred to a future polish
//! session: either drop the funnel MC cell size below 0.5 mm and
//! ship a flange-underside annular lip in the SDF, or wait until
//! iter-2's workshop print surfaces a wobble defect that motivates
//! a specific implementation.

use cf_design::Solid;
use nalgebra::{Point3, Unit, UnitQuaternion, Vector3};

use crate::mesh_csg::{CylinderParams, CylinderParent, MatingTransform};
use crate::pour::PourGateKind;
use crate::ribbon::Ribbon;

/// Nipple height (m).
///
/// 4 mm — just enough for self-alignment during insertion into
/// the pour-gate hole; short so the nipple's narrower inner Ø
/// isn't a long viscous bottleneck.
pub const FUNNEL_NIPPLE_HEIGHT_M: f64 = 0.004;

/// Nipple wall thickness (m).
///
/// 1.5 mm = 3-4 perimeters at a 0.4 mm FDM nozzle. Sized at 1.5 mm
/// rather than 1 mm so the nipple has multi-perimeter structural
/// rigidity against workshop pour-pressure deflection — and to
/// clear cf-cast's F4 `min_wall_thickness: 1.0` threshold with
/// margin around the inner lumen carve (which still passes through
/// MC at the flange annulus, even though the nipple OD itself is
/// bit-precise via mesh-CSG post-S7). Trade-off: nipple inner Ø
/// drops from 7.8 mm (theoretical 1 mm wall) to 6.8 mm (`1.5` mm
/// wall) — still wider than the pre-iter-1 6 mm pour-gate Ø, and
/// the 4 mm nipple height keeps the bottleneck short.
pub const FUNNEL_NIPPLE_WALL_M: f64 = 0.0015;

/// Diametral clearance between the nipple OD and the cup pour-gate
/// hole ID (m).
///
/// Total diametral gap = this value (asymmetric `/2` convention:
/// cup hole stays at exactly [`crate::PourGateSpec::gate_radius_m`];
/// funnel nipple shrinks by half this clearance per side).
///
/// 0.5 mm — resting-contact fit per `docs/CF_CAST_MATING_FEATURES_RECON.md`
/// §9 M5. Loosens vs the pre-S7 0.2 mm slack to absorb FDM bead
/// bulge on both sides (cup hole prints undersized; funnel nipple
/// prints oversized) without the workshop user reaming. The
/// mesh-CSG migration (S7) makes the nominal sizes bit-precise to
/// f64; the looser budget compensates only for the FDM
/// mechanism-C error.
pub const NIPPLE_DIAMETRAL_CLEARANCE_M: f64 = 0.0005;

/// Flange outer radius (m).
///
/// 10 mm = 20 mm Ø — wide enough that the flange rests stably on
/// the cup wall around a 10 mm pour-gate hole without tipping;
/// small enough to not interfere with the adjacent vent leg of
/// the V pour gate.
pub const FUNNEL_FLANGE_OUTER_RADIUS_M: f64 = 0.010;

/// Flange height / disk thickness (m).
///
/// 1 mm thick disk gives the flange a positive z-extent so the
/// workshop user can see + grip it visually distinct from the
/// nipple cylinder below.
pub const FUNNEL_FLANGE_HEIGHT_M: f64 = 0.001;

/// Top outer radius (m).
///
/// 15 mm = 30 mm Ø — workshop-friendly mouth for ladle-pouring
/// honey-thick silicone.
pub const FUNNEL_TOP_OUTER_RADIUS_M: f64 = 0.015;

/// Cone wall thickness (m).
///
/// 1.5 mm = 3-4 perimeters at a 0.4 mm FDM nozzle; structural
/// enough that the funnel doesn't deform under the weight of
/// poured silicone (~200 g per layer fill).
pub const FUNNEL_CONE_WALL_M: f64 = 0.0015;

/// Cone height (m).
///
/// 30 mm — adequate clearance for a workshop ladle to pour into
/// the mouth without splashing back, while keeping the funnel's
/// total height + footprint manageable.
pub const FUNNEL_CONE_HEIGHT_M: f64 = 0.030;

/// Overshoot past funnel surfaces for the inner cavity carve (m).
///
/// 0.5 mm. Two uses post-S7 (both at the same workshop-margin
/// value):
///
/// 1. **SDF top opening** — `inner_cone` extends `INNER_CAVITY_OVERSHOOT_M`
///    above `cone_z_top` so MC produces a clean opening at the
///    workshop pour mouth instead of leaving a stair-step shelf.
/// 2. **Mesh-CSG lumen margin** — the post-MC
///    [`MatingTransform::SubtractCylinder`] lumen extends
///    `INNER_CAVITY_OVERSHOOT_M` past both nipple end faces so
///    manifold3d's `difference` op definitely cuts through the
///    unioned nipple's flat caps (at the tip + at the flange
///    interface) rather than leaving a thin disk capping either
///    end.
const INNER_CAVITY_OVERSHOOT_M: f64 = 0.0005;

/// Build the workshop pour funnel [`Solid`] + post-MC mesh-CSG
/// transforms for the given ribbon, or `None` if the ribbon has no
/// pour gate enabled (no funnel needed).
///
/// Post-S7, the **nipple lives entirely as a post-MC mesh-CSG
/// primitive** ([`MatingTransform::UnionCylinder`]) so its OD is
/// bit-precise to f64 rather than MC-quantized (the cup pour-gate
/// carve on the matching cup side similarly migrates to mesh-CSG,
/// locking the workshop diametral fit at engine precision).
///
/// The funnel's nipple OD is sized to slide into the ribbon's
/// pour-gate hole: nipple OD = 2 × `pour_gate.gate_radius_m` −
/// [`NIPPLE_DIAMETRAL_CLEARANCE_M`] (asymmetric `/2` convention —
/// funnel side bears all the slack; cup-piece pour-gate-hole stays
/// at the nominal `2 × gate_radius_m`).
///
/// The returned `Vec<MatingTransform>` contains two ops in
/// declared order:
/// 1. [`MatingTransform::UnionCylinder`] for the nipple body
///    (`r = nipple_outer_r`, `z ∈ [0, FUNNEL_NIPPLE_HEIGHT_M]`).
/// 2. [`MatingTransform::SubtractCylinder`] for the lumen
///    (`r = nipple_inner_r`, z spans the nipple + small overshoot
///    at both ends so the lumen opens at the nipple tip and joins
///    the SDF-carved flange hole at the top).
///
/// Returns `None` when the ribbon's pour-gate kind is
/// [`crate::pour::PourGateKind::None`].
#[must_use]
pub fn build_funnel_solid(ribbon: &Ribbon) -> Option<(Solid, Vec<MatingTransform>)> {
    let spec = match &ribbon.pour_gate {
        PourGateKind::None => return None,
        PourGateKind::Default(spec) => spec,
    };
    let nipple_outer_r = spec.gate_radius_m - NIPPLE_DIAMETRAL_CLEARANCE_M / 2.0;
    let nipple_inner_r = nipple_outer_r - FUNNEL_NIPPLE_WALL_M;
    let cone_top_inner_r = FUNNEL_TOP_OUTER_RADIUS_M - FUNNEL_CONE_WALL_M;

    // Z-extents (axis along +Z, bottom at z=0).
    let nipple_z_top = FUNNEL_NIPPLE_HEIGHT_M;
    let flange_z_top = nipple_z_top + FUNNEL_FLANGE_HEIGHT_M;
    let cone_z_top = flange_z_top + FUNNEL_CONE_HEIGHT_M;

    // Outer body: flange disk ∪ cone frustum. The nipple + contact
    // ring live post-MC as mesh-CSG primitives (see returned Vec).
    let flange_outer =
        Solid::cylinder(FUNNEL_FLANGE_OUTER_RADIUS_M, FUNNEL_FLANGE_HEIGHT_M / 2.0).translate(
            Vector3::new(0.0, 0.0, f64::midpoint(nipple_z_top, flange_z_top)),
        );
    let cone_outer = truncated_cone(
        FUNNEL_FLANGE_OUTER_RADIUS_M,
        FUNNEL_TOP_OUTER_RADIUS_M,
        flange_z_top,
        cone_z_top,
    );
    let outer = flange_outer.union(cone_outer);

    // Inner cavity: cylinder through the flange hole, then taper up
    // through the cone. The portion below z = nipple_z_top has no
    // SDF material to carve (the nipple lives in mesh-CSG); the
    // overshoot at the lower end keeps MC from leaving a thin shelf
    // at the flange underside.
    let inner_lower_z_bot = -INNER_CAVITY_OVERSHOOT_M;
    let inner_lower_z_top = flange_z_top;
    let inner_lower_half_h = (inner_lower_z_top - inner_lower_z_bot) / 2.0;
    let inner_lower_z_center = f64::midpoint(inner_lower_z_bot, inner_lower_z_top);
    let inner_lower = Solid::cylinder(nipple_inner_r, inner_lower_half_h).translate(Vector3::new(
        0.0,
        0.0,
        inner_lower_z_center,
    ));
    let inner_cone = truncated_cone(
        nipple_inner_r,
        cone_top_inner_r,
        flange_z_top,
        cone_z_top + INNER_CAVITY_OVERSHOOT_M,
    );
    let inner = inner_lower.union(inner_cone);

    // Mesh-CSG ops (S7): nipple body as UnionCylinder, then
    // SubtractCylinder for the through-lumen. Order is load-bearing
    // — Union MUST precede the Subtract so the lumen carves through
    // the freshly-added material (manifold3d applies transforms in
    // declared order; recon §5 sequencing principle).
    let z_axis = Unit::new_unchecked(Vector3::new(0.0, 0.0, 1.0));
    let nipple_half_len = FUNNEL_NIPPLE_HEIGHT_M / 2.0;
    let nipple_parent = CylinderParent {
        center_m: Point3::new(0.0, 0.0, nipple_half_len),
        axis: z_axis,
        half_length_m: nipple_half_len,
    };
    // Lumen extends half the nipple plus a small overshoot at each
    // end so the bottom face opens to z<0 (workshop pour) and the
    // top joins the SDF-carved flange hole cleanly. Centred at the
    // nipple's midpoint so both ends overshoot symmetrically.
    let lumen_half_len = nipple_half_len + INNER_CAVITY_OVERSHOOT_M;
    let lumen_parent = CylinderParent {
        center_m: Point3::new(0.0, 0.0, nipple_half_len),
        axis: z_axis,
        half_length_m: lumen_half_len,
    };
    let transforms = vec![
        MatingTransform::UnionCylinder {
            params: CylinderParams {
                parent: nipple_parent,
                radius_m: nipple_outer_r,
                segments: FUNNEL_CYLINDER_SEGMENTS,
            },
        },
        MatingTransform::SubtractCylinder {
            params: CylinderParams {
                parent: lumen_parent,
                radius_m: nipple_inner_r,
                segments: FUNNEL_CYLINDER_SEGMENTS,
            },
        },
    ];
    Some((outer.subtract(inner), transforms))
}

/// Polygonal facet count around the circumference for the funnel's
/// mesh-CSG cylinder primitives (nipple + lumen).
///
/// Same workshop default as the registration pin + plug shaft
/// segment counts (see [`crate::registration::PIN_SEGMENTS`] /
/// [`crate::plug::PLUG_CYLINDER_SEGMENTS`]) — 32 facets at a 5 mm
/// nipple radius is < 0.05 mm chord error, well below workshop
/// tolerances. Part of the determinism contract (same segments →
/// bit-equal output across builds).
pub const FUNNEL_CYLINDER_SEGMENTS: u32 = 32;

/// Build a truncated cone (frustum) with bottom radius `r_bot` at
/// `z_bot` and top radius `r_top` at `z_top` (axis along +Z).
///
/// Constructed by:
/// 1. Building cf-design's `Solid::cone(r_top, h_total)` (apex at
///    `z=0`, base at `z=−h_total`, base radius = `r_top`).
/// 2. Rotating 180° around the X-axis to flip the cone so the apex
///    is at `z=0` and the base extends in `+Z` (matching the
///    funnel's widen-upward orientation).
/// 3. Translating so the virtual apex sits at `z=z_apex` below
///    `z_bot`.
/// 4. Intersecting with a cuboid clipper spanning `z ∈ [z_bot,
///    z_top]` to truncate the cone into a frustum.
///
/// Returns a cylinder when `r_bot >= r_top` (degenerate / non-
/// tapered input) — the funnel module doesn't construct such cases
/// in practice but the degenerate branch keeps the helper total.
fn truncated_cone(r_bot: f64, r_top: f64, z_bot: f64, z_top: f64) -> Solid {
    let h = z_top - z_bot;
    if r_bot >= r_top || h <= 0.0 {
        return Solid::cylinder(r_top.max(r_bot), h.abs() / 2.0).translate(Vector3::new(
            0.0,
            0.0,
            f64::midpoint(z_bot, z_top),
        ));
    }
    let dr = r_top - r_bot;
    let h_total = h * r_top / dr;
    let z_apex = z_bot - r_bot * h / dr;

    let flip = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), std::f64::consts::PI);
    let cone = Solid::cone(r_top, h_total)
        .rotate(flip)
        .translate(Vector3::new(0.0, 0.0, z_apex));

    // Clipper cuboid spans z ∈ [z_bot, z_top] with XY large enough
    // to cover r_top + a small slack (the cuboid only needs to
    // bound the cone's lateral extent at z_top — anywhere larger
    // is fine since the cone narrows toward the apex).
    let clip_xy_half = r_top + 0.001;
    let clip = Solid::cuboid(Vector3::new(clip_xy_half, clip_xy_half, h / 2.0))
        .translate(Vector3::new(0.0, 0.0, f64::midpoint(z_bot, z_top)));
    cone.intersect(clip)
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

    use super::*;
    use crate::pour::{PourGateKind, PourGateSpec};
    use crate::ribbon::{Ribbon, SplitNormal};
    use nalgebra::Point3;

    fn iter1_like_ribbon() -> Ribbon {
        let centerline = vec![Point3::new(-0.050, 0.0, 0.0), Point3::new(0.050, 0.0, 0.0)];
        let split = SplitNormal::new(Vector3::new(0.0, 1.0, 0.0)).unwrap();
        Ribbon::new(centerline, split)
            .unwrap()
            .with_pour_gate(PourGateKind::Default(PourGateSpec::iter1()))
    }

    #[test]
    fn build_funnel_solid_returns_none_when_pour_gate_none() {
        let centerline = vec![Point3::new(-0.050, 0.0, 0.0), Point3::new(0.050, 0.0, 0.0)];
        let split = SplitNormal::new(Vector3::new(0.0, 1.0, 0.0)).unwrap();
        let ribbon = Ribbon::new(centerline, split).unwrap();
        assert!(build_funnel_solid(&ribbon).is_none());
    }

    #[test]
    fn build_funnel_solid_returns_some_when_pour_gate_enabled() {
        let ribbon = iter1_like_ribbon();
        let (funnel, _) = build_funnel_solid(&ribbon).expect("funnel should build");
        let aabb = funnel.bounds().expect("funnel has finite bounds");
        // Total height = nipple (4 mm) + flange (1 mm) + cone (30 mm) = 35 mm.
        let total_height = FUNNEL_NIPPLE_HEIGHT_M + FUNNEL_FLANGE_HEIGHT_M + FUNNEL_CONE_HEIGHT_M;
        assert!(
            (aabb.max.z - total_height).abs() < 0.001,
            "funnel max.z should equal total height {total_height}; got {}",
            aabb.max.z,
        );
        // XY footprint extends to ±FUNNEL_TOP_OUTER_RADIUS_M (top
        // of cone), since top is widest. Allow MC slack.
        assert!(
            aabb.max.x >= FUNNEL_TOP_OUTER_RADIUS_M - 0.001,
            "funnel max.x should reach top radius {FUNNEL_TOP_OUTER_RADIUS_M}; \
             got {}",
            aabb.max.x,
        );
    }

    #[test]
    fn build_funnel_solid_sdf_is_empty_below_flange() {
        // Post-S7: nipple + contact ring no longer live in the
        // SDF (they're emitted as post-MC `UnionCylinder` ops, see
        // `funnel_emits_nipple_ring_lumen_transforms_per_spec`).
        // The SDF only carries the flange disk + cone, so any point
        // below the flange-bottom plane should report "outside"
        // (SDF > 0). The mesh-CSG step is responsible for adding
        // the nipple + ring material post-MC.
        let ribbon = iter1_like_ribbon();
        let (funnel, _) = build_funnel_solid(&ribbon).expect("funnel should build");
        let q_below_flange = Point3::new(0.0, 0.0, 0.002); // 2 mm above bottom — mid-nipple
        assert!(
            funnel.evaluate(&q_below_flange) > 0.0,
            "funnel SDF mid-nipple should be empty post-S7 (nipple lives \
             as mesh-CSG UnionCylinder); got {}",
            funnel.evaluate(&q_below_flange),
        );
        // Just outside the nipple radius (off the central axis) is
        // also empty pre-mesh-CSG.
        let q_off_axis = Point3::new(0.003, 0.0, 0.002); // 3 mm radius, mid-nipple
        assert!(
            funnel.evaluate(&q_off_axis) > 0.0,
            "funnel SDF off-axis below flange should be empty post-S7; got {}",
            funnel.evaluate(&q_off_axis),
        );
    }

    #[test]
    fn build_funnel_solid_has_open_top_at_cone() {
        // At z just below the cone top, on the central axis, the
        // funnel SDF should be > 0 (inside the cone cavity, where
        // workshop pours silicone in).
        let ribbon = iter1_like_ribbon();
        let (funnel, _) = build_funnel_solid(&ribbon).expect("funnel should build");
        let total_height = FUNNEL_NIPPLE_HEIGHT_M + FUNNEL_FLANGE_HEIGHT_M + FUNNEL_CONE_HEIGHT_M;
        let q_cone_interior = Point3::new(0.0, 0.0, total_height - 0.002);
        assert!(
            funnel.evaluate(&q_cone_interior) > 0.0,
            "funnel central axis near top should be in the cone cavity \
             (SDF > 0); got {}",
            funnel.evaluate(&q_cone_interior),
        );
    }

    /// S7 transform-parameter audit: `build_funnel_solid` must emit
    /// the two expected mesh-CSG ops in declared order — nipple
    /// `UnionCylinder` + lumen `SubtractCylinder` — with radii /
    /// half-lengths / poses matching the module-level constants.
    /// Replaces the pre-S7 SDF probes at nipple-interior /
    /// nipple-wall points (the nipple no longer lives in the SDF).
    #[test]
    fn funnel_emits_nipple_and_lumen_transforms_per_spec() {
        let ribbon = iter1_like_ribbon();
        let pour_radius = match &ribbon.pour_gate {
            PourGateKind::Default(spec) => spec.gate_radius_m,
            PourGateKind::None => panic!("iter1_like_ribbon must have pour gate"),
        };
        let (_, transforms) = build_funnel_solid(&ribbon).expect("funnel should build");
        assert_eq!(transforms.len(), 2, "expect [nipple, lumen] ops");

        let expected_nipple_outer_r = pour_radius - NIPPLE_DIAMETRAL_CLEARANCE_M / 2.0;
        let expected_nipple_inner_r = expected_nipple_outer_r - FUNNEL_NIPPLE_WALL_M;
        let nipple_half = FUNNEL_NIPPLE_HEIGHT_M / 2.0;

        // 1. Nipple body — UnionCylinder centred at nipple midpoint.
        match &transforms[0] {
            MatingTransform::UnionCylinder { params } => {
                assert!(
                    (params.radius_m - expected_nipple_outer_r).abs() < f64::EPSILON,
                    "nipple radius should be gate_radius - diametral/2 = \
                     {expected_nipple_outer_r}; got {}",
                    params.radius_m,
                );
                assert!(
                    (params.parent.half_length_m - nipple_half).abs() < f64::EPSILON,
                    "nipple half_length should be FUNNEL_NIPPLE_HEIGHT/2 = \
                     {nipple_half}; got {}",
                    params.parent.half_length_m,
                );
                assert!((params.parent.center_m.z - nipple_half).abs() < f64::EPSILON);
                assert_eq!(params.segments, FUNNEL_CYLINDER_SEGMENTS);
            }
            other => panic!("transforms[0] expected UnionCylinder; got {other:?}"),
        }

        // 2. Lumen — SubtractCylinder with the nipple-inner radius,
        // extended past both nipple ends by INNER_CAVITY_OVERSHOOT_M.
        match &transforms[1] {
            MatingTransform::SubtractCylinder { params } => {
                assert!(
                    (params.radius_m - expected_nipple_inner_r).abs() < f64::EPSILON,
                    "lumen radius should be nipple_inner_r = \
                     {expected_nipple_inner_r}; got {}",
                    params.radius_m,
                );
                let expected_half = nipple_half + INNER_CAVITY_OVERSHOOT_M;
                assert!((params.parent.half_length_m - expected_half).abs() < f64::EPSILON,);
            }
            other => panic!("transforms[1] expected SubtractCylinder; got {other:?}"),
        }
    }

    /// Math-verified M5 fit invariant per
    /// `feedback_math_verify_geometric_contracts`. The funnel nipple
    /// OD and the cup pour-gate-hole ID (carved at the nominal
    /// `2 × gate_radius_m`, see `crate::pour::build_pour_gate_transforms`)
    /// must differ by exactly [`NIPPLE_DIAMETRAL_CLEARANCE_M`] —
    /// asymmetric `/2` convention (funnel side bears all the slack;
    /// cup side stays at nominal). Verified at the transform-parameter
    /// level so a regression on either side's sizing math is caught
    /// without mesh round-trip noise.
    #[test]
    fn funnel_nipple_and_pour_gate_fit_invariant() {
        let ribbon = iter1_like_ribbon();
        let spec = match &ribbon.pour_gate {
            PourGateKind::Default(spec) => spec.clone(),
            PourGateKind::None => panic!("iter1_like_ribbon must have pour gate"),
        };
        let (_, funnel_transforms) = build_funnel_solid(&ribbon).expect("funnel should build");
        let cup_transforms = crate::pour::build_pour_gate_transforms(&ribbon);

        // Funnel nipple radius = transforms[0]'s UnionCylinder radius.
        let funnel_nipple_r = match &funnel_transforms[0] {
            MatingTransform::UnionCylinder { params } => params.radius_m,
            other => panic!("funnel_transforms[0] expected UnionCylinder; got {other:?}"),
        };
        // Cup pour-gate hole radius = first SubtractCylinder in the
        // cup-side transforms (pour leg; vent leg follows when
        // include_vent).
        let cup_pour_r = cup_transforms
            .iter()
            .find_map(|t| match t {
                MatingTransform::SubtractCylinder { params } => Some(params.radius_m),
                _ => None,
            })
            .expect("cup-side pour-gate transforms must include a SubtractCylinder");

        // Diametral gap = cup hole Ø − nipple Ø = 2 × (cup_r − funnel_r).
        let diametral_gap_m = 2.0 * (cup_pour_r - funnel_nipple_r);
        assert!(
            (diametral_gap_m - NIPPLE_DIAMETRAL_CLEARANCE_M).abs() < 1.0e-9,
            "diametral gap should equal NIPPLE_DIAMETRAL_CLEARANCE_M = \
             {NIPPLE_DIAMETRAL_CLEARANCE_M}; got {diametral_gap_m} \
             (funnel_r={funnel_nipple_r}, cup_r={cup_pour_r})",
        );
        // Cup hole stays at nominal `spec.gate_radius_m` (asymmetric
        // `/2` convention — funnel side bears the slack).
        assert!(
            (cup_pour_r - spec.gate_radius_m).abs() < 1.0e-9,
            "cup pour-gate hole radius should equal gate_radius_m = {} \
             (cup side stays nominal); got {cup_pour_r}",
            spec.gate_radius_m,
        );
    }
}
