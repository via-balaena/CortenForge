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
//!   hole). Inner Ø = outer Ø − 2 × [`FUNNEL_NIPPLE_WALL_M`].
//! - **Flange** above the nipple: cylinder outer Ø = 2 ×
//!   [`FUNNEL_FLANGE_OUTER_RADIUS_M`]. Rests flat on the cup wall
//!   outer surface around the pour-gate hole.
//! - **Cone** above the flange: truncated cone widening from
//!   flange Ø at the bottom to 2 × [`FUNNEL_TOP_OUTER_RADIUS_M`]
//!   at the top.
//! - **Hollow interior**: single continuous tapered cavity from
//!   the nipple's inner Ø at `z=0` up to (top Ø −
//!   2 × [`FUNNEL_CONE_WALL_M`]) at `z=top`.
//!
//! ## Architectural placement (recon-4 pattern)
//!
//! Nipple + flange + cone are all **SDF-meshed bulk geometry**:
//! they weld into one continuous funnel body via SDF unions, and
//! MC produces one connected mesh in a single pass. S7's migration
//! of the nipple OD to a post-MC `MatingTransform::UnionCylinder`
//! was reverted 2026-05-23 — the post-MC boolean union of the
//! nipple cylinder onto the SDF-meshed flange ran into the same
//! paradigm-boundary failure as recon-4 (P)'s seam-face film:
//! exactly-coincident faces (nipple top at z=4 mm vs flange bottom
//! at z=4 mm) produced 2 disconnected mesh components, and any
//! sized overlap-bias created `ThinWall` + `SelfIntersecting` +
//! `ExcessiveOverhang` F4 issues. The architectural-correctness
//! principle (`docs/CF_CAST_SEAM_FACE_FILM_RECON_PLAN.md` §F-2): a
//! feature welded into bulk geometry belongs to the SDF/MC
//! paradigm; only fine mating-feature primitives that need
//! bit-precise diametral fit belong to mesh-CSG. The nipple's
//! pour-gate-fit precision is bounded by [`NIPPLE_DIAMETRAL_CLEARANCE_M`]
//! (0.5 mm workshop slack), well above MC-quantization noise at
//! `crate::spec::FUNNEL_MAX_CELL_SIZE_M` (1 mm cells) — so the
//! S7 "bit-precise OD" benefit was workshop-irrelevant.
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
//! `crate::spec::FUNNEL_MAX_CELL_SIZE_M` (1 mm) and the resulting
//! mesh fails F4 `ThinWall` checks. Deferred to a future polish
//! session: either drop the funnel MC cell size below 0.5 mm and
//! ship a flange-underside annular lip in the SDF, or wait until
//! iter-2's workshop print surfaces a wobble defect that motivates
//! a specific implementation.

use cf_design::Solid;
use nalgebra::{UnitQuaternion, Vector3};

use crate::mesh_csg::MatingTransform;
use crate::pour::PourGateKind;
use crate::ribbon::Ribbon;

/// Nipple height (m).
///
/// 4 mm — just enough for self-alignment during insertion into
/// the pour-gate hole; short so the nipple's narrower inner Ø
/// isn't a long viscous bottleneck.
pub const FUNNEL_NIPPLE_HEIGHT_M: f64 = 0.004;

/// Nipple wall thickness (m). 1.5 mm = 3-4 perimeters at a 0.4 mm
/// FDM nozzle.
///
/// Sized at 1.5 mm rather than 1 mm to clear cf-cast's F4
/// `min_wall_thickness: 1.0` threshold with margin — MC stair-step
/// thinning at the nipple's outer surface can push a 1 mm-design
/// wall below 1.0 mm in places, triggering Critical `ThinWall`
/// failures. Trade-off: nipple inner Ø drops from 7.8 mm
/// (theoretical 1 mm wall) to 6.8 mm (`1.5` mm wall) — still wider
/// than the pre-iter-1 6 mm pour-gate Ø, and the 4 mm nipple
/// height keeps the bottleneck short.
pub const FUNNEL_NIPPLE_WALL_M: f64 = 0.0015;

/// Diametral clearance between the nipple OD and the cup pour-gate
/// hole ID (m).
///
/// Total diametral gap = this value (asymmetric `/2` convention:
/// cup hole stays at exactly [`crate::PourGateSpec::gate_radius_m`];
/// funnel nipple shrinks by half this clearance per side).
///
/// 0.5 mm — resting-contact fit per `docs/CF_CAST_MATING_FEATURES_RECON.md`
/// §9 M5. Absorbs both FDM bead-bulge (cup hole prints undersized;
/// funnel nipple prints oversized) AND MC-quantization noise on
/// the SDF-meshed nipple OD at the `crate::spec::FUNNEL_MAX_CELL_SIZE_M`
/// (1 mm) cell size (~0.5 mm radial precision). Workshop user
/// reams if first print binds.
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

/// MC tolerance overshoot for the inner cavity at the funnel's
/// open ends (m).
///
/// 0.5 mm — small overshoot past the funnel's top + bottom
/// surfaces ensures MC produces clean openings (workshop pour
/// entry + exit) instead of leaving stair-step shelves at the
/// cavity boundary.
const INNER_CAVITY_OVERSHOOT_M: f64 = 0.0005;

/// Build the workshop pour funnel [`Solid`] for the given ribbon,
/// or `None` if the ribbon has no pour gate enabled (no funnel
/// needed).
///
/// Returns `Some((solid, transforms))` where `solid` is the
/// SDF-encoded funnel body (nipple + flange + cone welded into one
/// continuous SDF, lumen carved through the entire interior) and
/// `transforms` is an empty Vec — the funnel composes entirely in
/// SDF/MC; no post-MC mesh-CSG operations run (see the module
/// docstring §"Architectural placement" for the recon-4 paradigm-
/// boundary rationale that reverted the S7 post-MC nipple
/// migration).
///
/// The funnel's nipple OD is sized to slide into the ribbon's
/// pour-gate hole: nipple OD = 2 × `pour_gate.gate_radius_m` −
/// [`NIPPLE_DIAMETRAL_CLEARANCE_M`] (asymmetric `/2` convention —
/// funnel side bears all the slack; cup-piece pour-gate-hole stays
/// at the nominal `2 × gate_radius_m`). The cup-side pour-gate
/// carve remains post-MC mesh-CSG (`crate::pour::build_pour_gate_transforms`)
/// because it lives on a separate cup-piece mesh and the
/// `SubtractCylinder` is a pure interior carve with no
/// paradigm-boundary junction.
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

    // Outer body: nipple cylinder ∪ flange disk ∪ cone frustum.
    // The nipple's top extends half the flange height past the
    // nominal nipple_z_top so the SDF union has overlap to
    // interpolate across at the junction. At the
    // FUNNEL_MAX_CELL_SIZE_M (1 mm) cell size the MC cell straddling
    // exactly-coincident nipple-top / flange-bottom z values can't
    // resolve the SDF union into one connected manifold (without
    // this overlap the mesh splits into a floating nipple component
    // + a flange+cone shell starting at z = 4.5 mm rather than the
    // nominal flange-bottom z = 4 mm — caught by
    // `funnel_mesh_is_single_connected_component`). The overlap
    // is INTERIOR to the flange's annular volume (nipple OD 4.75 mm
    // is fully contained inside flange OD 10 mm / ID 3.25 mm), so
    // workshop-visible geometry is unchanged.
    let nipple_outer_h = FUNNEL_NIPPLE_HEIGHT_M + FUNNEL_FLANGE_HEIGHT_M / 2.0;
    let nipple_outer = Solid::cylinder(nipple_outer_r, nipple_outer_h / 2.0)
        .translate(Vector3::new(0.0, 0.0, nipple_outer_h / 2.0));
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
    let outer = nipple_outer.union(flange_outer).union(cone_outer);

    // Inner cavity: cylinder through nipple + flange, then taper
    // up through the cone. Tiny overshoot at both ends so MC cuts
    // cleanly at the funnel openings (workshop pour-entry below
    // z=0, ladle-pour mouth above the cone top).
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

    // Funnel composes entirely in SDF/MC — no post-MC mesh-CSG ops.
    Some((outer.subtract(inner), Vec::new()))
}

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

    /// Nipple SDF should contain its wall material between
    /// `nipple_inner_r` and `nipple_outer_r` at mid-nipple z (the
    /// recon-4 S7 revert restored the nipple to SDF/MC so the
    /// boolean union with the flange welds continuously, fixing the
    /// workshop iter-1 cf-view-smoke disconnection).
    #[test]
    fn build_funnel_solid_nipple_wall_is_in_sdf() {
        let ribbon = iter1_like_ribbon();
        let spec = match &ribbon.pour_gate {
            PourGateKind::Default(s) => s.clone(),
            PourGateKind::None => panic!("iter1_like_ribbon must have pour gate"),
        };
        let (funnel, _) = build_funnel_solid(&ribbon).expect("funnel should build");
        let nipple_outer_r = spec.gate_radius_m - NIPPLE_DIAMETRAL_CLEARANCE_M / 2.0;
        let nipple_inner_r = nipple_outer_r - FUNNEL_NIPPLE_WALL_M;
        let mid_wall_r = f64::midpoint(nipple_inner_r, nipple_outer_r);
        let mid_z = FUNNEL_NIPPLE_HEIGHT_M / 2.0;
        // Mid-nipple-wall point should be INSIDE the funnel (SDF < 0).
        let q_wall = Point3::new(mid_wall_r, 0.0, mid_z);
        assert!(
            funnel.evaluate(&q_wall) < 0.0,
            "funnel SDF at mid-nipple-wall (r={mid_wall_r}, z={mid_z}) should be inside; got {}",
            funnel.evaluate(&q_wall),
        );
        // On-axis at mid-nipple is inside the lumen → outside (SDF > 0).
        let q_lumen = Point3::new(0.0, 0.0, mid_z);
        assert!(
            funnel.evaluate(&q_lumen) > 0.0,
            "funnel SDF on-axis at mid-nipple (lumen interior) should be outside; got {}",
            funnel.evaluate(&q_lumen),
        );
        // Off-axis beyond nipple OD is outside the funnel body.
        let q_off = Point3::new(nipple_outer_r + 0.002, 0.0, mid_z);
        assert!(
            funnel.evaluate(&q_off) > 0.0,
            "funnel SDF beyond nipple OD at mid-nipple z should be outside; got {}",
            funnel.evaluate(&q_off),
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

    /// `build_funnel_solid` returns no mesh-CSG transforms — the
    /// funnel composes entirely in SDF/MC (the S7 nipple migration
    /// was reverted 2026-05-23 per the module docstring §"Architectural
    /// placement" rationale).
    #[test]
    fn build_funnel_solid_returns_empty_transforms() {
        let ribbon = iter1_like_ribbon();
        let (_, transforms) = build_funnel_solid(&ribbon).expect("funnel should build");
        assert!(
            transforms.is_empty(),
            "funnel composes entirely in SDF/MC; expected empty transforms vec, got {} ops",
            transforms.len(),
        );
    }

    /// Math-verified M5 fit invariant per
    /// `feedback_math_verify_geometric_contracts`. The funnel nipple
    /// OD (sized in SDF as `gate_radius_m − NIPPLE_DIAMETRAL_CLEARANCE_M / 2`)
    /// and the cup pour-gate-hole ID (carved at the nominal
    /// `2 × gate_radius_m` via mesh-CSG, see
    /// `crate::pour::build_pour_gate_transforms`) must differ by
    /// exactly [`NIPPLE_DIAMETRAL_CLEARANCE_M`] — asymmetric `/2`
    /// convention (funnel side bears all the slack; cup side stays
    /// at nominal).
    ///
    /// Verified at the spec-sizing level (funnel side) + the
    /// transform-parameter level (cup side) so a regression on
    /// either side's sizing math is caught without mesh round-trip
    /// noise. NB: the funnel nipple OD is MC-quantized to the
    /// `FUNNEL_MAX_CELL_SIZE_M` cell size (~0.5 mm radial precision
    /// at 1 mm cells); workshop slack absorbs this.
    #[test]
    fn funnel_nipple_and_pour_gate_fit_invariant() {
        let ribbon = iter1_like_ribbon();
        let spec = match &ribbon.pour_gate {
            PourGateKind::Default(spec) => spec.clone(),
            PourGateKind::None => panic!("iter1_like_ribbon must have pour gate"),
        };
        // Funnel nipple OD is derived in build_funnel_solid as
        // `gate_radius_m - NIPPLE_DIAMETRAL_CLEARANCE_M / 2`.
        let funnel_nipple_r = spec.gate_radius_m - NIPPLE_DIAMETRAL_CLEARANCE_M / 2.0;

        let cup_transforms = crate::pour::build_pour_gate_transforms(&ribbon);
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

    /// Funnel connectivity regression guard — workshop iter-1
    /// cf-view smoke (2026-05-23) surfaced a 2-component funnel.stl
    /// when the nipple was a post-MC `UnionCylinder` boolean-unioned
    /// onto the SDF-meshed flange (exactly-coincident faces at
    /// z = `FUNNEL_NIPPLE_HEIGHT_M` produced disconnected output).
    /// The recon-4-pattern revert moves the nipple back to SDF so
    /// MC produces one continuous mesh in a single pass.
    ///
    /// Asserts the full pipeline (SDF → MC → mesh-CSG transforms)
    /// produces a SINGLE connected component. Math-instrumented at
    /// the lib-test level (`mesh_repair::components::find_connected_components`)
    /// so the regression is caught without a workshop STL regen.
    #[test]
    fn funnel_mesh_is_single_connected_component() {
        use crate::error::CastTarget;
        use crate::mesh_csg::apply_mating_transforms;
        use crate::mesher::solid_to_mm_mesh;
        use mesh_repair::components::find_connected_components;

        let ribbon = iter1_like_ribbon();
        let (solid, transforms) =
            build_funnel_solid(&ribbon).expect("iter1 fixture has pour gate enabled");
        // 1 mm cell size matches `FUNNEL_MAX_CELL_SIZE_M` in the
        // production funnel mesher path.
        let mesh = solid_to_mm_mesh(&solid, 0.001, CastTarget::Funnel).expect("funnel MC");
        let mesh = apply_mating_transforms(mesh, &transforms, CastTarget::Funnel)
            .expect("funnel mesh-CSG");
        let analysis = find_connected_components(&mesh);
        assert_eq!(
            analysis.component_count,
            1,
            "funnel mesh must be a single connected component (nipple + flange + \
             cone welded by SDF union, meshed by MC in one pass); got {} components \
             ({} verts / {} faces). A regression here means either (a) the SDF \
             chain dropped the nipple_outer union (the S7-era post-MC mesh-CSG \
             nipple migration produced 2 disconnected components — see the module \
             docstring for the recon-4 pattern), or (b) a future migration \
             introduced a new paradigm-boundary cylinder Union onto the \
             SDF-meshed funnel without an SDF-side weld.",
            analysis.component_count,
            mesh.vertices.len(),
            mesh.faces.len(),
        );
    }
}
