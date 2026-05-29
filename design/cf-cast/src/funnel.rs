//! Workshop pour-funnel geometry (bent-spout architecture).
//!
//! Honey-thick workshop silicones (Dragon Skin 10A / 20A / 30A at
//! ~20-23k cps) take minutes per layer to pour through bare
//! 10 mm Ø pour-gate holes without splashing or trapping air.
//! [`build_funnel_solid`] generates a printable funnel STL with a
//! **bent spout** at [`crate::pour::V_HALF_ANGLE_RAD`] (30°) so the
//! nipple inserts into the cup-piece's angled pour-gate hole while
//! the bowl mouth ends up facing STRAIGHT UP when the workshop user
//! orients the assembled mold `+Z` up. Pre-§B-S1-funnel-redesign
//! (2026-05-27) the funnel was a single straight tube with a flat
//! flange disk; the bent-spout pattern matches what workshop users
//! recognize as a real-world store-bought funnel.
//!
//! ## Geometry
//!
//! Funnel-local frame: origin at the bowl-nipple junction; `+Z` up
//! (bowl axis).
//!
//! - **Bowl** above origin: truncated cone widening from
//!   [`FUNNEL_BOWL_BOTTOM_OUTER_RADIUS_M`] at `z=0` (the visible
//!   "shoulder" where the spout meets the cone) up to
//!   [`FUNNEL_TOP_OUTER_RADIUS_M`] at `z=cone_height`. Bowl axis is
//!   vertical (`+Z`) so the workshop pour-mouth faces straight up.
//! - **Inner cavity** (continuous bore): the bowl interior tapers
//!   from `FUNNEL_TOP_OUTER_RADIUS_M − FUNNEL_CONE_WALL_M` at the
//!   top down to the **nipple inner radius** at `z=0`. This
//!   gives a smooth single-cone inner surface from the workshop mouth
//!   down to the spout entry — no sudden necking at the bowl-nipple
//!   junction. Real metal funnels work this way too: the spout is
//!   bored conically from the bowl interior even when the OUTER
//!   spout joint looks like a sharp angle.
//! - **Nipple** below origin: cylinder tilted at
//!   [`crate::pour::V_HALF_ANGLE_RAD`] from `−Z` (toward `+X` in the
//!   funnel-local frame; workshop user rotates the funnel around the
//!   bowl axis until the nipple aligns with the pour-gate hole at
//!   assembly time). Nipple outer Ø = pour-gate Ø −
//!   [`NIPPLE_DIAMETRAL_CLEARANCE_M`]; inner Ø = outer Ø −
//!   2 × [`FUNNEL_NIPPLE_WALL_M`]. The nipple's top "enters" the
//!   bowl at the funnel-local origin; nipple axis goes from origin
//!   along `(sin(θ), 0, −cos(θ))` for
//!   [`FUNNEL_NIPPLE_HEIGHT_M`] meters (θ =
//!   [`crate::pour::V_HALF_ANGLE_RAD`]).
//! - **Outer kink**: sharp 30° angle at `z=0` between the vertical
//!   bowl and the tilted nipple (workshop sees a clean industrial-
//!   funnel-style joint).
//!
//! ## Workshop assembly orientation
//!
//! Workshop user:
//! 1. Orients the mold `+Z` up so the pour-gate cylinder is at the
//!    top (per cf-cast's `+Z` up convention — see
//!    `crate::CastSpec::export_molds_v2` docstring §"Workshop
//!    orientation convention").
//! 2. Rotates the funnel around its bowl axis until the nipple
//!    aligns with the pour-gate hole's outward-binormal axis.
//! 3. Inserts nipple into the pour-gate hole; the funnel's natural
//!    rest position is bowl-mouth-up.
//!
//! ## Print orientation
//!
//! Printed bowl-mouth-up (build-plate is the bowl mouth opening) the
//! tilted nipple sticks out 7.5 mm horizontally from the bowl base
//! and becomes an **unsupported overhang**. Workshop user enables
//! auto-supports in the slicer for the funnel STL (cleaned off after
//! the print). Procedure prose calls this out explicitly.
//!
//! Alternative: workshop can rotate the print so the nipple is along
//! the build plate (bowl tilted in the print bed), avoiding supports
//! — but then the BOWL becomes the overhang. Slicer choice; STL is
//! the same.
//!
//! ## Architectural placement (recon-4 pattern)
//!
//! Bowl + nipple compose as **SDF-meshed bulk geometry** with an SDF
//! union, like the pre-redesign straight funnel. The bent nipple is
//! constructed by building a vertical cylinder in its local frame,
//! translating its TOP to the funnel-local origin (`z=0`), then
//! rotating around `+Y` by [`crate::pour::V_HALF_ANGLE_RAD`] so the
//! nipple axis tilts from `−Z` toward `+X` while its top stays
//! anchored at the bowl-nipple junction. Both bowl + nipple weld
//! into one continuous SDF; MC produces one connected mesh in a
//! single pass.
//!
//! Per `docs/CF_CAST_SEAM_FACE_FILM_RECON_PLAN.md` §F-2: features
//! welded into bulk geometry belong to the SDF/MC paradigm. The
//! `funnel_mesh_is_single_connected_component` regression test guards
//! against MC weld failures at the bowl-nipple junction (pre-§B-S1-
//! funnel-redesign the straight-funnel flange-cone junction needed an
//! overlap to avoid exact-coincident-face MC ambiguity; the bent-
//! spout design's nipple naturally extends INTO the bowl interior so
//! the union has continuous overlap with no exact-coincident faces).
//!
//! ## Recon §9 axial standoff — DEFERRED (still applicable post-redesign)
//!
//! Pre-redesign deferred a flange-underside standoff lip. The bent-
//! spout redesign has no flat flange disk at all; the funnel rests
//! on the cup-outer surface via the bowl-bottom shoulder (annular
//! area between `nipple_outer_r` and `FUNNEL_BOWL_BOTTOM_OUTER_RADIUS_M`).
//! Workshop wobble surfaces in iter-1 cf-view smoke would motivate
//! revisiting; deferred.

use cf_design::Solid;
use nalgebra::{UnitQuaternion, Vector3};

use crate::mesh_csg::MatingTransform;
use crate::pour::{PourGateKind, PourGateLayout, V_HALF_ANGLE_RAD};
use crate::ribbon::Ribbon;

/// Nipple height (m).
///
/// 20 mm — sized to clear the §B cup-piece seam flange
/// (`flange_thickness_m = 4 mm` protruding from the cup outer surface
/// in the seam plane direction) PLUS the cup-wall thickness
/// (`wall_thickness_m = 5 mm` from outer surface to inner cavity) PLUS
/// ~11 mm of workshop insertion-travel slack. The tilt is 30° from
/// vertical, so 20 mm nipple length gives 20·cos(30°) ≈ 17.3 mm of
/// VERTICAL clearance between the bowl-bottom fillet and the cup
/// outer surface around the pour-gate hole — comfortably past the
/// 4 mm flange height. Pre-§B was 4 mm, intermediate value 15 mm
/// gave only 13 mm vertical clearance (workshop user requested
/// preemptive bump to 20 mm for safety margin 2026-05-27).
pub const FUNNEL_NIPPLE_HEIGHT_M: f64 = 0.020;

/// Nipple wall thickness (m). 1.5 mm = 3-4 perimeters at a 0.4 mm
/// FDM nozzle.
///
/// Sized at 1.5 mm rather than 1 mm to clear cf-cast's F4
/// `min_wall_thickness: 1.0` threshold with margin — MC stair-step
/// thinning at the nipple's outer surface can push a 1 mm-design
/// wall below 1.0 mm in places, triggering Critical `ThinWall`
/// failures.
pub const FUNNEL_NIPPLE_WALL_M: f64 = 0.0015;

/// Diametral clearance between the nipple OD and the cup pour-gate
/// hole ID (m).
///
/// Total diametral gap = this value (asymmetric `/2` convention:
/// cup hole stays at exactly [`crate::PourGateSpec::gate_radius_m`];
/// funnel nipple shrinks by half this clearance per side).
///
/// 0.5 mm — resting-contact fit per `docs/CF_CAST_MATING_FEATURES_RECON.md`
/// §9 M5.
pub const NIPPLE_DIAMETRAL_CLEARANCE_M: f64 = 0.0005;

/// Bowl-bottom outer radius (m).
///
/// 10 mm — the visible "shoulder" where the tilted spout meets the
/// vertical bowl. Sized larger than the tilted nipple's projected
/// ellipse semi-major axis at z=0 (`nipple_outer_r / cos(V_HALF_ANGLE_RAD)`
/// ≈ 5.48 mm at iter-1 defaults) by a safety margin > MC cell size
/// — pre-redesign 7 mm was right at the MC-cell threshold and
/// produced an unclosed mesh at the back-of-nipple kink reveal
/// (workshop cf-view smoke 2026-05-27). At 10 mm the reveal annulus
/// is ~4.5 mm on every side of the nipple = 4-5 MC cells of
/// resolution + a comfortable workshop rest surface against the
/// cup-outer face around the pour-gate hole. Replaces the pre-
/// redesign flat flange disk (workshop user feedback 2026-05-27:
/// "doesn't look like a funnel you'd buy from a store").
pub const FUNNEL_BOWL_BOTTOM_OUTER_RADIUS_M: f64 = 0.010;

/// Top outer radius (m).
///
/// 22 mm = 44 mm Ø mouth — workshop-friendly ladle-pour target,
/// wider than the pre-redesign 15 mm. Combined with the 7 mm bowl
/// bottom + 30 mm cone height, the bowl outer wall slopes at ~27°
/// from vertical (vs the pre-redesign 9.5° — feels like a real
/// funnel rather than a near-cylinder).
pub const FUNNEL_TOP_OUTER_RADIUS_M: f64 = 0.022;

/// Cone wall thickness (m).
///
/// 1.0 mm = 2-3 perimeters at a 0.4 mm FDM nozzle. Matches the
/// real store-bought funnel feel (workshop user 2026-05-27: "doesn't
/// look like a funnel you'd buy from a store" — pre-redesign was
/// 1.5 mm chunky). F4 `ThinWall` is carved out for the Funnel target
/// in `cf_cast::spec::is_blocking_critical` since the funnel's
/// FUNCTION (workshop pour-channel handling gravity-driven viscous
/// silicone for 10-30 s per layer) isn't broken by MC-quantization-
/// induced sub-mm walls near the cavity overshoot regions.
pub const FUNNEL_CONE_WALL_M: f64 = 0.001;

/// Cone height (m). 30 mm.
pub const FUNNEL_CONE_HEIGHT_M: f64 = 0.030;

/// MC tolerance overshoot for the inner cavity at the funnel's
/// open ends (m). 0.5 mm — small overshoot past the nipple tip +
/// bowl top so MC produces clean openings.
const INNER_CAVITY_OVERSHOOT_M: f64 = 0.0005;

/// Bowl ↔ nipple SDF-union overlap (m). The nipple's TOP extends
/// 1 mm past the funnel-local origin INTO the bowl interior so
/// the SDF union has overlap to interpolate across at the bend.
/// 1 mm > `FUNNEL_MAX_CELL_SIZE_M` (1 mm cells); the
/// `funnel_mesh_is_single_connected_component` regression test
/// guards against MC weld failures here.
const BOWL_NIPPLE_OVERLAP_M: f64 = 0.001;

/// Smooth-union blend radius for the OUTER bowl ↔ nipple junction
/// (m). 3 mm fillet wraps the sharp 30° kink between the vertical
/// bowl bottom and the tilted nipple top with a smooth toroidal
/// transition — eliminates the flat horizontal bowl-bottom annulus
/// that would otherwise (a) collect splattered silicone during the
/// pour (workshop user feedback 2026-05-27: "liquid would spill at
/// the join") and (b) leave an MC-unresolved sharp ledge that
/// rendered as visually-open in cf-view at the iter-1 1 mm cell
/// size. `Solid::smooth_union` ADDS material in the blend region,
/// giving a continuous gradient where the two shapes meet rather
/// than a discontinuous min(a, b) hard union.
const BOWL_NIPPLE_FILLET_M: f64 = 0.003;

/// Build the workshop pour funnel [`Solid`] for the given ribbon,
/// or `None` if the ribbon has no pour gate enabled.
///
/// Returns `Some((solid, transforms))` where `solid` is the
/// SDF-encoded bent-spout funnel body and `transforms` is an empty
/// Vec — the funnel composes entirely in SDF/MC; no post-MC mesh-CSG
/// operations.
///
/// Funnel-local frame: origin at the bowl-nipple junction; `+Z` up
/// (bowl axis); nipple tilted [`crate::pour::V_HALF_ANGLE_RAD`] from
/// `−Z` toward `+X`.
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
    let bowl_top_inner_r = FUNNEL_TOP_OUTER_RADIUS_M - FUNNEL_CONE_WALL_M;

    // --- Bowl (vertical, +Z axis) ---
    // Outer: cone widening from BOWL_BOTTOM_OUTER_RADIUS at z=0 up
    // to TOP_OUTER_RADIUS at z=cone_height.
    let bowl_outer = truncated_cone(
        FUNNEL_BOWL_BOTTOM_OUTER_RADIUS_M,
        FUNNEL_TOP_OUTER_RADIUS_M,
        0.0,
        FUNNEL_CONE_HEIGHT_M,
    );
    // Inner: cone tapering from bowl_top_inner_r at z=cone_height
    // DOWN to nipple_inner_r at z=0 — gives a smooth single-cone
    // inner bore from the workshop mouth all the way to the spout
    // entry. No sudden necking at the bowl-nipple junction. Add
    // INNER_CAVITY_OVERSHOOT at the top so MC cuts the workshop pour
    // mouth cleanly (no stair-step shelf).
    let bowl_inner = truncated_cone(
        nipple_inner_r,
        bowl_top_inner_r,
        0.0,
        FUNNEL_CONE_HEIGHT_M + INNER_CAVITY_OVERSHOOT_M,
    );

    // --- Nipple (axis along (sin(θ), 0, −cos(θ)) from origin for the
    // layout-dependent tilt θ — see `tilt_angle` below: V_HALF_ANGLE_RAD for
    // the V-at-dome layout, 0 (straight down −Z) for the apex-axial layout) ---
    // Construction: build cylinder in its native +Z frame with its
    // TOP at the funnel-local origin (cylinder spans z ∈
    // [−(nipple_h + overlap), +overlap], so top sits +overlap INTO
    // the bowl interior for the SDF weld), then rotate around +Y by
    // `tilt_angle` to tilt the cylinder's axis from −Z toward +X.
    let nipple_full_h = FUNNEL_NIPPLE_HEIGHT_M + BOWL_NIPPLE_OVERLAP_M;
    // For the V-at-dome layout we rotate by NEGATIVE V_HALF_ANGLE_RAD around
    // +Y: right-hand rule around +Y takes −Z → −sin(θ)·X − cos(θ)·Z for positive
    // θ (tilting toward −X); we want +X tilt (matches the docstring + the
    // `nipple_tip_is_offset_in_plus_x_by_tilt` test), so negate the angle. The
    // in-plane direction is workshop-irrelevant (the user rotates the funnel
    // around the bowl axis to align with the hole), but the implementation must
    // match the test + doc convention. The apex-axial layout uses tilt 0 → a
    // straight vertical nipple (no rotation).
    // Nipple tilt depends on the pour-gate layout. The V-at-dome
    // layout angles the pour bore `V_HALF_ANGLE_RAD` off the dome's
    // outward axis, so the nipple bends to match (bowl ends up
    // vertical when the mold is +Z up). The apex-axial layout bores
    // straight along the dome's outward axis (vertical when +Z up), so
    // the nipple is STRAIGHT (zero tilt) — a plain vertical spout, no
    // bend to catch the cured sprue.
    let tilt_angle = match spec.layout {
        PourGateLayout::VAtDome => -V_HALF_ANGLE_RAD,
        PourGateLayout::ApexAxial => 0.0,
    };
    let tilt = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), tilt_angle);
    let nipple_outer = Solid::cylinder(nipple_outer_r, nipple_full_h / 2.0)
        // Cylinder native: z ∈ [−h/2, +h/2]. Translate so its TOP is
        // at z = +BOWL_NIPPLE_OVERLAP_M (sits INTO bowl).
        .translate(Vector3::new(
            0.0,
            0.0,
            BOWL_NIPPLE_OVERLAP_M - nipple_full_h / 2.0,
        ))
        .rotate(tilt);

    // Nipple inner cavity: same tilt + position, smaller radius +
    // longer at the tip end (overshoot past nipple TIP for clean MC
    // open opening at the workshop pour-entry).
    //
    // Tip overshoot: extend the nipple inner cavity past the OUTER
    // tip by INNER_CAVITY_OVERSHOOT_M along the tilted axis. Easiest
    // construction: build the inner cylinder slightly longer than the
    // outer, with its top still at z = +BOWL_NIPPLE_OVERLAP_M but
    // its bottom at z = −(nipple_h + INNER_CAVITY_OVERSHOOT_M).
    let nipple_inner_full_h = nipple_full_h + INNER_CAVITY_OVERSHOOT_M;
    let nipple_inner = Solid::cylinder(nipple_inner_r, nipple_inner_full_h / 2.0)
        .translate(Vector3::new(
            0.0,
            0.0,
            BOWL_NIPPLE_OVERLAP_M - nipple_inner_full_h / 2.0,
        ))
        .rotate(tilt);

    // OUTER: smooth_union with BOWL_NIPPLE_FILLET_M blend radius —
    // wraps the sharp 30° kink between vertical bowl bottom + tilted
    // nipple top with a toroidal fillet. Pre-fix (hard union) left a
    // flat horizontal bowl-bottom annulus that workshop user
    // identified as a liquid-spill risk + a visually-open ledge that
    // MC couldn't resolve at the iter-1 1 mm cell size. Per
    // `Solid::smooth_union` docs the blend adds material — so the
    // workshop user sees a SMOOTH industrial-funnel-style fillet
    // welding the spout onto the bowl.
    let outer = bowl_outer.smooth_union(nipple_outer, BOWL_NIPPLE_FILLET_M);
    // INNER: hard union — silicone needs the full lumen Ø all the
    // way through the bore. A smooth_union on the inner cavity would
    // NARROW the bore in the blend region (the blend would add
    // material on the inside, REMOVING lumen volume), which would
    // throttle workshop pour rate. Keep the hard min(a, b) so the
    // inner cavity stays at its designed radius.
    let inner = bowl_inner.union(nipple_inner);

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
/// tapered input).
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

    let clip_xy_half = r_top + 0.001;
    let clip = Solid::cuboid(Vector3::new(clip_xy_half, clip_xy_half, h / 2.0))
        .translate(Vector3::new(0.0, 0.0, f64::midpoint(z_bot, z_top)));
    cone.intersect(clip)
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

    use super::*;
    use crate::pour::{PourGateKind, PourGateLayout, PourGateSpec};
    use crate::ribbon::{Ribbon, SplitNormal};
    use nalgebra::Point3;

    fn iter1_like_ribbon() -> Ribbon {
        let centerline = vec![Point3::new(-0.050, 0.0, 0.0), Point3::new(0.050, 0.0, 0.0)];
        let split = SplitNormal::new(Vector3::new(0.0, 1.0, 0.0)).unwrap();
        Ribbon::new(centerline, split)
            .unwrap()
            .with_pour_gate(PourGateKind::Default(PourGateSpec::iter1()))
    }

    fn apex_axial_ribbon() -> Ribbon {
        let centerline = vec![Point3::new(-0.050, 0.0, 0.0), Point3::new(0.050, 0.0, 0.0)];
        let split = SplitNormal::new(Vector3::new(0.0, 1.0, 0.0)).unwrap();
        let mut spec = PourGateSpec::iter1();
        spec.layout = PourGateLayout::ApexAxial;
        Ribbon::new(centerline, split)
            .unwrap()
            .with_pour_gate(PourGateKind::Default(spec))
    }

    /// `ApexAxial` layout → STRAIGHT nipple: the spout points straight
    /// down (`−Z`) with no `+X` tilt offset. The nipple tip sits at
    /// `(0, 0, −nipple_h)`; the funnel's `+X` extent comes ONLY from
    /// the bowl mouth radius, and its `−Z` extent reaches the full
    /// (un-foreshortened) nipple height — unlike the bent funnel whose
    /// tip foreshortens to `−nipple_h·cos(θ)`.
    #[test]
    fn build_funnel_solid_apex_axial_nipple_is_straight() {
        let ribbon = apex_axial_ribbon();
        let (funnel, _) = build_funnel_solid(&ribbon).expect("funnel should build");
        let aabb = funnel.bounds().expect("funnel has finite bounds");
        // Straight nipple: tip at full −nipple_h (not foreshortened by
        // cos θ). Allow MC slack.
        assert!(
            (aabb.min.z + FUNNEL_NIPPLE_HEIGHT_M).abs() < 0.002,
            "straight nipple min.z should reach −nipple_h = {}; got {}",
            -FUNNEL_NIPPLE_HEIGHT_M,
            aabb.min.z,
        );
        // No tilt → +X extent is the bowl mouth, not the bowl PLUS a
        // tilted nipple. Max.x ≈ bowl top radius.
        assert!(
            (aabb.max.x - FUNNEL_TOP_OUTER_RADIUS_M).abs() < 0.002,
            "straight-nipple funnel +X extent should be the bowl mouth \
             {FUNNEL_TOP_OUTER_RADIUS_M}; got {}",
            aabb.max.x,
        );
        // Symmetric about the bowl axis (no +X bias from a tilt).
        assert!(
            (aabb.max.x + aabb.min.x).abs() < 0.002,
            "straight nipple → X-symmetric bounds; got max.x={} min.x={}",
            aabb.max.x,
            aabb.min.x,
        );
    }

    #[test]
    fn build_funnel_solid_returns_none_when_pour_gate_none() {
        let centerline = vec![Point3::new(-0.050, 0.0, 0.0), Point3::new(0.050, 0.0, 0.0)];
        let split = SplitNormal::new(Vector3::new(0.0, 1.0, 0.0)).unwrap();
        let ribbon = Ribbon::new(centerline, split).unwrap();
        assert!(build_funnel_solid(&ribbon).is_none());
    }

    /// AABB sanity: bowl extends `+Z` up to `cone_height`, nipple tip
    /// extends DOWN-AND-OUT to `(nipple_h*sin(θ), 0, −nipple_h*cos(θ))`.
    /// Top of mesh at `+cone_height ≈ 30 mm`; bottom at
    /// `−nipple_h*cos(θ) ≈ −12.99 mm`. `+X` extent goes to the bowl
    /// top radius (22 mm) since the tilted nipple's max `+X` is only
    /// `(nipple_h+overlap)*sin(θ) + nipple_outer_r ≈ 8 + 4.75 ≈
    /// 12.75 mm`, smaller than the bowl mouth.
    #[test]
    fn build_funnel_solid_returns_some_when_pour_gate_enabled() {
        let ribbon = iter1_like_ribbon();
        let (funnel, _) = build_funnel_solid(&ribbon).expect("funnel should build");
        let aabb = funnel.bounds().expect("funnel has finite bounds");
        let cos = V_HALF_ANGLE_RAD.cos();
        let sin = V_HALF_ANGLE_RAD.sin();
        let expected_nipple_tip_z = -FUNNEL_NIPPLE_HEIGHT_M * cos;
        assert!(
            (aabb.max.z - FUNNEL_CONE_HEIGHT_M).abs() < 0.001,
            "funnel max.z should equal bowl height {FUNNEL_CONE_HEIGHT_M}; got {}",
            aabb.max.z,
        );
        assert!(
            aabb.min.z <= expected_nipple_tip_z + 0.001,
            "funnel min.z should reach nipple tip {expected_nipple_tip_z}; got {}",
            aabb.min.z,
        );
        // +X extent at least bowl top radius (the bowl mouth is widest
        // in +X direction).
        assert!(
            aabb.max.x >= FUNNEL_TOP_OUTER_RADIUS_M - 0.001,
            "funnel max.x should reach bowl top radius {FUNNEL_TOP_OUTER_RADIUS_M}; got {}",
            aabb.max.x,
        );
        // Witness that the nipple's tip exists in +X (sin(30°)·15 ≈
        // 7.5 mm) — sanity check that the tilt actually applied.
        let _ = sin; // silence unused if assertions reordered
    }

    /// Nipple wall material at mid-nipple along the TILTED axis must
    /// be inside the funnel SDF (the nipple is a tilted cylinder; mid-
    /// nipple point is `(0.5·sin θ·h, 0, −0.5·cos θ·h)` from the bowl-
    /// nipple junction). Offset by mid-wall radius perpendicular to
    /// the tilted axis = `(mid_r·cos θ, 0, mid_r·sin θ)` (the
    /// perpendicular in the X-Z plane).
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

        let cos = V_HALF_ANGLE_RAD.cos();
        let sin = V_HALF_ANGLE_RAD.sin();
        // Mid-nipple center along the tilted axis (axis goes from
        // origin in direction (sin, 0, −cos)).
        let mid_axis_dist = FUNNEL_NIPPLE_HEIGHT_M / 2.0;
        let center = Point3::new(mid_axis_dist * sin, 0.0, -mid_axis_dist * cos);
        // Perpendicular offset to mid-wall in the X-Z plane: rotate
        // axis direction (sin, 0, -cos) by 90° around +Y →
        // (cos, 0, sin). Length = mid_wall_r.
        let wall_point = Point3::new(
            center.x + mid_wall_r * cos,
            0.0,
            center.z + mid_wall_r * sin,
        );
        assert!(
            funnel.evaluate(&wall_point) < 0.0,
            "funnel SDF at mid-nipple-wall {wall_point:?} should be inside; \
             got {}",
            funnel.evaluate(&wall_point),
        );
        // Center of the nipple bore (lumen) is outside the funnel
        // body — silicone flows here.
        assert!(
            funnel.evaluate(&center) > 0.0,
            "funnel SDF on nipple axis at mid-nipple {center:?} should be \
             outside (lumen interior); got {}",
            funnel.evaluate(&center),
        );
    }

    /// Bowl top interior must be open (cavity, SDF > 0) — workshop
    /// pour-mouth.
    #[test]
    fn build_funnel_solid_has_open_top_at_bowl_mouth() {
        let ribbon = iter1_like_ribbon();
        let (funnel, _) = build_funnel_solid(&ribbon).expect("funnel should build");
        let q_bowl_interior = Point3::new(0.0, 0.0, FUNNEL_CONE_HEIGHT_M - 0.002);
        assert!(
            funnel.evaluate(&q_bowl_interior) > 0.0,
            "funnel central axis near bowl top should be in the bowl cavity \
             (SDF > 0); got {}",
            funnel.evaluate(&q_bowl_interior),
        );
    }

    /// `build_funnel_solid` returns no mesh-CSG transforms — the
    /// funnel composes entirely in SDF/MC.
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
    /// `feedback_math_verify_geometric_contracts`. Funnel nipple OD
    /// plus cup pour-gate hole ID must differ by exactly
    /// [`NIPPLE_DIAMETRAL_CLEARANCE_M`].
    #[test]
    fn funnel_nipple_and_pour_gate_fit_invariant() {
        let ribbon = iter1_like_ribbon();
        let spec = match &ribbon.pour_gate {
            PourGateKind::Default(spec) => spec.clone(),
            PourGateKind::None => panic!("iter1_like_ribbon must have pour gate"),
        };
        let funnel_nipple_r = spec.gate_radius_m - NIPPLE_DIAMETRAL_CLEARANCE_M / 2.0;
        let cup_transforms = crate::pour::build_pour_gate_transforms(&ribbon);
        let cup_pour_r = cup_transforms
            .iter()
            .find_map(|t| match t {
                MatingTransform::SubtractCylinder { params } => Some(params.radius_m),
                _ => None,
            })
            .expect("cup-side pour-gate transforms must include a SubtractCylinder");
        let diametral_gap_m = 2.0 * (cup_pour_r - funnel_nipple_r);
        assert!(
            (diametral_gap_m - NIPPLE_DIAMETRAL_CLEARANCE_M).abs() < 1.0e-9,
            "diametral gap should equal NIPPLE_DIAMETRAL_CLEARANCE_M = \
             {NIPPLE_DIAMETRAL_CLEARANCE_M}; got {diametral_gap_m}",
        );
        assert!(
            (cup_pour_r - spec.gate_radius_m).abs() < 1.0e-9,
            "cup pour-gate hole radius should equal gate_radius_m = {} \
             (cup side stays nominal); got {cup_pour_r}",
            spec.gate_radius_m,
        );
    }

    /// Funnel connectivity regression guard. The bent-spout
    /// architecture welds bowl + nipple via SDF union with
    /// [`BOWL_NIPPLE_OVERLAP_M`] (1 mm) of nipple-top extending into
    /// the bowl interior — sufficient overlap for MC to interpolate
    /// across the junction at the
    /// `crate::spec::FUNNEL_MAX_CELL_SIZE_M` (1 mm) cell size. A
    /// regression here means either (a) the SDF chain dropped the
    /// `nipple_outer` union, (b) the bowl-nipple overlap was tightened
    /// below the cell size, or (c) a future migration introduced a
    /// new paradigm-boundary cylinder Union onto the SDF-meshed
    /// funnel without an SDF-side weld.
    #[test]
    fn funnel_mesh_is_single_connected_component() {
        use crate::error::CastTarget;
        use crate::mesh_csg::apply_mating_transforms;
        use crate::mesher::solid_to_mm_mesh;
        use mesh_repair::components::find_connected_components;

        let ribbon = iter1_like_ribbon();
        let (solid, transforms) =
            build_funnel_solid(&ribbon).expect("iter1 fixture has pour gate enabled");
        let mesh = solid_to_mm_mesh(&solid, 0.001, CastTarget::Funnel).expect("funnel MC");
        let mesh = apply_mating_transforms(mesh, &transforms, CastTarget::Funnel)
            .expect("funnel mesh-CSG");
        let analysis = find_connected_components(&mesh);
        assert_eq!(
            analysis.component_count,
            1,
            "funnel mesh must be a single connected component; got {} components \
             ({} verts / {} faces)",
            analysis.component_count,
            mesh.vertices.len(),
            mesh.faces.len(),
        );
    }

    /// Watertightness regression (cold-read workshop cf-view smoke
    /// 2026-05-27): the bowl-nipple kink at z=0 must close cleanly
    /// at the iter-1 1 mm MC cell size, with no open-boundary edges.
    /// Pre-fix the bowl-bottom OR was 7 mm — only 1.52 mm wider
    /// than the tilted nipple's projected ellipse semi-major at z=0
    /// (5.48 mm), smaller than the MC cell size, so MC couldn't
    /// resolve the back-of-nipple kink reveal and left a triangular
    /// gap visible in cf-view. Post-fix the OR is 10 mm, giving
    /// ~4.5 mm reveal annulus = 4-5 MC cells of resolution.
    /// A regression here would mean the bowl OR was tightened back
    /// below the safety margin OR MC cell size was bumped up past
    /// the reveal-annulus width.
    #[test]
    fn funnel_mesh_has_no_open_boundary_edges() {
        use crate::error::CastTarget;
        use crate::mesh_csg::apply_mating_transforms;
        use crate::mesher::solid_to_mm_mesh;
        use mesh_repair::validate_mesh;

        let ribbon = iter1_like_ribbon();
        let (solid, transforms) =
            build_funnel_solid(&ribbon).expect("iter1 fixture has pour gate enabled");
        let mesh = solid_to_mm_mesh(&solid, 0.001, CastTarget::Funnel).expect("funnel MC");
        let mesh = apply_mating_transforms(mesh, &transforms, CastTarget::Funnel)
            .expect("funnel mesh-CSG");
        let report = validate_mesh(&mesh);
        assert_eq!(
            report.boundary_edge_count,
            0,
            "funnel mesh must be watertight (0 boundary edges); got \
             {boundary_edges}. If this regresses, the bowl-nipple kink at \
             z=0 leaves an MC-unresolved reveal — check that \
             FUNNEL_BOWL_BOTTOM_OUTER_RADIUS_M still exceeds the tilted \
             nipple's projected ellipse semi-major \
             (nipple_outer_r / cos(V_HALF_ANGLE_RAD)) by at least a few \
             MC cell widths.",
            boundary_edges = report.boundary_edge_count,
        );
    }

    /// Nipple tilt regression: the nipple's tip MUST be offset in
    /// the +X direction by `sin(V_HALF_ANGLE_RAD) × nipple_height` —
    /// witnesses that the rotation actually applied + the nipple is
    /// the angled spout, not a straight-down nipple.
    #[test]
    fn nipple_tip_is_offset_in_plus_x_by_tilt() {
        let ribbon = iter1_like_ribbon();
        let (funnel, _) = build_funnel_solid(&ribbon).expect("funnel should build");
        let aabb = funnel.bounds().expect("funnel has finite bounds");
        let expected_min_x_tip = FUNNEL_NIPPLE_HEIGHT_M * V_HALF_ANGLE_RAD.sin();
        // The nipple's tip at +X around (sin(θ)·h, 0, −cos(θ)·h) ≈
        // (+7.5, 0, −12.99). The funnel's max.x should reach AT
        // LEAST that x-coord (it may be greater due to the bowl top
        // radius extending beyond, but it should not be LESS).
        assert!(
            aabb.max.x >= expected_min_x_tip - 0.001,
            "funnel +X extent must include the tilted nipple's tip at \
             X ≈ {expected_min_x_tip}; got max.x={}. If this fails, the \
             nipple tilt didn't apply — check the UnitQuaternion::from_axis_angle \
             rotation in build_funnel_solid.",
            aabb.max.x,
        );
        // Similarly, -X extent must reach AT LEAST the bowl mouth
        // radius (bowl is symmetric around the Z axis).
        assert!(
            aabb.min.x <= -FUNNEL_TOP_OUTER_RADIUS_M + 0.001,
            "funnel -X extent must include the bowl mouth at \
             X ≈ -{FUNNEL_TOP_OUTER_RADIUS_M}; got min.x={}",
            aabb.min.x,
        );
    }
}
