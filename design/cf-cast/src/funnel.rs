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
//!   `FUNNEL_NIPPLE_SLACK_M` (slides into the pour-leg hole),
//!   inner Ø = outer Ø − 2 × `FUNNEL_NIPPLE_WALL_M`. Self-aligns
//!   the funnel during pour.
//! - **Flange** above the nipple: cylinder outer Ø = 2 ×
//!   `FUNNEL_FLANGE_OUTER_RADIUS_M`. Rests flat on the cup wall
//!   outer surface around the pour-gate hole.
//! - **Cone** above the flange: truncated cone widening from
//!   flange Ø at the bottom to 2 × `FUNNEL_TOP_OUTER_RADIUS_M`
//!   at the top.
//! - **Hollow interior**: single continuous tapered cavity from
//!   the nipple's inner Ø at `z=0` up to (top Ø −
//!   2 × `FUNNEL_CONE_WALL_M`) at `z=top`.

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
/// than the pre-iter-1 6 mm pour-gate Ø (= the historical
/// bottleneck the bump-to-10 mm pour gate aimed to eliminate), and
/// the 4 mm nipple height keeps the bottleneck short.
pub const FUNNEL_NIPPLE_WALL_M: f64 = 0.0015;

/// Diametric slack between the nipple outer Ø and the pour-gate
/// hole Ø (m).
///
/// 0.2 mm — typical FDM hole-print tolerance (FDM holes typically
/// print 0.1-0.2 mm smaller than nominal); workshop reams to taste
/// if first print binds.
pub const FUNNEL_NIPPLE_SLACK_M: f64 = 0.0002;

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
/// surfaces ensures MC produces clean openings instead of
/// stair-step shelves at the cavity boundary.
const INNER_CAVITY_OVERSHOOT_M: f64 = 0.0005;

/// Build the workshop pour funnel [`Solid`] for the given ribbon,
/// or `None` if the ribbon has no pour gate enabled (no funnel
/// needed).
///
/// The funnel's nipple Ø is sized to slide into the ribbon's
/// pour-gate hole: nipple outer Ø = 2 × `pour_gate.gate_radius_m`
/// − [`FUNNEL_NIPPLE_SLACK_M`]. All other dimensions are workshop
/// defaults (see module-level constants).
///
/// Returns `None` when the ribbon's pour-gate kind is
/// [`crate::pour::PourGateKind::None`].
#[must_use]
pub fn build_funnel_solid(ribbon: &Ribbon) -> Option<(Solid, Vec<MatingTransform>)> {
    let spec = match &ribbon.pour_gate {
        PourGateKind::None => return None,
        PourGateKind::Default(spec) => spec,
    };
    let pour_gate_outer_diameter = spec.gate_radius_m * 2.0;
    let nipple_outer_diameter = pour_gate_outer_diameter - FUNNEL_NIPPLE_SLACK_M;
    let nipple_outer_r = nipple_outer_diameter / 2.0;
    let nipple_inner_r = nipple_outer_r - FUNNEL_NIPPLE_WALL_M;
    let cone_top_inner_r = FUNNEL_TOP_OUTER_RADIUS_M - FUNNEL_CONE_WALL_M;

    // Z-extents (axis along +Z, bottom at z=0).
    let nipple_z_top = FUNNEL_NIPPLE_HEIGHT_M;
    let flange_z_top = nipple_z_top + FUNNEL_FLANGE_HEIGHT_M;
    let cone_z_top = flange_z_top + FUNNEL_CONE_HEIGHT_M;

    // Outer body: nipple cylinder ∪ flange disk ∪ cone frustum.
    let nipple_outer = Solid::cylinder(nipple_outer_r, FUNNEL_NIPPLE_HEIGHT_M / 2.0)
        .translate(Vector3::new(0.0, 0.0, FUNNEL_NIPPLE_HEIGHT_M / 2.0));
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
    // cleanly at the funnel openings.
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

    // S3 plumbing: no mating transforms yet. S7 (sweep) may migrate
    // the nipple↔pour-gate fit to a mesh-CSG `SubtractCylinder` keyed
    // off a shared parent triple with the cup-piece pour-gate carve.
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

    #[test]
    fn build_funnel_solid_has_open_bottom_at_nipple() {
        // At z just above 0 (inside nipple), the funnel SDF on
        // the central axis should be > 0 (outside the funnel
        // material) — that's the inner cavity, where silicone
        // flows.
        let ribbon = iter1_like_ribbon();
        let (funnel, _) = build_funnel_solid(&ribbon).expect("funnel should build");
        let q_nipple_interior = Point3::new(0.0, 0.0, 0.002); // 2 mm above bottom
        assert!(
            funnel.evaluate(&q_nipple_interior) > 0.0,
            "funnel central axis at z=2mm should be in the inner cavity \
             (outside funnel material, SDF > 0); got {}",
            funnel.evaluate(&q_nipple_interior),
        );
    }

    #[test]
    fn build_funnel_solid_has_solid_nipple_wall() {
        // At z in the nipple region, just off-axis inside the wall
        // (between inner radius and outer radius), the funnel SDF
        // should be < 0 (solid wall material).
        let ribbon = iter1_like_ribbon();
        let (funnel, _) = build_funnel_solid(&ribbon).expect("funnel should build");
        let pour_radius = match &ribbon.pour_gate {
            PourGateKind::Default(spec) => spec.gate_radius_m,
            PourGateKind::None => panic!("iter1_like_ribbon must have pour gate"),
        };
        let nipple_outer_r = pour_radius - FUNNEL_NIPPLE_SLACK_M / 2.0;
        let mid_wall_r = nipple_outer_r - FUNNEL_NIPPLE_WALL_M / 2.0;
        let q_mid_wall = Point3::new(mid_wall_r, 0.0, 0.002); // 2 mm above bottom, mid-wall
        assert!(
            funnel.evaluate(&q_mid_wall) < 0.0,
            "funnel mid-wall at z=2mm should be inside the wall (SDF < 0); \
             got {}",
            funnel.evaluate(&q_mid_wall),
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

    #[test]
    fn build_funnel_solid_nipple_outer_matches_pour_gate_with_slack() {
        // Built funnel's nipple outer Ø should equal the ribbon's
        // pour-gate Ø minus the FDM slack. Probe the actual built
        // solid at points just inside / just outside the expected
        // nipple outer radius at a mid-nipple z, and verify the
        // SDF sign flips at the expected radius.
        let ribbon = iter1_like_ribbon();
        let pour_radius = match &ribbon.pour_gate {
            PourGateKind::Default(spec) => spec.gate_radius_m,
            PourGateKind::None => panic!("iter1_like_ribbon must have pour gate"),
        };
        let expected_nipple_outer_r = pour_radius - FUNNEL_NIPPLE_SLACK_M / 2.0;
        let (funnel, _) = build_funnel_solid(&ribbon).expect("funnel should build");
        let z_mid_nipple = FUNNEL_NIPPLE_HEIGHT_M / 2.0;
        // Just INSIDE the expected outer radius → INSIDE funnel
        // wall (SDF < 0) provided we're also outside the inner
        // cavity. Probe at a radius midway between inner + outer.
        let r_mid_wall = expected_nipple_outer_r - FUNNEL_NIPPLE_WALL_M / 2.0;
        let q_inside = nalgebra::Point3::new(r_mid_wall, 0.0, z_mid_nipple);
        assert!(
            funnel.evaluate(&q_inside) < 0.0,
            "funnel mid-nipple-wall at r={r_mid_wall} (just inside outer Ø) \
             should be inside funnel material (SDF < 0); got {}",
            funnel.evaluate(&q_inside),
        );
        // Just OUTSIDE the expected outer radius → OUTSIDE the
        // funnel (SDF > 0).
        let r_just_outside = expected_nipple_outer_r + 0.0005; // +0.5 mm beyond outer
        let q_outside = nalgebra::Point3::new(r_just_outside, 0.0, z_mid_nipple);
        assert!(
            funnel.evaluate(&q_outside) > 0.0,
            "funnel just-outside-nipple at r={r_just_outside} should be \
             outside funnel material (SDF > 0); got {}",
            funnel.evaluate(&q_outside),
        );
    }
}
