//! M5 through-bolt clamp pattern (§B of
//! `docs/CF_CAST_FLANGE_CONTINUITY_BOLT_PATTERN_RECON.md`).
//!
//! Both cup-halves get IDENTICAL bolt-clearance holes carved through
//! their mating faces in the flange band, placed by the
//! seam-placement solver (even pitch ≤30 mm, bracketing the pour).
//! Workshop user inserts an M5 hex
//! bolt + flat washer through one back face, threads a flat washer +
//! hex nut on the other side, and hand-tightens to compress the
//! gasket. Mirrors the engine-valve-cover clamp pattern; replaces the
//! pre-§B hand-clamped-with-rubber-bands assembly.
//!
//! Composes on top of the §M unified-mating-plane arc: the symmetric
//! [`crate::dowel_hole`] pattern registers the two halves laterally,
//! while the bolt pattern clamps them axially across the gasket.
//!
//! # Geometry
//!
//! Each bolt hole is a [`MatingTransform::SubtractCylinder`] post-MC
//! mesh-CSG primitive with:
//! - **axis** = ribbon binormal (perpendicular to mating plane —
//!   parallel to print Z when the cup piece is mated-face-down on the
//!   build plate, so the hole prints as a vertical cylinder requiring
//!   no support).
//! - **center** = a seam-plane point resolved by the constraint-aware
//!   seam-placement solver ([`plan_smart_bolt_placements`], the default
//!   since S5 of `docs/CF_CAST_SEAM_PLACEMENT_RECON.md`) and lifted onto
//!   the binormal-aligned seam plane through the ribbon midpoint (same
//!   projection §M-S2 dowel-hole uses). The count emerges from `max_pitch`
//!   and the radial offset is solved per bolt — no fixed count / offset.
//! - **`half_length`** = `flange_thickness_m + BOLT_AXIAL_SLACK_M`.
//!   The full cylinder spans `2 × half_length` straddling the seam
//!   plane, carving a through-hole through BOTH cup-halves' flange
//!   material. The +slack ensures the cylindrical caps are well
//!   OUTSIDE the flange's back face on each side (no mesh-CSG
//!   operating on coincident endcap planes).
//! - **radius** = `clearance_diameter_m / 2`. Workshop user supplies
//!   the M5 bolt + washer; the washer compensates for FDM dimensional
//!   error on the hole, so no extra radial clearance slack is added.
//!
//! # Bolt + dowel placement, pour bracketing
//!
//! Positions come from the constraint-aware seam-placement solver, not a
//! fixed-count arc-length loop: dowels are solved first (registration
//! extremes), then bolts fill around them excluding their footprints, and
//! the pour bore is excluded as a swept channel so its interval edges
//! bracket the apex by construction (§3.4/§3.6 of
//! `docs/CF_CAST_SEAM_PLACEMENT_RECON.md`). See
//! [`plan_smart_bolt_placements`] + [`crate::seam_placement`].
//!
//! # Side-agnostic emission
//!
//! Identical transform list applied to BOTH Negative + Positive cup
//! pieces (mesh-CSG subtract is invariant to which half it's applied
//! to; cylinder portion outside a given half is a no-op). Both
//! halves get matching bolt-hole positions mirrored across the seam
//! plane, so a single M5 bolt passes through the corresponding holes
//! on the two halves to clamp them together.

use nalgebra::Unit;

use crate::flange::FlangeSpec;
use crate::mesh_csg::{CylinderParams, CylinderParent, MatingTransform};
use crate::ribbon::Ribbon;
use crate::seam_placement::{cross_layer_snap, smart_center_to_world};
use crate::seam_profile::SeamProfile;
use crate::seam_solver::{
    DEFAULT_MAX_PITCH_M, Exclusion, FastenerClass, Feasibility, Seed, SeedKind, place_fasteners,
};
use crate::silhouette_2d::Point2;

/// Default bolt clearance hole diameter (5.5 mm) — standard M5
/// clearance per ISO 273 medium-fit.
const DEFAULT_CLEARANCE_DIAMETER_M: f64 = 0.0055;

/// Polygonal facets around each bolt cylinder. 32 segments give
/// ~0.4 mm chord error at 5.5 mm Ø, well below FDM print resolution
/// and matching the segment count of the dowel-HOLE
/// `SubtractCylinder` primitives so both surfaces have uniform
/// chord error.
const DEFAULT_SEGMENTS: u32 = 32;

/// Extra axial slack on each side of the hole's nominal half-length
/// (= `flange_thickness_m`).
///
/// Ensures the cylinder's CYLINDRICAL flat caps are well OUTSIDE the
/// flange's back face on each side, avoiding mesh-CSG operating on
/// coincident endcap planes at the flange surface. 0.5 mm is well
/// below FDM resolution + safely past any back-face MC ambiguity.
/// Public so `crate::procedure` can quote the bolt-traverse length
/// accurately in workshop-facing markdown.
pub const BOLT_AXIAL_SLACK_M: f64 = 0.0005;

/// M5 bolt-pattern geometry parameter envelope per §B-S1 recon.
///
/// Workshop user picks via cf-cast-cli `[bolt_pattern]` TOML block;
/// iter-1 starts at the defaults below.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BoltPatternSpec {
    /// Bolt clearance hole diameter (meters). Default 5.5 mm (M5
    /// ISO 273 medium fit).
    pub clearance_diameter_m: f64,
}

impl BoltPatternSpec {
    /// Workshop iter-1 starting defaults pinned at §B-S1 recon.
    ///
    /// Bolt *count* and radial offset are no longer spec fields: the
    /// seam-placement solver derives them per layer (even spacing at
    /// ≤30 mm pitch, variable offset from the flange band + washer
    /// clearance). See `docs/CF_CAST_SEAM_PLACEMENT_RECON.md` §7.5.
    #[must_use]
    pub const fn iter1() -> Self {
        Self {
            clearance_diameter_m: DEFAULT_CLEARANCE_DIAMETER_M,
        }
    }
}

impl Default for BoltPatternSpec {
    fn default() -> Self {
        Self::iter1()
    }
}

/// Bolt-pattern kind on the [`Ribbon`].
///
/// Matches the existing `DowelHoleKind` / `FlangeKind` / `GasketKind`
/// / `PourGateKind` / `PlugPinKind` patterns — bridges the
/// cf-cast-cli `[bolt_pattern]` TOML block to the per-piece bolt-hole
/// emission in [`crate::compose_piece_solid`].
///
/// Default [`Self::None`] (no bolt holes — cup pieces clamped by hand
/// or by external method). Set
/// `BoltPatternKind::Auto(BoltPatternSpec::iter1())` via
/// [`Ribbon::with_bolt_pattern`] to enable the M5 through-bolt
/// clamp pattern.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BoltPatternKind {
    /// No bolt-hole emission.
    None,
    /// Arc-length-equal-spaced M5 bolt holes around the silhouette,
    /// parameterized by the inner [`BoltPatternSpec`].
    Auto(BoltPatternSpec),
}

impl BoltPatternKind {
    /// Returns the inner [`BoltPatternSpec`] for [`BoltPatternKind::Auto`],
    /// `None` otherwise. Convenience accessor for the export pipeline.
    #[must_use]
    pub const fn spec(&self) -> Option<&BoltPatternSpec> {
        match self {
            Self::None => None,
            Self::Auto(spec) => Some(spec),
        }
    }
}

/// Build the side-agnostic bolt-pattern [`MatingTransform`] vec from the
/// seam-placement solver's resolved per-layer bolt `centers` (seam-plane points;
/// see [`plan_smart_bolt_placements`]).
///
/// Each center maps straight to a [`MatingTransform::SubtractCylinder`] carving
/// through both cup-halves' flange material — the cylinder (axis = ribbon
/// binormal, half-length = `flange_thickness + slack`, radius = clearance Ø/2) is
/// identical for both [`crate::ribbon::PieceSide`]s (mesh-CSG subtract is
/// side-invariant). An empty `centers` slice (no flange, or the solve placed
/// nothing) yields no holes.
#[must_use]
pub fn build_bolt_pattern_transforms(
    centers: &[Point2],
    ribbon: &Ribbon,
    spec: &BoltPatternSpec,
    flange_spec: &FlangeSpec,
) -> Vec<MatingTransform> {
    let (seam_midpoint, seam_normal) = ribbon.seam_plane_reference();
    let binormal = seam_normal.into_inner();
    let axis = Unit::new_normalize(binormal);
    let radius_m = spec.clearance_diameter_m / 2.0;
    let half_length_m = flange_spec.flange_thickness_m + BOLT_AXIAL_SLACK_M;
    centers
        .iter()
        .map(|&c| {
            let center_m = smart_center_to_world(ribbon, seam_midpoint, binormal, c);
            MatingTransform::SubtractCylinder {
                params: CylinderParams {
                    parent: CylinderParent {
                        center_m,
                        axis,
                        half_length_m,
                    },
                    radius_m,
                    segments: DEFAULT_SEGMENTS,
                },
            }
        })
        .collect()
}

// === Smart placement (S3, `docs/CF_CAST_SEAM_PLACEMENT_RECON.md`) ===

/// M5 flat-washer footprint radius (5 mm — OD 10 mm). This is the bolt's
/// *footprint* for feasibility (the washer seats on the flange back face), as
/// distinct from the `clearance_diameter_m / 2` HOLE radius that gets carved.
/// The seam solver keeps the whole washer disk inside the flange band + clear of
/// the pour/dowels (§3.3); the emitted cylinder is still the clearance hole.
const BOLT_WASHER_RADIUS_M: f64 = 0.005;

/// Safety margin the washer keeps beyond the cup-wall outer step (1 mm, ≈ FDM
/// extrusion tolerance). The inboard radial floor is `wall_thickness + washer +
/// this` — the *computed* form of the §3.2 "inboard washer-vs-cup-wall-step
/// clearance" (the surviving half of the legacy hand-set 13 mm offset; the old
/// 3 mm margin shrinks to FDM tolerance because the solver's max-margin objective
/// keeps bolts comfortably outboard in free band regions, so the hard floor only
/// has to guarantee the washer doesn't overhang the step). Matches the S0
/// finding that a feasible apex bolt sits at d ≈ 11.5 mm (which a 13 mm floor
/// would have excluded).
const WASHER_CUP_WALL_MARGIN_M: f64 = 0.001;

/// The flange-band feasibility regime for the bolt washer (§3.3): the washer
/// disk (radius [`BOLT_WASHER_RADIUS_M`]) must lie in the flange band, the radial
/// offset floored by the washer-vs-cup-wall-step clearance (§3.2).
fn bolt_feasibility(flange_spec: &FlangeSpec, wall_thickness_m: f64) -> Feasibility {
    let d_floor = wall_thickness_m + BOLT_WASHER_RADIUS_M + WASHER_CUP_WALL_MARGIN_M;
    Feasibility::band(
        flange_spec.flange_inner_offset_m,
        flange_spec.flange_width_m,
        d_floor,
        flange_spec.flange_width_m,
    )
}

/// The arc length where a pour channel pierces the loop — the loop point nearest
/// the (in-plane) bore axis segment. Sampled along the segment by minimum
/// `|signed_distance|` (the crossing sits on the loop, sd ≈ 0), then snapped to
/// its arc. Drives the [`SeedKind::PourPierce`] seed (§3.4).
fn pour_pierce_arc(profile: &SeamProfile, seg_a: Point2, seg_b: Point2) -> f64 {
    const N: u32 = 64;
    let mut best = (f64::INFINITY, seg_a);
    for k in 0..=N {
        let frac = f64::from(k) / f64::from(N);
        let pt = Point2::new(
            seg_a.x + (seg_b.x - seg_a.x) * frac,
            seg_a.z + (seg_b.z - seg_a.z) * frac,
        );
        let dist = profile.signed_distance(pt).abs();
        if dist < best.0 {
            best = (dist, pt);
        }
    }
    profile.nearest_arc(best.1)
}

/// Per-layer seam-plane bolt centers from the seam-placement solver (§3.5/§3.8).
///
/// The solver runs **once** on the outermost layer's loop (the largest
/// silhouette): the count emerges from `max_pitch`, the pour bore is excluded as
/// a swept channel (so its interval edges bracket it — the apex clamp, §3.4) plus
/// a complementary `PourPierce` seed, and every dowel footprint is excluded
/// (§3.6). Each resolved center is then snapped onto **every** layer's loop
/// (`cross_layer_snap`, §3.8); a position kept only if it is feasible on all
/// layers, so the stack shares one angular pattern (the workshop aligns one
/// jig). Returns a `Vec` parallel to `layers` (innermost-first) of the
/// per-layer centers; positions infeasible on some layer are dropped with a
/// `warn`.
///
/// `layers` are the per-layer seam loops + pour-channel exclusions built ONCE by
/// `seam_placement::build_layer_loops` and shared with the dowel planner
/// (S5d-(A) — the silhouette flood-fill is the dominant cost, so it is not rebuilt
/// per planner).
///
/// `smart_dowels` carries the seam-placement-solver dowel centers (S4), per layer
/// and parallel to `layers` — those footprints (radius `dowel_footprint_r` =
/// `smart_dowel_footprint`, hole + wall) are appended to each layer's exclusions
/// so the bolt run clears them (the dowels-first contract, §3.6). `None` (no dowel
/// pattern) excludes only the pour, keeping the washer's PLA wall around each
/// dowel hole.
#[must_use]
pub fn plan_smart_bolt_placements(
    layers: &[crate::seam_placement::LayerLoop],
    bolt_spec: &BoltPatternSpec,
    flange_spec: &FlangeSpec,
    wall_thickness_m: f64,
    dowel_footprint_r: Option<f64>,
    smart_dowels: Option<&[Vec<Point2>]>,
) -> Vec<Vec<Point2>> {
    let n_layers = layers.len();
    let result = vec![Vec::new(); n_layers];
    if n_layers == 0 {
        return result;
    }

    let feas = bolt_feasibility(flange_spec, wall_thickness_m);
    let washer_r = BOLT_WASHER_RADIUS_M;

    // The shared per-layer loops (`build_layer_loops`, S5d-(A)) carry the clean
    // seam profile + pour-channel exclusions. The bolt run additionally excludes
    // every solver-placed dowel disk for THIS layer (dowels run first — §3.6), so
    // append those to a clone of the shared pour-only exclusion set. The dowel
    // footprint radius (`smart_dowel_footprint`, hole + wall) is passed in so the
    // washer keeps the same PLA wall the dowel planner sized.
    let layers: Vec<crate::seam_placement::LayerLoop> = layers
        .iter()
        .enumerate()
        .map(|(layer_index, slot)| {
            slot.as_ref().map(|(profile, pour_excluded)| {
                let mut excluded = pour_excluded.clone();
                if let (Some(radius), Some(centers)) = (
                    dowel_footprint_r,
                    smart_dowels.and_then(|d| d.get(layer_index)),
                ) {
                    excluded.extend(
                        centers
                            .iter()
                            .map(|&center| Exclusion::Disk { center, radius }),
                    );
                }
                (profile.clone(), excluded)
            })
        })
        .collect();

    let Some(Some((outer_profile, outer_excluded))) = layers.last() else {
        eprintln!(
            "[cf-cast] WARNING: smart bolt placement — outermost layer silhouette is empty; \
             no bolts placed."
        );
        return result;
    };

    // Pour-pierce seeds on the outer loop (complementary to the channel's
    // interval-boundary brackets — §3.4 / S2 decision; dedup collapses overlap).
    let seeds: Vec<Seed> = outer_excluded
        .iter()
        .filter_map(|e| match *e {
            Exclusion::Channel { a, b, .. } => Some(Seed {
                arc: pour_pierce_arc(outer_profile, a, b),
                kind: SeedKind::PourPierce,
            }),
            Exclusion::Disk { .. } => None,
        })
        .collect();

    let class = FastenerClass {
        footprint_radius: washer_r,
        fill: Some(max_pitch(*bolt_spec)),
        seeds,
    };
    let master = place_fasteners(outer_profile, &feas, outer_excluded, &class);

    // Cross-layer snap + validate: keep a position only if feasible on EVERY
    // layer, so the stack shares one angular pattern (§3.8).
    cross_layer_snap(&master, &layers, &feas, washer_r, "bolt")
}

/// The even-contact bolt pitch for the smart placement solve. iter-1 uses the
/// workmanship default ([`DEFAULT_MAX_PITCH_M`], 30 mm); a `[bolt_pattern]`
/// override lands here in a later phase (§3.7).
const fn max_pitch(_spec: BoltPatternSpec) -> f64 {
    DEFAULT_MAX_PITCH_M
}

#[cfg(test)]
mod tests {
    #![allow(
        clippy::unwrap_used,
        clippy::panic,
        clippy::expect_used,
        clippy::float_cmp,
        // The `plan_bolts` test adapter mirrors the production planner's
        // `&BoltPatternSpec` API (now an 8-byte Copy type); the production fn is
        // exempt as exported API, the private helper would otherwise be flagged.
        clippy::trivially_copy_pass_by_ref
    )]

    use super::*;
    use crate::dowel_hole::{DowelHoleKind, DowelHoleSpec};
    use crate::flange::FlangeKind;
    use crate::pour::{PourGateKind, PourGateLayout, PourGateSpec, build_pour_gate_transforms};
    use crate::ribbon::SplitNormal;
    use crate::seam_placement::{build_layer_loops, seam_silhouette};
    use crate::silhouette_2d::SILHOUETTE_GRID_STEP_M;
    use cf_design::Solid;
    use nalgebra::{Point3, Vector3};

    /// Test adapter: build the shared per-layer loops (S5d-(A)) the way the v2
    /// pipeline does, then run the bolt planner — keeps the test call sites on the
    /// pre-S5d-(A) arg shape after loop construction moved out of the planner.
    fn plan_bolts(
        bodies: &[&Solid],
        bounds: &[cf_design::Aabb],
        ribbon: &Ribbon,
        bolt_spec: &BoltPatternSpec,
        flange: &FlangeSpec,
        wall: f64,
        smart_dowels: Option<&[Vec<Point2>]>,
    ) -> Vec<Vec<Point2>> {
        let loops = build_layer_loops(bodies, bounds, ribbon, flange);
        let dowel_r = ribbon
            .dowel_hole
            .spec()
            .map(crate::dowel_hole::smart_dowel_footprint);
        plan_smart_bolt_placements(&loops, bolt_spec, flange, wall, dowel_r, smart_dowels)
    }

    fn cylinder_fixture() -> (Solid, cf_design::Aabb, Ribbon) {
        // Cylinder along X, R=10 mm, length 60 mm.
        let body = Solid::cylinder(0.010, 0.030).rotate(nalgebra::UnitQuaternion::from_axis_angle(
            &Vector3::y_axis(),
            std::f64::consts::FRAC_PI_2,
        ));
        let bounding_region = Solid::cuboid(Vector3::new(0.100, 0.030, 0.030));
        let bounds = bounding_region.bounds().unwrap();
        let centerline = vec![Point3::new(-0.050, 0.0, 0.0), Point3::new(0.050, 0.0, 0.0)];
        let split = SplitNormal::new(Vector3::new(0.0, 0.0, 1.0)).unwrap();
        let ribbon = Ribbon::new(centerline, split).unwrap();
        (body, bounds, ribbon)
    }

    // Test-only geometry helpers. These were module fns under the legacy
    // pour-collision path (now deleted); the smart tests below still use them to
    // check washer-vs-pour clearance + to unit-test the finite-cylinder distance.

    /// Distance from `p` to the FINITE cylinder axis segment `c ± half_length·axis`
    /// (axis-projection clamped to the endcaps).
    fn point_to_cylinder_axis_distance(
        p: Point3<f64>,
        c: Point3<f64>,
        axis: Vector3<f64>,
        half_length: f64,
    ) -> f64 {
        let to_p = p - c;
        let t = to_p.dot(&axis).clamp(-half_length, half_length);
        let closest = c + axis * t;
        (p - closest).norm()
    }

    /// Distance from `p` to the nearest pour-gate cylinder axis, or `f64::MAX`.
    fn pour_axis_distance(p: Point3<f64>, pour_xforms: &[MatingTransform]) -> f64 {
        pour_xforms
            .iter()
            .filter_map(|t| match t {
                MatingTransform::SubtractCylinder { params } => {
                    Some(point_to_cylinder_axis_distance(
                        p,
                        params.parent.center_m,
                        params.parent.axis.into_inner(),
                        params.parent.half_length_m,
                    ))
                }
                _ => None,
            })
            .fold(f64::MAX, f64::min)
    }

    /// Largest pour-gate cylinder radius (0 if none).
    fn max_pour_radius(pour_xforms: &[MatingTransform]) -> f64 {
        pour_xforms
            .iter()
            .filter_map(|t| match t {
                MatingTransform::SubtractCylinder { params } => Some(params.radius_m),
                _ => None,
            })
            .fold(0.0_f64, f64::max)
    }

    /// Build the seam profile for a fixture body the way the planner does, so a
    /// test can re-check feasibility of each emitted center.
    fn fixture_profile(body: &Solid, ribbon: &Ribbon, bounds: cf_design::Aabb) -> SeamProfile {
        let pad = FlangeSpec::iter1().flange_width_m + SILHOUETTE_GRID_STEP_M;
        let sil = seam_silhouette(body, ribbon, bounds, pad);
        SeamProfile::from_silhouette(&sil).expect("fixture silhouette must form a loop")
    }

    #[test]
    fn bolt_pattern_spec_iter1_defaults() {
        let s = BoltPatternSpec::iter1();
        assert_eq!(s.clearance_diameter_m, 0.0055);
    }

    #[test]
    fn bolt_pattern_kind_spec_accessor() {
        assert!(BoltPatternKind::None.spec().is_none());
        let s = BoltPatternSpec::iter1();
        let kind = BoltPatternKind::Auto(s);
        assert_eq!(kind.spec(), Some(&s));
    }

    #[test]
    fn build_transforms_empty_centers_emits_empty_vec() {
        let (_body, _bounds, ribbon) = cylinder_fixture();
        let spec = BoltPatternSpec::iter1();
        let flange = FlangeSpec::iter1();
        let transforms = build_bolt_pattern_transforms(&[], &ribbon, &spec, &flange);
        assert!(transforms.is_empty());
    }

    /// The smart builder emits one cylinder per planned center, each with the bolt
    /// clearance radius, the flange-thickness half-length, and the binormal axis.
    #[test]
    fn smart_bolt_emission_matches_cylinder_geometry() {
        let (_body, _bounds, ribbon) = cylinder_fixture();
        let spec = BoltPatternSpec::iter1();
        let flange = FlangeSpec::iter1();
        // Synthetic seam-plane centers in the flange band.
        let centers = [
            Point2::new(0.0, 0.013),
            Point2::new(0.013, 0.0),
            Point2::new(-0.013, 0.0),
        ];
        let xforms = build_bolt_pattern_transforms(&centers, &ribbon, &spec, &flange);
        assert_eq!(xforms.len(), centers.len(), "one cylinder per center");

        let (_, seam_normal) = ribbon.seam_plane_reference();
        let binormal = seam_normal.into_inner();
        let expected_radius = spec.clearance_diameter_m / 2.0;
        let expected_half = flange.flange_thickness_m + BOLT_AXIAL_SLACK_M;
        for t in &xforms {
            let MatingTransform::SubtractCylinder { params } = t else {
                panic!("smart bolt emission must be SubtractCylinder; got {t:?}");
            };
            assert!((params.radius_m - expected_radius).abs() < 1e-12);
            assert!((params.parent.half_length_m - expected_half).abs() < 1e-12);
            let dot = params.parent.axis.into_inner().dot(&binormal).abs();
            assert!(
                (dot - 1.0).abs() < 1e-9,
                "bolt axis must align with ribbon binormal; got dot = {dot}"
            );
        }
    }

    /// Same regression discipline §M-S2 dowel-hole got post cold-read:
    /// bolt cylinder centers must land ON the binormal-aligned seam
    /// plane regardless of centerline tilt. Pre-fix the math put
    /// `center_m.y = seam_midpoint.y` (constant-Y plane), which for
    /// tilted centerlines placed the center off the actual seam plane
    /// along the binormal direction — making the bolt carve
    /// asymmetrically between the two cup-halves.
    /// Regression (§M-S2 discipline): smart bolt centers land ON the
    /// binormal-aligned seam plane regardless of centerline tilt. Pre-fix the math
    /// put `center_m.y = seam_midpoint.y` (constant-Y plane), which for tilted
    /// centerlines placed the center off the seam plane, carving asymmetrically.
    #[test]
    fn bolt_centers_lie_on_binormal_aligned_seam_plane_under_tilted_centerline() {
        let centerline = vec![
            Point3::new(-0.050, -0.005, 0.0),
            Point3::new(0.050, 0.005, 0.0),
        ];
        let split = SplitNormal::new(Vector3::new(0.0, 0.0, 1.0)).unwrap();
        let ribbon = Ribbon::new(centerline, split).unwrap();
        let spec = BoltPatternSpec::iter1();
        let flange = FlangeSpec::iter1();
        let centers = [Point2::new(0.0, 0.013), Point2::new(0.013, 0.0)];
        let transforms = build_bolt_pattern_transforms(&centers, &ribbon, &spec, &flange);
        assert_eq!(transforms.len(), centers.len());

        let (seam_midpoint, seam_normal) = ribbon.seam_plane_reference();
        let binormal = seam_normal.into_inner();
        for (i, t) in transforms.iter().enumerate() {
            let MatingTransform::SubtractCylinder { params } = t else {
                continue;
            };
            let dist_from_seam = (params.parent.center_m - seam_midpoint).dot(&binormal);
            assert!(
                dist_from_seam.abs() < 1e-9,
                "bolt #{i} center must lie ON the binormal-aligned seam plane; \
                 got binormal-distance = {dist_from_seam:.6} m"
            );
        }
    }

    #[test]
    fn point_to_cylinder_distance_zero_on_axis_inside_caps() {
        let c = Point3::new(0.0, 0.0, 0.0);
        let axis = Vector3::new(1.0, 0.0, 0.0);
        let half_length = 0.020;
        // Point ON the axis, INSIDE the cap span (|t| = 0.010 ≤ 0.020).
        let p = Point3::new(0.010, 0.0, 0.0);
        let d = point_to_cylinder_axis_distance(p, c, axis, half_length);
        assert!(
            d.abs() < 1e-12,
            "on-axis interior distance must be 0; got {d}"
        );
    }

    #[test]
    fn point_to_cylinder_distance_perpendicular_inside_caps() {
        let c = Point3::new(0.0, 0.0, 0.0);
        let axis = Vector3::new(1.0, 0.0, 0.0);
        let half_length = 0.020;
        // Point 3 mm off-axis, axis-projection inside caps.
        let p = Point3::new(0.005, 0.003, 0.0);
        let d = point_to_cylinder_axis_distance(p, c, axis, half_length);
        assert!(
            (d - 0.003).abs() < 1e-12,
            "perpendicular offset 3 mm inside caps; got {d}"
        );
    }

    /// Cold-read finding I1: a point BEYOND the cylinder's endcap
    /// must measure to the cap center, not to the infinite axis line.
    /// Pre-fix (infinite-line distance) returned 0.003 for this query;
    /// post-fix returns sqrt(0.010² + 0.003²) ≈ 0.0104.
    #[test]
    fn point_to_cylinder_distance_past_endcap_measures_to_cap_center() {
        let c = Point3::new(0.0, 0.0, 0.0);
        let axis = Vector3::new(1.0, 0.0, 0.0);
        let half_length = 0.020;
        // Point past the +X cap (axis-projection = 0.030 > 0.020),
        // 3 mm off-axis. Closest axis point is the +X cap center
        // (0.020, 0, 0). Distance = sqrt(0.010² + 0.003²) ≈ 0.0104 m.
        let p = Point3::new(0.030, 0.003, 0.0);
        let d = point_to_cylinder_axis_distance(p, c, axis, half_length);
        let expected = 0.010_f64.hypot(0.003_f64);
        assert!(
            (d - expected).abs() < 1e-12,
            "past-endcap distance must measure to cap center ({expected:.5}); \
             got {d:.5} (pre-fix infinite-line returned the perpendicular \
             ~0.003 — would falsely trigger pour-gate collision skips for \
             bolts well past the pour-gate leg's endpoint)",
        );
    }

    /// Fitted-seam (organic-parts) path: when the ribbon carries an apex-anchored
    /// seam plane (`with_planar_seam_at`), bolts must be built through the
    /// generalized `from_body_in_plane` silhouette and projected onto THAT plane —
    /// not the legacy Y-normal one. Invariants: (a) the builder still emits bolts,
    /// and (b) every emitted bolt center lies on the fitted seam plane.
    #[test]
    fn fitted_seam_bolts_lie_on_the_fitted_seam_plane() {
        let (body, bounds, base) = cylinder_fixture();
        let anchor = Point3::new(0.0, 0.0, 0.0);
        let normal = Vector3::new(0.3, 0.0, 1.0); // a diagonal (tilted) seam
        let ribbon = base
            .with_planar_seam_at(anchor, normal)
            .with_flange(FlangeKind::Plate(FlangeSpec::iter1()))
            .with_bolt_pattern(BoltPatternKind::Auto(BoltPatternSpec::iter1()));
        let n = normal.normalize();

        let spec = BoltPatternSpec::iter1();
        let flange = FlangeSpec::iter1();
        let plan = plan_bolts(&[&body], &[bounds], &ribbon, &spec, &flange, 0.005, None);
        let xforms = build_bolt_pattern_transforms(&plan[0], &ribbon, &spec, &flange);

        assert!(
            !xforms.is_empty(),
            "fitted-seam builder must still emit bolts",
        );
        for t in &xforms {
            let MatingTransform::SubtractCylinder { params } = t else {
                continue;
            };
            let d = (params.parent.center_m - anchor).dot(&n);
            assert!(
                d.abs() < 1.0e-6,
                "fitted-seam bolt must lie on the seam plane; off by {d:.6} m",
            );
        }
    }

    // === Smart placement (S3) ===

    /// The smart path emits one bolt per planned center, each clear of the body,
    /// inside the flange band, and clear of the apex pour bore by the washer.
    #[test]
    fn smart_placement_bolts_clear_body_band_and_pour() {
        let (body, bounds, base) = cylinder_fixture();
        let mut pour = PourGateSpec::iter1();
        pour.layout = PourGateLayout::ApexAxial;
        let ribbon = base
            .with_pour_gate(PourGateKind::Default(pour))
            .with_flange(FlangeKind::Plate(FlangeSpec::iter1()))
            .with_dowel_hole(DowelHoleKind::Auto(DowelHoleSpec::iter1()))
            .with_bolt_pattern(BoltPatternKind::Auto(BoltPatternSpec::iter1()));
        let bolt = BoltPatternSpec::iter1();
        let flange = FlangeSpec::iter1();
        let wall = 0.005;

        let plan = plan_bolts(&[&body], &[bounds], &ribbon, &bolt, &flange, wall, None);
        assert_eq!(plan.len(), 1, "one plan entry per layer");
        let centers = &plan[0];
        assert!(!centers.is_empty(), "smart placement produced no bolts");

        let xforms = build_bolt_pattern_transforms(centers, &ribbon, &bolt, &flange);
        assert_eq!(
            xforms.len(),
            centers.len(),
            "one SubtractCylinder per planned center"
        );

        let profile = fixture_profile(&body, &ribbon, bounds);
        let pour_xforms = build_pour_gate_transforms(&ribbon);
        for t in &xforms {
            let MatingTransform::SubtractCylinder { params } = t else {
                panic!("smart emission must be SubtractCylinder");
            };
            // hole radius is the clearance hole, not the washer footprint.
            assert!((params.radius_m - bolt.clearance_diameter_m / 2.0).abs() < 1e-12);
            // outside the body in 3D.
            assert!(
                body.evaluate(&params.parent.center_m) > 0.0,
                "smart bolt center inside the body"
            );
            // washer footprint (5 mm) inside the flange band.
            let sd = profile.signed_distance(profile.basis().project(params.parent.center_m));
            assert!(
                sd >= flange.flange_inner_offset_m + BOLT_WASHER_RADIUS_M - 5e-4
                    && sd <= flange.flange_width_m - BOLT_WASHER_RADIUS_M + 5e-4,
                "smart bolt washer out of the flange band: sd={sd}"
            );
            // washer clears the pour bore (the original washer-vs-sprue failure).
            assert!(
                pour_axis_distance(params.parent.center_m, &pour_xforms)
                    >= max_pour_radius(&pour_xforms) + BOLT_WASHER_RADIUS_M - 5e-4,
                "smart bolt washer fouls the pour bore"
            );
        }
    }

    /// The apex pour is bracketed by construction: ≥2 bolts straddle the bore
    /// (the channel exclusion's interval edges + the pierce seed), one each side.
    #[test]
    fn smart_placement_brackets_the_apex_pour_symmetrically() {
        let (body, bounds, base) = cylinder_fixture();
        let mut pour = PourGateSpec::iter1();
        pour.layout = PourGateLayout::ApexAxial;
        let ribbon = base
            .with_pour_gate(PourGateKind::Default(pour))
            .with_flange(FlangeKind::Plate(FlangeSpec::iter1()))
            .with_bolt_pattern(BoltPatternKind::Auto(BoltPatternSpec::iter1()));
        let bolt = BoltPatternSpec::iter1();
        let flange = FlangeSpec::iter1();

        let plan = plan_bolts(&[&body], &[bounds], &ribbon, &bolt, &flange, 0.005, None);
        let xforms = build_bolt_pattern_transforms(&plan[0], &ribbon, &bolt, &flange);
        let pour_xforms = build_pour_gate_transforms(&ribbon);
        let profile = fixture_profile(&body, &ribbon, bounds);
        let basis = profile.basis();
        let MatingTransform::SubtractCylinder { params: bore } = &pour_xforms[0] else {
            panic!("apex pour must emit a cylinder");
        };
        // The bore axis projected onto the seam plane → the pierce arc; the two
        // brackets must straddle it ALONG the loop (one each direction), which is
        // the geometric "both sides" — the world axes don't separate the sides
        // when the pierce sits at a rounded end of the silhouette.
        let bore_axis = bore.parent.axis.into_inner();
        let h = bore.parent.half_length_m;
        let a = basis.project(bore.parent.center_m - bore_axis * h);
        let b = basis.project(bore.parent.center_m + bore_axis * h);
        let pierce = pour_pierce_arc(&profile, a, b);
        let perim = profile.perimeter();
        let (mut near, mut ahead, mut behind) = (0u32, false, false);
        for t in &xforms {
            let MatingTransform::SubtractCylinder { params } = t else {
                continue;
            };
            if pour_axis_distance(params.parent.center_m, &pour_xforms) < 0.025 {
                near += 1;
                let arc = profile.nearest_arc(basis.project(params.parent.center_m));
                let mut gap = (arc - pierce).rem_euclid(perim);
                if gap > perim / 2.0 {
                    gap -= perim;
                }
                if gap > 1e-6 {
                    ahead = true;
                } else if gap < -1e-6 {
                    behind = true;
                }
            }
        }
        assert!(
            near >= 2,
            "apex pour must be bracketed by ≥2 bolts; got {near}"
        );
        assert!(
            ahead && behind,
            "apex pour must be clamped on BOTH sides of the pierce along the loop; \
             near={near} ahead={ahead} behind={behind}"
        );
    }

    /// Per-layer (§3.8): solving on the outer layer and snapping inward yields the
    /// SAME number of bolts on every layer (the shared, all-layer-feasible set).
    #[test]
    fn smart_placement_shares_one_pattern_across_layers() {
        // Two nested cylinders along X (inner R=10, outer R=12), innermost-first.
        let make = |r: f64| {
            Solid::cylinder(r, 0.030).rotate(nalgebra::UnitQuaternion::from_axis_angle(
                &Vector3::y_axis(),
                std::f64::consts::FRAC_PI_2,
            ))
        };
        let inner = make(0.010);
        let outer = make(0.012);
        let (_b, bounds, base) = cylinder_fixture();
        let mut pour = PourGateSpec::iter1();
        pour.layout = PourGateLayout::ApexAxial;
        let ribbon = base
            .with_pour_gate(PourGateKind::Default(pour))
            .with_flange(FlangeKind::Plate(FlangeSpec::iter1()))
            .with_bolt_pattern(BoltPatternKind::Auto(BoltPatternSpec::iter1()));
        let bolt = BoltPatternSpec::iter1();
        let flange = FlangeSpec::iter1();

        let plan = plan_bolts(
            &[&inner, &outer],
            &[bounds, bounds],
            &ribbon,
            &bolt,
            &flange,
            0.005,
            None,
        );
        assert_eq!(plan.len(), 2);
        assert!(!plan[0].is_empty() && !plan[1].is_empty());
        assert_eq!(
            plan[0].len(),
            plan[1].len(),
            "shared pattern: every layer carries the same bolt count"
        );
    }

    /// The dowels-first contract (§3.6): dowels are solved first, then the bolt
    /// run excludes their footprints. Every solver bolt washer must clear every
    /// solver dowel footprint (hole + wall) in the shared set — no overlap.
    #[test]
    fn smart_bolts_clear_every_solver_dowel_footprint() {
        use crate::dowel_hole::{plan_smart_dowel_placements, smart_dowel_footprint};

        let (body, bounds, base) = cylinder_fixture();
        let mut pour = PourGateSpec::iter1();
        pour.layout = PourGateLayout::ApexAxial;
        let ribbon = base
            .with_pour_gate(PourGateKind::Default(pour))
            .with_flange(FlangeKind::Plate(FlangeSpec::iter1()))
            .with_dowel_hole(DowelHoleKind::Auto(DowelHoleSpec::iter1()))
            .with_bolt_pattern(BoltPatternKind::Auto(BoltPatternSpec::iter1()));
        let dowel = DowelHoleSpec::iter1();
        let bolt = BoltPatternSpec::iter1();
        let flange = FlangeSpec::iter1();
        let wall = 0.005;

        // Dowels FIRST, then bolts exclude them. Both run on the shared loops
        // (S5d-(A)), the way the v2 pipeline feeds them.
        let loops = build_layer_loops(&[&body], &[bounds], &ribbon, &flange);
        let dowel_plan = plan_smart_dowel_placements(&loops, &dowel, &flange, wall);
        assert_eq!(dowel_plan[0].len(), 2, "two registration dowels");
        let bolt_plan = plan_bolts(
            &[&body],
            &[bounds],
            &ribbon,
            &bolt,
            &flange,
            wall,
            Some(&dowel_plan),
        );
        assert!(
            !bolt_plan[0].is_empty(),
            "smart placement produced no bolts"
        );

        // Every bolt washer clears every dowel footprint (hole + wall) — the disk
        // the bolt run excluded. Compared directly in the seam plane (both
        // planners return seam-plane centers in the same basis).
        let min_sep = smart_dowel_footprint(&dowel) + BOLT_WASHER_RADIUS_M;
        for &bc in &bolt_plan[0] {
            for &dc in &dowel_plan[0] {
                let d = (bc.x - dc.x).hypot(bc.z - dc.z);
                assert!(
                    d >= min_sep - 1e-6,
                    "bolt washer fouls a dowel footprint: dist={d} < {min_sep}"
                );
            }
        }
    }
}
