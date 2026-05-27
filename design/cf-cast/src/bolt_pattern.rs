//! M5 through-bolt clamp pattern (§B of
//! `docs/CF_CAST_FLANGE_CONTINUITY_BOLT_PATTERN_RECON.md`).
//!
//! Both cup-halves get IDENTICAL bolt-clearance holes carved through
//! their mating faces, arc-length-equal-spaced around the body
//! silhouette in the flange band. Workshop user inserts an M5 hex
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
//! - **center** = silhouette point at arc fraction
//!   `(k + 0.5) / count` (k = 0..count) offset OUTWARD from the
//!   silhouette by `silhouette_outboard_offset_m` in the seam plane,
//!   then projected onto the binormal-aligned seam plane through the
//!   ribbon midpoint (same projection §M-S2 dowel-hole uses).
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
//! # Bolt-dowel arc-length stagger (automatic at default counts)
//!
//! Both this module and [`crate::dowel_hole`] use the same
//! `(k + 0.5) / count` arc-fraction formula. With the iter-1 defaults
//! (`DowelHoleSpec::iter1().count = 4` + `BoltPatternSpec::iter1().count
//! = 8`), bolt positions naturally interleave between dowel positions
//! — dowels at arc fractions {1/8, 3/8, 5/8, 7/8}, bolts at
//! {1/16, 3/16, 5/16, …, 15/16}, minimum arc-length separation =
//! 1/16 of perimeter. For typical sock geometry (200 mm perimeter)
//! that's 12.5 mm separation, far above the radial-collision
//! threshold of `dowel_radius + bolt_radius + clearance`. Workshop
//! configs that violate this (e.g., equal counts) are gated at parse
//! time in `cf-cast-cli::config::validate_after_layer_source`.
//!
//! # Pour-gate collision skip
//!
//! The V-shape pour gate at the dome apex emits
//! [`MatingTransform::SubtractCylinder`] legs that may overlap with
//! arc-length-equal-spaced bolt positions near the dome. When
//! `spec.skip_pour_gate_collision` is true (the default), each bolt
//! position is checked against the ribbon's pour-gate cylinder
//! centerlines; bolts whose centers fall within
//! `bolt_radius + pour_gate_radius + spec.pour_gate_clearance_m` of
//! any pour-gate cylinder axis are dropped silently (no rebalance —
//! the workshop accepts a slight asymmetry near the pour gate; full
//! rebalance is bookmarked for a future arc).
//!
//! # Side-agnostic emission
//!
//! Identical transform list applied to BOTH Negative + Positive cup
//! pieces (mesh-CSG subtract is invariant to which half it's applied
//! to; cylinder portion outside a given half is a no-op). Both
//! halves get matching bolt-hole positions mirrored across the seam
//! plane, so a single M5 bolt passes through the corresponding holes
//! on the two halves to clamp them together.

use cf_design::Solid;
use nalgebra::{Point3, Unit, Vector3};

use crate::flange::FlangeSpec;
use crate::mesh_csg::{CylinderParams, CylinderParent, MatingTransform};
use crate::pour::build_pour_gate_transforms;
use crate::ribbon::Ribbon;
use crate::silhouette_2d::{SILHOUETTE_GRID_STEP_M, Silhouette2d};

/// Default bolt clearance hole diameter (5.5 mm) — standard M5
/// clearance per ISO 273 medium-fit.
const DEFAULT_CLEARANCE_DIAMETER_M: f64 = 0.0055;

/// Default bolt count around the silhouette (8 — even pressure
/// distribution on the iter-1 sock geometry, ~25 mm spacing on a
/// typical 200 mm perimeter; engine-valve-cover-equivalent for a
/// small mold). Workshop user picks per §B recon OQ1.
const DEFAULT_COUNT: u32 = 8;

/// Default radial offset from the body silhouette curve to the bolt
/// centerline (9 mm). With the default 5.5 mm bolt clearance and the
/// post-§B 16 mm flange width + 2 mm inner offset, this gives 4.25 mm
/// of inboard wall (toward the gasket channel) AND 4.25 mm of
/// outboard wall (toward the flange edge) — symmetric stress
/// distribution at 0.77× clearance Ø, comfortably within the FDM-PLA
/// 0.75× rule of thumb.
const DEFAULT_OUTBOARD_OFFSET_M: f64 = 0.009;

/// Default minimum clearance between bolt cylinder + pour-gate
/// cylinder centerlines, beyond the sum of the two radii. 1 mm of
/// PLA between the two cylindrical voids — well above the FDM-PLA
/// minimum-wall floor.
const DEFAULT_POUR_GATE_CLEARANCE_M: f64 = 0.001;

/// Polygonal facets around each bolt cylinder. 32 segments give
/// ~0.4 mm chord error at 5.5 mm Ø, well below FDM print resolution
/// and matching the segment count of the dowel-HOLE
/// [`SubtractCylinder`] primitives so both surfaces have uniform
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
    /// Number of bolts arc-length-equal-spaced around the silhouette.
    /// Default 8.
    pub count: u32,
    /// Radial offset from the body silhouette curve to the bolt
    /// centerline (meters). Default 9 mm.
    pub silhouette_outboard_offset_m: f64,
    /// Whether to silently drop bolts that collide with the pour-gate
    /// channels. Default `true`. When `false`, every bolt is emitted
    /// regardless — workshop may need to drill out the collision-zone
    /// pour-gate leg by hand or accept that those bolts can't seat.
    pub skip_pour_gate_collision: bool,
    /// Minimum clearance distance between bolt cylinder + pour-gate
    /// cylinder centerlines, ABOVE the sum of the two radii (meters).
    /// Default 1 mm. Only consulted when
    /// `skip_pour_gate_collision = true`.
    pub pour_gate_clearance_m: f64,
}

impl BoltPatternSpec {
    /// Workshop iter-1 starting defaults pinned at §B-S1 recon.
    #[must_use]
    pub const fn iter1() -> Self {
        Self {
            clearance_diameter_m: DEFAULT_CLEARANCE_DIAMETER_M,
            count: DEFAULT_COUNT,
            silhouette_outboard_offset_m: DEFAULT_OUTBOARD_OFFSET_M,
            skip_pour_gate_collision: true,
            pour_gate_clearance_m: DEFAULT_POUR_GATE_CLEARANCE_M,
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

/// Distance from point `p` to the FINITE cylinder axis segment
/// centered at `c` with unit direction `axis` and half-length
/// `half_length`. The cylinder's axis spans `c ± half_length * axis`;
/// the returned distance is the perpendicular distance to that
/// segment, clamping the axis-projection of `p` to the endcaps so
/// queries past the cylinder's caps measure to the cap center rather
/// than to the infinite axis line.
///
/// Used by the pour-gate collision check — pour-gate legs are short
/// (≈40-45 mm half-length) and a bolt whose axis-projection lands
/// PAST the leg's endcap is not actually colliding with the leg even
/// if its perpendicular-line distance is small. Cold-read finding
/// 2026-05-27 (I1).
///
/// `axis` must already be unit-length (the caller passes
/// `CylinderParent.axis: Unit<Vector3<f64>>`).
fn point_to_cylinder_axis_distance(
    p: Point3<f64>,
    c: Point3<f64>,
    axis: Vector3<f64>,
    half_length: f64,
) -> f64 {
    let to_p = p - c;
    // Project `to_p` onto axis; clamp to ±half_length to stay within
    // the finite cylinder. Closest axis point is then
    // `c + clamped_t * axis`.
    let t = to_p.dot(&axis).clamp(-half_length, half_length);
    let closest = c + axis * t;
    (p - closest).norm()
}

/// Returns `true` if a bolt at `bolt_center` with radius `bolt_radius`
/// collides with any pour-gate cylinder in `pour_xforms`, within
/// `extra_clearance` of the cylinder surfaces.
fn collides_with_pour_gate(
    bolt_center: Point3<f64>,
    bolt_radius: f64,
    pour_xforms: &[MatingTransform],
    extra_clearance: f64,
) -> bool {
    for t in pour_xforms {
        if let MatingTransform::SubtractCylinder { params } = t {
            let dist = point_to_cylinder_axis_distance(
                bolt_center,
                params.parent.center_m,
                params.parent.axis.into_inner(),
                params.parent.half_length_m,
            );
            if dist < bolt_radius + params.radius_m + extra_clearance {
                return true;
            }
        }
    }
    false
}

/// Build the side-agnostic bolt-pattern [`MatingTransform`] vec for
/// the given `ribbon.bolt_pattern` spec + layer body silhouette.
///
/// Returns an empty Vec if the silhouette has no polylines (e.g.,
/// body extends past silhouette region) — caller treats this as
/// "no bolt holes for this piece".
///
/// `flange_spec` carries the `flange_thickness_m` that sizes the
/// bolt cylinder half-length (the bolt must carve through both
/// halves' flange material). `bounds` is the cup-piece's MC bounds
/// (used to size the silhouette sampling region — same as the
/// flange's silhouette-construction bounds for consistency).
#[must_use]
pub fn build_bolt_pattern_transforms(
    layer_body: &Solid,
    ribbon: &Ribbon,
    spec: &BoltPatternSpec,
    flange_spec: &FlangeSpec,
    bounds: cf_design::Aabb,
) -> Vec<MatingTransform> {
    if spec.count == 0 {
        return Vec::new();
    }
    let (seam_midpoint, seam_normal) = ribbon.seam_plane_reference();
    let binormal = seam_normal.into_inner();
    let seam_plane_y = seam_midpoint.y;

    let radius_m = spec.clearance_diameter_m / 2.0;
    // Silhouette sampling padding: bolt center + radius + grid step.
    let pad = spec.silhouette_outboard_offset_m + radius_m + SILHOUETTE_GRID_STEP_M;
    let silhouette = Silhouette2d::from_body_at_y(
        layer_body,
        seam_plane_y,
        bounds.min.x - pad,
        bounds.max.x + pad,
        bounds.min.z - pad,
        bounds.max.z + pad,
    );

    // Through-hole half-length = flange thickness + slack. Cylinder
    // spans ±(flange + slack) in binormal direction, carving through
    // BOTH halves' flange material (which together span ±flange in
    // binormal). Slack ensures the cylindrical caps sit OUTSIDE the
    // flange's back face on each side.
    let half_length_m = flange_spec.flange_thickness_m + BOLT_AXIAL_SLACK_M;
    let axis = Unit::new_normalize(binormal);

    let pour_xforms = build_pour_gate_transforms(ribbon);

    let mut transforms = Vec::with_capacity(spec.count as usize);
    for k in 0..spec.count {
        // Same (k + 0.5) / count formula as dowel_hole — produces
        // natural arc-length stagger between dowel + bolt patterns
        // when their counts differ (the iter-1 defaults give
        // bolts {1/16, 3/16, …} interleaved with dowels {2/16, 6/16,
        // 10/16, 14/16}). The +0.5 also avoids placing a bolt at the
        // silhouette's polyline-assembly join point.
        let t = (f64::from(k) + 0.5) / f64::from(spec.count);
        let Some(p_silh) = silhouette.point_at_arc_fraction(t) else {
            return Vec::new();
        };
        let Some(n_out) = silhouette.outward_normal_at_arc_fraction(t) else {
            continue;
        };
        // Offset outward from the silhouette in (X, Z), then project
        // onto the binormal-aligned seam plane through seam_midpoint.
        // Same projection §M-S2 dowel-hole uses post cold-read fix —
        // for tilted centerlines the cast-Y-constant plane is NOT
        // the seam plane, and an unprojected center would carve the
        // bolt asymmetrically between the two halves.
        let candidate = Point3::new(
            spec.silhouette_outboard_offset_m.mul_add(n_out.x, p_silh.x),
            seam_midpoint.y,
            spec.silhouette_outboard_offset_m.mul_add(n_out.z, p_silh.z),
        );
        let signed_dist_from_seam = (candidate - seam_midpoint).dot(&binormal);
        let center_m = candidate - signed_dist_from_seam * binormal;

        if spec.skip_pour_gate_collision
            && collides_with_pour_gate(center_m, radius_m, &pour_xforms, spec.pour_gate_clearance_m)
        {
            continue;
        }

        transforms.push(MatingTransform::SubtractCylinder {
            params: CylinderParams {
                parent: CylinderParent {
                    center_m,
                    axis,
                    half_length_m,
                },
                radius_m,
                segments: DEFAULT_SEGMENTS,
            },
        });
    }
    transforms
}

#[cfg(test)]
mod tests {
    #![allow(
        clippy::unwrap_used,
        clippy::panic,
        clippy::expect_used,
        clippy::float_cmp
    )]

    use super::*;
    use crate::pour::{PourGateKind, PourGateSpec};
    use crate::ribbon::SplitNormal;
    use nalgebra::Vector3;

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

    #[test]
    fn bolt_pattern_spec_iter1_defaults() {
        let s = BoltPatternSpec::iter1();
        assert_eq!(s.clearance_diameter_m, 0.0055);
        assert_eq!(s.count, 8);
        assert_eq!(s.silhouette_outboard_offset_m, 0.009);
        assert!(s.skip_pour_gate_collision);
        assert_eq!(s.pour_gate_clearance_m, 0.001);
    }

    #[test]
    fn bolt_pattern_kind_spec_accessor() {
        assert!(BoltPatternKind::None.spec().is_none());
        let s = BoltPatternSpec::iter1();
        let kind = BoltPatternKind::Auto(s);
        assert_eq!(kind.spec(), Some(&s));
    }

    #[test]
    fn build_transforms_zero_count_emits_empty_vec() {
        let (body, bounds, ribbon) = cylinder_fixture();
        let spec = BoltPatternSpec {
            count: 0,
            ..BoltPatternSpec::iter1()
        };
        let flange = FlangeSpec::iter1();
        let transforms = build_bolt_pattern_transforms(&body, &ribbon, &spec, &flange, bounds);
        assert!(transforms.is_empty());
    }

    #[test]
    fn build_transforms_no_pour_gate_emits_count_subtract_cylinders() {
        let (body, bounds, ribbon) = cylinder_fixture();
        let spec = BoltPatternSpec::iter1();
        let flange = FlangeSpec::iter1();
        let transforms = build_bolt_pattern_transforms(&body, &ribbon, &spec, &flange, bounds);
        assert_eq!(
            transforms.len(),
            spec.count as usize,
            "must emit one SubtractCylinder per bolt when no pour-gate collision; \
             got {}",
            transforms.len()
        );
        for t in &transforms {
            assert!(
                matches!(t, MatingTransform::SubtractCylinder { .. }),
                "all transforms must be SubtractCylinder; got {t:?}"
            );
        }
    }

    #[test]
    fn bolt_radius_matches_clearance_half_no_slop() {
        let (body, bounds, ribbon) = cylinder_fixture();
        let spec = BoltPatternSpec::iter1();
        let flange = FlangeSpec::iter1();
        let transforms = build_bolt_pattern_transforms(&body, &ribbon, &spec, &flange, bounds);
        let expected_radius = spec.clearance_diameter_m / 2.0;
        for t in &transforms {
            let MatingTransform::SubtractCylinder { params } = t else {
                continue;
            };
            assert!(
                (params.radius_m - expected_radius).abs() < 1e-12,
                "bolt radius = clearance_diameter / 2 = {expected_radius:.5} m; \
                 got {:.5} m",
                params.radius_m
            );
        }
    }

    #[test]
    fn bolts_axis_aligned_with_ribbon_binormal() {
        let (body, bounds, ribbon) = cylinder_fixture();
        let spec = BoltPatternSpec::iter1();
        let flange = FlangeSpec::iter1();
        let transforms = build_bolt_pattern_transforms(&body, &ribbon, &spec, &flange, bounds);
        let (_, seam_normal) = ribbon.seam_plane_reference();
        let binormal = seam_normal.into_inner();
        for t in &transforms {
            let MatingTransform::SubtractCylinder { params } = t else {
                continue;
            };
            let dot = params.parent.axis.into_inner().dot(&binormal).abs();
            assert!(
                (dot - 1.0).abs() < 1e-9,
                "bolt axis must align with ribbon binormal; got dot = {dot}"
            );
        }
    }

    #[test]
    fn bolts_centered_outboard_of_silhouette() {
        let (body, bounds, ribbon) = cylinder_fixture();
        let spec = BoltPatternSpec::iter1();
        let flange = FlangeSpec::iter1();
        let transforms = build_bolt_pattern_transforms(&body, &ribbon, &spec, &flange, bounds);
        for (i, t) in transforms.iter().enumerate() {
            let MatingTransform::SubtractCylinder { params } = t else {
                panic!("expected SubtractCylinder, got {t:?}");
            };
            let center = params.parent.center_m;
            let body_dist = body.evaluate(&center);
            assert!(
                body_dist > 0.0,
                "bolt #{i} center must be OUTSIDE body in 3D; got body_dist = \
                 {body_dist:.5} m"
            );
            let target = spec.silhouette_outboard_offset_m;
            let tol = 2.0 * SILHOUETTE_GRID_STEP_M;
            assert!(
                (body_dist - target).abs() < tol,
                "bolt #{i} center distance from body must be ≈ {target:.4} m \
                 (silhouette_outboard_offset_m); got {body_dist:.5} m \
                 (tolerance {tol:.4} m)"
            );
        }
    }

    #[test]
    fn bolt_half_length_spans_flange_plus_slack() {
        let (body, bounds, ribbon) = cylinder_fixture();
        let spec = BoltPatternSpec::iter1();
        let flange = FlangeSpec::iter1();
        let transforms = build_bolt_pattern_transforms(&body, &ribbon, &spec, &flange, bounds);
        let expected_half_length = flange.flange_thickness_m + BOLT_AXIAL_SLACK_M;
        for t in &transforms {
            let MatingTransform::SubtractCylinder { params } = t else {
                continue;
            };
            assert!(
                (params.parent.half_length_m - expected_half_length).abs() < 1e-12,
                "bolt half-length = flange_thickness + slack = \
                 {expected_half_length:.5} m; got {:.5} m",
                params.parent.half_length_m
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
    #[test]
    fn bolt_centers_lie_on_binormal_aligned_seam_plane_under_tilted_centerline() {
        let centerline = vec![
            Point3::new(-0.050, -0.005, 0.0),
            Point3::new(0.050, 0.005, 0.0),
        ];
        let split = SplitNormal::new(Vector3::new(0.0, 0.0, 1.0)).unwrap();
        let ribbon = Ribbon::new(centerline, split).unwrap();
        let layer_body =
            Solid::cylinder(0.010, 0.060).rotate(nalgebra::UnitQuaternion::from_axis_angle(
                &Vector3::y_axis(),
                std::f64::consts::FRAC_PI_2,
            ));
        let bounding_region = Solid::cuboid(Vector3::new(0.100, 0.030, 0.030));
        let bounds = bounding_region.bounds().unwrap();
        let spec = BoltPatternSpec::iter1();
        let flange = FlangeSpec::iter1();
        let transforms =
            build_bolt_pattern_transforms(&layer_body, &ribbon, &spec, &flange, bounds);
        assert_eq!(transforms.len(), spec.count as usize);

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
                 got binormal-distance = {dist_from_seam:.6} m (pre-fix this \
                 could be several mm for tilted centerlines, producing \
                 asymmetric hole carve across the two cup-halves)"
            );
        }
    }

    /// Direct unit test of `collides_with_pour_gate` with a synthetic
    /// pour-gate cylinder at a known position. Bolt sits 2 mm from
    /// the pour-gate cylinder's axis midpoint (well inside the
    /// `bolt_radius + pour_radius + clearance` threshold = 8.5 mm) —
    /// must register as colliding.
    #[test]
    fn collides_with_pour_gate_detects_close_bolt() {
        let pour_xform = MatingTransform::SubtractCylinder {
            params: CylinderParams {
                parent: CylinderParent {
                    center_m: Point3::new(0.0, 0.0, 0.0),
                    axis: Unit::new_normalize(Vector3::new(1.0, 0.0, 0.0)),
                    half_length_m: 0.045, // pour-gate iter-1 default
                },
                radius_m: 0.005, // pour-gate iter-1 default
                segments: 32,
            },
        };
        // Bolt 2 mm off the pour-leg axis, inside the cap extent.
        let bolt_center = Point3::new(0.010, 0.002, 0.0);
        let bolt_radius = 0.00275; // M5 / 2
        let clearance = 0.001;
        assert!(
            collides_with_pour_gate(bolt_center, bolt_radius, &[pour_xform], clearance),
            "bolt 2 mm off pour-leg axis must register as colliding \
             (bolt_radius {bolt_radius} + pour_radius 0.005 + clearance \
             {clearance} = 0.00875 > 0.002)"
        );
    }

    /// Direct unit test of `collides_with_pour_gate`: bolt safely
    /// distant from the pour-leg must NOT register as colliding.
    /// Bolt center 20 mm off-axis is far above the 8.75 mm threshold.
    #[test]
    fn collides_with_pour_gate_ignores_distant_bolt() {
        let pour_xform = MatingTransform::SubtractCylinder {
            params: CylinderParams {
                parent: CylinderParent {
                    center_m: Point3::new(0.0, 0.0, 0.0),
                    axis: Unit::new_normalize(Vector3::new(1.0, 0.0, 0.0)),
                    half_length_m: 0.045,
                },
                radius_m: 0.005,
                segments: 32,
            },
        };
        let bolt_center = Point3::new(0.010, 0.020, 0.0);
        let bolt_radius = 0.00275;
        let clearance = 0.001;
        assert!(
            !collides_with_pour_gate(bolt_center, bolt_radius, &[pour_xform], clearance),
            "bolt 20 mm off pour-leg axis must NOT register as colliding"
        );
    }

    /// Cold-read I1 regression: a bolt PAST the pour-gate cylinder's
    /// endcap, even when close to the infinite-axis line, must NOT
    /// trigger collision (the cylinder is FINITE). Pre-fix
    /// `point_to_line_distance` returned the perpendicular ~0.002
    /// (false collision); post-fix `point_to_cylinder_axis_distance`
    /// clamps to the endcap and returns sqrt(0.030² + 0.002²) ≈
    /// 0.030 m, well past the 0.00875 m threshold.
    #[test]
    fn collides_with_pour_gate_does_not_count_bolt_past_endcap() {
        let pour_xform = MatingTransform::SubtractCylinder {
            params: CylinderParams {
                parent: CylinderParent {
                    center_m: Point3::new(0.0, 0.0, 0.0),
                    axis: Unit::new_normalize(Vector3::new(1.0, 0.0, 0.0)),
                    half_length_m: 0.020, // short pour leg for the test
                },
                radius_m: 0.005,
                segments: 32,
            },
        };
        // Bolt at X = 0.050 → axis-projection = 0.050, well past the
        // +X cap at X = 0.020. Off-axis only 0.002 m — pre-fix
        // (infinite line) would have flagged this as colliding.
        let bolt_center = Point3::new(0.050, 0.002, 0.0);
        let bolt_radius = 0.00275;
        let clearance = 0.001;
        assert!(
            !collides_with_pour_gate(bolt_center, bolt_radius, &[pour_xform], clearance),
            "bolt past pour-leg endcap must NOT register as colliding \
             (pre-I1-fix this would falsely fire because infinite-line \
             distance ignored the cap)"
        );
    }

    /// End-to-end skip-disabled test: with `skip_pour_gate_collision
    /// = false`, all bolts emitted regardless of pour-gate presence.
    /// Pairs with the direct unit tests above to lock the wiring.
    #[test]
    fn pour_gate_collision_skip_disabled_emits_all_bolts() {
        let (body, bounds, ribbon) = cylinder_fixture();
        let ribbon = ribbon.with_pour_gate(PourGateKind::Default(PourGateSpec::iter1()));
        let spec = BoltPatternSpec {
            skip_pour_gate_collision: false,
            ..BoltPatternSpec::iter1()
        };
        let flange = FlangeSpec::iter1();
        let transforms = build_bolt_pattern_transforms(&body, &ribbon, &spec, &flange, bounds);
        assert_eq!(
            transforms.len(),
            spec.count as usize,
            "skip_pour_gate_collision=false must emit all {} bolts; got {}",
            spec.count,
            transforms.len()
        );
    }

    /// Bolt-dowel arc-length stagger at default counts: bolts at
    /// `(k + 0.5) / 8` and dowels at `(k + 0.5) / 4` interleave with
    /// minimum arc-fraction separation = 1/16. On a 100 mm centerline
    /// (200 mm typical perimeter for the test body, ~62.8 mm for
    /// R=10 mm cylinder) this still gives well above the radial-
    /// collision threshold. Locks the documented "auto-stagger at
    /// defaults" invariant — workshop default configs never collide.
    #[test]
    fn bolt_arc_fractions_interleave_with_dowel_arc_fractions_at_default_counts() {
        // Dowels emitted at arc fractions {1/8, 3/8, 5/8, 7/8}.
        // Bolts emitted at arc fractions {1/16, 3/16, 5/16, …, 15/16}.
        // Min separation = 1/16 = 0.0625 of perimeter.
        let dowel_count = crate::dowel_hole::DowelHoleSpec::iter1().count;
        let bolt_count = BoltPatternSpec::iter1().count;
        let dowel_fractions: Vec<f64> = (0..dowel_count)
            .map(|k| (f64::from(k) + 0.5) / f64::from(dowel_count))
            .collect();
        let bolt_fractions: Vec<f64> = (0..bolt_count)
            .map(|k| (f64::from(k) + 0.5) / f64::from(bolt_count))
            .collect();
        let mut min_sep = f64::INFINITY;
        for db in &dowel_fractions {
            for bb in &bolt_fractions {
                let sep = (db - bb).abs();
                if sep < min_sep {
                    min_sep = sep;
                }
            }
        }
        // 1/16 = 0.0625; allow FP slack.
        assert!(
            (min_sep - 0.0625).abs() < 1e-9,
            "bolt + dowel arc-fraction interleave must give 1/16 separation \
             at default counts; got {min_sep:.6}"
        );
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
}
