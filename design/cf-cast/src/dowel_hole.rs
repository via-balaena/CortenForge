//! Symmetric dowel-hole registration primitive (§M-S2 of
//! `docs/CF_CAST_UNIFIED_MATING_PLANE_RECON.md`).
//!
//! Both cup-halves get IDENTICAL dowel holes through their mating
//! faces, arc-length-equal-spaced around the body silhouette in the
//! flange band. Workshop user inserts printed-separately PLA dowels
//! into the holes at assembly time; the dowels register the two
//! halves laterally (preventing slip in the seam plane) while the
//! bolts (§B arc) clamp them axially.
//!
//! Replaces the pre-§M asymmetric pin/socket pair pattern (post-
//! 2026-05-24 salvage mesh-CSG `UnionTruncatedPyramid` +
//! `SubtractTruncatedPyramid` paired with side-specific pose). The
//! symmetric `SubtractCylinder`-only emission is cleaner per
//! [[project-cf-cast-unified-mating-plane-recon]]'s §M-10 lesson
//! about symmetric registration being preferable to asymmetric
//! pin/socket primitives.
//!
//! # Geometry
//!
//! Each dowel hole is a `MatingTransform::SubtractCylinder` post-MC
//! mesh-CSG primitive with:
//! - **axis** = ribbon binormal (perpendicular to mating plane —
//!   parallel to print Z when the cup piece is mated-face-down on the
//!   build plate, so the hole prints as a vertical cylinder requiring
//!   no support).
//! - **center** = silhouette point at arc fraction
//!   `(k + 0.5) / count` (k = 0..count) offset OUTWARD from the
//!   silhouette by `silhouette_outboard_offset_m` in the seam plane.
//!   The +0.5 offset avoids placing dowels at the silhouette's
//!   polyline-assembly join point (which may be on an MC corner
//!   ambiguity).
//! - **`half_length`** = `depth_m + clearance_slack_m`. The full
//!   cylinder spans `2 × half_length` straddling the seam plane,
//!   carving `depth_m` into EACH cup half.
//! - **radius** = `dowel_diameter_m / 2 + clearance_m`. PLA-on-PLA
//!   slide-fit clearance.
//!
//! # Side-agnostic emission
//!
//! The same transform list is applied to BOTH Negative + Positive
//! cup pieces (mesh-CSG subtract is invariant to which half it's
//! applied to; cylinder portion outside a given half is a no-op).
//! Both halves get matching hole positions mirrored across the seam
//! plane, so a single printed dowel inserts through the corresponding
//! holes on the two halves to register them.

use cf_design::Solid;
use nalgebra::{Point3, Unit};

use crate::mesh_csg::{CylinderParams, CylinderParent, MatingTransform};
use crate::ribbon::Ribbon;
use crate::silhouette_2d::{SILHOUETTE_GRID_STEP_M, Silhouette2d};

/// Default dowel diameter (3 mm) — matches standard 3 mm wood-dowel
/// stock + small PLA-printed dowel sizes (per §M-S2 recon).
const DEFAULT_DIAMETER_M: f64 = 0.003;

/// Default radial clearance between dowel and hole wall (0.1 mm) —
/// PLA-on-PLA slide fit with thermal-tolerance margin.
const DEFAULT_CLEARANCE_M: f64 = 0.0001;

/// Default hole depth PER HALF (5 mm). Printed dowel length should
/// be `2 × DEFAULT_DEPTH_M - 2 × axial_slack` ≈ 9 mm (0.5 mm slack
/// each side for assembly play).
const DEFAULT_DEPTH_M: f64 = 0.005;

/// Default hole count around the silhouette (4 — one in each
/// quadrant for a typical sock-shaped body). Workshop user picks per
/// §M recon OQ3.
const DEFAULT_COUNT: u32 = 4;

/// Default radial offset from the body silhouette curve to the dowel
/// centerline (8 mm) — mid-band in the post-§B `[2, 16]` mm flange
/// band, leaves 4.4 mm inboard wall + 6.4 mm outboard wall on each
/// side of the hole.
const DEFAULT_OUTBOARD_OFFSET_M: f64 = 0.008;

/// Polygonal facets around the dowel cylinder. 32 segments give a
/// ~0.2 mm chord error at 3 mm diameter — workshop-imperceptible at
/// FDM print resolution. Determinism contract per
/// [`CylinderParams::segments`].
const DEFAULT_SEGMENTS: u32 = 32;

/// Extra axial slack on each side of the hole's nominal depth.
///
/// Ensures the cylinder's CYLINDRICAL flat caps are well INSIDE the
/// cup-piece material (avoids mesh-CSG operating on coincident
/// endcap planes). 0.5 mm is well below FDM resolution + safely past
/// any mating-face MC ambiguity. Public so consumers (e.g.
/// `crate::dowel`, `crate::procedure`) can compute total cavity
/// depth + dowel tip-slack accurately.
pub const HOLE_AXIAL_SLACK_M: f64 = 0.0005;

/// Dowel-hole geometry parameter envelope per §M-S2 recon.
///
/// Workshop user picks via cf-cast-cli `[dowel_hole]` TOML block
/// (§M-S3 territory); iter-1 starts at the defaults below.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DowelHoleSpec {
    /// Dowel diameter (meters). Default 3 mm.
    pub diameter_m: f64,
    /// Radial clearance between dowel and hole wall (meters).
    /// Default 0.1 mm.
    pub clearance_m: f64,
    /// Hole depth PER HALF (meters). The cylinder spans
    /// `2 × depth_m` straddling the seam plane. Default 5 mm.
    pub depth_m: f64,
    /// Number of dowels arc-length-equal-spaced around the
    /// silhouette. Default 4.
    pub count: u32,
    /// Radial offset from the body silhouette curve to the dowel
    /// centerline (meters). Default 8 mm. Must satisfy the §M-5-b
    /// cross-field invariants in the recon (inboard + outboard wall
    /// thicknesses ≥ FDM floor).
    pub silhouette_outboard_offset_m: f64,
}

impl DowelHoleSpec {
    /// Workshop iter-1 starting defaults pinned at §M-S2 recon.
    #[must_use]
    pub const fn iter1() -> Self {
        Self {
            diameter_m: DEFAULT_DIAMETER_M,
            clearance_m: DEFAULT_CLEARANCE_M,
            depth_m: DEFAULT_DEPTH_M,
            count: DEFAULT_COUNT,
            silhouette_outboard_offset_m: DEFAULT_OUTBOARD_OFFSET_M,
        }
    }
}

impl Default for DowelHoleSpec {
    fn default() -> Self {
        Self::iter1()
    }
}

/// Dowel-hole kind on the [`Ribbon`].
///
/// Matches the existing `RegistrationKind` / `PourGateKind` /
/// `FlangeKind` / `PlugPinKind` / `GasketKind` patterns — bridges
/// the cf-cast-cli `[dowel_hole]` TOML block (§M-S3 territory) to
/// the per-piece dowel-hole emission in
/// [`crate::compose_piece_solid`].
///
/// Default [`Self::None`] (no dowel holes — cup pieces register via
/// whatever other mechanism the workshop uses). Set
/// `DowelHoleKind::Auto(DowelHoleSpec::iter1())` via
/// [`Ribbon::with_dowel_hole`] to enable the symmetric dowel-hole
/// pattern.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DowelHoleKind {
    /// No dowel-hole emission.
    None,
    /// Arc-length-equal-spaced dowel holes around the silhouette,
    /// parameterized by the inner [`DowelHoleSpec`].
    Auto(DowelHoleSpec),
}

impl DowelHoleKind {
    /// Returns the inner [`DowelHoleSpec`] for [`DowelHoleKind::Auto`],
    /// `None` otherwise. Convenience accessor for the export pipeline.
    #[must_use]
    pub const fn spec(&self) -> Option<&DowelHoleSpec> {
        match self {
            Self::None => None,
            Self::Auto(spec) => Some(spec),
        }
    }
}

/// Build the side-agnostic dowel-hole [`MatingTransform`] vec for
/// the given `ribbon.dowel_hole` spec + layer body silhouette.
///
/// Returns an empty Vec if the silhouette has no polylines (e.g.,
/// body extends past silhouette region) — caller treats this as
/// "no dowel holes for this piece".
///
/// `bounds` is the cup-piece's MC bounds (used to size the silhouette
/// sampling region — same as the flange's silhouette-construction
/// bounds for consistency).
#[must_use]
pub fn build_dowel_hole_transforms(
    layer_body: &Solid,
    ribbon: &Ribbon,
    spec: &DowelHoleSpec,
    bounds: cf_design::Aabb,
) -> Vec<MatingTransform> {
    let (seam_midpoint, seam_normal) = ribbon.seam_plane_reference();
    let binormal = seam_normal.into_inner();
    let seam_plane_y = seam_midpoint.y;

    // Silhouette sampling region: same expansion as the flange uses
    // so the silhouette curve is fully captured for arc-length walks.
    // Pad by silhouette_outboard_offset + dowel radius + clearance
    // so the dowel centerline + hole radius stay inside the sampled
    // grid (otherwise a dowel near the silhouette extreme might fall
    // off the grid).
    let pad = spec.silhouette_outboard_offset_m
        + spec.diameter_m / 2.0
        + spec.clearance_m
        + SILHOUETTE_GRID_STEP_M;
    let silhouette = Silhouette2d::from_body_at_y(
        layer_body,
        seam_plane_y,
        bounds.min.x - pad,
        bounds.max.x + pad,
        bounds.min.z - pad,
        bounds.max.z + pad,
    );

    let half_length_m = spec.depth_m + HOLE_AXIAL_SLACK_M;
    let radius_m = spec.diameter_m / 2.0 + spec.clearance_m;

    let mut transforms = Vec::with_capacity(spec.count as usize);
    if spec.count == 0 {
        return transforms;
    }
    let axis = Unit::new_normalize(binormal);
    for k in 0..spec.count {
        // (k + 0.5) / count: midpoint of the k-th arc segment.
        // Avoids placing a dowel at the silhouette's polyline-
        // assembly join point (which may be on an MC corner with
        // ambiguous tangent).
        let t = (f64::from(k) + 0.5) / f64::from(spec.count);
        let Some(p_silh) = silhouette.point_at_arc_fraction(t) else {
            // No polylines (silhouette empty) → no dowels.
            return Vec::new();
        };
        let Some(n_out) = silhouette.outward_normal_at_arc_fraction(t) else {
            continue;
        };
        // Offset outward from the silhouette in (X, Z) by
        // silhouette_outboard_offset_m, then project the result onto
        // the binormal-aligned seam plane through seam_midpoint so
        // the cylinder center sits EXACTLY on the mating plane.
        //
        // Cold-read finding 2026-05-27: pre-projection version put
        // `center_m.y = seam_midpoint.y` (constant-Y plane), which
        // for the iter-1 ~3.4° tilted centerline placed the cylinder
        // center up to a few mm off the binormal-aligned seam plane —
        // making the dowel carve asymmetrically (one half ~9 mm
        // deep, the other ~2 mm). Now: project onto the binormal
        // plane so both halves get equal carve depth. Per
        // [[project-cf-cast-unified-mating-plane-recon]] §M-S1.5
        // (binormal-aligned vertical) extended to dowel positioning.
        let candidate = Point3::new(
            spec.silhouette_outboard_offset_m.mul_add(n_out.x, p_silh.x),
            seam_midpoint.y,
            spec.silhouette_outboard_offset_m.mul_add(n_out.z, p_silh.z),
        );
        let signed_dist_from_seam = (candidate - seam_midpoint).dot(&binormal);
        let center_m = candidate - signed_dist_from_seam * binormal;
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
    fn dowel_hole_spec_iter1_defaults() {
        let s = DowelHoleSpec::iter1();
        assert_eq!(s.diameter_m, 0.003);
        assert_eq!(s.clearance_m, 0.0001);
        assert_eq!(s.depth_m, 0.005);
        assert_eq!(s.count, 4);
        assert_eq!(s.silhouette_outboard_offset_m, 0.008);
    }

    #[test]
    fn dowel_hole_kind_spec_accessor() {
        assert!(DowelHoleKind::None.spec().is_none());
        let s = DowelHoleSpec::iter1();
        let kind = DowelHoleKind::Auto(s);
        assert_eq!(kind.spec(), Some(&s));
    }

    #[test]
    fn build_transforms_emits_count_subtract_cylinders() {
        let (body, bounds, ribbon) = cylinder_fixture();
        let spec = DowelHoleSpec::iter1();
        let transforms = build_dowel_hole_transforms(&body, &ribbon, &spec, bounds);
        assert_eq!(
            transforms.len(),
            spec.count as usize,
            "must emit one SubtractCylinder per dowel; got {}",
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
    fn build_transforms_zero_count_emits_empty_vec() {
        let (body, bounds, ribbon) = cylinder_fixture();
        let spec = DowelHoleSpec {
            count: 0,
            ..DowelHoleSpec::iter1()
        };
        let transforms = build_dowel_hole_transforms(&body, &ribbon, &spec, bounds);
        assert!(transforms.is_empty());
    }

    #[test]
    fn dowel_holes_centered_outboard_of_silhouette() {
        // Each dowel center must be OUTSIDE the body silhouette by
        // approximately silhouette_outboard_offset_m. Probe each
        // cylinder's center in the body SDF — should be at distance
        // ≈ offset (within the silhouette grid step).
        let (body, bounds, ribbon) = cylinder_fixture();
        let spec = DowelHoleSpec::iter1();
        let transforms = build_dowel_hole_transforms(&body, &ribbon, &spec, bounds);
        for (i, t) in transforms.iter().enumerate() {
            let MatingTransform::SubtractCylinder { params } = t else {
                panic!("expected SubtractCylinder, got {t:?}");
            };
            let center = params.parent.center_m;
            // 3D body distance at the cylinder center.
            let body_dist = body.evaluate(&center);
            assert!(
                body_dist > 0.0,
                "dowel #{i} center {center:?} must be OUTSIDE body in 3D; \
                 got body_dist = {body_dist:.5} m"
            );
            // Tolerance: silhouette_outboard_offset ± grid_step. (3D
            // body distance can be ≤ 2D silhouette distance for body
            // with Y extent; for our flat cylinder cross-section at
            // Y=0 the two metrics agree.)
            let target = spec.silhouette_outboard_offset_m;
            let tol = 2.0 * SILHOUETTE_GRID_STEP_M;
            assert!(
                (body_dist - target).abs() < tol,
                "dowel #{i} center distance from body must be ≈ {target:.4} m \
                 (silhouette_outboard_offset_m); got {body_dist:.5} m \
                 (tolerance {tol:.4} m)"
            );
        }
    }

    #[test]
    fn dowel_holes_axis_aligned_with_ribbon_binormal() {
        let (body, bounds, ribbon) = cylinder_fixture();
        let spec = DowelHoleSpec::iter1();
        let transforms = build_dowel_hole_transforms(&body, &ribbon, &spec, bounds);
        let (_, seam_normal) = ribbon.seam_plane_reference();
        let binormal = seam_normal.into_inner();
        for t in &transforms {
            let MatingTransform::SubtractCylinder { params } = t else {
                continue;
            };
            // Dot product of dowel axis with binormal should be ±1.
            let dot = params.parent.axis.into_inner().dot(&binormal).abs();
            assert!(
                (dot - 1.0).abs() < 1e-9,
                "dowel axis must align with ribbon binormal; got dot = {dot}"
            );
        }
    }

    /// Cold-read regression 2026-05-27: dowel cylinder centers must
    /// land ON the binormal-aligned seam plane regardless of
    /// centerline tilt. Pre-fix the code put `center_m.y =
    /// seam_midpoint.y` (constant-Y plane), which for tilted
    /// centerlines placed the center off the actual seam plane along
    /// the binormal direction — making the dowel carve asymmetrically
    /// between the two cup-halves.
    ///
    /// Fixture: tilted centerline (~5.7° XY-plane tilt, similar to
    /// the §M-S1.5 flange-thickness test). Verify each dowel center
    /// satisfies `(center - seam_midpoint).dot(&binormal) ≈ 0`.
    #[test]
    fn dowel_hole_centers_lie_on_binormal_aligned_seam_plane_under_tilted_centerline() {
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
        let spec = DowelHoleSpec::iter1();
        let transforms = build_dowel_hole_transforms(&layer_body, &ribbon, &spec, bounds);
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
                "dowel #{i} center must lie ON the binormal-aligned seam \
                 plane; got binormal-distance = {dist_from_seam:.6} m \
                 (pre-fix this could be several mm for tilted centerlines, \
                 producing asymmetric hole carve across the two cup-halves)"
            );
        }
    }

    #[test]
    fn dowel_hole_radius_includes_clearance() {
        let (body, bounds, ribbon) = cylinder_fixture();
        let spec = DowelHoleSpec::iter1();
        let transforms = build_dowel_hole_transforms(&body, &ribbon, &spec, bounds);
        let expected_radius = spec.diameter_m / 2.0 + spec.clearance_m;
        for t in &transforms {
            let MatingTransform::SubtractCylinder { params } = t else {
                continue;
            };
            assert!(
                (params.radius_m - expected_radius).abs() < 1e-12,
                "dowel hole radius = diameter/2 + clearance = {expected_radius:.5} m; \
                 got {:.5} m",
                params.radius_m
            );
        }
    }
}
