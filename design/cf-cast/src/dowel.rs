//! Printable dowel-array STL emission (the loose PLA dowels that
//! insert into the [`crate::dowel_hole`] holes at workshop assembly).
//!
//! The cup-piece mating faces carry matching `SubtractCylinder` hole
//! pockets straddling the seam plane, placed by the seam-placement
//! solver (at the body's long-axis extremes; count emergent per
//! layer). This module emits the corresponding printable dowels:
//! cylindrical rods laid out side-by-side in a single STL so the
//! workshop user prints them once per cast regen and uses them across
//! all the layers' mold-piece pairs. The rod count is the `count`
//! argument to [`build_dowel_array_mesh`] — the count the seam-placement
//! solver actually placed per layer, threaded from
//! `spec::mesh_and_gate_v2_pieces` to `spec::mesh_and_gate_v2_dowel` so the
//! printed array matches the holes carved into the cups.
//!
//! # Why direct-mesh, not SDF→MC
//!
//! Built directly as `manifold3d::Manifold` cylinders via
//! [`crate::mesh_csg::build_cylinder_along_axis`] with the same
//! 32-segment facet count the dowel-HOLE `SubtractCylinder`
//! primitives use. Going through `Solid::from_sdf` + marching-cubes
//! would low-poly the cylinder (~6 cells radial at 0.5 mm cells
//! gives ~12 visible side facets, jagged); the analytic 32-segment
//! manifold gives ~0.2 mm chord error at 3 mm Ø — workshop-
//! imperceptible for slide-fit insertion. §M-S2 of
//! [[project-cf-cast-unified-mating-plane-recon]].
//!
//! # Geometry
//!
//! Each printable dowel:
//! - **Diameter**: `DowelHoleSpec.diameter_m` (default 3 mm). Matches
//!   the hole's nominal diameter exactly; the hole gets
//!   `DowelHoleSpec.clearance_m` radial clearance, the dowel does
//!   not — so the dowel is the SOLID 3 mm OD, hole is 3.2 mm ID,
//!   slide fit with 0.1 mm radial slack.
//! - **Length**: `2 × depth_m - 2 × DOWEL_INSERTION_SLACK_M` (default
//!   2×5 - 2×0.5 = 9 mm). Hole is 11 mm total (5 mm/half + 0.5 mm
//!   internal slack each side); dowel 9 mm leaves 1 mm slack at
//!   each tip after insertion, which absorbs FDM length tolerance
//!   without preventing the two cup-halves from mating flush.
//! - **Axis**: cast +Z (vertical in the cf-cast world frame).
//!   Workshop user prints them vertically — cylindrical rods print
//!   clean on FDM without overhangs.
//!
//! # Layout
//!
//! N dowels laid out along cast +X axis with center-to-center
//! spacing = `diameter_m + DOWEL_PRINT_SPACING_M` (default 3 + 4 =
//! 7 mm). All dowels sit on the build plate (`z = 0` is the bottom
//! face). spec.rs's export pipeline passes in an additional
//! `offset_x_m` to place the array alongside the bounding region
//! so it doesn't collide with mold pieces in cf-view assembly mode.

use mesh_types::IndexedMesh;
use nalgebra::{Point3, Unit, Vector3};

use crate::dowel_hole::DowelHoleSpec;
use crate::mesh_csg::{CylinderParent, build_cylinder_along_axis, manifold_to_indexed_mesh};

/// Axial slack at each dowel tip after assembly insertion (0.5 mm).
///
/// Public so `crate::procedure` can quote the assembly tip-slack
/// accurately in workshop-facing markdown (the total tip slack the
/// workshop user experiences is this PLUS
/// [`crate::dowel_hole::HOLE_AXIAL_SLACK_M`] — the hole has its own
/// internal axial slack).
pub const DOWEL_INSERTION_SLACK_M: f64 = 0.0005;

/// Center-to-center extra spacing between adjacent dowels (4 mm).
const DOWEL_PRINT_SPACING_M: f64 = 0.004;

/// Polygonal facet count around each dowel cylinder. 32 segments
/// match the dowel-HOLE `SubtractCylinder` segment count (per
/// [`crate::dowel_hole`]); both surfaces have the same chord error
/// so slide-fit clearance is geometrically uniform.
const DEFAULT_SEGMENTS: u32 = 32;

/// Build the printable dowel-array mesh.
///
/// Contains `count` analytic 32-segment cylinders laid out along +X
/// starting at `(offset_x_m, 0, 0)`. Each dowel's bottom face sits at
/// `z = 0` (build-plate ready); cylinder axis = +Z. The cylinder
/// geometry (Ø, length) comes from `spec`; only the *number* of rods
/// is the `count` argument.
///
/// Returns `None` if `count == 0`.
///
/// Bypasses `Solid::from_sdf` + marching-cubes entirely — directly
/// composes `manifold3d::Manifold` cylinders + unions them, then
/// converts to [`IndexedMesh`] in mm coords. This avoids MC's
/// low-poly quantization of small-radius cylinders.
#[must_use]
pub fn build_dowel_array_mesh(
    spec: &DowelHoleSpec,
    count: u32,
    offset_x_m: f64,
) -> Option<IndexedMesh> {
    if count == 0 {
        return None;
    }
    let radius_m = spec.diameter_m / 2.0;
    // dowel length = 2 × depth - 2 × insertion_slack
    let length_m = 2.0_f64.mul_add(-DOWEL_INSERTION_SLACK_M, 2.0 * spec.depth_m);
    let half_length_m = length_m / 2.0;
    let pitch = spec.diameter_m + DOWEL_PRINT_SPACING_M;
    let axis = Unit::new_normalize(Vector3::new(0.0, 0.0, 1.0));

    // Build cylinders 1..count individually + union into one Manifold.
    let mut combined: Option<manifold3d::Manifold> = None;
    for i in 0..count {
        let center_m = Point3::new(f64::from(i).mul_add(pitch, offset_x_m), 0.0, half_length_m);
        let parent = CylinderParent {
            center_m,
            axis,
            half_length_m,
        };
        let cyl = build_cylinder_along_axis(&parent, radius_m, DEFAULT_SEGMENTS);
        combined = Some(match combined {
            None => cyl,
            Some(prev) => prev.union(&cyl),
        });
    }
    combined.map(|m| manifold_to_indexed_mesh(&m))
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::float_cmp)]

    use super::*;

    fn mesh_bounds_mm(mesh: &IndexedMesh) -> (Point3<f64>, Point3<f64>) {
        let mut lo = Point3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
        let mut hi = Point3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
        for v in &mesh.vertices {
            lo.x = lo.x.min(v.x);
            lo.y = lo.y.min(v.y);
            lo.z = lo.z.min(v.z);
            hi.x = hi.x.max(v.x);
            hi.y = hi.y.max(v.y);
            hi.z = hi.z.max(v.z);
        }
        (lo, hi)
    }

    #[test]
    fn build_dowel_array_zero_count_returns_none() {
        let spec = DowelHoleSpec::iter1();
        assert!(build_dowel_array_mesh(&spec, 0, 0.0).is_none());
    }

    #[test]
    fn build_dowel_array_iter1_default_has_expected_extents_mm() {
        // 4 dowels of 3 mm diameter laid out along X with 7 mm pitch
        // (3 + 4 mm spacing). Dowel length = 2×5 - 2×0.5 = 9 mm.
        // X extent: first at X=0 (radius ±1.5), last at X = 3 × 7 =
        // 21 mm (radius ±1.5). Total X: [-1.5, +22.5] mm.
        // Y: ±1.5 mm. Z: [0, 9] mm.
        let spec = DowelHoleSpec::iter1();
        let mesh = build_dowel_array_mesh(&spec, 4, 0.0).unwrap();
        let (lo, hi) = mesh_bounds_mm(&mesh);
        // Tolerance: 0.2 mm chord error at 3 mm Ø for 32-segment cylinder.
        let tol = 0.2_f64;
        assert!(
            (lo.x + 1.5).abs() < tol,
            "first dowel min X ≈ -1.5 mm; got {:.3}",
            lo.x
        );
        assert!(
            (hi.x - 22.5).abs() < tol,
            "last dowel max X ≈ +22.5 mm; got {:.3}",
            hi.x
        );
        assert!(lo.z.abs() < 1e-6, "dowel bottom at z=0; got {:.6}", lo.z);
        assert!(
            (hi.z - 9.0).abs() < 1e-6,
            "dowel top at z=9 mm; got {:.6}",
            hi.z
        );
    }

    #[test]
    fn build_dowel_array_with_offset_x_translates_in_x() {
        let spec = DowelHoleSpec::iter1();
        let mesh = build_dowel_array_mesh(&spec, 1, 0.030).unwrap();
        let (lo, hi) = mesh_bounds_mm(&mesh);
        // Single dowel centered at offset_x = 30 mm, radius 1.5 mm:
        // X extent [28.5, 31.5] mm.
        let tol = 0.2;
        assert!(
            (lo.x - 28.5).abs() < tol,
            "single dowel min X ≈ 28.5 mm; got {:.3}",
            lo.x
        );
        assert!(
            (hi.x - 31.5).abs() < tol,
            "single dowel max X ≈ 31.5 mm; got {:.3}",
            hi.x
        );
    }

    #[test]
    fn build_dowel_array_is_32_segment_smooth() {
        // 4 cylinders × (32 sides × 2 tris + 30 cap tris × 2 caps)
        // ≈ 4 × (64 + 60) = 4 × 124 = 496 triangles before union dedup.
        // Union shouldn't change face count meaningfully when cylinders
        // are disjoint (no overlap). MC-baseline was ~3400 faces with
        // ugly 12-faceted cylinders — 32-segment is way smoother + 7×
        // smaller mesh.
        let spec = DowelHoleSpec::iter1();
        let mesh = build_dowel_array_mesh(&spec, 4, 0.0).unwrap();
        assert!(
            mesh.faces.len() < 1000,
            "32-segment cylinder array should have < 1000 faces (MC \
             baseline was ~3400); got {}",
            mesh.faces.len()
        );
    }
}
