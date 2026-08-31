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
use crate::mesh_csg::{
    CylinderParent, build_chamfered_cylinder_along_axis, manifold_to_indexed_mesh,
};

/// Axial slack at each dowel tip after assembly insertion (0.5 mm).
///
/// Public so `crate::procedure` can quote the assembly tip-slack
/// accurately in workshop-facing markdown (the total tip slack the
/// workshop user experiences is this PLUS
/// [`crate::dowel_hole::HOLE_AXIAL_SLACK_M`] — the hole has its own
/// internal axial slack).
pub const DOWEL_INSERTION_SLACK_M: f64 = 0.0005;

/// Lead-in chamfer at each dowel tip (0.4 mm, 45°).
///
/// ⚠ Added 2026-08-31 after the workshop reported needing a CLAMP to seat a
/// dowel — a fail of recon §G-6's acceptance criterion (a), "seat without
/// forcing". The dowel was the only mating feature in this crate with
/// square-cut ends; the plug-floor lock has carried a chamfer described as a
/// "lead-in self-centering aid" since §G-6.
///
/// Under the mating-face-DOWN print lock (the hard flat-mating-face
/// constraint) the hole mouth sits on the bed, so first-layer squish narrows
/// precisely the entry a square-cut pin must find. A chamfer bridges that.
///
/// **0.4 mm is the FLOOR of the §G-6 typed chamfer range (0.4-0.8 mm)** —
/// chosen to buy the lead-in at the smallest cost in bearing length, which
/// drops from 4.5 mm to 4.1 mm per half. If the workshop still has to force a
/// dowel, the next move is up this range, NOT out to a looser radial
/// clearance: clearance trades away lateral registration, chamfer does not.
pub const DOWEL_TIP_CHAMFER_M: f64 = 0.0004;

/// Center-to-center extra spacing between adjacent dowels (4 mm).
const DOWEL_PRINT_SPACING_M: f64 = 0.004;

/// Polygonal facet count around each dowel cylinder. 32 segments
/// match the dowel-HOLE `SubtractCylinder` segment count (per
/// [`crate::dowel_hole`]); both surfaces have the same chord error
/// so slide-fit clearance is geometrically uniform.
const DEFAULT_SEGMENTS: u32 = 32;

/// The dowel's shank radius and half-length, derived from `spec`.
///
/// ★★★ **THE ONE DERIVATION.** [`build_dowel_array_mesh`] and
/// [`effective_tip_chamfer_m`] both call this. They previously each computed
/// it, with a comment claiming they "cannot drift" — a mirror, and mirrors
/// drift; nothing enforced it. Extracted so the claim is structural rather
/// than aspirational.
fn dowel_geometry_m(spec: &DowelHoleSpec) -> (f64, f64) {
    (spec.diameter_m / 2.0, length_m(spec) / 2.0)
}

/// The printed dowel's overall length — `2 × depth - 2 × insertion slack`.
///
/// ★★★ **THE ONE DERIVATION.** The mesh builder and the workshop prose both
/// call this; the prose used to re-derive it from `depth_m` and
/// [`DOWEL_INSERTION_SLACK_M`], so a change to how a dowel is sized would have
/// left the sheet quoting a length no dowel has.
#[must_use]
pub fn length_m(spec: &DowelHoleSpec) -> f64 {
    2.0_f64.mul_add(-DOWEL_INSERTION_SLACK_M, 2.0 * spec.depth_m)
}

/// The lead-in chamfer a dowel built from `spec` will ACTUALLY carry.
///
/// ★★★ `crate::procedure` MUST use this, not [`DOWEL_TIP_CHAMFER_M`] directly.
/// The constant is a REQUEST; the builder clamps it to the geometry that
/// exists (see [`crate::mesh_csg::effective_tip_chamfer_m`]). At iter-1
/// defaults the two agree, which is exactly why a prose site that re-derived
/// from the constant looked correct — on a sub-millimetre dowel it would have
/// stated a NEGATIVE tip diameter for a mesh whose tip is positive.
#[must_use]
pub fn effective_tip_chamfer_m(spec: &DowelHoleSpec) -> f64 {
    let (radius_m, half_length_m) = dowel_geometry_m(spec);
    crate::mesh_csg::effective_tip_chamfer_m(DOWEL_TIP_CHAMFER_M, radius_m, half_length_m)
}

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
    let (radius_m, half_length_m) = dowel_geometry_m(spec);
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
        let cyl = build_chamfered_cylinder_along_axis(
            &parent,
            radius_m,
            DOWEL_TIP_CHAMFER_M,
            DEFAULT_SEGMENTS,
        );
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
        // Each dowel is the convex hull of FOUR 32-point rings (tip,
        // chamfer shoulder, chamfer shoulder, tip) = 128 points, which
        // triangulates to ~252 faces; 4 dowels ≈ 1008.
        //
        // ⚠ This budget was `< 1000` while the dowel was a plain cylinder
        // (two rings, ~124 faces each). Adding the 2026-08-31 tip chamfer
        // doubled the rings and tripped it at 1008. Raised deliberately with
        // the new arithmetic shown — still ~3× under the ~3400-face MC
        // baseline this test was written to protect.
        let spec = DowelHoleSpec::iter1();
        let mesh = build_dowel_array_mesh(&spec, 4, 0.0).unwrap();
        assert!(
            mesh.faces.len() < 1200,
            "4 chamfered 32-segment dowels should stay under 1200 faces (MC \
             baseline was ~3400); got {}",
            mesh.faces.len()
        );
    }

    /// ★★★ The sheet and the mesh must state the SAME tip. This is the #850
    /// class a fourth time — and the first three were all "prose describes what
    /// was requested, not what is built", so the gate has to compare the two
    /// SOURCES, not re-run one of them.
    #[test]
    fn the_prose_chamfer_matches_the_chamfer_the_mesh_actually_carries() {
        for (label, spec) in [
            ("iter1", DowelHoleSpec::iter1()),
            // ⚠ The clamp has TWO bounds and they must be covered
            // SEPARATELY. This case binds on HALF-LENGTH (0.040 mm beats the
            // 0.120 mm radius bound)...
            (
                "short dowel — half-length binds",
                DowelHoleSpec {
                    diameter_m: 0.0006,
                    depth_m: 0.0006,
                    ..DowelHoleSpec::iter1()
                },
            ),
            // ...and this one binds on RADIUS (0.120 mm beats the 1.800 mm
            // half-length bound). Deleting the `radius_m * 0.4` term left the
            // whole suite green until this case existed — the guard was
            // vacuous on the bound it was written for.
            (
                "thin dowel — radius binds",
                DowelHoleSpec {
                    diameter_m: 0.0006,
                    depth_m: 0.005,
                    ..DowelHoleSpec::iter1()
                },
            ),
        ] {
            let quoted_mm = effective_tip_chamfer_m(&spec) * 1000.0;
            let mesh = build_dowel_array_mesh(&spec, 1, 0.0).unwrap();

            let r_mm = spec.diameter_m / 2.0 * 1000.0;
            let tip_mm = mesh
                .vertices
                .iter()
                .filter(|v| v[2].abs() < 1e-6)
                .map(|v| v[0].hypot(v[1]))
                .fold(0.0_f64, f64::max);
            let built_mm = r_mm - tip_mm;

            assert!(
                (quoted_mm - built_mm).abs() < 1e-6,
                "[{label}] the sheet would quote a {quoted_mm} mm chamfer but \
                 the mesh carries {built_mm} mm — a tip Ø of \
                 {} vs {}",
                2.0f64.mul_add(-quoted_mm, r_mm * 2.0),
                tip_mm * 2.0
            );
            assert!(
                quoted_mm > 0.0,
                "[{label}] a quoted chamfer of {quoted_mm} mm would render a \
                 negative tip diameter in the sheet"
            );
        }
    }

    /// ⚠⚠ `[dowel_hole].diameter_m` and `.depth_m` are user-settable from
    /// `cast.toml`, so a small dowel can be smaller than the fixed 0.4 mm
    /// chamfer. An earlier revision `assert!`ed against that — a panic
    /// reachable from a config file, on a path that took any positive radius
    /// before the chamfer existed. The chamfer is clamped instead.
    #[test]
    fn a_dowel_smaller_than_the_chamfer_still_builds() {
        // 0.6 mm Ø → 0.3 mm radius, well under the 0.4 mm chamfer.
        let tiny = DowelHoleSpec {
            diameter_m: 0.0006,
            depth_m: 0.0006,
            ..DowelHoleSpec::iter1()
        };
        // ⚠ ONE dowel: the array lays additional dowels out along +X at
        // `diameter + 4 mm` pitch, so a radius measured from the origin would
        // pick up the neighbour rather than this dowel's own shank.
        // A panic anywhere in this call IS the regression this test exists for.
        let mesh = build_dowel_array_mesh(&tiny, 1, 0.0).unwrap();
        assert!(
            !mesh.faces.is_empty(),
            "clamped-chamfer dowel produced an empty mesh"
        );
        // And the clamp must leave real bearing surface, not a bare cone.
        let r_mm = tiny.diameter_m / 2.0 * 1000.0;
        let widest = mesh
            .vertices
            .iter()
            .map(|v| v[0].hypot(v[1]))
            .fold(0.0_f64, f64::max);
        assert!(
            (widest - r_mm).abs() < 1e-6,
            "the shank should still reach the full {r_mm} mm radius; got {widest} mm"
        );
    }

    /// ★ The face-count budget above would stay green if the chamfer silently
    /// vanished only on ONE ring, so assert the lead-in geometrically: the
    /// extreme-Z ring must be NARROWER than the shank by exactly the chamfer.
    #[test]
    fn each_dowel_tip_is_chamfered_not_square_cut() {
        let spec = DowelHoleSpec::iter1();
        let mesh = build_dowel_array_mesh(&spec, 1, 0.0).unwrap();

        let r_mm = spec.diameter_m / 2.0 * 1000.0;
        let c_mm = DOWEL_TIP_CHAMFER_M * 1000.0;

        // Vertices are mm; the dowel spans z = 0 .. length.
        let radius_at = |z_target: f64| -> f64 {
            mesh.vertices
                .iter()
                .filter(|v| (v[2] - z_target).abs() < 1e-6)
                .map(|v| v[0].hypot(v[1]))
                .fold(0.0_f64, f64::max)
        };

        let tip = radius_at(0.0);
        let shoulder = radius_at(c_mm);
        assert!(
            (tip - (r_mm - c_mm)).abs() < 1e-6,
            "tip ring should be inset by the {c_mm} mm chamfer to \
             {} mm, got {tip} mm — a square-cut dowel is what made the \
             workshop reach for a clamp",
            r_mm - c_mm
        );
        assert!(
            (shoulder - r_mm).abs() < 1e-6,
            "chamfer shoulder should be at the full {r_mm} mm radius, got \
             {shoulder} mm"
        );
        assert!(
            tip < shoulder,
            "tip {tip} mm must be narrower than shoulder {shoulder} mm or \
             there is no lead-in at all"
        );
    }
}
