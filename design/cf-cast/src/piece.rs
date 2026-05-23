//! Per-piece SDF composition for v2 curve-following multi-piece molds.
//!
//! Step 5 of `docs/CURVE_FOLLOWING_DESIGN.md` §Algorithm §"Per-piece
//! SDF construction": for each layer × piece pair the mold-piece
//! geometry is the CSG composition
//!
//! ```text
//! piece_sdf      = bounding_region ∖ layer_body ∩ ribbon_side(side)
//! piece_meshcsg  = [plug-shaft + T-slot SubtractCylinder]
//!                + [pour-gate + vent SubtractCylinder]
//!                + [registration pin/socket UnionCylinder/SubtractCylinder]
//! ```
//!
//! The cup-piece [`Solid`] is **side-specific**: each
//! [`PieceSide`] returns its own half-shell via
//! [`Ribbon::halfspace_solid`]'s SDF half-space intersect (biased
//! inward by [`RIBBON_PIECE_OVERLAP_M`] so the two pieces overlap by
//! 1 mm at the seam). Marching-cubes meshes the half-shell directly;
//! the mating-features cylinders (S5/S6/S7) compose post-MC via
//! [`apply_mating_transforms`]. See
//! `docs/CF_CAST_SEAM_FACE_FILM_RECON_PLAN.md` §F-2 for the
//! recon-4 (P) architectural-correction rationale: the seam belongs
//! to the SDF/MC paradigm (continuous bulk geometry); mating-feature
//! cylinders belong to the mesh-CSG paradigm (exact discrete
//! primitives with bit-precise diametral fit).
//!
//! # v1 vs v2 mold cup
//!
//! v1's [`crate::CastSpec::export_molds`] carved the cup as
//! `bounding_region ∖ layer_body ∖ clip_above(layer_body)` — the
//! `clip_above` cuboid opened the cup at the `+Z` end for
//! straight-pull demolding. v2 replaces the straight `+Z` clip with
//! the ribbon-side intersection: the mold pieces separate along the
//! ribbon's curved surface, so the cup is closed at every face and
//! only opens where the pieces themselves come apart. No `+Z` axis
//! is privileged in v2.
//!
//! # Inter-piece seam overlap
//!
//! [`RIBBON_PIECE_OVERLAP_M`] biases each piece's half-space
//! inward by 0.5 mm so the two pieces' geometry overlaps by 1 mm at
//! the ribbon seam. v1 uses the same magnitude
//! (`CLIP_BODY_OVERLAP_M`) to break the coincidence between the
//! body's top face and the clip's bottom face; v2 applies it at the
//! ribbon ∩ mold-cup intersection per
//! `docs/CURVE_FOLLOWING_DESIGN.md` §Risks §"CSG numerical issues at
//! the ribbon ∩ `mold_cup` intersection". Both halves overlapping
//! also gives the workshop user mechanical tolerance during fit-up
//! (FDM accuracy is ~0.1 mm, well below the 1 mm seam).
//!
//! # Architectural history
//!
//! S4 of `docs/CF_CAST_MATING_FEATURES_PLAN.md` (2026-05-22)
//! migrated the seam from the SDF half-space intersect described
//! above to a post-MC mesh-trim against an exact plane via
//! `MatingTransform::SeamTrim`, making the cup-piece [`Solid`]
//! side-agnostic. Workshop iter-1 cf-view smoke surfaced an
//! orthogonal seam-face film bug (a thin membrane of cup-wall
//! material capping the body-cavity opening at the seam plane) that
//! bisected to S4 `93aaa0c2`. Recon-4 (P) (2026-05-23) reverted the
//! seam to the SDF form per `docs/CF_CAST_SEAM_FACE_FILM_RECON_PLAN.md`
//! §F-2: the body-cavity opening at the seam is established by SDF
//! arithmetic (`bounding ∖ body ∩ halfspace` evaluates linearly at
//! the seam plane), so MC interpolates seam-cap vertices exactly on
//! the plane (§F-4 audit: bit-precise to f64 precision). S5/S6/S7
//! mating-features mesh-CSG stays unchanged — `apply_mating_transforms`
//! still runs cylinder Union/Subtract ops post-MC against the
//! half-shell mesh. [`crate::mesh_csg::MatingTransform::SeamTrim`]
//! remains in the enum as a defensive primitive but is no longer
//! emitted from this function.

use cf_design::Solid;

use crate::error::{CastError, CastTarget};
use crate::mesh_csg::MatingTransform;
use crate::plug::build_plug_socket_transforms;
use crate::pour::build_pour_gate_transforms;
use crate::registration::build_registration_transforms;
use crate::ribbon::{PieceSide, Ribbon};

/// Inter-piece seam overlap distance applied at the
/// ribbon ∩ mold-cup intersection.
///
/// Each [`PieceSide`] half-space grows by this distance toward the
/// other side, so the two pieces overlap by
/// `2 * RIBBON_PIECE_OVERLAP_M = 1 mm` at the seam.
///
/// 0.5 mm matches v1's `CLIP_BODY_OVERLAP_M` — well above FDM layer
/// height (typically 0.2 mm) so workshop printers tolerate it, well
/// below the 2 mm cell-size cf-cast targets so marching-cubes never
/// resolves the bias as a separate surface. Single source of truth
/// for the v2 per-piece composition; not user-tunable until iter-1
/// surfaces a specific need.
pub const RIBBON_PIECE_OVERLAP_M: f64 = 0.0005;

/// Compose the per-(layer × piece) mold-piece geometry as the pair
/// `(Solid, Vec<MatingTransform>)`:
///
/// - **Solid** = `bounding_region ∖ layer_body ∩ ribbon.halfspace_solid(side, ...)`
///   — side-specific half-shell. Each [`PieceSide`] returns its own
///   biased half-space intersect ([`RIBBON_PIECE_OVERLAP_M`] grows
///   each side 0.5 mm into the other so the two pieces overlap by
///   1 mm at the seam).
/// - **`Vec<MatingTransform>`** =
///   1. Plug-shaft socket + T-slot
///      [`crate::mesh_csg::MatingTransform::SubtractCylinder`] ops
///      (S6).
///   2. Pour-gate + vent
///      [`crate::mesh_csg::MatingTransform::SubtractCylinder`] ops
///      (S7). The full-length cylinder is subtracted from the
///      half-shell; the SDF halfspace handles per-side bisection by
///      construction (cylinder portion outside the half-shell is a
///      no-op).
///   3. Per-pin registration ops — one
///      [`crate::mesh_csg::MatingTransform::UnionCylinder`] per pin
///      on [`PieceSide::Negative`]; one
///      [`crate::mesh_csg::MatingTransform::SubtractCylinder`] per
///      pin on [`PieceSide::Positive`] (S5).
///
/// The cup-piece Solid's SDF is **negative inside the cup-wall
/// material** on the half-shell side; the side-specific cut is
/// already baked into the Solid via the ribbon's half-space
/// intersect. `apply_mating_transforms` then runs the cylinder ops
/// post-MC against the half-shell mesh; cylinder portions outside
/// the half-shell volume contribute nothing (a no-op for
/// `SubtractCylinder`; a free-floating addition for `UnionCylinder`
/// — which under the binormal-axis pin design materializes as the
/// workshop-visible half-cylinder ridge across the seam face).
///
/// Recon-4 (P) reverted the seam cut from a post-MC
/// [`crate::mesh_csg::MatingTransform::SeamTrim`] back to this SDF
/// half-space intersect; see
/// `docs/CF_CAST_SEAM_FACE_FILM_RECON_PLAN.md` §F-2 + the module
/// docstring for the architectural-correction rationale.
/// [`crate::mesh_csg::MatingTransform::SeamTrim`] remains in the
/// enum as a defensive primitive (its `apply_one` arm still runs)
/// but is no longer emitted here.
///
/// `bounding_region` and `layer_body` are taken by reference and
/// cloned into the result; the underlying [`cf_design::Solid`] is
/// internally `Arc`-shared so cloning is cheap (no geometry
/// duplication).
///
/// # Errors
///
/// - [`CastError::InfiniteBounds`] with [`CastTarget::BoundingRegion`]
///   if `bounding_region.bounds()` returns `None` — the ribbon
///   half-space derives its AABB from the bounding region and the
///   downstream MC mesher needs a finite extent.
///
/// An unbounded `layer_body` (e.g., a [`cf_design::Solid::plane`])
/// is **not** an error: the final piece Solid's bounds come from
/// `bounding_region` (the minuend), and the body's SDF is
/// well-defined everywhere inside that region. v1's
/// `clip_above_body` rejected unbounded bodies because it needed
/// `body.z_max` to position the straight-pull clip cuboid; v2's
/// composition has no such requirement.
pub fn compose_piece_solid(
    layer_body: &Solid,
    bounding_region: &Solid,
    ribbon: &Ribbon,
    side: PieceSide,
) -> Result<(Solid, Vec<MatingTransform>), CastError> {
    let bounds = bounding_region
        .bounds()
        .ok_or(CastError::InfiniteBounds(CastTarget::BoundingRegion))?;
    // Pre-S4 / recon-4 (P) SDF seam: side-specific half-space
    // intersect at the SDF level. The biased halfspace's SDF is
    // exactly linear; MC interpolates seam-cap vertices on the seam
    // plane to f64 precision per §F-4. S5/S6/S7 mating-features
    // mesh-CSG cylinder primitives compose post-MC against the
    // resulting half-shell mesh.
    let halfspace = ribbon.halfspace_solid(side, bounds, RIBBON_PIECE_OVERLAP_M);
    let base_piece = bounding_region
        .clone()
        .subtract(layer_body.clone())
        .intersect(halfspace);

    // S6 plug-anchor socket + T-slot SubtractCylinders.
    let mut transforms = build_plug_socket_transforms(ribbon);
    // S7 pour-gate + air-vent SubtractCylinders.
    transforms.extend(build_pour_gate_transforms(ribbon));
    // S5 inter-piece registration pins/sockets. Negative emits
    // UnionCylinder per pin; Positive emits SubtractCylinder per
    // pin (inflated by diametral + axial clearance). Empty Vec
    // when `ribbon.registration` is `None`.
    transforms.extend(build_registration_transforms(
        ribbon,
        layer_body,
        bounding_region,
        side,
    ));
    Ok((base_piece, transforms))
}

#[cfg(test)]
mod tests {
    // Workspace lint policy allows unwrap/panic in tests for clean
    // assertion-failure prose; matches `spec.rs` + `ribbon.rs`.
    #![allow(clippy::unwrap_used, clippy::panic, clippy::expect_used)]

    use super::*;
    use crate::ribbon::SplitNormal;
    use cf_design::Solid;
    use nalgebra::{Point3, Vector3};

    /// Standard v2 piece test fixture:
    /// - layer body: 20 × 20 × 20 mm cuboid at origin
    /// - bounding region: 60 × 60 × 60 mm cuboid at origin (so the
    ///   cup wall is 20 mm thick on every face)
    /// - ribbon: 10 cm centerline along +X passing through origin
    ///   (so the body's centerline lies on it); +Y split-normal so
    ///   the binormal is +Z and the ribbon cuts at z = 0
    fn fixture() -> (Solid, Solid, Ribbon) {
        let layer_body = Solid::cuboid(Vector3::new(0.010, 0.010, 0.010));
        let bounding_region = Solid::cuboid(Vector3::new(0.030, 0.030, 0.030));
        let centerline = vec![Point3::new(-0.05, 0.0, 0.0), Point3::new(0.05, 0.0, 0.0)];
        let split = SplitNormal::new(Vector3::new(0.0, 1.0, 0.0)).unwrap();
        let ribbon = Ribbon::new(centerline, split).unwrap();
        (layer_body, bounding_region, ribbon)
    }

    /// Negative-side piece: covers the cup wall on the `-Z` side of
    /// the ribbon (binormal `+Z`, sign `+1` → SDF negative where
    /// `ribbon.sdf < +bias`, i.e., at and below the body's bottom).
    /// A query point in the cup wall below the body must be INSIDE
    /// (SDF < 0). Post-(P) the cup-piece [`Solid`] is side-specific
    /// again — the half-space intersect lives at the SDF level.
    #[test]
    fn negative_piece_contains_cup_wall_below_ribbon() {
        let (body, region, ribbon) = fixture();
        let (piece, _) = compose_piece_solid(&body, &region, &ribbon, PieceSide::Negative).unwrap();
        // Inside bounding region (-30 ≤ z ≤ +30 mm), outside body
        // (body z ∈ [-10, +10]), below ribbon (z < 0):
        let q = Point3::new(0.0, 0.0, -0.020);
        assert!(
            piece.evaluate(&q) < 0.0,
            "negative-side piece must contain cup-wall point at z = -20 mm; got {}",
            piece.evaluate(&q),
        );
    }

    /// Same negative-side piece: a cup-wall query ABOVE the ribbon
    /// must be OUTSIDE (the body's top hemisphere belongs to the
    /// positive piece). Restored from the pre-S4 form by recon-4
    /// (P) — the side-specific Solid distinguishes the two halves
    /// at the SDF level.
    #[test]
    fn negative_piece_excludes_cup_wall_above_ribbon() {
        let (body, region, ribbon) = fixture();
        let (piece, _) = compose_piece_solid(&body, &region, &ribbon, PieceSide::Negative).unwrap();
        let q = Point3::new(0.0, 0.0, 0.020);
        assert!(
            piece.evaluate(&q) > 0.0,
            "negative-side piece must exclude cup-wall point at z = +20 mm; got {}",
            piece.evaluate(&q),
        );
    }

    /// Body interior: every point inside the layer body must be
    /// OUTSIDE the piece (the body has been subtracted out).
    #[test]
    fn neither_piece_contains_layer_body_interior() {
        let (body, region, ribbon) = fixture();
        // A point well inside the body but offset to one side of the
        // ribbon — without the body subtraction the negative piece
        // would CONTAIN this point.
        let q = Point3::new(0.0, 0.0, -0.005);
        let (neg, _) = compose_piece_solid(&body, &region, &ribbon, PieceSide::Negative).unwrap();
        let (pos, _) = compose_piece_solid(&body, &region, &ribbon, PieceSide::Positive).unwrap();
        assert!(
            neg.evaluate(&q) > 0.0,
            "negative piece must not contain body interior; got {}",
            neg.evaluate(&q),
        );
        assert!(
            pos.evaluate(&q) > 0.0,
            "positive piece must not contain body interior; got {}",
            pos.evaluate(&q),
        );
    }

    /// Outside the bounding region: both pieces must report outside
    /// (the `bounding_region` minuend pins the outer envelope).
    #[test]
    fn neither_piece_contains_points_outside_bounding_region() {
        let (body, region, ribbon) = fixture();
        // Far outside the 30 mm half-extent bounding region.
        let q = Point3::new(0.10, 0.0, 0.0);
        let (neg, _) = compose_piece_solid(&body, &region, &ribbon, PieceSide::Negative).unwrap();
        let (pos, _) = compose_piece_solid(&body, &region, &ribbon, PieceSide::Positive).unwrap();
        assert!(
            neg.evaluate(&q) > 0.0,
            "neg piece must exclude far-out point"
        );
        assert!(
            pos.evaluate(&q) > 0.0,
            "pos piece must exclude far-out point"
        );
    }

    /// Negative and Positive pieces partition the cup material:
    /// at a cup-wall point on the `-Z` side, the negative piece is
    /// inside (SDF < 0) and the positive piece is outside (SDF > 0),
    /// with opposite signs at the symmetric `+Z` point. Restored
    /// from the pre-S4 form by recon-4 (P) — side-specific Solid
    /// makes the partition observable at the SDF layer.
    #[test]
    fn pieces_partition_cup_material_symmetrically() {
        let (body, region, ribbon) = fixture();
        let (neg, _) = compose_piece_solid(&body, &region, &ribbon, PieceSide::Negative).unwrap();
        let (pos, _) = compose_piece_solid(&body, &region, &ribbon, PieceSide::Positive).unwrap();

        let q_below = Point3::new(0.0, 0.0, -0.020);
        let q_above = Point3::new(0.0, 0.0, 0.020);

        // Below: in negative piece, out of positive piece.
        assert!(neg.evaluate(&q_below) < 0.0);
        assert!(pos.evaluate(&q_below) > 0.0);
        // Above: out of negative, in positive.
        assert!(neg.evaluate(&q_above) > 0.0);
        assert!(pos.evaluate(&q_above) < 0.0);
    }

    /// At the ribbon seam (z = 0 inside the cup wall), BOTH pieces
    /// contain the point — this is the design-doc seam overlap that
    /// keeps marching cubes from resolving coincident surfaces.
    /// `RIBBON_PIECE_OVERLAP_M = 0.5 mm` gives each piece an SDF of
    /// `-0.5 mm` along the seam (by symmetry of the bias). Restored
    /// from the pre-S4 form by recon-4 (P) — the SDF half-space
    /// intersect carries the overlap directly.
    #[test]
    fn both_pieces_overlap_at_ribbon_seam() {
        let (body, region, ribbon) = fixture();
        let (neg, _) = compose_piece_solid(&body, &region, &ribbon, PieceSide::Negative).unwrap();
        let (pos, _) = compose_piece_solid(&body, &region, &ribbon, PieceSide::Positive).unwrap();
        // Cup wall on the ribbon seam: y outside body, x within bounding,
        // z = 0 (the ribbon's zero plane). y = 0.020 is outside body
        // (10 mm half-extent) and inside bounding region (30 mm).
        let q_seam = Point3::new(0.0, 0.020, 0.0);
        // The body-subtract dominates the piece SDF outside the
        // bias-overlap region; at z=0 just outside the body, the
        // governing constraint is the half-space bias, so the SDF
        // should be exactly `-RIBBON_PIECE_OVERLAP_M`.
        assert!(neg.evaluate(&q_seam) < 0.0);
        assert!(pos.evaluate(&q_seam) < 0.0);
        // Both report the same SDF magnitude on the seam (by
        // symmetry of the bias).
        let neg_sdf = neg.evaluate(&q_seam);
        let pos_sdf = pos.evaluate(&q_seam);
        assert!(
            (neg_sdf - pos_sdf).abs() < 1e-12,
            "both pieces' SDFs at the seam should match by symmetry; got neg={neg_sdf} pos={pos_sdf}",
        );
    }

    /// Mesh-level test fixture pipeline: compose Solid → MC →
    /// post-CSG mating-features. Returns the post-CSG `IndexedMesh`
    /// in mm world coords, exactly the geometry that flows to F4 +
    /// STL-export in the live `mesh_and_gate_v2_*` sites.
    fn mesh_piece_through_p_pipeline(
        body: &Solid,
        bounding: &Solid,
        ribbon: &Ribbon,
        side: PieceSide,
        cell_size_m: f64,
    ) -> mesh_types::IndexedMesh {
        use crate::mesh_csg::apply_mating_transforms;
        use crate::mesher::solid_to_mm_mesh;
        let target = CastTarget::MoldPiece {
            layer_index: 0,
            piece_side: side,
        };
        let (solid, transforms) = compose_piece_solid(body, bounding, ribbon, side).unwrap();
        let mesh = solid_to_mm_mesh(&solid, cell_size_m, target).expect("MC must succeed");
        apply_mating_transforms(mesh, &transforms, target).expect("CSG must succeed")
    }

    /// Recon-4 (P) §F-4 falsification gate, promoted from the
    /// synthetic `mesh_csg::tests::f4_synthetic_presdf_seam_is_bit_precise_flat`
    /// probe to a production-fixture gate (mirrors the S4-era
    /// `s4_mating_face_is_mathematically_flat_and_coplanar` test
    /// structure on the pre-S4 SDF form).
    ///
    /// The SDF formula `bounding ∖ body ∩ halfspace` evaluates to
    /// the halfspace SDF (signed distance to a plane) at the seam
    /// plane: bounding interior negative; body exterior positive;
    /// the intersect's max-operator surfaces the halfspace's
    /// exactly-zero value. MC interpolates linearly between adjacent
    /// cell-corner SDF values, so for a linear SDF the interpolated
    /// vertex lands ON the SDF = 0 surface (to f64 precision modulo
    /// gradient + max-operator noise). The seam plane is at
    /// `signed_dist = 0`; each side's cap-vertex extreme along the
    /// kept-side normal lies at `signed_dist = ±RIBBON_PIECE_OVERLAP_M`
    /// (the inward bias from `halfspace_solid`'s `overlap_m`
    /// argument).
    ///
    /// Verifies the three pre-S4 form invariants:
    ///
    /// 1. Each piece's seam-cap vertex extreme is at
    ///    `signed_dist = ±RIBBON_PIECE_OVERLAP_M` to within 1 µm
    ///    (the synthetic §F-4 probe hit 0.000000 mm; production
    ///    curved geometry may add small noise — kept at 1 µm
    ///    strict, with the 10 µm relaxed fallback documented in
    ///    `docs/CF_CAST_SEAM_FACE_FILM_RECON_PLAN.md` §F-5 #2).
    /// 2. The two pieces' cap planes are co-planar with offsets
    ///    differing by `2 × RIBBON_PIECE_OVERLAP_M`.
    /// 3. No vertex escapes the kept half-space.
    ///
    /// Locks the §F-4 architectural-claim audit (pre-S4 SDF seam is
    /// bit-precise flat, contrary to S4's implicit premise that
    /// pre-S4 was NOT flat). Future seam-mechanism migrations can't
    /// silently regress.
    #[test]
    fn mating_face_is_mathematically_flat_and_coplanar() {
        let (body, region, ribbon) = fixture();
        // For the fixture, binormal = +Z and midpoint = origin, so
        // the seam plane is z = 0, Negative cap at z = +0.5 mm,
        // Positive cap at z = -0.5 mm.
        let (midpoint, binormal) = ribbon.seam_plane_reference();
        let normal_v = binormal.into_inner();
        let plane_d_mm = midpoint.coords.dot(&normal_v) * 1000.0;
        let overlap_mm = RIBBON_PIECE_OVERLAP_M * 1000.0;

        let neg =
            mesh_piece_through_p_pipeline(&body, &region, &ribbon, PieceSide::Negative, 0.005);
        let pos =
            mesh_piece_through_p_pipeline(&body, &region, &ribbon, PieceSide::Positive, 0.005);

        let signed_dist = |v: &Point3<f64>| -> f64 { v.coords.dot(&normal_v) - plane_d_mm };

        // (1) Negative kept where signed_dist < +overlap. Cap
        // vertices sit AT signed_dist = +overlap by MC's linear-SDF
        // interpolation property.
        let neg_max = neg
            .vertices
            .iter()
            .map(signed_dist)
            .fold(f64::MIN, f64::max);
        let neg_min = neg
            .vertices
            .iter()
            .map(signed_dist)
            .fold(f64::MAX, f64::min);
        let cap_tol_mm = 1.0e-3; // 1 µm — §F-4 strict gate
        assert!(
            (neg_max - overlap_mm).abs() < cap_tol_mm,
            "Negative cap vertices must lie on signed_dist = +{overlap_mm:.4} mm \
             (pre-S4 SDF seam is bit-precise flat by MC's linear-SDF interpolation, \
             per recon-4 §F-4); got max = {neg_max:.6} mm",
        );
        assert!(
            neg_min >= -50.0,
            "Negative should still have far-side material (bounding floor); got min = {neg_min:.3} mm",
        );

        // (2) Positive: mirror.
        let pos_max = pos
            .vertices
            .iter()
            .map(signed_dist)
            .fold(f64::MIN, f64::max);
        let pos_min = pos
            .vertices
            .iter()
            .map(signed_dist)
            .fold(f64::MAX, f64::min);
        assert!(
            (pos_min - -overlap_mm).abs() < cap_tol_mm,
            "Positive cap vertices must lie on signed_dist = -{overlap_mm:.4} mm; \
             got min = {pos_min:.6} mm",
        );
        assert!(
            pos_max <= 50.0,
            "Positive should still have far-side material; got max = {pos_max:.3} mm",
        );

        // (3) No vertex past the cap (the SDF halfspace intersect
        // bounds the kept-side mesh to `signed_dist ≤ +overlap` for
        // Negative and `signed_dist ≥ -overlap` for Positive).
        assert!(
            neg_max <= overlap_mm + cap_tol_mm,
            "Negative mesh must not extend past the cap",
        );
        assert!(
            pos_min >= -overlap_mm - cap_tol_mm,
            "Positive mesh must not extend past the cap",
        );

        // (4) Co-planarity across pieces: cap offsets differ by
        // exactly `2 × RIBBON_PIECE_OVERLAP_M`.
        let cap_offset_difference_mm = neg_max - pos_min;
        let expected_difference_mm = 2.0 * overlap_mm;
        assert!(
            (cap_offset_difference_mm - expected_difference_mm).abs() < cap_tol_mm,
            "Co-planarity gate: Negative.cap.signed_dist - Positive.cap.signed_dist \
             must equal 2 × overlap = {expected_difference_mm:.4} mm; \
             got {cap_offset_difference_mm:.6} mm",
        );
    }

    /// §F-4 flatness gate under a curved centerline — locks the
    /// contract for the production iter-1 ribbon shape (curved-
    /// centerline + ribbon binormal numerics introduce small
    /// deviations from the synthetic axis-aligned case). The
    /// `Ribbon::halfspace_solid` SDF samples the ribbon's binormal
    /// per-segment, so curvature can in principle introduce
    /// piecewise-linear-SDF noise. The gate stays at 1 µm strict
    /// (the synthetic §F-4 audit cleared 0.000000 mm); if production
    /// noise grows under aggressive curvature, the recon-4 §F-5 #2
    /// fallback is to relax to 10 µm (still ≪ FDM 0.4 mm bead).
    #[test]
    fn mating_face_is_mathematically_flat_under_curved_centerline() {
        // Bend the centerline mid-span (10° turn): tangent rotates
        // so segment binormals along the polyline vary.
        let centerline = vec![
            Point3::new(-0.060, 0.0, 0.0),
            Point3::new(-0.020, 0.0, 0.0),
            Point3::new(0.020, 0.007, 0.0), // ~10° kink
            Point3::new(0.060, 0.014, 0.0),
        ];
        let split = crate::ribbon::SplitNormal::new(Vector3::new(0.0, 1.0, 0.0)).unwrap();
        let ribbon = Ribbon::new(centerline, split).unwrap();
        let body = Solid::cuboid(Vector3::new(0.010, 0.010, 0.010));
        let region = Solid::cuboid(Vector3::new(0.080, 0.030, 0.030));

        let (midpoint, binormal) = ribbon.seam_plane_reference();
        let normal_v = binormal.into_inner();
        let plane_d_mm = midpoint.coords.dot(&normal_v) * 1000.0;
        let overlap_mm = RIBBON_PIECE_OVERLAP_M * 1000.0;

        let neg =
            mesh_piece_through_p_pipeline(&body, &region, &ribbon, PieceSide::Negative, 0.005);
        let pos =
            mesh_piece_through_p_pipeline(&body, &region, &ribbon, PieceSide::Positive, 0.005);

        let signed_dist = |v: &Point3<f64>| v.coords.dot(&normal_v) - plane_d_mm;
        let neg_max = neg
            .vertices
            .iter()
            .map(signed_dist)
            .fold(f64::MIN, f64::max);
        let pos_min = pos
            .vertices
            .iter()
            .map(signed_dist)
            .fold(f64::MAX, f64::min);
        // The piecewise-linear segment binormal stays exactly linear
        // within each segment, so cap-vertex deviation from the
        // arc-midpoint segment's binormal plane is small. Strict
        // gate 1 µm; loosen if production exceeds (recon-4 §F-5 #2).
        let cap_tol_mm = 1.0e-3;
        assert!(
            (neg_max - overlap_mm).abs() < cap_tol_mm,
            "curved-centerline Negative cap must still be flat at +{overlap_mm:.4} mm; \
             got max = {neg_max:.6} mm (relax to 10 µm per §F-5 #2 if production hits this)",
        );
        assert!(
            (pos_min - -overlap_mm).abs() < cap_tol_mm,
            "curved-centerline Positive cap must still be flat at -{overlap_mm:.4} mm; \
             got min = {pos_min:.6} mm",
        );
    }

    /// Unbounded bounding region surfaces as
    /// `CastError::InfiniteBounds(BoundingRegion)` — the ribbon
    /// half-space needs a finite AABB.
    #[test]
    fn unbounded_bounding_region_returns_infinite_bounds_error() {
        let body = Solid::cuboid(Vector3::new(0.010, 0.010, 0.010));
        let region = Solid::plane(Vector3::new(0.0, 0.0, 1.0), 0.0);
        let centerline = vec![Point3::new(-0.05, 0.0, 0.0), Point3::new(0.05, 0.0, 0.0)];
        let split = SplitNormal::new(Vector3::new(0.0, 1.0, 0.0)).unwrap();
        let ribbon = Ribbon::new(centerline, split).unwrap();
        let err = compose_piece_solid(&body, &region, &ribbon, PieceSide::Negative).unwrap_err();
        assert!(
            matches!(err, CastError::InfiniteBounds(CastTarget::BoundingRegion)),
            "expected InfiniteBounds(BoundingRegion), got {err:?}",
        );
    }

    /// Unbounded layer body is **not** an error in v2 (unlike v1's
    /// `clip_above_body` which needs `body.z_max`); the final piece
    /// Solid's bounds inherit from the bounding region.
    #[test]
    fn unbounded_layer_body_is_not_an_error() {
        let body = Solid::plane(Vector3::new(0.0, 0.0, 1.0), 0.0);
        let region = Solid::cuboid(Vector3::new(0.030, 0.030, 0.030));
        let centerline = vec![Point3::new(-0.05, 0.0, 0.0), Point3::new(0.05, 0.0, 0.0)];
        let split = SplitNormal::new(Vector3::new(0.0, 1.0, 0.0)).unwrap();
        let ribbon = Ribbon::new(centerline, split).unwrap();
        let (piece, _) = compose_piece_solid(&body, &region, &ribbon, PieceSide::Negative).unwrap();
        // Final piece carries the bounding region's finite AABB.
        let aabb = piece
            .bounds()
            .expect("composition inherits bounding region's finite AABB");
        assert!(aabb.size().x > 0.0);
    }

    // ----- Sub-leaf A regression: workshop iter-1 mold visual-gate -

    /// Workshop iter-1 mold visual-gate regression (per the
    /// `docs/WORKSHOP_ITER1_MOLD_RECON.md` sub-leaf A spike):
    ///
    /// **Wide-body fixture** — body half-extent 30 mm in `+X` (wider
    /// than the pre-C1 hardcoded 25 mm `offset_from_centerline_m`),
    /// bounding 40 mm in `+X` (10 mm cup-wall annulus). With the
    /// pre-C1 fixed offset the pin would have landed AT x = 25 mm —
    /// inside the body, free-floating from the cup-wall mesh. With
    /// C1's body-relative offset the pin lands at the annulus
    /// midpoint, `x = (body_dist + bounding_dist) / 2 = 35 mm`.
    ///
    /// Post-S5 the invariant lives in the transform parameters
    /// rather than in the post-CSG mesh topology: pre-S5 the pin
    /// SDF unioned into the piece Solid, so the MC output produced a
    /// single connected shell when the pin sat in the cup wall and
    /// 2 components when it floated free. Post-S5 the pin is a
    /// separate mesh-CSG cylinder primitive whose attachment to the
    /// cup-wall material is governed by the cylinder Union's
    /// `parent.center_m` (must lie OUTSIDE the body and INSIDE the
    /// bounding region — i.e., in the cup-wall annulus where the
    /// union intersects cup-wall material rather than empty body-
    /// cavity space).
    ///
    /// Recon-4 (P) reverted recon-3 (α)'s bounds-anchored offset to
    /// the pre-recon-2 / S5 annulus-midpoint design + binormal axis
    /// (perpendicular to the seam plane, pin sweeps through cup-wall
    /// material as a contained cylinder); see
    /// `docs/CF_CAST_SEAM_FACE_FILM_RECON_PLAN.md` §F-2/§F-3.
    #[test]
    fn pin_transforms_anchor_in_cup_wall_for_iter1_wide_body() {
        use crate::registration::{
            PIN_SEGMENTS, PinSpec, RegistrationKind, build_registration_transforms,
        };

        let layer_body = Solid::cuboid(Vector3::new(0.030, 0.030, 0.030));
        let bounding_region = Solid::cuboid(Vector3::new(0.040, 0.040, 0.040));
        // Centerline along +Z, split-normal +X. Binormal = tangent ×
        // split-normal = +Z × +X = +Y; pin cylinders extend along ±Y
        // (perpendicular to the seam plane, sweeping through the
        // cup-wall annulus). pin_offset = annulus midpoint along
        // ±X = ±(30 + 40)/2 = ±35 mm.
        let centerline = vec![Point3::new(0.0, 0.0, -0.050), Point3::new(0.0, 0.0, 0.050)];
        let split = SplitNormal::new(Vector3::new(1.0, 0.0, 0.0)).unwrap();
        // Chunky 5 mm half-length pin in this 10 mm cup-wall fixture
        // still fits inside the bounding region (pin extends ±5 mm
        // along ±Y from pin_center at |x| = 35 mm, ±Y body extent
        // 30 mm so pin doesn't escape the bounding region in Y).
        let chunky_pins = PinSpec {
            pin_radius_m: 0.005,
            pin_half_length_m: 0.005,
            arc_fractions: vec![0.5], // single pin for a cleaner reproducer
            ..PinSpec::iter1()
        };
        let ribbon = Ribbon::new(centerline, split)
            .unwrap()
            .with_registration(RegistrationKind::Pins(chunky_pins));

        // Two pins emitted (one arc fraction × two lateral mirror sides).
        let neg = build_registration_transforms(
            &ribbon,
            &layer_body,
            &bounding_region,
            PieceSide::Negative,
        );
        assert_eq!(neg.len(), 2, "1 arc fraction × 2 lateral mirrors = 2 pins");
        for t in &neg {
            let params = match t {
                MatingTransform::UnionCylinder { params } => params,
                other => panic!("Negative side expects UnionCylinder; got {other:?}"),
            };
            let c = params.parent.center_m;
            assert!(
                layer_body.evaluate(&c) > 0.0,
                "iter-1 wide-body regression: pin center {c:?} must be OUTSIDE the body \
                 (cup-wall material); body.evaluate = {}",
                layer_body.evaluate(&c),
            );
            assert!(
                bounding_region.evaluate(&c) < 0.0,
                "iter-1 wide-body regression: pin center {c:?} must be INSIDE the bounding \
                 region (cup-wall material); bounds.evaluate = {}",
                bounding_region.evaluate(&c),
            );
            // Annulus-midpoint offset along ±X = ±(30 + 40) / 2 = ±35 mm.
            assert!(
                (c.x.abs() - 0.035).abs() < 1e-6,
                "pin center should land at the annulus-midpoint offset |x| ≈ 35 mm; got {c:?}",
            );
            assert_eq!(params.segments, PIN_SEGMENTS);
        }
    }

    /// Workshop iter-1 plug-socket regression preserved post-S6.
    ///
    /// Pre-S6 this test queried the cup-piece Solid SDF directly to
    /// verify that the plug-shaft socket carved cup-wall material
    /// at the cap-plane centroid (where cf-scan-prep's
    /// `trim_floor_mm` leaves the centerline tip 40 mm short of
    /// the cap plane). Post-S6 the cup-piece Solid is plug-socket-
    /// agnostic — the socket appears as a `SubtractCylinder`
    /// transform whose [`CylinderParent::center_m`] lives at
    /// `cap_centroid + cap_normal * pin_length / 2`, and the carve
    /// materializes downstream in `apply_mating_transforms`. The
    /// composition-time invariant tested here: the shaft socket
    /// transform's parent anchors at the cap-plane centroid (NOT
    /// at the trimmed centerline tip).
    #[test]
    fn plug_socket_transform_anchors_at_cap_plane_centroid_past_trimmed_centerline_tip() {
        use crate::plug::{PlugPinKind, PlugPinSpec};

        let centerline = vec![
            Point3::new(0.0, 0.0, 0.073),
            Point3::new(0.0, 0.0, 0.020),
            Point3::new(0.0, 0.0, -0.013), // trimmed end (40 mm above cap)
        ];
        let split = SplitNormal::new(Vector3::new(1.0, 0.0, 0.0)).unwrap();
        let cap_centroid = Point3::new(0.0, 0.0, -0.054);
        let cap_normal = Vector3::new(0.0, 0.0, -1.0);
        let pin_spec = PlugPinSpec {
            pin_length_m: 0.020,
            ..PlugPinSpec::iter1()
        };
        let ribbon = Ribbon::new(centerline, split)
            .unwrap()
            .with_pour_end_hint(cap_centroid, cap_normal)
            .with_plug_pins(PlugPinKind::Axial(pin_spec));

        let body = Solid::cuboid(Vector3::new(0.020, 0.020, 0.060));
        let bounding = Solid::cuboid(Vector3::new(0.030, 0.030, 0.090));
        let (_solid, transforms) =
            compose_piece_solid(&body, &bounding, &ribbon, PieceSide::Negative).unwrap();

        // First SubtractCylinder = plug-shaft socket (per
        // `build_plug_socket_transforms`'s Vec ordering: shaft,
        // T-bar, [dome shaft]).
        let shaft_socket = transforms
            .iter()
            .find_map(|t| match t {
                MatingTransform::SubtractCylinder { params } => Some(params),
                _ => None,
            })
            .expect("S6: cup piece emits SubtractCylinder for plug shaft socket");

        // Socket parent center = cap_centroid + cap_normal *
        // pin_length / 2 = (0,0,-0.054) + (0,0,-1) * 0.010 =
        // (0, 0, -0.064). Past the trimmed centerline tip at
        // z=-0.013, exactly where the original SDF-era socket
        // anchored cup-wall material.
        let c = shaft_socket.parent.center_m;
        assert!(
            (c.z - -0.064).abs() < 1e-9,
            "shaft socket should anchor at cap_centroid + outward*half_length \
             (z = -0.064); got {c:?}",
        );
        assert!(
            c.z < -0.013,
            "shaft socket parent must NOT regress to centerline.last() (z = -0.013) anchoring; \
             got center.z = {}",
            c.z,
        );
    }

    /// Recon §5 T-bar bisection gate: post-CSG the Negative cup and
    /// Positive cup mating faces each carry a half-disk T-slot
    /// cross-section. The T-bar's axis is parallel to the seam
    /// normal (= ribbon binormal), so under recon-4 (P) the SDF
    /// halfspace intersect at the SDF level produces a half-shell
    /// whose seam face is at `signed_dist = ±RIBBON_PIECE_OVERLAP_M`;
    /// the mesh-CSG `SubtractCylinder` then removes the T-slot
    /// cylinder from the half-shell, punching a circular hole in
    /// the seam face whose rim consists of vertices at the kept-
    /// side cap with radial distance ≤ (`t_bar_radius` + diametral/2)
    /// from the T-bar's projected center.
    ///
    /// Under the pre-(P) S4 architecture the same property held via
    /// a different mechanism (`SeamTrim` bisected the full cylinder
    /// against the seam plane); this test exercises the cap-rim
    /// invariant rather than the bisection mechanism, so it works
    /// uniformly across (P) and (pre-P) for any future migration.
    /// Stronger than eyeball cf-view per
    /// `feedback_math_verify_geometric_contracts`.
    #[test]
    fn t_bar_halves_share_coplanar_seam_face() {
        use crate::plug::{PlugPinKind, PlugPinSpec};

        // Fixture mirroring the iter-1 cap-plane layout so the
        // T-bar axis exercises the typical `pour_outward ×
        // split_normal` path. Use a short pin (4 mm) so the T-bar
        // sits inside the bounding region for the test cuboid.
        let centerline = vec![Point3::new(0.0, 0.0, 0.073), Point3::new(0.0, 0.0, -0.013)];
        let split = SplitNormal::new(Vector3::new(1.0, 0.0, 0.0)).unwrap();
        let cap_centroid = Point3::new(0.0, 0.0, -0.054);
        let cap_normal = Vector3::new(0.0, 0.0, -1.0);
        let pin_spec = PlugPinSpec {
            pin_length_m: 0.004,
            ..PlugPinSpec::iter1()
        };
        let ribbon = Ribbon::new(centerline, split)
            .unwrap()
            .with_pour_end_hint(cap_centroid, cap_normal)
            .with_plug_pins(PlugPinKind::Axial(pin_spec));

        // Iter-1-like body + bounding. Use a body extent in Y that
        // covers the T-bar's lateral half-length (12 mm) plus the
        // axial clearance / 2 (0.5 mm) and a bounding cuboid that
        // spans the cup wall in every axis.
        let body = Solid::cuboid(Vector3::new(0.020, 0.020, 0.030));
        let bounding = Solid::cuboid(Vector3::new(0.040, 0.040, 0.090))
            .translate(Vector3::new(0.0, 0.0, -0.020));

        let neg =
            mesh_piece_through_p_pipeline(&body, &bounding, &ribbon, PieceSide::Negative, 0.005);
        let pos =
            mesh_piece_through_p_pipeline(&body, &bounding, &ribbon, PieceSide::Positive, 0.005);
        assert!(!neg.vertices.is_empty(), "Negative cup must survive trim");
        assert!(!pos.vertices.is_empty(), "Positive cup must survive trim");

        // Seam plane normal = ribbon binormal at arc midpoint. For
        // this centerline (along -Z) with split-normal +X, binormal
        // = tangent × split = -Z × +X = -Y. seam_plane_reference
        // returns the unit binormal directly.
        let (midpoint, binormal) = ribbon.seam_plane_reference();
        let normal_v = binormal.into_inner();
        let plane_d_mm = midpoint.coords.dot(&normal_v) * 1000.0;
        let overlap_mm = RIBBON_PIECE_OVERLAP_M * 1000.0;

        // T-bar center in mm coords. Plug-side T-bar parent half-
        // length = 12 mm; T-slot socket extends by axial/2 = 0.5 mm.
        // The T-bar axis is parallel to seam normal, so the
        // cap-vertex circle for the bisected T-slot sits at the
        // seam plane at radial distance ≤ (t_bar_radius +
        // diametral/2) from the projection of the T-bar center
        // onto the seam plane.
        let spec = PlugPinSpec::iter1();
        let t_slot_radius_mm =
            (spec.t_bar_radius_m + spec.t_bar_diametral_clearance_m / 2.0) * 1000.0;
        // T-bar center in m: cap_centroid + cap_normal * pin_length.
        let t_bar_center_m = cap_centroid + cap_normal * 0.004_f64;
        let t_bar_center_mm = nalgebra::Vector3::new(
            t_bar_center_m.x * 1000.0,
            t_bar_center_m.y * 1000.0,
            t_bar_center_m.z * 1000.0,
        );

        // Project a vertex onto the seam plane: signed_dist measures
        // distance along seam normal; the projected location lies at
        // `v - signed_dist * normal_v` in the seam plane.
        let signed_dist = |v: &Point3<f64>| -> f64 { v.coords.dot(&normal_v) - plane_d_mm };
        // Distance in the seam plane from the T-bar's projected
        // center to the vertex's projection.
        let in_plane_radius_from_t_bar = |v: &Point3<f64>| -> f64 {
            // Project v - t_bar_center onto the seam plane.
            let delta = v.coords - t_bar_center_mm;
            let along_normal = delta.dot(&normal_v);
            let in_plane = delta - along_normal * normal_v;
            in_plane.norm()
        };
        // A T-slot cap vertex sits NEAR the seam plane (within the
        // ±overlap cap) AND within `t_slot_radius_mm` of the T-bar's
        // projected center.
        let cap_tol_mm = 1.0e-3;
        let near_t_slot_cap = |signed: f64, radius: f64, expected_cap: f64| -> bool {
            (signed - expected_cap).abs() < cap_tol_mm && radius < t_slot_radius_mm + cap_tol_mm
        };

        // Negative kept half: cap vertices at signed_dist = +overlap.
        // Find any vertex that satisfies both conditions — proves
        // the T-slot's half-disk lands on the seam plane.
        let neg_has_t_slot_cap = neg
            .vertices
            .iter()
            .any(|v| near_t_slot_cap(signed_dist(v), in_plane_radius_from_t_bar(v), overlap_mm));
        assert!(
            neg_has_t_slot_cap,
            "Negative cup's seam cap must contain T-slot cross-section vertices \
             (within {t_slot_radius_mm:.4} mm of T-bar center, on seam plane at \
             signed_dist = +{overlap_mm:.4} mm)",
        );

        // Positive: mirror.
        let pos_has_t_slot_cap = pos
            .vertices
            .iter()
            .any(|v| near_t_slot_cap(signed_dist(v), in_plane_radius_from_t_bar(v), -overlap_mm));
        assert!(
            pos_has_t_slot_cap,
            "Positive cup's seam cap must contain T-slot cross-section vertices \
             at signed_dist = -{overlap_mm:.4} mm",
        );

        // Co-planarity gate: the difference between the Negative
        // T-slot cap's max signed_dist and the Positive T-slot
        // cap's min signed_dist should be exactly
        // `2 * RIBBON_PIECE_OVERLAP_M`. (Already exercised by
        // `mating_face_is_mathematically_flat_and_coplanar` —
        // S6's contribution is verifying T-slot bisection works
        // alongside the SDF seam.)
    }
}
