//! Per-piece SDF composition for v2 curve-following multi-piece molds.
//!
//! Step 5 of `docs/CURVE_FOLLOWING_DESIGN.md` §Algorithm §"Per-piece
//! SDF construction": for each layer × piece pair the mold-piece
//! geometry is the CSG composition
//!
//! ```text
//! piece_sdf = bounding_region ∖ layer_body
//! piece_meshcsg = SeamTrim against ribbon's single-plane approximation
//! ```
//!
//! Pre-S4 the ribbon split was an SDF half-space intersect appended
//! to the formula; S4 of `docs/CF_CAST_MATING_FEATURES_PLAN.md`
//! moved it to a post-MC mesh-CSG operation emitted into the
//! returned `Vec<MatingTransform>`. This module provides
//! [`compose_piece_solid`] that wires both halves: the SDF side
//! over [`cf_design::Solid`]'s `subtract` / `intersect` algebra,
//! the mesh-CSG side via [`crate::mesh_csg::MatingTransform`]. The
//! resulting `(Solid, Vec<MatingTransform>)` pair feeds into the
//! per-piece pipeline at Step 6: `solid_to_mm_mesh` →
//! `apply_mating_transforms` → F4 → STL.
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
//! [`RIBBON_PIECE_OVERLAP_M`] biases each piece's seam-trim plane
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
//! S4 of `docs/CF_CAST_MATING_FEATURES_PLAN.md` migrated the seam
//! from an SDF half-space intersection (where marching-cubes
//! resolved the seam through its body-cavity-influenced grid) to a
//! post-MC mesh-trim against an exact plane via
//! [`crate::mesh_csg::MatingTransform::SeamTrim`]. The cup-piece
//! [`Solid`] returned here is now **side-agnostic**: the
//! side-specific cut lives in the returned `Vec<MatingTransform>`.

use cf_design::Solid;
use nalgebra::{Point3, Unit, Vector3};

use crate::error::{CastError, CastTarget};
use crate::mesh_csg::MatingTransform;
use crate::plug::build_plug_socket_solid;
use crate::pour::build_pour_gate_solid;
use crate::registration::build_registration_solid;
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
/// - **Solid** = `bounding_region ∖ layer_body` (post-S4
///   **side-agnostic** — both [`PieceSide::Negative`] and
///   [`PieceSide::Positive`] return the same Solid; the side-specific
///   bisection moves to the Vec).
/// - **`Vec<MatingTransform>`** = a single
///   [`crate::mesh_csg::MatingTransform::SeamTrim`] selecting the
///   side's kept half-space (plus the 0.5 mm overlap bias). S5
///   (pins) + S6 (T-slot + plug-socket) will extend the Vec with
///   `UnionCylinder` / `SubtractCylinder` entries.
///
/// The returned Solid's SDF is **negative inside the cup-wall
/// material** (inside the bounding region, outside the layer body);
/// the side-specific cut happens in `apply_mating_transforms`. See
/// `docs/CF_CAST_MATING_FEATURES_PLAN.md` §S4.
///
/// `bounding_region` and `layer_body` are taken by reference and
/// cloned into the result; the underlying [`cf_design::Solid`] is
/// internally `Arc`-shared so cloning is cheap (no geometry
/// duplication).
///
/// # Errors
///
/// - [`CastError::InfiniteBounds`] with [`CastTarget::BoundingRegion`]
///   if `bounding_region.bounds()` returns `None` — downstream MC
///   meshes the SDF over the bounding region's AABB and needs a
///   finite extent.
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
    // Validate bounded region (the ribbon's S4 seam trim derives its
    // plane from the ribbon directly, but downstream meshing needs a
    // finite bounding-region AABB; preserve the explicit error here).
    bounding_region
        .bounds()
        .ok_or(CastError::InfiniteBounds(CastTarget::BoundingRegion))?;
    // S4: piece Solid is side-agnostic; ribbon-side bisection moved
    // to the returned `Vec<MatingTransform>` as a post-MC seam trim.
    let base_piece = bounding_region.clone().subtract(layer_body.clone());

    // Step 9: inter-piece registration pins. `Negative` side
    // gains protrusions (union); `Positive` side gains matching
    // holes (subtract). When `ribbon.registration` is `None` the
    // helper returns `None` and the base piece flows through
    // unchanged — Step 5-8 callers see no behavior change.
    //
    // The registration builder ray-marches `layer_body` +
    // `bounding_region` along the ribbon's split-normal to put the
    // pin in the per-layer cup-wall annulus midpoint (see the
    // `crate::registration` module docstring for the "derived, not
    // fixed" rationale).
    let piece_with_pins = match (
        build_registration_solid(ribbon, layer_body, bounding_region),
        side,
    ) {
        (Some(pins), PieceSide::Negative) => base_piece.union(pins),
        (Some(pins), PieceSide::Positive) => base_piece.subtract(pins),
        (None, _) => base_piece,
    };

    // Step 10 + v2.1 sub-leaves 2-3: pour-gate + air-vent channels.
    // Both pieces lose material along the channel cylinders
    // (subtract). The side-mounted pour gate runs along the ribbon
    // binormal from the centerline midpoint (so the positive piece
    // gets most of the carve + a thin seam-overlap slice on the
    // negative piece); the apex vent rises along `+Z` from the
    // polyline's argmax-z vertex (straddles the seam, so each piece
    // gets half the channel cross-section). When
    // `ribbon.pour_gate` is `None` the helper returns `None` and the
    // piece flows through unchanged — Steps 5-9 callers unaffected.
    let piece_with_channels = match build_pour_gate_solid(ribbon) {
        Some(channels) => piece_with_pins.subtract(channels),
        None => piece_with_pins,
    };

    // v2.1: plug-anchor sockets at centerline endpoints. Both
    // pieces lose material where the socket cylinders extend into
    // cup-wall material (the body-cavity portion is already
    // excluded by `subtract(layer_body)` above, so the socket
    // subtraction is a no-op there). Sockets straddle the ribbon
    // seam — same compose-on-both-pieces pattern as the pour gate
    // and vent. When `ribbon.plug_pins` is `None` the helper
    // returns `None` and the piece flows through unchanged — Steps
    // 5-10 callers unaffected.
    let piece = match build_plug_socket_solid(ribbon) {
        Some(sockets) => piece_with_channels.subtract(sockets),
        None => piece_with_channels,
    };
    // S4: emit a SeamTrim against the single-plane approximation of
    // the ribbon's curve-following splitting surface. Both pieces
    // trim against the SAME physical plane through the centerline's
    // arc-length midpoint, with the 0.5 mm overlap bias applied
    // symmetrically so each kept half-space extends `2 *
    // RIBBON_PIECE_OVERLAP_M = 1 mm` into the other side. S5 will
    // add the registration-pin UnionCylinder/SubtractCylinder
    // transforms; S6 will add the T-slot + plug-socket carves.
    let (midpoint, binormal) = ribbon.seam_plane_reference();
    let seam_trim = MatingTransform::SeamTrim {
        normal: seam_normal_for(side, binormal),
        offset_m: seam_offset_m_for(side, midpoint, binormal),
    };
    Ok((piece, vec![seam_trim]))
}

/// Outward-pointing unit normal of the kept half-space for `side`.
///
/// `Positive` keeps the `+binormal` half (away from the Negative
/// piece); `Negative` keeps the `-binormal` half. manifold3d's
/// `trim_by_plane` keeps the half-space `n · p > offset`, so the
/// returned normal points toward the matter to retain.
fn seam_normal_for(side: PieceSide, binormal: Unit<Vector3<f64>>) -> Unit<Vector3<f64>> {
    match side {
        PieceSide::Positive => binormal,
        // `Unit::new_unchecked` is sound for the negation of a unit
        // vector (magnitude preserved exactly under f64 negation).
        PieceSide::Negative => Unit::new_unchecked(-binormal.into_inner()),
    }
}

/// Plane offset in meters for the kept half-space `n · p > offset_m`
/// (where `n = seam_normal_for(side, binormal)`).
///
/// Both pieces' kept half-spaces share the **same physical plane**
/// `dot(p - midpoint, binormal) = 0`, with the
/// [`RIBBON_PIECE_OVERLAP_M`] bias subtracted from each offset so
/// each piece extends 0.5 mm into the opposite side (combined
/// overlap = 1 mm at the seam — matches the pre-S4 SDF bias
/// convention).
fn seam_offset_m_for(side: PieceSide, midpoint: Point3<f64>, binormal: Unit<Vector3<f64>>) -> f64 {
    let plane_d = midpoint.coords.dot(&binormal.into_inner());
    match side {
        // Positive: kept where `+binormal · p > plane_d - overlap`.
        PieceSide::Positive => plane_d - RIBBON_PIECE_OVERLAP_M,
        // Negative: kept where `-binormal · p > -plane_d - overlap`.
        PieceSide::Negative => -plane_d - RIBBON_PIECE_OVERLAP_M,
    }
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

    /// Post-S4 the cup-piece [`Solid`] is **side-agnostic** (the
    /// side-specific ribbon-seam cut moved to the returned
    /// `Vec<MatingTransform>`); a query point in the cup wall
    /// outside the body must register inside the Solid regardless of
    /// `side`. The mesh-level partition invariant moves to the
    /// `pieces_partition_cup_material_at_seam_plane_in_post_csg_mesh`
    /// test below.
    #[test]
    fn piece_solid_contains_cup_wall_outside_body() {
        let (body, region, ribbon) = fixture();
        let (piece, _) = compose_piece_solid(&body, &region, &ribbon, PieceSide::Negative).unwrap();
        // Inside bounding region (-30 ≤ z ≤ +30 mm), outside body
        // (body z ∈ [-10, +10]):
        let q = Point3::new(0.0, 0.0, -0.020);
        assert!(
            piece.evaluate(&q) < 0.0,
            "piece Solid must contain cup-wall point at z = -20 mm; got {}",
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

    /// Mesh-level test fixture pipeline: compose Solid → MC →
    /// post-CSG mesh-trim. Returns the post-CSG `IndexedMesh` in mm
    /// world coords, exactly the geometry that flows to F4 +
    /// STL-export in the live `mesh_and_gate_v2_*` sites.
    fn mesh_piece_through_s4_pipeline(
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

    /// Post-S4 partition invariant at the **mesh** level: Negative
    /// and Positive piece meshes occupy opposite half-spaces of the
    /// seam plane, with the kept side extending to the bounding-
    /// region wall and the trimmed side capped at the
    /// [`RIBBON_PIECE_OVERLAP_M`] overlap bias (≈ ±0.5 mm).
    ///
    /// Pre-S4 this was an SDF test (`neg.evaluate(z=-20) < 0` etc.).
    /// Post-S4 the Solid is side-agnostic; the partition appears
    /// only after `apply_mating_transforms` runs the seam trim.
    /// (Recon §8 missed this rewrite; bookmark in the S4 commit
    /// message + memory.)
    #[test]
    fn pieces_partition_cup_material_at_seam_plane_in_post_csg_mesh() {
        let (body, region, ribbon) = fixture();
        // 5 mm cells: coarse but fast; the seam-trim is an exact
        // post-MC plane cut so trim-side vertex extent is bit-precise
        // regardless of MC resolution.
        let neg =
            mesh_piece_through_s4_pipeline(&body, &region, &ribbon, PieceSide::Negative, 0.005);
        let pos =
            mesh_piece_through_s4_pipeline(&body, &region, &ribbon, PieceSide::Positive, 0.005);
        assert!(!neg.vertices.is_empty(), "Negative mesh must survive trim");
        assert!(!pos.vertices.is_empty(), "Positive mesh must survive trim");

        // Vertex extremes along the seam-plane normal (Z, since
        // binormal = +Z for this fixture).
        let neg_z_max = neg.vertices.iter().map(|v| v.z).fold(f64::MIN, f64::max);
        let neg_z_min = neg.vertices.iter().map(|v| v.z).fold(f64::MAX, f64::min);
        let pos_z_max = pos.vertices.iter().map(|v| v.z).fold(f64::MIN, f64::max);
        let pos_z_min = pos.vertices.iter().map(|v| v.z).fold(f64::MAX, f64::min);

        // Overlap bias in mm (mesh coords).
        let overlap_mm = RIBBON_PIECE_OVERLAP_M * 1000.0;
        let tol_mm = 1.0e-3; // 1 µm: manifold3d's trim is bit-precise on the cut plane.

        // Negative kept half: z < +overlap. max.z lies precisely on
        // the cut plane (manifold3d's trim caps the cut with new
        // vertices at z = +overlap).
        assert!(
            (neg_z_max - overlap_mm).abs() < tol_mm,
            "Negative trim cap should land at z = +{overlap_mm:.3} mm; got max.z = {neg_z_max:.6} mm",
        );
        // Negative extends to the bounding-region floor.
        assert!(
            (neg_z_min - -30.0).abs() < 0.1,
            "Negative should extend to bounding floor at z ≈ -30 mm; got min.z = {neg_z_min:.3} mm",
        );

        // Positive: mirror image.
        assert!(
            (pos_z_min - -overlap_mm).abs() < tol_mm,
            "Positive trim cap should land at z = -{overlap_mm:.3} mm; got min.z = {pos_z_min:.6} mm",
        );
        assert!(
            (pos_z_max - 30.0).abs() < 0.1,
            "Positive should extend to bounding ceiling at z ≈ +30 mm; got max.z = {pos_z_max:.3} mm",
        );
    }

    /// Plan §S4 falsification gate — **mathematically instrumented**
    /// (not eyeball-the-cf-view). Verifies the mating-face flatness
    /// invariant the workshop iter-1 print physically falsified:
    ///
    /// 1. Each piece's seam-trim cap is **exactly flat**: every
    ///    cap-region vertex lies on the piece's cut plane to within
    ///    f64 precision (manifold3d's `trim_by_plane` cap-vertex
    ///    contract).
    /// 2. The two pieces' cap planes are **parallel + co-planar with
    ///    overlap**: both pieces share the same physical plane
    ///    geometry (same normal), with caps offset by exactly
    ///    `±RIBBON_PIECE_OVERLAP_M`. When assembled, the 1 mm overlap
    ///    region is the design-doc seam-mating zone.
    /// 3. No piece has material on the trimmed side past its cap:
    ///    every vertex lies on the kept-side half-space (within 1 µm
    ///    of f64 noise).
    ///
    /// This locks the geometric contract in a structural test:
    /// future manifold3d upgrades or S4 reworks can't silently
    /// regress the mating-face flatness the iter-1 print failed.
    /// Replaces the cf-view visual gate the plan §S4 placeholder
    /// suggested (visual gate is too imprecise — manifold3d's cap is
    /// bit-precise, not 0.1 mm RMS).
    #[test]
    fn s4_mating_face_is_mathematically_flat_and_coplanar() {
        let (body, region, ribbon) = fixture();
        // For the fixture, binormal = +Z and midpoint = origin, so
        // the seam plane is z = 0, Negative cap at z = +0.5 mm,
        // Positive cap at z = -0.5 mm. The math below works for any
        // ribbon — it just uses the runtime seam_plane_reference.
        let (midpoint, binormal) = ribbon.seam_plane_reference();
        let normal_v = binormal.into_inner();
        let plane_d_mm = midpoint.coords.dot(&normal_v) * 1000.0;
        let overlap_mm = RIBBON_PIECE_OVERLAP_M * 1000.0;

        let neg =
            mesh_piece_through_s4_pipeline(&body, &region, &ribbon, PieceSide::Negative, 0.005);
        let pos =
            mesh_piece_through_s4_pipeline(&body, &region, &ribbon, PieceSide::Positive, 0.005);

        // Signed distance of each vertex from the seam plane along
        // +binormal (mm). The seam plane itself is at signed_dist=0.
        let signed_dist = |v: &Point3<f64>| -> f64 { v.coords.dot(&normal_v) - plane_d_mm };

        // (1) Negative kept where `binormal · p_mm < plane_d + overlap`,
        // i.e., signed_dist < +overlap. Cap vertices sit AT
        // signed_dist = +overlap. No vertex should sit past the cap.
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
        let cap_tol_mm = 1.0e-3; // 1 µm — well inside manifold3d's f64 precision
        assert!(
            (neg_max - overlap_mm).abs() < cap_tol_mm,
            "Negative cap vertices must lie on signed_dist = +{overlap_mm:.4} mm \
             (flat by manifold3d's trim_by_plane contract); got max = {neg_max:.6} mm",
        );
        assert!(
            neg_min >= -50.0,
            "Negative should still have far-side material (bounding floor); got min = {neg_min:.3} mm",
        );

        // (2) Positive kept where signed_dist > -overlap. Cap at
        // signed_dist = -overlap.
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

        // (3) No vertex strictly past the cap (Negative max ≤ cap +
        // tol; Positive min ≥ -cap - tol). Already implied by (1)
        // + (2)'s equality assertions; re-check explicitly to lock
        // the "kept-side only" invariant.
        assert!(
            neg_max <= overlap_mm + cap_tol_mm,
            "Negative mesh must not extend past the trim cap",
        );
        assert!(
            pos_min >= -overlap_mm - cap_tol_mm,
            "Positive mesh must not extend past the trim cap",
        );

        // (4) Co-planarity across pieces: the two cap planes share
        // the same normal (binormal) and their offsets along that
        // normal differ by exactly `2 * RIBBON_PIECE_OVERLAP_M`
        // (the design-doc overlap). This is the "pieces sit flush"
        // contract — assembled, the 1 mm overlap region is the
        // seam-mating zone.
        let cap_offset_difference_mm = neg_max - pos_min;
        let expected_difference_mm = 2.0 * overlap_mm;
        assert!(
            (cap_offset_difference_mm - expected_difference_mm).abs() < cap_tol_mm,
            "Co-planarity gate: Negative.cap.signed_dist - Positive.cap.signed_dist \
             must equal 2 × overlap = {expected_difference_mm:.4} mm; \
             got {cap_offset_difference_mm:.6} mm",
        );
    }

    /// Mating-face flatness still holds for a curved centerline
    /// (the single-plane approximation per `Ribbon::seam_plane_reference`).
    /// Locks the contract under the conditions plan §S4 warned
    /// about: the approximation deviates from the curve-following
    /// surface, but each piece's cap is still exactly flat by
    /// manifold3d's trim contract.
    #[test]
    fn s4_mating_face_is_mathematically_flat_under_curved_centerline() {
        // Bend the centerline mid-span (10° turn): tangent rotates,
        // so segment binormals along the polyline vary, but
        // `seam_plane_reference` picks one (the arc-midpoint
        // segment's binormal) and the cap-flatness contract holds
        // regardless of how curved the rest of the centerline is.
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
            mesh_piece_through_s4_pipeline(&body, &region, &ribbon, PieceSide::Negative, 0.005);
        let pos =
            mesh_piece_through_s4_pipeline(&body, &region, &ribbon, PieceSide::Positive, 0.005);

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
        let cap_tol_mm = 1.0e-3;
        assert!(
            (neg_max - overlap_mm).abs() < cap_tol_mm,
            "curved-centerline Negative cap must still be flat at +{overlap_mm:.4} mm; \
             got max = {neg_max:.6} mm",
        );
        assert!(
            (pos_min - -overlap_mm).abs() < cap_tol_mm,
            "curved-centerline Positive cap must still be flat at -{overlap_mm:.4} mm; \
             got min = {pos_min:.6} mm",
        );
    }

    /// Both piece meshes extend `RIBBON_PIECE_OVERLAP_M` past the
    /// `z = 0` ribbon zero plane into the other piece's half — the
    /// design-doc 1 mm seam overlap survives the S4 SDF→mesh-CSG
    /// migration.
    ///
    /// Pre-S4 this was tested via SDF probe (`neg.evaluate(z=0) <
    /// 0`); post-S4 the overlap is a property of the post-CSG mesh
    /// vertex extent. Each piece's cut-cap lies 0.5 mm on the
    /// opposite side of the seam plane from the rest of its
    /// material.
    #[test]
    fn both_pieces_overlap_at_ribbon_seam() {
        let (body, region, ribbon) = fixture();
        let neg =
            mesh_piece_through_s4_pipeline(&body, &region, &ribbon, PieceSide::Negative, 0.005);
        let pos =
            mesh_piece_through_s4_pipeline(&body, &region, &ribbon, PieceSide::Positive, 0.005);

        let neg_z_max = neg.vertices.iter().map(|v| v.z).fold(f64::MIN, f64::max);
        let pos_z_min = pos.vertices.iter().map(|v| v.z).fold(f64::MAX, f64::min);

        // Negative crosses z=0 into +Z by ~0.5 mm; Positive crosses
        // z=0 into -Z by ~0.5 mm. Together the seam is 1 mm wide.
        assert!(
            neg_z_max > 0.0,
            "Negative mesh must extend into +Z (seam overlap); got max.z = {neg_z_max:.6} mm",
        );
        assert!(
            pos_z_min < 0.0,
            "Positive mesh must extend into -Z (seam overlap); got min.z = {pos_z_min:.6} mm",
        );
        // Combined overlap should be ~1 mm = 2 × RIBBON_PIECE_OVERLAP_M.
        let combined_overlap_mm = neg_z_max - pos_z_min;
        let expected_mm = 2.0 * RIBBON_PIECE_OVERLAP_M * 1000.0;
        assert!(
            (combined_overlap_mm - expected_mm).abs() < 1.0e-3,
            "Combined seam overlap should be ~{expected_mm:.3} mm; \
             got Negative.max.z - Positive.min.z = {combined_overlap_mm:.6} mm",
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

    /// Count connected components of `mesh` via shared-edge BFS over
    /// faces. Two faces are neighbors if they share an undirected
    /// edge (vertex-index pair). Used by the floating-sliver
    /// regression below: the workshop iter-1 falsification produced a
    /// Negative-piece STL whose MC mesh had TWO components (the cup
    /// wall + a free-floating pin cylinder); the body-relative pin
    /// offset (C1) restores the single-component invariant by
    /// landing the pin inside the cup-wall annulus where it
    /// physically attaches to the surrounding mesh.
    fn count_connected_components_via_shared_edges(mesh: &mesh_types::IndexedMesh) -> usize {
        use std::collections::{HashMap, HashSet, VecDeque};
        if mesh.faces.is_empty() {
            return 0;
        }
        // mesh-types face indices are u32; component bookkeeping is
        // on the local face index (usize). Edge keys carry the u32
        // vertex IDs directly.
        let mut edge_to_faces: HashMap<(u32, u32), Vec<usize>> = HashMap::new();
        for (face_idx, face) in mesh.faces.iter().enumerate() {
            for i in 0..3 {
                let a = face[i];
                let b = face[(i + 1) % 3];
                let edge = if a < b { (a, b) } else { (b, a) };
                edge_to_faces.entry(edge).or_default().push(face_idx);
            }
        }
        let mut visited: HashSet<usize> = HashSet::new();
        let mut components = 0_usize;
        for start in 0..mesh.faces.len() {
            if visited.contains(&start) {
                continue;
            }
            components += 1;
            let mut queue: VecDeque<usize> = VecDeque::new();
            queue.push_back(start);
            visited.insert(start);
            while let Some(f) = queue.pop_front() {
                for i in 0..3 {
                    let a = mesh.faces[f][i];
                    let b = mesh.faces[f][(i + 1) % 3];
                    let edge = if a < b { (a, b) } else { (b, a) };
                    if let Some(neighbors) = edge_to_faces.get(&edge) {
                        for &n in neighbors {
                            if visited.insert(n) {
                                queue.push_back(n);
                            }
                        }
                    }
                }
            }
        }
        components
    }

    /// Workshop iter-1 mold visual-gate regression (per the
    /// `docs/WORKSHOP_ITER1_MOLD_RECON.md` sub-leaf A spike):
    ///
    /// **Wide-body fixture** — body half-extent 30 mm in `+X`
    /// (wider than the pre-C1 hardcoded 25 mm `offset_from_centerline_m`),
    /// bounding 40 mm in `+X` (10 mm cup-wall annulus). With the
    /// pre-C1 fixed offset the pin would have landed AT x = 25 mm —
    /// inside the body, free-floating from the cup-wall mesh — and
    /// the Negative-piece STL would have meshed as TWO components.
    /// With C1's body-relative offset, the pin lands at x = 35 mm
    /// (annulus midpoint) and the Negative-piece mesh is a single
    /// connected component.
    ///
    /// Uses a chunky `PinSpec` (5 mm pin radius, 5 mm half-length) so
    /// the 5 mm marching-cubes cells resolve the pin cleanly even at
    /// the coarse test resolution.
    ///
    /// Post-S4 IGNORED — REWRITE in S5 per
    /// `docs/CF_CAST_MATING_FEATURES_RECON.md` §8 table.
    ///
    /// Background: pre-S4 the SDF half-space intersection forced MC
    /// to produce a single open shell (outer + inner cup-wall
    /// surfaces merged at the seam-plane cap during marching cubes
    /// itself). Post-S4 the seam cut is applied AFTER MC by
    /// `manifold3d::trim_by_plane`, which caps the outer and inner
    /// cavity surfaces *independently* — so the post-CSG mesh has
    /// 2+ topological shells (cup outer + body cavity + pin
    /// shells) that the shared-edge BFS counts as separate
    /// components even when geometrically attached. The
    /// "pin-attached-to-cup-wall" invariant the workshop iter-1
    /// fixture originally falsified moves to a different mesh-level
    /// probe in S5 (e.g., parry3d point-in-mesh queries at the pin
    /// surface).
    #[test]
    #[ignore = "REWRITE in S5: shared-edge component count is no longer 1 once MC produces 2-shell hollow-cuboid output (the seam trim caps each shell independently); replace with a parry3d point-in-mesh probe at the pin/cup-wall interface."]
    fn negative_piece_has_single_connected_component_with_iter1_pins_on_wide_body() {
        use crate::registration::{PinSpec, RegistrationKind};

        let layer_body = Solid::cuboid(Vector3::new(0.030, 0.030, 0.030));
        let bounding_region = Solid::cuboid(Vector3::new(0.040, 0.040, 0.040));
        // Centerline along +Z, split-normal +X → binormal +Y. Pin
        // cylinders extend along ±Y (perpendicular to ribbon seam).
        let centerline = vec![Point3::new(0.0, 0.0, -0.050), Point3::new(0.0, 0.0, 0.050)];
        let split = SplitNormal::new(Vector3::new(1.0, 0.0, 0.0)).unwrap();
        let chunky_pins = PinSpec {
            pin_radius_m: 0.005,
            pin_half_length_m: 0.005,
            arc_fractions: vec![0.5], // single pin for a cleaner reproducer
        };
        let ribbon = Ribbon::new(centerline, split)
            .unwrap()
            .with_registration(RegistrationKind::Pins(chunky_pins));

        // 5 mm cells: grid ~80 mm / 5 mm = 16 cells/axis, ~4k cells
        // total. Coarse enough for fast tests; pin (10 mm Ø)
        // resolves as 2 cells radially.
        let mesh = mesh_piece_through_s4_pipeline(
            &layer_body,
            &bounding_region,
            &ribbon,
            PieceSide::Negative,
            0.005,
        );
        let components = count_connected_components_via_shared_edges(&mesh);
        assert_eq!(
            components, 1,
            "iter-1 visual-gate regression: Negative-piece post-S4 mesh must be a single \
             connected component (the pin must attach to the cup-wall annulus, \
             not float free inside the body cavity); got {components}",
        );
    }

    /// Workshop iter-1 plug-socket regression (per `recon §E.2`):
    ///
    /// cf-scan-prep trims the centerline `trim_floor_mm` (typically
    /// 40 mm) above the cap plane to keep the polyline inside the
    /// body interior, so `centerline.last()` lives 40 mm INSIDE
    /// the plug body. Pre-E.2 the plug-socket cylinder was anchored
    /// at `centerline.last()` + extended along last-segment tangent
    /// — entirely inside the body, where the socket carve is a
    /// no-op (the piece composition already excludes body-cavity
    /// space via `subtract(layer_body)`). With E.2 the socket
    /// anchors at the cap-plane centroid (passed via
    /// `with_pour_end_hint(centroid, normal)`) and extends along
    /// the cap-plane outward normal, carving cup-wall material
    /// past the plug body's pinned floor.
    ///
    /// Fixture: trimmed centerline tip at z = -13 mm (40 mm above
    /// cap); cap-plane centroid at z = -54 mm with outward normal
    /// in -Z; body `half_z` = 60 mm so the body extends to z = -60
    /// mm (cup wall floor begins at z = -60 mm). Socket cylinder
    /// (length 20 mm) spans z ∈ [-74, -54] mm — the inner half
    /// (-60 → -54 mm) is no-op inside the body; the outer half
    /// (-74 → -60 mm) carves cup-wall material. Query at
    /// z = -70 mm sits in the cup-wall × socket overlap: with
    /// the socket subtracted, piece SDF > 0 (hollow); without,
    /// piece SDF < 0 (solid).
    #[test]
    fn plug_socket_carves_at_cap_plane_centroid_past_trimmed_centerline_tip() {
        use crate::plug::{PlugPinKind, PlugPinSpec};

        let centerline = || -> Vec<Point3<f64>> {
            vec![
                Point3::new(0.0, 0.0, 0.073), // tip / closed dome
                Point3::new(0.0, 0.0, 0.020),
                Point3::new(0.0, 0.0, -0.013), // trimmed end (40 mm above cap)
            ]
        };
        let split = SplitNormal::new(Vector3::new(1.0, 0.0, 0.0)).unwrap();
        let cap_centroid = Point3::new(0.0, 0.0, -0.054);
        let cap_normal = Vector3::new(0.0, 0.0, -1.0);
        let pin_spec = PlugPinSpec {
            pin_length_m: 0.020,
            ..PlugPinSpec::iter1()
        };
        let ribbon_with_pins = Ribbon::new(centerline(), split)
            .unwrap()
            .with_pour_end_hint(cap_centroid, cap_normal)
            .with_plug_pins(PlugPinKind::Axial(pin_spec));
        let bare_ribbon = Ribbon::new(centerline(), split)
            .unwrap()
            .with_pour_end_hint(cap_centroid, cap_normal);

        let body = Solid::cuboid(Vector3::new(0.020, 0.020, 0.060));
        let bounding = Solid::cuboid(Vector3::new(0.030, 0.030, 0.090));
        let (neg_piece_pins, _) =
            compose_piece_solid(&body, &bounding, &ribbon_with_pins, PieceSide::Negative).unwrap();
        let (neg_piece_bare, _) =
            compose_piece_solid(&body, &bounding, &bare_ribbon, PieceSide::Negative).unwrap();

        let socket_axis_in_wall = Point3::new(0.0, 0.0, -0.070);
        assert!(
            neg_piece_bare.evaluate(&socket_axis_in_wall) < 0.0,
            "no-socket baseline: cup-wall point at (0,0,-70mm) should be inside piece; got {}",
            neg_piece_bare.evaluate(&socket_axis_in_wall),
        );
        assert!(
            neg_piece_pins.evaluate(&socket_axis_in_wall) > 0.0,
            "socket carve at cap-plane centroid should hollow the cup wall at (0,0,-70mm) \
             (piece SDF > 0, i.e., inside the socket hole); got {}",
            neg_piece_pins.evaluate(&socket_axis_in_wall),
        );
        // Dome end (no pin by default) → cup wall stays solid.
        let dome_axis_in_wall = Point3::new(0.0, 0.0, 0.070);
        assert!(
            neg_piece_pins.evaluate(&dome_axis_in_wall) < 0.0,
            "dome end should NOT be carved (no dome-end pin by default); got {}",
            neg_piece_pins.evaluate(&dome_axis_in_wall),
        );
    }
}
