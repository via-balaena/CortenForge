//! Per-piece SDF composition for v2 curve-following multi-piece molds.
//!
//! Step 5 of `docs/CURVE_FOLLOWING_DESIGN.md` §Algorithm §"Per-piece
//! SDF construction": for each layer × piece pair the mold-piece
//! geometry is the standard CSG composition
//!
//! ```text
//! piece_sdf = bounding_region ∖ layer_body ∩ ribbon_side
//! ```
//!
//! where `ribbon_side` is one of the two half-spaces divided by the
//! curve-following [`Ribbon`] surface. This module provides the
//! free function [`compose_piece_solid`] that wires the formula
//! together over [`cf_design::Solid`]'s
//! `subtract` / `intersect` algebra; the resulting `Solid` is ready
//! to feed into the internal `solid_to_mm_mesh` mesher in Step 6's
//! per-piece marching-cubes pipeline.
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
//! [`RIBBON_PIECE_OVERLAP_M`] biases each half-space inward by
//! 0.5 mm so the two pieces' geometry overlaps by 1 mm at the
//! ribbon seam. v1 uses the same magnitude
//! (`CLIP_BODY_OVERLAP_M`) to break the coincidence between the
//! body's top face and the clip's bottom face; v2 applies it at the
//! ribbon ∩ mold-cup intersection per
//! `docs/CURVE_FOLLOWING_DESIGN.md` §Risks §"CSG numerical issues at
//! the ribbon ∩ `mold_cup` intersection". Both halves overlapping
//! also gives the workshop user mechanical tolerance during fit-up
//! (FDM accuracy is ~0.1 mm, well below the 1 mm seam).

use cf_design::Solid;

use crate::error::{CastError, CastTarget};
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

/// Compose the per-(layer × piece) mold-piece SDF as
/// `bounding_region ∖ layer_body ∩ ribbon_side`.
///
/// Returns a [`cf_design::Solid`] expression tree whose evaluated
/// SDF is **negative inside the piece** (i.e., inside the bounding
/// region, outside the layer body, on the correct side of the
/// biased ribbon half-space). Step 6's per-piece marching cubes
/// passes this Solid through the internal `solid_to_mm_mesh` mesher.
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
///   half-space derives its AABB from the bounding region so it
///   must be finite.
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
) -> Result<Solid, CastError> {
    let bounds = bounding_region
        .bounds()
        .ok_or(CastError::InfiniteBounds(CastTarget::BoundingRegion))?;
    let halfspace = ribbon.halfspace_solid(side, bounds, RIBBON_PIECE_OVERLAP_M);
    let base_piece = bounding_region
        .clone()
        .subtract(layer_body.clone())
        .intersect(halfspace);

    // Step 9: inter-piece registration pins. `Negative` side
    // gains protrusions (union); `Positive` side gains matching
    // holes (subtract). When `ribbon.registration` is `None` the
    // helper returns `None` and the base piece flows through
    // unchanged — Step 5-8 callers see no behavior change.
    let piece_with_pins = match (build_registration_solid(ribbon), side) {
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
    Ok(piece)
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
    /// (SDF < 0).
    #[test]
    fn negative_piece_contains_cup_wall_below_ribbon() {
        let (body, region, ribbon) = fixture();
        let piece = compose_piece_solid(&body, &region, &ribbon, PieceSide::Negative).unwrap();
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
    /// positive piece).
    #[test]
    fn negative_piece_excludes_cup_wall_above_ribbon() {
        let (body, region, ribbon) = fixture();
        let piece = compose_piece_solid(&body, &region, &ribbon, PieceSide::Negative).unwrap();
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
        let neg = compose_piece_solid(&body, &region, &ribbon, PieceSide::Negative).unwrap();
        let pos = compose_piece_solid(&body, &region, &ribbon, PieceSide::Positive).unwrap();
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
        let neg = compose_piece_solid(&body, &region, &ribbon, PieceSide::Negative).unwrap();
        let pos = compose_piece_solid(&body, &region, &ribbon, PieceSide::Positive).unwrap();
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
    /// with opposite signs at the symmetric `+Z` point.
    #[test]
    fn pieces_partition_cup_material_symmetrically() {
        let (body, region, ribbon) = fixture();
        let neg = compose_piece_solid(&body, &region, &ribbon, PieceSide::Negative).unwrap();
        let pos = compose_piece_solid(&body, &region, &ribbon, PieceSide::Positive).unwrap();

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
    /// `-0.5 mm` along the seam.
    #[test]
    fn both_pieces_overlap_at_ribbon_seam() {
        let (body, region, ribbon) = fixture();
        let neg = compose_piece_solid(&body, &region, &ribbon, PieceSide::Negative).unwrap();
        let pos = compose_piece_solid(&body, &region, &ribbon, PieceSide::Positive).unwrap();
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
        let piece = compose_piece_solid(&body, &region, &ribbon, PieceSide::Negative).unwrap();
        // Final piece carries the bounding region's finite AABB.
        let aabb = piece
            .bounds()
            .expect("composition inherits bounding region's finite AABB");
        assert!(aabb.size().x > 0.0);
    }
}
