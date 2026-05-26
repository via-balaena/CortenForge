//! Per-piece SDF composition for v2 curve-following multi-piece molds.
//!
//! Step 5 of `docs/CURVE_FOLLOWING_DESIGN.md` §Algorithm §"Per-piece
//! SDF construction": for each layer × piece pair the mold-piece
//! geometry is the CSG composition
//!
//! ```text
//! piece_sdf      = bounding_region ∖ layer_body
//!                  ∩ ribbon_side(side)
//!                  ∪ registration pin solids        (Negative side)
//!                  ∖ registration socket solids     (Positive side)
//!                  ∖ plug-floor-lock socket          (both sides)
//! piece_meshcsg  = [pour-gate + vent SubtractCylinder]
//! ```
//!
//! The cup-piece [`Solid`] is **side-specific**: each
//! [`PieceSide`] returns its own half-shell via
//! [`Ribbon::halfspace_solid`]'s SDF half-space intersect (biased
//! inward by [`RIBBON_PIECE_OVERLAP_M`] so the two pieces overlap by
//! 1 mm at the seam), then unions / subtracts the registration
//! [`crate::PrismaticPin`][`crate::prismatic_pin`] solids per side
//! and subtracts the plug-floor-lock socket (side-agnostic single
//! solid; the per-side halfspace intersect bisects it laterally
//! across the seam by construction — S6 three-piece shared-
//! primitive invariant analog in SDF). Marching-cubes meshes the
//! composed half-shell; the remaining S7 pour-gate cylinders compose
//! post-MC via [`crate::mesh_csg::apply_mating_transforms`]. See
//! `docs/CF_CAST_SEAM_FACE_FILM_RECON_PLAN.md` §F-2 for the
//! recon-4 (P) seam architectural-correction rationale and
//! `docs/CF_CAST_FDM_FRIENDLY_GEOMETRY_RECON.md` §G-12 #2 for the
//! S3/S4 SDF-side architectural-correction rationale (the cup-pin
//! registration + plug-floor-lock primitives were mesh-CSG
//! `MatingTransform::UnionCylinder` / `SubtractCylinder` pre-S3 /
//! pre-S4; post-S4 they live entirely SDF-side per the §G-7
//! probe-spike outcome — `Manifold::hull_pts` truncated-pyramid
//! mesh-CSG union onto an SDF→MC curved-shell host fails at the
//! paradigm boundary, the §G-12 #2 SDF-union path clears).
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
//! the plane (§F-4 audit: bit-precise to f64 precision).
//! [`crate::mesh_csg::MatingTransform::SeamTrim`] remains in the
//! enum as a defensive primitive but is no longer emitted here.
//!
//! S3 of the FDM-friendly geometry arc (2026-05-24) migrated the
//! cup-pin registration mating features from mesh-CSG cylinders
//! (`MatingTransform::UnionCylinder` / `SubtractCylinder` consuming
//! a shared `crate::mesh_csg::CylinderParent`) to SDF-side
//! [`crate::PrismaticPin`][`crate::prismatic_pin`] solids composed
//! pre-MC into the half-shell. S4 (2026-05-24) migrated the
//! plug-floor lock from the S6 mesh-CSG plug-shaft + T-bar + T-slot
//! mechanism to a single SDF-side [`crate::PrismaticPin`] socket
//! subtracted from the half-shell (recon-1 §G-1 architectural
//! redesign — see [`crate::plug`] module docstring for the
//! cup-wall-penetration leak-path failure mode the pyramid lock
//! eliminates). The mesh-CSG `MatingTransform` variants stay for
//! the cup pour-gate carve (S7 of the prior mating-features arc).

use cf_design::{Sdf, Solid};
use nalgebra::{Point3, Vector3};

use crate::error::{CastError, CastTarget};
use crate::mesh_csg::MatingTransform;
use crate::plug::build_plug_lock_socket_transform;
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

/// MC bounds padding (1 mm) added to `max(wall_thickness_m, flange_width_m)`
/// when computing the cup-piece's MC sampling region. Ensures the MC
/// grid extends past the cup-wall outer surface AND the flange outer
/// edge by at least 1 mm so MC samples the SDF's "exterior" region
/// adjacent to every iso-surface (required for `marching_cubes` to
/// resolve the outer surface vertices, not just emit a clipped mesh
/// at the sampling-region boundary).
const MC_BOUNDS_PAD_M: f64 = 0.001;

/// SDF adapter that defines the cup-wall as a body-tracking SHELL of
/// thickness `wall_thickness_m` around `body`, rather than the
/// pre-§Q-1 `bounding_region.subtract(body)` cuboid-bounded form.
///
/// **Why this exists** (§Q-1, 2026-05-26): the old form used the
/// `bounding_region` cuboid as the cup-wall's outer boundary. When
/// the seam-flange S1 added a flange that extends 15 mm laterally
/// from the body perimeter — wider than the cuboid's `wall_thickness_m`
/// (5 mm) padding — the cuboid's faces ended up passing THROUGH the
/// flange's interior. At 2 mm MC cells, this produced topology-
/// ambiguous MC sample configurations at the cuboid-face × flange-
/// thickness-boundary intersection on Layer 2 cup pieces, surfacing
/// as 20-22 boundary edges (a closed-loop hole on the bounding
/// cuboid's face). See [[project-cf-cast-geometry-crispness-q1-finer-cells-blocked]].
///
/// The shell SDF replaces `bounding_region.subtract(body)` with
/// `max(body_dist - wall_thickness_m, -body_dist)` — the cup-wall is
/// the set of points OUTSIDE the body but within `wall_thickness_m`
/// of its surface. The cuboid disappears as an SDF surface; only the
/// body's outer surface and the shell's outer offset surface remain
/// as SDF zero-sets. No cuboid face × flange interior intersection
/// can exist.
///
/// **Behavior change**: pre-§Q-1, cup-wall thickness varied per
/// layer (because all layers shared the outer-most-layer-sized
/// bounding cuboid → inner layers had thicker walls). Post-§Q-1,
/// cup-wall thickness is uniform `wall_thickness_m` per layer.
/// Inner-layer molds become smaller / lighter; outer mold dims
/// vary per layer instead of being uniform. Workshop clamps grip
/// the flange face (body-tracking), not the cuboid outer, so this
/// is a workshop-neutral change.
///
/// SDF math (where `body_dist = body.evaluate(p)`):
/// - inside body (`body_dist < 0`): `max(neg - wall, pos) = pos`
///   → exterior (cup-wall doesn't extend into body cavity) ✓
/// - in shell (`0 < body_dist < wall_thickness_m`): `max(neg, neg)`
///   → interior (cup-wall material) ✓
/// - outside shell (`body_dist > wall_thickness_m`): `max(pos, neg)
///   = pos` → exterior ✓
///
/// Mirrors the [`crate::flange::FlangeSdf`] pattern — wraps a
/// `Solid` body's `evaluate` + composes via `max()`.
#[derive(Debug, Clone)]
struct CupWallShellSdf {
    body: Solid,
    wall_thickness_m: f64,
}

impl Sdf for CupWallShellSdf {
    fn eval(&self, p: Point3<f64>) -> f64 {
        let body_dist = self.body.evaluate(&p);
        (body_dist - self.wall_thickness_m).max(-body_dist)
    }

    fn grad(&self, _p: Point3<f64>) -> Vector3<f64> {
        // `max()` composition's analytical gradient is multi-valued at
        // facet boundaries. `Solid::from_sdf`'s `FieldNode::UserFn`
        // bridges via finite-differences, so this analytical grad is
        // unused. Pick +Z arbitrarily (matches `FlangeSdf::grad`'s
        // convention of an arbitrary unit vector).
        Vector3::new(0.0, 0.0, 1.0)
    }
}

/// Compose the per-(layer × piece) mold-piece geometry as the pair
/// `(Solid, Vec<MatingTransform>)`:
///
/// - **Solid** = `CupWallShellSdf(layer_body, wall_thickness_m)
///   ∩ ribbon.halfspace_solid(side, ...) [∪ flange]`
///   — side-specific half-shell tracking the body's outer surface.
///   Each [`PieceSide`] returns its own biased half-space intersect
///   ([`RIBBON_PIECE_OVERLAP_M`] grows each side 0.5 mm into the
///   other so the two pieces overlap by 1 mm at the seam). The
///   optional flange union (when `ribbon.flange` is
///   [`FlangeKind::Plate`][crate::flange::FlangeKind::Plate]) adds a
///   clamp-grip plate at the seam plane per
///   [`docs/CF_CAST_SEAM_FLANGE_RECON.md`] §F-13 S1.
///
/// **Post-§Q-1 (2026-05-26)**: the cup-wall is body-tracking via
/// [`CupWallShellSdf`], not bounded by a `bounding_region` cuboid.
/// See the [`CupWallShellSdf`] docstring +
/// [[project-cf-cast-geometry-crispness-q1-finer-cells-blocked]]
/// for the architectural-correction rationale. Pre-§Q-1 form was
/// `bounding_region.subtract(layer_body).intersect(halfspace)`;
/// the cuboid faces produced MC topology ambiguity at finer cells
/// when the flange's lateral reach exceeded the cuboid's padding.
/// - **`Vec<MatingTransform>`** =
///   1. Pour-gate + vent
///      [`crate::mesh_csg::MatingTransform::SubtractCylinder`] ops
///      (S7). The full-length cylinder is subtracted from the
///      half-shell; the SDF halfspace handles per-side bisection by
///      construction (cylinder portion outside the half-shell is a
///      no-op).
///
/// Post-S3 / post-S4 of the FDM-friendly geometry arc, all mating
/// features other than the pour-gate live SDF-side (per
/// `docs/CF_CAST_FDM_FRIENDLY_GEOMETRY_RECON.md` §G-12 #2) —
/// composed into the per-piece [`Solid`] above the half-shell
/// intersect, NOT in the `Vec<MatingTransform>`. The Negative side
/// unions [`crate::PrismaticPin`][`crate::prismatic_pin`] cup-pin
/// solids (workshop-visible ridge protrudes past the half-shell
/// seam face); the Positive side subtracts cup-pin socket solids
/// (matching cavity carved from cup-wall material). BOTH sides
/// subtract the plug-floor-lock socket solid (side-agnostic single
/// solid; per-side halfspace intersect bisects it laterally across
/// the seam by construction — S6 three-piece shared-primitive
/// invariant analog in SDF). See
/// [`crate::registration::build_registration_sdf_ops`] for the
/// cup-pin pose derivation and
/// [`crate::plug::build_plug_lock_socket_sdf`] for the plug-floor
/// lock socket emission.
///
/// The cup-piece Solid's SDF is **negative inside the cup-wall
/// material** on the half-shell side; the side-specific cut is
/// already baked into the Solid via the ribbon's half-space
/// intersect. `apply_mating_transforms` then runs the remaining S6 +
/// S7 cylinder ops post-MC against the half-shell mesh; cylinder
/// portions outside the half-shell volume contribute nothing (a
/// no-op for `SubtractCylinder`).
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
/// `layer_body` is taken by reference and cloned into the result;
/// the underlying [`cf_design::Solid`] is internally `Arc`-shared
/// so cloning is cheap (no geometry duplication).
///
/// # Errors
///
/// - [`CastError::InfiniteBounds`] with [`CastTarget::LayerBody`]
///   (post-§Q-1, 2026-05-26) if `layer_body.bounds()` returns
///   `None`. The cup-piece's MC sampling bounds derive from the
///   body's AABB expanded by `max(wall_thickness_m,
///   flange_width_m) + [MC_BOUNDS_PAD_M]`. Pre-§Q-1 the bounds came
///   from `bounding_region` and an unbounded `layer_body` was
///   allowed; post-§Q-1 the body must be bounded. The error's
///   `layer_index` is hardcoded to `0` because `compose_piece_solid`
///   doesn't receive the caller's layer index — in production all
///   per-layer bodies are bounded so this defensive guard only
///   fires in test fixtures that explicitly construct unbounded
///   `Solid::plane` bodies.
pub fn compose_piece_solid(
    layer_body: &Solid,
    wall_thickness_m: f64,
    ribbon: &Ribbon,
    side: PieceSide,
) -> Result<(Solid, Vec<MatingTransform>), CastError> {
    // §Q-1 (2026-05-26): MC bounds are body_bounds expanded by
    // max(wall_thickness, flange_width) + ε. The cup-wall shell
    // SDF + flange SDF must BOTH sample within this region for MC
    // to resolve their outer surfaces correctly. Pre-§Q-1 the
    // bounds came from `bounding_region.bounds()` which was a
    // cuboid sized only for `wall_thickness_m` past the body — too
    // small to enclose the flange's 15 mm lateral reach. See
    // [[project-cf-cast-geometry-crispness-q1-finer-cells-blocked]].
    let body_bounds =
        layer_body
            .bounds()
            .ok_or(CastError::InfiniteBounds(CastTarget::LayerBody {
                layer_index: 0,
            }))?;
    let flange_extent = ribbon.flange.spec().map_or(0.0, |s| s.flange_width_m);
    let mc_pad = wall_thickness_m.max(flange_extent) + MC_BOUNDS_PAD_M;
    let mc_bounds = body_bounds.expanded(mc_pad);

    // Pre-S4 / recon-4 (P) SDF seam: side-specific half-space
    // intersect at the SDF level. The biased halfspace's SDF is
    // exactly linear; MC interpolates seam-cap vertices on the seam
    // plane to f64 precision per §F-4. S5/S6/S7 mating-features
    // mesh-CSG cylinder primitives compose post-MC against the
    // resulting half-shell mesh.
    let halfspace = ribbon.halfspace_solid(side, mc_bounds, RIBBON_PIECE_OVERLAP_M);

    // §Q-1: cup-wall as body-tracking shell, not bounding cuboid.
    // See [`CupWallShellSdf`] docstring for the architectural
    // rationale. The shell SDF + `mc_bounds` give MC a clean
    // sampling region around the body's outer shell with no
    // cuboid SDF zero-set passing through flange interior.
    let shell = Solid::from_sdf(
        CupWallShellSdf {
            body: layer_body.clone(),
            wall_thickness_m,
        },
        mc_bounds,
    );
    let mut base_piece = shell.intersect(halfspace);

    // S1 of `docs/CF_CAST_SEAM_FLANGE_RECON.md` §F-13: when
    // `ribbon.flange` is `FlangeKind::Plate(spec)`, union the
    // per-side flange SDF into the cup-piece base. The flange is a
    // flat slab at the seam plane extending OUTWARD from the body
    // cavity perimeter. Pre-§Q-1 the flange's halfspace cut
    // explicitly used zero overlap_m so the cup-wall × flange
    // junction was flush; post-§Q-1 the cup-wall is body-tracking
    // (shell SDF) and the flange's outer reach is fully inside
    // `mc_bounds`, so no cuboid face × flange interaction can
    // produce MC ambiguity. `FlangeKind::None` short-circuits.
    if let Some(flange_spec) = ribbon.flange.spec() {
        let flange = crate::flange::build_flange_solid_for_side(
            layer_body,
            ribbon,
            flange_spec,
            mc_bounds,
            side,
        );
        base_piece = base_piece.union(flange);
    }

    // Post-mortem salvage (2026-05-24): cup-pin registration AND the
    // S4 plug-floor-lock socket are now emitted as POST-MC mesh-CSG
    // transforms via [`MatingTransform::UnionTruncatedPyramid`] /
    // [`SubtractTruncatedPyramid`]. Restores the prior
    // mating-features arc S5/S6 architecture (POST-MC primitives
    // carry their own native hull resolution; pose discipline per
    // the recon-4 (P) framework keeps placement CONTAINED /
    // PROTRUDING). The plug-lock socket is **side-agnostic** —
    // both Negative + Positive cup pieces apply the same
    // SubtractTruncatedPyramid to carve the matching cavity from
    // whichever half of cup-wall material exists on each piece.
    // See [[feedback-read-prior-arc-memory-before-architectural-decisions]]
    // for the post-mortem on why pre-MC SDF composition (the prior
    // S3/S4 architecture) was abandoned.
    //
    // §Q-1 (2026-05-26): registration pin placement now uses
    // `wall_thickness_m` for the cup-wall outer distance (was:
    // surface_distance_along_ray(bounding_region, ...) at the
    // OLD cuboid face). Pin position = body + wall_thickness/2 in
    // the shell midpoint.
    let mut transforms = build_pour_gate_transforms(ribbon);
    transforms.extend(build_registration_transforms(
        ribbon,
        layer_body,
        wall_thickness_m,
        side,
    ));
    if let Some(plug_lock_socket) = build_plug_lock_socket_transform(ribbon) {
        transforms.push(plug_lock_socket);
    }
    // **Cup-side cap-plane trim DISABLED 2026-05-24 night** — same
    // recon-4 (P) §F-2 paradigm-boundary issue that blocked the
    // plug-side trim. See `add_plug_pins` in `plug.rs` for the
    // full rationale + follow-up bookmark.

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
        let (body, _region, ribbon) = fixture();
        let (piece, _) = compose_piece_solid(&body, 0.020, &ribbon, PieceSide::Negative).unwrap();
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
        let (body, _region, ribbon) = fixture();
        let (piece, _) = compose_piece_solid(&body, 0.020, &ribbon, PieceSide::Negative).unwrap();
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
        let (body, _region, ribbon) = fixture();
        // A point well inside the body but offset to one side of the
        // ribbon — without the body subtraction the negative piece
        // would CONTAIN this point.
        let q = Point3::new(0.0, 0.0, -0.005);
        let (neg, _) = compose_piece_solid(&body, 0.020, &ribbon, PieceSide::Negative).unwrap();
        let (pos, _) = compose_piece_solid(&body, 0.020, &ribbon, PieceSide::Positive).unwrap();
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
        let (body, _region, ribbon) = fixture();
        // Far outside the 30 mm half-extent bounding region.
        let q = Point3::new(0.10, 0.0, 0.0);
        let (neg, _) = compose_piece_solid(&body, 0.020, &ribbon, PieceSide::Negative).unwrap();
        let (pos, _) = compose_piece_solid(&body, 0.020, &ribbon, PieceSide::Positive).unwrap();
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
        let (body, _region, ribbon) = fixture();
        let (neg, _) = compose_piece_solid(&body, 0.020, &ribbon, PieceSide::Negative).unwrap();
        let (pos, _) = compose_piece_solid(&body, 0.020, &ribbon, PieceSide::Positive).unwrap();

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
        let (body, _region, ribbon) = fixture();
        let (neg, _) = compose_piece_solid(&body, 0.020, &ribbon, PieceSide::Negative).unwrap();
        let (pos, _) = compose_piece_solid(&body, 0.020, &ribbon, PieceSide::Positive).unwrap();
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
        _bounding: &Solid,
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
        let (solid, transforms) = compose_piece_solid(body, 0.020, ribbon, side).unwrap();
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

    /// Unbounded layer body surfaces as
    /// `CastError::InfiniteBounds(LayerBody {layer_index: 0})` —
    /// post-§Q-1 (2026-05-26) the cup-piece's MC bounds derive
    /// from `layer_body.bounds()`, not `bounding_region.bounds()`.
    /// An unbounded body (e.g., a bare `Solid::plane`) can't be
    /// expanded by `wall_thickness_m`, so `compose_piece_solid`
    /// errors at the entry. Pre-§Q-1 this test exercised the
    /// opposite — unbounded bounding region was the error, unbounded
    /// body was OK.
    #[test]
    fn unbounded_layer_body_returns_infinite_bounds_error() {
        let body = Solid::plane(Vector3::new(0.0, 0.0, 1.0), 0.0);
        let centerline = vec![Point3::new(-0.05, 0.0, 0.0), Point3::new(0.05, 0.0, 0.0)];
        let split = SplitNormal::new(Vector3::new(0.0, 1.0, 0.0)).unwrap();
        let ribbon = Ribbon::new(centerline, split).unwrap();
        let err = compose_piece_solid(&body, 0.020, &ribbon, PieceSide::Negative).unwrap_err();
        assert!(
            matches!(
                err,
                CastError::InfiniteBounds(CastTarget::LayerBody { layer_index: 0 })
            ),
            "expected InfiniteBounds(LayerBody), got {err:?}",
        );
    }

    // ----- Sub-leaf A regression: workshop iter-1 mold visual-gate -
    //
    // The wide-body pin-anchor regression moved into
    // `crate::registration::tests::cup_pin_sdf_centre_stays_in_cup_wall_for_wide_body`
    // when S3 of the FDM-friendly geometry arc migrated the cup-pin
    // primitive from mesh-CSG cylinder transforms to SDF-side
    // PrismaticPin solids — the new test asserts the SAME annulus-
    // midpoint invariant on the SDF emission path.

    /// Workshop iter-1 plug-socket regression preserved post-salvage.
    ///
    /// Pre-S6 this verified the SDF-era plug-shaft socket carved
    /// cup-wall material at the cap-plane centroid. Post-S6 the
    /// invariant moved to the mesh-CSG `SubtractCylinder`
    /// transform's parent center. Pre-2026-05-24-salvage S4 the
    /// plug-floor lock socket lived SDF-side again (§G-12 #2)
    /// and this asserted on the cup-piece SDF reporting EXTERIOR
    /// at the probe. Post-2026-05-24-salvage the socket is back
    /// on the mesh-CSG post-MC path per
    /// [[feedback-read-prior-arc-memory-before-architectural-decisions]]
    /// — the invariant moves to the transform-parameter layer:
    /// `compose_piece_solid` emits a `SubtractTruncatedPyramid`
    /// transform whose `params.pose.center_m` anchors at the
    /// cap-plane centroid (NOT at the trimmed centerline tip),
    /// and the transform's params shape match the plug-side lock
    /// pyramid (modulo the symmetric `/2` clearance inflate).
    #[test]
    fn plug_lock_socket_carves_cup_material_at_cap_plane_centroid_past_trimmed_centerline_tip() {
        use crate::mesh_csg::MatingTransform;
        use crate::plug::{PlugPinKind, PlugPinSpec};

        let centerline = vec![
            Point3::new(0.0, 0.0, 0.073),
            Point3::new(0.0, 0.0, 0.020),
            Point3::new(0.0, 0.0, -0.013), // trimmed end (40 mm above cap)
        ];
        let split = SplitNormal::new(Vector3::new(1.0, 0.0, 0.0)).unwrap();
        let cap_centroid = Point3::new(0.0, 0.0, -0.054);
        let cap_normal = Vector3::new(0.0, 0.0, -1.0);
        let ribbon = Ribbon::new(centerline, split)
            .unwrap()
            .with_pour_end_hint(cap_centroid, cap_normal)
            .with_plug_pins(PlugPinKind::Axial(PlugPinSpec::iter1()));

        let body = Solid::cuboid(Vector3::new(0.020, 0.020, 0.040)).translate(Vector3::new(
            0.0,
            0.0,
            -0.054 + 0.040,
        ));
        let _bounding = Solid::cuboid(Vector3::new(0.030, 0.030, 0.090));

        for side in [PieceSide::Negative, PieceSide::Positive] {
            let (_piece, transforms) = compose_piece_solid(&body, 0.020, &ribbon, side).unwrap();

            let socket_transform = transforms
                .iter()
                .find_map(|t| match t {
                    MatingTransform::SubtractTruncatedPyramid { params } => Some(params),
                    _ => None,
                })
                .unwrap_or_else(|| {
                    panic!(
                        "{side:?} compose_piece_solid must emit a SubtractTruncatedPyramid for \
                         the plug-lock socket; got transforms = {transforms:#?}",
                    )
                });

            // Cap-plane-centroid anchor regression: pose center
            // lands at cap_centroid (z = -0.054), NOT at the
            // trimmed centerline tip (z = -0.013).
            assert!(
                (socket_transform.pose.center_m.z - cap_centroid.z).abs() < 1e-12,
                "{side:?} socket pose centre must anchor at cap_centroid \
                 (z = {}); got centre z = {} (cap-plane regression — pre-salvage was at \
                 trimmed centerline tip z = -0.013)",
                cap_centroid.z,
                socket_transform.pose.center_m.z,
            );
            assert!(
                (socket_transform.pose.axis_unit.z - 1.0).abs() < 1e-12,
                "{side:?} socket pose axis must align with -cap_normal = +Z post-2026-05-24 \
                 base-down captive-lock flip; got axis.z = {}",
                socket_transform.pose.axis_unit.z,
            );

            // Spec-match invariant: the socket's params come from
            // `PrismaticPinSpec::socket_params` (i.e., the spec's
            // inflated extents). half_length must equal
            // `lock_spec.pin_half_length_m + axial_clearance_m / 2`.
            let spec = PlugPinSpec::iter1();
            let expected_half_length =
                spec.lock_spec.pin_half_length_m + spec.lock_spec.axial_clearance_m / 2.0;
            assert!(
                (socket_transform.half_length_m - expected_half_length).abs() < 1e-12,
                "{side:?} socket half_length must match the spec's pin half-length + \
                 axial_clearance/2; expected {expected_half_length}, got {}",
                socket_transform.half_length_m,
            );
        }
    }

    // S4 (FDM-friendly geometry arc) deleted the
    // `t_bar_halves_share_coplanar_seam_face` test — the T-bar
    // mesh-CSG mechanism was retired along with the entire plug-
    // shaft + T-bar + T-slot retention path (replaced by the
    // truncated-pyramid plug-floor lock per recon-1 §G-1). The
    // post-S4 analog of "seam bisects the plug-retention feature
    // through its center" is captured by the cup-piece SDF subtract
    // in `compose_piece_solid` — both cup halves' halfspace
    // intersect bisect the side-agnostic socket SDF laterally,
    // verified at the SDF level by
    // `plug_lock_socket_carves_cup_material_at_cap_plane_centroid_past_trimmed_centerline_tip`.

    /// Cup-piece SDF half-shell connectivity guard — locks the
    /// recon-4 §F-3b invariant at the lib-test level: the SDF
    /// halfspace intersect `bounding ∖ body ∩ halfspace(side)`
    /// must mesh as 1 connected component (the kept half-shell,
    /// with the halfspace cap closing the body-cavity tube). The
    /// pre-S4 form was already verified on a synthetic axis-aligned
    /// cuboid by `crate::mesh_csg::tests::f3b_synthetic_presdf_seam_baseline_is_one_component`;
    /// this test extends the invariant to the production composition
    /// path (`compose_piece_solid` → MC) for BOTH `PieceSide`s
    /// without registration pins (the bare half-shell connectivity
    /// is the load-bearing recon-4 §F-2 architectural claim).
    ///
    /// Full mating-features connectivity (cup body + registration
    /// pins via mesh-CSG union) is verified at the integration tier
    /// — `~/scans/cast_iter1/mold_layer_{0..2}_piece_{0,1}.stl`
    /// regen post-recon-4 shows all 6 production cup-piece STLs
    /// PASS the §R1 inspector at 1 connected component each
    /// (~50k verts / ~17k tris each — production scale + capsule-
    /// body curvature). A synthetic cuboid fixture for the
    /// full-mating-features path produces 5 components for reasons
    /// orthogonal to the half-shell invariant (manifold3d's mesh-CSG
    /// union behavior on small synthetic cuboid hosts with
    /// 4 contained pins differs from production capsule-body
    /// geometry); the half-shell baseline captured here is the
    /// load-bearing recon-4 (P) regression guard.
    #[test]
    fn cup_piece_half_shell_is_single_connected_component() {
        use mesh_repair::components::find_connected_components;

        let (layer_body, bounding_region, ribbon) = fixture();

        for side in [PieceSide::Negative, PieceSide::Positive] {
            // 3 mm cells: matches iter-1's production
            // `mesh_cell_size_m` (workshop cast.toml default).
            let mesh =
                mesh_piece_through_p_pipeline(&layer_body, &bounding_region, &ribbon, side, 0.003);
            let analysis = find_connected_components(&mesh);
            assert_eq!(
                analysis.component_count,
                1,
                "{side:?} cup-piece SDF half-shell must mesh as 1 connected \
                 component (recon-4 (P) §F-2 architectural claim: the SDF \
                 halfspace intersect closes the body-cavity tube into one closed \
                 half-shell, replacing S4's 2-shell-hollow-cuboid + film artifact); \
                 got {} components ({} verts / {} faces). A regression here means \
                 the SDF seam reverted to S4's post-MC SeamTrim (the recon-4 (P) \
                 architectural correction was undone).",
                analysis.component_count,
                mesh.vertices.len(),
                mesh.faces.len(),
            );
        }
    }

    // ─── Seam-flange S1 paired-baseline tests (recon §F-13) ──────────
    // Per [[feedback-load-bearing-test-fixtures]]: pair every "with
    // flange" probe with a "without flange" probe, on the same probe
    // coordinate. Tests verify (1) FlangeKind::None preserves the
    // pre-S1 SDF bit-for-bit (backward-compat), (2) FlangeKind::Plate
    // ADDS material in the flange's lateral+vertical region (union
    // semantics with non-empty contribution), and (3) at the body
    // perimeter (gasket strip region) the flange contributes NO
    // material per the §F-4 lateral-disjoint invariant.
    //
    // **Fixture choice (cold-read pass-1 of S1):** these tests use
    // `flange_test_fixture()` with `split_normal = (0, 0, 1)` →
    // binormal = -Y → seam plane perpendicular to Y. This matches
    // `FlangeSdf::eval`'s `|p.y - seam_plane_y| - thickness` Y-axis
    // hardcoding (recon §F-1 simplification). The piece-level
    // `fixture()` used by other tests in this module has
    // `split_normal = +Y` → binormal = +Z → seam plane perpendicular
    // to Z, which would rotate the flange 90° from the actual seam
    // boundary — invalid for flange validation. Production
    // `cast.toml` uses `split_normal = +X` (binormal also -Y), so
    // `flange_test_fixture()` is geometrically equivalent to the
    // production setup.

    /// Flange-test fixture aligned with the `FlangeSdf` Y-hardcoding:
    /// - body: cylinder along X (10 mm radius, 30 mm half-length —
    ///   total 60 mm long; caps at |X| = 30 mm) — matches gasket-
    ///   mold S1 synthetic-fixture body
    /// - bounding region: cuboid (100, 15, 12) half-extents = 200 ×
    ///   30 × 24 mm — body fits inside with cup-wall thickness ~5 mm
    ///   in the radial direction
    /// - ribbon: centerline along +X through origin, `split_normal =
    ///   +Z` → binormal = -Y → seam plane = XZ (Y=0). Matches
    ///   `FlangeSdf`'s Y-hardcoded seam-plane convention.
    fn flange_test_fixture() -> (Solid, Solid, Ribbon) {
        let layer_body =
            Solid::cylinder(0.010, 0.030).rotate(nalgebra::UnitQuaternion::from_axis_angle(
                &Vector3::y_axis(),
                std::f64::consts::FRAC_PI_2,
            ));
        let bounding_region = Solid::cuboid(Vector3::new(0.100, 0.015, 0.012));
        let centerline = vec![Point3::new(-0.05, 0.0, 0.0), Point3::new(0.05, 0.0, 0.0)];
        let split = SplitNormal::new(Vector3::new(0.0, 0.0, 1.0)).unwrap();
        let ribbon = Ribbon::new(centerline, split).unwrap();
        (layer_body, bounding_region, ribbon)
    }

    #[test]
    #[allow(clippy::float_cmp)]
    fn flange_kind_none_preserves_cup_piece_sdf_bit_for_bit() {
        // §F-5 backward-compat invariant (rewritten post-§Q-1
        // 2026-05-26): `FlangeKind::None` (default post-S1) MUST
        // produce a cup-piece SDF bit-identical to the bare cup-wall
        // shell path. Pre-§Q-1 the bare form was
        // `region.subtract(body).intersect(halfspace)`; post-§Q-1 the
        // bare form is `CupWallShellSdf.intersect(halfspace)` because
        // the cup-wall is body-tracking, not cuboid-bounded. See
        // [[project-cf-cast-geometry-crispness-q1-finer-cells-blocked]].
        let (body, _region, ribbon) = fixture();
        assert_eq!(ribbon.flange, crate::flange::FlangeKind::None);

        let wall_thickness_m = 0.020;
        let (piece_none, _) =
            compose_piece_solid(&body, wall_thickness_m, &ribbon, PieceSide::Negative).unwrap();

        // Spot-check probes spanning inside-wall + outside + at-seam.
        // Coords picked relative to body extent (10 mm half-extent →
        // body_dist 10 mm at probe (0, 0.020, 0) etc) so they don't
        // depend on the pre-§Q-1 bounding-cuboid extent.
        let probes = [
            Point3::new(0.0, 0.0, 0.020),  // outside body Z, in cup wall
            Point3::new(0.0, -0.020, 0.0), // outside body Y, in Negative-side cup wall
            Point3::new(0.0, 0.0, 0.040),  // outside cup wall entirely
            Point3::new(0.0, 0.0, 0.005),  // inside body cavity
        ];

        // Hand-roll the post-§Q-1 "bare" composition: shell SDF
        // intersected with halfspace, NO flange union.
        let body_bounds = body.bounds().unwrap();
        let mc_bounds = body_bounds.expanded(wall_thickness_m + MC_BOUNDS_PAD_M);
        let halfspace =
            ribbon.halfspace_solid(PieceSide::Negative, mc_bounds, RIBBON_PIECE_OVERLAP_M);
        let shell = Solid::from_sdf(
            CupWallShellSdf {
                body,
                wall_thickness_m,
            },
            mc_bounds,
        );
        let bare = shell.intersect(halfspace);

        for probe in &probes {
            let v_none = piece_none.evaluate(probe);
            let v_bare = bare.evaluate(probe);
            assert_eq!(
                v_none, v_bare,
                "FlangeKind::None must be bit-identical to bare shell-based cup-piece SDF \
                 at {probe:?}; got piece-with-None={v_none} vs bare={v_bare}"
            );
        }
    }

    #[test]
    fn flange_plate_adds_material_outside_cup_wall_at_seam_plane() {
        // §F-13 S1 paired-baseline test for "Plate adds material
        // where None has none". The probe is positioned OUTSIDE the
        // cup-wall shell (body_dist > wall_thickness — shell has no
        // material there) BUT INSIDE the flange's lateral+vertical
        // region. Under None: piece_sdf > 0 (no material). Under
        // Plate: the flange union adds material → piece_sdf < 0.
        //
        // Fixture: cylinder body along X (radius 10 mm in YZ). The
        // flange extends `flange_width_m` = 15 mm radially from the
        // cylinder surface; we use `wall_thickness_m` = 0.005 (5 mm)
        // so the band body_dist ∈ [5 mm, 15 mm] is "outside cup-wall
        // shell" but "inside flange lateral reach". A probe at
        // Z = +0.020 m (body_dist = 0.020 - 0.010 = 0.010 = 10 mm)
        // lands in that band.
        //
        // Post-§Q-1 (2026-05-26): pre-refactor this used wall_thickness
        // derived from a bounding cuboid; post-refactor the cup-wall
        // is a body-tracking shell, so the probe coordinates moved
        // from "past bounding cuboid face" to "past shell outer
        // surface in body_dist sense". See
        // [[project-cf-cast-geometry-crispness-q1-finer-cells-blocked]].
        let (body, _region, ribbon_none) = flange_test_fixture();
        let spec = crate::flange::FlangeSpec::iter1();
        let ribbon_plate = ribbon_none
            .clone()
            .with_flange(crate::flange::FlangeKind::Plate(spec));
        let wall_thickness_m = 0.005;

        // Probe in flange-only band (5 mm < body_dist < 15 mm).
        // body_dist at (0, 0.001, 0.020) ≈ sqrt(0.001² + 0.020²) -
        // 0.010 ≈ 0.010 m = 10 mm. Y=+0.001 is on Negative side
        // for this fixture (split=+Z → binormal=-Y → Negative covers
        // +Y region per §F-test "per-side halfspace cut" finding).
        let probe = Point3::new(0.0, 0.001, 0.020);

        let (piece_none, _) =
            compose_piece_solid(&body, wall_thickness_m, &ribbon_none, PieceSide::Negative)
                .unwrap();
        let (piece_plate, _) =
            compose_piece_solid(&body, wall_thickness_m, &ribbon_plate, PieceSide::Negative)
                .unwrap();

        let sdf_none = piece_none.evaluate(&probe);
        let sdf_plate = piece_plate.evaluate(&probe);

        assert!(
            sdf_none > 0.0,
            "FlangeKind::None: probe at body_dist≈10 mm (outside 5 mm cup-wall shell) \
             must be OUTSIDE cup material; got sdf_none={sdf_none}"
        );
        assert!(
            sdf_plate < 0.0,
            "FlangeKind::Plate: probe at body_dist≈10 mm (inside flange's [2 mm, 15 mm] \
             lateral reach + thickness region) must be INSIDE material via flange \
             union; got sdf_plate={sdf_plate}"
        );
        assert!(
            sdf_plate < sdf_none,
            "Plate must produce more-inside SDF than None at flange-only probe \
             (union added material); plate={sdf_plate} vs none={sdf_none}"
        );
    }

    #[test]
    fn flange_plate_does_not_extend_to_body_perimeter() {
        // §F-4 gasket-disjoint invariant: at the body cavity perimeter
        // (body_dist = 0), the flange's `inner_offset_m` guard
        // ensures NO flange material is present (so the gasket strip
        // can sit at body_dist ≈ 0 without PLA pinching it). Under
        // both None and Plate the cup-piece SDF at body_dist = 0
        // must be identical within FP precision.
        let (body, _region, ribbon_none) = flange_test_fixture();
        let ribbon_plate = ribbon_none
            .clone()
            .with_flange(crate::flange::FlangeKind::Plate(
                crate::flange::FlangeSpec::iter1(),
            ));

        // Probe AT body cavity perimeter on the seam plane: cylinder
        // along X with radius 10 mm; at (0, 0, 0.010) the radial
        // distance from the cylinder axis is exactly 10 mm = body
        // surface. body_dist = 0.
        let probe = Point3::new(0.0, 0.0, 0.010);

        let (piece_none, _) =
            compose_piece_solid(&body, 0.020, &ribbon_none, PieceSide::Negative).unwrap();
        let (piece_plate, _) =
            compose_piece_solid(&body, 0.020, &ribbon_plate, PieceSide::Negative).unwrap();

        let sdf_none = piece_none.evaluate(&probe);
        let sdf_plate = piece_plate.evaluate(&probe);

        let dt = (sdf_none - sdf_plate).abs();
        assert!(
            dt < 1e-9,
            "Flange must NOT alter SDF at body perimeter (gasket-disjoint \
             invariant §F-4); diff {dt} between plate={sdf_plate} and none={sdf_none}"
        );
    }
}
