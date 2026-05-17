//! Decomposed SDF oracles.
//!
//! [`UnsignedDistance`] and [`Sign`] are orthogonal oracles whose
//! conflation was a recurring bug source — consumers couldn't tell from
//! a single `SignedDistanceField` API whether they needed extra sign
//! defense, and three callers shipped three different ad-hoc postures
//! (`cf-device-design::sdf_layers`, `cf-device-design::insertion_sim`,
//! `cf-cast-cli`). Splitting them lets the type carry the choice:
//! `Signed<TriMeshDistance, PseudoNormalSign>` is fast + fragile on
//! cleaned scans; `Signed<TriMeshDistance, FloodFillSign>` is robust at
//! the cost of a one-shot grid build.
//!
//! See `docs/MESH_SDF_ORACLE_DECOMPOSITION_SPEC.md` for the full
//! architectural rationale and migration plan.

use nalgebra::Point3;
use thiserror::Error;

/// Unsigned-distance oracle.
///
/// `distance(p)` returns the absolute distance from `p` to the nearest
/// surface (≥ 0). `closest_point(p)` returns the closest point on the
/// surface. Both must be deterministic and finite for any finite `p`.
///
/// Implementations should not panic on non-manifold input — the
/// distance to the nearest surface is well-defined regardless of
/// closure.
pub trait UnsignedDistance: Send + Sync {
    /// Absolute distance from `p` to the nearest surface (≥ 0).
    fn distance(&self, p: Point3<f64>) -> f64;

    /// Closest point on the surface to `p`.
    fn closest_point(&self, p: Point3<f64>) -> Point3<f64>;
}

/// Sign oracle.
///
/// `is_inside(p)` returns `true` iff `p` is in the closed body's
/// interior. Implementations differ in (a) what "inside" means for
/// non-manifold input and (b) cost — see the concrete impls
/// ([`crate::PseudoNormalSign`], `FloodFillSign` in D.2) for posture.
///
/// Boundary-point behavior is implementation-defined; do not probe
/// exactly on the surface and expect a particular branch.
pub trait Sign: Send + Sync {
    /// `true` iff `p` is inside the closed body.
    fn is_inside(&self, p: Point3<f64>) -> bool;
}

/// Composition of an [`UnsignedDistance`] oracle and a [`Sign`] oracle
/// into a signed-distance source.
///
/// `evaluate(p) = if sign.is_inside(p) { -distance.distance(p) } else
/// { distance.distance(p) }`. The two oracles are independent — pick
/// `D` for distance precision (a parry BVH, a cached grid, a CSG of
/// primitives) and `S` for sign robustness (parry's pseudo-normals,
/// flood-fill, generalized winding number).
///
/// `Signed<D, S>` implements `cf_design::Sdf` via the blanket
/// `impl<D: UnsignedDistance, S: Sign> Sdf for Signed<D, S>` in
/// cf-design — so the composition drops into `Solid::from_sdf` and the
/// `Arc<dyn Sdf>` consumer pattern without ceremony.
///
/// **Pub fields + same-name methods.** The `distance: D` field shares
/// a name with the [`Signed::distance`] method — Rust resolves
/// `sdf.distance(p)` to the method (takes args) and `sdf.distance` to
/// the field. Direct field access bypasses the sign oracle; reach for
/// the method unless you're consciously composing on the unsigned
/// oracle alone.
#[derive(Debug, Clone)]
pub struct Signed<D: UnsignedDistance, S: Sign> {
    /// Unsigned-distance oracle.
    pub distance: D,
    /// Sign oracle.
    pub sign: S,
}

impl<D: UnsignedDistance, S: Sign> Signed<D, S> {
    /// Signed distance at `p` (negative inside, positive outside).
    #[must_use]
    pub fn evaluate(&self, p: Point3<f64>) -> f64 {
        let u = self.distance.distance(p);
        if self.sign.is_inside(p) { -u } else { u }
    }

    /// Alias for [`Signed::evaluate`] — preserves the
    /// `SignedDistanceField::distance` API shape so deprecated call
    /// sites work unchanged.
    #[must_use]
    pub fn distance(&self, p: Point3<f64>) -> f64 {
        self.evaluate(p)
    }

    /// Unsigned distance — delegates to the wrapped distance oracle.
    #[must_use]
    pub fn unsigned_distance(&self, p: Point3<f64>) -> f64 {
        self.distance.distance(p)
    }

    /// Inside test — delegates to the wrapped sign oracle.
    #[must_use]
    pub fn is_inside(&self, p: Point3<f64>) -> bool {
        self.sign.is_inside(p)
    }

    /// Closest point on the surface — delegates to the wrapped
    /// distance oracle.
    #[must_use]
    pub fn closest_point(&self, p: Point3<f64>) -> Point3<f64> {
        self.distance.closest_point(p)
    }
}

/// Three-valued region tag produced by `FloodFillSign`'s
/// classification pass (D.2).
///
/// - `Inside`/`Outside`: the cell's center sits unambiguously inside
///   or outside the body.
/// - `Wall`: the cell straddles the surface (its unsigned distance is
///   within the wall-band threshold). Wall cells block the BFS so the
///   "outside" flood cannot leak across the surface to a neighboring
///   "inside" cell. After the flood, a second pass labels every wall
///   cell with its nearest non-wall region so every cell carries a
///   sign.
///
/// Exposed at the mesh-sdf API surface even though no public function
/// constructs it yet — D.2 fills in `FloodFillSign::build`, which
/// surfaces `FloodFillReport` whose `region_counts` is keyed by
/// [`Region`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Region {
    /// Cell center is unambiguously outside the body.
    Outside,
    /// Cell center is unambiguously inside the body.
    Inside,
    /// Cell straddles the surface (within the wall-band threshold).
    Wall,
}

/// Failure modes for `FloodFillSign::build` (D.2).
///
/// All variants describe construction-time failures; once built, the
/// flood-fill sign returns deterministic `is_inside` values for every
/// point in the build domain.
#[derive(Debug, Error)]
pub enum FloodFillError {
    /// Every corner of the supplied bounds is within the wall band of
    /// the surface. There is no "outside seed" from which the flood
    /// can propagate. Caller should expand the bounds margin until at
    /// least one corner sits outside the wall.
    #[error(
        "flood-fill seed pass failed: all 8 corner cells are within {wall_threshold_m:.6} m \
         of the surface (the wall band). The bounds margin must extend further past the body \
         so at least one corner can serve as an outside seed."
    )]
    NoOutsideSeed {
        /// The wall-band threshold (meters) used during construction.
        wall_threshold_m: f64,
    },

    /// Construction bounds are degenerate (one or more axis spans
    /// non-positive) so the grid has no cells.
    #[error("flood-fill build failed: degenerate bounds (min ≥ max along at least one axis)")]
    DegenerateBounds,

    /// Construction `cell_size` is non-positive.
    #[error("flood-fill build failed: non-positive cell_size {cell_size:.6} m")]
    NonPositiveCellSize {
        /// The cell size (meters) the caller supplied.
        cell_size: f64,
    },
}

/// Diagnostic summary produced by `FloodFillSign::build` (D.2).
///
/// `inside_components == 1` is the healthy outcome on watertight
/// input; `> 1` indicates the flood detected multiple disconnected
/// interior regions (a cavity inside a cavity, or — more commonly —
/// a flood-leak through a thin section). Consumers should surface
/// this number in build-time logging.
#[derive(Debug, Clone)]
pub struct FloodFillReport {
    /// Grid dimensions (cells per axis).
    pub dims: [usize; 3],
    /// Number of cells classified [`Region::Inside`] after the wall
    /// expansion pass.
    pub inside_cells: usize,
    /// Number of cells classified [`Region::Outside`] after the wall
    /// expansion pass.
    pub outside_cells: usize,
    /// Number of cells classified [`Region::Wall`] during the initial
    /// classification pass (before expansion).
    pub wall_cells: usize,
    /// Number of disjoint inside components found by the BFS.
    /// 1 = healthy; >1 indicates a flood leak.
    pub inside_components: usize,
    /// Wall-band half-width (meters) used during classification.
    pub wall_threshold_m: f64,
    /// Wall-clock build time in seconds.
    pub build_secs: f64,
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Stub oracles: small `Signed` instance for trait-shape tests
    /// (composition delegation, sign sign-flip, blanket Send+Sync).
    struct ConstDistance(f64);
    impl UnsignedDistance for ConstDistance {
        fn distance(&self, _p: Point3<f64>) -> f64 {
            self.0
        }
        fn closest_point(&self, p: Point3<f64>) -> Point3<f64> {
            p
        }
    }

    struct AlwaysInside;
    impl Sign for AlwaysInside {
        fn is_inside(&self, _p: Point3<f64>) -> bool {
            true
        }
    }

    struct AlwaysOutside;
    impl Sign for AlwaysOutside {
        fn is_inside(&self, _p: Point3<f64>) -> bool {
            false
        }
    }

    #[test]
    fn signed_flips_distance_by_sign_oracle() {
        let inside = Signed {
            distance: ConstDistance(2.5),
            sign: AlwaysInside,
        };
        let outside = Signed {
            distance: ConstDistance(2.5),
            sign: AlwaysOutside,
        };
        let p = Point3::new(0.0, 0.0, 0.0);
        assert_eq!(inside.evaluate(p), -2.5);
        assert_eq!(outside.evaluate(p), 2.5);
        // `distance` is an alias for `evaluate`.
        assert_eq!(inside.distance(p), -2.5);
        // `unsigned_distance` ignores the sign oracle.
        assert_eq!(inside.unsigned_distance(p), 2.5);
        assert_eq!(outside.unsigned_distance(p), 2.5);
        assert!(inside.is_inside(p));
        assert!(!outside.is_inside(p));
        assert_eq!(inside.closest_point(p), p);
    }

    #[test]
    fn signed_is_send_and_sync_when_oracles_are() {
        fn assert_send_sync<T: Send + Sync>() {}
        assert_send_sync::<Signed<ConstDistance, AlwaysInside>>();
    }

    #[test]
    fn region_variants_are_distinct() {
        assert_ne!(Region::Inside, Region::Outside);
        assert_ne!(Region::Inside, Region::Wall);
        assert_ne!(Region::Outside, Region::Wall);
    }
}
