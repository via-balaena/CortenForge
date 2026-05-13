//! v2 curve-following mold split-surface (the "ribbon").
//!
//! For curved scans, a flat splitting plane between mold pieces would
//! intersect the mesh non-trivially and produce ugly piece geometry.
//! The **ribbon** is a curved splitting surface that follows the
//! scan's centerline polyline: at each segment of the polyline, the
//! cutting plane is locally perpendicular to both the segment's
//! tangent direction AND a user-chosen "split-normal" world-frame
//! direction. The result is a C⁰-continuous curved surface whose
//! zero set, evaluated as an SDF, gives `+`/`-` half-spaces that
//! divide the mold into the requested number of pieces.
//!
//! See `docs/CURVE_FOLLOWING_DESIGN.md` §Algorithm §Step 2 for the
//! full geometric construction; this module is just the type
//! definitions + lightweight construction (no SDF evaluation yet —
//! that lands at Step 4).
//!
//! # Types
//!
//! - [`SplitNormal`] — user-supplied world-frame direction that
//!   anchors which way the mold "opens". Default `+X` (worldframe).
//!   The Ribbon construction projects this into each segment's
//!   tangent-perpendicular plane so the cutting plane stays
//!   perpendicular to the local curve direction.
//! - [`Ribbon`] — the curved splitting surface itself. Owns the
//!   centerline polyline plus per-segment cached tangent + binormal
//!   vectors. Constructed once at the start of v2 cast generation;
//!   reused across every layer × piece SDF composition.
//!
//! Both types live in cf-cast (not cf-design) because the ribbon is
//! a cast-specific construct (it parameterizes mold-piece geometry).
//! cf-design's [`Sdf`] trait could host a `RibbonSdf` newtype later
//! if other consumers need it, but Stage 2.5 ships the ribbon as a
//! cf-cast-internal concept.
//!
//! [`Sdf`]: cf_design::Sdf

use nalgebra::{Point3, Unit, Vector3};

/// World-frame direction anchoring "which way the mold opens".
/// Combined with each centerline segment's tangent at Ribbon
/// construction time to derive the local cutting-plane binormal.
///
/// The default `+X` (world-frame) is the most common choice for
/// scans aligned to the cast frame's `+Z` demolding axis: the
/// mold splits along the X axis, with the two halves coming apart
/// in the world `+X` / `-X` directions. Users with different
/// scan-frame orientations (or aesthetic preferences for the
/// split-line location) override via the v2 cast spec's
/// `split_normal: SplitNormal` field.
///
/// **Invariant**: the stored `Vector3<f64>` is unit-length. The
/// [`SplitNormal::new`] constructor normalizes input + rejects
/// near-zero vectors via [`Result`]; the [`SplitNormal::default`]
/// world-`+X` path skips the runtime check (compile-time-known unit).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SplitNormal(Unit<Vector3<f64>>);

impl SplitNormal {
    /// Construct a `SplitNormal` from a world-frame direction vector.
    /// Normalizes `direction` to unit length; returns `None` if the
    /// vector's magnitude is below [`f64::EPSILON`] (degenerate;
    /// cannot represent a direction).
    ///
    /// # Examples
    ///
    /// ```
    /// use nalgebra::Vector3;
    /// use cf_cast::SplitNormal;
    ///
    /// let positive_x = SplitNormal::new(Vector3::new(1.0, 0.0, 0.0)).unwrap();
    /// let custom = SplitNormal::new(Vector3::new(1.0, 1.0, 0.0)).unwrap();
    /// assert!((custom.as_vector().norm() - 1.0).abs() < 1e-12);
    /// ```
    #[must_use]
    pub fn new(direction: Vector3<f64>) -> Option<Self> {
        let norm = direction.norm();
        if !norm.is_finite() || norm < f64::EPSILON {
            return None;
        }
        Some(Self(Unit::new_unchecked(direction / norm)))
    }

    /// Underlying unit-length world-frame vector.
    #[must_use]
    pub fn as_vector(self) -> Vector3<f64> {
        self.0.into_inner()
    }
}

impl Default for SplitNormal {
    /// Default world-`+X`. Most cf-scan-prep scans are aligned to
    /// the cast frame's `+Z` demolding axis (after `[Snap boundary
    /// -> floor]` in the Cap panel), so `+X` is the natural split
    /// direction.
    fn default() -> Self {
        Self(Vector3::x_axis())
    }
}

/// Per-segment cached frame along the centerline polyline.
///
/// One [`RibbonSegment`] exists for each consecutive pair of points
/// in the centerline (i.e., `points.len() - 1` segments for a
/// polyline of `points.len()` vertices). The cached vectors are the
/// foundation of the [`Ribbon`] SDF (added at Step 4): the closest-
/// segment query reads `tangent` for projection + `binormal` for
/// the cutting-plane normal.
///
/// **All vectors are unit-length** at construction time
/// (`Ribbon::new` validates + caches). Callers can rely on this
/// invariant when evaluating the SDF (no defensive renormalize per
/// query).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RibbonSegment {
    /// Segment start point (world frame, meters). Matches the
    /// centerline polyline's i-th vertex.
    pub start: Point3<f64>,
    /// Segment end point (world frame, meters). Matches the
    /// centerline polyline's (i+1)-th vertex.
    pub end: Point3<f64>,
    /// Unit-length tangent direction: `normalize(end - start)`.
    /// Locally perpendicular to the cutting plane at this segment.
    pub tangent: Vector3<f64>,
    /// Unit-length binormal direction: `normalize(tangent × N_split)`,
    /// where `N_split` is the user-supplied
    /// [`SplitNormal`]. The cutting plane at this segment has this
    /// vector as its normal; the ribbon SDF at query point `Q`
    /// returns `dot(Q - P*, binormal)` where `P*` is the projection
    /// of `Q` onto the segment.
    pub binormal: Vector3<f64>,
}

/// The curved splitting surface for v2 multi-piece molds.
///
/// Owns the centerline polyline (passed in from cf-scan-prep's
/// `.prep.toml` `[centerline]` block) plus the precomputed per-
/// segment tangent + binormal frames. Constructed once at the start
/// of v2 cast generation; reused across every layer × piece SDF
/// composition (Step 5).
///
/// # Construction errors
///
/// [`Ribbon::new`] rejects centerlines that cannot produce a valid
/// ribbon frame:
///
/// - **Fewer than 2 points** — a single point has no tangent
///   direction; the polyline is degenerate.
/// - **Zero-length segment** — two adjacent points coincide; the
///   tangent direction is undefined.
/// - **Tangent parallel to [`SplitNormal`]** — the cross-product
///   `tangent × N_split` produces a zero vector, so the binormal is
///   undefined. This is the geometric "the curve points along the
///   axis you want to split along" pathology; the v2 cast spec
///   should re-roll `N_split` (the v2 cast spec validates this at
///   build time per `docs/CURVE_FOLLOWING_DESIGN.md` §Open
///   questions §4).
///
/// All three rejection cases surface as [`RibbonError`] variants;
/// no panic path.
#[derive(Debug, Clone, PartialEq)]
pub struct Ribbon {
    /// Centerline polyline vertices, in world-frame meters.
    /// `points[0]` = base end of the scan; `points.last()` = tip
    /// end. Order matches cf-scan-prep's centerline output
    /// (`compute_centerline_polyline` walks the scan from min-projection
    /// to max-projection along the chord-direction principal axis).
    pub points: Vec<Point3<f64>>,
    /// Per-segment cached frame; `segments.len() == points.len() - 1`.
    pub segments: Vec<RibbonSegment>,
    /// User-supplied world-frame split direction used to compute
    /// the per-segment binormals.
    pub split_normal: SplitNormal,
}

/// Errors encountered while constructing a [`Ribbon`] from a
/// centerline polyline + split normal. All variants surface
/// recoverable, user-facing geometric pathologies (no internal
/// bugs).
#[derive(Debug, thiserror::Error, PartialEq)]
pub enum RibbonError {
    /// Polyline has fewer than 2 points — no segments can be
    /// formed.
    #[error("ribbon centerline must have at least 2 points; got {count}")]
    InsufficientPoints {
        /// Length of the rejected centerline polyline (`0` or `1`).
        count: usize,
    },

    /// Adjacent centerline points coincide at segment index
    /// `segment_index`; the segment tangent is undefined.
    #[error(
        "ribbon centerline has zero-length segment at index {segment_index} \
         (points {start_index} and {end_index} coincide within {epsilon} m)"
    )]
    ZeroLengthSegment {
        /// Index of the first segment whose endpoints coincided.
        segment_index: usize,
        /// Index of the start vertex of the offending segment in the
        /// input centerline polyline.
        start_index: usize,
        /// Index of the end vertex of the offending segment in the
        /// input centerline polyline (`= start_index + 1`).
        end_index: usize,
        /// Numeric tolerance below which the segment vector was
        /// classified as zero-length.
        epsilon: f64,
    },

    /// Segment tangent is parallel to the [`SplitNormal`] at index
    /// `segment_index`; the binormal `tangent × N_split` is the
    /// zero vector and the cutting plane is undefined. v2 spec
    /// should re-roll `N_split` (an axis perpendicular to the
    /// curve's average tangent direction is a safe choice).
    #[error(
        "ribbon segment {segment_index} tangent is parallel to split-normal \
         (|tangent × N_split| = {cross_norm:.3e} < {epsilon:.3e}); \
         pick a different SplitNormal direction"
    )]
    TangentParallelToSplitNormal {
        /// Index of the first segment whose tangent was parallel to
        /// [`SplitNormal`].
        segment_index: usize,
        /// Magnitude of `tangent × N_split` at the offending segment.
        cross_norm: f64,
        /// Numeric tolerance below which the cross-product magnitude
        /// was classified as zero (binormal undefined).
        epsilon: f64,
    },
}

/// Below this magnitude the segment vector / cross-product is
/// treated as numerically zero. ~ 1 µm at the meter scale of
/// cf-cast inputs; tighter than any real scan's centerline
/// segment length (10s of mm typical) or cross-product magnitude
/// for non-degenerate inputs.
const RIBBON_NUMERIC_EPSILON: f64 = 1e-6;

impl Ribbon {
    /// Build a [`Ribbon`] from a centerline polyline + split-normal
    /// direction. Validates each segment's tangent + binormal at
    /// construction time so SDF evaluation (Step 4) can assume
    /// well-formed frames.
    ///
    /// `points` is moved (not borrowed) because the Ribbon owns the
    /// polyline thereafter — the centerline isn't useful after
    /// Ribbon construction except as the source of the `segments`,
    /// so storing it inside avoids a parallel-source-of-truth
    /// problem.
    ///
    /// # Errors
    ///
    /// Returns [`RibbonError`] on any of the three pathologies
    /// listed at the type-level docstring. No panic path.
    pub fn new(points: Vec<Point3<f64>>, split_normal: SplitNormal) -> Result<Self, RibbonError> {
        if points.len() < 2 {
            return Err(RibbonError::InsufficientPoints {
                count: points.len(),
            });
        }

        let split_vec = split_normal.as_vector();
        let mut segments = Vec::with_capacity(points.len() - 1);

        for (i, window) in points.windows(2).enumerate() {
            let start = window[0];
            let end = window[1];
            let edge = end - start;
            let edge_norm = edge.norm();
            if edge_norm < RIBBON_NUMERIC_EPSILON {
                return Err(RibbonError::ZeroLengthSegment {
                    segment_index: i,
                    start_index: i,
                    end_index: i + 1,
                    epsilon: RIBBON_NUMERIC_EPSILON,
                });
            }
            let tangent = edge / edge_norm;
            let cross = tangent.cross(&split_vec);
            let cross_norm = cross.norm();
            if cross_norm < RIBBON_NUMERIC_EPSILON {
                return Err(RibbonError::TangentParallelToSplitNormal {
                    segment_index: i,
                    cross_norm,
                    epsilon: RIBBON_NUMERIC_EPSILON,
                });
            }
            let binormal = cross / cross_norm;
            segments.push(RibbonSegment {
                start,
                end,
                tangent,
                binormal,
            });
        }

        Ok(Self {
            points,
            segments,
            split_normal,
        })
    }

    /// Total arc length of the centerline (sum of segment lengths).
    /// Useful for piece-count selection heuristics that depend on
    /// the curve's overall extent (e.g., "split into 2 pieces if
    /// arc length × max-tangent-rotation > threshold").
    #[must_use]
    pub fn arc_length(&self) -> f64 {
        self.segments.iter().map(|s| (s.end - s.start).norm()).sum()
    }

    /// Maximum tangent-rotation angle (radians) between consecutive
    /// segments along the centerline. Used by v2's piece-count
    /// selector (`docs/CURVE_FOLLOWING_DESIGN.md` §Piece count
    /// selection): `< 60°` -> 2 pieces, `60-120°` -> 2 pieces with
    /// warning, `> 120°` -> refuse with error.
    ///
    /// Returns `0.0` for a Ribbon with a single segment (no
    /// adjacent-pair to compute rotation between). For a
    /// straight-line centerline (all segments collinear), returns
    /// `0.0` since each consecutive tangent matches the previous.
    #[must_use]
    pub fn max_tangent_rotation_rad(&self) -> f64 {
        let mut max_angle: f64 = 0.0;
        for window in self.segments.windows(2) {
            let cos_theta = window[0].tangent.dot(&window[1].tangent).clamp(-1.0, 1.0);
            let angle = cos_theta.acos();
            if angle > max_angle {
                max_angle = angle;
            }
        }
        max_angle
    }

    /// Find the index of the centerline segment closest to `query` +
    /// the parameter `t ∈ [0, 1]` of the projected closest point
    /// along that segment. Linear scan over the (typically 10-30
    /// point) centerline — cheap; on a 30-point polyline this is
    /// ~30 dot-products per query, well under a microsecond.
    ///
    /// For query points beyond the polyline endpoints, the closest
    /// segment is the start or end segment with `t` clamped to
    /// `0.0` or `1.0` respectively. This makes the ribbon SDF
    /// well-defined everywhere in world space rather than only on
    /// the cylinder of points whose projection lands strictly
    /// within a segment.
    ///
    /// Returns `(segment_index, t)` where:
    /// - `segment_index ∈ [0, segments.len())`
    /// - `t ∈ [0.0, 1.0]` (clamped)
    fn closest_segment(&self, query: &Point3<f64>) -> (usize, f64) {
        let mut best_idx = 0usize;
        let mut best_t = 0.0f64;
        let mut best_dist_sq = f64::INFINITY;
        for (i, seg) in self.segments.iter().enumerate() {
            let edge = seg.end - seg.start;
            let edge_len_sq = edge.norm_squared();
            // Edge non-zero by construction (Ribbon::new validates).
            let to_query = query - seg.start;
            let t = (to_query.dot(&edge) / edge_len_sq).clamp(0.0, 1.0);
            let projection = seg.start + edge * t;
            let dist_sq = (query - projection).norm_squared();
            if dist_sq < best_dist_sq {
                best_dist_sq = dist_sq;
                best_idx = i;
                best_t = t;
            }
        }
        (best_idx, best_t)
    }

    /// Evaluate the ribbon SDF at world-frame query point `query`.
    /// Returns the signed distance to the local cutting plane at
    /// the closest centerline segment.
    ///
    /// Algorithm (`docs/CURVE_FOLLOWING_DESIGN.md` §Algorithm
    /// §Step 2):
    /// 1. Find the closest centerline segment to `query` (linear
    ///    scan; ~30 ops on a typical centerline).
    /// 2. Compute the projection `P*` of `query` onto that segment.
    /// 3. Return `dot(query - P*, B*)` where `B*` is the segment's
    ///    binormal (`tangent × N_split`, unit-length per
    ///    construction).
    ///
    /// Positive return value -> query is on the `+binormal` side of
    /// the ribbon (one piece); negative -> `-binormal` side (other
    /// piece). The zero set is the ribbon surface itself.
    ///
    /// **Continuity**: C⁰ everywhere; C¹ within each segment's
    /// Voronoi region; potential creases at segment-junction
    /// boundaries (where the closest-segment query switches from
    /// one to the next). Per design doc §Risks: at curved tangent
    /// transitions the discontinuity is small (proportional to the
    /// sin of the tangent rotation between adjacent segments) and
    /// well-tolerated by marching cubes at the spec's 2mm cell
    /// size. C¹ smoothing (linear interp of tangent/binormal across
    /// neighboring segments) deferred until iter-1 surfaces a
    /// visible artifact.
    ///
    /// # Examples
    ///
    /// ```
    /// use nalgebra::{Point3, Vector3};
    /// use cf_cast::{Ribbon, SplitNormal};
    ///
    /// // Centerline along +X, split-normal +Y → binormal +Z.
    /// let points = vec![
    ///     Point3::new(0.0, 0.0, 0.0),
    ///     Point3::new(0.10, 0.0, 0.0),
    /// ];
    /// let split = SplitNormal::new(Vector3::new(0.0, 1.0, 0.0)).unwrap();
    /// let ribbon = Ribbon::new(points, split).unwrap();
    ///
    /// // Query at +z above the centerline → positive SDF.
    /// let q = Point3::new(0.05, 0.0, 0.02);
    /// assert!((ribbon.sdf(&q) - 0.02).abs() < 1e-12);
    /// ```
    #[must_use]
    pub fn sdf(&self, query: &Point3<f64>) -> f64 {
        let (seg_idx, t) = self.closest_segment(query);
        let seg = &self.segments[seg_idx];
        let edge = seg.end - seg.start;
        let projection = seg.start + edge * t;
        (query - projection).dot(&seg.binormal)
    }
}

#[cfg(test)]
mod tests {
    // Workspace lint policy denies `unwrap`/`expect`/`panic!` in
    // production code; tests need them for assertion-failure paths.
    // Same allow-list as `spec.rs` to keep the cf-cast test idiom
    // consistent.
    #![allow(
        clippy::unwrap_used,
        clippy::panic,
        clippy::expect_used,
        clippy::float_cmp
    )]

    use super::*;
    use approx::assert_relative_eq;

    fn polyline_along_x(n: usize) -> Vec<Point3<f64>> {
        (0..n)
            .map(|i| {
                #[allow(clippy::cast_precision_loss)] // small test indices.
                let x = i as f64 * 0.01;
                Point3::new(x, 0.0, 0.0)
            })
            .collect()
    }

    #[test]
    fn split_normal_default_is_world_plus_x() {
        let sn = SplitNormal::default();
        assert_eq!(sn.as_vector(), Vector3::new(1.0, 0.0, 0.0));
    }

    #[test]
    fn split_normal_normalizes_input() {
        let sn = SplitNormal::new(Vector3::new(0.0, 5.0, 0.0))
            .expect("(0, 5, 0) is non-degenerate; constructor must accept");
        assert_relative_eq!(sn.as_vector().norm(), 1.0, epsilon = 1e-12);
        assert_relative_eq!(sn.as_vector().y, 1.0, epsilon = 1e-12);
    }

    #[test]
    fn split_normal_rejects_zero_vector() {
        assert!(SplitNormal::new(Vector3::zeros()).is_none());
    }

    #[test]
    fn split_normal_rejects_non_finite_vector() {
        assert!(SplitNormal::new(Vector3::new(f64::NAN, 0.0, 0.0)).is_none());
    }

    #[test]
    fn ribbon_along_x_with_plus_y_split_normal_produces_z_binormals() {
        let points = polyline_along_x(5);
        let split =
            SplitNormal::new(Vector3::new(0.0, 1.0, 0.0)).expect("(0, 1, 0) is non-degenerate");
        let ribbon = Ribbon::new(points, split).expect("non-degenerate ribbon");
        assert_eq!(ribbon.segments.len(), 4);
        for seg in &ribbon.segments {
            assert_relative_eq!(seg.tangent.x, 1.0, epsilon = 1e-12);
            assert_relative_eq!(seg.tangent.y, 0.0, epsilon = 1e-12);
            assert_relative_eq!(seg.tangent.z, 0.0, epsilon = 1e-12);
            // x × y = z; tangent × split_normal = +z
            assert_relative_eq!(seg.binormal.x, 0.0, epsilon = 1e-12);
            assert_relative_eq!(seg.binormal.y, 0.0, epsilon = 1e-12);
            assert_relative_eq!(seg.binormal.z, 1.0, epsilon = 1e-12);
        }
    }

    #[test]
    fn ribbon_arc_length_sums_segment_lengths() {
        let points = polyline_along_x(11); // 10 segments of 0.01m
        let split =
            SplitNormal::new(Vector3::new(0.0, 1.0, 0.0)).expect("(0, 1, 0) is non-degenerate");
        let ribbon = Ribbon::new(points, split).expect("non-degenerate ribbon");
        assert_relative_eq!(ribbon.arc_length(), 0.10, epsilon = 1e-12);
    }

    #[test]
    fn ribbon_max_tangent_rotation_zero_for_straight_line() {
        let points = polyline_along_x(5);
        let split =
            SplitNormal::new(Vector3::new(0.0, 1.0, 0.0)).expect("(0, 1, 0) is non-degenerate");
        let ribbon = Ribbon::new(points, split).expect("non-degenerate ribbon");
        assert_relative_eq!(ribbon.max_tangent_rotation_rad(), 0.0, epsilon = 1e-12);
    }

    #[test]
    fn ribbon_max_tangent_rotation_90_for_right_angle_polyline() {
        let points = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.01, 0.0, 0.0),
            Point3::new(0.01, 0.01, 0.0),
        ];
        let split =
            SplitNormal::new(Vector3::new(0.0, 0.0, 1.0)).expect("(0, 0, 1) is non-degenerate");
        let ribbon = Ribbon::new(points, split).expect("non-degenerate ribbon");
        assert_relative_eq!(
            ribbon.max_tangent_rotation_rad(),
            std::f64::consts::FRAC_PI_2,
            epsilon = 1e-12,
        );
    }

    #[test]
    fn ribbon_rejects_single_point() {
        let split = SplitNormal::default();
        let err = Ribbon::new(vec![Point3::new(0.0, 0.0, 0.0)], split).unwrap_err();
        assert_eq!(err, RibbonError::InsufficientPoints { count: 1 });
    }

    #[test]
    fn ribbon_rejects_empty_polyline() {
        let split = SplitNormal::default();
        let err = Ribbon::new(vec![], split).unwrap_err();
        assert_eq!(err, RibbonError::InsufficientPoints { count: 0 });
    }

    #[test]
    fn ribbon_rejects_zero_length_segment() {
        let points = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.01, 0.0, 0.0),
        ];
        let split =
            SplitNormal::new(Vector3::new(0.0, 1.0, 0.0)).expect("(0, 1, 0) is non-degenerate");
        match Ribbon::new(points, split) {
            Err(RibbonError::ZeroLengthSegment { segment_index, .. }) => {
                assert_eq!(segment_index, 0);
            }
            other => panic!("expected ZeroLengthSegment, got {other:?}"),
        }
    }

    #[test]
    fn ribbon_rejects_tangent_parallel_to_split_normal() {
        // Polyline along +X, SplitNormal also along +X.
        let points = polyline_along_x(3);
        let split = SplitNormal::default(); // +X
        match Ribbon::new(points, split) {
            Err(RibbonError::TangentParallelToSplitNormal { segment_index, .. }) => {
                assert_eq!(segment_index, 0);
            }
            other => panic!("expected TangentParallelToSplitNormal, got {other:?}"),
        }
    }

    // ----- SDF -----------------------------------------------------

    /// Build a straight-along-+X ribbon with +Y split-normal so the
    /// binormal is +Z. SDF at any query point along that ribbon's
    /// span should equal the query's Z coordinate exactly.
    fn ribbon_x_axis() -> Ribbon {
        let points = polyline_along_x(11); // 10 segments of 0.01m, total 0.10m
        let split =
            SplitNormal::new(Vector3::new(0.0, 1.0, 0.0)).expect("(0, 1, 0) is non-degenerate");
        Ribbon::new(points, split).expect("non-degenerate ribbon")
    }

    #[test]
    fn sdf_above_centerline_is_positive_z() {
        let ribbon = ribbon_x_axis();
        // Query at +Z above the midpoint of the centerline.
        let q = Point3::new(0.05, 0.0, 0.02);
        assert_relative_eq!(ribbon.sdf(&q), 0.02, epsilon = 1e-12);
    }

    #[test]
    fn sdf_below_centerline_is_negative_z() {
        let ribbon = ribbon_x_axis();
        let q = Point3::new(0.05, 0.0, -0.03);
        assert_relative_eq!(ribbon.sdf(&q), -0.03, epsilon = 1e-12);
    }

    #[test]
    fn sdf_on_centerline_is_zero() {
        let ribbon = ribbon_x_axis();
        let q = Point3::new(0.05, 0.0, 0.0);
        assert_relative_eq!(ribbon.sdf(&q), 0.0, epsilon = 1e-12);
    }

    #[test]
    fn sdf_ignores_offset_along_split_normal() {
        // Split-normal is +Y, so query offsets in +Y don't change
        // the cutting-plane distance (which is along the binormal,
        // i.e. +Z).
        let ribbon = ribbon_x_axis();
        let q1 = Point3::new(0.05, 0.0, 0.02);
        let q2 = Point3::new(0.05, 0.1, 0.02); // same Z, different Y.
        assert_relative_eq!(ribbon.sdf(&q1), ribbon.sdf(&q2), epsilon = 1e-12);
    }

    /// For a query beyond the polyline's end, the closest segment
    /// is the last segment with `t = 1.0` (clamped). The signed
    /// distance to the cutting plane should still equal the
    /// query's `z` component (since the last segment's binormal is
    /// still +Z).
    #[test]
    fn sdf_beyond_polyline_endpoint_clamps_to_last_segment() {
        let ribbon = ribbon_x_axis();
        let q = Point3::new(0.5, 0.0, 0.03); // x = 0.5m, well past 0.10m end
        assert_relative_eq!(ribbon.sdf(&q), 0.03, epsilon = 1e-12);
    }

    /// Right-angle polyline: first segment along +X (binormal +Z if
    /// split = +Y), second segment along +Y (binormal -Z if split
    /// = +Y, since X→Y rotates the tangent 90° counter-clockwise
    /// around +Z, but tangent × Y = -Z). Verify SDF sign behaves
    /// per the closest segment.
    #[test]
    fn sdf_right_angle_polyline_segment_dispatches_correctly() {
        // Right-angle XY polyline with +Z split-normal so both
        // segments produce well-defined (non-degenerate) binormals
        // in the XY plane.
        let points = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.10, 0.0, 0.0),  // segment 0 along +X
            Point3::new(0.10, 0.10, 0.0), // segment 1 along +Y
        ];
        let split =
            SplitNormal::new(Vector3::new(0.0, 0.0, 1.0)).expect("(0, 0, 1) is non-degenerate");
        let ribbon = Ribbon::new(points, split).expect("non-degenerate ribbon");
        // Segment 0: tangent = +X, binormal = X × Z = -Y.
        // Query at midpoint of segment 0 offset +Y: SDF = -y_offset.
        let q0 = Point3::new(0.05, 0.01, 0.0);
        assert_relative_eq!(ribbon.sdf(&q0), -0.01, epsilon = 1e-12);
        // Segment 1: tangent = +Y, binormal = Y × Z = +X.
        // Query near midpoint of segment 1 offset +X: SDF = +x_offset
        // from segment-1 line (which is at x=0.10).
        let q1 = Point3::new(0.11, 0.05, 0.0);
        assert_relative_eq!(ribbon.sdf(&q1), 0.01, epsilon = 1e-12);
    }

    /// SDF flips sign across the cutting plane: querying ±δ symmetric
    /// points should give equal-magnitude opposite-sign SDF values.
    #[test]
    fn sdf_is_antisymmetric_across_cutting_plane() {
        let ribbon = ribbon_x_axis();
        let q_plus = Point3::new(0.05, 0.0, 0.02);
        let q_minus = Point3::new(0.05, 0.0, -0.02);
        assert_relative_eq!(ribbon.sdf(&q_plus), -ribbon.sdf(&q_minus), epsilon = 1e-12);
    }

    /// Closest-segment dispatch picks the correct segment when the
    /// query is nearer to a non-first segment. Centerline goes
    /// 0.0 → 0.10 along +X; query at x=0.085 should pick segment 8
    /// (which spans x=0.08 to x=0.09; 0.085 is its midpoint).
    #[test]
    fn closest_segment_picks_nearest_in_long_polyline() {
        let ribbon = ribbon_x_axis(); // 10 segments along x = 0..0.10
        let q = Point3::new(0.085, 0.0, 0.0);
        let (idx, t) = ribbon.closest_segment(&q);
        assert_eq!(idx, 8);
        // segment 8 spans 0.08..0.09; query at 0.085 → t = 0.5.
        assert_relative_eq!(t, 0.5, epsilon = 1e-12);
    }
}
