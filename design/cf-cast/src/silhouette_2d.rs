//! 2D silhouette polyline + point-to-silhouette signed distance.
//!
//! §F-1 + §F-2 of
//! `docs/CF_CAST_FLANGE_CONTINUITY_BOLT_PATTERN_RECON.md`. Replaces
//! `crate::flange::FlangeSdf`'s 3D-projected `body.evaluate` with a
//! 2D point-to-silhouette signed distance so the flange ring closes
//! around body concavities (heel pocket etc. on production
//! sock-over-capsule scans).
//!
//! # Why a separate 2D distance metric
//!
//! Per [[project-cf-cast-flange-perimeter-continuity-bookmark]] +
//! §F-S0 empirical probe (`crate::flange::tests::s0_flange_continuity_probe`):
//! the pre-fix bug was that `body.evaluate(P_projected_to_seam)`
//! returns the 3D distance from `P` to the nearest body surface,
//! which can collapse to the distance to a 3D wrap-around feature
//! (overhang, lid, shelf) at a small Y offset rather than the in-plane
//! distance to the silhouette curve. When that 3D distance drops
//! below `flange_inner_offset_m` (2 mm), `FlangeSdf`'s inner check
//! falsely declares the query point to be in the gasket-channel
//! margin and suppresses the flange material — producing the
//! workshop-visible "flange isn't all the way around" symptom.
//!
//! The 2D silhouette distance is the geometrically correct metric: it
//! measures distance to the body's seam-plane cross-section curve,
//! independent of any 3D structure off the seam plane.
//!
//! # Algorithm
//!
//! 1. Sample `body.evaluate` on a uniform 2D grid in (X, Z) at
//!    Y = `seam_plane_y`, step [`SILHOUETTE_GRID_STEP_M`] (0.5 mm).
//! 2. Marching squares (16-case table) emits zero-isoline line
//!    segments per cell. Saddle ambiguities (cases 5 + 10) resolved
//!    by sampling the cell center.
//! 3. [`Silhouette2d::signed_distance_to`] computes
//!    `±min_segment_distance` with sign from even/odd +X ray-cast.
//!
//! # Scope (§F-S1)
//!
//! Segments are stored as 2-vertex polylines without chaining into
//! ordered closed polylines. Point-to-segment distance + ray-cast
//! parity both work correctly on raw segment soup; ordered polylines
//! are deferred to §B (bolt-pattern arc-length placement).

// Grid + marching-squares code casts between usize grid indices and
// f64 coordinates throughout; indices are bounded by the grid
// dimensions (small thousands at most) so precision/truncation/sign
// loss are safe and intended. Match-same-arms is allowed because
// keeping all 16 standard MC cases enumerated mirrors textbook
// references and stays robust if a future change makes one branch
// diverge (e.g., orientation-aware polyline assembly for §B).
#![allow(
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::match_same_arms,
    clippy::similar_names
)]

use cf_design::{Aabb, Solid};
use nalgebra::{Point3, Vector3};

/// Marching-squares grid step (0.5 mm). Picked at recon §F-2: sub-mm
/// polyline accuracy + ~0.16 s precompute cost per layer at 200 ×
/// 200 mm grid.
pub const SILHOUETTE_GRID_STEP_M: f64 = 0.0005;

/// 2D point in the seam plane (X, Z coordinates; Y implicit).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Point2 {
    /// X coordinate.
    pub x: f64,
    /// Z coordinate.
    pub z: f64,
}

impl Point2 {
    /// Construct a 2D point from its X and Z coordinates.
    #[must_use]
    pub const fn new(x: f64, z: f64) -> Self {
        Self { x, z }
    }
}

/// Orthonormal frame of a seam plane.
///
/// An in-plane `(u, v)` coordinate maps to world as `anchor + u·u_axis +
/// v·v_axis`. `u_axis` + `v_axis` are unit, mutually orthogonal, and ⟂ the seam
/// normal (the plane they span IS the seam plane). Returned by
/// `Ribbon::seam_plane_basis`; consumed by [`Silhouette2d::from_body_in_plane`]
/// + the flange/bolt/dowel builders.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SeamPlaneBasis {
    /// In-plane origin (a point ON the seam plane).
    pub anchor: Point3<f64>,
    /// In-plane `+u` world axis (the `Point2::x` direction).
    pub u_axis: Vector3<f64>,
    /// In-plane `+v` world axis (the `Point2::z` direction).
    pub v_axis: Vector3<f64>,
}

impl SeamPlaneBasis {
    /// The historical Y-normal seam frame: `anchor = (0, seam_y, 0)`,
    /// `u_axis = +X`, `v_axis = +Z`. Reproduces the pre-generalization
    /// constant-Y X-Z silhouette exactly.
    // Not const: nalgebra's `Vector3::x()` / `Vector3::z()` are not const fns.
    #[allow(clippy::missing_const_for_fn)]
    #[must_use]
    pub fn y_normal(seam_y: f64) -> Self {
        Self {
            anchor: Point3::new(0.0, seam_y, 0.0),
            u_axis: Vector3::x(),
            v_axis: Vector3::z(),
        }
    }

    /// Derive a basis from a seam plane given as `(anchor, normal)`: pick a
    /// stable in-plane `u_axis` by projecting world `+Z` onto the plane (so `u`
    /// runs roughly "up" the part — nice for print orientation), falling back to
    /// `+X` when the normal is ~vertical (`Z ∥ N`); `v_axis = N × u_axis`. The
    /// resulting `u_axis × v_axis == N`, so the basis's implied normal matches.
    #[must_use]
    pub fn from_anchor_normal(anchor: Point3<f64>, normal: Vector3<f64>) -> Self {
        let n = normal.normalize();
        let seed = if n.z.abs() < 0.9 {
            Vector3::z()
        } else {
            Vector3::x()
        };
        let u_axis = (seed - n * n.dot(&seed)).normalize();
        let v_axis = n.cross(&u_axis);
        Self {
            anchor,
            u_axis,
            v_axis,
        }
    }

    /// The seam-plane normal implied by this basis (`u_axis × v_axis`, unit).
    #[must_use]
    pub fn normal(&self) -> Vector3<f64> {
        self.u_axis.cross(&self.v_axis)
    }

    /// In-plane `(u_min, u_max, v_min, v_max)` covering `aabb` — projects all 8
    /// AABB corners into the basis and takes the extremes. Used to size the
    /// silhouette grid so it spans the body's full in-plane extent at any seam
    /// orientation (flange / bolt / dowel builders).
    #[must_use]
    pub fn inplane_bounds(&self, aabb: Aabb) -> (f64, f64, f64, f64) {
        let (mut u_min, mut u_max) = (f64::MAX, f64::MIN);
        let (mut v_min, mut v_max) = (f64::MAX, f64::MIN);
        for &cx in &[aabb.min.x, aabb.max.x] {
            for &cy in &[aabb.min.y, aabb.max.y] {
                for &cz in &[aabb.min.z, aabb.max.z] {
                    let rel = Point3::new(cx, cy, cz) - self.anchor;
                    let u = rel.dot(&self.u_axis);
                    let v = rel.dot(&self.v_axis);
                    u_min = u_min.min(u);
                    u_max = u_max.max(u);
                    v_min = v_min.min(v);
                    v_max = v_max.max(v);
                }
            }
        }
        (u_min, u_max, v_min, v_max)
    }

    /// Map an in-plane `(u, v)` point to its world position.
    #[must_use]
    pub fn to_world(&self, p: Point2) -> Point3<f64> {
        self.anchor + self.u_axis * p.x + self.v_axis * p.z
    }

    /// Map an in-plane `(u, v)` DIRECTION to world (no translation). Unit in →
    /// unit out (the axes are unit-orthonormal).
    #[must_use]
    pub fn dir_to_world(&self, p: Point2) -> Vector3<f64> {
        self.u_axis * p.x + self.v_axis * p.z
    }
}

/// Seam-plane silhouette of a body, stored as unordered line segments.
///
/// §F-S1 scope: each marching-squares cell contributes 0–2 segments;
/// ordered closed-polyline assembly is deferred to §B (bolt-pattern
/// arc-length placement needs ordering, but point-to-segment distance
/// + even/odd ray-cast both work correctly on raw segment soup).
#[derive(Debug, Clone)]
pub struct Silhouette2d {
    segments: Vec<[Point2; 2]>,
    /// Seam-plane frame the segments live in; maps in-plane `(u, v)` to world.
    basis: SeamPlaneBasis,
}

impl Silhouette2d {
    /// Build by sampling `body` SDF on a 2D grid at Y = `seam_plane_y`
    /// covering (X, Z) ∈ [`x_min`, `x_max`] × [`z_min`, `z_max`] with
    /// step [`SILHOUETTE_GRID_STEP_M`].
    ///
    /// The bounds should cover the body's seam-plane extent PLUS the
    /// flange's outward reach so the silhouette is captured everywhere
    /// the flange might lie. Callers pass the same expanded bounds
    /// used for [`Solid::from_sdf`]'s MC region.
    ///
    /// This is the Y-normal-seam special case of [`Self::from_body_in_plane`]
    /// (`anchor = (0, seam_plane_y, 0)`, `u_axis = +X`, `v_axis = +Z`) — kept
    /// as the bit-identical default path so every existing (binormal-seam) cast
    /// is unchanged.
    ///
    /// # Panics
    ///
    /// Panics if `x_max <= x_min` or `z_max <= z_min`.
    #[must_use]
    pub fn from_body_at_y(
        body: &Solid,
        seam_plane_y: f64,
        x_min: f64,
        x_max: f64,
        z_min: f64,
        z_max: f64,
    ) -> Self {
        Self::from_body_in_plane(
            body,
            SeamPlaneBasis::y_normal(seam_plane_y),
            x_min,
            x_max,
            z_min,
            z_max,
        )
    }

    /// Build by sampling `body` SDF on a 2D grid in the seam plane spanned by
    /// `u_axis` × `v_axis` through `anchor`: grid point `(u, v)` samples
    /// `body.evaluate(anchor + u·u_axis + v·v_axis)`. Stored [`Point2`]s are the
    /// in-plane `(u, v)` coordinates (`Point2::x = u`, `Point2::z = v`); map them
    /// back to world with [`Self::to_world`] / [`Self::dir_to_world`].
    ///
    /// `u_axis` + `v_axis` should be orthonormal and perpendicular to the seam
    /// normal (the plane they span IS the seam plane). The `(u, v)` bounds cover
    /// the body's in-plane extent PLUS the flange's outward reach.
    ///
    /// Generalizes [`Self::from_body_at_y`] to an arbitrary (e.g. apex-anchored
    /// fitted) seam plane — item A of `CF_CAST_ORGANIC_PARTS_RECON.md` + the
    /// arbitrary-seam-silhouette arc.
    ///
    /// # Panics
    ///
    /// Panics if `u_max <= u_min` or `v_max <= v_min`.
    #[must_use]
    pub fn from_body_in_plane(
        body: &Solid,
        basis: SeamPlaneBasis,
        u_min: f64,
        u_max: f64,
        v_min: f64,
        v_max: f64,
    ) -> Self {
        assert!(
            u_max > u_min && v_max > v_min,
            "silhouette bounds must be positive: u [{u_min}, {u_max}] v [{v_min}, {v_max}]"
        );
        let step = SILHOUETTE_GRID_STEP_M;
        let nx = (((u_max - u_min) / step).ceil() as usize).max(1);
        let nz = (((v_max - v_min) / step).ceil() as usize).max(1);
        // In-plane (u, v) → world sample point.
        let sample =
            |u: f64, v: f64| body.evaluate(&(basis.anchor + basis.u_axis * u + basis.v_axis * v));

        // Corner SDF grid: (nx + 1) × (nz + 1).
        let stride = nx + 1;
        let mut sdf = vec![0.0_f64; stride * (nz + 1)];
        for j in 0..=nz {
            let z = (j as f64).mul_add(step, v_min);
            for i in 0..=nx {
                let x = (i as f64).mul_add(step, u_min);
                sdf[j * stride + i] = sample(x, z);
            }
        }

        // Marching squares per cell (i, j) with i ∈ [0, nx), j ∈ [0, nz).
        let mut segments: Vec<[Point2; 2]> = Vec::new();
        for j in 0..nz {
            for i in 0..nx {
                let s00 = sdf[j * stride + i]; // corner 0: bottom-left  (x0, z0)
                let s10 = sdf[j * stride + i + 1]; // corner 1: bottom-right (x1, z0)
                let s11 = sdf[(j + 1) * stride + i + 1]; // corner 2: top-right    (x1, z1)
                let s01 = sdf[(j + 1) * stride + i]; // corner 3: top-left     (x0, z1)
                let x0 = (i as f64).mul_add(step, u_min);
                let x1 = x0 + step;
                let z0 = (j as f64).mul_add(step, v_min);
                let z1 = z0 + step;
                let p00 = Point2::new(x0, z0);
                let p10 = Point2::new(x1, z0);
                let p11 = Point2::new(x1, z1);
                let p01 = Point2::new(x0, z1);

                // Inside = SDF < 0. Bit i set if corner i is inside.
                let mut case = 0u8;
                if s00 < 0.0 {
                    case |= 1;
                }
                if s10 < 0.0 {
                    case |= 2;
                }
                if s11 < 0.0 {
                    case |= 4;
                }
                if s01 < 0.0 {
                    case |= 8;
                }

                // Edge crossing accessors (lazy — only compute when used).
                let e_bottom = || interp(s00, s10, p00, p10);
                let e_right = || interp(s10, s11, p10, p11);
                let e_top = || interp(s01, s11, p01, p11);
                let e_left = || interp(s00, s01, p00, p01);

                match case {
                    0 | 15 => {}
                    1 => segments.push([e_left(), e_bottom()]),
                    2 => segments.push([e_bottom(), e_right()]),
                    3 => segments.push([e_left(), e_right()]),
                    4 => segments.push([e_right(), e_top()]),
                    5 => {
                        // Saddle (corners 0 + 2 inside): resolve via
                        // cell-center SDF — if center is inside, the
                        // two inside corners are connected through the
                        // center; emit segments that DON'T separate
                        // them. Else separate.
                        let center =
                            sample(0.5_f64.mul_add(x1 - x0, x0), 0.5_f64.mul_add(z1 - z0, z0));
                        if center < 0.0 {
                            segments.push([e_left(), e_top()]);
                            segments.push([e_bottom(), e_right()]);
                        } else {
                            segments.push([e_left(), e_bottom()]);
                            segments.push([e_top(), e_right()]);
                        }
                    }
                    6 => segments.push([e_bottom(), e_top()]),
                    7 => segments.push([e_left(), e_top()]),
                    8 => segments.push([e_top(), e_left()]),
                    9 => segments.push([e_top(), e_bottom()]),
                    10 => {
                        // Saddle (corners 1 + 3 inside): mirror of case 5.
                        // - Center inside: 1+center+3 form one connected
                        //   inside region; cut off outside corners 0 + 2.
                        // - Center outside: 1 and 3 are isolated inside
                        //   islands; cut off each inside corner separately.
                        let center =
                            sample(0.5_f64.mul_add(x1 - x0, x0), 0.5_f64.mul_add(z1 - z0, z0));
                        if center < 0.0 {
                            // Cut off outside corner 0 (left + bottom edges).
                            segments.push([e_left(), e_bottom()]);
                            // Cut off outside corner 2 (top + right edges).
                            segments.push([e_top(), e_right()]);
                        } else {
                            // Cut off inside corner 1 (bottom + right edges).
                            segments.push([e_bottom(), e_right()]);
                            // Cut off inside corner 3 (top + left edges).
                            segments.push([e_top(), e_left()]);
                        }
                    }
                    11 => segments.push([e_top(), e_right()]),
                    12 => segments.push([e_left(), e_right()]),
                    13 => segments.push([e_bottom(), e_right()]),
                    14 => segments.push([e_left(), e_bottom()]),
                    // `case` is the 4-bit marching-squares cell mask built
                    // from `inside` bits 0..=3 above. The 16 case arms
                    // (0..=15) enumerate every reachable mask; cases 0/15
                    // (all inside / all outside) emit no segments and are
                    // handled by the outer `if/else` guard. Reachability is
                    // mechanical from the bit construction; no runtime
                    // input can produce a `case` outside 0..=15.
                    _ => unreachable!("case {case} out of 0..=15"),
                }
            }
        }

        Self { segments, basis }
    }

    /// The seam-plane frame these segments live in.
    #[must_use]
    pub const fn basis(&self) -> SeamPlaneBasis {
        self.basis
    }

    /// Map an in-plane `(u, v)` point (a [`Point2`] from this silhouette) back
    /// to its world position (delegates to [`SeamPlaneBasis::to_world`]).
    #[must_use]
    pub fn to_world(&self, p: Point2) -> Point3<f64> {
        self.basis.to_world(p)
    }

    /// Map an in-plane `(u, v)` DIRECTION (e.g. an outward normal from
    /// [`Self::outward_normal_at_arc_fraction`]) to its world direction
    /// (delegates to [`SeamPlaneBasis::dir_to_world`]).
    #[must_use]
    pub fn dir_to_world(&self, p: Point2) -> Vector3<f64> {
        self.basis.dir_to_world(p)
    }

    /// Signed distance from `(x, z)` to the silhouette curve.
    ///
    /// Negative = inside body (any silhouette polygon contains the
    /// query); positive = outside; magnitude is the Euclidean
    /// distance to the nearest segment.
    #[must_use]
    pub fn signed_distance_to(&self, x: f64, z: f64) -> f64 {
        let q = Point2::new(x, z);
        let unsigned = self
            .segments
            .iter()
            .map(|seg| point_to_segment_distance(q, seg[0], seg[1]))
            .fold(f64::INFINITY, f64::min);
        let crossings = self
            .segments
            .iter()
            .filter(|seg| ray_cast_crosses(q, seg[0], seg[1]))
            .count();
        if crossings % 2 == 1 {
            -unsigned
        } else {
            unsigned
        }
    }

    /// Number of segments in the silhouette (diagnostic).
    #[must_use]
    pub const fn segment_count(&self) -> usize {
        self.segments.len()
    }

    /// Assemble the segment soup into ordered closed polylines.
    ///
    /// §M-S2 (2026-05-27): lands the §F-S1-deferred ordered polyline
    /// assembly needed for arc-length parameterization (dowel-hole +
    /// §B bolt-pattern placement around the silhouette perimeter).
    ///
    /// Algorithm: hash each segment endpoint to a fixed-precision
    /// integer coordinate (1 µm grid), build an adjacency map, walk
    /// from any unvisited segment following shared endpoints until a
    /// loop closes. Each closed loop becomes one polyline (vertices
    /// in order, last vertex NOT duplicated; the closing edge from
    /// `last → first` is implicit).
    ///
    /// For grid-aligned marching-squares output, each cell-edge
    /// crossing point is exact to FP precision and the 1 µm hash
    /// reliably groups coincident endpoints from adjacent cells.
    ///
    /// Returns disjoint polylines if the silhouette has multiple
    /// connected components (rare; usually one main perimeter +
    /// possibly tiny MC artifact loops).
    #[must_use]
    pub fn polylines(&self) -> Vec<Vec<Point2>> {
        use std::collections::HashMap;
        // 1 µm precision hash for endpoint coords. SDF is in meters;
        // 1e6 multiplier gives µm-scale integer keys.
        let hash = |p: Point2| -> (i64, i64) {
            ((p.x * 1.0e6).round() as i64, (p.z * 1.0e6).round() as i64)
        };
        // adjacency: endpoint hash → list of (segment_idx, end_idx in {0,1})
        let mut adj: HashMap<(i64, i64), Vec<(usize, usize)>> = HashMap::new();
        for (i, seg) in self.segments.iter().enumerate() {
            adj.entry(hash(seg[0])).or_default().push((i, 0));
            adj.entry(hash(seg[1])).or_default().push((i, 1));
        }
        let mut visited = vec![false; self.segments.len()];
        let mut polylines: Vec<Vec<Point2>> = Vec::new();
        for start_seg in 0..self.segments.len() {
            if visited[start_seg] {
                continue;
            }
            let mut polyline: Vec<Point2> = Vec::new();
            let mut current_seg = start_seg;
            let mut current_end = 0usize; // walk OUT of this endpoint
            loop {
                if visited[current_seg] {
                    break;
                }
                visited[current_seg] = true;
                let seg = &self.segments[current_seg];
                // Append the "in" endpoint (opposite of current_end);
                // the "out" endpoint becomes the next iteration's
                // shared point with the neighbor segment.
                let in_idx = 1 - current_end;
                let out_idx = current_end;
                polyline.push(seg[in_idx]);
                let out_point = seg[out_idx];
                // Find neighbor sharing the out_point.
                let neighbors = adj.get(&hash(out_point)).map_or(&[][..], Vec::as_slice);
                let next = neighbors
                    .iter()
                    .find(|(s_idx, _)| *s_idx != current_seg && !visited[*s_idx]);
                if let Some(&(next_seg, next_endpoint_in)) = next {
                    current_seg = next_seg;
                    current_end = 1 - next_endpoint_in;
                } else {
                    break;
                }
            }
            if polyline.len() >= 2 {
                polylines.push(polyline);
            }
        }
        polylines
    }

    /// Pick the largest polyline by total arc length (main body
    /// perimeter, ignoring small MC-artifact loops) + compute its
    /// cumulative arc length at each vertex.
    ///
    /// Returns `(polyline, cumulative_arc_lengths, total_arc_length)`:
    /// the polyline has N vertices, the cumulative array has N+1
    /// entries where `cum[i]` = arc length from vertex 0 walking to
    /// vertex `i` (so `cum[0] = 0` and `cum[N]` = total including the
    /// closing edge back to vertex 0). Returns `None` if the
    /// silhouette has no polylines (empty segment set).
    #[must_use]
    pub fn longest_polyline_with_arc_length(&self) -> Option<(Vec<Point2>, Vec<f64>, f64)> {
        let polylines = self.polylines();
        let mut best: Option<(Vec<Point2>, Vec<f64>, f64)> = None;
        for poly in polylines {
            if poly.is_empty() {
                continue;
            }
            let mut cum = Vec::with_capacity(poly.len() + 1);
            cum.push(0.0_f64);
            let mut total = 0.0_f64;
            for i in 0..poly.len() {
                let a = poly[i];
                let b = poly[(i + 1) % poly.len()];
                let dx = b.x - a.x;
                let dz = b.z - a.z;
                total += dx.mul_add(dx, dz * dz).sqrt();
                cum.push(total);
            }
            // `cum` has poly.len() + 1 entries: cum[0]=0, cum[N]=total
            // (closes back to vertex 0).
            let beats = best
                .as_ref()
                .is_none_or(|(_, _, prev_total)| total > *prev_total);
            if beats {
                best = Some((poly, cum, total));
            }
        }
        best
    }

    /// Return the point on the longest polyline at arc-length fraction
    /// `t` ∈ [0, 1]. `t=0` and `t=1` both return the polyline's first
    /// vertex (closed loop). Returns `None` if the silhouette has no
    /// polylines.
    ///
    /// Used by [`crate::dowel_hole`] + future bolt-pattern placement
    /// for arc-length-equal spacing of post-MC mesh-CSG primitives
    /// around the body silhouette.
    #[must_use]
    pub fn point_at_arc_fraction(&self, t: f64) -> Option<Point2> {
        let (poly, cum, total) = self.longest_polyline_with_arc_length()?;
        if total <= 0.0 || poly.is_empty() {
            return Some(poly.first().copied().unwrap_or(Point2::new(0.0, 0.0)));
        }
        let target = t.clamp(0.0, 1.0) * total;
        // Binary search for the segment containing `target`.
        let mut lo = 0_usize;
        let mut hi = cum.len() - 1;
        while lo + 1 < hi {
            let mid = lo.midpoint(hi);
            if cum[mid] <= target {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        // Segment from poly[lo] to poly[(lo+1) % len], arc length
        // from cum[lo] to cum[lo+1].
        let seg_start = poly[lo];
        let seg_end = poly[(lo + 1) % poly.len()];
        let seg_len = cum[lo + 1] - cum[lo];
        let alpha = if seg_len > 0.0 {
            (target - cum[lo]) / seg_len
        } else {
            0.0
        };
        Some(Point2::new(
            alpha.mul_add(seg_end.x - seg_start.x, seg_start.x),
            alpha.mul_add(seg_end.z - seg_start.z, seg_start.z),
        ))
    }

    /// Outward-pointing 2D unit normal to the silhouette at the
    /// given arc-length fraction. Used to offset dowel positions
    /// outboard from the silhouette by `silhouette_outboard_offset_m`
    /// in [`crate::dowel_hole`].
    ///
    /// The local outward direction at arc fraction `t` is the
    /// perpendicular to the local tangent direction (the segment
    /// containing `t`), oriented so it points OUT of the closed
    /// curve (positive `signed_distance` side).
    #[must_use]
    pub fn outward_normal_at_arc_fraction(&self, t: f64) -> Option<Point2> {
        let (poly, cum, total) = self.longest_polyline_with_arc_length()?;
        if total <= 0.0 || poly.len() < 2 {
            return None;
        }
        let target = t.clamp(0.0, 1.0) * total;
        let mut lo = 0_usize;
        let mut hi = cum.len() - 1;
        while lo + 1 < hi {
            let mid = lo.midpoint(hi);
            if cum[mid] <= target {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        let seg_start = poly[lo];
        let seg_end = poly[(lo + 1) % poly.len()];
        let dx = seg_end.x - seg_start.x;
        let dz = seg_end.z - seg_start.z;
        let len = dx.mul_add(dx, dz * dz).sqrt();
        if len <= 0.0 {
            return None;
        }
        // Tangent (unit): (dx, dz) / len.
        // Two perpendiculars: (dz, -dx)/len  and  (-dz, dx)/len.
        // Pick the one with positive silhouette_dist sign (outward).
        let mid = Point2::new(
            0.5_f64.mul_add(seg_start.x + seg_end.x, 0.0),
            0.5_f64.mul_add(seg_start.z + seg_end.z, 0.0),
        );
        let probe_dist = 1.0e-6_f64;
        let candidate_a = Point2::new(dz / len, -dx / len);
        let probe_a = Point2::new(
            probe_dist.mul_add(candidate_a.x, mid.x),
            probe_dist.mul_add(candidate_a.z, mid.z),
        );
        // Sign at probe_a: positive = outside silhouette = outward.
        let sign_a = self.signed_distance_to(probe_a.x, probe_a.z);
        if sign_a > 0.0 {
            Some(candidate_a)
        } else {
            Some(Point2::new(-candidate_a.x, -candidate_a.z))
        }
    }
}

/// Linear interpolation of the zero-crossing on an edge with corner
/// SDF values `a` (at `p_a`) and `b` (at `p_b`). Requires opposite
/// signs (caller checks via case index).
fn interp(a: f64, b: f64, p_a: Point2, p_b: Point2) -> Point2 {
    let t = a / (a - b);
    Point2::new(
        t.mul_add(p_b.x - p_a.x, p_a.x),
        t.mul_add(p_b.z - p_a.z, p_a.z),
    )
}

/// Euclidean distance from `q` to the line segment `a`→`b`.
fn point_to_segment_distance(q: Point2, a: Point2, b: Point2) -> f64 {
    let ab_x = b.x - a.x;
    let ab_z = b.z - a.z;
    let aq_x = q.x - a.x;
    let aq_z = q.z - a.z;
    let len2 = ab_x.mul_add(ab_x, ab_z * ab_z);
    let t = if len2 > 0.0 {
        (aq_x.mul_add(ab_x, aq_z * ab_z) / len2).clamp(0.0, 1.0)
    } else {
        0.0
    };
    let proj_x = t.mul_add(ab_x, a.x);
    let proj_z = t.mul_add(ab_z, a.z);
    let dx = q.x - proj_x;
    let dz = q.z - proj_z;
    dx.mul_add(dx, dz * dz).sqrt()
}

/// Standard even/odd ray-cast: ray from `q` in +X direction crosses
/// segment `a`→`b` iff the segment straddles the horizontal line
/// `z = q.z` and the X-intersect is strictly greater than `q.x`.
fn ray_cast_crosses(q: Point2, a: Point2, b: Point2) -> bool {
    // Strict `>` on BOTH ends gives an effectively half-open convention:
    // an endpoint with `z == q.z` is classified "below" the ray, so two
    // chained segments sharing such a vertex contribute zero crossings
    // together (both straddle predicates evaluate equal and short-circuit).
    // Probability ≈ 0 for continuous coordinates from MC interpolation.
    let above_a = a.z > q.z;
    let above_b = b.z > q.z;
    if above_a == above_b {
        return false;
    }
    let t = (q.z - a.z) / (b.z - a.z);
    let x_intersect = t.mul_add(b.x - a.x, a.x);
    x_intersect > q.x
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
    use nalgebra::{Point3, UnitQuaternion, Vector3};

    /// Cylinder along X, R=10 mm, length 60 mm. At Y=0 the silhouette
    /// is the rectangle X ∈ [-30, 30], Z ∈ [-10, 10].
    fn cylinder_along_x() -> Solid {
        Solid::cylinder(0.010, 0.030).rotate(UnitQuaternion::from_axis_angle(
            &Vector3::y_axis(),
            std::f64::consts::FRAC_PI_2,
        ))
    }

    #[test]
    fn cylinder_silhouette_signed_distance_matches_2d_analytical() {
        let body = cylinder_along_x();
        let sil = Silhouette2d::from_body_at_y(&body, 0.0, -0.050, 0.050, -0.025, 0.025);
        assert!(
            sil.segment_count() > 0,
            "marching squares must emit at least one segment for a non-trivial silhouette"
        );

        // Outside body, just past +Z edge (Z=10 mm); expect +5 mm.
        let d_out = sil.signed_distance_to(0.0, 0.015);
        assert!(
            (d_out - 0.005).abs() < SILHOUETTE_GRID_STEP_M,
            "expected ≈+5 mm outside +Z edge, got {d_out}"
        );

        // Inside body, at center; expect -10 mm.
        let d_in = sil.signed_distance_to(0.0, 0.0);
        assert!(
            (d_in - (-0.010)).abs() < SILHOUETTE_GRID_STEP_M,
            "expected ≈-10 mm at body center, got {d_in}"
        );

        // Outside body, well past +Z edge; expect +15 mm.
        let d_far = sil.signed_distance_to(0.0, 0.025);
        assert!(
            (d_far - 0.015).abs() < SILHOUETTE_GRID_STEP_M,
            "expected ≈+15 mm at Z=25 mm, got {d_far}"
        );

        // Outside body, well past +X cap (X=30 mm); expect +5 mm.
        let d_x = sil.signed_distance_to(0.035, 0.0);
        assert!(
            (d_x - 0.005).abs() < SILHOUETTE_GRID_STEP_M,
            "expected ≈+5 mm past +X cap, got {d_x}"
        );
    }

    /// Byte-identity guard: `from_body_at_y` is the `(X, Z)` basis special case
    /// of `from_body_in_plane`, so the two must emit the exact same segments —
    /// the load-bearing invariant that keeps every existing (Y-normal) cast
    /// unchanged after the arbitrary-seam generalization.
    #[test]
    fn from_body_in_plane_with_xz_basis_is_identical_to_from_body_at_y() {
        let body = cylinder_along_x();
        let seam_y = 0.003;
        let at_y = Silhouette2d::from_body_at_y(&body, seam_y, -0.050, 0.050, -0.025, 0.025);
        let in_plane = Silhouette2d::from_body_in_plane(
            &body,
            SeamPlaneBasis {
                anchor: Point3::new(0.0, seam_y, 0.0),
                u_axis: Vector3::x(),
                v_axis: Vector3::z(),
            },
            -0.050,
            0.050,
            -0.025,
            0.025,
        );
        assert_eq!(at_y.segments.len(), in_plane.segments.len());
        for (sa, sb) in at_y.segments.iter().zip(&in_plane.segments) {
            assert_eq!(sa[0], sb[0], "segment start must be bit-identical");
            assert_eq!(sa[1], sb[1], "segment end must be bit-identical");
        }
    }

    /// End-to-end tilted-basis check: a great circle of a sphere sampled in an
    /// arbitrary (tilted) seam plane must (a) read ≈ −r at the plane center and
    /// (b) have its silhouette points map back — via `to_world` through the
    /// tilted basis — onto the sphere surface (`|world| ≈ r`).
    #[test]
    fn tilted_basis_silhouette_points_map_back_onto_the_body_surface() {
        let r = 0.020;
        let body = Solid::sphere(r);
        // Orthonormal in-plane basis ⟂ a tilted normal (0.6, 0, 0.8).
        let u_axis = Vector3::new(0.8, 0.0, -0.6).normalize();
        let v_axis = Vector3::new(0.6, 0.0, 0.8).cross(&u_axis); // = +Y
        let sil = Silhouette2d::from_body_in_plane(
            &body,
            SeamPlaneBasis {
                anchor: Point3::origin(),
                u_axis,
                v_axis,
            },
            -0.030,
            0.030,
            -0.030,
            0.030,
        );
        assert!(sil.segment_count() > 0);
        assert!(
            (sil.signed_distance_to(0.0, 0.0) - (-r)).abs() < 2.0 * SILHOUETTE_GRID_STEP_M,
            "plane center is inside the sphere → ≈ -r",
        );
        for k in 0..8 {
            let t = f64::from(k) / 8.0;
            if let Some(p2) = sil.point_at_arc_fraction(t) {
                let world = sil.to_world(p2);
                assert!(
                    (world.coords.norm() - r).abs() < 2.0 * SILHOUETTE_GRID_STEP_M,
                    "silhouette point must map onto the sphere surface (|w| ≈ {r}); \
                     got {}",
                    world.coords.norm(),
                );
            }
        }
    }

    #[test]
    fn c_shape_silhouette_distance_in_concavity_matches_2d() {
        // C-shape body matching the §F-S1 falsification fixture: outer
        // cuboid 80×30×40 mm with thin-Y notch on +Z face.
        let outer = Solid::cuboid(Vector3::new(0.040, 0.015, 0.020));
        let notch = Solid::cuboid(Vector3::new(0.020, 0.0015, 0.013))
            .translate(Vector3::new(0.0, 0.0, 0.010));
        let body = outer.subtract(notch);
        let sil = Silhouette2d::from_body_at_y(&body, 0.0, -0.060, 0.060, -0.040, 0.040);

        // Probe inside the C's mouth at (0, +10 mm). 2D distance to the
        // notch floor at (0, -3 mm) is 13 mm; expect ≈+13 mm.
        let d_concave = sil.signed_distance_to(0.0, 0.010);
        assert!(
            (d_concave - 0.013).abs() < 2.0 * SILHOUETTE_GRID_STEP_M,
            "expected ≈+13 mm at C's mouth probe (0, +10), got {d_concave}"
        );

        // Probe on a convex outer edge (X=0, Z=-25 mm), 5 mm past the
        // -Z outer face. Expect ≈+5 mm.
        let d_convex = sil.signed_distance_to(0.0, -0.025);
        assert!(
            (d_convex - 0.005).abs() < SILHOUETTE_GRID_STEP_M,
            "expected ≈+5 mm past -Z outer edge, got {d_convex}"
        );

        // Probe inside the body's solid spine (X=-30 mm, Z=0 mm), 10 mm
        // inboard of -X outer face. Expect ≈-10 mm.
        let d_inside = sil.signed_distance_to(-0.030, 0.0);
        assert!(
            (d_inside - (-0.010)).abs() < SILHOUETTE_GRID_STEP_M,
            "expected ≈-10 mm inside body spine, got {d_inside}"
        );
    }

    #[test]
    fn point_to_segment_distance_endpoint_and_perpendicular_cases() {
        let a = Point2::new(0.0, 0.0);
        let b = Point2::new(1.0, 0.0);
        // Perpendicular drop from above midpoint.
        assert!((point_to_segment_distance(Point2::new(0.5, 0.3), a, b) - 0.3).abs() < 1e-12);
        // Past endpoint b (clamp to b).
        assert!((point_to_segment_distance(Point2::new(2.0, 0.0), a, b) - 1.0).abs() < 1e-12);
        // Past endpoint a (clamp to a).
        assert!((point_to_segment_distance(Point2::new(-0.5, 0.0), a, b) - 0.5).abs() < 1e-12);
        // Degenerate (a == b) collapses to point distance.
        assert!((point_to_segment_distance(Point2::new(3.0, 4.0), a, a) - 5.0).abs() < 1e-12);
    }

    /// Saddle case 10 with center OUTSIDE (r < step/√2). Two disjoint
    /// spheres at the saddle cell's corner-1 and corner-3 positions;
    /// the cell center is outside both. Correct MC handling cuts off
    /// each inside corner separately (segments `[e_bottom, e_right]`
    /// and `[e_top, e_left]`), so the cell-center probe sees 0 crossings
    /// within the saddle cell + 1 from the cell to the right + 1 from
    /// the cell below = 2 crossings = even = OUTSIDE. If the saddle
    /// branches are swapped, the saddle cell contributes 1 crossing
    /// (segments cut across the X+ ray) → total 3 → odd → wrongly INSIDE.
    #[test]
    fn marching_squares_case_10_saddle_center_outside_classifies_outside() {
        let step = SILHOUETTE_GRID_STEP_M;
        let r = 0.6 * step;
        let body = Solid::sphere(r)
            .translate(Vector3::new(step, 0.0, 0.0))
            .union(Solid::sphere(r).translate(Vector3::new(0.0, 0.0, step)));
        let sil = Silhouette2d::from_body_at_y(&body, 0.0, -step, 2.0 * step, -step, 2.0 * step);
        let d = sil.signed_distance_to(0.5 * step, 0.5 * step);
        assert!(
            d > 0.0,
            "saddle-10 cell center must be classified OUTSIDE when \
             r < step/√2 (spheres don't reach center); got signed_distance \
             = {d} (negative = wrongly inside, indicates case-10 saddle \
             branches are swapped between center-inside and center-outside)"
        );
    }

    /// Saddle case 10 with center INSIDE (r > step/√2). Two overlapping
    /// spheres at the saddle cell's corner-1 and corner-3 positions;
    /// the cell center is inside the union. Correct MC handling cuts
    /// off the outside corners (segments `[e_left, e_bottom]` and
    /// `[e_top, e_right]`), so the saddle cell contributes 0 crossings
    /// to the +X cell-center probe. With contribution from the cell
    /// to the right of the saddle (case 1, segment crosses +X ray
    /// once), total = 1 crossing = odd = INSIDE. Swapped branches
    /// produce 2 crossings → wrongly OUTSIDE.
    #[test]
    fn marching_squares_case_10_saddle_center_inside_classifies_inside() {
        let step = SILHOUETTE_GRID_STEP_M;
        let r = 0.8 * step;
        let body = Solid::sphere(r)
            .translate(Vector3::new(step, 0.0, 0.0))
            .union(Solid::sphere(r).translate(Vector3::new(0.0, 0.0, step)));
        let sil = Silhouette2d::from_body_at_y(&body, 0.0, -step, 2.0 * step, -step, 2.0 * step);
        let d = sil.signed_distance_to(0.5 * step, 0.5 * step);
        assert!(
            d < 0.0,
            "saddle-10 cell center must be classified INSIDE when \
             r > step/√2 (overlapping spheres cover center); got \
             signed_distance = {d} (positive = wrongly outside, indicates \
             case-10 saddle branches are swapped between center-inside \
             and center-outside)"
        );
    }

    /// §M-S2 polyline assembly: cylinder body silhouette assembles
    /// into closed polylines whose total arc length matches the
    /// rectangle perimeter (2 × (60 + 20) = 160 mm). MC corner
    /// handling can split the perimeter into multiple chained
    /// polylines depending on how walks terminate at corner-endpoint
    /// ambiguities; what matters for downstream §M-S2 dowel placement
    /// is that the LONGEST polyline captures most of the perimeter
    /// and arc-length-parameterizes correctly.
    #[test]
    fn cylinder_silhouette_polylines_cover_full_perimeter() {
        let body = cylinder_along_x();
        let sil = Silhouette2d::from_body_at_y(&body, 0.0, -0.050, 0.050, -0.025, 0.025);
        let polylines = sil.polylines();
        assert!(
            !polylines.is_empty(),
            "cylinder silhouette must produce at least 1 polyline; got 0"
        );
        // Sum of all polyline arc lengths should approximate perimeter.
        let total_assembled: f64 = polylines
            .iter()
            .map(|poly| {
                let mut s = 0.0_f64;
                for i in 0..poly.len() {
                    let a = poly[i];
                    let b = poly[(i + 1) % poly.len()];
                    let dx = b.x - a.x;
                    let dz = b.z - a.z;
                    s += dx.mul_add(dx, dz * dz).sqrt();
                }
                s
            })
            .sum();
        let expected_perim = 2.0 * (0.060 + 0.020); // 2 × (length + width)
        assert!(
            (total_assembled - expected_perim).abs() < 0.005,
            "assembled-polyline total arc length should match perimeter \
             160 mm; got {:.4} m across {} polylines",
            total_assembled,
            polylines.len()
        );
        // Longest polyline must cover at least half the perimeter
        // (so dowel arc-length placement on it is meaningful).
        let (_poly, _cum, longest) = sil.longest_polyline_with_arc_length().unwrap();
        assert!(
            longest > 0.5 * expected_perim,
            "longest polyline must cover > 50% of perimeter for \
             arc-length placement; got {longest:.4} m / {expected_perim:.4} m"
        );
    }

    /// §M-S2 arc-length parameterization: 4 evenly-spaced points
    /// around the cylinder silhouette should land on its rectangular
    /// perimeter at roughly 25/50/75/100% positions.
    #[test]
    fn arc_fraction_walks_around_silhouette() {
        let body = cylinder_along_x();
        let sil = Silhouette2d::from_body_at_y(&body, 0.0, -0.050, 0.050, -0.025, 0.025);
        let total = sil.longest_polyline_with_arc_length().unwrap().2;
        // All 4 sample points must be on the perimeter (signed_distance ≈ 0).
        for i in 0..4 {
            let t = f64::from(i) / 4.0;
            let p = sil.point_at_arc_fraction(t).unwrap();
            let d = sil.signed_distance_to(p.x, p.z).abs();
            assert!(
                d < SILHOUETTE_GRID_STEP_M,
                "arc-fraction {t} sample {p:?} must be on silhouette \
                 (|signed_dist| < grid step); got {d:.6} m"
            );
        }
        // Going around the loop once should return to ~start.
        let p0 = sil.point_at_arc_fraction(0.0).unwrap();
        let p1 = sil.point_at_arc_fraction(1.0).unwrap();
        let d = (p1.x - p0.x).hypot(p1.z - p0.z);
        assert!(
            d < SILHOUETTE_GRID_STEP_M,
            "t=0 and t=1 must return same point (closed loop); got {d:.6} m apart"
        );
        // Sanity that we actually walked a non-trivial arc length.
        assert!(total > 0.100, "perimeter must be > 0.1m, got {total}");
    }

    /// §M-S2 outward normal: at multiple arc positions on the
    /// cylinder silhouette, the outward normal must point AWAY from
    /// the body interior (positive `signed_distance` side).
    #[test]
    fn outward_normal_points_away_from_body() {
        let body = cylinder_along_x();
        let sil = Silhouette2d::from_body_at_y(&body, 0.0, -0.050, 0.050, -0.025, 0.025);
        for i in 0..8 {
            let t = f64::from(i) / 8.0;
            let p = sil.point_at_arc_fraction(t).unwrap();
            let n = sil.outward_normal_at_arc_fraction(t).unwrap();
            // Probe 1 mm in normal direction from p; signed_distance
            // should be positive (outside body).
            let probe_x = 0.001_f64.mul_add(n.x, p.x);
            let probe_z = 0.001_f64.mul_add(n.z, p.z);
            let d = sil.signed_distance_to(probe_x, probe_z);
            assert!(
                d > 0.0,
                "outward normal at t={t} must point OUT of body; \
                 probe 1 mm out got signed_distance = {d:.6} m (should be positive)"
            );
        }
    }

    #[test]
    fn ray_cast_handles_strict_inequality_and_horizontal_segments() {
        let q = Point2::new(0.0, 0.0);
        // Segment fully above query → no crossing.
        assert!(!ray_cast_crosses(
            q,
            Point2::new(1.0, 1.0),
            Point2::new(2.0, 2.0)
        ));
        // Segment fully below → no crossing.
        assert!(!ray_cast_crosses(
            q,
            Point2::new(1.0, -1.0),
            Point2::new(2.0, -2.0)
        ));
        // Segment straddles, intersect at X=+1.5 > 0 → crossing.
        assert!(ray_cast_crosses(
            q,
            Point2::new(1.0, -1.0),
            Point2::new(2.0, 1.0)
        ));
        // Segment straddles, intersect at X=-0.5 < 0 → no crossing.
        assert!(!ray_cast_crosses(
            q,
            Point2::new(-1.0, -1.0),
            Point2::new(0.0, 1.0)
        ));
    }
}
