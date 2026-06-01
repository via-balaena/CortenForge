//! Clean analytic seam-placement substrate (§3.1 of
//! `docs/CF_CAST_SEAM_PLACEMENT_RECON.md`).
//!
//! The seam-placement solver (S2+) places fasteners on a 1-D closed loop. It
//! must NOT place directly on the raw marching-squares silhouette polyline:
//! that loop has vertices at the fixed [`SILHOUETTE_GRID_STEP_M`] (0.5 mm) grid
//! crossings, so it carries a sub-millimetre staircase and its vertex spacing is
//! non-uniform. Positions on it are fine (the staircase is ≤ 0.5 mm, below every
//! fastener tolerance), but its *derivatives* — the outward normal and the
//! curvature the corner-seeder needs — are dominated by that staircase.
//!
//! NOTE (S1, 2026-05-31): the silhouette grid is a FIXED 0.5 mm const
//! ([`SILHOUETTE_GRID_STEP_M`]), independent of `mesh_cell_size_m`, so placement
//! is *already* mesh-cell-independent (empirically: `base_mold`'s outer-layer
//! seam perimeter is identical at 3 mm and 0.5 mm mesh cells). The substrate's
//! job is therefore NOT to decouple from the mesh cell (already true) but to (a)
//! resample to uniform arc-length stations and (b) compute STABLE normals +
//! curvature despite the 0.5 mm staircase. We do (b) shrink-free: the stations
//! are the faithful resampled loop (no Laplacian point-smoothing, which would
//! shrink high-curvature regions), and the derivatives are windowed central
//! differences over a ~`DERIV_WINDOW_M` arc — the low-pass lives in the
//! derivative stencil, not in the point positions.
//!
//! `SeamProfile` exposes: arc-length `point_at` / `tangent_at` /
//! `outward_normal_at` / `curvature_at`, a `signed_distance` to the clean loop
//! (the cell-independent feasibility field, §3.3), `nearest_arc` (per-layer snap,
//! §3.8), and the world mapping via the seam-plane basis. It is a pure addition;
//! nothing in the build pipeline calls it yet (wired in S3).

// 2-D seam-plane geometry: f64↔index casts (grid/station counts) are bounded
// and non-negative, and short coordinate names (x, z, a, b, n) are idiomatic for
// the point/segment math here — same rationale + allow set as `silhouette_2d`.
#![allow(
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::many_single_char_names,
    clippy::similar_names
)]

use crate::silhouette_2d::{Point2, SeamPlaneBasis, Silhouette2d};
use nalgebra::{Point3, Vector3};

/// Default uniform resample step along the seam loop (1 mm).
///
/// Fine enough that linear interpolation between stations is sub-0.1 mm on
/// organic curvature, coarse enough to keep a ~400 mm seam at a few hundred
/// stations.
pub const DEFAULT_RESAMPLE_STEP_M: f64 = 0.001;

/// Default arc half-window for the central-difference tangent / normal stencil.
///
/// 2 mm: wide enough to average out the 0.5 mm silhouette staircase, narrow
/// enough to follow real organic curvature.
pub const DEFAULT_DERIV_WINDOW_M: f64 = 0.002;

/// Default arc half-window for the curvature estimate (8 mm).
///
/// Curvature is a second derivative, so it needs a wider stencil than the
/// normal: the turning signal over a window `W` is `~W/r` while the staircase
/// tangent noise is `~grid/W`, so a usable curvature needs `W ≳ √(grid·r)`
/// (≈ 4 mm on a 30 mm radius). 8 mm keeps smooth-region curvature well below any
/// real corner so the corner-seeder (§3.4) doesn't false-trigger on staircase
/// noise — a localized real corner (flat-cap edge) still spikes Menger curvature
/// far above the band.
pub const DEFAULT_CURV_WINDOW_M: f64 = 0.008;

/// A clean, uniformly-resampled closed seam loop with stable derivatives.
///
/// Built from a [`Silhouette2d`] (or a raw polyline) in the silhouette's
/// seam-plane basis. All arc-length arguments are in meters, wrapped into
/// `[0, perimeter)`. 2-D coordinates are the seam-plane `(x, z)` of [`Point2`];
/// map to world with [`Self::to_world`].
#[derive(Debug, Clone)]
pub struct SeamProfile {
    basis: SeamPlaneBasis,
    /// Uniform arc-length stations (closed loop; `stations[last]` connects back
    /// to `stations[0]`, which is NOT duplicated).
    stations: Vec<Point2>,
    /// Cumulative arc length, `cum.len() == stations.len() + 1`, `cum[0] = 0`,
    /// `cum[N] = perimeter` (closing edge included).
    cum: Vec<f64>,
    perimeter: f64,
    /// Half-window (in stations) for the derivative stencil.
    deriv_window: usize,
}

impl SeamProfile {
    /// Build from a [`Silhouette2d`] using the default resample step + derivative
    /// window. Uses the silhouette's longest polyline (the body perimeter).
    /// `None` if the silhouette has no polyline or too few vertices.
    #[must_use]
    pub fn from_silhouette(silhouette: &Silhouette2d) -> Option<Self> {
        Self::from_silhouette_with(silhouette, DEFAULT_RESAMPLE_STEP_M, DEFAULT_DERIV_WINDOW_M)
    }

    /// Build from a [`Silhouette2d`] with explicit resample step + derivative
    /// window.
    #[must_use]
    pub fn from_silhouette_with(
        silhouette: &Silhouette2d,
        step_m: f64,
        deriv_window_m: f64,
    ) -> Option<Self> {
        let (poly, _cum, _total) = silhouette.longest_polyline_with_arc_length()?;
        Self::from_polyline(&poly, silhouette.basis(), step_m, deriv_window_m)
    }

    /// Core constructor: resample a raw closed polyline (vertices in order, the
    /// `last → first` edge implicit) to uniform arc-length stations. `None` if
    /// fewer than 3 vertices or zero length.
    #[must_use]
    pub fn from_polyline(
        raw: &[Point2],
        basis: SeamPlaneBasis,
        step_m: f64,
        deriv_window_m: f64,
    ) -> Option<Self> {
        if raw.len() < 3 || step_m <= 0.0 {
            return None;
        }
        // Raw cumulative arc length (closed).
        let n = raw.len();
        let mut raw_cum = Vec::with_capacity(n + 1);
        raw_cum.push(0.0);
        let mut total = 0.0;
        for i in 0..n {
            let a = raw[i];
            let b = raw[(i + 1) % n];
            total += dist(a, b);
            raw_cum.push(total);
        }
        if total <= 0.0 {
            return None;
        }
        // Uniform stations at arc lengths k * total / m, k = 0..m.
        let m = ((total / step_m).round() as usize).max(8);
        let mut stations = Vec::with_capacity(m);
        for k in 0..m {
            let s = k as f64 * total / m as f64;
            stations.push(sample_polyline(raw, &raw_cum, s));
        }
        // Recompute cumulative arc length on the uniform stations.
        let (cum, perimeter) = cumulative(&stations);
        if perimeter <= 0.0 {
            return None;
        }
        let spacing = perimeter / m as f64;
        let deriv_window = ((deriv_window_m / spacing).round() as usize).max(1);
        Some(Self {
            basis,
            stations,
            cum,
            perimeter,
            deriv_window,
        })
    }

    /// Total seam-loop length (meters).
    #[must_use]
    pub const fn perimeter(&self) -> f64 {
        self.perimeter
    }

    /// Number of uniform stations.
    #[must_use]
    pub const fn station_count(&self) -> usize {
        self.stations.len()
    }

    /// The seam-plane basis (world mapping).
    #[must_use]
    pub const fn basis(&self) -> SeamPlaneBasis {
        self.basis
    }

    /// Point on the loop at arc length `s` (wrapped into `[0, perimeter)`),
    /// linearly interpolated between stations.
    #[must_use]
    pub fn point_at(&self, s: f64) -> Point2 {
        let s = s.rem_euclid(self.perimeter);
        let (i, frac) = self.locate(s);
        let a = self.stations[i];
        let b = self.stations[(i + 1) % self.stations.len()];
        Point2::new(a.x + (b.x - a.x) * frac, a.z + (b.z - a.z) * frac)
    }

    /// Unit tangent at arc length `s` via a windowed central difference (low-pass
    /// over the 0.5 mm staircase). Direction follows increasing arc length.
    #[must_use]
    pub fn tangent_at(&self, s: f64) -> Point2 {
        let n = self.stations.len();
        let j = self.nearest_station(s);
        let w = self.deriv_window.min(n / 2).max(1);
        let a = self.stations[(j + n - w) % n];
        let b = self.stations[(j + w) % n];
        normalize(Point2::new(b.x - a.x, b.z - a.z))
    }

    /// Unit outward normal at arc length `s` (perpendicular to the tangent,
    /// oriented to point OUT of the loop interior).
    #[must_use]
    pub fn outward_normal_at(&self, s: f64) -> Point2 {
        let t = self.tangent_at(s);
        // The two perpendiculars; pick the one that increases signed distance.
        let cand = Point2::new(t.z, -t.x);
        let p = self.point_at(s);
        // Small probe: `p` is ON the loop (sd ≈ 0), so a short step gives a clean
        // sign while staying inside any concavity narrower than a 2 mm probe.
        let probe = 0.5 * self.spacing();
        let outward = self.signed_distance(Point2::new(p.x + cand.x * probe, p.z + cand.z * probe))
            >= self.signed_distance(Point2::new(p.x - cand.x * probe, p.z - cand.z * probe));
        if outward {
            cand
        } else {
            Point2::new(-cand.x, -cand.z)
        }
    }

    /// Unsigned curvature (1/radius, meters⁻¹) at arc length `s` via the Menger
    /// (circumscribed-circle) curvature of three loop points spanning ±a
    /// curvature window. Robust to the 0.5 mm staircase (any three points ON a
    /// circle recover its curvature exactly; the wide window averages the noise).
    #[must_use]
    pub fn curvature_at(&self, s: f64) -> f64 {
        // Clamp the window to perimeter/4 so the three Menger points never go
        // near-antipodal on a tiny loop (which would return garbage).
        let c = DEFAULT_CURV_WINDOW_M
            .max(2.0 * self.spacing())
            .min(self.perimeter / 4.0);
        menger_curvature(self.point_at(s - c), self.point_at(s), self.point_at(s + c))
    }

    /// Signed distance from seam-plane point `p` to the clean loop. Negative =
    /// inside, positive = outside; magnitude is Euclidean distance to the nearest
    /// edge. This is the cell-independent feasibility field (§3.3).
    #[must_use]
    pub fn signed_distance(&self, p: Point2) -> f64 {
        let n = self.stations.len();
        let mut unsigned = f64::INFINITY;
        let mut crossings = 0u32;
        for i in 0..n {
            let a = self.stations[i];
            let b = self.stations[(i + 1) % n];
            unsigned = unsigned.min(point_to_segment(p, a, b));
            if ray_cast_crosses(p, a, b) {
                crossings += 1;
            }
        }
        if crossings % 2 == 1 {
            -unsigned
        } else {
            unsigned
        }
    }

    /// Arc length of the loop point nearest seam-plane `p` (per-layer snap, §3.8).
    #[must_use]
    pub fn nearest_arc(&self, p: Point2) -> f64 {
        let n = self.stations.len();
        let mut best = (f64::INFINITY, 0.0);
        for i in 0..n {
            let a = self.stations[i];
            let b = self.stations[(i + 1) % n];
            let (d, frac) = point_to_segment_param(p, a, b);
            if d < best.0 {
                let seg = self.cum[i + 1] - self.cum[i];
                best = (d, self.cum[i] + frac * seg);
            }
        }
        best.1
    }

    /// Footprint feasibility (§3.3 incremental regime): the disk of radius
    /// `footprint_r` about seam-plane `center` lies entirely within the flange
    /// band `inner ≤ signed_distance ≤ width`.
    ///
    /// `signed_distance` is a true (1-Lipschitz) distance field, so the disk's
    /// signed distance is bounded in `[sd(c) − ρ, sd(c) + ρ]`; the disk is inside
    /// the band iff `inner + ρ ≤ sd(c) ≤ width − ρ`. This single center test is
    /// exact-conservative (never false-accepts) and pinch-aware (sd is the true
    /// nearest-boundary distance in ALL directions) — strictly better than
    /// sampling the disk perimeter, which can false-accept a boundary intruding
    /// between samples.
    #[must_use]
    pub fn band_feasible(&self, center: Point2, footprint_r: f64, inner: f64, width: f64) -> bool {
        let sd = self.signed_distance(center);
        sd >= inner + footprint_r && sd <= width - footprint_r
    }

    /// Map a seam-plane point to world.
    #[must_use]
    pub fn to_world(&self, p: Point2) -> Point3<f64> {
        self.basis.to_world(p)
    }

    /// Map a seam-plane direction to world.
    #[must_use]
    pub fn dir_to_world(&self, p: Point2) -> Vector3<f64> {
        self.basis.dir_to_world(p)
    }

    // --- internals ---

    fn spacing(&self) -> f64 {
        self.perimeter / self.stations.len().max(1) as f64
    }

    /// Station index `i` + fraction into segment `i→i+1` for arc length `s`.
    fn locate(&self, s: f64) -> (usize, f64) {
        // cum is monotone; binary search the segment containing s.
        let n = self.stations.len();
        let mut lo = 0usize;
        let mut hi = n; // cum[n] = perimeter
        while lo + 1 < hi {
            let mid = lo.midpoint(hi);
            if self.cum[mid] <= s {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        let seg = self.cum[lo + 1] - self.cum[lo];
        let frac = if seg > 0.0 {
            (s - self.cum[lo]) / seg
        } else {
            0.0
        };
        (lo % n, frac.clamp(0.0, 1.0))
    }

    fn nearest_station(&self, s: f64) -> usize {
        let s = s.rem_euclid(self.perimeter);
        let (i, frac) = self.locate(s);
        if frac >= 0.5 {
            (i + 1) % self.stations.len()
        } else {
            i
        }
    }
}

// --- free helpers ---

fn dist(a: Point2, b: Point2) -> f64 {
    let dx = b.x - a.x;
    let dz = b.z - a.z;
    dx.mul_add(dx, dz * dz).sqrt()
}

/// Menger curvature (1/circumradius) of three points: `4·area / (a·b·c)` where
/// `a,b,c` are the triangle side lengths. Zero for collinear points.
fn menger_curvature(p0: Point2, p1: Point2, p2: Point2) -> f64 {
    let a = dist(p0, p1);
    let b = dist(p1, p2);
    let c = dist(p2, p0);
    let cross = (p1.x - p0.x) * (p2.z - p0.z) - (p1.z - p0.z) * (p2.x - p0.x);
    let denom = a * b * c;
    if denom > 0.0 {
        2.0 * cross.abs() / denom
    } else {
        0.0
    }
}

fn normalize(p: Point2) -> Point2 {
    let n = p.x.hypot(p.z);
    if n > 0.0 {
        Point2::new(p.x / n, p.z / n)
    } else {
        Point2::new(1.0, 0.0)
    }
}

fn cumulative(stations: &[Point2]) -> (Vec<f64>, f64) {
    let n = stations.len();
    let mut cum = Vec::with_capacity(n + 1);
    cum.push(0.0);
    let mut total = 0.0;
    for i in 0..n {
        total += dist(stations[i], stations[(i + 1) % n]);
        cum.push(total);
    }
    (cum, total)
}

/// Point on a closed raw polyline at arc length `s` (`raw_cum` has `len + 1` entries).
fn sample_polyline(raw: &[Point2], raw_cum: &[f64], s: f64) -> Point2 {
    let n = raw.len();
    // binary search segment
    let mut lo = 0usize;
    let mut hi = n;
    while lo + 1 < hi {
        let mid = lo.midpoint(hi);
        if raw_cum[mid] <= s {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    let a = raw[lo % n];
    let b = raw[(lo + 1) % n];
    let seg = raw_cum[lo + 1] - raw_cum[lo];
    let frac = if seg > 0.0 {
        (s - raw_cum[lo]) / seg
    } else {
        0.0
    };
    Point2::new(a.x + (b.x - a.x) * frac, a.z + (b.z - a.z) * frac)
}

fn point_to_segment(p: Point2, a: Point2, b: Point2) -> f64 {
    point_to_segment_param(p, a, b).0
}

/// `(distance, fraction-along-segment)` of the closest point on `a→b` to `p`.
fn point_to_segment_param(p: Point2, a: Point2, b: Point2) -> (f64, f64) {
    let vx = b.x - a.x;
    let vz = b.z - a.z;
    let len2 = vx.mul_add(vx, vz * vz);
    if len2 <= 0.0 {
        return (dist(p, a), 0.0);
    }
    let t = ((p.x - a.x) * vx + (p.z - a.z) * vz) / len2;
    let t = t.clamp(0.0, 1.0);
    let cx = a.x + vx * t;
    let cz = a.z + vz * t;
    (dist(p, Point2::new(cx, cz)), t)
}

/// Horizontal ray (+x) crossing test for even-odd inside/outside parity.
fn ray_cast_crosses(p: Point2, a: Point2, b: Point2) -> bool {
    let (a, b) = if a.z <= b.z { (a, b) } else { (b, a) };
    if p.z < a.z || p.z >= b.z {
        return false;
    }
    // x of the edge at height p.z
    let t = (p.z - a.z) / (b.z - a.z);
    let x = a.x + (b.x - a.x) * t;
    x > p.x
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

    fn circle(r: f64, n: usize) -> Vec<Point2> {
        (0..n)
            .map(|i| {
                let th = std::f64::consts::TAU * (i as f64) / (n as f64);
                Point2::new(r * th.cos(), r * th.sin())
            })
            .collect()
    }

    fn y_basis() -> SeamPlaneBasis {
        SeamPlaneBasis::y_normal(0.0)
    }

    /// A non-convex "peanut": `r(θ) = R·(1 + m·cos 2θ)` — two lobes with a
    /// concave waist for `m = 0.4`.
    fn peanut(big_r: f64, m: f64, n: usize) -> Vec<Point2> {
        (0..n)
            .map(|i| {
                let th = std::f64::consts::TAU * (i as f64) / (n as f64);
                let rr = big_r * m.mul_add((2.0 * th).cos(), 1.0);
                Point2::new(rr * th.cos(), rr * th.sin())
            })
            .collect()
    }

    /// A square loop (sharp corners): `pts_per_side` points per `side`-long edge.
    fn square(side: f64, pts_per_side: usize) -> Vec<Point2> {
        let corners = [(0.0, 0.0), (side, 0.0), (side, side), (0.0, side)];
        let mut pts = Vec::new();
        for c in 0..4 {
            let (x0, z0) = corners[c];
            let (x1, z1) = corners[(c + 1) % 4];
            for i in 0..pts_per_side {
                let t = i as f64 / pts_per_side as f64;
                pts.push(Point2::new(x0 + (x1 - x0) * t, z0 + (z1 - z0) * t));
            }
        }
        pts
    }

    #[test]
    fn non_convex_loop_outward_normal_and_inside() {
        let prof =
            SeamProfile::from_polyline(&peanut(0.030, 0.4, 360), y_basis(), 0.001, 0.002).unwrap();
        // origin is inside both lobes' overlap → strictly inside
        assert!(
            prof.signed_distance(Point2::new(0.0, 0.0)) < 0.0,
            "origin inside peanut"
        );
        for k in 0..48 {
            let s = prof.perimeter() * f64::from(k) / 48.0;
            let p = prof.point_at(s);
            let nrm = prof.outward_normal_at(s);
            let eps = 0.0008;
            let out = prof.signed_distance(Point2::new(
                nrm.x.mul_add(eps, p.x),
                nrm.z.mul_add(eps, p.z),
            ));
            let inn = prof.signed_distance(Point2::new(
                nrm.x.mul_add(-eps, p.x),
                nrm.z.mul_add(-eps, p.z),
            ));
            // stepping outward must increase signed distance and land outside,
            // even across the concave waist.
            assert!(
                out > inn,
                "outward must increase sd at s={s}: out={out} inn={inn}"
            );
            assert!(out > 0.0, "outward step lands outside at s={s}: {out}");
        }
    }

    #[test]
    fn sharp_corner_spikes_curvature() {
        let side = 0.040;
        let prof = SeamProfile::from_polyline(&square(side, 40), y_basis(), 0.001, 0.002).unwrap();
        // corner sits at arc 0 (first vertex); edge midpoint at side/2.
        let corner_k = prof.curvature_at(0.0);
        let edge_k = prof.curvature_at(side / 2.0);
        assert!(
            edge_k < 20.0,
            "straight edge curvature should be ~0, got {edge_k}"
        );
        assert!(
            corner_k > 100.0,
            "sharp corner curvature should spike, got {corner_k}"
        );
        assert!(
            corner_k > 5.0 * edge_k.max(1.0),
            "corner must dominate edge ({corner_k} vs {edge_k})"
        );
    }

    #[test]
    fn circle_perimeter_and_curvature() {
        let r = 0.030;
        let prof = SeamProfile::from_polyline(&circle(r, 256), y_basis(), 0.001, 0.002).unwrap();
        assert!(
            (prof.perimeter() - std::f64::consts::TAU * r).abs() < 0.0005,
            "perimeter {} vs {}",
            prof.perimeter(),
            std::f64::consts::TAU * r
        );
        // curvature of a circle is 1/r everywhere
        for k in 0..8 {
            let s = prof.perimeter() * (f64::from(k) + 0.5) / 8.0;
            let kappa = prof.curvature_at(s);
            assert!(
                (kappa - 1.0 / r).abs() < 0.10 / r,
                "curvature {kappa} vs {} at s={s}",
                1.0 / r
            );
        }
    }

    #[test]
    fn outward_normal_points_away_from_center() {
        let prof =
            SeamProfile::from_polyline(&circle(0.030, 200), y_basis(), 0.001, 0.002).unwrap();
        for k in 0..16 {
            let s = prof.perimeter() * f64::from(k) / 16.0;
            let p = prof.point_at(s);
            let nrm = prof.outward_normal_at(s);
            // on a circle centered at origin, outward normal ≈ radial unit
            let radial = normalize(p);
            let dot = nrm.x * radial.x + nrm.z * radial.z;
            assert!(dot > 0.9, "outward dot radial = {dot} at s={s}");
        }
    }

    #[test]
    fn uniform_station_spacing() {
        let prof = SeamProfile::from_polyline(&circle(0.030, 37), y_basis(), 0.001, 0.002).unwrap();
        let sp = prof.perimeter() / prof.station_count() as f64;
        for i in 0..prof.station_count() {
            let a = prof.stations[i];
            let b = prof.stations[(i + 1) % prof.station_count()];
            assert!((dist(a, b) - sp).abs() < 0.1 * sp, "spacing drift at {i}");
        }
    }

    #[test]
    fn signed_distance_inside_outside() {
        let r = 0.030;
        let prof = SeamProfile::from_polyline(&circle(r, 256), y_basis(), 0.001, 0.002).unwrap();
        assert!(
            prof.signed_distance(Point2::new(0.0, 0.0)) < 0.0,
            "center inside"
        );
        let d = prof.signed_distance(Point2::new(r + 0.010, 0.0));
        assert!((d - 0.010).abs() < 0.0005, "outside distance {d}");
    }

    #[test]
    fn band_feasibility_matches_offset_window() {
        let r = 0.030;
        let prof = SeamProfile::from_polyline(&circle(r, 256), y_basis(), 0.001, 0.002).unwrap();
        let (inner, width, washer) = (0.002, 0.020, 0.005);
        // center at body_dist = 13mm → disk spans [8,18]mm ⊂ [2,20] → feasible
        let c_ok = Point2::new(r + 0.013, 0.0);
        assert!(prof.band_feasible(c_ok, washer, inner, width));
        // center at body_dist = 4mm → disk reaches body_dist -1mm < inner → infeasible
        let c_lo = Point2::new(r + 0.004, 0.0);
        assert!(!prof.band_feasible(c_lo, washer, inner, width));
        // center at body_dist = 17mm → disk reaches 22mm > width → infeasible
        let c_hi = Point2::new(r + 0.017, 0.0);
        assert!(!prof.band_feasible(c_hi, washer, inner, width));
    }

    /// The stability gate: windowed normals + curvature must be robust to the
    /// 0.5 mm silhouette staircase. Add ±0.4 mm radial staircase noise to a
    /// circle and confirm normals/curvature stay close to the clean values.
    #[test]
    fn derivatives_stable_under_staircase_noise() {
        let r = 0.030;
        let clean = circle(r, 256);
        // quantize each vertex to a 0.5 mm grid → staircase like marching squares
        let grid = 0.0005;
        let noisy: Vec<Point2> = clean
            .iter()
            .map(|p| Point2::new((p.x / grid).round() * grid, (p.z / grid).round() * grid))
            .collect();
        let pc = SeamProfile::from_polyline(&clean, y_basis(), 0.001, 0.002).unwrap();
        let pn = SeamProfile::from_polyline(&noisy, y_basis(), 0.001, 0.002).unwrap();
        let mut max_norm_err = 0.0_f64;
        let mut max_curv_err = 0.0_f64;
        let mut max_noisy_curv = 0.0_f64;
        for k in 0..64 {
            let s = pc.perimeter() * f64::from(k) / 64.0;
            let nc = pc.outward_normal_at(s);
            let nn = pn.outward_normal_at(s);
            let dot = (nc.x * nn.x + nc.z * nn.z).clamp(-1.0, 1.0);
            max_norm_err = max_norm_err.max(dot.acos().to_degrees());
            max_curv_err = max_curv_err.max((pc.curvature_at(s) - pn.curvature_at(s)).abs() * r);
            max_noisy_curv = max_noisy_curv.max(pn.curvature_at(s) * r);
        }
        // Normals stay well-aligned, curvature stays close to the clean 1/r, and
        // crucially the noise NEVER spikes curvature to a corner-like value
        // (≥ 3/r) that would false-trigger the corner-seeder.
        assert!(
            max_norm_err < 8.0,
            "normal angle drift {max_norm_err}° under staircase"
        );
        assert!(
            max_curv_err < 0.5,
            "curvature drift {max_curv_err} (×r) under staircase"
        );
        assert!(
            max_noisy_curv < 3.0,
            "noisy curvature spiked to {max_noisy_curv}×(1/r) — false corner"
        );
    }

    #[test]
    fn nearest_arc_round_trips() {
        let prof =
            SeamProfile::from_polyline(&circle(0.030, 256), y_basis(), 0.001, 0.002).unwrap();
        for k in 0..16 {
            let s = prof.perimeter() * (f64::from(k) + 0.3) / 16.0;
            let p = prof.point_at(s);
            let s2 = prof.nearest_arc(p);
            let gap = (s2 - s).rem_euclid(prof.perimeter());
            let gap = gap.min(prof.perimeter() - gap);
            assert!(gap < 0.0015, "nearest_arc round-trip gap {gap} at s={s}");
        }
    }

    #[test]
    fn too_few_vertices_returns_none() {
        assert!(
            SeamProfile::from_polyline(&[Point2::new(0.0, 0.0)], y_basis(), 0.001, 0.002).is_none()
        );
    }
}
