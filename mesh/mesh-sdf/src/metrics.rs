//! Surface-fidelity metrics: how far one surface deviates from a reference.
//!
//! Rung 0 of the geometry-fidelity ladder — the instrument that turns "is
//! this geometry faithful?" from an opinion into a number. Two metrics,
//! both returning a [`DeviationReport`]:
//!
//! - [`surface_deviation_to_sdf`] samples a triangle mesh's surface and
//!   evaluates a reference [`Sdf`] at each sample. Because the mesh should
//!   lie on the field's zero level set, the field value *is* the pointwise
//!   deviation. This measures meshing error against an analytic ground
//!   truth (e.g. how far a marching-cubes surface strays from the true
//!   shape). It is **directional**: it sees points that are *on* the mesh
//!   but not surface regions the mesher *dropped* — use
//!   [`hausdorff_distance`] to catch those.
//!
//! - [`hausdorff_distance`] is the symmetric sampled Hausdorff distance
//!   between two meshes (the max of both one-sided directions). This is the
//!   literal mesh→SDF→mesh round-trip metric: compare an original mesh
//!   against one re-meshed from its SDF. It catches **both** dropped and
//!   spurious geometry.
//!
//! # Sampling
//!
//! Both metrics sample each triangle at its centroid plus
//! [`SampleOptions::samples_per_triangle`] deterministic low-discrepancy
//! (Halton) interior points — no RNG, so repeated runs of the same input are
//! bit-identical. Per-sample deviations are **area-weighted**, so
//! `mean_abs` / `rms` approximate the true surface average and become
//! independent of tessellation density in the dense-sampling limit (at a
//! fixed `samples_per_triangle` a small centroid-sampling bias remains).
//! `max_abs` is a sampled *lower bound* on the true Hausdorff supremum; it
//! tightens as `samples_per_triangle` grows.
//!
//! # Malformed input
//!
//! Every failure mode is surfaced as an [`SdfError`], never a panic or a
//! silently misleading score: an empty mesh ([`SdfError::EmptyMesh`]), a
//! face referencing a vertex beyond the vertex list
//! ([`SdfError::FaceIndexOutOfRange`]), or a non-finite sample from corrupt
//! vertices or reference field ([`SdfError::NonFiniteDeviation`]). A
//! degenerate mesh whose faces all have zero area still reports its real
//! deviation via an unweighted sample mean rather than a misleading zero.

// A handful of `usize`/`u32` → `f64` casts for sample counts and weights;
// all values are small (sample indices, triangle counts) and far below the
// f64 integer-exactness limit.
#![allow(clippy::cast_precision_loss)]

use cf_geometry::Sdf;
use mesh_types::{IndexedMesh, Point3};

use crate::{SdfError, SdfResult, TriMeshDistance, UnsignedDistance};

/// Signed deviation extremes across all samples, in meters.
///
/// Carried only by the signed metric [`surface_deviation_to_sdf`]: a
/// negative value means a sampled mesh point sits *inside* the reference
/// field's zero level set, positive means *outside*. So `min < 0 < max`
/// indicates a mesh that straddles the true surface; `max <= 0` a mesh that
/// lies entirely inside it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SignedExtremes {
    /// Most-negative (deepest-inside) signed deviation.
    pub min: f64,
    /// Most-positive (furthest-outside) signed deviation.
    pub max: f64,
}

/// Aggregate deviation statistics between a sampled surface and a reference.
///
/// All distances are in meters. Produced by [`surface_deviation_to_sdf`]
/// (signed, against an [`Sdf`]) and [`hausdorff_distance`] (unsigned,
/// against another mesh).
#[derive(Debug, Clone, PartialEq)]
pub struct DeviationReport {
    /// Maximum absolute deviation over all samples. A sampled *lower
    /// bound* on the true one-sided (or, for [`hausdorff_distance`],
    /// symmetric) Hausdorff distance — it tightens as sampling density
    /// grows.
    pub max_abs: f64,
    /// Area-weighted mean of the absolute deviation (see the module `Sampling`
    /// section for the density-independence caveat).
    pub mean_abs: f64,
    /// Area-weighted root-mean-square deviation (`≥ mean_abs`).
    pub rms: f64,
    /// Signed inside/outside extremes for the signed metric
    /// ([`surface_deviation_to_sdf`]); `None` for the unsigned mesh↔mesh
    /// metric, which has no inside/outside notion.
    pub signed_extremes: Option<SignedExtremes>,
    /// Number of surface samples aggregated.
    pub n_samples: usize,
}

/// Controls surface-sample density for the deviation metrics.
#[derive(Debug, Clone, Copy)]
pub struct SampleOptions {
    /// Deterministic low-discrepancy (Halton) interior points drawn per
    /// triangle, **in addition to** the centroid. The centroid is always
    /// sampled because it is the worst case for a chord bulging away from a
    /// curved reference. A value of `0` is treated as `1` by the metric
    /// functions, so every triangle always contributes at least the centroid
    /// plus one interior point.
    pub samples_per_triangle: usize,
}

impl Default for SampleOptions {
    fn default() -> Self {
        Self {
            samples_per_triangle: 8,
        }
    }
}

/// Signed deviation of a mesh surface from a reference signed-distance
/// field.
///
/// Samples `mesh`'s surface (centroid plus low-discrepancy interior points
/// per triangle — see the module-level `Sampling` section) and evaluates
/// `reference` at each sample. Because the mesh is expected to
/// lie on the field's zero level set, `reference.eval(p)` is the pointwise
/// deviation: negative where the mesh sits inside the true surface,
/// positive outside.
///
/// # Errors
///
/// - [`SdfError::EmptyMesh`] if `mesh` has no faces.
/// - [`SdfError::FaceIndexOutOfRange`] if a face references a vertex index
///   beyond the vertex list.
/// - [`SdfError::NonFiniteDeviation`] if any sample evaluates to NaN or an
///   infinity (corrupt vertices or reference field) — surfaced rather than
///   silently reported as zero deviation.
pub fn surface_deviation_to_sdf(
    mesh: &IndexedMesh,
    reference: &dyn Sdf,
    opts: SampleOptions,
) -> SdfResult<DeviationReport> {
    if mesh.faces.is_empty() {
        return Err(SdfError::EmptyMesh);
    }
    validate_faces(mesh)?;
    let bary = barycentric_samples(opts.samples_per_triangle.max(1));
    let raw = sample_stats(mesh, &bary, |p| reference.eval(p));
    finalize(raw, true)
}

/// Symmetric sampled Hausdorff distance between two triangle meshes.
///
/// Samples each mesh's surface and measures the distance to the *other*
/// mesh (via a parry BVH), in both directions. `max_abs` is the symmetric
/// Hausdorff — the larger of the two one-sided maxima — so it catches both
/// geometry present in `a` but missing from `b` and vice versa. This is the
/// mesh→SDF→mesh round-trip metric (original vs re-meshed-from-its-SDF).
///
/// # Errors
///
/// - [`SdfError::EmptyMesh`] if either mesh has no faces.
/// - [`SdfError::FaceIndexOutOfRange`] if either mesh has a face referencing
///   a vertex index beyond its vertex list.
/// - [`SdfError::NonFiniteDeviation`] if any sampled distance is non-finite.
pub fn hausdorff_distance(
    a: &IndexedMesh,
    b: &IndexedMesh,
    opts: SampleOptions,
) -> SdfResult<DeviationReport> {
    // Validate indices before building the BVHs — an out-of-range index
    // would otherwise reach parry as an unchecked panic.
    validate_faces(a)?;
    validate_faces(b)?;
    // `new` returns `EmptyMesh` for a faceless mesh, so `?` covers the
    // empty-input guard for both directions.
    let dist_b = TriMeshDistance::new(b.clone())?;
    let dist_a = TriMeshDistance::new(a.clone())?;
    let bary = barycentric_samples(opts.samples_per_triangle.max(1));
    let ab = sample_stats(a, &bary, |p| dist_b.distance(p));
    let ba = sample_stats(b, &bary, |p| dist_a.distance(p));
    finalize(combine(ab, ba), false)
}

// ── internals ───────────────────────────────────────────────────────────

/// Confirm every face references an in-range vertex, so `sample_stats` can
/// index the vertex list without panicking.
fn validate_faces(mesh: &IndexedMesh) -> SdfResult<()> {
    let vertex_count = mesh.vertices.len();
    for &face in &mesh.faces {
        for index in face {
            if index as usize >= vertex_count {
                return Err(SdfError::FaceIndexOutOfRange {
                    index,
                    vertex_count,
                });
            }
        }
    }
    Ok(())
}

/// Running accumulator over surface samples. Signed extremes are tracked
/// even for the unsigned metric (they are simply discarded at finalize).
/// Unweighted sums back the degenerate-mesh (zero total area) fallback so
/// the report never claims zero deviation on a nonzero-deviation mesh.
struct RawStats {
    max_abs: f64,
    sum_w_abs: f64,
    sum_w_sq: f64,
    sum_w: f64,
    sum_abs: f64,
    sum_sq: f64,
    min_signed: f64,
    max_signed: f64,
    n: usize,
    non_finite: bool,
}

/// Barycentric weights for one triangle: the centroid followed by `n`
/// Halton (base 2, 3) interior points. Deterministic — no RNG.
fn barycentric_samples(n: usize) -> Vec<[f64; 3]> {
    let mut out = Vec::with_capacity(n + 1);
    out.push([1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0]);
    for i in 1..=n {
        let u = halton(i, 2);
        let v = halton(i, 3);
        // sqrt map: uniform over the triangle for uniform (u, v) in [0,1)².
        let su = u.sqrt();
        out.push([1.0 - su, su * (1.0 - v), su * v]);
    }
    out
}

/// Radical-inverse Halton sample of `index` in the given `base`.
fn halton(mut index: usize, base: usize) -> f64 {
    let mut f = 1.0;
    let mut r = 0.0;
    while index > 0 {
        f /= base as f64;
        r += f * (index % base) as f64;
        index /= base;
    }
    r
}

/// Walk every triangle's samples, evaluate `f` (signed deviation) at each,
/// and accumulate area-weighted statistics.
fn sample_stats(
    mesh: &IndexedMesh,
    bary: &[[f64; 3]],
    mut f: impl FnMut(Point3<f64>) -> f64,
) -> RawStats {
    let mut s = RawStats {
        max_abs: 0.0,
        sum_w_abs: 0.0,
        sum_w_sq: 0.0,
        sum_w: 0.0,
        sum_abs: 0.0,
        sum_sq: 0.0,
        min_signed: f64::INFINITY,
        max_signed: f64::NEG_INFINITY,
        n: 0,
        non_finite: false,
    };
    for &face in &mesh.faces {
        let a = mesh.vertices[face[0] as usize];
        let b = mesh.vertices[face[1] as usize];
        let c = mesh.vertices[face[2] as usize];
        let area = 0.5 * (b - a).cross(&(c - a)).norm();
        let w = area / bary.len() as f64;
        for &[wa, wb, wc] in bary {
            let p = Point3::from(a.coords * wa + b.coords * wb + c.coords * wc);
            let d = f(p);
            s.n += 1;
            // A non-finite sample means corrupt input; flag it so the
            // public entry point can surface an error instead of letting
            // NaN's always-false comparisons leave a misleading zero max.
            if !d.is_finite() {
                s.non_finite = true;
                continue;
            }
            let ad = d.abs();
            if ad > s.max_abs {
                s.max_abs = ad;
            }
            s.sum_w_abs += w * ad;
            s.sum_w_sq += w * d * d;
            s.sum_w += w;
            s.sum_abs += ad;
            s.sum_sq += d * d;
            if d < s.min_signed {
                s.min_signed = d;
            }
            if d > s.max_signed {
                s.max_signed = d;
            }
        }
    }
    s
}

/// Merge two one-sided passes into a symmetric accumulator.
fn combine(a: RawStats, b: RawStats) -> RawStats {
    RawStats {
        max_abs: a.max_abs.max(b.max_abs),
        sum_w_abs: a.sum_w_abs + b.sum_w_abs,
        sum_w_sq: a.sum_w_sq + b.sum_w_sq,
        sum_w: a.sum_w + b.sum_w,
        sum_abs: a.sum_abs + b.sum_abs,
        sum_sq: a.sum_sq + b.sum_sq,
        min_signed: a.min_signed.min(b.min_signed),
        max_signed: a.max_signed.max(b.max_signed),
        n: a.n + b.n,
        non_finite: a.non_finite || b.non_finite,
    }
}

/// Convert raw accumulated sums into a report. `signed` decides whether the
/// inside/outside extremes are carried (true) or discarded (false).
///
/// # Errors
///
/// Returns [`SdfError::NonFiniteDeviation`] if any sample was non-finite, or
/// if the aggregates themselves come out non-finite (e.g. an area weight
/// overflowed to infinity on a mesh with a huge-but-finite vertex).
fn finalize(raw: RawStats, signed: bool) -> SdfResult<DeviationReport> {
    if raw.non_finite {
        return Err(SdfError::NonFiniteDeviation);
    }
    let (mean_abs, rms) = if raw.sum_w > 0.0 {
        (raw.sum_w_abs / raw.sum_w, (raw.sum_w_sq / raw.sum_w).sqrt())
    } else if raw.n > 0 {
        // Degenerate mesh (zero total surface area): area weights vanish, so
        // fall back to an unweighted sample mean. This still reports the
        // real deviation magnitude rather than a misleading zero.
        (
            raw.sum_abs / raw.n as f64,
            (raw.sum_sq / raw.n as f64).sqrt(),
        )
    } else {
        (0.0, 0.0)
    };
    // The per-sample guard above only sees `d`; an area weight or a running
    // sum can still overflow to infinity (→ inf/inf = NaN aggregates) while
    // every `d` is finite. Surface that rather than return a NaN report.
    if !mean_abs.is_finite() || !rms.is_finite() || !raw.max_abs.is_finite() {
        return Err(SdfError::NonFiniteDeviation);
    }
    Ok(DeviationReport {
        max_abs: raw.max_abs,
        mean_abs,
        rms,
        signed_extremes: signed.then_some(SignedExtremes {
            min: raw.min_signed,
            max: raw.max_signed,
        }),
        n_samples: raw.n,
    })
}

#[cfg(test)]
#[allow(
    clippy::unwrap_used,
    clippy::float_cmp,
    clippy::cast_precision_loss,
    clippy::similar_names
)]
mod tests {
    use super::*;
    use mesh_types::Vector3;

    // ── analytic reference SDFs (closed-form ground truth) ──────────────

    /// Exact sphere SDF: `‖p − c‖ − r`.
    struct AnalyticSphere {
        center: Point3<f64>,
        radius: f64,
    }

    impl Sdf for AnalyticSphere {
        fn eval(&self, p: Point3<f64>) -> f64 {
            (p - self.center).norm() - self.radius
        }
        fn grad(&self, p: Point3<f64>) -> Vector3<f64> {
            let d = p - self.center;
            let n = d.norm();
            if n > 1e-12 { d / n } else { Vector3::z() }
        }
    }

    /// Exact torus SDF in the XY plane (tube axis = +Z), major radius `R`,
    /// minor radius `r`.
    struct AnalyticTorus {
        major: f64,
        minor: f64,
    }

    impl Sdf for AnalyticTorus {
        fn eval(&self, p: Point3<f64>) -> f64 {
            let radial = p.x.hypot(p.y) - self.major;
            radial.hypot(p.z) - self.minor
        }
        fn grad(&self, p: Point3<f64>) -> Vector3<f64> {
            // Not exercised by the metrics; a finite-difference gradient is
            // sufficient for the trait contract.
            let e = 1e-6;
            let dx = self.eval(Point3::new(p.x + e, p.y, p.z))
                - self.eval(Point3::new(p.x - e, p.y, p.z));
            let dy = self.eval(Point3::new(p.x, p.y + e, p.z))
                - self.eval(Point3::new(p.x, p.y - e, p.z));
            let dz = self.eval(Point3::new(p.x, p.y, p.z + e))
                - self.eval(Point3::new(p.x, p.y, p.z - e));
            Vector3::new(dx, dy, dz) / (2.0 * e)
        }
    }

    // ── test meshes with vertices exactly on an analytic surface ────────

    fn project_to_sphere(p: Point3<f64>, radius: f64) -> Point3<f64> {
        Point3::from(p.coords.normalize() * radius)
    }

    /// Icosphere: icosahedron subdivided `subdiv` times, every vertex
    /// projected onto the sphere of the given `radius`. Triangle *interiors*
    /// are chords sitting just inside the sphere, so the deviation is a pure
    /// meshing artifact that shrinks with subdivision.
    fn icosphere(radius: f64, subdiv: usize) -> IndexedMesh {
        let t = (1.0 + 5.0_f64.sqrt()) / 2.0;
        let raw = [
            [-1.0, t, 0.0],
            [1.0, t, 0.0],
            [-1.0, -t, 0.0],
            [1.0, -t, 0.0],
            [0.0, -1.0, t],
            [0.0, 1.0, t],
            [0.0, -1.0, -t],
            [0.0, 1.0, -t],
            [t, 0.0, -1.0],
            [t, 0.0, 1.0],
            [-t, 0.0, -1.0],
            [-t, 0.0, 1.0],
        ];
        let v: Vec<Point3<f64>> = raw
            .iter()
            .map(|c| project_to_sphere(Point3::new(c[0], c[1], c[2]), radius))
            .collect();
        let faces = [
            [0, 11, 5],
            [0, 5, 1],
            [0, 1, 7],
            [0, 7, 10],
            [0, 10, 11],
            [1, 5, 9],
            [5, 11, 4],
            [11, 10, 2],
            [10, 7, 6],
            [7, 1, 8],
            [3, 9, 4],
            [3, 4, 2],
            [3, 2, 6],
            [3, 6, 8],
            [3, 8, 9],
            [4, 9, 5],
            [2, 4, 11],
            [6, 2, 10],
            [8, 6, 7],
            [9, 8, 1],
        ];
        let mut tris: Vec<[Point3<f64>; 3]> =
            faces.iter().map(|f| [v[f[0]], v[f[1]], v[f[2]]]).collect();

        for _ in 0..subdiv {
            let mut next = Vec::with_capacity(tris.len() * 4);
            for [a, b, c] in tris {
                let ab = project_to_sphere(Point3::from((a.coords + b.coords) * 0.5), radius);
                let bc = project_to_sphere(Point3::from((b.coords + c.coords) * 0.5), radius);
                let ca = project_to_sphere(Point3::from((c.coords + a.coords) * 0.5), radius);
                next.push([a, ab, ca]);
                next.push([ab, b, bc]);
                next.push([ca, bc, c]);
                next.push([ab, bc, ca]);
            }
            tris = next;
        }
        build_from_tris(&tris)
    }

    /// Parametric torus grid, every vertex exactly on the analytic torus.
    fn torus_mesh(major: f64, minor: f64, nu: usize, nv: usize) -> IndexedMesh {
        let point = |i: usize, j: usize| {
            let u = std::f64::consts::TAU * (i as f64) / (nu as f64);
            let vv = std::f64::consts::TAU * (j as f64) / (nv as f64);
            let ring = major + minor * vv.cos();
            Point3::new(ring * u.cos(), ring * u.sin(), minor * vv.sin())
        };
        let mut tris: Vec<[Point3<f64>; 3]> = Vec::with_capacity(nu * nv * 2);
        for i in 0..nu {
            for j in 0..nv {
                let p00 = point(i, j);
                let p10 = point((i + 1) % nu, j);
                let p01 = point(i, (j + 1) % nv);
                let p11 = point((i + 1) % nu, (j + 1) % nv);
                tris.push([p00, p10, p11]);
                tris.push([p00, p11, p01]);
            }
        }
        build_from_tris(&tris)
    }

    /// Build an `IndexedMesh` from independent triangles (no vertex sharing
    /// — fine for the metrics, which never traverse topology).
    fn build_from_tris(tris: &[[Point3<f64>; 3]]) -> IndexedMesh {
        let mut mesh = IndexedMesh::new();
        for [a, b, c] in tris {
            let i = mesh.vertices.len() as u32;
            mesh.vertices.push(*a);
            mesh.vertices.push(*b);
            mesh.vertices.push(*c);
            mesh.faces.push([i, i + 1, i + 2]);
        }
        mesh
    }

    fn translate(mesh: &IndexedMesh, delta: Vector3<f64>) -> IndexedMesh {
        let mut out = mesh.clone();
        for v in &mut out.vertices {
            *v += delta;
        }
        out
    }

    // ── Hausdorff: identity and known shift ─────────────────────────────

    #[test]
    fn hausdorff_of_a_mesh_with_itself_is_zero() {
        let m = icosphere(1.0, 1);
        let r = hausdorff_distance(&m, &m, SampleOptions::default()).unwrap();
        // Bound is parry's BVH closest-point residual (~1e-7 at O(1)
        // coordinates), not a geometric deviation — a real mismatch would
        // be orders of magnitude larger.
        assert!(
            r.max_abs < 1e-6,
            "self-Hausdorff should be ~0, got {}",
            r.max_abs
        );
        assert_eq!(r.signed_extremes, None);
    }

    #[test]
    fn hausdorff_recovers_a_known_translation() {
        let m = icosphere(1.0, 3);
        let delta = 0.05;
        let shifted = translate(&m, Vector3::new(delta, 0.0, 0.0));
        let r = hausdorff_distance(&m, &shifted, SampleOptions::default()).unwrap();
        // For a rigid shift of a closed surface the symmetric Hausdorff is
        // the shift magnitude (attained at the leading/trailing poles);
        // sampling makes it a slight underestimate.
        assert!(
            (r.max_abs - delta).abs() < 4e-3,
            "translation Hausdorff should recover {delta}, got {}",
            r.max_abs
        );
        assert!(r.mean_abs > 0.0 && r.mean_abs < delta);
    }

    // ── surface-to-SDF: sign, convergence, orientation ──────────────────

    #[test]
    fn inscribed_icosphere_deviates_inward_only() {
        let mesh = icosphere(1.0, 2);
        let sphere = AnalyticSphere {
            center: Point3::origin(),
            radius: 1.0,
        };
        let r = surface_deviation_to_sdf(&mesh, &sphere, SampleOptions::default()).unwrap();
        let ext = r.signed_extremes.unwrap();
        assert!(ext.min < 0.0, "chords must fall inside the sphere");
        assert!(
            ext.max < 1e-9,
            "no inscribed sample should lie outside the sphere, got {}",
            ext.max
        );
    }

    #[test]
    fn sphere_deviation_converges_quadratically_with_subdivision() {
        let sphere = AnalyticSphere {
            center: Point3::origin(),
            radius: 1.0,
        };
        let dev = |s| {
            surface_deviation_to_sdf(&icosphere(1.0, s), &sphere, SampleOptions::default())
                .unwrap()
                .max_abs
        };
        let (d1, d2, d3) = (dev(1), dev(2), dev(3));
        // Strictly decreasing, ~4× per level (halving edge length on a
        // curved surface quarters the chord sagitta). The ratio is bounded on
        // BOTH sides so this pins quadratic-ish convergence — a bug that
        // over-shrinks `max_abs` toward zero would blow the upper bound.
        assert!(
            d1 > d2 && d2 > d3,
            "deviation must shrink: {d1} > {d2} > {d3}"
        );
        let ratio = d2 / d3;
        assert!(
            (2.5..6.0).contains(&ratio),
            "expected ~4× per-level convergence, got {ratio}"
        );
    }

    #[test]
    fn torus_deviation_converges_and_guards_axis_convention() {
        let torus = AnalyticTorus {
            major: 1.0,
            minor: 0.3,
        };
        let coarse = surface_deviation_to_sdf(
            &torus_mesh(1.0, 0.3, 12, 8),
            &torus,
            SampleOptions::default(),
        )
        .unwrap();
        let fine = surface_deviation_to_sdf(
            &torus_mesh(1.0, 0.3, 48, 32),
            &torus,
            SampleOptions::default(),
        )
        .unwrap();
        // A wrong axis convention in the analytic torus would leave the mesh
        // far off the field at every resolution; instead deviation is small
        // and shrinks with refinement.
        assert!(
            coarse.max_abs < 0.3,
            "coarse torus deviation should be modest"
        );
        assert!(
            fine.max_abs < coarse.max_abs,
            "torus deviation must shrink with resolution"
        );
        assert!(
            fine.signed_extremes.unwrap().min < 0.0,
            "chords must fall inside the tube"
        );
    }

    // ── report shape + guards ───────────────────────────────────────────

    #[test]
    fn report_orders_max_mean_rms_and_counts_samples() {
        let mesh = icosphere(1.0, 1); // 80 faces
        let sphere = AnalyticSphere {
            center: Point3::origin(),
            radius: 1.0,
        };
        let opts = SampleOptions {
            samples_per_triangle: 8,
        };
        let r = surface_deviation_to_sdf(&mesh, &sphere, opts).unwrap();
        // Both orderings hold up to a rounding ulp (a division may round the
        // quotient just past the stored extremum).
        assert!(
            r.mean_abs <= r.max_abs + 1e-15,
            "mean must be <= max of |d|"
        );
        assert!(r.rms + 1e-15 >= r.mean_abs, "rms must be >= mean of |d|");
        assert_eq!(r.n_samples, 80 * (8 + 1)); // centroid + 8 per face
    }

    #[test]
    fn samples_per_triangle_is_clamped_to_at_least_one() {
        let mesh = icosphere(1.0, 0); // 20 faces
        let sphere = AnalyticSphere {
            center: Point3::origin(),
            radius: 1.0,
        };
        let r = surface_deviation_to_sdf(
            &mesh,
            &sphere,
            SampleOptions {
                samples_per_triangle: 0,
            },
        )
        .unwrap();
        assert_eq!(r.n_samples, 20 * (1 + 1)); // centroid + clamped-to-1
    }

    #[test]
    fn empty_mesh_is_rejected() {
        let empty = IndexedMesh::new();
        let sphere = AnalyticSphere {
            center: Point3::origin(),
            radius: 1.0,
        };
        assert!(matches!(
            surface_deviation_to_sdf(&empty, &sphere, SampleOptions::default()),
            Err(SdfError::EmptyMesh)
        ));
        let m = icosphere(1.0, 0);
        assert!(matches!(
            hausdorff_distance(&empty, &m, SampleOptions::default()),
            Err(SdfError::EmptyMesh)
        ));
        assert!(matches!(
            hausdorff_distance(&m, &empty, SampleOptions::default()),
            Err(SdfError::EmptyMesh)
        ));
    }

    /// Reference SDF that always returns NaN — stands in for corrupt input.
    struct NanSdf;
    impl Sdf for NanSdf {
        fn eval(&self, _p: Point3<f64>) -> f64 {
            f64::NAN
        }
        fn grad(&self, _p: Point3<f64>) -> Vector3<f64> {
            Vector3::zeros()
        }
    }

    #[test]
    fn non_finite_samples_are_surfaced_not_reported_as_zero() {
        // A NaN reference field must error, not silently report max_abs = 0
        // (the always-false NaN comparisons would otherwise hide it).
        let mesh = icosphere(1.0, 1);
        assert!(matches!(
            surface_deviation_to_sdf(&mesh, &NanSdf, SampleOptions::default()),
            Err(SdfError::NonFiniteDeviation)
        ));
    }

    #[test]
    fn degenerate_mesh_reports_real_deviation_not_zero() {
        // Three collinear vertices → zero-area faces → total area 0. The
        // unweighted fallback must still report the true nonzero deviation
        // instead of a misleading mean_abs = 0.
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Point3::new(2.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(3.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(4.0, 0.0, 0.0));
        mesh.faces.push([0, 1, 2]);
        let sphere = AnalyticSphere {
            center: Point3::origin(),
            radius: 1.0,
        };
        let r = surface_deviation_to_sdf(&mesh, &sphere, SampleOptions::default()).unwrap();
        // Samples sit ~1–3 m outside a unit sphere → deviation is ~1–3, and
        // must not collapse to zero just because the weights vanished.
        assert!(
            r.max_abs > 1.0,
            "max deviation should be large, got {}",
            r.max_abs
        );
        assert!(
            r.mean_abs > 0.5,
            "degenerate-mesh mean must reflect real deviation, got {}",
            r.mean_abs
        );
    }

    #[test]
    fn out_of_range_face_index_errors_rather_than_panicking() {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Point3::origin());
        mesh.vertices.push(Point3::new(1.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(0.0, 1.0, 0.0));
        mesh.faces.push([0, 1, 5]); // vertex 5 does not exist
        let sphere = AnalyticSphere {
            center: Point3::origin(),
            radius: 1.0,
        };
        assert!(matches!(
            surface_deviation_to_sdf(&mesh, &sphere, SampleOptions::default()),
            Err(SdfError::FaceIndexOutOfRange {
                index: 5,
                vertex_count: 3
            })
        ));
        // The Hausdorff path validates before building the parry BVH, so it
        // also errors cleanly instead of panicking inside parry.
        let good = icosphere(1.0, 0);
        assert!(matches!(
            hausdorff_distance(&mesh, &good, SampleOptions::default()),
            Err(SdfError::FaceIndexOutOfRange { .. })
        ));
    }

    #[test]
    fn overflowing_area_weight_errors_instead_of_a_nan_report() {
        // A finite-but-enormous vertex makes the triangle area (a cross
        // product) overflow to +inf, so the area weight and its running sums
        // go infinite while every per-sample `d` stays finite. The final
        // finiteness guard must surface this rather than return a NaN mean.
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Point3::origin());
        mesh.vertices.push(Point3::new(1e200, 0.0, 0.0));
        mesh.vertices.push(Point3::new(0.0, 1e200, 0.0));
        mesh.faces.push([0, 1, 2]);
        let sphere = AnalyticSphere {
            center: Point3::origin(),
            radius: 1.0,
        };
        assert!(matches!(
            surface_deviation_to_sdf(&mesh, &sphere, SampleOptions::default()),
            Err(SdfError::NonFiniteDeviation)
        ));
    }

    #[test]
    fn hausdorff_is_symmetric_and_driven_by_the_larger_one_sided_distance() {
        // A = unit sphere. B = A plus one far-away "spike" triangle near
        // x = 10. Then h(A->B) ~ 0 (every A point lies on B too), but
        // h(B->A) ~ 9 (the spike is ~9 m from the sphere). The reported
        // symmetric Hausdorff must be the LARGER of the two — a one-sided
        // `a.max_abs`, or a `.min()`, would report ~0 and fail here.
        let a = icosphere(1.0, 1);
        let mut b = a.clone();
        let base = b.vertices.len() as u32;
        b.vertices.push(Point3::new(10.0, 0.0, 0.0));
        b.vertices.push(Point3::new(10.1, 0.0, 0.0));
        b.vertices.push(Point3::new(10.0, 0.1, 0.0));
        b.faces.push([base, base + 1, base + 2]);

        let ab = hausdorff_distance(&a, &b, SampleOptions::default()).unwrap();
        assert!(
            ab.max_abs > 8.0,
            "symmetric Hausdorff must catch the far spike via the B->A \
             direction, got {}",
            ab.max_abs
        );
        // Swapping the arguments must give the same symmetric distance.
        let ba = hausdorff_distance(&b, &a, SampleOptions::default()).unwrap();
        assert!(
            (ab.max_abs - ba.max_abs).abs() < 1e-9,
            "Hausdorff must be symmetric: {} vs {}",
            ab.max_abs,
            ba.max_abs
        );
    }

    #[test]
    fn circumscribed_mesh_reports_positive_outside_deviation() {
        // A mesh sitting entirely OUTSIDE the reference sphere must produce
        // positive signed deviation — exercises the "outside = positive"
        // branch that inscribed meshes never reach.
        let mesh = icosphere(1.05, 2);
        let sphere = AnalyticSphere {
            center: Point3::origin(),
            radius: 1.0,
        };
        let r = surface_deviation_to_sdf(&mesh, &sphere, SampleOptions::default()).unwrap();
        let ext = r.signed_extremes.unwrap();
        assert!(
            ext.min > 0.0,
            "every sample should lie outside the unit sphere, got min {}",
            ext.min
        );
        assert!(
            ext.max > 0.03,
            "outside deviation should be ~0.05, got {}",
            ext.max
        );
    }

    #[test]
    fn mean_is_area_weighted_not_a_flat_sample_average() {
        // Two far-apart triangles of very unequal area (100x). The big
        // triangle sits at deviation ~10, the small one at ~2. Area-weighted,
        // the mean is dominated by the big triangle (~9.95); a flat
        // (unweighted) sample average would be ~6. Assert we get the former.
        let mut mesh = IndexedMesh::new();
        // Small triangle near x = 3 (|d| ~ 2), area 5e-5.
        mesh.vertices.push(Point3::new(3.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(3.01, 0.0, 0.0));
        mesh.vertices.push(Point3::new(3.0, 0.01, 0.0));
        mesh.faces.push([0, 1, 2]);
        // Large triangle near x = 11 (|d| ~ 10), area 5e-3 (100x larger).
        mesh.vertices.push(Point3::new(11.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(11.1, 0.0, 0.0));
        mesh.vertices.push(Point3::new(11.0, 0.1, 0.0));
        mesh.faces.push([3, 4, 5]);

        let sphere = AnalyticSphere {
            center: Point3::origin(),
            radius: 1.0,
        };
        let r = surface_deviation_to_sdf(&mesh, &sphere, SampleOptions::default()).unwrap();
        // ~9.95 if area-weighted; ~6.0 if a flat sample average. The `> 8`
        // bound cleanly separates the two.
        assert!(
            r.mean_abs > 8.0 && r.mean_abs < 10.1,
            "mean must be area-weighted (~9.95), got {}",
            r.mean_abs
        );
    }
}
