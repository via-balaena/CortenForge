//! Rigid placement: PCA orient, recenter, rescale, pivot bakes.
//!
//! Moved verbatim from the flat `lib.rs` (pure module split).

use mesh_types::{Aabb, Bounded, IndexedMesh, Point3};
use nalgebra::{Matrix3, UnitQuaternion, Vector3};

/// Compute the **post-Reorient** AABB of the raw scan AABB in
/// physics-frame **millimeters**. Used by the Recenter panel's
/// `[Center origin]` and `[Floor -> z=0]` click handlers to figure out
/// where the rotated mesh actually sits in space.
///
/// Walks the 8 corners of the raw AABB, rotates each by `rot_physics`
/// (no translation applied — translation is what we're trying to
/// compute), and accumulates the rotated min/max. Returns mm bounds
/// because the Recenter sliders + status messages all work in mm
/// (workshop convention; matches the Scan Info panel's AABB display
/// units).
pub fn rotated_aabb_around_centroid_physics_mm(
    rot_physics: UnitQuaternion<f64>,
    raw_aabb_m: &Aabb,
) -> (Vector3<f64>, Vector3<f64>) {
    // Pivot rotation around the raw AABB centroid: corner_centered =
    // corner - centroid, rotated_centered = R * corner_centered,
    // world = rotated_centered + centroid. This keeps the mesh
    // ROTATING IN PLACE — the AABB CENTER stays at `centroid * 1000`
    // mm regardless of `rot_physics`; only the AABB extents rotate.
    // Without this pivot the [Center origin] / [Floor -> z=0] click
    // handlers would compute the FULL swing of the off-origin mesh
    // through space, which the user perceives as "rotation moves the
    // model" rather than "rotation rotates the model in place".
    let centroid = raw_aabb_m.center();
    let lo = raw_aabb_m.min;
    let hi = raw_aabb_m.max;
    let mut min_mm = Vector3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut max_mm = Vector3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    for &x in &[lo.x, hi.x] {
        for &y in &[lo.y, hi.y] {
            for &z in &[lo.z, hi.z] {
                let centered_mm = Vector3::new(
                    (x - centroid.x) * 1000.0,
                    (y - centroid.y) * 1000.0,
                    (z - centroid.z) * 1000.0,
                );
                let rotated_centered = rot_physics.transform_vector(&centered_mm);
                let world_mm = Vector3::new(
                    rotated_centered.x + centroid.x * 1000.0,
                    rotated_centered.y + centroid.y * 1000.0,
                    rotated_centered.z + centroid.z * 1000.0,
                );
                min_mm.x = min_mm.x.min(world_mm.x);
                min_mm.y = min_mm.y.min(world_mm.y);
                min_mm.z = min_mm.z.min(world_mm.z);
                max_mm.x = max_mm.x.max(world_mm.x);
                max_mm.y = max_mm.y.max(world_mm.y);
                max_mm.z = max_mm.z.max(world_mm.z);
            }
        }
    }
    (min_mm, max_mm)
}

/// Bake a physics-frame point through Reorient + Recenter with the
/// rotation pivoted at `centroid` (the scan's raw AABB centroid).
/// Returns the world-frame point:
///
/// ```text
/// world = rotation * (point - centroid) + centroid + translation_m
/// ```
///
/// The centroid pivot is what makes rotation feel like "rotate in
/// place" — without it, rotating an off-origin mesh swings the
/// centroid through space because the bake formula pivots at the
/// physics-frame origin by default. The pivot compensation
/// `(I - R) * centroid` is folded into the bake formula uniformly
/// so the cleaned STL on disk, the viewport mesh + wireframe, and
/// the cap / centerline overlays all agree.
pub fn bake_vertex_with_pivot(
    point: &Point3<f64>,
    rotation: UnitQuaternion<f64>,
    centroid: &Point3<f64>,
    translation_m: Vector3<f64>,
) -> Point3<f64> {
    let centered = point.coords - centroid.coords;
    let rotated_centered = rotation.transform_vector(&centered);
    Point3::from(rotated_centered + centroid.coords + translation_m)
}

/// Compute the principal-axis rotation that aligns a scan's
/// long axis with `+Z` (the cast-frame demolding axis).
///
/// PCA on the mean-centered vertex positions: 3×3 covariance,
/// symmetric eigendecomposition, principal eigenvector = the
/// largest-variance direction (the "long axis" of the scan). The
/// returned [`UnitQuaternion`] is the shortest rotation that maps
/// that principal axis to `+Z`.
///
/// Sign convention: the principal eigenvector is determined only
/// up to sign. This helper picks the side whose `z` component is
/// non-negative (i.e., the side already closer to `+Z`) — the
/// resulting rotation is therefore at most 90° from identity.
/// Users who want the opposite end pointed up flip post-hoc with
/// a 180° follow-on rotation (e.g. by setting roll = 180°).
///
/// Returns `None` for:
/// - fewer than 3 vertices (PCA underdetermined)
/// - degenerate mesh whose covariance has all near-zero
///   eigenvalues (all vertices coincident; no principal axis)
///
/// **Resolves cf-scan-prep deferred item §5 "Auto-PCA initial
/// orientation guess"** from `docs/SCAN_PREP_DESIGN.md` — the
/// manual-slider Reorient panel introduces tilt that propagates
/// downstream to the cleaned STL + centerline + (via cf-cast-cli)
/// the mold geometry. Auto-PCA gives a deterministic starting
/// orientation that the user can then nudge with the existing
/// sliders.
pub fn compute_pca_orientation(vertices: &[Point3<f64>]) -> Option<UnitQuaternion<f64>> {
    if vertices.len() < 3 {
        return None;
    }

    // Mean-center.
    let n = vertices.len() as f64;
    let mut centroid = Vector3::zeros();
    for v in vertices {
        centroid += v.coords;
    }
    centroid /= n;

    // 3×3 covariance accumulation (Σ d · d^T / n). Symmetric by
    // construction, so we use SymmetricEigen for the
    // eigendecomposition below.
    let mut cov = Matrix3::zeros();
    for v in vertices {
        let d = v.coords - centroid;
        cov += d * d.transpose();
    }
    cov /= n;

    let eigen = cov.symmetric_eigen();

    // Largest eigenvalue's column = principal axis.
    let (mut max_i, mut max_val) = (0_usize, eigen.eigenvalues[0]);
    for i in 1..3 {
        if eigen.eigenvalues[i] > max_val {
            max_i = i;
            max_val = eigen.eigenvalues[i];
        }
    }
    // Degeneracy guard: all-zero (or all near-zero) eigenvalues =
    // no meaningful principal direction (e.g. all vertices
    // coincident). Threshold relative to the largest eigenvalue
    // scale; absolute 1e-30 m² catches the all-zeros case
    // independently.
    if max_val <= 1e-30 {
        return None;
    }
    let mut principal: Vector3<f64> = eigen.eigenvectors.column(max_i).into_owned();

    // Sign pick: orient the principal axis toward `+Z` so the
    // resulting rotation is the SHORTEST rotation (≤ 90°).
    if principal.z < 0.0 {
        principal = -principal;
    }

    // `rotation_between` returns `None` for anti-parallel inputs;
    // the sign-pick above ensures `principal.z >= 0`, so the
    // anti-parallel case (principal == -Z) is precluded. Identity
    // case (principal == +Z, already aligned) yields the identity
    // quaternion.
    UnitQuaternion::rotation_between(&principal, &Vector3::z())
}

/// Apply the PCA-derived rotation to all vertices in `mesh`, taking
/// the principal axis to `+Z`. Returns the quaternion that was
/// applied (or `None` if PCA was degenerate — coincident vertices,
/// < 3 verts, or all-zero covariance).
///
/// Operates on the mesh after auto-center, so the rotation pivot is
/// the origin (= scan centroid). The result has its principal axis
/// aligned with cast-frame `+Z`.
///
/// Skips when the resulting quaternion is near-identity (within 1
/// µrad sin(θ/2) of identity) — the rotation would be a no-op
/// modulo FP drift, and skipping the vertex walk keeps already-
/// upright fixtures bit-exact.
pub fn auto_pca_in_place(mesh: &mut IndexedMesh) -> Option<UnitQuaternion<f64>> {
    let q = compute_pca_orientation(&mesh.vertices)?;
    // Near-identity check: |q.i|² + |q.j|² + |q.k|² < 1e-12 means
    // the vector part is sub-µrad; rotation is effectively identity.
    let vec_sq = q.i * q.i + q.j * q.j + q.k * q.k;
    if vec_sq < 1e-12 {
        return Some(q);
    }
    for v in &mut mesh.vertices {
        *v = q.transform_point(v);
    }
    Some(q)
}

/// Translate `mesh`'s vertices so the AABB centroid lands at physics
/// origin. Returns the offset that was applied (`= -aabb.center()`
/// from the input mesh in meters).
///
/// No-op for an empty mesh (no vertices to walk). When `centroid`
/// is already at origin, returns near-zero offset and skips the
/// walk to avoid FP-drift on bit-exact-centered fixtures.
pub fn auto_center_in_place(mesh: &mut IndexedMesh) -> Vector3<f64> {
    if mesh.vertices.is_empty() {
        return Vector3::zeros();
    }
    let centroid = mesh.aabb().center();
    // FP-drift guard: if the centroid is already within 1 µm of the
    // origin, skip the walk — we'd be moving vertices by sub-FP-
    // precision amounts that meshopt couldn't see anyway.
    let centroid_v = centroid.coords;
    if centroid_v.norm() < 1e-6 {
        return Vector3::zeros();
    }
    let offset = -centroid_v;
    for v in &mut mesh.vertices {
        v.x += offset.x;
        v.y += offset.y;
        v.z += offset.z;
    }
    offset
}

/// Multiply each vertex of `mesh` by `factor` in place. No-op when
/// `factor` is exactly `1.0` (skips the f64 vertex walk for the
/// `--stl-units m` identity case).
pub fn scale_vertices_in_place(mesh: &mut IndexedMesh, factor: f64) {
    if factor == 1.0 {
        return;
    }
    for v in &mut mesh.vertices {
        v.x *= factor;
        v.y *= factor;
        v.z *= factor;
    }
}
