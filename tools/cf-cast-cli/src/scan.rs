//! Scan STL loading + a sharable [`cf_design::Sdf`] wrapper.
//!
//! Loading a multi-million-face scan once and cloning the
//! [`SignedDistanceField`] into N + 1 [`cf_design::Solid`] expression
//! trees (one per layer + plug) costs O(N × scan-size). The shared
//! `Arc<SignedDistanceField>` wrapper exposed here keeps the heavy
//! mesh + face-normal arrays in a single allocation that's cloned
//! cheaply by ref-count for each `cf_design::Solid::from_sdf` call.

use std::path::Path;
use std::sync::Arc;

use anyhow::{Context, Result};
use cf_geometry::{Aabb, Bounded};
use mesh_sdf::SignedDistanceField;
use nalgebra::{Point3, Vector3};

/// Reference-counted wrapper around [`SignedDistanceField`] that
/// satisfies [`cf_design::Sdf`].
///
/// Cloning is `Arc::clone` (one atomic increment), so passing this to
/// [`cf_design::Solid::from_sdf`] N + 1 times stores N + 1 lightweight
/// handles to a single underlying SDF — no deep mesh duplication.
///
/// `cf_design::Sdf` requires `Send + Sync`; `Arc<T>` is `Send + Sync`
/// whenever `T` is, and [`SignedDistanceField`] is `Send + Sync` (it
/// holds plain `Vec`s of `f64`/`u32`/`Vector3<f64>`), so the bound
/// trivially holds.
#[derive(Debug, Clone)]
pub struct SharedScanSdf(Arc<SignedDistanceField>);

impl SharedScanSdf {
    /// Wrap an existing [`SignedDistanceField`] in an [`Arc`].
    #[must_use]
    pub fn new(sdf: SignedDistanceField) -> Self {
        Self(Arc::new(sdf))
    }

    /// Clone the underlying `Arc<SignedDistanceField>` — used by the
    /// pinned-floor plumbing in [`crate::derive`] to share the
    /// closed-body SDF with the [`cf_cap_planes::DomeWallSignedSdf`]
    /// adapter (which holds `Arc<dyn Sdf>`).
    #[must_use]
    pub fn inner_arc(&self) -> Arc<SignedDistanceField> {
        Arc::clone(&self.0)
    }
}

impl cf_design::Sdf for SharedScanSdf {
    fn eval(&self, p: Point3<f64>) -> f64 {
        self.0.distance(p)
    }

    fn grad(&self, p: Point3<f64>) -> Vector3<f64> {
        // Central finite difference at 1e-6 — matches the bare
        // `mesh_sdf::SignedDistanceField` impl in cf-design's sdf.rs
        // (the canonical reference). Reproducing the math here
        // rather than forwarding to a `&SignedDistanceField` impl
        // keeps SharedScanSdf a self-contained `Sdf` impl.
        let eps = 1e-6;
        let inv_2eps = 0.5 / eps;
        Vector3::new(
            (self.0.distance(Point3::new(p.x + eps, p.y, p.z))
                - self.0.distance(Point3::new(p.x - eps, p.y, p.z)))
                * inv_2eps,
            (self.0.distance(Point3::new(p.x, p.y + eps, p.z))
                - self.0.distance(Point3::new(p.x, p.y - eps, p.z)))
                * inv_2eps,
            (self.0.distance(Point3::new(p.x, p.y, p.z + eps))
                - self.0.distance(Point3::new(p.x, p.y, p.z - eps)))
                * inv_2eps,
        )
    }
}

/// Load a cleaned-scan STL + precompute its [`SignedDistanceField`] +
/// snapshot the mesh AABB before the SDF takes ownership.
///
/// Returns the [`SharedScanSdf`] (wrapping the SDF in an `Arc`) plus
/// the scan AABB (in meters, the same frame the SDF queries) plus an
/// `Arc<IndexedMesh>` of the loaded cleaned scan — kept so the
/// pinned-floor plumbing in [`crate::derive`] can build a second
/// `SignedDistanceField` over the cap-polygon-stripped mesh via
/// `cf_cap_planes::dome_wall_only_mesh` without re-loading the STL.
///
/// The mesh clone is small relative to SDF construction
/// (~6 MB on iter-1's 167 k-face cleaned scan vs O(faces²) SDF build);
/// shared via `Arc` so it's cheap to hand to multiple consumers.
///
/// # Errors
///
/// - STL file not found / unreadable
/// - STL contents malformed
/// - empty mesh (cf-scan-prep should never emit this, but defensive)
pub fn load_scan_sdf(path: &Path) -> Result<LoadedScan> {
    let mesh = mesh_io::load_stl(path).with_context(|| format!("load_stl({})", path.display()))?;
    let aabb = mesh.aabb();
    let mesh_arc = Arc::new(mesh.clone());
    let sdf = SignedDistanceField::new(mesh).context("build SignedDistanceField from scan mesh")?;
    Ok(LoadedScan {
        sdf: SharedScanSdf::new(sdf),
        aabb,
        mesh: mesh_arc,
    })
}

/// Output of [`load_scan_sdf`].
#[derive(Debug, Clone)]
pub struct LoadedScan {
    /// Reference-counted SDF — cheap to clone into multiple
    /// `Solid::from_sdf` calls.
    pub sdf: SharedScanSdf,
    /// AABB of the cleaned scan mesh, in meters (same frame as the
    /// SDF queries).
    pub aabb: Aabb,
    /// Reference-counted cleaned scan mesh — used by the pinned-floor
    /// plumbing to derive the cap-polygon-stripped open-body SDF.
    /// Empty consumers can ignore this field.
    pub mesh: Arc<cf_geometry::IndexedMesh>,
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use super::*;
    use cf_design::Sdf;
    use cf_geometry::IndexedMesh;

    fn unit_cube_mesh() -> IndexedMesh {
        // 8 vertices of the [-0.5, 0.5]^3 cube, 12 triangles.
        let mut m = IndexedMesh::new();
        let coords = [
            (-0.5, -0.5, -0.5),
            (0.5, -0.5, -0.5),
            (0.5, 0.5, -0.5),
            (-0.5, 0.5, -0.5),
            (-0.5, -0.5, 0.5),
            (0.5, -0.5, 0.5),
            (0.5, 0.5, 0.5),
            (-0.5, 0.5, 0.5),
        ];
        for (x, y, z) in coords {
            m.vertices.push(Point3::new(x, y, z));
        }
        let faces = [
            // -z
            [0, 2, 1],
            [0, 3, 2],
            // +z
            [4, 5, 6],
            [4, 6, 7],
            // -y
            [0, 1, 5],
            [0, 5, 4],
            // +y
            [2, 3, 7],
            [2, 7, 6],
            // -x
            [0, 4, 7],
            [0, 7, 3],
            // +x
            [1, 2, 6],
            [1, 6, 5],
        ];
        for f in faces {
            m.faces.push(f);
        }
        m
    }

    #[test]
    fn shared_scan_sdf_eval_matches_underlying() {
        let mesh = unit_cube_mesh();
        let sdf = SignedDistanceField::new(mesh).unwrap();
        let p = Point3::new(0.0, 0.0, 0.0);
        let direct = sdf.distance(p);
        let shared = SharedScanSdf::new(sdf);
        assert!((shared.eval(p) - direct).abs() < 1e-12);
    }

    #[test]
    fn shared_scan_sdf_grad_points_outward_on_face() {
        let mesh = unit_cube_mesh();
        let sdf = SignedDistanceField::new(mesh).unwrap();
        let shared = SharedScanSdf::new(sdf);
        // Sample just outside the +x face. The mesh-sdf sign convention
        // is "negative inside, positive outside"; central differences
        // of distance point away from the surface, so at p just
        // outside +x face the gradient should have positive x component.
        let p = Point3::new(0.6, 0.0, 0.0);
        let g = shared.grad(p);
        assert!(
            g.x > 0.0,
            "grad.x at +x exterior probe must be positive: {g:?}"
        );
    }

    #[test]
    fn shared_scan_sdf_clone_shares_arc() {
        let mesh = unit_cube_mesh();
        let sdf = SignedDistanceField::new(mesh).unwrap();
        let a = SharedScanSdf::new(sdf);
        let b = a.clone();
        assert!(Arc::ptr_eq(&a.0, &b.0));
    }
}
