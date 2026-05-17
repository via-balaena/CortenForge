//! Scan STL loading + a sharable [`cf_design::Sdf`] wrapper.
//!
//! Loading a multi-million-face scan once and cloning the underlying
//! signed-distance field into N + 1 [`cf_design::Solid`] expression
//! trees (one per layer + plug) costs O(N × scan-size). The shared
//! `Arc`-backed wrapper exposed here keeps the parry BVH + flood-fill
//! grid in a single allocation that's cloned cheaply by ref-count for
//! each `cf_design::Solid::from_sdf` call.
//!
//! # Sign source
//!
//! Post-D arc ([[project-mesh-sdf-oracle-decomposition-spec]],
//! `docs/MESH_SDF_ORACLE_DECOMPOSITION_SPEC.md`) the scan SDF's sign
//! comes from [`mesh_sdf::FloodFillSign`] — topology-blind reachability
//! from the bounds corners. Pre-D.5 cf-cast-cli used parry's
//! pseudo-normal sign, which silently produced wrong-sign far-field on
//! iter-1 sock_over_capsule's cap-fan + dome-apex regions and propagated
//! through `body.subtract(rind)` into a non-watertight `plug_layer_0.stl`
//! (3156 open edges; see
//! [[project-cf-cast-plug-layer-0-watertight-discovery]]). Flood-fill
//! pays a one-shot grid build at load time + sign queries are O(1)
//! grid lookups — robust for any mesh whose interior is connected AND
//! fits inside the supplied bounds with the wall band below grid
//! resolution.

use std::path::Path;
use std::sync::Arc;

use anyhow::{Context, Result};
use cf_geometry::{Aabb, Bounded};
use mesh_sdf::{
    FloodFillSign, Signed, TriMeshDistance, WALL_THRESHOLD_FACTOR_DEFAULT, flood_filled_sdf,
};
use nalgebra::{Point3, Vector3};

/// Reference-counted wrapper around a flood-fill-signed scan SDF that
/// satisfies [`cf_design::Sdf`].
///
/// Cloning is `Arc::clone` (one atomic increment), so passing this to
/// [`cf_design::Solid::from_sdf`] N + 1 times stores N + 1 lightweight
/// handles to a single underlying SDF — no deep mesh duplication.
///
/// `cf_design::Sdf` requires `Send + Sync`; `Arc<T>` is `Send + Sync`
/// whenever `T` is, and the inner [`Signed`] holds plain `Vec`s + an
/// `Arc<parry3d::TriMesh>`, all `Send + Sync`.
#[derive(Debug, Clone)]
pub struct SharedScanSdf(Arc<Signed<TriMeshDistance, FloodFillSign>>);

impl SharedScanSdf {
    /// Wrap a pre-built `Signed<TriMeshDistance, FloodFillSign>` in an
    /// [`Arc`].
    ///
    /// Use [`load_scan_sdf`] for the common case of building from an
    /// STL on disk.
    #[must_use]
    pub fn new(sdf: Signed<TriMeshDistance, FloodFillSign>) -> Self {
        Self(Arc::new(sdf))
    }

    /// Borrow the underlying cleaned-scan mesh.
    ///
    /// `mesh_sdf::TriMeshDistance::mesh` returns the mesh the SDF was
    /// constructed over; cf-cap-planes' `dome_wall_only_mesh` needs it
    /// to strip cap-polygon faces for the candidate-A open SDF that
    /// `cf_design::pinned_floor_shell`'s unsigned-rind adapter
    /// consumes.
    #[must_use]
    pub fn mesh(&self) -> &cf_geometry::IndexedMesh {
        self.0.distance.mesh()
    }
}

impl cf_design::Sdf for SharedScanSdf {
    fn eval(&self, p: Point3<f64>) -> f64 {
        self.0.distance(p)
    }

    fn grad(&self, p: Point3<f64>) -> Vector3<f64> {
        // Central finite difference at 1e-6 — matches cf-design's
        // blanket `impl Sdf for Signed<D, S>` (the canonical
        // reference). Reproducing the math here rather than forwarding
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

/// Load a cleaned-scan STL, build a flood-fill-signed SDF over the
/// scan AABB padded by `bounds_padding_m`, and return the
/// [`SharedScanSdf`] + the original (un-padded) scan AABB.
///
/// `bounds_padding_m` is the per-axis margin the flood-fill grid is
/// padded by beyond the scan AABB; it MUST be ≥ the cumulative
/// outward thickness of the consumer's geometry (every layer outer
/// surface + cup-wall margin in cf-cast-cli's case) so flood-fill sign
/// queries inside the consumer's eval domain land within the grid
/// rather than falling through to the "outside" defensive default.
/// `cell_size` is the flood-fill grid resolution; pick the same value
/// the consumer uses for marching-cubes extraction so a wall-cell band
/// of `0.75 × cell_size` blocks the BFS at the same scale.
///
/// # Errors
///
/// - STL file not found / unreadable / malformed.
/// - Empty mesh (cf-scan-prep should never emit this, but defensive).
/// - Flood-fill build failure (degenerate bounds, non-positive
///   `cell_size`, or no positive-SDF corner seed — see
///   [`mesh_sdf::FloodFillError`]).
pub fn load_scan_sdf(path: &Path, bounds_padding_m: f64, cell_size: f64) -> Result<LoadedScan> {
    let mesh = mesh_io::load_stl(path).with_context(|| format!("load_stl({})", path.display()))?;
    let aabb = mesh.aabb();
    let bounds = aabb.expanded(bounds_padding_m);
    let (sdf, _report) = flood_filled_sdf(mesh, bounds, cell_size, WALL_THRESHOLD_FACTOR_DEFAULT)
        .with_context(|| {
        format!(
            "build flood-fill-signed SDF from scan mesh at {}",
            path.display()
        )
    })?;
    Ok(LoadedScan {
        sdf: SharedScanSdf::new(sdf),
        aabb,
    })
}

/// Output of [`load_scan_sdf`].
#[derive(Debug, Clone)]
pub struct LoadedScan {
    /// Reference-counted SDF — cheap to clone into multiple
    /// `Solid::from_sdf` calls.
    pub sdf: SharedScanSdf,
    /// AABB of the cleaned scan mesh, in meters (same frame as the
    /// SDF queries). This is the ORIGINAL scan AABB — not the padded
    /// flood-fill bounds — so consumers that need to compute their own
    /// eval-domain bounds (e.g. [`crate::derive_spec_and_ribbon`]) can
    /// reuse the same baseline that fed the flood-fill build.
    pub aabb: Aabb,
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

    fn unit_cube_signed() -> Signed<TriMeshDistance, FloodFillSign> {
        let mesh = unit_cube_mesh();
        // 0.0173 m cell — prime-ish; doesn't divide the cube's 0.5 m
        // half-extent. An aligned cell_size puts a lattice node EXACTLY
        // on the cube surface, tags it Wall, and lets the BFS label
        // expansion pick an arbitrary sign for that node — which then
        // propagates to probes that round to it.
        let bounds = Aabb::new(Point3::new(-1.0, -1.0, -1.0), Point3::new(1.0, 1.0, 1.0));
        let (signed, _report) =
            flood_filled_sdf(mesh, bounds, 0.0173, WALL_THRESHOLD_FACTOR_DEFAULT).unwrap();
        signed
    }

    #[test]
    fn shared_scan_sdf_eval_matches_underlying() {
        let signed = unit_cube_signed();
        let p = Point3::new(0.0, 0.0, 0.0);
        let direct = signed.distance(p);
        let shared = SharedScanSdf::new(signed);
        assert!((shared.eval(p) - direct).abs() < 1e-12);
    }

    #[test]
    fn shared_scan_sdf_grad_points_outward_on_face() {
        let shared = SharedScanSdf::new(unit_cube_signed());
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
        let a = SharedScanSdf::new(unit_cube_signed());
        let b = a.clone();
        assert!(Arc::ptr_eq(&a.0, &b.0));
    }
}
