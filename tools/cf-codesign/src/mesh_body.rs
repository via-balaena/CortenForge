//! Mesh-backed bodies for the geometry co-design targets.
//!
//! [`RouteTarget`](crate::RouteTarget) and [`ConduitTarget`](crate::ConduitTarget)
//! optimize a route against any body that satisfies [`cf_design::Sdf`]. An analytic
//! [`Solid`] is one such body; this module builds another — a **real triangle mesh**,
//! so the optimizer can route a conduit around scanned or imported geometry (an
//! anatomical vertebra, a printed part) exactly as it routes around a primitive.
//!
//! # Why a grid-cached field
//! The body is queried hundreds of thousands of times per optimization (Adam
//! iterations × finite-difference stencils × route samples), and the gradient is
//! taken by finite differences, so the field must be both **cheap** and **smooth**.
//! Both matter enough to pick the representation deliberately:
//!
//! - The **direct** parry-BVH distance ([`mesh_sdf::TriMeshDistance`] composed into a
//!   [`mesh_sdf::Signed`]) is exact but `O(log faces)` per query, and its gradient
//!   kinks at every facet ridge — everywhere on a faceted surface, which is exactly
//!   where the clearance penalty is active.
//! - The **grid-cached** [`mesh_sdf::CachedGridSdf`] samples that distance onto a
//!   lattice once and answers queries by trilinear interpolation: `O(1)`, and the
//!   interpolation low-pass-filters the sub-cell facet ridges, leaving only gentle
//!   gradient seams on the coarse lattice. Its sign comes from a topology-blind
//!   flood fill rather than a pseudo-normal test, which is why it is the preferred
//!   sign for a cleaned anatomical scan — the flood fill does not depend on
//!   consistent winding, so it is *designed* to tolerate the decimated, winding-
//!   flipped meshes such scans produce. (That robustness is a property of the
//!   algorithm, not something the clean-primitive tests here measure; it is
//!   exercised for real at the imported-mesh rung.)
//!
//! The cost is a one-time build and a discretization error on the order of
//! `cell_size`; both are under the caller's control through the build parameters.

use std::sync::Arc;

use cf_design::{Sdf, Solid};
use mesh_sdf::{CachedGridSdf, TriMeshDistance, WALL_THRESHOLD_FACTOR_DEFAULT};
use mesh_types::{Bounded, IndexedMesh};

/// Failure building a mesh body — either the source mesh is unusable or the
/// signed-distance grid fill could not classify inside from outside.
///
/// `#[non_exhaustive]` in step with the crate's other public error enums
/// ([`InfeasibleDesign`](crate::InfeasibleDesign), [`StopReason`](crate::StopReason)):
/// the set of ways a mesh body can fail to build may grow, and callers should match
/// with a wildcard arm rather than assume these two variants are all of them.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum MeshBodyError {
    /// The parry BVH could not be built (an empty mesh — no faces).
    #[error("mesh signed-distance construction failed: {0}")]
    Sdf(#[from] mesh_sdf::SdfError),
    /// The flood fill that assigns sign to the grid failed (e.g. the padded
    /// bounds left no exterior cell to seed from).
    #[error("signed-distance grid fill failed: {0}")]
    Grid(#[from] mesh_sdf::FloodFillError),
}

/// Build a grid-cached signed-distance [`Sdf`] body from `mesh`.
///
/// The lattice spans the mesh's bounding box padded by `pad` on every side, with
/// nodes `cell_size` apart, and queries interpolate it trilinearly (see the module
/// docs for why this representation).
///
/// **`pad` must enclose the whole region the route is optimized over — the fixed
/// endpoints and any detour — not merely the body.** The lattice spans the mesh's
/// bounding box, so the body is always inside it: a route sample that penetrates the
/// body reads its true (negative) distance and the penalty fires, and no under-sized
/// `pad` can ever hide a collision. The failure mode is subtler. A query *outside* the
/// lattice returns the clamped edge value — roughly `pad` — instead of the larger true
/// distance, so a route point that strays beyond the padded box reads a clearance of
/// only `pad`. If that is below `req = tube_radius + margin` the point picks up a
/// *spurious* clearance penalty, which biases the optimized route and radius. The bias
/// is always conservative (it under-credits real clearance, never over-credits, so the
/// conduit only ever comes out too timid, never colliding), but it distorts the
/// optimum, so size `pad` to cover the endpoints and the expected detour, with headroom
/// beyond `req`. Route endpoints far outside the body's own bounding box are the common
/// trap.
///
/// # Panics
/// Panics if `cell_size` or `pad` is not strictly positive and finite — a malformed
/// build request, not a recoverable runtime condition.
///
/// # Errors
/// Returns [`MeshBodyError::Sdf`] if `mesh` has no faces, or [`MeshBodyError::Grid`]
/// if the sign flood fill fails over the padded bounds.
pub fn mesh_body(
    mesh: IndexedMesh,
    cell_size: f64,
    pad: f64,
) -> Result<Arc<dyn Sdf>, MeshBodyError> {
    assert!(
        cell_size.is_finite() && cell_size > 0.0,
        "cell_size must be positive and finite, got {cell_size}"
    );
    assert!(
        pad.is_finite() && pad > 0.0,
        "pad must be positive and finite, got {pad}"
    );
    let bounds = mesh.aabb().expanded(pad);
    let distance = TriMeshDistance::new(mesh)?;
    let (grid, _report) =
        CachedGridSdf::build(&distance, bounds, cell_size, WALL_THRESHOLD_FACTOR_DEFAULT)?;
    Ok(Arc::new(grid))
}

/// Mesh an analytic [`Solid`] at `mesh_tol` and build a grid-cached mesh body from
/// the result — the same pipeline a real imported mesh takes, driven from a body
/// whose exact field is known.
///
/// This is the natural **stand-in scene** when a real mesh asset is absent: the
/// analytic `Solid::evaluate` is precisely the field this faceted-then-gridded body
/// approximates, so a test can measure the mesh body against ground truth and a
/// co-design run has a primitive to route around without any external file.
///
/// # Panics
/// Panics if `mesh_tol`, `cell_size`, or `pad` is not strictly positive and finite
/// (`mesh_tol` via [`Solid::mesh`], the others via [`mesh_body`]).
///
/// # Errors
/// Propagates [`mesh_body`]'s errors. Note a `Solid` that meshes to nothing (e.g.
/// unbounded geometry) yields an empty mesh and therefore [`MeshBodyError::Sdf`].
pub fn solid_mesh_body(
    solid: &Solid,
    mesh_tol: f64,
    cell_size: f64,
    pad: f64,
) -> Result<Arc<dyn Sdf>, MeshBodyError> {
    mesh_body(solid.mesh(mesh_tol).geometry, cell_size, pad)
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Point3;

    // A coarse sphere on a small lattice — kept tiny so the grid build stays fast in
    // the unmoptimized `--lib` test profile (the quantitative fidelity checks against
    // an analytic body live in the release integration gate `mesh_body_conduit.rs`).
    fn tiny_sphere_body() -> Arc<dyn Sdf> {
        solid_mesh_body(&Solid::sphere(0.5), 0.08, 0.08, 0.3).expect("tiny sphere body builds")
    }

    #[test]
    fn mesh_body_signs_inside_and_outside() {
        let body = tiny_sphere_body();
        assert!(
            body.eval(Point3::origin()) < 0.0,
            "the sphere center must read as inside (negative)"
        );
        assert!(
            body.eval(Point3::new(0.7, 0.0, 0.0)) > 0.0,
            "a point beyond the surface must read as outside (positive)"
        );
    }

    #[test]
    fn mesh_body_eval_approximates_analytic_distance() {
        let body = tiny_sphere_body();
        // On the +x axis at 0.7, the analytic distance to the r=0.5 sphere is 0.2.
        // Coarse mesh + coarse grid, so only a loose bound — a tight one is the
        // integration gate's job.
        let got = body.eval(Point3::new(0.7, 0.0, 0.0));
        assert!(
            (got - 0.2).abs() < 0.05,
            "expected ~0.2 outside the sphere, got {got}"
        );
    }

    #[test]
    #[should_panic(expected = "cell_size must be positive")]
    fn mesh_body_rejects_nonpositive_cell_size() {
        let mesh = Solid::sphere(0.5).mesh(0.1).geometry;
        let _ = mesh_body(mesh, 0.0, 0.3);
    }

    #[test]
    #[should_panic(expected = "pad must be positive")]
    fn mesh_body_rejects_nonpositive_pad() {
        let mesh = Solid::sphere(0.5).mesh(0.1).geometry;
        let _ = mesh_body(mesh, 0.05, 0.0);
    }

    #[test]
    #[should_panic(expected = "mesh tolerance must be positive")]
    fn solid_mesh_body_rejects_nonpositive_mesh_tol() {
        // The documented `mesh_tol` panic path, which routes through `Solid::mesh`.
        let _ = solid_mesh_body(&Solid::sphere(0.5), 0.0, 0.05, 0.3);
    }

    #[test]
    fn mesh_body_rejects_empty_mesh() {
        // `Arc<dyn Sdf>` is not `Debug`, so match rather than `expect_err`.
        let err = match mesh_body(mesh_types::IndexedMesh::new(), 0.05, 0.3) {
            Ok(_) => panic!("an empty mesh (no faces) must not build a body"),
            Err(e) => e,
        };
        assert!(matches!(err, MeshBodyError::Sdf(_)));
    }
}
