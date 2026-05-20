//! `cf_geometry::Sdf` adapter blankets for mesh-sdf's signed-distance
//! types.
//!
//! Hosts the two adapter impls that bridge mesh-sdf's
//! [`Signed`](crate::Signed) composition + [`CachedGridSdf`](crate::CachedGridSdf)
//! cached grid to the workspace-wide [`Sdf`](cf_geometry::Sdf) trait.
//! Living in mesh-sdf is orphan-rule-clean: both the implementing
//! types and (post-migration) the trait host crate are mesh-sdf's
//! direct deps.

use cf_geometry::Sdf;
use nalgebra::{Point3, Vector3};

use crate::{CachedGridSdf, Sign, Signed, UnsignedDistance};

/// Every composed [`Signed<D, S>`] satisfies the [`Sdf`] contract —
/// blanket adapter spanning the decomposed [`UnsignedDistance`] +
/// [`Sign`] traits.
///
/// `eval` delegates to [`Signed::evaluate`] (negative inside, positive
/// outside, zero on the surface).
///
/// `grad` is a central finite-difference approximation with step size
/// `1e-6` (matching cf-design's existing `Solid::user_fn` finite-diff
/// fallback). Mesh SDFs have piecewise-smooth gradients with
/// discontinuities across face boundaries, so an analytic gradient is
/// not available in closed form. Within `1e-6` of a face boundary the
/// computed gradient may be biased toward one side's outward normal;
/// consumers requiring boundary-stable gradients should query at points
/// well within face interiors.
///
/// **Sign-oracle caveat.** The reliability of `eval`'s sign branch is
/// determined by the wrapped `S: Sign` oracle.
/// [`crate::PseudoNormalSign`] is reliable on watertight, well-formed
/// meshes but **fragile** on decimated / cleaned scans with cap fans
/// whose winding flipped during reconstruction or high-valence apex
/// vertices — see project memory `pinned-floor-visual-gate-postmortem`
/// for the canonical iter-1 failure mode. Consumers deriving an SDF
/// from a cf-scan-prep cleaned scan should prefer [`crate::FloodFillSign`]
/// in place of [`crate::PseudoNormalSign`]. See
/// `docs/MESH_SDF_ORACLE_DECOMPOSITION_SPEC.md` for the rationale.
impl<D, S> Sdf for Signed<D, S>
where
    D: UnsignedDistance,
    S: Sign,
{
    fn eval(&self, p: Point3<f64>) -> f64 {
        self.evaluate(p)
    }

    fn grad(&self, p: Point3<f64>) -> Vector3<f64> {
        let eps = 1e-6;
        let inv_2eps = 0.5 / eps;
        Vector3::new(
            (self.evaluate(Point3::new(p.x + eps, p.y, p.z))
                - self.evaluate(Point3::new(p.x - eps, p.y, p.z)))
                * inv_2eps,
            (self.evaluate(Point3::new(p.x, p.y + eps, p.z))
                - self.evaluate(Point3::new(p.x, p.y - eps, p.z)))
                * inv_2eps,
            (self.evaluate(Point3::new(p.x, p.y, p.z + eps))
                - self.evaluate(Point3::new(p.x, p.y, p.z - eps)))
                * inv_2eps,
        )
    }
}

/// [`CachedGridSdf`] implements [`Sdf`] via its native trilinear-
/// interpolated signed distance + analytic-on-the-trilinear gradient.
/// The grid IS the smoothed source, so `eval` reads directly and
/// `grad` central-differences the trilinear interpolant at the lattice
/// spacing — exact for the piecewise-trilinear representation, no
/// `Signed::grad`-style 1e-6 finite-difference step needed.
impl Sdf for CachedGridSdf {
    fn eval(&self, p: Point3<f64>) -> f64 {
        self.signed_distance(p)
    }

    fn grad(&self, p: Point3<f64>) -> Vector3<f64> {
        self.gradient(p)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    /// Regular tetrahedron with the bottom face on z=0 and apex above.
    /// Bottom face winding `[0, 2, 1]` is CCW from below — outward face
    /// normal of the bottom is `-z`.
    fn unit_tetrahedron() -> mesh_types::IndexedMesh {
        let mut mesh = mesh_types::IndexedMesh::new();
        mesh.vertices.push(Point3::new(0.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(1.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(0.5, 0.866, 0.0));
        mesh.vertices.push(Point3::new(0.5, 0.289, 0.816));
        mesh.faces.push([0, 2, 1]); // bottom (outward = -z)
        mesh.faces.push([0, 1, 3]); // front
        mesh.faces.push([1, 2, 3]); // right
        mesh.faces.push([2, 0, 3]); // left
        mesh
    }

    /// Construct the explicit `Signed<TriMeshDistance, PseudoNormalSign>`
    /// composition the post-D.1 API uses — the blanket
    /// `impl<D, S> Sdf for Signed<D, S>` is what these tests pin.
    #[allow(clippy::expect_used)]
    fn mesh_sdf_for_tetrahedron() -> Signed<crate::TriMeshDistance, crate::PseudoNormalSign> {
        let distance = crate::TriMeshDistance::new(unit_tetrahedron())
            .expect("tetrahedron fixture has four faces");
        let sign = crate::PseudoNormalSign::from_distance(&distance);
        Signed { distance, sign }
    }

    #[test]
    fn mesh_sdf_eval_sign_convention_in_contact_band() {
        let sdf = mesh_sdf_for_tetrahedron();

        // Strictly inside (negative): on the bottom-centroid → apex
        // axis halfway up.
        let interior = Point3::new(0.5, 0.289, 0.4);
        assert!(Sdf::eval(&sdf, interior) < 0.0);

        // Strictly outside (positive): one unit below the bottom-face
        // centroid — the closest face is unambiguously the bottom
        // (outward `-z`), and the signed distance is exactly 1.
        let exterior_below = Point3::new(0.5, 0.289, -1.0);
        assert_relative_eq!(Sdf::eval(&sdf, exterior_below), 1.0, epsilon = 1e-12,);
    }

    #[test]
    fn mesh_sdf_grad_below_bottom_face_approximates_outward_normal() {
        let sdf = mesh_sdf_for_tetrahedron();

        // Probe directly below the bottom-face centroid (0.5, 0.289, 0)
        // at z = -1 — the closest face is the bottom triangle (winding
        // `[0, 2, 1]`, outward normal -z), and the central-difference
        // gradient should match that normal up to the float-equality
        // tolerance of mesh-sdf's distance computation.
        let p = Point3::new(0.5, 0.289, -1.0);
        let g = Sdf::grad(&sdf, p);
        assert_relative_eq!(g, -Vector3::z(), epsilon = 1e-6);
    }

    /// `impl Sdf for CachedGridSdf` — `eval` reads the cached signed
    /// grid directly; `grad` central-differences the trilinear
    /// interpolant. Verifies the adapter honors the [`Sdf`] sign
    /// convention end-to-end on a known fixture (signed-distance
    /// negative interior, positive below the bottom face, outward
    /// gradient pointing -z below).
    #[allow(clippy::expect_used)]
    #[test]
    fn cached_grid_sdf_adapter_honors_sdf_sign_convention() {
        let distance = crate::TriMeshDistance::new(unit_tetrahedron())
            .expect("tetrahedron fixture has four faces");
        let bounds =
            mesh_types::Aabb::new(Point3::new(-1.0, -1.0, -1.5), Point3::new(2.0, 2.0, 2.0));
        let (cached, report) = crate::CachedGridSdf::build(
            &distance,
            bounds,
            0.1,
            crate::WALL_THRESHOLD_FACTOR_DEFAULT,
        )
        .expect("CachedGridSdf builds for the tetrahedron fixture");
        assert_eq!(report.inside_components, 1);

        // Interior probe (negative).
        let interior = Point3::new(0.5, 0.289, 0.4);
        assert!(Sdf::eval(&cached, interior) < 0.0);

        // Exterior probe below the bottom face — gradient should
        // point predominantly -z (outward from the body).
        let below = Point3::new(0.5, 0.289, -1.0);
        assert!(Sdf::eval(&cached, below) > 0.0);
        let g = Sdf::grad(&cached, below);
        assert!(
            g.z < -0.5,
            "below-bottom-face gradient should point -z, got {g:?}"
        );
    }
}
