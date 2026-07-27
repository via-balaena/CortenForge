//! Newton projection of a point onto an [`Sdf`] zero level-set.
//!
//! The building block the Tet10 SDF-projection mesher uses to decide *where*
//! a boundary midside belongs: given the inscribed chord midpoint of a
//! boundary-face edge, [`project_point_onto_sdf`] slides it onto the true
//! rigid surface so the isoparametric element integrates over the real curved
//! geometry rather than the faceted chord ("exact geometry IS the exact
//! physics"). Deciding *which* nodes to project — boundary-only selection and
//! the inversion back-off — is the mesher's concern
//! ([`Tet10Mesh::with_sdf_projected_boundary`](crate::mesh::Tet10Mesh::with_sdf_projected_boundary));
//! this module only moves one point to the surface.

use crate::Vec3;
use crate::sdf_bridge::Sdf;
use nalgebra::Point3;

/// Project `p` onto the zero level-set of `sdf` by damped Newton iteration.
///
/// Each step moves along the (negated) gradient by the current signed
/// distance: `p ← p − f·g/‖g‖²` where `f = sdf.eval(p)` and `g = sdf.grad(p)`.
/// For a true signed-distance field `‖g‖ = 1` and the step is the exact
/// first-order projection `p − f·g`; the `1/‖g‖²` normalization keeps the step
/// well scaled for the not-quite-unit gradients a composed or discrete SDF can
/// produce. Iterates until `|f| ≤ tol` or `max_iter` steps are taken.
///
/// Returns the (possibly still off-surface) point. Convergence is not
/// guaranteed for a point far from the surface or one that lands on an SDF
/// singularity (a CSG crease, or the centre of a sphere where the gradient is
/// undefined); the iteration bails out on an exactly-zero gradient rather than
/// dividing by it. Callers that need the on-surface guarantee should check
/// `sdf.eval(result.into()).abs() ≤ tol` — the mesher does, and keeps a node
/// that fails to converge at its straight position.
#[must_use]
pub fn project_point_onto_sdf(sdf: &dyn Sdf, p: Vec3, tol: f64, max_iter: usize) -> Vec3 {
    let mut q = p;
    for _ in 0..max_iter {
        let f = sdf.eval(Point3::from(q));
        if f.abs() <= tol {
            break;
        }
        let g = sdf.grad(Point3::from(q));
        let gg = g.norm_squared();
        // Float equality is intentional: `norm_squared` is a sum of squares,
        // exactly `0.0` iff every component is exactly `0.0` — the SDF
        // gradient singularity (e.g. a sphere centre). There is no descent
        // direction there, so bail rather than divide by zero; the returned
        // point stays off-surface and the caller's convergence check rejects
        // it.
        #[allow(clippy::float_cmp)]
        let singular = gg == 0.0;
        if singular {
            break;
        }
        q -= g * (f / gg);
    }
    q
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sdf_bridge::{DifferenceSdf, SphereSdf};

    const TOL: f64 = 1e-12;

    #[test]
    fn projects_exterior_point_onto_sphere() {
        let sdf = SphereSdf { radius: 1.0 };
        // A point well outside the unit sphere, off-axis.
        let p = Vec3::new(3.0, -2.0, 1.5);
        let q = project_point_onto_sdf(&sdf, p, TOL, 32);
        assert!(
            sdf.eval(Point3::from(q)).abs() <= TOL,
            "must land on surface"
        );
        // First-order projection onto a sphere is exactly radial: the
        // projected point is the input scaled to unit radius.
        let expected = p / p.norm();
        assert!((q - expected).norm() < 1e-10, "sphere projection is radial");
    }

    #[test]
    fn projects_interior_point_onto_sphere() {
        let sdf = SphereSdf { radius: 2.0 };
        let p = Vec3::new(0.3, 0.1, -0.2); // inside, off-centre
        let q = project_point_onto_sdf(&sdf, p, TOL, 32);
        assert!(sdf.eval(Point3::from(q)).abs() <= TOL);
        assert!((q.norm() - 2.0).abs() < 1e-10);
    }

    #[test]
    fn on_surface_point_is_idempotent() {
        let sdf = SphereSdf { radius: 1.0 };
        let on = Vec3::new(1.0, 0.0, 0.0); // exactly on the surface
        let q = project_point_onto_sdf(&sdf, on, TOL, 32);
        // eval is already 0 → the loop breaks on the first iteration and the
        // point is returned unchanged, bit-for-bit.
        assert_eq!(q, on);
    }

    #[test]
    fn converges_on_composed_difference_sdf() {
        // A hollow shell: outer radius 1.0 minus a cavity radius 0.4. The
        // difference SDF has a non-unit gradient in general and a crease, but
        // near either spherical surface it behaves like that sphere.
        let shell = DifferenceSdf::new(
            Box::new(SphereSdf { radius: 1.0 }),
            Box::new(SphereSdf { radius: 0.4 }),
        );
        // A point just inside the outer surface projects out to it.
        let outer = project_point_onto_sdf(&shell, Vec3::new(0.9, 0.0, 0.0), TOL, 32);
        assert!(shell.eval(Point3::from(outer)).abs() <= TOL);
        assert!((outer.norm() - 1.0).abs() < 1e-9);
        // A point just outside the cavity (inside the material) projects onto
        // the cavity surface, the nearer of the two zero surfaces.
        let cavity = project_point_onto_sdf(&shell, Vec3::new(0.45, 0.0, 0.0), TOL, 32);
        assert!(shell.eval(Point3::from(cavity)).abs() <= TOL);
        assert!((cavity.norm() - 0.4).abs() < 1e-9);
    }

    #[test]
    fn centre_singularity_bails_without_nan() {
        // The sphere centre has an undefined gradient; the projector must bail
        // rather than divide by zero and return a finite (if off-surface)
        // point. The caller rejects it via the convergence check.
        let sdf = SphereSdf { radius: 1.0 };
        let q = project_point_onto_sdf(&sdf, Vec3::zeros(), TOL, 32);
        assert!(q.iter().all(|c| c.is_finite()));
    }
}
