//! Implicit-surface SDF trait.
//!
//! [`Sdf`] is the contract for signed-distance functions in cf-design.
//! [`Solid`] is the built-in implementor; downstream crates and user
//! code add more by writing their own `impl Sdf for ...`. Consumers
//! query SDFs through the trait without naming the concrete source.

use mesh_sdf::SignedDistanceField;
use nalgebra::{Point3, Vector3};

use crate::Solid;

/// Signed-distance function over R³.
///
/// Sign convention: `eval` returns a negative value strictly inside the
/// body, positive strictly outside, zero on the surface. `grad` returns
/// the gradient of the signed-distance field — unit-length on the zero
/// set when the field is exact, undefined at interior singularities (a
/// concrete impl picks an arbitrary fallback there).
///
/// `Send + Sync` so trait objects (`Box<dyn Sdf>` inside composition
/// trees) satisfy thread-safety bounds at every storage site without
/// `+ Send + Sync` clutter. Every reasonable implementor is naturally
/// `Send + Sync` — a pure function over `Point3<f64>`, or a composition
/// of the same — and the supertrait documents the invariant.
pub trait Sdf: Send + Sync {
    /// Signed distance from `p` to the surface.
    fn eval(&self, p: Point3<f64>) -> f64;

    /// Gradient of [`Sdf::eval`] at `p`.
    fn grad(&self, p: Point3<f64>) -> Vector3<f64>;
}

/// Forwarding impl through `Box<T>` for any `T: Sdf + ?Sized`.
///
/// Lets a heap-erased SDF satisfy [`Sdf`] directly, so a `Box<dyn Sdf>`
/// (or a `Vec<Box<dyn Sdf>>` of mixed concrete implementors) is callable
/// at every site that takes any `S: Sdf` — without an inner deref or a
/// `&dyn Sdf` reborrow.
impl<T: Sdf + ?Sized> Sdf for Box<T> {
    fn eval(&self, p: Point3<f64>) -> f64 {
        (**self).eval(p)
    }

    fn grad(&self, p: Point3<f64>) -> Vector3<f64> {
        (**self).grad(p)
    }
}

/// Every [`Solid`] satisfies the [`Sdf`] contract.
///
/// `eval` delegates to [`Solid::evaluate`]; `grad` delegates to
/// [`Solid::gradient`], which uses analytic derivatives for built-in
/// primitives and operations and falls back to finite differences only
/// for [`Solid::user_fn`].
impl Sdf for Solid {
    fn eval(&self, p: Point3<f64>) -> f64 {
        self.evaluate(&p)
    }

    fn grad(&self, p: Point3<f64>) -> Vector3<f64> {
        self.gradient(&p)
    }
}

/// Every [`mesh_sdf::SignedDistanceField`] satisfies the [`Sdf`] contract.
///
/// `eval` delegates to [`SignedDistanceField::distance`]; both use the
/// same sign convention (negative inside, positive outside, zero on
/// surface).
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
/// **Sign-heuristic caveat (inherited from `mesh-sdf`).** mesh-sdf's
/// `distance` infers inside/outside from a face-normal heuristic
/// applied to the closest face. The heuristic is reliable for points
/// within roughly one mesh-edge-length of the surface, but for points
/// far from the surface where the closest face's plane happens to lie
/// between the probe and the rest of the body, the heuristic can
/// return the wrong sign. Consumers needing robust far-field
/// inside/outside should prefer [`SignedDistanceField::is_inside`]
/// (ray-cast based) directly.
impl Sdf for SignedDistanceField {
    fn eval(&self, p: Point3<f64>) -> f64 {
        self.distance(p)
    }

    fn grad(&self, p: Point3<f64>) -> Vector3<f64> {
        let eps = 1e-6;
        let inv_2eps = 0.5 / eps;
        Vector3::new(
            (self.distance(Point3::new(p.x + eps, p.y, p.z))
                - self.distance(Point3::new(p.x - eps, p.y, p.z)))
                * inv_2eps,
            (self.distance(Point3::new(p.x, p.y + eps, p.z))
                - self.distance(Point3::new(p.x, p.y - eps, p.z)))
                * inv_2eps,
            (self.distance(Point3::new(p.x, p.y, p.z + eps))
                - self.distance(Point3::new(p.x, p.y, p.z - eps)))
                * inv_2eps,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn solid_sphere_eval_sign_convention() {
        let s = Solid::sphere(1.0);

        // Strictly inside (negative).
        assert!(<Solid as Sdf>::eval(&s, Point3::origin()) < 0.0);
        assert!(<Solid as Sdf>::eval(&s, Point3::new(0.5, 0.0, 0.0)) < 0.0);

        // On the surface (zero).
        assert_relative_eq!(
            <Solid as Sdf>::eval(&s, Point3::new(1.0, 0.0, 0.0)),
            0.0,
            epsilon = 1e-15,
        );

        // Strictly outside (positive).
        assert!(<Solid as Sdf>::eval(&s, Point3::new(2.0, 0.0, 0.0)) > 0.0);
    }

    #[test]
    fn solid_sphere_grad_unit_outward_on_surface() {
        let s = Solid::sphere(1.0);
        for (p, expected) in [
            (Point3::new(1.0, 0.0, 0.0), Vector3::x()),
            (Point3::new(0.0, 1.0, 0.0), Vector3::y()),
            (Point3::new(0.0, 0.0, 1.0), Vector3::z()),
            (Point3::new(-1.0, 0.0, 0.0), -Vector3::x()),
        ] {
            let g = <Solid as Sdf>::grad(&s, p);
            assert_relative_eq!(g.norm(), 1.0, epsilon = 1e-12);
            assert_relative_eq!(g, expected, epsilon = 1e-12);
        }
    }

    #[test]
    fn solid_csg_difference_eval_at_hollow_shell_probes() {
        let shell = Solid::sphere(1.0).subtract(Solid::sphere(0.4));

        // Probe at radius 0.7 (inside outer, outside inner): φ_outer =
        // -0.3, -φ_inner = -0.3. max(-0.3, -0.3) = -0.3. The tie-break
        // is exact since the probe is equidistant from both surfaces.
        let mid_shell = Point3::new(0.7, 0.0, 0.0);
        assert_relative_eq!(
            <Solid as Sdf>::eval(&shell, mid_shell),
            -0.3,
            epsilon = 1e-12
        );

        // Probe at radius 0.2 (inside inner cavity): φ_outer = -0.8,
        // -φ_inner = 0.2. max(-0.8, 0.2) = 0.2 — outside the shell
        // body because we are inside the cavity.
        let in_cavity = Point3::new(0.2, 0.0, 0.0);
        assert_relative_eq!(
            <Solid as Sdf>::eval(&shell, in_cavity),
            0.2,
            epsilon = 1e-12
        );
    }

    #[test]
    fn box_dyn_sdf_blanket_forwards_eval_and_grad() {
        let s = Solid::sphere(1.0);
        let boxed: Box<dyn Sdf> = Box::new(s.clone());
        for &p in &[
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.5, 0.5, 0.5),
            Point3::new(2.0, 0.0, 0.0),
        ] {
            assert_relative_eq!(boxed.eval(p), s.evaluate(&p), epsilon = 0.0);
            assert_relative_eq!(boxed.grad(p), s.gradient(&p), epsilon = 0.0);
        }
    }

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

    #[test]
    // Localized expect: tetrahedron fixture has 4 faces, never empty.
    #[allow(clippy::expect_used)]
    fn mesh_sdf_eval_sign_convention_in_contact_band() {
        let sdf = SignedDistanceField::new(unit_tetrahedron())
            .expect("tetrahedron fixture has four faces");

        // Strictly inside (negative): on the bottom-centroid → apex
        // axis halfway up.
        let interior = Point3::new(0.5, 0.289, 0.4);
        assert!(<SignedDistanceField as Sdf>::eval(&sdf, interior) < 0.0);

        // Strictly outside (positive): one unit below the bottom-face
        // centroid — the closest face is unambiguously the bottom
        // (outward `-z`), and the signed distance is exactly 1.
        let exterior_below = Point3::new(0.5, 0.289, -1.0);
        assert_relative_eq!(
            <SignedDistanceField as Sdf>::eval(&sdf, exterior_below),
            1.0,
            epsilon = 1e-12,
        );
    }

    #[test]
    // Localized expect: tetrahedron fixture has 4 faces, never empty.
    #[allow(clippy::expect_used)]
    fn mesh_sdf_grad_below_bottom_face_approximates_outward_normal() {
        let sdf = SignedDistanceField::new(unit_tetrahedron())
            .expect("tetrahedron fixture has four faces");

        // Probe directly below the bottom-face centroid (0.5, 0.289, 0)
        // at z = -1 — the closest face is the bottom triangle (winding
        // `[0, 2, 1]`, outward normal -z), and the central-difference
        // gradient should match that normal up to the float-equality
        // tolerance of mesh-sdf's distance computation.
        let p = Point3::new(0.5, 0.289, -1.0);
        let g = <SignedDistanceField as Sdf>::grad(&sdf, p);
        assert_relative_eq!(g, -Vector3::z(), epsilon = 1e-6);
    }
}
