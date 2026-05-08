//! Implicit-surface SDF trait.
//!
//! [`Sdf`] is the contract that every signed-distance source in cf-design
//! satisfies. Consumers query SDFs through the trait without naming the
//! concrete source — a [`Solid`] composed from primitives and CSG
//! operators, a scan-derived field added by a downstream crate, or a
//! user-defined source written from scratch.

use nalgebra::{Point3, Vector3};

use crate::Solid;

/// Signed-distance function over R³.
///
/// Sign convention: `eval` returns a negative value strictly inside the
/// body, positive strictly outside, zero on the surface. `grad` returns
/// the gradient of the signed-distance field — unit-length on the zero
/// set when the field is exact, undefined at interior singularities (a
/// concrete impl picks an arbitrary fallback there, see e.g. the sphere
/// centre case in downstream `SphereSdf` impls).
///
/// `Send + Sync` so trait objects (`Box<dyn Sdf>` inside CSG composition
/// trees and the downstream simulation surface's spatial fields) satisfy
/// thread-safety bounds at every storage site without `+ Send + Sync`
/// clutter. Every reasonable implementor is naturally `Send + Sync` —
/// a pure function over `Point3<f64>`, or a composition of the same —
/// and the supertrait documents the invariant.
pub trait Sdf: Send + Sync {
    /// Signed distance from `p` to the surface.
    fn eval(&self, p: Point3<f64>) -> f64;

    /// Gradient of [`Sdf::eval`] at `p`.
    fn grad(&self, p: Point3<f64>) -> Vector3<f64>;
}

/// Forwarding impl through `Box<T>` (and any other heap-erased
/// `Box<dyn Sdf>`).
///
/// Lets a heap-erased SDF satisfy [`Sdf`] directly, so a uniform
/// composition tree (e.g. `Vec<Box<dyn Sdf>>`) can be passed to the
/// same generic constructors that accept a homogeneous concrete vector.
/// Sites that mix concrete primitives box each into a homogeneous
/// `Vec<Box<dyn Sdf>>` and pass it through the same call path as a
/// concrete vector.
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
///
/// Receiver-type idiom shift: [`Solid::evaluate`] and [`Solid::gradient`]
/// take `&Point3<f64>`; the [`Sdf`] trait takes `Point3<f64>` by value.
/// `Point3<f64>` is `Copy` (24 bytes), so the shift is zero-cost — by
/// value matches the wider Rust trait idiom for tiny `Copy` types and
/// drops the deref at every call site of every implementor.
impl Sdf for Solid {
    fn eval(&self, p: Point3<f64>) -> f64 {
        self.evaluate(&p)
    }

    fn grad(&self, p: Point3<f64>) -> Vector3<f64> {
        self.gradient(&p)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn solid_sphere_eval_matches_inherent_evaluate() {
        let s = Solid::sphere(1.0);
        for &p in &[
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.5, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
        ] {
            assert_relative_eq!(<Solid as Sdf>::eval(&s, p), s.evaluate(&p), epsilon = 0.0);
        }
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
    fn solid_csg_difference_eval_matches_max_chain_rule() {
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
}
