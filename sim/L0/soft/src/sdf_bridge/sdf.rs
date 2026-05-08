//! Sdf trait re-export + sphere validation impl.
//!
//! The [`Sdf`] trait is owned by [`cf_design`]; sim-soft re-exports it
//! so existing import paths (`sim_soft::Sdf`,
//! `sim_soft::sdf_bridge::Sdf`) keep resolving. Trait method receivers
//! are `Point3<f64>`; sim-soft call sites adapt at the boundary via
//! `Point3::from(vec3)`.
//!
//! # Two SDF surfaces, one trait
//!
//! sim-soft's [`SphereSdf`] and cf-design's [`cf_design::Solid::sphere`]
//! ctor are both [`Sdf`] implementors of the same primitive; they are
//! retained side-by-side because each surfaces a different consumer
//! role:
//!
//! - [`cf_design::Solid::sphere`] is the **production design surface**
//!   for typed-CSG composition between cf-design primitives — pair it
//!   with [`cf_design::Solid::subtract`] / `union` / etc. when building
//!   a body whose operands are all themselves `Solid`-typed.
//! - [`SphereSdf`] is the **trait teaching primitive** + the
//!   bit-pinned reference surface for IV-1 / IV-2 / IV-3 / IV-5
//!   invariant fixtures. The Tier 1 example crates
//!   [`sphere-sdf-eval`][r1], [`hollow-shell-sdf`][r2] (via
//!   [`DifferenceSdf`](super::DifferenceSdf)), and
//!   [`sdf-to-tet-sphere`][r3] teach the [`Sdf`] contract through
//!   `SphereSdf`'s minimal-surface implementation; the
//!   [`solid-to-sim-soft`][r16] HEADLINE A bridge anchor pins
//!   `Solid::sphere(r).evaluate(&p)` bit-equal to `SphereSdf::eval(p)`
//!   at `EXACT_TOL = 0.0` as the cross-crate semantic-equivalence
//!   contract.
//!
//! Both surfaces are correct + retained; the choice is "are you
//! composing typed cf-design solids" (use `Solid`) vs "are you
//! teaching the `Sdf` trait or anchoring the bridge" (use `SphereSdf`).
//!
//! [r1]: ../../../../../examples/sim-soft/sphere-sdf-eval/
//! [r2]: ../../../../../examples/sim-soft/hollow-shell-sdf/
//! [r3]: ../../../../../examples/sim-soft/sdf-to-tet-sphere/
//! [r16]: ../../../../../examples/sim-soft/solid-to-sim-soft/

use crate::Vec3;
use nalgebra::Point3;

pub use cf_design::Sdf;

/// Sphere centred at the origin with the given radius.
///
/// `eval = ‖p‖ − r` and `grad = p / ‖p‖`. At `p = 0` the gradient is
/// undefined (the sphere centre is an interior singularity); the impl
/// returns `Vec3::z()` arbitrarily — meshing only queries the gradient
/// near the SDF zero set, never at the centre, so the choice is
/// unobservable downstream.
///
/// `SphereSdf` is the canonical Tier 1 teaching primitive for the
/// [`Sdf`] trait + the bit-pinned reference surface for IV-1 / IV-2 /
/// IV-3 invariant fixtures (where reference values depend on this
/// struct's specific `eval` arithmetic). For typed-CSG composition
/// between cf-design primitives, use [`cf_design::Solid::sphere`]
/// instead — that's the production design surface; `SphereSdf` is the
/// minimal-surface impl that teaches the trait. See the module-level
/// docs for the role split.
///
/// `Solid::sphere(r).evaluate(&p)` is bit-equivalent on `eval`, and
/// both impls return `Vector3::z()` at the origin singularity —
/// cf-design's `grad_sphere` uses a `< 1e-15` near-singularity guard
/// (uniform with capsule / torus / cuboid) while sim-soft uses the
/// singleton `n == 0.0` predicate per the IEEE 754 sqrt argument below;
/// the fallback values agree, so the trait contract is satisfied
/// uniformly even though the predicate bands differ slightly.
#[derive(Clone, Debug)]
pub struct SphereSdf {
    /// Radius in world units (metres). Must be positive for the SDF to
    /// describe a non-empty interior.
    pub radius: f64,
}

impl Sdf for SphereSdf {
    fn eval(&self, p: Point3<f64>) -> f64 {
        p.coords.norm() - self.radius
    }

    fn grad(&self, p: Point3<f64>) -> Vec3 {
        let n = p.coords.norm();
        // Float equality is intentional and correct here. `f64::sqrt`
        // of a sum of squares returns exactly `0.0` iff every component
        // is exactly `0.0` (IEEE 754 sqrt is exact at 0), so `n == 0.0`
        // triggers iff `p == Point3::origin()` exactly — the singleton
        // singularity at the sphere centre. Any other input gives `n`
        // strictly positive and the normalised gradient is well-defined.
        // `Vec3::z()` fallback is unobservable downstream — the centre
        // lives strictly inside the SDF, far from the zero set the
        // mesher actually queries.
        #[allow(clippy::float_cmp)]
        let at_singularity = n == 0.0;
        if at_singularity {
            Vec3::z()
        } else {
            p.coords / n
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn eval_at_origin_returns_negative_radius() {
        let s = SphereSdf { radius: 0.1 };
        assert_relative_eq!(s.eval(Point3::origin()), -0.1, epsilon = 1e-15);
    }

    #[test]
    fn eval_on_surface_is_zero() {
        let s = SphereSdf { radius: 0.1 };
        assert_relative_eq!(s.eval(Point3::new(0.1, 0.0, 0.0)), 0.0, epsilon = 1e-15,);
        let inv_sqrt3 = 1.0 / 3.0_f64.sqrt();
        assert_relative_eq!(
            s.eval(Point3::new(
                0.1 * inv_sqrt3,
                0.1 * inv_sqrt3,
                0.1 * inv_sqrt3
            )),
            0.0,
            epsilon = 1e-15,
        );
    }

    #[test]
    fn eval_at_exterior_is_positive() {
        let s = SphereSdf { radius: 0.1 };
        assert_relative_eq!(s.eval(Point3::new(0.2, 0.0, 0.0)), 0.1, epsilon = 1e-15,);
    }

    #[test]
    fn grad_at_axis_aligned_points_is_unit_axis() {
        let s = SphereSdf { radius: 0.1 };
        assert_relative_eq!(
            s.grad(Point3::new(0.1, 0.0, 0.0)),
            Vec3::x(),
            epsilon = 1e-15,
        );
        assert_relative_eq!(
            s.grad(Point3::new(0.0, 0.1, 0.0)),
            Vec3::y(),
            epsilon = 1e-15,
        );
        assert_relative_eq!(
            s.grad(Point3::new(0.0, 0.0, 0.1)),
            Vec3::z(),
            epsilon = 1e-15,
        );
    }

    #[test]
    fn grad_at_origin_returns_documented_z_fallback() {
        let s = SphereSdf { radius: 0.1 };
        assert_relative_eq!(s.grad(Point3::origin()), Vec3::z(), epsilon = 1e-15);
    }
}
