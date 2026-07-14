//! Sdf trait re-export + sphere validation impl.
//!
//! The [`Sdf`] trait is owned by [`cf_geometry`]; sim-soft re-exports
//! it so existing import paths (`sim_soft::Sdf`,
//! `sim_soft::sdf_bridge::Sdf`) keep resolving. Trait method receivers
//! are `Point3<f64>`; sim-soft call sites adapt at the boundary via
//! `Point3::from(vec3)`.
//!
//! # Two SDF surfaces, one trait
//!
//! sim-soft's [`SphereSdf`] and cf-design's `cf_design::Solid::sphere`
//! ctor are both [`Sdf`] implementors of the same primitive; they are
//! retained side-by-side because each surfaces a different consumer
//! role:
//!
//! - `cf_design::Solid::sphere` is the **production design surface**
//!   for typed-CSG composition between cf-design primitives — pair it
//!   with `cf_design::Solid::subtract` / `union` / etc. when building
//!   a body whose operands are all themselves `Solid`-typed.
//! - [`SphereSdf`] is the **trait teaching primitive** + the
//!   bit-pinned reference surface for IV-1 / IV-2 / IV-3 / IV-5
//!   invariant fixtures. The Tier 1 [`sdf/stress-test`][sdf] modules
//!   `sphere_eval`, `hollow_shell` (via
//!   [`DifferenceSdf`](super::DifferenceSdf)), and
//!   `sdf_to_tet` teach the [`Sdf`] contract through
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
//! [sdf]: ../../../../../examples/sim-soft/sdf/stress-test/
//! [r16]: ../../../../../examples/sim-soft/solid-to-sim-soft/

use crate::Vec3;
use nalgebra::{Matrix3, Point3};

pub use cf_geometry::Sdf;

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
/// between cf-design primitives, use `cf_design::Solid::sphere`
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

    /// `∇²(‖p‖ − r) = (I − n̂n̂ᵀ)/‖p‖` — the curvature of a sphere's signed-distance
    /// field (the `−r` is constant, Hessian-free). `n̂ = p/‖p‖`; the result is the
    /// tangential projector at `p` scaled by `1/‖p‖`, so the normal swings faster as
    /// the query nears the centre. Returns the zero matrix at the centre singularity
    /// (the same `n == 0.0` predicate `grad` uses), unobservable downstream.
    fn hessian(&self, p: Point3<f64>) -> Matrix3<f64> {
        let n = p.coords.norm();
        // `n == 0.0` iff `p` is exactly the centre singularity (IEEE 754 `sqrt` is
        // exact at 0) — the same predicate + rationale as `grad` above; the result
        // there (`(I − n̂n̂ᵀ)/n`) blows up, so the centre returns the zero matrix.
        #[allow(clippy::float_cmp)]
        if n == 0.0 {
            Matrix3::zeros()
        } else {
            let nhat = p.coords / n;
            (Matrix3::identity() - nhat * nhat.transpose()) / n
        }
    }
}

/// An [`Sdf`] translated so its origin sits at a world `offset` — a query-point
/// shift `p ↦ p − offset` forwarded to the `inner` primitive.
///
/// Translation commutes with the SDF derivatives: `eval`, `grad`, and `hessian`
/// (the curvature term #415 added) are all evaluated at the shifted point, so a
/// translated [`SphereSdf`] keeps the sphere's own `∇²sd` curvature exactly. This
/// is the shared posing wrapper for a finite collider riding a moving frame — the
/// keystone coupling poses a `TranslatedSdf<SphereSdf>` end-effector over the soft
/// block, and the `soft_pose_sensitivity` / `friction_sphere_tangent` curved-pose
/// gates pose the same type (deduping their former local copies).
///
/// Translation alone is the full pose for a sphere (rotation-invariant about its
/// centre); a primitive needing orientation would compose this with a rotation
/// wrapper.
#[derive(Clone, Copy, Debug)]
pub struct TranslatedSdf<S> {
    /// The origin-centred primitive being posed.
    pub inner: S,
    /// World-space point the primitive's origin is translated to.
    pub offset: Vec3,
}

impl<S: Sdf> Sdf for TranslatedSdf<S> {
    fn eval(&self, p: Point3<f64>) -> f64 {
        self.inner.eval(p - self.offset)
    }

    fn grad(&self, p: Point3<f64>) -> Vec3 {
        self.inner.grad(p - self.offset)
    }

    fn hessian(&self, p: Point3<f64>) -> Matrix3<f64> {
        self.inner.hessian(p - self.offset)
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
