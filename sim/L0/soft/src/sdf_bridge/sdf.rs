//! Signed-distance-function trait + sphere validation impl.
//!
//! Names follow the book-canonical short forms (`eval` / `grad`) per
//! Part 7 Ch 00 §02. [`SphereSdf`] is the III-1 / III-2 / III-3
//! validation scene per scope-memo Decision C; cube + CSG composition
//! deferred to Phase 4 or a Phase 3 follow-on.

use crate::Vec3;

/// Implicit-surface SDF over R³.
///
/// `eval` returns signed distance — negative inside, positive outside,
/// zero on the surface. `grad` returns the gradient — unit-length when
/// the SDF is well-formed, undefined at interior singularities (the
/// concrete impl picks an arbitrary fallback there; see
/// [`SphereSdf::grad`]).
///
/// `Send + Sync` so trait objects (`Box<dyn Sdf>` inside Phase 4's
/// [`LayeredScalarField`](crate::field::LayeredScalarField) +
/// [`BlendedScalarField`](crate::field::BlendedScalarField), and the
/// upcoming `MaterialField` / `MeshingHints::material_field` storage
/// sites) satisfy the [`Field<T>: Send + Sync`](crate::field::Field)
/// bound at every level of the typed-field composition tree without
/// `+ Send + Sync` clutter at every storage site. Every reasonable Sdf
/// impl is naturally Send + Sync (pure-function over Vec3, or a
/// composition of the same); the supertrait documents the invariant.
pub trait Sdf: Send + Sync {
    /// Signed distance from `p` to the surface.
    fn eval(&self, p: Vec3) -> f64;

    /// Gradient of [`Sdf::eval`] at `p`.
    fn grad(&self, p: Vec3) -> Vec3;
}

/// Sphere centred at the origin with the given radius.
///
/// `eval = ‖p‖ − r` and `grad = p / ‖p‖`. At `p = 0` the gradient is
/// undefined (the sphere centre is an interior singularity); the impl
/// returns `Vec3::z()` arbitrarily — meshing only queries the gradient
/// near the SDF zero set, never at the centre, so the choice is
/// unobservable downstream (memo §2 + Decision M).
#[derive(Clone, Debug)]
pub struct SphereSdf {
    /// Radius in world units (metres). Must be positive for the SDF to
    /// describe a non-empty interior.
    pub radius: f64,
}

impl Sdf for SphereSdf {
    fn eval(&self, p: Vec3) -> f64 {
        p.norm() - self.radius
    }

    fn grad(&self, p: Vec3) -> Vec3 {
        let n = p.norm();
        // Float equality is intentional and correct here. `f64::sqrt`
        // of a sum of squares returns exactly `0.0` iff every component
        // is exactly `0.0` (IEEE 754 sqrt is exact at 0), so `n == 0.0`
        // triggers iff `p == Vec3::zeros()` exactly — the singleton
        // singularity at the sphere centre. Any other input gives `n`
        // strictly positive and the normalised gradient is well-defined.
        // `Vec3::z()` fallback is unobservable downstream — the centre
        // lives strictly inside the SDF, far from the zero set the
        // mesher actually queries (memo §2 + Decision M).
        #[allow(clippy::float_cmp)]
        let at_singularity = n == 0.0;
        if at_singularity { Vec3::z() } else { p / n }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn eval_at_origin_returns_negative_radius() {
        let s = SphereSdf { radius: 0.1 };
        assert_relative_eq!(s.eval(Vec3::zeros()), -0.1, epsilon = 1e-15);
    }

    #[test]
    fn eval_on_surface_is_zero() {
        let s = SphereSdf { radius: 0.1 };
        assert_relative_eq!(s.eval(Vec3::new(0.1, 0.0, 0.0)), 0.0, epsilon = 1e-15);
        let inv_sqrt3 = 1.0 / 3.0_f64.sqrt();
        assert_relative_eq!(
            s.eval(Vec3::new(0.1 * inv_sqrt3, 0.1 * inv_sqrt3, 0.1 * inv_sqrt3)),
            0.0,
            epsilon = 1e-15,
        );
    }

    #[test]
    fn eval_at_exterior_is_positive() {
        let s = SphereSdf { radius: 0.1 };
        assert_relative_eq!(s.eval(Vec3::new(0.2, 0.0, 0.0)), 0.1, epsilon = 1e-15);
    }

    #[test]
    fn grad_at_axis_aligned_points_is_unit_axis() {
        let s = SphereSdf { radius: 0.1 };
        assert_relative_eq!(s.grad(Vec3::new(0.1, 0.0, 0.0)), Vec3::x(), epsilon = 1e-15);
        assert_relative_eq!(s.grad(Vec3::new(0.0, 0.1, 0.0)), Vec3::y(), epsilon = 1e-15);
        assert_relative_eq!(s.grad(Vec3::new(0.0, 0.0, 0.1)), Vec3::z(), epsilon = 1e-15);
    }

    #[test]
    fn grad_at_origin_returns_documented_z_fallback() {
        let s = SphereSdf { radius: 0.1 };
        assert_relative_eq!(s.grad(Vec3::zeros()), Vec3::z(), epsilon = 1e-15);
    }
}
