//! `impl Sdf for Solid` — cf-design's binding to the workspace-wide
//! signed-distance-function trait.
//!
//! The [`Sdf`] trait itself lives in cf-geometry (the L0 geometric
//! kernel); this module hosts cf-design's canonical implementor
//! [`Solid`]. Downstream crates and user code add more by writing
//! their own `impl Sdf for ...` against `cf_geometry::Sdf`. Consumers
//! query SDFs through the trait without naming the concrete source.
//!
//! The Box/Arc forwarding blankets live next to the trait in
//! cf-geometry. The mesh-sdf adapter impls (`impl Sdf for Signed<D, S>`
//! and `impl Sdf for CachedGridSdf`) now live in mesh-sdf alongside
//! their implementing types — orphan-rule-clean post-migration.

use cf_geometry::Sdf;
use nalgebra::{Point3, Vector3};

use crate::Solid;

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
}
