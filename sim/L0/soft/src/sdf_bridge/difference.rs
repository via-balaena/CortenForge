//! `DifferenceSdf` — sharp-CSG difference combinator.
//!
//! Per book Part 7 §00 §01 [SDF operations][o] sharp-CSG table:
//!
//! ```text
//! Difference  A \ B  ⇒  φ(p) = max(φ_a(p), -φ_b(p))
//! ```
//!
//! under the `φ < 0` inside convention. A point lies inside `A \ B` iff
//! it is inside `A` and outside `B`. The hollow thick-walled sphere
//! IV-5 scene meshes as `SphereSdf{R_outer} \ SphereSdf{R_cavity}` —
//! interior between the two surfaces, exterior elsewhere.
//!
//! # Two CSG surfaces, one trait
//!
//! `cf_design::Solid` ships the broad CSG kernel (Union, Subtract,
//! Intersect, smoothed variants) over typed `Solid` operands;
//! `cf_design::Solid::from_sdf` (PR3 F5) lifts any `impl Sdf` into a
//! typed `Solid` so `Solid::subtract` itself now covers heterogeneous
//! cases (e.g., a typed sphere differenced against a scan-derived
//! `mesh_sdf::Signed<TriMeshDistance, _>`). `DifferenceSdf` and
//! `Solid::subtract` are both [`Sdf`] implementors of the same sharp
//! difference, retained side-by-side because each surfaces a different
//! consumer role:
//!
//! - `cf_design::Solid::subtract` (with `from_sdf` when
//!   needed) is the **production design surface** for typed-CSG.
//!   Use it when both operands are or can be lifted into `Solid`s and
//!   you want to compose with the rest of cf-design's CSG kernel.
//! - `DifferenceSdf` is the **trait teaching primitive** for sharp-CSG
//!   composition + a `Box<dyn Sdf>` heterogeneous-composition escape
//!   hatch when lifting through `from_sdf` is undesirable. The Tier 1
//!   [`sdf/stress-test`][sdf] module `hollow_shell` teaches the difference
//!   operator through `DifferenceSdf::new(SphereSdf, SphereSdf)` at minimal
//!   surface; [`sdf-bridge/stress-test`'s `solid_to_sim`][r16] uses
//!   `DifferenceSdf<SphereSdf>` as the bit-equality baseline against
//!   typed-Solid `Solid::sphere().subtract()` at HEADLINE A's
//!   `EXACT_TOL = 0.0` cross-crate equivalence anchor.
//!
//! Both surfaces are correct + retained; the choice is "are your
//! operands typed-`Solid`-friendly" (use `Solid::subtract`) vs "are
//! you teaching the trait or anchoring the bridge" (use
//! `DifferenceSdf`).
//!
//! [sdf]: ../../../../../examples/sim-soft/sdf/stress-test/
//! [r16]: ../../../../../examples/sim-soft/sdf-bridge/stress-test/
//!
//! # Differentiability
//!
//! `eval` is `C⁰` everywhere and `C^∞` away from the operand-equality
//! crease (the locus where `φ_a = -φ_b`, geometrically the "edge" where
//! the two operand surfaces meet). `grad` is the sharp chain rule on
//! the active operand:
//!
//! ```text
//! grad(p) = ∇φ_a(p)    if  φ_a(p) >= -φ_b(p)
//!         = -∇φ_b(p)   otherwise
//! ```
//!
//! At the crease the two branches disagree; the impl picks the `a`
//! branch deterministically (the `>=` tie-break) so [`Sdf::grad`] stays
//! a pure function. Smoothed variants (log-sum-exp form per book §01)
//! are the `C^∞` upgrade path when smooth-CSG gradients are needed.
//!
//! [o]: ../../../../../../docs/studies/soft_body_architecture/src/70-sdf-pipeline/00-sdf-primitive/01-operations.md

use crate::Vec3;
use nalgebra::Point3;

use super::sdf::Sdf;

/// Sharp-CSG difference of two SDFs: `A \ B = max(φ_a, -φ_b)`.
///
/// Holds two boxed [`Sdf`] trait objects rather than concrete generics
/// so a single `DifferenceSdf` value can be stored in a uniform
/// composition tree (per book Part 7 §00 §01 closed-algebra
/// commitment) and so heterogeneous operands (e.g., a sphere combined
/// with a scan-derived `mesh_sdf::Signed<TriMeshDistance, _>`) compose
/// at the trait-object boundary without going through
/// `cf_design::Solid::from_sdf`.
///
/// `Send + Sync` follows automatically from `Box<dyn Sdf>`'s supertrait
/// bound.
pub struct DifferenceSdf {
    a: Box<dyn Sdf>,
    b: Box<dyn Sdf>,
}

impl DifferenceSdf {
    /// Construct `A \ B` from two boxed SDFs.
    ///
    /// No validation: `Sdf` trait objects are caller-owned and any
    /// reasonable impl is well-defined for `eval` and `grad` over `R³`.
    /// Degenerate cases (`A` empty, `B` ⊇ `A`) produce empty meshes via
    /// [`MeshingError::EmptyMesh`](crate::sdf_bridge::MeshingError) at
    /// mesh-build time — surfaced loudly downstream rather than at
    /// construction time.
    #[must_use]
    pub fn new(a: Box<dyn Sdf>, b: Box<dyn Sdf>) -> Self {
        Self { a, b }
    }
}

impl Sdf for DifferenceSdf {
    fn eval(&self, p: Point3<f64>) -> f64 {
        let phi_a = self.a.eval(p);
        let phi_b = self.b.eval(p);
        phi_a.max(-phi_b)
    }

    fn grad(&self, p: Point3<f64>) -> Vec3 {
        let phi_a = self.a.eval(p);
        let phi_b = self.b.eval(p);
        if phi_a >= -phi_b {
            self.a.grad(p)
        } else {
            -self.b.grad(p)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sdf_bridge::SphereSdf;
    use approx::assert_relative_eq;

    /// Hollow shell `SphereSdf{R_outer=0.10} \ SphereSdf{R_cavity=0.04}`.
    /// Used across every test below.
    fn hollow_shell() -> DifferenceSdf {
        DifferenceSdf::new(
            Box::new(SphereSdf { radius: 0.10 }),
            Box::new(SphereSdf { radius: 0.04 }),
        )
    }

    #[test]
    fn eval_inside_shell_is_negative() {
        // Probe at r = 0.07 (mid-shell): inside outer (φ_a = -0.03),
        // outside cavity (φ_b = 0.03 ⇒ -φ_b = -0.03). Difference:
        // max(-0.03, -0.03) = -0.03 (the tie-break is exact since
        // probe is equidistant from both surfaces, and either branch
        // returns the same value).
        let s = hollow_shell();
        assert_relative_eq!(s.eval(Point3::new(0.07, 0.0, 0.0)), -0.03, epsilon = 1e-15,);
    }

    #[test]
    fn eval_in_cavity_is_positive() {
        // Probe at r = 0.02 (inside cavity): inside outer
        // (φ_a = -0.08), inside cavity (φ_b = -0.02 ⇒ -φ_b = 0.02).
        // Difference: max(-0.08, 0.02) = 0.02 (inside cavity ⇒ outside
        // shell body).
        let s = hollow_shell();
        assert_relative_eq!(s.eval(Point3::new(0.02, 0.0, 0.0)), 0.02, epsilon = 1e-15,);
    }

    #[test]
    fn eval_exterior_is_positive() {
        // Probe at r = 0.15 (outside outer): outside outer
        // (φ_a = 0.05), outside cavity (φ_b = 0.11 ⇒ -φ_b = -0.11).
        // Difference: max(0.05, -0.11) = 0.05.
        let s = hollow_shell();
        assert_relative_eq!(s.eval(Point3::new(0.15, 0.0, 0.0)), 0.05, epsilon = 1e-15,);
    }

    #[test]
    fn eval_on_outer_surface_is_zero() {
        // r = R_outer = 0.10 exactly: φ_a = 0, -φ_b = -0.06 (cavity
        // surface lives at r = 0.04, so at r = 0.10 the cavity SDF is
        // 0.06, negated to -0.06). max(0, -0.06) = 0.
        let s = hollow_shell();
        assert_relative_eq!(s.eval(Point3::new(0.10, 0.0, 0.0)), 0.0, epsilon = 1e-15,);
    }

    #[test]
    fn eval_on_cavity_surface_is_zero() {
        // r = R_cavity = 0.04 exactly: φ_a = -0.06, -φ_b = 0. max(-0.06, 0)
        // = 0. Cavity surface is the inner zero-set of the hollow body.
        let s = hollow_shell();
        assert_relative_eq!(s.eval(Point3::new(0.04, 0.0, 0.0)), 0.0, epsilon = 1e-15,);
    }

    #[test]
    fn grad_in_outer_active_branch_is_outer_gradient() {
        // Probe at r = 0.09 (in shell, closer to outer): φ_a = -0.01,
        // -φ_b = -0.05. φ_a >= -φ_b ⇒ a-branch active ⇒ grad = ∇φ_a =
        // p / |p| = +x̂.
        let s = hollow_shell();
        assert_relative_eq!(
            s.grad(Point3::new(0.09, 0.0, 0.0)),
            Vec3::x(),
            epsilon = 1e-15,
        );
    }

    #[test]
    fn grad_in_cavity_active_branch_is_negated_cavity_gradient() {
        // Probe at r = 0.05 (in shell, closer to cavity): φ_a = -0.05,
        // -φ_b = -0.01. φ_a < -φ_b ⇒ b-branch active ⇒ grad = -∇φ_b =
        // -(p / |p|) = -x̂. The negated-cavity-gradient branch points
        // INWARD across the difference's interior surface (cavity wall),
        // matching the documented `φ < 0 inside` convention's "outward
        // normal" semantics: at the cavity wall, the body's outward
        // normal points INTO the cavity, i.e. radially inward.
        let s = hollow_shell();
        assert_relative_eq!(
            s.grad(Point3::new(0.05, 0.0, 0.0)),
            -Vec3::x(),
            epsilon = 1e-15,
        );
    }
}
