//! Linear tetrahedron (4 nodes, 1 Gauss point) — constant strain.
//!
//! Barycentric shape functions on the reference tet with parametric
//! coordinates `(ξ_1, ξ_2, ξ_3)` on the simplex `ξ_i ≥ 0, Σ ξ_i ≤ 1`:
//!
//! - `N_0 = 1 − ξ_1 − ξ_2 − ξ_3`, `N_1 = ξ_1`, `N_2 = ξ_2`, `N_3 = ξ_3`
//!
//! Gradients `∂N_a / ∂ξ_b` are constant on the element and form the
//! rows of the returned `SMatrix<f64, 4, 3>` — row `a` is node `a`'s
//! gradient with respect to all three parametric axes. One-point
//! quadrature at the centroid `ξ = (1/4, 1/4, 1/4)` with weight
//! `1/6` = reference-tet volume; exact for the constant integrands
//! Tet4 produces per Part 3 Ch 00 00-tet4.md §"No quadrature for Tet4".

use nalgebra::{SMatrix, SVector};

use super::{Element, RestValidity};
use crate::Vec3;

/// Linear tetrahedron element. Constant strain, 12 DOFs total.
#[derive(Clone, Copy, Debug, Default)]
pub struct Tet4;

impl Element<4, 1> for Tet4 {
    fn shape_functions(&self, xi: Vec3) -> SVector<f64, 4> {
        SVector::<f64, 4>::new(1.0 - xi.x - xi.y - xi.z, xi.x, xi.y, xi.z)
    }

    // Gradients are independent of `xi` (shape functions are linear in
    // barycentric coordinates). Row `a` = `∂N_a / ∂ξ`.
    fn shape_gradients(&self, _xi: Vec3) -> SMatrix<f64, 4, 3> {
        SMatrix::<f64, 4, 3>::new(
            -1.0, -1.0, -1.0, // ∂N_0 / ∂ξ
            1.0, 0.0, 0.0, // ∂N_1 / ∂ξ
            0.0, 1.0, 0.0, // ∂N_2 / ∂ξ
            0.0, 0.0, 1.0, // ∂N_3 / ∂ξ
        )
    }

    fn gauss_points(&self) -> [(Vec3, f64); 1] {
        // Centroid, weight = 1/6 = volume of the reference tet.
        [(Vec3::new(0.25, 0.25, 0.25), 1.0 / 6.0)]
    }

    /// Exact with no machinery. Tet4's map is affine, so `det J` is the same
    /// constant at every point of the element and the single centroid
    /// evaluation already answers for the whole of it — there is no gap between
    /// sampling and certifying here, which is the property `Tet10` lacks.
    ///
    /// The margin is `1.0` because every Bernstein coefficient of a constant
    /// equals that constant, so the smallest is the largest: this is the widest
    /// margin the type can express, and it is the honest one.
    ///
    /// This impl exists so the solver's validity gate can be written once for
    /// both element types, not because Tet4 needed a certificate.
    fn certify_orientation(&self, x: &SMatrix<f64, 4, 3>) -> RestValidity {
        let value = self.rest_jacobian_dets(x)[0];
        // `is_finite` as well as `> 0.0`: every comparison against NaN is
        // false, so a bare `> 0.0` would fall through to `Violated` correctly
        // here — but stating finiteness keeps this impl's contract identical to
        // Tet10's, where the fold over coefficients cannot infer it.
        if value.is_finite() && value > 0.0 {
            RestValidity::Certified { margin: 1.0 }
        } else {
            RestValidity::Violated {
                at: self.gauss_points()[0].0,
                value,
            }
        }
    }
}
