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

use super::Element;
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
}
