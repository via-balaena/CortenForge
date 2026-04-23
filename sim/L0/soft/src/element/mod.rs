//! `Element` trait — per-element shape functions and quadrature.
//!
//! Const-generic in node count `N` and Gauss-point count `G` so per-tet
//! local assembly uses stack-allocated matrices. Skeleton ships `Tet4`
//! (N=4, G=1). Tet10 (N=10, G=4) is additive in Phase H per spec §8.

use nalgebra::{SMatrix, SVector};

use crate::Vec3;

pub mod tet4;

pub use tet4::Tet4;

/// Per-element geometric surface: shape functions, their gradients,
/// and Gauss-point weights. `N` = nodes per element, `G` = Gauss points.
pub trait Element<const N: usize, const G: usize>: Send + Sync {
    /// Shape functions evaluated at parametric point `xi`.
    fn shape_functions(&self, xi: Vec3) -> SVector<f64, N>;

    /// Shape-function gradients with respect to `xi`.
    fn shape_gradients(&self, xi: Vec3) -> SMatrix<f64, N, 3>;

    /// Gauss-point parametric locations and weights for this element.
    fn gauss_points(&self) -> [(Vec3, f64); G];

    /// Degrees of freedom for this element (3 per node by default).
    fn n_dof(&self) -> usize {
        3 * N
    }
}
