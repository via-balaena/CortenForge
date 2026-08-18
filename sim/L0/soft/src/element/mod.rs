//! `Element` trait — per-element shape functions and quadrature.
//!
//! Const-generic in node count `N` and Gauss-point count `G` so per-tet
//! local assembly uses stack-allocated matrices. Ships `Tet4` (N=4, G=1) and
//! `Tet10` (N=10, G=4); the forward solver frees the Tet10 midside DOFs from
//! ladder rung 3b, with the multi-Gauss-point stiffness (rung 4) and the
//! differentiable adjoint (rung 7) following in later Tet10-ladder rungs.

use nalgebra::{SMatrix, SVector};

use crate::Vec3;

pub mod tet10;
pub mod tet4;
pub mod validity;

pub use tet4::Tet4;
pub use tet10::{TET10_EDGE_NODES, Tet10};
pub use validity::{
    DET_J_COEFFS, RestValidity, ValidityBar, certify_rest, rest_det_j_coefficients,
};

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

    /// Signed determinant of the rest-frame Jacobian `J(ξ_q) = x_refᵀ · ∇_ξN(ξ_q)`
    /// at each of the `G` Gauss points, where row `a` of `x_ref` is the rest
    /// position of node `a`.
    ///
    /// This is the *same* `J` the curved per-Gauss-point stiffness geometry
    /// assembles in the solver (`curved_gauss_geometry`), which then takes
    /// `|detJ|` — so a non-positive entry here marks an inverted or degenerate
    /// element at that Gauss point, the sign flip the solver's `.abs()` would
    /// silently hide. A mesher that moves nodes (e.g. projecting boundary
    /// midsides onto an SDF) uses this to reject a placement before it reaches
    /// the assembler. For a straight-edged element `J` is constant, so all `G`
    /// entries are equal.
    fn rest_jacobian_dets(&self, x_ref: &SMatrix<f64, N, 3>) -> [f64; G] {
        let gps = self.gauss_points();
        std::array::from_fn(|q| {
            let grad_xi = self.shape_gradients(gps[q].0);
            (x_ref.transpose() * grad_xi).determinant()
        })
    }

    /// **Prove** — or refute — that the isoparametric map from the reference
    /// simplex to the node matrix `x` preserves orientation over the WHOLE
    /// element.
    ///
    /// The certifying counterpart of [`Self::rest_jacobian_dets`]. That method
    /// evaluates `det J` at the `G` points quadrature happens to use; this one
    /// answers for every point of the element. [`RestValidity::Certified`] is a
    /// proof, [`RestValidity::Violated`] carries an **evaluated** witness point,
    /// and [`RestValidity::Undetermined`] is neither and must be read as failure
    /// — absence of a proof is not a proof.
    ///
    /// ★ `x` is **any** node matrix, not only the rest one, and the deformed
    /// configuration is the reason this sits on the trait. Writing `J_def(ξ) =
    /// x_defᵀ ∇N(ξ)`, the deformation gradient is `F = J_def · J_rest⁻¹`, so
    ///
    /// ```text
    ///     det F = det J_def / det J_rest
    /// ```
    ///
    /// and `J_def` is affine in `ξ` in exactly the way `J_rest` is. `det F > 0`
    /// everywhere therefore holds **iff both determinants are positive
    /// everywhere**, each of which this method decides. `F` itself is a ratio
    /// and no polynomial bound applies to it directly; the bound is never
    /// applied to `F`.
    ///
    /// ⚠ Both halves are load-bearing. At rest `det F ≡ 1`, so an element that
    /// is folded in the REST configuration is invisible to any gate reading
    /// `det F` alone — which is why the solver certifies the rest mesh once at
    /// construction and the deformed state at each step boundary, rather than
    /// treating the deformed certificate as sufficient on its own.
    fn certify_orientation(&self, x: &SMatrix<f64, N, 3>) -> RestValidity;
}
