//! Linear tetrahedron (4 nodes, 1 Gauss point) — constant strain.
//!
//! Phase B fills in barycentric shape functions and the single midpoint
//! Gauss quadrature at `xi = (1/4, 1/4, 1/4)`, `w = 1/6`.

use nalgebra::{SMatrix, SVector};

use super::Element;
use crate::Vec3;

/// Linear tetrahedron element. Constant strain, 12 DOFs total.
#[derive(Clone, Copy, Debug, Default)]
pub struct Tet4;

impl Element<4, 1> for Tet4 {
    fn shape_functions(&self, _xi: Vec3) -> SVector<f64, 4> {
        unimplemented!("skeleton phase 2")
    }

    fn shape_gradients(&self, _xi: Vec3) -> SMatrix<f64, 4, 3> {
        unimplemented!("skeleton phase 2")
    }

    fn gauss_points(&self) -> [(Vec3, f64); 1] {
        unimplemented!("skeleton phase 2")
    }
}
