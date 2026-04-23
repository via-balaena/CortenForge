//! Compressible Neo-Hookean material — closed-form `P` and tangent.
//!
//! Phase B fills in bodies per Part 2 Ch 04 00-energy / 01-tangent.

use nalgebra::{Matrix3, SMatrix};

use super::{Material, ValidityDomain};

/// Compressible Neo-Hookean energy and its derivatives. Skeleton scene
/// parameters per spec §2: `mu = 1e5`, `lambda = 4e5`. See Part 2 Ch 04
/// for the closed-form expressions Phase B fills in.
#[derive(Clone, Debug)]
pub struct NeoHookean {
    /// Shear modulus \(\mu\) (Pa).
    pub mu: f64,
    /// First Lamé parameter \(\lambda\) (Pa).
    pub lambda: f64,
}

impl Material for NeoHookean {
    fn energy(&self, _f: &Matrix3<f64>) -> f64 {
        unimplemented!("skeleton phase 2")
    }

    fn first_piola(&self, _f: &Matrix3<f64>) -> Matrix3<f64> {
        unimplemented!("skeleton phase 2")
    }

    fn tangent(&self, _f: &Matrix3<f64>) -> SMatrix<f64, 9, 9> {
        unimplemented!("skeleton phase 2")
    }

    fn validity(&self) -> ValidityDomain {
        unimplemented!("skeleton phase 2")
    }
}
