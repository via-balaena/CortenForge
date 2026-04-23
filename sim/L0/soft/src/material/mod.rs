//! `Material` trait — constitutive-model surface.
//!
//! Four items per spec §14 / Part 11 Ch 01 00-core.md: `energy`,
//! `first_piola`, `tangent`, `validity`. One impl (`NeoHookean`) in
//! scaffold; additive extensions (Mooney-Rivlin, Ogden, corotational,
//! viscoelasticity) land in Phase D per spec §8.

use nalgebra::{Matrix3, SMatrix};

pub mod neo_hookean;

pub use neo_hookean::NeoHookean;

/// Constitutive-model surface: energy density and its first two
/// derivatives in deformation gradient `F`, plus a validity predicate
/// (e.g. `J > 0` for hyperelastic).
pub trait Material: Send + Sync {
    /// Strain-energy density in joules per cubic meter.
    fn energy(&self, f: &Matrix3<f64>) -> f64;

    /// First Piola stress in pascals.
    fn first_piola(&self, f: &Matrix3<f64>) -> Matrix3<f64>;

    /// Material tangent flattened to a 9x9 row-major matrix, with
    /// column-major flattening of the deformation gradient per spec.
    fn tangent(&self, f: &Matrix3<f64>) -> SMatrix<f64, 9, 9>;

    /// Region of deformation-gradient space where the energy is
    /// physically admissible (e.g. `J > 0` for compressible hyperelastic).
    fn validity(&self) -> ValidityDomain;
}

/// Domain of deformation gradients on which the material's energy is
/// well-defined. Unit stub; Phase B populates with bound predicates.
#[derive(Clone, Debug, Default)]
pub struct ValidityDomain;
