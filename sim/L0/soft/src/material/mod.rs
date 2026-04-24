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

    /// Fourth-order material tangent `∂P/∂F` as a 9×9 matrix, with both
    /// `P_ij` (rows) and `F_kl` (columns) flattened column-major:
    /// `tangent[(i + 3j, k + 3l)] = ∂P_ij / ∂F_kl`. Convention ratified by
    /// scope §13 BF-5 pending book edit of Part 2 Ch 04 01-tangent.md.
    fn tangent(&self, f: &Matrix3<f64>) -> SMatrix<f64, 9, 9>;

    /// Region of deformation-gradient space where the energy is
    /// physically admissible (e.g. `J > 0` for compressible hyperelastic).
    fn validity(&self) -> ValidityDomain;
}

/// Domain of deformation gradients on which the material's energy is
/// physically admissible.
///
/// Shape follows Part 2 Ch 00 02-validity.md: six slots covering stretch,
/// rotation, Poisson, temperature, strain-rate, and element-inversion
/// handling. Decorators (`Thermal<M>`, `Viscoelastic<M>`, mixed-u-p,
/// F-bar) fill in the `temperature_range` and `strain_rate_range` slots
/// and widen the `poisson_range` bound. `#[non_exhaustive]` matches the
/// `InversionHandling` precedent — future cross-domain concerns (e.g.
/// residual stress, piezo coupling) land as additional slots without
/// breaking downstream pattern matches.
#[derive(Clone, Debug)]
#[non_exhaustive]
pub struct ValidityDomain {
    /// Maximum principal-stretch deviation `|λ − 1|` at which the
    /// declared error bound against the next-more-accurate reference
    /// law still holds.
    pub max_stretch_deviation: f64,

    /// Maximum rotation angle in radians. `f64::INFINITY` for
    /// rotation-invariant laws.
    pub max_rotation: f64,

    /// Allowed Poisson ratio range. Upper bound below 0.5 for laws with
    /// no near-incompressibility cure; widened when wrapped by a
    /// mixed-u-p or F-bar locking-fix decorator from Part 2 Ch 05.
    pub poisson_range: (f64, f64),

    /// Allowed temperature range when wrapped by `Thermal<M>`. `None`
    /// for isothermal laws.
    pub temperature_range: Option<(f64, f64)>,

    /// Strain-rate regime for rate-dependent laws. `None` for
    /// rate-independent.
    pub strain_rate_range: Option<(f64, f64)>,

    /// Behavior under element inversion (`det F ≤ 0`).
    pub inversion: InversionHandling,
}

/// How a `Material` impl responds to element inversion (`det F ≤ 0`).
///
/// Phase A names only `RequireOrientation`, the policy hyperelastic laws
/// (`NeoHookean`, `MooneyRivlin`, `Ogden`) share per Part 2 Ch 04: rely
/// on the IPC barrier to keep `det F > 0`, panic if an inversion reaches
/// constitutive evaluation. Additional variants (`Barrier`, `OptIn`,
/// etc.) land with the impls that first require them.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum InversionHandling {
    /// Evaluation requires `det F > 0`; the impl panics on non-positive
    /// `J` and surfaces as an IPC-barrier failure downstream rather
    /// than a constitutive bug.
    RequireOrientation,
}
