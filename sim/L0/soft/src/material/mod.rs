//! `Material` trait — constitutive-model surface.
//!
//! Four items per spec §14 / Part 11 Ch 01 00-core.md: `energy`,
//! `first_piola`, `tangent`, `validity`. Two impls today:
//! [`NeoHookean`] (Phase 4 scaffold; calibrated by `(μ, λ)`) and
//! [`Yeoh`] (Yeoh arc, 2-parameter compressible polynomial calibrated
//! by `(μ, λ, C₂)`). Additional decorators (Mooney-Rivlin
//! corotational, viscoelasticity, HGO anisotropy, thermal coupling)
//! land in Phase H per spec §8.

use nalgebra::{Matrix3, SMatrix};

pub mod material_field;
pub mod neo_hookean;
pub mod silicone_table;
pub mod uniaxial;
pub mod yeoh;

pub use material_field::{BuildableFromField, MaterialField, MaterialFieldKind};
pub use neo_hookean::NeoHookean;
pub use silicone_table::{
    ConstructionSource, MeasuredMaterialError, ShoreInterpolationError, ShoreReading,
    SiliconeMaterial,
};
pub use uniaxial::{
    DeformationMode, UniaxialResponse, fit_yeoh_uniaxial, free_transverse, free_transverse_uniaxial,
};
pub use yeoh::Yeoh;

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

    /// Gradient of the first Piola stress w.r.t. each calibration parameter:
    /// `∂P/∂p_k` for `k` in a fixed, per-material parameter order. The
    /// material-parameter analog of [`Self::tangent`] (which is `∂P/∂F`) — the
    /// ingredient for differentiating a soft solve w.r.t. its material
    /// parameters (`∂r/∂p_k = ∂f_int/∂p_k` assembles from these, keystone S5).
    ///
    /// Default empty: a material exposes no differentiable parameters until it
    /// overrides this (so an un-adapted material yields a *zero* contribution,
    /// never a silently-wrong gradient). [`NeoHookean`] overrides it with
    /// `[∂P/∂μ, ∂P/∂λ]` in `(μ, λ)` order; the closed forms follow from
    /// `P = μ(F − F⁻ᵀ) + λ ln(J) F⁻ᵀ`:
    /// `∂P/∂μ = F − F⁻ᵀ`, `∂P/∂λ = ln(J) · F⁻ᵀ`.
    ///
    /// The returned `Vec`'s length and ordering are the material's parameter
    /// count and convention; consumers index by the same convention the
    /// material documents.
    fn first_piola_param_grad(&self, _f: &Matrix3<f64>) -> Vec<Matrix3<f64>> {
        Vec::new()
    }
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
    ///
    /// Legacy NH-style symmetric bound. The solver gate falls back to
    /// this only when both `max_principal_stretch` and
    /// `min_principal_stretch` are `None`.
    pub max_stretch_deviation: f64,

    /// Tensile principal-stretch cap. `Some(m)` directs the solver gate
    /// to panic when any singular value of `F` exceeds `m`. `None`
    /// defers to the legacy `max_stretch_deviation` symmetric bound.
    /// Yeoh's calibrated `0.8 · λ_break` lands here per arc memo D8.
    pub max_principal_stretch: Option<f64>,

    /// Compressive principal-stretch cap. `Some(n)` directs the solver
    /// gate to panic when any singular value of `F` falls below `n`.
    /// `None` defers to the legacy `max_stretch_deviation` symmetric
    /// bound (or to `det F > 0` inversion when the tensile bound is
    /// `Some`, since the gate routes through the per-principal-stretch
    /// flavor on either bound's presence).  Yeoh's `0.20` (H4-2-A
    /// research-informed default, was 0.30 pre-H4-2-A) lands here per
    /// arc memo D8 + `docs/CANDIDATE_H4_COMPRESSION_RESEARCH.md` —
    /// but note H4-2-C drops this slot to `None` for per-tet `Yeoh`s
    /// built via [`crate::MaterialField::sample_yeoh`], so in the
    /// cf-device-design FEM path the compressive cap is currently
    /// dormant (preserved for future Option B re-enable per
    /// `docs/CANDIDATE_H4_FALSIFICATION_BOOKMARK.md` §5).
    pub min_principal_stretch: Option<f64>,

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
