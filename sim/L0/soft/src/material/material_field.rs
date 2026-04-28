//! `MaterialField` — per-parameter typed-field aggregator backing
//! [`NeoHookean`].
//!
//! Per Part 2 §09 §00 [`00-sdf-valued.md`][s] "`MaterialField` aggregates
//! per-parameter typed fields" + "Trait-surface contract": holds one
//! `Box<dyn Field<f64>>` per scalar Lamé parameter slot (`μ` and `λ`
//! for Phase 4's [`NeoHookean`]-only specialization per scope memo
//! Decision G); `sample(x_ref)` produces a per-element `NeoHookean`
//! instance with scalar parameters. Field-to-scalar conversion happens
//! at element construction (and warm-restart from re-mesh); the Newton
//! hot path sees per-element scalar materials, never this aggregator.
//!
//! Phase 4 monomorphizes against [`NeoHookean`] directly rather than
//! generic `M: Material` — decorator composition (HGO, Mooney-Rivlin,
//! viscoelastic) is Phase H per Decision G, and the trait surface
//! preserves the drop-in path. The constant-everywhere case is
//! [`MaterialField::uniform`], which slots two
//! [`ConstantField<f64>`](crate::field::ConstantField) handles; the
//! graded case uses [`MaterialField::from_fields`] with any
//! [`Field<f64>`](crate::field::Field) impl
//! ([`LayeredScalarField`](crate::field::LayeredScalarField),
//! [`BlendedScalarField`](crate::field::BlendedScalarField), or a
//! Phase-H `T = f64` impl).
//!
//! Determinism: stateless beyond the two boxed handles; `sample` is a
//! pure function of `x_ref` whenever both slot impls are deterministic
//! (Phase 2 invariant I-5 carry-forward per scope memo Decision N).
//! `Send + Sync` follows automatically: `Box<dyn Field<f64>>` is
//! `Send + Sync` via the [`Field`] supertrait
//! bound (commit 1).
//!
//! Validity-domain checks belong at solver step start (commit 12,
//! IV-7) per scope memo Decision Q, not at field-sample time;
//! `MaterialField` is a pure compositional aggregator and propagates
//! `NaN` per the BF-7 sentinel pattern.
//!
//! [s]: ../../../../docs/studies/soft_body_architecture/src/20-materials/09-spatial-fields/00-sdf-valued.md

use super::NeoHookean;
use crate::Vec3;
use crate::field::{ConstantField, Field};

/// Aggregator over the two scalar Lamé fields backing a per-element
/// [`NeoHookean`].
///
/// Each [`MaterialField::sample`] evaluates `mu` and `lambda` at the
/// reference-space probe and returns a fresh [`NeoHookean`] via
/// `NeoHookean::from_lame(mu, lambda)`.
///
/// Construction:
///
/// - [`MaterialField::uniform`] for the degenerate case (both slots
///   become `ConstantField<f64>`); pre-Phase-4 invariant tests run
///   through this path bit-equal per scope memo Decision P / IV-1
///   regression net.
/// - [`MaterialField::from_fields`] for graded cases (any
///   `Field<f64>` impl in either or both slots —
///   `LayeredScalarField`, `BlendedScalarField`, or a future Phase-H
///   impl).
pub struct MaterialField {
    mu: Box<dyn Field<f64>>,
    lambda: Box<dyn Field<f64>>,
}

impl MaterialField {
    /// Construct a uniform-everywhere material from two scalar Lamé
    /// parameters. Both slots become
    /// [`ConstantField<f64>`](crate::field::ConstantField); every
    /// `sample(x_ref)` returns [`NeoHookean`] built via
    /// `NeoHookean::from_lame(mu, lambda)`.
    #[must_use]
    pub fn uniform(mu: f64, lambda: f64) -> Self {
        Self {
            mu: Box::new(ConstantField::new(mu)),
            lambda: Box::new(ConstantField::new(lambda)),
        }
    }

    /// IV-1 baseline material — `MaterialField::uniform(1.0e5, 4.0e5)`.
    ///
    /// Pinned Lamé pair: μ = 1.0×10⁵ Pa, λ = 4.0×10⁵ Pa (Ecoflex 00-30
    /// compressible regime, ν = 0.4 — same constants the IV-1, IV-2,
    /// and IV-3 invariant tests anchor on, and the implicit baseline
    /// that pre-Phase-4 SDF tests ran through the solver's
    /// hardcoded `NeoHookean::from_lame(1e5, 4e5)`). Constructor exists
    /// so the SDF mesher's `MeshingHints::material_field = None` path
    /// has a single named source of truth for the synthesized default
    /// and so `grep skeleton_default` surfaces every consumer.
    #[must_use]
    pub fn skeleton_default() -> Self {
        Self::uniform(1.0e5, 4.0e5)
    }

    /// Construct from two heterogeneous [`Field<f64>`](crate::field::Field)
    /// impls. The two slots sample independently; either or both can
    /// be `LayeredScalarField`, `BlendedScalarField`, or a Phase-H
    /// impl.
    #[must_use]
    pub fn from_fields(mu: Box<dyn Field<f64>>, lambda: Box<dyn Field<f64>>) -> Self {
        Self { mu, lambda }
    }

    /// Sample both slots at `x_ref` and return a fresh per-element
    /// [`NeoHookean`].
    ///
    /// Per Part 2 §09 §00's "Trait-surface contract" paragraph this is
    /// the field-to-scalar conversion the element-assembly pass calls
    /// at element construction (and warm restart from re-mesh); the
    /// Newton hot path sees the returned scalar [`NeoHookean`], not
    /// this aggregator.
    #[must_use]
    pub fn sample(&self, x_ref: Vec3) -> NeoHookean {
        let mu = self.mu.sample(x_ref);
        let lambda = self.lambda.sample(x_ref);
        NeoHookean::from_lame(mu, lambda)
    }
}
