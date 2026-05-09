//! `MaterialField` — per-parameter typed-field aggregator backing
//! [`NeoHookean`] or [`Yeoh`].
//!
//! Per Part 2 §09 §00 [`00-sdf-valued.md`][s] "`MaterialField` aggregates
//! per-parameter typed fields" + "Trait-surface contract". Originally
//! Phase 4-monomorphic to [`NeoHookean`]; the F4.0a Yeoh-arc commit
//! widens the same-type aggregator to also build [`Yeoh`] from a
//! `(μ, C₂, λ)` triplet of fields per arc memo D10 ("`MaterialField` gets
//! parallel constructors; row picks one material model"). Internal
//! enum-tagged storage; no per-tet `Box<dyn Material>` dispatch.
//!
//! Two construction families:
//!
//! - **NH constructors** ([`MaterialField::uniform`],
//!   [`MaterialField::skeleton_default`], [`MaterialField::from_fields`])
//!   produce the legacy NH variant. Existing call sites compile
//!   unchanged.
//! - **Yeoh constructor** ([`MaterialField::from_yeoh_fields`])
//!   produces the Yeoh variant; row 23+ Yeoh consumers use this.
//!
//! Sampling is variant-typed: [`MaterialField::sample`] returns
//! [`NeoHookean`] (panics on Yeoh variant);
//! [`MaterialField::sample_yeoh`] returns [`Yeoh`] (panics on NH
//! variant). Mesh constructors pick the right one based on the
//! parameterized material type per F4.0b.
//!
//! Field-to-scalar conversion happens at element construction (and
//! warm-restart from re-mesh); the Newton hot path sees per-element
//! scalar materials, never this aggregator.
//!
//! Determinism: stateless beyond the boxed handles; sampling is a pure
//! function of `x_ref` whenever the slot impls are deterministic
//! (Phase 2 invariant I-5 carry-forward per scope memo Decision N).
//! `Send + Sync` follows from the [`Field`] supertrait bound on every
//! `Box<dyn Field<f64>>` slot.
//!
//! Validity-domain checks belong at solver step start (commit 12,
//! IV-7) per scope memo Decision Q, not at field-sample time;
//! `MaterialField` is a pure compositional aggregator and propagates
//! `NaN` per the BF-7 sentinel pattern.
//!
//! [s]: ../../../../docs/studies/soft_body_architecture/src/20-materials/09-spatial-fields/00-sdf-valued.md

use super::{NeoHookean, Yeoh};
use crate::Vec3;
use crate::field::{ConstantField, Field};
use crate::sdf_bridge::Sdf;

/// Aggregator over the scalar Lamé / Yeoh fields backing a per-element
/// [`NeoHookean`] or [`Yeoh`] material.
///
/// Internal enum-tagged storage; the variant is fixed at construction
/// time:
///
/// - NH variant (μ, λ slots) via [`MaterialField::uniform`],
///   [`MaterialField::skeleton_default`], [`MaterialField::from_fields`].
/// - Yeoh variant (μ, C₂, λ slots) via
///   [`MaterialField::from_yeoh_fields`].
///
/// **Interface SDF (commit 12, IV-6 per Part 7 §02 §01).** Optional
/// slot carrying the SDF whose zero set drives the material blend
/// (e.g., the `Box<dyn Sdf>` inside a
/// [`BlendedScalarField`](crate::field::BlendedScalarField) used for
/// the `mu` or `lambda` field). When set via
/// [`MaterialField::with_interface_sdf`], mesh constructors flag tets
/// whose centroid lands within one mean-edge-length of the SDF zero
/// per the book's `|φ(x_c)| < L_e` rule, exposed via
/// [`crate::Mesh::interface_flags`]. Common to both variants.
pub struct MaterialField {
    inner: MaterialFieldInner,
    interface_sdf: Option<Box<dyn Sdf>>,
}

/// Variant-tagged backing storage. Pub-crate so mesh constructors can
/// dispatch on the variant; not a stable public API surface.
pub(crate) enum MaterialFieldInner {
    NeoHookean {
        mu: Box<dyn Field<f64>>,
        lambda: Box<dyn Field<f64>>,
    },
    Yeoh {
        mu: Box<dyn Field<f64>>,
        c2: Box<dyn Field<f64>>,
        lambda: Box<dyn Field<f64>>,
    },
}

/// Coarse identifier for which material model a [`MaterialField`]
/// builds. Used by mesh constructors to verify the field's variant
/// matches the mesh's parameterized material type at build time.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum MaterialFieldKind {
    /// NH — `(μ, λ)` slots; sampled into [`NeoHookean`].
    NeoHookean,
    /// Yeoh — `(μ, C₂, λ)` slots; sampled into [`Yeoh`].
    Yeoh,
}

impl MaterialField {
    /// Construct a uniform-everywhere NH material from two scalar Lamé
    /// parameters. Both slots become
    /// [`ConstantField<f64>`](crate::field::ConstantField); every
    /// `sample(x_ref)` returns [`NeoHookean`] built via
    /// `NeoHookean::from_lame(mu, lambda)`.
    #[must_use]
    pub fn uniform(mu: f64, lambda: f64) -> Self {
        Self::from_fields(
            Box::new(ConstantField::new(mu)),
            Box::new(ConstantField::new(lambda)),
        )
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

    /// Construct an NH variant from two heterogeneous
    /// [`Field<f64>`](crate::field::Field) impls. The two slots sample
    /// independently; either or both can be `LayeredScalarField`,
    /// `BlendedScalarField`, or a Phase-H impl.
    #[must_use]
    pub fn from_fields(mu: Box<dyn Field<f64>>, lambda: Box<dyn Field<f64>>) -> Self {
        Self {
            inner: MaterialFieldInner::NeoHookean { mu, lambda },
            interface_sdf: None,
        }
    }

    /// Construct a Yeoh variant from three heterogeneous
    /// [`Field<f64>`](crate::field::Field) impls (μ, C₂, λ). Yeoh's
    /// `C₁ = μ / 2` is derived; the field doesn't store `C₁`
    /// separately. Per arc memo D10 — row picks one material model
    /// for all its tets, no in-row mixing.
    #[must_use]
    pub fn from_yeoh_fields(
        mu: Box<dyn Field<f64>>,
        c2: Box<dyn Field<f64>>,
        lambda: Box<dyn Field<f64>>,
    ) -> Self {
        Self {
            inner: MaterialFieldInner::Yeoh { mu, c2, lambda },
            interface_sdf: None,
        }
    }

    /// Attach an interface SDF to this field for IV-6 interface-tet
    /// flagging (Part 7 §02 §01).
    ///
    /// The supplied SDF's zero set is the "material-interface"
    /// boundary the [`crate::Mesh::interface_flags`] population pass
    /// thresholds against per the book's `|φ(x_c)| < L_e` rule. For
    /// the canonical "stiff skin over soft core" pattern this is the
    /// `Box<dyn Sdf>` driving a [`crate::field::BlendedScalarField`]
    /// used for `mu` or `lambda`; for other compositions the caller
    /// passes the SDF whose zero set the flag should track.
    ///
    /// Builder method (`self`-by-value, returns `Self`) so existing
    /// constructor call sites stay untouched: a graded-with-flag scene
    /// reads
    /// `MaterialField::from_fields(mu, lambda).with_interface_sdf(sdf)`.
    #[must_use]
    pub fn with_interface_sdf(mut self, sdf: Box<dyn Sdf>) -> Self {
        self.interface_sdf = Some(sdf);
        self
    }

    /// Borrow the interface SDF if one was attached via
    /// [`MaterialField::with_interface_sdf`]; `None` otherwise.
    ///
    /// Read by the [`crate::Mesh`] impls' constructors at mesh-build
    /// time to populate the per-tet [`crate::Mesh::interface_flags`]
    /// vector. Not read by the Newton hot path (Decision K
    /// diagnostic-only).
    #[must_use]
    pub fn interface_sdf(&self) -> Option<&dyn Sdf> {
        self.interface_sdf.as_deref()
    }

    /// Coarse identifier for the variant this field carries. Mesh
    /// constructors check this against their parameterized material
    /// type and refuse to build a `Mesh<NeoHookean>` from a Yeoh
    /// field (or vice versa).
    #[must_use]
    pub const fn kind(&self) -> MaterialFieldKind {
        match &self.inner {
            MaterialFieldInner::NeoHookean { .. } => MaterialFieldKind::NeoHookean,
            MaterialFieldInner::Yeoh { .. } => MaterialFieldKind::Yeoh,
        }
    }

    /// Sample the NH variant at `x_ref` and return a fresh per-element
    /// [`NeoHookean`].
    ///
    /// Per Part 2 §09 §00's "Trait-surface contract" paragraph this is
    /// the field-to-scalar conversion the element-assembly pass calls
    /// at element construction (and warm restart from re-mesh); the
    /// Newton hot path sees the returned scalar [`NeoHookean`], not
    /// this aggregator.
    ///
    /// # Panics
    ///
    /// Panics if this field was constructed via
    /// [`MaterialField::from_yeoh_fields`] — call
    /// [`MaterialField::sample_yeoh`] for Yeoh variants. The mesh
    /// constructors gate variant selection at build time so no
    /// hot-path call site reaches this branch.
    #[must_use]
    #[allow(clippy::panic)]
    pub fn sample(&self, x_ref: Vec3) -> NeoHookean {
        match &self.inner {
            MaterialFieldInner::NeoHookean { mu, lambda } => {
                NeoHookean::from_lame(mu.sample(x_ref), lambda.sample(x_ref))
            }
            MaterialFieldInner::Yeoh { .. } => {
                panic!(
                    "MaterialField::sample expects NH variant; this field is Yeoh — use sample_yeoh"
                )
            }
        }
    }

    /// Sample the Yeoh variant at `x_ref` and return a fresh per-element
    /// [`Yeoh`]. Validity bounds default to `None` on the produced
    /// `Yeoh` — per-anchor bounds are wired in by the row author via
    /// the [`crate::SiliconeMaterial::to_yeoh`] path, not the
    /// `MaterialField` aggregator (which only carries the `(μ, C₂, λ)`
    /// scalar fields).
    ///
    /// # Panics
    ///
    /// Panics if this field was constructed via
    /// [`MaterialField::from_fields`] / [`MaterialField::uniform`] /
    /// [`MaterialField::skeleton_default`] — call
    /// [`MaterialField::sample`] for NH variants.
    #[must_use]
    #[allow(clippy::panic)]
    pub fn sample_yeoh(&self, x_ref: Vec3) -> Yeoh {
        match &self.inner {
            MaterialFieldInner::Yeoh { mu, c2, lambda } => {
                Yeoh::from_lame_and_c2(mu.sample(x_ref), lambda.sample(x_ref), c2.sample(x_ref))
            }
            MaterialFieldInner::NeoHookean { .. } => {
                panic!(
                    "MaterialField::sample_yeoh expects Yeoh variant; this field is NH — use sample"
                )
            }
        }
    }
}
