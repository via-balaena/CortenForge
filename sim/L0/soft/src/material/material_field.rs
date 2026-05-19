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
//! - **Yeoh constructors** ([`MaterialField::from_yeoh_fields`] —
//!   legacy 3-arg, bounds drop to `None`;
//!   [`MaterialField::from_yeoh_fields_with_bounds`] — 5-arg,
//!   threads the calibrated per-anchor `(max, min)` principal-
//!   stretch caps into `bounds`).  Note that
//!   [`MaterialField::sample_yeoh`] routes the 5-arg path through
//!   [`Yeoh::with_max_principal_stretch_only`] per H4-2-C, so the
//!   compressive value reaches `bounds` but is dropped at sample
//!   time — see `docs/CANDIDATE_H4_FALSIFICATION_BOOKMARK.md` §5.
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

use super::{Material, NeoHookean, Yeoh};
use crate::Vec3;
use crate::field::{ConstantField, Field};
use crate::mesh::VertexId;
use crate::sdf_bridge::Sdf;

/// Aggregator over the scalar Lamé / Yeoh fields backing a per-element
/// [`NeoHookean`] or [`Yeoh`] material.
///
/// Internal enum-tagged storage; the variant is fixed at construction
/// time:
///
/// - NH variant (μ, λ slots) via [`MaterialField::uniform`],
///   [`MaterialField::skeleton_default`], [`MaterialField::from_fields`].
/// - Yeoh variant (μ, C₂, λ slots, optional `(max, min)` principal-
///   stretch caps; `min` is preserved in `bounds` but dropped at
///   [`MaterialField::sample_yeoh`] time under H4-2-C — see
///   `docs/CANDIDATE_H4_FALSIFICATION_BOOKMARK.md` §5) via
///   [`MaterialField::from_yeoh_fields`] (bounds-less) or
///   [`MaterialField::from_yeoh_fields_with_bounds`] (calibrated).
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
        /// Optional per-tet asymmetric principal-stretch caps
        /// (memo D8). `None` → legacy path: the produced [`Yeoh`]
        /// inherits `(max_principal_stretch, min_principal_stretch)
        /// = (None, None)` and the solver gate falls back to
        /// `max_stretch_deviation` (NH-style symmetric). `Some` →
        /// the `max_principal_stretch` field is sampled + routed
        /// through [`Yeoh::with_max_principal_stretch_only`] per
        /// H4-2-C (`docs/CANDIDATE_H4_FALSIFICATION_BOOKMARK.md`
        /// §5); the produced `Yeoh` carries `(Some(max), None)`.
        /// The `min_principal_stretch` field stays threaded but
        /// is dropped at [`MaterialField::sample_yeoh`] time —
        /// preserved for future Option B (Phase H F-bar /
        /// mixed-u-p decorator) re-enable by flipping the
        /// `sample_yeoh` call back to
        /// [`Yeoh::with_principal_stretch_bounds`].
        bounds: Option<YeohBoundsFields>,
    },
}

/// Paired principal-stretch cap fields for the Yeoh variant.
/// Constructed by [`MaterialField::from_yeoh_fields_with_bounds`];
/// every [`crate::SiliconeMaterial`] anchor carries both bounds so
/// both fields are populated from the catalog.  Under H4-2-C only
/// the `max` field reaches the solver gate (see
/// [`MaterialFieldInner::Yeoh::bounds`] doc); the `min` field is
/// load-bearing for future Option B re-enable.
pub(crate) struct YeohBoundsFields {
    pub max_principal_stretch: Box<dyn Field<f64>>,
    pub min_principal_stretch: Box<dyn Field<f64>>,
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
    ///
    /// Bounds-less legacy path: the produced per-tet [`Yeoh`] has
    /// `validity().max_principal_stretch == None` AND
    /// `validity().min_principal_stretch == None`, so the solver
    /// gate falls back to the symmetric `max_stretch_deviation`
    /// bound. Callers with calibrated per-anchor caps (e.g.,
    /// every [`crate::SiliconeMaterial`] in
    /// `silicone_table.rs`) prefer
    /// [`MaterialField::from_yeoh_fields_with_bounds`] — the
    /// 5-arg path threads `0.8 · λ_break` (tensile) and `0.20`
    /// (compressive, family-uniform) through the aggregator;
    /// the tensile cap reaches the per-tet validity gate at sample
    /// time, and the compressive cap stays in `bounds` for future
    /// Option B re-enable per H4-2-C
    /// (`docs/CANDIDATE_H4_FALSIFICATION_BOOKMARK.md` §5).
    #[must_use]
    pub fn from_yeoh_fields(
        mu: Box<dyn Field<f64>>,
        c2: Box<dyn Field<f64>>,
        lambda: Box<dyn Field<f64>>,
    ) -> Self {
        Self {
            inner: MaterialFieldInner::Yeoh {
                mu,
                c2,
                lambda,
                bounds: None,
            },
            interface_sdf: None,
        }
    }

    /// Yeoh-with-calibrated-bounds constructor — five
    /// [`Field<f64>`](crate::field::Field) impls (μ, C₂, λ,
    /// max-principal-stretch, min-principal-stretch).
    ///
    /// Each sample point's `Yeoh` is built via
    /// [`Yeoh::from_lame_and_c2`] then decorated with
    /// [`Yeoh::with_max_principal_stretch_only`] per H4-2-C
    /// asymmetric one-sided bound (see
    /// `docs/CANDIDATE_H4_FALSIFICATION_BOOKMARK.md` §5).  The
    /// produced `Yeoh` carries `(Some(max), None)` — the tensile
    /// `0.8 · λ_break` cap reaches the solver gate at
    /// `check_validity_at_step_start`; the compressive value is
    /// sampled from `min_principal_stretch` but dropped at the
    /// `with_max_principal_stretch_only` call.  Pre-H4 path
    /// (`from_yeoh_fields` 3-arg) falls back to the legacy
    /// `max_stretch_deviation = 1.0` symmetric gate which
    /// accepted σ ∈ [0, 2] including extreme compression; the
    /// H4-2-C asymmetric path matches that compressive permissivity
    /// while adding the calibrated tensile cap.
    ///
    /// Use this when the per-layer materials come from
    /// [`crate::SiliconeMaterial`] anchors (the
    /// `validity_max_principal_stretch` +
    /// `validity_min_principal_stretch` fields lift directly into
    /// the two bound fields).  Closes the
    /// `MaterialField`-drops-bounds gap diagnosed at
    /// `docs/CANDIDATE_E_B_FALSIFICATION_BOOKMARK.md` §10.4 +
    /// originally addressed by
    /// `docs/CANDIDATE_H4_YEOH_BOUND_CALIBRATION_SPEC.md` (spec
    /// later case-D falsified — see the H4 falsification bookmark).
    #[must_use]
    pub fn from_yeoh_fields_with_bounds(
        mu: Box<dyn Field<f64>>,
        c2: Box<dyn Field<f64>>,
        lambda: Box<dyn Field<f64>>,
        max_principal_stretch: Box<dyn Field<f64>>,
        min_principal_stretch: Box<dyn Field<f64>>,
    ) -> Self {
        Self {
            inner: MaterialFieldInner::Yeoh {
                mu,
                c2,
                lambda,
                bounds: Some(YeohBoundsFields {
                    max_principal_stretch,
                    min_principal_stretch,
                }),
            },
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

    /// Coarse identifier for the variant this field carries.
    ///
    /// Mesh constructors enforce variant-vs-mesh-type matching by
    /// calling [`MaterialField::sample`] (NH path) or
    /// [`MaterialField::sample_yeoh`] (Yeoh path), which panic on
    /// mismatch. `kind()` is a non-panicking probe for callers that
    /// want graceful error handling before reaching the mesh
    /// constructor.
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
    // Variant-mismatch panic is a logic-error guard hit only when a
    // Yeoh-built field is sampled via the NH accessor; mesh constructors
    // gate the variant per the doc above so the runtime path never
    // reaches this branch.
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
    /// [`Yeoh`].
    ///
    /// Two paths depending on which constructor built the field:
    ///
    /// - [`MaterialField::from_yeoh_fields`] (3-arg legacy path):
    ///   bounds are `None`; the produced `Yeoh`'s validity
    ///   inherits `(max_principal_stretch, min_principal_stretch) =
    ///   (None, None)` and the solver gate falls back to
    ///   `max_stretch_deviation`.
    /// - [`MaterialField::from_yeoh_fields_with_bounds`] (5-arg
    ///   calibrated path, H4-2-C asymmetric one-sided): the
    ///   `max_principal_stretch` field is sampled + routed through
    ///   [`Yeoh::with_max_principal_stretch_only`]; the produced
    ///   `Yeoh` carries `(Some(max), None)`.  The `min_principal_stretch`
    ///   field stays threaded through the aggregator (preserved for
    ///   future Option B / Phase H F-bar work that re-enables the
    ///   compressive gate) but is unused at sample time — see
    ///   `docs/CANDIDATE_H4_FALSIFICATION_BOOKMARK.md` §5 for the
    ///   falsification analysis that motivated dropping the
    ///   compressive direction.  `det F > 0` inversion remains the
    ///   only compressive safety net.
    ///
    /// # Panics
    ///
    /// Panics if this field was constructed via
    /// [`MaterialField::from_fields`] / [`MaterialField::uniform`] /
    /// [`MaterialField::skeleton_default`] — call
    /// [`MaterialField::sample`] for NH variants.
    #[must_use]
    // Symmetric to `sample`: variant-mismatch panic is an assembly-time
    // logic-error guard, not a runtime path. Mesh constructors gate the
    // variant per the doc above.
    #[allow(clippy::panic)]
    pub fn sample_yeoh(&self, x_ref: Vec3) -> Yeoh {
        match &self.inner {
            MaterialFieldInner::Yeoh {
                mu,
                c2,
                lambda,
                bounds,
            } => {
                let yeoh = Yeoh::from_lame_and_c2(
                    mu.sample(x_ref),
                    lambda.sample(x_ref),
                    c2.sample(x_ref),
                );
                match bounds {
                    None => yeoh,
                    // H4-2-C asymmetric one-sided bound — tensile cap
                    // only; `min_principal_stretch` field stays
                    // threaded through `bounds` for future Option B
                    // (Phase H F-bar / mixed-u-p decorator) re-enable
                    // but is unused at sample time.  Falsification
                    // motivating this asymmetry at
                    // `docs/CANDIDATE_H4_FALSIFICATION_BOOKMARK.md` §5.
                    Some(b) => {
                        yeoh.with_max_principal_stretch_only(b.max_principal_stretch.sample(x_ref))
                    }
                }
            }
            MaterialFieldInner::NeoHookean { .. } => {
                panic!(
                    "MaterialField::sample_yeoh expects Yeoh variant; this field is NH — use sample"
                )
            }
        }
    }
}

/// Material types that mesh constructors can populate from a
/// [`MaterialField`].
///
/// Sealed at this layer because [`Material`] alone doesn't know how
/// to read the field's enum-tagged storage — that dispatch lives
/// here. Implemented for [`NeoHookean`] (reads NH variant via
/// [`MaterialField::sample`]) and [`Yeoh`] (reads Yeoh variant via
/// [`MaterialField::sample_yeoh`]). A future material model would add
/// a third impl + matching `MaterialFieldInner` variant + matching
/// `MaterialField::sample_*` accessor.
pub trait BuildableFromField: Material + Sized + 'static {
    /// Walk every tet, sample the field at each tet's centroid, and
    /// collect a `Vec<Self>` matching `tets.len()`. The Tet4 default
    /// per Part 7 §02 §00 — Phase H Tet10 will sample at four Gauss
    /// points instead and bypass this helper.
    fn cache_from_field(
        positions: &[Vec3],
        tets: &[[VertexId; 4]],
        field: &MaterialField,
    ) -> Vec<Self>;
}

/// Centroid-walk shared by every [`BuildableFromField`] impl. Pure
/// topology + a per-centroid sampler closure; the only thing each
/// impl varies is `sampler` (which `MaterialField::sample_*` to call).
fn cache_walk<M, F: Fn(Vec3) -> M>(
    positions: &[Vec3],
    tets: &[[VertexId; 4]],
    sampler: F,
) -> Vec<M> {
    tets.iter()
        .map(|&tv| {
            let v0 = positions[tv[0] as usize];
            let v1 = positions[tv[1] as usize];
            let v2 = positions[tv[2] as usize];
            let v3 = positions[tv[3] as usize];
            let centroid = (v0 + v1 + v2 + v3) * 0.25;
            sampler(centroid)
        })
        .collect()
}

impl BuildableFromField for NeoHookean {
    fn cache_from_field(
        positions: &[Vec3],
        tets: &[[VertexId; 4]],
        field: &MaterialField,
    ) -> Vec<Self> {
        cache_walk(positions, tets, |x| field.sample(x))
    }
}

impl BuildableFromField for Yeoh {
    fn cache_from_field(
        positions: &[Vec3],
        tets: &[[VertexId; 4]],
        field: &MaterialField,
    ) -> Vec<Self> {
        cache_walk(positions, tets, |x| field.sample_yeoh(x))
    }
}
