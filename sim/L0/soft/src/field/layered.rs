//! `LayeredScalarField` + `BlendedScalarField` — graded-material building
//! blocks per Part 7 §02 §01 [composition][c].
//!
//! [`LayeredScalarField`] is the discrete-step composition: an SDF drives
//! shell selection over a sorted threshold list, and each shell carries a
//! constant `f64` value. The 3-shell concentric-`SphereSdf` is the IV-4
//! validation scene per scope memo §1; the layered silicone device's
//! outer / middle / inner shells are the IV-5 end-to-end load case.
//!
//! [`BlendedScalarField`] is the smooth-step composition the book names
//! at §02 §01: a cubic Hermite smoothstep (`s²(3 − 2s)`, C¹ at the band
//! edges) blends two underlying [`Field<f64>`]s by an SDF-derived weight,
//! with the transition concentrated in a band `±band_half_width` around
//! the SDF zero set. Outside the band the blend snaps cleanly to the
//! dominant side's field with bit-exact 0/1 weights — the property the
//! book commits to in the "outside the band, the step resolves cleanly
//! to 0 or 1" sentence.
//!
//! Phase 4 ships `f64` only. `Vec3` (HGO fiber direction) and `Tensor3`
//! (full orthotropy) are Phase H per scope memo Decision A; the
//! existing [`Field<T>`](super::Field) trait surface is already generic
//! in `T`, so those land as additional concrete types here without a
//! trait change.
//!
//! Both types are deterministic functions of `x_ref` (Decision N
//! carry-forward of walking-skeleton invariant I-5): the threshold list
//! sorts once at construction and lives in a `Vec`; the blend kernel is
//! pure arithmetic on `phi`. No `HashMap` iteration, no float-equality
//! comparison on the sample hot path.
//!
//! [c]: ../../../../docs/studies/soft_body_architecture/src/70-sdf-pipeline/02-material-assignment/01-composition.md

use super::Field;
use crate::Vec3;
use crate::sdf_bridge::Sdf;

/// N-shell concentric step field over `f64`, keyed on a single SDF.
///
/// Construction takes a sorted-ascending strictly-monotone `thresholds`
/// list of length `N` and a `values` list of length `N + 1`. At
/// `sample(x)`, `phi = sdf.eval(x)` is bucketed against `thresholds`
/// (binary search via [`slice::partition_point`]) and the corresponding
/// `values[i]` is returned. **Boundary convention**: at exactly
/// `phi == threshold[i]`, the point belongs to the *outer* shell
/// (`values[i + 1]`); the convention falls out of the
/// `partition_point(|&t| t <= phi)` predicate and is deterministic
/// without per-call float-equality comparison.
///
/// Example — 3-shell concentric structure on a `SphereSdf` of radius
/// `R`:
///
/// ```text
/// thresholds = [-0.30 R, -0.10 R]
/// values     = [inner_value, middle_value, outer_value]
/// phi <= -0.30 R              → values[0]  (innermost shell)
/// -0.30 R < phi <= -0.10 R    → values[1]  (middle shell)
///          phi >  -0.10 R     → values[2]  (outer shell + exterior)
/// ```
///
/// `Send + Sync` follows automatically: `Box<dyn Sdf>` is Send + Sync
/// via the [`Sdf`] supertrait bound, and `Vec<f64>` is too.
pub struct LayeredScalarField {
    sdf: Box<dyn Sdf>,
    thresholds: Vec<f64>,
    values: Vec<f64>,
}

impl LayeredScalarField {
    /// Construct a layered scalar field.
    ///
    /// # Panics
    ///
    /// First-violator-wins per scope memo Decision Q. Panics naming the
    /// specific violation when:
    ///
    /// - `thresholds` is empty (use `ConstantField` for the zero-shell
    ///   degenerate case);
    /// - `values.len() != thresholds.len() + 1`;
    /// - any threshold is non-finite (`NaN` or `±∞`);
    /// - `thresholds` is not strictly monotone-increasing.
    #[must_use]
    pub fn new(sdf: Box<dyn Sdf>, thresholds: Vec<f64>, values: Vec<f64>) -> Self {
        assert!(
            !thresholds.is_empty(),
            "LayeredScalarField: thresholds must be non-empty (use ConstantField for the \
             zero-shell case)"
        );
        assert_eq!(
            values.len(),
            thresholds.len() + 1,
            "LayeredScalarField: values.len() ({}) must be thresholds.len() + 1 ({})",
            values.len(),
            thresholds.len() + 1,
        );
        for (i, &t) in thresholds.iter().enumerate() {
            assert!(
                t.is_finite(),
                "LayeredScalarField: thresholds[{i}] is non-finite ({t})"
            );
        }
        for i in 1..thresholds.len() {
            assert!(
                thresholds[i] > thresholds[i - 1],
                "LayeredScalarField: thresholds must be strictly monotone-increasing; \
                 thresholds[{}] = {} is not greater than thresholds[{}] = {}",
                i,
                thresholds[i],
                i - 1,
                thresholds[i - 1],
            );
        }
        Self {
            sdf,
            thresholds,
            values,
        }
    }
}

impl Field<f64> for LayeredScalarField {
    fn sample(&self, x_ref: Vec3) -> f64 {
        let phi = self.sdf.eval(x_ref);
        // partition_point returns the first index `i` where the predicate
        // is false. With `t <= phi`, that's the first threshold strictly
        // greater than phi — equivalently, the index of the shell phi
        // belongs to. At `phi == threshold[i]` the predicate holds, so
        // the point is bucketed into the outer shell (values[i + 1]),
        // matching the documented boundary convention.
        let i = self.thresholds.partition_point(|&t| t <= phi);
        self.values[i]
    }
}

/// Smooth-step blend between two `Field<f64>`s, weighted by an SDF.
///
/// Per [Part 7 §02 §01 composition][c], with `phi = sdf.eval(x)`:
///
/// ```text
/// s              = clamp((phi + band) / (2 · band), 0, 1)
/// outside_weight = s² (3 − 2s)                      // cubic Hermite smoothstep
/// sample(x)      = (1 − outside_weight) · inside_field.sample(x)
///                + outside_weight · outside_field.sample(x)
/// ```
///
/// Boundary behaviour (relied on by the unit tests + IV-6 interface
/// flagging):
///
/// - `phi <= -band` ⇒ `s = 0`, `outside_weight = 0` exactly ⇒ returns
///   `inside_field.sample(x)` bit-exact;
/// - `phi == 0`     ⇒ `s = 0.5`, `outside_weight = 0.5` exactly ⇒
///   returns the arithmetic mean;
/// - `phi >= band`  ⇒ `s = 1`, `outside_weight = 1` exactly ⇒ returns
///   `outside_field.sample(x)` bit-exact.
///
/// The cubic Hermite kernel is C¹ at both band edges (zero derivative)
/// and bit-exact at the three reference points above; that's why the
/// "snaps cleanly to 0 or 1 outside the band" sentence at §01
/// composition holds without further tolerance machinery. Quintic
/// `t³(6t² − 15t + 10)` (C²) is a Phase H upgrade option if blending
/// derivatives ever need second-order continuity.
///
/// `Send + Sync` follows automatically: `Box<dyn Sdf>` is Send + Sync
/// via the [`Sdf`] supertrait bound, and `Box<dyn Field<f64>>` is
/// Send + Sync via the [`Field`] supertrait bound from commit 1.
///
/// [c]: ../../../../docs/studies/soft_body_architecture/src/70-sdf-pipeline/02-material-assignment/01-composition.md
pub struct BlendedScalarField {
    sdf: Box<dyn Sdf>,
    inside_field: Box<dyn Field<f64>>,
    outside_field: Box<dyn Field<f64>>,
    band_half_width: f64,
}

impl BlendedScalarField {
    /// Construct a smoothstep-blended scalar field.
    ///
    /// `inside_field` dominates at SDF-negative samples (geometrically:
    /// inside the SDF's surface); `outside_field` dominates at
    /// SDF-positive samples; the cubic Hermite kernel mixes them across
    /// `[-band_half_width, +band_half_width]`.
    ///
    /// # Panics
    ///
    /// First-violator-wins per scope memo Decision Q. Panics when
    /// `band_half_width` is not strictly positive and finite (zero,
    /// negative, `NaN`, or `±∞`); a zero-width band is a step function
    /// (use [`LayeredScalarField`] with one threshold) and a non-finite
    /// width is a programming error.
    #[must_use]
    pub fn new(
        sdf: Box<dyn Sdf>,
        inside_field: Box<dyn Field<f64>>,
        outside_field: Box<dyn Field<f64>>,
        band_half_width: f64,
    ) -> Self {
        assert!(
            band_half_width.is_finite() && band_half_width > 0.0,
            "BlendedScalarField: band_half_width must be strictly positive and finite \
             (got {band_half_width}); use LayeredScalarField with one threshold for the \
             zero-width step case"
        );
        Self {
            sdf,
            inside_field,
            outside_field,
            band_half_width,
        }
    }
}

impl Field<f64> for BlendedScalarField {
    fn sample(&self, x_ref: Vec3) -> f64 {
        let phi = self.sdf.eval(x_ref);
        let s = ((phi + self.band_half_width) / (2.0 * self.band_half_width)).clamp(0.0, 1.0);
        // Cubic Hermite smoothstep `s² (3 − 2s)` via FMA: `mul_add(-s, 3)`
        // computes `−2s + 3` with one rounding, matching the formula in
        // print and giving bit-exact results at s ∈ {0, 0.5, 1} (which
        // is what the unit-test boundary cases assert).
        let outside_weight = s * s * 2.0_f64.mul_add(-s, 3.0);
        let inside_value = self.inside_field.sample(x_ref);
        let outside_value = self.outside_field.sample(x_ref);
        // FMA blend: single rounding for the lerp, again bit-exact at
        // outside_weight ∈ {0, 0.5, 1}.
        (1.0 - outside_weight).mul_add(inside_value, outside_weight * outside_value)
    }
}
