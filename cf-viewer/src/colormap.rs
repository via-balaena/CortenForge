//! Colormap pipeline — auto-detect distribution category + per-vertex RGBA.
//!
//! Per `docs/VIEWER_DESIGN.md` Q5 (locked iter 1) + iter 1.5 normalization
//! interpretation:
//!
//! - Any value `< 0.0` → **divergent** (`coolwarm`, "centered at 0" via
//!   `TwoSlopeNorm`: negatives stretch to `t ∈ [0, 0.5]` independent of
//!   positive magnitude, positives to `t ∈ [0.5, 1.0]` — value 0 always
//!   lands at `t = 0.5` (white center), both extremes hit the saturated
//!   ends of the colormap).
//! - Else if all values cast equal to integer AND there are 2..16 unique
//!   values → **categorical** (`tab10`, indexed by `value mod 10`).
//! - Otherwise → **sequential** (`viridis`, range = data extent).
//!
//! ## Edge cases
//!
//! - Non-finite values (NaN / ±inf) are filtered out of every detection
//!   check, so a stray NaN does not hijack the divergent branch nor break
//!   the integer-cast test. [`Colormap::rgba`] returns mid-grey for
//!   non-finite inputs.
//! - Empty values slice (or all-NaN slice) → Sequential. Harmless default;
//!   the colormap will not actually be applied because there is no data.
//! - Single unique integer value (e.g. all-zero) → Sequential. Categorical
//!   needs ≥ 2 classes to be meaningful; Divergent needs negatives.
//!
//! ## Bevy 0.18 vertex-color path (callers, FYI)
//!
//! `Mesh::ATTRIBUTE_COLOR` is `VertexFormat::Float32x4`. The PBR fragment
//! shader OVERWRITES `pbr_input.material.base_color` from `in.color` (see
//! `bevy_pbr-0.18.1/src/render/pbr_fragment.wgsl:54-56`), so a shared
//! template `StandardMaterial` "just works" as the no-scalars fallback —
//! its `base_color` is ignored when vertex colors are present.

#![allow(clippy::cast_possible_truncation)] // f32 → i64, table-len → f32 are intentional
#![allow(clippy::cast_sign_loss)] // rem_euclid output is non-negative by construction
#![allow(clippy::cast_precision_loss)] // table lengths are 9 or 10; well within f32 precision
#![allow(clippy::cast_possible_wrap)] // TAB10.len() = 10; usize → i64 is trivially safe

use std::collections::HashSet;

/// Distribution category of a per-vertex scalar — picks which colormap
/// table to sample from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ColormapKind {
    /// Symmetric around 0; signed scalars (e.g. `signed_distance`).
    Divergent,
    /// Continuous positive scalars; magnitudes, distances, fractions.
    Sequential,
    /// Small set of integer labels; zone IDs, material IDs.
    Categorical,
}

/// A detected colormap — kind plus the value range used to normalize lookups.
///
/// Construct with [`Colormap::from_values`]; map values with
/// [`Colormap::rgba`].
#[derive(Debug, Clone, Copy)]
pub struct Colormap {
    /// Detected distribution category.
    pub kind: ColormapKind,
    /// Data extent `(min, max)`. For [`ColormapKind::Divergent`] this is
    /// the raw data range — normalization is asymmetric (`TwoSlopeNorm`),
    /// not via a symmetric `(-m, m)` bound. For [`ColormapKind::Sequential`]
    /// it's the data extent. Unused for [`ColormapKind::Categorical`]
    /// (which indexes by `value mod 10`).
    pub range: (f32, f32),
}

impl Colormap {
    /// Detect the distribution category of `values` and store the data
    /// extent. All three kinds use the raw `(min, max)` extent — the
    /// `TwoSlopeNorm` two-half mapping for [`ColormapKind::Divergent`]
    /// happens at sample time inside [`Self::rgba`].
    #[must_use]
    pub fn from_values(values: &[f32]) -> Self {
        let kind = detect(values);
        let range = data_range(values);
        Self { kind, range }
    }

    /// Map a scalar value to an RGBA color in linear `[0, 1]` channels.
    ///
    /// Non-finite inputs render as mid-grey rather than panicking or
    /// producing garbage colors at clamp boundaries.
    #[must_use]
    pub fn rgba(&self, value: f32) -> [f32; 4] {
        if !value.is_finite() {
            return [0.5, 0.5, 0.5, 1.0];
        }
        match self.kind {
            ColormapKind::Sequential => sample(VIRIDIS, normalize(value, self.range)),
            ColormapKind::Divergent => sample(COOLWARM, two_slope_normalize(value, self.range)),
            ColormapKind::Categorical => {
                let idx = (value as i64).rem_euclid(TAB10.len() as i64) as usize;
                let (r, g, b) = TAB10[idx];
                [r, g, b, 1.0]
            }
        }
    }
}

/// Detect the distribution category of `values` per the Q5 rules.
///
/// Pure function over the values slice — uniqueness is tracked via a
/// `HashSet<u32>` keyed on `f32::to_bits` (only meaningful while every
/// observed value is integer-valued; the set is dropped at the function
/// boundary).
#[must_use]
pub fn detect(values: &[f32]) -> ColormapKind {
    let mut any_finite = false;
    let mut any_negative = false;
    let mut all_int = true;
    let mut unique: HashSet<u32> = HashSet::new();

    for &v in values {
        if !v.is_finite() {
            continue;
        }
        any_finite = true;
        if v < 0.0 {
            any_negative = true;
        }
        if v != v.trunc() {
            all_int = false;
        }
        // Only track unique values while the integer-cast invariant still
        // holds; once `all_int` flips false the count is irrelevant.
        if all_int {
            unique.insert(v.to_bits());
        }
    }

    if !any_finite {
        return ColormapKind::Sequential;
    }
    if any_negative {
        return ColormapKind::Divergent;
    }
    // Categorical needs ≥ 2 distinct integer classes to carry information.
    if all_int && unique.len() >= 2 && unique.len() < 16 {
        return ColormapKind::Categorical;
    }
    ColormapKind::Sequential
}

/// Min/max over finite values. Defaults to `(0.0, 1.0)` if no finite value
/// is present (degenerate inputs — empty, all-NaN — would otherwise leave
/// the bounds at ±inf).
#[must_use]
fn data_range(values: &[f32]) -> (f32, f32) {
    let mut min = f32::INFINITY;
    let mut max = f32::NEG_INFINITY;
    for &v in values {
        if v.is_finite() {
            if v < min {
                min = v;
            }
            if v > max {
                max = v;
            }
        }
    }
    if min.is_finite() && max.is_finite() {
        (min, max)
    } else {
        (0.0, 1.0)
    }
}

/// Normalize `value` into `[0, 1]` over `range`. Degenerate range (zero
/// extent) maps every value to `0.5` so the colormap still returns a
/// well-defined center color.
#[inline]
#[must_use]
fn normalize(value: f32, range: (f32, f32)) -> f32 {
    let (a, b) = range;
    if b > a {
        ((value - a) / (b - a)).clamp(0.0, 1.0)
    } else {
        0.5
    }
}

/// Two-slope normalization for divergent maps: pin value 0 to `t = 0.5`
/// (center of the colormap) and stretch each side to its own extent.
/// Negatives map `[min, 0] → [0, 0.5]` (deep blue → white); positives map
/// `[0, max] → [0.5, 1.0]` (white → deep red). Either side absent (e.g.
/// `min ≥ 0` or `max ≤ 0`) collapses to `0.5` for that side.
#[inline]
#[must_use]
fn two_slope_normalize(value: f32, range: (f32, f32)) -> f32 {
    let (min, max) = range;
    if value < 0.0 && min < 0.0 {
        (0.5 * (1.0 - value / min)).clamp(0.0, 0.5)
    } else if value > 0.0 && max > 0.0 {
        (0.5 + 0.5 * (value / max)).clamp(0.5, 1.0)
    } else {
        0.5
    }
}

/// Linearly interpolate between `table` control points. `t` is clamped to
/// `[0, 1]`.
#[must_use]
fn sample(table: &[(f32, f32, f32)], t: f32) -> [f32; 4] {
    debug_assert!(table.len() >= 2, "colormap table needs ≥ 2 control points");
    let scaled = t.clamp(0.0, 1.0) * (table.len() - 1) as f32;
    let i0 = scaled.floor() as usize;
    let i1 = (i0 + 1).min(table.len() - 1);
    let frac = scaled - i0 as f32;
    let (r0, g0, b0) = table[i0];
    let (r1, g1, b1) = table[i1];
    [
        r0 + (r1 - r0) * frac,
        g0 + (g1 - g0) * frac,
        b0 + (b1 - b0) * frac,
        1.0,
    ]
}

/// 9-point saturated blue→white→red divergent table (bwr-style).
///
/// Picked over matplotlib's perceptually-uniform `coolwarm` (whose
/// endpoints are `(0.230, 0.299, 0.754)` and `(0.706, 0.016, 0.150)` —
/// medium-saturation by design) because for visual review the
/// negative/positive split must be unmistakable at a glance. Saturation
/// trumps perceptual-luminance uniformity here. Iter 1.7 lock — see
/// `docs/VIEWER_DESIGN.md`.
const COOLWARM: &[(f32, f32, f32)] = &[
    (0.000, 0.000, 1.000),
    (0.250, 0.250, 1.000),
    (0.500, 0.500, 1.000),
    (0.750, 0.750, 1.000),
    (1.000, 1.000, 1.000),
    (1.000, 0.750, 0.750),
    (1.000, 0.500, 0.500),
    (1.000, 0.250, 0.250),
    (1.000, 0.000, 0.000),
];

/// 9-point control approximation of matplotlib's `viridis` sequential
/// colormap. Perceptually monotonic in luminance from dark purple → yellow.
const VIRIDIS: &[(f32, f32, f32)] = &[
    (0.267, 0.005, 0.329),
    (0.282, 0.140, 0.458),
    (0.253, 0.265, 0.530),
    (0.207, 0.372, 0.553),
    (0.164, 0.471, 0.558),
    (0.128, 0.567, 0.551),
    (0.135, 0.659, 0.518),
    (0.267, 0.749, 0.441),
    (0.992, 0.906, 0.144),
];

/// Matplotlib `tab10` — 10 distinct categorical colors. Indexed by
/// `value mod 10` for integer-valued scalars.
const TAB10: &[(f32, f32, f32)] = &[
    (0.122, 0.467, 0.706),
    (1.000, 0.498, 0.055),
    (0.173, 0.627, 0.173),
    (0.839, 0.153, 0.157),
    (0.580, 0.404, 0.741),
    (0.549, 0.337, 0.294),
    (0.890, 0.467, 0.761),
    (0.498, 0.498, 0.498),
    (0.737, 0.741, 0.133),
    (0.090, 0.745, 0.812),
];

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn detect_negative_sample_is_divergent() {
        assert_eq!(detect(&[-1.0, 0.0, 1.0]), ColormapKind::Divergent);
        assert_eq!(detect(&[-0.5, 1.5]), ColormapKind::Divergent);
    }

    #[test]
    fn detect_small_integer_set_is_categorical() {
        assert_eq!(detect(&[0.0, 1.0, 2.0, 3.0]), ColormapKind::Categorical);
        assert_eq!(detect(&[5.0, 5.0, 7.0, 9.0]), ColormapKind::Categorical);
    }

    #[test]
    fn detect_continuous_positive_is_sequential() {
        assert_eq!(detect(&[0.5, 1.5, 2.5]), ColormapKind::Sequential);
        assert_eq!(detect(&[0.0, 0.1, 0.2, 0.3]), ColormapKind::Sequential);
    }

    #[test]
    fn detect_large_integer_set_is_sequential() {
        // 16 unique integers — at-or-above the categorical threshold.
        let values: Vec<f32> = (0..16).map(|i| i as f32).collect();
        assert_eq!(detect(&values), ColormapKind::Sequential);
    }

    #[test]
    fn detect_single_unique_value_is_sequential() {
        // All-zero (and any all-same value) is ambiguous — fall back
        // to Sequential rather than picking a single categorical color.
        assert_eq!(detect(&[0.0, 0.0, 0.0]), ColormapKind::Sequential);
        assert_eq!(detect(&[3.0, 3.0, 3.0, 3.0]), ColormapKind::Sequential);
    }

    #[test]
    fn detect_empty_is_sequential() {
        assert_eq!(detect(&[]), ColormapKind::Sequential);
    }

    #[test]
    fn detect_all_nan_is_sequential() {
        assert_eq!(detect(&[f32::NAN, f32::NAN]), ColormapKind::Sequential);
    }

    #[test]
    fn detect_nan_does_not_hijack_divergent() {
        // NaN is not a negative value — its presence must not flip the
        // category to Divergent.
        assert_eq!(detect(&[f32::NAN, 0.5, 1.5, 2.5]), ColormapKind::Sequential,);
    }

    #[test]
    fn coolwarm_endpoints_and_center() {
        let cm = Colormap {
            kind: ColormapKind::Divergent,
            range: (-1.0, 1.0),
        };
        let center = cm.rgba(0.0);
        let positive = cm.rgba(1.0);
        let negative = cm.rgba(-1.0);
        // Center: light grey — all channels above 0.7.
        assert!(
            center[0] > 0.7 && center[1] > 0.7 && center[2] > 0.7,
            "coolwarm center not ~white: {center:?}",
        );
        // Positive end: red dominates, blue suppressed.
        assert!(
            positive[0] > 0.5 && positive[2] < 0.3,
            "coolwarm +1 not red-end: {positive:?}",
        );
        // Negative end: blue dominates, red suppressed.
        assert!(
            negative[2] > 0.5 && negative[0] < 0.4,
            "coolwarm -1 not blue-end: {negative:?}",
        );
    }

    #[test]
    fn viridis_monotonic_endpoints() {
        let cm = Colormap {
            kind: ColormapKind::Sequential,
            range: (0.0, 1.0),
        };
        // Yellow end (1.0) is brighter and yellower than purple end (0.0).
        let lo = cm.rgba(0.0);
        let hi = cm.rgba(1.0);
        assert!(
            hi[0] + hi[1] > lo[0] + lo[1],
            "viridis 1.0 not yellower than 0.0: lo={lo:?} hi={hi:?}",
        );
    }

    #[test]
    fn rgba_nan_is_mid_grey() {
        let cm = Colormap {
            kind: ColormapKind::Sequential,
            range: (0.0, 1.0),
        };
        assert_eq!(cm.rgba(f32::NAN), [0.5, 0.5, 0.5, 1.0]);
    }

    #[test]
    fn from_values_divergent_range_is_data_extent() {
        let cm = Colormap::from_values(&[-2.0, 0.0, 1.0]);
        assert_eq!(cm.kind, ColormapKind::Divergent);
        // TwoSlopeNorm semantics: range is the raw data extent, not a
        // symmetric `(-m, m)` envelope.
        assert_eq!(cm.range, (-2.0, 1.0));
    }

    #[test]
    fn divergent_two_slope_saturates_each_extreme() {
        // Asymmetric data — typical sphere-SDF shape (min=-1, max=+2.46).
        let cm = Colormap {
            kind: ColormapKind::Divergent,
            range: (-1.0, 2.46),
        };
        // Most-negative point → deep blue end (t=0).
        let deep_neg = cm.rgba(-1.0);
        assert!(
            deep_neg[2] > 0.7 && deep_neg[0] < 0.3,
            "two-slope: min should hit deep blue: {deep_neg:?}",
        );
        // Most-positive point → deep red end (t=1).
        let deep_pos = cm.rgba(2.46);
        assert!(
            deep_pos[0] > 0.6 && deep_pos[2] < 0.2,
            "two-slope: max should hit deep red: {deep_pos:?}",
        );
        // Zero → white center.
        let center = cm.rgba(0.0);
        assert!(
            center[0] > 0.7 && center[1] > 0.7 && center[2] > 0.7,
            "two-slope: 0 should be ~white: {center:?}",
        );
    }

    #[test]
    fn divergent_two_slope_collapses_when_one_side_absent() {
        // All-positive data (min > 0): negatives can't occur in the data
        // but the function shouldn't panic if asked.
        let cm = Colormap {
            kind: ColormapKind::Divergent,
            range: (0.5, 2.0),
        };
        assert_eq!(cm.rgba(-1.0)[..3], cm.rgba(0.0)[..3]);
    }

    #[test]
    fn from_values_sequential_range_is_data_extent() {
        let cm = Colormap::from_values(&[0.5, 1.5, 2.5]);
        assert_eq!(cm.kind, ColormapKind::Sequential);
        assert_eq!(cm.range, (0.5, 2.5));
    }

    #[test]
    fn categorical_indexes_by_value_modulo_table_length() {
        let cm = Colormap {
            kind: ColormapKind::Categorical,
            range: (0.0, 9.0),
        };
        // Wrap-around: same color 10 indices apart.
        assert_eq!(cm.rgba(0.0), cm.rgba(10.0));
        // Distinct integers in-range produce distinct colors.
        assert_ne!(cm.rgba(0.0), cm.rgba(1.0));
    }
}
