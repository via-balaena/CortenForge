//! Slacker recipe data — Smooth-On Slacker™ silicone-softening
//! tables, transcribed verbatim from the Slacker Tactile Mutator
//! Technical Bulletin (rev 011524DH).
//!
//! Slacker is a clear fluid pre-mixed into a platinum-cure silicone's
//! Part B (then Part A is added) to soften the cured rubber and add
//! surface tack. The per-layer recipe feature lets the user dial a
//! base silicone + a Slacker ratio and read back the effective cured
//! hardness — so the bench moment is "read the recipe off the panel"
//! instead of deriving the ratio on the spot.
//!
//! This is the data layer only; the recipe-panel UI lives in
//! cf-device-design's `main.rs`. Of the eight catalog silicones only
//! four have published Slacker data — the rest carry an honest
//! [`Support::NotRecommended`] / [`Support::NoData`] marker rather
//! than a guessed curve.
//!
//! Mix-ratio convention: the TB tabulates "X parts Slacker per
//! 100A + 100B" of base silicone, i.e. per 200 parts base — so a
//! "+50 Slacker" row is a [`Point::slacker_fraction`] of
//! 50 / 200 = 0.25.
//!
//! The recipe-panel UI consumes this through [`support`] (keyed on a
//! layer's base-material anchor key) and renders the resulting curve
//! as a Slacker-ratio picker. [`resolve_slacker_fraction`] is the
//! canonical "snap an arbitrary fraction to the curve, or fall back
//! to native" function.

use std::fmt;

/// Durometer scale for a Slacker-recipe hardness reading. Slacker
/// pushes silicones across scales: a Shore A base softens through
/// Shore 00 (OO) into Shore 000 (OOO, the gel scale). These are
/// distinct durometer scales — not cleanly interconvertible — so the
/// recipe readout surfaces the scale explicitly instead of
/// collapsing them to one number.
// `OO` / `OOO` are the established Shore durometer-scale names;
// clippy's `Ooo` suggestion would obscure them.
#[allow(clippy::upper_case_acronyms)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ShoreScale {
    /// Shore A — the firmest scale here; Dragon Skin's native grades.
    A,
    /// Shore 00 (OO) — soft rubbers; the Ecoflex line's native
    /// scale.
    OO,
    /// Shore 000 (OOO) — gels and ultra-soft rubbers; where most
    /// Slacker recipes land.
    OOO,
}

impl fmt::Display for ShoreScale {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::A => write!(f, "A"),
            Self::OO => write!(f, "00"),
            Self::OOO => write!(f, "000"),
        }
    }
}

/// A cured Shore hardness reading on a named durometer scale.
/// Displays the way Smooth-On writes it — `10A`, `00-30`, `000-7`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ShoreHardness {
    /// Which durometer scale `points` is read on.
    pub scale: ShoreScale,
    /// Durometer points on `scale`. Whole numbers in the TB; `u32`
    /// because Slacker recipes snap to the tabulated data points
    /// (the OO/OOO scale split makes interpolating a single hardness
    /// number meaningless).
    pub points: u32,
}

impl fmt::Display for ShoreHardness {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.scale {
            // Shore A is written suffixed: "10A".
            ShoreScale::A => write!(f, "{}A", self.points),
            // Shore 00 / 000 are written prefixed + hyphenated:
            // "00-30", "000-7".
            ShoreScale::OO | ShoreScale::OOO => {
                write!(f, "{}-{}", self.scale, self.points)
            }
        }
    }
}

/// Cured surface tack at a given Slacker fraction, from the TB's
/// per-row tack column.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Tack {
    /// No tack — the base silicone, or a low Slacker fraction.
    None,
    /// Slight tack.
    Slight,
    /// Between slight and very ("Slight to Very" in the TB).
    SlightToVery,
    /// Very tacky / self-sticking.
    Very,
}

impl fmt::Display for Tack {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let label = match self {
            Self::None => "no tack",
            Self::Slight => "slight tack",
            Self::SlightToVery => "slight-to-very tack",
            Self::Very => "very tacky",
        };
        write!(f, "{label}")
    }
}

/// One point on a base silicone's Slacker softening curve: a Slacker
/// fraction plus the resulting cured hardness + tack.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Point {
    /// Slacker mass as a fraction of the base silicone's combined
    /// Part A + Part B mass. `0.0` is the base with no Slacker
    /// (native hardness); the TB's "+50 / +100 / +150 / +200 Slacker
    /// per 100A+100B" rows are `0.25 / 0.50 / 0.75 / 1.00`.
    pub slacker_fraction: f64,
    /// Cured Shore hardness at this Slacker fraction.
    pub hardness: ShoreHardness,
    /// Cured surface tack at this Slacker fraction.
    pub tack: Tack,
}

/// Table-construction helper — keeps the const curves below readable.
/// Module-internal; the curves are its only callers.
const fn point(slacker_fraction: f64, scale: ShoreScale, points: u32, tack: Tack) -> Point {
    Point {
        slacker_fraction,
        hardness: ShoreHardness { scale, points },
        tack,
    }
}

/// What Slacker data exists for a given base silicone.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Support {
    /// Smooth-On's TB tabulates a softening curve for this base. The
    /// slice leads with the `0.0`-fraction native-hardness point,
    /// then the TB's data points in increasing-Slacker order; the
    /// last point's fraction is the per-base recommended Slacker cap.
    Curve(&'static [Point]),
    /// Smooth-On's TB explicitly advises against Slacker with this
    /// base (Ecoflex 00-10).
    NotRecommended,
    /// No published Slacker data for this base (Dragon Skin 15 / 20A
    /// / 30A). Not a guess — the TB simply does not cover it.
    NoData,
}

// Per-base softening curves, transcribed verbatim from the Slacker TB
// (rev 011524DH). Each leads with the native-hardness point at
// slacker_fraction 0.0. Ecoflex bases stop at 0.50 — the TB marks
// +150 / +200 Slacker "Not Recommended" for the Ecoflex line; the
// Dragon Skin line has no such cap.

/// Ecoflex 00-20 + Slacker — native Shore 00-20.
const ECOFLEX_00_20_CURVE: &[Point] = &[
    point(0.0, ShoreScale::OO, 20, Tack::None),
    point(0.25, ShoreScale::OOO, 47, Tack::Slight),
    point(0.50, ShoreScale::OOO, 32, Tack::SlightToVery),
];

/// Ecoflex 00-30 + Slacker — native Shore 00-30.
const ECOFLEX_00_30_CURVE: &[Point] = &[
    point(0.0, ShoreScale::OO, 30, Tack::None),
    point(0.25, ShoreScale::OOO, 40, Tack::SlightToVery),
    point(0.50, ShoreScale::OOO, 20, Tack::Very),
];

/// Ecoflex 00-50 + Slacker — native Shore 00-50.
const ECOFLEX_00_50_CURVE: &[Point] = &[
    point(0.0, ShoreScale::OO, 50, Tack::None),
    point(0.25, ShoreScale::OOO, 55, Tack::Slight),
    point(0.50, ShoreScale::OOO, 35, Tack::SlightToVery),
];

/// Dragon Skin 10A + Slacker — native Shore 10A. The TB's "Dragon
/// Skin 10" row; runs to 1.0 (the Ecoflex +150 / +200 cap does not
/// apply to the Dragon Skin line).
const DRAGON_SKIN_10A_CURVE: &[Point] = &[
    point(0.0, ShoreScale::A, 10, Tack::None),
    point(0.25, ShoreScale::OO, 30, Tack::None),
    point(0.50, ShoreScale::OOO, 50, Tack::Slight),
    point(0.75, ShoreScale::OOO, 20, Tack::SlightToVery),
    point(1.00, ShoreScale::OOO, 7, Tack::Very),
];

/// Slacker data available for a layer's base silicone, keyed by the
/// catalog `anchor_key` ([`crate::LAYER_MATERIALS`]).
#[must_use]
pub fn support(anchor_key: &str) -> Support {
    match anchor_key {
        "ECOFLEX_00_20" => Support::Curve(ECOFLEX_00_20_CURVE),
        "ECOFLEX_00_30" => Support::Curve(ECOFLEX_00_30_CURVE),
        "ECOFLEX_00_50" => Support::Curve(ECOFLEX_00_50_CURVE),
        "DRAGON_SKIN_10A" => Support::Curve(DRAGON_SKIN_10A_CURVE),
        // The TB explicitly says do NOT use Slacker with Ecoflex
        // 00-10 (or Ecoflex GEL).
        "ECOFLEX_00_10" => Support::NotRecommended,
        // Dragon Skin 15 / 20A / 30A are not covered by the Slacker
        // TB — no guessed curve. `_` also catches any off-catalog
        // key defensively.
        "DRAGON_SKIN_15" | "DRAGON_SKIN_20A" | "DRAGON_SKIN_30A" => Support::NoData,
        _ => Support::NoData,
    }
}

/// The Slacker fraction a layer should actually use: `requested` if
/// it is an exact data point on the base silicone's Smooth-On TB
/// curve, otherwise `0.0` (no Slacker — the always-valid native
/// default).
///
/// Called every frame before the Slacker control renders, so a
/// base-material change that orphans the stored fraction (the new
/// material has a different curve, or no Slacker support at all)
/// snaps cleanly back to "no Slacker" instead of silently keeping a
/// fraction that is not on the new curve.
#[must_use]
pub fn resolve_slacker_fraction(anchor_key: &str, requested: f64) -> f64 {
    match support(anchor_key) {
        Support::Curve(curve) => {
            let on_curve = curve
                .iter()
                .any(|p| (p.slacker_fraction - requested).abs() < f64::EPSILON);
            if on_curve { requested } else { 0.0 }
        }
        // No Slacker curve for this base — force the native silicone.
        Support::NotRecommended | Support::NoData => 0.0,
    }
}

#[cfg(test)]
mod tests {
    use super::{
        DRAGON_SKIN_10A_CURVE, ECOFLEX_00_20_CURVE, ECOFLEX_00_30_CURVE, ECOFLEX_00_50_CURVE,
        Point, ShoreHardness, ShoreScale, Support, Tack, point, resolve_slacker_fraction, support,
    };
    use crate::LAYER_MATERIALS;

    /// The four TB-tabulated curves, paired with their catalog anchor
    /// key. Iterating this keeps the structural tests free of
    /// per-curve duplication.
    const ALL_CURVES: &[(&str, &[Point])] = &[
        ("ECOFLEX_00_20", ECOFLEX_00_20_CURVE),
        ("ECOFLEX_00_30", ECOFLEX_00_30_CURVE),
        ("ECOFLEX_00_50", ECOFLEX_00_50_CURVE),
        ("DRAGON_SKIN_10A", DRAGON_SKIN_10A_CURVE),
    ];

    /// Every catalog silicone + the [`Support`] variant it must
    /// resolve to. Adding a catalog entry without deciding its
    /// Slacker status trips `support_covers_every_catalog_silicone`.
    const EXPECTED: &[(&str, &str)] = &[
        ("ECOFLEX_00_10", "not_recommended"),
        ("ECOFLEX_00_20", "curve"),
        ("ECOFLEX_00_30", "curve"),
        ("ECOFLEX_00_50", "curve"),
        ("DRAGON_SKIN_10A", "curve"),
        ("DRAGON_SKIN_15", "no_data"),
        ("DRAGON_SKIN_20A", "no_data"),
        ("DRAGON_SKIN_30A", "no_data"),
    ];

    fn variant_tag(s: Support) -> &'static str {
        match s {
            Support::Curve(_) => "curve",
            Support::NotRecommended => "not_recommended",
            Support::NoData => "no_data",
        }
    }

    #[test]
    fn support_covers_every_catalog_silicone() {
        // Pins that every catalog silicone has a deliberate Slacker
        // decision, and that the catalog + this table stay in sync
        // (count + per-key variant).
        let catalog: Vec<&str> = LAYER_MATERIALS.iter().map(|(k, _, _)| *k).collect();
        let expected_keys: Vec<&str> = EXPECTED.iter().map(|(k, _)| *k).collect();
        assert_eq!(catalog.len(), EXPECTED.len());
        for key in &catalog {
            assert!(
                expected_keys.contains(key),
                "catalog silicone {key} has no Slacker decision in EXPECTED",
            );
        }
        for (key, tag) in EXPECTED {
            assert_eq!(variant_tag(support(key)), *tag, "{key}");
        }
    }

    #[test]
    fn support_wires_each_curve_key_to_its_table() {
        // The four curve keys resolve to Curve(_) pointing at the
        // matching const table — guards against a copy-paste swap in
        // `support`'s match arms.
        for (key, curve) in ALL_CURVES {
            assert!(
                matches!(support(key), Support::Curve(c) if c == *curve),
                "{key} did not resolve to its own curve",
            );
        }
    }

    #[test]
    fn curve_structural_invariants() {
        for (key, curve) in ALL_CURVES {
            // Leads with the native-hardness point: 0.0 Slacker, no
            // tack.
            assert!(
                (curve[0].slacker_fraction - 0.0).abs() < f64::EPSILON,
                "{key}: curve must lead with slacker_fraction 0.0",
            );
            assert_eq!(
                curve[0].tack,
                Tack::None,
                "{key}: native point must have no tack",
            );
            // Slacker fraction strictly increases along the curve.
            for pair in curve.windows(2) {
                assert!(
                    pair[1].slacker_fraction > pair[0].slacker_fraction,
                    "{key}: slacker_fraction must strictly increase",
                );
            }
        }
    }

    #[test]
    fn ecoflex_bases_cap_at_half_slacker_dragon_skin_runs_full() {
        // The last point's fraction is the per-base recommended
        // Slacker cap: Ecoflex bases stop at 0.50 (TB marks +150/+200
        // "Not Recommended"); Dragon Skin 10A runs to 1.00.
        let cap = |curve: &[Point]| curve[curve.len() - 1].slacker_fraction;
        assert!((cap(ECOFLEX_00_20_CURVE) - 0.50).abs() < f64::EPSILON);
        assert!((cap(ECOFLEX_00_30_CURVE) - 0.50).abs() < f64::EPSILON);
        assert!((cap(ECOFLEX_00_50_CURVE) - 0.50).abs() < f64::EPSILON);
        assert!((cap(DRAGON_SKIN_10A_CURVE) - 1.00).abs() < f64::EPSILON);
    }

    #[test]
    fn curve_hardness_values_match_tb() {
        // Verbatim transcription pin against the Slacker TB (rev
        // 011524DH). A drift in any tabulated hardness or tack value
        // trips here.
        assert_eq!(
            ECOFLEX_00_20_CURVE,
            &[
                point(0.0, ShoreScale::OO, 20, Tack::None),
                point(0.25, ShoreScale::OOO, 47, Tack::Slight),
                point(0.50, ShoreScale::OOO, 32, Tack::SlightToVery),
            ][..],
        );
        assert_eq!(
            ECOFLEX_00_30_CURVE,
            &[
                point(0.0, ShoreScale::OO, 30, Tack::None),
                point(0.25, ShoreScale::OOO, 40, Tack::SlightToVery),
                point(0.50, ShoreScale::OOO, 20, Tack::Very),
            ][..],
        );
        assert_eq!(
            ECOFLEX_00_50_CURVE,
            &[
                point(0.0, ShoreScale::OO, 50, Tack::None),
                point(0.25, ShoreScale::OOO, 55, Tack::Slight),
                point(0.50, ShoreScale::OOO, 35, Tack::SlightToVery),
            ][..],
        );
        assert_eq!(
            DRAGON_SKIN_10A_CURVE,
            &[
                point(0.0, ShoreScale::A, 10, Tack::None),
                point(0.25, ShoreScale::OO, 30, Tack::None),
                point(0.50, ShoreScale::OOO, 50, Tack::Slight),
                point(0.75, ShoreScale::OOO, 20, Tack::SlightToVery),
                point(1.00, ShoreScale::OOO, 7, Tack::Very),
            ][..],
        );
    }

    #[test]
    fn shore_hardness_displays_like_smooth_on() {
        // Shore A suffixed; Shore 00 / 000 prefixed + hyphenated.
        assert_eq!(
            ShoreHardness {
                scale: ShoreScale::A,
                points: 10,
            }
            .to_string(),
            "10A",
        );
        assert_eq!(
            ShoreHardness {
                scale: ShoreScale::OO,
                points: 30,
            }
            .to_string(),
            "00-30",
        );
        assert_eq!(
            ShoreHardness {
                scale: ShoreScale::OOO,
                points: 7,
            }
            .to_string(),
            "000-7",
        );
    }

    #[test]
    fn tack_displays_human_labels() {
        assert_eq!(Tack::None.to_string(), "no tack");
        assert_eq!(Tack::Slight.to_string(), "slight tack");
        assert_eq!(Tack::SlightToVery.to_string(), "slight-to-very tack");
        assert_eq!(Tack::Very.to_string(), "very tacky");
    }

    #[test]
    fn resolve_keeps_on_curve_values() {
        // ECOFLEX_00_30's curve has points at 0.0, 0.25, 0.50.
        assert!((resolve_slacker_fraction("ECOFLEX_00_30", 0.25) - 0.25).abs() < f64::EPSILON,);
        // DRAGON_SKIN_10A's curve runs to 1.0.
        assert!((resolve_slacker_fraction("DRAGON_SKIN_10A", 0.75) - 0.75).abs() < f64::EPSILON,);
        // Native 0.0 is always on every curve.
        assert!((resolve_slacker_fraction("ECOFLEX_00_30", 0.0) - 0.0).abs() < f64::EPSILON,);
    }

    #[test]
    fn resolve_resets_orphaned_values() {
        // ECOFLEX_00_30 has no point at 0.75 — snap to 0.0.
        assert!((resolve_slacker_fraction("ECOFLEX_00_30", 0.75) - 0.0).abs() < f64::EPSILON,);
        // Off-curve arbitrary fraction also snaps to 0.0.
        assert!((resolve_slacker_fraction("ECOFLEX_00_30", 0.123) - 0.0).abs() < f64::EPSILON,);
        // NotRecommended → force 0.0.
        assert!((resolve_slacker_fraction("ECOFLEX_00_10", 0.25) - 0.0).abs() < f64::EPSILON,);
        // NoData → force 0.0.
        assert!((resolve_slacker_fraction("DRAGON_SKIN_15", 0.25) - 0.0).abs() < f64::EPSILON,);
    }
}
