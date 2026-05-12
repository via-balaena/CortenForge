//! Cure-protocol data for cast-time silicone procedures (F3).
//!
//! cf-cast-local table of mix ratio + pot life + cure time per
//! Smooth-On platinum-cure silicone, keyed by the same anchor name
//! convention `sim_soft::SiliconeMaterial` uses
//! (`"ECOFLEX_00_30"`, `"DRAGON_SKIN_10A"`, …). [`lookup`] resolves a
//! [`crate::MoldingMaterial::anchor_key`] into the corresponding
//! [`CureProtocol`].
//!
//! Decoupled from `sim-soft`'s `SiliconeMaterial` per the Stage 2
//! architecture decision — workshop-procedure data is a distinct
//! concern from FEM material properties. Values here propagate
//! by-name, not by-import, so updates to the FEM-side anchor table
//! do not require touching cf-cast.
//!
//! All values are per the Smooth-On Technical Data Sheets
//! ([smooth-on.com](https://www.smooth-on.com)) for each grade at
//! 73 °F (23 °C). Cooler temperatures slow both pot life and
//! cure; warmer accelerates. Treat the numbers here as a
//! starting-point reference for the F3 procedure markdown — the
//! workshop user revises against bench reality (iter-1 onward).

/// Cure-protocol record for one Smooth-On platinum-cure silicone.
///
/// All times are at 73 °F (23 °C, the reference temperature on every
/// Smooth-On TDS row). Mix ratio is per-data-sheet — for the entire
/// Ecoflex and Dragon Skin lines the value is `"1A:1B"` (by weight
/// or volume; the TDS rows accept either), so the field is
/// `&'static str` rather than a numeric ratio to surface this
/// faithfully.
#[derive(Debug, Clone, Copy)]
pub struct CureProtocol {
    /// Smooth-On TDS mix ratio (Part A : Part B), e.g. `"1A:1B"`.
    /// The Ecoflex / Dragon Skin lines are all 1:1; the field is
    /// `&'static str` so future Smooth-On lines with non-1:1 ratios
    /// land cleanly without numeric-conversion friction.
    pub mix_ratio_a_to_b: &'static str,
    /// Pot life (working time before the mix begins to gel) in
    /// minutes at 73 °F.
    pub pot_life_minutes: u32,
    /// Cure time (time to demold-ready firmness) in hours at 73 °F.
    /// `f64` because some grades publish fractional hours (e.g.
    /// Ecoflex 00-50 at 3 hours sharp, Dragon Skin 30A at 16 hours).
    pub cure_time_hours: f64,
}

/// Resolve a [`crate::MoldingMaterial::anchor_key`] to its
/// [`CureProtocol`], or `None` if the key isn't a recognized
/// Smooth-On anchor.
///
/// `None` outcomes (the caller's [`crate::MoldingMaterial`] carries
/// `anchor_key = None`, OR the supplied key is non-Smooth-On)
/// surface in F3 procedure markdown as a "consult Smooth-On TDS for
/// cure protocol" placeholder.
#[must_use]
pub fn lookup(anchor_key: &str) -> Option<&'static CureProtocol> {
    match anchor_key {
        "ECOFLEX_00_10" => Some(&ECOFLEX_00_10_CURE),
        "ECOFLEX_00_20" => Some(&ECOFLEX_00_20_CURE),
        "ECOFLEX_00_30" => Some(&ECOFLEX_00_30_CURE),
        "ECOFLEX_00_50" => Some(&ECOFLEX_00_50_CURE),
        "DRAGON_SKIN_10A" => Some(&DRAGON_SKIN_10A_CURE),
        "DRAGON_SKIN_15" => Some(&DRAGON_SKIN_15_CURE),
        "DRAGON_SKIN_20A" => Some(&DRAGON_SKIN_20A_CURE),
        "DRAGON_SKIN_30A" => Some(&DRAGON_SKIN_30A_CURE),
        _ => None,
    }
}

// Per Smooth-On TDS at 73 °F (23 °C). Sources: smooth-on.com data
// sheets per grade. Treat as starting-point reference; the workshop
// user is the ultimate source of truth post-iter-1.

/// Ecoflex 00-10 cure protocol — Shore 00-10, softest Ecoflex grade.
pub const ECOFLEX_00_10_CURE: CureProtocol = CureProtocol {
    mix_ratio_a_to_b: "1A:1B",
    pot_life_minutes: 30,
    cure_time_hours: 4.0,
};

/// Ecoflex 00-20 cure protocol — Shore 00-20.
pub const ECOFLEX_00_20_CURE: CureProtocol = CureProtocol {
    mix_ratio_a_to_b: "1A:1B",
    pot_life_minutes: 30,
    cure_time_hours: 4.0,
};

/// Ecoflex 00-30 cure protocol — Shore 00-30, the most-cited
/// Ecoflex grade in the soft-robotics literature.
pub const ECOFLEX_00_30_CURE: CureProtocol = CureProtocol {
    mix_ratio_a_to_b: "1A:1B",
    pot_life_minutes: 45,
    cure_time_hours: 4.0,
};

/// Ecoflex 00-50 cure protocol — Shore 00-50, firmest Ecoflex grade.
pub const ECOFLEX_00_50_CURE: CureProtocol = CureProtocol {
    mix_ratio_a_to_b: "1A:1B",
    pot_life_minutes: 18,
    cure_time_hours: 3.0,
};

/// Dragon Skin 10A (Medium cure-speed variant) cure protocol —
/// Shore 10A. Fast and Slow variants share the mechanical properties
/// but differ in pot life / cure time; Medium is the default
/// reference here.
pub const DRAGON_SKIN_10A_CURE: CureProtocol = CureProtocol {
    mix_ratio_a_to_b: "1A:1B",
    pot_life_minutes: 30,
    cure_time_hours: 5.0,
};

/// Dragon Skin 15 cure protocol — Shore 15A.
pub const DRAGON_SKIN_15_CURE: CureProtocol = CureProtocol {
    mix_ratio_a_to_b: "1A:1B",
    pot_life_minutes: 35,
    cure_time_hours: 5.0,
};

/// Dragon Skin 20A cure protocol — Shore 20A.
pub const DRAGON_SKIN_20A_CURE: CureProtocol = CureProtocol {
    mix_ratio_a_to_b: "1A:1B",
    pot_life_minutes: 25,
    cure_time_hours: 4.0,
};

/// Dragon Skin 30A cure protocol — Shore 30A, firmest Dragon Skin
/// grade. Long cure time relative to softer variants is per TDS.
pub const DRAGON_SKIN_30A_CURE: CureProtocol = CureProtocol {
    mix_ratio_a_to_b: "1A:1B",
    pot_life_minutes: 45,
    cure_time_hours: 16.0,
};

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use super::{
        DRAGON_SKIN_10A_CURE, DRAGON_SKIN_15_CURE, DRAGON_SKIN_20A_CURE, DRAGON_SKIN_30A_CURE,
        ECOFLEX_00_10_CURE, ECOFLEX_00_20_CURE, ECOFLEX_00_30_CURE, ECOFLEX_00_50_CURE, lookup,
    };

    /// Full enumeration of the 8 currently-anchored cure protocols,
    /// in source-PSI order (mirrors `silicone_table.rs::ALL`).
    /// Adding a new anchor here surfaces in every contract test
    /// below without per-test duplication.
    const ALL: &[(&str, super::CureProtocol)] = &[
        ("ECOFLEX_00_10", ECOFLEX_00_10_CURE),
        ("ECOFLEX_00_20", ECOFLEX_00_20_CURE),
        ("ECOFLEX_00_30", ECOFLEX_00_30_CURE),
        ("ECOFLEX_00_50", ECOFLEX_00_50_CURE),
        ("DRAGON_SKIN_10A", DRAGON_SKIN_10A_CURE),
        ("DRAGON_SKIN_15", DRAGON_SKIN_15_CURE),
        ("DRAGON_SKIN_20A", DRAGON_SKIN_20A_CURE),
        ("DRAGON_SKIN_30A", DRAGON_SKIN_30A_CURE),
    ];

    #[test]
    fn lookup_returns_some_for_every_anchor_in_table() {
        // Pins that `lookup`'s match arms stay parallel to the
        // const-anchor declarations. Adding an anchor without a
        // matching arm (or vice versa) trips here.
        for (key, expected) in ALL {
            let got = lookup(key).expect("anchor key should resolve");
            assert_eq!(got.mix_ratio_a_to_b, expected.mix_ratio_a_to_b);
            assert_eq!(got.pot_life_minutes, expected.pot_life_minutes);
            assert!((got.cure_time_hours - expected.cure_time_hours).abs() < f64::EPSILON);
        }
    }

    #[test]
    fn lookup_returns_none_for_unknown_or_empty_key() {
        // Non-Smooth-On / unrecognized anchor → caller falls back
        // to the "consult Smooth-On TDS" placeholder in F3 markdown.
        assert!(lookup("").is_none());
        assert!(lookup("UNKNOWN_GRADE").is_none());
        assert!(lookup("ecoflex_00_30").is_none(), "case-sensitive");
        // Looks like a Smooth-On grade but isn't yet anchored in
        // this table (Smooth-On Mold Star line) — surfaces None and
        // the placeholder path. Adding it later is non-breaking.
        assert!(lookup("MOLD_STAR_30").is_none());
    }

    #[test]
    fn all_cure_protocols_have_positive_finite_times() {
        // Catches accidental zero / NaN / negative typos in the
        // const-anchor declarations. Same shape as the
        // `silicone_table::all_values_strictly_positive_and_finite`
        // contract test.
        for (name, protocol) in ALL {
            assert!(
                protocol.pot_life_minutes > 0,
                "{name}: pot life must be > 0, got {}",
                protocol.pot_life_minutes
            );
            assert!(
                protocol.cure_time_hours > 0.0 && protocol.cure_time_hours.is_finite(),
                "{name}: cure time must be > 0 and finite, got {}",
                protocol.cure_time_hours
            );
            assert!(
                !protocol.mix_ratio_a_to_b.is_empty(),
                "{name}: mix ratio string must be non-empty"
            );
        }
    }

    #[test]
    fn pot_life_strictly_less_than_cure_time() {
        // Physical invariant: pot life (mix begins to gel) must
        // precede demold readiness. If a Smooth-On TDS update
        // inverts these values for a grade, our table needs to
        // reflect the new TDS — fire-loudly catches such drift.
        for (name, protocol) in ALL {
            let pot_life_hours = f64::from(protocol.pot_life_minutes) / 60.0;
            assert!(
                pot_life_hours < protocol.cure_time_hours,
                "{name}: pot life ({pot_life_hours} hr) must be < cure time ({} hr)",
                protocol.cure_time_hours
            );
        }
    }

    #[test]
    fn all_anchors_share_one_to_one_mix_ratio() {
        // The whole Ecoflex + Dragon Skin lines are 1A:1B per TDS.
        // Pin this as a contract — if a future grade with a
        // non-1:1 ratio gets added, this assertion catches the new
        // case and the F3 markdown template needs review for ratio
        // formatting.
        for (name, protocol) in ALL {
            assert_eq!(
                protocol.mix_ratio_a_to_b, "1A:1B",
                "{name}: unexpected mix ratio {:?} \
                 — F3 markdown template may need review",
                protocol.mix_ratio_a_to_b
            );
        }
    }
}
