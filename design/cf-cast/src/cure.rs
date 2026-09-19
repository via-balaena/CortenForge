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
    /// `f64` is forward-compatible — all eight currently-anchored
    /// grades publish whole-hour cure schedules, but the wider type
    /// accommodates future grades with fractional-hour TDS values
    /// (or sub-hour cures from accelerated formulations) without a
    /// table-wide retype.
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

// Per Smooth-On TDS at 73 °F (23 °C). Sources, retrieved
// 2026-09-19 — the two series technical bulletins, whose rows are
// quoted verbatim in `tests::TDS_ROWS` and checked against every
// constant below:
//   https://www.smooth-on.com/tb/files/ECOFLEX_SERIES_TB.pdf
//   https://www.smooth-on.com/tb/files/DRAGON_SKIN_SERIES_TB.pdf
// Treat as starting-point reference; the workshop user is the
// ultimate source of truth post-iter-1.

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
/// Shore 10A.
///
/// The bulletin lists five Shore-10A rows — Very Fast, Fast,
/// Medium, Slow and 10 AF — which share every mechanical property
/// (475 psi tensile, 22 psi 100 % modulus, 1000 % elongation,
/// 102 pli tear) and differ only in pot life / cure time. Medium
/// is the default reference here.
pub const DRAGON_SKIN_10A_CURE: CureProtocol = CureProtocol {
    mix_ratio_a_to_b: "1A:1B",
    pot_life_minutes: 20,
    cure_time_hours: 5.0,
};

/// Dragon Skin 15 cure protocol — Shore 15A. Single cure speed:
/// unlike Dragon Skin 10, the bulletin lists exactly one 15A row.
pub const DRAGON_SKIN_15_CURE: CureProtocol = CureProtocol {
    mix_ratio_a_to_b: "1A:1B",
    pot_life_minutes: 40,
    cure_time_hours: 7.0,
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

    /// The Smooth-On bulletin rows the constants above were read
    /// from — verbatim as `pdftotext -layout` renders the two series
    /// technical bulletins, whitespace collapsed, retrieved
    /// 2026-09-19.
    ///
    /// Column order per the bulletin header: mixed viscosity,
    /// specific gravity, specific volume, **pot life**, **cure
    /// time**, Shore hardness, tensile strength, 100 % modulus,
    /// elongation at break, die B tear strength, shrinkage.
    ///
    /// This is a **double-entry** check, not an independent one. It
    /// catches a constant drifting away from the row it was read
    /// from, and it puts the source text in the repository where a
    /// reader can compare it against the data sheet without
    /// refetching. It cannot catch an error already present in the
    /// quoted row — only a second source can, and the two rows that
    /// this check was written after were each confirmed against the
    /// per-grade product page as well.
    const TDS_ROWS: &[(&str, &str)] = &[
        (
            "ECOFLEX_00_10",
            "Ecoflex\u{2122} 00-10 14,000 cps 1.04 26.6 30 min. 4 hours 00-10 120 psi 8 psi 800% 22 pli < .001 in./in.",
        ),
        (
            "ECOFLEX_00_20",
            "Ecoflex\u{2122} 00-20 3,000 cps 1.07 26.0 30 min. 4 hours 00-20 160 psi 8 psi 845% 30 pli < .001 in./in.",
        ),
        (
            "ECOFLEX_00_30",
            "Ecoflex\u{2122} 00-30 3,000 cps 1.07 26.0 45 min. 4 hours 00-30 200 psi 10 psi 900% 38 pli < .001 in./in.",
        ),
        (
            "ECOFLEX_00_50",
            "Ecoflex\u{2122} 00-50 8,000 cps 1.07 25.9 18 min. 3 hours 00-50 315 psi 12 psi 980% 50 pli < .001 in./in.",
        ),
        (
            "DRAGON_SKIN_10A",
            "Dragon Skin\u{2122} 10 Medium 23,000 cps 1.07 25.8 20 min. 5 hours 10A 475 psi 22 psi 1000% 102 pli < .001 in./in.",
        ),
        (
            "DRAGON_SKIN_15",
            "Dragon Skin\u{2122} 15 21,000 cps 1.07 25.8 40 min. 7 hours 15A 537 psi 40 psi 771% 112 pli < .001 in./in.",
        ),
        (
            "DRAGON_SKIN_20A",
            "Dragon Skin\u{2122} 20 20,000 cps 1.08 25.6 25 min. 4 hours 20A 550 psi 49 psi 620% 120 pli < .001 in./in.",
        ),
        (
            "DRAGON_SKIN_30A",
            "Dragon Skin\u{2122} 30 20,000 cps 1.08 25.7 45 min. 16 hours 30A 500 psi 86 psi 364% 108 pli < .001 in./in.",
        ),
    ];

    /// Reads pot life and cure time out of a bulletin row: the token
    /// before `min.` and the token before `hours` / `hour`.
    ///
    /// Panics rather than returning an `Option` so a row that stops
    /// stating one of the two fails the test loudly instead of
    /// silently skipping the comparison it exists to make.
    fn pot_life_and_cure_from_row(row: &str) -> (u32, f64) {
        let toks: Vec<&str> = row.split_whitespace().collect();
        let mut pot: Option<u32> = None;
        let mut cure: Option<f64> = None;
        for (i, tok) in toks.iter().enumerate() {
            if i == 0 {
                continue;
            }
            if pot.is_none() && *tok == "min." {
                pot = toks[i - 1].parse().ok();
            }
            if cure.is_none() && (*tok == "hours" || *tok == "hour") {
                cure = toks[i - 1].parse().ok();
            }
        }
        (
            pot.expect("bulletin row states a pot life in minutes"),
            cure.expect("bulletin row states a cure time in hours"),
        )
    }

    #[test]
    fn every_anchor_matches_its_verbatim_tds_row() {
        // The collection first: a row silently dropped would make
        // this test pass by checking nothing.
        assert_eq!(
            TDS_ROWS.len(),
            ALL.len(),
            "one quoted bulletin row per anchored grade"
        );
        for (key, _) in ALL {
            assert!(
                TDS_ROWS.iter().any(|(k, _)| k == key),
                "{key}: anchored but has no quoted bulletin row"
            );
        }

        for (key, row) in TDS_ROWS {
            let (pot, cure) = pot_life_and_cure_from_row(row);
            let got = lookup(key).expect("anchor key should resolve");
            assert_eq!(
                got.pot_life_minutes, pot,
                "{key}: constant says {} min, bulletin row says {pot} min",
                got.pot_life_minutes
            );
            assert!(
                (got.cure_time_hours - cure).abs() < f64::EPSILON,
                "{key}: constant says {} h, bulletin row says {cure} h",
                got.cure_time_hours
            );
        }
    }

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
