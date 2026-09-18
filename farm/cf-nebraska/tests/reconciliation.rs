//! The gates on the calibration dataset, pre-registered before the data was
//! entered.
//!
//! Each test names the mutation that must make it fail. Two checks are gated
//! here and they catch different things: the within-edition unit reconciliation
//! catches a misread digit, and the cross-edition agreement catches a figure
//! taken from the wrong row — which the first check cannot see, because a
//! neighbouring row's value is a real figure and reconciles perfectly.
#![allow(
    clippy::panic,
    reason = "a gate that cannot name the row it rejected is not much of a gate"
)]

use cf_nebraska::{
    ABSENT, CATCHABLE_ERROR, CORRUPT_SCANS, Conversion, EXCLUDED_EDITIONS, JOHN_DEERE_8245R,
    NOT_TRANSCRIBED, Observation, Printed, Reading, WEAKLY_CHECKED_8245R, reconcile,
};

fn datum(name: &str) -> &'static cf_nebraska::Datum {
    JOHN_DEERE_8245R
        .data
        .iter()
        .find(|d| d.name == name)
        .unwrap_or_else(|| panic!("{name} is not in the data"))
}

// ---------------------------------------------------------------- resolution

/// Every figure resolves to a single value backed by at least two editions.
///
/// MUTATION: drop an edition's observation from any figure so only one remains,
/// and this fails — one source cannot outvote anything.
#[test]
fn every_figure_resolves_on_at_least_two_agreeing_editions() {
    for d in JOHN_DEERE_8245R.data {
        match d.read() {
            Reading::Agreed { agreeing, .. } => assert!(
                agreeing >= 2,
                "{}: resolved on only {agreeing} edition(s)",
                d.name
            ),
            other => panic!("{}: did not resolve — {other:?}", d.name),
        }
    }
    assert_eq!(JOHN_DEERE_8245R.data.len(), 25);
}

/// ⛔⛔ The gate the cross-edition layer exists for.
///
/// A figure taken from the wrong row passes the within-edition check, because
/// it is a real figure that reconciles with its own conversion. Only
/// disagreement between editions catches it. Adjacent rows here differ by as
/// little as 0.17%, so this is not hypothetical — an automated extractor
/// written for this crate made exactly this mistake on four figures.
///
/// MUTATION: make `reconcile` return the first value instead of comparing, and
/// this fails.
#[test]
fn cross_edition_agreement_catches_a_figure_from_the_wrong_row() {
    // 50% load power, and the neighbouring reduced-rpm row: 0.17% apart.
    let right = Printed::new(104.27, 2, 77.75, 2, Conversion::HpToKw);
    let neighbour = Printed::new(104.45, 2, 77.88, 2, Conversion::HpToKw);
    assert!(
        right.internally_consistent() && neighbour.internally_consistent(),
        "both rows must reconcile on their own, or this proves nothing"
    );
    let mixed = [
        Observation::new(2016, right),
        Observation::new(2017, right),
        Observation::new(2019, neighbour),
    ];
    assert!(
        matches!(reconcile(&mixed), Reading::Conflicted { .. }),
        "a wrong-row figure was not caught by cross-edition comparison"
    );
    let clean = [Observation::new(2016, right), Observation::new(2017, right)];
    assert!(reconcile(&clean).agreed().is_some(), "clean set must agree");
}

/// With no internally-consistent edition there is nothing to report.
#[test]
fn a_figure_with_no_consistent_edition_is_unresolved() {
    let broken = Printed::new(154.17, 2, 115.19, 2, Conversion::HpToKw);
    assert!(!broken.internally_consistent());
    let obs = [Observation::new(2019, broken)];
    assert_eq!(reconcile(&obs), Reading::Unresolved);
    assert_eq!(reconcile(&[]), Reading::Unresolved);
}

// ------------------------------------------------------------ corrupt scans

/// MUTATION: add a figure whose scans are all clean, or drop a real one, and
/// this fails. Checked both ways so the roster cannot drift.
#[test]
fn the_corrupt_scan_roster_is_exact() {
    let computed: Vec<(&str, u16)> = JOHN_DEERE_8245R
        .data
        .iter()
        .flat_map(|d| d.corrupt_editions().into_iter().map(move |e| (d.name, e)))
        .collect();
    assert_eq!(
        computed, CORRUPT_SCANS,
        "recorded corrupt-scan roster does not match the computed one"
    );
}

/// Each corrupt scan is outvoted, not merely noted.
#[test]
fn every_corrupt_scan_is_outvoted() {
    for (name, edition) in CORRUPT_SCANS {
        let d = datum(name);
        let r = d.read();
        assert!(
            r.rejected() >= 1,
            "{name}: roster says {edition}'s scan is corrupt but none was rejected"
        );
        assert!(
            r.agreed().is_some(),
            "{name}: a corrupt scan left the figure unresolved"
        );
    }
}

/// The corrupt scans are one edition's, which is a fact about that scan.
///
/// MUTATION: a corruption appearing in another edition changes this and should
/// be looked at rather than absorbed.
#[test]
fn the_corrupt_scans_are_all_one_edition() {
    let mut years: Vec<u16> = CORRUPT_SCANS.iter().map(|(_, y)| *y).collect();
    years.dedup();
    assert_eq!(
        years,
        vec![2019],
        "corruption is no longer confined to 2019"
    );
}

// -------------------------------------------------- within-edition strength

/// MUTATION: change any digit of a strongly-checked observation by more than
/// [`CATCHABLE_ERROR`] and its edition must stop reconciling. Both directions:
/// the printed pair is rarely an exact conversion, and that standing offset
/// eats margin on one side only.
#[test]
fn a_perturbed_observation_stops_reconciling() {
    let mut proven = 0;
    for d in JOHN_DEERE_8245R.data {
        if WEAKLY_CHECKED_8245R.contains(&d.name) {
            continue;
        }
        for o in d.observations {
            let p = o.printed();
            if !p.internally_consistent() {
                continue;
            }
            for sign in [1.0, -1.0] {
                assert!(
                    !p.perturbed(sign * CATCHABLE_ERROR).internally_consistent(),
                    "{} [{}]: a {}% error in direction {sign} was not caught",
                    d.name,
                    o.edition(),
                    CATCHABLE_ERROR * 100.0
                );
            }
            proven += 1;
        }
    }
    assert!(proven > 0, "no observation was perturbed");
    assert_eq!(
        proven, 49,
        "17 strongly-checked figures x 3 editions, less the two corrupt 2019 \
         scans that fall inside them"
    );
}

/// MUTATION: add a name that is actually strongly checked, or drop one that is
/// not, and this fails.
#[test]
fn the_weakly_checked_roster_is_exact() {
    let computed: Vec<&str> = JOHN_DEERE_8245R
        .data
        .iter()
        .filter(|d| {
            d.observations
                .iter()
                .all(|o| o.printed().resolution().is_none_or(|r| r > CATCHABLE_ERROR))
        })
        .map(|d| d.name)
        .collect();
    assert_eq!(
        computed, WEAKLY_CHECKED_8245R,
        "recorded weakly-checked roster does not match the computed one"
    );
}

/// A weakly-checked figure is still checked within its edition, at its own
/// coarser threshold — the roster is not a place to park unverified figures.
#[test]
fn weakly_checked_figures_are_still_checked() {
    for name in WEAKLY_CHECKED_8245R {
        for o in datum(name).observations {
            let p = o.printed();
            let Some(r) = p.resolution() else { continue };
            for sign in [1.0, -1.0] {
                assert!(
                    !p.perturbed(sign * r * 1.001).internally_consistent(),
                    "{name} [{}]: an error of {:.4}% was not caught despite being \
                     the resolution this observation claims",
                    o.edition(),
                    r * 100.0
                );
            }
        }
    }
}

/// `resolution()` is pinned from BELOW as well as above.
///
/// Every other test perturbs by at least the resolution and asserts detection,
/// which an over-reporting `resolution()` would also satisfy. This asserts the
/// closed form is tight: just under it, in the direction that needs the most,
/// nothing fires.
#[test]
fn resolution_does_not_over_report() {
    let mut checked = 0;
    for d in JOHN_DEERE_8245R.data {
        for o in d.observations {
            let p = o.printed();
            let Some(r) = p.resolution() else { continue };
            let worse = if p.perturbed(r * 0.999).internally_consistent() {
                1.0
            } else {
                -1.0
            };
            assert!(
                p.perturbed(worse * r * 0.999).internally_consistent(),
                "{} [{}]: an error below the stated resolution of {:.4}% was \
                 caught, so the figure is better protected than it claims",
                d.name,
                o.edition(),
                r * 100.0
            );
            checked += 1;
        }
    }
    assert_eq!(checked, 72, "wrong number of consistent observations");
}

/// The weakness is a property of significant figures, not of a particular row.
#[test]
fn only_low_significance_figures_are_weakly_checked() {
    for name in WEAKLY_CHECKED_8245R {
        assert!(
            name.contains("speed") || name.contains("fuel economy"),
            "{name}: weakly checked but is neither a speed nor a fuel economy"
        );
    }
}

/// The roster is one-directional, and an earlier draft of the docs claimed
/// otherwise. Pins the split so the stronger, false claim cannot creep back.
#[test]
fn the_weak_roster_does_not_contain_every_speed_and_fuel_economy() {
    let strong = |kind: &str| {
        JOHN_DEERE_8245R
            .data
            .iter()
            .filter(|d| d.name.contains(kind))
            .filter(|d| !WEAKLY_CHECKED_8245R.contains(&d.name))
            .count()
    };
    assert_eq!(
        (strong("speed"), strong("fuel economy")),
        (3, 1),
        "the strong/weak split among speeds and fuel economies moved; the \
         WEAKLY_CHECKED_8245R doc states these counts"
    );
}

// ----------------------------------------------------------------- structure

/// Every figure is paired with the conversion its units call for.
///
/// A mis-assigned conversion is mostly caught by reconciliation, but the two
/// gallons-per-something conversions share a factor, so swapping them is
/// invisible to the checksum and visible only here.
#[test]
fn every_figure_uses_the_conversion_its_units_imply() {
    for d in JOHN_DEERE_8245R.data {
        let n = d.name;
        let expected = if n.contains("speed") {
            Conversion::MphToKmh
        } else if n.contains("fuel economy") {
            Conversion::HpHrPerGalToKwhPerL
        } else if n.contains("fuel rate") {
            Conversion::GalPerHrToLPerH
        } else if n.contains("hydraulic flow") {
            Conversion::GalPerMinToLPerMin
        } else if n.contains("weight") {
            Conversion::LbToKg
        } else if n.contains("pull") || n.contains("lift") {
            Conversion::LbfToKn
        } else if n.contains("power") {
            Conversion::HpToKw
        } else {
            panic!("{n}: no expected conversion for this row — extend the test")
        };
        for o in d.observations {
            assert_eq!(o.printed().conversion(), expected, "{n} [{}]", o.edition());
        }
    }
}

/// Row labels are unique — `get` takes the first match, so a duplicate would
/// make lookups silently ambiguous.
#[test]
fn row_labels_are_unique() {
    let mut names: Vec<&str> = JOHN_DEERE_8245R.data.iter().map(|d| d.name).collect();
    let before = names.len();
    names.sort_unstable();
    names.dedup();
    assert_eq!(before, names.len(), "duplicate row label");
}

/// Every figure carries a reading from every declared edition.
#[test]
fn every_edition_reads_every_figure() {
    let years: Vec<u16> = JOHN_DEERE_8245R.editions.iter().map(|e| e.year).collect();
    assert_eq!(years, vec![2016, 2017, 2019]);
    for d in JOHN_DEERE_8245R.data {
        let mut got: Vec<u16> = d.observations.iter().map(|o| o.edition()).collect();
        got.sort_unstable();
        assert_eq!(
            got, years,
            "{}: editions do not match the declared set",
            d.name
        );
    }
}

// ---------------------------------------------------------------- provenance

/// MUTATION: blank any edition field and this fails.
#[test]
fn provenance_is_complete() {
    assert!(!JOHN_DEERE_8245R.editions.is_empty());
    for e in JOHN_DEERE_8245R.editions {
        for (field, value) in [
            ("title", e.title),
            ("url", e.url),
            ("retrieved", e.retrieved),
        ] {
            assert!(
                !value.trim().is_empty(),
                "edition {}: {field} is empty",
                e.year
            );
        }
        assert!(
            e.retrieved.len() == 10 && e.retrieved.starts_with("2026-"),
            "edition {}: retrieval date is not an ISO date",
            e.year
        );
        assert!(
            e.url.contains(&e.year.to_string()),
            "edition {}: url does not name its own year",
            e.year
        );
    }
}

/// Every absence, omission and exclusion says what it would take to close it.
#[test]
fn every_gap_explains_itself() {
    for (label, list) in [
        ("ABSENT", ABSENT),
        ("NOT_TRANSCRIBED", NOT_TRANSCRIBED),
        ("EXCLUDED_EDITIONS", EXCLUDED_EDITIONS),
    ] {
        assert!(!list.is_empty(), "{label} is empty");
        for a in list {
            assert!(
                !a.what.trim().is_empty(),
                "{label}: an entry has no subject"
            );
            assert!(a.why.len() > 80, "{label}/{}: reason too short", a.what);
        }
    }
}

/// ⚠ MISSION names "drawbar **and slip**". The slip half is in no edition.
#[test]
fn no_slip_figure_is_exposed_and_its_absence_is_recorded() {
    for d in JOHN_DEERE_8245R.data {
        assert!(
            !d.name.to_ascii_lowercase().contains("slip"),
            "{}: a slip figure is present, but no source carries one",
            d.name
        );
    }
    assert!(ABSENT.iter().any(|a| a.what.contains("slip")));
}

/// MUTATION: assert a drawbar surface anywhere and this fails.
#[test]
fn no_drawbar_surface_is_asserted() {
    for d in JOHN_DEERE_8245R.data {
        let n = d.name.to_ascii_lowercase();
        for banned in ["concrete", "asphalt", "tilled", "firm soil", "sod"] {
            assert!(!n.contains(banned), "{}: names a test surface", d.name);
        }
    }
    assert!(ABSENT.iter().any(|a| a.what.contains("surface")));
}

/// An excluded edition must say why, and the reason must be the circularity
/// one rather than convenience.
#[test]
fn excluded_editions_are_excluded_for_a_stated_reason() {
    assert_eq!(
        EXCLUDED_EDITIONS.len(),
        2,
        "the excluded-edition list changed"
    );
    for a in EXCLUDED_EDITIONS {
        assert!(
            a.why.contains("reconcile")
                || a.why.contains("check")
                || a.why.contains("independently"),
            "{}: the reason does not explain why reading it would be circular",
            a.what
        );
    }
}

/// Identity is pinned so an edit cannot quietly swap tractors.
#[test]
fn the_test_identifies_itself() {
    assert_eq!(JOHN_DEERE_8245R.summary_no, 963);
    assert_eq!(JOHN_DEERE_8245R.year_tested, 2014);
    assert!(JOHN_DEERE_8245R.make_model.contains("8245R"));
    assert_eq!(JOHN_DEERE_8245R.data.len(), 25);
    let obs: usize = JOHN_DEERE_8245R
        .data
        .iter()
        .map(|d| d.observations.len())
        .sum();
    assert_eq!(obs, 75);
}

/// Printed figures come back as provenance, precision included.
#[test]
fn printed_values_are_returned_as_provenance() {
    let d = datum("drawbar max pull");
    let o2019 = d
        .observations
        .iter()
        .find(|o| o.edition() == 2019)
        .unwrap_or_else(|| panic!("2019 reads this figure"));
    let p = o2019.printed();
    assert!((p.as_printed_us() - 24702.0).abs() < f64::EPSILON);
    assert!(
        (p.as_printed_si() - 109.68).abs() < 1e-9,
        "the 2019 scan's own SI figure"
    );
    assert_eq!(p.us_decimals(), 0);
    assert_eq!(p.si_decimals(), 2);
    // and the resolved value is the one the other editions agree on
    assert!((d.read().agreed().unwrap_or_default() - 109.88).abs() < 1e-9);
}

/// The constructor stores precision rather than inferring it.
#[test]
fn the_constructor_stores_precision_rather_than_inferring_it() {
    let coarse = Printed::new(60.6, 1, 229.3, 1, Conversion::GalPerMinToLPerMin);
    let fine = Printed::new(60.6, 2, 229.3, 2, Conversion::GalPerMinToLPerMin);
    assert!(
        coarse.internally_consistent(),
        "as printed, this reconciles"
    );
    assert!(
        !fine.internally_consistent(),
        "claiming precision the source did not print manufactures a disagreement"
    );
    assert!(coarse.printing_slack() > fine.printing_slack());
}

/// An agreed span is a point, so span arithmetic needs no special case.
#[test]
fn an_agreed_span_is_a_point() {
    for d in JOHN_DEERE_8245R.data {
        let r = d.read();
        let Some(v) = r.agreed() else { continue };
        let (lo, hi) = r.span().unwrap_or_default();
        assert!(
            (lo - hi).abs() < f64::EPSILON && (lo - v).abs() < f64::EPSILON,
            "{}",
            d.name
        );
    }
    assert!(Reading::Unresolved.span().is_none());
}

/// The corrupt-scan summary names exactly the figures the roster does.
///
/// MUTATION: make `with_corrupt_editions` return every row and this fails.
#[test]
fn the_corrupt_scan_summary_matches_the_roster() {
    let mut expected: Vec<&str> = CORRUPT_SCANS.iter().map(|(n, _)| *n).collect();
    expected.dedup();
    assert_eq!(JOHN_DEERE_8245R.with_corrupt_editions(), expected);
}

/// An unknown row label is a miss, not a panic or a wrong row.
#[test]
fn an_unknown_row_label_returns_nothing() {
    assert!(JOHN_DEERE_8245R.get("drawbar wheel slip").is_none());
    assert!(JOHN_DEERE_8245R.get("").is_none());
    assert!(JOHN_DEERE_8245R.get("drawbar max pull").is_some());
}

/// A conflicted reading reports the disagreement's range and yields no value.
///
/// Pins the behaviour callers depend on when two editions genuinely differ:
/// nothing is picked, and the span is wide enough to carry the doubt.
#[test]
fn a_conflicted_reading_carries_its_range_and_no_value() {
    let a = Printed::new(104.27, 2, 77.75, 2, Conversion::HpToKw);
    let b = Printed::new(104.45, 2, 77.88, 2, Conversion::HpToKw);
    let r = reconcile(&[Observation::new(2016, a), Observation::new(2017, b)]);
    assert_eq!(
        r,
        Reading::Conflicted {
            low: 77.75,
            high: 77.88
        }
    );
    assert!(r.agreed().is_none(), "a conflict must not yield a value");
    assert_eq!(r.span(), Some((77.75, 77.88)));
    assert_eq!(r.rejected(), 0, "both scans were internally fine");
    assert_eq!(Reading::Unresolved.rejected(), 0);
}

/// A resolved figure reports how many editions backed it and how many were
/// set aside, so a caller can see the vote rather than just the winner.
#[test]
fn a_resolved_figure_reports_the_vote() {
    let r = datum("drawbar max pull").read();
    assert_eq!(
        r,
        Reading::Agreed {
            si: 109.88,
            agreeing: 2,
            rejected: 1
        },
        "two editions agree on 109.88 and the 2019 scan is set aside"
    );
    let clean = datum("weight as tested").read();
    assert_eq!(clean.rejected(), 0, "no edition mis-scans the weight");
    assert_eq!(clean.agreed(), Some(11512.0));
}
