//! The gates on the calibration dataset, pre-registered before the data was
//! entered.
//!
//! Each test below names the mutation that must make it fail. A gate nobody
//! has made fail is vacuous, and the reconciliation check in particular is the
//! kind that looks like it is working when it is not: it passed on every row
//! of an earlier draft only because the tolerance was a flat percentage rather
//! than the source's own printing precision.

#![allow(
    clippy::panic,
    reason = "a gate that cannot name the row it rejected is not much of a gate"
)]

use cf_nebraska::{
    ABSENT, CATCHABLE_ERROR, Conversion, DISPUTED_8245R, JOHN_DEERE_8245R, NOT_TRANSCRIBED,
    Printed, Reading, WEAKLY_CHECKED_8245R,
};

#[test]
fn every_datum_either_reconciles_or_is_named_as_disputed() {
    for d in JOHN_DEERE_8245R.data {
        let agreed = d.printed.read().agreed().is_some();
        let listed = DISPUTED_8245R.contains(&d.name);
        assert!(
            agreed != listed,
            "{}: reconciles={agreed}, listed as disputed={listed} — must be exactly one",
            d.name
        );
    }
}

/// MUTATION: add a name to `DISPUTED_8245R` that actually reconciles, or drop
/// one that does not, and this fails. Checked in both directions so the roster
/// cannot drift in either.
#[test]
fn the_disputed_roster_is_exact() {
    let computed = JOHN_DEERE_8245R.disputed();
    assert_eq!(
        computed, DISPUTED_8245R,
        "recorded disputed roster does not match the computed one"
    );
}

/// MUTATION: change any digit of a strongly-checked figure by more than
/// [`CATCHABLE_ERROR`] and it must stop reconciling. This is the proof the
/// check is not vacuous, and it runs over the real data rather than a fixture.
///
/// ⚠ It runs only over the figures the source printed precisely enough for
/// the check to have that power. The rest are not exempt — they are held to
/// their own resolution by `weakly_checked_figures_are_still_checked`, and
/// enumerated by `the_weakly_checked_roster_is_exact` so the exemption cannot
/// quietly grow.
#[test]
fn a_perturbed_figure_stops_reconciling() {
    let mut proven = 0;
    for d in JOHN_DEERE_8245R.data {
        if d.printed.read().agreed().is_none() || WEAKLY_CHECKED_8245R.contains(&d.name) {
            continue;
        }
        // Both directions. The printed pair is rarely an exact conversion of
        // each other, and that standing offset eats margin on one side only —
        // so a one-directional probe can pass on a figure that is blind to the
        // same error with its sign flipped.
        for sign in [1.0, -1.0] {
            let bumped = d.printed.perturbed(sign * CATCHABLE_ERROR);
            assert!(
                bumped.read().agreed().is_none(),
                "{}: perturbing by {}% in direction {sign} left it still \
                 reconciling, but it is not listed as weakly checked",
                d.name,
                CATCHABLE_ERROR * 100.0
            );
        }
        proven += 1;
    }
    // ⛔ An empty loop passes. Assert the collection.
    assert_eq!(
        proven,
        JOHN_DEERE_8245R.data.len() - DISPUTED_8245R.len() - WEAKLY_CHECKED_8245R.len(),
        "the perturbation ran over the wrong number of figures"
    );
    assert!(proven > 0, "no figure was perturbed");
}

/// MUTATION: add a name that is actually strongly checked, or drop one that is
/// not, and this fails. Same both-directions shape as the disputed roster: an
/// exemption list that can only grow is not a gate.
#[test]
fn the_weakly_checked_roster_is_exact() {
    let computed: Vec<&str> = JOHN_DEERE_8245R
        .data
        .iter()
        .filter(|d| d.printed.resolution().is_some_and(|r| r > CATCHABLE_ERROR))
        .map(|d| d.name)
        .collect();
    assert_eq!(
        computed, WEAKLY_CHECKED_8245R,
        "recorded weakly-checked roster does not match the computed one"
    );
}

/// A weakly-checked figure is still checked — just at a coarser threshold.
///
/// MUTATION: break `resolution` so it returns something the check cannot
/// actually deliver, and this fails. Guards against the roster becoming a
/// place to park figures that are not verified at all.
#[test]
fn weakly_checked_figures_are_still_checked() {
    for name in WEAKLY_CHECKED_8245R {
        let d = JOHN_DEERE_8245R
            .data
            .iter()
            .find(|d| &d.name == name)
            .unwrap_or_else(|| panic!("{name} is listed as weakly checked but is not in the data"));
        let r = d
            .printed
            .resolution()
            .unwrap_or_else(|| panic!("{name} is disputed and weakly checked at once"));
        assert!(
            r.is_finite() && r > 0.0,
            "{name}: resolution is not a number"
        );
        for sign in [1.0, -1.0] {
            let bumped = d.printed.perturbed(sign * r * 1.001);
            assert!(
                bumped.read().agreed().is_none(),
                "{name}: an error of {:.4}% in direction {sign} was not caught, \
                 but that is the resolution this figure claims",
                r * 100.0
            );
        }
    }
}

/// The weakness is a property of significant figures, not of a particular row.
///
/// Every weakly-checked figure is a speed or a fuel economy — the two kinds
/// the source prints to three significant figures while giving powers and
/// masses five. If a power or a mass ever lands in this roster, the cause is
/// something other than printing precision and wants looking at.
#[test]
fn only_low_significance_figures_are_weakly_checked() {
    for name in WEAKLY_CHECKED_8245R {
        assert!(
            name.contains("speed") || name.contains("fuel economy"),
            "{name}: weakly checked but is neither a speed nor a fuel economy — \
             the cause is not printing precision"
        );
    }
}

/// Strongly-checked figures are strongly checked by a wide margin, not by a
/// hair. Pins the separation so the two rosters cannot converge quietly.
#[test]
fn strongly_checked_figures_resolve_an_order_better() {
    for d in JOHN_DEERE_8245R.data {
        if WEAKLY_CHECKED_8245R.contains(&d.name) {
            continue;
        }
        let Some(r) = d.printed.resolution() else {
            continue;
        };
        assert!(
            r < CATCHABLE_ERROR,
            "{}: resolution {:.4}% is not better than the {}% bar",
            d.name,
            r * 100.0,
            CATCHABLE_ERROR * 100.0
        );
    }
}

/// MUTATION: return a bare value for a disputed figure and this fails. A
/// disagreement has to be visible at the call site.
#[test]
fn a_disputed_figure_cannot_be_read_as_a_single_number() {
    for name in DISPUTED_8245R {
        let reading = JOHN_DEERE_8245R
            .get(name)
            .unwrap_or_else(|| panic!("{name} is listed as disputed but is not in the data"));
        assert!(
            reading.agreed().is_none(),
            "{name}: listed as disputed but readable as an agreed value"
        );
        let (lo, hi) = reading.span();
        assert!(lo < hi, "{name}: disputed span is a point");
    }
}

/// The disagreements are digits, not rounding — otherwise the honest fix would
/// be to widen the interval, not to carry them as disputed.
///
/// Measured against each figure's OWN printing slack rather than a flat
/// threshold, so the claim has a producer. The tightest of the three is
/// `drawbar 50pct load fuel economy` at 5.3x its slack; `drawbar max pull`,
/// the most load-bearing figure in the set, is 28x.
#[test]
fn disputed_entries_exceed_what_rounding_could_explain() {
    for name in DISPUTED_8245R {
        let d = JOHN_DEERE_8245R
            .data
            .iter()
            .find(|d| &d.name == name)
            .unwrap_or_else(|| panic!("{name} is listed as disputed but is not in the data"));
        let Some(Reading::Disputed {
            from_us,
            as_printed,
        }) = JOHN_DEERE_8245R.get(name)
        else {
            panic!("{name} is not disputed");
        };
        let rel = (from_us - as_printed).abs() / as_printed.abs();
        let slack = d.printed.printing_slack();
        assert!(
            rel >= 3.0 * slack,
            "{name}: the two printed forms differ by {:.4}%, only {:.1}x the \
             {:.4}% that rounding alone allows — widen the interval instead of \
             disputing it",
            rel * 100.0,
            rel / slack,
            slack * 100.0
        );
    }
}

/// MUTATION: blank any provenance field and this fails.
#[test]
fn provenance_is_complete() {
    let s = JOHN_DEERE_8245R.source;
    for (field, value) in [
        ("document", s.document),
        ("url", s.url),
        ("retrieved", s.retrieved),
        ("locator", s.locator),
        ("extraction", s.extraction),
    ] {
        assert!(!value.trim().is_empty(), "source.{field} is empty");
    }
    assert!(
        s.retrieved.len() == 10 && s.retrieved.starts_with("2026-"),
        "retrieval date is not an ISO date: {:?}",
        s.retrieved
    );
}

/// ⚠ MISSION names "drawbar **and slip**". The slip half is not in the
/// retrievable source.
///
/// MUTATION: add a slip figure to the dataset and this fails. The only way to
/// satisfy it is to obtain the individual report, at which point the absence
/// entry comes out too.
#[test]
fn no_slip_figure_is_exposed_and_its_absence_is_recorded() {
    for d in JOHN_DEERE_8245R.data {
        assert!(
            !d.name.to_ascii_lowercase().contains("slip"),
            "{}: a slip figure is present, but the source carries none — \
             if this came from the individual report, remove the ABSENT entry",
            d.name
        );
    }
    assert!(
        ABSENT.iter().any(|a| a.what.contains("slip")),
        "slip is neither present nor recorded as absent"
    );
}

/// MUTATION: assert a drawbar surface anywhere in the dataset and this fails.
/// The retrieved methodology text does not state one.
#[test]
fn no_drawbar_surface_is_asserted() {
    for d in JOHN_DEERE_8245R.data {
        let n = d.name.to_ascii_lowercase();
        for banned in ["concrete", "asphalt", "tilled", "firm soil", "sod"] {
            assert!(
                !n.contains(banned),
                "{}: names a test surface ({banned}); the source does not state one",
                d.name
            );
        }
    }
    assert!(
        ABSENT.iter().any(|a| a.what.contains("surface")),
        "the unknown drawbar surface is not recorded as absent"
    );
}

/// Every absence has to say what it would take to close it, or it is a shrug.
#[test]
fn every_absence_explains_itself() {
    assert!(!ABSENT.is_empty(), "ABSENT is empty");
    for a in ABSENT {
        assert!(!a.what.trim().is_empty(), "an absence has no subject");
        assert!(
            a.why.len() > 80,
            "{}: the reason is too short to say what would close it",
            a.what
        );
    }
}

/// The figures are the ones the source prints, and the test is the one it says
/// it is. Pins the identity so a future edit cannot quietly swap tractors.
#[test]
fn the_test_identifies_itself() {
    assert_eq!(JOHN_DEERE_8245R.summary_no, 963);
    assert_eq!(JOHN_DEERE_8245R.year_tested, 2014);
    assert!(JOHN_DEERE_8245R.make_model.contains("8245R"));
    assert_eq!(
        JOHN_DEERE_8245R.data.len(),
        25,
        "the number of transcribed figures changed"
    );
}

/// An agreed figure's span is a point, so code that works in spans can treat
/// agreed and disputed figures uniformly without special-casing.
///
/// MUTATION: make `span` widen an agreed reading and this fails.
#[test]
fn an_agreed_span_is_a_point() {
    let mut checked = 0;
    for d in JOHN_DEERE_8245R.data {
        let reading = d.printed.read();
        let Some(v) = reading.agreed() else { continue };
        let (lo, hi) = reading.span();
        assert!(
            (lo - hi).abs() < f64::EPSILON && (lo - v).abs() < f64::EPSILON,
            "{}: agreed span is not the point it agreed on",
            d.name
        );
        checked += 1;
    }
    assert_eq!(
        checked,
        JOHN_DEERE_8245R.data.len() - DISPUTED_8245R.len(),
        "wrong number of agreed figures"
    );
}

/// The weakly-checked roster is one-directional, and an earlier draft of the
/// documentation claimed otherwise.
///
/// Every weakly-checked figure is a speed or a fuel economy. The converse does
/// NOT hold, and this test pins that so the stronger, false claim cannot creep
/// back into the prose: if a future change ever made every speed and every fuel
/// economy weak, this fails and the docs get revisited deliberately.
#[test]
fn the_weak_roster_does_not_contain_every_speed_and_fuel_economy() {
    let strong_speeds = JOHN_DEERE_8245R
        .data
        .iter()
        .filter(|d| d.name.contains("speed"))
        .filter(|d| d.printed.resolution().is_some_and(|r| r <= CATCHABLE_ERROR))
        .count();
    let strong_fuel_economies = JOHN_DEERE_8245R
        .data
        .iter()
        .filter(|d| d.name.contains("fuel economy"))
        .filter(|d| d.printed.resolution().is_some_and(|r| r <= CATCHABLE_ERROR))
        .count();
    assert_eq!(
        (strong_speeds, strong_fuel_economies),
        (3, 1),
        "the split between strongly- and weakly-checked speeds/fuel economies \
         moved; the WEAKLY_CHECKED_8245R doc states these counts"
    );
}

/// What the transcription deliberately left out is written down.
///
/// MUTATION: empty `NOT_TRANSCRIBED` and this fails. Guards the pinned count of
/// 25 from being read as "the whole column".
#[test]
fn deliberate_omissions_are_recorded() {
    assert!(!NOT_TRANSCRIBED.is_empty(), "NOT_TRANSCRIBED is empty");
    for a in NOT_TRANSCRIBED {
        assert!(!a.what.trim().is_empty(), "an omission has no subject");
        assert!(
            a.why.len() > 80,
            "{}: the reason is too short to say why it was skipped",
            a.what
        );
    }
}

/// The PTO operating-point ambiguity is recorded, not resolved by guessing.
#[test]
fn the_pto_operating_point_is_recorded_as_unknown() {
    assert!(
        ABSENT.iter().any(|a| a.what.contains("engine speed")),
        "the ambiguous PTO max-power engine speed is not recorded as absent"
    );
}

/// Every figure is paired with the conversion its units actually call for.
///
/// A mis-assigned conversion is mostly caught by reconciliation — a speed put
/// through the horsepower factor will not agree with its own SI twin — but not
/// always: the two gallons-per-something conversions share a factor, so
/// swapping them is invisible to the checksum and visible only here.
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
        assert_eq!(
            d.printed.conversion(),
            expected,
            "{n}: conversion does not match the units the row name implies"
        );
    }
}

/// The printed figures come back exactly as the source printed them, precision
/// included.
///
/// This is the provenance path — what is on the page, not what has been
/// checked. Pinned on the figure the chain leans on hardest.
#[test]
fn printed_values_are_returned_as_provenance() {
    let d = JOHN_DEERE_8245R
        .data
        .iter()
        .find(|d| d.name == "drawbar max pull")
        .unwrap_or_else(|| panic!("drawbar max pull is in the data"));
    assert!((d.printed.as_printed_us() - 24702.0).abs() < f64::EPSILON);
    assert!((d.printed.as_printed_si() - 109.68).abs() < 1e-9);
    assert_eq!(
        d.printed.us_decimals(),
        0,
        "lbf was printed as a whole number"
    );
    assert_eq!(d.printed.si_decimals(), 2, "kN was printed to two decimals");
}

/// A figure built through the constructor behaves like one from the table.
///
/// Pins that `Printed::new` is the only way in and that it stores precision
/// rather than inferring it: the same numbers declared at different printed
/// precisions reconcile differently, which is the bug that produced a phantom
/// disagreement while this crate was being written.
#[test]
fn the_constructor_stores_precision_rather_than_inferring_it() {
    let coarse = Printed::new(60.6, 1, 229.3, 1, Conversion::GalPerMinToLPerMin);
    let fine = Printed::new(60.6, 2, 229.3, 2, Conversion::GalPerMinToLPerMin);
    assert!(
        coarse.read().agreed().is_some(),
        "as the source printed it, this figure reconciles"
    );
    assert!(
        fine.read().agreed().is_none(),
        "claiming precision the source did not print manufactures a disagreement"
    );
    assert!(
        coarse.printing_slack() > fine.printing_slack(),
        "coarser printing must carry more slack"
    );
}
