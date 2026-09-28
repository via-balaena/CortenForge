//! The friction library (`sim_soft::pairing`, soft-contact recon §5c, §16w): every pairing rests on measurements
//! that read as values, gives a verdict's three corners, and marks the corners above the checked friction.

// A corner is one of its measurements' values, copied, so the comparisons are exact; a missing pairing is the test's
// panic.
#![allow(clippy::float_cmp, clippy::expect_used)]

use sim_soft::pairing::{FRICTION_CHECKED_TO, PAIRINGS, Phase, checked};

#[test]
fn every_measurement_is_a_positive_range_with_a_source() {
    assert!(!PAIRINGS.is_empty());
    for pairing in PAIRINGS {
        assert!(!pairing.measurements.is_empty(), "{}", pairing.name);
        for m in pairing.measurements {
            assert!(
                0.0 < m.low && m.low <= m.high && m.high < 5.0,
                "{}: {} to {}",
                pairing.name,
                m.low,
                m.high
            );
            assert!(m.source.contains("https://"), "{}", pairing.name);
            assert!(!m.conditions.is_empty(), "{}", pairing.name);
        }
    }
}

#[test]
fn names_are_unique() {
    for (i, a) in PAIRINGS.iter().enumerate() {
        assert!(
            PAIRINGS[i + 1..].iter().all(|b| b.name != a.name),
            "{} twice",
            a.name
        );
    }
}

#[test]
fn a_verdicts_corners_are_zero_and_the_pairings_span() {
    for pairing in PAIRINGS {
        let [zero, low, high] = pairing.corners();
        assert!(zero == 0.0 && zero < low && low <= high, "{}", pairing.name);
        assert!(
            pairing
                .measurements
                .iter()
                .all(|m| low <= m.low && m.high <= high)
        );
        assert!(pairing.measurements.iter().any(|m| m.low == low));
        assert!(pairing.measurements.iter().any(|m| m.high == high));
    }
    // Every pairing's corners, from §5c's table: dry onset 0.94 and 1.14, sliding 0.61 ± 0.21; a fresh water-based
    // gel 0.18 at onset and 0.104–0.145 sliding; 0.96 at 5 min; a silicone lubricant 0.30 fresh and 1.21 at 20 min.
    let corners: Vec<(&str, [f64; 3])> = PAIRINGS.iter().map(|p| (p.name, p.corners())).collect();
    assert_eq!(
        corners,
        [
            ("silicone on skin, dry", [0.0, 0.40, 1.14]),
            (
                "silicone on skin, water-based gel, fresh",
                [0.0, 0.104, 0.18]
            ),
            (
                "silicone on skin, water-based gel, after 5 min",
                [0.0, 0.96, 0.96]
            ),
            (
                "silicone on skin, silicone lubricant, fresh",
                [0.0, 0.30, 0.30]
            ),
            (
                "silicone on skin, silicone lubricant, after 20 min",
                [0.0, 1.21, 1.21]
            ),
        ]
    );
    // §5c's dry row: onset 0.94 and 1.14, sliding 0.61 ± 0.21.
    let dry = PAIRINGS
        .iter()
        .find(|p| p.name == "silicone on skin, dry")
        .expect("§5c's dry row");
    assert_eq!(dry.corners(), [0.0, 0.40, 1.14]);
    assert!(dry.measurements.iter().any(|m| m.phase == Phase::Onset));
    assert!(dry.measurements.iter().any(|m| m.phase == Phase::Sliding));
}

#[test]
fn friction_above_the_checked_value_is_marked() {
    assert!(checked(0.0) && checked(FRICTION_CHECKED_TO));
    assert!(!checked(FRICTION_CHECKED_TO + 1e-9));
    // Of §5c's pairings, only the fresh lubricants' spans are checked throughout: a silicone lubricant's 0.30 on
    // application sits at the checked value.
    let whole: Vec<&str> = PAIRINGS
        .iter()
        .filter(|p| p.corners().iter().all(|&mu| checked(mu)))
        .map(|p| p.name)
        .collect();
    assert_eq!(
        whole,
        [
            "silicone on skin, water-based gel, fresh",
            "silicone on skin, silicone lubricant, fresh"
        ]
    );
}
