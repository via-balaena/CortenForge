//! Smoke test for `tests/common/mod.rs::write_verdict_json`.
//!
//! The actual d2c_sr_rematch* consumers (now in the `sim-rl-baselines`
//! crate, which carries its own copy of `common/`) run multi-thousand
//! seconds release-only, so they're not exercised in normal CI. This file is
//! a fast (sub-millisecond) integration test that constructs a
//! synthetic [`TwoMetricOutcome`] and verifies the JSON write +
//! round-trip on its own integration-test crate.
//!
//! Lives at `tests/common_smoke.rs` (its own integration test) so it
//! runs once regardless of how many fixtures `mod common;`. Compiles
//! a separate copy of `common/mod.rs` per integration-test crate per
//! cargo's standard model.
//!
//! Idempotent: writes to a unique fixture name per run; does not
//! collide with the rematch fixtures' artifact paths.

#![allow(
    clippy::expect_used,
    clippy::float_cmp,
    clippy::doc_markdown,
    clippy::let_underscore_must_use
)]

mod common;

use std::fs;
use std::path::PathBuf;

use sim_opt::{BootstrapCi, RematchOutcome, TwoMetricOutcome};

fn synthetic_outcome() -> TwoMetricOutcome {
    let best_ci = BootstrapCi {
        point_estimate: 0.42,
        lower: 0.10,
        upper: 0.74,
        n_resamples: 10_000,
    };
    let final_ci = BootstrapCi {
        point_estimate: -0.05,
        lower: -0.20,
        upper: 0.10,
        n_resamples: 10_000,
    };
    TwoMetricOutcome {
        best_ci,
        best_outcome: best_ci.classify(),
        final_ci,
        final_outcome: final_ci.classify(),
    }
}

fn artifact_path(fixture_name: &str) -> PathBuf {
    let manifest_dir = env!("CARGO_MANIFEST_DIR");
    let mut path = PathBuf::from(manifest_dir);
    path.push("../../..");
    path.push("target/rematch_verdicts");
    path.push(format!("{fixture_name}.json"));
    path
}

#[test]
fn write_verdict_json_creates_artifact_and_roundtrips() {
    let fixture_name = "common_smoke_test";
    let outcome = synthetic_outcome();

    let path = artifact_path(fixture_name);
    // Pre-clean so a leftover from a previous run doesn't false-pass.
    if path.exists() {
        fs::remove_file(&path).expect("pre-clean of stale artifact should succeed");
    }

    common::write_verdict_json(fixture_name, &outcome).expect("smoke write should succeed");

    let json = fs::read_to_string(&path).expect("artifact should exist at the canonical path");
    let parsed: TwoMetricOutcome =
        serde_json::from_str(&json).expect("artifact should be valid JSON for TwoMetricOutcome");

    // Field-level round-trip — TwoMetricOutcome is `Copy` but not
    // `PartialEq`, so compare per-field on the f64 payloads + the
    // RematchOutcome enums (which derive PartialEq).
    assert_eq!(
        parsed.best_ci.point_estimate,
        outcome.best_ci.point_estimate
    );
    assert_eq!(parsed.best_ci.lower, outcome.best_ci.lower);
    assert_eq!(parsed.best_ci.upper, outcome.best_ci.upper);
    assert_eq!(parsed.best_ci.n_resamples, outcome.best_ci.n_resamples);
    assert_eq!(parsed.best_outcome, outcome.best_outcome);
    assert_eq!(
        parsed.final_ci.point_estimate,
        outcome.final_ci.point_estimate
    );
    assert_eq!(parsed.final_ci.lower, outcome.final_ci.lower);
    assert_eq!(parsed.final_ci.upper, outcome.final_ci.upper);
    assert_eq!(parsed.final_ci.n_resamples, outcome.final_ci.n_resamples);
    assert_eq!(parsed.final_outcome, outcome.final_outcome);

    // Sanity on the synthetic CIs landing in the expected three-cell
    // classification space (see Ch 30): `best_ci.lower > 0` →
    // Positive; `final_ci.lower < 0 < final_ci.upper` with point
    // estimate `< 0` → Null. Catches a regression in `classify_outcome`
    // landing in this code path via the round-trip.
    assert_eq!(parsed.best_outcome, RematchOutcome::Positive);
    assert_eq!(parsed.final_outcome, RematchOutcome::Null);

    // Best-effort cleanup; a leftover doesn't break subsequent runs
    // (pre-clean above handles staleness) but keeps `target/` tidy.
    let _ = fs::remove_file(&path);
}
