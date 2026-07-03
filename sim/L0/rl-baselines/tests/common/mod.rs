//! Shared test helpers for the d2c_sr_rematch* integration fixtures.
//!
//! Lives at `tests/common/mod.rs` (not `tests/common.rs`) so cargo
//! treats it as a shared module rather than its own integration-test
//! target. Each fixture imports via `mod common;`.

use std::fs;
use std::io;
use std::path::PathBuf;

use sim_opt::TwoMetricOutcome;

/// Persist a rematch verdict to a stable artifact path under
/// `target/rematch_verdicts/<fixture_name>.json`.
///
/// Replaces the `eprintln!`-only verdict reporting that libtest's
/// default capture policy swallows on test pass. `std::fs::write` is
/// unaffected by libtest capture (it's filesystem IO, not stdio), so
/// the verdict survives a clean `cargo test --release` run regardless
/// of whether `--nocapture` / `--show-output` was passed.
///
/// The path is rooted at `CARGO_MANIFEST_DIR/../../../target/...` —
/// resolves to the workspace `target/` from the fixture files (they live
/// at `sim/L0/rl-baselines/tests/d2c_sr_rematch*.rs`, the same three-levels-
/// up depth as the crate's manifest). Directory is created if absent.
///
/// # Errors
///
/// Returns the underlying `io::Error` from `create_dir_all` or `write`
/// if filesystem IO fails. Caller decides whether to propagate (panic
/// in a test harness is acceptable; the rematch protocol's scientific
/// verdict is informational, not load-bearing for the gate per
/// Ch 42 §6 sub-decision (g) shape-α framing).
pub fn write_verdict_json(fixture_name: &str, outcome: &TwoMetricOutcome) -> io::Result<()> {
    let manifest_dir = env!("CARGO_MANIFEST_DIR");
    let mut artifact_dir = PathBuf::from(manifest_dir);
    // sim/L0/rl-baselines → workspace root
    artifact_dir.push("../../..");
    artifact_dir.push("target/rematch_verdicts");
    fs::create_dir_all(&artifact_dir)?;

    let mut artifact_path = artifact_dir;
    artifact_path.push(format!("{fixture_name}.json"));
    let json = serde_json::to_string_pretty(outcome)
        .map_err(|e| io::Error::new(io::ErrorKind::InvalidData, e))?;
    fs::write(&artifact_path, json)?;
    Ok(())
}
