//! Rematch statistical analysis machinery.
//!
//! This module implements Chapter 32's rematch protocol as a
//! library surface: bootstrap confidence intervals on the
//! difference of means and medians, Pearson's bimodality
//! coefficient, the three-outcome classifier from Ch 30, and
//! the folded-pilot driver [`run_rematch`].
//!
//! The public functions are shaped to be reused by Ch 30's
//! pre-committed null follow-ups — a richer-proposal SA variant
//! and a Parallel Tempering algorithm — both of which would run
//! their own instance of the same protocol with different
//! algorithms.

use rand::Rng;
use serde::{Deserialize, Serialize};
use sim_ml_chassis::{Algorithm, Competition, CompetitionResult, EnvError, TaskConfig};
use sim_thermostat::prf::splitmix64;

// ── Rematch constants ────────────────────────────────────────────────────

/// Pre-registered master seed from Ch 32 Decision 4. Matches
/// `d2c_cem_training.rs:69`'s `SEED_BASE` literal and Ch 23
/// §1.2's example master value.
pub const REMATCH_MASTER_SEED: u64 = 20_260_412;

/// Initial batch size picked by Ch 32 Decision 3.
pub const N_INITIAL: usize = 10;

/// Expanded batch size picked by Ch 32 Decision 3 (only used
/// when the initial batch classifies as `Ambiguous`).
pub const N_EXPANDED: usize = 20;

/// Task name the rematch uses inside `Competition`'s task
/// registry. Fixed so `replicate_best_rewards` calls find the
/// task consistently across the initial and expanded batches.
pub const REMATCH_TASK_NAME: &str = "d2c-sr-rematch";

// ── BootstrapCi + RematchOutcome ─────────────────────────────────────────

/// A bootstrap confidence interval on a difference statistic,
/// with the point estimate and both bounds.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct BootstrapCi {
    /// Observed difference-of-statistic from the original
    /// samples, not from the bootstrap distribution.
    pub point_estimate: f64,
    /// Lower bound (2.5th percentile for a two-sided 95% CI).
    pub lower: f64,
    /// Upper bound (97.5th percentile).
    pub upper: f64,
    /// Number of bootstrap resamples drawn.
    pub n_resamples: usize,
}

impl BootstrapCi {
    /// Classify this CI into one of Ch 30's three outcomes per
    /// Ch 32 §3.3's table.
    #[must_use]
    pub fn classify(&self) -> RematchOutcome {
        classify_outcome(self.lower, self.upper, self.point_estimate)
    }
}

/// One of Ch 30's three meaningful outcomes for the rematch.
///
/// Per Ch 31 §3.2: "a 'weak positive' or 'near-null' hedge is
/// not on the menu — Ch 30 deliberately specified three
/// outcomes."
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum RematchOutcome {
    /// CI lower bound > 0: SA reliably outperforms CEM.
    Positive,
    /// CI upper bound <= 0, or CI straddles zero with point
    /// estimate <= 0. SA does not outperform CEM.
    Null,
    /// CI straddles zero with point estimate > 0. The margin
    /// is within the seed-variance envelope; triggers the
    /// folded-pilot expansion from `N_INITIAL` to `N_EXPANDED`.
    Ambiguous,
}

/// Dual-metric verdict returned by [`run_rematch`] under the
/// Ch 51 metric-sensitivity amendment.
///
/// Both metrics pass through the same Ch 32 §3.2/§3.3
/// bootstrap-and-classify pipeline: one pair of
/// `(BootstrapCi, RematchOutcome)` for `RunResult::best_reward`
/// (the chassis primitive Ch 24 shipped) and one pair for
/// `RunResult::final_reward` (the converged-policy signal Ch 51
/// §2.3's interpretation framework treats as the primary
/// operationalization of Ch 30's "resolves the peak" question).
///
/// The `best_reward` side of the struct has exactly the shape
/// and semantics the old `RematchOutcome` return value had
/// before Ch 51; the amendment is additive — it extends the
/// protocol to run the same pipeline a second time in parallel
/// on `final_reward`, not to replace `best_reward` as
/// `Competition::run_replicates`'s default surface.  See
/// Ch 51 §2.1 for the amendment's scope and Ch 51 §2.3 for the
/// nine-cell interpretation framework a human reader applies to
/// the `(best_outcome, final_outcome)` pair.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct TwoMetricOutcome {
    /// Bootstrap CI on `mean(SA best) - mean(CEM best)`.
    pub best_ci: BootstrapCi,
    /// Ch 30 classification derived from `best_ci`.
    pub best_outcome: RematchOutcome,
    /// Bootstrap CI on `mean(SA final) - mean(CEM final)`.
    pub final_ci: BootstrapCi,
    /// Ch 30 classification derived from `final_ci`.
    pub final_outcome: RematchOutcome,
}

// ── Classification ───────────────────────────────────────────────────────

/// Ch 30's three-outcome classifier per Ch 32 §3.3's table.
///
/// Takes the CI bounds and point estimate directly — the free
/// function is useful for callers that want to classify without
/// constructing a [`BootstrapCi`] (e.g., for testing edge cases).
///
/// The arms are kept separate (rather than merged via `|`) so
/// each `(lower_pos, upper_pos, point_pos)` boolean triple can
/// carry its own semantic comment in-line with the verdict.
// match_same_arms: each arm is deliberately spelled out so the
// per-arm `(lower_pos, upper_pos, point_pos)` comment stays
// adjacent to the outcome it maps to; collapsing them via `|`
// would fuse comments that describe distinct cases.
#[must_use]
#[allow(clippy::match_same_arms)]
pub fn classify_outcome(lower: f64, upper: f64, point_estimate: f64) -> RematchOutcome {
    let lower_pos = lower > 0.0;
    let upper_pos = upper > 0.0;
    let point_pos = point_estimate > 0.0;
    match (lower_pos, upper_pos, point_pos) {
        // Strict-positive CI: SA reliably outperforms CEM.
        (true, true, _) => RematchOutcome::Positive,
        // Strict-negative CI: SA reliably does not outperform.
        (false, false, _) => RematchOutcome::Null,
        // Straddle with positive point: margin within seed
        // variance, triggers folded-pilot expansion.
        (false, true, true) => RematchOutcome::Ambiguous,
        // Straddle with non-positive point: no SA advantage.
        (false, true, false) => RematchOutcome::Null,
        // Impossible under a well-formed CI where `lower <= upper`
        // by construction; treat as `Null` defensively.
        (true, false, _) => RematchOutcome::Null,
    }
}

// ── Bootstrap: means ─────────────────────────────────────────────────────

/// Number of bootstrap resamples. Ch 32 Decision 1 / §3.2.
const B: usize = 10_000;

/// Percentile bootstrap CI on the difference of means between
/// two samples. Ch 32 §3.2 pseudocode implementation.
///
/// The point estimate is `mean(r_a) - mean(r_b)`. Paired
/// resampling draws `n_a` samples with replacement from `r_a`
/// and `n_b` samples with replacement from `r_b`, independently,
/// once per bootstrap iteration. The 2.5th and 97.5th percentiles
/// of the `B`-element bootstrap distribution become the CI bounds.
///
/// # Panics
///
/// Panics if either input slice is empty. The rematch protocol
/// at `run_rematch` guarantees non-empty inputs at every call
/// site, so the panic is a defensive guard rather than a runtime
/// condition.
// `B` (bootstrap iterations) and slice lengths are usize cast to f64 for
// percentile indexing; B = 10_000 and rematch slices are <= a few hundred,
// far below f64's 2^52 mantissa ceiling.
#[allow(
    clippy::cast_precision_loss,
    clippy::cast_sign_loss,
    clippy::cast_possible_truncation
)]
pub fn bootstrap_diff_means(r_a: &[f64], r_b: &[f64], rng: &mut impl Rng) -> BootstrapCi {
    assert!(!r_a.is_empty(), "bootstrap_diff_means: r_a is empty");
    assert!(!r_b.is_empty(), "bootstrap_diff_means: r_b is empty");

    let n_a = r_a.len();
    let n_b = r_b.len();

    let point_estimate = mean(r_a) - mean(r_b);

    let mut diffs: Vec<f64> = Vec::with_capacity(B);
    for _ in 0..B {
        let resample_mean_a: f64 =
            (0..n_a).map(|_| r_a[rng.random_range(0..n_a)]).sum::<f64>() / (n_a as f64);
        let resample_mean_b: f64 =
            (0..n_b).map(|_| r_b[rng.random_range(0..n_b)]).sum::<f64>() / (n_b as f64);
        diffs.push(resample_mean_a - resample_mean_b);
    }
    diffs.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));

    let lower_idx = (0.025 * B as f64) as usize;
    let upper_idx = (0.975 * B as f64) as usize;

    BootstrapCi {
        point_estimate,
        lower: diffs[lower_idx],
        upper: diffs[upper_idx],
        n_resamples: B,
    }
}

// ── Bootstrap: medians ───────────────────────────────────────────────────

/// Percentile bootstrap CI on the difference of medians.
///
/// Identical shape to [`bootstrap_diff_means`] with `median`
/// substituted for `mean` at three sites: the point estimate
/// and both per-resample statistics.
///
/// Used by the rematch's bimodality contingency: if either of
/// the two input distributions has `bimodality_coefficient >
/// 5/9`, the rematch protocol at Ch 32 §6.3 substitutes
/// medians for means at the bootstrap step.
///
/// # Panics
///
/// Panics if either input slice is empty (defensive guard).
// Same precision-loss reasoning as `bootstrap_diff_means`: B = 10_000 and
// slice lengths stay far below f64's 2^52 mantissa ceiling.
#[allow(
    clippy::cast_precision_loss,
    clippy::cast_sign_loss,
    clippy::cast_possible_truncation
)]
pub fn bootstrap_diff_medians(r_a: &[f64], r_b: &[f64], rng: &mut impl Rng) -> BootstrapCi {
    assert!(!r_a.is_empty(), "bootstrap_diff_medians: r_a is empty");
    assert!(!r_b.is_empty(), "bootstrap_diff_medians: r_b is empty");

    let n_a = r_a.len();
    let n_b = r_b.len();

    let point_estimate = median(r_a) - median(r_b);

    let mut diffs: Vec<f64> = Vec::with_capacity(B);
    let mut resample_a = vec![0.0; n_a];
    let mut resample_b = vec![0.0; n_b];
    for _ in 0..B {
        for slot in &mut resample_a {
            *slot = r_a[rng.random_range(0..n_a)];
        }
        for slot in &mut resample_b {
            *slot = r_b[rng.random_range(0..n_b)];
        }
        diffs.push(median(&resample_a) - median(&resample_b));
    }
    diffs.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));

    let lower_idx = (0.025 * B as f64) as usize;
    let upper_idx = (0.975 * B as f64) as usize;

    BootstrapCi {
        point_estimate,
        lower: diffs[lower_idx],
        upper: diffs[upper_idx],
        n_resamples: B,
    }
}

// ── Bimodality coefficient ───────────────────────────────────────────────

/// Pearson's bimodality coefficient in the SAS small-sample
/// corrected form.
///
/// `BC > 5/9` indicates bimodality (or strong skew); values
/// below the threshold indicate unimodality. The formula uses
/// biased sample moments (dividing by `n`) with the
/// `3 * (n - 1)^2 / ((n - 2) * (n - 3))` correction applied at
/// the denominator level — the SAS convention Ch 32 §6.2 picks.
///
/// Guards against near-constant inputs (where the second moment
/// is zero or near-zero) by returning `0.0` defensively.
///
/// # Panics
///
/// Panics if `values.len() < 4`. The denominator has a
/// `(n - 2) * (n - 3)` factor, and the rematch protocol
/// guarantees `n = 10` or `n = 20` at every call site.
// cast_precision_loss: `n = values.len() as f64` is at most 20 at
// every call site (Ch 32 Decision 3); f64 is exact for all such
// integer magnitudes.
#[must_use]
#[allow(clippy::cast_precision_loss)]
pub fn bimodality_coefficient(values: &[f64]) -> f64 {
    assert!(
        values.len() >= 4,
        "bimodality_coefficient requires n >= 4, got {}",
        values.len(),
    );
    let n = values.len() as f64;
    let mean = values.iter().sum::<f64>() / n;
    let m2: f64 = values.iter().map(|v| (v - mean).powi(2)).sum::<f64>() / n;
    let m3: f64 = values.iter().map(|v| (v - mean).powi(3)).sum::<f64>() / n;
    let m4: f64 = values.iter().map(|v| (v - mean).powi(4)).sum::<f64>() / n;

    // Sample skewness g = m3 / m2^(3/2). Sample excess kurtosis
    // kappa = m4 / m2^2 - 3. Both guard against m2 ~ 0.
    let g = if m2 > 0.0 { m3 / m2.powf(1.5) } else { 0.0 };
    let kappa = if m2 > 0.0 { m4 / (m2 * m2) - 3.0 } else { 0.0 };

    let numerator = g.mul_add(g, 1.0);
    let correction = (3.0 * (n - 1.0).powi(2)) / ((n - 2.0) * (n - 3.0));
    let denominator = kappa + correction;

    if denominator > 0.0 {
        numerator / denominator
    } else {
        // Degenerate case: constant or near-constant input.
        // Return 0.0 (unimodal) defensively.
        0.0
    }
}

// ── Bimodality-aware classification pipeline ─────────────────────────────

/// Apply Ch 32's test family and classification rule to a pair
/// of replicate reward vectors.
///
/// Checks bimodality first via `bimodality_coefficient`; if
/// either vector exceeds the `5/9` threshold, substitutes
/// medians for means at the bootstrap step. Otherwise uses
/// means.
///
/// The returned outcome is in **SA-vs-CEM orientation**:
/// positive means SA's statistic exceeds CEM's statistic.
///
/// Returns both the [`BootstrapCi`] and the classified
/// [`RematchOutcome`] so the driver can emit the CI alongside the
/// verdict in the run-end summary without recomputing the bootstrap.
fn test_and_classify(
    r_cem: &[f64],
    r_sa: &[f64],
    rng: &mut impl Rng,
) -> (BootstrapCi, RematchOutcome) {
    let bc_cem = bimodality_coefficient(r_cem);
    let bc_sa = bimodality_coefficient(r_sa);
    let ci = if bc_cem > 5.0 / 9.0 || bc_sa > 5.0 / 9.0 {
        bootstrap_diff_medians(r_sa, r_cem, rng)
    } else {
        bootstrap_diff_means(r_sa, r_cem, rng)
    };
    let outcome = ci.classify();
    (ci, outcome)
}

/// Emit the rematch's dual-metric per-replicate summary to stderr.
///
/// Printed once at the end of [`run_rematch_with_runner`], after the
/// folded-pilot path has settled on its four final replicate vectors.
/// The layout stacks two tables — one for `best_reward`, one for
/// `final_reward` — each followed by its means, CI, and
/// classification, then prints a joint footer naming the
/// `(best_outcome, final_outcome)` pair.  A reader skimming the
/// end of the log sees the exact data both bootstraps saw and the
/// two verdicts side-by-side.
///
/// The header paragraph points the reader at Ch 51 §2.3's
/// nine-cell interpretation framework, which is where the joint
/// verdict is actually read.  Per Ch 51 §2, this code deliberately
/// does not emit a Ch 30 reading (Positive / Null / Split /
/// Expand) derived from the pair — the framework lives in the
/// chapter, applied by a human reader, not in the machine output.
fn emit_rematch_summary(
    outcome: &TwoMetricOutcome,
    r_cem_best: &[f64],
    r_sa_best: &[f64],
    r_cem_final: &[f64],
    r_sa_final: &[f64],
) {
    eprintln!("\n--- Rematch summary: dual-metric (Ch 51 §2) ---");
    eprintln!(
        "Per Ch 51 §2.3, final_reward is the primary operationalization of \"resolves the peak\";"
    );
    eprintln!(
        "best_reward is retained as a complementary signal about exploration reach.  A human"
    );
    eprintln!(
        "reader applies Ch 51 §2.3's nine-cell framework to the (best, final) pair printed below."
    );

    emit_metric_block(
        "best_reward",
        "max-across-epochs",
        r_cem_best,
        r_sa_best,
        &outcome.best_ci,
        outcome.best_outcome,
    );
    emit_metric_block(
        "final_reward",
        "last-epoch mean_reward",
        r_cem_final,
        r_sa_final,
        &outcome.final_ci,
        outcome.final_outcome,
    );

    eprintln!();
    eprintln!(
        "Joint (best, final) = ({:?}, {:?}) — see Ch 51 §2.3 for the reading.",
        outcome.best_outcome, outcome.final_outcome,
    );
}

/// One metric's slice of the dual-metric summary.
///
/// Prints a titled per-replicate table, followed by means, the
/// bootstrap CI on `mean(SA) - mean(CEM)`, and the Ch 30
/// classification.  Called once per metric by
/// [`emit_rematch_summary`].
fn emit_metric_block(
    metric: &str,
    shape_hint: &str,
    r_cem: &[f64],
    r_sa: &[f64],
    ci: &BootstrapCi,
    outcome: RematchOutcome,
) {
    eprintln!();
    eprintln!("[{metric} — {shape_hint} per replicate]");
    eprintln!(
        "{:>3}  {:>12}  {:>12}  {:>14}",
        "rep",
        format!("CEM {metric}"),
        format!("SA {metric}"),
        "SA-CEM diff",
    );
    for (i, (c, s)) in r_cem.iter().zip(r_sa.iter()).enumerate() {
        eprintln!("{:>3}  {:>12.2}  {:>12.2}  {:>+14.2}", i, c, s, s - c);
    }
    eprintln!();
    eprintln!("mean(CEM) = {:.4}", mean(r_cem));
    eprintln!("mean(SA)  = {:.4}", mean(r_sa));
    eprintln!(
        "bootstrap CI on mean(SA) - mean(CEM): point={:+.4}, [{:+.4}, {:+.4}], B={}",
        ci.point_estimate, ci.lower, ci.upper, ci.n_resamples,
    );
    eprintln!("classification: {outcome:?}");
}

// ── run_rematch: the folded-pilot driver ─────────────────────────────────

/// Run the folded-pilot rematch protocol end-to-end.
///
/// This is the public driver: it runs `N_INITIAL` replicates,
/// classifies the result, expands to `N_EXPANDED` if ambiguous,
/// and returns the final classified outcome. Seeds are derived
/// from [`REMATCH_MASTER_SEED`] via
/// [`sim_thermostat::prf::splitmix64`], matching the
/// `d2c_cem_training.rs:69` `SEED_BASE` pattern.
///
/// # Parameters
///
/// - `competition`: the `Competition` runner pre-configured
///   with the rematch's `Steps(16M)` budget and `n_envs = 32`.
/// - `task`: the single `TaskConfig` whose `build_fn` closure
///   uses its `seed: u64` parameter to construct a fresh
///   `VecEnv` with a freshly-seeded `LangevinThermostat` on
///   each call. Under the Ch 42 §2 patched `TaskConfig::build_fn`
///   signature, each replicate's seed threads through to the
///   physics noise sequence.
/// - `algorithm_builders`: slice of builder closures producing
///   `Box<dyn Algorithm>`. Rematch requires exactly two: CEM
///   and SA at matched complexity, in that order.
/// - `bootstrap_rng`: the RNG used for bootstrap resampling.
///   Independent of the per-replicate training RNGs.
///
/// # Errors
///
/// Propagates any `EnvError` from `Competition::run_replicates`.
pub fn run_rematch(
    competition: &Competition,
    task: &TaskConfig,
    algorithm_builders: &[&dyn Fn(&TaskConfig) -> Box<dyn Algorithm>],
    bootstrap_rng: &mut impl Rng,
) -> Result<TwoMetricOutcome, EnvError> {
    run_rematch_with_runner(
        |seeds| competition.run_replicates(std::slice::from_ref(task), algorithm_builders, seeds),
        bootstrap_rng,
    )
}

/// Folded-pilot control flow, parameterized over the
/// replicate-runner. Production passes a closure delegating to
/// `Competition::run_replicates`; tests pass a mock that returns
/// pre-built `CompetitionResult`s without running physics.
///
/// Keeping this helper separate from [`run_rematch`] lets the
/// unit tests exercise the folded-pilot branching (initial batch
/// only if unambiguous, initial + expansion if ambiguous) with
/// zero env construction and zero ml-bridge dependency at the
/// call site.
///
/// # Errors
///
/// Propagates any error returned by `run_replicates_fn`.
fn run_rematch_with_runner<F>(
    mut run_replicates_fn: F,
    bootstrap_rng: &mut impl Rng,
) -> Result<TwoMetricOutcome, EnvError>
where
    F: FnMut(&[u64]) -> Result<CompetitionResult, EnvError>,
{
    // Ch 51 §2.1 / §2.2: the amendment runs Ch 32's
    // bootstrap-and-classify pipeline twice in parallel, once on
    // best_reward and once on final_reward.  The order in which
    // the two classify passes consume the bootstrap RNG is
    // deliberately best-first so that the best_reward call burns
    // the same random bits in the same order the pre-amendment
    // single-metric driver did, keeping the amended re-run's
    // best_reward numbers bit-for-bit comparable to Ch 50.
    let initial_seeds: Vec<u64> = (0..N_INITIAL)
        .map(|i| splitmix64(REMATCH_MASTER_SEED.wrapping_add(i as u64)))
        .collect();
    let initial_result = run_replicates_fn(&initial_seeds)?;
    let (mut r_cem_best, mut r_sa_best, mut r_cem_final, mut r_sa_final) =
        extract_replicate_vectors(&initial_result);

    let (initial_best_ci, initial_best_outcome) =
        test_and_classify(&r_cem_best, &r_sa_best, bootstrap_rng);
    let (initial_final_ci, initial_final_outcome) =
        test_and_classify(&r_cem_final, &r_sa_final, bootstrap_rng);

    // Ch 51 §2.2: expansion fires if *either* metric classifies
    // as Ambiguous.  Both metrics expand together so the
    // replicate pool stays matched.
    let either_ambiguous = initial_best_outcome == RematchOutcome::Ambiguous
        || initial_final_outcome == RematchOutcome::Ambiguous;
    let (best_ci, best_outcome, final_ci, final_outcome) = if either_ambiguous {
        let expansion_seeds: Vec<u64> = (N_INITIAL..N_EXPANDED)
            .map(|i| splitmix64(REMATCH_MASTER_SEED.wrapping_add(i as u64)))
            .collect();
        let expansion_result = run_replicates_fn(&expansion_seeds)?;
        let (exp_cem_best, exp_sa_best, exp_cem_final, exp_sa_final) =
            extract_replicate_vectors(&expansion_result);

        r_cem_best.extend_from_slice(&exp_cem_best);
        r_sa_best.extend_from_slice(&exp_sa_best);
        r_cem_final.extend_from_slice(&exp_cem_final);
        r_sa_final.extend_from_slice(&exp_sa_final);

        let (best_ci, best_outcome) = test_and_classify(&r_cem_best, &r_sa_best, bootstrap_rng);
        let (final_ci, final_outcome) = test_and_classify(&r_cem_final, &r_sa_final, bootstrap_rng);
        (best_ci, best_outcome, final_ci, final_outcome)
    } else {
        (
            initial_best_ci,
            initial_best_outcome,
            initial_final_ci,
            initial_final_outcome,
        )
    };

    let outcome = TwoMetricOutcome {
        best_ci,
        best_outcome,
        final_ci,
        final_outcome,
    };
    emit_rematch_summary(&outcome, &r_cem_best, &r_sa_best, &r_cem_final, &r_sa_final);
    Ok(outcome)
}

/// Pull the four paired replicate vectors out of a single
/// `run_replicates_fn` call's result and verify the pool is
/// aligned index-by-index.
///
/// Returns `(r_cem_best, r_sa_best, r_cem_final, r_sa_final)`.
///
/// The defensive length checks are in service of a silent
/// invariant: `CompetitionResult::replicate_best_rewards` and
/// `replicate_final_rewards` both drop replicates whose
/// `metrics` Vec is empty (Ch 24 §4.7's silent-None-filter
/// contract), and they drop *the same* replicates because
/// `best_reward()` and `final_reward()` are both `None` iff
/// `metrics.last()` is `None`.  CEM and SA also ought to retain
/// the same set of replicates because `run_replicates_fn`
/// threads a shared seed slice through both algorithms.  If any
/// of those invariants slip in a future refactor the bootstrap
/// downstream would silently operate on a misaligned pool; the
/// asserts turn that failure mode into a loud panic instead.
fn extract_replicate_vectors(
    result: &CompetitionResult,
) -> (Vec<f64>, Vec<f64>, Vec<f64>, Vec<f64>) {
    let r_cem_best = result.replicate_best_rewards(REMATCH_TASK_NAME, "CEM");
    let r_sa_best = result.replicate_best_rewards(REMATCH_TASK_NAME, "SA");
    let r_cem_final = result.replicate_final_rewards(REMATCH_TASK_NAME, "CEM");
    let r_sa_final = result.replicate_final_rewards(REMATCH_TASK_NAME, "SA");
    assert_eq!(
        r_cem_best.len(),
        r_cem_final.len(),
        "CEM replicate pool misaligned: best={}, final={}",
        r_cem_best.len(),
        r_cem_final.len(),
    );
    assert_eq!(
        r_sa_best.len(),
        r_sa_final.len(),
        "SA replicate pool misaligned: best={}, final={}",
        r_sa_best.len(),
        r_sa_final.len(),
    );
    assert_eq!(
        r_cem_best.len(),
        r_sa_best.len(),
        "CEM/SA replicate pools unequal: cem={}, sa={}",
        r_cem_best.len(),
        r_sa_best.len(),
    );
    (r_cem_best, r_sa_best, r_cem_final, r_sa_final)
}

// ── Private helpers ──────────────────────────────────────────────────────

#[allow(clippy::cast_precision_loss)]
fn mean(values: &[f64]) -> f64 {
    values.iter().sum::<f64>() / values.len() as f64
}

fn median(values: &[f64]) -> f64 {
    let mut sorted: Vec<f64> = values.to_vec();
    sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let n = sorted.len();
    if n % 2 == 0 {
        0.5 * (sorted[n / 2 - 1] + sorted[n / 2])
    } else {
        sorted[n / 2]
    }
}

// ── Tests ────────────────────────────────────────────────────────────────

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::float_cmp)]
mod tests {
    use super::*;
    use rand::SeedableRng;
    use rand::rngs::StdRng;
    use sim_ml_chassis::{
        Activation, CURRENT_VERSION, EpochMetrics, NetworkKind, PolicyArtifact, PolicyDescriptor,
        RunResult,
    };
    use std::collections::BTreeMap;

    // ── Bootstrap and classification unit tests ──────────────────────────

    #[test]
    fn bootstrap_diff_means_positive_ci() {
        let r_a = [1.0, 1.0, 1.0, 1.0, 1.0];
        let r_b = [0.0, 0.0, 0.0, 0.0, 0.0];
        let mut rng = StdRng::seed_from_u64(42);
        let ci = bootstrap_diff_means(&r_a, &r_b, &mut rng);
        assert_eq!(ci.point_estimate, 1.0);
        assert_eq!(ci.lower, 1.0);
        assert_eq!(ci.upper, 1.0);
        assert_eq!(ci.classify(), RematchOutcome::Positive);
        assert_eq!(ci.n_resamples, B);
    }

    #[test]
    fn bootstrap_diff_means_null_ci() {
        // Identical samples — bootstrap produces a distribution
        // centered at zero, so the CI straddles zero with
        // point estimate = 0, classifying as Null.
        let r_a = [1.0, 2.0, 3.0, 4.0, 5.0];
        let r_b = [1.0, 2.0, 3.0, 4.0, 5.0];
        let mut rng = StdRng::seed_from_u64(42);
        let ci = bootstrap_diff_means(&r_a, &r_b, &mut rng);
        assert_eq!(ci.point_estimate, 0.0);
        assert!(ci.lower < 0.0);
        assert!(ci.upper > 0.0);
        assert_eq!(ci.classify(), RematchOutcome::Null);
    }

    #[test]
    fn bootstrap_diff_means_ambiguous_ci() {
        // r_a has higher mean than r_b but with within-sample
        // variance large enough that the CI straddles zero.
        // The point estimate is positive, which classifies as
        // Ambiguous per Ch 32 §3.3.
        let r_a = [1.0, -0.5, 2.0, 0.0, 1.0];
        let r_b = [0.0, 0.5, -0.5, 1.0, -1.0];
        let mut rng = StdRng::seed_from_u64(42);
        let ci = bootstrap_diff_means(&r_a, &r_b, &mut rng);
        assert!(ci.point_estimate > 0.0);
        assert!(ci.lower < 0.0);
        assert!(ci.upper > 0.0);
        assert_eq!(ci.classify(), RematchOutcome::Ambiguous);
    }

    #[test]
    fn bootstrap_diff_medians_robust_to_outlier() {
        // r_a has one outlier that would inflate a mean-based
        // difference, but the median is immune. median(r_a) =
        // 1.0, median(r_b) = 0.0, difference = 1.0.
        let r_a = [1.0, 1.0, 1.0, 1.0, 100.0];
        let r_b = [0.0, 0.0, 0.0, 0.0, 0.0];
        let mut rng = StdRng::seed_from_u64(42);
        let ci = bootstrap_diff_medians(&r_a, &r_b, &mut rng);
        assert_eq!(ci.point_estimate, 1.0);
        assert!(ci.lower >= 0.0);
        assert!(ci.upper >= 1.0);
    }

    #[test]
    fn bimodality_coefficient_unimodal() {
        // Symmetric linear ramp — unimodal with low kurtosis.
        let values = [1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0, 5.5];
        let bc = bimodality_coefficient(&values);
        assert!(
            bc < 5.0 / 9.0,
            "unimodal input should have BC < 5/9, got {bc}"
        );
    }

    #[test]
    fn bimodality_coefficient_bimodal() {
        // Asymmetric bimodal: 8 values at 1.0, 2 values at 5.0.
        // Symmetric bimodal (5+5 at 1/5) has skewness = 0 and
        // Pearson's coefficient does not cross the 5/9
        // threshold at n = 10; asymmetric does. Ch 32 §6.2's
        // rematch inputs are plausibly asymmetric under
        // seed-varied training runs, so the test input matches
        // the real-world shape the threshold is meant to catch.
        let values = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 5.0, 5.0];
        let bc = bimodality_coefficient(&values);
        assert!(
            bc > 5.0 / 9.0,
            "asymmetric bimodal input should have BC > 5/9, got {bc}"
        );
    }

    #[test]
    #[should_panic(expected = "bimodality_coefficient requires n >= 4")]
    fn bimodality_coefficient_requires_n_geq_4() {
        let values = [1.0, 2.0, 3.0];
        let bc = bimodality_coefficient(&values);
        // Unreachable — the call above panics — but we still
        // consume the result so the `#[must_use]` on
        // bimodality_coefficient doesn't trip the workspace's
        // let_underscore_must_use deny.
        assert!(bc.is_finite());
    }

    #[test]
    fn classify_outcome_boundary_cases() {
        // Strict-positive CI → Positive.
        assert_eq!(classify_outcome(0.1, 1.0, 0.5), RematchOutcome::Positive);
        // Strict-negative CI → Null.
        assert_eq!(classify_outcome(-1.0, -0.1, -0.5), RematchOutcome::Null);
        // Straddle with positive point → Ambiguous.
        assert_eq!(classify_outcome(-0.5, 1.0, 0.25), RematchOutcome::Ambiguous);
        // Straddle with non-positive point → Null.
        assert_eq!(classify_outcome(-1.0, 0.5, 0.0), RematchOutcome::Null);
        assert_eq!(classify_outcome(-1.0, 0.5, -0.25), RematchOutcome::Null);
        // Boundary: lower == 0 (not strictly positive) with
        // positive upper and positive point → Ambiguous.
        assert_eq!(classify_outcome(0.0, 1.0, 0.5), RematchOutcome::Ambiguous);
        // Boundary: upper == 0 → Null (strict-positive fails).
        assert_eq!(classify_outcome(-1.0, 0.0, 0.0), RematchOutcome::Null);
    }

    // ── Mock-runner helpers for run_rematch tests ────────────────────────

    fn mock_policy_artifact() -> PolicyArtifact {
        PolicyArtifact {
            version: CURRENT_VERSION,
            descriptor: PolicyDescriptor {
                kind: NetworkKind::Linear,
                obs_dim: 1,
                act_dim: 1,
                hidden_dims: vec![],
                activation: Activation::Tanh,
                obs_scale: vec![1.0],
                stochastic: false,
            },
            params: vec![0.0, 0.0],
            provenance: None,
        }
    }

    fn mock_run_result(algo: &str, replicate: usize, reward: f64) -> RunResult {
        mock_run_result_with_epochs(algo, replicate, &[reward])
    }

    /// Build a multi-epoch mock `RunResult`.  `best_reward()`
    /// returns `max(rewards)`; `final_reward()` returns
    /// `*rewards.last()`.  The single-epoch `mock_run_result`
    /// helper is a thin wrapper whose best and final are equal,
    /// which is why the pre-Ch 51 tests produce the same
    /// verdict on both metrics.
    fn mock_run_result_with_epochs(algo: &str, replicate: usize, rewards: &[f64]) -> RunResult {
        let metrics = rewards
            .iter()
            .enumerate()
            .map(|(epoch, &r)| EpochMetrics {
                epoch,
                mean_reward: r,
                done_count: 0,
                total_steps: 1,
                wall_time_ms: 0,
                extra: BTreeMap::new(),
            })
            .collect();
        RunResult {
            task_name: REMATCH_TASK_NAME.to_string(),
            algorithm_name: algo.to_string(),
            replicate_index: replicate,
            metrics,
            artifact: mock_policy_artifact(),
            best_artifact: mock_policy_artifact(),
        }
    }

    /// Build a `CompetitionResult` with one single-epoch run per
    /// (algorithm, replicate) pair.  Best and final rewards are
    /// equal for every replicate under this shape.
    /// `cem_rewards.len()` must equal `sa_rewards.len()`.
    fn mock_competition_result(cem_rewards: &[f64], sa_rewards: &[f64]) -> CompetitionResult {
        assert_eq!(cem_rewards.len(), sa_rewards.len());
        let mut runs = Vec::with_capacity(cem_rewards.len() * 2);
        for (i, (&r_cem, &r_sa)) in cem_rewards.iter().zip(sa_rewards.iter()).enumerate() {
            runs.push(mock_run_result("CEM", i, r_cem));
            runs.push(mock_run_result("SA", i, r_sa));
        }
        CompetitionResult { runs }
    }

    /// Build a `CompetitionResult` from per-replicate per-epoch
    /// reward trajectories.  `cem_epochs[i]` is the `mean_reward`
    /// history for CEM replicate `i`; the same for SA.  This
    /// lets tests exercise the `best_reward` / `final_reward`
    /// gap that Ch 50 documented for population-vs-incumbent
    /// algorithms, which single-epoch mocks cannot express.
    fn mock_competition_result_multi_epoch(
        cem_epochs: &[&[f64]],
        sa_epochs: &[&[f64]],
    ) -> CompetitionResult {
        assert_eq!(cem_epochs.len(), sa_epochs.len());
        let mut runs = Vec::with_capacity(cem_epochs.len() * 2);
        for (i, (cem_traj, sa_traj)) in cem_epochs.iter().zip(sa_epochs.iter()).enumerate() {
            runs.push(mock_run_result_with_epochs("CEM", i, cem_traj));
            runs.push(mock_run_result_with_epochs("SA", i, sa_traj));
        }
        CompetitionResult { runs }
    }

    #[test]
    fn run_rematch_positive_short_circuits() {
        // Initial batch classifies as Positive: all 10 SA
        // rewards are ~1.0 above all 10 CEM rewards, so the
        // bootstrap CI on the diff is tight and strictly
        // positive. run_rematch should NOT call the runner
        // a second time.  Under the Ch 51 amendment both
        // metrics classify as Positive because the single-epoch
        // mocks have best == final for every replicate.
        let initial = mock_competition_result(
            &[0.0, 0.05, -0.05, 0.02, -0.02, 0.01, -0.01, 0.0, 0.03, -0.03],
            &[1.0, 1.05, 0.95, 1.02, 0.98, 1.01, 0.99, 1.0, 1.03, 0.97],
        );
        let call_count = std::cell::Cell::new(0);
        let mut rng = StdRng::seed_from_u64(42);
        let outcome = run_rematch_with_runner(
            |_seeds| {
                call_count.set(call_count.get() + 1);
                Ok(initial.clone())
            },
            &mut rng,
        )
        .unwrap();
        assert_eq!(outcome.best_outcome, RematchOutcome::Positive);
        assert_eq!(outcome.final_outcome, RematchOutcome::Positive);
        assert_eq!(
            call_count.get(),
            1,
            "run_rematch must short-circuit on Positive without calling the runner a second time"
        );
    }

    #[test]
    fn run_rematch_ambiguous_triggers_expansion() {
        // Initial batch classifies as Ambiguous: SA's mean is
        // slightly above CEM's but the within-sample variance
        // straddles zero with a positive point estimate.
        // run_rematch must run the runner a second time with
        // the expansion seeds.  Under the Ch 51 amendment both
        // metrics are ambiguous in the initial batch (best ==
        // final for single-epoch mocks), so the expansion rule
        // fires on either metric alone just as it did pre-Ch 51.
        let initial = mock_competition_result(
            &[0.0, 1.0, -1.0, 0.5, -0.5, 0.2, -0.2, 0.1, -0.1, 0.0],
            &[0.3, 1.3, -0.7, 0.8, -0.2, 0.5, 0.1, 0.4, 0.2, 0.3],
        );
        // Expansion batch: SA now clearly exceeds CEM with
        // tight within-sample variance. Post-extension, both
        // metrics should classify as Positive (the expansion
        // batch's verdict is the headline).
        let expansion = mock_competition_result(
            &[0.0, 0.05, -0.05, 0.02, -0.02, 0.01, -0.01, 0.0, 0.03, -0.03],
            &[1.0, 1.05, 0.95, 1.02, 0.98, 1.01, 0.99, 1.0, 1.03, 0.97],
        );
        let call_count = std::cell::Cell::new(0);
        let mut rng = StdRng::seed_from_u64(42);
        let runs = std::cell::RefCell::new(vec![expansion, initial]);
        let outcome = run_rematch_with_runner(
            |_seeds| {
                call_count.set(call_count.get() + 1);
                Ok(runs.borrow_mut().pop().unwrap())
            },
            &mut rng,
        )
        .unwrap();
        assert_eq!(
            call_count.get(),
            2,
            "run_rematch must call the runner twice when the initial batch is Ambiguous"
        );
        assert_eq!(outcome.best_outcome, RematchOutcome::Positive);
        assert_eq!(outcome.final_outcome, RematchOutcome::Positive);
    }

    #[test]
    fn run_rematch_metric_sensitivity_classifies_differently() {
        // Ch 51 §2 is motivated by a concrete pattern: a
        // population method that transiently touches high
        // rewards and then regresses off-peak (CEM in Ch 50's
        // per-replicate table) vs an incumbent tracker where
        // best ≈ final by construction (SA under the Metropolis
        // accept rule).  Under best_reward the population
        // method's peaks push mean(CEM_best) above mean(SA_best);
        // under final_reward the incumbent tracker's held
        // rewards push mean(SA_final) above mean(CEM_final).
        // The same replicate set therefore classifies as `Null`
        // under best_reward (SA-CEM diff negative) and
        // `Positive` under final_reward (SA-CEM diff positive).
        //
        // This is the `(Null, Positive)` cell of Ch 51 §2.3's
        // nine-cell table — exactly the cell the 2026-04-14
        // data is predicted to land in and the cell the
        // amendment's interpretation framework reads as
        // `Positive` for Ch 30's scientific question.  Having
        // a unit test that lands in this cell gives the
        // framework a machine-checkable anchor independent of
        // the fixture.
        //
        // CEM trajectory per replicate: [300.0, 100.0] —
        // best = 300 (peak on epoch 0), final = 100 (regresses
        // on epoch 1).
        // SA trajectory per replicate:  [200.0, 200.0] —
        // best = 200, final = 200 (monotone incumbent).
        //
        // Slight per-replicate jitter on each scalar so the
        // bootstrap doesn't collapse to a point mass.
        let cem_trajs: Vec<[f64; 2]> = (0..10i32)
            .map(|i| {
                let jitter = f64::from(i) * 0.1;
                [300.0 + jitter, 100.0 + jitter]
            })
            .collect();
        let sa_trajs: Vec<[f64; 2]> = (0..10i32)
            .map(|i| {
                let jitter = f64::from(i) * 0.1;
                [200.0 + jitter, 200.0 + jitter]
            })
            .collect();
        let cem_refs: Vec<&[f64]> = cem_trajs.iter().map(<[f64; 2]>::as_slice).collect();
        let sa_refs: Vec<&[f64]> = sa_trajs.iter().map(<[f64; 2]>::as_slice).collect();
        let initial = mock_competition_result_multi_epoch(&cem_refs, &sa_refs);

        let call_count = std::cell::Cell::new(0);
        let mut rng = StdRng::seed_from_u64(42);
        let outcome = run_rematch_with_runner(
            |_seeds| {
                call_count.set(call_count.get() + 1);
                Ok(initial.clone())
            },
            &mut rng,
        )
        .unwrap();

        // Headline assertion: the two metrics must disagree.
        assert_ne!(
            outcome.best_outcome, outcome.final_outcome,
            "metric-sensitivity fixture should produce different classifications under best_reward vs final_reward"
        );
        // And specifically, the `(Null, Positive)` cell.
        assert_eq!(
            outcome.best_outcome,
            RematchOutcome::Null,
            "best_reward: CEM's peak touches push mean(CEM_best) above mean(SA_best)"
        );
        assert_eq!(
            outcome.final_outcome,
            RematchOutcome::Positive,
            "final_reward: SA's held rewards push mean(SA_final) above mean(CEM_final)"
        );
        // Point-estimate signs must bracket zero in opposite
        // directions.  best_ci is SA-CEM ≈ 200 - 300 = -100;
        // final_ci is SA-CEM ≈ 200 - 100 = +100.
        assert!(
            outcome.best_ci.point_estimate < 0.0,
            "best_ci point estimate should be negative, got {}",
            outcome.best_ci.point_estimate
        );
        assert!(
            outcome.final_ci.point_estimate > 0.0,
            "final_ci point estimate should be positive, got {}",
            outcome.final_ci.point_estimate
        );
        // Neither metric is Ambiguous, so the expansion rule
        // does not fire and the runner is called exactly once.
        assert_eq!(
            call_count.get(),
            1,
            "metric-sensitivity fixture should not trigger the N=20 expansion"
        );
    }
}
