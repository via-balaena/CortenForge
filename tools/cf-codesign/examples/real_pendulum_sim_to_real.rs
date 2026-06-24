//! Worked example — a **sim-to-real** loop on a REAL pendulum.
//!
//! The system-ID estimator ([`FrictionSystemId`]) was hardened on sim-to-sim data
//! (full-trajectory matching + observation noise). This closes the existential loop:
//! it recovers a real pendulum's friction from MEASURED data and predicts a held-out
//! window of the real motion it never fit.
//!
//! Data: the Schmidt & Lipson (2009) real single-pendulum decay (`real_pend_h_1.txt`,
//! columns `[trial, t, θ, ω]`; 556 samples, ~6 Hz, natural frequency ≈ 1.1 rad/s
//! (period ≈ 5.6 s), ~17 periods of light decay). It is **referenced, not
//! redistributed**: we commit no data bytes, only fetch them at runtime. Original
//! source — supplement to M. Schmidt & H. Lipson, *Distilling Free-Form Natural Laws
//! from Experimental Data*, Science 324:81 (2009); its terms are the publisher's.
//! Access path — the Apache-2.0 `erwincoumans/tiny-differentiable-simulator` mirror,
//! pinned to a commit (see [`DATA_URL`]).
//!
//! Pipeline (a real calibration):
//!   1. CALIBRATE the conservative dynamics — choose the model's gravity so its free
//!      oscillation matches the data's frequency on a short early window (where
//!      dissipation is negligible and phase-wrap can't trap the search).
//!   2. FIT dry friction on a TRAIN window via the shipped
//!      [`FrictionSystemId::from_measured_trajectory`] — the sim-to-real entry point,
//!      driven here on real measured data.
//!   3. VALIDATE on a HELD-OUT later window: angle prediction RMS, against a
//!      no-dissipation baseline.
//!
//! ```text
//! cargo run -p cf-codesign --example real_pendulum_sim_to_real --release
//! ```
//! Requires `curl` and network access to fetch the dataset once (then cached in the
//! system temp dir). Offline: download the URL printed below to that path manually.

#![allow(clippy::expect_used, clippy::print_stdout)]

use std::process::Command;

use cf_codesign::{FrictionSystemId, Normalized, OptConfig};
use sim_core::{DVector, Model, Vector3};

/// Pinned public mirror of the Schmidt-Lipson dataset (pinned to a commit so the
/// bytes can't change underneath the example). Referenced, never vendored.
const DATA_URL: &str = "https://raw.githubusercontent.com/erwincoumans/tiny-differentiable-simulator/8381b8c9ad7f0a959548e4982eb8a63431a842d2/data/schmidt-lipson-exp-data/real_pend_h_1.txt";

/// Sub-step the ~6 Hz data 8× so Euler integrates the pendulum accurately and each
/// data sample lands on a distinct model step.
const DT_MODEL: f64 = 0.166_67 / 8.0;

/// One real sample: (time s, angle rad, angular velocity rad/s).
type Sample = (f64, f64, f64);

/// Fetch the dataset to a temp-dir cache (reused on later runs). Returns `None` with
/// a guidance message if `curl`/network is unavailable — an example shouldn't panic
/// on a missing network.
fn fetch_data() -> Option<String> {
    let cache = std::env::temp_dir().join("cf_real_pend_h_1.txt");
    if !cache.exists() {
        println!("Fetching dataset → {}", cache.display());
        // Download to a temp name and rename on success, so an interrupted transfer
        // never leaves a truncated file that later runs would trust as the cache.
        let part = cache.with_extension("part");
        let ok = Command::new("curl")
            .args(["-sSL", "-o", part.to_str().expect("utf-8 path"), DATA_URL])
            .status()
            .map(|s| s.success())
            .unwrap_or(false)
            && std::fs::rename(&part, &cache).is_ok();
        if !ok {
            let _ = std::fs::remove_file(&part);
            println!("\nCould not fetch the dataset (offline?). Download it manually:");
            println!("  curl -sSL -o {} \\\n    {DATA_URL}", cache.display());
            return None;
        }
    }
    std::fs::read_to_string(&cache).ok()
}

/// Parse `[trial, t, θ, ω]` rows, keeping `(t, θ, ω)`.
fn parse(raw: &str) -> Vec<Sample> {
    raw.lines()
        .filter_map(|l| {
            let c: Vec<f64> = l
                .split_whitespace()
                .filter_map(|s| s.parse().ok())
                .collect();
            // `then` (lazy) — `then_some` would index `c` on the non-numeric header.
            (c.len() >= 4).then(|| (c[1], c[2], c[3]))
        })
        .collect()
}

/// A 1-link hinge pendulum with gravity magnitude `g`, sub-stepped at [`DT_MODEL`].
fn model(g: f64) -> Model {
    let mut m = Model::n_link_pendulum(1, 1.0, 0.2);
    m.timestep = DT_MODEL;
    m.gravity = Vector3::new(0.0, 0.0, -g);
    m
}

/// Model step index a data time maps to, relative to the first sample. Row 0 (the
/// initial condition) maps to step 0; later rows are strictly increasing (the ~6 Hz
/// data is sub-stepped 8×, so consecutive samples are ≥ 8 steps apart).
fn step_of(t: f64, t0: f64) -> usize {
    ((t - t0) / DT_MODEL).round() as usize
}

/// Roll a model out from the data's initial `(θ, ω)` and sample model `(θ, ω)` at
/// each data row's mapped step — one rollout, a single cursor walking the sorted
/// step schedule.
fn rollout(m: &Model, data: &[Sample]) -> Vec<(f64, f64)> {
    let t0 = data[0].0;
    let steps: Vec<usize> = data.iter().map(|&(t, _, _)| step_of(t, t0)).collect();
    let mut d = m.make_data();
    d.qpos[0] = data[0].1;
    d.qvel[0] = data[0].2;
    let mut out = vec![(0.0, 0.0); data.len()];
    let mut cursor = 0;
    // Row 0 maps to step 0 — the initial state, before any integration.
    while cursor < steps.len() && steps[cursor] == 0 {
        out[cursor] = (d.qpos[0], d.qvel[0]);
        cursor += 1;
    }
    for step in 1..=*steps.last().expect("nonempty") {
        d.step(m).expect("rollout");
        while cursor < steps.len() && steps[cursor] == step {
            out[cursor] = (d.qpos[0], d.qvel[0]);
            cursor += 1;
        }
    }
    out
}

/// RMS angle error (rad) over a row range.
fn rms(pred: &[(f64, f64)], data: &[Sample], range: std::ops::Range<usize>) -> f64 {
    let n = range.len() as f64;
    let s: f64 = range.map(|i| (pred[i].0 - data[i].1).powi(2)).sum();
    (s / n).sqrt()
}

/// Calibrate gravity to the conservative frequency on the first `window` rows
/// (coarse grid then local refine — phase-RMS vs g has a clean basin over a few
/// periods, but is multimodal over the full record).
fn calibrate_gravity(data: &[Sample], window: usize) -> (f64, f64) {
    let win = &data[0..window]; // score (and roll out) only the calibration window
    let score = |g: f64| rms(&rollout(&model(g), win), win, 0..window);
    let (mut best_g, mut best) = (1.0, f64::INFINITY);
    for k in 0..=200 {
        let g = 0.1 + 0.05 * k as f64;
        let e = score(g);
        if e < best {
            (best, best_g) = (e, g);
        }
    }
    let mut step = 0.05;
    for _ in 0..40 {
        for &g in &[best_g - step, best_g + step] {
            if g > 0.0 && score(g) < best {
                (best, best_g) = (score(g), g);
            }
        }
        step *= 0.7;
    }
    (best_g, best)
}

fn main() {
    let Some(raw) = fetch_data() else { return };
    let data = parse(&raw);
    let n = data.len();
    let (t0, tn) = (data[0].0, data[n - 1].0);
    println!("\n=== Real pendulum sim-to-real (Schmidt & Lipson 2009) ===");
    println!(
        "{n} samples, t = [{t0:.1}, {tn:.1}] s; initial θ₀ = {:.4} rad, ω₀ = {:.4} rad/s",
        data[0].1, data[0].2
    );

    // 1. Calibrate the conservative frequency on ~3 periods (period ≈ 5.6 s).
    let calib_rows = data.iter().take_while(|&&(t, _, _)| t - t0 < 17.0).count();
    let (g, g_rms) = calibrate_gravity(&data, calib_rows);
    println!(
        "\n1. Calibrated gravity g = {g:.4} m/s² (3-period frequency-match RMS {g_rms:.3e} rad)"
    );

    // 2. Fit dry friction on the TRAIN window via the shipped sim-to-real estimator.
    let train_end = n * 60 / 100; // first 60% train, last 40% held out
    let theta0 = data[0].1;
    // Observations are rows 1..train_end (row 0 is the initial condition); each maps
    // to a distinct, strictly increasing model step.
    let mut waypoints = Vec::new();
    let mut observed = Vec::new();
    for &(t, th, w) in &data[1..train_end] {
        let s = step_of(t, t0);
        if waypoints.last().is_none_or(|&last| s > last) {
            waypoints.push(s);
            observed.push(th - theta0);
            observed.push(w);
        }
    }
    let n_steps = *waypoints.last().expect("train window nonempty");

    let mut m = model(g);
    let mut data0 = m.make_data();
    data0.qpos[0] = data[0].1;
    data0.qvel[0] = data[0].2;
    m.dof_frictionloss = vec![0.0]; // targeted DOF — overwritten each evaluate
    let problem = FrictionSystemId::from_measured_trajectory(
        m,
        data0,
        n_steps,
        waypoints,
        vec![0],
        DVector::from_vec(observed),
    );
    // Friction is a positive scale → log-space relative steps.
    let normalized = Normalized::with_residual_scale(&problem, 1.0, true);
    let cfg = OptConfig {
        max_iters: 600,
        ..normalized.recommended_config()
    };
    let mu = normalized.optimize(&[1.0e-3], &cfg).params[0];
    println!(
        "2. Fitted dry friction μ = {mu:.5} N·m (FrictionSystemId::from_measured_trajectory, train rows 1..{train_end})"
    );

    // 3. Validate: predict the WHOLE record with the calibrated + fitted model.
    let mut fitted = model(g);
    fitted.dof_frictionloss = vec![mu];
    let pred = rollout(&fitted, &data);
    let baseline = rollout(&model(g), &data); // no dissipation
    let (train, test) = (0..train_end, train_end..n);
    println!(
        "\n3. Held-out validation (rows {train_end}..{n}, t ≥ {:.1} s):",
        data[train_end].0
    );
    println!(
        "   no dissipation : train RMS {:.3e}   held-out RMS {:.3e} rad ({:.1}°)",
        rms(&baseline, &data, train.clone()),
        rms(&baseline, &data, test.clone()),
        rms(&baseline, &data, test.clone()).to_degrees(),
    );
    let train_rms = rms(&pred, &data, train);
    let held = rms(&pred, &data, test.clone());
    println!(
        "   fitted (μ)     : train RMS {train_rms:.3e}   held-out RMS {held:.3e} rad ({:.2}°)",
        held.to_degrees(),
    );
    println!(
        "\n→ The shipped estimator recovers a real pendulum's friction from measured data\n  \
         and predicts {:.1} s of UNSEEN motion to {:.2}° RMS (vs {:.0}° with no dissipation).\n  \
         Held-out RMS ≈ train RMS ({:.2}° vs {:.2}°): the residual is the conservative model's\n  \
         floor (single effective gravity + dry friction), NOT a generalization gap — a sharper\n  \
         conservative model is the next fidelity lever, orthogonal to this dissipation estimator.",
        tn - data[train_end].0,
        held.to_degrees(),
        rms(&baseline, &data, test).to_degrees(),
        held.to_degrees(),
        train_rms.to_degrees(),
    );
}
