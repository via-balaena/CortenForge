//! S0 bridge spike — gait2392 knee → MJCF, graded against the OpenSim oracle.
//!
//! Throwaway-grade (`#[ignore]`): the deliverable is the printed scorecard. Run:
//!   cargo test -p cf-osim --test s0_knee_bridge -- --ignored --nocapture
//!
//! Pipeline: parse the vendored `.osim` knee subgraph → build the straight-
//! segment oracle (true coupled-knee kinematics) → emit the frozen-hinge MJCF →
//! load it → sweep 0–100° flexion → compare moment arms (`-ten_J` vs `-dL/dθ`).
//!
//! "Oracle" here means OUR re-derivation of OpenSim's GeometryPath geometry from
//! the .osim coordinates (no wrap objects ⇒ a straight-segment sum) — NOT a real
//! OpenSim run. That independent cross-check is still pending (S1-remaining).
//!
//! Because there are no wrap objects, the our-engine-vs-oracle gap isolates the
//! G1 approximations, attributed by computing the moment arm under each freeze:
//! R3 = frozen coupled tibial translation (FREEZE_COUPLING_ONLY vs TRUTH);
//! O2 = frozen patella MovingPathPoint (FREEZE_MOVING_ONLY vs TRUTH); Cond =
//! frozen conditional membership. The engine (`ten_J`) is cross-checked against
//! the analytic all-frozen variant; since BOTH compute the same straight-segment
//! sum, this confirms the parse→emit→import→extract PLUMBING is self-consistent
//! with the analytic model — it does NOT prove agreement with real OpenSim.
//! Caveat: the R3/O2/Cond columns are oracle-internal sensitivities (oracle-vs-
//! oracle), non-additive and partly cancelling, so they rank the approximations
//! rather than decompose the engine error; R3-dominance is cleanest on the
//! plain-point hamstrings (R3 ≈ total there).

use cf_osim::emit::emit_frozen_hinge;
use cf_osim::oracle::{Kinematics, Variant};
use cf_osim::osim::{Kind, parse_knee_subgraph};
use cf_osim::{joint_id, moment_arm_sweep, tendon_id};
use sim_mjcf::load_model;
use std::f64::consts::PI;

const DEG: f64 = PI / 180.0;

fn osim_path() -> String {
    format!(
        "{}/../../sim/L0/tests/assets/opensim_gait2392/gait2392.osim",
        env!("CARGO_MANIFEST_DIR")
    )
}

fn rms(a: &[f64], b: &[f64]) -> f64 {
    let n = a.len() as f64;
    (a.iter().zip(b).map(|(x, y)| (x - y).powi(2)).sum::<f64>() / n).sqrt()
}

#[test]
#[ignore = "S0 spike — run with --ignored --nocapture to see the scorecard"]
fn s0_knee_moment_arm_scorecard() {
    let xml = std::fs::read_to_string(osim_path()).expect("read vendored gait2392.osim");
    let sub = parse_knee_subgraph(&xml);

    // Flexion sweep: knee_angle_r from 0 (extension) to -100° (flexion), 5° steps.
    let angles: Vec<f64> = (0..=20).map(|i| -(i as f64) * 5.0 * DEG).collect();
    let eps = 0.5 * DEG;

    let kin = Kinematics::new(&sub);
    let emitted = emit_frozen_hinge(&sub);
    let model = load_model(&emitted.mjcf).expect("emitted knee MJCF must load");
    let knee = joint_id(&model, "knee");

    // mm moment-arm curve under a given oracle variant.
    let oracle_curve = |m: &_, v: Variant| -> Vec<f64> {
        angles
            .iter()
            .map(|&t| kin.moment_arm(m, t, eps, v) * 1000.0)
            .collect()
    };

    println!("\n================== S0 KNEE BRIDGE SCORECARD ==================");
    println!(
        "gait2392 (Apache-2.0) | knee axis {:?} | sweep 0..100° flexion, 5° steps",
        sub.knee.flexion_axis
    );
    println!("moment arms in mm; RMSE vs our OpenSim-geometry oracle (re-derivation)\n");
    println!(
        "{:<11} {:>3} {:>8} {:>8} | {:>6} {:>6} {:>6} {:>6} | {:>8}",
        "muscle", "mov", "truth@0", "tru@90", "totalΔ", "R3", "O2", "Cond", "eng≈FF"
    );
    println!("{:-<78}", "");

    for m in &sub.muscles {
        let n_moving = m
            .path
            .iter()
            .filter(|p| matches!(p.kind, Kind::Moving(_)))
            .count();

        let truth = oracle_curve(m, Variant::TRUTH);
        let ff = oracle_curve(m, Variant::ENGINE); // analytic all-frozen
        let r3 = oracle_curve(m, Variant::FREEZE_COUPLING_ONLY);
        let o2 = oracle_curve(m, Variant::FREEZE_MOVING_ONLY);
        let cond = oracle_curve(m, Variant::FREEZE_CONDITIONAL_ONLY);

        let t = tendon_id(&model, &m.name);
        let engine: Vec<f64> = moment_arm_sweep(&model, t, knee, &angles)
            .iter()
            .map(|s| s.moment_arm * 1000.0)
            .collect();

        // Geometry transfer at θ=0: frozen sites == true positions ⇒ lengths equal.
        let len0_engine = moment_arm_sweep(&model, t, knee, &[0.0])[0].length;
        assert!(
            (len0_engine - kin.path_length(m, 0.0, Variant::TRUTH)).abs() < 1e-4,
            "{}: θ=0 length mismatch — geometry transfer broken",
            m.name
        );

        let eng_vs_ff = engine
            .iter()
            .zip(&ff)
            .map(|(a, b)| (a - b).abs())
            .fold(0.0_f64, f64::max);

        println!(
            "{:<11} {:>3} {:>8.1} {:>8.1} | {:>4.1}mm {:>4.1}mm {:>4.1}mm {:>4.1}mm | {:>6.3}mm",
            m.name,
            n_moving,
            truth[0],
            truth[18], // 90°
            rms(&engine, &truth),
            rms(&r3, &truth),
            rms(&o2, &truth),
            rms(&cond, &truth),
            eng_vs_ff,
        );

        // Plumbing self-consistency: the engine's -ten_J must match the analytic
        // both-frozen moment arm. Both compute the same straight-segment sum, so
        // agreement confirms the emit→import→extract plumbing is self-consistent
        // with the analytic model — NOT agreement with real OpenSim. (The 5mm
        // budget is NOT asserted — its failure IS the S0 finding: a bare hinge is
        // insufficient, dominated by R3.)
        assert!(
            eng_vs_ff < 0.5,
            "{}: engine moment arm diverges from analytic frozen model by {eng_vs_ff:.3}mm \
             — emit/extraction bug, not a modelling gap",
            m.name
        );
    }

    println!("{:-<74}", "");
    println!("FINDING: a bare frozen hinge misses the 5mm budget for ALL four muscles.");
    println!("  R3 (coupled tibial translation) dominates O2 (patella moving point);");
    println!("  even plain-point hamstrings fail ⇒ cf-mjcf-emit must model the knee");
    println!("  coupling (MuJoCo equality-joint coupling), not just a 1-DOF hinge.");
    println!("=============================================================\n");
}
