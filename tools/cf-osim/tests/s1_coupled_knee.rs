//! S1 — the coupled-knee model vs the OpenSim oracle (the G1 gate result).
//!
//! Run: cargo test -p cf-osim --test s1_coupled_knee -- --ignored --nocapture
//!
//! S0 proved a bare hinge fails (8.7–14.6 mm), dominated by R3, the coupled
//! tibial translation. S1 emits a model that represents the gait2392 knee:
//! coupled tibial translation → two coupled slide DOFs (knee_tx, knee_ty); each
//! patella MovingPathPoint → a coupled "patella" body (3 slides); conditional
//! points dropped (a static tendon can't toggle membership).
//!
//! What this proves (and its limit): the model is DRIVEN with the same gait2392
//! splines the oracle uses, so this is a *driven-consistency / representation*
//! test — it shows the emitted coupled MJCF faithfully reproduces the oracle's
//! kinematics (a pure-translation muscle matches to ~0, which is near-tautological
//! by construction). The residuals (≤3.7 mm) come ONLY from the dropped
//! conditional points. It does NOT independently validate against real OpenSim —
//! that cross-check (with the cubic splines now in place) is still pending.

use cf_osim::emit::emit_coupled_knee;
use cf_osim::oracle::{Kinematics, Variant};
use cf_osim::osim::{Kind, parse_knee_subgraph};
use cf_osim::{coupled_moment_arm, joint_id, tendon_id};
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
#[ignore = "S1 spike — run with --ignored --nocapture"]
fn s1_coupled_knee_scorecard() {
    let xml = std::fs::read_to_string(osim_path()).expect("read gait2392.osim");
    let sub = parse_knee_subgraph(&xml);

    let angles: Vec<f64> = (0..=20).map(|i| -(i as f64) * 5.0 * DEG).collect();
    let eps = 0.5 * DEG;

    let kin = Kinematics::new(&sub);
    let emitted = emit_coupled_knee(&sub);
    let model = load_model(&emitted.mjcf).expect("coupled knee MJCF must load");

    // Drive ALL coupled DOFs with the EXACT gait2392 splines: knee = θ,
    // knee_tx = tx(θ), knee_ty = ty(θ), and each patella body's slides = the
    // moving point's location splines.
    let adr_knee = model.jnt_qpos_adr[joint_id(&model, "knee")];
    let adr_tx = model.jnt_qpos_adr[joint_id(&model, "knee_tx")];
    let adr_ty = model.jnt_qpos_adr[joint_id(&model, "knee_ty")];
    let pat: Vec<_> = emitted
        .patellae
        .iter()
        .map(|p| {
            (
                model.jnt_qpos_adr[joint_id(&model, &p.jx)],
                model.jnt_qpos_adr[joint_id(&model, &p.jy)],
                model.jnt_qpos_adr[joint_id(&model, &p.jz)],
                &p.sx,
                &p.sy,
                &p.sz,
            )
        })
        .collect();
    let drive = |th: f64, q: &mut [f64]| {
        q[adr_knee] = th;
        q[adr_tx] = sub.knee.tx.eval(th);
        q[adr_ty] = sub.knee.ty.eval(th);
        for (ax, ay, az, sx, sy, sz) in &pat {
            q[*ax] = sx.eval(th);
            q[*ay] = sy.eval(th);
            q[*az] = sz.eval(th);
        }
    };

    println!("\n============== S1 COUPLED-KNEE SCORECARD ==============");
    println!("coupled translation + patella bodies + dropped conditionals\n");
    println!(
        "{:<11} {:>3} {:>9} {:>9} {:>9}",
        "muscle", "mov", "vs truth", "S0 hinge", "verdict"
    );
    println!("{:-<54}", "");

    for m in &sub.muscles {
        let n_moving = m
            .path
            .iter()
            .filter(|p| matches!(p.kind, Kind::Moving(_)))
            .count();
        let t = tendon_id(&model, &m.name);

        let truth: Vec<f64> = angles
            .iter()
            .map(|&th| kin.moment_arm(m, th, eps, Variant::TRUTH) * 1000.0)
            .collect();
        let coupled: Vec<f64> = angles
            .iter()
            .map(|&th| coupled_moment_arm(&model, t, th, eps, &drive) * 1000.0)
            .collect();
        let s0_hinge: Vec<f64> = angles
            .iter()
            .map(|&th| kin.moment_arm(m, th, eps, Variant::ENGINE) * 1000.0)
            .collect();

        let rmse = rms(&coupled, &truth);
        let s0_rmse = rms(&s0_hinge, &truth);
        let verdict = if rmse < 5.0 {
            "PASS <5mm"
        } else if n_moving > 0 {
            "needs patella"
        } else {
            "CHECK"
        };
        println!(
            "{:<11} {:>3} {:>7.2}mm {:>7.2}mm {:>9}",
            m.name, n_moving, rmse, s0_rmse, verdict
        );

        // Driven-consistency gate: the coupled model reproduces the re-derived
        // oracle within 5 mm for EVERY muscle (the bare hinge missed it for all
        // four). NOT a real-OpenSim check — see the module docstring.
        assert!(
            rmse < 5.0,
            "{}: coupled model RMSE {rmse:.2}mm exceeds the 5mm budget",
            m.name
        );

        // A pure-translation muscle (no moving point, no toggling conditional)
        // matches to sub-mm — the coupling is captured exactly.
        let toggling_cond = m.path.iter().any(
            |p| matches!(p.kind, Kind::Conditional { lo, hi } if !(lo <= -1.745 && hi >= 0.0)),
        );
        if n_moving == 0 && !toggling_cond {
            assert!(
                rmse < 1.0,
                "{}: pure-translation should match <1mm, got {rmse:.2}mm",
                m.name
            );
        }
    }

    println!("{:-<54}", "");
    println!("ALL FOUR < 5mm vs the re-derived oracle (driven-consistency, not real");
    println!("OpenSim) — the coupled representation recovers what the bare S0 hinge");
    println!("missed for every muscle (8.7-14.6mm). Real-OpenSim cross-check pending.");
    println!("======================================================\n");
}
