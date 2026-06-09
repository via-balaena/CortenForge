//! G2-PR3b: the MUSCLE-DRIVEN leg twin loads, drives, and produces correct dynamics.
//! The end-to-end close of the dynamics-validation arc — the emitted twin's Millard
//! actuators are driven at known activations and the results checked two ways.
//!
//! **Gate (machine-exact):** the loaded twin's actuator force equals the OpenSim-
//! validated standalone `millard_path_force` evaluated at the twin's OWN actuator
//! (musculotendon) length. This proves the emit → MJCF-parse → tendon-transmission →
//! Millard-dispatch wiring is correct (the force the driven twin produces IS the
//! validated force). It is exact because both sides are the same force model at the
//! same length — no geometry confound. (It anchors the *wiring*: that the params land
//! in the right slots vs OpenSim is anchored separately by the knee_ref.xml snapshot
//! and the joint-moment report below.)
//!
//! **Reported (end-to-end vs OpenSim):** the per-muscle knee joint moment
//! (−actuator_force × coupled moment arm) vs real OpenSim. This is the literal "joint
//! torque vs OpenSim" number, but it folds in the emit's GEOMETRY fidelity (the force
//! model is exact). The dominant residual is the emit's **dropped-conditional via-point**
//! approximation (a conditional path point active over part of the ROM in OpenSim is
//! dropped by the emit, bending the moment arm there): it bites the quads at DEEP
//! flexion and the semimem hamstring at EXTENSION (its conditional is active ≈0…−32°),
//! where the arm error locally exceeds the A1 5 mm bound. So per muscle: the quads and
//! bifemlh are ~1–2% across the functional ROM, but semimem reaches ~25% even in the
//! functional band (its extension moment-arm error, our ~24 mm vs OpenSim ~30 mm).
//! That is the emit's documented geometry limit, not a dynamics bug — hence reported.
//!
//! References: `gen_muscle_forces.py` (params) / `gen_muscle_joint_moments.py`.

use cf_mjcf_emit::emit;
use cf_osim::{coupled_moment_arm, joint_id, parse_leg_chain, tendon_id};
use serde_json::Value;
use sim_core::forward::{MillardMuscleParams, default_millard_curves, millard_path_force};
use sim_mjcf::load_model;
use std::collections::HashMap;
use std::f64::consts::PI;

const DEG: f64 = PI / 180.0;
const MUSCLES: [&str; 4] = ["rect_fem_r", "vas_int_r", "bifemlh_r", "semimem_r"];

fn asset(name: &str) -> String {
    format!(
        "{}/../../sim/L0/tests/assets/opensim_gait2392/{name}",
        env!("CARGO_MANIFEST_DIR")
    )
}

#[test]
fn muscle_driven_twin_force_and_torque_vs_opensim() {
    let template = parse_leg_chain(&std::fs::read_to_string(asset("gait2392.osim")).unwrap());
    let emitted = emit(&template); // the muscle-driven twin (Millard actuators on tendons)
    let model = load_model(&emitted.mjcf).expect("emitted muscle-driven MJCF must load");
    let d: Value = serde_json::from_str(
        &std::fs::read_to_string(asset("muscle_joint_moments_opensim.json")).unwrap(),
    )
    .unwrap();
    let curves = default_millard_curves();

    // Drive every emitted joint (free hinge + coupled slides + patella) from knee angle.
    let adr: HashMap<String, usize> = emitted
        .driven
        .iter()
        .map(|j| {
            (
                j.joint.clone(),
                model.jnt_qpos_adr[joint_id(&model, &j.joint)],
            )
        })
        .collect();
    let drive = |th: f64, q: &mut [f64]| {
        let coords = HashMap::from([("knee_angle_r".to_string(), th)]);
        for (joint, val) in emitted.qpos_targets(&coords) {
            q[adr[&joint]] = val;
        }
    };
    let eps = 0.5 * DEG;
    const ABS_FLOOR_NM: f64 = 0.5;

    println!(
        "\n===== muscle-driven twin vs REAL {} =====",
        d["source"].as_str().unwrap()
    );
    let mut worst_force_gap = 0.0_f64; // Gate A: loaded actuator vs standalone (N)
    let mut moment_func = 0.0_f64; // reported: joint moment, functional ROM
    let mut moment_deep = 0.0_f64; // reported: joint moment, deep flexion (>75°)
    let mut n = 0usize;

    for m in MUSCLES {
        let tid = tendon_id(&model, m);
        let aid = model
            .actuator_name
            .iter()
            .position(|nm| nm.as_deref() == Some(m))
            .expect("muscle actuator must be emitted");
        let act_adr = model.actuator_act_adr[aid];
        // The Millard params the emit wired into this actuator's gainprm.
        let gp = &model.actuator_gainprm[aid];
        let params = MillardMuscleParams {
            f0: gp[2],
            l0: gp[4],
            lts: gp[5],
            vmax: gp[6],
            penn0: gp[7],
        };

        let mut mf = 0.0_f64;
        let mut md = 0.0_f64;
        for act_key in ["1.00", "0.50"] {
            let act: f64 = act_key.parse().unwrap();
            for row in d["muscles"][m].as_array().unwrap() {
                let deg = row["angle_deg"].as_f64().unwrap();
                let th = row["angle_rad"].as_f64().unwrap();
                let os = row["by_activation"][act_key]["knee_moment_Nm"]
                    .as_f64()
                    .unwrap();

                let mut data = model.make_data();
                drive(th, data.qpos.as_mut_slice());
                data.act[act_adr] = act;
                data.forward(&model).expect("forward failed");
                let force = data.actuator_force[aid]; // ≤ 0 (tension)

                // Gate A: the loaded twin's actuator force IS the validated Millard force
                // (engine convention is negative tension) at the twin's own actuator
                // (musculotendon) length — actuator_length = gear · tendon length, so this
                // is gear-correct (the emit uses gear 1). Static ⇒ tendon velocity 0.
                let standalone =
                    -millard_path_force(curves, params, data.actuator_length[aid], 0.0, act);
                worst_force_gap = worst_force_gap.max((force - standalone).abs());

                // Reported: end-to-end knee joint moment vs OpenSim.
                let arm = coupled_moment_arm(&model, tid, th, eps, &drive);
                let rel = (-force * arm - os).abs() / os.abs().max(ABS_FLOOR_NM);
                if deg.abs() <= 75.0 {
                    mf = mf.max(rel);
                } else {
                    md = md.max(rel);
                }
                n += 1;
            }
        }
        println!(
            "  {m:<11} joint-moment functional {:.2}%   deep(>75°) {:.1}%",
            mf * 100.0,
            md * 100.0
        );
        moment_func = moment_func.max(mf);
        moment_deep = moment_deep.max(md);
    }
    println!(
        "  Gate A (loaded force vs standalone) worst {worst_force_gap:.2e} N\n  \
         Joint moment vs OpenSim: functional worst {:.2}% (semimem extension arm), \
         deep-flexion worst {:.0}% (quad arm) — reported, the emit's dropped-conditional \
         via-point geometry   (n={n})\n",
        moment_func * 100.0,
        moment_deep * 100.0
    );

    // The machine-exact gate: the driven twin computes the validated Millard force.
    assert!(
        worst_force_gap < 1e-6,
        "loaded muscle-driven actuator force vs standalone worst {worst_force_gap:.2e} N exceeds 1e-6"
    );
}
