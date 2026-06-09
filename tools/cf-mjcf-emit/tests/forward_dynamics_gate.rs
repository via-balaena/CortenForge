//! G3-PR2b: the muscle-driven twin's FORWARD DYNAMICS (activations → joint
//! ACCELERATION) matches OpenSim's `Manager` forward dynamics — the end-to-end
//! "activations → motion" check that closes the G3 loop.
//!
//! Apples-to-apples with `gen_forward_dynamics.py`: reduced 5-DOF right leg, GRAVITY
//! OFF (isolate muscle→accel), rigid tendon, activations set directly, and the
//! gait2392 segment inertias INJECTED (the emit ships placeholder inertias, so the
//! gate injects the matched mass matrix to validate the forward-dynamics SOLVER +
//! the coupled-knee equality constraint + the Millard force — not anthropometry,
//! which is a separate concern). The slides are posed ON the constraint manifold
//! (`poly(knee)`), and only the 4 right-leg muscles apply force.
//!
//! **GATE:** at the functional-ROM sample angles (knee −0.3/−0.55/−0.8 rad ≈
//! 17°/31°/46°), the full-activation knee acceleration matches OpenSim within 8% —
//! possible because G3-PR2b stiffened the coupled-knee equality impedance enough to
//! couple the heavy shank+foot-bearing tibia slides at the ACCELERATION level (a
//! too-soft constraint left the knee's effective inertia wrong and the udot ~2.4×
//! too high). The 8% budget leaves headroom over the ~3.2% actual match (for
//! cross-platform float/solver variance) while staying well below the
//! geometry-limited deep-flexion residual. **REPORTED:** deep flexion + per-muscle, where the
//! residual is the emit's dropped-conditional via-point GEOMETRY (semimem extension,
//! quad deep-flexion — the same limit PR1 documented for the static moment arm), not
//! a dynamics error.

use cf_mjcf_emit::emit;
use cf_osim::parse_leg_chain;
use nalgebra::{Quaternion, UnitQuaternion, Vector3};
use serde_json::Value;
use sim_core::EqualityType;
use sim_mjcf::load_model;
use std::collections::HashMap;

fn asset(name: &str) -> String {
    format!(
        "{}/../../sim/L0/tests/assets/opensim_gait2392/{name}",
        env!("CARGO_MANIFEST_DIR")
    )
}

fn poly(coef: &[f64], q: f64) -> f64 {
    let mut p = coef[coef.len() - 1];
    for k in (0..coef.len() - 1).rev() {
        p = p * q + coef[k];
    }
    p
}

fn v3(a: &Value) -> Vector3<f64> {
    let v = a.as_array().unwrap();
    Vector3::new(
        v[0].as_f64().unwrap(),
        v[1].as_f64().unwrap(),
        v[2].as_f64().unwrap(),
    )
}

#[test]
fn forward_dynamics_matches_opensim() {
    let template = parse_leg_chain(&std::fs::read_to_string(asset("gait2392.osim")).unwrap());
    let emitted = emit(&template);
    let mut model = load_model(&emitted.mjcf).expect("twin loads");
    let d: Value = serde_json::from_str(
        &std::fs::read_to_string(asset("forward_dynamics_opensim.json")).unwrap(),
    )
    .unwrap();

    // Match OpenSim: gravity off + inject the gait2392 segment inertias.
    model.gravity = Vector3::zeros();
    for (name, iner) in d["inertias"].as_object().unwrap() {
        let b = model.body_name_to_id[name];
        model.body_mass[b] = iner["mass"].as_f64().unwrap();
        model.body_ipos[b] = v3(&iner["com"]);
        model.body_inertia[b] = v3(&iner["idiag"]);
        let q = iner["iquat"].as_array().unwrap();
        model.body_iquat[b] = UnitQuaternion::from_quaternion(Quaternion::new(
            q[0].as_f64().unwrap(),
            q[1].as_f64().unwrap(),
            q[2].as_f64().unwrap(),
            q[3].as_f64().unwrap(),
        ));
    }

    let adr: HashMap<String, usize> = emitted
        .driven
        .iter()
        .map(|j| {
            let jid = model
                .jnt_name
                .iter()
                .position(|n| n.as_deref() == Some(&j.joint))
                .unwrap();
            (j.joint.clone(), model.jnt_qpos_adr[jid])
        })
        .collect();
    let knee_dof = {
        let jid = model
            .jnt_name
            .iter()
            .position(|n| n.as_deref() == Some("knee_angle_r"))
            .unwrap();
        model.jnt_dof_adr[jid]
    };
    // Coupled slides (slide qpos adr, knee qpos adr, polycoef) — to pose on-manifold.
    let couplings: Vec<(usize, usize, [f64; 11])> = (0..model.neq)
        .filter(|&e| model.eq_type[e] == EqualityType::Joint && model.eq_obj2id[e] < model.njnt)
        .map(|e| {
            (
                model.jnt_qpos_adr[model.eq_obj1id[e]],
                model.jnt_qpos_adr[model.eq_obj2id[e]],
                model.eq_data[e],
            )
        })
        .collect();
    let act_adr = |m: &str| {
        let aid = model
            .actuator_name
            .iter()
            .position(|n| n.as_deref() == Some(m))
            .unwrap();
        model.actuator_act_adr[aid]
    };
    let muscles: Vec<String> = d["muscles"]
        .as_array()
        .unwrap()
        .iter()
        .map(|m| m.as_str().unwrap().to_string())
        .collect();

    // Pose at `knee` on the constraint manifold and set the given activations.
    let pose = |model: &sim_core::Model, knee: f64, acts: &[(&str, f64)]| {
        let mut data = model.make_data();
        let coords = HashMap::from([("knee_angle_r".to_string(), knee)]);
        for (joint, val) in emitted.qpos_targets(&coords) {
            data.qpos[adr[&joint]] = val;
        }
        for &(s, k, c) in &couplings {
            data.qpos[s] = poly(&c, data.qpos[k]);
        }
        for &(m, a) in acts {
            data.act[act_adr(m)] = a;
        }
        data.forward(model).expect("forward");
        data
    };

    println!(
        "\n===== muscle-driven twin FORWARD DYNAMICS vs {} =====",
        d["source"].as_str().unwrap()
    );
    let mut worst_func = 0.0_f64;
    let mut worst_deep = 0.0_f64;
    for case in d["cases"].as_array().unwrap() {
        let knee = case["knee_rad"].as_f64().unwrap();
        let os_full = case["full_activation_knee_udot"].as_f64().unwrap();
        let all: Vec<(&str, f64)> = muscles.iter().map(|m| (m.as_str(), 1.0)).collect();
        let twin_full = pose(&model, knee, &all).qacc[knee_dof];
        let rel = (twin_full - os_full).abs() / os_full.abs();
        if knee.abs() <= 1.0 {
            worst_func = worst_func.max(rel);
        } else {
            worst_deep = worst_deep.max(rel);
        }
        // Per-muscle (reported): others fully off (gainprm F0 -> 0 isolates this muscle).
        let mut solo = String::new();
        for m in &muscles {
            let os = case["solo_knee_udot"][m].as_f64().unwrap();
            let mut m_only = model.clone();
            for aid in 0..m_only.actuator_gainprm.len() {
                if m_only.actuator_name[aid].as_deref() != Some(m) {
                    m_only.actuator_gainprm[aid][2] = 0.0;
                }
            }
            let twin = pose(&m_only, knee, &[(m, 1.0)]).qacc[knee_dof];
            solo.push_str(&format!(" {}:{:+.0}/{:+.0}", &m[..4], twin, os));
        }
        println!(
            "  knee={knee:+.2}  full_act twin {twin_full:+.1} vs OS {os_full:+.1}  ({:.1}%)  solo[twin/OS]{solo}",
            rel * 100.0
        );
    }
    println!(
        "  GATE functional-ROM worst {:.1}%   (deep-flexion {:.1}% + per-muscle = the emit's \
         dropped-conditional via-point geometry, reported)\n",
        worst_func * 100.0,
        worst_deep * 100.0
    );

    // 8% budget: headroom over the ~3.2% actual match (cross-platform float/solver
    // variance), well under the geometry-limited deep-flexion residual (~12.7%).
    assert!(
        worst_func < 0.08,
        "functional-ROM forward-dynamics knee acceleration vs OpenSim worst {:.1}% exceeds 8%",
        worst_func * 100.0
    );
}
