//! G3-PR1: the coupled knee is held by engine EQUALITY CONSTRAINTS, so it can move
//! under torque (forward dynamics) instead of being posed kinematically.
//!
//! The emit now lowers each coupled (SimmSpline-driven) slide as a degree-8
//! polynomial joint-equality to the knee coordinate, plus ROM range limits. Two
//! checks:
//!
//! * **Static (the constraint reproduces the kinematic pose):** across the knee
//!   ROM, the polynomial constraint target `poly(knee)` matches the SimmSpline pose
//!   the kinematic `qpos_targets` path sets — i.e. the degree-8 fit residual is
//!   ≤0.2 mm everywhere. So switching the coupled knee from kinematic to dynamic
//!   does not move the model.
//! * **Dynamic (the solver holds the manifold under motion):** short forward-
//!   dynamics sweeps seeded with constraint-consistent initial velocities across the
//!   whole ROM — including DEEP FLEXION, the hardest region — keep every coupled
//!   slide on its manifold (`q_slide = poly(knee)`) to ~1 mm while the knee moves
//!   under its own dynamics. The thing that was impossible with kinematic driving.

use cf_mjcf_emit::emit;
use cf_osim::parse_leg_chain;
use sim_core::EqualityType;
use sim_mjcf::load_model;
use std::collections::HashMap;
use std::f64::consts::PI;

const DEG: f64 = PI / 180.0;

fn asset(name: &str) -> String {
    format!(
        "{}/../../sim/L0/tests/assets/opensim_gait2392/{name}",
        env!("CARGO_MANIFEST_DIR")
    )
}

/// Horner evaluation of a joint-equality polynomial (`eq_data` coefficients).
fn poly(coef: &[f64], q: f64) -> f64 {
    let mut p = coef[coef.len() - 1];
    for k in (0..coef.len() - 1).rev() {
        p = p * q + coef[k];
    }
    p
}

/// Horner evaluation of the polynomial's derivative `poly'(q)`.
fn poly_deriv(coef: &[f64], q: f64) -> f64 {
    let mut d = 0.0;
    let mut p = coef[coef.len() - 1];
    for k in (0..coef.len() - 1).rev() {
        d = d * q + p;
        p = p * q + coef[k];
    }
    d
}

/// One coupled-knee joint equality: the dependent slide's qpos + dof addresses, the
/// independent knee's qpos + dof addresses, and the polynomial coefficients.
struct Coupling {
    slide_qpos: usize,
    slide_dof: usize,
    knee_qpos: usize,
    knee_dof: usize,
    coef: [f64; 11],
}

fn joint_couplings(model: &sim_core::Model) -> Vec<Coupling> {
    (0..model.neq)
        .filter(|&e| model.eq_type[e] == EqualityType::Joint && model.eq_obj2id[e] < model.njnt)
        .map(|e| Coupling {
            slide_qpos: model.jnt_qpos_adr[model.eq_obj1id[e]],
            slide_dof: model.jnt_dof_adr[model.eq_obj1id[e]],
            knee_qpos: model.jnt_qpos_adr[model.eq_obj2id[e]],
            knee_dof: model.jnt_dof_adr[model.eq_obj2id[e]],
            coef: model.eq_data[e],
        })
        .collect()
}

/// G3-PR2a regression: the emit declares `<compiler angle="radian"/>`, so the
/// joint ranges are the coordinate ROMs in RADIANS — not silently treated as
/// degrees and shrunk ~57× into a bogus range the joint is always outside (which
/// makes the limit fire mid-range and inject a phantom constraint force that
/// corrupts forward dynamics). Asserts the loaded range is radians, and that a
/// MID-RANGE twin with NO force/gravity/velocity has zero acceleration.
#[test]
fn emitted_joint_limits_are_radian_no_midrange_phantom() {
    let template = parse_leg_chain(&std::fs::read_to_string(asset("gait2392.osim")).unwrap());
    let emitted = emit(&template);
    let mut model = load_model(&emitted.mjcf).expect("emitted twin must load");

    // The knee range must be the gait2392 ROM in radians, not degree-shrunk.
    let kj = model
        .jnt_name
        .iter()
        .position(|n| n.as_deref() == Some("knee_angle_r"))
        .unwrap();
    let (lo, hi) = model.jnt_range[kj];
    assert!(
        (lo + 2.0943951).abs() < 1e-6 && (hi - 0.17453293).abs() < 1e-6,
        "knee range must be radians (-2.0943951, 0.17453293), got ({lo}, {hi})"
    );

    // A mid-range twin with NO muscle force (F0=0), gravity off, zero velocity must
    // have ~0 acceleration. A limit firing mid-range (wrong range) would inject a
    // phantom force; this is the behavioral guard for the degree/radian bug.
    model.gravity = nalgebra::Vector3::zeros();
    for aid in 0..model.actuator_gainprm.len() {
        model.actuator_gainprm[aid][2] = 0.0; // F0 = 0 ⇒ no active and no passive force
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
    let couplings = joint_couplings(&model);
    let mut data = model.make_data();
    let coords = HashMap::from([("knee_angle_r".to_string(), -0.5)]); // mid-range
    for (joint, val) in emitted.qpos_targets(&coords) {
        data.qpos[adr[&joint]] = val;
    }
    for c in &couplings {
        data.qpos[c.slide_qpos] = poly(&c.coef, data.qpos[c.knee_qpos]);
    }
    data.forward(&model).expect("forward");
    let worst = data.qacc.iter().fold(0.0_f64, |m, &x| m.max(x.abs()));
    assert!(
        worst < 1e-6,
        "mid-range no-force twin must not accelerate (limit phantom?), worst |qacc| = {worst}"
    );
}

#[test]
fn equality_constraint_reproduces_kinematic_pose_across_rom() {
    let template = parse_leg_chain(&std::fs::read_to_string(asset("gait2392.osim")).unwrap());
    let emitted = emit(&template);
    let model = load_model(&emitted.mjcf).expect("emitted twin must load");

    // jnt_qpos_adr per emitted (kinematically) driven joint, to pose via qpos_targets.
    let adr: HashMap<String, usize> = emitted
        .driven
        .iter()
        .map(|j| {
            let jid = model
                .jnt_name
                .iter()
                .position(|n| n.as_deref() == Some(&j.joint));
            (
                j.joint.clone(),
                model.jnt_qpos_adr[jid.expect("driven joint in model")],
            )
        })
        .collect();
    let couplings = joint_couplings(&model);
    assert_eq!(
        couplings.len(),
        8,
        "expected 8 coupled-knee equality constraints"
    );

    let mut worst = 0.0_f64;
    let mut data = model.make_data();
    for deg in (-115..=5).step_by(5) {
        let th = deg as f64 * DEG;
        let coords = HashMap::from([("knee_angle_r".to_string(), th)]);
        for (joint, val) in emitted.qpos_targets(&coords) {
            data.qpos[adr[&joint]] = val;
        }
        // Residual = |SimmSpline pose (qpos_targets) − polynomial constraint target|.
        for c in &couplings {
            let resid = data.qpos[c.slide_qpos] - poly(&c.coef, data.qpos[c.knee_qpos]);
            worst = worst.max(resid.abs());
        }
    }
    println!(
        "\nstatic worst |spline − poly| across ROM: {:.4} mm",
        worst * 1000.0
    );
    assert!(
        worst < 2e-4,
        "degree-8 constraint should reproduce the kinematic SimmSpline pose to ≤0.2 mm, worst {:.4} mm",
        worst * 1000.0
    );
}

#[test]
fn equality_constraint_holds_manifold_under_forward_dynamics() {
    let template = parse_leg_chain(&std::fs::read_to_string(asset("gait2392.osim")).unwrap());
    let emitted = emit(&template);
    let model = load_model(&emitted.mjcf).expect("emitted twin must load");

    let adr: HashMap<String, usize> = emitted
        .driven
        .iter()
        .map(|j| {
            let jid = model
                .jnt_name
                .iter()
                .position(|n| n.as_deref() == Some(&j.joint));
            (
                j.joint.clone(),
                model.jnt_qpos_adr[jid.expect("driven joint in model")],
            )
        })
        .collect();
    let couplings = joint_couplings(&model);
    let knee_qpos = couplings[0].knee_qpos;
    let knee_dof = couplings[0].knee_dof;

    // Sample short FORWARD-DYNAMICS sweeps seeded across the WHOLE ROM — including
    // DEEP FLEXION, the hardest region for the coupling (tibia-tx is wiggliest and
    // the patella-x slopes are steepest there). Driving the free-floating leg to a
    // target angle by torque is unreliable (knee torque reacts into the thigh), so
    // each sample instead starts ON the manifold at a knee angle with a CONSTRAINT-
    // CONSISTENT initial velocity (knee velocity + each slide's `poly'(knee)·v`), then
    // integrates: genuine forward dynamics, and the constraint must hold the slides
    // on the manifold throughout. Together the samples cover the full ROM under
    // motion, not just the gentle near-extension band a passive swing reaches.
    let starts = [-1.85, -1.5, -1.1, -0.7, -0.3]; // rad; -1.85 ≈ 106° flexion
    let mut kmin = f64::INFINITY;
    let mut worst = 0.0_f64;
    let mut worst_start = 0.0_f64;
    for &start in &starts {
        let mut data = model.make_data();
        let coords = HashMap::from([("knee_angle_r".to_string(), start)]);
        for (joint, val) in emitted.qpos_targets(&coords) {
            data.qpos[adr[&joint]] = val;
        }
        // Constraint-consistent initial velocity: extend at +3 rad/s, slides follow.
        let kv = 3.0;
        data.qvel[knee_dof] = kv;
        for c in &couplings {
            data.qvel[c.slide_dof] = poly_deriv(&c.coef, start) * kv;
        }
        data.forward(&model).expect("forward");

        for _ in 0..150 {
            data.step(&model).expect("step");
            let knee = data.qpos[knee_qpos];
            assert!(knee.is_finite(), "blow-up: knee {knee}");
            kmin = kmin.min(knee);
            for c in &couplings {
                let resid = (data.qpos[c.slide_qpos] - poly(&c.coef, data.qpos[c.knee_qpos])).abs();
                if resid > worst {
                    worst = resid;
                    worst_start = start;
                }
            }
        }
    }
    println!(
        "\ndynamics: covered down to {:.1}° flexion   worst |slide − poly(knee)| {:.4} mm (seed {:.2} rad)",
        kmin / DEG,
        worst * 1000.0,
        worst_start
    );
    // The samples must reach DEEP FLEXION (≥ ~95°), where the coupling fit is hardest.
    assert!(
        kmin < -95.0 * DEG,
        "forward-dynamics samples must cover deep flexion, reached only {:.1}°",
        kmin / DEG
    );
    // ~1.1 mm worst is in the deepest-flexion sample (steepest patella slopes, ≈2%
    // of the 57 mm excursion) — far under the ~8.5 mm a too-soft constraint produces.
    assert!(
        worst < 1.5e-3,
        "coupled slides must stay on the constraint manifold under dynamics, worst {:.4} mm",
        worst * 1000.0
    );
}
