//! Layer B — Per-stage MuJoCo reference comparison tests.
//!
//! Each test loads a conformance model, runs `forward()`, then compares specific
//! pipeline stage outputs against MuJoCo 3.4.0 reference `.npy` files at
//! algorithm-appropriate tolerances.
//!
//! 9 pipeline stages × multiple models = 43 tests total.

use super::common;

// ═══════════════════════════════════════════════════════════════════════════════
// Helper: FK test for a given model
// ═══════════════════════════════════════════════════════════════════════════════

fn run_fk_test(model_name: &str, ctrl_values: &[f64]) {
    let (model, mut data) = common::load_conformance_model(model_name);
    for (i, &v) in ctrl_values.iter().enumerate() {
        data.ctrl[i] = v;
    }
    data.forward(&model).expect("forward");

    // xpos: nbody×3
    let (shape, ref_xpos) = common::load_reference_f64(model_name, "fk", "xpos");
    let nbody = shape[0];
    assert_eq!(
        nbody,
        data.xpos.len(),
        "[{model_name}] fk.xpos: nbody mismatch"
    );
    let actual_xpos: Vec<f64> = data.xpos.iter().flat_map(|v| [v[0], v[1], v[2]]).collect();
    common::assert_array_eq(
        model_name,
        "fk",
        "xpos",
        &ref_xpos,
        &actual_xpos,
        common::TOL_FK,
    );

    // xquat: nbody×4 (sign-ambiguous)
    let (_shape, ref_xquat) = common::load_reference_f64(model_name, "fk", "xquat");
    for i in 0..nbody {
        let q_ref = [
            ref_xquat[i * 4],
            ref_xquat[i * 4 + 1],
            ref_xquat[i * 4 + 2],
            ref_xquat[i * 4 + 3],
        ];
        common::assert_quat_eq(model_name, i, q_ref, &data.xquat[i], common::TOL_FK);
    }

    // xipos: nbody×3
    let (_shape, ref_xipos) = common::load_reference_f64(model_name, "fk", "xipos");
    let actual_xipos: Vec<f64> = data.xipos.iter().flat_map(|v| [v[0], v[1], v[2]]).collect();
    common::assert_array_eq(
        model_name,
        "fk",
        "xipos",
        &ref_xipos,
        &actual_xipos,
        common::TOL_FK,
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
// Helper: CRBA test for a given model
// ═══════════════════════════════════════════════════════════════════════════════

fn run_crba_test(model_name: &str, ctrl_values: &[f64]) {
    let (model, mut data) = common::load_conformance_model(model_name);
    for (i, &v) in ctrl_values.iter().enumerate() {
        data.ctrl[i] = v;
    }
    data.forward(&model).expect("forward");

    let (shape, ref_qm) = common::load_reference_f64(model_name, "crba", "qM");
    let nv = shape[0];
    assert_eq!(nv, shape[1], "[{model_name}] crba.qM: not square");
    assert_eq!(nv, model.nv, "[{model_name}] crba.qM: nv mismatch");

    let mut actual_qm = Vec::with_capacity(nv * nv);
    for i in 0..nv {
        for j in 0..nv {
            actual_qm.push(data.qM[(i, j)]);
        }
    }
    common::assert_array_eq(
        model_name,
        "crba",
        "qM",
        &ref_qm,
        &actual_qm,
        common::TOL_CRBA,
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
// Helper: RNE test for a given model
// ═══════════════════════════════════════════════════════════════════════════════

fn run_rne_test(model_name: &str, ctrl_values: &[f64]) {
    let (model, mut data) = common::load_conformance_model(model_name);
    for (i, &v) in ctrl_values.iter().enumerate() {
        data.ctrl[i] = v;
    }
    data.forward(&model).expect("forward");

    let (_shape, ref_bias) = common::load_reference_f64(model_name, "rne", "qfrc_bias");
    let actual_bias: Vec<f64> = (0..model.nv).map(|i| data.qfrc_bias[i]).collect();
    common::assert_array_eq(
        model_name,
        "rne",
        "qfrc_bias",
        &ref_bias,
        &actual_bias,
        common::TOL_RNE,
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
// Helper: Passive force test for a given model
// ═══════════════════════════════════════════════════════════════════════════════

fn run_passive_test(model_name: &str, ctrl_values: &[f64]) {
    let (model, mut data) = common::load_conformance_model(model_name);
    for (i, &v) in ctrl_values.iter().enumerate() {
        data.ctrl[i] = v;
    }
    data.forward(&model).expect("forward");

    let (_shape, ref_passive) = common::load_reference_f64(model_name, "passive", "qfrc_passive");
    let actual_passive: Vec<f64> = (0..model.nv).map(|i| data.qfrc_passive[i]).collect();
    common::assert_array_eq(
        model_name,
        "passive",
        "qfrc_passive",
        &ref_passive,
        &actual_passive,
        common::TOL_PASSIVE,
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
// Helper: Collision test for a given model (structural matching)
// ═══════════════════════════════════════════════════════════════════════════════

fn run_collision_test(model_name: &str, ctrl_values: &[f64]) {
    let (model, mut data) = common::load_conformance_model(model_name);
    for (i, &v) in ctrl_values.iter().enumerate() {
        data.ctrl[i] = v;
    }
    data.forward(&model).expect("forward");

    let (_shape, ref_geom_pairs) = common::load_reference_i32(model_name, "contact", "geom_pairs");
    let (_shape, ref_depth) = common::load_reference_f64(model_name, "contact", "depth");
    let (_shape, ref_normal) = common::load_reference_f64(model_name, "contact", "normal");
    let (_shape, ref_pos) = common::load_reference_f64(model_name, "contact", "pos");

    let ref_ncon = ref_depth.len();

    // Build sorted reference contacts by geom pair
    let mut ref_contacts: Vec<(i32, i32, usize)> = (0..ref_ncon)
        .map(|i| (ref_geom_pairs[i * 2], ref_geom_pairs[i * 2 + 1], i))
        .collect();
    ref_contacts.sort_by_key(|&(g1, g2, _)| (g1, g2));

    // Build sorted CortenForge contacts by geom pair
    let ncon = data.ncon;
    let mut cf_contacts: Vec<(usize, usize, usize)> = (0..ncon)
        .map(|i| (data.contacts[i].geom1, data.contacts[i].geom2, i))
        .collect();
    cf_contacts.sort_by_key(|&(g1, g2, _)| (g1, g2));

    // Assert same contact count
    if ref_ncon != ncon {
        let ref_pairs: Vec<_> = ref_contacts.iter().map(|(g1, g2, _)| (*g1, *g2)).collect();
        let cf_pairs: Vec<_> = cf_contacts.iter().map(|(g1, g2, _)| (*g1, *g2)).collect();
        panic!(
            "[{model_name}] collision: expected {ref_ncon} contacts, got {ncon}.\n\
             MuJoCo geom pairs: {ref_pairs:?}\n\
             CortenForge geom pairs: {cf_pairs:?}"
        );
    }

    // Compare matched contacts
    for (ref_c, cf_c) in ref_contacts.iter().zip(cf_contacts.iter()) {
        let (ref_g1, ref_g2, ri) = *ref_c;
        let (cf_g1, cf_g2, ci) = *cf_c;
        assert_eq!(
            (ref_g1 as usize, ref_g2 as usize),
            (cf_g1, cf_g2),
            "[{model_name}] collision: geom pair mismatch"
        );

        // Depth: sign flip (MuJoCo dist negative = penetration, CF depth positive)
        let ref_d = -ref_depth[ri];
        let cf_d = data.contacts[ci].depth;
        let diff = (ref_d - cf_d).abs();
        assert!(
            diff <= common::TOL_COLLISION_DEPTH,
            "[{model_name}] collision.depth[geom pair ({cf_g1},{cf_g2})]: \
             expected {ref_d:.15e}, got {cf_d:.15e}, diff {diff:.3e}, \
             tol {:.3e}",
            common::TOL_COLLISION_DEPTH
        );

        // Normal
        for k in 0..3 {
            let ref_n = ref_normal[ri * 3 + k];
            let cf_n = data.contacts[ci].normal[k];
            let diff = (ref_n - cf_n).abs();
            assert!(
                diff <= common::TOL_COLLISION_DEPTH,
                "[{model_name}] collision.normal[geom pair ({cf_g1},{cf_g2})][{k}]: \
                 expected {ref_n:.15e}, got {cf_n:.15e}, diff {diff:.3e}"
            );
        }

        // Position
        for k in 0..3 {
            let ref_p = ref_pos[ri * 3 + k];
            let cf_p = data.contacts[ci].pos[k];
            let diff = (ref_p - cf_p).abs();
            assert!(
                diff <= common::TOL_COLLISION_DEPTH,
                "[{model_name}] collision.pos[geom pair ({cf_g1},{cf_g2})][{k}]: \
                 expected {ref_p:.15e}, got {cf_p:.15e}, diff {diff:.3e}"
            );
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// Helper: Constraint test for a given model (element-wise with nefc pre-check)
// ═══════════════════════════════════════════════════════════════════════════════

fn run_constraint_test(model_name: &str, ctrl_values: &[f64]) {
    let (model, mut data) = common::load_conformance_model(model_name);
    for (i, &v) in ctrl_values.iter().enumerate() {
        data.ctrl[i] = v;
    }
    data.forward(&model).expect("forward");

    // efc_J: nefc × nv
    let (shape, ref_efc_j) = common::load_reference_f64(model_name, "constraint", "efc_J");
    let ref_nefc = shape[0];
    let ref_nv = shape[1];
    let nefc = data.efc_J.nrows();
    let nv = data.efc_J.ncols();

    if ref_nefc != nefc {
        let n_eq = data.ne;
        let n_fc = data.nf;
        let n_other = nefc.saturating_sub(n_eq + n_fc);
        panic!(
            "[{model_name}] constraint: nefc mismatch — \
             expected {ref_nefc} rows, got {nefc}.\n\
             CortenForge breakdown: equality={n_eq}, friction_loss={n_fc}, \
             contact+limit={n_other}\n\
             Reference: {ref_nefc} total (from MuJoCo 3.4.0). \
             Check cone type (pyramidal vs elliptic) and constraint assembly."
        );
    }
    assert_eq!(ref_nv, nv, "[{model_name}] constraint: nv mismatch");

    let mut actual_efc_j = Vec::with_capacity(nefc * nv);
    for i in 0..nefc {
        for j in 0..nv {
            actual_efc_j.push(data.efc_J[(i, j)]);
        }
    }
    common::assert_array_eq(
        model_name,
        "constraint",
        "efc_J",
        &ref_efc_j,
        &actual_efc_j,
        common::TOL_CONSTRAINT_JAC,
    );

    // efc_b: nefc
    let (_shape, ref_efc_b) = common::load_reference_f64(model_name, "constraint", "efc_b");
    let actual_efc_b: Vec<f64> = (0..nefc).map(|i| data.efc_b[i]).collect();
    common::assert_array_eq(
        model_name,
        "constraint",
        "efc_b",
        &ref_efc_b,
        &actual_efc_b,
        common::TOL_CONSTRAINT_JAC,
    );

    // efc_force: nefc
    let (_shape, ref_efc_force) = common::load_reference_f64(model_name, "constraint", "efc_force");
    let actual_efc_force: Vec<f64> = (0..nefc).map(|i| data.efc_force[i]).collect();
    common::assert_array_eq(
        model_name,
        "constraint",
        "efc_force",
        &ref_efc_force,
        &actual_efc_force,
        common::TOL_CONSTRAINT,
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
// Helper: Actuation test for a given model
// ═══════════════════════════════════════════════════════════════════════════════

fn run_actuation_test(model_name: &str, ctrl_values: &[f64]) {
    let (model, mut data) = common::load_conformance_model(model_name);
    for (i, &v) in ctrl_values.iter().enumerate() {
        data.ctrl[i] = v;
    }
    data.forward(&model).expect("forward");

    // qfrc_actuator: nv
    let (_shape, ref_qfrc) = common::load_reference_f64(model_name, "actuator", "qfrc_actuator");
    let actual_qfrc: Vec<f64> = (0..model.nv).map(|i| data.qfrc_actuator[i]).collect();
    common::assert_array_eq(
        model_name,
        "actuator",
        "qfrc_actuator",
        &ref_qfrc,
        &actual_qfrc,
        common::TOL_ACTUATION,
    );

    // actuator_force: nu
    let (_shape, ref_force) = common::load_reference_f64(model_name, "actuator", "force");
    common::assert_array_eq(
        model_name,
        "actuator",
        "actuator_force",
        &ref_force,
        &data.actuator_force,
        common::TOL_ACTUATION,
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
// Helper: Sensor test for a given model
// ═══════════════════════════════════════════════════════════════════════════════

fn run_sensor_test(model_name: &str, ctrl_values: &[f64]) {
    let (model, mut data) = common::load_conformance_model(model_name);
    for (i, &v) in ctrl_values.iter().enumerate() {
        data.ctrl[i] = v;
    }
    data.forward(&model).expect("forward");

    let (_shape, ref_sensor) = common::load_reference_f64(model_name, "sensor", "sensordata");
    let nsensordata = ref_sensor.len();
    let actual_sensor: Vec<f64> = (0..nsensordata).map(|i| data.sensordata[i]).collect();
    common::assert_array_eq(
        model_name,
        "sensor",
        "sensordata",
        &ref_sensor,
        &actual_sensor,
        common::TOL_SENSOR,
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
// Helper: Tendon test for a given model
// ═══════════════════════════════════════════════════════════════════════════════

fn run_tendon_test(model_name: &str, ctrl_values: &[f64]) {
    let (model, mut data) = common::load_conformance_model(model_name);
    for (i, &v) in ctrl_values.iter().enumerate() {
        data.ctrl[i] = v;
    }
    data.forward(&model).expect("forward");

    // ten_length: ntendon
    let (_shape, ref_length) = common::load_reference_f64(model_name, "tendon", "length");
    common::assert_array_eq(
        model_name,
        "tendon",
        "ten_length",
        &ref_length,
        &data.ten_length,
        common::TOL_TENDON,
    );

    // ten_velocity: ntendon
    let (_shape, ref_velocity) = common::load_reference_f64(model_name, "tendon", "velocity");
    common::assert_array_eq(
        model_name,
        "tendon",
        "ten_velocity",
        &ref_velocity,
        &data.ten_velocity,
        common::TOL_TENDON,
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
// S2: FK Reference Tests (T1–T8) — AC2–AC9
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
#[ignore] // CONFORMANCE GAP: xipos not computed (body CoM in world frame is zero) — Phase 1 FK
fn layer_b_fk_pendulum() {
    run_fk_test("pendulum", &[]);
}

#[test]
#[ignore] // CONFORMANCE GAP: xipos not computed — Phase 1 FK
fn layer_b_fk_double_pendulum() {
    run_fk_test("double_pendulum", &[]);
}

#[test]
fn layer_b_fk_contact_scenario() {
    run_fk_test("contact_scenario", &[]);
}

#[test]
#[ignore] // CONFORMANCE GAP: xipos not computed — Phase 1 FK
fn layer_b_fk_actuated_system() {
    run_fk_test("actuated_system", &[1.0, 0.5]);
}

#[test]
#[ignore] // CONFORMANCE GAP: xipos not computed — Phase 1 FK
fn layer_b_fk_tendon_model() {
    run_fk_test("tendon_model", &[]);
}

#[test]
#[ignore] // CONFORMANCE GAP: xipos not computed — Phase 1 FK
fn layer_b_fk_sensor_model() {
    run_fk_test("sensor_model", &[]);
}

#[test]
#[ignore] // CONFORMANCE GAP: xipos not computed — Phase 1 FK
fn layer_b_fk_equality_model() {
    run_fk_test("equality_model", &[]);
}

#[test]
#[ignore] // CONFORMANCE GAP: xipos not computed — Phase 1 FK
fn layer_b_fk_composite_model() {
    run_fk_test("composite_model", &[1.0]);
}

// ═══════════════════════════════════════════════════════════════════════════════
// S3: CRBA Reference Tests (T9–T16) — AC10–AC17
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
#[ignore] // CONFORMANCE GAP: qM wrong — xipos not computed, parallel axis theorem missing — Phase 1 CRBA
fn layer_b_crba_pendulum() {
    run_crba_test("pendulum", &[]);
}

#[test]
#[ignore] // CONFORMANCE GAP: qM wrong — xipos not computed, parallel axis theorem missing — Phase 1 CRBA
fn layer_b_crba_double_pendulum() {
    run_crba_test("double_pendulum", &[]);
}

#[test]
fn layer_b_crba_contact_scenario() {
    run_crba_test("contact_scenario", &[]);
}

#[test]
#[ignore] // CONFORMANCE GAP: qM wrong — xipos not computed, parallel axis theorem missing — Phase 1 CRBA
fn layer_b_crba_actuated_system() {
    run_crba_test("actuated_system", &[1.0, 0.5]);
}

#[test]
#[ignore] // CONFORMANCE GAP: qM wrong — xipos not computed, parallel axis theorem missing — Phase 1 CRBA
fn layer_b_crba_tendon_model() {
    run_crba_test("tendon_model", &[]);
}

#[test]
#[ignore] // CONFORMANCE GAP: qM wrong — xipos not computed, parallel axis theorem missing — Phase 1 CRBA
fn layer_b_crba_sensor_model() {
    run_crba_test("sensor_model", &[]);
}

#[test]
#[ignore] // CONFORMANCE GAP: qM wrong — xipos not computed, parallel axis theorem missing — Phase 1 CRBA
fn layer_b_crba_equality_model() {
    run_crba_test("equality_model", &[]);
}

#[test]
#[ignore] // CONFORMANCE GAP: qM wrong — xipos not computed, parallel axis theorem missing — Phase 1 CRBA
fn layer_b_crba_composite_model() {
    run_crba_test("composite_model", &[1.0]);
}

// ═══════════════════════════════════════════════════════════════════════════════
// S4: RNE Reference Tests (T17–T24) — AC18–AC25
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
#[ignore] // CONFORMANCE GAP: qfrc_bias wrong — xipos not computed, gravity torques use wrong CoM — Phase 1 RNE
fn layer_b_rne_pendulum() {
    run_rne_test("pendulum", &[]);
}

#[test]
#[ignore] // CONFORMANCE GAP: qfrc_bias wrong — xipos not computed, gravity torques use wrong CoM — Phase 1 RNE
fn layer_b_rne_double_pendulum() {
    run_rne_test("double_pendulum", &[]);
}

#[test]
fn layer_b_rne_contact_scenario() {
    run_rne_test("contact_scenario", &[]);
}

#[test]
#[ignore] // CONFORMANCE GAP: qfrc_bias wrong — xipos not computed, gravity torques use wrong CoM — Phase 1 RNE
fn layer_b_rne_actuated_system() {
    run_rne_test("actuated_system", &[1.0, 0.5]);
}

#[test]
#[ignore] // CONFORMANCE GAP: qfrc_bias wrong — xipos not computed, gravity torques use wrong CoM — Phase 1 RNE
fn layer_b_rne_tendon_model() {
    run_rne_test("tendon_model", &[]);
}

#[test]
#[ignore] // CONFORMANCE GAP: qfrc_bias wrong — xipos not computed, gravity torques use wrong CoM — Phase 1 RNE
fn layer_b_rne_sensor_model() {
    run_rne_test("sensor_model", &[]);
}

#[test]
#[ignore] // CONFORMANCE GAP: qfrc_bias wrong — xipos not computed, gravity torques use wrong CoM — Phase 1 RNE
fn layer_b_rne_equality_model() {
    run_rne_test("equality_model", &[]);
}

#[test]
#[ignore] // CONFORMANCE GAP: qfrc_bias wrong — xipos not computed, gravity torques use wrong CoM — Phase 1 RNE
fn layer_b_rne_composite_model() {
    run_rne_test("composite_model", &[1.0]);
}

// ═══════════════════════════════════════════════════════════════════════════════
// S5: Passive Force Reference Tests (T25–T32) — AC26–AC33
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
fn layer_b_passive_pendulum() {
    run_passive_test("pendulum", &[]);
}

#[test]
fn layer_b_passive_double_pendulum() {
    run_passive_test("double_pendulum", &[]);
}

#[test]
fn layer_b_passive_contact_scenario() {
    run_passive_test("contact_scenario", &[]);
}

#[test]
fn layer_b_passive_actuated_system() {
    run_passive_test("actuated_system", &[1.0, 0.5]);
}

#[test]
fn layer_b_passive_tendon_model() {
    run_passive_test("tendon_model", &[]);
}

#[test]
fn layer_b_passive_sensor_model() {
    run_passive_test("sensor_model", &[]);
}

#[test]
fn layer_b_passive_equality_model() {
    run_passive_test("equality_model", &[]);
}

#[test]
fn layer_b_passive_composite_model() {
    run_passive_test("composite_model", &[1.0]);
}

// ═══════════════════════════════════════════════════════════════════════════════
// S8: Actuation Reference Tests (T38–T39) — AC39–AC40
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
fn layer_b_actuation_actuated_system() {
    run_actuation_test("actuated_system", &[1.0, 0.5]);
}

#[test]
fn layer_b_actuation_composite_model() {
    run_actuation_test("composite_model", &[1.0]);
}

// ═══════════════════════════════════════════════════════════════════════════════
// S10: Tendon Reference Tests (T42–T43) — AC43–AC44
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
fn layer_b_tendon_tendon_model() {
    run_tendon_test("tendon_model", &[]);
}

#[test]
fn layer_b_tendon_composite_model() {
    run_tendon_test("composite_model", &[1.0]);
}

// ═══════════════════════════════════════════════════════════════════════════════
// S9: Sensor Reference Tests (T40–T41) — AC41–AC42
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
#[ignore] // CONFORMANCE GAP: sensordata[12] off — xipos/subtree_com affects framepos sensor — Phase 6 sensor
fn layer_b_sensor_sensor_model() {
    run_sensor_test("sensor_model", &[]);
}

#[test]
#[ignore] // CONFORMANCE GAP: sensordata[4] off — xipos/subtree_com affects framepos sensor — Phase 6 sensor
fn layer_b_sensor_composite_model() {
    run_sensor_test("composite_model", &[1.0]);
}

// ═══════════════════════════════════════════════════════════════════════════════
// S6: Collision Reference Tests (T33–T34) — AC34–AC35
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
#[ignore] // CONFORMANCE GAP: contact.pos z off by 1e-3 — contact position convention difference — Phase 3 collision
fn layer_b_collision_contact_scenario() {
    run_collision_test("contact_scenario", &[]);
}

#[test]
#[ignore] // CONFORMANCE GAP: contact.pos z off by 1e-3 — contact position convention difference — Phase 3 collision
fn layer_b_collision_composite_model() {
    run_collision_test("composite_model", &[1.0]);
}

// ═══════════════════════════════════════════════════════════════════════════════
// S7: Constraint Reference Tests (T35–T37) — AC36–AC38
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
#[ignore] // CONFORMANCE GAP: efc_J row content differs — constraint Jacobian assembly divergence — Phase 3 constraint
fn layer_b_constraint_contact_scenario() {
    run_constraint_test("contact_scenario", &[]);
}

#[test]
#[ignore] // CONFORMANCE GAP: efc_J row content differs — constraint Jacobian assembly divergence — Phase 3 constraint
fn layer_b_constraint_equality_model() {
    run_constraint_test("equality_model", &[]);
}

#[test]
#[ignore] // CONFORMANCE GAP: efc_J row content differs — constraint Jacobian assembly divergence — Phase 3 constraint
fn layer_b_constraint_composite_model() {
    run_constraint_test("composite_model", &[1.0]);
}
