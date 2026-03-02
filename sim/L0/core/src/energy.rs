//! Energy queries — potential and kinetic energy computation.
//!
//! `mj_energy_pos` computes gravitational + spring potential energy (called after
//! FK). `mj_energy_vel` computes kinetic energy from the mass matrix or body
//! velocities (called after CRBA). `Data::total_energy()` returns the sum.

use crate::tendon::subquat;
use crate::types::{
    DISABLE_GRAVITY, DISABLE_SPRING, Data, ENABLE_SLEEP, MjJointType, Model, SleepState,
};
use nalgebra::{Quaternion, UnitQuaternion, Vector3};

/// Compute potential energy (gravitational + spring).
///
/// Following MuJoCo `mj_energyPos()` semantics:
/// - Phase 1: Gravitational PE — unconditional (no sleep filter).
/// - Phase 2: Spring PE — gated on `DISABLE_SPRING`, sleep filter applied.
///   Guard: `stiffness == 0.0` (exact zero; negative stiffness processed).
///   Ball/free: quaternion geodesic distance via `subquat()`.
pub(crate) fn mj_energy_pos(model: &Model, data: &mut Data) {
    let mut potential = 0.0;

    // S4.2: Effective gravity — zero when DISABLE_GRAVITY is set.
    let grav = if model.disableflags & DISABLE_GRAVITY != 0 {
        Vector3::zeros()
    } else {
        model.gravity
    };

    // Phase 1: Gravitational potential energy — unconditional (no sleep filter).
    // MuJoCo computes gravity PE for ALL bodies regardless of sleep state.
    for body_id in 1..model.nbody {
        let mass = model.body_mass[body_id];
        let com = data.xipos[body_id];
        potential -= mass * grav.dot(&com);
    }

    // Phase 2: Spring potential energy — gated on DISABLE_SPRING + sleep filter.
    if model.disableflags & DISABLE_SPRING == 0 {
        let sleep_enabled = model.enableflags & ENABLE_SLEEP != 0;

        for jnt_id in 0..model.njnt {
            let stiffness = model.jnt_stiffness[jnt_id];
            // MuJoCo: `if (stiffness == 0) continue;` — exact zero check.
            // Processes negative stiffness (though physically meaningless).
            if stiffness == 0.0 {
                continue;
            }

            // Sleep filter: skip sleeping bodies' springs.
            if sleep_enabled {
                let body_id = model.jnt_body[jnt_id];
                if data.body_sleep_state[body_id] == SleepState::Asleep {
                    continue;
                }
            }

            let qpos_adr = model.jnt_qpos_adr[jnt_id];

            match model.jnt_type[jnt_id] {
                MjJointType::Hinge | MjJointType::Slide => {
                    let q = data.qpos[qpos_adr];
                    let springref = model.qpos_spring[qpos_adr];
                    let displacement = q - springref;
                    potential += 0.5 * stiffness * displacement * displacement;
                }
                MjJointType::Free => {
                    // Translational: E = 0.5 * k * ||pos - pos_spring||²
                    let mut trans_sq = 0.0;
                    for i in 0..3 {
                        let d = data.qpos[qpos_adr + i] - model.qpos_spring[qpos_adr + i];
                        trans_sq += d * d;
                    }
                    potential += 0.5 * stiffness * trans_sq;

                    // Falls through to ball case for rotational energy.
                    let quat_adr = qpos_adr + 3;
                    let q_cur = UnitQuaternion::new_normalize(Quaternion::new(
                        data.qpos[quat_adr],
                        data.qpos[quat_adr + 1],
                        data.qpos[quat_adr + 2],
                        data.qpos[quat_adr + 3],
                    ));
                    let q_spring = UnitQuaternion::new_normalize(Quaternion::new(
                        model.qpos_spring[quat_adr],
                        model.qpos_spring[quat_adr + 1],
                        model.qpos_spring[quat_adr + 2],
                        model.qpos_spring[quat_adr + 3],
                    ));
                    let dif = subquat(&q_cur, &q_spring);
                    potential += 0.5 * stiffness * dif.dot(&dif);
                }
                MjJointType::Ball => {
                    let q_cur = UnitQuaternion::new_normalize(Quaternion::new(
                        data.qpos[qpos_adr],
                        data.qpos[qpos_adr + 1],
                        data.qpos[qpos_adr + 2],
                        data.qpos[qpos_adr + 3],
                    ));
                    let q_spring = UnitQuaternion::new_normalize(Quaternion::new(
                        model.qpos_spring[qpos_adr],
                        model.qpos_spring[qpos_adr + 1],
                        model.qpos_spring[qpos_adr + 2],
                        model.qpos_spring[qpos_adr + 3],
                    ));
                    let dif = subquat(&q_cur, &q_spring);
                    potential += 0.5 * stiffness * dif.dot(&dif);
                }
            }
        }
    }

    data.energy_potential = potential;
}

/// Compute kinetic energy from mass matrix and velocities.
///
/// `E_k` = 0.5 * qvel^T * M * qvel
///
/// This must be called AFTER `mj_crba()` has computed the mass matrix.
/// However, for the `forward()` pipeline we call it after velocity FK
/// but before CRBA. We use an approximation based on body velocities
/// when M is not yet available.
pub(crate) fn mj_energy_vel(model: &Model, data: &mut Data) {
    // If mass matrix is available and computed, use exact formula
    // E_k = 0.5 * qvel^T * M * qvel
    let kinetic = if data.qM.nrows() == model.nv && model.nv > 0 {
        let m_qvel = &data.qM * &data.qvel;
        0.5 * data.qvel.dot(&m_qvel)
    } else {
        // Fallback: compute from body velocities directly (König's theorem)
        // E_k = 0.5 * Σ (m_i * |v_com_i|² + ω_i^T * I_i * ω_i)
        let mut energy = 0.0;
        for body_id in 1..model.nbody {
            let mass = model.body_mass[body_id];
            let inertia = model.body_inertia[body_id];

            // Extract linear and angular velocity from spatial velocity
            let omega = Vector3::new(
                data.cvel[body_id][0],
                data.cvel[body_id][1],
                data.cvel[body_id][2],
            );
            let v_origin = Vector3::new(
                data.cvel[body_id][3],
                data.cvel[body_id][4],
                data.cvel[body_id][5],
            );

            // Shift from body origin (xpos) to body COM (xipos)
            let com_offset = data.xipos[body_id] - data.xpos[body_id];
            let v_com = v_origin + omega.cross(&com_offset);

            // Translational kinetic energy: 0.5 * m * |v_com|²
            energy += 0.5 * mass * v_com.norm_squared();

            // Rotational kinetic energy: 0.5 * ω^T * I * ω
            // Using diagonal inertia in body frame (approximation - should transform)
            let omega_body = data.xquat[body_id].inverse() * omega;
            energy += 0.5
                * (inertia.x * omega_body.x.powi(2)
                    + inertia.y * omega_body.y.powi(2)
                    + inertia.z * omega_body.z.powi(2));
        }
        energy
    };

    data.energy_kinetic = kinetic;
}

impl Data {
    /// Get total mechanical energy (kinetic + potential).
    #[must_use]
    pub fn total_energy(&self) -> f64 {
        self.energy_kinetic + self.energy_potential
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod spec_b_tests {
    //! Spec B — Joint Physics tests (T1-T17).
    //! Ball/free spring force, energy, margin, DISABLE_SPRING, sleep filter, etc.

    use crate::types::{
        DISABLE_SPRING, ENABLE_ENERGY, ENABLE_SLEEP, Integrator, MjJointType, Model, SleepState,
    };
    use nalgebra::{DVector, UnitQuaternion, Vector3};
    use std::f64::consts::{FRAC_PI_2, FRAC_PI_4, PI};

    /// Build a minimal model with one body and one ball joint.
    fn build_ball_joint_model(stiffness: f64) -> Model {
        let mut model = Model::empty();
        model.nbody = 2;
        model.body_parent = vec![0, 0];
        model.body_rootid = vec![0, 1];
        model.body_jnt_adr = vec![0, 0];
        model.body_jnt_num = vec![0, 1];
        model.body_dof_adr = vec![0, 0];
        model.body_dof_num = vec![0, 3];
        model.body_geom_adr = vec![0, 0];
        model.body_geom_num = vec![0, 0];
        model.body_pos = vec![Vector3::zeros(); 2];
        model.body_quat = vec![UnitQuaternion::identity(); 2];
        model.body_ipos = vec![Vector3::zeros(); 2];
        model.body_iquat = vec![UnitQuaternion::identity(); 2];
        model.body_mass = vec![0.0, 1.0];
        model.body_inertia = vec![Vector3::zeros(), Vector3::new(0.1, 0.1, 0.1)];
        model.body_name = vec![Some("world".into()), Some("body".into())];
        model.body_subtreemass = vec![1.0, 1.0];

        model.njnt = 1;
        model.nq = 4; // ball joint: 4 qpos (quaternion)
        model.nv = 3; // ball joint: 3 DOFs
        model.jnt_type = vec![MjJointType::Ball];
        model.jnt_body = vec![1];
        model.jnt_qpos_adr = vec![0];
        model.jnt_dof_adr = vec![0];
        model.jnt_pos = vec![Vector3::zeros()];
        model.jnt_axis = vec![Vector3::z()];
        model.jnt_limited = vec![false];
        model.jnt_range = vec![(0.0, 0.0)];
        model.jnt_stiffness = vec![stiffness];
        model.jnt_springref = vec![0.0];
        model.jnt_damping = vec![0.0];
        model.jnt_armature = vec![0.0];
        model.jnt_solref = vec![[0.02, 1.0]];
        model.jnt_solimp = vec![[0.9, 0.95, 0.001, 0.5, 2.0]];
        model.jnt_name = vec![Some("ball".into())];
        model.jnt_margin = vec![0.0];

        // qpos_spring: identity quaternion [w, x, y, z] = [1, 0, 0, 0]
        model.qpos_spring = vec![1.0, 0.0, 0.0, 0.0];

        model.dof_body = vec![1, 1, 1];
        model.dof_jnt = vec![0, 0, 0];
        model.dof_parent = vec![None, Some(0), Some(1)];
        model.dof_armature = vec![0.0; 3];
        model.dof_damping = vec![0.0; 3];
        model.dof_frictionloss = vec![0.0; 3];

        model.qpos0 = DVector::from_vec(vec![1.0, 0.0, 0.0, 0.0]);
        model.gravity = Vector3::new(0.0, 0.0, -9.81);

        model.body_ancestor_joints = vec![vec![]; 2];
        model.body_ancestor_mask = vec![vec![]; 2];
        model.compute_ancestors();
        model.compute_implicit_params();
        model.compute_qld_csr_metadata();

        model
    }

    /// Build a minimal model with one body and one free joint.
    fn build_free_joint_model(stiffness: f64) -> Model {
        let mut model = Model::empty();
        model.nbody = 2;
        model.body_parent = vec![0, 0];
        model.body_rootid = vec![0, 1];
        model.body_jnt_adr = vec![0, 0];
        model.body_jnt_num = vec![0, 1];
        model.body_dof_adr = vec![0, 0];
        model.body_dof_num = vec![0, 6];
        model.body_geom_adr = vec![0, 0];
        model.body_geom_num = vec![0, 0];
        model.body_pos = vec![Vector3::zeros(); 2];
        model.body_quat = vec![UnitQuaternion::identity(); 2];
        model.body_ipos = vec![Vector3::zeros(); 2];
        model.body_iquat = vec![UnitQuaternion::identity(); 2];
        model.body_mass = vec![0.0, 1.0];
        model.body_inertia = vec![Vector3::zeros(), Vector3::new(0.1, 0.1, 0.1)];
        model.body_name = vec![Some("world".into()), Some("body".into())];
        model.body_subtreemass = vec![1.0, 1.0];

        model.njnt = 1;
        model.nq = 7; // free joint: 3 pos + 4 quat
        model.nv = 6; // free joint: 3 trans + 3 rot
        model.jnt_type = vec![MjJointType::Free];
        model.jnt_body = vec![1];
        model.jnt_qpos_adr = vec![0];
        model.jnt_dof_adr = vec![0];
        model.jnt_pos = vec![Vector3::zeros()];
        model.jnt_axis = vec![Vector3::z()];
        model.jnt_limited = vec![false];
        model.jnt_range = vec![(0.0, 0.0)];
        model.jnt_stiffness = vec![stiffness];
        model.jnt_springref = vec![0.0];
        model.jnt_damping = vec![0.0];
        model.jnt_armature = vec![0.0];
        model.jnt_solref = vec![[0.02, 1.0]];
        model.jnt_solimp = vec![[0.9, 0.95, 0.001, 0.5, 2.0]];
        model.jnt_name = vec![Some("free".into())];
        model.jnt_margin = vec![0.0];

        // qpos_spring: origin position [0,0,0] + identity quat [1,0,0,0]
        model.qpos_spring = vec![0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0];

        model.dof_body = vec![1; 6];
        model.dof_jnt = vec![0; 6];
        model.dof_parent = vec![None, Some(0), Some(1), Some(2), Some(3), Some(4)];
        model.dof_armature = vec![0.0; 6];
        model.dof_damping = vec![0.0; 6];
        model.dof_frictionloss = vec![0.0; 6];

        model.qpos0 = DVector::from_vec(vec![0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]);
        model.gravity = Vector3::new(0.0, 0.0, -9.81);

        model.body_ancestor_joints = vec![vec![]; 2];
        model.body_ancestor_mask = vec![vec![]; 2];
        model.compute_ancestors();
        model.compute_implicit_params();
        model.compute_qld_csr_metadata();

        model
    }

    /// Build a hinge joint model for energy/spring tests.
    fn build_hinge_model(stiffness: f64) -> Model {
        let mut model = Model::empty();
        model.nbody = 2;
        model.body_parent = vec![0, 0];
        model.body_rootid = vec![0, 1];
        model.body_jnt_adr = vec![0, 0];
        model.body_jnt_num = vec![0, 1];
        model.body_dof_adr = vec![0, 0];
        model.body_dof_num = vec![0, 1];
        model.body_geom_adr = vec![0, 0];
        model.body_geom_num = vec![0, 0];
        model.body_pos = vec![Vector3::zeros(); 2];
        model.body_quat = vec![UnitQuaternion::identity(); 2];
        model.body_ipos = vec![Vector3::zeros(); 2];
        model.body_iquat = vec![UnitQuaternion::identity(); 2];
        model.body_mass = vec![0.0, 1.0];
        model.body_inertia = vec![Vector3::zeros(), Vector3::new(0.1, 0.1, 0.1)];
        model.body_name = vec![Some("world".into()), Some("body".into())];
        model.body_subtreemass = vec![1.0, 1.0];

        model.njnt = 1;
        model.nq = 1;
        model.nv = 1;
        model.jnt_type = vec![MjJointType::Hinge];
        model.jnt_body = vec![1];
        model.jnt_qpos_adr = vec![0];
        model.jnt_dof_adr = vec![0];
        model.jnt_pos = vec![Vector3::zeros()];
        model.jnt_axis = vec![Vector3::y()];
        model.jnt_limited = vec![false];
        model.jnt_range = vec![(-PI, PI)];
        model.jnt_stiffness = vec![stiffness];
        model.jnt_springref = vec![0.0];
        model.jnt_damping = vec![0.0];
        model.jnt_armature = vec![0.0];
        model.jnt_solref = vec![[0.02, 1.0]];
        model.jnt_solimp = vec![[0.9, 0.95, 0.001, 0.5, 2.0]];
        model.jnt_name = vec![Some("hinge".into())];
        model.jnt_margin = vec![0.0];

        model.qpos_spring = vec![0.0];

        model.dof_body = vec![1];
        model.dof_jnt = vec![0];
        model.dof_parent = vec![None];
        model.dof_armature = vec![0.0];
        model.dof_damping = vec![0.0];
        model.dof_frictionloss = vec![0.0];

        model.qpos0 = DVector::zeros(1);
        model.gravity = Vector3::new(0.0, 0.0, -9.81);

        model.body_ancestor_joints = vec![vec![]; 2];
        model.body_ancestor_mask = vec![vec![]; 2];
        model.compute_ancestors();
        model.compute_implicit_params();
        model.compute_qld_csr_metadata();

        model
    }

    // ---- T1: Ball joint spring force — 90° rotation about X ----
    #[test]
    fn t1_ball_spring_force_90deg() {
        let model = build_ball_joint_model(2.0);
        let mut data = model.make_data();
        // Set qpos to 90° about X: [cos(45°), sin(45°), 0, 0]
        data.qpos[0] = FRAC_PI_4.cos(); // w
        data.qpos[1] = FRAC_PI_4.sin(); // x
        data.qpos[2] = 0.0; // y
        data.qpos[3] = 0.0; // z

        data.forward(&model).expect("forward failed");

        // subquat([cos45, sin45, 0, 0], [1, 0, 0, 0]) = [π/2, 0, 0]
        // force = -2.0 * [π/2, 0, 0] = [-π, 0, 0]
        let expected_x = -2.0 * FRAC_PI_2;
        assert!(
            (data.qfrc_spring[0] - expected_x).abs() < 1e-10,
            "qfrc_spring[0] = {}, expected {}",
            data.qfrc_spring[0],
            expected_x
        );
        assert!(data.qfrc_spring[1].abs() < 1e-10, "y should be ~0");
        assert!(data.qfrc_spring[2].abs() < 1e-10, "z should be ~0");
    }

    // ---- T2: Free joint spring force — translational + rotational ----
    #[test]
    fn t2_free_spring_force_trans_rot() {
        let model = build_free_joint_model(1.0);
        let mut data = model.make_data();
        // Position displaced [1, 0, 0], rotation 90° about Z
        data.qpos[0] = 1.0; // x
        data.qpos[1] = 0.0; // y
        data.qpos[2] = 0.0; // z
        data.qpos[3] = FRAC_PI_4.cos(); // qw
        data.qpos[4] = 0.0; // qx
        data.qpos[5] = 0.0; // qy
        data.qpos[6] = FRAC_PI_4.sin(); // qz

        data.forward(&model).expect("forward failed");

        // Translational: -1.0 * (1 - 0) = -1.0 for X, 0 for Y, Z
        assert!(
            (data.qfrc_spring[0] - (-1.0)).abs() < 1e-10,
            "trans X = {}, expected -1.0",
            data.qfrc_spring[0]
        );
        assert!(data.qfrc_spring[1].abs() < 1e-10, "trans Y should be ~0");
        assert!(data.qfrc_spring[2].abs() < 1e-10, "trans Z should be ~0");

        // Rotational: Z component ≈ -π/2, X/Y ≈ 0
        assert!(data.qfrc_spring[3].abs() < 1e-10, "rot X should be ~0");
        assert!(data.qfrc_spring[4].abs() < 1e-10, "rot Y should be ~0");
        let expected_z = -FRAC_PI_2;
        assert!(
            (data.qfrc_spring[5] - expected_z).abs() < 1e-10,
            "rot Z = {}, expected {}",
            data.qfrc_spring[5],
            expected_z
        );
    }

    // ---- T3: Ball joint spring energy — 90° rotation ----
    #[test]
    fn t3_ball_spring_energy_90deg() {
        let mut model = build_ball_joint_model(2.0);
        model.enableflags |= ENABLE_ENERGY;
        let mut data = model.make_data();
        data.qpos[0] = FRAC_PI_4.cos();
        data.qpos[1] = FRAC_PI_4.sin();
        data.qpos[2] = 0.0;
        data.qpos[3] = 0.0;

        data.forward(&model).expect("forward failed");
        let with_spring = data.energy_potential;

        // Zero stiffness reference (isolate spring contribution from gravity)
        let mut model_no_spring = build_ball_joint_model(0.0);
        model_no_spring.enableflags |= ENABLE_ENERGY;
        let mut data_no_spring = model_no_spring.make_data();
        data_no_spring.qpos[0] = FRAC_PI_4.cos();
        data_no_spring.qpos[1] = FRAC_PI_4.sin();
        data_no_spring.qpos[2] = 0.0;
        data_no_spring.qpos[3] = 0.0;
        data_no_spring.forward(&model_no_spring).expect("forward");

        let spring_energy = with_spring - data_no_spring.energy_potential;
        // E = 0.5 * 2.0 * (π/2)² ≈ 2.4674
        let expected = 0.5 * 2.0 * FRAC_PI_2 * FRAC_PI_2;
        assert!(
            (spring_energy - expected).abs() < 1e-4,
            "spring energy = {spring_energy}, expected {expected}",
        );
    }

    // ---- T4: Free joint spring energy — translational + rotational ----
    #[test]
    fn t4_free_spring_energy_trans_rot() {
        let mut model = build_free_joint_model(1.0);
        model.enableflags |= ENABLE_ENERGY;
        let mut data = model.make_data();
        data.qpos[0] = 1.0;
        data.qpos[1] = 0.0;
        data.qpos[2] = 0.0;
        data.qpos[3] = FRAC_PI_4.cos();
        data.qpos[4] = 0.0;
        data.qpos[5] = 0.0;
        data.qpos[6] = FRAC_PI_4.sin();

        data.forward(&model).expect("forward failed");

        let mut model_no_spring = build_free_joint_model(0.0);
        model_no_spring.enableflags |= ENABLE_ENERGY;
        let mut data_no_spring = model_no_spring.make_data();
        data_no_spring.qpos[0] = 1.0;
        data_no_spring.qpos[3] = FRAC_PI_4.cos();
        data_no_spring.qpos[6] = FRAC_PI_4.sin();
        data_no_spring.forward(&model_no_spring).expect("forward");

        let spring_energy = data.energy_potential - data_no_spring.energy_potential;
        // Trans: 0.5 * 1.0 * 1² = 0.5
        // Rot: 0.5 * 1.0 * (π/2)² ≈ 1.2337
        let expected = 0.5 + 0.5 * FRAC_PI_2 * FRAC_PI_2;
        assert!(
            (spring_energy - expected).abs() < 1e-4,
            "spring energy = {spring_energy}, expected {expected}",
        );
    }

    // ---- T5: Zero stiffness — no spring force or energy ----
    #[test]
    fn t5_zero_stiffness_no_force_or_energy() {
        let mut model = build_ball_joint_model(0.0);
        model.enableflags |= ENABLE_ENERGY;
        let mut data = model.make_data();
        data.qpos[0] = FRAC_PI_4.cos();
        data.qpos[1] = FRAC_PI_4.sin();
        data.qpos[2] = 0.0;
        data.qpos[3] = 0.0;

        data.forward(&model).expect("forward failed");

        assert!(data.qfrc_spring[0].abs() < 1e-15, "force X should be 0");
        assert!(data.qfrc_spring[1].abs() < 1e-15, "force Y should be 0");
        assert!(data.qfrc_spring[2].abs() < 1e-15, "force Z should be 0");

        // Verify no spring energy contribution (compare with reference)
        let mut model2 = build_ball_joint_model(0.0);
        model2.enableflags |= ENABLE_ENERGY;
        let mut data2 = model2.make_data();
        // identity qpos (no displacement)
        data2.forward(&model2).expect("forward");
        // Both should have identical energy (gravity only)
        let diff = (data.energy_potential - data2.energy_potential).abs();
        assert!(
            diff < 1e-10,
            "zero stiffness should have no spring energy, diff = {diff}",
        );
    }

    // ---- T6: DISABLE_SPRING suppresses all 4 joint types ----
    #[test]
    fn t6_disable_spring_suppresses_all() {
        // Ball joint with spring
        let mut model = build_ball_joint_model(2.0);
        model.disableflags |= DISABLE_SPRING;
        model.enableflags |= ENABLE_ENERGY;
        let mut data = model.make_data();
        data.qpos[0] = FRAC_PI_4.cos();
        data.qpos[1] = FRAC_PI_4.sin();
        data.qpos[2] = 0.0;
        data.qpos[3] = 0.0;
        data.forward(&model).expect("forward failed");

        assert!(
            data.qfrc_spring[0].abs() < 1e-15,
            "DISABLE_SPRING: force X should be 0"
        );
        assert!(
            data.qfrc_spring[1].abs() < 1e-15,
            "DISABLE_SPRING: force Y should be 0"
        );
        assert!(
            data.qfrc_spring[2].abs() < 1e-15,
            "DISABLE_SPRING: force Z should be 0"
        );

        // Verify energy matches gravity-only (no spring contribution)
        let mut model_ref = build_ball_joint_model(0.0);
        model_ref.disableflags |= DISABLE_SPRING;
        model_ref.enableflags |= ENABLE_ENERGY;
        let mut data_ref = model_ref.make_data();
        data_ref.qpos[0] = FRAC_PI_4.cos();
        data_ref.qpos[1] = FRAC_PI_4.sin();
        data_ref.qpos[2] = 0.0;
        data_ref.qpos[3] = 0.0;
        data_ref.forward(&model_ref).expect("forward");
        let diff = (data.energy_potential - data_ref.energy_potential).abs();
        assert!(
            diff < 1e-10,
            "DISABLE_SPRING: spring energy should be 0, diff = {diff}",
        );

        // Also test hinge
        let mut hinge_model = build_hinge_model(1.0);
        hinge_model.disableflags |= DISABLE_SPRING;
        hinge_model.enableflags |= ENABLE_ENERGY;
        let mut hinge_data = hinge_model.make_data();
        hinge_data.qpos[0] = 1.0;
        hinge_data.forward(&hinge_model).expect("forward");
        assert!(
            hinge_data.qfrc_spring[0].abs() < 1e-15,
            "DISABLE_SPRING hinge: force should be 0"
        );
    }

    // ---- T7: implicit_mode suppresses ball/free spring force ----
    #[test]
    fn t7_implicit_mode_suppresses_ball_spring() {
        let mut model = build_ball_joint_model(2.0);
        model.integrator = Integrator::ImplicitSpringDamper;
        model.compute_implicit_params();
        let mut data = model.make_data();
        data.qpos[0] = FRAC_PI_4.cos();
        data.qpos[1] = FRAC_PI_4.sin();
        data.qpos[2] = 0.0;
        data.qpos[3] = 0.0;

        data.forward(&model).expect("forward failed");

        assert!(
            data.qfrc_spring[0].abs() < 1e-15,
            "implicit mode: ball spring force X should be 0, got {}",
            data.qfrc_spring[0]
        );
        assert!(data.qfrc_spring[1].abs() < 1e-15);
        assert!(data.qfrc_spring[2].abs() < 1e-15);
    }

    // ---- T12: Sleep filter on spring energy ----
    // Tests mj_energy_pos directly to avoid complex forward() sleep initialization.
    #[test]
    fn t12_sleep_filter_spring_energy() {
        use crate::energy::mj_energy_pos;

        let mut model = build_hinge_model(2.0);
        model.enableflags |= ENABLE_ENERGY | ENABLE_SLEEP;
        let mut data = model.make_data();
        data.qpos[0] = 0.5; // displaced from springref=0
        // Set body COM position for gravity PE
        data.xipos[1] = Vector3::new(0.0, 0.0, 1.0);
        // Set body sleep state: body 0 (world) = Static, body 1 = Asleep
        data.body_sleep_state = vec![SleepState::Static, SleepState::Asleep];

        mj_energy_pos(&model, &mut data);
        let asleep_energy = data.energy_potential;
        // Gravity PE: -mass * g.dot(com) = -(1.0) * (0, 0, -9.81).dot((0,0,1)) = 9.81
        // Spring PE: sleeping → excluded
        let expected_gravity = 1.0 * 9.81 * 1.0;
        assert!(
            (asleep_energy - expected_gravity).abs() < 1e-4,
            "sleeping body: energy = {asleep_energy}, expected gravity-only = {expected_gravity}",
        );

        // Awake: should include spring
        data.body_sleep_state = vec![SleepState::Static, SleepState::Awake];
        mj_energy_pos(&model, &mut data);
        let awake_energy = data.energy_potential;
        let spring_contribution = 0.5 * 2.0 * 0.5 * 0.5; // 0.5 * k * d^2 = 0.25
        assert!(
            (awake_energy - (expected_gravity + spring_contribution)).abs() < 1e-4,
            "awake body: energy = {}, expected {}",
            awake_energy,
            expected_gravity + spring_contribution
        );
    }

    // ---- T13: Negative stiffness — force and energy computed ----
    #[test]
    fn t13_negative_stiffness() {
        let mut model = build_hinge_model(-1.0);
        model.enableflags |= ENABLE_ENERGY;
        model.gravity = Vector3::zeros(); // remove gravity for clean energy check
        let mut data = model.make_data();
        data.qpos[0] = 1.0; // displaced 1 rad from springref=0

        data.forward(&model).expect("forward failed");

        // force = -(-1.0) * (1.0 - 0.0) = 1.0
        assert!(
            (data.qfrc_spring[0] - 1.0).abs() < 1e-10,
            "negative stiffness force = {}, expected 1.0",
            data.qfrc_spring[0]
        );
        // energy = 0.5 * (-1.0) * 1.0² = -0.5
        assert!(
            (data.energy_potential - (-0.5)).abs() < 1e-10,
            "negative stiffness energy = {}, expected -0.5",
            data.energy_potential
        );
    }

    // ---- T15: Near-180° rotation edge case ----
    #[test]
    fn t15_near_180_rotation() {
        let model = build_ball_joint_model(1.0);
        let mut data = model.make_data();
        // 179° about Y
        let angle = 179.0_f64.to_radians();
        let half = angle / 2.0;
        data.qpos[0] = half.cos(); // w
        data.qpos[1] = 0.0; // x
        data.qpos[2] = half.sin(); // y
        data.qpos[3] = 0.0; // z

        data.forward(&model).expect("forward failed");

        // force magnitude should be ≈ angle (179° in rad)
        let force_mag = (data.qfrc_spring[0].powi(2)
            + data.qfrc_spring[1].powi(2)
            + data.qfrc_spring[2].powi(2))
        .sqrt();
        // Expected: |force| = k * θ = 1.0 * 179° in rad
        assert!(
            (force_mag - angle).abs() < 1e-6,
            "near-180° force magnitude = {force_mag}, expected {angle}",
        );
    }

    // ---- T16: Identity quaternion — zero force/energy ----
    #[test]
    fn t16_identity_quat_zero_force() {
        let mut model = build_ball_joint_model(5.0);
        model.enableflags |= ENABLE_ENERGY;
        model.gravity = Vector3::zeros();
        let mut data = model.make_data();
        // identity = reference → zero displacement
        data.qpos[0] = 1.0;
        data.qpos[1] = 0.0;
        data.qpos[2] = 0.0;
        data.qpos[3] = 0.0;

        data.forward(&model).expect("forward failed");

        assert!(
            data.qfrc_spring[0].abs() < 1e-15,
            "identity: force X = {}",
            data.qfrc_spring[0]
        );
        assert!(data.qfrc_spring[1].abs() < 1e-15);
        assert!(data.qfrc_spring[2].abs() < 1e-15);
        assert!(
            data.energy_potential.abs() < 1e-15,
            "identity: energy = {}",
            data.energy_potential
        );
    }

    // ---- T17: Multi-joint model — free + hinge chain ----
    #[test]
    fn t17_multi_joint_free_hinge() {
        let mut model = Model::empty();
        model.nbody = 3; // world + body1(free) + body2(hinge child of body1)
        model.body_parent = vec![0, 0, 1];
        model.body_rootid = vec![0, 1, 1];
        model.body_jnt_adr = vec![0, 0, 1];
        model.body_jnt_num = vec![0, 1, 1];
        model.body_dof_adr = vec![0, 0, 6];
        model.body_dof_num = vec![0, 6, 1];
        model.body_geom_adr = vec![0, 0, 0];
        model.body_geom_num = vec![0, 0, 0];
        model.body_pos = vec![Vector3::zeros(); 3];
        model.body_quat = vec![UnitQuaternion::identity(); 3];
        model.body_ipos = vec![Vector3::zeros(); 3];
        model.body_iquat = vec![UnitQuaternion::identity(); 3];
        model.body_mass = vec![0.0, 1.0, 0.5];
        model.body_inertia = vec![
            Vector3::zeros(),
            Vector3::new(0.1, 0.1, 0.1),
            Vector3::new(0.05, 0.05, 0.05),
        ];
        model.body_name = vec![
            Some("world".into()),
            Some("body1".into()),
            Some("body2".into()),
        ];
        model.body_subtreemass = vec![1.5, 1.5, 0.5];

        model.njnt = 2;
        model.nq = 8; // 7 (free) + 1 (hinge)
        model.nv = 7; // 6 (free) + 1 (hinge)
        model.jnt_type = vec![MjJointType::Free, MjJointType::Hinge];
        model.jnt_body = vec![1, 2];
        model.jnt_qpos_adr = vec![0, 7];
        model.jnt_dof_adr = vec![0, 6];
        model.jnt_pos = vec![Vector3::zeros(); 2];
        model.jnt_axis = vec![Vector3::z(), Vector3::y()];
        model.jnt_limited = vec![false, false];
        model.jnt_range = vec![(0.0, 0.0), (-PI, PI)];
        model.jnt_stiffness = vec![1.0, 2.0];
        model.jnt_springref = vec![0.0, 0.0];
        model.jnt_damping = vec![0.0, 0.0];
        model.jnt_armature = vec![0.0, 0.0];
        model.jnt_solref = vec![[0.02, 1.0]; 2];
        model.jnt_solimp = vec![[0.9, 0.95, 0.001, 0.5, 2.0]; 2];
        model.jnt_name = vec![Some("free".into()), Some("hinge".into())];
        model.jnt_margin = vec![0.0, 0.0];

        model.qpos_spring = vec![0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0];

        model.dof_body = vec![1, 1, 1, 1, 1, 1, 2];
        model.dof_jnt = vec![0, 0, 0, 0, 0, 0, 1];
        model.dof_parent = vec![None, Some(0), Some(1), Some(2), Some(3), Some(4), Some(5)];
        model.dof_armature = vec![0.0; 7];
        model.dof_damping = vec![0.0; 7];
        model.dof_frictionloss = vec![0.0; 7];

        model.qpos0 = DVector::from_vec(vec![0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0]);
        model.gravity = Vector3::zeros(); // no gravity for clean test

        model.body_ancestor_joints = vec![vec![]; 3];
        model.body_ancestor_mask = vec![vec![]; 3];
        model.compute_ancestors();
        model.compute_implicit_params();
        model.compute_qld_csr_metadata();

        let mut data = model.make_data();
        // Free joint: displace [1,0,0] in position
        data.qpos[0] = 1.0;
        data.qpos[3] = 1.0; // identity quat
        // Hinge: displace 0.5 rad
        data.qpos[7] = 0.5;

        data.forward(&model).expect("forward failed");

        // Free: trans force = -1.0 * (1-0) = -1.0 at dof 0
        assert!(
            (data.qfrc_spring[0] - (-1.0)).abs() < 1e-10,
            "free trans X = {}",
            data.qfrc_spring[0]
        );
        // Free: rot force should be 0 (identity quat, no rotation displacement)
        assert!(
            data.qfrc_spring[3].abs() < 1e-10,
            "free rot X = {}",
            data.qfrc_spring[3]
        );
        assert!(data.qfrc_spring[4].abs() < 1e-10);
        assert!(data.qfrc_spring[5].abs() < 1e-10);

        // Hinge: force = -2.0 * (0.5 - 0) = -1.0 at dof 6
        assert!(
            (data.qfrc_spring[6] - (-1.0)).abs() < 1e-10,
            "hinge force = {}, expected -1.0",
            data.qfrc_spring[6]
        );
    }
}
