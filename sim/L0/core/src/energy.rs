//! Energy queries — potential and kinetic energy computation.
//!
//! `mj_energy_pos` computes gravitational + spring potential energy (called after
//! FK). `mj_energy_vel` computes kinetic energy from the mass matrix or body
//! velocities (called after CRBA). `Data::total_energy()` returns the sum.

use crate::types::{DISABLE_GRAVITY, Data, MjJointType, Model};
use nalgebra::Vector3;

/// Compute potential energy (gravitational + spring).
///
/// Following `MuJoCo` semantics:
/// - Gravitational: `E_g` = -Σ `m_i` * g · `p_i` (negative because work done against gravity)
/// - Spring: `E_s` = 0.5 * Σ `k_i` * (`q_i` - `q0_i)²`
pub(crate) fn mj_energy_pos(model: &Model, data: &mut Data) {
    let mut potential = 0.0;

    // S4.2: Effective gravity — zero when DISABLE_GRAVITY is set.
    let grav = if model.disableflags & DISABLE_GRAVITY != 0 {
        Vector3::zeros()
    } else {
        model.gravity
    };

    // Gravitational potential energy
    // E_g = -Σ m_i * g · com_i
    // Using xipos (COM in world frame) for correct calculation
    for body_id in 1..model.nbody {
        let mass = model.body_mass[body_id];
        let com = data.xipos[body_id];
        // Potential energy: -m * g · h (negative of work done by gravity)
        // With g = (0, 0, -9.81), this becomes m * 9.81 * z
        potential -= mass * grav.dot(&com);
    }

    // Spring potential energy
    // E_s = 0.5 * k * (q - q0)²
    for jnt_id in 0..model.njnt {
        let stiffness = model.jnt_stiffness[jnt_id];
        if stiffness > 0.0 {
            let qpos_adr = model.jnt_qpos_adr[jnt_id];

            match model.jnt_type[jnt_id] {
                MjJointType::Hinge | MjJointType::Slide => {
                    let q = data.qpos[qpos_adr];
                    let q0 = model.qpos0.get(qpos_adr).copied().unwrap_or(0.0);
                    let displacement = q - q0;
                    potential += 0.5 * stiffness * displacement * displacement;
                }
                MjJointType::Ball | MjJointType::Free => {
                    // Ball/Free joint springs would use quaternion distance
                    // Not commonly used, skip for now
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
        // Fallback: compute from body velocities directly
        // E_k = 0.5 * Σ (m_i * v_i^T * v_i + ω_i^T * I_i * ω_i)
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
            let v = Vector3::new(
                data.cvel[body_id][3],
                data.cvel[body_id][4],
                data.cvel[body_id][5],
            );

            // Translational kinetic energy: 0.5 * m * |v|²
            energy += 0.5 * mass * v.norm_squared();

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
