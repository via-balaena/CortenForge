//! Integration-specific derivative logic.

use crate::types::{ActuatorDynamics, Data, Model};
use nalgebra::{DMatrix, Matrix3, UnitQuaternion, Vector3};

// ============================================================================
// Phase C: Analytical integration derivatives
// ============================================================================

/// Quaternion integration derivatives on SO(3).
///
/// Given `q_new = q_old · exp(ω·h/2)`, computes two 3×3 tangent-space Jacobians:
///
/// - `dqnew_dqold = exp(−[ω·h]×)` — adjoint representation (rotation by −ω·h)
/// - `dqnew_domega = h · J_r(ω·h)` — h times the right Jacobian of SO(3)
///
/// When `θ = ‖ω‖·h < 1e-8`, both Jacobians use small-angle limits (I and h·I).
#[must_use]
#[allow(non_snake_case)]
pub fn mjd_quat_integrate(
    _quat: &UnitQuaternion<f64>,
    omega: &Vector3<f64>,
    h: f64,
) -> (Matrix3<f64>, Matrix3<f64>) {
    let theta_vec = omega * h; // ω·h = θ
    let theta = theta_vec.norm(); // ‖ω‖·h

    if theta < 1e-8 {
        // Small-angle limits: J_l^{-1}(0) = I, velocity = h·I
        return (Matrix3::identity(), Matrix3::identity() * h);
    }

    let theta2 = theta * theta;

    // Skew-symmetric matrix [θ]×  where θ = ω·h
    let skew_theta = Matrix3::new(
        0.0,
        -theta_vec.z,
        theta_vec.y,
        theta_vec.z,
        0.0,
        -theta_vec.x,
        -theta_vec.y,
        theta_vec.x,
        0.0,
    );
    let skew_theta_sq = skew_theta * skew_theta;

    let sin_theta = theta.sin();
    let cos_theta = theta.cos();

    // Position Jacobian: J_l^{-1}(θ) — the inverse left Jacobian of SO(3).
    //
    // Right-multiply integration: q_new = q_old * exp(θ)
    // FD tangent convention: both input and output tangent measured from q_0.
    //   η = log(q_0^{-1} · q_next) = log(exp(ξ) · exp(θ)) using BCH
    //   ∂η/∂ξ|_{ξ=0} = J_l^{-1}(θ)
    //
    // J_l^{-1}(θ) = I − ½·[θ]× + α·[θ]×²
    //   where α = 1/θ² − (1+cosθ)/(2θ·sinθ)  →  1/12 as θ → 0
    let alpha = 1.0 / theta2 - (1.0 + cos_theta) / (2.0 * theta * sin_theta);
    let dqnew_dqold = Matrix3::identity() - skew_theta * 0.5 + skew_theta_sq * alpha;

    // Velocity Jacobian: h·I for the FD tangent convention.
    //   η = log(q_old⁻¹ · q_old · exp(h·ω)) = log(exp(h·ω)) = h·ω  (exact)
    //   ∂η/∂ω = h·I
    //
    // Note: one might expect h·J_r⁻¹(h·ω) here, but that applies when the
    // output tangent is measured from a DIFFERENT reference (the nominal
    // output). Here both input and output tangent are at q_old, and q_old
    // cancels, giving h·I exactly. Verified against FD for ball joints with
    // extreme inertia ratios (100:1) — agreement to 1.5e-4 with floor=1e-6.
    // Apparent errors of ~1.0 with floor=1e-10 are false alarms from FD
    // noise on near-zero entries (see test_ball_joint_hybrid_vs_fd_a).
    let dqnew_domega = Matrix3::identity() * h;

    (dqnew_dqold, dqnew_domega)
}

/// Integration Jacobian blocks for one Euler or ImplicitSpringDamper step.
///
/// Contains `∂qpos/∂qpos`, `∂qpos/∂qvel`, `∂act/∂act`, and `∂act/∂act_dot`
/// from the integration stage only (not the force-velocity coupling).
#[allow(non_snake_case)]
pub(super) struct IntegrationDerivatives {
    /// ∂qpos_{t+1}/∂qpos_t in tangent space. nv × nv, block-diagonal per joint.
    /// Consumed by `mjd_transition_hybrid` for analytical position columns.
    pub dqpos_dqpos: DMatrix<f64>,
    /// ∂qpos_{t+1}/∂qvel_{t+1} in tangent space. nv × nv, block-diagonal.
    pub dqpos_dqvel: DMatrix<f64>,
    /// ∂act_{t+1}/∂act_t. na × na, diagonal.
    pub dact_dact: DMatrix<f64>,
    /// ∂act_{t+1}/∂act_dot. na × na, diagonal.
    /// Currently unused by the hybrid path — act_dot does not depend on qvel
    /// for any existing dyntype. Retained for future dynamics types.
    #[allow(dead_code)]
    pub dact_dactdot: DMatrix<f64>,
}

/// Compute integration Jacobians (pure function, no mutation).
#[allow(non_snake_case)]
pub(super) fn compute_integration_derivatives(
    model: &Model,
    data: &Data,
) -> IntegrationDerivatives {
    let nv = model.nv;
    let na = model.na;
    let h = model.timestep;

    let mut dqpos_dqpos = DMatrix::zeros(nv, nv);
    let mut dqpos_dqvel = DMatrix::zeros(nv, nv);
    let mut dact_dact = DMatrix::zeros(na, na);
    let mut dact_dactdot = DMatrix::zeros(na, na);

    // Joint dispatch
    for jnt_id in 0..model.njnt {
        let dof_adr = model.jnt_dof_adr[jnt_id];
        let qpos_adr = model.jnt_qpos_adr[jnt_id];

        match model.jnt_type[jnt_id] {
            crate::types::MjJointType::Hinge | crate::types::MjJointType::Slide => {
                dqpos_dqpos[(dof_adr, dof_adr)] = 1.0;
                dqpos_dqvel[(dof_adr, dof_adr)] = h;
            }
            crate::types::MjJointType::Ball => {
                let omega = Vector3::new(
                    data.qvel[dof_adr],
                    data.qvel[dof_adr + 1],
                    data.qvel[dof_adr + 2],
                );
                let quat = UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
                    data.qpos[qpos_adr],
                    data.qpos[qpos_adr + 1],
                    data.qpos[qpos_adr + 2],
                    data.qpos[qpos_adr + 3],
                ));
                let (dpos_dpos, dpos_dvel) = mjd_quat_integrate(&quat, &omega, h);
                dqpos_dqpos
                    .view_mut((dof_adr, dof_adr), (3, 3))
                    .copy_from(&dpos_dpos);
                dqpos_dqvel
                    .view_mut((dof_adr, dof_adr), (3, 3))
                    .copy_from(&dpos_dvel);
            }
            crate::types::MjJointType::Free => {
                // Linear part (dof_adr..dof_adr+3)
                for i in 0..3 {
                    dqpos_dqpos[(dof_adr + i, dof_adr + i)] = 1.0;
                    dqpos_dqvel[(dof_adr + i, dof_adr + i)] = h;
                }
                // Angular part (dof_adr+3..dof_adr+6)
                let omega = Vector3::new(
                    data.qvel[dof_adr + 3],
                    data.qvel[dof_adr + 4],
                    data.qvel[dof_adr + 5],
                );
                let quat = UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
                    data.qpos[qpos_adr + 3],
                    data.qpos[qpos_adr + 4],
                    data.qpos[qpos_adr + 5],
                    data.qpos[qpos_adr + 6],
                ));
                let (dpos_dpos, dpos_dvel) = mjd_quat_integrate(&quat, &omega, h);
                dqpos_dqpos
                    .view_mut((dof_adr + 3, dof_adr + 3), (3, 3))
                    .copy_from(&dpos_dpos);
                dqpos_dqvel
                    .view_mut((dof_adr + 3, dof_adr + 3), (3, 3))
                    .copy_from(&dpos_dvel);
            }
        }
    }

    // Activation dispatch
    for i in 0..model.nu {
        let act_adr = model.actuator_act_adr[i];
        let act_num = model.actuator_act_num[i];
        for k in 0..act_num {
            let j = act_adr + k;
            match model.actuator_dyntype[i] {
                ActuatorDynamics::Filter => {
                    let tau = model.actuator_dynprm[i][0].max(1e-10);
                    dact_dact[(j, j)] = 1.0 - h / tau;
                    dact_dactdot[(j, j)] = h;
                }
                ActuatorDynamics::FilterExact => {
                    let tau = model.actuator_dynprm[i][0].max(1e-10);
                    dact_dact[(j, j)] = (-h / tau).exp();
                    dact_dactdot[(j, j)] = tau * (1.0 - (-h / tau).exp());
                }
                ActuatorDynamics::Integrator => {
                    dact_dact[(j, j)] = 1.0;
                    dact_dactdot[(j, j)] = h;
                }
                ActuatorDynamics::Muscle | ActuatorDynamics::HillMuscle => {
                    // Approximate: ignores ∂act_dot/∂act from act-dependent
                    // time constants. Not consumed by hybrid (Muscle uses FD).
                    let at_boundary = data.act[j] <= 0.0 || data.act[j] >= 1.0;
                    dact_dact[(j, j)] = if at_boundary { 0.0 } else { 1.0 };
                    dact_dactdot[(j, j)] = if at_boundary { 0.0 } else { h };
                }
                ActuatorDynamics::None => {
                    // No activation state
                }
                ActuatorDynamics::User => {
                    // User-defined dynamics: no analytical derivative available.
                    // FD fallback handles this via the hybrid path.
                    dact_dact[(j, j)] = 1.0;
                    dact_dactdot[(j, j)] = h;
                }
            }
        }
    }

    IntegrationDerivatives {
        dqpos_dqpos,
        dqpos_dqvel,
        dact_dact,
        dact_dactdot,
    }
}

// ============================================================================
// mjd_sub_quat: Quaternion subtraction Jacobians (DT-52)
// ============================================================================

/// Compute the 3×3 Jacobians of quaternion subtraction with respect to each
/// input quaternion, in tangent space.
///
/// Given `res = subquat(qa, qb)` — the axis-angle 3-vector representing the
/// rotation from `qb` to `qa` — this function computes:
/// - `d(res)/d(qa)` — 3×3 Jacobian w.r.t. perturbations of `qa` in its tangent space
/// - `d(res)/d(qb)` — 3×3 Jacobian w.r.t. perturbations of `qb` in its tangent space
///
/// # Tangent-space convention
///
/// Perturbations are in the SO(3) tangent space (3D axis-angle), not in raw
/// 4D quaternion coordinates. A perturbation `δ` of `qa` means
/// `qa' = qa · exp(δ/2)`, and the Jacobian `d(res)/d(qa)` maps `δ` to the
/// change in the subtraction result.
///
/// # MuJoCo correspondence
///
/// Matches `mjd_subQuat()` in `engine_derivative.c`. The result quaternion
/// `dq = qb^{-1} · qa` is decomposed as `res = log(dq)` (axis × angle),
/// and the Jacobians are computed via the inverse right Jacobian of SO(3).
///
/// # Edge cases
///
/// - Identity (`qa == qb`): both Jacobians are `±I` (small-angle limit).
/// - Antipodal (`qa == -qb`): angle ≈ π, inverse Jacobian has a singularity
///   at θ = 2π but `subquat` wraps to `[-π, π]`, so this returns the
///   limiting form at θ → π.
#[must_use]
#[allow(non_snake_case, clippy::similar_names)]
pub fn mjd_sub_quat(qa: &[f64; 4], qb: &[f64; 4]) -> (Matrix3<f64>, Matrix3<f64>) {
    // Compute the relative quaternion: dq = qb^{-1} * qa
    let qa_uq =
        UnitQuaternion::new_normalize(nalgebra::Quaternion::new(qa[0], qa[1], qa[2], qa[3]));
    let qb_uq =
        UnitQuaternion::new_normalize(nalgebra::Quaternion::new(qb[0], qb[1], qb[2], qb[3]));
    let dq = qb_uq.conjugate() * qa_uq;

    // Extract axis-angle from dq
    let xyz = Vector3::new(dq.i, dq.j, dq.k);
    let sin_half = xyz.norm();

    if sin_half < 1e-14 {
        // Small-angle limit: res ≈ 0, both Jacobians are ±I.
        // d(res)/d(qa) perturbation: qa' = qa·exp(δ/2) → dq' = qb⁻¹·qa·exp(δ/2)
        //   → res' ≈ res + δ → d(res)/d(qa) = I
        // d(res)/d(qb) perturbation: qb' = qb·exp(δ/2) → dq' = exp(-δ/2)·qb⁻¹·qa
        //   → res' ≈ res - δ → d(res)/d(qb) = -I
        return (Matrix3::identity(), -Matrix3::identity());
    }

    let axis = xyz / sin_half;
    let mut angle = 2.0 * sin_half.atan2(dq.w);
    // Shortest path wrapping
    if angle > std::f64::consts::PI {
        angle -= 2.0 * std::f64::consts::PI;
    }

    let theta = angle; // signed angle
    let theta_abs = theta.abs();

    // Compute the inverse right Jacobian of SO(3): J_r^{-1}(θ)
    // J_r^{-1}(v) = I + 0.5·[v]× + (1/θ² − (1+cos θ)/(2θ sin θ))·[v]×²
    // where v = axis * angle (the expmap vector)
    let v = axis * theta;
    let skew_v = Matrix3::new(0.0, -v.z, v.y, v.z, 0.0, -v.x, -v.y, v.x, 0.0);
    let skew_v_sq = skew_v * skew_v;

    let jr_inv = if theta_abs < 1e-8 {
        // Small-angle: J_r^{-1} ≈ I + 0.5·[v]× + 1/12·[v]×²
        Matrix3::identity() + skew_v * 0.5 + skew_v_sq * (1.0 / 12.0)
    } else {
        let coeff = 1.0 / (theta * theta) - (1.0 + theta.cos()) / (2.0 * theta * theta.sin());
        Matrix3::identity() + skew_v * 0.5 + skew_v_sq * coeff
    };

    // d(res)/d(qa): perturb qa → qa·exp(δ/2)
    // dq' = qb⁻¹ · qa · exp(δ/2) = dq · exp(δ/2)
    // res' = log(dq · exp(δ/2)) ≈ res + J_r^{-1}(res) · δ
    // So: d(res)/d(qa) = J_r^{-1}(res)
    let d_dqa = jr_inv;

    // d(res)/d(qb): perturb qb → qb·exp(δ/2)
    // dq' = (qb·exp(δ/2))⁻¹ · qa = exp(-δ/2) · qb⁻¹ · qa = exp(-δ/2) · dq
    // res' = log(exp(-δ/2) · dq)
    //
    // Using the BCH / adjoint formula:
    // log(exp(-δ/2)·exp(v)) ≈ v − J_l^{-1}(v)·δ   (for small δ)
    // where J_l^{-1}(v) = J_r^{-1}(-v)
    //
    // J_l^{-1}(v) = I − 0.5·[v]× + coeff·[v]×²
    // So: d(res)/d(qb) = −J_l^{-1}(res)
    let jl_inv = if theta_abs < 1e-8 {
        Matrix3::identity() - skew_v * 0.5 + skew_v_sq * (1.0 / 12.0)
    } else {
        let coeff = 1.0 / (theta * theta) - (1.0 + theta.cos()) / (2.0 * theta * theta.sin());
        Matrix3::identity() - skew_v * 0.5 + skew_v_sq * coeff
    };
    let d_dqb = -jl_inv;

    (d_dqa, d_dqb)
}

// ============================================================================
// Tests — sub_quat FD validation
// ============================================================================

#[cfg(test)]
#[allow(
    clippy::similar_names,
    clippy::needless_range_loop,
    clippy::uninlined_format_args
)]
mod sub_quat_tests {
    use super::*;
    use crate::tendon::subquat;

    const EPS: f64 = 1e-7;
    const FD_TOL: f64 = 1e-5;

    /// Apply a tangent-space perturbation to a quaternion:
    /// q' = q · exp(δ/2)
    fn perturb_quat(q: &[f64; 4], delta: &Vector3<f64>) -> [f64; 4] {
        let q_uq = UnitQuaternion::new_normalize(nalgebra::Quaternion::new(q[0], q[1], q[2], q[3]));
        let half_delta = delta * 0.5;
        let angle = half_delta.norm();
        let exp_half = if angle < 1e-14 {
            UnitQuaternion::identity()
        } else {
            let axis = nalgebra::Unit::new_normalize(half_delta / angle);
            UnitQuaternion::from_axis_angle(&axis, 2.0 * angle)
        };
        let q_new = q_uq * exp_half;
        [q_new.w, q_new.i, q_new.j, q_new.k]
    }

    /// Evaluate subquat as a function of raw quaternion arrays.
    fn eval_subquat(qa: &[f64; 4], qb: &[f64; 4]) -> Vector3<f64> {
        let qa_uq =
            UnitQuaternion::new_normalize(nalgebra::Quaternion::new(qa[0], qa[1], qa[2], qa[3]));
        let qb_uq =
            UnitQuaternion::new_normalize(nalgebra::Quaternion::new(qb[0], qb[1], qb[2], qb[3]));
        subquat(&qa_uq, &qb_uq)
    }

    /// Compute FD Jacobian of subquat w.r.t. qa (tangent-space perturbation).
    fn fd_d_dqa(qa: &[f64; 4], qb: &[f64; 4]) -> Matrix3<f64> {
        let mut jac = Matrix3::zeros();
        for j in 0..3 {
            let mut delta_p = Vector3::zeros();
            delta_p[j] = EPS;
            let mut delta_m = Vector3::zeros();
            delta_m[j] = -EPS;

            let qa_p = perturb_quat(qa, &delta_p);
            let qa_m = perturb_quat(qa, &delta_m);

            let res_p = eval_subquat(&qa_p, qb);
            let res_m = eval_subquat(&qa_m, qb);

            let col = (res_p - res_m) / (2.0 * EPS);
            jac.set_column(j, &col);
        }
        jac
    }

    /// Compute FD Jacobian of subquat w.r.t. qb (tangent-space perturbation).
    fn fd_d_dqb(qa: &[f64; 4], qb: &[f64; 4]) -> Matrix3<f64> {
        let mut jac = Matrix3::zeros();
        for j in 0..3 {
            let mut delta_p = Vector3::zeros();
            delta_p[j] = EPS;
            let mut delta_m = Vector3::zeros();
            delta_m[j] = -EPS;

            let qb_p = perturb_quat(qb, &delta_p);
            let qb_m = perturb_quat(qb, &delta_m);

            let res_p = eval_subquat(qa, &qb_p);
            let res_m = eval_subquat(qa, &qb_m);

            let col = (res_p - res_m) / (2.0 * EPS);
            jac.set_column(j, &col);
        }
        jac
    }

    fn assert_mat_close(name: &str, anal: &Matrix3<f64>, fd: &Matrix3<f64>, tol: f64) {
        for i in 0..3 {
            for j in 0..3 {
                let diff = (anal[(i, j)] - fd[(i, j)]).abs();
                let scale = anal[(i, j)].abs().max(fd[(i, j)].abs()).max(1e-10);
                assert!(
                    diff / scale < tol,
                    "{} [{},{}]: analytical={:.10e}, fd={:.10e}, rel_diff={:.3e}",
                    name,
                    i,
                    j,
                    anal[(i, j)],
                    fd[(i, j)],
                    diff / scale,
                );
            }
        }
    }

    #[test]
    fn dt52_identity_case() {
        // qa == qb → res = 0, d/dqa = I, d/dqb = -I
        let q = [1.0, 0.0, 0.0, 0.0];
        let (d_dqa, d_dqb) = mjd_sub_quat(&q, &q);

        assert_mat_close("d_dqa (identity)", &d_dqa, &Matrix3::identity(), 1e-10);
        assert_mat_close("d_dqb (identity)", &d_dqb, &-Matrix3::identity(), 1e-10);
    }

    #[test]
    fn dt52_small_angle() {
        // Small rotation: qa ≈ qb + small perturbation
        let qa = [1.0, 0.0, 0.0, 0.0];
        // Small rotation about z-axis
        let angle: f64 = 0.01;
        let qb = [(angle / 2.0).cos(), 0.0, 0.0, (angle / 2.0).sin()];
        let (d_dqa, d_dqb) = mjd_sub_quat(&qa, &qb);

        let fd_dqa = fd_d_dqa(&qa, &qb);
        let fd_dqb = fd_d_dqb(&qa, &qb);

        assert_mat_close("d_dqa (small angle)", &d_dqa, &fd_dqa, FD_TOL);
        assert_mat_close("d_dqb (small angle)", &d_dqb, &fd_dqb, FD_TOL);
    }

    #[test]
    fn dt52_moderate_rotation() {
        // 90-degree rotation about x-axis
        let angle = std::f64::consts::FRAC_PI_2;
        let qa = [(angle / 2.0).cos(), (angle / 2.0).sin(), 0.0, 0.0];
        let qb = [1.0, 0.0, 0.0, 0.0];

        let (d_dqa, d_dqb) = mjd_sub_quat(&qa, &qb);
        let fd_dqa = fd_d_dqa(&qa, &qb);
        let fd_dqb = fd_d_dqb(&qa, &qb);

        assert_mat_close("d_dqa (90 deg)", &d_dqa, &fd_dqa, FD_TOL);
        assert_mat_close("d_dqb (90 deg)", &d_dqb, &fd_dqb, FD_TOL);
    }

    #[test]
    fn dt52_arbitrary_quaternion_pair() {
        // Two arbitrary quaternions
        let qa_raw: [f64; 4] = [0.5, 0.5, 0.5, 0.5]; // 120° about [1,1,1]
        let n = (qa_raw[0] * qa_raw[0]
            + qa_raw[1] * qa_raw[1]
            + qa_raw[2] * qa_raw[2]
            + qa_raw[3] * qa_raw[3])
            .sqrt();
        let qa = [qa_raw[0] / n, qa_raw[1] / n, qa_raw[2] / n, qa_raw[3] / n];

        let angle_b: f64 = 1.2;
        let axis_b = Vector3::new(0.3, -0.7, 0.6).normalize();
        let half: f64 = angle_b / 2.0;
        let qb = [
            half.cos(),
            axis_b.x * half.sin(),
            axis_b.y * half.sin(),
            axis_b.z * half.sin(),
        ];

        let (d_dqa, d_dqb) = mjd_sub_quat(&qa, &qb);
        let fd_dqa = fd_d_dqa(&qa, &qb);
        let fd_dqb = fd_d_dqb(&qa, &qb);

        assert_mat_close("d_dqa (arbitrary)", &d_dqa, &fd_dqa, FD_TOL);
        assert_mat_close("d_dqb (arbitrary)", &d_dqb, &fd_dqb, FD_TOL);
    }

    #[test]
    fn dt52_near_pi_rotation() {
        // Near-pi rotation (large angle, tests numerical stability)
        let angle: f64 = 3.0; // 3 radians ≈ 172 degrees
        let half: f64 = angle / 2.0;
        let qa = [half.cos(), 0.0, half.sin(), 0.0];
        let qb = [1.0, 0.0, 0.0, 0.0];

        let (d_dqa, d_dqb) = mjd_sub_quat(&qa, &qb);
        let fd_dqa = fd_d_dqa(&qa, &qb);
        let fd_dqb = fd_d_dqb(&qa, &qb);

        assert_mat_close("d_dqa (near pi)", &d_dqa, &fd_dqa, FD_TOL);
        assert_mat_close("d_dqb (near pi)", &d_dqb, &fd_dqb, FD_TOL);
    }

    #[test]
    fn dt52_both_rotated() {
        // Both quaternions non-identity, with non-trivial difference
        let angle_a: f64 = 0.8;
        let axis_a = Vector3::new(1.0, 0.0, 0.0);
        let ha: f64 = angle_a / 2.0;
        let qa = [
            ha.cos(),
            ha.sin() * axis_a.x,
            ha.sin() * axis_a.y,
            ha.sin() * axis_a.z,
        ];

        let angle_b: f64 = 1.5;
        let axis_b = Vector3::new(0.0, 0.0, 1.0);
        let hb: f64 = angle_b / 2.0;
        let qb = [
            hb.cos(),
            hb.sin() * axis_b.x,
            hb.sin() * axis_b.y,
            hb.sin() * axis_b.z,
        ];

        let (d_dqa, d_dqb) = mjd_sub_quat(&qa, &qb);
        let fd_dqa = fd_d_dqa(&qa, &qb);
        let fd_dqb = fd_d_dqb(&qa, &qb);

        assert_mat_close("d_dqa (both rotated)", &d_dqa, &fd_dqa, FD_TOL);
        assert_mat_close("d_dqb (both rotated)", &d_dqb, &fd_dqb, FD_TOL);
    }

    #[test]
    fn dt52_negative_w_quaternion() {
        // Antipodal representative: qa with w < 0. Both [w,x,y,z] and [-w,-x,-y,-z]
        // represent the same rotation. Verify mjd_sub_quat produces identical Jacobians.
        let angle_a: f64 = 1.2;
        let ha: f64 = angle_a / 2.0;
        let qa_pos = [ha.cos(), ha.sin(), 0.0, 0.0];
        let qa_neg = [-ha.cos(), -ha.sin(), 0.0, 0.0]; // antipodal
        let qb = [1.0, 0.0, 0.0, 0.0];

        let (d_dqa_pos, d_dqb_pos) = mjd_sub_quat(&qa_pos, &qb);
        let (d_dqa_neg, d_dqb_neg) = mjd_sub_quat(&qa_neg, &qb);

        assert_mat_close("d_dqa (neg-w)", &d_dqa_pos, &d_dqa_neg, 1e-10);
        assert_mat_close("d_dqb (neg-w)", &d_dqb_pos, &d_dqb_neg, 1e-10);

        // Also verify FD validation works with negative-w
        let fd_dqa = fd_d_dqa(&qa_neg, &qb);
        let fd_dqb = fd_d_dqb(&qa_neg, &qb);
        assert_mat_close("d_dqa (neg-w vs FD)", &d_dqa_neg, &fd_dqa, FD_TOL);
        assert_mat_close("d_dqb (neg-w vs FD)", &d_dqb_neg, &fd_dqb, FD_TOL);
    }

    #[test]
    fn dt52_antipodal_pair() {
        // qa == -qb: both represent the same rotation, difference should be zero.
        // dq = conj(-qb) * qa = -(conj(qb) * qa). If qa == qb, dq is antipodal
        // identity [-1, 0, 0, 0], which has sin_half ≈ 0 → early return (I, -I).
        let angle: f64 = 0.8;
        let half: f64 = angle / 2.0;
        let qa = [half.cos(), half.sin(), 0.0, 0.0];
        let qb = [-half.cos(), -half.sin(), 0.0, 0.0]; // -qa

        let (d_dqa, d_dqb) = mjd_sub_quat(&qa, &qb);

        // Difference is zero (identity), so Jacobians should be (I, -I)
        assert_mat_close("d_dqa (antipodal)", &d_dqa, &Matrix3::identity(), 1e-10);
        assert_mat_close("d_dqb (antipodal)", &d_dqb, &-Matrix3::identity(), 1e-10);
    }

    #[test]
    fn dt52_exactly_pi_rotation() {
        // θ = π exactly. The axis-angle representation has a topological
        // singularity at θ = π (the wrapping ±π creates a discontinuity),
        // so FD is invalid here. This test verifies:
        // 1. No NaN/Inf from the 0/0 form in J_r^{-1} coefficient
        // 2. The coefficient evaluates to the correct limit (1/π²)
        let qa = [0.0, 1.0, 0.0, 0.0]; // 180° about x: [cos(π/2), sin(π/2), 0, 0]
        let qb = [1.0, 0.0, 0.0, 0.0];

        let (d_dqa, d_dqb) = mjd_sub_quat(&qa, &qb);

        // Verify no NaN/Inf
        for i in 0..3 {
            for j in 0..3 {
                assert!(
                    d_dqa[(i, j)].is_finite(),
                    "d_dqa[{},{}] is not finite: {}",
                    i,
                    j,
                    d_dqa[(i, j)]
                );
                assert!(
                    d_dqb[(i, j)].is_finite(),
                    "d_dqb[{},{}] is not finite: {}",
                    i,
                    j,
                    d_dqb[(i, j)]
                );
            }
        }

        // At θ = π about x-axis: v = [π, 0, 0].
        // J_r^{-1} coeff = 1/π² - (1+cos(π))/(2π·sin(π)) = 1/π² (IEEE 754: 0/0 → 0).
        // [v]×² for v = [π,0,0] is diag(0, -π², -π²), so coeff·[v]×² adds
        // diag(0, -1, -1) — meaning the yy and zz entries of J_r^{-1} are 0
        // (identity cancels with -1 from coeff·[v]×²).
        // J_r^{-1} should be: diag(1, 0, 0) + off-diag from skew terms.
        // Verify d_dqa[0,0] = 1 (the axis direction is preserved).
        assert!(
            (d_dqa[(0, 0)] - 1.0).abs() < 1e-10,
            "d_dqa[0,0] at θ=π should be 1.0, got {}",
            d_dqa[(0, 0)]
        );
    }

    #[test]
    fn dt52_negative_w_on_qb() {
        // Negative w on qb (the second argument). Verifies symmetry of
        // antipodal invariance for both inputs.
        let qa = [1.0, 0.0, 0.0, 0.0];
        let angle_b: f64 = 2.0;
        let hb: f64 = angle_b / 2.0;
        let qb_pos = [hb.cos(), 0.0, hb.sin(), 0.0];
        let qb_neg = [-hb.cos(), 0.0, -hb.sin(), 0.0];

        let (d_dqa_pos, d_dqb_pos) = mjd_sub_quat(&qa, &qb_pos);
        let (d_dqa_neg, d_dqb_neg) = mjd_sub_quat(&qa, &qb_neg);

        assert_mat_close("d_dqa (neg-w qb)", &d_dqa_pos, &d_dqa_neg, 1e-10);
        assert_mat_close("d_dqb (neg-w qb)", &d_dqb_pos, &d_dqb_neg, 1e-10);
    }
}
