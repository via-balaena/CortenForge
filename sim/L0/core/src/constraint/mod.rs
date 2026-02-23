//! Constraint system — assembly, dispatch, and solver orchestration.
//!
//! Implements the full constraint pipeline: compute unconstrained acceleration,
//! assemble constraint rows, dispatch to the configured solver (PGS, CG, Newton),
//! and map solver forces back to joint space. Corresponds to MuJoCo's
//! `engine_core_constraint.c` (assembly) and the solver dispatch in
//! `engine_forward.c`.

pub(crate) mod assembly;
pub(crate) mod equality;
pub(crate) mod impedance;
pub(crate) mod jacobian;
pub(crate) mod solver;

use nalgebra::{DMatrix, DVector, Vector3};

use crate::linalg::{cholesky_in_place, cholesky_solve_in_place, mj_solve_sparse};
use crate::types::{
    ConstraintType, Data, ENABLE_SLEEP, Integrator, MjJointType, Model, SolverType,
};

// Functions still in monolith — will be removed as later Phase 6 steps extract them.
use crate::constraint::assembly::assemble_unified_constraints;
use crate::constraint::assembly::tendon_deadband_displacement;
use crate::constraint::solver::compute_qfrc_constraint_from_efc;
use crate::constraint::solver::extract_qfrc_frictionloss;
use crate::constraint::solver::pgs::pgs_solve_unified;
use crate::mujoco_pipeline::NewtonResult; // monolith: removed in Phase 6 step 11
use crate::mujoco_pipeline::accumulate_tendon_kd; // monolith: removed in Phase 8a
use crate::mujoco_pipeline::cg_solve_unified; // monolith: removed in Phase 6 step 9
use crate::mujoco_pipeline::newton_solve; // monolith: removed in Phase 6 step 11
use crate::mujoco_pipeline::noslip_postprocess; // monolith: removed in Phase 6 step 12
use crate::mujoco_pipeline::populate_efc_island; // monolith: removed in Phase 6 step 5
use crate::mujoco_pipeline::tendon_all_dofs_sleeping; // monolith: removed in Phase 8a

/// Island-aware constraint dispatch.
///
/// Currently routes directly to [`mj_fwd_constraint`] — island decomposition
/// is not yet implemented; the unified solvers handle all constraint types
/// globally.
#[allow(clippy::cast_sign_loss, clippy::too_many_lines)]
pub(crate) fn mj_fwd_constraint_islands(model: &Model, data: &mut Data) {
    // §29: ALL solver types now route through unified constraint assembly + solver.
    // Island decomposition is no longer needed — the unified solvers handle all
    // constraint types (equality, friction, limits, contacts, flex) globally.
    mj_fwd_constraint(model, data);
}

/// Compute the unconstrained acceleration (`qacc_smooth`) and smooth forces.
///
/// - `qfrc_smooth = qfrc_applied + qfrc_actuator + qfrc_passive - qfrc_bias`
/// - `qacc_smooth = M⁻¹ · qfrc_smooth` (via sparse LDL solve)
///
/// Friction loss is no longer subtracted here — it is handled entirely by
/// solver constraint rows and extracted post-solve into `qfrc_frictionloss` (§29).
fn compute_qacc_smooth(model: &Model, data: &mut Data) -> (DVector<f64>, DVector<f64>) {
    let nv = model.nv;

    // Per-step meaninertia: trace(qM) / nv (more accurate than model-level constant)
    if nv > 0 {
        let mut trace = 0.0_f64;
        for i in 0..nv {
            trace += data.qM[(i, i)];
        }
        #[allow(clippy::cast_precision_loss)]
        let mi = trace / nv as f64;
        data.stat_meaninertia = if mi > 0.0 { mi } else { model.stat_meaninertia };
    } else {
        data.stat_meaninertia = model.stat_meaninertia;
    }

    // qfrc_smooth = qfrc_applied + qfrc_actuator + qfrc_passive - qfrc_bias
    // Note: friction loss is no longer in qfrc_passive or subtracted here.
    let mut qfrc_smooth = DVector::<f64>::zeros(nv);
    for k in 0..nv {
        qfrc_smooth[k] =
            data.qfrc_applied[k] + data.qfrc_actuator[k] + data.qfrc_passive[k] - data.qfrc_bias[k];
    }

    // qacc_smooth = M⁻¹ · qfrc_smooth (via sparse LDL solve)
    let mut qacc_smooth = qfrc_smooth.clone();
    let (rowadr, rownnz, colind) = model.qld_csr();
    mj_solve_sparse(
        rowadr,
        rownnz,
        colind,
        &data.qLD_data,
        &data.qLD_diag_inv,
        &mut qacc_smooth,
    );

    // Store on Data for solver access
    data.qacc_smooth = qacc_smooth.clone();
    data.qfrc_smooth = qfrc_smooth.clone();

    (qacc_smooth, qfrc_smooth)
}

/// Build the implicit-modified mass matrix for Newton solver (DT-35).
///
/// Returns `M + h·D_jnt + h²·K_jnt + h·D_ten + h²·K_ten` — joint diagonal
/// K/D plus tendon non-diagonal K/D. Called once per step in
/// `mj_fwd_constraint` when `ImplicitSpringDamper` is active. The returned
/// matrix replaces `data.qM` in all Newton computations.
///
/// Uses `accumulate_tendon_kd` for the tendon contribution,
/// guaranteeing identical mass matrix modification as the non-Newton path
/// in `mj_fwd_acceleration_implicit`.
#[must_use]
fn build_m_impl_for_newton(model: &Model, data: &Data) -> DMatrix<f64> {
    let h = model.timestep;
    let h2 = h * h;
    let nv = model.nv;
    let sleep_enabled = model.enableflags & ENABLE_SLEEP != 0;

    let mut m_impl = data.qM.clone();

    // Add diagonal joint K/D (matching mj_fwd_acceleration_implicit)
    let k = &model.implicit_stiffness;
    let d = &model.implicit_damping;
    for i in 0..nv {
        m_impl[(i, i)] += h * d[i] + h2 * k[i];
    }

    // Add non-diagonal tendon K/D (shared helper)
    accumulate_tendon_kd(
        &mut m_impl,
        model,
        &data.ten_J,
        &data.ten_length,
        &data.tree_awake,
        h,
        sleep_enabled,
    );

    m_impl
}

/// Compute implicit-corrected smooth forces for Newton solver (DT-35).
///
/// In ImplicitSpringDamper mode, qfrc_passive excludes spring/damper forces.
/// This function computes the RHS forces that, together with M_impl in the
/// Hessian, produce the correct unconstrained acceleration matching the
/// non-Newton implicit path.
///
/// From the equivalence derivation:
///   a = M_impl⁻¹ · (f_ext − D·v − K·(Δq + h·v))
///
/// So: qfrc_smooth_impl = qfrc_smooth_base
///     − D_jnt·v − D_ten·v           (damper forces)
///     − K_jnt·(Δq + h·v)            (spring forces + velocity correction)
///     − K_ten·(δ + h·V_ten)          (tendon spring + velocity correction)
#[must_use]
fn compute_qfrc_smooth_implicit(model: &Model, data: &Data) -> DVector<f64> {
    let h = model.timestep;
    let nv = model.nv;
    let sleep_enabled = model.enableflags & ENABLE_SLEEP != 0;

    // Start with existing qfrc_smooth (which has everything EXCEPT
    // implicit spring/damper forces)
    let mut qfrc = data.qfrc_smooth.clone();

    // Add joint spring forces: −K·(Δq + h·v)
    // where Δq = q − q_eq, so total = −K·(q − q_eq) − h·K·v
    for jnt_id in 0..model.njnt {
        let dof_adr = model.jnt_dof_adr[jnt_id];
        let k = model.implicit_stiffness[dof_adr];
        if k <= 0.0 {
            continue;
        }
        let q_eq = model.implicit_springref[dof_adr];
        match model.jnt_type[jnt_id] {
            MjJointType::Hinge | MjJointType::Slide => {
                let q = data.qpos[model.jnt_qpos_adr[jnt_id]];
                let v = data.qvel[dof_adr];
                qfrc[dof_adr] += -k * (q - q_eq) - h * k * v;
            }
            // Ball/Free: compute_implicit_params sets implicit_stiffness=0
            // for these types, so the `k <= 0.0` guard above catches them.
            _ => {
                debug_assert!(
                    k <= 0.0,
                    "Ball/Free joint {jnt_id} has implicit_stiffness={k} > 0; \
                     compute_implicit_params should set this to 0.0"
                );
            }
        }
    }

    // Add joint damper forces: −D·v
    for i in 0..nv {
        let d = model.implicit_damping[i];
        if d > 0.0 {
            qfrc[i] += -d * data.qvel[i];
        }
    }

    // Add tendon spring forces: −k·(δ + h·V_ten) projected via J^T
    // and tendon damper forces: −b · V projected via J^T
    for t in 0..model.ntendon {
        if sleep_enabled && tendon_all_dofs_sleeping(model, data, t) {
            continue;
        }
        let kt = model.tendon_stiffness[t];
        if kt > 0.0 {
            let displacement =
                tendon_deadband_displacement(data.ten_length[t], model.tendon_lengthspring[t]);
            // Only apply spring force + velocity correction when OUTSIDE
            // deadband. Inside (displacement == 0.0), no spring force exists,
            // and the velocity correction h·K·v must also be zero.
            if displacement != 0.0 {
                let velocity = data.ten_velocity[t]; // J · qvel
                let f = -kt * (displacement + h * velocity);
                let j = &data.ten_J[t];
                for dof in 0..nv {
                    if j[dof] != 0.0 {
                        qfrc[dof] += f * j[dof];
                    }
                }
            }
        }
        // Add tendon damper forces: −b · V projected via J^T
        let bt = model.tendon_damping[t];
        if bt > 0.0 {
            let j = &data.ten_J[t];
            let velocity = data.ten_velocity[t]; // J · qvel
            let f = -bt * velocity;
            for dof in 0..nv {
                if j[dof] != 0.0 {
                    qfrc[dof] += f * j[dof];
                }
            }
        }
    }

    qfrc
}

/// Unified constraint pipeline (§29).
///
/// Pipeline:
/// 1. Compute qacc_smooth (unconstrained acceleration)
/// 2. Assemble ALL constraints into efc_* arrays
/// 3. Dispatch to configured solver (Newton, CG, PGS)
/// 4. Map efc_force → qfrc_constraint via J^T
/// 5. Extract qfrc_frictionloss from efc_force
fn mj_fwd_constraint(model: &Model, data: &mut Data) {
    data.qfrc_constraint.fill(0.0);
    data.qfrc_frictionloss.fill(0.0);
    data.jnt_limit_frc.iter_mut().for_each(|f| *f = 0.0);
    data.ten_limit_frc.iter_mut().for_each(|f| *f = 0.0);
    data.newton_solved = false;

    // Step 1: Shared qacc_smooth computation
    let (qacc_smooth, _qfrc_smooth) = compute_qacc_smooth(model, data);

    let implicit_sd = model.integrator == Integrator::ImplicitSpringDamper;

    // DT-35: For ImplicitSpringDamper, precompute the implicit-modified
    // quantities that Newton needs. Also recompute qacc_smooth using M_impl
    // so the constraint assembly sees the correct unconstrained motion.
    let m_impl_owned: Option<DMatrix<f64>>;
    let qfrc_impl_owned: Option<DVector<f64>>;
    let qacc_smooth_impl: DVector<f64>;

    if implicit_sd {
        let m_impl = build_m_impl_for_newton(model, data);
        let qfrc_impl = compute_qfrc_smooth_implicit(model, data);

        // qacc_smooth_impl = M_impl⁻¹ · qfrc_smooth_impl
        let mut m_impl_factor = m_impl.clone();
        if cholesky_in_place(&mut m_impl_factor).is_err() {
            // M_impl should always be SPD; if Cholesky fails, fall back to
            // the base qacc_smooth (degrades gracefully).
            qacc_smooth_impl = qacc_smooth.clone();
        } else {
            let mut qa = qfrc_impl.clone();
            cholesky_solve_in_place(&m_impl_factor, &mut qa);
            qacc_smooth_impl = qa;
        }
        m_impl_owned = Some(m_impl);
        qfrc_impl_owned = Some(qfrc_impl);
    } else {
        qacc_smooth_impl = qacc_smooth.clone();
        m_impl_owned = None;
        qfrc_impl_owned = None;
    }

    // DT-35: Override data.qacc_smooth and data.qfrc_smooth with implicit
    // versions so Newton (and other consumers) see the correct values.
    if implicit_sd {
        data.qacc_smooth = qacc_smooth_impl.clone();
        if let Some(ref qfrc) = qfrc_impl_owned {
            data.qfrc_smooth = qfrc.clone();
        }
    }

    // Use implicit-corrected qacc_smooth for constraint assembly when
    // ImplicitSpringDamper is active, so efc_b sees the correct
    // unconstrained motion including spring/damper effects.
    let qacc_for_assembly = if implicit_sd {
        &qacc_smooth_impl
    } else {
        &qacc_smooth
    };

    // Step 2: Assemble ALL constraints (universal for all solver types)
    assemble_unified_constraints(model, data, qacc_for_assembly);
    let nefc = data.efc_type.len();

    // Step 2b: Populate efc_island from constraint rows and island data.
    // This must happen after assembly since mj_island runs before us with stale efc data.
    populate_efc_island(model, data);

    if nefc == 0 {
        data.qacc.copy_from(&qacc_smooth_impl);
        return;
    }

    // Store implicit quantities for Newton solver.
    // Clone into owned values so we don't hold an immutable borrow on `data`
    // while passing it mutably to `newton_solve`.
    let m_eff: DMatrix<f64> = m_impl_owned.unwrap_or_else(|| data.qM.clone());
    let qfrc_eff: DVector<f64> = qfrc_impl_owned.unwrap_or_else(|| data.qfrc_smooth.clone());

    // Step 3: Dispatch to solver
    match model.solver_type {
        SolverType::Newton => {
            let result = newton_solve(model, data, &m_eff, &qfrc_eff, implicit_sd);
            match result {
                NewtonResult::Converged => {
                    // Noslip post-processor (Phase C §15.10, §33)
                    if model.noslip_iterations > 0 {
                        noslip_postprocess(model, data);
                    }
                    data.newton_solved = true;
                }
                NewtonResult::CholeskyFailed | NewtonResult::MaxIterationsExceeded => {
                    // Fallback to PGS
                    pgs_solve_unified(model, data);
                    // §33: Noslip also runs after Newton's PGS fallback
                    if model.noslip_iterations > 0 {
                        noslip_postprocess(model, data);
                    }
                }
            }
        }
        SolverType::CG => {
            cg_solve_unified(model, data);
            // §33: Noslip post-processor for CG solver
            if model.noslip_iterations > 0 {
                noslip_postprocess(model, data);
            }
        }
        SolverType::PGS => {
            pgs_solve_unified(model, data);
            // §33: Noslip post-processor for PGS solver
            if model.noslip_iterations > 0 {
                noslip_postprocess(model, data);
            }
        }
    }

    // Step 4: Map efc_force → qfrc_constraint via J^T
    compute_qfrc_constraint_from_efc(model, data);

    // Step 5: Extract qfrc_frictionloss from efc_force
    extract_qfrc_frictionloss(data, model.nv);

    // Step 5b: Extract per-joint and per-tendon limit forces from efc_force
    for i in 0..nefc {
        match data.efc_type[i] {
            ConstraintType::LimitJoint => {
                data.jnt_limit_frc[data.efc_id[i]] = data.efc_force[i];
            }
            ConstraintType::LimitTendon => {
                data.ten_limit_frc[data.efc_id[i]] = data.efc_force[i];
            }
            _ => {}
        }
    }

    // Step 6: Zero sleeping DOFs (§16.26.5)
    //
    // Sleeping trees must have zero qacc, qfrc_constraint, and qfrc_frictionloss.
    // The solver may produce non-zero forces for constraints involving sleeping bodies
    // (e.g., resting contact with the ground plane). We zero them here to maintain
    // the sleeping invariant. This is simpler than filtering constraints during assembly.
    if model.enableflags & ENABLE_SLEEP != 0 {
        for tree in 0..model.ntree {
            if data.tree_asleep[tree] < 0 {
                continue; // Awake
            }
            let dof_start = model.tree_dof_adr[tree];
            let dof_end = dof_start + model.tree_dof_num[tree];
            for dof in dof_start..dof_end {
                data.qacc[dof] = 0.0;
                data.qfrc_constraint[dof] = 0.0;
                data.qfrc_frictionloss[dof] = 0.0;
            }
        }
    }
}

/// Compute the velocity of a point on a body.
pub(crate) fn compute_point_velocity(
    data: &Data,
    body_id: usize,
    point: Vector3<f64>,
) -> Vector3<f64> {
    if body_id == 0 {
        return Vector3::zeros(); // World is stationary
    }

    // Get body velocity (linear and angular)
    let cvel = &data.cvel[body_id];
    let v_linear = Vector3::new(cvel[3], cvel[4], cvel[5]);
    let omega = Vector3::new(cvel[0], cvel[1], cvel[2]);

    // Point velocity = v_body + omega × r
    let body_pos = data.xpos[body_id];
    let r = point - body_pos;

    v_linear + omega.cross(&r)
}
