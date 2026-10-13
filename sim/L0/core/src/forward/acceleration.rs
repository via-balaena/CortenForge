//! Forward acceleration computation (explicit and implicit paths).
//!
//! Dispatches to explicit Euler, implicit spring-damper, implicit-fast,
//! or full implicit acceleration solvers. Corresponds to the acceleration
//! portion of MuJoCo's `engine_forward.c`.

use crate::constraint::assembly::tendon_deadband_displacement;
use crate::dynamics::factor::{mj_factor_i, mj_solve_ld};
use crate::dynamics::rne::free_joint_velocity_product;
use crate::dynamics::spatial::{
    SpatialVector, spatial_cross_force, spatial_cross_motion, transport_motion_spatial,
};
use crate::integrate::implicit::{
    accumulate_tendon_kd, isd_damping, isd_stiffness, isd_tendon_stiffness,
    tendon_all_dofs_sleeping,
};
use crate::joint_visitor::{JointContext, JointVisitor, joint_motion_subspace};
use crate::linalg::{
    cholesky_in_place, cholesky_solve_in_place, lu_factor_sparse, lu_solve_sparse, mj_solve_sparse,
};
use crate::types::flags::disabled;
use crate::types::{
    ConstraintType, DISABLE_GRAVITY, DISABLE_SPRING, Data, ENABLE_SLEEP, EqualityType, Integrator,
    MjJointType, Model, StepError,
};
use nalgebra::DVector;

/// Dispatch to the correct forward acceleration solver.
///
/// # Errors
///
/// Returns `Err(StepError::CholeskyFailed)` if using implicit integration
/// and the modified mass matrix (M + h*D + h²*K) is not positive definite.
pub fn mj_fwd_acceleration(model: &Model, data: &mut Data) -> Result<(), StepError> {
    if model.nv == 0 {
        return Ok(());
    }

    match model.integrator {
        Integrator::ImplicitSpringDamper => mj_fwd_acceleration_implicit(model, data),
        Integrator::ImplicitFast | Integrator::Implicit => {
            // MuJoCo leaves the explicit acceleration in qacc (mj_fwdConstraint,
            // engine_forward.c:747-750) and solves M − h·D only in the integrator
            // (mj_implicitSkip, :1128). After a Newton solve qacc is already its result.
            if !data.newton_solved {
                mj_fwd_acceleration_explicit(model, data);
            }
            if matches!(model.integrator, Integrator::ImplicitFast) {
                mj_fwd_acceleration_implicitfast(model, data);
                Ok(())
            } else {
                mj_fwd_acceleration_implicit_full(model, data)
            }
        }
        Integrator::Euler | Integrator::RungeKutta4 => {
            // qacc is always computed explicitly for Euler/RK4.
            // MuJoCo's eulerdamp is applied as a post-velocity correction in
            // the Euler integration step (see integrate/mod.rs), not here.
            mj_fwd_acceleration_explicit(model, data);
            Ok(())
        }
    }
}

/// Explicit forward acceleration (semi-implicit Euler or RK4).
///
/// Computes: qacc = M⁻¹ * (qfrc_smooth + qfrc_constraint)
///
/// Uses `data.qfrc_smooth` (already computed by `compute_qacc_smooth()`) which
/// includes `qfrc_applied + qfrc_actuator + qfrc_passive - qfrc_bias` plus the
/// DT-21 xfrc_applied projection.
fn mj_fwd_acceleration_explicit(model: &Model, data: &mut Data) {
    let mut qfrc_total = data.qfrc_smooth.clone();
    qfrc_total += &data.qfrc_constraint;

    // Solve M * qacc = qfrc_total using sparse L^T D L factorization from mj_crba
    data.qacc.copy_from(&qfrc_total);
    let (rowadr, rownnz, colind) = model.qld_csr();
    mj_solve_sparse(
        rowadr,
        rownnz,
        colind,
        &data.qLD_data,
        &data.qLD_diag_inv,
        &mut data.qacc,
    );
}

/// Implicit forward acceleration for springs and dampers.
///
/// Solves:
/// ```text
/// (M + h*D + h²*K) * v_new = M*v_old + h*f_ext - h*K*(q - q_eq)
/// ```
///
/// This provides unconditional stability for stiff springs by treating
/// spring and damper forces implicitly in the velocity update.
///
/// # Implementation Notes
///
/// - Spring/damper parameters are diagonal (no coupling between DOFs)
/// - Friction loss remains explicit (velocity-sign-dependent, cannot linearize)
/// - The modified matrix M + h*D + h²*K is still SPD if M is SPD and D, K ≥ 0
///
/// # Errors
///
/// Returns `Err(StepError::CholeskyFailed)` if the modified mass matrix
/// is not positive definite. This can happen with:
/// - Negative stiffness or damping values
/// - Corrupted mass matrix
/// - Extreme numerical conditions
fn mj_fwd_acceleration_implicit(model: &Model, data: &mut Data) -> Result<(), StepError> {
    // Guard against zero timestep (would cause division by zero)
    debug_assert!(
        model.timestep > 0.0,
        "Timestep must be positive for implicit integration"
    );
    let h = model.timestep;
    let h2 = h * h;

    let q_eq = &model.implicit_springref;

    // Build external forces into scratch buffer (avoids allocation).
    // IMPORTANT: Must NOT use data.qfrc_smooth here — in ImplicitSpringDamper
    // mode, the Newton solver overwrites qfrc_smooth with the implicit-corrected
    // version (which includes spring/damper forces). This solver handles
    // springs/dampers implicitly via K*(q-q_eq) and D in the mass matrix, so
    // the force vector must exclude them (qfrc_passive has them zeroed in
    // implicit mode, so assembling from components is correct).
    // f_ext = qfrc_applied + qfrc_actuator + qfrc_passive + qfrc_constraint - qfrc_bias
    //       + DT-21 xfrc_applied projection
    data.scratch_force.copy_from(&data.qfrc_applied);
    data.scratch_force += &data.qfrc_actuator;
    data.scratch_force += &data.qfrc_passive;
    data.scratch_force += &data.qfrc_constraint;
    data.scratch_force -= &data.qfrc_bias;

    // DT-21: Project xfrc_applied (Cartesian body forces) into scratch_force.
    // This replicates the projection done in compute_qacc_smooth(), which is
    // needed because we can't use data.qfrc_smooth (overridden by Newton).
    // No sleep guard — MuJoCo projects ALL bodies unconditionally.
    for body_id in 1..model.nbody {
        let wrench = data.xfrc_applied[body_id];
        if wrench.is_zero() {
            continue;
        }
        let point = data.xipos[body_id];
        crate::jacobian::mj_apply_ft(
            model,
            &data.xpos,
            &data.xquat,
            &data.xaxis,
            &data.xanchor,
            &wrench.force,
            &wrench.torque,
            &point,
            body_id,
            &mut data.scratch_force,
        );
    }

    // Build modified mass matrix: M_impl = M + h*D_jnt + h²*K_jnt
    // Copy M into scratch, then modify diagonal from joint K/D
    data.scratch_m_impl.copy_from(&data.qM);
    for i in 0..model.nv {
        data.scratch_m_impl[(i, i)] += h * isd_damping(model, i) + h2 * isd_stiffness(model, i);
    }

    // DT-35: Non-diagonal tendon stiffness and damping (Step 0 helper).
    // Adds Σ_t (h²·k_active_t + h·b_t) · J_t^T · J_t to scratch_m_impl.
    // Spring K is deadband-aware: zero inside [lower, upper], k outside.
    // Damping D always applies. Sleep guard skips fully-sleeping tendons.
    let sleep_enabled = model.enableflags & ENABLE_SLEEP != 0;
    accumulate_tendon_kd(
        &mut data.scratch_m_impl,
        model,
        &data.ten_J,
        &data.ten_length,
        &data.tree_awake,
        h,
        sleep_enabled,
    );

    // Build RHS into scratch buffer: M*v_old + h*f_ext - h*K*(q - q_eq)
    // Start with M*v_old
    data.qM.mul_to(&data.qvel, &mut data.scratch_rhs);

    // Add h*f_ext for each DOF
    for i in 0..model.nv {
        data.scratch_rhs[i] += h * data.scratch_force[i];
    }

    // Subtract h*K*(q - q_eq) for joint spring displacement using visitor
    // (none with springs disabled: `isd_stiffness`).
    if !disabled(model, DISABLE_SPRING) {
        let mut spring_visitor = ImplicitSpringVisitor {
            k: &model.implicit_stiffness,
            q_eq,
            h,
            qpos: &data.qpos,
            rhs: &mut data.scratch_rhs,
        };
        model.visit_joints(&mut spring_visitor);
    }

    // DT-35: Tendon spring displacement contribution to implicit RHS
    // RHS[dof] -= h · Σ_t k_t · J_t[dof] · deadband_disp(L_t)
    // (sleep_enabled already computed in Step 2, same function scope)
    for t in 0..model.ntendon {
        // Must match the guard in accumulate_tendon_kd — if K is not in the
        // LHS, the corresponding spring displacement must not be in the RHS.
        if sleep_enabled && tendon_all_dofs_sleeping(model, data, t) {
            continue;
        }
        let kt = isd_tendon_stiffness(model, t);
        if kt == 0.0 {
            continue;
        }
        let displacement =
            tendon_deadband_displacement(data.ten_length[t], model.tendon_lengthspring[t]);
        // SAFETY: exact `== 0.0` comparison is correct here.
        // `tendon_deadband_displacement` returns literal `0.0` from the else
        // branch (no arithmetic). At boundary, `length - upper` is exactly
        // 0.0 when both operands are equal.
        if displacement == 0.0 {
            continue; // Inside deadband — no spring force
        }
        // Spring force in tendon space: F = k * (ref - L) = -k * displacement
        // Joint-space force: qfrc = J^T * F = -J^T * k * displacement
        // RHS += h * qfrc = -h * k * displacement * J^T
        let j = &data.ten_J[t];
        let scale = -h * kt * displacement;
        for dof in 0..model.nv {
            if j[dof] != 0.0 {
                data.scratch_rhs[dof] += scale * j[dof];
            }
        }
    }

    // Factorize M_impl in place (overwrites lower triangle with L where M_impl = L·L^T).
    // A negative stiffness or damping can make M_impl indefinite: refused.
    cholesky_in_place(&mut data.scratch_m_impl)?;
    data.scratch_v_new.copy_from(&data.scratch_rhs);
    cholesky_solve_in_place(&data.scratch_m_impl, &mut data.scratch_v_new);

    // qacc = (v_new - v_old) / h. qvel is not written here (`integrate` adds
    // h·qacc to it), so `forward()` leaves the state alone.
    for i in 0..model.nv {
        data.qacc[i] = (data.scratch_v_new[i] - data.qvel[i]) / h;
    }

    Ok(())
}

/// Visitor for computing spring displacement contribution to implicit RHS.
/// Computes: `rhs\[dof\] -= h * K\[dof\] * (q - q_eq)` for joints with springs.
struct ImplicitSpringVisitor<'a> {
    k: &'a DVector<f64>,
    q_eq: &'a DVector<f64>,
    h: f64,
    qpos: &'a DVector<f64>,
    rhs: &'a mut DVector<f64>,
}

impl JointVisitor for ImplicitSpringVisitor<'_> {
    #[inline]
    fn visit_hinge(&mut self, ctx: JointContext) {
        let q = self.qpos[ctx.qpos_adr];
        self.rhs[ctx.dof_adr] -= self.h * self.k[ctx.dof_adr] * (q - self.q_eq[ctx.dof_adr]);
    }

    #[inline]
    fn visit_slide(&mut self, ctx: JointContext) {
        let q = self.qpos[ctx.qpos_adr];
        self.rhs[ctx.dof_adr] -= self.h * self.k[ctx.dof_adr] * (q - self.q_eq[ctx.dof_adr]);
    }

    // Ball and Free joints have no springs (k = 0), so default no-ops are correct.
}

/// Implicit-fast forward acceleration: solves
/// `(M − h·D) · qacc_implicit = qfrc_smooth + qfrc_constraint`, D being
/// `∂(qfrc_smooth)/∂(qvel)` without the RNE's terms (`mjd_passive_vel`,
/// `mjd_actuator_vel`), as MuJoCo 3.5.0's `mj_implicitSkip` does
/// (`engine_forward.c:1180-1190`, `:1292-1294`, `:1302-1309`). `M − h·D` is
/// taken on the mass matrix's pattern (`Model::M_*`: its lower triangle, a
/// simple dof's row its diagonal only), which leaves out what D couples
/// beyond it, a tendon's damping across two branches among them (MuJoCo
/// documents it, `doc/computation/index.rst:544-546`). It goes into
/// `Data::qH`, factored as `L'·D·L` over the awake dofs by MuJoCo's
/// `mj_factorI`, which checks no pivot: an `M − h·D` that is not positive
/// definite steps as MuJoCo's does. A sleeping dof's `qacc_implicit` is 0.
fn mj_fwd_acceleration_implicitfast(model: &Model, data: &mut Data) {
    use crate::derivatives::{mjd_actuator_vel, mjd_passive_vel};

    data.qDeriv.fill(0.0);
    mjd_passive_vel(model, data);
    mjd_actuator_vel(model, data);

    // qH = M + qDeriv·(−h) on M's pattern: MuJoCo gathers qDeriv's lower
    // entries there (`mapD2M`), then adds M (`mju_addScl`).
    let neg_h = -model.timestep;
    for i in 0..model.nv {
        for adr in model.M_rowadr[i]..model.M_rowadr[i] + model.M_rownnz[i] {
            let j = model.M_colind[adr];
            data.qH[adr] = data.qM[(i, j)] + data.qDeriv[(i, j)] * neg_h;
        }
    }

    let index = awake_dofs(model, data).then(|| &data.dof_awake_ind[..data.nv_awake]);
    mj_factor_i(
        &mut data.qH,
        &mut data.qHDiagInv,
        &model.M_rownnz,
        &model.M_rowadr,
        &model.M_colind,
        index,
    );
    implicit_rhs(model, data);
    let index = awake_dofs(model, data).then(|| &data.dof_awake_ind[..data.nv_awake]);
    data.qacc_implicit.copy_from(&data.scratch_rhs);
    mj_solve_ld(
        data.qacc_implicit.as_mut_slice(),
        &data.qH,
        &data.qHDiagInv,
        &model.M_rownnz,
        &model.M_rowadr,
        &model.M_colind,
        index,
    );
}

/// Full implicit forward acceleration: as
/// [`mj_fwd_acceleration_implicitfast`] with the RNE's velocity derivative
/// (`mjd_rne_vel`) in D, on the pattern of every pair of dofs on one branch
/// (`Model::D_*`, both triangles; MuJoCo's `mj_implicitSkip`,
/// `engine_forward.c:1168-1178`, `:1288-1291`, `:1297-1301`): `M − h·D` goes
/// into `Data::qLU`, factored as `(U + I)·L` over the awake dofs by MuJoCo's
/// `mju_factorLUSparse`, without pivoting. A sleeping dof's
/// `qacc_implicit` is 0.
///
/// # Errors
///
/// [`StepError::LuSingular`] when a diagonal of the factor is smaller than
/// mjMINVAL in magnitude, where MuJoCo stops with an error.
fn mj_fwd_acceleration_implicit_full(model: &Model, data: &mut Data) -> Result<(), StepError> {
    use crate::derivatives::{mjd_actuator_vel, mjd_passive_vel, mjd_rne_vel};

    data.qDeriv.fill(0.0);
    mjd_passive_vel(model, data);
    mjd_actuator_vel(model, data);
    mjd_rne_vel(model, data);

    // qLU = M + qDeriv·(−h) on D's pattern: MuJoCo gathers M there, 0 where
    // M's pattern holds no entry (`mapM2D`), then adds qDeriv
    // (`mju_addToScl`).
    let neg_h = -model.timestep;
    for i in 0..model.nv {
        for adr in model.D_rowadr[i]..model.D_rowadr[i] + model.D_rownnz[i] {
            let j = model.D_colind[adr];
            let in_m = i == j || model.dof_simplenum[i.max(j)] == 0;
            let m = if in_m { data.qM[(i, j)] } else { 0.0 };
            data.qLU[adr] = m + data.qDeriv[(i, j)] * neg_h;
        }
    }

    let index = awake_dofs(model, data).then(|| &data.dof_awake_ind[..data.nv_awake]);
    lu_factor_sparse(
        &mut data.qLU,
        &model.D_rownnz,
        &model.D_rowadr,
        &model.D_colind,
        index,
    )?;
    implicit_rhs(model, data);
    let index = awake_dofs(model, data).then(|| &data.dof_awake_ind[..data.nv_awake]);
    data.qacc_implicit.fill(0.0);
    lu_solve_sparse(
        data.qacc_implicit.as_mut_slice(),
        &data.qLU,
        data.scratch_rhs.as_slice(),
        &model.D_rownnz,
        &model.D_rowadr,
        &model.D_diag,
        &model.D_colind,
        index,
    );
    Ok(())
}

/// Whether the implicit integrators work over the awake dofs only, as
/// MuJoCo's do: sleep enabled and a tree asleep.
fn awake_dofs(model: &Model, data: &Data) -> bool {
    model.enableflags & ENABLE_SLEEP != 0 && data.nv_awake < model.nv
}

/// `scratch_rhs = qfrc_smooth + qfrc_constraint` at the awake dofs, 0 at the
/// others (MuJoCo's `qfrc` in `mj_implicitSkip`).
fn implicit_rhs(model: &Model, data: &mut Data) {
    if awake_dofs(model, data) {
        data.scratch_rhs.fill(0.0);
        for &i in &data.dof_awake_ind[..data.nv_awake] {
            data.scratch_rhs[i] = data.qfrc_smooth[i] + data.qfrc_constraint[i];
        }
    } else {
        data.scratch_rhs.copy_from(&data.qfrc_smooth);
        data.scratch_rhs += &data.qfrc_constraint;
    }
}

/// Solves `(M − h·D)·x = b` in place, `b` given in `x`, with the factor the
/// implicit or implicitfast acceleration stage left in `data` (`qH` or
/// `qLU`), over every dof: for the analytic derivatives, which take their
/// columns with every tree awake.
pub fn solve_implicit(model: &Model, data: &Data, x: &mut DVector<f64>) {
    if model.integrator == Integrator::Implicit {
        let b = x.clone();
        lu_solve_sparse(
            x.as_mut_slice(),
            &data.qLU,
            b.as_slice(),
            &model.D_rownnz,
            &model.D_rowadr,
            &model.D_diag,
            &model.D_colind,
            None,
        );
    } else {
        mj_solve_ld(
            x.as_mut_slice(),
            &data.qH,
            &data.qHDiagInv,
            &model.M_rownnz,
            &model.M_rowadr,
            &model.M_colind,
            None,
        );
    }
}

/// One contact's force and torque in its frame, from its constraint rows:
/// MuJoCo's `mj_contactForce` (`engine_core_util.c:891-908`). A pyramidal
/// contact's edges decode as `mju_decodePyramid` (`engine_util_misc.c:1142-1159`):
/// the normal force is their sum, each other component its pair's difference
/// times its friction coefficient. An elliptic contact's rows are its
/// components.
fn contact_force(rows: &[f64], mu: &[f64; 5], dim: usize, pyramidal: bool) -> [f64; 6] {
    let mut f = [0.0; 6];
    if pyramidal && dim > 1 {
        for &p in &rows[..2 * (dim - 1)] {
            f[0] += p;
        }
        for i in 0..dim - 1 {
            f[i + 1] = (rows[2 * i] - rows[2 * i + 1]) * mu[i];
        }
    } else {
        f[..dim].copy_from_slice(&rows[..dim]);
    }
    f
}

/// Compute per-body accelerations and forces after forward dynamics (§51).
///
/// Populates `data.cacc`, `data.cfrc_int`, and `data.cfrc_ext`:
///
/// 1. **cfrc_ext**: External forces on each body — `xfrc_applied` plus contact
///    and constraint forces from the solver.
///
/// 2. **cacc forward pass** (root→leaf): Propagate accelerations through the
///    kinematic chain. `cacc[0] = [0,0,0, -gx,-gy,-gz]` (gravity pseudo-acceleration).
///    For each body: `cacc[b] = cacc[parent] + S * qacc + cvel×(S * qvel)` (joint
///    acceleration plus Coriolis term).
///
/// 3. **cfrc_int backward pass** (leaf→root):
///    `cfrc_int[b] = I*cacc[b] + v×*(I*v) - cfrc_ext[b]`, accumulated into parent.
///    Propagation includes world body (`cfrc_int[0]` = total gravity force).
///
/// Mirrors the RNE algorithm structure but uses actual `qacc` instead of bias-only.
///
/// # MuJoCo Equivalence
///
/// MuJoCo's `mj_rnePostConstraint` (`engine_core_smooth.c:2492-2683`).
///
/// # Convention Difference
///
/// Our `cacc[b]`, `cfrc_int[b]`, and `cvel[b]` are stored at `xpos[b]` (body
/// origin). MuJoCo stores them at `subtree_com[body_rootid[b]]`. The spatial
/// transform formulas are identical — only the shift vector differs. When
/// porting MuJoCo code that reads these fields, substitute `xpos[body_id]`
/// wherever MuJoCo uses `subtree_com + 3*rootid`.
#[allow(clippy::needless_range_loop)] // jnt_id indexes both joint_subspaces and model arrays
pub fn mj_body_accumulators(model: &Model, data: &mut Data) {
    // ===== Step 1: cfrc_ext = xfrc_applied + contact/constraint forces =====
    // As MuJoCo's `mj_rnePostConstraint` (`engine_core_smooth.c:2503-2517`):
    // its loop over applied wrenches starts at body 1, so one applied to the
    // world body reaches nothing; an applied force acts at the centre of mass,
    // and cfrc_ext holds it at the body's origin, its torque moved there.
    data.cfrc_ext[0] = SpatialVector::zeros();
    for b in 1..model.nbody {
        let (t, f) = (data.xfrc_applied[b].torque, data.xfrc_applied[b].force);
        let t = t + (data.xipos[b] - data.xpos[b]).cross(&f);
        data.cfrc_ext[b] = SpatialVector::new(t.x, t.y, t.z, f.x, f.y, f.z);
    }

    // Contact forces (`engine_core_smooth.c:2519-2560`): each contact's force
    // and torque in its own frame, as `mj_contactForce` decodes them
    // (`engine_core_util.c:891-908`), turned to the world and moved from the
    // contact point to the body's origin, `−` on body1 and `+` on body2. A
    // contact that involves a flex adds nothing: MuJoCo's loop skips it.
    {
        let nefc = data.efc_type.len();
        let mut row = 0;
        while row < nefc {
            let pyramidal = match data.efc_type[row] {
                ConstraintType::ContactPyramidal => true,
                ConstraintType::ContactFrictionless | ConstraintType::ContactElliptic => false,
                _ => {
                    row += 1;
                    continue;
                }
            };
            let nrows = data.efc_dim[row];
            let (ci, adr) = (data.efc_id[row], row);
            row += nrows;
            let contact = &data.contacts[ci];
            if contact.flex_vertex.is_some() || contact.flex_vertex2.is_some() {
                continue;
            }
            let lfrc = contact_force(
                &data.efc_force.as_slice()[adr..adr + nrows],
                &contact.mu,
                contact.dim,
                pyramidal,
            );
            // The contact frame's rows are the normal and the two tangents;
            // `mju_mulMatTVec3` turns each half to the world.
            let frame = [contact.normal, contact.frame[0], contact.frame[1]];
            let to_world = |v: &[f64]| {
                nalgebra::Vector3::from_fn(|k, _| {
                    frame[0][k] * v[0] + frame[1][k] * v[1] + frame[2][k] * v[2]
                })
            };
            let torque = to_world(&lfrc[3..6]);
            let force = to_world(&lfrc[0..3]);
            let (body1, body2) = contact.bodies(model);
            for (body, add) in [(body1, false), (body2, true)] {
                if body == 0 {
                    continue;
                }
                // `mju_transformSpatial` of a force: the torque less the lever
                // from the contact point to the new point, crossed with the force.
                let t = torque - (data.xpos[body] - contact.pos).cross(&force);
                for k in 0..3 {
                    if add {
                        data.cfrc_ext[body][k] += t[k];
                        data.cfrc_ext[body][3 + k] += force[k];
                    } else {
                        data.cfrc_ext[body][k] -= t[k];
                        data.cfrc_ext[body][3 + k] -= force[k];
                    }
                }
            }
        }
    }

    // Connect and weld forces (`engine_core_smooth.c:2562-2622`): the rows'
    // force, and a weld's torque, act at each body's anchor, `+` on body1 and
    // `−` on body2. MuJoCo's anchors: a connect's `eq_data[0..3]` on body1 and
    // `[3..6]` on body2, a weld's the other way round, as the rows' Jacobians.
    {
        let nefc = data.efc_type.len();
        let mut i = 0;
        while i < nefc {
            if data.efc_type[i] != ConstraintType::Equality {
                i += 1;
                continue;
            }
            let eq_id = data.efc_id[i];
            let weld = match model.eq_type[eq_id] {
                EqualityType::Connect => false,
                EqualityType::Weld => true,
                EqualityType::Joint | EqualityType::Distance | EqualityType::Tendon => {
                    i += 1;
                    continue;
                }
            };
            let f = nalgebra::Vector3::new(
                data.efc_force[i],
                data.efc_force[i + 1],
                data.efc_force[i + 2],
            );
            let t = if weld {
                nalgebra::Vector3::new(
                    data.efc_force[i + 3],
                    data.efc_force[i + 4],
                    data.efc_force[i + 5],
                )
            } else {
                nalgebra::Vector3::zeros()
            };
            let ed = &model.eq_data[eq_id];
            let lo = nalgebra::Vector3::new(ed[0], ed[1], ed[2]);
            let hi = nalgebra::Vector3::new(ed[3], ed[4], ed[5]);
            let (anchor1, anchor2) = if weld { (hi, lo) } else { (lo, hi) };
            for (body, anchor, sign) in [
                (model.eq_obj1id[eq_id], anchor1, 1.0_f64),
                (model.eq_obj2id[eq_id], anchor2, -1.0_f64),
            ] {
                if body == 0 {
                    continue;
                }
                let lever = data.xquat[body] * anchor;
                let torque = t + lever.cross(&f);
                for k in 0..3 {
                    data.cfrc_ext[body][k] += sign * torque[k];
                    data.cfrc_ext[body][3 + k] += sign * f[k];
                }
            }
            i += if weld { 6 } else { 3 };
        }
    }

    // ===== Step 2: cacc forward pass (root → leaf) =====
    // Root pseudo-acceleration: gravity acts as downward acceleration on all bodies.
    let grav = if model.disableflags & DISABLE_GRAVITY != 0 {
        SpatialVector::zeros()
    } else {
        // Spatial convention: [angular; linear]. Gravity pseudo-acceleration has
        // zero angular and negative gravity as linear acceleration.
        let mut sv = SpatialVector::zeros();
        sv[3] = -model.gravity[0];
        sv[4] = -model.gravity[1];
        sv[5] = -model.gravity[2];
        sv
    };
    data.cacc[0] = grav;

    // Pre-compute joint motion subspaces (same cache as RNE)
    let joint_subspaces: Vec<_> = (0..model.njnt)
        .map(|jnt_id| joint_motion_subspace(model, data, jnt_id))
        .collect();

    for body_id in 1..model.nbody {
        let parent_id = model.body_parent[body_id];

        // Start with parent's acceleration, transported from xpos[parent] to xpos[child].
        // Spatial motion transport: angular unchanged, linear += alpha × r.
        // Our joint_motion_subspace computes S with linear part at xpos[body], so
        // cacc is stored at xpos[body]. We must transport when propagating.
        let parent_cacc = data.cacc[parent_id];
        let alpha_p = nalgebra::Vector3::new(parent_cacc[0], parent_cacc[1], parent_cacc[2]);
        let a_p = nalgebra::Vector3::new(parent_cacc[3], parent_cacc[4], parent_cacc[5]);
        let r = data.xpos[body_id] - data.xpos[parent_id];
        let a_transported = a_p + alpha_p.cross(&r);
        let mut acc = SpatialVector::zeros();
        acc[0] = alpha_p.x;
        acc[1] = alpha_p.y;
        acc[2] = alpha_p.z;
        acc[3] = a_transported.x;
        acc[4] = a_transported.y;
        acc[5] = a_transported.z;

        // Add joint acceleration + Coriolis contribution
        let jnt_start = model.body_jnt_adr[body_id];
        let jnt_end = jnt_start + model.body_jnt_num[body_id];

        // The body's velocity before each joint, as `mj_rne`'s bias pass
        // (`dynamics/rne.rs`): the parent velocity transported to this body's
        // origin, then each earlier joint's S·qvel.
        let mut v_partial = transport_motion_spatial(data.cvel[parent_id], &r);

        for jnt_id in jnt_start..jnt_end {
            let dof_adr = model.jnt_dof_adr[jnt_id];
            let s = &joint_subspaces[jnt_id];
            let nv = model.jnt_type[jnt_id].nv();

            // S * qacc (joint acceleration contribution)
            for row in 0..6 {
                for d in 0..nv {
                    acc[row] += s[(row, d)] * data.qacc[dof_adr + d];
                }
            }

            // Coriolis acceleration c = v ×_m (S·qvel), v the velocity before this
            // joint (`v_partial`).
            let mut v_joint = SpatialVector::zeros();
            for d in 0..nv {
                for row in 0..6 {
                    v_joint[row] += s[(row, d)] * data.qvel[dof_adr + d];
                }
            }
            acc += spatial_cross_motion(v_partial, v_joint);
            v_partial += v_joint;
            // A free joint's term, as `mj_rne`'s bias pass.
            if model.jnt_type[jnt_id] == MjJointType::Free {
                let c = free_joint_velocity_product(data, body_id, dof_adr);
                acc[3] -= c.x;
                acc[4] -= c.y;
                acc[5] -= c.z;
            }
        }

        data.cacc[body_id] = acc;
    }

    // ===== Step 3: cfrc_int backward pass (leaf → root) =====
    // For each body: cfrc_int = I*cacc + v×*(I*v) - cfrc_ext
    for body_id in 1..model.nbody {
        let inertia = &data.cinert[body_id];
        let v = data.cvel[body_id];
        let acc = data.cacc[body_id];

        // I * cacc (inertial force from acceleration)
        let i_a = inertia * acc;

        // v ×* (I * v) (gyroscopic force, same as RNE backward pass)
        let i_v = inertia * v;
        let gyro = spatial_cross_force(v, i_v);

        // cfrc_int = I*cacc + v×*(I*v) - cfrc_ext
        data.cfrc_int[body_id] = i_a + gyro - data.cfrc_ext[body_id];
    }

    // §51 Fix C: Propagate internal forces from leaves to root,
    // including accumulation into world body (body 0).
    // MuJoCo propagates cfrc_int[0] = sum of all children.
    //
    // Spatial-force transport (Xᵀ) to the parent ORIGIN: `cfrc_int[b]` is a spatial
    // force at body `b`'s own origin, so the torque gains the moment lever
    // (xpos[child] − xpos[parent]) × f when moved to the parent. A plain sum drops
    // it and corrupts the accumulated wrench on ancestor / world bodies of a
    // multi-link chain — the same defect fixed in `mj_rne`'s backward pass.
    data.cfrc_int[0] = SpatialVector::zeros();
    for body_id in (1..model.nbody).rev() {
        let parent_id = model.body_parent[body_id];
        let child_force = data.cfrc_int[body_id];
        let r = data.xpos[body_id] - data.xpos[parent_id];
        let f_lin = nalgebra::Vector3::new(child_force[3], child_force[4], child_force[5]);
        let lever = r.cross(&f_lin);
        data.cfrc_int[parent_id][0] += child_force[0] + lever.x;
        data.cfrc_int[parent_id][1] += child_force[1] + lever.y;
        data.cfrc_int[parent_id][2] += child_force[2] + lever.z;
        data.cfrc_int[parent_id][3] += child_force[3];
        data.cfrc_int[parent_id][4] += child_force[4];
        data.cfrc_int[parent_id][5] += child_force[5];
    }

    data.flg_rnepost = true;
}
