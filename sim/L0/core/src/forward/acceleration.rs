//! Forward acceleration computation (explicit and implicit paths).
//!
//! Dispatches to explicit Euler, implicit spring-damper, implicit-fast,
//! or full implicit acceleration solvers. Corresponds to the acceleration
//! portion of MuJoCo's `engine_forward.c`.

use crate::constraint::assembly::tendon_deadband_displacement;
use crate::constraint::solver::decode_pyramid_into;
use crate::dynamics::factor::{mj_factor_i, mj_solve_ld, mj_solve_m};
use crate::dynamics::spatial::{
    SpatialVector, mju_mul_dof_vec, mju_mul_inert_vec, mju_transform_spatial, spatial_cross_force,
};
use crate::integrate::implicit::{
    accumulate_tendon_kd, isd_damping, isd_stiffness, isd_tendon_j, isd_tendon_stiffness,
};
use crate::joint_visitor::{JointContext, JointVisitor};
use crate::linalg::{
    cholesky_in_place, cholesky_solve_in_place, lu_factor_sparse, lu_solve_sparse,
};
use crate::types::flags::disabled;
use crate::types::{
    ConstraintType, DISABLE_GRAVITY, DISABLE_SPRING, Data, ENABLE_SLEEP, EqualityType, Integrator,
    Model, StepError,
};
use nalgebra::DVector;

/// Dispatch to the correct forward acceleration solver.
///
/// # Errors
///
/// `Err(StepError::CholeskyFailed)` under the implicit spring-damper
/// integrator when its modified mass matrix (M + h*D + h²*K) is not positive
/// definite.
pub fn mj_fwd_acceleration(model: &Model, data: &mut Data) -> Result<(), StepError> {
    if model.nv == 0 {
        return Ok(());
    }

    match model.integrator {
        Integrator::ImplicitSpringDamper => mj_fwd_acceleration_implicit(model, data),
        Integrator::ImplicitFast | Integrator::Implicit => {
            // MuJoCo leaves the explicit acceleration in qacc (mj_fwdConstraint,
            // engine_forward.c:747-750) and factors and solves M − h·D only in
            // the integrator (mj_implicitSkip, :1128; `mj_implicit_factor`).
            // After a constraint solve qacc is already the solver's
            // (`Data::solver_qacc`); the forward pass calls this only when not.
            if !data.solver_qacc {
                mj_fwd_acceleration_explicit(model, data);
            }
            Ok(())
        }
        Integrator::Euler | Integrator::RungeKutta4 => {
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
/// Uses `data.qfrc_smooth` (already computed by `compute_qacc_smooth()`):
/// `((qfrc_passive - qfrc_bias) + qfrc_applied) + qfrc_actuator`, then the
/// DT-21 xfrc_applied projection.
fn mj_fwd_acceleration_explicit(model: &Model, data: &mut Data) {
    let mut qfrc_total = data.qfrc_smooth.clone();
    qfrc_total += &data.qfrc_constraint;

    // Solve M * qacc = qfrc_total with the factor mj_crba left
    mj_solve_m(model, data, qfrc_total.as_mut_slice());
    data.qacc = qfrc_total;
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
        let mut qfrc = std::mem::take(&mut data.scratch_force);
        crate::jacobian::mj_apply_ft(
            model,
            data,
            &wrench.force,
            &wrench.torque,
            &point,
            body_id,
            &mut qfrc,
        );
        data.scratch_force = qfrc;
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
    // Damping D always applies. Each row at the awake dofs only (sleep).
    let sleep_filter = crate::island::constraint_sleep_filter(model, data);
    accumulate_tendon_kd(
        &mut data.scratch_m_impl,
        model,
        &data.ten_J,
        &data.ten_length,
        &data.tree_awake,
        h,
        sleep_filter,
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
    // Each row at the awake dofs only, as in accumulate_tendon_kd: the LHS
    // and the RHS take the same entries.
    for t in 0..model.ntendon {
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
        let scale = -h * kt * displacement;
        for dof in 0..model.nv {
            let j = isd_tendon_j(model, &data.tree_awake, sleep_filter, &data.ten_J[t], dof);
            if j != 0.0 {
                data.scratch_rhs[dof] += scale * j;
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

/// The implicit integrators' factor of `M − h·D`: the factorization of
/// MuJoCo 3.5.0's `mj_implicitSkip` (`engine_forward.c:1166-1196`,
/// `:1288-1295`), which runs in the integrator. `forward` leaves it, as
/// `mj_forward` does.
///
/// Under implicitfast D is `∂(qfrc_smooth)/∂(qvel)` without the RNE's terms
/// (`mjd_passive_vel`, `mjd_actuator_vel`), and `M − h·D` is taken on the
/// mass matrix's pattern (`Model::M_*`: its lower triangle, a simple dof's
/// row its diagonal only), which leaves out what D couples beyond it, a
/// tendon's damping across two branches among them (MuJoCo documents it,
/// `doc/computation/index.rst:544-546`). It goes into `Data::qH`, factored
/// as `L'·D·L` by MuJoCo's `mj_factorI`, which checks no pivot: an `M − h·D`
/// that is not positive definite steps as MuJoCo's does. Under implicit D
/// takes the RNE's velocity derivative (`mjd_rne_vel`) too, on the pattern
/// of every pair of dofs on one branch (`Model::D_*`, both triangles), into
/// `Data::qLU`, factored as `(U + I)·L` by `mju_factorLUSparse`, without
/// pivoting. Both over the awake dofs; `Data::qDeriv` holds D after it.
///
/// # Errors
///
/// [`StepError::LuSingular`] under implicit when a diagonal of the factor is
/// smaller than mjMINVAL in magnitude, where MuJoCo stops with an error.
pub fn mj_implicit_factor(model: &Model, data: &mut Data) -> Result<(), StepError> {
    crate::derivatives::mjd_smooth_vel_bias(model, data, model.integrator == Integrator::Implicit);
    match model.integrator {
        Integrator::ImplicitFast => implicit_fast_matrix(model, data),
        Integrator::Implicit => implicit_matrix(model, data),
        _ => {}
    }
    let index = awake_dofs(model, data).then(|| &data.dof_awake_ind[..data.nv_awake]);
    match model.integrator {
        Integrator::ImplicitFast => {
            mj_factor_i(
                &mut data.qH,
                &mut data.qHDiagInv,
                &model.M_rownnz,
                &model.M_rowadr,
                &model.M_colind,
                index,
            );
            Ok(())
        }
        Integrator::Implicit => lu_factor_sparse(
            &mut data.qLU,
            &model.D_rownnz,
            &model.D_rowadr,
            &model.D_colind,
            index,
        ),
        _ => Ok(()),
    }
}

/// `qH = M + qDeriv·(−h)` on M's pattern, implicitfast's matrix: MuJoCo
/// gathers qDeriv's lower entries there (`mapD2M`), then adds M
/// (`mju_addScl`, `engine_forward.c:1180-1184`, `engine_inverse.c:139-142`).
pub fn implicit_fast_matrix(model: &Model, data: &mut Data) {
    let neg_h = -model.timestep;
    for i in 0..model.nv {
        for adr in model.M_rowadr[i]..model.M_rowadr[i] + model.M_rownnz[i] {
            let j = model.M_colind[adr];
            data.qH[adr] = data.qM[(i, j)] + data.qDeriv[(i, j)] * neg_h;
        }
    }
}

/// `qLU = M + qDeriv·(−h)` on D's pattern, implicit's matrix: MuJoCo
/// gathers M there, 0 where M's pattern holds no entry (`mapM2D`), then
/// adds qDeriv (`mju_addToScl`, `engine_forward.c:1173-1177`,
/// `engine_inverse.c:120-123`).
pub fn implicit_matrix(model: &Model, data: &mut Data) {
    let neg_h = -model.timestep;
    for i in 0..model.nv {
        for adr in model.D_rowadr[i]..model.D_rowadr[i] + model.D_rownnz[i] {
            let j = model.D_colind[adr];
            let in_m = i == j || model.dof_simplenum[i.max(j)] == 0;
            let m = if in_m { data.qM[(i, j)] } else { 0.0 };
            data.qLU[adr] = m + data.qDeriv[(i, j)] * neg_h;
        }
    }
}

/// `qacc_implicit = (M − h·D)⁻¹·(qfrc_smooth + qfrc_constraint)` with the
/// factor in `data` ([`mj_implicit_factor`]'s), over the awake dofs, a
/// sleeping dof's entry 0: the solve of MuJoCo 3.5.0's `mj_implicitSkip`
/// (`engine_forward.c:1297-1310`).
pub fn mj_implicit_solve(model: &Model, data: &mut Data) {
    implicit_rhs(model, data);
    let index = awake_dofs(model, data).then(|| &data.dof_awake_ind[..data.nv_awake]);
    if model.integrator == Integrator::Implicit {
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
    } else {
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
}

/// Whether MuJoCo's sleep filter is on: sleep enabled and a tree asleep. The
/// mass matrix's factor, Euler's damped one and the implicit integrators'
/// then work over the awake dofs only.
pub fn awake_dofs(model: &Model, data: &Data) -> bool {
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
/// contact's edges decode as `mju_decodePyramid`; an elliptic or
/// frictionless contact's rows are its components.
fn contact_force(rows: &[f64], mu: &[f64; 5], dim: usize, pyramidal: bool) -> [f64; 6] {
    let mut f = [0.0; 6];
    if pyramidal {
        decode_pyramid_into(rows, mu, dim, &mut f);
    } else {
        f[..dim].copy_from_slice(&rows[..dim]);
    }
    f
}

/// Each body's acceleration `cacc` and its external and internal forces
/// `cfrc_ext`, `cfrc_int` after the constraint forces, about its tree's subtree
/// centre of mass in the world's orientation: MuJoCo's `mj_rnePostConstraint`
/// (`engine_core_smooth.c:2491-2683`), every body, a sleeping one included.
///
/// `cfrc_ext` holds each body's applied wrench (from its centre of mass), its
/// contacts' forces (`−` on the first geom's body, `+` on the second's; a
/// contact with a flex adds nothing) and its connect and weld forces (at the
/// anchor, placed by the body's `xmat`; `+` on the first body, `−` on the
/// second), each moved to the body's reference point. `cacc` is the parent's
/// plus `cdof_dot · qvel + cdof · qacc`, the world's `[0; −gravity]`.
/// `cfrc_int = cinert · cacc + cvel ×* (cinert · cvel) − cfrc_ext`, summed into
/// each ancestor's, the world's included.
///
/// One deliberate difference (registry `D-WORLD-CFRC-INT`): MuJoCo adds each
/// root's `cfrc_int`, about that root's own subtree centre of mass, into the
/// world's without moving it; here each is moved to the world's reference
/// point, `subtree_com[0]`, first.
pub fn mj_body_accumulators(model: &Model, data: &mut Data) {
    let reference = |data: &Data, body: usize| data.subtree_com[model.body_rootid[body]];

    data.cacc[0] = SpatialVector::zeros();
    if model.disableflags & DISABLE_GRAVITY == 0 {
        for k in 0..3 {
            data.cacc[0][3 + k] = -model.gravity[k];
        }
    }

    // Applied wrenches, from the centre of mass (`:2503-2517`).
    data.cfrc_ext.fill(SpatialVector::zeros());
    for b in 1..model.nbody {
        let (t, f) = (data.xfrc_applied[b].torque, data.xfrc_applied[b].force);
        if t.iter().chain(f.iter()).all(|&x| x == 0.0) {
            continue;
        }
        let cfrc = SpatialVector::new(t.x, t.y, t.z, f.x, f.y, f.z);
        let moved = mju_transform_spatial(&cfrc, true, &reference(data, b), &data.xipos[b], None);
        data.cfrc_ext[b] += moved;
    }

    // Contact forces (`:2519-2560`): each contact's force and torque in its
    // own frame, as `mj_contactForce` decodes them (`engine_core_util.c:891-908`),
    // turned to the world and moved from the contact point.
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
            // A contact with a flex side adds to no body
            // (`engine_core_smooth.c:2530-2533`).
            let Some((geom1, geom2)) = contact.geoms() else {
                continue;
            };
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
            let (torque, force) = (to_world(&lfrc[3..6]), to_world(&lfrc[0..3]));
            let cfrc = SpatialVector::new(torque.x, torque.y, torque.z, force.x, force.y, force.z);
            let pos = contact.pos;
            let (body1, body2) = (model.geom_body[geom1], model.geom_body[geom2]);
            for (body, add) in [(body1, false), (body2, true)] {
                if body == 0 {
                    continue;
                }
                let moved = mju_transform_spatial(&cfrc, true, &reference(data, body), &pos, None);
                if add {
                    data.cfrc_ext[body] += moved;
                } else {
                    data.cfrc_ext[body] -= moved;
                }
            }
        }
    }

    // Connect and weld forces (`:2562-2651`): the rows' force, and a weld's
    // torque, at each body's anchor, placed by `mj_local2Global` (the body's
    // `xmat`), `+` on body1 and `−` on body2. MuJoCo's anchors: a connect's
    // `eq_data[0..3]` on body1 and `[3..6]` on body2, a weld's the other way.
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
                EqualityType::Joint
                | EqualityType::Distance
                | EqualityType::Tendon
                | EqualityType::Flex => {
                    i += 1;
                    continue;
                }
            };
            let t = if weld {
                [
                    data.efc_force[i + 3],
                    data.efc_force[i + 4],
                    data.efc_force[i + 5],
                ]
            } else {
                [0.0; 3]
            };
            let cfrc = SpatialVector::new(
                t[0],
                t[1],
                t[2],
                data.efc_force[i],
                data.efc_force[i + 1],
                data.efc_force[i + 2],
            );
            let ed = &model.eq_data[eq_id];
            let lo = nalgebra::Vector3::new(ed[0], ed[1], ed[2]);
            let hi = nalgebra::Vector3::new(ed[3], ed[4], ed[5]);
            let (anchor1, anchor2) = if weld { (hi, lo) } else { (lo, hi) };
            for (body, anchor, add) in [
                (model.eq_obj1id[eq_id], anchor1, true),
                (model.eq_obj2id[eq_id], anchor2, false),
            ] {
                if body == 0 {
                    continue;
                }
                let m = &data.xmat[body];
                let pos = nalgebra::Vector3::from_fn(|r, _| {
                    m[(r, 0)] * anchor.x + m[(r, 1)] * anchor.y + m[(r, 2)] * anchor.z
                }) + data.xpos[body];
                let moved = mju_transform_spatial(&cfrc, true, &reference(data, body), &pos, None);
                if add {
                    data.cfrc_ext[body] += moved;
                } else {
                    data.cfrc_ext[body] -= moved;
                }
            }
            i += if weld { 6 } else { 3 };
        }
    }

    // Accelerations and internal forces, root to leaf (`:2653-2674`).
    data.cfrc_int[0] = SpatialVector::zeros();
    for b in 1..model.nbody {
        let dofs = model.body_dof_adr[b]..model.body_dof_adr[b] + model.body_dof_num[b];
        let qvel = &data.qvel.as_slice()[dofs.clone()];
        let qacc = &data.qacc.as_slice()[dofs.clone()];
        data.cacc[b] =
            data.cacc[model.body_parent[b]] + mju_mul_dof_vec(&data.cdof_dot[dofs.clone()], qvel);
        data.cacc[b] += mju_mul_dof_vec(&data.cdof[dofs], qacc);

        let inertia = data.cinert[b];
        let momentum = mju_mul_inert_vec(&inertia, &data.cvel[b]);
        let force = mju_mul_inert_vec(&inertia, &data.cacc[b])
            + spatial_cross_force(data.cvel[b], momentum);
        data.cfrc_int[b] = force - data.cfrc_ext[b];
    }

    // Summed into each parent, leaf to root (`:2676-2679`); a root's moved to
    // the world's reference point (`D-WORLD-CFRC-INT`).
    for b in (1..model.nbody).rev() {
        let parent = model.body_parent[b];
        let force = if parent == 0 {
            mju_transform_spatial(
                &data.cfrc_int[b],
                true,
                &reference(data, 0),
                &reference(data, b),
                None,
            )
        } else {
            data.cfrc_int[b]
        };
        data.cfrc_int[parent] += force;
    }

    data.flg_rnepost = true;
}
