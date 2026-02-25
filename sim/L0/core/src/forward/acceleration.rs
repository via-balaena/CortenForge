//! Forward acceleration computation (explicit and implicit paths).
//!
//! Dispatches to explicit Euler, implicit spring-damper, implicit-fast,
//! or full implicit acceleration solvers. Corresponds to the acceleration
//! portion of MuJoCo's `engine_forward.c`.

use crate::constraint::assembly::tendon_deadband_displacement;
use crate::dynamics::spatial::{SpatialVector, spatial_cross_force};
use crate::integrate::implicit::{accumulate_tendon_kd, tendon_all_dofs_sleeping};
use crate::joint_visitor::{JointContext, JointVisitor, joint_motion_subspace};
use crate::linalg::{
    cholesky_in_place, cholesky_solve_in_place, lu_factor_in_place, lu_solve_factored,
    mj_solve_sparse,
};
use crate::types::{DISABLE_GRAVITY, Data, ENABLE_SLEEP, Integrator, Model, StepError};
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
        Integrator::ImplicitFast => mj_fwd_acceleration_implicitfast(model, data),
        Integrator::Implicit => mj_fwd_acceleration_implicit_full(model, data),
        Integrator::Euler | Integrator::RungeKutta4 => {
            // S4.14: DISABLE_EULERDAMP guard site — MuJoCo's Euler applies
            // optional implicit damping when both eulerdamp and damper are
            // enabled. Our Euler currently uses explicit-only; when implicit
            // Euler damping is added, gate it on:
            //   !disabled(model, DISABLE_EULERDAMP) && !disabled(model, DISABLE_DAMPER)
            mj_fwd_acceleration_explicit(model, data);
            Ok(())
        }
    }
}

/// Explicit forward acceleration (semi-implicit Euler or RK4).
///
/// Computes: qacc = M⁻¹ * (f_applied + f_actuator + f_passive + f_constraint - f_bias)
fn mj_fwd_acceleration_explicit(model: &Model, data: &mut Data) {
    // Sum all forces: τ = applied + actuator + passive + constraint - bias
    let mut qfrc_total = data.qfrc_applied.clone();
    qfrc_total += &data.qfrc_actuator;
    qfrc_total += &data.qfrc_passive;
    qfrc_total += &data.qfrc_constraint;
    qfrc_total -= &data.qfrc_bias;

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

    // Use cached spring-damper parameters from Model (avoids allocation)
    let k = &model.implicit_stiffness;
    let d = &model.implicit_damping;
    let q_eq = &model.implicit_springref;

    // Build external forces into scratch buffer (avoids allocation)
    // f_ext = applied + actuator + passive(friction) + constraint - bias
    data.scratch_force.copy_from(&data.qfrc_applied);
    data.scratch_force += &data.qfrc_actuator;
    data.scratch_force += &data.qfrc_passive; // Friction loss (explicit even in implicit mode)
    data.scratch_force += &data.qfrc_constraint;
    data.scratch_force -= &data.qfrc_bias;

    // Build modified mass matrix: M_impl = M + h*D_jnt + h²*K_jnt
    // Copy M into scratch, then modify diagonal from joint K/D
    data.scratch_m_impl.copy_from(&data.qM);
    for i in 0..model.nv {
        data.scratch_m_impl[(i, i)] += h * d[i] + h2 * k[i];
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
    let mut spring_visitor = ImplicitSpringVisitor {
        k,
        q_eq,
        h,
        qpos: &data.qpos,
        rhs: &mut data.scratch_rhs,
    };
    model.visit_joints(&mut spring_visitor);

    // DT-35: Tendon spring displacement contribution to implicit RHS
    // RHS[dof] -= h · Σ_t k_t · J_t[dof] · deadband_disp(L_t)
    // (sleep_enabled already computed in Step 2, same function scope)
    for t in 0..model.ntendon {
        // Must match the guard in accumulate_tendon_kd — if K is not in the
        // LHS, the corresponding spring displacement must not be in the RHS.
        if sleep_enabled && tendon_all_dofs_sleeping(model, data, t) {
            continue;
        }
        let kt = model.tendon_stiffness[t];
        if kt <= 0.0 {
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
    // M_impl is SPD (M is SPD from CRBA, D ≥ 0, K ≥ 0).
    cholesky_in_place(&mut data.scratch_m_impl)?;
    data.scratch_v_new.copy_from(&data.scratch_rhs);
    cholesky_solve_in_place(&data.scratch_m_impl, &mut data.scratch_v_new);

    // Compute qacc = (v_new - v_old) / h and update qvel
    for i in 0..model.nv {
        data.qacc[i] = (data.scratch_v_new[i] - data.qvel[i]) / h;
        data.qvel[i] = data.scratch_v_new[i];
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

/// Implicit-fast forward acceleration: symmetric D, Cholesky factorization.
///
/// Solves `(M − h·D) · qacc = qfrc_smooth + qfrc_applied + qfrc_constraint`
/// where D = ∂(qfrc_smooth)/∂(qvel) is assembled from DOF damping, tendon
/// damping, and actuator velocity derivatives (Coriolis terms skipped).
/// D is symmetrized: `D ← (D + D^T) / 2`.
///
/// After return, `data.scratch_m_impl` holds the Cholesky factors (L where
/// M−hD = L·L^T), available for derivative column solves in
/// `mjd_transition_hybrid`.
///
/// # Errors
///
/// Returns `Err(StepError::CholeskyFailed)` if `M − h·D` is not positive
/// definite (can happen with positive velocity feedback actuators, see KA#7).
fn mj_fwd_acceleration_implicitfast(model: &Model, data: &mut Data) -> Result<(), StepError> {
    use crate::derivatives::{mjd_actuator_vel, mjd_passive_vel};

    let h = model.timestep;

    // Step 1: Assemble qDeriv = ∂(qfrc_smooth)/∂(qvel)
    //   Components: DOF damping + tendon damping J^T B J + actuator vel derivatives
    //   Coriolis terms SKIPPED for implicitfast.
    data.qDeriv.fill(0.0);
    mjd_passive_vel(model, data);
    mjd_actuator_vel(model, data);

    // Step 2: Symmetrize D ← (D + D^T) / 2
    for i in 0..model.nv {
        for j in (i + 1)..model.nv {
            let avg = 0.5 * (data.qDeriv[(i, j)] + data.qDeriv[(j, i)]);
            data.qDeriv[(i, j)] = avg;
            data.qDeriv[(j, i)] = avg;
        }
    }

    // Step 3: Form M_hat = M − h·D into scratch_m_impl
    data.scratch_m_impl.copy_from(&data.qM);
    for i in 0..model.nv {
        for j in 0..model.nv {
            data.scratch_m_impl[(i, j)] -= h * data.qDeriv[(i, j)];
        }
    }

    // Step 4: RHS = qfrc_smooth + qfrc_applied + qfrc_constraint
    data.scratch_rhs.copy_from(&data.qfrc_applied);
    data.scratch_rhs += &data.qfrc_actuator;
    data.scratch_rhs += &data.qfrc_passive;
    data.scratch_rhs += &data.qfrc_constraint;
    data.scratch_rhs -= &data.qfrc_bias;

    // Step 5: Solve (M − h·D) · qacc = rhs via dense Cholesky
    cholesky_in_place(&mut data.scratch_m_impl)?;
    data.qacc.copy_from(&data.scratch_rhs);
    cholesky_solve_in_place(&data.scratch_m_impl, &mut data.qacc);

    Ok(())
}

/// Full implicit forward acceleration: asymmetric D, LU factorization.
///
/// Same as `mj_fwd_acceleration_implicitfast` but includes Coriolis velocity
/// derivatives (`mjd_rne_vel`), does NOT symmetrize D, and uses LU
/// factorization instead of Cholesky.
///
/// After return, `data.scratch_m_impl` holds the LU factors and
/// `data.scratch_lu_piv` holds the pivot permutation, available for
/// derivative column solves in `mjd_transition_hybrid`.
///
/// # Errors
///
/// Returns `Err(StepError::LuSingular)` if any pivot magnitude is below
/// `1e-30` during LU factorization.
fn mj_fwd_acceleration_implicit_full(model: &Model, data: &mut Data) -> Result<(), StepError> {
    use crate::derivatives::{mjd_actuator_vel, mjd_passive_vel, mjd_rne_vel};

    let h = model.timestep;

    // Step 1: Assemble qDeriv = ∂(qfrc_smooth)/∂(qvel) — ALL components
    data.qDeriv.fill(0.0);
    mjd_passive_vel(model, data);
    mjd_actuator_vel(model, data);
    mjd_rne_vel(model, data);

    // Step 2: No symmetrization — D is asymmetric (Coriolis terms break symmetry)

    // Step 3: Form M_hat = M − h·D into scratch_m_impl
    data.scratch_m_impl.copy_from(&data.qM);
    for i in 0..model.nv {
        for j in 0..model.nv {
            data.scratch_m_impl[(i, j)] -= h * data.qDeriv[(i, j)];
        }
    }

    // Step 4: RHS = qfrc_smooth + qfrc_applied + qfrc_constraint
    data.scratch_rhs.copy_from(&data.qfrc_applied);
    data.scratch_rhs += &data.qfrc_actuator;
    data.scratch_rhs += &data.qfrc_passive;
    data.scratch_rhs += &data.qfrc_constraint;
    data.scratch_rhs -= &data.qfrc_bias;

    // Step 5: Factor (M − h·D) = P·L·U, then solve for qacc
    lu_factor_in_place(&mut data.scratch_m_impl, &mut data.scratch_lu_piv)?;
    data.qacc.copy_from(&data.scratch_rhs);
    lu_solve_factored(&data.scratch_m_impl, &data.scratch_lu_piv, &mut data.qacc);

    Ok(())
}

/// Compute per-body accelerations and forces after forward dynamics (§51).
///
/// Populates `data.cacc`, `data.cfrc_int`, and `data.cfrc_ext`:
///
/// 1. **cfrc_ext**: Copy of `xfrc_applied` for each body (Cartesian external forces).
///
/// 2. **cacc forward pass** (root→leaf): Propagate accelerations through the
///    kinematic chain. `cacc[0] = [0,0,0, -gx,-gy,-gz]` (gravity pseudo-acceleration).
///    For each body: `cacc[b] = cacc[parent] + S * qacc` (joint acceleration
///    contribution via motion subspace).
///
/// 3. **cfrc_int backward pass** (leaf→root):
///    `cfrc_int[b] = I*cacc[b] + v×*(I*v) - cfrc_ext[b]`, accumulated into parent.
///
/// Mirrors the RNE algorithm structure but uses actual `qacc` instead of bias-only.
///
/// # MuJoCo Equivalence
///
/// Matches `mj_rnePostConstraint()` in `engine_forward.c`.
#[allow(clippy::needless_range_loop)] // jnt_id indexes both joint_subspaces and model arrays
pub fn mj_body_accumulators(model: &Model, data: &mut Data) {
    // ===== Step 1: cfrc_ext = copy of xfrc_applied =====
    for b in 0..model.nbody {
        data.cfrc_ext[b] = data.xfrc_applied[b];
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

        // Start with parent's acceleration
        let mut acc = data.cacc[parent_id];

        // Add joint acceleration contribution: S * qacc
        let jnt_start = model.body_jnt_adr[body_id];
        let jnt_end = jnt_start + model.body_jnt_num[body_id];

        for jnt_id in jnt_start..jnt_end {
            let dof_adr = model.jnt_dof_adr[jnt_id];
            let s = &joint_subspaces[jnt_id];
            let nv = model.jnt_type[jnt_id].nv();

            for row in 0..6 {
                for d in 0..nv {
                    acc[row] += s[(row, d)] * data.qacc[dof_adr + d];
                }
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

    // Propagate internal forces from leaves to root
    for body_id in (1..model.nbody).rev() {
        let parent_id = model.body_parent[body_id];
        if parent_id != 0 {
            let child_force = data.cfrc_int[body_id];
            data.cfrc_int[parent_id] += child_force;
        }
    }
}
