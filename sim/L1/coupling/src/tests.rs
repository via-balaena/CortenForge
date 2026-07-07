//! Unit tests for the staggered soft↔rigid coupling + its differentiable
//! gradient family (single-step, trajectory, articulated, actuated, policy).
#![cfg(test)]

use super::*;
use sim_core::SpatialVector;
use sim_mjcf::load_model;
use sim_ml_chassis::Tensor;
use sim_ml_chassis::autograd::VjpOp;
use sim_soft::{BoundaryConditions, CpuNewtonSolver, HandBuiltTetMesh, Solver, Tet4};

use crate::contact::SoftSolver;
use crate::vjp::{
    ContactForceTrajVjp, ContactWrenchTrajVjp, FrictionWrenchTrajVjp, StateComponentVjp,
    VzControlCarryVjp, WrenchPose, assemble_friction_wrench, right_jacobian_so3,
};

/// A centred free-body platen over the soft block (COM at the block centroid), with contact-axis
/// `damping`, moment on. Used by the carry-subsumption proof.
fn centred_freebody(mu: f64, damping: f64) -> StaggeredCoupling {
    let mjcf = r#"<mujoco><option gravity="0 0 -9.81" timestep="0.001"/><worldbody>
  <body name="p" pos="0.05 0.05 0.1049"><freejoint/>
<geom type="box" size="0.06 0.06 0.005" mass="0.2"/></body></worldbody></mujoco>"#;
    let model = load_model(mjcf).expect("load");
    let mut data = model.make_data();
    data.forward(&model).expect("fwd");
    StaggeredCoupling::new(
        model, data, 1, 0.005, 4, 0.1, mu, 1.0e-3, 3.0e4, 1.0e-2, damping,
    )
    .with_contact_moment(true)
}

/// `∂z_N/∂param` via the GENERAL wrench carry, reading the stale-FK `xpos.z` (= `s_prev`'s
/// z-translation row) so the readout matches `step`'s `rigid_z`.
fn z_grad_via_wrench(mu: f64, n: usize, param_idx: usize, damping: f64) -> f64 {
    let mut c = centred_freebody(mu, damping);
    let (_, dadr) = c.free_joint_adrs();
    let nv = c.model.nv;
    let (mut tape, p_var, _s_final, s_prev) = c.build_freebody_wrench_tape(n, param_idx);
    let z = c.data.xpos[c.body].z;
    // The state's z-translation is the free joint's 3rd linear DOF row, `dadr + 2` (the qpos
    // rows of `s` are DOF-indexed; z-translation is Euclidean, so raw == tangent).
    let obj = tape.push_custom(
        &[s_prev],
        Tensor::from_slice(&[z], &[1]),
        Box::new(StateComponentVjp {
            idx: dadr + 2,
            n_state: 2 * nv,
        }),
    );
    tape.backward(obj);
    tape.grad_tensor(p_var).as_slice()[0]
}

/// The scalar `VzCarryVjp`/`ZCarryVjp` z-carry is the **z-special-case of the general wrench
/// carry**: reading `s_prev`'s stale-FK z-translation through the general carry reproduces
/// `coupled_trajectory_material_gradient`'s `z_N` gradient to machine precision — both undamped
/// AND under the contact-axis damping the keystone uses (`rigid_damping = 60`, captured by the
/// damping-aware loaded `J_state`). Proves the two free-body carries are one dynamics at two
/// readout conventions (not redundant implementations), so retiring the scalar carry would be a
/// safe, provable move. (λ=4μ block tie ⇒ compare the design gradient `grad(0) + 4·grad(1)`.)
#[test]
fn wrench_carry_subsumes_scalar_z_carry() {
    for &damping in &[0.0_f64, 60.0] {
        for &n in &[4_usize, 8, 16] {
            let g_wrench = z_grad_via_wrench(3.0e4, n, 0, damping)
                + 4.0 * z_grad_via_wrench(3.0e4, n, 1, damping);
            let g_scalar = centred_freebody(3.0e4, damping)
                .coupled_trajectory_material_gradient(n, 0)
                .1
                + 4.0
                    * centred_freebody(3.0e4, damping)
                        .coupled_trajectory_material_gradient(n, 1)
                        .1;
            let rel = (g_wrench - g_scalar).abs() / g_scalar.abs().max(1e-30);
            assert!(
                rel < 1e-8,
                "general wrench carry must reproduce the scalar z_N gradient at \
                 damping={damping} n={n}: wrench={g_wrench:e} scalar={g_scalar:e} rel={rel:e}"
            );
        }
    }
}

/// [`right_jacobian_so3`] is the exact tangent map of the quaternion exp-map step,
/// in the "output tangent at the nominal `q'`" convention: for `q' = q ⊕ exp(φ)`,
/// perturbing `φ → φ + δ` gives `log(q'⁻¹ · q'(φ+δ)) = J_r(φ)·δ`. FD-validated against
/// the real SO(3) integration at several angles (small, moderate, large).
#[test]
fn right_jacobian_so3_matches_quaternion_expmap_fd() {
    use sim_core::UnitQuaternion;
    // exp-map of a rotation vector → unit quaternion (right tangent).
    let expq = |v: Vec3| -> UnitQuaternion<f64> {
        let a = v.norm();
        if a < 1e-12 {
            UnitQuaternion::identity()
        } else {
            UnitQuaternion::from_axis_angle(&nalgebra::Unit::new_normalize(v), a)
        }
    };
    for phi in [
        Vec3::new(1e-7, 0.0, 0.0),    // near-zero (small-angle branch)
        Vec3::new(0.03, -0.02, 0.05), // moderate
        Vec3::new(0.8, -0.6, 0.4),    // large
    ] {
        let jr = right_jacobian_so3(phi);
        let qp = expq(phi); // the nominal q' (base q = identity, WLOG for the tangent map)
        let eps = 1e-6;
        for k in 0..3 {
            let mut d = Vec3::zeros();
            d[k] = eps;
            // log(q'⁻¹ · exp(φ+δ)) and · exp(φ−δ), central difference → column k.
            let plus = qp.inverse() * expq(phi + d);
            let minus = qp.inverse() * expq(phi - d);
            let logv = |q: UnitQuaternion<f64>| -> Vec3 {
                let v = Vec3::new(q.i, q.j, q.k);
                let s = v.norm();
                if s < 1e-12 {
                    Vec3::zeros()
                } else {
                    v * (2.0 * s.atan2(q.w) / s)
                }
            };
            let col_fd = (logv(plus) - logv(minus)) / (2.0 * eps);
            let col_an = jr.column(k);
            let err = (col_fd - col_an).norm();
            assert!(
                err < 1e-5,
                "J_r column {k} at φ={phi:?}: analytic {col_an:?} != FD {col_fd:?} (err {err:.3e})"
            );
        }
    }
}

/// The Vjp ops scatter their PRECOMPUTED per-pair factors `(vertex_id, ∂fz/∂x_v,
/// ∂fz/∂height)` straight through: `ContactForceVjp` scatters `∂fz/∂x_v` onto x*, and
/// `ContactForceTrajVjp` additionally sums `∂fz/∂height` onto its z parent. Distinct
/// per-pair values verify both ops route the factors correctly (the curvature-aware
/// generalization beyond the constant-`κ` penalty path the keystone gates exercise);
/// the flat plane values `∂fz/∂x_v = (0,0,−cᵥ)`, `∂fz/∂height = cᵥ` here mirror what
/// `active_pair_force_factors` produces for `n̂ = −ẑ`, `H = 0`.
#[test]
fn contact_force_factors_scatter_per_pair() {
    // Precomputed factors for two pairs with distinct stiffness cᵥ = 2, 5 on the flat
    // plane: ∂fz/∂x_v = (0,0,−cᵥ), ∂fz/∂height = cᵥ.
    let factors = vec![
        (0_usize, Vec3::new(0.0, 0.0, -2.0), 2.0),
        (1_usize, Vec3::new(0.0, 0.0, -5.0), 5.0),
    ];
    // ContactForceVjp scatters ∂fz/∂x_v (cot=1): z-components −2, −5.
    let mut parent = vec![Tensor::zeros(&[6])];
    ContactForceVjp::new(factors.clone()).vjp(&Tensor::from_slice(&[1.0], &[1]), &mut parent);
    let g = parent[0].as_slice();
    assert!(
        (g[2] + 2.0).abs() < 1e-12 && (g[5] + 5.0).abs() < 1e-12,
        "∂fz/∂x* should carry the per-pair ∂fz/∂x_v factor, got {g:?}"
    );
    // ContactForceTrajVjp: ∂fz/∂z = Σ ∂fz/∂height = 7.
    let traj = ContactForceTrajVjp { factors, n_dof: 6 };
    let mut parents = vec![Tensor::zeros(&[6]), Tensor::zeros(&[1])];
    traj.vjp(&Tensor::from_slice(&[1.0], &[1]), &mut parents);
    assert!(
        (parents[1].as_slice()[0] - 7.0).abs() < 1e-12,
        "∂fz/∂z should be Σ ∂fz/∂height = 7, got {}",
        parents[1].as_slice()[0]
    );
}

const PLATEN_MJCF: &str = r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
<body name="platen" pos="0 0 0.125">
  <freejoint/>
  <geom type="box" size="0.06 0.06 0.005" mass="0.2"/>
</body>
  </worldbody>
</mujoco>"#;

// `.expect` surfaces a malformed fixture as a test panic — the canonical
// fixture idiom; the coupling itself is graded by the integration gate.
#[allow(clippy::expect_used)]
fn coupling() -> StaggeredCoupling {
    let model = load_model(PLATEN_MJCF).expect("platen MJCF loads");
    let mut data = model.make_data();
    data.forward(&model).expect("initial forward");
    StaggeredCoupling::new(
        model, data, 1, 0.005, 4, 0.1, 3.0e4, 1.0e-3, 3.0e4, 1.0e-2, 8.0,
    )
}

#[test]
fn coupled_step_is_bounded_and_engages_contact() {
    let mut c = coupling();
    let z0 = c.data().xpos[1].z;
    let mut last = c.step();
    for _ in 0..150 {
        last = c.step();
    }
    // platen descended under gravity, loop stayed finite + bounded.
    assert!(
        last.rigid_z.is_finite() && last.rigid_z < z0 && last.rigid_z > 0.0,
        "platen z out of range: {} (z0={z0})",
        last.rigid_z
    );
    // contact has engaged: a nonzero upward reaction on the platen.
    assert!(
        -last.force_on_soft.z > 0.0,
        "expected an upward contact reaction once engaged; got {:?}",
        last.force_on_soft
    );
}

/// Lib-level smoke test of the differentiability probes (the scientific
/// FD validation lives in the `tests/` gates `contact_force_jacobian`,
/// `coupled_step_jacobian`, `coupled_total_jacobian`). Exercises every
/// probe at a deeply-engaged height (the rest top face is at z=0.1; the
/// plane at h=0.099 penetrates ~1 mm so the 25 top vertices are active)
/// and pins the qualitative relationships the gates quantify.
#[test]
fn differentiability_probes_are_finite_and_consistent() {
    let c = coupling();
    let h = 0.099;

    // S1 explicit (fixed-soft-position) factor — `+κ·N_active·ẑ`, positive.
    let explicit = c.contact_force_height_jacobian(h);
    assert!(
        explicit.z > 0.0 && explicit.z.is_finite(),
        "explicit ∂force/∂h.z should be +κ·N > 0, got {explicit:?}"
    );

    // Forward building blocks: fixed-position vs re-solved contact force.
    let f_fixed = c.contact_force_at_height(h);
    let f_resolved = c.resolved_contact_force(h);
    assert!(
        f_fixed.z != 0.0 && f_resolved.z.is_finite(),
        "engaged contact force should be nonzero/finite: fixed {f_fixed:?}, resolved {f_resolved:?}"
    );

    // S2 rigid factor probe: a free body accelerates under an applied force.
    let (z_up, vz_up) = c.rigid_step_probe(1.0);
    let (_z_dn, vz_dn) = c.rigid_step_probe(-1.0);
    assert!(
        z_up.is_finite() && vz_up > vz_dn,
        "more upward force ⇒ higher next vz: vz(+1)={vz_up}, vz(-1)={vz_dn}"
    );

    // S3 total factor: the implicit soft re-equilibration reduces the
    // explicit-only sensitivity (the soft body follows the rising plane).
    let total = c.contact_force_height_total_jacobian(h);
    assert!(
        total.z.is_finite() && total.z.abs() < explicit.z.abs(),
        "total ∂force/∂h.z ({}) should be finite and smaller in magnitude \
         than the explicit-only partial ({}) — implicit re-equilibration",
        total.z,
        explicit.z,
    );
}

/// Lib-level smoke test of the S4 cross-engine tape crossing (the scientific
/// FD validation is in `tests/rigid_step_vjp.rs` and the `load·plane`/`load·sphere`
/// rows of `tests/coupling_grad_harness.rs`).
/// Exercises `rigid_vz_response`, `coupled_step_load_gradient` (one
/// `tape.backward` across both engines), and the forward oracle
/// `coupled_step_load_vz` at a deeply-engaged height with a loaded top face.
#[test]
fn cross_engine_crossing_smoke() {
    let c = coupling();
    let mesh = HandBuiltTetMesh::uniform_block(4, 0.1, &MaterialField::uniform(3.0e4, 1.2e5));
    let loaded: Vec<VertexId> =
        sim_soft::pick_vertices_by_predicate(&mesh, |p| (p.z - 0.1).abs() < 1e-9);
    assert!(!loaded.is_empty());

    // Rigid response factor is the free-body dt/m (= 5e-3).
    let (_vz, dvz_dfz) = c.rigid_vz_response(5.0);
    assert!(
        (dvz_dfz - 1.0e-3 / 0.2).abs() < 1e-9,
        "∂vz'/∂fz should be dt/m"
    );

    let h = 0.099;
    let theta0 = 5.0;
    let (vz, grad) = c.coupled_step_load_gradient(h, &loaded, theta0);
    assert!(vz.is_finite() && grad.is_finite());
    // The cross-engine gradient is nonzero (load couples through to vz')
    // and consistent with the forward oracle's local slope sign.
    assert!(
        grad.abs() > 1e-4,
        "expected a nonzero cross-engine gradient"
    );
    let eps = 1.0e-4;
    let fd = (c.coupled_step_load_vz(h, &loaded, theta0 + eps)
        - c.coupled_step_load_vz(h, &loaded, theta0 - eps))
        / (2.0 * eps);
    assert!(
        (grad - fd).abs() / fd.abs() < 1e-6,
        "smoke: tape grad {grad} vs FD {fd}"
    );
}

/// Lib-level smoke test of the multi-step time-adjoint (the scientific FD
/// validation is the `platen·material[μ]` row of `tests/coupling_grad_harness.rs`
/// plus the all-lengths sweep in `tests/coupled_trajectory_gradient.rs`): one
/// `tape.backward` over a coupled rollout that crosses the contact make event
/// gives a finite gradient, and the tape's forward rollout reproduces the
/// real `step` dynamics exactly.
#[test]
fn trajectory_gradient_smoke() {
    // Independent couplings: one for the tape gradient, one for the real
    // reference rollout (the gradient call advances its own coupling).
    let (z_tape, grad) = coupling().coupled_trajectory_material_gradient(80, 0);
    let mut c_ref = coupling();
    let mut z_ref = c_ref.data().xpos[1].z;
    for _ in 0..80 {
        z_ref = c_ref.step().rigid_z;
    }
    assert!(
        (z_tape - z_ref).abs() < 1e-12,
        "tape forward z_N {z_tape} != real rollout {z_ref}"
    );
    // Past the make event the gradient is finite and nonzero (the block
    // stiffness bears the platen).
    assert!(
        grad.is_finite() && grad.abs() > 1e-12,
        "expected a finite nonzero ∂z_N/∂μ"
    );
}

/// Lib-level smoke test of the friction-coefficient trajectory gradient (the
/// scientific FD validation is the `friction·tangential-coeff[μ_c]` row of the coupling
/// gradient harness, `tests/coupling_grad_harness.rs`):
/// the `μ_c` driver's tape forward reproduces the real grip rollout exactly, and one
/// `tape.backward` gives a finite nonzero `∂x_N/∂μ_c` through both channels (the soft
/// equilibrium shift and the direct reaction `∂fx/∂μ_c`). Uses a tilted-gravity,
/// contact-engaged scene (the default `coupling()` has no lateral drive ⇒ no grip slip).
#[test]
// expect_used: test fixture — a malformed MJCF/setup surfaces as a loud panic, the canonical idiom.
#[allow(clippy::expect_used)]
fn friction_coeff_trajectory_gradient_smoke() {
    // Tilted gravity + a platen started engaged (z = 0.115) so the grip slides.
    const GRIP_MJCF: &str = r#"<mujoco>
  <option gravity="2.0 0 -9.81" timestep="0.001"/>
  <worldbody>
<body name="platen" pos="0 0 0.115">
  <freejoint/>
  <geom type="box" size="0.06 0.06 0.005" mass="0.2"/>
</body>
  </worldbody>
</mujoco>"#;
    let build = || -> StaggeredCoupling {
        let model = load_model(GRIP_MJCF).expect("grip MJCF loads");
        let mut data = model.make_data();
        data.forward(&model).expect("initial forward");
        StaggeredCoupling::new(
            model, data, 1, 0.005, 4, 0.1, 3.0e3, 1.0e-3, 3.0e4, 1.0e-2, 8.0,
        )
        .with_friction(2.5, 0.1)
    };
    let (x_tape, grad) = build().coupled_trajectory_tangential_friction_coeff_gradient(20);
    let x_ref = build().coupled_trajectory_grip(20).x;
    assert!(
        (x_tape - x_ref).abs() < 1e-12,
        "tape forward x_N {x_tape} != real grip rollout {x_ref}"
    );
    assert!(
        grad.is_finite() && grad.abs() > 1e-12,
        "expected a finite nonzero ∂x_N/∂μ_c"
    );
}

/// Lib-level smoke test of the S5 co-design crossing `∂vz'/∂μ` (the
/// scientific FD validation is the `material[μ]`/`material[λ]` rows of
/// `tests/coupling_grad_harness.rs`): the material parameter rides the same
/// crossing via `MaterialStepVjp`.
#[test]
fn material_crossing_smoke() {
    let c = coupling();
    let h = 0.099;
    let (vz, grad_mu) = c.coupled_step_material_gradient(h, 0);
    assert!(vz.is_finite() && grad_mu.is_finite());
    assert!(grad_mu.abs() > 1e-9, "expected a nonzero ∂vz'/∂μ");
    let eps = 3.0e4 * 1e-6;
    let fd = (c.coupled_step_material_vz(h, 0, 3.0e4 + eps)
        - c.coupled_step_material_vz(h, 0, 3.0e4 - eps))
        / (2.0 * eps);
    assert!(
        (grad_mu - fd).abs() / fd.abs() < 1e-5,
        "smoke: material tape grad {grad_mu} vs FD {fd}"
    );
}

/// Lib-level smoke test of the multi-DOF rigid factor `rigid_xfrc_column`
/// (the scientific FD validation — hinge / 2-link / free-body — is in
/// `tests/rigid_multidof_response.rs`): on the free platen the column reduces
/// to the scalar `dt/m` on the contact axis, tying the matrix factor to the
/// merged `rigid_vz_response`.
#[test]
fn rigid_xfrc_column_free_body_smoke() {
    let model = load_model(PLATEN_MJCF).expect("platen MJCF loads");
    let mut data = model.make_data();
    data.forward(&model).expect("forward");
    let col = rigid_xfrc_column(&model, &data, 1);
    assert_eq!(col.shape(), (6, 6)); // free joint: nv=6, 6 spatial-force columns
    // ∂vz'/∂f_z (qvel[2] vs xfrc[5]) is the free-body dt/m (read from the model,
    // not hardcoded, so it can't go stale if the fixture changes).
    let dt_over_m = model.timestep / model.body_mass[1];
    assert!(
        (col[(2, 5)] - dt_over_m).abs() < 1e-12,
        "free-body ∂vz'/∂f_z must be dt/m, got {}",
        col[(2, 5)]
    );
}

/// Lib-level smoke of the multi-DOF (articulated) coupled trajectory gradient
/// (the scientific hinge FD validation is in
/// `tests/articulated_trajectory_gradient.rs`). Exercises the full articulated
/// path — the `PoseSeamVjp`, the loaded `RigidStateCarryVjp`, and the forward
/// oracle — on a free-joint body (nv = 6, `rigid_damping = 0`): the tape forward
/// `tip_z_N` reproduces the real rollout and the gradient is finite.
#[test]
fn articulated_trajectory_smoke() {
    // A free-joint platen with rigid_damping = 0 (the articulated path's scope).
    let build = || {
        let model = load_model(PLATEN_MJCF).expect("platen MJCF loads");
        let mut data = model.make_data();
        data.forward(&model).expect("forward");
        StaggeredCoupling::<PenaltyRigidContact>::new(
            model, data, 1, 0.005, 4, 0.1, 3.0e4, 1.0e-3, 3.0e4, 1.0e-2, 0.0,
        )
    };
    let (tip_z, grad) = build().coupled_trajectory_material_gradient_articulated(5, 0);
    assert!(tip_z.is_finite() && grad.is_finite());
    let z_ref = build().coupled_trajectory_articulated_z(5);
    assert!(
        (tip_z - z_ref).abs() < 1e-12,
        "tape forward tip_z {tip_z} != real rollout {z_ref}"
    );
}

/// The analytic single-hinge state Jacobian ([`StaggeredCoupling::analytic_state_jacobian`])
/// is MACHINE-EXACT against the FD [`StaggeredCoupling::loaded_state_jacobian`] for an
/// arbitrary held wrench (force AND torque components, nonzero `qvel`) — pinning the
/// closed-form geometric stiffness `Δt·M⁻¹·∂(Jᵀw)/∂q = Δt·M⁻¹·(â×(â×r))·f` added to the
/// unloaded transition `A`. It also confirms the scope predicate: `single_hinge` selects
/// the hinge and rejects the free joint (which falls back to the FD form, J = I ⇒ no
/// geometric stiffness). This replaces the FD Jacobian's noise with the exact term; see
/// `docs/keystone/geometric_stiffness_recon.md`.
#[test]
fn analytic_state_jacobian_matches_fd_loaded() {
    const HINGE_MJCF: &str = r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
<body name="arm" pos="0 0 0.2">
  <joint type="hinge" axis="0 1 0"/>
  <geom type="sphere" pos="0 0 -0.095" size="0.004" mass="0.2"/>
</body>
  </worldbody>
</mujoco>"#;
    let model = load_model(HINGE_MJCF).expect("hinge MJCF loads");
    let mut data = model.make_data();
    data.qpos[0] = 0.3;
    data.qvel[0] = -1.5; // nonzero velocity exercises the full Jacobian
    data.forward(&model).expect("forward");
    let c: StaggeredCoupling = StaggeredCoupling::new(
        model, data, 1, 0.005, 4, 0.1, 3.0e4, 1.0e-3, 3.0e4, 1.0e-2, 0.0,
    );
    assert!(
        c.single_hinge().is_some(),
        "hinge must select the analytic path"
    );

    for wrench in [
        SpatialVector::from_row_slice(&[0.1, 0.2, 0.0, 5.0, -3.0, 800.0]),
        SpatialVector::from_row_slice(&[0.0, 0.0, 0.0, 0.0, 0.0, 600.0]),
        SpatialVector::from_row_slice(&[0.3, -0.4, 0.2, -10.0, 7.0, -250.0]),
    ] {
        let loaded = c.loaded_state_jacobian(&wrench);
        let analytic = c
            .analytic_state_jacobian(&wrench)
            .expect("single hinge → analytic");
        let (err, loc) = sim_core::max_relative_error(&loaded, &analytic, 1e-3);
        assert!(
            err < 1e-6,
            "analytic J_state must match FD loaded to FD precision, got rel {err:.3e} at {loc:?}"
        );
    }

    // Free-joint platen: no single hinge ⇒ the analytic path declines (FD fallback).
    let pmodel = load_model(PLATEN_MJCF).expect("platen MJCF loads");
    let mut pdata = pmodel.make_data();
    pdata.forward(&pmodel).expect("forward");
    let pc: StaggeredCoupling = StaggeredCoupling::new(
        pmodel, pdata, 1, 0.005, 4, 0.1, 3.0e4, 1.0e-3, 3.0e4, 1.0e-2, 0.0,
    );
    assert!(
        pc.single_hinge().is_none()
            && pc
                .analytic_state_jacobian(&SpatialVector::zeros())
                .is_none(),
        "free joint must fall back to the FD loaded Jacobian"
    );
}

/// Lib smoke for the articulated FRICTION-grip path: the full gripped forward oracle and the
/// matrix-carry friction material gradient (exercising the per-vertex friction wrench node,
/// the articulated drift node, and sim-soft's `friction_force_jacobians`). Machine-exact
/// accuracy lives in the `friction_articulated_material_gradient` integration test; this
/// keeps the `--lib` coverage honest over the new public surface.
#[test]
fn articulated_friction_grip_smoke() {
    const HINGE_MJCF: &str = r#"<mujoco>
  <option gravity="2.0 0 -9.81" timestep="0.001"/>
  <worldbody>
<body name="arm" pos="0 0 0.2">
  <joint type="hinge" axis="0 1 0"/>
  <geom type="sphere" pos="0 0 -0.095" size="0.004" mass="0.2"/>
</body>
  </worldbody>
</mujoco>"#;
    let build = || -> StaggeredCoupling {
        let model = load_model(HINGE_MJCF).expect("hinge MJCF loads");
        let mut data = model.make_data();
        data.qpos[0] = 0.3;
        data.forward(&model).expect("forward");
        StaggeredCoupling::new(
            model, data, 1, 0.005, 4, 0.1, 3.0e3, 1.0e-3, 3.0e4, 1.0e-2, 0.0,
        )
        .with_friction(2.5, 0.1)
    };
    let tip = build().coupled_trajectory_gripped_articulated(3);
    assert!(
        tip.x.is_finite() && (0.10..0.12).contains(&tip.z),
        "gripped articulated rollout must stay finite and engaged, got {tip:?}"
    );
    let (tip_x, grad) = build().coupled_trajectory_tangential_material_gradient_articulated(3, 0);
    assert!(
        tip_x.is_finite() && grad.is_finite(),
        "articulated friction material gradient must be finite, got ({tip_x}, {grad})"
    );
    let (cx, cgrad) = build().coupled_trajectory_tangential_friction_coeff_gradient_articulated(3);
    assert!(
        cx.is_finite() && cgrad.is_finite(),
        "articulated friction-coeff gradient must be finite, got ({cx}, {cgrad})"
    );
}

/// Lib smoke for the actuator CONTROL gradient through the friction grip: the gripped-actuated
/// forward oracle and the control gradient (exercising the friction wrench node + the actuator
/// `g_act` channel on one tape). Machine-exact accuracy lives in the
/// `actuator_friction_gradient` integration test; this keeps `--lib` coverage honest.
#[test]
fn actuator_friction_grip_smoke() {
    const MOTOR_MJCF: &str = r#"<mujoco>
  <option gravity="2.0 0 -9.81" timestep="0.001"/>
  <worldbody>
<body name="arm" pos="0 0 0.2">
  <joint name="j" type="hinge" axis="0 1 0"/>
  <geom type="sphere" pos="0 0 -0.095" size="0.004" mass="0.2"/>
</body>
  </worldbody>
  <actuator><motor joint="j" gear="1"/></actuator>
</mujoco>"#;
    let build = || -> StaggeredCoupling {
        let model = load_model(MOTOR_MJCF).expect("motor MJCF loads");
        let mut data = model.make_data();
        data.qpos[0] = 0.3;
        data.forward(&model).expect("forward");
        StaggeredCoupling::new(
            model, data, 1, 0.005, 4, 0.1, 3.0e3, 1.0e-3, 3.0e4, 1.0e-2, 0.0,
        )
        .with_friction(2.5, 0.1)
    };
    let controls = [0.03_f64; 3];
    let tip = build().coupled_trajectory_actuated_gripped_x(&controls);
    assert!(
        tip.x.is_finite() && (0.10..0.12).contains(&tip.z),
        "gripped-actuated rollout must stay finite and engaged, got {tip:?}"
    );
    let (tx, grad) = build().coupled_trajectory_actuator_friction_gradient(&controls);
    assert!(
        tx.is_finite() && grad.iter().all(|g| g.is_finite()),
        "actuator-friction gradient must be finite, got ({tx}, {grad:?})"
    );
    // Closed-loop policy gradient through friction (backprop-through-time).
    let params = [0.08, -0.02, 0.01];
    let pol = build().coupled_trajectory_policy_gripped_x(&LinearFeedback, &params, 3);
    assert!(
        pol.x.is_finite() && (0.10..0.12).contains(&pol.z),
        "policy-gripped rollout must stay finite and engaged, got {pol:?}"
    );
    // The frame-capturing sibling: byte-identical tip_x + n_steps+1 frames (the R5 viz
    // surface). Accuracy/identity gates live in `tests/grip_rollout_capture.rs`.
    let (cap_tip, frames) =
        build().coupled_trajectory_policy_gripped_capture(&LinearFeedback, &params, 3);
    assert!(
        cap_tip.x == pol.x && frames.len() == 4,
        "capture must byte-match the scalar oracle and yield n_steps+1 frames, \
         got tip {} vs {} / {} frames",
        cap_tip.x,
        pol.x,
        frames.len()
    );
    let (px, pgrad) =
        build().coupled_trajectory_policy_friction_gradient(&LinearFeedback, &params, 3);
    assert!(
        px.is_finite() && pgrad.len() == 3 && pgrad.iter().all(|g| g.is_finite()),
        "policy-friction gradient must be finite, got ({px}, {pgrad:?})"
    );
    // Design + policy on ONE friction-grip tape: the material μ leaf AND the policy θ leaves,
    // both differentiated in one backward.
    let (dx, dmu, dgrad) =
        build().coupled_trajectory_design_policy_friction_gradient(&LinearFeedback, &params, 3);
    assert!(
        dx.is_finite()
            && dmu.is_finite()
            && dgrad.len() == 3
            && dgrad.iter().all(|g| g.is_finite()),
        "design+policy-friction gradient must be finite, got ({dx}, {dmu}, {dgrad:?})"
    );
    // Trajectory-integrated HOLDING gradient (Σ (qₖ − q_hold)²) on the SAME shared tape —
    // accuracy is gated by the `*·design-policy-hold[μ+θ]` rows of `coupling_grad_harness.rs`;
    // this keeps --lib coverage honest.
    let (lcost, lmu, lgrad) =
        build().coupled_trajectory_design_policy_hold_gradient(&LinearFeedback, &params, 3, 0.25);
    assert!(
        lcost.is_finite()
            && lmu.is_finite()
            && lgrad.len() == 3
            && lgrad.iter().all(|g| g.is_finite()),
        "holding gradient must be finite, got ({lcost}, {lmu}, {lgrad:?})"
    );
}

/// A DAMPED single hinge's analytic `J_state` matches the FD `loaded_state_jacobian` to the
/// FD floor: the `M → M_impl = M + Δt·D` correction (rescale `A`'s bare-`M` velocity rows by
/// `M/M_impl`, geom-stiff over `M_impl`, position rows from `θ' = θ + Δt·ω'`) reconciles the
/// unloaded `A` with the real eulerdamp step. A nonzero `qvel` exercises the damping term.
#[test]
fn analytic_state_jacobian_damped_matches_fd() {
    const DAMPED_HINGE: &str = r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
<body name="arm" pos="0 0 0.2">
  <joint type="hinge" axis="0 1 0" damping="0.7"/>
  <geom type="sphere" pos="0 0 -0.095" size="0.004" mass="0.2"/>
</body>
  </worldbody>
</mujoco>"#;
    let model = load_model(DAMPED_HINGE).expect("damped hinge loads");
    let mut data = model.make_data();
    data.qpos[0] = 0.3;
    data.qvel[0] = 0.4; // nonzero velocity so the implicit-damping term is exercised
    data.forward(&model).expect("forward");
    assert!(model.implicit_damping[0] > 0.0, "damping must be live");
    let c: StaggeredCoupling = StaggeredCoupling::new(
        model, data, 1, 0.005, 4, 0.1, 3.0e4, 1.0e-3, 3.0e4, 1.0e-2, 0.0,
    );
    assert!(
        c.single_hinge().is_some(),
        "geometry is a single hinge (the analytic path's predicate)"
    );
    let wrench = SpatialVector::from_row_slice(&[0.1, 0.2, 0.0, 5.0, -3.0, 800.0]);
    let analytic = c
        .analytic_state_jacobian(&wrench)
        .expect("a damped single hinge now has an analytic J_state");
    let fd = c.loaded_state_jacobian(&wrench);
    let rel = (&analytic - &fd).norm() / fd.norm().max(1e-30);
    eprintln!("damped analytic J_state vs FD: rel={rel:.3e}");
    assert!(
        rel < 1e-6,
        "damped analytic J_state must match the FD loaded Jacobian, got rel {rel:.3e}"
    );
}

/// The analytic UNDAMPED hinge **chain** `J_state`
/// ([`StaggeredCoupling::chain_state_jacobian`]) is MACHINE-EXACT against the FD
/// [`StaggeredCoupling::loaded_state_jacobian`] for arbitrary held wrenches (force AND
/// torque, nonzero per-joint `qvel`). Run on a planar (parallel-axis) 2-link, a SPATIAL
/// (non-parallel-axis, off-axis COM) 2-link, and a SPATIAL 3-link — the spatial cases
/// exercise the off-diagonal kinematics and depend on the sim-core Coriolis fixes (the
/// `∂S/∂q` ancestor term + the multi-hop bias-acceleration X_b transport) for their unloaded
/// `A`; the 3-link additionally exercises multi-hop ancestors. Pins the full chain assembly:
/// the case-split geometric-stiffness Hessian `G`, the mass-directional derivative `dMu` (the
/// `∂M⁻¹/∂q` term that vanishes for a single hinge), and the `A + Δt·M⁻¹·(G − dMu)`
/// decomposition on both the velocity and semi-implicit position rows. See
/// `docs/keystone/multilink_recon.md`.
#[test]
fn chain_state_jacobian_matches_fd_loaded() {
    const PLANAR_MJCF: &str = r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
<body name="upper" pos="0 0 0.2">
  <joint type="hinge" axis="0 1 0"/>
  <geom type="sphere" pos="0 0 -0.025" size="0.004" mass="0.3"/>
  <body name="lower" pos="0 0 -0.05">
    <joint type="hinge" axis="0 1 0"/>
    <geom type="sphere" pos="0 0 -0.04" size="0.004" mass="0.4"/>
  </body>
</body>
  </worldbody>
</mujoco>"#;
    const SPATIAL_MJCF: &str = r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
<body name="upper" pos="0 0 0.2">
  <joint type="hinge" axis="0 1 0"/>
  <geom type="sphere" pos="0.01 0 -0.025" size="0.004" mass="0.3"/>
  <body name="lower" pos="0.02 0 -0.05">
    <joint type="hinge" axis="1 0.3 0"/>
    <geom type="sphere" pos="0.015 0.01 -0.04" size="0.004" mass="0.4"/>
  </body>
</body>
  </worldbody>
</mujoco>"#;
    // 3-link, all axes non-parallel → multi-hop strict ancestors (joint 0 is an ancestor
    // of joint 2 two links up).
    const THREELINK_MJCF: &str = r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
<body name="upper" pos="0 0 0.2">
  <joint type="hinge" axis="0 1 0"/>
  <geom type="sphere" pos="0.01 0 -0.025" size="0.004" mass="0.3"/>
  <body name="mid" pos="0.02 0 -0.05">
    <joint type="hinge" axis="1 0.3 0"/>
    <geom type="sphere" pos="0.015 0.01 -0.03" size="0.004" mass="0.35"/>
    <body name="lower" pos="0.01 0.005 -0.05">
      <joint type="hinge" axis="0.2 1 0.1"/>
      <geom type="sphere" pos="0.012 0.008 -0.03" size="0.004" mass="0.4"/>
    </body>
  </body>
</body>
  </worldbody>
</mujoco>"#;

    // (mjcf, contact body = tip, nv). Planar + spatial 2-link, and spatial 3-link (multi-hop).
    for (mjcf, body, nv) in [
        (PLANAR_MJCF, 2, 2),
        (SPATIAL_MJCF, 2, 2),
        (THREELINK_MJCF, 3, 3),
    ] {
        let model = load_model(mjcf).expect("chain MJCF loads");
        let mut data = model.make_data();
        let q0 = [0.2, -0.15, 0.1];
        let v0 = [-1.5, 0.8, -1.1]; // nonzero velocities exercise the full transition (Coriolis)
        for i in 0..nv {
            data.qpos[i] = q0[i];
            data.qvel[i] = v0[i];
        }
        data.forward(&model).expect("forward");
        let c: StaggeredCoupling = StaggeredCoupling::new(
            model, data, body, 0.005, 4, 0.1, 3.0e4, 1.0e-3, 3.0e4, 1.0e-2, 0.0,
        );
        assert!(
            c.single_hinge().is_none(),
            "a multi-link chain must NOT take the single-hinge path"
        );

        for wrench in [
            SpatialVector::from_row_slice(&[0.1, 0.2, 0.0, 5.0, -3.0, 800.0]),
            SpatialVector::from_row_slice(&[0.0, 0.0, 0.0, 0.0, 0.0, 600.0]),
            SpatialVector::from_row_slice(&[0.3, -0.4, 0.2, -10.0, 7.0, -250.0]),
        ] {
            let loaded = c.loaded_state_jacobian(&wrench);
            let analytic = c
                .analytic_state_jacobian(&wrench)
                .expect("multi-link chain → analytic chain J_state");
            // Tolerance is the FD REFERENCE's own central-difference floor (~1e-6, looser
            // on the chain's velocity-velocity block); the analytic form is exact — the
            // end-to-end gradient gate validates machine-exactness against re-rolled FD.
            let (err, loc) = sim_core::max_relative_error(&loaded, &analytic, 1e-3);
            assert!(
                err < 1e-5,
                "analytic chain J_state (nv={nv}) must match FD loaded to FD precision, \
                 got rel {err:.3e} at {loc:?}"
            );
        }
    }
}

/// A DAMPED serial hinge chain's analytic `J_state` matches the FD
/// [`StaggeredCoupling::loaded_state_jacobian`] — the multi-link analogue of
/// [`analytic_state_jacobian_damped_matches_fd`]. The loaded term routes through
/// `M_impl = M + Δt·D`; the unloaded `A` is eulerdamp-correct from sim-core. Covers 2-link
/// (spatial) and 3-link (multi-hop) chains at several damping levels; `damp = 0` confirms the
/// damped code path collapses to the bare-M result.
#[test]
fn chain_state_jacobian_damped_matches_fd() {
    // Spatial 2-link and multi-hop 3-link, joints carrying `damping` (interpolated below).
    let mk = |damp: f64| -> [(String, usize, usize); 2] {
        let two = format!(
            r#"<mujoco><option gravity="0 0 -9.81" timestep="0.001"/><worldbody>
<body name="upper" pos="0 0 0.2"><joint type="hinge" axis="0 1 0" damping="{damp}"/>
<geom type="sphere" pos="0.01 0 -0.025" size="0.004" mass="0.3"/>
<body name="lower" pos="0.02 0 -0.05"><joint type="hinge" axis="1 0.3 0" damping="{damp}"/>
<geom type="sphere" pos="0.015 0.01 -0.04" size="0.004" mass="0.4"/></body></body></worldbody></mujoco>"#
        );
        let three = format!(
            r#"<mujoco><option gravity="0 0 -9.81" timestep="0.001"/><worldbody>
<body name="upper" pos="0 0 0.2"><joint type="hinge" axis="0 1 0" damping="{damp}"/>
<geom type="sphere" pos="0.01 0 -0.025" size="0.004" mass="0.3"/>
<body name="mid" pos="0.02 0 -0.05"><joint type="hinge" axis="1 0.3 0" damping="{damp}"/>
<geom type="sphere" pos="0.015 0.01 -0.03" size="0.004" mass="0.35"/>
<body name="lower" pos="0.01 0.005 -0.05"><joint type="hinge" axis="0.2 1 0.1" damping="{damp}"/>
<geom type="sphere" pos="0.012 0.008 -0.03" size="0.004" mass="0.4"/></body></body></body></worldbody></mujoco>"#
        );
        [(two, 2, 2), (three, 3, 3)]
    };
    for &damp in &[0.0_f64, 0.5, 2.0] {
        for (mjcf, body, nv) in mk(damp) {
            let model = load_model(&mjcf).expect("damped chain MJCF loads");
            let mut data = model.make_data();
            let q0 = [0.2, -0.15, 0.1];
            let v0 = [-1.5, 0.8, -1.1]; // nonzero ⇒ exercises Coriolis
            for i in 0..nv {
                data.qpos[i] = q0[i];
                data.qvel[i] = v0[i];
            }
            data.forward(&model).expect("forward");
            let c: StaggeredCoupling = StaggeredCoupling::new(
                model, data, body, 0.005, 4, 0.1, 3.0e4, 1.0e-3, 3.0e4, 1.0e-2, 0.0,
            );
            let wrench = SpatialVector::from_row_slice(&[0.3, -0.4, 0.2, -10.0, 7.0, -250.0]);
            let loaded = c.loaded_state_jacobian(&wrench);
            let analytic = c
                .analytic_state_jacobian(&wrench)
                .expect("damped multi-link chain → analytic chain J_state");
            let (err, loc) = sim_core::max_relative_error(&loaded, &analytic, 1e-3);
            assert!(
                err < 1e-5,
                "damped analytic chain J_state (nv={nv}, damp={damp}) must match FD loaded, \
                 got rel {err:.3e} at {loc:?}"
            );
        }
    }
}

/// The tangent FD `loaded_state_jacobian` for a ball joint (`nq = 4, nv = 3`) is
/// SO(3)-correct: the position-VELOCITY block `∂(tangent qpos')/∂qvel` is the
/// integrator's `≈ Δt·I` (NOT zero — the bug the lossy `sqrt(1−w²)+acos` log map in
/// `mj_differentiate_pos` produced when a tiny FD rotation's `w` rounded to 1; fixed
/// to the vector-norm + atan2 form), and the position-VELOCITY and velocity-rows
/// match sim-core's analytic transition `A`. (The position-POSITION block legitimately
/// differs from `A`: this carry measures the output tangent at the nominal `qpos'`
/// (`mj_differentiate_pos`, the convention `jz`/`J_r` consume), whereas
/// `transition_derivatives` references it at `q_old` — a different, internally
/// consistent tangent. The end-to-end flat `ball·material[μ]` trajectory row of
/// `tests/coupling_grad_harness.rs` validates the full convention-consistent composition.)
#[test]
fn loaded_state_jacobian_ball_velocity_block_matches_analytic() {
    const BALL_MJCF: &str = r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
<body name="arm" pos="0 0 0.2">
  <joint type="ball"/>
  <geom type="sphere" pos="0 0 -0.095" size="0.004" mass="0.2"/>
</body>
  </worldbody>
</mujoco>"#;
    let model = load_model(BALL_MJCF).expect("ball MJCF loads");
    let mut data = model.make_data();
    let half = 0.15_f64;
    data.qpos[0] = half.cos();
    data.qpos[2] = half.sin();
    data.qvel[1] = -1.5; // nonzero ang vel exercises the integrator coupling
    data.forward(&model).expect("forward");
    let c: StaggeredCoupling = StaggeredCoupling::new(
        model, data, 1, 0.005, 4, 0.1, 3.0e4, 1.0e-3, 3.0e4, 1.0e-2, 0.0,
    );
    // wrench = 0 ⇒ the FD loaded Jacobian is the unloaded transition. Two checks:
    //  (1) the velocity rows (`∂qvel'/∂·`, convention-free — qvel' has no quaternion)
    //      match the validated analytic transition `A`; and
    //  (2) the position-velocity block's DIAGONAL is ≈ Δt — the integrator's `J_r ≈ I`.
    //      This is the atan2-log-map regression guard: the lossy `sqrt(1−w²)+acos` form
    //      returned ZERO here (a tiny FD rotation's `w` rounds to 1). The off-diagonals
    //      are O(Δt²) and convention-dependent (see the doc), so are not compared.
    let loaded = c.loaded_state_jacobian(&SpatialVector::zeros());
    let a = c
        .data
        .transition_derivatives(&c.model, &sim_core::DerivativeConfig::default())
        .expect("transition derivatives")
        .A;
    let nv = 3;
    let dt = 1.0e-3;
    for r in nv..2 * nv {
        for col in 0..2 * nv {
            let (lv, av) = (loaded[(r, col)], a[(r, col)]);
            let rel = (lv - av).abs() / av.abs().max(1e-6);
            assert!(
                rel < 1e-4,
                "ball velocity row J_state[{r},{col}] = {lv:.6e} != A {av:.6e}"
            );
        }
    }
    for i in 0..nv {
        let d = loaded[(i, nv + i)];
        assert!(
            (d - dt).abs() < 1e-5,
            "ball position-velocity diagonal J_state[{i},{}] = {d:.6e} must be ≈Δt={dt:.0e} \
             (atan2 log-map fix — was ZERO with the lossy sqrt(1−w²)+acos form)",
            nv + i
        );
    }
}

/// `ContactWrenchTrajVjp`'s analytic Jacobian (the contact-MOMENT leaf's core
/// new math — `∂w/∂x*` with the moment's explicit-`rᵢ` + via-`gᵢ` parts, `∂w/∂h`
/// the force/moment-vs-height feedback, and `∂w/∂s` the moment's `c(q)` feedback)
/// is FD-EXACT against the REAL contact readout at a deeply-engaged off-COM hinge
/// config (the tip COM `xipos` offset ~0.08 m from the block-top contact
/// centroid, so the moment is large). This pins the wrench node independently of
/// the multi-step composition (the trajectory gate validates the composition).
#[test]
// similar_names: paired gradient-probe locals (analytic vs FD, per-param) read clearly in the
// derivative math here; renaming would obscure the intended pairing.
#[allow(clippy::similar_names)]
fn contact_wrench_node_matches_readout_fd() {
    const HINGE_MJCF: &str = r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
<body name="arm" pos="0 0 0.2">
  <joint type="hinge" axis="0 1 0"/>
  <geom type="sphere" pos="0 0 -0.095" size="0.004" mass="0.2"/>
</body>
  </worldbody>
</mujoco>"#;
    let model = load_model(HINGE_MJCF).expect("hinge MJCF loads");
    let mut data = model.make_data();
    data.qpos[0] = 0.3; // tilt → off-COM contact (large moment)
    data.forward(&model).expect("forward");
    let c: StaggeredCoupling = StaggeredCoupling::new(
        model, data, 1, 0.005, 4, 0.1, 3.0e4, 1.0e-3, 3.0e4, 1.0e-2, 0.0,
    );

    let height = c.tip_plane_height();
    let positions = c.positions();
    let com = c.data.xipos[c.body];
    let n_dof = 3 * c.n_vertices;
    let nv = c.model.nv;

    // The REAL reaction wrench from the contact readout (recomputed g + r).
    let wrench_of = |h: f64, pos: &[Vec3], cc: Vec3| -> [f64; 6] {
        let mut w = [0.0_f64; 6];
        for (_, g, _, _, r) in c.active_pair_wrench_data(h, pos) {
            let f = -g;
            w[3] += f.x;
            w[4] += f.y;
            w[5] += f.z;
            let tau = (r - cc).cross(&f);
            w[0] += tau.x;
            w[1] += tau.y;
            w[2] += tau.z;
        }
        w
    };

    // The analytic node at this config.
    let mut active = Vec::new();
    let mut force = Vec3::zeros();
    for (v, g, n, curv, r) in c.active_pair_wrench_data(height, &positions) {
        force += -g;
        active.push((v, g, n, curv, r - com, c.collider_hessian(height, r)));
    }
    assert!(!active.is_empty(), "config must be contact-engaged");
    let node = ContactWrenchTrajVjp {
        active,
        force,
        jlin: c.com_linear_jacobian(),
        n_dof,
        nv,
        pose: WrenchPose::Height,
    };
    // Row k of the Jacobian = the parent-cotangents for the unit cotangent e_k.
    let jac_row = |k: usize| -> (Vec<f64>, f64, Vec<f64>) {
        let mut cot = Tensor::zeros(&[6]);
        cot.as_mut_slice()[k] = 1.0;
        let mut pc = vec![
            Tensor::zeros(&[n_dof]),
            Tensor::zeros(&[1]),
            Tensor::zeros(&[2 * nv]),
        ];
        node.vjp(&cot, &mut pc);
        (
            pc[0].as_slice().to_vec(),
            pc[1].as_slice()[0],
            pc[2].as_slice().to_vec(),
        )
    };
    let rows: Vec<_> = (0..6).map(jac_row).collect();
    let d = 1.0e-7;

    let mut worst = 0.0_f64; // max |fd − analytic| / scale over all checked entries
    let mut check = |fd: f64, an: f64, scale: f64| {
        let rel = (fd - an).abs() / scale.max(1.0);
        worst = worst.max(rel);
    };
    // Scales per channel (the dominant Jacobian magnitudes seen in this scene).
    let sx = 1.0e3;
    let sh = 1.0e5;
    let sq = 1.0e3;

    // ∂w/∂x* (each soft DOF) — the stable engaged active set holds across ±d.
    for j in 0..n_dof {
        let mut pp = positions.clone();
        let mut pm = positions.clone();
        pp[j / 3][j % 3] += d;
        pm[j / 3][j % 3] -= d;
        let wp = wrench_of(height, &pp, com);
        let wm = wrench_of(height, &pm, com);
        for k in 0..6 {
            check((wp[k] - wm[k]) / (2.0 * d), rows[k].0[j], sx);
        }
    }
    // ∂w/∂h (plane height).
    let wp = wrench_of(height + d, &positions, com);
    let wm = wrench_of(height - d, &positions, com);
    for k in 0..6 {
        check((wp[k] - wm[k]) / (2.0 * d), rows[k].1, sh);
    }
    // ∂w/∂qpos (the moment's c(q) feedback — perturb xipos via qpos, height held).
    for jq in 0..nv {
        let scratch_com = |dq: f64| -> Vec3 {
            let mut s = c.model.make_data();
            s.qpos.copy_from(&c.data.qpos);
            s.qpos[jq] += dq;
            s.forward(&c.model).expect("forward");
            s.xipos[c.body]
        };
        let wp = wrench_of(height, &positions, scratch_com(d));
        let wm = wrench_of(height, &positions, scratch_com(-d));
        for k in 0..6 {
            check((wp[k] - wm[k]) / (2.0 * d), rows[k].2[jq], sq);
        }
    }
    assert!(
        worst < 1e-5,
        "ContactWrenchTrajVjp Jacobian must be FD-exact vs the real readout, \
         worst scaled error {worst:.3e}"
    );
}

/// The finite-sphere centre defaults to the block-face centroid (byte-identical to the
/// pre-end-effector posing) and follows the `sphere_center_override` when set — the forward
/// end-effector posing primitive.
#[test]
fn sphere_center_default_and_override() {
    const MJCF: &str = r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
<body name="platen" pos="0 0 0.125">
  <joint type="slide" axis="0 0 1"/>
  <geom type="box" size="0.05 0.05 0.02" mass="1"/>
</body>
  </worldbody>
</mujoco>"#;
    const SPHERE_R: f64 = 0.08;
    const EDGE: f64 = 0.1;
    let model = load_model(MJCF).expect("MJCF loads");
    let mut data = model.make_data();
    data.forward(&model).expect("forward");
    let mut c: StaggeredCoupling = StaggeredCoupling::new(
        model, data, 1, 0.005, 4, EDGE, 3.0e4, 1.0e-3, 3.0e4, 1.0e-2, 0.0,
    )
    .with_sphere_collider(SPHERE_R);

    let height = 0.09_f64;
    // Default = block top-face centroid; `z = height + radius`.
    assert_eq!(
        c.sphere_center(SPHERE_R, height),
        Vec3::new(EDGE / 2.0, EDGE / 2.0, height + SPHERE_R),
    );

    // Override = the full end-effector centre, `height`-independent.
    let ee = Vec3::new(0.02, 0.03, 0.12);
    c.sphere_center_override = Some(ee);
    assert_eq!(c.sphere_center(SPHERE_R, height), ee);
}

/// `step_articulated` + `with_contact_geom` poses the finite fist at the rigid geom's world
/// centre each frame (tracking the arm tip), engages the soft block, and routes the off-COM
/// reaction wrench — the per-frame forward stepper the striker viewers drive.
#[test]
fn step_articulated_poses_at_contact_geom() {
    // Slide arm directly over the block centre (xy = edge/2): the fist geom hangs `0.13` below
    // the body origin at `z = 0.25`, so its centre sits at `z ≈ 0.12` and its south pole
    // (`0.12 − 0.04 = 0.08`) penetrates the rest top face (`z = 0.1`).
    const MJCF: &str = r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
<body name="arm" pos="0.05 0.05 0.25">
  <joint type="slide" axis="0 0 1"/>
  <geom name="fist" type="sphere" pos="0 0 -0.13" size="0.04" mass="0.5"/>
</body>
  </worldbody>
</mujoco>"#;
    const FIST_R: f64 = 0.04;
    let model = load_model(MJCF).expect("MJCF loads");
    let fist_gid = model.geom_id("fist").expect("fist geom");
    let mut data = model.make_data();
    data.forward(&model).expect("forward");
    // The fist's world centre at the (unchanged) start config — what `step_articulated`'s
    // leading FK poses the contact sphere at. (Captured before `data` moves into the coupling;
    // the step's trailing FK then advances the live pose.)
    let fist0 = data.geom_xpos[fist_gid];
    let mut c: StaggeredCoupling = StaggeredCoupling::new(
        model, data, 1, 0.005, 4, 0.1, 3.0e4, 1.0e-3, 3.0e4, 1.0e-2, 0.0,
    )
    .with_sphere_collider(FIST_R)
    .with_contact_geom(fist_gid);

    let readout = c.step_articulated();
    // The fist posed at the geom's world centre (not the block centroid).
    assert_eq!(c.sphere_center_override, Some(fist0));
    // Engaged: a finite upward contact force on the soft block.
    assert!(readout.force_on_soft.z.is_finite());
    assert!(
        readout.force_on_soft.norm() > 1e-6,
        "fist over the block should engage the soft contact",
    );
}

/// `step_kinematic` + `set_sphere_center` pose the finite sphere at a scripted world point and
/// take ONE soft step (no rigid integration) — the kinematic striker path. The fist over the
/// block engages and dimples the soft body; no dynamic rigid body is needed (a static anchor
/// just hosts the coupling).
#[test]
fn step_kinematic_poses_at_set_center_and_engages() {
    const MJCF: &str = r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
<body name="anchor">
  <geom type="sphere" size="0.01"/>
</body>
  </worldbody>
</mujoco>"#;
    const FIST_R: f64 = 0.04;
    const EDGE: f64 = 0.1;
    let model = load_model(MJCF).expect("MJCF loads");
    let mut data = model.make_data();
    data.forward(&model).expect("forward");
    let mut c: StaggeredCoupling = StaggeredCoupling::new(
        model, data, 1, 0.005, 4, EDGE, 3.0e4, 1.0e-3, 3.0e4, 1.0e-2, 0.0,
    )
    .with_sphere_collider(FIST_R);

    // Pose the fist over the block centre, south pole pressing into the rest top face (z = EDGE).
    let center = Vec3::new(EDGE / 2.0, EDGE / 2.0, EDGE + FIST_R - 0.01);
    c.set_sphere_center(center);
    let rest = c.soft_positions().to_vec();
    let readout = c.step_kinematic();

    assert_eq!(c.sphere_center_override, Some(center));
    assert!(
        readout.force_on_soft.norm() > 1e-6,
        "fist over the block should engage the soft contact",
    );
    let moved = c
        .soft_positions()
        .iter()
        .zip(&rest)
        .any(|(a, b)| (a - b).abs() > 1e-9);
    assert!(moved, "kinematic step should deform the soft body");
}

/// L1b articulated NORMAL wrench curvature: the contact-wrench node on a FINITE sphere
/// collider is FD-exact against the real readout. Same engaged off-COM hinge config as
/// `contact_wrench_node_matches_readout_fd`, but with `with_sphere_collider` — the sphere's
/// contact normal turns as a soft vertex slides (`∂n̂/∂x = H`) and as the primitive
/// translates with the tip height (`∂n̂/∂h = −H·ẑ`), so `∂w/∂x*` and `∂w/∂h` carry the
/// geometric-stiffness term `f_mag·H` (zero for the plane's constant normal). A flat
/// `ContactWrenchTrajVjp` misses it and disagrees with the curved-readout FD at the
/// curvature scale.
#[test]
// similar_names: paired gradient-probe locals (analytic vs FD, per-param) read clearly in the
// derivative math here; renaming would obscure the intended pairing.
#[allow(clippy::similar_names)]
fn sphere_contact_wrench_node_matches_readout_fd() {
    const HINGE_MJCF: &str = r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
<body name="arm" pos="0 0 0.2">
  <joint type="hinge" axis="0 1 0"/>
  <geom type="sphere" pos="0 0 -0.095" size="0.004" mass="0.2"/>
</body>
  </worldbody>
</mujoco>"#;
    // Sphere radius: large vs the 0.1 m block so the south-pole patch spans several top-face
    // vertices yet stays curved enough that f_mag·H is materially nonzero.
    const SPHERE_R: f64 = 0.08;
    let model = load_model(HINGE_MJCF).expect("hinge MJCF loads");
    let mut data = model.make_data();
    data.qpos[0] = 0.3; // tilt → off-COM contact (large moment) + engaged sphere
    data.forward(&model).expect("forward");
    let c: StaggeredCoupling = StaggeredCoupling::new(
        model, data, 1, 0.005, 4, 0.1, 3.0e4, 1.0e-3, 3.0e4, 1.0e-2, 0.0,
    )
    .with_sphere_collider(SPHERE_R);

    let height = c.tip_plane_height();
    let positions = c.positions();
    let com = c.data.xipos[c.body];
    let n_dof = 3 * c.n_vertices;
    let nv = c.model.nv;

    // The REAL reaction wrench from the curved-collider readout (recomputed g + r).
    let wrench_of = |h: f64, pos: &[Vec3], cc: Vec3| -> [f64; 6] {
        let mut w = [0.0_f64; 6];
        for (_, g, _, _, r) in c.active_pair_wrench_data(h, pos) {
            let f = -g;
            w[3] += f.x;
            w[4] += f.y;
            w[5] += f.z;
            let tau = (r - cc).cross(&f);
            w[0] += tau.x;
            w[1] += tau.y;
            w[2] += tau.z;
        }
        w
    };

    // The analytic node at this config (the curved H comes from `active_pair_wrench_curv`).
    let mut active = Vec::new();
    let mut force = Vec3::zeros();
    for (v, g, n, curv, r) in c.active_pair_wrench_data(height, &positions) {
        force += -g;
        active.push((
            v,
            g,
            n,
            curv,
            r - com,
            c.collider_hessian(height, positions[v]),
        ));
    }
    assert!(!active.is_empty(), "config must be contact-engaged");
    let node = ContactWrenchTrajVjp {
        active,
        force,
        jlin: c.com_linear_jacobian(),
        n_dof,
        nv,
        pose: WrenchPose::Height,
    };
    let jac_row = |k: usize| -> (Vec<f64>, f64, Vec<f64>) {
        let mut cot = Tensor::zeros(&[6]);
        cot.as_mut_slice()[k] = 1.0;
        let mut pc = vec![
            Tensor::zeros(&[n_dof]),
            Tensor::zeros(&[1]),
            Tensor::zeros(&[2 * nv]),
        ];
        node.vjp(&cot, &mut pc);
        (
            pc[0].as_slice().to_vec(),
            pc[1].as_slice()[0],
            pc[2].as_slice().to_vec(),
        )
    };
    let rows: Vec<_> = (0..6).map(jac_row).collect();
    let d = 1.0e-7;

    let mut worst = 0.0_f64;
    let mut check = |fd: f64, an: f64, scale: f64| {
        let rel = (fd - an).abs() / scale.max(1.0);
        worst = worst.max(rel);
    };
    let sx = 1.0e3;
    let sh = 1.0e5;
    let sq = 1.0e3;

    for j in 0..n_dof {
        let mut pp = positions.clone();
        let mut pm = positions.clone();
        pp[j / 3][j % 3] += d;
        pm[j / 3][j % 3] -= d;
        let wp = wrench_of(height, &pp, com);
        let wm = wrench_of(height, &pm, com);
        for k in 0..6 {
            check((wp[k] - wm[k]) / (2.0 * d), rows[k].0[j], sx);
        }
    }
    let wp = wrench_of(height + d, &positions, com);
    let wm = wrench_of(height - d, &positions, com);
    for k in 0..6 {
        check((wp[k] - wm[k]) / (2.0 * d), rows[k].1, sh);
    }
    for jq in 0..nv {
        let scratch_com = |dq: f64| -> Vec3 {
            let mut s = c.model.make_data();
            s.qpos.copy_from(&c.data.qpos);
            s.qpos[jq] += dq;
            s.forward(&c.model).expect("forward");
            s.xipos[c.body]
        };
        let wp = wrench_of(height, &positions, scratch_com(d));
        let wm = wrench_of(height, &positions, scratch_com(-d));
        for k in 0..6 {
            check((wp[k] - wm[k]) / (2.0 * d), rows[k].2[jq], sq);
        }
    }
    assert!(
        worst < 1e-5,
        "sphere ContactWrenchTrajVjp Jacobian must be FD-exact vs the curved readout, \
         worst scaled error {worst:.3e}"
    );
}

/// The CENTRE-channel companion to `sphere_contact_wrench_node_matches_readout_fd`:
/// the moving-end-effector sphere is posed at an explicit centre OVERRIDE (the arm tip),
/// and the wrench node's pose parent is the 3-vector centre channel ([`WrenchPose::Centre`])
/// rather than the scalar height. FD: translate the centre along each axis, recompute the
/// curved-collider wrench readout (soft positions held), compare to the analytic per-axis
/// pose Jacobian. The z axis reproduces the `Height` channel; x/y are the lateral channels
/// the moving-EE carry needs (the `f_mag·H` normal-rotation term is materially nonzero on
/// the off-pole curved patch the off-centroid centre selects).
#[test]
// similar_names: paired gradient-probe locals (analytic vs FD, per-param) read clearly in the
// derivative math here; renaming would obscure the intended pairing.
#[allow(clippy::similar_names)]
fn sphere_contact_wrench_node_centre_matches_readout_fd() {
    const HINGE_MJCF: &str = r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
<body name="arm" pos="0 0 0.2">
  <joint type="hinge" axis="0 1 0"/>
  <geom type="sphere" pos="0 0 -0.095" size="0.004" mass="0.2"/>
</body>
  </worldbody>
</mujoco>"#;
    const SPHERE_R: f64 = 0.08;
    let model = load_model(HINGE_MJCF).expect("hinge MJCF loads");
    let mut data = model.make_data();
    data.qpos[0] = 0.3; // tilt → off-COM contact + engaged sphere
    data.forward(&model).expect("forward");
    let mut c: StaggeredCoupling = StaggeredCoupling::new(
        model, data, 1, 0.005, 4, 0.1, 3.0e4, 1.0e-3, 3.0e4, 1.0e-2, 0.0,
    )
    .with_sphere_collider(SPHERE_R);

    let height = c.tip_plane_height();
    // Pose the fist at an explicit centre (the moving-EE scenario), OFF the block centroid
    // so n̂·x̂ and n̂·ŷ are nonzero on the curved patch ⇒ the lateral channels are exercised.
    let base = Vec3::new(
        c.edge / 2.0 + 0.012,
        c.edge / 2.0 - 0.009,
        height + SPHERE_R,
    );
    c.set_sphere_center(base);
    let positions = c.positions();
    let com = c.data.xipos[c.body];
    let n_dof = 3 * c.n_vertices;
    let nv = c.model.nv;

    // The curved-collider wrench readout at the current sphere-centre override, soft
    // positions held fixed (the explicit ∂w/∂centre the pose parent carries).
    let wrench_of = |c: &StaggeredCoupling, pos: &[Vec3]| -> [f64; 6] {
        let mut w = [0.0_f64; 6];
        for (_, g, _, _, r) in c.active_pair_wrench_data(height, pos) {
            let f = -g;
            w[3] += f.x;
            w[4] += f.y;
            w[5] += f.z;
            let tau = (r - com).cross(&f);
            w[0] += tau.x;
            w[1] += tau.y;
            w[2] += tau.z;
        }
        w
    };

    // Analytic node at the base centre (curv + H read here, before the FD mutates the override).
    let mut active = Vec::new();
    let mut force = Vec3::zeros();
    for (v, g, n, curv, r) in c.active_pair_wrench_data(height, &positions) {
        force += -g;
        active.push((
            v,
            g,
            n,
            curv,
            r - com,
            c.collider_hessian(height, positions[v]),
        ));
    }
    assert!(!active.is_empty(), "config must be contact-engaged");
    let basis = vec![
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
    ];
    let node = ContactWrenchTrajVjp {
        active,
        force,
        jlin: c.com_linear_jacobian(),
        n_dof,
        nv,
        pose: WrenchPose::Centre { basis },
    };
    // Pose-parent cotangent rows: ∂w_k/∂(centre axis), one [3] per wrench component k.
    let pose_rows: Vec<[f64; 3]> = (0..6)
        .map(|k| {
            let mut cot = Tensor::zeros(&[6]);
            cot.as_mut_slice()[k] = 1.0;
            let mut pc = vec![
                Tensor::zeros(&[n_dof]),
                Tensor::zeros(&[3]),
                Tensor::zeros(&[2 * nv]),
            ];
            node.vjp(&cot, &mut pc);
            let s = pc[1].as_slice();
            [s[0], s[1], s[2]]
        })
        .collect();

    let d = 1.0e-7;
    // The centre channel is a translation like the height channel ⇒ the same 1e5 scale
    // the height gate calibrated (the z axis IS that channel; x/y are smaller).
    let scale = 1.0e5;
    let mut worst = 0.0_f64;
    for axis in 0..3 {
        let mut dir = Vec3::zeros();
        dir[axis] = 1.0;
        c.set_sphere_center(base + d * dir);
        let wp = wrench_of(&c, &positions);
        c.set_sphere_center(base - d * dir);
        let wm = wrench_of(&c, &positions);
        for k in 0..6 {
            let fd = (wp[k] - wm[k]) / (2.0 * d);
            worst = worst.max((fd - pose_rows[k][axis]).abs() / scale);
        }
    }
    assert!(
        worst < 1e-5,
        "sphere ContactWrenchTrajVjp CENTRE Jacobian must be FD-exact vs the curved readout, \
         worst scaled error {worst:.3e}"
    );
}

/// The FRICTION wrench node's 3-vector CENTRE pose channel is FD-exact — the friction analog of
/// `sphere_contact_wrench_node_centre_matches_readout_fd`, covering the L1 friction-pose WIRING
/// (the `[axis][vertex] → [vertex][axis]` `dforce_dpose` transpose + `n_pose` + the node's
/// `pose_slot` loop) that the (pose-INSENSITIVE) friction trajectory gate cannot reach. FD:
/// perturb the sphere centre per axis (soft x* held), recompute the friction wrench
/// `Σ[∇D_v; (r_v−c)×∇D_v]`, and compare to the node's pose-parent cotangent rows.
#[test]
// similar_names: paired gradient-probe locals (analytic vs FD, per-param) read clearly in the
// derivative math here; renaming would obscure the intended pairing.
#[allow(clippy::similar_names)]
fn friction_wrench_node_centre_matches_readout_fd() {
    const HINGE_MJCF: &str = r#"<mujoco>
  <option gravity="1.0 0 -9.81" timestep="0.001"/>
  <worldbody>
<body name="arm" pos="0 0 0.2">
  <joint type="hinge" axis="0 1 0"/>
  <geom type="sphere" pos="0 0 -0.095" size="0.004" mass="0.2"/>
</body>
  </worldbody>
</mujoco>"#;
    const SPHERE_R: f64 = 0.08;
    let model = load_model(HINGE_MJCF).expect("hinge MJCF loads");
    let mut data = model.make_data();
    data.qpos[0] = 0.3;
    data.forward(&model).expect("forward");
    let mut c: StaggeredCoupling = StaggeredCoupling::new(
        model, data, 1, 0.005, 4, 0.1, 3.0e3, 1.2e4, 3.0e4, 1.0e-2, 0.0,
    )
    .with_sphere_collider(SPHERE_R)
    .with_friction(2.5, 0.1);

    let height = c.tip_plane_height();
    let dt = c.cfg.dt;
    let n = c.n_vertices;
    let nv = c.model.nv;
    let drift = Vec3::new(5.0e-4, 0.0, 0.0);
    let drift_dir = Vec3::new(1.0, 0.0, 0.0);
    let com = c.data.xipos[c.body];
    let x_start = c.x.clone();
    // Off-centroid centre (the moving-EE scenario) so the lateral axes are exercised.
    let base = Vec3::new(c.edge / 2.0 + 0.01, c.edge / 2.0 - 0.008, height + SPHERE_R);
    let basis = [
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
    ];

    // One friction soft solve at the base centre (x* held fixed for the explicit-pose FD).
    c.sphere_center_override = Some(base);
    let x_next = {
        let bc = BoundaryConditions::new(c.pinned.clone(), Vec::new());
        let solver: SoftSolver<_> =
            CpuNewtonSolver::new(Tet4, c.fresh_mesh(), c.build_contact(height), c.cfg, bc)
                .with_friction_surface_drift(drift);
        solver
            .replay_step(
                &Tensor::from_slice(&c.x, &[3 * n]),
                &Tensor::from_slice(&c.v, &[3 * n]),
                &Tensor::zeros(&[0]),
                dt,
            )
            .x_final
    };
    let positions: Vec<Vec3> = x_next
        .chunks_exact(3)
        .map(|q| Vec3::new(q[0], q[1], q[2]))
        .collect();

    // Friction wrench readout Σ[∇D_v; (r_v−c)×∇D_v] at sphere centre `centre`, x* held.
    let wrench_of = |c: &mut StaggeredCoupling, centre: Vec3| -> [f64; 6] {
        c.sphere_center_override = Some(centre);
        let bc = BoundaryConditions::new(c.pinned.clone(), Vec::new());
        let solver: SoftSolver<_> =
            CpuNewtonSolver::new(Tet4, c.fresh_mesh(), c.build_contact(height), c.cfg, bc)
                .with_friction_surface_drift(drift);
        let pv = solver.friction_force_jacobians(&x_next, &x_start, dt, drift_dir, basis[2]);
        let mut w = [0.0_f64; 6];
        for p in pv {
            let f = p.force;
            let tau = (positions[p.vid as usize] - com).cross(&f);
            w[0] += tau.x;
            w[1] += tau.y;
            w[2] += tau.z;
            w[3] += f.x;
            w[4] += f.y;
            w[5] += f.z;
        }
        w
    };

    // Analytic node at the base centre: per-axis `dforce_dpose` (the transpose the gradient builds).
    c.sphere_center_override = Some(base);
    let bc = BoundaryConditions::new(c.pinned.clone(), Vec::new());
    let solver: SoftSolver<_> =
        CpuNewtonSolver::new(Tet4, c.fresh_mesh(), c.build_contact(height), c.cfg, bc)
            .with_friction_surface_drift(drift);
    let pv = solver.friction_force_jacobians(&x_next, &x_start, dt, drift_dir, basis[2]);
    let per_axis: Vec<Vec<Vec3>> = basis
        .iter()
        .map(|&e| {
            solver
                .friction_force_jacobians(&x_next, &x_start, dt, drift_dir, e)
                .into_iter()
                .map(|p| p.dforce_dheight)
                .collect()
        })
        .collect();
    let pose_dforce: Vec<Vec<Vec3>> = (0..pv.len())
        .map(|i| per_axis.iter().map(|a| a[i]).collect())
        .collect();
    let mut w_total = SpatialVector::zeros();
    let (fverts, f_total) =
        assemble_friction_wrench(pv, &positions, com, &mut w_total, Some(pose_dforce));
    assert!(!fverts.is_empty(), "config must be friction-engaged");
    let node = FrictionWrenchTrajVjp {
        verts: fverts,
        f_total,
        jlin: c.com_linear_jacobian(),
        n_dof: 3 * n,
        nv,
        n_pose: 3,
        mu_c: false,
    };
    // Pose-parent cotangent rows: ∂(w·e_k)/∂(centre axis), one [3] per wrench component k.
    let pose_rows: Vec<[f64; 3]> = (0..6)
        .map(|k| {
            let mut cot = Tensor::zeros(&[6]);
            cot.as_mut_slice()[k] = 1.0;
            let mut pc = vec![
                Tensor::zeros(&[6]),
                Tensor::zeros(&[3 * n]),
                Tensor::zeros(&[3]),
                Tensor::zeros(&[2 * nv]),
                Tensor::zeros(&[1]),
                Tensor::zeros(&[3 * n]),
            ];
            node.vjp(&cot, &mut pc);
            let s = pc[2].as_slice();
            [s[0], s[1], s[2]]
        })
        .collect();

    let d = 1.0e-7;
    let scale = 1.0e5; // friction-wrench-vs-centre derivatives ~1e5 (cf. the leaf reaction gate)
    let mut worst = 0.0_f64;
    for axis in 0..3 {
        let mut dir = Vec3::zeros();
        dir[axis] = 1.0;
        let wp = wrench_of(&mut c, base + d * dir);
        let wm = wrench_of(&mut c, base - d * dir);
        for k in 0..6 {
            let fd = (wp[k] - wm[k]) / (2.0 * d);
            worst = worst.max((fd - pose_rows[k][axis]).abs() / scale);
        }
    }
    assert!(
        worst < 1e-5,
        "friction wrench CENTRE pose Jacobian must be FD-exact vs the readout, worst {worst:.3e}"
    );
}

/// `VzControlCarryVjp` carries the three rigid-carry coefficients onto its
/// parents: `∂vz'/∂vz = a`, `∂vz'/∂fz = −Δt/m`, `∂vz'/∂u = +Δt/m` (the control
/// term, opposite sign to the contact term).
#[test]
fn control_carry_vjp_coefficients() {
    let op = VzControlCarryVjp {
        a: 0.7,
        neg_dt_over_m: -5e-3,
        dt_over_m: 5e-3,
    };
    let mut parents = vec![
        Tensor::zeros(&[1]),
        Tensor::zeros(&[1]),
        Tensor::zeros(&[1]),
    ];
    op.vjp(&Tensor::from_slice(&[1.0], &[1]), &mut parents);
    assert!(
        (parents[0].as_slice()[0] - 0.7).abs() < 1e-15,
        "∂vz'/∂vz = a"
    );
    assert!(
        (parents[1].as_slice()[0] + 5e-3).abs() < 1e-15,
        "∂vz'/∂fz = −Δt/m"
    );
    assert!(
        (parents[2].as_slice()[0] - 5e-3).abs() < 1e-15,
        "∂vz'/∂u = +Δt/m"
    );
}

/// Lib-level smoke test of the control gradient (the scientific FD validation
/// is the `control` row of `tests/coupling_grad_harness.rs`): one `tape.backward` over a
/// short coupled rollout under a control schedule gives one finite gradient
/// per control input, the forward replays the real dynamics, and a single
/// control input matches an independent FD of the real re-rollout.
#[test]
fn control_gradient_smoke() {
    let controls = vec![-1.5_f64, 1.0, -1.5, 1.0, -1.5, 1.0];
    let (z_tape, grad) = coupling().coupled_trajectory_control_gradient(&controls);
    assert_eq!(grad.len(), controls.len());
    assert!(grad.iter().all(|g| g.is_finite()), "gradients finite");

    // Forward reproduces the real rollout.
    let z_ref = coupling().coupled_trajectory_control_z(&controls);
    assert!(
        (z_tape - z_ref).abs() < 1e-12,
        "tape forward z_N {z_tape} != real rollout {z_ref}"
    );

    // One control input vs an independent FD of the real coupled re-rollout.
    let k = 1;
    let eps = 1e-2;
    let mut up = controls.clone();
    let mut dn = controls.clone();
    up[k] += eps;
    dn[k] -= eps;
    let fd = (coupling().coupled_trajectory_control_z(&up)
        - coupling().coupled_trajectory_control_z(&dn))
        / (2.0 * eps);
    assert!(
        (grad[k] - fd).abs() / fd.abs().max(1e-30) < 1e-6,
        "smoke: control tape grad {} vs FD {fd}",
        grad[k]
    );
}

/// Lib-level smoke test of the closed-loop policy gradient (the scientific FD
/// validation is the `policy(θ)` row of `tests/coupling_grad_harness.rs`): one
/// `tape.backward`
/// over a short closed-loop rollout under `LinearFeedback` gives one finite
/// gradient per policy parameter, the feedback-weight gradients are nonzero
/// (the recurrence is live), the forward replays the real dynamics, and a
/// feedback weight matches an independent FD of the real re-rollout.
#[test]
fn policy_gradient_smoke() {
    let theta = [-20.0_f64, -5.0, 2.0];
    let n = 8;
    let (z_tape, grad) = coupling().coupled_trajectory_policy_gradient(&LinearFeedback, &theta, n);
    assert_eq!(grad.len(), 3);
    assert!(grad.iter().all(|g| g.is_finite()), "gradients finite");
    // Feedback-weight gradients are nonzero: the recurrence is exercised, not
    // just the bias.
    assert!(
        grad[0].abs() > 1e-9 && grad[1].abs() > 1e-9,
        "feedback-weight gradients should be nonzero, got {grad:?}"
    );

    // Forward reproduces the real closed-loop rollout.
    let z_ref = coupling().coupled_trajectory_policy_z(&LinearFeedback, &theta, n);
    assert!(
        (z_tape - z_ref).abs() < 1e-12,
        "tape forward z_N {z_tape} != real rollout {z_ref}"
    );

    // The proportional weight w_z vs an independent FD of the real re-rollout
    // (its gradient flows only through the state→control recurrence).
    let eps = 1e-2;
    let mut up = theta;
    let mut dn = theta;
    up[0] += eps;
    dn[0] -= eps;
    let fd = (coupling().coupled_trajectory_policy_z(&LinearFeedback, &up, n)
        - coupling().coupled_trajectory_policy_z(&LinearFeedback, &dn, n))
        / (2.0 * eps);
    assert!(
        (grad[0] - fd).abs() / fd.abs().max(1e-30) < 1e-5,
        "smoke: policy w_z tape grad {} vs FD {fd}",
        grad[0]
    );
}

/// Lib-level smoke test of the JOINT design+policy gradient (the scientific FD
/// validation is the `joint(μ+θ)` row of `tests/coupling_grad_harness.rs`): one
/// `tape.backward`
/// yields BOTH a finite material gradient `∂z_N/∂μ_total` AND the policy
/// gradient `∂z_N/∂θ`, the forward replays the real rollout, and the joint
/// policy block matches the policy-only method (the material leaf does not
/// perturb the policy gradient).
#[test]
fn joint_gradient_smoke() {
    let theta = [-20.0_f64, -5.0, 2.0];
    let n = 8;
    let (z_tape, dz_dmu, dz_dtheta) =
        coupling().coupled_trajectory_joint_gradient(&LinearFeedback, &theta, n);
    assert_eq!(dz_dtheta.len(), 3);
    // Both blocks finite (the `coupling()` fixture starts above contact, so at
    // n=8 the block may be undeformed ⇒ ∂z/∂μ can be 0; the engaged μ gradient
    // is validated nonzero + FD-exact by the `joint(μ+θ)` row of
    // `tests/coupling_grad_harness.rs`). The policy block is genuinely live
    // (control always moves z).
    assert!(
        dz_dmu.is_finite() && dz_dtheta.iter().all(|g| g.is_finite()),
        "joint gradients finite: μ={dz_dmu}, θ={dz_dtheta:?}"
    );
    assert!(
        dz_dtheta.iter().any(|g| g.abs() > 1e-9),
        "policy block should be live, got {dz_dtheta:?}"
    );

    // Forward reproduces the real closed-loop rollout.
    let z_ref = coupling().coupled_trajectory_policy_z(&LinearFeedback, &theta, n);
    assert!(
        (z_tape - z_ref).abs() < 1e-12,
        "tape z_N {z_tape} != real {z_ref}"
    );

    // The policy block equals the policy-only method (fusion is sound).
    let (_zp, g_theta) = coupling().coupled_trajectory_policy_gradient(&LinearFeedback, &theta, n);
    for (&gj, &gp) in dz_dtheta.iter().zip(&g_theta) {
        assert!((gj - gp).abs() < 1e-14, "joint θ {gj} != policy-only {gp}");
    }
}

// --- Rung 6b: BondedSandwich (two-rigid bonded disc) lib smoke ---

/// Build the canonical FSU sandwich: two free-joint boxes (L5 lower, L4 upper) with
/// a primitive disc's rest slab `z ∈ [0, 0.02]` between their facing surfaces.
fn bonded_fsu() -> BondedSandwich {
    let h = 0.01;
    let edge = 0.02;
    let c = edge / 2.0; // boxes centred on the disc's lateral centre (rest slab [0, edge]²)
    let mjcf = format!(
        r#"<mujoco><option gravity="0 0 0" timestep="0.001"/><worldbody>
  <body name="L5" pos="{c} {c} {z5}"><freejoint/><geom type="box" size="{hx} {hx} {h}" mass="0.05"/></body>
  <body name="L4" pos="{c} {c} {z4}"><freejoint/><geom type="box" size="{hx} {hx} {h}" mass="0.05"/></body>
</worldbody></mujoco>"#,
        c = c,
        z5 = -h,
        z4 = edge + h,
        hx = edge / 2.0,
        h = h,
    );
    let model = load_model(&mjcf).expect("FSU mjcf loads");
    let mut data = model.make_data();
    data.forward(&model).expect("fwd");
    BondedSandwich::new(model, data, 1, 2, 2, edge, 1.0e5, 1.0e3)
}

/// The bond conserves (Newton's third law across the interface) and carries two-way
/// tension under flexion — the rung-6b keystone, pinned in-crate. The decisive
/// magnitudes + the coupled rollout live in `tests/bonded_sandwich_fsu.rs`.
#[test]
fn bonded_sandwich_conserves_and_is_two_way() {
    use nalgebra::UnitQuaternion;
    use sim_soft::Vec3;

    let sum_face = |c: &BondedSandwich, verts: &[u32]| -> Vec3 {
        let r = c.last_reaction();
        verts.iter().fold(Vec3::zeros(), |a, &v| {
            let i = v as usize;
            a + Vec3::new(r[3 * i], r[3 * i + 1], r[3 * i + 2])
        })
    };

    // Compression: press L4 down 5% → disc pushes the plates apart, no tension.
    // (L4 COM rests at (0.01, 0.01, 0.03), centred on the disc.)
    let mut c = bonded_fsu();
    c.set_body_pose(
        2,
        Vec3::new(0.01, 0.01, 0.03 - 0.001),
        UnitQuaternion::identity(),
    );
    let s = c.probe();
    assert!(
        s.force_upper.z > 0.0 && s.force_lower.z < 0.0,
        "disc must spring the plates apart"
    );
    let f_lower = sum_face(&c, c.lower_face());
    let f_upper = sum_face(&c, c.upper_face());
    let scale = f_lower.norm().max(f_upper.norm());
    assert!(
        (f_lower + f_upper).norm() / scale < 1e-9,
        "force not conserved across the bond"
    );

    // Flexion: rotate L4 about x → the bonded face carries BOTH tension and compression.
    let mut c = bonded_fsu();
    let rot = UnitQuaternion::from_axis_angle(&Vec3::x_axis(), 0.08);
    let pivot = Vec3::new(0.01, 0.01, 0.02);
    c.set_body_pose(2, pivot + rot * (Vec3::new(0.01, 0.01, 0.03) - pivot), rot);
    let flex = c.probe();
    let r = c.last_reaction();
    let (min_z, max_z) =
        c.upper_face()
            .iter()
            .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), &v| {
                let rz = r[3 * v as usize + 2];
                (lo.min(rz), hi.max(rz))
            });
    assert!(
        min_z < 0.0 && max_z > 0.0,
        "flexed face must carry tension AND compression"
    );
    assert!(
        flex.moment_upper.x < 0.0,
        "flexion must produce a restoring moment"
    );

    // The coupled loop through the rigid engine runs (round-trip forward).
    let mut c = bonded_fsu();
    let step = c.step();
    assert!(step.force_upper.z.is_finite() && c.data().qpos.iter().all(|q| q.is_finite()));
}

/// Rung 6d: the reverse-mode pose gradient of the bonded wrench is finite, live, and has
/// the right physical sign — pinned in-crate for `--lib` coverage. The full 6-twist FD
/// match (both bodies) lives in `tests/bonded_pose_gradient.rs`.
#[test]
fn bonded_pose_gradient_is_live_and_physical() {
    use nalgebra::UnitQuaternion;
    use sim_soft::Vec3;

    // Compress L4 straight down 5%, then differentiate L = force_upper.z.
    let mut c = bonded_fsu();
    c.set_body_pose(
        2,
        Vec3::new(0.01, 0.01, 0.03 - 0.001),
        UnitQuaternion::identity(),
    );
    let mut cot_upper = SpatialVector::zeros();
    cot_upper[5] = 1.0; // ∂L/∂force_upper.z
    let (step, grads) = c.probe_with_pose_gradient(SpatialVector::zeros(), cot_upper);

    assert!(
        step.force_upper.z > 0.0,
        "compressed disc pushes the upper plate up"
    );
    let grad_upper = grads[1];
    assert!(
        grad_upper.iter().all(|g| g.is_finite()),
        "gradient must be finite"
    );
    let live = grad_upper.iter().map(|g| g.abs()).fold(0.0, f64::max);
    assert!(live > 1.0, "pose gradient degenerate (max {live:.3e})");
    // Raising the upper plate (+v_z) relieves compression, so ∂(force_upper.z)/∂v_z < 0 —
    // the axial bond stiffness (the reverse dual of the block spike's −9061.966).
    assert!(
        grad_upper[5] < 0.0,
        "axial stiffness sign wrong: ∂F_up.z/∂v_z = {}",
        grad_upper[5]
    );
}
