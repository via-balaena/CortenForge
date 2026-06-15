//! Keystone multi-DOF rigid coupling — the coupled ARTICULATED trajectory
//! gradient, WITH the off-COM contact moment.
//!
//! `StaggeredCoupling::coupled_trajectory_material_gradient_articulated` is the
//! articulated successor to the free-body-platen `coupled_trajectory_material_gradient`:
//! the rigid body is a Y-hinge (a tilted arm with a point mass at the tip pressing
//! on the soft block), so a contact force at the tip maps to a generalized joint
//! acceleration coupled across the joint (the matrix `Δt·M⁻¹·Jᵀ` ≠ the scalar
//! `dt/m`), the contact-plane pose tracks the moving tip (`∂(tip height)/∂q = J_z`),
//! and the reaction is routed as the full spatial **wrench** `[τ; f]` about the body
//! COM — including the off-COM contact **moment** `τ = −Σ(rᵢ−c)×gᵢ`. The tip COM
//! (`xipos`, x ≈ −0.028 when tilted) is offset ~0.08 m from the block-top contact
//! centroid (x ≈ 0.05), so the resultant genuinely misses the COM: routing the
//! moment vs a pure force at the COM shifts the rollout materially (~4.5% in tip_z_N
//! by n = 6, growing to ~7% by n = 10 — the forward model is first-order wrong
//! without it). One `tape.backward` gives `∂(tip height)_N/∂μ` across the whole
//! rollout.
//!
//! Gates: the one-tape gradient matches a central FD of the FULL real coupled
//! re-rollout (a fresh coupling at μ±ε, the real articulated `step` loop —
//! `coupled_trajectory_articulated_z`, which routes the same moment), an INDEPENDENT
//! oracle. The wrench node (`∂w/∂x*`, `∂w/∂h`, `∂w/∂s`) is separately FD-validated
//! machine-exact against the real contact readout in the
//! `contact_wrench_node_matches_readout_fd` lib unit test; the rigid state carry
//! `J_state` is the analytic single-hinge geometric stiffness, FD-validated in
//! `analytic_state_jacobian_matches_fd_loaded`.
//!
//! **Accuracy — MACHINE-EXACT at every horizon, single-hinge through multi-link.** The
//! composed gradient matches the full-coupled FD to ~1e-9 at n = 1, 2, 6, 10, 15 — for
//! the single hinge, a free-joint platen (nv = 6), AND a 2-link chain (nv = 2 at
//! FD-carry precision, `twolink_chain_gradient_matches_fd`). The earlier long-rollout moment residual
//! (~1e-3 at n = 10) and the 74%-at-n=2 multi-link error were BOTH the same defect: the
//! stale-FK contact pose + §8a position-row drop was a self-consistent pair calibrated
//! ONLY for nv = 1. The **fully-fresh formulation** — fresh-FK contact pose + fresh
//! output + the true position-row carry `∂qpos'/∂w = Δt·G_vel` — is the correct
//! differentiable formulation (and the fresh pose is more physically faithful). See
//! docs/keystone/moment_residual_recon.md and docs/keystone/multilink_recon.md.
//!
//! **Quaternion joints (`nq ≠ nv`).** The carry is reformulated in SO(3)/SE(3) TANGENT
//! space (SO(3)-aware FD `loaded_state_jacobian` + the position-row right Jacobian
//! `G_pos = Δt·J_r(Δt·qvel')·G_vel`), so the gradient is correct through quaternion DOFs:
//! an off-COM BALL joint (`nv = 3`, `ball_joint_gradient_matches_fd`) and an off-COM FREE
//! joint (`nv = 6`, the floating base, `free_joint_offcom_gradient_matches_fd`) are
//! machine-exact at n = 1, 2, 4. Both need an OFF-COM contact: a centered free platen is
//! degenerate (`xipos.z` is orientation-blind, so the quaternion never enters the
//! μ-gradient). See docs/keystone/quaternion_joints_recon.md.

#![allow(clippy::expect_used)]

use sim_coupling::StaggeredCoupling;
use sim_mjcf::load_model;

// A Y-hinge: pivot above the soft block, a point mass on the link tip. Started
// tilted (qpos = 0.3) so the tip arcs (the moment arm makes dt/m wrong) and
// presses into the block top (z = 0.1); gravity swings it inward → sustained
// engagement. No joint damping/stiffness (the bare-M⁻¹ scope).
const HINGE_MJCF: &str = r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="arm" pos="0 0 0.2">
      <joint type="hinge" axis="0 1 0"/>
      <geom type="sphere" pos="0 0 -0.095" size="0.004" mass="0.2"/>
    </body>
  </worldbody>
</mujoco>"#;

const MU0: f64 = 3.0e4;

fn build(mu: f64) -> StaggeredCoupling {
    let model = load_model(HINGE_MJCF).expect("hinge MJCF loads");
    let mut data = model.make_data();
    data.qpos[0] = 0.3; // tilt off vertical
    data.forward(&model).expect("initial forward");
    // body=1, clearance=0.005, n_per_edge=4 (even), edge=0.1, mu, dt=1e-3,
    // kappa=3e4, d_hat=1e-2, rigid_damping=0 (articulated v1 scope).
    StaggeredCoupling::new(
        model, data, 1, 0.005, 4, 0.1, mu, 1.0e-3, 3.0e4, 1.0e-2, 0.0,
    )
}

#[test]
fn articulated_trajectory_engages_and_is_stable() {
    let mut c = build(MU0);
    let z0 = c.data().xipos[1].z;
    let zn = c.coupled_trajectory_articulated_z(10);
    assert!(zn.is_finite(), "tip height must stay finite");
    // The tilted arm presses into the block and stays engaged (the plane =
    // tip_z − 0.005 sits within d_hat of the block top z = 0.1).
    assert!(
        zn < 0.115 && zn > 0.10,
        "tip should remain engaged near the block top, got z0={z0} zN={zn}"
    );
}

/// The total `∂tip_z_N/∂μ` (along the coupling's λ = 4μ tie) from one tape, and a
/// central FD of the full real coupled re-rollout at μ±ε.
fn tape_and_fd(n: usize) -> (f64, f64, f64) {
    let (tip_z, g_mu) = build(MU0).coupled_trajectory_material_gradient_articulated(n, 0);
    let (_t2, g_la) = build(MU0).coupled_trajectory_material_gradient_articulated(n, 1);
    let total = g_mu + 4.0 * g_la;
    let eps = MU0 * 1e-4;
    let zp = build(MU0 + eps).coupled_trajectory_articulated_z(n);
    let zm = build(MU0 - eps).coupled_trajectory_articulated_z(n);
    let fd = (zp - zm) / (2.0 * eps);
    (tip_z, total, fd)
}

#[test]
fn articulated_gradient_matches_full_coupled_fd_through_make_break() {
    // n = 6 spans the initial violent make (~25 active, f_z ~ 800 N), a liftoff, and a
    // re-touch. The moment-routing tape gradient is MACHINE-EXACT against the
    // independent full-coupled FD oracle (the wrench node is FD-exact vs the real
    // readout in `contact_wrench_node_matches_readout_fd`; the analytic single-hinge
    // `J_state` in `analytic_state_jacobian_matches_fd_loaded`). The fully-fresh
    // formulation (fresh-FK contact pose + fresh output + true position-row carry
    // `∂qpos'/∂w = Δt·G_vel`) removed the old long-rollout moment residual — see
    // `docs/keystone/moment_residual_recon.md`.
    let n = 6;
    let (tip_z, total, fd) = tape_and_fd(n);
    let z_oracle = build(MU0).coupled_trajectory_articulated_z(n);
    assert!(
        (tip_z - z_oracle).abs() < 1e-12,
        "tape forward tip_z {tip_z} != real rollout {z_oracle}"
    );
    let rel = (total - fd).abs() / fd.abs().max(1e-30);
    println!("n={n}: tape total={total:.8e} FD={fd:.8e} rel={rel:.3e}");
    assert!(
        rel < 1e-7,
        "articulated dμ gradient (with moment) must be machine-exact vs full-coupled \
         FD through make/break, got rel {rel:.3e}"
    );
    assert!(total.abs() > 1e-9, "expected a nonzero ∂tip_z/∂μ");
    // The dt/m-vs-multi-DOF distinction — J_z ≠ 1 on the tilted contact axis — is
    // asserted at the primitive level in tests/rigid_multidof_response.rs.
}

/// The articulated gradient stays MACHINE-EXACT over LONG rollouts with sustained
/// re-engagement (n = 10 and n = 15) — the fully-fresh formulation removed the
/// moment-residual growth the earlier stale-FK convention carried (~1e-3 at n = 10,
/// ~5e-3 at n = 15). See `docs/keystone/moment_residual_recon.md`.
#[test]
fn articulated_gradient_long_rollout_machine_exact() {
    for n in [10usize, 15] {
        let (_tip_z, total, fd) = tape_and_fd(n);
        let rel = (total - fd).abs() / fd.abs().max(1e-30);
        println!("n={n}: tape total={total:.8e} FD={fd:.8e} rel={rel:.3e}");
        assert!(
            rel < 1e-7,
            "n={n} articulated dμ gradient must be machine-exact, got rel {rel:.3e}"
        );
    }
}

/// The articulated path on a free-joint platen (no off-COM moment, symmetric contact)
/// is machine-exact at every n — the fully-fresh formulation handles the free joint
/// (nv = 6) as cleanly as the hinge. (Historically this was the discriminator that
/// isolated the long-rollout residual to the off-COM moment, since fixed by the
/// fully-fresh formulation; see `docs/keystone/moment_residual_recon.md`.)
#[test]
fn articulated_free_platen_exact_all_n() {
    const FREE_PLATEN_MJCF: &str = r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="platen" pos="0 0 0.108">
      <freejoint/>
      <geom type="box" size="0.06 0.06 0.005" mass="0.2"/>
    </body>
  </worldbody>
</mujoco>"#;
    let build_free = |mu: f64| -> StaggeredCoupling {
        let model = load_model(FREE_PLATEN_MJCF).expect("platen MJCF loads");
        let mut data = model.make_data();
        data.forward(&model).expect("forward");
        StaggeredCoupling::new(
            model, data, 1, 0.005, 4, 0.1, mu, 1.0e-3, 3.0e4, 1.0e-2, 0.0,
        )
    };
    for n in [6usize, 10, 12] {
        let (_t, g_mu) = build_free(MU0).coupled_trajectory_material_gradient_articulated(n, 0);
        let (_t2, g_la) = build_free(MU0).coupled_trajectory_material_gradient_articulated(n, 1);
        let total = g_mu + 4.0 * g_la;
        let eps = MU0 * 1e-4;
        let zp = build_free(MU0 + eps).coupled_trajectory_articulated_z(n);
        let zm = build_free(MU0 - eps).coupled_trajectory_articulated_z(n);
        let fd = (zp - zm) / (2.0 * eps);
        let rel = (total - fd).abs() / fd.abs().max(1e-30);
        println!("free-platen articulated n={n}: tape={total:.6e} FD={fd:.6e} rel={rel:.3e}");
        assert!(
            rel < 1e-6,
            "free-platen articulated gradient (no moment) must be machine-exact at \
             n={n}, got rel {rel:.3e}"
        );
    }
}

/// Short rollouts are machine-exact too. With the fresh output (`tip_z_N = xipos(q_N)`,
/// not the old lagged `xipos(q_{N-1})`), even n = 1 carries a real, nonzero μ-gradient
/// (the step-0 wrench reaches `qpos_1` through `Δt·qvel_1`, the true position-row term) —
/// and the tape matches the full-coupled FD machine-exactly at n = 1 and n = 2.
#[test]
fn articulated_gradient_short_rollout_is_machine_exact() {
    for n in [1usize, 2] {
        let (_t, total, fd) = tape_and_fd(n);
        let rel = (total - fd).abs() / fd.abs().max(1e-30);
        assert!(
            total.abs() > 1e-12,
            "n={n}: fresh-output gradient should be nonzero (tape {total:.3e})"
        );
        assert!(
            rel < 1e-7,
            "n={n} articulated gradient must be machine-exact, got rel {rel:.3e}"
        );
    }
}

// A 2-LINK hinge chain (nv = 2): a genuine multi-DOF articulated mechanism — the
// distal tip presses the soft block, the contact wrench couples ACROSS both joints
// (off-diagonal M, Coriolis velocity coupling). The capstone-exo bridge beyond the
// single hinge.
const TWOLINK_MJCF: &str = r#"<mujoco>
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

fn build_2link(mu: f64) -> StaggeredCoupling {
    let model = load_model(TWOLINK_MJCF).expect("2-link MJCF loads");
    let mut data = model.make_data();
    data.qpos[0] = 0.1;
    data.qpos[1] = -0.05; // off-center tip; the off-diagonal joint coupling is live
    data.forward(&model).expect("forward");
    StaggeredCoupling::new(
        model, data, 2, 0.005, 4, 0.1, mu, 1.0e-3, 3.0e4, 1.0e-2, 0.0,
    )
}

/// **The multi-link (nv = 2) coupled gradient matches the full-coupled FD.** A 2-link
/// hinge chain — genuine multi-DOF (off-diagonal mass coupling + Coriolis) — through the
/// same articulated path, FD-gated at n = 2/6/10. The fully-fresh formulation handles the
/// chain that the single-hinge-calibrated stale-FK convention got 74% wrong at n = 2.
/// The accuracy here is FD-CARRY precision (~1e-6): a chain has no single-hinge analytic
/// `J_state`, so it uses the FD `loaded_state_jacobian` (eps 1e-6). The analytic CHAIN
/// carry (the Jacobian Hessian + `∂M⁻¹/∂q`) would lift this to ~1e-9 and is the
/// documented follow-on; see `docs/keystone/multilink_recon.md`.
#[test]
fn twolink_chain_gradient_matches_fd() {
    let z0 = build_2link(MU0).data().xipos[2].z;
    assert!(
        z0 > 0.10,
        "2-link tip should start engaged near the block top, got {z0}"
    );
    for n in [2usize, 6, 10] {
        let (_t, gm) = build_2link(MU0).coupled_trajectory_material_gradient_articulated(n, 0);
        let (_t2, gl) = build_2link(MU0).coupled_trajectory_material_gradient_articulated(n, 1);
        let total = gm + 4.0 * gl;
        let eps = MU0 * 1e-4;
        let zp = build_2link(MU0 + eps).coupled_trajectory_articulated_z(n);
        let zm = build_2link(MU0 - eps).coupled_trajectory_articulated_z(n);
        let fd = (zp - zm) / (2.0 * eps);
        let rel = (total - fd).abs() / fd.abs().max(1e-30);
        println!("2-link n={n}: tape={total:.6e} FD={fd:.6e} rel={rel:.3e}");
        assert!(
            total.abs() > 1e-12 && rel < 1e-5,
            "2-link nv=2 gradient must match full-coupled FD (FD-carry precision) at \
             n={n}, got rel {rel:.3e}"
        );
    }
}

// A BALL joint (`nq = 4, nv = 3`): the first QUATERNION joint in the coupled gradient. A
// single arm on a ball joint, tilted θ = 0.3 about Y so the off-COM tip (`xipos.x ≈
// −0.028`) presses the block — the contact reaction's off-COM moment swings the ball, so
// ∂tip_z/∂μ flows THROUGH the quaternion DOFs (orientation enters the gradient). This is
// the scene S0 found the raw-`qpos` carry got WRONG-SIGN (rel 1.15 at n = 4): the
// `loaded_state_jacobian` perturbed un-normalized quaternion components and differenced
// raw `qpos'`, not the SO(3) tangent. The tangent-FD carry + the SO(3) right-Jacobian
// position-row (`G_pos = Δt·J_r·G_vel`) make it match the full-coupled FD. A free body
// with COM contact is a DEGENERATE quaternion test (orientation never enters the
// COM-height plane — `articulated_free_platen_exact_all_n` passes even at 107° rotation);
// the off-COM ball joint is the scene that exercises SO(3) in the gradient.
const BALL_MJCF: &str = r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="arm" pos="0 0 0.2">
      <joint type="ball"/>
      <geom type="sphere" pos="0 0 -0.095" size="0.004" mass="0.2"/>
    </body>
  </worldbody>
</mujoco>"#;

fn build_ball(mu: f64) -> StaggeredCoupling {
    let model = load_model(BALL_MJCF).expect("ball MJCF loads");
    let mut data = model.make_data();
    // Tilt θ = 0.3 about Y as a unit quaternion [cos(θ/2), 0, sin(θ/2), 0] — the same
    // tilt the hinge scene sets via qpos[0] = 0.3, but on SO(3) (nq = 4 ≠ nv = 3).
    let half = 0.15_f64;
    data.qpos[0] = half.cos();
    data.qpos[1] = 0.0;
    data.qpos[2] = half.sin();
    data.qpos[3] = 0.0;
    data.forward(&model).expect("forward");
    StaggeredCoupling::new(
        model, data, 1, 0.005, 4, 0.1, mu, 1.0e-3, 3.0e4, 1.0e-2, 0.0,
    )
}

/// **The ball-joint (`nq = 4, nv = 3`) coupled gradient matches the full-coupled FD.** The
/// first quaternion joint: the off-COM tip's height depends on the ball ORIENTATION, so
/// ∂tip_z/∂μ flows through the quaternion DOFs. The TANGENT-space reformulation — the
/// SO(3)-aware FD `loaded_state_jacobian` (perturb via `mj_integrate_pos_explicit`,
/// difference via `mj_differentiate_pos`) plus the position-row SO(3) right Jacobian
/// `G_pos = Δt·J_r(Δt·qvel')·G_vel` — fixes the S0 gap (raw `qpos` was WRONG-SIGN, rel
/// 1.15 at n = 4; now machine-exact). Accuracy is FD-CARRY precision (~1e-6): a ball joint
/// has no single-hinge analytic `J_state`, so it uses the FD `loaded_state_jacobian` (eps
/// 1e-6), like the 2-link chain.
///
/// Gated at n = 1, 2, 4 — the stably-engaged, well-conditioned regime (FD converged across
/// eps_rel 1e-3…1e-6). A ball joint is UNCONSTRAINED (3 rotational DOFs, no restoring
/// stiffness), so the violent initial contact impulse eventually launches the arm into a
/// chaotic regime (by n ≈ 6 the tip leaves the block and the FD is non-convergent — the
/// GRADIENT itself becomes ill-conditioned, not the tape). Long-rollout gradient
/// robustness is covered by the constrained hinge / free-platen / chain gates above. See
/// `docs/keystone/quaternion_joints_recon.md`.
#[test]
fn ball_joint_gradient_matches_fd() {
    let z0 = build_ball(MU0).data().xipos[1].z;
    assert!(
        z0 > 0.10 && z0 < 0.115,
        "ball-joint tip should start engaged near the block top, got {z0}"
    );
    for n in [1usize, 2, 4] {
        let (tip_z, gm) = build_ball(MU0).coupled_trajectory_material_gradient_articulated(n, 0);
        let (_t2, gl) = build_ball(MU0).coupled_trajectory_material_gradient_articulated(n, 1);
        let total = gm + 4.0 * gl;
        // The fresh-output tape forward must reproduce the independent real rollout.
        let z_oracle = build_ball(MU0).coupled_trajectory_articulated_z(n);
        assert!(
            (tip_z - z_oracle).abs() < 1e-12,
            "n={n}: tape forward tip_z {tip_z} != real rollout {z_oracle}"
        );
        let eps = MU0 * 1e-4;
        let zp = build_ball(MU0 + eps).coupled_trajectory_articulated_z(n);
        let zm = build_ball(MU0 - eps).coupled_trajectory_articulated_z(n);
        let fd = (zp - zm) / (2.0 * eps);
        let rel = (total - fd).abs() / fd.abs().max(1e-30);
        println!("ball n={n}: tape={total:.6e} FD={fd:.6e} rel={rel:.3e}");
        assert!(
            total.abs() > 1e-12 && rel < 1e-5,
            "ball-joint nq=4 nv=3 gradient must match full-coupled FD (FD-carry \
             precision) at n={n}, got rel {rel:.3e}"
        );
    }
}

// A FREE joint (`nq = 7, nv = 6`): the floating base — the full SE(3) successor to the
// ball joint, the capstone exo's root. Two geoms: a stabilizing plate at the body origin
// + an OFFSET mass (`0.03, 0, −0.02`), so the body COM is offset ~0.025 m HORIZONTALLY
// from the contact-patch centroid (`xipos.x ≈ 0.025`). That offset is the whole point:
// (1) the contact resultant misses the COM → an off-COM MOMENT that rotates the free body
// (the quaternion evolves — |Δq| ≈ 1.35 over the 4-step check, asserted), and (2) the objective
// `xipos.z = body_z + (R·offset).z` depends on the body ORIENTATION, so ∂tip_z/∂μ flows
// THROUGH the SE(3) angular DOFs. A CENTERED free platen
// (`articulated_free_platen_exact_all_n`) is DEGENERATE — `xipos.z` = body-z is
// orientation-blind, so the quaternion never enters the μ-gradient (it passes even while
// spinning). The offset COM is what makes this a real SE(3) test.
const FREE_OFFCOM_MJCF: &str = r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="body" pos="0 0 0.116">
      <freejoint/>
      <geom type="box" pos="0 0 0" size="0.06 0.06 0.003" mass="0.05"/>
      <geom type="sphere" pos="0.03 0 -0.02" size="0.004" mass="0.3"/>
    </body>
  </worldbody>
</mujoco>"#;

fn build_free_offcom(mu: f64) -> StaggeredCoupling {
    let model = load_model(FREE_OFFCOM_MJCF).expect("free off-COM MJCF loads");
    let mut data = model.make_data();
    // free joint qpos = [x y z  qw qx qy qz]; seed a small θ=0.05 tilt about Y.
    let half = 0.025_f64;
    data.qpos[3] = half.cos();
    data.qpos[5] = half.sin();
    data.forward(&model).expect("forward");
    StaggeredCoupling::new(
        model, data, 1, 0.005, 4, 0.1, mu, 1.0e-3, 3.0e4, 1.0e-2, 0.0,
    )
}

/// **The free-joint (`nq = 7, nv = 6`) off-COM coupled gradient matches the full-coupled
/// FD.** The full SE(3) floating base: an off-COM contact moment rotates the free body and
/// the objective `xipos.z = body_z + (R·offset).z` reads the orientation, so ∂tip_z/∂μ
/// flows through ALL SIX DOFs incl. the 3 SO(3) angular ones. The same tangent-space
/// reformulation that fixed the ball joint (the SO(3)-aware FD `loaded_state_jacobian` +
/// the position-row right Jacobian `G_pos = Δt·J_r(Δt·qvel')·G_vel`, with the free joint's
/// `[Δt·I₃ | Δt·J_r]` integrator block) handles `nv = 6` with no new machinery — machine
/// exact at FD-carry precision. Materiality is asserted directly: the COM is off the
/// contact-patch centroid AND the quaternion rotates materially over the rollout (vs the
/// degenerate centered `articulated_free_platen_exact_all_n`). See
/// `docs/keystone/quaternion_joints_recon.md`.
///
/// Gated at n = 1, 2, 4 (FD-converged, well-conditioned). Like the ball, an unconstrained
/// free body resting on a single block is rotationally unstable at longer horizons; this
/// scene stays machine-exact further (n = 6 ≈ 6e-6) but the hard gate matches the ball's
/// CI-proven margins.
#[test]
fn free_joint_offcom_gradient_matches_fd() {
    // Materiality #1: the COM is genuinely off the contact-patch centroid (orientation
    // enters the objective). A centered platen has xipos.x ≈ 0 and is degenerate.
    let cx = build_free_offcom(MU0).data().xipos[1].x;
    assert!(
        cx > 0.015,
        "scene must be off-COM for SE(3) to enter the gradient, got xipos.x = {cx}"
    );
    // Materiality #2: the free body actually ROTATES over the rollout (the SO(3) angular
    // DOFs are live, not a frozen orientation).
    let q0 = {
        let c = build_free_offcom(MU0);
        [
            c.data().qpos[3],
            c.data().qpos[4],
            c.data().qpos[5],
            c.data().qpos[6],
        ]
    };
    let qn = {
        let mut c = build_free_offcom(MU0);
        for _ in 0..4 {
            let _ = c.coupled_trajectory_articulated_z(1);
        }
        [
            c.data().qpos[3],
            c.data().qpos[4],
            c.data().qpos[5],
            c.data().qpos[6],
        ]
    };
    let dq: f64 = (0..4).map(|i| (qn[i] - q0[i]).powi(2)).sum::<f64>().sqrt();
    // |Δq| ≈ 1.35 here — a large rotation; `> 1.0` proves the SO(3) DOFs are genuinely
    // live (not a frozen orientation) with margin, without pinning the exact tumble.
    assert!(
        dq > 1.0,
        "the free body must rotate materially, got |Δq| = {dq:.3e}"
    );

    for n in [1usize, 2, 4] {
        let (tip_z, gm) =
            build_free_offcom(MU0).coupled_trajectory_material_gradient_articulated(n, 0);
        let (_t2, gl) =
            build_free_offcom(MU0).coupled_trajectory_material_gradient_articulated(n, 1);
        let total = gm + 4.0 * gl;
        let z_oracle = build_free_offcom(MU0).coupled_trajectory_articulated_z(n);
        assert!(
            (tip_z - z_oracle).abs() < 1e-12,
            "n={n}: tape forward tip_z {tip_z} != real rollout {z_oracle}"
        );
        let eps = MU0 * 1e-4;
        let zp = build_free_offcom(MU0 + eps).coupled_trajectory_articulated_z(n);
        let zm = build_free_offcom(MU0 - eps).coupled_trajectory_articulated_z(n);
        let fd = (zp - zm) / (2.0 * eps);
        let rel = (total - fd).abs() / fd.abs().max(1e-30);
        println!("free-offcom n={n}: tape={total:.6e} FD={fd:.6e} rel={rel:.3e}");
        assert!(
            total.abs() > 1e-12 && rel < 1e-5,
            "free-joint nq=7 nv=6 off-COM gradient must match full-coupled FD (FD-carry \
             precision) at n={n}, got rel {rel:.3e}"
        );
    }
}
