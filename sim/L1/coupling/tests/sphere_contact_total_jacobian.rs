//! Total single-step contact-force sensitivity to a FINITE posed sphere's pose —
//! the coupling-level lift of #415's L0 curved-pose gate (`soft_pose_sensitivity`'s
//! `sphere_pose_sensitivity_matches_resolve_fd`).
//!
//! The keystone S3 gate (`coupled_total_jacobian.rs`) validates
//! `d force/d height = ∂force/∂h|_x + (∂force/∂x)·(∂x*/∂h)` for the infinite
//! `RigidPlane`. This gate re-runs the SAME analytic assembly
//! ([`StaggeredCoupling::contact_force_height_total_jacobian`]) against the SAME
//! black-box oracle ([`StaggeredCoupling::resolved_contact_force`]) but with the
//! collider swapped to a finite [`PosedSphere`] (`with_sphere_collider`) — a curved
//! end-effector whose `∇²sd ≠ 0`, so the contact normal turns as a vertex slides
//! over it.
//!
//! Why it's a genuinely new check (not the plane gate re-skinned):
//! - The plane's normal is constant, so its geometric-stiffness term `dE·H` (the
//!   curvature #415 added) is identically zero. A sphere's is not — both the
//!   pose-residual (`−H·u`) and the Newton tangent (`dE·H`) curvature terms are
//!   live, and the IFT pose sensitivity `∂x*/∂h` reuses that tangent.
//! - If the implicit term in the total Jacobian dropped the curved-normal
//!   stiffness, the assembly would disagree with the nonlinear re-solve FD here
//!   (it agrees for the plane regardless, because the dropped term is zero there).
//!
//! Scope: contact-engaged, stable-active-set regime (the penalty active-set
//! boundary is non-smooth — IPC the deferred cure); hard penalty
//! (`d²E/dsd² = κ`). See `docs/keystone/s3_soft_pose_sensitivity_recon.md` and the
//! L0 curvature term in `sim/L0/soft/src/contact/penalty.rs`.

#![allow(clippy::expect_used)]

use sim_coupling::StaggeredCoupling;
use sim_mjcf::load_model;

const PLATEN_MJCF: &str = r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="platen" pos="0 0 0.125">
      <freejoint/>
      <geom type="box" size="0.06 0.06 0.005" mass="0.2"/>
    </body>
  </worldbody>
</mujoco>"#;

const KAPPA: f64 = 3.0e4;
/// Sphere radius (m). Large vs the 0.1 m block so the south-pole contact patch
/// spans several top-face vertices (a non-degenerate active set) yet stays curved
/// enough that the geometric-stiffness term is materially nonzero.
const SPHERE_R: f64 = 0.08;

#[test]
fn sphere_total_force_jacobian_wrt_height_matches_resolve_fd() {
    let model = load_model(PLATEN_MJCF).expect("platen MJCF loads");
    let mut data = model.make_data();
    data.forward(&model).expect("initial forward");
    // Same block / contact params as the plane S3 gate (n_per_edge=4, edge=0.1,
    // μ=3e4, dt=1e-3, κ=3e4, d̂=1e-2), but with a finite sphere collider.
    let coupling: StaggeredCoupling = StaggeredCoupling::new(
        model, data, 1, 0.005, 4, 0.1, 3.0e4, 1.0e-3, KAPPA, 1.0e-2, 12.0,
    )
    .with_sphere_collider(SPHERE_R);

    // Engaged height: the sphere's south pole presses ~1.5 mm into the rest top
    // face (z = 0.1). The patch is the central cluster of top-face vertices; the
    // active set stays stable across the ±ε FD perturbation.
    let h = 0.0985;

    // Analytic total = explicit (∂force/∂h|_x) + implicit (∂force/∂x · ∂x*/∂h).
    let analytic = coupling.contact_force_height_total_jacobian(h).z;
    // The explicit-only partial, for contrast (over-predicts the engaged total).
    let explicit_only = coupling.contact_force_height_jacobian(h).z;

    // Black-box oracle: re-solve the nonlinear soft step at h ± ε and
    // central-difference the resulting contact force. No analytic machinery.
    let dh = 1.0e-6;
    let fd = (coupling.resolved_contact_force(h + dh).z
        - coupling.resolved_contact_force(h - dh).z)
        / (2.0 * dh);

    eprintln!(
        "sphere total ∂force_z/∂h: analytic={analytic:.6e}  re-solve FD={fd:.6e}  \
         (explicit-only={explicit_only:.6e})"
    );

    // Non-degenerate gate: the re-solve FD must be a real, nonzero slope.
    assert!(
        fd.abs() > 1.0,
        "degenerate gate: re-solve FD slope ≈ 0 ({fd}) — sphere not engaging the block?"
    );
    // The implicit term must actually contribute (the soft re-equilibration moves
    // the total away from the fixed-position partial).
    assert!(
        (analytic - explicit_only).abs() / explicit_only.abs() > 1e-3,
        "implicit term negligible (analytic {analytic} ≈ explicit-only {explicit_only})"
    );
    // HEADLINE: the analytically-assembled total matches the independent nonlinear
    // re-solve FD even though the collider is CURVED — the curved-normal stiffness
    // composes correctly through the coupling adjoint. Same 1e-7 bound as the plane
    // S3 gate (the oracle's Newton tol ~1e-10 ÷ 2ε ≈ 2e-6 floor leaves headroom).
    let rel = (analytic - fd).abs() / fd.abs();
    assert!(
        rel < 1e-7,
        "sphere total Jacobian disagrees with re-solve FD (rel {rel:.3e}): {analytic} vs {fd}"
    );
    eprintln!(
        "✓ sphere finite-contact total Jacobian matches the nonlinear re-solve FD (rel {rel:.2e})"
    );
}

/// The L1a scope guard: the MULTI-step trajectory gradients still assemble the flat
/// `−cᵥ·n̂⊗n̂` contact-force Jacobian (missing the curved `dE·H` term — the L1b
/// follow-on), so calling one with a finite sphere collider must FAIL LOUDLY rather
/// than emit a silently-wrong gradient (a silent contract violation on a public API
/// is ship-blocking). The guard lives in `active_pair_curvatures`, the single factor
/// every multi-step gradient path consumes.
#[test]
#[should_panic(expected = "plane-only")]
fn sphere_collider_panics_on_multistep_gradient() {
    let model = load_model(PLATEN_MJCF).expect("platen MJCF loads");
    let mut data = model.make_data();
    data.forward(&model).expect("initial forward");
    let mut coupling: StaggeredCoupling = StaggeredCoupling::new(
        model, data, 1, 0.005, 4, 0.1, 3.0e4, 1.0e-3, KAPPA, 1.0e-2, 12.0,
    )
    .with_sphere_collider(SPHERE_R);
    // A multi-step material gradient on the sphere collider is not yet curvature-correct;
    // it must panic in `active_pair_curvatures` rather than return a wrong number.
    let _ = coupling.coupled_trajectory_material_gradient(2, 0);
}

/// A tilted Y-hinge arm pressing a point-mass tip into the soft block — the
/// articulated scene the wrench-path trajectory gradients target.
const HINGE_MJCF: &str = r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="arm" pos="0 0 0.2">
      <joint type="hinge" axis="0 1 0"/>
      <geom type="sphere" pos="0 0 -0.095" size="0.004" mass="0.2"/>
    </body>
  </worldbody>
</mujoco>"#;

/// The ARTICULATED arm of the L1a scope guard (the gap the pre-PR review missed):
/// the articulated trajectory gradients assemble the contact WRENCH via
/// `active_pair_wrench_data` → `ContactWrenchTrajVjp`, a different path than the
/// free-body `active_pair_curvatures` guard covers — and `active_pair_wrench_data`
/// cannot self-guard because the curvature-correct single-step pose Jacobians use it
/// too. So each articulated gradient method carries a method-entry `require_plane_collider`
/// guard; a finite sphere collider must panic LOUDLY rather than emit a silently-wrong
/// gradient (and the sphere is doubly unsupported here — `build_contact` poses it over
/// the block centre, not the arm tip).
#[test]
#[should_panic(expected = "plane collider")]
fn sphere_collider_panics_on_articulated_gradient() {
    let model = load_model(HINGE_MJCF).expect("hinge MJCF loads");
    let mut data = model.make_data();
    data.qpos[0] = 0.3; // tilt off vertical
    data.forward(&model).expect("initial forward");
    // Articulated v1 scope requires rigid_damping = 0.
    let mut coupling: StaggeredCoupling = StaggeredCoupling::new(
        model, data, 1, 0.005, 4, 0.1, 3.0e4, 1.0e-3, KAPPA, 1.0e-2, 0.0,
    )
    .with_sphere_collider(SPHERE_R);
    let _ = coupling.coupled_trajectory_material_gradient_articulated(2, 0);
}
