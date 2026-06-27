//! Total single-step contact-force sensitivity to a FINITE posed sphere's pose —
//! the coupling-level lift of #415's L0 curved-pose gate (`soft_pose_sensitivity`'s
//! `sphere_pose_sensitivity_matches_resolve_fd`).
//!
//! The keystone S3 gate (`coupled_total_jacobian.rs`) validates
//! `d force/d height = ∂force/∂h|_x + (∂force/∂x)·(∂x*/∂h)` for the infinite
//! `RigidPlane`. This gate re-runs the SAME analytic assembly
//! ([`StaggeredCoupling::contact_force_height_total_jacobian`]) against the SAME
//! black-box oracle ([`StaggeredCoupling::resolved_contact_force`]) but with the
//! collider swapped to a finite `TranslatedSdf<SphereSdf>` (`with_sphere_collider`) — a curved
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
use sim_soft::Vec3;

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

/// The **moving-end-effector leaf** (single-step): the total `∂(contact force)/∂(sphere centre
/// LATERAL x̂)` matches a black-box re-solve FD. The keystone validates only the z-HEIGHT pose
/// channel; tracking the arm tip moves the centre in x/y too, so the trajectory carry needs the
/// per-axis sensitivity. `contact_force_centre_total_jacobian(dir = x̂)` reuses the same explicit +
/// implicit structure with `dir` substituted — the curvature term is axis-generic (its L0 leaf is
/// `soft_pose_sensitivity::sphere_pose_sensitivity_lateral_matches_resolve_fd`). Proves the L1
/// per-axis sensitivity BEFORE any trajectory threading.
#[test]
fn sphere_total_force_jacobian_wrt_lateral_centre_matches_resolve_fd() {
    let model = load_model(PLATEN_MJCF).expect("platen MJCF loads");
    let mut data = model.make_data();
    data.forward(&model).expect("initial forward");
    let mut coupling: StaggeredCoupling = StaggeredCoupling::new(
        model, data, 1, 0.005, 4, 0.1, 3.0e4, 1.0e-3, KAPPA, 1.0e-2, 12.0,
    )
    .with_sphere_collider(SPHERE_R);

    // Pose the sphere over the block centre at the engaged depth (south pole ~1.5 mm in), then probe
    // the LATERAL sensitivity by re-pointing the centre override along x̂ (the moving-tip channel).
    let h = 0.0985;
    let centre = Vec3::new(0.05, 0.05, h + SPHERE_R);
    coupling.set_sphere_center(centre);
    let dir = Vec3::new(1.0, 0.0, 0.0);

    // Analytic total = explicit (∂force/∂cₓ|_x) + implicit (∂force/∂x · ∂x*/∂cₓ).
    let analytic = coupling.contact_force_centre_total_jacobian(h, dir);

    // Black-box oracle: re-point the centre ± ε along x̂ and central-difference the contact force.
    let eps = 1.0e-6;
    coupling.set_sphere_center(centre + dir * eps);
    let fp = coupling.resolved_contact_force(h);
    coupling.set_sphere_center(centre - dir * eps);
    let fm = coupling.resolved_contact_force(h);
    let fd = (fp - fm) / (2.0 * eps);

    let rel = (analytic - fd).norm() / fd.norm();
    eprintln!("sphere lateral ∂force/∂cₓ: analytic={analytic:?}  FD={fd:?}  rel={rel:.3e}");
    assert!(
        fd.norm() > 1.0,
        "degenerate gate: lateral re-solve FD ≈ 0 ({}) — sphere not engaging?",
        fd.norm()
    );
    assert!(
        rel < 1e-6,
        "lateral centre Jacobian disagrees with re-solve FD (rel {rel:.3e}): {analytic:?} vs {fd:?}"
    );
}
