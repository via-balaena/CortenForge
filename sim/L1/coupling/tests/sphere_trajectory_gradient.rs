//! L1b free-body NORMAL curvature carry — the coupled gradients on a FINITE posed
//! sphere collider.
//!
//! The keystone gates (`coupled_load_gradient.rs`, `coupled_trajectory_gradient.rs`)
//! validate the cross-engine load gradient and the multi-step trajectory gradient for
//! the infinite `RigidPlane`. These gates re-run the SAME tape machinery
//! (`coupled_step_load_gradient`, `coupled_trajectory_material_gradient`) against the
//! SAME independent full-coupled FD oracles (`coupled_step_load_vz`, a real re-rollout)
//! but with the collider swapped to a finite `TranslatedSdf<SphereSdf>` (`with_sphere_collider`).
//!
//! Why it's a genuinely new check, not the plane gate re-skinned: a sphere's contact
//! normal turns both as a soft vertex slides over it (`∂n̂/∂x = H`) and as the primitive
//! translates with the rigid height (`∂n̂/∂h = −H·ẑ`), so the contact-force-z Jacobian
//! `fz(x*, z)` carries the geometric-stiffness term `f_mag·H` (#415) that is identically
//! zero for the plane's constant normal. If the free-body adjoint
//! (`ContactForceTrajVjp` / `ContactForceVjp`, fed by `active_pair_force_factors`) dropped
//! that term, these gates would disagree with the nonlinear oracle (the plane gates would
//! not, because the dropped term vanishes there). The single-step analytic counterpart is
//! `sphere_contact_total_jacobian.rs`; this lifts it onto the autograd tape.
//!
//! Scope: contact-engaged, stable-active-set regime (the penalty active-set boundary is
//! non-smooth — IPC the deferred cure); hard penalty (`d²E/dsd² = κ`); NORMAL contact only
//! (the free-body TANGENTIAL/friction gradients stay plane-guarded — the L1b-tangential
//! follow-on). See `docs/keystone/s3_soft_pose_sensitivity_recon.md`.

#![allow(clippy::expect_used)]

use sim_coupling::StaggeredCoupling;
use sim_mjcf::load_model;
use sim_soft::{HandBuiltTetMesh, MaterialField, VertexId, pick_vertices_by_predicate};

const KAPPA: f64 = 3.0e4;
const N_PER_EDGE: usize = 4;
const EDGE: f64 = 0.1;
const MU0: f64 = 3.0e4;
/// Sphere radius (m). Large vs the 0.1 m block so the south-pole patch spans several
/// top-face vertices (a non-degenerate active set) yet stays curved enough that the
/// geometric-stiffness term is materially nonzero (mirrors `sphere_contact_total_jacobian`).
const SPHERE_R: f64 = 0.08;

// ── Single-step S4 crossing on the sphere ────────────────────────────────────────────

/// A platen body whose pose is irrelevant to the single-step gate (it passes an explicit
/// `height`), but must exist for the rigid half of the crossing.
const PLATEN_MJCF: &str = r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="platen" pos="0 0 0.125">
      <freejoint/>
      <geom type="box" size="0.06 0.06 0.005" mass="0.2"/>
    </body>
  </worldbody>
</mujoco>"#;

fn sphere_coupling(damping: f64) -> StaggeredCoupling {
    let model = load_model(PLATEN_MJCF).expect("platen MJCF loads");
    let mut data = model.make_data();
    data.forward(&model).expect("initial forward");
    StaggeredCoupling::new(
        model, data, 1, 0.005, N_PER_EDGE, EDGE, MU0, 1.0e-3, KAPPA, 1.0e-2, damping,
    )
    .with_sphere_collider(SPHERE_R)
}

fn top_face_loaded() -> Vec<VertexId> {
    let mesh =
        HandBuiltTetMesh::uniform_block(N_PER_EDGE, EDGE, &MaterialField::uniform(MU0, 4.0 * MU0));
    pick_vertices_by_predicate(&mesh, |p| (p.z - EDGE).abs() < 1e-9)
}

/// The one-tape cross-engine load gradient `∂vz'/∂theta` matches the independent
/// full-coupled-step FD on the sphere — the S4 crossing's curved-collider lift. The
/// south pole at `height` (the sphere's `sphere_center` rides `height + radius`, so the
/// pole sits at `height`) presses into the rest top face (z = 0.1).
#[test]
fn sphere_cross_engine_backward_matches_full_coupled_fd() {
    let c = sphere_coupling(12.0);
    let loaded = top_face_loaded();
    assert!(!loaded.is_empty(), "top face must have loadable vertices");

    let check = |h: f64, theta0: f64| {
        let (vz, grad) = c.coupled_step_load_gradient(h, &loaded, theta0);
        let eps = 1.0e-4;
        let fd = (c.coupled_step_load_vz(h, &loaded, theta0 + eps)
            - c.coupled_step_load_vz(h, &loaded, theta0 - eps))
            / (2.0 * eps);
        let rel = (grad - fd).abs() / fd.abs();
        eprintln!(
            "sphere S4 @ (h={h}, θ={theta0}): vz'={vz:.6e}  tape ∂vz'/∂θ={grad:.6e}  \
             full-coupled FD={fd:.6e}  rel={rel:.3e}"
        );
        assert!(vz.is_finite(), "vz' not finite: {vz}");
        assert!(
            fd.abs() > 1e-4,
            "degenerate gate: ∂vz'/∂θ ≈ 0 ({fd}) — load not coupling to the rigid body?"
        );
        // Same 1e-6 bound as the plane S4 gate: the curved-normal stiffness composes
        // exactly through the crossing (a flat ContactForceVjp would miss the f_mag·H term).
        assert!(
            rel < 1e-6,
            "sphere tape grad disagrees with full-coupled FD at (h={h}, θ={theta0}) \
             (rel {rel:.3e}): {grad} vs {fd}"
        );
    };

    // South pole 1 mm and 3 mm into the rest top face — a shallow and a deeper engagement.
    check(0.099, 5.0);
    check(0.097, 9.0);
}

// ── Multi-step trajectory gradient on the sphere ─────────────────────────────────────

// A platen started already engaged: at z = 0.103 the sphere south pole (plane_height =
// xpos.z − clearance = 0.098) presses ~2 mm into the rest top face (z = 0.1). A short
// rollout under firm damping stays engaged with a stable active set.
const ENGAGED_MJCF: &str = r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="platen" pos="0 0 0.103">
      <freejoint/>
      <geom type="box" size="0.06 0.06 0.005" mass="0.2"/>
    </body>
  </worldbody>
</mujoco>"#;

const TRAJ_DAMPING: f64 = 40.0; // settles into the curved contact without breaking it

fn engaged_sphere_coupling(mu: f64) -> StaggeredCoupling {
    let model = load_model(ENGAGED_MJCF).expect("engaged platen MJCF loads");
    let mut data = model.make_data();
    data.forward(&model).expect("initial forward");
    StaggeredCoupling::new(
        model,
        data,
        1,
        0.005,
        N_PER_EDGE,
        EDGE,
        mu,
        1.0e-3,
        KAPPA,
        1.0e-2,
        TRAJ_DAMPING,
    )
    .with_sphere_collider(SPHERE_R)
}

/// Final platen height after `n_steps` of the REAL coupled sphere dynamics at `mu` — the
/// independent oracle (re-runs the real Newton solves + sim-core steps; `step` is
/// SDF-generic, so it drives the actual posed-sphere contact).
fn final_z(mu: f64, n_steps: usize) -> f64 {
    let mut c = engaged_sphere_coupling(mu);
    let mut z = c.data().xpos[1].z;
    for _ in 0..n_steps {
        z = c.step().rigid_z;
    }
    z
}

/// The tape's forward rollout reproduces the real coupled sphere dynamics exactly (the
/// nodes carry real `step` values; only the backward pass is analytic).
#[test]
fn sphere_trajectory_forward_matches_real_rollout() {
    let n = 8;
    let (z_n, _grad) = engaged_sphere_coupling(MU0).coupled_trajectory_material_gradient(n, 0);
    let z_ref = final_z(MU0, n);
    assert!(
        (z_n - z_ref).abs() < 1e-12,
        "tape forward z_N {z_n} != real sphere rollout {z_ref}"
    );
}

/// One `tape.backward` over a deeply-engaged coupled sphere rollout matches the
/// full-coupled FD oracle. The block ties `λ = 4μ`, so the constructor-FD (moving λ = 4μ
/// with μ) measures `d/dμ|_{λ=4μ} = ∂/∂μ + 4·∂/∂λ` — the S5 linear combination. The
/// curved-normal `f_mag·H` term (zero for the plane) is what makes this match.
#[test]
fn sphere_trajectory_gradient_engaged_matches_full_fd() {
    let n = 8;
    let grad_mu = engaged_sphere_coupling(MU0)
        .coupled_trajectory_material_gradient(n, 0)
        .1;
    let grad_lambda = engaged_sphere_coupling(MU0)
        .coupled_trajectory_material_gradient(n, 1)
        .1;
    let grad_total = grad_mu + 4.0 * grad_lambda;

    let eps = MU0 * 5e-4;
    let fd = (final_z(MU0 + eps, n) - final_z(MU0 - eps, n)) / (2.0 * eps);
    let rel = (grad_total - fd).abs() / fd.abs().max(1e-30);
    eprintln!(
        "sphere engaged: ∂/∂μ={grad_mu:.6e} ∂/∂λ={grad_lambda:.6e} \
         total(tape)={grad_total:.6e} FD={fd:.6e} rel={rel:.3e}"
    );
    assert!(grad_total.abs() > 1e-9, "gradient implausibly ~0");
    // Machine-exact like the plane trajectory gate; 1e-6 bound so a curvature-carry
    // regression in the glue VJPs — invisible to the forward rollout, which takes the real
    // `step` values — is caught immediately.
    assert!(
        rel < 1e-6,
        "one-tape sphere total dz_N/dμ {grad_total} disagrees with full-coupled FD {fd} (rel {rel:e})"
    );
}
