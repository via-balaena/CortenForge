//! Total single-step contact-force sensitivity to the plane pose, assembled +
//! FD-gated (keystone S3) — the implicit soft-re-equilibration term that lifts
//! the explicit (fixed-soft-position) coupled-step Jacobian to the total.
//!
//! `d force/d height = ∂force/∂h|_x + (∂force/∂x)·(∂x*/∂h)`:
//! - the first term is the S1 explicit factor (`contact_force_height_jacobian`),
//! - the second is the S3 implicit term — the penalty force Jacobian
//!   `∂force/∂x = −κ·n̂⊗n̂` contracted against the soft solver's pose sensitivity
//!   `∂x*/∂h` (`equilibrium_pose_sensitivity`), the autograd path that was
//!   load-only before S3.
//!
//! Unlike S2's gate (the free-body rigid response is exactly affine, so its
//! FD-vs-assembly agreement is an affine identity), THIS gate is a genuinely
//! independent numeric check: the oracle re-runs the full NONLINEAR
//! backward-Euler soft Newton solve at perturbed plane heights
//! (`resolved_contact_force(h ± ε)`) and central-differences the resulting
//! contact force — touching none of the analytic `∂x*/∂h` / `A⁻¹` machinery.
//! Agreement therefore validates the implicit IFT term itself.
//!
//! Scope: contact-engaged, stable-active-set regime (the penalty active-set
//! boundary is non-smooth — IPC the deferred cure); hard penalty
//! (`d²E/dsd² = κ`). See `docs/keystone/s3_soft_pose_sensitivity_recon.md`.

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

#[test]
fn total_force_jacobian_wrt_height_matches_resolve_fd() {
    let model = load_model(PLATEN_MJCF).expect("platen MJCF loads");
    let mut data = model.make_data();
    data.forward(&model).expect("initial forward");
    // Same block / contact params as the S1/S2 gates (n_per_edge=4, edge=0.1,
    // μ=3e4, dt=1e-3, κ=3e4, d̂=1e-2). The soft state starts at rest.
    let coupling: StaggeredCoupling = StaggeredCoupling::new(
        model, data, 1, 0.005, 4, 0.1, 3.0e4, 1.0e-3, KAPPA, 1.0e-2, 12.0,
    );

    // Deeply-engaged height: the plane penetrates the rest top face (z=0.1) by
    // ~1 mm, so the active set is the 25 top vertices and stays stable across
    // the FD perturbation (the next vertex layer at z=0.075 has sd=0.024 > d̂).
    let h = 0.099;

    // Analytic total = explicit (S1) + implicit (S3 via ∂x*/∂h).
    let analytic = coupling.contact_force_height_total_jacobian(h).z;
    // The explicit-only S1 factor, for contrast — it is the fixed-soft-position
    // partial and (in an engaged regime) over-predicts the true total.
    let explicit_only = coupling.contact_force_height_jacobian(h).z;

    // Black-box oracle: re-solve the nonlinear soft step at h ± ε and
    // central-difference the resulting contact force. No analytic machinery.
    let dh = 1.0e-6;
    let fd = (coupling.resolved_contact_force(h + dh).z
        - coupling.resolved_contact_force(h - dh).z)
        / (2.0 * dh);

    eprintln!(
        "S3 total ∂force_z/∂h: analytic={analytic:.6e}  re-solve FD={fd:.6e}  \
         (explicit-only S1={explicit_only:.6e})"
    );

    // Non-degenerate gate: the re-solve FD must be a real, nonzero slope.
    assert!(
        fd.abs() > 1.0,
        "degenerate gate: re-solve FD slope ≈ 0 ({fd}) — no active contact?"
    );
    // The implicit term must actually contribute — the total differs from the
    // explicit-only partial by a material amount (S3 is not a no-op here).
    assert!(
        (analytic - explicit_only).abs() / explicit_only.abs() > 1e-3,
        "implicit term negligible (analytic {analytic} ≈ explicit-only {explicit_only}) — \
         the soft re-equilibration should move the total"
    );
    // HEADLINE: the analytically-assembled total (with the S3 implicit term)
    // matches the independent nonlinear re-solve FD (observed rel ~6e-11). The
    // bound is looser than S1's machine-exact 2e-12 because the oracle re-runs
    // Newton to tol 1e-10 and the central difference divides that ~1e-10 floor
    // by 2ε ≈ 2e-6; 1e-7 leaves >1000× headroom while still failing hard if the
    // implicit term were dropped (that would be rel ≈ 3.6).
    let rel = (analytic - fd).abs() / fd.abs();
    assert!(
        rel < 1e-7,
        "total Jacobian disagrees with re-solve FD (rel {rel:.3e}): {analytic} vs {fd}"
    );
    eprintln!(
        "✓ S3 total single-step force Jacobian matches the nonlinear re-solve FD (rel {rel:.2e})"
    );
}
