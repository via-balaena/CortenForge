//! Analytic contact-force-vs-plane-pose derivative, FD-gated (keystone S2-S1).
//!
//! One factor of the coupled-step Jacobian: `∂(force_on_soft)/∂(plane height)`
//! at fixed soft positions. The analytic value (`−κ·n` summed over active pairs)
//! must match a central finite difference of the contact force at perturbed
//! plane heights — evaluated in the contact-engaged regime where the active set
//! is stable across the perturbation (the penalty active-set boundary is
//! non-smooth; that cap is documented, IPC deferred). See
//! `docs/keystone/s2_differentiability_recon.md`.

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

#[test]
fn analytic_contact_force_height_jacobian_matches_finite_difference() {
    let model = load_model(PLATEN_MJCF).expect("platen MJCF loads");
    let mut data = model.make_data();
    data.forward(&model).expect("initial forward");
    // Rest config (block top at z = 0.1, bottom pinned). μ = 30 kPa, κ = 30 kPa, d̂ = 1 cm.
    let coupling: StaggeredCoupling = StaggeredCoupling::new(
        model, data, 1, 0.005, 4, 0.1, 3.0e4, 1.0e-3, 3.0e4, 1.0e-2, 12.0,
    );

    // Plane height that penetrates the top face (z = 0.1) by ~1 mm while the
    // next vertex layer (z = 0.075) stays > d̂ outside — so the active set is
    // exactly the 5×5 = 25 top-face vertices and is stable across the FD step.
    let h = 0.099;

    let analytic = coupling.contact_force_height_jacobian(h);
    let delta = 1.0e-6;
    let fd = (coupling.contact_force_at_height(h + delta)
        - coupling.contact_force_at_height(h - delta))
        / (2.0 * delta);

    eprintln!("∂force/∂height  analytic={analytic:?}  fd={fd:?}");

    // Lateral components are ~zero on both (symmetric, axis-aligned contact).
    assert!(analytic.x.abs() < 1e-6 && analytic.y.abs() < 1e-6);
    assert!(
        fd.x.abs() < 1.0 && fd.y.abs() < 1.0,
        "unexpected lateral FD: {fd:?}"
    );
    // The active set must be the 25 top-face vertices ⇒ analytic z = κ·25 = 7.5e5.
    assert!(
        (analytic.z - 3.0e4 * 25.0).abs() < 1.0,
        "expected analytic ∂force_z/∂height = κ·25 = 7.5e5; got {}",
        analytic.z
    );
    // Analytic matches the finite difference to tight relative tolerance.
    assert!(
        fd.z.abs() > 1.0,
        "degenerate gate: FD slope ≈ 0 ({fd:?}) — active set not engaged?"
    );
    let rel = (analytic.z - fd.z).abs() / fd.z.abs();
    assert!(
        rel < 1e-4,
        "analytic vs FD mismatch (rel {rel:.2e}): analytic.z={} fd.z={}",
        analytic.z,
        fd.z
    );
    eprintln!("✓ analytic ∂force/∂height matches FD to rel {rel:.2e}");
}
