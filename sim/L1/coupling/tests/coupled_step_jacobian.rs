//! Explicit single-step cross-engine velocity Jacobian, assembled + FD-gated
//! (keystone S2).
//!
//! Composes the two interface factors into one coupled-step gradient:
//! `∂(platen vz')/∂(plane height)` = (rigid factor `∂vz'/∂fz`) · (`∂xfrc/∂height`),
//! where `xfrc_z = −force_on_soft_z` so `∂xfrc/∂height = −∂force/∂height` (the
//! analytic S1 factor). This is the *explicit* (fixed soft-positions) Jacobian —
//! the contact-engaged-regime building block; the implicit soft-re-equilibration
//! term (`∂x*/∂height`, needs a soft-pose VJP) is the S3 leaf.
//!
//! Gates: (1) the rigid factor matches the closed-form free-body semi-implicit
//! Euler response `dt/m`; (2) the analytically-assembled `∂vz'/∂height` matches a
//! black-box explicit finite difference. See `docs/keystone/s2_differentiability_recon.md`.

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

const DT: f64 = 1.0e-3;
const MASS: f64 = 0.2;

#[test]
fn explicit_velocity_jacobian_wrt_height_matches_assembly_and_fd() {
    let model = load_model(PLATEN_MJCF).expect("platen MJCF loads");
    let mut data = model.make_data();
    data.forward(&model).expect("initial forward");
    let coupling = StaggeredCoupling::new(
        model, data, 1, 0.005, 4, 0.1, 3.0e4, 1.0e-3, 3.0e4, 1.0e-2, 12.0,
    );

    // --- rigid factor ∂vz'/∂fz (FD over the rigid step) vs closed-form dt/m ---
    let (f0, df) = (5.0_f64, 1.0e-2);
    let r =
        (coupling.rigid_step_probe(f0 + df).1 - coupling.rigid_step_probe(f0 - df).1) / (2.0 * df);
    let analytic_rigid = DT / MASS; // semi-implicit Euler free body
    eprintln!("rigid factor ∂vz'/∂fz: FD={r:.6e}  dt/m={analytic_rigid:.6e}");
    assert!(
        (r - analytic_rigid).abs() / analytic_rigid < 1e-6,
        "rigid factor {r} != dt/m {analytic_rigid}"
    );

    // --- assemble ∂vz'/∂height = r · (−∂force_z/∂height) at a deeply-engaged config ---
    // h penetrates the top face ~1 mm (active set = 25 top vertices, stable across the FD step).
    let h = 0.099;
    let soft = coupling.contact_force_height_jacobian(h).z; // analytic ∂force_z/∂height = +κ·25
    let assembled = r * (-soft); // ∂xfrc/∂height = −∂force/∂height

    // --- black-box explicit FD: force at fixed positions → rigid step → vz' ---
    let dh = 1.0e-6;
    let vz_plus = coupling
        .rigid_step_probe(-coupling.contact_force_at_height(h + dh).z)
        .1;
    let vz_minus = coupling
        .rigid_step_probe(-coupling.contact_force_at_height(h - dh).z)
        .1;
    let black_box = (vz_plus - vz_minus) / (2.0 * dh);

    eprintln!("∂vz'/∂height: assembled={assembled:.6e}  black-box FD={black_box:.6e}");
    let rel = (assembled - black_box).abs() / black_box.abs();
    assert!(
        rel < 1e-6,
        "assembled vs black-box mismatch (rel {rel:.2e}): {assembled} vs {black_box}"
    );
    // Physical: raising the plane reduces penetration → less up-force → lower vz' (< 0).
    assert!(
        assembled < 0.0,
        "expected ∂vz'/∂height < 0; got {assembled}"
    );
    eprintln!(
        "✓ explicit coupled-step velocity Jacobian: analytic-soft × rigid-factor = black-box (rel {rel:.2e})"
    );
}
