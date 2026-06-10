//! Forward soft↔rigid coupling gate (Layer-2 keystone, forward half).
//!
//! Drives [`StaggeredCoupling`] on the canonical scene: a `sim-core` rigid platen
//! (free-joint box) falls under gravity onto a pinned Neo-Hookean soft block.
//! Asserts the two forward-coupling correctness properties:
//!
//! 1. **Stability** — the staggered two-way loop stays bounded and settles (no
//!    partitioned-scheme blow-up).
//! 2. **Force balance** — at the settled equilibrium the soft contact reaction
//!    matches the platen's weight (Newton's 3rd law + system equilibrium across
//!    the two engines).
//!
//! See `docs/keystone/recon.md`.

#![allow(
    // A missing/malformed fixture (MJCF load, body index) surfaces as a test
    // panic — the canonical fixture idiom in this workspace's integration tests.
    clippy::expect_used
)]

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
fn rigid_platen_settles_on_soft_block_at_force_balance() {
    let model = load_model(PLATEN_MJCF).expect("platen MJCF loads");
    let mut data = model.make_data();
    data.forward(&model).expect("initial forward");

    let weight = 0.2 * 9.81;
    let mut coupling = StaggeredCoupling::new(
        model, data, /* body */ 1, /* contact_clearance (platen half-thickness) */ 0.005,
        /* n_per_edge */ 4, /* edge */ 0.1, /* mu */ 3.0e4, /* dt */ 1.0e-3,
        /* kappa */ 3.0e4, /* d_hat */ 1.0e-2, /* rigid_damping */ 8.0,
    );

    let mut last = coupling.step();
    let mut max_abs_z = last.rigid_z.abs();
    for _ in 1..900 {
        last = coupling.step();
        max_abs_z = max_abs_z.max(last.rigid_z.abs());
    }
    let contact_f = -last.force_on_soft.z; // reaction magnitude (up on the platen)
    eprintln!(
        "settled: rigid_z={:.5} m, contact_F={contact_f:.4} N, weight={weight:.4} N",
        last.rigid_z
    );

    // (1) Stability: the loop stayed bounded (no blow-up).
    assert!(
        last.rigid_z.is_finite() && max_abs_z < 1.0 && last.rigid_z > 0.0,
        "staggered coupling diverged: rigid_z={}, max|z|={max_abs_z}",
        last.rigid_z
    );
    // (2) Force balance: settled soft reaction ≈ platen weight.
    assert!(
        (contact_f - weight).abs() / weight < 0.1,
        "force balance off: contact_F={contact_f} N vs weight={weight} N"
    );
}
