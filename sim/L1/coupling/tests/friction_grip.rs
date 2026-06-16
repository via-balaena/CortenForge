//! Keystone friction leaf PR3a — the tangential friction GRIP in the FORWARD coupled solve.
//!
//! A free-joint rigid platen presses onto a pinned Neo-Hookean soft block and is pushed
//! sideways by tilted gravity. Frictionless, the platen's tangential motion sweeps over a
//! tangentially-invariant contact (the soft block is never dragged) so it slides freely.
//! Once friction is on, the moving platen DRAGS the soft block (the moving-collider drift)
//! and the block's friction reaction HOLDS the platen — the tangential slide collapses, and
//! monotonically as `μ` grows. This gates the grip routing end-to-end: friction-aware soft
//! solve (with the collider drift) → gripped contact wrench (force + off-COM moment) → rigid
//! step.
//!
//! Forward-only (PR3a): the friction-coupled GRADIENT is PR3b. See
//! `docs/keystone/friction_recon.md` and the project memory `project-friction-leaf.md`.

// A missing/malformed fixture (MJCF load, body index) surfaces as a test panic — the
// canonical fixture idiom in this workspace's integration tests.
#![allow(clippy::expect_used)]

use sim_coupling::StaggeredCoupling;
use sim_mjcf::load_model;

// Tilted gravity: a steady sideways push `gx` plus the downward `gz` pressing the platen
// onto the block. The platen starts near vertical force balance (xpos.z = 0.115: contact
// face 0.11, block top 0.1 ⇒ sd ≈ d_hat, a gentle engaged load — a deeper start launches a
// stiff penalty spike that throws the platen off contact). `gx = 2.0` keeps `m·gx = 0.4 N`
// well inside the Coulomb cone `μ·(m·gz) ≈ μ·1.96 N` for μ ≥ 1, so friction can hold it.
const PLATEN_MJCF: &str = r#"<mujoco>
  <option gravity="2.0 0 -9.81" timestep="0.001"/>
  <worldbody>
    <body name="platen" pos="0 0 0.115">
      <freejoint/>
      <geom type="box" size="0.06 0.06 0.005" mass="0.2"/>
    </body>
  </worldbody>
</mujoco>"#;

const N_STEPS: usize = 600;
const EPS_V: f64 = 0.1; // the converging stick-band choice (PR1's friction_stick_slip gate)

/// Build the grip coupling at coefficient `mu` and roll it forward, returning the platen's
/// tangential (x) slide after the rollout.
fn x_slide(mu: f64) -> f64 {
    let model = load_model(PLATEN_MJCF).expect("platen MJCF loads");
    let mut data = model.make_data();
    data.forward(&model).expect("initial forward");
    let mut coupling: StaggeredCoupling = StaggeredCoupling::new(
        model, data, /* body */ 1, /* contact_clearance */ 0.005,
        /* n_per_edge */ 4, /* edge */ 0.1, /* mu (soft) */ 3.0e4,
        /* dt */ 1.0e-3, /* kappa */ 3.0e4, /* d_hat */ 1.0e-2,
        /* rigid_damping */ 8.0,
    )
    .with_friction(mu, EPS_V);
    let x0 = coupling.data().xpos[1].x;
    let final_pos = coupling.coupled_trajectory_grip(N_STEPS);
    // Contact-persistence guard: the grip comparison is only meaningful if the platen stayed
    // ENGAGED on the block (the penalty band is `0 < (z − clearance) − block_top < d_hat`, i.e.
    // z ∈ (0.105, 0.115) for clearance 0.005 / block top 0.1 / d_hat 0.01). A platen that flew
    // off (z ≫ 0.115) or sank through (z < 0.1) would slide frictionlessly and mask the grip —
    // assert it settled in the engaged band so a "small slide" can only mean friction held it.
    assert!(
        (0.100..0.116).contains(&final_pos.z),
        "platen left the contact band at μ={mu}: final z={} (expected engaged ≈0.105–0.115)",
        final_pos.z
    );
    final_pos.x - x0
}

#[test]
fn friction_grip_holds_the_platen_vs_frictionless_slip() {
    let slide_free = x_slide(0.0);
    let slide_mu1 = x_slide(1.0);
    let slide_mu3 = x_slide(3.0);
    eprintln!(
        "tangential slide after {N_STEPS} steps: frictionless={slide_free:.6e} m, \
         μ=1 {slide_mu1:.6e} m, μ=3 {slide_mu3:.6e} m"
    );

    // The frictionless platen genuinely slides (the scene is non-degenerate).
    assert!(
        slide_free.abs() > 1e-2,
        "degenerate: frictionless platen barely slid ({slide_free} m) — no tangential load?"
    );
    // Friction GRIPS: the slide collapses once friction is on.
    assert!(
        slide_mu1.abs() < 0.5 * slide_free.abs(),
        "μ=1 did not hold the platen: slide {slide_mu1} vs frictionless {slide_free}"
    );
    // Monotone in μ: more friction → less slide.
    assert!(
        slide_mu3.abs() <= slide_mu1.abs() + 1e-9,
        "slide not monotone in μ: μ=3 {slide_mu3} vs μ=1 {slide_mu1}"
    );
}
