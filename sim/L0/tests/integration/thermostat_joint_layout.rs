//! Thermostat components read a DOF's position through its joint.
//!
//! `qpos` and `qvel` share indices only until the first ball or free joint: a free joint
//! has 7 position coordinates and 6 velocity DOFs. Here a free body comes first, so slide
//! DOFs 6 and 7 sit at `qpos[7]` and `qpos[8]`.

use sim_mjcf::load_model;
use sim_thermostat::{DoubleWellPotential, PairwiseCoupling, PassiveStack, RatchetPotential};

const MJCF: &str = r#"<mujoco>
  <option timestep="0.001" gravity="0 0 0"/>
  <worldbody>
    <body name="free" pos="0 0 1">
      <freejoint/>
      <geom type="sphere" size="0.1" mass="1"/>
    </body>
    <body name="p0" pos="0 1 0">
      <joint name="x0" type="slide" axis="1 0 0"/>
      <geom type="sphere" size="0.05" mass="1"/>
    </body>
    <body name="p1" pos="0 2 0">
      <joint name="x1" type="slide" axis="1 0 0"/>
      <geom type="sphere" size="0.05" mass="1"/>
    </body>
  </worldbody>
  <actuator>
    <motor joint="x1"/>
  </actuator>
</mujoco>"#;

#[test]
fn components_read_slide_positions_after_a_free_joint() {
    let mut model = load_model(MJCF).expect("MJCF loads");
    assert_eq!((model.nq, model.nv), (9, 8));

    // A double well on slide DOF 6 (ΔV = 1, x₀ = 1), a coupling J = 0.5 between DOFs 6 and
    // 7, and a ratchet on DOF 7 driven by control 0.
    let coupling = || PairwiseCoupling::new(vec![0.5], vec![(6, 7)]);
    let ratchet = || RatchetPotential::new(1.0, 0.25, 0.3, 1.0, 7, 0);
    PassiveStack::builder()
        .with(DoubleWellPotential::new(1.0, 1.0, 6))
        .with(coupling())
        .with(ratchet())
        .build()
        .install(&mut model);

    let mut data = model.make_data();
    let (x6, x7) = (0.5, -0.3);
    data.qpos[7] = x6;
    data.qpos[8] = x7;
    data.ctrl[0] = 1.0;
    data.forward(&model).expect("forward");

    // F₆ = −V′(x₆) + J·x₇ = −4x₆(x₆² − 1) + 0.5·x₇ = 1.5 − 0.15;
    // F₇ = J·x₆ + the ratchet's force at x₇.
    let f6 = 1.5 + 0.5 * x7;
    let f7 = 0.5f64.mul_add(x6, ratchet().force(x7, 1.0));
    assert!(
        (data.qfrc_passive[6] - f6).abs() < 1e-12,
        "DOF 6: expected {f6}, got {}",
        data.qfrc_passive[6]
    );
    assert!(
        (data.qfrc_passive[7] - f7).abs() < 1e-12,
        "DOF 7: expected {f7}, got {}",
        data.qfrc_passive[7]
    );
    // V = −J·x₆·x₇.
    let energy = coupling()
        .coupling_energy(&model, &data)
        .expect("DOFs 6 and 7 are slides");
    assert!(
        (energy - (-0.5 * x6 * x7)).abs() < 1e-12,
        "coupling energy {energy}"
    );
}
