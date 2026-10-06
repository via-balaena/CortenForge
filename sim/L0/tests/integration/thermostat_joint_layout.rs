//! Thermostat components read a DOF's position through its joint.
//!
//! `qpos` and `qvel` share indices only until the first ball or free joint: a free joint
//! has 7 position coordinates and 6 velocity DOFs. Here a free body comes first, so slide
//! DOFs 6 and 7 sit at `qpos[7]` and `qpos[8]`.

use sim_mjcf::load_model;
use sim_thermostat::{
    DoubleWellPotential, PairwiseCoupling, PassiveStack, RatchetPotential, ThermostatError,
};

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
        .try_install(&mut model)
        .expect("the stack installs");

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

/// A slide (DOF 0, qpos 0), a ball (DOFs 1–3, qpos 1–4), a hinge (DOF 4, qpos 5) and a free
/// body (DOFs 5–10, qpos 6–12; translations 5–7 at qpos 6–8).
const MIXED: &str = r#"<mujoco>
  <option timestep="0.001" gravity="0 0 0"/>
  <worldbody>
    <body name="s" pos="0 0 0">
      <joint name="slide" type="slide" axis="1 0 0"/>
      <geom type="sphere" size="0.05" mass="1"/>
    </body>
    <body name="b" pos="0 1 0">
      <joint name="ball" type="ball"/>
      <geom type="sphere" size="0.05" mass="1"/>
    </body>
    <body name="h" pos="0 2 0">
      <joint name="hinge" type="hinge" axis="0 0 1"/>
      <geom type="box" size="0.1 0.02 0.02" mass="1"/>
    </body>
    <body name="f" pos="0 3 1">
      <freejoint/>
      <geom type="sphere" size="0.1" mass="1"/>
    </body>
  </worldbody>
</mujoco>"#;

#[test]
fn components_read_hinge_and_free_translation_positions_after_a_ball() {
    let mut model = load_model(MIXED).expect("MJCF loads");
    assert_eq!((model.nq, model.nv), (13, 11));

    // Double wells on the hinge (DOF 4) and the free body's y translation (DOF 6), and a
    // coupling J = 0.5 between the slide (DOF 0) and the hinge.
    PassiveStack::builder()
        .with(DoubleWellPotential::new(1.0, 1.0, 4))
        .with(DoubleWellPotential::new(1.0, 1.0, 6))
        .with(PairwiseCoupling::new(vec![0.5], vec![(0, 4)]))
        .build()
        .try_install(&mut model)
        .expect("the stack installs");

    let mut data = model.make_data();
    let (x0, x4, x6) = (0.2, 0.5, -0.4);
    data.qpos[0] = x0;
    data.qpos[5] = x4;
    data.qpos[7] = 3.0 + x6; // the free body starts at y = 3
    data.forward(&model).expect("forward");

    // F = −4x(x² − 1) for each well; the coupling adds J·x_other to both ends.
    let well = |x: f64| -4.0 * x * (x * x - 1.0);
    let y = 3.0 + x6;
    let expected = [(0, 0.5 * x4), (4, well(x4) + 0.5 * x0), (6, well(y))];
    for (dof, f) in expected {
        assert!(
            (data.qfrc_passive[dof] - f).abs() < 1e-12,
            "DOF {dof}: expected {f}, got {}",
            data.qfrc_passive[dof]
        );
    }
}

#[test]
fn position_readers_refuse_ball_and_free_rotation_dofs() {
    for dof in [2, 9] {
        let mut model = load_model(MIXED).expect("MJCF loads");
        let verdict = PassiveStack::builder()
            .with(DoubleWellPotential::new(1.0, 1.0, dof))
            .build()
            .try_install(&mut model);
        assert_eq!(
            verdict,
            Err(ThermostatError::NoPositionCoordinate {
                component: "DoubleWellPotential",
                dof
            })
        );
    }
}
