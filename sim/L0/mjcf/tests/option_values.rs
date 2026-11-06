//! `<option>` values as MuJoCo 3.5.0 loads them (book L28; ledger L24, L131).
//!
//! MuJoCo's reader and compiler check no `<option>` number but
//! `actuatorgroupdisable`'s groups (`xml_native_reader.cc:1181-1229`), so
//! every document here loads in
//! MuJoCo 3.5.0 (measured) and keeps what it writes. This crate refuses a
//! timestep that is not above zero (`D-OPTION-TIMESTEP`) and keeps a negative
//! count as zero (`D-OPTION-NEGATIVE-COUNT`). The whole documents are in the
//! parity census, which runs each against MuJoCo's.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::float_cmp)]

use sim_core::Model;
use sim_mjcf::{MjcfError, load_model};

/// A box resting on a plane, under an `<option>` with `attributes`.
fn resting(attributes: &str) -> String {
    format!(
        r#"<mujoco><option {attributes}/><worldbody><geom type="plane" size="1 1 0.1"/><body pos="0 0 0.05"><freejoint/><geom type="box" size="0.05 0.05 0.05" mass="1"/></body></worldbody></mujoco>"#
    )
}

/// Loads `xml` and steps it 100 times, as the census does.
fn loads_and_steps(xml: &str) -> Model {
    let model = load_model(xml).unwrap();
    let mut data = model.make_data();
    for _ in 0..100 {
        data.step(&model).unwrap();
    }
    model
}

#[test]
fn a_timestep_above_one_loads() {
    let model = loads_and_steps(
        r#"<mujoco model="timestep above one">
  <option timestep="1.5"/>
  <worldbody>
    <body pos="0 0 0.05">
      <freejoint/>
      <geom type="box" size="0.05 0.05 0.05" mass="1"/>
    </body>
  </worldbody>
</mujoco>"#,
    );
    assert_eq!(model.timestep, 1.5);
}

#[test]
fn no_solver_iterations_and_a_zero_tolerance_load() {
    let model = loads_and_steps(
        r#"<mujoco model="no iterations, zero tolerance">
  <option iterations="0" tolerance="0"/>
  <worldbody>
    <geom type="plane" size="1 1 0.1"/>
    <body pos="0 0 0.05">
      <freejoint/>
      <geom type="box" size="0.05 0.05 0.05" mass="1"/>
    </body>
  </worldbody>
</mujoco>"#,
    );
    assert_eq!((model.solver_iterations, model.solver_tolerance), (0, 0.0));
}

#[test]
fn an_impratio_of_zero_loads_under_the_elliptic_cone() {
    let model = loads_and_steps(
        r#"<mujoco model="impratio zero, elliptic">
  <option impratio="0" cone="elliptic"/>
  <worldbody>
    <geom type="plane" size="1 1 0.1"/>
    <body pos="0 0 0.05">
      <freejoint/>
      <geom type="box" size="0.05 0.05 0.05" mass="1"/>
    </body>
  </worldbody>
</mujoco>"#,
    );
    assert_eq!(model.impratio, 0.0);
}

#[test]
fn a_negative_impratio_loads() {
    let model = loads_and_steps(
        r#"<mujoco model="impratio negative">
  <option impratio="-1"/>
  <worldbody>
    <geom type="plane" size="1 1 0.1"/>
    <body pos="0 0 0.05">
      <freejoint/>
      <geom type="box" size="0.05 0.05 0.05" mass="1"/>
    </body>
  </worldbody>
</mujoco>"#,
    );
    assert_eq!(model.impratio, -1.0);
}

/// A falling body, so the medium acts on it.
#[test]
fn a_negative_density_and_viscosity_load() {
    let model = loads_and_steps(
        r#"<mujoco model="density and viscosity negative">
  <option density="-1" viscosity="-1"/>
  <worldbody>
    <body pos="0 0 1">
      <freejoint/>
      <geom type="box" size="0.05 0.1 0.2" mass="1"/>
    </body>
  </worldbody>
</mujoco>"#,
    );
    assert_eq!((model.density, model.viscosity), (-1.0, -1.0));
}

/// Each value MuJoCo stores as written, stored as written here.
#[test]
fn values_mujoco_takes_are_kept_as_written() {
    let cases: [(&str, fn(&Model) -> f64, f64); 10] = [
        (r#"timestep="2""#, |m| m.timestep, 2.0),
        (r#"timestep="1""#, |m| m.timestep, 1.0),
        (r#"timestep="1e-12""#, |m| m.timestep, 1e-12),
        (r#"tolerance="-1""#, |m| m.solver_tolerance, -1.0),
        (r#"tolerance="-1e-8""#, |m| m.solver_tolerance, -1e-8),
        (r#"impratio="1e-300""#, |m| m.impratio, 1e-300),
        (r#"ls_tolerance="-1""#, |m| m.ls_tolerance, -1.0),
        (r#"noslip_tolerance="0""#, |m| m.noslip_tolerance, 0.0),
        (r#"ccd_tolerance="-1""#, |m| m.ccd_tolerance, -1.0),
        (r#"sleep_tolerance="-1""#, |m| m.sleep_tolerance, -1.0),
    ];
    for (attribute, read, written) in cases {
        let model = load_model(&resting(attribute)).unwrap();
        assert_eq!(read(&model), written, "{attribute}");
    }
    let counts: [(&str, fn(&Model) -> usize); 2] = [
        (r#"ls_iterations="0""#, |m| m.ls_iterations),
        (r#"sdf_initpoints="0""#, |m| m.sdf_initpoints),
    ];
    for (attribute, read) in counts {
        let model = load_model(&resting(attribute)).unwrap();
        assert_eq!(read(&model), 0, "{attribute}");
    }
}

/// `D-OPTION-NEGATIVE-COUNT`: MuJoCo 3.5.0 keeps each count as written,
/// -1 (measured); this crate's counts are unsigned and its reader takes a
/// negative one as zero.
#[test]
fn a_negative_count_is_kept_as_zero() {
    let cases: [(&str, fn(&Model) -> usize); 6] = [
        ("iterations", |m| m.solver_iterations),
        ("ls_iterations", |m| m.ls_iterations),
        ("noslip_iterations", |m| m.noslip_iterations),
        ("ccd_iterations", |m| m.ccd_iterations),
        ("sdf_iterations", |m| m.sdf_iterations),
        ("sdf_initpoints", |m| m.sdf_initpoints),
    ];
    for (attribute, read) in cases {
        let model = load_model(&resting(&format!(r#"{attribute}="-1""#))).unwrap();
        assert_eq!(read(&model), 0, "{attribute}");
    }
}

/// `D-OPTION-TIMESTEP`: MuJoCo 3.5.0 loads a timestep of 0 and -0.001
/// (measured); this crate refuses each when it loads the model, as its step
/// refuses one (`D-STEP-TIMESTEP`).
#[test]
fn a_timestep_not_above_zero_is_refused() {
    for timestep in ["0", "-0.001"] {
        let refused = load_model(&resting(&format!(r#"timestep="{timestep}""#))).unwrap_err();
        assert!(
            matches!(&refused, MjcfError::InvalidValue { attribute, .. } if attribute == "timestep"),
            "{timestep}: {refused}"
        );
    }
}

/// `xml`'s `o_margin` as loaded, and the contacts it lists after a forward
/// pass: `(o_margin, count, each margin)`.
fn contacts(xml: &str) -> (f64, usize, Vec<f64>) {
    let model = load_model(xml).unwrap();
    let mut data = model.make_data();
    data.forward(&model).unwrap();
    let margins = data.contacts.iter().map(|c| c.includemargin).collect();
    (model.o_margin, data.ncon, margins)
}

/// Ledger L183(a): under `<flag override>` MuJoCo 3.5.0 keeps a negative
/// `o_margin` as written (measured): a sphere 0.02 into a plane makes no
/// contact at -0.2 and one at -0.01, whose margin is -0.01. This crate took
/// a negative one as 0 before Rigid-loading PR RL-7d, and listed a contact
/// of margin 0 for both.
#[test]
fn a_negative_override_margin_is_kept() {
    let past = contacts(
        r#"<mujoco model="override margin past the overlap">
  <option o_margin="-0.2" gravity="0 0 0">
    <flag override="enable"/>
  </option>
  <worldbody>
    <geom type="plane" size="1 1 0.1"/>
    <body pos="0 0 0.08">
      <freejoint/>
      <geom type="sphere" size="0.1" mass="1"/>
    </body>
  </worldbody>
</mujoco>"#,
    );
    assert_eq!(past, (-0.2, 0, vec![]));
    let within = contacts(
        r#"<mujoco model="override margin within the overlap">
  <option o_margin="-0.01" gravity="0 0 0">
    <flag override="enable"/>
  </option>
  <worldbody>
    <geom type="plane" size="1 1 0.1"/>
    <body pos="0 0 0.08">
      <freejoint/>
      <geom type="sphere" size="0.1" mass="1"/>
    </body>
  </worldbody>
</mujoco>"#,
    );
    assert_eq!(within, (-0.01, 1, vec![-0.01]));
}
