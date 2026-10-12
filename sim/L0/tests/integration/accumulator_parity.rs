//! Body accumulators (`cacc`, `cfrc_ext`, `cfrc_int`) and the sensors that
//! read them, against MuJoCo 3.5.0's `mj_rnePostConstraint`
//! (`engine_core_smooth.c:2477-2681`).
//!
//! The golden is `assets/golden/accumulators/accumulators.json`, from the
//! unfused MuJoCo 3.5.0 oracle (`scripts/gen_accumulators_reference.py`,
//! which describes its cases). sim-core keeps `cfrc_ext` at each body's
//! origin `xpos`; the generator moved MuJoCo's there.

use serde_json::Value;
use sim_core::{BodyWrench, Data, Model};
use sim_mjcf::load_model;

const TOL: f64 = 1e-12;

fn golden() -> Value {
    serde_json::from_str(include_str!(
        "../assets/golden/accumulators/accumulators.json"
    ))
    .expect("accumulators golden parses")
}

fn floats(v: &Value) -> Vec<f64> {
    v.as_array()
        .expect("array")
        .iter()
        .map(|x| x.as_f64().expect("number"))
        .collect()
}

fn assert_close(what: &str, got: &[f64], want: &[f64], tol: f64) {
    assert_eq!(got.len(), want.len(), "{what}: length");
    for (i, (a, b)) in got.iter().zip(want).enumerate() {
        assert!((a - b).abs() <= tol, "{what}[{i}]: {a}, expected {b}");
    }
}

/// The case's model and its state after `forward`, with its applied wrenches.
fn forward(case: &Value) -> (Model, Data) {
    let model = load_model(case["xml"].as_str().expect("xml")).expect("load");
    let mut data = model.make_data();
    data.qpos.copy_from_slice(&floats(&case["qpos"]));
    data.qvel.copy_from_slice(&floats(&case["qvel"]));
    for (body, row) in case["xfrc_applied"].as_object().expect("xfrc_applied") {
        let row: [f64; 6] = floats(row).try_into().expect("six values");
        data.xfrc_applied[body.parse::<usize>().expect("body")] = BodyWrench::from_mujoco_row(row);
    }
    data.forward(&model).expect("forward");
    (model, data)
}

fn assert_case(name: &str, tol: f64) -> (Model, Data) {
    let case = &golden()["cases"][name];
    let (model, data) = forward(case);
    assert_close(
        &format!("{name} sensordata"),
        data.sensordata.as_slice(),
        &floats(&case["sensordata"]),
        tol,
    );
    for b in 0..model.nbody {
        assert_close(
            &format!("{name} cfrc_ext[{b}]"),
            data.cfrc_ext[b].as_slice(),
            &floats(&case["cfrc_ext"][b]),
            tol,
        );
    }
    (model, data)
}

/// A rotated free box with a hinged child: the accelerometer, framelinacc,
/// frameangacc, force and torque sensors, at a state and after steps 1, 10
/// and 100. MuJoCo's free joint takes its rotational velocity products with
/// the translational velocity already in the body velocity
/// (`engine_core_smooth.c:2301-2321`), which the accumulators now share with
/// `mj_rne`.
#[test]
fn free_body_sensors_match_mujoco_3_5_0() {
    let (model, mut data) = assert_case("free_box", TOL);
    let traj = &golden()["cases"]["free_box"]["traj"];
    for step in 1..=100 {
        data.step(&model).expect("step");
        if let Some(c) = traj.get(step.to_string()) {
            let what = format!("free_box step {step}");
            assert_close(
                &format!("{what} qpos"),
                data.qpos.as_slice(),
                &floats(&c["qpos"]),
                TOL,
            );
            assert_close(
                &format!("{what} qvel"),
                data.qvel.as_slice(),
                &floats(&c["qvel"]),
                TOL,
            );
            assert_close(
                &format!("{what} sensordata"),
                data.sensordata.as_slice(),
                &floats(&c["sensordata"]),
                TOL,
            );
        }
    }
}

/// A pendulum whose tip is connected to the world: the connect's force
/// reaches `cfrc_ext` at the anchor and the force sensor reads it.
#[test]
fn force_sensor_includes_connect_force() {
    assert_case("connect_force", FORCE_TOL);
}

/// A box on a hinge welded to the world at its own pose, turned 0.3 rad: the
/// weld's force and torque reach `cfrc_ext` at its anchors.
#[test]
fn force_sensor_includes_weld_force() {
    assert_case("weld_force", FORCE_TOL);
}

/// A free, frictionless sphere whose geom is off its body's origin, pressed
/// into a plane: cfrc_ext holds the contact's torque about the origin.
#[test]
fn contact_torque_is_taken_about_the_origin() {
    assert_case("contact_torque", TOL);
}

/// An applied wrench acts at the centre of mass; `cfrc_ext` holds it at the
/// body's origin, its torque moved there.
#[test]
fn xfrc_torque_moved_to_origin() {
    assert_case("applied_wrench", TOL);
}

/// A wrench applied to the world body reaches nothing: MuJoCo's loop over
/// applied wrenches starts at body 1.
#[test]
fn a_world_wrench_stays_out_of_cfrc_ext() {
    let (_, data) = assert_case("world_wrench", TOL);
    assert_eq!(data.cfrc_ext[0].as_slice(), &[0.0; 6]);
}

const FORCE_TOL: f64 = 1e-10;

/// An accelerometer on a body welded to the world reads gravity, as one on
/// an identical box held at rest by a vertical hinge does. MuJoCo reads 0 on
/// the welded one (`mj_objectAcceleration` returns 0 for a static body,
/// `engine_core_util.c:823-827`): registry row D-STATIC-ACCELEROMETER.
#[test]
fn static_site_reads_gravity() {
    let model = load_model(
        r#"<mujoco>
          <worldbody>
            <body name="welded" pos="0 0 1">
              <geom type="box" size="0.1 0.1 0.1" mass="1" contype="0" conaffinity="0"/>
              <site name="s_welded" pos="0.05 0 0" quat="0.9 0.1 0.3 0.2"/>
            </body>
            <body name="hinged" pos="1 0 1">
              <joint type="hinge" axis="0 0 1"/>
              <geom type="box" size="0.1 0.1 0.1" mass="1" contype="0" conaffinity="0"/>
              <site name="s_hinged" pos="0.05 0 0" quat="0.9 0.1 0.3 0.2"/>
            </body>
          </worldbody>
          <sensor>
            <accelerometer site="s_welded"/>
            <accelerometer site="s_hinged"/>
          </sensor>
        </mujoco>"#,
    )
    .expect("load");
    let mut data = model.make_data();
    data.forward(&model).expect("forward");
    let welded = &data.sensordata.as_slice()[0..3];
    let hinged = &data.sensordata.as_slice()[3..6];
    assert_eq!(welded, hinged, "welded and hinged at rest");
    let g = data.site_xmat[0].transpose() * nalgebra::Vector3::new(0.0, 0.0, 9.81);
    assert_close("welded accelerometer", welded, g.as_slice(), TOL);
}

/// A torque sensor on a world site reads the root bodies' wrenches moved to
/// the site: sim-core's `cfrc_int[0]` holds them moved to the world origin.
/// MuJoCo adds each root's `cfrc_int`, held at that root's subtree centre of
/// mass, into body 0 without moving it (`engine_core_smooth.c:2676-2679`), so
/// its torque mixes reference points: registry row D-WORLD-CFRC-INT.
#[test]
fn world_site_torque_is_the_sum_of_root_wrenches_moved_to_it() {
    let model = load_model(
        r#"<mujoco>
          <worldbody>
            <site name="w" pos="0.2 0.1 0.3" quat="0.9 0.2 -0.1 0.3"/>
            <body name="a" pos="0 0 1">
              <joint type="hinge" axis="1 0 0"/>
              <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.02" mass="1" contype="0" conaffinity="0"/>
            </body>
            <body name="b" pos="0.5 0.2 0.8">
              <joint type="hinge" axis="0 1 0"/>
              <geom type="box" size="0.05 0.04 0.03" pos="0.1 0 0" mass="2" contype="0" conaffinity="0"/>
            </body>
          </worldbody>
          <sensor>
            <torque site="w"/>
          </sensor>
        </mujoco>"#,
    )
    .expect("load");
    let mut data = model.make_data();
    data.qvel.copy_from_slice(&[0.7, -0.4]);
    data.forward(&model).expect("forward");
    let site = data.site_xpos[0];
    let mut torque = nalgebra::Vector3::zeros();
    for b in 1..model.nbody {
        let w = data.cfrc_int[b];
        let (t, f) = (
            nalgebra::Vector3::new(w[0], w[1], w[2]),
            nalgebra::Vector3::new(w[3], w[4], w[5]),
        );
        torque += t + (data.xpos[b] - site).cross(&f);
    }
    let expected = data.site_xmat[0].transpose() * torque;
    assert_close(
        "world site torque",
        data.sensordata.as_slice(),
        expected.as_slice(),
        TOL,
    );
}
