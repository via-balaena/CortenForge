//! Body accumulators (`cacc`, `cfrc_ext`, `cfrc_int`) and the sensors that
//! read them, against MuJoCo 3.5.0's `mj_rnePostConstraint`
//! (`engine_core_smooth.c:2492-2683`).
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
    assert_trajectory("free_box", TOL);
}

/// The case at its state, then its state and sensordata after the golden's steps.
fn assert_trajectory(name: &str, tol: f64) {
    let (model, mut data) = assert_case(name, tol);
    let traj = &golden()["cases"][name]["traj"];
    let last = traj
        .as_object()
        .expect("traj")
        .keys()
        .map(|k| k.parse::<usize>().expect("step"))
        .max()
        .expect("steps");
    for step in 1..=last {
        data.step(&model).expect("step");
        if let Some(c) = traj.get(step.to_string()) {
            let what = format!("{name} step {step}");
            assert_close(
                &format!("{what} qpos"),
                data.qpos.as_slice(),
                &floats(&c["qpos"]),
                tol,
            );
            assert_close(
                &format!("{what} qvel"),
                data.qvel.as_slice(),
                &floats(&c["qvel"]),
                tol,
            );
            assert_close(
                &format!("{what} sensordata"),
                data.sensordata.as_slice(),
                &floats(&c["sensordata"]),
                tol,
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

/// A free sphere 2 mm into a plane, spinning and rolling, with condim 3, 4
/// and 6, and two spheres stacked on the plane (two contacts the generator's
/// `distinct` holds apart). With MuJoCo's `efc_force` and `qacc` written in,
/// `cfrc_ext` and the force and torque sensors read MuJoCo's: each contact
/// decoded as `mj_contactForce` (`engine_core_util.c:891-908`), every pyramid
/// edge in the normal force and the torsional and rolling components as
/// torques.
#[test]
fn pyramidal_contact_wrench_decodes_as_mj_contact_force() {
    for condim in [3, 4, 6] {
        assert_decoded(&format!("contact_c{condim}_pyramidal"));
    }
    assert_decoded("contact_two_contacts");
}

/// The same under the elliptic cone, and a contact whose two bodies both move
/// (`−` on body1, `+` on body2).
#[test]
fn elliptic_contact_wrench_decodes_as_mj_contact_force() {
    for condim in [3, 4, 6] {
        assert_decoded(&format!("contact_c{condim}_elliptic"));
    }
    assert_decoded("contact_two_bodies");
}

/// The case's contacts against MuJoCo's, then `cfrc_ext` and the sensordata
/// from MuJoCo's `efc_force` and `qacc`.
fn assert_decoded(name: &str) {
    let case = &golden()["cases"][name];
    let (model, mut data) = forward(case);
    let contacts = case["contacts"].as_array().expect("contacts");
    assert_eq!(data.ncon, contacts.len(), "{name}: contacts");
    for (c, want) in data.contacts.iter().zip(contacts) {
        assert_eq!(
            c.dim as u64,
            want["dim"].as_u64().expect("dim"),
            "{name}: dim"
        );
        let frame = floats(&want["frame"]);
        assert_close(
            &format!("{name} pos"),
            c.pos.as_slice(),
            &floats(&want["pos"]),
            TOL,
        );
        assert_close(
            &format!("{name} normal"),
            c.normal.as_slice(),
            &frame[0..3],
            TOL,
        );
        assert_close(
            &format!("{name} t1"),
            c.frame[0].as_slice(),
            &frame[3..6],
            TOL,
        );
        assert_close(
            &format!("{name} t2"),
            c.frame[1].as_slice(),
            &frame[6..9],
            TOL,
        );
    }
    let efc_force = floats(&case["efc_force"]);
    assert_eq!(data.efc_force.len(), efc_force.len(), "{name}: rows");
    data.efc_force.as_mut_slice().copy_from_slice(&efc_force);
    data.qacc.copy_from_slice(&floats(&case["qacc"]));
    data.flg_rnepost = false;
    sim_core::sensor::mj_sensor_acc(&model, &mut data);
    assert_close(
        &format!("{name} sensordata"),
        data.sensordata.as_slice(),
        &floats(&case["sensordata"]),
        TOL,
    );
    for b in 0..model.nbody {
        assert_close(
            &format!("{name} cfrc_ext[{b}]"),
            data.cfrc_ext[b].as_slice(),
            &floats(&case["cfrc_ext"][b]),
            TOL,
        );
    }
}

/// A connect between two moving bodies: each body's `cfrc_ext` takes the
/// rows' force at its own anchor, `+` on body1 and `−` on body2.
#[test]
fn connect_between_two_moving_bodies_matches_mujoco_3_5_0() {
    assert_trajectory("connect_two_bodies", FORCE_TOL);
}

/// A weld between a hinged body and a free one: its force and torque reach
/// both bodies at their anchors.
#[test]
fn weld_between_two_moving_bodies_matches_mujoco_3_5_0() {
    assert_trajectory("weld_two_bodies", FORCE_TOL);
}

/// A contact that involves a flex adds nothing to `cfrc_ext`: MuJoCo's loop
/// skips it (`engine_core_smooth.c:2530-2533`). A flex pressed into a sphere
/// on a slide leaves the sphere's `cfrc_ext` and the vertices' at zero while
/// its contacts carry force.
#[test]
fn a_flex_contact_stays_out_of_cfrc_ext() {
    let model = load_model(
        r#"<mujoco>
          <option timestep="0.001"/>
          <worldbody>
            <body name="sphere" pos="0 0 -0.5">
              <joint type="slide" axis="0 0 1"/>
              <geom type="sphere" size="1.0" mass="100"/>
              <site name="s" pos="0.1 0 0"/>
            </body>
          </worldbody>
          <deformable>
            <flex name="f" dim="2" density="1000" radius="0.02">
              <contact condim="1"/>
              <vertex pos="-0.15 -0.15 0.49  0.15 -0.15 0.49  -0.15 0.15 0.49  0.15 0.15 0.49"/>
              <element data="0 1 2  1 3 2"/>
            </flex>
          </deformable>
          <sensor>
            <force site="s"/>
          </sensor>
        </mujoco>"#,
    )
    .expect("load");
    let mut data = model.make_data();
    data.forward(&model).expect("forward");
    let flex_force: f64 = (0..data.efc_type.len())
        .filter(|&r| {
            matches!(
                data.efc_type[r],
                sim_core::ConstraintType::ContactFrictionless
                    | sim_core::ConstraintType::ContactPyramidal
                    | sim_core::ConstraintType::ContactElliptic
            ) && data.contacts[data.efc_id[r]].flex_vertex.is_some()
        })
        .map(|r| data.efc_force[r])
        .sum();
    assert!(
        flex_force > 0.0,
        "the flex's contacts carry force: {flex_force}"
    );
    for b in 0..model.nbody {
        assert_eq!(data.cfrc_ext[b].as_slice(), &[0.0; 6], "cfrc_ext[{b}]");
    }
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

/// An accelerometer or a framelinacc on a static body (welded to the world,
/// or mocap) reads gravity, as one on an identical box held at rest by a
/// vertical hinge does. MuJoCo reads 0 on the static ones
/// (`mj_objectAcceleration` returns 0 for a static body,
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
            <body name="mocap" mocap="true" pos="2 0 1">
              <geom type="box" size="0.1 0.1 0.1" contype="0" conaffinity="0"/>
              <site name="s_mocap" pos="0.05 0 0" quat="0.9 0.1 0.3 0.2"/>
            </body>
          </worldbody>
          <sensor>
            <accelerometer site="s_welded"/>
            <accelerometer site="s_hinged"/>
            <accelerometer site="s_mocap"/>
            <framelinacc objtype="site" objname="s_welded"/>
            <framelinacc objtype="site" objname="s_hinged"/>
            <framelinacc objtype="site" objname="s_mocap"/>
          </sensor>
        </mujoco>"#,
    )
    .expect("load");
    let mut data = model.make_data();
    data.forward(&model).expect("forward");
    let s = data.sensordata.as_slice();
    let g = data.site_xmat[0].transpose() * nalgebra::Vector3::new(0.0, 0.0, 9.81);
    assert_close("welded accelerometer", &s[0..3], g.as_slice(), TOL);
    assert_eq!(
        &s[0..3],
        &s[3..6],
        "welded and hinged accelerometers at rest"
    );
    assert_eq!(&s[0..3], &s[6..9], "welded and mocap accelerometers");
    assert_close("welded framelinacc", &s[9..12], &[0.0, 0.0, 9.81], TOL);
    assert_eq!(
        &s[9..12],
        &s[12..15],
        "welded and hinged framelinacc at rest"
    );
    assert_eq!(&s[9..12], &s[15..18], "welded and mocap framelinacc");
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
