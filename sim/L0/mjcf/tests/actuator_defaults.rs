//! An actuator's or sensor's own written value over its class's, whatever
//! the value (book L17): one equal to the built-in default is the element's
//! too. Each value is MuJoCo 3.5.0's for the same document (measured), but
//! the sensor's: MuJoCo has no sensor default class.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::float_cmp)]

use sim_mjcf::load_model;

/// The first actuator's gain, bias and dynamics parameters.
fn params(xml: &str) -> ([f64; 9], [f64; 9], [f64; 10]) {
    let model = load_model(xml).unwrap();
    (
        model.actuator_gainprm[0],
        model.actuator_biasprm[0],
        model.actuator_dynprm[0],
    )
}

#[test]
fn a_position_keeps_its_own_kp_of_one() {
    let (gain, bias, _) = params(
        r#"<mujoco><default><default class="c"><position kp="50"/></default></default><worldbody><body><joint name="h" type="hinge"/><geom size="0.1"/></body></worldbody><actuator><position class="c" joint="h" kp="1"/></actuator></mujoco>"#,
    );
    assert_eq!((gain[0], bias[1]), (1.0, -1.0));
}

#[test]
fn a_position_takes_its_class_kp() {
    let (gain, bias, _) = params(
        r#"<mujoco><default><default class="c"><position kp="50"/></default></default><worldbody><body><joint name="h" type="hinge"/><geom size="0.1"/></body></worldbody><actuator><position class="c" joint="h"/></actuator></mujoco>"#,
    );
    assert_eq!((gain[0], bias[1]), (50.0, -50.0));
}

#[test]
fn a_cylinder_keeps_its_own_area_of_one() {
    let (gain, _, _) = params(
        r#"<mujoco><default><default class="c"><cylinder area="5"/></default></default><worldbody><body><joint name="h" type="hinge"/><geom size="0.1"/></body></worldbody><actuator><cylinder class="c" joint="h" area="1"/></actuator></mujoco>"#,
    );
    assert_eq!(gain[0], 1.0);
}

#[test]
fn a_cylinder_takes_its_class_area() {
    let (gain, _, _) = params(
        r#"<mujoco><default><default class="c"><cylinder area="5"/></default></default><worldbody><body><joint name="h" type="hinge"/><geom size="0.1"/></body></worldbody><actuator><cylinder class="c" joint="h"/></actuator></mujoco>"#,
    );
    assert_eq!(gain[0], 5.0);
}

#[test]
fn a_cylinder_keeps_its_own_bias_of_zero() {
    let (_, bias, _) = params(
        r#"<mujoco><default><default class="c"><cylinder bias="1 0 0"/></default></default><worldbody><body><joint name="h" type="hinge"/><geom size="0.1"/></body></worldbody><actuator><cylinder class="c" joint="h" bias="0 0 0"/></actuator></mujoco>"#,
    );
    assert_eq!(bias[0], 0.0);
}

#[test]
fn a_cylinder_takes_its_class_bias() {
    let (_, bias, _) = params(
        r#"<mujoco><default><default class="c"><cylinder bias="1 0 0"/></default></default><worldbody><body><joint name="h" type="hinge"/><geom size="0.1"/></body></worldbody><actuator><cylinder class="c" joint="h"/></actuator></mujoco>"#,
    );
    assert_eq!(bias[0], 1.0);
}

#[test]
fn an_adhesion_keeps_its_own_gain_of_one() {
    let (gain, _, _) = params(
        r#"<mujoco><default><default class="c"><adhesion gain="5"/></default></default><worldbody><body><joint name="h" type="hinge"/><geom size="0.1"/></body></worldbody><actuator><adhesion class="c" body="world" gain="1" ctrlrange="0 1"/></actuator></mujoco>"#,
    );
    assert_eq!(gain[0], 1.0);
}

#[test]
fn an_adhesion_takes_its_class_gain() {
    let (gain, _, _) = params(
        r#"<mujoco><default><default class="c"><adhesion gain="5"/></default></default><worldbody><body><joint name="h" type="hinge"/><geom size="0.1"/></body></worldbody><actuator><adhesion class="c" body="world" ctrlrange="0 1"/></actuator></mujoco>"#,
    );
    assert_eq!(gain[0], 5.0);
}

/// Every muscle parameter but `force` written at its built-in default, under
/// a class that writes each: the element's own, and the class's force.
#[test]
fn a_muscle_keeps_its_own_values_at_their_defaults() {
    let (gain, bias, dynamics) = params(
        r#"<mujoco><default><default class="c"><muscle range="0.5 1.5" force="10" scale="100" lmin="0.4" lmax="1.7" vmax="2" fpmax="1.4" fvmax="1.3" timeconst="0.02 0.05"/></default></default><worldbody><body><joint name="h" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody><actuator><muscle class="c" joint="h" range="0.75 1.05" scale="200" lmin="0.5" lmax="1.6" vmax="1.5" fpmax="1.3" fvmax="1.2" timeconst="0.01 0.04"/></actuator></mujoco>"#,
    );
    let expected = [0.75, 1.05, 10.0, 200.0, 0.5, 1.6, 1.5, 1.3, 1.2];
    assert_eq!(gain, expected);
    assert_eq!(bias, expected);
    assert_eq!((dynamics[0], dynamics[1]), (0.01, 0.04));
}

#[test]
fn a_muscle_takes_its_class_values() {
    let (gain, _, dynamics) = params(
        r#"<mujoco><default><default class="c"><muscle range="0.5 1.5" force="10" scale="100" lmin="0.4" lmax="1.7" vmax="2" fpmax="1.4" fvmax="1.3" timeconst="0.02 0.05"/></default></default><worldbody><body><joint name="h" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody><actuator><muscle class="c" joint="h"/></actuator></mujoco>"#,
    );
    assert_eq!(gain, [0.5, 1.5, 10.0, 100.0, 0.4, 1.7, 2.0, 1.4, 1.3]);
    assert_eq!((dynamics[0], dynamics[1]), (0.02, 0.05));
}

/// MuJoCo reads a negative `force` as unwritten (`mjs_setToMuscle`): the
/// class's force, or -1 (computed when the model is made) without one.
#[test]
fn a_muscle_negative_force_takes_its_class_force() {
    let (gain, _, _) = params(
        r#"<mujoco><default><default class="c"><muscle force="77"/></default></default><worldbody><body><joint name="h" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody><actuator><muscle class="c" joint="h" force="-1"/></actuator></mujoco>"#,
    );
    assert_eq!(gain[2], 77.0);
}

#[test]
fn a_gear_of_one_is_the_elements_own() {
    let model = load_model(
        r#"<mujoco><default><default class="c"><motor gear="50"/></default></default><worldbody><body><joint name="h" type="hinge"/><geom size="0.1"/></body></worldbody><actuator><motor class="c" joint="h" gear="1"/></actuator></mujoco>"#,
    )
    .unwrap();
    assert_eq!(model.actuator_gear[0][0], 1.0);
}

/// A sensor's own noise and cutoff of 0 over its class's (sensor classes are
/// sim-mjcf's: MuJoCo 3.5.0's schema has no `<default><sensor>`).
#[test]
fn a_sensor_keeps_its_own_noise_and_cutoff_of_zero() {
    let model = load_model(
        r#"<mujoco><default><default class="c"><sensor noise="0.5" cutoff="2"/></default></default><worldbody><body><joint name="h" type="hinge"/><geom size="0.1"/></body></worldbody><sensor><jointpos class="c" joint="h" noise="0" cutoff="0"/></sensor></mujoco>"#,
    )
    .unwrap();
    assert_eq!((model.sensor_noise[0], model.sensor_cutoff[0]), (0.0, 0.0));
}

#[test]
fn a_sensor_takes_its_class_noise_and_cutoff() {
    let model = load_model(
        r#"<mujoco><default><default class="c"><sensor noise="0.5" cutoff="2"/></default></default><worldbody><body><joint name="h" type="hinge"/><geom size="0.1"/></body></worldbody><sensor><jointpos class="c" joint="h"/></sensor></mujoco>"#,
    )
    .unwrap();
    assert_eq!((model.sensor_noise[0], model.sensor_cutoff[0]), (0.5, 2.0));
}

/// Actuators and a sensor that write nothing, under no class: MuJoCo's
/// built-in values.
#[test]
fn values_written_by_none_are_mujocos_defaults() {
    let model = load_model(
        r#"<mujoco><worldbody><body><joint name="h" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody><actuator><position joint="h"/><cylinder joint="h"/><adhesion body="world" ctrlrange="0 1"/><muscle joint="h"/></actuator><sensor><jointpos joint="h"/></sensor></mujoco>"#,
    )
    .unwrap();
    assert_eq!(
        (model.actuator_gainprm[0][0], model.actuator_biasprm[0][1]),
        (1.0, -1.0)
    );
    assert_eq!(
        (model.actuator_gainprm[1][0], model.actuator_biasprm[1][0]),
        (1.0, 0.0)
    );
    assert_eq!(model.actuator_gainprm[2][0], 1.0);
    let muscle = [0.75, 1.05, -1.0, 200.0, 0.5, 1.6, 1.5, 1.3, 1.2];
    assert_eq!(model.actuator_gainprm[3], muscle);
    assert_eq!(model.actuator_biasprm[3], muscle);
    assert_eq!(
        (model.actuator_dynprm[3][0], model.actuator_dynprm[3][1]),
        (0.01, 0.04)
    );
    assert_eq!((model.sensor_noise[0], model.sensor_cutoff[0]), (0.0, 0.0));
}
