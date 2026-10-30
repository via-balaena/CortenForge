//! An actuator's or sensor's own written value over its class's, whatever
//! the value (book L17): one equal to the built-in default is the element's
//! too. Each value is MuJoCo 3.5.0's for the same document (measured), but
//! the sensor's: MuJoCo has no sensor default class. A muscle's negative
//! value, which MuJoCo silently replaces, is refused (D-MUSCLE-NEGATIVE).

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::float_cmp
)]

use sim_mjcf::{MjcfError, load_model, model_from_mjcf, parse_mjcf_str};

/// The attribute a document's refusal names, which must be a muscle's.
fn refused_muscle_attribute(xml: &str) -> String {
    match load_model(xml) {
        Err(MjcfError::InvalidValue {
            element, attribute, ..
        }) if element == "muscle" => attribute,
        Err(err) => panic!("{xml}: not a muscle's invalid value: {err}"),
        Ok(_) => panic!("{xml}: loaded"),
    }
}

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

/// A negative `force` asks for one computed when the model is made; MuJoCo
/// reads it as unwritten (`mjs_setToMuscle`), so under a class's positive
/// force it builds the class's (77, measured): refused.
#[test]
fn a_muscle_negative_force_under_a_class_force_is_refused() {
    assert_eq!(
        refused_muscle_attribute(
            r#"<mujoco><default><default class="c"><muscle force="77"/></default></default><worldbody><body><joint name="h" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody><actuator><muscle class="c" joint="h" force="-1"/></actuator></mujoco>"#,
        ),
        "force"
    );
}

/// A negative force under no class force, or under a class's own negative or
/// zero one, is computed as asked (MuJoCo: -1 in each, measured).
#[test]
fn a_muscle_negative_force_with_no_positive_class_force_loads() {
    for xml in [
        r#"<mujoco><worldbody><body><joint name="h" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody><actuator><muscle joint="h" force="-1"/></actuator></mujoco>"#,
        r#"<mujoco><worldbody><body><joint name="h" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody><actuator><muscle joint="h" force="-5"/></actuator></mujoco>"#,
        r#"<mujoco><default><default class="c"><muscle force="-1"/></default></default><worldbody><body><joint name="h" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody><actuator><muscle class="c" joint="h" force="-5"/></actuator></mujoco>"#,
        r#"<mujoco><default><default class="c"><muscle force="0"/></default></default><worldbody><body><joint name="h" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody><actuator><muscle class="c" joint="h" force="-1"/></actuator></mujoco>"#,
    ] {
        let (gain, _, _) = params(xml);
        assert!(gain[2] < 0.0, "{xml}: force {}", gain[2]);
    }
}

/// MuJoCo keeps its class's or the built-in value in place of each (lmin
/// -0.5 builds 0.5, scale -3 builds 200, timeconst -1 0.05 builds 0.01 0.05,
/// range -0.5 1.5 builds 0.75 1.5, vmax -1 builds 1.5: measured).
#[test]
fn a_muscles_negative_values_are_refused() {
    for (xml, attribute) in [
        (
            r#"<mujoco><worldbody><body><joint name="h" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody><actuator><muscle joint="h" timeconst="-1 0.05"/></actuator></mujoco>"#,
            "timeconst",
        ),
        (
            r#"<mujoco><worldbody><body><joint name="h" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody><actuator><muscle joint="h" timeconst="0.01 -0.04"/></actuator></mujoco>"#,
            "timeconst",
        ),
        (
            r#"<mujoco><worldbody><body><joint name="h" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody><actuator><muscle joint="h" range="-0.5 1.5"/></actuator></mujoco>"#,
            "range",
        ),
        (
            r#"<mujoco><worldbody><body><joint name="h" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody><actuator><muscle joint="h" range="0.5 -1.5"/></actuator></mujoco>"#,
            "range",
        ),
        (
            r#"<mujoco><worldbody><body><joint name="h" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody><actuator><muscle joint="h" scale="-3"/></actuator></mujoco>"#,
            "scale",
        ),
        (
            r#"<mujoco><worldbody><body><joint name="h" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody><actuator><muscle joint="h" lmin="-0.5"/></actuator></mujoco>"#,
            "lmin",
        ),
        (
            r#"<mujoco><worldbody><body><joint name="h" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody><actuator><muscle joint="h" lmax="-1.6"/></actuator></mujoco>"#,
            "lmax",
        ),
        (
            r#"<mujoco><worldbody><body><joint name="h" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody><actuator><muscle joint="h" vmax="-1"/></actuator></mujoco>"#,
            "vmax",
        ),
        (
            r#"<mujoco><worldbody><body><joint name="h" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody><actuator><muscle joint="h" fpmax="-1.3"/></actuator></mujoco>"#,
            "fpmax",
        ),
        (
            r#"<mujoco><worldbody><body><joint name="h" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody><actuator><muscle joint="h" fvmax="-1.2"/></actuator></mujoco>"#,
            "fvmax",
        ),
    ] {
        assert_eq!(refused_muscle_attribute(xml), attribute, "{xml}");
    }
}

/// A muscle built in code is held to the same refusal.
#[test]
fn a_code_built_muscles_negative_value_is_refused() {
    let mut mjcf = parse_mjcf_str(
        r#"<mujoco><worldbody><body><joint name="h" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody><actuator><muscle joint="h" force="-1"/></actuator></mujoco>"#,
    )
    .unwrap();
    mjcf.actuators[0].lmin = Some(-0.5);
    match model_from_mjcf(&mjcf, None) {
        Err(MjcfError::InvalidValue {
            element, attribute, ..
        }) => assert_eq!((element.as_str(), attribute.as_str()), ("muscle", "lmin")),
        Err(err) => panic!("not a muscle's invalid value: {err}"),
        Ok(_) => panic!("a code-built muscle's lmin -0.5 loaded"),
    }
}

/// A class's own negative muscle value, used or not (MuJoCo keeps 0.5 for
/// an actuator of the class, measured).
#[test]
fn a_muscle_class_negative_value_is_refused() {
    for xml in [
        r#"<mujoco><default><default class="c"><muscle lmin="-0.5"/></default></default><worldbody><body><joint name="h" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody><actuator><muscle class="c" joint="h"/></actuator></mujoco>"#,
        r#"<mujoco><default><default class="c"><muscle lmin="-0.5"/></default></default><worldbody><body><joint name="h" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody><actuator><motor joint="h"/></actuator></mujoco>"#,
    ] {
        assert_eq!(refused_muscle_attribute(xml), "lmin", "{xml}");
    }
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

/// The attribute a document's refusal names, which must be a cylinder's.
fn refused_cylinder_attribute(xml: &str) -> String {
    match load_model(xml) {
        Err(MjcfError::InvalidValue {
            element, attribute, ..
        }) if element == "cylinder" => attribute,
        Err(err) => panic!("{xml}: not a cylinder's invalid value: {err}"),
        Ok(_) => panic!("{xml}: loaded"),
    }
}

/// MuJoCo reads a cylinder's three `bias` values into one number and keeps
/// the first (`bias="1 2 3"` builds biasprm 1 0 0, `"0 0 -0.5"` 0 0 0:
/// measured), so a second or third that is not 0 is refused
/// (D-CYLINDER-BIAS), on an element or a class's, used or not.
#[test]
fn a_cylinders_bias_beyond_its_first_value_is_refused() {
    for xml in [
        r#"<mujoco><worldbody><body><joint name="h" type="hinge"/><geom size="0.1"/></body></worldbody><actuator><cylinder joint="h" bias="1 2 3"/></actuator></mujoco>"#,
        r#"<mujoco><worldbody><body><joint name="h" type="hinge"/><geom size="0.1"/></body></worldbody><actuator><cylinder joint="h" bias="0 0 -0.5"/></actuator></mujoco>"#,
        r#"<mujoco><default><default class="c"><cylinder bias="1 2 3"/></default></default><worldbody><body><joint name="h" type="hinge"/><geom size="0.1"/></body></worldbody><actuator><cylinder class="c" joint="h"/></actuator></mujoco>"#,
        r#"<mujoco><default><default class="c"><cylinder bias="1 2 3"/></default></default><worldbody><body><joint name="h" type="hinge"/><geom size="0.1"/></body></worldbody><actuator><motor joint="h"/></actuator></mujoco>"#,
    ] {
        assert_eq!(refused_cylinder_attribute(xml), "bias", "{xml}");
    }
}

#[test]
fn a_cylinders_bias_of_its_first_value_loads() {
    let (_, bias, _) = params(
        r#"<mujoco><worldbody><body><joint name="h" type="hinge"/><geom size="0.1"/></body></worldbody><actuator><cylinder joint="h" bias="1 0 0"/></actuator></mujoco>"#,
    );
    assert_eq!(bias[..3], [1.0, 0.0, 0.0]);
}

/// A cylinder built in code is held to the same refusal.
#[test]
fn a_code_built_cylinders_bias_beyond_its_first_value_is_refused() {
    let mut mjcf = parse_mjcf_str(
        r#"<mujoco><worldbody><body><joint name="h" type="hinge"/><geom size="0.1"/></body></worldbody><actuator><cylinder joint="h" bias="1 0 0"/></actuator></mujoco>"#,
    )
    .unwrap();
    mjcf.actuators[0].bias = Some([1.0, 2.0, 3.0]);
    match model_from_mjcf(&mjcf, None) {
        Err(MjcfError::InvalidValue {
            element, attribute, ..
        }) => assert_eq!((element.as_str(), attribute.as_str()), ("cylinder", "bias")),
        Err(err) => panic!("not a cylinder's invalid value: {err}"),
        Ok(_) => panic!("a code-built cylinder's bias 1 2 3 loaded"),
    }
}
