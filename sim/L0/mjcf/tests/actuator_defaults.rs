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

// ── A class's one actuator default (book L17, MuJoCo's `OneActuator`) ──────
//
// MuJoCo keeps one actuator default per class, which each actuator element
// in the class writes in place, and an actuator starts from a copy of it:
// a class written by two kinds, or used by another, builds what MuJoCo
// builds. Each value is MuJoCo 3.5.0's for the same document (measured).

/// The `i`th actuator's gain, bias and dynamics types, parameters and gear.
fn actuator(
    model: &sim_core::Model,
    i: usize,
) -> (
    sim_core::GainType,
    sim_core::BiasType,
    sim_core::ActuatorDynamics,
    [f64; 9],
    [f64; 9],
    [f64; 10],
    f64,
) {
    (
        model.actuator_gaintype[i],
        model.actuator_biastype[i],
        model.actuator_dyntype[i],
        model.actuator_gainprm[i],
        model.actuator_biasprm[i],
        model.actuator_dynprm[i],
        model.actuator_gear[i][0],
    )
}

/// A `<motor>` then a `<position>` in one class: the motor's gear reaches
/// both, the position's kp the position (ledger L310, book L16's d11).
#[test]
fn two_kinds_in_one_class_write_one_default() {
    let model = load_model(
        r#"<mujoco><default><default class="c"><motor gear="5"/><position kp="10"/></default></default><worldbody><body><joint name="h" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody><actuator><motor class="c" joint="h"/><position class="c" joint="h"/></actuator></mujoco>"#,
    )
    .unwrap();
    let (gaintype, biastype, _, gain, _, _, gear) = actuator(&model, 0);
    assert_eq!(
        (gaintype, biastype, gain[0], gear),
        (
            sim_core::GainType::Fixed,
            sim_core::BiasType::None,
            1.0,
            5.0
        )
    );
    let (_, biastype, _, gain, bias, _, gear) = actuator(&model, 1);
    assert_eq!(
        (biastype, gain[0], bias[1], gear),
        (sim_core::BiasType::Affine, 10.0, -10.0, 5.0)
    );
}

/// The other order: the motor resets the gain, so the position's kp is 1.
#[test]
fn two_kinds_in_one_class_in_the_other_order() {
    let model = load_model(
        r#"<mujoco><default><default class="c"><position kp="10"/><motor gear="5"/></default></default><worldbody><body><joint name="h" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody><actuator><motor class="c" joint="h"/><position class="c" joint="h"/></actuator></mujoco>"#,
    )
    .unwrap();
    let (_, _, _, gain, bias, _, gear) = actuator(&model, 1);
    assert_eq!((gain[0], bias[1], gear), (1.0, -1.0, 5.0));
}

/// Two top-level blocks write the main class's one default in order.
#[test]
fn top_level_blocks_write_one_default_in_order() {
    let model = load_model(
        r#"<mujoco><default><motor gear="5"/></default><default><position kp="10"/></default><worldbody><body><joint name="h" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody><actuator><motor joint="h"/><position joint="h"/></actuator></mujoco>"#,
    )
    .unwrap();
    assert_eq!(actuator(&model, 0).6, 5.0);
    let (_, _, _, gain, _, _, gear) = actuator(&model, 1);
    assert_eq!((gain[0], gear), (10.0, 5.0));
}

/// A class of one kind used by another (ledger L310, L330's N5): each
/// value its kind reads from the class.
#[test]
fn a_class_used_by_another_kind() {
    let load = |xml: &str| load_model(xml).unwrap();
    // A <general>'s gainprm under a <motor>.
    let m = load(
        r#"<mujoco><default><general gainprm="1 2 3"/></default><worldbody><body><joint name="h" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody><actuator><motor joint="h"/></actuator></mujoco>"#,
    );
    assert_eq!(m.actuator_gainprm[0][..3], [1.0, 2.0, 3.0]);
    // A <position>'s kp under a <general>: an affine bias -10.
    let m = load(
        r#"<mujoco><default><position kp="10"/></default><worldbody><body><joint name="h" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody><actuator><general joint="h" gainprm="5"/></actuator></mujoco>"#,
    );
    let (_, biastype, _, gain, bias, _, _) = actuator(&m, 0);
    assert_eq!(
        (biastype, gain[0], bias[1]),
        (sim_core::BiasType::Affine, 5.0, -10.0)
    );
    // A <position>'s kp is a <velocity>'s kv.
    let m = load(
        r#"<mujoco><default><position kp="10"/></default><worldbody><body><joint name="h" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody><actuator><velocity joint="h"/></actuator></mujoco>"#,
    );
    assert_eq!(
        (m.actuator_gainprm[0][0], m.actuator_biasprm[0][2]),
        (10.0, -10.0)
    );
    // A <velocity>'s kv is a <position>'s kp and damping.
    let m = load(
        r#"<mujoco><default><velocity kv="4"/></default><worldbody><body><joint name="h" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody><actuator><position joint="h"/></actuator></mujoco>"#,
    );
    assert_eq!(m.actuator_biasprm[0][..3], [0.0, -4.0, -4.0]);
    // A <muscle>'s dynamics stay under a <position>.
    let m = load(
        r#"<mujoco><default><muscle force="7"/></default><worldbody><body><joint name="h" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody><actuator><position joint="h"/></actuator></mujoco>"#,
    );
    assert_eq!(m.actuator_dyntype[0], sim_core::ActuatorDynamics::Muscle);
    // A <damper>'s forced control limit reaches a <motor>.
    let m = load(
        r#"<mujoco><default><damper kv="4" ctrlrange="0 1"/></default><worldbody><body><joint name="h" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody><actuator><motor joint="h"/></actuator></mujoco>"#,
    );
    assert_eq!(m.actuator_ctrlrange[0], (0.0, 1.0));
    // A <motor>'s gain 1 is no muscle range a muscle asked for.
    let m = load(
        r#"<mujoco><default><motor gear="2"/></default><worldbody><body><joint name="h" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody><actuator><muscle joint="h"/></actuator></mujoco>"#,
    );
    assert_eq!(
        (m.actuator_gainprm[0][0], m.actuator_gear[0][0]),
        (0.75, 2.0)
    );
    // An <intvelocity>'s integrator and activation limit under a <general>
    // and a <position>; a <position>'s timeconst under an <intvelocity>.
    for xml in [
        r#"<mujoco><default><intvelocity kp="3" actrange="-1 1"/></default><worldbody><body><joint name="h" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody><actuator><general joint="h"/></actuator></mujoco>"#,
        r#"<mujoco><default><intvelocity kp="3" actrange="-1 1"/></default><worldbody><body><joint name="h" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody><actuator><position joint="h"/></actuator></mujoco>"#,
    ] {
        let m = load(xml);
        assert_eq!(
            (
                m.actuator_dyntype[0],
                m.actuator_actlimited[0],
                m.actuator_gainprm[0][0]
            ),
            (sim_core::ActuatorDynamics::Integrator, true, 3.0),
            "{xml}"
        );
    }
    let m = load(
        r#"<mujoco><default><position timeconst="0.2"/></default><worldbody><body><joint name="h" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody><actuator><intvelocity joint="h" actrange="-1 1"/></actuator></mujoco>"#,
    );
    assert_eq!(
        (m.actuator_dynprm[0][0], m.actuator_actrange[0]),
        (0.2, (-1.0, 1.0))
    );
}

/// An element's own kv over its class's dampratio: they share one value
/// (ledger L328: MuJoCo's biasprm[2] -2).
#[test]
fn an_elements_kv_replaces_its_class_dampratio() {
    let model = load_model(
        r#"<mujoco><default><default class="c"><position dampratio="0.5"/></default></default><worldbody><body><joint name="h" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody><actuator><position class="c" joint="h" kp="3" kv="2"/></actuator></mujoco>"#,
    )
    .unwrap();
    assert_eq!(model.actuator_biasprm[0][2], -2.0);
}

/// A servo class's `inheritrange` beside a range it copies from its parent
/// (ledger L330), refused when the class is read, as MuJoCo refuses it.
#[test]
fn a_servo_class_inheritrange_beside_its_parents_range_is_refused() {
    for (xml, message) in [
        (
            r#"<mujoco><default><default class="p"><motor ctrlrange="-1 1"/><default class="c"><position kp="3" inheritrange="1"/></default></default></default><worldbody><body><joint name="h" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody><actuator><motor joint="h"/></actuator></mujoco>"#,
            "ctrlrange and inheritrange cannot both be defined",
        ),
        (
            r#"<mujoco><default><default class="p"><general actrange="-1 1"/><default class="c"><intvelocity inheritrange="1"/></default></default></default><worldbody><body><joint name="h" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody><actuator><motor joint="h"/></actuator></mujoco>"#,
            "actrange and inheritrange cannot both be defined",
        ),
    ] {
        let err = load_model(xml).expect_err(xml).to_string();
        assert!(err.contains(message), "{xml}: {err}");
    }
}

/// A class's `cranklength` and `diameter` reach its actuators.
#[test]
fn a_class_cranklength_and_diameter_reach_its_actuators() {
    let model = load_model(
        r#"<mujoco><default><motor cranklength="2"/></default><worldbody><body><joint name="h" type="slide"/><geom size="0.1"/><site name="c"/><site name="s" pos="0 0 1"/></body></worldbody><actuator><motor cranksite="c" slidersite="s"/><motor joint="h"/></actuator></mujoco>"#,
    )
    .unwrap();
    assert_eq!(model.actuator_cranklength, vec![2.0, 2.0]);
    let (gain, _, _) = params(
        r#"<mujoco><default><cylinder diameter="2"/></default><worldbody><body><joint name="h" type="hinge"/><geom size="0.1"/></body></worldbody><actuator><cylinder joint="h"/></actuator></mujoco>"#,
    );
    assert_eq!(gain[0], std::f64::consts::PI);
}

/// MuJoCo stores a muscle's fvmax as written, below 1 too.
#[test]
fn a_muscles_fvmax_below_one_is_kept() {
    let (gain, _, _) = params(
        r#"<mujoco><worldbody><body><joint name="h" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody><actuator><muscle joint="h" fvmax="0.5"/></actuator></mujoco>"#,
    );
    assert_eq!(gain[8], 0.5);
}

/// Where MuJoCo builds a value other than the document asks, refused: a
/// class's damping a damper does not take (D-DAMPER-CLASS-KV); a muscle
/// value a class asked for that MuJoCo's muscle replaces
/// (D-MUSCLE-SENTINEL); a class muscle's negative force under its parent's
/// (D-MUSCLE-NEGATIVE); `actearly` off a `<general>` (MuJoCo's schema); a
/// cylinder's second bias value (D-CYLINDER-BIAS).
#[test]
fn values_mujoco_would_replace_are_refused() {
    for (xml, attribute) in [
        (
            r#"<mujoco><default><damper kv="4" ctrlrange="0 1"/></default><worldbody><body><joint name="h" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody><actuator><damper joint="h"/></actuator></mujoco>"#,
            "kv",
        ),
        (
            r#"<mujoco><default><general gaintype="affine" gainprm="0 0 -3"/></default><worldbody><body><joint name="h" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody><actuator><damper joint="h" ctrlrange="0 1"/></actuator></mujoco>"#,
            "kv",
        ),
        (
            r#"<mujoco><default><muscle range="1 2"/></default><worldbody><body><joint name="h" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody><actuator><muscle joint="h"/></actuator></mujoco>"#,
            "range",
        ),
        (
            r#"<mujoco><default><muscle force="0"/></default><worldbody><body><joint name="h" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody><actuator><muscle joint="h"/></actuator></mujoco>"#,
            "force",
        ),
        (
            r#"<mujoco><default><muscle timeconst="1 0"/></default><worldbody><body><joint name="h" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody><actuator><muscle joint="h"/></actuator></mujoco>"#,
            "timeconst",
        ),
        (
            r#"<mujoco><default><general gainprm="1 0 5"/></default><worldbody><body><joint name="h" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody><actuator><muscle joint="h"/></actuator></mujoco>"#,
            "range",
        ),
        (
            r#"<mujoco><default><default class="p"><muscle force="7"/><default class="c"><muscle force="-1"/></default></default></default><worldbody><body><joint name="h" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody><actuator><motor joint="h"/></actuator></mujoco>"#,
            "force",
        ),
        (
            r#"<mujoco><worldbody><body><joint name="h" type="hinge"/><geom size="0.1"/></body></worldbody><actuator><cylinder joint="h" bias="1 2 0"/></actuator></mujoco>"#,
            "bias",
        ),
    ] {
        match load_model(xml) {
            Err(MjcfError::InvalidValue { attribute: a, .. }) => assert_eq!(a, attribute, "{xml}"),
            Err(err) => panic!("{xml}: not an invalid value: {err}"),
            Ok(_) => panic!("{xml}: loaded"),
        }
    }
    for xml in [
        r#"<mujoco><worldbody><body><joint name="h" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody><actuator><position joint="h" timeconst="0.1" actearly="true"/></actuator></mujoco>"#,
        r#"<mujoco><default><position actearly="true" timeconst="0.1"/></default><worldbody><body><joint name="h" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody><actuator><general joint="h"/></actuator></mujoco>"#,
    ] {
        let err = load_model(xml).expect_err(xml).to_string();
        assert!(err.contains("actearly"), "{xml}: {err}");
    }
}

/// A damper's or adhesion's negative control range, its class's too, and a
/// damper's negative kv and an adhesion's negative gain, are refused, as
/// MuJoCo refuses them (its messages, measured).
#[test]
fn a_negative_control_range_from_a_class_is_refused() {
    for (xml, message) in [
        (
            r#"<mujoco><worldbody><body><joint name="h" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody><actuator><damper joint="h" kv="-1" ctrlrange="0 1"/></actuator></mujoco>"#,
            "damping coefficient cannot be negative",
        ),
        (
            r#"<mujoco><worldbody><body><joint name="h" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody><actuator><adhesion body="world" gain="-1" ctrlrange="0 1"/></actuator></mujoco>"#,
            "adhesion gain cannot be negative",
        ),
        (
            r#"<mujoco><default><motor ctrlrange="-1 1"/></default><worldbody><body><joint name="h" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody><actuator><damper joint="h" kv="1"/></actuator></mujoco>"#,
            "damper control range cannot be negative",
        ),
        (
            r#"<mujoco><default><motor ctrlrange="-1 1"/></default><worldbody><body><joint name="h" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody><actuator><adhesion body="world" gain="1"/></actuator></mujoco>"#,
            "adhesion control range cannot be negative",
        ),
    ] {
        let err = load_model(xml).expect_err(xml).to_string();
        assert!(err.contains(message), "{xml}: {err}");
    }
}

/// A class's damper kv written, and a <general>'s fixed gain carrying a
/// third value (no damping), load: MuJoCo's kv 0 is then what is asked.
#[test]
fn a_damper_under_a_class_with_no_damping_loads() {
    for xml in [
        r#"<mujoco><default><damper kv="4" ctrlrange="0 1"/></default><worldbody><body><joint name="h" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody><actuator><damper joint="h" kv="4"/></actuator></mujoco>"#,
        r#"<mujoco><default><general gainprm="1 0 -3"/></default><worldbody><body><joint name="h" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody><actuator><damper joint="h" ctrlrange="0 1"/></actuator></mujoco>"#,
    ] {
        load_model(xml).expect(xml);
    }
}

/// An <actuator> in a default class has no kind: refused.
#[test]
fn an_actuator_element_in_a_class_is_refused() {
    let err = load_model(
        r#"<mujoco><default><actuator ctrlrange="-1 1"/></default><worldbody><body><joint name="h" type="hinge"/><geom size="0.1"/></body></worldbody><actuator><motor joint="h"/></actuator></mujoco>"#,
    )
    .unwrap_err();
    assert!(matches!(err, MjcfError::UnknownElement { .. }), "{err}");
}

/// A muscle's own `tausmooth` is its `dynprm[2]`, the smooth switching time
/// of its activation; a negative one, and one in a class (MuJoCo's default
/// schema has none), are refused, as MuJoCo refuses them (measured).
#[test]
fn a_muscles_tausmooth_is_its_dynprm2() {
    let (_, _, dynamics) = params(
        r#"<mujoco><worldbody><body><joint name="h" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody><actuator><muscle joint="h" tausmooth="0.1"/></actuator></mujoco>"#,
    );
    assert_eq!(dynamics[2], 0.1);
    let err = load_model(
        r#"<mujoco><worldbody><body><joint name="h" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody><actuator><muscle joint="h" tausmooth="-0.1"/></actuator></mujoco>"#,
    )
    .unwrap_err()
    .to_string();
    assert!(err.contains("muscle tausmooth cannot be negative"), "{err}");
    let err = load_model(
        r#"<mujoco><default><muscle tausmooth="0.2"/></default><worldbody><body><joint name="h" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody><actuator><muscle joint="h"/></actuator></mujoco>"#,
    )
    .unwrap_err();
    assert!(
        matches!(&err, MjcfError::UnknownAttribute { attribute, .. } if attribute == "tausmooth"),
        "{err}"
    );
}

// ── What `e6efb4b3`'s mutants found untested ───────────────────────────────

/// A `<velocity>` zeroes its class's bias before writing its own damping
/// (`mjs_setToVelocity`: biasprm 0 0 -10 under a position class's kp 10,
/// measured); a `<damper>` zeroes its class's gain and takes kv 0 under a
/// class whose fixed gain has a third value (gainprm 0 0 -0, measured).
#[test]
fn a_velocity_and_a_damper_clear_what_their_class_wrote() {
    let model = load_model(
        r#"<mujoco><default><position kp="10"/></default><worldbody><body><joint name="h" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody><actuator><velocity joint="h"/></actuator></mujoco>"#,
    )
    .unwrap();
    assert_eq!(model.actuator_biasprm[0][..3], [0.0, 0.0, -10.0]);
    let model = load_model(
        r#"<mujoco><default><general gainprm="1 0 -3"/></default><worldbody><body><joint name="h" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody><actuator><damper joint="h" ctrlrange="0 1"/></actuator></mujoco>"#,
    )
    .unwrap();
    assert_eq!(model.actuator_gainprm[0][..3], [0.0, 0.0, 0.0]);
}

/// A damper's and an adhesion's control are limited whatever `autolimits`
/// says (`mjs_setToDamper`, `mjs_setToAdhesion`; measured with it off).
#[test]
fn a_damper_and_an_adhesion_are_control_limited_without_autolimits() {
    let model = load_model(
        r#"<mujoco><compiler autolimits="false"/><worldbody><body><joint name="h" type="hinge"/><geom size="0.1"/></body></worldbody><actuator><damper joint="h" kv="1" ctrlrange="0 1"/><adhesion body="world" gain="1" ctrlrange="0 1"/></actuator></mujoco>"#,
    )
    .unwrap();
    assert_eq!(model.actuator_ctrlrange, vec![(0.0, 1.0), (0.0, 1.0)]);
}

/// A cylinder class's `dynprm[0]` of 1 is its own default timeconst, not a
/// muscle time constant anyone asked for: a muscle under it takes 0.01, as
/// MuJoCo's muscle replaces it (measured: dynprm 0.01 0.04).
#[test]
fn a_muscle_replaces_a_value_its_class_kind_set() {
    let (_, _, dynamics) = params(
        r#"<mujoco><default><cylinder/></default><worldbody><body><joint name="h" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody><actuator><muscle joint="h"/></actuator></mujoco>"#,
    );
    assert_eq!((dynamics[0], dynamics[1]), (0.01, 0.04));
}

// ── Where the stricter rules apply: at the actuator that takes the value ──
//
// A class's own `<damper>` or `<muscle>` may drop a value an earlier writer
// asked for, as MuJoCo drops it; that is refused only for an actuator that
// would be built with the loss (D-DAMPER-CLASS-KV, D-MUSCLE-SENTINEL), not
// for the class, nor for an actuator that writes the value itself. Each
// value is MuJoCo 3.5.0's for the same document (measured).

#[test]
fn a_class_that_drops_a_value_refuses_no_actuator_writing_its_own() {
    for (xml, gain) in [
        (
            r#"<mujoco><default><default class="c"><general gaintype="affine" gainprm="0 0 -3"/><damper ctrlrange="0 1"/></default></default><worldbody><body><joint name="h" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody><actuator><damper class="c" joint="h" kv="2"/></actuator></mujoco>"#,
            [0.0, 0.0, -2.0, 0.0],
        ),
        (
            r#"<mujoco><default><default class="c"><damper kv="4" ctrlrange="0 1"/><default class="d"><damper ctrlrange="0 2"/></default></default></default><worldbody><body><joint name="h" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody><actuator><damper class="d" joint="h" kv="2"/></actuator></mujoco>"#,
            [0.0, 0.0, -2.0, 0.0],
        ),
        (
            r#"<mujoco><default><damper kv="4" ctrlrange="0 1"/></default><default><damper ctrlrange="0 2"/></default><worldbody><body><joint name="h" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody><actuator><damper joint="h" kv="1"/></actuator></mujoco>"#,
            [0.0, 0.0, -1.0, 0.0],
        ),
        (
            r#"<mujoco><default><default class="c"><general gainprm="1"/><muscle scale="100"/></default></default><worldbody><body><joint name="h" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody><actuator><muscle class="c" joint="h" range="0.5 1.2"/></actuator></mujoco>"#,
            [0.5, 1.2, -1.0, 100.0],
        ),
        // A <motor> under a class whose muscle replaced a general's values.
        (
            r#"<mujoco><default><general gainprm="1 0 5"/><muscle/></default><worldbody><body><joint name="h" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody><actuator><motor joint="h"/></actuator></mujoco>"#,
            [1.0, 1.05, 5.0, 200.0],
        ),
    ] {
        let (built, _, _) = params(xml);
        assert_eq!(built[..4], gain, "{xml}");
    }
}

/// A class no actuator takes the dropped value from loads.
#[test]
fn a_class_that_drops_a_value_unused_loads() {
    for xml in [
        r#"<mujoco><default><default class="p"><muscle range="1 2"/><default class="c"><muscle/></default></default></default><worldbody><body><joint name="h" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody><actuator><motor joint="h"/></actuator></mujoco>"#,
        r#"<mujoco><default><default class="p"><damper kv="4" ctrlrange="0 1"/><default class="c"><damper/></default></default></default><worldbody><body><joint name="h" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody><actuator><motor joint="h"/></actuator></mujoco>"#,
    ] {
        let (gain, _, _) = params(xml);
        assert_eq!(gain[0], 1.0, "{xml}");
    }
}

/// An actuator that takes a value its class's muscle or damper dropped is
/// refused (MuJoCo builds range 0.75 2 for the first, kv 0 for the second).
#[test]
fn an_actuator_taking_a_value_its_class_dropped_is_refused() {
    for (xml, attribute) in [
        (
            r#"<mujoco><default><default class="p"><muscle range="1 2"/><default class="c"><muscle/></default></default></default><worldbody><body><joint name="h" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody><actuator><muscle class="c" joint="h"/></actuator></mujoco>"#,
            "range",
        ),
        (
            r#"<mujoco><default><default class="p"><damper kv="4" ctrlrange="0 1"/><default class="c"><damper ctrlrange="0 1"/></default></default></default><worldbody><body><joint name="h" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody><actuator><damper class="c" joint="h"/></actuator></mujoco>"#,
            "kv",
        ),
    ] {
        match load_model(xml) {
            Err(MjcfError::InvalidValue { attribute: a, .. }) => assert_eq!(a, attribute, "{xml}"),
            Err(err) => panic!("{xml}: not an invalid value: {err}"),
            Ok(_) => panic!("{xml}: loaded"),
        }
    }
}

/// A value a kind's own write set is no muscle value anyone asked for, even
/// where a writer asked for that value before it: a muscle replaces it, as
/// MuJoCo does (gainprm 0.75, dynprm 0.01, measured).
#[test]
fn a_kinds_own_write_clears_what_was_asked() {
    for xml in [
        r#"<mujoco><default><general gainprm="1"/><position kp="3"/><velocity kv="1"/></default><worldbody><body><joint name="h" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody><actuator><muscle joint="h"/></actuator></mujoco>"#,
        r#"<mujoco><default><general gainprm="1"/><position kp="2"/><cylinder area="1"/></default><worldbody><body><joint name="h" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody><actuator><muscle joint="h"/></actuator></mujoco>"#,
        r#"<mujoco><default><general dynprm="1"/><position timeconst="0.5"/><cylinder timeconst="1"/></default><worldbody><body><joint name="h" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody><actuator><muscle joint="h"/></actuator></mujoco>"#,
        r#"<mujoco><default><default class="c"><general gainprm="1"/><velocity kv="3"/><default class="d"><velocity kv="1"/></default></default></default><worldbody><body><joint name="h" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody><actuator><muscle class="d" joint="h"/></actuator></mujoco>"#,
    ] {
        let (gain, _, dynamics) = params(xml);
        assert_eq!((gain[0], dynamics[0]), (0.75, 0.01), "{xml}");
    }
}

/// A kind-specific attribute its kind does not read, on an element, a
/// class's element or an actuator built in code, is refused, as MuJoCo's
/// schema refuses it ("unrecognized attribute", measured): sim-mjcf would
/// otherwise drop it unread.
#[test]
fn an_attribute_its_kind_does_not_read_is_refused() {
    let unknown = |result: Result<sim_core::Model, MjcfError>, what: &str| match result {
        Err(MjcfError::UnknownAttribute { attribute, .. }) => attribute,
        Err(err) => panic!("{what}: not an unknown attribute: {err}"),
        Ok(_) => panic!("{what}: loaded"),
    };
    for (xml, attribute) in [
        (
            r#"<mujoco><worldbody><body><joint name="h" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody><actuator><motor joint="h" kp="5"/></actuator></mujoco>"#,
            "kp",
        ),
        (
            r#"<mujoco><worldbody><body><joint name="h" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody><actuator><position joint="h" area="5"/></actuator></mujoco>"#,
            "area",
        ),
        (
            r#"<mujoco><worldbody><body><joint name="h" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody><actuator><velocity joint="h" gain="2"/></actuator></mujoco>"#,
            "gain",
        ),
        (
            r#"<mujoco><worldbody><body><joint name="h" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody><actuator><motor joint="h" gaintype="affine"/></actuator></mujoco>"#,
            "gaintype",
        ),
        (
            r#"<mujoco><worldbody><body><joint name="h" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody><actuator><damper joint="h" kv="1" ctrlrange="0 1" tausmooth="0.1"/></actuator></mujoco>"#,
            "tausmooth",
        ),
        (
            r#"<mujoco><default><motor kp="5"/></default><worldbody><body><joint name="h" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody><actuator><position joint="h"/></actuator></mujoco>"#,
            "kp",
        ),
        (
            r#"<mujoco><default><motor area="3"/></default><worldbody><body><joint name="h" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody><actuator><cylinder joint="h"/></actuator></mujoco>"#,
            "area",
        ),
        (
            r#"<mujoco><default><position gain="4"/></default><worldbody><body><joint name="h" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody><actuator><adhesion body="world" ctrlrange="0 1"/></actuator></mujoco>"#,
            "gain",
        ),
        (
            r#"<mujoco><default><motor range="0.5 2"/></default><worldbody><body><joint name="h" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody><actuator><muscle joint="h"/></actuator></mujoco>"#,
            "range",
        ),
    ] {
        assert_eq!(unknown(load_model(xml), xml), attribute, "{xml}");
    }
    let mut mjcf = parse_mjcf_str(
        r#"<mujoco><worldbody><body><joint name="h" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody><actuator><position joint="h" kp="3" timeconst="0.1"/></actuator></mujoco>"#,
    )
    .unwrap();
    mjcf.actuators[0].actearly = Some(true);
    assert_eq!(
        unknown(
            model_from_mjcf(&mjcf, None),
            "a code-built position's actearly"
        ),
        "actearly"
    );
    let mut mjcf = parse_mjcf_str(
        r#"<mujoco><worldbody><body><joint name="h" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody><actuator><intvelocity joint="h" kp="3" actrange="-1 1"/></actuator></mujoco>"#,
    )
    .unwrap();
    mjcf.actuators[0].dynprm = sim_mjcf::Prefix::new(&[0.5]);
    assert_eq!(
        unknown(
            model_from_mjcf(&mjcf, None),
            "a code-built intvelocity's dynprm"
        ),
        "dynprm"
    );
}

/// `actearly` on a `<muscle>`, element or class: sim-mjcf's allowlisted
/// extension (the book's settled call A4-Q2), which MuJoCo's schema refuses.
#[test]
fn a_muscles_actearly_is_allowlisted() {
    for xml in [
        r#"<mujoco><worldbody><body><joint name="h" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody><actuator><muscle joint="h" actearly="true"/></actuator></mujoco>"#,
        r#"<mujoco><default><muscle actearly="true"/></default><worldbody><body><joint name="h" type="hinge" range="-30 60"/><geom size="0.1"/></body></worldbody><actuator><muscle joint="h"/></actuator></mujoco>"#,
    ] {
        assert!(load_model(xml).unwrap().actuator_actearly[0], "{xml}");
    }
}
