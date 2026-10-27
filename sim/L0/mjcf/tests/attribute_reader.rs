//! Numbers and lengths in attribute values, read as MuJoCo 3.5.0's XML reader reads them
//! (`xml_util.cc` `ReadAttr`, `ReadAttrInt`; `user_util.cc` `StringToVector`). Each expected value
//! or refusal was measured on MuJoCo 3.5.0.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::float_cmp
)]

use sim_core::Model;
use sim_mjcf::{MjcfError, ValueCount, load_model};

fn load(xml: &str) -> Model {
    load_model(xml).unwrap_or_else(|e| panic!("refused: {e}"))
}

fn refused(xml: &str) -> MjcfError {
    match load_model(xml) {
        Ok(_) => panic!("loaded; MuJoCo 3.5.0 refuses it"),
        Err(e) => e,
    }
}

/// The error is a bad number in `attribute` on `element`.
fn assert_bad_number(err: &MjcfError, element: &str, attribute: &str) {
    assert!(
        matches!(err, MjcfError::BadNumber { element: e, attribute: a, .. } if e == element && a == attribute),
        "{err}"
    );
}

/// The error is a wrong number of values in `attribute` on `element`.
fn assert_count(
    err: &MjcfError,
    element: &str,
    attribute: &str,
    expected: ValueCount,
    found: usize,
) {
    assert!(
        matches!(err, MjcfError::WrongValueCount { element: e, attribute: a, expected: x, found: f, .. }
            if e == element && a == attribute && *x == expected && *f == found),
        "{err}"
    );
}

// ── numbers ──

#[test]
fn a_unit_after_a_number_is_refused() {
    let err = refused(r#"<mujoco><option timestep="0.01s"/><worldbody/></mujoco>"#);
    assert_bad_number(&err, "option", "timestep");
}

#[test]
fn a_word_where_a_number_belongs_is_refused() {
    let err = refused(
        r#"<mujoco><worldbody><body><joint damping="abc"/><geom size="0.1"/></body></worldbody></mujoco>"#,
    );
    assert_bad_number(&err, "joint", "damping");
}

#[test]
fn a_number_with_a_unit_or_a_comma_is_refused() {
    for mass in ["2kg", "2,5"] {
        let err = refused(&format!(
            r#"<mujoco><worldbody><body><freejoint/><geom size="0.1" mass="{mass}"/></body></worldbody></mujoco>"#
        ));
        assert_bad_number(&err, "geom", "mass");
    }
}

#[test]
fn spaces_around_a_number_are_trimmed() {
    let m = load(
        r#"<mujoco><worldbody><body><freejoint/><geom size="0.1" mass=" 2 "/></body></worldbody></mujoco>"#,
    );
    assert_eq!(m.body_mass[1], 2.0);
}

#[test]
fn a_vertical_tab_and_a_tab_separate_like_spaces() {
    let m = load(
        "<mujoco><worldbody><body><freejoint/><geom size=\"0.1\" mass=\"&#11;2&#9;\"/></body></worldbody></mujoco>",
    );
    assert_eq!(m.body_mass[1], 2.0);
}

#[test]
fn an_empty_number_is_absent() {
    // The geom's mass comes from its density, as with no `mass` attribute.
    let empty = load(
        r#"<mujoco><worldbody><body><freejoint/><geom size="0.1" mass=""/></body></worldbody></mujoco>"#,
    );
    let none = load(
        r#"<mujoco><worldbody><body><freejoint/><geom size="0.1"/></body></worldbody></mujoco>"#,
    );
    assert_eq!(empty.body_mass[1], none.body_mass[1]);
    let empty_pos =
        load(r#"<mujoco><worldbody><body pos=""><geom size="0.1"/></body></worldbody></mujoco>"#);
    assert_eq!(empty_pos.body_pos[1], nalgebra::Vector3::zeros());
}

#[test]
fn a_number_that_underflows_is_out_of_range() {
    for mass in ["1e-999", "1e-310"] {
        let err = refused(&format!(
            r#"<mujoco><worldbody><body><freejoint/><geom size="0.1" mass="{mass}"/></body></worldbody></mujoco>"#
        ));
        assert!(
            matches!(&err, MjcfError::OutOfRange { element, attribute, .. } if element == "geom" && attribute == "mass"),
            "{mass}: {err}"
        );
    }
}

#[test]
fn an_integer_attribute_takes_only_an_integer() {
    for condim in ["3.0", "1e0", ""] {
        let err = refused(&format!(
            r#"<mujoco><worldbody><body><freejoint/><geom size="0.1" condim="{condim}"/></body></worldbody></mujoco>"#
        ));
        assert_bad_number(&err, "geom", "condim");
    }
    let err = refused(
        r#"<mujoco><worldbody><body><freejoint/><geom size="0.1" group="99999999999"/></body></worldbody></mujoco>"#,
    );
    assert_bad_number(&err, "geom", "group");
}

#[test]
fn an_integer_may_carry_spaces_and_a_plus_sign() {
    for condim in [" 4 ", "+4"] {
        let m = load(&format!(
            r#"<mujoco><worldbody><body><freejoint/><geom size="0.1" condim="{condim}"/></body></worldbody></mujoco>"#
        ));
        assert_eq!(m.geom_condim[0], 4, "{condim:?}");
    }
}

// ── lengths ──

#[test]
fn a_position_takes_exactly_three_numbers() {
    let err = refused(
        r#"<mujoco><worldbody><body pos="0 0 0 1"><geom size="0.1"/></body></worldbody></mujoco>"#,
    );
    assert_count(&err, "body", "pos", ValueCount::Exactly(3), 4);
    let err = refused(
        r#"<mujoco><worldbody><body pos="0 0"><geom size="0.1"/></body></worldbody></mujoco>"#,
    );
    assert_count(&err, "body", "pos", ValueCount::Exactly(3), 2);
}

#[test]
fn a_geom_friction_takes_at_most_three_numbers() {
    let err = refused(
        r#"<mujoco><worldbody><body><freejoint/><geom size="0.1" friction="1 2 3 4"/></body></worldbody></mujoco>"#,
    );
    assert_count(&err, "geom", "friction", ValueCount::AtMost(3), 4);
}

#[test]
fn a_gear_takes_at_most_six_numbers() {
    let err = refused(
        r#"<mujoco>
            <worldbody><body><joint name="j"/><geom size="0.1"/></body></worldbody>
            <actuator><motor joint="j" gear="1 0 0 0 0 0 0"/></actuator>
        </mujoco>"#,
    );
    assert_count(&err, "motor", "gear", ValueCount::AtMost(6), 7);
}

#[test]
fn a_tenth_gain_parameter_is_a_stated_limitation() {
    // MuJoCo 3.5.0 stores ten gain parameters; sim-core stores nine (D-GAINPRM-10).
    let err = refused(
        r#"<mujoco>
            <worldbody><body><joint name="j"/><geom size="0.1"/></body></worldbody>
            <actuator><general joint="j" gainprm="1 2 3 4 5 6 7 8 9 10"/></actuator>
        </mujoco>"#,
    );
    assert!(matches!(&err, MjcfError::Unsupported { .. }), "{err}");
    // A tenth of 0, MuJoCo's default, is what sim-core's nine mean.
    let m = load(
        r#"<mujoco>
            <worldbody><body><joint name="j"/><geom size="0.1"/></body></worldbody>
            <actuator><general joint="j" gainprm="1 2 3 4 5 6 7 8 9 0"/></actuator>
        </mujoco>"#,
    );
    assert_eq!(
        m.actuator_gainprm[0],
        [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0]
    );
}

#[test]
fn a_site_size_and_a_fromto_take_their_counts() {
    let err = refused(
        r#"<mujoco><worldbody><body><geom size="0.1"/><site type="box" size="0.1 0.2 0.3 0.4"/></body></worldbody></mujoco>"#,
    );
    assert_count(&err, "site", "size", ValueCount::AtMost(3), 4);
    let err = refused(
        r#"<mujoco><worldbody><body><geom size="0.1"/><site type="capsule" size="0.1" fromto="0 0 0 0 0"/></body></worldbody></mujoco>"#,
    );
    assert_count(&err, "site", "fromto", ValueCount::Exactly(6), 5);
    let err = refused(
        r#"<mujoco><worldbody><body><geom size="0.1"/><site type="capsule" size="0.1" fromto="0 0 0 0 0 1 2"/></body></worldbody></mujoco>"#,
    );
    assert_count(&err, "site", "fromto", ValueCount::Exactly(6), 7);
    let err = refused(
        r#"<mujoco><worldbody><body><geom type="capsule" size="0.1" fromto="0 0 0 0 0"/></body></worldbody></mujoco>"#,
    );
    assert_count(&err, "geom", "fromto", ValueCount::Exactly(6), 5);
}

// ── prefixes: MuJoCo's `exact = false` attributes ──

#[test]
fn a_single_friction_value_keeps_the_rest_of_the_default() {
    let m = load(
        r#"<mujoco><worldbody><body><freejoint/><geom size="0.1" friction="0.7"/></body></worldbody></mujoco>"#,
    );
    assert_eq!(
        m.geom_friction[0],
        nalgebra::Vector3::new(0.7, 0.005, 0.0001)
    );
}

#[test]
fn a_single_friction_value_keeps_the_rest_of_its_class() {
    let m = load(
        r#"<mujoco>
            <default><geom friction="1 0.1 0.2"/></default>
            <worldbody><body><freejoint/><geom size="0.1" friction="0.7"/></body></worldbody>
        </mujoco>"#,
    );
    assert_eq!(m.geom_friction[0], nalgebra::Vector3::new(0.7, 0.1, 0.2));
}

#[test]
fn a_short_solref_and_solimp_keep_the_rest_of_the_default() {
    let m = load(
        r#"<mujoco><worldbody><body><freejoint/><geom size="0.1" solref="0.05" solimp="0.8 0.9 0.01"/></body></worldbody></mujoco>"#,
    );
    assert_eq!(m.geom_solref[0], [0.05, 1.0]);
    assert_eq!(m.geom_solimp[0], [0.8, 0.9, 0.01, 0.5, 2.0]);
}

#[test]
fn two_fluid_coefficients_keep_the_rest_of_the_default() {
    let m = load(
        r#"<mujoco><worldbody><body><freejoint/><geom size="0.1" fluidshape="ellipsoid" fluidcoef="0.5 0.25"/></body></worldbody></mujoco>"#,
    );
    assert_eq!(m.geom_fluid[0][1..6], [0.5, 0.25, 1.5, 1.0, 1.0]);
}

#[test]
fn a_single_gear_value_is_the_first_of_six() {
    let m = load(
        r#"<mujoco>
            <worldbody><body><joint name="j"/><geom size="0.1"/></body></worldbody>
            <actuator><motor joint="j" gear="5"/></actuator>
        </mujoco>"#,
    );
    assert_eq!(m.actuator_gear[0], [5.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
}

#[test]
fn an_explicit_gear_equal_to_the_default_beats_its_class() {
    // A3 §3 `s11_gear`: before, a gear equal to the built-in default read as unset.
    let m = load(
        r#"<mujoco>
            <default><motor gear="50"/></default>
            <worldbody><body><joint name="j"/><geom size="0.1"/></body></worldbody>
            <actuator><motor joint="j" gear="1"/></actuator>
        </mujoco>"#,
    );
    assert_eq!(m.actuator_gear[0][0], 1.0);
}

// ── text ──

#[test]
fn an_entity_in_a_name_is_unescaped() {
    let m = load(
        r#"<mujoco><worldbody><body name="a&amp;b"><geom size="0.1"/></body></worldbody></mujoco>"#,
    );
    assert_eq!(m.body_name[1].as_deref(), Some("a&b"));
}

#[test]
fn a_malformed_or_repeated_attribute_is_refused() {
    let err = refused(r"<mujoco><worldbody><body><geom size=/></body></worldbody></mujoco>");
    assert!(matches!(&err, MjcfError::Xml { .. }), "{err}");
    let err = refused(
        r#"<mujoco><worldbody><body><geom size="0.1" size="0.2"/></body></worldbody></mujoco>"#,
    );
    assert!(matches!(&err, MjcfError::Xml { .. }), "{err}");
}

// ── each reader's count, as MuJoCo's call reads it ──

#[test]
fn a_muscle_timeconst_takes_two_numbers_and_a_cylinder_bias_three() {
    let err = refused(
        r#"<mujoco>
            <worldbody><body><joint name="j"/><geom size="0.1"/></body></worldbody>
            <actuator><muscle joint="j" timeconst="0.01"/></actuator>
        </mujoco>"#,
    );
    assert_count(&err, "muscle", "timeconst", ValueCount::Exactly(2), 1);
    let err = refused(
        r#"<mujoco>
            <worldbody><body><joint name="j"/><geom size="0.1"/></body></worldbody>
            <actuator><cylinder joint="j" bias="1 2"/></actuator>
        </mujoco>"#,
    );
    assert_count(&err, "cylinder", "bias", ValueCount::Exactly(3), 2);
}

#[test]
fn a_joint_range_and_a_weld_relpose_take_their_counts() {
    let err = refused(
        r#"<mujoco><worldbody><body><joint range="1"/><geom size="0.1"/></body></worldbody></mujoco>"#,
    );
    assert_count(&err, "joint", "range", ValueCount::Exactly(2), 1);
    let err = refused(
        r#"<mujoco>
            <worldbody><body name="a"><freejoint/><geom size="0.1"/></body></worldbody>
            <equality><weld body1="a" relpose="0 0 0 1 0 0"/></equality>
        </mujoco>"#,
    );
    assert_count(&err, "weld", "relpose", ValueCount::Exactly(7), 6);
}

#[test]
fn a_composite_count_takes_at_most_three_and_a_flexcomp_count_and_spacing_three() {
    let err = refused(
        r#"<mujoco><worldbody><composite type="cable" count="5 1 1 1" curve="s 0 0" size="1"><geom size="0.01"/></composite></worldbody></mujoco>"#,
    );
    assert_count(&err, "composite", "count", ValueCount::AtMost(3), 4);
    // sim-mjcf reads a <flexcomp> in <deformable>, MuJoCo in a body (ledger
    // L152); the counts were measured in MuJoCo's form.
    let err = refused(
        r#"<mujoco><worldbody/><deformable><flexcomp name="f" type="grid" count="2 2" spacing="1 1 1" dim="2"/></deformable></mujoco>"#,
    );
    assert_count(&err, "flexcomp", "count", ValueCount::Exactly(3), 2);
    let err = refused(
        r#"<mujoco><worldbody/><deformable><flexcomp name="f" type="grid" count="2 2 1" spacing="1.0" dim="2"/></deformable></mujoco>"#,
    );
    assert_count(&err, "flexcomp", "spacing", ValueCount::Exactly(3), 1);
}

#[test]
fn a_flexcomp_spaces_its_grid_along_each_axis() {
    // MuJoCo's grid, in a body, spans 2 in y as well (measured; ledger L152
    // has the form).
    let m = load(
        r#"<mujoco><worldbody/><deformable><flexcomp name="f" type="grid" count="2 2 1" spacing="1 2 3" dim="2"/></deformable></mujoco>"#,
    );
    let mut d = m.make_data();
    d.forward(&m).unwrap();
    let ys: Vec<f64> = (0..m.nflexvert).map(|v| d.flexvert_xpos[v].y).collect();
    let span =
        ys.iter().copied().fold(f64::MIN, f64::max) - ys.iter().copied().fold(f64::MAX, f64::min);
    assert!((span - 2.0).abs() < 1e-12, "{ys:?}");
}

#[test]
fn a_tendon_polycoef_takes_at_most_five_and_a_short_one_keeps_the_default() {
    let err = refused(
        r#"<mujoco>
            <worldbody><body><joint name="a"/><geom size="0.1"/></body></worldbody>
            <tendon>
                <fixed name="t1"><joint joint="a" coef="1"/></fixed>
                <fixed name="t2"><joint joint="a" coef="2"/></fixed>
            </tendon>
            <equality><tendon tendon1="t1" tendon2="t2" polycoef="0 1 0 0 0 0"/></equality>
        </mujoco>"#,
    );
    assert_count(&err, "tendon", "polycoef", ValueCount::AtMost(5), 6);
    let m = load(
        r#"<mujoco>
            <worldbody>
                <body><joint name="a"/><geom size="0.1"/></body>
                <body pos="1 0 0"><joint name="b"/><geom size="0.1"/></body>
            </worldbody>
            <equality><joint joint1="a" joint2="b" polycoef="0.5"/></equality>
        </mujoco>"#,
    );
    assert_eq!(m.eq_data[0][..5], [0.5, 1.0, 0.0, 0.0, 0.0]);
}

#[test]
fn a_disabled_actuator_group_is_one_of_0_to_30() {
    for groups in ["31", "-1"] {
        let err = refused(&format!(
            r#"<mujoco><option actuatorgroupdisable="{groups}"/><worldbody/></mujoco>"#
        ));
        assert!(
            matches!(&err, MjcfError::InvalidValue { attribute, .. } if attribute == "actuatorgroupdisable"),
            "{groups}: {err}"
        );
    }
    let m = load(r#"<mujoco><option actuatorgroupdisable="1 2"/><worldbody/></mujoco>"#);
    assert_eq!(m.disableactuator, 6);
}

#[test]
fn a_keyframe_time_and_a_mesh_face_take_their_numbers() {
    let err = refused(
        r#"<mujoco><worldbody><body><freejoint/><geom size="0.1"/></body></worldbody><keyframe><key time="abc"/></keyframe></mujoco>"#,
    );
    assert_bad_number(&err, "key", "time");
    let err = refused(
        r#"<mujoco>
            <asset><mesh name="m" vertex="0 0 0 1 0 0 0 1 0 0 0 1" face="0.5 1 2 0 1 3"/></asset>
            <worldbody><body><geom type="mesh" mesh="m"/></body></worldbody>
        </mujoco>"#,
    );
    assert_bad_number(&err, "mesh", "face");
}

#[test]
fn a_pair_friction_written_short_keeps_the_pair_default() {
    let m = load(
        r#"<mujoco>
            <worldbody>
                <body><freejoint/><geom name="a" size="0.1"/></body>
                <body pos="0 0 1"><freejoint/><geom name="b" size="0.1"/></body>
            </worldbody>
            <contact><pair geom1="a" geom2="b" friction="0.7"/></contact>
        </mujoco>"#,
    );
    assert_eq!(
        m.contact_pairs[0].friction,
        [0.7, 1.0, 0.005, 0.0001, 0.0001]
    );
}

#[test]
fn a_sensor_interval_reads_its_period() {
    // The phase, MuJoCo's second value, is read with delay and history (book L48).
    let m = load(
        r#"<mujoco>
            <worldbody><body><joint name="j"/><geom size="0.1"/></body></worldbody>
            <sensor><jointpos joint="j" nsample="2" interval="0.03 -0.01"/></sensor>
        </mujoco>"#,
    );
    assert_eq!(m.sensor_interval[0].0, 0.03);
}

#[test]
fn a_hexadecimal_float_is_a_stated_limitation() {
    // MuJoCo's strtod reads 0x1p1 as 2; Rust's parser has no hexadecimal floats (D-HEX-FLOAT).
    let err = refused(
        r#"<mujoco><worldbody><body><freejoint/><geom size="0.1" mass="0x1p1"/></body></worldbody></mujoco>"#,
    );
    assert_bad_number(&err, "geom", "mass");
}

#[test]
fn an_ampersand_that_starts_no_known_entity_is_kept() {
    // tinyxml2 replaces the five predefined entities and character references only.
    let m = load(
        r#"<mujoco><worldbody><site name="x &amp; y"/><site name="a & b"/><site name="a&foo;b"/><site name="a&lt;b&#62;"/></worldbody></mujoco>"#,
    );
    let names: Vec<_> = m
        .site_name
        .iter()
        .map(|n| n.as_deref().unwrap_or(""))
        .collect();
    assert_eq!(names, ["x & y", "a & b", "a&foo;b", "a<b>"]);
}

// ── the class merge of every `exact = false` attribute ──

/// One prefix attribute: where it goes, the class's full value, the element's
/// first value, and the merged values MuJoCo 3.5.0 builds.
struct ClassTail {
    tag: &'static str,
    class: &'static str,
    written: &'static str,
    expected: &'static [f64],
    read: fn(&Model) -> Vec<f64>,
}

/// A document with `tag` written as `element` (`<tag ATTRS/>`'s attributes),
/// under `defaults`.
fn class_tail_doc(tag: &str, element: &str, defaults: &str) -> String {
    let world = r#"<worldbody>
        <body><joint name="j" limited="true" range="0 1" JOINT/><geom name="a" size="0.1" GEOM/><site SITE/></body>
        <body pos="0 0 1"><freejoint/><geom name="b" size="0.1"/></body>
        <body name="c" pos="1 0 0"><freejoint/><geom size="0.1"/></body>
    </worldbody>"#;
    let slot = |t: &str| if t == tag { element } else { "" };
    let world = world
        .replace("JOINT", slot("joint"))
        .replace("GEOM", slot("geom"))
        .replace("SITE", slot("site"));
    let extra = match tag {
        "tendon" => format!(
            r#"<tendon><fixed limited="true" range="0 1" {element}><joint joint="j" coef="1"/></fixed></tendon>"#
        ),
        "pair" => format!(r#"<contact><pair geom1="a" geom2="b" {element}/></contact>"#),
        "equality" => format!(r#"<equality><weld body1="c" {element}/></equality>"#),
        "motor" | "general" => format!(r#"<actuator><{tag} joint="j" {element}/></actuator>"#),
        _ => String::new(),
    };
    format!("<mujoco><default>{defaults}</default>{world}{extra}</mujoco>")
}

/// Every attribute MuJoCo reads with `exact = false` that a class can set.
const CLASS_TAILS: &[ClassTail] = &[
    ClassTail {
        tag: "geom",
        class: r#"friction="1 0.1 0.2""#,
        written: r#"friction="0.7""#,
        expected: &[0.7, 0.1, 0.2],
        read: |m| m.geom_friction[0].iter().copied().collect(),
    },
    ClassTail {
        tag: "geom",
        class: r#"solref="0.03 0.5""#,
        written: r#"solref="0.05""#,
        expected: &[0.05, 0.5],
        read: |m| m.geom_solref[0].to_vec(),
    },
    ClassTail {
        tag: "geom",
        class: r#"solimp="0.7 0.8 0.002 0.4 3""#,
        written: r#"solimp="0.6""#,
        expected: &[0.6, 0.8, 0.002, 0.4, 3.0],
        read: |m| m.geom_solimp[0].to_vec(),
    },
    ClassTail {
        tag: "geom",
        class: r#"fluidshape="ellipsoid" fluidcoef="0.1 0.2 0.3 0.4 0.5""#,
        written: r#"fluidcoef="0.9""#,
        expected: &[0.9, 0.2, 0.3, 0.4, 0.5],
        read: |m| m.geom_fluid[0][1..6].to_vec(),
    },
    ClassTail {
        tag: "joint",
        class: r#"solreflimit="0.03 0.5""#,
        written: r#"solreflimit="0.05""#,
        expected: &[0.05, 0.5],
        read: |m| m.jnt_solref[0].to_vec(),
    },
    ClassTail {
        tag: "joint",
        class: r#"solimplimit="0.7 0.8 0.002 0.4 3""#,
        written: r#"solimplimit="0.6""#,
        expected: &[0.6, 0.8, 0.002, 0.4, 3.0],
        read: |m| m.jnt_solimp[0].to_vec(),
    },
    ClassTail {
        tag: "joint",
        class: r#"solreffriction="0.03 0.5""#,
        written: r#"solreffriction="0.05""#,
        expected: &[0.05, 0.5],
        read: |m| m.dof_solref[0].to_vec(),
    },
    ClassTail {
        tag: "joint",
        class: r#"solimpfriction="0.7 0.8 0.002 0.4 3""#,
        written: r#"solimpfriction="0.6""#,
        expected: &[0.6, 0.8, 0.002, 0.4, 3.0],
        read: |m| m.dof_solimp[0].to_vec(),
    },
    ClassTail {
        tag: "site",
        class: r#"size="0.1 0.2 0.3""#,
        written: r#"size="0.9""#,
        expected: &[0.9, 0.2, 0.3],
        read: |m| m.site_size[0].iter().copied().collect(),
    },
    ClassTail {
        tag: "motor",
        class: r#"gear="2 3 4 5 6 7""#,
        written: r#"gear="9""#,
        expected: &[9.0, 3.0, 4.0, 5.0, 6.0, 7.0],
        read: |m| m.actuator_gear[0].to_vec(),
    },
    ClassTail {
        tag: "general",
        class: r#"gainprm="1 2 3 4 5 6 7 8 9""#,
        written: r#"gainprm="9""#,
        expected: &[9.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0],
        read: |m| m.actuator_gainprm[0].to_vec(),
    },
    ClassTail {
        tag: "general",
        class: r#"biasprm="1 2 3 4 5 6 7 8 9""#,
        written: r#"biasprm="9""#,
        expected: &[9.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0],
        read: |m| m.actuator_biasprm[0].to_vec(),
    },
    ClassTail {
        tag: "general",
        class: r#"dyntype="filter" dynprm="1 2 3 4 5 6 7 8 9 10""#,
        written: r#"dynprm="9""#,
        expected: &[9.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0],
        read: |m| m.actuator_dynprm[0].to_vec(),
    },
    ClassTail {
        tag: "tendon",
        class: r#"solreflimit="0.03 0.5""#,
        written: r#"solreflimit="0.05""#,
        expected: &[0.05, 0.5],
        read: |m| m.tendon_solref_lim[0].to_vec(),
    },
    ClassTail {
        tag: "tendon",
        class: r#"solimplimit="0.7 0.8 0.002 0.4 3""#,
        written: r#"solimplimit="0.6""#,
        expected: &[0.6, 0.8, 0.002, 0.4, 3.0],
        read: |m| m.tendon_solimp_lim[0].to_vec(),
    },
    ClassTail {
        tag: "tendon",
        class: r#"solreffriction="0.03 0.5""#,
        written: r#"solreffriction="0.05""#,
        expected: &[0.05, 0.5],
        read: |m| m.tendon_solref_fri[0].to_vec(),
    },
    ClassTail {
        tag: "tendon",
        class: r#"solimpfriction="0.7 0.8 0.002 0.4 3""#,
        written: r#"solimpfriction="0.6""#,
        expected: &[0.6, 0.8, 0.002, 0.4, 3.0],
        read: |m| m.tendon_solimp_fri[0].to_vec(),
    },
    ClassTail {
        tag: "pair",
        class: r#"friction="2 3 0.5 0.06 0.07""#,
        written: r#"friction="0.7""#,
        expected: &[0.7, 3.0, 0.5, 0.06, 0.07],
        read: |m| m.contact_pairs[0].friction.to_vec(),
    },
    ClassTail {
        tag: "pair",
        class: r#"solref="0.03 0.5""#,
        written: r#"solref="0.05""#,
        expected: &[0.05, 0.5],
        read: |m| m.contact_pairs[0].solref.to_vec(),
    },
    ClassTail {
        tag: "pair",
        class: r#"solreffriction="0.03 0.5""#,
        written: r#"solreffriction="0.05""#,
        expected: &[0.05, 0.5],
        read: |m| m.contact_pairs[0].solreffriction.to_vec(),
    },
    ClassTail {
        tag: "pair",
        class: r#"solimp="0.7 0.8 0.002 0.4 3""#,
        written: r#"solimp="0.6""#,
        expected: &[0.6, 0.8, 0.002, 0.4, 3.0],
        read: |m| m.contact_pairs[0].solimp.to_vec(),
    },
    ClassTail {
        tag: "equality",
        class: r#"solref="0.03 0.5""#,
        written: r#"solref="0.05""#,
        expected: &[0.05, 0.5],
        read: |m| m.eq_solref[0].to_vec(),
    },
    ClassTail {
        tag: "equality",
        class: r#"solimp="0.7 0.8 0.002 0.4 3""#,
        written: r#"solimp="0.6""#,
        expected: &[0.6, 0.8, 0.002, 0.4, 3.0],
        read: |m| m.eq_solimp[0].to_vec(),
    },
];

#[test]
fn every_prefix_attribute_keeps_its_class_tail() {
    for case in CLASS_TAILS {
        let tag = case.tag;
        // The element written over its class, and a child class written over
        // its parent with the element in the child class.
        let over_class = class_tail_doc(tag, case.written, &format!("<{tag} {}/>", case.class));
        let over_parent = class_tail_doc(
            tag,
            r#"class="c""#,
            &format!(
                r#"<{tag} {}/><default class="c"><{tag} {}/></default>"#,
                case.class, case.written
            ),
        );
        for (form, xml) in [
            ("element over class", over_class),
            ("child class over parent", over_parent),
        ] {
            let got = (case.read)(&load(&xml));
            assert_eq!(got, case.expected, "{tag} {} {form}", case.written);
        }
    }
}

#[test]
fn an_integer_list_and_text_follow_the_rest_of_the_reader() {
    let err = refused(
        r#"<mujoco><worldbody><body><freejoint/><geom size="0.1" condim="3 4"/></body></worldbody></mujoco>"#,
    );
    assert_count(&err, "geom", "condim", ValueCount::Exactly(1), 2);
    let err =
        refused(r#"<mujoco><option actuatorgroupdisable="99999999999"/><worldbody/></mujoco>"#);
    assert!(
        matches!(&err, MjcfError::OutOfRange { attribute, .. } if attribute == "actuatorgroupdisable"),
        "{err}"
    );
    let m = load(
        "<mujoco><worldbody><site name=\"a\r\nb\"/><site name=\"a&#x3e;b\"/></worldbody></mujoco>",
    );
    let names: Vec<_> = m
        .site_name
        .iter()
        .map(|n| n.as_deref().unwrap_or(""))
        .collect();
    assert_eq!(names, ["a\nb", "a>b"]);
}

// ── quaternions and orientations (book L05, `ReadQuat` and `ReadAlternative`) ──

/// The error is a quaternion of zeros on `element`.
fn assert_zero_quat(err: &MjcfError, element: &str) {
    assert!(
        matches!(err, MjcfError::InvalidValue { element: e, attribute, reason, .. }
            if e == element && attribute == "quat" && reason.contains("zero quaternion")),
        "{err}"
    );
}

/// The error is two orientations, `first` and `second`, on `element`.
fn assert_two_orientations(err: &MjcfError, element: &str, first: &str, second: &str) {
    assert!(
        matches!(err, MjcfError::ConflictingAttributes { element: e, attributes, .. }
            if e == element && attributes == &[first, second]),
        "{err}"
    );
}

#[test]
fn an_element_takes_one_orientation() {
    let err = refused(
        r#"<mujoco><worldbody><body euler="0 0 30" zaxis="1 0 0"><geom size="0.1"/></body></worldbody></mujoco>"#,
    );
    assert_two_orientations(&err, "body", "zaxis", "euler");
    let err = refused(
        r#"<mujoco><worldbody><body><geom size="0.1" quat="1 0 0 0" euler="0 0 30"/></body></worldbody></mujoco>"#,
    );
    assert_two_orientations(&err, "geom", "quat", "euler");
    let err = refused(
        r#"<mujoco><worldbody><body><geom size="0.1"/><site axisangle="0 0 1 30" xyaxes="1 0 0 0 1 0"/></body></worldbody></mujoco>"#,
    );
    assert_two_orientations(&err, "site", "axisangle", "xyaxes");
    let err = refused(
        r#"<mujoco><worldbody><frame quat="1 0 0 0" euler="0 0 30"><body><geom size="0.1"/></body></frame></worldbody></mujoco>"#,
    );
    assert_two_orientations(&err, "frame", "quat", "euler");
    let err = refused(
        r#"<mujoco><worldbody><body><inertial pos="0 0 0" mass="1" diaginertia="1 1 1" quat="1 0 0 0" euler="0 0 30"/></body></worldbody></mujoco>"#,
    );
    assert_two_orientations(&err, "inertial", "quat", "euler");
    let err = refused(
        r#"<mujoco><default><geom quat="1 0 0 0" euler="0 0 1"/></default><worldbody/></mujoco>"#,
    );
    assert_two_orientations(&err, "geom", "quat", "euler");
}

#[test]
fn a_written_quat_counts_and_an_empty_alternative_does_not() {
    let err = refused(
        r#"<mujoco><worldbody><body quat="" euler="0 0 30"><geom size="0.1"/></body></worldbody></mujoco>"#,
    );
    assert_two_orientations(&err, "body", "quat", "euler");
    load(
        r#"<mujoco><worldbody><body quat="1 0 0 0" euler=""><geom size="0.1"/></body></worldbody></mujoco>"#,
    );
    load(r#"<mujoco><worldbody><body quat=""><geom size="0.1"/></body></worldbody></mujoco>"#);
    // A class's quat and an element's euler are not two on one element.
    load(
        r#"<mujoco><default><geom quat="1 0 0 0"/></default><worldbody><body><geom size="0.1" euler="0 0 30"/></body></worldbody></mujoco>"#,
    );
}

#[test]
fn a_quaternion_of_zeros_is_refused() {
    let err = refused(
        r#"<mujoco><worldbody><body quat="0 0 0 0"><geom size="0.1"/></body></worldbody></mujoco>"#,
    );
    assert_zero_quat(&err, "body");
    let err = refused(
        r#"<mujoco><worldbody><body><geom size="0.1" quat="0 0 0 0"/></body></worldbody></mujoco>"#,
    );
    assert_zero_quat(&err, "geom");
    let err = refused(
        r#"<mujoco><worldbody><body><geom size="0.1"/><site quat="0 0 0 0"/></body></worldbody></mujoco>"#,
    );
    assert_zero_quat(&err, "site");
    let err = refused(
        r#"<mujoco><worldbody><frame quat="0 0 0 0"><body><geom size="0.1"/></body></frame></worldbody></mujoco>"#,
    );
    assert_zero_quat(&err, "frame");
    let err = refused(
        r#"<mujoco><worldbody><body><inertial pos="0 0 0" mass="1" diaginertia="1 1 1" quat="0 0 0 0"/></body></worldbody></mujoco>"#,
    );
    assert_zero_quat(&err, "inertial");
    let err = refused(r#"<mujoco><default><site quat="0 0 0 0"/></default><worldbody/></mujoco>"#);
    assert_zero_quat(&err, "site");
}

// ── what the mutants of L05's first commit showed untested ──

#[test]
fn a_malformed_attribute_is_refused_on_an_element_sim_mjcf_does_not_read() {
    // tinyxml2 refuses the document before MuJoCo reads any element (measured).
    for xml in [
        r"<mujoco><visual><global fovy=/></visual><worldbody/></mujoco>",
        r#"<mujoco><statistic extent="1" extent="2"/><worldbody/></mujoco>"#,
        r"<mujoco><worldbody><light pos=></light></worldbody></mujoco>",
    ] {
        let err = refused(xml);
        assert!(matches!(&err, MjcfError::Xml { .. }), "{xml}: {err}");
    }
}

#[test]
fn every_equality_keeps_its_class_tail() {
    let world = r#"<worldbody>
            <body name="a"><joint name="ja"/><geom name="ga" size="0.1"/></body>
            <body name="b" pos="1 0 0"><joint name="jb"/><geom name="gb" size="0.1"/></body>
        </worldbody>
        <tendon>
            <fixed name="t1"><joint joint="ja" coef="1"/></fixed>
            <fixed name="t2"><joint joint="jb" coef="1"/></fixed>
        </tendon>"#;
    for element in [
        r#"<connect body1="a" body2="b" anchor="0 0 0" ATTR/>"#,
        r#"<joint joint1="ja" joint2="jb" ATTR/>"#,
        r#"<tendon tendon1="t1" tendon2="t2" ATTR/>"#,
        // sim-mjcf's <distance> (MuJoCo 3.5.0 refuses the element): no measured
        // value, so it is held to the other equalities' rule.
        r#"<distance geom1="ga" geom2="gb" ATTR/>"#,
    ] {
        let doc = |class: &str, written: &str| {
            format!(
                "<mujoco><default><equality {class}/></default>{world}<equality>{}</equality></mujoco>",
                element.replace("ATTR", written)
            )
        };
        let m = load(&doc(r#"solref="0.03 0.5""#, r#"solref="0.05""#));
        assert_eq!(m.eq_solref[0], [0.05, 0.5], "{element}");
        let m = load(&doc(r#"solimp="0.7 0.8 0.002 0.4 3""#, r#"solimp="0.6""#));
        assert_eq!(m.eq_solimp[0], [0.6, 0.8, 0.002, 0.4, 3.0], "{element}");
    }
}

#[test]
fn an_override_written_short_keeps_the_rest_of_the_default() {
    let m = load(
        r#"<mujoco><option o_solref="0.05" o_solimp="0.6" o_friction="0.7"><flag override="enable"/></option><worldbody/></mujoco>"#,
    );
    assert_eq!(m.o_solref, [0.05, 1.0]);
    assert_eq!(m.o_solimp, [0.6, 0.95, 0.001, 0.5, 2.0]);
    assert_eq!(m.o_friction, [0.7, 1.0, 0.005, 0.0001, 0.0001]);
}

#[test]
fn at_most_31_actuator_groups_are_disabled() {
    let groups = vec!["1"; 32].join(" ");
    let err = refused(&format!(
        r#"<mujoco><option actuatorgroupdisable="{groups}"/><worldbody/></mujoco>"#
    ));
    assert_count(
        &err,
        "option",
        "actuatorgroupdisable",
        ValueCount::AtMost(31),
        32,
    );
}

#[test]
fn a_box_flexcomp_spaces_its_grid_along_z() {
    // sim-mjcf's box generator, not MuJoCo's (ledger L52 U09): the spacing is
    // read per axis, as MuJoCo reads it.
    let m = load(
        r#"<mujoco><worldbody/><deformable><flexcomp name="f" type="box" count="2 2 2" spacing="1 1 3"/></deformable></mujoco>"#,
    );
    let mut d = m.make_data();
    d.forward(&m).unwrap();
    let zs: Vec<f64> = (0..m.nflexvert).map(|v| d.flexvert_xpos[v].z).collect();
    let span =
        zs.iter().copied().fold(f64::MIN, f64::max) - zs.iter().copied().fold(f64::MAX, f64::min);
    assert!((span - 3.0).abs() < 1e-12, "{zs:?}");
}

// ── required attributes (book L05; MuJoCo's reader marks them `required`) ──

/// The error is `attribute` missing on `element`.
fn assert_missing(err: &MjcfError, element: &str, attribute: &str) {
    assert!(
        matches!(err, MjcfError::MissingAttribute { element: e, attribute: a, .. } if e == element && a == attribute),
        "{err}"
    );
}

#[test]
fn an_inertial_needs_its_pos_and_mass() {
    let err = refused(
        r#"<mujoco><worldbody><body><inertial mass="1" diaginertia="1 1 1"/></body></worldbody></mujoco>"#,
    );
    assert_missing(&err, "inertial", "pos");
    let err = refused(
        r#"<mujoco><worldbody><body><inertial pos="" mass="1" diaginertia="1 1 1"/></body></worldbody></mujoco>"#,
    );
    assert_missing(&err, "inertial", "pos");
    let err = refused(
        r#"<mujoco><worldbody><body><inertial pos="0 0 0" diaginertia="1 1 1"/></body></worldbody></mujoco>"#,
    );
    assert_missing(&err, "inertial", "mass");
}

#[test]
fn a_tendons_path_names_its_parts_and_their_numbers() {
    let joint = r#"<worldbody><body><joint name="j"/><geom size="0.1"/></body></worldbody>"#;
    let err = refused(&format!(
        r#"<mujoco>{joint}<tendon><fixed><joint joint="j"/></fixed></tendon></mujoco>"#
    ));
    assert_missing(&err, "joint", "coef");
    let err = refused(&format!(
        r#"<mujoco>{joint}<tendon><fixed><joint joint="j" coef=""/></fixed></tendon></mujoco>"#
    ));
    assert_missing(&err, "joint", "coef");
    let err = refused(&format!(
        r#"<mujoco>{joint}<tendon><fixed><joint coef="1"/></fixed></tendon></mujoco>"#
    ));
    assert_missing(&err, "joint", "joint");
    let sites = r#"<worldbody><body><geom name="g" size="0.1"/><site name="s1"/><site name="s2" pos="0 0 1"/></body></worldbody>"#;
    let err = refused(&format!(
        r#"<mujoco>{sites}<tendon><spatial><site site="s1"/><pulley/><site site="s2"/></spatial></tendon></mujoco>"#
    ));
    assert_missing(&err, "pulley", "divisor");
    let err = refused(&format!(
        r#"<mujoco>{sites}<tendon><spatial><site site="s1"/><site/><site site="s2"/></spatial></tendon></mujoco>"#
    ));
    assert_missing(&err, "site", "site");
    let err = refused(&format!(
        r#"<mujoco>{sites}<tendon><spatial><site site="s1"/><geom/><site site="s2"/></spatial></tendon></mujoco>"#
    ));
    assert_missing(&err, "geom", "geom");
}

#[test]
fn a_composite_joint_names_its_kind() {
    let err = refused(
        r#"<mujoco><worldbody><composite type="cable" count="5 1 1" curve="s 0 0" size="1"><joint damping="1"/><geom type="capsule" size="0.01"/></composite></worldbody></mujoco>"#,
    );
    assert_missing(&err, "joint", "kind");
}

// ── what the mutants of the quaternion and required-attribute commits found ──

#[test]
fn a_class_takes_one_orientation_and_no_quaternion_of_zeros() {
    let err = refused(
        r#"<mujoco><default><site quat="1 0 0 0" euler="0 0 1"/></default><worldbody/></mujoco>"#,
    );
    assert_two_orientations(&err, "site", "quat", "euler");
    let err = refused(r#"<mujoco><default><geom quat="0 0 0 0"/></default><worldbody/></mujoco>"#);
    assert_zero_quat(&err, "geom");
}

#[test]
fn a_composite_joint_with_a_closing_tag_names_its_kind() {
    let err = refused(
        r#"<mujoco><worldbody><composite type="cable" count="5 1 1" curve="s 0 0" size="1"><joint damping="1"></joint><geom type="capsule" size="0.01"/></composite></worldbody></mujoco>"#,
    );
    assert_missing(&err, "joint", "kind");
}
