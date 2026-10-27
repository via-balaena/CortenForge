//! What a refused document's error says: its variant, and where it was found.
//!
//! These match the error as code outside the crate must, with `{ .. }`.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use sim_mjcf::{ElementKind, MjcfError, load_model, load_model_from_file};

#[test]
fn a_repeated_body_name_is_a_duplicate_name() {
    let err = load_model(
        r#"<mujoco>
            <worldbody>
                <body name="a"><geom size="0.1"/></body>
                <body name="a"><geom size="0.1"/></body>
            </worldbody>
        </mujoco>"#,
    )
    .unwrap_err();
    assert!(
        matches!(&err, MjcfError::DuplicateName { kind: ElementKind::Body, name, .. } if name == "a"),
        "{err}"
    );
}

#[test]
fn a_missing_file_is_an_io_error_naming_it() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("missing.xml");
    let err = load_model_from_file(&path).unwrap_err();
    match &err {
        MjcfError::Io {
            path: p, source, ..
        } => {
            assert_eq!(p, &path);
            assert_eq!(source.kind(), std::io::ErrorKind::NotFound);
        }
        other => panic!("{other}"),
    }
}

#[test]
fn a_deprecated_composite_type_is_an_invalid_value_not_a_limitation() {
    let err = load_model(
        r#"<mujoco>
            <worldbody>
                <body><composite type="grid" count="3 3 1"><geom size="0.1"/></composite></body>
            </worldbody>
        </mujoco>"#,
    )
    .unwrap_err();
    assert!(
        !err.to_string().to_lowercase().contains("unsupported"),
        "a MuJoCo rule, not a limitation: {err}"
    );
    assert!(
        matches!(
            &err,
            MjcfError::InvalidValue { element, attribute, .. }
                if element == "composite" && attribute == "type"
        ),
        "{err}"
    );
}

#[test]
fn a_fixed_tendon_on_an_unknown_joint_is_an_undefined_reference() {
    let err = load_model(
        r#"<mujoco>
            <worldbody>
                <body><joint name="j" type="hinge"/><geom size="0.1"/></body>
            </worldbody>
            <tendon>
                <fixed name="t"><joint joint="nope" coef="1"/></fixed>
            </tendon>
        </mujoco>"#,
    )
    .unwrap_err();
    assert!(
        matches!(&err, MjcfError::UndefinedReference { kind: ElementKind::Joint, name, .. } if name == "nope"),
        "{err}"
    );
    assert!(err.to_string().contains("(tendon 't')"), "{err}");
}

#[test]
fn a_parse_error_names_its_line_and_element() {
    let err = load_model(
        "<mujoco>\n  <worldbody>\n    <body name=\"a\">\n      <geom size=\"0.1 x\"/>\n    </body>\n  </worldbody>\n</mujoco>",
    )
    .unwrap_err();
    let text = err.to_string();
    assert!(text.contains("line 4"), "{text}");
    assert!(text.contains("body 'a' > geom"), "{text}");
}

#[test]
fn a_build_error_names_an_unnamed_element_by_its_index() {
    let err = load_model(
        r#"<mujoco>
            <worldbody>
                <body><joint name="j" type="hinge"/><geom size="0.1"/></body>
            </worldbody>
            <actuator>
                <motor joint="j"/>
                <motor joint="nope"/>
            </actuator>
        </mujoco>"#,
    )
    .unwrap_err();
    let text = err.to_string();
    assert!(text.contains("actuator #1"), "{text}");
    assert!(text.contains("'nope'"), "{text}");
}

/// `xml`'s error, which must exist.
fn refusal(xml: &str) -> MjcfError {
    match load_model(xml) {
        Ok(_) => panic!("loaded: {xml}"),
        Err(err) => err,
    }
}

/// Asserts that `xml`'s error ends with the location `at`.
fn assert_located(xml: &str, at: &str) -> MjcfError {
    let err = refusal(xml);
    assert!(err.to_string().ends_with(at), "wanted {at}: {err}");
    err
}

#[test]
fn a_syntax_error_names_its_own_line_and_the_elements_open_there() {
    // An unclosed quote on line 3; MuJoCo 3.5.0 reports line 3 too.
    let err = refusal(
        "<mujoco>\n  <worldbody>\n    <body name=\"a>\n    </body>\n  </worldbody>\n</mujoco>",
    );
    assert!(matches!(err, MjcfError::Xml { .. }), "{err}");
    assert!(err.to_string().ends_with("(line 3: worldbody)"), "{err}");
}

#[test]
fn a_build_error_in_a_sensor_equality_pair_or_exclude_names_it() {
    let bodies = r#"<worldbody>
            <body name="a"><joint name="j"/><geom name="g" size="0.1"/></body>
            <body name="b"><geom size="0.1"/></body>
        </worldbody>"#;
    let at = |section: &str, at: &str| {
        assert_located(&format!("<mujoco>{bodies}{section}</mujoco>"), at);
    };
    at(
        r#"<sensor><jointpos joint="j"/><jointpos joint="nope"/></sensor>"#,
        "(sensor #1)",
    );
    at(
        r#"<equality><connect name="c" body1="a" body2="nope" anchor="0 0 0"/></equality>"#,
        "(equality 'c')",
    );
    at(
        r#"<contact><pair geom1="g" geom2="nope"/></contact>"#,
        "(pair #0)",
    );
    let err = refusal(&format!(
        r#"<mujoco>{bodies}<contact><exclude body1="a" body2="b"/><exclude body1="a" body2="nope"/></contact></mujoco>"#
    ));
    assert!(
        matches!(&err, MjcfError::UndefinedReference { kind: ElementKind::Body, name, .. } if name == "nope"),
        "{err}"
    );
    assert!(err.to_string().ends_with("(exclude #1)"), "{err}");
}

#[test]
fn a_build_error_in_a_keyframe_names_it_by_its_tag() {
    assert_located(
        r#"<mujoco>
            <worldbody><body><joint/><geom size="0.1"/></body></worldbody>
            <keyframe><key name="k" qpos="1 2 3"/></keyframe>
        </mujoco>"#,
        "(key 'k')",
    );
}

#[test]
fn an_element_without_a_name_is_named_by_its_body() {
    assert_located(
        r#"<mujoco><worldbody>
            <body name="b"><geom size="0.1"/><geom type="mesh" mesh="nope"/></body>
        </worldbody></mujoco>"#,
        "(geom #1 in body 'b')",
    );
    assert_located(
        r#"<mujoco><worldbody>
            <body name="b"><geom size="0.1"/><site type="sphere" fromto="0 0 0 0 0 1"/></body>
        </worldbody></mujoco>"#,
        "(site in body 'b')",
    );
    assert_located(
        r#"<mujoco><worldbody>
            <body name="b"><geom size="0.1"/><frame childclass="nope"/></body>
        </worldbody></mujoco>"#,
        "(frame in body 'b')",
    );
    assert_located(
        r#"<mujoco><worldbody>
            <body name="b"><joint name="j"/><joint name="j"/><geom size="0.1"/></body>
        </worldbody></mujoco>"#,
        "(joint 'j' in body 'b')",
    );
}

#[test]
fn a_tendons_bad_coefficient_names_its_joint_and_tendon() {
    // The reader refuses a written NaN first (D-NONFINITE); a model built in
    // code is checked when it is built, and that check names the joint and
    // the tendon.
    let mut mjcf = sim_mjcf::parse_mjcf_str(
        r#"<mujoco>
            <worldbody>
                <body><joint name="j0"/><joint name="j1"/><geom size="0.1"/></body>
            </worldbody>
            <tendon><fixed><joint joint="j0" coef="1"/><joint joint="j1" coef="1"/></fixed></tendon>
        </mujoco>"#,
    )
    .unwrap();
    mjcf.tendons[0].joints[1].1 = f64::NAN;
    let err = sim_mjcf::model_from_mjcf(&mjcf, None).unwrap_err();
    assert!(
        err.to_string().ends_with("(joint 'j1' in tendon #0)"),
        "{err}"
    );
    assert!(
        matches!(&err, MjcfError::NonFinite { attribute, .. } if attribute == "coef"),
        "{err}"
    );
}

#[test]
fn a_keyframe_or_an_inertial_built_in_code_is_checked_for_finite_numbers() {
    fn inertial(mjcf: &mut sim_mjcf::MjcfModel) -> &mut sim_mjcf::MjcfInertial {
        mjcf.worldbody.children[0].inertial.as_mut().unwrap()
    }
    // The reader refuses a written NaN first (D-NONFINITE); a model built in
    // code is checked when it is built, at each number the row names.
    let built = || {
        sim_mjcf::parse_mjcf_str(
            r#"<mujoco>
                <worldbody>
                    <body><freejoint/><inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/></body>
                </worldbody>
                <keyframe><key time="0" qpos="0 0 0 1 0 0 0"/></keyframe>
            </mujoco>"#,
        )
        .unwrap()
    };
    let mut cases: Vec<(sim_mjcf::MjcfModel, &str, &str)> = Vec::new();
    let mut mjcf = built();
    mjcf.keyframes[0].time = f64::NAN;
    cases.push((mjcf, "key", "time"));
    let mut mjcf = built();
    mjcf.keyframes[0].qpos.as_mut().unwrap()[0] = f64::INFINITY;
    cases.push((mjcf, "key", "qpos[0]"));
    let mut mjcf = built();
    inertial(&mut mjcf).mass = f64::NAN;
    cases.push((mjcf, "inertial", "mass"));
    let mut mjcf = built();
    inertial(&mut mjcf).diaginertia.as_mut().unwrap().y = f64::NAN;
    cases.push((mjcf, "inertial", "diaginertia"));
    for (mjcf, element, attribute) in &cases {
        let err = sim_mjcf::model_from_mjcf(mjcf, None).unwrap_err();
        assert!(
            matches!(&err, MjcfError::NonFinite { element: e, attribute: a, .. } if e == element && a == attribute),
            "{element} {attribute}: {err}"
        );
    }
}

#[test]
fn a_mocap_body_or_composite_error_names_its_element() {
    assert_located(
        r#"<mujoco><worldbody>
            <body name="a"><geom size="0.1"/><body name="m" mocap="true"><geom size="0.1"/></body></body>
        </worldbody></mujoco>"#,
        "(body 'm')",
    );
    assert_located(
        r#"<mujoco><worldbody>
            <body name="holder"><composite type="grid" count="3 3 1"><geom size="0.1"/></composite></body>
        </worldbody></mujoco>"#,
        "(body 'holder')",
    );
}

#[test]
fn a_height_field_built_in_code_without_its_rows_names_it() {
    // The parser refuses a missing `nrow` first, so the builder's check is
    // reached by a model changed in code.
    let mut model = sim_mjcf::parse_mjcf_str(
        r#"<mujoco>
            <asset><hfield name="h" nrow="2" ncol="2" size="1 1 1 0.1" elevation="1 2 3 4"/></asset>
            <worldbody><geom type="hfield" hfield="h"/></worldbody>
        </mujoco>"#,
    )
    .unwrap();
    model.hfields[0].nrow = None;
    let err = sim_mjcf::model_from_mjcf(&model, None).unwrap_err();
    assert!(err.to_string().ends_with("(hfield 'h')"), "{err}");
}

#[test]
fn an_actuator_error_raised_while_building_names_the_actuator() {
    // The reader refuses the keyword first; a model built in code reaches the
    // builder, which refuses it with the same words and names the actuator.
    let mut mjcf = sim_mjcf::parse_mjcf_str(
        r#"<mujoco>
            <worldbody><body><joint name="j"/><geom size="0.1"/></body></worldbody>
            <actuator>
                <motor joint="j"/>
                <general joint="j"/>
            </actuator>
        </mujoco>"#,
    )
    .unwrap();
    mjcf.actuators[1].gaintype = Some("bogus".to_owned());
    let err = sim_mjcf::model_from_mjcf(&mjcf, None).unwrap_err();
    assert!(err.to_string().ends_with("(actuator #1)"), "{err}");
    assert!(
        matches!(&err, MjcfError::InvalidKeyword { attribute, value, .. }
            if attribute == "gaintype" && value == "bogus"),
        "{err}"
    );
}

#[test]
fn a_nan_option_or_spring_length_is_non_finite() {
    let err = refusal(r#"<mujoco><option gravity="0 0 nan"/><worldbody/></mujoco>"#);
    assert!(
        matches!(&err, MjcfError::NonFinite { element, attribute, .. }
            if element == "option" && attribute == "gravity"),
        "{err}"
    );
    let err = refusal(
        r#"<mujoco>
            <worldbody><body><joint name="j"/><geom size="0.1"/></body></worldbody>
            <tendon><fixed springlength="nan"><joint joint="j" coef="1"/></fixed></tendon>
        </mujoco>"#,
    );
    assert!(
        matches!(&err, MjcfError::NonFinite { element, attribute, .. }
            if element == "fixed" && attribute == "springlength"),
        "{err}"
    );
}

#[test]
fn a_reader_that_knows_its_attribute_names_it() {
    let err = refusal(
        r#"<mujoco><worldbody><body gravcomp="x"><geom size="0.1"/></body></worldbody></mujoco>"#,
    );
    assert!(
        matches!(&err, MjcfError::BadNumber { element, attribute, value, .. }
            if element == "body" && attribute == "gravcomp" && value == "x"),
        "{err}"
    );
    let err = refusal(
        r#"<mujoco><worldbody><geom size="0.1" fluidshape="ellipsoid" fluidcoef="1 2 3 4 5 6"/></worldbody></mujoco>"#,
    );
    assert!(
        matches!(&err, MjcfError::WrongValueCount { element, attribute, found: 6, .. }
            if element == "geom" && attribute == "fluidcoef"),
        "{err}"
    );
    assert!(
        err.to_string()
            .contains("in attribute 'fluidcoef' on <geom>"),
        "{err}"
    );
    let err = refusal(
        r#"<mujoco><asset><hfield name="h" nrow="2" ncol="2" size="1 1 1"/></asset><worldbody/></mujoco>"#,
    );
    assert!(
        matches!(&err, MjcfError::WrongValueCount { element, attribute, found: 3, .. }
            if element == "hfield" && attribute == "size"),
        "{err}"
    );
}

#[test]
fn a_rule_on_an_elements_value_names_the_element_by_its_tag() {
    // nuser_jnt's errors are a <joint>'s, not a "jnt"'s.
    let err = refusal(
        r#"<mujoco>
            <size nuser_jnt="1"/>
            <worldbody><body><joint name="j" user="1 2"/><geom size="0.1"/></body></worldbody>
        </mujoco>"#,
    );
    assert!(
        matches!(&err, MjcfError::InvalidValue { element, attribute, .. }
            if element == "joint" && attribute == "user"),
        "{err}"
    );
    // A muscle's parameter checks are on the <general> that names it.
    let err = refusal(
        r#"<mujoco>
            <worldbody><body><joint name="j"/><geom size="0.1"/></body></worldbody>
            <actuator>
                <general joint="j" dyntype="hillmuscle" gaintype="hillmuscle" biastype="hillmuscle"
                         gainprm="1 1 1 1 0"/>
            </actuator>
        </mujoco>"#,
    );
    assert!(
        matches!(&err, MjcfError::InvalidValue { element, attribute, .. }
            if element == "general" && attribute == "gainprm"),
        "{err}"
    );
}

#[test]
fn a_body_without_a_name_is_named_by_its_index() {
    // The world is body #0.
    assert_located(
        r#"<mujoco><worldbody>
            <body><geom size="0.1"/><geom type="mesh" mesh="nope"/></body>
        </worldbody></mujoco>"#,
        "(geom #1 in body #1)",
    );
    assert_located(
        r#"<mujoco><worldbody>
            <body><geom size="0.1"/><body mocap="true"><geom size="0.1"/></body></body>
        </worldbody></mujoco>"#,
        "(body #2)",
    );
}

#[test]
fn a_default_classs_fluidcoef_of_the_wrong_length_names_it() {
    let err = refusal(
        r#"<mujoco><default><geom fluidshape="ellipsoid" fluidcoef="1 2 3 4 5 6"/></default><worldbody/></mujoco>"#,
    );
    assert!(
        matches!(&err, MjcfError::WrongValueCount { element, attribute, found: 6, .. }
            if element == "geom" && attribute == "fluidcoef"),
        "{err}"
    );
}

#[test]
fn an_actuators_actdim_is_refused_as_not_read() {
    // MuJoCo 3.5.0 gives a user-dynamics actuator the written number of
    // activations; sim-mjcf gives every stateful actuator one.
    for xml in [
        r#"<mujoco>
            <worldbody><body><joint name="j"/><geom size="0.1"/></body></worldbody>
            <actuator><general joint="j" dyntype="user" actdim="2"/></actuator>
        </mujoco>"#,
        r#"<mujoco><default><general actdim="1"/></default><worldbody/></mujoco>"#,
    ] {
        let err = refusal(xml);
        assert!(
            matches!(&err, MjcfError::Unsupported { feature, .. } if feature.contains("actdim")),
            "{err}"
        );
    }
}

#[test]
fn a_syntax_error_in_a_document_with_includes_names_its_line() {
    // The include expander reads the document first; its syntax error is
    // named as the parser would name it.
    let dir = tempfile::tempdir().unwrap();
    std::fs::write(
        dir.path().join("part.xml"),
        r#"<mujoco><option timestep="0.01"/></mujoco>"#,
    )
    .unwrap();
    let path = dir.path().join("main.xml");
    std::fs::write(
        &path,
        "<mujoco>\n  <include file=\"part.xml\"/>\n  <worldbody>\n    <body name=\"a>\n    </body>\n  </worldbody>\n</mujoco>",
    )
    .unwrap();
    let err = load_model_from_file(&path).unwrap_err();
    assert!(matches!(err, MjcfError::Xml { .. }), "{err}");
    assert!(err.to_string().ends_with("(line 4: worldbody)"), "{err}");
}
