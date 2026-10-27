//! Keywords, read as MuJoCo 3.5.0's XML reader reads them (`xml_util.cc` `MapValue`): the
//! attribute's whole text, untrimmed and case-sensitive, must be one of its map's keywords
//! (`xml_native_reader.cc`, the `mjMap`s). Each refusal was measured on MuJoCo 3.5.0
//! ("invalid keyword: '…'"), except where a test says the word is an extension.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use sim_core::{ActuatorDynamics, FlexBendingType, GainType, Integrator, Model};
use sim_mjcf::{MjcfError, load_model};

fn load(xml: &str) -> Model {
    load_model(xml).unwrap_or_else(|e| panic!("refused: {e}"))
}

fn refused(xml: &str) -> MjcfError {
    match load_model(xml) {
        Ok(_) => panic!("loaded; the test expects a refusal"),
        Err(e) => e,
    }
}

/// The error is an invalid keyword `value` in `attribute` on `element`.
fn assert_keyword(err: &MjcfError, element: &str, attribute: &str, value: &str) {
    assert!(
        matches!(err, MjcfError::InvalidKeyword { element: e, attribute: a, value: v, .. }
            if e == element && a == attribute && v == value),
        "{err}"
    );
}

fn option(attrs: &str) -> String {
    format!("<mujoco><option {attrs}/><worldbody/></mujoco>")
}

fn compiler(attrs: &str) -> String {
    format!("<mujoco><compiler {attrs}/><worldbody/></mujoco>")
}

// ── <option> and <compiler> ──

#[test]
fn an_unknown_integrator_is_refused_not_dropped() {
    // Before: an unknown integrator was dropped and Euler kept.
    assert_keyword(
        &refused(&option(r#"integrator="RK45""#)),
        "option",
        "integrator",
        "RK45",
    );
}

#[test]
fn an_option_keyword_is_case_sensitive() {
    assert_keyword(
        &refused(&option(r#"integrator="euler""#)),
        "option",
        "integrator",
        "euler",
    );
    assert_keyword(
        &refused(&option(r#"solver="newton""#)),
        "option",
        "solver",
        "newton",
    );
    assert_keyword(
        &refused(&option(r#"cone="Elliptic""#)),
        "option",
        "cone",
        "Elliptic",
    );
    assert_keyword(
        &refused(&option(r#"jacobian="Dense""#)),
        "option",
        "jacobian",
        "Dense",
    );
    assert_eq!(
        load(&option(r#"integrator="Euler""#)).integrator,
        Integrator::Euler
    );
}

#[test]
fn a_keyword_is_not_trimmed_and_may_not_be_empty() {
    assert_keyword(
        &refused(&option(r#"integrator=" RK4""#)),
        "option",
        "integrator",
        " RK4",
    );
    assert_keyword(
        &refused(&option(r#"integrator="""#)),
        "option",
        "integrator",
        "",
    );
}

#[test]
fn implicitspringdamper_is_an_extension() {
    // MuJoCo 3.5.0 refuses it (measured); sim-mjcf keeps its integrator.
    assert_eq!(
        load(&option(r#"integrator="implicitspringdamper""#)).integrator,
        Integrator::ImplicitSpringDamper
    );
}

#[test]
fn a_compiler_keyword_is_case_sensitive() {
    assert_keyword(
        &refused(&compiler(r#"angle="RADIAN""#)),
        "compiler",
        "angle",
        "RADIAN",
    );
    assert_keyword(
        &refused(&compiler(r#"inertiafromgeom="TRUE""#)),
        "compiler",
        "inertiafromgeom",
        "TRUE",
    );
    assert_keyword(
        &refused(&compiler(r#"coordinate="LOCAL""#)),
        "compiler",
        "coordinate",
        "LOCAL",
    );
}

// ── types ──

#[test]
fn a_type_in_a_default_class_is_refused_not_dropped() {
    // Before: an unknown type in a class was dropped, and the element's own
    // type kept.
    let err = refused(
        r#"<mujoco><default><joint type="bogus"/></default>
            <worldbody><body><joint/><geom size="0.1"/></body></worldbody></mujoco>"#,
    );
    assert_keyword(&err, "joint", "type", "bogus");
    let err = refused(
        r#"<mujoco><default><geom type="bogus"/></default>
            <worldbody><body><geom size="0.1"/></body></worldbody></mujoco>"#,
    );
    assert_keyword(&err, "geom", "type", "bogus");
}

#[test]
fn cylindrical_planar_and_trimesh_are_not_types() {
    let err = refused(
        r#"<mujoco><worldbody><body><joint type="cylindrical"/><geom size="0.1"/></body></worldbody></mujoco>"#,
    );
    assert_keyword(&err, "joint", "type", "cylindrical");
    let err = refused(
        r#"<mujoco><worldbody><body><joint type="planar"/><geom size="0.1"/></body></worldbody></mujoco>"#,
    );
    assert_keyword(&err, "joint", "type", "planar");
    let err = refused(
        r#"<mujoco><worldbody><body><geom type="trimesh" size="0.1"/></body></worldbody></mujoco>"#,
    );
    assert_keyword(&err, "geom", "type", "trimesh");
}

#[test]
fn a_type_is_not_trimmed() {
    let err = refused(
        r#"<mujoco><worldbody><body><geom type=" box" size="0.1 0.1 0.1"/></body></worldbody></mujoco>"#,
    );
    assert_keyword(&err, "geom", "type", " box");
}

#[test]
fn an_unknown_site_type_is_refused() {
    // Before: an unknown word was a sphere.
    let err = refused(
        r#"<mujoco><worldbody><body><geom size="0.1"/><site type="bogus"/></body></worldbody></mujoco>"#,
    );
    assert_keyword(&err, "site", "type", "bogus");
    let err = refused(
        r#"<mujoco><default><site type="Box"/></default>
            <worldbody><body><geom size="0.1"/><site/></body></worldbody></mujoco>"#,
    );
    assert_keyword(&err, "site", "type", "Box");
}

// ── bodies, actuators, sensors ──

#[test]
fn an_unknown_sleep_policy_is_refused() {
    // Before: warned about and dropped.
    let err = refused(
        r#"<mujoco><worldbody><body sleep="Never"><freejoint/><geom size="0.1"/></body></worldbody></mujoco>"#,
    );
    assert_keyword(&err, "body", "sleep", "Never");
}

#[test]
fn an_actuators_keywords_are_read_when_written() {
    let general = |attrs: &str| {
        format!(
            r#"<mujoco><worldbody><body><joint name="j"/><geom size="0.1"/></body></worldbody>
            <actuator><general joint="j" {attrs}/></actuator></mujoco>"#
        )
    };
    // A class's keyword is refused though no actuator takes the class
    // (before: checked only when building an actuator that took it).
    let err =
        refused(r#"<mujoco><default><general biastype="Affine"/></default><worldbody/></mujoco>"#);
    assert_keyword(&err, "general", "biastype", "Affine");
    // Before: refused by the builder, on <general> for these two and on
    // <actuator> for interp.
    assert_keyword(
        &refused(&general(r#"gaintype="Fixed""#)),
        "general",
        "gaintype",
        "Fixed",
    );
    assert_keyword(
        &refused(&general(r#"dyntype="bogus""#)),
        "general",
        "dyntype",
        "bogus",
    );
    assert_keyword(
        &refused(&general(r#"interp="Linear" nsample="2""#)),
        "general",
        "interp",
        "Linear",
    );
}

#[test]
fn hillmuscle_and_millardmuscle_are_extensions() {
    // MuJoCo 3.5.0 refuses `hillmuscle` (measured); sim-mjcf implements both.
    let m = load(
        r#"<mujoco><worldbody><body><joint name="j" range="-1 1"/><geom size="0.1"/></body></worldbody>
        <actuator><general joint="j" dyntype="hillmuscle" gaintype="hillmuscle"
            biastype="hillmuscle" lengthrange="0.1 0.5"/></actuator></mujoco>"#,
    );
    assert_eq!(m.actuator_dyntype[0], ActuatorDynamics::HillMuscle);
    assert_eq!(m.actuator_gaintype[0], GainType::HillMuscle);
}

#[test]
fn a_sensors_interp_is_a_keyword() {
    // Before: refused by the builder, on <sensor>; now by the reader, on the
    // sensor's tag.
    let err = refused(
        r#"<mujoco><worldbody><body><joint name="j"/><geom size="0.1"/></body></worldbody>
        <sensor><jointpos joint="j" interp="Cubic" nsample="2"/></sensor></mujoco>"#,
    );
    assert_keyword(&err, "jointpos", "interp", "Cubic");
}

// ── flex and composite ──

fn flex(child: &str) -> String {
    format!(
        r#"<mujoco><worldbody>
            <body name="v0"><freejoint/><geom size="0.01"/></body>
            <body name="v1" pos="1 0 0"><freejoint/><geom size="0.01"/></body>
        </worldbody><deformable><flex name="f" dim="1">
            <vertex pos="0 0 0"/><vertex pos="1 0 0"/><element data="0 1"/>{child}
        </flex></deformable></mujoco>"#
    )
}

#[test]
fn an_unknown_selfcollide_is_refused() {
    // Before: any unknown word was `auto`.
    assert_keyword(
        &refused(&flex(r#"<contact selfcollide="bogus"/>"#)),
        "contact",
        "selfcollide",
        "bogus",
    );
}

#[test]
fn bending_model_is_an_exact_extension() {
    // Before: any word but `bridson` was `cotangent`.
    let err = refused(&flex(r#"<elasticity bending_model="Bridson"/>"#));
    assert_keyword(&err, "elasticity", "bending_model", "Bridson");
    let m = load(&flex(r#"<elasticity bending_model="bridson"/>"#));
    assert_eq!(m.flex_bending_type[0], FlexBendingType::Bridson);
}

#[test]
fn a_flexcomp_type_is_generated_or_refused() {
    let flexcomp = |ty: &str| {
        format!(
            r#"<mujoco><worldbody/><deformable>
                <flexcomp name="c" type="{ty}" count="2 2 1" spacing="0.1 0.1 0.1" dim="2"/>
            </deformable></mujoco>"#
        )
    };
    assert_keyword(&refused(&flexcomp("bogus")), "flexcomp", "type", "bogus");
    // A MuJoCo type sim-mjcf's form does not generate. Before: like an
    // unknown word, an empty flex, refused later for having no vertices.
    let err = refused(&flexcomp("cylinder"));
    assert!(
        matches!(&err, MjcfError::InvalidValue { element, attribute, .. }
            if element == "flexcomp" && attribute == "type"),
        "{err}"
    );
    assert!(err.to_string().contains("cylinder"), "{err}");
}

#[test]
fn a_composite_joint_kind_is_main() {
    // Before: a joint of another kind was dropped.
    let err = refused(
        r#"<mujoco><worldbody><composite type="cable" count="4 1 1">
            <joint kind="bogus"/><geom type="capsule" size="0.01"/>
        </composite></worldbody></mujoco>"#,
    );
    assert_keyword(&err, "joint", "kind", "bogus");
}

#[test]
fn a_composite_type_is_case_sensitive() {
    let err = refused(
        r#"<mujoco><worldbody><composite type="Cable" count="4 1 1">
            <geom type="capsule" size="0.01"/>
        </composite></worldbody></mujoco>"#,
    );
    assert_keyword(&err, "composite", "type", "Cable");
}

// ── booleans, `false|true|auto` and flags ──

fn hinge(joint_attrs: &str, class: &str) -> String {
    format!(
        r#"<mujoco><default>{class}</default><worldbody><body>
            <joint {joint_attrs}/><geom size="0.1"/>
        </body></worldbody></mujoco>"#
    )
}

#[test]
fn a_boolean_is_exactly_false_or_true() {
    // Before: any text but `true` was false.
    assert_keyword(
        &refused(&compiler(r#"autolimits="1""#)),
        "compiler",
        "autolimits",
        "1",
    );
    assert_keyword(
        &refused(&compiler(r#"fusestatic="TRUE""#)),
        "compiler",
        "fusestatic",
        "TRUE",
    );
    let err = refused(
        r#"<mujoco><worldbody><body mocap="1"><geom size="0.1"/></body></worldbody></mujoco>"#,
    );
    assert_keyword(&err, "body", "mocap", "1");
    let err = refused(
        r#"<mujoco><worldbody>
            <body name="a"><freejoint/><geom size="0.1"/></body>
            <body name="c" pos="1 0 0"><freejoint/><geom size="0.1"/></body>
        </worldbody><equality><weld body1="a" body2="c" active="1"/></equality></mujoco>"#,
    );
    assert_keyword(&err, "weld", "active", "1");
    let err = refused(
        r#"<mujoco><worldbody><body><joint name="j"/><geom size="0.1"/></body></worldbody>
        <actuator><general joint="j" dyntype="integrator" actearly="yes"/></actuator></mujoco>"#,
    );
    assert_keyword(&err, "general", "actearly", "yes");
}

#[test]
fn limited_is_exactly_false_true_or_auto() {
    // Before: any text but `true` was false.
    assert_keyword(
        &refused(&hinge(r#"limited="1" range="-1 1""#, "")),
        "joint",
        "limited",
        "1",
    );
    assert_keyword(
        &refused(&hinge(r#"limited="True" range="-1 1""#, "")),
        "joint",
        "limited",
        "True",
    );
    assert_keyword(
        &refused(&hinge(r#"limited="" range="-1 1""#, "")),
        "joint",
        "limited",
        "",
    );
}

#[test]
fn an_elements_auto_lies_over_its_class() {
    // MuJoCo 3.5.0, measured: `auto` over a class's `false`, with a range,
    // gives jnt_limited 1. Before: `auto` was read as `false`.
    let m = load(&hinge(
        r#"limited="auto" range="-1 1""#,
        r#"<joint limited="false"/>"#,
    ));
    assert!(m.jnt_limited[0]);
    let m = load(
        r#"<mujoco><worldbody><body><joint name="j" type="slide"/><geom size="0.1"/></body></worldbody>
        <tendon><fixed limited="auto" range="-1 1"><joint joint="j" coef="1"/></fixed></tendon>
        <actuator><general joint="j" ctrllimited="auto" ctrlrange="-2 2"/></actuator></mujoco>"#,
    );
    assert!(m.tendon_limited[0]);
    assert_eq!(m.actuator_ctrlrange[0], (-2.0, 2.0));
}

#[test]
fn a_flag_is_exactly_enable_or_disable() {
    // Before: any text but `disable` enabled the flag, so `off` kept gravity.
    let flag =
        |attrs: &str| format!("<mujoco><option><flag {attrs}/></option><worldbody/></mujoco>");
    assert_keyword(
        &refused(&flag(r#"gravity="off""#)),
        "flag",
        "gravity",
        "off",
    );
    assert_keyword(
        &refused(&flag(r#"gravity="true""#)),
        "flag",
        "gravity",
        "true",
    );
    assert_keyword(
        &refused(&flag(r#"energy="Enable""#)),
        "flag",
        "energy",
        "Enable",
    );
    let m = load(&flag(r#"gravity="disable" energy="enable""#));
    assert!(sim_core::disabled(&m, sim_core::DISABLE_GRAVITY));
    assert!(sim_core::enabled(&m, sim_core::ENABLE_ENERGY));
}

fn cable_with_joint(joint_attrs: &str) -> String {
    format!(
        r#"<mujoco><worldbody><composite type="cable" prefix="C" count="5 1 1" curve="s 0 0" size="1">
            <joint kind="main" {joint_attrs}/><geom type="capsule" size="0.01"/>
        </composite></worldbody></mujoco>"#
    )
}

#[test]
fn a_composite_joints_limited_is_read_from_the_joint() {
    // D-COMPOSITE-JOINT-LIMITED: MuJoCo 3.5.0 reads a composite joint's
    // `limited` from the <composite> element (xml_native_reader.cc, OneComposite),
    // whose schema does not hold it, so a written value is never read: measured,
    // `limited="false"` with a range gives jnt_limited 1 1 1 1, and
    // `limited="bogus"` loads. sim-mjcf reads it from the joint.
    let m = load(&cable_with_joint(r#"limited="false" range="0 0.5""#));
    assert_eq!(m.jnt_limited.len(), 4);
    assert!(m.jnt_limited.iter().all(|limited| !limited));
    let m = load(&cable_with_joint(r#"limited="true" range="0 0.5""#));
    assert!(m.jnt_limited.iter().all(|limited| *limited));
    assert_keyword(
        &refused(&cable_with_joint(r#"limited="bogus" range="0 0.5""#)),
        "joint",
        "limited",
        "bogus",
    );
}

// ── found untested by L07a's mutants ──

/// The reader refused it: the error is located by its line, not by the
/// element the builder was building.
fn assert_read_refusal(err: &MjcfError) {
    assert!(err.to_string().contains("(line "), "{err}");
}

#[test]
fn a_reader_refusal_is_located_by_its_line() {
    // The builder refuses these words too, for a model built in code; a
    // document's are refused when read.
    let err = refused(
        r#"<mujoco><worldbody><body sleep="Never"><freejoint/><geom size="0.1"/></body></worldbody></mujoco>"#,
    );
    assert_read_refusal(&err);
    let err = refused(
        r#"<mujoco><worldbody><body><joint name="j"/><geom size="0.1"/></body></worldbody>
        <actuator><general joint="j" gaintype="Fixed"/></actuator></mujoco>"#,
    );
    assert_read_refusal(&err);
}

#[test]
fn fluidshape_and_mesh_inertia_are_keywords_in_an_element_and_a_class() {
    let err = refused(
        r#"<mujoco><worldbody><body><geom size="0.1" fluidshape="Ellipsoid"/></body></worldbody></mujoco>"#,
    );
    assert_keyword(&err, "geom", "fluidshape", "Ellipsoid");
    let err =
        refused(r#"<mujoco><default><geom fluidshape="box"/></default><worldbody/></mujoco>"#);
    assert_keyword(&err, "geom", "fluidshape", "box");
    let err = refused(
        r#"<mujoco><asset><mesh name="m" inertia="Exact" vertex="0 0 0 1 0 0 0 1 0 0 0 1"/></asset>
        <worldbody/></mujoco>"#,
    );
    assert_keyword(&err, "mesh", "inertia", "Exact");
    let err =
        refused(r#"<mujoco><default><mesh inertia="volume"/></default><worldbody/></mujoco>"#);
    assert_keyword(&err, "mesh", "inertia", "volume");
}

#[test]
fn an_actuator_classs_gaintype_and_interp_are_keywords() {
    let err =
        refused(r#"<mujoco><default><general gaintype="Muscle"/></default><worldbody/></mujoco>"#);
    assert_keyword(&err, "general", "gaintype", "Muscle");
    let err =
        refused(r#"<mujoco><default><motor interp="spline"/></default><worldbody/></mujoco>"#);
    assert_keyword(&err, "motor", "interp", "spline");
}

#[test]
fn a_composite_geoms_type_is_a_keyword() {
    let err = refused(
        r#"<mujoco><worldbody><composite type="cable" count="4 1 1">
            <geom type="Capsule" size="0.01"/>
        </composite></worldbody></mujoco>"#,
    );
    assert_keyword(&err, "geom", "type", "Capsule");
}

#[test]
fn the_builder_refuses_a_sleep_or_selfcollide_word_built_in_code() {
    // Before: an unknown sleep policy was warned about and dropped, an
    // unknown selfcollide read as `auto`.
    let mut mjcf = sim_mjcf::parse_mjcf_str(
        r#"<mujoco><worldbody><body name="b"><freejoint/><geom size="0.1"/></body></worldbody></mujoco>"#,
    )
    .unwrap();
    mjcf.worldbody.children[0].sleep = Some("Never".to_owned());
    let err = sim_mjcf::model_from_mjcf(&mjcf, None).unwrap_err();
    assert_keyword(&err, "body", "sleep", "Never");
    assert!(err.to_string().ends_with("(body 'b')"), "{err}");

    let mut mjcf = sim_mjcf::parse_mjcf_str(&flex("")).unwrap();
    mjcf.flex[0].selfcollide = Some("bogus".to_owned());
    let err = sim_mjcf::model_from_mjcf(&mjcf, None).unwrap_err();
    assert_keyword(&err, "flex", "selfcollide", "bogus");
}

// ── found untested by L07b's mutants ──

#[test]
fn a_flag_left_unwritten_keeps_its_default() {
    // MuJoCo 3.5.0 (measured): `<flag gravity="disable"/>` alone sets no
    // enable flag.
    let m = load(r#"<mujoco><option><flag gravity="disable"/></option><worldbody/></mujoco>"#);
    assert_eq!(m.disableflags, sim_core::DISABLE_GRAVITY);
    assert_eq!(m.enableflags, 0);
}

#[test]
fn a_tendon_written_unlimited_needs_no_range() {
    // MuJoCo 3.5.0 (measured): `limited="false"` and `limited="auto"` without
    // a range load, unlimited.
    for limited in ["false", "auto"] {
        let m = load(&format!(
            r#"<mujoco><worldbody><body><joint name="j" type="slide"/><geom size="0.1"/></body></worldbody>
            <tendon><fixed limited="{limited}"><joint joint="j" coef="1"/></fixed></tendon></mujoco>"#
        ));
        assert!(!m.tendon_limited[0], "{limited}");
    }
}

#[test]
fn the_builder_refuses_a_site_type_built_in_code() {
    // A site's type is held to MuJoCo's geom type words in the builder too.
    // Before: any word the builder did not know was a sphere.
    let mut mjcf = sim_mjcf::parse_mjcf_str(
        r#"<mujoco><worldbody><body><geom size="0.1"/><site name="s"/></body></worldbody></mujoco>"#,
    )
    .unwrap();
    for word in ["bogus", "Box", ""] {
        mjcf.worldbody.children[0].sites[0].site_type = Some(word.to_owned());
        let err = sim_mjcf::model_from_mjcf(&mjcf, None).unwrap_err();
        assert_keyword(&err, "site", "type", word);
    }
    mjcf.worldbody.children[0].sites[0].site_type = Some("box".to_owned());
    assert!(sim_mjcf::model_from_mjcf(&mjcf, None).is_ok());
}

#[test]
fn a_ball_joint_written_limited_with_a_zero_range_is_limited() {
    // MuJoCo 3.5.0 refuses a range that does not increase on a hinge or slide
    // joint, not on a ball (`user_objects.cc`, `type != mjJNT_BALL`): a ball written
    // `limited="true"` with `range="0 0"` loads limited (measured). Found by
    // RL-2b's review: no test told it from reading `true` as "limited when the
    // range makes a limit".
    let m = load(&hinge(r#"type="ball" limited="true" range="0 0""#, ""));
    assert!(m.jnt_limited[0]);
}
