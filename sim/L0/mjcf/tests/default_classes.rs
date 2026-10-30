//! Default classes made and named as MuJoCo 3.5.0 makes and names them (book
//! L16): every top-level `<default>` is the one main class, which `class` and
//! `childclass` may name `main`; a nested class copies its parent as the
//! parent stands when the class is made; and a class name that is empty,
//! repeated or unknown is refused. Each value or refusal is MuJoCo 3.5.0's
//! for the same document (measured).

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use approx::assert_relative_eq;
use sim_mjcf::{ElementKind, MjcfError, load_model};

/// The one joint's damping and armature.
fn joint_values(xml: &str) -> (f64, f64) {
    let model = load_model(xml).unwrap();
    (model.dof_damping[0], model.dof_armature[0])
}

fn refused(xml: &str) -> MjcfError {
    load_model(xml).unwrap_err()
}

fn assert_unknown_class(err: &MjcfError, name: &str) {
    assert!(
        matches!(err, MjcfError::UnknownClass { name: n, .. } if n == name),
        "{err}"
    );
}

#[test]
fn a_top_level_class_named_main_is_the_main_class() {
    let xml = r#"<mujoco><default class="main"><joint damping="2"/></default><worldbody><body><joint name="j" type="hinge"/><geom size=".1"/></body></worldbody></mujoco>"#;
    assert_eq!(joint_values(xml), (2.0, 0.0));
}

#[test]
fn a_top_level_class_named_main_gives_a_site_its_values() {
    let model = load_model(
        r#"<mujoco><default class="main"><site type="box" size=".1 .2 .3"/></default><worldbody><body><geom size=".1"/><site name="s"/></body></worldbody></mujoco>"#,
    )
    .unwrap();
    assert_relative_eq!(model.site_size[0].x, 0.1);
    assert_relative_eq!(model.site_size[0].y, 0.2);
    assert_relative_eq!(model.site_size[0].z, 0.3);
}

/// Inside a `childclass="a"` body, naming `main` takes the main class
/// (damping 2), not the body's class (7).
#[test]
fn an_element_may_name_the_main_class() {
    let xml = r#"<mujoco><default><joint damping="2"/><geom contype="2"/><site type="box" size=".03 .04 .05"/><default class="a"><joint damping="7"/><geom contype="4"/><site type="sphere" size=".2"/></default></default><worldbody><body childclass="a"><joint name="j" type="hinge" class="main"/><geom size=".1"/></body></worldbody></mujoco>"#;
    assert_eq!(joint_values(xml), (2.0, 0.0));
}

#[test]
fn a_site_may_name_the_main_class() {
    let model = load_model(
        r#"<mujoco><default><joint damping="2"/><geom contype="2"/><site type="box" size=".03 .04 .05"/><default class="a"><joint damping="7"/><geom contype="4"/><site type="sphere" size=".2"/></default></default><worldbody><body childclass="a"><geom size=".1"/><site name="s" class="main"/></body></worldbody></mujoco>"#,
    )
    .unwrap();
    assert_relative_eq!(model.site_size[0].x, 0.03);
    assert_relative_eq!(model.site_size[0].z, 0.05);
}

#[test]
fn a_body_childclass_may_name_the_main_class() {
    let model = load_model(
        r#"<mujoco><default><joint damping="2"/><geom contype="2"/><site type="box" size=".03 .04 .05"/><default class="a"><joint damping="7"/><geom contype="4"/><site type="sphere" size=".2"/></default></default><worldbody><body childclass="a"><geom size=".1"/><body childclass="main"><joint name="j" type="hinge"/><geom size=".1"/></body></body></worldbody></mujoco>"#,
    )
    .unwrap();
    assert_relative_eq!(model.dof_damping[0], 2.0);
    assert_eq!(model.geom_contype, vec![4, 2]);
}

#[test]
fn a_frame_childclass_may_name_the_main_class() {
    let model = load_model(
        r#"<mujoco><default><joint damping="2"/><geom contype="2"/><site type="box" size=".03 .04 .05"/><default class="a"><joint damping="7"/><geom contype="4"/><site type="sphere" size=".2"/></default></default><worldbody><body childclass="a"><frame childclass="main"><geom size=".1"/></frame></body></worldbody></mujoco>"#,
    )
    .unwrap();
    assert_eq!(model.geom_contype, vec![2]);
}

/// Two top-level blocks are one main class, read in document order.
#[test]
fn top_level_blocks_merge_onto_the_main_class() {
    let xml = r#"<mujoco><default><joint damping="2"/></default><default><joint armature=".5"/></default><worldbody><body><joint name="j" type="hinge"/><geom size=".1"/></body></worldbody></mujoco>"#;
    assert_eq!(joint_values(xml), (2.0, 0.5));
}

#[test]
fn a_top_level_block_named_main_twice_merges() {
    let xml = r#"<mujoco><default class="main"><joint damping="2"/></default><default class="main"><joint armature="1"/></default><worldbody><body><joint name="j" type="hinge"/><geom size=".1"/></body></worldbody></mujoco>"#;
    assert_eq!(joint_values(xml), (2.0, 1.0));
}

#[test]
fn a_class_in_a_top_level_block_named_main_takes_its_values() {
    let xml = r#"<mujoco><default class="main"><joint damping="2"/><default class="a"><joint armature="1"/></default></default><worldbody><body><joint name="j" type="hinge" class="a"/><geom size=".1"/></body></worldbody></mujoco>"#;
    assert_eq!(joint_values(xml), (2.0, 1.0));
}

/// A class copies the main class as it stood when the class was made, so a
/// later top-level block does not reach it.
#[test]
fn a_class_does_not_see_a_later_top_level_block() {
    let xml = r#"<mujoco><default><default class="a"><joint damping="1"/></default></default><default><joint armature="2"/></default><worldbody><body><joint name="j" type="hinge" class="a"/><geom size=".1"/></body></worldbody></mujoco>"#;
    assert_eq!(joint_values(xml), (1.0, 0.0));
}

/// A block's own elements are read before its nested classes, wherever they
/// are written in it.
#[test]
fn a_class_sees_its_parents_elements_written_after_it() {
    let xml = r#"<mujoco><default><default class="a"><joint damping="1"/></default><joint armature="2"/></default><worldbody><body><joint name="j" type="hinge" class="a"/><geom size=".1"/></body></worldbody></mujoco>"#;
    assert_eq!(joint_values(xml), (1.0, 2.0));
}

#[test]
fn a_top_level_class_written_empty_is_the_main_class() {
    let xml = r#"<mujoco><default class=""><joint damping="2"/></default><worldbody><body><joint name="j" type="hinge"/><geom size=".1"/></body></worldbody></mujoco>"#;
    assert_eq!(joint_values(xml), (2.0, 0.0));
}

#[test]
fn an_unknown_class_is_refused() {
    let err = refused(
        r#"<mujoco><default><joint damping="2"/></default><worldbody><body><joint name="j" type="hinge" class="nope"/><geom size=".1"/></body></worldbody></mujoco>"#,
    );
    assert_unknown_class(&err, "nope");
}

#[test]
fn an_empty_class_name_on_an_element_is_refused() {
    let err = refused(
        r#"<mujoco><default><joint damping="2"/></default><worldbody><body><joint name="j" type="hinge" class=""/><geom size=".1"/></body></worldbody></mujoco>"#,
    );
    assert_unknown_class(&err, "");
}

#[test]
fn an_unknown_body_childclass_is_refused() {
    let err = refused(
        r#"<mujoco><worldbody><body childclass="nope"><joint name="j" type="hinge"/><geom size=".1"/></body></worldbody></mujoco>"#,
    );
    assert_unknown_class(&err, "nope");
}

#[test]
fn a_renamed_top_level_class_is_refused() {
    let err = refused(
        r#"<mujoco><default class="foo"><joint damping="2"/></default><worldbody><body><joint name="j" type="hinge"/><geom size=".1"/></body></worldbody></mujoco>"#,
    );
    assert!(
        matches!(&err, MjcfError::RenamedRootClass { name, .. } if name == "foo"),
        "{err}"
    );
}

#[test]
fn a_repeated_class_name_is_refused() {
    let err = refused(
        r#"<mujoco><default><default class="a"/><default class="a"/></default><worldbody><body><joint name="j" type="hinge"/><geom size=".1"/></body></worldbody></mujoco>"#,
    );
    assert!(
        matches!(&err, MjcfError::DuplicateName { kind: ElementKind::DefaultClass, name, .. } if name == "a"),
        "{err}"
    );
}

#[test]
fn a_nested_class_named_main_is_refused() {
    let err = refused(
        r#"<mujoco><default><default class="main"/></default><worldbody><body><joint name="j" type="hinge"/><geom size=".1"/></body></worldbody></mujoco>"#,
    );
    assert!(
        matches!(&err, MjcfError::DuplicateName { kind: ElementKind::DefaultClass, name, .. } if name == "main"),
        "{err}"
    );
}

/// Ledger L26's second shape: before this commit's resolver it walked
/// `a`→`a` without end, growing memory (established by reading; not run on
/// that code).
#[test]
fn a_class_nested_in_a_class_of_its_name_is_refused() {
    let err = refused(
        r#"<mujoco><default><default class="a"><default class="a"/></default></default><worldbody><body><joint name="j" type="hinge"/><geom size=".1"/></body></worldbody></mujoco>"#,
    );
    assert!(
        matches!(&err, MjcfError::DuplicateName { kind: ElementKind::DefaultClass, name, .. } if name == "a"),
        "{err}"
    );
}

/// Ledger L26's first shape.
#[test]
fn a_nested_class_without_a_name_is_refused() {
    let err = refused(
        r#"<mujoco><default><joint/><default><joint/></default></default><worldbody><body><joint name="j" type="hinge"/><geom size=".1"/></body></worldbody></mujoco>"#,
    );
    assert!(matches!(&err, MjcfError::EmptyClassName { .. }), "{err}");
}

#[test]
fn a_nested_class_written_empty_is_refused() {
    let err = refused(
        r#"<mujoco><default><default class=""><joint/></default></default><worldbody><body><joint name="j" type="hinge"/><geom size=".1"/></body></worldbody></mujoco>"#,
    );
    assert!(matches!(&err, MjcfError::EmptyClassName { .. }), "{err}");
}

/// MuJoCo 3.5.0's schema takes each element at most once in a class
/// ("unique element 'joint' found 2 times").
#[test]
fn an_element_written_twice_in_a_class_is_refused() {
    let err = refused(
        r#"<mujoco><default><joint damping="2"/><joint armature=".5"/></default><worldbody><body><joint name="j" type="hinge"/><geom size=".1"/></body></worldbody></mujoco>"#,
    );
    assert!(
        matches!(&err, MjcfError::RepeatedElement { element, parent, .. } if element == "joint" && parent == "default"),
        "{err}"
    );
}

#[test]
fn an_unknown_site_class_is_refused() {
    let err = refused(
        r#"<mujoco><worldbody><body><geom size=".1"/><site name="s" class="nope"/></body></worldbody></mujoco>"#,
    );
    assert_unknown_class(&err, "nope");
}

#[test]
fn an_unknown_geom_class_in_a_frame_is_refused() {
    let err = refused(
        r#"<mujoco><worldbody><body><frame pos="1 0 0"><geom class="nope" size=".1"/></frame></body></worldbody></mujoco>"#,
    );
    assert_unknown_class(&err, "nope");
}

#[test]
fn an_unknown_tendon_class_is_refused() {
    let err = refused(
        r#"<mujoco><worldbody><body><joint name="j" type="hinge"/><geom size=".1"/></body></worldbody><tendon><fixed class="nope"><joint joint="j" coef="1"/></fixed></tendon></mujoco>"#,
    );
    assert_unknown_class(&err, "nope");
}

#[test]
fn an_unknown_actuator_class_is_refused() {
    let err = refused(
        r#"<mujoco><worldbody><body><joint name="j" type="hinge"/><geom size=".1"/></body></worldbody><actuator><motor joint="j" class="nope"/></actuator></mujoco>"#,
    );
    assert_unknown_class(&err, "nope");
}

#[test]
fn an_unknown_equality_class_is_refused() {
    let err = refused(
        r#"<mujoco><worldbody><body><joint name="j" type="hinge"/><geom size=".1"/></body></worldbody><equality><joint joint1="j" class="nope"/></equality></mujoco>"#,
    );
    assert_unknown_class(&err, "nope");
}

#[test]
fn an_unknown_pair_class_is_refused() {
    let err = refused(
        r#"<mujoco><worldbody><geom name="a" size=".1"/><geom name="b" size=".1" pos="1 0 0"/></worldbody><contact><pair geom1="a" geom2="b" class="nope"/></contact></mujoco>"#,
    );
    assert_unknown_class(&err, "nope");
}

/// A sensor's `class` is sim-mjcf's own (MuJoCo 3.5.0 refuses the attribute),
/// checked as every other class name is.
#[test]
fn an_unknown_sensor_class_is_refused() {
    let err = refused(
        r#"<mujoco><worldbody><body><joint name="j" type="hinge"/><geom size=".1"/></body></worldbody><sensor><jointpos joint="j" class="nope"/></sensor></mujoco>"#,
    );
    assert_unknown_class(&err, "nope");
}

/// An equality's class is checked when the equality is read, before the
/// bodies, joints or tendons it names, as MuJoCo 3.5.0's `GetClass` comes
/// first: each names an unknown class and a missing element.
#[test]
fn an_equalitys_class_is_checked_before_what_it_names() {
    for xml in [
        r#"<mujoco><worldbody><body name="b1"><joint name="j" type="hinge"/><geom size=".1"/></body></worldbody><equality><connect class="nope" body1="missing" anchor="0 0 0"/></equality></mujoco>"#,
        r#"<mujoco><worldbody><body name="b1"><joint name="j" type="hinge"/><geom size=".1"/></body></worldbody><equality><weld class="nope" body1="missing"/></equality></mujoco>"#,
        r#"<mujoco><worldbody><body name="b1"><joint name="j" type="hinge"/><geom size=".1"/></body></worldbody><equality><joint class="nope" joint1="missing"/></equality></mujoco>"#,
        r#"<mujoco><worldbody><body name="b1"><joint name="j" type="hinge"/><geom size=".1"/></body></worldbody><equality><tendon class="nope" tendon1="missing"/></equality></mujoco>"#,
    ] {
        assert_unknown_class(&refused(xml), "nope");
    }
}

/// A geom's unknown class is refused on the world body's own geoms and a
/// body's, outside any frame.
#[test]
fn an_unknown_geom_class_is_refused() {
    for xml in [
        r#"<mujoco><worldbody><geom class="nope" size=".1"/></worldbody></mujoco>"#,
        r#"<mujoco><worldbody><body><geom class="nope" size=".1"/></body></worldbody></mujoco>"#,
    ] {
        assert_unknown_class(&refused(xml), "nope");
    }
}

/// A body's unknown `childclass` is refused where it is written, though no
/// element takes it (MuJoCo: "unknown default childclass").
#[test]
fn an_unknown_childclass_on_a_body_without_elements_is_refused() {
    assert_unknown_class(
        &refused(r#"<mujoco><worldbody><body childclass="nope"/></worldbody></mujoco>"#),
        "nope",
    );
}

/// Each equality kind's unknown class is refused. The distance equality is
/// sim-mjcf's own (MuJoCo 3.5.0's schema refuses the element).
#[test]
fn every_equality_kinds_unknown_class_is_refused() {
    for xml in [
        r#"<mujoco><worldbody><body name="b1"><joint name="j" type="hinge"/><geom size=".1"/></body><body name="b2" pos="1 0 0"><joint name="j2" type="hinge"/><geom size=".1"/></body></worldbody><equality><connect class="nope" body1="b1" body2="b2" anchor="0 0 0"/></equality></mujoco>"#,
        r#"<mujoco><worldbody><body name="b1"><joint name="j" type="hinge"/><geom size=".1"/></body><body name="b2" pos="1 0 0"><joint name="j2" type="hinge"/><geom size=".1"/></body></worldbody><equality><weld class="nope" body1="b1" body2="b2"/></equality></mujoco>"#,
        r#"<mujoco><worldbody><body name="b1"><joint name="j" type="hinge"/><geom size=".1"/></body><body name="b2" pos="1 0 0"><joint name="j2" type="hinge"/><geom size=".1"/></body></worldbody><tendon><fixed name="t1"><joint joint="j" coef="1"/></fixed><fixed name="t2"><joint joint="j2" coef="1"/></fixed></tendon><equality><tendon class="nope" tendon1="t1" tendon2="t2"/></equality></mujoco>"#,
        r#"<mujoco><worldbody><body name="b1"><joint name="j" type="hinge"/><geom name="g1" size=".1"/></body><body name="b2" pos="1 0 0"><joint name="j2" type="hinge"/><geom name="g2" size=".1"/></body></worldbody><equality><distance class="nope" geom1="g1" geom2="g2"/></equality></mujoco>"#,
    ] {
        assert_unknown_class(&refused(xml), "nope");
    }
}
