//! A flex's edge equality, as MuJoCo 3.5.0 makes one: an `<equality><flex>`
//! naming the flex, or a `<flexcomp>`'s `<edge equality="true">`, and none
//! otherwise.
//!
//! Each expected value was measured on MuJoCo 3.5.0 for the same model: the
//! `<flex>` documents as written here, each vertex a body of its own; the
//! `<flexcomp>` ones in MuJoCo's form, the flexcomp in a body, which MuJoCo
//! reads where this crate reads it in `<deformable>`.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::fmt::Write as _;

use sim_core::EqualityType;
use sim_mjcf::{ElementKind, MjcfError, load_model};

/// MuJoCo's default `solref` and `solimp` (`mj_defaultSolRefImp`).
const SOLREF: [f64; 2] = [0.02, 1.0];
const SOLIMP: [f64; 5] = [0.9, 0.95, 0.001, 0.5, 2.0];

/// The bodies `v0`…`v3` at the corners of a unit square at height `z`, each
/// with three slides.
fn square_bodies(z: u32) -> String {
    let slides = r#"<joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/>"#;
    let mut bodies = String::new();
    for i in 0..4 {
        write!(
            bodies,
            r#"<body name="v{i}" pos="{} {} {z}">{slides}<inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body>"#,
            i % 2,
            i / 2
        )
        .unwrap();
    }
    bodies
}

/// A square flex `f` over [`square_bodies`] at height 0, `default` in
/// `<default>` and `equality` in `<equality>`.
fn square(default: &str, equality: &str) -> String {
    let bodies = square_bodies(0);
    format!(
        r#"<mujoco>
            <default>{default}</default>
            <worldbody>{bodies}</worldbody>
            <deformable>
                <flex name="f" dim="2" body="v0 v1 v2 v3" element="0 1 2  1 3 2">
                    <contact contype="0" conaffinity="0"/>
                </flex>
            </deformable>
            <equality>{equality}</equality>
        </mujoco>"#
    )
}

/// A 2×2 cloth flexcomp `c` with `children` (its `<edge>` and `<pin>`),
/// `default` in `<default>` and `equality` in `<equality>`.
fn cloth(children: &str, default: &str, equality: &str) -> String {
    format!(
        r#"<mujoco>
            <default>{default}</default>
            <deformable>
                <flexcomp name="c" type="grid" count="2 2 1" spacing="0.1 0.1 0.1" dim="2" mass="1">
                    {children}
                    <contact contype="0" conaffinity="0"/>
                </flexcomp>
            </deformable>
            <equality>{equality}</equality>
        </mujoco>"#
    )
}

#[test]
fn a_flex_no_equality_names_has_none() {
    let model = load_model(&square("", "")).unwrap();
    assert_eq!(model.neq, 0);
    assert_eq!(model.flex_edgeequality, vec![false]);
}

#[test]
fn an_equality_flex_names_its_flex() {
    let model = load_model(&square("", r#"<flex flex="f"/>"#)).unwrap();
    assert_eq!(model.eq_type, vec![EqualityType::Flex]);
    assert_eq!(model.eq_obj1id, vec![0]);
    assert_eq!(model.eq_obj2id, vec![usize::MAX]);
    assert_eq!(model.eq_solref, vec![SOLREF]);
    assert_eq!(model.eq_solimp, vec![SOLIMP]);
    assert_eq!(model.eq_active, vec![true]);
    assert_eq!(model.flex_edgeequality, vec![true]);
}

#[test]
fn an_equality_flex_takes_its_class_and_its_written_values() {
    // MuJoCo: the main class's 0.05 and 0.8, a class's 0.04, and a written
    // 0.03 with `active="false"`, which leaves `flex_edgeequality` set.
    let main = r#"<equality solref="0.05 1" solimp="0.8 0.9 0.01 0.5 2"/>"#;
    let model = load_model(&square(main, r#"<flex flex="f"/>"#)).unwrap();
    assert_eq!(model.eq_solref, vec![[0.05, 1.0]]);
    assert_eq!(model.eq_solimp, vec![[0.8, 0.9, 0.01, 0.5, 2.0]]);

    let class = r#"<default class="c"><equality solref="0.04 1"/></default>"#;
    let model = load_model(&square(class, r#"<flex flex="f" class="c"/>"#)).unwrap();
    assert_eq!(model.eq_solref, vec![[0.04, 1.0]]);

    let written = r#"<flex name="e" flex="f" solref="0.03 1" active="false"/>"#;
    let model = load_model(&square("", written)).unwrap();
    assert_eq!(model.eq_solref, vec![[0.03, 1.0]]);
    assert_eq!(model.eq_active, vec![false]);
    assert_eq!(model.eq_name, vec![Some("e".to_string())]);
    assert_eq!(model.flex_edgeequality, vec![true]);
}

#[test]
fn two_equalities_on_one_flex_both_load() {
    let model = load_model(&square(
        "",
        r#"<flex flex="f"/><flex flex="f" solref="0.03 1"/>"#,
    ))
    .unwrap();
    assert_eq!(model.eq_type, vec![EqualityType::Flex; 2]);
    assert_eq!(model.eq_obj1id, vec![0, 0]);
    assert_eq!(model.eq_solref, vec![SOLREF, [0.03, 1.0]]);
}

#[test]
fn an_equality_naming_no_flex_is_refused() {
    // MuJoCo: "unknown element 'nope' in equality constraint".
    let err = load_model(&square("", r#"<flex flex="nope"/>"#)).unwrap_err();
    assert!(
        matches!(&err, MjcfError::UndefinedReference { kind: ElementKind::Flex, name, .. } if name == "nope"),
        "{err}"
    );
}

#[test]
fn an_equality_flex_without_its_flex_is_refused() {
    // MuJoCo: "required attribute missing: 'flex'".
    let err = load_model(&square("", "<flex/>")).unwrap_err();
    assert!(
        matches!(&err, MjcfError::MissingAttribute { element, attribute, .. } if element == "flex" && attribute == "flex"),
        "{err}"
    );
}

#[test]
fn a_flexvert_equality_is_refused() {
    // MuJoCo loads it (an `mjEQ_FLEXVERT`, two rows per vertex), which
    // sim-core does not have.
    let err = load_model(&square("", r#"<flexvert flex="f"/>"#)).unwrap_err();
    assert!(matches!(&err, MjcfError::Unsupported { .. }), "{err}");
}

#[test]
fn a_vertex_equality_is_refused_after_mujocos_checks() {
    // MuJoCo checks a vertex equality's flex as a flex equality's:
    // "unknown element 'nope' in equality constraint", and "rigid flex 'c'
    // in equality constraint 0" for a flexcomp every vertex of which is
    // pinned.
    let err = load_model(&square("", r#"<flexvert flex="nope"/>"#)).unwrap_err();
    assert!(
        matches!(&err, MjcfError::UndefinedReference { kind: ElementKind::Flex, name, .. } if name == "nope"),
        "{err}"
    );
    let err = load_model(&cloth(
        r#"<pin id="0 1 2 3"/>"#,
        "",
        r#"<flexvert flex="c"/>"#,
    ))
    .unwrap_err();
    assert!(
        matches!(&err, MjcfError::InvalidModel { reason, .. } if reason.contains("rigid flex 'c'")),
        "{err}"
    );
    // A rigid flexcomp's `<edge equality="vert">` makes none, and loads, as
    // MuJoCo's; with one pin it makes one (MuJoCo's `mjEQ_FLEXVERT`).
    let model = load_model(&cloth(
        r#"<edge equality="vert"/><pin id="0 1 2 3"/>"#,
        "",
        "",
    ))
    .unwrap();
    assert_eq!(model.neq, 0);
    let err = load_model(&cloth(r#"<edge equality="vert"/><pin id="0"/>"#, "", "")).unwrap_err();
    assert!(matches!(&err, MjcfError::Unsupported { .. }), "{err}");
}

#[test]
fn an_empty_flex_name_beside_an_unnamed_flex_is_refused() {
    // MuJoCo binds `flex=""` to the unnamed flex (neq 1, five rows); this
    // crate refuses it (registry `D-FLEX-EQ-NO-NAME`). With no unnamed flex
    // MuJoCo refuses it too: "unknown element '' in equality constraint".
    let named = square("", r#"<flex flex=""/>"#);
    assert_eq!(named.matches(r#"<flex name="f" "#).count(), 1);
    let unnamed = named.replace(r#"<flex name="f" "#, "<flex ");
    for xml in [unnamed, named] {
        let err = load_model(&xml).unwrap_err();
        assert!(
            matches!(&err, MjcfError::UndefinedReference { kind: ElementKind::Flex, name, .. } if name.is_empty()),
            "{err}"
        );
    }
}

#[test]
fn a_flexcomps_edge_equality_has_mujocos_own_parameters() {
    // MuJoCo: one unnamed active equality on the flexcomp, MuJoCo's default
    // `solref` and `solimp` under a main class that writes others, or the
    // `<edge>`'s own.
    let main = r#"<equality solref="0.05 1" solimp="0.8 0.9 0.01 0.5 2"/>"#;
    let model = load_model(&cloth(r#"<edge equality="true"/>"#, main, "")).unwrap();
    assert_eq!(model.eq_type, vec![EqualityType::Flex]);
    assert_eq!(model.eq_obj1id, vec![0]);
    assert_eq!(model.eq_solref, vec![SOLREF]);
    assert_eq!(model.eq_solimp, vec![SOLIMP]);
    assert_eq!(model.eq_active, vec![true]);
    assert_eq!(model.eq_name, vec![None]);

    let edge = r#"<edge equality="true" solref="0.03 1" solimp="0.7 0.8 0.002 0.5 2"/>"#;
    let model = load_model(&cloth(edge, "", "")).unwrap();
    assert_eq!(model.eq_solref, vec![[0.03, 1.0]]);
    assert_eq!(model.eq_solimp, vec![[0.7, 0.8, 0.002, 0.5, 2.0]]);

    for children in [r#"<edge equality="false"/>"#, ""] {
        let model = load_model(&cloth(children, "", "")).unwrap();
        assert_eq!(model.neq, 0, "{children}");
    }
}

#[test]
fn a_flexcomps_vertex_equality_and_an_unknown_keyword_are_refused() {
    let err = load_model(&cloth(r#"<edge equality="vert"/>"#, "", "")).unwrap_err();
    assert!(matches!(&err, MjcfError::Unsupported { .. }), "{err}");
    // MuJoCo: "invalid keyword: 'yes'".
    let err = load_model(&cloth(r#"<edge equality="yes"/>"#, "", "")).unwrap_err();
    assert!(matches!(&err, MjcfError::InvalidKeyword { .. }), "{err}");
    // MuJoCo reads the `<edge>`'s `solref` whatever its `equality`: "bad
    // format in attribute 'solref'" beside `false` too.
    for equality in ["false", "true"] {
        let edge = format!(r#"<edge equality="{equality}" solref="0.02 x"/>"#);
        assert!(load_model(&cloth(&edge, "", "")).is_err(), "{equality}");
    }
}

#[test]
fn a_flexcomps_equality_follows_the_documents() {
    // MuJoCo makes a flexcomp's equality while it reads `<worldbody>`, after
    // `<equality>`: `f`'s equality, then `c`'s. The flex ids are this crate's:
    // it numbers the flexes as written, `c` 0 and `f` 1, where MuJoCo, reading
    // `<deformable>` before the flexcomp in `<worldbody>`, numbers `f` 0 and
    // `c` 1 (its `eq_obj1id` `[0, 1]`).
    let bodies = square_bodies(1);
    let xml = format!(
        r#"<mujoco>
            <worldbody>{bodies}</worldbody>
            <deformable>
                <flexcomp name="c" type="grid" count="2 2 1" spacing="0.1 0.1 0.1" dim="2" mass="1">
                    <edge equality="true"/>
                    <contact contype="0" conaffinity="0"/>
                </flexcomp>
                <flex name="f" dim="2" body="v0 v1 v2 v3" element="0 1 2  1 3 2">
                    <contact contype="0" conaffinity="0"/>
                </flex>
            </deformable>
            <equality><flex flex="f"/></equality>
        </mujoco>"#
    );
    let model = load_model(&xml).unwrap();
    assert_eq!(model.eq_obj1id, vec![1, 0]);
}

#[test]
fn a_rigid_flexcomp_makes_no_equality_and_one_named_is_refused() {
    // MuJoCo: every vertex pinned makes the flexcomp rigid, and `Make`
    // returns before it makes the `<edge>`'s equality; one naming it is
    // refused, "rigid flex 'c' in equality constraint 0".
    let pinned = r#"<edge equality="true"/><pin id="0 1 2 3"/>"#;
    let model = load_model(&cloth(pinned, "", "")).unwrap();
    assert_eq!(model.neq, 0);

    let err = load_model(&cloth(r#"<pin id="0 1 2 3"/>"#, "", r#"<flex flex="c"/>"#)).unwrap_err();
    assert!(
        matches!(&err, MjcfError::InvalidModel { reason, .. } if reason.contains("rigid flex 'c'")),
        "{err}"
    );

    // One pin: not rigid.
    let model = load_model(&cloth(r#"<pin id="0"/>"#, "", r#"<flex flex="c"/>"#)).unwrap();
    assert_eq!(model.neq, 1);
}
