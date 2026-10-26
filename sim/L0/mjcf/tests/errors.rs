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
