//! A `<flex>` whose vertices are the bodies it names, as MuJoCo 3.5.0's
//! (`mjCFlex::Compile`, `user_mesh.cc:4103-4213`), and the subset of it that
//! sim-core's flex code takes.
//!
//! Each vertex is the body `body` names, at that body's origin. sim-core reads
//! a vertex's three dofs as its world x, y and z and a vertex without dofs as
//! fixed, so a vertex body must be jointless or have exactly three slides along
//! x, y and z, unrotated, with no dof above it; anything else MuJoCo loads is
//! refused as a stated limitation. The arrays below were measured on MuJoCo
//! 3.5.0 for the same documents (edges, `flexedge_rigid`, `flexedge_length0`).

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use sim_mjcf::{ElementKind, MjcfError, load_model};

/// A body named `name` at `pos`, with three slides along x, y and z, or none.
fn body(name: &str, pos: &str, slides: bool) -> String {
    let joints = if slides {
        r#"<joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/>"#
    } else {
        ""
    };
    format!(
        r#"<body name="{name}" pos="{pos}">{joints}<inertial pos="0 0 0" mass="0.5" diaginertia="1e-15 1e-15 1e-15"/></body>"#
    )
}

fn doc(bodies: &str, flex: &str) -> String {
    format!("<mujoco><worldbody>{bodies}</worldbody><deformable>{flex}</deformable></mujoco>")
}

/// Three bodies with three slides each, at the corners of a triangle.
fn triangle_bodies() -> String {
    [
        body("a", "0 0 1", true),
        body("b", "0.1 0 1", true),
        body("c", "0 0.1 1", true),
    ]
    .concat()
}

const TRIANGLE: &str = r#"<flex name="f" dim="2" body="a b c" element="0 1 2"></flex>"#;

fn refusal(xml: &str) -> MjcfError {
    load_model(xml).expect_err("refused")
}

fn assert_unsupported(xml: &str, words: &str) {
    match refusal(xml) {
        MjcfError::Unsupported { feature, .. } => {
            assert!(feature.contains(words), "{feature:?} lacks {words:?}");
        }
        other => panic!("not a stated limitation: {other}"),
    }
}

#[test]
fn flex_body_names_existing_bodies() {
    let model = load_model(&doc(&triangle_bodies(), TRIANGLE)).unwrap();
    assert_eq!((model.nbody, model.nflexvert), (4, 3));
    assert_eq!(model.flexvert_bodyid, [1, 2, 3]);
    assert_eq!(model.flexvert_dofadr, [0, 3, 6]);
    assert_eq!(model.flexvert_qposadr, [0, 3, 6]);
    assert_eq!(model.flexvert_mass, [0.5; 3]);
    assert_eq!(model.flexvert_invmass, [2.0; 3]);
    assert_eq!(model.flexedge_vert, [[1, 2], [0, 2], [0, 1]]);
    assert_eq!(model.flexedge_rigid, [false; 3]);
    assert_eq!(model.flex_rigid, [false]);
    let len0 = &model.flexedge_length0;
    assert!((len0[0] - 0.02_f64.sqrt()).abs() < 1e-15, "{len0:?}");
    assert!(
        (len0[1] - 0.1).abs() < 1e-15 && (len0[2] - 0.1).abs() < 1e-15,
        "{len0:?}"
    );
    // The loader makes no body, joint or dof of its own.
    assert_eq!((model.njnt, model.nv), (9, 9));
}

/// A zero `vertex` for each body is the same flex (MuJoCo's centered flex).
#[test]
fn a_zero_vertex_offset_is_the_bodys_origin() {
    let flex =
        r#"<flex name="f" dim="2" body="a b c" vertex="0 0 0 0 0 0 0 0 0" element="0 1 2"></flex>"#;
    let with = load_model(&doc(&triangle_bodies(), flex)).unwrap();
    let without = load_model(&doc(&triangle_bodies(), TRIANGLE)).unwrap();
    assert_eq!(with.flexedge_length0, without.flexedge_length0);
    assert_eq!(with.flexvert_bodyid, without.flexvert_bodyid);
}

/// A jointless vertex body is fixed: no dofs, no inverse mass, and an edge
/// between two of them (both welded to the world) is rigid, as MuJoCo's
/// weld rule makes it; the flex is not rigid (its vertices are three bodies).
#[test]
fn jointless_vertex_bodies_are_fixed_and_their_edge_rigid() {
    let bodies = [
        body("a", "0 0 1", false),
        body("b", "0.1 0 1", false),
        body("c", "0 0.1 1", true),
    ]
    .concat();
    let model = load_model(&doc(&bodies, TRIANGLE)).unwrap();
    assert_eq!(model.flexvert_dofadr, [usize::MAX, usize::MAX, 0]);
    assert_eq!(model.flexvert_invmass, [0.0, 0.0, 2.0]);
    assert_eq!(model.flexedge_vert, [[1, 2], [0, 2], [0, 1]]);
    assert_eq!(model.flexedge_rigid, [false, false, true]);
    assert_eq!(model.flex_rigid, [false]);
}

#[test]
fn flex_unknown_vertex_body_is_an_error() {
    let xml = doc(
        &triangle_bodies(),
        r#"<flex name="f" dim="2" body="a b nope" element="0 1 2"></flex>"#,
    );
    match refusal(&xml) {
        MjcfError::UndefinedReference {
            kind: ElementKind::Body,
            name,
            ..
        } => assert_eq!(name, "nope"),
        other => panic!("{other}"),
    }
}

#[test]
fn flex_without_body_or_element_is_refused() {
    for (flex, missing) in [
        (r#"<flex name="f" dim="2" element="0 1 2"></flex>"#, "body"),
        (r#"<flex name="f" dim="2" body="a b c"></flex>"#, "element"),
        // MuJoCo 3.5.0 reads an empty `element` as missing (measured).
        (
            r#"<flex name="f" dim="2" body="a b c" element=""></flex>"#,
            "element",
        ),
    ] {
        match refusal(&doc(&triangle_bodies(), flex)) {
            MjcfError::MissingAttribute {
                element, attribute, ..
            } => {
                assert_eq!(
                    (element.as_str(), attribute.as_str()),
                    ("flex", missing),
                    "{flex}"
                );
            }
            other => panic!("{flex}: {other}"),
        }
    }
}

/// MuJoCo's own refusals of a flex's bodies and vertices (measured on 3.5.0).
#[test]
fn mujocos_body_and_vertex_refusals() {
    for (flex, reason) in [
        (
            r#"<flex name="f" dim="2" body="" element="0 1 2"></flex>"#,
            "vertbody and nodebody are both empty",
        ),
        (
            r#"<flex name="f" dim="2" body="a b" vertex="0 0 0 0 0 0 0 0 0" element="0 1 2"></flex>"#,
            "vertbody size must be 1 or nvert",
        ),
        (
            r#"<flex name="f" dim="2" body="a b c" vertex="0 0 0 0" element="0 1 2"></flex>"#,
            "vert size must be a multiple of 3",
        ),
        (
            r#"<flex name="f" dim="2" body="a b c" element="0 1 2 0"></flex>"#,
            "elem size must be multiple of (dim+1)",
        ),
    ] {
        match refusal(&doc(&triangle_bodies(), flex)) {
            MjcfError::InvalidValue { reason: got, .. } => assert_eq!(got, reason, "{flex}"),
            other => panic!("{flex}: {other}"),
        }
    }
}

/// MuJoCo's rigid flex (one body for every vertex) and one body holding two
/// vertices of a flex: MuJoCo loads both; sim-core's flex code takes a
/// vertex body per vertex.
#[test]
fn flex_repeated_vertex_body_is_refused() {
    assert_unsupported(
        &doc(
            &triangle_bodies(),
            r#"<flex name="f" dim="2" body="a a b" element="0 1 2"></flex>"#,
        ),
        "body 'a' holds two of its vertices",
    );
    assert_unsupported(
        &doc(
            &triangle_bodies(),
            r#"<flex name="f" dim="2" body="a" vertex="0 0 0 0.1 0 0 0 0.1 0" element="0 1 2"></flex>"#,
        ),
        "one body for every vertex",
    );
}

#[test]
fn flex_vertex_offset_is_refused() {
    assert_unsupported(
        &doc(
            &triangle_bodies(),
            r#"<flex name="f" dim="2" body="a b c" vertex="0 0 0 0.01 0 0 0 0 0" element="0 1 2"></flex>"#,
        ),
        "a vertex away from its body's origin",
    );
}

/// MuJoCo loads a vertex body with a hinge; sim-core would read the hinge's
/// dof as the vertex's x.
#[test]
fn flex_vertex_body_with_a_hinge_is_refused() {
    let bodies = [
        r#"<body name="a" pos="0 0 1"><joint type="hinge" axis="0 0 1"/><inertial pos="0 0 0" mass="0.5" diaginertia="1e-3 1e-3 1e-3"/></body>"#.to_string(),
        body("b", "0.1 0 1", true),
        body("c", "0 0.1 1", true),
    ]
    .concat();
    assert_unsupported(
        &doc(&bodies, TRIANGLE),
        "vertex body 'a' has joints other than three slides",
    );
}

/// Ledger-L141's fixture: each vertex on one y-slide. MuJoCo drops such an
/// edge's all-zero row at the dense size; sim-core's flex code takes three
/// slides along x, y and z, so the body is refused before a row is made.
#[test]
fn flex_vertex_body_with_one_slide_is_refused() {
    let one_slide = |name: &str, pos: &str| {
        format!(
            r#"<body name="{name}" pos="{pos}"><joint type="slide" axis="0 1 0"/><inertial pos="0 0 0" mass="0.5" diaginertia="1e-15 1e-15 1e-15"/></body>"#
        )
    };
    let bodies = one_slide("a", "0 0 1") + &one_slide("b", "0.1 0 1");
    assert_unsupported(
        &doc(
            &bodies,
            r#"<flex name="f" dim="1" body="a b" element="0 1"></flex>"#,
        ),
        "vertex body 'a' has joints other than three slides",
    );
    // Three slides out of order are not x, y and z either.
    let swapped = r#"<body name="a" pos="0 0 1"><joint type="slide" axis="0 1 0"/><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.5" diaginertia="1e-15 1e-15 1e-15"/></body>"#;
    assert_unsupported(
        &doc(
            &(swapped.to_string() + &body("b", "0.1 0 1", true)),
            r#"<flex name="f" dim="1" body="a b" element="0 1"></flex>"#,
        ),
        "vertex body 'a' has joints other than three slides",
    );
}

#[test]
fn flex_rotated_vertex_body_is_refused() {
    let rotated = r#"<body name="a" pos="0 0 1" euler="0 0 30"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.5" diaginertia="1e-15 1e-15 1e-15"/></body>"#;
    assert_unsupported(
        &doc(
            &(rotated.to_string() + &body("b", "0.1 0 1", true)),
            r#"<flex name="f" dim="1" body="a b" element="0 1"></flex>"#,
        ),
        "vertex body 'a' is rotated",
    );
}

/// A jointless vertex body under a moving body moves, and sim-core reads a
/// vertex without dofs as fixed; refused. This is also where ledger-L125's
/// pinned-vertex force would reach a dof: MuJoCo applies a pinned vertex's
/// elasticity force through its body (`mj_applyFT`, `engine_passive.c`), and
/// a body with no dof above it takes none.
#[test]
fn flex_jointless_vertex_body_on_a_moving_parent_is_refused() {
    let bodies = format!(
        r#"<body name="arm" pos="0 0 1"><freejoint/><inertial pos="0 0 0" mass="1" diaginertia="1e-3 1e-3 1e-3"/>{}</body>{}"#,
        body("a", "0 0 0", false),
        body("b", "0.1 0 1", true),
    );
    assert_unsupported(
        &doc(
            &bodies,
            r#"<flex name="f" dim="1" body="a b" element="0 1"></flex>"#,
        ),
        "vertex body 'a' moves with a body above it",
    );
}

/// Two flexes whose `body` lists share a body: MuJoCo's collision skips an
/// element pair with vertices on one body (`engine_collision_driver.c:
/// 2018-2029`). Two cables meet at `a`; sharing it they make no contact, and
/// with a body of their own at the same place they make one (MuJoCo 3.5.0:
/// one contact at distance -0.04, and none, measured).
#[test]
fn two_flexes_on_one_body_skip_their_pairs() {
    let flexes = |second: &str| {
        format!(
            r#"<flex name="f" dim="1" radius="0.02" body="a b" element="0 1"></flex>
               <flex name="g" dim="1" radius="0.02" body="{second} d" element="0 1"></flex>"#
        )
    };
    let bodies = body("a", "0 0 1", true)
        + &body("b", "0.1 0 1", true)
        + &body("a2", "0 0 1", true)
        + &body("d", "0.05 0.1 1", true);
    let contacts = |second: &str| {
        let model = load_model(&doc(&bodies, &flexes(second))).unwrap();
        let mut data = model.make_data();
        data.forward(&model).expect("forward");
        data.contacts
            .iter()
            .filter(|c| c.flex[0].is_some() && c.flex[1].is_some() && c.flex[0] != c.flex[1])
            .count()
    };
    assert_eq!(contacts("a2"), 1, "the control");
    assert_eq!(contacts("a"), 0);
}

/// A refusal names its flex once, in its location.
#[test]
fn a_flex_refusal_names_its_flex_once() {
    let xml = doc(
        &triangle_bodies(),
        r#"<flex name="cloth" dim="2" body="a a b" element="0 1 2"></flex>"#,
    );
    let message = refusal(&xml).to_string();
    assert_eq!(message.matches("cloth").count(), 1, "{message}");
}
