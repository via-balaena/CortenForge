//! A `<flex>` whose vertices are the bodies it names, as MuJoCo 3.5.0's
//! (`mjCFlex::Compile`, `user_mesh.cc:4103-4213`), and the subset of it that
//! sim-core's flex code takes.
//!
//! Each vertex is the body `body` names, at that body's origin. sim-core reads
//! a vertex's three dofs as its world x, y and z and a vertex without dofs as
//! fixed, so a vertex body must be jointless with no joint above it, or have
//! exactly three slides along x, y and z, unrotated, with no joint or mocap
//! body above it; anything else MuJoCo loads is refused as a stated
//! limitation. The arrays below were measured on MuJoCo
//! 3.5.0 for the same documents (edges, `flexedge_rigid`, `flexedge_length0`).

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use sim_mjcf::{ElementKind, MjcfError, load_model, model_from_mjcf, parse_mjcf_str};

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

/// sim-mjcf's old child form: `<vertex>`, `<element>` and `<pin>` inside a
/// `<flex>`, each "Schema violation: unrecognized element" in MuJoCo 3.5.0
/// (measured), whether written open or self-closing, and refused before a
/// missing attribute is, as MuJoCo checks its schema first.
#[test]
fn flex_child_forms_are_refused() {
    for child in [
        r#"<vertex pos="0 0 0"/>"#,
        r#"<vertex pos="0 0 0"></vertex>"#,
        r#"<element data="0 1 2"/>"#,
        r#"<element data="0 1 2"></element>"#,
        r#"<pin id="0"/>"#,
        r#"<pin id="0"></pin>"#,
    ] {
        for flex in [
            format!(r#"<flex name="f" dim="2" body="a b c" element="0 1 2">{child}</flex>"#),
            format!(r#"<flex name="f" dim="2">{child}</flex>"#),
        ] {
            match refusal(&doc(&triangle_bodies(), &flex)) {
                MjcfError::UnknownElement {
                    element, parent, ..
                } => {
                    assert_eq!(parent, "flex", "{flex}");
                    assert!(
                        child.starts_with(&format!("<{element}")),
                        "{flex}: {element}"
                    );
                }
                other => panic!("{flex}: {other}"),
            }
        }
    }
}

/// `mass` on a `<flex>`: "Schema violation: unrecognized attribute: 'mass'" in
/// MuJoCo 3.5.0 (measured); a vertex's mass is its body's.
#[test]
fn flex_mass_is_refused() {
    match refusal(&doc(
        &triangle_bodies(),
        r#"<flex name="f" dim="2" mass="1" body="a b c" element="0 1 2"></flex>"#,
    )) {
        MjcfError::UnknownAttribute {
            element, attribute, ..
        } => assert_eq!((element.as_str(), attribute.as_str()), ("flex", "mass")),
        other => panic!("{other}"),
    }
}

/// MuJoCo 3.5.0's own trilinear flex, as `mj_saveLastXML` writes the one a
/// trilinear flexcomp makes: its 9 vertices on one body at offsets, 8 node
/// bodies, `selfcollide` none. MuJoCo loads it (measured).
const TRILINEAR: &str = r#"<mujoco model="trilinear">
  <worldbody>
      <body name="holder" pos="0 0 1">
        <body name="fc_0_0_0" pos="-0.1 -0.1 0">
          <inertial pos="0 0 0" mass="0.125" diaginertia="2.08333e-06 2.08333e-06 2.08333e-06"/>
          <joint pos="0 0 0" axis="1 0 0" type="slide"/>
          <joint pos="0 0 0" axis="0 1 0" type="slide"/>
          <joint pos="0 0 0" axis="0 0 1" type="slide"/>
        </body>
        <body name="fc_0_0_1" pos="-0.1 -0.1 0">
          <inertial pos="0 0 0" mass="0.125" diaginertia="2.08333e-06 2.08333e-06 2.08333e-06"/>
          <joint pos="0 0 0" axis="1 0 0" type="slide"/>
          <joint pos="0 0 0" axis="0 1 0" type="slide"/>
          <joint pos="0 0 0" axis="0 0 1" type="slide"/>
        </body>
        <body name="fc_0_1_0" pos="-0.1 0.1 0">
          <inertial pos="0 0 0" mass="0.125" diaginertia="2.08333e-06 2.08333e-06 2.08333e-06"/>
          <joint pos="0 0 0" axis="1 0 0" type="slide"/>
          <joint pos="0 0 0" axis="0 1 0" type="slide"/>
          <joint pos="0 0 0" axis="0 0 1" type="slide"/>
        </body>
        <body name="fc_0_1_1" pos="-0.1 0.1 0">
          <inertial pos="0 0 0" mass="0.125" diaginertia="2.08333e-06 2.08333e-06 2.08333e-06"/>
          <joint pos="0 0 0" axis="1 0 0" type="slide"/>
          <joint pos="0 0 0" axis="0 1 0" type="slide"/>
          <joint pos="0 0 0" axis="0 0 1" type="slide"/>
        </body>
        <body name="fc_1_0_0" pos="0.1 -0.1 0">
          <inertial pos="0 0 0" mass="0.125" diaginertia="2.08333e-06 2.08333e-06 2.08333e-06"/>
          <joint pos="0 0 0" axis="1 0 0" type="slide"/>
          <joint pos="0 0 0" axis="0 1 0" type="slide"/>
          <joint pos="0 0 0" axis="0 0 1" type="slide"/>
        </body>
        <body name="fc_1_0_1" pos="0.1 -0.1 0">
          <inertial pos="0 0 0" mass="0.125" diaginertia="2.08333e-06 2.08333e-06 2.08333e-06"/>
          <joint pos="0 0 0" axis="1 0 0" type="slide"/>
          <joint pos="0 0 0" axis="0 1 0" type="slide"/>
          <joint pos="0 0 0" axis="0 0 1" type="slide"/>
        </body>
        <body name="fc_1_1_0" pos="0.1 0.1 0">
          <inertial pos="0 0 0" mass="0.125" diaginertia="2.08333e-06 2.08333e-06 2.08333e-06"/>
          <joint pos="0 0 0" axis="1 0 0" type="slide"/>
          <joint pos="0 0 0" axis="0 1 0" type="slide"/>
          <joint pos="0 0 0" axis="0 0 1" type="slide"/>
        </body>
        <body name="fc_1_1_1" pos="0.1 0.1 0">
          <inertial pos="0 0 0" mass="0.125" diaginertia="2.08333e-06 2.08333e-06 2.08333e-06"/>
          <joint pos="0 0 0" axis="1 0 0" type="slide"/>
          <joint pos="0 0 0" axis="0 1 0" type="slide"/>
          <joint pos="0 0 0" axis="0 0 1" type="slide"/>
        </body>
      </body>
    </worldbody>
  <deformable>
    <flex name="fc" body="holder holder holder holder holder holder holder holder holder" vertex="-0.1 -0.1 0 -0.1 0 0 -0.1 0.1 0 0 -0.1 0 0 0 0 0 0.1 0 0.1 -0.1 0 0.1 0 0 0.1 0.1 0" element="0 3 4 0 4 1 1 4 5 1 5 2 3 6 7 3 7 4 4 7 8 4 8 5" node="fc_0_0_0 fc_0_0_1 fc_0_1_0 fc_0_1_1 fc_1_0_0 fc_1_0_1 fc_1_1_0 fc_1_1_1">
      <contact selfcollide="none"/>
      <elasticity/>
    </flex>
  </deformable>
</mujoco>"#;

/// `node` (MuJoCo's trilinear interpolation): MuJoCo 3.5.0's own checks of
/// it first, each with MuJoCo's message (measured; the element count's with
/// the count, which MuJoCo's message misprints), then a stated limitation
/// (registry `D-FLEX-NODE`). `node` alone is refused for the
/// missing `body`, and an empty `node` is no node, as MuJoCo reads them.
#[test]
fn flex_node_is_refused() {
    assert_unsupported(TRILINEAR, "a flex's node");
    for (xml, reason) in [
        (
            TRILINEAR.replace(r#"<contact selfcollide="none"/>"#, "<contact/>"),
            "trilinear interpolation cannot do self-collision",
        ),
        (
            TRILINEAR.replace(
                r#"<contact selfcollide="none"/>"#,
                r#"<contact selfcollide="none" internal="true"/>"#,
            ),
            "trilinear interpolation cannot do internal collisions",
        ),
        (
            TRILINEAR.replace(r#" fc_0_1_1 fc_1_0_0 fc_1_0_1 fc_1_1_0 fc_1_1_1""#, "\""),
            "number of nodes must be 0^3 but it is 3",
        ),
        // MuJoCo's cube root truncated: 64 nodes read as 2^3 (measured).
        (
            TRILINEAR.replace(
                r#"node="fc_0_0_0 fc_0_0_1 fc_0_1_0 fc_0_1_1 fc_1_0_0 fc_1_0_1 fc_1_1_0 fc_1_1_1""#,
                &format!(
                    r#"node="{}""#,
                    (0..64)
                        .map(|i| format!("n{i}"))
                        .collect::<Vec<_>>()
                        .join(" ")
                ),
            ),
            "number of nodes must be 2^3 but it is 64",
        ),
        // With a Young's modulus, at least (3·8)²/21 = 27 elements for 8
        // nodes, where the flex has 8 (MuJoCo refuses it, measured).
        (
            TRILINEAR.replace("<elasticity/>", r#"<elasticity young="1e4"/>"#),
            "Trilinear dofs are require at least 27 elements",
        ),
    ] {
        match refusal(&xml) {
            MjcfError::InvalidValue { reason: got, .. } => assert_eq!(got, reason),
            other => panic!("{reason}: {other}"),
        }
    }
    match refusal(&TRILINEAR.replace(r#" fc_1_1_1""#, r#" nope""#)) {
        MjcfError::UndefinedReference { name, .. } => assert_eq!(name, "nope"),
        other => panic!("an unknown node body: {other}"),
    }
    match refusal(&doc(
        &triangle_bodies(),
        r#"<flex name="f" dim="2" node="a b c" element="0 1 2"></flex>"#,
    )) {
        MjcfError::MissingAttribute { attribute, .. } => assert_eq!(attribute, "body"),
        other => panic!("{other}"),
    }
    load_model(&doc(
        &triangle_bodies(),
        r#"<flex name="f" dim="2" body="a b c" node="" element="0 1 2"></flex>"#,
    ))
    .expect("an empty node is no node");
}

/// Three joints along x, y and z that are not slides (three hinges): refused
/// as any other joints are; sim-core reads a vertex's three dofs as
/// translations.
#[test]
fn flex_vertex_body_with_three_hinges_is_refused() {
    let hinges = r#"<body name="a" pos="0 0 1"><joint type="hinge" axis="1 0 0"/><joint type="hinge" axis="0 1 0"/><joint type="hinge" axis="0 0 1"/><inertial pos="0 0 0" mass="0.5" diaginertia="1e-3 1e-3 1e-3"/></body>"#;
    assert_unsupported(
        &doc(
            &(hinges.to_string() + &body("b", "0.1 0 1", true)),
            r#"<flex name="f" dim="1" body="a b" element="0 1"></flex>"#,
        ),
        "vertex body 'a' has joints other than three slides",
    );
}

/// Two slides along x and y are not three, even where the joint declared
/// next (another body's) slides along z.
#[test]
fn flex_vertex_body_with_two_slides_is_refused() {
    let two = r#"<body name="a" pos="0 0 1"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><inertial pos="0 0 0" mass="0.5" diaginertia="1e-15 1e-15 1e-15"/></body>"#;
    let next = r#"<body name="c" pos="1 1 1"><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.5" diaginertia="1e-15 1e-15 1e-15"/></body>"#;
    assert_unsupported(
        &doc(
            &[two, next, &body("b", "0.1 0 1", true)].concat(),
            r#"<flex name="f" dim="1" body="a b" element="0 1"></flex>"#,
        ),
        "vertex body 'a' has joints other than three slides",
    );
}

/// After a free joint (7 qpos, 6 dofs) a vertex's qpos address is its first
/// slide's, one past its dof address.
#[test]
fn a_vertex_after_a_free_joint_takes_its_slides_qpos_address() {
    let free =
        r#"<body name="ball" pos="1 1 1"><freejoint/><geom type="sphere" size="0.05"/></body>"#;
    let model = load_model(&doc(&(free.to_string() + &triangle_bodies()), TRIANGLE)).unwrap();
    assert_eq!(model.flexvert_dofadr, [6, 9, 12]);
    assert_eq!(model.flexvert_qposadr, [7, 10, 13]);
}

/// MuJoCo 3.5.0's own refusals come first, in its order, before sim-mjcf's
/// stated limitations (measured on the oracle for each document): a rigid
/// flex (one body) with too few vertices, an element id out of range or a
/// partial element, and an unknown body named after a repeated one or with
/// a repeated vertex in an element.
#[test]
fn mujocos_refusals_come_before_the_stated_limitations() {
    let one = |dim: usize, vertex: &str, element: &str| {
        doc(
            &triangle_bodies(),
            &format!(
                r#"<flex name="f" dim="{dim}" body="a" vertex="{vertex}" element="{element}"></flex>"#
            ),
        )
    };
    let rigid_poisson = doc(
        &triangle_bodies(),
        r#"<flex name="f" dim="1" body="a" vertex="0 0 0 0.1 0 0" element="0 1"><elasticity young="100" poisson="0.6"/></flex>"#,
    );
    for (xml, reason) in [
        (rigid_poisson, "Poisson ratio must be in [0, 0.5)"),
        (one(1, "0 0 0", "0 1"), "not enough vertices"),
        (
            one(1, "0 0 0 0.1 0 0", "0 5"),
            "elem vertex id out of range",
        ),
        (
            one(1, "0 0 0 0.1 0 0", "0 1 1"),
            "elem size must be multiple of (dim+1)",
        ),
    ] {
        match refusal(&xml) {
            MjcfError::InvalidValue { reason: got, .. } => assert_eq!(got, reason, "{xml}"),
            other => panic!("{xml}: {other}"),
        }
    }
    for flex in [
        r#"<flex name="f" dim="2" body="a a nope" element="0 1 2"></flex>"#,
        r#"<flex name="f" dim="2" body="a b nope" element="0 0 1"></flex>"#,
    ] {
        match refusal(&doc(&triangle_bodies(), flex)) {
            MjcfError::UndefinedReference {
                kind: ElementKind::Body,
                name,
                ..
            } => assert_eq!(name, "nope", "{flex}"),
            other => panic!("{flex}: {other}"),
        }
    }
}

/// Slides inside a mocap body turn with it, so their dofs stop being world
/// x, y and z, which sim-core's flex code assumes: refused. MuJoCo loads it;
/// turned 90° about z its edge Jacobian takes the x-slide (measured). A
/// jointless vertex body on the mocap body has no dofs to misread and loads.
#[test]
fn flex_slide_vertex_body_under_a_mocap_body_is_refused() {
    let bodies = |slides: bool| {
        format!(
            r#"{}<body name="m" mocap="true" pos="0 0 1">{}</body>"#,
            body("a", "0 0 1", false),
            body("b", "0.1 0 0", slides),
        )
    };
    let flex = r#"<flex name="f" dim="1" body="a b" element="0 1"><edge stiffness="10"/></flex>"#;
    assert_unsupported(
        &doc(&bodies(true), flex),
        "vertex body 'b' moves with a body above it",
    );
    let model = load_model(&doc(&bodies(false), flex)).unwrap();
    assert_eq!(model.flexvert_dofadr, [usize::MAX, usize::MAX]);
}

/// A jointless vertex body may be rotated: it has no dofs, and its vertex
/// is at its origin whatever its frame. MuJoCo 3.5.0 loads the same
/// document (measured).
#[test]
fn a_rotated_jointless_vertex_body_loads() {
    let rotated = r#"<body name="a" pos="0 0 1" euler="0 0 30"><inertial pos="0 0 0" mass="0.5" diaginertia="1e-15 1e-15 1e-15"/></body>"#;
    let model = load_model(&doc(
        &(rotated.to_string() + &body("b", "0.1 0 1", true)),
        r#"<flex name="f" dim="1" body="a b" element="0 1"></flex>"#,
    ))
    .unwrap();
    assert!((model.flexedge_length0[0] - 0.1).abs() < 1e-15);
    assert_eq!(model.flexedge_rigid, [false]);
}

/// A flex built in code with no element is refused as MuJoCo's compiler
/// refuses it ("elem is empty"); the parser reads an empty `element` as
/// missing.
#[test]
fn a_code_built_flex_with_no_element_is_refused() {
    let mut mjcf = parse_mjcf_str(&doc(&triangle_bodies(), TRIANGLE)).unwrap();
    mjcf.flex[0].elements.clear();
    match model_from_mjcf(&mjcf, None) {
        Err(MjcfError::InvalidValue { reason, .. }) => assert_eq!(reason, "elem is empty"),
        other => panic!("{other:?}"),
    }
}

/// With a Young's modulus, MuJoCo 3.5.0 requires a Poisson ratio in
/// [0, 0.5): 0.6, -0.1 and 0.5 refused, 0.49 loaded (measured). sim-mjcf
/// clamped it and loaded every one.
#[test]
fn a_poisson_ratio_out_of_range_is_refused() {
    let flex = |poisson: &str| {
        doc(
            &triangle_bodies(),
            &format!(
                r#"<flex name="f" dim="2" body="a b c" element="0 1 2"><elasticity young="100" poisson="{poisson}" thickness="0.01"/></flex>"#
            ),
        )
    };
    for poisson in ["0.6", "-0.1", "0.5"] {
        match refusal(&flex(poisson)) {
            MjcfError::InvalidValue { reason, .. } => {
                assert_eq!(reason, "Poisson ratio must be in [0, 0.5)", "{poisson}");
            }
            other => panic!("{poisson}: {other}"),
        }
    }
    load_model(&flex("0.49")).expect("0.49");
    let flexcomp = r#"<mujoco><deformable><flexcomp name="g" type="grid" count="2 2 1" spacing="0.1 0.1 0.1" dim="2"><elasticity young="100" poisson="0.6" thickness="0.01"/></flexcomp></deformable></mujoco>"#;
    match refusal(flexcomp) {
        MjcfError::InvalidValue { reason, .. } => {
            assert_eq!(reason, "Poisson ratio must be in [0, 0.5)");
        }
        other => panic!("a flexcomp: {other}"),
    }
}

/// A positive edge stiffness on a flex of dim above 1 is refused as MuJoCo
/// 3.5.0's compiler refuses it (`user_mesh.cc:4123-4125`); a stiffness of 0
/// there and one on a cable load (measured on the same `<flex>` documents).
/// The flexcomp twin is refused the same way. sim-mjcf loaded both.
#[test]
fn an_edge_stiffness_on_a_flex_of_dim_above_1_is_refused() {
    let reason = "edge stiffness only available for dim=1, please use elasticity plugins";
    let triangle = |stiffness: &str| {
        doc(
            &triangle_bodies(),
            &format!(
                r#"<flex name="f" dim="2" body="a b c" element="0 1 2"><edge stiffness="{stiffness}"/></flex>"#
            ),
        )
    };
    let flexcomp = r#"<mujoco><deformable><flexcomp name="g" type="grid" count="2 2 1" spacing="0.1 0.1 0.1"><edge stiffness="1"/></flexcomp></deformable></mujoco>"#;
    for (form, xml) in [("flex", triangle("1")), ("flexcomp", flexcomp.to_string())] {
        match refusal(&xml) {
            MjcfError::InvalidValue {
                element,
                attribute,
                reason: got,
                ..
            } => assert_eq!(
                (element.as_str(), attribute.as_str(), got.as_str()),
                ("edge", "stiffness", reason),
                "{form}"
            ),
            other => panic!("{form}: {other}"),
        }
    }
    load_model(&triangle("0")).expect("a stiffness of 0");
    load_model(&doc(
        &triangle_bodies(),
        r#"<flex name="f" dim="1" body="a b" element="0 1"><edge stiffness="1"/></flex>"#,
    ))
    .expect("a cable");
}

/// A `<deformable><flexcomp>` that generates no element (a grid one vertex
/// wide) is refused, "elem is empty", as MuJoCo's compiler refuses an empty
/// flex; MuJoCo's own grid flexcomp refuses it as "No elements were created
/// in grid" (measured). It loaded with no element before Rigid-loading L10.
#[test]
fn a_flexcomp_with_no_element_is_refused() {
    let xml = r#"<mujoco><deformable><flexcomp name="g" type="grid" count="3 1 1" spacing="0.1 0.1 0.1" dim="2"/></deformable></mujoco>"#;
    match refusal(xml) {
        MjcfError::InvalidValue { reason, .. } => assert_eq!(reason, "elem is empty"),
        other => panic!("{other}"),
    }
}

/// A `<deformable><flexcomp>` makes its vertices' bodies; `body` on it is
/// refused as MuJoCo 3.5.0's schema refuses it ("unrecognized attribute:
/// 'body'", measured on its own flexcomp). It once hung the vertex bodies on
/// the named bodies.
#[test]
fn a_flexcomp_body_is_refused() {
    let xml = r#"<mujoco><worldbody><body name="arm"><joint type="hinge"/><geom size="0.1"/></body></worldbody><deformable><flexcomp name="g" type="grid" count="2 2 1" spacing="0.1 0.1 0.1" body="arm arm arm arm"/></deformable></mujoco>"#;
    match refusal(xml) {
        MjcfError::UnknownAttribute {
            element, attribute, ..
        } => assert_eq!((element.as_str(), attribute.as_str()), ("flexcomp", "body")),
        other => panic!("{other}"),
    }
}

/// A flexcomp built in code with a `<flex>`'s `body` or `node` is refused
/// as the parser refuses `body` on one; it loaded with them ignored.
#[test]
fn a_code_built_flexcomp_with_body_or_node_is_refused() {
    let xml = r#"<mujoco><deformable><flexcomp name="g" type="grid" count="2 2 1" spacing="0.1 0.1 0.1"/></deformable></mujoco>"#;
    for attribute in ["body", "node"] {
        let mut mjcf = parse_mjcf_str(xml).unwrap();
        let names = vec!["world".to_string(); 4];
        if attribute == "body" {
            mjcf.flex[0].body = names;
        } else {
            mjcf.flex[0].node = names;
        }
        match model_from_mjcf(&mjcf, None) {
            Err(MjcfError::UnknownAttribute {
                element,
                attribute: got,
                ..
            }) => assert_eq!((element.as_str(), got.as_str()), ("flexcomp", attribute)),
            other => panic!("{attribute}: {other:?}"),
        }
    }
    let mjcf = parse_mjcf_str(xml).unwrap();
    model_from_mjcf(&mjcf, None).expect("without them");
}

/// With `vertcollide`, each vertex body a flexcomp makes gets a sphere of the
/// flex's radius in group 4, in the root class (its `contype` here), as
/// MuJoCo 3.5.0's flexcomp (`user_flexcomp.cc:505-511`); a pinned vertex
/// gets none, MuJoCo making it no body. MuJoCo's own flexcomp under
/// `<worldbody>` gives the same three spheres, and each vertex body the
/// same inertia (measured). sim-mjcf stored the flag and made no geom.
#[test]
fn a_flexcomp_with_vertcollide_gives_its_vertex_bodies_spheres() {
    let xml = r#"<mujoco><default><geom contype="2"/></default><deformable><flexcomp name="g" type="grid" count="2 2 1" spacing="0.1 0.1 0.1" radius="0.02" mass="1"><contact vertcollide="true"/><pin id="3"/></flexcomp></deformable></mujoco>"#;
    let model = load_model(xml).unwrap();
    assert_eq!(model.ngeom, 3);
    for g in 0..3 {
        assert_eq!(model.geom_type[g], sim_core::GeomType::Sphere, "{g}");
        assert_eq!(model.geom_size[g].x.to_bits(), 0.02_f64.to_bits(), "{g}");
        assert_eq!((model.geom_group[g], model.geom_contype[g]), (4, 2), "{g}");
        assert_eq!(model.geom_body[g], model.flexvert_bodyid[g], "{g}");
    }
    assert_eq!(model.body_mass[1..4], [0.25; 3]);
    assert_eq!(
        model.body_inertia[1].x.to_bits(),
        4.166_666_666_666_667e-6_f64.to_bits()
    );
    let plain = load_model(&xml.replace(r#" vertcollide="true""#, "")).unwrap();
    assert_eq!(plain.ngeom, 0);
}

/// The spheres hold up a cloth whose own contact is off: a grid of four
/// vertices with `vertcollide`, started on a plane, comes to rest with each
/// vertex about its radius above it (0.0196 for 0.02, measured), and
/// without the spheres it falls through.
#[test]
fn a_cloth_rests_on_its_vertex_spheres() {
    let xml = r#"<mujoco><worldbody><geom type="plane" size="1 1 0.1"/></worldbody><deformable><flexcomp name="g" type="grid" count="2 2 1" spacing="0.1 0.1 0.1" radius="0.02" mass="1"><contact contype="0" conaffinity="0" vertcollide="true"/></flexcomp></deformable></mujoco>"#;
    let lowest = |xml: &str| {
        let model = load_model(xml).unwrap();
        let mut data = model.make_data();
        for _ in 0..500 {
            data.step(&model).expect("step");
        }
        data.flexvert_xpos
            .iter()
            .map(|p| p.z)
            .fold(f64::INFINITY, f64::min)
    };
    let resting = lowest(xml);
    assert!((resting - 0.02).abs() < 2e-3, "on its spheres: {resting}");
    let fallen = lowest(&xml.replace(r#" vertcollide="true""#, ""));
    assert!(fallen < -0.1, "without them: {fallen}");
}
