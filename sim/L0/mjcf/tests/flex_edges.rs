//! A flex's edges in MuJoCo 3.5.0's order, its hinges in edge order, and the
//! element checks MuJoCo's compiler makes before either.
//!
//! The expected edges and flaps were measured on MuJoCo 3.5.0, each vertex a
//! body of its own: MuJoCo numbers an edge where it first appears, the elements
//! in order and each element's edges in its `eledge` order
//! (`user_mesh.cc:3447-3452`, `:4297-4321`). Each document whose edges or
//! hinges are checked is loaded twenty times, so an order that depends on a
//! hash seed cannot pass by chance.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::fmt::Write as _;

use sim_mjcf::{MjcfError, load_model, model_from_mjcf, parse_mjcf_str};

/// A flex of `dim` whose vertices are bodies `v0`, `v1`, ... at `vertices`
/// (three numbers each), each with three slides, as MuJoCo 3.5.0's
/// `<flex body=...>`.
fn flex_doc(dim: usize, vertices: &str, elements: &str) -> String {
    let coords: Vec<&str> = vertices.split_whitespace().collect();
    let mut bodies = String::new();
    for (i, pos) in coords.chunks(3).enumerate() {
        write!(
            bodies,
            r#"<body name="v{i}" pos="{}"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.001" diaginertia="1e-15 1e-15 1e-15"/></body>"#,
            pos.join(" ")
        )
        .unwrap();
    }
    let names: Vec<String> = (0..coords.len() / 3).map(|i| format!("v{i}")).collect();
    format!(
        r#"<mujoco>
            <worldbody>{bodies}</worldbody>
            <deformable>
                <flex name="f" dim="{dim}" body="{}" element="{elements}"></flex>
            </deformable>
        </mujoco>"#,
        names.join(" ")
    )
}

fn edges_of(xml: &str) -> Vec<[usize; 2]> {
    load_model(xml).unwrap().flexedge_vert
}

/// Every one of twenty loads of `xml` has the edges `expected`.
fn assert_edges(xml: &str, expected: &[[usize; 2]]) {
    for _ in 0..20 {
        assert_eq!(edges_of(xml), expected, "{xml}");
    }
}

#[test]
fn a_triangle_pair_has_mujocos_edges_and_one_hinge_beside_its_flap() {
    let xml = flex_doc(2, "0 0 0  1 0 0  0 1 0  1 1 0", "0 1 2  1 3 2");
    assert_edges(&xml, &[[1, 2], [0, 2], [0, 1], [2, 3], [1, 3]]);
    let model = load_model(&xml).unwrap();
    assert_eq!(model.flexedge_flap[0], [0, 3]);
    assert_eq!(model.flexhinge_vert, vec![[1, 2, 0, 3]]);
}

#[test]
fn a_cable_has_mujocos_edges() {
    let xml = flex_doc(1, "0 0 0  1 0 0  2 0 0  3 0 0", "0 1  1 2  2 3");
    assert_edges(&xml, &[[0, 1], [1, 2], [2, 3]]);
}

#[test]
fn a_negatively_turned_tetrahedron_keeps_its_vertices_and_has_mujocos_edges() {
    let xml = flex_doc(3, "0 0 0  0 1 0  1 0 0  0 0 1", "0 1 2 3");
    assert_edges(&xml, &[[0, 1], [1, 2], [0, 2], [2, 3], [0, 3], [1, 3]]);
}

#[test]
fn a_positively_turned_tetrahedron_has_the_edges_of_mujocos_turned_element() {
    // MuJoCo turns the element to (0, 2, 1, 3) before it numbers the edges.
    let xml = flex_doc(3, "0 0 0  1 0 0  0 1 0  0 0 1", "0 1 2 3");
    assert_edges(&xml, &[[0, 2], [1, 2], [0, 1], [1, 3], [0, 3], [2, 3]]);
}

#[test]
fn a_grids_hinges_follow_its_edges() {
    let xml = flex_doc(
        2,
        "0 0 0  1 0 0  2 0 0  0 1 0  1 1 0  2 1 0",
        "0 1 3  1 4 3  1 2 4  2 5 4",
    );
    assert_edges(
        &xml,
        &[
            [1, 3],
            [0, 3],
            [0, 1],
            [3, 4],
            [1, 4],
            [2, 4],
            [1, 2],
            [4, 5],
            [2, 5],
        ],
    );
    for _ in 0..20 {
        let model = load_model(&xml).unwrap();
        // The interior edges 0, 4 and 5, in that order, each with MuJoCo's flap.
        assert_eq!(
            model.flexhinge_vert,
            vec![[1, 3, 0, 4], [1, 4, 3, 2], [2, 4, 1, 5]]
        );
        assert_eq!(model.flexedge_flap[0], [0, 4]);
        assert_eq!(model.flexedge_flap[4], [3, 2]);
        assert_eq!(model.flexedge_flap[5], [1, 5]);
    }
}

fn refusal(err: &MjcfError) -> (&str, &str) {
    match err {
        MjcfError::InvalidValue {
            element,
            attribute,
            reason,
            ..
        } if element == "flex" => (attribute.as_str(), reason.as_str()),
        other => panic!("{other}"),
    }
}

#[test]
fn a_flex_of_dimension_four_is_refused() {
    let err = load_model(&flex_doc(
        4,
        "0 0 0  1 0 0  0 1 0  0 0 1  1 1 1",
        "0 1 2 3 4",
    ))
    .unwrap_err();
    assert_eq!(refusal(&err), ("dim", "dim must be 1, 2 or 3"));
    assert!(err.to_string().contains("(flex 'f')"), "{err}");
}

#[test]
fn an_element_naming_a_vertex_the_flex_lacks_is_refused() {
    // Three vertices: index 3, the first past them, and one far past.
    for elements in ["0 1 3", "0 1 9"] {
        let err = load_model(&flex_doc(2, "0 0 0  1 0 0  0 1 0", elements)).unwrap_err();
        assert_eq!(
            refusal(&err),
            ("element", "elem vertex id out of range"),
            "{elements}"
        );
    }
}

#[test]
fn an_element_naming_a_vertex_twice_is_refused() {
    // MuJoCo 3.5.0 refuses each (measured), in every dimension.
    for (dim, vertices, elements) in [
        (1, "0 0 0  1 0 0", "0 0"),
        (2, "0 0 0  1 0 0  0 1 0", "0 1 1"),
        (2, "0 0 0  1 0 0  0 1 0", "1 0 1"),
        (3, "0 0 0  1 0 0  0 1 0  0 0 1", "0 1 2 2"),
    ] {
        let err = load_model(&flex_doc(dim, vertices, elements)).unwrap_err();
        assert_eq!(
            refusal(&err),
            ("element", "repeated vertex in element"),
            "{elements}"
        );
    }
}

#[test]
fn a_flex_with_fewer_vertices_than_an_element_takes_is_refused() {
    // Checked before the elements, as MuJoCo 3.5.0 does (measured: refused
    // with this text, though the element also repeats a vertex).
    let err = load_model(&flex_doc(2, "0 0 0  1 0 0", "0 1 1")).unwrap_err();
    assert_eq!(refusal(&err), ("vertex", "not enough vertices"));
}

#[test]
fn an_unnamed_flexs_refusal_names_it_by_its_index() {
    let err = load_model(
        r#"<mujoco>
            <worldbody>
                <body name="a0"></body><body name="a1" pos="1 0 0"></body><body name="a2" pos="0 1 0"></body>
                <body name="b0"></body><body name="b1" pos="1 0 0"></body><body name="b2" pos="0 1 0"></body>
                <body name="b3" pos="0 0 1"></body><body name="b4" pos="1 1 1"></body>
            </worldbody>
            <deformable>
                <flex name="a" dim="2" body="a0 a1 a2" element="0 1 2"></flex>
                <flex dim="4" body="b0 b1 b2 b3 b4" element="0 1 2 3 4"></flex>
            </deformable>
        </mujoco>"#,
    )
    .unwrap_err();
    assert_eq!(refusal(&err), ("dim", "dim must be 1, 2 or 3"));
    assert!(err.to_string().ends_with("(flex #1)"), "{err}");
}

#[test]
fn an_element_of_the_wrong_size_is_refused() {
    // The parser makes elements of dim + 1 vertices; a caller can build others.
    let mut mjcf = parse_mjcf_str(&flex_doc(2, "0 0 0  1 0 0  0 1 0", "0 1 2")).unwrap();
    mjcf.flex[0].elements[0].pop();
    let err = model_from_mjcf(&mjcf, None).unwrap_err();
    assert_eq!(
        refusal(&err),
        ("element", "elem size must be multiple of (dim+1)")
    );
}
