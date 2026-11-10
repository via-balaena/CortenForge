//! A mesh's mass properties as MuJoCo 3.5.0 computes them (book
//! Rigid-loading L35; `mjCMesh::Process`, `user_mesh.cc:1537-1711`), and a
//! mesh geom's mass and inertia from them, through the body it gives them
//! to.
//!
//! Every expected value was printed by MuJoCo 3.5.0 on the census's unfused
//! oracle (2026-11-10).

// The values are MuJoCo's, printed to the last digit.
#![allow(clippy::unwrap_used, clippy::unreadable_literal)]

use sim_mjcf::load_model;

/// A 0.4 × 0.2 × 0.1 box mesh from the origin, its asset's attributes
/// `attrs`, on a free body.
fn box_mesh(attrs: &str) -> String {
    format!(
        r#"<mujoco><asset><mesh name="bx" vertex="0 0 0  0.4 0 0  0.4 0.2 0  0 0.2 0  0 0 0.1  0.4 0 0.1  0.4 0.2 0.1  0 0.2 0.1" face="0 2 1  0 3 2  4 5 6  4 6 7  0 1 5  0 5 4  1 2 6  1 6 5  2 3 7  2 7 6  3 0 4  3 4 7" {attrs}/></asset><worldbody><body name="b"><freejoint/><geom type="mesh" mesh="bx"/></body></worldbody></mujoco>"#
    )
}

/// Body 1's mass, centre of mass, inertial frame (w, x, y, z) and principal
/// moments.
fn body(xml: &str) -> (f64, [f64; 3], [f64; 4], [f64; 3]) {
    let model = load_model(xml).unwrap();
    let q = model.body_iquat[1];
    (
        model.body_mass[1],
        model.body_ipos[1].into(),
        [q.w, q.i, q.j, q.k],
        model.body_inertia[1].into(),
    )
}

/// The largest of `ours - mujoco`, relative to the largest of `mujoco`.
fn relative_gap(ours: &[f64], mujoco: &[f64]) -> f64 {
    let scale = mujoco.iter().fold(0.0_f64, |m, v| m.max(v.abs()));
    ours.iter()
        .zip(mujoco)
        .fold(0.0_f64, |m, (a, b)| m.max((a - b).abs()))
        / scale
}

/// `body`'s values against MuJoCo's, each within `tolerance` relative.
fn assert_body(xml: &str, mujoco: (f64, [f64; 3], [f64; 4], [f64; 3]), tolerance: f64) {
    let ours = body(xml);
    let gaps = [
        relative_gap(&[ours.0], &[mujoco.0]),
        relative_gap(&ours.1, &mujoco.1),
        relative_gap(&ours.2, &mujoco.2),
        relative_gap(&ours.3, &mujoco.3),
    ];
    assert!(
        gaps.iter().all(|&gap| gap <= tolerance),
        "{ours:?} against MuJoCo's {mujoco:?}: gaps {gaps:?}"
    );
}

// MuJoCo moves a mesh by minus `refpos`, then turns it by `refquat`'s
// inverse, then scales it (`ApplyTransformations`, `user_mesh.cc:1440-1507`),
// before its mass properties, so both move the body's inertial frame. The
// mass properties themselves are not yet MuJoCo's arithmetic here, so these
// compare within 1e-12.

#[test]
fn a_meshs_refpos_moves_its_bodys_centre_of_mass() {
    assert_body(
        &box_mesh(r#"refpos="0.1 0.2 0.3""#),
        (
            8.000000357627876,
            [
                0.10000000298023222,
                -0.09999999850988386,
                -0.24999999925494187,
            ],
            [0.0, 0.7071067811865489, 0.0, 0.7071067811865489],
            [0.13333334326744115, 0.113333341777325, 0.033333335816860286],
        ),
        1e-12,
    );
}

#[test]
fn a_meshs_refquat_turns_its_bodys_inertial_frame() {
    assert_body(
        &box_mesh(r#"refquat="0.9 0.1 0.2 0.3""#),
        (
            8.000000357627876,
            [
                0.19052631862853703,
                -0.010526315946327988,
                0.12684210715325248,
            ],
            [
                0.29019049224388177,
                0.5078333795298153,
                -0.1450952377627668,
                0.7980238773527708,
            ],
            [
                0.1333333432674411,
                0.11333334177732483,
                0.033333335816860356,
            ],
        ),
        1e-12,
    );
}

#[test]
fn a_mesh_is_moved_then_turned_then_scaled() {
    // Scaled before the turn, its centre and moments would differ.
    assert_body(
        &box_mesh(r#"refpos="0.1 0.2 0.3" refquat="0.9 0.1 0.2 0.3" scale="1 2 3""#),
        (
            48.00000214576724,
            [
                0.09052631862853704,
                -0.4210526318926561,
                -0.5194736785402423,
            ],
            [
                0.9233491783402378,
                0.29706323927218753,
                0.20992395497458563,
                -0.12292949141419075,
            ],
            [2.7987409360325235, 2.5204054101737654, 0.5708840095326626],
        ),
        1e-12,
    );
}

/// A unit cube mesh about the origin, its faces `faces`, with `vertices`
/// after its eight and `attrs` on its asset, on a free body.
fn cube_mesh(vertices: &str, faces: &str, attrs: &str) -> String {
    format!(
        r#"<mujoco><asset><mesh name="m" {attrs} vertex="-0.5 -0.5 -0.5  0.5 -0.5 -0.5  0.5 0.5 -0.5  -0.5 0.5 -0.5  -0.5 -0.5 0.5  0.5 -0.5 0.5  0.5 0.5 0.5  -0.5 0.5 0.5 {vertices}" face="{faces}"/></asset><worldbody><body><freejoint/><geom type="mesh" mesh="m"/></body></worldbody></mujoco>"#
    )
}

/// The cube's twelve faces, each turned outward.
const CUBE_FACES: &str =
    "0 2 1  0 3 2  4 5 6  4 6 7  0 1 5  0 5 4  2 3 7  2 7 6  0 4 7  0 7 3  1 2 6  1 6 5";

/// The cube's faces with the first turned inward: it walks the edge from
/// vertex 0 to vertex 1 as the fifth does.
const ONE_FACE_TURNED: &str =
    "0 1 2  0 3 2  4 5 6  4 6 7  0 1 5  0 5 4  2 3 7  2 7 6  0 4 7  0 7 3  1 2 6  1 6 5";

// MuJoCo refuses an `exact` mesh two of whose faces walk an edge the same
// way, counting only faces of area above sqrt(mjMINVAL) on the vertices as
// written (`user_mesh.cc:1538-1563`), and no mesh of another mode.

#[test]
fn an_exact_mesh_with_a_face_turned_inward_is_refused() {
    let err = load_model(&cube_mesh("", ONE_FACE_TURNED, r#"inertia="exact""#))
        .unwrap_err()
        .to_string();
    assert!(
        err.contains("inconsistent orientation") && err.contains("the vertices 1 and 2."),
        "{err}"
    );
}

#[test]
fn a_mesh_of_another_mode_with_a_face_turned_inward_loads() {
    load_model(&cube_mesh("", ONE_FACE_TURNED, "")).unwrap();
}

#[test]
fn a_face_of_no_area_on_an_exact_meshs_edge_is_not_counted() {
    load_model(&cube_mesh(
        "",
        &format!("{CUBE_FACES}  0 1 1"),
        r#"inertia="exact""#,
    ))
    .unwrap();
}

#[test]
fn only_a_face_of_area_above_sqrt_mjminval_is_counted() {
    // A sliver on the edge from vertex 0 to vertex 1, its third vertex one
    // single-precision step from the edge: area 1.5e-8, below sqrt(mjMINVAL),
    // so MuJoCo loads the mesh; 1e-4 from it, area 5e-5, it refuses it.
    let faces = format!("{CUBE_FACES}  0 1 8");
    load_model(&cube_mesh(
        "0 -0.5 -0.49999997",
        &faces,
        r#"inertia="exact""#,
    ))
    .unwrap();
    let err = load_model(&cube_mesh("0 -0.5 -0.4999", &faces, r#"inertia="exact""#))
        .unwrap_err()
        .to_string();
    assert!(err.contains("the vertices 1 and 2."), "{err}");
}

#[test]
fn the_faces_counted_are_those_of_the_mesh_as_written() {
    // Scaled by 10 the sliver's area is 1.5e-6 and scaled by 1e-3 the thin
    // face's is 5e-11, on the other side of sqrt(mjMINVAL) from each as
    // written; MuJoCo counts each as written.
    let faces = format!("{CUBE_FACES}  0 1 8");
    let exact_scaled = |scale: &str| format!(r#"inertia="exact" scale="{scale}""#);
    load_model(&cube_mesh(
        "0 -0.5 -0.49999997",
        &faces,
        &exact_scaled("10 10 10"),
    ))
    .unwrap();
    let err = load_model(&cube_mesh(
        "0 -0.5 -0.4999",
        &faces,
        &exact_scaled("0.001 0.001 0.001"),
    ))
    .unwrap_err()
    .to_string();
    assert!(err.contains("the vertices 1 and 2."), "{err}");
}
