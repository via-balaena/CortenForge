//! A mesh's mass properties as MuJoCo 3.5.0 computes them (book
//! Rigid-loading L35; `mjCMesh::Process`, `user_mesh.cc:1537-1708`), and a
//! mesh geom's mass and inertia from them, through the body it gives them
//! to.
//!
//! Every expected value was printed by MuJoCo 3.5.0 on the census's unfused
//! oracle (2026-11-10).

// The values are MuJoCo's, printed to the last digit, and compared to it.
#![allow(clippy::unwrap_used, clippy::unreadable_literal, clippy::float_cmp)]

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

/// The largest of `|ours - mujoco|`, relative to the largest of `|mujoco|`
/// where that is not 0; `NaN` where any of `ours` is.
fn relative_gap(ours: &[f64], mujoco: &[f64]) -> f64 {
    let scale = mujoco.iter().fold(0.0_f64, |m, v| m.max(v.abs()));
    let gap = ours.iter().zip(mujoco).fold(0.0_f64, |m, (a, b)| {
        let d = (a - b).abs();
        if m.is_nan() || d.is_nan() {
            f64::NAN
        } else {
            m.max(d)
        }
    });
    if scale > 0.0 { gap / scale } else { gap }
}

#[test]
fn the_comparison_fails_a_nan_and_compares_against_zero_absolutely() {
    assert!(relative_gap(&[1.0, f64::NAN], &[1.0, 2.0]).is_nan());
    assert!(relative_gap(&[f64::NAN, 1.0], &[1.0, 2.0]).is_nan());
    assert_eq!(relative_gap(&[0.0, 1e-20], &[0.0, 0.0]), 1e-20);
    assert_eq!(relative_gap(&[1.0, 3.0], &[2.0, 4.0]), 0.25);
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
// inverse, then scales it (`ApplyTransformations`, `user_mesh.cc:1440-1505`),
// before its mass properties, so both move the body's inertial frame.

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
        0.0,
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
        0.0,
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
        0.0,
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

/// The L-shape of `mesh_inertia_modes.rs`, a bar from (0, 0, 0) to (3, 1, 1)
/// and a column from (0, 1, 0) to (1, 3, 1), with `attrs` on its asset, on a
/// free body.
fn l_shape(attrs: &str) -> String {
    format!(
        r#"<mujoco><asset><mesh name="l" {attrs} vertex="0 0 0  3 0 0  3 1 0  1 1 0  1 3 0  0 3 0  0 0 1  3 0 1  3 1 1  1 1 1  1 3 1  0 3 1" face="0 2 1  0 3 2  0 4 3  0 5 4  6 7 8  6 8 9  6 9 10  6 10 11  0 1 7  0 7 6  1 2 8  1 8 7  2 3 9  2 9 8  3 4 10  3 10 9  4 5 11  4 11 10  5 0 6  5 6 11"/></asset><worldbody><body><freejoint/><geom type="mesh" mesh="l"/></body></worldbody></mujoco>"#
    )
}

// MuJoCo's mass properties (`mjCMesh::Process`): the volume and centre of
// mass from pyramids with their apex at the centre of the faces, the
// inertia from pyramids with their apex at the centre of mass.

#[test]
fn legacy_mode_takes_each_pyramids_volume_positive_from_the_faces_centre() {
    // Not 5000, the L-shape's volume times its density: the centre of its
    // faces, (1.136, 1.136, 0.5), lies in its notch, so four of its twenty
    // pyramids have a negative volume, which legacy counts positive.
    assert_body(
        &l_shape(r#"inertia="legacy""#),
        (
            5322.547508988188,
            [1.1209553158705703, 1.1209553158705703, 0.49999999999999994],
            [
                0.2705980500730991,
                0.6532814824381896,
                -0.2705980500730991,
                0.6532814824381896,
            ],
            [7454.608707160513, 5978.479712378018, 2341.7170790146465],
        ),
        0.0,
    );
}

#[test]
fn exact_mode_is_mujocos() {
    assert_body(
        &l_shape(r#"inertia="exact""#),
        (
            4999.999999999999,
            [1.1, 1.1, 0.5],
            [
                0.2705980500730989,
                0.6532814824381892,
                -0.2705980500730989,
                0.6532814824381892,
            ],
            [7233.333333333335, 5833.333333333332, 2233.3333333333344],
        ),
        0.0,
    );
}

#[test]
fn shell_mode_is_mujocos() {
    assert_body(
        &l_shape(r#"inertia="shell""#),
        (
            22000.0,
            [1.1363636363636362, 1.1363636363636365, 0.5],
            [
                0.2705980500730991,
                0.6532814824381896,
                -0.2705980500730991,
                0.6532814824381896,
            ],
            [39848.48484848484, 32333.33333333333, 14515.151515151518],
        ),
        0.0,
    );
}

#[test]
fn convex_mode_is_mujocos_to_rounding() {
    // Over the hull's faces: this crate's hull equals MuJoCo's as a set, not
    // in order or triangulation (book Rigid-loading L36), so not bit for bit.
    assert_body(
        &l_shape(r#"inertia="convex""#),
        (
            7000.0,
            [1.2619047619047619, 1.2619047619047619, 0.5],
            [
                0.27059805007309906,
                0.6532814824381896,
                -0.27059805007309906,
                0.6532814824381896,
            ],
            [9039.682539682539, 6666.666666666664, 3539.6825396825384],
        ),
        1e-14,
    );
}

#[test]
fn an_open_mesh_under_legacy_mode_is_mujocos() {
    // The cube without its top: the apex at the centre of its five faces.
    assert_body(
        &cube_mesh(
            "",
            "0 2 1  0 3 2  0 1 5  0 5 4  2 3 7  2 7 6  0 4 7  0 7 3  1 2 6  1 6 5",
            r#"inertia="legacy""#,
        ),
        (
            804.1666666666667,
            [0.0, 0.0, -0.08750000000000001],
            [
                0.5000000000000007,
                0.5000000000000007,
                -0.5000000000000007,
                0.5000000000000007,
            ],
            [147.0833333333333, 123.97539062499999, 123.97539062499999],
        ),
        0.0,
    );
}

#[test]
fn a_mesh_with_no_volume_is_refused() {
    // One triangle four times over (ledger L384).
    let xml = r#"<mujoco><asset><mesh name="m" vertex="0 0 0 1 0 0 0 1 0 0 0 1" face="0 1 2 0 1 2 0 1 2 0 1 2"/></asset><worldbody><body><freejoint/><geom type="mesh" mesh="m"/></body></worldbody></mujoco>"#;
    let err = load_model(xml).unwrap_err().to_string();
    assert!(
        err.contains("mesh volume is too small. Try setting inertia to shell"),
        "{err}"
    );
}

#[test]
fn a_mesh_whose_faces_have_no_area_is_refused() {
    let xml = r#"<mujoco><asset><mesh name="m" vertex="0 0 0  1 0 0  0 1 0  0 0 1" face="0 1 1  0 2 2  0 3 3  1 2 2"/></asset><worldbody><body><freejoint/><geom type="mesh" mesh="m" contype="0" conaffinity="0"/></body></worldbody></mujoco>"#;
    let err = load_model(xml).unwrap_err().to_string();
    assert!(err.contains("mesh surface area is too small"), "{err}");
}

#[test]
fn a_written_mass_on_a_mesh_of_tiny_volume_is_kept() {
    // Volume 1e-12, above mjEPS: MuJoCo keeps the mass (ledger L43, RA-11).
    let tiny = "-5e-05 -5e-05 -5e-05 5e-05 -5e-05 -5e-05 5e-05 5e-05 -5e-05 -5e-05 5e-05 -5e-05 -5e-05 -5e-05 5e-05 5e-05 -5e-05 5e-05 5e-05 5e-05 5e-05 -5e-05 5e-05 5e-05";
    let xml = format!(
        r#"<mujoco><asset><mesh name="m" vertex="{tiny}" face="{CUBE_FACES}"/></asset><worldbody><body><freejoint/><geom type="mesh" mesh="m" mass="1"/></body></worldbody></mujoco>"#
    );
    assert_body(
        &xml,
        (
            1.0,
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0, 0.0],
            [
                1.6666665824595854e-09,
                1.6666665824595854e-09,
                1.6666665824595854e-09,
            ],
        ),
        0.0,
    );
}

#[test]
fn a_mesh_with_no_inertia_attribute_is_legacy() {
    // MuJoCo 3.5.0's default mode (`user_init.c:242`), not convex (7000).
    assert_body(
        &l_shape(""),
        (
            5322.547508988188,
            [1.1209553158705703, 1.1209553158705703, 0.49999999999999994],
            [
                0.2705980500730991,
                0.6532814824381896,
                -0.2705980500730991,
                0.6532814824381896,
            ],
            [7454.608707160513, 5978.479712378018, 2341.7170790146465],
        ),
        0.0,
    );
}
