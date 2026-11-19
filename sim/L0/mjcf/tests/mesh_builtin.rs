//! MuJoCo 3.5.0's built-in meshes (book Rigid-loading L36; ledger L407):
//! `<mesh builtin="…" params="…"/>` read as MuJoCo's reader reads it, which
//! has `mjs_makeMesh` make the mesh's vertices and faces, and the mesh then
//! loaded as one written with them. The generators themselves are compared
//! with MuJoCo's, case by case, in `parser/mesh_builtin.rs`.
//!
//! Every expected value was printed by MuJoCo 3.5.0 on the census's unfused
//! oracle (2026-11-18).

// The values are MuJoCo's, printed to the last digit, and compared to it.
#![allow(
    clippy::unwrap_used,
    clippy::unreadable_literal,
    clippy::float_cmp,
    clippy::approx_constant
)]

use sim_mjcf::{load_model, parse_mjcf_str};

/// A mesh asset `m` with `mesh` attributes, on a free body's geom with
/// `geom` attributes.
fn builtin(mesh: &str, geom: &str) -> String {
    format!(
        r#"<mujoco><asset><mesh name="m" {mesh}/></asset><worldbody><body name="b"><freejoint/><geom type="mesh" mesh="m" {geom}/></body></worldbody></mujoco>"#
    )
}

/// Body 1's mass, centre of mass, inertial frame (w, x, y, z) and principal
/// moments; its mesh's vertex and face counts.
type Body = (f64, [f64; 3], [f64; 4], [f64; 3]);

fn body_and_counts(xml: &str) -> (Body, (usize, usize)) {
    let model = load_model(xml).unwrap();
    let q = model.body_iquat[1];
    let mesh = &model.mesh_data[0];
    (
        (
            model.body_mass[1],
            model.body_ipos[1].into(),
            [q.w, q.i, q.j, q.k],
            model.body_inertia[1].into(),
        ),
        (mesh.vertex_count(), mesh.triangle_count()),
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

/// 0 on macOS arm64, where MuJoCo's values were printed, and 1e-12
/// elsewhere, for a value whose last bit a platform's libm may turn.
const LIBM_TOLERANCE: f64 = if cfg!(all(target_os = "macos", target_arch = "aarch64")) {
    0.0
} else {
    1e-12
};

/// `xml`'s body against MuJoCo's, and its mesh's counts.
fn assert_loads_as_mujoco(xml: &str, mujoco: Body, counts: (usize, usize)) {
    let (ours, our_counts) = body_and_counts(xml);
    assert_eq!(our_counts, counts, "{xml}");
    let gaps = [
        relative_gap(&[ours.0], &[mujoco.0]),
        relative_gap(&ours.1, &mujoco.1),
        relative_gap(&ours.2, &mujoco.2),
        relative_gap(&ours.3, &mujoco.3),
    ];
    assert!(
        gaps.iter().all(|&gap| gap <= LIBM_TOLERANCE),
        "{xml}: {ours:?} against MuJoCo's {mujoco:?}: gaps {gaps:?}"
    );
}

/// A sphere (icosahedral, subdivided) and a hemisphere, each a mesh
/// `mjs_makeMesh` makes, load with MuJoCo's mass, centre, frame and moments,
/// under a scale, a reference frame, another inertia mode and a geom's mass
/// too.
#[test]
fn a_builtin_mesh_loads_as_mujoco_loads_it() {
    for (mesh, geom, mujoco, counts) in BUILTIN_LOADS {
        assert_loads_as_mujoco(&builtin(mesh, geom), mujoco, counts);
    }
}

/// [`a_builtin_mesh_loads_as_mujoco_loads_it`]'s cases: a mesh's attributes,
/// its geom's, MuJoCo's body and the mesh's vertex and face counts.
const BUILTIN_LOADS: [(&str, &str, Body, (usize, usize)); 14] = [
    (
        r#"builtin="sphere" params="0""#,
        "",
        (
            2536.1504726624075,
            [
                5.4719891258052174e-18,
                -1.7783964658866957e-17,
                -5.4719891258052174e-18,
            ],
            [1.0, 0.0, 0.0, 0.0],
            [734.0702430338127, 734.0702430338127, 734.0702430338127],
        ),
        (12, 20),
    ),
    (
        r#"builtin="sphere" params="1""#,
        "",
        (
            3658.712126251761,
            [
                -1.3275780015278598e-17,
                9.00856501036762e-18,
                -1.1379240013095942e-17,
            ],
            [1.0, 0.0, 0.0, 0.0],
            [1338.326589578931, 1338.3265895789311, 1338.326589578931],
        ),
        (42, 80),
    ),
    (
        r#"builtin="sphere" params="2""#,
        "",
        (
            4047.0446616510244,
            [
                5.958089997020361e-17,
                -1.9288780565893255e-17,
                7.710178748443807e-18,
            ],
            [
                0.5000000000000007,
                0.5000000000000007,
                -0.5000000000000007,
                0.5000000000000007,
            ],
            [1582.1694633536251, 1582.1694526509998, 1582.1694513676375],
        ),
        (162, 320),
    ),
    (
        r#"builtin="sphere" params="3""#,
        "",
        (
            4152.740778128605,
            [
                1.4620542167948428e-16,
                9.346703743081318e-18,
                5.168178057460992e-17,
            ],
            [0.0, 0.7071067811865489, 0.0, 0.7071067811865489],
            [1651.557421303255, 1651.5573942570534, 1651.557391414529],
        ),
        (642, 1280),
    ),
    (
        r#"builtin="sphere" params="4""#,
        "",
        (
            4179.738957796405,
            [
                1.1164348294224054e-16,
                2.5641167027975084e-16,
                1.638001865080816e-16,
            ],
            [0.707106781186548, 0.0, 0.0, 0.707106781186548],
            [1669.4866010399317, 1669.486598843994, 1669.486591552173],
        ),
        (2562, 5120),
    ),
    (
        r#"builtin="hemisphere" params="0""#,
        "",
        (
            666.6666666666665,
            [0.0, 0.0, 0.25],
            [
                0.5000000000000007,
                0.5000000000000007,
                -0.5000000000000007,
                0.5000000000000007,
            ],
            [133.33333333333334, 91.66666666666669, 91.66666666666669],
        ),
        (6, 8),
    ),
    (
        r#"builtin="hemisphere" params="3""#,
        "",
        (
            1909.7436909829885,
            [
                1.816708162637162e-18,
                5.22303596758184e-18,
                0.36260610739429655,
            ],
            [
                0.5000000000000007,
                0.5000000000000007,
                -0.5000000000000007,
                0.5000000000000007,
            ],
            [718.7665140993022, 467.6673232077385, 467.6673232077385],
        ),
        (42, 80),
    ),
    (
        r#"builtin="hemisphere" params="10""#,
        "",
        (
            2068.308741012736,
            [
                3.145184713442631e-18,
                -6.2641595542732396e-18,
                0.3732706675536752,
            ],
            [
                0.5000000000000007,
                0.5000000000000007,
                -0.5000000000000007,
                0.5000000000000007,
            ],
            [820.4497973168873, 532.2702902082029, 532.2702902082029],
        ),
        (266, 528),
    ),
    (
        r#"builtin="sphere" params="2" inertia="exact""#,
        "",
        (
            4047.0446616510244,
            [
                5.958089997020361e-17,
                -1.9288780565893255e-17,
                7.710178748443807e-18,
            ],
            [
                0.5000000000000007,
                0.5000000000000007,
                -0.5000000000000007,
                0.5000000000000007,
            ],
            [1582.1694633536251, 1582.1694526509998, 1582.1694513676375],
        ),
        (162, 320),
    ),
    (
        r#"builtin="sphere" params="2" inertia="shell""#,
        "",
        (
            12329.848552323809,
            [
                2.7575829487976417e-17,
                8.132055838801209e-17,
                1.3923260983838267e-17,
            ],
            [
                0.5000000000000007,
                0.5000000000000007,
                -0.5000000000000007,
                0.5000000000000007,
            ],
            [8033.79075581463, 8033.790712074468, 8033.790707804819],
        ),
        (162, 320),
    ),
    (
        r#"builtin="sphere" params="2" scale="1 2 3""#,
        "",
        (
            24282.267969906206,
            [
                -4.715035249440562e-17,
                -1.4230833661947878e-16,
                -1.0715775195555041e-16,
            ],
            [1.0, 0.0, 0.0, 0.0],
            [61704.60847359922, 47465.08328416618, 23732.54196185462],
        ),
        (162, 320),
    ),
    (
        r#"builtin="sphere" params="2" scale="-1 1 1""#,
        "",
        (
            4047.0446616510244,
            [
                -5.958089997020361e-17,
                -1.9288780565893255e-17,
                7.710178748443807e-18,
            ],
            [
                0.5000000000000007,
                0.5000000000000007,
                -0.5000000000000007,
                0.5000000000000007,
            ],
            [1582.1694633536251, 1582.1694526509998, 1582.1694513676375],
        ),
        (162, 320),
    ),
    (
        r#"builtin="hemisphere" params="2" refpos="0.1 0.2 0.3" refquat="0.9 0.1 0.2 0.3""#,
        "",
        (
            1785.6405784234898,
            [
                -0.21187753101689136,
                -0.0881224689831085,
                0.01698633788119251,
            ],
            [
                0.4774750135451681,
                0.3378207728928719,
                -0.4684731225315864,
                0.6621387092653716,
            ],
            [643.1733906578007, 419.0513878176552, 419.0513878176552],
        ),
        (26, 48),
    ),
    (
        r#"builtin="hemisphere" params="4""#,
        r#"mass="2.5""#,
        (
            2.5,
            [
                -5.275339927447311e-18,
                -1.2309126497377061e-17,
                0.36689708879090543,
            ],
            [
                0.5000000000000007,
                0.5000000000000007,
                -0.5000000000000007,
                0.5000000000000007,
            ],
            [0.961243441703218, 0.6247097572951137, 0.6247097572951137],
        ),
        (62, 120),
    ),
];

// A plate's points are coplanar, so MuJoCo's qhull refuses its hull, which
// MuJoCo builds for a mesh geom that can collide (`user_model.cc:4905-4922`).
// This crate builds no hull for it (ledger L43 (h)), so the plates here are
// on geoms that cannot collide.

/// `MakeRect` makes a plate a shell, whatever `inertia` is written on it or
/// on its class: MuJoCo reads `inertia` before `builtin`
/// (`xml_native_reader.cc:1571-1573`, `user_mesh.cc:2501`).
#[test]
fn a_plate_is_a_shell_whatever_inertia_is_written() {
    let six_by_two: Body = (
        1666.6666269302368,
        [-1.665334576642405e-17, 0.0, -1.0],
        [0.0, 0.7071067811865489, 0.0, 0.7071067811865489],
        [524.69132711858, 385.80244154106026, 138.88888557751974],
    );
    for written in [
        "",
        r#"inertia="exact""#,
        r#"inertia="legacy""#,
        r#"inertia="convex""#,
    ] {
        let xml = builtin(
            &format!(r#"builtin="plate" params="6 2" {written}"#),
            r#"contype="0" conaffinity="0""#,
        );
        assert_loads_as_mujoco(&xml, six_by_two, (12, 10));
    }
    let class_exact = r#"<mujoco><default><default class="c"><mesh inertia="exact"/></default></default><asset><mesh name="m" class="c" builtin="plate" params="3 3"/></asset><worldbody><body name="b"><freejoint/><geom type="mesh" mesh="m" contype="0" conaffinity="0"/></body></worldbody></mujoco>"#;
    assert_loads_as_mujoco(
        class_exact,
        (
            1777.7778837415922,
            [-7.806255176606557e-18, 7.806255176606557e-18, -1.0],
            [
                0.5000000000000007,
                0.5000000000000007,
                -0.5000000000000007,
                0.5000000000000007,
            ],
            [526.7490339867891, 263.37451699339454, 263.37451699339454],
        ),
        (9, 8),
    );
}

/// A written `face` is replaced by the faces a generator makes, a plate's
/// too, and kept beside a cone's vertices, which come with none: MuJoCo
/// refuses only written vertices beside `builtin`. A face a sphere replaces
/// is not checked, as MuJoCo checks a face only when it compiles the mesh.
#[test]
fn a_written_face_is_kept_by_a_cone_and_replaced_by_a_sphere_or_a_plate() {
    let sphere0: Body = (
        2536.1504726624075,
        [
            5.4719891258052174e-18,
            -1.7783964658866957e-17,
            -5.4719891258052174e-18,
        ],
        [1.0, 0.0, 0.0, 0.0],
        [734.0702430338127, 734.0702430338127, 734.0702430338127],
    );
    for face in ["0 1 2", "0 -1 2"] {
        let xml = builtin(&format!(r#"builtin="sphere" params="0" face="{face}""#), "");
        assert_loads_as_mujoco(&xml, sphere0, (12, 20));
    }
    assert_loads_as_mujoco(
        &builtin(
            r#"builtin="plate" params="2 2" face="0 1 2""#,
            r#"contype="0" conaffinity="0""#,
        ),
        (
            1000.0,
            [0.0, 0.0, -1.0],
            [
                0.5000000000000007,
                0.5000000000000007,
                -0.5000000000000007,
                0.5000000000000007,
            ],
            [166.66666666666666, 83.33333333333333, 83.33333333333333],
        ),
        (4, 2),
    );
    let cone = builtin(r#"builtin="cone" params="4 0.5" face="0 1 4 1 2 5""#, "");
    assert_eq!(
        parse_mjcf_str(&cone).unwrap().meshes[0].face,
        Some(vec![0, 1, 4, 1, 2, 5])
    );
    assert_loads_as_mujoco(
        &cone,
        (
            269.44444444444446,
            [0.0208333333333334, 0.42916666666666675, -0.3333333333333333],
            [
                0.6819386202249729,
                0.34025231548499907,
                0.21642903000723993,
                0.6102020608150098,
            ],
            [73.50472966044046, 49.69364418076206, 38.87905671435298],
        ),
        (8, 2),
    );
}

/// A built-in mesh that makes no faces, or no vertices, is a mesh written
/// without them: a plate one point wide replaces a written face with none
/// and is refused as a mesh with no faces, which MuJoCo refuses for its hull
/// ("qhull error"); a cone of no edges is refused as MuJoCo refuses it, for
/// its count of vertices.
#[test]
fn a_builtin_mesh_that_makes_no_faces_or_no_vertices_is_one_written_without_them() {
    for (mesh, refusal) in [
        (
            r#"builtin="plate" params="1 4" face="0 1 2""#,
            "embedded vertex data requires face data",
        ),
        (
            r#"builtin="cone" params="0 0.5""#,
            "mesh 'm': at least 4 vertices required",
        ),
    ] {
        let err = load_model(&builtin(mesh, "")).unwrap_err().to_string();
        assert!(err.contains(refusal), "{mesh}: {err}");
    }
}

/// What MuJoCo's reader refuses about a built-in mesh, in its order: the
/// keyword, `params` (required), a `file` beside it, written vertices beside
/// it, then `mjs_makeMesh`'s count and range checks. Each in MuJoCo's words
/// but the keyword's, which this crate words as it words every keyword's
/// (MuJoCo: "invalid keyword: 'cube'").
#[test]
fn a_builtin_mesh_is_refused_as_mujoco_refuses_it() {
    let cases = [
        (
            r#"builtin="cube" params="1""#,
            "invalid keyword \"cube\" in attribute 'builtin'",
        ),
        (
            r#"builtin="cube""#,
            "invalid keyword \"cube\" in attribute 'builtin'",
        ),
        (
            r#"builtin="sphere""#,
            "required attribute missing: 'params'",
        ),
        (
            r#"builtin="sphere" params="""#,
            "required attribute missing: 'params'",
        ),
        (
            r#"builtin="sphere" file="none.obj""#,
            "required attribute missing: 'params'",
        ),
        (
            r#"builtin="sphere" params="5" file="none.obj""#,
            "builtin cannot be used with a mesh file",
        ),
        (
            r#"builtin="sphere" params="1" file="none.obj" vertex="0 0 0 1 0 0 0 1 0 0 0 1""#,
            "builtin cannot be used with a mesh file",
        ),
        (
            r#"builtin="sphere" params="5" vertex="0 0 0 1 0 0 0 1 0 0 0 1""#,
            "builtin mesh cannot be used with user vertex data",
        ),
        (r#"builtin="none" params="1""#, "Unsupported mesh type"),
        (
            r#"builtin="sphere" params="7 1""#,
            "Sphere mesh type requires 1 parameter",
        ),
        (
            r#"builtin="sphere" params="5""#,
            "Sphere subdivision cannot be greater than 4",
        ),
        (
            r#"builtin="hemisphere" params="-1""#,
            "Hemisphere resolution cannot be negative",
        ),
        (
            r#"builtin="supertorus" params="8 0 1 1""#,
            "Supertorus radius must be in (0, 1]",
        ),
        (
            r#"builtin="wedge" params="4 4 90 91 0""#,
            "`fov[1]` must be a float between (0, 90] degrees",
        ),
        (
            r#"builtin="plate" params="0 2""#,
            "Horizontal and vertical resolutions must be positive",
        ),
    ];
    for (mesh, refusal) in cases {
        let err = load_model(&builtin(mesh, "")).unwrap_err().to_string();
        assert!(err.contains(refusal), "{mesh}: {err}");
    }
}
