//! A body's inertia from its geoms, as MuJoCo 3.5.0's `InertiaFromGeom`
//! gives it (book Rigid-loading L33; `user_objects.cc:2158-2224`): each geom
//! in its frame as its body holds it, its orientation alternative included.
//!
//! Every expected value was printed by MuJoCo 3.5.0 on the census's unfused
//! oracle (2026-11-09).

// The values are MuJoCo's, printed to the last digit.
#![allow(
    clippy::unwrap_used,
    clippy::panic,
    clippy::unreadable_literal,
    clippy::approx_constant
)]

use nalgebra::{Matrix3, Quaternion, UnitQuaternion, Vector3};
use sim_mjcf::load_model;

/// A body's inertia tensor about its centre of mass, in its own frame's
/// axes: its principal moments turned by its inertial frame.
fn tensor(moments: [f64; 3], iquat: [f64; 4]) -> Matrix3<f64> {
    let [qw, qx, qy, qz] = iquat;
    let turn =
        UnitQuaternion::from_quaternion(Quaternion::new(qw, qx, qy, qz)).to_rotation_matrix();
    turn.matrix() * Matrix3::from_diagonal(&Vector3::from(moments)) * turn.matrix().transpose()
}

/// Body 1's tensor, as [`tensor`] makes it.
fn body_tensor(xml: &str) -> Matrix3<f64> {
    let model = load_model(xml).unwrap();
    let q = model.body_iquat[1];
    tensor(model.body_inertia[1].into(), [q.w, q.i, q.j, q.k])
}

#[test]
fn geom_euler_rotates_body_inertia() {
    // A box turned 45 degrees about z beside a sphere.
    let ours = body_tensor(
        r#"<mujoco><worldbody><body name="b"><freejoint/><geom type="box" size="0.1 0.2 0.3" euler="0 0 45"/><geom type="sphere" size="0.05" pos="0.3 0 0"/></body></worldbody></mujoco>"#,
    );
    let mujoco = tensor(
        [2.1049604083525657, 1.622702183927395, 0.8471389935043618],
        [0.914347784560959, 0.0, 0.0, 0.40492978263949364],
    );
    let gap = (ours - mujoco).abs().max() / mujoco.abs().max();
    assert!(gap <= 1e-15, "{ours} against MuJoCo's {mujoco}: {gap:e}");
}

/// A 0.4 × 0.2 × 0.1 box mesh from the origin, its geom turned 30 degrees
/// about z.
const TURNED_BOX_MESH: &str = r#"<mujoco><asset><mesh name="bx" vertex="0 0 0  0.4 0 0  0.4 0.2 0  0 0.2 0  0 0 0.1  0.4 0 0.1  0.4 0.2 0.1  0 0.2 0.1" face="0 2 1  0 3 2  4 5 6  4 6 7  0 1 5  0 5 4  1 2 6  1 6 5  2 3 7  2 7 6  3 0 4  3 4 7"/></asset><worldbody><body name="b"><freejoint/><geom type="mesh" mesh="bx" euler="0 0 30"/></body></worldbody></mujoco>"#;

/// The largest of `ours - mujoco`, relative to the largest of `mujoco`.
fn relative_gap(ours: &[f64], mujoco: &[f64]) -> f64 {
    let scale = mujoco.iter().fold(0.0_f64, |m, v| m.max(v.abs()));
    ours.iter()
        .zip(mujoco)
        .fold(0.0_f64, |m, (a, b)| m.max((a - b).abs()))
        / scale
}

#[test]
fn a_turned_mesh_geom_turns_its_bodys_centre_of_mass() {
    // MuJoCo's centre of mass is the mesh's, turned.
    let model = load_model(TURNED_BOX_MESH).unwrap();
    let mujoco = Vector3::new(0.12320508259278645, 0.1866025431590383, 0.05000000074505804);
    // MuJoCo holds a mesh's vertices in single precision (ledger L281, book
    // L35), so the centre of mass is compared within single precision.
    let gap = (model.body_ipos[1] - mujoco).abs().max() / mujoco.abs().max();
    assert!(
        gap <= 1e-7,
        "{} against MuJoCo's {mujoco}: {gap:e}",
        model.body_ipos[1]
    );
}

#[test]
fn a_turned_mesh_geom_gives_its_body_the_meshs_principal_frame_turned() {
    // MuJoCo's frame is the geom's turn composed with the mesh's principal
    // frame (`user_mesh.cc:1640-1686`), whose moments `eig3` orders largest
    // first. Compared within single precision, as the centre of mass is.
    let model = load_model(TURNED_BOX_MESH).unwrap();
    let q = model.body_iquat[1];
    let iquat = [q.w, q.i, q.j, q.k];
    let mujoco_iquat = [
        -0.18301270189221966,
        0.6830127018922206,
        0.18301270189221966,
        0.6830127018922206,
    ];
    let gap = relative_gap(&iquat, &mujoco_iquat);
    assert!(
        gap <= 1e-7,
        "{iquat:?} against MuJoCo's {mujoco_iquat:?}: {gap:e}"
    );
    let inertia: [f64; 3] = model.body_inertia[1].into();
    let mujoco_inertia = [0.1333333432674411, 0.11333334177732492, 0.03333333581686027];
    let gap = relative_gap(&inertia, &mujoco_inertia);
    assert!(
        gap <= 1e-7,
        "{inertia:?} against MuJoCo's {mujoco_inertia:?}: {gap:e}"
    );
}

/// One body's inertial as MuJoCo 3.5.0 compiles it.
struct Body {
    mass: f64,
    inertia: [f64; 3],
    ipos: [f64; 3],
    iquat: [f64; 4],
}

/// What loading a document gives.
enum Expect {
    /// MuJoCo refuses it, with these words.
    Refused(&'static str),
    /// MuJoCo loads it with these bodies, the world's left out.
    Loads(&'static [Body]),
}

/// Each case loaded as MuJoCo loads it: each body's mass, moments, centre
/// and frame bit for bit, or MuJoCo's refusal.
fn check(cases: &[(&str, &str, Expect)]) {
    for (name, xml, expected) in cases {
        let loaded = load_model(xml);
        match expected {
            Expect::Refused(words) => {
                let error = loaded.err().map(|e| e.to_string()).unwrap_or_default();
                assert!(
                    error.contains(words),
                    "{name}: expected '{words}', got '{error}'"
                );
            }
            Expect::Loads(bodies) => {
                let model = loaded.unwrap_or_else(|e| panic!("{name}: {e}"));
                assert_eq!(model.nbody, bodies.len() + 1, "{name}: nbody");
                for (k, want) in bodies.iter().enumerate() {
                    let b = k + 1;
                    let q = model.body_iquat[b];
                    let ours: Vec<u64> = std::iter::once(model.body_mass[b])
                        .chain(model.body_inertia[b].iter().copied())
                        .chain(model.body_ipos[b].iter().copied())
                        .chain([q.w, q.i, q.j, q.k])
                        .map(f64::to_bits)
                        .collect();
                    let mujoco: Vec<u64> = std::iter::once(want.mass)
                        .chain(want.inertia)
                        .chain(want.ipos)
                        .chain(want.iquat)
                        .map(f64::to_bits)
                        .collect();
                    assert_eq!(
                        ours, mujoco,
                        "{name}: body {b} mass, inertia, ipos, iquat {:?} {:?} {:?} {:?}",
                        model.body_mass[b], model.body_inertia[b], model.body_ipos[b], q
                    );
                }
            }
        }
    }
}

/// `(name, document, expected)`, generated from MuJoCo 3.5.0's results on
/// the census's unfused oracle (2026-11-09): MuJoCo's geom selection, the
/// copy of one geom, and several geoms' principal axes by `eig3`.
const FROM_GEOMS: &[(&str, &str, Expect)] = &[
    (
        "single_geom_body_copies_geom_frame",
        r#"<mujoco><worldbody><body name="b" pos="0.1 0.2 0.3"><freejoint/><geom type="box" size="0.1 0.2 0.3" pos="0.05 -0.02 0.01" euler="10 20 30"/></body></worldbody></mujoco>"#,
        Expect::Loads(&[Body {
            mass: 48.00000000000001,
            inertia: [2.0800000000000005, 1.6000000000000003, 0.8000000000000003],
            ipos: [0.05, -0.02, 0.01],
            iquat: [
                0.943714364147489,
                0.12767944069578063,
                0.14487812541736914,
                0.2685358227515692,
            ],
        }]),
    ),
    (
        "one_turned_capsule_copied",
        r#"<mujoco><worldbody><body name="cap" pos="0 0 0.65"><freejoint/><geom type="capsule" size="0.2 0.4" euler="0 90 0"/></body></worldbody></mujoco>"#,
        Expect::Loads(&[Body {
            mass: 134.04128655316455,
            inertia: [14.275397017912027, 14.275397017912027, 2.546784444510126],
            ipos: [0.0, 0.0, 0.0],
            iquat: [0.7071067811865476, 0.0, 0.7071067811865475, 0.0],
        }]),
    ),
    (
        "geom_euler_rotates_body_inertia",
        r#"<mujoco><worldbody><body name="b"><freejoint/><geom type="box" size="0.1 0.2 0.3" euler="0 0 45"/><geom type="sphere" size="0.05" pos="0.3 0 0"/></body></worldbody></mujoco>"#,
        Expect::Loads(&[Body {
            mass: 48.52359877559831,
            inertia: [2.1049604083525657, 1.622702183927395, 0.8471389935043618],
            ipos: [0.0032371801894974525, 0.0, 0.0],
            iquat: [0.914347784560959, 0.0, 0.0, 0.40492978263949364],
        }]),
    ),
    (
        "a_box_turned_about_a_general_axis_beside_a_sphere",
        r#"<mujoco><worldbody><body name="b"><freejoint/><geom type="box" size="0.1 0.2 0.3" euler="35 50 70"/><geom type="sphere" size="0.05" pos="0.3 0 0"/></body></worldbody></mujoco>"#,
        Expect::Loads(&[Body {
            mass: 48.52359877559831,
            inertia: [2.1250074505900955, 1.630633059754131, 0.8191610754400932],
            ipos: [0.0032371801894974525, 0.0, 0.0],
            iquat: [
                0.6248478408227656,
                0.4621161630469068,
                0.1809599376527008,
                0.6027166238247172,
            ],
        }]),
    ),
    (
        "two_spheres_principal_axes_in_eig3s_order",
        r#"<mujoco><worldbody><body name="b1" pos="0 0 1"><freejoint/><geom type="sphere" size="0.05" pos="0.3 0.1 0" mass="1"/><geom type="sphere" size="0.05" pos="-0.1 0.2 0" mass="0.5"/></body></worldbody></mujoco>"#,
        Expect::Loads(&[Body {
            mass: 1.5,
            inertia: [
                0.058166666666666665,
                0.05816666666666667,
                0.0014999999999999996,
            ],
            ipos: [0.16666666666666666, 0.13333333333333333, 0.0],
            iquat: [
                0.5573454101893046,
                0.5573454101893046,
                0.43516214649359986,
                0.43516214649359986,
            ],
        }]),
    ),
    (
        "a_box_and_two_spheres_principal_axes",
        r#"<mujoco><worldbody><body name="b"><freejoint/><geom type="box" size="0.1 0.2 0.3"/><geom type="sphere" size="0.05" pos="0.2 0.1 0"/><geom type="sphere" size="0.07" pos="-0.1 0 0.2"/></body></worldbody></mujoco>"#,
        Expect::Loads(&[Body {
            mass: 49.960353815840044,
            inertia: [2.1452077183506115, 1.6941938223489272, 0.843176812330064],
            ipos: [
                -0.0007797332470484317,
                0.0010480285578607948,
                0.005751580725540042,
            ],
            iquat: [
                0.9998721247825223,
                0.00043973195098815846,
                -0.010952167485772305,
                -0.011644343951418024,
            ],
        }]),
    ),
    (
        "a_height_field_beside_a_box",
        r#"<mujoco><asset><hfield name="h" size="1 1 1 0.5" nrow="2" ncol="2" elevation="0 0.5 1 0.25"/></asset><worldbody><body name="b" pos="0 0 1"><freejoint name="f"/><geom type="hfield" hfield="h"/><geom type="box" size="0.1 0.2 0.3" pos="0.3 0 0.2"/></body></worldbody></mujoco>"#,
        Expect::Loads(&[Body {
            mass: 4048.0,
            inertia: [2671.7435313353417, 1674.4326745718051, 1670.6358099031306],
            ipos: [0.0035573122529644276, 0.0, 0.0023715415019762848],
            iquat: [0.0, 0.7061010153060066, 0.0, 0.7081111185286042],
        }]),
    ),
    (
        "a_fromto_capsule_copied",
        r#"<mujoco><worldbody><body name="p"><joint type="hinge" axis="0 1 0"/><geom type="capsule" size="0.02" fromto="0 0 0 0.3 0 0"/></body></worldbody></mujoco>"#,
        Expect::Loads(&[Body {
            mass: 0.4105014400690663,
            inertia: [
                0.0036998746120837237,
                0.0036998746120837237,
                8.075987514828162e-05,
            ],
            ipos: [0.15, 0.0, 0.0],
            iquat: [0.7071067811865476, 0.0, -0.7071067811865475, 0.0],
        }]),
    ),
    (
        "true_keeps_an_inertial_where_no_geom_is_selected",
        r#"<mujoco><compiler inertiafromgeom="true"/><worldbody><body name="b"><freejoint/><inertial pos="0.1 0 0" mass="9" diaginertia="1 1 1"/></body></worldbody></mujoco>"#,
        Expect::Loads(&[Body {
            mass: 9.0,
            inertia: [1.0, 1.0, 1.0],
            ipos: [0.1, 0.0, 0.0],
            iquat: [1.0, 0.0, 0.0, 0.0],
        }]),
    ),
    (
        "true_keeps_an_inertial_beside_a_geom_of_mass_0",
        r#"<mujoco><compiler inertiafromgeom="true"/><worldbody><body name="b"><freejoint/><inertial pos="0.1 0 0" mass="9" diaginertia="1 1 1"/><geom type="sphere" size="0.1" mass="0"/></body></worldbody></mujoco>"#,
        Expect::Loads(&[Body {
            mass: 9.0,
            inertia: [1.0, 1.0, 1.0],
            ipos: [0.1, 0.0, 0.0],
            iquat: [1.0, 0.0, 0.0, 0.0],
        }]),
    ),
    (
        "true_refuses_a_kept_inertial_that_breaks_the_triangle",
        r#"<mujoco><compiler inertiafromgeom="true"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="1 1 3"/></body></worldbody></mujoco>"#,
        Expect::Refused("inertia must satisfy A + B >= C; use 'balanceinertia' to fix"),
    ),
    (
        "true_refuses_a_kept_inertial_beside_a_plane_that_breaks_the_triangle",
        r#"<mujoco><compiler inertiafromgeom="true"/><worldbody><body name="b" pos="0 0 1"><inertial pos="0 0 0" mass="1" diaginertia="1 1 3"/><geom type="plane" size="1 1 0.1"/></body></worldbody></mujoco>"#,
        Expect::Refused("inertia must satisfy A + B >= C; use 'balanceinertia' to fix"),
    ),
    (
        "a_moving_bodys_only_geom_at_most_mj_eps",
        r#"<mujoco><compiler inertiafromgeom="true"/><worldbody><body name="b"><freejoint/><geom type="sphere" size="0.1" mass="1e-15"/></body></worldbody></mujoco>"#,
        Expect::Refused("mass and inertia of moving bodies must be larger than mjMINVAL"),
    ),
    (
        "a_geom_at_most_mj_eps_is_not_selected",
        r#"<mujoco><worldbody><body name="b"><freejoint/><geom type="sphere" size="0.1" mass="1e-15" pos="5 0 0"/><geom type="box" size="0.1 0.2 0.3" euler="0 0 30"/></body></worldbody></mujoco>"#,
        Expect::Loads(&[Body {
            mass: 48.00000000000001,
            inertia: [2.0800000000000005, 1.6000000000000003, 0.8000000000000003],
            ipos: [0.0, 0.0, 0.0],
            iquat: [0.9659258262890683, 0.0, 0.0, 0.25881904510252074],
        }]),
    ),
    (
        "a_moving_bodys_only_geom_out_of_range",
        r#"<mujoco><compiler inertiagrouprange="0 2"/><worldbody><body name="b"><freejoint/><geom type="sphere" size="0.1" group="4"/></body></worldbody></mujoco>"#,
        Expect::Refused("mass and inertia of moving bodies must be larger than mjMINVAL"),
    ),
    (
        "a_nan_summed_inertia_keeps_the_prior_moments_and_frame",
        r#"<mujoco><worldbody><body name="s"><geom type="sphere" size="6e99"/><geom type="sphere" size="0.1" pos="1 0 0"/></body></worldbody></mujoco>"#,
        Expect::Loads(&[Body {
            mass: 9.047786842338604e+302,
            inertia: [0.0, 0.0, 0.0],
            ipos: [4.62962962962963e-303, 0.0, 0.0],
            iquat: [1.0, 0.0, 0.0, 0.0],
        }]),
    ),
    (
        "summed_moments_below_mj_eps_refused",
        r#"<mujoco><worldbody><body name="n"><geom type="cylinder" size="1e-7 1" mass="0.5"/><geom type="cylinder" size="1e-7 1" mass="0.5" pos="0 0 3"/></body></worldbody></mujoco>"#,
        Expect::Refused(
            "error 'inertia must have positive eigenvalues' in alternative for principal axes",
        ),
    ),
];

#[test]
fn a_bodys_inertia_from_its_geoms_is_mujocos() {
    check(FROM_GEOMS);
}

#[test]
fn a_body_whose_one_geoms_inertia_overflows_is_refused_where_mujoco_loads_it() {
    // MuJoCo 3.5.0 copies the geom: mass 9.047786842338604e302, inertia
    // infinite (registry row D-INERTIA-NONFINITE).
    let error = load_model(
        r#"<mujoco><worldbody><body name="s"><geom type="sphere" size="6e99"/></body></worldbody></mujoco>"#,
    )
    .err()
    .map(|e| e.to_string())
    .unwrap_or_default();
    assert!(error.contains("not finite"), "{error}");
}

/// `(name, document, expected)`, generated as [`FROM_GEOMS`]: a body that
/// neither an `<inertial>` nor a selected geom gives an inertial frame takes
/// its own pose as one, its pose in the `<frame>` holding it where one does
/// (`user_objects.cc:2442-2446`); an `<inertial>` kept under
/// `inertiafromgeom="true"` keeps its own.
const NO_INERTIAL_FRAME: &[(&str, &str, Expect)] = &[
    (
        "massless_body_inertial_frame_as_mujoco",
        r#"<mujoco><worldbody><body name="m" pos="1 0 0" euler="0 30 0"/></worldbody></mujoco>"#,
        Expect::Loads(&[Body {
            mass: 0.0,
            inertia: [0.0, 0.0, 0.0],
            ipos: [1.0, 0.0, 0.0],
            iquat: [0.9659258262890683, 0.0, 0.25881904510252074, 0.0],
        }]),
    ),
    (
        "a_massless_body_under_a_hinge",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><joint type="hinge" axis="0 1 0"/><geom type="sphere" size="0.05"/><body name="tip" pos="0.04 0 -0.1"/></body></worldbody></mujoco>"#,
        Expect::Loads(&[
            Body {
                mass: 0.5235987755982989,
                inertia: [
                    0.000523598775598299,
                    0.000523598775598299,
                    0.000523598775598299,
                ],
                ipos: [0.0, 0.0, 0.0],
                iquat: [1.0, 0.0, 0.0, 0.0],
            },
            Body {
                mass: 0.0,
                inertia: [0.0, 0.0, 0.0],
                ipos: [0.04, 0.0, -0.1],
                iquat: [1.0, 0.0, 0.0, 0.0],
            },
        ]),
    ),
    (
        "a_body_whose_only_geom_has_mass_0",
        r#"<mujoco><worldbody><body name="s" pos="0.5 0 0" euler="0 0 30"><geom type="sphere" size="0.1" mass="0"/></body></worldbody></mujoco>"#,
        Expect::Loads(&[Body {
            mass: 0.0,
            inertia: [0.0, 0.0, 0.0],
            ipos: [0.5, 0.0, 0.0],
            iquat: [0.9659258262890683, 0.0, 0.0, 0.25881904510252074],
        }]),
    ),
    (
        "a_static_bodys_only_geom_out_of_range",
        r#"<mujoco><compiler inertiagrouprange="0 2"/><worldbody><body name="b" pos="0 0 1" euler="0 0 45"><geom type="sphere" size="0.1" group="4"/></body></worldbody></mujoco>"#,
        Expect::Loads(&[Body {
            mass: 0.0,
            inertia: [0.0, 0.0, 0.0],
            ipos: [0.0, 0.0, 1.0],
            iquat: [0.9238795325112867, 0.0, 0.0, 0.3826834323650898],
        }]),
    ),
    (
        "a_massless_body_at_the_origin_turned",
        r#"<mujoco><worldbody><body name="m" euler="0 0 60"/></worldbody></mujoco>"#,
        Expect::Loads(&[Body {
            mass: 0.0,
            inertia: [0.0, 0.0, 0.0],
            ipos: [0.0, 0.0, 0.0],
            iquat: [0.8660254037844387, 0.0, 0.0, 0.49999999999999994],
        }]),
    ),
    (
        "a_massless_body_in_a_turned_frame_takes_its_pose_in_the_frame",
        r#"<mujoco><worldbody><frame pos="0 1 0" euler="0 0 90"><body name="m" pos="1 0 0" euler="0 30 0"/></frame></worldbody></mujoco>"#,
        Expect::Loads(&[Body {
            mass: 0.0,
            inertia: [0.0, 0.0, 0.0],
            ipos: [1.0, 0.0, 0.0],
            iquat: [0.9659258262890683, 0.0, 0.25881904510252074, 0.0],
        }]),
    ),
    (
        "a_massless_body_in_a_frame_under_a_moving_body",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><joint type="hinge" axis="0 1 0"/><geom type="sphere" size="0.05"/><frame pos="0.1 0 0" euler="0 0 45"><body name="tip" pos="0.04 0 -0.1" euler="10 0 0"/></frame></body></worldbody></mujoco>"#,
        Expect::Loads(&[
            Body {
                mass: 0.5235987755982989,
                inertia: [
                    0.000523598775598299,
                    0.000523598775598299,
                    0.000523598775598299,
                ],
                ipos: [0.0, 0.0, 0.0],
                iquat: [1.0, 0.0, 0.0, 0.0],
            },
            Body {
                mass: 0.0,
                inertia: [0.0, 0.0, 0.0],
                ipos: [0.04, 0.0, -0.1],
                iquat: [0.9961946980917455, 0.08715574274765817, 0.0, 0.0],
            },
        ]),
    ),
    (
        "inertiafromgeom_false_and_no_inertial",
        r#"<mujoco><compiler inertiafromgeom="false"/><worldbody><body name="s" pos="0.5 0 0" euler="0 0 30"><geom type="box" size="0.1 0.2 0.3"/></body></worldbody></mujoco>"#,
        Expect::Loads(&[Body {
            mass: 0.0,
            inertia: [0.0, 0.0, 0.0],
            ipos: [0.5, 0.0, 0.0],
            iquat: [0.9659258262890683, 0.0, 0.0, 0.25881904510252074],
        }]),
    ),
    (
        "true_keeps_an_inertials_frame_where_no_geom_is_selected",
        r#"<mujoco><compiler inertiafromgeom="true" inertiagrouprange="0 2"/><worldbody><body name="s" pos="0.5 0 0" euler="0 0 30"><freejoint/><inertial pos="0.1 0.2 0.3" mass="1" diaginertia="0.1 0.2 0.3"/><geom type="sphere" size="0.1" group="4"/></body></worldbody></mujoco>"#,
        Expect::Loads(&[Body {
            mass: 1.0,
            inertia: [0.1, 0.2, 0.3],
            ipos: [0.1, 0.2, 0.3],
            iquat: [1.0, 0.0, 0.0, 0.0],
        }]),
    ),
];

#[test]
fn a_body_with_no_inertial_frame_takes_its_own() {
    check(NO_INERTIAL_FRAME);
}

/// `(name, document, expected)`, generated as [`FROM_GEOMS`]: an
/// `<inertial>`'s principal axes from its quaternion, made unit by
/// `mjuu_normvec`, or from its orientation alternative, resolved as a
/// body's is (`user_objects.cc:2386`, `:2420-2425`).
const INERTIAL_FRAME: &[(&str, &str, Expect)] = &[
    (
        "euler",
        r#"<mujoco><worldbody><body name="b"><freejoint/><inertial pos="0 0 0" mass="1" diaginertia="1 2 3" euler="0 0 90"/></body></worldbody></mujoco>"#,
        Expect::Loads(&[Body {
            mass: 1.0,
            inertia: [1.0, 2.0, 3.0],
            ipos: [0.0, 0.0, 0.0],
            iquat: [0.7071067811865476, 0.0, 0.0, 0.7071067811865475],
        }]),
    ),
    (
        "axisangle_in_radians",
        r#"<mujoco><compiler angle="radian"/><worldbody><body name="b"><freejoint/><inertial pos="0 0 0" mass="1" diaginertia="1 2 3" axisangle="1 1 0 0.7"/></body></worldbody></mujoco>"#,
        Expect::Loads(&[Body {
            mass: 1.0,
            inertia: [1.0, 2.0, 3.0],
            ipos: [0.0, 0.0, 0.0],
            iquat: [
                0.9393727128473789,
                0.24246536490574871,
                0.24246536490574871,
                0.0,
            ],
        }]),
    ),
    (
        "xyaxes",
        r#"<mujoco><worldbody><body name="b"><freejoint/><inertial pos="0 0 0" mass="1" diaginertia="1 2 3" xyaxes="0 1 0 -1 0 0"/></body></worldbody></mujoco>"#,
        Expect::Loads(&[Body {
            mass: 1.0,
            inertia: [1.0, 2.0, 3.0],
            ipos: [0.0, 0.0, 0.0],
            iquat: [0.7071067811865476, 0.0, 0.0, 0.7071067811865475],
        }]),
    ),
    (
        "zaxis",
        r#"<mujoco><worldbody><body name="b"><freejoint/><inertial pos="0 0 0" mass="1" diaginertia="1 2 3" zaxis="1 0 0"/></body></worldbody></mujoco>"#,
        Expect::Loads(&[Body {
            mass: 1.0,
            inertia: [1.0, 2.0, 3.0],
            ipos: [0.0, 0.0, 0.0],
            iquat: [0.7071067811865476, 0.0, 0.7071067811865475, 0.0],
        }]),
    ),
    (
        "euler_in_another_sequence",
        r#"<mujoco><compiler eulerseq="zyx"/><worldbody><body name="b"><freejoint/><inertial pos="0 0 0" mass="1" diaginertia="1 2 3" euler="10 20 30"/></body></worldbody></mujoco>"#,
        Expect::Loads(&[Body {
            mass: 1.0,
            inertia: [1.0, 2.0, 3.0],
            ipos: [0.0, 0.0, 0.0],
            iquat: [
                0.9515485246437885,
                0.2392983377447303,
                0.18930785741199999,
                0.03813457647485015,
            ],
        }]),
    ),
    (
        "a_quaternion_made_unit_as_mujoco_makes_it",
        r#"<mujoco><worldbody><body name="b"><freejoint/><inertial pos="0 0 0" mass="1" diaginertia="1 2 3" quat="0.3 0.1 0.2 0.4"/></body></worldbody></mujoco>"#,
        Expect::Loads(&[Body {
            mass: 1.0,
            inertia: [1.0, 2.0, 3.0],
            ipos: [0.0, 0.0, 0.0],
            iquat: [
                0.5477225575051661,
                0.18257418583505536,
                0.3651483716701107,
                0.7302967433402214,
            ],
        }]),
    ),
    (
        "a_degenerate_xyaxes",
        r#"<mujoco><worldbody><body name="b"><freejoint/><inertial pos="0 0 0" mass="1" diaginertia="1 1 1" xyaxes="1 0 0 1 0 0"/></body></worldbody></mujoco>"#,
        Expect::Refused("error 'yaxis too small' in inertia alternative"),
    ),
    (
        "a_zero_zaxis",
        r#"<mujoco><worldbody><body name="b"><freejoint/><inertial pos="0 0 0" mass="1" diaginertia="1 1 1" zaxis="0 0 0"/></body></worldbody></mujoco>"#,
        Expect::Refused("error 'zaxis too small' in inertia alternative"),
    ),
    (
        "a_zero_zaxis_named_before_its_bodys_bad_condim",
        r#"<mujoco><worldbody><body name="b"><freejoint/><inertial pos="0 0 0" mass="1" diaginertia="1 2 3" zaxis="0 0 0"/><geom type="sphere" size="0.1" condim="2"/></body></worldbody></mujoco>"#,
        Expect::Refused("error 'zaxis too small' in inertia alternative"),
    ),
    (
        "an_earlier_bodys_bad_condim_named_before_a_zero_zaxis",
        r#"<mujoco><worldbody><body name="a"><freejoint/><geom type="sphere" size="0.1" condim="2"/></body><body name="b"><freejoint/><inertial pos="0 0 0" mass="1" diaginertia="1 2 3" zaxis="0 0 0"/></body></worldbody></mujoco>"#,
        Expect::Refused("invalid condim in geom"),
    ),
    (
        "an_earlier_bodys_bad_geom_named_before_a_zero_zaxis",
        r#"<mujoco><worldbody><body name="a"><freejoint/><geom type="box" size="0.1 0.1 0" mass="1"/></body><body name="b"><freejoint/><inertial pos="0 0 0" mass="1" diaginertia="1 2 3" zaxis="0 0 0"/></body></worldbody></mujoco>"#,
        Expect::Refused("size 2 must be positive"),
    ),
];

#[test]
fn an_inertials_frame_is_mujocos() {
    check(INERTIAL_FRAME);
}
