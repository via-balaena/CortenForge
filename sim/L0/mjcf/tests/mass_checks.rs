//! A body's mass and inertia as MuJoCo 3.5.0 checks them (book L22; ledger
//! L207): an `<inertial>`'s rules, a geom's (a plane only in a static body;
//! a negative mass or density where the geom infers its body's inertia), and
//! a body's (a `fullinertia` beside a diagonal inertia; its moments' triangle
//! after `boundinertia`, balanced or refused), a moving body's mass and
//! inertia at least mjMINVAL (ledger L207), and `settotalmass` after it.
//!
//! Every document below was loaded by MuJoCo 3.5.0 on the census's unfused
//! oracle (2026-11-02): each refusal expects MuJoCo's words; each load,
//! MuJoCo's `body_mass` bit for bit and its `body_inertia` within a relative
//! 1e-12, which a value that is not finite fails.

// The values are MuJoCo's, printed to the last digit.
#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::unreadable_literal
)]

use sim_mjcf::load_model;

/// What loading a document gives.
enum Expect {
    /// MuJoCo refuses it, with these words.
    Refused(&'static str),
    /// MuJoCo loads it with these bodies' masses and inertias, the world's
    /// first.
    Loads {
        mass: &'static [f64],
        inertia: &'static [[f64; 3]],
    },
}

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
            Expect::Loads { mass, inertia } => {
                let model = loaded.unwrap_or_else(|e| panic!("{name}: {e}"));
                let bits = |v: &[f64]| v.iter().map(|x| x.to_bits()).collect::<Vec<_>>();
                assert_eq!(
                    bits(&model.body_mass),
                    bits(mass),
                    "{name}: body_mass {:?}",
                    model.body_mass
                );
                assert_eq!(model.body_inertia.len(), inertia.len(), "{name}: nbody");
                for (got, want) in model.body_inertia.iter().zip(*inertia) {
                    for (g, w) in got.iter().zip(want) {
                        assert!(
                            (g - w).abs() <= 1e-12 * w.abs(),
                            "{name}: body_inertia {:?}, MuJoCo's {inertia:?}",
                            model.body_inertia
                        );
                    }
                }
            }
        }
    }
}

/// `(name, document, expected)`, generated from the oracle's results.
const INERTIALS: &[(&str, &str, Expect)] = &[
    (
        "quat_and_full",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" quat="1 0 0 0" fullinertia="1 1 1 0 0 0"/></body></worldbody></mujoco>"#,
        Expect::Refused("fullinertia and inertial orientation cannot both be specified"),
    ),
    (
        "quat_empty_and_full",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" quat="" fullinertia="1 1 1 0 0 0"/></body></worldbody></mujoco>"#,
        Expect::Refused("fullinertia and inertial orientation cannot both be specified"),
    ),
    (
        "euler_and_full",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" euler="0 0 0" fullinertia="1 1 1 0 0 0"/></body></worldbody></mujoco>"#,
        Expect::Refused("fullinertia and inertial orientation cannot both be specified"),
    ),
    (
        "axisangle_and_full",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" axisangle="0 0 1 0" fullinertia="1 1 1 0 0 0"/></body></worldbody></mujoco>"#,
        Expect::Refused("fullinertia and inertial orientation cannot both be specified"),
    ),
    (
        "xyaxes_and_full",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" xyaxes="1 0 0 0 1 0" fullinertia="1 1 1 0 0 0"/></body></worldbody></mujoco>"#,
        Expect::Refused("fullinertia and inertial orientation cannot both be specified"),
    ),
    (
        "zaxis_and_full",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" zaxis="0 0 1" fullinertia="1 1 1 0 0 0"/></body></worldbody></mujoco>"#,
        Expect::Refused("fullinertia and inertial orientation cannot both be specified"),
    ),
    (
        "full_alone",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" fullinertia="1 1 1 0 0 0"/></body></worldbody></mujoco>"#,
        Expect::Loads {
            mass: &[0.0, 1.0],
            inertia: &[[0.0, 0.0, 0.0], [1.0, 1.0, 1.0]],
        },
    ),
    (
        "quat_and_diag",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" quat="1 0 0 0" diaginertia="1 1 1"/></body></worldbody></mujoco>"#,
        Expect::Loads {
            mass: &[0.0, 1.0],
            inertia: &[[0.0, 0.0, 0.0], [1.0, 1.0, 1.0]],
        },
    ),
];

#[test]
fn each_inertial_rule_is_mujocos() {
    check(INERTIALS);
}

/// `(name, document, expected)`, generated from the oracle's results.
const GEOMS: &[(&str, &str, Expect)] = &[
    (
        "geom_density_neg_moving",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0"/><geom type="sphere" size="0.1" density="-1"/></body></worldbody></mujoco>"#,
        Expect::Refused("mass, inertia or density are negative in geom"),
    ),
    (
        "geom_density_neg_static",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><geom type="sphere" size="0.1" density="-1"/></body></worldbody></mujoco>"#,
        Expect::Refused("mass, inertia or density are negative in geom"),
    ),
    (
        "geom_density_neg_plane_static",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><geom type="plane" size="1 1 0.1" density="-1"/></body></worldbody></mujoco>"#,
        Expect::Refused("mass, inertia or density are negative in geom"),
    ),
    (
        "g_density_neg_class",
        r#"<mujoco><default><default class="d"><geom density="-1"/></default></default><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0"/><geom class="d" type="sphere" size="0.1"/></body></worldbody></mujoco>"#,
        Expect::Refused("mass, inertia or density are negative in geom"),
    ),
    (
        "geom_mass_neg_moving",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0"/><geom type="sphere" size="0.1" mass="-1"/></body></worldbody></mujoco>"#,
        Expect::Refused("mass, inertia or density are negative in geom"),
    ),
    (
        "geom_mass_neg_static",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><geom type="sphere" size="0.1" mass="-1"/></body></worldbody></mujoco>"#,
        Expect::Refused("mass, inertia or density are negative in geom"),
    ),
    (
        "geom_mass_neg_inertiafromgeom_true",
        r#"<mujoco><compiler inertiafromgeom="true"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/><geom type="sphere" size="0.1" mass="-1"/></body></worldbody></mujoco>"#,
        Expect::Refused("mass, inertia or density are negative in geom"),
    ),
    (
        "g_mass_neg_ifg_false_no_inertial",
        r#"<mujoco><compiler inertiafromgeom="false"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0"/><geom type="sphere" size="0.1" mass="-1"/></body></worldbody></mujoco>"#,
        Expect::Refused("mass, inertia or density are negative in geom"),
    ),
    (
        "g_mass_neg_second_geom",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0"/><geom type="sphere" size="0.1"/><geom type="sphere" size="0.1" mass="-1"/></body></worldbody></mujoco>"#,
        Expect::Refused("mass, inertia or density are negative in geom"),
    ),
    (
        "g_mass_pos_density_neg_plane_static",
        r#"<mujoco><worldbody><body name="s" pos="0 0 1"><geom type="plane" size="1 1 0.1" mass="1" density="-1"/></body></worldbody></mujoco>"#,
        Expect::Refused("mass, inertia or density are negative in geom"),
    ),
    (
        "g_negative_mesh_faces_mass",
        r#"<mujoco><asset><mesh name="m" vertex="0 0 0 1 0 0 0 1 0 0 0 1" face="0 2 1 0 1 3 0 3 2 1 2 3"/></asset><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0"/><geom type="mesh" mesh="m" mass="-1"/></body></worldbody></mujoco>"#,
        Expect::Refused("mass, inertia or density are negative in geom"),
    ),
    (
        "plane_in_moving_body",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0"/><geom type="sphere" size="0.1"/><geom type="plane" size="1 1 0.1"/></body></worldbody></mujoco>"#,
        Expect::Refused("plane only allowed in static bodies"),
    ),
    (
        "plane_static_child_of_moving",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0"/><geom type="sphere" size="0.1"/><body name="c" pos="0 0 0"><geom type="plane" size="1 1 0.1"/></body></body></worldbody></mujoco>"#,
        Expect::Refused("plane only allowed in static bodies"),
    ),
    (
        "g_plane_static_grandchild_of_moving",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/><body name="c"><body name="d"><geom type="plane" size="1 1 0.1"/></body></body></body></worldbody></mujoco>"#,
        Expect::Refused("plane only allowed in static bodies"),
    ),
    (
        "g_plane_freejoint",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><freejoint/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/><geom type="plane" size="1 1 0.1"/></body></worldbody></mujoco>"#,
        Expect::Refused("plane only allowed in static bodies"),
    ),
    (
        "g_plane_then_negative_same_geom",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0"/><geom type="plane" size="1 1 0.1" density="-1"/></body></worldbody></mujoco>"#,
        Expect::Refused("plane only allowed in static bodies"),
    ),
    (
        "geom_mass_neg_beside_inertial",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/><geom type="sphere" size="0.1" mass="-1"/></body></worldbody></mujoco>"#,
        Expect::Loads {
            mass: &[0.0, 1.0],
            inertia: &[[0.0, 0.0, 0.0], [0.01, 0.01, 0.01]],
        },
    ),
    (
        "geom_mass_neg_inertiafromgeom_false",
        r#"<mujoco><compiler inertiafromgeom="false"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/><geom type="sphere" size="0.1" mass="-1"/></body></worldbody></mujoco>"#,
        Expect::Loads {
            mass: &[0.0, 1.0],
            inertia: &[[0.0, 0.0, 0.0], [0.01, 0.01, 0.01]],
        },
    ),
    (
        "geom_mass_neg_world",
        r#"<mujoco><worldbody><geom type="sphere" size="0.1" mass="-1"/></worldbody></mujoco>"#,
        Expect::Loads {
            mass: &[0.0],
            inertia: &[[0.0, 0.0, 0.0]],
        },
    ),
    (
        "geom_density_neg_world",
        r#"<mujoco><worldbody><geom type="sphere" size="0.1" density="-1"/></worldbody></mujoco>"#,
        Expect::Loads {
            mass: &[0.0],
            inertia: &[[0.0, 0.0, 0.0]],
        },
    ),
    (
        "geom_mass_neg_plane_static",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><geom type="plane" size="1 1 0.1" mass="-1"/></body></worldbody></mujoco>"#,
        Expect::Loads {
            mass: &[0.0, 0.0],
            inertia: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
        },
    ),
    (
        "g_mass_neg_plane_moving_tiny",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/><geom type="sphere" size="0.1"/></body><body name="s" pos="1 0 1"><geom type="box" size="1e-6 1e-6 1e-6" mass="-1"/></body></worldbody></mujoco>"#,
        Expect::Loads {
            mass: &[0.0, 1.0, 0.0],
            inertia: &[[0.0, 0.0, 0.0], [0.01, 0.01, 0.01], [0.0, 0.0, 0.0]],
        },
    ),
    (
        "g_mass_pos_density_neg_sphere",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0"/><geom type="sphere" size="0.1" mass="1" density="-1"/></body></worldbody></mujoco>"#,
        Expect::Loads {
            mass: &[0.0, 1.0],
            inertia: &[
                [0.0, 0.0, 0.0],
                [
                    0.004000000000000001,
                    0.004000000000000001,
                    0.004000000000000001,
                ],
            ],
        },
    ),
    (
        "g_mass0_density_neg_static",
        r#"<mujoco><worldbody><body name="s" pos="0 0 1"><geom type="sphere" size="0.1" mass="0" density="-1"/></body></worldbody></mujoco>"#,
        Expect::Loads {
            mass: &[0.0, 0.0],
            inertia: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
        },
    ),
    (
        "g_mesh_faces_mass_pos",
        r#"<mujoco><asset><mesh name="m" vertex="0 0 0 1 0 0 0 1 0 0 0 1" face="0 2 1 0 1 3 0 3 2 1 2 3"/></asset><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0"/><geom type="mesh" mesh="m" mass="2"/></body></worldbody></mujoco>"#,
        Expect::Loads {
            mass: &[0.0, 2.0],
            inertia: &[
                [0.0, 0.0, 0.0],
                [
                    0.19999999999999987,
                    0.12499999999999999,
                    0.12499999999999999,
                ],
            ],
        },
    ),
    (
        "plane_in_static_body",
        r#"<mujoco><worldbody><body name="floor" pos="0 0 0"><geom type="plane" size="1 1 0.1"/></body></worldbody></mujoco>"#,
        Expect::Loads {
            mass: &[0.0, 0.0],
            inertia: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
        },
    ),
    (
        "plane_in_world",
        r#"<mujoco><worldbody><geom type="plane" size="1 1 0.1"/><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0"/><geom type="sphere" size="0.1"/></body></worldbody></mujoco>"#,
        Expect::Loads {
            mass: &[0.0, 4.188790204786391],
            inertia: &[
                [0.0, 0.0, 0.0],
                [
                    0.01675516081914557,
                    0.01675516081914557,
                    0.01675516081914557,
                ],
            ],
        },
    ),
    (
        "plane_in_mocap_body",
        r#"<mujoco><worldbody><body name="m" mocap="true"><geom type="plane" size="1 1 0.1"/></body></worldbody></mujoco>"#,
        Expect::Loads {
            mass: &[0.0, 0.0],
            inertia: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
        },
    ),
    (
        "g_plane_under_mocap",
        r#"<mujoco><worldbody><body name="m" mocap="true" pos="0 0 1"><geom type="plane" size="1 1 0.1"/></body></worldbody></mujoco>"#,
        Expect::Loads {
            mass: &[0.0, 0.0],
            inertia: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
        },
    ),
    (
        "g_plane_sibling_moves",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body><body name="s" pos="1 0 1"><geom type="plane" size="1 1 0.1"/></body></worldbody></mujoco>"#,
        Expect::Loads {
            mass: &[0.0, 1.0, 0.0],
            inertia: &[[0.0, 0.0, 0.0], [0.01, 0.01, 0.01], [0.0, 0.0, 0.0]],
        },
    ),
];

#[test]
fn each_geom_check_is_mujocos() {
    check(GEOMS);
}

/// `(name, document, expected)`, generated from the oracle's results.
const BODIES: &[(&str, &str, Expect)] = &[
    (
        "diag_and_full",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="1 1 1" fullinertia="1 1 1 0 0 0"/></body></worldbody></mujoco>"#,
        Expect::Refused("fullinertia and diagonal inertia cannot both be specified"),
    ),
    (
        "t_full_and_diag_ifg_true",
        r#"<mujoco><compiler inertiafromgeom="true"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="1 1 1" fullinertia="1 1 1 0 0 0"/><geom type="sphere" size="0.1"/></body></worldbody></mujoco>"#,
        Expect::Refused("fullinertia and diagonal inertia cannot both be specified"),
    ),
    (
        "t_full_and_diag_static",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><inertial pos="0 0 0" mass="1" diaginertia="0 0 1" fullinertia="1 1 1 0 0 0"/></body></worldbody></mujoco>"#,
        Expect::Refused("fullinertia and diagonal inertia cannot both be specified"),
    ),
    (
        "inertia_triangle_moving",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="1 1 3"/></body></worldbody></mujoco>"#,
        Expect::Refused("inertia must satisfy A + B >= C; use 'balanceinertia' to fix"),
    ),
    (
        "inertia_triangle_static",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><inertial pos="0 0 0" mass="1" diaginertia="1 1 3"/></body></worldbody></mujoco>"#,
        Expect::Refused("inertia must satisfy A + B >= C; use 'balanceinertia' to fix"),
    ),
    (
        "t_triangle_full_moving",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" fullinertia="0.1 0.2 0.3 0.02 0.01 0.03"/></body></worldbody></mujoco>"#,
        Expect::Refused("inertia must satisfy A + B >= C; use 'balanceinertia' to fix"),
    ),
    (
        "t_triangle_mocap",
        r#"<mujoco><worldbody><body name="m" mocap="true" pos="0 0 1"><inertial pos="0 0 0" mass="1" diaginertia="1 1 3"/></body></worldbody></mujoco>"#,
        Expect::Refused("inertia must satisfy A + B >= C; use 'balanceinertia' to fix"),
    ),
    (
        "t_triangle_second_body",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><joint name="ja" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body><body name="b" pos="1 0 1"><joint name="jb" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="1 1 3"/></body></worldbody></mujoco>"#,
        Expect::Refused("inertia must satisfy A + B >= C; use 'balanceinertia' to fix"),
    ),
    (
        "zero_diag_and_full",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="0 0 0" fullinertia="1 1 1 0 0 0"/></body></worldbody></mujoco>"#,
        Expect::Loads {
            mass: &[0.0, 1.0],
            inertia: &[[0.0, 0.0, 0.0], [1.0, 1.0, 1.0]],
        },
    ),
    (
        "bound_then_balance",
        r#"<mujoco><compiler boundinertia="0.5" balanceinertia="true"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 1"/></body></worldbody></mujoco>"#,
        Expect::Loads {
            mass: &[0.0, 1.0],
            inertia: &[[0.0, 0.0, 0.0], [0.5, 0.5, 1.0]],
        },
    ),
    (
        "bound_then_triangle",
        r#"<mujoco><compiler boundinertia="2"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="1 1 3"/></body></worldbody></mujoco>"#,
        Expect::Loads {
            mass: &[0.0, 1.0],
            inertia: &[[0.0, 0.0, 0.0], [2.0, 2.0, 3.0]],
        },
    ),
    (
        "t_boundinertia_makes_valid",
        r#"<mujoco><compiler boundinertia="1"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="1 0 0"/></body></worldbody></mujoco>"#,
        Expect::Loads {
            mass: &[0.0, 1.0],
            inertia: &[[0.0, 0.0, 0.0], [1.0, 1.0, 1.0]],
        },
    ),
    (
        "inertia_triangle_balanced",
        r#"<mujoco><compiler balanceinertia="true"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="1 1 3"/></body></worldbody></mujoco>"#,
        Expect::Loads {
            mass: &[0.0, 1.0],
            inertia: &[
                [0.0, 0.0, 0.0],
                [1.6666666666666667, 1.6666666666666667, 1.6666666666666667],
            ],
        },
    ),
    (
        "t_triangle_balanced_value",
        r#"<mujoco><compiler balanceinertia="true"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="0.1 0.2 0.7"/></body></worldbody></mujoco>"#,
        Expect::Loads {
            mass: &[0.0, 1.0],
            inertia: &[
                [0.0, 0.0, 0.0],
                [0.3333333333333333, 0.3333333333333333, 0.3333333333333333],
            ],
        },
    ),
    (
        "inertia_triangle_equal",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="1 1 2"/></body></worldbody></mujoco>"#,
        Expect::Loads {
            mass: &[0.0, 1.0],
            inertia: &[[0.0, 0.0, 0.0], [1.0, 1.0, 2.0]],
        },
    ),
    (
        "geom_triangle_from_geoms",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0"/><geom type="box" size="1 0.001 0.001"/></body></worldbody></mujoco>"#,
        Expect::Loads {
            mass: &[0.0, 0.008],
            inertia: &[
                [0.0, 0.0, 0.0],
                [
                    5.333333333333334e-09,
                    0.002666669333333333,
                    0.002666669333333333,
                ],
            ],
        },
    ),
];

#[test]
fn each_body_inertia_check_is_mujocos() {
    check(BODIES);
}

/// `(name, document, expected)`, generated from the oracle's results.
const MOVING: &[(&str, &str, Expect)] = &[
    (
        "moving_body_no_mass",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0"/></body></worldbody></mujoco>"#,
        Expect::Refused("mass and inertia of moving bodies must be larger than mjMINVAL"),
    ),
    (
        "moving_body_site_only",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0"/><site name="s"/></body></worldbody></mujoco>"#,
        Expect::Refused("mass and inertia of moving bodies must be larger than mjMINVAL"),
    ),
    (
        "moving_mass_1e-16",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1e-16" diaginertia="1 1 1"/></body></worldbody></mujoco>"#,
        Expect::Refused("mass and inertia of moving bodies must be larger than mjMINVAL"),
    ),
    (
        "moving_inertia_1e-20",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="1e-20 1e-20 1e-20"/></body></worldbody></mujoco>"#,
        Expect::Refused("mass and inertia of moving bodies must be larger than mjMINVAL"),
    ),
    (
        "diaginertia_zero_moving",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><freejoint name="f"/><inertial pos="0 0 0" mass="1" diaginertia="0 0 0"/></body></worldbody></mujoco>"#,
        Expect::Refused("mass and inertia of moving bodies must be larger than mjMINVAL"),
    ),
    (
        "geom_mass_zero_moving",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0"/><geom type="sphere" size="0.1" mass="0"/></body></worldbody></mujoco>"#,
        Expect::Refused("mass and inertia of moving bodies must be larger than mjMINVAL"),
    ),
    (
        "geom_density_zero_moving",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0"/><geom type="sphere" size="0.1" density="0"/></body></worldbody></mujoco>"#,
        Expect::Refused("mass and inertia of moving bodies must be larger than mjMINVAL"),
    ),
    (
        "inertiafromgeom_false_no_inertial_moving",
        r#"<mujoco><compiler inertiafromgeom="false"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0"/><geom type="sphere" size="0.1"/></body></worldbody></mujoco>"#,
        Expect::Refused("mass and inertia of moving bodies must be larger than mjMINVAL"),
    ),
    (
        "moving_nomass_moving_child",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0"/><body name="c" pos="0 0 0"><joint name="j2" type="hinge" axis="0 1 0"/><geom type="sphere" size="0.1"/></body></body></worldbody></mujoco>"#,
        Expect::Refused("mass and inertia of moving bodies must be larger than mjMINVAL"),
    ),
    (
        "settotalmass_after_check",
        r#"<mujoco><compiler settotalmass="10"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1e-16" diaginertia="1 1 1"/></body></worldbody></mujoco>"#,
        Expect::Refused("mass and inertia of moving bodies must be larger than mjMINVAL"),
    ),
    (
        "moving_mass_1e-14",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1e-14" diaginertia="1 1 1"/></body></worldbody></mujoco>"#,
        Expect::Loads {
            mass: &[0.0, 1e-14],
            inertia: &[[0.0, 0.0, 0.0], [1.0, 1.0, 1.0]],
        },
    ),
    (
        "moving_nomass_static_child",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0"/><body name="c" pos="0 0 0"><geom type="sphere" size="0.1"/></body></body></worldbody></mujoco>"#,
        Expect::Loads {
            mass: &[0.0, 0.0, 4.188790204786391],
            inertia: &[
                [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0],
                [
                    0.01675516081914557,
                    0.01675516081914557,
                    0.01675516081914557,
                ],
            ],
        },
    ),
    (
        "moving_nomass_static_grandchild",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0"/><body name="c" pos="0 0 0"><body name="g" pos="0 0 0"><geom type="sphere" size="0.1"/></body></body></body></worldbody></mujoco>"#,
        Expect::Loads {
            mass: &[0.0, 0.0, 0.0, 4.188790204786391],
            inertia: &[
                [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0],
                [
                    0.01675516081914557,
                    0.01675516081914557,
                    0.01675516081914557,
                ],
            ],
        },
    ),
    (
        "mocap_geom_mass0",
        r#"<mujoco><worldbody><body name="m" mocap="true"><geom type="sphere" size="0.1" mass="0"/></body></worldbody></mujoco>"#,
        Expect::Loads {
            mass: &[0.0, 0.0],
            inertia: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
        },
    ),
    (
        "static_body_no_mass",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><site name="s"/></body></worldbody></mujoco>"#,
        Expect::Loads {
            mass: &[0.0, 0.0],
            inertia: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
        },
    ),
    (
        "plane_static_settotalmass",
        r#"<mujoco><compiler settotalmass="10"/><worldbody><body name="floor" pos="0 0 0"><geom type="plane" size="1 1 0.1"/></body><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0"/><geom type="sphere" size="0.1"/></body></worldbody></mujoco>"#,
        Expect::Loads {
            mass: &[0.0, 0.0, 10.0],
            inertia: &[
                [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0],
                [
                    0.04000000000000001,
                    0.04000000000000001,
                    0.04000000000000001,
                ],
            ],
        },
    ),
    (
        "t_settotalmass_tiny_total",
        r#"<mujoco><compiler settotalmass="10"/><worldbody><body name="s" pos="0 0 1"><inertial pos="0 0 0" mass="1e-20" diaginertia="1e-20 1e-20 1e-20"/></body></worldbody></mujoco>"#,
        Expect::Loads {
            mass: &[0.0, 9.999999999999999e-05],
            inertia: &[
                [0.0, 0.0, 0.0],
                [
                    9.999999999999999e-05,
                    9.999999999999999e-05,
                    9.999999999999999e-05,
                ],
            ],
        },
    ),
    (
        "t_settotalmass_two_bodies",
        r#"<mujoco><compiler settotalmass="10"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="2" diaginertia="0.1 0.2 0.25"/></body><body name="s" pos="1 0 1"><inertial pos="0 0 0" mass="3" diaginertia="0.3 0.3 0.3"/></body></worldbody></mujoco>"#,
        Expect::Loads {
            mass: &[0.0, 4.0, 6.0],
            inertia: &[[0.0, 0.0, 0.0], [0.2, 0.4, 0.5], [0.6, 0.6, 0.6]],
        },
    ),
    (
        "t_moving_exactly_minval",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1e-15" diaginertia="1e-15 1e-15 1e-15"/></body></worldbody></mujoco>"#,
        Expect::Loads {
            mass: &[0.0, 1e-15],
            inertia: &[[0.0, 0.0, 0.0], [1e-15, 1e-15, 1e-15]],
        },
    ),
    (
        "t_settotalmass_tiny_target",
        r#"<mujoco><compiler settotalmass="1e-20"/><worldbody><body name="s" pos="0 0 1"><inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/></body></worldbody></mujoco>"#,
        Expect::Loads {
            mass: &[0.0, 1e-15],
            inertia: &[[0.0, 0.0, 0.0], [1e-15, 1e-15, 1e-15]],
        },
    ),
];

#[test]
fn each_moving_body_check_and_total_mass_is_mujocos() {
    check(MOVING);
}

/// A flexcomp's vertex bodies are bodies to the moving-body check and to
/// `settotalmass`. This crate reads a flexcomp only inside `<deformable>`,
/// which MuJoCo 3.5.0 refuses, and MuJoCo's form (in `<worldbody>`) is
/// skipped here (ledger L358), so each document below is the `<deformable>`
/// form of one MuJoCo loaded in its own form, and expects MuJoCo's result
/// there (2026-11-02): mass 4e-15 refused; mass 3.2e-14 with inertiabox 2,
/// each vertex body 8e-15 with moments 2.1333333333333335e-14; and
/// `settotalmass="10"` beside a sphere's body, each vertex body
/// 0.4818078783940578 and the sphere's 8.072768486423769 (MuJoCo lists the
/// vertex bodies first, this crate after the bodies).
#[test]
fn a_flexcomps_vertex_bodies_are_checked_and_scaled_as_mujocos() {
    let flexcomp = |compiler: &str, body: &str, mass: &str, inertiabox: &str| {
        format!(
            r#"<mujoco>{compiler}<worldbody>{body}</worldbody><deformable><flexcomp name="cloth" type="grid" count="2 2 1" spacing="0.04 0.04 0.04" dim="2" mass="{mass}" inertiabox="{inertiabox}"><edge equality="false"/></flexcomp></deformable></mujoco>"#
        )
    };
    let error = load_model(&flexcomp("", "", "4e-15", "0.005"))
        .err()
        .map(|e| e.to_string())
        .unwrap_or_default();
    assert!(
        error.contains("mass and inertia of moving bodies must be larger than mjMINVAL"),
        "{error}"
    );

    let model = load_model(&flexcomp("", "", "3.2e-14", "2")).expect("loads");
    assert_eq!(model.body_mass[1..], [8e-15; 4]);
    for inertia in &model.body_inertia[1..] {
        assert_eq!(inertia.as_slice(), [2.1333333333333335e-14; 3]);
    }

    let sphere = r#"<body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0"/><geom type="sphere" size="0.1"/></body>"#;
    let model = load_model(&flexcomp(
        r#"<compiler settotalmass="10"/>"#,
        sphere,
        "1",
        "0.005",
    ))
    .expect("loads");
    let close = |got: f64, want: f64| (got - want).abs() <= 1e-12 * want;
    assert!(
        close(model.body_mass[1], 8.072768486423769),
        "{:?}",
        model.body_mass
    );
    for &mass in &model.body_mass[2..] {
        assert!(close(mass, 0.4818078783940578), "{:?}", model.body_mass);
    }
}
