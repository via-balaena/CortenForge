//! A body's mass and inertia as MuJoCo 3.5.0 checks them (book L22; ledger
//! L207): an `<inertial>`'s rules, a geom's (a plane only in a static body;
//! a negative mass or density where the geom infers its body's inertia), and
//! a body's (a `fullinertia` beside a diagonal inertia; its moments' triangle
//! after `boundinertia`, balanced or refused).
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
