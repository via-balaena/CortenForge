//! A body's mass and inertia as MuJoCo 3.5.0 checks them (book L22; ledger
//! L207): an `<inertial>`'s rules.
//!
//! Every document below was loaded by MuJoCo 3.5.0 on the census's unfused
//! oracle (2026-11-02): each refusal expects MuJoCo's words; each load,
//! MuJoCo's `body_mass` bit for bit and its `body_inertia` within a relative
//! 1e-12, which a value that is not finite fails.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

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
