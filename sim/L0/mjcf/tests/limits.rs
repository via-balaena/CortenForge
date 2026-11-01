//! A joint's and a tendon's `limited` and `range`, and a tendon's
//! `springlength`, as MuJoCo 3.5.0 resolves and checks them (book L21;
//! ledger L1, L55, L69, L349; `builder/limits.rs`).
//!
//! Every document below was loaded by MuJoCo 3.5.0 on the census's unfused
//! oracle (2026-11-01): each refusal expects MuJoCo's words; each load,
//! MuJoCo's `jnt_limited` and `jnt_range`, or `tendon_limited`,
//! `tendon_range` and `tendon_lengthspring`, bit for bit (a range is the
//! written one, or, limited in degrees, times `pi/180`; a spring length is
//! the written one or the tendon's length at `qpos0`, of joints at 0: no
//! library call to differ between platforms). An automatic range that does
//! not increase, which MuJoCo leaves unlimited, is refused (`D-AUTO-RANGE`).

// The values are MuJoCo's, printed to the last digit; some are fractions of pi.
#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::unreadable_literal,
    clippy::approx_constant
)]

use sim_mjcf::load_model;

/// What loading a document gives.
enum Expect {
    /// MuJoCo refuses it, with these words.
    Refused(&'static str),
    /// MuJoCo leaves the automatic range unlimited; refused (`D-AUTO-RANGE`).
    Automatic,
    /// MuJoCo loads it with these joints.
    Joints {
        limited: &'static [bool],
        range: &'static [(f64, f64)],
    },
    /// MuJoCo loads it with these tendons.
    Tendons {
        limited: &'static [bool],
        range: &'static [(f64, f64)],
        lengthspring: &'static [[f64; 2]],
    },
}

/// Each pair's bits, so that a comparison is exact.
fn bits(r: &[(f64, f64)]) -> Vec<(u64, u64)> {
    r.iter().map(|(a, b)| (a.to_bits(), b.to_bits())).collect()
}

/// `(name, document, expected)`, generated from the oracle's results.
const JOINTS: &[(&str, &str, Expect)] = &[
    (
        "j_ball_auto_0_60",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="ball" range="0 60"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody></mujoco>"#,
        Expect::Joints {
            limited: &[true],
            range: &[(0.0_f64, 1.0471975511965976_f64)],
        },
    ),
    (
        "j_ball_auto_0_m10",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="ball" range="0 -10"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody></mujoco>"#,
        Expect::Automatic,
    ),
    (
        "j_ball_auto_0_m1_rad",
        r#"<mujoco><compiler angle="radian"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="ball" range="0 -1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody></mujoco>"#,
        Expect::Automatic,
    ),
    (
        "j_ball_auto_45_0",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="ball" range="45 0"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody></mujoco>"#,
        Expect::Automatic,
    ),
    (
        "j_ball_auto_zero",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="ball" range="0 0"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody></mujoco>"#,
        Expect::Joints {
            limited: &[false],
            range: &[(0.0_f64, 0.0_f64)],
        },
    ),
    (
        "j_ball_true_0_m10",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="ball" limited="true" range="0 -10"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody></mujoco>"#,
        Expect::Joints {
            limited: &[true],
            range: &[(0.0_f64, -0.17453292519943295_f64)],
        },
    ),
    (
        "j_ball_true_45_0",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="ball" limited="true" range="45 0"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody></mujoco>"#,
        Expect::Refused("range[0] should be 0 in ball joint"),
    ),
    (
        "j_ball_true_norange",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="ball" limited="true"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody></mujoco>"#,
        Expect::Joints {
            limited: &[true],
            range: &[(0.0_f64, 0.0_f64)],
        },
    ),
    (
        "j_ball_true_zero",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="ball" limited="true" range="0 0"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody></mujoco>"#,
        Expect::Joints {
            limited: &[true],
            range: &[(0.0_f64, 0.0_f64)],
        },
    ),
    (
        "j_class_backwards_true",
        r#"<mujoco><default><joint range="1 -1"/></default><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" limited="true"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody></mujoco>"#,
        Expect::Refused("range[0] should be smaller than range[1] in joint"),
    ),
    (
        "j_class_true_backwards",
        r#"<mujoco><default><joint limited="true"/></default><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" range="1 -1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody></mujoco>"#,
        Expect::Refused("range[0] should be smaller than range[1] in joint"),
    ),
    (
        "j_free_auto_range",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="free" range="0 0.5"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody></mujoco>"#,
        Expect::Joints {
            limited: &[false],
            range: &[(0.0_f64, 0.5_f64)],
        },
    ),
    (
        "j_free_backwards",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="free" range="1 -1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody></mujoco>"#,
        Expect::Joints {
            limited: &[false],
            range: &[(1.0_f64, -1.0_f64)],
        },
    ),
    (
        "j_free_class_range",
        r#"<mujoco><default><joint range="0 0.5"/></default><worldbody><body name="b" pos="0 0 1"><joint name="j" type="free"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody></mujoco>"#,
        Expect::Joints {
            limited: &[false],
            range: &[(0.0_f64, 0.5_f64)],
        },
    ),
    (
        "j_free_noauto_range",
        r#"<mujoco><compiler autolimits="false"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="free" range="0 0.5"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody></mujoco>"#,
        Expect::Joints {
            limited: &[false],
            range: &[(0.0_f64, 0.5_f64)],
        },
    ),
    (
        "j_free_true_range",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="free" limited="true" range="0 0.5"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody></mujoco>"#,
        Expect::Joints {
            limited: &[false],
            range: &[(0.0_f64, 0.5_f64)],
        },
    ),
    (
        "j_hinge_auto_backwards",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" range="1 -1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody></mujoco>"#,
        Expect::Automatic,
    ),
    (
        "j_hinge_auto_deg",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" range="-90 90"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody></mujoco>"#,
        Expect::Joints {
            limited: &[true],
            range: &[(-1.5707963267948966_f64, 1.5707963267948966_f64)],
        },
    ),
    (
        "j_hinge_auto_equal",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" range="1 1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody></mujoco>"#,
        Expect::Automatic,
    ),
    (
        "j_hinge_auto_rad",
        r#"<mujoco><compiler angle="radian"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" range="-1 1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody></mujoco>"#,
        Expect::Joints {
            limited: &[true],
            range: &[(-1.0_f64, 1.0_f64)],
        },
    ),
    (
        "j_hinge_auto_zero",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" range="0 0"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody></mujoco>"#,
        Expect::Joints {
            limited: &[false],
            range: &[(0.0_f64, 0.0_f64)],
        },
    ),
    (
        "j_hinge_false_backwards",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" limited="false" range="1 -1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody></mujoco>"#,
        Expect::Joints {
            limited: &[false],
            range: &[(1.0_f64, -1.0_f64)],
        },
    ),
    (
        "j_hinge_false_range_deg",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" limited="false" range="-90 90"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody></mujoco>"#,
        Expect::Joints {
            limited: &[false],
            range: &[(-90.0_f64, 90.0_f64)],
        },
    ),
    (
        "j_hinge_none",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody></mujoco>"#,
        Expect::Joints {
            limited: &[false],
            range: &[(0.0_f64, 0.0_f64)],
        },
    ),
    (
        "j_hinge_true_backwards",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" limited="true" range="1 -1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody></mujoco>"#,
        Expect::Refused("range[0] should be smaller than range[1] in joint"),
    ),
    (
        "j_hinge_true_norange",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" limited="true"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody></mujoco>"#,
        Expect::Refused("range[0] should be smaller than range[1] in joint"),
    ),
    (
        "j_hinge_true_zero",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" limited="true" range="0 0"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody></mujoco>"#,
        Expect::Refused("range[0] should be smaller than range[1] in joint"),
    ),
    (
        "j_noauto_backwards",
        r#"<mujoco><compiler autolimits="false"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" range="1 -1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody></mujoco>"#,
        Expect::Refused(
            "joint has `range` but not `limited`. set the autolimits=\"true\" compiler option, specify `limited` explicitly (\"true\" or \"false\"), or remove the `range` attribute.",
        ),
    ),
    (
        "j_noauto_false",
        r#"<mujoco><compiler autolimits="false"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" limited="false" range="-1 1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody></mujoco>"#,
        Expect::Joints {
            limited: &[false],
            range: &[(-1.0_f64, 1.0_f64)],
        },
    ),
    (
        "j_noauto_range",
        r#"<mujoco><compiler autolimits="false"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" range="-1 1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody></mujoco>"#,
        Expect::Refused(
            "joint has `range` but not `limited`. set the autolimits=\"true\" compiler option, specify `limited` explicitly (\"true\" or \"false\"), or remove the `range` attribute.",
        ),
    ),
    (
        "j_noauto_true",
        r#"<mujoco><compiler autolimits="false"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" limited="true" range="-1 1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody></mujoco>"#,
        Expect::Joints {
            limited: &[true],
            range: &[(-0.017453292519943295_f64, 0.017453292519943295_f64)],
        },
    ),
    (
        "j_noauto_written_auto",
        r#"<mujoco><compiler autolimits="false"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" limited="auto" range="-1 1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody></mujoco>"#,
        Expect::Refused(
            "joint has `range` but not `limited`. set the autolimits=\"true\" compiler option, specify `limited` explicitly (\"true\" or \"false\"), or remove the `range` attribute.",
        ),
    ),
    (
        "j_noauto_zero",
        r#"<mujoco><compiler autolimits="false"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" range="0 0"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody></mujoco>"#,
        Expect::Joints {
            limited: &[false],
            range: &[(0.0_f64, 0.0_f64)],
        },
    ),
    (
        "j_slide_true_backwards",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="slide" axis="1 0 0" limited="true" range="1 -1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody></mujoco>"#,
        Expect::Refused("range[0] should be smaller than range[1] in joint"),
    ),
    (
        "j_slide_true_norange",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="slide" axis="1 0 0" limited="true"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody></mujoco>"#,
        Expect::Refused("range[0] should be smaller than range[1] in joint"),
    ),
];

/// Each document loads, or is refused, as `expected` says.
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
            Expect::Automatic => {
                let error = loaded.err().map(|e| e.to_string()).unwrap_or_default();
                assert!(
                    error.contains("an automatic `") && error.contains("limited` needs"),
                    "{name}: expected the automatic range refused, got '{error}'"
                );
            }
            Expect::Joints { limited, range } => {
                let model = loaded.unwrap_or_else(|e| panic!("{name}: {e}"));
                assert_eq!(model.jnt_limited, *limited, "{name}: jnt_limited");
                assert_eq!(
                    bits(&model.jnt_range),
                    bits(range),
                    "{name}: jnt_range {:?}",
                    model.jnt_range
                );
                model
                    .try_make_data()
                    .unwrap_or_else(|e| panic!("{name}: make_data {e}"));
            }
            Expect::Tendons {
                limited,
                range,
                lengthspring,
            } => {
                let model = loaded.unwrap_or_else(|e| panic!("{name}: {e}"));
                assert_eq!(model.tendon_limited, *limited, "{name}: tendon_limited");
                assert_eq!(
                    bits(&model.tendon_range),
                    bits(range),
                    "{name}: tendon_range"
                );
                let pairs = |s: &[[f64; 2]]| -> Vec<(f64, f64)> {
                    s.iter().map(|&pair| pair.into()).collect()
                };
                assert_eq!(
                    bits(&pairs(&model.tendon_lengthspring)),
                    bits(&pairs(lengthspring)),
                    "{name}: tendon_lengthspring {:?}",
                    model.tendon_lengthspring
                );
                model
                    .try_make_data()
                    .unwrap_or_else(|e| panic!("{name}: make_data {e}"));
            }
        }
    }
}

#[test]
fn each_joint_limit_is_mujocos() {
    check(JOINTS);
}

const TENDONS: &[(&str, &str, Expect)] = &[
    (
        "t_fixed_auto_backwards",
        r#"<mujoco><compiler angle="radian"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" range="-1 1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><tendon><fixed name="t" range="1 0"><joint joint="j" coef="1"/></fixed></tendon></mujoco>"#,
        Expect::Automatic,
    ),
    (
        "t_fixed_auto_equal",
        r#"<mujoco><compiler angle="radian"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" range="-1 1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><tendon><fixed name="t" range="1 1"><joint joint="j" coef="1"/></fixed></tendon></mujoco>"#,
        Expect::Automatic,
    ),
    (
        "t_fixed_auto_ok",
        r#"<mujoco><compiler angle="radian"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" range="-1 1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><tendon><fixed name="t" range="-0.5 0.5"><joint joint="j" coef="1"/></fixed></tendon></mujoco>"#,
        Expect::Tendons {
            limited: &[true],
            range: &[(-0.5_f64, 0.5_f64)],
            lengthspring: &[[0.0_f64, 0.0_f64]],
        },
    ),
    (
        "t_fixed_auto_zero",
        r#"<mujoco><compiler angle="radian"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" range="-1 1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><tendon><fixed name="t" range="0 0"><joint joint="j" coef="1"/></fixed></tendon></mujoco>"#,
        Expect::Tendons {
            limited: &[false],
            range: &[(0.0_f64, 0.0_f64)],
            lengthspring: &[[0.0_f64, 0.0_f64]],
        },
    ),
    (
        "t_fixed_false_backwards",
        r#"<mujoco><compiler angle="radian"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" range="-1 1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><tendon><fixed name="t" limited="false" range="1 -1"><joint joint="j" coef="1"/></fixed></tendon></mujoco>"#,
        Expect::Tendons {
            limited: &[false],
            range: &[(1.0_f64, -1.0_f64)],
            lengthspring: &[[0.0_f64, 0.0_f64]],
        },
    ),
    (
        "t_fixed_noauto_range",
        r#"<mujoco><compiler angle="radian" autolimits="false"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" limited="true" range="-1 1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><tendon><fixed name="t" range="-0.5 0.5"><joint joint="j" coef="1"/></fixed></tendon></mujoco>"#,
        Expect::Refused(
            "tendon has `range` but not `limited`. set the autolimits=\"true\" compiler option, specify `limited` explicitly (\"true\" or \"false\"), or remove the `range` attribute.",
        ),
    ),
    (
        "t_fixed_noauto_zero",
        r#"<mujoco><compiler angle="radian" autolimits="false"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" limited="true" range="-1 1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><tendon><fixed name="t" range="0 0"><joint joint="j" coef="1"/></fixed></tendon></mujoco>"#,
        Expect::Tendons {
            limited: &[false],
            range: &[(0.0_f64, 0.0_f64)],
            lengthspring: &[[0.0_f64, 0.0_f64]],
        },
    ),
    (
        "t_fixed_none",
        r#"<mujoco><compiler angle="radian"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" range="-1 1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><tendon><fixed name="t" ><joint joint="j" coef="1"/></fixed></tendon></mujoco>"#,
        Expect::Tendons {
            limited: &[false],
            range: &[(0.0_f64, 0.0_f64)],
            lengthspring: &[[0.0_f64, 0.0_f64]],
        },
    ),
    (
        "t_fixed_true_backwards",
        r#"<mujoco><compiler angle="radian"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" range="-1 1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><tendon><fixed name="t" limited="true" range="1 -1"><joint joint="j" coef="1"/></fixed></tendon></mujoco>"#,
        Expect::Refused("invalid limits in tendon"),
    ),
    (
        "t_fixed_true_norange",
        r#"<mujoco><compiler angle="radian"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" range="-1 1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><tendon><fixed name="t" limited="true"><joint joint="j" coef="1"/></fixed></tendon></mujoco>"#,
        Expect::Refused("invalid limits in tendon"),
    ),
    (
        "t_fixed_true_zero",
        r#"<mujoco><compiler angle="radian"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" range="-1 1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><tendon><fixed name="t" limited="true" range="0 0"><joint joint="j" coef="1"/></fixed></tendon></mujoco>"#,
        Expect::Refused("invalid limits in tendon"),
    ),
    (
        "t_spatial_auto_backwards",
        r#"<mujoco><worldbody><site name="s0" pos="0 0 1.2"/><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/><site name="s1" pos="0.1 0 0"/></body></worldbody><tendon><spatial name="t" range="1 -1"><site site="s0"/><site site="s1"/></spatial></tendon></mujoco>"#,
        Expect::Automatic,
    ),
    (
        "t_spatial_spring_neg",
        r#"<mujoco><worldbody><site name="s0" pos="0 0 1.2"/><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/><site name="s1" pos="0.1 0 0"/></body></worldbody><tendon><spatial name="t" springlength="-0.5"><site site="s0"/><site site="s1"/></spatial></tendon></mujoco>"#,
        Expect::Tendons {
            limited: &[false],
            range: &[(0.0_f64, 0.0_f64)],
            lengthspring: &[[-0.5_f64, -0.5_f64]],
        },
    ),
    (
        "t_spatial_true_backwards",
        r#"<mujoco><worldbody><site name="s0" pos="0 0 1.2"/><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/><site name="s1" pos="0.1 0 0"/></body></worldbody><tendon><spatial name="t" limited="true" range="1 -1"><site site="s0"/><site site="s1"/></spatial></tendon></mujoco>"#,
        Expect::Refused("invalid limits in tendon"),
    ),
    (
        "t_spring_backwards",
        r#"<mujoco><compiler angle="radian"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" range="-1 1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><tendon><fixed name="t" springlength="0.7 0.3"><joint joint="j" coef="1"/></fixed></tendon></mujoco>"#,
        Expect::Refused("invalid springlength in tendon"),
    ),
    (
        "t_spring_class_backwards",
        r#"<mujoco><compiler angle="radian"/><default><tendon springlength="0.7 0.3"/></default><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" range="-1 1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><tendon><fixed name="t" ><joint joint="j" coef="1"/></fixed></tendon></mujoco>"#,
        Expect::Refused("invalid springlength in tendon"),
    ),
    (
        "t_spring_class_neg",
        r#"<mujoco><compiler angle="radian"/><default><tendon springlength="-0.5"/></default><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" range="-1 1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><tendon><fixed name="t" ><joint joint="j" coef="1"/></fixed></tendon></mujoco>"#,
        Expect::Tendons {
            limited: &[false],
            range: &[(0.0_f64, 0.0_f64)],
            lengthspring: &[[-0.5_f64, -0.5_f64]],
        },
    ),
    (
        "t_spring_m1",
        r#"<mujoco><compiler angle="radian"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" range="-1 1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><tendon><fixed name="t" springlength="-1"><joint joint="j" coef="1"/></fixed></tendon></mujoco>"#,
        Expect::Tendons {
            limited: &[false],
            range: &[(0.0_f64, 0.0_f64)],
            lengthspring: &[[0.0_f64, 0.0_f64]],
        },
    ),
    (
        "t_spring_m1_0",
        r#"<mujoco><compiler angle="radian"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" range="-1 1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><tendon><fixed name="t" springlength="-1 0"><joint joint="j" coef="1"/></fixed></tendon></mujoco>"#,
        Expect::Tendons {
            limited: &[false],
            range: &[(0.0_f64, 0.0_f64)],
            lengthspring: &[[-1.0_f64, 0.0_f64]],
        },
    ),
    (
        "t_spring_m1_m1",
        r#"<mujoco><compiler angle="radian"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" range="-1 1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><tendon><fixed name="t" springlength="-1 -1"><joint joint="j" coef="1"/></fixed></tendon></mujoco>"#,
        Expect::Tendons {
            limited: &[false],
            range: &[(0.0_f64, 0.0_f64)],
            lengthspring: &[[0.0_f64, 0.0_f64]],
        },
    ),
    (
        "t_spring_neg",
        r#"<mujoco><compiler angle="radian"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" range="-1 1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><tendon><fixed name="t" springlength="-0.5"><joint joint="j" coef="1"/></fixed></tendon></mujoco>"#,
        Expect::Tendons {
            limited: &[false],
            range: &[(0.0_f64, 0.0_f64)],
            lengthspring: &[[-0.5_f64, -0.5_f64]],
        },
    ),
    (
        "t_spring_neg_pos",
        r#"<mujoco><compiler angle="radian"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" range="-1 1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><tendon><fixed name="t" springlength="-0.5 0.5"><joint joint="j" coef="1"/></fixed></tendon></mujoco>"#,
        Expect::Tendons {
            limited: &[false],
            range: &[(0.0_f64, 0.0_f64)],
            lengthspring: &[[-0.5_f64, 0.5_f64]],
        },
    ),
];

#[test]
fn each_tendon_limit_and_spring_length_is_mujocos() {
    check(TENDONS);
}
