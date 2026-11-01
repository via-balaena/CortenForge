//! A joint's and a tendon's `limited` and `range`, a tendon's
//! `springlength`, an actuator's control, force and activation limits and a
//! muscle's parameters, as MuJoCo 3.5.0 resolves and checks them (book L21;
//! ledger L1, L55, L69, L272, L331, L349, L350; `builder/limits.rs`).
//!
//! Every document below was loaded by MuJoCo 3.5.0 on the census's unfused
//! oracle (2026-11-01): each refusal expects MuJoCo's words; each load,
//! MuJoCo's `jnt_limited` and `jnt_range`, or `tendon_limited`,
//! `tendon_range` and `tendon_lengthspring`, or an actuator's limited ranges,
//! bit for bit (an unlimited control or force range, which MuJoCo stores as
//! written, is unbounded here; a range is the
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
    /// MuJoCo loads it with these actuators' ranges, `None` where unlimited.
    Actuators {
        ctrl: &'static [Option<(f64, f64)>],
        force: &'static [Option<(f64, f64)>],
        act: &'static [Option<(f64, f64)>],
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
            Expect::Actuators { ctrl, force, act } => {
                let model = loaded.unwrap_or_else(|e| panic!("{name}: {e}"));
                let unbounded = (f64::NEG_INFINITY, f64::INFINITY);
                let clamp = |r: &[Option<(f64, f64)>]| -> Vec<(f64, f64)> {
                    r.iter().map(|r| r.unwrap_or(unbounded)).collect()
                };
                assert_eq!(
                    bits(&model.actuator_ctrlrange),
                    bits(&clamp(ctrl)),
                    "{name}: ctrlrange"
                );
                assert_eq!(
                    bits(&model.actuator_forcerange),
                    bits(&clamp(force)),
                    "{name}: forcerange"
                );
                let limited: Vec<bool> = act.iter().map(Option::is_some).collect();
                assert_eq!(model.actuator_actlimited, limited, "{name}: actlimited");
                let ranges: Vec<(f64, f64)> = act.iter().map(|r| r.unwrap_or((0.0, 0.0))).collect();
                assert_eq!(
                    bits(&model.actuator_actrange),
                    bits(&ranges),
                    "{name}: actrange"
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

const ACTUATORS: &[(&str, &str, Expect)] = &[
    (
        "a_act_auto_backwards",
        r#"<mujoco><compiler angle="radian"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" range="-1 1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><actuator><general joint="j" dyntype="integrator" actrange="1 -1"/></actuator></mujoco>"#,
        Expect::Automatic,
    ),
    (
        "a_act_auto_dyn_none",
        r#"<mujoco><compiler angle="radian"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" range="-1 1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><actuator><general joint="j" actrange="0 1"/></actuator></mujoco>"#,
        Expect::Refused("actrange specified but dyntype is 'none' in actuator"),
    ),
    (
        "a_act_dyn_none",
        r#"<mujoco><compiler angle="radian"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" range="-1 1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><actuator><general joint="j" actlimited="true" actrange="0 1"/></actuator></mujoco>"#,
        Expect::Refused("actrange specified but dyntype is 'none' in actuator"),
    ),
    (
        "a_act_true_backwards",
        r#"<mujoco><compiler angle="radian"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" range="-1 1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><actuator><general joint="j" dyntype="integrator" actlimited="true" actrange="1 -1"/></actuator></mujoco>"#,
        Expect::Refused("invalid actrange for actuator"),
    ),
    (
        "a_act_true_none",
        r#"<mujoco><compiler angle="radian"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" range="-1 1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><actuator><general joint="j" dyntype="integrator" actlimited="true"/></actuator></mujoco>"#,
        Expect::Refused("invalid actrange for actuator"),
    ),
    (
        "a_adhesion_none",
        r#"<mujoco><compiler angle="radian"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" range="-1 1"/><geom type="sphere" size="0.1"/></body></worldbody><actuator><adhesion body="b" gain="1"/></actuator></mujoco>"#,
        Expect::Refused("invalid control range for actuator"),
    ),
    (
        "a_class_damper_motor",
        r#"<mujoco><compiler angle="radian"/><default><default class="c"><damper kv="4"/></default></default><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" range="-1 1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><actuator><motor class="c" joint="j"/></actuator></mujoco>"#,
        Expect::Refused("invalid control range for actuator"),
    ),
    (
        "a_ctrl_auto_backwards",
        r#"<mujoco><compiler angle="radian"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" range="-1 1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><actuator><motor joint="j" ctrlrange="1 -1"/></actuator></mujoco>"#,
        Expect::Automatic,
    ),
    (
        "a_ctrl_auto_equal",
        r#"<mujoco><compiler angle="radian"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" range="-1 1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><actuator><motor joint="j" ctrlrange="1 1"/></actuator></mujoco>"#,
        Expect::Automatic,
    ),
    (
        "a_ctrl_auto_zero",
        r#"<mujoco><compiler angle="radian"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" range="-1 1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><actuator><motor joint="j" ctrlrange="0 0"/></actuator></mujoco>"#,
        Expect::Actuators {
            ctrl: &[None],
            force: &[None],
            act: &[None],
        },
    ),
    (
        "a_ctrl_true_backwards",
        r#"<mujoco><compiler angle="radian"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" range="-1 1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><actuator><motor joint="j" ctrllimited="true" ctrlrange="1 -1"/></actuator></mujoco>"#,
        Expect::Refused("invalid control range for actuator"),
    ),
    (
        "a_ctrl_true_none",
        r#"<mujoco><compiler angle="radian"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" range="-1 1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><actuator><motor joint="j" ctrllimited="true"/></actuator></mujoco>"#,
        Expect::Refused("invalid control range for actuator"),
    ),
    (
        "a_damper_negative",
        r#"<mujoco><compiler angle="radian"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" range="-1 1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><actuator><damper joint="j" kv="1" ctrlrange="-1 1"/></actuator></mujoco>"#,
        Expect::Refused("damper control range cannot be negative"),
    ),
    (
        "a_damper_none",
        r#"<mujoco><compiler angle="radian"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" range="-1 1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><actuator><damper joint="j" kv="1"/></actuator></mujoco>"#,
        Expect::Refused("invalid control range for actuator"),
    ),
    (
        "a_damper_ok",
        r#"<mujoco><compiler angle="radian"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" range="-1 1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><actuator><damper joint="j" kv="1" ctrlrange="0 1"/></actuator></mujoco>"#,
        Expect::Actuators {
            ctrl: &[Some((0.0_f64, 1.0_f64))],
            force: &[None],
            act: &[None],
        },
    ),
    (
        "a_force_auto_backwards",
        r#"<mujoco><compiler angle="radian"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" range="-1 1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><actuator><motor joint="j" forcerange="1 -1"/></actuator></mujoco>"#,
        Expect::Automatic,
    ),
    (
        "a_force_true_backwards",
        r#"<mujoco><compiler angle="radian"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" range="-1 1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><actuator><motor joint="j" forcelimited="true" forcerange="1 -1"/></actuator></mujoco>"#,
        Expect::Refused("invalid force range for actuator"),
    ),
    (
        "a_force_true_none",
        r#"<mujoco><compiler angle="radian"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" range="-1 1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><actuator><motor joint="j" forcelimited="true"/></actuator></mujoco>"#,
        Expect::Refused("invalid force range for actuator"),
    ),
    (
        "a_intvel_none",
        r#"<mujoco><compiler angle="radian"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" range="-1 1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><actuator><intvelocity joint="j" kp="1"/></actuator></mujoco>"#,
        Expect::Refused("invalid actrange for actuator"),
    ),
    (
        "a_intvel_ok",
        r#"<mujoco><compiler angle="radian"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" range="-1 1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><actuator><intvelocity joint="j" kp="1" actrange="-1 1"/></actuator></mujoco>"#,
        Expect::Actuators {
            ctrl: &[None],
            force: &[None],
            act: &[Some((-1.0_f64, 1.0_f64))],
        },
    ),
    (
        "a_motor_plain",
        r#"<mujoco><compiler angle="radian"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" range="-1 1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><actuator><motor joint="j"/></actuator></mujoco>"#,
        Expect::Actuators {
            ctrl: &[None],
            force: &[None],
            act: &[None],
        },
    ),
    (
        "a_noauto_act",
        r#"<mujoco><compiler angle="radian" autolimits="false"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" limited="true" range="-1 1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><actuator><general joint="j" dyntype="integrator" actrange="-1 1"/></actuator></mujoco>"#,
        Expect::Refused(
            "actuator has `actrange` but not `actlimited`. set the autolimits=\"true\" compiler option, specify `actlimited` explicitly (\"true\" or \"false\"), or remove the `actrange` attribute.",
        ),
    ),
    (
        "a_noauto_ctrl",
        r#"<mujoco><compiler angle="radian" autolimits="false"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" limited="true" range="-1 1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><actuator><motor joint="j" ctrlrange="-1 1"/></actuator></mujoco>"#,
        Expect::Refused(
            "actuator has `ctrlrange` but not `ctrllimited`. set the autolimits=\"true\" compiler option, specify `ctrllimited` explicitly (\"true\" or \"false\"), or remove the `ctrlrange` attribute.",
        ),
    ),
    (
        "a_noauto_force",
        r#"<mujoco><compiler angle="radian" autolimits="false"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" limited="true" range="-1 1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><actuator><motor joint="j" forcerange="-1 1"/></actuator></mujoco>"#,
        Expect::Refused(
            "actuator has `forcerange` but not `forcelimited`. set the autolimits=\"true\" compiler option, specify `forcelimited` explicitly (\"true\" or \"false\"), or remove the `forcerange` attribute.",
        ),
    ),
    (
        "a_noauto_order",
        r#"<mujoco><compiler angle="radian" autolimits="false"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" limited="true" range="-1 1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><actuator><motor joint="j" ctrlrange="-1 1" forcerange="-1 1"/></actuator></mujoco>"#,
        Expect::Refused(
            "actuator has `forcerange` but not `forcelimited`. set the autolimits=\"true\" compiler option, specify `forcelimited` explicitly (\"true\" or \"false\"), or remove the `forcerange` attribute.",
        ),
    ),
    (
        "a_position_auto_backwards",
        r#"<mujoco><compiler angle="radian"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" range="-1 1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><actuator><position joint="j" kp="1" ctrlrange="1 -1"/></actuator></mujoco>"#,
        Expect::Automatic,
    ),
];

const MUSCLES: &[(&str, &str, Expect)] = &[
    (
        "m_fpmax_0",
        r#"<mujoco><compiler angle="radian"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" range="-1 1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><actuator><muscle joint="j" fpmax="0"/></actuator></mujoco>"#,
        Expect::Refused("positive scale, vmax, fpmax, fvmax required in muscle"),
    ),
    (
        "m_fvmax_0",
        r#"<mujoco><compiler angle="radian"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" range="-1 1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><actuator><muscle joint="j" fvmax="0"/></actuator></mujoco>"#,
        Expect::Refused("positive scale, vmax, fpmax, fvmax required in muscle"),
    ),
    (
        "m_general_actlimited",
        r#"<mujoco><compiler angle="radian"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" range="-1 1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><actuator><general joint="j" dyntype="muscle" gaintype="muscle" biastype="muscle" actlimited="true" actrange="0 1" gainprm="0.75 1.05 -1 200 0.5 1.6 1.5 1.3 1.2" biasprm="0.75 1.05 -1 200 0.5 1.6 1.5 1.3 1.2"/></actuator></mujoco>"#,
        Expect::Actuators {
            ctrl: &[None],
            force: &[None],
            act: &[Some((0.0_f64, 1.0_f64))],
        },
    ),
    (
        "m_general_bias_backwards",
        r#"<mujoco><compiler angle="radian"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" range="-1 1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><actuator><general joint="j" dyntype="muscle" gaintype="muscle" biastype="muscle" gainprm="0.75 1.05 -1 200 0.5 1.6 1.5 1.3 1.2" biasprm="1 0.5 -1 200 0.5 1.6 1.5 1.3 1.2"/></actuator></mujoco>"#,
        Expect::Refused("range[0]<range[1] required in muscle"),
    ),
    (
        "m_general_gain_backwards",
        r#"<mujoco><compiler angle="radian"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" range="-1 1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><actuator><general joint="j" dyntype="muscle" gaintype="muscle" biastype="muscle" gainprm="1 0.5 -1 200 0.5 1.6 1.5 1.3 1.2" biasprm="0.75 1.05 -1 200 0.5 1.6 1.5 1.3 1.2"/></actuator></mujoco>"#,
        Expect::Refused("range[0]<range[1] required in muscle"),
    ),
    (
        "m_general_gain_only",
        r#"<mujoco><compiler angle="radian"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" range="-1 1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><actuator><general joint="j" gaintype="muscle" gainprm="0.75 1.05 -1 0 0.5 1.6 1.5 1.3 1.2"/></actuator></mujoco>"#,
        Expect::Refused("positive scale, vmax, fpmax, fvmax required in muscle"),
    ),
    (
        "m_lmax_09",
        r#"<mujoco><compiler angle="radian"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" range="-1 1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><actuator><muscle joint="j" lmax="0.9"/></actuator></mujoco>"#,
        Expect::Refused("lmin<1<lmax required in muscle"),
    ),
    (
        "m_lmin_12",
        r#"<mujoco><compiler angle="radian"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" range="-1 1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><actuator><muscle joint="j" lmin="1.2"/></actuator></mujoco>"#,
        Expect::Refused("lmin<1<lmax required in muscle"),
    ),
    (
        "m_order_ctrl_then_muscle",
        r#"<mujoco><compiler angle="radian"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" range="-1 1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><actuator><muscle joint="j" ctrllimited="true" ctrlrange="1 -1" lmin="1.2"/></actuator></mujoco>"#,
        Expect::Refused("invalid control range for actuator"),
    ),
    (
        "m_range_backwards",
        r#"<mujoco><compiler angle="radian"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" range="-1 1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><actuator><muscle joint="j" range="1 0.5"/></actuator></mujoco>"#,
        Expect::Refused("range[0]<range[1] required in muscle"),
    ),
    (
        "m_scale_0",
        r#"<mujoco><compiler angle="radian"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" range="-1 1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><actuator><muscle joint="j" scale="0"/></actuator></mujoco>"#,
        Expect::Refused("positive scale, vmax, fpmax, fvmax required in muscle"),
    ),
    (
        "m_tausmooth_neg",
        r#"<mujoco><compiler angle="radian"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" range="-1 1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><actuator><muscle joint="j" tausmooth="-1"/></actuator></mujoco>"#,
        Expect::Refused("muscle tausmooth cannot be negative"),
    ),
    (
        "m_unwritten_actlimited",
        r#"<mujoco><compiler angle="radian"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" range="-1 1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><actuator><muscle joint="j"/></actuator></mujoco>"#,
        Expect::Actuators {
            ctrl: &[None],
            force: &[None],
            act: &[None],
        },
    ),
    (
        "m_vmax_0",
        r#"<mujoco><compiler angle="radian"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" range="-1 1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><actuator><muscle joint="j" vmax="0"/></actuator></mujoco>"#,
        Expect::Refused("positive scale, vmax, fpmax, fvmax required in muscle"),
    ),
];

#[test]
fn each_actuator_limit_is_mujocos() {
    check(ACTUATORS);
}

#[test]
fn each_muscle_parameter_check_is_mujocos() {
    check(MUSCLES);
}
