//! A body's `sleep` policy at load, as MuJoCo 3.5.0's compiler checks it
//! (book L46): a policy other than `auto` only on a moving tree's first body
//! (`user_model.cc:3036-3048`); an explicit `allowed` or `init` on no tree
//! that a tendon coupling it to others reaches (`engine_setconst.c:203-246`);
//! with sleep enabled, every `init` tree put to sleep (`user_model.cc:5159-5185`);
//! each among MuJoCo's checks where MuJoCo makes it. Where each falls among
//! this crate's own refusals, which MuJoCo does not make: ledger L353.
//!
//! Every expected outcome in `REFUSED` and `LOADS` was printed by MuJoCo 3.5.0
//! on the census's unfused oracle (2026-11-27) for the same document: a
//! refusal's message less its prefixes ("Error: ", "engine error:
//! <function>: "), which this crate's is, followed by the body MuJoCo names
//! (its "Element name"), the `init` trees' two lines on one; a load's trees'
//! policies, and each tree's `tree_asleep` once its `Data` is made.
//! `a_keyframe_is_named_before_a_coupled_trees_policy` and
//! `this_crates_own_refusal_is_named_before_an_init_tree` give this crate's
//! words for the faults they name.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use sim_core::SleepPolicy::{Allowed, AutoAllowed, AutoNever, Init, Never};
use sim_mjcf::load_model;

/// A document MuJoCo 3.5.0 refuses at load.
struct Refused {
    what: &'static str,
    xml: &'static str,
    /// MuJoCo's message.
    mujoco: &'static str,
    /// The body MuJoCo's message names.
    body: Option<&'static str>,
}

/// A document MuJoCo 3.5.0 loads.
struct Loads {
    what: &'static str,
    xml: &'static str,
    /// Each tree's policy.
    policies: &'static [sim_core::SleepPolicy],
    /// Each tree's `tree_asleep` once the `Data` is made.
    asleep: &'static [i32],
}

const REFUSED: &[Refused] = &[
    Refused {
        what: "a welded child's never",
        xml: r#"<mujoco><option timestep="0.002"><flag sleep="enable"/></option>
<worldbody><body name="a" pos="0 0 1"><joint name="ja" axis="0 1 0" damping="0.1"/><geom type="sphere" size="0.05" mass="1"/><site name="sa"/><body name="b" pos="0.2 0 0" sleep="never"><geom type="sphere" size="0.05" mass="1"/><site name="sb"/></body></body></worldbody></mujoco>"#,
        mujoco: "sleep policy only allowed for movable root bodies",
        body: Some("b"),
    },
    Refused {
        what: "a grandchild's init",
        xml: r#"<mujoco><option timestep="0.002"><flag sleep="enable"/></option>
<worldbody><body name="a" pos="0 0 1"><joint name="ja" axis="0 1 0" damping="0.1"/><geom type="sphere" size="0.05" mass="1"/><site name="sa"/><body name="b" pos="0.2 0 0"><joint name="jb" axis="0 1 0" damping="0.1"/><geom type="sphere" size="0.05" mass="1"/><site name="sb"/><body name="c" pos="0.2 0 0" sleep="init"><geom type="sphere" size="0.05" mass="1"/><site name="sc"/></body></body></body></worldbody></mujoco>"#,
        mujoco: "sleep policy only allowed for movable root bodies",
        body: Some("c"),
    },
    Refused {
        what: "a static body's never",
        xml: r#"<mujoco><option timestep="0.002"><flag sleep="enable"/></option>
<worldbody><body name="a" pos="0 0 1" sleep="never"><geom type="sphere" size="0.05" mass="1"/><site name="sa"/></body></worldbody></mujoco>"#,
        mujoco: "sleep policy only allowed for movable root bodies",
        body: Some("a"),
    },
    Refused {
        what: "a mocap body's init",
        xml: r#"<mujoco><option timestep="0.002"><flag sleep="enable"/></option>
<worldbody><body name="m" pos="0 0 1" mocap="true" sleep="init"><geom type="sphere" size="0.05" contype="0" conaffinity="0"/></body></worldbody></mujoco>"#,
        mujoco: "sleep policy only allowed for movable root bodies",
        body: Some("m"),
    },
    Refused {
        what: "a welded child's init, sleep disabled",
        xml: r#"<mujoco><option timestep="0.002"/>
<worldbody><body name="a" pos="0 0 1"><joint name="ja" axis="0 1 0" damping="0.1"/><geom type="sphere" size="0.05" mass="1"/><site name="sa"/><body name="b" pos="0.2 0 0" sleep="init"><geom type="sphere" size="0.05" mass="1"/><site name="sb"/></body></body></worldbody></mujoco>"#,
        mujoco: "sleep policy only allowed for movable root bodies",
        body: Some("b"),
    },
    Refused {
        what: "a child's never before a static body's allowed: the first body named",
        xml: r#"<mujoco><option timestep="0.002"><flag sleep="enable"/></option>
<worldbody><body name="a" pos="0 0 1"><joint name="ja" axis="0 1 0" damping="0.1"/><geom type="sphere" size="0.05" mass="1"/><site name="sa"/><body name="b" pos="0.2 0 0" sleep="never"><geom type="sphere" size="0.05" mass="1"/><site name="sb"/></body></body><body name="c" pos="1 0 1" sleep="allowed"><geom type="sphere" size="0.05" mass="1"/><site name="sc"/></body></worldbody></mujoco>"#,
        mujoco: "sleep policy only allowed for movable root bodies",
        body: Some("b"),
    },
    Refused {
        what: "a static body fusestatic keeps (a sensor names it)",
        xml: r#"<mujoco><compiler fusestatic="true"/><option timestep="0.002"><flag sleep="enable"/></option><worldbody><body name="a" pos="0 0 1" sleep="never"><geom type="sphere" size="0.05" mass="1"/><site name="sa"/></body><body name="b" pos="1 0 1"><joint name="jb" axis="0 1 0" damping="0.1"/><geom type="sphere" size="0.05" mass="1"/><site name="sb"/></body></worldbody><sensor><framepos objtype="body" objname="a"/></sensor></mujoco>"#,
        mujoco: "sleep policy only allowed for movable root bodies",
        body: Some("a"),
    },
    Refused {
        what: "a static body's never after a moving tree's body",
        xml: r#"<mujoco><option timestep="0.002"><flag sleep="enable"/></option>
<worldbody><body name="a" pos="0 0 1"><joint name="ja" axis="0 1 0" damping="0.1"/><geom type="sphere" size="0.05" mass="1"/><site name="sa"/></body><body name="s" pos="1 0 1" sleep="never"><geom type="sphere" size="0.05" mass="1"/><site name="ss"/></body></worldbody></mujoco>"#,
        mujoco: "sleep policy only allowed for movable root bodies",
        body: Some("s"),
    },
    Refused {
        what: "a mocap body's init after a moving tree's body",
        xml: r#"<mujoco><option timestep="0.002"><flag sleep="enable"/></option>
<worldbody><body name="a" pos="0 0 1"><joint name="ja" axis="0 1 0" damping="0.1"/><geom type="sphere" size="0.05" mass="1"/><site name="sa"/></body><body name="m" pos="2 0 1" mocap="true" sleep="init"><geom type="sphere" size="0.05" contype="0" conaffinity="0"/></body></worldbody></mujoco>"#,
        mujoco: "sleep policy only allowed for movable root bodies",
        body: Some("m"),
    },
    Refused {
        what: "init on the first of two trees a stiff fixed tendon couples",
        xml: r#"<mujoco><option timestep="0.002"><flag sleep="enable"/></option>
<worldbody><body name="a" pos="0 0 1" sleep="init"><joint name="ja" axis="0 1 0" damping="0.1"/><geom type="sphere" size="0.05" mass="1"/><site name="sa"/></body><body name="b" pos="1 0 1"><joint name="jb" axis="0 1 0" damping="0.1"/><geom type="sphere" size="0.05" mass="1"/><site name="sb"/></body></worldbody><tendon><fixed name="t" stiffness="1"><joint joint="ja" coef="1"/><joint joint="jb" coef="1"/></fixed></tendon></mujoco>"#,
        mujoco: "tree 0 connected to tendon 0 with non-zero stiffness or damping, sleeping not allowed",
        body: None,
    },
    Refused {
        what: "allowed on the second of two trees a damped fixed tendon couples",
        xml: r#"<mujoco><option timestep="0.002"><flag sleep="enable"/></option>
<worldbody><body name="a" pos="0 0 1"><joint name="ja" axis="0 1 0" damping="0.1"/><geom type="sphere" size="0.05" mass="1"/><site name="sa"/></body><body name="b" pos="1 0 1" sleep="allowed"><joint name="jb" axis="0 1 0" damping="0.1"/><geom type="sphere" size="0.05" mass="1"/><site name="sb"/></body></worldbody><tendon><fixed name="t" damping="1"><joint joint="ja" coef="1"/><joint joint="jb" coef="1"/></fixed></tendon></mujoco>"#,
        mujoco: "tree 1 connected to tendon 0 with non-zero stiffness or damping, sleeping not allowed",
        body: None,
    },
    Refused {
        what: "init on a tree a stiff spatial tendon couples",
        xml: r#"<mujoco><option timestep="0.002"><flag sleep="enable"/></option>
<worldbody><body name="a" pos="0 0 1"><joint name="ja" axis="0 1 0" damping="0.1"/><geom type="sphere" size="0.05" mass="1"/><site name="sa"/></body><body name="b" pos="1 0 1" sleep="init"><joint name="jb" axis="0 1 0" damping="0.1"/><geom type="sphere" size="0.05" mass="1"/><site name="sb"/></body></worldbody><tendon><spatial name="t" stiffness="1"><site site="sa"/><site site="sb"/></spatial></tendon></mujoco>"#,
        mujoco: "tree 1 connected to tendon 0 with non-zero stiffness or damping, sleeping not allowed",
        body: None,
    },
    Refused {
        what: "allowed on the third tree of a plain fixed tendon",
        xml: r#"<mujoco><option timestep="0.002"><flag sleep="enable"/></option>
<worldbody><body name="a" pos="0 0 1"><joint name="ja" axis="0 1 0" damping="0.1"/><geom type="sphere" size="0.05" mass="1"/><site name="sa"/></body><body name="b" pos="1 0 1"><joint name="jb" axis="0 1 0" damping="0.1"/><geom type="sphere" size="0.05" mass="1"/><site name="sb"/></body><body name="c" pos="2 0 1" sleep="allowed"><joint name="jc" axis="0 1 0" damping="0.1"/><geom type="sphere" size="0.05" mass="1"/><site name="sc"/></body></worldbody><tendon><fixed name="t"><joint joint="ja" coef="1"/><joint joint="jb" coef="1"/><joint joint="jc" coef="1"/></fixed></tendon></mujoco>"#,
        mujoco: "tree 2 connected to tendon 0 which spans more than 2 trees, sleeping not allowed",
        body: None,
    },
    Refused {
        what: "init on a tree of a spatial tendon over a pulley, three trees",
        xml: r#"<mujoco><option timestep="0.002"><flag sleep="enable"/></option><worldbody><body name="a" pos="0 0 1"><joint name="ja" axis="0 1 0" damping="0.1"/><geom type="sphere" size="0.05" mass="1"/><site name="sa"/></body><body name="b" pos="1 0 1" sleep="init"><joint name="jb" axis="0 1 0" damping="0.1"/><geom type="sphere" size="0.05" mass="1"/><site name="sb"/></body><body name="c" pos="2 0 1"><joint name="jc" axis="0 1 0" damping="0.1"/><geom type="sphere" size="0.05" mass="1"/><site name="sc"/></body></worldbody><tendon><spatial name="t" stiffness="1"><site site="sa"/><site site="sb"/><pulley divisor="2"/><site site="sb"/><site site="sc"/></spatial></tendon></mujoco>"#,
        mujoco: "tree 1 connected to tendon 0 which spans more than 2 trees, sleeping not allowed",
        body: None,
    },
    Refused {
        what: "allowed on a tree of a stiff tendon wrapping a static body too",
        xml: r#"<mujoco><option timestep="0.002"><flag sleep="enable"/></option>
<worldbody><body name="w" pos="0.5 0 1.5"><geom type="sphere" size="0.05" mass="1"/><site name="sw"/></body><body name="a" pos="0 0 1"><joint name="ja" axis="0 1 0" damping="0.1"/><geom type="sphere" size="0.05" mass="1"/><site name="sa"/></body><body name="b" pos="1 0 1" sleep="allowed"><joint name="jb" axis="0 1 0" damping="0.1"/><geom type="sphere" size="0.05" mass="1"/><site name="sb"/></body></worldbody><tendon><spatial name="t" stiffness="1"><site site="sw"/><site site="sa"/><site site="sb"/></spatial></tendon></mujoco>"#,
        mujoco: "tree 1 connected to tendon 0 with non-zero stiffness or damping, sleeping not allowed",
        body: None,
    },
    Refused {
        what: "allowed and init, the wraps reversed: the first tree in wrap order named",
        xml: r#"<mujoco><option timestep="0.002"><flag sleep="enable"/></option>
<worldbody><body name="a" pos="0 0 1" sleep="allowed"><joint name="ja" axis="0 1 0" damping="0.1"/><geom type="sphere" size="0.05" mass="1"/><site name="sa"/></body><body name="b" pos="1 0 1" sleep="init"><joint name="jb" axis="0 1 0" damping="0.1"/><geom type="sphere" size="0.05" mass="1"/><site name="sb"/></body></worldbody><tendon><fixed name="t" stiffness="1"><joint joint="jb" coef="1"/><joint joint="ja" coef="1"/></fixed></tendon></mujoco>"#,
        mujoco: "tree 1 connected to tendon 0 with non-zero stiffness or damping, sleeping not allowed",
        body: None,
    },
    Refused {
        what: "init on a tree the second tendon couples",
        xml: r#"<mujoco><option timestep="0.002"><flag sleep="enable"/></option>
<worldbody><body name="a" pos="0 0 1"><joint name="ja" axis="0 1 0" damping="0.1"/><geom type="sphere" size="0.05" mass="1"/><site name="sa"/></body><body name="b" pos="1 0 1" sleep="init"><joint name="jb" axis="0 1 0" damping="0.1"/><geom type="sphere" size="0.05" mass="1"/><site name="sb"/></body><body name="c" pos="2 0 1"><joint name="jc" axis="0 1 0" damping="0.1"/><geom type="sphere" size="0.05" mass="1"/><site name="sc"/></body></worldbody><tendon><fixed name="t0"><joint joint="ja" coef="1"/><joint joint="jb" coef="1"/></fixed><fixed name="t1" stiffness="1"><joint joint="jb" coef="1"/><joint joint="jc" coef="1"/></fixed></tendon></mujoco>"#,
        mujoco: "tree 1 connected to tendon 1 with non-zero stiffness or damping, sleeping not allowed",
        body: None,
    },
    Refused {
        what: "allowed and init on trees two tendons couple: the first tendon's first flagged tree named",
        xml: r#"<mujoco><option timestep="0.002"><flag sleep="enable"/></option>
<worldbody><body name="a" pos="0 0 1" sleep="allowed"><joint name="ja" axis="0 1 0" damping="0.1"/><geom type="sphere" size="0.05" mass="1"/><site name="sa"/></body><body name="b" pos="1 0 1"><joint name="jb" axis="0 1 0" damping="0.1"/><geom type="sphere" size="0.05" mass="1"/><site name="sb"/></body><body name="c" pos="2 0 1" sleep="init"><joint name="jc" axis="0 1 0" damping="0.1"/><geom type="sphere" size="0.05" mass="1"/><site name="sc"/></body></worldbody><tendon><fixed name="t0" damping="1"><joint joint="jc" coef="1"/><joint joint="jb" coef="1"/></fixed><fixed name="t1" stiffness="1"><joint joint="ja" coef="1"/><joint joint="jb" coef="1"/></fixed></tendon></mujoco>"#,
        mujoco: "tree 2 connected to tendon 0 with non-zero stiffness or damping, sleeping not allowed",
        body: None,
    },
    Refused {
        what: "init on a tree a stiff tendon couples, sleep disabled",
        xml: r#"<mujoco><option timestep="0.002"/>
<worldbody><body name="a" pos="0 0 1"><joint name="ja" axis="0 1 0" damping="0.1"/><geom type="sphere" size="0.05" mass="1"/><site name="sa"/></body><body name="b" pos="1 0 1" sleep="init"><joint name="jb" axis="0 1 0" damping="0.1"/><geom type="sphere" size="0.05" mass="1"/><site name="sb"/></body></worldbody><tendon><fixed name="t" stiffness="1"><joint joint="ja" coef="1"/><joint joint="jb" coef="1"/></fixed></tendon></mujoco>"#,
        mujoco: "tree 1 connected to tendon 0 with non-zero stiffness or damping, sleeping not allowed",
        body: None,
    },
    Refused {
        what: "a child's never and a coupled tree's init: the body named first",
        xml: r#"<mujoco><option timestep="0.002"><flag sleep="enable"/></option>
<worldbody><body name="a" pos="0 0 1"><joint name="ja" axis="0 1 0" damping="0.1"/><geom type="sphere" size="0.05" mass="1"/><site name="sa"/><body name="x" pos="0.2 0 0" sleep="never"><geom type="sphere" size="0.05" mass="1"/><site name="sx"/></body></body><body name="b" pos="1 0 1" sleep="init"><joint name="jb" axis="0 1 0" damping="0.1"/><geom type="sphere" size="0.05" mass="1"/><site name="sb"/></body></worldbody><tendon><fixed name="t" stiffness="1"><joint joint="ja" coef="1"/><joint joint="jb" coef="1"/></fixed></tendon></mujoco>"#,
        mujoco: "sleep policy only allowed for movable root bodies",
        body: Some("x"),
    },
    Refused {
        what: "a massless moving body after a child's never: the mass named first",
        xml: r#"<mujoco><option timestep="0.002"><flag sleep="enable"/></option><worldbody><body name="b" pos="1 0 1"><joint name="jb" axis="0 1 0" damping="0.1"/><geom type="sphere" size="0.05" mass="1"/><site name="sb"/><body name="c" pos="0.2 0 0" sleep="never"><geom type="sphere" size="0.05" mass="1"/><site name="sc"/></body></body><body name="a" pos="0 0 1"><joint name="ja" axis="0 1 0"/><geom type="sphere" size="0.05" mass="0"/></body></worldbody></mujoco>"#,
        mujoco: "mass and inertia of moving bodies must be larger than mjMINVAL",
        body: Some("a"),
    },
    Refused {
        what: "a nested free joint after a child's never: the free joint named first",
        xml: r#"<mujoco><option timestep="0.002"><flag sleep="enable"/></option><worldbody><body name="b" pos="1 0 1"><joint name="jb" axis="0 1 0" damping="0.1"/><geom type="sphere" size="0.05" mass="1"/><site name="sb"/><body name="c" pos="0.2 0 0" sleep="never"><geom type="sphere" size="0.05" mass="1"/><site name="sc"/></body></body><body name="p" pos="0 0 2"><joint name="jp" axis="0 1 0"/><geom type="sphere" size="0.05" mass="1"/><body name="f" pos="0.2 0 0"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></body></worldbody></mujoco>"#,
        mujoco: "free joint can only be used on top level",
        body: Some("f"),
    },
    Refused {
        what: "a child's never and a keyframe of the wrong size: the body named first",
        xml: r#"<mujoco><option timestep="0.002"><flag sleep="enable"/></option><worldbody><body name="a" pos="0 0 1"><joint name="ja" axis="0 1 0" damping="0.1"/><geom type="sphere" size="0.05" mass="1"/><site name="sa"/><body name="c" pos="0.2 0 0" sleep="never"><geom type="sphere" size="0.05" mass="1"/><site name="sc"/></body></body></worldbody><keyframe><key qpos="0 0 0"/></keyframe></mujoco>"#,
        mujoco: "sleep policy only allowed for movable root bodies",
        body: Some("c"),
    },
    Refused {
        what: "init on a coupled tree and a slide after a ball, which this crate refuses: the tree named",
        xml: r#"<mujoco><option timestep="0.002"><flag sleep="enable"/></option><worldbody><body name="a" pos="0 0 1"><joint name="ja" axis="0 1 0" damping="0.1"/><geom type="sphere" size="0.05" mass="1"/><site name="sa"/></body><body name="b" pos="1 0 1" sleep="init"><joint name="jb" type="ball"/><joint name="js" type="slide"/><geom type="sphere" size="0.05" mass="1"/></body></worldbody><tendon><fixed name="t" stiffness="1"><joint joint="ja" coef="1"/><joint joint="js" coef="1"/></fixed></tendon></mujoco>"#,
        mujoco: "tree 1 connected to tendon 0 with non-zero stiffness or damping, sleeping not allowed",
        body: None,
    },
    Refused {
        what: "init on a coupled tree and an actuator whose length range does not converge: the tree named",
        xml: r#"<mujoco><compiler><lengthrange mode="all"/></compiler><option timestep="0.002"><flag sleep="enable"/></option>
<worldbody><body name="a" pos="0 0 1"><joint name="ja" axis="0 1 0" damping="0.1"/><geom type="sphere" size="0.05" mass="1"/><site name="sa"/></body><body name="b" pos="1 0 1" sleep="init"><joint name="jb" axis="0 1 0" damping="0.1"/><geom type="sphere" size="0.05" mass="1"/><site name="sb"/></body><body name="c" pos="2 0 1"><joint name="jc" axis="0 1 0" damping="0.1"/><geom type="sphere" size="0.05" mass="1"/><site name="sc"/></body></worldbody><tendon><fixed name="t" stiffness="1"><joint joint="ja" coef="1"/><joint joint="jb" coef="1"/></fixed></tendon><actuator><position joint="jc"/></actuator></mujoco>"#,
        mujoco: "tree 1 connected to tendon 0 with non-zero stiffness or damping, sleeping not allowed",
        body: None,
    },
    Refused {
        what: "an init tree on an awake one and an actuator whose length range does not converge: the length range named",
        xml: r#"<mujoco><compiler><lengthrange mode="all"/></compiler><option timestep="0.002"><flag sleep="enable"/></option>
<worldbody><geom type="plane" size="5 5 0.1"/><body name="a" pos="0 0 0.0995"><freejoint/><geom type="sphere" size="0.1" mass="1"/></body><body name="b" pos="0 0 0.2985" sleep="init"><freejoint/><geom type="sphere" size="0.1" mass="1"/></body><body name="c" pos="2 0 1"><joint name="jc" axis="0 1 0" damping="0.1"/><geom type="sphere" size="0.05" mass="1"/><site name="sc"/></body></worldbody><actuator><position joint="jc"/></actuator></mujoco>"#,
        mujoco: "Lengthrange computation did not converge in actuator 0:\n  eval (-181.003, -140.808)\n  range (-181.003, 181.003)",
        body: None,
    },
    Refused {
        what: "an init tree resting on an awake one",
        xml: r#"<mujoco><option timestep="0.002"><flag sleep="enable"/></option>
<worldbody><geom type="plane" size="5 5 0.1"/><body name="a" pos="0 0 0.0995"><freejoint/><geom type="sphere" size="0.1" mass="1"/></body><body name="b" pos="0 0 0.2985" sleep="init"><freejoint/><geom type="sphere" size="0.1" mass="1"/></body></worldbody></mujoco>"#,
        mujoco: "1 trees were marked as sleep='init' but only 0 could be slept. Body 'b' (id=2) is the root of the first tree that could not be slept.",
        body: None,
    },
    Refused {
        what: "an init tree joined by a connect to an awake one",
        xml: r#"<mujoco><option timestep="0.002"><flag sleep="enable"/></option>
<worldbody><geom type="plane" size="5 5 0.1"/><body name="a" pos="0 0 0.0995" sleep="init"><freejoint/><geom type="sphere" size="0.1" mass="1"/></body><body name="b" pos="0.3 0 0.0995"><freejoint/><geom type="sphere" size="0.1" mass="1"/></body></worldbody><equality><connect body1="a" body2="b" anchor="0.15 0 0"/></equality></mujoco>"#,
        mujoco: "1 trees were marked as sleep='init' but only 0 could be slept. Body 'a' (id=1) is the root of the first tree that could not be slept.",
        body: None,
    },
    Refused {
        what: "two init trees, each resting on an awake one",
        xml: r#"<mujoco><option timestep="0.002"><flag sleep="enable"/></option>
<worldbody><geom type="plane" size="5 5 0.1"/><body name="a" pos="0 0 0.0995"><freejoint/><geom type="sphere" size="0.1" mass="1"/></body><body name="b" pos="0 0 0.2985" sleep="init"><freejoint/><geom type="sphere" size="0.1" mass="1"/></body><body name="c" pos="2 0 0.0995"><freejoint/><geom type="sphere" size="0.1" mass="1"/></body><body name="d" pos="2 0 0.2985" sleep="init"><freejoint/><geom type="sphere" size="0.1" mass="1"/></body></worldbody></mujoco>"#,
        mujoco: "2 trees were marked as sleep='init' but only 0 could be slept. Body 'b' (id=2) is the root of the first tree that could not be slept.",
        body: None,
    },
    Refused {
        what: "an init tree alone and one resting on an awake one",
        xml: r#"<mujoco><option timestep="0.002"><flag sleep="enable"/></option>
<worldbody><geom type="plane" size="5 5 0.1"/><body name="a" pos="-2 0 0.5" sleep="init"><freejoint/><geom type="sphere" size="0.1" mass="1"/></body><body name="c" pos="2 0 0.0995"><freejoint/><geom type="sphere" size="0.1" mass="1"/></body><body name="d" pos="2 0 0.2985" sleep="init"><freejoint/><geom type="sphere" size="0.1" mass="1"/></body></worldbody></mujoco>"#,
        mujoco: "2 trees were marked as sleep='init' but only 1 could be slept. Body 'd' (id=3) is the root of the first tree that could not be slept.",
        body: None,
    },
    Refused {
        what: "an init tree with no name resting on an awake one",
        xml: r#"<mujoco><option timestep="0.002"><flag sleep="enable"/></option>
<worldbody><geom type="plane" size="5 5 0.1"/><body name="a" pos="0 0 0.0995"><freejoint/><geom type="sphere" size="0.1" mass="1"/></body><body pos="0 0 0.2985" sleep="init"><freejoint/><geom type="sphere" size="0.1" mass="1"/></body></worldbody></mujoco>"#,
        mujoco: "1 trees were marked as sleep='init' but only 0 could be slept. Body '' (id=2) is the root of the first tree that could not be slept.",
        body: None,
    },
];

const LOADS: &[Loads] = &[
    Loads {
        what: "init on a tree's root under a static body",
        xml: r#"<mujoco><option timestep="0.002"><flag sleep="enable"/></option>
<worldbody><body name="a" pos="0 0 1"><geom type="sphere" size="0.05" mass="1"/><site name="sa"/><body name="b" pos="0.2 0 0" sleep="init"><joint name="jb" axis="0 1 0" damping="0.1"/><geom type="sphere" size="0.05" mass="1"/><site name="sb"/></body></body></worldbody></mujoco>"#,
        policies: &[Init],
        asleep: &[0],
    },
    Loads {
        what: "init on a tree's root after a tree whose last body is a child",
        xml: r#"<mujoco><option timestep="0.002"><flag sleep="enable"/></option>
<worldbody><body name="a" pos="0 0 1"><joint name="ja" axis="0 1 0" damping="0.1"/><geom type="sphere" size="0.05" mass="1"/><site name="sa"/><body name="b" pos="0.2 0 0"><joint name="jb" axis="0 1 0" damping="0.1"/><geom type="sphere" size="0.05" mass="1"/><site name="sb"/></body></body><body name="c" pos="1 0 1" sleep="init"><joint name="jc" axis="0 1 0" damping="0.1"/><geom type="sphere" size="0.05" mass="1"/><site name="sc"/></body></worldbody></mujoco>"#,
        policies: &[AutoAllowed, Init],
        asleep: &[-10, 1],
    },
    Loads {
        what: "never on the second of two trees under one static body",
        xml: r#"<mujoco><option timestep="0.002"><flag sleep="enable"/></option>
<worldbody><body name="a" pos="0 0 1"><geom type="sphere" size="0.05" mass="1"/><site name="sa"/><body name="b" pos="0.2 0 0"><joint name="jb" axis="0 1 0" damping="0.1"/><geom type="sphere" size="0.05" mass="1"/><site name="sb"/><body name="bb" pos="0.1 0 0"><geom type="sphere" size="0.05" mass="1"/><site name="sbb"/></body></body><body name="c" pos="-0.2 0 0" sleep="never"><joint name="jc" axis="0 1 0" damping="0.1"/><geom type="sphere" size="0.05" mass="1"/><site name="sc"/></body></body></worldbody></mujoco>"#,
        policies: &[AutoAllowed, Never],
        asleep: &[-11, -11],
    },
    Loads {
        what: "an explicit auto on a welded child",
        xml: r#"<mujoco><option timestep="0.002"><flag sleep="enable"/></option>
<worldbody><body name="a" pos="0 0 1"><joint name="ja" axis="0 1 0" damping="0.1"/><geom type="sphere" size="0.05" mass="1"/><site name="sa"/><body name="b" pos="0.2 0 0" sleep="auto"><geom type="sphere" size="0.05" mass="1"/><site name="sb"/></body></body></worldbody></mujoco>"#,
        policies: &[AutoAllowed],
        asleep: &[-11],
    },
    Loads {
        what: "never on a static body fusestatic fuses",
        xml: r#"<mujoco><compiler fusestatic="true"/><option timestep="0.002"><flag sleep="enable"/></option><worldbody><body name="a" pos="0 0 1" sleep="never"><geom type="sphere" size="0.05" mass="1"/><site name="sa"/></body><body name="b" pos="1 0 1"><joint name="jb" axis="0 1 0" damping="0.1"/><geom type="sphere" size="0.05" mass="1"/><site name="sb"/></body></worldbody></mujoco>"#,
        policies: &[AutoAllowed],
        asleep: &[-11],
    },
    Loads {
        what: "never on a stiff tendon's coupled tree",
        xml: r#"<mujoco><option timestep="0.002"><flag sleep="enable"/></option>
<worldbody><body name="a" pos="0 0 1"><joint name="ja" axis="0 1 0" damping="0.1"/><geom type="sphere" size="0.05" mass="1"/><site name="sa"/></body><body name="b" pos="1 0 1" sleep="never"><joint name="jb" axis="0 1 0" damping="0.1"/><geom type="sphere" size="0.05" mass="1"/><site name="sb"/></body></worldbody><tendon><fixed name="t" stiffness="1"><joint joint="ja" coef="1"/><joint joint="jb" coef="1"/></fixed></tendon></mujoco>"#,
        policies: &[AutoNever, Never],
        asleep: &[-11, -11],
    },
    Loads {
        what: "init on a tree a plain fixed tendon reaches, two trees",
        xml: r#"<mujoco><option timestep="0.002"><flag sleep="enable"/></option>
<worldbody><body name="a" pos="0 0 1"><joint name="ja" axis="0 1 0" damping="0.1"/><geom type="sphere" size="0.05" mass="1"/><site name="sa"/></body><body name="b" pos="1 0 1" sleep="init"><joint name="jb" axis="0 1 0" damping="0.1"/><geom type="sphere" size="0.05" mass="1"/><site name="sb"/></body></worldbody><tendon><fixed name="t"><joint joint="ja" coef="1"/><joint joint="jb" coef="1"/></fixed></tendon></mujoco>"#,
        policies: &[AutoAllowed, Init],
        asleep: &[-10, 1],
    },
    Loads {
        what: "allowed on a tree a limited-only fixed tendon reaches",
        xml: r#"<mujoco><option timestep="0.002"><flag sleep="enable"/></option>
<worldbody><body name="a" pos="0 0 1" sleep="allowed"><joint name="ja" axis="0 1 0" damping="0.1"/><geom type="sphere" size="0.05" mass="1"/><site name="sa"/></body><body name="b" pos="1 0 1"><joint name="jb" axis="0 1 0" damping="0.1"/><geom type="sphere" size="0.05" mass="1"/><site name="sb"/></body></worldbody><tendon><fixed name="t" limited="true" range="-1 1"><joint joint="ja" coef="1"/><joint joint="jb" coef="1"/></fixed></tendon></mujoco>"#,
        policies: &[Allowed, AutoAllowed],
        asleep: &[-11, -11],
    },
    Loads {
        what: "init on a tree a fixed tendon with a spring length only reaches",
        xml: r#"<mujoco><option timestep="0.002"><flag sleep="enable"/></option>
<worldbody><body name="a" pos="0 0 1" sleep="init"><joint name="ja" axis="0 1 0" damping="0.1"/><geom type="sphere" size="0.05" mass="1"/><site name="sa"/></body><body name="b" pos="1 0 1"><joint name="jb" axis="0 1 0" damping="0.1"/><geom type="sphere" size="0.05" mass="1"/><site name="sb"/></body></worldbody><tendon><fixed name="t" springlength="0.5"><joint joint="ja" coef="1"/><joint joint="jb" coef="1"/></fixed></tendon></mujoco>"#,
        policies: &[Init, AutoAllowed],
        asleep: &[0, -10],
    },
    Loads {
        what: "init on a tree a stiff tendon within it reaches",
        xml: r#"<mujoco><option timestep="0.002"><flag sleep="enable"/></option>
<worldbody><body name="a" pos="0 0 1" sleep="init"><joint name="ja" axis="0 1 0" damping="0.1"/><geom type="sphere" size="0.05" mass="1"/><site name="sa"/><body name="b" pos="0.2 0 0"><joint name="jb" axis="0 1 0" damping="0.1"/><geom type="sphere" size="0.05" mass="1"/><site name="sb"/></body></body></worldbody><tendon><fixed name="t" stiffness="1"><joint joint="ja" coef="1"/><joint joint="jb" coef="1"/></fixed></tendon></mujoco>"#,
        policies: &[Init],
        asleep: &[0],
    },
    Loads {
        what: "allowed on a tree an actuated plain tendon reaches",
        xml: r#"<mujoco><option timestep="0.002"><flag sleep="enable"/></option>
<worldbody><body name="a" pos="0 0 1"><joint name="ja" axis="0 1 0" damping="0.1"/><geom type="sphere" size="0.05" mass="1"/><site name="sa"/></body><body name="b" pos="1 0 1" sleep="allowed"><joint name="jb" axis="0 1 0" damping="0.1"/><geom type="sphere" size="0.05" mass="1"/><site name="sb"/></body></worldbody><tendon><fixed name="t"><joint joint="ja" coef="1"/><joint joint="jb" coef="1"/></fixed></tendon><actuator><motor tendon="t"/></actuator></mujoco>"#,
        policies: &[AutoNever, Allowed],
        asleep: &[-11, -11],
    },
    Loads {
        what: "init on the one moving tree of a stiff tendon wrapping a static body",
        xml: r#"<mujoco><option timestep="0.002"><flag sleep="enable"/></option>
<worldbody><body name="w" pos="0.5 0 1.5"><geom type="sphere" size="0.05" mass="1"/><site name="sw"/></body><body name="a" pos="0 0 1"><joint name="ja" axis="0 1 0" damping="0.1"/><geom type="sphere" size="0.05" mass="1"/><site name="sa"/></body><body name="b" pos="1 0 1" sleep="init"><joint name="jb" axis="0 1 0" damping="0.1"/><geom type="sphere" size="0.05" mass="1"/><site name="sb"/></body></worldbody><tendon><spatial name="t" stiffness="1"><site site="sw"/><site site="sb"/></spatial></tendon></mujoco>"#,
        policies: &[AutoAllowed, Init],
        asleep: &[-10, 1],
    },
    Loads {
        what: "init on a tree a tendon of stiffness and damping -0 reaches",
        xml: r#"<mujoco><option timestep="0.002"><flag sleep="enable"/></option><worldbody><body name="a" pos="0 0 1"><joint name="ja" axis="0 1 0" damping="0.1"/><geom type="sphere" size="0.05" mass="1"/><site name="sa"/></body><body name="b" pos="1 0 1" sleep="init"><joint name="jb" axis="0 1 0" damping="0.1"/><geom type="sphere" size="0.05" mass="1"/><site name="sb"/></body></worldbody><tendon><fixed name="t" stiffness="-0" damping="-0"><joint joint="ja" coef="1"/><joint joint="jb" coef="1"/></fixed></tendon></mujoco>"#,
        policies: &[AutoAllowed, Init],
        asleep: &[-10, 1],
    },
    Loads {
        what: "never on all three trees of a plain fixed tendon",
        xml: r#"<mujoco><option timestep="0.002"><flag sleep="enable"/></option>
<worldbody><body name="a" pos="0 0 1" sleep="never"><joint name="ja" axis="0 1 0" damping="0.1"/><geom type="sphere" size="0.05" mass="1"/><site name="sa"/></body><body name="b" pos="1 0 1" sleep="never"><joint name="jb" axis="0 1 0" damping="0.1"/><geom type="sphere" size="0.05" mass="1"/><site name="sb"/></body><body name="c" pos="2 0 1" sleep="never"><joint name="jc" axis="0 1 0" damping="0.1"/><geom type="sphere" size="0.05" mass="1"/><site name="sc"/></body></worldbody><tendon><fixed name="t"><joint joint="ja" coef="1"/><joint joint="jb" coef="1"/><joint joint="jc" coef="1"/></fixed></tendon></mujoco>"#,
        policies: &[Never, Never, Never],
        asleep: &[-11, -11, -11],
    },
    Loads {
        what: "an init tree resting on the floor",
        xml: r#"<mujoco><option timestep="0.002"><flag sleep="enable"/></option>
<worldbody><geom type="plane" size="5 5 0.1"/><body name="a" pos="0 0 0.0995" sleep="init"><freejoint/><geom type="sphere" size="0.1" mass="1"/></body></worldbody></mujoco>"#,
        policies: &[Init],
        asleep: &[0],
    },
    Loads {
        what: "two init trees resting one on the other",
        xml: r#"<mujoco><option timestep="0.002"><flag sleep="enable"/></option>
<worldbody><geom type="plane" size="5 5 0.1"/><body name="a" pos="0 0 0.0995" sleep="init"><freejoint/><geom type="sphere" size="0.1" mass="1"/></body><body name="b" pos="0 0 0.2985" sleep="init"><freejoint/><geom type="sphere" size="0.1" mass="1"/></body></worldbody></mujoco>"#,
        policies: &[Init, Init],
        asleep: &[1, 0],
    },
    Loads {
        what: "an actuated init tree",
        xml: r#"<mujoco><option timestep="0.002"><flag sleep="enable"/></option>
<worldbody><body name="a" pos="0 0 1" sleep="init"><joint name="ja" axis="0 1 0" damping="0.1"/><geom type="sphere" size="0.05" mass="1"/><site name="sa"/></body></worldbody><actuator><motor joint="ja"/></actuator></mujoco>"#,
        policies: &[Init],
        asleep: &[0],
    },
    Loads {
        what: "an init tree resting on an awake one, sleep disabled",
        xml: r#"<mujoco><option timestep="0.002"/>
<worldbody><geom type="plane" size="5 5 0.1"/><body name="a" pos="0 0 0.0995"><freejoint/><geom type="sphere" size="0.1" mass="1"/></body><body name="b" pos="0 0 0.2985" sleep="init"><freejoint/><geom type="sphere" size="0.1" mass="1"/></body></worldbody></mujoco>"#,
        policies: &[AutoAllowed, Init],
        asleep: &[-11, -11],
    },
];

/// Each document MuJoCo refuses is refused with MuJoCo's message, the body it
/// names given as this crate gives a body.
#[test]
fn each_document_mujoco_refuses_is_refused_with_its_message() {
    for case in REFUSED {
        let ours = load_model(case.xml).expect_err(case.what).to_string();
        let want = case.body.map_or_else(
            || case.mujoco.to_owned(),
            |body| format!("{} (body '{body}')", case.mujoco),
        );
        assert_eq!(ours, want, "{}", case.what);
    }
}

/// Each document MuJoCo loads is loaded with its trees' policies, and its
/// `Data` made with each tree asleep or awake as MuJoCo's.
#[test]
fn each_document_mujoco_loads_is_loaded_with_its_policies() {
    for case in LOADS {
        let model = load_model(case.xml).expect(case.what);
        assert_eq!(model.tree_sleep_policy, case.policies, "{}", case.what);
        let data = model.try_make_data().expect(case.what);
        assert_eq!(data.tree_asleep, case.asleep, "{}", case.what);
    }
}

/// A keyframe of the wrong size is named before a coupled tree's `init`, as
/// MuJoCo compiles the keyframes before `mj_setConst`
/// (`user_model.cc:5028`, `:5126`): MuJoCo's message is "keyframe '':
/// invalid qpos size, expected 2, got 3".
#[test]
fn a_keyframe_is_named_before_a_coupled_trees_policy() {
    let ours = load_model(KEY_THEN_COUPLED)
        .expect_err("refused")
        .to_string();
    assert_eq!(
        ours,
        "invalid qpos on <key>: length 3 does not match 2 (key #0)"
    );
}

/// An `init` tree that cannot sleep beside a refusal this crate makes and
/// MuJoCo does not: this crate names its refusal, MuJoCo the tree ("1 trees
/// were marked as sleep='init' but only 0 could be slept.", then "Body 'b'
/// (id=2) is the root of the first tree that could not be slept.", for each
/// document; ledger L353).
#[test]
fn this_crates_own_refusal_is_named_before_an_init_tree() {
    for (xml, ours) in [
        (
            INIT_BESIDE_BALL_SLIDE,
            "not supported by sim-mjcf (valid in MuJoCo 3.5.0): a slide joint after a ball joint \
             on one body (body 'c')",
        ),
        (
            INIT_BESIDE_NEGATIVE_ARMATURE,
            "invalid armature on <tendon>: tendon armature cannot be negative (tendon 't')",
        ),
        (
            INIT_BESIDE_NEGATIVE_INERTIAL,
            "invalid mass on <inertial>: must not be negative, got -1 (MuJoCo raises it to \
             boundmass, D-INERTIAL-NEGATIVE) (body 'c')",
        ),
    ] {
        assert_eq!(load_model(xml).expect_err("refused").to_string(), ours);
    }
}

const KEY_THEN_COUPLED: &str = r#"<mujoco><option timestep="0.002"><flag sleep="enable"/></option><worldbody><body name="a" pos="0 0 1"><joint name="ja" axis="0 1 0" damping="0.1"/><geom type="sphere" size="0.05" mass="1"/><site name="sa"/></body><body name="b" pos="1 0 1" sleep="init"><joint name="jb" axis="0 1 0" damping="0.1"/><geom type="sphere" size="0.05" mass="1"/><site name="sb"/></body></worldbody><tendon><fixed name="t" stiffness="1"><joint joint="ja" coef="1"/><joint joint="jb" coef="1"/></fixed></tendon><keyframe><key qpos="0 0 0"/></keyframe></mujoco>"#;

const INIT_BESIDE_BALL_SLIDE: &str = r#"<mujoco><option timestep="0.002"><flag sleep="enable"/></option>
<worldbody><geom type="plane" size="5 5 0.1"/><body name="a" pos="0 0 0.0995"><freejoint/><geom type="sphere" size="0.1" mass="1"/></body><body name="b" pos="0 0 0.2985" sleep="init"><freejoint/><geom type="sphere" size="0.1" mass="1"/></body><body name="c" pos="2 0 1"><joint name="jc1" type="ball"/><joint name="jc2" type="slide"/><geom type="sphere" size="0.05" mass="1"/></body></worldbody></mujoco>"#;

const INIT_BESIDE_NEGATIVE_ARMATURE: &str = r#"<mujoco><option timestep="0.002"><flag sleep="enable"/></option>
<worldbody><geom type="plane" size="5 5 0.1"/><body name="a" pos="0 0 0.0995"><freejoint/><geom type="sphere" size="0.1" mass="1"/></body><body name="b" pos="0 0 0.2985" sleep="init"><freejoint/><geom type="sphere" size="0.1" mass="1"/></body><body name="c" pos="2 0 1"><joint name="jc" axis="0 1 0" damping="0.1"/><geom type="sphere" size="0.05" mass="1"/><site name="sc"/></body></worldbody><tendon><fixed name="t" armature="-1"><joint joint="jc" coef="1"/></fixed></tendon></mujoco>"#;

const INIT_BESIDE_NEGATIVE_INERTIAL: &str = r#"<mujoco><option timestep="0.002"><flag sleep="enable"/></option>
<worldbody><geom type="plane" size="5 5 0.1"/><body name="a" pos="0 0 0.0995"><freejoint/><geom type="sphere" size="0.1" mass="1"/></body><body name="b" pos="0 0 0.2985" sleep="init"><freejoint/><geom type="sphere" size="0.1" mass="1"/></body><body name="c" pos="2 0 1"><inertial pos="0 0 0" mass="-1" diaginertia="1 1 1"/></body></worldbody></mujoco>"#;
