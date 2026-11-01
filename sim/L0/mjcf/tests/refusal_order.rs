//! Which fault a document with two is refused for, where RL-6a's refusals
//! meet the ones this crate made before (book L19, L20; ledger L164), and
//! where RL-6b's limit checks meet both (book L21; ledger L363).
//!
//! RL-6a's refusals that MuJoCo 3.5.0 makes (a joint's axis, a body's joint
//! layout, a tendon's armature, a free joint below the world's children) run
//! after every check this crate makes before building the model, so that a
//! document it refused before is refused for the same fault, as MuJoCo
//! refuses it here; a keyframe's fault comes after them, as in MuJoCo. The
//! two that MuJoCo does not make (a slide after a ball, a fixed tendon's
//! negative armature) run after the keyframes, which this crate resolves
//! once the model is built. Where MuJoCo orders a new refusal before a check this crate makes
//! later, this crate names the later one, as it did before.
//!
//! Every document below was refused by MuJoCo 3.5.0, on the census's unfused
//! oracle (2026-10-31), for the fault its expected error names. Eleven are the
//! RL-6a review's first round, three its fix-diff pass's (a keyframe's fault),
//! and four set the new checks against each other; the rest are RL-6b's, each
//! a limit's fault beside another, and MuJoCo checks a joint's limit in the
//! joint's compile, before its axis and its body's degrees of freedom.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use sim_mjcf::load_model;

/// `(what, document, MuJoCo's reason, sim-mjcf's error for that fault)`.
const TWO_FAULTS: &[(&str, &str, &str, &str)] = &[
    (
        "axis zero user",
        r#"<mujoco><size nuser_jnt="1"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 0 0" user="1 2"/><geom type="sphere" size="0.1"/></body></worldbody></mujoco>"#,
        "user has more values than nuser_jnt in joint",
        "invalid user on <joint>: user data exceeds nuser_jnt=1",
    ),
    (
        "ballslide then later seven",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><joint type="ball"/><joint type="slide" axis="1 0 0"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body><body name="c" pos="1 0 1"><joint name="h0" type="hinge" axis="0 1 0" /><joint name="h1" type="hinge" axis="0 1 0" /><joint name="h2" type="hinge" axis="0 1 0" /><joint name="h3" type="hinge" axis="0 1 0" /><joint name="h4" type="hinge" axis="0 1 0" /><joint name="h5" type="hinge" axis="0 1 0" /><joint name="h6" type="hinge" axis="0 1 0" /><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody></mujoco>"#,
        "more than 6 dofs in body 'c'",
        "more than 6 dofs (body 'c')",
    ),
    (
        "mesh shellinertia zero axis",
        r#"<mujoco><asset><mesh name="m" vertex="0 0 0 1 0 0 0 1 0 0 0 1" face="0 2 1 0 1 3 0 3 2 1 2 3"/></asset><worldbody><body name="b" pos="0 0 1"><joint name="z" type="hinge" axis="0 0 0"/><geom type="mesh" mesh="m" shellinertia="true"/></body></worldbody></mujoco>"#,
        "for mesh geoms, inertia should be specified in the mesh asset",
        "invalid shellinertia on <geom>: for mesh geoms, inertia should be specified in the mesh asset (geom #0 in body 'b')",
    ),
    (
        "ax user overflow",
        r#"<mujoco><size nuser_jnt="1"/><worldbody><body name="b" pos="0 0 1"><joint name="z" type="hinge" axis="0 0 0" user="1 2"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody></mujoco>"#,
        "user has more values than nuser_jnt in joint",
        "invalid user on <joint>: user data exceeds nuser_jnt=1",
    ),
    (
        "ballslide then nested free",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><joint type="ball"/><joint type="slide" axis="1 0 0"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body><body name="p" pos="1 0 1"><joint name="k" type="hinge" axis="0 1 0" /><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/><body name="c"><freejoint/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></body></worldbody></mujoco>"#,
        "free joint can only be used on top level",
        "free joint can only be used on top level (body 'c')",
    ),
    (
        "pos geom user overflow",
        r#"<mujoco><size nuser_tendon="1"/><worldbody><site name="s0" pos="0.1 0 1.2"/><site name="s9" pos="-0.1 0 1.2"/><body name="b1" pos="0 0 1"><joint name="j1" type="hinge" axis="0 1 0"/><inertial pos="0 0 -0.15" mass="0.5" diaginertia="0.004 0.004 0.0002"/><site name="s1" pos="0.05 0 -0.1"/><body name="b2" pos="0 0 -0.3"><joint name="j2" type="hinge" axis="0 1 0"/><inertial pos="0 0 -0.15" mass="0.5" diaginertia="0.004 0.004 0.0002"/><site name="s2" pos="0.05 0 -0.2"/></body></body><body name="wb" pos="0.1 0 0.9"><geom name="w" type="sphere" size="0.04"/></body></worldbody><tendon><spatial armature="0.2" user="1 2"><site site="s0"/><geom geom="w"/><site site="s2"/></spatial></tendon></mujoco>"#,
        "user has more values than nuser_tendon in tendon",
        "invalid user on <tendon>: user data exceeds nuser_tendon=1",
    ),
    (
        "fixed neg then spatial neg",
        r#"<mujoco><worldbody><site name="s0" pos="0.1 0 1.2"/><site name="s9" pos="-0.1 0 1.2"/><body name="b1" pos="0 0 1"><joint name="j1" type="hinge" axis="0 1 0"/><inertial pos="0 0 -0.15" mass="0.5" diaginertia="0.004 0.004 0.0002"/><site name="s1" pos="0.05 0 -0.1"/><body name="b2" pos="0 0 -0.3"><joint name="j2" type="hinge" axis="0 1 0"/><inertial pos="0 0 -0.15" mass="0.5" diaginertia="0.004 0.004 0.0002"/><site name="s2" pos="0.05 0 -0.2"/></body></body><body name="wb" pos="0.1 0 0.9"><geom name="w" type="sphere" size="0.04"/></body></worldbody><tendon><fixed armature="-0.1"><joint joint="j1" coef="1"/></fixed><spatial armature="-0.1"><site site="s0"/><site site="s2"/></spatial></tendon></mujoco>"#,
        "tendon '' (id = 1): tendon armature cannot be negative",
        "invalid armature on <tendon>: tendon armature cannot be negative (tendon #1)",
    ),
    (
        "neg after bad flex",
        r#"<mujoco><worldbody><site name="s0" pos="0.1 0 1.2"/><body name="b1" pos="0 0 1"><joint name="j1" type="slide" axis="1 0 0"/><inertial pos="0 0 -0.15" mass="0.5" diaginertia="0.004 0.004 0.0002"/><site name="s1" pos="0.05 0 -0.1"/></body><body name="b2" pos="0.3 0 1"><joint name="j2" type="slide" axis="1 0 0"/><inertial pos="0 0 -0.15" mass="0.5" diaginertia="0.004 0.004 0.0002"/></body></worldbody><deformable><flex name="f" dim="1" body="b1 b2" vertex="0 0 0 0 0 0" element="0 5"/></deformable><tendon><spatial armature="-0.1"><site site="s0"/><site site="s1"/></spatial></tendon></mujoco>"#,
        "elem vertex id out of range",
        "invalid element on <flex>: elem vertex id out of range (flex 'f')",
    ),
    (
        "neg after bad equality",
        r#"<mujoco><worldbody><site name="s0" pos="0.1 0 1.2"/><site name="s9" pos="-0.1 0 1.2"/><body name="b1" pos="0 0 1"><joint name="j1" type="hinge" axis="0 1 0"/><inertial pos="0 0 -0.15" mass="0.5" diaginertia="0.004 0.004 0.0002"/><site name="s1" pos="0.05 0 -0.1"/><body name="b2" pos="0 0 -0.3"><joint name="j2" type="hinge" axis="0 1 0"/><inertial pos="0 0 -0.15" mass="0.5" diaginertia="0.004 0.004 0.0002"/><site name="s2" pos="0.05 0 -0.2"/></body></body><body name="wb" pos="0.1 0 0.9"><geom name="w" type="sphere" size="0.04"/></body></worldbody><tendon><spatial armature="-0.1"><site site="s0"/><site site="s2"/></spatial></tendon><equality><connect body1="b1" body2="b1" anchor="0 0 0"/></equality></mujoco>"#,
        "element 'b1' is repeated in equality constraint 0",
        "invalid body2 on <connect>: element 'b1' is repeated in equality constraint 0 (equality #0)",
    ),
    (
        "pos geom after bad equality",
        r#"<mujoco><worldbody><site name="s0" pos="0.1 0 1.2"/><site name="s9" pos="-0.1 0 1.2"/><body name="b1" pos="0 0 1"><joint name="j1" type="hinge" axis="0 1 0"/><inertial pos="0 0 -0.15" mass="0.5" diaginertia="0.004 0.004 0.0002"/><site name="s1" pos="0.05 0 -0.1"/><body name="b2" pos="0 0 -0.3"><joint name="j2" type="hinge" axis="0 1 0"/><inertial pos="0 0 -0.15" mass="0.5" diaginertia="0.004 0.004 0.0002"/><site name="s2" pos="0.05 0 -0.2"/></body></body><body name="wb" pos="0.1 0 0.9"><geom name="w" type="sphere" size="0.04"/></body></worldbody><tendon><spatial armature="0.2"><site site="s0"/><geom geom="w"/><site site="s2"/></spatial></tendon><equality><connect body1="b1" body2="b1" anchor="0 0 0"/></equality></mujoco>"#,
        "element 'b1' is repeated in equality constraint 0",
        "invalid body2 on <connect>: element 'b1' is repeated in equality constraint 0 (equality #0)",
    ),
    (
        "neg user overflow",
        r#"<mujoco><size nuser_tendon="1"/><worldbody><site name="s0" pos="0.1 0 1.2"/><site name="s9" pos="-0.1 0 1.2"/><body name="b1" pos="0 0 1"><joint name="j1" type="hinge" axis="0 1 0"/><inertial pos="0 0 -0.15" mass="0.5" diaginertia="0.004 0.004 0.0002"/><site name="s1" pos="0.05 0 -0.1"/><body name="b2" pos="0 0 -0.3"><joint name="j2" type="hinge" axis="0 1 0"/><inertial pos="0 0 -0.15" mass="0.5" diaginertia="0.004 0.004 0.0002"/><site name="s2" pos="0.05 0 -0.2"/></body></body><body name="wb" pos="0.1 0 0.9"><geom name="w" type="sphere" size="0.04"/></body></worldbody><tendon><spatial armature="-0.1" user="1 2"><site site="s0"/><site site="s2"/></spatial></tendon></mujoco>"#,
        "user has more values than nuser_tendon in tendon",
        "invalid user on <tendon>: user data exceeds nuser_tendon=1",
    ),
    (
        "axis then spatial negative",
        r#"<mujoco><worldbody><site name="s0" pos="0 0 2"/><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 0 0"/><geom type="sphere" size="0.1"/><site name="s1"/></body></worldbody><tendon><spatial name="t" armature="-0.1"><site site="s0"/><site site="s1"/></spatial></tendon></mujoco>"#,
        "axis too small in joint",
        "invalid axis on <joint>: axis too small (joint 'j' in body 'b')",
    ),
    (
        "nested free then spatial negative",
        r#"<mujoco><worldbody><site name="s0" pos="0 0 2"/><body name="a" pos="0 0 1"><joint type="hinge"/><geom type="sphere" size="0.1"/><body name="b" pos="0 0 -0.3"><freejoint/><geom type="sphere" size="0.1"/><site name="s1"/></body></body></worldbody><tendon><spatial name="t" armature="-0.1"><site site="s0"/><site site="s1"/></spatial></tendon></mujoco>"#,
        "tendon 't' (id = 0): tendon armature cannot be negative",
        "invalid armature on <tendon>: tendon armature cannot be negative (tendon 't')",
    ),
    (
        "nested free then fixed negative",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><joint name="h" type="hinge"/><geom type="sphere" size="0.1"/><body name="b" pos="0 0 -0.3"><freejoint/><geom type="sphere" size="0.1"/></body></body></worldbody><tendon><fixed name="t" armature="-0.1"><joint joint="h" coef="1"/></fixed></tendon></mujoco>"#,
        "free joint can only be used on top level",
        "free joint can only be used on top level (body 'b')",
    ),
    (
        "ball slide then spatial negative",
        r#"<mujoco><worldbody><site name="s0" pos="0 0 2"/><body name="b" pos="0 0 1"><joint type="ball"/><joint type="slide"/><geom type="sphere" size="0.1"/><site name="s1"/></body></worldbody><tendon><spatial name="t" armature="-0.1"><site site="s0"/><site site="s1"/></spatial></tendon></mujoco>"#,
        "tendon 't' (id = 0): tendon armature cannot be negative",
        "invalid armature on <tendon>: tendon armature cannot be negative (tendon 't')",
    ),
    (
        "fixed negative then bad keyframe qpos",
        r#"<mujoco><worldbody><site name="s0" pos="0.1 0 1.2"/><body name="b1" pos="0 0 1"><joint name="j1" type="hinge" axis="0 1 0"/><inertial pos="0 0 -0.15" mass="0.5" diaginertia="0.004 0.004 0.0002"/><site name="s1" pos="0.05 0 -0.1"/><body name="b2" pos="0 0 -0.3"><joint name="j2" type="hinge" axis="0 1 0"/><inertial pos="0 0 -0.15" mass="0.5" diaginertia="0.004 0.004 0.0002"/><site name="s2" pos="0.05 0 -0.2"/></body></body><body name="wb" pos="0.1 0 0.9"><geom name="w" type="sphere" size="0.04"/></body></worldbody><tendon><fixed armature="-0.1"><joint joint="j1" coef="1"/></fixed></tendon><keyframe><key qpos="0 0 0"/></keyframe></mujoco>"#,
        "keyframe '': invalid qpos size, expected 2, got 3",
        "invalid qpos on <key>: length 3 does not match 2 (key #0)",
    ),
    (
        "ball slide then bad keyframe ctrl",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><joint type="ball"/><joint type="slide" axis="1 0 0"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><keyframe><key ctrl="1"/></keyframe></mujoco>"#,
        "keyframe 0: invalid ctrl size, expected length 0",
        "invalid ctrl on <key>: length 1 does not match 0 (key #0)",
    ),
    (
        "zero axis then bad keyframe qpos",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><joint name="z" type="hinge" axis="0 0 0"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><keyframe><key qpos="0 0"/></keyframe></mujoco>"#,
        "axis too small in joint",
        "invalid axis on <joint>: axis too small (joint 'z' in body 'a')",
    ),
    (
        "zero axis then backwards range",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><joint name="z" type="hinge" axis="0 0 0" limited="true" range="1 -1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody></mujoco>"#,
        "range[0] should be smaller than range[1] in joint",
        "invalid range on <joint>: range[0] should be smaller than range[1] in joint (joint 'z' in body 'a')",
    ),
    (
        "backwards range then nested free",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><joint name="h" type="hinge" axis="0 1 0" limited="true" range="1 -1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/><body name="b" pos="0 0 -0.3"><freejoint/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></body></worldbody></mujoco>"#,
        "range[0] should be smaller than range[1] in joint",
        "invalid range on <joint>: range[0] should be smaller than range[1] in joint (joint 'h' in body 'a')",
    ),
    (
        "backwards joint range then spatial negative armature",
        r#"<mujoco><worldbody><site name="s0" pos="0 0 1.2"/><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" limited="true" range="1 -1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/><site name="s1" pos="0.1 0 0"/></body></worldbody><tendon><spatial armature="-0.1"><site site="s0"/><site site="s1"/></spatial></tendon></mujoco>"#,
        "range[0] should be smaller than range[1] in joint",
        "invalid range on <joint>: range[0] should be smaller than range[1] in joint (joint 'j' in body 'b')",
    ),
    (
        "backwards range then ball slide",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><joint name="b" type="ball"/><joint name="s" type="slide" axis="1 0 0"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body><body name="c" pos="1 0 1"><joint name="k" type="hinge" axis="0 1 0" limited="true" range="1 -1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody></mujoco>"#,
        "range[0] should be smaller than range[1] in joint",
        "invalid range on <joint>: range[0] should be smaller than range[1] in joint (joint 'k' in body 'c')",
    ),
    (
        "range without limited then backwards ctrlrange",
        r#"<mujoco><compiler angle="radian" autolimits="false"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" range="-1 1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><actuator><motor joint="j" ctrllimited="true" ctrlrange="1 -1"/></actuator></mujoco>"#,
        "joint has `range` but not `limited`. set the autolimits=\"true\" compiler option, specify `limited` explicitly (\"true\" or \"false\"), or remove the `range` attribute.",
        "invalid range on <joint>: joint has `range` but not `limited`. set the autolimits=\"true\" compiler option, specify `limited` explicitly (\"true\" or \"false\"), or remove the `range` attribute. (joint 'j' in body 'b')",
    ),
    (
        "backwards range then seven dofs",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><joint name="h0" type="hinge" axis="0 1 0" limited="true" range="1 -1"/><joint name="h1" type="hinge" axis="0 1 0"/><joint name="h2" type="hinge" axis="0 1 0"/><joint name="h3" type="hinge" axis="0 1 0"/><joint name="h4" type="hinge" axis="0 1 0"/><joint name="h5" type="hinge" axis="0 1 0"/><joint name="h6" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody></mujoco>"#,
        "range[0] should be smaller than range[1] in joint",
        "invalid range on <joint>: range[0] should be smaller than range[1] in joint (joint 'h0' in body 'a')",
    ),
    (
        "zero axis then next joint backwards",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="z" type="hinge" axis="0 0 0"/><joint name="j" type="hinge" axis="0 1 0" limited="true" range="1 -1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody></mujoco>"#,
        "axis too small in joint",
        "invalid axis on <joint>: axis too small (joint 'z' in body 'b')",
    ),
    (
        "seven dofs then later backwards range",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><joint name="h0" type="hinge" axis="0 1 0"/><joint name="h1" type="hinge" axis="0 1 0"/><joint name="h2" type="hinge" axis="0 1 0"/><joint name="h3" type="hinge" axis="0 1 0"/><joint name="h4" type="hinge" axis="0 1 0"/><joint name="h5" type="hinge" axis="0 1 0"/><joint name="h6" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body><body name="c" pos="1 0 1"><joint name="k" type="hinge" axis="0 1 0" limited="true" range="1 -1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody></mujoco>"#,
        "more than 6 dofs in body 'a'",
        "more than 6 dofs (body 'a')",
    ),
    (
        "backwards joint range then backwards tendon range",
        r#"<mujoco><compiler angle="radian"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" range="-1 1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body><body name="c" pos="1 0 1"><joint name="k" type="hinge" axis="0 1 0" limited="true" range="1 -1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><tendon><fixed name="t" limited="true" range="1 -1"><joint joint="j" coef="1"/></fixed></tendon></mujoco>"#,
        "range[0] should be smaller than range[1] in joint",
        "invalid range on <joint>: range[0] should be smaller than range[1] in joint (joint 'k' in body 'c')",
    ),
    (
        "spatial negative armature then backwards tendon range",
        r#"<mujoco><worldbody><site name="s0" pos="0 0 1.2"/><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/><site name="s1" pos="0.1 0 0"/></body></worldbody><tendon><spatial armature="-0.1" limited="true" range="1 -1"><site site="s0"/><site site="s1"/></spatial></tendon></mujoco>"#,
        "tendon '' (id = 0): tendon armature cannot be negative",
        "invalid armature on <tendon>: tendon armature cannot be negative (tendon #0)",
    ),
    (
        "spatial negative armature then backwards range, named",
        r#"<mujoco><worldbody><site name="s0" pos="0 0 1.2"/><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/><site name="s1" pos="0.1 0 0"/></body></worldbody><tendon><spatial name="t" armature="-0.1" limited="true" range="1 -1"><site site="s0"/><site site="s1"/></spatial></tendon></mujoco>"#,
        "tendon 't' (id = 0): tendon armature cannot be negative",
        "invalid armature on <tendon>: tendon armature cannot be negative (tendon 't')",
    ),
    (
        "backwards tendon range then backwards springlength",
        r#"<mujoco><compiler angle="radian"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" range="-1 1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><tendon><fixed name="t" limited="true" range="1 -1" springlength="0.7 0.3"><joint joint="j" coef="1"/></fixed></tendon></mujoco>"#,
        "invalid limits in tendon",
        "invalid range on <tendon>: invalid limits in tendon (tendon 't')",
    ),
    (
        "backwards tendon range then backwards ctrlrange",
        r#"<mujoco><compiler angle="radian"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" range="-1 1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><tendon><fixed name="t" limited="true" range="1 -1"><joint joint="j" coef="1"/></fixed></tendon><actuator><motor joint="j" ctrllimited="true" ctrlrange="1 -1"/></actuator></mujoco>"#,
        "invalid limits in tendon",
        "invalid range on <tendon>: invalid limits in tendon (tendon 't')",
    ),
    (
        "fixed negative armature then backwards tendon range",
        r#"<mujoco><compiler angle="radian"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" range="-1 1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><tendon><fixed name="t" armature="-0.1" limited="true" range="1 -1"><joint joint="j" coef="1"/></fixed></tendon></mujoco>"#,
        "invalid limits in tendon",
        "invalid range on <tendon>: invalid limits in tendon (tendon 't')",
    ),
    (
        "backwards ctrlrange then fixed negative armature",
        r#"<mujoco><worldbody><site name="s0" pos="0.1 0 1.2"/><body name="b1" pos="0 0 1"><joint name="j1" type="hinge" axis="0 1 0"/><inertial pos="0 0 -0.15" mass="0.5" diaginertia="0.004 0.004 0.0002"/><site name="s1" pos="0.05 0 -0.1"/><body name="b2" pos="0 0 -0.3"><joint name="j2" type="hinge" axis="0 1 0"/><inertial pos="0 0 -0.15" mass="0.5" diaginertia="0.004 0.004 0.0002"/><site name="s2" pos="0.05 0 -0.2"/></body></body><body name="wb" pos="0.1 0 0.9"><geom name="w" type="sphere" size="0.04"/></body></worldbody><tendon><fixed armature="-0.1"><joint joint="j1" coef="1"/></fixed></tendon><actuator><motor joint="j1" ctrllimited="true" ctrlrange="1 -1"/></actuator></mujoco>"#,
        "invalid control range for actuator",
        "invalid ctrlrange on <actuator>: invalid control range for actuator (actuator #0)",
    ),
    (
        "backwards ctrlrange then backwards actrange",
        r#"<mujoco><compiler angle="radian"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" range="-1 1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><actuator><general joint="j" dyntype="integrator" ctrllimited="true" ctrlrange="1 -1" actlimited="true" actrange="1 -1"/></actuator></mujoco>"#,
        "invalid control range for actuator",
        "invalid ctrlrange on <actuator>: invalid control range for actuator (actuator #0)",
    ),
    (
        "backwards ctrlrange then bad keyframe qpos",
        r#"<mujoco><compiler angle="radian"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" range="-1 1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><actuator><motor joint="j" ctrllimited="true" ctrlrange="1 -1"/></actuator><keyframe><key qpos="1 2"/></keyframe></mujoco>"#,
        "invalid control range for actuator",
        "invalid ctrlrange on <actuator>: invalid control range for actuator (actuator #0)",
    ),
    (
        "backwards forcerange then backwards ctrlrange",
        r#"<mujoco><compiler angle="radian"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" range="-1 1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><actuator><motor joint="j" ctrllimited="true" ctrlrange="1 -1" forcelimited="true" forcerange="1 -1"/></actuator></mujoco>"#,
        "invalid force range for actuator",
        "invalid forcerange on <actuator>: invalid force range for actuator (actuator #0)",
    ),
    (
        "backwards ctrlrange then limit sensor on an unlimited joint",
        r#"<mujoco><compiler angle="radian"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><actuator><motor joint="j" ctrllimited="true" ctrlrange="1 -1"/></actuator><sensor><jointlimitfrc joint="j"/></sensor></mujoco>"#,
        "invalid control range for actuator",
        "invalid ctrlrange on <actuator>: invalid control range for actuator (actuator #0)",
    ),
    (
        "backwards forcerange then ctrlrange without ctrllimited",
        r#"<mujoco><compiler angle="radian" autolimits="false"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" limited="true" range="-1 1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><actuator><motor joint="j" forcelimited="true" forcerange="1 -1" ctrlrange="-1 1"/></actuator></mujoco>"#,
        "actuator has `ctrlrange` but not `ctrllimited`. set the autolimits=\"true\" compiler option, specify `ctrllimited` explicitly (\"true\" or \"false\"), or remove the `ctrlrange` attribute.",
        "invalid ctrlrange on <actuator>: actuator has `ctrlrange` but not `ctrllimited`. set the autolimits=\"true\" compiler option, specify `ctrllimited` explicitly (\"true\" or \"false\"), or remove the `ctrlrange` attribute. (actuator #0)",
    ),
    (
        "limit sensor on an unlimited joint then nested free",
        r#"<mujoco><compiler angle="radian"/><worldbody><body name="a" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/><body name="c" pos="0 0 -0.3"><freejoint/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></body></worldbody><sensor><jointlimitfrc joint="j"/></sensor></mujoco>"#,
        "joint must be limited in sensor",
        "joint must be limited in sensor (sensor #0)",
    ),
];

#[test]
fn a_document_with_two_faults_is_refused_for_mujocos() {
    let mut missed = Vec::new();
    for &(what, xml, mujoco, expected) in TWO_FAULTS {
        match load_model(xml) {
            Err(e) if e.to_string() == expected => {}
            Err(e) => missed.push(format!("{what}: {e} (MuJoCo: {mujoco})")),
            Ok(_) => missed.push(format!("{what}: loads (MuJoCo: {mujoco})")),
        }
    }
    assert!(missed.is_empty(), "{}", missed.join("\n"));
}
