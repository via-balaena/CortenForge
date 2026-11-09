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
//! once the model is built, and so does RL-6b's third, an automatic range
//! that does not increase (`D-AUTO-RANGE`), which MuJoCo leaves unlimited, as
//! every check before it takes it. RL-6b's limit checks MuJoCo makes run with
//! RL-6a's, in MuJoCo's order; a limited tendon's range and an element's
//! backwards spring length stay where this crate checked them, and a class's
//! spring length is checked once the class is applied, after the actuators'
//! references (ledger L353 holds the orders this leaves unlike MuJoCo's). Where
//! MuJoCo orders a new refusal before a check this crate makes later, this
//! crate names the later one, as it did before. RL-6d's checks of a body (a
//! `fullinertia` beside a diagonal inertia; its geoms' plane only in a static
//! body and negative mass or density; its moments' triangle) run with
//! RL-6a's, body by body, before the body's joints; a height field's sizes
//! after every body's and before the tendons'; a moving body's mass and
//! inertia after the sensors and before the free joint below the world's
//! children, as MuJoCo orders them. An `<inertial>`'s negative mass or
//! diagonal moment, which MuJoCo raises to its bounds, is refused after the
//! keyframes (`D-INERTIAL-NEGATIVE`), every check before it taking MuJoCo's
//! raised value. RL-7a's check of repeated names (book L23) runs where this
//! crate checked a body's, a joint's and an actuator's name before, now
//! ahead of the options too, as MuJoCo's reader meets a name before any
//! check its compiler makes; a frame's name after the world's tree. Where
//! MuJoCo's reader meets the repeated name first and this crate meets the
//! other fault first (a parse fault, an undefined class, a composite's, a
//! frame repeated before a later name), this crate names the other: ledger
//! L353's. RL-7b's faults inside frames are named as MuJoCo compiles bodies,
//! one at a time in preorder: a body's frames' own orientations, then its
//! own where a frame holds it, then its frames' geoms', then their sites',
//! each kind's first in document order, then a cable's segment too short in
//! it, before the next body's; all before any fault the builder finds
//! outside frames. A composite's other faults are met when it is made,
//! before frames, as MuJoCo's reader meets them. `FRAME_AND_COMPOSITE_COSTS`
//! pins the orders found unlike MuJoCo's (ledger L353). RL-7c's check of a
//! geom's condim (book L25) runs with RL-6d's, geom by geom from the
//! world's, before its plane check; a geom `fusestatic` moves into an
//! earlier body has its condim, plane and negative-density checks at its own
//! body's turn, as MuJoCo compiles every body before it fuses (the density
//! is judged by its new body's inertia rule: ledger L388); a pair's condim,
//! then its geoms, after the assets and before the sizes, where MuJoCo
//! compiles pairs; `CONDIM_COSTS` pins the faults this crate meets first
//! that MuJoCo meets after a condim. A `<size>` `nuser_` below -1 is refused
//! once the document is parsed, or when a fault outside `<compiler>` and
//! `<option>` stops the parser first, as MuJoCo's reader reads `<compiler>`,
//! `<option>` and then `<size>` before the other sections; `NUSER_COSTS`
//! pins a fault the parser meets before a later `<size>`. A model built in
//! code has its `nuser_` sizes checked before its names. A
//! flex's condim outside 1 to 6, which MuJoCo loads, and a pair naming a
//! geom "" where the model has an unnamed geom, which MuJoCo binds to it,
//! are refused after the keyframes (`D-FLEX-CONDIM-RANGE`,
//! `D-PAIR-NO-GEOM`); where it has none, MuJoCo refuses that pair where it
//! compiles pairs, and so does this crate.
//! RL-7d's checks of a sensor (ledger L197, L199, L391) run with RL-6b's
//! limit check, sensor by sensor, each sensor's in MuJoCo's order: a
//! negative noise, a negative cutoff, the limit, a height field in a geom
//! distance, then a cutoff on a quaternion or an axis. MuJoCo checks a
//! sensor's noise and cutoff before its object, interval, delay and a user
//! sensor's dim, and a sensor's whole compile before a later sensor's; this
//! crate checks those as it reads each sensor, before RL-7d's, and a timestep
//! not above zero (`D-OPTION-TIMESTEP`) before all of them, and names them:
//! `SENSOR_COSTS` (ledger L353).
//! RL-8a's checks of a flex (book L31, L32): a bending flex's thickness runs
//! with RL-6a's, after the bodies' and before the height fields' sizes, as
//! MuJoCo compiles the flexes before the height fields; a flex equality's
//! flex (unknown, or rigid) after the height fields' and before the
//! tendons', in index order, as MuJoCo compiles the equalities. A
//! flexcomp's stretch beside the equality its `<edge>` asks for is refused
//! while parsing, as MuJoCo's reader refuses it. A flex vertex equality
//! (`D-FLEX-VERT-EQUALITY`) and an empty `flex` beside an unnamed flex
//! (`D-FLEX-EQ-NO-NAME`), which MuJoCo makes, and a flex's membrane, a dim-3
//! flex's Young's modulus and an edge of three triangles in a flex that
//! bends (`D-FLEX-MEMBRANE`, `D-FLEX-TET-ELASTICITY`,
//! `D-FLEX-BEND-NONMANIFOLD`), which MuJoCo computes, are refused after the
//! keyframes. Where MuJoCo checks a flex before a fault this crate meets
//! while parsing or before RL-6a's checks, this crate names that fault:
//! `FLEX_COSTS` (ledger L353).
//! RL-8b's body-level `<flexcomp>` (made as MuJoCo's `mjCFlexcomp::Make`
//! makes it): its reader's checks (its name, its keywords, two
//! orientations, an edge equality beside a stretching elasticity) run while
//! parsing, and Make's when the composites are made, in document order among
//! them, before every frame's and every later check, as MuJoCo makes a
//! flexcomp while its reader meets it; two of Make's checks in one flexcomp
//! are named in Make's order. A type this crate does not make (mesh, gmsh)
//! and a box's count of 1 (where MuJoCo's maker never ends) are refused where
//! Make reaches the maker; a form MuJoCo loads that this crate makes and
//! sim-core does not take (a radial or interpolated dof, a vertex body in a
//! moving or turned holding body) and a `<plugin>` are refused after the
//! keyframes. Where MuJoCo's reader meets a fault written before the
//! flexcomp first (a repeated name, an unknown class, a composite's), this
//! crate names the flexcomp's, as it names a composite's; and a parse fault
//! or an unknown class written after the flexcomp before Make's checks, as
//! before: `FLEXCOMP_COSTS` (ledger L353).
//! RL-8c's refusals of what MuJoCo 3.5.0's schema refuses (a `<flexcomp>` in
//! `<deformable>`; on a `<flex>` the first written of the attributes this
//! crate once read for its own flexcomp) are made where the reader meets
//! them, so before every check made after parsing, as MuJoCo's schema walk
//! makes them before any value is read; a fault the parser meets written
//! before them is named first here: `SCHEMA_COSTS` (ledger L353).
//!
//! Every document below was refused by MuJoCo 3.5.0, on the census's unfused
//! oracle, with the reason its row gives as MuJoCo's (RL-6a's measured
//! 2026-10-31, RL-6b's 2026-11-01, RL-6d's 2026-11-02). Eleven are the RL-6a
//! review's first round, three its fix-diff pass's (a keyframe's fault), and
//! four set the new checks against each other; RL-6b's set a limit's fault
//! beside another, and RL-6d's a body's or a height field's.
//! RL-7a's (measured 2026-11-03) set a repeated name beside a size, a
//! reference and an axis MuJoCo refuses too, and beside an option this crate
//! refuses and MuJoCo does not. RL-7b's (measured 2026-11-03; review round
//! 1's documents, RA and RW) set a fault in a frame or a cable beside
//! another. RL-7c's (measured 2026-11-04) set a geom's or a pair's condim,
//! a pair's unknown geom, a `nuser_` below -1 or a flex's condim beside
//! another; its review's first round's, a geom `fusestatic` moves beside an
//! earlier body's fault, a `nuser_` below -1 beside a repeated name, a
//! class's fault or a parse fault, and a pair naming a geom "" beside
//! another fault. RL-7d's (measured 2026-11-05) set a sensor's negative
//! noise or cutoff, a cutoff on a quaternion or a height field in a geom
//! distance beside another fault: in `TWO_FAULTS`, sixteen of the 87
//! two-fault documents generated at its start; in `SENSOR_COSTS`, the twelve
//! of the 87 this crate names otherwise and its review's first round's
//! nineteen. RL-8a's (measured 2026-11-07; most of them its review's first
//! round's, RA's) set a flex's thickness, a flex equality, a flex vertex
//! equality, a flexcomp's stretch beside its edge equality, or a refusal made
//! after the keyframes beside another fault: fifteen in `TWO_FAULTS`, three
//! in `FLEX_COSTS`. RL-8b's
//! (measured 2026-11-07, the 503 two-fault documents generated at its start,
//! each fault alone refused by MuJoCo but the forms it loads) set a
//! flexcomp's check or a refused form beside another fault: four in
//! `TWO_FAULTS`, six in `FLEXCOMP_COSTS`; its review's first round's set two
//! of Make's checks in one flexcomp (eight in `TWO_FAULTS`) and two
//! flexcomps of one name (one in `FLEXCOMP_COSTS`).
//! RL-8c's (measured 2026-11-08, its two-fault set of 149 documents: each
//! of its six refusals beside a fault of every stage, written before and
//! after it) set a `<flexcomp>` in `<deformable>` or a `<flex>`'s refused
//! attribute beside another fault: four in `TWO_FAULTS`, six in
//! `SCHEMA_COSTS`.

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
    (
        "backwards tendon range then a sensor's missing joint",
        r#"<mujoco><compiler angle="radian"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" /><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/><site name="s1" pos="0.1 0 0"/></body></worldbody><tendon><fixed name="t" limited="true" range="1 -1"><joint joint="j" coef="1"/></fixed></tendon><sensor><jointpos joint="nope"/></sensor></mujoco>"#,
        "invalid limits in tendon",
        "invalid range on <tendon>: invalid limits in tendon (tendon 't')",
    ),
    (
        "backwards springlength then a sensor's missing joint",
        r#"<mujoco><compiler angle="radian"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" /><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/><site name="s1" pos="0.1 0 0"/></body></worldbody><tendon><fixed name="t" springlength="0.7 0.3"><joint joint="j" coef="1"/></fixed></tendon><sensor><jointpos joint="nope"/></sensor></mujoco>"#,
        "invalid springlength in tendon",
        "invalid springlength on <tendon>: invalid springlength in tendon (line 1: tendon > fixed 't')",
    ),
    (
        "backwards tendon range then inheritrange on a ball",
        r#"<mujoco><compiler angle="radian"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" /><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/><site name="s1" pos="0.1 0 0"/></body><body name="c" pos="1 0 1"><joint name="k" type="ball"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><tendon><fixed name="t" limited="true" range="1 -1"><joint joint="j" coef="1"/></fixed></tendon><actuator><position joint="k" kp="1" inheritrange="1"/></actuator></mujoco>"#,
        "invalid limits in tendon",
        "invalid range on <tendon>: invalid limits in tendon (tendon 't')",
    ),
    (
        "backwards springlength then an actuator's missing joint",
        r#"<mujoco><compiler angle="radian"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" /><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/><site name="s1" pos="0.1 0 0"/></body></worldbody><tendon><fixed name="t" springlength="0.7 0.3"><joint joint="j" coef="1"/></fixed></tendon><actuator><motor joint="nope"/></actuator></mujoco>"#,
        "invalid springlength in tendon",
        "invalid springlength on <tendon>: invalid springlength in tendon (line 1: tendon > fixed 't')",
    ),
    (
        "limited tendon with no range then a sensor's missing joint",
        r#"<mujoco><compiler angle="radian"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" /><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/><site name="s1" pos="0.1 0 0"/></body></worldbody><tendon><fixed name="t" limited="true"><joint joint="j" coef="1"/></fixed></tendon><sensor><jointpos joint="nope"/></sensor></mujoco>"#,
        "invalid limits in tendon",
        "invalid range on <tendon>: invalid limits in tendon (tendon 't')",
    ),
    (
        "class's backwards springlength then a sensor's missing joint",
        r#"<mujoco><compiler angle="radian"/><default><tendon springlength="0.7 0.3"/></default><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" /><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/><site name="s1" pos="0.1 0 0"/></body></worldbody><tendon><fixed name="t"><joint joint="j" coef="1"/></fixed></tendon><sensor><jointpos joint="nope"/></sensor></mujoco>"#,
        "invalid springlength in tendon",
        "invalid springlength on <tendon>: invalid springlength in tendon (tendon 't')",
    ),
    (
        "automatic backwards range then zero axis",
        r#"<mujoco><compiler angle="radian"/><worldbody><body name="a" pos="0 0 1"><joint name="z" type="hinge" axis="0 0 0" range="1 -1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody></mujoco>"#,
        "axis too small in joint",
        "invalid axis on <joint>: axis too small (joint 'z' in body 'a')",
    ),
    (
        "automatic backwards range then backwards ctrlrange",
        r#"<mujoco><compiler angle="radian"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" range="1 -1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/><site name="s1" pos="0.1 0 0"/></body></worldbody><actuator><motor joint="j" ctrllimited="true" ctrlrange="1 -1"/></actuator></mujoco>"#,
        "invalid control range for actuator",
        "invalid ctrlrange on <actuator>: invalid control range for actuator (actuator #0)",
    ),
    (
        "automatic backwards range then bad keyframe qpos",
        r#"<mujoco><compiler angle="radian"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" range="1 -1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/><site name="s1" pos="0.1 0 0"/></body></worldbody><keyframe><key qpos="1 2"/></keyframe></mujoco>"#,
        "keyframe '': invalid qpos size, expected 1, got 2",
        "invalid qpos on <key>: length 2 does not match 1 (key #0)",
    ),
    (
        "automatic backwards forcerange then backwards ctrlrange, one actuator",
        r#"<mujoco><compiler angle="radian"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" /><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/><site name="s1" pos="0.1 0 0"/></body></worldbody><actuator><motor joint="j" forcerange="1 -1" ctrllimited="true" ctrlrange="1 -1"/></actuator></mujoco>"#,
        "invalid control range for actuator",
        "invalid ctrlrange on <actuator>: invalid control range for actuator (actuator #0)",
    ),
    (
        "automatic backwards tendon range then its limit sensor",
        r#"<mujoco><compiler angle="radian"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/><site name="s1" pos="0.1 0 0"/></body></worldbody><tendon><fixed name="t" range="1 -1"><joint joint="j" coef="1"/></fixed></tendon><sensor><tendonlimitfrc tendon="t"/></sensor></mujoco>"#,
        "tendon must be limited in sensor",
        "tendon must be limited in sensor (sensor #0)",
    ),
    (
        "automatic backwards range then seven dofs",
        r#"<mujoco><compiler angle="radian"/><worldbody><body name="a" pos="0 0 1"><joint name="h0" type="hinge" axis="0 1 0" range="1 -1"/><joint name="h1" type="hinge" axis="0 1 0"/><joint name="h2" type="hinge" axis="0 1 0"/><joint name="h3" type="hinge" axis="0 1 0"/><joint name="h4" type="hinge" axis="0 1 0"/><joint name="h5" type="hinge" axis="0 1 0"/><joint name="h6" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody></mujoco>"#,
        "more than 6 dofs in body 'a'",
        "more than 6 dofs (body 'a')",
    ),
    (
        "automatic ball range then ball followed by hinge",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><joint name="b" type="ball" range="0 -10"/><joint name="h" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody></mujoco>"#,
        "ball followed by rotation in body 'a'",
        "ball followed by rotation (body 'a')",
    ),
    (
        "automatic backwards range and zero axis, one hinge",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><joint name="z" type="hinge" axis="0 0 0" range="1 -1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody></mujoco>
"#,
        "axis too small in joint",
        "invalid axis on <joint>: axis too small (joint 'z' in body 'a')",
    ),
    (
        "automatic backwards range then a limited backwards one",
        r#"<mujoco><compiler angle="radian"/><worldbody><body name="a" pos="0 0 1"><joint name="j1" type="hinge" axis="0 1 0" range="1 -1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body><body name="c" pos="1 0 1"><joint name="j2" type="hinge" axis="0 1 0" limited="true" range="1 -1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody></mujoco>
"#,
        "range[0] should be smaller than range[1] in joint",
        "invalid range on <joint>: range[0] should be smaller than range[1] in joint (joint 'j2' in body 'c')",
    ),
    (
        "automatic backwards forcerange beside a limited backwards ctrlrange",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><actuator><motor joint="j" forcerange="1 -1" ctrllimited="true" ctrlrange="1 -1"/></actuator></mujoco>
"#,
        "invalid control range for actuator",
        "invalid ctrlrange on <actuator>: invalid control range for actuator (actuator #0)",
    ),
    (
        "automatic backwards joint range then backwards tendon range",
        r#"<mujoco><compiler angle="radian"/><worldbody><body name="a" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0" range="1 -1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><tendon><fixed limited="true" range="1 -1"><joint joint="j" coef="1"/></fixed></tendon></mujoco>
"#,
        "invalid limits in tendon",
        "invalid range on <tendon>: invalid limits in tendon (tendon #0)",
    ),
    (
        "automatic backwards tendon range and backwards springlength",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><tendon><fixed range="1 0" springlength="2 1"><joint joint="j" coef="1"/></fixed></tendon></mujoco>
"#,
        "invalid springlength in tendon",
        "invalid springlength on <tendon>: invalid springlength in tendon (line 1: tendon > fixed)",
    ),
    (
        "automatic backwards range then nested free",
        r#"<mujoco><compiler angle="radian"/><worldbody><body name="a" pos="0 0 1"><joint name="h" type="hinge" axis="0 1 0" range="1 -1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/><body name="b" pos="0 0 -0.3"><freejoint/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></body></worldbody></mujoco>
"#,
        "free joint can only be used on top level",
        "free joint can only be used on top level (body 'b')",
    ),
    (
        "automatic backwards range then its limit sensor",
        r#"<mujoco><compiler angle="radian"/><worldbody><body name="a" pos="0 0 1"><joint name="h" type="hinge" axis="0 1 0" range="1 -1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><sensor><jointlimitfrc joint="h"/></sensor></mujoco>
"#,
        "joint must be limited in sensor",
        "joint must be limited in sensor (sensor #0)",
    ),
    (
        "automatic backwards range then limited backwards ctrlrange",
        r#"<mujoco><compiler angle="radian"/><worldbody><body name="a" pos="0 0 1"><joint name="h" type="hinge" axis="0 1 0" range="1 -1"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><actuator><motor joint="h" ctrllimited="true" ctrlrange="1 -1"/></actuator></mujoco>
"#,
        "invalid control range for actuator",
        "invalid ctrlrange on <actuator>: invalid control range for actuator (actuator #0)",
    ),
    (
        "N3plane S4axis n first",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><joint name="ja" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/><geom type="plane" size="1 1 0.1"/></body><body name="b" pos="1 0 1"><joint name="jb" type="hinge" axis="0 0 0"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody></mujoco>"#,
        "plane only allowed in static bodies",
        "plane only allowed in static bodies (geom #0 in body 'a')",
    ),
    (
        "N4mass S4axis n first",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><joint name="ja" type="hinge" axis="0 1 0"/><geom type="sphere" size="0.1" mass="-1"/></body><body name="b" pos="1 0 1"><joint name="jb" type="hinge" axis="0 0 0"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody></mujoco>"#,
        "mass, inertia or density are negative in geom",
        "mass, inertia or density are negative in geom (geom #0 in body 'a')",
    ),
    (
        "N3plane S4axis s first",
        r#"<mujoco><worldbody><body name="b" pos="1 0 1"><joint name="jb" type="hinge" axis="0 0 0"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body><body name="a" pos="0 0 1"><joint name="ja" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/><geom type="plane" size="1 1 0.1"/></body></worldbody></mujoco>"#,
        "axis too small in joint",
        "invalid axis on <joint>: axis too small (joint 'jb' in body 'b')",
    ),
    (
        "N4mass S4axis s first",
        r#"<mujoco><worldbody><body name="b" pos="1 0 1"><joint name="jb" type="hinge" axis="0 0 0"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body><body name="a" pos="0 0 1"><joint name="ja" type="hinge" axis="0 1 0"/><geom type="sphere" size="0.1" mass="-1"/></body></worldbody></mujoco>"#,
        "axis too small in joint",
        "invalid axis on <joint>: axis too small (joint 'jb' in body 'b')",
    ),
    (
        "N3plane S4axis same after",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><joint name="ja" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/><geom type="plane" size="1 1 0.1"/><joint name="jz" type="hinge" axis="0 0 0"/></body></worldbody></mujoco>"#,
        "plane only allowed in static bodies",
        "plane only allowed in static bodies (geom #0 in body 'a')",
    ),
    (
        "N4mass S4axis same after",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><joint name="ja" type="hinge" axis="0 1 0"/><geom type="sphere" size="0.1" mass="-1"/><joint name="jz" type="hinge" axis="0 0 0"/></body></worldbody></mujoco>"#,
        "mass, inertia or density are negative in geom",
        "mass, inertia or density are negative in geom (geom #0 in body 'a')",
    ),
    (
        "N3plane S4axis same before",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><joint name="jz" type="hinge" axis="0 0 0"/><joint name="ja" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/><geom type="plane" size="1 1 0.1"/></body></worldbody></mujoco>"#,
        "plane only allowed in static bodies",
        "plane only allowed in static bodies (geom #0 in body 'a')",
    ),
    (
        "N4mass S4axis same before",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><joint name="jz" type="hinge" axis="0 0 0"/><joint name="ja" type="hinge" axis="0 1 0"/><geom type="sphere" size="0.1" mass="-1"/></body></worldbody></mujoco>"#,
        "mass, inertia or density are negative in geom",
        "mass, inertia or density are negative in geom (geom #0 in body 'a')",
    ),
    (
        "N3plane S4freenest",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><joint name="ja" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/><geom type="plane" size="1 1 0.1"/></body><body name="o" pos="2 0 1"><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/><body name="oc"><freejoint/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></body></worldbody></mujoco>"#,
        "plane only allowed in static bodies",
        "plane only allowed in static bodies (geom #0 in body 'a')",
    ),
    (
        "N4mass S4freenest",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><joint name="ja" type="hinge" axis="0 1 0"/><geom type="sphere" size="0.1" mass="-1"/></body><body name="o" pos="2 0 1"><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/><body name="oc"><freejoint/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></body></worldbody></mujoco>"#,
        "mass, inertia or density are negative in geom",
        "mass, inertia or density are negative in geom (geom #0 in body 'a')",
    ),
    (
        "N3plane S4sensor",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><joint name="ja" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/><geom type="plane" size="1 1 0.1"/></body><body name="o" pos="2 0 1"><joint name="jo" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><sensor><jointlimitfrc joint="jo"/></sensor></mujoco>"#,
        "plane only allowed in static bodies",
        "plane only allowed in static bodies (geom #0 in body 'a')",
    ),
    (
        "N4mass S4sensor",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><joint name="ja" type="hinge" axis="0 1 0"/><geom type="sphere" size="0.1" mass="-1"/></body><body name="o" pos="2 0 1"><joint name="jo" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><sensor><jointlimitfrc joint="jo"/></sensor></mujoco>"#,
        "mass, inertia or density are negative in geom",
        "mass, inertia or density are negative in geom (geom #0 in body 'a')",
    ),
    (
        "N3plane S5key",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><joint name="ja" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/><geom type="plane" size="1 1 0.1"/></body></worldbody><keyframe><key qpos="1 2 3 4 5 6 7 8 9"/></keyframe></mujoco>"#,
        "plane only allowed in static bodies",
        "plane only allowed in static bodies (geom #0 in body 'a')",
    ),
    (
        "N4mass S5key",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><joint name="ja" type="hinge" axis="0 1 0"/><geom type="sphere" size="0.1" mass="-1"/></body></worldbody><keyframe><key qpos="1 2 3 4 5 6 7 8 9"/></keyframe></mujoco>"#,
        "mass, inertia or density are negative in geom",
        "mass, inertia or density are negative in geom (geom #0 in body 'a')",
    ),
    (
        "N4class S4axis same before",
        r#"<mujoco><default><default class="neg"><geom mass="-1"/></default></default><worldbody><body name="a" pos="0 0 1"><joint name="jz" type="hinge" axis="0 0 0"/><joint name="ja" type="hinge" axis="0 1 0"/><geom class="neg" type="sphere" size="0.1"/></body></worldbody></mujoco>"#,
        "mass, inertia or density are negative in geom",
        "mass, inertia or density are negative in geom (geom #0 in body 'a')",
    ),
    (
        "N4dens S5key",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><joint name="ja" type="hinge" axis="0 1 0"/><geom type="sphere" size="0.1" density="-1"/></body></worldbody><keyframe><key qpos="1 2 3 4 5 6 7 8 9"/></keyframe></mujoco>"#,
        "mass, inertia or density are negative in geom",
        "mass, inertia or density are negative in geom (geom #0 in body 'a')",
    ),
    (
        "N3plane N4mass nn",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><joint name="ja" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/><geom type="plane" size="1 1 0.1"/></body><body name="b" pos="1 0 1"><joint name="jb" type="hinge" axis="0 1 0"/><geom type="sphere" size="0.1" mass="-1"/></body></worldbody></mujoco>"#,
        "plane only allowed in static bodies",
        "plane only allowed in static bodies (geom #0 in body 'a')",
    ),
    (
        "N4mass N3plane nn",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><joint name="ja" type="hinge" axis="0 1 0"/><geom type="sphere" size="0.1" mass="-1"/></body><body name="b" pos="1 0 1"><joint name="jb" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/><geom type="plane" size="1 1 0.1"/></body></worldbody></mujoco>"#,
        "mass, inertia or density are negative in geom",
        "mass, inertia or density are negative in geom (geom #0 in body 'a')",
    ),
    (
        "N2fulldiag S4axis n first",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><joint name="ja" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="1 1 1" fullinertia="1 1 1 0 0 0"/></body><body name="b" pos="1 0 1"><joint name="jb" type="hinge" axis="0 0 0"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody></mujoco>"#,
        "fullinertia and diagonal inertia cannot both be specified",
        "invalid fullinertia on <inertial>: fullinertia and diagonal inertia cannot both be specified (body 'a')",
    ),
    (
        "N2fulldiag S4axis s first",
        r#"<mujoco><worldbody><body name="b" pos="1 0 1"><joint name="jb" type="hinge" axis="0 0 0"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body><body name="a" pos="0 0 1"><joint name="ja" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="1 1 1" fullinertia="1 1 1 0 0 0"/></body></worldbody></mujoco>"#,
        "axis too small in joint",
        "invalid axis on <joint>: axis too small (joint 'jb' in body 'b')",
    ),
    (
        "N2fulldiag S4axis same after",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><joint name="ja" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="1 1 1" fullinertia="1 1 1 0 0 0"/><joint name="jz" type="hinge" axis="0 0 0"/></body></worldbody></mujoco>"#,
        "fullinertia and diagonal inertia cannot both be specified",
        "invalid fullinertia on <inertial>: fullinertia and diagonal inertia cannot both be specified (body 'a')",
    ),
    (
        "N2fulldiag S4axis same before",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><joint name="jz" type="hinge" axis="0 0 0"/><joint name="ja" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="1 1 1" fullinertia="1 1 1 0 0 0"/></body></worldbody></mujoco>"#,
        "fullinertia and diagonal inertia cannot both be specified",
        "invalid fullinertia on <inertial>: fullinertia and diagonal inertia cannot both be specified (body 'a')",
    ),
    (
        "N2fulldiag S4freenest",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><joint name="ja" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="1 1 1" fullinertia="1 1 1 0 0 0"/></body><body name="o" pos="2 0 1"><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/><body name="oc"><freejoint/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></body></worldbody></mujoco>"#,
        "fullinertia and diagonal inertia cannot both be specified",
        "invalid fullinertia on <inertial>: fullinertia and diagonal inertia cannot both be specified (body 'a')",
    ),
    (
        "N2fulldiag S4sensor",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><joint name="ja" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="1 1 1" fullinertia="1 1 1 0 0 0"/></body><body name="o" pos="2 0 1"><joint name="jo" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><sensor><jointlimitfrc joint="jo"/></sensor></mujoco>"#,
        "fullinertia and diagonal inertia cannot both be specified",
        "invalid fullinertia on <inertial>: fullinertia and diagonal inertia cannot both be specified (body 'a')",
    ),
    (
        "N2fulldiag S5key",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><joint name="ja" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="1 1 1" fullinertia="1 1 1 0 0 0"/></body></worldbody><keyframe><key qpos="1 2 3 4 5 6 7 8 9"/></keyframe></mujoco>"#,
        "fullinertia and diagonal inertia cannot both be specified",
        "invalid fullinertia on <inertial>: fullinertia and diagonal inertia cannot both be specified (body 'a')",
    ),
    (
        "N5tri S4axis n first",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><joint name="ja" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="1 1 3"/></body><body name="b" pos="1 0 1"><joint name="jb" type="hinge" axis="0 0 0"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody></mujoco>"#,
        "inertia must satisfy A + B >= C; use 'balanceinertia' to fix",
        "inertia must satisfy A + B >= C; use 'balanceinertia' to fix (body 'a')",
    ),
    (
        "N5tri S4axis s first",
        r#"<mujoco><worldbody><body name="b" pos="1 0 1"><joint name="jb" type="hinge" axis="0 0 0"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body><body name="a" pos="0 0 1"><joint name="ja" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="1 1 3"/></body></worldbody></mujoco>"#,
        "axis too small in joint",
        "invalid axis on <joint>: axis too small (joint 'jb' in body 'b')",
    ),
    (
        "N5tri S4axis same after",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><joint name="ja" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="1 1 3"/><joint name="jz" type="hinge" axis="0 0 0"/></body></worldbody></mujoco>"#,
        "inertia must satisfy A + B >= C; use 'balanceinertia' to fix",
        "inertia must satisfy A + B >= C; use 'balanceinertia' to fix (body 'a')",
    ),
    (
        "N5tri S4axis same before",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><joint name="jz" type="hinge" axis="0 0 0"/><joint name="ja" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="1 1 3"/></body></worldbody></mujoco>"#,
        "inertia must satisfy A + B >= C; use 'balanceinertia' to fix",
        "inertia must satisfy A + B >= C; use 'balanceinertia' to fix (body 'a')",
    ),
    (
        "N5tri S4freenest",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><joint name="ja" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="1 1 3"/></body><body name="o" pos="2 0 1"><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/><body name="oc"><freejoint/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></body></worldbody></mujoco>"#,
        "inertia must satisfy A + B >= C; use 'balanceinertia' to fix",
        "inertia must satisfy A + B >= C; use 'balanceinertia' to fix (body 'a')",
    ),
    (
        "N5tri S4sensor",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><joint name="ja" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="1 1 3"/></body><body name="o" pos="2 0 1"><joint name="jo" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><sensor><jointlimitfrc joint="jo"/></sensor></mujoco>"#,
        "inertia must satisfy A + B >= C; use 'balanceinertia' to fix",
        "inertia must satisfy A + B >= C; use 'balanceinertia' to fix (body 'a')",
    ),
    (
        "N5tri S5key",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><joint name="ja" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="1 1 3"/></body></worldbody><keyframe><key qpos="1 2 3 4 5 6 7 8 9"/></keyframe></mujoco>"#,
        "inertia must satisfy A + B >= C; use 'balanceinertia' to fix",
        "inertia must satisfy A + B >= C; use 'balanceinertia' to fix (body 'a')",
    ),
    (
        "N2fulldiag N3plane nn",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><joint name="ja" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="1 1 1" fullinertia="1 1 1 0 0 0"/></body><body name="b" pos="1 0 1"><joint name="jb" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/><geom type="plane" size="1 1 0.1"/></body></worldbody></mujoco>"#,
        "fullinertia and diagonal inertia cannot both be specified",
        "invalid fullinertia on <inertial>: fullinertia and diagonal inertia cannot both be specified (body 'a')",
    ),
    (
        "N3plane N2fulldiag nn",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><joint name="ja" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/><geom type="plane" size="1 1 0.1"/></body><body name="b" pos="1 0 1"><joint name="jb" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="1 1 1" fullinertia="1 1 1 0 0 0"/></body></worldbody></mujoco>"#,
        "plane only allowed in static bodies",
        "plane only allowed in static bodies (geom #0 in body 'a')",
    ),
    (
        "N5tri N4mass nn",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><joint name="ja" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="1 1 3"/></body><body name="b" pos="1 0 1"><joint name="jb" type="hinge" axis="0 1 0"/><geom type="sphere" size="0.1" mass="-1"/></body></worldbody></mujoco>"#,
        "inertia must satisfy A + B >= C; use 'balanceinertia' to fix",
        "inertia must satisfy A + B >= C; use 'balanceinertia' to fix (body 'a')",
    ),
    (
        "N4mass N5tri nn",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><joint name="ja" type="hinge" axis="0 1 0"/><geom type="sphere" size="0.1" mass="-1"/></body><body name="b" pos="1 0 1"><joint name="jb" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="1 1 3"/></body></worldbody></mujoco>"#,
        "mass, inertia or density are negative in geom",
        "mass, inertia or density are negative in geom (geom #0 in body 'a')",
    ),
    (
        "N2fulldiag N5tri nn",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><joint name="ja" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="1 1 1" fullinertia="1 1 1 0 0 0"/></body><body name="b" pos="1 0 1"><joint name="jb" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="1 1 3"/></body></worldbody></mujoco>"#,
        "fullinertia and diagonal inertia cannot both be specified",
        "invalid fullinertia on <inertial>: fullinertia and diagonal inertia cannot both be specified (body 'a')",
    ),
    (
        "N5tri N2fulldiag nn",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><joint name="ja" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="1 1 3"/></body><body name="b" pos="1 0 1"><joint name="jb" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="1 1 1" fullinertia="1 1 1 0 0 0"/></body></worldbody></mujoco>"#,
        "inertia must satisfy A + B >= C; use 'balanceinertia' to fix",
        "inertia must satisfy A + B >= C; use 'balanceinertia' to fix (body 'a')",
    ),
    (
        "o geom then triangle",
        r#"<mujoco><compiler inertiafromgeom="true"/><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="1 1 3"/><geom type="sphere" size="0.1" mass="-1"/></body></worldbody></mujoco>"#,
        "mass, inertia or density are negative in geom",
        "mass, inertia or density are negative in geom (geom #0 in body 'b')",
    ),
    (
        "N7minval S4axis n first",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><joint name="ja" type="hinge" axis="0 1 0"/></body><body name="b" pos="1 0 1"><joint name="jb" type="hinge" axis="0 0 0"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody></mujoco>"#,
        "axis too small in joint",
        "invalid axis on <joint>: axis too small (joint 'jb' in body 'b')",
    ),
    (
        "N7minval S4axis s first",
        r#"<mujoco><worldbody><body name="b" pos="1 0 1"><joint name="jb" type="hinge" axis="0 0 0"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body><body name="a" pos="0 0 1"><joint name="ja" type="hinge" axis="0 1 0"/></body></worldbody></mujoco>"#,
        "axis too small in joint",
        "invalid axis on <joint>: axis too small (joint 'jb' in body 'b')",
    ),
    (
        "N7minval S4axis same after",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><joint name="ja" type="hinge" axis="0 1 0"/><joint name="jz" type="hinge" axis="0 0 0"/></body></worldbody></mujoco>"#,
        "axis too small in joint",
        "invalid axis on <joint>: axis too small (joint 'jz' in body 'a')",
    ),
    (
        "N7minval S4tendlim",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><joint name="ja" type="hinge" axis="0 1 0"/></body><body name="o" pos="2 0 1"><joint name="jo" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><tendon><fixed name="t" limited="true" range="1 0"><joint joint="jo" coef="1"/></fixed></tendon></mujoco>"#,
        "invalid limits in tendon",
        "invalid range on <tendon>: invalid limits in tendon (tendon 't')",
    ),
    (
        "N7minval S4sensor",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><joint name="ja" type="hinge" axis="0 1 0"/></body><body name="o" pos="2 0 1"><joint name="jo" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><sensor><jointlimitfrc joint="jo"/></sensor></mujoco>"#,
        "joint must be limited in sensor",
        "joint must be limited in sensor (sensor #0)",
    ),
    (
        "N7minval S4freenest",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><joint name="ja" type="hinge" axis="0 1 0"/></body><body name="o" pos="2 0 1"><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/><body name="oc"><freejoint/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></body></worldbody></mujoco>"#,
        "mass and inertia of moving bodies must be larger than mjMINVAL",
        "mass and inertia of moving bodies must be larger than mjMINVAL (body 'a')",
    ),
    (
        "N7minval S5key",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><joint name="ja" type="hinge" axis="0 1 0"/></body></worldbody><keyframe><key qpos="1 2 3 4 5 6 7 8 9"/></keyframe></mujoco>"#,
        "mass and inertia of moving bodies must be larger than mjMINVAL",
        "mass and inertia of moving bodies must be larger than mjMINVAL (body 'a')",
    ),
    (
        "N7tiny S4freenest",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><joint name="ja" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1e-16" diaginertia="0.01 0.01 0.01"/></body><body name="o" pos="2 0 1"><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/><body name="oc"><freejoint/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></body></worldbody></mujoco>"#,
        "mass and inertia of moving bodies must be larger than mjMINVAL",
        "mass and inertia of moving bodies must be larger than mjMINVAL (body 'a')",
    ),
    (
        "N7tiny S5key",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><joint name="ja" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1e-16" diaginertia="0.01 0.01 0.01"/></body></worldbody><keyframe><key qpos="1 2 3 4 5 6 7 8 9"/></keyframe></mujoco>"#,
        "mass and inertia of moving bodies must be larger than mjMINVAL",
        "mass and inertia of moving bodies must be larger than mjMINVAL (body 'a')",
    ),
    (
        "N7minval N3plane nn",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><joint name="ja" type="hinge" axis="0 1 0"/></body><body name="b" pos="1 0 1"><joint name="jb" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/><geom type="plane" size="1 1 0.1"/></body></worldbody></mujoco>"#,
        "plane only allowed in static bodies",
        "plane only allowed in static bodies (geom #0 in body 'b')",
    ),
    (
        "N7minval N4mass nn",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><joint name="ja" type="hinge" axis="0 1 0"/></body><body name="b" pos="1 0 1"><joint name="jb" type="hinge" axis="0 1 0"/><geom type="sphere" size="0.1" mass="-1"/></body></worldbody></mujoco>"#,
        "mass, inertia or density are negative in geom",
        "mass, inertia or density are negative in geom (geom #0 in body 'b')",
    ),
    (
        "N7minval N5tri nn",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><joint name="ja" type="hinge" axis="0 1 0"/></body><body name="b" pos="1 0 1"><joint name="jb" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="1 1 3"/></body></worldbody></mujoco>"#,
        "inertia must satisfy A + B >= C; use 'balanceinertia' to fix",
        "inertia must satisfy A + B >= C; use 'balanceinertia' to fix (body 'b')",
    ),
    (
        "N7minval N2fulldiag nn",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><joint name="ja" type="hinge" axis="0 1 0"/></body><body name="b" pos="1 0 1"><joint name="jb" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="1 1 1" fullinertia="1 1 1 0 0 0"/></body></worldbody></mujoco>"#,
        "fullinertia and diagonal inertia cannot both be specified",
        "invalid fullinertia on <inertial>: fullinertia and diagonal inertia cannot both be specified (body 'b')",
    ),
    (
        "Xms S2size x first",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><inertial pos="0 0 0" mass="-1" diaginertia="0.01 0.01 0.01"/></body><body name="b" pos="1 0 1"><joint name="jb" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/><geom type="box" size="0.1 0 0.1"/></body></worldbody></mujoco>"#,
        "size 1 must be positive in geom",
        "invalid size on <geom>: size 1 must be positive (geom #0 in body 'b')",
    ),
    (
        "Xms S3eigen x first",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><inertial pos="0 0 0" mass="-1" diaginertia="0.01 0.01 0.01"/></body><body name="b" pos="1 0 1"><joint name="jb" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" fullinertia="1 1 1 2 0 0"/></body></worldbody></mujoco>"#,
        "error 'inertia must have positive eigenvalues' in fullinertia",
        "invalid fullinertia on <inertial>: inertia must have positive eigenvalues (body 'b')",
    ),
    (
        "Xms S4axis x first",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><inertial pos="0 0 0" mass="-1" diaginertia="0.01 0.01 0.01"/></body><body name="b" pos="1 0 1"><joint name="jb" type="hinge" axis="0 0 0"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody></mujoco>"#,
        "axis too small in joint",
        "invalid axis on <joint>: axis too small (joint 'jb' in body 'b')",
    ),
    (
        "Xms N2fulldiag x first",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><inertial pos="0 0 0" mass="-1" diaginertia="0.01 0.01 0.01"/></body><body name="b" pos="1 0 1"><joint name="jb" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="1 1 1" fullinertia="1 1 1 0 0 0"/></body></worldbody></mujoco>"#,
        "fullinertia and diagonal inertia cannot both be specified",
        "invalid fullinertia on <inertial>: fullinertia and diagonal inertia cannot both be specified (body 'b')",
    ),
    (
        "Xms N3plane x first",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><inertial pos="0 0 0" mass="-1" diaginertia="0.01 0.01 0.01"/></body><body name="b" pos="1 0 1"><joint name="jb" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/><geom type="plane" size="1 1 0.1"/></body></worldbody></mujoco>"#,
        "plane only allowed in static bodies",
        "plane only allowed in static bodies (geom #0 in body 'b')",
    ),
    (
        "Xms N4mass x first",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><inertial pos="0 0 0" mass="-1" diaginertia="0.01 0.01 0.01"/></body><body name="b" pos="1 0 1"><joint name="jb" type="hinge" axis="0 1 0"/><geom type="sphere" size="0.1" mass="-1"/></body></worldbody></mujoco>"#,
        "mass, inertia or density are negative in geom",
        "mass, inertia or density are negative in geom (geom #0 in body 'b')",
    ),
    (
        "Xms N5tri x first",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><inertial pos="0 0 0" mass="-1" diaginertia="0.01 0.01 0.01"/></body><body name="b" pos="1 0 1"><joint name="jb" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="1 1 3"/></body></worldbody></mujoco>"#,
        "inertia must satisfy A + B >= C; use 'balanceinertia' to fix",
        "inertia must satisfy A + B >= C; use 'balanceinertia' to fix (body 'b')",
    ),
    (
        "Xms N7minval x first",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><inertial pos="0 0 0" mass="-1" diaginertia="0.01 0.01 0.01"/></body><body name="b" pos="1 0 1"><joint name="jb" type="hinge" axis="0 1 0"/></body></worldbody></mujoco>"#,
        "mass and inertia of moving bodies must be larger than mjMINVAL",
        "mass and inertia of moving bodies must be larger than mjMINVAL (body 'b')",
    ),
    (
        "Xms S3actref",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><inertial pos="0 0 0" mass="-1" diaginertia="0.01 0.01 0.01"/></body></worldbody><actuator><motor joint="nope"/></actuator></mujoco>"#,
        "unknown transmission target 'nope' for actuator id = 0",
        "unknown joint 'nope' (actuator #0)",
    ),
    (
        "Xms S2tendref",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><inertial pos="0 0 0" mass="-1" diaginertia="0.01 0.01 0.01"/></body></worldbody><tendon><fixed name="t"><joint joint="nope" coef="1"/></fixed></tendon></mujoco>"#,
        "joint 'nope' not found in tendon 0, wrap 0",
        "unknown joint 'nope' (tendon 't')",
    ),
    (
        "Xms S4tendlim",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><inertial pos="0 0 0" mass="-1" diaginertia="0.01 0.01 0.01"/></body><body name="o" pos="2 0 1"><joint name="jo" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><tendon><fixed name="t" limited="true" range="1 0"><joint joint="jo" coef="1"/></fixed></tendon></mujoco>"#,
        "invalid limits in tendon",
        "invalid range on <tendon>: invalid limits in tendon (tendon 't')",
    ),
    (
        "Xms S4sensor",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><inertial pos="0 0 0" mass="-1" diaginertia="0.01 0.01 0.01"/></body><body name="o" pos="2 0 1"><joint name="jo" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><sensor><jointlimitfrc joint="jo"/></sensor></mujoco>"#,
        "joint must be limited in sensor",
        "joint must be limited in sensor (sensor #0)",
    ),
    (
        "Xms S4freenest",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><inertial pos="0 0 0" mass="-1" diaginertia="0.01 0.01 0.01"/></body><body name="o" pos="2 0 1"><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/><body name="oc"><freejoint/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></body></worldbody></mujoco>"#,
        "free joint can only be used on top level",
        "free joint can only be used on top level (body 'oc')",
    ),
    (
        "Xms S5key",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><inertial pos="0 0 0" mass="-1" diaginertia="0.01 0.01 0.01"/></body></worldbody><keyframe><key qpos="1 2 3 4 5 6 7 8 9"/></keyframe></mujoco>"#,
        "keyframe '': invalid qpos size, expected 0, got 9",
        "invalid qpos on <key>: length 9 does not match 0 (key #0)",
    ),
    (
        "Xds N7mass0 x first",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><inertial pos="0 0 0" mass="1" diaginertia="-0.01 0.01 0.01"/></body><body name="b" pos="1 0 1"><joint name="jb" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="0" diaginertia="0.01 0.01 0.01"/></body></worldbody></mujoco>"#,
        "mass and inertia of moving bodies must be larger than mjMINVAL",
        "mass and inertia of moving bodies must be larger than mjMINVAL (body 'b')",
    ),
    (
        "Xdm S4sensor",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><joint name="ja" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="-0.01 0.01 0.01"/></body><body name="o" pos="2 0 1"><joint name="jo" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><sensor><jointlimitfrc joint="jo"/></sensor></mujoco>"#,
        "joint must be limited in sensor",
        "joint must be limited in sensor (sensor #0)",
    ),
    (
        "Xdm S4freenest",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><joint name="ja" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="-0.01 0.01 0.01"/></body><body name="o" pos="2 0 1"><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/><body name="oc"><freejoint/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></body></worldbody></mujoco>"#,
        "mass and inertia of moving bodies must be larger than mjMINVAL",
        "mass and inertia of moving bodies must be larger than mjMINVAL (body 'a')",
    ),
    (
        "Xmm S5key",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><joint name="ja" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="-1" diaginertia="0.01 0.01 0.01"/></body></worldbody><keyframe><key qpos="1 2 3 4 5 6 7 8 9"/></keyframe></mujoco>"#,
        "mass and inertia of moving bodies must be larger than mjMINVAL",
        "mass and inertia of moving bodies must be larger than mjMINVAL (body 'a')",
    ),
    (
        "Xmm S4axis same after",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><joint name="ja" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="-1" diaginertia="0.01 0.01 0.01"/><joint name="jz" type="hinge" axis="0 0 0"/></body></worldbody></mujoco>"#,
        "axis too small in joint",
        "invalid axis on <joint>: axis too small (joint 'jz' in body 'a')",
    ),
    (
        "N6hf3 S4sensor",
        r#"<mujoco><asset><hfield name="hn" size="1 1 0.1 0" nrow="2" ncol="2" elevation="0 0.5 1 0.25"/></asset><worldbody><body name="a" pos="0 0 1"><joint name="ja" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body><body name="o" pos="2 0 1"><joint name="jo" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><sensor><jointlimitfrc joint="jo"/></sensor></mujoco>"#,
        "size parameter is not positive in hfield",
        "size parameter is not positive in hfield (hfield 'hn')",
    ),
    (
        "N6hf3 S4freenest",
        r#"<mujoco><asset><hfield name="hn" size="1 1 0.1 0" nrow="2" ncol="2" elevation="0 0.5 1 0.25"/></asset><worldbody><body name="a" pos="0 0 1"><joint name="ja" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body><body name="o" pos="2 0 1"><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/><body name="oc"><freejoint/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></body></worldbody></mujoco>"#,
        "size parameter is not positive in hfield",
        "size parameter is not positive in hfield (hfield 'hn')",
    ),
    (
        "N6hf3 S5key",
        r#"<mujoco><asset><hfield name="hn" size="1 1 0.1 0" nrow="2" ncol="2" elevation="0 0.5 1 0.25"/></asset><worldbody><body name="a" pos="0 0 1"><joint name="ja" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body></worldbody><keyframe><key qpos="1 2 3 4 5 6 7 8 9"/></keyframe></mujoco>"#,
        "size parameter is not positive in hfield",
        "size parameter is not positive in hfield (hfield 'hn')",
    ),
    (
        "N6hf2 N7minval mn",
        r#"<mujoco><asset><hfield name="hn" size="1 1 0 0.1" nrow="2" ncol="2" elevation="0 0.5 1 0.25"/></asset><worldbody><body name="a" pos="0 0 1"><joint name="ja" type="hinge" axis="0 1 0"/></body></worldbody></mujoco>"#,
        "size parameter is not positive in hfield",
        "size parameter is not positive in hfield (hfield 'hn')",
    ),
    (
        "N6hf3 N7mass0 mn",
        r#"<mujoco><asset><hfield name="hn" size="1 1 0.1 0" nrow="2" ncol="2" elevation="0 0.5 1 0.25"/></asset><worldbody><body name="a" pos="0 0 1"><joint name="ja" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="0" diaginertia="0.01 0.01 0.01"/></body></worldbody></mujoco>"#,
        "size parameter is not positive in hfield",
        "size parameter is not positive in hfield (hfield 'hn')",
    ),
    (
        "N6hf3 N4mass mn",
        r#"<mujoco><asset><hfield name="hn" size="1 1 0.1 0" nrow="2" ncol="2" elevation="0 0.5 1 0.25"/></asset><worldbody><body name="a" pos="0 0 1"><joint name="ja" type="hinge" axis="0 1 0"/><geom type="sphere" size="0.1" mass="-1"/></body></worldbody></mujoco>"#,
        "mass, inertia or density are negative in geom",
        "mass, inertia or density are negative in geom (geom #0 in body 'a')",
    ),
    (
        "N6hf3 N5tri mn",
        r#"<mujoco><asset><hfield name="hn" size="1 1 0.1 0" nrow="2" ncol="2" elevation="0 0.5 1 0.25"/></asset><worldbody><body name="a" pos="0 0 1"><joint name="ja" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="1 1 3"/></body></worldbody></mujoco>"#,
        "inertia must satisfy A + B >= C; use 'balanceinertia' to fix",
        "inertia must satisfy A + B >= C; use 'balanceinertia' to fix (body 'a')",
    ),
    (
        "Xms N6hf3",
        r#"<mujoco><asset><hfield name="hn" size="1 1 0.1 0" nrow="2" ncol="2" elevation="0 0.5 1 0.25"/></asset><worldbody><body name="a" pos="0 0 1"><inertial pos="0 0 0" mass="-1" diaginertia="0.01 0.01 0.01"/></body></worldbody></mujoco>"#,
        "size parameter is not positive in hfield",
        "size parameter is not positive in hfield (hfield 'hn')",
    ),
    (
        "o hfield then spatial armature",
        r#"<mujoco><asset><hfield name="h" size="1 1 1 0" nrow="2" ncol="2" elevation="0 0.5 1 0.25"/></asset><worldbody><site name="s0" pos="0 0 2"/><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/><site name="s1"/></body></worldbody><tendon><spatial name="t" armature="-0.1"><site site="s0"/><site site="s1"/></spatial></tendon></mujoco>"#,
        "size parameter is not positive in hfield",
        "size parameter is not positive in hfield (hfield 'h')",
    ),
    (
        "plane and triangle moving",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="1 1 3"/><geom type="plane" size="1 1 0.1"/></body></worldbody></mujoco>"#,
        "plane only allowed in static bodies",
        "plane only allowed in static bodies (geom #0 in body 'b')",
    ),
    (
        "full diag and plane moving",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="1 1 1" fullinertia="1 1 1 0 0 0"/><geom type="plane" size="1 1 0.1"/></body></worldbody></mujoco>"#,
        "fullinertia and diagonal inertia cannot both be specified",
        "invalid fullinertia on <inertial>: fullinertia and diagonal inertia cannot both be specified (body 'b')",
    ),
    (
        "plane then fullquat body",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/><geom type="plane" size="1 1 0.1"/></body><body name="z" pos="1 0 1"><joint name="k" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" quat="1 0 0 0" fullinertia="1 1 1 0 0 0"/></body></worldbody></mujoco>"#,
        "fullinertia and inertial orientation cannot both be specified",
        "invalid fullinertia on <inertial>: fullinertia and inertial orientation cannot both be specified (line 1: worldbody > body 'z' > inertial)",
    ),
    (
        "name then size",
        r#"<mujoco><worldbody><body name="base" pos="0 0 0"><joint name="basej" type="hinge"/><geom type="sphere" size="0.1" mass="1"/></body><body name="sz" pos="0 0 7"><joint name="szj" type="hinge"/><geom type="sphere" size="0" mass="1"/></body><body name="g1" pos="5 1 0"><joint/><geom name="gg" size="0.1" mass="1"/></body><body name="g2" pos="6 1 0"><joint/><geom name="gg" size="0.1" mass="1"/></body></worldbody></mujoco>"#,
        "repeated name 'gg' in geom",
        "repeated name 'gg' in geom (geom 'gg' in body 'g2')",
    ),
    (
        "name then reference",
        r#"<mujoco><worldbody><body name="base" pos="0 0 0"><joint name="basej" type="hinge"/><geom type="sphere" size="0.1" mass="1"/></body></worldbody><tendon><fixed name="t"><joint joint="basej" coef="1"/></fixed><fixed name="t"><joint joint="basej" coef="2"/></fixed></tendon><actuator><motor name="mr" joint="nope"/></actuator></mujoco>"#,
        "repeated name 't' in tendon",
        "repeated name 't' in tendon",
    ),
    (
        "name then axis",
        r#"<mujoco><worldbody><body name="base" pos="0 0 0"><joint name="basej" type="hinge"/><geom type="sphere" size="0.1" mass="1"/></body><body name="ax" pos="0 0 8"><joint type="hinge" axis="0 0 0"/><geom type="sphere" size="0.1" mass="1"/></body></worldbody><keyframe><key name="k"/><key name="k"/></keyframe></mujoco>"#,
        "repeated name 'k' in key",
        "repeated name 'k' in key",
    ),
    (
        "name then option",
        r#"<mujoco><option timestep="0"/><worldbody><body name="base" pos="0 0 0"><joint name="basej" type="hinge"/><geom type="sphere" size="0.1" mass="1"/></body><body name="a" pos="5 0 0"><joint name="aj" type="hinge"/><geom type="sphere" size="0.1" mass="1"/></body><body name="a" pos="6 0 0"><joint name="aj" type="hinge"/><geom type="sphere" size="0.1" mass="1"/></body></worldbody></mujoco>"#,
        "repeated name 'a' in body",
        "repeated name 'a' in body",
    ),
    (
        "geom name then axis",
        r#"<mujoco><worldbody><body name="base" pos="0 0 0"><joint name="basej" type="hinge"/><geom type="sphere" size="0.1" mass="1"/></body><body name="ax" pos="0 0 8"><joint type="hinge" axis="0 0 0"/><geom type="sphere" size="0.1" mass="1"/></body><body name="g1" pos="5 1 0"><joint/><geom name="gg" size="0.1" mass="1"/></body><body name="g2" pos="6 1 0"><joint/><geom name="gg" size="0.1" mass="1"/></body></worldbody></mujoco>"#,
        "repeated name 'gg' in geom",
        "repeated name 'gg' in geom (geom 'gg' in body 'g2')",
    ),
    (
        "site then geom in frame",
        r#"<mujoco><worldbody><frame><site name="s" zaxis="0 0 0"/><geom name="g" size=".1" zaxis="0 0 0"/></frame></worldbody></mujoco>"#,
        "orientation specification error 'zaxis too small' in geom 0",
        "invalid zaxis on <geom>: zaxis too small (geom 'g' in body 'world')",
    ),
    (
        "site then geom in frame in body",
        r#"<mujoco><worldbody><body name="b"><freejoint/><geom size=".1"/><frame><site name="s" zaxis="0 0 0"/><geom name="g" size=".1" zaxis="0 0 0"/></frame></body></worldbody></mujoco>"#,
        "orientation specification error 'zaxis too small' in geom 1",
        "invalid zaxis on <geom>: zaxis too small (geom 'g' in body 'b')",
    ),
    (
        "body then geom in frame",
        r#"<mujoco><worldbody><frame><body name="c" zaxis="0 0 0"><geom size=".1"/></body><geom name="g" size=".1" zaxis="0 0 0"/></frame></worldbody></mujoco>"#,
        "orientation specification error 'zaxis too small' in geom 0",
        "invalid zaxis on <geom>: zaxis too small (geom 'g' in body 'world')",
    ),
    (
        "frame site xyaxes then geom xyaxes",
        r#"<mujoco><worldbody><frame><site name="s" xyaxes="1 0 0 1 0 0"/><geom name="g" size=".1" xyaxes="1 0 0 1 0 0"/></frame></worldbody></mujoco>"#,
        "orientation specification error 'yaxis too small' in geom 0",
        "invalid xyaxes on <geom>: yaxis too small (geom 'g' in body 'world')",
    ),
    (
        "frame body then site zaxis",
        r#"<mujoco><worldbody><frame><body name="c" zaxis="0 0 0"><geom size=".1"/></body><site name="s" zaxis="0 0 0"/></frame></worldbody></mujoco>"#,
        "orientation specification error 'zaxis too small' in site 0",
        "invalid zaxis on <site>: zaxis too small (site 's' in body 'world')",
    ),
    (
        "geom zaxis in frame1 then frame2 zaxis",
        r#"<mujoco><worldbody><body name="b"><freejoint/><geom size=".1"/><frame><geom name="g" size=".1" zaxis="0 0 0"/></frame><frame name="f2" zaxis="0 0 0"><geom size=".1"/></frame></body></worldbody></mujoco>"#,
        "orientation specification error 'zaxis too small' in site -1",
        "invalid zaxis on <frame>: zaxis too small (frame 'f2' in body 'b')",
    ),
    (
        "frame zaxis then zeroseg cable in frame",
        r#"<mujoco><worldbody><frame zaxis="0 0 0"><geom size=".1"/></frame><frame pos="0 0 1"><composite type="cable" prefix="v" vertex="0 0 0  0.3 0 0  0.3 0 0  0.9 0 0"><joint kind="main" damping="0.1"/><geom type="capsule" size=".01" mass="0.1"/></composite></frame></worldbody></mujoco>"#,
        "orientation specification error 'zaxis too small' in site -1",
        "invalid zaxis on <frame>: zaxis too small (frame in body 'world')",
    ),
    (
        "world frame zaxis then zeroseg cable",
        r#"<mujoco><worldbody><frame zaxis="0 0 0"><geom size=".1"/></frame><composite type="cable" prefix="v" vertex="0 0 0  0.3 0 0  0.3 0 0  0.9 0 0"><joint kind="main" damping="0.1"/><geom type="capsule" size=".01" mass="0.1"/></composite></worldbody></mujoco>"#,
        "orientation specification error 'zaxis too small' in site -1",
        "invalid zaxis on <frame>: zaxis too small (frame in body 'world')",
    ),
    (
        "zeroseg cable then world frame zaxis",
        r#"<mujoco><worldbody><composite type="cable" prefix="v" vertex="0 0 0  0.3 0 0  0.3 0 0  0.9 0 0"><joint kind="main" damping="0.1"/><geom type="capsule" size=".01" mass="0.1"/></composite><frame zaxis="0 0 0"><geom size=".1"/></frame></worldbody></mujoco>"#,
        "orientation specification error 'zaxis too small' in site -1",
        "invalid zaxis on <frame>: zaxis too small (frame in body 'world')",
    ),
    (
        "body frame xyaxes then zeroseg cable",
        r#"<mujoco><worldbody><body name="b"><freejoint/><geom size=".1"/><frame xyaxes="1 0 0 2 0 0"><geom size=".1"/></frame><composite type="cable" prefix="v" vertex="0 0 0  0.3 0 0  0.3 0 0  0.9 0 0"><joint kind="main" damping="0.1"/><geom type="capsule" size=".01" mass="0.1"/></composite></body></worldbody></mujoco>"#,
        "orientation specification error 'yaxis too small' in site -1",
        "invalid xyaxes on <frame>: yaxis too small (frame in body 'b')",
    ),
    (
        "geom in frame zaxis then zeroseg cable",
        r#"<mujoco><worldbody><body name="b"><freejoint/><geom size=".1"/><frame><geom name="g" size=".1" zaxis="0 0 0"/></frame><composite type="cable" prefix="v" vertex="0 0 0  0.3 0 0  0.3 0 0  0.9 0 0"><joint kind="main" damping="0.1"/><geom type="capsule" size=".01" mass="0.1"/></composite></body></worldbody></mujoco>"#,
        "orientation specification error 'zaxis too small' in geom 1",
        "invalid zaxis on <geom>: zaxis too small (geom 'g' in body 'b')",
    ),
    (
        "frame zaxis then bad cable",
        r#"<mujoco><worldbody><frame zaxis="0 0 0"><geom size=".1"/></frame><composite type="cable" count="4 2 1" curve="s" size="1" prefix="k"><geom type="capsule" size=".01"/></composite></worldbody></mujoco>"#,
        "Cable must be one-dimensional",
        "Cable must be one-dimensional (body 'world')",
    ),
    (
        "bad frame holding bad cable",
        r#"<mujoco><worldbody><frame zaxis="0 0 0"><composite type="cable" count="4 2 1" curve="s" size="1" prefix="k"><geom type="capsule" size=".01"/></composite></frame></worldbody></mujoco>"#,
        "Cable must be one-dimensional",
        "Cable must be one-dimensional (body 'world')",
    ),
    (
        "nested geom then outer geom",
        r#"<mujoco><worldbody><frame><frame><geom name="a" size=".1" zaxis="0 0 0"/></frame><geom name="b" size=".1" zaxis="0 0 0"/></frame></worldbody></mujoco>"#,
        "orientation specification error 'zaxis too small' in geom 0",
        "invalid zaxis on <geom>: zaxis too small (geom 'a' in body 'world')",
    ),
    (
        "frame geom size0 then body geom size0",
        r#"<mujoco><worldbody><body name="b"><freejoint/><frame><geom name="a" size="0"/></frame><geom name="b" size="0"/></body></worldbody></mujoco>"#,
        "size 0 must be positive in geom",
        "invalid size on <geom>: size 0 must be positive (geom 'a' in body 'b')",
    ),
    (
        "zeroseg cable in frame then later geom size0",
        r#"<mujoco><worldbody><frame pos="0 0 1"><composite type="cable" prefix="v" vertex="0 0 0  0.3 0 0  0.3 0 0  0.9 0 0"><joint kind="main" damping="0.1"/><geom type="capsule" size=".01" mass="0.1"/></composite></frame><body name="a"><freejoint/><geom name="z" size="0"/></body></worldbody></mujoco>"#,
        "fromto points too close in geom",
        "invalid fromto on <geom>: fromto points too close (geom 'vG1' in body 'vB_1')",
    ),
    (
        "cable in bodyA then bodyB frame zaxis",
        r#"<mujoco><worldbody><body name="a"><composite type="cable" prefix="v" vertex="0 0 0  0.3 0 0  0.3 0 0  0.9 0 0"><joint kind="main" damping="0.1"/><geom type="capsule" size=".01" mass="0.1"/></composite></body><body name="b"><freejoint/><geom size=".1"/><frame name="fb" zaxis="0 0 0"><geom size=".1"/></frame></body></worldbody></mujoco>"#,
        "fromto points too close in geom",
        "invalid fromto on <geom>: fromto points too close (geom 'vG1' in body 'vB_1')",
    ),
    (
        "cable world then world frame body zaxis",
        r#"<mujoco><worldbody><composite type="cable" prefix="v" vertex="0 0 0  0.3 0 0  0.3 0 0  0.9 0 0"><joint kind="main" damping="0.1"/><geom type="capsule" size=".01" mass="0.1"/></composite><frame><body name="c" zaxis="0 0 0"><freejoint/><geom size=".1"/></body></frame></worldbody></mujoco>"#,
        "fromto points too close in geom",
        "invalid fromto on <geom>: fromto points too close (geom 'vG1' in body 'vB_1')",
    ),
    (
        "earlier child frame geom then frame body zaxis",
        r#"<mujoco><worldbody><body name="p"><freejoint/><geom size=".1"/><body name="d"><joint/><geom size=".1"/><frame><geom name="gd" size=".1" zaxis="0 0 0"/></frame></body><frame><body name="c" zaxis="0 0 0"><joint/><geom size=".1"/></body></frame></body></worldbody></mujoco>"#,
        "orientation specification error 'zaxis too small' in geom 2",
        "invalid zaxis on <geom>: zaxis too small (geom 'gd' in body 'd')",
    ),
    (
        "frame body zaxis holding frame zaxis",
        r#"<mujoco><worldbody><body name="p"><freejoint/><geom size=".1"/><frame><body name="c" zaxis="0 0 0"><joint/><geom size=".1"/><frame name="fc" zaxis="0 0 0"><geom size=".1"/></frame></body></frame></body></worldbody></mujoco>"#,
        "orientation specification error 'zaxis too small' in site -1",
        "invalid zaxis on <frame>: zaxis too small (frame 'fc' in body 'c')",
    ),
    (
        "held body zaxis then later body size0",
        r#"<mujoco><worldbody><body name="p"><freejoint/><geom size=".1"/><frame><body name="c" zaxis="0 0 0"><joint/><geom size=".1"/></body></frame></body><body name="q"><freejoint/><geom name="z" size="0"/></body></worldbody></mujoco>"#,
        "error 'zaxis too small' in frame alternative",
        "invalid zaxis on <body>: zaxis too small (body 'c')",
    ),
    (
        "held body zaxis then next held bodys frame geom",
        r#"<mujoco><worldbody><body name="p"><freejoint/><geom size=".1"/><frame><body name="c1" zaxis="0 0 0"><joint/><geom size=".1"/></body><body name="c2"><joint/><geom size=".1"/><frame><geom name="g2" size=".1" zaxis="0 0 0"/></frame></body></frame></body></worldbody></mujoco>"#,
        "error 'zaxis too small' in frame alternative",
        "invalid zaxis on <body>: zaxis too small (body 'c1')",
    ),
    (
        "cable two zero segments",
        r#"<mujoco><worldbody><body name="a"><composite type="cable" prefix="v" vertex="0 0 0  0.3 0 0  0.3 0 0  0.9 0 0  0.9 0 0"><joint kind="main" damping="0.1"/><geom type="capsule" size=".01" mass="0.1"/></composite></body></worldbody></mujoco>"#,
        "fromto points too close in geom",
        "invalid fromto on <geom>: fromto points too close (geom 'vG1' in body 'vB_1')",
    ),
    (
        "condim then its plane",
        r#"<mujoco><worldbody><body name="b"><freejoint/><geom size=".1"/><geom name="g" type="plane" size="1 1 .1" condim="2"/></body></worldbody></mujoco>"#,
        "invalid condim in geom",
        "invalid condim in geom (geom 'g' in body 'b')",
    ),
    (
        "condim then its negative mass",
        r#"<mujoco><worldbody><body name="b"><freejoint/><geom size=".1"/><geom name="g" size=".1" mass="-1" condim="2"/></body></worldbody></mujoco>"#,
        "invalid condim in geom",
        "invalid condim in geom (geom 'g' in body 'b')",
    ),
    (
        "condim then its body's axis",
        r#"<mujoco><worldbody><body name="b"><joint type="hinge" axis="0 0 0"/><geom name="g" size=".1" condim="2"/></body></worldbody></mujoco>"#,
        "invalid condim in geom",
        "invalid condim in geom (geom 'g' in body 'b')",
    ),
    (
        "an earlier body's axis then condim",
        r#"<mujoco><worldbody><body name="a"><joint type="hinge" axis="0 0 0"/><geom size=".1"/></body><body name="b"><freejoint/><geom name="g" size=".1" condim="2"/></body></worldbody></mujoco>"#,
        "axis too small in joint",
        "invalid axis on <joint>: axis too small (joint 'a_joint0' in body 'a')",
    ),
    (
        "condim then a massless moving body",
        r#"<mujoco><worldbody><body name="b"><freejoint/><geom name="g" size=".1" condim="2"/></body><body name="c"><joint type="slide"/></body></worldbody></mujoco>"#,
        "invalid condim in geom",
        "invalid condim in geom (geom 'g' in body 'b')",
    ),
    (
        "condim then a keyframe",
        r#"<mujoco><worldbody><body name="b"><freejoint/><geom name="g" size=".1" condim="2"/></body></worldbody><keyframe><key qpos="0 0 0 1 0 0 0 0"/></keyframe></mujoco>"#,
        "invalid condim in geom",
        "invalid condim in geom (geom 'g' in body 'b')",
    ),
    (
        "its user data then condim",
        r#"<mujoco><size nuser_geom="0"/><worldbody><body name="b"><freejoint/><geom name="g" size=".1" condim="2" user="1"/></body></worldbody></mujoco>"#,
        "user has more values than nuser_geom in geom 'g' (id = 0)",
        "invalid user on <geom>: user data exceeds nuser_geom=0",
    ),
    (
        "condim then a free joint below the top",
        r#"<mujoco><worldbody><body name="b"><freejoint/><geom name="g" size=".1" condim="2"/><body name="c"><freejoint/><geom size=".1"/></body></body></worldbody></mujoco>"#,
        "invalid condim in geom",
        "invalid condim in geom (geom 'g' in body 'b')",
    ),
    (
        "the world's condim then a body's plane",
        r#"<mujoco><worldbody><geom name="w" size=".1" condim="2"/><body name="b"><freejoint/><geom type="plane" size="1 1 .1"/></body></worldbody></mujoco>"#,
        "invalid condim in geom",
        "invalid condim in geom (geom 'w' in body 'world')",
    ),
    (
        "a pair's condim then its geom",
        r#"<mujoco><worldbody><geom name="floor" type="plane" size="1 1 .1"/><body name="b"><freejoint/><geom name="g" size=".1"/></body></worldbody><contact><pair geom1="nope" geom2="floor" condim="2"/></contact></mujoco>"#,
        "invalid condim in contact pair",
        "invalid condim in contact pair (pair #0)",
    ),
    (
        "a pair's unknown geom then a later pair's condim",
        r#"<mujoco><worldbody><geom name="floor" type="plane" size="1 1 .1"/><body name="b"><freejoint/><geom name="g" size=".1"/></body></worldbody><contact><pair geom1="nope" geom2="floor"/><pair geom1="g" geom2="floor" condim="2"/></contact></mujoco>"#,
        "geom 'nope' not found in collision 0",
        "unknown geom 'nope' (pair #0)",
    ),
    (
        "a pair's condim then a later pair's unknown geom",
        r#"<mujoco><worldbody><geom name="floor" type="plane" size="1 1 .1"/><body name="b"><freejoint/><geom name="g" size=".1"/></body></worldbody><contact><pair geom1="g" geom2="floor" condim="2"/><pair geom1="nope" geom2="floor"/></contact></mujoco>"#,
        "invalid condim in contact pair",
        "invalid condim in contact pair (pair #0)",
    ),
    (
        "a pair's condim then a geom's",
        r#"<mujoco><worldbody><geom name="floor" type="plane" size="1 1 .1"/><body name="b"><freejoint/><geom name="g" size=".1" condim="2"/></body></worldbody><contact><pair geom1="g" geom2="floor" condim="2"/></contact></mujoco>"#,
        "invalid condim in contact pair",
        "invalid condim in contact pair (pair #0)",
    ),
    (
        "a pair's condim then a size",
        r#"<mujoco><worldbody><geom name="floor" type="plane" size="1 1 .1"/><body name="b"><freejoint/><geom name="g" size=".1"/><geom name="z" size="0"/></body></worldbody><contact><pair geom1="g" geom2="floor" condim="2"/></contact></mujoco>"#,
        "invalid condim in contact pair",
        "invalid condim in contact pair (pair #0)",
    ),
    (
        "a pair's unknown geom then a size",
        r#"<mujoco><worldbody><geom name="floor" type="plane" size="1 1 .1"/><body name="b"><freejoint/><geom name="g" size=".1"/><geom name="z" size="0"/></body></worldbody><contact><pair geom1="nope" geom2="floor"/></contact></mujoco>"#,
        "geom 'nope' not found in collision 0",
        "unknown geom 'nope' (pair #0)",
    ),
    (
        "a pair's unknown geom2 then a size",
        r#"<mujoco><worldbody><geom name="floor" type="plane" size="1 1 .1"/><body name="b"><freejoint/><geom name="g" size=".1"/><geom name="z" size="0"/></body></worldbody><contact><pair geom1="g" geom2="nope"/></contact></mujoco>"#,
        "geom 'nope' not found in collision 0",
        "unknown geom 'nope' (pair #0)",
    ),
    (
        "a pair's condim then a mesh",
        r#"<mujoco><asset><mesh name="m" vertex="0 0 0 1 0 0 0 1 0 0 0 1" face="0 1 2 0 1 2 0 1 2 0 1 2"/></asset><worldbody><geom name="floor" type="plane" size="1 1 .1"/><body name="b"><freejoint/><geom name="g" size=".1"/><geom type="mesh" mesh="m"/></body></worldbody><contact><pair geom1="g" geom2="floor" condim="2"/></contact></mujoco>"#,
        "invalid condim in contact pair",
        "invalid condim in contact pair (pair #0)",
    ),
    (
        "a pair's condim then a tendon's joint",
        r#"<mujoco><worldbody><geom name="floor" type="plane" size="1 1 .1"/><body name="b"><freejoint/><geom name="g" size=".1"/></body></worldbody><contact><pair geom1="g" geom2="floor" condim="2"/></contact><tendon><fixed name="t"><joint joint="nope" coef="1"/></fixed></tendon></mujoco>"#,
        "invalid condim in contact pair",
        "invalid condim in contact pair (pair #0)",
    ),
    (
        "a pair's unknown geom then an actuator's joint",
        r#"<mujoco><worldbody><geom name="floor" type="plane" size="1 1 .1"/><body name="b"><freejoint/><geom name="g" size=".1"/></body></worldbody><contact><pair geom1="nope" geom2="floor"/></contact><actuator><motor joint="nope"/></actuator></mujoco>"#,
        "geom 'nope' not found in collision 0",
        "unknown geom 'nope' (pair #0)",
    ),
    (
        "a pair's condim then a massless moving body",
        r#"<mujoco><worldbody><geom name="floor" type="plane" size="1 1 .1"/><body name="b"><freejoint/><geom name="g" size=".1"/></body><body name="c"><joint type="slide"/></body></worldbody><contact><pair geom1="g" geom2="floor" condim="2"/></contact></mujoco>"#,
        "invalid condim in contact pair",
        "invalid condim in contact pair (pair #0)",
    ),
    (
        "a pair's condim then a keyframe",
        r#"<mujoco><worldbody><geom name="floor" type="plane" size="1 1 .1"/><body name="b"><freejoint/><geom name="g" size=".1"/></body></worldbody><contact><pair geom1="g" geom2="floor" condim="2"/></contact><keyframe><key qpos="0 0 0 1 0 0 0 0"/></keyframe></mujoco>"#,
        "invalid condim in contact pair",
        "invalid condim in contact pair (pair #0)",
    ),
    (
        "a repeated name then a pair's condim",
        r#"<mujoco><worldbody><geom name="floor" type="plane" size="1 1 .1"/><body name="b"><freejoint/><geom name="g" size=".1"/><geom name="g" size=".1"/></body></worldbody><contact><pair geom1="g" geom2="floor" condim="2"/></contact></mujoco>"#,
        "repeated name 'g' in geom",
        "repeated name 'g' in geom (geom 'g' in body 'b')",
    ),
    (
        "a nuser below -1 then a pair's condim",
        r#"<mujoco><size nuser_geom="-2"/><worldbody><geom name="floor" type="plane" size="1 1 .1"/><body name="b"><freejoint/><geom name="g" size=".1"/></body></worldbody><contact><pair geom1="g" geom2="floor" condim="2"/></contact></mujoco>"#,
        "nuser_geom must be >= -1",
        "invalid nuser_geom on <size>: must be >= -1, got -2 (line 1: size)",
    ),
    (
        "a nuser below -1 then a geom's mesh",
        r#"<mujoco><size nuser_geom="-2"/><worldbody><geom name="floor" type="plane" size="1 1 .1"/><body name="b"><freejoint/><geom name="g" size=".1"/><geom type="mesh" mesh="nope"/></body></worldbody><contact><pair geom1="g" geom2="floor"/></contact></mujoco>"#,
        "nuser_geom must be >= -1",
        "invalid nuser_geom on <size>: must be >= -1, got -2 (line 1: size)",
    ),
    (
        "a flex's condim 7 then a keyframe",
        r#"<mujoco><worldbody><body name="s" pos="0 0 0.105"><freejoint/><geom name="gs" type="sphere" size="0.1" condim="3"/></body><body name="v0" pos="-0.5 -0.5 0"/><body name="v1" pos="0.5 -0.5 0"/><body name="v2" pos="0.5 0.5 0"/><body name="v3" pos="-0.5 0.5 0"/></worldbody><deformable><flex name="f" dim="2" body="v0 v1 v2 v3" element="0 1 2 0 2 3" radius="0.01"><contact condim="7"/></flex></deformable><keyframe><key qpos="0 0 0 1 0 0 0 0"/></keyframe></mujoco>"#,
        "keyframe '': invalid qpos size, expected 7, got 8",
        "invalid qpos on <key>: length 8 does not match 7 (key #0)",
    ),
    (
        "a flex's condim 7 then a body's mass",
        r#"<mujoco><worldbody><body name="s" pos="0 0 0.105"><freejoint/><geom name="gs" type="sphere" size="0.1" condim="3"/></body><body name="v0" pos="-0.5 -0.5 0"/><body name="v1" pos="0.5 -0.5 0"/><body name="v2" pos="0.5 0.5 0"/><body name="v3" pos="-0.5 0.5 0"/><body name="c"><freejoint/></body></worldbody><deformable><flex name="f" dim="2" body="v0 v1 v2 v3" element="0 1 2 0 2 3" radius="0.01"><contact condim="7"/></flex></deformable></mujoco>"#,
        "mass and inertia of moving bodies must be larger than mjMINVAL",
        "mass and inertia of moving bodies must be larger than mjMINVAL (body 'c')",
    ),
    (
        "fusestatic: a plane in a moving body, then a fused body's condim",
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="a" pos="0 0 1"><freejoint/><geom name="ga" size=".1"/><geom name="gp" type="plane" size="1 1 .1"/></body><body name="s" pos="2 0 0"><geom name="gs" size=".1" condim="2"/></body></worldbody></mujoco>"#,
        "plane only allowed in static bodies",
        "plane only allowed in static bodies (geom 'gp' in body 'a')",
    ),
    (
        "fusestatic: a moving body's condim, then a fused body's",
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="a" pos="0 0 1"><freejoint/><geom name="ga" size=".1" condim="5"/></body><body name="s" pos="2 0 0"><geom name="gs" size=".1" condim="2"/></body></worldbody></mujoco>"#,
        "invalid condim in geom",
        "invalid condim in geom (geom 'ga' in body 'a')",
    ),
    (
        "fusestatic: a negative density, then a fused body's condim",
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="a" pos="0 0 1"><freejoint/><geom name="ga" size=".1" density="-1"/></body><body name="s" pos="2 0 0"><geom name="gs" size=".1" condim="2"/></body></worldbody></mujoco>"#,
        "mass, inertia or density are negative in geom",
        "mass, inertia or density are negative in geom (geom 'ga' in body 'a')",
    ),
    (
        "fusestatic: a child's plane, then the condim of a body fused into the parent",
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="a" pos="0 0 1"><freejoint/><geom name="ga" size=".1"/><body name="c" pos="0 0 .3"><joint type="hinge"/><geom name="gc" type="plane" size="1 1 .1"/></body><body name="s" pos="0 .3 0"><geom name="gs" size=".1" condim="2"/></body></body></worldbody></mujoco>"#,
        "plane only allowed in static bodies",
        "plane only allowed in static bodies (geom 'gc' in body 'c')",
    ),
    (
        "fusestatic: a fused body's condim, then a later body's plane",
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="s" pos="2 0 0"><geom name="gs" size=".1" condim="2"/></body><body name="a" pos="0 0 1"><freejoint/><geom name="ga" size=".1"/><geom name="gp" type="plane" size="1 1 .1"/></body></worldbody></mujoco>"#,
        "invalid condim in geom",
        "invalid condim in geom (geom 'gs' in body 's')",
    ),
    (
        "fusestatic: a joint's axis, then a fused body's condim",
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="a" pos="0 0 1"><joint type="hinge" axis="0 0 0"/><geom name="ga" size=".1"/></body><body name="s" pos="2 0 0"><geom name="gs" size=".1" condim="2"/></body></worldbody></mujoco>"#,
        "axis too small in joint",
        "invalid axis on <joint>: axis too small (joint 'a_joint0' in body 'a')",
    ),
    (
        "fusestatic: a fullinertia, then a fused body's condim",
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="a" pos="0 0 1"><freejoint/><inertial pos="0 0 0" mass="1" fullinertia="1 1 1 0 0 0" diaginertia="1 1 1"/><geom name="ga" size=".1"/></body><body name="s" pos="2 0 0"><geom name="gs" size=".1" condim="2"/></body></worldbody></mujoco>"#,
        "fullinertia and diagonal inertia cannot both be specified",
        "invalid fullinertia on <inertial>: fullinertia and diagonal inertia cannot both be specified (body 'a')",
    ),
    (
        "fusestatic: a joint's axis, then a plane fused into its body",
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="a" pos="0 0 1"><joint type="hinge" axis="0 0 0"/><geom name="ga" size=".1"/><body name="s" pos="0 0 .3"><geom name="gp" type="plane" size="1 1 .1"/></body></body></worldbody></mujoco>"#,
        "axis too small in joint",
        "invalid axis on <joint>: axis too small (joint 'a_joint0' in body 'a')",
    ),
    (
        "fusestatic: a joint's axis, then a negative density fused into its body",
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="a" pos="0 0 1"><joint type="hinge" axis="0 0 0"/><geom name="ga" size=".1"/><body name="s" pos="0 0 .3"><geom name="gn" size=".1" density="-1"/></body></body></worldbody></mujoco>"#,
        "axis too small in joint",
        "invalid axis on <joint>: axis too small (joint 'a_joint0' in body 'a')",
    ),
    (
        "fusestatic: two fused bodies' condims, the first fused into a moving body",
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="a" pos="0 0 1"><freejoint/><geom name="ga" size=".1"/><body name="s1" pos="0 0 .3"><geom name="g1" size=".1" condim="2"/></body></body><body name="s2" pos="2 0 0"><geom name="g2" size=".1" condim="5"/></body></worldbody></mujoco>"#,
        "invalid condim in geom",
        "invalid condim in geom (geom 'g1' in body 's1')",
    ),
    (
        "fusestatic: a condim fused through two bodies, then a later joint's axis",
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="s" pos="2 0 0"><geom name="gs" size=".1"/><body name="t" pos="0 0 .3"><geom name="gt" size=".1" condim="2"/></body></body><body name="a" pos="0 0 1"><joint type="hinge" axis="0 0 0"/><geom name="ga" size=".1"/></body></worldbody></mujoco>"#,
        "invalid condim in geom",
        "invalid condim in geom (geom 'gt' in body 't')",
    ),
    (
        "fusestatic: a plane fused into a moving body, then a later body's condim",
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="a" pos="0 0 1"><freejoint/><geom name="ga" size=".1"/><body name="s" pos="0 0 .3"><geom name="gp" type="plane" size="1 1 .1"/></body></body><body name="c" pos="2 0 1"><freejoint/><geom name="gc" size=".1" condim="2"/></body></worldbody></mujoco>"#,
        "plane only allowed in static bodies",
        "plane only allowed in static bodies (geom 'gp' in body 's')",
    ),
    (
        "a nuser below -1 then a repeated body name",
        r#"<mujoco><size nuser_geom="-2"/><worldbody><body name="a" pos="0 0 1"><freejoint/><geom name="ga" size=".1"/></body><body name="a" pos="1 0 1"><freejoint/><geom size=".1"/></body></worldbody></mujoco>"#,
        "nuser_geom must be >= -1",
        "invalid nuser_geom on <size>: must be >= -1, got -2 (line 1: size)",
    ),
    (
        "a nuser below -1 then a repeated pair name",
        r#"<mujoco><size nuser_geom="-2"/><worldbody><geom name="floor" type="plane" size="1 1 .1"/><body name="a" pos="0 0 1"><freejoint/><geom name="ga" size=".1"/></body></worldbody><contact><pair name="p" geom1="ga" geom2="floor"/><pair name="p" geom1="ga" geom2="floor"/></contact></mujoco>"#,
        "nuser_geom must be >= -1",
        "invalid nuser_geom on <size>: must be >= -1, got -2 (line 1: size)",
    ),
    (
        "a nuser below -1 then a repeated geom name",
        r#"<mujoco><size nuser_geom="-2"/><worldbody><geom name="ga" type="plane" size="1 1 .1"/><body name="a" pos="0 0 1"><freejoint/><geom name="ga" size=".1"/></body></worldbody></mujoco>"#,
        "nuser_geom must be >= -1",
        "invalid nuser_geom on <size>: must be >= -1, got -2 (line 1: size)",
    ),
    (
        "a nuser below -1 then an undefined class",
        r#"<mujoco><size nuser_geom="-2"/><worldbody><body name="a" pos="0 0 1"><freejoint/><geom class="nope" size=".1"/></body></worldbody></mujoco>"#,
        "nuser_geom must be >= -1",
        "invalid nuser_geom on <size>: must be >= -1, got -2 (line 1: size)",
    ),
    (
        "two <size>, the first's nuser below -1",
        r#"<mujoco><size nuser_geom="-3"/><size nuser_body="-2"/><worldbody><body name="a" pos="0 0 1"><freejoint/><geom name="ga" size=".1"/></body></worldbody></mujoco>"#,
        "nuser_geom must be >= -1",
        "invalid nuser_geom on <size>: must be >= -1, got -3 (line 1: size)",
    ),
    (
        "a nuser below -1 then a repeated actuator name",
        r#"<mujoco><size nuser_sensor="-2"/><worldbody><body name="a" pos="0 0 1"><joint name="j" type="hinge"/><geom name="ga" size=".1"/></body></worldbody><actuator><motor name="m" joint="j"/><motor name="m" joint="j"/></actuator></mujoco>"#,
        "nuser_sensor must be >= -1",
        "invalid nuser_sensor on <size>: must be >= -1, got -2 (line 1: size)",
    ),
    (
        "a nuser below -1 then a repeated tendon name",
        r#"<mujoco><size nuser_tendon="-2"/><worldbody><body name="a" pos="0 0 1"><joint name="j" type="hinge"/><geom name="ga" size=".1"/></body></worldbody><tendon><fixed name="t"><joint joint="j" coef="1"/></fixed><fixed name="t"><joint joint="j" coef="1"/></fixed></tendon></mujoco>"#,
        "nuser_tendon must be >= -1",
        "invalid nuser_tendon on <size>: must be >= -1, got -2 (line 1: size)",
    ),
    (
        "a nuser below -1 then a repeated frame name",
        r#"<mujoco><size nuser_geom="-2"/><worldbody><frame name="f"/><frame name="f"/><body name="a" pos="0 0 1"><freejoint/><geom name="ga" size=".1"/></body></worldbody></mujoco>"#,
        "nuser_geom must be >= -1",
        "invalid nuser_geom on <size>: must be >= -1, got -2 (line 1: size)",
    ),
    (
        "a nuser below -1 then a sphere's fromto",
        r#"<mujoco><size nuser_geom="-2"/><worldbody><body name="a" pos="0 0 1"><freejoint/><geom type="sphere" size=".1" fromto="0 0 0 0 0 1"/></body></worldbody></mujoco>"#,
        "nuser_geom must be >= -1",
        "invalid nuser_geom on <size>: must be >= -1, got -2 (line 1: size)",
    ),
    (
        "a nuser below -1 then a parse fault",
        r#"<mujoco><size nuser_geom="-2"/><worldbody><body name="b"><freejoint/><geom name="g" type="nosuch" size=".1"/></body></worldbody></mujoco>"#,
        "nuser_geom must be >= -1",
        "invalid nuser_geom on <size>: must be >= -1, got -2 (line 1: size)",
    ),
    (
        "a nuser below -1, written last, then a repeated name",
        r#"<mujoco><worldbody><body name="b"><freejoint/><geom name="g" size=".1"/><geom name="g" size=".1"/></body></worldbody><size nuser_geom="-2"/></mujoco>"#,
        "nuser_geom must be >= -1",
        "invalid nuser_geom on <size>: must be >= -1, got -2 (line 1: size)",
    ),
    (
        "a camera's nuser below -1 then a repeated name",
        r#"<mujoco><size nuser_cam="-2"/><worldbody><body name="a" pos="0 0 1"><freejoint/><geom name="g" size=".1"/><geom name="g" size=".1"/></body></worldbody></mujoco>"#,
        "nuser_cam must be >= -1",
        "invalid nuser_cam on <size>: must be >= -1, got -2 (line 1: size)",
    ),
    (
        "a pair naming an empty geom, then a joint's axis",
        r#"<mujoco><worldbody><geom name="floor" type="plane" size="1 1 .1"/><body name="b"><freejoint/><geom name="g" size=".1"/><geom size=".2"/></body><body name="c"><joint type="hinge" axis="0 0 0"/><geom size=".1"/></body></worldbody><contact><pair geom1="" geom2="g"/></contact></mujoco>"#,
        "axis too small in joint",
        "invalid axis on <joint>: axis too small (joint 'c_joint0' in body 'c')",
    ),
    (
        "a pair naming an empty geom, then a keyframe",
        r#"<mujoco><worldbody><geom name="floor" type="plane" size="1 1 .1"/><body name="b"><freejoint/><geom name="g" size=".1"/><geom size=".2"/></body></worldbody><contact><pair geom1="" geom2="g"/></contact><keyframe><key qpos="1 2 3 4 5 6 7 8"/></keyframe></mujoco>"#,
        "invalid qpos size, expected 7, got 8",
        "invalid qpos on <key>: length 8 does not match 7 (key #0)",
    ),
    (
        "a pair naming an empty geom, then an actuator's joint",
        r#"<mujoco><worldbody><geom name="floor" type="plane" size="1 1 .1"/><body name="b"><freejoint/><geom name="g" size=".1"/><geom size=".2"/></body></worldbody><contact><pair geom1="" geom2="g"/></contact><actuator><motor joint="nope"/></actuator></mujoco>"#,
        "unknown transmission target 'nope' for actuator id = 0",
        "unknown joint 'nope' (actuator #0)",
    ),
    (
        "a pair naming an empty geom, every geom named, then a size",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><freejoint/><geom name="ga" size=".1"/></body><body name="c" pos="1 0 1"><freejoint/><geom name="z" size="0"/></body></worldbody><contact><pair geom1="" geom2="ga"/></contact></mujoco>"#,
        "geom '' not found in collision 0",
        "unknown geom '' (pair #0)",
    ),
    (
        "a pair's empty geom2, every geom named, then a keyframe",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><freejoint/><geom name="ga" size=".1"/></body></worldbody><contact><pair geom1="ga" geom2=""/></contact><keyframe><key qpos="0 0 0 1 0 0 0 9"/></keyframe></mujoco>"#,
        "geom '' not found in collision 0",
        "unknown geom '' (pair #0)",
    ),
    (
        "a <size>'s nuser below -1, then a <compiler>'s angle",
        r#"<mujoco><size nuser_geom="-2"/><compiler angle="foo"/><worldbody><body name="a"><freejoint/><geom name="ga" size=".1"/></body></worldbody></mujoco>"#,
        "invalid keyword: 'foo'",
        "invalid keyword \"foo\" in attribute 'angle' on <compiler> (line 1: compiler)",
    ),
    (
        "a <size>'s nuser below -1, then an <option>'s integrator",
        r#"<mujoco><size nuser_geom="-2"/><option integrator="bogus"/><worldbody><body name="a"><freejoint/><geom name="ga" size=".1"/></body></worldbody></mujoco>"#,
        "invalid keyword: 'bogus'",
        "invalid keyword \"bogus\" in attribute 'integrator' on <option> (line 1: option)",
    ),
    (
        "a <size>'s nuser below -1, then an <option>'s timestep",
        r#"<mujoco><size nuser_geom="-2"/><option timestep="abc"/><worldbody><body name="a"><freejoint/><geom name="ga" size=".1"/></body></worldbody></mujoco>"#,
        "bad format in attribute 'timestep'",
        "bad number \"abc\" in attribute 'timestep' on <option> (line 1: option)",
    ),
    (
        "a <size>'s nuser below -1, then a <compiler>'s inertiafromgeom",
        r#"<mujoco><size nuser_geom="-2"/><compiler inertiafromgeom="maybe"/><worldbody><body name="a"><freejoint/><geom name="ga" size=".1"/></body></worldbody></mujoco>"#,
        "invalid keyword: 'maybe'",
        "invalid keyword \"maybe\" in attribute 'inertiafromgeom' on <compiler> (line 1: compiler)",
    ),
    (
        "a <size>'s nuser below -1, then an <option>'s flag",
        r#"<mujoco><size nuser_geom="-2"/><option><flag gravity="maybe"/></option><worldbody><body name="a"><freejoint/><geom name="ga" size=".1"/></body></worldbody></mujoco>"#,
        "invalid keyword: 'maybe'",
        "invalid keyword \"maybe\" in attribute 'gravity' on <flag> (line 1: option > flag)",
    ),
    (
        "a <size>'s nuser below -1, then a <compiler>'s boundmass",
        r#"<mujoco><size nuser_geom="-2"/><compiler boundmass="x"/><worldbody><body name="a"><freejoint/><geom name="ga" size=".1"/></body></worldbody></mujoco>"#,
        "bad format in attribute 'boundmass'",
        "bad number \"x\" in attribute 'boundmass' on <compiler> (line 1: compiler)",
    ),
    (
        "a <size>'s nuser below -1, then an <option>'s gravity",
        r#"<mujoco><size nuser_geom="-2"/><option gravity="0 0"/><worldbody><body name="a"><freejoint/><geom name="ga" size=".1"/></body></worldbody></mujoco>"#,
        "attribute 'gravity' does not have enough data",
        "expected exactly 3 values, found 2 in attribute 'gravity' on <option> (line 1: option)",
    ),
    (
        "a <size>'s nuser below -1, then an <option>'s cone",
        r#"<mujoco><size nuser_geom="-2"/><option cone="round"/><worldbody><body name="a"><freejoint/><geom name="ga" size=".1"/></body></worldbody></mujoco>"#,
        "invalid keyword: 'round'",
        "invalid keyword \"round\" in attribute 'cone' on <option> (line 1: option)",
    ),
    (
        "one <size>'s nuser_geom then nuser_body below -1",
        r#"<mujoco><size nuser_geom="-2" nuser_body="-3"/><worldbody><body name="a" pos="0 0 1"><freejoint/><geom name="g" size=".1"/></body></worldbody></mujoco>"#,
        "nuser_body must be >= -1",
        "invalid nuser_body on <size>: must be >= -1, got -3 (line 1: size)",
    ),
    (
        "one <size>'s nuser_tendon then nuser_cam below -1",
        r#"<mujoco><size nuser_tendon="-2" nuser_cam="-3"/><worldbody><body name="a" pos="0 0 1"><freejoint/><geom name="g" size=".1"/></body></worldbody></mujoco>"#,
        "nuser_cam must be >= -1",
        "invalid nuser_cam on <size>: must be >= -1, got -3 (line 1: size)",
    ),
    (
        "negative cutoff beside a height field's size",
        r#"<mujoco model="p_C_hfsize"><asset><hfield name="hf" nrow="3" ncol="3" size="1 1 0 0.1" elevation="0 0 0 0 0 0 0 0 0"/></asset><worldbody><geom name="floor" type="plane" size="1 1 0.1"/><geom name="hfg" type="hfield" hfield="hf" pos="5 0 0"/><body name="b" pos="0 0 0.5" ><joint name="j" type="hinge" axis="0 1 0" /><geom name="s" type="sphere" size="0.1" mass="1" /><site name="st" size="0.01"/></body><body name="c" pos="0.5 0 0.5"><geom name="s2" type="sphere" size="0.1" /></body></worldbody><contact></contact><equality></equality><tendon></tendon><actuator></actuator><sensor><jointpos joint="j" cutoff="-0.1"/></sensor><keyframe></keyframe></mujoco>"#,
        "size parameter is not positive in hfield",
        "size parameter is not positive in hfield (hfield 'hf')",
    ),
    (
        "negative cutoff beside a joint's range",
        r#"<mujoco model="p_C_jrange"><asset><hfield name="hf" nrow="3" ncol="3" size="1 1 0.1 0.1" elevation="0 0 0 0 0 0 0 0 0"/></asset><worldbody><geom name="floor" type="plane" size="1 1 0.1"/><geom name="hfg" type="hfield" hfield="hf" pos="5 0 0"/><body name="b" pos="0 0 0.5" ><joint name="j" type="hinge" axis="0 1 0" limited="true" range="1 0"/><geom name="s" type="sphere" size="0.1" mass="1" /><site name="st" size="0.01"/></body><body name="c" pos="0.5 0 0.5"><geom name="s2" type="sphere" size="0.1" /></body></worldbody><contact></contact><equality></equality><tendon></tendon><actuator></actuator><sensor><jointpos joint="j" cutoff="-0.1"/></sensor><keyframe></keyframe></mujoco>"#,
        "range[0] should be smaller than range[1] in joint",
        "invalid range on <joint>: range[0] should be smaller than range[1] in joint (joint 'j' in body 'b')",
    ),
    (
        "height-field distance beside its own negative interval",
        r#"<mujoco model="p_H_interval_same"><asset><hfield name="hf" nrow="3" ncol="3" size="1 1 0.1 0.1" elevation="0 0 0 0 0 0 0 0 0"/></asset><worldbody><geom name="floor" type="plane" size="1 1 0.1"/><geom name="hfg" type="hfield" hfield="hf" pos="5 0 0"/><body name="b" pos="0 0 0.5" ><joint name="j" type="hinge" axis="0 1 0" /><geom name="s" type="sphere" size="0.1" mass="1" /><site name="st" size="0.01"/></body><body name="c" pos="0.5 0 0.5"><geom name="s2" type="sphere" size="0.1" /></body></worldbody><contact></contact><equality></equality><tendon></tendon><actuator></actuator><sensor><distance geom1="hfg" geom2="s" interval="-1"/></sensor><keyframe></keyframe></mujoco>"#,
        "negative interval in sensor",
        "invalid interval on <sensor>: negative interval in sensor (sensor #0)",
    ),
    (
        "height-field distance beside an earlier sensor's unknown object",
        r#"<mujoco model="p_H_obj_before"><asset><hfield name="hf" nrow="3" ncol="3" size="1 1 0.1 0.1" elevation="0 0 0 0 0 0 0 0 0"/></asset><worldbody><geom name="floor" type="plane" size="1 1 0.1"/><geom name="hfg" type="hfield" hfield="hf" pos="5 0 0"/><body name="b" pos="0 0 0.5" ><joint name="j" type="hinge" axis="0 1 0" /><geom name="s" type="sphere" size="0.1" mass="1" /><site name="st" size="0.01"/></body><body name="c" pos="0.5 0 0.5"><geom name="s2" type="sphere" size="0.1" /></body></worldbody><contact></contact><equality></equality><tendon></tendon><actuator></actuator><sensor><jointvel joint="nope"/><distance geom1="hfg" geom2="s"/></sensor><keyframe></keyframe></mujoco>"#,
        "unrecognized name 'nope' of sensorized object",
        "unknown joint 'nope' (sensor #0)",
    ),
    (
        "height-field distance beside a tendon's limits",
        r#"<mujoco model="p_H_tendon"><asset><hfield name="hf" nrow="3" ncol="3" size="1 1 0.1 0.1" elevation="0 0 0 0 0 0 0 0 0"/></asset><worldbody><geom name="floor" type="plane" size="1 1 0.1"/><geom name="hfg" type="hfield" hfield="hf" pos="5 0 0"/><body name="b" pos="0 0 0.5" ><joint name="j" type="hinge" axis="0 1 0" /><geom name="s" type="sphere" size="0.1" mass="1" /><site name="st" size="0.01"/></body><body name="c" pos="0.5 0 0.5"><geom name="s2" type="sphere" size="0.1" /></body></worldbody><contact></contact><equality></equality><tendon><fixed name="t" limited="true" range="1 0"><joint joint="j" coef="1"/></fixed></tendon><actuator></actuator><sensor><distance geom1="hfg" geom2="s"/></sensor><keyframe></keyframe></mujoco>"#,
        "invalid limits in tendon",
        "invalid range on <tendon>: invalid limits in tendon (tendon 't')",
    ),
    (
        "negative noise beside an actuator's control range",
        r#"<mujoco model="p_N_actuator"><asset><hfield name="hf" nrow="3" ncol="3" size="1 1 0.1 0.1" elevation="0 0 0 0 0 0 0 0 0"/></asset><worldbody><geom name="floor" type="plane" size="1 1 0.1"/><geom name="hfg" type="hfield" hfield="hf" pos="5 0 0"/><body name="b" pos="0 0 0.5" ><joint name="j" type="hinge" axis="0 1 0" /><geom name="s" type="sphere" size="0.1" mass="1" /><site name="st" size="0.01"/></body><body name="c" pos="0.5 0 0.5"><geom name="s2" type="sphere" size="0.1" /></body></worldbody><contact></contact><equality></equality><tendon></tendon><actuator><motor joint="j" ctrllimited="true" ctrlrange="1 0"/></actuator><sensor><jointpos joint="j" noise="-0.1"/></sensor><keyframe></keyframe></mujoco>"#,
        "invalid control range for actuator",
        "invalid ctrlrange on <actuator>: invalid control range for actuator (actuator #0)",
    ),
    (
        "negative noise beside a body's user data",
        r#"<mujoco model="p_N_buser"><size nuser_body="1"/><asset><hfield name="hf" nrow="3" ncol="3" size="1 1 0.1 0.1" elevation="0 0 0 0 0 0 0 0 0"/></asset><worldbody><geom name="floor" type="plane" size="1 1 0.1"/><geom name="hfg" type="hfield" hfield="hf" pos="5 0 0"/><body name="b" pos="0 0 0.5" user="1 2"><joint name="j" type="hinge" axis="0 1 0" /><geom name="s" type="sphere" size="0.1" mass="1" /><site name="st" size="0.01"/></body><body name="c" pos="0.5 0 0.5"><geom name="s2" type="sphere" size="0.1" /></body></worldbody><contact></contact><equality></equality><tendon></tendon><actuator></actuator><sensor><jointpos joint="j" noise="-0.1"/></sensor><keyframe></keyframe></mujoco>"#,
        "user has more values than nuser_body in body",
        "invalid user on <body>: user data exceeds nuser_body=1",
    ),
    (
        "negative noise beside a geom's condim",
        r#"<mujoco model="p_N_gcondim"><asset><hfield name="hf" nrow="3" ncol="3" size="1 1 0.1 0.1" elevation="0 0 0 0 0 0 0 0 0"/></asset><worldbody><geom name="floor" type="plane" size="1 1 0.1"/><geom name="hfg" type="hfield" hfield="hf" pos="5 0 0"/><body name="b" pos="0 0 0.5" ><joint name="j" type="hinge" axis="0 1 0" /><geom name="s" type="sphere" size="0.1" mass="1" condim="2"/><site name="st" size="0.01"/></body><body name="c" pos="0.5 0 0.5"><geom name="s2" type="sphere" size="0.1" /></body></worldbody><contact></contact><equality></equality><tendon></tendon><actuator></actuator><sensor><jointpos joint="j" noise="-0.1"/></sensor><keyframe></keyframe></mujoco>"#,
        "invalid condim in geom",
        "invalid condim in geom (geom 's' in body 'b')",
    ),
    (
        "negative noise beside a keyframe's qpos",
        r#"<mujoco model="p_N_key"><asset><hfield name="hf" nrow="3" ncol="3" size="1 1 0.1 0.1" elevation="0 0 0 0 0 0 0 0 0"/></asset><worldbody><geom name="floor" type="plane" size="1 1 0.1"/><geom name="hfg" type="hfield" hfield="hf" pos="5 0 0"/><body name="b" pos="0 0 0.5" ><joint name="j" type="hinge" axis="0 1 0" /><geom name="s" type="sphere" size="0.1" mass="1" /><site name="st" size="0.01"/></body><body name="c" pos="0.5 0 0.5"><geom name="s2" type="sphere" size="0.1" /></body></worldbody><contact></contact><equality></equality><tendon></tendon><actuator></actuator><sensor><jointpos joint="j" noise="-0.1"/></sensor><keyframe><key qpos="1 2 3"/></keyframe></mujoco>"#,
        "negative noise in sensor",
        "negative noise in sensor (sensor #0)",
    ),
    (
        "negative noise beside a later unlimited limit sensor",
        r#"<mujoco model="p_N_limit_after"><asset><hfield name="hf" nrow="3" ncol="3" size="1 1 0.1 0.1" elevation="0 0 0 0 0 0 0 0 0"/></asset><worldbody><geom name="floor" type="plane" size="1 1 0.1"/><geom name="hfg" type="hfield" hfield="hf" pos="5 0 0"/><body name="b" pos="0 0 0.5" ><joint name="j" type="hinge" axis="0 1 0" /><geom name="s" type="sphere" size="0.1" mass="1" /><site name="st" size="0.01"/></body><body name="c" pos="0.5 0 0.5"><geom name="s2" type="sphere" size="0.1" /></body></worldbody><contact></contact><equality></equality><tendon></tendon><actuator></actuator><sensor><jointpos joint="j" noise="-0.1"/><jointlimitfrc joint="j"/></sensor><keyframe></keyframe></mujoco>"#,
        "negative noise in sensor",
        "negative noise in sensor (sensor #0)",
    ),
    (
        "negative noise beside an earlier unlimited limit sensor",
        r#"<mujoco model="p_N_limit_before"><asset><hfield name="hf" nrow="3" ncol="3" size="1 1 0.1 0.1" elevation="0 0 0 0 0 0 0 0 0"/></asset><worldbody><geom name="floor" type="plane" size="1 1 0.1"/><geom name="hfg" type="hfield" hfield="hf" pos="5 0 0"/><body name="b" pos="0 0 0.5" ><joint name="j" type="hinge" axis="0 1 0" /><geom name="s" type="sphere" size="0.1" mass="1" /><site name="st" size="0.01"/></body><body name="c" pos="0.5 0 0.5"><geom name="s2" type="sphere" size="0.1" /></body></worldbody><contact></contact><equality></equality><tendon></tendon><actuator></actuator><sensor><jointlimitfrc joint="j"/><jointpos joint="j" noise="-0.1"/></sensor><keyframe></keyframe></mujoco>"#,
        "joint must be limited in sensor",
        "joint must be limited in sensor (sensor #0)",
    ),
    (
        "negative noise beside a massless moving body",
        r#"<mujoco model="p_N_mass"><asset><hfield name="hf" nrow="3" ncol="3" size="1 1 0.1 0.1" elevation="0 0 0 0 0 0 0 0 0"/></asset><worldbody><geom name="floor" type="plane" size="1 1 0.1"/><geom name="hfg" type="hfield" hfield="hf" pos="5 0 0"/><body name="b" pos="0 0 0.5" ><joint name="j" type="hinge" axis="0 1 0" /><geom name="s" type="sphere" size="0.1" mass="1" /><site name="st" size="0.01"/></body><body name="c" pos="0.5 0 0.5"><joint name="cj" type="hinge"/><geom name="s2" type="sphere" size="0.1" mass="0"/></body></worldbody><contact></contact><equality></equality><tendon></tendon><actuator></actuator><sensor><jointpos joint="j" noise="-0.1"/></sensor><keyframe></keyframe></mujoco>"#,
        "negative noise in sensor",
        "negative noise in sensor (sensor #0)",
    ),
    (
        "negative noise beside its own user data",
        r#"<mujoco model="p_N_suser_same"><size nuser_sensor="1"/><asset><hfield name="hf" nrow="3" ncol="3" size="1 1 0.1 0.1" elevation="0 0 0 0 0 0 0 0 0"/></asset><worldbody><geom name="floor" type="plane" size="1 1 0.1"/><geom name="hfg" type="hfield" hfield="hf" pos="5 0 0"/><body name="b" pos="0 0 0.5" ><joint name="j" type="hinge" axis="0 1 0" /><geom name="s" type="sphere" size="0.1" mass="1" /><site name="st" size="0.01"/></body><body name="c" pos="0.5 0 0.5"><geom name="s2" type="sphere" size="0.1" /></body></worldbody><contact></contact><equality></equality><tendon></tendon><actuator></actuator><sensor><jointpos joint="j" noise="-0.1" user="1 2"/></sensor><keyframe></keyframe></mujoco>"#,
        "user has more values than nuser_sensor in sensor",
        "invalid user on <sensor>: user data exceeds nuser_sensor=1",
    ),
    (
        "quaternion cutoff beside its own negative interval",
        r#"<mujoco model="p_Q_interval_same"><asset><hfield name="hf" nrow="3" ncol="3" size="1 1 0.1 0.1" elevation="0 0 0 0 0 0 0 0 0"/></asset><worldbody><geom name="floor" type="plane" size="1 1 0.1"/><geom name="hfg" type="hfield" hfield="hf" pos="5 0 0"/><body name="b" pos="0 0 0.5" ><joint name="j" type="hinge" axis="0 1 0" /><geom name="s" type="sphere" size="0.1" mass="1" /><site name="st" size="0.01"/></body><body name="c" pos="0.5 0 0.5"><geom name="s2" type="sphere" size="0.1" /></body></worldbody><contact></contact><equality></equality><tendon></tendon><actuator></actuator><sensor><framequat objtype="site" objname="st" cutoff="0.5" interval="-1"/></sensor><keyframe></keyframe></mujoco>"#,
        "negative interval in sensor",
        "invalid interval on <sensor>: negative interval in sensor (sensor #0)",
    ),
    (
        "quaternion cutoff beside its own unknown object",
        r#"<mujoco model="p_Q_obj_same"><asset><hfield name="hf" nrow="3" ncol="3" size="1 1 0.1 0.1" elevation="0 0 0 0 0 0 0 0 0"/></asset><worldbody><geom name="floor" type="plane" size="1 1 0.1"/><geom name="hfg" type="hfield" hfield="hf" pos="5 0 0"/><body name="b" pos="0 0 0.5" ><joint name="j" type="hinge" axis="0 1 0" /><geom name="s" type="sphere" size="0.1" mass="1" /><site name="st" size="0.01"/></body><body name="c" pos="0.5 0 0.5"><geom name="s2" type="sphere" size="0.1" /></body></worldbody><contact></contact><equality></equality><tendon></tendon><actuator></actuator><sensor><framequat objtype="site" objname="nope" cutoff="0.5"/></sensor><keyframe></keyframe></mujoco>"#,
        "unrecognized name 'nope' of sensorized object",
        "unknown site 'nope' (sensor #0)",
    ),
    (
        "quaternion cutoff beside its own user data",
        r#"<mujoco model="p_Q_suser_same"><size nuser_sensor="1"/><asset><hfield name="hf" nrow="3" ncol="3" size="1 1 0.1 0.1" elevation="0 0 0 0 0 0 0 0 0"/></asset><worldbody><geom name="floor" type="plane" size="1 1 0.1"/><geom name="hfg" type="hfield" hfield="hf" pos="5 0 0"/><body name="b" pos="0 0 0.5" ><joint name="j" type="hinge" axis="0 1 0" /><geom name="s" type="sphere" size="0.1" mass="1" /><site name="st" size="0.01"/></body><body name="c" pos="0.5 0 0.5"><geom name="s2" type="sphere" size="0.1" /></body></worldbody><contact></contact><equality></equality><tendon></tendon><actuator></actuator><sensor><framequat objtype="site" objname="st" cutoff="0.5" user="1 2"/></sensor><keyframe></keyframe></mujoco>"#,
        "user has more values than nuser_sensor in sensor",
        "invalid user on <sensor>: user data exceeds nuser_sensor=1",
    ),
    (
        "a bending flex's thickness then a height field's size",
        r#"<mujoco><default></default><worldbody><body name="v0" pos="0 0 0"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body><body name="v1" pos="1 0 0"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body><body name="v2" pos="0 1 0"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body><body name="v3" pos="1 1 0"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body></worldbody><deformable><flex name="f" dim="2" body="v0 v1 v2 v3" element="0 1 2  1 3 2" ><contact contype="0" conaffinity="0"/><elasticity elastic2d="bend" young="10000.0" thickness="-0.5"/></flex></deformable><equality></equality><asset><hfield name="h" nrow="2" ncol="2" elevation="0 0 0 0" size="1 1 0 0.1"/></asset></mujoco>"#,
        "thickness must be positive for bending stiffness",
        "invalid thickness on <elasticity>: thickness must be positive for bending stiffness (flex 'f')",
    ),
    (
        "a flex vertex equality then a joint axis",
        r#"<mujoco><default></default><worldbody><body name="v0" pos="0 0 0"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body><body name="v1" pos="1 0 0"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body><body name="v2" pos="0 1 0"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body><body name="v3" pos="1 1 0"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body><body name="j" pos="5 0 0"><joint type="hinge" axis="0 0 0"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body></worldbody><deformable><flex name="f" dim="2" body="v0 v1 v2 v3" element="0 1 2  1 3 2" ><contact contype="0" conaffinity="0"/></flex></deformable><equality><flexvert flex="f"/></equality></mujoco>"#,
        "axis too small in joint",
        "invalid axis on <joint>: axis too small (joint 'j_joint0' in body 'j')",
    ),
    (
        "a flexcomp's vertex equality then a joint axis",
        r#"<mujoco><default></default><worldbody><body name="j" pos="5 0 0"><joint type="hinge" axis="0 0 0"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body><flexcomp name="c" type="grid" count="3 3 1" spacing="0.1 0.1 0.1" dim="2" mass="1" radius="0.005"><edge equality="vert"/><contact contype="0" conaffinity="0"/></flexcomp></worldbody><equality></equality></mujoco>"#,
        "axis too small in joint",
        "invalid axis on <joint>: axis too small (joint 'j_joint0' in body 'j')",
    ),
    (
        "a flex vertex equality then a connect's unknown body",
        r#"<mujoco><default></default><worldbody><body name="v0" pos="0 0 0"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body><body name="v1" pos="1 0 0"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body><body name="v2" pos="0 1 0"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body><body name="v3" pos="1 1 0"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body></worldbody><deformable><flex name="f" dim="2" body="v0 v1 v2 v3" element="0 1 2  1 3 2" ><contact contype="0" conaffinity="0"/></flex></deformable><equality><connect body1="nobody" body2="v1" anchor="0 0 0"/><flexvert flex="f"/></equality></mujoco>"#,
        "unknown element 'nobody' in equality constraint",
        "unknown body 'nobody' (equality #0)",
    ),
    (
        "a flexcomp's vertex equality then a flex equality's unknown flex",
        r#"<mujoco><default></default><worldbody><flexcomp name="c" type="grid" count="3 3 1" spacing="0.1 0.1 0.1" dim="2" mass="1" radius="0.005"><edge equality="vert"/><contact contype="0" conaffinity="0"/></flexcomp></worldbody><equality><flex flex="nope"/></equality></mujoco>"#,
        "unknown element 'nope' in equality constraint",
        "unknown flex 'nope' (equality #0)",
    ),
    (
        "a flexcomp's stretch beside its vertex equality",
        r#"<mujoco><default></default><worldbody><flexcomp name="c" type="grid" count="3 3 1" spacing="0.1 0.1 0.1" dim="2" mass="1"><edge equality="vert"/><elasticity elastic2d="stretch"/><contact contype="0" conaffinity="0"/></flexcomp></worldbody><equality></equality></mujoco>"#,
        "elasticity and edge constraints cannot both be present",
        "invalid elastic2d on <flexcomp>: elasticity and edge constraints cannot both be present (line 1: worldbody > flexcomp 'c' > contact)",
    ),
    (
        "a flexcomp's stretch beside its edge equality then its contact's keyword",
        r#"<mujoco><default></default><worldbody><flexcomp name="c" type="grid" count="3 3 1" spacing="0.1 0.1 0.1" dim="2" mass="1"><contact selfcollide="bogus"/><edge equality="true"/><elasticity elastic2d="stretch"/></flexcomp></worldbody><equality></equality></mujoco>"#,
        "elasticity and edge constraints cannot both be present",
        "invalid elastic2d on <flexcomp>: elasticity and edge constraints cannot both be present (line 1: worldbody > flexcomp 'c' > elasticity)",
    ),
    (
        "a flexcomp's stretch beside its edge equality then its count",
        r#"<mujoco><default></default><worldbody><flexcomp name="c" type="grid" count="0 3 1" spacing="0.1 0.1 0.1" dim="2" mass="1"><edge equality="true"/><elasticity elastic2d="stretch"/></flexcomp></worldbody><equality></equality></mujoco>"#,
        "elasticity and edge constraints cannot both be present",
        "invalid elastic2d on <flexcomp>: elasticity and edge constraints cannot both be present (line 1: worldbody > flexcomp 'c' > elasticity)",
    ),
    (
        "a flex equality's unknown flex then a tendon's negative armature",
        r#"<mujoco><worldbody><site name="s0" pos="0 0 2"/><site name="s1" pos="1 0 2"/><body name="v0" pos="0 0 0"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body><body name="v1" pos="1 0 0"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body><body name="v2" pos="0 1 0"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body><body name="v3" pos="1 1 0"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body></worldbody><deformable><flex name="f" dim="2" body="v0 v1 v2 v3" element="0 1 2  1 3 2"><contact contype="0" conaffinity="0"/></flex></deformable><equality><flex flex="nope"/></equality><tendon><spatial armature="-1"><site site="s0"/><site site="s1"/></spatial></tendon></mujoco>"#,
        "unknown element 'nope' in equality constraint",
        "unknown flex 'nope' (equality #0)",
    ),
    (
        "a bending flex without a thickness then a flex equality's unknown flex",
        r#"<mujoco><worldbody><body name="v0" pos="0 0 0"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body><body name="v1" pos="1 0 0"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body><body name="v2" pos="0 1 0"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body><body name="v3" pos="1 1 0"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body></worldbody><deformable><flex name="f" dim="2" body="v0 v1 v2 v3" element="0 1 2  1 3 2"><contact contype="0" conaffinity="0"/><elasticity elastic2d="bend" young="1e4"/></flex></deformable><equality><flex flex="nope"/></equality></mujoco>"#,
        "thickness must be positive for bending stiffness",
        "invalid thickness on <elasticity>: thickness must be positive for bending stiffness (flex 'f')",
    ),
    (
        "a flex vertex equality then a keyframe",
        r#"<mujoco><worldbody><body name="v0" pos="0 0 0"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body><body name="v1" pos="1 0 0"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body><body name="v2" pos="0 1 0"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body><body name="v3" pos="1 1 0"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body></worldbody><deformable><flex name="f" dim="2" body="v0 v1 v2 v3" element="0 1 2  1 3 2"><contact contype="0" conaffinity="0"/></flex></deformable><equality><flexvert flex="f"/></equality><keyframe><key qpos="0 0 0 0 0 0 0 0 0 0 0 0 0"/></keyframe></mujoco>"#,
        "keyframe '': invalid qpos size, expected 12, got 13",
        "invalid qpos on <key>: length 13 does not match 12 (key #0)",
    ),
    (
        "an empty flex name beside an unnamed flex then a keyframe",
        r#"<mujoco><worldbody><body name="v0" pos="0 0 0"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body><body name="v1" pos="1 0 0"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body><body name="v2" pos="0 1 0"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body><body name="v3" pos="1 1 0"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body></worldbody><deformable><flex dim="2" body="v0 v1 v2 v3" element="0 1 2  1 3 2"><contact contype="0" conaffinity="0"/></flex></deformable><equality><flex flex=""/></equality><keyframe><key qpos="0 0 0 0 0 0 0 0 0 0 0 0 0"/></keyframe></mujoco>"#,
        "keyframe '': invalid qpos size, expected 12, got 13",
        "invalid qpos on <key>: length 13 does not match 12 (key #0)",
    ),
    (
        "a flex's membrane then a keyframe",
        r#"<mujoco><worldbody><body name="v0" pos="0 0 0"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body><body name="v1" pos="1 0 0"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body><body name="v2" pos="0 1 0"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body><body name="v3" pos="1 1 0"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body></worldbody><deformable><flex name="f" dim="2" body="v0 v1 v2 v3" element="0 1 2  1 3 2"><contact contype="0" conaffinity="0"/><elasticity elastic2d="stretch" young="1e4" thickness="0.01"/></flex></deformable><keyframe><key qpos="0 0 0 0 0 0 0 0 0 0 0 0 0"/></keyframe></mujoco>"#,
        "keyframe '': invalid qpos size, expected 12, got 13",
        "invalid qpos on <key>: length 13 does not match 12 (key #0)",
    ),
    (
        "a dim-3 flex's Young's modulus then a keyframe",
        r#"<mujoco><worldbody><body name="v0" pos="0 0 0"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body><body name="v1" pos="1 0 0"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body><body name="v2" pos="0 1 0"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body><body name="v3" pos="1 1 0"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body><body name="v4" pos="0 0 1"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body></worldbody><deformable><flex name="f" dim="3" body="v0 v1 v2 v3 v4" element="0 1 2 4"><contact contype="0" conaffinity="0"/><elasticity young="1e4"/></flex></deformable><keyframe><key qpos="0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0"/></keyframe></mujoco>"#,
        "keyframe '': invalid qpos size, expected 15, got 16",
        "invalid qpos on <key>: length 16 does not match 15 (key #0)",
    ),
    (
        "an edge of three triangles in a bending flex then a keyframe",
        r#"<mujoco><worldbody><body name="v0" pos="0 0 0"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body><body name="v1" pos="1 0 0"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body><body name="v2" pos="0 1 0"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body><body name="v3" pos="1 1 0"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body><body name="v4" pos="0 0 1"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body></worldbody><deformable><flex name="f" dim="2" body="v0 v1 v2 v3 v4" element="0 1 2  1 0 3  0 1 4"><contact contype="0" conaffinity="0"/><elasticity elastic2d="bend"/></flex></deformable><keyframe><key qpos="0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0"/></keyframe></mujoco>"#,
        "keyframe '': invalid qpos size, expected 15, got 16",
        "invalid qpos on <key>: length 16 does not match 15 (key #0)",
    ),
    (
        "a flexcomp's count then a keyframe's ctrl",
        r#"<mujoco><worldbody><flexcomp name="f" type="grid" count="0 1 1" spacing="0.1 0.1 0.1" dim="1"/></worldbody><keyframe><key ctrl="1 2 3"/></keyframe></mujoco>"#,
        "Count too small",
        "invalid count on <flexcomp>: Count too small (flex 'f')",
    ),
    (
        "a flexcomp's pin id then a joint's range",
        r#"<mujoco><worldbody><flexcomp name="f" type="grid" count="2 1 1" spacing="0.1 0.1 0.1" dim="1"><pin id="9"/></flexcomp><body name="jb" pos="2 0 0"><joint type="hinge" range="1 0" limited="true"/><geom size="0.1"/></body></worldbody></mujoco>"#,
        "pinid out of range",
        "invalid pin on <flexcomp>: pinid out of range (flex 'f')",
    ),
    (
        "a radial flexcomp, which MuJoCo loads, then a keyframe's ctrl",
        r#"<mujoco><worldbody><flexcomp name="f" type="grid" count="2 1 1" spacing="0.1 0.1 0.1" dim="1" dof="radial"/></worldbody><keyframe><key ctrl="1 2 3"/></keyframe></mujoco>"#,
        "keyframe 0: invalid ctrl size, expected length 0",
        "invalid ctrl on <key>: length 3 does not match 0 (key #0)",
    ),
    (
        "a flexcomp in a turned body, which MuJoCo loads, then a massless moving body",
        r#"<mujoco><worldbody><body name="p" euler="0 0 90"><flexcomp name="f" type="grid" count="2 1 1" spacing="0.1 0.1 0.1" dim="1"/></body><body name="mb" pos="3 0 0"><joint type="hinge"/></body></worldbody></mujoco>"#,
        "mass and inertia of moving bodies must be larger than mjMINVAL",
        "mass and inertia of moving bodies must be larger than mjMINVAL (body 'mb')",
    ),
    (
        "a flexcomp's dim then its count",
        r#"<mujoco><worldbody><flexcomp name="f" type="grid" count="0 1 1" spacing="0.1 0.1 0.1" dim="4"/></worldbody></mujoco>"#,
        "Invalid dim, must be between 1 and 3",
        "invalid dim on <flexcomp>: Invalid dim, must be between 1 and 3 (flex 'f')",
    ),
    (
        "a flexcomp's count then its spacing",
        r#"<mujoco><worldbody><flexcomp name="f" type="grid" count="0 1 1" spacing="0.001 0.1 0.1" dim="1"/></worldbody></mujoco>"#,
        "Count too small",
        "invalid count on <flexcomp>: Count too small (flex 'f')",
    ),
    (
        "a flexcomp's spacing then its scale",
        r#"<mujoco><worldbody><flexcomp name="f" type="grid" count="2 1 1" spacing="0.001 0.1 0.1" dim="1" scale="0 1 1"/></worldbody></mujoco>"#,
        "Spacing must be larger than geometry size",
        "invalid spacing on <flexcomp>: Spacing must be larger than geometry size (flex 'f')",
    ),
    (
        "a flexcomp's scale then its mass",
        r#"<mujoco><worldbody><flexcomp name="f" type="grid" count="2 1 1" spacing="0.1 0.1 0.1" dim="1" scale="0 1 1" mass="0"/></worldbody></mujoco>"#,
        "Scale must be larger than mjMINVAL",
        "invalid scale on <flexcomp>: Scale must be larger than mjMINVAL (flex 'f')",
    ),
    (
        "a flexcomp's mass then its orientation",
        r#"<mujoco><worldbody><flexcomp name="f" type="grid" count="2 1 1" spacing="0.1 0.1 0.1" dim="1" mass="0" axisangle="0 0 0 90"/></worldbody></mujoco>"#,
        "Mass and inertiabox must be larger than mjMINVAL",
        "invalid mass on <flexcomp>: Mass and inertiabox must be larger than mjMINVAL (flex 'f')",
    ),
    (
        "a flexcomp's orientation then its maker's",
        r#"<mujoco><worldbody><flexcomp name="f" type="grid" count="1 1 1" spacing="0.1 0.1 0.1" dim="1" axisangle="0 0 0 90"/></worldbody></mujoco>"#,
        "axisangle too small",
        "invalid axisangle on <flexcomp>: axisangle too small (flex 'f')",
    ),
    (
        "a flexcomp's pin range size then its missing points",
        r#"<mujoco><worldbody><flexcomp name="f" type="direct" dim="1"><pin range="0"/></flexcomp></worldbody></mujoco>"#,
        "Pin range number must be multiple of 2",
        "invalid pin on <flexcomp>: Pin range number must be multiple of 2 (flex 'f')",
    ),
    (
        "a flexcomp's element id then its pin id",
        r#"<mujoco><worldbody><flexcomp name="f" type="direct" dim="1" point="0 0 0 1 0 0" element="0 5"><pin id="9"/></flexcomp></worldbody></mujoco>"#,
        "element 1 has point id 5, number of points is 2",
        "invalid element on <flexcomp>: element 1 has point id 5, number of points is 2 (flex 'f')",
    ),
    (
        "a deformable flexcomp after a joint's range",
        r#"<mujoco><worldbody><body name="a"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body><body name="b" pos="1 0 0"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body><body name="r"><joint type="hinge" range="2 1" limited="true"/><geom size="0.1"/></body></worldbody><deformable><flexcomp name="c" type="grid" count="2 2 1" spacing="0.1 0.1 0.1" dim="2"/></deformable></mujoco>"#,
        "Schema violation: unrecognized element",
        "unrecognized element <flexcomp> in <deformable> (line 1: deformable > flexcomp 'c')",
    ),
    (
        "a deformable flexcomp after a repeated name",
        r#"<mujoco><worldbody><body name="a"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body><body name="b" pos="1 0 0"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body><body name="dup"/><body name="dup"/></worldbody><deformable><flexcomp name="c" type="grid" count="2 2 1" spacing="0.1 0.1 0.1" dim="2"/></deformable></mujoco>"#,
        "Schema violation: unrecognized element",
        "unrecognized element <flexcomp> in <deformable> (line 1: deformable > flexcomp 'c')",
    ),
    (
        "a flex's density after an unknown class",
        r#"<mujoco><worldbody><body name="a"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body><body name="b" pos="1 0 0"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body><geom class="nope" size="0.1"/></worldbody><deformable><flex name="f" dim="1" body="a b" element="0 1" density="10"/></deformable></mujoco>"#,
        "Schema violation: unrecognized attribute: 'density'",
        "unrecognized attribute 'density' on <flex> (line 1: deformable > flex 'f')",
    ),
    (
        "a flex's quat before a keyframe's size",
        r#"<mujoco><worldbody><body name="a"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body><body name="b" pos="1 0 0"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body></worldbody><deformable><flex name="f" dim="1" body="a b" element="0 1" quat="1 0 0 0"/></deformable><keyframe><key qpos="1 2 3 4 5 6 7 8 9"/></keyframe></mujoco>"#,
        "Schema violation: unrecognized attribute: 'quat'",
        "unrecognized attribute 'quat' on <flex> (line 1: deformable > flex 'f')",
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

/// Two faults, one in a frame or a composite, where this crate names the
/// other: ledger L353 holds each. `(what, document, MuJoCo 3.5.0's reason,
/// this crate's error)`, measured 2026-11-03 (the documents of review round
/// 1, RA's and RW's, and of its fix-diff pass).
const FRAME_AND_COMPOSITE_COSTS: &[(&str, &str, &str, &str)] = &[
    (
        "parent geom zaxis then body in frame zaxis",
        r#"<mujoco><worldbody><body name="b"><freejoint/><geom name="g" size=".1" zaxis="0 0 0"/><frame><body name="c" zaxis="0 0 0"><joint/><geom size=".1"/></body></frame></body></worldbody></mujoco>"#,
        "orientation specification error 'zaxis too small' in geom 0",
        "invalid zaxis on <body>: zaxis too small (body 'c')",
    ),
    (
        "body zaxis and geom zaxis in frame",
        r#"<mujoco><worldbody><body name="b" zaxis="0 0 0"><freejoint/><geom size=".1"/><frame><geom name="g" size=".1" zaxis="0 0 0"/></frame></body></worldbody></mujoco>"#,
        "error 'zaxis too small' in frame alternative",
        "invalid zaxis on <geom>: zaxis too small (geom 'g' in body 'b')",
    ),
    (
        "body zaxis then later frame geom zaxis",
        r#"<mujoco><worldbody><body name="b" zaxis="0 0 0"><freejoint/><geom size=".1"/></body><body name="a"><freejoint/><geom size=".1"/><frame><geom name="g" size=".1" zaxis="0 0 0"/></frame></body></worldbody></mujoco>"#,
        "error 'zaxis too small' in frame alternative",
        "invalid zaxis on <geom>: zaxis too small (geom 'g' in body 'a')",
    ),
    (
        "frame site then geom zaxis body level",
        r#"<mujoco><worldbody><body name="b"><freejoint/><geom size=".1"/><frame><site name="s" zaxis="0 0 0"/></frame><geom name="g" size=".1" zaxis="0 0 0"/></body></worldbody></mujoco>"#,
        "orientation specification error 'zaxis too small' in geom 1",
        "invalid zaxis on <site>: zaxis too small (site 's' in body 'b')",
    ),
    (
        "geom size0 then zeroseg cable in frame",
        r#"<mujoco><worldbody><body name="a"><freejoint/><geom name="z" size="0"/></body><frame pos="0 0 1"><composite type="cable" prefix="v" vertex="0 0 0  0.3 0 0  0.3 0 0  0.9 0 0"><joint kind="main" damping="0.1"/><geom type="capsule" size=".01" mass="0.1"/></composite></frame></worldbody></mujoco>"#,
        "size 0 must be positive in geom",
        "invalid fromto on <geom>: fromto points too close (geom 'vG1' in body 'vB_1')",
    ),
    (
        "geom size0 then zeroseg cable no frame",
        r#"<mujoco><worldbody><body name="a"><freejoint/><geom name="z" size="0"/></body><composite type="cable" prefix="v" vertex="0 0 0  0.3 0 0  0.3 0 0  0.9 0 0"><joint kind="main" damping="0.1"/><geom type="capsule" size=".01" mass="0.1"/></composite></worldbody></mujoco>"#,
        "size 0 must be positive in geom",
        "invalid fromto on <geom>: fromto points too close (geom 'vG1' in body 'vB_1')",
    ),
    (
        "body zaxis then zeroseg cable in frame",
        r#"<mujoco><worldbody><body name="a" zaxis="0 0 0"><freejoint/><geom size=".1"/></body><frame pos="0 0 1"><composite type="cable" prefix="v" vertex="0 0 0  0.3 0 0  0.3 0 0  0.9 0 0"><joint kind="main" damping="0.1"/><geom type="capsule" size=".01" mass="0.1"/></composite></frame></worldbody></mujoco>"#,
        "error 'zaxis too small' in frame alternative",
        "invalid fromto on <geom>: fromto points too close (geom 'vG1' in body 'vB_1')",
    ),
    (
        "repeated frame then bad comp in frame",
        r#"<mujoco><worldbody><frame name="f"/><frame name="f"><composite type="grid" count="4 4 1" prefix="c"><geom type="sphere" size=".01"/></composite></frame></worldbody></mujoco>"#,
        "repeated name 'f' in frame",
        r#"invalid type on <composite>: The "grid" composite type is deprecated. Please use "flex" instead. (body 'world')"#,
    ),
    (
        "comp in frame then bad childclass",
        r#"<mujoco><default><default class="k"><geom size=".2"/></default></default><worldbody><body name="b"><freejoint/><geom size=".1"/><frame><composite type="grid" count="4 4 1" spacing=".1" prefix="c"><geom type="sphere" size=".01"/></composite></frame><frame childclass="nope"><geom size=".1"/></frame></body></worldbody></mujoco>"#,
        r#"The "grid" composite type is deprecated. Please use "flex" instead."#,
        "unknown default class name 'nope' (frame in body 'b')",
    ),
    (
        "comp in frame then fromto close",
        r#"<mujoco><worldbody><body name="b"><freejoint/><geom size=".1"/><frame><composite type="grid" count="4 4 1" spacing=".1" prefix="c"><geom type="sphere" size=".01"/></composite></frame><frame><geom type="capsule" size=".1" fromto="0 0 0 0 0 0"/></frame></body></worldbody></mujoco>"#,
        r#"The "grid" composite type is deprecated. Please use "flex" instead."#,
        "invalid fromto on <geom>: fromto points too close (geom in body 'b')",
    ),
    (
        "comp deep in frame then parse fault",
        r#"<mujoco><worldbody><body name="b"><freejoint/><geom size=".1"/><frame><composite type="grid" count="4 4 1" spacing=".1" prefix="c"><geom type="sphere" size=".01"/></composite></frame><geom name="x" type="nosuchtype" size=".1"/></body></worldbody></mujoco>"#,
        r#"The "grid" composite type is deprecated. Please use "flex" instead."#,
        r#"invalid keyword "nosuchtype" in attribute 'type' on <geom> (line 1: worldbody > body 'b' > geom 'x')"#,
    ),
    (
        "frame zaxis then fromto close",
        r#"<mujoco><worldbody><body name="b"><freejoint/><geom size=".1"/><frame name="f" zaxis="0 0 0"><geom size=".1"/></frame><geom type="capsule" size=".1" fromto="0 0 0 0 0 0"/></body></worldbody></mujoco>"#,
        "orientation specification error 'zaxis too small' in site -1",
        "invalid fromto on <geom>: fromto points too close (geom in body 'b')",
    ),
    (
        "frame zaxis then repeated body",
        r#"<mujoco><worldbody><body name="b"><freejoint/><geom size=".1"/><frame name="f" zaxis="0 0 0"><geom size=".1"/></frame></body><body name="b"><freejoint/><geom size=".1"/></body></worldbody></mujoco>"#,
        "repeated name 'b' in body",
        "invalid zaxis on <frame>: zaxis too small (frame 'f' in body 'b')",
    ),
];

#[test]
fn a_fault_in_a_frame_or_a_composite_beside_another_is_named_as_ledgered() {
    let mut moved = Vec::new();
    for &(what, xml, _mujoco, ours) in FRAME_AND_COMPOSITE_COSTS {
        match load_model(xml) {
            Err(e) if e.to_string() == ours => {}
            Err(e) => moved.push(format!("{what}: {e}")),
            Ok(_) => moved.push(format!("{what}: loads")),
        }
    }
    assert!(moved.is_empty(), "{}", moved.join("\n"));
}

/// A `<size>` `nuser_` below -1 written after a fault this crate's parser
/// meets first: MuJoCo 3.5.0 reads the whole document, then `<compiler>`,
/// `<option>` and `<size>` before the other sections; this crate's parser
/// stops at the earlier fault (ledger L353).
/// `(what, document, MuJoCo's reason, this crate's error)`, measured
/// 2026-11-04.
const NUSER_COSTS: &[(&str, &str, &str, &str)] = &[(
    "a parse fault then a nuser below -1",
    r#"<mujoco><worldbody><body name="b"><freejoint/><geom name="g" type="nosuch" size=".1"/></body></worldbody><size nuser_geom="-2"/></mujoco>"#,
    "nuser_geom must be >= -1",
    "invalid keyword \"nosuch\" in attribute 'type' on <geom> (line 1: worldbody > body 'b' > geom 'g')",
)];

#[test]
fn a_nuser_after_a_parse_fault_is_named_as_ledgered() {
    let mut moved = Vec::new();
    for &(what, xml, _mujoco, ours) in NUSER_COSTS {
        match load_model(xml) {
            Err(e) if e.to_string() == ours => {}
            Err(e) => moved.push(format!("{what}: {e}")),
            Ok(_) => moved.push(format!("{what}: loads")),
        }
    }
    assert!(moved.is_empty(), "{}", moved.join("\n"));
}

/// A model built in code, its `<size>` not parsed, has its `nuser_` sizes
/// checked before anything else too: before a repeated name here.
#[test]
fn a_nuser_below_minus_1_set_in_code_is_refused_first() {
    let mut mjcf = sim_mjcf::parse_mjcf_str(
        r#"<mujoco><worldbody><body name="a"><freejoint/><geom name="g" size=".1"/><geom name="g" size=".1"/></body></worldbody></mujoco>"#,
    )
    .expect("parses");
    mjcf.nuser_body = -2;
    let error = sim_mjcf::model_from_mjcf(&mjcf, None).expect_err("nuser_body -2");
    assert_eq!(
        error.to_string(),
        "invalid nuser_body on <size>: must be >= -1, got -2"
    );
}

/// Two faults, one a geom's or a pair's condim or a pair's geom, where this
/// crate names the other: a geom's condim is checked with the checks RL-6a
/// added, after every check this crate makes before building the model, a
/// pair's after the assets; a sphere's `fromto` is refused when the classes
/// are applied, a height field written without `elevation` while parsing
/// (ledger L359), a body's orientation when the tree is built, and a pair
/// with no `geom1` while parsing (registry `D-PAIR-NO-GEOM`); MuJoCo 3.5.0
/// checks each condim and pair before these (ledger L353). `(what, document, MuJoCo's reason, this crate's
/// error)`, measured 2026-11-04 (the last five: review round 1's).
const CONDIM_COSTS: &[(&str, &str, &str, &str)] = &[
    (
        "condim then its size",
        r#"<mujoco><worldbody><body name="b"><freejoint/><geom size=".1"/><geom name="g" size="0" condim="2"/></body></worldbody></mujoco>"#,
        "invalid condim in geom",
        "invalid size on <geom>: size 0 must be positive (geom 'g' in body 'b')",
    ),
    (
        "condim then a later body's size",
        r#"<mujoco><worldbody><body name="b"><freejoint/><geom name="g" size=".1" condim="2"/></body><body name="c"><freejoint/><geom name="z" size="0"/></body></worldbody></mujoco>"#,
        "invalid condim in geom",
        "invalid size on <geom>: size 0 must be positive (geom 'z' in body 'c')",
    ),
    (
        "condim then its fromto",
        r#"<mujoco><worldbody><body name="b"><freejoint/><geom size=".1"/><geom name="g" size=".1" fromto="0 0 0 0 0 1" condim="2"/></body></worldbody></mujoco>"#,
        "invalid condim in geom",
        "invalid fromto on <geom>: fromto requires a capsule, cylinder, ellipsoid or box (geom 'g' in body 'b')",
    ),
    (
        "condim then a later frame's xyaxes",
        r#"<mujoco><worldbody><body name="b"><freejoint/><geom name="g" size=".1" condim="2"/></body><body name="c"><freejoint/><geom size=".1"/><frame xyaxes="1 0 0 1 0 0"><geom size=".1"/></frame></body></worldbody></mujoco>"#,
        "invalid condim in geom",
        "invalid xyaxes on <frame>: yaxis too small (frame in body 'c')",
    ),
    (
        "condim then a tendon's joint",
        r#"<mujoco><worldbody><body name="b"><freejoint/><geom name="g" size=".1" condim="2"/></body></worldbody><tendon><fixed name="t"><joint joint="nope" coef="1"/></fixed></tendon></mujoco>"#,
        "invalid condim in geom",
        "unknown joint 'nope' (tendon 't')",
    ),
    (
        "condim then an actuator's joint",
        r#"<mujoco><worldbody><body name="b"><freejoint/><geom name="g" size=".1" condim="2"/></body></worldbody><actuator><motor joint="nope"/></actuator></mujoco>"#,
        "invalid condim in geom",
        "unknown joint 'nope' (actuator #0)",
    ),
    (
        "condim then a sensor's joint",
        r#"<mujoco><worldbody><body name="b"><freejoint/><geom name="g" size=".1" condim="2"/></body></worldbody><sensor><jointpos joint="nope"/></sensor></mujoco>"#,
        "invalid condim in geom",
        "unknown joint 'nope' (sensor #0)",
    ),
    (
        "condim then an equality's body",
        r#"<mujoco><worldbody><body name="b"><freejoint/><geom name="g" size=".1" condim="2"/></body></worldbody><equality><connect body1="nope" body2="b" anchor="0 0 0"/></equality></mujoco>"#,
        "invalid condim in geom",
        "unknown body 'nope' (equality #0)",
    ),
    (
        "condim then an exclude's body",
        r#"<mujoco><worldbody><body name="b"><freejoint/><geom name="g" size=".1" condim="2"/></body></worldbody><contact><exclude body1="nope" body2="b"/></contact></mujoco>"#,
        "invalid condim in geom",
        "unknown body 'nope' (exclude #0)",
    ),
    (
        "the world's condim then a body's size",
        r#"<mujoco><worldbody><geom name="w" size=".1" condim="2"/><body name="b"><freejoint/><geom name="z" size="0"/></body></worldbody></mujoco>"#,
        "invalid condim in geom",
        "invalid size on <geom>: size 0 must be positive (geom 'z' in body 'b')",
    ),
    (
        "the world's condim then a body's user data",
        r#"<mujoco><size nuser_body="0"/><worldbody><geom name="w" size=".1" condim="2"/><body name="b" user="1"><freejoint/><geom size=".1"/></body></worldbody></mujoco>"#,
        "invalid condim in geom",
        "invalid user on <body>: user data exceeds nuser_body=0",
    ),
    (
        "a pair's condim then a frame's xyaxes",
        r#"<mujoco><worldbody><geom name="floor" type="plane" size="1 1 .1"/><body name="b"><freejoint/><geom name="g" size=".1"/><frame xyaxes="1 0 0 1 0 0"><geom size=".1"/></frame></body></worldbody><contact><pair geom1="g" geom2="floor" condim="2"/></contact></mujoco>"#,
        "invalid condim in contact pair",
        "invalid xyaxes on <frame>: yaxis too small (frame in body 'b')",
    ),
    (
        "a pair's condim then a fromto",
        r#"<mujoco><worldbody><geom name="floor" type="plane" size="1 1 .1"/><body name="b"><freejoint/><geom name="g" size=".1"/><geom size=".1" fromto="0 0 0 0 0 1"/></body></worldbody><contact><pair geom1="g" geom2="floor" condim="2"/></contact></mujoco>"#,
        "invalid condim in contact pair",
        "invalid fromto on <geom>: fromto requires a capsule, cylinder, ellipsoid or box (geom in body 'b')",
    ),
    (
        "a geom's condim then a sphere's fromto",
        r#"<mujoco><worldbody><geom name="floor" type="plane" size="1 1 0.1"/><body name="a" pos="0 0 0.09" ><freejoint/><geom name="ga" type="sphere" size="0.1" condim="2"/></body><body name="b" pos="0.5 0 0.09"><freejoint/><geom name="gb" type="sphere" size="0.1" fromto="0 0 0 0 0 1"/></body></worldbody><contact><pair geom1="ga" geom2="floor"/><pair geom1="gb" geom2="floor"/></contact></mujoco>"#,
        "invalid condim in geom",
        "invalid fromto on <geom>: fromto requires a capsule, cylinder, ellipsoid or box (geom 'gb' in body 'b')",
    ),
    (
        "a pair's condim then a sphere's fromto",
        r#"<mujoco><worldbody><geom name="floor" type="plane" size="1 1 0.1"/><body name="a" pos="0 0 0.09" ><freejoint/><geom name="ga" type="sphere" size="0.1"/></body><body name="b" pos="0.5 0 0.09"><freejoint/><geom name="gb" type="sphere" size="0.1" fromto="0 0 0 0 0 1"/></body></worldbody><contact><pair geom1="ga" geom2="floor" condim="2"/><pair geom1="gb" geom2="floor"/></contact></mujoco>"#,
        "invalid condim in contact pair",
        "invalid fromto on <geom>: fromto requires a capsule, cylinder, ellipsoid or box (geom 'gb' in body 'b')",
    ),
    (
        "a pair's unknown geom then a sphere's fromto",
        r#"<mujoco><worldbody><geom name="floor" type="plane" size="1 1 .1"/><body name="a" pos="0 0 .09"><freejoint/><geom name="ga" size=".1"/></body><body name="h" pos="1 0 1"><freejoint/><geom type="sphere" size=".1" fromto="0 0 0 0 0 1"/></body></worldbody><contact><pair geom1="zz" geom2="floor"/></contact></mujoco>"#,
        "geom 'zz' not found in collision 0",
        "invalid fromto on <geom>: fromto requires a capsule, cylinder, ellipsoid or box (geom in body 'h')",
    ),
    (
        "a geom's condim then a height field without elevation",
        r#"<mujoco><asset><hfield name="h" nrow="2" ncol="2" size="1 1 0 0.1"/></asset><worldbody><geom name="floor" type="plane" size="1 1 0.1"/><geom type="hfield" hfield="h" pos="3 0 0"/><body name="a" pos="0 0 0.09" ><freejoint/><geom name="ga" type="sphere" size="0.1" condim="2"/></body><body name="b" pos="0.5 0 0.09"><freejoint/><geom name="gb" type="box" size="0.1 0.1 0.1"/></body></worldbody><contact><pair geom1="ga" geom2="floor"/><pair geom1="gb" geom2="floor"/></contact></mujoco>"#,
        "invalid condim in geom",
        "invalid elevation on <hfield>: at least one of 'file' or 'elevation' must be specified (line 1: asset > hfield 'h')",
    ),
    (
        "a pair's condim then a height field without elevation",
        r#"<mujoco><asset><hfield name="h" nrow="2" ncol="2" size="1 1 0 0.1"/></asset><worldbody><geom name="floor" type="plane" size="1 1 0.1"/><geom type="hfield" hfield="h" pos="3 0 0"/><body name="a" pos="0 0 0.09" ><freejoint/><geom name="ga" type="sphere" size="0.1"/></body><body name="b" pos="0.5 0 0.09"><freejoint/><geom name="gb" type="box" size="0.1 0.1 0.1"/></body></worldbody><contact><pair geom1="ga" geom2="floor" condim="2"/><pair geom1="gb" geom2="floor"/></contact></mujoco>"#,
        "invalid condim in contact pair",
        "invalid elevation on <hfield>: at least one of 'file' or 'elevation' must be specified (line 1: asset > hfield 'h')",
    ),
    (
        "a geom's condim then a later moving body's zaxis",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><freejoint/><geom name="ga" size=".1" condim="2"/></body><body name="s" pos="2 0 0" zaxis="0 0 0"><joint type="hinge"/><geom name="gs" size=".1"/></body></worldbody></mujoco>"#,
        "invalid condim in geom",
        "invalid zaxis on <body>: zaxis too small (body 's')",
    ),
    (
        "a geom's condim then a later fused body's zaxis",
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="a" pos="0 0 1"><freejoint/><geom name="ga" size=".1" condim="2"/></body><body name="s" pos="2 0 0" zaxis="0 0 0"><geom name="gs" size=".1"/></body></worldbody></mujoco>"#,
        "invalid condim in geom",
        "invalid zaxis on <body>: zaxis too small (body 's')",
    ),
    (
        "a pair with no geom1, bound by MuJoCo, then a size",
        r#"<mujoco><worldbody><geom name="floor" type="plane" size="1 1 .1"/><body name="b"><freejoint/><geom name="g" size=".1"/><geom size=".2"/></body><body name="c" pos="1 0 1"><freejoint/><geom name="z" size="0"/></body></worldbody><contact><pair geom2="g"/></contact></mujoco>"#,
        "size 0 must be positive in geom",
        "required attribute missing: 'geom1' on <pair> (line 1: contact > pair)",
    ),
];

#[test]
fn a_condim_beside_another_fault_is_named_as_ledgered() {
    let mut moved = Vec::new();
    for &(what, xml, _mujoco, ours) in CONDIM_COSTS {
        match load_model(xml) {
            Err(e) if e.to_string() == ours => {}
            Err(e) => moved.push(format!("{what}: {e}")),
            Ok(_) => moved.push(format!("{what}: loads")),
        }
    }
    assert!(moved.is_empty(), "{}", moved.join("\n"));
}

/// Two faults, one a sensor's noise, cutoff, quaternion cutoff or height
/// field, where this crate names the other: ledger L353 holds each. `(what,
/// document, MuJoCo 3.5.0's reason, this crate's error)`, measured
/// 2026-11-05.
const SENSOR_COSTS: &[(&str, &str, &str, &str)] = &[
    (
        "negative cutoff beside its own negative interval",
        r#"<mujoco model="p_C_interval_same"><asset><hfield name="hf" nrow="3" ncol="3" size="1 1 0.1 0.1" elevation="0 0 0 0 0 0 0 0 0"/></asset><worldbody><geom name="floor" type="plane" size="1 1 0.1"/><geom name="hfg" type="hfield" hfield="hf" pos="5 0 0"/><body name="b" pos="0 0 0.5" ><joint name="j" type="hinge" axis="0 1 0" /><geom name="s" type="sphere" size="0.1" mass="1" /><site name="st" size="0.01"/></body><body name="c" pos="0.5 0 0.5"><geom name="s2" type="sphere" size="0.1" /></body></worldbody><contact></contact><equality></equality><tendon></tendon><actuator></actuator><sensor><jointpos joint="j" cutoff="-0.1" interval="-1"/></sensor><keyframe></keyframe></mujoco>"#,
        "negative cutoff in sensor",
        "invalid interval on <sensor>: negative interval in sensor (sensor #0)",
    ),
    (
        "negative cutoff beside a later sensor's unknown object",
        r#"<mujoco model="p_C_obj_after"><asset><hfield name="hf" nrow="3" ncol="3" size="1 1 0.1 0.1" elevation="0 0 0 0 0 0 0 0 0"/></asset><worldbody><geom name="floor" type="plane" size="1 1 0.1"/><geom name="hfg" type="hfield" hfield="hf" pos="5 0 0"/><body name="b" pos="0 0 0.5" ><joint name="j" type="hinge" axis="0 1 0" /><geom name="s" type="sphere" size="0.1" mass="1" /><site name="st" size="0.01"/></body><body name="c" pos="0.5 0 0.5"><geom name="s2" type="sphere" size="0.1" /></body></worldbody><contact></contact><equality></equality><tendon></tendon><actuator></actuator><sensor><jointpos joint="j" cutoff="-0.1"/><jointvel joint="nope"/></sensor><keyframe></keyframe></mujoco>"#,
        "negative cutoff in sensor",
        "unknown joint 'nope' (sensor #1)",
    ),
    (
        "negative cutoff beside its own unknown object",
        r#"<mujoco model="p_C_obj_same"><asset><hfield name="hf" nrow="3" ncol="3" size="1 1 0.1 0.1" elevation="0 0 0 0 0 0 0 0 0"/></asset><worldbody><geom name="floor" type="plane" size="1 1 0.1"/><geom name="hfg" type="hfield" hfield="hf" pos="5 0 0"/><body name="b" pos="0 0 0.5" ><joint name="j" type="hinge" axis="0 1 0" /><geom name="s" type="sphere" size="0.1" mass="1" /><site name="st" size="0.01"/></body><body name="c" pos="0.5 0 0.5"><geom name="s2" type="sphere" size="0.1" /></body></worldbody><contact></contact><equality></equality><tendon></tendon><actuator></actuator><sensor><jointpos joint="nope" cutoff="-0.1"/></sensor><keyframe></keyframe></mujoco>"#,
        "negative cutoff in sensor",
        "unknown joint 'nope' (sensor #0)",
    ),
    (
        "negative cutoff beside a later sensor's user data",
        r#"<mujoco model="p_C_suser_after"><size nuser_sensor="1"/><asset><hfield name="hf" nrow="3" ncol="3" size="1 1 0.1 0.1" elevation="0 0 0 0 0 0 0 0 0"/></asset><worldbody><geom name="floor" type="plane" size="1 1 0.1"/><geom name="hfg" type="hfield" hfield="hf" pos="5 0 0"/><body name="b" pos="0 0 0.5" ><joint name="j" type="hinge" axis="0 1 0" /><geom name="s" type="sphere" size="0.1" mass="1" /><site name="st" size="0.01"/></body><body name="c" pos="0.5 0 0.5"><geom name="s2" type="sphere" size="0.1" /></body></worldbody><contact></contact><equality></equality><tendon></tendon><actuator></actuator><sensor><jointpos joint="j" cutoff="-0.1"/><jointvel joint="j" user="1 2"/></sensor><keyframe></keyframe></mujoco>"#,
        "negative cutoff in sensor",
        "invalid user on <sensor>: user data exceeds nuser_sensor=1",
    ),
    (
        "height-field distance beside a later sensor's unknown object",
        r#"<mujoco model="p_H_obj_after"><asset><hfield name="hf" nrow="3" ncol="3" size="1 1 0.1 0.1" elevation="0 0 0 0 0 0 0 0 0"/></asset><worldbody><geom name="floor" type="plane" size="1 1 0.1"/><geom name="hfg" type="hfield" hfield="hf" pos="5 0 0"/><body name="b" pos="0 0 0.5" ><joint name="j" type="hinge" axis="0 1 0" /><geom name="s" type="sphere" size="0.1" mass="1" /><site name="st" size="0.01"/></body><body name="c" pos="0.5 0 0.5"><geom name="s2" type="sphere" size="0.1" /></body></worldbody><contact></contact><equality></equality><tendon></tendon><actuator></actuator><sensor><distance geom1="hfg" geom2="s"/><jointvel joint="nope"/></sensor><keyframe></keyframe></mujoco>"#,
        "height fields are not supported in geom distance sensors",
        "unknown joint 'nope' (sensor #1)",
    ),
    (
        "height-field distance beside a later sensor's user data",
        r#"<mujoco model="p_H_suser_after"><size nuser_sensor="1"/><asset><hfield name="hf" nrow="3" ncol="3" size="1 1 0.1 0.1" elevation="0 0 0 0 0 0 0 0 0"/></asset><worldbody><geom name="floor" type="plane" size="1 1 0.1"/><geom name="hfg" type="hfield" hfield="hf" pos="5 0 0"/><body name="b" pos="0 0 0.5" ><joint name="j" type="hinge" axis="0 1 0" /><geom name="s" type="sphere" size="0.1" mass="1" /><site name="st" size="0.01"/></body><body name="c" pos="0.5 0 0.5"><geom name="s2" type="sphere" size="0.1" /></body></worldbody><contact></contact><equality></equality><tendon></tendon><actuator></actuator><sensor><distance geom1="hfg" geom2="s"/><jointvel joint="j" user="1 2"/></sensor><keyframe></keyframe></mujoco>"#,
        "height fields are not supported in geom distance sensors",
        "invalid user on <sensor>: user data exceeds nuser_sensor=1",
    ),
    (
        "negative noise beside its own negative interval",
        r#"<mujoco model="p_N_interval_same"><asset><hfield name="hf" nrow="3" ncol="3" size="1 1 0.1 0.1" elevation="0 0 0 0 0 0 0 0 0"/></asset><worldbody><geom name="floor" type="plane" size="1 1 0.1"/><geom name="hfg" type="hfield" hfield="hf" pos="5 0 0"/><body name="b" pos="0 0 0.5" ><joint name="j" type="hinge" axis="0 1 0" /><geom name="s" type="sphere" size="0.1" mass="1" /><site name="st" size="0.01"/></body><body name="c" pos="0.5 0 0.5"><geom name="s2" type="sphere" size="0.1" /></body></worldbody><contact></contact><equality></equality><tendon></tendon><actuator></actuator><sensor><jointpos joint="j" noise="-0.1" interval="-1"/></sensor><keyframe></keyframe></mujoco>"#,
        "negative noise in sensor",
        "invalid interval on <sensor>: negative interval in sensor (sensor #0)",
    ),
    (
        "negative noise beside a later sensor's unknown object",
        r#"<mujoco model="p_N_obj_after"><asset><hfield name="hf" nrow="3" ncol="3" size="1 1 0.1 0.1" elevation="0 0 0 0 0 0 0 0 0"/></asset><worldbody><geom name="floor" type="plane" size="1 1 0.1"/><geom name="hfg" type="hfield" hfield="hf" pos="5 0 0"/><body name="b" pos="0 0 0.5" ><joint name="j" type="hinge" axis="0 1 0" /><geom name="s" type="sphere" size="0.1" mass="1" /><site name="st" size="0.01"/></body><body name="c" pos="0.5 0 0.5"><geom name="s2" type="sphere" size="0.1" /></body></worldbody><contact></contact><equality></equality><tendon></tendon><actuator></actuator><sensor><jointpos joint="j" noise="-0.1"/><jointvel joint="nope"/></sensor><keyframe></keyframe></mujoco>"#,
        "negative noise in sensor",
        "unknown joint 'nope' (sensor #1)",
    ),
    (
        "negative noise beside its own unknown object",
        r#"<mujoco model="p_N_obj_same"><asset><hfield name="hf" nrow="3" ncol="3" size="1 1 0.1 0.1" elevation="0 0 0 0 0 0 0 0 0"/></asset><worldbody><geom name="floor" type="plane" size="1 1 0.1"/><geom name="hfg" type="hfield" hfield="hf" pos="5 0 0"/><body name="b" pos="0 0 0.5" ><joint name="j" type="hinge" axis="0 1 0" /><geom name="s" type="sphere" size="0.1" mass="1" /><site name="st" size="0.01"/></body><body name="c" pos="0.5 0 0.5"><geom name="s2" type="sphere" size="0.1" /></body></worldbody><contact></contact><equality></equality><tendon></tendon><actuator></actuator><sensor><jointpos joint="nope" noise="-0.1"/></sensor><keyframe></keyframe></mujoco>"#,
        "negative noise in sensor",
        "unknown joint 'nope' (sensor #0)",
    ),
    (
        "negative noise beside a later sensor's user data",
        r#"<mujoco model="p_N_suser_after"><size nuser_sensor="1"/><asset><hfield name="hf" nrow="3" ncol="3" size="1 1 0.1 0.1" elevation="0 0 0 0 0 0 0 0 0"/></asset><worldbody><geom name="floor" type="plane" size="1 1 0.1"/><geom name="hfg" type="hfield" hfield="hf" pos="5 0 0"/><body name="b" pos="0 0 0.5" ><joint name="j" type="hinge" axis="0 1 0" /><geom name="s" type="sphere" size="0.1" mass="1" /><site name="st" size="0.01"/></body><body name="c" pos="0.5 0 0.5"><geom name="s2" type="sphere" size="0.1" /></body></worldbody><contact></contact><equality></equality><tendon></tendon><actuator></actuator><sensor><jointpos joint="j" noise="-0.1"/><jointvel joint="j" user="1 2"/></sensor><keyframe></keyframe></mujoco>"#,
        "negative noise in sensor",
        "invalid user on <sensor>: user data exceeds nuser_sensor=1",
    ),
    (
        "quaternion cutoff beside a later sensor's unknown object",
        r#"<mujoco model="p_Q_obj_after"><asset><hfield name="hf" nrow="3" ncol="3" size="1 1 0.1 0.1" elevation="0 0 0 0 0 0 0 0 0"/></asset><worldbody><geom name="floor" type="plane" size="1 1 0.1"/><geom name="hfg" type="hfield" hfield="hf" pos="5 0 0"/><body name="b" pos="0 0 0.5" ><joint name="j" type="hinge" axis="0 1 0" /><geom name="s" type="sphere" size="0.1" mass="1" /><site name="st" size="0.01"/></body><body name="c" pos="0.5 0 0.5"><geom name="s2" type="sphere" size="0.1" /></body></worldbody><contact></contact><equality></equality><tendon></tendon><actuator></actuator><sensor><framequat objtype="site" objname="st" cutoff="0.5"/><jointvel joint="nope"/></sensor><keyframe></keyframe></mujoco>"#,
        "cutoff applied to axis or quaternion datatype in sensor",
        "unknown joint 'nope' (sensor #1)",
    ),
    (
        "quaternion cutoff beside a later sensor's user data",
        r#"<mujoco model="p_Q_suser_after"><size nuser_sensor="1"/><asset><hfield name="hf" nrow="3" ncol="3" size="1 1 0.1 0.1" elevation="0 0 0 0 0 0 0 0 0"/></asset><worldbody><geom name="floor" type="plane" size="1 1 0.1"/><geom name="hfg" type="hfield" hfield="hf" pos="5 0 0"/><body name="b" pos="0 0 0.5" ><joint name="j" type="hinge" axis="0 1 0" /><geom name="s" type="sphere" size="0.1" mass="1" /><site name="st" size="0.01"/></body><body name="c" pos="0.5 0 0.5"><geom name="s2" type="sphere" size="0.1" /></body></worldbody><contact></contact><equality></equality><tendon></tendon><actuator></actuator><sensor><framequat objtype="site" objname="st" cutoff="0.5"/><jointvel joint="j" user="1 2"/></sensor><keyframe></keyframe></mujoco>"#,
        "cutoff applied to axis or quaternion datatype in sensor",
        "invalid user on <sensor>: user data exceeds nuser_sensor=1",
    ),
    (
        "negative noise beside a later sensor's delay without a history buffer",
        r#"<mujoco model="later_N_delay"><asset><hfield name="hf" nrow="3" ncol="3" size="1 1 0.1 0.1" elevation="0 0 0 0 0 0 0 0 0"/></asset><worldbody><geom name="floor" type="plane" size="2 2 0.1"/><geom name="hfg" type="hfield" hfield="hf" pos="5 0 0"/><body name="b" pos="0 0 0.5"><joint name="j" type="hinge" axis="0 1 0"/><geom name="s" type="sphere" size="0.1" mass="1"/><site name="st" size="0.01"/></body><body name="d" pos="1 0 0.5"><joint name="dj" type="slide" axis="1 0 0"/><geom name="s3" type="box" size="0.1 0.1 0.1" mass="1"/></body></worldbody><sensor><jointpos joint="j" noise="-1"/><jointvel joint="j" delay="0.1"/></sensor></mujoco>"#,
        "negative noise in sensor",
        "invalid delay on <sensor>: setting delay > 0 without a history buffer (nsample must be > 0) (sensor #1)",
    ),
    (
        "negative noise beside a later user sensor's negative dim",
        r#"<mujoco model="later_N_dimneg"><asset><hfield name="hf" nrow="3" ncol="3" size="1 1 0.1 0.1" elevation="0 0 0 0 0 0 0 0 0"/></asset><worldbody><geom name="floor" type="plane" size="2 2 0.1"/><geom name="hfg" type="hfield" hfield="hf" pos="5 0 0"/><body name="b" pos="0 0 0.5"><joint name="j" type="hinge" axis="0 1 0"/><geom name="s" type="sphere" size="0.1" mass="1"/><site name="st" size="0.01"/></body><body name="d" pos="1 0 0.5"><joint name="dj" type="slide" axis="1 0 0"/><geom name="s3" type="box" size="0.1 0.1 0.1" mass="1"/></body></worldbody><sensor><jointpos joint="j" noise="-1"/><user dim="-1"/></sensor></mujoco>"#,
        "negative noise in sensor",
        "invalid dim on <user>: sensor dim must be non-negative in sensor (sensor #1)",
    ),
    (
        "negative noise beside a later sensor's negative interval",
        r#"<mujoco model="later_N_interval"><asset><hfield name="hf" nrow="3" ncol="3" size="1 1 0.1 0.1" elevation="0 0 0 0 0 0 0 0 0"/></asset><worldbody><geom name="floor" type="plane" size="2 2 0.1"/><geom name="hfg" type="hfield" hfield="hf" pos="5 0 0"/><body name="b" pos="0 0 0.5"><joint name="j" type="hinge" axis="0 1 0"/><geom name="s" type="sphere" size="0.1" mass="1"/><site name="st" size="0.01"/></body><body name="d" pos="1 0 0.5"><joint name="dj" type="slide" axis="1 0 0"/><geom name="s3" type="box" size="0.1 0.1 0.1" mass="1"/></body></worldbody><sensor><jointpos joint="j" noise="-1"/><jointvel joint="j" interval="-1"/></sensor></mujoco>"#,
        "negative noise in sensor",
        "invalid interval on <sensor>: negative interval in sensor (sensor #1)",
    ),
    (
        "negative cutoff beside a later sensor's delay without a history buffer",
        r#"<mujoco model="later_C_delay"><asset><hfield name="hf" nrow="3" ncol="3" size="1 1 0.1 0.1" elevation="0 0 0 0 0 0 0 0 0"/></asset><worldbody><geom name="floor" type="plane" size="2 2 0.1"/><geom name="hfg" type="hfield" hfield="hf" pos="5 0 0"/><body name="b" pos="0 0 0.5"><joint name="j" type="hinge" axis="0 1 0"/><geom name="s" type="sphere" size="0.1" mass="1"/><site name="st" size="0.01"/></body><body name="d" pos="1 0 0.5"><joint name="dj" type="slide" axis="1 0 0"/><geom name="s3" type="box" size="0.1 0.1 0.1" mass="1"/></body></worldbody><sensor><jointpos joint="j" cutoff="-1"/><jointvel joint="j" delay="0.1"/></sensor></mujoco>"#,
        "negative cutoff in sensor",
        "invalid delay on <sensor>: setting delay > 0 without a history buffer (nsample must be > 0) (sensor #1)",
    ),
    (
        "negative cutoff beside a later user sensor's negative dim",
        r#"<mujoco model="later_C_dimneg"><asset><hfield name="hf" nrow="3" ncol="3" size="1 1 0.1 0.1" elevation="0 0 0 0 0 0 0 0 0"/></asset><worldbody><geom name="floor" type="plane" size="2 2 0.1"/><geom name="hfg" type="hfield" hfield="hf" pos="5 0 0"/><body name="b" pos="0 0 0.5"><joint name="j" type="hinge" axis="0 1 0"/><geom name="s" type="sphere" size="0.1" mass="1"/><site name="st" size="0.01"/></body><body name="d" pos="1 0 0.5"><joint name="dj" type="slide" axis="1 0 0"/><geom name="s3" type="box" size="0.1 0.1 0.1" mass="1"/></body></worldbody><sensor><jointpos joint="j" cutoff="-1"/><user dim="-1"/></sensor></mujoco>"#,
        "negative cutoff in sensor",
        "invalid dim on <user>: sensor dim must be non-negative in sensor (sensor #1)",
    ),
    (
        "negative cutoff beside a later sensor's negative interval",
        r#"<mujoco model="later_C_interval"><asset><hfield name="hf" nrow="3" ncol="3" size="1 1 0.1 0.1" elevation="0 0 0 0 0 0 0 0 0"/></asset><worldbody><geom name="floor" type="plane" size="2 2 0.1"/><geom name="hfg" type="hfield" hfield="hf" pos="5 0 0"/><body name="b" pos="0 0 0.5"><joint name="j" type="hinge" axis="0 1 0"/><geom name="s" type="sphere" size="0.1" mass="1"/><site name="st" size="0.01"/></body><body name="d" pos="1 0 0.5"><joint name="dj" type="slide" axis="1 0 0"/><geom name="s3" type="box" size="0.1 0.1 0.1" mass="1"/></body></worldbody><sensor><jointpos joint="j" cutoff="-1"/><jointvel joint="j" interval="-1"/></sensor></mujoco>"#,
        "negative cutoff in sensor",
        "invalid interval on <sensor>: negative interval in sensor (sensor #1)",
    ),
    (
        "quaternion cutoff beside a later sensor's delay without a history buffer",
        r#"<mujoco model="later_Q_delay"><asset><hfield name="hf" nrow="3" ncol="3" size="1 1 0.1 0.1" elevation="0 0 0 0 0 0 0 0 0"/></asset><worldbody><geom name="floor" type="plane" size="2 2 0.1"/><geom name="hfg" type="hfield" hfield="hf" pos="5 0 0"/><body name="b" pos="0 0 0.5"><joint name="j" type="hinge" axis="0 1 0"/><geom name="s" type="sphere" size="0.1" mass="1"/><site name="st" size="0.01"/></body><body name="d" pos="1 0 0.5"><joint name="dj" type="slide" axis="1 0 0"/><geom name="s3" type="box" size="0.1 0.1 0.1" mass="1"/></body></worldbody><sensor><framequat objtype="site" objname="st" cutoff="1"/><jointvel joint="j" delay="0.1"/></sensor></mujoco>"#,
        "cutoff applied to axis or quaternion datatype in sensor",
        "invalid delay on <sensor>: setting delay > 0 without a history buffer (nsample must be > 0) (sensor #1)",
    ),
    (
        "quaternion cutoff beside a later user sensor's negative dim",
        r#"<mujoco model="later_Q_dimneg"><asset><hfield name="hf" nrow="3" ncol="3" size="1 1 0.1 0.1" elevation="0 0 0 0 0 0 0 0 0"/></asset><worldbody><geom name="floor" type="plane" size="2 2 0.1"/><geom name="hfg" type="hfield" hfield="hf" pos="5 0 0"/><body name="b" pos="0 0 0.5"><joint name="j" type="hinge" axis="0 1 0"/><geom name="s" type="sphere" size="0.1" mass="1"/><site name="st" size="0.01"/></body><body name="d" pos="1 0 0.5"><joint name="dj" type="slide" axis="1 0 0"/><geom name="s3" type="box" size="0.1 0.1 0.1" mass="1"/></body></worldbody><sensor><framequat objtype="site" objname="st" cutoff="1"/><user dim="-1"/></sensor></mujoco>"#,
        "cutoff applied to axis or quaternion datatype in sensor",
        "invalid dim on <user>: sensor dim must be non-negative in sensor (sensor #1)",
    ),
    (
        "quaternion cutoff beside a later sensor's negative interval",
        r#"<mujoco model="later_Q_interval"><asset><hfield name="hf" nrow="3" ncol="3" size="1 1 0.1 0.1" elevation="0 0 0 0 0 0 0 0 0"/></asset><worldbody><geom name="floor" type="plane" size="2 2 0.1"/><geom name="hfg" type="hfield" hfield="hf" pos="5 0 0"/><body name="b" pos="0 0 0.5"><joint name="j" type="hinge" axis="0 1 0"/><geom name="s" type="sphere" size="0.1" mass="1"/><site name="st" size="0.01"/></body><body name="d" pos="1 0 0.5"><joint name="dj" type="slide" axis="1 0 0"/><geom name="s3" type="box" size="0.1 0.1 0.1" mass="1"/></body></worldbody><sensor><framequat objtype="site" objname="st" cutoff="1"/><jointvel joint="j" interval="-1"/></sensor></mujoco>"#,
        "cutoff applied to axis or quaternion datatype in sensor",
        "invalid interval on <sensor>: negative interval in sensor (sensor #1)",
    ),
    (
        "height-field distance beside a later sensor's delay without a history buffer",
        r#"<mujoco model="later_H_delay"><asset><hfield name="hf" nrow="3" ncol="3" size="1 1 0.1 0.1" elevation="0 0 0 0 0 0 0 0 0"/></asset><worldbody><geom name="floor" type="plane" size="2 2 0.1"/><geom name="hfg" type="hfield" hfield="hf" pos="5 0 0"/><body name="b" pos="0 0 0.5"><joint name="j" type="hinge" axis="0 1 0"/><geom name="s" type="sphere" size="0.1" mass="1"/><site name="st" size="0.01"/></body><body name="d" pos="1 0 0.5"><joint name="dj" type="slide" axis="1 0 0"/><geom name="s3" type="box" size="0.1 0.1 0.1" mass="1"/></body></worldbody><sensor><distance geom1="hfg" geom2="s"/><jointvel joint="j" delay="0.1"/></sensor></mujoco>"#,
        "height fields are not supported in geom distance sensors",
        "invalid delay on <sensor>: setting delay > 0 without a history buffer (nsample must be > 0) (sensor #1)",
    ),
    (
        "height-field distance beside a later user sensor's negative dim",
        r#"<mujoco model="later_H_dimneg"><asset><hfield name="hf" nrow="3" ncol="3" size="1 1 0.1 0.1" elevation="0 0 0 0 0 0 0 0 0"/></asset><worldbody><geom name="floor" type="plane" size="2 2 0.1"/><geom name="hfg" type="hfield" hfield="hf" pos="5 0 0"/><body name="b" pos="0 0 0.5"><joint name="j" type="hinge" axis="0 1 0"/><geom name="s" type="sphere" size="0.1" mass="1"/><site name="st" size="0.01"/></body><body name="d" pos="1 0 0.5"><joint name="dj" type="slide" axis="1 0 0"/><geom name="s3" type="box" size="0.1 0.1 0.1" mass="1"/></body></worldbody><sensor><distance geom1="hfg" geom2="s"/><user dim="-1"/></sensor></mujoco>"#,
        "height fields are not supported in geom distance sensors",
        "invalid dim on <user>: sensor dim must be non-negative in sensor (sensor #1)",
    ),
    (
        "height-field distance beside a later sensor's negative interval",
        r#"<mujoco model="later_H_interval"><asset><hfield name="hf" nrow="3" ncol="3" size="1 1 0.1 0.1" elevation="0 0 0 0 0 0 0 0 0"/></asset><worldbody><geom name="floor" type="plane" size="2 2 0.1"/><geom name="hfg" type="hfield" hfield="hf" pos="5 0 0"/><body name="b" pos="0 0 0.5"><joint name="j" type="hinge" axis="0 1 0"/><geom name="s" type="sphere" size="0.1" mass="1"/><site name="st" size="0.01"/></body><body name="d" pos="1 0 0.5"><joint name="dj" type="slide" axis="1 0 0"/><geom name="s3" type="box" size="0.1 0.1 0.1" mass="1"/></body></worldbody><sensor><distance geom1="hfg" geom2="s"/><jointvel joint="j" interval="-1"/></sensor></mujoco>"#,
        "height fields are not supported in geom distance sensors",
        "invalid interval on <sensor>: negative interval in sensor (sensor #1)",
    ),
    (
        "negative noise beside its own delay without a history buffer",
        r#"<mujoco model="same_N_delay"><asset><hfield name="hf" nrow="3" ncol="3" size="1 1 0.1 0.1" elevation="0 0 0 0 0 0 0 0 0"/></asset><worldbody><geom name="floor" type="plane" size="2 2 0.1"/><geom name="hfg" type="hfield" hfield="hf" pos="5 0 0"/><body name="b" pos="0 0 0.5"><joint name="j" type="hinge" axis="0 1 0"/><geom name="s" type="sphere" size="0.1" mass="1"/><site name="st" size="0.01"/></body><body name="d" pos="1 0 0.5"><joint name="dj" type="slide" axis="1 0 0"/><geom name="s3" type="box" size="0.1 0.1 0.1" mass="1"/></body></worldbody><sensor><jointpos joint="j" noise="-1" delay="0.1"/></sensor></mujoco>"#,
        "negative noise in sensor",
        "invalid delay on <sensor>: setting delay > 0 without a history buffer (nsample must be > 0) (sensor #0)",
    ),
    (
        "negative cutoff beside its own delay without a history buffer",
        r#"<mujoco model="same_C_delay"><asset><hfield name="hf" nrow="3" ncol="3" size="1 1 0.1 0.1" elevation="0 0 0 0 0 0 0 0 0"/></asset><worldbody><geom name="floor" type="plane" size="2 2 0.1"/><geom name="hfg" type="hfield" hfield="hf" pos="5 0 0"/><body name="b" pos="0 0 0.5"><joint name="j" type="hinge" axis="0 1 0"/><geom name="s" type="sphere" size="0.1" mass="1"/><site name="st" size="0.01"/></body><body name="d" pos="1 0 0.5"><joint name="dj" type="slide" axis="1 0 0"/><geom name="s3" type="box" size="0.1 0.1 0.1" mass="1"/></body></worldbody><sensor><jointpos joint="j" cutoff="-1" delay="0.1"/></sensor></mujoco>"#,
        "negative cutoff in sensor",
        "invalid delay on <sensor>: setting delay > 0 without a history buffer (nsample must be > 0) (sensor #0)",
    ),
    (
        "negative noise beside its own negative user dim",
        r#"<mujoco model="f_nn_userdim"><asset><hfield name="hf" nrow="3" ncol="3" size="1 1 0.1 0.1" elevation="0 0 0 0 0 0 0 0 0"/></asset><worldbody><geom name="floor" type="plane" size="2 2 0.1"/><body name="hb" pos="5 0 0"><geom name="hfg" type="hfield" hfield="hf"/></body><frame name="fr" pos="0 5 0"><geom name="hff" type="hfield" hfield="hf"/></frame><body name="b" pos="0 0 0.5"><joint name="j" type="hinge" axis="0 1 0" limited="true" range="-1 1"/><geom name="s" type="sphere" size="0.1" mass="1"/><site name="st" size="0.01"/><body name="c" pos="0 0 0.3"><joint name="bj" type="ball"/><geom name="s2" type="sphere" size="0.05" mass="0.5"/></body></body><body name="d" pos="1 0 0.5"><joint name="dj" type="slide" axis="1 0 0"/><geom name="s3" type="box" size="0.1 0.1 0.1" mass="1"/></body></worldbody><tendon><fixed name="t" limited="true" range="-1 1"><joint joint="j" coef="1"/></fixed></tendon><actuator><motor name="a" joint="j"/><motor name="at" tendon="t"/></actuator><sensor><user noise="-1" dim="-1"/></sensor></mujoco>"#,
        "negative noise in sensor",
        "invalid dim on <user>: sensor dim must be non-negative in sensor (sensor #0)",
    ),
    (
        "negative noise beside a timestep of zero",
        r#"<mujoco model="stage_N_timestep0"><option timestep="0"/><asset><hfield name="hf" nrow="3" ncol="3" size="1 1 0.1 0.1" elevation="0 0 0 0 0 0 0 0 0"/></asset><worldbody><geom name="floor" type="plane" size="2 2 0.1"/><geom name="hfg" type="hfield" hfield="hf" pos="5 0 0"/><body name="b" pos="0 0 0.5"><joint name="j" type="hinge" axis="0 1 0"/><geom name="s" type="sphere" size="0.1" mass="1"/><site name="st" size="0.01"/></body><body name="d" pos="1 0 0.5"><joint name="dj" type="slide" axis="1 0 0"/><geom name="s3" type="box" size="0.1 0.1 0.1" mass="1"/></body></worldbody><sensor><jointpos joint="j" noise="-1"/></sensor></mujoco>"#,
        "negative noise in sensor",
        "invalid timestep on <option>: must be positive, got 0",
    ),
    (
        "negative cutoff beside a timestep of zero",
        r#"<mujoco model="stage_C_timestep0"><option timestep="0"/><asset><hfield name="hf" nrow="3" ncol="3" size="1 1 0.1 0.1" elevation="0 0 0 0 0 0 0 0 0"/></asset><worldbody><geom name="floor" type="plane" size="2 2 0.1"/><geom name="hfg" type="hfield" hfield="hf" pos="5 0 0"/><body name="b" pos="0 0 0.5"><joint name="j" type="hinge" axis="0 1 0"/><geom name="s" type="sphere" size="0.1" mass="1"/><site name="st" size="0.01"/></body><body name="d" pos="1 0 0.5"><joint name="dj" type="slide" axis="1 0 0"/><geom name="s3" type="box" size="0.1 0.1 0.1" mass="1"/></body></worldbody><sensor><jointpos joint="j" cutoff="-1"/></sensor></mujoco>"#,
        "negative cutoff in sensor",
        "invalid timestep on <option>: must be positive, got 0",
    ),
    (
        "quaternion cutoff beside a timestep of zero",
        r#"<mujoco model="stage_Q_timestep0"><option timestep="0"/><asset><hfield name="hf" nrow="3" ncol="3" size="1 1 0.1 0.1" elevation="0 0 0 0 0 0 0 0 0"/></asset><worldbody><geom name="floor" type="plane" size="2 2 0.1"/><geom name="hfg" type="hfield" hfield="hf" pos="5 0 0"/><body name="b" pos="0 0 0.5"><joint name="j" type="hinge" axis="0 1 0"/><geom name="s" type="sphere" size="0.1" mass="1"/><site name="st" size="0.01"/></body><body name="d" pos="1 0 0.5"><joint name="dj" type="slide" axis="1 0 0"/><geom name="s3" type="box" size="0.1 0.1 0.1" mass="1"/></body></worldbody><sensor><framequat objtype="site" objname="st" cutoff="1"/></sensor></mujoco>"#,
        "cutoff applied to axis or quaternion datatype in sensor",
        "invalid timestep on <option>: must be positive, got 0",
    ),
    (
        "height-field distance beside a timestep of zero",
        r#"<mujoco model="stage_H_timestep0"><option timestep="0"/><asset><hfield name="hf" nrow="3" ncol="3" size="1 1 0.1 0.1" elevation="0 0 0 0 0 0 0 0 0"/></asset><worldbody><geom name="floor" type="plane" size="2 2 0.1"/><geom name="hfg" type="hfield" hfield="hf" pos="5 0 0"/><body name="b" pos="0 0 0.5"><joint name="j" type="hinge" axis="0 1 0"/><geom name="s" type="sphere" size="0.1" mass="1"/><site name="st" size="0.01"/></body><body name="d" pos="1 0 0.5"><joint name="dj" type="slide" axis="1 0 0"/><geom name="s3" type="box" size="0.1 0.1 0.1" mass="1"/></body></worldbody><sensor><distance geom1="hfg" geom2="s"/></sensor></mujoco>"#,
        "height fields are not supported in geom distance sensors",
        "invalid timestep on <option>: must be positive, got 0",
    ),
];

#[test]
fn a_sensors_fault_beside_another_is_named_as_ledgered() {
    let mut moved = Vec::new();
    for &(what, xml, _mujoco, ours) in SENSOR_COSTS {
        match load_model(xml) {
            Err(e) if e.to_string() == ours => {}
            Err(e) => moved.push(format!("{what}: {e}")),
            Ok(_) => moved.push(format!("{what}: loads")),
        }
    }
    assert!(moved.is_empty(), "{}", moved.join("\n"));
}

/// Two faults, one a flex's thickness or a flex equality's flex, where this
/// crate names the other: the thickness and the flex equalities are checked
/// with the checks RL-6a added, after every check this crate makes before
/// building the model, where a flex's repeated vertex is refused when the
/// flexes are built, a connect's body when the equalities are made and a
/// tendon's joint when the tendons are made; MuJoCo 3.5.0 checks the flex
/// first (ledger L353). `(what, document, MuJoCo's reason, this crate's
/// error)`, measured 2026-11-07 (RL-8a's review round 1).
const FLEX_COSTS: &[(&str, &str, &str, &str)] = &[
    (
        "a flex equality's unknown flex then a later connect's unknown body",
        r#"<mujoco><default></default><worldbody><body name="v0" pos="0 0 0"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body><body name="v1" pos="1 0 0"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body><body name="v2" pos="0 1 0"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body><body name="v3" pos="1 1 0"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body></worldbody><deformable><flex name="f" dim="2" body="v0 v1 v2 v3" element="0 1 2  1 3 2" ><contact contype="0" conaffinity="0"/></flex></deformable><equality><flex flex="nope"/><connect body1="nobody" body2="v1" anchor="0 0 0"/></equality></mujoco>"#,
        "unknown element 'nope' in equality constraint",
        "unknown body 'nobody' (equality #1)",
    ),
    (
        "a flex's thickness then a later flex's repeated vertex",
        r#"<mujoco><default></default><worldbody><body name="v0" pos="0 0 0"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body><body name="v1" pos="1 0 0"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body><body name="v2" pos="0 1 0"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body><body name="v3" pos="1 1 0"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body><body name="w0" pos="3 0 0"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body><body name="w1" pos="4 0 0"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body><body name="w2" pos="3 1 0"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body><body name="w3" pos="4 1 0"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body></worldbody><deformable><flex name="f" dim="2" body="v0 v1 v2 v3" element="0 1 2  1 3 2" ><contact contype="0" conaffinity="0"/><elasticity elastic2d="bend" young="10000.0" thickness="-0.5"/></flex><flex name="g" dim="2" body="w0 w1 w2 w3" element="0 1 1  1 3 2" ><contact contype="0" conaffinity="0"/></flex></deformable><equality></equality></mujoco>"#,
        "thickness must be positive for bending stiffness",
        "invalid element on <flex>: repeated vertex in element (flex 'g')",
    ),
    (
        "a flex equality's rigid flexcomp then a tendon's unknown joint",
        r#"<mujoco><default></default><worldbody><flexcomp name="c" type="grid" count="2 2 1" spacing="0.1 0.1 0.1" dim="2" mass="1"><pin id="0 1 2 3"/><contact contype="0" conaffinity="0"/></flexcomp></worldbody><equality><flex flex="c"/></equality><tendon><fixed armature="-1"><joint joint="nojoint" coef="1"/></fixed></tendon></mujoco>"#,
        "rigid flex 'c' in equality constraint 0",
        "unknown joint 'nojoint' (tendon #0)",
    ),
];

#[test]
fn a_flex_fault_beside_another_is_named_as_ledgered() {
    let mut moved = Vec::new();
    for &(what, xml, _mujoco, ours) in FLEX_COSTS {
        match load_model(xml) {
            Err(e) if e.to_string() == ours => {}
            Err(e) => moved.push(format!("{what}: {e}")),
            Ok(_) => moved.push(format!("{what}: loads")),
        }
    }
    assert!(moved.is_empty(), "{}", moved.join("\n"));
}

/// Two faults, one a body-level flexcomp's, where this crate names the other
/// one than MuJoCo 3.5.0: ledger L353 holds each. `(what, document, MuJoCo's
/// reason, this crate's error)`, measured 2026-11-07 (RL-8b's two-fault set):
/// MuJoCo's reader meets a repeated name, an unknown class or a composite's
/// fault written before the flexcomp first, where this crate checks names
/// after making it and its reader's checks before the classes and the
/// composites; and MuJoCo makes the flexcomp before a later parse fault or
/// unknown class, which this crate meets first, as it did before the flexcomp
/// was made; of two flexcomps of one name MuJoCo names their first bodies'
/// name, this crate their flexes', checked before the bodies'.
const FLEXCOMP_COSTS: &[(&str, &str, &str, &str)] = &[
    (
        "a repeated name before a flexcomp's count",
        r#"<mujoco><worldbody><body name="dup"/><body name="dup"/><flexcomp name="f" type="grid" count="0 1 1" spacing="0.1 0.1 0.1" dim="1"/></worldbody></mujoco>"#,
        "repeated name 'dup' in body",
        "invalid count on <flexcomp>: Count too small (flex 'f')",
    ),
    (
        "a repeated name before a flexcomp with no name",
        r#"<mujoco><worldbody><body name="dup"/><body name="dup"/><flexcomp type="grid" count="2 1 1" spacing="0.1 0.1 0.1" dim="1"/></worldbody></mujoco>"#,
        "repeated name 'dup' in body",
        "required attribute missing: 'name' on <flexcomp> (line 1: worldbody > flexcomp)",
    ),
    (
        "an unknown class before a flexcomp's type keyword",
        r#"<mujoco><worldbody><geom class="nope" size="0.1"/><flexcomp name="f" type="blob" count="2 1 1" spacing="0.1 0.1 0.1" dim="1"/></worldbody></mujoco>"#,
        "unknown default class name 'nope'",
        "invalid keyword \"blob\" in attribute 'type' on <flexcomp> (line 1: worldbody > flexcomp 'f')",
    ),
    (
        "a composite's fault before a flexcomp's two orientations",
        r#"<mujoco><worldbody><body name="cb"><composite type="cable" count="4 2 1" curve="s" size="1"><geom type="capsule" size=".01"/></composite></body><flexcomp name="f" type="grid" count="2 1 1" spacing="0.1 0.1 0.1" dim="1" euler="0 0 90" axisangle="0 0 1 90"/></worldbody></mujoco>"#,
        "Cable must be one-dimensional",
        "<flexcomp> may set only one of: axisangle, euler (line 1: worldbody > flexcomp 'f')",
    ),
    (
        "a flexcomp's spacing before a later geom's unknown class",
        r#"<mujoco><worldbody><flexcomp name="f" type="grid" count="2 1 1" spacing="0.001 0.1 0.1" dim="1"/><geom class="nope" size="0.1"/></worldbody></mujoco>"#,
        "Spacing must be larger than geometry size",
        "unknown default class name 'nope' (geom in body 'world')",
    ),
    (
        "a flexcomp's mass before a later geom's type keyword",
        r#"<mujoco><worldbody><flexcomp name="f" type="grid" count="2 1 1" spacing="0.1 0.1 0.1" dim="1" mass="0"/><geom type="sausage" size="0.1"/></worldbody></mujoco>"#,
        "Mass and inertiabox must be larger than mjMINVAL",
        "invalid keyword \"sausage\" in attribute 'type' on <geom> (line 1: worldbody > geom)",
    ),
    (
        "two flexcomps of one name",
        r#"<mujoco><worldbody><flexcomp name="f" type="grid" count="2 1 1" spacing="0.1 0.1 0.1" dim="1"/><flexcomp name="f" type="grid" count="2 1 1" spacing="0.1 0.1 0.1" dim="1"/></worldbody></mujoco>"#,
        "repeated name 'f_0' in body",
        "repeated name 'f' in flex",
    ),
];

#[test]
fn a_flexcomp_fault_beside_another_is_named_as_ledgered() {
    let mut moved = Vec::new();
    for &(what, xml, _mujoco, ours) in FLEXCOMP_COSTS {
        match load_model(xml) {
            Err(e) if e.to_string() == ours => {}
            Err(e) => moved.push(format!("{what}: {e}")),
            Ok(_) => moved.push(format!("{what}: loads")),
        }
    }
    assert!(moved.is_empty(), "{}", moved.join("\n"));
}

/// Two faults, one refused as MuJoCo 3.5.0's schema refuses it (a `<flexcomp>`
/// in `<deformable>`, or on a `<flex>` an attribute this crate once read for
/// its own flexcomp), where this crate names a fault its parser meets written
/// before it, and MuJoCo's schema, walking the whole document before any
/// value is read, names the refused element or attribute: ledger L353 holds
/// each. `(what, document, MuJoCo's reason, this crate's error)`, measured
/// 2026-11-08 (RL-8c's two-fault set). Each of the other four attributes
/// stands as `density` does.
const SCHEMA_COSTS: &[(&str, &str, &str, &str)] = &[
    (
        "a parse fault before a deformable flexcomp",
        r#"<mujoco><option timestep="abc"/><worldbody><body name="a"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body><body name="b" pos="1 0 0"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body></worldbody><deformable><flexcomp name="c" type="grid" count="2 2 1" spacing="0.1 0.1 0.1" dim="2"/></deformable></mujoco>"#,
        "Schema violation: unrecognized element",
        "bad number \"abc\" in attribute 'timestep' on <option> (line 1: option)",
    ),
    (
        "a keyword before a deformable flexcomp",
        r#"<mujoco><option integrator="sausage"/><worldbody><body name="a"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body><body name="b" pos="1 0 0"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body></worldbody><deformable><flexcomp name="c" type="grid" count="2 2 1" spacing="0.1 0.1 0.1" dim="2"/></deformable></mujoco>"#,
        "Schema violation: unrecognized element",
        "invalid keyword \"sausage\" in attribute 'integrator' on <option> (line 1: option)",
    ),
    (
        "a frame's two orientations before a deformable flexcomp",
        r#"<mujoco><worldbody><body name="a"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body><body name="b" pos="1 0 0"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body><frame euler="0 0 1" axisangle="0 0 1 1"><geom size="0.1"/></frame></worldbody><deformable><flexcomp name="c" type="grid" count="2 2 1" spacing="0.1 0.1 0.1" dim="2"/></deformable></mujoco>"#,
        "Schema violation: unrecognized element",
        "<frame> may set only one of: axisangle, euler (line 1: worldbody > frame)",
    ),
    (
        "a parse fault before a flex's density",
        r#"<mujoco><option timestep="abc"/><worldbody><body name="a"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body><body name="b" pos="1 0 0"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body></worldbody><deformable><flex name="f" dim="1" body="a b" element="0 1" density="10"/></deformable></mujoco>"#,
        "Schema violation: unrecognized attribute: 'density'",
        "bad number \"abc\" in attribute 'timestep' on <option> (line 1: option)",
    ),
    (
        "a keyword before a flex's density",
        r#"<mujoco><option integrator="sausage"/><worldbody><body name="a"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body><body name="b" pos="1 0 0"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body></worldbody><deformable><flex name="f" dim="1" body="a b" element="0 1" density="10"/></deformable></mujoco>"#,
        "Schema violation: unrecognized attribute: 'density'",
        "invalid keyword \"sausage\" in attribute 'integrator' on <option> (line 1: option)",
    ),
    (
        "a frame's two orientations before a flex's density",
        r#"<mujoco><worldbody><body name="a"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body><body name="b" pos="1 0 0"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body><frame euler="0 0 1" axisangle="0 0 1 1"><geom size="0.1"/></frame></worldbody><deformable><flex name="f" dim="1" body="a b" element="0 1" density="10"/></deformable></mujoco>"#,
        "Schema violation: unrecognized attribute: 'density'",
        "<frame> may set only one of: axisangle, euler (line 1: worldbody > frame)",
    ),
];

#[test]
fn a_schema_fault_beside_another_is_named_as_ledgered() {
    let mut moved = Vec::new();
    for &(what, xml, _mujoco, ours) in SCHEMA_COSTS {
        match load_model(xml) {
            Err(e) if e.to_string() == ours => {}
            Err(e) => moved.push(format!("{what}: {e}")),
            Ok(_) => moved.push(format!("{what}: loads")),
        }
    }
    assert!(moved.is_empty(), "{}", moved.join("\n"));
}
