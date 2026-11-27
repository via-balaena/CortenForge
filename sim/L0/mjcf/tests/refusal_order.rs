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
//! that MuJoCo meets after a condim (a height field: `HFIELD_COSTS`). A
//! `<size>` `nuser_` below -1 is refused
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
//! are named in Make's order. The type this crate does not make (gmsh)
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
//! before them is named first here: `SCHEMA_COSTS` (ledger L353); and
//! `REFUSAL_FIRST_COSTS` pins each kind measured where MuJoCo names the other
//! fault and this crate the refusal (ledgers L315, L353).
//! RL-9b's refusal of `shellinertia` on a mesh or SDF geom runs with RL-6d's
//! checks of a geom, after its condim and plane checks and before its
//! negative mass, as MuJoCo orders it; a fault this crate meets as it reads
//! the model comes before it: the geom's own `user` data or orientation, as
//! in MuJoCo, and a later element's, where MuJoCo names the earlier geom's
//! `shellinertia`: `SHELLINERTIA_COSTS` (ledger L353). Its refusals of a mesh
//! (its area, its volume, an exact mesh's orientation, its principal
//! moments) are made as the meshes are built, after the checks this crate
//! makes before building the model, where MuJoCo makes them before its
//! bodies; a fault this crate checks first (a geom's size or `fromto`, a
//! tendon's or an actuator's reference among them) is named first here:
//! `MESH_COSTS` (ledger L353). RL-9b3 fits a primitive geom to the mesh it
//! names as its body is built, after those checks, where MuJoCo fits each
//! geom as it compiles it; a size those checks refuse in a later body or a
//! site is named before a fitted size MuJoCo refuses: `FIT_COSTS` (ledger
//! L353). A fitted size is named before a fault this crate checks after
//! the build (`check_bodies`) that MuJoCo meets first: `FIT_FIRST_COSTS`. A
//! geom's height field is checked as the model is validated, before its
//! condim and a mesh or SDF geom's mesh, which MuJoCo checks first:
//! `HFIELD_COSTS`. RL-9d checks each mesh's written vertex and face counts
//! after every name, the names the meshes take from their files among them,
//! which this crate gives before the classes, as it did before RL-9d; MuJoCo
//! checks the counts first as it begins to compile the model
//! (`mesh_data_checks.rs`'s `NAMED_OTHERWISE`). A `NaN` written in a vertex,
//! refused as it is read, is named before them: `MESH_DATA_COSTS`. Beside a
//! geom's mesh name that no
//! mesh holds, which MuJoCo names as it indexes the assets, before any geom
//! compiles, this crate names a `fromto` it refuses as the classes are
//! applied: `ASSET_COSTS` (ledger L353).
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
//! `SCHEMA_COSTS`; its review's (measured 2026-11-09) set one beside a fault
//! MuJoCo names first: nine in `REFUSAL_FIRST_COSTS`.
//! RL-9a's (measured 2026-11-09, its two-fault set of 95 documents: seven
//! faults of a body's inertia and eleven others, each alone, and the 77
//! pairs) set an `<inertial>`'s orientation alternative or a body's
//! principal axes from its geoms beside another fault: four in `TWO_FAULTS`,
//! five in `INERTIAL_COSTS`; its review's (measured 2026-11-09) set those and
//! a body's non-finite inertia from its geoms beside an orientation, a
//! `fullinertia`, a reference, a mocap body's place or another body's checks:
//! ten in `TWO_FAULTS`, twenty-six in `INERTIAL_COSTS`. Each is a sample
//! of the pairs, not all of them. RL-9b's review's (measured 2026-11-10,
//! RB's documents) set `shellinertia` on an SDF or mesh geom beside a condim,
//! a plane in a moving body, a `zaxis`, a `user` count or an earlier geom's
//! negative mass: nine in `TWO_FAULTS`; and a mesh refused for its volume
//! or an exact mesh's orientation beside a geom's size, `fromto` or fit: ten,
//! of which the two beside a fit are in `TWO_FAULTS` since RL-9b3 fits a
//! primitive to its mesh, the rest in `MESH_COSTS`. Its fix-diff pass's (FD's
//! documents) set `shellinertia` beside a later element's fault this crate
//! meets as it reads (seven in `SHELLINERTIA_COSTS`) and a mesh refused
//! beside a tendon's or an actuator's reference (three more in
//! `MESH_COSTS`). RL-9b3's (measured 2026-11-13, its start set) set a
//! fitted size MuJoCo refuses beside a later body's or a site's size: two in
//! `FIT_COSTS`. Its review's first round's (2026-11-14) set a fitted size
//! beside a fault checked after the build, seven in `FIT_FIRST_COSTS`, and a
//! height field beside a condim or a mesh geom's missing mesh, two in
//! `HFIELD_COSTS`. RL-9c3's review's (measured 2026-11-19, RW's documents)
//! set a negative face id beside a bad `maxhullvert` or `inertia`, which
//! MuJoCo names: two in `TWO_FAULTS`. RL-9d's (measured 2026-11-19) set a
//! mesh's written counts beside a vertex's `NaN`: one in `MESH_DATA_COSTS`;
//! and a box's `fromto` beside a mesh name no mesh holds: one in
//! `ASSET_COSTS`. Its review's first round's (measured 2026-11-19) set
//! fourteen in `TWO_FAULTS`, each named as MuJoCo names it: a repeated name
//! beside a mesh's count or a later class, an SDF geom naming no mesh beside
//! an earlier fault, a mesh attribute that does not read beside its
//! `maxhullvert` or `inertia`. Its fix-diff pass's (measured 2026-11-20) set
//! twelve more there: an unnamed mesh or height field, or two meshes one file
//! names, beside a plugin's repeated instance name, a name a flexcomp makes
//! or a `fromto`, each named as main and MuJoCo name it; an unnamed mesh
//! before a repeated mesh name; and a mesh's `scale` or `inertia` beside its
//! `refpos` or `maxhullvert`. RL-11b compiles each height field where MuJoCo
//! compiles it, after the condims, and loads one with counts and no
//! elevation, so two left `CONDIM_COSTS` for `TWO_FAULTS`: a geom's or a
//! pair's condim beside a height field's top of 0 (with no elevation,
//! which main refused while parsing), each named as MuJoCo names it
//! (measured 2026-11-26).

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
    (
        "an inertial's degenerate xyaxes then a later body's zero hinge axis",
        r#"<mujoco><worldbody><body name="n"><freejoint/><inertial pos="0 0 0" mass="1" diaginertia="1 1 1" xyaxes="1 0 0 1 0 0"/></body><body name="q" pos="2 0 0"><joint type="hinge" axis="0 0 0"/><geom type="sphere" size="0.1"/></body></worldbody></mujoco>"#,
        "error 'yaxis too small' in inertia alternative",
        "error 'yaxis too small' in inertia alternative (body 'n')",
    ),
    (
        "an inertial's degenerate xyaxes then a later body's moments' triangle",
        r#"<mujoco><worldbody><body name="n"><freejoint/><inertial pos="0 0 0" mass="1" diaginertia="1 1 1" xyaxes="1 0 0 1 0 0"/></body><body name="q" pos="2 0 0"><freejoint/><inertial pos="0 0 0" mass="1" diaginertia="1 1 5"/></body></worldbody></mujoco>"#,
        "error 'yaxis too small' in inertia alternative",
        "error 'yaxis too small' in inertia alternative (body 'n')",
    ),
    (
        "a body's geoms' principal axes then its own geom's size",
        r#"<mujoco><worldbody><body name="n"><geom type="cylinder" size="1e-7 1" mass="0.5"/><geom type="cylinder" size="1e-7 1" mass="0.5" pos="0 0 3"/><geom type="box" size="0.1 0.1 0"/></body></worldbody></mujoco>"#,
        "size 2 must be positive in geom",
        "invalid size on <geom>: size 2 must be positive (geom #2 in body 'n')",
    ),
    (
        "an earlier moving body below mjMINVAL then a body's geoms' principal axes",
        r#"<mujoco><worldbody><body name="p" pos="-2 0 0"><freejoint/><geom type="sphere" size="0.1" mass="0"/></body><body name="n"><geom type="cylinder" size="1e-7 1" mass="0.5"/><geom type="cylinder" size="1e-7 1" mass="0.5" pos="0 0 3"/></body></worldbody></mujoco>"#,
        "error 'inertia must have positive eigenvalues' in alternative for principal axes",
        "error 'inertia must have positive eigenvalues' in alternative for principal axes (body 'n')",
    ),
    (
        "an earlier body's zero zaxis then an inertial's degenerate xyaxes",
        r#"<mujoco><worldbody><body name="p" pos="-2 0 0" zaxis="0 0 0"><freejoint/><geom type="sphere" size="0.1"/></body><body name="n"><freejoint/><inertial pos="0 0 0" mass="1" diaginertia="1 1 1" xyaxes="1 0 0 1 0 0"/></body></worldbody></mujoco>"#,
        "error 'zaxis too small' in frame alternative",
        "invalid zaxis on <body>: zaxis too small (body 'p')",
    ),
    (
        "an inertial's degenerate xyaxes beside its own body's zero hinge axis",
        r#"<mujoco><worldbody><body name="n"><joint type="hinge" axis="0 0 0"/><inertial pos="0 0 0" mass="1" diaginertia="1 1 1" xyaxes="1 0 0 1 0 0"/></body></worldbody></mujoco>"#,
        "error 'yaxis too small' in inertia alternative",
        "error 'yaxis too small' in inertia alternative (body 'n')",
    ),
    (
        "an inertial's degenerate xyaxes then a later body's fullinertia beside a diagonal inertia",
        r#"<mujoco><worldbody><body name="n"><freejoint/><inertial pos="0 0 0" mass="1" diaginertia="1 1 1" xyaxes="1 0 0 1 0 0"/></body><body name="q" pos="2 0 0"><freejoint/><inertial pos="0 0 0" mass="1" diaginertia="1 1 1" fullinertia="1 1 1 0 0 0"/></body></worldbody></mujoco>"#,
        "error 'yaxis too small' in inertia alternative",
        "error 'yaxis too small' in inertia alternative (body 'n')",
    ),
    (
        "an inertial's degenerate xyaxes then a later body's negative mass",
        r#"<mujoco><worldbody><body name="n"><freejoint/><inertial pos="0 0 0" mass="1" diaginertia="1 1 1" xyaxes="1 0 0 1 0 0"/></body><body name="q" pos="2 0 0"><freejoint/><inertial pos="0 0 0" mass="-1" diaginertia="1 1 1"/></body></worldbody></mujoco>"#,
        "error 'yaxis too small' in inertia alternative",
        "error 'yaxis too small' in inertia alternative (body 'n')",
    ),
    (
        "an inertial's degenerate xyaxes then a keyframe's short qpos",
        r#"<mujoco><worldbody><body name="n"><freejoint/><inertial pos="0 0 0" mass="1" diaginertia="1 1 1" xyaxes="1 0 0 1 0 0"/></body></worldbody><keyframe><key qpos="1 2"/></keyframe></mujoco>"#,
        "error 'yaxis too small' in inertia alternative",
        "error 'yaxis too small' in inertia alternative (body 'n')",
    ),
    (
        "an inertial's degenerate xyaxes then a later body's unknown childclass",
        r#"<mujoco><worldbody><body name="n"><freejoint/><inertial pos="0 0 0" mass="1" diaginertia="1 1 1" xyaxes="1 0 0 1 0 0"/></body><body name="q" childclass="nope"><freejoint/><geom type="sphere" size="0.1"/></body></worldbody></mujoco>"#,
        "unknown default childclass",
        "unknown default class name 'nope' (body 'q')",
    ),
    (
        "an inertial's degenerate xyaxes then a contact pair's unknown geom",
        r#"<mujoco><worldbody><body name="n"><freejoint/><inertial pos="0 0 0" mass="1" diaginertia="1 1 1" xyaxes="1 0 0 1 0 0"/><geom name="g" type="sphere" size="0.1"/></body></worldbody><contact><pair geom1="nope" geom2="g"/></contact></mujoco>"#,
        "geom 'nope' not found in collision 0",
        "unknown geom 'nope' (pair #0)",
    ),
    (
        "an inertial's degenerate xyaxes then a zero-zaxis frame in the world",
        r#"<mujoco><worldbody><body name="n"><freejoint/><inertial pos="0 0 0" mass="1" diaginertia="1 1 1" xyaxes="1 0 0 1 0 0"/></body><frame zaxis="0 0 0"><geom type="sphere" size="0.1"/></frame></worldbody></mujoco>"#,
        "orientation specification error 'zaxis too small' in site -1",
        "invalid zaxis on <frame>: zaxis too small (frame in body 'world')",
    ),
    (
        "an earlier body's bad condim then a body's non-finite inertia from its geoms (D-INERTIA-NONFINITE: MuJoCo loads that body)",
        r#"<mujoco><worldbody><body name="p" pos="-2 0 0"><freejoint/><geom type="sphere" size="0.1" condim="2"/></body><body name="s"><geom type="sphere" size="6e99"/></body></worldbody></mujoco>"#,
        "invalid condim in geom",
        "invalid condim in geom (geom #0 in body 'p')",
    ),
    (
        "an earlier body's inertia breaking the triangle then a body's non-finite inertia from its geoms (D-INERTIA-NONFINITE: MuJoCo loads that body)",
        r#"<mujoco><worldbody><body name="p" pos="-2 0 0"><freejoint/><inertial pos="0 0 0" mass="1" diaginertia="1 1 5"/></body><body name="s"><geom type="sphere" size="6e99"/></body></worldbody></mujoco>"#,
        "inertia must satisfy A + B >= C; use 'balanceinertia' to fix",
        "inertia must satisfy A + B >= C; use 'balanceinertia' to fix (body 'p')",
    ),
    (
        "an SDF geom's shellinertia and its condim",
        r#"<mujoco><asset><mesh name="m" vertex="-0.5 -0.5 -0.5  0.5 -0.5 -0.5  0.5 0.5 -0.5  -0.5 0.5 -0.5  -0.5 -0.5 0.5  0.5 -0.5 0.5  0.5 0.5 0.5  -0.5 0.5 0.5" face="0 2 1  0 3 2  4 5 6  4 6 7  0 1 5  0 5 4  2 3 7  2 7 6  0 4 7  0 7 3  1 2 6  1 6 5"/></asset><worldbody><body><freejoint/><geom type="sdf" mesh="m" shellinertia="true" condim="2"/></body></worldbody></mujoco>"#,
        "invalid condim in geom",
        "invalid condim in geom (geom #0 in body #1)",
    ),
    (
        "a mesh geom's shellinertia and its condim",
        r#"<mujoco><asset><mesh name="m" vertex="-0.5 -0.5 -0.5  0.5 -0.5 -0.5  0.5 0.5 -0.5  -0.5 0.5 -0.5  -0.5 -0.5 0.5  0.5 -0.5 0.5  0.5 0.5 0.5  -0.5 0.5 0.5" face="0 2 1  0 3 2  4 5 6  4 6 7  0 1 5  0 5 4  2 3 7  2 7 6  0 4 7  0 7 3  1 2 6  1 6 5"/></asset><worldbody><body><freejoint/><geom type="mesh" mesh="m" shellinertia="true" condim="2"/></body></worldbody></mujoco>"#,
        "invalid condim in geom",
        "invalid condim in geom (geom #0 in body #1)",
    ),
    (
        "a plane in a moving body, then an SDF geom's shellinertia",
        r#"<mujoco><asset><mesh name="m" vertex="-0.5 -0.5 -0.5  0.5 -0.5 -0.5  0.5 0.5 -0.5  -0.5 0.5 -0.5  -0.5 -0.5 0.5  0.5 -0.5 0.5  0.5 0.5 0.5  -0.5 0.5 0.5" face="0 2 1  0 3 2  4 5 6  4 6 7  0 1 5  0 5 4  2 3 7  2 7 6  0 4 7  0 7 3  1 2 6  1 6 5"/></asset><worldbody><body><freejoint/><geom type="plane" size="1 1 1"/><geom type="sdf" mesh="m" shellinertia="true"/></body></worldbody></mujoco>"#,
        "plane only allowed in static bodies",
        "plane only allowed in static bodies (geom #0 in body #1)",
    ),
    (
        "a plane in a moving body, then a mesh geom's shellinertia",
        r#"<mujoco><asset><mesh name="m" vertex="-0.5 -0.5 -0.5  0.5 -0.5 -0.5  0.5 0.5 -0.5  -0.5 0.5 -0.5  -0.5 -0.5 0.5  0.5 -0.5 0.5  0.5 0.5 0.5  -0.5 0.5 0.5" face="0 2 1  0 3 2  4 5 6  4 6 7  0 1 5  0 5 4  2 3 7  2 7 6  0 4 7  0 7 3  1 2 6  1 6 5"/></asset><worldbody><body><freejoint/><geom type="plane" size="1 1 1"/><geom type="mesh" mesh="m" shellinertia="true"/></body></worldbody></mujoco>"#,
        "plane only allowed in static bodies",
        "plane only allowed in static bodies (geom #0 in body #1)",
    ),
    (
        "an SDF geom's shellinertia and its zaxis too small",
        r#"<mujoco><asset><mesh name="m" vertex="-0.5 -0.5 -0.5  0.5 -0.5 -0.5  0.5 0.5 -0.5  -0.5 0.5 -0.5  -0.5 -0.5 0.5  0.5 -0.5 0.5  0.5 0.5 0.5  -0.5 0.5 0.5" face="0 2 1  0 3 2  4 5 6  4 6 7  0 1 5  0 5 4  2 3 7  2 7 6  0 4 7  0 7 3  1 2 6  1 6 5"/></asset><worldbody><body><freejoint/><geom type="sdf" mesh="m" shellinertia="true" zaxis="0 0 0"/></body></worldbody></mujoco>"#,
        "orientation specification error 'zaxis too small' in geom 0",
        "invalid zaxis on <geom>: zaxis too small (geom #0 in body #1)",
    ),
    (
        "an earlier geom's condim, then an SDF geom's shellinertia",
        r#"<mujoco><asset><mesh name="m" vertex="-0.5 -0.5 -0.5  0.5 -0.5 -0.5  0.5 0.5 -0.5  -0.5 0.5 -0.5  -0.5 -0.5 0.5  0.5 -0.5 0.5  0.5 0.5 0.5  -0.5 0.5 0.5" face="0 2 1  0 3 2  4 5 6  4 6 7  0 1 5  0 5 4  2 3 7  2 7 6  0 4 7  0 7 3  1 2 6  1 6 5"/></asset><worldbody><body><freejoint/><geom size=".1" condim="2"/><geom type="sdf" mesh="m" shellinertia="true"/></body></worldbody></mujoco>"#,
        "invalid condim in geom",
        "invalid condim in geom (geom #0 in body #1)",
    ),
    (
        "an earlier body's plane, then an SDF geom's shellinertia",
        r#"<mujoco><asset><mesh name="m" vertex="-0.5 -0.5 -0.5  0.5 -0.5 -0.5  0.5 0.5 -0.5  -0.5 0.5 -0.5  -0.5 -0.5 0.5  0.5 -0.5 0.5  0.5 0.5 0.5  -0.5 0.5 0.5" face="0 2 1  0 3 2  4 5 6  4 6 7  0 1 5  0 5 4  2 3 7  2 7 6  0 4 7  0 7 3  1 2 6  1 6 5"/></asset><worldbody><body name="a"><freejoint/><geom type="plane" size="1 1 1"/></body><body><freejoint/><geom type="sdf" mesh="m" shellinertia="true"/></body></worldbody></mujoco>"#,
        "plane only allowed in static bodies",
        "plane only allowed in static bodies (geom #0 in body 'a')",
    ),
    (
        "an SDF geom's shellinertia and its user over nuser_geom",
        r#"<mujoco><size nuser_geom="1"/><asset><mesh name="m" vertex="-0.5 -0.5 -0.5  0.5 -0.5 -0.5  0.5 0.5 -0.5  -0.5 0.5 -0.5  -0.5 -0.5 0.5  0.5 -0.5 0.5  0.5 0.5 0.5  -0.5 0.5 0.5" face="0 2 1  0 3 2  4 5 6  4 6 7  0 1 5  0 5 4  2 3 7  2 7 6  0 4 7  0 7 3  1 2 6  1 6 5"/></asset><worldbody><body><freejoint/><geom type="sdf" mesh="m" shellinertia="true" user="1 2"/></body></worldbody></mujoco>"#,
        "user has more values than nuser_geom in geom '' (id = 0)",
        "invalid user on <geom>: user data exceeds nuser_geom=1",
    ),
    (
        "an earlier geom's negative mass, then an SDF geom's shellinertia",
        r#"<mujoco><asset><mesh name="m" vertex="-0.5 -0.5 -0.5  0.5 -0.5 -0.5  0.5 0.5 -0.5  -0.5 0.5 -0.5  -0.5 -0.5 0.5  0.5 -0.5 0.5  0.5 0.5 0.5  -0.5 0.5 0.5" face="0 2 1  0 3 2  4 5 6  4 6 7  0 1 5  0 5 4  2 3 7  2 7 6  0 4 7  0 7 3  1 2 6  1 6 5"/></asset><worldbody><body><freejoint/><geom size=".1" mass="-1"/><geom type="sdf" mesh="m" shellinertia="true"/></body></worldbody></mujoco>"#,
        "mass, inertia or density are negative in geom",
        "mass, inertia or density are negative in geom (geom #0 in body #1)",
    ),
    (
        "a mesh too small in volume, then a sphere fitted to a mesh",
        r#"<mujoco><asset><mesh name="flat" vertex="0 0 0 1 0 0 0 1 0 0 0 1" face="0 1 2 0 1 2 0 1 2 0 1 2"/><mesh name="m" vertex="-0.5 -0.5 -0.5  0.5 -0.5 -0.5  0.5 0.5 -0.5  -0.5 0.5 -0.5  -0.5 -0.5 0.5  0.5 -0.5 0.5  0.5 0.5 0.5  -0.5 0.5 0.5" face="0 2 1  0 3 2  4 5 6  4 6 7  0 1 5  0 5 4  2 3 7  2 7 6  0 4 7  0 7 3  1 2 6  1 6 5"/></asset><worldbody><body name="b"><freejoint/><geom type="mesh" mesh="flat" contype="0" conaffinity="0"/><geom size=".1"/></body><body name="c"><freejoint/><geom type="sphere" mesh="m"/></body></worldbody></mujoco>"#,
        "mesh volume is too small: flat . Try setting inertia to shell",
        "mesh 'flat': mesh volume is too small. Try setting inertia to shell",
    ),
    (
        "an exact mesh's faces turned, then a sphere fitted to a mesh",
        r#"<mujoco><asset><mesh name="turn" inertia="exact" vertex="-0.5 -0.5 -0.5  0.5 -0.5 -0.5  0.5 0.5 -0.5  -0.5 0.5 -0.5  -0.5 -0.5 0.5  0.5 -0.5 0.5  0.5 0.5 0.5  -0.5 0.5 0.5" face="0 1 2  0 3 2  4 5 6  4 6 7  0 1 5  0 5 4  2 3 7  2 7 6  0 4 7  0 7 3  1 2 6  1 6 5"/><mesh name="m" vertex="-0.5 -0.5 -0.5  0.5 -0.5 -0.5  0.5 0.5 -0.5  -0.5 0.5 -0.5  -0.5 -0.5 0.5  0.5 -0.5 0.5  0.5 0.5 0.5  -0.5 0.5 0.5" face="0 2 1  0 3 2  4 5 6  4 6 7  0 1 5  0 5 4  2 3 7  2 7 6  0 4 7  0 7 3  1 2 6  1 6 5"/></asset><worldbody><body name="b"><freejoint/><geom type="mesh" mesh="turn" contype="0" conaffinity="0"/><geom size=".1"/></body><body name="c"><freejoint/><geom type="sphere" mesh="m"/></body></worldbody></mujoco>"#,
        "faces of mesh 'turn' have inconsistent orientation. Please check the faces containing the vertices 1 and 2.",
        "mesh 'turn': its faces have inconsistent orientation. Please check the faces containing the vertices 1 and 2.",
    ),
    (
        "a negative face id, then a maxhullvert below 4",
        r#"<mujoco><asset><mesh name="m" vertex="0 0 0 1 0 0 0 1 0 0 0 1" face="0 2 1 0 1 -3 0 3 2 1 2 3" maxhullvert="2"/></asset><worldbody><geom type="mesh" mesh="m"/></worldbody></mujoco>"#,
        "maxhullvert must be larger than 3",
        "invalid maxhullvert on <mesh>: maxhullvert must be larger than 3 (line 1: asset > mesh 'm')",
    ),
    (
        "a negative face id, then an inertia keyword",
        r#"<mujoco><asset><mesh name="m" vertex="0 0 0 1 0 0 0 1 0 0 0 1" face="0 2 1 0 1 -3 0 3 2 1 2 3" inertia="bogus"/></asset><worldbody><geom type="mesh" mesh="m"/></worldbody></mujoco>"#,
        "invalid keyword: 'bogus'",
        "invalid keyword \"bogus\" in attribute 'inertia' on <mesh> (line 1: asset > mesh 'm')",
    ),
    (
        "a repeated mesh name, then the second mesh's face of four numbers",
        r#"<mujoco><asset><mesh name="m" vertex="0 0 0  1 0 0  0 2 0  0 0 3" face="0 2 1  0 1 3  0 3 2  1 2 3"/><mesh name="m" vertex="0 0 0  1 0 0  0 2 0  0 0 3" face="0 2 1 3"/></asset><worldbody><body name="b" pos="0 0 1"><freejoint/><geom type="mesh" mesh="m"/></body></worldbody></mujoco>"#,
        "repeated name 'm' in mesh",
        "repeated name 'm' in mesh",
    ),
    (
        "an SDF geom naming no mesh with a condim of 2",
        r#"<mujoco><asset><mesh name="m" vertex="0 0 0  1 0 0  0 2 0  0 0 3" face="0 2 1  0 1 3  0 3 2  1 2 3"/></asset><worldbody><body name="b"><freejoint/><geom type="sdf" condim="2"/><geom size=".1"/></body></worldbody></mujoco>"#,
        "invalid condim in geom",
        "invalid condim in geom (geom #0 in body 'b')",
    ),
    (
        "a mesh too small in volume, then an SDF geom naming no mesh",
        r#"<mujoco><asset><mesh name="flat" vertex="0 0 0 1 0 0 0 1 0 0 0 1" face="0 1 2 0 1 2 0 1 2 0 1 2"/></asset><worldbody><body name="b"><freejoint/><geom type="sdf"/><geom size=".1"/></body></worldbody></mujoco>"#,
        "mesh volume is too small: flat . Try setting inertia to shell",
        "mesh 'flat': mesh volume is too small. Try setting inertia to shell",
    ),
    (
        "a repeated mesh name, then a geom of an unknown class",
        r#"<mujoco><asset><mesh name="m" vertex="0 0 0  1 0 0  0 2 0  0 0 3" face="0 2 1  0 1 3  0 3 2  1 2 3"/><mesh name="m" vertex="0 0 0  1 0 0  0 2 0  0 0 3" face="0 2 1  0 1 3  0 3 2  1 2 3"/></asset><worldbody><body name="b" pos="0 0 1"><freejoint/><geom type="mesh" mesh="m"/></body><geom class="nope" size=".1"/></worldbody></mujoco>"#,
        "repeated name 'm' in mesh",
        "repeated name 'm' in mesh",
    ),
    (
        "a repeated height-field name, then a geom of an unknown class",
        r#"<mujoco><asset><mesh name="m" vertex="0 0 0  1 0 0  0 2 0  0 0 3" face="0 2 1  0 1 3  0 3 2  1 2 3"/><hfield name="h" nrow="2" ncol="2" elevation="0 0 0 0" size="1 1 .1 .1"/><hfield name="h" nrow="2" ncol="2" elevation="0 0 0 0" size="1 1 .1 .1"/></asset><worldbody><body name="b" pos="0 0 1"><freejoint/><geom type="mesh" mesh="m"/></body><geom class="nope" size=".1"/></worldbody></mujoco>"#,
        "repeated name 'h' in hfield",
        "repeated name 'h' in hfield",
    ),
    (
        "a mesh's vertex count, then a repeated geom name",
        r#"<mujoco><asset><mesh name="m" vertex="0 0 0  1 0 0  0 2 0  0 0 3 5" face="0 2 1  0 1 3  0 3 2  1 2 3"/></asset><worldbody><body name="b" pos="0 0 1"><freejoint/><geom name="g" type="mesh" mesh="m"/><geom name="g" size=".1"/></body></worldbody></mujoco>"#,
        "repeated name 'g' in geom",
        "repeated name 'g' in geom (geom 'g' in body 'b')",
    ),
    (
        "a normal that does not read beside a maxhullvert of 2",
        r#"<mujoco><asset><mesh name="m" vertex="0 0 0  1 0 0  0 2 0  0 0 3" face="0 2 1  0 1 3  0 3 2  1 2 3" normal="x" maxhullvert="2"/></asset><worldbody><body name="b" pos="0 0 1"><freejoint/><geom type="mesh" mesh="m"/></body></worldbody></mujoco>"#,
        "maxhullvert must be larger than 3",
        "invalid maxhullvert on <mesh>: maxhullvert must be larger than 3 (line 1: asset > mesh 'm')",
    ),
    (
        "a texcoord that does not read beside a maxhullvert of 2",
        r#"<mujoco><asset><mesh name="m" vertex="0 0 0  1 0 0  0 2 0  0 0 3" face="0 2 1  0 1 3  0 3 2  1 2 3" texcoord="x" maxhullvert="2"/></asset><worldbody><body name="b" pos="0 0 1"><freejoint/><geom type="mesh" mesh="m"/></body></worldbody></mujoco>"#,
        "maxhullvert must be larger than 3",
        "invalid maxhullvert on <mesh>: maxhullvert must be larger than 3 (line 1: asset > mesh 'm')",
    ),
    (
        "a normal that does not read beside an unknown inertia keyword",
        r#"<mujoco><asset><mesh name="m" vertex="0 0 0  1 0 0  0 2 0  0 0 3" face="0 2 1  0 1 3  0 3 2  1 2 3" normal="x" inertia="nope"/></asset><worldbody><body name="b" pos="0 0 1"><freejoint/><geom type="mesh" mesh="m"/></body></worldbody></mujoco>"#,
        "invalid keyword: 'nope'",
        "invalid keyword \"nope\" in attribute 'inertia' on <mesh> (line 1: asset > mesh 'm')",
    ),
    (
        "a face number that does not read beside a maxhullvert of 2",
        r#"<mujoco><asset><mesh name="m" vertex="0 0 0  1 0 0  0 2 0  0 0 3" face="0 2 x" maxhullvert="2"/></asset><worldbody><body name="b" pos="0 0 1"><freejoint/><geom type="mesh" mesh="m"/></body></worldbody></mujoco>"#,
        "maxhullvert must be larger than 3",
        "invalid maxhullvert on <mesh>: maxhullvert must be larger than 3 (line 1: asset > mesh 'm')",
    ),
    (
        "a vertex number that does not read beside an unknown inertia keyword",
        r#"<mujoco><asset><mesh name="m" vertex="0 0 a  1 0 0  0 2 0  0 0 3" face="0 2 1  0 1 3  0 3 2  1 2 3" inertia="bogus"/></asset><worldbody><body name="b" pos="0 0 1"><freejoint/><geom type="mesh" mesh="m"/></body></worldbody></mujoco>"#,
        "invalid keyword: 'bogus'",
        "invalid keyword \"bogus\" in attribute 'inertia' on <mesh> (line 1: asset > mesh 'm')",
    ),
    (
        "a body's user data beside its SDF geom naming no mesh",
        r#"<mujoco><size nuser_body="1"/><worldbody><body name="b" pos="0 0 1" user="1 2"><freejoint/><geom type="sdf"/></body></worldbody></mujoco>"#,
        "user has more values than nuser_body in body",
        "invalid user on <body>: user data exceeds nuser_body=1",
    ),
    (
        "a joint's zero axis, then a later body's SDF geom naming no mesh",
        r#"<mujoco><worldbody><body name="a" pos="0 0 1"><joint type="hinge" axis="0 0 0"/><geom size=".1"/></body><body name="b" pos="1 0 1"><freejoint/><geom type="sdf"/></body></worldbody></mujoco>"#,
        "axis too small in joint",
        "invalid axis on <joint>: axis too small (joint 'a_joint0' in body 'a')",
    ),
    (
        "an inertial's degenerate xyaxes beside an SDF geom naming no mesh",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><freejoint/><inertial pos="0 0 0" mass="1" diaginertia="1 1 1" xyaxes="1 0 0 1 0 0"/><geom type="sdf"/></body></worldbody></mujoco>"#,
        "error 'yaxis too small' in inertia alternative",
        "error 'yaxis too small' in inertia alternative (body 'b')",
    ),
    (
        "a plugin's repeated instance name beside an unnamed mesh",
        r#"<mujoco><extension><plugin plugin="mujoco.elasticity.cable"><instance name="p"/><instance name="p"/></plugin></extension><asset><mesh vertex="0 0 0  1 0 0  0 2 0  0 0 3" face="0 2 1  0 1 3  0 3 2  1 2 3"/></asset><worldbody><geom size=".1"/></worldbody></mujoco>"#,
        "empty name in mesh",
        "empty name in mesh (mesh #0)",
    ),
    (
        "a plugin's repeated instance name beside an unnamed height field",
        r#"<mujoco><extension><plugin plugin="mujoco.elasticity.cable"><instance name="p"/><instance name="p"/></plugin></extension><asset><hfield nrow="2" ncol="2" elevation="0 0 0 0" size="1 1 .1 .1"/></asset><worldbody><geom size=".1"/></worldbody></mujoco>"#,
        "empty name in height field",
        "empty name in height field (hfield #0)",
    ),
    (
        "two flexcomps of one name beside an unnamed mesh",
        r#"<mujoco><asset><mesh vertex="0 0 0  1 0 0  0 2 0  0 0 3" face="0 2 1  0 1 3  0 3 2  1 2 3"/></asset><worldbody><flexcomp name="f" type="grid" count="2 2 1" spacing=".1 .1 .1" dim="2"><edge equality="false"/></flexcomp><body name="x" pos="1 0 0"><flexcomp name="f" type="grid" count="2 2 1" spacing=".1 .1 .1" dim="2"><edge equality="false"/></flexcomp></body></worldbody></mujoco>"#,
        "empty name in mesh",
        "empty name in mesh (mesh #0)",
    ),
    (
        "a body of the name a flexcomp gives its body, beside an unnamed mesh",
        r#"<mujoco><asset><mesh vertex="0 0 0  1 0 0  0 2 0  0 0 3" face="0 2 1  0 1 3  0 3 2  1 2 3"/></asset><worldbody><body name="f_0"/><flexcomp name="f" type="grid" count="2 2 1" spacing=".1 .1 .1" dim="2"><edge equality="false"/></flexcomp></worldbody></mujoco>"#,
        "empty name in mesh",
        "empty name in mesh (mesh #0)",
    ),
    (
        "a box's fromto naming a mesh beside an unnamed mesh",
        r#"<mujoco><asset><mesh vertex="0 0 0  1 0 0  0 2 0  0 0 3" face="0 2 1  0 1 3  0 3 2  1 2 3"/></asset><worldbody><body name="b" pos="0 0 1"><freejoint/><geom type="box" size=".1" fromto="0 0 0 0 0 1" mesh="nope"/></body></worldbody></mujoco>"#,
        "empty name in mesh",
        "empty name in mesh (mesh #0)",
    ),
    (
        "a capsule's fromto of no length beside an unnamed mesh",
        r#"<mujoco><asset><mesh vertex="0 0 0  1 0 0  0 2 0  0 0 3" face="0 2 1  0 1 3  0 3 2  1 2 3"/></asset><worldbody><body name="b" pos="0 0 1"><freejoint/><geom type="capsule" size=".1" fromto="0 0 0 0 0 0"/></body></worldbody></mujoco>"#,
        "empty name in mesh",
        "empty name in mesh (mesh #0)",
    ),
    (
        "a box's fromto naming a mesh beside two meshes one file names",
        r#"<mujoco><asset><mesh file="tet.stl"/><mesh file="tet.stl"/></asset><worldbody><body name="b" pos="0 0 1"><freejoint/><geom type="box" size=".1" fromto="0 0 0 0 0 1" mesh="nope"/></body></worldbody></mujoco>"#,
        "repeated name 'tet' in mesh",
        "repeated name 'tet' in mesh",
    ),
    (
        "a cable's fromto of no length beside an unnamed mesh",
        r#"<mujoco><extension><plugin plugin="mujoco.elasticity.cable"/></extension><asset><mesh vertex="0 0 0  1 0 0  0 2 0  0 0 3" face="0 2 1  0 1 3  0 3 2  1 2 3"/></asset><worldbody><composite prefix="c" type="cable" count="4 1 1" size="1"><geom type="capsule" size="0.01"/></composite></worldbody></mujoco>"#,
        "empty name in mesh",
        "empty name in mesh (mesh #0)",
    ),
    (
        "a plugin's repeated instance name beside two meshes one file names",
        r#"<mujoco><extension><plugin plugin="mujoco.elasticity.cable"><instance name="p"/><instance name="p"/></plugin></extension><asset><mesh file="tet.stl"/><mesh file="tet.stl"/></asset><worldbody><geom size=".1"/></worldbody></mujoco>"#,
        "repeated name 'tet' in mesh",
        "repeated name 'tet' in mesh",
    ),
    (
        "a mesh's scale and refpos that do not read",
        r#"<mujoco><asset><mesh name="m" vertex="0 0 0  1 0 0  0 2 0  0 0 3" face="0 2 1  0 1 3  0 3 2  1 2 3" scale="x 1 1" refpos="y 0 0"/></asset><worldbody><body name="b" pos="0 0 1"><freejoint/><geom type="mesh" mesh="m"/></body></worldbody></mujoco>"#,
        "bad format in attribute 'refpos'",
        "bad number \"y\" in attribute 'refpos' on <mesh> (line 1: asset > mesh 'm')",
    ),
    (
        "a mesh's inertia keyword and maxhullvert both bad",
        r#"<mujoco><asset><mesh name="m" vertex="0 0 0  1 0 0  0 2 0  0 0 3" face="0 2 1  0 1 3  0 3 2  1 2 3" inertia="nope" maxhullvert="2"/></asset><worldbody><body name="b" pos="0 0 1"><freejoint/><geom type="mesh" mesh="m"/></body></worldbody></mujoco>"#,
        "invalid keyword: 'nope'",
        "invalid keyword \"nope\" in attribute 'inertia' on <mesh> (line 1: asset > mesh 'm')",
    ),
    (
        "an unnamed mesh, then a repeated mesh name",
        r#"<mujoco><asset><mesh vertex="0 0 0  1 0 0  0 2 0  0 0 3" face="0 2 1  0 1 3  0 3 2  1 2 3"/><mesh name="m" vertex="0 0 0  1 0 0  0 2 0  0 0 3" face="0 2 1  0 1 3  0 3 2  1 2 3"/><mesh name="m" vertex="0 0 0  1 0 0  0 2 0  0 0 3" face="0 2 1  0 1 3  0 3 2  1 2 3"/></asset><worldbody><geom size=".1"/></worldbody></mujoco>"#,
        "repeated name 'm' in mesh",
        "repeated name 'm' in mesh",
    ),
    (
        "a geom's condim then a height field's top of 0",
        r#"<mujoco><asset><hfield name="h" nrow="2" ncol="2" size="1 1 0 0.1"/></asset><worldbody><geom name="floor" type="plane" size="1 1 0.1"/><geom type="hfield" hfield="h" pos="3 0 0"/><body name="a" pos="0 0 0.09" ><freejoint/><geom name="ga" type="sphere" size="0.1" condim="2"/></body><body name="b" pos="0.5 0 0.09"><freejoint/><geom name="gb" type="box" size="0.1 0.1 0.1"/></body></worldbody><contact><pair geom1="ga" geom2="floor"/><pair geom1="gb" geom2="floor"/></contact></mujoco>"#,
        "invalid condim in geom",
        "invalid condim in geom (geom 'ga' in body 'a')",
    ),
    (
        "a pair's condim then a height field's top of 0",
        r#"<mujoco><asset><hfield name="h" nrow="2" ncol="2" size="1 1 0 0.1"/></asset><worldbody><geom name="floor" type="plane" size="1 1 0.1"/><geom type="hfield" hfield="h" pos="3 0 0"/><body name="a" pos="0 0 0.09" ><freejoint/><geom name="ga" type="sphere" size="0.1"/></body><body name="b" pos="0.5 0 0.09"><freejoint/><geom name="gb" type="box" size="0.1 0.1 0.1"/></body></worldbody><contact><pair geom1="ga" geom2="floor" condim="2"/><pair geom1="gb" geom2="floor"/></contact></mujoco>"#,
        "invalid condim in contact pair",
        "invalid condim in contact pair (pair #0)",
    ),
    (
        "a height field with no data beside actuator ctrlrange",
        r#"<mujoco model="f"><asset><hfield name="h" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom name="g" size="0.1"/><site name="s"/></body></worldbody><actuator><motor joint="j" ctrllimited="true" ctrlrange="1 0"/></actuator></mujoco>"#,
        "hfield not specified",
        "hfield not specified (hfield 'h')",
    ),
    (
        "a height field with no data beside equality body",
        r#"<mujoco model="f"><asset><hfield name="h" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom name="g" size="0.1"/><site name="s"/></body></worldbody><equality><weld body1="nope"/></equality></mujoco>"#,
        "hfield not specified",
        "hfield not specified (hfield 'h')",
    ),
    (
        "a height field with no data beside equality joint",
        r#"<mujoco model="f"><asset><hfield name="h" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom name="g" size="0.1"/><site name="s"/></body></worldbody><equality><joint joint1="nope"/></equality></mujoco>"#,
        "hfield not specified",
        "hfield not specified (hfield 'h')",
    ),
    (
        "a height field with no data beside exclude body",
        r#"<mujoco model="f"><asset><hfield name="h" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom name="g" size="0.1"/><site name="s"/></body></worldbody><contact><exclude body1="nope" body2="b"/></contact></mujoco>"#,
        "hfield not specified",
        "hfield not specified (hfield 'h')",
    ),
    (
        "a height field with no data beside geom hfield name",
        r#"<mujoco model="f"><asset><hfield name="h" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom name="g" size="0.1"/><site name="s"/></body><geom type="hfield" hfield="nope"/></worldbody></mujoco>"#,
        "hfield 'nope' not found in geom 1",
        "unknown hfield 'nope' (geom #1 in body 'world')",
    ),
    (
        "a height field with no data beside joint axis",
        r#"<mujoco model="f"><asset><hfield name="h" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom name="g" size="0.1"/><site name="s"/></body><body pos="2 0 1"><joint type="hinge" axis="0 0 0"/><geom size="0.1"/></body></worldbody></mujoco>"#,
        "axis too small in joint",
        "invalid axis on <joint>: axis too small (joint #1 in body #2)",
    ),
    (
        "a height field with no data beside key qpos",
        r#"<mujoco model="f"><asset><hfield name="h" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom name="g" size="0.1"/><site name="s"/></body></worldbody><keyframe><key qpos="1 2 3"/></keyframe></mujoco>"#,
        "hfield not specified",
        "hfield not specified (hfield 'h')",
    ),
    (
        "a height field with no data beside light target",
        r#"<mujoco model="f"><asset><hfield name="h" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom name="g" size="0.1"/><site name="s"/></body><light mode="targetbody" target="nope"/></worldbody></mujoco>"#,
        "unknown target body in light",
        "unknown target body in light (body 'world')",
    ),
    (
        "a height field with no data beside material texture",
        r#"<mujoco model="f"><asset><hfield name="h" size="1 1 0.5 0.1"/><material name="m" texture="nope"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom name="g" size="0.1"/><site name="s"/></body></worldbody></mujoco>"#,
        "hfield not specified",
        "hfield not specified (hfield 'h')",
    ),
    (
        "a height field with no data beside pair geom",
        r#"<mujoco model="f"><asset><hfield name="h" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom name="g" size="0.1"/><site name="s"/></body></worldbody><contact><pair geom1="nope" geom2="g"/></contact></mujoco>"#,
        "geom 'nope' not found in collision 0",
        "unknown geom 'nope' (pair #0)",
    ),
    (
        "a height field with no data beside sensor joint",
        r#"<mujoco model="f"><asset><hfield name="h" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom name="g" size="0.1"/><site name="s"/></body></worldbody><sensor><jointpos joint="nope"/></sensor></mujoco>"#,
        "hfield not specified",
        "hfield not specified (hfield 'h')",
    ),
    (
        "a height field with no data beside sensor noise",
        r#"<mujoco model="f"><asset><hfield name="h" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom name="g" size="0.1"/><site name="s"/></body></worldbody><sensor><jointpos joint="j" noise="-1"/></sensor></mujoco>"#,
        "hfield not specified",
        "hfield not specified (hfield 'h')",
    ),
    (
        "a height field with no data beside sensor site",
        r#"<mujoco model="f"><asset><hfield name="h" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom name="g" size="0.1"/><site name="s"/></body></worldbody><sensor><accelerometer site="nope"/></sensor></mujoco>"#,
        "hfield not specified",
        "hfield not specified (hfield 'h')",
    ),
    (
        "a height field's top of 0 beside actuator ctrlrange",
        r#"<mujoco model="f"><asset><hfield name="h" nrow="2" ncol="2" size="1 1 0 0.1" elevation="0 0 1 1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom name="g" size="0.1"/><site name="s"/></body></worldbody><actuator><motor joint="j" ctrllimited="true" ctrlrange="1 0"/></actuator></mujoco>"#,
        "size parameter is not positive in hfield",
        "size parameter is not positive in hfield (hfield 'h')",
    ),
    (
        "a height field's top of 0 beside equality body",
        r#"<mujoco model="f"><asset><hfield name="h" nrow="2" ncol="2" size="1 1 0 0.1" elevation="0 0 1 1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom name="g" size="0.1"/><site name="s"/></body></worldbody><equality><weld body1="nope"/></equality></mujoco>"#,
        "size parameter is not positive in hfield",
        "size parameter is not positive in hfield (hfield 'h')",
    ),
    (
        "a height field's top of 0 beside equality joint",
        r#"<mujoco model="f"><asset><hfield name="h" nrow="2" ncol="2" size="1 1 0 0.1" elevation="0 0 1 1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom name="g" size="0.1"/><site name="s"/></body></worldbody><equality><joint joint1="nope"/></equality></mujoco>"#,
        "size parameter is not positive in hfield",
        "size parameter is not positive in hfield (hfield 'h')",
    ),
    (
        "a height field's top of 0 beside exclude body",
        r#"<mujoco model="f"><asset><hfield name="h" nrow="2" ncol="2" size="1 1 0 0.1" elevation="0 0 1 1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom name="g" size="0.1"/><site name="s"/></body></worldbody><contact><exclude body1="nope" body2="b"/></contact></mujoco>"#,
        "size parameter is not positive in hfield",
        "size parameter is not positive in hfield (hfield 'h')",
    ),
    (
        "a height field's top of 0 beside geom hfield name",
        r#"<mujoco model="f"><asset><hfield name="h" nrow="2" ncol="2" size="1 1 0 0.1" elevation="0 0 1 1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom name="g" size="0.1"/><site name="s"/></body><geom type="hfield" hfield="nope"/></worldbody></mujoco>"#,
        "hfield 'nope' not found in geom 1",
        "unknown hfield 'nope' (geom #1 in body 'world')",
    ),
    (
        "a height field's top of 0 beside joint axis",
        r#"<mujoco model="f"><asset><hfield name="h" nrow="2" ncol="2" size="1 1 0 0.1" elevation="0 0 1 1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom name="g" size="0.1"/><site name="s"/></body><body pos="2 0 1"><joint type="hinge" axis="0 0 0"/><geom size="0.1"/></body></worldbody></mujoco>"#,
        "axis too small in joint",
        "invalid axis on <joint>: axis too small (joint #1 in body #2)",
    ),
    (
        "a height field's top of 0 beside key qpos",
        r#"<mujoco model="f"><asset><hfield name="h" nrow="2" ncol="2" size="1 1 0 0.1" elevation="0 0 1 1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom name="g" size="0.1"/><site name="s"/></body></worldbody><keyframe><key qpos="1 2 3"/></keyframe></mujoco>"#,
        "size parameter is not positive in hfield",
        "size parameter is not positive in hfield (hfield 'h')",
    ),
    (
        "a height field's top of 0 beside light target",
        r#"<mujoco model="f"><asset><hfield name="h" nrow="2" ncol="2" size="1 1 0 0.1" elevation="0 0 1 1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom name="g" size="0.1"/><site name="s"/></body><light mode="targetbody" target="nope"/></worldbody></mujoco>"#,
        "unknown target body in light",
        "unknown target body in light (body 'world')",
    ),
    (
        "a height field's top of 0 beside material texture",
        r#"<mujoco model="f"><asset><hfield name="h" nrow="2" ncol="2" size="1 1 0 0.1" elevation="0 0 1 1"/><material name="m" texture="nope"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom name="g" size="0.1"/><site name="s"/></body></worldbody></mujoco>"#,
        "size parameter is not positive in hfield",
        "size parameter is not positive in hfield (hfield 'h')",
    ),
    (
        "a height field's top of 0 beside pair geom",
        r#"<mujoco model="f"><asset><hfield name="h" nrow="2" ncol="2" size="1 1 0 0.1" elevation="0 0 1 1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom name="g" size="0.1"/><site name="s"/></body></worldbody><contact><pair geom1="nope" geom2="g"/></contact></mujoco>"#,
        "geom 'nope' not found in collision 0",
        "unknown geom 'nope' (pair #0)",
    ),
    (
        "a height field's top of 0 beside sensor joint",
        r#"<mujoco model="f"><asset><hfield name="h" nrow="2" ncol="2" size="1 1 0 0.1" elevation="0 0 1 1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom name="g" size="0.1"/><site name="s"/></body></worldbody><sensor><jointpos joint="nope"/></sensor></mujoco>"#,
        "size parameter is not positive in hfield",
        "size parameter is not positive in hfield (hfield 'h')",
    ),
    (
        "a height field's top of 0 beside sensor noise",
        r#"<mujoco model="f"><asset><hfield name="h" nrow="2" ncol="2" size="1 1 0 0.1" elevation="0 0 1 1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom name="g" size="0.1"/><site name="s"/></body></worldbody><sensor><jointpos joint="j" noise="-1"/></sensor></mujoco>"#,
        "size parameter is not positive in hfield",
        "size parameter is not positive in hfield (hfield 'h')",
    ),
    (
        "a height field's top of 0 beside sensor site",
        r#"<mujoco model="f"><asset><hfield name="h" nrow="2" ncol="2" size="1 1 0 0.1" elevation="0 0 1 1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom name="g" size="0.1"/><site name="s"/></body></worldbody><sensor><accelerometer site="nope"/></sensor></mujoco>"#,
        "size parameter is not positive in hfield",
        "size parameter is not positive in hfield (hfield 'h')",
    ),
    (
        "a height field with no data beside a ball joint's equality",
        r#"<mujoco><asset><hfield name="h" size="1 1 0.5 0.1"/></asset><worldbody><geom type="hfield" hfield="h"/><body name="b" pos="0 0 1"><joint name="ba" type="ball"/><geom size="0.1"/></body></worldbody><equality><joint joint1="ba"/></equality></mujoco>"#,
        "hfield not specified",
        "hfield not specified (hfield 'h')",
    ),
    (
        "a height field with no data beside a sensor naming no object, which MuJoCo's reader refuses",
        r#"<mujoco model="f"><asset><hfield name="h" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><camera name="c" pos="0 0 3"/><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom name="g" size="0.1"/><site name="s"/><body name="b2" pos="0 0 0.5"><joint name="j2" type="slide"/><geom name="g2" size="0.1"/><geom name="gx" type="box" size="0.1 0.1 0.1"/><site name="s2" pos="0.2 0 0"/><body name="b3" pos="0 0 0.5"><joint name="jb" type="ball"/><geom name="g3" size="0.1"/></body></body></body></worldbody><sensor><actuatorfrc/></sensor></mujoco>"#,
        "required attribute missing: 'actuator'",
        "required attribute missing: 'objname' on <sensor> (sensor #0)",
    ),
    (
        "a height field with no data beside a frame sensor's refname without a reftype, which MuJoCo's reader refuses",
        r#"<mujoco model="f"><asset><hfield name="h" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><camera name="c" pos="0 0 3"/><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom name="g" size="0.1"/><site name="s"/><body name="b2" pos="0 0 0.5"><joint name="j2" type="slide"/><geom name="g2" size="0.1"/><geom name="gx" type="box" size="0.1 0.1 0.1"/><site name="s2" pos="0.2 0 0"/><body name="b3" pos="0 0 0.5"><joint name="jb" type="ball"/><geom name="g3" size="0.1"/></body></body></body></worldbody><sensor><framepos objtype="body" objname="b" refname="b2"/></sensor></mujoco>"#,
        "refname 'b2' given but reftype is missing",
        "invalid refname on <sensor>: refname 'b2' given but reftype is missing (sensor #0)",
    ),
    (
        "a height field with no data beside a distance sensor naming neither geom1 nor body1, which MuJoCo's reader refuses",
        r#"<mujoco model="f"><asset><hfield name="h" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><camera name="c" pos="0 0 3"/><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom name="g" size="0.1"/><site name="s"/><body name="b2" pos="0 0 0.5"><joint name="j2" type="slide"/><geom name="g2" size="0.1"/><geom name="gx" type="box" size="0.1 0.1 0.1"/><site name="s2" pos="0.2 0 0"/><body name="b3" pos="0 0 0.5"><joint name="jb" type="ball"/><geom name="g3" size="0.1"/></body></body></body></worldbody><sensor><distance geom2="g2"/></sensor></mujoco>"#,
        "exactly one of (geom1, body1) must be specified",
        "distance sensor: exactly one of (geom1, body1) must be specified (sensor #0)",
    ),
    (
        "a height field with no data beside a distance sensor naming neither geom2 nor body2, which MuJoCo's reader refuses",
        r#"<mujoco model="f"><asset><hfield name="h" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><camera name="c" pos="0 0 3"/><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom name="g" size="0.1"/><site name="s"/><body name="b2" pos="0 0 0.5"><joint name="j2" type="slide"/><geom name="g2" size="0.1"/><geom name="gx" type="box" size="0.1 0.1 0.1"/><site name="s2" pos="0.2 0 0"/><body name="b3" pos="0 0 0.5"><joint name="jb" type="ball"/><geom name="g3" size="0.1"/></body></body></body></worldbody><sensor><distance geom1="g"/></sensor></mujoco>"#,
        "exactly one of (geom2, body2) must be specified",
        "distance sensor: exactly one of (geom2, body2) must be specified (sensor #0)",
    ),
    (
        "a height field with no data beside a rangefinder on a camera no element names, which MuJoCo compiles after the field",
        r#"<mujoco model="f"><asset><hfield name="h" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><camera name="c" pos="0 0 3"/><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom name="g" size="0.1"/><site name="s"/><body name="b2" pos="0 0 0.5"><joint name="j2" type="slide"/><geom name="g2" size="0.1"/><geom name="gx" type="box" size="0.1 0.1 0.1"/><site name="s2" pos="0.2 0 0"/><body name="b3" pos="0 0 0.5"><joint name="jb" type="ball"/><geom name="g3" size="0.1"/></body></body></body></worldbody><sensor><rangefinder camera="nope"/></sensor></mujoco>"#,
        "hfield not specified",
        "hfield not specified (hfield 'h')",
    ),
    (
        "a height field with no data beside a tendon's user data past nuser_tendon",
        r#"<mujoco><size nuser_tendon="1"/><asset><hfield name="h" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom name="g" size="0.1"/><site name="s"/></body></worldbody><tendon><fixed user="1 2"><joint joint="j" coef="1"/></fixed></tendon></mujoco>"#,
        "hfield not specified",
        "hfield not specified (hfield 'h')",
    ),
    (
        "a height field with no data beside an actuator's user data past nuser_actuator",
        r#"<mujoco><size nuser_actuator="1"/><asset><hfield name="h" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom name="g" size="0.1"/><site name="s"/></body></worldbody><actuator><motor joint="j" user="1 2"/></actuator></mujoco>"#,
        "hfield not specified",
        "hfield not specified (hfield 'h')",
    ),
    (
        "a height field with no data beside a sensor's user data past nuser_sensor",
        r#"<mujoco><size nuser_sensor="1"/><asset><hfield name="h" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom name="g" size="0.1"/><site name="s"/></body></worldbody><sensor><jointpos joint="j" user="1 2"/></sensor></mujoco>"#,
        "hfield not specified",
        "hfield not specified (hfield 'h')",
    ),
    (
        "an actuator's repeated name, which MuJoCo's reader reads before the sensors, beside a sensor naming no object",
        r#"<mujoco>
  <worldbody><body name="b"><joint name="j"/><geom name="g" size="0.1"/></body></worldbody>
  <actuator><motor name="a" joint="j"/><motor name="a" joint="j"/></actuator>
  <sensor><jointpos/></sensor>
</mujoco>"#,
        "repeated name 'a' in actuator",
        "repeated name 'a' in actuator",
    ),
    (
        "a height field's repeated name beside a sensor naming no object",
        r#"<mujoco>
  <asset><hfield name="h" nrow="2" ncol="2" size="1 1 1 0.1"/><hfield name="h" nrow="2" ncol="2" size="1 1 1 0.1"/></asset>
  <worldbody><body name="b"><joint name="j"/><geom name="g" size="0.1"/></body></worldbody>
  <sensor><jointpos/></sensor>
</mujoco>"#,
        "repeated name 'h' in hfield",
        "repeated name 'h' in hfield",
    ),
    (
        "a height field's elevation of the wrong length, written after the sensors, beside a sensor naming no object",
        r#"<mujoco>
  <worldbody><body name="b"><joint name="j"/><geom name="g" size="0.1"/></body></worldbody>
  <sensor><jointpos/></sensor>
  <asset><hfield name="h" nrow="2" ncol="2" size="1 1 1 0.1" elevation="1 2 3"/></asset>
</mujoco>"#,
        "elevation data length must match nrow*ncol",
        "invalid elevation on <hfield>: elevation data length must match nrow*ncol (line 4: asset > hfield 'h')",
    ),
    (
        "a sensor's repeated name beside a later sensor naming no object",
        r#"<mujoco>
  <worldbody><body name="b"><joint name="j"/><geom name="g" size="0.1"/></body></worldbody>
  <sensor><jointpos name="s" joint="j"/><jointpos name="s" joint="j"/><jointpos/></sensor>
</mujoco>"#,
        "repeated name 's' in sensor",
        "repeated name 's' in sensor",
    ),
    (
        "a sensor naming no object, which MuJoCo's reader refuses, beside an equality's unknown body",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom name="g" size="0.1"/><site name="s"/><site name="s2" pos="0.2 0 0"/></body></worldbody><equality><connect body1="nope" anchor="0 0 0"/></equality><sensor><jointpos/></sensor></mujoco>"#,
        "required attribute missing: 'joint'",
        "required attribute missing: 'objname' on <sensor> (sensor #0)",
    ),
    (
        "a sensor naming no object beside a spatial tendon wrapping a box",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom name="g" size="0.1"/><site name="s"/><site name="s2" pos="0.2 0 0"/></body><geom name="bx" type="box" size="0.1 0.1 0.1"/></worldbody><tendon><spatial><site site="s"/><geom geom="bx"/><site site="s2"/></spatial></tendon><sensor><jointpos/></sensor></mujoco>"#,
        "required attribute missing: 'joint'",
        "required attribute missing: 'objname' on <sensor> (sensor #0)",
    ),
    (
        "a sensor naming no object beside a mesh of three vertices",
        r#"<mujoco><asset><mesh name="m" vertex="0 0 0 1 0 0 0 1 0"/></asset><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom name="g" size="0.1"/><site name="s"/><site name="s2" pos="0.2 0 0"/></body><geom type="mesh" mesh="m"/></worldbody><sensor><jointpos/></sensor></mujoco>"#,
        "required attribute missing: 'joint'",
        "required attribute missing: 'objname' on <sensor> (sensor #0)",
    ),
    (
        "a sensor naming no object beside a body's axisangle of no axis",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom name="g" size="0.1"/><site name="s"/><site name="s2" pos="0.2 0 0"/></body><body axisangle="0 0 0 1"><geom size="0.1"/></body></worldbody><sensor><jointpos/></sensor></mujoco>"#,
        "required attribute missing: 'joint'",
        "required attribute missing: 'objname' on <sensor> (sensor #0)",
    ),
    (
        "a sensor naming no object beside an actuator's delay with no history",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom name="g" size="0.1"/><site name="s"/><site name="s2" pos="0.2 0 0"/></body></worldbody><actuator><motor joint="j" delay="0.1"/></actuator><sensor><jointpos/></sensor></mujoco>"#,
        "required attribute missing: 'joint'",
        "required attribute missing: 'objname' on <sensor> (sensor #0)",
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

/// Three faults: the tree's (a joint's axis, a light's target, a body's or a
/// geom's user data), which MuJoCo 3.5.0 meets compiling the bodies, a
/// height field's, and an element's MuJoCo compiles after the height fields,
/// each named as MuJoCo names it: the tree's, or a sensor's fault MuJoCo's
/// reader meets, before it compiles anything. `(what, document, MuJoCo's
/// reason, sim-mjcf's error for that fault)`, measured 2026-11-27 (RL-11b's
/// fix-diff passes, FA's, FB's and FC's documents).
const THREE_FAULTS: &[(&str, &str, &str, &str)] = &[
    (
        "a joint's axis of 0, a height field with no data and an equality's unknown body",
        r#"<mujoco><asset><hfield name="h" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom name="g" size="0.1"/><site name="s"/></body><body pos="2 0 1"><joint type="hinge" axis="0 0 0"/><geom size="0.1"/></body></worldbody><equality><weld body1="nope"/></equality></mujoco>"#,
        "axis too small in joint",
        "invalid axis on <joint>: axis too small (joint #1 in body #2)",
    ),
    (
        "a joint's axis of 0, a height field with no data and a sensor's unknown joint",
        r#"<mujoco><asset><hfield name="h" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom name="g" size="0.1"/><site name="s"/></body><body pos="2 0 1"><joint type="hinge" axis="0 0 0"/><geom size="0.1"/></body></worldbody><sensor><jointpos joint="nope"/></sensor></mujoco>"#,
        "axis too small in joint",
        "invalid axis on <joint>: axis too small (joint #1 in body #2)",
    ),
    (
        "a light's unknown target, a height field with no data and an exclude's unknown body",
        r#"<mujoco><asset><hfield name="h" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom name="g" size="0.1"/><site name="s"/></body><light mode="targetbody" target="nope"/></worldbody><contact><exclude body1="nope" body2="b"/></contact></mujoco>"#,
        "unknown target body in light",
        "unknown target body in light (body 'world')",
    ),
    (
        "a body's user data past nuser_body, a height field with no data and an equality's unknown body",
        r#"<mujoco><size nuser_body="1"/><asset><hfield name="h" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body name="b" pos="0 0 1" user="1 2"><joint name="j" type="hinge"/><geom name="g" size="0.1"/><site name="s"/></body></worldbody><equality><weld body1="nope"/></equality></mujoco>"#,
        "user has more values than nuser_body in body",
        "invalid user on <body>: user data exceeds nuser_body=1",
    ),
    (
        "a geom's user data past nuser_geom, a height field with no data and an actuator's delay with no history",
        r#"<mujoco model="f"><size nuser_geom="1"/><asset><hfield name="h" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><camera name="c" pos="0 0 3"/><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom name="g" size="0.1"/><site name="s"/><body name="b2" pos="0 0 0.5"><joint name="j2" type="slide"/><geom name="g2" size="0.1"/><geom name="gx" type="box" size="0.1 0.1 0.1"/><site name="s2" pos="0.2 0 0"/><body name="b3" pos="0 0 0.5"><joint name="jb" type="ball"/><geom name="g3" size="0.1"/></body></body><geom size="0.1" user="1 2"/></body></worldbody><actuator><motor joint="j" delay="0.1"/></actuator></mujoco>"#,
        "user has more values than nuser_geom in geom '' (id = 2)",
        "invalid user on <geom>: user data exceeds nuser_geom=1",
    ),
    (
        "a body's user data past nuser_body, a height field with no data and a sensor's delay with no history",
        r#"<mujoco model="f"><size nuser_body="1"/><asset><hfield name="h" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><camera name="c" pos="0 0 3"/><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom name="g" size="0.1"/><site name="s"/><body name="b2" pos="0 0 0.5"><joint name="j2" type="slide"/><geom name="g2" size="0.1"/><geom name="gx" type="box" size="0.1 0.1 0.1"/><site name="s2" pos="0.2 0 0"/><body name="b3" pos="0 0 0.5"><joint name="jb" type="ball"/><geom name="g3" size="0.1"/></body></body><body name="bu" user="1 2"><geom size="0.1"/></body></body></worldbody><sensor><jointpos joint="j" delay="0.1"/></sensor></mujoco>"#,
        "user has more values than nuser_body in body",
        "invalid user on <body>: user data exceeds nuser_body=1",
    ),
    (
        "a body's user data past nuser_body, a height field with no data and an exclude's unknown body",
        r#"<mujoco model="f"><size nuser_body="1"/><asset><hfield name="h" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><camera name="c" pos="0 0 3"/><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom name="g" size="0.1"/><site name="s"/><body name="b2" pos="0 0 0.5"><joint name="j2" type="slide"/><geom name="g2" size="0.1"/><geom name="gx" type="box" size="0.1 0.1 0.1"/><site name="s2" pos="0.2 0 0"/><body name="b3" pos="0 0 0.5"><joint name="jb" type="ball"/><geom name="g3" size="0.1"/></body></body><body name="bu" user="1 2"><geom size="0.1"/></body></body></worldbody><contact><exclude body1="b" body2="nope"/></contact></mujoco>"#,
        "user has more values than nuser_body in body",
        "invalid user on <body>: user data exceeds nuser_body=1",
    ),
    (
        "a joint's axis of 0, a height field with no data and a sensor naming no object, which MuJoCo's reader refuses before it compiles",
        r#"<mujoco><asset><hfield name="h" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom name="g" size="0.1"/><site name="s"/></body><body pos="2 0 1"><joint type="hinge" axis="0 0 0"/><geom size="0.1"/></body></worldbody><sensor><jointpos/></sensor></mujoco>"#,
        "required attribute missing: 'joint'",
        "required attribute missing: 'objname' on <sensor> (sensor #0)",
    ),
    (
        "a sensor naming no object, a height field with no data and a spatial tendon wrapping a box",
        r#"<mujoco><asset><hfield name="h" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom name="g" size="0.1"/><site name="s"/><site name="s2" pos="0.2 0 0"/></body><geom name="bx" type="box" size="0.1 0.1 0.1"/></worldbody><tendon><spatial><site site="s"/><geom geom="bx"/><site site="s2"/></spatial></tendon><sensor><jointpos/></sensor></mujoco>"#,
        "required attribute missing: 'joint'",
        "required attribute missing: 'objname' on <sensor> (sensor #0)",
    ),
];

#[test]
fn a_document_with_a_tree_fault_a_height_fields_and_a_later_one_is_refused_for_the_trees() {
    let mut missed = Vec::new();
    for &(what, xml, mujoco, expected) in THREE_FAULTS {
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
/// are applied, a body's orientation when the tree is built, and a pair
/// with no `geom1` while parsing (registry `D-PAIR-NO-GEOM`); MuJoCo 3.5.0
/// checks each condim and pair before these (ledger L353). `(what, document,
/// MuJoCo's reason, this crate's error)`, measured 2026-11-04 (the last three:
/// review round 1's).
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

/// Two faults, a sensor's MuJoCo 3.5.0's reader meets and one this crate
/// meets as it reads the document or in its front end, before it builds the
/// model (a repeated body or sensor name, a geom's malformed size, an
/// actuator's unknown joint, a mesh's vertex count), where this crate names
/// the other: MuJoCo's reader reads the sensors before the worldbody
/// (`xml_native_reader.cc:1064-1078`) and its compiler resolves names and
/// meshes once the document is read, and this crate names a read
/// document's sensor faults after its front end (ledger L353). `(what,
/// document, MuJoCo's reason, this crate's error)`, measured 2026-11-27
/// (RL-11b's second and third fix-diff passes, FC's and FE's documents).
const SENSOR_READ_COSTS: &[(&str, &str, &str, &str)] = &[
    (
        "a body's repeated name beside a sensor naming no object",
        r#"<mujoco>
  <worldbody><body name="b"><joint name="j"/><geom size="0.1"/></body><body name="b"><geom size="0.1"/></body></worldbody>
  <sensor><jointpos/></sensor>
</mujoco>"#,
        "required attribute missing: 'joint'",
        "repeated name 'b' in body",
    ),
    (
        "a geom's size that is not a number beside a sensor naming no object",
        r#"<mujoco>
  <worldbody><body name="b"><joint name="j"/><geom name="g" size="0.1"/><site name="s"/></body></worldbody>
  <sensor><jointpos/></sensor><worldbody><geom size="abc"/></worldbody>
</mujoco>"#,
        "required attribute missing: 'joint'",
        "bad number \"abc\" in attribute 'size' on <geom> (line 3: worldbody > geom)",
    ),
    (
        "a sensor naming no object beside an actuator's unknown joint, which this crate's validation resolves",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom name="g" size="0.1"/><site name="s"/><site name="s2" pos="0.2 0 0"/></body></worldbody><actuator><motor joint="nope"/></actuator><sensor><jointpos/></sensor></mujoco>"#,
        "required attribute missing: 'joint'",
        "unknown joint 'nope' (actuator #0)",
    ),
    (
        "a sensor naming no object beside a later sensor's repeated name",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom name="g" size="0.1"/><site name="s"/><site name="s2" pos="0.2 0 0"/></body></worldbody><sensor><jointpos/><jointpos name="x" joint="j"/><jointpos name="x" joint="j"/></sensor></mujoco>"#,
        "required attribute missing: 'joint'",
        "repeated name 'x' in sensor",
    ),
    (
        "a sensor naming no object beside a mesh whose vertex data is not a multiple of 3",
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom name="g" size="0.1"/><site name="s"/><site name="s2" pos="0.2 0 0"/></body></worldbody><sensor><jointpos/></sensor><asset><mesh name="m" vertex="0 0 0 1 0 0 0 1 0 0 0"/></asset></mujoco>"#,
        "required attribute missing: 'joint'",
        "vertex data must be a multiple of 3 (mesh 'm')",
    ),
];

#[test]
fn a_sensors_reader_fault_beside_a_worldbody_fault_is_named_as_ledgered() {
    let mut moved = Vec::new();
    for &(what, xml, _mujoco, ours) in SENSOR_READ_COSTS {
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

/// Two faults, one refused as MuJoCo 3.5.0's schema refuses it (as in
/// `SCHEMA_COSTS`), where MuJoCo names the other and this crate the refused
/// element or attribute, one row per kind (ledgers L315, L353).
/// `(what, document, MuJoCo's reason, this crate's error)`, measured
/// 2026-11-09 (RL-8c's review). Each of the other four attributes stands as
/// `density` does.
const REFUSAL_FIRST_COSTS: &[(&str, &str, &str, &str)] = &[
    (
        "an unknown element before a deformable flexcomp",
        r#"<mujoco><worldbody><body name="a"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body><body name="b" pos="1 0 0"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body><blob/></worldbody><deformable><flexcomp name="c" type="grid" count="2 2 1" spacing="0.1 0.1 0.1" dim="2"/></deformable></mujoco>"#,
        "Schema violation: unrecognized element; Element 'blob', line 1",
        "unrecognized element <flexcomp> in <deformable> (line 1: deformable > flexcomp 'c')",
    ),
    (
        "an unknown attribute on another element before a deformable flexcomp",
        r#"<mujoco><option blob="1"/><worldbody><body name="a"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body><body name="b" pos="1 0 0"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body></worldbody><deformable><flexcomp name="c" type="grid" count="2 2 1" spacing="0.1 0.1 0.1" dim="2"/></deformable></mujoco>"#,
        "Schema violation: unrecognized attribute: 'blob'; Element 'option', line 1",
        "unrecognized element <flexcomp> in <deformable> (line 1: deformable > flexcomp 'c')",
    ),
    (
        "an unknown attribute on the deformable holding a flexcomp",
        r#"<mujoco><worldbody><body name="a"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body><body name="b" pos="1 0 0"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body></worldbody><deformable blob="1"><flexcomp name="c" type="grid" count="2 2 1" spacing="0.1 0.1 0.1" dim="2"/></deformable></mujoco>"#,
        "Schema violation: unrecognized attribute: 'blob'; Element 'deformable', line 1",
        "unrecognized element <flexcomp> in <deformable> (line 1: deformable > flexcomp 'c')",
    ),
    (
        "a repeated unique child of a flex before a deformable flexcomp",
        r#"<mujoco><worldbody><body name="a"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body><body name="b" pos="1 0 0"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body></worldbody><deformable><flex name="f" dim="1" body="a b" element="0 1"><contact/><contact/></flex><flexcomp name="c" type="grid" count="2 2 1" spacing="0.1 0.1 0.1" dim="2"/></deformable></mujoco>"#,
        "Schema violation: unique element 'contact' found 2 times; Element 'flex', line 1",
        "unrecognized element <flexcomp> in <deformable> (line 1: deformable > flexcomp 'c')",
    ),
    (
        "an end tag closing another element, after a deformable flexcomp",
        r#"<mujoco><worldbody><body name="a"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body><body name="b" pos="1 0 0"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body></worldbody><deformable><flexcomp name="c" type="grid" count="2 2 1" spacing="0.1 0.1 0.1" dim="2"/></deformable><worldbody></mujoco>"#,
        "XML parse error 14: Error=XML_ERROR_MISMATCHED_ELEMENT ErrorID=14 (0xe) Line number=1: XMLElement name=worldbody",
        "unrecognized element <flexcomp> in <deformable> (line 1: deformable > flexcomp 'c')",
    ),
    (
        "an unknown element before a flex's density",
        r#"<mujoco><worldbody><body name="a"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body><body name="b" pos="1 0 0"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body><blob/></worldbody><deformable><flex name="f" dim="1" body="a b" element="0 1" density="10"/></deformable></mujoco>"#,
        "Schema violation: unrecognized element; Element 'blob', line 1",
        "unrecognized attribute 'density' on <flex> (line 1: deformable > flex 'f')",
    ),
    (
        "an unknown attribute of the same flex written before its density",
        r#"<mujoco><worldbody><body name="a"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body><body name="b" pos="1 0 0"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body></worldbody><deformable><flex name="f" dim="1" body="a b" element="0 1" blob="1" density="10"/></deformable></mujoco>"#,
        "Schema violation: unrecognized attribute: 'blob'; Element 'flex', line 1",
        "unrecognized attribute 'density' on <flex> (line 1: deformable > flex 'f')",
    ),
    (
        "an end tag closing another element, after a flex's density",
        r#"<mujoco><worldbody><body name="a"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body><body name="b" pos="1 0 0"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body></worldbody><deformable><flex name="f" dim="1" body="a b" element="0 1" density="10"/></deformable><worldbody></mujoco>"#,
        "XML parse error 14: Error=XML_ERROR_MISMATCHED_ELEMENT ErrorID=14 (0xe) Line number=1: XMLElement name=worldbody",
        "unrecognized attribute 'density' on <flex> (line 1: deformable > flex 'f')",
    ),
    (
        "an extension attribute this crate reads, before a deformable flexcomp",
        r#"<mujoco><worldbody><body name="a"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body><body name="b" pos="1 0 0"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body><body name="c" pos="0 1 0"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/></body></worldbody><deformable><flex name="f" dim="2" body="a b c" element="0 1 2"><elasticity elastic2d="bend" young="1e5" thickness="0.001" bending_model="bridson"/></flex><flexcomp name="c" type="grid" count="2 2 1" spacing="0.1 0.1 0.1" dim="2"/></deformable></mujoco>"#,
        "Schema violation: unrecognized attribute: 'bending_model'; Element 'elasticity', line 1",
        "unrecognized element <flexcomp> in <deformable> (line 1: deformable > flexcomp 'c')",
    ),
];

#[test]
fn a_fault_named_first_by_mujoco_beside_a_refusal_is_named_as_ledgered() {
    let mut moved = Vec::new();
    for &(what, xml, _mujoco, ours) in REFUSAL_FIRST_COSTS {
        match load_model(xml) {
            Err(e) if e.to_string() == ours => {}
            Err(e) => moved.push(format!("{what}: {e}")),
            Ok(_) => moved.push(format!("{what}: loads")),
        }
    }
    assert!(moved.is_empty(), "{}", moved.join("\n"));
}

/// Two faults, one an `<inertial>`'s orientation alternative, a body's
/// principal axes from its geoms or its non-finite inertia from them, where
/// this crate names the other one than MuJoCo 3.5.0 (ledger L353; the
/// non-finite rows' first fault is one MuJoCo loads, D-INERTIA-NONFINITE).
/// `(what, document, MuJoCo's reason, this crate's error)`, measured
/// 2026-11-09 (RL-9a's two-fault set and its review's pairs).
const INERTIAL_COSTS: &[(&str, &str, &str, &str)] = &[
    (
        "an inertial's degenerate xyaxes then its own body's geom's size",
        r#"<mujoco><worldbody><body name="n"><freejoint/><inertial pos="0 0 0" mass="1" diaginertia="1 1 1" xyaxes="1 0 0 1 0 0"/><geom type="box" size="0.1 0.1 0"/></body></worldbody></mujoco>"#,
        "error 'yaxis too small' in inertia alternative",
        "invalid size on <geom>: size 2 must be positive (geom #0 in body 'n')",
    ),
    (
        "an inertial's degenerate xyaxes then a later body's geom's size",
        r#"<mujoco><worldbody><body name="n"><freejoint/><inertial pos="0 0 0" mass="1" diaginertia="1 1 1" xyaxes="1 0 0 1 0 0"/></body><body name="q" pos="2 0 0"><freejoint/><geom type="sphere" size="0.1"/><geom type="box" size="0.1 0.1 0"/></body></worldbody></mujoco>"#,
        "error 'yaxis too small' in inertia alternative",
        "invalid size on <geom>: size 2 must be positive (geom #1 in body 'q')",
    ),
    (
        "an inertial's degenerate xyaxes then an actuator's unknown joint",
        r#"<mujoco><worldbody><body name="n"><freejoint/><inertial pos="0 0 0" mass="1" diaginertia="1 1 1" xyaxes="1 0 0 1 0 0"/></body></worldbody><actuator><motor joint="nope"/></actuator></mujoco>"#,
        "error 'yaxis too small' in inertia alternative",
        "unknown joint 'nope' (actuator #0)",
    ),
    (
        "a body's geoms' principal axes then a later body's geom's size",
        r#"<mujoco><worldbody><body name="n"><geom type="cylinder" size="1e-7 1" mass="0.5"/><geom type="cylinder" size="1e-7 1" mass="0.5" pos="0 0 3"/></body><body name="q" pos="2 0 0"><freejoint/><geom type="sphere" size="0.1"/><geom type="box" size="0.1 0.1 0"/></body></worldbody></mujoco>"#,
        "error 'inertia must have positive eigenvalues' in alternative for principal axes",
        "invalid size on <geom>: size 2 must be positive (geom #3 in body 'q')",
    ),
    (
        "a body's geoms' principal axes then an actuator's unknown joint",
        r#"<mujoco><worldbody><body name="n"><geom type="cylinder" size="1e-7 1" mass="0.5"/><geom type="cylinder" size="1e-7 1" mass="0.5" pos="0 0 3"/></body></worldbody><actuator><motor joint="nope"/></actuator></mujoco>"#,
        "error 'inertia must have positive eigenvalues' in alternative for principal axes",
        "unknown joint 'nope' (actuator #0)",
    ),
    (
        "an inertial's degenerate xyaxes then a later body's zero zaxis, the body in a frame",
        r#"<mujoco><worldbody><body name="n"><freejoint/><inertial pos="0 0 0" mass="1" diaginertia="1 1 1" xyaxes="1 0 0 1 0 0"/></body><frame pos="0 1 0"><body name="q" pos="2 0 0" zaxis="0 0 0"><freejoint/><geom type="sphere" size="0.1"/></body></frame></worldbody></mujoco>"#,
        "error 'yaxis too small' in inertia alternative",
        "invalid zaxis on <body>: zaxis too small (body 'q')",
    ),
    (
        "an inertial's degenerate xyaxes then a later body's zero zaxis",
        r#"<mujoco><worldbody><body name="n"><freejoint/><inertial pos="0 0 0" mass="1" diaginertia="1 1 1" xyaxes="1 0 0 1 0 0"/></body><body name="q" pos="2 0 0" zaxis="0 0 0"><freejoint/><geom type="sphere" size="0.1"/></body></worldbody></mujoco>"#,
        "error 'yaxis too small' in inertia alternative",
        "invalid zaxis on <body>: zaxis too small (body 'q')",
    ),
    (
        "an inertial's degenerate xyaxes then a zero-zaxis frame in a later body",
        r#"<mujoco><worldbody><body name="n"><freejoint/><inertial pos="0 0 0" mass="1" diaginertia="1 1 1" xyaxes="1 0 0 1 0 0"/></body><body name="q" pos="2 0 0"><freejoint/><geom type="sphere" size="0.1"/><frame zaxis="0 0 0"><geom type="sphere" size="0.1"/></frame></body></worldbody></mujoco>"#,
        "error 'yaxis too small' in inertia alternative",
        "invalid zaxis on <frame>: zaxis too small (frame in body 'q')",
    ),
    (
        "an inertial's degenerate xyaxes then its own body's geom's zero zaxis",
        r#"<mujoco><worldbody><body name="n"><freejoint/><inertial pos="0 0 0" mass="1" diaginertia="1 1 1" xyaxes="1 0 0 1 0 0"/><geom type="sphere" size="0.1" zaxis="0 0 0"/></body></worldbody></mujoco>"#,
        "error 'yaxis too small' in inertia alternative",
        "invalid zaxis on <geom>: zaxis too small (geom #0 in body 'n')",
    ),
    (
        "an inertial's degenerate xyaxes then a later body's geom's zero zaxis",
        r#"<mujoco><worldbody><body name="n"><freejoint/><inertial pos="0 0 0" mass="1" diaginertia="1 1 1" xyaxes="1 0 0 1 0 0"/></body><body name="q" pos="2 0 0"><freejoint/><geom type="sphere" size="0.1"/><geom type="box" size="0.1 0.1 0.1" zaxis="0 0 0"/></body></worldbody></mujoco>"#,
        "error 'yaxis too small' in inertia alternative",
        "invalid zaxis on <geom>: zaxis too small (geom #1 in body 'q')",
    ),
    (
        "an inertial's degenerate xyaxes then its own body's site's zero zaxis",
        r#"<mujoco><worldbody><body name="n"><freejoint/><inertial pos="0 0 0" mass="1" diaginertia="1 1 1" xyaxes="1 0 0 1 0 0"/><site name="s" zaxis="0 0 0"/></body></worldbody></mujoco>"#,
        "error 'yaxis too small' in inertia alternative",
        "invalid zaxis on <site>: zaxis too small (site 's' in body 'n')",
    ),
    (
        "an inertial's degenerate xyaxes then a later body's site's zero zaxis",
        r#"<mujoco><worldbody><body name="n"><freejoint/><inertial pos="0 0 0" mass="1" diaginertia="1 1 1" xyaxes="1 0 0 1 0 0"/></body><body name="q" pos="2 0 0"><freejoint/><geom type="sphere" size="0.1"/><site name="s" zaxis="0 0 0"/></body></worldbody></mujoco>"#,
        "error 'yaxis too small' in inertia alternative",
        "invalid zaxis on <site>: zaxis too small (site 's' in body 'q')",
    ),
    (
        "an inertial's degenerate xyaxes then its own body's geom's degenerate xyaxes",
        r#"<mujoco><worldbody><body name="n"><freejoint/><inertial pos="0 0 0" mass="1" diaginertia="1 1 1" xyaxes="1 0 0 1 0 0"/><geom type="sphere" size="0.1" xyaxes="1 0 0 1 0 0"/></body></worldbody></mujoco>"#,
        "error 'yaxis too small' in inertia alternative",
        "invalid xyaxes on <geom>: yaxis too small (geom #0 in body 'n')",
    ),
    (
        "an inertial's degenerate xyaxes then a later body's degenerate xyaxes",
        r#"<mujoco><worldbody><body name="n"><freejoint/><inertial pos="0 0 0" mass="1" diaginertia="1 1 1" xyaxes="1 0 0 1 0 0"/></body><body name="q" pos="2 0 0" xyaxes="1 0 0 1 0 0"><freejoint/><geom type="sphere" size="0.1"/></body></worldbody></mujoco>"#,
        "error 'yaxis too small' in inertia alternative",
        "invalid xyaxes on <body>: yaxis too small (body 'q')",
    ),
    (
        "an inertial's degenerate xyaxes then a later body's geom's degenerate xyaxes",
        r#"<mujoco><worldbody><body name="n"><freejoint/><inertial pos="0 0 0" mass="1" diaginertia="1 1 1" xyaxes="1 0 0 1 0 0"/></body><body name="q" pos="2 0 0"><freejoint/><geom type="sphere" size="0.1" xyaxes="1 0 0 1 0 0"/></body></worldbody></mujoco>"#,
        "error 'yaxis too small' in inertia alternative",
        "invalid xyaxes on <geom>: yaxis too small (geom #0 in body 'q')",
    ),
    (
        "an inertial's degenerate xyaxes then a later body's site's degenerate xyaxes",
        r#"<mujoco><worldbody><body name="n"><freejoint/><inertial pos="0 0 0" mass="1" diaginertia="1 1 1" xyaxes="1 0 0 1 0 0"/></body><body name="q" pos="2 0 0"><freejoint/><geom type="sphere" size="0.1"/><site xyaxes="1 0 0 1 0 0"/></body></worldbody></mujoco>"#,
        "error 'yaxis too small' in inertia alternative",
        "invalid xyaxes on <site>: yaxis too small (site in body 'q')",
    ),
    (
        "an inertial's degenerate xyaxes then a later body's fullinertia without positive eigenvalues",
        r#"<mujoco><worldbody><body name="n"><freejoint/><inertial pos="0 0 0" mass="1" diaginertia="1 1 1" xyaxes="1 0 0 1 0 0"/></body><body name="q" pos="2 0 0"><freejoint/><inertial pos="0 0 0" mass="1" fullinertia="1 1 1 2 2 2"/></body></worldbody></mujoco>"#,
        "error 'yaxis too small' in inertia alternative",
        "invalid fullinertia on <inertial>: inertia must have positive eigenvalues (body 'q')",
    ),
    (
        "an inertial's degenerate xyaxes then a mocap body nested in its body",
        r#"<mujoco><worldbody><body name="n"><freejoint/><inertial pos="0 0 0" mass="1" diaginertia="1 1 1" xyaxes="1 0 0 1 0 0"/><body name="mc" mocap="true"/></body></worldbody></mujoco>"#,
        "error 'yaxis too small' in inertia alternative",
        "a mocap body must be a direct child of worldbody (body 'mc')",
    ),
    (
        "an inertial's degenerate xyaxes then a tendon's unknown joint",
        r#"<mujoco><worldbody><body name="n"><freejoint/><inertial pos="0 0 0" mass="1" diaginertia="1 1 1" xyaxes="1 0 0 1 0 0"/></body></worldbody><tendon><fixed name="t"><joint joint="nope" coef="1"/></fixed></tendon></mujoco>"#,
        "error 'yaxis too small' in inertia alternative",
        "unknown joint 'nope' (tendon 't')",
    ),
    (
        "an inertial's degenerate xyaxes then a weld's unknown body",
        r#"<mujoco><worldbody><body name="n"><freejoint/><inertial pos="0 0 0" mass="1" diaginertia="1 1 1" xyaxes="1 0 0 1 0 0"/></body></worldbody><equality><weld body1="nope"/></equality></mujoco>"#,
        "error 'yaxis too small' in inertia alternative",
        "unknown body 'nope' (equality #0)",
    ),
    (
        "an inertial's degenerate xyaxes then a sensor's unknown joint",
        r#"<mujoco><worldbody><body name="n"><freejoint/><inertial pos="0 0 0" mass="1" diaginertia="1 1 1" xyaxes="1 0 0 1 0 0"/></body></worldbody><sensor><jointpos joint="nope"/></sensor></mujoco>"#,
        "error 'yaxis too small' in inertia alternative",
        "unknown joint 'nope' (sensor #0)",
    ),
    (
        "an inertial's degenerate xyaxes then an exclude's unknown body",
        r#"<mujoco><worldbody><body name="n"><freejoint/><inertial pos="0 0 0" mass="1" diaginertia="1 1 1" xyaxes="1 0 0 1 0 0"/></body></worldbody><contact><exclude body1="nope" body2="n"/></contact></mujoco>"#,
        "error 'yaxis too small' in inertia alternative",
        "unknown body 'nope' (exclude #0)",
    ),
    (
        "a body's geoms' principal axes then its own body's site's zero zaxis",
        r#"<mujoco><worldbody><body name="n"><geom type="cylinder" size="1e-7 1" mass="0.5"/><geom type="cylinder" size="1e-7 1" mass="0.5" pos="0 0 3"/><site name="s" zaxis="0 0 0"/></body></worldbody></mujoco>"#,
        "error 'inertia must have positive eigenvalues' in alternative for principal axes",
        "invalid zaxis on <site>: zaxis too small (site 's' in body 'n')",
    ),
    (
        "a body's geoms' principal axes then a later body's zero zaxis",
        r#"<mujoco><worldbody><body name="n"><geom type="cylinder" size="1e-7 1" mass="0.5"/><geom type="cylinder" size="1e-7 1" mass="0.5" pos="0 0 3"/></body><body name="q" pos="2 0 0" zaxis="0 0 0"><freejoint/><geom type="sphere" size="0.1"/></body></worldbody></mujoco>"#,
        "error 'inertia must have positive eigenvalues' in alternative for principal axes",
        "invalid zaxis on <body>: zaxis too small (body 'q')",
    ),
    (
        "a body's geoms' principal axes then a later body's geom's zero zaxis",
        r#"<mujoco><worldbody><body name="n"><geom type="cylinder" size="1e-7 1" mass="0.5"/><geom type="cylinder" size="1e-7 1" mass="0.5" pos="0 0 3"/></body><body name="q" pos="2 0 0"><freejoint/><geom type="sphere" size="0.1"/><geom type="box" size="0.1 0.1 0.1" zaxis="0 0 0"/></body></worldbody></mujoco>"#,
        "error 'inertia must have positive eigenvalues' in alternative for principal axes",
        "invalid zaxis on <geom>: zaxis too small (geom #3 in body 'q')",
    ),
    (
        "a body's geoms' principal axes then a later body's fullinertia without positive eigenvalues",
        r#"<mujoco><worldbody><body name="n"><geom type="cylinder" size="1e-7 1" mass="0.5"/><geom type="cylinder" size="1e-7 1" mass="0.5" pos="0 0 3"/></body><body name="q" pos="2 0 0"><freejoint/><inertial pos="0 0 0" mass="1" fullinertia="1 1 1 2 2 2"/></body></worldbody></mujoco>"#,
        "error 'inertia must have positive eigenvalues' in alternative for principal axes",
        "invalid fullinertia on <inertial>: inertia must have positive eigenvalues (body 'q')",
    ),
    (
        "a body's geoms' principal axes then a tendon's unknown joint",
        r#"<mujoco><worldbody><body name="n"><geom type="cylinder" size="1e-7 1" mass="0.5"/><geom type="cylinder" size="1e-7 1" mass="0.5" pos="0 0 3"/></body></worldbody><tendon><fixed name="t"><joint joint="nope" coef="1"/></fixed></tendon></mujoco>"#,
        "error 'inertia must have positive eigenvalues' in alternative for principal axes",
        "unknown joint 'nope' (tendon 't')",
    ),
    (
        "a body's geoms' principal axes then a sensor's unknown joint",
        r#"<mujoco><worldbody><body name="n"><geom type="cylinder" size="1e-7 1" mass="0.5"/><geom type="cylinder" size="1e-7 1" mass="0.5" pos="0 0 3"/></body></worldbody><sensor><jointpos joint="nope"/></sensor></mujoco>"#,
        "error 'inertia must have positive eigenvalues' in alternative for principal axes",
        "unknown joint 'nope' (sensor #0)",
    ),
    (
        "a body's non-finite inertia from its geoms (D-INERTIA-NONFINITE: MuJoCo loads that body) then a later body's zero hinge axis",
        r#"<mujoco><worldbody><body name="s"><geom type="sphere" size="6e99"/></body><body name="q" pos="2 0 0"><joint type="hinge" axis="0 0 0"/><geom type="sphere" size="0.1"/></body></worldbody></mujoco>"#,
        "axis too small in joint",
        "the body's mass or inertia computed from its geoms is not finite (body 's')",
    ),
    (
        "a body's non-finite inertia from its geoms (D-INERTIA-NONFINITE: MuJoCo loads that body) then a later body's geom's bad condim",
        r#"<mujoco><worldbody><body name="s"><geom type="sphere" size="6e99"/></body><body name="q" pos="2 0 0"><freejoint/><geom type="sphere" size="0.1" condim="2"/></body></worldbody></mujoco>"#,
        "invalid condim in geom",
        "the body's mass or inertia computed from its geoms is not finite (body 's')",
    ),
    (
        "a body's non-finite inertia from its geoms (D-INERTIA-NONFINITE: MuJoCo loads that body) then a later body's inertia breaking the triangle",
        r#"<mujoco><worldbody><body name="s"><geom type="sphere" size="6e99"/></body><body name="q" pos="2 0 0"><freejoint/><inertial pos="0 0 0" mass="1" diaginertia="1 1 5"/></body></worldbody></mujoco>"#,
        "inertia must satisfy A + B >= C; use 'balanceinertia' to fix",
        "the body's mass or inertia computed from its geoms is not finite (body 's')",
    ),
];

#[test]
fn an_inertia_fault_beside_another_is_named_as_ledgered() {
    let mut moved = Vec::new();
    for &(what, xml, _mujoco, ours) in INERTIAL_COSTS {
        match load_model(xml) {
            Err(e) if e.to_string() == ours => {}
            Err(e) => moved.push(format!("{what}: {e}")),
            Ok(_) => moved.push(format!("{what}: loads")),
        }
    }
    assert!(moved.is_empty(), "{}", moved.join("\n"));
}

/// Two faults, one a mesh MuJoCo 3.5.0 refuses as it builds the meshes, before
/// the bodies (its volume, an exact mesh's orientation), one a geom's this
/// crate checks before it builds the meshes (a size, a `fromto`), where this
/// crate names the geom's, as it did before RL-9b: ledger L353 holds each.
/// `(what, document, MuJoCo 3.5.0's reason, this crate's error)`, measured
/// 2026-11-10 (RL-9b's review, RB's documents).
const MESH_COSTS: &[(&str, &str, &str, &str)] = &[
    (
        "a mesh too small in volume, then a sphere of size 0",
        r#"<mujoco><asset><mesh name="flat" vertex="0 0 0 1 0 0 0 1 0 0 0 1" face="0 1 2 0 1 2 0 1 2 0 1 2"/><mesh name="m" vertex="-0.5 -0.5 -0.5  0.5 -0.5 -0.5  0.5 0.5 -0.5  -0.5 0.5 -0.5  -0.5 -0.5 0.5  0.5 -0.5 0.5  0.5 0.5 0.5  -0.5 0.5 0.5" face="0 2 1  0 3 2  4 5 6  4 6 7  0 1 5  0 5 4  2 3 7  2 7 6  0 4 7  0 7 3  1 2 6  1 6 5"/></asset><worldbody><body name="b"><freejoint/><geom type="mesh" mesh="flat" contype="0" conaffinity="0"/><geom size=".1"/></body><body name="c"><freejoint/><geom type="sphere" size="0"/></body></worldbody></mujoco>"#,
        "mesh volume is too small: flat . Try setting inertia to shell",
        "invalid size on <geom>: size 0 must be positive (geom #2 in body 'c')",
    ),
    (
        "an exact mesh's faces turned, then a sphere of size 0",
        r#"<mujoco><asset><mesh name="turn" inertia="exact" vertex="-0.5 -0.5 -0.5  0.5 -0.5 -0.5  0.5 0.5 -0.5  -0.5 0.5 -0.5  -0.5 -0.5 0.5  0.5 -0.5 0.5  0.5 0.5 0.5  -0.5 0.5 0.5" face="0 1 2  0 3 2  4 5 6  4 6 7  0 1 5  0 5 4  2 3 7  2 7 6  0 4 7  0 7 3  1 2 6  1 6 5"/><mesh name="m" vertex="-0.5 -0.5 -0.5  0.5 -0.5 -0.5  0.5 0.5 -0.5  -0.5 0.5 -0.5  -0.5 -0.5 0.5  0.5 -0.5 0.5  0.5 0.5 0.5  -0.5 0.5 0.5" face="0 2 1  0 3 2  4 5 6  4 6 7  0 1 5  0 5 4  2 3 7  2 7 6  0 4 7  0 7 3  1 2 6  1 6 5"/></asset><worldbody><body name="b"><freejoint/><geom type="mesh" mesh="turn" contype="0" conaffinity="0"/><geom size=".1"/></body><body name="c"><freejoint/><geom type="sphere" size="0"/></body></worldbody></mujoco>"#,
        "faces of mesh 'turn' have inconsistent orientation. Please check the faces containing the vertices 1 and 2.",
        "invalid size on <geom>: size 0 must be positive (geom #2 in body 'c')",
    ),
    (
        "a mesh too small in volume, then a mesh geom's fromto",
        r#"<mujoco><asset><mesh name="flat" vertex="0 0 0 1 0 0 0 1 0 0 0 1" face="0 1 2 0 1 2 0 1 2 0 1 2"/><mesh name="m" vertex="-0.5 -0.5 -0.5  0.5 -0.5 -0.5  0.5 0.5 -0.5  -0.5 0.5 -0.5  -0.5 -0.5 0.5  0.5 -0.5 0.5  0.5 0.5 0.5  -0.5 0.5 0.5" face="0 2 1  0 3 2  4 5 6  4 6 7  0 1 5  0 5 4  2 3 7  2 7 6  0 4 7  0 7 3  1 2 6  1 6 5"/></asset><worldbody><body name="b"><freejoint/><geom type="mesh" mesh="flat" contype="0" conaffinity="0"/><geom size=".1"/></body><body name="c"><freejoint/><geom type="mesh" mesh="m" fromto="0 0 0 1 0 0"/></body></worldbody></mujoco>"#,
        "mesh volume is too small: flat . Try setting inertia to shell",
        "invalid fromto on <geom>: fromto requires a capsule, cylinder, ellipsoid or box (geom in body 'c')",
    ),
    (
        "an exact mesh's faces turned, then a mesh geom's fromto",
        r#"<mujoco><asset><mesh name="turn" inertia="exact" vertex="-0.5 -0.5 -0.5  0.5 -0.5 -0.5  0.5 0.5 -0.5  -0.5 0.5 -0.5  -0.5 -0.5 0.5  0.5 -0.5 0.5  0.5 0.5 0.5  -0.5 0.5 0.5" face="0 1 2  0 3 2  4 5 6  4 6 7  0 1 5  0 5 4  2 3 7  2 7 6  0 4 7  0 7 3  1 2 6  1 6 5"/><mesh name="m" vertex="-0.5 -0.5 -0.5  0.5 -0.5 -0.5  0.5 0.5 -0.5  -0.5 0.5 -0.5  -0.5 -0.5 0.5  0.5 -0.5 0.5  0.5 0.5 0.5  -0.5 0.5 0.5" face="0 2 1  0 3 2  4 5 6  4 6 7  0 1 5  0 5 4  2 3 7  2 7 6  0 4 7  0 7 3  1 2 6  1 6 5"/></asset><worldbody><body name="b"><freejoint/><geom type="mesh" mesh="turn" contype="0" conaffinity="0"/><geom size=".1"/></body><body name="c"><freejoint/><geom type="mesh" mesh="m" fromto="0 0 0 1 0 0"/></body></worldbody></mujoco>"#,
        "faces of mesh 'turn' have inconsistent orientation. Please check the faces containing the vertices 1 and 2.",
        "invalid fromto on <geom>: fromto requires a capsule, cylinder, ellipsoid or box (geom in body 'c')",
    ),
    (
        "a mesh too small in volume, then a box of size 0, the mesh unused",
        r#"<mujoco><asset><mesh name="flat" vertex="0 0 0 1 0 0 0 1 0 0 0 1" face="0 1 2 0 1 2 0 1 2 0 1 2"/></asset><worldbody><body name="c"><freejoint/><geom type="box" size="0 1 1"/></body></worldbody></mujoco>"#,
        "mesh volume is too small: flat . Try setting inertia to shell",
        "invalid size on <geom>: size 0 must be positive (geom #0 in body 'c')",
    ),
    (
        "an exact mesh's faces turned, then a box of size 0, the mesh unused",
        r#"<mujoco><asset><mesh name="turn" inertia="exact" vertex="-0.5 -0.5 -0.5  0.5 -0.5 -0.5  0.5 0.5 -0.5  -0.5 0.5 -0.5  -0.5 -0.5 0.5  0.5 -0.5 0.5  0.5 0.5 0.5  -0.5 0.5 0.5" face="0 1 2  0 3 2  4 5 6  4 6 7  0 1 5  0 5 4  2 3 7  2 7 6  0 4 7  0 7 3  1 2 6  1 6 5"/></asset><worldbody><body name="c"><freejoint/><geom type="box" size="0 1 1"/></body></worldbody></mujoco>"#,
        "faces of mesh 'turn' have inconsistent orientation. Please check the faces containing the vertices 1 and 2.",
        "invalid size on <geom>: size 0 must be positive (geom #0 in body 'c')",
    ),
    (
        "a mesh too small in volume, then a capsule's fromto points coinciding",
        r#"<mujoco><asset><mesh name="flat" vertex="0 0 0 1 0 0 0 1 0 0 0 1" face="0 1 2 0 1 2 0 1 2 0 1 2"/><mesh name="m" vertex="-0.5 -0.5 -0.5  0.5 -0.5 -0.5  0.5 0.5 -0.5  -0.5 0.5 -0.5  -0.5 -0.5 0.5  0.5 -0.5 0.5  0.5 0.5 0.5  -0.5 0.5 0.5" face="0 2 1  0 3 2  4 5 6  4 6 7  0 1 5  0 5 4  2 3 7  2 7 6  0 4 7  0 7 3  1 2 6  1 6 5"/></asset><worldbody><body name="b"><freejoint/><geom type="mesh" mesh="flat" contype="0" conaffinity="0"/><geom size=".1"/></body><body name="c"><freejoint/><geom type="capsule" size=".1" fromto="0 0 0 0 0 0"/></body></worldbody></mujoco>"#,
        "mesh volume is too small: flat . Try setting inertia to shell",
        "invalid fromto on <geom>: fromto points too close (geom in body 'c')",
    ),
    (
        "an exact mesh's faces turned, then a capsule's fromto points coinciding",
        r#"<mujoco><asset><mesh name="turn" inertia="exact" vertex="-0.5 -0.5 -0.5  0.5 -0.5 -0.5  0.5 0.5 -0.5  -0.5 0.5 -0.5  -0.5 -0.5 0.5  0.5 -0.5 0.5  0.5 0.5 0.5  -0.5 0.5 0.5" face="0 1 2  0 3 2  4 5 6  4 6 7  0 1 5  0 5 4  2 3 7  2 7 6  0 4 7  0 7 3  1 2 6  1 6 5"/><mesh name="m" vertex="-0.5 -0.5 -0.5  0.5 -0.5 -0.5  0.5 0.5 -0.5  -0.5 0.5 -0.5  -0.5 -0.5 0.5  0.5 -0.5 0.5  0.5 0.5 0.5  -0.5 0.5 0.5" face="0 2 1  0 3 2  4 5 6  4 6 7  0 1 5  0 5 4  2 3 7  2 7 6  0 4 7  0 7 3  1 2 6  1 6 5"/></asset><worldbody><body name="b"><freejoint/><geom type="mesh" mesh="turn" contype="0" conaffinity="0"/><geom size=".1"/></body><body name="c"><freejoint/><geom type="capsule" size=".1" fromto="0 0 0 0 0 0"/></body></worldbody></mujoco>"#,
        "faces of mesh 'turn' have inconsistent orientation. Please check the faces containing the vertices 1 and 2.",
        "invalid fromto on <geom>: fromto points too close (geom in body 'c')",
    ),
    (
        "a mesh too small in volume, then a tendon's unknown joint",
        r#"<mujoco><asset><mesh name="flat" vertex="0 0 0 1 0 0 0 1 0 0 0 1" face="0 1 2 0 1 2 0 1 2 0 1 2"/></asset><worldbody><body name="b"><freejoint/><geom type="mesh" mesh="flat"/><geom size=".1"/></body></worldbody><tendon><fixed name="t"><joint joint="nope" coef="1"/></fixed></tendon></mujoco>"#,
        "mesh volume is too small: flat . Try setting inertia to shell",
        "unknown joint 'nope' (tendon 't')",
    ),
    (
        "a mesh too small in volume, then a spatial tendon with one site",
        r#"<mujoco><asset><mesh name="flat" vertex="0 0 0 1 0 0 0 1 0 0 0 1" face="0 1 2 0 1 2 0 1 2 0 1 2"/></asset><worldbody><body name="b"><freejoint/><geom type="mesh" mesh="flat"/><geom size=".1"/><site name="s1"/></body></worldbody><tendon><spatial name="t"><site site="s1"/></spatial></tendon></mujoco>"#,
        "mesh volume is too small: flat . Try setting inertia to shell",
        "spatial tendon needs at least 2 sites, has 1 (tendon 't')",
    ),
    (
        "a mesh too small in volume, then an actuator's unknown joint",
        r#"<mujoco><asset><mesh name="flat" vertex="0 0 0 1 0 0 0 1 0 0 0 1" face="0 1 2 0 1 2 0 1 2 0 1 2"/></asset><worldbody><body name="b"><freejoint/><geom type="mesh" mesh="flat"/><geom size=".1"/></body></worldbody><actuator><motor joint="nope"/></actuator></mujoco>"#,
        "mesh volume is too small: flat . Try setting inertia to shell",
        "unknown joint 'nope' (actuator #0)",
    ),
];

#[test]
fn a_mesh_refusal_beside_a_geoms_fault_is_named_as_ledgered() {
    let mut moved = Vec::new();
    for &(what, xml, _mujoco, ours) in MESH_COSTS {
        match load_model(xml) {
            Err(e) if e.to_string() == ours => {}
            Err(e) => moved.push(format!("{what}: {e}")),
            Ok(_) => moved.push(format!("{what}: loads")),
        }
    }
    assert!(moved.is_empty(), "{}", moved.join("\n"));
}

/// Two faults, one a mesh's written vertex or face count, which MuJoCo
/// 3.5.0 checks as it begins to compile the model, where this crate names
/// the other: a `NaN` written in a vertex, which this crate refuses as it
/// reads it and MuJoCo after the counts (registry `D-NONFINITE`). `(what,
/// document, MuJoCo's reason, this crate's error)`, measured 2026-11-19
/// (RL-9d).
const MESH_DATA_COSTS: &[(&str, &str, &str, &str)] = &[(
    "a vertex's NaN and a face of four numbers",
    r#"<mujoco><asset><mesh name="m" vertex="nan 0 0  1 0 0  0 2 0  0 0 3" face="0 2 1 3"/></asset><worldbody><body name="b" pos="0 0 1"><freejoint/><geom type="mesh" mesh="m"/></body></worldbody></mujoco>"#,
    "face data must be a multiple of 3",
    "vertex on <mesh> is not finite (NaN) (line 1: asset > mesh 'm')",
)];

#[test]
fn a_meshs_written_count_beside_another_fault_is_named_as_ledgered() {
    let mut moved = Vec::new();
    for &(what, xml, _mujoco, ours) in MESH_DATA_COSTS {
        match load_model(xml) {
            Err(e) if e.to_string() == ours => {}
            Err(e) => moved.push(format!("{what}: {e}")),
            Ok(_) => moved.push(format!("{what}: loads")),
        }
    }
    assert!(moved.is_empty(), "{}", moved.join("\n"));
}

/// Two faults, `shellinertia` on an earlier mesh geom and one this crate meets
/// as it reads the model (a later geom's, body's or site's orientation or
/// `user` data, a sensor's, equality's or exclude's reference), where MuJoCo
/// 3.5.0 names the `shellinertia`, compiling each geom whole before the next,
/// and this crate the other, which it meets before it checks the geoms:
/// ledger L353 holds each. `(what, document, MuJoCo 3.5.0's reason, this
/// crate's error)`, measured 2026-11-10 (RL-9b's fix-diff pass, FD's
/// documents).
const SHELLINERTIA_COSTS: &[(&str, &str, &str, &str)] = &[
    (
        "a mesh geom's shellinertia, then a later geom's zaxis",
        r#"<mujoco><asset><mesh name="m" vertex="-0.5 -0.5 -0.5  0.5 -0.5 -0.5  0.5 0.5 -0.5  -0.5 0.5 -0.5  -0.5 -0.5 0.5  0.5 -0.5 0.5  0.5 0.5 0.5  -0.5 0.5 0.5" face="0 2 1  0 3 2  4 5 6  4 6 7  0 1 5  0 5 4  2 3 7  2 7 6  0 4 7  0 7 3  1 2 6  1 6 5"/></asset><worldbody><body name="a"><freejoint/><geom name="s" type="mesh" mesh="m" shellinertia="true"/><geom size=".1" zaxis="0 0 0"/></body></worldbody></mujoco>"#,
        "for mesh geoms, inertia should be specified in the mesh asset",
        "invalid zaxis on <geom>: zaxis too small (geom #1 in body 'a')",
    ),
    (
        "a mesh geom's shellinertia, then a later geom's user over nuser_geom",
        r#"<mujoco><size nuser_geom="1"/><asset><mesh name="m" vertex="-0.5 -0.5 -0.5  0.5 -0.5 -0.5  0.5 0.5 -0.5  -0.5 0.5 -0.5  -0.5 -0.5 0.5  0.5 -0.5 0.5  0.5 0.5 0.5  -0.5 0.5 0.5" face="0 2 1  0 3 2  4 5 6  4 6 7  0 1 5  0 5 4  2 3 7  2 7 6  0 4 7  0 7 3  1 2 6  1 6 5"/></asset><worldbody><body name="a"><freejoint/><geom name="s" type="mesh" mesh="m" shellinertia="true"/><geom size=".1" user="1 2"/></body></worldbody></mujoco>"#,
        "for mesh geoms, inertia should be specified in the mesh asset",
        "invalid user on <geom>: user data exceeds nuser_geom=1",
    ),
    (
        "a mesh geom's shellinertia, then a later body's zaxis",
        r#"<mujoco><asset><mesh name="m" vertex="-0.5 -0.5 -0.5  0.5 -0.5 -0.5  0.5 0.5 -0.5  -0.5 0.5 -0.5  -0.5 -0.5 0.5  0.5 -0.5 0.5  0.5 0.5 0.5  -0.5 0.5 0.5" face="0 2 1  0 3 2  4 5 6  4 6 7  0 1 5  0 5 4  2 3 7  2 7 6  0 4 7  0 7 3  1 2 6  1 6 5"/></asset><worldbody><body name="a"><freejoint/><geom name="s" type="mesh" mesh="m" shellinertia="true"/></body><body name="c" zaxis="0 0 0"><freejoint/><geom size=".1"/></body></worldbody></mujoco>"#,
        "for mesh geoms, inertia should be specified in the mesh asset",
        "invalid zaxis on <body>: zaxis too small (body 'c')",
    ),
    (
        "a mesh geom's shellinertia, then a sensor's unknown joint",
        r#"<mujoco><asset><mesh name="m" vertex="-0.5 -0.5 -0.5  0.5 -0.5 -0.5  0.5 0.5 -0.5  -0.5 0.5 -0.5  -0.5 -0.5 0.5  0.5 -0.5 0.5  0.5 0.5 0.5  -0.5 0.5 0.5" face="0 2 1  0 3 2  4 5 6  4 6 7  0 1 5  0 5 4  2 3 7  2 7 6  0 4 7  0 7 3  1 2 6  1 6 5"/></asset><worldbody><body name="a"><freejoint/><geom name="s" type="mesh" mesh="m" shellinertia="true"/></body></worldbody><sensor><jointpos joint="nope"/></sensor></mujoco>"#,
        "for mesh geoms, inertia should be specified in the mesh asset",
        "unknown joint 'nope' (sensor #0)",
    ),
    (
        "a mesh geom's shellinertia, then an equality's unknown body",
        r#"<mujoco><asset><mesh name="m" vertex="-0.5 -0.5 -0.5  0.5 -0.5 -0.5  0.5 0.5 -0.5  -0.5 0.5 -0.5  -0.5 -0.5 0.5  0.5 -0.5 0.5  0.5 0.5 0.5  -0.5 0.5 0.5" face="0 2 1  0 3 2  4 5 6  4 6 7  0 1 5  0 5 4  2 3 7  2 7 6  0 4 7  0 7 3  1 2 6  1 6 5"/></asset><worldbody><body name="a"><freejoint/><geom name="s" type="mesh" mesh="m" shellinertia="true"/></body></worldbody><equality><weld body1="nope"/></equality></mujoco>"#,
        "for mesh geoms, inertia should be specified in the mesh asset",
        "unknown body 'nope' (equality #0)",
    ),
    (
        "a mesh geom's shellinertia, then an exclude's unknown body",
        r#"<mujoco><asset><mesh name="m" vertex="-0.5 -0.5 -0.5  0.5 -0.5 -0.5  0.5 0.5 -0.5  -0.5 0.5 -0.5  -0.5 -0.5 0.5  0.5 -0.5 0.5  0.5 0.5 0.5  -0.5 0.5 0.5" face="0 2 1  0 3 2  4 5 6  4 6 7  0 1 5  0 5 4  2 3 7  2 7 6  0 4 7  0 7 3  1 2 6  1 6 5"/></asset><worldbody><body name="a"><freejoint/><geom name="s" type="mesh" mesh="m" shellinertia="true"/></body></worldbody><contact><exclude body1="a" body2="nope"/></contact></mujoco>"#,
        "for mesh geoms, inertia should be specified in the mesh asset",
        "unknown body 'nope' (exclude #0)",
    ),
    (
        "a mesh geom's shellinertia, then a later site's zaxis",
        r#"<mujoco><asset><mesh name="m" vertex="-0.5 -0.5 -0.5  0.5 -0.5 -0.5  0.5 0.5 -0.5  -0.5 0.5 -0.5  -0.5 -0.5 0.5  0.5 -0.5 0.5  0.5 0.5 0.5  -0.5 0.5 0.5" face="0 2 1  0 3 2  4 5 6  4 6 7  0 1 5  0 5 4  2 3 7  2 7 6  0 4 7  0 7 3  1 2 6  1 6 5"/></asset><worldbody><body name="a"><freejoint/><geom name="s" type="mesh" mesh="m" shellinertia="true"/></body><body name="c"><freejoint/><geom size=".1"/><site zaxis="0 0 0"/></body></worldbody></mujoco>"#,
        "for mesh geoms, inertia should be specified in the mesh asset",
        "invalid zaxis on <site>: zaxis too small (site in body 'c')",
    ),
];

#[test]
fn shellinertia_beside_a_later_fault_is_named_as_ledgered() {
    let mut moved = Vec::new();
    for &(what, xml, _mujoco, ours) in SHELLINERTIA_COSTS {
        match load_model(xml) {
            Err(e) if e.to_string() == ours => {}
            Err(e) => moved.push(format!("{what}: {e}")),
            Ok(_) => moved.push(format!("{what}: loads")),
        }
    }
    assert!(moved.is_empty(), "{}", moved.join("\n"));
}

/// Two faults, a sphere, capsule, cylinder, ellipsoid or box whose fit to the
/// mesh it names gives a size MuJoCo 3.5.0 refuses, and a size this crate
/// checks before it builds the model, in a later body or a site of the
/// geom's body. MuJoCo fits and checks each geom as it compiles it, before
/// the body's sites and the next body; this crate fits it as it builds the
/// body, after those checks, and names the other: ledger L353 holds each.
/// `(what, document, MuJoCo 3.5.0's reason, this crate's error)`, measured
/// 2026-11-13 (RL-9b3's start set).
const FIT_COSTS: &[(&str, &str, &str, &str)] = &[
    (
        "a capsule fitted to a flat mesh's box, its half length not above 0, then a later body's sphere of size 0",
        r#"<mujoco><compiler fitaabb="true"/><asset><mesh name="f" vertex="-1 -1 -0.1  1 -1 -0.1  1 1 -0.1  -1 1 -0.1  -1 -1 0.1  1 -1 0.1  1 1 0.1  -1 1 0.1" face="0 2 1  0 3 2  4 5 6  4 6 7  0 1 5  0 5 4  2 3 7  2 7 6  0 4 7  0 7 3  1 2 6  1 6 5"/></asset><worldbody><body name="b" pos="0 0 1"><freejoint/><geom type="capsule" mesh="f"/></body><body name="c"><freejoint/><geom type="sphere" size="0"/></body></worldbody></mujoco>"#,
        "size 1 must be positive in geom",
        "invalid size on <geom>: size 0 must be positive (geom #1 in body 'c')",
    ),
    (
        "a capsule fitted to a flat mesh's box, its half length not above 0, then its body's site of size 0",
        r#"<mujoco><compiler fitaabb="true"/><asset><mesh name="f" vertex="-1 -1 -0.1  1 -1 -0.1  1 1 -0.1  -1 1 -0.1  -1 -1 0.1  1 -1 0.1  1 1 0.1  -1 1 0.1" face="0 2 1  0 3 2  4 5 6  4 6 7  0 1 5  0 5 4  2 3 7  2 7 6  0 4 7  0 7 3  1 2 6  1 6 5"/></asset><worldbody><body name="b" pos="0 0 1"><freejoint/><geom type="capsule" mesh="f"/><site type="sphere" size="0"/></body></worldbody></mujoco>"#,
        "size 1 must be positive in geom",
        "invalid size on <site>: size 0 must be positive (site in body 'b')",
    ),
];

#[test]
fn a_fitted_size_beside_a_size_checked_before_the_build_is_named_as_ledgered() {
    let mut moved = Vec::new();
    for &(what, xml, _mujoco, ours) in FIT_COSTS {
        match load_model(xml) {
            Err(e) if e.to_string() == ours => {}
            Err(e) => moved.push(format!("{what}: {e}")),
            Ok(_) => moved.push(format!("{what}: loads")),
        }
    }
    assert!(moved.is_empty(), "{}", moved.join("\n"));
}

/// Two faults, a fitted size MuJoCo 3.5.0 refuses and a fault this crate
/// checks after it builds the model (`check_bodies`), in the fitted geom or
/// before it. MuJoCo meets the other first, as it compiles each body and
/// geom in turn; this crate checks the fitted size as it builds the body,
/// before those checks, and names it: ledger L353 holds the class. The rows
/// are the documents review round 1 measured (RA-2), not one for each fault
/// `check_bodies` makes. `(what, document, MuJoCo 3.5.0's reason, this
/// crate's error)`, measured 2026-11-14.
const FIT_FIRST_COSTS: &[(&str, &str, &str, &str)] = &[
    (
        "a capsule fitted to a flat mesh's box, its half length not above 0, with a condim of 2",
        r#"<mujoco><compiler fitaabb="true"/><asset><mesh name="f" vertex="-1 -1 -0.1  1 -1 -0.1  1 1 -0.1  -1 1 -0.1  -1 -1 0.1  1 -1 0.1  1 1 0.1  -1 1 0.1" face="0 2 1  0 3 2  4 5 6  4 6 7  0 1 5  0 5 4  2 3 7  2 7 6  0 4 7  0 7 3  1 2 6  1 6 5"/></asset><worldbody><body name="b" pos="0 0 1"><freejoint/><geom type="capsule" mesh="f" condim="2"/></body></worldbody></mujoco>"#,
        "invalid condim in geom",
        "invalid size on <geom>: size 1 must be positive (geom #0 in body 'b')",
    ),
    (
        "a sphere with a condim of 2, then a capsule fitted to a flat mesh's box, its half length not above 0",
        r#"<mujoco><compiler fitaabb="true"/><asset><mesh name="f" vertex="-1 -1 -0.1  1 -1 -0.1  1 1 -0.1  -1 1 -0.1  -1 -1 0.1  1 -1 0.1  1 1 0.1  -1 1 0.1" face="0 2 1  0 3 2  4 5 6  4 6 7  0 1 5  0 5 4  2 3 7  2 7 6  0 4 7  0 7 3  1 2 6  1 6 5"/></asset><worldbody><body name="b" pos="0 0 1"><freejoint/><geom size=".1" condim="2"/><geom type="capsule" mesh="f"/></body></worldbody></mujoco>"#,
        "invalid condim in geom",
        "invalid size on <geom>: size 1 must be positive (geom #1 in body 'b')",
    ),
    (
        "a mesh geom with a condim of 5, then a capsule fitted to a flat mesh's box, its half length not above 0",
        r#"<mujoco><compiler fitaabb="true"/><asset><mesh name="f" vertex="-1 -1 -0.1  1 -1 -0.1  1 1 -0.1  -1 1 -0.1  -1 -1 0.1  1 -1 0.1  1 1 0.1  -1 1 0.1" face="0 2 1  0 3 2  4 5 6  4 6 7  0 1 5  0 5 4  2 3 7  2 7 6  0 4 7  0 7 3  1 2 6  1 6 5"/></asset><worldbody><body name="b" pos="0 0 1"><freejoint/><geom type="mesh" mesh="f" condim="5"/><geom type="capsule" mesh="f"/></body></worldbody></mujoco>"#,
        "invalid condim in geom",
        "invalid size on <geom>: size 1 must be positive (geom #1 in body 'b')",
    ),
    (
        "a sphere of mass -1, then a capsule fitted to a flat mesh's box, its half length not above 0",
        r#"<mujoco><compiler fitaabb="true"/><asset><mesh name="f" vertex="-1 -1 -0.1  1 -1 -0.1  1 1 -0.1  -1 1 -0.1  -1 -1 0.1  1 -1 0.1  1 1 0.1  -1 1 0.1" face="0 2 1  0 3 2  4 5 6  4 6 7  0 1 5  0 5 4  2 3 7  2 7 6  0 4 7  0 7 3  1 2 6  1 6 5"/></asset><worldbody><body name="b" pos="0 0 1"><freejoint/><geom size=".1" mass="-1"/><geom type="capsule" mesh="f"/></body></worldbody></mujoco>"#,
        "mass, inertia or density are negative in geom",
        "invalid size on <geom>: size 1 must be positive (geom #1 in body 'b')",
    ),
    (
        "a body's sphere of mass -1, then a later body's capsule fitted to a flat mesh's box, its half length not above 0",
        r#"<mujoco><compiler fitaabb="true"/><asset><mesh name="f" vertex="-1 -1 -0.1  1 -1 -0.1  1 1 -0.1  -1 1 -0.1  -1 -1 0.1  1 -1 0.1  1 1 0.1  -1 1 0.1" face="0 2 1  0 3 2  4 5 6  4 6 7  0 1 5  0 5 4  2 3 7  2 7 6  0 4 7  0 7 3  1 2 6  1 6 5"/></asset><worldbody><body name="a" pos="0 0 1"><freejoint/><geom size=".1" mass="-1"/></body><body name="b"><freejoint/><geom type="capsule" mesh="f"/></body></worldbody></mujoco>"#,
        "mass, inertia or density are negative in geom",
        "invalid size on <geom>: size 1 must be positive (geom #1 in body 'b')",
    ),
    (
        "a plane in a moving body, then a capsule fitted to a flat mesh's box, its half length not above 0",
        r#"<mujoco><compiler fitaabb="true"/><asset><mesh name="f" vertex="-1 -1 -0.1  1 -1 -0.1  1 1 -0.1  -1 1 -0.1  -1 -1 0.1  1 -1 0.1  1 1 0.1  -1 1 0.1" face="0 2 1  0 3 2  4 5 6  4 6 7  0 1 5  0 5 4  2 3 7  2 7 6  0 4 7  0 7 3  1 2 6  1 6 5"/></asset><worldbody><body name="b" pos="0 0 1"><freejoint/><geom type="plane" size="1 1 1"/><geom type="capsule" mesh="f"/></body></worldbody></mujoco>"#,
        "plane only allowed in static bodies",
        "invalid size on <geom>: size 1 must be positive (geom #1 in body 'b')",
    ),
    (
        "a body's inertial with fullinertia and a diagonal inertia, then a later body's capsule fitted to a flat mesh's box, its half length not above 0",
        r#"<mujoco><compiler fitaabb="true"/><asset><mesh name="f" vertex="-1 -1 -0.1  1 -1 -0.1  1 1 -0.1  -1 1 -0.1  -1 -1 0.1  1 -1 0.1  1 1 0.1  -1 1 0.1" face="0 2 1  0 3 2  4 5 6  4 6 7  0 1 5  0 5 4  2 3 7  2 7 6  0 4 7  0 7 3  1 2 6  1 6 5"/></asset><worldbody><body name="a" pos="0 0 1"><freejoint/><inertial pos="0 0 0" mass="1" fullinertia="1 1 1 0 0 0" diaginertia="1 1 1"/><geom size=".1"/></body><body name="b"><freejoint/><geom type="capsule" mesh="f"/></body></worldbody></mujoco>"#,
        "fullinertia and diagonal inertia cannot both be specified",
        "invalid size on <geom>: size 1 must be positive (geom #1 in body 'b')",
    ),
];

#[test]
fn a_fitted_size_beside_a_fault_checked_after_the_build_is_named_as_ledgered() {
    let mut moved = Vec::new();
    for &(what, xml, _mujoco, ours) in FIT_FIRST_COSTS {
        match load_model(xml) {
            Err(e) if e.to_string() == ours => {}
            Err(e) => moved.push(format!("{what}: {e}")),
            Ok(_) => moved.push(format!("{what}: loads")),
        }
    }
    assert!(moved.is_empty(), "{}", moved.join("\n"));
}

/// Two faults in one geom, a height field it names but is not, and a fault
/// MuJoCo 3.5.0 checks before that: its condim, or a mesh or SDF geom's
/// missing mesh (`user_objects.cc:3651-3664`). This crate checks the height
/// field as it validates, before both, and names the height field: ledger
/// L353 holds each. `(what, document, MuJoCo 3.5.0's reason, this crate's
/// error)`, measured 2026-11-14 (review round 1, RW-4, RB-2; F1-5).
const HFIELD_COSTS: &[(&str, &str, &str, &str)] = &[
    (
        "a sphere with a condim of 2 that names a height field",
        r#"<mujoco><asset><mesh name="m" vertex="0 0 0  1 0 0  0 2 0  0 0 3" face="0 2 1  0 1 3  0 3 2  1 2 3"/><hfield name="h" nrow="2" ncol="2" elevation="0 0 0 0" size="1 1 0.1 0.1"/></asset><worldbody><geom condim="2" hfield="h" size="0.1"/></worldbody></mujoco>"#,
        "invalid condim in geom",
        "hfield geom must have valid hfieldid (geom #0 in body 'world')",
    ),
    (
        "a mesh geom that names no mesh but a height field",
        r#"<mujoco><asset><mesh name="m" vertex="0 0 0  1 0 0  0 2 0  0 0 3" face="0 2 1  0 1 3  0 3 2  1 2 3"/><hfield name="h" nrow="2" ncol="2" elevation="0 0 0 0" size="1 1 0.1 0.1"/></asset><worldbody><geom type="mesh" hfield="h"/></worldbody></mujoco>"#,
        "mesh geom '' (id = 0) must have valid meshid",
        "hfield geom must have valid hfieldid (geom #0 in body 'world')",
    ),
    (
        "an SDF geom that names no mesh but a height field",
        r#"<mujoco><asset><hfield name="h" nrow="2" ncol="2" elevation="0 0 0 0" size="1 1 0.1 0.1"/></asset><worldbody><geom type="sdf" hfield="h"/></worldbody></mujoco>"#,
        "mesh geom '' (id = 0) must have valid meshid",
        "hfield geom must have valid hfieldid (geom #0 in body 'world')",
    ),
];

#[test]
fn a_height_field_beside_a_geoms_earlier_fault_is_named_as_ledgered() {
    let mut moved = Vec::new();
    for &(what, xml, _mujoco, ours) in HFIELD_COSTS {
        match load_model(xml) {
            Err(e) if e.to_string() == ours => {}
            Err(e) => moved.push(format!("{what}: {e}")),
            Ok(_) => moved.push(format!("{what}: loads")),
        }
    }
    assert!(moved.is_empty(), "{}", moved.join("\n"));
}

/// Two faults in one geom, a mesh name that no mesh holds and a `fromto`,
/// which MuJoCo 3.5.0 refuses beside any mesh as it compiles the geom
/// (`user_objects.cc:3727-3730`), after it indexes the assets and names the
/// missing mesh (`IndexAssets`, `user_model.cc:1889-1899`). This crate refuses
/// the `fromto` as the classes are applied, before it checks the names, and
/// names it: ledger L353. `(what, document, MuJoCo 3.5.0's reason, this
/// crate's error)`, measured 2026-11-19 (RL-9d's start set, `a04`).
const ASSET_COSTS: &[(&str, &str, &str, &str)] = &[(
    "a box's fromto naming a mesh no mesh holds",
    r#"<mujoco><asset><mesh name="m" vertex="0 0 0  1 0 0  0 1 0  0 0 1" face="0 2 1  0 1 3  0 3 2  1 2 3"/></asset><worldbody><body name="b" pos="0 0 1"><freejoint/><geom type="box" size=".1" fromto="0 0 0 0 0 1" mesh="nope"/></body></worldbody></mujoco>"#,
    "mesh 'nope' not found in geom 0",
    "invalid fromto on <geom>: fromto cannot be used with mesh geom (geom in body 'b')",
)];

#[test]
fn a_fromto_beside_a_missing_mesh_is_named_as_ledgered() {
    let mut moved = Vec::new();
    for &(what, xml, _mujoco, ours) in ASSET_COSTS {
        match load_model(xml) {
            Err(e) if e.to_string() == ours => {}
            Err(e) => moved.push(format!("{what}: {e}")),
            Ok(_) => moved.push(format!("{what}: loads")),
        }
    }
    assert!(moved.is_empty(), "{}", moved.join("\n"));
}

/// Two faults, one an equality's refusal Rigid-loading PR RL-10d made, where
/// this crate names the other fault than MuJoCo 3.5.0 (ledger L353): the
/// scalar-joint refusal, made in the builder's late block by its rule since
/// RL-6a, after the build's check of an element MuJoCo checks after it; an
/// equality's attribute that another element of `<equality>` takes, refused
/// where the parser reads it, before a fault this crate finds when it builds
/// an element written earlier (an actuator kind's attribute, which MuJoCo's
/// schema, walking the document in order, names first), or after a fault the
/// parser meets earlier where MuJoCo's schema names the later schema fault;
/// and a weld's form, checked where the parser reads it, before its class is
/// looked up. `(what, document, MuJoCo's reason, this crate's error)`,
/// measured 2026-11-26 (RL-10d's fix-diff pass).
const EQUALITY_COSTS: &[(&str, &str, &str, &str)] = &[
    (
        "a ball joint's coupling before a joint equality naming an unknown joint",
        r#"<mujoco model="m"><option timestep="0.002"/><worldbody><body name="a" pos="0 0 1"><joint name="j1" type="ball"/><geom name="ga" type="capsule" fromto="0 0 0 0.3 0 0" size="0.02"/><site name="s1" pos="0.1 0 0"/></body><body name="b" pos="0.6 0 1"><joint name="j2" type="hinge" axis="0 1 0"/><geom name="gb" type="capsule" fromto="0 0 0 0.3 0.05 0" size="0.02"/><site name="s2" pos="0.1 0 0"/></body></worldbody><equality><joint joint1="j1" joint2="j2"/><joint joint1="nope"/></equality></mujoco>"#,
        "only HINGE and SLIDE joint allowed in constraint",
        "unknown joint 'nope' (equality #1)",
    ),
    (
        "a ball joint's coupling beside a sensor naming an unknown joint",
        r#"<mujoco model="m"><option timestep="0.002"/><worldbody><body name="a" pos="0 0 1"><joint name="j1" type="ball"/><geom name="ga" type="capsule" fromto="0 0 0 0.3 0 0" size="0.02"/><site name="s1" pos="0.1 0 0"/></body><body name="b" pos="0.6 0 1"><joint name="j2" type="hinge" axis="0 1 0"/><geom name="gb" type="capsule" fromto="0 0 0 0.3 0.05 0" size="0.02"/><site name="s2" pos="0.1 0 0"/></body></worldbody><equality><joint joint1="j1" joint2="j2"/></equality><sensor><jointpos joint="nope"/></sensor></mujoco>"#,
        "only HINGE and SLIDE joint allowed in constraint",
        "unknown joint 'nope' (sensor #0)",
    ),
    (
        "a ball joint's coupling before a tendon equality naming an unknown tendon",
        r#"<mujoco model="m"><option timestep="0.002"/><worldbody><body name="a" pos="0 0 1"><joint name="j1" type="ball"/><geom name="ga" type="capsule" fromto="0 0 0 0.3 0 0" size="0.02"/><site name="s1" pos="0.1 0 0"/></body><body name="b" pos="0.6 0 1"><joint name="j2" type="hinge" axis="0 1 0"/><geom name="gb" type="capsule" fromto="0 0 0 0.3 0.05 0" size="0.02"/><site name="s2" pos="0.1 0 0"/></body></worldbody><equality><joint joint1="j1" joint2="j2"/><tendon tendon1="nope"/></equality></mujoco>"#,
        "only HINGE and SLIDE joint allowed in constraint",
        "unknown tendon 'nope' (equality #1)",
    ),
    (
        "a ball joint's coupling before a weld naming an unknown body",
        r#"<mujoco model="m"><option timestep="0.002"/><worldbody><body name="a" pos="0 0 1"><joint name="j1" type="ball"/><geom name="ga" type="capsule" fromto="0 0 0 0.3 0 0" size="0.02"/><site name="s1" pos="0.1 0 0"/></body><body name="b" pos="0.6 0 1"><joint name="j2" type="hinge" axis="0 1 0"/><geom name="gb" type="capsule" fromto="0 0 0 0.3 0.05 0" size="0.02"/><site name="s2" pos="0.1 0 0"/></body></worldbody><equality><joint joint1="j1" joint2="j2"/><weld body1="nope"/></equality></mujoco>"#,
        "only HINGE and SLIDE joint allowed in constraint",
        "unknown body 'nope' (equality #1)",
    ),
    (
        "a ball joint's coupling before a connect naming an unknown site",
        r#"<mujoco model="m"><option timestep="0.002"/><worldbody><body name="a" pos="0 0 1"><joint name="j1" type="ball"/><geom name="ga" type="capsule" fromto="0 0 0 0.3 0 0" size="0.02"/><site name="s1" pos="0.1 0 0"/></body><body name="b" pos="0.6 0 1"><joint name="j2" type="hinge" axis="0 1 0"/><geom name="gb" type="capsule" fromto="0 0 0 0.3 0.05 0" size="0.02"/><site name="s2" pos="0.1 0 0"/></body></worldbody><equality><joint joint1="j1" joint2="j2"/><connect site1="s1" site2="nope"/></equality></mujoco>"#,
        "only HINGE and SLIDE joint allowed in constraint",
        "unknown site 'nope' (equality #1)",
    ),
    (
        "a ball joint's coupling beside a tendon's user data too long",
        r#"<mujoco model="m"><option timestep="0.002"/><size nuser_tendon="1"/><worldbody><body name="a" pos="0 0 1"><joint name="j1" type="ball"/><geom name="ga" type="capsule" fromto="0 0 0 0.3 0 0" size="0.02"/><site name="s1" pos="0.1 0 0"/></body><body name="b" pos="0.6 0 1"><joint name="j2" type="hinge" axis="0 1 0"/><geom name="gb" type="capsule" fromto="0 0 0 0.3 0.05 0" size="0.02"/><site name="s2" pos="0.1 0 0"/></body></worldbody><equality><joint joint1="j1" joint2="j2"/></equality><tendon><spatial user="1 2"><site site="s1"/><site site="s2"/></spatial></tendon></mujoco>"#,
        "only HINGE and SLIDE joint allowed in constraint",
        "invalid user on <tendon>: user data exceeds nuser_tendon=1",
    ),
    (
        "a connect's unknown attribute written before its relpose",
        r#"<mujoco model="m"><option timestep="0.002"/><worldbody><body name="a" pos="0.3 0 1"><freejoint/><geom name="g" type="box" size="0.1 0.05 0.02" mass="1"/><site name="s" pos="-0.1 0 0"/></body><body name="c" pos="0 0 2"><joint name="h" type="hinge"/><geom size="0.05"/></body></worldbody><equality><connect body1="a" anchor="0 0 0" foo="1" relpose="0 0 0 1 0 0 0"/></equality></mujoco>"#,
        "Schema violation: unrecognized attribute: 'foo'",
        "unrecognized attribute 'relpose' on <connect> (line 1: equality > connect)",
    ),
    (
        "a connect's mixed form before a weld's polycoef",
        r#"<mujoco model="m"><option timestep="0.002"/><worldbody><body name="a" pos="0.3 0 1"><freejoint/><geom name="g" type="box" size="0.1 0.05 0.02" mass="1"/><site name="s" pos="-0.1 0 0"/></body><body name="c" pos="0 0 2"><joint name="h" type="hinge"/><geom size="0.05"/></body></worldbody><equality><connect body1="a" site2="s"/><weld body1="a" polycoef="0 1"/></equality></mujoco>"#,
        "Schema violation: unrecognized attribute: 'polycoef'",
        "invalid site2 on <connect>: body and site semantics cannot be mixed (line 1: equality > connect)",
    ),
    (
        "a geom's bad mass before a connect's relpose",
        r#"<mujoco model="m"><option timestep="0.002"/><worldbody><body name="a" pos="0.3 0 1"><freejoint/><geom name="g" type="box" size="0.1 0.05 0.02" mass="x"/><site name="s" pos="-0.1 0 0"/></body><body name="c" pos="0 0 2"><joint name="h" type="hinge"/><geom size="0.05"/></body></worldbody><equality><connect body1="a" anchor="0 0 0" relpose="0 0 0 1 0 0 0"/></equality></mujoco>"#,
        "Schema violation: unrecognized attribute: 'relpose'",
        "bad number \"x\" in attribute 'mass' on <geom> (line 1: worldbody > body 'a' > geom 'g')",
    ),
    (
        "a geom's unknown attribute before a connect's relpose",
        r#"<mujoco model="m"><option timestep="0.002"/><worldbody><body name="a" pos="0.3 0 1"><freejoint/><geom name="g" type="box" size="0.1 0.05 0.02" mass="1" foo="1"/><site name="s" pos="-0.1 0 0"/></body><body name="c" pos="0 0 2"><joint name="h" type="hinge"/><geom size="0.05"/></body></worldbody><equality><connect body1="a" anchor="0 0 0" relpose="0 0 0 1 0 0 0"/></equality></mujoco>"#,
        "Schema violation: unrecognized attribute: 'foo'",
        "unrecognized attribute 'relpose' on <connect> (line 1: equality > connect)",
    ),
    (
        "a motor's kp before a connect's relpose",
        r#"<mujoco model="m"><option timestep="0.002"/><worldbody><body name="a" pos="0.3 0 1"><freejoint/><geom name="g" type="box" size="0.1 0.05 0.02" mass="1"/><site name="s" pos="-0.1 0 0"/></body><body name="c" pos="0 0 2"><joint name="h" type="hinge"/><geom size="0.05"/></body></worldbody><actuator><motor joint="h" kp="1"/></actuator><equality><connect body1="a" anchor="0 0 0" relpose="0 0 0 1 0 0 0"/></equality></mujoco>"#,
        "Schema violation: unrecognized attribute: 'kp'",
        "unrecognized attribute 'relpose' on <connect> (line 1: equality > connect)",
    ),
    (
        "a default class's motor kp before a connect's relpose",
        r#"<mujoco model="m"><option timestep="0.002"/><default><motor kp="1"/></default><worldbody><body name="a" pos="0.3 0 1"><freejoint/><geom name="g" type="box" size="0.1 0.05 0.02" mass="1"/><site name="s" pos="-0.1 0 0"/></body><body name="c" pos="0 0 2"><joint name="h" type="hinge"/><geom size="0.05"/></body></worldbody><equality><connect body1="a" anchor="0 0 0" relpose="0 0 0 1 0 0 0"/></equality></mujoco>"#,
        "Schema violation: unrecognized attribute: 'kp'",
        "unrecognized attribute 'relpose' on <connect> (line 1: equality > connect)",
    ),
    (
        "a general actuator's kp before a connect's relpose",
        r#"<mujoco model="m"><option timestep="0.002"/><worldbody><body name="a" pos="0.3 0 1"><freejoint/><geom name="g" type="box" size="0.1 0.05 0.02" mass="1"/><site name="s" pos="-0.1 0 0"/></body><body name="c" pos="0 0 2"><joint name="h" type="hinge"/><geom size="0.05"/></body></worldbody><actuator><general joint="h" kp="1"/></actuator><equality><connect body1="a" anchor="0 0 0" relpose="0 0 0 1 0 0 0"/></equality></mujoco>"#,
        "Schema violation: unrecognized attribute: 'kp'",
        "unrecognized attribute 'relpose' on <connect> (line 1: equality > connect)",
    ),
    (
        "a position actuator's area before a connect's relpose",
        r#"<mujoco model="m"><option timestep="0.002"/><worldbody><body name="a" pos="0.3 0 1"><freejoint/><geom name="g" type="box" size="0.1 0.05 0.02" mass="1"/><site name="s" pos="-0.1 0 0"/></body><body name="c" pos="0 0 2"><joint name="h" type="hinge"/><geom size="0.05"/></body></worldbody><actuator><position joint="h" area="1"/></actuator><equality><connect body1="a" anchor="0 0 0" relpose="0 0 0 1 0 0 0"/></equality></mujoco>"#,
        "Schema violation: unrecognized attribute: 'area'",
        "unrecognized attribute 'relpose' on <connect> (line 1: equality > connect)",
    ),
    (
        "a motor's kp before a weld's polycoef",
        r#"<mujoco model="m"><option timestep="0.002"/><worldbody><body name="a" pos="0.3 0 1"><freejoint/><geom name="g" type="box" size="0.1 0.05 0.02" mass="1"/><site name="s" pos="-0.1 0 0"/></body><body name="c" pos="0 0 2"><joint name="h" type="hinge"/><geom size="0.05"/></body></worldbody><actuator><motor joint="h" kp="1"/></actuator><equality><weld body1="a" polycoef="1"/></equality></mujoco>"#,
        "Schema violation: unrecognized attribute: 'kp'",
        "unrecognized attribute 'polycoef' on <weld> (line 1: equality > weld)",
    ),
    (
        "a motor's kp before a joint equality's torquescale",
        r#"<mujoco model="m"><option timestep="0.002"/><worldbody><body name="a" pos="0.3 0 1"><freejoint/><geom name="g" type="box" size="0.1 0.05 0.02" mass="1"/><site name="s" pos="-0.1 0 0"/></body><body name="c" pos="0 0 2"><joint name="h" type="hinge"/><geom size="0.05"/></body></worldbody><actuator><motor joint="h" kp="1"/></actuator><equality><joint joint1="h" torquescale="1"/></equality></mujoco>"#,
        "Schema violation: unrecognized attribute: 'kp'",
        "unrecognized attribute 'torquescale' on <joint> (line 1: equality > joint)",
    ),
    (
        "a weld's unknown class beside its mixed form",
        r#"<mujoco model="m"><option timestep="0.002"/><worldbody><site name="w" pos="0 0 1"/><body name="a" pos="0.3 0 1"><freejoint/><geom type="box" size="0.1 0.05 0.02" mass="1"/><site name="s" pos="-0.1 0 0"/></body></worldbody><equality><weld class="nope" body1="a" site1="s"/></equality></mujoco>"#,
        "unknown default class name 'nope'",
        "invalid site1 on <weld>: body and site semantics cannot be mixed (line 1: equality > weld)",
    ),
];

#[test]
fn a_fault_beside_an_equalitys_refusal_is_named_as_ledgered() {
    let mut moved = Vec::new();
    for &(what, xml, _mujoco, ours) in EQUALITY_COSTS {
        match load_model(xml) {
            Err(e) if e.to_string() == ours => {}
            Err(e) => moved.push(format!("{what}: {e}")),
            Ok(_) => moved.push(format!("{what}: loads")),
        }
    }
    assert!(moved.is_empty(), "{}", moved.join("\n"));
}

/// Two faults, one a height field's (no data, or a top of 0), and three
/// faults with a flex's, where this crate and MuJoCo 3.5.0 name different
/// ones (ledger L353): an actuator's or a tendon's reference or a tendon's
/// range, which this crate's validation resolves before it builds the model
/// and MuJoCo after the height fields; a skin's bone and a numeric's size,
/// which MuJoCo meets before the height fields (its skins compile first, its
/// reader reads the numeric) and this crate after; a flex's, which MuJoCo
/// compiles before the height fields and this crate builds after the
/// elements MuJoCo compiles after them, so of three faults it names the
/// height field's. `(what, document, MuJoCo's reason, this crate's error)`,
/// measured 2026-11-27 (RL-11b's review round 1, RA's documents; the
/// flexes' its fix-diff pass, FA's).
const HFIELD_COMPILE_COSTS: &[(&str, &str, &str, &str)] = &[
    (
        "a height field with no data beside actuator joint",
        r#"<mujoco model="f"><asset><hfield name="h" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom name="g" size="0.1"/><site name="s"/></body></worldbody><actuator><motor joint="nope"/></actuator></mujoco>"#,
        "hfield not specified",
        "unknown joint 'nope' (actuator #0)",
    ),
    (
        "a height field with no data beside actuator site",
        r#"<mujoco model="f"><asset><hfield name="h" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom name="g" size="0.1"/><site name="s"/></body></worldbody><actuator><motor site="nope"/></actuator></mujoco>"#,
        "hfield not specified",
        "unknown site 'nope' (actuator #0)",
    ),
    (
        "a height field with no data beside numeric size",
        r#"<mujoco model="f"><asset><hfield name="h" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom name="g" size="0.1"/><site name="s"/></body></worldbody><custom><numeric name="n" size="1" data="1 2"/></custom></mujoco>"#,
        "attribute 'data' has too much data",
        "hfield not specified (hfield 'h')",
    ),
    (
        "a height field with no data beside skin bone",
        r#"<mujoco model="f"><asset><hfield name="h" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom name="g" size="0.1"/><site name="s"/></body></worldbody><deformable><skin vertex="0 0 0 1 0 0 0 1 0" face="0 1 2"><bone body="nope" bindpos="0 0 0" bindquat="1 0 0 0" vertid="0" vertweight="1"/></skin></deformable></mujoco>"#,
        "unknown body 'nope' in skin",
        "hfield not specified (hfield 'h')",
    ),
    (
        "a height field with no data beside tendon joint",
        r#"<mujoco model="f"><asset><hfield name="h" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom name="g" size="0.1"/><site name="s"/></body></worldbody><tendon><fixed><joint joint="nope" coef="1"/></fixed></tendon></mujoco>"#,
        "hfield not specified",
        "unknown joint 'nope' (tendon #0)",
    ),
    (
        "a height field with no data beside tendon range",
        r#"<mujoco model="f"><asset><hfield name="h" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom name="g" size="0.1"/><site name="s"/></body></worldbody><tendon><fixed limited="true" range="1 0"><joint joint="j" coef="1"/></fixed></tendon></mujoco>"#,
        "hfield not specified",
        "invalid range on <tendon>: invalid limits in tendon (tendon #0)",
    ),
    (
        "a height field with no data beside tendon site",
        r#"<mujoco model="f"><asset><hfield name="h" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom name="g" size="0.1"/><site name="s"/></body></worldbody><tendon><spatial><site site="s"/><site site="nope"/></spatial></tendon></mujoco>"#,
        "hfield not specified",
        "unknown site 'nope' (tendon #0)",
    ),
    (
        "a height field's top of 0 beside actuator joint",
        r#"<mujoco model="f"><asset><hfield name="h" nrow="2" ncol="2" size="1 1 0 0.1" elevation="0 0 1 1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom name="g" size="0.1"/><site name="s"/></body></worldbody><actuator><motor joint="nope"/></actuator></mujoco>"#,
        "size parameter is not positive in hfield",
        "unknown joint 'nope' (actuator #0)",
    ),
    (
        "a height field's top of 0 beside actuator site",
        r#"<mujoco model="f"><asset><hfield name="h" nrow="2" ncol="2" size="1 1 0 0.1" elevation="0 0 1 1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom name="g" size="0.1"/><site name="s"/></body></worldbody><actuator><motor site="nope"/></actuator></mujoco>"#,
        "size parameter is not positive in hfield",
        "unknown site 'nope' (actuator #0)",
    ),
    (
        "a height field's top of 0 beside numeric size",
        r#"<mujoco model="f"><asset><hfield name="h" nrow="2" ncol="2" size="1 1 0 0.1" elevation="0 0 1 1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom name="g" size="0.1"/><site name="s"/></body></worldbody><custom><numeric name="n" size="1" data="1 2"/></custom></mujoco>"#,
        "attribute 'data' has too much data",
        "size parameter is not positive in hfield (hfield 'h')",
    ),
    (
        "a height field's top of 0 beside skin bone",
        r#"<mujoco model="f"><asset><hfield name="h" nrow="2" ncol="2" size="1 1 0 0.1" elevation="0 0 1 1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom name="g" size="0.1"/><site name="s"/></body></worldbody><deformable><skin vertex="0 0 0 1 0 0 0 1 0" face="0 1 2"><bone body="nope" bindpos="0 0 0" bindquat="1 0 0 0" vertid="0" vertweight="1"/></skin></deformable></mujoco>"#,
        "unknown body 'nope' in skin",
        "size parameter is not positive in hfield (hfield 'h')",
    ),
    (
        "a height field's top of 0 beside tendon joint",
        r#"<mujoco model="f"><asset><hfield name="h" nrow="2" ncol="2" size="1 1 0 0.1" elevation="0 0 1 1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom name="g" size="0.1"/><site name="s"/></body></worldbody><tendon><fixed><joint joint="nope" coef="1"/></fixed></tendon></mujoco>"#,
        "size parameter is not positive in hfield",
        "unknown joint 'nope' (tendon #0)",
    ),
    (
        "a height field's top of 0 beside tendon range",
        r#"<mujoco model="f"><asset><hfield name="h" nrow="2" ncol="2" size="1 1 0 0.1" elevation="0 0 1 1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom name="g" size="0.1"/><site name="s"/></body></worldbody><tendon><fixed limited="true" range="1 0"><joint joint="j" coef="1"/></fixed></tendon></mujoco>"#,
        "size parameter is not positive in hfield",
        "invalid range on <tendon>: invalid limits in tendon (tendon #0)",
    ),
    (
        "a height field's top of 0 beside tendon site",
        r#"<mujoco model="f"><asset><hfield name="h" nrow="2" ncol="2" size="1 1 0 0.1" elevation="0 0 1 1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom name="g" size="0.1"/><site name="s"/></body></worldbody><tendon><spatial><site site="s"/><site site="nope"/></spatial></tendon></mujoco>"#,
        "size parameter is not positive in hfield",
        "unknown site 'nope' (tendon #0)",
    ),
    (
        "a flex's unknown body, a height field with no data and an actuator's delay with no history",
        r#"<mujoco model="f"><asset><hfield name="h" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><camera name="c" pos="0 0 3"/><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom name="g" size="0.1"/><site name="s"/><body name="b2" pos="0 0 0.5"><joint name="j2" type="slide"/><geom name="g2" size="0.1"/><geom name="gx" type="box" size="0.1 0.1 0.1"/><site name="s2" pos="0.2 0 0"/><body name="b3" pos="0 0 0.5"><joint name="jb" type="ball"/><geom name="g3" size="0.1"/></body></body></body></worldbody><deformable><flex name="fl" dim="1" body="b nope" vertex="0 0 0 0 0 0" element="0 1"/></deformable><actuator><motor joint="j" delay="0.1"/></actuator></mujoco>"#,
        "unknown body 'nope' in flex",
        "hfield not specified (hfield 'h')",
    ),
    (
        "a flex's element past its vertices, a height field with no data and an exclude's unknown body",
        r#"<mujoco model="f"><asset><hfield name="h" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><camera name="c" pos="0 0 3"/><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom name="g" size="0.1"/><site name="s"/><body name="b2" pos="0 0 0.5"><joint name="j2" type="slide"/><geom name="g2" size="0.1"/><geom name="gx" type="box" size="0.1 0.1 0.1"/><site name="s2" pos="0.2 0 0"/><body name="b3" pos="0 0 0.5"><joint name="jb" type="ball"/><geom name="g3" size="0.1"/></body></body></body></worldbody><deformable><flex name="fl" dim="1" body="b b2" vertex="0 0 0 0 0 0" element="0 5"/></deformable><contact><exclude body1="b" body2="nope"/></contact></mujoco>"#,
        "elem vertex id out of range",
        "hfield not specified (hfield 'h')",
    ),
    (
        "a flex's pins under bending, a height field with no data and a sensor's delay with no history",
        r#"<mujoco model="f"><asset><hfield name="h" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><camera name="c" pos="0 0 3"/><body name="b" pos="0 0 1"><joint name="j" type="hinge"/><geom name="g" size="0.1"/><site name="s"/><body name="b2" pos="0 0 0.5"><joint name="j2" type="slide"/><geom name="g2" size="0.1"/><geom name="gx" type="box" size="0.1 0.1 0.1"/><site name="s2" pos="0.2 0 0"/><body name="b3" pos="0 0 0.5"><joint name="jb" type="ball"/><geom name="g3" size="0.1"/></body></body></body></worldbody><deformable><flex name="fl" dim="2" body="b b2 b3" vertex="0 0 0 0 0 0 0 0 0" element="0 1 2"><elasticity young="1" thickness="-1" elastic2d="bend"/></flex></deformable><sensor><jointpos joint="j" delay="0.1"/></sensor></mujoco>"#,
        "pins are not supported for bending",
        "hfield not specified (hfield 'h')",
    ),
];

#[test]
fn a_height_field_beside_a_fault_this_crate_meets_first_is_named_as_ledgered() {
    let mut moved = Vec::new();
    for &(what, xml, _mujoco, ours) in HFIELD_COMPILE_COSTS {
        match load_model(xml) {
            Err(e) if e.to_string() == ours => {}
            Err(e) => moved.push(format!("{what}: {e}")),
            Ok(_) => moved.push(format!("{what}: loads")),
        }
    }
    assert!(moved.is_empty(), "{}", moved.join("\n"));
}
