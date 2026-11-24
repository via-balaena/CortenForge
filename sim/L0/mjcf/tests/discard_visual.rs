//! `discardvisual` as MuJoCo 3.5.0 makes it (book Rigid-loading L39; ledger
//! L336, L342): every geom and mesh built and checked, each body's mass,
//! inertia and inertial frame from all its geoms, the visual ones among them,
//! and only then the visual geoms deleted, a geom a pair, a tendon or a sensor
//! names kept, and the meshes no kept geom names deleted.
//!
//! Every expected value was printed by MuJoCo 3.5.0 on the census's unfused
//! oracle (2026-11-23).

// The values are MuJoCo's, printed to the last digit.
#![allow(
    clippy::unwrap_used,
    clippy::unreadable_literal,
    clippy::approx_constant
)]

use sim_core::{GeomType, MjObjectType, WrapType};
use sim_mjcf::load_model;

/// The fields `discardvisual` decides, as MuJoCo 3.5.0 compiles them.
struct Model {
    ngeom: usize,
    nmesh: usize,
    /// MuJoCo's `mjtGeom` numbers.
    geom_type: &'static [i32],
    geom_body: &'static [usize],
    /// Each geom's mesh, -1 for none.
    geom_mesh: &'static [i64],
    geom_size: &'static [[f64; 3]],
    geom_pos: &'static [[f64; 3]],
    body_mass: &'static [f64],
    body_inertia: &'static [[f64; 3]],
    body_ipos: &'static [[f64; 3]],
    /// (w, x, y, z).
    body_iquat: &'static [[f64; 4]],
}

/// What MuJoCo 3.5.0 makes of a document.
enum Expect {
    Loads(Model),
    /// Refused, in words these hold: the geom by MuJoCo's id where this
    /// crate's refusal names one.
    Refused(&'static str),
}

const CASES: &[(&str, Expect)] = &[
    // two of five spheres visual: their mass kept, the geoms deleted
    (
        r#"<mujoco><compiler discardvisual="true"/><worldbody><body name="b" pos="0 0 1"><freejoint/><geom type="sphere" size="0.1" pos="0.0 0 0" /><geom type="sphere" size="0.1" pos="0.3 0 0" contype="0" conaffinity="0"/><geom type="sphere" size="0.1" pos="0.6 0 0" /><geom type="sphere" size="0.1" pos="0.8999999999999999 0 0" contype="0" conaffinity="0"/><geom type="sphere" size="0.1" pos="1.2 0 0" /></body></worldbody></mujoco>"#,
        Expect::Loads(Model {
            ngeom: 3,
            nmesh: 0,
            geom_type: &[2, 2, 2],
            geom_body: &[1, 1, 1],
            geom_mesh: &[-1, -1, -1],
            geom_size: &[[0.1, 0.0, 0.0], [0.1, 0.0, 0.0], [0.1, 0.0, 0.0]],
            geom_pos: &[[0.0, 0.0, 0.0], [0.6, 0.0, 0.0], [1.2, 0.0, 0.0]],
            body_mass: &[0.0, 20.94395102393196],
            body_inertia: &[
                [0.0, 0.0, 0.0],
                [3.8536869884034797, 3.8536869884034797, 0.08377580409572785],
            ],
            body_ipos: &[[0.0, 0.0, 0.0], [0.6, 0.0, 0.0]],
            body_iquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [
                    0.5000000000000007,
                    0.5000000000000007,
                    0.5000000000000007,
                    0.5000000000000007,
                ],
            ],
        }),
    ),
    // a moving body whose only geom is visual: its mass and inertial frame kept
    (
        r#"<mujoco><compiler discardvisual="true"/><worldbody><body name="b" pos="0 0 1"><freejoint/><geom type="sphere" size="0.1" pos="0.2 0 0" contype="0" conaffinity="0"/></body></worldbody></mujoco>"#,
        Expect::Loads(Model {
            ngeom: 0,
            nmesh: 0,
            geom_type: &[],
            geom_body: &[],
            geom_mesh: &[],
            geom_size: &[],
            geom_pos: &[],
            body_mass: &[0.0, 4.188790204786391],
            body_inertia: &[
                [0.0, 0.0, 0.0],
                [
                    0.01675516081914557,
                    0.01675516081914557,
                    0.01675516081914557,
                ],
            ],
            body_ipos: &[[0.0, 0.0, 0.0], [0.2, 0.0, 0.0]],
            body_iquat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
        }),
    ),
    // a visual geom a tendon wraps: kept
    (
        r#"<mujoco><compiler discardvisual="true"/><worldbody><body name="b" pos="0 0 1"><joint type="hinge" axis="0 1 0"/><geom type="capsule" size="0.05 0.3"/><site name="s1" pos="0.2 0 0.3"/><site name="s2" pos="-0.2 0 -0.3"/><geom name="w" type="cylinder" size="0.05 0.1" contype="0" conaffinity="0"/></body></worldbody><tendon><spatial name="t"><site site="s1"/><geom geom="w"/><site site="s2"/></spatial></tendon></mujoco>"#,
        Expect::Loads(Model {
            ngeom: 2,
            nmesh: 0,
            geom_type: &[3, 5],
            geom_body: &[1, 1],
            geom_mesh: &[-1, -1],
            geom_size: &[[0.05, 0.3, 0.0], [0.05, 0.1, 0.0]],
            geom_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            body_mass: &[0.0, 6.806784082777886],
            body_inertia: &[
                [0.0, 0.0, 0.0],
                [
                    0.20407262278943702,
                    0.20407262278943702,
                    0.008377580409572784,
                ],
            ],
            body_ipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            body_iquat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
        }),
    ),
    // a visual geom a pair names: kept
    (
        r#"<mujoco><compiler discardvisual="true"/><worldbody><geom name="g1" type="plane" size="1 1 0.1"/><body name="b" pos="0 0 0.5"><freejoint/><geom type="sphere" size="0.1"/><geom name="g2" type="sphere" size="0.12" contype="0" conaffinity="0"/></body></worldbody><contact><pair geom1="g1" geom2="g2"/></contact></mujoco>"#,
        Expect::Loads(Model {
            ngeom: 3,
            nmesh: 0,
            geom_type: &[0, 2, 2],
            geom_body: &[0, 1, 1],
            geom_mesh: &[-1, -1, -1],
            geom_size: &[[1.0, 1.0, 0.1], [0.1, 0.0, 0.0], [0.12, 0.0, 0.0]],
            geom_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            body_mass: &[0.0, 11.427019678657274],
            body_inertia: &[
                [0.0, 0.0, 0.0],
                [
                    0.058447362588641846,
                    0.058447362588641846,
                    0.058447362588641846,
                ],
            ],
            body_ipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            body_iquat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
        }),
    ),
    // a visual geom a distance sensor names: kept
    (
        r#"<mujoco><compiler discardvisual="true"/><worldbody><geom name="g1" type="plane" size="1 1 0.1"/><body name="b" pos="0 0 0.5"><freejoint/><geom type="sphere" size="0.1"/><geom name="g2" type="sphere" size="0.12" contype="0" conaffinity="0"/></body></worldbody><sensor><distance geom1="g1" geom2="g2"/></sensor></mujoco>"#,
        Expect::Loads(Model {
            ngeom: 3,
            nmesh: 0,
            geom_type: &[0, 2, 2],
            geom_body: &[0, 1, 1],
            geom_mesh: &[-1, -1, -1],
            geom_size: &[[1.0, 1.0, 0.1], [0.1, 0.0, 0.0], [0.12, 0.0, 0.0]],
            geom_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            body_mass: &[0.0, 11.427019678657274],
            body_inertia: &[
                [0.0, 0.0, 0.0],
                [
                    0.058447362588641846,
                    0.058447362588641846,
                    0.058447362588641846,
                ],
            ],
            body_ipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            body_iquat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
        }),
    ),
    // a mesh no geom names: deleted
    (
        r#"<mujoco><compiler discardvisual="true"/><asset><mesh name="m" vertex="0 0 0 1 0 0 0 1 0 0 0 1 1 1 1 1 0 1 0 1 1 1 1 0"/></asset><worldbody><body name="b" pos="0 0 1"><freejoint/><geom type="sphere" size="0.1"/></body></worldbody></mujoco>"#,
        Expect::Loads(Model {
            ngeom: 1,
            nmesh: 0,
            geom_type: &[2],
            geom_body: &[1],
            geom_mesh: &[-1],
            geom_size: &[[0.1, 0.0, 0.0]],
            geom_pos: &[[0.0, 0.0, 0.0]],
            body_mass: &[0.0, 4.188790204786391],
            body_inertia: &[
                [0.0, 0.0, 0.0],
                [
                    0.01675516081914557,
                    0.01675516081914557,
                    0.01675516081914557,
                ],
            ],
            body_ipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            body_iquat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
        }),
    ),
    // a mesh a visual and a colliding geom name: kept
    (
        r#"<mujoco><compiler discardvisual="true"/><asset><mesh name="m" vertex="0 0 0 1 0 0 0 1 0 0 0 1 1 1 1 1 0 1 0 1 1 1 1 0"/></asset><worldbody><body name="b" pos="0 0 1"><freejoint/><geom type="mesh" mesh="m"/><geom type="mesh" mesh="m" pos="1 0 0" contype="0" conaffinity="0"/></body></worldbody></mujoco>"#,
        Expect::Loads(Model {
            ngeom: 1,
            nmesh: 1,
            geom_type: &[7],
            geom_body: &[1],
            geom_mesh: &[0],
            geom_size: &[[0.5, 0.5, 0.5000000000000001]],
            geom_pos: &[[0.5, 0.49999999999999994, 0.4999999999999999]],
            body_mass: &[0.0, 2000.0],
            body_inertia: &[
                [0.0, 0.0, 0.0],
                [833.3333333333333, 833.3333333333333, 333.3333333333332],
            ],
            body_ipos: &[
                [0.0, 0.0, 0.0],
                [1.0, 0.49999999999999994, 0.4999999999999999],
            ],
            body_iquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [
                    0.5000000000000007,
                    0.5000000000000007,
                    0.5000000000000007,
                    0.5000000000000007,
                ],
            ],
        }),
    ),
    // an <inertial> beside a visual geom: the inertial's
    (
        r#"<mujoco><compiler discardvisual="true"/><worldbody><body name="b" pos="0 0 1"><freejoint/><inertial pos="0 0 0" mass="2" diaginertia="0.1 0.1 0.1"/><geom type="sphere" size="0.1"/><geom type="sphere" size="0.1" pos="0.3 0 0" contype="0" conaffinity="0"/></body></worldbody></mujoco>"#,
        Expect::Loads(Model {
            ngeom: 1,
            nmesh: 0,
            geom_type: &[2],
            geom_body: &[1],
            geom_mesh: &[-1],
            geom_size: &[[0.1, 0.0, 0.0]],
            geom_pos: &[[0.0, 0.0, 0.0]],
            body_mass: &[0.0, 2.0],
            body_inertia: &[[0.0, 0.0, 0.0], [0.1, 0.1, 0.1]],
            body_ipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            body_iquat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
        }),
    ),
    // a visual geom a frame sensor names: kept
    (
        r#"<mujoco><compiler discardvisual="true"/><worldbody><body name="b" pos="0 0 1"><freejoint/><geom type="sphere" size="0.1"/><geom name="v" type="sphere" size="0.1" pos="0.3 0 0" contype="0" conaffinity="0"/></body></worldbody><sensor><framepos objtype="geom" objname="v"/></sensor></mujoco>"#,
        Expect::Loads(Model {
            ngeom: 2,
            nmesh: 0,
            geom_type: &[2, 2],
            geom_body: &[1, 1],
            geom_mesh: &[-1, -1],
            geom_size: &[[0.1, 0.0, 0.0], [0.1, 0.0, 0.0]],
            geom_pos: &[[0.0, 0.0, 0.0], [0.3, 0.0, 0.0]],
            body_mass: &[0.0, 8.377580409572783],
            body_inertia: &[
                [0.0, 0.0, 0.0],
                [
                    0.22200588085367873,
                    0.22200588085367873,
                    0.03351032163829114,
                ],
            ],
            body_ipos: &[[0.0, 0.0, 0.0], [0.15, 0.0, 0.0]],
            body_iquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [
                    0.5000000000000007,
                    0.5000000000000007,
                    0.5000000000000007,
                    0.5000000000000007,
                ],
            ],
        }),
    ),
    // visual geoms and a mesh among kept ones: each renumbered
    (
        r#"<mujoco><compiler discardvisual="true"/><asset><mesh name="m1" vertex="0 0 0 1 0 0 0 1 0 0 0 1 1 1 1 1 0 1 0 1 1 1 1 0"/><mesh name="m2" vertex="0 0 0 1 0 0 0 1 0 0 0 1 1 1 1 1 0 1 0 1 1 1 1 0"/></asset><worldbody><body name="a" pos="0 0 1"><freejoint/><geom type="mesh" mesh="m1" contype="0" conaffinity="0"/><geom type="box" size="0.1 0.1 0.1"/><geom type="mesh" mesh="m2"/><geom type="sphere" size="0.05" contype="0" conaffinity="0"/></body><body name="c" pos="1 0 1"><freejoint/><geom type="mesh" mesh="m2"/></body></worldbody><sensor><framepos objtype="body" objname="c"/></sensor></mujoco>"#,
        Expect::Loads(Model {
            ngeom: 3,
            nmesh: 1,
            geom_type: &[6, 7, 7],
            geom_body: &[1, 1, 2],
            geom_mesh: &[-1, 0, 0],
            geom_size: &[
                [0.1, 0.1, 0.1],
                [0.5, 0.5, 0.5000000000000001],
                [0.5, 0.5, 0.5000000000000001],
            ],
            geom_pos: &[
                [0.0, 0.0, 0.0],
                [0.5, 0.49999999999999994, 0.4999999999999999],
                [0.5, 0.49999999999999994, 0.4999999999999999],
            ],
            body_mass: &[0.0, 2008.5235987755982, 1000.0],
            body_inertia: &[
                [0.0, 0.0, 0.0],
                [339.75276056354136, 339.75276056354136, 333.38719026544214],
                [166.6666666666666, 166.66666666666663, 166.6666666666666],
            ],
            body_ipos: &[
                [0.0, 0.0, 0.0],
                [0.49787814323396695, 0.4978781432339669, 0.4978781432339668],
                [0.5, 0.49999999999999994, 0.4999999999999999],
            ],
            body_iquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [
                    0.8804762392171492,
                    -0.2798481423331215,
                    0.3647051996310009,
                    0.11591689595929522,
                ],
                [1.0, 0.0, 0.0, 0.0],
            ],
        }),
    ),
    // the same with meshes of different sizes: the kept mesh's data
    (
        r#"<mujoco><compiler discardvisual="true"/><asset><mesh name="m1" vertex="0 0 0 1 0 0 0 1 0 0 0 1 1 1 1 1 0 1 0 1 1 1 1 0"/><mesh name="m2" vertex="0 0 0 2 0 0 0 1 0 0 0 3 2 1 3 2 0 3 0 1 3 2 1 0"/></asset><worldbody><body name="a" pos="0 0 1"><freejoint/><geom type="mesh" mesh="m1" contype="0" conaffinity="0"/><geom type="box" size="0.1 0.1 0.1"/><geom type="mesh" mesh="m2"/><geom type="sphere" size="0.05" contype="0" conaffinity="0"/></body><body name="c" pos="1 0 1"><freejoint/><geom type="mesh" mesh="m2"/></body></worldbody><sensor><framepos objtype="body" objname="c"/></sensor></mujoco>"#,
        Expect::Loads(Model {
            ngeom: 3,
            nmesh: 1,
            geom_type: &[6, 7, 7],
            geom_body: &[1, 1, 2],
            geom_mesh: &[-1, 0, 0],
            geom_size: &[
                [0.1, 0.1, 0.1],
                [0.5000000000000007, 1.0000000000000013, 1.500000000000002],
                [0.5000000000000007, 1.0000000000000013, 1.500000000000002],
            ],
            geom_pos: &[[0.0, 0.0, 0.0], [1.0, 0.5, 1.5], [1.0, 0.5, 1.5]],
            body_mass: &[0.0, 7008.523598775599, 6000.0],
            body_inertia: &[
                [0.0, 0.0, 0.0],
                [7761.183432106294, 6101.761301162238, 2830.3715483750384],
                [6500.000000000001, 5000.000000000002, 2500.0],
            ],
            body_ipos: &[
                [0.0, 0.0, 0.0],
                [0.9274421222089573, 0.49939191195866933, 1.3554923324592454],
                [1.0, 0.5, 1.5],
            ],
            body_iquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [
                    0.7047767029049052,
                    0.04762815594028887,
                    0.0484330111872577,
                    0.7061696688697092,
                ],
                [0.707106781186548, 0.0, 0.0, 0.707106781186548],
            ],
        }),
    ),
    // discardvisual off: every geom kept
    (
        r#"<mujoco><compiler discardvisual="false"/><worldbody><body name="b" pos="0 0 1"><freejoint/><geom type="sphere" size="0.1" pos="0.0 0 0" /><geom type="sphere" size="0.1" pos="0.3 0 0" contype="0" conaffinity="0"/><geom type="sphere" size="0.1" pos="0.6 0 0" /><geom type="sphere" size="0.1" pos="0.8999999999999999 0 0" contype="0" conaffinity="0"/><geom type="sphere" size="0.1" pos="1.2 0 0" /></body></worldbody></mujoco>"#,
        Expect::Loads(Model {
            ngeom: 5,
            nmesh: 0,
            geom_type: &[2, 2, 2, 2, 2],
            geom_body: &[1, 1, 1, 1, 1],
            geom_mesh: &[-1, -1, -1, -1, -1],
            geom_size: &[
                [0.1, 0.0, 0.0],
                [0.1, 0.0, 0.0],
                [0.1, 0.0, 0.0],
                [0.1, 0.0, 0.0],
                [0.1, 0.0, 0.0],
            ],
            geom_pos: &[
                [0.0, 0.0, 0.0],
                [0.3, 0.0, 0.0],
                [0.6, 0.0, 0.0],
                [0.8999999999999999, 0.0, 0.0],
                [1.2, 0.0, 0.0],
            ],
            body_mass: &[0.0, 20.94395102393196],
            body_inertia: &[
                [0.0, 0.0, 0.0],
                [3.8536869884034797, 3.8536869884034797, 0.08377580409572785],
            ],
            body_ipos: &[[0.0, 0.0, 0.0], [0.6, 0.0, 0.0]],
            body_iquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [
                    0.5000000000000007,
                    0.5000000000000007,
                    0.5000000000000007,
                    0.5000000000000007,
                ],
            ],
        }),
    ),
    // a class makes a geom visual
    (
        r#"<mujoco><compiler discardvisual="true"/><default><default class="vis"><geom contype="0" conaffinity="0"/></default></default><worldbody><body name="b" pos="0 0 1"><freejoint/><geom type="sphere" size="0.1"/><geom class="vis" type="sphere" size="0.1" pos="0.3 0 0"/></body></worldbody></mujoco>"#,
        Expect::Loads(Model {
            ngeom: 1,
            nmesh: 0,
            geom_type: &[2],
            geom_body: &[1],
            geom_mesh: &[-1],
            geom_size: &[[0.1, 0.0, 0.0]],
            geom_pos: &[[0.0, 0.0, 0.0]],
            body_mass: &[0.0, 8.377580409572783],
            body_inertia: &[
                [0.0, 0.0, 0.0],
                [
                    0.22200588085367873,
                    0.22200588085367873,
                    0.03351032163829114,
                ],
            ],
            body_ipos: &[[0.0, 0.0, 0.0], [0.15, 0.0, 0.0]],
            body_iquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [
                    0.5000000000000007,
                    0.5000000000000007,
                    0.5000000000000007,
                    0.5000000000000007,
                ],
            ],
        }),
    ),
    // inertiafromgeom true beside an <inertial>: the visual geom's mass counts
    (
        r#"<mujoco><compiler discardvisual="true" inertiafromgeom="true"/><worldbody><body name="b" pos="0 0 1"><freejoint/><inertial pos="0 0 0" mass="2" diaginertia="0.1 0.1 0.1"/><geom type="sphere" size="0.1"/><geom type="sphere" size="0.1" pos="0.3 0 0" contype="0" conaffinity="0"/></body></worldbody></mujoco>"#,
        Expect::Loads(Model {
            ngeom: 1,
            nmesh: 0,
            geom_type: &[2],
            geom_body: &[1],
            geom_mesh: &[-1],
            geom_size: &[[0.1, 0.0, 0.0]],
            geom_pos: &[[0.0, 0.0, 0.0]],
            body_mass: &[0.0, 8.377580409572783],
            body_inertia: &[
                [0.0, 0.0, 0.0],
                [
                    0.22200588085367873,
                    0.22200588085367873,
                    0.03351032163829114,
                ],
            ],
            body_ipos: &[[0.0, 0.0, 0.0], [0.15, 0.0, 0.0]],
            body_iquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [
                    0.5000000000000007,
                    0.5000000000000007,
                    0.5000000000000007,
                    0.5000000000000007,
                ],
            ],
        }),
    ),
    // a turned visual box: its mass and inertia in the body's frame
    (
        r#"<mujoco><compiler discardvisual="true"/><worldbody><body name="b" pos="0 0 1"><freejoint/><geom type="box" size="0.1 0.2 0.3"/><geom type="box" size="0.3 0.1 0.1" pos="0.2 0.1 0" euler="0.3 0.4 0.5" contype="0" conaffinity="0"/></body></worldbody></mujoco>"#,
        Expect::Loads(Model {
            ngeom: 1,
            nmesh: 0,
            geom_type: &[6],
            geom_body: &[1],
            geom_mesh: &[-1],
            geom_size: &[[0.1, 0.2, 0.3]],
            geom_pos: &[[0.0, 0.0, 0.0]],
            body_mass: &[0.0, 72.0],
            body_inertia: &[
                [0.0, 0.0, 0.0],
                [3.1765047756322433, 2.4000891547534047, 2.2634060696143523],
            ],
            body_ipos: &[
                [0.0, 0.0, 0.0],
                [0.06666666666666668, 0.03333333333333334, 0.0],
            ],
            body_iquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [
                    0.38626717229899954,
                    0.3967672578755246,
                    0.5980044938723582,
                    0.5794514992564427,
                ],
            ],
        }),
    ),
    // a visual geom on the world
    (
        r#"<mujoco><compiler discardvisual="true"/><worldbody><geom type="plane" size="1 1 0.1"/><geom type="sphere" size="0.2" pos="0 0 1" contype="0" conaffinity="0"/><body name="b" pos="0 0 1"><freejoint/><geom type="sphere" size="0.1"/></body></worldbody></mujoco>"#,
        Expect::Loads(Model {
            ngeom: 2,
            nmesh: 0,
            geom_type: &[0, 2],
            geom_body: &[0, 1],
            geom_mesh: &[-1, -1],
            geom_size: &[[1.0, 1.0, 0.1], [0.1, 0.0, 0.0]],
            geom_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            body_mass: &[0.0, 4.188790204786391],
            body_inertia: &[
                [0.0, 0.0, 0.0],
                [
                    0.01675516081914557,
                    0.01675516081914557,
                    0.01675516081914557,
                ],
            ],
            body_ipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            body_iquat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
        }),
    ),
    // contype 0 alone is not visual
    (
        r#"<mujoco><compiler discardvisual="true"/><worldbody><body name="b" pos="0 0 1"><freejoint/><geom type="sphere" size="0.1"/><geom type="sphere" size="0.1" pos="0.3 0 0" contype="0"/></body></worldbody></mujoco>"#,
        Expect::Loads(Model {
            ngeom: 2,
            nmesh: 0,
            geom_type: &[2, 2],
            geom_body: &[1, 1],
            geom_mesh: &[-1, -1],
            geom_size: &[[0.1, 0.0, 0.0], [0.1, 0.0, 0.0]],
            geom_pos: &[[0.0, 0.0, 0.0], [0.3, 0.0, 0.0]],
            body_mass: &[0.0, 8.377580409572783],
            body_inertia: &[
                [0.0, 0.0, 0.0],
                [
                    0.22200588085367873,
                    0.22200588085367873,
                    0.03351032163829114,
                ],
            ],
            body_ipos: &[[0.0, 0.0, 0.0], [0.15, 0.0, 0.0]],
            body_iquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [
                    0.5000000000000007,
                    0.5000000000000007,
                    0.5000000000000007,
                    0.5000000000000007,
                ],
            ],
        }),
    ),
    // a cable whose geoms are all visual: its bodies keep their mass
    (
        r#"<mujoco>  <compiler discardvisual="true"/>  <extension><plugin plugin="mujoco.elasticity.cable"/></extension>  <worldbody>    <composite prefix="c" type="cable" curve="s" count="5 1 1" size="1" initial="none">      <joint kind="main" damping="0.1"/>      <geom type="capsule" size="0.01" contype="0" conaffinity="0" density="1000"/>    </composite>  </worldbody></mujoco>"#,
        Expect::Loads(Model {
            ngeom: 0,
            nmesh: 0,
            geom_type: &[],
            geom_body: &[],
            geom_mesh: &[],
            geom_size: &[],
            geom_pos: &[],
            body_mass: &[
                0.0,
                0.08272860654453122,
                0.08272860654453122,
                0.08272860654453122,
                0.08272860654453122,
            ],
            body_inertia: &[
                [0.0, 0.0, 0.0],
                [
                    0.0004805694282196307,
                    0.0004805694282196307,
                    4.094542425178697e-06,
                ],
                [
                    0.0004805694282196307,
                    0.0004805694282196307,
                    4.094542425178697e-06,
                ],
                [
                    0.0004805694282196307,
                    0.0004805694282196307,
                    4.094542425178697e-06,
                ],
                [
                    0.0004805694282196307,
                    0.0004805694282196307,
                    4.094542425178697e-06,
                ],
            ],
            body_ipos: &[
                [0.0, 0.0, 0.0],
                [0.125, 0.0, 0.0],
                [0.125, 0.0, 0.0],
                [0.125, 0.0, 0.0],
                [0.125, 0.0, 0.0],
            ],
            body_iquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [0.7071067811865476, 0.0, -0.7071067811865475, 0.0],
                [0.7071067811865476, 0.0, -0.7071067811865475, 0.0],
                [0.7071067811865476, 0.0, -0.7071067811865475, 0.0],
                [0.7071067811865476, 0.0, -0.7071067811865475, 0.0],
            ],
        }),
    ),
    // MuJoCo: "mass, inertia or density are negative in geom"
    (
        r#"<mujoco><compiler discardvisual="true"/><worldbody><body name="b" pos="0 0 1"><freejoint/><geom type="sphere" size="0.1"/><geom type="sphere" size="0.1" density="-5" contype="0" conaffinity="0"/></body></worldbody></mujoco>"#,
        Expect::Refused("mass, inertia or density are negative in geom (geom #1 in body 'b')"),
    ),
    // MuJoCo: "invalid condim in geom"
    (
        r#"<mujoco><compiler discardvisual="true"/><worldbody><body name="b" pos="0 0 1"><freejoint/><geom type="sphere" size="0.1"/><geom type="sphere" size="0.1" condim="2" contype="0" conaffinity="0"/></body></worldbody></mujoco>"#,
        Expect::Refused("invalid condim in geom (geom #1 in body 'b')"),
    ),
    // MuJoCo: "invalid condim in geom"
    (
        r#"<mujoco><compiler discardvisual="true"/><worldbody><body name="b" pos="0 0 1"><freejoint/><geom type="sphere" size="0.1" contype="0" conaffinity="0"/><geom type="sphere" size="0.1" condim="2"/></body></worldbody></mujoco>"#,
        Expect::Refused("invalid condim in geom (geom #1 in body 'b')"),
    ),
    // MuJoCo: "mass, inertia or density are negative in geom"
    (
        r#"<mujoco><compiler discardvisual="true"/><worldbody><body name="b" pos="0 0 1"><freejoint/><geom type="sphere" size="0.1"/><geom type="sphere" size="0.1" mass="-1" contype="0" conaffinity="0"/></body></worldbody></mujoco>"#,
        Expect::Refused("mass, inertia or density are negative in geom (geom #1 in body 'b')"),
    ),
    // MuJoCo: "size 0 must be positive in geom"
    (
        r#"<mujoco>  <compiler discardvisual="true" fitaabb="true"/>  <asset>    <mesh name="m" inertia="shell" vertex="0 0 0  1 0 0  0 1 0  1 1 0" face="0 1 2  1 3 2"/>  </asset>  <worldbody>    <body pos="0 0 1"><freejoint/><geom type="sphere" size=".1"/><geom type="box" mesh="m" contype="0" conaffinity="0"/></body>  </worldbody></mujoco>"#,
        Expect::Refused("size 0 must be positive (geom #1 in body #1)"),
    ),
    // MuJoCo: "plane only allowed in static bodies"
    (
        r#"<mujoco>  <compiler discardvisual="true"/>  <worldbody>    <body pos="0 0 1"><freejoint/><geom type="sphere" size=".1"/><geom type="plane" size="1 1 1" contype="0" conaffinity="0"/></body>  </worldbody></mujoco>"#,
        Expect::Refused("plane only allowed in static bodies (geom #1 in body #1)"),
    ),
    // MuJoCo: "fromto cannot be used with mesh geom"
    (
        r#"<mujoco>  <compiler discardvisual="true"/>  <asset>    <mesh name="m" vertex="0 0 0  1 0 0  0 1 0  0 0 1" face="0 2 1  0 1 3  0 3 2  1 2 3"/>  </asset>  <worldbody>    <body pos="0 0 1"><freejoint/><geom type="sphere" size=".1"/><geom type="capsule" mesh="m" fromto="0 0 0 0 0 0.3" size="0.05" contype="0" conaffinity="0"/></body>  </worldbody></mujoco>"#,
        Expect::Refused("fromto cannot be used with mesh geom"),
    ),
    // MuJoCo: "in face 1, vertex index 7 does not exist"
    (
        r#"<mujoco><compiler discardvisual="true"/><asset><mesh name="m" vertex="0 0 0 1 0 0 0 1 0 0 0 1" face="0 1 2 0 1 7 0 2 3 1 2 3"/></asset><worldbody><body name="b" pos="0 0 1"><freejoint/><geom type="sphere" size="0.1"/><geom type="mesh" mesh="m" contype="0" conaffinity="0"/></body></worldbody></mujoco>"#,
        Expect::Refused("in face 1, vertex index 7 does not exist"),
    ),
    // MuJoCo: "in face 3, vertex index 4 does not exist"
    (
        r#"<mujoco>  <compiler discardvisual="true"/>  <asset>    <mesh name="m" vertex="0 0 0  1 0 0  0 1 0  0 0 1" face="0 2 1  0 1 3  0 3 2  1 2 4"/>  </asset>  <worldbody>    <body pos="0 0 1"><freejoint/><geom type="box" size=".1 .1 .1"/></body>  </worldbody></mujoco>"#,
        Expect::Refused("in face 3, vertex index 4 does not exist"),
    ),
    // MuJoCo: "height fields are not supported in geom distance sensors"
    (
        r#"<mujoco>  <compiler discardvisual="true"/>  <asset>    <hfield name="h" size="1 1 0.1 0.1" nrow="3" ncol="3" elevation="0 0 0 0 1 0 0 0 0"/>  </asset>  <worldbody>    <geom name="hf" type="hfield" hfield="h" contype="0" conaffinity="0"/>    <body pos="0 0 1"><freejoint/><geom name="s" type="sphere" size=".1"/></body>  </worldbody>  <sensor><distance geom1="s" geom2="hf"/></sensor></mujoco>"#,
        Expect::Refused("height fields are not supported in geom distance sensors"),
    ),
];

/// The geom ids a document's references hold after the discard, as MuJoCo
/// 3.5.0's: each geom wrap's, each sensor object and reference that is a
/// geom, each pair's two, and each geom name's.
struct GeomIds {
    wraps: &'static [usize],
    sensor_objects: &'static [usize],
    sensor_references: &'static [usize],
    pairs: &'static [(usize, usize)],
    names: &'static [(&'static str, usize)],
}

/// Visual geoms before the geoms a reference keeps, so every kept geom's id
/// moves.
const RENUMBERED: &[(&str, GeomIds)] = &[
    // pairs after visual geoms
    (
        r#"<mujoco><compiler discardvisual="true"/><worldbody><geom name="v0" type="sphere" size="0.05" pos="3 3 3" contype="0" conaffinity="0"/><geom name="floor" type="plane" size="2 2 0.1"/><body name="b" pos="0 0 0.09"><freejoint/><geom name="v1" type="sphere" size="0.05" pos="0 0 0.5" contype="0" conaffinity="0"/><geom name="ball" type="sphere" size="0.1"/></body><body name="c" pos="1 0 0.09"><freejoint/><geom name="v2" type="sphere" size="0.04" pos="0 0 0.3" contype="0" conaffinity="0"/><geom name="ball2" type="sphere" size="0.1" contype="0" conaffinity="0"/></body></worldbody><contact><pair geom1="floor" geom2="ball" condim="1" margin="0.02"/><pair geom1="floor" geom2="ball2"/></contact><sensor><framepos objtype="body" objname="b"/><framepos objtype="body" objname="c"/></sensor></mujoco>"#,
        GeomIds {
            wraps: &[],
            sensor_objects: &[],
            sensor_references: &[],
            pairs: &[(0, 1), (0, 2)],
            names: &[("ball", 1), ("ball2", 2), ("floor", 0)],
        },
    ),
    // tendon wraps after visual geoms
    (
        r#"<mujoco><compiler discardvisual="true"/><worldbody><geom type="sphere" size="0.05" pos="3 3 3" contype="0" conaffinity="0"/><body name="b" pos="0 0 1"><joint name="h" type="hinge" axis="0 1 0"/><geom type="sphere" size="0.05" pos="0 0 0.3" contype="0" conaffinity="0"/><geom type="capsule" size="0.05 0.3"/><site name="s1" pos="0.2 0 0.3"/><site name="s2" pos="-0.2 0 -0.3"/><site name="s3" pos="0.3 0 0"/><geom type="sphere" size="0.05" pos="0 0 0.4" contype="0" conaffinity="0"/><geom name="w" type="cylinder" size="0.15 0.1" contype="0" conaffinity="0"/><geom name="w2" type="sphere" size="0.12" pos="0 0 -0.1" contype="0" conaffinity="0"/></body><body name="c" pos="1 0 1"><joint type="hinge" axis="0 1 0"/><geom type="sphere" size="0.05" contype="0" conaffinity="0"/><geom name="k" type="sphere" size="0.08"/><geom name="k2" type="box" size="0.1 0.1 0.1" pos="0 0 -0.3"/></body></worldbody><tendon><spatial name="t"><site site="s1"/><geom geom="w"/><site site="s2"/></spatial><spatial name="t2"><site site="s3"/><geom geom="w2" sidesite="s1"/><site site="s2"/></spatial></tendon><actuator><motor tendon="t" gear="1"/></actuator><sensor><tendonpos tendon="t"/><tendonpos tendon="t2"/></sensor></mujoco>"#,
        GeomIds {
            wraps: &[1, 2],
            sensor_objects: &[],
            sensor_references: &[],
            pairs: &[],
            names: &[("k", 3), ("k2", 4), ("w", 1), ("w2", 2)],
        },
    ),
    // sensors' objects and references after visual geoms
    (
        r#"<mujoco><compiler discardvisual="true"/><worldbody><geom type="sphere" size="0.05" pos="3 3 3" contype="0" conaffinity="0"/><geom name="floor" type="plane" size="2 2 0.1"/><body name="b" pos="0 0 0.5" euler="0.1 0.2 0.3"><freejoint/><geom type="sphere" size="0.05" pos="0 0 0.3" contype="0" conaffinity="0"/><geom name="ball" type="sphere" size="0.1"/><geom name="v" type="box" size="0.05 0.05 0.05" pos="0.2 0.1 0" euler="0.3 0 0" contype="0" conaffinity="0"/></body><body name="c" pos="1 0 0.5"><freejoint/><geom type="sphere" size="0.04" contype="0" conaffinity="0"/><geom name="ball2" type="sphere" size="0.1" pos="0 0.1 0"/></body></worldbody><sensor><framepos objtype="geom" objname="v"/><framepos objtype="geom" objname="ball" reftype="geom" refname="ball2"/><framequat objtype="geom" objname="v" reftype="geom" refname="ball2"/><distance geom1="ball" geom2="ball2"/><distance geom1="v" body2="c"/><normal geom1="v" geom2="ball2"/><framexaxis objtype="geom" objname="ball2"/></sensor></mujoco>"#,
        GeomIds {
            wraps: &[],
            sensor_objects: &[2, 1, 2, 1, 2, 2, 3],
            sensor_references: &[3, 3, 3, 3],
            pairs: &[],
            names: &[("ball", 1), ("ball2", 3), ("floor", 0), ("v", 2)],
        },
    ),
];

/// 0 on macOS arm64, where MuJoCo's values were printed, and 1e-12 of an
/// array's largest entry elsewhere: a turned geom's `euler` passes through
/// `sin` and `cos`, whose last bit is the platform's libm's.
const TOLERANCE: f64 = if cfg!(all(target_os = "macos", target_arch = "aarch64")) {
    0.0
} else {
    1e-12
};

/// Whether `ours` is `mujoco` within [`TOLERANCE`] of its largest entry, bit
/// for bit where that is 0; a value that is not a number fails.
fn close(ours: &[f64], mujoco: &[f64]) -> bool {
    if ours.len() != mujoco.len() {
        return false;
    }
    if TOLERANCE == 0.0 {
        return ours
            .iter()
            .zip(mujoco)
            .all(|(a, b)| a.to_bits() == b.to_bits());
    }
    let scale = mujoco.iter().fold(0.0_f64, |m, v| m.max(v.abs()));
    ours.iter()
        .zip(mujoco)
        .all(|(a, b)| (a - b).abs() <= TOLERANCE * scale)
}

/// MuJoCo's `mjtGeom` number of `kind`.
const fn mujoco_type(kind: GeomType) -> i32 {
    match kind {
        GeomType::Plane => 0,
        GeomType::Hfield => 1,
        GeomType::Sphere => 2,
        GeomType::Capsule => 3,
        GeomType::Ellipsoid => 4,
        GeomType::Cylinder => 5,
        GeomType::Box => 6,
        GeomType::Mesh => 7,
        GeomType::Sdf => 8,
    }
}

#[test]
fn each_reference_is_renumbered_as_mujocos() {
    for (doc, want) in RENUMBERED {
        let m = load_model(doc).unwrap();
        let geom_ids = |kinds: &[MjObjectType], ids: &[usize]| -> Vec<usize> {
            kinds
                .iter()
                .zip(ids)
                .filter(|(k, _)| **k == MjObjectType::Geom)
                .map(|(_, &id)| id)
                .collect()
        };
        let wraps: Vec<usize> = m
            .wrap_type
            .iter()
            .zip(&m.wrap_objid)
            .filter(|(t, _)| **t == WrapType::Geom)
            .map(|(_, &id)| id)
            .collect();
        let pairs: Vec<(usize, usize)> =
            m.contact_pairs.iter().map(|p| (p.geom1, p.geom2)).collect();
        let mut names: Vec<(&str, usize)> = m
            .geom_name_to_id
            .iter()
            .map(|(n, &id)| (n.as_str(), id))
            .collect();
        names.sort_unstable();
        assert_eq!(wraps, want.wraps, "wraps: {doc}");
        assert_eq!(
            geom_ids(&m.sensor_objtype, &m.sensor_objid),
            want.sensor_objects,
            "sensor objects: {doc}"
        );
        assert_eq!(
            geom_ids(&m.sensor_reftype, &m.sensor_refid),
            want.sensor_references,
            "sensor references: {doc}"
        );
        assert_eq!(pairs, want.pairs, "pairs: {doc}");
        let set: std::collections::HashSet<(usize, usize)> =
            pairs.iter().map(|&(a, b)| (a.min(b), a.max(b))).collect();
        assert_eq!(m.contact_pair_set, set, "the pair set: {doc}");
        assert_eq!(names, want.names, "names: {doc}");
    }
}

#[test]
fn each_document_discards_as_mujoco() {
    let mut wrong = Vec::new();
    for (doc, expect) in CASES {
        match (load_model(doc), expect) {
            (Ok(m), Expect::Loads(want)) => {
                let flat3 = |v: &[nalgebra::Vector3<f64>]| -> Vec<f64> {
                    v.iter().flat_map(|x| x.iter().copied()).collect()
                };
                let iquat: Vec<f64> = m
                    .body_iquat
                    .iter()
                    .flat_map(|q| [q.w, q.i, q.j, q.k])
                    .collect();
                let mesh: Vec<i64> = m
                    .geom_mesh
                    .iter()
                    .map(|g| g.map_or(-1, |id| i64::try_from(id).unwrap()))
                    .collect();
                let kinds: Vec<i32> = m.geom_type.iter().map(|&t| mujoco_type(t)).collect();
                let ok = m.ngeom == want.ngeom
                    && m.nmesh == want.nmesh
                    && kinds == want.geom_type
                    && m.geom_body == want.geom_body
                    && close(&flat3(&m.geom_size), want.geom_size.as_flattened())
                    && close(&flat3(&m.geom_pos), want.geom_pos.as_flattened())
                    && mesh == want.geom_mesh
                    && close(&m.body_mass, want.body_mass)
                    && close(&flat3(&m.body_inertia), want.body_inertia.as_flattened())
                    && close(&flat3(&m.body_ipos), want.body_ipos.as_flattened())
                    && close(&iquat, want.body_iquat.as_flattened());
                if !ok {
                    wrong.push(format!(
                        "ngeom {} nmesh {} bodies {:?} meshes {mesh:?} mass {:?} inertia {:?} ipos {:?} iquat {iquat:?}: {doc}",
                        m.ngeom, m.nmesh, m.geom_body, m.body_mass, m.body_inertia, m.body_ipos
                    ));
                }
            }
            (Err(e), Expect::Refused(words)) if e.to_string().contains(words) => {}
            (Ok(_), Expect::Refused(words)) => {
                wrong.push(format!("loads, MuJoCo refuses ({words}): {doc}"));
            }
            (Err(e), _) => wrong.push(format!("refused ({e}): {doc}")),
        }
    }
    assert!(
        wrong.is_empty(),
        "{} of {}:\n{}",
        wrong.len(),
        CASES.len(),
        wrong.join("\n")
    );
}

/// A visual geom a `<custom><tuple>` names is kept, as MuJoCo keeps it
/// (`SetNotVisual`, `user_objects.cc:7918`; measured: ngeom 2), the mass of
/// both geoms in its body.
#[test]
fn a_tuple_naming_a_visual_geom_keeps_it() {
    let m = load_model(
        r#"<mujoco><compiler discardvisual="true"/><worldbody><body name="b" pos="0 0 1"><freejoint/><geom type="sphere" size="0.1"/><geom name="v" type="sphere" size="0.1" pos="0.3 0 0" contype="0" conaffinity="0"/></body></worldbody><custom><tuple name="t"><element objtype="geom" objname="v"/></tuple></custom></mujoco>"#,
    )
    .unwrap();
    assert_eq!(m.ngeom, 2, "the visual geom kept");
    assert_eq!(
        m.body_mass[1].to_bits(),
        8.377580409572783_f64.to_bits(),
        "both geoms' mass, MuJoCo's"
    );
}

/// Under `fusestatic` a tuple's geom element is bound by its index from
/// before the fusion, which moving a fused body's geoms renumbers: MuJoCo
/// 3.5.0 binds the tuple naming the visual geom `v` to `g1`, so deletes `v`
/// (measured: ngeom 2, `tuple_objid` 1); this crate keeps the geom the tuple
/// names (registry `D-FUSESTATIC-PAIR-GEOM`).
#[test]
fn a_fused_models_tuple_holds_the_geom_it_names() {
    let m = load_model(
        r#"<mujoco><compiler discardvisual="true" fusestatic="true"/><worldbody><geom name="floor" type="plane" size="2 2 0.1"/><body pos="0 0 0.3"><freejoint/><geom name="g1" size="0.1"/><geom name="v" size="0.05" pos="0.2 0 0" contype="0" conaffinity="0"/></body><body pos="1 0 0.3"><geom name="gs" size="0.1" contype="0" conaffinity="0"/></body></worldbody><custom><tuple name="t"><element objtype="geom" objname="v"/></tuple></custom></mujoco>"#,
    )
    .unwrap();
    let mut names: Vec<&str> = m.geom_name_to_id.keys().map(String::as_str).collect();
    names.sort_unstable();
    assert_eq!(names, ["floor", "g1", "v"], "the tuple's geom kept");
}

/// A mesh with a `<plugin>`, which only its plugin makes (registry
/// `D-MESH-PLUGIN`): one with no file that no geom names, which
/// `discardvisual` deletes, is not built, and the document loads as MuJoCo
/// 3.5.0 loads it (ngeom 1, nmesh 0, the sphere's mass, measured); one a geom
/// names, a visual one among them, is refused, as one is without
/// `discardvisual`.
#[test]
fn a_plugin_mesh_no_geom_names_is_not_built_and_one_a_geom_names_is_refused() {
    for doc in [
        // a plugin mesh with no vertices, which no geom names
        r#"<mujoco><compiler discardvisual="true"/><extension><plugin plugin="mujoco.sdf.torus"><instance name="torus"><config key="radius1" value="0.35"/><config key="radius2" value="0.15"/></instance></plugin></extension><asset><mesh name="torus"><plugin instance="torus"/></mesh></asset><worldbody><body name="b" pos="0 0 1"><freejoint/><geom type="sphere" size="0.1"/></body></worldbody></mujoco>"#,
        // the same with vertices of its own
        r#"<mujoco><compiler discardvisual="true"/><extension><plugin plugin="mujoco.sdf.torus"><instance name="torus"><config key="radius1" value="0.35"/><config key="radius2" value="0.15"/></instance></plugin></extension><asset><mesh name="cube" vertex="-0.1 -0.1 -0.1 0.1 -0.1 -0.1 0.1 0.1 -0.1 -0.1 0.1 -0.1 -0.1 -0.1 0.1 0.1 -0.1 0.1 0.1 0.1 0.1 -0.1 0.1 0.1"><plugin instance="torus"/></mesh></asset><worldbody><body name="b" pos="0 0 1"><freejoint/><geom type="sphere" size="0.1"/></body></worldbody></mujoco>"#,
    ] {
        let m = load_model(doc).unwrap();
        assert_eq!((m.ngeom, m.nmesh), (1, 0), "{doc}");
        assert_eq!(
            m.body_mass[1].to_bits(),
            4.188790204786391_f64.to_bits(),
            "{doc}"
        );
    }
    for doc in [
        // a visual SDF geom over a plugin mesh with no vertices
        r#"<mujoco><compiler discardvisual="true"/><extension><plugin plugin="mujoco.sdf.torus"><instance name="torus"><config key="radius1" value="0.35"/><config key="radius2" value="0.15"/></instance></plugin></extension><asset><mesh name="torus"><plugin instance="torus"/></mesh></asset><worldbody><body name="b" pos="0 0 1"><freejoint/><geom type="sphere" size="0.1"/><geom type="sdf" mesh="torus" contype="0" conaffinity="0"><plugin instance="torus"/></geom></body></worldbody></mujoco>"#,
        // a visual mesh geom over it
        r#"<mujoco><compiler discardvisual="true"/><extension><plugin plugin="mujoco.sdf.torus"><instance name="torus"><config key="radius1" value="0.35"/><config key="radius2" value="0.15"/></instance></plugin></extension><asset><mesh name="torus"><plugin instance="torus"/></mesh></asset><worldbody><body name="b" pos="0 0 1"><freejoint/><geom type="sphere" size="0.1"/><geom type="mesh" mesh="torus" contype="0" conaffinity="0"/></body></worldbody></mujoco>"#,
        // a visual mesh geom over a plugin mesh with vertices of its own
        r#"<mujoco><compiler discardvisual="true"/><extension><plugin plugin="mujoco.sdf.torus"><instance name="torus"><config key="radius1" value="0.35"/><config key="radius2" value="0.15"/></instance></plugin></extension><asset><mesh name="cube" vertex="-0.1 -0.1 -0.1 0.1 -0.1 -0.1 0.1 0.1 -0.1 -0.1 0.1 -0.1 -0.1 -0.1 0.1 0.1 -0.1 0.1 0.1 0.1 0.1 -0.1 0.1 0.1"><plugin instance="torus"/></mesh></asset><worldbody><body name="b" pos="0 0 1"><freejoint/><geom type="sphere" size="0.1"/><geom type="mesh" mesh="cube" contype="0" conaffinity="0"/></body></worldbody></mujoco>"#,
        // a visual SDF geom with the plugin over it
        r#"<mujoco><compiler discardvisual="true"/><extension><plugin plugin="mujoco.sdf.torus"><instance name="torus"><config key="radius1" value="0.35"/><config key="radius2" value="0.15"/></instance></plugin></extension><asset><mesh name="cube" vertex="-0.1 -0.1 -0.1 0.1 -0.1 -0.1 0.1 0.1 -0.1 -0.1 0.1 -0.1 -0.1 -0.1 0.1 0.1 -0.1 0.1 0.1 0.1 0.1 -0.1 0.1 0.1"><plugin instance="torus"/></mesh></asset><worldbody><body name="b" pos="0 0 1"><freejoint/><geom type="sphere" size="0.1"/><geom type="sdf" mesh="cube" contype="0" conaffinity="0"><plugin instance="torus"/></geom></body></worldbody></mujoco>"#,
        // a mesh geom over a plugin mesh with no vertices, no discardvisual
        r#"<mujoco><extension><plugin plugin="mujoco.sdf.torus"><instance name="torus"><config key="radius1" value="0.35"/><config key="radius2" value="0.15"/></instance></plugin></extension><asset><mesh name="torus"><plugin instance="torus"/></mesh></asset><worldbody><body name="b" pos="0 0 1"><freejoint/><geom type="mesh" mesh="torus"/></body></worldbody></mujoco>"#,
    ] {
        let error = load_model(doc).unwrap_err().to_string();
        assert!(error.contains("D-MESH-PLUGIN"), "{error}: {doc}");
    }
}

/// The geoms' user data length MuJoCo sets before it discards
/// (`SetNuser`, `user_model.cc:4926`) stays when every geom is discarded
/// (measured: `nuser_geom` 3).
#[test]
fn a_discarded_geoms_user_data_length_stays() {
    let m = load_model(r#"<mujoco><compiler discardvisual="true"/><worldbody><body name="b" pos="0 0 1"><freejoint/><geom type="sphere" size="0.1" contype="0" conaffinity="0" user="1 2 3"/></body></worldbody></mujoco>"#).unwrap();
    assert_eq!((m.ngeom, m.nuser_geom), (0, 3));
}

/// Under `fusestatic` a sensor's and a tendon's geom hold the geoms they
/// name, and `discardvisual` keeps those; MuJoCo binds each to the geom at its
/// index before the fusion and keeps that one (measured: the frame sensor on
/// `g1` reads the fused `gs0` and four geoms stay; the tendon's wrap is
/// refused, "is not sphere or cylinder"; registry `D-FUSESTATIC-PAIR-GEOM`).
#[test]
fn a_fused_models_references_hold_the_geoms_they_name() {
    let m = load_model(r#"<mujoco><compiler discardvisual="true" fusestatic="true"/><worldbody><geom name="floor" type="plane" size="2 2 0.1"/><body name="b" pos="0 0 0.3"><freejoint/><geom name="g1" type="sphere" size="0.1" contype="0" conaffinity="0"/><geom name="g2" type="box" size="0.05 0.05 0.05" pos="0 0 0.2"/></body><body name="s" pos="1 0 0"><geom name="gs0" type="box" size="0.1 0.1 0.1" contype="0" conaffinity="0"/></body></worldbody><sensor><framepos objtype="geom" objname="g1"/></sensor></mujoco>"#).unwrap();
    let g1 = m.geom_name_to_id["g1"];
    assert_eq!(
        (m.ngeom, m.sensor_objid[0]),
        (3, g1),
        "the frame sensor's geom"
    );
    let m = load_model(r#"<mujoco><compiler discardvisual="true" fusestatic="true"/><worldbody><geom name="floor" type="plane" size="2 2 0.1"/><body name="b" pos="0 0 0.3"><freejoint/><site name="x1" pos="-0.2 0 0"/><site name="x2" pos="0.2 0 0.01"/><geom name="g1" type="sphere" size="0.1" contype="0" conaffinity="0"/><geom name="g2" type="box" size="0.05 0.05 0.05" pos="0 0 0.2"/></body><body name="s" pos="1 0 0"><geom name="gs0" type="box" size="0.1 0.1 0.1" contype="0" conaffinity="0"/></body></worldbody><tendon><spatial><site site="x1"/><geom geom="g1"/><site site="x2"/></spatial></tendon></mujoco>"#).unwrap();
    let g1 = m.geom_name_to_id["g1"];
    let wraps: Vec<usize> = m
        .wrap_type
        .iter()
        .zip(&m.wrap_objid)
        .filter(|(t, _)| **t == WrapType::Geom)
        .map(|(_, &id)| id)
        .collect();
    assert_eq!((m.ngeom, wraps), (3, vec![g1]), "the tendon's geom wrap");
}

/// A frame sensor's `refname` with no `reftype`, which MuJoCo 3.5.0's reader
/// refuses (`xml_native_reader.cc:4213-4214`; measured), is refused here too,
/// a visual geom it names among them; and the compile checks MuJoCo makes of
/// a plugin mesh no geom names before `discardvisual` deletes it are made: a
/// file it cannot open, a scale other than 1 (`user_mesh.cc:367-370`;
/// measured, each refused by MuJoCo).
#[test]
fn a_discarded_elements_reader_and_compile_checks_are_made() {
    for (doc, words) in [
        (
            r#"<mujoco><compiler discardvisual="true"/><worldbody><geom name="floor" type="plane" size="2 2 0.1"/><body name="b" pos="0 0 1"><freejoint/><geom name="v" type="sphere" size="0.05" pos="0.3 0 0" contype="0" conaffinity="0"/><geom name="ball" type="sphere" size="0.1"/></body></worldbody><sensor><framepos objtype="body" objname="b" refname="v"/></sensor></mujoco>"#,
            "refname 'v' given but reftype is missing",
        ),
        (
            r#"<mujoco><compiler discardvisual="true"/><extension><plugin plugin="mujoco.sdf.torus"><instance name="torus"><config key="radius1" value="0.35"/><config key="radius2" value="0.15"/></instance></plugin></extension><asset><mesh name="t" scale="2 2 2"><plugin instance="torus"/></mesh></asset><worldbody><body name="b" pos="0 0 1"><freejoint/><geom type="sphere" size="0.1"/></body></worldbody></mujoco>"#,
            "attribute scale is not compatible with SDFs in mesh 't'",
        ),
        (
            r#"<mujoco><compiler discardvisual="true"/><extension><plugin plugin="mujoco.sdf.torus"><instance name="torus"><config key="radius1" value="0.35"/><config key="radius2" value="0.15"/></instance></plugin></extension><asset><mesh name="cf" file="missing.obj"><plugin instance="torus"/></mesh></asset><worldbody><body name="b" pos="0 0 1"><freejoint/><geom type="sphere" size="0.1"/></body></worldbody></mujoco>"#,
            "missing.obj",
        ),
    ] {
        let error = load_model(doc).unwrap_err().to_string();
        assert!(error.contains(words), "{error}: {doc}");
    }
}

/// A distance equality's geoms (this crate's; MuJoCo 3.5.0 refuses the
/// element) keep the geoms they name when visual geoms before them are
/// deleted.
#[test]
fn a_distance_equalitys_geoms_are_renumbered() {
    let m = load_model(r#"<mujoco><compiler discardvisual="true"/><worldbody><geom type="sphere" size="0.05" pos="3 3 3" contype="0" conaffinity="0"/><body name="a" pos="0 0 1"><freejoint/><geom type="sphere" size="0.04" contype="0" conaffinity="0"/><geom name="g1" type="sphere" size="0.1"/></body><body name="b" pos="0.5 0 1.2"><freejoint/><geom type="sphere" size="0.04" contype="0" conaffinity="0"/><geom name="g2" type="box" size="0.1 0.1 0.1"/></body></worldbody><equality><distance geom1="g1" geom2="g2" distance="0.3"/></equality><sensor><framepos objtype="body" objname="a"/><framepos objtype="body" objname="b"/></sensor></mujoco>"#).unwrap();
    let ids = (m.geom_name_to_id["g1"], m.geom_name_to_id["g2"]);
    assert_eq!((m.ngeom, m.eq_obj1id[0], m.eq_obj2id[0]), (2, ids.0, ids.1));
}
