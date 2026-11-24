//! `discardvisual` as MuJoCo 3.5.0 makes it (book Rigid-loading L39; ledger
//! L336, L342): every geom and mesh built and checked, each body's mass,
//! inertia and inertial frame from all its geoms, the visual ones among them,
//! and only then the visual geoms deleted, a geom a pair, a tendon or a sensor
//! names kept, and the meshes no kept geom names deleted.
//!
//! Every expected value was printed by MuJoCo 3.5.0 on the census's unfused
//! oracle (2026-11-23).

// The values are MuJoCo's, printed to the last digit.
#![allow(clippy::unwrap_used, clippy::unreadable_literal)]

use sim_mjcf::load_model;

/// The fields `discardvisual` decides, as MuJoCo 3.5.0 compiles them.
struct Model {
    ngeom: usize,
    nmesh: usize,
    geom_body: &'static [usize],
    /// Each geom's mesh, -1 for none.
    geom_mesh: &'static [i64],
    body_mass: &'static [f64],
    body_inertia: &'static [[f64; 3]],
    body_ipos: &'static [[f64; 3]],
    /// (w, x, y, z).
    body_iquat: &'static [[f64; 4]],
}

/// What MuJoCo 3.5.0 makes of a document.
enum Expect {
    Loads(Model),
    /// Refused, in words these hold.
    Refused(&'static str),
}

const CASES: &[(&str, Expect)] = &[
    // two of five spheres visual: their mass kept, the geoms deleted
    (
        r#"<mujoco><compiler discardvisual="true"/><worldbody><body name="b" pos="0 0 1"><freejoint/><geom type="sphere" size="0.1" pos="0.0 0 0" /><geom type="sphere" size="0.1" pos="0.3 0 0" contype="0" conaffinity="0"/><geom type="sphere" size="0.1" pos="0.6 0 0" /><geom type="sphere" size="0.1" pos="0.8999999999999999 0 0" contype="0" conaffinity="0"/><geom type="sphere" size="0.1" pos="1.2 0 0" /></body></worldbody></mujoco>"#,
        Expect::Loads(Model {
            ngeom: 3,
            nmesh: 0,
            geom_body: &[1, 1, 1],
            geom_mesh: &[-1, -1, -1],
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
            geom_body: &[],
            geom_mesh: &[],
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
            geom_body: &[1, 1],
            geom_mesh: &[-1, -1],
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
            geom_body: &[0, 1, 1],
            geom_mesh: &[-1, -1, -1],
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
            geom_body: &[0, 1, 1],
            geom_mesh: &[-1, -1, -1],
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
            geom_body: &[1],
            geom_mesh: &[-1],
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
            geom_body: &[1],
            geom_mesh: &[0],
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
            geom_body: &[1],
            geom_mesh: &[-1],
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
            geom_body: &[1, 1],
            geom_mesh: &[-1, -1],
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
            geom_body: &[1, 1, 2],
            geom_mesh: &[-1, 0, 0],
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
    // discardvisual off: every geom kept
    (
        r#"<mujoco><compiler discardvisual="false"/><worldbody><body name="b" pos="0 0 1"><freejoint/><geom type="sphere" size="0.1" pos="0.0 0 0" /><geom type="sphere" size="0.1" pos="0.3 0 0" contype="0" conaffinity="0"/><geom type="sphere" size="0.1" pos="0.6 0 0" /><geom type="sphere" size="0.1" pos="0.8999999999999999 0 0" contype="0" conaffinity="0"/><geom type="sphere" size="0.1" pos="1.2 0 0" /></body></worldbody></mujoco>"#,
        Expect::Loads(Model {
            ngeom: 5,
            nmesh: 0,
            geom_body: &[1, 1, 1, 1, 1],
            geom_mesh: &[-1, -1, -1, -1, -1],
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
            geom_body: &[1],
            geom_mesh: &[-1],
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
            geom_body: &[1],
            geom_mesh: &[-1],
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
            geom_body: &[1],
            geom_mesh: &[-1],
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
            geom_body: &[0, 1],
            geom_mesh: &[-1, -1],
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
            geom_body: &[1, 1],
            geom_mesh: &[-1, -1],
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
    // MuJoCo: "mass, inertia or density are negative in geom"
    (
        r#"<mujoco><compiler discardvisual="true"/><worldbody><body name="b" pos="0 0 1"><freejoint/><geom type="sphere" size="0.1"/><geom type="sphere" size="0.1" density="-5" contype="0" conaffinity="0"/></body></worldbody></mujoco>"#,
        Expect::Refused("mass, inertia or density are negative in geom"),
    ),
    // MuJoCo: "invalid condim in geom"
    (
        r#"<mujoco><compiler discardvisual="true"/><worldbody><body name="b" pos="0 0 1"><freejoint/><geom type="sphere" size="0.1"/><geom type="sphere" size="0.1" condim="2" contype="0" conaffinity="0"/></body></worldbody></mujoco>"#,
        Expect::Refused("invalid condim in geom"),
    ),
    // MuJoCo: "fromto requires capsule, cylinder, box or ellipsoid in geom"
    (
        r#"<mujoco><compiler discardvisual="true"/><worldbody><body name="b" pos="0 0 1"><freejoint/><geom type="sphere" size="0.1"/><geom type="sphere" size="0.1" fromto="0 0 0 0 0 1" contype="0" conaffinity="0"/></body></worldbody></mujoco>"#,
        Expect::Refused("fromto requires"),
    ),
    // MuJoCo: "in face 1, vertex index 7 does not exist"
    (
        r#"<mujoco><compiler discardvisual="true"/><asset><mesh name="m" vertex="0 0 0 1 0 0 0 1 0 0 0 1" face="0 1 2 0 1 7 0 2 3 1 2 3"/></asset><worldbody><body name="b" pos="0 0 1"><freejoint/><geom type="sphere" size="0.1"/><geom type="mesh" mesh="m" contype="0" conaffinity="0"/></body></worldbody></mujoco>"#,
        Expect::Refused("vertex index 7 does not exist"),
    ),
    // MuJoCo: "fromto points too close in geom"
    (
        r#"<mujoco><compiler discardvisual="true"/><worldbody><body name="b" pos="0 0 1"><freejoint/><geom type="sphere" size="0.1"/><geom type="capsule" size="0.1" fromto="0 0 0 0 0 0" contype="0" conaffinity="0"/></body></worldbody></mujoco>"#,
        Expect::Refused("fromto points too close"),
    ),
    // MuJoCo: "mass, inertia or density are negative in geom"
    (
        r#"<mujoco><compiler discardvisual="true"/><worldbody><body name="b" pos="0 0 1"><freejoint/><geom type="sphere" size="0.1"/><geom type="sphere" size="0.1" mass="-1" contype="0" conaffinity="0"/></body></worldbody></mujoco>"#,
        Expect::Refused("mass, inertia or density are negative in geom"),
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
                let ok = m.ngeom == want.ngeom
                    && m.nmesh == want.nmesh
                    && m.geom_body == want.geom_body
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

/// A visual geom a `<custom><tuple>` names, which MuJoCo keeps
/// (`user_objects.cc:7918`; measured: ngeom 2), is deleted here, where no
/// `<custom>` is read; its mass stays in its body, as every visual geom's
/// does (registry `D-DISCARD-TUPLE`).
#[test]
fn a_tuple_naming_a_visual_geom_keeps_its_mass_not_the_geom() {
    let m = load_model(
        r#"<mujoco><compiler discardvisual="true"/><worldbody><body name="b" pos="0 0 1"><freejoint/><geom type="sphere" size="0.1"/><geom name="v" type="sphere" size="0.1" pos="0.3 0 0" contype="0" conaffinity="0"/></body></worldbody><custom><tuple name="t"><element objtype="geom" objname="v"/></tuple></custom></mujoco>"#,
    )
    .unwrap();
    assert_eq!(m.ngeom, 1, "the visual geom deleted");
    assert_eq!(
        m.body_mass[1].to_bits(),
        8.377580409572783_f64.to_bits(),
        "both geoms' mass, MuJoCo's"
    );
}
