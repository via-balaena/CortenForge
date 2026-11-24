//! `fusestatic` as MuJoCo 3.5.0 makes it (book Rigid-loading L42; ledger
//! L250, L253, L255, L343, L361, L373, L388, L406, L422, L425, L438): once
//! every body is built and checked, each static body fused into its parent
//! unless resolving the other elements' references keeps it, its geoms,
//! sites and children moved in MuJoCo's arithmetic, its mass and inertia
//! added to its parent's, then every body, geom and site numbered again.
//!
//! Every expected value was printed by MuJoCo 3.5.0 on the census's unfused
//! oracle (2026-11-24), but the fixtures' fused dynamics, which MuJoCo does
//! not keep (registry `D-FUSESTATIC-INERTIA`).

// The values are MuJoCo's, printed to the last digit.
#![allow(
    clippy::unwrap_used,
    clippy::unreadable_literal,
    clippy::approx_constant
)]

use sim_mjcf::load_model;

/// The fields `fusestatic` decides, as MuJoCo 3.5.0 compiles them.
struct Model {
    /// Each body's name, "" for none.
    body_name: &'static [&'static str],
    body_parent: &'static [usize],
    body_pos: &'static [[f64; 3]],
    /// (w, x, y, z), as each quaternion below.
    body_quat: &'static [[f64; 4]],
    geom_body: &'static [usize],
    geom_pos: &'static [[f64; 3]],
    geom_quat: &'static [[f64; 4]],
    site_body: &'static [usize],
    site_pos: &'static [[f64; 3]],
    site_quat: &'static [[f64; 4]],
    body_rootid: &'static [usize],
    /// Each body with geoms: its id, its first geom and its count (MuJoCo's
    /// first is -1 for a body with none).
    body_geoms: &'static [(usize, usize, usize)],
    jnt_body: &'static [usize],
    dof_body: &'static [usize],
    body_mass: &'static [f64],
    body_inertia: &'static [[f64; 3]],
    body_ipos: &'static [[f64; 3]],
    body_iquat: &'static [[f64; 4]],
    qpos0: &'static [f64],
    qpos_spring: &'static [f64],
    eq_data: &'static [[f64; 11]],
}

/// What MuJoCo 3.5.0 makes of a document.
enum Expect {
    Loads(&'static Model),
    /// Refused for MuJoCo's fault, in this crate's words, a body or geom by
    /// MuJoCo's id.
    Refused(&'static str),
}

const CASES: &[(&str, Expect)] = &[
    // a mocap body with a site: kept (L250)
    (
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="m" mocap="true" pos="0 0 0.1"><site name="s"/></body><body name="f" pos="0 0 2"><freejoint/><geom size="0.1"/></body></worldbody></mujoco>"#,
        Expect::Loads(&Model {
            body_name: &["world", "m", "f"],
            body_parent: &[0, 0, 0],
            body_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.1], [0.0, 0.0, 2.0]],
            body_quat: &[
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
            ],
            geom_body: &[2],
            geom_pos: &[[0.0, 0.0, 0.0]],
            geom_quat: &[[1.0, 0.0, 0.0, 0.0]],
            site_body: &[1],
            site_pos: &[[0.0, 0.0, 0.0]],
            site_quat: &[[1.0, 0.0, 0.0, 0.0]],
            body_rootid: &[0, 1, 2],
            body_geoms: &[(2, 0, 1)],
            jnt_body: &[2],
            dof_body: &[2, 2, 2, 2, 2, 2],
            body_mass: &[0.0, 0.0, 4.188790204786391],
            body_inertia: &[
                [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0],
                [
                    0.01675516081914557,
                    0.01675516081914557,
                    0.01675516081914557,
                ],
            ],
            body_ipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.1], [0.0, 0.0, 0.0]],
            body_iquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
            ],
            qpos0: &[0.0, 0.0, 2.0, 1.0, 0.0, 0.0, 0.0],
            qpos_spring: &[0.0, 0.0, 2.0, 1.0, 0.0, 0.0, 0.0],
            eq_data: &[],
        }),
    ),
    // a mocap body alone: kept (L361)
    (
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="m" mocap="true" pos="0 0 1"><geom size="0.1" contype="0" conaffinity="0"/></body><body name="f" pos="0 0 2"><freejoint/><geom size="0.1"/></body></worldbody></mujoco>"#,
        Expect::Loads(&Model {
            body_name: &["world", "m", "f"],
            body_parent: &[0, 0, 0],
            body_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, 0.0, 2.0]],
            body_quat: &[
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
            ],
            geom_body: &[1, 2],
            geom_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            geom_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            site_body: &[],
            site_pos: &[],
            site_quat: &[],
            body_rootid: &[0, 1, 2],
            body_geoms: &[(1, 0, 1), (2, 1, 1)],
            jnt_body: &[2],
            dof_body: &[2, 2, 2, 2, 2, 2],
            body_mass: &[0.0, 4.188790204786391, 4.188790204786391],
            body_inertia: &[
                [0.0, 0.0, 0.0],
                [
                    0.01675516081914557,
                    0.01675516081914557,
                    0.01675516081914557,
                ],
                [
                    0.01675516081914557,
                    0.01675516081914557,
                    0.01675516081914557,
                ],
            ],
            body_ipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            body_iquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
            ],
            qpos0: &[0.0, 0.0, 2.0, 1.0, 0.0, 0.0, 0.0],
            qpos_spring: &[0.0, 0.0, 2.0, 1.0, 0.0, 0.0, 0.0],
            eq_data: &[],
        }),
    ),
    // a static body an exclude names: kept (L253)
    (
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="s" pos="0 0 0.5"><geom size="0.2"/></body><body name="f" pos="0 0 2"><freejoint/><geom size="0.1"/></body></worldbody><contact><exclude body1="s" body2="f"/></contact></mujoco>"#,
        Expect::Loads(&Model {
            body_name: &["world", "s", "f"],
            body_parent: &[0, 0, 0],
            body_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5], [0.0, 0.0, 2.0]],
            body_quat: &[
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
            ],
            geom_body: &[1, 2],
            geom_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            geom_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            site_body: &[],
            site_pos: &[],
            site_quat: &[],
            body_rootid: &[0, 1, 2],
            body_geoms: &[(1, 0, 1), (2, 1, 1)],
            jnt_body: &[2],
            dof_body: &[2, 2, 2, 2, 2, 2],
            body_mass: &[0.0, 33.51032163829113, 4.188790204786391],
            body_inertia: &[
                [0.0, 0.0, 0.0],
                [0.5361651462126582, 0.5361651462126582, 0.5361651462126582],
                [
                    0.01675516081914557,
                    0.01675516081914557,
                    0.01675516081914557,
                ],
            ],
            body_ipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            body_iquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
            ],
            qpos0: &[0.0, 0.0, 2.0, 1.0, 0.0, 0.0, 0.0],
            qpos_spring: &[0.0, 0.0, 2.0, 1.0, 0.0, 0.0, 0.0],
            eq_data: &[],
        }),
    ),
    // a cable with initial="none": its first body, an exclude's, kept (L343)
    (
        r#"<mujoco><compiler fusestatic="true"/><worldbody><composite type="cable" prefix="c" curve="s" count="5 1 1" size="1" offset="0 0 1" initial="none"><joint kind="main" damping="0.01"/><geom type="capsule" size="0.01"/></composite></worldbody></mujoco>"#,
        Expect::Loads(&Model {
            body_name: &["world", "cB_first", "cB_1", "cB_2", "cB_last"],
            body_parent: &[0, 0, 1, 2, 3],
            body_pos: &[
                [0.0, 0.0, 0.0],
                [0.0, 0.0, 1.0],
                [0.25, 0.0, 0.0],
                [0.25, 0.0, 0.0],
                [0.25, 0.0, 0.0],
            ],
            body_quat: &[
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
            ],
            geom_body: &[1, 2, 3, 4],
            geom_pos: &[
                [0.125, 0.0, 0.0],
                [0.125, 0.0, 0.0],
                [0.125, 0.0, 0.0],
                [0.125, 0.0, 0.0],
            ],
            geom_quat: &[
                [0.7071067811865476, 0.0, -0.7071067811865475, 0.0],
                [0.7071067811865476, 0.0, -0.7071067811865475, 0.0],
                [0.7071067811865476, 0.0, -0.7071067811865475, 0.0],
                [0.7071067811865476, 0.0, -0.7071067811865475, 0.0],
            ],
            site_body: &[1, 4],
            site_pos: &[[0.0, 0.0, 0.0], [0.25, 0.0, 0.0]],
            site_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            body_rootid: &[0, 1, 1, 1, 1],
            body_geoms: &[(1, 0, 1), (2, 1, 1), (3, 2, 1), (4, 3, 1)],
            jnt_body: &[2, 3, 4],
            dof_body: &[2, 2, 2, 3, 3, 3, 4, 4, 4],
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
            qpos0: &[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
            qpos_spring: &[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
            eq_data: &[],
        }),
    ),
    // a static body holding a force sensor's site: kept
    (
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="p" pos="0 0 1"><joint name="h" type="hinge"/><geom size="0.1"/><body name="s" pos="0.2 0 0"><geom size="0.05"/><site name="fs"/></body></body></worldbody><sensor><force site="fs"/></sensor></mujoco>"#,
        Expect::Loads(&Model {
            body_name: &["world", "p", "s"],
            body_parent: &[0, 0, 1],
            body_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.2, 0.0, 0.0]],
            body_quat: &[
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
            ],
            geom_body: &[1, 2],
            geom_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            geom_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            site_body: &[2],
            site_pos: &[[0.0, 0.0, 0.0]],
            site_quat: &[[1.0, 0.0, 0.0, 0.0]],
            body_rootid: &[0, 1, 1],
            body_geoms: &[(1, 0, 1), (2, 1, 1)],
            jnt_body: &[1],
            dof_body: &[1],
            body_mass: &[0.0, 4.188790204786391, 0.5235987755982989],
            body_inertia: &[
                [0.0, 0.0, 0.0],
                [
                    0.01675516081914557,
                    0.01675516081914557,
                    0.01675516081914557,
                ],
                [
                    0.000523598775598299,
                    0.000523598775598299,
                    0.000523598775598299,
                ],
            ],
            body_ipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            body_iquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
            ],
            qpos0: &[0.0],
            qpos_spring: &[0.0],
            eq_data: &[],
        }),
    ),
    // MuJoCo's stale body map: b kept, a sensor naming m, the last body (L406)
    (
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="a" pos="0 0 1"><geom size="0.1"/><body name="b" pos="0.2 0 0"><geom size="0.1"/><body name="m" pos="0.2 0 0"><geom size="0.1"/></body></body></body></worldbody><sensor><framepos objtype="body" objname="m"/></sensor></mujoco>"#,
        Expect::Loads(&Model {
            body_name: &["world", "b", "m"],
            body_parent: &[0, 0, 1],
            body_pos: &[[0.0, 0.0, 0.0], [0.2, 0.0, 1.0], [0.2, 0.0, 0.0]],
            body_quat: &[
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
            ],
            geom_body: &[0, 1, 2],
            geom_pos: &[[0.0, 0.0, 1.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            geom_quat: &[
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
            ],
            site_body: &[],
            site_pos: &[],
            site_quat: &[],
            body_rootid: &[0, 1, 1],
            body_geoms: &[(0, 0, 1), (1, 1, 1), (2, 2, 1)],
            jnt_body: &[],
            dof_body: &[],
            body_mass: &[0.0, 4.188790204786391, 4.188790204786391],
            body_inertia: &[
                [0.0, 0.0, 0.0],
                [
                    0.01675516081914557,
                    0.01675516081914557,
                    0.01675516081914557,
                ],
                [
                    0.01675516081914557,
                    0.01675516081914557,
                    0.01675516081914557,
                ],
            ],
            body_ipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            body_iquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
            ],
            qpos0: &[],
            qpos_spring: &[],
            eq_data: &[],
        }),
    ),
    // the same, b unnamed: fused
    (
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="a" pos="0 0 1"><geom size="0.1"/><body pos="0.2 0 0"><geom size="0.1"/><body name="m" pos="0.2 0 0"><geom size="0.1"/></body></body></body></worldbody><sensor><framepos objtype="body" objname="m"/></sensor></mujoco>"#,
        Expect::Loads(&Model {
            body_name: &["world", "m"],
            body_parent: &[0, 0],
            body_pos: &[[0.0, 0.0, 0.0], [0.4, 0.0, 1.0]],
            body_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            geom_body: &[0, 0, 1],
            geom_pos: &[[0.0, 0.0, 1.0], [0.2, 0.0, 1.0], [0.0, 0.0, 0.0]],
            geom_quat: &[
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
            ],
            site_body: &[],
            site_pos: &[],
            site_quat: &[],
            body_rootid: &[0, 1],
            body_geoms: &[(0, 0, 2), (1, 2, 1)],
            jnt_body: &[],
            dof_body: &[],
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
            qpos0: &[],
            qpos_spring: &[],
            eq_data: &[],
        }),
    ),
    // the same, no sensor: every static body fused
    (
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="a" pos="0 0 1"><geom size="0.1"/><body name="b" pos="0.2 0 0"><geom size="0.1"/><body name="m" pos="0.2 0 0"><geom size="0.1"/></body></body></body></worldbody></mujoco>"#,
        Expect::Loads(&Model {
            body_name: &["world"],
            body_parent: &[0],
            body_pos: &[[0.0, 0.0, 0.0]],
            body_quat: &[[1.0, 0.0, 0.0, 0.0]],
            geom_body: &[0, 0, 0],
            geom_pos: &[[0.0, 0.0, 1.0], [0.2, 0.0, 1.0], [0.4, 0.0, 1.0]],
            geom_quat: &[
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
            ],
            site_body: &[],
            site_pos: &[],
            site_quat: &[],
            body_rootid: &[0],
            body_geoms: &[(0, 0, 3)],
            jnt_body: &[],
            dof_body: &[],
            body_mass: &[0.0],
            body_inertia: &[[0.0, 0.0, 0.0]],
            body_ipos: &[[0.0, 0.0, 0.0]],
            body_iquat: &[[1.0, 0.0, 0.0, 0.0]],
            qpos0: &[],
            qpos_spring: &[],
            eq_data: &[],
        }),
    ),
    // two fused before b: a reference to n throws, b kept
    (
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="a" pos="0 0 1"><geom size="0.1"/></body><body name="a2" pos="1 0 1"><geom size="0.1"/></body><body name="b" pos="2 0 1"><geom size="0.1"/></body><body name="n" pos="3 0 1"><freejoint/><geom size="0.1"/></body><body name="m" pos="4 0 1"><freejoint/><geom size="0.1"/></body></worldbody><sensor><framepos objtype="body" objname="n"/></sensor></mujoco>"#,
        Expect::Loads(&Model {
            body_name: &["world", "b", "n", "m"],
            body_parent: &[0, 0, 0, 0],
            body_pos: &[
                [0.0, 0.0, 0.0],
                [2.0, 0.0, 1.0],
                [3.0, 0.0, 1.0],
                [4.0, 0.0, 1.0],
            ],
            body_quat: &[
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
            ],
            geom_body: &[0, 0, 1, 2, 3],
            geom_pos: &[
                [0.0, 0.0, 1.0],
                [1.0, 0.0, 1.0],
                [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0],
            ],
            geom_quat: &[
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
            ],
            site_body: &[],
            site_pos: &[],
            site_quat: &[],
            body_rootid: &[0, 1, 2, 3],
            body_geoms: &[(0, 0, 2), (1, 2, 1), (2, 3, 1), (3, 4, 1)],
            jnt_body: &[2, 3],
            dof_body: &[2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3],
            body_mass: &[0.0, 4.188790204786391, 4.188790204786391, 4.188790204786391],
            body_inertia: &[
                [0.0, 0.0, 0.0],
                [
                    0.01675516081914557,
                    0.01675516081914557,
                    0.01675516081914557,
                ],
                [
                    0.01675516081914557,
                    0.01675516081914557,
                    0.01675516081914557,
                ],
                [
                    0.01675516081914557,
                    0.01675516081914557,
                    0.01675516081914557,
                ],
            ],
            body_ipos: &[
                [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0],
            ],
            body_iquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
            ],
            qpos0: &[
                3.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 4.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0,
            ],
            qpos_spring: &[
                3.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 4.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0,
            ],
            eq_data: &[],
        }),
    ),
    // a static body a weld names: kept
    (
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="s" pos="0 0 1"><geom size="0.1"/></body><body name="f" pos="0 0 2"><freejoint/><geom size="0.1"/></body></worldbody><equality><weld body1="f" body2="s"/></equality></mujoco>"#,
        Expect::Loads(&Model {
            body_name: &["world", "s", "f"],
            body_parent: &[0, 0, 0],
            body_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, 0.0, 2.0]],
            body_quat: &[
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
            ],
            geom_body: &[1, 2],
            geom_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            geom_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            site_body: &[],
            site_pos: &[],
            site_quat: &[],
            body_rootid: &[0, 1, 2],
            body_geoms: &[(1, 0, 1), (2, 1, 1)],
            jnt_body: &[2],
            dof_body: &[2, 2, 2, 2, 2, 2],
            body_mass: &[0.0, 4.188790204786391, 4.188790204786391],
            body_inertia: &[
                [0.0, 0.0, 0.0],
                [
                    0.01675516081914557,
                    0.01675516081914557,
                    0.01675516081914557,
                ],
                [
                    0.01675516081914557,
                    0.01675516081914557,
                    0.01675516081914557,
                ],
            ],
            body_ipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            body_iquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
            ],
            qpos0: &[0.0, 0.0, 2.0, 1.0, 0.0, 0.0, 0.0],
            qpos_spring: &[0.0, 0.0, 2.0, 1.0, 0.0, 0.0, 0.0],
            eq_data: &[[0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 1.0, 0.0, 0.0, 0.0, 1.0]],
        }),
    ),
    // a static body an adhesion actuator names: kept
    (
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="s" pos="0 0 0.5"><geom size="0.2"/></body><body name="f" pos="0 0 2"><freejoint/><geom size="0.1"/></body></worldbody><actuator><adhesion body="s" ctrlrange="0 1"/></actuator></mujoco>"#,
        Expect::Loads(&Model {
            body_name: &["world", "s", "f"],
            body_parent: &[0, 0, 0],
            body_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5], [0.0, 0.0, 2.0]],
            body_quat: &[
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
            ],
            geom_body: &[1, 2],
            geom_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            geom_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            site_body: &[],
            site_pos: &[],
            site_quat: &[],
            body_rootid: &[0, 1, 2],
            body_geoms: &[(1, 0, 1), (2, 1, 1)],
            jnt_body: &[2],
            dof_body: &[2, 2, 2, 2, 2, 2],
            body_mass: &[0.0, 33.51032163829113, 4.188790204786391],
            body_inertia: &[
                [0.0, 0.0, 0.0],
                [0.5361651462126582, 0.5361651462126582, 0.5361651462126582],
                [
                    0.01675516081914557,
                    0.01675516081914557,
                    0.01675516081914557,
                ],
            ],
            body_ipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            body_iquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
            ],
            qpos0: &[0.0, 0.0, 2.0, 1.0, 0.0, 0.0, 0.0],
            qpos_spring: &[0.0, 0.0, 2.0, 1.0, 0.0, 0.0, 0.0],
            eq_data: &[],
        }),
    ),
    // a static body a subtree sensor names: kept
    (
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="p" pos="0 0 1"><freejoint/><geom size="0.1"/><body name="s" pos="0.2 0 0"><geom size="0.05"/></body></body></worldbody><sensor><subtreecom body="s"/></sensor></mujoco>"#,
        Expect::Loads(&Model {
            body_name: &["world", "p", "s"],
            body_parent: &[0, 0, 1],
            body_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.2, 0.0, 0.0]],
            body_quat: &[
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
            ],
            geom_body: &[1, 2],
            geom_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            geom_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            site_body: &[],
            site_pos: &[],
            site_quat: &[],
            body_rootid: &[0, 1, 1],
            body_geoms: &[(1, 0, 1), (2, 1, 1)],
            jnt_body: &[1],
            dof_body: &[1, 1, 1, 1, 1, 1],
            body_mass: &[0.0, 4.188790204786391, 0.5235987755982989],
            body_inertia: &[
                [0.0, 0.0, 0.0],
                [
                    0.01675516081914557,
                    0.01675516081914557,
                    0.01675516081914557,
                ],
                [
                    0.000523598775598299,
                    0.000523598775598299,
                    0.000523598775598299,
                ],
            ],
            body_ipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            body_iquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
            ],
            qpos0: &[0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0],
            qpos_spring: &[0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0],
            eq_data: &[],
        }),
    ),
    // an unturned static child: its mass added to its parent's
    (
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="p" pos="0 0 1"><joint name="h" type="hinge" axis="0 1 0"/><geom type="box" size="0.1 0.05 0.02"/><body name="s" pos="0.3 0.1 0"><geom size="0.05"/></body></body></worldbody></mujoco>"#,
        Expect::Loads(&Model {
            body_name: &["world", "p"],
            body_parent: &[0, 0],
            body_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 1.0]],
            body_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            geom_body: &[1, 1],
            geom_pos: &[[0.0, 0.0, 0.0], [0.3, 0.1, 0.0]],
            geom_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            site_body: &[],
            site_pos: &[],
            site_quat: &[],
            body_rootid: &[0, 1],
            body_geoms: &[(1, 0, 2)],
            jnt_body: &[1],
            dof_body: &[1],
            body_mass: &[0.0, 1.323598775598299],
            body_inertia: &[
                [0.0, 0.0, 0.0],
                [
                    0.03550391064963749,
                    0.03475473517702022,
                    0.0014861075815489019,
                ],
            ],
            body_ipos: &[
                [0.0, 0.0, 0.0],
                [0.11867616952764697, 0.03955872317588233, 0.0],
            ],
            body_iquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [
                    -0.10696605905415207,
                    0.6989694286665373,
                    0.10696605905415207,
                    0.6989694286665373,
                ],
            ],
            qpos0: &[0.0],
            qpos_spring: &[0.0],
            eq_data: &[],
        }),
    ),
    // an unturned static child with an offset box
    (
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="p" pos="0 0 1"><joint name="h" type="hinge" axis="0 1 0"/><geom type="box" size="0.1 0.05 0.02"/><body name="s" pos="0.3 0.1 0"><geom type="box" pos="0.02 0 0.01" size="0.05 0.03 0.02"/></body></body></worldbody></mujoco>"#,
        Expect::Loads(&Model {
            body_name: &["world", "p"],
            body_parent: &[0, 0],
            body_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 1.0]],
            body_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            geom_body: &[1, 1],
            geom_pos: &[[0.0, 0.0, 0.0], [0.32, 0.1, 0.01]],
            geom_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            site_body: &[],
            site_pos: &[],
            site_quat: &[],
            body_rootid: &[0, 1],
            body_geoms: &[(1, 0, 2)],
            jnt_body: &[1],
            dof_body: &[1],
            body_mass: &[0.0, 1.0400000000000003],
            body_inertia: &[
                [0.0, 0.0, 0.0],
                [
                    0.024372888772308712,
                    0.023601182112695962,
                    0.0010523906534568614,
                ],
            ],
            body_ipos: &[
                [0.0, 0.0, 0.0],
                [
                    0.07384615384615384,
                    0.023076923076923075,
                    0.0023076923076923075,
                ],
            ],
            body_iquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [
                    -0.10620299563325047,
                    0.6920220049184653,
                    0.08851485452983815,
                    0.7085136476841623,
                ],
            ],
            qpos0: &[0.0],
            qpos_spring: &[0.0],
            eq_data: &[],
        }),
    ),
    // two static bodies, one in the other: each added to the moving body
    (
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="p" pos="0 0 1"><joint name="h" type="hinge" axis="0 1 0"/><geom type="box" size="0.1 0.05 0.02"/><body name="a" pos="0.2 0 0"><geom size="0.04"/><body name="b" pos="0.1 0.1 0"><geom size="0.03"/></body></body></body></worldbody></mujoco>"#,
        Expect::Loads(&Model {
            body_name: &["world", "p"],
            body_parent: &[0, 0],
            body_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 1.0]],
            body_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            geom_body: &[1, 1, 1],
            geom_pos: &[
                [0.0, 0.0, 0.0],
                [0.2, 0.0, 0.0],
                [0.30000000000000004, 0.1, 0.0],
            ],
            geom_quat: &[
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
            ],
            site_body: &[],
            site_pos: &[],
            site_quat: &[],
            body_rootid: &[0, 1],
            body_geoms: &[(1, 0, 3)],
            jnt_body: &[1],
            dof_body: &[1],
            body_mass: &[0.0, 1.1811799086355619],
            body_inertia: &[
                [0.0, 0.0, 0.0],
                [
                    0.018981726714109637,
                    0.01781200755997678,
                    0.0015953403750447628,
                ],
            ],
            body_ipos: &[
                [0.0, 0.0, 0.0],
                [0.07411717270162839, 0.009574945755712756, 0.0],
            ],
            body_iquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [
                    -0.056601271848890966,
                    0.7048377799359865,
                    0.056601271848890966,
                    0.7048377799359865,
                ],
            ],
            qpos0: &[0.0],
            qpos_spring: &[0.0],
            eq_data: &[],
        }),
    ),
    // a static child with no mass: not added
    (
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="p" pos="0 0 1"><joint name="h" type="hinge" axis="0 1 0"/><geom type="box" size="0.1 0.05 0.02"/><body name="s" pos="0.3 0.1 0" euler="0 0 30"><site name="x"/></body></body></worldbody></mujoco>"#,
        Expect::Loads(&Model {
            body_name: &["world", "p"],
            body_parent: &[0, 0],
            body_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 1.0]],
            body_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            geom_body: &[1],
            geom_pos: &[[0.0, 0.0, 0.0]],
            geom_quat: &[[1.0, 0.0, 0.0, 0.0]],
            site_body: &[1],
            site_pos: &[[0.3, 0.1, 0.0]],
            site_quat: &[[0.9659258262890683, 0.0, 0.0, 0.25881904510252074]],
            body_rootid: &[0, 1],
            body_geoms: &[(1, 0, 1)],
            jnt_body: &[1],
            dof_body: &[1],
            body_mass: &[0.0, 0.8000000000000002],
            body_inertia: &[
                [0.0, 0.0, 0.0],
                [
                    0.0007733333333333337,
                    0.0027733333333333342,
                    0.0033333333333333344,
                ],
            ],
            body_ipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            body_iquat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            qpos0: &[0.0],
            qpos_spring: &[0.0],
            eq_data: &[],
        }),
    ),
    // a static child of the world: its mass not added
    (
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="s" pos="0.3 0.1 0.5" euler="0 0 30"><geom type="box" size="0.1 0.1 0.1"/></body><body name="f" pos="0 0 2"><freejoint/><geom size="0.1"/></body></worldbody></mujoco>"#,
        Expect::Loads(&Model {
            body_name: &["world", "f"],
            body_parent: &[0, 0],
            body_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 2.0]],
            body_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            geom_body: &[0, 1],
            geom_pos: &[[0.3, 0.1, 0.5], [0.0, 0.0, 0.0]],
            geom_quat: &[
                [0.9659258262890683, 0.0, 0.0, 0.25881904510252074],
                [1.0, 0.0, 0.0, 0.0],
            ],
            site_body: &[],
            site_pos: &[],
            site_quat: &[],
            body_rootid: &[0, 1],
            body_geoms: &[(0, 0, 1), (1, 1, 1)],
            jnt_body: &[1],
            dof_body: &[1, 1, 1, 1, 1, 1],
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
            qpos0: &[0.0, 0.0, 2.0, 1.0, 0.0, 0.0, 0.0],
            qpos_spring: &[0.0, 0.0, 2.0, 1.0, 0.0, 0.0, 0.0],
            eq_data: &[],
        }),
    ),
    // a site-only static child, raised to boundmass before it is added
    (
        r#"<mujoco><compiler fusestatic="true" boundmass="0.5" boundinertia="0.01"/><worldbody><body name="p" pos="0 0 1"><joint name="h" type="hinge" axis="0 1 0"/><geom type="box" size="0.1 0.05 0.02"/><body name="s" pos="0.3 0.1 0"><site name="x"/></body></body></worldbody></mujoco>"#,
        Expect::Loads(&Model {
            body_name: &["world", "p"],
            body_parent: &[0, 0],
            body_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 1.0]],
            body_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            geom_body: &[1],
            geom_pos: &[[0.0, 0.0, 0.0]],
            geom_quat: &[[1.0, 0.0, 0.0, 0.0]],
            site_body: &[1],
            site_pos: &[[0.3, 0.1, 0.0]],
            site_quat: &[[1.0, 0.0, 0.0, 0.0]],
            body_rootid: &[0, 1],
            body_geoms: &[(1, 0, 1)],
            jnt_body: &[1],
            dof_body: &[1],
            body_mass: &[0.0, 1.3000000000000003],
            body_inertia: &[
                [0.0, 0.0, 0.0],
                [
                    0.1430769230769231,
                    0.14307692307692307,
                    0.019999999999999997,
                ],
            ],
            body_ipos: &[
                [0.0, 0.0, 0.0],
                [0.23076923076923073, 0.07692307692307691, 0.0],
            ],
            body_iquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [
                    0.41345260731526534,
                    0.41345260731526534,
                    0.5736348503222328,
                    0.5736348503222328,
                ],
            ],
            qpos0: &[0.0],
            qpos_spring: &[0.0],
            eq_data: &[],
        }),
    ),
    // a turned static child whose inertial frame is the identity
    (
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="p" pos="0 0 1"><joint name="h" type="hinge" axis="0 1 0"/><geom type="box" size="0.1 0.05 0.02"/><body name="s" pos="0.3 0.1 0" euler="20 30 40"><geom size="0.05"/></body></body></worldbody></mujoco>"#,
        Expect::Loads(&Model {
            body_name: &["world", "p"],
            body_parent: &[0, 0],
            body_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 1.0]],
            body_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            geom_body: &[1, 1],
            geom_pos: &[[0.0, 0.0, 0.0], [0.3, 0.1, 0.0]],
            geom_quat: &[
                [1.0, 0.0, 0.0, 0.0],
                [
                    0.8785122060499201,
                    0.24479231586341083,
                    0.18214796572990116,
                    0.36758011983238364,
                ],
            ],
            site_body: &[],
            site_pos: &[],
            site_quat: &[],
            body_rootid: &[0, 1],
            body_geoms: &[(1, 0, 2)],
            jnt_body: &[1],
            dof_body: &[1],
            body_mass: &[0.0, 1.323598775598299],
            body_inertia: &[
                [0.0, 0.0, 0.0],
                [
                    0.03550391064963749,
                    0.03475473517702022,
                    0.0014861075815489019,
                ],
            ],
            body_ipos: &[
                [0.0, 0.0, 0.0],
                [0.11867616952764697, 0.03955872317588233, 0.0],
            ],
            body_iquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [
                    -0.10696605905415207,
                    0.6989694286665373,
                    0.10696605905415207,
                    0.6989694286665373,
                ],
            ],
            qpos0: &[0.0],
            qpos_spring: &[0.0],
            eq_data: &[],
        }),
    ),
    // two static children added in turn
    (
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="p" pos="0 0 1"><joint name="h" type="hinge" axis="0 1 0"/><geom type="box" size="0.1 0.05 0.02"/><body name="s1" pos="0.3 0 0"><geom size="0.05"/></body><body name="s2" pos="-0.3 0 0"><geom type="box" size="0.05 0.04 0.03"/></body></body></worldbody></mujoco>"#,
        Expect::Loads(&Model {
            body_name: &["world", "p"],
            body_parent: &[0, 0],
            body_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 1.0]],
            body_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            geom_body: &[1, 1, 1],
            geom_pos: &[[0.0, 0.0, 0.0], [0.3, 0.0, 0.0], [-0.3, 0.0, 0.0]],
            geom_quat: &[
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
            ],
            site_body: &[],
            site_pos: &[],
            site_quat: &[],
            body_rootid: &[0, 1],
            body_geoms: &[(1, 0, 3)],
            jnt_body: &[1],
            dof_body: &[1],
            body_mass: &[0.0, 1.803598775598299],
            body_inertia: &[
                [0.0, 0.0, 0.0],
                [
                    0.09474196889261435,
                    0.09406996889261435,
                    0.001696932108931643,
                ],
            ],
            body_ipos: &[[0.0, 0.0, 0.0], [0.007251963605459216, 0.0, 0.0]],
            body_iquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 0.7071067811865489, 0.0, 0.7071067811865489],
            ],
            qpos0: &[0.0],
            qpos_spring: &[0.0],
            eq_data: &[],
        }),
    ),
    // inertiafromgeom true: a static child's geoms over its <inertial>
    (
        r#"<mujoco><compiler fusestatic="true" inertiafromgeom="true"/><worldbody><body name="p" pos="0 0 1"><joint name="h" type="hinge" axis="0 1 0"/><geom type="box" size="0.1 0.05 0.02"/><body name="s" pos="0.3 0.1 0"><inertial pos="0 0 0" mass="3" diaginertia="1 1 1"/><geom size="0.05"/></body></body></worldbody></mujoco>"#,
        Expect::Loads(&Model {
            body_name: &["world", "p"],
            body_parent: &[0, 0],
            body_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 1.0]],
            body_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            geom_body: &[1, 1],
            geom_pos: &[[0.0, 0.0, 0.0], [0.3, 0.1, 0.0]],
            geom_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            site_body: &[],
            site_pos: &[],
            site_quat: &[],
            body_rootid: &[0, 1],
            body_geoms: &[(1, 0, 2)],
            jnt_body: &[1],
            dof_body: &[1],
            body_mass: &[0.0, 1.323598775598299],
            body_inertia: &[
                [0.0, 0.0, 0.0],
                [
                    0.03550391064963749,
                    0.03475473517702022,
                    0.0014861075815489019,
                ],
            ],
            body_ipos: &[
                [0.0, 0.0, 0.0],
                [0.11867616952764697, 0.03955872317588233, 0.0],
            ],
            body_iquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [
                    -0.10696605905415207,
                    0.6989694286665373,
                    0.10696605905415207,
                    0.6989694286665373,
                ],
            ],
            qpos0: &[0.0],
            qpos_spring: &[0.0],
            eq_data: &[],
        }),
    ),
    // a turned static body's geom, site and child, moved as MuJoCo moves them
    (
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="s" pos="0.3 -0.2 1" quat="0.8 0.3 0.1 -0.2"><geom type="box" pos="0.1 0.2 0.3" quat="0.6 0.2 -0.3 0.1" size="0.05 0.04 0.03"/><site name="x" pos="0.1 -0.1 0.05" euler="10 20 30"/><body name="c" pos="0.2 0.1 -0.1" euler="5 15 25"><joint type="hinge"/><geom size="0.05"/></body></body></worldbody></mujoco>"#,
        Expect::Loads(&Model {
            body_name: &["world", "c"],
            body_parent: &[0, 0],
            body_pos: &[
                [0.0, 0.0, 0.0],
                [0.5179487179487179, -0.13333333333333336, 0.9102564102564102],
            ],
            body_quat: &[
                [1.0, 0.0, 0.0, 0.0],
                [
                    0.887356309266148,
                    0.4434875540746263,
                    0.12553314024179385,
                    0.012609539543530252,
                ],
            ],
            geom_body: &[0, 1],
            geom_pos: &[[0.49999999999999994, -0.3, 1.3], [0.0, 0.0, 0.0]],
            geom_quat: &[
                [
                    0.7526023228839093,
                    0.46437164603475256,
                    -0.40032038451271773,
                    -0.24019223070763063,
                ],
                [1.0, 0.0, 0.0, 0.0],
            ],
            site_body: &[0],
            site_pos: &[[0.341025641025641, -0.3333333333333333, 0.9448717948717948]],
            site_quat: &[[
                0.855873503096924,
                0.49943275846745294,
                0.11795756628919363,
                0.06429213809884243,
            ]],
            body_rootid: &[0, 1],
            body_geoms: &[(0, 0, 1), (1, 1, 1)],
            jnt_body: &[1],
            dof_body: &[1],
            body_mass: &[0.0, 0.5235987755982989],
            body_inertia: &[
                [0.0, 0.0, 0.0],
                [
                    0.000523598775598299,
                    0.000523598775598299,
                    0.000523598775598299,
                ],
            ],
            body_ipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            body_iquat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            qpos0: &[0.0],
            qpos_spring: &[0.0],
            eq_data: &[],
        }),
    ),
    // a turned static body fused into the world
    (
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="s" pos="0.3 -0.2 0.2" euler="17 -23 41"><geom type="box" pos="0.1 0.2 0.05" size="0.05 0.04 0.03"/><site name="x" pos="0.1 -0.1 0.05"/></body><body name="f" pos="0 0 2"><freejoint/><geom size="0.1"/></body></worldbody></mujoco>"#,
        Expect::Loads(&Model {
            body_name: &["world", "f"],
            body_parent: &[0, 0],
            body_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 2.0]],
            body_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            geom_body: &[0, 1],
            geom_pos: &[
                [
                    0.2291537226623364,
                    -3.0060682622290447e-06,
                    0.2864985977092304,
                ],
                [0.0, 0.0, 0.0],
            ],
            geom_quat: &[
                [
                    0.9181062995996286,
                    0.06661635239589836,
                    -0.23541596583845503,
                    0.31180507894723203,
                ],
                [1.0, 0.0, 0.0, 0.0],
            ],
            site_body: &[0],
            site_pos: &[[0.41032537876345354, -0.2390069186699677, 0.2938443983228356]],
            site_quat: &[[
                0.9181062995996286,
                0.06661635239589836,
                -0.23541596583845503,
                0.31180507894723203,
            ]],
            body_rootid: &[0, 1],
            body_geoms: &[(0, 0, 1), (1, 1, 1)],
            jnt_body: &[1],
            dof_body: &[1, 1, 1, 1, 1, 1],
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
            qpos0: &[0.0, 0.0, 2.0, 1.0, 0.0, 0.0, 0.0],
            qpos_spring: &[0.0, 0.0, 2.0, 1.0, 0.0, 0.0, 0.0],
            eq_data: &[],
        }),
    ),
    // a free body below a turned static body: its qpos0 its fused pose
    (
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="s" pos="0.3 0.2 0.5" euler="10 20 30"><geom size="0.1"/><body name="f" pos="0.1 0.2 0.3" euler="5 6 7"><freejoint/><geom size="0.1"/></body></body></worldbody></mujoco>"#,
        Expect::Loads(&Model {
            body_name: &["world", "f"],
            body_parent: &[0, 0],
            body_pos: &[
                [0.0, 0.0, 0.0],
                [0.3900165490540471, 0.37006562982737234, 0.8208967161686443],
            ],
            body_quat: &[
                [1.0, 0.0, 0.0, 0.0],
                [
                    0.9095342823939611,
                    0.16702497862838267,
                    0.1954588370747527,
                    0.3265668211464165,
                ],
            ],
            geom_body: &[0, 1],
            geom_pos: &[[0.3, 0.2, 0.5], [0.0, 0.0, 0.0]],
            geom_quat: &[
                [
                    0.943714364147489,
                    0.12767944069578063,
                    0.14487812541736914,
                    0.2685358227515692,
                ],
                [1.0, 0.0, 0.0, 0.0],
            ],
            site_body: &[],
            site_pos: &[],
            site_quat: &[],
            body_rootid: &[0, 1],
            body_geoms: &[(0, 0, 1), (1, 1, 1)],
            jnt_body: &[1],
            dof_body: &[1, 1, 1, 1, 1, 1],
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
            qpos0: &[
                0.3900165490540471,
                0.37006562982737234,
                0.8208967161686443,
                0.9095342823939611,
                0.16702497862838267,
                0.1954588370747527,
                0.3265668211464165,
            ],
            qpos_spring: &[
                0.3900165490540471,
                0.37006562982737234,
                0.8208967161686443,
                0.9095342823939611,
                0.16702497862838267,
                0.1954588370747527,
                0.3265668211464165,
            ],
            eq_data: &[],
        }),
    ),
    // a static body a light targets: kept (L361)
    (
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="t" pos="0 0 1"><geom size="0.1" contype="0" conaffinity="0"/></body><light name="l" mode="targetbody" target="t" pos="1 1 3"/><body name="f" pos="0 0 2"><freejoint/><geom size="0.1"/></body></worldbody></mujoco>"#,
        Expect::Loads(&Model {
            body_name: &["world", "t", "f"],
            body_parent: &[0, 0, 0],
            body_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, 0.0, 2.0]],
            body_quat: &[
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
            ],
            geom_body: &[1, 2],
            geom_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            geom_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            site_body: &[],
            site_pos: &[],
            site_quat: &[],
            body_rootid: &[0, 1, 2],
            body_geoms: &[(1, 0, 1), (2, 1, 1)],
            jnt_body: &[2],
            dof_body: &[2, 2, 2, 2, 2, 2],
            body_mass: &[0.0, 4.188790204786391, 4.188790204786391],
            body_inertia: &[
                [0.0, 0.0, 0.0],
                [
                    0.01675516081914557,
                    0.01675516081914557,
                    0.01675516081914557,
                ],
                [
                    0.01675516081914557,
                    0.01675516081914557,
                    0.01675516081914557,
                ],
            ],
            body_ipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            body_iquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
            ],
            qpos0: &[0.0, 0.0, 2.0, 1.0, 0.0, 0.0, 0.0],
            qpos_spring: &[0.0, 0.0, 2.0, 1.0, 0.0, 0.0, 0.0],
            eq_data: &[],
        }),
    ),
    // a static body a <custom> tuple names: kept
    (
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="s" pos="0 0 0.5"><geom size="0.2"/></body><body name="f" pos="0 0 2"><freejoint/><geom size="0.1"/></body></worldbody><custom><tuple name="t"><element objtype="body" objname="s"/></tuple></custom></mujoco>"#,
        Expect::Loads(&Model {
            body_name: &["world", "s", "f"],
            body_parent: &[0, 0, 0],
            body_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5], [0.0, 0.0, 2.0]],
            body_quat: &[
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
            ],
            geom_body: &[1, 2],
            geom_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            geom_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            site_body: &[],
            site_pos: &[],
            site_quat: &[],
            body_rootid: &[0, 1, 2],
            body_geoms: &[(1, 0, 1), (2, 1, 1)],
            jnt_body: &[2],
            dof_body: &[2, 2, 2, 2, 2, 2],
            body_mass: &[0.0, 33.51032163829113, 4.188790204786391],
            body_inertia: &[
                [0.0, 0.0, 0.0],
                [0.5361651462126582, 0.5361651462126582, 0.5361651462126582],
                [
                    0.01675516081914557,
                    0.01675516081914557,
                    0.01675516081914557,
                ],
            ],
            body_ipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            body_iquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
            ],
            qpos0: &[0.0, 0.0, 2.0, 1.0, 0.0, 0.0, 0.0],
            qpos_spring: &[0.0, 0.0, 2.0, 1.0, 0.0, 0.0, 0.0],
            eq_data: &[],
        }),
    ),
    // a static body a camera in a frame targets: kept
    (
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="t" pos="0 0 1"><geom size="0.1" contype="0" conaffinity="0"/></body><frame pos="1 0 0"><camera name="cam" mode="targetbody" target="t" pos="1 1 1"/></frame><body name="f" pos="0 0 2"><freejoint/><geom size="0.1"/></body></worldbody></mujoco>"#,
        Expect::Loads(&Model {
            body_name: &["world", "t", "f"],
            body_parent: &[0, 0, 0],
            body_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, 0.0, 2.0]],
            body_quat: &[
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
            ],
            geom_body: &[1, 2],
            geom_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            geom_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            site_body: &[],
            site_pos: &[],
            site_quat: &[],
            body_rootid: &[0, 1, 2],
            body_geoms: &[(1, 0, 1), (2, 1, 1)],
            jnt_body: &[2],
            dof_body: &[2, 2, 2, 2, 2, 2],
            body_mass: &[0.0, 4.188790204786391, 4.188790204786391],
            body_inertia: &[
                [0.0, 0.0, 0.0],
                [
                    0.01675516081914557,
                    0.01675516081914557,
                    0.01675516081914557,
                ],
                [
                    0.01675516081914557,
                    0.01675516081914557,
                    0.01675516081914557,
                ],
            ],
            body_ipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            body_iquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
            ],
            qpos0: &[0.0, 0.0, 2.0, 1.0, 0.0, 0.0, 0.0],
            qpos_spring: &[0.0, 0.0, 2.0, 1.0, 0.0, 0.0, 0.0],
            eq_data: &[],
        }),
    ),
    // a moving body whose mass is a static child's <inertial>: added (L373)
    (
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="p" pos="0 0 1"><joint name="h" type="hinge" axis="0 1 0"/><body name="s" pos="0.3 0 0"><inertial pos="0 0 0" mass="2" diaginertia="0.01 0.02 0.03"/></body></body></worldbody></mujoco>"#,
        Expect::Loads(&Model {
            body_name: &["world", "p"],
            body_parent: &[0, 0],
            body_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 1.0]],
            body_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            geom_body: &[],
            geom_pos: &[],
            geom_quat: &[],
            site_body: &[],
            site_pos: &[],
            site_quat: &[],
            body_rootid: &[0, 1],
            body_geoms: &[],
            jnt_body: &[1],
            dof_body: &[1],
            body_mass: &[0.0, 2.0],
            body_inertia: &[[0.0, 0.0, 0.0], [0.03, 0.02, 0.01]],
            body_ipos: &[[0.0, 0.0, 0.0], [0.3, 0.0, 0.0]],
            body_iquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 0.7071067811865489, 0.0, 0.7071067811865489],
            ],
            qpos0: &[0.0],
            qpos_spring: &[0.0],
            eq_data: &[],
        }),
    ),
    // a fused body's children in its place, before its later sibling
    (
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="p" pos="0 0 1"><freejoint/><geom size="0.1"/><body name="a" pos="0.2 0 0"><geom size="0.05"/><body name="c" pos="0.1 0 0"><joint type="hinge"/><geom size="0.03"/></body></body><body name="d" pos="-0.2 0 0"><joint type="hinge"/><geom size="0.04"/></body></body></worldbody></mujoco>"#,
        Expect::Loads(&Model {
            body_name: &["world", "p", "c", "d"],
            body_parent: &[0, 0, 1, 1],
            body_pos: &[
                [0.0, 0.0, 0.0],
                [0.0, 0.0, 1.0],
                [0.30000000000000004, 0.0, 0.0],
                [-0.2, 0.0, 0.0],
            ],
            body_quat: &[
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
            ],
            geom_body: &[1, 1, 2, 3],
            geom_pos: &[
                [0.0, 0.0, 0.0],
                [0.2, 0.0, 0.0],
                [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0],
            ],
            geom_quat: &[
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
            ],
            site_body: &[],
            site_pos: &[],
            site_quat: &[],
            body_rootid: &[0, 1, 1, 1],
            body_geoms: &[(1, 0, 2), (2, 2, 1), (3, 3, 1)],
            jnt_body: &[1, 2, 3],
            dof_body: &[1, 1, 1, 1, 1, 1, 2, 3],
            body_mass: &[
                0.0,
                4.712388980384691,
                0.11309733552923254,
                0.26808257310632905,
            ],
            body_inertia: &[
                [0.0, 0.0, 0.0],
                [
                    0.035895604949350055,
                    0.035895604949350055,
                    0.017278759594743866,
                ],
                [
                    4.071504079052371e-05,
                    4.071504079052371e-05,
                    4.071504079052371e-05,
                ],
                [
                    0.00017157284678805058,
                    0.00017157284678805058,
                    0.00017157284678805058,
                ],
            ],
            body_ipos: &[
                [0.0, 0.0, 0.0],
                [0.022222222222222223, 0.0, 0.0],
                [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0],
            ],
            body_iquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [
                    0.5000000000000007,
                    0.5000000000000007,
                    0.5000000000000007,
                    0.5000000000000007,
                ],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
            ],
            qpos0: &[0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            qpos_spring: &[0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            eq_data: &[],
        }),
    ),
    // a camera's empty target names no body: the static body fused
    (
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="s" pos="0 0 1"><geom size="0.1" contype="0" conaffinity="0"/></body><camera name="cam" target="" pos="1 1 1"/><body name="f" pos="0 0 2"><freejoint/><geom size="0.1"/></body></worldbody></mujoco>"#,
        Expect::Loads(&Model {
            body_name: &["world", "f"],
            body_parent: &[0, 0],
            body_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 2.0]],
            body_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            geom_body: &[0, 1],
            geom_pos: &[[0.0, 0.0, 1.0], [0.0, 0.0, 0.0]],
            geom_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            site_body: &[],
            site_pos: &[],
            site_quat: &[],
            body_rootid: &[0, 1],
            body_geoms: &[(0, 0, 1), (1, 1, 1)],
            jnt_body: &[1],
            dof_body: &[1, 1, 1, 1, 1, 1],
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
            qpos0: &[0.0, 0.0, 2.0, 1.0, 0.0, 0.0, 0.0],
            qpos_spring: &[0.0, 0.0, 2.0, 1.0, 0.0, 0.0, 0.0],
            eq_data: &[],
        }),
    ),
    // a static body a light in another body targets: kept
    (
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="t" pos="0 0 1"><geom size="0.1" contype="0" conaffinity="0"/></body><body name="f" pos="0 0 2"><freejoint/><geom size="0.1"/><light name="l" mode="targetbody" target="t" pos="0 0 1"/></body></worldbody></mujoco>"#,
        Expect::Loads(&Model {
            body_name: &["world", "t", "f"],
            body_parent: &[0, 0, 0],
            body_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, 0.0, 2.0]],
            body_quat: &[
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
            ],
            geom_body: &[1, 2],
            geom_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            geom_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            site_body: &[],
            site_pos: &[],
            site_quat: &[],
            body_rootid: &[0, 1, 2],
            body_geoms: &[(1, 0, 1), (2, 1, 1)],
            jnt_body: &[2],
            dof_body: &[2, 2, 2, 2, 2, 2],
            body_mass: &[0.0, 4.188790204786391, 4.188790204786391],
            body_inertia: &[
                [0.0, 0.0, 0.0],
                [
                    0.01675516081914557,
                    0.01675516081914557,
                    0.01675516081914557,
                ],
                [
                    0.01675516081914557,
                    0.01675516081914557,
                    0.01675516081914557,
                ],
            ],
            body_ipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            body_iquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
            ],
            qpos0: &[0.0, 0.0, 2.0, 1.0, 0.0, 0.0, 0.0],
            qpos_spring: &[0.0, 0.0, 2.0, 1.0, 0.0, 0.0, 0.0],
            eq_data: &[],
        }),
    ),
    // geoms and sites re-ordered across bodies by fusion (round 1 RB-2)
    (
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="P" pos="0 0 1"><joint type="hinge"/><geom name="gp" size="0.1"/><site name="sp"/><body name="A" pos="0.3 0 0"><joint type="hinge"/><geom name="ga" size="0.05"/><site name="sa" pos="0.01 0 0"/></body><body pos="0 0.3 0"><geom name="gs" size="0.04"/><site name="ss" pos="0 0.02 0"/></body></body><body name="F" pos="2 0 1"><freejoint/><geom name="gf" size="0.1"/></body></worldbody></mujoco>"#,
        Expect::Loads(&Model {
            body_name: &["world", "P", "A", "F"],
            body_parent: &[0, 0, 1, 0],
            body_pos: &[
                [0.0, 0.0, 0.0],
                [0.0, 0.0, 1.0],
                [0.3, 0.0, 0.0],
                [2.0, 0.0, 1.0],
            ],
            body_quat: &[
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
            ],
            geom_body: &[1, 1, 2, 3],
            geom_pos: &[
                [0.0, 0.0, 0.0],
                [0.0, 0.3, 0.0],
                [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0],
            ],
            geom_quat: &[
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
            ],
            site_body: &[1, 1, 2],
            site_pos: &[[0.0, 0.0, 0.0], [0.0, 0.32, 0.0], [0.01, 0.0, 0.0]],
            site_quat: &[
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
            ],
            body_rootid: &[0, 1, 1, 3],
            body_geoms: &[(1, 0, 2), (2, 2, 1), (3, 3, 1)],
            jnt_body: &[1, 2, 3],
            dof_body: &[1, 2, 3, 3, 3, 3, 3, 3],
            body_mass: &[
                0.0,
                4.456872777892721,
                0.5235987755982989,
                4.188790204786391,
            ],
            body_inertia: &[
                [0.0, 0.0, 0.0],
                [
                    0.039602891165529126,
                    0.039602891165529126,
                    0.01692673366593362,
                ],
                [
                    0.000523598775598299,
                    0.000523598775598299,
                    0.000523598775598299,
                ],
                [
                    0.01675516081914557,
                    0.01675516081914557,
                    0.01675516081914557,
                ],
            ],
            body_ipos: &[
                [0.0, 0.0, 0.0],
                [0.0, 0.01804511278195489, 0.0],
                [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0],
            ],
            body_iquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [0.707106781186548, 0.707106781186548, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
            ],
            qpos0: &[0.0, 0.0, 2.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0],
            qpos_spring: &[0.0, 0.0, 2.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0],
            eq_data: &[],
        }),
    ),
    // a weld and a connect on bodies fusion renumbers: their eq_data
    (
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body pos="0.3 0.1 0.2"><geom size="0.05"/></body><body name="f" pos="0 0 1" euler="10 20 30"><freejoint/><geom size="0.1"/></body><body name="k" pos="0.5 0.2 1.3" euler="-5 15 40"><freejoint/><geom size="0.1"/></body></worldbody><equality><weld body1="f" body2="k"/><connect body1="k" body2="f" anchor="0.1 0 0"/></equality></mujoco>"#,
        Expect::Loads(&Model {
            body_name: &["world", "f", "k"],
            body_parent: &[0, 0, 0],
            body_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.5, 0.2, 1.3]],
            body_quat: &[
                [1.0, 0.0, 0.0, 0.0],
                [
                    0.943714364147489,
                    0.12767944069578063,
                    0.14487812541736914,
                    0.2685358227515692,
                ],
                [
                    0.9327139748246638,
                    0.00396194569632418,
                    0.13732883723664985,
                    0.33342125699526975,
                ],
            ],
            geom_body: &[0, 1, 2],
            geom_pos: &[[0.3, 0.1, 0.2], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            geom_quat: &[
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
            ],
            site_body: &[],
            site_pos: &[],
            site_quat: &[],
            body_rootid: &[0, 1, 2],
            body_geoms: &[(0, 0, 1), (1, 1, 1), (2, 2, 1)],
            jnt_body: &[1, 2],
            dof_body: &[1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2],
            body_mass: &[0.0, 4.188790204786391, 4.188790204786391],
            body_inertia: &[
                [0.0, 0.0, 0.0],
                [
                    0.01675516081914557,
                    0.01675516081914557,
                    0.01675516081914557,
                ],
                [
                    0.01675516081914557,
                    0.01675516081914557,
                    0.01675516081914557,
                ],
            ],
            body_ipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            body_iquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
            ],
            qpos0: &[
                0.0,
                0.0,
                1.0,
                0.943714364147489,
                0.12767944069578063,
                0.14487812541736914,
                0.2685358227515692,
                0.5,
                0.2,
                1.3,
                0.9327139748246638,
                0.00396194569632418,
                0.13732883723664985,
                0.33342125699526975,
            ],
            qpos_spring: &[
                0.0,
                0.0,
                1.0,
                0.943714364147489,
                0.12767944069578063,
                0.14487812541736914,
                0.2685358227515692,
                0.5,
                0.2,
                1.3,
                0.9327139748246638,
                0.00396194569632418,
                0.13732883723664985,
                0.33342125699526975,
            ],
            eq_data: &[
                [
                    0.0,
                    0.0,
                    0.0,
                    0.45420423056029324,
                    0.025350167011773464,
                    0.41599986294902436,
                    0.9901529307683885,
                    -0.1267771879640755,
                    0.03597645935069925,
                    0.047227245052831546,
                    1.0,
                ],
                [
                    0.1,
                    0.0,
                    0.0,
                    0.5534992868997546,
                    0.03379040716213197,
                    0.40767795615145375,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    1.0,
                ],
            ],
        }),
    ),
    // a stiff free joint below a turned fused body: its qpos_spring
    (
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="s" pos="0.3 0.2 0.5" euler="10 20 30"><geom size="0.1"/><body name="f" pos="0.1 0.2 0.3" euler="5 6 7"><joint type="free" stiffness="3"/><geom size="0.1"/></body></body></worldbody></mujoco>"#,
        Expect::Loads(&Model {
            body_name: &["world", "f"],
            body_parent: &[0, 0],
            body_pos: &[
                [0.0, 0.0, 0.0],
                [0.3900165490540471, 0.37006562982737234, 0.8208967161686443],
            ],
            body_quat: &[
                [1.0, 0.0, 0.0, 0.0],
                [
                    0.9095342823939611,
                    0.16702497862838267,
                    0.1954588370747527,
                    0.3265668211464165,
                ],
            ],
            geom_body: &[0, 1],
            geom_pos: &[[0.3, 0.2, 0.5], [0.0, 0.0, 0.0]],
            geom_quat: &[
                [
                    0.943714364147489,
                    0.12767944069578063,
                    0.14487812541736914,
                    0.2685358227515692,
                ],
                [1.0, 0.0, 0.0, 0.0],
            ],
            site_body: &[],
            site_pos: &[],
            site_quat: &[],
            body_rootid: &[0, 1],
            body_geoms: &[(0, 0, 1), (1, 1, 1)],
            jnt_body: &[1],
            dof_body: &[1, 1, 1, 1, 1, 1],
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
            qpos0: &[
                0.3900165490540471,
                0.37006562982737234,
                0.8208967161686443,
                0.9095342823939611,
                0.16702497862838267,
                0.1954588370747527,
                0.3265668211464165,
            ],
            qpos_spring: &[
                0.3900165490540471,
                0.37006562982737234,
                0.8208967161686443,
                0.9095342823939611,
                0.16702497862838267,
                0.1954588370747527,
                0.3265668211464165,
            ],
            eq_data: &[],
        }),
    ),
    // hinge chains below fused bodies: root ids, joint and dof bodies
    (
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="f" pos="0 0 1"><freejoint/><geom size="0.1"/><body pos="0.2 0 0" euler="0 0 30"><geom size="0.05"/><body name="h1" pos="0.1 0 0"><joint type="hinge" axis="0 1 0"/><geom size="0.04" pos="0.05 0 0"/><body pos="0.1 0 0"><geom size="0.03"/><body name="h2" pos="0.1 0 0"><joint type="hinge" axis="1 0 0"/><geom size="0.02" pos="0 0 0.03"/></body></body></body></body></body><body pos="1 0 0"><geom size="0.1"/><body name="g" pos="0 0 1"><freejoint/><geom size="0.1"/></body></body></worldbody><sensor><subtreecom body="h1"/><framepos objtype="body" objname="h2"/><framelinvel objtype="body" objname="g"/></sensor></mujoco>"#,
        Expect::Loads(&Model {
            body_name: &["world", "f", "h1", "h2", "g"],
            body_parent: &[0, 0, 1, 2, 0],
            body_pos: &[
                [0.0, 0.0, 0.0],
                [0.0, 0.0, 1.0],
                [0.2866025403784439, 0.049999999999999996, 0.0],
                [0.2, 0.0, 0.0],
                [1.0, 0.0, 1.0],
            ],
            body_quat: &[
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [0.9659258262890683, 0.0, 0.0, 0.25881904510252074],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
            ],
            geom_body: &[0, 1, 1, 2, 2, 3, 4],
            geom_pos: &[
                [1.0, 0.0, 0.0],
                [0.0, 0.0, 0.0],
                [0.2, 0.0, 0.0],
                [0.05, 0.0, 0.0],
                [0.1, 0.0, 0.0],
                [0.0, 0.0, 0.03],
                [0.0, 0.0, 0.0],
            ],
            geom_quat: &[
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [0.9659258262890683, 0.0, 0.0, 0.25881904510252074],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
            ],
            site_body: &[],
            site_pos: &[],
            site_quat: &[],
            body_rootid: &[0, 1, 1, 1, 4],
            body_geoms: &[(0, 0, 1), (1, 1, 2), (2, 3, 2), (3, 5, 1), (4, 6, 1)],
            jnt_body: &[1, 2, 3, 4],
            dof_body: &[1, 1, 1, 1, 1, 1, 2, 3, 4, 4, 4, 4, 4, 4],
            body_mass: &[
                0.0,
                4.712388980384691,
                0.3811799086355616,
                0.03351032163829113,
                4.188790204786391,
            ],
            body_inertia: &[
                [0.0, 0.0, 0.0],
                [
                    0.035895604949350055,
                    0.035895604949350055,
                    0.017278759594743866,
                ],
                [
                    0.00041114034565195026,
                    0.00041114034565195026,
                    0.0002122878875785743,
                ],
                [
                    5.361651462126581e-06,
                    5.361651462126581e-06,
                    5.361651462126581e-06,
                ],
                [
                    0.01675516081914557,
                    0.01675516081914557,
                    0.01675516081914557,
                ],
            ],
            body_ipos: &[
                [0.0, 0.0, 0.0],
                [0.022222222222222223, 0.0, 0.0],
                [0.06483516483516483, 0.0, 0.0],
                [0.0, 0.0, 0.03],
                [0.0, 0.0, 0.0],
            ],
            body_iquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [
                    0.5000000000000007,
                    0.5000000000000007,
                    0.5000000000000007,
                    0.5000000000000007,
                ],
                [
                    0.5000000000000007,
                    0.5000000000000007,
                    0.5000000000000007,
                    0.5000000000000007,
                ],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
            ],
            qpos0: &[
                0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0,
            ],
            qpos_spring: &[
                0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0,
            ],
            eq_data: &[],
        }),
    ),
    // a kept name put back at the loop's index, not its first (round 1 RB-3)
    (
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body pos="0 0 1"><geom size="0.1"/></body><body pos="1 0 1"><geom size="0.1"/></body><body name="K" pos="2 0 1"><joint type="hinge"/><geom size="0.1"/></body><body name="S" pos="3 0 1"><geom size="0.1"/></body></worldbody><sensor><framepos objtype="body" objname="K"/></sensor></mujoco>"#,
        Expect::Loads(&Model {
            body_name: &["world", "K"],
            body_parent: &[0, 0],
            body_pos: &[[0.0, 0.0, 0.0], [2.0, 0.0, 1.0]],
            body_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            geom_body: &[0, 0, 0, 1],
            geom_pos: &[
                [0.0, 0.0, 1.0],
                [1.0, 0.0, 1.0],
                [3.0, 0.0, 1.0],
                [0.0, 0.0, 0.0],
            ],
            geom_quat: &[
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
            ],
            site_body: &[],
            site_pos: &[],
            site_quat: &[],
            body_rootid: &[0, 1],
            body_geoms: &[(0, 0, 3), (1, 3, 1)],
            jnt_body: &[1],
            dof_body: &[1],
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
            qpos0: &[0.0],
            qpos_spring: &[0.0],
            eq_data: &[],
        }),
    ),
    // a static body a distance sensor's body1 names: kept
    (
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="s" pos="0 0 0.5"><geom size="0.2"/><site name="ss"/></body><body name="f" pos="0 0 2"><freejoint/><geom size="0.1"/><site name="fs"/></body></worldbody><sensor><distance body1="s" body2="f"/></sensor></mujoco>"#,
        Expect::Loads(&Model {
            body_name: &["world", "s", "f"],
            body_parent: &[0, 0, 0],
            body_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5], [0.0, 0.0, 2.0]],
            body_quat: &[
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
            ],
            geom_body: &[1, 2],
            geom_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            geom_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            site_body: &[1, 2],
            site_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            site_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            body_rootid: &[0, 1, 2],
            body_geoms: &[(1, 0, 1), (2, 1, 1)],
            jnt_body: &[2],
            dof_body: &[2, 2, 2, 2, 2, 2],
            body_mass: &[0.0, 33.51032163829113, 4.188790204786391],
            body_inertia: &[
                [0.0, 0.0, 0.0],
                [0.5361651462126582, 0.5361651462126582, 0.5361651462126582],
                [
                    0.01675516081914557,
                    0.01675516081914557,
                    0.01675516081914557,
                ],
            ],
            body_ipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            body_iquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
            ],
            qpos0: &[0.0, 0.0, 2.0, 1.0, 0.0, 0.0, 0.0],
            qpos_spring: &[0.0, 0.0, 2.0, 1.0, 0.0, 0.0, 0.0],
            eq_data: &[],
        }),
    ),
    // a static body a frame sensor's reftype body names: kept
    (
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="s" pos="0 0 0.5"><geom size="0.2"/><site name="ss"/></body><body name="f" pos="0 0 2"><freejoint/><geom size="0.1"/><site name="fs"/></body></worldbody><sensor><framepos objtype="body" objname="f" reftype="body" refname="s"/></sensor></mujoco>"#,
        Expect::Loads(&Model {
            body_name: &["world", "s", "f"],
            body_parent: &[0, 0, 0],
            body_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5], [0.0, 0.0, 2.0]],
            body_quat: &[
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
            ],
            geom_body: &[1, 2],
            geom_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            geom_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            site_body: &[1, 2],
            site_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            site_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            body_rootid: &[0, 1, 2],
            body_geoms: &[(1, 0, 1), (2, 1, 1)],
            jnt_body: &[2],
            dof_body: &[2, 2, 2, 2, 2, 2],
            body_mass: &[0.0, 33.51032163829113, 4.188790204786391],
            body_inertia: &[
                [0.0, 0.0, 0.0],
                [0.5361651462126582, 0.5361651462126582, 0.5361651462126582],
                [
                    0.01675516081914557,
                    0.01675516081914557,
                    0.01675516081914557,
                ],
            ],
            body_ipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            body_iquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
            ],
            qpos0: &[0.0, 0.0, 2.0, 1.0, 0.0, 0.0, 0.0],
            qpos_spring: &[0.0, 0.0, 2.0, 1.0, 0.0, 0.0, 0.0],
            eq_data: &[],
        }),
    ),
    // a static body a user sensor's objtype body names: kept
    (
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="s" pos="0 0 0.5"><geom size="0.2"/><site name="ss"/></body><body name="f" pos="0 0 2"><freejoint/><geom size="0.1"/><site name="fs"/></body></worldbody><sensor><user objtype="body" objname="s" dim="1"/></sensor></mujoco>"#,
        Expect::Loads(&Model {
            body_name: &["world", "s", "f"],
            body_parent: &[0, 0, 0],
            body_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5], [0.0, 0.0, 2.0]],
            body_quat: &[
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
            ],
            geom_body: &[1, 2],
            geom_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            geom_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            site_body: &[1, 2],
            site_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            site_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            body_rootid: &[0, 1, 2],
            body_geoms: &[(1, 0, 1), (2, 1, 1)],
            jnt_body: &[2],
            dof_body: &[2, 2, 2, 2, 2, 2],
            body_mass: &[0.0, 33.51032163829113, 4.188790204786391],
            body_inertia: &[
                [0.0, 0.0, 0.0],
                [0.5361651462126582, 0.5361651462126582, 0.5361651462126582],
                [
                    0.01675516081914557,
                    0.01675516081914557,
                    0.01675516081914557,
                ],
            ],
            body_ipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            body_iquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
            ],
            qpos0: &[0.0, 0.0, 2.0, 1.0, 0.0, 0.0, 0.0],
            qpos_spring: &[0.0, 0.0, 2.0, 1.0, 0.0, 0.0, 0.0],
            eq_data: &[],
        }),
    ),
    // a static body holding a torque sensor's site: kept
    (
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="s" pos="0 0 0.5"><geom size="0.2"/><site name="ss"/></body><body name="f" pos="0 0 2"><freejoint/><geom size="0.1"/><site name="fs"/></body></worldbody><sensor><torque site="ss"/></sensor></mujoco>"#,
        Expect::Loads(&Model {
            body_name: &["world", "s", "f"],
            body_parent: &[0, 0, 0],
            body_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5], [0.0, 0.0, 2.0]],
            body_quat: &[
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
            ],
            geom_body: &[1, 2],
            geom_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            geom_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            site_body: &[1, 2],
            site_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            site_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            body_rootid: &[0, 1, 2],
            body_geoms: &[(1, 0, 1), (2, 1, 1)],
            jnt_body: &[2],
            dof_body: &[2, 2, 2, 2, 2, 2],
            body_mass: &[0.0, 33.51032163829113, 4.188790204786391],
            body_inertia: &[
                [0.0, 0.0, 0.0],
                [0.5361651462126582, 0.5361651462126582, 0.5361651462126582],
                [
                    0.01675516081914557,
                    0.01675516081914557,
                    0.01675516081914557,
                ],
            ],
            body_ipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            body_iquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
            ],
            qpos0: &[0.0, 0.0, 2.0, 1.0, 0.0, 0.0, 0.0],
            qpos_spring: &[0.0, 0.0, 2.0, 1.0, 0.0, 0.0, 0.0],
            eq_data: &[],
        }),
    ),
    // a static body a connect names: kept
    (
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="s" pos="0 0 0.5"><geom size="0.2"/><site name="ss"/></body><body name="f" pos="0 0 2"><freejoint/><geom size="0.1"/><site name="fs"/></body></worldbody><equality><connect body1="f" body2="s" anchor="0 0 0"/></equality></mujoco>"#,
        Expect::Loads(&Model {
            body_name: &["world", "s", "f"],
            body_parent: &[0, 0, 0],
            body_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5], [0.0, 0.0, 2.0]],
            body_quat: &[
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
            ],
            geom_body: &[1, 2],
            geom_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            geom_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            site_body: &[1, 2],
            site_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            site_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            body_rootid: &[0, 1, 2],
            body_geoms: &[(1, 0, 1), (2, 1, 1)],
            jnt_body: &[2],
            dof_body: &[2, 2, 2, 2, 2, 2],
            body_mass: &[0.0, 33.51032163829113, 4.188790204786391],
            body_inertia: &[
                [0.0, 0.0, 0.0],
                [0.5361651462126582, 0.5361651462126582, 0.5361651462126582],
                [
                    0.01675516081914557,
                    0.01675516081914557,
                    0.01675516081914557,
                ],
            ],
            body_ipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            body_iquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
            ],
            qpos0: &[0.0, 0.0, 2.0, 1.0, 0.0, 0.0, 0.0],
            qpos_spring: &[0.0, 0.0, 2.0, 1.0, 0.0, 0.0, 0.0],
            eq_data: &[[0.0, 0.0, 0.0, 0.0, 0.0, 1.5, 0.0, 0.0, 0.0, 0.0, 1.0]],
        }),
    ),
    // a static body a subtreelinvel names: kept
    (
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="s" pos="0 0 0.5"><geom size="0.2"/><site name="ss"/></body><body name="f" pos="0 0 2"><freejoint/><geom size="0.1"/><site name="fs"/></body></worldbody><sensor><subtreelinvel body="s"/><subtreeangmom body="s"/></sensor></mujoco>"#,
        Expect::Loads(&Model {
            body_name: &["world", "s", "f"],
            body_parent: &[0, 0, 0],
            body_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5], [0.0, 0.0, 2.0]],
            body_quat: &[
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
            ],
            geom_body: &[1, 2],
            geom_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            geom_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            site_body: &[1, 2],
            site_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            site_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            body_rootid: &[0, 1, 2],
            body_geoms: &[(1, 0, 1), (2, 1, 1)],
            jnt_body: &[2],
            dof_body: &[2, 2, 2, 2, 2, 2],
            body_mass: &[0.0, 33.51032163829113, 4.188790204786391],
            body_inertia: &[
                [0.0, 0.0, 0.0],
                [0.5361651462126582, 0.5361651462126582, 0.5361651462126582],
                [
                    0.01675516081914557,
                    0.01675516081914557,
                    0.01675516081914557,
                ],
            ],
            body_ipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            body_iquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
            ],
            qpos0: &[0.0, 0.0, 2.0, 1.0, 0.0, 0.0, 0.0],
            qpos_spring: &[0.0, 0.0, 2.0, 1.0, 0.0, 0.0, 0.0],
            eq_data: &[],
        }),
    ),
    // a static body a skin's bone names: kept
    (
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="s" pos="0 0 0.5"><geom size="0.2"/></body><body name="f" pos="0 0 2"><freejoint/><geom size="0.1"/></body></worldbody><deformable><skin name="sk" vertex="0 0 0 1 0 0 0 1 0" face="0 1 2"><bone body="s" bindpos="0 0 0" bindquat="1 0 0 0" vertid="0 1 2" vertweight="1 1 1"/></skin></deformable></mujoco>"#,
        Expect::Loads(&Model {
            body_name: &["world", "s", "f"],
            body_parent: &[0, 0, 0],
            body_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5], [0.0, 0.0, 2.0]],
            body_quat: &[
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
            ],
            geom_body: &[1, 2],
            geom_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            geom_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            site_body: &[],
            site_pos: &[],
            site_quat: &[],
            body_rootid: &[0, 1, 2],
            body_geoms: &[(1, 0, 1), (2, 1, 1)],
            jnt_body: &[2],
            dof_body: &[2, 2, 2, 2, 2, 2],
            body_mass: &[0.0, 33.51032163829113, 4.188790204786391],
            body_inertia: &[
                [0.0, 0.0, 0.0],
                [0.5361651462126582, 0.5361651462126582, 0.5361651462126582],
                [
                    0.01675516081914557,
                    0.01675516081914557,
                    0.01675516081914557,
                ],
            ],
            body_ipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            body_iquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
            ],
            qpos0: &[0.0, 0.0, 2.0, 1.0, 0.0, 0.0, 0.0],
            qpos_spring: &[0.0, 0.0, 2.0, 1.0, 0.0, 0.0, 0.0],
            eq_data: &[],
        }),
    ),
    // a light in a nested frame targets a static body: kept
    (
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="t" pos="0 0 1"><geom size="0.1" contype="0" conaffinity="0"/></body><frame pos="1 0 0"><frame pos="0 1 0"><light name="l" mode="targetbody" target="t" pos="1 1 3"/></frame></frame><body name="f" pos="0 0 2"><freejoint/><geom size="0.1"/></body></worldbody></mujoco>"#,
        Expect::Loads(&Model {
            body_name: &["world", "t", "f"],
            body_parent: &[0, 0, 0],
            body_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, 0.0, 2.0]],
            body_quat: &[
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
            ],
            geom_body: &[1, 2],
            geom_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            geom_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            site_body: &[],
            site_pos: &[],
            site_quat: &[],
            body_rootid: &[0, 1, 2],
            body_geoms: &[(1, 0, 1), (2, 1, 1)],
            jnt_body: &[2],
            dof_body: &[2, 2, 2, 2, 2, 2],
            body_mass: &[0.0, 4.188790204786391, 4.188790204786391],
            body_inertia: &[
                [0.0, 0.0, 0.0],
                [
                    0.01675516081914557,
                    0.01675516081914557,
                    0.01675516081914557,
                ],
                [
                    0.01675516081914557,
                    0.01675516081914557,
                    0.01675516081914557,
                ],
            ],
            body_ipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            body_iquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
            ],
            qpos0: &[0.0, 0.0, 2.0, 1.0, 0.0, 0.0, 0.0],
            qpos_spring: &[0.0, 0.0, 2.0, 1.0, 0.0, 0.0, 0.0],
            eq_data: &[],
        }),
    ),
    // a camera in a frame in a body targets a static body: kept
    (
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="t" pos="0 0 1"><geom size="0.1" contype="0" conaffinity="0"/></body><body name="f" pos="0 0 2"><freejoint/><geom size="0.1"/><frame pos="0.1 0 0"><camera name="c" mode="targetbody" target="t"/></frame></body></worldbody></mujoco>"#,
        Expect::Loads(&Model {
            body_name: &["world", "t", "f"],
            body_parent: &[0, 0, 0],
            body_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, 0.0, 2.0]],
            body_quat: &[
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
            ],
            geom_body: &[1, 2],
            geom_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            geom_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            site_body: &[],
            site_pos: &[],
            site_quat: &[],
            body_rootid: &[0, 1, 2],
            body_geoms: &[(1, 0, 1), (2, 1, 1)],
            jnt_body: &[2],
            dof_body: &[2, 2, 2, 2, 2, 2],
            body_mass: &[0.0, 4.188790204786391, 4.188790204786391],
            body_inertia: &[
                [0.0, 0.0, 0.0],
                [
                    0.01675516081914557,
                    0.01675516081914557,
                    0.01675516081914557,
                ],
                [
                    0.01675516081914557,
                    0.01675516081914557,
                    0.01675516081914557,
                ],
            ],
            body_ipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            body_iquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
            ],
            qpos0: &[0.0, 0.0, 2.0, 1.0, 0.0, 0.0, 0.0],
            qpos_spring: &[0.0, 0.0, 2.0, 1.0, 0.0, 0.0, 0.0],
            eq_data: &[],
        }),
    ),
    // a camera's target in mode fixed: kept
    (
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="t" pos="0 0 1"><geom size="0.1" contype="0" conaffinity="0"/></body><body name="f" pos="0 0 2"><freejoint/><geom size="0.1"/><camera name="c" mode="fixed" target="t"/></body></worldbody></mujoco>"#,
        Expect::Loads(&Model {
            body_name: &["world", "t", "f"],
            body_parent: &[0, 0, 0],
            body_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, 0.0, 2.0]],
            body_quat: &[
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
            ],
            geom_body: &[1, 2],
            geom_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            geom_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            site_body: &[],
            site_pos: &[],
            site_quat: &[],
            body_rootid: &[0, 1, 2],
            body_geoms: &[(1, 0, 1), (2, 1, 1)],
            jnt_body: &[2],
            dof_body: &[2, 2, 2, 2, 2, 2],
            body_mass: &[0.0, 4.188790204786391, 4.188790204786391],
            body_inertia: &[
                [0.0, 0.0, 0.0],
                [
                    0.01675516081914557,
                    0.01675516081914557,
                    0.01675516081914557,
                ],
                [
                    0.01675516081914557,
                    0.01675516081914557,
                    0.01675516081914557,
                ],
            ],
            body_ipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            body_iquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
            ],
            qpos0: &[0.0, 0.0, 2.0, 1.0, 0.0, 0.0, 0.0],
            qpos_spring: &[0.0, 0.0, 2.0, 1.0, 0.0, 0.0, 0.0],
            eq_data: &[],
        }),
    ),
    // a free body below a static body fused to the top (the same with a camera in the static body stops MuJoCo: D-FUSESTATIC-CAMERA)
    (
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="t" pos="0 0 1"><geom size="0.1"/><body name="c" pos="0.3 0 0"><freejoint/><geom size="0.1"/></body></body></worldbody></mujoco>"#,
        Expect::Loads(&Model {
            body_name: &["world", "c"],
            body_parent: &[0, 0],
            body_pos: &[[0.0, 0.0, 0.0], [0.3, 0.0, 1.0]],
            body_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            geom_body: &[0, 1],
            geom_pos: &[[0.0, 0.0, 1.0], [0.0, 0.0, 0.0]],
            geom_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            site_body: &[],
            site_pos: &[],
            site_quat: &[],
            body_rootid: &[0, 1],
            body_geoms: &[(0, 0, 1), (1, 1, 1)],
            jnt_body: &[1],
            dof_body: &[1, 1, 1, 1, 1, 1],
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
            qpos0: &[0.3, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0],
            qpos_spring: &[0.3, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0],
            eq_data: &[],
        }),
    ),
    // a nested frame's target in a body: kept
    (
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="t" pos="0 0 1"><geom size="0.1" contype="0" conaffinity="0"/></body><body name="f" pos="0 0 2"><freejoint/><geom size="0.1"/><frame><frame><camera name="c" target="t"/></frame></frame></body></worldbody></mujoco>"#,
        Expect::Loads(&Model {
            body_name: &["world", "t", "f"],
            body_parent: &[0, 0, 0],
            body_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, 0.0, 2.0]],
            body_quat: &[
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
            ],
            geom_body: &[1, 2],
            geom_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            geom_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            site_body: &[],
            site_pos: &[],
            site_quat: &[],
            body_rootid: &[0, 1, 2],
            body_geoms: &[(1, 0, 1), (2, 1, 1)],
            jnt_body: &[2],
            dof_body: &[2, 2, 2, 2, 2, 2],
            body_mass: &[0.0, 4.188790204786391, 4.188790204786391],
            body_inertia: &[
                [0.0, 0.0, 0.0],
                [
                    0.01675516081914557,
                    0.01675516081914557,
                    0.01675516081914557,
                ],
                [
                    0.01675516081914557,
                    0.01675516081914557,
                    0.01675516081914557,
                ],
            ],
            body_ipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            body_iquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
            ],
            qpos0: &[0.0, 0.0, 2.0, 1.0, 0.0, 0.0, 0.0],
            qpos_spring: &[0.0, 0.0, 2.0, 1.0, 0.0, 0.0, 0.0],
            eq_data: &[],
        }),
    ),
    // a sum whose centre overflows: the inertia left as it was (the NaN branch)
    (
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="p" pos="0 0 1"><joint name="h" type="hinge" axis="0 1 0"/><geom type="box" size="0.1 0.05 0.02"/><body name="s" pos="0 1e308 0"><inertial pos="0 1e308 0" mass="2" diaginertia="1 1 1"/></body></body></worldbody></mujoco>"#,
        Expect::Loads(&Model {
            body_name: &["world", "p"],
            body_parent: &[0, 0],
            body_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 1.0]],
            body_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            geom_body: &[1],
            geom_pos: &[[0.0, 0.0, 0.0]],
            geom_quat: &[[1.0, 0.0, 0.0, 0.0]],
            site_body: &[],
            site_pos: &[],
            site_quat: &[],
            body_rootid: &[0, 1],
            body_geoms: &[(1, 0, 1)],
            jnt_body: &[1],
            dof_body: &[1],
            body_mass: &[0.0, 2.8000000000000003],
            body_inertia: &[
                [0.0, 0.0, 0.0],
                [
                    0.0007733333333333337,
                    0.0027733333333333342,
                    0.0033333333333333344,
                ],
            ],
            body_ipos: &[[0.0, 0.0, 0.0], [0.0, f64::INFINITY, 0.0]],
            body_iquat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            qpos0: &[0.0],
            qpos_spring: &[0.0],
            eq_data: &[],
        }),
    ),
    // a tendon's wrapped geom found through MuJoCo's stale geom map, a box: n kept (round 1 RW-1); MuJoCo's tendon then wraps p0, the geom at s1's old index (D-FUSESTATIC-PAIR-GEOM), this crate's s1
    (
        r#"<mujoco><compiler fusestatic="true"/><worldbody><site name="sb" pos="0 0 3"/><body name="p" pos="0 0 1"><joint name="hp" type="hinge" axis="0 1 0"/><geom name="p0" size="0.1"/><site name="sa" pos="0.1 0 0"/><body name="s1b" pos="0.3 0 0"><joint name="hs" type="hinge" axis="0 1 0"/><geom name="s1" size="0.05"/></body><body pos="-0.3 0 0"><geom name="b0" type="box" size="0.05 0.05 0.05"/></body></body><body name="n" pos="2 0 1"><geom name="n0" size="0.1"/></body><body pos="3 0 1"><geom name="c0" size="0.1"/></body></worldbody><tendon><spatial name="t"><site site="sa"/><geom geom="s1"/><site site="sb"/></spatial></tendon></mujoco>"#,
        Expect::Loads(&Model {
            body_name: &["world", "p", "s1b", "n"],
            body_parent: &[0, 0, 1, 0],
            body_pos: &[
                [0.0, 0.0, 0.0],
                [0.0, 0.0, 1.0],
                [0.3, 0.0, 0.0],
                [2.0, 0.0, 1.0],
            ],
            body_quat: &[
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
            ],
            geom_body: &[0, 1, 1, 2, 3],
            geom_pos: &[
                [3.0, 0.0, 1.0],
                [0.0, 0.0, 0.0],
                [-0.3, 0.0, 0.0],
                [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0],
            ],
            geom_quat: &[
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
            ],
            site_body: &[0, 1],
            site_pos: &[[0.0, 0.0, 3.0], [0.1, 0.0, 0.0]],
            site_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            body_rootid: &[0, 1, 1, 3],
            body_geoms: &[(0, 0, 1), (1, 1, 2), (2, 3, 1), (3, 4, 1)],
            jnt_body: &[1, 2],
            dof_body: &[1, 2],
            body_mass: &[
                0.0,
                5.188790204786391,
                0.5235987755982989,
                4.188790204786391,
            ],
            body_inertia: &[
                [0.0, 0.0, 0.0],
                [
                    0.09107674386362617,
                    0.09107674386362617,
                    0.018421827485812236,
                ],
                [
                    0.000523598775598299,
                    0.000523598775598299,
                    0.000523598775598299,
                ],
                [
                    0.01675516081914557,
                    0.01675516081914557,
                    0.01675516081914557,
                ],
            ],
            body_ipos: &[
                [0.0, 0.0, 0.0],
                [-0.05781694540728695, 0.0, 0.0],
                [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0],
            ],
            body_iquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [
                    0.5000000000000007,
                    0.5000000000000007,
                    0.5000000000000007,
                    0.5000000000000007,
                ],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
            ],
            qpos0: &[0.0, 0.0],
            qpos_spring: &[0.0, 0.0],
            eq_data: &[],
        }),
    ),
    // a negative density beside an <inertial>: no geom mass checked, loads
    (
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="p" pos="0 0 1"><joint name="h" type="hinge" axis="0 1 0"/><geom type="box" size="0.1 0.05 0.02"/><body name="s" pos="0.3 0 0"><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/><geom size="0.05" density="-5"/></body></body></worldbody></mujoco>"#,
        Expect::Loads(&Model {
            body_name: &["world", "p"],
            body_parent: &[0, 0],
            body_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 1.0]],
            body_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            geom_body: &[1, 1],
            geom_pos: &[[0.0, 0.0, 0.0], [0.3, 0.0, 0.0]],
            geom_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            site_body: &[],
            site_pos: &[],
            site_quat: &[],
            body_rootid: &[0, 1],
            body_geoms: &[(1, 0, 2)],
            jnt_body: &[1],
            dof_body: &[1],
            body_mass: &[0.0, 1.8000000000000003],
            body_inertia: &[
                [0.0, 0.0, 0.0],
                [
                    0.05333333333333333,
                    0.05277333333333333,
                    0.010773333333333334,
                ],
            ],
            body_ipos: &[[0.0, 0.0, 0.0], [0.16666666666666663, 0.0, 0.0]],
            body_iquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 0.7071067811865489, 0.0, 0.7071067811865489],
            ],
            qpos0: &[0.0],
            qpos_spring: &[0.0],
            eq_data: &[],
        }),
    ),
    // MuJoCo: "Error: mass, inertia or density are negative in geom"
    (
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="p" pos="0 0 1"><joint name="h" type="hinge" axis="0 1 0"/><geom type="box" size="0.1 0.05 0.02"/><body name="s" pos="0.3 0 0"><geom size="0.05" mass="-1"/></body></body></worldbody></mujoco>"#,
        Expect::Refused("mass, inertia or density are negative in geom (geom #1 in body 's')"),
    ),
    // MuJoCo: "Error: inertia must satisfy A + B >= C; use 'balanceinertia' to fix"
    (
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="p" pos="0 0 1"><joint name="h" type="hinge" axis="0 1 0"/><geom type="box" size="0.1 0.05 0.02"/><body name="s" pos="0.3 0 0"><inertial pos="0 0 0" mass="1" diaginertia="1 1 3"/></body></body></worldbody></mujoco>"#,
        Expect::Refused("inertia must satisfy A + B >= C; use 'balanceinertia' to fix (body 's')"),
    ),
    // MuJoCo: "Error: mass, inertia or density are negative in geom"
    (
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="s" pos="0 0 0.5"><geom size="0.1" density="-5"/></body><body name="f" pos="0 0 2"><freejoint/><geom size="0.1"/></body></worldbody></mujoco>"#,
        Expect::Refused("mass, inertia or density are negative in geom (geom #0 in body 's')"),
    ),
    // MuJoCo: "Error: mass, inertia or density are negative in geom"
    (
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="p" pos="0 0 1"><joint type="hinge"/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/><body name="s" pos="0.3 0 0"><geom size="0.05" density="-5"/></body></body></worldbody></mujoco>"#,
        Expect::Refused("mass, inertia or density are negative in geom (geom #0 in body 's')"),
    ),
    // MuJoCo: "Error: error 'inertia must have positive eigenvalues' in fullinertia"
    (
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="p" pos="0 0 1"><joint name="h" type="hinge" axis="0 1 0"/><geom type="box" size="0.1 0.05 0.02"/><body name="s" pos="0.3 0 0"><inertial pos="0 0 0" mass="1" fullinertia="1 1 1 2 0 0"/></body></body></worldbody></mujoco>"#,
        Expect::Refused("inertia must have positive eigenvalues (body 's')"),
    ),
    // MuJoCo: "Error: invalid condim in geom"
    (
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="a" pos="0 0 1"><geom size="0.1"/></body><body name="c" pos="1 0 1"><freejoint/><geom size="0.1"/><geom size="0.1" condim="2"/></body></worldbody></mujoco>"#,
        Expect::Refused("invalid condim in geom (geom #2 in body 'c')"),
    ),
    // MuJoCo: "Error: invalid condim in geom"
    (
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body pos="0 0 1"><geom size="0.1" condim="2"/></body></worldbody></mujoco>"#,
        Expect::Refused("invalid condim in geom (geom #0 in body #1)"),
    ),
    // MuJoCo: "Error: qhull error"
    (
        r#"<mujoco><compiler fusestatic="true"/><asset><mesh name="flat" vertex="0 0 0 1 0 0 0 1 0 1 1 0"/></asset><worldbody><body name="s" pos="0 0 1" zaxis="0 0 0"><geom type="mesh" mesh="flat"/></body><body name="f" pos="0 0 2"><freejoint/><geom size="0.1"/></body></worldbody></mujoco>"#,
        Expect::Refused("mesh 'flat': qhull error"),
    ),
    // MuJoCo: "Error: free joint can only be used on top level"
    (
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="m" mocap="true" pos="0 0 1"><geom size="0.1" contype="0" conaffinity="0"/><body name="c" pos="0.3 0 0"><freejoint/><geom size="0.1"/></body></body></worldbody></mujoco>"#,
        Expect::Refused("free joint can only be used on top level (body 'c')"),
    ),
    // MuJoCo: "Error: free joint can only be used on top level"
    (
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="t" pos="0 0 1"><geom size="0.1"/><body name="c" pos="0.3 0 0"><freejoint/><geom size="0.1"/></body></body><camera name="cam" mode="targetbody" target="t" pos="1 1 1"/></worldbody></mujoco>"#,
        Expect::Refused("free joint can only be used on top level (body 'c')"),
    ),
    // MuJoCo: "Error: error 'inertia must have positive eigenvalues' in fusing static body inertias"
    (
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="p" pos="0 0 1"><joint name="h" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="0 0 0"/><body name="s" pos="0 0 0"><inertial pos="0 0 0" mass="1" diaginertia="0 0 0"/></body></body></worldbody></mujoco>"#,
        Expect::Refused(
            "error 'inertia must have positive eigenvalues' in fusing static body inertias",
        ),
    ),
    // MuJoCo: "Error: mass, inertia or density are negative in geom"
    (
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="p" pos="0 0 1"><joint name="h" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="0 0 0"/><body name="s" pos="0 0 0"><inertial pos="0 0 0" mass="1" diaginertia="0 0 0"/></body></body><body name="q" pos="2 0 1"><freejoint/><geom size="0.1" mass="-1"/></body></worldbody></mujoco>"#,
        Expect::Refused("mass, inertia or density are negative in geom"),
    ),
    // MuJoCo: "Error: unrecognized name 'nosuch' of sensorized object"
    (
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="p" pos="0 0 1"><joint name="h" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="0 0 0"/><site name="ps"/><body name="s" pos="0 0 0"><inertial pos="0 0 0" mass="1" diaginertia="0 0 0"/></body></body></worldbody><sensor><force site="nosuch"/></sensor></mujoco>"#,
        Expect::Refused("unknown site 'nosuch' (sensor #0)"),
    ),
    // MuJoCo: "Error: unrecognized name 's' of sensorized object"
    (
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="s" pos="0 0 0.5"><geom size="0.2"/></body><body name="f" pos="0 0 2"><freejoint/><geom size="0.1"/></body></worldbody><sensor><framepos objtype="xbody" objname="s"/></sensor></mujoco>"#,
        Expect::Refused("unknown body 's' (sensor #0)"),
    ),
    // MuJoCo: "Error: unrecognized name 's' of object"
    (
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="s" pos="0 0 0.5"><geom size="0.2"/></body><body name="f" pos="0 0 2"><freejoint/><geom size="0.1"/></body></worldbody><sensor><framepos objtype="body" objname="f" reftype="xbody" refname="s"/></sensor></mujoco>"#,
        Expect::Refused("unknown body 's' (sensor #0)"),
    ),
    // MuJoCo: "Error: unrecognized name 's' of sensorized object"
    (
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="s" pos="0 0 0.5"><geom size="0.2"/></body><body name="f" pos="0 0 2"><freejoint/><geom size="0.1"/></body></worldbody><sensor><user objtype="xbody" objname="s" dim="1"/></sensor></mujoco>"#,
        Expect::Refused("unknown body 's' (sensor #0)"),
    ),
    // MuJoCo: "Error: unknown target body in camera"
    (
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="t" pos="0 0 1"><geom size="0.1" contype="0" conaffinity="0"/></body><camera name="cam" mode="targetbody" target="nobody" pos="1 1 1"/><body name="f" pos="0 0 2"><freejoint/><geom size="0.1"/></body></worldbody></mujoco>"#,
        Expect::Refused("unknown target body in camera"),
    ),
    // MuJoCo: "Error: unknown target body in light"
    (
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="t" pos="0 0 1"><geom size="0.1" contype="0" conaffinity="0"/></body><light name="l" mode="targetbody" target="nobody" pos="1 1 1"/><body name="f" pos="0 0 2"><freejoint/><geom size="0.1"/></body></worldbody></mujoco>"#,
        Expect::Refused("unknown target body in light"),
    ),
    // MuJoCo: "Error: user has more values than nuser_body in body"
    (
        r#"<mujoco><compiler fusestatic="true"/><size nuser_body="1"/><worldbody><body name="p" pos="0 0 1"><joint type="hinge"/><geom size="0.1"/><body pos="0.3 0 0" user="1 2 3"><geom size="0.05"/></body></body></worldbody></mujoco>"#,
        Expect::Refused("user data exceeds nuser_body=1"),
    ),
    // MuJoCo: "Error: user has more values than nuser_body in body"
    (
        r#"<mujoco><compiler fusestatic="true"/><size nuser_body="1"/><worldbody><body name="p" pos="0 0 1" user="1 2"><joint name="h" type="hinge" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia="0 0 0"/><body name="s" pos="0 0 0"><inertial pos="0 0 0" mass="1" diaginertia="0 0 0"/></body></body></worldbody></mujoco>"#,
        Expect::Refused("user data exceeds nuser_body=1"),
    ),
];

/// Each fixture, fused, with MuJoCo 3.5.0's `qacc` unfused.
const FIXTURES: &[(&str, f64)] = &[
    // MuJoCo: qacc fused np.float64(29.130299346210602)
    (
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="p" pos="0 0 1"><joint name="h" type="hinge" axis="0 1 0"/><geom type="box" size="0.1 0.05 0.02"/><body name="s" pos="0.3 0.1 0" euler="20 30 40"><inertial pos="0.05 0.02 0.01" euler="10 0 50" mass="0.7" diaginertia="0.01 0.02 0.025"/></body></body></worldbody></mujoco>"#,
        24.798337967198677,
    ),
    // MuJoCo: qacc fused np.float64(11.477805739125529)
    (
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="p" pos="0 0 1"><joint name="h" type="hinge" axis="0 1 0"/><geom type="box" size="0.1 0.05 0.02"/><body name="s" pos="0.2 0 0.1" euler="0 30 45"><geom type="capsule" fromto="0 0 0 0.2 0.1 0" size="0.03"/></body></body></worldbody></mujoco>"#,
        34.977110889627234,
    ),
    // MuJoCo: qacc fused np.float64(16.96819809431316)
    (
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="p" pos="0 0 1"><joint name="h" type="hinge" axis="0 1 0"/><inertial pos="0.01 0 0" mass="1" diaginertia="0.01 0.012 0.014"/><body name="s" pos="0.3 0 0" euler="0 0 60"><geom type="box" pos="0.05 0 0" size="0.05 0.02 0.01"/></body></body></worldbody></mujoco>"#,
        17.162573705695586,
    ),
    // MuJoCo: qacc fused np.float64(27.965566161579975)
    (
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="arm" pos="0 0 1"><joint name="h" type="hinge" axis="0 1 0"/><inertial pos="0.2 0 0" mass="2" diaginertia="0.01 0.05 0.05"/><body name="tool" pos="0.4 0 0" euler="0 90 0"><inertial pos="0 0 0.05" euler="0 17.19 0" mass="0.5" diaginertia="0.001 0.002 0.0025"/></body></body></worldbody></mujoco>"#,
        26.286173633440516,
    ),
];

/// 0 on macOS arm64, where MuJoCo's values were printed, and 1e-12 of an
/// array's largest entry elsewhere, where the platform's libm gives other
/// last bits.
const TOLERANCE: f64 = if cfg!(all(target_os = "macos", target_arch = "aarch64")) {
    0.0
} else {
    1e-12
};

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
    // Equal bits pass, an infinity's among them; otherwise both entries are
    // finite and within the tolerance of the array's largest finite entry.
    let scale = mujoco
        .iter()
        .filter(|v| v.is_finite())
        .fold(0.0_f64, |m, v| m.max(v.abs()));
    ours.iter().zip(mujoco).all(|(a, b)| {
        a.to_bits() == b.to_bits()
            || (a.is_finite() && b.is_finite() && (a - b).abs() <= TOLERANCE * scale)
    })
}

fn flat3(v: &[nalgebra::Vector3<f64>]) -> Vec<f64> {
    v.iter().flat_map(|x| x.iter().copied()).collect()
}

fn flat4(v: &[nalgebra::UnitQuaternion<f64>]) -> Vec<f64> {
    v.iter().flat_map(|q| [q.w, q.i, q.j, q.k]).collect()
}

/// Each document's bodies, their poses and inertias, its geoms' and sites'
/// bodies and poses and its `qpos0`, as MuJoCo 3.5.0 fuses them; each
/// document MuJoCo refuses refused for its fault.
#[test]
fn each_document_fuses_as_mujoco() {
    let mut wrong = Vec::new();
    for (doc, expect) in CASES {
        match (load_model(doc), expect) {
            (Ok(m), Expect::Loads(want)) => {
                let names: Vec<&str> = m
                    .body_name
                    .iter()
                    .map(|n| n.as_deref().unwrap_or(""))
                    .collect();
                let body_geoms: Vec<(usize, usize, usize)> = (0..m.nbody)
                    .filter(|&b| m.body_geom_num[b] > 0)
                    .map(|b| (b, m.body_geom_adr[b], m.body_geom_num[b]))
                    .collect();
                let differ: Vec<&str> = [
                    ("body_name", names == want.body_name),
                    ("body_parent", m.body_parent == want.body_parent),
                    (
                        "body_pos",
                        close(&flat3(&m.body_pos), want.body_pos.as_flattened()),
                    ),
                    (
                        "body_quat",
                        close(&flat4(&m.body_quat), want.body_quat.as_flattened()),
                    ),
                    ("geom_body", m.geom_body == want.geom_body),
                    (
                        "geom_pos",
                        close(&flat3(&m.geom_pos), want.geom_pos.as_flattened()),
                    ),
                    (
                        "geom_quat",
                        close(&flat4(&m.geom_quat), want.geom_quat.as_flattened()),
                    ),
                    ("site_body", m.site_body == want.site_body),
                    (
                        "site_pos",
                        close(&flat3(&m.site_pos), want.site_pos.as_flattened()),
                    ),
                    (
                        "site_quat",
                        close(&flat4(&m.site_quat), want.site_quat.as_flattened()),
                    ),
                    ("body_mass", close(&m.body_mass, want.body_mass)),
                    (
                        "body_inertia",
                        close(&flat3(&m.body_inertia), want.body_inertia.as_flattened()),
                    ),
                    (
                        "body_ipos",
                        close(&flat3(&m.body_ipos), want.body_ipos.as_flattened()),
                    ),
                    (
                        "body_iquat",
                        close(&flat4(&m.body_iquat), want.body_iquat.as_flattened()),
                    ),
                    ("body_rootid", m.body_rootid == want.body_rootid),
                    ("body_geoms", body_geoms == want.body_geoms),
                    ("jnt_body", m.jnt_body == want.jnt_body),
                    ("dof_body", m.dof_body == want.dof_body),
                    ("qpos0", close(m.qpos0.as_slice(), want.qpos0)),
                    ("qpos_spring", close(&m.qpos_spring, want.qpos_spring)),
                    (
                        "eq_data",
                        close(m.eq_data.as_flattened(), want.eq_data.as_flattened()),
                    ),
                ]
                .into_iter()
                .filter(|&(_, same)| !same)
                .map(|(field, _)| field)
                .collect();
                if !differ.is_empty() {
                    wrong.push(format!("{differ:?} differ: {doc}"));
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

/// A fused model's dynamics are the unfused one's (book L42's test, registry
/// `D-FUSESTATIC-INERTIA`): `qacc` at the first `forward`, fused against the
/// same document unfused, within 1e-5 of the unfused, relative (the
/// principal axes' `eig3` leaves 7.3e-8 here); the unfused `qacc` is MuJoCo
/// 3.5.0's (bit for bit on macOS arm64, within 1e-12 relative elsewhere).
/// MuJoCo adds a fused child's inertia in the other frame order and is off by
/// 4.33, -23.5, -0.194 and 1.68 on these four (each document's comment
/// gives its fused `qacc`).
#[test]
fn a_fused_models_dynamics_are_the_unfused_ones() {
    for &(doc, mujoco) in FIXTURES {
        let unfused_doc = doc.replace(r#"<compiler fusestatic="true"/>"#, "");
        assert_ne!(unfused_doc, doc, "the fixture fuses: {doc}");
        let fused = load_model(doc).unwrap();
        let unfused = load_model(&unfused_doc).unwrap();
        assert!(fused.nbody < unfused.nbody, "a body fused: {doc}");
        let qacc = |m: &sim_core::Model| {
            let mut d = m.make_data();
            d.forward(m).unwrap();
            d.qacc[0]
        };
        let (f, u) = (qacc(&fused), qacc(&unfused));
        assert!(
            close(&[u], &[mujoco]),
            "unfused qacc {u}, MuJoCo's {mujoco}: {doc}"
        );
        assert!(
            (f - u).abs() <= 1e-5 * u.abs(),
            "fused qacc {f}, unfused {u}: {doc}"
        );
    }
}

/// A fused body's user data counts toward the automatic `nuser_body`, as
/// MuJoCo 3.5.0 sizes user data before it fuses (`SetNuser`,
/// `user_model.cc:4926`; measured: 3).
#[test]
fn a_fused_bodys_user_data_sizes_nuser_body() {
    let m = load_model(
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="p" pos="0 0 1"><joint type="hinge"/><geom size="0.1"/><body pos="0.3 0 0" user="1 2 3"><geom size="0.05"/></body></body></worldbody></mujoco>"#,
    )
    .unwrap();
    assert_eq!(
        (m.nbody, m.nuser_body),
        (2, 3),
        "the body fused, its user data counted"
    );
}

/// A frame sensor with no `objtype` naming a site, which MuJoCo refuses and
/// this crate reads as that site (registry `D-FRAME-SENSOR-OBJTYPE`), keeps
/// no body from fusion: the two named static bodies fuse, as they do for
/// the same sensor with `objtype="site"` (MuJoCo: nbody 2).
#[test]
fn a_frame_sensor_naming_a_site_keeps_no_body() {
    let doc = |objtype: &str| {
        format!(
            r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="s" pos="0 0 1"><geom size="0.1"/></body><body name="u" pos="1 0 1"><geom size="0.1"/></body><body name="f" pos="0 0 2"><freejoint/><geom size="0.1"/><site name="fs"/></body></worldbody><sensor><framepos{objtype} objname="fs"/></sensor></mujoco>"#
        )
    };
    let unwritten = load_model(&doc("")).unwrap();
    let site = load_model(&doc(r#" objtype="site""#)).unwrap();
    assert_eq!(
        (unwritten.nbody, site.nbody),
        (2, 2),
        "both static bodies fused"
    );
}

/// A static body holding a camera or a light is fused, which stops MuJoCo
/// 3.5.0 (SIGSEGV: `FuseStatic` deletes the body and its cameras and lights
/// with it, then compiles them, registry `D-FUSESTATIC-CAMERA`): this crate
/// makes no camera or light, and loads it. Each document is a `format!`
/// template, which the parity census does not extract: its oracle cannot
/// write a golden for a document that stops it.
#[test]
fn a_fused_bodys_camera_or_light_is_no_fault() {
    for element in ["camera", "light"] {
        let doc = format!(
            r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="s" pos="0 0 1"><geom size="0.1"/><{element} name="e" pos="0 0 1"/></body><body name="f" pos="0 0 2"><freejoint/><geom size="0.1"/></body></worldbody></mujoco>"#
        );
        let m = load_model(&doc).unwrap();
        assert_eq!(m.nbody, 2, "the body fused: {doc}");
    }
}
