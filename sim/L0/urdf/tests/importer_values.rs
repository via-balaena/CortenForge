//! A URDF's model as MuJoCo 3.5.0's own importer makes it (`xml_urdf.cc`):
//! every tensor its body's `fullinertia`, so a diagonal one takes its
//! principal axes as the inertial frame, the moments in decreasing order
//! (but for moments within 1e-12 of each other), and six
//! zeros a tensor not given; every origin's `rpy` read as extrinsic x-y-z,
//! and every `xyz` and `rpy` as written, however small; and no option set,
//! so energy is not computed. One difference is deliberate: a link's tensor
//! in a turned `<inertial>` origin is turned into the link's frame, R·I·Rᵀ,
//! as URDF defines the origin, where MuJoCo's import drops the turn
//! (D-URDF-INERTIAL-TURN): its principal moments are MuJoCo's, its inertial
//! frame turned.
//!
//! Every expected value was printed on the census's oracle (2026-11-25):
//! MuJoCo 3.5.0's URDF import's, and for a turned origin's frame its
//! `mju_euler2Quat` and `mju_mulQuat`.

// The values are MuJoCo's, printed to the last digit.
#![allow(
    clippy::unwrap_used,
    clippy::unreadable_literal,
    clippy::approx_constant
)]

use sim_urdf::load_urdf_model;

/// The fields of a converted URDF MuJoCo 3.5.0's import decides, and what
/// its first `forward` and its fifth step make of them.
struct Model {
    body_pos: &'static [[f64; 3]],
    /// (w, x, y, z), as each quaternion below.
    body_quat: &'static [[f64; 4]],
    body_mass: &'static [f64],
    body_inertia: &'static [[f64; 3]],
    body_ipos: &'static [[f64; 3]],
    body_iquat: &'static [[f64; 4]],
    /// MuJoCo's `body_simple` and `dof_simplenum`, which a body's inertial
    /// frame decides.
    body_simple: &'static [u8],
    dof_simplenum: &'static [usize],
    geom_pos: &'static [[f64; 3]],
    geom_quat: &'static [[f64; 4]],
    geom_size: &'static [[f64; 3]],
    jnt_axis: &'static [[f64; 3]],
    jnt_limited: &'static [bool],
    jnt_range: &'static [[f64; 2]],
    dof_damping: &'static [f64],
    dof_frictionloss: &'static [f64],
    qpos0: &'static [f64],
    meaninertia: f64,
    /// Each body's world pose and its inertial frame's position, and `qacc`,
    /// at the first `forward`.
    xpos: &'static [[f64; 3]],
    xquat: &'static [[f64; 4]],
    xipos: &'static [[f64; 3]],
    qacc: &'static [f64],
    /// `qpos` after five steps.
    qpos5: &'static [f64],
    /// The model's options: a URDF writes none, so MJCF's defaults.
    enableflags: u32,
    disableflags: u32,
    timestep: f64,
    gravity: [f64; 3],
}

/// What MuJoCo 3.5.0 makes of a document.
enum Expect {
    Loads(&'static Model),
    /// Refused for MuJoCo's fault, in this crate's words.
    Refused(&'static str),
}

const CASES: &[(&str, Expect)] = &[
    // an axis written unnormalised, on a link of an unsorted diagonal tensor
    (
        r#"<robot name="r"><link name="base"><inertial><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="arm"><inertial><mass value="1"/><inertia ixx="0.1" iyy="0.2" izz="0.3" ixy="0" ixz="0" iyz="0"/></inertial><collision><origin xyz="0.05 0 0"/><geometry><box size="0.1 0.2 0.3"/></geometry></collision></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5"/><axis xyz="0 0 2"/></joint></robot>"#,
        Expect::Loads(&Model {
            body_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            body_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            body_mass: &[0.0, 1.0],
            body_inertia: &[[0.0, 0.0, 0.0], [0.3, 0.2, 0.1]],
            body_ipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            body_iquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 0.7071067811865489, 0.0, 0.7071067811865489],
            ],
            body_simple: &[1, 0],
            dof_simplenum: &[0],
            geom_pos: &[[0.05, 0.0, 0.0]],
            geom_quat: &[[1.0, 0.0, 0.0, 0.0]],
            geom_size: &[[0.05, 0.1, 0.15]],
            jnt_axis: &[[0.0, 0.0, 1.0]],
            jnt_limited: &[false],
            jnt_range: &[[0.0, 0.0]],
            dof_damping: &[0.0],
            dof_frictionloss: &[0.0],
            qpos0: &[0.0],
            meaninertia: 0.30000000000000243,
            xpos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            xquat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            xipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            qacc: &[0.0],
            qpos5: &[0.0],
            enableflags: 0,
            disableflags: 0,
            timestep: 0.002,
            gravity: [0.0, 0.0, -9.81],
        }),
    ),
    // an axis not written: x
    (
        r#"<robot name="r"><link name="base"><inertial><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="arm"><inertial><mass value="1"/><inertia ixx="0.1" iyy="0.2" izz="0.3" ixy="0" ixz="0" iyz="0"/></inertial><collision><origin xyz="0.05 0 0"/><geometry><box size="0.1 0.2 0.3"/></geometry></collision></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5"/></joint></robot>"#,
        Expect::Loads(&Model {
            body_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            body_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            body_mass: &[0.0, 1.0],
            body_inertia: &[[0.0, 0.0, 0.0], [0.3, 0.2, 0.1]],
            body_ipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            body_iquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 0.7071067811865489, 0.0, 0.7071067811865489],
            ],
            body_simple: &[1, 0],
            dof_simplenum: &[0],
            geom_pos: &[[0.05, 0.0, 0.0]],
            geom_quat: &[[1.0, 0.0, 0.0, 0.0]],
            geom_size: &[[0.05, 0.1, 0.15]],
            jnt_axis: &[[1.0, 0.0, 0.0]],
            jnt_limited: &[false],
            jnt_range: &[[0.0, 0.0]],
            dof_damping: &[0.0],
            dof_frictionloss: &[0.0],
            qpos0: &[0.0],
            meaninertia: 0.10000000000000081,
            xpos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            xquat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            xipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            qacc: &[0.0],
            qpos5: &[0.0],
            enableflags: 0,
            disableflags: 0,
            timestep: 0.002,
            gravity: [0.0, 0.0, -9.81],
        }),
    ),
    // a full tensor (MuJoCo's representation at main too)
    (
        r#"<robot name="r"><link name="base"><inertial><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="arm"><inertial><mass value="1"/><inertia ixx="0.5" iyy="0.4" izz="0.3" ixy="0.05" ixz="0.02" iyz="0.01"/></inertial><collision><origin xyz="0.05 0 0"/><geometry><box size="0.1 0.2 0.3"/></geometry></collision></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5"/><axis xyz="0 0 1"/></joint></robot>"#,
        Expect::Loads(&Model {
            body_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            body_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            body_mass: &[0.0, 1.0],
            body_inertia: &[
                [0.0, 0.0, 0.0],
                [0.5229423155075864, 0.3793196536284876, 0.297738030863926],
            ],
            body_ipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            body_iquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [
                    0.9793196531499334,
                    0.019166380058604496,
                    -0.046996395245123626,
                    0.19584944642175348,
                ],
            ],
            body_simple: &[1, 0],
            dof_simplenum: &[0],
            geom_pos: &[[0.05, 0.0, 0.0]],
            geom_quat: &[[1.0, 0.0, 0.0, 0.0]],
            geom_size: &[[0.05, 0.1, 0.15]],
            jnt_axis: &[[0.0, 0.0, 1.0]],
            jnt_limited: &[false],
            jnt_range: &[[0.0, 0.0]],
            dof_damping: &[0.0],
            dof_frictionloss: &[0.0],
            qpos0: &[0.0],
            meaninertia: 0.29999999987250386,
            xpos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            xquat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            xipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            qacc: &[0.0],
            qpos5: &[0.0],
            enableflags: 0,
            disableflags: 0,
            timestep: 0.002,
            gravity: [0.0, 0.0, -9.81],
        }),
    ),
    // a diagonal tensor (0.1, 0.3, 0.2): its principal axes the inertial frame, the moments largest first
    (
        r#"<robot name="r"><link name="base"><inertial><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="arm"><inertial><mass value="1"/><inertia ixx="0.1" iyy="0.3" izz="0.2" ixy="0" ixz="0" iyz="0"/></inertial><collision><origin xyz="0.05 0 0"/><geometry><box size="0.1 0.2 0.3"/></geometry></collision></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5"/><axis xyz="0 0 1"/></joint></robot>"#,
        Expect::Loads(&Model {
            body_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            body_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            body_mass: &[0.0, 1.0],
            body_inertia: &[[0.0, 0.0, 0.0], [0.3, 0.2, 0.1]],
            body_ipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            body_iquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [
                    0.5000000000000007,
                    0.5000000000000007,
                    0.5000000000000007,
                    0.5000000000000007,
                ],
            ],
            body_simple: &[1, 0],
            dof_simplenum: &[0],
            geom_pos: &[[0.05, 0.0, 0.0]],
            geom_quat: &[[1.0, 0.0, 0.0, 0.0]],
            geom_size: &[[0.05, 0.1, 0.15]],
            jnt_axis: &[[0.0, 0.0, 1.0]],
            jnt_limited: &[false],
            jnt_range: &[[0.0, 0.0]],
            dof_damping: &[0.0],
            dof_frictionloss: &[0.0],
            qpos0: &[0.0],
            meaninertia: 0.20000000000000107,
            xpos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            xquat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            xipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            qacc: &[0.0],
            qpos5: &[0.0],
            enableflags: 0,
            disableflags: 0,
            timestep: 0.002,
            gravity: [0.0, 0.0, -9.81],
        }),
    ),
    // a diagonal tensor written largest first: its own frame
    (
        r#"<robot name="r"><link name="base"><inertial><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="arm"><inertial><mass value="1"/><inertia ixx="0.3" iyy="0.2" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial><collision><origin xyz="0.05 0 0"/><geometry><box size="0.1 0.2 0.3"/></geometry></collision></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5"/><axis xyz="0 0 1"/></joint></robot>"#,
        Expect::Loads(&Model {
            body_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            body_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            body_mass: &[0.0, 1.0],
            body_inertia: &[[0.0, 0.0, 0.0], [0.3, 0.2, 0.1]],
            body_ipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            body_iquat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            body_simple: &[1, 1],
            dof_simplenum: &[1],
            geom_pos: &[[0.05, 0.0, 0.0]],
            geom_quat: &[[1.0, 0.0, 0.0, 0.0]],
            geom_size: &[[0.05, 0.1, 0.15]],
            jnt_axis: &[[0.0, 0.0, 1.0]],
            jnt_limited: &[false],
            jnt_range: &[[0.0, 0.0]],
            dof_damping: &[0.0],
            dof_frictionloss: &[0.0],
            qpos0: &[0.0],
            meaninertia: 0.1,
            xpos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            xquat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            xipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            qacc: &[0.0],
            qpos5: &[0.0],
            enableflags: 0,
            disableflags: 0,
            timestep: 0.002,
            gravity: [0.0, 0.0, -9.81],
        }),
    ),
    // a diagonal tensor with two equal moments
    (
        r#"<robot name="r"><link name="base"><inertial><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="arm"><inertial><mass value="1"/><inertia ixx="0.1" iyy="0.2" izz="0.2" ixy="0" ixz="0" iyz="0"/></inertial><collision><origin xyz="0.05 0 0"/><geometry><box size="0.1 0.2 0.3"/></geometry></collision></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5"/><axis xyz="0 0 1"/></joint></robot>"#,
        Expect::Loads(&Model {
            body_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            body_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            body_mass: &[0.0, 1.0],
            body_inertia: &[[0.0, 0.0, 0.0], [0.2, 0.2, 0.1]],
            body_ipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            body_iquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [
                    0.5000000000000007,
                    0.5000000000000007,
                    0.5000000000000007,
                    0.5000000000000007,
                ],
            ],
            body_simple: &[1, 0],
            dof_simplenum: &[0],
            geom_pos: &[[0.05, 0.0, 0.0]],
            geom_quat: &[[1.0, 0.0, 0.0, 0.0]],
            geom_size: &[[0.05, 0.1, 0.15]],
            jnt_axis: &[[0.0, 0.0, 1.0]],
            jnt_limited: &[false],
            jnt_range: &[[0.0, 0.0]],
            dof_damping: &[0.0],
            dof_frictionloss: &[0.0],
            qpos0: &[0.0],
            meaninertia: 0.20000000000000107,
            xpos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            xquat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            xipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            qacc: &[0.0],
            qpos5: &[0.0],
            enableflags: 0,
            disableflags: 0,
            timestep: 0.002,
            gravity: [0.0, 0.0, -9.81],
        }),
    ),
    // a diagonal tensor at an offset origin
    (
        r#"<robot name="r"><link name="base"><inertial><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="arm"><inertial><origin xyz="0.1 -0.2 0.05"/><mass value="1"/><inertia ixx="0.1" iyy="0.2" izz="0.3" ixy="0" ixz="0" iyz="0"/></inertial><collision><origin xyz="0.05 0 0"/><geometry><box size="0.1 0.2 0.3"/></geometry></collision></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5"/><axis xyz="0 0 1"/></joint></robot>"#,
        Expect::Loads(&Model {
            body_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            body_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            body_mass: &[0.0, 1.0],
            body_inertia: &[[0.0, 0.0, 0.0], [0.3, 0.2, 0.1]],
            body_ipos: &[[0.0, 0.0, 0.0], [0.1, -0.2, 0.05]],
            body_iquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 0.7071067811865489, 0.0, 0.7071067811865489],
            ],
            body_simple: &[1, 0],
            dof_simplenum: &[0],
            geom_pos: &[[0.05, 0.0, 0.0]],
            geom_quat: &[[1.0, 0.0, 0.0, 0.0]],
            geom_size: &[[0.05, 0.1, 0.15]],
            jnt_axis: &[[0.0, 0.0, 1.0]],
            jnt_limited: &[false],
            jnt_range: &[[0.0, 0.0]],
            dof_damping: &[0.0],
            dof_frictionloss: &[0.0],
            qpos0: &[0.0],
            meaninertia: 0.3500000000000024,
            xpos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            xquat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            xipos: &[[0.0, 0.0, 0.0], [0.1, -0.2, 0.55]],
            qacc: &[0.0],
            qpos5: &[0.0],
            enableflags: 0,
            disableflags: 0,
            timestep: 0.002,
            gravity: [0.0, 0.0, -9.81],
        }),
    ),
    // an off-diagonal moment of 1e-11 between two equal moments, which turns the principal axes and which the converter dropped
    (
        r#"<robot name="r"><link name="base"><inertial><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="arm"><inertial><mass value="1"/><inertia ixx="0.2" iyy="0.2" izz="0.3" ixy="1e-11" ixz="0" iyz="0"/></inertial><collision><origin xyz="0.05 0 0"/><geometry><box size="0.1 0.2 0.3"/></geometry></collision></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5"/><axis xyz="0 0 1"/></joint></robot>"#,
        Expect::Loads(&Model {
            body_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            body_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            body_mass: &[0.0, 1.0],
            body_inertia: &[[0.0, 0.0, 0.0], [0.3, 0.20000000001, 0.19999999999000004]],
            body_ipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            body_iquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [
                    0.2705980500730991,
                    0.6532814824381896,
                    -0.2705980500730991,
                    0.6532814824381896,
                ],
            ],
            body_simple: &[1, 0],
            dof_simplenum: &[0],
            geom_pos: &[[0.05, 0.0, 0.0]],
            geom_quat: &[[1.0, 0.0, 0.0, 0.0]],
            geom_size: &[[0.05, 0.1, 0.15]],
            jnt_axis: &[[0.0, 0.0, 1.0]],
            jnt_limited: &[false],
            jnt_range: &[[0.0, 0.0]],
            dof_damping: &[0.0],
            dof_frictionloss: &[0.0],
            qpos0: &[0.0],
            meaninertia: 0.30000000000000254,
            xpos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            xquat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            xipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            qacc: &[0.0],
            qpos5: &[0.0],
            enableflags: 0,
            disableflags: 0,
            timestep: 0.002,
            gravity: [0.0, 0.0, -9.81],
        }),
    ),
    // a root link's diagonal tensor, fused into the world
    (
        r#"<robot name="r"><link name="base"><inertial><mass value="2"/><inertia ixx="0.1" iyy="0.3" izz="0.2" ixy="0" ixz="0" iyz="0"/></inertial></link></robot>"#,
        Expect::Loads(&Model {
            body_pos: &[[0.0, 0.0, 0.0]],
            body_quat: &[[1.0, 0.0, 0.0, 0.0]],
            body_mass: &[0.0],
            body_inertia: &[[0.0, 0.0, 0.0]],
            body_ipos: &[[0.0, 0.0, 0.0]],
            body_iquat: &[[1.0, 0.0, 0.0, 0.0]],
            body_simple: &[1],
            dof_simplenum: &[],
            geom_pos: &[],
            geom_quat: &[],
            geom_size: &[],
            jnt_axis: &[],
            jnt_limited: &[],
            jnt_range: &[],
            dof_damping: &[],
            dof_frictionloss: &[],
            qpos0: &[],
            meaninertia: 1.0,
            xpos: &[[0.0, 0.0, 0.0]],
            xquat: &[[1.0, 0.0, 0.0, 0.0]],
            xipos: &[[0.0, 0.0, 0.0]],
            qacc: &[],
            qpos5: &[],
            enableflags: 0,
            disableflags: 0,
            timestep: 0.002,
            gravity: [0.0, 0.0, -9.81],
        }),
    ),
    // a limit whose lower bound is above its upper: not limited
    (
        r#"<robot name="r"><link name="base"><inertial><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="arm"><inertial><mass value="1"/><inertia ixx="0.1" iyy="0.2" izz="0.3" ixy="0" ixz="0" iyz="0"/></inertial><collision><origin xyz="0.05 0 0"/><geometry><box size="0.1 0.2 0.3"/></geometry></collision></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5"/><axis xyz="0 0 1"/><limit lower="1" upper="-1"/></joint></robot>"#,
        Expect::Loads(&Model {
            body_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            body_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            body_mass: &[0.0, 1.0],
            body_inertia: &[[0.0, 0.0, 0.0], [0.3, 0.2, 0.1]],
            body_ipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            body_iquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 0.7071067811865489, 0.0, 0.7071067811865489],
            ],
            body_simple: &[1, 0],
            dof_simplenum: &[0],
            geom_pos: &[[0.05, 0.0, 0.0]],
            geom_quat: &[[1.0, 0.0, 0.0, 0.0]],
            geom_size: &[[0.05, 0.1, 0.15]],
            jnt_axis: &[[0.0, 0.0, 1.0]],
            jnt_limited: &[false],
            jnt_range: &[[1.0, -1.0]],
            dof_damping: &[0.0],
            dof_frictionloss: &[0.0],
            qpos0: &[0.0],
            meaninertia: 0.30000000000000243,
            xpos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            xquat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            xipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            qacc: &[0.0],
            qpos5: &[0.0],
            enableflags: 0,
            disableflags: 0,
            timestep: 0.002,
            gravity: [0.0, 0.0, -9.81],
        }),
    ),
    // a limit with a lower bound alone: not limited
    (
        r#"<robot name="r"><link name="base"><inertial><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="arm"><inertial><mass value="1"/><inertia ixx="0.1" iyy="0.2" izz="0.3" ixy="0" ixz="0" iyz="0"/></inertial><collision><origin xyz="0.05 0 0"/><geometry><box size="0.1 0.2 0.3"/></geometry></collision></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5"/><axis xyz="0 0 1"/><limit lower="-1"/></joint></robot>"#,
        Expect::Loads(&Model {
            body_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            body_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            body_mass: &[0.0, 1.0],
            body_inertia: &[[0.0, 0.0, 0.0], [0.3, 0.2, 0.1]],
            body_ipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            body_iquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 0.7071067811865489, 0.0, 0.7071067811865489],
            ],
            body_simple: &[1, 0],
            dof_simplenum: &[0],
            geom_pos: &[[0.05, 0.0, 0.0]],
            geom_quat: &[[1.0, 0.0, 0.0, 0.0]],
            geom_size: &[[0.05, 0.1, 0.15]],
            jnt_axis: &[[0.0, 0.0, 1.0]],
            jnt_limited: &[false],
            jnt_range: &[[-1.0, 0.0]],
            dof_damping: &[0.0],
            dof_frictionloss: &[0.0],
            qpos0: &[0.0],
            meaninertia: 0.30000000000000243,
            xpos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            xquat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            xipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            qacc: &[0.0],
            qpos5: &[0.0],
            enableflags: 0,
            disableflags: 0,
            timestep: 0.002,
            gravity: [0.0, 0.0, -9.81],
        }),
    ),
    // a limit with no bounds
    (
        r#"<robot name="r"><link name="base"><inertial><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="arm"><inertial><mass value="1"/><inertia ixx="0.1" iyy="0.2" izz="0.3" ixy="0" ixz="0" iyz="0"/></inertial><collision><origin xyz="0.05 0 0"/><geometry><box size="0.1 0.2 0.3"/></geometry></collision></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5"/><axis xyz="0 0 1"/><limit velocity="1" effort="2"/></joint></robot>"#,
        Expect::Loads(&Model {
            body_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            body_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            body_mass: &[0.0, 1.0],
            body_inertia: &[[0.0, 0.0, 0.0], [0.3, 0.2, 0.1]],
            body_ipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            body_iquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 0.7071067811865489, 0.0, 0.7071067811865489],
            ],
            body_simple: &[1, 0],
            dof_simplenum: &[0],
            geom_pos: &[[0.05, 0.0, 0.0]],
            geom_quat: &[[1.0, 0.0, 0.0, 0.0]],
            geom_size: &[[0.05, 0.1, 0.15]],
            jnt_axis: &[[0.0, 0.0, 1.0]],
            jnt_limited: &[false],
            jnt_range: &[[0.0, 0.0]],
            dof_damping: &[0.0],
            dof_frictionloss: &[0.0],
            qpos0: &[0.0],
            meaninertia: 0.30000000000000243,
            xpos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            xquat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            xipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            qacc: &[0.0],
            qpos5: &[0.0],
            enableflags: 0,
            disableflags: 0,
            timestep: 0.002,
            gravity: [0.0, 0.0, -9.81],
        }),
    ),
    // a prismatic joint's limit
    (
        r#"<robot name="r"><link name="base"><inertial><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="arm"><inertial><mass value="1"/><inertia ixx="0.1" iyy="0.2" izz="0.3" ixy="0" ixz="0" iyz="0"/></inertial><collision><origin xyz="0.05 0 0"/><geometry><box size="0.1 0.2 0.3"/></geometry></collision></link><joint name="j" type="prismatic"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5"/><axis xyz="1 0 0"/><limit lower="-0.1" upper="0.2"/></joint></robot>"#,
        Expect::Loads(&Model {
            body_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            body_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            body_mass: &[0.0, 1.0],
            body_inertia: &[[0.0, 0.0, 0.0], [0.3, 0.2, 0.1]],
            body_ipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            body_iquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 0.7071067811865489, 0.0, 0.7071067811865489],
            ],
            body_simple: &[1, 0],
            dof_simplenum: &[0],
            geom_pos: &[[0.05, 0.0, 0.0]],
            geom_quat: &[[1.0, 0.0, 0.0, 0.0]],
            geom_size: &[[0.05, 0.1, 0.15]],
            jnt_axis: &[[1.0, 0.0, 0.0]],
            jnt_limited: &[true],
            jnt_range: &[[-0.1, 0.2]],
            dof_damping: &[0.0],
            dof_frictionloss: &[0.0],
            qpos0: &[0.0],
            meaninertia: 1.0,
            xpos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            xquat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            xipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            qacc: &[0.0],
            qpos5: &[0.0],
            enableflags: 0,
            disableflags: 0,
            timestep: 0.002,
            gravity: [0.0, 0.0, -9.81],
        }),
    ),
    // a joint's damping and friction
    (
        r#"<robot name="r"><link name="base"><inertial><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="arm"><inertial><mass value="1"/><inertia ixx="0.1" iyy="0.2" izz="0.3" ixy="0" ixz="0" iyz="0"/></inertial><collision><origin xyz="0.05 0 0"/><geometry><box size="0.1 0.2 0.3"/></geometry></collision></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5"/><axis xyz="0 0 1"/><dynamics damping="0.3" friction="0.1"/></joint></robot>"#,
        Expect::Loads(&Model {
            body_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            body_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            body_mass: &[0.0, 1.0],
            body_inertia: &[[0.0, 0.0, 0.0], [0.3, 0.2, 0.1]],
            body_ipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            body_iquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 0.7071067811865489, 0.0, 0.7071067811865489],
            ],
            body_simple: &[1, 0],
            dof_simplenum: &[0],
            geom_pos: &[[0.05, 0.0, 0.0]],
            geom_quat: &[[1.0, 0.0, 0.0, 0.0]],
            geom_size: &[[0.05, 0.1, 0.15]],
            jnt_axis: &[[0.0, 0.0, 1.0]],
            jnt_limited: &[false],
            jnt_range: &[[0.0, 0.0]],
            dof_damping: &[0.3],
            dof_frictionloss: &[0.1],
            qpos0: &[0.0],
            meaninertia: 0.30000000000000243,
            xpos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            xquat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            xipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            qacc: &[0.0],
            qpos5: &[0.0],
            enableflags: 0,
            disableflags: 0,
            timestep: 0.002,
            gravity: [0.0, 0.0, -9.81],
        }),
    ),
    // a planar joint's damping and friction on its three joints
    (
        r#"<robot name="r"><link name="base"><inertial><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="arm"><inertial><mass value="1"/><inertia ixx="0.1" iyy="0.2" izz="0.3" ixy="0" ixz="0" iyz="0"/></inertial><collision><origin xyz="0.05 0 0"/><geometry><box size="0.1 0.2 0.3"/></geometry></collision></link><joint name="j" type="planar"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5"/><axis xyz="0 0 1"/><dynamics damping="0.3" friction="0.1"/></joint></robot>"#,
        Expect::Loads(&Model {
            body_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            body_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            body_mass: &[0.0, 1.0],
            body_inertia: &[[0.0, 0.0, 0.0], [0.3, 0.2, 0.1]],
            body_ipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            body_iquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 0.7071067811865489, 0.0, 0.7071067811865489],
            ],
            body_simple: &[1, 0],
            dof_simplenum: &[0, 0, 0],
            geom_pos: &[[0.05, 0.0, 0.0]],
            geom_quat: &[[1.0, 0.0, 0.0, 0.0]],
            geom_size: &[[0.05, 0.1, 0.15]],
            jnt_axis: &[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            jnt_limited: &[false, false, false],
            jnt_range: &[[0.0, 0.0], [0.0, 0.0], [0.0, 0.0]],
            dof_damping: &[0.3, 0.3, 0.3],
            dof_frictionloss: &[0.1, 0.1, 0.1],
            qpos0: &[0.0, 0.0, 0.0],
            meaninertia: 0.7666666666666675,
            xpos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            xquat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            xipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            qacc: &[0.0, 0.0, 0.0],
            qpos5: &[0.0, 0.0, 0.0],
            enableflags: 0,
            disableflags: 0,
            timestep: 0.002,
            gravity: [0.0, 0.0, -9.81],
        }),
    ),
    // a limit of equal bounds
    (
        r#"<robot name="r"><link name="base"><inertial><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="arm"><inertial><mass value="1"/><inertia ixx="0.1" iyy="0.2" izz="0.3" ixy="0" ixz="0" iyz="0"/></inertial><collision><origin xyz="0.05 0 0"/><geometry><box size="0.1 0.2 0.3"/></geometry></collision></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5"/><axis xyz="0 0 1"/><limit lower="0.5" upper="0.5"/></joint></robot>"#,
        Expect::Loads(&Model {
            body_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            body_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            body_mass: &[0.0, 1.0],
            body_inertia: &[[0.0, 0.0, 0.0], [0.3, 0.2, 0.1]],
            body_ipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            body_iquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 0.7071067811865489, 0.0, 0.7071067811865489],
            ],
            body_simple: &[1, 0],
            dof_simplenum: &[0],
            geom_pos: &[[0.05, 0.0, 0.0]],
            geom_quat: &[[1.0, 0.0, 0.0, 0.0]],
            geom_size: &[[0.05, 0.1, 0.15]],
            jnt_axis: &[[0.0, 0.0, 1.0]],
            jnt_limited: &[false],
            jnt_range: &[[0.5, 0.5]],
            dof_damping: &[0.0],
            dof_frictionloss: &[0.0],
            qpos0: &[0.0],
            meaninertia: 0.30000000000000243,
            xpos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            xquat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            xipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            qacc: &[0.0],
            qpos5: &[0.0],
            enableflags: 0,
            disableflags: 0,
            timestep: 0.002,
            gravity: [0.0, 0.0, -9.81],
        }),
    ),
    // a fixed link of mass 0 and six zero moments: a tensor not given, fused
    (
        r#"<robot name="r"><link name="base"><inertial><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="t"><inertial><mass value="0"/><inertia ixx="0" iyy="0" izz="0" ixy="0" ixz="0" iyz="0"/></inertial></link><joint name="f" type="fixed"><parent link="base"/><child link="t"/><origin xyz="0 0 0.5"/></joint></robot>"#,
        Expect::Loads(&Model {
            body_pos: &[[0.0, 0.0, 0.0]],
            body_quat: &[[1.0, 0.0, 0.0, 0.0]],
            body_mass: &[0.0],
            body_inertia: &[[0.0, 0.0, 0.0]],
            body_ipos: &[[0.0, 0.0, 0.0]],
            body_iquat: &[[1.0, 0.0, 0.0, 0.0]],
            body_simple: &[1],
            dof_simplenum: &[],
            geom_pos: &[],
            geom_quat: &[],
            geom_size: &[],
            jnt_axis: &[],
            jnt_limited: &[],
            jnt_range: &[],
            dof_damping: &[],
            dof_frictionloss: &[],
            qpos0: &[],
            meaninertia: 1.0,
            xpos: &[[0.0, 0.0, 0.0]],
            xquat: &[[1.0, 0.0, 0.0, 0.0]],
            xipos: &[[0.0, 0.0, 0.0]],
            qacc: &[],
            qpos5: &[],
            enableflags: 0,
            disableflags: 0,
            timestep: 0.002,
            gravity: [0.0, 0.0, -9.81],
        }),
    ),
    // a root link of mass 0
    (
        r#"<robot name="r"><link name="base"><inertial><mass value="0"/><inertia ixx="0" iyy="0" izz="0" ixy="0" ixz="0" iyz="0"/></inertial></link></robot>"#,
        Expect::Loads(&Model {
            body_pos: &[[0.0, 0.0, 0.0]],
            body_quat: &[[1.0, 0.0, 0.0, 0.0]],
            body_mass: &[0.0],
            body_inertia: &[[0.0, 0.0, 0.0]],
            body_ipos: &[[0.0, 0.0, 0.0]],
            body_iquat: &[[1.0, 0.0, 0.0, 0.0]],
            body_simple: &[1],
            dof_simplenum: &[],
            geom_pos: &[],
            geom_quat: &[],
            geom_size: &[],
            jnt_axis: &[],
            jnt_limited: &[],
            jnt_range: &[],
            dof_damping: &[],
            dof_frictionloss: &[],
            qpos0: &[],
            meaninertia: 1.0,
            xpos: &[[0.0, 0.0, 0.0]],
            xquat: &[[1.0, 0.0, 0.0, 0.0]],
            xipos: &[[0.0, 0.0, 0.0]],
            qacc: &[],
            qpos5: &[],
            enableflags: 0,
            disableflags: 0,
            timestep: 0.002,
            gravity: [0.0, 0.0, -9.81],
        }),
    ),
    // a link with no inertial: its geom's mass and inertia
    (
        r#"<robot name="r"><link name="base"><inertial><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="arm"><!-- none --><collision><origin xyz="0.05 0 0"/><geometry><box size="0.1 0.2 0.3"/></geometry></collision></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5"/><axis xyz="0 0 1"/></joint></robot>"#,
        Expect::Loads(&Model {
            body_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            body_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            body_mass: &[0.0, 6.000000000000001],
            body_inertia: &[
                [0.0, 0.0, 0.0],
                [
                    0.06500000000000002,
                    0.05000000000000001,
                    0.02500000000000001,
                ],
            ],
            body_ipos: &[[0.0, 0.0, 0.0], [0.05, 0.0, 0.0]],
            body_iquat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            body_simple: &[1, 0],
            dof_simplenum: &[0],
            geom_pos: &[[0.05, 0.0, 0.0]],
            geom_quat: &[[1.0, 0.0, 0.0, 0.0]],
            geom_size: &[[0.05, 0.1, 0.15]],
            jnt_axis: &[[0.0, 0.0, 1.0]],
            jnt_limited: &[false],
            jnt_range: &[[0.0, 0.0]],
            dof_damping: &[0.0],
            dof_frictionloss: &[0.0],
            qpos0: &[0.0],
            meaninertia: 0.040000000000000015,
            xpos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            xquat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            xipos: &[[0.0, 0.0, 0.0], [0.05, 0.0, 0.5]],
            qacc: &[0.0],
            qpos5: &[0.0],
            enableflags: 0,
            disableflags: 0,
            timestep: 0.002,
            gravity: [0.0, 0.0, -9.81],
        }),
    ),
    // a fixed link of mass 0 and positive moments
    (
        r#"<robot name="r"><link name="base"><inertial><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="t"><inertial><mass value="0"/><inertia ixx="0.1" iyy="0.2" izz="0.3" ixy="0" ixz="0" iyz="0"/></inertial></link><joint name="f" type="fixed"><parent link="base"/><child link="t"/><origin xyz="0 0 0.5"/></joint></robot>"#,
        Expect::Loads(&Model {
            body_pos: &[[0.0, 0.0, 0.0]],
            body_quat: &[[1.0, 0.0, 0.0, 0.0]],
            body_mass: &[0.0],
            body_inertia: &[[0.0, 0.0, 0.0]],
            body_ipos: &[[0.0, 0.0, 0.0]],
            body_iquat: &[[1.0, 0.0, 0.0, 0.0]],
            body_simple: &[1],
            dof_simplenum: &[],
            geom_pos: &[],
            geom_quat: &[],
            geom_size: &[],
            jnt_axis: &[],
            jnt_limited: &[],
            jnt_range: &[],
            dof_damping: &[],
            dof_frictionloss: &[],
            qpos0: &[],
            meaninertia: 1.0,
            xpos: &[[0.0, 0.0, 0.0]],
            xquat: &[[1.0, 0.0, 0.0, 0.0]],
            xipos: &[[0.0, 0.0, 0.0]],
            qacc: &[],
            qpos5: &[],
            enableflags: 0,
            disableflags: 0,
            timestep: 0.002,
            gravity: [0.0, 0.0, -9.81],
        }),
    ),
    // a fixed link fused into the root
    (
        r#"<robot name="r"><link name="base"><inertial><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="tool"><inertial><origin xyz="0 0 0.1"/><mass value="0.5"/><inertia ixx="0.001" iyy="0.002" izz="0.003" ixy="0" ixz="0" iyz="0"/></inertial><collision><geometry><box size="0.1 0.2 0.3"/></geometry></collision></link><joint name="f" type="fixed"><parent link="base"/><child link="tool"/><origin xyz="0.2 0 0" rpy="0 0.6 0"/></joint></robot>"#,
        Expect::Loads(&Model {
            body_pos: &[[0.0, 0.0, 0.0]],
            body_quat: &[[1.0, 0.0, 0.0, 0.0]],
            body_mass: &[0.0],
            body_inertia: &[[0.0, 0.0, 0.0]],
            body_ipos: &[[0.0, 0.0, 0.0]],
            body_iquat: &[[1.0, 0.0, 0.0, 0.0]],
            body_simple: &[1],
            dof_simplenum: &[],
            geom_pos: &[[0.2, 0.0, 0.0]],
            geom_quat: &[[0.955336489125606, 0.0, 0.29552020666133955, 0.0]],
            geom_size: &[[0.05, 0.1, 0.15]],
            jnt_axis: &[],
            jnt_limited: &[],
            jnt_range: &[],
            dof_damping: &[],
            dof_frictionloss: &[],
            qpos0: &[],
            meaninertia: 1.0,
            xpos: &[[0.0, 0.0, 0.0]],
            xquat: &[[1.0, 0.0, 0.0, 0.0]],
            xipos: &[[0.0, 0.0, 0.0]],
            qacc: &[],
            qpos5: &[],
            enableflags: 0,
            disableflags: 0,
            timestep: 0.002,
            gravity: [0.0, 0.0, -9.81],
        }),
    ),
    // a fixed link's mass and inertia on a moving link (review RW, RL-10b)
    (
        r#"<robot name="r"><link name="base"><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="1" iyy="1" izz="1" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="arm"/><link name="tool"><inertial><origin xyz="0 0 0"/><mass value="2"/><inertia ixx="0.01" iyy="0.01" izz="0.01" ixy="0" ixz="0" iyz="0"/></inertial></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 1"/><axis xyz="0 1 0"/><limit lower="-1" upper="1" effort="1" velocity="1"/></joint><joint name="f" type="fixed"><parent link="arm"/><child link="tool"/><origin xyz="0.2 0 0"/></joint></robot>"#,
        Expect::Loads(&Model {
            body_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 1.0]],
            body_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            body_mass: &[0.0, 2.0],
            body_inertia: &[[0.0, 0.0, 0.0], [0.01, 0.01, 0.01]],
            body_ipos: &[[0.0, 0.0, 0.0], [0.2, 0.0, 0.0]],
            body_iquat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            body_simple: &[1, 0],
            dof_simplenum: &[0],
            geom_pos: &[],
            geom_quat: &[],
            geom_size: &[],
            jnt_axis: &[[0.0, 1.0, 0.0]],
            jnt_limited: &[true],
            jnt_range: &[[-1.0, 1.0]],
            dof_damping: &[0.0],
            dof_frictionloss: &[0.0],
            qpos0: &[0.0],
            meaninertia: 0.09000000000000001,
            xpos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 1.0]],
            xquat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            xipos: &[[0.0, 0.0, 0.0], [0.2, 0.0, 1.0]],
            qacc: &[43.599999999999994],
            qpos5: &[0.0026159994615996066],
            enableflags: 0,
            disableflags: 0,
            timestep: 0.002,
            gravity: [0.0, 0.0, -9.81],
        }),
    ),
    // a joint origin's rpy about three axes
    (
        r#"<robot name="r"><link name="base"><inertial><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="arm"><inertial><mass value="1"/><inertia ixx="0.1" iyy="0.2" izz="0.3" ixy="0" ixz="0" iyz="0"/></inertial><collision><origin xyz="0.05 0 0"/><geometry><box size="0.1 0.2 0.3"/></geometry></collision></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5" rpy="0.2 -0.6 1.1"/><axis xyz="0 0 1"/></joint></robot>"#,
        Expect::Loads(&Model {
            body_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            body_quat: &[
                [1.0, 0.0, 0.0, 0.0],
                [
                    0.7949582047263282,
                    0.23500206304840215,
                    -0.20082854509626413,
                    0.5219994047657581,
                ],
            ],
            body_mass: &[0.0, 1.0],
            body_inertia: &[[0.0, 0.0, 0.0], [0.3, 0.2, 0.1]],
            body_ipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            body_iquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 0.7071067811865489, 0.0, 0.7071067811865489],
            ],
            body_simple: &[1, 0],
            dof_simplenum: &[0],
            geom_pos: &[[0.05, 0.0, 0.0]],
            geom_quat: &[[1.0, 0.0, 0.0, 0.0]],
            geom_size: &[[0.05, 0.1, 0.15]],
            jnt_axis: &[[0.0, 0.0, 1.0]],
            jnt_limited: &[false],
            jnt_range: &[[0.0, 0.0]],
            dof_damping: &[0.0],
            dof_frictionloss: &[0.0],
            qpos0: &[0.0],
            meaninertia: 0.3000000000000022,
            xpos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            xquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [
                    0.7949582047263282,
                    0.23500206304840215,
                    -0.20082854509626413,
                    0.5219994047657581,
                ],
            ],
            xipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            qacc: &[0.0],
            qpos5: &[0.0],
            enableflags: 0,
            disableflags: 0,
            timestep: 0.002,
            gravity: [0.0, 0.0, -9.81],
        }),
    ),
    // a collision origin's rpy about three axes
    (
        r#"<robot name="r"><link name="base"><inertial><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="arm"><inertial><mass value="1"/><inertia ixx="0.1" iyy="0.2" izz="0.3" ixy="0" ixz="0" iyz="0"/></inertial><collision><origin xyz="0.05 0 0" rpy="0.4 0.3 -0.9"/><geometry><box size="0.1 0.2 0.3"/></geometry></collision></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5"/><axis xyz="0 0 1"/></joint></robot>"#,
        Expect::Loads(&Model {
            body_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            body_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            body_mass: &[0.0, 1.0],
            body_inertia: &[[0.0, 0.0, 0.0], [0.3, 0.2, 0.1]],
            body_ipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            body_iquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 0.7071067811865489, 0.0, 0.7071067811865489],
            ],
            body_simple: &[1, 0],
            dof_simplenum: &[0],
            geom_pos: &[[0.05, 0.0, 0.0]],
            geom_quat: &[[
                0.8596750142879865,
                0.24058722359164875,
                0.04643489747844233,
                -0.4482415174318178,
            ]],
            geom_size: &[[0.05, 0.1, 0.15]],
            jnt_axis: &[[0.0, 0.0, 1.0]],
            jnt_limited: &[false],
            jnt_range: &[[0.0, 0.0]],
            dof_damping: &[0.0],
            dof_frictionloss: &[0.0],
            qpos0: &[0.0],
            meaninertia: 0.30000000000000243,
            xpos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            xquat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            xipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            qacc: &[0.0],
            qpos5: &[0.0],
            enableflags: 0,
            disableflags: 0,
            timestep: 0.002,
            gravity: [0.0, 0.0, -9.81],
        }),
    ),
    // an rpy about one axis, the same in either order
    (
        r#"<robot name="r"><link name="base"><inertial><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="arm"><inertial><mass value="1"/><inertia ixx="0.1" iyy="0.2" izz="0.3" ixy="0" ixz="0" iyz="0"/></inertial><collision><origin xyz="0.05 0 0" rpy="0.3 0 0"/><geometry><box size="0.1 0.2 0.3"/></geometry></collision></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5" rpy="0 0 0.7"/><axis xyz="0 0 1"/></joint></robot>"#,
        Expect::Loads(&Model {
            body_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            body_quat: &[
                [1.0, 0.0, 0.0, 0.0],
                [0.9393727128473789, 0.0, 0.0, 0.34289780745545134],
            ],
            body_mass: &[0.0, 1.0],
            body_inertia: &[[0.0, 0.0, 0.0], [0.3, 0.2, 0.1]],
            body_ipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            body_iquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 0.7071067811865489, 0.0, 0.7071067811865489],
            ],
            body_simple: &[1, 0],
            dof_simplenum: &[0],
            geom_pos: &[[0.05, 0.0, 0.0]],
            geom_quat: &[[0.9887710779360422, 0.14943813247359922, 0.0, 0.0]],
            geom_size: &[[0.05, 0.1, 0.15]],
            jnt_axis: &[[0.0, 0.0, 1.0]],
            jnt_limited: &[false],
            jnt_range: &[[0.0, 0.0]],
            dof_damping: &[0.0],
            dof_frictionloss: &[0.0],
            qpos0: &[0.0],
            meaninertia: 0.3000000000000021,
            xpos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            xquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [0.9393727128473789, 0.0, 0.0, 0.34289780745545134],
            ],
            xipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            qacc: &[0.0],
            qpos5: &[0.0],
            enableflags: 0,
            disableflags: 0,
            timestep: 0.002,
            gravity: [0.0, 0.0, -9.81],
        }),
    ),
    // an rpy about two axes
    (
        r#"<robot name="r"><link name="base"><inertial><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="arm"><inertial><mass value="1"/><inertia ixx="0.1" iyy="0.2" izz="0.3" ixy="0" ixz="0" iyz="0"/></inertial><collision><origin xyz="0.05 0 0"/><geometry><box size="0.1 0.2 0.3"/></geometry></collision></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5" rpy="0.5 0.5 0"/><axis xyz="0 0 1"/></joint></robot>"#,
        Expect::Loads(&Model {
            body_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            body_quat: &[
                [1.0, 0.0, 0.0, 0.0],
                [
                    0.9387912809451863,
                    0.2397127693021015,
                    0.2397127693021015,
                    -0.06120871905481365,
                ],
            ],
            body_mass: &[0.0, 1.0],
            body_inertia: &[[0.0, 0.0, 0.0], [0.3, 0.2, 0.1]],
            body_ipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            body_iquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 0.7071067811865489, 0.0, 0.7071067811865489],
            ],
            body_simple: &[1, 0],
            dof_simplenum: &[0],
            geom_pos: &[[0.05, 0.0, 0.0]],
            geom_quat: &[[1.0, 0.0, 0.0, 0.0]],
            geom_size: &[[0.05, 0.1, 0.15]],
            jnt_axis: &[[0.0, 0.0, 1.0]],
            jnt_limited: &[false],
            jnt_range: &[[0.0, 0.0]],
            dof_damping: &[0.0],
            dof_frictionloss: &[0.0],
            qpos0: &[0.0],
            meaninertia: 0.30000000000000226,
            xpos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            xquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [
                    0.9387912809451863,
                    0.2397127693021015,
                    0.2397127693021015,
                    -0.06120871905481365,
                ],
            ],
            xipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            qacc: &[0.0],
            qpos5: &[0.0],
            enableflags: 0,
            disableflags: 0,
            timestep: 0.002,
            gravity: [0.0, 0.0, -9.81],
        }),
    ),
    // angles past pi
    (
        r#"<robot name="r"><link name="base"><inertial><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="arm"><inertial><mass value="1"/><inertia ixx="0.1" iyy="0.2" izz="0.3" ixy="0" ixz="0" iyz="0"/></inertial><collision><origin xyz="0.05 0 0" rpy="-2.0 1.5 2.5"/><geometry><box size="0.1 0.2 0.3"/></geometry></collision></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5" rpy="3.0 -2.5 4.0"/><axis xyz="0 0 1"/></joint></robot>"#,
        Expect::Loads(&Model {
            body_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            body_quat: &[
                [1.0, 0.0, 0.0, 0.0],
                [
                    -0.870029837342751,
                    -0.06985190750636554,
                    0.3139388905062799,
                    -0.37364577634272067,
                ],
            ],
            body_mass: &[0.0, 1.0],
            body_inertia: &[[0.0, 0.0, 0.0], [0.3, 0.2, 0.1]],
            body_ipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            body_iquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 0.7071067811865489, 0.0, 0.7071067811865489],
            ],
            body_simple: &[1, 0],
            dof_simplenum: &[0],
            geom_pos: &[[0.05, 0.0, 0.0]],
            geom_quat: &[[
                -0.41966048228262914,
                -0.5436448756876524,
                -0.46815465445583565,
                0.5560274707783827,
            ]],
            geom_size: &[[0.05, 0.1, 0.15]],
            jnt_axis: &[[0.0, 0.0, 1.0]],
            jnt_limited: &[false],
            jnt_range: &[[0.0, 0.0]],
            dof_damping: &[0.0],
            dof_frictionloss: &[0.0],
            qpos0: &[0.0],
            meaninertia: 0.3000000000000023,
            xpos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            xquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [
                    -0.870029837342751,
                    -0.06985190750636554,
                    0.3139388905062799,
                    -0.37364577634272067,
                ],
            ],
            xipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            qacc: &[0.0],
            qpos5: &[0.0],
            enableflags: 0,
            disableflags: 0,
            timestep: 0.002,
            gravity: [0.0, 0.0, -9.81],
        }),
    ),
    // a floating joint's origin: its qpos0
    (
        r#"<robot name="r"><link name="base"><inertial><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="arm"><inertial><mass value="1"/><inertia ixx="0.1" iyy="0.2" izz="0.3" ixy="0" ixz="0" iyz="0"/></inertial><collision><origin xyz="0.05 0 0"/><geometry><box size="0.1 0.2 0.3"/></geometry></collision></link><joint name="j" type="floating"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5" rpy="0.3 0.2 0.1"/></joint></robot>"#,
        Expect::Loads(&Model {
            body_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            body_quat: &[
                [1.0, 0.0, 0.0, 0.0],
                [
                    0.9833474432563559,
                    0.14357217502739192,
                    0.10602051106179562,
                    0.03427079855048211,
                ],
            ],
            body_mass: &[0.0, 1.0],
            body_inertia: &[[0.0, 0.0, 0.0], [0.3, 0.2, 0.1]],
            body_ipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            body_iquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 0.7071067811865489, 0.0, 0.7071067811865489],
            ],
            body_simple: &[1, 0],
            dof_simplenum: &[0, 0, 0, 0, 0, 0],
            geom_pos: &[[0.05, 0.0, 0.0]],
            geom_quat: &[[1.0, 0.0, 0.0, 0.0]],
            geom_size: &[[0.05, 0.1, 0.15]],
            jnt_axis: &[[0.0, 0.0, 1.0]],
            jnt_limited: &[false],
            jnt_range: &[[0.0, 0.0]],
            dof_damping: &[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            dof_frictionloss: &[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            qpos0: &[
                0.0,
                0.0,
                0.5,
                0.9833474432563559,
                0.14357217502739192,
                0.10602051106179562,
                0.03427079855048211,
            ],
            meaninertia: 0.6000000000000009,
            xpos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            xquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [
                    0.9833474432563559,
                    0.14357217502739192,
                    0.10602051106179562,
                    0.03427079855048211,
                ],
            ],
            xipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            qacc: &[0.0, 0.0, -9.81, 0.0, 0.0, 0.0],
            qpos5: &[
                0.0,
                0.0,
                0.49941140000000006,
                0.9833474432563559,
                0.14357217502739192,
                0.10602051106179562,
                0.03427079855048211,
            ],
            enableflags: 0,
            disableflags: 0,
            timestep: 0.002,
            gravity: [0.0, 0.0, -9.81],
        }),
    ),
    // a planar joint in a turned origin
    (
        r#"<robot name="r"><link name="base"><inertial><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="arm"><inertial><mass value="1"/><inertia ixx="0.1" iyy="0.2" izz="0.3" ixy="0" ixz="0" iyz="0"/></inertial><collision><origin xyz="0.05 0 0"/><geometry><box size="0.1 0.2 0.3"/></geometry></collision></link><joint name="j" type="planar"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5" rpy="0.1 0.2 0.3"/><axis xyz="0 1 1"/></joint></robot>"#,
        Expect::Loads(&Model {
            body_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            body_quat: &[
                [1.0, 0.0, 0.0, 0.0],
                [
                    0.9833474432563558,
                    0.034270798550482096,
                    0.10602051106179562,
                    0.1435721750273919,
                ],
            ],
            body_mass: &[0.0, 1.0],
            body_inertia: &[[0.0, 0.0, 0.0], [0.3, 0.2, 0.1]],
            body_ipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            body_iquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 0.7071067811865489, 0.0, 0.7071067811865489],
            ],
            body_simple: &[1, 0],
            dof_simplenum: &[0, 0, 0],
            geom_pos: &[[0.05, 0.0, 0.0]],
            geom_quat: &[[1.0, 0.0, 0.0, 0.0]],
            geom_size: &[[0.05, 0.1, 0.15]],
            jnt_axis: &[
                [1.0, 0.0, -0.0],
                [-0.0, 0.7071067811865475, -0.7071067811865476],
                [0.0, 0.7071067811865475, 0.7071067811865475],
            ],
            jnt_limited: &[false, false, false],
            jnt_range: &[[0.0, 0.0], [0.0, 0.0], [0.0, 0.0]],
            dof_damping: &[0.0, 0.0, 0.0],
            dof_frictionloss: &[0.0, 0.0, 0.0],
            qpos0: &[0.0, 0.0, 0.0],
            meaninertia: 0.7500000000000006,
            xpos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            xquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [
                    0.9833474432563558,
                    0.034270798550482096,
                    0.10602051106179562,
                    0.1435721750273919,
                ],
            ],
            xipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            qacc: &[1.9489461350995507, 6.0857691043398905, 0.0],
            qpos5: &[0.00011693676810597305, 0.00036514614626039345, 0.0],
            enableflags: 0,
            disableflags: 0,
            timestep: 0.002,
            gravity: [0.0, 0.0, -9.81],
        }),
    ),
    // a fixed link's turned origin, its geom fused into the parent
    (
        r#"<robot name="r"><link name="base"><inertial><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="arm"><inertial><mass value="1"/><inertia ixx="0.1" iyy="0.2" izz="0.3" ixy="0" ixz="0" iyz="0"/></inertial><collision><origin xyz="0.05 0 0"/><geometry><box size="0.1 0.2 0.3"/></geometry></collision></link><joint name="j" type="fixed"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5" rpy="0.2 -0.6 1.1"/></joint></robot>"#,
        Expect::Loads(&Model {
            body_pos: &[[0.0, 0.0, 0.0]],
            body_quat: &[[1.0, 0.0, 0.0, 0.0]],
            body_mass: &[0.0],
            body_inertia: &[[0.0, 0.0, 0.0]],
            body_ipos: &[[0.0, 0.0, 0.0]],
            body_iquat: &[[1.0, 0.0, 0.0, 0.0]],
            body_simple: &[1],
            dof_simplenum: &[],
            geom_pos: &[[
                0.018718451689871195,
                0.03677725872641679,
                0.5282321236697518,
            ]],
            geom_quat: &[[
                0.7949582047263282,
                0.23500206304840215,
                -0.20082854509626413,
                0.5219994047657581,
            ]],
            geom_size: &[[0.05, 0.1, 0.15]],
            jnt_axis: &[],
            jnt_limited: &[],
            jnt_range: &[],
            dof_damping: &[],
            dof_frictionloss: &[],
            qpos0: &[],
            meaninertia: 1.0,
            xpos: &[[0.0, 0.0, 0.0]],
            xquat: &[[1.0, 0.0, 0.0, 0.0]],
            xipos: &[[0.0, 0.0, 0.0]],
            qacc: &[],
            qpos5: &[],
            enableflags: 0,
            disableflags: 0,
            timestep: 0.002,
            gravity: [0.0, 0.0, -9.81],
        }),
    ),
    // a prismatic joint in a turned origin
    (
        r#"<robot name="r"><link name="base"><inertial><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="arm"><inertial><mass value="1"/><inertia ixx="0.1" iyy="0.2" izz="0.3" ixy="0" ixz="0" iyz="0"/></inertial><collision><origin xyz="0.05 0 0"/><geometry><box size="0.1 0.2 0.3"/></geometry></collision></link><joint name="j" type="prismatic"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5" rpy="-0.4 0.8 0.2"/><axis xyz="1 0 0"/></joint></robot>"#,
        Expect::Loads(&Model {
            body_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            body_quat: &[
                [1.0, 0.0, 0.0, 0.0],
                [
                    0.8904676905493675,
                    -0.22017441332300763,
                    0.3614810376741493,
                    0.16709871096919254,
                ],
            ],
            body_mass: &[0.0, 1.0],
            body_inertia: &[[0.0, 0.0, 0.0], [0.3, 0.2, 0.1]],
            body_ipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            body_iquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 0.7071067811865489, 0.0, 0.7071067811865489],
            ],
            body_simple: &[1, 0],
            dof_simplenum: &[0],
            geom_pos: &[[0.05, 0.0, 0.0]],
            geom_quat: &[[1.0, 0.0, 0.0, 0.0]],
            geom_size: &[[0.05, 0.1, 0.15]],
            jnt_axis: &[[1.0, 0.0, 0.0]],
            jnt_limited: &[false],
            jnt_range: &[[0.0, 0.0]],
            dof_damping: &[0.0],
            dof_frictionloss: &[0.0],
            qpos0: &[0.0],
            meaninertia: 1.0,
            xpos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            xquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [
                    0.8904676905493675,
                    -0.22017441332300763,
                    0.3614810376741493,
                    0.16709871096919254,
                ],
            ],
            xipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            qacc: &[7.037263251724319],
            qpos5: &[0.00042223579510345917],
            enableflags: 0,
            disableflags: 0,
            timestep: 0.002,
            gravity: [0.0, 0.0, -9.81],
        }),
    ),
    // three turned joints in a chain
    (
        r#"<robot name="r"><link name="base"><inertial><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="l1"><inertial><mass value="1"/><inertia ixx="0.1" iyy="0.2" izz="0.3" ixy="0" ixz="0" iyz="0"/></inertial><collision><origin xyz="0.05 0 0" rpy="0.1 0.2 0.3"/><geometry><box size="0.1 0.2 0.3"/></geometry></collision></link><link name="l2"><inertial><mass value="1"/><inertia ixx="0.1" iyy="0.2" izz="0.3" ixy="0" ixz="0" iyz="0"/></inertial><collision><origin xyz="0.05 0 0" rpy="0.1 0.2 0.3"/><geometry><box size="0.1 0.2 0.3"/></geometry></collision></link><link name="l3"><inertial><mass value="1"/><inertia ixx="0.1" iyy="0.2" izz="0.3" ixy="0" ixz="0" iyz="0"/></inertial><collision><origin xyz="0.05 0 0" rpy="0.1 0.2 0.3"/><geometry><box size="0.1 0.2 0.3"/></geometry></collision></link><joint name="j1" type="revolute"><parent link="base"/><child link="l1"/><origin xyz="0 0 0.5" rpy="0.2 0.3 0.4"/><axis xyz="0 1 0"/></joint><joint name="j2" type="continuous"><parent link="l1"/><child link="l2"/><origin xyz="0 0 0.5" rpy="-0.5 0.1 0.7"/><axis xyz="1 0 0"/></joint><joint name="j3" type="revolute"><parent link="l2"/><child link="l3"/><origin xyz="0 0 0.5" rpy="1.0 -0.2 0.3"/><axis xyz="0 0 1"/></joint></robot>"#,
        Expect::Loads(&Model {
            body_pos: &[
                [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.5],
                [0.0, 0.0, 0.5],
                [0.0, 0.0, 0.5],
            ],
            body_quat: &[
                [1.0, 0.0, 0.0, 0.0],
                [
                    0.9671841473204752,
                    0.0672042655833245,
                    0.16533875799493103,
                    0.18083557991740754,
                ],
                [
                    0.9047924681355074,
                    -0.24871905972294878,
                    -0.03923871942853643,
                    0.3434381193656255,
                ],
                [
                    0.856240717808154,
                    0.4847664540368659,
                    -0.015341743204846797,
                    0.17781436703297324,
                ],
            ],
            body_mass: &[0.0, 1.0, 1.0, 1.0],
            body_inertia: &[
                [0.0, 0.0, 0.0],
                [0.3, 0.2, 0.1],
                [0.3, 0.2, 0.1],
                [0.3, 0.2, 0.1],
            ],
            body_ipos: &[
                [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0],
            ],
            body_iquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 0.7071067811865489, 0.0, 0.7071067811865489],
                [0.0, 0.7071067811865489, 0.0, 0.7071067811865489],
                [0.0, 0.7071067811865489, 0.0, 0.7071067811865489],
            ],
            body_simple: &[1, 0, 0, 0],
            dof_simplenum: &[0, 0, 0],
            geom_pos: &[[0.05, 0.0, 0.0], [0.05, 0.0, 0.0], [0.05, 0.0, 0.0]],
            geom_quat: &[
                [
                    0.9833474432563558,
                    0.034270798550482096,
                    0.10602051106179562,
                    0.1435721750273919,
                ],
                [
                    0.9833474432563558,
                    0.034270798550482096,
                    0.10602051106179562,
                    0.1435721750273919,
                ],
                [
                    0.9833474432563558,
                    0.034270798550482096,
                    0.10602051106179562,
                    0.1435721750273919,
                ],
            ],
            geom_size: &[[0.05, 0.1, 0.15], [0.05, 0.1, 0.15], [0.05, 0.1, 0.15]],
            jnt_axis: &[[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]],
            jnt_limited: &[false, false, false],
            jnt_range: &[[0.0, 0.0], [0.0, 0.0], [0.0, 0.0]],
            dof_damping: &[0.0, 0.0, 0.0],
            dof_frictionloss: &[0.0, 0.0, 0.0],
            qpos0: &[0.0, 0.0, 0.0],
            meaninertia: 0.8040715052098358,
            xpos: &[
                [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.5],
                [
                    0.17206594801003772,
                    -0.03509977011966921,
                    0.9681466817920996,
                ],
                [0.1466073704065945, 0.08508009457962462, 1.4528204493381356],
            ],
            xquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [
                    0.9671841473204752,
                    0.0672042655833245,
                    0.16533875799493103,
                    0.18083557991740754,
                ],
                [
                    0.8361977631952838,
                    -0.11587182968701702,
                    0.04358843354545987,
                    0.5342724664188115,
                ],
                [
                    0.677824751102791,
                    0.32209376661484335,
                    0.30409450536381544,
                    0.5868012816174133,
                ],
            ],
            xipos: &[
                [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.5],
                [
                    0.17206594801003772,
                    -0.03509977011966921,
                    0.9681466817920996,
                ],
                [0.1466073704065945, 0.08508009457962462, 1.4528204493381356],
            ],
            qacc: &[3.022308017688325, -4.40000896546912, 1.299942726954686],
            qpos5: &[
                0.00018135544523172441,
                -0.00026402472530273645,
                7.799234723583309e-05,
            ],
            enableflags: 0,
            disableflags: 0,
            timestep: 0.002,
            gravity: [0.0, 0.0, -9.81],
        }),
    ),
    // an rpy of 1e-11 and 2e-11, which the converter dropped
    (
        r#"<robot name="r"><link name="base"><inertial><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="arm"><inertial><mass value="1"/><inertia ixx="0.1" iyy="0.2" izz="0.3" ixy="0" ixz="0" iyz="0"/></inertial><collision><origin xyz="0.05 0 0"/><geometry><box size="0.1 0.2 0.3"/></geometry></collision></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5" rpy="1e-11 2e-11 0"/><axis xyz="0 0 1"/></joint></robot>"#,
        Expect::Loads(&Model {
            body_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            body_quat: &[
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 5e-12, 1e-11, -4.9999999999999997e-23],
            ],
            body_mass: &[0.0, 1.0],
            body_inertia: &[[0.0, 0.0, 0.0], [0.3, 0.2, 0.1]],
            body_ipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            body_iquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 0.7071067811865489, 0.0, 0.7071067811865489],
            ],
            body_simple: &[1, 0],
            dof_simplenum: &[0],
            geom_pos: &[[0.05, 0.0, 0.0]],
            geom_quat: &[[1.0, 0.0, 0.0, 0.0]],
            geom_size: &[[0.05, 0.1, 0.15]],
            jnt_axis: &[[0.0, 0.0, 1.0]],
            jnt_limited: &[false],
            jnt_range: &[[0.0, 0.0]],
            dof_damping: &[0.0],
            dof_frictionloss: &[0.0],
            qpos0: &[0.0],
            meaninertia: 0.30000000000000243,
            xpos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            xquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 5e-12, 1e-11, -4.9999999999999997e-23],
            ],
            xipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            qacc: &[0.0],
            qpos5: &[0.0],
            enableflags: 0,
            disableflags: 0,
            timestep: 0.002,
            gravity: [0.0, 0.0, -9.81],
        }),
    ),
    // a geom's rpy of 1e-11, which the converter dropped
    (
        r#"<robot name="r"><link name="base"><inertial><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="arm"><inertial><mass value="1"/><inertia ixx="0.1" iyy="0.2" izz="0.3" ixy="0" ixz="0" iyz="0"/></inertial><collision><origin xyz="0.05 0 0" rpy="0 1e-11 0"/><geometry><box size="0.1 0.2 0.3"/></geometry></collision></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5"/><axis xyz="0 0 1"/></joint></robot>"#,
        Expect::Loads(&Model {
            body_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            body_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            body_mass: &[0.0, 1.0],
            body_inertia: &[[0.0, 0.0, 0.0], [0.3, 0.2, 0.1]],
            body_ipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            body_iquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 0.7071067811865489, 0.0, 0.7071067811865489],
            ],
            body_simple: &[1, 0],
            dof_simplenum: &[0],
            geom_pos: &[[0.05, 0.0, 0.0]],
            geom_quat: &[[1.0, 0.0, 5e-12, 0.0]],
            geom_size: &[[0.05, 0.1, 0.15]],
            jnt_axis: &[[0.0, 0.0, 1.0]],
            jnt_limited: &[false],
            jnt_range: &[[0.0, 0.0]],
            dof_damping: &[0.0],
            dof_frictionloss: &[0.0],
            qpos0: &[0.0],
            meaninertia: 0.30000000000000243,
            xpos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            xquat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            xipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            qacc: &[0.0],
            qpos5: &[0.0],
            enableflags: 0,
            disableflags: 0,
            timestep: 0.002,
            gravity: [0.0, 0.0, -9.81],
        }),
    ),
    // a geom's xyz of 1e-11, which the converter dropped
    (
        r#"<robot name="r"><link name="base"><inertial><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="arm"><inertial><mass value="1"/><inertia ixx="0.1" iyy="0.2" izz="0.3" ixy="0" ixz="0" iyz="0"/></inertial><collision><origin xyz="1e-11 0 0"/><geometry><box size="0.1 0.2 0.3"/></geometry></collision></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5"/><axis xyz="0 0 1"/></joint></robot>"#,
        Expect::Loads(&Model {
            body_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            body_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            body_mass: &[0.0, 1.0],
            body_inertia: &[[0.0, 0.0, 0.0], [0.3, 0.2, 0.1]],
            body_ipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            body_iquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 0.7071067811865489, 0.0, 0.7071067811865489],
            ],
            body_simple: &[1, 0],
            dof_simplenum: &[0],
            geom_pos: &[[1e-11, 0.0, 0.0]],
            geom_quat: &[[1.0, 0.0, 0.0, 0.0]],
            geom_size: &[[0.05, 0.1, 0.15]],
            jnt_axis: &[[0.0, 0.0, 1.0]],
            jnt_limited: &[false],
            jnt_range: &[[0.0, 0.0]],
            dof_damping: &[0.0],
            dof_frictionloss: &[0.0],
            qpos0: &[0.0],
            meaninertia: 0.30000000000000243,
            xpos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            xquat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            xipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            qacc: &[0.0],
            qpos5: &[0.0],
            enableflags: 0,
            disableflags: 0,
            timestep: 0.002,
            gravity: [0.0, 0.0, -9.81],
        }),
    ),
    // an rpy of -0: the identity
    (
        r#"<robot name="r"><link name="base"><inertial><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="arm"><inertial><mass value="1"/><inertia ixx="0.1" iyy="0.2" izz="0.3" ixy="0" ixz="0" iyz="0"/></inertial><collision><origin xyz="0.05 0 0" rpy="-0 0 -0"/><geometry><box size="0.1 0.2 0.3"/></geometry></collision></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5" rpy="-0 -0 -0"/><axis xyz="0 0 1"/></joint></robot>"#,
        Expect::Loads(&Model {
            body_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            body_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            body_mass: &[0.0, 1.0],
            body_inertia: &[[0.0, 0.0, 0.0], [0.3, 0.2, 0.1]],
            body_ipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            body_iquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 0.7071067811865489, 0.0, 0.7071067811865489],
            ],
            body_simple: &[1, 0],
            dof_simplenum: &[0],
            geom_pos: &[[0.05, 0.0, 0.0]],
            geom_quat: &[[1.0, 0.0, 0.0, 0.0]],
            geom_size: &[[0.05, 0.1, 0.15]],
            jnt_axis: &[[0.0, 0.0, 1.0]],
            jnt_limited: &[false],
            jnt_range: &[[0.0, 0.0]],
            dof_damping: &[0.0],
            dof_frictionloss: &[0.0],
            qpos0: &[0.0],
            meaninertia: 0.30000000000000243,
            xpos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            xquat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            xipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            qacc: &[0.0],
            qpos5: &[0.0],
            enableflags: 0,
            disableflags: 0,
            timestep: 0.002,
            gravity: [0.0, 0.0, -9.81],
        }),
    ),
    // a cylinder, a capsule and a sphere in turned origins
    (
        r#"<robot name="r"><link name="base"><inertial><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="arm"><inertial><mass value="1"/><inertia ixx="0.1" iyy="0.2" izz="0.3" ixy="0" ixz="0" iyz="0"/></inertial><collision><origin rpy="0.3 -0.2 0.5"/><geometry><cylinder radius="0.05" length="0.2"/></geometry></collision><collision><origin xyz="0.1 0 0" rpy="-0.7 0.4 0.2"/><geometry><capsule radius="0.03" length="0.1"/></geometry></collision><collision><origin xyz="0 0.1 0" rpy="0.6 0.6 0.6"/><geometry><sphere radius="0.04"/></geometry></collision></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5" rpy="0.1 0.2 0.3"/><axis xyz="0 0 1"/></joint></robot>"#,
        Expect::Loads(&Model {
            body_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            body_quat: &[
                [1.0, 0.0, 0.0, 0.0],
                [
                    0.9833474432563558,
                    0.034270798550482096,
                    0.10602051106179562,
                    0.1435721750273919,
                ],
            ],
            body_mass: &[0.0, 1.0],
            body_inertia: &[[0.0, 0.0, 0.0], [0.3, 0.2, 0.1]],
            body_ipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            body_iquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 0.7071067811865489, 0.0, 0.7071067811865489],
            ],
            body_simple: &[1, 0],
            dof_simplenum: &[0],
            geom_pos: &[[0.0, 0.0, 0.0], [0.1, 0.0, 0.0], [0.0, 0.1, 0.0]],
            geom_quat: &[
                [
                    0.9495554075012557,
                    0.16849094096611827,
                    -0.058856783978165426,
                    0.2578588952842697,
                ],
                [
                    0.9092474161620773,
                    -0.3530151333729427,
                    0.15214191720853149,
                    0.15969436070313306,
                ],
                [
                    0.8977132865010043,
                    0.1862801488584703,
                    0.353143409285941,
                    0.18628014885847036,
                ],
            ],
            geom_size: &[[0.05, 0.1, 0.0], [0.03, 0.05, 0.0], [0.04, 0.0, 0.0]],
            jnt_axis: &[[0.0, 0.0, 1.0]],
            jnt_limited: &[false],
            jnt_range: &[[0.0, 0.0]],
            dof_damping: &[0.0],
            dof_frictionloss: &[0.0],
            qpos0: &[0.0],
            meaninertia: 0.30000000000000215,
            xpos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            xquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [
                    0.9833474432563558,
                    0.034270798550482096,
                    0.10602051106179562,
                    0.1435721750273919,
                ],
            ],
            xipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            qacc: &[0.0],
            qpos5: &[0.0],
            enableflags: 0,
            disableflags: 0,
            timestep: 0.002,
            gravity: [0.0, 0.0, -9.81],
        }),
    ),
    // a root link's turned geom, fused into the world
    (
        r#"<robot name="r"><link name="base"><inertial><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial><collision><origin rpy="0.3 0.4 0.5"/><geometry><box size="0.1 0.2 0.3"/></geometry></collision></link></robot>"#,
        Expect::Loads(&Model {
            body_pos: &[[0.0, 0.0, 0.0]],
            body_quat: &[[1.0, 0.0, 0.0, 0.0]],
            body_mass: &[0.0],
            body_inertia: &[[0.0, 0.0, 0.0]],
            body_ipos: &[[0.0, 0.0, 0.0]],
            body_iquat: &[[1.0, 0.0, 0.0, 0.0]],
            body_simple: &[1],
            dof_simplenum: &[],
            geom_pos: &[[0.0, 0.0, 0.0]],
            geom_quat: &[[
                0.9462808319656861,
                0.0933065937729005,
                0.2265663068902134,
                0.2109838268563661,
            ]],
            geom_size: &[[0.05, 0.1, 0.15]],
            jnt_axis: &[],
            jnt_limited: &[],
            jnt_range: &[],
            dof_damping: &[],
            dof_frictionloss: &[],
            qpos0: &[],
            meaninertia: 1.0,
            xpos: &[[0.0, 0.0, 0.0]],
            xquat: &[[1.0, 0.0, 0.0, 0.0]],
            xipos: &[[0.0, 0.0, 0.0]],
            qacc: &[],
            qpos5: &[],
            enableflags: 0,
            disableflags: 0,
            timestep: 0.002,
            gravity: [0.0, 0.0, -9.81],
        }),
    ),
    // a geom's xyz of -0: its sign kept, as MuJoCo keeps it
    (
        r#"<robot name="r"><link name="base"><inertial><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="arm"><inertial><mass value="1"/><inertia ixx="0.1" iyy="0.2" izz="0.3" ixy="0" ixz="0" iyz="0"/></inertial><collision><origin xyz="-0 -0 -0"/><geometry><box size="0.1 0.2 0.3"/></geometry></collision></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5"/><axis xyz="0 0 1"/></joint></robot>"#,
        Expect::Loads(&Model {
            body_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            body_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            body_mass: &[0.0, 1.0],
            body_inertia: &[[0.0, 0.0, 0.0], [0.3, 0.2, 0.1]],
            body_ipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            body_iquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 0.7071067811865489, 0.0, 0.7071067811865489],
            ],
            body_simple: &[1, 0],
            dof_simplenum: &[0],
            geom_pos: &[[-0.0, -0.0, -0.0]],
            geom_quat: &[[1.0, 0.0, 0.0, 0.0]],
            geom_size: &[[0.05, 0.1, 0.15]],
            jnt_axis: &[[0.0, 0.0, 1.0]],
            jnt_limited: &[false],
            jnt_range: &[[0.0, 0.0]],
            dof_damping: &[0.0],
            dof_frictionloss: &[0.0],
            qpos0: &[0.0],
            meaninertia: 0.30000000000000243,
            xpos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            xquat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            xipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            qacc: &[0.0],
            qpos5: &[0.0],
            enableflags: 0,
            disableflags: 0,
            timestep: 0.002,
            gravity: [0.0, 0.0, -9.81],
        }),
    ),
    // MuJoCo: "Error: mass and inertia of moving bodies must be larger than mjMINVAL"
    (
        r#"<robot name="r"><link name="base"><inertial><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="arm"><inertial><mass value="0"/><inertia ixx="0" iyy="0" izz="0" ixy="0" ixz="0" iyz="0"/></inertial><collision><origin xyz="0.05 0 0"/><geometry><box size="0.1 0.2 0.3"/></geometry></collision></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5"/><axis xyz="0 0 1"/></joint></robot>"#,
        Expect::Refused("mass and inertia of moving bodies must be larger than mjMINVAL"),
    ),
    // MuJoCo: "Error: mass and inertia of moving bodies must be larger than mjMINVAL"
    (
        r#"<robot name="r"><link name="base"><inertial><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="arm"><inertial><mass value="1"/><inertia ixx="0" iyy="0" izz="0" ixy="0" ixz="0" iyz="0"/></inertial><collision><origin xyz="0.05 0 0"/><geometry><box size="0.1 0.2 0.3"/></geometry></collision></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5"/><axis xyz="0 0 1"/></joint></robot>"#,
        Expect::Refused("mass and inertia of moving bodies must be larger than mjMINVAL"),
    ),
    // MuJoCo: "Error: inertia must satisfy A + B >= C; use 'balanceinertia' to fix"
    (
        r#"<robot name="r"><link name="base"><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="1" iyy="1" izz="1" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="arm"><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="tip"><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="1" iyy="1" izz="3" ixy="0" ixz="0" iyz="0"/></inertial></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 1"/><axis xyz="0 1 0"/><limit lower="-1" upper="1" effort="1" velocity="1"/></joint><joint name="f" type="fixed"><parent link="arm"/><child link="tip"/><origin xyz="0 0 0.5"/></joint></robot>"#,
        Expect::Refused("inertia must satisfy A + B >= C"),
    ),
    // MuJoCo: "Error: error 'inertia must have positive eigenvalues' in fullinertia"
    (
        r#"<robot name="r"><link name="base"><inertial><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="t"><inertial><mass value="1"/><inertia ixx="-0.1" iyy="0.2" izz="0.3" ixy="0" ixz="0" iyz="0"/></inertial></link><joint name="f" type="fixed"><parent link="base"/><child link="t"/><origin xyz="0 0 0.5"/></joint></robot>"#,
        Expect::Refused("inertia must have positive eigenvalues"),
    ),
    // MuJoCo: "Error: error 'inertia must have positive eigenvalues' in fullinertia"
    (
        r#"<robot name="r"><link name="base"><inertial><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="arm"><inertial><mass value="1"/><inertia ixx="-0.1" iyy="0.2" izz="0.3" ixy="0" ixz="0" iyz="0"/></inertial><collision><origin xyz="0.05 0 0"/><geometry><box size="0.1 0.2 0.3"/></geometry></collision></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5"/><axis xyz="0 0 1"/></joint></robot>"#,
        Expect::Refused("inertia must have positive eigenvalues"),
    ),
    // MuJoCo: "Error: inertia must satisfy A + B >= C; use 'balanceinertia' to fix"
    (
        r#"<robot name="r"><link name="base"><inertial><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="arm"><inertial><origin xyz="0 0 0" rpy="-2.73641 0.229645 -0.859591"/><mass value="1"/><inertia ixx="2.0000000000000004" iyy="1" izz="1" ixy="0" ixz="0" iyz="0"/></inertial></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5"/><axis xyz="0 0 1"/></joint></robot>"#,
        Expect::Refused("inertia must satisfy A + B >= C"),
    ),
    // MuJoCo: "Error: error 'inertia must have positive eigenvalues' in fullinertia"
    (
        r#"<robot name="r"><link name="base"><inertial><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="arm"><inertial><origin rpy="0.3 0.5 0.7"/><mass value="1"/><inertia ixx="-0.1" iyy="0.2" izz="0.3" ixy="0" ixz="0" iyz="0"/></inertial><collision><origin xyz="0.05 0 0"/><geometry><box size="0.1 0.2 0.3"/></geometry></collision></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5"/><axis xyz="0 0 1"/></joint></robot>"#,
        Expect::Refused("inertia must have positive eigenvalues"),
    ),
    // MuJoCo: "Error: error 'inertia must have positive eigenvalues' in fullinertia"
    (
        r#"<robot name="r"><link name="base"><inertial><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="arm"><inertial><origin xyz="0 0 0" rpy="-1.60178 -2.90773 -0.648646"/><mass value="1"/><inertia ixx="9.999999999999998e-15" iyy="9.999999999999998e-15" izz="9.999999999999998e-15" ixy="0" ixz="0" iyz="0"/></inertial></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5"/><axis xyz="0 0 1"/></joint></robot>"#,
        Expect::Refused("inertia must have positive eigenvalues"),
    ),
];

/// Documents whose `<inertial>` origin turns, each with its link's principal
/// moments, MuJoCo 3.5.0's, and its inertial frame, the turn composed with
/// MuJoCo's principal axes, as MuJoCo's import computes it before its compiler
/// overwrites it.
const TURNED: &[(&str, [f64; 3], [f64; 4])] = &[
    // a diagonal tensor in an origin turned about three axes
    (
        r#"<robot name="r"><link name="base"><inertial><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="arm"><inertial><origin xyz="0.01 0.02 0.03" rpy="0.3 0.5 0.7"/><mass value="1"/><inertia ixx="0.1" iyy="0.2" izz="0.3" ixy="0" ixz="0" iyz="0"/></inertial><collision><origin xyz="0.05 0 0"/><geometry><box size="0.1 0.2 0.3"/></geometry></collision></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5"/><axis xyz="0 0 1"/></joint></robot>"#,
        [0.3, 0.2, 0.1],
        [
            -0.2445950119726401,
            0.8429215111361577,
            0.17086864945343438,
            0.4477281662080279,
        ],
    ),
    // a full tensor in an origin turned about three axes
    (
        r#"<robot name="r"><link name="base"><inertial><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="arm"><inertial><origin rpy="0.3 0.5 0.7"/><mass value="1"/><inertia ixx="0.5" iyy="0.4" izz="0.3" ixy="0.05" ixz="0.02" iyz="0.01"/></inertial><collision><origin xyz="0.05 0 0"/><geometry><box size="0.1 0.2 0.3"/></geometry></collision></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5"/><axis xyz="0 0 1"/></joint></robot>"#,
        [0.5229423155075864, 0.3793196536284876, 0.297738030863926],
        [
            0.8483512627293807,
            0.13708145321389958,
            0.22619525281265973,
            0.4586333152042573,
        ],
    ),
    // a diagonal tensor in an origin turned about z
    (
        r#"<robot name="r"><link name="base"><inertial><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="arm"><inertial><origin rpy="0 0 0.5"/><mass value="1"/><inertia ixx="0.1" iyy="0.2" izz="0.3" ixy="0" ixz="0" iyz="0"/></inertial><collision><origin xyz="0.05 0 0"/><geometry><box size="0.1 0.2 0.3"/></geometry></collision></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5"/><axis xyz="0 0 1"/></joint></robot>"#,
        [0.3, 0.2, 0.1],
        [
            -0.17494101728127381,
            0.6851245437674781,
            0.17494101728127381,
            0.6851245437674781,
        ],
    ),
    // an isotropic tensor in a turned origin
    (
        r#"<robot name="r"><link name="base"><inertial><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="arm"><inertial><origin rpy="0.3 0.5 0.7"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial><collision><origin xyz="0.05 0 0"/><geometry><box size="0.1 0.2 0.3"/></geometry></collision></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5"/><axis xyz="0 0 1"/></joint></robot>"#,
        [0.1, 0.1, 0.1],
        [
            0.9126271389863014,
            0.052132410889547995,
            0.2794438940784743,
            0.29377717233096856,
        ],
    ),
    // a tensor on the triangle's edge, A + B = C, turned (review RA)
    (
        r#"<robot name="r"><link name="base"><inertial><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="arm"><inertial><origin xyz="0 0 0" rpy="-1.12747 -2.23457 0.965981"/><mass value="1"/><inertia ixx="1" iyy="1" izz="2" ixy="0" ixz="0" iyz="0"/></inertial></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5"/><axis xyz="0 0 1"/></joint></robot>"#,
        [2.0, 1.0, 1.0],
        [
            -0.061370952383563576,
            -0.16921221565602865,
            -0.865814713495449,
            0.46686798366466276,
        ],
    ),
    // an isotropic tensor of mjEPS, turned (review RA)
    (
        r#"<robot name="r"><link name="base"><inertial><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="arm"><inertial><origin xyz="0 0 0" rpy="-0.304771 0.382543 2.71495"/><mass value="1"/><inertia ixx="1e-14" iyy="1e-14" izz="1e-14" ixy="0" ixz="0" iyz="0"/></inertial></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5"/><axis xyz="0 0 1"/></joint></robot>"#,
        [1e-14, 1e-14, 1e-14],
        [
            0.17723419335906102,
            -0.21519547511372214,
            -0.10586946942048892,
            0.9545001852485764,
        ],
    ),
    // an isotropic tensor turned by 3e-308: a component of its frame subnormal, written as zero (review FA)
    (
        r#"<robot name="r"><link name="base"><inertial><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="arm"><inertial><origin xyz="0.1 0.02 0.03" rpy="3e-308 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5"/><axis xyz="0.3 0.4 0.866"/><limit lower="-1" upper="1" effort="1" velocity="1"/></joint></robot>"#,
        [0.1, 0.1, 0.1],
        [1.0, 1.5000000000000004e-308, 0.0, 0.0],
    ),
    // a diagonal tensor written largest first, turned by 1e-160 about two axes (review FA)
    (
        r#"<robot name="r"><link name="base"><inertial><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="arm"><inertial><origin xyz="0.1 0.02 0.03" rpy="1e-160 1e-160 0"/><mass value="1"/><inertia ixx="0.3" iyy="0.2" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5"/><axis xyz="0.3 0.4 0.866"/><limit lower="-1" upper="1" effort="1" velocity="1"/></joint></robot>"#,
        [0.3, 0.2, 0.1],
        [1.0, 5e-161, 5e-161, -2.5e-321],
    ),
];

/// Documents with a link of negative mass, which MuJoCo 3.5.0's import
/// raises to `boundmass` (0) before fusing it: each loads in MuJoCo.
const NEGATIVE_MASS: &[&str] = &[
    r#"<robot name="r"><link name="base"><inertial><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="t"><inertial><mass value="-1"/><inertia ixx="0.1" iyy="0.2" izz="0.3" ixy="0" ixz="0" iyz="0"/></inertial></link><joint name="f" type="fixed"><parent link="base"/><child link="t"/><origin xyz="0 0 0.5"/></joint></robot>"#,
    r#"<robot name="r"><link name="base"><inertial><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="arm"><inertial><mass value="1"/><inertia ixx="0.3" iyy="0.2" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="t"><inertial><mass value="-0.25"/><inertia ixx="0.03" iyy="0.02" izz="0.01" ixy="0" ixz="0" iyz="0"/></inertial></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5"/><axis xyz="0 1 0"/></joint><joint name="f" type="fixed"><parent link="arm"/><child link="t"/><origin xyz="0.2 0 0"/></joint></robot>"#,
];

/// Documents with a tensor in a turned origin whose principal moments hold a
/// negative one `mjuu_eig3` does not sort last: each loads in MuJoCo, the
/// moment raised to `boundinertia` (0).
const RAISED: &[&str] = &[
    r#"<robot name="r"><link name="base"><inertial><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="t"><inertial><origin rpy="0.3 0.5 0.7"/><mass value="1"/><inertia ixx="2e-14" iyy="-1e-15" izz="2e-14" ixy="0" ixz="0" iyz="0"/></inertial></link><joint name="f" type="fixed"><parent link="base"/><child link="t"/><origin xyz="0 0 0.5"/></joint></robot>"#,
    r#"<robot name="r"><link name="base"><inertial><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="arm"><inertial><mass value="1"/><inertia ixx="0.3" iyy="0.2" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="t"><inertial><origin rpy="0.3 0.5 0.7"/><mass value="0.25"/><inertia ixx="2e-14" iyy="-1e-15" izz="2e-14" ixy="0" ixz="0" iyz="0"/></inertial></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5"/><axis xyz="0 1 0"/></joint><joint name="f" type="fixed"><parent link="arm"/><child link="t"/><origin xyz="0.2 0 0"/></joint></robot>"#,
    r#"<robot name="r"><link name="base"><inertial><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="arm"><inertial><mass value="1"/><inertia ixx="0.3" iyy="0.2" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="t"><inertial><origin rpy="0.3 0.5 0.7"/><mass value="0.25"/><inertia ixx="-1e-15" iyy="2e-14" izz="2e-14" ixy="0" ixz="0" iyz="0"/></inertial></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5"/><axis xyz="0 1 0"/></joint><joint name="f" type="fixed"><parent link="arm"/><child link="t"/><origin xyz="0.2 0 0"/></joint></robot>"#,
    r#"<robot name="r"><link name="base"><inertial><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="arm"><inertial><mass value="1"/><inertia ixx="0.3" iyy="0.2" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="t"><inertial><origin rpy="0.3 0.5 0.7"/><mass value="0.25"/><inertia ixx="-5e-13" iyy="5e-13" izz="5e-13" ixy="0" ixz="0" iyz="0"/></inertial></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5"/><axis xyz="0 1 0"/></joint><joint name="f" type="fixed"><parent link="arm"/><child link="t"/><origin xyz="0.2 0 0"/></joint></robot>"#,
];

/// Documents with a turned tensor near the largest double, whose principal
/// moments overflow: each loads in MuJoCo.
const OVERFLOW: &[&str] = &[
    r#"<robot name="r"><link name="base"><inertial><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="arm"><inertial><origin xyz="0.1 0.02 0.03" rpy="0.3 0.5 0.7"/><mass value="1"/><inertia ixx="1.7e+308" iyy="1.7e+308" izz="1e+308" ixy="1e+307" ixz="0" iyz="0"/></inertial></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5"/><axis xyz="0.3 0.4 0.866"/><limit lower="-1" upper="1" effort="1" velocity="1"/></joint></robot>"#,
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
    // Equal bits pass; otherwise both entries are finite and within the
    // tolerance of the array's largest finite entry, and two zeros have one
    // sign.
    let scale = mujoco
        .iter()
        .filter(|v| v.is_finite())
        .fold(0.0_f64, |m, v| m.max(v.abs()));
    ours.iter().zip(mujoco).all(|(a, b)| {
        a.to_bits() == b.to_bits()
            || (a.is_finite()
                && b.is_finite()
                && (a - b).abs() <= TOLERANCE * scale
                && !(*a == 0.0 && *b == 0.0))
    })
}

fn flat3(v: &[nalgebra::Vector3<f64>]) -> Vec<f64> {
    v.iter().flat_map(|x| x.iter().copied()).collect()
}

fn flat4(v: &[nalgebra::UnitQuaternion<f64>]) -> Vec<f64> {
    v.iter().flat_map(|q| [q.w, q.i, q.j, q.k]).collect()
}

/// The fields of `m` that are not MuJoCo 3.5.0's `want`, its first `forward`
/// and five steps run.
fn differing(m: &sim_core::Model, want: &Model) -> Vec<&'static str> {
    let mut d = m.make_data();
    d.forward(m).unwrap();
    let (world, turn, inertial) = (flat3(&d.xpos), flat4(&d.xquat), flat3(&d.xipos));
    let qacc = d.qacc.as_slice().to_vec();
    for _ in 0..5 {
        d.step(m).unwrap();
    }
    let range: Vec<f64> = m
        .jnt_range
        .iter()
        .flat_map(|&bounds| <[f64; 2]>::from(bounds))
        .collect();
    [
        (
            "body_pos",
            close(&flat3(&m.body_pos), want.body_pos.as_flattened()),
        ),
        (
            "body_quat",
            close(&flat4(&m.body_quat), want.body_quat.as_flattened()),
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
        ("body_simple", m.body_simple == want.body_simple),
        ("dof_simplenum", m.dof_simplenum == want.dof_simplenum),
        (
            "geom_pos",
            close(&flat3(&m.geom_pos), want.geom_pos.as_flattened()),
        ),
        (
            "geom_quat",
            close(&flat4(&m.geom_quat), want.geom_quat.as_flattened()),
        ),
        (
            "geom_size",
            close(&flat3(&m.geom_size), want.geom_size.as_flattened()),
        ),
        (
            "jnt_axis",
            close(&flat3(&m.jnt_axis), want.jnt_axis.as_flattened()),
        ),
        ("jnt_limited", m.jnt_limited == want.jnt_limited),
        ("jnt_range", close(&range, want.jnt_range.as_flattened())),
        ("dof_damping", close(&m.dof_damping, want.dof_damping)),
        (
            "dof_frictionloss",
            close(&m.dof_frictionloss, want.dof_frictionloss),
        ),
        ("qpos0", close(m.qpos0.as_slice(), want.qpos0)),
        (
            "meaninertia",
            close(&[m.stat_meaninertia], &[want.meaninertia]),
        ),
        ("xpos", close(&world, want.xpos.as_flattened())),
        ("xquat", close(&turn, want.xquat.as_flattened())),
        ("xipos", close(&inertial, want.xipos.as_flattened())),
        ("qacc", close(&qacc, want.qacc)),
        ("qpos5", close(d.qpos.as_slice(), want.qpos5)),
        ("enableflags", m.enableflags == want.enableflags),
        ("disableflags", m.disableflags == want.disableflags),
        ("timestep", close(&[m.timestep], &[want.timestep])),
        ("gravity", close(m.gravity.as_slice(), &want.gravity)),
    ]
    .into_iter()
    .filter(|&(_, same)| !same)
    .map(|(field, _)| field)
    .collect()
}

/// Each document's bodies, inertias, geoms and joints, its first `forward`
/// and five steps, as MuJoCo 3.5.0's URDF import makes them; each document
/// MuJoCo refuses refused for its fault.
#[test]
fn each_urdf_loads_as_mujocos_import() {
    let mut wrong = Vec::new();
    for (doc, expect) in CASES {
        match (load_urdf_model(doc), expect) {
            (Ok(m), Expect::Loads(want)) => {
                let differ = differing(&m, want);
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

/// A link whose `<inertial>` origin turns keeps MuJoCo 3.5.0's principal
/// moments, bit for bit, and takes the turn composed with
/// MuJoCo's principal axes as its inertial frame, within 1e-12 on every
/// platform (the turn's quaternion is this crate's arithmetic, the expected
/// one MuJoCo's `mju_euler2Quat`): its tensor is R·I·Rᵀ in the link's frame,
/// as URDF defines the origin (ledger L372; D-URDF-INERTIAL-TURN), and every
/// check of its moments is MuJoCo's.
#[test]
fn an_inertial_origins_turn_turns_its_tensor() {
    let mut wrong = Vec::new();
    for (doc, moments, frame) in TURNED {
        let m = match load_urdf_model(doc) {
            Ok(m) => m,
            Err(e) => {
                wrong.push(format!("refused ({e}): {doc}"));
                continue;
            }
        };
        let (inertia, q) = (m.body_inertia[1], m.body_iquat[1]);
        let turned = [q.w, q.i, q.j, q.k];
        let frame_ok = turned
            .iter()
            .zip(frame)
            .all(|(a, b)| (a - b).abs() <= 1e-12);
        // Bit for bit on every platform: `mjuu_eig3` is arithmetic and square
        // roots, which round the same everywhere.
        let moments_ok = inertia
            .iter()
            .zip(moments)
            .all(|(a, b)| a.to_bits() == b.to_bits());
        if !moments_ok || !frame_ok {
            wrong.push(format!(
                "{inertia:?} {turned:?}, want {moments:?} {frame:?}: {doc}"
            ));
        }
    }
    assert!(wrong.is_empty(), "{}", wrong.join("\n"));
}

/// A link's negative mass is refused, where MuJoCo 3.5.0's import raises it
/// to `boundmass` (0) and fuses the fixed link into its parent, adding
/// nothing (D-INERTIAL-NEGATIVE).
#[test]
fn a_negative_mass_is_refused_where_mujocos_import_raises_it() {
    for doc in NEGATIVE_MASS {
        let refused = load_urdf_model(doc).err().map(|e| e.to_string());
        assert!(
            refused
                .as_deref()
                .is_some_and(|e| e.contains("invalid mass on <inertial>: must not be negative")),
            "{refused:?}: {doc}"
        );
    }
}

/// A turned tensor whose principal moments overflow is refused as the same
/// tensor unturned is, in D-INERTIA-NONFINITE's words, where MuJoCo 3.5.0's
/// import loads it.
#[test]
fn a_turned_tensor_whose_moments_overflow_is_refused_as_unturned() {
    for doc in OVERFLOW {
        let refused = load_urdf_model(doc).err().map(|e| e.to_string());
        assert!(
            refused
                .as_deref()
                .is_some_and(|e| e.contains("its principal moments or axes are not finite")),
            "{refused:?}: {doc}"
        );
    }
}

/// A turned tensor whose principal moments hold a negative one that
/// `mjuu_eig3` does not sort last (moments within 1e-12 of each other) loads,
/// the moment raised to 0 as MuJoCo 3.5.0's compiler raises it to
/// `boundinertia`, where MuJoCo's import loads it.
#[test]
fn a_turned_tensors_negative_principal_moment_is_raised_as_mujoco_raises_it() {
    for doc in RAISED {
        assert!(
            load_urdf_model(doc).is_ok(),
            "{:?}: {doc}",
            load_urdf_model(doc).err()
        );
    }
}
