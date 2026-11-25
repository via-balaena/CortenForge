//! A URDF's model as MuJoCo 3.5.0's own importer makes it (`xml_urdf.cc`):
//! every tensor its body's `fullinertia`, so a diagonal one takes its
//! principal axes as the inertial frame, the moments largest first, and six
//! zeros a tensor not given.
//!
//! Every expected value was printed by MuJoCo 3.5.0's URDF import on the
//! census's oracle (2026-11-25).

// The values are MuJoCo's, printed to the last digit.
#![allow(clippy::unwrap_used, clippy::unreadable_literal)]

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
        }),
    ),
    // an isotropic tensor in a turned origin: no axes to take
    (
        r#"<robot name="r"><link name="base"><inertial><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="arm"><inertial><origin rpy="0.3 0.5 0.7"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial><collision><origin xyz="0.05 0 0"/><geometry><box size="0.1 0.2 0.3"/></geometry></collision></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5"/><axis xyz="0 0 1"/></joint></robot>"#,
        Expect::Loads(&Model {
            body_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            body_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            body_mass: &[0.0, 1.0],
            body_inertia: &[[0.0, 0.0, 0.0], [0.1, 0.1, 0.1]],
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
        }),
    ),
    // an off-diagonal moment of 1e-11, which the converter dropped
    (
        r#"<robot name="r"><link name="base"><inertial><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="arm"><inertial><mass value="1"/><inertia ixx="0.1" iyy="0.2" izz="0.3" ixy="1e-11" ixz="0" iyz="0"/></inertial><collision><origin xyz="0.05 0 0"/><geometry><box size="0.1 0.2 0.3"/></geometry></collision></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5"/><axis xyz="0 0 1"/></joint></robot>"#,
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
