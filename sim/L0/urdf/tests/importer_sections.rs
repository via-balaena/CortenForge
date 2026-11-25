//! What a URDF holds beyond its links and joints, as MuJoCo 3.5.0's own
//! importer reads it (`xml_urdf.cc`): each geom's colour, the colour its
//! link's visuals last set before it, from a material's own `<color>` or the
//! robot's material table (`:291-347`, `:741-768`).
//!
//! Every expected value was printed on the census's oracle (2026-11-25):
//! MuJoCo 3.5.0's URDF import's.

// The values are MuJoCo's, printed to the last digit.
#![allow(clippy::unwrap_used, clippy::unreadable_literal)]

use sim_urdf::load_urdf_model;

/// The fields of a converted URDF MuJoCo 3.5.0's import decides, and `qpos`
/// after five steps.
struct Model {
    name: &'static str,
    body_parent: &'static [usize],
    body_mass: &'static [f64],
    body_inertia: &'static [[f64; 3]],
    body_ipos: &'static [[f64; 3]],
    /// (w, x, y, z).
    body_iquat: &'static [[f64; 4]],
    geom_name: &'static [&'static str],
    /// Each geom's type by its name in sim-core's `GeomType`.
    geom_type: &'static [&'static str],
    geom_body: &'static [usize],
    geom_pos: &'static [[f64; 3]],
    geom_size: &'static [[f64; 3]],
    geom_contype: &'static [u32],
    geom_conaffinity: &'static [u32],
    geom_group: &'static [i32],
    /// Each geom's colour, MuJoCo's single-precision values.
    geom_rgba: &'static [[f64; 4]],
    jnt_limited: &'static [bool],
    jnt_range: &'static [[f64; 2]],
    /// Every keyframe's `qpos`, in order.
    key_qpos: &'static [f64],
    qpos5: &'static [f64],
    /// `timestep`, `impratio`, `tolerance`, `ls_tolerance`,
    /// `noslip_tolerance`, `ccd_tolerance`, `density`, `viscosity`,
    /// `o_margin`.
    reals: [f64; 9],
    /// `gravity`, `wind`, `magnetic`.
    vectors: [f64; 9],
    /// `o_solref`, `o_solimp`, `o_friction`.
    contact_override: [f64; 12],
    /// `integrator`, `cone`, `jacobian`, `solver` (MuJoCo's numbers), then
    /// `iterations`, `ls_iterations`, `noslip_iterations`,
    /// `ccd_iterations`, `sdf_iterations`, `sdf_initpoints`.
    counts: [i64; 10],
    /// `enableflags`, `disableflags`, `disableactuator`.
    flags: [u32; 3],
    /// `nuser_body`, `nuser_jnt`, `nuser_geom`.
    nuser: [i32; 3],
}

/// What MuJoCo 3.5.0 makes of a document.
enum Expect {
    Loads(&'static Model),
    /// Refused for MuJoCo's fault, in this crate's words.
    Refused(&'static str),
}

const CASES: &[(&str, Expect)] = &[
    // a visual's colour, taken by the collision after it
    (
        r#"<robot name="r"><link name="base"><inertial><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial><collision><origin xyz="0 0 0"/><geometry><box size="0.1 0.2 0.3"/></geometry></collision></link><link name="arm"><inertial><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial><visual><origin xyz="0 0 0.2"/><geometry><box size="0.05 0.05 0.05"/></geometry><material name="red"><color rgba="1 0 0 1"/></material></visual><collision><origin xyz="0 0 0.1"/><geometry><box size="0.1 0.2 0.3"/></geometry></collision></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5" rpy="0.3 0 0"/><axis xyz="0 1 0"/><limit lower="-1" upper="1" effort="2" velocity="1"/></joint></robot>"#,
        Expect::Loads(&Model {
            name: "r",
            body_parent: &[0, 0],
            body_mass: &[0.0, 1.0],
            body_inertia: &[[0.0, 0.0, 0.0], [0.1, 0.1, 0.1]],
            body_ipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            body_iquat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            geom_name: &["", ""],
            geom_type: &["Box", "Box"],
            geom_body: &[0, 1],
            geom_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.1]],
            geom_size: &[[0.05, 0.1, 0.15], [0.05, 0.1, 0.15]],
            geom_contype: &[1, 1],
            geom_conaffinity: &[1, 1],
            geom_group: &[0, 0],
            geom_rgba: &[[0.5, 0.5, 0.5, 1.0], [1.0, 0.0, 0.0, 1.0]],
            jnt_limited: &[true],
            jnt_range: &[[-1.0, 1.0]],
            key_qpos: &[],
            qpos5: &[0.0],
            reals: [0.002, 1.0, 1e-08, 0.01, 1e-06, 1e-06, 0.0, 0.0, 0.0],
            vectors: [0.0, 0.0, -9.81, 0.0, 0.0, 0.0, 0.0, -0.5, 0.0],
            contact_override: [
                0.02, 1.0, 0.9, 0.95, 0.001, 0.5, 2.0, 1.0, 1.0, 0.005, 0.0001, 0.0001,
            ],
            counts: [0, 0, 2, 2, 100, 50, 0, 35, 10, 40],
            flags: [0, 0, 0],
            nuser: [0, 0, 0],
        }),
    ),
    // a visual's material named in the robot's table
    (
        r#"<robot name="r"><link name="base"><inertial><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial><collision><origin xyz="0 0 0"/><geometry><box size="0.1 0.2 0.3"/></geometry></collision></link><link name="arm"><inertial><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial><visual><origin xyz="0 0 0.2"/><geometry><box size="0.05 0.05 0.05"/></geometry><material name="blue"/></visual><collision><origin xyz="0 0 0.1"/><geometry><box size="0.1 0.2 0.3"/></geometry></collision></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5" rpy="0.3 0 0"/><axis xyz="0 1 0"/><limit lower="-1" upper="1" effort="2" velocity="1"/></joint><material name="blue"><color rgba="0 0 1 1"/></material><material name="tex"><texture filename="t.png"/></material></robot>"#,
        Expect::Loads(&Model {
            name: "r",
            body_parent: &[0, 0],
            body_mass: &[0.0, 1.0],
            body_inertia: &[[0.0, 0.0, 0.0], [0.1, 0.1, 0.1]],
            body_ipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            body_iquat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            geom_name: &["", ""],
            geom_type: &["Box", "Box"],
            geom_body: &[0, 1],
            geom_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.1]],
            geom_size: &[[0.05, 0.1, 0.15], [0.05, 0.1, 0.15]],
            geom_contype: &[1, 1],
            geom_conaffinity: &[1, 1],
            geom_group: &[0, 0],
            geom_rgba: &[[0.5, 0.5, 0.5, 1.0], [0.0, 0.0, 1.0, 1.0]],
            jnt_limited: &[true],
            jnt_range: &[[-1.0, 1.0]],
            key_qpos: &[],
            qpos5: &[0.0],
            reals: [0.002, 1.0, 1e-08, 0.01, 1e-06, 1e-06, 0.0, 0.0, 0.0],
            vectors: [0.0, 0.0, -9.81, 0.0, 0.0, 0.0, 0.0, -0.5, 0.0],
            contact_override: [
                0.02, 1.0, 0.9, 0.95, 0.001, 0.5, 2.0, 1.0, 1.0, 0.005, 0.0001, 0.0001,
            ],
            counts: [0, 0, 2, 2, 100, 50, 0, 35, 10, 40],
            flags: [0, 0, 0],
            nuser: [0, 0, 0],
        }),
    ),
    // a material name the table does not hold: no colour
    (
        r#"<robot name="r"><link name="base"><inertial><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial><collision><origin xyz="0 0 0"/><geometry><box size="0.1 0.2 0.3"/></geometry></collision></link><link name="arm"><inertial><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial><visual><origin xyz="0 0 0.2"/><geometry><box size="0.05 0.05 0.05"/></geometry><material name="nope"/></visual><collision><origin xyz="0 0 0.1"/><geometry><box size="0.1 0.2 0.3"/></geometry></collision></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5" rpy="0.3 0 0"/><axis xyz="0 1 0"/><limit lower="-1" upper="1" effort="2" velocity="1"/></joint></robot>"#,
        Expect::Loads(&Model {
            name: "r",
            body_parent: &[0, 0],
            body_mass: &[0.0, 1.0],
            body_inertia: &[[0.0, 0.0, 0.0], [0.1, 0.1, 0.1]],
            body_ipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            body_iquat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            geom_name: &["", ""],
            geom_type: &["Box", "Box"],
            geom_body: &[0, 1],
            geom_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.1]],
            geom_size: &[[0.05, 0.1, 0.15], [0.05, 0.1, 0.15]],
            geom_contype: &[1, 1],
            geom_conaffinity: &[1, 1],
            geom_group: &[0, 0],
            geom_rgba: &[[0.5, 0.5, 0.5, 1.0], [0.5, 0.5, 0.5, 1.0]],
            jnt_limited: &[true],
            jnt_range: &[[-1.0, 1.0]],
            key_qpos: &[],
            qpos5: &[0.0],
            reals: [0.002, 1.0, 1e-08, 0.01, 1e-06, 1e-06, 0.0, 0.0, 0.0],
            vectors: [0.0, 0.0, -9.81, 0.0, 0.0, 0.0, 0.0, -0.5, 0.0],
            contact_override: [
                0.02, 1.0, 0.9, 0.95, 0.001, 0.5, 2.0, 1.0, 1.0, 0.005, 0.0001, 0.0001,
            ],
            counts: [0, 0, 2, 2, 100, 50, 0, 35, 10, 40],
            flags: [0, 0, 0],
            nuser: [0, 0, 0],
        }),
    ),
    // two visuals: the collision takes the last one's colour
    (
        r#"<robot name="r"><link name="base"><inertial><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial><collision><origin xyz="0 0 0"/><geometry><box size="0.1 0.2 0.3"/></geometry></collision></link><link name="arm"><inertial><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial><visual><origin xyz="0 0 0.2"/><geometry><box size="0.05 0.05 0.05"/></geometry><material name="red"><color rgba="1 0 0 1"/></material></visual><visual><origin xyz="0 0 0.3"/><geometry><box size="0.05 0.05 0.05"/></geometry><material name="green"><color rgba="0 1 0 1"/></material></visual><collision><origin xyz="0 0 0.1"/><geometry><box size="0.1 0.2 0.3"/></geometry></collision></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5" rpy="0.3 0 0"/><axis xyz="0 1 0"/><limit lower="-1" upper="1" effort="2" velocity="1"/></joint></robot>"#,
        Expect::Loads(&Model {
            name: "r",
            body_parent: &[0, 0],
            body_mass: &[0.0, 1.0],
            body_inertia: &[[0.0, 0.0, 0.0], [0.1, 0.1, 0.1]],
            body_ipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            body_iquat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            geom_name: &["", ""],
            geom_type: &["Box", "Box"],
            geom_body: &[0, 1],
            geom_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.1]],
            geom_size: &[[0.05, 0.1, 0.15], [0.05, 0.1, 0.15]],
            geom_contype: &[1, 1],
            geom_conaffinity: &[1, 1],
            geom_group: &[0, 0],
            geom_rgba: &[[0.5, 0.5, 0.5, 1.0], [0.0, 1.0, 0.0, 1.0]],
            jnt_limited: &[true],
            jnt_range: &[[-1.0, 1.0]],
            key_qpos: &[],
            qpos5: &[0.0],
            reals: [0.002, 1.0, 1e-08, 0.01, 1e-06, 1e-06, 0.0, 0.0, 0.0],
            vectors: [0.0, 0.0, -9.81, 0.0, 0.0, 0.0, 0.0, -0.5, 0.0],
            contact_override: [
                0.02, 1.0, 0.9, 0.95, 0.001, 0.5, 2.0, 1.0, 1.0, 0.005, 0.0001, 0.0001,
            ],
            counts: [0, 0, 2, 2, 100, 50, 0, 35, 10, 40],
            flags: [0, 0, 0],
            nuser: [0, 0, 0],
        }),
    ),
    // a collision before the link's visual: no colour; one after it: the visual's
    (
        r#"<robot name="r"><link name="base"><inertial><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial><collision><origin xyz="0 0 0.3"/><geometry><box size="0.1 0.2 0.3"/></geometry></collision><visual><origin xyz="0 0 0.2"/><geometry><box size="0.05 0.05 0.05"/></geometry><material name="red"><color rgba="1 0 0 1"/></material></visual><collision><origin xyz="0 0 0"/><geometry><box size="0.1 0.2 0.3"/></geometry></collision></link><link name="arm"><inertial><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial><collision><origin xyz="0 0 0.1"/><geometry><box size="0.1 0.2 0.3"/></geometry></collision></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5" rpy="0.3 0 0"/><axis xyz="0 1 0"/><limit lower="-1" upper="1" effort="2" velocity="1"/></joint></robot>"#,
        Expect::Loads(&Model {
            name: "r",
            body_parent: &[0, 0],
            body_mass: &[0.0, 1.0],
            body_inertia: &[[0.0, 0.0, 0.0], [0.1, 0.1, 0.1]],
            body_ipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            body_iquat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            geom_name: &["", "", ""],
            geom_type: &["Box", "Box", "Box"],
            geom_body: &[0, 0, 1],
            geom_pos: &[[0.0, 0.0, 0.3], [0.0, 0.0, 0.0], [0.0, 0.0, 0.1]],
            geom_size: &[[0.05, 0.1, 0.15], [0.05, 0.1, 0.15], [0.05, 0.1, 0.15]],
            geom_contype: &[1, 1, 1],
            geom_conaffinity: &[1, 1, 1],
            geom_group: &[0, 0, 0],
            geom_rgba: &[
                [0.5, 0.5, 0.5, 1.0],
                [1.0, 0.0, 0.0, 1.0],
                [0.5, 0.5, 0.5, 1.0],
            ],
            jnt_limited: &[true],
            jnt_range: &[[-1.0, 1.0]],
            key_qpos: &[],
            qpos5: &[0.0],
            reals: [0.002, 1.0, 1e-08, 0.01, 1e-06, 1e-06, 0.0, 0.0, 0.0],
            vectors: [0.0, 0.0, -9.81, 0.0, 0.0, 0.0, 0.0, -0.5, 0.0],
            contact_override: [
                0.02, 1.0, 0.9, 0.95, 0.001, 0.5, 2.0, 1.0, 1.0, 0.005, 0.0001, 0.0001,
            ],
            counts: [0, 0, 2, 2, 100, 50, 0, 35, 10, 40],
            flags: [0, 0, 0],
            nuser: [0, 0, 0],
        }),
    ),
    // a visual without a material keeps the link's colour
    (
        r#"<robot name="r"><link name="base"><inertial><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial><collision><origin xyz="0 0 0"/><geometry><box size="0.1 0.2 0.3"/></geometry></collision></link><link name="arm"><inertial><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial><visual><origin xyz="0 0 0.2"/><geometry><box size="0.05 0.05 0.05"/></geometry><material name="red"><color rgba="1 0 0 1"/></material></visual><visual><origin xyz="0 0 0.3"/><geometry><box size="0.05 0.05 0.05"/></geometry></visual><collision><origin xyz="0 0 0.1"/><geometry><box size="0.1 0.2 0.3"/></geometry></collision></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5" rpy="0.3 0 0"/><axis xyz="0 1 0"/><limit lower="-1" upper="1" effort="2" velocity="1"/></joint></robot>"#,
        Expect::Loads(&Model {
            name: "r",
            body_parent: &[0, 0],
            body_mass: &[0.0, 1.0],
            body_inertia: &[[0.0, 0.0, 0.0], [0.1, 0.1, 0.1]],
            body_ipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            body_iquat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            geom_name: &["", ""],
            geom_type: &["Box", "Box"],
            geom_body: &[0, 1],
            geom_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.1]],
            geom_size: &[[0.05, 0.1, 0.15], [0.05, 0.1, 0.15]],
            geom_contype: &[1, 1],
            geom_conaffinity: &[1, 1],
            geom_group: &[0, 0],
            geom_rgba: &[[0.5, 0.5, 0.5, 1.0], [1.0, 0.0, 0.0, 1.0]],
            jnt_limited: &[true],
            jnt_range: &[[-1.0, 1.0]],
            key_qpos: &[],
            qpos5: &[0.0],
            reals: [0.002, 1.0, 1e-08, 0.01, 1e-06, 1e-06, 0.0, 0.0, 0.0],
            vectors: [0.0, 0.0, -9.81, 0.0, 0.0, 0.0, 0.0, -0.5, 0.0],
            contact_override: [
                0.02, 1.0, 0.9, 0.95, 0.001, 0.5, 2.0, 1.0, 1.0, 0.005, 0.0001, 0.0001,
            ],
            counts: [0, 0, 2, 2, 100, 50, 0, 35, 10, 40],
            flags: [0, 0, 0],
            nuser: [0, 0, 0],
        }),
    ),
    // a visual's own named material is in the table for every link
    (
        r#"<robot name="r"><link name="base"><inertial><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial><visual><origin xyz="0 0 0.2"/><geometry><box size="0.05 0.05 0.05"/></geometry><material name="red"/></visual><collision><origin xyz="0 0 0"/><geometry><box size="0.1 0.2 0.3"/></geometry></collision></link><link name="arm"><inertial><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial><visual><origin xyz="0 0 0.2"/><geometry><box size="0.05 0.05 0.05"/></geometry><material name="red"><color rgba="1 0 0 1"/></material></visual><collision><origin xyz="0 0 0.1"/><geometry><box size="0.1 0.2 0.3"/></geometry></collision></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5" rpy="0.3 0 0"/><axis xyz="0 1 0"/><limit lower="-1" upper="1" effort="2" velocity="1"/></joint></robot>"#,
        Expect::Loads(&Model {
            name: "r",
            body_parent: &[0, 0],
            body_mass: &[0.0, 1.0],
            body_inertia: &[[0.0, 0.0, 0.0], [0.1, 0.1, 0.1]],
            body_ipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            body_iquat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            geom_name: &["", ""],
            geom_type: &["Box", "Box"],
            geom_body: &[0, 1],
            geom_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.1]],
            geom_size: &[[0.05, 0.1, 0.15], [0.05, 0.1, 0.15]],
            geom_contype: &[1, 1],
            geom_conaffinity: &[1, 1],
            geom_group: &[0, 0],
            geom_rgba: &[[1.0, 0.0, 0.0, 1.0], [1.0, 0.0, 0.0, 1.0]],
            jnt_limited: &[true],
            jnt_range: &[[-1.0, 1.0]],
            key_qpos: &[],
            qpos5: &[0.0],
            reals: [0.002, 1.0, 1e-08, 0.01, 1e-06, 1e-06, 0.0, 0.0, 0.0],
            vectors: [0.0, 0.0, -9.81, 0.0, 0.0, 0.0, 0.0, -0.5, 0.0],
            contact_override: [
                0.02, 1.0, 0.9, 0.95, 0.001, 0.5, 2.0, 1.0, 1.0, 0.005, 0.0001, 0.0001,
            ],
            counts: [0, 0, 2, 2, 100, 50, 0, 35, 10, 40],
            flags: [0, 0, 0],
            nuser: [0, 0, 0],
        }),
    ),
    // two table materials of one name: the first
    (
        r#"<robot name="r"><link name="base"><inertial><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial><collision><origin xyz="0 0 0"/><geometry><box size="0.1 0.2 0.3"/></geometry></collision></link><link name="arm"><inertial><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial><visual><origin xyz="0 0 0.2"/><geometry><box size="0.05 0.05 0.05"/></geometry><material name="c"/></visual><collision><origin xyz="0 0 0.1"/><geometry><box size="0.1 0.2 0.3"/></geometry></collision></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5" rpy="0.3 0 0"/><axis xyz="0 1 0"/><limit lower="-1" upper="1" effort="2" velocity="1"/></joint><material name="c"><color rgba="1 1 0 1"/></material><material name="c"><color rgba="0 1 1 1"/></material></robot>"#,
        Expect::Loads(&Model {
            name: "r",
            body_parent: &[0, 0],
            body_mass: &[0.0, 1.0],
            body_inertia: &[[0.0, 0.0, 0.0], [0.1, 0.1, 0.1]],
            body_ipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            body_iquat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            geom_name: &["", ""],
            geom_type: &["Box", "Box"],
            geom_body: &[0, 1],
            geom_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.1]],
            geom_size: &[[0.05, 0.1, 0.15], [0.05, 0.1, 0.15]],
            geom_contype: &[1, 1],
            geom_conaffinity: &[1, 1],
            geom_group: &[0, 0],
            geom_rgba: &[[0.5, 0.5, 0.5, 1.0], [1.0, 1.0, 0.0, 1.0]],
            jnt_limited: &[true],
            jnt_range: &[[-1.0, 1.0]],
            key_qpos: &[],
            qpos5: &[0.0],
            reals: [0.002, 1.0, 1e-08, 0.01, 1e-06, 1e-06, 0.0, 0.0, 0.0],
            vectors: [0.0, 0.0, -9.81, 0.0, 0.0, 0.0, 0.0, -0.5, 0.0],
            contact_override: [
                0.02, 1.0, 0.9, 0.95, 0.001, 0.5, 2.0, 1.0, 1.0, 0.005, 0.0001, 0.0001,
            ],
            counts: [0, 0, 2, 2, 100, 50, 0, 35, 10, 40],
            flags: [0, 0, 0],
            nuser: [0, 0, 0],
        }),
    ),
    // a colour set in one link is not another's
    (
        r#"<robot name="r"><link name="base"><inertial><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial><visual><origin xyz="0 0 0.2"/><geometry><box size="0.05 0.05 0.05"/></geometry><material name="red"><color rgba="1 0 0 1"/></material></visual><collision><origin xyz="0 0 0"/><geometry><box size="0.1 0.2 0.3"/></geometry></collision></link><link name="arm"><inertial><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial><collision><origin xyz="0 0 0.1"/><geometry><box size="0.1 0.2 0.3"/></geometry></collision></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5" rpy="0.3 0 0"/><axis xyz="0 1 0"/><limit lower="-1" upper="1" effort="2" velocity="1"/></joint></robot>"#,
        Expect::Loads(&Model {
            name: "r",
            body_parent: &[0, 0],
            body_mass: &[0.0, 1.0],
            body_inertia: &[[0.0, 0.0, 0.0], [0.1, 0.1, 0.1]],
            body_ipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            body_iquat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            geom_name: &["", ""],
            geom_type: &["Box", "Box"],
            geom_body: &[0, 1],
            geom_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.1]],
            geom_size: &[[0.05, 0.1, 0.15], [0.05, 0.1, 0.15]],
            geom_contype: &[1, 1],
            geom_conaffinity: &[1, 1],
            geom_group: &[0, 0],
            geom_rgba: &[[1.0, 0.0, 0.0, 1.0], [0.5, 0.5, 0.5, 1.0]],
            jnt_limited: &[true],
            jnt_range: &[[-1.0, 1.0]],
            key_qpos: &[],
            qpos5: &[0.0],
            reals: [0.002, 1.0, 1e-08, 0.01, 1e-06, 1e-06, 0.0, 0.0, 0.0],
            vectors: [0.0, 0.0, -9.81, 0.0, 0.0, 0.0, 0.0, -0.5, 0.0],
            contact_override: [
                0.02, 1.0, 0.9, 0.95, 0.001, 0.5, 2.0, 1.0, 1.0, 0.005, 0.0001, 0.0001,
            ],
            counts: [0, 0, 2, 2, 100, 50, 0, 35, 10, 40],
            flags: [0, 0, 0],
            nuser: [0, 0, 0],
        }),
    ),
    // a table material without a colour is not in the table
    (
        r#"<robot name="r"><link name="base"><inertial><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial><collision><origin xyz="0 0 0"/><geometry><box size="0.1 0.2 0.3"/></geometry></collision></link><link name="arm"><inertial><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial><visual><origin xyz="0 0 0.2"/><geometry><box size="0.05 0.05 0.05"/></geometry><material name="tex"/></visual><collision><origin xyz="0 0 0.1"/><geometry><box size="0.1 0.2 0.3"/></geometry></collision></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5" rpy="0.3 0 0"/><axis xyz="0 1 0"/><limit lower="-1" upper="1" effort="2" velocity="1"/></joint><material name="blue"><color rgba="0 0 1 1"/></material><material name="tex"><texture filename="t.png"/></material></robot>"#,
        Expect::Loads(&Model {
            name: "r",
            body_parent: &[0, 0],
            body_mass: &[0.0, 1.0],
            body_inertia: &[[0.0, 0.0, 0.0], [0.1, 0.1, 0.1]],
            body_ipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            body_iquat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            geom_name: &["", ""],
            geom_type: &["Box", "Box"],
            geom_body: &[0, 1],
            geom_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.1]],
            geom_size: &[[0.05, 0.1, 0.15], [0.05, 0.1, 0.15]],
            geom_contype: &[1, 1],
            geom_conaffinity: &[1, 1],
            geom_group: &[0, 0],
            geom_rgba: &[[0.5, 0.5, 0.5, 1.0], [0.5, 0.5, 0.5, 1.0]],
            jnt_limited: &[true],
            jnt_range: &[[-1.0, 1.0]],
            key_qpos: &[],
            qpos5: &[0.0],
            reals: [0.002, 1.0, 1e-08, 0.01, 1e-06, 1e-06, 0.0, 0.0, 0.0],
            vectors: [0.0, 0.0, -9.81, 0.0, 0.0, 0.0, 0.0, -0.5, 0.0],
            contact_override: [
                0.02, 1.0, 0.9, 0.95, 0.001, 0.5, 2.0, 1.0, 1.0, 0.005, 0.0001, 0.0001,
            ],
            counts: [0, 0, 2, 2, 100, 50, 0, 35, 10, 40],
            flags: [0, 0, 0],
            nuser: [0, 0, 0],
        }),
    ),
    // a colour outside [0, 1], as written
    (
        r#"<robot name="r"><link name="base"><inertial><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial><collision><origin xyz="0 0 0"/><geometry><box size="0.1 0.2 0.3"/></geometry></collision></link><link name="arm"><inertial><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial><visual><origin xyz="0 0 0.2"/><geometry><box size="0.05 0.05 0.05"/></geometry><material name="x"><color rgba="2 0.5 -1 0.5"/></material></visual><collision><origin xyz="0 0 0.1"/><geometry><box size="0.1 0.2 0.3"/></geometry></collision></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5" rpy="0.3 0 0"/><axis xyz="0 1 0"/><limit lower="-1" upper="1" effort="2" velocity="1"/></joint></robot>"#,
        Expect::Loads(&Model {
            name: "r",
            body_parent: &[0, 0],
            body_mass: &[0.0, 1.0],
            body_inertia: &[[0.0, 0.0, 0.0], [0.1, 0.1, 0.1]],
            body_ipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            body_iquat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            geom_name: &["", ""],
            geom_type: &["Box", "Box"],
            geom_body: &[0, 1],
            geom_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.1]],
            geom_size: &[[0.05, 0.1, 0.15], [0.05, 0.1, 0.15]],
            geom_contype: &[1, 1],
            geom_conaffinity: &[1, 1],
            geom_group: &[0, 0],
            geom_rgba: &[[0.5, 0.5, 0.5, 1.0], [2.0, 0.5, -1.0, 0.5]],
            jnt_limited: &[true],
            jnt_range: &[[-1.0, 1.0]],
            key_qpos: &[],
            qpos5: &[0.0],
            reals: [0.002, 1.0, 1e-08, 0.01, 1e-06, 1e-06, 0.0, 0.0, 0.0],
            vectors: [0.0, 0.0, -9.81, 0.0, 0.0, 0.0, 0.0, -0.5, 0.0],
            contact_override: [
                0.02, 1.0, 0.9, 0.95, 0.001, 0.5, 2.0, 1.0, 1.0, 0.005, 0.0001, 0.0001,
            ],
            counts: [0, 0, 2, 2, 100, 50, 0, 35, 10, 40],
            flags: [0, 0, 0],
            nuser: [0, 0, 0],
        }),
    ),
    // a table material written inside a link
    (
        r#"<robot name="r"><link name="base"><inertial><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial><material name="deep"><color rgba="0.2 0.4 0.6 1"/></material><collision><origin xyz="0 0 0"/><geometry><box size="0.1 0.2 0.3"/></geometry></collision></link><link name="arm"><inertial><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial><visual><origin xyz="0 0 0.2"/><geometry><box size="0.05 0.05 0.05"/></geometry><material name="deep"/></visual><collision><origin xyz="0 0 0.1"/><geometry><box size="0.1 0.2 0.3"/></geometry></collision></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5" rpy="0.3 0 0"/><axis xyz="0 1 0"/><limit lower="-1" upper="1" effort="2" velocity="1"/></joint></robot>"#,
        Expect::Loads(&Model {
            name: "r",
            body_parent: &[0, 0],
            body_mass: &[0.0, 1.0],
            body_inertia: &[[0.0, 0.0, 0.0], [0.1, 0.1, 0.1]],
            body_ipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            body_iquat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            geom_name: &["", ""],
            geom_type: &["Box", "Box"],
            geom_body: &[0, 1],
            geom_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.1]],
            geom_size: &[[0.05, 0.1, 0.15], [0.05, 0.1, 0.15]],
            geom_contype: &[1, 1],
            geom_conaffinity: &[1, 1],
            geom_group: &[0, 0],
            geom_rgba: &[
                [0.5, 0.5, 0.5, 1.0],
                [
                    0.20000000298023224,
                    0.4000000059604645,
                    0.6000000238418579,
                    1.0,
                ],
            ],
            jnt_limited: &[true],
            jnt_range: &[[-1.0, 1.0]],
            key_qpos: &[],
            qpos5: &[0.0],
            reals: [0.002, 1.0, 1e-08, 0.01, 1e-06, 1e-06, 0.0, 0.0, 0.0],
            vectors: [0.0, 0.0, -9.81, 0.0, 0.0, 0.0, 0.0, -0.5, 0.0],
            contact_override: [
                0.02, 1.0, 0.9, 0.95, 0.001, 0.5, 2.0, 1.0, 1.0, 0.005, 0.0001, 0.0001,
            ],
            counts: [0, 0, 2, 2, 100, 50, 0, 35, 10, 40],
            flags: [0, 0, 0],
            nuser: [0, 0, 0],
        }),
    ),
    // a visual's colour over its material's table colour
    (
        r#"<robot name="r"><link name="base"><inertial><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial><collision><origin xyz="0 0 0"/><geometry><box size="0.1 0.2 0.3"/></geometry></collision></link><link name="arm"><inertial><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial><visual><origin xyz="0 0 0.2"/><geometry><box size="0.05 0.05 0.05"/></geometry><material name="blue"><color rgba="1 0 1 1"/></material></visual><collision><origin xyz="0 0 0.1"/><geometry><box size="0.1 0.2 0.3"/></geometry></collision></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5" rpy="0.3 0 0"/><axis xyz="0 1 0"/><limit lower="-1" upper="1" effort="2" velocity="1"/></joint><material name="blue"><color rgba="0 0 1 1"/></material><material name="tex"><texture filename="t.png"/></material></robot>"#,
        Expect::Loads(&Model {
            name: "r",
            body_parent: &[0, 0],
            body_mass: &[0.0, 1.0],
            body_inertia: &[[0.0, 0.0, 0.0], [0.1, 0.1, 0.1]],
            body_ipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            body_iquat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            geom_name: &["", ""],
            geom_type: &["Box", "Box"],
            geom_body: &[0, 1],
            geom_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.1]],
            geom_size: &[[0.05, 0.1, 0.15], [0.05, 0.1, 0.15]],
            geom_contype: &[1, 1],
            geom_conaffinity: &[1, 1],
            geom_group: &[0, 0],
            geom_rgba: &[[0.5, 0.5, 0.5, 1.0], [1.0, 0.0, 1.0, 1.0]],
            jnt_limited: &[true],
            jnt_range: &[[-1.0, 1.0]],
            key_qpos: &[],
            qpos5: &[0.0],
            reals: [0.002, 1.0, 1e-08, 0.01, 1e-06, 1e-06, 0.0, 0.0, 0.0],
            vectors: [0.0, 0.0, -9.81, 0.0, 0.0, 0.0, 0.0, -0.5, 0.0],
            contact_override: [
                0.02, 1.0, 0.9, 0.95, 0.001, 0.5, 2.0, 1.0, 1.0, 0.005, 0.0001, 0.0001,
            ],
            counts: [0, 0, 2, 2, 100, 50, 0, 35, 10, 40],
            flags: [0, 0, 0],
            nuser: [0, 0, 0],
        }),
    ),
    // a table material without a name is not in the table
    (
        r#"<robot name="r"><link name="base"><inertial><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial><collision><origin xyz="0 0 0"/><geometry><box size="0.1 0.2 0.3"/></geometry></collision></link><link name="arm"><inertial><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial><visual><origin xyz="0 0 0.2"/><geometry><box size="0.05 0.05 0.05"/></geometry><material name="red"><color rgba="1 0 0 1"/></material></visual><collision><origin xyz="0 0 0.1"/><geometry><box size="0.1 0.2 0.3"/></geometry></collision></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5" rpy="0.3 0 0"/><axis xyz="0 1 0"/><limit lower="-1" upper="1" effort="2" velocity="1"/></joint><material><color rgba="1 1 1 1"/></material></robot>"#,
        Expect::Loads(&Model {
            name: "r",
            body_parent: &[0, 0],
            body_mass: &[0.0, 1.0],
            body_inertia: &[[0.0, 0.0, 0.0], [0.1, 0.1, 0.1]],
            body_ipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            body_iquat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            geom_name: &["", ""],
            geom_type: &["Box", "Box"],
            geom_body: &[0, 1],
            geom_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.1]],
            geom_size: &[[0.05, 0.1, 0.15], [0.05, 0.1, 0.15]],
            geom_contype: &[1, 1],
            geom_conaffinity: &[1, 1],
            geom_group: &[0, 0],
            geom_rgba: &[[0.5, 0.5, 0.5, 1.0], [1.0, 0.0, 0.0, 1.0]],
            jnt_limited: &[true],
            jnt_range: &[[-1.0, 1.0]],
            key_qpos: &[],
            qpos5: &[0.0],
            reals: [0.002, 1.0, 1e-08, 0.01, 1e-06, 1e-06, 0.0, 0.0, 0.0],
            vectors: [0.0, 0.0, -9.81, 0.0, 0.0, 0.0, 0.0, -0.5, 0.0],
            contact_override: [
                0.02, 1.0, 0.9, 0.95, 0.001, 0.5, 2.0, 1.0, 1.0, 0.005, 0.0001, 0.0001,
            ],
            counts: [0, 0, 2, 2, 100, 50, 0, 35, 10, 40],
            flags: [0, 0, 0],
            nuser: [0, 0, 0],
        }),
    ),
    // RL-10c's g01
    (
        r#"<robot name="r"><link name="base"><inertial><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="arm"><inertial><mass value="1"/><inertia ixx="0.1" iyy="0.2" izz="0.3" ixy="0" ixz="0" iyz="0"/></inertial><visual><origin xyz="0 0 0.1"/><geometry><sphere radius="0.05"/></geometry><material name="red"><color rgba="1 0 0 1"/></material></visual><collision><geometry><box size="0.1 0.2 0.3"/></geometry></collision></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5"/><axis xyz="0 0 1"/></joint></robot>"#,
        Expect::Loads(&Model {
            name: "r",
            body_parent: &[0, 0],
            body_mass: &[0.0, 1.0],
            body_inertia: &[[0.0, 0.0, 0.0], [0.3, 0.2, 0.1]],
            body_ipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            body_iquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 0.7071067811865489, 0.0, 0.7071067811865489],
            ],
            geom_name: &[""],
            geom_type: &["Box"],
            geom_body: &[1],
            geom_pos: &[[0.0, 0.0, 0.0]],
            geom_size: &[[0.05, 0.1, 0.15]],
            geom_contype: &[1],
            geom_conaffinity: &[1],
            geom_group: &[0],
            geom_rgba: &[[1.0, 0.0, 0.0, 1.0]],
            jnt_limited: &[false],
            jnt_range: &[[0.0, 0.0]],
            key_qpos: &[],
            qpos5: &[0.0],
            reals: [0.002, 1.0, 1e-08, 0.01, 1e-06, 1e-06, 0.0, 0.0, 0.0],
            vectors: [0.0, 0.0, -9.81, 0.0, 0.0, 0.0, 0.0, -0.5, 0.0],
            contact_override: [
                0.02, 1.0, 0.9, 0.95, 0.001, 0.5, 2.0, 1.0, 1.0, 0.005, 0.0001, 0.0001,
            ],
            counts: [0, 0, 2, 2, 100, 50, 0, 35, 10, 40],
            flags: [0, 0, 0],
            nuser: [0, 0, 0],
        }),
    ),
    // RL-10c's g02
    (
        r#"<robot name="r"><link name="base"><inertial><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="arm"><inertial><mass value="1"/><inertia ixx="0.1" iyy="0.2" izz="0.3" ixy="0" ixz="0" iyz="0"/></inertial><collision><geometry><box size="0.1 0.2 0.3"/></geometry></collision><visual><origin xyz="0 0 0.1"/><geometry><sphere radius="0.05"/></geometry><material name="red"><color rgba="1 0 0 1"/></material></visual><collision><origin xyz="0.1 0 0"/><geometry><box size="0.1 0.2 0.3"/></geometry></collision></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5"/><axis xyz="0 0 1"/></joint></robot>"#,
        Expect::Loads(&Model {
            name: "r",
            body_parent: &[0, 0],
            body_mass: &[0.0, 1.0],
            body_inertia: &[[0.0, 0.0, 0.0], [0.3, 0.2, 0.1]],
            body_ipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            body_iquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 0.7071067811865489, 0.0, 0.7071067811865489],
            ],
            geom_name: &["", ""],
            geom_type: &["Box", "Box"],
            geom_body: &[1, 1],
            geom_pos: &[[0.0, 0.0, 0.0], [0.1, 0.0, 0.0]],
            geom_size: &[[0.05, 0.1, 0.15], [0.05, 0.1, 0.15]],
            geom_contype: &[1, 1],
            geom_conaffinity: &[1, 1],
            geom_group: &[0, 0],
            geom_rgba: &[[0.5, 0.5, 0.5, 1.0], [1.0, 0.0, 0.0, 1.0]],
            jnt_limited: &[false],
            jnt_range: &[[0.0, 0.0]],
            key_qpos: &[],
            qpos5: &[0.0],
            reals: [0.002, 1.0, 1e-08, 0.01, 1e-06, 1e-06, 0.0, 0.0, 0.0],
            vectors: [0.0, 0.0, -9.81, 0.0, 0.0, 0.0, 0.0, -0.5, 0.0],
            contact_override: [
                0.02, 1.0, 0.9, 0.95, 0.001, 0.5, 2.0, 1.0, 1.0, 0.005, 0.0001, 0.0001,
            ],
            counts: [0, 0, 2, 2, 100, 50, 0, 35, 10, 40],
            flags: [0, 0, 0],
            nuser: [0, 0, 0],
        }),
    ),
    // RL-10c's g03
    (
        r#"<robot name="r"><material name="blue"><color rgba="0 0 1 1"/></material><link name="base"><inertial><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="arm"><inertial><mass value="1"/><inertia ixx="0.1" iyy="0.2" izz="0.3" ixy="0" ixz="0" iyz="0"/></inertial><visual><geometry><sphere radius="0.05"/></geometry><material name="blue"/></visual><collision><geometry><box size="0.1 0.2 0.3"/></geometry></collision></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5"/><axis xyz="0 0 1"/></joint></robot>"#,
        Expect::Loads(&Model {
            name: "r",
            body_parent: &[0, 0],
            body_mass: &[0.0, 1.0],
            body_inertia: &[[0.0, 0.0, 0.0], [0.3, 0.2, 0.1]],
            body_ipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            body_iquat: &[
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 0.7071067811865489, 0.0, 0.7071067811865489],
            ],
            geom_name: &[""],
            geom_type: &["Box"],
            geom_body: &[1],
            geom_pos: &[[0.0, 0.0, 0.0]],
            geom_size: &[[0.05, 0.1, 0.15]],
            geom_contype: &[1],
            geom_conaffinity: &[1],
            geom_group: &[0],
            geom_rgba: &[[0.0, 0.0, 1.0, 1.0]],
            jnt_limited: &[false],
            jnt_range: &[[0.0, 0.0]],
            key_qpos: &[],
            qpos5: &[0.0],
            reals: [0.002, 1.0, 1e-08, 0.01, 1e-06, 1e-06, 0.0, 0.0, 0.0],
            vectors: [0.0, 0.0, -9.81, 0.0, 0.0, 0.0, 0.0, -0.5, 0.0],
            contact_override: [
                0.02, 1.0, 0.9, 0.95, 0.001, 0.5, 2.0, 1.0, 1.0, 0.005, 0.0001, 0.0001,
            ],
            counts: [0, 0, 2, 2, 100, 50, 0, 35, 10, 40],
            flags: [0, 0, 0],
            nuser: [0, 0, 0],
        }),
    ),
    // a robot with no colour: the default
    (
        r#"<robot name="r"><link name="base"><inertial><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial><collision><origin xyz="0 0 0"/><geometry><box size="0.1 0.2 0.3"/></geometry></collision></link><link name="arm"><inertial><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial><collision><origin xyz="0 0 0.1"/><geometry><box size="0.1 0.2 0.3"/></geometry></collision></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5" rpy="0.3 0 0"/><axis xyz="0 1 0"/><limit lower="-1" upper="1" effort="2" velocity="1"/></joint></robot>"#,
        Expect::Loads(&Model {
            name: "r",
            body_parent: &[0, 0],
            body_mass: &[0.0, 1.0],
            body_inertia: &[[0.0, 0.0, 0.0], [0.1, 0.1, 0.1]],
            body_ipos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            body_iquat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            geom_name: &["", ""],
            geom_type: &["Box", "Box"],
            geom_body: &[0, 1],
            geom_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.1]],
            geom_size: &[[0.05, 0.1, 0.15], [0.05, 0.1, 0.15]],
            geom_contype: &[1, 1],
            geom_conaffinity: &[1, 1],
            geom_group: &[0, 0],
            geom_rgba: &[[0.5, 0.5, 0.5, 1.0], [0.5, 0.5, 0.5, 1.0]],
            jnt_limited: &[true],
            jnt_range: &[[-1.0, 1.0]],
            key_qpos: &[],
            qpos5: &[0.0],
            reals: [0.002, 1.0, 1e-08, 0.01, 1e-06, 1e-06, 0.0, 0.0, 0.0],
            vectors: [0.0, 0.0, -9.81, 0.0, 0.0, 0.0, 0.0, -0.5, 0.0],
            contact_override: [
                0.02, 1.0, 0.9, 0.95, 0.001, 0.5, 2.0, 1.0, 1.0, 0.005, 0.0001, 0.0001,
            ],
            counts: [0, 0, 2, 2, 100, 50, 0, 35, 10, 40],
            flags: [0, 0, 0],
            nuser: [0, 0, 0],
        }),
    ),
    // MuJoCo: "XML Error: required attribute missing: 'rgba'"
    (
        r#"<robot name="r"><link name="base"><inertial><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial><collision><origin xyz="0 0 0"/><geometry><box size="0.1 0.2 0.3"/></geometry></collision></link><link name="arm"><inertial><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial><visual><origin xyz="0 0 0.2"/><geometry><box size="0.05 0.05 0.05"/></geometry><material name="x"><color/></material></visual><collision><origin xyz="0 0 0.1"/><geometry><box size="0.1 0.2 0.3"/></geometry></collision></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5" rpy="0.3 0 0"/><axis xyz="0 1 0"/><limit lower="-1" upper="1" effort="2" velocity="1"/></joint></robot>"#,
        Expect::Refused("missing required attribute: rgba on color"),
    ),
    // MuJoCo: "XML Error: attribute 'rgba' does not have enough data"
    (
        r#"<robot name="r"><link name="base"><inertial><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial><collision><origin xyz="0 0 0"/><geometry><box size="0.1 0.2 0.3"/></geometry></collision></link><link name="arm"><inertial><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial><visual><origin xyz="0 0 0.2"/><geometry><box size="0.05 0.05 0.05"/></geometry><material name="x"><color rgba="1 0 0"/></material></visual><collision><origin xyz="0 0 0.1"/><geometry><box size="0.1 0.2 0.3"/></geometry></collision></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5" rpy="0.3 0 0"/><axis xyz="0 1 0"/><limit lower="-1" upper="1" effort="2" velocity="1"/></joint></robot>"#,
        Expect::Refused("does not have enough data"),
    ),
    // MuJoCo: "XML Error: required attribute missing: 'name'"
    (
        r#"<robot name="r"><link name="base"><inertial><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial><collision><origin xyz="0 0 0"/><geometry><box size="0.1 0.2 0.3"/></geometry></collision></link><link name="arm"><inertial><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial><visual><origin xyz="0 0 0.2"/><geometry><box size="0.05 0.05 0.05"/></geometry><material/></visual><collision><origin xyz="0 0 0.1"/><geometry><box size="0.1 0.2 0.3"/></geometry></collision></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5" rpy="0.3 0 0"/><axis xyz="0 1 0"/><limit lower="-1" upper="1" effort="2" velocity="1"/></joint></robot>"#,
        Expect::Refused("missing required attribute: name on material"),
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

/// sim-core's integrator, cone, Jacobian and solver as MuJoCo numbers them
/// (`mjtIntegrator`, `mjtCone`, `mjtJacobian`, `mjtSolver`); an integrator
/// MuJoCo has no number for is -1.
fn option_numbers(m: &sim_core::Model) -> [i64; 4] {
    let integrator = match m.integrator {
        sim_core::Integrator::Euler => 0,
        sim_core::Integrator::RungeKutta4 => 1,
        sim_core::Integrator::Implicit => 2,
        sim_core::Integrator::ImplicitFast => 3,
        // `ImplicitSpringDamper`, and any later one (the enum is non-exhaustive).
        _ => -1,
    };
    let jacobian = match m.jacobian {
        sim_core::JacobianType::Dense => 0,
        sim_core::JacobianType::Sparse => 1,
        sim_core::JacobianType::Auto => 2,
    };
    let solver = match m.solver_type {
        sim_core::SolverType::PGS => 0,
        sim_core::SolverType::CG => 1,
        sim_core::SolverType::Newton => 2,
    };
    [integrator, i64::from(m.cone), jacobian, solver]
}

/// `m`'s options as [`Model`] holds them: its reals, its vectors, its
/// contact override and its counts.
fn options(m: &sim_core::Model) -> ([f64; 9], Vec<f64>, Vec<f64>, [i64; 10]) {
    let reals = [
        m.timestep,
        m.impratio,
        m.solver_tolerance,
        m.ls_tolerance,
        m.noslip_tolerance,
        m.ccd_tolerance,
        m.density,
        m.viscosity,
        m.o_margin,
    ];
    let vectors = [m.gravity, m.wind, m.magnetic]
        .iter()
        .flat_map(|v| v.iter().copied())
        .collect();
    let contact = m
        .o_solref
        .iter()
        .chain(&m.o_solimp)
        .chain(&m.o_friction)
        .copied()
        .collect();
    let [integrator, cone, jacobian, solver] = option_numbers(m);
    let count = |n: usize| i64::try_from(n).unwrap();
    let counts = [
        integrator,
        cone,
        jacobian,
        solver,
        count(m.solver_iterations),
        count(m.ls_iterations),
        count(m.noslip_iterations),
        count(m.ccd_iterations),
        count(m.sdf_iterations),
        count(m.sdf_initpoints),
    ];
    (reals, vectors, contact, counts)
}

/// The fields of `m` that are not MuJoCo 3.5.0's `want`, five steps run.
fn differing(m: &sim_core::Model, want: &Model) -> Vec<&'static str> {
    let mut d = m.make_data();
    for _ in 0..5 {
        d.step(m).unwrap();
    }
    let range: Vec<f64> = m
        .jnt_range
        .iter()
        .flat_map(|&bounds| <[f64; 2]>::from(bounds))
        .collect();
    let rgba: Vec<f64> = m.geom_rgba.iter().flatten().copied().collect();
    let names: Vec<&str> = m
        .geom_name
        .iter()
        .map(|n| n.as_deref().unwrap_or(""))
        .collect();
    let types: Vec<String> = m.geom_type.iter().map(|t| format!("{t:?}")).collect();
    let keys: Vec<f64> = m
        .keyframes
        .iter()
        .flat_map(|k| k.qpos.iter().copied())
        .collect();
    let (reals, vectors, contact, counts) = options(m);
    [
        ("name", m.name == want.name),
        ("body_parent", m.body_parent == want.body_parent),
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
        ("geom_name", names == want.geom_name),
        ("geom_type", types == want.geom_type),
        ("geom_body", m.geom_body == want.geom_body),
        (
            "geom_pos",
            close(&flat3(&m.geom_pos), want.geom_pos.as_flattened()),
        ),
        (
            "geom_size",
            close(&flat3(&m.geom_size), want.geom_size.as_flattened()),
        ),
        ("geom_contype", m.geom_contype == want.geom_contype),
        (
            "geom_conaffinity",
            m.geom_conaffinity == want.geom_conaffinity,
        ),
        ("geom_group", m.geom_group == want.geom_group),
        ("geom_rgba", close(&rgba, want.geom_rgba.as_flattened())),
        ("jnt_limited", m.jnt_limited == want.jnt_limited),
        ("jnt_range", close(&range, want.jnt_range.as_flattened())),
        ("key_qpos", close(&keys, want.key_qpos)),
        ("qpos5", close(d.qpos.as_slice(), want.qpos5)),
        ("reals", close(&reals, &want.reals)),
        ("vectors", close(&vectors, &want.vectors)),
        ("contact_override", close(&contact, &want.contact_override)),
        ("counts", counts == want.counts),
        (
            "flags",
            [m.enableflags, m.disableflags, m.disableactuator] == want.flags,
        ),
        (
            "nuser",
            [m.nuser_body, m.nuser_jnt, m.nuser_geom] == want.nuser,
        ),
    ]
    .into_iter()
    .filter(|&(_, same)| !same)
    .map(|(field, _)| field)
    .collect()
}

/// Each document's name, colours and the fields above, and five steps, as
/// MuJoCo 3.5.0's URDF import makes them; each document MuJoCo refuses
/// refused for its fault.
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
