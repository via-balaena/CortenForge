//! A URDF's reader and its refusals, as MuJoCo 3.5.0's own importer reads
//! it (`xml_urdf.cc`, `xml.cc`): a `spherical` joint as a ball joint
//! (`:42`, `:479-484`), and a robot without links as an empty world; the
//! compiler's refusals of the model and of a `<mujoco>` section, each in its
//! own words; and the XML outside attribute text as tinyxml2 reads it (the
//! root's name in any case, a declaration only before every other node, a
//! document ending in text refused, an `&` that starts no reference kept).
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
    /// (w, x, y, z), as each quaternion below.
    body_quat: &'static [[f64; 4]],
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
    geom_quat: &'static [[f64; 4]],
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
    /// `noslip_tolerance`, `ccd_tolerance`, `sleep_tolerance`, `density`,
    /// `viscosity`, `o_margin`.
    reals: [f64; 10],
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
    /// `nuser_body`, `nuser_jnt`, `nuser_geom`, `nuser_site`, `nuser_tendon`,
    /// `nuser_actuator`, `nuser_sensor`.
    nuser: [i32; 7],
}

/// What MuJoCo 3.5.0 makes of a document.
enum Expect {
    Loads(&'static Model),
    /// Refused for MuJoCo's fault, in this crate's words.
    Refused(&'static str),
}

const CASES: &[(&str, Expect)] = &[
    // a spherical joint: MuJoCo's ball
    (
        r#"<robot name="r"><link name="base"><inertial><origin xyz="0 0 0"/><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link><link name="arm"><inertial><origin xyz="0.1 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link><joint name="j" type="spherical"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5"/><axis xyz="0 1 0"/></joint></robot>"#,
        Expect::Loads(&Model {
            name: "r",
            body_parent: &[0, 0],
            body_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            body_mass: &[0.0, 1.0],
            body_inertia: &[[0.0, 0.0, 0.0], [0.1, 0.1, 0.1]],
            body_ipos: &[[0.0, 0.0, 0.0], [0.1, 0.0, 0.0]],
            body_iquat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            geom_name: &["", ""],
            geom_type: &["Box", "Box"],
            geom_body: &[0, 1],
            geom_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            geom_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            geom_size: &[[0.05, 0.05, 0.05], [0.05, 0.05, 0.05]],
            geom_contype: &[1, 1],
            geom_conaffinity: &[1, 1],
            geom_group: &[0, 0],
            geom_rgba: &[[0.5, 0.5, 0.5, 1.0], [0.5, 0.5, 0.5, 1.0]],
            jnt_limited: &[false],
            jnt_range: &[[0.0, 0.0]],
            key_qpos: &[],
            qpos5: &[0.9999999642097158, 0.0, 0.00026754544904981085, 0.0],
            reals: [0.002, 1.0, 1e-08, 0.01, 1e-06, 1e-06, 0.0001, 0.0, 0.0, 0.0],
            vectors: [0.0, 0.0, -9.81, 0.0, 0.0, 0.0, 0.0, -0.5, 0.0],
            contact_override: [
                0.02, 1.0, 0.9, 0.95, 0.001, 0.5, 2.0, 1.0, 1.0, 0.005, 0.0001, 0.0001,
            ],
            counts: [0, 0, 2, 2, 100, 50, 0, 35, 10, 40],
            flags: [0, 0, 0],
            nuser: [0, 0, 0, 0, 0, 0, 0],
        }),
    ),
    // a spherical joint's limit from 0: a ball's cone
    (
        r#"<robot name="r"><link name="base"><inertial><origin xyz="0 0 0"/><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link><link name="arm"><inertial><origin xyz="0.1 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link><joint name="j" type="spherical"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5"/><axis xyz="0 1 0"/><limit lower="0" upper="0.5" effort="0" velocity="1"/></joint></robot>"#,
        Expect::Loads(&Model {
            name: "r",
            body_parent: &[0, 0],
            body_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            body_mass: &[0.0, 1.0],
            body_inertia: &[[0.0, 0.0, 0.0], [0.1, 0.1, 0.1]],
            body_ipos: &[[0.0, 0.0, 0.0], [0.1, 0.0, 0.0]],
            body_iquat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            geom_name: &["", ""],
            geom_type: &["Box", "Box"],
            geom_body: &[0, 1],
            geom_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            geom_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            geom_size: &[[0.05, 0.05, 0.05], [0.05, 0.05, 0.05]],
            geom_contype: &[1, 1],
            geom_conaffinity: &[1, 1],
            geom_group: &[0, 0],
            geom_rgba: &[[0.5, 0.5, 0.5, 1.0], [0.5, 0.5, 0.5, 1.0]],
            jnt_limited: &[true],
            jnt_range: &[[0.0, 0.5]],
            key_qpos: &[],
            qpos5: &[0.9999999642097158, 0.0, 0.00026754544904981085, 0.0],
            reals: [0.002, 1.0, 1e-08, 0.01, 1e-06, 1e-06, 0.0001, 0.0, 0.0, 0.0],
            vectors: [0.0, 0.0, -9.81, 0.0, 0.0, 0.0, 0.0, -0.5, 0.0],
            contact_override: [
                0.02, 1.0, 0.9, 0.95, 0.001, 0.5, 2.0, 1.0, 1.0, 0.005, 0.0001, 0.0001,
            ],
            counts: [0, 0, 2, 2, 100, 50, 0, 35, 10, 40],
            flags: [0, 0, 0],
            nuser: [0, 0, 0, 0, 0, 0, 0],
        }),
    ),
    // a spherical joint's damping and friction on its three dofs
    (
        r#"<robot name="r"><link name="base"><inertial><origin xyz="0 0 0"/><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link><link name="arm"><inertial><origin xyz="0.1 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link><joint name="j" type="spherical"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5"/><axis xyz="0 1 0"/><dynamics damping="0.3" friction="0.1"/></joint></robot>"#,
        Expect::Loads(&Model {
            name: "r",
            body_parent: &[0, 0],
            body_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            body_mass: &[0.0, 1.0],
            body_inertia: &[[0.0, 0.0, 0.0], [0.1, 0.1, 0.1]],
            body_ipos: &[[0.0, 0.0, 0.0], [0.1, 0.0, 0.0]],
            body_iquat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            geom_name: &["", ""],
            geom_type: &["Box", "Box"],
            geom_body: &[0, 1],
            geom_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            geom_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            geom_size: &[[0.05, 0.05, 0.05], [0.05, 0.05, 0.05]],
            geom_contype: &[1, 1],
            geom_conaffinity: &[1, 1],
            geom_group: &[0, 0],
            geom_rgba: &[[0.5, 0.5, 0.5, 1.0], [0.5, 0.5, 0.5, 1.0]],
            jnt_limited: &[false],
            jnt_range: &[[0.0, 0.0]],
            key_qpos: &[],
            qpos5: &[0.9999999718567538, 0.0, 0.0002372477423050891, 0.0],
            reals: [0.002, 1.0, 1e-08, 0.01, 1e-06, 1e-06, 0.0001, 0.0, 0.0, 0.0],
            vectors: [0.0, 0.0, -9.81, 0.0, 0.0, 0.0, 0.0, -0.5, 0.0],
            contact_override: [
                0.02, 1.0, 0.9, 0.95, 0.001, 0.5, 2.0, 1.0, 1.0, 0.005, 0.0001, 0.0001,
            ],
            counts: [0, 0, 2, 2, 100, 50, 0, 35, 10, 40],
            flags: [0, 0, 0],
            nuser: [0, 0, 0, 0, 0, 0, 0],
        }),
    ),
    // a spherical joint without an axis
    (
        r#"<robot name="r"><link name="base"><inertial><origin xyz="0 0 0"/><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link><link name="arm"><inertial><origin xyz="0.1 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link><joint name="j" type="spherical"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5"/></joint></robot>"#,
        Expect::Loads(&Model {
            name: "r",
            body_parent: &[0, 0],
            body_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            body_mass: &[0.0, 1.0],
            body_inertia: &[[0.0, 0.0, 0.0], [0.1, 0.1, 0.1]],
            body_ipos: &[[0.0, 0.0, 0.0], [0.1, 0.0, 0.0]],
            body_iquat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            geom_name: &["", ""],
            geom_type: &["Box", "Box"],
            geom_body: &[0, 1],
            geom_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            geom_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            geom_size: &[[0.05, 0.05, 0.05], [0.05, 0.05, 0.05]],
            geom_contype: &[1, 1],
            geom_conaffinity: &[1, 1],
            geom_group: &[0, 0],
            geom_rgba: &[[0.5, 0.5, 0.5, 1.0], [0.5, 0.5, 0.5, 1.0]],
            jnt_limited: &[false],
            jnt_range: &[[0.0, 0.0]],
            key_qpos: &[],
            qpos5: &[0.9999999642097158, 0.0, 0.00026754544904981085, 0.0],
            reals: [0.002, 1.0, 1e-08, 0.01, 1e-06, 1e-06, 0.0001, 0.0, 0.0, 0.0],
            vectors: [0.0, 0.0, -9.81, 0.0, 0.0, 0.0, 0.0, -0.5, 0.0],
            contact_override: [
                0.02, 1.0, 0.9, 0.95, 0.001, 0.5, 2.0, 1.0, 1.0, 0.005, 0.0001, 0.0001,
            ],
            counts: [0, 0, 2, 2, 100, 50, 0, 35, 10, 40],
            flags: [0, 0, 0],
            nuser: [0, 0, 0, 0, 0, 0, 0],
        }),
    ),
    // a spherical joint's effort: no force range on a ball
    (
        r#"<robot name="r"><link name="base"><inertial><origin xyz="0 0 0"/><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link><link name="arm"><inertial><origin xyz="0.1 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link><joint name="j" type="spherical"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5"/><axis xyz="0 1 0"/><limit lower="0" upper="0.5" effort="3" velocity="1"/></joint></robot>"#,
        Expect::Loads(&Model {
            name: "r",
            body_parent: &[0, 0],
            body_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            body_mass: &[0.0, 1.0],
            body_inertia: &[[0.0, 0.0, 0.0], [0.1, 0.1, 0.1]],
            body_ipos: &[[0.0, 0.0, 0.0], [0.1, 0.0, 0.0]],
            body_iquat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            geom_name: &["", ""],
            geom_type: &["Box", "Box"],
            geom_body: &[0, 1],
            geom_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            geom_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            geom_size: &[[0.05, 0.05, 0.05], [0.05, 0.05, 0.05]],
            geom_contype: &[1, 1],
            geom_conaffinity: &[1, 1],
            geom_group: &[0, 0],
            geom_rgba: &[[0.5, 0.5, 0.5, 1.0], [0.5, 0.5, 0.5, 1.0]],
            jnt_limited: &[true],
            jnt_range: &[[0.0, 0.5]],
            key_qpos: &[],
            qpos5: &[0.9999999642097158, 0.0, 0.00026754544904981085, 0.0],
            reals: [0.002, 1.0, 1e-08, 0.01, 1e-06, 1e-06, 0.0001, 0.0, 0.0, 0.0],
            vectors: [0.0, 0.0, -9.81, 0.0, 0.0, 0.0, 0.0, -0.5, 0.0],
            contact_override: [
                0.02, 1.0, 0.9, 0.95, 0.001, 0.5, 2.0, 1.0, 1.0, 0.005, 0.0001, 0.0001,
            ],
            counts: [0, 0, 2, 2, 100, 50, 0, 35, 10, 40],
            flags: [0, 0, 0],
            nuser: [0, 0, 0, 0, 0, 0, 0],
        }),
    ),
    // an empty robot: an empty world
    (
        r#"<robot name="r"/>"#,
        Expect::Loads(&Model {
            name: "r",
            body_parent: &[0],
            body_quat: &[[1.0, 0.0, 0.0, 0.0]],
            body_mass: &[0.0],
            body_inertia: &[[0.0, 0.0, 0.0]],
            body_ipos: &[[0.0, 0.0, 0.0]],
            body_iquat: &[[1.0, 0.0, 0.0, 0.0]],
            geom_name: &[],
            geom_type: &[],
            geom_body: &[],
            geom_pos: &[],
            geom_quat: &[],
            geom_size: &[],
            geom_contype: &[],
            geom_conaffinity: &[],
            geom_group: &[],
            geom_rgba: &[],
            jnt_limited: &[],
            jnt_range: &[],
            key_qpos: &[],
            qpos5: &[],
            reals: [0.002, 1.0, 1e-08, 0.01, 1e-06, 1e-06, 0.0001, 0.0, 0.0, 0.0],
            vectors: [0.0, 0.0, -9.81, 0.0, 0.0, 0.0, 0.0, -0.5, 0.0],
            contact_override: [
                0.02, 1.0, 0.9, 0.95, 0.001, 0.5, 2.0, 1.0, 1.0, 0.005, 0.0001, 0.0001,
            ],
            counts: [0, 0, 2, 2, 100, 50, 0, 35, 10, 40],
            flags: [0, 0, 0],
            nuser: [0, 0, 0, 0, 0, 0, 0],
        }),
    ),
    // an empty robot, open and closed
    (
        r#"<robot name="r"></robot>"#,
        Expect::Loads(&Model {
            name: "r",
            body_parent: &[0],
            body_quat: &[[1.0, 0.0, 0.0, 0.0]],
            body_mass: &[0.0],
            body_inertia: &[[0.0, 0.0, 0.0]],
            body_ipos: &[[0.0, 0.0, 0.0]],
            body_iquat: &[[1.0, 0.0, 0.0, 0.0]],
            geom_name: &[],
            geom_type: &[],
            geom_body: &[],
            geom_pos: &[],
            geom_quat: &[],
            geom_size: &[],
            geom_contype: &[],
            geom_conaffinity: &[],
            geom_group: &[],
            geom_rgba: &[],
            jnt_limited: &[],
            jnt_range: &[],
            key_qpos: &[],
            qpos5: &[],
            reals: [0.002, 1.0, 1e-08, 0.01, 1e-06, 1e-06, 0.0001, 0.0, 0.0, 0.0],
            vectors: [0.0, 0.0, -9.81, 0.0, 0.0, 0.0, 0.0, -0.5, 0.0],
            contact_override: [
                0.02, 1.0, 0.9, 0.95, 0.001, 0.5, 2.0, 1.0, 1.0, 0.005, 0.0001, 0.0001,
            ],
            counts: [0, 0, 2, 2, 100, 50, 0, 35, 10, 40],
            flags: [0, 0, 0],
            nuser: [0, 0, 0, 0, 0, 0, 0],
        }),
    ),
    // a robot of one material
    (
        r#"<robot name="r"><material name="m"><color rgba="1 0 0 1"/></material></robot>"#,
        Expect::Loads(&Model {
            name: "r",
            body_parent: &[0],
            body_quat: &[[1.0, 0.0, 0.0, 0.0]],
            body_mass: &[0.0],
            body_inertia: &[[0.0, 0.0, 0.0]],
            body_ipos: &[[0.0, 0.0, 0.0]],
            body_iquat: &[[1.0, 0.0, 0.0, 0.0]],
            geom_name: &[],
            geom_type: &[],
            geom_body: &[],
            geom_pos: &[],
            geom_quat: &[],
            geom_size: &[],
            geom_contype: &[],
            geom_conaffinity: &[],
            geom_group: &[],
            geom_rgba: &[],
            jnt_limited: &[],
            jnt_range: &[],
            key_qpos: &[],
            qpos5: &[],
            reals: [0.002, 1.0, 1e-08, 0.01, 1e-06, 1e-06, 0.0001, 0.0, 0.0, 0.0],
            vectors: [0.0, 0.0, -9.81, 0.0, 0.0, 0.0, 0.0, -0.5, 0.0],
            contact_override: [
                0.02, 1.0, 0.9, 0.95, 0.001, 0.5, 2.0, 1.0, 1.0, 0.005, 0.0001, 0.0001,
            ],
            counts: [0, 0, 2, 2, 100, 50, 0, 35, 10, 40],
            flags: [0, 0, 0],
            nuser: [0, 0, 0, 0, 0, 0, 0],
        }),
    ),
    // a robot of the world link alone
    (
        r#"<robot name="r"><link name="world"/></robot>"#,
        Expect::Loads(&Model {
            name: "r",
            body_parent: &[0],
            body_quat: &[[1.0, 0.0, 0.0, 0.0]],
            body_mass: &[0.0],
            body_inertia: &[[0.0, 0.0, 0.0]],
            body_ipos: &[[0.0, 0.0, 0.0]],
            body_iquat: &[[1.0, 0.0, 0.0, 0.0]],
            geom_name: &[],
            geom_type: &[],
            geom_body: &[],
            geom_pos: &[],
            geom_quat: &[],
            geom_size: &[],
            geom_contype: &[],
            geom_conaffinity: &[],
            geom_group: &[],
            geom_rgba: &[],
            jnt_limited: &[],
            jnt_range: &[],
            key_qpos: &[],
            qpos5: &[],
            reals: [0.002, 1.0, 1e-08, 0.01, 1e-06, 1e-06, 0.0001, 0.0, 0.0, 0.0],
            vectors: [0.0, 0.0, -9.81, 0.0, 0.0, 0.0, 0.0, -0.5, 0.0],
            contact_override: [
                0.02, 1.0, 0.9, 0.95, 0.001, 0.5, 2.0, 1.0, 1.0, 0.005, 0.0001, 0.0001,
            ],
            counts: [0, 0, 2, 2, 100, 50, 0, 35, 10, 40],
            flags: [0, 0, 0],
            nuser: [0, 0, 0, 0, 0, 0, 0],
        }),
    ),
    // an empty robot with a <mujoco> section
    (
        r#"<robot name="r"><mujoco><option timestep="0.01"/></mujoco></robot>"#,
        Expect::Loads(&Model {
            name: "r",
            body_parent: &[0],
            body_quat: &[[1.0, 0.0, 0.0, 0.0]],
            body_mass: &[0.0],
            body_inertia: &[[0.0, 0.0, 0.0]],
            body_ipos: &[[0.0, 0.0, 0.0]],
            body_iquat: &[[1.0, 0.0, 0.0, 0.0]],
            geom_name: &[],
            geom_type: &[],
            geom_body: &[],
            geom_pos: &[],
            geom_quat: &[],
            geom_size: &[],
            geom_contype: &[],
            geom_conaffinity: &[],
            geom_group: &[],
            geom_rgba: &[],
            jnt_limited: &[],
            jnt_range: &[],
            key_qpos: &[],
            qpos5: &[],
            reals: [0.01, 1.0, 1e-08, 0.01, 1e-06, 1e-06, 0.0001, 0.0, 0.0, 0.0],
            vectors: [0.0, 0.0, -9.81, 0.0, 0.0, 0.0, 0.0, -0.5, 0.0],
            contact_override: [
                0.02, 1.0, 0.9, 0.95, 0.001, 0.5, 2.0, 1.0, 1.0, 0.005, 0.0001, 0.0001,
            ],
            counts: [0, 0, 2, 2, 100, 50, 0, 35, 10, 40],
            flags: [0, 0, 0],
            nuser: [0, 0, 0, 0, 0, 0, 0],
        }),
    ),
    // a root <Robot>: MuJoCo's strcasecmp
    (
        r#"<Robot name="r"><link name="base"><inertial><origin xyz="0 0 0"/><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link><link name="arm"><inertial><origin xyz="0.1 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5"/><axis xyz="0 1 0"/></joint></Robot>"#,
        Expect::Loads(&Model {
            name: "r",
            body_parent: &[0, 0],
            body_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            body_mass: &[0.0, 1.0],
            body_inertia: &[[0.0, 0.0, 0.0], [0.1, 0.1, 0.1]],
            body_ipos: &[[0.0, 0.0, 0.0], [0.1, 0.0, 0.0]],
            body_iquat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            geom_name: &["", ""],
            geom_type: &["Box", "Box"],
            geom_body: &[0, 1],
            geom_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            geom_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            geom_size: &[[0.05, 0.05, 0.05], [0.05, 0.05, 0.05]],
            geom_contype: &[1, 1],
            geom_conaffinity: &[1, 1],
            geom_group: &[0, 0],
            geom_rgba: &[[0.5, 0.5, 0.5, 1.0], [0.5, 0.5, 0.5, 1.0]],
            jnt_limited: &[false],
            jnt_range: &[[0.0, 0.0]],
            key_qpos: &[],
            qpos5: &[0.0005350909044833069],
            reals: [0.002, 1.0, 1e-08, 0.01, 1e-06, 1e-06, 0.0001, 0.0, 0.0, 0.0],
            vectors: [0.0, 0.0, -9.81, 0.0, 0.0, 0.0, 0.0, -0.5, 0.0],
            contact_override: [
                0.02, 1.0, 0.9, 0.95, 0.001, 0.5, 2.0, 1.0, 1.0, 0.005, 0.0001, 0.0001,
            ],
            counts: [0, 0, 2, 2, 100, 50, 0, 35, 10, 40],
            flags: [0, 0, 0],
            nuser: [0, 0, 0, 0, 0, 0, 0],
        }),
    ),
    // a root <ROBOT>
    (
        r#"<ROBOT name="r"><link name="base"><inertial><origin xyz="0 0 0"/><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link><link name="arm"><inertial><origin xyz="0.1 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5"/><axis xyz="0 1 0"/></joint></ROBOT>"#,
        Expect::Loads(&Model {
            name: "r",
            body_parent: &[0, 0],
            body_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            body_mass: &[0.0, 1.0],
            body_inertia: &[[0.0, 0.0, 0.0], [0.1, 0.1, 0.1]],
            body_ipos: &[[0.0, 0.0, 0.0], [0.1, 0.0, 0.0]],
            body_iquat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            geom_name: &["", ""],
            geom_type: &["Box", "Box"],
            geom_body: &[0, 1],
            geom_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            geom_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            geom_size: &[[0.05, 0.05, 0.05], [0.05, 0.05, 0.05]],
            geom_contype: &[1, 1],
            geom_conaffinity: &[1, 1],
            geom_group: &[0, 0],
            geom_rgba: &[[0.5, 0.5, 0.5, 1.0], [0.5, 0.5, 0.5, 1.0]],
            jnt_limited: &[false],
            jnt_range: &[[0.0, 0.0]],
            key_qpos: &[],
            qpos5: &[0.0005350909044833069],
            reals: [0.002, 1.0, 1e-08, 0.01, 1e-06, 1e-06, 0.0001, 0.0, 0.0, 0.0],
            vectors: [0.0, 0.0, -9.81, 0.0, 0.0, 0.0, 0.0, -0.5, 0.0],
            contact_override: [
                0.02, 1.0, 0.9, 0.95, 0.001, 0.5, 2.0, 1.0, 1.0, 0.005, 0.0001, 0.0001,
            ],
            counts: [0, 0, 2, 2, 100, 50, 0, 35, 10, 40],
            flags: [0, 0, 0],
            nuser: [0, 0, 0, 0, 0, 0, 0],
        }),
    ),
    // a root <rObOt>
    (
        r#"<rObOt name="r"><link name="base"><inertial><origin xyz="0 0 0"/><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link><link name="arm"><inertial><origin xyz="0.1 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5"/><axis xyz="0 1 0"/></joint></rObOt>"#,
        Expect::Loads(&Model {
            name: "r",
            body_parent: &[0, 0],
            body_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            body_mass: &[0.0, 1.0],
            body_inertia: &[[0.0, 0.0, 0.0], [0.1, 0.1, 0.1]],
            body_ipos: &[[0.0, 0.0, 0.0], [0.1, 0.0, 0.0]],
            body_iquat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            geom_name: &["", ""],
            geom_type: &["Box", "Box"],
            geom_body: &[0, 1],
            geom_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            geom_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            geom_size: &[[0.05, 0.05, 0.05], [0.05, 0.05, 0.05]],
            geom_contype: &[1, 1],
            geom_conaffinity: &[1, 1],
            geom_group: &[0, 0],
            geom_rgba: &[[0.5, 0.5, 0.5, 1.0], [0.5, 0.5, 0.5, 1.0]],
            jnt_limited: &[false],
            jnt_range: &[[0.0, 0.0]],
            key_qpos: &[],
            qpos5: &[0.0005350909044833069],
            reals: [0.002, 1.0, 1e-08, 0.01, 1e-06, 1e-06, 0.0001, 0.0, 0.0, 0.0],
            vectors: [0.0, 0.0, -9.81, 0.0, 0.0, 0.0, 0.0, -0.5, 0.0],
            contact_override: [
                0.02, 1.0, 0.9, 0.95, 0.001, 0.5, 2.0, 1.0, 1.0, 0.005, 0.0001, 0.0001,
            ],
            counts: [0, 0, 2, 2, 100, 50, 0, 35, 10, 40],
            flags: [0, 0, 0],
            nuser: [0, 0, 0, 0, 0, 0, 0],
        }),
    ),
    // an & in a link's text that starts no reference (x &amp y)
    (
        r#"<robot name="r"><link name="base"><inertial><origin xyz="0 0 0"/><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision>x &amp y</link><link name="arm"><inertial><origin xyz="0.1 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5"/><axis xyz="0 1 0"/></joint></robot>"#,
        Expect::Loads(&Model {
            name: "r",
            body_parent: &[0, 0],
            body_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            body_mass: &[0.0, 1.0],
            body_inertia: &[[0.0, 0.0, 0.0], [0.1, 0.1, 0.1]],
            body_ipos: &[[0.0, 0.0, 0.0], [0.1, 0.0, 0.0]],
            body_iquat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            geom_name: &["", ""],
            geom_type: &["Box", "Box"],
            geom_body: &[0, 1],
            geom_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            geom_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            geom_size: &[[0.05, 0.05, 0.05], [0.05, 0.05, 0.05]],
            geom_contype: &[1, 1],
            geom_conaffinity: &[1, 1],
            geom_group: &[0, 0],
            geom_rgba: &[[0.5, 0.5, 0.5, 1.0], [0.5, 0.5, 0.5, 1.0]],
            jnt_limited: &[false],
            jnt_range: &[[0.0, 0.0]],
            key_qpos: &[],
            qpos5: &[0.0005350909044833069],
            reals: [0.002, 1.0, 1e-08, 0.01, 1e-06, 1e-06, 0.0001, 0.0, 0.0, 0.0],
            vectors: [0.0, 0.0, -9.81, 0.0, 0.0, 0.0, 0.0, -0.5, 0.0],
            contact_override: [
                0.02, 1.0, 0.9, 0.95, 0.001, 0.5, 2.0, 1.0, 1.0, 0.005, 0.0001, 0.0001,
            ],
            counts: [0, 0, 2, 2, 100, 50, 0, 35, 10, 40],
            flags: [0, 0, 0],
            nuser: [0, 0, 0, 0, 0, 0, 0],
        }),
    ),
    // a bare & in a link's text
    (
        r#"<robot name="r"><link name="base"><inertial><origin xyz="0 0 0"/><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision>a & b</link><link name="arm"><inertial><origin xyz="0.1 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5"/><axis xyz="0 1 0"/></joint></robot>"#,
        Expect::Loads(&Model {
            name: "r",
            body_parent: &[0, 0],
            body_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            body_mass: &[0.0, 1.0],
            body_inertia: &[[0.0, 0.0, 0.0], [0.1, 0.1, 0.1]],
            body_ipos: &[[0.0, 0.0, 0.0], [0.1, 0.0, 0.0]],
            body_iquat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            geom_name: &["", ""],
            geom_type: &["Box", "Box"],
            geom_body: &[0, 1],
            geom_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            geom_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            geom_size: &[[0.05, 0.05, 0.05], [0.05, 0.05, 0.05]],
            geom_contype: &[1, 1],
            geom_conaffinity: &[1, 1],
            geom_group: &[0, 0],
            geom_rgba: &[[0.5, 0.5, 0.5, 1.0], [0.5, 0.5, 0.5, 1.0]],
            jnt_limited: &[false],
            jnt_range: &[[0.0, 0.0]],
            key_qpos: &[],
            qpos5: &[0.0005350909044833069],
            reals: [0.002, 1.0, 1e-08, 0.01, 1e-06, 1e-06, 0.0001, 0.0, 0.0, 0.0],
            vectors: [0.0, 0.0, -9.81, 0.0, 0.0, 0.0, 0.0, -0.5, 0.0],
            contact_override: [
                0.02, 1.0, 0.9, 0.95, 0.001, 0.5, 2.0, 1.0, 1.0, 0.005, 0.0001, 0.0001,
            ],
            counts: [0, 0, 2, 2, 100, 50, 0, 35, 10, 40],
            flags: [0, 0, 0],
            nuser: [0, 0, 0, 0, 0, 0, 0],
        }),
    ),
    // a lone & in a link's text
    (
        r#"<robot name="r"><link name="base"><inertial><origin xyz="0 0 0"/><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision>&</link><link name="arm"><inertial><origin xyz="0.1 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5"/><axis xyz="0 1 0"/></joint></robot>"#,
        Expect::Loads(&Model {
            name: "r",
            body_parent: &[0, 0],
            body_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            body_mass: &[0.0, 1.0],
            body_inertia: &[[0.0, 0.0, 0.0], [0.1, 0.1, 0.1]],
            body_ipos: &[[0.0, 0.0, 0.0], [0.1, 0.0, 0.0]],
            body_iquat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            geom_name: &["", ""],
            geom_type: &["Box", "Box"],
            geom_body: &[0, 1],
            geom_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            geom_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            geom_size: &[[0.05, 0.05, 0.05], [0.05, 0.05, 0.05]],
            geom_contype: &[1, 1],
            geom_conaffinity: &[1, 1],
            geom_group: &[0, 0],
            geom_rgba: &[[0.5, 0.5, 0.5, 1.0], [0.5, 0.5, 0.5, 1.0]],
            jnt_limited: &[false],
            jnt_range: &[[0.0, 0.0]],
            key_qpos: &[],
            qpos5: &[0.0005350909044833069],
            reals: [0.002, 1.0, 1e-08, 0.01, 1e-06, 1e-06, 0.0001, 0.0, 0.0, 0.0],
            vectors: [0.0, 0.0, -9.81, 0.0, 0.0, 0.0, 0.0, -0.5, 0.0],
            contact_override: [
                0.02, 1.0, 0.9, 0.95, 0.001, 0.5, 2.0, 1.0, 1.0, 0.005, 0.0001, 0.0001,
            ],
            counts: [0, 0, 2, 2, 100, 50, 0, 35, 10, 40],
            flags: [0, 0, 0],
            nuser: [0, 0, 0, 0, 0, 0, 0],
        }),
    ),
    // &# in a link's text
    (
        r#"<robot name="r"><link name="base"><inertial><origin xyz="0 0 0"/><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision>&#</link><link name="arm"><inertial><origin xyz="0.1 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5"/><axis xyz="0 1 0"/></joint></robot>"#,
        Expect::Loads(&Model {
            name: "r",
            body_parent: &[0, 0],
            body_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            body_mass: &[0.0, 1.0],
            body_inertia: &[[0.0, 0.0, 0.0], [0.1, 0.1, 0.1]],
            body_ipos: &[[0.0, 0.0, 0.0], [0.1, 0.0, 0.0]],
            body_iquat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            geom_name: &["", ""],
            geom_type: &["Box", "Box"],
            geom_body: &[0, 1],
            geom_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            geom_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            geom_size: &[[0.05, 0.05, 0.05], [0.05, 0.05, 0.05]],
            geom_contype: &[1, 1],
            geom_conaffinity: &[1, 1],
            geom_group: &[0, 0],
            geom_rgba: &[[0.5, 0.5, 0.5, 1.0], [0.5, 0.5, 0.5, 1.0]],
            jnt_limited: &[false],
            jnt_range: &[[0.0, 0.0]],
            key_qpos: &[],
            qpos5: &[0.0005350909044833069],
            reals: [0.002, 1.0, 1e-08, 0.01, 1e-06, 1e-06, 0.0001, 0.0, 0.0, 0.0],
            vectors: [0.0, 0.0, -9.81, 0.0, 0.0, 0.0, 0.0, -0.5, 0.0],
            contact_override: [
                0.02, 1.0, 0.9, 0.95, 0.001, 0.5, 2.0, 1.0, 1.0, 0.005, 0.0001, 0.0001,
            ],
            counts: [0, 0, 2, 2, 100, 50, 0, 35, 10, 40],
            flags: [0, 0, 0],
            nuser: [0, 0, 0, 0, 0, 0, 0],
        }),
    ),
    // a declaration before the root
    (
        r#"<?foo bar?><robot name="r"><link name="base"><inertial><origin xyz="0 0 0"/><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link><link name="arm"><inertial><origin xyz="0.1 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5"/><axis xyz="0 1 0"/></joint></robot>"#,
        Expect::Loads(&Model {
            name: "r",
            body_parent: &[0, 0],
            body_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            body_mass: &[0.0, 1.0],
            body_inertia: &[[0.0, 0.0, 0.0], [0.1, 0.1, 0.1]],
            body_ipos: &[[0.0, 0.0, 0.0], [0.1, 0.0, 0.0]],
            body_iquat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            geom_name: &["", ""],
            geom_type: &["Box", "Box"],
            geom_body: &[0, 1],
            geom_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            geom_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            geom_size: &[[0.05, 0.05, 0.05], [0.05, 0.05, 0.05]],
            geom_contype: &[1, 1],
            geom_conaffinity: &[1, 1],
            geom_group: &[0, 0],
            geom_rgba: &[[0.5, 0.5, 0.5, 1.0], [0.5, 0.5, 0.5, 1.0]],
            jnt_limited: &[false],
            jnt_range: &[[0.0, 0.0]],
            key_qpos: &[],
            qpos5: &[0.0005350909044833069],
            reals: [0.002, 1.0, 1e-08, 0.01, 1e-06, 1e-06, 0.0001, 0.0, 0.0, 0.0],
            vectors: [0.0, 0.0, -9.81, 0.0, 0.0, 0.0, 0.0, -0.5, 0.0],
            contact_override: [
                0.02, 1.0, 0.9, 0.95, 0.001, 0.5, 2.0, 1.0, 1.0, 0.005, 0.0001, 0.0001,
            ],
            counts: [0, 0, 2, 2, 100, 50, 0, 35, 10, 40],
            flags: [0, 0, 0],
            nuser: [0, 0, 0, 0, 0, 0, 0],
        }),
    ),
    // two declarations before the root
    (
        r#"<?xml version="1.0"?><?foo bar?><robot name="r"><link name="base"><inertial><origin xyz="0 0 0"/><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link><link name="arm"><inertial><origin xyz="0.1 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5"/><axis xyz="0 1 0"/></joint></robot>"#,
        Expect::Loads(&Model {
            name: "r",
            body_parent: &[0, 0],
            body_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            body_mass: &[0.0, 1.0],
            body_inertia: &[[0.0, 0.0, 0.0], [0.1, 0.1, 0.1]],
            body_ipos: &[[0.0, 0.0, 0.0], [0.1, 0.0, 0.0]],
            body_iquat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            geom_name: &["", ""],
            geom_type: &["Box", "Box"],
            geom_body: &[0, 1],
            geom_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            geom_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            geom_size: &[[0.05, 0.05, 0.05], [0.05, 0.05, 0.05]],
            geom_contype: &[1, 1],
            geom_conaffinity: &[1, 1],
            geom_group: &[0, 0],
            geom_rgba: &[[0.5, 0.5, 0.5, 1.0], [0.5, 0.5, 0.5, 1.0]],
            jnt_limited: &[false],
            jnt_range: &[[0.0, 0.0]],
            key_qpos: &[],
            qpos5: &[0.0005350909044833069],
            reals: [0.002, 1.0, 1e-08, 0.01, 1e-06, 1e-06, 0.0001, 0.0, 0.0, 0.0],
            vectors: [0.0, 0.0, -9.81, 0.0, 0.0, 0.0, 0.0, -0.5, 0.0],
            contact_override: [
                0.02, 1.0, 0.9, 0.95, 0.001, 0.5, 2.0, 1.0, 1.0, 0.005, 0.0001, 0.0001,
            ],
            counts: [0, 0, 2, 2, 100, 50, 0, 35, 10, 40],
            flags: [0, 0, 0],
            nuser: [0, 0, 0, 0, 0, 0, 0],
        }),
    ),
    // the XML declaration at the start
    (
        r#"<?xml version="1.0"?><robot name="r"><link name="base"><inertial><origin xyz="0 0 0"/><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link><link name="arm"><inertial><origin xyz="0.1 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5"/><axis xyz="0 1 0"/></joint></robot>"#,
        Expect::Loads(&Model {
            name: "r",
            body_parent: &[0, 0],
            body_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            body_mass: &[0.0, 1.0],
            body_inertia: &[[0.0, 0.0, 0.0], [0.1, 0.1, 0.1]],
            body_ipos: &[[0.0, 0.0, 0.0], [0.1, 0.0, 0.0]],
            body_iquat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            geom_name: &["", ""],
            geom_type: &["Box", "Box"],
            geom_body: &[0, 1],
            geom_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            geom_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            geom_size: &[[0.05, 0.05, 0.05], [0.05, 0.05, 0.05]],
            geom_contype: &[1, 1],
            geom_conaffinity: &[1, 1],
            geom_group: &[0, 0],
            geom_rgba: &[[0.5, 0.5, 0.5, 1.0], [0.5, 0.5, 0.5, 1.0]],
            jnt_limited: &[false],
            jnt_range: &[[0.0, 0.0]],
            key_qpos: &[],
            qpos5: &[0.0005350909044833069],
            reals: [0.002, 1.0, 1e-08, 0.01, 1e-06, 1e-06, 0.0001, 0.0, 0.0, 0.0],
            vectors: [0.0, 0.0, -9.81, 0.0, 0.0, 0.0, 0.0, -0.5, 0.0],
            contact_override: [
                0.02, 1.0, 0.9, 0.95, 0.001, 0.5, 2.0, 1.0, 1.0, 0.005, 0.0001, 0.0001,
            ],
            counts: [0, 0, 2, 2, 100, 50, 0, 35, 10, 40],
            flags: [0, 0, 0],
            nuser: [0, 0, 0, 0, 0, 0, 0],
        }),
    ),
    // text before the root, ended by its <
    (
        r#"junk<robot name="r"><link name="base"><inertial><origin xyz="0 0 0"/><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link><link name="arm"><inertial><origin xyz="0.1 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5"/><axis xyz="0 1 0"/></joint></robot>"#,
        Expect::Loads(&Model {
            name: "r",
            body_parent: &[0, 0],
            body_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            body_mass: &[0.0, 1.0],
            body_inertia: &[[0.0, 0.0, 0.0], [0.1, 0.1, 0.1]],
            body_ipos: &[[0.0, 0.0, 0.0], [0.1, 0.0, 0.0]],
            body_iquat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            geom_name: &["", ""],
            geom_type: &["Box", "Box"],
            geom_body: &[0, 1],
            geom_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            geom_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            geom_size: &[[0.05, 0.05, 0.05], [0.05, 0.05, 0.05]],
            geom_contype: &[1, 1],
            geom_conaffinity: &[1, 1],
            geom_group: &[0, 0],
            geom_rgba: &[[0.5, 0.5, 0.5, 1.0], [0.5, 0.5, 0.5, 1.0]],
            jnt_limited: &[false],
            jnt_range: &[[0.0, 0.0]],
            key_qpos: &[],
            qpos5: &[0.0005350909044833069],
            reals: [0.002, 1.0, 1e-08, 0.01, 1e-06, 1e-06, 0.0001, 0.0, 0.0, 0.0],
            vectors: [0.0, 0.0, -9.81, 0.0, 0.0, 0.0, 0.0, -0.5, 0.0],
            contact_override: [
                0.02, 1.0, 0.9, 0.95, 0.001, 0.5, 2.0, 1.0, 1.0, 0.005, 0.0001, 0.0001,
            ],
            counts: [0, 0, 2, 2, 100, 50, 0, 35, 10, 40],
            flags: [0, 0, 0],
            nuser: [0, 0, 0, 0, 0, 0, 0],
        }),
    ),
    // text after the root, ended by a comment's <
    (
        r#"<robot name="r"><link name="base"><inertial><origin xyz="0 0 0"/><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link><link name="arm"><inertial><origin xyz="0.1 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5"/><axis xyz="0 1 0"/></joint></robot>junk<!-- c -->"#,
        Expect::Loads(&Model {
            name: "r",
            body_parent: &[0, 0],
            body_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            body_mass: &[0.0, 1.0],
            body_inertia: &[[0.0, 0.0, 0.0], [0.1, 0.1, 0.1]],
            body_ipos: &[[0.0, 0.0, 0.0], [0.1, 0.0, 0.0]],
            body_iquat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            geom_name: &["", ""],
            geom_type: &["Box", "Box"],
            geom_body: &[0, 1],
            geom_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            geom_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            geom_size: &[[0.05, 0.05, 0.05], [0.05, 0.05, 0.05]],
            geom_contype: &[1, 1],
            geom_conaffinity: &[1, 1],
            geom_group: &[0, 0],
            geom_rgba: &[[0.5, 0.5, 0.5, 1.0], [0.5, 0.5, 0.5, 1.0]],
            jnt_limited: &[false],
            jnt_range: &[[0.0, 0.0]],
            key_qpos: &[],
            qpos5: &[0.0005350909044833069],
            reals: [0.002, 1.0, 1e-08, 0.01, 1e-06, 1e-06, 0.0001, 0.0, 0.0, 0.0],
            vectors: [0.0, 0.0, -9.81, 0.0, 0.0, 0.0, 0.0, -0.5, 0.0],
            contact_override: [
                0.02, 1.0, 0.9, 0.95, 0.001, 0.5, 2.0, 1.0, 1.0, 0.005, 0.0001, 0.0001,
            ],
            counts: [0, 0, 2, 2, 100, 50, 0, 35, 10, 40],
            flags: [0, 0, 0],
            nuser: [0, 0, 0, 0, 0, 0, 0],
        }),
    ),
    // text between two comments after the root
    (
        r#"<robot name="r"><link name="base"><inertial><origin xyz="0 0 0"/><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link><link name="arm"><inertial><origin xyz="0.1 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5"/><axis xyz="0 1 0"/></joint></robot><!-- a -->junk<!-- b -->"#,
        Expect::Loads(&Model {
            name: "r",
            body_parent: &[0, 0],
            body_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            body_mass: &[0.0, 1.0],
            body_inertia: &[[0.0, 0.0, 0.0], [0.1, 0.1, 0.1]],
            body_ipos: &[[0.0, 0.0, 0.0], [0.1, 0.0, 0.0]],
            body_iquat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            geom_name: &["", ""],
            geom_type: &["Box", "Box"],
            geom_body: &[0, 1],
            geom_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            geom_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            geom_size: &[[0.05, 0.05, 0.05], [0.05, 0.05, 0.05]],
            geom_contype: &[1, 1],
            geom_conaffinity: &[1, 1],
            geom_group: &[0, 0],
            geom_rgba: &[[0.5, 0.5, 0.5, 1.0], [0.5, 0.5, 0.5, 1.0]],
            jnt_limited: &[false],
            jnt_range: &[[0.0, 0.0]],
            key_qpos: &[],
            qpos5: &[0.0005350909044833069],
            reals: [0.002, 1.0, 1e-08, 0.01, 1e-06, 1e-06, 0.0001, 0.0, 0.0, 0.0],
            vectors: [0.0, 0.0, -9.81, 0.0, 0.0, 0.0, 0.0, -0.5, 0.0],
            contact_override: [
                0.02, 1.0, 0.9, 0.95, 0.001, 0.5, 2.0, 1.0, 1.0, 0.005, 0.0001, 0.0001,
            ],
            counts: [0, 0, 2, 2, 100, 50, 0, 35, 10, 40],
            flags: [0, 0, 0],
            nuser: [0, 0, 0, 0, 0, 0, 0],
        }),
    ),
    // a CDATA section after the root: a node, not text
    (
        r#"<robot name="r"><link name="base"><inertial><origin xyz="0 0 0"/><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link><link name="arm"><inertial><origin xyz="0.1 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5"/><axis xyz="0 1 0"/></joint></robot><![CDATA[ c ]]>"#,
        Expect::Loads(&Model {
            name: "r",
            body_parent: &[0, 0],
            body_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            body_mass: &[0.0, 1.0],
            body_inertia: &[[0.0, 0.0, 0.0], [0.1, 0.1, 0.1]],
            body_ipos: &[[0.0, 0.0, 0.0], [0.1, 0.0, 0.0]],
            body_iquat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            geom_name: &["", ""],
            geom_type: &["Box", "Box"],
            geom_body: &[0, 1],
            geom_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            geom_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            geom_size: &[[0.05, 0.05, 0.05], [0.05, 0.05, 0.05]],
            geom_contype: &[1, 1],
            geom_conaffinity: &[1, 1],
            geom_group: &[0, 0],
            geom_rgba: &[[0.5, 0.5, 0.5, 1.0], [0.5, 0.5, 0.5, 1.0]],
            jnt_limited: &[false],
            jnt_range: &[[0.0, 0.0]],
            key_qpos: &[],
            qpos5: &[0.0005350909044833069],
            reals: [0.002, 1.0, 1e-08, 0.01, 1e-06, 1e-06, 0.0001, 0.0, 0.0, 0.0],
            vectors: [0.0, 0.0, -9.81, 0.0, 0.0, 0.0, 0.0, -0.5, 0.0],
            contact_override: [
                0.02, 1.0, 0.9, 0.95, 0.001, 0.5, 2.0, 1.0, 1.0, 0.005, 0.0001, 0.0001,
            ],
            counts: [0, 0, 2, 2, 100, 50, 0, 35, 10, 40],
            flags: [0, 0, 0],
            nuser: [0, 0, 0, 0, 0, 0, 0],
        }),
    ),
    // a DOCTYPE after the root
    (
        r#"<robot name="r"><link name="base"><inertial><origin xyz="0 0 0"/><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link><link name="arm"><inertial><origin xyz="0.1 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5"/><axis xyz="0 1 0"/></joint></robot><!DOCTYPE x>"#,
        Expect::Loads(&Model {
            name: "r",
            body_parent: &[0, 0],
            body_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            body_mass: &[0.0, 1.0],
            body_inertia: &[[0.0, 0.0, 0.0], [0.1, 0.1, 0.1]],
            body_ipos: &[[0.0, 0.0, 0.0], [0.1, 0.0, 0.0]],
            body_iquat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            geom_name: &["", ""],
            geom_type: &["Box", "Box"],
            geom_body: &[0, 1],
            geom_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            geom_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            geom_size: &[[0.05, 0.05, 0.05], [0.05, 0.05, 0.05]],
            geom_contype: &[1, 1],
            geom_conaffinity: &[1, 1],
            geom_group: &[0, 0],
            geom_rgba: &[[0.5, 0.5, 0.5, 1.0], [0.5, 0.5, 0.5, 1.0]],
            jnt_limited: &[false],
            jnt_range: &[[0.0, 0.0]],
            key_qpos: &[],
            qpos5: &[0.0005350909044833069],
            reals: [0.002, 1.0, 1e-08, 0.01, 1e-06, 1e-06, 0.0001, 0.0, 0.0, 0.0],
            vectors: [0.0, 0.0, -9.81, 0.0, 0.0, 0.0, 0.0, -0.5, 0.0],
            contact_override: [
                0.02, 1.0, 0.9, 0.95, 0.001, 0.5, 2.0, 1.0, 1.0, 0.005, 0.0001, 0.0001,
            ],
            counts: [0, 0, 2, 2, 100, 50, 0, 35, 10, 40],
            flags: [0, 0, 0],
            nuser: [0, 0, 0, 0, 0, 0, 0],
        }),
    ),
    // a comment after the root
    (
        r#"<robot name="r"><link name="base"><inertial><origin xyz="0 0 0"/><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link><link name="arm"><inertial><origin xyz="0.1 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5"/><axis xyz="0 1 0"/></joint></robot><!-- c -->"#,
        Expect::Loads(&Model {
            name: "r",
            body_parent: &[0, 0],
            body_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            body_mass: &[0.0, 1.0],
            body_inertia: &[[0.0, 0.0, 0.0], [0.1, 0.1, 0.1]],
            body_ipos: &[[0.0, 0.0, 0.0], [0.1, 0.0, 0.0]],
            body_iquat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            geom_name: &["", ""],
            geom_type: &["Box", "Box"],
            geom_body: &[0, 1],
            geom_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            geom_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            geom_size: &[[0.05, 0.05, 0.05], [0.05, 0.05, 0.05]],
            geom_contype: &[1, 1],
            geom_conaffinity: &[1, 1],
            geom_group: &[0, 0],
            geom_rgba: &[[0.5, 0.5, 0.5, 1.0], [0.5, 0.5, 0.5, 1.0]],
            jnt_limited: &[false],
            jnt_range: &[[0.0, 0.0]],
            key_qpos: &[],
            qpos5: &[0.0005350909044833069],
            reals: [0.002, 1.0, 1e-08, 0.01, 1e-06, 1e-06, 0.0001, 0.0, 0.0, 0.0],
            vectors: [0.0, 0.0, -9.81, 0.0, 0.0, 0.0, 0.0, -0.5, 0.0],
            contact_override: [
                0.02, 1.0, 0.9, 0.95, 0.001, 0.5, 2.0, 1.0, 1.0, 0.005, 0.0001, 0.0001,
            ],
            counts: [0, 0, 2, 2, 100, 50, 0, 35, 10, 40],
            flags: [0, 0, 0],
            nuser: [0, 0, 0, 0, 0, 0, 0],
        }),
    ),
    // whitespace after the root
    (
        "<robot name=\"r\"><link name=\"base\"><inertial><origin xyz=\"0 0 0\"/><mass value=\"2\"/><inertia ixx=\"0.5\" iyy=\"0.5\" izz=\"0.5\" ixy=\"0\" ixz=\"0\" iyz=\"0\"/></inertial><collision><geometry><box size=\"0.1 0.1 0.1\"/></geometry></collision></link><link name=\"arm\"><inertial><origin xyz=\"0.1 0 0\"/><mass value=\"1\"/><inertia ixx=\"0.1\" iyy=\"0.1\" izz=\"0.1\" ixy=\"0\" ixz=\"0\" iyz=\"0\"/></inertial><collision><geometry><box size=\"0.1 0.1 0.1\"/></geometry></collision></link><joint name=\"j\" type=\"revolute\"><parent link=\"base\"/><child link=\"arm\"/><origin xyz=\"0 0 0.5\"/><axis xyz=\"0 1 0\"/></joint></robot>  \n ",
        Expect::Loads(&Model {
            name: "r",
            body_parent: &[0, 0],
            body_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            body_mass: &[0.0, 1.0],
            body_inertia: &[[0.0, 0.0, 0.0], [0.1, 0.1, 0.1]],
            body_ipos: &[[0.0, 0.0, 0.0], [0.1, 0.0, 0.0]],
            body_iquat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            geom_name: &["", ""],
            geom_type: &["Box", "Box"],
            geom_body: &[0, 1],
            geom_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            geom_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            geom_size: &[[0.05, 0.05, 0.05], [0.05, 0.05, 0.05]],
            geom_contype: &[1, 1],
            geom_conaffinity: &[1, 1],
            geom_group: &[0, 0],
            geom_rgba: &[[0.5, 0.5, 0.5, 1.0], [0.5, 0.5, 0.5, 1.0]],
            jnt_limited: &[false],
            jnt_range: &[[0.0, 0.0]],
            key_qpos: &[],
            qpos5: &[0.0005350909044833069],
            reals: [0.002, 1.0, 1e-08, 0.01, 1e-06, 1e-06, 0.0001, 0.0, 0.0, 0.0],
            vectors: [0.0, 0.0, -9.81, 0.0, 0.0, 0.0, 0.0, -0.5, 0.0],
            contact_override: [
                0.02, 1.0, 0.9, 0.95, 0.001, 0.5, 2.0, 1.0, 1.0, 0.005, 0.0001, 0.0001,
            ],
            counts: [0, 0, 2, 2, 100, 50, 0, 35, 10, 40],
            flags: [0, 0, 0],
            nuser: [0, 0, 0, 0, 0, 0, 0],
        }),
    ),
    // a vertical tab after the root: isspace
    (
        "<robot name=\"r\"><link name=\"base\"><inertial><origin xyz=\"0 0 0\"/><mass value=\"2\"/><inertia ixx=\"0.5\" iyy=\"0.5\" izz=\"0.5\" ixy=\"0\" ixz=\"0\" iyz=\"0\"/></inertial><collision><geometry><box size=\"0.1 0.1 0.1\"/></geometry></collision></link><link name=\"arm\"><inertial><origin xyz=\"0.1 0 0\"/><mass value=\"1\"/><inertia ixx=\"0.1\" iyy=\"0.1\" izz=\"0.1\" ixy=\"0\" ixz=\"0\" iyz=\"0\"/></inertial><collision><geometry><box size=\"0.1 0.1 0.1\"/></geometry></collision></link><joint name=\"j\" type=\"revolute\"><parent link=\"base\"/><child link=\"arm\"/><origin xyz=\"0 0 0.5\"/><axis xyz=\"0 1 0\"/></joint></robot>\u{b}",
        Expect::Loads(&Model {
            name: "r",
            body_parent: &[0, 0],
            body_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            body_mass: &[0.0, 1.0],
            body_inertia: &[[0.0, 0.0, 0.0], [0.1, 0.1, 0.1]],
            body_ipos: &[[0.0, 0.0, 0.0], [0.1, 0.0, 0.0]],
            body_iquat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            geom_name: &["", ""],
            geom_type: &["Box", "Box"],
            geom_body: &[0, 1],
            geom_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            geom_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            geom_size: &[[0.05, 0.05, 0.05], [0.05, 0.05, 0.05]],
            geom_contype: &[1, 1],
            geom_conaffinity: &[1, 1],
            geom_group: &[0, 0],
            geom_rgba: &[[0.5, 0.5, 0.5, 1.0], [0.5, 0.5, 0.5, 1.0]],
            jnt_limited: &[false],
            jnt_range: &[[0.0, 0.0]],
            key_qpos: &[],
            qpos5: &[0.0005350909044833069],
            reals: [0.002, 1.0, 1e-08, 0.01, 1e-06, 1e-06, 0.0001, 0.0, 0.0, 0.0],
            vectors: [0.0, 0.0, -9.81, 0.0, 0.0, 0.0, 0.0, -0.5, 0.0],
            contact_override: [
                0.02, 1.0, 0.9, 0.95, 0.001, 0.5, 2.0, 1.0, 1.0, 0.005, 0.0001, 0.0001,
            ],
            counts: [0, 0, 2, 2, 100, 50, 0, 35, 10, 40],
            flags: [0, 0, 0],
            nuser: [0, 0, 0, 0, 0, 0, 0],
        }),
    ),
    // a form feed after the root: isspace
    (
        "<robot name=\"r\"><link name=\"base\"><inertial><origin xyz=\"0 0 0\"/><mass value=\"2\"/><inertia ixx=\"0.5\" iyy=\"0.5\" izz=\"0.5\" ixy=\"0\" ixz=\"0\" iyz=\"0\"/></inertial><collision><geometry><box size=\"0.1 0.1 0.1\"/></geometry></collision></link><link name=\"arm\"><inertial><origin xyz=\"0.1 0 0\"/><mass value=\"1\"/><inertia ixx=\"0.1\" iyy=\"0.1\" izz=\"0.1\" ixy=\"0\" ixz=\"0\" iyz=\"0\"/></inertial><collision><geometry><box size=\"0.1 0.1 0.1\"/></geometry></collision></link><joint name=\"j\" type=\"revolute\"><parent link=\"base\"/><child link=\"arm\"/><origin xyz=\"0 0 0.5\"/><axis xyz=\"0 1 0\"/></joint></robot>\u{c}",
        Expect::Loads(&Model {
            name: "r",
            body_parent: &[0, 0],
            body_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            body_mass: &[0.0, 1.0],
            body_inertia: &[[0.0, 0.0, 0.0], [0.1, 0.1, 0.1]],
            body_ipos: &[[0.0, 0.0, 0.0], [0.1, 0.0, 0.0]],
            body_iquat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            geom_name: &["", ""],
            geom_type: &["Box", "Box"],
            geom_body: &[0, 1],
            geom_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            geom_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            geom_size: &[[0.05, 0.05, 0.05], [0.05, 0.05, 0.05]],
            geom_contype: &[1, 1],
            geom_conaffinity: &[1, 1],
            geom_group: &[0, 0],
            geom_rgba: &[[0.5, 0.5, 0.5, 1.0], [0.5, 0.5, 0.5, 1.0]],
            jnt_limited: &[false],
            jnt_range: &[[0.0, 0.0]],
            key_qpos: &[],
            qpos5: &[0.0005350909044833069],
            reals: [0.002, 1.0, 1e-08, 0.01, 1e-06, 1e-06, 0.0001, 0.0, 0.0, 0.0],
            vectors: [0.0, 0.0, -9.81, 0.0, 0.0, 0.0, 0.0, -0.5, 0.0],
            contact_override: [
                0.02, 1.0, 0.9, 0.95, 0.001, 0.5, 2.0, 1.0, 1.0, 0.005, 0.0001, 0.0001,
            ],
            counts: [0, 0, 2, 2, 100, 50, 0, 35, 10, 40],
            flags: [0, 0, 0],
            nuser: [0, 0, 0, 0, 0, 0, 0],
        }),
    ),
    // a second root element after the root
    (
        r#"<robot name="r"><link name="base"><inertial><origin xyz="0 0 0"/><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link><link name="arm"><inertial><origin xyz="0.1 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5"/><axis xyz="0 1 0"/></joint></robot><robot name="s"/>"#,
        Expect::Loads(&Model {
            name: "r",
            body_parent: &[0, 0],
            body_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            body_mass: &[0.0, 1.0],
            body_inertia: &[[0.0, 0.0, 0.0], [0.1, 0.1, 0.1]],
            body_ipos: &[[0.0, 0.0, 0.0], [0.1, 0.0, 0.0]],
            body_iquat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            geom_name: &["", ""],
            geom_type: &["Box", "Box"],
            geom_body: &[0, 1],
            geom_pos: &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            geom_quat: &[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]],
            geom_size: &[[0.05, 0.05, 0.05], [0.05, 0.05, 0.05]],
            geom_contype: &[1, 1],
            geom_conaffinity: &[1, 1],
            geom_group: &[0, 0],
            geom_rgba: &[[0.5, 0.5, 0.5, 1.0], [0.5, 0.5, 0.5, 1.0]],
            jnt_limited: &[false],
            jnt_range: &[[0.0, 0.0]],
            key_qpos: &[],
            qpos5: &[0.0005350909044833069],
            reals: [0.002, 1.0, 1e-08, 0.01, 1e-06, 1e-06, 0.0001, 0.0, 0.0, 0.0],
            vectors: [0.0, 0.0, -9.81, 0.0, 0.0, 0.0, 0.0, -0.5, 0.0],
            contact_override: [
                0.02, 1.0, 0.9, 0.95, 0.001, 0.5, 2.0, 1.0, 1.0, 0.005, 0.0001, 0.0001,
            ],
            counts: [0, 0, 2, 2, 100, 50, 0, 35, 10, 40],
            flags: [0, 0, 0],
            nuser: [0, 0, 0, 0, 0, 0, 0],
        }),
    ),
    // MuJoCo: "Error: range[0] should be 0 in ball joint"
    (
        r#"<robot name="r"><link name="base"><inertial><origin xyz="0 0 0"/><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link><link name="arm"><inertial><origin xyz="0.1 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link><joint name="j" type="spherical"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5"/><axis xyz="0 1 0"/><limit lower="-0.5" upper="0.5" effort="0" velocity="1"/></joint></robot>"#,
        Expect::Refused("range[0] should be 0 in ball joint"),
    ),
    // MuJoCo: "XML Error: invalid joint type in URDF joint definition"
    (
        r#"<robot name="r"><link name="base"><inertial><origin xyz="0 0 0"/><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link><link name="arm"><inertial><origin xyz="0.1 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link><joint name="j" type="Spherical"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5"/><axis xyz="0 1 0"/></joint></robot>"#,
        Expect::Refused("unknown joint type: Spherical"),
    ),
    // MuJoCo: "XML Error: URDF joint parent or child missing"
    (
        r#"<robot name="r"><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5"/><axis xyz="0 1 0"/></joint></robot>"#,
        Expect::Refused("reference to undefined link: base in joint j"),
    ),
    // MuJoCo: "Error: mass and inertia of moving bodies must be larger than mjMINVAL"
    (
        r#"<robot name="r"><link name="base"><inertial><origin xyz="0 0 0"/><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link><link name="arm"/><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5"/><axis xyz="0 1 0"/></joint></robot>"#,
        Expect::Refused(
            "the model is refused: mass and inertia of moving bodies must be larger than mjMINVAL",
        ),
    ),
    // MuJoCo: "Error: inertia must satisfy A + B >= C; use 'balanceinertia' to fix"
    (
        r#"<robot name="r"><link name="base"><inertial><origin xyz="0 0 0"/><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link><link name="arm"><inertial><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5"/><axis xyz="0 1 0"/></joint></robot>"#,
        Expect::Refused("the model is refused: inertia must satisfy A + B >= C"),
    ),
    // MuJoCo: "XML Error: bad format in attribute 'timestep'"
    (
        r#"<robot name="r"><mujoco><option timestep="x"/></mujoco><link name="base"><inertial><origin xyz="0 0 0"/><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link><link name="arm"><inertial><origin xyz="0.1 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5"/><axis xyz="0 1 0"/></joint></robot>"#,
        Expect::Refused("a <mujoco> section is refused: bad number"),
    ),
    // MuJoCo: "XML parse error 11: Error=XML_ERROR_PARSING_DECLARATION ErrorID=11 (0xb) Line number=1: XMLDeclaration value=foo bar"
    (
        r#"<robot name="r"><link name="base"><inertial><origin xyz="0 0 0"/><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision><?foo bar?></link><link name="arm"><inertial><origin xyz="0.1 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5"/><axis xyz="0 1 0"/></joint></robot>"#,
        Expect::Refused(
            "a declaration (`<?...?>`) is allowed only at the document's level, before every other node",
        ),
    ),
    // MuJoCo: "XML parse error 11: Error=XML_ERROR_PARSING_DECLARATION ErrorID=11 (0xb) Line number=1: XMLDeclaration value=foo bar"
    (
        r#"<robot name="r"><?foo bar?><link name="base"><inertial><origin xyz="0 0 0"/><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link><link name="arm"><inertial><origin xyz="0.1 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5"/><axis xyz="0 1 0"/></joint></robot>"#,
        Expect::Refused(
            "a declaration (`<?...?>`) is allowed only at the document's level, before every other node",
        ),
    ),
    // MuJoCo: "XML parse error 11: Error=XML_ERROR_PARSING_DECLARATION ErrorID=11 (0xb) Line number=1: XMLDeclaration value=foo bar"
    (
        r#"<robot name="r"><link name="base"><inertial><origin xyz="0 0 0"/><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link><link name="arm"><inertial><origin xyz="0.1 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5"/><axis xyz="0 1 0"/></joint></robot><?foo bar?>"#,
        Expect::Refused(
            "a declaration (`<?...?>`) is allowed only at the document's level, before every other node",
        ),
    ),
    // MuJoCo: "XML parse error 11: Error=XML_ERROR_PARSING_DECLARATION ErrorID=11 (0xb) Line number=1: XMLDeclaration value=xml version="1.0""
    (
        r#"<!-- c --><?xml version="1.0"?><robot name="r"><link name="base"><inertial><origin xyz="0 0 0"/><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link><link name="arm"><inertial><origin xyz="0.1 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5"/><axis xyz="0 1 0"/></joint></robot>"#,
        Expect::Refused(
            "a declaration (`<?...?>`) is allowed only at the document's level, before every other node",
        ),
    ),
    // MuJoCo: "XML parse error 11: Error=XML_ERROR_PARSING_DECLARATION ErrorID=11 (0xb) Line number=1: XMLDeclaration value=xml version="1.0""
    (
        r#"<!DOCTYPE robot><?xml version="1.0"?><robot name="r"><link name="base"><inertial><origin xyz="0 0 0"/><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link><link name="arm"><inertial><origin xyz="0.1 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5"/><axis xyz="0 1 0"/></joint></robot>"#,
        Expect::Refused(
            "a declaration (`<?...?>`) is allowed only at the document's level, before every other node",
        ),
    ),
    // MuJoCo: "XML parse error 11: Error=XML_ERROR_PARSING_DECLARATION ErrorID=11 (0xb) Line number=1: XMLDeclaration value=foo bar"
    (
        r#"junk<?foo bar?><robot name="r"><link name="base"><inertial><origin xyz="0 0 0"/><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link><link name="arm"><inertial><origin xyz="0.1 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5"/><axis xyz="0 1 0"/></joint></robot>"#,
        Expect::Refused(
            "a declaration (`<?...?>`) is allowed only at the document's level, before every other node",
        ),
    ),
    // MuJoCo: "XML parse error 8: Error=XML_ERROR_PARSING_TEXT ErrorID=8 (0x8) Line number=1"
    (
        r#"<robot name="r"><link name="base"><inertial><origin xyz="0 0 0"/><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link><link name="arm"><inertial><origin xyz="0.1 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5"/><axis xyz="0 1 0"/></joint></robot>junk"#,
        Expect::Refused("the document ends in text"),
    ),
    // MuJoCo: "XML parse error 8: Error=XML_ERROR_PARSING_TEXT ErrorID=8 (0x8) Line number=1"
    (
        r#"<robot name="r"><link name="base"><inertial><origin xyz="0 0 0"/><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link><link name="arm"><inertial><origin xyz="0.1 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5"/><axis xyz="0 1 0"/></joint></robot>&amp;"#,
        Expect::Refused("the document ends in text"),
    ),
    // MuJoCo: "XML parse error 8: Error=XML_ERROR_PARSING_TEXT ErrorID=8 (0x8) Line number=1"
    (
        "<robot name=\"r\"><link name=\"base\"><inertial><origin xyz=\"0 0 0\"/><mass value=\"2\"/><inertia ixx=\"0.5\" iyy=\"0.5\" izz=\"0.5\" ixy=\"0\" ixz=\"0\" iyz=\"0\"/></inertial><collision><geometry><box size=\"0.1 0.1 0.1\"/></geometry></collision></link><link name=\"arm\"><inertial><origin xyz=\"0.1 0 0\"/><mass value=\"1\"/><inertia ixx=\"0.1\" iyy=\"0.1\" izz=\"0.1\" ixy=\"0\" ixz=\"0\" iyz=\"0\"/></inertial><collision><geometry><box size=\"0.1 0.1 0.1\"/></geometry></collision></link><joint name=\"j\" type=\"revolute\"><parent link=\"base\"/><child link=\"arm\"/><origin xyz=\"0 0 0.5\"/><axis xyz=\"0 1 0\"/></joint></robot>junk \n",
        Expect::Refused("the document ends in text"),
    ),
];

/// Documents whose load status this crate and MuJoCo 3.5.0's import do not
/// share, each with the row that holds it and whether it loads here.
const DIVERGENT: &[(&str, &str, bool)] = &[];

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
fn options(m: &sim_core::Model) -> ([f64; 10], Vec<f64>, Vec<f64>, [i64; 10]) {
    let reals = [
        m.timestep,
        m.impratio,
        m.solver_tolerance,
        m.ls_tolerance,
        m.noslip_tolerance,
        m.ccd_tolerance,
        m.sleep_tolerance,
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
        ("geom_name", names == want.geom_name),
        ("geom_type", types == want.geom_type),
        ("geom_body", m.geom_body == want.geom_body),
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
            [
                m.nuser_body,
                m.nuser_jnt,
                m.nuser_geom,
                m.nuser_site,
                m.nuser_tendon,
                m.nuser_actuator,
                m.nuser_sensor,
            ] == want.nuser,
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

/// Each document whose load status is not MuJoCo 3.5.0's keeps the status its
/// row records, so a change to it is seen.
#[test]
fn each_known_divergence_holds() {
    let wrong: Vec<String> = DIVERGENT
        .iter()
        .filter(|(doc, _, loads_here)| load_urdf_model(doc).is_ok() != *loads_here)
        .map(|(doc, row, _)| format!("{row}: {doc}"))
        .collect();
    assert!(wrong.is_empty(), "{}", wrong.join("\n"));
}
