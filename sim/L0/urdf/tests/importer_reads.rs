//! URDFs as MuJoCo 3.5.0's importer reads them (`xml_urdf.cc`): review round 1's documents (reviewers RA, RB,
//! RW, 2026-11-03), each measured on MuJoCo 3.5.0. `REFUSED`: MuJoCo refuses the document, with the words its row
//! names (MuJoCo's), for the fault this crate's error names too — a sub-element written twice, a required one
//! missing, a colour's numbers, a repeated attribute, and of two faults the one MuJoCo's passes meet first.
//! `LOADS`: MuJoCo loads it — a capsule, a fixed joint's unread limit and dynamics, a geometry's unread second
//! shape, a repeated material name's unread colour, entities in names, an unnamed robot. Every `LOADS` row and
//! every `REFUSED` row but the two-fault ones was refused or loaded otherwise by the reader before this file.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use sim_urdf::{load_urdf_model, parse_urdf_str};

/// `(case, URDF, MuJoCo's error, words of this crate's error)`.
const REFUSED: &[(&str, &str, &str, &str)] = &[
    (
        "RA/urdf3/robot_rgba_float_overflow",
        r#"<robot name="r"><material name="m"><color rgba="1e39 0 0 1"/></material><link name="a"><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial></link></robot>"#,
        "number is too large in attribute 'rgba'",
        "number is too large",
    ),
    (
        "RA/urdf3/robot_rgba_float_subnormal",
        r#"<robot name="r"><material name="m"><color rgba="1e-40 0 0 1"/></material><link name="a"><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial></link></robot>"#,
        "number is too large in attribute 'rgba'",
        "number is too large",
    ),
    (
        "RA/urdf3/vis_geom_dup_attr",
        r#"<robot name="r"><link name="a"><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial><visual><geometry><sphere radius="1" radius="2"/></geometry></visual></link></robot>"#,
        "XML parse error 7:",
        "duplicated attribute",
    ),
    (
        "RA/urdf3/vis_rgba_float_overflow",
        r#"<robot name="r"><link name="a"><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial><visual><geometry><box size="1 1 1"/></geometry><material name="m"><color rgba="1e39 0 0 1"/></material></visual></link></robot>"#,
        "number is too large in attribute 'rgba'",
        "number is too large",
    ),
    (
        "RA/urdf4/col_dup_attr",
        r#"<robot name="r"><link name="a"><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial><collision name="c"><geometry><sphere radius="1" radius="x"/></geometry></collision></link></robot>"#,
        "XML parse error 7:",
        "duplicated attribute",
    ),
    (
        "RA/urdf4/link_dup_attr",
        r#"<robot name="r"><link name="a" name="b"><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial></link></robot>"#,
        "XML parse error 7:",
        "duplicated attribute",
    ),
    (
        "RA/urdf5/o_duplink_vs_missing_ixy",
        r#"<robot name="r"><link name="a"><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="b"><inertial><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1"/></inertial></link><link name="a"><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial></link></robot>"#,
        "repeated URDF name:",
        "duplicate link name",
    ),
    (
        "RA/urdf5/o_loop_vs_bad_box",
        r#"<robot name="r"><link name="a"><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial><collision><geometry><box size="1 1"/></geometry></collision></link><link name="b"><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial></link><joint name="j" type="revolute"><parent link="a"/><child link="b"/></joint><joint name="k" type="revolute"><parent link="b"/><child link="a"/></joint></robot>"#,
        "URDF body not found",
        "reached from no root link",
    ),
    (
        "RA/urdf5/o_twoparents_vs_bad_axis",
        r#"<robot name="r"><link name="a"><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="b"><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="c"><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial></link><joint name="j1" type="revolute"><parent link="a"/><child link="b"/><axis xyz="1 0"/></joint><joint name="j2" type="revolute"><parent link="c"/><child link="b"/></joint></robot>"#,
        "URDF body has multiple parents:",
        "multiple parent joints",
    ),
    (
        "RA/urdf5/o_undefined_vs_missing_ixy_before",
        r#"<robot name="r"><link name="a"><inertial><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1"/></inertial></link><link name="b"><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial></link><joint name="j" type="revolute"><parent link="a"/><child link="zz"/></joint></robot>"#,
        "URDF joint parent or child missing",
        "reference to undefined link",
    ),
    (
        "RA/urdf5/o_worldchild_vs_bad_limit",
        r#"<robot name="r"><link name="world"/><link name="a"><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial></link><joint name="j" type="revolute"><parent link="a"/><child link="world"/><limit lower="x" upper="1"/></joint></robot>"#,
        "bad format in attribute 'lower'",
        "invalid value for lower",
    ),
    (
        "RA/urdf/collision_two_geometries",
        r#"<robot name="r"><link name="a"><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial><collision name="c"><geometry><box size="1 1 1"/></geometry><geometry><box size="1 1 1"/></geometry></collision></link></robot>"#,
        "repeated element: 'geometry'",
        "repeated element: geometry",
    ),
    (
        "RA/urdf/collision_two_origins",
        r#"<robot name="r"><link name="a"><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial><collision name="c"><origin xyz="1 0 0"/><origin xyz="2 0 0"/><geometry><box size="1 1 1"/></geometry></collision></link></robot>"#,
        "repeated element: 'origin'",
        "repeated element: origin",
    ),
    (
        "RA/urdf/empty_joint_element",
        r#"<robot name="r"><link name="a"><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="b"><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial></link><joint name="j" type="fixed"/></robot>"#,
        "missing element: 'parent'",
        "missing required element: parent",
    ),
    (
        "RA/urdf/gazebo_material_bad",
        r#"<robot name="r"><link name="a"><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial></link><gazebo reference="a"><material name="g"><color rgba="1 1"/></material></gazebo></robot>"#,
        "attribute 'rgba' does not have enough data",
        "does not have enough data",
    ),
    (
        "RA/urdf/geom_two_boxes",
        r#"<robot name="r"><link name="a"><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial><collision name="c"><geometry><box size="1 1 1"/><box size="2 2 2"/></geometry></collision></link></robot>"#,
        "repeated element: 'box'",
        "repeated element: box",
    ),
    (
        "RA/urdf/inertial_two_masses",
        r#"<robot name="r"><link name="a"><inertial><mass value="1"/><mass value="2"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial></link></robot>"#,
        "repeated element: 'mass'",
        "repeated element: mass",
    ),
    (
        "RA/urdf/joint_two_axes",
        r#"<robot name="r"><link name="a"><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="b"><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial></link><joint name="j" type="revolute"><parent link="a"/><child link="b"/><axis xyz="0 1 0"/><axis xyz="0 0 1"/></joint></robot>"#,
        "repeated element: 'axis'",
        "repeated element: axis",
    ),
    (
        "RA/urdf/joint_two_limits",
        r#"<robot name="r"><link name="a"><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="b"><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial></link><joint name="j" type="revolute"><parent link="a"/><child link="b"/><limit lower="-1" upper="1"/><limit lower="-2" upper="2"/></joint></robot>"#,
        "repeated element: 'limit'",
        "repeated element: limit",
    ),
    (
        "RA/urdf/joint_two_parents",
        r#"<robot name="r"><link name="a"><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="b"><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="c"><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial></link><joint name="j" type="fixed"><parent link="a"/><parent link="c"/><child link="b"/></joint></robot>"#,
        "repeated element: 'parent'",
        "repeated element: parent",
    ),
    (
        "RA/urdf/link_level_material_bad",
        r#"<robot name="r"><link name="a"><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial><material name="lm"><color rgba="1 1"/></material></link></robot>"#,
        "attribute 'rgba' does not have enough data",
        "does not have enough data",
    ),
    (
        "RA/urdf/link_two_inertials",
        r#"<robot name="r"><link name="a"><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial></link></robot>"#,
        "repeated element: 'inertial'",
        "repeated element: inertial",
    ),
    (
        "RA/urdf/two_joint_axis_noparent",
        r#"<robot name="r"><link name="a"><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="b"><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial></link><joint name="j" type="revolute"><axis xyz="1 0"/><child link="b"/></joint></robot>"#,
        "missing element: 'parent'",
        "missing required element: parent",
    ),
    (
        "RA/urdf/two_joint_axis_then_link_mass",
        r#"<robot name="r"><link name="a"><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="b"><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial></link><joint name="j" type="revolute"><parent link="a"/><child link="b"/><axis xyz="1 0"/></joint><link name="c"><inertial><mass value="abc"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial></link></robot>"#,
        "bad format in attribute 'value'",
        "invalid value for value",
    ),
    (
        "RA/urdf/two_joint_axis_then_link_noname",
        r#"<robot name="r"><link name="a"><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="b"><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial></link><joint name="j" type="revolute"><parent link="a"/><child link="b"/><axis xyz="1 0"/></joint><link><inertial><mass value="1"/></inertial></link></robot>"#,
        "required attribute missing: 'name'",
        "missing required attribute: name",
    ),
    (
        "RA/urdf/two_joint_limit_then_undefined_link",
        r#"<robot name="r"><link name="a"><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="b"><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial></link><joint name="j" type="revolute"><parent link="a"/><child link="b"/><limit lower="x" upper="1"/></joint><joint name="k" type="revolute"><parent link="a"/><child link="zz"/></joint></robot>"#,
        "URDF joint parent or child missing",
        "reference to undefined link",
    ),
    (
        "RA/urdf/two_joint_notype_noname",
        r#"<robot name="r"><link name="a"><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="b"><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial></link><joint><parent link="a"/><child link="b"/></joint></robot>"#,
        "required attribute missing: 'type'",
        "missing required attribute: type",
    ),
    (
        "RA/urdf/two_link_mass_then_material",
        r#"<robot name="r"><link name="a"><inertial><mass value="abc"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial></link><material name="m"><color rgba="1"/></material></robot>"#,
        "attribute 'rgba' does not have enough data",
        "does not have enough data",
    ),
    (
        "RA/urdf/visual_two_materials",
        r#"<robot name="r"><link name="a"><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial><visual><geometry><box size="1 1 1"/></geometry><material name="m1"/><material><color rgba="1 0 0 1"/></material></visual></link></robot>"#,
        "repeated element: 'material'",
        "repeated element: material",
    ),
    (
        "RW/urdf_dup/two_axes",
        r#"<robot name="r"><link name="a"><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="b"><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial></link><joint name="j" type="revolute"><parent link="a"/><child link="b"/><axis xyz="0 0 1"/><axis xyz="1 0 0"/><limit lower="-1" upper="1"/></joint></robot>"#,
        "repeated element: 'axis'",
        "repeated element: axis",
    ),
    (
        "RW/urdf_dup/two_geometries",
        r#"<robot name="r"><link name="a"><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial><collision><geometry><box size="1 1 1"/></geometry><geometry><sphere radius="1"/></geometry></collision></link></robot>"#,
        "repeated element: 'geometry'",
        "repeated element: geometry",
    ),
    (
        "RW/urdf_dup/two_inertials",
        r#"<robot name="r"><link name="a"><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial></link></robot>"#,
        "repeated element: 'inertial'",
        "repeated element: inertial",
    ),
    (
        "RW/urdf_dup/two_limits_second_bad",
        r#"<robot name="r"><link name="a"><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="b"><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial></link><joint name="j" type="revolute"><parent link="a"/><child link="b"/><axis xyz="0 0 1"/><limit lower="-1" upper="1"/><limit lower="abc"/></joint></robot>"#,
        "repeated element: 'limit'",
        "repeated element: limit",
    ),
];

/// `(case, URDF)`, loaded by MuJoCo.
const LOADS: &[(&str, &str)] = &[
    (
        "RA/mjcf3/ent_collision",
        r#"<robot name="r"><link name="a"><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial><collision name="&#99;"><geometry><box size="1 1 1"/></geometry></collision><collision name="c"><geometry><box size="1 1 1"/></geometry></collision></link></robot>"#,
    ),
    (
        "RA/mjcf3/ent_link",
        r#"<robot name="r"><link name="a&amp;b"><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="c"><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial></link><joint name="j" type="revolute"><parent link="a&#38;b"/><child link="c"/></joint></robot>"#,
    ),
    (
        "RA/urdf/capsule_collision",
        r#"<robot name="r"><link name="a"><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial><collision name="c"><geometry><capsule radius="0.1" length="0.5"/></geometry></collision></link></robot>"#,
    ),
    (
        "RA/urdf/fixed_dynamics_bad",
        r#"<robot name="r"><link name="a"><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="b"><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial></link><joint name="j" type="fixed"><parent link="a"/><child link="b"/><dynamics damping="abc"/></joint></robot>"#,
    ),
    (
        "RA/urdf/fixed_limit_bad",
        r#"<robot name="r"><link name="a"><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="b"><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial></link><joint name="j" type="fixed"><parent link="a"/><child link="b"/><limit lower="abc" upper="1"/></joint></robot>"#,
    ),
    (
        "RA/urdf/fixed_limit_too_much",
        r#"<robot name="r"><link name="a"><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="b"><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial></link><joint name="j" type="fixed"><parent link="a"/><child link="b"/><limit lower="1 2" upper="1"/></joint></robot>"#,
    ),
    (
        "RA/urdf/geom_box_then_bad_sphere",
        r#"<robot name="r"><link name="a"><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial><collision name="c"><geometry><box size="1 1 1"/><sphere radius="x"/></geometry></collision></link></robot>"#,
    ),
    (
        "RA/urdf/robot_material_repeat_bad",
        r#"<robot name="r"><material name="m"><color rgba="1 0 0 1"/></material><material name="m"><color rgba="x"/></material><link name="a"><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial></link></robot>"#,
    ),
    (
        "RA/urdf/robot_no_name",
        r#"<robot><link name="a"><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial></link></robot>"#,
    ),
    (
        "RB/mat2/vis_then_robot_bad",
        r#"<robot name="r"><link name="base"><visual><geometry><sphere radius="0.1"/></geometry><material name="m"><color rgba="1 0 0 1"/></material></visual></link><material name="m"><color rgba="1 0"/></material></robot>"#,
    ),
    (
        "RB/misc/mat_repeat_bad",
        r#"<robot name="r"><material name="m"><color rgba="1 0 0 1"/></material><material name="m"><color rgba="1 0"/></material><link name="base"/></robot>"#,
    ),
    (
        "RW/urdf_fixed/fixed_dynamics_damping__x__",
        r#"<robot name="r"><link name="a"><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="b"><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial></link><joint name="j" type="fixed"><parent link="a"/><child link="b"/><origin xyz="0 0 3"/><dynamics damping="x"/></joint></robot>"#,
    ),
    (
        "RW/urdf_fixed/fixed_limit_effort__x__",
        r#"<robot name="r"><link name="a"><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="b"><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial></link><joint name="j" type="fixed"><parent link="a"/><child link="b"/><origin xyz="0 0 3"/><limit effort="x"/></joint></robot>"#,
    ),
    (
        "RW/urdf_fixed/fixed_limit_lower__1_2__",
        r#"<robot name="r"><link name="a"><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="b"><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial></link><joint name="j" type="fixed"><parent link="a"/><child link="b"/><origin xyz="0 0 3"/><limit lower="1 2"/></joint></robot>"#,
    ),
    (
        "RW/urdf_fixed/fixed_limit_lower__abc__upper__1__",
        r#"<robot name="r"><link name="a"><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="b"><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial></link><joint name="j" type="fixed"><parent link="a"/><child link="b"/><origin xyz="0 0 3"/><limit lower="abc" upper="1"/></joint></robot>"#,
    ),
    (
        "RW/urdf_fixed/fixed_mimic_joint__j0__multiplier__x",
        r#"<robot name="r"><link name="a"><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="b"><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial></link><joint name="j" type="fixed"><parent link="a"/><child link="b"/><origin xyz="0 0 3"/><mimic joint="j0" multiplier="x"/></joint></robot>"#,
    ),
    (
        "RW/urdf_shape/capsule_and_box",
        r#"<robot name="r"><link name="a"><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial><collision name="c"><geometry><capsule radius="0.1" length="0.5"/></geometry></collision><collision name="d"><geometry><box size="1 1 1"/></geometry></collision></link></robot>"#,
    ),
    (
        "RW/urdf_shape/capsule_collision",
        r#"<robot name="r"><link name="a"><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial><collision name="c"><geometry><capsule radius="0.1" length="0.5"/></geometry></collision></link></robot>"#,
    ),
];

#[test]
fn what_mujocos_importer_refuses_is_refused_for_its_fault() {
    let mut missed = Vec::new();
    for &(case, urdf, mujoco, words) in REFUSED {
        match load_urdf_model(urdf) {
            Err(e) if e.to_string().contains(words) => {}
            Err(e) => missed.push(format!("{case}: {e} (MuJoCo: {mujoco})")),
            Ok(_) => missed.push(format!("{case}: loads (MuJoCo: {mujoco})")),
        }
    }
    assert!(missed.is_empty(), "{}", missed.join("\n"));
}

#[test]
fn what_mujocos_importer_loads_is_read() {
    for &(case, urdf) in LOADS {
        parse_urdf_str(urdf).unwrap_or_else(|e| panic!("{case}: {e}"));
        load_urdf_model(urdf).unwrap_or_else(|e| panic!("{case}: {e}"));
    }
}
