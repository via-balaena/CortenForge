//! A link on a fixed joint, which sim-urdf's `fusestatic` fuses into its
//! parent, as MuJoCo 3.5.0's URDF import does: its mass and inertia added to
//! the parent's, and its own checks made before it is fused (sim-mjcf's
//! `fusestatic`, book Rigid-loading L42; ledger L373). Each expected value
//! was printed by MuJoCo 3.5.0's own URDF import (review round 1 RW-4,
//! 2026-11-24).

#![allow(clippy::unwrap_used)]

use sim_urdf::load_urdf_model;

/// A revolute link with no `<inertial>` whose mass is a fixed child's: the
/// child's mass, centre and moments are the link's, MuJoCo's import's (mass
/// 2, at (0.2, 0, 0), moments 0.01; its `qacc` 43.6). Before RL-10b the link
/// was refused below mjMINVAL, the child's `<inertial>` dropped.
#[test]
fn a_fixed_links_mass_is_its_parents() {
    let m = load_urdf_model(r#"<robot name="r"><link name="base"><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="1" iyy="1" izz="1" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="arm"/><link name="tool"><inertial><origin xyz="0 0 0"/><mass value="2"/><inertia ixx="0.01" iyy="0.01" izz="0.01" ixy="0" ixz="0" iyz="0"/></inertial></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 1"/><axis xyz="0 1 0"/><limit lower="-1" upper="1" effort="1" velocity="1"/></joint><joint name="f" type="fixed"><parent link="arm"/><child link="tool"/><origin xyz="0.2 0 0"/></joint></robot>"#).unwrap();
    assert_eq!(m.nbody, 2, "the fixed link fused");
    let close = |a: f64, b: f64| (a - b).abs() <= 1e-12 * b.abs().max(1.0);
    let ipos = m.body_ipos[1];
    let inertia = m.body_inertia[1];
    assert!(close(m.body_mass[1], 2.0), "mass {}", m.body_mass[1]);
    assert!(
        close(ipos.x, 0.2) && close(ipos.y, 0.0) && close(ipos.z, 0.0),
        "centre {ipos:?}"
    );
    assert!(
        inertia.iter().all(|&moment| close(moment, 0.01)),
        "moments {inertia:?}"
    );
    let mut d = m.make_data();
    d.forward(&m).unwrap();
    assert!(
        close(d.qacc[0], 43.599_999_999_999_994),
        "qacc {}",
        d.qacc[0]
    );
}

/// A fixed link whose moments break A + B >= C is refused, as MuJoCo's
/// import refuses it ("inertia must satisfy A + B >= C; use
/// 'balanceinertia' to fix"): fusion checks a body before it fuses it.
/// Before RL-10b it loaded, fused first.
#[test]
fn a_fixed_links_inertia_is_checked_before_it_is_fused() {
    let refused = load_urdf_model(r#"<robot name="r"><link name="base"><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="1" iyy="1" izz="1" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="arm"><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="tip"><inertial><origin xyz="0 0 0"/><mass value="1"/><inertia ixx="1" iyy="1" izz="3" ixy="0" ixz="0" iyz="0"/></inertial></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 1"/><axis xyz="0 1 0"/><limit lower="-1" upper="1" effort="1" velocity="1"/></joint><joint name="f" type="fixed"><parent link="arm"/><child link="tip"/><origin xyz="0 0 0.5"/></joint></robot>"#).err();
    assert!(
        refused
            .as_ref()
            .is_some_and(|e| e.to_string().contains("inertia must satisfy A + B >= C")),
        "{refused:?}"
    );
}
