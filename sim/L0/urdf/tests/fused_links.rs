//! A link on a fixed joint, which sim-urdf's `fusestatic` fuses into its
//! parent, as MuJoCo 3.5.0's URDF import does: its mass and inertia added to
//! the parent's, and its own checks made before it is fused (sim-mjcf's
//! `fusestatic`, book Rigid-loading L42; ledger L373). The expected values
//! of `a_fixed_links_mass_is_its_parents` were printed by MuJoCo 3.5.0's own
//! URDF import (review round 1 RW-4, 2026-11-24); those of
//! `a_fused_links_mass_centre_and_tensor_are_physical` by numpy.

// The expected values are printed to the last digit.
#![allow(clippy::unwrap_used, clippy::unreadable_literal)]

use sim_urdf::load_urdf_model;

/// A body's tensor `(ixx, iyy, izz, ixy, ixz, iyz)` in its frame, from its
/// inertial frame and moments: R·diag·Rᵀ.
fn link_tensor(m: &sim_core::Model, body: usize) -> [f64; 6] {
    let turn = m.body_iquat[body].to_rotation_matrix();
    let r = turn.matrix();
    let d = m.body_inertia[body];
    let at = |i: usize, j: usize| (0..3).map(|k| r[(i, k)] * d[k] * r[(j, k)]).sum::<f64>();
    [at(0, 0), at(1, 1), at(2, 2), at(0, 1), at(0, 2), at(1, 2)]
}

/// A fixed link fused into a moving one gives the moving link the two links'
/// mass, centre and tensor, each link's tensor in its own inertial frame: the
/// physics, computed in numpy from the written values. MuJoCo 3.5.0's import
/// composes the fused link's inertial frame before its pose
/// (D-FUSESTATIC-INERTIA): its combined centre for the first document is
/// (0, 0, 0.1), the fused link's centre taken as its inertial position plus
/// its position turned by the principal axes of its unsorted diagonal tensor,
/// (0, 0, 0.3) (measured, review RL-10c RW-5, FB-10). The second document's
/// fused link has a turned inertial origin (D-URDF-INERTIAL-TURN). The tensor
/// is rebuilt from the loaded frame and moments, within 1e-5 of its largest
/// entry: `mjuu_eig3` stops with a rotation left whose cosine is above
/// 1 − 1e-12, and MuJoCo's own round trip reaches 1.4e-6 of the largest entry
/// over 3000 random tensors (measured, review RL-10c FB-4); here it is 7e-9.
#[test]
fn a_fused_links_mass_centre_and_tensor_are_physical() {
    let cases: [(&str, [f64; 3], [f64; 6]); 2] = [
        (
            r#"<robot name="r"><link name="base"><inertial><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="arm"><inertial><mass value="1"/><inertia ixx="0.1" iyy="0.2" izz="0.3" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="tool"><inertial><origin xyz="0 0 0.1"/><mass value="0.5"/><inertia ixx="0.001" iyy="0.002" izz="0.003" ixy="0" ixz="0" iyz="0"/></inertial></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5"/><axis xyz="0 1 0"/></joint><joint name="f" type="fixed"><parent link="arm"/><child link="tool"/><origin xyz="0.2 0 0"/></joint></robot>"#,
            [0.06666666666666667, 0.0, 0.03333333333333333],
            [
                0.10433333333333335,
                0.21866666666666668,
                0.3163333333333333,
                0.0,
                -0.006666666666666668,
                0.0,
            ],
        ),
        (
            r#"<robot name="r"><link name="base"><inertial><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="arm"><inertial><mass value="1"/><inertia ixx="0.1" iyy="0.2" izz="0.3" ixy="0" ixz="0" iyz="0"/></inertial></link><link name="tool"><inertial><origin xyz="0.01 0.02 0" rpy="0.3 0.5 0.7"/><mass value="0.5"/><inertia ixx="0.01" iyy="0.02" izz="0.03" ixy="0" ixz="0" iyz="0"/></inertial></link><joint name="j" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.5"/><axis xyz="0 1 0"/></joint><joint name="f" type="fixed"><parent link="arm"/><child link="tool"/><origin xyz="0.2 0 0"/></joint></robot>"#,
            [0.06999999999999999, 0.006666666666666667, 0.0],
            [
                0.11855149763657577,
                0.23155140252584533,
                0.3395637665042455,
                -0.004821470847023271,
                0.007751008354002574,
                0.0032892206727466473,
            ],
        ),
    ];
    for (doc, centre, tensor) in cases {
        let m = load_urdf_model(doc).unwrap();
        assert_eq!(m.nbody, 2, "the fixed link fused: {doc}");
        let ipos = m.body_ipos[1];
        let close = |a: f64, b: f64| (a - b).abs() <= 1e-12 * b.abs().max(1.0);
        assert!(close(m.body_mass[1], 1.5), "mass {}: {doc}", m.body_mass[1]);
        assert!(
            ipos.iter().zip(centre).all(|(&a, b)| close(a, b)),
            "centre {ipos:?}, want {centre:?}: {doc}"
        );
        let got = link_tensor(&m, 1);
        let scale = tensor.iter().fold(0.0_f64, |a, v| a.max(v.abs()));
        assert!(
            got.iter()
                .zip(tensor)
                .all(|(a, b)| (a - b).abs() <= 1e-5 * scale),
            "tensor {got:?}, want {tensor:?}: {doc}"
        );
    }
}

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
