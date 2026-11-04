//! An explicit `<pair>` of two geoms on one free body whose `<inertial>`
//! sits at its origin, so the body is simple (registry `D-SAME-BODY-PAIR`,
//! ledger L222). MuJoCo 3.5.0 stops at the sparse size on its first forward
//! pass ('mj_makeConstraint: constraint Jacobian mis-allocation: found nJ=48
//! but allocated 24'; `mj_mergeChainSimple`, `engine_core_util.c:95-121`,
//! has no test for both sides being one body). At the dense size it runs:
//! 4 rows, every Jacobian value 0, `qacc` that of the same body without the
//! pair, each force 312.49999999999983 (measured). This crate runs the
//! document at both sizes as MuJoCo runs it at the dense one.

use sim_mjcf::load_model;

/// The pair at the sparse size, where MuJoCo 3.5.0 stops.
const SPARSE: &str = r#"<mujoco><option jacobian="sparse"/><worldbody><body name="ball" pos="0 0 1"><freejoint/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/><geom name="g1" type="sphere" size="0.1" pos="-0.075 0 0" mass="0"/><geom name="g2" type="sphere" size="0.1" pos="0.075 0 0" mass="0"/></body></worldbody><contact><pair geom1="g1" geom2="g2"/></contact></mujoco>"#;

/// The pair at the dense size, where MuJoCo 3.5.0 runs.
const DENSE: &str = r#"<mujoco><option jacobian="dense"/><worldbody><body name="ball" pos="0 0 1"><freejoint/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/><geom name="g1" type="sphere" size="0.1" pos="-0.075 0 0" mass="0"/><geom name="g2" type="sphere" size="0.1" pos="0.075 0 0" mass="0"/></body></worldbody><contact><pair geom1="g1" geom2="g2"/></contact></mujoco>"#;

/// The body without the pair.
const WITHOUT_THE_PAIR: &str = r#"<mujoco><option jacobian="sparse"/><worldbody><body name="ball" pos="0 0 1"><freejoint/><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/><geom name="g1" type="sphere" size="0.1" pos="-0.075 0 0" mass="0"/><geom name="g2" type="sphere" size="0.1" pos="0.075 0 0" mass="0"/></body></worldbody></mujoco>"#;

fn forward(xml: &str) -> sim_core::Data {
    let model = load_model(xml).expect("load");
    let mut data = model.make_data();
    data.forward(&model).expect("forward");
    data
}

/// At both sizes: 4 rows, every Jacobian value 0, `qacc` bit for bit that
/// of the body without the pair, and each force MuJoCo's dense one; at the
/// sparse size each row lists the body's 6 dofs once.
#[test]
fn a_pair_on_one_simple_body_runs_at_both_sizes_as_mujoco_at_the_dense_one() {
    let without = forward(WITHOUT_THE_PAIR);
    assert!(without.efc_type.is_empty());
    for (size, xml) in [("sparse", SPARSE), ("dense", DENSE)] {
        assert_eq!(load_model(xml).expect(size).is_sparse(), size == "sparse");
        let data = forward(xml);
        assert_eq!(data.efc_type.len(), 4, "{size}: rows");
        assert!(data.efc_J.iter().all(|&x| x == 0.0), "{size}: Jacobian");
        if size == "sparse" {
            assert_eq!(data.efc_J_rownnz, [6; 4], "{size}: each dof once");
        }
        assert_eq!(
            data.qacc.as_slice(),
            without.qacc.as_slice(),
            "{size}: qacc"
        );
        for &force in data.efc_force.iter() {
            assert!(
                (force - 312.499_999_999_999_83).abs() < 1e-9,
                "{size}: force {force}"
            );
        }
    }
}
