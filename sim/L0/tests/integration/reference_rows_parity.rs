//! Each constraint row's velocity `J·qvel`, its reference acceleration and
//! `efc_b`, bit for bit against MuJoCo 3.5.0 (`mj_referenceConstraint`,
//! `engine_core_constraint.c:2375-2387`; `mj_fwdConstraint`,
//! `engine_forward.c:755-756`).
//!
//! The golden is `assets/golden/reference_rows/reference_rows.json`, from the
//! unfused MuJoCo 3.5.0 oracle (`scripts/gen_reference_rows_reference.py`,
//! which describes its case). Its rows' Jacobians are exact, so they are
//! checked bit for bit first: a difference after them is the rows' own
//! arithmetic.

use nalgebra::DVector;
use serde_json::Value;
use sim_core::{ConstraintType, SolverType};
use sim_mjcf::load_model;

fn floats(v: &Value) -> Vec<f64> {
    v.as_array()
        .expect("array")
        .iter()
        .map(|x| x.as_f64().expect("number"))
        .collect()
}

fn bits(v: impl IntoIterator<Item = f64>) -> Vec<u64> {
    v.into_iter().map(f64::to_bits).collect()
}

fn vec_bits(v: &DVector<f64>) -> Vec<u64> {
    bits(v.iter().copied())
}

/// Six slides under two fixed tendons, a tendon equality, two friction
/// losses and a tendon limit, at MuJoCo's qvel: the equality's velocity and
/// `efc_b` and the second tendon's differ in their last bits when the
/// products are summed in column order, which MuJoCo's `mju_dot` does not.
/// The same at jacobian="sparse", where each is a `mju_dotSparse` over the
/// row's chain. Under each solver: `efc_b` is made before the solve.
#[test]
fn rows_reference_as_mujoco_3_5_0() {
    for name in ["tendon_rows", "tendon_rows_sparse"] {
        for solver in [SolverType::Newton, SolverType::CG, SolverType::PGS] {
            rows_reference_under(name, solver);
        }
    }
}

fn rows_reference_under(name: &str, solver: SolverType) {
    let golden: Value = serde_json::from_str(include_str!(
        "../assets/golden/reference_rows/reference_rows.json"
    ))
    .expect("reference rows golden parses");
    let case = &golden["cases"][name];
    let mut model = load_model(case["xml"].as_str().expect("xml")).expect("load");
    model.solver_type = solver;
    let mut data = model.make_data();
    data.qvel
        .as_mut_slice()
        .copy_from_slice(&floats(&case["qvel"]));
    data.forward(&model).expect("forward");

    assert_eq!(
        data.efc_type,
        [
            ConstraintType::Equality,
            ConstraintType::FrictionDof,
            ConstraintType::FrictionTendon,
            ConstraintType::LimitTendon,
        ],
        "{name} {solver:?}"
    );
    let nv = model.nv;
    let jacobian = (0..data.efc_type.len()).flat_map(|r| (0..nv).map(move |c| (r, c)));
    assert_eq!(
        bits(jacobian.map(|(r, c)| data.efc_J[(r, c)])),
        bits(floats(&case["efc_J"])),
        "{name} {solver:?} efc_J"
    );
    assert_eq!(
        bits(data.efc_pos.iter().copied()),
        bits(floats(&case["efc_pos"])),
        "{name} {solver:?} efc_pos"
    );
    assert_eq!(
        vec_bits(&data.qacc_smooth),
        bits(floats(&case["qacc_smooth"])),
        "{name} {solver:?} qacc_smooth"
    );
    for (what, ours) in [
        ("efc_vel", &data.efc_vel),
        ("efc_aref", &data.efc_aref),
        ("efc_b", &data.efc_b),
    ] {
        assert_eq!(
            vec_bits(ours),
            bits(floats(&case[what])),
            "{name} {solver:?} {what}"
        );
    }
}
