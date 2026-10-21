//! At the sparse size each constraint row lists its dof chain, as MuJoCo's
//! (`efc_J_rownnz`, `efc_J_rowadr`, `efc_J_colind`): the dofs the row kind
//! passes to `mj_addConstraint` (`engine_core_constraint.c:237-329`),
//! whatever their values. The golden is
//! `assets/golden/sparse_rows/sparse_rows.json`, from the unfused MuJoCo
//! 3.5.0 oracle (`scripts/gen_sparse_rows_reference.py`, which describes the
//! scene: every rigid row kind, at jacobian="sparse").

use serde_json::Value;
use sim_core::{ConstraintType, EqualityType};
use sim_mjcf::load_model;

fn golden() -> Value {
    serde_json::from_str(include_str!(
        "../assets/golden/sparse_rows/sparse_rows.json"
    ))
    .expect("sparse rows golden parses")
}

fn usizes(v: &Value) -> Vec<usize> {
    v.as_array()
        .expect("array")
        .iter()
        .map(|x| usize::try_from(x.as_u64().expect("count")).expect("fits"))
        .collect()
}

fn floats(v: &Value) -> Vec<f64> {
    v.as_array()
        .expect("array")
        .iter()
        .map(|x| x.as_f64().expect("number"))
        .collect()
}

/// MuJoCo's `mjtConstraint` value for a row type.
fn mj_type(t: ConstraintType) -> usize {
    match t {
        ConstraintType::Equality => 0,
        ConstraintType::FrictionDof => 1,
        ConstraintType::FrictionTendon => 2,
        ConstraintType::LimitJoint => 3,
        ConstraintType::LimitTendon => 4,
        ConstraintType::ContactFrictionless => 5,
        ConstraintType::ContactPyramidal => 6,
        ConstraintType::ContactElliptic => 7,
    }
}

/// Each case's rows are MuJoCo's rows (type and id), each row's dof chain is
/// MuJoCo's, and its values at the chain's columns are MuJoCo's sparse
/// values.
#[test]
fn rows_list_their_dof_chains_as_mujoco() {
    let g = golden();
    let mut off = Vec::new();
    for (name, case) in g["cases"].as_object().expect("cases") {
        let model = load_model(case["xml"].as_str().expect("xml")).expect("load");
        assert!(model.is_sparse(), "{name}: the sparse size");
        let mut data = model.make_data();
        data.qpos.copy_from_slice(&floats(&case["qpos"]));
        data.forward(&model).expect("forward");

        let types: Vec<usize> = data.efc_type.iter().map(|&t| mj_type(t)).collect();
        assert_eq!(types, usizes(&case["efc_type"]), "{name}: row types");
        assert_eq!(data.efc_id, usizes(&case["efc_id"]), "{name}: row ids");
        assert_eq!(
            data.efc_J_rownnz,
            usizes(&case["efc_J_rownnz"]),
            "{name}: rownnz"
        );
        assert_eq!(
            data.efc_J_rowadr,
            usizes(&case["efc_J_rowadr"]),
            "{name}: rowadr"
        );
        assert_eq!(
            data.efc_J_colind,
            usizes(&case["efc_J_colind"]),
            "{name}: colind"
        );

        let want = floats(&case["efc_J"]);
        for row in 0..data.efc_type.len() {
            let span = data.efc_J_rowadr[row]..data.efc_J_rowadr[row] + data.efc_J_rownnz[row];
            for (&dof, &mj) in data.efc_J_colind[span.clone()].iter().zip(&want[span]) {
                let ours = data.efc_J[(row, dof)];
                if ours.to_bits() != mj.to_bits() {
                    off.push(format!(
                        "{name} row {row} dof {dof}: {ours:e}, MuJoCo {mj:e}"
                    ));
                }
            }
        }
    }
    assert!(off.is_empty(), "{} values differ:\n{off:#?}", off.len());
}

/// A flex's rows at the sparse size, which MuJoCo cannot load in this crate's
/// form (it refuses its `<deformable>` documents; Rigid-loading L31): a
/// two-triangle cloth whose vertices are in a free slab under it, its edges'
/// rows and its vertices' contacts, whose chains join a vertex's dofs to the
/// slab's. (A world plane makes no contact with it here: ledger L171. Under
/// CG, which solves it, as dense Newton does; sparse Newton's factor fails on
/// it with `CholeskyFailed`.) Every row's chain is strictly ascending and
/// holds every dof its row has a nonzero entry at.
#[test]
fn a_flex_s_rows_hold_their_entries_in_their_chains() {
    let xml = r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.001" jacobian="sparse" solver="CG"/>
  <worldbody>
    <body name="slab" pos="0.05 0.05 -0.01">
      <freejoint/>
      <geom type="box" size="0.2 0.2 0.01" mass="1"/>
    </body>
  </worldbody>
  <deformable>
    <flex name="cloth" dim="2" density="1000" radius="0.02">
      <contact margin="0.005"/>
      <vertex pos="0 0 0.015  0.1 0 0.015  0 0.1 0.015  0.1 0.1 0.015"/>
      <element data="0 1 2  1 3 2"/>
    </flex>
  </deformable>
</mujoco>"#;
    let model = load_model(xml).expect("load");
    assert!(model.is_sparse(), "the sparse size");
    let mut data = model.make_data();
    data.forward(&model).expect("forward");
    let flex_rows = (0..data.efc_type.len())
        .filter(|&r| {
            data.efc_type[r] == ConstraintType::Equality
                && model.eq_type[data.efc_id[r]] == EqualityType::Flex
        })
        .count();
    let contact_rows = data
        .efc_type
        .iter()
        .filter(|t| !matches!(t, ConstraintType::Equality))
        .count();
    assert!(
        flex_rows > 0 && contact_rows > 0,
        "{flex_rows} edge rows, {contact_rows} contact rows"
    );
    for row in 0..data.efc_type.len() {
        let adr = data.efc_J_rowadr[row];
        let chain = &data.efc_J_colind[adr..adr + data.efc_J_rownnz[row]];
        assert!(
            chain.windows(2).all(|w| w[0] < w[1]),
            "row {row}: chain {chain:?}"
        );
        for dof in 0..model.nv {
            if data.efc_J[(row, dof)] != 0.0 {
                assert!(
                    chain.contains(&dof),
                    "row {row}: dof {dof} outside its chain {chain:?}"
                );
            }
        }
    }
}
