//! A constraint other than a contact whose Jacobian rows are all zero adds no
//! rows, against MuJoCo 3.5.0 (`mj_addConstraint`,
//! `engine_core_constraint.c:237-329`).
//!
//! The golden is `assets/golden/empty_rows/empty_rows.json`, from the unfused
//! MuJoCo 3.5.0 oracle (`scripts/gen_empty_rows_reference.py`, which describes
//! its cases). The flex case is checked against the rule: MuJoCo refuses this
//! crate's `<flex>` documents.

use serde_json::Value;
use sim_core::ConstraintType;
use sim_mjcf::load_model;

fn golden() -> Value {
    serde_json::from_str(include_str!("../assets/golden/empty_rows/empty_rows.json"))
        .expect("empty rows golden parses")
}

fn forward(case: &Value) -> sim_core::Data {
    let model = load_model(case["xml"].as_str().expect("xml")).expect("load");
    let mut data = model.make_data();
    data.forward(&model).expect("forward");
    data
}

fn nefc(case: &Value) -> usize {
    usize::try_from(case["nefc"].as_u64().expect("nefc")).expect("fits")
}

/// Every row array the counting pass sized holds the rows made, `n`.
fn assert_row_arrays(name: &str, data: &sim_core::Data, n: usize) {
    for (what, len) in [
        ("efc_type", data.efc_type.len()),
        ("efc_J", data.efc_J.nrows()),
        ("efc_vel", data.efc_vel.len()),
        ("efc_aref", data.efc_aref.len()),
        ("efc_force", data.efc_force.len()),
        ("efc_jar", data.efc_jar.len()),
        ("efc_b", data.efc_b.len()),
        ("efc_state", data.efc_state.len()),
    ] {
        assert_eq!(len, n, "{name}: {what}");
    }
}

fn floats(v: &Value) -> Vec<f64> {
    v.as_array()
        .expect("array")
        .iter()
        .map(|x| x.as_f64().expect("number"))
        .collect()
}

/// A weld between two bodies without dofs, and a connect from such a body to
/// the world, make no rows (census docs 0ee0d13e, f85c405a).
#[test]
fn empty_equality_adds_no_rows() {
    let g = golden();
    for name in ["weld_static", "connect_static"] {
        let case = &g["cases"][name];
        let data = forward(case);
        assert_row_arrays(name, &data, nefc(case));
    }
}

/// A tendon whose Jacobian is zero (between two sites on the world body)
/// makes neither its friction-loss row nor a limit row, upper or lower; the
/// hinge beside it keeps its friction-loss row.
#[test]
fn empty_tendon_rows_are_not_made() {
    let case = &golden()["cases"]["tendon_static"];
    let data = forward(case);
    assert_row_arrays("tendon_static", &data, nefc(case));
    assert_eq!(
        data.efc_type,
        [ConstraintType::FrictionLoss],
        "the hinge's row"
    );
    assert_eq!(data.efc_id, [0], "on dof 0");
}

/// An edge between two pinned vertices has no dofs (a rigid edge): the flex
/// equality makes no row for it (MuJoCo skips rigid edges,
/// `engine_core_constraint.c:620-624`); the edge from the second pinned
/// vertex to the free one gets its row.
#[test]
fn empty_flex_edge_rows_are_not_made() {
    let model = load_model(
        r#"<mujoco>
          <option gravity="0 0 -9.81" timestep="0.001"/>
          <deformable>
            <flex name="rope" dim="1" density="0.1">
              <elasticity young="1e4" damping="0.1"/>
              <vertex pos="0 0 1  0.05 0 1  0.1 0 1"/>
              <element data="0 1  1 2"/>
              <pin id="0 1"/>
            </flex>
          </deformable>
        </mujoco>"#,
    )
    .expect("load");
    assert_eq!(model.nflexedge, 2);
    let mut data = model.make_data();
    data.forward(&model).expect("forward");
    let edge_rows: Vec<usize> = (0..data.efc_type.len())
        .filter(|&r| {
            data.efc_type[r] == ConstraintType::Equality
                && model.eq_type[data.efc_id[r]] == sim_core::EqualityType::Flex
        })
        .collect();
    assert_eq!(edge_rows.len(), 1, "one edge row: {edge_rows:?}");
    let e = model
        .flexedge_rigid
        .iter()
        .position(|&r| !r)
        .expect("a non-rigid edge");
    let verts = (model.flexedge_vert[e][0], model.flexedge_vert[e][1]);
    assert!(
        verts == (1, 2) || verts == (2, 1),
        "the row's edge is the one to the free vertex, got {verts:?}"
    );
}

/// A ball joint's limit inside its margin makes its row whatever the
/// rotation: at the identity its direction is MuJoCo's x axis
/// (`mju_normalize3` below mjMINVAL), and at a rotation of about 9.2e-12 rad
/// the rotation's own axis. `efc_J` and `efc_pos` bit for bit, `qacc` within
/// 1e-12.
#[test]
fn ball_limit_rows_take_mujocos_direction() {
    let g = golden();
    for name in ["ball_identity", "ball_tiny"] {
        let case = &g["cases"][name];
        let model = load_model(case["xml"].as_str().expect("xml")).expect("load");
        let mut data = model.make_data();
        data.qpos.copy_from_slice(&floats(&case["qpos"]));
        data.forward(&model).expect("forward");
        assert_row_arrays(name, &data, nefc(case));
        assert_eq!(data.efc_type, [ConstraintType::LimitJoint], "{name}");
        let want_j = floats(&case["efc_J"]);
        for (k, want) in want_j.iter().enumerate() {
            let got = data.efc_J[(0, k)];
            assert_eq!(
                got.to_bits(),
                want.to_bits(),
                "{name} efc_J[{k}]: {got:e}, MuJoCo {want:e}"
            );
        }
        let (got, want) = (data.efc_pos[0], floats(&case["efc_pos"])[0]);
        assert_eq!(
            got.to_bits(),
            want.to_bits(),
            "{name} efc_pos: {got:e}, MuJoCo {want:e}"
        );
        for (k, (got, want)) in data.qacc.iter().zip(floats(&case["qacc"])).enumerate() {
            assert!(
                (got - want).abs() <= 1e-12,
                "{name} qacc[{k}]: {got:e}, MuJoCo {want:e}"
            );
        }
    }
}
