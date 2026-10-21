//! A constraint other than a contact whose Jacobian rows are all zero adds no
//! rows, and at the sparse size one whose dof chain is empty, against MuJoCo
//! 3.5.0 (`mj_addConstraint`, `engine_core_constraint.c:237-329`); and a
//! contact between two static geoms.
//!
//! The golden is `assets/golden/empty_rows/empty_rows.json`, from the unfused
//! MuJoCo 3.5.0 oracle (`scripts/gen_empty_rows_reference.py`, which describes
//! its cases). The flex case is checked against the rule: MuJoCo refuses this
//! crate's `<flex>` documents.

use serde_json::Value;
use sim_core::{ConstraintType, StepError};
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
        [ConstraintType::FrictionDof],
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

/// The row type for MuJoCo's `mjtConstraint` value, for the kinds these
/// cases make.
fn row_type(mj: &Value) -> ConstraintType {
    match mj.as_u64().expect("type") {
        0 => ConstraintType::Equality,
        1 => ConstraintType::FrictionDof,
        2 => ConstraintType::FrictionTendon,
        3 => ConstraintType::LimitJoint,
        4 => ConstraintType::LimitTendon,
        5 => ConstraintType::ContactFrictionless,
        6 => ConstraintType::ContactPyramidal,
        7 => ConstraintType::ContactElliptic,
        other => panic!("no such row type: {other}"),
    }
}

/// At the sparse size (`<option jacobian="sparse">`, or the default from 60
/// dofs) a constraint other than a contact adds no rows only when its dof
/// chain is empty, whatever its values (ledger L130): a connect, a weld and a
/// fixed tendon of coef 0, each between or on two jointless children of one
/// hinged body, make MuJoCo's rows at 61 and 60 dofs and, under
/// `jacobian="sparse"`, at 1 (3, 6, and a friction-loss and a limit row);
/// under `jacobian="dense"`, or the default at 59 dofs or 1, none; a tendon
/// equality on that tendon likewise. At the sparse size a spatial tendon from
/// the world to a hinged arm keeps its friction-loss row, a connect from a
/// body without dofs to a hinged arm its rows, and the static weld, connect
/// and tendons above still make none.
#[test]
fn the_sparse_size_keeps_a_constraint_whose_chain_is_not_empty() {
    let g = golden();
    let mut off = Vec::new();
    for (name, case) in g["cases"].as_object().expect("cases") {
        if !(name.starts_with("chain_") || name.ends_with("_sparse")) {
            continue;
        }
        let model = load_model(case["xml"].as_str().expect("xml")).expect("load");
        if let Some(nv) = case["nv"].as_u64() {
            assert_eq!(model.nv as u64, nv, "{name}: the fixture's dofs");
        }
        let mut data = model.make_data();
        data.forward(&model).expect("forward");
        let mujoco: Vec<ConstraintType> = case["efc_type"]
            .as_array()
            .expect("efc_type")
            .iter()
            .map(row_type)
            .collect();
        if data.efc_type != mujoco {
            off.push(format!(
                "{name}: rows {:?}, MuJoCo {mujoco:?}",
                data.efc_type
            ));
        }
    }
    assert!(off.is_empty(), "{off:#?}");
}

/// Each tendon's dof chain is MuJoCo's sparse `ten_J_rownnz`: a fixed tendon
/// over two hinges, one of coef 0 (2), spatial tendons from the world to the
/// second of two hinged bodies, from the first to a slider, from the first to
/// the second (2 each), and from the slider to the second (3); a fixed tendon
/// naming one hinge twice (1).
#[test]
fn tendon_chains_are_mujocos_sparse_rows() {
    let case = &golden()["cases"]["tendon_chains"];
    let model = load_model(case["xml"].as_str().expect("xml")).expect("load");
    let mut data = model.make_data();
    data.forward(&model).expect("forward");
    let mujoco: Vec<usize> = case["ten_J_rownnz"]
        .as_array()
        .expect("ten_J_rownnz")
        .iter()
        .map(|x| usize::try_from(x.as_u64().expect("count")).expect("fits"))
        .collect();
    assert_eq!(data.ten_J_rownnz, mujoco);
}

/// This crate's distance equality, which MuJoCo 3.5.0 lacks, takes its
/// geoms' bodies' chain at the sparse size, as a connect takes its bodies':
/// between two geoms on bodies without dofs, or from one such geom to the
/// world (no `geom2`), it makes no row under `jacobian="sparse"`, as its
/// zero rows make none at the dense size.
#[test]
fn a_distance_equality_between_static_geoms_makes_no_row_at_the_sparse_size() {
    for (name, geom2) in [
        ("distance_static", r#" geom2="g2""#),
        ("distance_to_world", ""),
    ] {
        let model = load_model(&format!(
            r#"<mujoco>
          <option jacobian="sparse"/>
          <worldbody>
            <body pos="0 0 1"><geom name="g1" type="sphere" size="0.1" mass="1"/></body>
            <body pos="0 0 2"><geom name="g2" type="sphere" size="0.1" mass="1"/></body>
            <body pos="1 0 2">
              <joint type="hinge" axis="0 1 0"/>
              <geom type="sphere" size="0.1" mass="1"/>
            </body>
          </worldbody>
          <equality><distance geom1="g1"{geom2} distance="0.5"/></equality>
        </mujoco>"#
        ))
        .expect("load");
        assert!(model.is_sparse(), "{name}: jacobian sparse");
        let mut data = model.make_data();
        data.forward(&model).expect("forward");
        assert_row_arrays(name, &data, 0);
    }
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

/// A contact between two static geoms under a `<pair>` (a sphere 1 mm into
/// the floor, both on the world body), beside a free body. At the sparse size
/// it makes no rows, as MuJoCo's ("skip contact if no DOFs affected",
/// `engine_core_constraint.c:1028-1033`); at the dense size without islands
/// it makes its rows, their Jacobians zero, and the forward pass goes on, as
/// MuJoCo's; at the dense size with islands MuJoCo stops (`treeFirst`,
/// `engine_island.c:225-228`, the golden's message) and the forward pass
/// refuses it (ledger L148).
#[test]
fn a_contact_between_two_static_geoms_makes_no_rows_or_is_refused() {
    let g = golden();
    for name in ["static_pair_sparse", "static_pair_dense_no_islands"] {
        let case = &g["cases"][name];
        let data = forward(case);
        assert_row_arrays(name, &data, nefc(case));
        let want: Vec<ConstraintType> = case["efc_type"]
            .as_array()
            .expect("types")
            .iter()
            .map(row_type)
            .collect();
        assert_eq!(data.efc_type, want, "{name}: rows");
        assert_eq!(data.contacts.len(), 1, "{name}: the contact");
        let qacc: Vec<f64> = case["qacc"]
            .as_array()
            .expect("qacc")
            .iter()
            .map(|x| x.as_f64().expect("number"))
            .collect();
        assert_eq!(data.qacc.as_slice(), qacc.as_slice(), "{name}: qacc");
    }
    let case = &g["cases"]["static_pair_dense_islands"];
    assert!(
        case["error"]
            .as_str()
            .expect("MuJoCo's error")
            .contains("between two static bodies"),
        "MuJoCo stops"
    );
    let model = load_model(case["xml"].as_str().expect("xml")).expect("load");
    let mut data = model.make_data();
    assert!(
        matches!(
            data.forward(&model),
            Err(StepError::ConstraintWithoutTree { row: 0 })
        ),
        "the forward pass refuses the contact's first row"
    );
}
