//! The Jacobians against MuJoCo 3.5.0: `mj_jac` built from MuJoCo's `cdof`
//! about the subtree centre of mass (`engine_core_util.c:172-220`),
//! `mj_applyFT`'s transposes (`engine_support.c:446-501`), and a contact's
//! rows (`mj_contactJacobian`, `engine_core_constraint.c:907-986`), a
//! connect's and a weld's rows, the spatial tendons' Jacobians and the
//! kinetic energy.
//!
//! The golden is `assets/golden/jacobian/jacobian.json`, from the unfused
//! MuJoCo 3.5.0 oracle (`scripts/gen_jacobian_reference.py`, which describes
//! its cases).

use nalgebra::{DVector, Quaternion, UnitQuaternion, Vector3};
use serde_json::Value;
use sim_core::{
    Contact, Data, Model, compute_contact_jacobian, mj_apply_ft, mj_energy_vel, mj_jac, mj_mul_m,
};
use sim_mjcf::load_model;

fn golden() -> Value {
    serde_json::from_str(include_str!("../assets/golden/jacobian/jacobian.json"))
        .expect("jacobian golden parses")
}

fn floats(v: &Value) -> Vec<f64> {
    v.as_array()
        .expect("array")
        .iter()
        .map(|x| x.as_f64().expect("number"))
        .collect()
}

fn vec3(v: &Value) -> Vector3<f64> {
    Vector3::from_column_slice(&floats(v))
}

fn assert_bits(what: &str, got: &[f64], want: &[f64]) {
    assert_eq!(got.len(), want.len(), "{what}: length");
    let differ: Vec<String> = got
        .iter()
        .zip(want)
        .enumerate()
        .filter(|(_, (a, b))| a.to_bits() != b.to_bits())
        .map(|(i, (a, b))| format!("[{i}] {a:e}, MuJoCo {b:e}"))
        .collect();
    assert!(
        differ.is_empty(),
        "{what}: {} differ: {}",
        differ.len(),
        differ.join("; ")
    );
}

/// [`assert_bits`] on macOS arm64, where the golden was made. The values
/// pass through `sin` and `cos` (the moved state's rotations), which another
/// platform's libm may round otherwise by an ulp; there each entry is within
/// `1e-13 · max(1, |MuJoCo's|)`.
fn assert_golden(what: &str, got: &[f64], want: &[f64]) {
    if cfg!(all(target_os = "macos", target_arch = "aarch64")) {
        assert_bits(what, got, want);
        return;
    }
    assert_eq!(got.len(), want.len(), "{what}: length");
    for (i, (a, b)) in got.iter().zip(want).enumerate() {
        assert!(
            (a - b).abs() <= 1e-13 * b.abs().max(1.0),
            "{what}[{i}]: {a:e}, MuJoCo {b:e}"
        );
    }
}

fn vec3s(v: &Value) -> Vec<Vector3<f64>> {
    floats(v)
        .chunks(3)
        .map(Vector3::from_column_slice)
        .collect()
}

/// The case's model with MuJoCo's frame and mass fields written in (the
/// loader rounds a normalized axis or quaternion otherwise), at its state.
fn at_state(case: &Value) -> (Model, Data) {
    at_state_of(
        load_model(case["xml"].as_str().expect("xml")).expect("load"),
        case,
    )
}

fn at_state_of(mut model: Model, case: &Value) -> (Model, Data) {
    let f = &case["model"];
    let quats = |v: &Value| -> Vec<UnitQuaternion<f64>> {
        floats(v)
            .chunks(4)
            .map(|q| UnitQuaternion::new_unchecked(Quaternion::new(q[0], q[1], q[2], q[3])))
            .collect()
    };
    model.body_pos = vec3s(&f["body_pos"]);
    model.body_quat = quats(&f["body_quat"]);
    model.body_ipos = vec3s(&f["body_ipos"]);
    model.body_iquat = quats(&f["body_iquat"]);
    model.body_mass = floats(&f["body_mass"]);
    model.body_inertia = vec3s(&f["body_inertia"]);
    model.jnt_pos = vec3s(&f["jnt_pos"]);
    model.jnt_axis = vec3s(&f["jnt_axis"]);
    model.qpos0.copy_from_slice(&floats(&f["qpos0"]));
    model.recompute_derived().expect("recompute_derived");
    let mut data = model.make_data();
    data.qpos.copy_from_slice(&floats(&case["qpos"]));
    data.forward(&model).expect("forward");
    (model, data)
}

fn row_major(m: &nalgebra::DMatrix<f64>) -> Vec<f64> {
    (0..m.nrows())
        .flat_map(|r| (0..m.ncols()).map(move |c| m[(r, c)]))
        .collect()
}

/// `mj_jac` at a point near each body's centre of mass: a hinge and a ball
/// off their bodies' origins, a slide and a hinge on one body, a jointless
/// body welded to it, a free body; and `subtree_com`, which it reads.
#[test]
fn jacobians_match_mujoco_3_5_0_bit_for_bit() {
    let case = &golden()["cases"]["jac"];
    let (model, data) = at_state(case);
    let com: Vec<f64> = data
        .subtree_com
        .iter()
        .flat_map(|c| [c.x, c.y, c.z])
        .collect();
    assert_golden("subtree_com", &com, &floats(&case["subtree_com"]));
    for b in case["bodies"].as_array().expect("bodies") {
        let body = usize::try_from(b["body"].as_u64().expect("body")).expect("fits");
        let (jacp, jacr) = mj_jac(&model, &data, body, &vec3(&b["point"]));
        assert_golden(
            &format!("body {body} jacp"),
            &row_major(&jacp),
            &floats(&b["jacp"]),
        );
        assert_golden(
            &format!("body {body} jacr"),
            &row_major(&jacr),
            &floats(&b["jacr"]),
        );
    }
}

/// `mj_applyFT` of a force and a torque at that point, added to a vector that
/// is not 0: the force's term first, then the torque's.
#[test]
fn apply_ft_matches_mujoco_3_5_0_bit_for_bit() {
    let case = &golden()["cases"]["apply_ft"];
    let (model, data) = at_state(case);
    for b in case["bodies"].as_array().expect("bodies") {
        let body = usize::try_from(b["body"].as_u64().expect("body")).expect("fits");
        let mut qfrc = DVector::from_vec(floats(&b["qfrc0"]));
        mj_apply_ft(
            &model,
            &data,
            &vec3(&b["force"]),
            &vec3(&b["torque"]),
            &vec3(&b["point"]),
            body,
            &mut qfrc,
        );
        assert_golden(
            &format!("body {body} qfrc"),
            qfrc.as_slice(),
            &floats(&b["qfrc"]),
        );
    }
}

/// Gravity compensation projects each body's weight through `mj_applyFT`.
#[test]
fn gravity_compensation_matches_mujoco_3_5_0_bit_for_bit() {
    let case = &golden()["cases"]["gravcomp"];
    let (_, data) = at_state(case);
    assert_golden(
        "qfrc_gravcomp",
        data.qfrc_gravcomp.as_slice(),
        &floats(&case["qfrc_gravcomp"]),
    );
}

/// A contact's Jacobian rows in its frame, for MuJoCo's own contacts (their
/// geoms, position, frame and dimension written into a sim-core contact):
/// a free box on a plane (condim 3), a hinged capsule (6) and a sphere on a
/// ball joint below it (4), each row bit for bit with MuJoCo's `efc_J`
/// (an elliptic cone, whose rows are the Jacobian's).
#[test]
fn contact_jacobians_match_mujoco_3_5_0_bit_for_bit() {
    let case = &golden()["cases"]["contacts"];
    let (model, data) = at_state(case);
    for (i, c) in case["contacts"]
        .as_array()
        .expect("contacts")
        .iter()
        .enumerate()
    {
        let geom: Vec<usize> = c["geom"]
            .as_array()
            .expect("geom")
            .iter()
            .map(|g| usize::try_from(g.as_u64().expect("geom")).expect("fits"))
            .collect();
        let frame = floats(&c["frame"]);
        let dim = usize::try_from(c["dim"].as_u64().expect("dim")).expect("fits");
        let contact = Contact {
            pos: vec3(&c["pos"]),
            normal: Vector3::from_column_slice(&frame[0..3]),
            depth: 0.0,
            geom: [Some(geom[0]), Some(geom[1])],
            flex: [None; 2],
            elem: [None; 2],
            vert: [None; 2],
            friction: 1.0,
            dim,
            includemargin: 0.0,
            mu: [1.0, 1.0, 0.005, 0.0001, 0.0001],
            solref: [0.02, 1.0],
            solreffriction: [0.0, 0.0],
            solimp: [0.9, 0.95, 0.001, 0.5, 2.0],
            frame: [
                Vector3::from_column_slice(&frame[3..6]),
                Vector3::from_column_slice(&frame[6..9]),
            ],
            regularized_mu: 0.0,
        };
        let j = compute_contact_jacobian(&model, &data, &contact);
        assert_golden(
            &format!("contact {i} (dim {dim})"),
            &row_major(&j),
            &floats(&c["efc_J"]),
        );
    }
}

/// Connect and weld rows: a connect between a box on a ball joint (below a
/// hinged arm) and a free sphere, a weld between the sphere and the arm, a
/// weld of the box to the world, at a moved state, `efc_J` and `efc_pos`
/// bit for bit with MuJoCo's. MuJoCo's `eq_data` (its compiler's anchors
/// and `relpose`) is written in, as the frames are.
#[test]
fn equality_rows_match_mujoco_3_5_0_bit_for_bit() {
    let case = &golden()["cases"]["equalities"];
    let mut model = load_model(case["xml"].as_str().expect("xml")).expect("load");
    let eq_data = floats(&case["eq_data"]);
    for (eq, chunk) in model.eq_data.iter_mut().zip(eq_data.chunks(11)) {
        eq.copy_from_slice(chunk);
    }
    let (_, data) = at_state_of(model, case);
    assert_eq!(data.efc_J.nrows(), 15, "the equality rows alone");
    assert_golden("efc_J", &row_major(&data.efc_J), &floats(&case["efc_J"]));
    assert_golden("efc_pos", &data.efc_pos, &floats(&case["efc_pos"]));
}

/// Spatial tendons through sites on a hinged arm, a box on a ball joint
/// below it and a free sphere, one with a pulley: `ten_J` bit for bit with
/// MuJoCo's (each segment between different bodies adds its Jacobians'
/// difference projected on its normalized direction, scaled by 1/divisor).
#[test]
fn spatial_tendon_jacobians_match_mujoco_3_5_0_bit_for_bit() {
    let case = &golden()["cases"]["tendons"];
    let (_, data) = at_state(case);
    let sites: Vec<f64> = data
        .site_xpos
        .iter()
        .flat_map(|p| [p.x, p.y, p.z])
        .collect();
    assert_golden("site_xpos", &sites, &floats(&case["site_xpos"]));
    assert_golden("ten_length", &data.ten_length, &floats(&case["ten_length"]));
    let ten_j: Vec<f64> = data
        .ten_J
        .iter()
        .flat_map(|row| row.iter().copied())
        .collect();
    assert_golden("ten_J", &ten_j, &floats(&case["ten_J"]));
}

/// MuJoCo's mass matrix of a chain of seven hinges written into `qM`, times
/// `qvel` (`mj_mulM`) and the kinetic energy from it (`mj_energyVel`), bit
/// for bit, at velocities that make the order of each sum show (the
/// generator checks that). sim-core's own mass matrix rounds otherwise, as
/// the mass factor golden shows.
#[test]
fn mass_product_and_kinetic_energy_match_mujoco_3_5_0_bit_for_bit() {
    let case = &golden()["cases"]["energy"];
    let (model, mut data) = at_state(case);
    data.qvel.copy_from_slice(&floats(&case["qvel"]));
    let m = floats(&case["M"]);
    for i in 0..model.nv {
        let row = model.M_rowadr[i]..model.M_rowadr[i] + model.M_rownnz[i];
        for (&j, &v) in model.M_colind[row.clone()].iter().zip(&m[row]) {
            data.qM[(i, j)] = v;
            data.qM[(j, i)] = v;
        }
    }
    let mqvel = mj_mul_m(&model, &data, data.qvel.as_slice());
    assert_bits("mj_mulM", mqvel.as_slice(), &floats(&case["mul_m"]));
    mj_energy_vel(&model, &mut data);
    assert_bits(
        "energy_kinetic",
        &[data.energy_kinetic],
        &[case["energy_kinetic"].as_f64().expect("energy")],
    );
}

/// A flex vertex whose body hangs off a moving body (a hinged arm): MuJoCo
/// 3.5.0 loads it; sim-core's flex edge Jacobian and passive forces take only
/// a vertex's own three dofs, so sim-mjcf refuses it, a stated limitation.
/// Until Rigid-loading L10 this test loaded the form through sim-mjcf's own
/// `<flex>` and checked the vertex–floor contact row's hinge column (ledger
/// L114); no document reaches that row now.
#[test]
fn flex_vertex_body_on_a_moving_body_is_refused() {
    let xml = r#"<mujoco>
      <worldbody>
        <geom name="floor" type="plane" size="2 2 .1"/>
        <body name="arm" pos="0 0 0.3">
          <joint name="h" type="hinge" axis="0 1 0" pos="-0.2 0 0"/>
          <geom type="sphere" size="0.02" mass="1" contype="0" conaffinity="0"/>
          <body name="c_v0" pos="0.1 0 -0.28"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/><inertial pos="0 0 0" mass="0.1" diaginertia="1e-15 1e-15 1e-15"/></body>
          <body name="c_v1" pos="0.2 0 -0.28"><inertial pos="0 0 0" mass="0.1" diaginertia="1e-15 1e-15 1e-15"/></body>
        </body>
      </worldbody>
      <deformable>
        <flex name="c" dim="1" radius="0.05" body="c_v0 c_v1" element="0 1">
          <edge stiffness="10"/>
        </flex>
      </deformable>
    </mujoco>"#;
    match load_model(xml) {
        Err(sim_mjcf::MjcfError::Unsupported { feature, .. }) => assert!(
            feature.contains("vertex body 'c_v0' moves with a body above it"),
            "{feature}"
        ),
        other => panic!("not a stated limitation: {other:?}"),
    }
}

fn flex_golden() -> Value {
    serde_json::from_str(include_str!("../assets/golden/flex/flex_jacobian.json"))
        .expect("flex jacobian golden parses")
}

/// MuJoCo's id for a contact side's object: -1 for none.
fn side(v: &Value, k: usize) -> Option<usize> {
    usize::try_from(v[k].as_i64().expect("id")).ok()
}

/// A contact with a flex element on a side takes MuJoCo's general Jacobian
/// (`engine_core_constraint.c:932-985`): each side's bodies with weights,
/// an element's vertices' bodies weighted by their inverse distance to the
/// contact (`mj_elemBodyWeight`, `:142-182`, the vertex on the other side
/// left out), summed (`mj_jacSum`, `engine_core_util.c:503-573`). For
/// MuJoCo's own contacts (`gen_flex_jacobian_reference.py`: a free sphere
/// on a cloth's cell at condim 3 and 6, a box across one, a sphere on a
/// cable's element, a thin tetrahedron's internal contacts, each flex after
/// a cable; the elliptic cone, whose rows are the Jacobian's), each row bit
/// for bit with MuJoCo's `efc_J`; at the sparse size MuJoCo's values at its
/// columns, and nothing elsewhere. A rotational row at the sparse size is
/// the dense size's (divergence `D-FLEX-SPARSE-ROT`: MuJoCo's sparse
/// rotational rows of such a contact are other than its dense ones, which
/// the test confirms).
#[test]
fn flex_contact_jacobians_match_mujoco_3_5_0_bit_for_bit() {
    let golden = flex_golden();
    let cases = golden["cases"].as_array().expect("cases");
    for case in cases {
        let name = case["name"].as_str().expect("name");
        let dense_twin = name.strip_suffix("_sparse").map(|dense| {
            cases
                .iter()
                .find(|c| c["name"].as_str() == Some(dense))
                .expect("the dense twin")
        });
        let model = load_model(case["ours"].as_str().expect("ours")).expect("load");
        let mut data = model.make_data();
        data.forward(&model).expect("forward");
        let nv = usize::try_from(case["nv"].as_u64().expect("nv")).expect("fits");
        assert_eq!(model.nv, nv, "{name}: nv");
        for (i, c) in case["contacts"]
            .as_array()
            .expect("contacts")
            .iter()
            .enumerate()
        {
            if c["efc_address"].as_i64().expect("efc_address") < 0 {
                continue;
            }
            let frame = floats(&c["frame"]);
            let dim = usize::try_from(c["dim"].as_u64().expect("dim")).expect("fits");
            let contact = Contact {
                pos: vec3(&c["pos"]),
                normal: Vector3::from_column_slice(&frame[0..3]),
                depth: 0.0,
                geom: [side(&c["geom"], 0), side(&c["geom"], 1)],
                flex: [side(&c["flex"], 0), side(&c["flex"], 1)],
                elem: [side(&c["elem"], 0), side(&c["elem"], 1)],
                vert: [side(&c["vert"], 0), side(&c["vert"], 1)],
                friction: 1.0,
                dim,
                includemargin: 0.0,
                mu: [1.0, 1.0, 0.005, 0.0001, 0.0001],
                solref: [0.02, 1.0],
                solreffriction: [0.0, 0.0],
                solimp: [0.9, 0.95, 0.001, 0.5, 2.0],
                frame: [
                    Vector3::from_column_slice(&frame[3..6]),
                    Vector3::from_column_slice(&frame[6..9]),
                ],
                regularized_mu: 0.0,
            };
            let j = compute_contact_jacobian(&model, &data, &contact);
            let what = format!("{name} contact {i} (dim {dim})");
            if let Some(rows) = c["rows"].as_array() {
                for (r, row) in rows.iter().enumerate() {
                    let colind: Vec<usize> = row["colind"]
                        .as_array()
                        .expect("colind")
                        .iter()
                        .map(|x| usize::try_from(x.as_u64().expect("col")).expect("fits"))
                        .collect();
                    let values = floats(&row["values"]);
                    if r >= 3 {
                        // A rotational row: MuJoCo's dense one, which MuJoCo's
                        // sparse one is not (D-FLEX-SPARSE-ROT).
                        let twin = dense_twin.expect("a dense twin for a rotational row");
                        let dense = floats(&twin["contacts"][i]["efc_J"]);
                        let want = &dense[r * nv..(r + 1) * nv];
                        let ours_row: Vec<f64> = (0..nv).map(|col| j[(r, col)]).collect();
                        assert_bits(&format!("{what} row {r}, MuJoCo's dense"), &ours_row, want);
                        let sparse: Vec<f64> = (0..nv)
                            .map(|col| {
                                colind
                                    .iter()
                                    .position(|&c| c == col)
                                    .map_or(0.0, |k| values[k])
                            })
                            .collect();
                        assert_ne!(sparse, want, "{what} row {r}: drop D-FLEX-SPARSE-ROT");
                        continue;
                    }
                    let ours: Vec<f64> = colind.iter().map(|&col| j[(r, col)]).collect();
                    assert_bits(&format!("{what} row {r}"), &ours, &values);
                    for col in (0..nv).filter(|col| !colind.contains(col)) {
                        assert_eq!(
                            j[(r, col)],
                            0.0,
                            "{what} row {r}: column {col} off the chain"
                        );
                    }
                }
            } else {
                assert_bits(&what, &row_major(&j), &floats(&c["efc_J"]));
            }
        }
    }
}
