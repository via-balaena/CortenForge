//! A body's geoms against a flex, against MuJoCo 3.5.0.
//!
//! MuJoCo pairs a body with a flex in its broadphase, descends the two
//! hierarchies together (a world-welded body's planes first against every
//! vertex) or, without the midphase, takes each geom against every element,
//! and collides a geom with an element by type: raw functions for a sphere,
//! a capsule or a box against a cable's element or a triangle, a height
//! field's prisms, else the convex route (`engine_collision_driver.c:
//! 264-482`, `:622-847`, `:1658-1710`, `:1866-1998`). The golden is
//! `assets/golden/flex/flex_rigid.json`, from the unfused MuJoCo 3.5.0
//! oracle (`scripts/gen_flex_rigid_reference.py`, which describes its
//! cases). A signed distance field is sim-core's own route (registry
//! `D-FLEX-SDF`), tested against the sphere it samples.

use std::sync::Arc;

use nalgebra::{Point3, Vector3};
use serde_json::Value;
use sim_core::{ConstraintType, Data, GeomType, Model, SdfGrid, ShapeSphere};
use sim_mjcf::load_model;

fn golden() -> Value {
    serde_json::from_str(include_str!("../assets/golden/flex/flex_rigid.json"))
        .expect("flex rigid golden parses")
}

fn floats(v: &Value) -> Vec<f64> {
    v.as_array()
        .expect("array")
        .iter()
        .map(|x| x.as_f64().expect("number"))
        .collect()
}

fn usizes(v: &Value) -> Vec<usize> {
    v.as_array()
        .expect("array")
        .iter()
        .map(|x| usize::try_from(x.as_u64().expect("count")).expect("fits"))
        .collect()
}

/// MuJoCo's id for a contact side's object: -1 for none.
fn side(v: &Value, k: usize) -> Option<usize> {
    usize::try_from(v[k].as_i64().expect("id")).ok()
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

/// [`bits`] on macOS arm64, where the golden was made; elsewhere each entry
/// within `1e-12 · max(1, |MuJoCo's|)`. On CI's Linux every case's `forward`
/// and first step are MuJoCo's bit for bit, and four runs differ from the
/// second step in the last bits (at most 1.07e-13 of the value by step 20);
/// what differs between the platforms there has not been isolated.
fn run_bits(off: &mut Vec<String>, what: &str, got: &[f64], want: &[f64]) {
    if cfg!(all(target_os = "macos", target_arch = "aarch64")) {
        bits(off, what, got, want);
        return;
    }
    if got.len() != want.len() {
        off.push(format!(
            "{what}: length {}, MuJoCo {}",
            got.len(),
            want.len()
        ));
        return;
    }
    for (i, (a, b)) in got.iter().zip(want).enumerate() {
        if (a - b).abs() > 1e-12 * b.abs().max(1.0) {
            off.push(format!(
                "{what}[{i}]: {a:e}, MuJoCo {b:e} (off {:e})",
                a - b
            ));
        }
    }
}

/// Where `got` differs from `want` in any bit, each as a line in `off`.
fn bits(off: &mut Vec<String>, what: &str, got: &[f64], want: &[f64]) {
    if got.len() != want.len() {
        off.push(format!(
            "{what}: length {}, MuJoCo {}",
            got.len(),
            want.len()
        ));
        return;
    }
    for (i, (a, b)) in got.iter().zip(want).enumerate() {
        if a.to_bits() != b.to_bits() {
            off.push(format!(
                "{what}[{i}]: {a:e}, MuJoCo {b:e} (off {:e})",
                a - b
            ));
        }
    }
}

fn v3(v: &Vector3<f64>) -> [f64; 3] {
    [v.x, v.y, v.z]
}

/// Each contact as MuJoCo lists it: its sides, distance, position, frame,
/// dimension, margin less gap, friction, solref, solimp, and its `exclude`
/// (1 in the gap, 4 of a passive flex, else 0).
fn contacts(off: &mut Vec<String>, name: &str, model: &Model, data: &Data, want: &[Value]) {
    if data.contacts.len() != want.len() {
        off.push(format!(
            "{name}: ncon {}, MuJoCo {}",
            data.contacts.len(),
            want.len()
        ));
        return;
    }
    for (i, (c, w)) in data.contacts.iter().zip(want).enumerate() {
        let what = format!("{name} contact {i}");
        let ids = |key: &str| [side(&w[key], 0), side(&w[key], 1)];
        let sides = (c.geom, c.flex, c.elem, c.vert);
        let mj = (ids("geom"), ids("flex"), ids("elem"), ids("vert"));
        if sides != mj {
            off.push(format!("{what}: sides {sides:?}, MuJoCo {mj:?}"));
            continue;
        }
        bits(
            off,
            &format!("{what} dist"),
            &[-c.depth],
            &[w["dist"].as_f64().expect("dist")],
        );
        bits(off, &format!("{what} pos"), &v3(&c.pos), &floats(&w["pos"]));
        let frame: Vec<f64> = [c.normal, c.frame[0], c.frame[1]]
            .iter()
            .flat_map(v3)
            .collect();
        bits(off, &format!("{what} frame"), &frame, &floats(&w["frame"]));
        let dim = usize::try_from(w["dim"].as_u64().expect("dim")).expect("fits");
        if c.dim != dim {
            off.push(format!("{what}: dim {}, MuJoCo {dim}", c.dim));
        }
        let includemargin = w["includemargin"].as_f64().expect("includemargin");
        bits(
            off,
            &format!("{what} includemargin"),
            &[c.includemargin],
            &[includemargin],
        );
        bits(
            off,
            &format!("{what} friction"),
            &c.mu,
            &floats(&w["friction"]),
        );
        bits(
            off,
            &format!("{what} solref"),
            &c.solref,
            &floats(&w["solref"]),
        );
        bits(
            off,
            &format!("{what} solimp"),
            &c.solimp,
            &floats(&w["solimp"]),
        );
        let exclude = w["exclude"].as_i64().expect("exclude");
        let ours = if c.is_passive(model) {
            4
        } else {
            i64::from(c.is_excluded())
        };
        if ours != exclude {
            off.push(format!("{what}: exclude {ours}, MuJoCo {exclude}"));
        }
    }
}

/// The constraint rows: their types, Jacobian (at the sparse size each
/// row's columns and its values there), R and D.
fn rows(off: &mut Vec<String>, name: &str, model: &Model, data: &Data, case: &Value) {
    let nefc = usize::try_from(case["nefc"].as_u64().expect("nefc")).expect("fits");
    if data.efc_type.len() != nefc {
        off.push(format!(
            "{name}: nefc {}, MuJoCo {nefc}",
            data.efc_type.len()
        ));
        return;
    }
    let types: Vec<usize> = data.efc_type.iter().map(|&t| mj_type(t)).collect();
    if types != usizes(&case["efc_type"]) {
        off.push(format!(
            "{name}: row types {types:?}, MuJoCo {:?}",
            case["efc_type"]
        ));
    }
    if let Some(sparse) = case["efc_J_rows"].as_array() {
        for (r, row) in sparse.iter().enumerate() {
            let span = data.efc_J_rowadr[r]..data.efc_J_rowadr[r] + data.efc_J_rownnz[r];
            let colind = &data.efc_J_colind[span];
            if colind != usizes(&row["colind"]).as_slice() {
                off.push(format!(
                    "{name} row {r}: columns {colind:?}, MuJoCo {:?}",
                    row["colind"]
                ));
                continue;
            }
            let values: Vec<f64> = colind.iter().map(|&c| data.efc_J[(r, c)]).collect();
            bits(
                off,
                &format!("{name} row {r} efc_J"),
                &values,
                &floats(&row["values"]),
            );
        }
    } else {
        let j: Vec<f64> = (0..nefc)
            .flat_map(|r| (0..model.nv).map(move |c| (r, c)))
            .map(|rc| data.efc_J[rc])
            .collect();
        bits(off, &format!("{name} efc_J"), &j, &floats(&case["efc_J"]));
    }
    bits(
        off,
        &format!("{name} efc_R"),
        &data.efc_R,
        &floats(&case["efc_R"]),
    );
    bits(
        off,
        &format!("{name} efc_D"),
        &data.efc_D,
        &floats(&case["efc_D"]),
    );
}

/// From MuJoCo's `qpos0` and controls, each case's contacts and rows after
/// `forward`, then `qacc`, `qfrc_constraint`, `qfrc_spring` and
/// `qfrc_passive`, and qpos, qvel and the contact count at the golden's
/// checkpoints of a run, each bit for bit with MuJoCo's (a run's qpos and
/// qvel as [`run_bits`]). A passive flex's contacts make no rows and push as
/// springs (`mj_nc`'s exclude 4, `mj_contactPassive`).
#[test]
fn a_body_s_geoms_collide_with_a_flex_as_mujoco() {
    let golden = golden();
    let mut off = Vec::new();
    for case in golden["cases"].as_array().expect("cases") {
        let name = case["name"].as_str().expect("name");
        let model = load_model(case["ours"].as_str().expect("ours")).expect("load");
        let nv = usize::try_from(case["nv"].as_u64().expect("nv")).expect("fits");
        assert_eq!(model.nv, nv, "{name}: nv");
        let mut data = model.make_data();
        // The fixture: the state the golden starts from.
        let qpos0 = floats(&case["qpos0"]);
        let start: Vec<u64> = data.qpos.iter().map(|x| x.to_bits()).collect();
        let want: Vec<u64> = qpos0.iter().map(|x| x.to_bits()).collect();
        if start != want {
            off.push(format!("{name}: qpos0 {:?}, MuJoCo's {qpos0:?}", data.qpos));
            continue;
        }
        if let Some(ctrl) = case.get("ctrl") {
            data.ctrl.copy_from_slice(&floats(ctrl));
        }
        data.forward(&model).expect("forward");
        contacts(
            &mut off,
            name,
            &model,
            &data,
            case["contacts"].as_array().expect("contacts"),
        );
        rows(&mut off, name, &model, &data, case);
        bits(
            &mut off,
            &format!("{name} qacc"),
            data.qacc.as_slice(),
            &floats(&case["qacc"]),
        );
        for (key, ours) in [
            ("qfrc_constraint", &data.qfrc_constraint),
            ("qfrc_spring", &data.qfrc_spring),
            ("qfrc_passive", &data.qfrc_passive),
        ] {
            bits(
                &mut off,
                &format!("{name} {key}"),
                ours.as_slice(),
                &floats(&case[key]),
            );
        }
        let trajectory = case["trajectory"].as_object().expect("trajectory");
        let last = trajectory
            .keys()
            .map(|k| k.parse::<usize>().expect("step"))
            .max();
        for step in 1..=last.expect("a checkpoint") {
            data.step(&model).expect("step");
            if let Some(c) = trajectory.get(&step.to_string()) {
                let ncon = usize::try_from(c["ncon"].as_u64().expect("ncon")).expect("fits");
                if data.contacts.len() != ncon {
                    off.push(format!(
                        "{name} step {step}: ncon {}, MuJoCo {ncon}",
                        data.contacts.len()
                    ));
                }
                run_bits(
                    &mut off,
                    &format!("{name} step {step} qpos"),
                    data.qpos.as_slice(),
                    &floats(&c["qpos"]),
                );
                run_bits(
                    &mut off,
                    &format!("{name} step {step} qvel"),
                    data.qvel.as_slice(),
                    &floats(&c["qvel"]),
                );
            }
        }
    }
    assert!(
        off.is_empty(),
        "{} values differ:\n{}",
        off.len(),
        off.join("\n")
    );
}

/// A cloth of nine vertices (a 3×3 grid, 5 cm apart, radius 1 cm, margin
/// 5 mm) whose middle sits 5 cm above the centre of a world sphere of radius
/// 6.5 cm, the sphere's own attributes `bits`; a far box of the world's
/// lets the world body pair with the flex whatever the sphere's bits.
fn cloth_over_a_sphere(midphase: bool, bits: &str) -> (Model, Data) {
    let flag = if midphase {
        ""
    } else {
        r#"<flag midphase="disable"/>"#
    };
    let xml = format!(
        r#"<mujoco>
          <option gravity="0 0 0">{flag}</option>
          <worldbody>
            <geom type="sphere" size="0.065" pos="0 0 -0.05" {bits}/>
            <geom type="box" size="0.01 0.01 0.01" pos="1 1 1"/>
          </worldbody>
          <deformable>
            <flex name="c" dim="2" radius="0.01">
              <contact selfcollide="none" internal="false" margin="0.005"/>
              <vertex pos="-0.05 -0.05 0  0 -0.05 0  0.05 -0.05 0  -0.05 0 0  0 0 0  0.05 0 0
                           -0.05 0.05 0  0 0.05 0  0.05 0.05 0"/>
              <element data="0 1 3  1 4 3  1 2 4  2 5 4  3 4 6  4 7 6  4 5 7  5 8 7"/>
            </flex>
          </deformable>
        </mujoco>"#
    );
    let mut model = load_model(&xml).expect("load");
    let grid = Arc::new(SdfGrid::sphere(Point3::origin(), 0.065, 32, 0.02));
    model
        .shape_data
        .push(Arc::new(ShapeSphere::new(grid, 0.065)));
    model.nshape = model.shape_data.len();
    model.geom_type[0] = GeomType::Sdf;
    model.geom_shape[0] = Some(model.nshape - 1);
    let mut data = model.make_data();
    data.forward(&model).expect("forward");
    (model, data)
}

/// A signed distance field against a flex (registry `D-FLEX-SDF`): MuJoCo
/// collides the field's mesh hull with each element through the convex
/// route; sim-core's field has no hull, so each vertex, a sphere of the
/// flex's radius, meets the field within the margin. A cloth over a field
/// sphere: a contact for each vertex within the radius and margin of the
/// surface (its middle, 1.5 cm in, and the four at its edges' middles,
/// 0.57 cm off), none at its corners (2.2 cm off), each listing the field
/// and the vertex, none an element; its distance the vertex's from the
/// sphere less the radius (the margin not in it), to the field's grid; the
/// same list, bit for bit, with the midphase on and off; none where the
/// field's bitmask misses the flex's.
#[test]
fn a_signed_distance_field_meets_each_flex_vertex() {
    let (model, data) = cloth_over_a_sphere(true, "");
    let verts: Vec<usize> = data.contacts.iter().filter_map(|c| c.vert[1]).collect();
    assert_eq!(verts, [1, 3, 4, 5, 7], "the vertices within reach");
    for c in &data.contacts {
        assert_eq!(c.geom, [Some(0), None], "the field first");
        assert_eq!(c.flex, [None, Some(0)]);
        assert_eq!(c.elem, [None, None], "no element");
        let v = data.flexvert_xpos[model.flex_vertadr[0] + c.vert[1].expect("a vertex")];
        let want = (v - Vector3::new(0.0, 0.0, -0.05)).norm() - 0.065 - 0.01;
        assert!(
            (-c.depth - want).abs() < 2e-4,
            "dist {}, the sphere's {want}",
            -c.depth
        );
    }
    let (_, all_to_all) = cloth_over_a_sphere(false, "");
    let key = |d: &Data| -> Vec<(Option<usize>, u64, [u64; 3])> {
        d.contacts
            .iter()
            .map(|c| (c.vert[1], c.depth.to_bits(), c.pos.map(f64::to_bits).into()))
            .collect()
    };
    assert_eq!(key(&data), key(&all_to_all), "the midphase on and off");
    for midphase in [true, false] {
        let (_, missed) = cloth_over_a_sphere(midphase, r#"contype="2" conaffinity="2""#);
        assert!(
            missed.contacts.is_empty(),
            "midphase {midphase}: the bitmask"
        );
    }
}

/// A geom turned by a NaN against a flex: each call returns, with the
/// midphase on and off, for a height field, a box, an ellipsoid, a cylinder
/// and a signed distance field (which collides as nothing there, its turn
/// not a quaternion).
#[test]
fn a_non_finite_turn_against_a_flex_returns() {
    use nalgebra::{Quaternion, UnitQuaternion};
    let cloth = r#"<deformable><flex name="c" dim="2" radius="0.01">
      <contact selfcollide="none" internal="false"/>
      <vertex pos="0 0 0.005  0.05 0 0.005  0.1 0 0.005  0 0.05 0.005  0.05 0.05 0.005  0.1 0.05 0.005
                   0 0.1 0.005  0.05 0.1 0.005  0.1 0.1 0.005"/>
      <element data="0 1 3  1 4 3  1 2 4  2 5 4  3 4 6  4 7 6  4 5 7  5 8 7"/>
    </flex></deformable>"#;
    let geoms = [
        (
            "hfield",
            r#"<asset><hfield name="h" nrow="3" ncol="3" size="1 1 0.05 0.05" elevation="1 1 1 1 1 1 1 1 1"/></asset>"#,
            r#"<geom type="hfield" hfield="h"/>"#,
        ),
        (
            "box",
            "",
            r#"<geom type="box" size="1 1 0.01" pos="0 0 -0.005"/>"#,
        ),
        (
            "ellipsoid",
            "",
            r#"<geom type="ellipsoid" size="0.1 0.1 0.02" pos="0.05 0.05 -0.015"/>"#,
        ),
        (
            "cylinder",
            "",
            r#"<geom type="cylinder" size="0.1 0.02" pos="0.05 0.05 -0.015"/>"#,
        ),
        (
            "sdf",
            "",
            r#"<geom type="sphere" size="0.1" pos="0.05 0.05 -0.09"/>"#,
        ),
    ];
    for (name, asset, geom) in geoms {
        for midphase in [true, false] {
            let flag = if midphase {
                ""
            } else {
                r#"midphase="disable""#
            };
            let xml = format!(
                r#"<mujoco><option><flag equality="disable" {flag}/></option>{asset}
                   <worldbody>{geom}</worldbody>{cloth}</mujoco>"#
            );
            let mut model = load_model(&xml).expect("load");
            if name == "sdf" {
                let grid = Arc::new(SdfGrid::sphere(Point3::origin(), 0.1, 16, 0.02));
                model.shape_data.push(Arc::new(ShapeSphere::new(grid, 0.1)));
                model.nshape = model.shape_data.len();
                model.geom_type[0] = GeomType::Sdf;
                model.geom_shape[0] = Some(model.nshape - 1);
            }
            let mut data = model.make_data();
            data.forward(&model).expect("forward, unturned");
            assert!(
                data.ncon > 0,
                "{name} midphase {midphase}: the cloth touches the geom"
            );
            model.geom_quat[0] =
                UnitQuaternion::new_unchecked(Quaternion::new(f64::NAN, 0.0, 0.0, 0.0));
            model.geom_sameframe[0] = Default::default();
            // Each call returns; its result is not asserted.
            let _ = data.forward(&model);
            if name == "sdf" {
                assert_eq!(data.ncon, 0, "{name} midphase {midphase}: nothing");
            }
        }
    }
}

/// A flex's `<contact passive>` (registry `D-FLEX-PASSIVE-READ`): MuJoCo
/// 3.5.0's reader overwrites the flag with a stale variable whenever the
/// attribute is given (`xml_native_reader.cc:1457-1523`), here the
/// `internal` flag's value, so the golden's passive cases,
/// `internal="false" passive="true"`, read as not passive there (the
/// generator sets the flag on MuJoCo's compiled model). sim-mjcf reads the
/// attribute as written.
#[test]
fn a_flex_s_passive_attribute_reads_as_written() {
    let golden = golden();
    let cases: Vec<&Value> = golden["cases"]
        .as_array()
        .expect("cases")
        .iter()
        .filter(|c| c.get("flex_passive_read").is_some())
        .collect();
    assert!(!cases.is_empty(), "the golden's passive cases");
    for case in cases {
        let name = case["name"].as_str().expect("name");
        let model = load_model(case["ours"].as_str().expect("ours")).expect("load");
        assert_eq!(
            model.flex_passive,
            [false, true],
            "{name}: ours, as written"
        );
        assert_eq!(
            usizes(&case["flex_passive_read"]),
            [0, 0],
            "{name}: MuJoCo's reader, its internal flag"
        );
    }
}
