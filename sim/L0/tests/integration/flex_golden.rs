//! The flex collision goldens (`assets/golden/flex/flex_rigid.json` and
//! `flex_self.json`, from the unfused MuJoCo 3.5.0 oracle by
//! `scripts/gen_flex_rigid_reference.py` and `gen_flex_self_reference.py`)
//! and their comparison with sim-core.

use nalgebra::Vector3;
use serde_json::Value;
use sim_core::{ConstraintType, Data, Model};
use sim_mjcf::load_model;

pub(crate) fn floats(v: &Value) -> Vec<f64> {
    v.as_array()
        .expect("array")
        .iter()
        .map(|x| x.as_f64().expect("number"))
        .collect()
}

pub(crate) fn usizes(v: &Value) -> Vec<usize> {
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
/// within `1e-12 · max(1, |MuJoCo's|)`, and a NaN never. In CI run
/// 39766239505 (Linux) four cases' checkpoints after the first differed by
/// up to 1.07e-13 of the value (step 20), past the 1e-13 the repository's
/// other platform gates take; what differs between the platforms has not
/// been isolated (ledger-L231). No CI job runs this crate on macOS, so the
/// bit-for-bit branch runs locally only.
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
        // Within the bound or pushed: a NaN is never within it.
        if (a - b).abs() <= 1e-12 * b.abs().max(1.0) {
            continue;
        }
        off.push(format!(
            "{what}[{i}]: {a:e}, MuJoCo {b:e} (off {:e})",
            a - b
        ));
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

/// Where sim-core differs from each case of `golden`, a line each: from
/// MuJoCo's `qpos0` and controls, the case's contacts and rows after
/// `forward`, then `qacc`, `qfrc_constraint`, `qfrc_spring` and
/// `qfrc_passive`, and qpos, qvel and the contact count at the golden's
/// checkpoints of a run, each bit for bit with MuJoCo's (a run's qpos and
/// qvel after the first step as [`run_bits`]). A passive flex's contacts
/// make no rows and push as springs (`mj_nc`'s exclude 4,
/// `mj_contactPassive`).
pub(crate) fn differences(golden: &Value) -> Vec<String> {
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
                let compare = if step == 1 { bits } else { run_bits };
                let ncon = usize::try_from(c["ncon"].as_u64().expect("ncon")).expect("fits");
                if data.contacts.len() != ncon {
                    off.push(format!(
                        "{name} step {step}: ncon {}, MuJoCo {ncon}",
                        data.contacts.len()
                    ));
                }
                compare(
                    &mut off,
                    &format!("{name} step {step} qpos"),
                    data.qpos.as_slice(),
                    &floats(&c["qpos"]),
                );
                compare(
                    &mut off,
                    &format!("{name} step {step} qvel"),
                    data.qvel.as_slice(),
                    &floats(&c["qvel"]),
                );
            }
        }
    }
    off
}
