//! The body spatial quantities against MuJoCo 3.5.0, which takes them about
//! each tree's subtree centre of mass (`subtree_com[body_rootid[b]]`) in the
//! world's orientation: the dofs' motion axes `cdof` (`mj_comPos`,
//! `engine_core_smooth.c:245-350`), the body velocities `cvel` and the axes'
//! rates `cdof_dot` (`mj_comVel`, `:2276-2343`), and what reads `cvel`: the
//! object velocities and the subtree velocities.
//!
//! The golden is `assets/golden/com_frame/com_frame.json`, from the unfused
//! MuJoCo 3.5.0 oracle (`scripts/gen_com_frame_reference.py`, which describes
//! its cases). Each test first checks that what the quantity is made from,
//! the kinematics, is MuJoCo's, so a difference there is not read as one in
//! the frame.

use serde_json::Value;
use sim_core::dynamics::object_velocity;
use sim_core::{Data, Model, SleepState};
use sim_mjcf::load_model;

const CASES: [&str; 7] = [
    "compos",
    "chain",
    "slow",
    "nogravity",
    "simple",
    "mocap",
    "sleeping",
];

fn golden() -> Value {
    serde_json::from_str(include_str!("../assets/golden/com_frame/com_frame.json"))
        .expect("com frame golden parses")
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

/// [`assert_bits`] on macOS arm64, where the golden was made. The moved
/// states turn hinges through `sin` and `cos`, which another platform's libm
/// may round otherwise by an ulp; there each entry is within
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

fn vec3s(v: &[sim_core::Vector3<f64>]) -> Vec<f64> {
    v.iter().flat_map(|p| [p.x, p.y, p.z]).collect()
}

fn mat3s(v: &[sim_core::Matrix3<f64>]) -> Vec<f64> {
    v.iter()
        .flat_map(|m| (0..9).map(move |k| m[(k / 3, k % 3)]))
        .collect()
}

fn spatial(v: &[sim_core::SpatialVector]) -> Vec<f64> {
    v.iter().flat_map(|s| s.iter().copied()).collect()
}

/// The case's model at its moved state after `forward`, with the model's
/// trees, simple dofs, sleep states, masses and inertias checked against
/// MuJoCo's, and the kinematics every quantity here is made from checked
/// against MuJoCo's.
fn forward(name: &str) -> (Model, Data, Value) {
    let case = golden()["cases"][name].clone();
    let model = load_model(case["xml"].as_str().expect("xml")).expect("load");
    let mut data = model.make_data();
    data.qpos.copy_from_slice(&floats(&case["qpos"]));
    data.qvel.copy_from_slice(&floats(&case["qvel"]));
    data.forward(&model).expect("forward");

    assert_eq!(
        model.body_rootid,
        usizes(&case["body_rootid"]),
        "{name}: body_rootid"
    );
    assert_eq!(
        model.dof_simplenum,
        usizes(&case["dof_simplenum"]),
        "{name}: dof_simplenum"
    );
    let awake: Vec<i64> = data
        .body_sleep_state
        .iter()
        .map(|s| match s {
            SleepState::Static => -1,
            SleepState::Asleep => 0,
            SleepState::Awake => 1,
        })
        .collect();
    let want: Vec<i64> = case["body_awake"]
        .as_array()
        .expect("body_awake")
        .iter()
        .map(|x| x.as_i64().expect("state"))
        .collect();
    assert_eq!(awake, want, "{name}: body sleep states");

    let iquat: Vec<f64> = model
        .body_iquat
        .iter()
        .flat_map(|q| [q.w, q.i, q.j, q.k])
        .collect();
    assert_bits(
        &format!("{name} body_mass"),
        &model.body_mass,
        &floats(&case["body_mass"]),
    );
    assert_bits(
        &format!("{name} body_inertia"),
        &vec3s(&model.body_inertia),
        &floats(&case["body_inertia"]),
    );
    assert_bits(
        &format!("{name} body_ipos"),
        &vec3s(&model.body_ipos),
        &floats(&case["body_ipos"]),
    );
    assert_bits(
        &format!("{name} body_iquat"),
        &iquat,
        &floats(&case["body_iquat"]),
    );
    assert_bits(
        &format!("{name} body_subtreemass"),
        &model.body_subtreemass,
        &floats(&case["body_subtreemass"]),
    );

    assert_golden(
        &format!("{name} xpos"),
        &vec3s(&data.xpos),
        &floats(&case["xpos"]),
    );
    assert_golden(
        &format!("{name} xmat"),
        &mat3s(&data.xmat),
        &floats(&case["xmat"]),
    );
    assert_golden(
        &format!("{name} xipos"),
        &vec3s(&data.xipos),
        &floats(&case["xipos"]),
    );
    assert_golden(
        &format!("{name} ximat"),
        &mat3s(&data.ximat),
        &floats(&case["ximat"]),
    );
    assert_golden(
        &format!("{name} xanchor"),
        &vec3s(&data.xanchor),
        &floats(&case["xanchor"]),
    );
    assert_golden(
        &format!("{name} xaxis"),
        &vec3s(&data.xaxis),
        &floats(&case["xaxis"]),
    );
    assert_golden(
        &format!("{name} subtree_com"),
        &vec3s(&data.subtree_com),
        &floats(&case["subtree_com"]),
    );
    (model, data, case)
}

/// Each dof's motion axis, `[axis; axis × (subtree_com[root] − anchor)]` for
/// a rotation and `[0; axis]` for a translation, is MuJoCo's bit for bit:
/// hinges and slides on their own and sharing a body, balls, free joints,
/// simple bodies, and a sleeping tree's, which MuJoCo computed before the
/// tree slept.
#[test]
fn cdof_is_mujocos() {
    for name in CASES {
        let (_, data, case) = forward(name);
        assert_golden(
            &format!("{name} cdof"),
            &spatial(&data.cdof),
            &floats(&case["cdof"]),
        );
    }
}

/// Each body's spatial velocity `[angular; linear]` about its tree's subtree
/// centre of mass (`cvel`) and each dof's axis rate (`cdof_dot`) are
/// MuJoCo's bit for bit (`mj_comVel`, `engine_core_smooth.c:2276-2343`), a
/// sleeping tree's included.
#[test]
fn cvel_and_cdof_dot_are_mujocos() {
    for name in CASES {
        let (_, data, case) = forward(name);
        assert_golden(
            &format!("{name} cvel"),
            &spatial(&data.cvel),
            &floats(&case["cvel"]),
        );
        assert_golden(
            &format!("{name} cdof_dot"),
            &spatial(&data.cdof_dot),
            &floats(&case["cdof_dot"]),
        );
    }
}

/// Each body's velocity at its centre of mass, in the world's orientation
/// and in its inertial frame's, is MuJoCo's `mj_objectVelocity` bit for bit
/// (`engine_core_util.c:720-774`): `cvel` moved from the tree's subtree
/// centre of mass to the point; 0 for the world, a static and a mocap body.
#[test]
fn object_velocities_are_mujocos() {
    for name in CASES {
        let (model, data, case) = forward(name);
        let mut world = Vec::new();
        let mut local = Vec::new();
        for b in 0..model.nbody {
            for (rot, out) in [(None, &mut world), (Some(&data.ximat[b]), &mut local)] {
                let (w, v) = object_velocity(&model, &data, b, &data.xipos[b], rot);
                out.extend([w.x, w.y, w.z, v.x, v.y, v.z]);
            }
        }
        assert_golden(&format!("{name} objvel"), &world, &floats(&case["objvel"]));
        assert_golden(
            &format!("{name} objvel_local"),
            &local,
            &floats(&case["objvel_local"]),
        );
    }
}

/// The subtree velocities and angular momenta are MuJoCo's bit for bit
/// (`mj_subtreeVel`, `engine_core_smooth.c:2346-2420`), which read each
/// body's velocity at its centre of mass: the cases whose subtree sensors
/// make the forward pass compute them.
#[test]
fn subtree_velocities_are_mujocos() {
    for name in ["compos", "slow", "nogravity"] {
        let (_, data, case) = forward(name);
        assert_golden(
            &format!("{name} subtree_linvel"),
            &vec3s(&data.subtree_linvel),
            &floats(&case["subtree_linvel"]),
        );
        assert_golden(
            &format!("{name} subtree_angmom"),
            &vec3s(&data.subtree_angmom),
            &floats(&case["subtree_angmom"]),
        );
    }
}
