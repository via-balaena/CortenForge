//! Muscles against MuJoCo 3.5.0: a muscle's force stays -1 in the model and
//! is resolved as `scale / max(mjMINVAL, acc0)` at each call, the gain from
//! `gainprm` and the bias from `biasprm` (`mju_muscleGain`, `mju_muscleBias`,
//! `engine_util_misc.c:653-725`), each divisor clamped at `mjMINVAL`.
//!
//! The golden is `assets/golden/muscle/muscle.json`, from the unfused MuJoCo
//! 3.5.0 oracle (`scripts/gen_muscle_reference.py`, which describes its
//! fixtures). sim-mjcf does not read `lengthrange` yet (ledger L44), so each
//! test sets MuJoCo's after loading.

use serde_json::Value;
use sim_mjcf::load_model;

fn golden() -> Value {
    serde_json::from_str(include_str!("../assets/golden/muscle/muscle.json"))
        .expect("muscle golden parses")
}

fn floats(v: &Value) -> Vec<f64> {
    v.as_array()
        .expect("array")
        .iter()
        .map(|x| x.as_f64().expect("number"))
        .collect()
}

fn num(v: &Value) -> f64 {
    v.as_f64().expect("number")
}

/// Fixture `name` at each of its states: `actuator_force` and
/// `qfrc_actuator` after a forward pass within `tol` of MuJoCo's.
fn assert_matches_mujoco(name: &str, tol: f64) {
    let golden = golden();
    let case = golden["fixtures"]
        .as_array()
        .expect("fixtures")
        .iter()
        .find(|c| c["name"] == name)
        .unwrap_or_else(|| panic!("no fixture {name}"));
    let mut model = load_model(case["xml"].as_str().expect("xml")).expect("load");
    let lr = floats(&case["lengthrange"]);
    model.actuator_lengthrange[0] = (lr[0], lr[1]);
    assert!(
        (model.actuator_acc0[0] - num(&case["acc0"])).abs() <= 1e-12,
        "{name}: acc0 {}, MuJoCo {}",
        model.actuator_acc0[0],
        num(&case["acc0"])
    );
    for state in case["states"].as_array().expect("states") {
        let mut data = model.make_data();
        data.qpos[0] = num(&state["qpos"]);
        data.qvel[0] = num(&state["qvel"]);
        data.ctrl[0] = num(&state["ctrl"]);
        data.forward(&model).expect("forward");
        for (field, got) in [
            ("actuator_force", data.actuator_force.as_slice()),
            ("qfrc_actuator", data.qfrc_actuator.as_slice()),
        ] {
            for (a, b) in got.iter().zip(floats(&state[field])) {
                assert!(
                    (a - b).abs() <= tol,
                    "{name} at qpos {}: {field} ours {a}, MuJoCo {b}",
                    state["qpos"]
                );
            }
        }
    }
}

/// A general actuator with a muscle gain and bias and no muscle dynamics
/// keeps its force of -1 and resolves it at each call (A20 §1A.3.8): the
/// force was resolved only for a muscle's dynamics, so this one read -1.
#[test]
fn general_muscle_f0_matches_mujoco_3_5_0() {
    assert_matches_mujoco("auto_force", 1e-12);
}

/// A muscle bias reads its own `biasprm`: the gain's force 300, the bias's
/// 100, past the optimal length.
#[test]
fn general_biastype_muscle_reads_biasprm() {
    assert_matches_mujoco("bias_from_biasprm", 1e-12);
}

/// The muscle's divisors are clamped at mjMINVAL, as MuJoCo's: a length
/// range 1e-12 wide gives an optimal length of 3.3e-12, which a clamp at
/// 1e-10 replaced (the forces are explicit, so only the clamps differ).
#[test]
fn muscle_clamps_use_mjminval() {
    assert_matches_mujoco("narrow_range", 1e-12);
}

/// A muscle's `acc0` is stored unclamped, 3.1e-17 here (a gear of 1e-18), and
/// clamped at mjMINVAL where its force is resolved (`engine_setconst.c:776`,
/// `engine_util_misc.c:666`); a clamp at 1e-10 where it is stored made the
/// force 5e5 times too small.
#[test]
fn tiny_acc0_is_stored_unclamped() {
    assert_matches_mujoco("tiny_acc0", 0.0);
}

/// A muscle's `acc0` sums its squares in MuJoCo's order (`mju_norm`, four
/// interleaved sums): over five slides, 1.754197806780044 where a sum left to
/// right gives 1.7541978067800443.
#[test]
fn acc0_sums_in_mujoco_order() {
    let golden = golden();
    let case = golden["fixtures"]
        .as_array()
        .expect("fixtures")
        .iter()
        .find(|c| c["name"] == "acc0_order")
        .expect("acc0_order");
    let model = load_model(case["xml"].as_str().expect("xml")).expect("load");
    assert_eq!(
        model.actuator_acc0[0].to_bits(),
        num(&case["acc0"]).to_bits()
    );
    assert_matches_mujoco("acc0_order", 0.0);
}
