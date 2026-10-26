//! Muscles against MuJoCo 3.5.0: a muscle's force stays -1 in the model and
//! is resolved as `scale / max(mjMINVAL, acc0)` at each call, the gain from
//! `gainprm` and the bias from `biasprm` (`mju_muscleGain`, `mju_muscleBias`,
//! `engine_util_misc.c:653-725`), each divisor clamped at `mjMINVAL`.
//!
//! The golden is `assets/golden/muscle/muscle.json`, from the unfused MuJoCo
//! 3.5.0 oracle (`scripts/gen_muscle_reference.py`, which describes its
//! fixtures). sim-mjcf does not read `lengthrange` yet (ledger L44), so each
//! test sets MuJoCo's after loading; `a_written_length_range_is_not_read_yet`
//! fails once it does.

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
    assert_matches_mujoco("auto_force", 0.0);
}

/// A muscle bias reads its own `biasprm`: the gain's force 300, the bias's
/// 100, at ten states, nine past the optimal length; with a control of 0 the
/// force is the passive force alone, multiplied in MuJoCo's order.
#[test]
fn general_biastype_muscle_reads_biasprm() {
    assert_matches_mujoco("bias_from_biasprm", 0.0);
}

/// The muscle's divisors are clamped at mjMINVAL, as MuJoCo's: a length
/// range 1e-12 wide gives an optimal length of 3.3e-12, which a clamp at
/// 1e-10 replaced (the forces are explicit, so only the clamps differ).
#[test]
fn muscle_clamps_use_mjminval() {
    assert_matches_mujoco("narrow_range", 0.0);
}

/// A muscle's `acc0` of 3.1e-17 (a gear of 1e-18) is raised to mjMINVAL
/// where its force is resolved (`engine_util_misc.c:666`); a clamp at 1e-10
/// where it was stored made the force 1e5 times too small. This pins an
/// `acc0` at or below mjMINVAL; that it is stored unclamped
/// (`engine_setconst.c:776`), `test_acc0_no_dofs` and
/// `test_muscle_acc0_spatial_tendon` pin (an `acc0` of 0).
#[test]
fn tiny_acc0_force_uses_mjminval() {
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

/// An fvmax of 1 + 5e-11: the force-velocity curve divides by
/// max(mjMINVAL, fvmax - 1), at normalized velocities inside (0, 5e-11); and
/// one step under each implicit integrator, whose velocity derivative
/// divides by it too.
#[test]
fn force_velocity_curve_clamps_at_mjminval() {
    assert_matches_mujoco("tiny_fv", 0.0);
    assert_implicit_steps("tiny_fv");
}

/// The narrow range's velocity derivative divides by the clamped optimal
/// length: one step under each implicit integrator.
#[test]
fn narrow_range_steps_implicitly_as_mujoco_3_5_0() {
    assert_implicit_steps("narrow_range");
}

/// An lmax of 1 + 1e-10: the length curve and the passive force divide by
/// max(mjMINVAL, b - 1), b their midpoint, 5e-11 past 1.
#[test]
fn length_curves_clamp_at_mjminval() {
    assert_matches_mujoco("flat_lmax", 0.0);
}

/// A force of 0 stays 0: only a negative force is resolved from `acc0`.
#[test]
fn a_zero_force_is_not_resolved() {
    assert_matches_mujoco("zero_force", 0.0);
}

/// Fixture `name`'s steps under implicit and implicitfast from each of its
/// states: `qpos` and `qvel`, bit for bit.
fn assert_implicit_steps(name: &str) {
    let golden = golden();
    let case = golden["fixtures"]
        .as_array()
        .expect("fixtures")
        .iter()
        .find(|c| c["name"] == name)
        .unwrap_or_else(|| panic!("no fixture {name}"));
    let lr = floats(&case["lengthrange"]);
    let states = case["states"].as_array().expect("states");
    let steps = case["implicit_steps"].as_array().expect("implicit_steps");
    assert_eq!(
        steps.len(),
        2 * states.len(),
        "{name}: implicit and implicitfast"
    );
    for step in steps {
        let state =
            &states[usize::try_from(step["state"].as_u64().expect("state")).expect("index")];
        let mut model = load_model(step["xml"].as_str().expect("xml")).expect("load");
        model.actuator_lengthrange[0] = (lr[0], lr[1]);
        let mut data = model.make_data();
        data.qpos[0] = num(&state["qpos"]);
        data.qvel[0] = num(&state["qvel"]);
        data.ctrl[0] = num(&state["ctrl"]);
        data.step(&model).expect("step");
        for (field, got) in [
            ("qpos", data.qpos.as_slice()),
            ("qvel", data.qvel.as_slice()),
        ] {
            assert_eq!(
                got,
                floats(&step[field]).as_slice(),
                "{name} under {}: {field}",
                step["integrator"]
            );
        }
    }
}

/// sim-mjcf drops a written `lengthrange` (ledgers L37, L272): a muscle on an
/// unlimited hinge, its range written, loads with (0, 0), where MuJoCo 3.5.0
/// compiles the written (-1.5, 1.5) (and refuses the model without it: the
/// range's simulation does not converge). Rigid-loading's L44 keeps the
/// written range, and this test fails there.
#[test]
fn a_written_length_range_is_not_read_yet() {
    let model = load_model(
        r#"<mujoco>
          <worldbody>
            <body name="b" pos="0 0 1">
              <joint name="h" axis="0 1 0"/>
              <geom type="box" size="0.15 0.03 0.03" pos="0.15 0 0"/>
            </body>
          </worldbody>
          <actuator>
            <muscle joint="h" force="20" lengthrange="-1.5 1.5"/>
          </actuator>
        </mujoco>"#,
    )
    .expect("load");
    assert_eq!(model.actuator_lengthrange[0], (0.0, 0.0));
}
