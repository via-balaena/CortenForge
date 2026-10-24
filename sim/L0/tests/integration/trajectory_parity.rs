//! Runs against MuJoCo 3.5.0's, on models sim-mjcf compiles as MuJoCo does.
//!
//! The golden is `assets/golden/trajectory/trajectory.json`, from
//! `scripts/gen_trajectory_reference.py` on the unfused oracle: a hinge pair
//! past its limit, a box resting on a plane, a cylinder dropped onto a box
//! slab and onto a convex mesh slab, and five bodies dropped at two spots of
//! a bumpy height field. Each body's inertial is written out as MuJoCo
//! compiles it and the mesh in the frame MuJoCo moves it to, so a difference
//! here is the forward pass's or the step's, not the loader's (ledger L198
//! and Rigid-loading's L35 hold where the loader's differs). These runs
//! close the forward side of ledgers L154, L206 and L214.

use serde_json::Value;

fn golden() -> Value {
    serde_json::from_str(include_str!("../assets/golden/trajectory/trajectory.json"))
        .expect("trajectory golden parses")
}

fn floats(v: &Value) -> Vec<f64> {
    v.as_array()
        .expect("array")
        .iter()
        .map(|x| x.as_f64().expect("number"))
        .collect()
}

/// Every checkpoint's qpos and qvel bit for bit on macOS arm64, where the
/// golden was made. A run turns through `sin` and `cos`, which another
/// platform's libm may round otherwise by an ulp, and a contact can make an
/// ulp large (ledger L231); there the first checkpoint is compared, within
/// `1e-12 · max(1, |MuJoCo's|)`, a NaN failing, and every one is finite.
#[test]
fn runs_as_mujoco_with_the_loaders_inputs_as_mujocos() {
    let exact = cfg!(all(target_os = "macos", target_arch = "aarch64"));
    let golden = golden();
    let mut failures = Vec::new();
    let mut checkpoints = 0;
    for (name, case) in golden["cases"].as_object().expect("cases") {
        let model = sim_mjcf::load_model(case["xml"].as_str().expect("xml")).expect("load");
        let mut data = model.make_data();
        data.qpos
            .as_mut_slice()
            .copy_from_slice(&floats(&case["start"]["qpos"]));
        data.qvel
            .as_mut_slice()
            .copy_from_slice(&floats(&case["start"]["qvel"]));
        let every = case["every"].as_u64().expect("every");
        let mut steps = 0;
        for (k, checkpoint) in case["checkpoints"]
            .as_array()
            .expect("checkpoints")
            .iter()
            .enumerate()
        {
            let at = checkpoint["step"].as_u64().expect("step");
            while steps < at {
                data.step(&model).expect("step");
                steps += 1;
            }
            assert_eq!(at, every * (k as u64 + 1), "{name}: checkpoint {k}");
            for (what, ours, mj) in [
                ("qpos", data.qpos.as_slice(), floats(&checkpoint["qpos"])),
                ("qvel", data.qvel.as_slice(), floats(&checkpoint["qvel"])),
            ] {
                let matches = ours.len() == mj.len()
                    && ours.iter().zip(&mj).all(|(a, b)| {
                        if exact {
                            a.to_bits() == b.to_bits()
                        } else if k == 0 {
                            (a - b).abs() <= 1e-12 * b.abs().max(1.0)
                        } else {
                            a.is_finite()
                        }
                    });
                if !matches {
                    failures.push(format!("{name} step {at}: {what} {ours:?}, MuJoCo {mj:?}"));
                }
            }
            checkpoints += 1;
        }
    }
    assert_eq!(
        checkpoints,
        40 + 15 + 12 + 12 + 10 * 20,
        "every checkpoint compared"
    );
    assert!(
        failures.is_empty(),
        "{} differ:\n{}",
        failures.len(),
        failures.join("\n")
    );
}
