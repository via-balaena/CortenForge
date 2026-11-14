//! Runs against MuJoCo 3.5.0's, on models sim-mjcf compiles as MuJoCo does.
//!
//! The golden is `assets/golden/trajectory/trajectory.json`, from
//! `scripts/gen_trajectory_reference.py` on the unfused oracle: a hinge pair
//! past its limit, a box resting on a plane, a cylinder dropped onto a box
//! slab and onto a convex mesh slab, and five bodies dropped at two spots of
//! a bumpy height field. Each body's inertial is written out as MuJoCo
//! compiles it (sim-mjcf compiles each the same without it:
//! `each_written_inertial_is_the_loaders_own`) and the mesh in the frame
//! MuJoCo moves it to. These runs close the forward side of ledgers L154,
//! L206 and L214.

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
/// `1e-12 · max(1, |MuJoCo's|)`, a NaN failing, and every one is finite. On
/// every platform no step may find a bad position, velocity or acceleration
/// (`Data::divergence_detected`): sim-core resets such a state, which would
/// leave the next checkpoint finite.
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
                if data.divergence_detected() {
                    failures.push(format!("{name} step {steps}: diverged"));
                }
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

/// `xml` with its `<inertial …/>` elements cut out.
fn without_inertials(xml: &str) -> String {
    let mut out = String::new();
    let mut rest = xml;
    while let Some(start) = rest.find("<inertial") {
        out.push_str(&rest[..start]);
        let end = start + rest[start..].find("/>").expect("a self-closing <inertial>") + 2;
        rest = &rest[end..];
    }
    out.push_str(rest);
    out
}

/// Each scene's `<inertial>`s, written out as MuJoCo 3.5.0 compiles them,
/// are what sim-mjcf compiles from the bodies' geoms without them, bit for
/// bit: each body's mass, principal moments, centre of mass and frame (book
/// Rigid-loading L33).
#[test]
fn each_written_inertial_is_the_loaders_own() {
    let golden = golden();
    for (name, case) in golden["cases"].as_object().expect("cases") {
        let xml = case["xml"].as_str().expect("xml");
        assert!(xml.contains("<inertial"), "{name}: no inertial written");
        let written = sim_mjcf::load_model(xml).expect("load");
        let derived = sim_mjcf::load_model(&without_inertials(xml)).expect("load");
        let bits = |m: &sim_core::Model| -> Vec<u64> {
            (0..m.nbody)
                .flat_map(|b| {
                    let q = m.body_iquat[b];
                    std::iter::once(m.body_mass[b])
                        .chain(m.body_inertia[b].iter().copied())
                        .chain(m.body_ipos[b].iter().copied())
                        .chain([q.w, q.i, q.j, q.k])
                })
                .map(f64::to_bits)
                .collect()
        };
        assert_eq!(bits(&derived), bits(&written), "{name}");
    }
}
