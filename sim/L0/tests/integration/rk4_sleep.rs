//! Sleep under RK4 (registry `D-RK4-SLEEP`): a tree asleep at a step's start
//! is held there through the stages and the advance until a stage's forward
//! pass wakes it, and the sleep step
//! evaluates its forward pass again from the position stage. MuJoCo 3.5.0
//! wakes such a tree the next step: its sleep step keeps the last stage's
//! frames, which the next kinematics find changed, and its stages move a
//! sleeping dof with the stale acceleration it keeps.
//!
//! The golden is `assets/golden/sleep/rk4_sleep.json`, from the unfused
//! MuJoCo 3.5.0 oracle (`scripts/gen_rk4_sleep_reference.py`, which describes
//! its fixtures and the two interventions that take those wakes out of
//! MuJoCo).

use serde_json::Value;
use sim_core::{Data, ENABLE_SLEEP, Integrator, Model, SleepState};
use sim_mjcf::load_model;

fn golden() -> Value {
    serde_json::from_str(include_str!("../assets/golden/sleep/rk4_sleep.json"))
        .expect("rk4 sleep golden parses")
}

fn fixtures() -> Vec<Value> {
    golden()["fixtures"].as_array().expect("fixtures").clone()
}

fn floats(v: &Value) -> Vec<f64> {
    v.as_array()
        .expect("array")
        .iter()
        .map(|x| x.as_f64().expect("number"))
        .collect()
}

fn count(v: &Value) -> usize {
    usize::try_from(v.as_u64().expect("count")).expect("count")
}

/// Step `model` `nstep` times and return `tree_asleep` after every step and
/// the data after each step `keep` accepts.
fn run(model: &Model, nstep: usize, keep: impl Fn(usize) -> bool) -> (Vec<Vec<i32>>, Vec<Data>) {
    let mut data = model.make_data();
    let (mut asleep, mut kept) = (Vec::with_capacity(nstep), Vec::new());
    for k in 0..nstep {
        data.step(model).expect("step");
        asleep.push(data.tree_asleep.clone());
        if keep(k) {
            kept.push(data.clone());
        }
    }
    (asleep, kept)
}

/// The steps after which any tree's sleep changes, from awake to asleep or
/// back.
fn transitions(asleep: &[Vec<i32>]) -> Vec<usize> {
    let any = |t: &[i32]| t.iter().any(|&a| a >= 0);
    (0..asleep.len())
        .filter(|&k| any(&asleep[k]) != (k > 0 && any(&asleep[k - 1])))
        .collect()
}

/// A box resting on the plane, an actuated hinge and a swinging chain each
/// fall asleep once, at MuJoCo's first sleep step, and stay asleep to the end
/// (3,000, 1,500 and 4,500 steps), where MuJoCo's wake the next step and
/// sleep and wake again and again: 584, 174 and 170 changes.
#[test]
fn rk4_sleeping_tree_stays_asleep() {
    for fixture in fixtures() {
        let name = fixture["name"].as_str().expect("name");
        let model = load_model(fixture["xml"].as_str().expect("xml")).expect("load");
        let (asleep, _) = run(&model, count(&fixture["nstep"]), |_| false);
        let first = count(&fixture["mujoco"]["first"][0]);
        assert_eq!(transitions(&asleep), [first], "{name}: ours");
        assert!(
            count(&fixture["mujoco"]["transitions"]) > 100,
            "{name}: MuJoCo"
        );
    }
}

/// After every step, every body of a sleeping tree has the frame the
/// kinematics give its `qpos`, bit for bit, under every integrator: the
/// test MuJoCo's kinematics make before they wake a tree
/// (`engine_core_smooth.c:163-179`), against the kinematics of a model with
/// sleep disabled. MuJoCo's RK4 fails it after 292, 87 and 85 steps of the
/// three fixtures.
#[test]
fn sleeping_pose_equals_fk_of_qpos() {
    let mut cases: Vec<(String, Model, usize)> = Vec::new();
    for fixture in fixtures() {
        let name = fixture["name"].as_str().expect("name");
        let xml = fixture["xml"].as_str().expect("xml");
        let nstep = count(&fixture["nstep"]);
        cases.push((format!("{name} RK4"), load_model(xml).expect("load"), nstep));
        for (integrator, label) in [
            (Integrator::Euler, "Euler"),
            (Integrator::Implicit, "implicit"),
            (Integrator::ImplicitFast, "implicitfast"),
        ] {
            let mut model = load_model(xml).expect("load");
            model.integrator = integrator;
            cases.push((format!("{name} {label}"), model, nstep));
        }
    }
    for (name, model, nstep) in cases {
        let mut kinematics = model.clone();
        kinematics.enableflags &= !ENABLE_SLEEP;
        let mut fk = kinematics.make_data();
        let mut data = model.make_data();
        let mut checked = 0;
        for k in 0..nstep {
            data.step(&model).expect("step");
            fk.qpos.copy_from(&data.qpos);
            fk.forward(&kinematics).expect("forward");
            for b in 0..model.nbody {
                if data.body_sleep_state[b] != SleepState::Asleep {
                    continue;
                }
                checked += 1;
                assert_eq!(
                    data.xpos[b], fk.xpos[b],
                    "{name}: body {b} xpos after step {k}"
                );
                assert_eq!(
                    data.xquat[b].coords, fk.xquat[b].coords,
                    "{name}: body {b} xquat after step {k}"
                );
            }
        }
        assert!(checked > 0, "{name}: no body slept");
    }
}

/// A box falls asleep beside a pendulum that never sleeps: on that step the
/// pendulum's velocimeter, framelinvel and accelerometer read the forward
/// pass of the step's start state with the box's velocity zeroed, as the
/// sleep step zeroes it, bit for bit, and within 1e-14 of what they read with
/// the box never sleeping (measured 1.8e-15, the accelerometer; from the
/// zeroed velocity, by the first comparison; by what path it reaches the
/// pendulum is not isolated). In MuJoCo they read the last RK4 stage's
/// positions with the step's start velocities, 0.116 apart.
#[test]
fn rk4_sleep_step_does_not_disturb_awake_sensors() {
    let two = &golden()["two_trees"];
    let step = count(&two["step"]);
    let sleeping = load_model(two["xml_sleeps"].as_str().expect("xml")).expect("load");
    let (asleep, kept) = run(&sleeping, step + 1, |k| k == step);
    assert_eq!(transitions(&asleep), [step]);
    let sleeps = kept[0].sensordata.as_slice();
    let never = load_model(two["xml_never"].as_str().expect("xml")).expect("load");
    let (_, kept) = run(&never, step + 1, |k| k + 1 >= step);
    let mut start = kept[0].clone();
    start.qvel.rows_mut(0, 6).fill(0.0);
    start.forward(&never).expect("forward");
    assert_eq!(sleeps, start.sensordata.as_slice());
    for (a, b) in sleeps.iter().zip(kept[1].sensordata.iter()) {
        assert!((a - b).abs() <= 1e-14, "box asleep {a}, never asleep {b}");
    }
    let theirs = floats(&two["sleeps"])
        .iter()
        .zip(floats(&two["never"]))
        .map(|(a, b)| (a - b).abs())
        .fold(0.0_f64, f64::max);
    assert!(theirs > 0.1, "MuJoCo's: {theirs}");
}

/// While the box sleeps, the pendulum beside it, awake and accelerating,
/// steps as it does with the box never sleeping: its `qpos` and `qvel` bit
/// for bit, from the box's sleep step to 30 steps after it. (The held dofs
/// are the box's alone.)
#[test]
fn rk4_awake_tree_steps_as_if_nothing_slept() {
    let two = &golden()["two_trees"];
    let step = count(&two["step"]);
    let sleeping = load_model(two["xml_sleeps"].as_str().expect("xml")).expect("load");
    let never = load_model(two["xml_never"].as_str().expect("xml")).expect("load");
    let keep = |k: usize| k >= step;
    let (asleep, slept) = run(&sleeping, step + 31, keep);
    let (_, awake) = run(&never, step + 31, keep);
    assert_eq!(transitions(&asleep), [step]);
    assert_eq!(slept.len(), 31);
    let pendulum = sleeping.jnt_qpos_adr[1];
    let dof = sleeping.jnt_dof_adr[1];
    for (k, (a, b)) in slept.iter().zip(&awake).enumerate() {
        assert_eq!(
            a.qpos[pendulum], b.qpos[pendulum],
            "qpos {k} steps after the sleep step"
        );
        assert_eq!(
            a.qvel[dof], b.qvel[dof],
            "qvel {k} steps after the sleep step"
        );
    }
    assert!(
        awake.iter().any(|d| d.qvel[dof].abs() > 0.1),
        "the pendulum swings"
    );
}

/// The sleep step evaluates the forward pass again at the step's start
/// state, the slept tree's velocity zeroed and its activation advanced: after
/// the actuated hinge's sleep step its actuator force, accelerometer and
/// joint velocity are that evaluation's, bit for bit. MuJoCo's read the last
/// RK4 stage's positions, and keep them while the hinge sleeps: the force
/// 2.0e-5 off, the accelerometer 1.9e-4.
#[test]
fn rk4_sleep_step_evaluates_the_start_state() {
    let fixture = fixtures()
        .into_iter()
        .find(|f| f["name"] == "actuated_hinge")
        .expect("actuated_hinge");
    let model = load_model(fixture["xml"].as_str().expect("xml")).expect("load");
    let step = count(&fixture["mujoco"]["first"][0]);
    let mut data = model.make_data();
    for _ in 0..step {
        data.step(&model).expect("step");
    }
    let mut start = data.clone();
    data.step(&model).expect("step");
    assert!(
        data.tree_asleep[0] >= 0,
        "the hinge sleeps after step {step}"
    );
    start.qvel.fill(0.0);
    start.act.copy_from(&data.act);
    start.forward(&model).expect("forward");
    assert_eq!(data.sensordata, start.sensordata);
    let theirs = fixture["intervened"]["samples"]
        .as_array()
        .expect("samples")
        .iter()
        .find(|s| count(&s["step"]) == step)
        .map(|s| floats(&s["sensordata"]))
        .expect("sample at the sleep step");
    let apart = data
        .sensordata
        .iter()
        .zip(&theirs)
        .map(|(a, b)| (a - b).abs())
        .fold(0.0_f64, f64::max);
    assert!(apart > 1e-5, "MuJoCo's: {apart}");
}

/// A tree woken during an RK4 stage takes, from that stage on, each
/// stage's acceleration, as MuJoCo's, whose stages hold no tree (the stages
/// before count as 0, its kept acceleration here, asleep from the start): a box
/// asleep from the start with a sphere falling onto it, and a hinged capsule
/// asleep from the start, held by a spatial and a fixed tendon with springs,
/// with a sphere falling onto it. Each wakes at MuJoCo's step, where a
/// forward pass at the step's start state leaves it asleep (so a stage woke
/// it), and `qpos` and `qvel` are MuJoCo's within 1e-12 after every step to
/// 20 past it (measured: 2.2e-16 and 6.2e-15). Before (ledger L169), the
/// woken tree was held at the step's start until the step ended: the
/// capsule's hinge kept `qvel` 0 where MuJoCo's took 0.241.
#[test]
fn rk4_tree_woken_during_a_stage_steps_as_mujoco() {
    for name in ["stage_wake", "stage_wake_hinge"] {
        let case = &golden()[name];
        let model = load_model(case["xml"].as_str().expect("xml")).expect("load");
        let wake = count(&case["wake"]);
        let mut data = model.make_data();
        for (k, want) in case["traj"].as_array().expect("traj").iter().enumerate() {
            if k == wake {
                let mut probe = data.clone();
                probe.forward(&model).expect("forward");
                assert!(
                    probe.tree_asleep[0] >= 0,
                    "{name}: the start state wakes nothing"
                );
            }
            data.step(&model).expect("step");
            assert_eq!(
                data.tree_asleep[0] < 0,
                k >= wake,
                "{name}: tree 0 awake after step {k}"
            );
            for (field, ours) in [("qpos", &data.qpos), ("qvel", &data.qvel)] {
                for (i, (a, b)) in ours.iter().zip(floats(&want[field])).enumerate() {
                    assert!(
                        (a - b).abs() <= 1e-12,
                        "{name} step {k}: {field}[{i}] {a}, MuJoCo {b}"
                    );
                }
            }
        }
    }
}

/// A tree woken during an RK4 stage after an earlier step put it to sleep
/// takes no acceleration from the stages it was held through: a box resting
/// on a plane falls asleep (once in 600 steps, keeping the free-fall
/// acceleration −9.81 on its vertical dof), then a sphere launched onto it
/// wakes it in a stage (a forward pass at that step's start leaves it
/// asleep), from two heights: the contact first appears in the second
/// stage, and in the last. After the waking step the box's vertical
/// acceleration is 0 at each stage that held it (one, then three), the
/// acceleration those stages moved it with, and not 0 at the stage that
/// woke it; and its velocity is h·Σ Bⱼ·(stage j's acceleration) within
/// 1e-15·max(1, |it|). Before, the held stages counted with the kept
/// acceleration (−9.81; on the first launch the box's vertical velocity
/// after the step −0.010526, where with the held stage at 0 it is
/// −0.007256).
/// MuJoCo's stages move a sleeping tree with its kept acceleration (registry
/// `D-RK4-SLEEP`); on this scene it sleeps and wakes again and again, so no
/// MuJoCo trajectory is compared.
#[test]
fn rk4_tree_woken_during_a_stage_takes_no_kept_acceleration() {
    let model = load_model(
        r#"<mujoco model="rk4_kept">
  <option timestep="0.002" integrator="RK4"><flag sleep="enable"/></option>
  <worldbody>
    <geom type="plane" size="5 5 0.1"/>
    <body name="b" pos="0 0 0.0995"><freejoint/><geom type="box" size="0.1 0.1 0.1" mass="1"/></body>
    <body name="s" pos="0 0 0.5" gravcomp="1"><freejoint/><geom type="sphere" size="0.1" mass="1"/></body>
  </worldbody>
</mujoco>"#,
    )
    .expect("load");
    let box_tree = model.body_treeid[1];
    let mut settled = model.make_data();
    for _ in 0..600 {
        settled.step(&model).expect("step");
    }
    assert!(
        !settled.tree_awake[box_tree],
        "the box is awake after 600 steps"
    );
    let mut probe = settled.clone();
    probe.forward(&model).expect("forward");
    assert!(
        probe.qacc[2].abs() > 1.0,
        "the box's kept acceleration {}",
        probe.qacc[2]
    );
    let h = model.timestep;
    let weights = [1.0 / 6.0, 1.0 / 3.0, 1.0 / 3.0, 1.0 / 6.0];
    // The sphere launched from two heights: its contact with the box first
    // appears in the second stage, and in the last (the gap at the step's
    // start between h·2 m/s and its half), so one stage, then three, hold it.
    for (height, held) in [(0.5, 1), (0.5025, 3)] {
        let mut data = settled.clone();
        data.qpos[9] = height;
        data.qvel[8] = -2.0;
        let mut woke = false;
        for k in 0..200 {
            let was_asleep = !data.tree_awake[box_tree];
            let mut probe = data.clone();
            probe.forward(&model).expect("forward");
            data.step(&model).expect("step");
            if was_asleep && data.tree_awake[box_tree] {
                assert!(
                    !probe.tree_awake[box_tree],
                    "{height}, step {k}: the start state wakes the box"
                );
                let stages: Vec<f64> = (0..4).map(|j| data.rk4_qacc[j][2]).collect();
                assert!(
                    stages[..held].iter().all(|&a| a == 0.0) && stages[held] != 0.0,
                    "{height}, step {k}: the box's vertical acceleration at each stage {stages:?}, \
                     {held} held"
                );
                for dof in 0..6 {
                    let advance: f64 = (0..4)
                        .map(|j| weights[j] * data.rk4_qacc[j][dof])
                        .sum::<f64>()
                        * h;
                    assert!(
                        (data.qvel[dof] - advance).abs() <= 1e-15 * advance.abs().max(1.0),
                        "{height}, step {k}: the box's qvel[{dof}] {}, its stages' {advance}",
                        data.qvel[dof]
                    );
                }
                woke = true;
                break;
            }
        }
        assert!(woke, "{height}: the sphere never woke the box");
    }
}

/// MuJoCo with both wakes taken out of it (after every step, a sleeping
/// tree's frames set to its kinematics and its dofs' `qacc_smooth` to 0):
/// the same changes of `tree_asleep` at the same steps, and `qpos`, `qvel`,
/// `sensordata` and `act` within 1e-12 after every 50th step and the steps
/// around a change (measured: 1.3e-15, the chain's `qvel` while it swings),
/// but the actuated hinge's sensors from its sleep step on, which in MuJoCo
/// are the last stage's (`rk4_sleep_step_evaluates_the_start_state`): no
/// intervention after a step reaches the values the step computed.
#[test]
fn rk4_sleep_matches_mujoco_without_its_wake_defects() {
    for fixture in fixtures() {
        let name = fixture["name"].as_str().expect("name");
        let first = count(&fixture["mujoco"]["first"][0]);
        let model = load_model(fixture["xml"].as_str().expect("xml")).expect("load");
        let intervened = &fixture["intervened"];
        let samples = intervened["samples"].as_array().expect("samples");
        let steps: Vec<usize> = samples.iter().map(|s| count(&s["step"])).collect();
        let (asleep, kept) = run(&model, count(&fixture["nstep"]), |k| steps.contains(&k));
        let ours: Vec<(usize, Vec<i64>)> = (0..asleep.len())
            .filter(|&k| {
                asleep[k]
                    != if k > 0 {
                        asleep[k - 1].clone()
                    } else {
                        model.make_data().tree_asleep
                    }
            })
            .map(|k| (k, asleep[k].iter().map(|&a| i64::from(a)).collect()))
            .collect();
        let theirs: Vec<(usize, Vec<i64>)> = intervened["changes"]
            .as_array()
            .expect("changes")
            .iter()
            .map(|c| {
                let t = c[1].as_array().expect("tree_asleep");
                (
                    count(&c[0]),
                    t.iter().map(|a| a.as_i64().expect("int")).collect(),
                )
            })
            .collect();
        assert_eq!(ours, theirs, "{name}: changes of tree_asleep");
        for (sample, data) in samples.iter().zip(&kept) {
            let k = count(&sample["step"]);
            for (field, got) in [
                ("qpos", data.qpos.as_slice()),
                ("qvel", data.qvel.as_slice()),
                ("sensordata", data.sensordata.as_slice()),
                ("act", data.act.as_slice()),
            ] {
                if field == "sensordata" && name == "actuated_hinge" && k >= first {
                    continue;
                }
                let want = floats(&sample[field]);
                assert_eq!(got.len(), want.len(), "{name}: {field} length");
                for (i, (a, b)) in got.iter().zip(&want).enumerate() {
                    assert!(
                        (a - b).abs() <= 1e-12,
                        "{name}: {field}[{i}] after step {k}: ours {a}, MuJoCo {b}"
                    );
                }
            }
        }
    }
}
