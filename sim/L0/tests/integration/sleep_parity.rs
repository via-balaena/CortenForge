//! Sleep against MuJoCo 3.5.0: the kinematic trees, the automatic sleep
//! policies, the bodies' sleep states, `dof_length` and the test a tree must
//! pass to sleep.
//!
//! The golden is `assets/golden/sleep/sleep.json`, from the unfused MuJoCo
//! 3.5.0 oracle (`scripts/gen_sleep_reference.py`, which describes its
//! models).

use serde_json::Value;
use sim_core::{BodyWrench, ENABLE_SLEEP, Model, SleepPolicy, SleepState};
use sim_mjcf::load_model;

fn golden() -> Value {
    serde_json::from_str(include_str!("../assets/golden/sleep/sleep.json"))
        .expect("sleep golden parses")
}

/// The case's model: its `ours_xml` where sim-mjcf reads the model in
/// another form than MuJoCo (the generator says which and why), else its
/// `xml`.
fn model_of(case: &Value) -> Model {
    let xml = case["ours_xml"].as_str().or_else(|| case["xml"].as_str());
    load_model(xml.expect("xml")).expect("load")
}

/// MuJoCo's ids, its -1 as `usize::MAX`.
fn ids(v: &Value) -> Vec<usize> {
    v.as_array()
        .expect("array")
        .iter()
        .map(|x| usize::try_from(x.as_i64().expect("int")).unwrap_or(usize::MAX))
        .collect()
}

fn policies(v: &Value) -> Vec<SleepPolicy> {
    v.as_array()
        .expect("array")
        .iter()
        .map(|p| match p.as_str().expect("name") {
            "AutoNever" => SleepPolicy::AutoNever,
            "AutoAllowed" => SleepPolicy::AutoAllowed,
            "Never" => SleepPolicy::Never,
            "Allowed" => SleepPolicy::Allowed,
            "Init" => SleepPolicy::Init,
            other => panic!("unresolved policy {other}"),
        })
        .collect()
}

/// MuJoCo's `body_awake` codes (`mjtSleepState`).
fn states(v: &Value) -> Vec<SleepState> {
    v.as_array()
        .expect("array")
        .iter()
        .map(|s| match s.as_i64().expect("code") {
            -1 => SleepState::Static,
            0 => SleepState::Asleep,
            1 => SleepState::Awake,
            other => panic!("unknown sleep state {other}"),
        })
        .collect()
}

/// The tree tables and the resolved automatic policies are MuJoCo's: a tree
/// starts at a moving body with static ancestors, a static body is in no
/// tree, a tendon keeps its trees in wrap order, and the automatic policy
/// follows each transmission, tendon and flex rule.
#[test]
fn kinematic_trees_match_mujoco() {
    for case in golden()["trees"].as_array().expect("trees") {
        let name = case["name"].as_str().expect("name");
        let model = model_of(case);
        let ntree = case["ntree"].as_u64().and_then(|n| usize::try_from(n).ok());
        assert_eq!(Some(model.ntree), ntree, "{name}: ntree");
        // (ours, MuJoCo's name)
        let tables: [(&[usize], &str); 8] = [
            (&model.body_treeid, "body_treeid"),
            (&model.tree_body_adr, "tree_bodyadr"),
            (&model.tree_body_num, "tree_bodynum"),
            (&model.tree_dof_adr, "tree_dofadr"),
            (&model.tree_dof_num, "tree_dofnum"),
            (&model.dof_treeid, "dof_treeid"),
            (&model.tendon_treenum, "tendon_treenum"),
            (&model.tendon_tree, "tendon_treeid"),
        ];
        for (ours, theirs) in tables {
            assert_eq!(ours, ids(&case[theirs]), "{name}: {theirs}");
        }
        assert_eq!(
            model.tree_sleep_policy,
            policies(&case["tree_sleep_policy"]),
            "{name}: tree_sleep_policy"
        );
    }
}

/// A body in no tree is `Static` and a body under a mocap body `Awake`, after
/// `make_data` and after a step, with sleep enabled and disabled, as MuJoCo's
/// `body_awake`.
#[test]
fn static_body_is_not_a_tree() {
    for case in golden()["trees"].as_array().expect("trees") {
        let name = case["name"].as_str().expect("name");
        let mut model = model_of(case);
        for (suffix, sleep) in [("", true), ("_nosleep", false)] {
            if !sleep {
                model.enableflags &= !ENABLE_SLEEP;
            }
            let mut data = model.make_data();
            assert_eq!(
                data.body_sleep_state,
                states(&case[format!("body_awake_reset{suffix}")]),
                "{name}{suffix}: after make_data"
            );
            data.step(&model).expect("step");
            assert_eq!(
                data.body_sleep_state,
                states(&case[format!("body_awake_step{suffix}")]),
                "{name}{suffix}: after a step"
            );
        }
    }
}

fn case<'a>(golden: &'a Value, name: &str) -> &'a Value {
    golden["trees"]
        .as_array()
        .expect("trees")
        .iter()
        .find(|c| c["name"] == name)
        .unwrap_or_else(|| panic!("no case {name}"))
}

/// A static body with gravity compensation, sleep enabled: the pass skips
/// only sleeping trees, so it runs, and `qfrc_gravcomp` is MuJoCo's.
#[test]
fn gravcomp_on_a_static_body_with_sleep() {
    let golden = golden();
    let case = case(&golden, "gravcomp");
    let model = model_of(case);
    let mut data = model.make_data();
    data.forward(&model).expect("forward");
    let theirs: Vec<f64> = case["qfrc_gravcomp"]
        .as_array()
        .expect("qfrc_gravcomp")
        .iter()
        .map(|x| x.as_f64().expect("number"))
        .collect();
    for (i, (a, b)) in data.qfrc_gravcomp.iter().zip(&theirs).enumerate() {
        assert!(
            (a - b).abs() <= 1e-12,
            "qfrc_gravcomp[{i}]: ours {a}, MuJoCo {b}"
        );
    }
    data.step(&model).expect("step");
}

/// A contact between a static body and a moving one is in the moving body's
/// island, whichever geom comes first.
#[test]
fn contact_with_a_static_body_has_an_island() {
    let golden = golden();
    let model = model_of(case(&golden, "table"));
    let mut data = model.make_data();
    data.forward(&model).expect("forward");
    assert!(!data.contacts.is_empty(), "the box touches the table");
    let island = data.tree_island[model.body_treeid[2]];
    assert!(island >= 0, "the box's tree is in an island");
    for (i, c) in data.contacts.iter().enumerate() {
        assert_eq!(
            data.contact_island[i], island,
            "contact {i} ({} – {})",
            c.geom1, c.geom2
        );
    }
}

fn floats(v: &Value) -> Vec<f64> {
    v.as_array()
        .expect("array")
        .iter()
        .map(|x| x.as_f64().expect("number"))
        .collect()
}

/// `dof_length` is MuJoCo's: a rotational dof takes its body's size, the
/// largest distance from its centre of mass to a joint anchor of its own or
/// of a child's, or to the far side of one of its geoms, at least 1e-5.
#[test]
fn dof_length_matches_mujoco() {
    for case in golden()["lengths"].as_array().expect("lengths") {
        let name = case["name"].as_str().expect("name");
        let model = model_of(case);
        let theirs = floats(&case["dof_length"]);
        assert_eq!(model.dof_length.len(), theirs.len(), "{name}: nv");
        for (i, (a, b)) in model.dof_length.iter().zip(&theirs).enumerate() {
            assert!(
                (a - b).abs() <= 1e-12,
                "{name}: dof_length[{i}] ours {a}, MuJoCo {b}"
            );
        }
    }
}

/// Run the golden's run `name` for its steps, applying its sets (MuJoCo's
/// flat index; `xfrc_applied` force first), and return whether tree 0 is
/// asleep after each step, with MuJoCo's flags.
fn run(name: &str) -> (Vec<bool>, Vec<bool>) {
    let golden = golden();
    let case = golden["runs"]
        .as_array()
        .expect("runs")
        .iter()
        .find(|c| c["name"] == name)
        .unwrap_or_else(|| panic!("no run {name}"));
    let model = model_of(case);
    let mut data = model.make_data();
    let theirs: Vec<bool> = case["asleep"]
        .as_array()
        .expect("asleep")
        .iter()
        .map(|a| a.as_bool().expect("bool"))
        .collect();
    let sets = case["sets"].as_array().expect("sets");
    let mut ours = Vec::with_capacity(theirs.len());
    for k in 0..theirs.len() {
        for set in sets {
            if set[0].as_u64() != u64::try_from(k).ok() {
                continue;
            }
            let idx = usize::try_from(set[2].as_u64().expect("index")).expect("index");
            let value = set[3].as_f64().expect("value");
            match set[1].as_str().expect("field") {
                "qvel" => data.qvel[idx] = value,
                "qfrc_applied" => data.qfrc_applied[idx] = value,
                "xfrc_applied" => {
                    let mut row = data.xfrc_applied[idx / 6].to_mujoco_row();
                    row[idx % 6] = value;
                    data.xfrc_applied[idx / 6] = BodyWrench::from_mujoco_row(row);
                }
                other => panic!("unknown field {other}"),
            }
        }
        data.step(&model).expect("step");
        ours.push(data.tree_asleep[0] >= 0);
    }
    (ours, theirs)
}

/// A free box spun in zero gravity at 3e-4 rad/s sleeps when MuJoCo's does:
/// its rotational `dof_length` is 0.173, so `dof_length · |ω|` is under the
/// tolerance 1e-4. Its velocity is constant, so the step on which sleep is
/// decided does not matter.
#[test]
fn rotational_sleep_matches_mujoco() {
    let (ours, theirs) = run("spin");
    assert_eq!(ours, theirs);
}

/// A force of -0.0 applied to a sleeping box wakes it and keeps it awake, as
/// MuJoCo compares the applied forces bit for bit: `qfrc_applied`, and the
/// force of `xfrc_applied`. The step the box first sleeps on is not compared.
#[test]
fn negative_zero_force_blocks_sleep() {
    for name in ["negzero_qfrc", "negzero_xfrc"] {
        let (ours, theirs) = run(name);
        assert!(ours[199] && theirs[199], "{name}: asleep before the force");
        assert_eq!(ours[200..], theirs[200..], "{name}: from the force on");
        assert!(
            theirs[200..].iter().all(|a| !a),
            "{name}: MuJoCo stays awake"
        );
    }
}

/// A velocity exactly at the tolerance blocks sleep: MuJoCo refuses at
/// `dof_length · |v| >= tol`.
#[test]
fn velocity_at_the_tolerance_blocks_sleep() {
    let (ours, theirs) = run("at_tolerance");
    assert_eq!(ours, theirs);
    assert!(theirs.iter().all(|a| !a), "MuJoCo never sleeps");
}

/// A tree an actuator acts on never sleeps (its automatic policy is never),
/// though its hinge comes to rest: MuJoCo lets the same model sleep at step
/// 636 with `sleep="allowed"` (A8).
#[test]
fn an_actuated_tree_never_sleeps() {
    let (ours, theirs) = run("actuated");
    assert_eq!(ours, theirs);
    assert!(theirs.iter().all(|a| !a), "MuJoCo never sleeps");
}

/// With a tolerance of 0 a tree sleeps only at a velocity of exactly +0.
#[test]
fn zero_tolerance_sleeps_at_rest() {
    let (ours, theirs) = run("tol0_zero");
    assert_eq!(ours, theirs);
}
