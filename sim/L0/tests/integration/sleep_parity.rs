//! Sleep against MuJoCo 3.5.0: the kinematic trees, the automatic sleep
//! policies and the bodies' sleep states.
//!
//! The golden is `assets/golden/sleep/sleep.json`, from the unfused MuJoCo
//! 3.5.0 oracle (`scripts/gen_sleep_reference.py`, which describes its
//! models).

use serde_json::Value;
use sim_core::{ENABLE_SLEEP, Model, SleepPolicy, SleepState};
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
