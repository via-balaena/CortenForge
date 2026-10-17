//! Sleep against MuJoCo 3.5.0: the kinematic trees, the automatic sleep
//! policies, the bodies' sleep states, `dof_length`, the test a tree must
//! pass to sleep, and per-step traces of sleep, wake and the states around
//! them.
//!
//! The goldens are `assets/golden/sleep/sleep.json` and `sleep_traces.json`,
//! from the unfused MuJoCo 3.5.0 oracle (`scripts/gen_sleep_reference.py`,
//! which describes their models).

use std::sync::{Arc, Mutex};

use serde_json::Value;
use sim_core::{
    BodyWrench, DISABLE_DAMPER, DISABLE_SPRING, ENABLE_SLEEP, MakeDataError, MjStage, Model,
    ResetError, SleepPolicy, SleepState, StepError,
};
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

// ── Per-step traces (sleep_traces.json) ──────────────────────────────────

fn traces_golden() -> Value {
    serde_json::from_str(include_str!("../assets/golden/sleep/sleep_traces.json"))
        .expect("sleep traces golden parses")
}

/// What one step left, in this crate or in MuJoCo: the discrete fields at
/// every step, `qvel`, `qacc`, `qacc_warmstart`, `sensordata` and `act` where
/// the golden keeps them.
#[derive(Debug, PartialEq)]
struct Step {
    tree_asleep: Vec<i64>,
    ncon: usize,
    nefc: usize,
    nisland: usize,
    cb: String,
    floats: Option<[Vec<f64>; 5]>,
}

const FLOATS: [&str; 5] = ["qvel", "qacc", "qacc_warmstart", "sensordata", "act"];

fn count(v: &Value) -> usize {
    usize::try_from(v.as_u64().expect("count")).expect("count")
}

/// Run the golden's trace `name` on this crate, logging the callbacks as the
/// generator does (P passive, C control, F the contact filter where the trace
/// logs it), and return its steps with MuJoCo's.
fn trace(name: &str) -> (Vec<Step>, Vec<Step>) {
    let golden = traces_golden();
    let case = golden["traces"]
        .as_array()
        .expect("traces")
        .iter()
        .find(|c| c["name"] == name)
        .unwrap_or_else(|| panic!("no trace {name}"));
    let mut model = model_of(case);
    if case["sleep"] == false {
        model.enableflags &= !ENABLE_SLEEP;
    }
    let log = Arc::new(Mutex::new(String::new()));
    let (passive, control) = (Arc::clone(&log), Arc::clone(&log));
    // The passive counter: 0.001 times the callback's call count added to one
    // dof's passive force, as the generator's callback does.
    let counter = case["passive_force"].as_array().map(|pf| {
        let dof = usize::try_from(pf[0].as_u64().expect("dof")).expect("dof");
        (dof, pf[1].as_f64().expect("scale"))
    });
    let calls = Arc::new(Mutex::new(0.0_f64));
    model.set_passive_callback(move |_, data| {
        passive.lock().expect("log").push('P');
        if let Some((dof, scale)) = counter {
            let mut n = calls.lock().expect("calls");
            *n += 1.0;
            data.qfrc_passive[dof] += scale * *n;
        }
    });
    model.set_control_callback(move |_, _| control.lock().expect("log").push('C'));
    if case["log_filter"] == true {
        let filter = Arc::clone(&log);
        model.set_contactfilter_callback(move |_, _, _, _| {
            filter.lock().expect("log").push('F');
            true
        });
    }
    let mut data = model.make_data();
    let sets = case["sets"].as_array().expect("sets");
    let theirs: Vec<Step> = case["steps"]
        .as_array()
        .expect("steps")
        .iter()
        .map(|s| Step {
            tree_asleep: s["tree_asleep"]
                .as_array()
                .expect("tree_asleep")
                .iter()
                .map(|a| a.as_i64().expect("int"))
                .collect(),
            ncon: count(&s["ncon"]),
            nefc: count(&s["nefc"]),
            nisland: count(&s["nisland"]),
            cb: s["cb"].as_str().expect("cb").to_owned(),
            floats: s.get("qvel").map(|_| FLOATS.map(|f| floats(&s[f]))),
        })
        .collect();
    let mut ours = Vec::with_capacity(theirs.len());
    for (k, mine) in theirs.iter().enumerate() {
        for set in sets {
            if set[0].as_u64() == u64::try_from(k).ok() {
                let idx = usize::try_from(set[2].as_u64().expect("index")).expect("index");
                let value = set[3].as_f64().expect("value");
                match set[1].as_str().expect("field") {
                    "qvel" => data.qvel[idx] = value,
                    "qpos" => data.qpos[idx] = value,
                    "ctrl" => data.ctrl[idx] = value,
                    "xfrc_applied" => {
                        let mut row = data.xfrc_applied[idx / 6].to_mujoco_row();
                        row[idx % 6] = value;
                        data.xfrc_applied[idx / 6] = BodyWrench::from_mujoco_row(row);
                    }
                    "sleep_off" => model.enableflags &= !ENABLE_SLEEP,
                    "eq_active" => model.eq_active[idx] = value != 0.0,
                    other => panic!("{name}: unknown field {other}"),
                }
            }
        }
        log.lock().expect("log").clear();
        data.step(&model).expect("step");
        ours.push(Step {
            tree_asleep: data.tree_asleep.iter().map(|&a| i64::from(a)).collect(),
            ncon: data.ncon,
            nefc: data.efc_type.len(),
            nisland: data.nisland,
            cb: log.lock().expect("log").clone(),
            floats: mine.floats.as_ref().map(|_| {
                [
                    data.qvel.as_slice().to_vec(),
                    data.qacc.as_slice().to_vec(),
                    data.qacc_warmstart.as_slice().to_vec(),
                    data.sensordata.as_slice().to_vec(),
                    data.act.as_slice().to_vec(),
                ]
            }),
        });
    }
    (ours, theirs)
}

/// The first step at which `field` of trace `name` differs from MuJoCo's:
/// `tree_asleep`, `ncon`, `nefc`, `nisland` or `cb`.
fn assert_trace(name: &str, fields: &[&str]) {
    let (ours, theirs) = trace(name);
    assert_steps(name, &ours, &theirs, fields);
}

/// [`assert_trace`] over the steps given.
fn assert_steps(name: &str, ours: &[Step], theirs: &[Step], fields: &[&str]) {
    for (k, (a, b)) in ours.iter().zip(theirs).enumerate() {
        for &field in fields {
            let (x, y) = match field {
                "tree_asleep" => (
                    format!("{:?}", a.tree_asleep),
                    format!("{:?}", b.tree_asleep),
                ),
                "ncon" => (a.ncon.to_string(), b.ncon.to_string()),
                "nefc" => (a.nefc.to_string(), b.nefc.to_string()),
                "nisland" => (a.nisland.to_string(), b.nisland.to_string()),
                "cb" => (a.cb.clone(), b.cb.clone()),
                other => panic!("unknown field {other}"),
            };
            assert_eq!(x, y, "{name}: {field} after step {k}: ours, MuJoCo");
        }
    }
}

/// The first step at which a tree is asleep in MuJoCo's trace `name`.
fn first_sleep(name: &str) -> Option<usize> {
    trace(name)
        .1
        .iter()
        .position(|s| s.tree_asleep.iter().any(|&a| a >= 0))
}

/// Sleep is decided in the advance, from the velocities the step started
/// with: a box resting on the plane falls asleep after step 86, as in
/// MuJoCo (85 when it was decided after the velocity update, A8 §0).
#[test]
fn sleep_step_matches_mujoco() {
    assert_eq!(first_sleep("box_rest"), Some(86));
    assert_trace("box_rest", &["tree_asleep"]);
}

/// On the step that puts a tree to sleep, the forward pass runs again from
/// the velocity stage: both callbacks fire twice.
#[test]
fn sleep_step_reforward_fires_both_callbacks() {
    assert_eq!(trace("box_rest").1[86].cb, "PCPC");
    assert_trace("box_rest", &["cb"]);
}

/// The re-forward computes the sensors at the zeroed velocity: the box's
/// `framelinvel` after its sleep step is 0, as in MuJoCo.
#[test]
fn sleep_step_sensors_see_zero_velocity() {
    let (ours, theirs) = trace("box_rest");
    let sensors = |s: &Step| s.floats.as_ref().expect("kept at a sleep step")[3][..3].to_vec();
    assert_eq!(sensors(&theirs[86]), vec![0.0; 3]);
    assert_eq!(sensors(&ours[86]), vec![0.0; 3]);
}

/// The implicit integrators take the same sleep step.
#[test]
fn sleep_step_matches_mujoco_implicit() {
    for name in ["box_rest_implicit", "box_rest_implicitfast"] {
        assert_eq!(first_sleep(name), Some(86), "{name}");
        assert_trace(name, &["tree_asleep", "cb"]);
    }
}

/// Two stacked spheres, one island, sleep together after step 72.
#[test]
fn sleep_wakes_and_sleeps_with_contact() {
    assert_trace("sstack", &["tree_asleep", "ncon", "nefc", "nisland"]);
}

/// A box asleep on the plane makes no contacts and no rows (MuJoCo skips a
/// sleeping body against a static one before the narrow phase).
#[test]
fn asleep_on_static_makes_no_contacts() {
    let (_, theirs) = trace("box_rest");
    assert!(theirs[87..].iter().all(|s| s.ncon == 0 && s.nefc == 0));
    assert_trace("box_rest", &["ncon", "nefc"]);
}

/// A box on a static body sleeps when MuJoCo's does and, asleep, makes no
/// contacts with it.
#[test]
fn box_on_static_body_sleeps_as_mujoco() {
    assert_eq!(first_sleep("table"), Some(86));
    assert_trace("table", &["tree_asleep", "ncon", "nefc"]);
}

/// The rows of a sleeping equality, joint limit and dof friction are not
/// made.
#[test]
fn sleeping_rows_dropped() {
    assert_trace("eqpair", &["tree_asleep", "nefc"]);
    assert_trace("limit", &["tree_asleep", "nefc"]);
}

/// With islands disabled, a tree with constraint rows cannot sleep, as in
/// MuJoCo.
#[test]
fn island_disabled_blocks_sleep_with_constraints() {
    assert_eq!(first_sleep("box_noisland"), None);
    assert_trace("box_noisland", &["tree_asleep", "nisland"]);
}

/// Friction-loss rows join their tree to an island: the hinge on its limit
/// with friction loss sleeps after step 219.
#[test]
fn friction_rows_make_islands() {
    assert_trace("limit", &["tree_asleep", "nisland"]);
}

/// Islands are made from the constraint rows with sleep disabled too.
#[test]
fn islands_exist_without_sleep() {
    assert!(trace("box_rest_nosleep").1.iter().all(|s| s.nisland == 1));
    assert_trace("box_rest_nosleep", &["nisland"]);
}

/// A8's damped chain and its filtered actuator with an allowed policy fall
/// asleep when MuJoCo's do (the activations advance before the sleep step).
#[test]
fn sleep_timing_matches_mujoco() {
    assert_eq!(first_sleep("chain2"), Some(2792));
    assert_eq!(first_sleep("act_sleep"), Some(636));
    for name in ["chain2", "act_sleep"] {
        assert_trace(name, &["tree_asleep", "cb"]);
    }
}

/// A sleeping body meets a static one in no narrow phase and no contact
/// filter call (MuJoCo filters the body pair first), and an explicit pair
/// with a sleeping body and a static one collides no more.
#[test]
fn asleep_on_static_skips_the_contact_filter_and_explicit_pairs() {
    assert_eq!(trace("box_rest_filter").1[87].cb, "PC");
    assert_trace("box_rest_filter", &["tree_asleep", "ncon", "cb"]);
    assert_trace("box_rest_pair", &["tree_asleep", "ncon", "nefc"]);
}

/// The awake trees advance with the acceleration computed before the sleep
/// step's second forward pass: a passive force that grows with each call
/// gives the falling sphere another acceleration in that pass, and its
/// velocity is MuJoCo's.
#[test]
fn awake_trees_advance_with_the_first_pass_acceleration() {
    assert_eq!(trace("box_rest_counter").1[86].cb, "PCPC");
    assert_trace("box_rest_counter", &["tree_asleep", "cb"]);
    let (ours, theirs) = trace("box_rest_counter");
    let vz = |s: &Step| s.floats.as_ref().expect("kept at a sleep step")[0][8];
    assert!((vz(&ours[86]) - vz(&theirs[86])).abs() <= 1e-12);
}

/// A sleeping actuated tree (policy allowed) keeps its last acceleration
/// when its ctrl changes, and stays asleep, as in MuJoCo: the qacc of its
/// sleep step (636) bit for bit at step 725, after the change at 700, and
/// MuJoCo's to the tolerance.
#[test]
fn ctrl_change_keeps_a_sleeping_tree_asleep() {
    assert_trace("act_sleep_ctrl", &["tree_asleep"]);
    let (ours, theirs) = trace("act_sleep_ctrl");
    let qacc = |s: &Step| s.floats.as_ref().expect("kept")[1].clone();
    assert_eq!(qacc(&ours[725]), qacc(&ours[636]));
    for (a, b) in qacc(&ours[725]).iter().zip(&qacc(&theirs[725])) {
        assert!((a - b).abs() <= 1e-12, "qacc at 725: ours {a}, MuJoCo {b}");
    }
}

fn asleep(step: &Step) -> bool {
    step.tree_asleep.iter().any(|&a| a >= 0)
}

/// Under RK4 a resting box falls asleep at MuJoCo's step, with MuJoCo's
/// callbacks through it, and stays asleep, where MuJoCo's wakes the next step
/// and sleeps and wakes every ten steps (registry `D-RK4-SLEEP`;
/// `rk4_sleep.rs`).
#[test]
fn rk4_sleeps_as_mujoco_and_stays_asleep() {
    let (ours, theirs) = trace("box_rest_RK4");
    let first = theirs.iter().position(asleep).expect("MuJoCo's box sleeps");
    assert_steps(
        "box_rest_RK4",
        &ours[..=first],
        &theirs[..=first],
        &["tree_asleep", "cb"],
    );
    assert!(ours[first..].iter().all(asleep));
    assert!(!asleep(&theirs[first + 1]));
}

/// With a tolerance of 0 and a velocity of -0.0, MuJoCo's first test sees
/// -0.0 and refuses; the step makes the velocity +0.0, so the box sleeps a
/// step later than at +0.0.
#[test]
fn zero_tolerance_negative_zero_velocity_sleeps_a_step_later() {
    let (ours, theirs) = run("tol0_negzero");
    assert_eq!(theirs.iter().position(|&a| a), Some(10));
    assert_eq!(ours, theirs);
}

/// Every trace, every step: the discrete fields equal MuJoCo's, and `qvel`,
/// `qacc`, `qacc_warmstart`, `sensordata` and `act` agree to 1e-12 where the
/// golden keeps them, the box under `island="disable"` and the two hinges
/// joined by a joint equality (`joint_eq_sleep`) included. Under RK4,
/// through MuJoCo's first sleep step, and its floats before it: from
/// there MuJoCo's re-forward reads the last stage's positions and wakes the
/// tree, where this crate's reads the step's start and keeps the tree asleep
/// (registry `D-RK4-SLEEP`; `rk4_sleep.rs`).
#[test]
fn traces_match_mujoco() {
    for case in traces_golden()["traces"].as_array().expect("traces") {
        let name = case["name"].as_str().expect("name");
        let (ours, theirs) = trace(name);
        let rk4 = case["xml"]
            .as_str()
            .expect("xml")
            .contains(r#"integrator="RK4""#);
        let (end, floats_end) = match theirs.iter().position(asleep) {
            Some(first) if rk4 => (first + 1, first),
            _ => (theirs.len(), theirs.len()),
        };
        assert_steps(
            name,
            &ours[..end],
            &theirs[..end],
            &["tree_asleep", "ncon", "nefc", "nisland", "cb"],
        );
        let tol = 1e-12;
        for (k, (a, b)) in ours[..floats_end].iter().zip(&theirs).enumerate() {
            let (Some(a), Some(b)) = (&a.floats, &b.floats) else {
                continue;
            };
            for (field, (x, y)) in FLOATS.iter().zip(a.iter().zip(b)) {
                assert_eq!(x.len(), y.len(), "{name}: {field} length after step {k}");
                for (i, (p, q)) in x.iter().zip(y).enumerate() {
                    assert!(
                        (p - q).abs() <= tol,
                        "{name}: {field}[{i}] after step {k}: ours {p}, MuJoCo {q}"
                    );
                }
            }
        }
    }
}

/// A sleeping tree woken by contact takes the countdown of the awake tree
/// that touched it: the resting sphere moved against the sleeping one at
/// step 5, its countdown at -5, leaves both at -4 after the step (fully
/// awake would be -10); the sphere landing on a sleeping one.
#[test]
fn wake_on_contact_inherits_countdown() {
    assert_eq!(trace("wake_countdown").1[5].tree_asleep, vec![-4, -4]);
    assert_trace("wake_countdown", &["tree_asleep", "ncon"]);
    assert_eq!(trace("swake").1[124].tree_asleep, vec![-10, -11]);
    assert_trace("swake", &["tree_asleep"]);
}

/// A velocity the user writes into a sleeping tree wakes it, as MuJoCo's
/// wake test reads every velocity bit, one below the sleep tolerance too.
#[test]
fn user_qvel_wakes_sleeping_tree() {
    assert_trace("uw_qvel", &["tree_asleep", "ncon", "nefc"]);
    assert_trace("uw_qvel_small", &["tree_asleep", "ncon", "nefc"]);
}

/// A connect made active between two trees asleep in two cycles wakes both.
#[test]
fn equality_joins_sleeping_cycles() {
    assert_eq!(trace("eq_activate").1[149].tree_asleep, vec![0, 1]);
    assert_trace("eq_activate", &["tree_asleep", "nefc"]);
}

/// After `make_data`, every trace's sleep state, contacts, rows and islands
/// are what MuJoCo's `mj_makeData` leaves: a reset that puts a tree to sleep
/// keeps its forward pass's contacts and clears its rows.
#[test]
fn reset_matches_mujoco() {
    for case in traces_golden()["traces"].as_array().expect("traces") {
        let name = case["name"].as_str().expect("name");
        let mut model = model_of(case);
        if case["sleep"] == false {
            model.enableflags &= !ENABLE_SLEEP;
        }
        let data = model.make_data();
        let reset = &case["reset"];
        let theirs: Vec<i64> = reset["tree_asleep"]
            .as_array()
            .expect("tree_asleep")
            .iter()
            .map(|a| a.as_i64().expect("int"))
            .collect();
        let ours: Vec<i64> = data.tree_asleep.iter().map(|&a| i64::from(a)).collect();
        assert_eq!(ours, theirs, "{name}: tree_asleep");
        assert_eq!(data.ncon, count(&reset["ncon"]), "{name}: ncon");
        assert_eq!(data.efc_type.len(), count(&reset["nefc"]), "{name}: nefc");
        assert_eq!(data.nisland, count(&reset["nisland"]), "{name}: nisland");
        let qacc = floats(&reset["qacc"]);
        assert_eq!(data.qacc.len(), qacc.len(), "{name}: qacc length");
        for (i, (a, b)) in data.qacc.iter().zip(&qacc).enumerate() {
            assert!(
                (a - b).abs() <= 1e-12,
                "{name}: qacc[{i}] after make_data: ours {a}, MuJoCo {b}"
            );
        }
    }
}

/// The island arrays after a forward pass: a resting box (16 contact rows)
/// beside a hinge set onto its limit (one row), two islands in MuJoCo's
/// order, the limit row's island first in the row order.
#[test]
fn islands_match_mujoco() {
    let golden = golden();
    let case = &golden["islands"];
    let model = model_of(case);
    let mut data = model.make_data();
    for set in case["qpos"].as_array().expect("qpos") {
        let i = usize::try_from(set[0].as_u64().expect("index")).expect("index");
        data.qpos[i] = set[1].as_f64().expect("value");
    }
    data.forward(&model).expect("forward");
    let ints = |v: &Value| -> Vec<i64> {
        v.as_array()
            .expect("ints")
            .iter()
            .map(|x| x.as_i64().expect("int"))
            .collect()
    };
    let nefc = data.efc_type.len();
    assert_eq!(nefc, count(&case["nefc"]));
    assert_eq!(data.nisland, count(&case["nisland"]));
    let wide = |v: &[i32]| v.iter().map(|&x| i64::from(x)).collect::<Vec<_>>();
    assert_eq!(wide(&data.efc_island[..nefc]), ints(&case["efc_island"]));
    assert_eq!(
        wide(&data.map_efc2iefc[..nefc]),
        ints(&case["map_efc2iefc"])
    );
    let wide = |v: &[usize]| {
        v.iter()
            .map(|&x| i64::try_from(x).expect("small"))
            .collect::<Vec<_>>()
    };
    assert_eq!(
        wide(&data.map_iefc2efc[..nefc]),
        ints(&case["map_iefc2efc"])
    );
    assert_eq!(
        wide(&data.island_nefc[..data.nisland]),
        ints(&case["island_nefc"])
    );
}

/// A position the user writes wakes the tree through the pose the
/// kinematics finds changed; a force on `xfrc_applied` wakes it through its
/// bytes.
#[test]
fn user_qpos_and_xfrc_wake() {
    for name in ["uw_qpos", "uw_xfrc", "rbox"] {
        assert_trace(name, &["tree_asleep", "ncon", "nefc"]);
    }
}

/// Switching sleep off wakes every sleeping tree at the next forward pass.
#[test]
fn disabling_sleep_wakes_all() {
    assert!(trace("sleep_off").1[150].tree_asleep.iter().all(|&a| a < 0));
    assert_trace("sleep_off", &["tree_asleep", "ncon", "nefc"]);
}

/// A tree that starts asleep has the values of a forward pass at the reset,
/// as MuJoCo's reset runs one: its box's position sensor reads 0.0995.
#[test]
fn init_tree_has_reset_values() {
    let (ours, theirs) = trace("box_init");
    let pos_z = |s: &Step| s.floats.as_ref().expect("kept at step 0")[3][5];
    assert_eq!(pos_z(&theirs[0]), 0.0995);
    assert!((pos_z(&ours[0]) - 0.0995).abs() <= 1e-12);
    assert_trace("box_init", &["tree_asleep", "ncon", "nefc", "cb"]);
}

fn refusal(name: &str) -> (Model, String) {
    let golden = golden();
    let case = golden["refusals"]
        .as_array()
        .expect("refusals")
        .iter()
        .find(|c| c["name"] == name)
        .unwrap_or_else(|| panic!("no refusal {name}"));
    let message = case["message"].as_str().expect("MuJoCo refuses").to_owned();
    (model_of(case), message)
}

/// A tree that starts asleep but cannot sleep, because a connect or a
/// contact joins it to an awake tree, is refused where MuJoCo refuses it: at
/// making a `Data` and at a reset, with MuJoCo's counts.
#[test]
fn init_mixed_island_refused() {
    for (name, tree, root_body) in [("initmix", 0, 1), ("initmix_contact", 1, 2)] {
        let (model, theirs) = refusal(name);
        assert!(theirs.contains("1 trees were marked as sleep='init' but only 0 could be slept"));
        assert!(theirs.contains(&format!("(id={root_body}) is the root of the first tree")));
        let refused = MakeDataError::InitSleep {
            marked: 1,
            slept: 0,
            tree,
            root_body,
        };
        assert_eq!(model.try_make_data().err(), Some(refused.clone()), "{name}");
        assert!(
            refused
                .to_string()
                .starts_with("1 trees were marked as sleep='init' but only 0 could be slept")
        );
        let mut awake = model.clone();
        awake.tree_sleep_policy.fill(SleepPolicy::AutoAllowed);
        let mut data = awake.make_data();
        data.step(&awake).expect("step");
        assert_eq!(
            data.try_reset(&model),
            Err(ResetError::InitSleep {
                marked: 1,
                slept: 0,
                tree,
                root_body,
            }),
            "{name}"
        );
        // A refused reset leaves the Data as it was
        assert_eq!(data.time, awake.timestep, "{name}");
    }
}

/// A tendon equality with sleep enabled is refused by the calls that run the
/// position stage, as MuJoCo's raise an error there, and by none of the
/// others; an inactive one, and one with sleep disabled, by none. Each call
/// on a fresh `Data`, against the MuJoCo call the golden names.
#[test]
fn tendon_equality_with_sleep_refused() {
    let (_, theirs) = refusal("tendon_equality");
    assert_eq!(
        theirs,
        "mj_wakeEquality: tendon equality does not yet support sleeping"
    );
    let golden = golden();
    let config = sim_core::DerivativeConfig::default();
    for key in ["active", "inactive"] {
        let case = &golden["tendon_equality_calls"][key];
        let model = model_of(case);
        let calls = case["calls"].as_object().expect("calls");
        assert_eq!(calls.len(), 10, "{key}");
        for (call, message) in calls {
            let mut data = model.make_data();
            let ours = match call.as_str() {
                "mj_forward" => data.forward(&model),
                "mj_step" => data.step(&model),
                "mj_step1" => data.step1(&model),
                "mj_step2" => data.step2(&model),
                "mj_forwardSkip(NONE)" => data.forward_skip(&model, MjStage::None, false),
                "mj_forwardSkip(POS)" => data.forward_skip(&model, MjStage::Pos, false),
                "mj_forwardSkip(VEL)" => data.forward_skip(&model, MjStage::Vel, false),
                "mj_Euler" => data.integrate(&model),
                "mjd_transitionFD" => sim_core::mjd_transition_fd(&model, &data, &config).map(drop),
                "mjd_inverseFD" => sim_core::mjd_inverse_fd(&model, &data, &config).map(drop),
                other => panic!("no call {other}"),
            };
            let want = if message.is_null() {
                Ok(())
            } else {
                Err(StepError::TendonEqualityWithSleep { eq: 0 })
            };
            assert_eq!(ours, want, "{key} {call}");
        }
        let mut asleep_off = model.clone();
        asleep_off.enableflags &= !ENABLE_SLEEP;
        let mut data = asleep_off.make_data();
        assert_eq!(data.forward(&asleep_off), Ok(()), "{key}, sleep disabled");
    }
    // `step` refuses before any work: a NaN position stays, where the check
    // `step` makes first would reset it (MuJoCo's `mj_step` resets it, then
    // its forward pass raises).
    let model = model_of(&golden["tendon_equality_calls"]["active"]);
    let mut data = model.make_data();
    data.qpos[0] = f64::NAN;
    assert_eq!(
        data.step(&model),
        Err(StepError::TendonEqualityWithSleep { eq: 0 })
    );
    assert!(data.qpos[0].is_nan());
    // Each call makes its input checks before this one.
    let mut zero_step = model;
    zero_step.timestep = 0.0;
    let mut data = zero_step.make_data();
    assert_eq!(data.forward(&zero_step), Err(StepError::InvalidTimestep));
    assert_eq!(
        data.forward_skip(&zero_step, MjStage::None, false),
        Err(StepError::InvalidTimestep)
    );
}

/// A tree that starts asleep beside an active tendon equality: MuJoCo
/// refuses the model at compile, where its reset's `mj_forward` raises the
/// tendon-equality error; making its `Data` here refuses it with that error.
#[test]
fn init_sleep_with_a_tendon_equality_refused() {
    let (model, theirs) = refusal("init_tendon_equality");
    assert!(
        theirs.ends_with("mj_wakeEquality: tendon equality does not yet support sleeping"),
        "{theirs}"
    );
    assert_eq!(
        model.try_make_data().err(),
        Some(MakeDataError::InitForward(
            StepError::TendonEqualityWithSleep { eq: 0 }
        ))
    );
}

/// The trees that start asleep see each mocap body at its pose. MuJoCo's
/// reset runs the init-sleep `mj_forward` with every mocap body at the origin
/// and places them after it (`engine_io.c:1374-1375`, `:1463`, `:1507-1517`),
/// so its sphere under a mocap sphere at (0, 0, 5) touches it there and keeps
/// that touch reading while it sleeps; here the sphere touches nothing.
/// Registry row `D-INIT-SLEEP-MOCAP`.
#[test]
fn init_sleep_sees_mocap_bodies_where_they_are() {
    let golden = golden();
    let case = &golden["init_mocap"];
    assert_eq!(case["ncon"], 1);
    assert!((case["sensordata"][0].as_f64().expect("touch") - 625.0).abs() < 1e-9);
    assert!((case["after_20"]["sensordata"][0].as_f64().expect("touch") - 625.0).abs() < 1e-9);
    let model = model_of(case);
    let mut data = model.make_data();
    assert_eq!(data.tree_asleep, vec![0]);
    assert_eq!(data.ncon, 0);
    assert!((data.xpos[1].z - 5.0).abs() < 1e-12, "{}", data.xpos[1]);
    assert!(data.sensordata[0].abs() < 1e-12);
    for _ in 0..20 {
        data.step(&model).expect("step");
    }
    assert_eq!(data.tree_asleep, vec![0]);
    assert!(data.sensordata[0].abs() < 1e-12);
}

/// A model with a tree that starts asleep makes its `Data` at a timestep of
/// 0 or below, its tree asleep: MuJoCo's reset runs `mj_forward`, which
/// checks no timestep (`init_timestep` in the golden).
#[test]
fn init_sleep_takes_no_timestep_check() {
    let golden = golden();
    let xml = traces_golden()["traces"]
        .as_array()
        .expect("traces")
        .iter()
        .find(|c| c["name"] == "box_init")
        .expect("box_init")["xml"]
        .as_str()
        .expect("xml")
        .to_owned();
    for (dt, want) in [(0.0, "0.0"), (-0.001, "-0.001")] {
        let mut model = load_model(&xml).expect("load");
        model.timestep = dt;
        let data = model.try_make_data().expect("MuJoCo makes this Data");
        let theirs: Vec<i64> = golden["init_timestep"][want]
            .as_array()
            .expect("tree_asleep")
            .iter()
            .map(|x| x.as_i64().expect("int"))
            .collect();
        let ours: Vec<i64> = data.tree_asleep.iter().map(|&a| i64::from(a)).collect();
        assert_eq!(ours, theirs, "timestep {dt}");
    }
}

/// A sleeping actuator acts with no force and keeps its `act_dot`, so its
/// activation stays where it was when its ctrl changes, and the sensors of
/// a sleeping actuator and tendon keep their values, as MuJoCo skips them.
#[test]
fn sleeping_actuators_and_sensors_are_skipped() {
    for name in ["act_sleep_ctrl", "act_tendon_ctrl"] {
        assert_trace(name, &["tree_asleep"]);
        let (ours, theirs) = trace(name);
        for (field, (a, b)) in FLOATS.iter().zip(
            ours[725]
                .floats
                .as_ref()
                .expect("kept")
                .iter()
                .zip(theirs[725].floats.as_ref().expect("kept")),
        ) {
            assert_eq!(a.len(), b.len(), "{name}: {field} length");
            for (x, y) in a.iter().zip(b) {
                assert!(
                    (x - y).abs() <= 1e-12,
                    "{name}: {field} after step 725: ours {x}, MuJoCo {y}"
                );
            }
        }
    }
}

/// A slider-crank whose slider site is on the world body is awake while its
/// crank's tree is: MuJoCo's `mj_actuatorSleepState` takes either site's body
/// being awake, and a static body is not awake. Driven from step 0, it acts
/// (its force sensor reads 1 after that step), and its crank sleeps at step
/// 1075, as MuJoCo's does.
#[test]
fn slider_crank_on_a_static_site_follows_its_crank() {
    let (ours, theirs) = trace("slider_static");
    let force = |steps: &[Step]| steps[0].floats.as_ref().expect("kept")[3][0];
    assert!((force(&theirs) - 1.0).abs() <= 1e-12);
    assert!(
        (force(&ours) - force(&theirs)).abs() <= 1e-12,
        "actuatorfrc after step 0: ours {}, MuJoCo {}",
        force(&ours),
        force(&theirs)
    );
    assert_eq!(
        theirs.iter().position(|s| s.tree_asleep[0] >= 0),
        Some(1075)
    );
    assert_trace("slider_static", &["tree_asleep"]);
}

/// Under implicitfast a sleeping actuator adds no term to the velocity
/// derivative, as MuJoCo's `mjd_actuator_vel` skips it
/// (`engine_derivative.c:1087-1090`): its velocity feedback reaches the awake
/// sprung slide through the derivative alone once the slide is pushed at
/// step 50.
#[test]
fn sleeping_actuator_adds_no_implicit_derivative() {
    assert_trace("deriv_asleep", &["tree_asleep"]);
    let (ours, theirs) = trace("deriv_asleep");
    let qvel = |steps: &[Step]| steps[75].floats.as_ref().expect("kept")[0][1];
    assert!(
        (qvel(&ours) - qvel(&theirs)).abs() <= 1e-12,
        "qvel[1] after step 75: ours {}, MuJoCo {}",
        qvel(&ours),
        qvel(&theirs)
    );
}

/// A damped hinge chain at rest, which sleeps, carrying a fixed and a
/// spatial tendon, each with friction loss and a limit, beside a swinging
/// hinge and a tendon between two world sites (on no tree: static, not
/// asleep), dense and sparse: after each step until five past the first
/// with a tree asleep, the trees' states, each tendon's length and Jacobian
/// row bit for bit, and the row count are MuJoCo's. MuJoCo's `mj_tendon`
/// clears every tendon and skips a sleeping one
/// (`engine_core_smooth.c:928-947`), so the asleep tendons read 0 and make
/// no rows; a sleeping tendon's wrap path keeps its address and count, which
/// the next tendon's path then takes, and the wrap arrays are MuJoCo's too.
/// A motor on the chain's spatial tendon and one on its site: MuJoCo's
/// transmission skips a sleeping actuator, which keeps its length and has
/// no moment (`engine_core_smooth.c:1279-1290`); both lengths and the site
/// motor's moment row are MuJoCo's. Before, the tendons kept their length
/// and row and made their two friction-loss rows.
#[test]
fn sleeping_tendons_have_no_length_row_or_constraint() {
    let golden = golden();
    for name in ["sleeping_tendons", "sleeping_tendons_sparse"] {
        sleeping_tendons_as_mujoco(name, &golden[name]);
    }
}

fn sleeping_tendons_as_mujoco(name: &str, case: &Value) {
    let model = model_of(case);
    let mut data = model.make_data();
    data.qvel
        .as_mut_slice()
        .copy_from_slice(&floats(&case["qvel"]));
    let bits = |v: &[f64]| v.iter().map(|x| x.to_bits()).collect::<Vec<_>>();
    for (k, want) in case["steps"].as_array().expect("steps").iter().enumerate() {
        data.step(&model).expect("step");
        let step = k + 1;
        let awake: Vec<bool> = want["tree_awake"]
            .as_array()
            .expect("tree_awake")
            .iter()
            .map(|a| a.as_i64().expect("int") != 0)
            .collect();
        assert_eq!(data.tree_awake, awake, "{name} step {step}: tree_awake");
        assert_eq!(
            bits(&data.ten_length),
            bits(&floats(&want["ten_length"])),
            "{name} step {step}: ten_length {:?}",
            data.ten_length
        );
        let rows: Vec<f64> = data.ten_J.iter().flat_map(|r| r.iter().copied()).collect();
        assert_eq!(
            bits(&rows),
            bits(&floats(&want["ten_J"])),
            "{name} step {step}: ten_J {rows:?}"
        );
        assert_eq!(
            data.efc_type.len(),
            count(&want["nefc"]),
            "{name} step {step}: nefc"
        );
        assert_eq!(
            bits(&data.actuator_length),
            bits(&floats(&want["actuator_length"])),
            "{name} step {step}: actuator_length {:?}",
            data.actuator_length
        );
        // The site motor's row; a tendon transmission's `actuator_moment`
        // is zero here by design (its force reads `ten_J`), MuJoCo's the
        // gear-scaled row while awake.
        let nv = model.nv;
        assert_eq!(
            bits(data.actuator_moment[1].as_slice()),
            bits(&floats(&want["actuator_moment"])[nv..2 * nv]),
            "{name} step {step}: the site motor's actuator_moment"
        );
        for (field, ours) in [
            ("ten_wrapadr", &data.ten_wrapadr),
            ("ten_wrapnum", &data.ten_wrapnum),
        ] {
            let theirs: Vec<usize> = want[field]
                .as_array()
                .expect(field)
                .iter()
                .map(|x| usize::try_from(x.as_u64().expect("count")).expect("count"))
                .collect();
            assert_eq!(ours, &theirs, "{name} step {step}: {field}");
        }
    }
}

/// Trees asleep from the reset (sleep="init") carrying actuators, each
/// struck by a sphere given a velocity: a site motor and a position
/// actuator on a slide; a free-joint and a ball-joint motor; a
/// slider-crank. MuJoCo's `mj_transmission` runs after the position
/// stage's wakes (`engine_forward.c:194-209`), so on the step a contact
/// wakes a tree its actuators act. After each step until ten past the wake:
/// the trees' states, and `qfrc_actuator`, `actuator_length`, the moment rows
/// of the actuators that act through one, `qpos` and `qvel` within
/// TOLERANCE·max(1, |MuJoCo's|).
#[test]
fn actuators_on_a_tree_a_contact_wakes_act() {
    const TOLERANCE: f64 = 1e-12;
    let golden = golden();
    for name in ["woken_site", "woken_free", "woken_crank"] {
        let case = &golden[name];
        let model = model_of(case);
        let mut data = model.make_data();
        data.qvel
            .as_mut_slice()
            .copy_from_slice(&floats(&case["qvel"]));
        data.ctrl
            .as_mut_slice()
            .copy_from_slice(&floats(&case["ctrl"]));
        let through: Vec<usize> = case["moment_actuators"]
            .as_array()
            .expect("moment_actuators")
            .iter()
            .map(|x| usize::try_from(x.as_u64().expect("index")).expect("index"))
            .collect();
        let nv = model.nv;
        for (k, want) in case["steps"].as_array().expect("steps").iter().enumerate() {
            data.step(&model).expect("step");
            let step = k + 1;
            let awake: Vec<bool> = want["tree_awake"]
                .as_array()
                .expect("tree_awake")
                .iter()
                .map(|a| a.as_i64().expect("int") != 0)
                .collect();
            assert_eq!(data.tree_awake, awake, "{name} step {step}: tree_awake");
            let moments: Vec<f64> = floats(&want["actuator_moment"]);
            let mut fields: Vec<(String, Vec<f64>, Vec<f64>)> = vec![
                (
                    "qfrc_actuator".into(),
                    data.qfrc_actuator.as_slice().to_vec(),
                    floats(&want["qfrc_actuator"]),
                ),
                (
                    "actuator_length".into(),
                    data.actuator_length.as_slice().to_vec(),
                    floats(&want["actuator_length"]),
                ),
                (
                    "qpos".into(),
                    data.qpos.as_slice().to_vec(),
                    floats(&want["qpos"]),
                ),
                (
                    "qvel".into(),
                    data.qvel.as_slice().to_vec(),
                    floats(&want["qvel"]),
                ),
            ];
            for &i in &through {
                fields.push((
                    format!("actuator_moment[{i}]"),
                    data.actuator_moment[i].as_slice().to_vec(),
                    moments[i * nv..(i + 1) * nv].to_vec(),
                ));
            }
            for (field, ours, theirs) in fields {
                assert_eq!(
                    ours.len(),
                    theirs.len(),
                    "{name} step {step}: {field}'s length"
                );
                for (j, (a, b)) in ours.iter().zip(&theirs).enumerate() {
                    assert!(
                        (a - b).abs() <= TOLERANCE * b.abs().max(1.0),
                        "{name} step {step}: {field}[{j}] {a}, MuJoCo {b}"
                    );
                }
            }
        }
    }
}

/// A tree asleep from the reset (sleep="init") carrying a damped fixed
/// tendon, given a velocity: the forward's kinematics wake the tree before
/// the tendon is made (`mj_kinematics` runs `mj_wake` between its body and
/// geom passes, `engine_core_smooth.c:236-241`), so the tendon's row,
/// velocity and damper force are MuJoCo's bit for bit (census doc
/// e7609717 is this, under implicit).
#[test]
fn a_tendon_on_a_tree_the_forward_wakes_is_made() {
    let golden = golden();
    let case = &golden["tendon_woken"];
    let model = model_of(case);
    let mut data = model.make_data();
    assert_eq!(data.tree_awake, vec![false], "asleep from the reset");
    data.qvel
        .as_mut_slice()
        .copy_from_slice(&floats(&case["qvel"]));
    data.forward(&model).expect("forward");
    assert_eq!(data.tree_awake, vec![true], "tree_awake");
    let bits = |v: &[f64]| v.iter().map(|x| x.to_bits()).collect::<Vec<_>>();
    assert_eq!(
        bits(data.ten_J[0].as_slice()),
        bits(&floats(&case["ten_J"])),
        "ten_J"
    );
    assert_eq!(
        bits(&data.ten_velocity),
        bits(&floats(&case["ten_velocity"])),
        "ten_velocity"
    );
    assert_eq!(
        bits(data.qfrc_passive.as_slice()),
        bits(&floats(&case["qfrc_passive"])),
        "qfrc_passive"
    );
}

/// A spatial tendon from a world site to a tree's site, its spring engaged,
/// whose row is 0 at this state (the site moves square to it), beside a
/// second tree; energy enabled.
const ZERO_ROW_TENDON: &str = r#"
<mujoco>
  <option gravity="0 0 0"><flag sleep="enable" energy="enable"/></option>
  <worldbody>
    <site name="w" pos="0.3 0.5 1"/>
    <body name="a" pos="0 0 1">
      <joint name="ja" type="hinge" axis="0 1 0"/>
      <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.02" mass="1" contype="0" conaffinity="0"/>
      <site name="sa" pos="0.3 0 0"/>
    </body>
    <body name="b" pos="1 0 1">
      <joint name="jb" type="hinge" axis="0 1 0"/>
      <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.02" mass="1" contype="0" conaffinity="0"/>
    </body>
  </worldbody>
  <tendon>
    <spatial name="t" stiffness="50" springlength="0.2 0.3">
      <site site="w"/><site site="sa"/>
    </spatial>
  </tendon>
</mujoco>"#;

/// A spatial tendon between two world sites (on no tree: static), its
/// spring engaged, beside two trees; energy enabled.
const STATIC_TENDON: &str = r#"
<mujoco>
  <option gravity="0 0 0"><flag sleep="enable" energy="enable"/></option>
  <worldbody>
    <site name="w" pos="0.3 0.5 1"/>
    <site name="w2" pos="0 -0.5 2"/>
    <body name="a" pos="0 0 1">
      <joint name="ja" type="hinge" axis="0 1 0"/>
      <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.02" mass="1" contype="0" conaffinity="0"/>
    </body>
    <body name="b" pos="1 0 1">
      <joint name="jb" type="hinge" axis="0 1 0"/>
      <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.02" mass="1" contype="0" conaffinity="0"/>
    </body>
  </worldbody>
  <tendon>
    <spatial name="t" stiffness="50" springlength="0.2 0.3"><site site="w"/><site site="w2"/></spatial>
  </tendon>
</mujoco>"#;

/// `xml` with tree `b` asleep from the reset, or with sleep disabled, and
/// `ten_force` set to 7 before a forward pass, so a force the pass does not
/// make shows.
fn forward_tendon_scene(xml: &str, b_asleep: bool, sleep: bool) -> sim_core::Data {
    let xml = if b_asleep {
        let out = xml.replace(
            r#"<body name="b" pos="1 0 1">"#,
            r#"<body name="b" pos="1 0 1" sleep="init">"#,
        );
        assert_ne!(out, xml, "tree b's body");
        out
    } else {
        xml.to_owned()
    };
    let mut model = load_model(&xml).expect("load");
    if !sleep {
        model.enableflags &= !ENABLE_SLEEP;
    }
    let mut data = model.make_data();
    data.ten_force[0] = 7.0;
    data.forward(&model).expect("forward");
    let awake = if b_asleep {
        vec![true, false]
    } else {
        vec![true, true]
    };
    assert_eq!(data.tree_awake, awake, "tree_awake");
    data
}

/// A tendon's spring energy where MuJoCo's `mj_energyPos` adds it: every
/// tendon awake by its trees, and, while no body sleeps, a static one too
/// (`engine_sensor.c:1631`, `:1679-1682`). On the zero-row scene, with
/// every tree awake, with tree b asleep and with sleep disabled,
/// `energy_potential` is MuJoCo's 1.0; on the static one, MuJoCo's
/// 32.81475155779856 with every tree awake and with sleep disabled, and 0
/// with tree b asleep (measured on the oracle). Before, a tendon whose row
/// reached no awake dof was left out whenever sleep was enabled: 0 on the
/// zero-row scene, 0 on the static one with every tree awake.
#[test]
fn a_tendon_spring_energy_is_skipped_as_mujoco() {
    for (name, xml, want) in [
        ("zero row", ZERO_ROW_TENDON, [1.0, 1.0, 1.0]),
        (
            "static",
            STATIC_TENDON,
            [32.81475155779856, 0.0, 32.81475155779856],
        ),
    ] {
        for ((state, b_asleep, sleep), want) in [
            ("every tree awake", false, true),
            ("tree b asleep", true, true),
            ("sleep disabled", false, false),
        ]
        .into_iter()
        .zip(want)
        {
            let got = forward_tendon_scene(xml, b_asleep, sleep).energy_potential;
            assert!(
                (got - want).abs() <= 1e-12 * f64::max(1.0, want),
                "{name}, {state}: energy_potential {got}, MuJoCo's {want}"
            );
        }
    }
}

/// `ten_force` is the force the passive pass applies: made for a tendon it
/// does not skip, whatever its row, and 0 for one it skips (a static tendon
/// while a tree sleeps, as MuJoCo's `mj_springdamper` skips it,
/// `engine_passive.c:123`, `:446-451`). On the zero-row scene with sleep
/// enabled, with every tree awake and with tree b asleep, it is the same
/// model's with sleep disabled, bit for bit; on the static one it is too
/// with every tree awake, and is 0 with tree b asleep. Before, a tendon
/// whose row reached no awake dof was skipped whenever sleep was enabled
/// and kept the force it held (7 here).
#[test]
fn ten_force_is_the_force_the_passive_pass_applies() {
    for (name, xml) in [("zero row", ZERO_ROW_TENDON), ("static", STATIC_TENDON)] {
        let off = forward_tendon_scene(xml, false, false).ten_force[0];
        assert!(
            off != 7.0 && off != 0.0,
            "{name}: sleep disabled, a force is made"
        );
        let awake = forward_tendon_scene(xml, false, true).ten_force[0];
        assert_eq!(
            awake.to_bits(),
            off.to_bits(),
            "{name}, every tree awake: ten_force {awake}"
        );
        let asleep = forward_tendon_scene(xml, true, true).ten_force[0];
        let want = if name == "static" { 0.0 } else { off };
        assert_eq!(
            asleep.to_bits(),
            want.to_bits(),
            "{name}, tree b asleep: ten_force {asleep}"
        );
    }
}

/// A fixed tendon over tree a's joint (coefficient 0) and tree b's
/// (coefficient 1), its spring stretched by 0.2 at the reset, tree b asleep
/// from it. The stiffness is 0 here: MuJoCo refuses a tree asleep at load
/// beside a two-tree tendon with stiffness or damping, so it is set after
/// loading; energy enabled.
const ONE_SIDED_TENDON: &str = r#"
<mujoco>
  <option gravity="0 0 0"><flag sleep="enable" energy="enable"/></option>
  <worldbody>
    <body name="a" pos="0 0 1">
      <joint name="ja" type="hinge" axis="0 1 0"/>
      <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.02" mass="1" contype="0" conaffinity="0"/>
    </body>
    <body name="b" pos="1 0 1" sleep="init">
      <joint name="jb" type="hinge" axis="0 1 0"/>
      <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.02" mass="1" contype="0" conaffinity="0"/>
    </body>
  </worldbody>
  <tendon>
    <fixed name="t" springlength="0.2"><joint joint="ja" coef="0"/><joint joint="jb" coef="1"/></fixed>
  </tendon>
</mujoco>"#;

/// An awake tendon whose row reaches only a sleeping dof applies its force
/// there, as MuJoCo's `mj_springdamper` (`engine_passive.c:446-451`, the
/// tendon awake with one of its two trees), and the sleeping dof's
/// `qfrc_spring`, which the pass does not clear (`:645-649`), gains it on
/// every pass: MuJoCo's 10 after `mj_makeData`, then 20, 30 and 40 after
/// three forwards, with `qfrc_passive` 10, summed over the awake dofs only
/// (`:677`), and `energy_potential` 1.0 (measured on the oracle; 10 and 1.0
/// with sleep disabled). Before, a tendon whose row reached no awake dof
/// was skipped: no force at the sleeping dof, and no energy.
#[test]
fn an_awake_tendon_over_a_sleeping_dof_applies_its_force_there_as_mujoco() {
    let mut model = load_model(ONE_SIDED_TENDON).expect("load");
    model.tendon_stiffness[0] = 50.0;
    let mut data = model.make_data();
    assert_eq!(data.tree_awake, [true, false], "tree a awake, b asleep");
    assert_eq!(data.qfrc_spring.as_slice(), [0.0, 10.0], "after make_data");
    for want in [20.0, 30.0, 40.0] {
        data.forward(&model).expect("forward");
        assert_eq!(data.ten_J[0].as_slice(), [0.0, 1.0], "the tendon's row");
        assert_eq!(data.qfrc_spring.as_slice(), [0.0, want], "qfrc_spring");
        assert_eq!(data.qfrc_passive.as_slice(), [0.0, 10.0], "qfrc_passive");
        assert_eq!(data.energy_potential, 1.0, "energy_potential");
    }
    model.enableflags &= !ENABLE_SLEEP;
    let mut data = model.make_data();
    data.forward(&model).expect("forward");
    data.forward(&model).expect("forward");
    assert_eq!(data.qfrc_spring.as_slice(), [0.0, 10.0], "sleep disabled");
    assert_eq!(data.energy_potential, 1.0, "sleep disabled");
}

/// A spatial tendon between two world sites in a model with no dofs, its
/// spring engaged; energy enabled.
const NO_DOF_TENDON: &str = r#"
<mujoco>
  <option><flag energy="enable"/></option>
  <worldbody>
    <site name="w" pos="0.3 0.5 1"/>
    <site name="w2" pos="0 -0.5 2"/>
  </worldbody>
  <tendon>
    <spatial name="t" stiffness="50" springlength="0.2 0.3"><site site="w"/><site site="w2"/></spatial>
  </tendon>
</mujoco>"#;

/// Where the passive pass makes no tendon force, `ten_force` is 0: with
/// springs and dampers both disabled, and in a model with no dofs, where
/// the pass returns before its tendons. `ten_force` is set to 7 before the
/// forward. The no-dof model's `energy_potential` is MuJoCo's
/// 32.81475155779856 (measured on the oracle). Before, both returns left
/// `ten_force` as it was (7 here).
#[test]
fn ten_force_is_0_where_the_passive_pass_makes_no_force() {
    for sleep in [true, false] {
        let mut model = load_model(ZERO_ROW_TENDON).expect("load");
        model.disableflags |= DISABLE_SPRING | DISABLE_DAMPER;
        if !sleep {
            model.enableflags &= !ENABLE_SLEEP;
        }
        let mut data = model.make_data();
        data.ten_force[0] = 7.0;
        data.forward(&model).expect("forward");
        assert_eq!(
            data.ten_force[0].to_bits(),
            0.0_f64.to_bits(),
            "springs and dampers disabled, sleep {sleep}: ten_force {}",
            data.ten_force[0]
        );
    }
    let model = load_model(NO_DOF_TENDON).expect("load");
    assert_eq!((model.nv, model.ntendon), (0, 1), "no dofs, one tendon");
    let mut data = model.make_data();
    data.ten_force[0] = 7.0;
    data.forward(&model).expect("forward");
    assert_eq!(
        data.ten_force[0].to_bits(),
        0.0_f64.to_bits(),
        "no dofs: ten_force {}",
        data.ten_force[0]
    );
    let want = 32.81475155779856;
    assert!(
        (data.energy_potential - want).abs() <= 1e-12 * want,
        "no dofs: energy_potential {}, MuJoCo's {want}",
        data.energy_potential
    );
}
