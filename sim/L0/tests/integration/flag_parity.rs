//! Runtime flags against MuJoCo 3.5.0, each on a scene the flag changes.
//!
//! `golden_flags` runs every flag on one scene against 3.4.0 goldens, and
//! twelve of those goldens are within its tolerance of the baseline's
//! (`golden_flags::goldens_within_tolerance_of_the_baseline` pins which), so
//! their tests cannot tell their flag ignored. Here nine of those flags
//! each run on a scene the flag changes, with and without the flag. The
//! golden is `assets/golden/flags/flags_3_5_0.json`, from the unfused
//! MuJoCo 3.5.0 oracle (`scripts/gen_flag_reference.py`, which describes
//! the scenes and says why midphase, nativeccd and multiccd are not
//! here).

use serde_json::Value;
use sim_core::{
    DISABLE_AUTORESET, DISABLE_ISLAND, DISABLE_SENSOR, DISABLE_WARMSTART, Data, ENABLE_ENERGY,
    ENABLE_FWDINV, ENABLE_INVDISCRETE, ENABLE_OVERRIDE, ENABLE_SLEEP,
};
use sim_mjcf::load_model;

fn golden() -> Value {
    serde_json::from_str(include_str!("../assets/golden/flags/flags_3_5_0.json"))
        .expect("flag golden parses")
}

fn floats(v: &Value) -> Vec<f64> {
    v.as_array()
        .expect("array")
        .iter()
        .map(|x| x.as_f64().expect("number"))
        .collect()
}

fn count(v: &Value) -> usize {
    usize::try_from(v.as_u64().expect("count")).expect("count fits")
}

/// `|ours - want| / max(1, |want|)`, the largest over the entries.
fn distance(ours: &[f64], want: &[f64]) -> f64 {
    assert_eq!(ours.len(), want.len(), "lengths");
    ours.iter()
        .zip(want)
        .map(|(x, y)| (x - y).abs() / y.abs().max(1.0))
        .fold(0.0, f64::max)
}

const CHANNELS: [&str; 6] = ["qpos", "qvel", "qacc", "sensordata", "energy", "fwdinv"];

/// Each channel's distance from MuJoCo's, here and in the generator.
const TOLERANCE: f64 = 1e-8;

/// A flagged run must be further than this from its baseline, here and in
/// the generator: a sim-core that ignored the flag is then more than
/// `TOLERANCE` from MuJoCo's flagged run.
const REFUSE_BELOW: f64 = 1e-5;

fn channel(data: &Data, name: &str) -> Vec<f64> {
    match name {
        "qpos" => data.qpos.as_slice().to_vec(),
        "qvel" => data.qvel.as_slice().to_vec(),
        "qacc" => data.qacc.as_slice().to_vec(),
        "sensordata" => data.sensordata.as_slice().to_vec(),
        "energy" => vec![data.energy_potential, data.energy_kinetic],
        "fwdinv" => data.solver_fwdinv.to_vec(),
        _ => unreachable!("{name}"),
    }
}

/// The case's scene stepped from `make_data`, every ctrl at the case's,
/// once without the flag and once with it: each step's channels within
/// `TOLERANCE` of MuJoCo's, `nisland`, the iterations and each warning's
/// count equal. First, the golden was written with this file's two
/// constants, and its flagged run is more than `REFUSE_BELOW` from its
/// baseline (the generator's refusal, checked here too), so a sim-core
/// that ignored the flag would fail the flagged run.
fn check(name: &str, kind: &str, flag: u32) {
    let g = golden();
    assert_eq!(g["tolerance"].as_f64(), Some(TOLERANCE), "golden tolerance");
    assert_eq!(
        g["refuse_below"].as_f64(),
        Some(REFUSE_BELOW),
        "golden refuse_below"
    );
    let (tolerance, refuse_below) = (TOLERANCE, REFUSE_BELOW);
    let case = &g["cases"][name];
    assert_eq!(case["kind"], kind, "{name}: kind");
    let (baseline, flagged) = (
        case["baseline"].as_array().expect("baseline"),
        case["flagged"].as_array().expect("flagged"),
    );
    assert_eq!(baseline.len(), count(&g["nsteps"]), "{name}: steps");
    let gap = baseline
        .iter()
        .zip(flagged)
        .flat_map(|(a, b)| CHANNELS.map(|ch| distance(&floats(&a[ch]), &floats(&b[ch]))))
        .fold(0.0, f64::max);
    assert!(
        gap > refuse_below,
        "{name}: the golden's flagged run is {gap:e} from its baseline"
    );

    for (run, steps, set) in [("baseline", baseline, false), ("flagged", flagged, true)] {
        let mut model = load_model(case["xml"].as_str().expect("xml")).expect("load");
        if set && kind == "disable" {
            model.disableflags |= flag;
        } else if set {
            model.enableflags |= flag;
        }
        let mut data = model.make_data();
        data.ctrl.fill(case["ctrl"].as_f64().expect("ctrl"));
        for (step, want) in steps.iter().enumerate() {
            data.step(&model).expect("step");
            for ch in CHANNELS {
                let d = distance(&channel(&data, ch), &floats(&want[ch]));
                assert!(
                    d <= tolerance,
                    "{name} {run} step {step}: {ch} {d:e} from MuJoCo's"
                );
            }
            assert_eq!(
                data.nisland,
                count(&want["nisland"]),
                "{name} {run} step {step}: nisland"
            );
            assert_eq!(
                data.solver_niter,
                count(&want["niter"]),
                "{name} {run} step {step}: iterations"
            );
            let warnings: Vec<usize> = data
                .warnings
                .iter()
                .map(|w| usize::try_from(w.count).expect("count"))
                .collect();
            let want_warnings: Vec<usize> = want["warnings"]
                .as_array()
                .expect("warnings")
                .iter()
                .map(count)
                .collect();
            assert_eq!(
                warnings, want_warnings,
                "{name} {run} step {step}: warnings"
            );
        }
    }
}

macro_rules! flag_case {
    ($test:ident, $name:literal, $kind:literal, $flag:expr) => {
        #[test]
        fn $test() {
            check($name, $kind, $flag);
        }
    };
}

flag_case!(
    golden_3_5_0_disable_autoreset,
    "autoreset",
    "disable",
    DISABLE_AUTORESET
);
flag_case!(
    golden_3_5_0_disable_sensor,
    "sensor",
    "disable",
    DISABLE_SENSOR
);
flag_case!(
    golden_3_5_0_enable_energy,
    "energy",
    "enable",
    ENABLE_ENERGY
);
flag_case!(
    golden_3_5_0_enable_fwdinv,
    "fwdinv",
    "enable",
    ENABLE_FWDINV
);
flag_case!(
    golden_3_5_0_enable_override,
    "override",
    "enable",
    ENABLE_OVERRIDE
);
flag_case!(
    golden_3_5_0_disable_island,
    "island",
    "disable",
    DISABLE_ISLAND
);
flag_case!(
    golden_3_5_0_disable_warmstart,
    "warmstart",
    "disable",
    DISABLE_WARMSTART
);
flag_case!(golden_3_5_0_enable_sleep, "sleep", "enable", ENABLE_SLEEP);
flag_case!(
    golden_3_5_0_enable_invdiscrete,
    "invdiscrete",
    "enable",
    ENABLE_INVDISCRETE
);

/// Every case in the golden has a test above.
#[test]
fn every_golden_case_is_tested() {
    let g = golden();
    let mut names: Vec<&str> = g["cases"]
        .as_object()
        .expect("cases")
        .keys()
        .map(String::as_str)
        .collect();
    names.sort_unstable();
    assert_eq!(
        names,
        [
            "autoreset",
            "energy",
            "fwdinv",
            "invdiscrete",
            "island",
            "override",
            "sensor",
            "sleep",
            "warmstart"
        ]
    );
}
