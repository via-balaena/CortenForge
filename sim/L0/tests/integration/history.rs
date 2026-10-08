//! Actuator and sensor delays (history buffers) against MuJoCo 3.5.0.
//!
//! The golden is `assets/golden/history/history.json`, from the unfused
//! MuJoCo 3.5.0 oracle (`scripts/gen_history_reference.py`, which describes
//! its models and drivers). Two of MuJoCo's results are not matched, each
//! a registry row with its test here: under RK4 a delayed sensor's sample is
//! taken at the step's start state (`D-HISTORY-RK4-SENSOR`,
//! `delayed_sensor_reads_the_value_one_step_earlier`), and the accelerometer
//! after `step1` + `step2` equals `step`'s (`D-STEP12-ACCELEROMETER`,
//! `step1_step2_accelerometer_equals_step`).

use serde_json::Value;
use sim_core::{Data, Integrator, MakeDataError, Model, ResetError};
use sim_mjcf::load_model;

const TOL: f64 = 1e-12;

fn golden() -> Value {
    serde_json::from_str(include_str!("../assets/golden/history/history.json"))
        .expect("history golden parses")
}

fn floats(v: &Value) -> Vec<f64> {
    v.as_array()
        .expect("array")
        .iter()
        .map(|x| match x {
            Value::String(s) => match s.as_str() {
                "nan" => f64::NAN,
                "inf" => f64::INFINITY,
                "-inf" => f64::NEG_INFINITY,
                other => panic!("unexpected string {other}"),
            },
            _ => x.as_f64().expect("number"),
        })
        .collect()
}

/// The golden's case `name`.
fn case<'a>(golden: &'a Value, name: &str) -> &'a Value {
    golden["cases"]
        .as_array()
        .expect("cases")
        .iter()
        .find(|c| c["name"] == name)
        .unwrap_or_else(|| panic!("no case {name}"))
}

/// The case's model. sim-mjcf reads a one-value `interval` until Rigid-loading
/// (an `"0.03 -0.01"` reads as no interval), so the interval-with-phase
/// sensor of the sensor models gets its period and phase here.
fn model_of(case: &Value) -> Model {
    let mut model = load_model(case["xml"].as_str().expect("xml")).expect("load");
    let name = case["name"].as_str().expect("name");
    if name.starts_with("sens_") {
        model.sensor_interval[11] = (0.03, -0.01);
    }
    model
}

/// The `Data` the case starts from: made, then reset to its keyframe.
fn start(model: &Model, case: &Value) -> Data {
    let mut data = model.make_data();
    if case["key"].as_u64().is_some() {
        data.reset_to_keyframe(model, 0).expect("keyframe");
    }
    data
}

/// The range of `history` that sensor `i`'s buffer occupies.
fn sensor_buffer(model: &Model, i: usize) -> std::ops::Range<usize> {
    let n = usize::try_from(model.sensor_nsample[i]).unwrap_or(0);
    if n == 0 {
        return 0..0;
    }
    let adr = usize::try_from(model.sensor_historyadr[i]).expect("adr");
    adr..adr + 2 + n + n * model.sensor_dim[i]
}

fn assert_close(what: &str, ours: &[f64], theirs: &[f64], skip: &[std::ops::Range<usize>]) {
    assert_eq!(ours.len(), theirs.len(), "{what}: length");
    for (j, (&a, &b)) in ours.iter().zip(theirs).enumerate() {
        if skip.iter().any(|r| r.contains(&j)) {
            continue;
        }
        assert!(
            (a - b).abs() <= TOL || (a.is_nan() && b.is_nan()),
            "{what}[{j}]: ours {a}, MuJoCo {b}"
        );
    }
}

/// Run `name` and compare it with the golden, skipping the listed sensors'
/// values and buffers.
fn run(golden: &Value, name: &str, skip_sensors: &[usize]) {
    let case = case(golden, name);
    let model = model_of(case);
    let mut data = start(&model, case);
    let skip_data: Vec<_> = skip_sensors
        .iter()
        .map(|&i| model.sensor_adr[i]..model.sensor_adr[i] + model.sensor_dim[i])
        .collect();
    let skip_hist: Vec<_> = skip_sensors
        .iter()
        .map(|&i| sensor_buffer(&model, i))
        .collect();
    assert_eq!(
        model.nhistory as u64,
        case["nhistory"].as_u64().unwrap(),
        "{name}: nhistory"
    );
    assert_close(
        &format!("{name} initial history"),
        &data.history,
        &floats(&case["init_history"]),
        &skip_hist,
    );
    let ctrls = case["ctrl"].as_array().expect("ctrl");
    let reset_at = case["reset_at"].as_u64();
    for (k, rec) in case["records"]
        .as_array()
        .expect("records")
        .iter()
        .enumerate()
    {
        if reset_at == Some(k as u64) {
            data.reset(&model);
        }
        data.ctrl.copy_from_slice(&floats(&ctrls[k]));
        let at = format!("{name} step {k}");
        assert!(
            (data.time - rec["time"].as_f64().unwrap()).abs() <= TOL,
            "{at}: time"
        );
        match case["driver"].as_str().expect("driver") {
            "step" => data.step(&model).expect("step"),
            "forward" => data.forward(&model).expect("forward"),
            "step12" => {
                data.step1(&model).expect("step1");
                data.step2(&model).expect("step2");
            }
            other => panic!("driver {other}"),
        }
        assert_close(
            &format!("{at} actuator_force"),
            &data.actuator_force,
            &floats(&rec["actuator_force"]),
            &[],
        );
        assert_close(
            &format!("{at} qpos"),
            data.qpos.as_slice(),
            &floats(&rec["qpos"]),
            &[],
        );
        assert_close(
            &format!("{at} qvel"),
            data.qvel.as_slice(),
            &floats(&rec["qvel"]),
            &[],
        );
        assert_close(
            &format!("{at} sensordata"),
            data.sensordata.as_slice(),
            &floats(&rec["sensordata"]),
            &skip_data,
        );
        assert_close(
            &format!("{at} history"),
            &data.history,
            &floats(&rec["history"]),
            &skip_hist,
        );
    }
}

/// Delayed actuators act on the control their buffer holds at `time - delay`,
/// with MuJoCo's interpolation, under each integrator and driver, across a
/// reset and from keyframes at positive and negative times.
#[test]
fn delayed_actuators_match_mujoco_3_5_0() {
    let golden = golden();
    for name in [
        "act_Euler",
        "act_RK4",
        "act_implicitfast",
        "act_forward_only",
        "act_step12_Euler",
        "act_step12_RK4",
        "act_reset_mid",
        "act_key_t0.5",
        "act_key_t-0.015",
        "act_key_t-0.02",
        "act_key_t-0.0333",
    ] {
        run(&golden, name, &[]);
    }
}

/// Delayed, buffered and interval sensors read and fill their buffers as
/// MuJoCo's. Not compared: under RK4 the delayed framepos, framequat and
/// accelerometer (sensors 6, 7, 9; `D-HISTORY-RK4-SENSOR`), and after
/// `step1` + `step2` the accelerometer (`D-STEP12-ACCELEROMETER`).
#[test]
fn delayed_sensors_match_mujoco_3_5_0() {
    let golden = golden();
    for (name, skip) in [
        ("sens_Euler", &[][..]),
        ("sens_implicitfast", &[]),
        ("sens_RK4", &[6, 7, 9]),
        ("sens_forward_only", &[]),
        ("sens_step12_Euler", &[9]),
        ("sens_step12_RK4", &[9]),
        ("sens_key_t0.5", &[]),
        ("sens_key_t-0.015", &[]),
        ("sens_key_t-0.02", &[]),
        ("acc_Euler", &[]),
        ("acc_implicitfast", &[]),
        ("acc_implicit", &[]),
        ("acc_RK4", &[1]),
    ] {
        run(&golden, name, skip);
    }
}

/// A buffer's initial timestamps (with an interval, rounded up to a timestep
/// from the phase) and the first compute tick, as MuJoCo's `_resetData`.
#[test]
fn sensor_history_initial_state_matches_mujoco() {
    let golden = golden();
    for case in golden["cases"].as_array().expect("cases") {
        let model = model_of(case);
        let data = start(&model, case);
        let theirs = floats(&case["init_history"]);
        for (j, (a, b)) in data.history.iter().zip(&theirs).enumerate() {
            assert_eq!(
                a.to_bits(),
                b.to_bits(),
                "{} history[{j}]: ours {a}, MuJoCo {b}",
                case["name"]
            );
        }
        assert_eq!(data.history.len(), theirs.len());
    }
}

/// A value read from a buffer is not clamped by the sensor's cutoff again
/// (MuJoCo applies the cutoff when it computes): sensor 13, a cubic jointvel
/// with cutoff 0.02, whose interpolated values the golden holds.
#[test]
fn interpolated_value_is_not_reclamped_by_cutoff() {
    let golden = golden();
    let case = case(&golden, "sens_Euler");
    let model = model_of(case);
    let adr = model.sensor_adr[13];
    let records = case["records"].as_array().expect("records");
    assert!(
        records
            .iter()
            .any(|r| floats(&r["sensordata"])[adr].abs() > 0.02),
        "the fixture reads a value beyond the cutoff"
    );
    run(&golden, "sens_Euler", &[]);
}

const ONE_STEP_DELAY: &str = r#"<mujoco>
  <option timestep="0.01" gravity="0 0 0"/>
  <worldbody>
    <body name="b">
      <joint name="j" type="hinge" axis="0 0 1" damping="0.05"/>
      <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.03" mass="0.5"/>
      <site name="s" pos="0.3 0 0"/>
    </body>
  </worldbody>
  <actuator><motor name="m" joint="j" gear="0.2"/></actuator>
  <sensor>
    <jointpos joint="j"/>
    <jointpos joint="j" nsample="2" delay="0.01"/>
    <framepos objtype="site" objname="s"/>
    <framepos objtype="site" objname="s" nsample="2" delay="0.01"/>
    <jointvel joint="j"/>
    <jointvel joint="j" nsample="2" delay="0.01"/>
    <accelerometer site="s"/>
    <accelerometer site="s" nsample="2" delay="0.01"/>
  </sensor>
</mujoco>"#;

/// Registry `D-HISTORY-RK4-SENSOR`: a sensor delayed by one timestep reads,
/// bit for bit, the undelayed sensor's value one step earlier, under every
/// integrator. MuJoCo 3.5.0 takes the RK4 sample from the last stage's
/// kinematics instead: on this model, framepos 3.1e-3 and the accelerometer
/// 6.8e-2 from the undelayed values one step earlier (measured on the
/// unfused oracle).
#[test]
fn delayed_sensor_reads_the_value_one_step_earlier() {
    for integrator in [
        Integrator::Euler,
        Integrator::RungeKutta4,
        Integrator::ImplicitFast,
        Integrator::Implicit,
    ] {
        let mut model = load_model(ONE_STEP_DELAY).expect("load");
        model.integrator = integrator;
        let mut data = model.make_data();
        let mut previous: Option<Vec<f64>> = None;
        for k in 0..20 {
            data.ctrl[0] = (0.9 * f64::from(k)).sin() + 0.05 * f64::from(k);
            data.step(&model).expect("step");
            let now = data.sensordata.as_slice().to_vec();
            if let Some(before) = &previous {
                for pair in [0, 2, 4, 6] {
                    let (undelayed, delayed) = (pair, pair + 1);
                    let (ua, da) = (model.sensor_adr[undelayed], model.sensor_adr[delayed]);
                    for d in 0..model.sensor_dim[undelayed] {
                        assert_eq!(
                            now[da + d].to_bits(),
                            before[ua + d].to_bits(),
                            "{integrator:?} step {k}: sensor {delayed}[{d}]"
                        );
                    }
                }
            }
            previous = Some(now);
        }
    }
}

/// Registry `D-STEP12-ACCELEROMETER`: `step1` + `step2` leave the
/// accelerometer `step` leaves. MuJoCo 3.5.0's are 5.9 off `mj_step`'s on
/// this model, on the same trajectory: `mj_step2` does not clear
/// `flg_rnepost`, so it reads `mj_step1`'s body accelerations; clearing the
/// flag between the two calls removes the difference (measured on the
/// unfused oracle).
#[test]
fn step1_step2_accelerometer_equals_step() {
    let model = load_model(ONE_STEP_DELAY).expect("load");
    let (mut whole, mut split) = (model.make_data(), model.make_data());
    for k in 0..20 {
        let ctrl = (0.9 * f64::from(k)).sin() + 0.05 * f64::from(k);
        whole.ctrl[0] = ctrl;
        split.ctrl[0] = ctrl;
        whole.step(&model).expect("step");
        split.step1(&model).expect("step1");
        split.step2(&model).expect("step2");
        assert_eq!(whole.qpos, split.qpos, "step {k}: same trajectory");
        let adr = model.sensor_adr[6];
        assert_eq!(
            whole.sensordata.as_slice()[adr..adr + 3],
            split.sensordata.as_slice()[adr..adr + 3],
            "step {k}: accelerometer"
        );
    }
}

/// A user sensor with a delay cannot be computed when its sample is inserted
/// (MuJoCo `mjERROR`s there): `try_make_data` and `try_reset` refuse it.
/// MJCF cannot express one; a code-built model can.
#[test]
fn try_make_data_refuses_a_delayed_user_sensor() {
    let mut model = load_model(
        r#"<mujoco><worldbody><body><joint type="hinge"/>
<geom type="sphere" size="0.1" mass="1"/></body></worldbody>
<sensor><user dim="1"/></sensor></mujoco>"#,
    )
    .expect("load");
    let mut data = model.make_data();
    model.sensor_nsample[0] = 2;
    model.sensor_delay[0] = 0.002;
    model.recompute_derived().expect("derive");
    assert_eq!(
        model.try_make_data().err(),
        Some(MakeDataError::DelayedUserSensor { sensor: 0 })
    );
    data.history = vec![0.0; model.nhistory];
    assert_eq!(
        data.try_reset(&model),
        Err(ResetError::DelayedUserSensor { sensor: 0 })
    );
}

/// History timestamps are multiples of the timestep, so MuJoCo's
/// `_resetData` refuses a model with a buffer and a timestep that is not
/// positive; so do `try_make_data` and `try_reset`.
#[test]
fn try_reset_refuses_history_with_a_nonpositive_timestep() {
    let mut model = load_model(ONE_STEP_DELAY).expect("load");
    let mut data = model.make_data();
    for timestep in [0.0, -0.01] {
        model.timestep = timestep;
        assert_eq!(
            model.try_make_data().err(),
            Some(MakeDataError::InvalidTimestep)
        );
        assert_eq!(data.try_reset(&model), Err(ResetError::InvalidTimestep));
    }
}

/// `InterpolationType` from MuJoCo's integer code; another value is refused.
#[test]
fn interpolation_type_comes_from_mujoco_codes() {
    use sim_core::InterpolationType;
    assert_eq!(InterpolationType::try_from(0), Ok(InterpolationType::Zoh));
    assert_eq!(
        InterpolationType::try_from(1),
        Ok(InterpolationType::Linear)
    );
    assert_eq!(InterpolationType::try_from(2), Ok(InterpolationType::Cubic));
    assert_eq!(InterpolationType::try_from(3), Err(3));
    assert_eq!(InterpolationType::try_from(-1), Err(-1));
}
