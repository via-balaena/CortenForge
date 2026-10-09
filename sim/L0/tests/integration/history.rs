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
use sim_core::{
    Data, HistoryError, Integrator, InterpolationType, MakeDataError, MjSensorType, Model,
    ResetError,
};
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
    let mut model = model_of(case);
    if case["driver"] == "step_cb" {
        // The generator's control callback, on every actuator.
        model.set_control_callback(|_, data| {
            let ctrl = (37.0 * data.time).sin() + 3.0 * data.qpos[0];
            data.ctrl.fill(ctrl);
        });
    }
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
            "step" | "step_cb" => data.step(&model).expect("step"),
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
/// with MuJoCo's interpolation, under Euler, RK4 and implicitfast and each
/// driver, across a reset and from keyframes at positive and negative times.
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
/// `flg_rnepost`, so it reads body accelerations an earlier pass left
/// (`mj_step1` computes none); clearing the flag between the two calls
/// removes the difference (measured on the unfused oracle).
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
/// MuJoCo's schema has no delay on a user sensor; sim-mjcf loads one until
/// Rigid-loading L48, and `make_data` then refuses it.
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
    // A plugin sensor's sample cannot be computed either; this user sensor
    // is made into one, as no MJCF loads a plugin sensor.
    for kind in [MjSensorType::User, MjSensorType::Plugin] {
        model.sensor_type[0] = kind;
        assert_eq!(
            model.try_make_data().err(),
            Some(MakeDataError::DelayedUserSensor { sensor: 0 }),
            "{kind:?}"
        );
        data.history = vec![0.0; model.nhistory];
        assert_eq!(
            data.try_reset(&model),
            Err(ResetError::DelayedUserSensor { sensor: 0 }),
            "{kind:?}"
        );
    }
}

/// Under RK4 a delayed actuator's buffer takes `ctrl` as the last stage's
/// control callback left it, as MuJoCo's `mj_RungeKutta` inserts it after
/// the stages (`engine_forward.c:1113-1121`): the act model with a callback
/// that writes `ctrl = sin(37 t) + 3 qpos[0]`.
#[test]
fn rk4_buffers_take_the_last_stage_control() {
    run(&golden(), "act_RK4_control_cb", &[]);
}

/// A negative delay reads the newest sample, as MuJoCo tests a delay for
/// non-zero (`mj_readCtrl`): the act model with delays of -1 and -0.3
/// timesteps.
#[test]
fn a_negative_delay_reads_the_newest_sample() {
    run(&golden(), "act_negative_delay", &[]);
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
    assert_eq!(InterpolationType::try_from(0), Ok(InterpolationType::Zoh));
    assert_eq!(
        InterpolationType::try_from(1),
        Ok(InterpolationType::Linear)
    );
    assert_eq!(InterpolationType::try_from(2), Ok(InterpolationType::Cubic));
    assert_eq!(InterpolationType::try_from(3), Err(3));
    assert_eq!(InterpolationType::try_from(-1), Err(-1));
}

fn api_golden() -> Value {
    serde_json::from_str(include_str!("../assets/golden/history/history_api.json"))
        .expect("history API golden parses")
}

/// The API golden's `kind` model ("act" or "sens"), with the sensor model's
/// interval phase set in code as [`model_of`] sets it.
fn api_model(golden: &Value, kind: &str) -> Model {
    let xml = golden["api"][kind]["xml"].as_str().expect("xml");
    let mut model = load_model(xml).expect("load");
    if kind == "sens" {
        model.sensor_interval[11] = (0.03, -0.01);
    }
    model
}

/// The API golden's `kind` model after the 12 Euler steps it takes, at its
/// control sequence.
fn after_twelve_steps(golden: &Value, kind: &str) -> (Model, Data) {
    let model = api_model(golden, kind);
    let mut data = model.make_data();
    for k in 0..12 {
        data.ctrl
            .fill((0.9 * f64::from(k)).sin() + 0.05 * f64::from(k));
        data.step(&model).expect("step");
    }
    (model, data)
}

/// MuJoCo's interpolation code, -1 meaning the model's own.
fn interp_of(code: &Value) -> Option<InterpolationType> {
    let code = i32::try_from(code.as_i64().expect("interp")).expect("small");
    (code >= 0).then(|| InterpolationType::try_from(code).expect("0, 1 or 2"))
}

/// `Data::read_ctrl` and `Data::read_sensor` are MuJoCo's `mj_readCtrl` and
/// `mj_readSensor`: every actuator and sensor of the golden models, after 12
/// steps, at 55 times around and inside their buffers, with each
/// interpolation.
#[test]
fn history_reads_match_mujoco_3_5_0() {
    let golden = api_golden();
    for kind in ["act", "sens"] {
        let case = &golden["api"][kind];
        let (model, mut data) = after_twelve_steps(&golden, kind);
        assert_close(
            &format!("{kind} history"),
            &data.history,
            &floats(&case["history"]),
            &[],
        );
        for read in case["reads"].as_array().expect("reads") {
            let id = usize::try_from(read[0].as_u64().expect("id")).expect("id");
            let interp = interp_of(&read[1]);
            let time = read[2].as_f64().expect("time");
            let theirs = floats(&read[3]);
            let ours = if kind == "act" {
                vec![data.read_ctrl(&model, id, time, interp).expect("read_ctrl")]
            } else {
                let mut out = vec![0.0; model.sensor_dim[id]];
                data.read_sensor(&model, id, time, interp, &mut out)
                    .expect("read_sensor");
                out
            };
            assert_close(
                &format!("{kind} {id} at {time} interp {interp:?}"),
                &ours,
                &theirs,
                &[],
            );
        }
        if kind == "act" {
            // an init where the cursor is not at the last slot (actuator 8,
            // 5 samples, after 12 steps): the given order becomes the stored one
            data.init_ctrl_history(
                &model,
                8,
                Some(&[-0.03, -0.02, -0.01, 0.0, 0.05]),
                Some(&[1.0, 2.0, 3.0, 4.0, 5.0]),
            )
            .expect("init 8");
            // To the tolerance, not bit for bit: the other slots hold samples
            // of controls computed with `f64::sin`, and bit for bit slot 7
            // differed by 4 ULP on CI's Linux runner, not on macOS.
            assert_close(
                "after init: history",
                &data.history,
                &floats(&case["history_after_init"]),
                &[],
            );
        }
    }
}

/// `Data::init_ctrl_history` and `Data::init_sensor_history` leave the
/// buffers MuJoCo's `mj_initCtrlHistory` and `mj_initSensorHistory` leave,
/// bit for bit: new times and values, kept times with new values (the
/// buffer's own times, in their stored order), and a sensor's phase. The
/// delayed actuators then act on the values written.
#[test]
fn history_inits_match_mujoco_3_5_0() {
    let golden = api_golden();
    let init = &golden["api"]["init"];
    let model = api_model(&golden, "act");
    let mut data = model.make_data();
    data.init_ctrl_history(
        &model,
        2,
        Some(&[-0.03, -0.02, -0.01]),
        Some(&[1.0, 2.0, 3.0]),
    )
    .expect("init 2");
    data.init_ctrl_history(&model, 7, None, Some(&[0.5, -0.5, 0.25, 4.0, 1.0]))
        .expect("init 7");
    let theirs = floats(&init["act_history"]);
    assert_eq!(data.history.len(), theirs.len());
    for (j, (a, b)) in data.history.iter().zip(&theirs).enumerate() {
        assert_eq!(
            a.to_bits(),
            b.to_bits(),
            "act history[{j}]: ours {a}, MuJoCo {b}"
        );
    }
    for (k, forces) in init["act_forces"]
        .as_array()
        .expect("forces")
        .iter()
        .enumerate()
    {
        data.ctrl.fill(0.0);
        data.step(&model).expect("step");
        assert_close(
            &format!("force after init, step {k}"),
            &data.actuator_force,
            &floats(forces),
            &[],
        );
    }

    let model = api_model(&golden, "sens");
    let mut data = model.make_data();
    let values: Vec<f64> = (0..12).map(f64::from).collect();
    data.init_sensor_history(&model, 6, None, Some(&values), 0.123)
        .expect("init sensor 6");
    let theirs = floats(&init["sens_history"]);
    for (j, (a, b)) in data.history.iter().zip(&theirs).enumerate() {
        assert_eq!(
            a.to_bits(),
            b.to_bits(),
            "sens history[{j}]: ours {a}, MuJoCo {b}"
        );
    }
}

/// The API returns the reason where MuJoCo `mjERROR`s (a bad index, no
/// buffer, times that do not increase) and for a slice of the wrong length,
/// which MuJoCo's C does not check (its Python binding does).
#[test]
fn history_api_refuses_what_mujoco_refuses() {
    let golden = api_golden();
    let refused = &golden["api"]["init"]["refused"];
    let model = api_model(&golden, "act");
    let mut data = model.make_data();
    assert!(
        refused["bad_actuator"].is_string(),
        "MuJoCo refuses actuator 99"
    );
    assert_eq!(
        data.read_ctrl(&model, 99, 0.0, None),
        Err(HistoryError::InvalidActuator {
            id: 99,
            nu: model.nu
        })
    );
    assert!(
        refused["no_buffer"].is_string(),
        "MuJoCo refuses actuator 0"
    );
    assert_eq!(
        data.init_ctrl_history(&model, 0, None, Some(&[])),
        Err(HistoryError::NoBuffer)
    );
    assert!(
        refused["not_increasing"].is_string(),
        "MuJoCo refuses equal times"
    );
    assert_eq!(
        data.init_ctrl_history(&model, 2, Some(&[0.0, 0.0, 1.0]), Some(&[0.0; 3])),
        Err(HistoryError::TimesNotIncreasing { index: 0 })
    );
    assert_eq!(
        data.init_ctrl_history(&model, 2, Some(&[0.0, 1.0]), None),
        Err(HistoryError::WrongLength {
            expected: 3,
            actual: 2
        })
    );

    let model = api_model(&golden, "sens");
    let data = model.make_data();
    let mut out = [0.0; 2];
    assert_eq!(
        data.read_sensor(&model, 99, 0.0, None, &mut out),
        Err(HistoryError::InvalidSensor {
            id: 99,
            nsensor: model.nsensor
        })
    );
    assert_eq!(
        data.read_sensor(&model, 6, 0.0, None, &mut out),
        Err(HistoryError::WrongLength {
            expected: 3,
            actual: 2
        })
    );
}

/// `None` keeps what the buffer holds: new times with the values kept, then
/// new values with the times kept (MuJoCo's C API takes NULL for either;
/// its Python binding, which the golden comes from, takes `None` for the
/// times only).
#[test]
fn history_init_keeps_what_it_is_not_given() {
    let golden = api_golden();
    let model = api_model(&golden, "act");
    let mut data = model.make_data();
    let adr = usize::try_from(model.actuator_historyadr[4]).expect("adr");
    data.history[adr + 6..adr + 10].copy_from_slice(&[1.0, 2.0, 3.0, 4.0]);
    data.init_ctrl_history(&model, 4, Some(&[-0.05, -0.03, -0.02, -0.01]), None)
        .expect("times only");
    assert_eq!(
        data.history[adr + 2..adr + 10],
        [-0.05, -0.03, -0.02, -0.01, 1.0, 2.0, 3.0, 4.0]
    );
    data.init_ctrl_history(&model, 4, None, Some(&[5.0, 6.0, 7.0, 8.0]))
        .expect("values only");
    assert_eq!(
        data.init_ctrl_history(&model, 2, None, Some(&[1.0, 2.0])),
        Err(HistoryError::WrongLength {
            expected: 3,
            actual: 2
        })
    );
    assert_eq!(
        data.history[adr + 2..adr + 10],
        [-0.05, -0.03, -0.02, -0.01, 5.0, 6.0, 7.0, 8.0]
    );
}

/// A value the user writes into an actuator buffer's slot 0 (MuJoCo's user
/// slot) is kept by `Data::init_ctrl_history`, bit for bit, and by the steps
/// after, as MuJoCo's `mj_initCtrlHistory` and `mj_step` keep it.
#[test]
fn history_init_keeps_the_user_slot() {
    let golden = api_golden();
    let user = &golden["api"]["init"]["user_slot"];
    let model = api_model(&golden, "act");
    let mut data = model.make_data();
    for written in user["written"].as_array().expect("written") {
        let id = usize::try_from(written[0].as_u64().expect("id")).expect("id");
        let adr = usize::try_from(model.actuator_historyadr[id]).expect("adr");
        data.history[adr] = written[1].as_f64().expect("value");
    }
    data.init_ctrl_history(
        &model,
        2,
        Some(&[-0.03, -0.02, -0.01]),
        Some(&[1.0, 2.0, 3.0]),
    )
    .expect("init 2");
    data.init_ctrl_history(&model, 7, None, Some(&[0.5, -0.5, 0.25, 4.0, 1.0]))
        .expect("init 7");
    let theirs = floats(&user["after_init"]);
    assert_eq!(data.history.len(), theirs.len());
    for (j, (a, b)) in data.history.iter().zip(&theirs).enumerate() {
        assert_eq!(
            a.to_bits(),
            b.to_bits(),
            "history[{j}] after init: ours {a}, MuJoCo {b}"
        );
    }
    for _ in 0..3 {
        data.step(&model).expect("step");
    }
    assert_close(
        "history after 3 steps",
        &data.history,
        &floats(&user["after_steps"]),
        &[],
    );
}

/// A `Data` of another shape is refused, as the step's input check refuses
/// one (registry `D-DATA-SHAPE`), not read or written past its end: each call
/// on a `Data` whose `history` is one shorter than its model's, and a read of
/// an unbuffered actuator or sensor (which reads `ctrl` or `sensordata`) on
/// one whose `ctrl` or `sensordata` is.
#[test]
fn history_api_refuses_a_data_of_another_shape() {
    let golden = api_golden();
    let refusal = |model: &Model| HistoryError::DataShapeMismatch {
        field: "history",
        expected: model.nhistory,
        actual: model.nhistory - 1,
    };
    let act = api_model(&golden, "act");
    let mut data = act.make_data();
    data.history.pop();
    assert_eq!(
        data.read_ctrl(&act, 2, 0.0, None).err(),
        Some(refusal(&act))
    );
    assert_eq!(
        data.init_ctrl_history(&act, 2, None, Some(&[1.0, 2.0, 3.0]))
            .err(),
        Some(refusal(&act))
    );
    let sens = api_model(&golden, "sens");
    let mut data = sens.make_data();
    data.history.pop();
    let mut out = vec![0.0; sens.sensor_dim[6]];
    assert_eq!(
        data.read_sensor(&sens, 6, 0.0, None, &mut out).err(),
        Some(refusal(&sens))
    );
    let values: Vec<f64> = (0..12).map(f64::from).collect();
    assert_eq!(
        data.init_sensor_history(&sens, 6, None, Some(&values), 0.0)
            .err(),
        Some(refusal(&sens))
    );

    let mut data = act.make_data();
    data.ctrl = nalgebra::DVector::zeros(act.nu - 1);
    let unbuffered = (0..act.nu)
        .find(|&i| act.actuator_nsample[i] == 0)
        .expect("an unbuffered actuator");
    assert_eq!(
        data.read_ctrl(&act, unbuffered, 0.0, None).err(),
        Some(HistoryError::DataShapeMismatch {
            field: "ctrl",
            expected: act.nu,
            actual: act.nu - 1,
        })
    );
    let mut data = sens.make_data();
    data.sensordata = nalgebra::DVector::zeros(sens.nsensordata - 1);
    let unbuffered = (0..sens.nsensor)
        .find(|&i| sens.sensor_nsample[i] == 0)
        .expect("an unbuffered sensor");
    let mut out = vec![0.0; sens.sensor_dim[unbuffered]];
    assert_eq!(
        data.read_sensor(&sens, unbuffered, 0.0, None, &mut out)
            .err(),
        Some(HistoryError::DataShapeMismatch {
            field: "sensordata",
            expected: sens.nsensordata,
            actual: sens.nsensordata - 1,
        })
    );
}

/// An init with `None` times keeps the buffer's own times in their stored
/// order, as MuJoCo's does: after 12 steps those times no longer increase
/// where they are stored, and both refuse.
#[test]
fn an_init_with_the_buffers_own_times_refuses_them_out_of_order() {
    let golden = api_golden();
    let theirs = golden["api"]["act"]["stored_order"]
        .as_str()
        .expect("MuJoCo refuses");
    assert!(
        theirs.contains("times must be strictly increasing, got times[1]"),
        "{theirs}"
    );
    let (model, mut data) = after_twelve_steps(&golden, "act");
    assert_eq!(
        data.init_ctrl_history(&model, 7, None, Some(&[0.5, -0.5, 0.25, 4.0, 1.0])),
        Err(HistoryError::TimesNotIncreasing { index: 1 })
    );
}
