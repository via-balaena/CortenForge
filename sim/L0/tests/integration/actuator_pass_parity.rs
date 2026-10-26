//! MuJoCo 3.5.0's actuation passes (`mj_fwdActuation`,
//! `engine_forward.c:277-570`) against sim-core's: every actuator's act_dot,
//! then the plugins', then the forces, then the plugins', then the clamp and
//! the mapping over every actuator.
//!
//! The golden is `assets/golden/actuator_pass/actuator_pass.json`, from
//! `scripts/gen_actuator_pass_reference.py` on the unfused oracle: two damped
//! hinges under implicitfast, 12 drawn states each, `act_dot` set before each
//! pass. Its cases: a callback reading another actuator's act_dot or force, or
//! its own force before its bias, or `qfrc_actuator` as the pass before left
//! it; a dynamics callback writing an earlier actuator's act_dot or a later
//! one's ctrl, or adding to `qfrc_actuator`; a `mujoco.pid` plugin's
//! actuator, with an integral term, a filter, a force range, the dynamics
//! callback called for it, and a user gain reading its act_dot or its force;
//! a force range that excludes 0 on a disabled actuator, and on a sleeping
//! tree's (its `tree_asleep` checked too); two actuators sharing an
//! activation; an infinite gear on actuators with no force. sim-mjcf
//! refuses a user type and loads no plugin, so each case loads a stand-in and
//! changes it to MuJoCo's compiled actuators, then checks it against them; the
//! callbacks below are the script's, in the same operations, and the plugin is
//! [`Pid`], a port of `mujoco.pid`. After a forward pass each actuator's
//! force, `qfrc_actuator` and `act_dot` are compared; after a step `qDeriv` on
//! D's pattern, and qpos, qvel and act; bit for bit on the golden's platform
//! (see `matches_golden`).

use std::sync::Arc;

use serde_json::Value;
use sim_core::plugin::{Plugin, PluginCapabilities, PluginCapabilityBit, PluginStage};
use sim_core::{ActuatorDynamics, BiasType, Data, GainType, Model};

fn golden() -> Value {
    serde_json::from_str(include_str!(
        "../assets/golden/actuator_pass/actuator_pass.json"
    ))
    .expect("actuator pass golden parses")
}

/// A number, or the script's "inf" for an infinite one.
fn float(v: &Value) -> f64 {
    match v.as_str() {
        Some("inf") => f64::INFINITY,
        _ => v.as_f64().expect("number"),
    }
}

fn floats(v: &Value) -> Vec<f64> {
    v.as_array().expect("array").iter().map(float).collect()
}

fn index(v: &Value) -> usize {
    usize::try_from(v.as_u64().expect("index")).expect("fits")
}

fn ints(v: &Value) -> Vec<i64> {
    v.as_array()
        .expect("array")
        .iter()
        .map(|x| x.as_i64().expect("integer"))
        .collect()
}

fn usizes(v: &Value) -> Vec<usize> {
    ints(v)
        .into_iter()
        .map(|x| usize::try_from(x).expect("index"))
        .collect()
}

fn strings(v: &Value) -> Vec<&str> {
    v.as_array()
        .expect("array")
        .iter()
        .map(|x| x.as_str().expect("string"))
        .collect()
}

/// Bit for bit on macOS arm64, where the golden was made. The states turn
/// hinges through `sin` and `cos`, which another platform's libm may round
/// otherwise by an ulp; there each entry is within `1e-13 · max(1,
/// |MuJoCo's|)`, a NaN failing.
fn matches_golden(ours: &[f64], mj: &[f64]) -> bool {
    ours.len() == mj.len()
        && ours.iter().zip(mj).all(|(a, b)| {
            a.to_bits() == b.to_bits()
                || (!cfg!(all(target_os = "macos", target_arch = "aarch64"))
                    && (a - b).abs() <= 1e-13 * b.abs().max(1.0))
        })
}

/// `mujoco.pid` (`plugin/actuator/pid.cc`) without `imax` or `slewmax`, in its
/// operations: each of its actuators' activation derivative (`Pid::ActDot`)
/// and force (`Pid::Compute`).
struct Pid {
    kp: f64,
    ki: f64,
    kd: f64,
}

impl Pid {
    fn actuators(model: &Model, instance: usize) -> impl Iterator<Item = usize> + '_ {
        (0..model.nu).filter(move |&i| model.actuator_plugin[i] == Some(instance))
    }

    /// `Pid::GetCtrl`: the control, clamped to its range, with no dynamics;
    /// else the last activation, the next one under `actearly` (the plugin's
    /// own `NextActivation`, Euler here).
    fn ctrl(model: &Model, data: &Data, i: usize, actearly: bool) -> f64 {
        if model.actuator_dyntype[i] == ActuatorDynamics::None {
            let (lo, hi) = model.actuator_ctrlrange[i];
            return data.ctrl[i].clamp(lo, hi);
        }
        let adr = model.actuator_act_adr[i] + model.actuator_act_num[i] - 1;
        if !actearly {
            return data.act[adr];
        }
        assert_ne!(model.actuator_dyntype[i], ActuatorDynamics::FilterExact);
        let act = data.act[adr] + data.act_dot[adr] * model.timestep;
        if model.actuator_actlimited[i] {
            let (lo, hi) = model.actuator_actrange[i];
            act.clamp(lo, hi)
        } else {
            act
        }
    }
}

impl Plugin for Pid {
    fn name(&self) -> &'static str {
        "mujoco.pid"
    }

    fn capabilities(&self) -> PluginCapabilities {
        PluginCapabilityBit::Actuator.into()
    }

    fn actuator_act_dot(&self, model: &Model, data: &mut Data, instance: usize) {
        for i in Self::actuators(model, instance) {
            let error = Self::ctrl(model, data, i, false) - data.actuator_length[i];
            let state = model.actuator_act_adr[i];
            if self.ki != 0.0 {
                let integral = data.act[state] + error * model.timestep;
                data.act_dot[state] = (integral - data.act[state]) / model.timestep;
            }
        }
    }

    fn compute(&self, model: &Model, data: &mut Data, instance: usize, _: PluginCapabilityBit) {
        for i in Self::actuators(model, instance) {
            let error =
                Self::ctrl(model, data, i, model.actuator_actearly[i]) - data.actuator_length[i];
            let ctrl_dot = if model.actuator_dyntype[i] == ActuatorDynamics::None {
                0.0
            } else {
                data.act_dot[model.actuator_act_adr[i] + model.actuator_act_num[i] - 1]
            };
            let error_dot = ctrl_dot - data.actuator_velocity[i];
            let integral = if self.ki != 0.0 {
                data.act[model.actuator_act_adr[i]] + error * model.timestep
            } else {
                0.0
            };
            data.actuator_force[i] = self.kp * error + self.kd * error_dot + self.ki * integral;
        }
    }
}

fn install(model: &mut Model, plugin: Arc<dyn Plugin>) {
    model.nplugin = 1;
    model.plugin_capabilities.push(plugin.capabilities());
    model.plugin_needstage.push(PluginStage::Acc);
    model.plugin_objects.push(plugin);
    model.plugin_stateadr.push(0);
    model.plugin_statenum.push(0);
    model.plugin_name.push(None);
    model.plugin_attradr.push(0);
    model.plugin_attrnum.push(0);
}

fn set_callbacks(model: &mut Model, callbacks: &Value) {
    match callbacks["gain"].as_str() {
        Some("later_act_dot") => model.set_act_gain_callback(|_, d, _| 1.0 + d.act_dot[0]),
        Some("earlier_force") => model.set_act_gain_callback(|_, d, _| 1.0 + d.actuator_force[0]),
        Some("qfrc1") => model.set_act_gain_callback(|_, d, _| 1.0 + d.qfrc_actuator[1]),
        None => {}
        Some(other) => panic!("gain callback {other}"),
    }
    match callbacks["bias"].as_str() {
        Some("own_force") => model.set_act_bias_callback(|_, d, i| 0.5 * d.actuator_force[i]),
        None => {}
        Some(other) => panic!("bias callback {other}"),
    }
    match callbacks["dyn"].as_str() {
        Some("earlier_force") => model.set_act_dyn_callback(|m, d, i| {
            let a = m.actuator_act_adr[i] + m.actuator_act_num[i] - 1;
            d.actuator_force[0] + (d.ctrl[i] - d.act[a]) / 0.07
        }),
        Some("write_earlier") => model.set_act_dyn_callback(|m, d, i| {
            let a = m.actuator_act_adr[i] + m.actuator_act_num[i] - 1;
            d.act_dot[0] += 0.5 * d.ctrl[i];
            (d.ctrl[i] - d.act[a]) / 0.07
        }),
        Some("add_qfrc0") => model.set_act_dyn_callback(|m, d, i| {
            let a = m.actuator_act_adr[i] + m.actuator_act_num[i] - 1;
            d.qfrc_actuator[0] += 7.0;
            (d.ctrl[i] - d.act[a]) / 0.07
        }),
        Some("write_later_ctrl") => model.set_act_dyn_callback(|m, d, i| {
            let a = m.actuator_act_adr[i] + m.actuator_act_num[i] - 1;
            d.ctrl[1] += 0.25;
            (d.ctrl[i] - d.act[a]) / 0.07
        }),
        None => {}
        Some(other) => panic!("dynamics callback {other}"),
    }
}

/// The stand-in loaded and changed to MuJoCo's compiled actuators: its user
/// types, activation counts (the addresses then summed as the compilers sum
/// them), plugins and what the case sets after compiling; then every field
/// the cases vary checked against MuJoCo's.
fn patched(case: &Value) -> Model {
    let mut model = sim_mjcf::load_model(case["xml"].as_str().expect("xml")).expect("load");
    let compiled = &case["compiled"];
    let actnum = usizes(&compiled["actnum"]);
    let plugin = ints(&compiled["plugin"]);
    for a in 0..model.nu {
        if compiled["dyntype"][a] == "mjDYN_USER" {
            model.actuator_dyntype[a] = ActuatorDynamics::User;
        }
        if compiled["gaintype"][a] == "mjGAIN_USER" {
            model.actuator_gaintype[a] = GainType::User;
        }
        if compiled["biastype"][a] == "mjBIAS_USER" {
            model.actuator_biastype[a] = BiasType::User;
        }
        model.actuator_plugin[a] = usize::try_from(plugin[a]).ok();
    }
    let mut adr = 0;
    for (a, &n) in actnum.iter().enumerate() {
        model.actuator_act_num[a] = n;
        if n > 0 {
            model.actuator_act_adr[a] = adr;
            adr += n;
        }
    }
    model.na = adr;
    let after = &case["after_compile"];
    for pair in after["actadr"].as_array().into_iter().flatten() {
        model.actuator_act_adr[index(&pair[0])] = index(&pair[1]);
    }
    for pair in after["gear"].as_array().into_iter().flatten() {
        model.actuator_gear[index(&pair[0])][0] = float(&pair[1]);
    }
    if let Some(pid) = case["pid"].as_object() {
        let gain = |key: &str| pid.get(key).map_or(0.0, |v| v.as_f64().expect("gain"));
        install(
            &mut model,
            Arc::new(Pid {
                kp: gain("kp"),
                ki: gain("ki"),
                kd: gain("kd"),
            }),
        );
    }
    set_callbacks(&mut model, &case["callbacks"]);

    let name = case["name"].as_str().expect("name");
    let dyntype: Vec<&str> = model
        .actuator_dyntype
        .iter()
        .map(|t| match t {
            ActuatorDynamics::None => "mjDYN_NONE",
            ActuatorDynamics::Integrator => "mjDYN_INTEGRATOR",
            ActuatorDynamics::Filter => "mjDYN_FILTER",
            ActuatorDynamics::User => "mjDYN_USER",
            other => panic!("dynamics {other:?}"),
        })
        .collect();
    let gaintype: Vec<&str> = model
        .actuator_gaintype
        .iter()
        .map(|t| match t {
            GainType::Fixed => "mjGAIN_FIXED",
            GainType::Affine => "mjGAIN_AFFINE",
            GainType::User => "mjGAIN_USER",
            other => panic!("gain {other:?}"),
        })
        .collect();
    let biastype: Vec<&str> = model
        .actuator_biastype
        .iter()
        .map(|t| match t {
            BiasType::None => "mjBIAS_NONE",
            BiasType::Affine => "mjBIAS_AFFINE",
            BiasType::User => "mjBIAS_USER",
            other => panic!("bias {other:?}"),
        })
        .collect();
    assert_eq!(dyntype, strings(&compiled["dyntype"]), "{name}: dyntype");
    assert_eq!(gaintype, strings(&compiled["gaintype"]), "{name}: gaintype");
    assert_eq!(biastype, strings(&compiled["biastype"]), "{name}: biastype");
    let actadr = ints(&compiled["actadr"]);
    for a in 0..model.nu {
        if actnum[a] > 0 {
            assert_eq!(
                Some(model.actuator_act_adr[a]),
                usize::try_from(actadr[a]).ok(),
                "{name}: actadr {a}"
            );
        }
    }
    assert_eq!(model.na, index(&compiled["na"]), "{name}: na");
    let actearly: Vec<i64> = model
        .actuator_actearly
        .iter()
        .map(|&b| i64::from(b))
        .collect();
    assert_eq!(actearly, ints(&compiled["actearly"]), "{name}: actearly");
    let (limited, range) = (
        ints(&compiled["forcelimited"]),
        floats(&compiled["forcerange"]),
    );
    for a in 0..model.nu {
        let want = if limited[a] == 1 {
            (range[2 * a], range[2 * a + 1])
        } else {
            (f64::NEG_INFINITY, f64::INFINITY)
        };
        assert_eq!(model.actuator_forcerange[a], want, "{name}: forcerange {a}");
    }
    let gear0: Vec<f64> = model.actuator_gear.iter().map(|g| g[0]).collect();
    assert_eq!(gear0, floats(&compiled["gear0"]), "{name}: gear");
    model
}

#[test]
fn actuation_passes_act_as_mujoco() {
    let golden = golden();
    let mut failures = Vec::new();
    let mut compared = 0;
    for case in golden["cases"].as_array().expect("cases") {
        let name = case["name"].as_str().expect("name");
        let model = patched(case);
        let (rownnz, rowadr, colind) = (
            usizes(&case["D_rownnz"]),
            usizes(&case["D_rowadr"]),
            usizes(&case["D_colind"]),
        );
        for (k, state) in case["states"]
            .as_array()
            .expect("states")
            .iter()
            .enumerate()
        {
            let start = || {
                let mut data = model.make_data();
                for (field, key) in [
                    (data.qpos.as_mut_slice(), "qpos"),
                    (data.qvel.as_mut_slice(), "qvel"),
                    (data.ctrl.as_mut_slice(), "ctrl"),
                    (data.act.as_mut_slice(), "act"),
                    (data.act_dot.as_mut_slice(), "act_dot_before"),
                ] {
                    field.copy_from_slice(&floats(&state[key]));
                }
                if let Some(before) = state.get("qfrc_actuator_before") {
                    data.qfrc_actuator
                        .as_mut_slice()
                        .copy_from_slice(&floats(before));
                }
                data
            };
            let mut data = start();
            data.forward(&model).expect("forward");
            if let Some(asleep) = state.get("tree_asleep") {
                let ours: Vec<i64> = data.tree_asleep.iter().map(|&t| i64::from(t)).collect();
                if ours != ints(asleep) {
                    failures.push(format!(
                        "{name}, state {k}: tree_asleep {ours:?}, MuJoCo {asleep}"
                    ));
                }
            }
            let mut compare = |what: &str, ours: &[f64], key: &str| {
                let mj = floats(&state[key]);
                if !matches_golden(ours, &mj) {
                    failures.push(format!("{name}, state {k}: {what} {ours:?}, MuJoCo {mj:?}"));
                }
            };
            compare(
                "actuator_force",
                data.actuator_force.as_slice(),
                "actuator_force",
            );
            compare(
                "qfrc_actuator",
                data.qfrc_actuator.as_slice(),
                "qfrc_actuator",
            );
            compare("act_dot", data.act_dot.as_slice(), "act_dot");
            let mut data = start();
            data.step(&model).expect("step");
            let qderiv: Vec<f64> = (0..model.nv)
                .flat_map(|r| (rowadr[r]..rowadr[r] + rownnz[r]).map(move |j| (r, j)))
                .map(|(r, j)| data.qDeriv[(r, colind[j])])
                .collect();
            compare("qDeriv after the step", &qderiv, "qDeriv");
            compare("qpos after the step", data.qpos.as_slice(), "qpos_after");
            compare("qvel after the step", data.qvel.as_slice(), "qvel_after");
            compare("act after the step", data.act.as_slice(), "act_after");
            compared += 1;
        }
    }
    assert_eq!(compared, 216, "18 cases of 12 states");
    assert!(
        failures.is_empty(),
        "{} differ:\n{}",
        failures.len(),
        failures.join("\n")
    );
}
