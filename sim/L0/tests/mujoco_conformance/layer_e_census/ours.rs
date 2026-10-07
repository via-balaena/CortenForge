//! Our side of one census doc, in the golden's JSON shape.
//!
//! `gen_census_golden.py` writes MuJoCo's side; the two must describe the same
//! quantities under the same names, so a field added here is added there too.
//! Enums are their `Debug` names, which the generator maps MuJoCo's values to;
//! sensor types are the exception, kept in MuJoCo's names on the golden side
//! and brought to one spelling in `compare.rs`.
//! Non-finite numbers are the strings `"inf"`, `"-inf"` and `"nan"`.

use std::panic::{AssertUnwindSafe, catch_unwind};

use nalgebra::{Matrix3, UnitQuaternion, Vector3};
use serde_json::{Map, Value, json};
use sim_core::{Data, Model};

/// Steps per excitation.
pub const NSTEP: usize = 100;
/// Steps after which a `forward` dump is taken (0 is the initial state).
pub const DUMPS: [usize; 3] = [0, 1, NSTEP];
/// Above this many dofs the mass matrix is compared by its diagonal only.
const FULL_QM_MAX_NV: usize = 120;

/// The two initial velocities each doc is run from.
#[derive(Clone, Copy)]
pub enum Excitation {
    /// `qvel[i] = 0.1 (1 + i mod 3)`.
    E1,
    /// `qvel[i] = 0.1 (1 + i mod 5) (-1)^i`.
    E2,
}

impl Excitation {
    pub const ALL: [Self; 2] = [Self::E1, Self::E2];

    /// The key the golden stores this excitation under.
    pub fn key(self) -> &'static str {
        match self {
            Self::E1 => "e1",
            Self::E2 => "e2",
        }
    }

    fn qvel(self, i: usize) -> f64 {
        match self {
            Self::E1 => 0.1 * (1.0 + (i % 3) as f64),
            Self::E2 => 0.1 * (1.0 + (i % 5) as f64) * if i.is_multiple_of(2) { 1.0 } else { -1.0 },
        }
    }
}

fn num(x: f64) -> Value {
    if x.is_nan() {
        json!("nan")
    } else if x.is_infinite() {
        json!(if x > 0.0 { "inf" } else { "-inf" })
    } else {
        json!(x)
    }
}

fn nums(xs: impl IntoIterator<Item = f64>) -> Value {
    Value::Array(xs.into_iter().map(num).collect())
}

fn vec3s<'a>(vs: impl IntoIterator<Item = &'a Vector3<f64>>) -> Value {
    Value::Array(vs.into_iter().map(|v| nums(v.iter().copied())).collect())
}

fn quats<'a>(qs: impl IntoIterator<Item = &'a UnitQuaternion<f64>>) -> Value {
    Value::Array(qs.into_iter().map(|q| nums([q.w, q.i, q.j, q.k])).collect())
}

fn pairs<'a>(ps: impl IntoIterator<Item = &'a (f64, f64)>) -> Value {
    Value::Array(ps.into_iter().map(|&(a, b)| nums([a, b])).collect())
}

fn rows<'a, const N: usize>(rs: impl IntoIterator<Item = &'a [f64; N]>) -> Value {
    Value::Array(rs.into_iter().map(|r| nums(r.iter().copied())).collect())
}

fn names<T: std::fmt::Debug>(xs: impl IntoIterator<Item = T>) -> Value {
    Value::Array(xs.into_iter().map(|x| json!(format!("{x:?}"))).collect())
}

fn flags<'a>(xs: impl IntoIterator<Item = &'a bool>) -> Value {
    Value::Array(xs.into_iter().map(|&b| json!(u8::from(b))).collect())
}

/// An index as MuJoCo stores it: `usize::MAX` (our "none") is -1.
fn id(x: usize) -> i64 {
    i64::try_from(x).unwrap_or(-1)
}

fn ids(xs: impl IntoIterator<Item = usize>) -> Value {
    Value::Array(xs.into_iter().map(|x| json!(id(x))).collect())
}

fn merge(parts: impl IntoIterator<Item = Value>) -> Value {
    let mut out = Map::new();
    for part in parts {
        if let Value::Object(fields) = part {
            out.extend(fields);
        }
    }
    Value::Object(out)
}

/// The model fields the census compares.
pub fn model_json(m: &Model) -> Value {
    let counts = json!({
        "nq": m.nq, "nv": m.nv, "nu": m.nu, "na": m.na, "nbody": m.nbody, "ngeom": m.ngeom,
        "njnt": m.njnt, "neq": m.eq_type.len(), "nsite": m.nsite, "ntendon": m.ntendon,
        "nsensor": m.nsensor, "nsensordata": m.nsensordata, "nflex": m.nflex,
        "nflexvert": m.nflexvert, "nflexedge": m.nflexedge, "nmocap": m.nmocap,
        "qpos0": nums(m.qpos0.iter().copied()), "qpos_spring": nums(m.qpos_spring.iter().copied()),
    });
    let options = json!({
        "timestep": num(m.timestep), "gravity": nums(m.gravity.iter().copied()),
        "integrator": format!("{:?}", m.integrator), "solver": format!("{:?}", m.solver_type),
        "iterations": m.solver_iterations, "tolerance": num(m.solver_tolerance), "cone": m.cone,
        "impratio": num(m.impratio), "disableflags": m.disableflags, "enableflags": m.enableflags,
        "ls_iterations": m.ls_iterations, "ls_tolerance": num(m.ls_tolerance),
        "noslip_iterations": m.noslip_iterations, "noslip_tolerance": num(m.noslip_tolerance),
        "density": num(m.density), "viscosity": num(m.viscosity),
        "wind": nums(m.wind.iter().copied()), "magnetic": nums(m.magnetic.iter().copied()),
        "o_margin": num(m.o_margin), "o_solref": nums(m.o_solref), "o_solimp": nums(m.o_solimp),
        "o_friction": nums(m.o_friction), "ccd_iterations": m.ccd_iterations,
        "ccd_tolerance": num(m.ccd_tolerance), "sleep_tolerance": num(m.sleep_tolerance),
        "meaninertia": num(m.stat_meaninertia),
    });
    // The full inertia tensor R diag(I) Rᵀ, row-major: invariant to how the
    // principal axes are ordered, which the two compilers choose differently.
    let body_full_inertia = (0..m.nbody).map(|b| {
        let r = m.body_iquat[b].to_rotation_matrix();
        let t = r.matrix() * Matrix3::from_diagonal(&m.body_inertia[b]) * r.matrix().transpose();
        nums(t.transpose().iter().copied())
    });
    let bodies = json!({
        "body_parent": m.body_parent, "body_pos": vec3s(&m.body_pos),
        "body_quat": quats(&m.body_quat), "body_ipos": vec3s(&m.body_ipos),
        "body_iquat": quats(&m.body_iquat), "body_mass": nums(m.body_mass.iter().copied()),
        "body_inertia": vec3s(&m.body_inertia), "body_gravcomp": nums(m.body_gravcomp.iter().copied()),
        "body_mocapid": m.body_mocapid.iter().map(|x| x.map_or(-1, id)).collect::<Vec<_>>(),
        "body_I": body_full_inertia.collect::<Vec<_>>(),
    });
    let joints = json!({
        "jnt_type": names(&m.jnt_type), "jnt_body": m.jnt_body, "jnt_qposadr": m.jnt_qpos_adr,
        "jnt_dofadr": m.jnt_dof_adr, "jnt_pos": vec3s(&m.jnt_pos), "jnt_axis": vec3s(&m.jnt_axis),
        "jnt_limited": flags(&m.jnt_limited), "jnt_range": pairs(&m.jnt_range),
        "jnt_stiffness": nums(m.jnt_stiffness.iter().copied()),
        "jnt_margin": nums(m.jnt_margin.iter().copied()), "jnt_solref": rows(&m.jnt_solref),
        "jnt_solimp": rows(&m.jnt_solimp), "jnt_actgravcomp": flags(&m.jnt_actgravcomp),
        "dof_body": m.dof_body, "dof_damping": nums(m.dof_damping.iter().copied()),
        "dof_armature": nums(m.dof_armature.iter().copied()),
        "dof_frictionloss": nums(m.dof_frictionloss.iter().copied()),
        "dof_solref": rows(&m.dof_solref), "dof_solimp": rows(&m.dof_solimp),
    });
    let geoms = json!({
        "geom_type": names(&m.geom_type), "geom_body": m.geom_body, "geom_pos": vec3s(&m.geom_pos),
        "geom_quat": quats(&m.geom_quat), "geom_size": vec3s(&m.geom_size),
        "geom_friction": vec3s(&m.geom_friction), "geom_condim": m.geom_condim,
        "geom_contype": m.geom_contype, "geom_conaffinity": m.geom_conaffinity,
        "geom_margin": nums(m.geom_margin.iter().copied()), "geom_gap": nums(m.geom_gap.iter().copied()),
        "geom_priority": m.geom_priority, "geom_solmix": nums(m.geom_solmix.iter().copied()),
        "geom_solref": rows(&m.geom_solref), "geom_solimp": rows(&m.geom_solimp),
    });
    let actuators = json!({
        "actuator_trntype": names(&m.actuator_trntype), "actuator_dyntype": names(&m.actuator_dyntype),
        "actuator_gaintype": names(&m.actuator_gaintype), "actuator_biastype": names(&m.actuator_biastype),
        "actuator_trnid": ids(m.actuator_trnid.iter().map(|t| t[0])),
        "actuator_gear": rows(&m.actuator_gear), "actuator_ctrlrange": pairs(&m.actuator_ctrlrange),
        "actuator_forcerange": pairs(&m.actuator_forcerange),
        "actuator_actlimited": flags(&m.actuator_actlimited), "actuator_actrange": pairs(&m.actuator_actrange),
        "actuator_actearly": flags(&m.actuator_actearly), "actuator_dynprm": rows(&m.actuator_dynprm),
        "actuator_gainprm": rows(&m.actuator_gainprm), "actuator_biasprm": rows(&m.actuator_biasprm),
        "actuator_lengthrange": pairs(&m.actuator_lengthrange),
        "actuator_acc0": nums(m.actuator_acc0.iter().copied()), "actuator_actnum": m.actuator_act_num,
        "actuator_delay": nums(m.actuator_delay.iter().copied()), "actuator_nsample": m.actuator_nsample,
    });
    let tendons = json!({
        "tendon_limited": flags(&m.tendon_limited),
        "tendon_range": pairs(&m.tendon_range), "tendon_stiffness": nums(m.tendon_stiffness.iter().copied()),
        "tendon_damping": nums(m.tendon_damping.iter().copied()),
        "tendon_lengthspring": rows(&m.tendon_lengthspring),
        "tendon_frictionloss": nums(m.tendon_frictionloss.iter().copied()),
        "tendon_margin": nums(m.tendon_margin.iter().copied()), "tendon_num": m.tendon_num,
        "tendon_length0": nums(m.tendon_length0.iter().copied()),
        "tendon_solref_lim": rows(&m.tendon_solref_lim), "tendon_solimp_lim": rows(&m.tendon_solimp_lim),
    });
    let equalities = json!({
        "eq_type": names(&m.eq_type), "eq_obj1id": ids(m.eq_obj1id.iter().copied()),
        "eq_obj2id": ids(m.eq_obj2id.iter().copied()), "eq_data": rows(&m.eq_data),
        "eq_solref": rows(&m.eq_solref), "eq_solimp": rows(&m.eq_solimp), "eq_active": flags(&m.eq_active),
    });
    let sensors_and_flex = json!({
        "sensor_type": names(&m.sensor_type), "sensor_dim": m.sensor_dim, "sensor_adr": m.sensor_adr,
        "sensor_objid": ids(m.sensor_objid.iter().copied()),
        "sensor_cutoff": nums(m.sensor_cutoff.iter().copied()),
        "sensor_noise": nums(m.sensor_noise.iter().copied()),
        "sensor_delay": nums(m.sensor_delay.iter().copied()), "sensor_nsample": m.sensor_nsample,
        "flex_dim": m.flex_dim, "flex_vertnum": m.flex_vertnum, "flex_edgenum": m.flex_edgenum,
        "flex_elemnum": m.flex_elemnum,
        "flexvert_bodyid": m.flexvert_bodyid,
    });
    merge([
        counts,
        options,
        bodies,
        joints,
        geoms,
        actuators,
        tendons,
        equalities,
        sensors_and_flex,
    ])
}

/// The `forward` quantities the census compares, in MuJoCo's pipeline order.
fn data_json(m: &Model, d: &Data) -> Value {
    let nv = m.nv;
    let (qm_full, qm) = if nv <= FULL_QM_MAX_NV {
        (
            1,
            nums((0..nv).flat_map(|i| (0..nv).map(move |j| d.qM[(i, j)]))),
        )
    } else {
        (0, nums((0..nv).map(|i| d.qM[(i, i)])))
    };
    let ncon = d.ncon.min(d.contacts.len());
    let contacts: Vec<Value> = d.contacts[..ncon]
        .iter()
        .map(|c| {
            let vertex = |v: Option<usize>| v.map_or(-1, id);
            json!([
                id(c.geom1),
                id(c.geom2),
                vertex(c.flex_vertex),
                vertex(c.flex_vertex2),
                num(-c.depth),
                nums(c.pos.iter().copied()),
                nums(c.normal.iter().copied()),
                nums(c.frame[0].iter().copied()),
                nums(c.frame[1].iter().copied()),
            ])
        })
        .collect();
    let mut efc_counts = Map::new();
    for t in &d.efc_type {
        let n = efc_counts.entry(format!("{t:?}")).or_insert(json!(0));
        *n = json!(n.as_u64().unwrap_or(0) + 1);
    }
    json!({
        "time": num(d.time), "qpos": nums(d.qpos.iter().copied()), "qvel": nums(d.qvel.iter().copied()),
        "act": nums(d.act.iter().copied()), "xpos": vec3s(&d.xpos), "xquat": quats(&d.xquat),
        "qM_full": qm_full, "qM": qm, "ncon": ncon, "con": contacts,
        "nefc": d.efc_type.len(), "efc_counts": efc_counts,
        "qfrc_bias": nums(d.qfrc_bias.iter().copied()),
        "qfrc_passive": nums(d.qfrc_passive.iter().copied()),
        "qfrc_actuator": nums(d.qfrc_actuator.iter().copied()),
        "qacc_smooth": nums(d.qacc_smooth.iter().copied()),
        "qfrc_constraint": nums(d.qfrc_constraint.iter().copied()),
        "qacc": nums(d.qacc.iter().copied()), "sensordata": nums(d.sensordata.iter().copied()),
        "warn": d.warnings.iter().map(|w| w.count).collect::<Vec<_>>(),
    })
}

fn panic_text(payload: &(dyn std::any::Any + Send)) -> String {
    payload
        .downcast_ref::<&str>()
        .map(|s| (*s).to_string())
        .or_else(|| payload.downcast_ref::<String>().cloned())
        .unwrap_or_else(|| "panic".to_string())
}

/// A `forward` on a copy, so the dump never touches the trajectory's state.
fn forward_dump(m: &Model, d: &Data) -> Value {
    let mut copy = d.clone();
    match catch_unwind(AssertUnwindSafe(|| copy.forward(m))) {
        Err(p) => json!({ "err": format!("panic: {}", panic_text(p.as_ref())) }),
        Ok(Err(e)) => json!({ "err": format!("{e:?}") }),
        Ok(Ok(())) => data_json(m, &copy),
    }
}

fn initial_state(m: &Model, excitation: Excitation) -> Data {
    let mut d = m.make_data();
    d.qpos.copy_from(&m.qpos0);
    for i in 0..m.nv {
        d.qvel[i] = excitation.qvel(i);
    }
    for (ctrl, &(lo, hi)) in d.ctrl.iter_mut().zip(&m.actuator_ctrlrange) {
        *ctrl = if lo.is_finite() && hi.is_finite() {
            lo + 0.625 * (hi - lo)
        } else {
            0.0
        };
    }
    d
}

fn run_excitation(m: &Model, excitation: Excitation) -> Value {
    let mut d = initial_state(m, excitation);
    let mut dumps = Map::new();
    dumps.insert("0".to_string(), forward_dump(m, &d));
    let (mut q, mut v, mut a, mut t) = (Vec::new(), Vec::new(), Vec::new(), Vec::new());
    let mut step_err = Value::Null;
    for k in 1..=NSTEP {
        let err = match catch_unwind(AssertUnwindSafe(|| d.step(m))) {
            Err(p) => Some(format!("panic: {}", panic_text(p.as_ref()))),
            Ok(Err(e)) => Some(format!("{e:?}")),
            Ok(Ok(())) => None,
        };
        if let Some(err) = err {
            step_err = json!({ "k": k, "err": err });
            break;
        }
        q.push(nums(d.qpos.iter().copied()));
        v.push(nums(d.qvel.iter().copied()));
        a.push(nums(d.act.iter().copied()));
        t.push(num(d.time));
        if DUMPS.contains(&k) {
            dumps.insert(k.to_string(), forward_dump(m, &d));
        }
    }
    json!({ "dump": dumps, "step_err": step_err, "traj": { "q": q, "v": v, "a": a, "t": t } })
}

/// Our record for one doc: its load status, and when it loads its model and
/// both excitations (every step's state, so any golden checkpoint can be read).
pub fn record(xml: &str) -> Value {
    let model = match catch_unwind(AssertUnwindSafe(|| sim_mjcf::load_model(xml))) {
        Err(p) => return json!({ "status": "panic", "err": panic_text(p.as_ref()) }),
        Ok(Err(e)) => return json!({ "status": "refused", "err": e.to_string() }),
        Ok(Ok(m)) => m,
    };
    if let Err(e) = model.try_make_data() {
        return json!({ "status": "refused", "err": e.to_string() });
    }
    let mut out = Map::new();
    out.insert("status".to_string(), json!("ok"));
    out.insert("model".to_string(), model_json(&model));
    for excitation in Excitation::ALL {
        out.insert(
            excitation.key().to_string(),
            run_excitation(&model, excitation),
        );
    }
    Value::Object(out)
}
