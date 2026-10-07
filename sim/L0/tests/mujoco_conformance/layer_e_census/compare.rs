//! The census comparison: which quantities of our record differ from MuJoCo's.
//!
//! A float quantity differs when `max|ours − mj| / max(1, max|mj|)` exceeds
//! [`TOLERANCE`]; contacts and quaternions are compared on absolute
//! differences. A shape mismatch, a finite value where the other side is not,
//! or two different non-finite values always differ, and NaN equals NaN at
//! the same position. Model fields are compared only where they act (a
//! joint's range only when it is limited, a geom's solver parameters only when
//! it collides, and so on).

use std::cmp::Ordering;
use std::collections::BTreeMap;

use serde_json::Value;

/// The census tolerance. Each doc's verdict rests on its largest difference,
/// and those leave the decade below and above this value nearly empty
/// (A20 §2.3 of the Rigid spec book).
pub const TOLERANCE: f64 = 1e-9;

/// Contacts closer to zero distance than this are compared only by count.
const ZERO_DISTANCE: f64 = 1e-12;

fn num(v: &Value) -> f64 {
    match v {
        Value::Number(n) => n.as_f64().unwrap_or(f64::NAN),
        Value::String(s) => match s.as_str() {
            "inf" => f64::INFINITY,
            "-inf" => f64::NEG_INFINITY,
            _ => f64::NAN,
        },
        Value::Bool(b) => f64::from(u8::from(*b)),
        _ => f64::NAN,
    }
}

/// The shape and the flattened numbers of a nested array.
fn flatten(v: &Value) -> (Vec<usize>, Vec<f64>) {
    fn shape(v: &Value, out: &mut Vec<usize>) {
        if let Value::Array(a) = v {
            out.push(a.len());
            if let Some(first) = a.first() {
                shape(first, out);
            }
        }
    }
    fn walk(v: &Value, out: &mut Vec<f64>) {
        match v {
            Value::Array(a) => a.iter().for_each(|x| walk(x, out)),
            x => out.push(num(x)),
        }
    }
    let (mut s, mut d) = (Vec::new(), Vec::new());
    shape(v, &mut s);
    walk(v, &mut d);
    (s, d)
}

/// `|x − y|`, except that a finite value against a non-finite one, or two
/// different non-finite values, are infinitely far apart, and NaN equals NaN.
fn diff(x: f64, y: f64) -> f64 {
    if x.is_finite() && y.is_finite() {
        (x - y).abs()
    } else if (x.is_nan() && y.is_nan()) || x == y {
        0.0
    } else {
        f64::INFINITY
    }
}

/// The scaled difference of two quantities.
fn scaled(ours: &Value, mj: &Value) -> f64 {
    let (so, o) = flatten(ours);
    let (sm, m) = flatten(mj);
    if so != sm || o.len() != m.len() {
        return f64::INFINITY;
    }
    let (mut raw, mut scale) = (0.0_f64, 0.0_f64);
    for (x, y) in o.iter().zip(&m) {
        if x.is_finite() != y.is_finite() {
            return f64::INFINITY;
        }
        if x.is_finite() {
            raw = raw.max((x - y).abs());
            scale = scale.max(y.abs());
        } else if !(x.is_nan() && y.is_nan()) && x != y {
            return f64::INFINITY;
        }
    }
    raw / scale.max(1.0)
}

/// The largest difference between two lists of quaternions, each up to sign.
fn quat_scaled(ours: &Value, mj: &Value) -> f64 {
    let (_, o) = flatten(ours);
    let (_, m) = flatten(mj);
    if o.len() != m.len() || o.len() % 4 != 0 {
        return f64::INFINITY;
    }
    o.chunks(4).zip(m.chunks(4)).fold(0.0, |worst, (a, b)| {
        let minus = a
            .iter()
            .zip(b)
            .map(|(&x, &y)| diff(x, y))
            .fold(0.0, f64::max);
        let plus = a
            .iter()
            .zip(b)
            .map(|(&x, &y)| diff(x, -y))
            .fold(0.0, f64::max);
        worst.max(minus.min(plus))
    })
}

fn arr(v: &Value) -> &[Value] {
    v.as_array().map_or(&[], Vec::as_slice)
}

fn pick(v: &Value, idx: &[usize]) -> Value {
    let a = arr(v);
    Value::Array(
        idx.iter()
            .map(|&i| a.get(i).cloned().unwrap_or(Value::Null))
            .collect(),
    )
}

fn str_at(v: &Value, i: usize) -> &str {
    arr(v).get(i).and_then(Value::as_str).unwrap_or("")
}

fn int_at(v: &Value, i: usize) -> i64 {
    arr(v).get(i).and_then(Value::as_i64).unwrap_or(0)
}

/// The first `n` entries of row `i` of a list of rows.
fn row_head(v: &Value, i: usize, n: usize) -> Value {
    Value::Array(
        arr(arr(v).get(i).unwrap_or(&Value::Null))
            .iter()
            .take(n)
            .cloned()
            .collect(),
    )
}

/// Counts whose mismatch makes the rest of the model incomparable: the arrays
/// they size cannot be matched entry by entry.
pub const COUNT_FIELDS: &[&str] = &[
    "nq",
    "nv",
    "nu",
    "na",
    "nbody",
    "ngeom",
    "njnt",
    "neq",
    "nsite",
    "ntendon",
    "nsensor",
    "nsensordata",
    "nflex",
    "nflexvert",
    "nflexedge",
    "nmocap",
];

/// Options compared exactly.
const OPTION_EXACT: &[&str] = &[
    "integrator",
    "solver",
    "iterations",
    "cone",
    "disableflags",
    "enableflags",
    "ls_iterations",
    "noslip_iterations",
    "ccd_iterations",
];

const OPTION_FLOATS: &[&str] = &[
    "timestep",
    "gravity",
    "tolerance",
    "impratio",
    "ls_tolerance",
    "noslip_tolerance",
    "density",
    "viscosity",
    "wind",
    "magnetic",
    "o_margin",
    "o_solref",
    "o_solimp",
    "o_friction",
    "ccd_tolerance",
    "sleep_tolerance",
    "meaninertia",
];

/// MuJoCo's sensor names and our `Debug` names, brought to one spelling.
fn sensor_name(t: &str) -> String {
    let t = t.replace('_', "").to_lowercase();
    match t.as_str() {
        "jointactuatorfrc" => "jointactfrc".to_string(),
        "tendonactuatorfrc" => "tendonactfrc".to_string(),
        _ => t,
    }
}

/// Collects the model fields that differ, in the census's order.
struct ModelDiff<'a> {
    ours: &'a Value,
    mj: &'a Value,
    fields: Vec<String>,
}

impl ModelDiff<'_> {
    fn exact(&mut self, f: &str) {
        if self.ours[f] != self.mj[f] {
            self.fields.push(f.to_string());
        }
    }

    fn close(&mut self, f: &str) {
        self.close_values(f, &self.ours[f], &self.mj[f]);
    }

    fn close_values(&mut self, f: &str, ours: &Value, mj: &Value) {
        if scaled(ours, mj) > TOLERANCE {
            self.fields.push(f.to_string());
        }
    }

    fn exact_at(&mut self, f: &str, idx: &[usize]) {
        if pick(&self.ours[f], idx) != pick(&self.mj[f], idx) {
            self.fields.push(f.to_string());
        }
    }

    fn close_at(&mut self, f: &str, idx: &[usize]) {
        self.close_values(f, &pick(&self.ours[f], idx), &pick(&self.mj[f], idx));
    }

    fn quat(&mut self, f: &str) {
        if quat_scaled(&self.ours[f], &self.mj[f]) > TOLERANCE {
            self.fields.push(f.to_string());
        }
    }

    /// Row by row, the first `len(i)` entries of each; the field is reported once.
    fn close_rows(&mut self, f: &str, n: usize, len: impl Fn(usize) -> usize) {
        let differs = (0..n).any(|i| {
            scaled(
                &row_head(&self.ours[f], i, len(i)),
                &row_head(&self.mj[f], i, len(i)),
            ) > TOLERANCE
        });
        if differs {
            self.fields.push(f.to_string());
        }
    }

    /// Indices `0..n` where `keep` holds of MuJoCo's model.
    fn where_mj(&self, n: usize, keep: impl Fn(&Value, usize) -> bool) -> Vec<usize> {
        (0..n).filter(|&i| keep(self.mj, i)).collect()
    }
}

/// The model fields that differ, first first.
pub fn model_diff(ours: &Value, mj: &Value) -> Vec<String> {
    let mut d = ModelDiff {
        ours,
        mj,
        fields: Vec::new(),
    };
    for f in COUNT_FIELDS {
        d.exact(f);
    }
    let counts_agree = d.fields.is_empty();
    for f in OPTION_EXACT {
        d.exact(f);
    }
    for f in OPTION_FLOATS {
        d.close(f);
    }
    if !counts_agree {
        return d.fields;
    }
    let count = |f: &str| {
        mj[f]
            .as_u64()
            .and_then(|n| usize::try_from(n).ok())
            .unwrap_or(0)
    };
    let (nbody, njnt, ngeom, nu) = (count("nbody"), count("njnt"), count("ngeom"), count("nu"));
    let (ntendon, neq, nsensor) = (count("ntendon"), count("neq"), count("nsensor"));

    d.close("qpos0");
    d.close("qpos_spring");
    d.exact("body_parent");
    d.exact("body_mocapid");
    d.close("body_pos");
    d.quat("body_quat");
    d.close("body_mass");
    let mass = |v: &Value, b: usize| arr(&v["body_mass"]).get(b).map_or(0.0, num);
    let massive: Vec<usize> = (0..nbody)
        .filter(|&b| mass(mj, b) > 1e-15 || mass(ours, b) > 1e-15)
        .collect();
    d.close_at("body_ipos", &massive);
    d.close("body_I");
    d.close("body_gravcomp");

    for f in ["jnt_type", "jnt_body", "jnt_qposadr", "jnt_dofadr"] {
        d.exact(f);
    }
    let hinge_slide = d.where_mj(njnt, |m, j| {
        matches!(str_at(&m["jnt_type"], j), "Hinge" | "Slide")
    });
    let with_anchor = d.where_mj(njnt, |m, j| {
        matches!(str_at(&m["jnt_type"], j), "Hinge" | "Slide" | "Ball")
    });
    d.close_at("jnt_pos", &with_anchor);
    d.close_at("jnt_axis", &hinge_slide);
    d.exact("jnt_limited");
    let limited = d.where_mj(njnt, |m, j| int_at(&m["jnt_limited"], j) != 0);
    for f in ["jnt_range", "jnt_margin", "jnt_solref", "jnt_solimp"] {
        d.close_at(f, &limited);
    }
    d.close("jnt_stiffness");
    d.exact("jnt_actgravcomp");

    d.exact("dof_body");
    for f in ["dof_damping", "dof_armature", "dof_frictionloss"] {
        d.close(f);
    }
    let ndof = arr(&mj["dof_frictionloss"]).len();
    let frictional = d.where_mj(ndof, |m, i| num(&arr(&m["dof_frictionloss"])[i]) > 0.0);
    d.close_at("dof_solref", &frictional);
    d.close_at("dof_solimp", &frictional);

    d.exact("geom_type");
    d.exact("geom_body");
    d.close("geom_pos");
    d.quat("geom_quat");
    d.close_rows("geom_size", ngeom, |g| match str_at(&mj["geom_type"], g) {
        "Sphere" => 1,
        "Capsule" | "Cylinder" => 2,
        "Plane" | "Mesh" | "Hfield" | "Sdf" => 0,
        _ => 3,
    });
    let colliding = d.where_mj(ngeom, |m, g| {
        int_at(&m["geom_contype"], g) != 0 || int_at(&m["geom_conaffinity"], g) != 0
    });
    d.exact("geom_contype");
    d.exact("geom_conaffinity");
    for f in ["geom_condim", "geom_priority"] {
        d.exact_at(f, &colliding);
    }
    for f in [
        "geom_friction",
        "geom_margin",
        "geom_gap",
        "geom_solmix",
        "geom_solref",
        "geom_solimp",
    ] {
        d.close_at(f, &colliding);
    }

    for f in [
        "actuator_trntype",
        "actuator_dyntype",
        "actuator_gaintype",
        "actuator_biastype",
        "actuator_trnid",
        "actuator_actlimited",
        "actuator_actearly",
        "actuator_actnum",
        "actuator_nsample",
    ] {
        d.exact(f);
    }
    for f in [
        "actuator_gear",
        "actuator_ctrlrange",
        "actuator_forcerange",
        "actuator_delay",
    ] {
        d.close(f);
    }
    let stateful = d.where_mj(nu, |m, i| str_at(&m["actuator_dyntype"], i) != "None");
    d.close_at("actuator_dynprm", &stateful);
    let params_used = |kind: &str| match kind {
        "Fixed" => 1,
        "Affine" => 3,
        "None" => 0,
        _ => 9,
    };
    d.close_rows("actuator_gainprm", nu, |i| {
        params_used(str_at(&mj["actuator_gaintype"], i))
    });
    d.close_rows("actuator_biasprm", nu, |i| {
        params_used(str_at(&mj["actuator_biastype"], i))
    });
    let act_limited = d.where_mj(nu, |m, i| int_at(&m["actuator_actlimited"], i) != 0);
    d.close_at("actuator_actrange", &act_limited);
    let muscles = d.where_mj(nu, |m, i| {
        str_at(&m["actuator_gaintype"], i) == "Muscle"
            || str_at(&m["actuator_biastype"], i) == "Muscle"
    });
    d.close_at("actuator_lengthrange", &muscles);
    d.close_at("actuator_acc0", &muscles);

    d.exact("tendon_limited");
    d.exact("tendon_num");
    let tendon_limited = d.where_mj(ntendon, |m, i| int_at(&m["tendon_limited"], i) != 0);
    for f in [
        "tendon_range",
        "tendon_margin",
        "tendon_solref_lim",
        "tendon_solimp_lim",
    ] {
        d.close_at(f, &tendon_limited);
    }
    for f in [
        "tendon_stiffness",
        "tendon_damping",
        "tendon_lengthspring",
        "tendon_frictionloss",
        "tendon_length0",
    ] {
        d.close(f);
    }

    for f in ["eq_type", "eq_obj1id", "eq_obj2id", "eq_active"] {
        d.exact(f);
    }
    // A weld's data is 11 numbers on both sides, but ours never writes the last
    // (MuJoCo's torque scale), so ours is compared with that number taken as 1.
    let eq_data_differs = (0..neq).any(|e| match str_at(&mj["eq_type"], e) {
        "Weld" => {
            let mut o = arr(&row_head(&ours["eq_data"], e, 10)).to_vec();
            o.push(Value::from(1.0));
            scaled(&Value::Array(o), &row_head(&mj["eq_data"], e, 11)) > TOLERANCE
        }
        kind => {
            let n = match kind {
                "Connect" => 6,
                "Joint" | "Tendon" => 5,
                "Distance" => 1,
                "Flex" | "FlexVert" => 0,
                _ => 11,
            };
            scaled(
                &row_head(&ours["eq_data"], e, n),
                &row_head(&mj["eq_data"], e, n),
            ) > TOLERANCE
        }
    });
    if eq_data_differs {
        d.fields.push("eq_data".to_string());
    }
    d.close("eq_solref");
    d.close("eq_solimp");

    let sensor_names = |v: &Value| {
        arr(&v["sensor_type"])
            .iter()
            .map(|t| sensor_name(t.as_str().unwrap_or("")))
            .collect::<Vec<_>>()
    };
    if sensor_names(ours) != sensor_names(mj) {
        d.fields.push("sensor_type".to_string());
    }
    d.exact("sensor_dim");
    d.exact("sensor_adr");
    let attached = d.where_mj(nsensor, |m, i| int_at(&m["sensor_objid"], i) >= 0);
    d.exact_at("sensor_objid", &attached);
    d.exact("sensor_nsample");
    for f in ["sensor_cutoff", "sensor_delay", "sensor_noise"] {
        d.close(f);
    }
    for f in [
        "flex_dim",
        "flex_vertnum",
        "flex_edgenum",
        "flex_elemnum",
        "flexvert_bodyid",
    ] {
        d.exact(f);
    }
    d.fields
}

/// The `forward` quantities in MuJoCo's pipeline order; the first that
/// differs names a dump's verdict. `con_zero` (the count of zero-distance
/// contacts) is checked last.
const PIPELINE_ORDER: &[&str] = &[
    "time",
    "qpos",
    "qvel",
    "act",
    "xpos",
    "xquat",
    "qM",
    "ncon",
    "con_pairs",
    "con_dist",
    "con_pos",
    "con_normal",
    "con_frame",
    "nefc",
    "efc_counts",
    "qfrc_bias",
    "qfrc_passive",
    "qfrc_actuator",
    "qacc_smooth",
    "qfrc_constraint",
    "qacc",
    "sensordata",
    "con_zero",
];

/// A contact's two objects, sorted, so the pair matches whichever side lists
/// it first; and whether sorting swapped them, and whether one is a flex vertex.
type PairKey = Vec<(char, i64)>;

fn pair_key(c: &[Value], ours: bool) -> (PairKey, bool, bool) {
    let at = |i: usize| c.get(i).and_then(Value::as_i64).unwrap_or(-1);
    let (g1, g2, v1, v2) = (at(0), at(1), at(2), at(3));
    let objects = if ours {
        // Ours names a flex vertex in place of a geom.
        if v1 >= 0 && v2 >= 0 {
            vec![('v', v1), ('v', v2)]
        } else if v1 >= 0 {
            vec![('v', v1), ('g', g1)]
        } else {
            vec![('g', g1), ('g', g2)]
        }
    } else {
        let side = |g: i64, v: i64| if g >= 0 { ('g', g) } else { ('v', v) };
        vec![side(g1, v1), side(g2, v2)]
    };
    let mut key = objects.clone();
    key.sort_unstable();
    let swapped = key != objects;
    let flex = objects.iter().any(|o| o.0 == 'v');
    (key, swapped, flex)
}

fn vec3(v: &Value) -> [f64; 3] {
    let a = arr(v);
    let at = |i: usize| a.get(i).map_or(f64::NAN, num);
    [at(0), at(1), at(2)]
}

fn max_abs_diff(a: [f64; 3], b: [f64; 3], sign: f64) -> f64 {
    (0..3).map(|i| diff(a[i], sign * b[i])).fold(0.0, f64::max)
}

type Grouped<'a> = BTreeMap<PairKey, Vec<(&'a [Value], bool, bool)>>;

fn group<'a>(contacts: &[&'a [Value]], ours: bool) -> Grouped<'a> {
    let mut g: Grouped<'a> = BTreeMap::new();
    for c in contacts {
        let (key, swapped, flex) = pair_key(c, ours);
        g.entry(key).or_default().push((c, swapped, flex));
    }
    g
}

/// Contacts as multisets keyed by object pair: within a pair each of
/// MuJoCo's contacts is matched to our nearest one by position. A normal is
/// compared after undoing a swap (and up to sign against a flex vertex); a
/// tangent up to sign.
fn contact_diffs(ours: &[&[Value]], mj: &[&[Value]], out: &mut Vec<(&'static str, f64)>) {
    let (go, gm) = (group(ours, true), group(mj, false));
    let counts = |g: &Grouped<'_>| {
        g.iter()
            .map(|(k, v)| (k.clone(), v.len()))
            .collect::<Vec<_>>()
    };
    if counts(&go) != counts(&gm) {
        out.push(("con_pairs", f64::INFINITY));
        return;
    }
    let (mut dist, mut pos, mut normal, mut frame) = (0.0_f64, 0.0_f64, 0.0_f64, 0.0_f64);
    for (key, mine) in &gm {
        let mut candidates = go[key].clone();
        for &(cm, swapped_m, flex_m) in mine {
            let pm = vec3(&cm[5]);
            let nearest = (0..candidates.len())
                .min_by(|&x, &y| {
                    let dx = max_abs_diff(vec3(&candidates[x].0[5]), pm, 1.0);
                    let dy = max_abs_diff(vec3(&candidates[y].0[5]), pm, 1.0);
                    dx.partial_cmp(&dy).unwrap_or(Ordering::Equal)
                })
                .unwrap_or(0);
            let (co, swapped_o, _) = candidates.remove(nearest);
            dist = dist.max(diff(num(&co[4]), num(&cm[4])));
            pos = pos.max(max_abs_diff(vec3(&co[5]), pm, 1.0));
            let flip = |swapped: bool| if swapped { -1.0 } else { 1.0 };
            let no = vec3(&co[6]).map(|x| x * flip(swapped_o));
            let nm = vec3(&cm[6]).map(|x| x * flip(swapped_m));
            let dn = if flex_m {
                max_abs_diff(no, nm, 1.0).min(max_abs_diff(no, nm, -1.0))
            } else {
                max_abs_diff(no, nm, 1.0)
            };
            normal = normal.max(dn);
            for k in [7, 8] {
                let (to, tm) = (vec3(&co[k]), vec3(&cm[k]));
                frame = frame.max(max_abs_diff(to, tm, 1.0).min(max_abs_diff(to, tm, -1.0)));
            }
        }
    }
    out.extend([
        ("con_dist", dist),
        ("con_pos", pos),
        ("con_normal", normal),
        ("con_frame", frame),
    ]);
}

/// Constraint-row counts with a flex edge counted as an equality (ours keeps
/// a separate kind). The golden already merges MuJoCo's two friction-loss kinds.
fn row_counts(c: &Value) -> BTreeMap<String, i64> {
    let mut out = BTreeMap::new();
    if let Value::Object(o) = c {
        for (k, v) in o {
            let kind = if k == "FlexEdge" {
                "Equality"
            } else {
                k.as_str()
            };
            *out.entry(kind.to_string()).or_insert(0) += v.as_i64().unwrap_or(0);
        }
    }
    out
}

fn nonzero_contacts(dump: &Value) -> Vec<&[Value]> {
    arr(&dump["con"])
        .iter()
        .filter_map(Value::as_array)
        .map(Vec::as_slice)
        .filter(|c| {
            // A NaN distance is not zero: it is compared, and differs.
            let d = c.get(4).map_or(0.0, num).abs();
            d > ZERO_DISTANCE || d.is_nan()
        })
        .collect()
}

/// The quantities of a `forward` dump that differ, in pipeline order.
pub fn dump_diff(ours: &Value, mj: &Value) -> Vec<&'static str> {
    if ours.get("err").is_some() {
        return vec!["ours_err"];
    }
    let mut q: Vec<(&'static str, f64)> = Vec::new();
    for f in ["time", "qpos", "qvel", "act", "xpos"] {
        q.push((f, scaled(&ours[f], &mj[f])));
    }
    q.push(("xquat", quat_scaled(&ours["xquat"], &mj["xquat"])));
    let qm = if ours["qM_full"] == mj["qM_full"] {
        scaled(&ours["qM"], &mj["qM"])
    } else {
        f64::INFINITY
    };
    q.push(("qM", qm));
    let (co, cm) = (nonzero_contacts(ours), nonzero_contacts(mj));
    let zero_o = arr(&ours["con"]).len() - co.len();
    let zero_m = arr(&mj["con"]).len() - cm.len();
    q.push((
        "con_zero",
        if zero_o == zero_m { 0.0 } else { f64::INFINITY },
    ));
    if co.len() == cm.len() {
        q.push(("ncon", 0.0));
        contact_diffs(&co, &cm, &mut q);
    } else {
        q.push(("ncon", f64::INFINITY));
    }
    q.push((
        "nefc",
        if ours["nefc"] == mj["nefc"] {
            0.0
        } else {
            f64::INFINITY
        },
    ));
    let rows_agree = row_counts(&ours["efc_counts"]) == row_counts(&mj["efc_counts"]);
    q.push(("efc_counts", if rows_agree { 0.0 } else { f64::INFINITY }));
    for f in [
        "qfrc_bias",
        "qfrc_passive",
        "qfrc_actuator",
        "qacc_smooth",
        "qfrc_constraint",
        "qacc",
        "sensordata",
    ] {
        q.push((f, scaled(&ours[f], &mj[f])));
    }
    PIPELINE_ORDER
        .iter()
        .copied()
        .filter(|f| q.iter().any(|(k, v)| k == f && *v > TOLERANCE))
        .collect()
}

/// Whether qpos, qvel, act or time differ after our step `k_ours` and
/// MuJoCo's checkpoint `k_mj` (both 0-based indices into their trajectories).
pub fn state_differs(ours: &Value, mj: &Value, k_ours: usize, k_mj: usize) -> bool {
    ["q", "v", "a", "t"].iter().any(|x| {
        match (arr(&ours[*x]).get(k_ours), arr(&mj[*x]).get(k_mj)) {
            (Some(a), Some(b)) => scaled(a, b) > TOLERANCE,
            _ => true,
        }
    })
}
