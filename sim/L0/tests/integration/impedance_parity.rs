//! Constraint impedance against MuJoCo 3.5.0.
//!
//! MuJoCo takes one impedance for all rows of a connect (3) or a weld (6),
//! from the norm of their violation (`getposdim`,
//! `engine_core_constraint.c:1365-1398`), and a row's impedance from
//! `pos - margin` (`getimpedance`, `:1413-1461`, whose flat and linear
//! tests compare exactly), and replaces a mixed
//! `solref` or `solreffriction` (`getsolparam`, `:1291-1362`). A pyramidal or
//! elliptic contact's rows then take R from the normal row's, by impratio and
//! the friction ratios, and the regularized cone's μ from their R; every
//! row's diagApprox is remade from its R (`mj_makeImpedance`,
//! `:1465-1579`). The golden is
//! `assets/golden/impedance/impedance.json`, from the unfused MuJoCo 3.5.0
//! oracle (`scripts/gen_impedance_reference.py`, which describes its cases).

use serde_json::Value;
use sim_core::{ConstraintState, ConstraintType, Data, Model};
use sim_mjcf::load_model;

const TOL: f64 = 1e-12;

fn golden() -> Value {
    serde_json::from_str(include_str!("../assets/golden/impedance/impedance.json"))
        .expect("impedance golden parses")
}

fn floats(v: &Value) -> Vec<f64> {
    v.as_array()
        .expect("array")
        .iter()
        .map(|x| x.as_f64().expect("number"))
        .collect()
}

fn assert_close(what: &str, got: &[f64], want: &[f64], tol: f64) {
    assert_eq!(got.len(), want.len(), "{what}: length");
    for (i, (a, b)) in got.iter().zip(want).enumerate() {
        assert!((a - b).abs() <= tol, "{what}[{i}]: {a}, expected {b}");
    }
}

/// qpos and qvel at the golden's checkpoints, stepping `model` from `data`;
/// returns the data at the last checkpoint.
fn assert_trajectory(what: &str, model: &Model, mut data: Data, traj: &Value, tol: f64) -> Data {
    let checkpoints = traj.as_object().expect("checkpoints");
    let last = checkpoints
        .keys()
        .map(|k| k.parse::<usize>().expect("step"))
        .max();
    for step in 1..=last.expect("a checkpoint") {
        data.step(model).expect("step");
        if let Some(c) = checkpoints.get(&step.to_string()) {
            assert_close(
                &format!("{what} step {step} qpos"),
                data.qpos.as_slice(),
                &floats(&c["qpos"]),
                tol,
            );
            assert_close(
                &format!("{what} step {step} qvel"),
                data.qvel.as_slice(),
                &floats(&c["qvel"]),
                tol,
            );
        }
    }
    data
}

/// A hinged capsule and a free box joined by a connect, the hinge turned
/// 0.002 rad: each of the connect's three rows takes the impedance of the
/// norm of their violation, as MuJoCo's, and the acceleration follows.
#[test]
fn connect_impedance_is_shared_across_rows() {
    let case = &golden()["cases"]["connect_rows"];
    let model = load_model(case["xml"].as_str().expect("xml")).expect("load");
    let mut data = model.make_data();
    data.qpos[0] = case["hinge"].as_f64().expect("hinge");
    data.forward(&model).expect("forward");
    let rows: Vec<usize> = (0..data.efc_type.len())
        .filter(|&i| data.efc_type[i] == ConstraintType::Equality)
        .collect();
    let pos: Vec<f64> = rows.iter().map(|&i| data.efc_pos[i]).collect();
    assert_close("connect efc_pos", &pos, &floats(&case["efc_pos"]), TOL);
    let imp: Vec<f64> = rows.iter().map(|&i| data.efc_imp[i]).collect();
    assert_close("connect efc_imp", &imp, &floats(&case["efc_imp"]), 1e-15);
    assert_close("qacc", data.qacc.as_slice(), &floats(&case["qacc"]), TOL);
}

/// A box on a hinge welded to the world, started 0.008 rad from its weld pose,
/// under Euler, implicit and implicitfast.
#[test]
fn weld_matches_mujoco_3_5_0() {
    let case = &golden()["cases"]["weld_hinge"];
    for (integrator, traj) in case["traj"].as_object().expect("traj") {
        let model = load_model(case["xml"][integrator].as_str().expect("xml")).expect("load");
        let mut data = model.make_data();
        data.qpos[0] += case["hinge"].as_f64().expect("hinge");
        assert_trajectory(&format!("weld {integrator}"), &model, data, traj, TOL);
    }
}

/// The composite cable of `implicit_integration.rs`'s
/// `test_implicitfast_connect_ball_chain_stability`, as MuJoCo writes it out
/// (plain bodies), its last segment connected to a fixed body.
#[test]
fn a_connected_cable_matches_mujoco_3_5_0() {
    let case = &golden()["cases"]["cable"];
    let model = load_model(case["xml"].as_str().expect("xml")).expect("load");
    let data = model.make_data();
    assert_trajectory("cable", &model, data, &case["traj"], CABLE_TOL);
}

const CABLE_TOL: f64 = 1e-12;

/// A hinge limited to ±0.5 with margin 0.1, at 0.42 rad: its upper limit
/// row is 0.08 from the limit, 0.02 inside the margin, and takes the
/// impedance at 0.02.
#[test]
fn a_limit_takes_its_impedance_inside_the_margin() {
    let case = &golden()["cases"]["limit_margin"];
    let model = load_model(case["xml"].as_str().expect("xml")).expect("load");
    let mut data = model.make_data();
    data.qpos[0] = case["hinge"].as_f64().expect("hinge");
    data.forward(&model).expect("forward");
    let rows: Vec<usize> = (0..data.efc_type.len())
        .filter(|&i| data.efc_type[i] == ConstraintType::LimitJoint)
        .collect();
    assert_eq!(rows.len(), 1, "one limit row");
    let i = rows[0];
    let want = |k: &str| case[k].as_f64().expect(k);
    assert_close("limit efc_pos", &[data.efc_pos[i]], &[want("efc_pos")], TOL);
    assert_close(
        "limit efc_margin",
        &[data.efc_margin[i]],
        &[want("efc_margin")],
        TOL,
    );
    assert_close(
        "limit efc_imp",
        &[data.efc_imp[i]],
        &[want("efc_imp")],
        1e-15,
    );
}

/// Census doc d81df1fa, a sphere sliding on a plane under an elliptic cone
/// whose `<pair>` gives `solreffriction="0.1 0.0"`: one value positive, the
/// other not. MuJoCo replaces it by (0, 0), so the friction rows take the
/// pair's `solref` (`getsolparam`, `engine_core_constraint.c:1345-1349`;
/// `mj_makeImpedance`, `:1489-1491`).
///
/// The second case gives the pair `solref="0.05 1"`, not the default, and
/// `solreffriction="-0.1 0.5"`, mixed the other way: there a mixed
/// solreffriction replaced by the default solref, not by (0, 0), would show.
#[test]
fn mixed_solreffriction_uses_solref() {
    for name in ["mixed_solreffriction", "mixed_solreffriction_own_solref"] {
        let case = &golden()["cases"][name];
        let model = load_model(case["xml"].as_str().expect("xml")).expect("load");
        let mut data = model.make_data();
        data.qvel.copy_from_slice(&floats(&case["qvel"]));
        assert_trajectory(name, &model, data, &case["traj"], TOL);
    }
}

/// Two hinged arms falling onto their limits, whose `solreflimit` is mixed
/// each way ("0.05 -2", "-500 2"): MuJoCo replaces it by the default
/// (0.02, 1) (`getsolparam`, `engine_core_constraint.c:1334-1338`), which the
/// limit rows then hold.
#[test]
fn mixed_solref_uses_default() {
    let case = &golden()["cases"]["mixed_solref_limit"];
    let model = load_model(case["xml"].as_str().expect("xml")).expect("load");
    let end = assert_trajectory(
        "mixed solref limit",
        &model,
        model.make_data(),
        &case["traj"],
        TOL,
    );
    assert_eq!(
        end.efc_type,
        [ConstraintType::LimitJoint; 2],
        "two limit rows"
    );
    assert_eq!(end.efc_solref, [[0.02, 1.0]; 2], "the limit rows' solref");
}

/// A mixed solref on each other row kind: a weld, a connect, a joint
/// equality, two hinges' friction loss, a fixed tendon's limit and friction
/// loss, a contact whose geoms' solref mix (solmix) to a mixed one, and
/// override's `o_solref`. MuJoCo replaces each by the default
/// (`getsolparam`, `engine_core_constraint.c:1334-1338`); 200 steps at TOL.
#[test]
fn mixed_solref_on_every_row_kind_uses_the_default() {
    for (name, case) in golden()["cases"]["mixed_solref_rows"]
        .as_object()
        .expect("cases")
    {
        let model = load_model(case["xml"].as_str().expect("xml")).expect("load");
        let mut data = model.make_data();
        if !case["qpos"].is_null() {
            data.qpos.copy_from_slice(&floats(&case["qpos"]));
        }
        if !case["qvel"].is_null() {
            data.qvel.copy_from_slice(&floats(&case["qvel"]));
        }
        assert_trajectory(name, &model, data, &case["traj"], TOL);
    }
}

/// Three limited hinges past their limits, each solimplimit at one of
/// `getimpedance`'s tests, which compare exactly
/// (`engine_core_constraint.c:1416`, `:1439`): a width of 1e-11, above
/// mjMINVAL, so the curve is not flat and saturates at `solimp[1]`;
/// `solimp[0]` and `solimp[1]` 5e-13 apart; and a power of 1 + 1e-13, not
/// linear. Each row's impedance is MuJoCo's, bit for bit on every platform:
/// the first two rows call no libm function (their power is 2), and the
/// last row's two `powf` calls, each or both off by up to 8 ulps, leave its
/// bits as they are (measured, replaying the formula; 9 move them).
#[test]
fn impedance_flat_and_linear_tests_are_exact() {
    let case = &golden()["cases"]["impedance_edges"];
    let model = load_model(case["xml"].as_str().expect("xml")).expect("load");
    let mut data = model.make_data();
    data.qpos.copy_from_slice(&floats(&case["qpos"]));
    data.forward(&model).expect("forward");
    assert_eq!(
        data.efc_type,
        [ConstraintType::LimitJoint; 3],
        "three limit rows"
    );
    let want = floats(&case["efc_imp"]);
    let differ: Vec<String> = data
        .efc_imp
        .iter()
        .zip(&want)
        .enumerate()
        .filter(|(_, (got, want))| got.to_bits() != want.to_bits())
        .map(|(i, (got, want))| format!("row {i}: {got}, expected {want}"))
        .collect();
    assert!(differ.is_empty(), "efc_imp differs: {differ:?}");
    assert_close("qacc", data.qacc.as_slice(), &floats(&case["qacc"]), TOL);
}

/// `|got − want|` over `max(1, |want|)`: R reaches 5e6 on a rolling row.
fn relative(got: f64, want: f64) -> f64 {
    (got - want).abs() / want.abs().max(1.0)
}

/// The first entry of `got` farther than `tol` (relative) from `want`, or a
/// length that differs.
fn differs(what: &str, got: &[f64], want: &[f64], tol: f64) -> Option<String> {
    if got.len() != want.len() {
        return Some(format!(
            "{what}: {} values, expected {}",
            got.len(),
            want.len()
        ));
    }
    got.iter()
        .zip(want)
        .enumerate()
        .find(|(_, (a, b))| relative(**a, **b) > tol)
        .map(|(i, (a, b))| format!("{what}[{i}]: {a}, expected {b}"))
}

/// The first checkpoint whose qpos or qvel is farther than `tol` from the
/// golden's, stepping `model` from `data`; with `niter`, also the first step
/// whose iteration count is not MuJoCo's.
fn trajectory_differs(
    what: &str,
    model: &Model,
    mut data: Data,
    traj: &Value,
    tol: f64,
    niter: Option<&Value>,
) -> Option<String> {
    let checkpoints = traj.as_object().expect("checkpoints");
    let last = checkpoints
        .keys()
        .map(|k| k.parse::<usize>().expect("step"))
        .max()
        .expect("a checkpoint");
    for step in 1..=last {
        if let Err(e) = data.step(model) {
            return Some(format!("{what} step {step}: {e}"));
        }
        if let Some(n) = niter.and_then(|n| n[step - 1].as_u64())
            && data.solver_niter as u64 != n
        {
            return Some(format!(
                "{what} step {step} niter: {}, expected {n}",
                data.solver_niter
            ));
        }
        if let Some(c) = checkpoints.get(&step.to_string()) {
            let q = differs("qpos", data.qpos.as_slice(), &floats(&c["qpos"]), tol);
            let v = differs("qvel", data.qvel.as_slice(), &floats(&c["qvel"]), tol);
            if let Some(d) = q.or(v) {
                return Some(format!("{what} step {step} {d}"));
            }
        }
    }
    None
}

/// A model from a golden case, its qvel set, after a forward.
fn forward_case(case: &Value) -> (Model, Data) {
    let model = load_model(case["xml"].as_str().expect("xml")).expect("load");
    let mut data = model.make_data();
    data.qvel.copy_from_slice(&floats(&case["qvel"]));
    data.forward(&model).expect("forward");
    (model, data)
}

/// A first forward's rows against `rows`: R, D and the forces, qacc, and the
/// contact's regularized cone μ.
fn rows_differ(what: &str, data: &Data, rows: &Value, out: &mut Vec<String>) {
    let checks = [
        differs("efc_R", &data.efc_R, &floats(&rows["efc_R"]), 1e-12),
        differs("efc_D", &data.efc_D, &floats(&rows["efc_D"]), 1e-12),
        differs(
            "efc_force",
            data.efc_force.as_slice(),
            &floats(&rows["efc_force"]),
            1e-12,
        ),
        differs("qacc", data.qacc.as_slice(), &floats(&rows["qacc"]), 1e-12),
        differs(
            "mu",
            &[data.contacts[0].regularized_mu],
            &[rows["mu"].as_f64().expect("mu")],
            1e-15,
        ),
    ];
    out.extend(checks.into_iter().flatten().map(|d| format!("{what}: {d}")));
}

/// A sphere 1 mm into a plane under an elliptic cone at condim 3, 4 and 6,
/// sliding and also spinning, and at impratio 10, sliding, spinning and
/// rolling, elliptic and pyramidal: the first forward's R, D, forces, qacc
/// and regularized cone μ are MuJoCo's. MuJoCo's elliptic friction rows take
/// R[1] = R[0]/impratio and R[j+1] = R[1]·f₀²/f_j² (a torsional row 4·10⁴
/// times the normal's at the default friction, a rolling one 10⁸ times), and
/// μ = f₀·√(R[1]/R[0]) (`engine_core_constraint.c:1536-1548`). The same at a
/// `<pair>` of condim 6 whose two sliding frictions differ (0.8, 0.4),
/// impratio 2.5, under Newton, CG and PGS, and under PGS 100 steps. Then
/// census doc 2cb92700 (`<pair condim="6" solreffriction="0.08 0.6">`),
/// started sliding and spinning, steps as MuJoCo's for 100 steps.
#[test]
fn elliptic_friction_rows_as_mujoco() {
    let g = golden();
    let mut out = Vec::new();
    for (name, case) in g["cases"]["elliptic_rows"].as_object().expect("cases") {
        let (_, data) = forward_case(case);
        rows_differ(name, &data, case, &mut out);
    }
    for (name, case) in g["cases"]["impratio"].as_object().expect("cases") {
        let (_, data) = forward_case(case);
        rows_differ(name, &data, &case["forward"], &mut out);
    }
    for (name, case) in g["cases"]["aniso_rows"].as_object().expect("cases") {
        let (model, data) = forward_case(case);
        rows_differ(&format!("aniso {name}"), &data, case, &mut out);
        if !case["traj"].is_null() {
            let mut data = model.make_data();
            data.qvel.copy_from_slice(&floats(&case["qvel"]));
            out.extend(trajectory_differs(
                &format!("aniso {name}"),
                &model,
                data,
                &case["traj"],
                TOL,
                None,
            ));
        }
    }
    let case = &g["cases"]["pair_condim6"];
    let model = load_model(case["xml"].as_str().expect("xml")).expect("load");
    let mut data = model.make_data();
    data.qvel.copy_from_slice(&floats(&case["qvel"]));
    out.extend(trajectory_differs(
        "pair_condim6",
        &model,
        data,
        &case["traj"],
        TOL,
        None,
    ));
    assert!(out.is_empty(), "{} differ:\n{}", out.len(), out.join("\n"));
}

/// Every row's diagApprox is made from its R, `R·imp/(1 − imp)`, as
/// MuJoCo's (`engine_core_constraint.c:1575-1578`): census docs a0ff89f4 and
/// 406a1991 (boxes into a plane, pyramidal) and 60c23753 (frictionless,
/// moved 1 mm into its plane), a scene with equality, friction-loss, limit
/// and elliptic contact rows, two hinges with friction loss whose rows'
/// diagApprox is an ulp from their dofs' invweight0 (the friction-loss and
/// limit rows bit for bit on the golden's platform), and the elliptic
/// sphere's rows.
#[test]
fn diag_approx_as_mujoco() {
    let g = golden();
    let mut out = Vec::new();
    for (name, case) in g["cases"]["diag_approx"].as_object().expect("cases") {
        let model = load_model(case["xml"].as_str().expect("xml")).expect("load");
        let mut data = model.make_data();
        if case["qpos"].is_null() {
            data.qpos[2] += case["dz"].as_f64().expect("dz");
        } else {
            data.qpos.copy_from_slice(&floats(&case["qpos"]));
            data.qvel.copy_from_slice(&floats(&case["qvel"]));
        }
        data.forward(&model).expect("forward");
        let checks = [
            differs(
                "efc_diagApprox",
                &data.efc_diagApprox,
                &floats(&case["efc_diagApprox"]),
                1e-12,
            ),
            differs("efc_R", &data.efc_R, &floats(&case["efc_R"]), 1e-12),
        ];
        out.extend(checks.into_iter().flatten().map(|d| format!("{name}: {d}")));
        // The friction-loss and limit rows' diagApprox, made from R, is
        // within an ulp of the one it is made from: bit for bit on macOS
        // arm64, where the golden was made, else within 1e-13.
        let want = floats(&case["efc_diagApprox"]);
        for (i, kind) in data.efc_type.iter().enumerate() {
            if !matches!(
                kind,
                ConstraintType::FrictionDof
                    | ConstraintType::FrictionTendon
                    | ConstraintType::LimitJoint
                    | ConstraintType::LimitTendon
            ) {
                continue;
            }
            let (a, b) = (data.efc_diagApprox[i], want[i]);
            if a.to_bits() != b.to_bits()
                && (cfg!(all(target_os = "macos", target_arch = "aarch64"))
                    || (a - b).abs() > 1e-13 * b.abs().max(1.0))
            {
                out.push(format!(
                    "{name}: efc_diagApprox[{i}] ({kind:?}): {a}, expected {b}"
                ));
            }
        }
    }
    for (name, case) in g["cases"]["elliptic_rows"].as_object().expect("cases") {
        let (_, data) = forward_case(case);
        let d = differs(
            "efc_diagApprox",
            &data.efc_diagApprox,
            &floats(&case["efc_diagApprox"]),
            1e-12,
        );
        out.extend(d.map(|d| format!("{name}: {d}")));
    }
    assert!(out.is_empty(), "{} differ:\n{}", out.len(), out.join("\n"));
}

/// Each diag_approx case's rows' diagApprox and R as MuJoCo's
/// `mj_diagApprox` and `mj_makeImpedance` round them, bit for bit on macOS
/// arm64 ([`golden_bits`]): a pyramidal facet pair's `tran + μ²·tran`, each
/// product apart (`engine_core_constraint.c:1276-1280`; the
/// `pyramidal_friction_*` cases, whose mass makes `(1 + μ²)·tran` round
/// otherwise, ledger L227), and a tendon equality's two tendons'
/// `tendon_invweight0` added (`:1139-1150`; `tendon_pair`, ledger L230).
/// `every_row_kind` is left to [`diag_approx_as_mujoco`]'s 1e-12: its weld's
/// rotational rows take its box's rotational `body_invweight0`, which rounds
/// 1 ulp off MuJoCo's (ledger L223).
#[test]
fn diag_approx_rounds_as_mujoco() {
    let g = golden();
    let mut off = Vec::new();
    for (name, case) in g["cases"]["diag_approx"].as_object().expect("cases") {
        if name == "every_row_kind" {
            continue;
        }
        let model = load_model(case["xml"].as_str().expect("xml")).expect("load");
        let mut data = model.make_data();
        if case["qpos"].is_null() {
            data.qpos[2] += case["dz"].as_f64().expect("dz");
        } else {
            data.qpos.copy_from_slice(&floats(&case["qpos"]));
            data.qvel.copy_from_slice(&floats(&case["qvel"]));
        }
        data.forward(&model).expect("forward");
        if let Some(weights) = case.get("tendon_invweight0") {
            let what = format!("{name} tendon_invweight0");
            golden_bits(&mut off, &what, &model.tendon_invweight0, &floats(weights));
        }
        let want = floats(&case["efc_diagApprox"]);
        golden_bits(
            &mut off,
            &format!("{name} efc_diagApprox"),
            &data.efc_diagApprox,
            &want,
        );
        let want = floats(&case["efc_R"]);
        golden_bits(&mut off, &format!("{name} efc_R"), &data.efc_R, &want);
    }
    assert!(off.is_empty(), "{} differ:\n{}", off.len(), off.join("\n"));
}

/// Where `got` differs from `want`, each as a line in `off`: in any bit on
/// macOS arm64, where the golden was made; elsewhere by more than
/// `1e-13 · max(1, |MuJoCo's|)`, and a NaN always.
fn golden_bits(off: &mut Vec<String>, what: &str, got: &[f64], want: &[f64]) {
    if got.len() != want.len() {
        off.push(format!(
            "{what}: {} values, MuJoCo {}",
            got.len(),
            want.len()
        ));
        return;
    }
    let exact = cfg!(all(target_os = "macos", target_arch = "aarch64"));
    for (i, (a, b)) in got.iter().zip(want).enumerate() {
        let same = if exact {
            a.to_bits() == b.to_bits()
        } else {
            // Within the bound or pushed: a NaN is never within it.
            (a - b).abs() <= 1e-13 * b.abs().max(1.0)
        };
        if !same {
            off.push(format!("{what}[{i}]: {a:e}, MuJoCo {b:e}"));
        }
    }
}

/// The sphere at impratio 10, sliding, spinning and rolling, at condim 3 and
/// 6 under an elliptic cone (Newton, CG, PGS) and a pyramidal one (Newton):
/// 300 steps within 1e-9 of MuJoCo's, the book's tolerance (A18 §10), where
/// the regularized cone μ = f₀·√(R[1]/R[0]) is not f₀. A checkpoint is
/// compared only where MuJoCo's own 1-ulp spread there (the golden's: the
/// largest change of qpos when one starting velocity moves by 1 ulp) is
/// below that tolerance, and each case keeps at least three. Each of the 300
/// steps' iteration counts is MuJoCo's.
#[test]
fn impratio_steps_as_mujoco() {
    const IMPRATIO_TOL: f64 = 1e-9;
    let g = golden();
    let mut out = Vec::new();
    for (name, case) in g["cases"]["impratio"].as_object().expect("cases") {
        let model = load_model(case["xml"].as_str().expect("xml")).expect("load");
        let mut data = model.make_data();
        data.qvel.copy_from_slice(&floats(&case["qvel"]));
        let spread = &case["ulp_spread"];
        let kept: serde_json::Map<String, Value> = case["traj"]
            .as_object()
            .expect("traj")
            .iter()
            .filter(|(step, _)| spread[step.as_str()].as_f64().expect("spread") < IMPRATIO_TOL)
            .map(|(step, c)| (step.clone(), c.clone()))
            .collect();
        assert!(
            kept.len() >= 3,
            "{name}: {} checkpoints compared",
            kept.len()
        );
        out.extend(trajectory_differs(
            name,
            &model,
            data,
            &Value::Object(kept),
            IMPRATIO_TOL,
            Some(&case["niter"]),
        ));
    }
    assert!(out.is_empty(), "{} differ:\n{}", out.len(), out.join("\n"));
}

/// MuJoCo's `mjtConstraintState` code of a row's state.
fn mj_state(state: ConstraintState) -> u64 {
    match state {
        ConstraintState::Satisfied => 0,
        ConstraintState::Quadratic => 1,
        ConstraintState::LinearNeg => 2,
        ConstraintState::LinearPos => 3,
        ConstraintState::Cone => 4,
    }
}

/// The impratio sphere, elliptic, Newton, at condim 3 and 6, noslip on: the
/// first forward's iteration count, each statistic's active and changed
/// rows, and every row's state are MuJoCo's, under the rows' R and the
/// cone's μ that impratio sets. Noslip's `dualState` sorts the rows by that
/// μ (`engine_solver.c:272`); this does not pin it there: with friction[0]
/// in its place the test still passes (a mutant run).
#[test]
fn noslip_under_impratio_as_mujoco() {
    let g = golden();
    let mut out = Vec::new();
    for (name, case) in g["cases"]["impratio_noslip"].as_object().expect("cases") {
        let (_, data) = forward_case(case);
        let ints = |v: &Value| -> Vec<u64> {
            v.as_array()
                .expect("array")
                .iter()
                .map(|x| x.as_u64().expect("count"))
                .collect()
        };
        let checks = [
            (
                "niter",
                vec![data.solver_niter as u64],
                vec![case["niter"].as_u64().expect("niter")],
            ),
            (
                "nactive",
                data.solver_stat.iter().map(|s| s.nactive as u64).collect(),
                ints(&case["nactive"]),
            ),
            (
                "nchange",
                data.solver_stat.iter().map(|s| s.nchange as u64).collect(),
                ints(&case["nchange"]),
            ),
            (
                "efc_state",
                data.efc_state.iter().map(|&s| mj_state(s)).collect(),
                ints(&case["efc_state"]),
            ),
        ];
        for (what, ours, mujoco) in checks {
            if ours != mujoco {
                out.push(format!("{name} {what}: {ours:?}, MuJoCo {mujoco:?}"));
            }
        }
    }
    assert!(out.is_empty(), "{} differ:\n{}", out.len(), out.join("\n"));
}

/// A free box welded to the world, started turned and spinning, at
/// torquescale 0, 0.5 and 2, and two free boxes welded to each other at 0.3:
/// MuJoCo scales the weld's rotational error and rotational Jacobian rows by
/// `eq_data[10]` (`engine_core_constraint.c:482`, `:510`, `:530`); 500 steps
/// within 1e-12. sim-mjcf reads no `torquescale` until Rigid-loading's L41,
/// so the test writes it into `eq_data[10]` after loading.
#[test]
fn weld_torquescale_matches_mujoco() {
    let g = golden();
    let mut out = Vec::new();
    for (name, case) in g["cases"]["weld_torquescale"].as_object().expect("cases") {
        let mut model = load_model(case["xml"].as_str().expect("xml")).expect("load");
        model.eq_data[0][10] = case["torquescale"].as_f64().expect("torquescale");
        let mut data = model.make_data();
        data.qpos.copy_from_slice(&floats(&case["qpos"]));
        data.qvel.copy_from_slice(&floats(&case["qvel"]));
        out.extend(trajectory_differs(
            name,
            &model,
            data,
            &case["traj"],
            TOL,
            None,
        ));
    }
    assert!(out.is_empty(), "{} differ:\n{}", out.len(), out.join("\n"));
}
