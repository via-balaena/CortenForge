//! The implicit integrators against MuJoCo 3.5.0: `M − h·∂f/∂v` taken on
//! MuJoCo's patterns and factored as MuJoCo factors it (`mj_implicitSkip`,
//! `engine_forward.c:1128-1353`), and the compiler's simple bodies and copied
//! frames, which the patterns and the kinematics read.
//!
//! The golden is `assets/golden/implicit/implicit.json`, from the unfused
//! MuJoCo 3.5.0 oracle (`scripts/gen_implicit_reference.py`, which describes
//! its cases).

use nalgebra::{Quaternion, UnitQuaternion, Vector3};
use serde_json::Value;
use sim_core::{Data, Integrator, Model, SameFrame};
use sim_mjcf::load_model;

const TOL: f64 = 1e-12;

fn golden() -> Value {
    serde_json::from_str(include_str!("../assets/golden/implicit/implicit.json"))
        .expect("implicit golden parses")
}

fn floats(v: &Value) -> Vec<f64> {
    v.as_array()
        .expect("array")
        .iter()
        .map(|x| x.as_f64().expect("number"))
        .collect()
}

fn ints(v: &Value) -> Vec<usize> {
    v.as_array()
        .expect("array")
        .iter()
        .map(|x| usize::try_from(x.as_u64().expect("integer")).expect("fits"))
        .collect()
}

fn assert_close(what: &str, got: &[f64], want: &[f64], tol: f64) {
    assert_eq!(got.len(), want.len(), "{what}: length");
    for (i, (a, b)) in got.iter().zip(want).enumerate() {
        assert!((a - b).abs() <= tol, "{what}[{i}]: {a}, expected {b}");
    }
}

fn load(case: &Value) -> Model {
    load_model(case["xml"].as_str().expect("xml")).expect("load")
}

/// Steps `data` to each of the case's checkpoints and compares `qpos` and
/// `qvel` with MuJoCo's, and after the first step the factor it left
/// (`qH` and `qHDiagInv`, or `qLU`).
fn assert_trajectory(name: &str, model: &Model, data: &mut Data, traj: &Value) {
    let checkpoints: Vec<usize> = traj
        .as_object()
        .expect("object")
        .keys()
        .filter_map(|k| k.parse().ok())
        .collect();
    let last = checkpoints.iter().copied().max().expect("checkpoints");
    for step in 1..=last {
        data.step(model).expect("step");
        if step == 1 {
            let factor = &traj["factor"];
            if model.integrator == Integrator::Implicit {
                assert_close(
                    &format!("{name} qLU"),
                    &data.qLU,
                    &floats(&factor["qLU"]),
                    TOL,
                );
            } else {
                assert_close(&format!("{name} qH"), &data.qH, &floats(&factor["qH"]), TOL);
                assert_close(
                    &format!("{name} qHDiagInv"),
                    &data.qHDiagInv,
                    &floats(&factor["qHDiagInv"]),
                    TOL,
                );
            }
        }
        if checkpoints.contains(&step) {
            let want = &traj[step.to_string()];
            assert_close(
                &format!("{name} qpos at step {step}"),
                data.qpos.as_slice(),
                &floats(&want["qpos"]),
                TOL,
            );
            assert_close(
                &format!("{name} qvel at step {step}"),
                data.qvel.as_slice(),
                &floats(&want["qvel"]),
                TOL,
            );
        }
    }
}

/// A free ellipsoid in a fluid under implicitfast (census doc
/// `d58bb68a2ae92f44`): its six dofs are simple, so MuJoCo's mass-matrix
/// pattern holds their diagonal only and its step leaves out the fluid's
/// coupling between them. Before, every entry of `M − h·D` was solved.
#[test]
fn implicitfast_fluid_matches_mujoco_3_5_0() {
    let case = &golden()["cases"]["fluid"];
    let model = load(case);
    let mut data = model.make_data();
    data.qvel
        .as_mut_slice()
        .copy_from_slice(&floats(&case["qvel"]));
    assert_trajectory("fluid", &model, &mut data, &case["traj"]);
}

/// A damped fixed tendon over two arms off the world: MuJoCo's implicit and
/// implicitfast steps leave out its damping between the two branches, as
/// MuJoCo documents (`doc/computation/index.rst:544-546`), and keep it within
/// the jointed arm. Before, both kept every term.
#[test]
fn implicit_cross_branch_tendon_damping() {
    let g = golden();
    for integrator in ["implicit", "implicitfast"] {
        let case = &g["cases"]["cross_branch"][integrator];
        let model = load(case);
        let mut data = model.make_data();
        data.qpos
            .as_mut_slice()
            .copy_from_slice(&floats(&case["qpos"]));
        data.qvel
            .as_mut_slice()
            .copy_from_slice(&floats(&case["qvel"]));
        assert_trajectory(integrator, &model, &mut data, &case["traj"]);
    }
}

/// Two hinged arms off one free body, joined by a damped fixed tendon and a
/// damped spatial tendon: D's pattern pairs every dof of the free body with
/// both arms' but leaves out the pair of the two arms, sibling branches of
/// one tree (MuJoCo's `D_rownnz`, `D_rowadr`, `D_diag`, `D_colind`), and the
/// implicit and implicitfast steps leave out the tendons' damping between
/// them.
#[test]
fn implicit_sibling_branches_leave_out_their_cross_terms() {
    let g = golden();
    for integrator in ["implicit", "implicitfast"] {
        let case = &g["cases"]["sibling_branches"][integrator];
        let model = load(case);
        let d = &case["D"];
        for (field, got) in [
            ("D_rownnz", &model.D_rownnz),
            ("D_rowadr", &model.D_rowadr),
            ("D_diag", &model.D_diag),
            ("D_colind", &model.D_colind),
        ] {
            assert_eq!(got, &ints(&d[field]), "{integrator}: {field}");
        }
        let mut data = model.make_data();
        data.qpos
            .as_mut_slice()
            .copy_from_slice(&floats(&case["qpos"]));
        data.qvel
            .as_mut_slice()
            .copy_from_slice(&floats(&case["qvel"]));
        assert_trajectory(integrator, &model, &mut data, &case["traj"]);
    }
}

/// A sphere whose geom and centre of mass are off its body's origin, held
/// by one vertical slide 5 mm into a plane (ledger L112): the body is not
/// simple, so its `body_invweight0` is MuJoCo's (1/3, 1/3), not the short
/// form's (1, 0), and the contact's force, whose `R` reads it, is MuJoCo's.
#[test]
fn slide_held_sphere_contact_matches_mujoco_3_5_0() {
    let case = &golden()["cases"]["slide_contact"];
    let model = load(case);
    let invweight: Vec<f64> = model.body_invweight0.iter().flatten().copied().collect();
    assert_close(
        "body_invweight0",
        &invweight,
        &floats(&case["body_invweight0"]),
        TOL,
    );
    assert_close(
        "dof_invweight0",
        &model.dof_invweight0,
        &floats(&case["dof_invweight0"]),
        TOL,
    );
    let mut data = model.make_data();
    data.forward(&model).expect("forward");
    assert_close(
        "efc_force",
        data.efc_force.as_slice(),
        &floats(&case["efc_force"]),
        TOL,
    );
}

/// A double pendulum whose fixed tendon has stiffness −2 and damping −50, set
/// in code (the loader refuses negative tendon parameters; MuJoCo loads
/// them): under implicitfast `M − h·D` is not positive definite (one diagonal
/// of MuJoCo's factor is negative), and MuJoCo's `mj_factorI`, which checks
/// no pivot, steps it (ledger L103). Before, the step refused it
/// (`CholeskyFailed`).
#[test]
fn implicitfast_steps_where_m_minus_hd_is_not_positive_definite() {
    let case = &golden()["cases"]["negative_damping"];
    let mut model = load(case);
    model.tendon_stiffness[0] = case["tendon_stiffness"].as_f64().expect("number");
    model.tendon_damping[0] = case["tendon_damping"].as_f64().expect("number");
    model.compute_implicit_params();
    assert!(
        floats(&case["qHDiagInv"]).iter().any(|d| *d < 0.0),
        "the fixture's factor has a negative diagonal"
    );
    let mut data = model.make_data();
    data.qpos
        .as_mut_slice()
        .copy_from_slice(&floats(&case["qpos"]));
    data.qvel
        .as_mut_slice()
        .copy_from_slice(&floats(&case["qvel"]));
    assert_trajectory("negative_damping", &model, &mut data, &case["traj"]);
}

/// With sleep enabled and a tree asleep, MuJoCo's implicit integrators fill
/// `M − h·D` over every row but factor and solve the awake dofs only
/// (`dof_awake_ind`): after a step the sleeping tree's rows of `qH` or `qLU`
/// hold `M − h·D` unfactored, as MuJoCo's do, its `qHDiagInv` entries are
/// not written, and the moving tree steps as MuJoCo's. (Those `qHDiagInv`
/// entries differ from MuJoCo's: sim-core factors in the acceleration stage
/// and MuJoCo in its integrator, so the forward pass `make_data` runs before
/// putting the tree to sleep wrote them, where MuJoCo's hold 0.)
#[test]
fn implicit_integrators_factor_the_awake_dofs() {
    let g = golden();
    for integrator in ["implicitfast", "implicit"] {
        let case = &g["cases"]["sleeping"][integrator];
        let model = load(case);
        let mut data = model.make_data();
        assert_eq!(
            data.nv_awake, 2,
            "{integrator}: the resting tree starts asleep"
        );
        data.qpos
            .as_mut_slice()
            .copy_from_slice(&floats(&case["qpos"]));
        data.qvel
            .as_mut_slice()
            .copy_from_slice(&floats(&case["qvel"]));
        data.step(&model).expect("step");
        let want = &case["traj"]["factor"];
        if model.integrator == Integrator::Implicit {
            assert_close("qLU", &data.qLU, &floats(&want["qLU"]), TOL);
        } else {
            assert_close("qH", &data.qH, &floats(&want["qH"]), TOL);
            // The sleeping entries are MuJoCo's 0: no forward pass (that of
            // `make_data` included) factors.
            assert_close(
                "qHDiagInv",
                &data.qHDiagInv,
                &floats(&want["qHDiagInv"]),
                TOL,
            );
        }
        for _ in 2..=10 {
            data.step(&model).expect("step");
        }
        let want = &case["traj"]["10"];
        assert_close(
            &format!("{integrator} qpos at step 10"),
            data.qpos.as_slice(),
            &floats(&want["qpos"]),
            TOL,
        );
        assert_close(
            &format!("{integrator} qvel at step 10"),
            data.qvel.as_slice(),
            &floats(&want["qvel"]),
            TOL,
        );
        assert_eq!(
            data.nv_awake, 2,
            "{integrator}: the resting tree stays asleep"
        );
    }
}

/// A body whose joints are all slides takes MuJoCo's short form of
/// `body_invweight0` and `dof_invweight0`, `1/mass` with its armature left
/// out, only when it is simple (`engine_setconst.c:628-635`, `:663-673`):
/// the bodies of the flags case, among them slides on a body with a child, on
/// a child of a moving body and with a turned inertial frame. Before, every
/// body whose joints were all slides took it.
#[test]
fn invweight0_reads_simple_bodies_as_mujoco() {
    let case = &golden()["cases"]["flags"];
    let model = load(case);
    let invweight: Vec<f64> = model.body_invweight0.iter().flatten().copied().collect();
    let want = &case["invweight"];
    assert_close(
        "body_invweight0",
        &invweight,
        &floats(&want["body_invweight0"]),
        TOL,
    );
    assert_close(
        "dof_invweight0",
        &model.dof_invweight0,
        &floats(&want["dof_invweight0"]),
        TOL,
    );
}

/// MuJoCo's `mjtSameFrame` numbering.
fn same_frame(code: usize) -> SameFrame {
    match code {
        0 => SameFrame::None,
        1 => SameFrame::Body,
        2 => SameFrame::Inertia,
        3 => SameFrame::BodyRot,
        4 => SameFrame::InertiaRot,
        _ => panic!("no mjtSameFrame {code}"),
    }
}

fn to_vec3s(v: &[f64]) -> Vec<Vector3<f64>> {
    v.chunks(3)
        .map(|c| Vector3::new(c[0], c[1], c[2]))
        .collect()
}

fn to_quats(v: &[f64]) -> Vec<UnitQuaternion<f64>> {
    v.chunks(4)
        .map(|c| UnitQuaternion::new_unchecked(Quaternion::new(c[0], c[1], c[2], c[3])))
        .collect()
}

fn mat_rows(m: &[nalgebra::Matrix3<f64>]) -> Vec<f64> {
    m.iter()
        .flat_map(|x| (0..3).flat_map(move |r| (0..3).map(move |c| x[(r, c)])))
        .collect()
}

/// Bodies of each kind, simple or not and why, and geoms and sites at, within
/// 1e-6 of, and off their body's frame and inertial frame: the flags and the
/// two dof–dof patterns equal MuJoCo's compiler's as loaded; and with the
/// model's frames set to MuJoCo's, every inertial, geom and site frame the
/// kinematics write equals MuJoCo's bit for bit, a frame within 1e-6 copied
/// as MuJoCo's `mj_local2Global` copies it (ledger L113). Before, those
/// frames were computed: 1e-7 off.
#[test]
fn simple_bodies_and_copied_frames_match_mujoco() {
    let case = &golden()["cases"]["flags"];
    let mut model = load(case);
    let derived = &case["derived"];
    for (name, ours) in [
        ("body_sameframe", &model.body_sameframe),
        ("geom_sameframe", &model.geom_sameframe),
        ("site_sameframe", &model.site_sameframe),
    ] {
        let want: Vec<SameFrame> = ints(&derived[name]).into_iter().map(same_frame).collect();
        assert_eq!(*ours, want, "{name}");
    }
    let simple: Vec<usize> = model.body_simple.iter().map(|&s| usize::from(s)).collect();
    assert_eq!(simple, ints(&derived["body_simple"]), "body_simple");
    for (name, ours) in [
        ("dof_simplenum", &model.dof_simplenum),
        ("M_rownnz", &model.M_rownnz),
        ("M_rowadr", &model.M_rowadr),
        ("M_colind", &model.M_colind),
        ("D_rownnz", &model.D_rownnz),
        ("D_rowadr", &model.D_rowadr),
        ("D_diag", &model.D_diag),
        ("D_colind", &model.D_colind),
    ] {
        assert_eq!(*ours, ints(&derived[name]), "{name}");
    }

    let fields = &case["model"];
    model.body_pos = to_vec3s(&floats(&fields["body_pos"]));
    model.body_quat = to_quats(&floats(&fields["body_quat"]));
    model.body_ipos = to_vec3s(&floats(&fields["body_ipos"]));
    model.body_iquat = to_quats(&floats(&fields["body_iquat"]));
    model.jnt_pos = to_vec3s(&floats(&fields["jnt_pos"]));
    model.jnt_axis = to_vec3s(&floats(&fields["jnt_axis"]));
    model.geom_pos = to_vec3s(&floats(&fields["geom_pos"]));
    model.geom_quat = to_quats(&floats(&fields["geom_quat"]));
    model.site_pos = to_vec3s(&floats(&fields["site_pos"]));
    model.site_quat = to_quats(&floats(&fields["site_quat"]));
    model.compute_simple_bodies();
    let mut data = model.make_data();
    data.qpos
        .as_mut_slice()
        .copy_from_slice(&floats(&case["qpos"]));
    data.forward(&model).expect("forward");
    let poses = &case["poses"];
    let vecs =
        |v: &[Vector3<f64>]| -> Vec<f64> { v.iter().flat_map(|x| [x.x, x.y, x.z]).collect() };
    for (name, ours) in [
        ("xipos", vecs(&data.xipos)),
        ("ximat", mat_rows(&data.ximat)),
        ("geom_xpos", vecs(&data.geom_xpos)),
        ("geom_xmat", mat_rows(&data.geom_xmat)),
        ("site_xpos", vecs(&data.site_xpos)),
        ("site_xmat", mat_rows(&data.site_xmat)),
    ] {
        assert_close(name, &ours, &floats(&poses[name]), 0.0);
    }
}

/// sim-core's own ImplicitSpringDamper under sleep takes each tendon's row
/// at the awake dofs only, so its solve is over the awake dofs, as MuJoCo's
/// implicit integrators factor: two hinged capsules on two trees, the second
/// asleep from the reset, joined by two damped fixed tendons (coefficients
/// 0.7 and 0.3; 0 and 1), the damping set after loading. With damping alone
/// MuJoCo's implicitfast step solves the same system there (its D leaves
/// out the tendons' entries between the two trees, which lie at the
/// sleeping dof), so its trajectory is the referent: `qpos` and `qvel` within
/// 1e-12·max(1, |MuJoCo's|) after each of 50 steps (measured: 1.1e-15).
/// Before (ledger L167), a tendon whose row reached an awake dof was taken
/// whole, so the solve over every dof coupled the moving arm to the sleeping
/// one through the tendons: `qvel` 2.3e-2 off by step 41.
#[test]
fn implicitspringdamper_under_sleep_solves_the_awake_dofs_as_mujoco() {
    let case = &golden()["cases"]["isd_referent"];
    let mut model = load_model(case["xml"].as_str().expect("xml")).expect("load");
    assert_eq!(model.integrator, Integrator::ImplicitSpringDamper);
    for (t, b) in floats(&case["tendon_damping"]).into_iter().enumerate() {
        model.tendon_damping[t] = b;
    }
    let mut data = model.make_data();
    data.qvel[0] = case["qvel0"].as_f64().expect("qvel0");
    for (k, want) in case["rows"].as_array().expect("rows").iter().enumerate() {
        data.step(&model).expect("step");
        assert_eq!(data.tree_awake, [true, false], "step {k}: trees");
        for (field, ours) in [("qpos", &data.qpos), ("qvel", &data.qvel)] {
            for (i, (a, b)) in ours.iter().zip(floats(&want[field])).enumerate() {
                assert!(
                    (a - b).abs() <= 1e-12 * b.abs().max(1.0),
                    "step {k}: {field}[{i}] {a}, MuJoCo's implicitfast {b}"
                );
            }
        }
    }
}
