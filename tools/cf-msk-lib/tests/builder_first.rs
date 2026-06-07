//! Builder-first artifact tests — the canonical (no-scan) body is real and
//! oracle-valid, and the morph seam is faithful.
//!
//! These run in CI (not `#[ignore]`) as regression guards on the production
//! `ParamSource → realize → emit` path. They read the vendored gait2392 `.osim`
//! (the same asset `cf-osim`'s `opensim_cross_check` uses).
//!
//! Coverage:
//! * `canonical_knee_reproduces_oracle` — `build_canonical` (NO scan) emits a knee
//!   whose moment arms reproduce the OpenSim-geometry oracle within the S1 5 mm
//!   gate, for all four muscles. This is the same gate `cf-osim`'s
//!   `s1_coupled_knee` scorecard asserts, now driven through the production API.
//! * `canonical_is_byte_identical_to_validated_emitter` — the API wrapper does not
//!   drift from the validated `emit_coupled_knee` (byte-stable emit, recon S1).
//! * `canonical_matches_frozen_reference` — the committed `knee_ref.xml` snapshot
//!   pins the exact canonical MJCF bytes (catches any emitter change).
//! * `uniform_scale_is_exact_dilation` — a uniform morph scales every moment arm
//!   by exactly `s` with shape preserved (ported from the validated morph spike).

use cf_msk_lib::{BodyParams, build_canonical, realize};
use cf_osim::emit::emit_coupled_knee;
use cf_osim::oracle::{Kinematics, Variant};
use cf_osim::osim::{Subgraph, parse_knee_subgraph};
use cf_osim::{coupled_moment_arm, joint_id, tendon_id};
use sim_mjcf::load_model;
use std::f64::consts::PI;

const DEG: f64 = PI / 180.0;

fn template() -> Subgraph {
    let path = format!(
        "{}/../../sim/L0/tests/assets/opensim_gait2392/gait2392.osim",
        env!("CARGO_MANIFEST_DIR")
    );
    parse_knee_subgraph(&std::fs::read_to_string(path).expect("read gait2392.osim"))
}

fn sweep() -> Vec<f64> {
    (0..=20).map(|i| -(i as f64) * 5.0 * DEG).collect()
}

/// Moment-arm curve (mm) per muscle for an emitted `sub`, driving every coupled
/// DOF with `sub`'s own splines — the validated S1 grading harness.
fn moment_arm_curves(sub: &Subgraph) -> Vec<(String, Vec<f64>)> {
    let angles = sweep();
    let eps = 0.5 * DEG;
    let emitted = emit_coupled_knee(sub);
    let model = load_model(&emitted.mjcf).expect("emitted MJCF must load");

    let adr_knee = model.jnt_qpos_adr[joint_id(&model, "knee")];
    let adr_tx = model.jnt_qpos_adr[joint_id(&model, "knee_tx")];
    let adr_ty = model.jnt_qpos_adr[joint_id(&model, "knee_ty")];
    let pat: Vec<_> = emitted
        .patellae
        .iter()
        .map(|p| {
            (
                model.jnt_qpos_adr[joint_id(&model, &p.jx)],
                model.jnt_qpos_adr[joint_id(&model, &p.jy)],
                model.jnt_qpos_adr[joint_id(&model, &p.jz)],
                &p.sx,
                &p.sy,
                &p.sz,
            )
        })
        .collect();
    let drive = |th: f64, q: &mut [f64]| {
        q[adr_knee] = th;
        q[adr_tx] = sub.knee.tx.eval(th);
        q[adr_ty] = sub.knee.ty.eval(th);
        for (ax, ay, az, sx, sy, sz) in &pat {
            q[*ax] = sx.eval(th);
            q[*ay] = sy.eval(th);
            q[*az] = sz.eval(th);
        }
    };

    emitted
        .muscles
        .iter()
        .map(|name| {
            let t = tendon_id(&model, name);
            let curve = angles
                .iter()
                .map(|&th| coupled_moment_arm(&model, t, th, eps, &drive) * 1000.0)
                .collect();
            (name.clone(), curve)
        })
        .collect()
}

#[test]
fn canonical_knee_reproduces_oracle() {
    let t = template();
    // The canonical body is built from library defaults — NO scan, NO landmarks.
    let canonical = build_canonical(&t);
    let model = load_model(&canonical.mjcf).expect("canonical MJCF must load");

    let kin = Kinematics::new(&t);
    let angles = sweep();
    let eps = 0.5 * DEG;

    let adr_knee = model.jnt_qpos_adr[joint_id(&model, "knee")];
    let adr_tx = model.jnt_qpos_adr[joint_id(&model, "knee_tx")];
    let adr_ty = model.jnt_qpos_adr[joint_id(&model, "knee_ty")];
    let pat: Vec<_> = canonical
        .patellae
        .iter()
        .map(|p| {
            (
                model.jnt_qpos_adr[joint_id(&model, &p.jx)],
                model.jnt_qpos_adr[joint_id(&model, &p.jy)],
                model.jnt_qpos_adr[joint_id(&model, &p.jz)],
                &p.sx,
                &p.sy,
                &p.sz,
            )
        })
        .collect();
    let drive = |th: f64, q: &mut [f64]| {
        q[adr_knee] = th;
        q[adr_tx] = t.knee.tx.eval(th);
        q[adr_ty] = t.knee.ty.eval(th);
        for (ax, ay, az, sx, sy, sz) in &pat {
            q[*ax] = sx.eval(th);
            q[*ay] = sy.eval(th);
            q[*az] = sz.eval(th);
        }
    };

    for m in &t.muscles {
        let tdn = tendon_id(&model, &m.name);
        let n = angles.len() as f64;
        let rmse = (angles
            .iter()
            .map(|&th| {
                let ours = coupled_moment_arm(&model, tdn, th, eps, &drive) * 1000.0;
                let oracle = kin.moment_arm(m, th, eps, Variant::TRUTH) * 1000.0;
                (ours - oracle).powi(2)
            })
            .sum::<f64>()
            / n)
            .sqrt();
        // The S1 gate, through the no-scan production API.
        assert!(
            rmse < 5.0,
            "canonical {}: moment-arm RMSE {rmse:.2} mm exceeds the 5 mm S1 gate",
            m.name
        );
    }
}

#[test]
fn canonical_is_byte_identical_to_validated_emitter() {
    let t = template();
    // CanonicalSource → realize(IDENTITY) → emit must equal the validated emitter
    // applied to the raw template — i.e. the param/morph layer is a true no-op.
    assert_eq!(
        build_canonical(&t).mjcf,
        emit_coupled_knee(&t).mjcf,
        "canonical build drifted from the validated emit_coupled_knee output"
    );
}

#[test]
fn canonical_matches_frozen_reference() {
    let reference = format!("{}/tests/assets/knee_ref.xml", env!("CARGO_MANIFEST_DIR"));
    let frozen = std::fs::read_to_string(&reference)
        .expect("knee_ref.xml must exist (regenerate with `cargo run -p cf-msk-lib --example dump_canonical`)");
    assert_eq!(
        build_canonical(&template()).mjcf,
        frozen,
        "canonical MJCF changed vs the committed knee_ref.xml snapshot"
    );
}

#[test]
fn uniform_scale_is_exact_dilation() {
    let t = template();
    let base = moment_arm_curves(&t);
    let s = 1.137;
    let scaled = moment_arm_curves(&realize(&t, &BodyParams::uniform(s)));

    for ((name, b), (_, u)) in base.iter().zip(scaled.iter()) {
        for (bi, ui) in b.iter().zip(u) {
            assert!(
                (ui - s * bi).abs() < 1e-6,
                "{name}: uniform scale not an exact dilation ({ui} vs {})",
                s * bi
            );
        }
    }
}
