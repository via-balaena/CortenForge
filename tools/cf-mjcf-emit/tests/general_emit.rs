//! The general emitter's checkpoints — the oracle-gate no-regression for the
//! leg-region cutover.
//!
//! These run in CI (not `#[ignore]`) on the production `parse_leg_chain → realize
//! → emit` path:
//!
//! * `canonical_knee_reproduces_oracle` — `build_canonical` (NO scan) emits a knee
//!   whose moment arms reproduce the OpenSim-geometry oracle within the S1 5 mm
//!   gate, for all four muscles. The functional no-regression: the general emitter
//!   meets the same gate the bespoke `emit_coupled_knee` did. (The residual comes
//!   from dropped `ConditionalPathPoint`s — rect_fem_r/vas_int_r/semimem_r each
//!   have two; `bifemlh_r` has none and matches the oracle to ~machine precision,
//!   asserted below as a tight anchor that the emit geometry + coupled driving are
//!   exact.)
//! * `canonical_matches_frozen_reference` — the committed `knee_ref.xml` pins the
//!   exact bytes of *this* emitter's canonical output (catches any emitter change).
//! * `uniform_scale_is_exact_dilation` — a uniform morph scales every moment arm
//!   by exactly `s`, shape preserved (ported from the validated morph spike).
//! * `anisotropic_morph_emits_loadable_model` — a per-segment morph still emits a
//!   model the importer accepts.

use cf_mjcf_emit::{build, build_canonical, emit};
use cf_msk_lib::{BodyParams, CanonicalSource, Model, realize};
use cf_osim::oracle::Kinematics;
use cf_osim::parse_leg_chain;
use cf_osim::{coupled_moment_arm, joint_id, tendon_id};
use sim_mjcf::load_model;
use std::collections::HashMap;
use std::f64::consts::PI;

const DEG: f64 = PI / 180.0;

fn template() -> Model {
    let path = format!(
        "{}/../../sim/L0/tests/assets/opensim_gait2392/gait2392.osim",
        env!("CARGO_MANIFEST_DIR")
    );
    parse_leg_chain(&std::fs::read_to_string(path).expect("read gait2392.osim"))
}

fn sweep() -> Vec<f64> {
    (0..=20).map(|i| -(i as f64) * 5.0 * DEG).collect()
}

/// Moment-arm curve (mm) per muscle for an emitted model, driving every joint
/// (free hinge + coupled slides + patella) from the emitter's `driven` table.
fn moment_arm_curves(emitted: &cf_mjcf_emit::Emitted) -> Vec<(String, Vec<f64>)> {
    let angles = sweep();
    let eps = 0.5 * DEG;
    let model = load_model(&emitted.mjcf).expect("emitted MJCF must load");

    let adr: HashMap<String, usize> = emitted
        .driven
        .iter()
        .map(|d| {
            (
                d.joint.clone(),
                model.jnt_qpos_adr[joint_id(&model, &d.joint)],
            )
        })
        .collect();
    let drive = |th: f64, q: &mut [f64]| {
        let coords = HashMap::from([("knee_angle_r".to_string(), th)]);
        for (joint, val) in emitted.qpos_targets(&coords) {
            q[adr[&joint]] = val;
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
    // Built from library defaults — NO scan, NO landmarks.
    let curves = moment_arm_curves(&build_canonical(&t));
    let kin = Kinematics::new(&t);
    let angles = sweep();
    let eps = 0.5 * DEG;

    for ((name, curve), m) in curves.iter().zip(&t.muscles) {
        let n = angles.len() as f64;
        let rmse = (angles
            .iter()
            .zip(curve)
            .map(|(&th, &ours)| {
                let q = HashMap::from([("knee_angle_r".to_string(), th)]);
                let oracle = kin.moment_arm(m, &q, "knee_angle_r", eps) * 1000.0;
                (ours - oracle).powi(2)
            })
            .sum::<f64>()
            / n)
            .sqrt();
        // The S1 gate, through the no-scan general emitter.
        assert!(
            rmse < 5.0,
            "canonical {name}: moment-arm RMSE {rmse:.2} mm exceeds the 5 mm S1 gate"
        );
        // bifemlh_r has no conditional/moving points, so the emit places its sites
        // and drives the coupled knee exactly — it must match the oracle to
        // ~machine precision. A tight anchor that the only residual elsewhere is
        // the known dropped-conditional approximation, not an emitter bug.
        if name == "bifemlh_r" {
            assert!(
                rmse < 1e-3,
                "canonical {name}: RMSE {rmse:.2e} mm — expected ~0 (no conditional/moving points)"
            );
        }
    }
}

#[test]
fn canonical_matches_frozen_reference() {
    let reference = format!("{}/tests/assets/knee_ref.xml", env!("CARGO_MANIFEST_DIR"));
    let frozen = std::fs::read_to_string(&reference).expect(
        "knee_ref.xml must exist (regenerate with `cargo run -p cf-mjcf-emit --example dump_canonical`)",
    );
    assert_eq!(
        build_canonical(&template()).mjcf,
        frozen,
        "canonical MJCF changed vs the committed knee_ref.xml snapshot"
    );
}

#[test]
fn canonical_equals_explicit_identity_build() {
    // CanonicalSource → realize(IDENTITY) → emit must equal emit() on the raw
    // template — i.e. the param/morph layer is a true no-op at identity.
    let t = template();
    assert_eq!(build_canonical(&t).mjcf, emit(&t).mjcf);
    assert_eq!(
        build(&t, &CanonicalSource).mjcf,
        emit(&realize(&t, &BodyParams::IDENTITY)).mjcf
    );
}

#[test]
fn uniform_scale_is_exact_dilation() {
    let t = template();
    let base = moment_arm_curves(&build_canonical(&t));
    let s = 1.137;
    let scaled = moment_arm_curves(&emit(&realize(&t, &BodyParams::uniform(s))));

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

#[test]
fn anisotropic_morph_emits_loadable_model() {
    let t = template();
    let p = BodyParams {
        pelvis_scale: 1.0,
        femur_scale: 2.0,
        tibia_scale: 0.5,
    };
    let model = load_model(&emit(&realize(&t, &p)).mjcf)
        .expect("anisotropic morph must emit a loadable model");
    assert!(model.ntendon >= 4);
}
