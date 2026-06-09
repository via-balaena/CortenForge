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
//! * `canonical_reproduces_oracle_at_multidof_base_pose` — the emitted twin
//!   reproduces the oracle's moment arms about EVERY DOF (incl. the unwelded hip)
//!   at a pose with several non-zero hip rotations, confirming MuJoCo's
//!   multiple-hinges-on-one-body composition matches our FK (A2).
//! * `canonical_matches_frozen_reference` — the committed `knee_ref.xml` pins the
//!   exact bytes of *this* emitter's canonical output (catches any emitter change).
//! * `canonical_equals_explicit_identity_build` — the param/morph layer is a true
//!   no-op at identity (`build_canonical` == `emit` on the raw template).
//! * `uniform_scale_is_exact_dilation` — a uniform morph scales every moment arm
//!   by exactly `s`, shape preserved (ported from the validated morph spike).
//! * `anisotropic_morph_emits_loadable_model` — a per-segment morph still emits a
//!   model the importer accepts.
//! * `girth_morph_drives_moment_arms_end_to_end` — `with_girth_scales` exercised
//!   through realize→emit→load: girth measurably moves the moment arms (~20–30%,
//!   not second-order) and stays finite (the transverse path is live, not dead).
//! * `emit_tracks_oracle_under_scaling` — the shipped MJCF (which drops conditional
//!   path-points) stays within the S1 5 mm gate of the oracle when the body is
//!   *dialed*, confirming that emit↔oracle gap is orthogonal to scaling (cf-osim's
//!   differential oracle grades the oracle-vs-OpenSim; this guards the emit path).
//! * `length_round_trip_on_real_template` — Tier-2 internal consistency on the real
//!   gait2392 proportions: dialing real femur/tibia axial lengths and re-measuring
//!   reproduces the targets exactly (the convention pin, anchored on real geometry).

use cf_mjcf_emit::{build, build_canonical, emit};
use cf_msk_lib::{BodyParams, CanonicalSource, Model, SegmentScale, realize};
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
fn canonical_reproduces_oracle_at_multidof_base_pose() {
    // The emitted (simulatable) twin must reproduce the oracle's moment arms about
    // EVERY DOF — incl. the unwelded hip — at a pose with several non-zero hip
    // rotations at once. This confirms MuJoCo's multiple-hinges-on-one-body
    // composition matches our FK (and so, with the real-OpenSim cross-check, the
    // emitted twin matches OpenSim) for the hip, not just the knee.
    let t = template();
    let emitted = build_canonical(&t);
    let model = load_model(&emitted.mjcf).expect("emitted MJCF must load");
    let kin = Kinematics::new(&t);
    let eps = 0.5 * DEG;

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

    // The same multi-DOF base pose the real-OpenSim cross-check uses.
    let base = HashMap::from([
        ("hip_flexion_r".to_string(), 0.3),
        ("hip_adduction_r".to_string(), -0.2),
        ("hip_rotation_r".to_string(), 0.15),
        ("knee_angle_r".to_string(), -0.6),
    ]);

    for coord in [
        "hip_flexion_r",
        "hip_adduction_r",
        "hip_rotation_r",
        "knee_angle_r",
    ] {
        let drive = |v: f64, q: &mut [f64]| {
            let mut coords = base.clone();
            coords.insert(coord.to_string(), v);
            for (joint, val) in emitted.qpos_targets(&coords) {
                q[adr[&joint]] = val;
            }
        };
        for m in &t.muscles {
            let tdn = tendon_id(&model, &m.name);
            let emit_ma = coupled_moment_arm(&model, tdn, base[coord], eps, &drive) * 1000.0;
            let oracle_ma = kin.moment_arm(m, &base, coord, eps) * 1000.0;
            let d = (emit_ma - oracle_ma).abs();
            // The 5 mm gate (dropped-conditional approximation), about every DOF.
            assert!(
                d < 5.0,
                "{} about {coord}: emit {emit_ma:.2} vs oracle {oracle_ma:.2} mm (Δ {d:.2} > 5 mm)",
                m.name
            );
            // bifemlh_r has no dropped conditional/patella, so the emit reproduces
            // the oracle to ~machine precision about every DOF — the tight anchor
            // that the hip multi-hinge composition is exact, not just within 5 mm.
            if m.name == "bifemlh_r" {
                assert!(
                    d < 1e-2,
                    "{} about {coord}: emit vs oracle Δ {d:.2e} mm — expected ~0",
                    m.name
                );
            }
        }
    }
}

#[test]
fn anisotropic_morph_emits_loadable_model() {
    let t = template();
    // Per-axis anisotropy: distinct axial (length) and transverse (girth) factors
    // per segment — the A3 full-girth morph.
    let p = BodyParams {
        pelvis: SegmentScale::IDENTITY,
        femur: SegmentScale {
            axial: 2.0,
            transverse: 1.4,
        },
        tibia: SegmentScale {
            axial: 0.5,
            transverse: 0.8,
        },
    };
    let model = load_model(&emit(&realize(&t, &p)).mjcf)
        .expect("anisotropic morph must emit a loadable model");
    assert!(model.ntendon >= 4);
}

#[test]
fn girth_morph_drives_moment_arms_end_to_end() {
    // The full-girth machinery end-to-end on the real template: dialing girth
    // (transverse only, lengths untouched) emits a loadable model whose moment
    // arms are finite and MEASURABLY changed — proof the transverse path is live,
    // not a no-op. (Girth is NOT second-order: moving attachment points off the
    // bone axis shifts moment arms ~20–30%.) Note `with_girth_scales` is exercised
    // through `realize → emit → load → moment arms`, not just unit-tested.
    // The MAGNITUDE oracle (vs real OpenSim ScaleTool) and the §7 shape-correlation
    // bounds are A3-PR2/PR4; here we only assert the path is real and finite.
    let t = template();
    let base = moment_arm_curves(&build_canonical(&t));
    let girthed = moment_arm_curves(&emit(&realize(
        &t,
        &BodyParams::IDENTITY.with_girth_scales(1.4, 0.8),
    )));

    let mut any_changed = false;
    for ((name, b), (_, g)) in base.iter().zip(&girthed) {
        assert!(
            g.iter().all(|v| v.is_finite()),
            "{name}: non-finite moment arm under girth morph"
        );
        if b.iter()
            .zip(g)
            .any(|(x, y)| (x - y).abs() > 0.01 * x.abs().max(1e-6))
        {
            any_changed = true;
        }
    }
    assert!(
        any_changed,
        "girth had no effect on any moment arm — the transverse path is dead"
    );
}

#[test]
fn emit_tracks_oracle_under_scaling() {
    // The SHIPPED artifact (the emitted MJCF) drops ConditionalPathPoints, so even
    // unscaled it sits within the S1 5 mm gate of the oracle. cf-osim's differential
    // oracle grades the *oracle*-on-realized vs OpenSim ScaleTool (proving the morph
    // machinery); this complements it by checking the EMIT path under anisotropic
    // scaling — the conditional-drop residual must stay within the same 5 mm gate
    // when the body is dialed, confirming that gap is orthogonal to scaling (not a
    // residual that blows up under morphing).
    let t = template();
    let angles = sweep();
    let eps = 0.5 * DEG;
    let seg = |a: f64, tr: f64| SegmentScale {
        axial: a,
        transverse: tr,
    };
    let configs = [
        ("femur_axial_1.2", seg(1.2, 1.0), SegmentScale::IDENTITY),
        (
            "tibia_transverse_1.2",
            SegmentScale::IDENTITY,
            seg(1.0, 1.2),
        ),
        ("realistic_mix", seg(1.10, 1.05), seg(1.08, 0.95)),
    ];
    for (name, femur, tibia) in configs {
        let p = BodyParams {
            pelvis: SegmentScale::IDENTITY,
            femur,
            tibia,
        };
        let model = realize(&t, &p);
        let kin = cf_osim::oracle::Kinematics::new(&model);
        for (mname, ec) in &moment_arm_curves(&emit(&model)) {
            let m = model.muscles.iter().find(|mm| &mm.name == mname).unwrap();
            let n = angles.len() as f64;
            let rmse = (angles
                .iter()
                .zip(ec)
                .map(|(&th, &emit_mm)| {
                    let q = HashMap::from([("knee_angle_r".to_string(), th)]);
                    let oracle_mm = kin.moment_arm(m, &q, "knee_angle_r", eps) * 1000.0;
                    (emit_mm - oracle_mm).powi(2)
                })
                .sum::<f64>()
                / n)
                .sqrt();
            assert!(
                rmse < 5.0,
                "{name}/{mname}: emit vs oracle UNDER SCALING RMSE {rmse:.2}mm exceeds the 5mm \
                 S1 gate — the conditional-drop residual is not orthogonal to scaling"
            );
        }
    }
}

#[test]
fn length_round_trip_on_real_template() {
    let t = template();
    // Real gait2392 axial lengths (sanity: physiological femur/tibia).
    let f0 = t.segment_axial_length("femur_r", "tibia_r");
    let t0 = t.segment_axial_length("tibia_r", "talus_r");
    assert!((0.40..0.50).contains(&f0), "template femur axial {f0}");
    assert!((0.40..0.45).contains(&t0), "template tibia axial {t0}");

    // Dial a taller subject: realize → re-measure reproduces the targets exactly,
    // and the morphed model still emits something the importer accepts.
    for &(fl, tl) in &[(0.50, 0.46), (0.38, 0.40)] {
        let r = realize(&t, &BodyParams::from_lengths(&t, fl, tl));
        assert!((r.segment_axial_length("femur_r", "tibia_r") - fl).abs() < 1e-12);
        assert!((r.segment_axial_length("tibia_r", "talus_r") - tl).abs() < 1e-12);
        load_model(&emit(&r).mjcf).expect("length-dialed model must load");
    }
}

#[test]
fn emitted_twin_registers_millard_actuators() {
    // The canonical (force-carrying) template emits a driven Millard actuator per
    // muscle that the importer registers as a MillardMuscle on its tendon.
    use sim_core::{ActuatorDynamics, BiasType, GainType};
    let model = load_model(&cf_mjcf_emit::build_canonical(&template()).mjcf)
        .expect("muscle-driven canonical MJCF must load");
    for m in ["rect_fem_r", "vas_int_r", "bifemlh_r", "semimem_r"] {
        let aid = model
            .actuator_name
            .iter()
            .position(|n| n.as_deref() == Some(m))
            .unwrap_or_else(|| panic!("actuator {m} missing"));
        assert_eq!(model.actuator_dyntype[aid], ActuatorDynamics::MillardMuscle);
        assert_eq!(model.actuator_gaintype[aid], GainType::MillardMuscle);
        assert_eq!(model.actuator_biastype[aid], BiasType::MillardMuscle);
        assert_eq!(model.actuator_actrange[aid], (0.0, 1.0));
    }
}

#[test]
fn kinematic_only_emit_has_no_actuator_block() {
    // A muscle with no force params (the kinematic-only path) emits no actuator —
    // the emit is unchanged for force=None, and the MJCF still loads.
    let mut t = template();
    for m in &mut t.muscles {
        m.force = None;
    }
    let mjcf = emit(&t).mjcf;
    assert!(
        !mjcf.contains("<actuator>"),
        "force=None must emit no actuator block"
    );
    load_model(&mjcf).expect("kinematic-only MJCF must load");
}
