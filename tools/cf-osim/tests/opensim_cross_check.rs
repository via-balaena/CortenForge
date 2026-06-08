//! The independent anchor: cf-osim's oracle vs REAL OpenSim 4.6.
//!
//! Every other cf-osim test compares the engine against our *re-derivation* of
//! OpenSim's GeometryPath geometry — a self-consistency loop. These tests close
//! that loop by grading our oracle (the general FK + natural-cubic SimmSplines)
//! against moment arms computed by real OpenSim 4.6 on the same gait2392 model.
//! Both references are vendored (generated via the `opensim` PyPI wheel), so this
//! runs in CI with no OpenSim install.
//!
//! * `oracle_matches_real_opensim_knee_rom` — the broad knee study: knee
//!   0 → −100° at the neutral hip (`knee_moment_arms_opensim.json`, generator
//!   `gen_moment_arms.py`). Validates the coupled-knee SimmSpline geometry across
//!   the full flexion ROM.
//! * `oracle_matches_real_opensim_multidof` — the unwelded-hip study (A2): every
//!   moment arm evaluated at a **multi-DOF base pose** with several non-zero hip
//!   rotations at once (`moment_arms_opensim.json`, generator
//!   `gen_leg_moment_arms.py`). Passing this pins the hip rotation-*composition
//!   order* (R-rot) against real OpenSim, not just one rotation in isolation.
//!
//! * `morph_matches_real_opensim_scaletool` — the A3 **differential oracle**:
//!   our per-axis morph (`cf_msk_lib::realize`) graded against OpenSim's own
//!   ScaleTool (`scaled_moment_arms_opensim.json`, generator
//!   `gen_scaled_moment_arms.py`). For a grid of per-axis scale configs (length &
//!   girth on each segment), OpenSim scales gait2392 and we `realize` the same
//!   factors; the **knee** moment arms (4 muscles, 0…−100° ROM, neutral hip) of our
//!   morphed model must match OpenSim's scaled model to within ~the unscaled band.
//!   This is what makes a *dialed* body validatable: for this scope, the morph
//!   reproduces OpenSim's ScaleTool. **Scope/caveats:** graded on the **oracle**
//!   (which keeps the conditional path-points), so it validates the *morph
//!   machinery* — NOT the emitted MJCF (whose deep-flexion residual is the
//!   separate, known dropped-conditional S1 approximation; that the residual stays
//!   within the S1 gate *under scaling* is checked by cf-mjcf-emit's
//!   `emit_tracks_oracle_under_scaling`). Knee moment arms only (hip-under-scaling
//!   not graded); the pelvis is the fixed root (knee moment arms are translation-
//!   invariant to it), so it is exercised only in the uniform config.
//!
//! Together: the first two confirm our oracle (= the FK the emitter ships)
//! reproduces OpenSim's leg muscle geometry across both joints + the ROM
//! *unscaled*; the third confirms morphing the knee tracks OpenSim's own scaling.

use cf_msk_lib::{BodyParams, SegmentScale, realize};
use cf_osim::oracle::{Kinematics, Pose};
use cf_osim::parse_leg_chain;
use std::f64::consts::PI;

fn asset(name: &str) -> String {
    format!(
        "{}/../../sim/L0/tests/assets/opensim_gait2392/{name}",
        env!("CARGO_MANIFEST_DIR")
    )
}

fn json(name: &str) -> serde_json::Value {
    serde_json::from_str(&std::fs::read_to_string(asset(name)).unwrap()).unwrap()
}

const EPS: f64 = 0.5 * PI / 180.0;
const GATE_MM: f64 = 2.0;

#[test]
fn oracle_matches_real_opensim_knee_rom() {
    let model = parse_leg_chain(&std::fs::read_to_string(asset("gait2392.osim")).unwrap());
    let kin = Kinematics::new(&model);
    let reference = json("knee_moment_arms_opensim.json");

    println!(
        "\n===== oracle vs REAL {} (knee ROM @ neutral hip) =====",
        reference["source"].as_str().unwrap()
    );
    let mut worst = 0.0_f64;
    for m in &model.muscles {
        let rows = reference["muscles"][&m.name].as_array().unwrap();
        let (mut sse, mut maxd, mut n) = (0.0, 0.0_f64, 0.0);
        for row in rows {
            let theta = row["angle_rad"].as_f64().unwrap();
            let q = Pose::from([("knee_angle_r".to_string(), theta)]);
            let osim_mm = row["moment_arm_m"].as_f64().unwrap() * 1000.0;
            let ours_mm = kin.moment_arm(m, &q, "knee_angle_r", EPS) * 1000.0;
            let d = (osim_mm - ours_mm).abs();
            sse += d * d;
            maxd = maxd.max(d);
            n += 1.0;
        }
        let rmse = (sse / n).sqrt();
        worst = worst.max(rmse);
        println!("  {:<11} RMSE {rmse:>6.3}mm  max|Δ| {maxd:>6.3}mm", m.name);
        assert!(
            rmse < GATE_MM,
            "{}: knee-ROM oracle vs real OpenSim RMSE {rmse:.3}mm exceeds {GATE_MM}mm",
            m.name
        );
    }
    println!("worst-muscle RMSE: {worst:.3}mm\n");
}

#[test]
fn oracle_matches_real_opensim_multidof() {
    let model = parse_leg_chain(&std::fs::read_to_string(asset("gait2392.osim")).unwrap());
    let kin = Kinematics::new(&model);
    let reference = json("moment_arms_opensim.json");

    // The base pose every sample is evaluated at (several non-zero hip rotations).
    let base: Pose = reference["base_pose_rad"]
        .as_object()
        .unwrap()
        .iter()
        .map(|(k, v)| (k.clone(), v.as_f64().unwrap()))
        .collect();

    println!(
        "\n===== oracle vs REAL {} (multi-DOF base pose) =====",
        reference["source"].as_str().unwrap()
    );
    println!(
        "{:<16} {:<11} {:>8} {:>8}",
        "coord", "muscle", "RMSE", "max|Δ|"
    );

    let sweeps = reference["sweeps"].as_object().unwrap();
    let mut worst = 0.0_f64;
    for (coord, per_muscle) in sweeps {
        for m in &model.muscles {
            let rows = per_muscle[&m.name].as_array().unwrap();
            let (mut sse, mut maxd, mut n) = (0.0, 0.0_f64, 0.0);
            for row in rows {
                // The pose is the base with this coordinate overridden to the sample.
                let mut q = base.clone();
                q.insert(coord.clone(), row["value_rad"].as_f64().unwrap());
                let osim_mm = row["moment_arm_m"].as_f64().unwrap() * 1000.0;
                let ours_mm = kin.moment_arm(m, &q, coord, EPS) * 1000.0;
                let d = (osim_mm - ours_mm).abs();
                sse += d * d;
                maxd = maxd.max(d);
                n += 1.0;
            }
            let rmse = (sse / n).sqrt();
            worst = worst.max(rmse);
            println!("{coord:<16} {:<11} {rmse:>6.3}mm {maxd:>6.3}mm", m.name);
            assert!(
                rmse < GATE_MM,
                "{} about {coord}: multi-DOF oracle vs real OpenSim RMSE {rmse:.3}mm exceeds {GATE_MM}mm",
                m.name
            );
        }
    }
    println!("worst (coord,muscle) RMSE: {worst:.3}mm\n");
}

/// Read a config's per-body Vec3 factors (`[x, y, z]`, y=axial, x=z=transverse)
/// into a [`BodyParams`]. A `talus_r` factor (in the uniform config) is ignored:
/// the talus has no independent scale — its length scales via the tibia parent and
/// it carries no muscle points.
fn body_params_from_factors(factors: &serde_json::Value) -> BodyParams {
    let seg = |name: &str| -> SegmentScale {
        match factors.get(name) {
            Some(v) => {
                let a = v.as_array().unwrap();
                let (x, y, z) = (
                    a[0].as_f64().unwrap(),
                    a[1].as_f64().unwrap(),
                    a[2].as_f64().unwrap(),
                );
                assert!(
                    (x - z).abs() < 1e-12,
                    "{name}: transverse x != z ({x} vs {z})"
                );
                SegmentScale {
                    axial: y,
                    transverse: x,
                }
            }
            None => SegmentScale::IDENTITY,
        }
    };
    BodyParams {
        pelvis: seg("pelvis"),
        femur: seg("femur_r"),
        tibia: seg("tibia_r"),
    }
}

/// The scale configs the vendored JSON must carry. A drift guard: if the
/// generator's grid changes but the JSON is not regenerated (or vice versa), the
/// test must fail loudly rather than silently grade a stale/partial grid.
const EXPECTED_CONFIGS: [&str; 13] = [
    "uniform_1.137",
    "femur_axial_1.2",
    "femur_axial_0.9",
    "femur_transverse_1.3",
    "femur_transverse_0.85",
    "tibia_axial_0.9",
    "tibia_transverse_1.2",
    "tibia_transverse_0.85",
    "realistic_mix",
    // The generator's own coupled output at the sampled (0.01–0.99) percentile
    // extremes, both sexes — added in A3-PR4 so the differential oracle directly
    // grades what `AnthroSource` emits, and the whole coupled family across the
    // advertised range lands inside the T1-coverage envelope.
    "gen_male_01",
    "gen_male_99",
    "gen_female_01",
    "gen_female_99",
];

/// Tight gate for the differential oracle: the morph must track OpenSim's
/// ScaleTool to within the SAME sub-mm band as the *unscaled* cross-check
/// (`oracle_matches_real_opensim_knee_rom`, ~0.3 mm). Set ~2× the measured worst
/// (~0.37 mm) so a precision/convention regression is caught here long before it
/// would reach the loose 2 mm `GATE_MM` — and so "morphing adds ~no error" is
/// machine-checked, not just asserted in prose.
const SCALED_GATE_MM: f64 = 0.8;

#[test]
fn morph_matches_real_opensim_scaletool() {
    let template = parse_leg_chain(&std::fs::read_to_string(asset("gait2392.osim")).unwrap());
    let reference = json("scaled_moment_arms_opensim.json");
    let configs = reference["configs"].as_object().unwrap();

    // Drift guard: the JSON's config grid must match the generator's (above).
    assert_eq!(
        configs.len(),
        EXPECTED_CONFIGS.len(),
        "scaled_moment_arms_opensim.json has {} configs, expected {} — regenerate it \
         (gen_scaled_moment_arms.py) after changing the grid",
        configs.len(),
        EXPECTED_CONFIGS.len()
    );
    for name in EXPECTED_CONFIGS {
        assert!(
            configs.contains_key(name),
            "config '{name}' missing from the JSON"
        );
    }

    println!(
        "\n===== morph (realize) vs REAL {} ScaleTool (gate {SCALED_GATE_MM} mm) =====",
        reference["source"].as_str().unwrap()
    );
    let mut worst = 0.0_f64;
    for (name, cfg) in configs {
        // `realize` the SAME factors OpenSim scaled with, then grade the morphed
        // model's oracle moment arms against OpenSim's scaled model.
        let params = body_params_from_factors(&cfg["factors"]);
        let model = realize(&template, &params);
        let kin = Kinematics::new(&model);
        let mut cfg_worst = 0.0_f64;
        for m in &model.muscles {
            // Drift guard: every muscle of the model must be present in the config.
            let rows = cfg["muscles"]
                .get(&m.name)
                .and_then(|v| v.as_array())
                .unwrap_or_else(|| {
                    panic!(
                        "muscle '{}' missing in config '{name}' — regenerate JSON",
                        m.name
                    )
                });
            let (mut sse, mut maxd, mut n) = (0.0, 0.0_f64, 0.0);
            for row in rows {
                let theta = row["angle_rad"].as_f64().unwrap();
                let q = Pose::from([("knee_angle_r".to_string(), theta)]);
                let osim_mm = row["moment_arm_m"].as_f64().unwrap() * 1000.0;
                let ours_mm = kin.moment_arm(m, &q, "knee_angle_r", EPS) * 1000.0;
                let d = (osim_mm - ours_mm).abs();
                sse += d * d;
                maxd = maxd.max(d);
                n += 1.0;
            }
            let rmse = (sse / n).sqrt();
            cfg_worst = cfg_worst.max(rmse);
            assert!(
                rmse < SCALED_GATE_MM,
                "{name}/{}: morph vs real OpenSim ScaleTool RMSE {rmse:.3}mm exceeds \
                 {SCALED_GATE_MM}mm (max|Δ| {maxd:.3}mm) — scaling should add ~no error",
                m.name
            );
        }
        worst = worst.max(cfg_worst);
        println!("  {name:<22} worst-muscle RMSE {cfg_worst:>6.3}mm");
    }
    // Machine-check the headline claim: morphing tracks OpenSim within the unscaled
    // band (the per-config gate above already enforces this; this is the summary).
    assert!(
        worst < SCALED_GATE_MM,
        "worst scaled RMSE {worst:.3}mm exceeds {SCALED_GATE_MM}mm"
    );
    println!(
        "worst RMSE across all scale configs: {worst:.3}mm \
         (vs ~0.3mm unscaled — scaling adds ~no error)\n"
    );
}
