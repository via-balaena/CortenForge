//! Tier-3 validation of the A3 anthropometric generator (`cf_msk_lib::AnthroSource`).
//!
//! A *dialed* body has no real subject, so no moment-arm oracle (unlike the morph
//! *machinery*, which A3-PR2's differential oracle grades against OpenSim's
//! ScaleTool). The generator's **parameter-choice** layer is therefore validated by
//! the recon's three-tier pyramid Tier 3:
//!
//! * **Plausibility** — across the percentile range, both sexes, the realized
//!   femur/tibia axial lengths stay physiological and ordered (taller percentile →
//!   longer; female 50th < male 50th). On the REAL gait2392 template.
//! * **Shape-correlation** — a dialed body's knee moment-arm *curves* stay
//!   correlated with the canonical (50th-male = template) across the percentile
//!   range. The **default** generator (girth tracks stature, `AnthroSource::new`)
//!   clears the recon §7 bar (≥ 0.95) for all muscles/percentiles/both sexes:
//!   proportional scaling changes magnitudes, not curve shape. An **extreme
//!   decoupled** build (short+stocky / tall+lean, stature and girth percentiles far
//!   apart) is the documented boundary — the most girth-sensitive hamstring dips to
//!   ~0.87, because a large transverse-only offset against an opposite length change
//!   does move the curve shape (the same effect PR1 measured: girth is ~20–30%, not
//!   second-order). We assert the §7 bar for the realistic family and only a loose
//!   sanity floor (reported) for extreme mismatches.
//!
//! (Internal consistency — reference⇒identity, monotonicity, round-trip — is unit-
//! tested in `cf-msk-lib`. This file adds the oracle-based shape check that needs
//! `cf-osim`'s `Kinematics`.) Honesty: this validates that the generator produces a
//! plausible, shape-consistent *family*, NOT that any member is a specific person.

use cf_msk_lib::anthro::{AnthroSource, Sex};
use cf_msk_lib::{ParamSource, realize};
use cf_osim::oracle::{Kinematics, Pose};
use cf_osim::parse_leg_chain;
use std::f64::consts::PI;

const EPS: f64 = 0.5 * PI / 180.0;

fn template() -> cf_msk_lib::Model {
    let path = format!(
        "{}/../../sim/L0/tests/assets/opensim_gait2392/gait2392.osim",
        env!("CARGO_MANIFEST_DIR")
    );
    parse_leg_chain(&std::fs::read_to_string(path).expect("read gait2392.osim"))
}

/// Knee moment-arm curve (mm) per muscle, via the oracle, for a model.
fn curves(model: &cf_msk_lib::Model) -> Vec<(String, Vec<f64>)> {
    let kin = Kinematics::new(model);
    let angles: Vec<f64> = (0..=20).map(|i| -(i as f64) * 5.0 * PI / 180.0).collect();
    model
        .muscles
        .iter()
        .map(|m| {
            let c = angles
                .iter()
                .map(|&th| {
                    let q = Pose::from([("knee_angle_r".to_string(), th)]);
                    kin.moment_arm(m, &q, "knee_angle_r", EPS) * 1000.0
                })
                .collect();
            (m.name.clone(), c)
        })
        .collect()
}

/// Pearson correlation of two equal-length curves.
fn pearson(a: &[f64], b: &[f64]) -> f64 {
    let n = a.len() as f64;
    let (ma, mb) = (a.iter().sum::<f64>() / n, b.iter().sum::<f64>() / n);
    let mut sab = 0.0;
    let mut saa = 0.0;
    let mut sbb = 0.0;
    for (&x, &y) in a.iter().zip(b) {
        sab += (x - ma) * (y - mb);
        saa += (x - ma).powi(2);
        sbb += (y - mb).powi(2);
    }
    sab / (saa.sqrt() * sbb.sqrt())
}

#[test]
fn dialed_lengths_are_plausible_and_ordered() {
    let t = template();
    let femur = |s: Sex, p: f64| {
        realize(&t, &AnthroSource::new(s, p).params(&t)).segment_axial_length("femur_r", "tibia_r")
    };
    let tibia = |s: Sex, p: f64| {
        realize(&t, &AnthroSource::new(s, p).params(&t)).segment_axial_length("tibia_r", "talus_r")
    };

    // Ordering: taller percentile longer; female 50th shorter than male 50th.
    assert!(femur(Sex::Male, 0.95) > femur(Sex::Male, 0.05));
    assert!(femur(Sex::Female, 0.5) < femur(Sex::Male, 0.5));
    // Reference (male 50th) reproduces the template length.
    assert!((femur(Sex::Male, 0.5) - t.segment_axial_length("femur_r", "tibia_r")).abs() < 1e-9);

    // Plausibility across the range, both sexes (real gait2392 proportions).
    for &p in &[0.01, 0.25, 0.5, 0.75, 0.99] {
        for s in [Sex::Male, Sex::Female] {
            let (f, ti) = (femur(s, p), tibia(s, p));
            assert!(
                (0.34..0.55).contains(&f),
                "femur {f} implausible at {s:?}/{p}"
            );
            assert!(
                (0.32..0.52).contains(&ti),
                "tibia {ti} implausible at {s:?}/{p}"
            );
        }
    }
}

#[test]
fn dialed_moment_arm_shape_stays_correlated() {
    let t = template();
    // Canonical = the reference percentile (50th-male), which == the template.
    let canonical = curves(&realize(&t, &AnthroSource::new(Sex::Male, 0.5).params(&t)));

    let worst_corr = |srcs: &[AnthroSource]| -> f64 {
        let mut worst = 1.0_f64;
        for src in srcs {
            let c = curves(&realize(&t, &src.params(&t)));
            for ((_, base), (_, d)) in canonical.iter().zip(&c) {
                worst = worst.min(pearson(base, d));
            }
        }
        worst
    };

    // DEFAULT family: girth tracks stature (`new`), full percentile range, both
    // sexes. The realistic dial — must clear the §7 bar (≥ 0.95) for every muscle.
    let realistic = [
        AnthroSource::new(Sex::Male, 0.01),
        AnthroSource::new(Sex::Male, 0.99),
        AnthroSource::new(Sex::Female, 0.05),
        AnthroSource::new(Sex::Female, 0.50),
        AnthroSource::new(Sex::Female, 0.95),
    ];
    let realistic_worst = worst_corr(&realistic);
    println!("realistic (coupled) worst moment-arm shape correlation: {realistic_worst:.4}");
    assert!(
        realistic_worst >= 0.95,
        "realistic dialing broke moment-arm shape (corr {realistic_worst:.4} < 0.95)"
    );

    // EXTREME decoupled builds (short+stocky / tall+lean): the documented boundary,
    // REPORTED not gated at a quality bar. A large transverse-only offset combined
    // with an opposite length change moves the most girth-sensitive hamstring's
    // curve shape — empirically down to ~0.87 corr (tall+lean is worst). We keep
    // only a loose sanity floor (≥ 0.80) to catch a gross regression; the §7 bar is
    // asserted for the realistic family above, not here.
    let extreme = [
        AnthroSource::new(Sex::Female, 0.10).with_girth_percentile(0.90),
        AnthroSource::new(Sex::Male, 0.90).with_girth_percentile(0.05),
    ];
    let extreme_worst = worst_corr(&extreme);
    println!(
        "extreme decoupled worst moment-arm shape correlation: {extreme_worst:.4} (boundary, reported)"
    );
    assert!(
        extreme_worst >= 0.80,
        "extreme decoupled dialing fell below the sanity floor (corr {extreme_worst:.4} < 0.80)"
    );
}
