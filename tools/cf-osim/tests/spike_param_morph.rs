//! SPIKE (throwaway, `#[ignore]`) — the parameter/morph seam for the
//! parametric-builder-first reframe.
//! Run: cargo test -p cf-osim --test spike_param_morph -- --ignored --nocapture
//!
//! WHAT THIS RETIRES (architecture decisions in
//! `docs/msk_builder/03_phases/g1_knee_kinematics/recon.md` §5):
//!
//! * **D7/D8** — "parameters morph the IR, then emit MJCF; `CanonicalSource`
//!   (default params) is builder-first." Risk: does inserting a param→morph layer
//!   between the `Subgraph` template and `emit_coupled_knee` REGRESS the validated
//!   S1 oracle result? **Spike A** proves identity params reproduce the S1 moment
//!   arms exactly, and a uniform scale `s` scales every moment arm by exactly `s`
//!   with shape preserved (the analytic anchor: dilating a wrap-free straight-
//!   segment path by `s` scales `L`, hence `-dL/dθ`, by `s`).
//! * **D8 (randomizer)** — sampled params must emit models that load and give
//!   finite moment arms. **Spike A** load-checks a few randomized param sets.
//! * **D9/D11** — "per-segment scale rules; coupling stays symbolic through the
//!   morph." The §7 *scaled-subject* metric ASSUMES magnitude scales but SHAPE is
//!   preserved (corr ≥ 0.95). **Spike B** measures shape preservation under an
//!   anisotropic (femur≠tibia) scale — the one case that is NOT analytically a
//!   pure dilation.
//!
//! HOW IT STAYS HONEST: `BodyParams`/`realize` are prototyped HERE, in the test,
//! operating on the existing `Subgraph` IR. The morph scales geometry only — it
//! multiplies path-point locations and the *output* of the coupling/patella
//! splines, leaving the spline RELATIONSHIP (knot structure, θ-dependence) intact.
//! That is exactly "coupling stays symbolic" (D11). Grading reuses the validated
//! `emit_coupled_knee` + `coupled_moment_arm` harness (the same one the S1
//! scorecard, anchored to real OpenSim 4.6 at 0.3 mm, runs through).
//!
//! NOT a real-OpenSim check and NOT production code: it proves the morph SEAM is
//! faithful to the already-validated S1 model, and measures one design-open
//! sensitivity. cf-msk-lib's real `realize` is the build target this de-risks.

use cf_osim::emit::emit_coupled_knee;
use cf_osim::osim::{Kind, Subgraph, parse_knee_subgraph};
use cf_osim::{coupled_moment_arm, joint_id, tendon_id};
use sim_mjcf::load_model;
use std::f64::consts::PI;

const DEG: f64 = PI / 180.0;

fn osim_path() -> String {
    format!(
        "{}/../../sim/L0/tests/assets/opensim_gait2392/gait2392.osim",
        env!("CARGO_MANIFEST_DIR")
    )
}

// --- prototype of cf-msk-lib::BodyParams (the per-segment slice of it) -------

/// A per-segment scale — the only `BodyParams` field a uniform/anisotropic morph
/// needs to exercise the seam. The real `BodyParams` adds joint centers, girths,
/// etc.; those don't touch the question this spike answers.
#[derive(Clone, Copy)]
struct BodyScale {
    pelvis: f64,
    femur: f64,
    tibia: f64,
}

impl BodyScale {
    /// `CanonicalSource` defaults — the morph must be a no-op here.
    fn identity() -> Self {
        Self {
            pelvis: 1.0,
            femur: 1.0,
            tibia: 1.0,
        }
    }
    fn uniform(s: f64) -> Self {
        Self {
            pelvis: s,
            femur: s,
            tibia: s,
        }
    }
}

/// Prototype `realize(template, params) -> BodySpec`. A PURE function: scales each
/// path point's body-frame location by its segment factor, and the coupling /
/// patella spline OUTPUTS by the owning segment (knee translation = femur condyle
/// geometry → femur; patella rides the tibia frame → tibia). The spline knots and
/// θ-dependence are untouched (D11). Body-frame origins sit at the proximal joint,
/// so scaling a location IS "scale the segment about its proximal joint" — the
/// OpenSim Scale-tool convention. Moment arms are translation-invariant, so
/// `hip_in_pelvis` (pure global placement) is scaled with the femur only for
/// internal consistency; it does not affect any result here.
fn realize(sub: &Subgraph, bs: &BodyScale) -> Subgraph {
    let k_of = |body: &str| match body {
        "femur_r" => bs.femur,
        "tibia_r" => bs.tibia,
        _ => bs.pelvis, // pelvis / ground (proximal anchors)
    };

    let mut out = sub.clone();
    out.hip_in_pelvis *= bs.femur;

    // Knee coupling splines: tibial translation in the femur frame → femur scale.
    out.knee.tx.scale *= bs.femur;
    out.knee.ty.scale *= bs.femur;
    out.knee.tz.scale *= bs.femur;

    for m in &mut out.muscles {
        for p in &mut m.path {
            let k = k_of(&p.body);
            p.location *= k;
            if let Kind::Moving(s) = &mut p.kind {
                // Patella moving point: location splines ride the tibia frame.
                s.x.scale *= k;
                s.y.scale *= k;
                s.z.scale *= k;
            }
        }
    }
    out
}

// --- grading: moment-arm curve per muscle, via the validated S1 harness ------

fn sweep_angles() -> Vec<f64> {
    (0..=20).map(|i| -(i as f64) * 5.0 * DEG).collect()
}

/// Emit the coupled knee for `sub`, load it, and return (muscle, moment-arm curve
/// in mm) over the sweep — driving every coupled DOF with `sub`'s OWN (morphed)
/// splines, exactly as the S1 scorecard does.
fn moment_arm_curves(sub: &Subgraph) -> Vec<(String, Vec<f64>)> {
    let angles = sweep_angles();
    let eps = 0.5 * DEG;
    let emitted = emit_coupled_knee(sub);
    let model = load_model(&emitted.mjcf).expect("morphed coupled-knee MJCF must load");

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

fn max_abs_diff(a: &[f64], b: &[f64]) -> f64 {
    a.iter()
        .zip(b)
        .map(|(x, y)| (x - y).abs())
        .fold(0.0, f64::max)
}

/// Pearson correlation — the §7 "shape" metric (scale/offset invariant).
fn pearson(a: &[f64], b: &[f64]) -> f64 {
    let n = a.len() as f64;
    let (ma, mb) = (a.iter().sum::<f64>() / n, b.iter().sum::<f64>() / n);
    let mut sab = 0.0;
    let mut saa = 0.0;
    let mut sbb = 0.0;
    for (x, y) in a.iter().zip(b) {
        sab += (x - ma) * (y - mb);
        saa += (x - ma).powi(2);
        sbb += (y - mb).powi(2);
    }
    if saa < 1e-12 || sbb < 1e-12 {
        return 1.0; // a flat curve has no shape to distort
    }
    sab / (saa * sbb).sqrt()
}

/// Best-fit single scale of `b` onto `a` (least squares through origin) — the
/// magnitude ratio after removing shape.
fn best_fit_ratio(base: &[f64], other: &[f64]) -> f64 {
    let num: f64 = base.iter().zip(other).map(|(x, y)| x * y).sum();
    let den: f64 = base.iter().map(|x| x * x).sum();
    if den < 1e-12 { 1.0 } else { num / den }
}

#[test]
#[ignore = "spike — run with --ignored --nocapture"]
fn spike_a_morph_seam_preserves_oracle() {
    let xml = std::fs::read_to_string(osim_path()).expect("read gait2392.osim");
    let sub = parse_knee_subgraph(&xml);

    let base = moment_arm_curves(&sub);
    let ident = moment_arm_curves(&realize(&sub, &BodyScale::identity()));

    let s = 1.137; // an arbitrary non-trivial uniform scale
    let uni = moment_arm_curves(&realize(&sub, &BodyScale::uniform(s)));

    println!("\n========== SPIKE A — MORPH SEAM vs VALIDATED S1 ==========");
    println!("identity params must be a no-op; uniform s={s} must scale MA by s\n");
    println!(
        "{:<11} {:>14} {:>14} {:>10}",
        "muscle", "identity Δmax", "uniform ratio", "uni corr"
    );
    println!("{:-<52}", "");

    for ((name, b), ((_, i), (_, u))) in base.iter().zip(ident.iter().zip(uni.iter())) {
        let id_diff = max_abs_diff(b, i);
        let ratio = best_fit_ratio(b, u);
        let corr = pearson(b, u);
        println!("{name:<11} {id_diff:>11.2e}mm {ratio:>14.6} {corr:>10.6}");

        // D7/D8 — identity morph is byte-exact in result.
        assert!(
            id_diff < 1e-6,
            "{name}: identity morph changed the moment arm by {id_diff:.3e} mm (should be ~0)"
        );
        // D7 — uniform dilation scales every moment arm by EXACTLY s, shape intact.
        assert!(
            (ratio - s).abs() < 1e-4,
            "{name}: uniform scale ratio {ratio:.6} != s={s}"
        );
        assert!(
            corr > 0.99999,
            "{name}: uniform scale distorted shape (corr {corr:.6})"
        );
    }

    // D8 — randomizer: a handful of sampled per-segment scales must each emit a
    // model that loads and yields finite moment arms.
    println!("\n-- randomizer load-check (sampled per-segment scales) --");
    let samples = [
        (0.85, 1.05, 0.95),
        (1.20, 0.90, 1.10),
        (0.70, 1.30, 0.80),
        (1.15, 1.15, 1.15),
    ];
    for (i, &(pelvis, femur, tibia)) in samples.iter().enumerate() {
        let bs = BodyScale {
            pelvis,
            femur,
            tibia,
        };
        let curves = moment_arm_curves(&realize(&sub, &bs));
        let all_finite = curves.iter().all(|(_, c)| c.iter().all(|v| v.is_finite()));
        assert!(all_finite, "sample {i} produced a non-finite moment arm");
        let span = curves
            .iter()
            .map(|(_, c)| c.iter().cloned().fold(f64::MIN, f64::max))
            .fold(f64::MIN, f64::max);
        println!(
            "  sample {i}: scales(p={pelvis},f={femur},t={tibia}) → loads ✓, finite ✓, max|MA|≈{span:.1}mm"
        );
    }

    println!("\nSEAM IS FAITHFUL: identity reproduces S1 to <1e-6 mm; uniform scale");
    println!("is an exact dilation; randomized params load + stay finite.");
    println!("==========================================================\n");
}

#[test]
#[ignore = "spike — run with --ignored --nocapture"]
fn spike_b_anisotropic_scale_preserves_shape() {
    let xml = std::fs::read_to_string(osim_path()).expect("read gait2392.osim");
    let sub = parse_knee_subgraph(&xml);

    let base = moment_arm_curves(&sub);

    // The genuinely-uncertain case: femur and tibia scale DIFFERENTLY, so the
    // morph is NOT a pure dilation. The §7 scaled-subject metric assumes the curve
    // SHAPE survives (corr ≥ 0.95) while magnitude scales. Measure it.
    let cases = [
        (
            "femur +20%",
            BodyScale {
                pelvis: 1.0,
                femur: 1.20,
                tibia: 1.0,
            },
        ),
        (
            "tibia +20%",
            BodyScale {
                pelvis: 1.0,
                femur: 1.0,
                tibia: 1.20,
            },
        ),
        (
            "femur +15% / tibia -10%",
            BodyScale {
                pelvis: 1.0,
                femur: 1.15,
                tibia: 0.90,
            },
        ),
        (
            "realistic subject (f1.08/t0.94)",
            BodyScale {
                pelvis: 1.0,
                femur: 1.08,
                tibia: 0.94,
            },
        ),
    ];

    println!("\n===== SPIKE B — ANISOTROPIC SCALE, SHAPE PRESERVATION =====");
    println!("§7 scaled-subject assumes shape corr ≥ 0.95 (magnitude may scale)\n");

    let mut worst = 1.0_f64;
    for (label, bs) in &cases {
        let scaled = moment_arm_curves(&realize(&sub, bs));
        println!("{label}:");
        println!(
            "  {:<11} {:>10} {:>12} {:>14}",
            "muscle", "shape corr", "fit ratio", "shape-resid mm"
        );
        for ((name, b), (_, s)) in base.iter().zip(scaled.iter()) {
            let corr = pearson(b, s);
            let ratio = best_fit_ratio(b, s);
            // residual after removing the best-fit scale = pure shape error
            let resid = {
                let n = b.len() as f64;
                (b.iter()
                    .zip(s)
                    .map(|(x, y)| (y - ratio * x).powi(2))
                    .sum::<f64>()
                    / n)
                    .sqrt()
            };
            let flag = if corr >= 0.95 { "ok" } else { "INVESTIGATE" };
            println!("  {name:<11} {corr:>10.4} {ratio:>12.4} {resid:>11.3} mm  {flag}");
            worst = worst.min(corr);
        }
        println!();
    }

    println!("worst-case shape correlation across all cases/muscles: {worst:.4}");
    if worst >= 0.95 {
        println!("→ §7's corr≥0.95 scaled-subject assumption HOLDS under naive");
        println!("  per-segment scaling. ScaleRule design can proceed on this basis.");
    } else {
        println!("→ naive per-segment scaling drops below 0.95 for some muscle —");
        println!("  the ScaleRule (D9) needs care there; flag for the S3 design.");
    }
    println!("(Exploratory: no real scaled-subject ground truth exists — this");
    println!(" measures internal shape stability of the morph, not accuracy.)");
    println!("===========================================================\n");
}
