//! S2 validation — landmark detection vs known ground truth on synthetic legs.
//!
//! Real leg scans aren't available yet, so the detector is validated against
//! synthetic limbs whose knee/width/lengths we set exactly. Exit criteria
//! (recon S2): knee-line ±10 mm, epicondyle width ±8 mm, segment length ±5%.
//! Run with `--nocapture` to see the scorecard.
//!
//! CAVEAT (honesty): the generator places the knee *at* the area minimum and the
//! detector *finds* the area minimum, so this proves the numerics (search,
//! parabola refine, noise robustness) — NOT the anatomical premise that the
//! area minimum is the knee joint line. That awaits real scans + an independent
//! landmark ground truth (see the crate-level "Real-scan readiness" notes).

use cf_anthro::synthetic::LegSpec;
use cf_anthro::{DetectError, detect_landmarks};
use mesh_types::IndexedMesh;

struct Case {
    name: &'static str,
    spec: LegSpec,
    /// Vertex-jitter amplitude (m) to mimic scan noise.
    noise_m: f64,
}

fn cases() -> Vec<Case> {
    let base = LegSpec::default_leg();
    vec![
        Case {
            name: "default",
            spec: LegSpec::default_leg(),
            noise_m: 0.0,
        },
        Case {
            name: "small-short",
            spec: base.scaled(0.82, 0.90),
            noise_m: 0.0,
        },
        Case {
            name: "large-long",
            spec: base.scaled(1.18, 1.08),
            noise_m: 0.0,
        },
        // ~0.8mm of scan-like noise — exercises the smoothing + parabola refine.
        Case {
            name: "noisy",
            spec: LegSpec::default_leg(),
            noise_m: 0.0008,
        },
    ]
}

/// Deterministic per-vertex jitter (splitmix64-style hash → no RNG dependency,
/// reproducible across runs).
fn jitter(mesh: &mut IndexedMesh, amp: f64) {
    let rand = |i: usize, k: u64| -> f64 {
        let mut x = (i as u64)
            .wrapping_mul(0x9E37_79B9_7F4A_7C15)
            .wrapping_add(k.wrapping_mul(0xD1B5_4A32_D192_ED03));
        x = (x ^ (x >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        x = (x ^ (x >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        ((x >> 11) as f64 / (1u64 << 53) as f64) * 2.0 - 1.0
    };
    for (i, v) in mesh.vertices.iter_mut().enumerate() {
        v.x += amp * rand(i, 1);
        v.y += amp * rand(i, 2);
        v.z += amp * rand(i, 3);
    }
}

#[test]
fn detects_landmarks_within_tolerance() {
    println!("\n=============== S2 LANDMARK SCORECARD (synthetic) ===============");
    println!(
        "{:<12} {:>10} {:>10} | {:>10} {:>10} {:>10}",
        "case", "knee Δmm", "epicond Δmm", "thigh %", "shank %", "verdict"
    );
    println!("{:-<64}", "");

    for c in cases() {
        let (mut mesh, gt) = c.spec.build(160, 64);
        if c.noise_m > 0.0 {
            jitter(&mut mesh, c.noise_m);
        }
        let lm =
            detect_landmarks(&mesh).expect("detection should succeed on a clean synthetic leg");

        let knee_dmm = (lm.knee_z - gt.knee_z) * 1000.0;
        let epi_dmm = (lm.epicondyle_width_m - gt.epicondyle_width_m) * 1000.0;
        let thigh_pct = (lm.thigh_length_m - gt.thigh_length_m) / gt.thigh_length_m * 100.0;
        let shank_pct = (lm.shank_length_m - gt.shank_length_m) / gt.shank_length_m * 100.0;

        let pass = knee_dmm.abs() < 10.0
            && epi_dmm.abs() < 8.0
            && thigh_pct.abs() < 5.0
            && shank_pct.abs() < 5.0;
        println!(
            "{:<12} {:>+9.2} {:>+10.2} | {:>+9.1} {:>+9.1} {:>10}",
            c.name,
            knee_dmm,
            epi_dmm,
            thigh_pct,
            shank_pct,
            if pass { "PASS" } else { "FAIL" }
        );

        assert!(
            knee_dmm.abs() < 10.0,
            "{}: knee off by {knee_dmm:.2}mm (>10mm)",
            c.name
        );
        assert!(
            epi_dmm.abs() < 8.0,
            "{}: epicondyle width off by {epi_dmm:.2}mm (>8mm)",
            c.name
        );
        assert!(
            thigh_pct.abs() < 5.0,
            "{}: thigh length off {thigh_pct:.1}% (>5%)",
            c.name
        );
        assert!(
            shank_pct.abs() < 5.0,
            "{}: shank length off {shank_pct:.1}% (>5%)",
            c.name
        );
    }
    println!("{:-<64}", "");
    println!("all synthetic legs within tolerance (knee ±10mm, width ±8mm, length ±5%)");
    println!("================================================================\n");
}

/// A stray disconnected island (scan noise / un-cleaned blob) near the knee must
/// NOT corrupt the area profile — the detector tracks the largest contour, so
/// the small island is ignored and the knee is still found.
#[test]
fn ignores_noise_island() {
    let (mut mesh, gt) = LegSpec::default_leg().build(160, 64);
    // ~1 cm cube floating 15 cm off-axis at knee height — a separate contour.
    let island = cf_anthro::markers::cube(mesh_types::Point3::new(0.15, 0.0, gt.knee_z), 0.005);
    let base = mesh.vertices.len() as u32;
    mesh.vertices.extend(island.vertices.iter().copied());
    mesh.faces.extend(
        island
            .faces
            .iter()
            .map(|f| [f[0] + base, f[1] + base, f[2] + base]),
    );

    let lm = detect_landmarks(&mesh).expect("a small island should be ignored, not fatal");
    let knee_dmm = (lm.knee_z - gt.knee_z) * 1000.0;
    assert!(
        knee_dmm.abs() < 10.0,
        "knee off by {knee_dmm:.2}mm with a noise island present — largest-contour failed"
    );
}

// --- Error paths: detection degrades gracefully (bails, never panics) ---

// (TooFewGirthMaxima / NoKneeMinimum need a profile with no clean bulges or no
// interior minimum — awkward to construct deterministically from the marker
// primitives, so they're left to real-scan fixtures; EmptyMesh and the
// MultiContourSection bail below are the cheaply-reachable variants.)

#[test]
fn empty_mesh_errors() {
    let err = detect_landmarks(&IndexedMesh::default()).unwrap_err();
    assert!(matches!(err, DetectError::EmptyMesh), "got {err:?}");
}

#[test]
fn rival_contour_at_knee_bails_multicontour() {
    // A LARGE off-axis blob at the knee (rivals the limb's area, unlike the
    // small island above) is genuinely ambiguous → bail rather than guess.
    let (mut mesh, gt) = LegSpec::default_leg().build(160, 64);
    let blob = cf_anthro::markers::cube(mesh_types::Point3::new(0.18, 0.0, gt.knee_z), 0.030);
    let base = mesh.vertices.len() as u32;
    mesh.vertices.extend(blob.vertices.iter().copied());
    mesh.faces.extend(
        blob.faces
            .iter()
            .map(|f| [f[0] + base, f[1] + base, f[2] + base]),
    );
    let err = detect_landmarks(&mesh).unwrap_err();
    assert!(
        matches!(err, DetectError::MultiContourSection(_)),
        "got {err:?}"
    );
}
