//! Write a cf-viewer assembly visualizing detected landmarks on a synthetic leg.
//!
//!   cargo run -p cf-anthro --example landmark_overlay [out_dir]
//!   cargo run -p cf-viewer -- <out_dir> --assembly
//!
//! Each STL is a separate colored assembly piece: the leg, the knee joint-line
//! band, the two epicondyle posts + the width bar, the thigh/calf girth bands,
//! and the limb axis.

use cf_anthro::{detect_landmarks, markers, synthetic::LegSpec};
use mesh_io::save_stl;
use mesh_types::Point3;
use std::f64::consts::TAU;

fn main() {
    let out = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "/tmp/knee_landmarks".to_string());
    std::fs::create_dir_all(&out).unwrap();

    let (mesh, gt) = LegSpec::default_leg().build(220, 96);
    let lm = detect_landmarks(&mesh);

    // radius at a girth (band sits just outside the surface)
    let r_at = |girth: f64| girth / TAU;
    let (kx, ky) = (lm.knee_point.x, lm.knee_point.y);

    let write = |name: &str, m: &mesh_types::IndexedMesh| {
        save_stl(m, format!("{out}/{name}"), true).unwrap();
    };

    write("0_leg.stl", &mesh);
    // Knee joint line — the prominent marker (tall band).
    write(
        "1_knee_jointline.stl",
        &markers::band(kx, ky, lm.knee_z, r_at(lm.knee_girth_m) + 0.006, 0.012, 96),
    );
    // Epicondyle width: posts at the M-L extremes + a bar across.
    write("2_epicondyle_a.stl", &markers::cube(lm.epicondyle_a, 0.007));
    write("3_epicondyle_b.stl", &markers::cube(lm.epicondyle_b, 0.007));
    write(
        "4_epicondyle_bar.stl",
        &markers::rod(lm.epicondyle_a, lm.epicondyle_b, 0.0025),
    );
    // Thigh / calf girth bands (thinner).
    write(
        "5_thigh_girth.stl",
        &markers::band(
            0.0,
            0.0,
            lm.thigh_z,
            r_at(lm.thigh_girth_m) + 0.004,
            0.006,
            96,
        ),
    );
    write(
        "6_calf_girth.stl",
        &markers::band(
            0.0,
            0.0,
            lm.calf_z,
            r_at(lm.calf_girth_m) + 0.004,
            0.006,
            96,
        ),
    );
    // Limb axis (centerline stand-in for the straight synthetic leg).
    let (z0, z1) = (lm.knee_z - lm.shank_length_m, lm.knee_z + lm.thigh_length_m);
    write(
        "7_limb_axis.stl",
        &markers::rod(Point3::new(0.0, 0.0, z0), Point3::new(0.0, 0.0, z1), 0.003),
    );

    std::fs::write(format!("{out}/landmarks.toml"), lm.to_toml_string()).unwrap();

    println!("Landmark overlay written to {out}/");
    println!(
        "  knee_z          = {:.1} mm   (truth {:.1} mm, Δ {:+.1} mm)",
        lm.knee_z * 1000.0,
        gt.knee_z * 1000.0,
        (lm.knee_z - gt.knee_z) * 1000.0
    );
    println!(
        "  epicondyle wid  = {:.1} mm   (truth {:.1} mm, Δ {:+.1} mm)",
        lm.epicondyle_width_m * 1000.0,
        gt.epicondyle_width_m * 1000.0,
        (lm.epicondyle_width_m - gt.epicondyle_width_m) * 1000.0
    );
    println!(
        "  thigh / shank   = {:.0} / {:.0} mm",
        lm.thigh_length_m * 1000.0,
        lm.shank_length_m * 1000.0
    );
    println!(
        "  girths (T/K/C)  = {:.0} / {:.0} / {:.0} mm",
        lm.thigh_girth_m * 1000.0,
        lm.knee_girth_m * 1000.0,
        lm.calf_girth_m * 1000.0
    );
    println!("\nView:  cargo run -p cf-viewer -- {out} --assembly");
}
