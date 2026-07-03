//! D4 — bistable p-bit: printable curved-beam sweep generator.
//!
//! Generates a sweep of clamped-clamped pre-curved bistable beams as STL and
//! reports geometry + mass properties + a bistability prediction. This is the
//! geometry half of gate **G1** of the D4 sim-to-real arc
//! (`docs/thermo_computing/03_phases/d4_physical_pbit/recon.md`).
//!
//! ## The device
//!
//! A thin beam clamped at both ends, pre-shaped to the first clamped-clamped
//! buckling mode `y(s) = (h/2)·(1 − cos 2πs)` for `s ∈ [0,1]` — zero slope and
//! displacement at both clamps, rise `h` at midspan. This is the canonical
//! bistable mechanism (Qiu, Lang & Slocum 2004): above a rise/thickness ratio
//! of ≈ 2.31 it has two stable states (the as-fabricated curve and its snapped
//! mirror), so the midpoint lateral position is a two-well order parameter — a
//! mechanical p-bit. The buckling/snap direction is **+Y**; span is along **X**;
//! width is along **Z**.
//!
//! Two integral clamp posts plus a base slab hold the ends at fixed separation
//! and give a flat mounting face, so each candidate prints as one part with no
//! assembly. The beam is free to invert into the clearance below it.
//!
//! ## Usage
//!
//! ```text
//! cargo run -p example-thermo-bistable-pbit --release -- [OUT_DIR]
//! ```
//!
//! `OUT_DIR` defaults to `./pbit_stl`. Each `(rise, thickness)` combination is
//! written as `pbit_h<rise×10>_t<thick×10>.stl` (binary STL, millimetres).

// Numeric research code: index/dimension casts and the threshold round-trip are
// intentional and benign. Matches the house allow-block used across the thermo
// experiments (e.g. `sim/L0/rl-baselines/tests/ising_chain.rs`).
#![allow(clippy::cast_precision_loss, clippy::cast_possible_truncation)]

use anyhow::{Context, Result};
use cf_design::Solid;
use cf_design::mechanism::mass::mass_properties;
use mesh_io::save_stl;
use nalgebra::Vector3;
use std::path::PathBuf;

/// First-mode rise/thickness bistability threshold for a clamped-clamped
/// pre-curved beam (Qiu, Lang & Slocum 2004). Above this ratio the beam has a
/// second stable (snapped) equilibrium; below it, it springs back.
const BISTABLE_HT_THRESHOLD: f64 = 2.31;

/// PLA density (kg/m³) for the mass estimate. Geometry is in millimetres.
const PLA_DENSITY: f64 = 1240.0;

/// Parametric geometry of one bistable beam (all lengths in mm).
struct BeamSpec {
    /// Clamp-to-clamp span `L` along X.
    span: f64,
    /// Midspan pre-curve rise `h` (snap direction, +Y).
    rise: f64,
    /// Beam thickness `t` in the snap direction.
    thickness: f64,
    /// Beam width along Z.
    width: f64,
    /// Clamp-post footprint along X at each end.
    post_w: f64,
    /// Clamp-post height below the beam ends (Y).
    post_h: f64,
    /// Base-slab thickness (Y).
    base_t: f64,
    /// Number of cuboid segments approximating the curved beam.
    n_seg: usize,
}

/// First clamped-clamped buckling mode shape, `s ∈ [0, 1]` → lateral offset (mm).
fn centerline_y(s: f64, rise: f64) -> f64 {
    0.5 * rise * (1.0 - (2.0 * std::f64::consts::PI * s).cos())
}

/// Compose the beam + integral posts + base into a single printable solid.
///
/// Built by unioning the base slab, two end posts, and a chain of overlapping
/// thin cuboids that track the cosine centerline. The curve is shallow (`h ≪ L`)
/// so axis-aligned segments approximate it to well under print resolution; plain
/// (non-smooth) unions keep the beam thickness crisp.
fn build_beam(spec: &BeamSpec) -> Solid {
    let l = spec.span;
    let half_w_z = spec.width * 0.5;

    // Base slab spanning both posts: fixes end separation + mounting face.
    let base_y = -spec.post_h - spec.base_t * 0.5;
    let mut solid = Solid::cuboid(Vector3::new(
        l * 0.5 + spec.post_w,
        spec.base_t * 0.5,
        half_w_z + 1.0,
    ))
    .translate(Vector3::new(l * 0.5, base_y, 0.0));

    // Two clamp posts, from the base up to the beam ends at y = 0.
    for &x_end in &[0.0_f64, l] {
        let post = Solid::cuboid(Vector3::new(
            spec.post_w * 0.5,
            spec.post_h * 0.5,
            half_w_z + 1.0,
        ))
        .translate(Vector3::new(x_end, -spec.post_h * 0.5, 0.0));
        solid = solid.union(post);
    }

    // Curved beam: overlapping thin cuboids along the cosine centerline.
    let dx = l / spec.n_seg as f64;
    let seg_hx = dx * 0.75; // overlap neighbours so the sweep stays continuous
    for i in 0..=spec.n_seg {
        let s = i as f64 / spec.n_seg as f64;
        let seg = Solid::cuboid(Vector3::new(seg_hx, spec.thickness * 0.5, half_w_z))
            .translate(Vector3::new(s * l, centerline_y(s, spec.rise), 0.0));
        solid = solid.union(seg);
    }

    solid
}

fn main() -> Result<()> {
    let out_dir = std::env::args()
        .nth(1)
        .map_or_else(|| PathBuf::from("pbit_stl"), PathBuf::from);
    std::fs::create_dir_all(&out_dir)
        .with_context(|| format!("create output dir {}", out_dir.display()))?;

    // Fixed frame; sweep rise × thickness to bracket the h/t ≈ 2.31 threshold.
    let span = 30.0;
    let width = 6.0;
    let post_w = 6.0;
    let post_h = 8.0;
    let base_t = 3.0;
    let n_seg = 96;
    let mesh_tol = 0.25; // mm
    let mass_cell = 0.5; // mm

    let rises = [1.5_f64, 2.5, 3.5];
    let thicknesses = [0.8_f64, 1.0, 1.2];

    println!("D4 bistable p-bit sweep — clamped-clamped pre-curved beam");
    println!(
        "span={span}mm width={width}mm  |  predicted bistable when h/t > {BISTABLE_HT_THRESHOLD}"
    );
    println!();
    println!(
        "{:<24} {:>5} {:>6} {:>6} {:>9} {:>8} {:>10} {:>9}",
        "file", "rise", "thick", "h/t", "bistable", "mass_g", "verts", "x0_est"
    );

    for &rise in &rises {
        for &t in &thicknesses {
            let spec = BeamSpec {
                span,
                rise,
                thickness: t,
                width,
                post_w,
                post_h,
                base_t,
                n_seg,
            };
            let solid = build_beam(&spec);
            let mesh = solid.mesh(mesh_tol);
            let verts = mesh.geometry.positions().len();

            let name = format!(
                "pbit_h{:02}_t{:02}.stl",
                (rise * 10.0).round() as i64,
                (t * 10.0).round() as i64
            );
            let path = out_dir.join(&name);
            save_stl(&mesh.geometry, &path, true)
                .with_context(|| format!("write STL {}", path.display()))?;

            let ht = rise / t;
            let bistable = if ht > BISTABLE_HT_THRESHOLD {
                "yes"
            } else {
                "no"
            };
            let mass_g = mass_properties(&solid, PLA_DENSITY, mass_cell)
                .map_or(f64::NAN, |mp| mp.mass * 1000.0);
            // Well half-separation estimate: the midpoint travels ~rise either
            // side of flat between the two stable states, so x0 ≈ rise (mm).
            let x0_est = rise;

            println!(
                "{name:<24} {rise:>5.1} {t:>6.1} {ht:>6.2} {bistable:>9} {mass_g:>8.2} {verts:>10} {x0_est:>9.2}"
            );
        }
    }

    println!("\nSTLs written to {}", out_dir.display());
    println!("Print the h/t≈2–3 candidates and confirm two stable states by hand (G2).");
    Ok(())
}
