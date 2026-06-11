//! S0 shell-texture spike — de-risk the bonded/texture arc
//! (see `docs/CF_CAST_BONDED_INPLACE_TEXTURE_RECON.md` §7 S0).
//!
//! The canal feature field has only ever wrapped the layer-0 *plug*. The arc
//! wants it on a layer *body*'s outer surface (exterior / inter-layer ridges).
//! `build_canal_plug` wraps any base `Solid`, so this probe applies it to a
//! synthetic cylindrical shell body and asks the two questions that gate the
//! ship:
//!
//! - **Q-survives** — does an axisymmetric ring survive marching cubes at the
//!   production 0.5 mm cell (and how badly does a coarse 1.5 mm cell smear it)?
//!   Decides the cup mesh resolution exterior ridges need.
//! - **Q-undercut** — how deep is the radial undercut the rings create (R2,
//!   demolding)? A 2-piece cup pulls off radially; the ring depth *is* the
//!   undercut a cured layer must flex past to release.
//!
//! Self-contained (synthetic cylinder, no `~/scans` dependency). `#[ignore]`d
//! — a diagnostic scaffold, run on demand:
//! `cargo test -p cf-cast --test s0_shell_texture_probe -- --ignored --nocapture`

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::print_stdout)]

use cf_cast::{CanalSpec, RingSpec, build_canal_plug};
use cf_design::Solid;
use mesh_offset::{MarchingCubesConfig, ScalarGrid, marching_cubes};
use mesh_types::IndexedMesh;
use nalgebra::{Point3, Vector3};

/// MC grid padding (matches `mesher.rs`'s `GRID_PADDING_CELLS`).
const GRID_PADDING_CELLS: usize = 2;

/// Shell body: a Z-aligned cylinder, radius 10 mm, half-height 40 mm
/// (z ∈ [-40, 40] mm) — a stand-in for one layer's outer body.
const SHELL_RADIUS_M: f64 = 0.010;
const SHELL_HALF_HEIGHT_M: f64 = 0.040;
/// Ring inward-pinch depth (m) — the exterior ridge amplitude under test.
const RING_DEPTH_M: f64 = 0.002;

/// An axisymmetric ring spec — the "even exterior ridge" mechanism (the
/// canal's `rings`, which are axisymmetric, unlike the one-sided texture).
/// One-sided features (texture / D-section / suction) are zeroed so the
/// probe isolates the symmetric ridge case.
fn ring_spec() -> CanalSpec {
    CanalSpec {
        rings: vec![
            RingSpec {
                center_frac: 0.30,
                depth_m: RING_DEPTH_M,
                half_width_frac: 0.04,
            },
            RingSpec {
                center_frac: 0.50,
                depth_m: RING_DEPTH_M,
                half_width_frac: 0.04,
            },
            RingSpec {
                center_frac: 0.70,
                depth_m: RING_DEPTH_M,
                half_width_frac: 0.04,
            },
        ],
        texture_amp_m: 0.0,
        texture_pitch_m: 0.008,
        texture_zone: (0.0, 1.0),
        dsection_depth_m: 0.0,
        dsection_zone: (0.0, 1.0),
        suction_bulge_m: 0.0,
        suction_start_frac: 1.0,
        frenulum_dir: Vector3::new(0.0, 1.0, 0.0),
        plug_mesh_cell_size_m: 0.0005,
    }
}

/// Mesh `solid` at `cell_size_m` via uniform MC (meters). Replicates the
/// production `solid_to_mm_mesh` sampling loop so the probe doesn't widen the
/// surface.
fn mesh_uniform(solid: &Solid, cell_size_m: f64) -> IndexedMesh {
    let bounds = solid.bounds().expect("finite bounds");
    let mut grid = ScalarGrid::from_bounds(bounds.min, bounds.max, cell_size_m, GRID_PADDING_CELLS);
    let (nx, ny, nz) = grid.dimensions();
    for iz in 0..nz {
        for iy in 0..ny {
            for ix in 0..nx {
                let p = grid.position(ix, iy, iz);
                grid.set(ix, iy, iz, solid.evaluate(&p));
            }
        }
    }
    marching_cubes(&grid, &MarchingCubesConfig::default())
}

/// Radial-modulation metric over the cylindrical mid-section (excludes the
/// hemispherical-ish end caps where r naturally falls off). Returns
/// `(max_r, min_r)` in meters: `max_r` ≈ the nominal radius between rings,
/// `min_r` ≈ the radius at a ring crest. `max_r - min_r` is the achieved ring
/// depth = the radial undercut a cup half must clear.
fn radial_modulation(mesh: &IndexedMesh) -> (f64, f64) {
    let z_lim = SHELL_HALF_HEIGHT_M * 0.8; // stay clear of the caps
    let mut max_r = f64::MIN;
    let mut min_r = f64::MAX;
    for v in &mesh.vertices {
        if v.z.abs() > z_lim {
            continue;
        }
        let r = v.x.hypot(v.y);
        max_r = max_r.max(r);
        min_r = min_r.min(r);
    }
    (max_r, min_r)
}

#[test]
#[ignore = "S0 diagnostic spike — run with --ignored --nocapture"]
fn s0_shell_texture_ring_survives_and_bounds_undercut() {
    let body = Solid::cylinder(SHELL_RADIUS_M, SHELL_HALF_HEIGHT_M);
    let centerline = vec![
        Point3::new(0.0, 0.0, -SHELL_HALF_HEIGHT_M),
        Point3::new(0.0, 0.0, SHELL_HALF_HEIGHT_M),
    ];
    let spec = ring_spec();
    let textured = build_canal_plug(&body, &centerline, None, &spec);

    println!("\n=== S0 shell-texture probe ===");
    println!(
        "  body: cylinder r = {:.1} mm, L = {:.0} mm; ring depth (spec) = {:.1} mm",
        SHELL_RADIUS_M * 1e3,
        SHELL_HALF_HEIGHT_M * 2e3,
        RING_DEPTH_M * 1e3,
    );

    // Baseline: the un-textured body should show ~no radial modulation.
    let base_mesh = mesh_uniform(&body, 0.0005);
    let (base_max, base_min) = radial_modulation(&base_mesh);
    println!(
        "  base body  @0.5mm: r ∈ [{:.2}, {:.2}] mm, modulation {:.2} mm (≈0 expected)",
        base_min * 1e3,
        base_max * 1e3,
        (base_max - base_min) * 1e3,
    );

    for cell in [0.0005, 0.0015] {
        let mesh = mesh_uniform(&textured, cell);
        let (mx, mn) = radial_modulation(&mesh);
        let achieved = mx - mn;
        let fraction = achieved / RING_DEPTH_M;
        println!(
            "  textured   @{:.1}mm: r ∈ [{:.2}, {:.2}] mm, ring depth achieved {:.2} mm ({:.0}% of spec), {} verts",
            cell * 1e3,
            mn * 1e3,
            mx * 1e3,
            achieved * 1e3,
            fraction * 100.0,
            mesh.vertices.len(),
        );
    }

    // Q-survives gate: at the production 0.5 mm cell the ring must recover to
    // at least 60% of its specified depth (the canal probe found 0.5 mm is the
    // resolution sub-2 mm features need; below that they smear).
    let fine = mesh_uniform(&textured, 0.0005);
    let (mx, mn) = radial_modulation(&fine);
    let achieved = mx - mn;
    assert!(
        achieved >= 0.6 * RING_DEPTH_M,
        "ring smeared at 0.5 mm: achieved {:.2} mm of {:.2} mm spec",
        achieved * 1e3,
        RING_DEPTH_M * 1e3,
    );

    // Q-undercut (R2): report the undercut vs a soft-silicone demold bound.
    // Ecoflex 00-30 elongates >800%; a radial undercut that is a small
    // fraction of the part radius releases by flexing. Flag if a ring exceeds
    // ~30% of the radius (the regime where a rigid cup half would tear it).
    let undercut_fraction = achieved / SHELL_RADIUS_M;
    println!(
        "  undercut: {:.2} mm = {:.0}% of the {:.1} mm radius — {} for soft silicone",
        achieved * 1e3,
        undercut_fraction * 100.0,
        SHELL_RADIUS_M * 1e3,
        if undercut_fraction < 0.30 {
            "demoldable"
        } else {
            "TOO DEEP (rigid cup would tear it)"
        },
    );
    println!("=== end S0 ===\n");
}
