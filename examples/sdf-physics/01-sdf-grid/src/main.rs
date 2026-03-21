//! SDF Physics 01 — SDF Grid
//!
//! Pure cf-design, no simulation. Creates a `Solid`, builds an `SdfGrid` from
//! it — using the same resolution and expansion that `to_model()` uses — and
//! verifies the grid faithfully represents the implicit surface.
//!
//! Uses `Solid::sphere(5.0)`: the exact shape that steps 02+ will simulate.
//! Uses 1.0 mm cell size: the exact resolution that `to_model()` will use.
//!
//! Pass criteria:
//! - `SdfGrid` is constructed without panic
//! - Grid dimensions and cell count are physically reasonable
//! - Min distance is negative (interior exists), max is positive (exterior exists)
//! - Accuracy: grid agrees with `solid.evaluate()` at sampled points
//! - Visual mesh looks correct when orbiting
//!
//! New concept: Solid → `SdfGrid` construction
//!
//! Run with: `cargo run -p example-sdf-01-sdf-grid --release`

#![allow(
    clippy::needless_pass_by_value,
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss
)]

use bevy::prelude::*;
use cf_design::Solid;
use cf_geometry::SdfGrid;
use nalgebra::Point3;
use sim_bevy::camera::OrbitCameraPlugin;
use sim_bevy::mesh::spawn_design_mesh;
use sim_bevy::scene::ExampleScene;

/// SDF grid resolution in mm — matches what `to_model(1.0, _)` uses.
const SDF_RESOLUTION: f64 = 1.0;

/// Visual mesh tolerance in mm.
const MESH_TOLERANCE: f64 = 0.3;

fn main() {
    let solid = Solid::sphere(5.0);

    let Some(bounds) = solid.bounds() else {
        eprintln!("FAIL: solid has no bounds");
        return;
    };

    // Build SdfGrid the same way to_model() does:
    // expand bounds by 2×resolution, then sample with from_fn.
    let sdf = build_sdf_grid(&solid, &bounds, SDF_RESOLUTION);

    eprintln!();
    print_grid_info(&bounds, &sdf);
    eprintln!();

    let pass = run_checks(&solid, &sdf, &bounds);

    eprintln!();
    if pass {
        eprintln!("  All checks passed.");
    } else {
        eprintln!("  SOME CHECKS FAILED.");
    }
    eprintln!();

    let mesh_data = solid.mesh(MESH_TOLERANCE);
    eprintln!(
        "  Visual mesh: {} vertices, {} faces",
        mesh_data.vertices.len(),
        mesh_data.faces.len()
    );
    eprintln!();

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "SDF Physics 01 — SDF Grid (sphere r=5)".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .insert_resource(MeshDataRes(mesh_data))
        .add_systems(Startup, setup)
        .run();
}

// ── SDF grid construction (mirrors to_model logic) ──────────────────────

fn build_sdf_grid(solid: &Solid, bounds: &cf_geometry::Aabb, resolution: f64) -> SdfGrid {
    let expanded = bounds.expanded(resolution * 2.0);
    let size = expanded.size();

    let nx = ((size.x / resolution).ceil() as usize).max(2);
    let ny = ((size.y / resolution).ceil() as usize).max(2);
    let nz = ((size.z / resolution).ceil() as usize).max(2);

    SdfGrid::from_fn(nx, ny, nz, resolution, expanded.min, |p| solid.evaluate(&p))
}

// ── Diagnostics ─────────────────────────────────────────────────────────

fn print_grid_info(bounds: &cf_geometry::Aabb, sdf: &SdfGrid) {
    let size = bounds.size();
    let cell_count = sdf.width() * sdf.height() * sdf.depth();
    let memory_kb = (cell_count * 8) as f64 / 1024.0;

    eprintln!("  SDF Grid Diagnostics");
    eprintln!("  --------------------");
    eprintln!(
        "  Solid bounds: ({:.1}, {:.1}, {:.1}) to ({:.1}, {:.1}, {:.1})",
        bounds.min.x, bounds.min.y, bounds.min.z, bounds.max.x, bounds.max.y, bounds.max.z,
    );
    eprintln!(
        "  Solid size:   {:.1} × {:.1} × {:.1} mm",
        size.x, size.y, size.z
    );
    eprintln!(
        "  Grid dims:    {} × {} × {}  ({} cells, {:.1} KB)",
        sdf.width(),
        sdf.height(),
        sdf.depth(),
        cell_count,
        memory_kb,
    );
    eprintln!("  Cell size:    {:.4} mm", sdf.cell_size());
    eprintln!(
        "  Value range:  [{:.4}, {:.4}]",
        sdf.min_value(),
        sdf.max_value()
    );
}

// ── Checks ──────────────────────────────────────────────────────────────

fn run_checks(solid: &Solid, sdf: &SdfGrid, bounds: &cf_geometry::Aabb) -> bool {
    let mut pass = true;

    // 1. Interior/exterior existence
    pass &= check("min_value < 0 (interior exists)", sdf.min_value() < 0.0);
    pass &= check("max_value > 0 (exterior exists)", sdf.max_value() > 0.0);

    // 2. Known-point checks for sphere(5.0)
    //    Center should be deep inside (d = -5.0)
    if let Some(d) = sdf.distance(Point3::origin()) {
        pass &= check(
            &format!("center is inside, d ≈ -5.0 (got {d:.4})"),
            d < -4.0,
        );
    } else {
        pass &= check("center is within grid", false);
    }

    //    Surface point should be near zero
    let surface_pt = Point3::new(5.0, 0.0, 0.0);
    if let Some(d) = sdf.distance(surface_pt) {
        pass &= check(
            &format!("surface point d ≈ 0.0 (got {d:.4})"),
            d.abs() < 0.5,
        );
    } else {
        pass &= check("surface point within grid", false);
    }

    //    Point just outside sphere, inside grid: (5.5, 0, 0) → d ≈ 0.5
    let outside_pt = Point3::new(5.5, 0.0, 0.0);
    if let Some(d) = sdf.distance(outside_pt) {
        pass &= check(
            &format!("outside-sphere point d ≈ 0.5 (got {d:.4})"),
            (d - 0.5).abs() < 0.3,
        );
    } else {
        pass &= check("outside-sphere point within grid", false);
    }

    // 3. Accuracy: compare SdfGrid vs Solid at sampled points
    let accuracy = measure_accuracy(solid, sdf, bounds);
    pass &= check(
        &format!(
            "grid accuracy: max err = {:.4} mm, mean err = {:.4} mm ({} samples)",
            accuracy.max_err, accuracy.mean_err, accuracy.n_samples,
        ),
        accuracy.max_err < sdf.cell_size(), // max error should be < 1 cell
    );

    pass
}

struct Accuracy {
    max_err: f64,
    mean_err: f64,
    n_samples: usize,
}

/// Sample a 10×10×10 uniform grid inside the bounds, evaluate both
/// `solid.evaluate()` and `sdf.distance()`, compute error statistics.
fn measure_accuracy(solid: &Solid, sdf: &SdfGrid, bounds: &cf_geometry::Aabb) -> Accuracy {
    let n = 10;
    let mut total_err = 0.0;
    let mut max_err: f64 = 0.0;
    let mut count = 0usize;

    for iz in 0..n {
        for iy in 0..n {
            for ix in 0..n {
                let t = |i: usize| (i as f64 + 0.5) / n as f64;
                let p = Point3::new(
                    bounds.min.x + t(ix) * (bounds.max.x - bounds.min.x),
                    bounds.min.y + t(iy) * (bounds.max.y - bounds.min.y),
                    bounds.min.z + t(iz) * (bounds.max.z - bounds.min.z),
                );

                let exact = solid.evaluate(&p);
                if let Some(grid) = sdf.distance(p) {
                    let err = (exact - grid).abs();
                    total_err += err;
                    max_err = max_err.max(err);
                    count += 1;
                }
            }
        }
    }

    Accuracy {
        max_err,
        mean_err: if count > 0 {
            total_err / count as f64
        } else {
            f64::INFINITY
        },
        n_samples: count,
    }
}

fn check(label: &str, ok: bool) -> bool {
    if ok {
        eprintln!("  PASS: {label}");
    } else {
        eprintln!("  FAIL: {label}");
    }
    ok
}

// ── Bevy visualization ──────────────────────────────────────────────────

#[derive(Resource)]
struct MeshDataRes(cf_geometry::IndexedMesh);

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mesh_data: Res<MeshDataRes>,
) {
    spawn_design_mesh(
        &mut commands,
        &mut meshes,
        &mut materials,
        &mesh_data.0,
        Point3::origin(),
        Color::srgb(0.3, 0.6, 0.9),
    );

    ExampleScene::new(25.0, 30.0).with_angles(0.5, 0.6).spawn(
        &mut commands,
        &mut meshes,
        &mut materials,
    );
}
