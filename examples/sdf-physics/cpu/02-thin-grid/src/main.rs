//! SDF Physics 02 — Thin Grid
//!
//! Tests `SdfGrid` fidelity on thin-walled geometry at fine resolutions.
//! Step 01 proved the grid works for a solid sphere. This step proves it
//! can represent the thin walls that sockets and shells require.
//!
//! Three test cases of increasing difficulty:
//! 1. `sphere(5).shell(1.0)` at 1.0 mm — thick shell, wall = 2.0 mm (easy)
//! 2. `sphere(5).shell(0.3)` at 0.5 mm — thin shell, wall = 0.6 mm (socket scenario)
//! 3. finger-design socket geometry at 0.5 mm — the literal target shape
//!
//! Pass criteria per case:
//! - Interior exists (wall material has negative SDF values)
//! - Void is not filled in (void center evaluates positive)
//! - Grid accuracy < cell size (sampled comparison vs `solid.evaluate()`)
//! - Wall is resolved (grid captures the thin wall faithfully)
//!
//! New concept: thin-wall `SdfGrid` fidelity
//! Depends on: 01-sdf-grid (`SdfGrid` construction works for solid shapes)
//!
//! Run with: `cargo run -p example-sdf-cpu-02-thin-grid --release`

#![allow(
    clippy::needless_pass_by_value,
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss
)]

use bevy::prelude::*;
use cf_design::{Aabb, SdfGrid, Solid};
use nalgebra::{Point3, Vector3};
use sim_bevy::camera::OrbitCameraPlugin;
use sim_bevy::mesh::spawn_design_mesh;
use sim_bevy::scene::ExampleScene;

fn main() {
    // ── Test case 1: thick shell (easy) ─────────────────────────────
    let case1 = TestCase {
        name: "Thick shell",
        solid: Solid::sphere(5.0).shell(1.0),
        cell_size: 1.0,
        wall_thickness: 2.0, // shell(1.0) → 1.0 on each side = 2.0 total
        wall_probe: Point3::new(5.0, 0.0, 0.0), // original surface = deepest in wall
        void_probe: Point3::origin(), // center of sphere void
    };

    // ── Test case 2: thin shell (socket-wall scenario) ──────────────
    let case2 = TestCase {
        name: "Thin shell (0.6 mm wall)",
        solid: Solid::sphere(5.0).shell(0.3),
        cell_size: 0.5,
        wall_thickness: 0.6, // shell(0.3) → 0.3 on each side = 0.6 total
        wall_probe: Point3::new(5.0, 0.0, 0.0),
        void_probe: Point3::origin(),
    };

    // ── Test case 3: finger-design socket geometry ──────────────────
    let socket_outer = Solid::ellipsoid(Vector3::new(6.4, 4.9, 4.9));
    let socket_void = Solid::ellipsoid(Vector3::new(5.8, 4.3, 4.3));
    let case3 = TestCase {
        name: "Socket (finger-design)",
        solid: socket_outer.subtract(socket_void),
        cell_size: 0.5,
        wall_thickness: 0.6,                    // 6.4 - 5.8 = 0.6 on X axis
        wall_probe: Point3::new(6.1, 0.0, 0.0), // midpoint of wall on X
        void_probe: Point3::origin(),           // center of socket void
    };

    let test_cases = [case1, case2, case3];
    let mut all_pass = true;

    for case in &test_cases {
        eprintln!();
        let pass = run_case(case);
        all_pass &= pass;
    }

    eprintln!();
    if all_pass {
        eprintln!("  All test cases passed.");
    } else {
        eprintln!("  SOME TEST CASES FAILED.");
    }
    eprintln!();

    // ── Visualize all three side by side ─────────────────────────────
    let meshes_for_viz: Vec<_> = test_cases.iter().map(|c| c.solid.mesh(0.3)).collect();

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "SDF Physics 02 — Thin Grid".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .insert_resource(MeshDataRes(meshes_for_viz))
        .add_systems(Startup, setup)
        .run();
}

// ── Test case definition ────────────────────────────────────────────────

struct TestCase {
    name: &'static str,
    solid: Solid,
    cell_size: f64,
    wall_thickness: f64,
    wall_probe: Point3<f64>, // point in the middle of the wall (should be inside)
    void_probe: Point3<f64>, // point in the void (should be outside)
}

// ── Run one test case ───────────────────────────────────────────────────

fn run_case(case: &TestCase) -> bool {
    eprintln!(
        "  {} (cell={:.1} mm, wall={:.1} mm)",
        case.name, case.cell_size, case.wall_thickness
    );
    eprintln!("  {}", "-".repeat(50));

    let Some(bounds) = case.solid.bounds() else {
        eprintln!("  FAIL: solid has no bounds");
        return false;
    };

    let Some(sdf) = case.solid.sdf_grid_at(case.cell_size) else {
        eprintln!("  FAIL: sdf_grid_at returned None");
        return false;
    };

    // Print grid info
    let cell_count = sdf.width() * sdf.height() * sdf.depth();
    eprintln!(
        "  Grid: {} × {} × {} ({} cells, {:.1} KB)",
        sdf.width(),
        sdf.height(),
        sdf.depth(),
        cell_count,
        (cell_count * 8) as f64 / 1024.0,
    );
    eprintln!(
        "  Value range: [{:.4}, {:.4}]",
        sdf.min_value(),
        sdf.max_value()
    );

    let mut pass = true;

    // 1. Interior exists (wall material)
    pass &= check(
        "min_value < 0 (wall interior exists)",
        sdf.min_value() < 0.0,
    );

    // 2. Void is not filled in
    let void_exact = case.solid.evaluate(&case.void_probe);
    if let Some(void_grid) = sdf.distance(case.void_probe) {
        pass &= check(
            &format!("void center is outside wall (d = {void_grid:.4}, exact = {void_exact:.4})"),
            void_grid > 0.0,
        );
    } else {
        pass &= check("void probe within grid", false);
    }

    // 3. Wall probe is inside
    let wall_exact = case.solid.evaluate(&case.wall_probe);
    if let Some(wall_grid) = sdf.distance(case.wall_probe) {
        pass &= check(
            &format!("wall probe is inside (d = {wall_grid:.4}, exact = {wall_exact:.4})"),
            wall_grid < 0.0,
        );
    } else {
        pass &= check("wall probe within grid", false);
    }

    // 4. Accuracy: compare grid vs solid at sampled points
    let accuracy = measure_accuracy(&case.solid, &sdf, &bounds);
    pass &= check(
        &format!(
            "accuracy: max err = {:.4} mm, mean err = {:.4} mm ({} samples)",
            accuracy.max_err, accuracy.mean_err, accuracy.n_samples,
        ),
        accuracy.max_err < case.cell_size,
    );

    // 5. Wall resolution: walk along +X radially, verify at least one
    //    grid sample is negative (inside the wall)
    let wall_resolved = check_wall_resolved(&sdf, &case.solid, case.wall_thickness);
    pass &= check(
        &format!("wall resolved ({wall_resolved} interior samples along +X radial)"),
        wall_resolved > 0,
    );

    pass
}

// ── Wall resolution check ───────────────────────────────────────────────

/// Walk along +X in 0.1mm steps through the wall region. Count how many
/// grid samples return negative (inside the wall).
fn check_wall_resolved(sdf: &SdfGrid, solid: &Solid, _wall_thickness: f64) -> usize {
    let mut count = 0usize;
    let n_steps: usize = 150;
    for i in 0..n_steps {
        let r = (i as f64 + 0.5) * 15.0 / n_steps as f64;
        let p = Point3::new(r, 0.0, 0.0);
        let d = solid.evaluate(&p);
        if d < 0.0
            && let Some(gd) = sdf.distance(p)
            && gd < 0.0
        {
            count += 1;
        }
    }
    count
}

// ── Accuracy measurement (same as step 01) ──────────────────────────────

struct Accuracy {
    max_err: f64,
    mean_err: f64,
    n_samples: usize,
}

fn measure_accuracy(solid: &Solid, sdf: &SdfGrid, bounds: &Aabb) -> Accuracy {
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
struct MeshDataRes(Vec<cf_design::IndexedMesh>);

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mesh_data: Res<MeshDataRes>,
) {
    let colors = [
        Color::srgb(0.9, 0.4, 0.3), // red-orange
        Color::srgb(0.3, 0.7, 0.4), // green
        Color::srgb(0.3, 0.5, 0.9), // blue
    ];

    for (i, (mesh_data, &color)) in mesh_data.0.iter().zip(colors.iter()).enumerate() {
        let x_offset = (i as f64 - 1.0) * 18.0; // spread apart
        spawn_design_mesh(
            &mut commands,
            &mut meshes,
            &mut materials,
            mesh_data,
            Point3::new(x_offset, 0.0, 0.0),
            color,
        );
    }

    ExampleScene::new(60.0, 80.0).with_angles(0.5, 0.6).spawn(
        &mut commands,
        &mut meshes,
        &mut materials,
    );
}
