//! Fluid Drag — Terminal Velocity and the Inertia-Box Model
//!
//! Three spheres of different mass fall through a dense medium. Each reaches
//! a different terminal velocity where drag force balances weight. The light
//! sphere reaches its (low) terminal velocity quickly and drifts down slowly.
//! The heavy sphere accelerates longer before drag catches up.
//!
//! Uses the inertia-box fluid model (body-level, automatic when no per-geom
//! fluid params are set). Drag has two terms:
//! - Viscous (Stokes): F ~ beta * v  (linear, dominant at low Re)
//! - Quadratic: F ~ 0.5 * rho * A * v^2  (dominant at high Re)
//!
//! Run: `cargo run -p example-passive-fluid-drag --release`

#![allow(
    clippy::doc_markdown,
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::similar_names,
    clippy::too_many_lines,
    clippy::needless_range_loop,
    clippy::needless_pass_by_value,
    clippy::cast_sign_loss,
    clippy::cast_lossless,
    clippy::let_underscore_must_use,
    clippy::unwrap_used
)]

use bevy::prelude::*;
use sim_bevy::camera::OrbitCameraPlugin;
use sim_bevy::examples::{
    PhysicsHud, ValidationHarness, render_physics_hud, spawn_example_camera, spawn_physics_hud,
    validation_system,
};
use sim_bevy::materials::MetalPreset;
use sim_bevy::model_data::{
    PhysicsAccumulator, PhysicsData, PhysicsModel, spawn_model_geoms, step_physics_realtime,
    sync_geom_transforms,
};
use sim_core::validation::{Check, print_report};

// ── MJCF ──────────────────────────────────────────────────────────────────

/// Dense medium (density=500 kg/m^3, viscosity=0.5 Pa*s) for visible terminal
/// velocities. Three spheres with same radius but different mass.
///
/// Terminal velocity (inertia-box, quadratic-dominated):
///   v_t = sqrt(2*m*g / (rho * bx * by))
///   bx = by = r * sqrt(12/5) for a sphere
///
/// Light (0.5 kg): v_t ~ 0.55 m/s
/// Medium (2.0 kg): v_t ~ 1.10 m/s
/// Heavy (8.0 kg): v_t ~ 2.20 m/s
const MJCF: &str = r#"
<mujoco model="fluid-drag">
  <option gravity="0 0 -9.81" timestep="0.002"
          density="500" viscosity="0.5"/>
  <worldbody>
    <body name="light" pos="-1 0 0">
      <freejoint name="light_free"/>
      <geom name="light_geom" type="sphere" size="0.15" mass="0.5"
            rgba="0.3 0.6 1.0 1"/>
    </body>
    <body name="medium" pos="0 0 0">
      <freejoint name="medium_free"/>
      <geom name="medium_geom" type="sphere" size="0.15" mass="2.0"
            rgba="0.9 0.7 0.2 1"/>
    </body>
    <body name="heavy" pos="1 0 0">
      <freejoint name="heavy_free"/>
      <geom name="heavy_geom" type="sphere" size="0.15" mass="8.0"
            rgba="0.8 0.2 0.2 1"/>
    </body>
  </worldbody>
</mujoco>
"#;

const REPORT_TIME: f64 = 10.0;

// ── Bevy app ──────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Fluid Drag — Terminal Velocity ===");
    println!("  Three spheres (light/medium/heavy) falling in a dense medium.");
    println!("  Each reaches a different terminal velocity.\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "Fluid Drag — Terminal Velocity".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .init_resource::<PhysicsHud>()
        .init_resource::<DragValidation>()
        .insert_resource(
            ValidationHarness::new()
                .report_at(REPORT_TIME)
                .print_every(1.0)
                .display(|m, d| {
                    let t = d.time;
                    let dof_l = m.body_dof_adr[m.body_id("light").unwrap()];
                    let dof_m = m.body_dof_adr[m.body_id("medium").unwrap()];
                    let dof_h = m.body_dof_adr[m.body_id("heavy").unwrap()];
                    let vz_l = -d.qvel[dof_l + 2];
                    let vz_m = -d.qvel[dof_m + 2];
                    let vz_h = -d.qvel[dof_h + 2];
                    format!("t={t:.1}s  v_light={vz_l:.3}  v_med={vz_m:.3}  v_heavy={vz_h:.3} m/s")
                }),
        )
        .add_systems(Startup, setup)
        .add_systems(Update, step_physics_realtime)
        .add_systems(
            PostUpdate,
            (
                sync_geom_transforms,
                validation_system,
                drag_diagnostics,
                update_hud,
                render_physics_hud,
            )
                .chain(),
        )
        .run();
}

// ── Setup ─────────────────────────────────────────────────────────────────

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let model = sim_mjcf::load_model(MJCF).expect("MJCF should parse");
    let data = model.make_data();

    let mat_light =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.3, 0.6, 1.0)));
    let mat_medium =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.9, 0.7, 0.2)));
    let mat_heavy =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.8, 0.2, 0.2)));

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[
            ("light_geom", mat_light),
            ("medium_geom", mat_medium),
            ("heavy_geom", mat_heavy),
        ],
    );

    // Camera: Bevy Y-up. Spheres start at MuJoCo z=0 -> Bevy y=0.
    // They fall in -Y (Bevy). Camera looks at (0, -3, 0) to see the
    // initial separation zone.
    spawn_example_camera(
        &mut commands,
        Vec3::new(0.0, -3.0, 0.0),
        10.0,
        std::f32::consts::FRAC_PI_4,
        0.3,
    );

    spawn_physics_hud(&mut commands);

    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── HUD ───────────────────────────────────────────────────────────────────

fn update_hud(model: Res<PhysicsModel>, data: Res<PhysicsData>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();

    let m = &model.0;
    let d = &data.0;

    hud.section("Terminal Velocity");
    hud.raw(String::new());

    let bodies: [(&str, &str, &str); 3] = [
        ("Light ", "0.5 kg", "light"),
        ("Medium", "2.0 kg", "medium"),
        ("Heavy ", "8.0 kg", "heavy"),
    ];

    //  Header
    hud.raw("           speed    drag/wt".to_string());

    for (name, mass_label, body_name) in &bodies {
        let bid = m.body_id(body_name).expect(body_name);
        let dof = m.body_dof_adr[bid];
        let vz = -d.qvel[dof + 2];
        let fz = d.qfrc_fluid[dof + 2];
        let weight = m.body_mass[bid] * 9.81;
        let pct = fz / weight * 100.0;

        hud.raw(format!("  {name} ({mass_label})  {vz:5.3} m/s  {pct:5.1}%"));
    }

    hud.raw(String::new());
    hud.section("Medium Properties");
    hud.scalar("density (kg/m3)", m.density, 0);
    hud.scalar("viscosity (Pa s)", m.viscosity, 2);
    hud.scalar("time (s)", d.time, 1);
}

// ── Validation ────────────────────────────────────────────────────────────

#[derive(Resource, Default)]
struct DragValidation {
    /// Track whether light sphere velocity has plateaued
    light_plateau_start: Option<f64>,
    light_plateau_duration: f64,
    prev_vz_light: f64,
    reported: bool,
}

fn drag_diagnostics(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<DragValidation>,
) {
    let d = &data.0;
    let m = &model.0;

    if d.time < 0.5 {
        return;
    }

    // Light sphere z-velocity (downward speed, positive)
    let bid_l = m.body_id("light").expect("light");
    let dof_light = m.body_dof_adr[bid_l];
    let vz_light = -d.qvel[dof_light + 2];

    // Check if acceleration is small (velocity plateau)
    let dt = m.timestep;
    let dvdt = (vz_light - val.prev_vz_light) / dt;
    val.prev_vz_light = vz_light;

    if dvdt.abs() < 0.1 {
        if val.light_plateau_start.is_none() {
            val.light_plateau_start = Some(d.time);
        }
        val.light_plateau_duration = d.time - val.light_plateau_start.unwrap_or(d.time);
    } else {
        val.light_plateau_start = None;
        val.light_plateau_duration = 0.0;
    }

    if harness.reported() && !val.reported {
        val.reported = true;

        let bid_m = m.body_id("medium").expect("medium");
        let bid_h = m.body_id("heavy").expect("heavy");
        let dof_medium = m.body_dof_adr[bid_m];
        let dof_heavy = m.body_dof_adr[bid_h];
        let vz_medium = -d.qvel[dof_medium + 2];
        let vz_heavy = -d.qvel[dof_heavy + 2];

        // Drag-weight balance for each body
        let drag_l = d.qfrc_fluid[dof_light + 2];
        let w_l = m.body_mass[bid_l] * 9.81;
        let err_l = ((drag_l - w_l) / w_l * 100.0).abs();

        let drag_m = d.qfrc_fluid[dof_medium + 2];
        let w_m = m.body_mass[bid_m] * 9.81;
        let err_m = ((drag_m - w_m) / w_m * 100.0).abs();

        let drag_h = d.qfrc_fluid[dof_heavy + 2];
        let w_h = m.body_mass[bid_h] * 9.81;
        let err_h = ((drag_h - w_h) / w_h * 100.0).abs();

        let checks = vec![
            Check {
                name: "Light sphere velocity plateau",
                pass: val.light_plateau_duration > 2.0,
                detail: format!(
                    "plateau duration = {:.1}s (need > 2s)",
                    val.light_plateau_duration
                ),
            },
            Check {
                name: "Terminal velocity ordering",
                pass: vz_light < vz_medium && vz_medium < vz_heavy,
                detail: format!(
                    "v_light={vz_light:.3} < v_med={vz_medium:.3} < v_heavy={vz_heavy:.3}"
                ),
            },
            Check {
                name: "Light drag ~ weight",
                pass: err_l < 5.0,
                detail: format!("drag={drag_l:.4}, weight={w_l:.4}, err={err_l:.2}%"),
            },
            Check {
                name: "Medium drag ~ weight",
                pass: err_m < 5.0,
                detail: format!("drag={drag_m:.4}, weight={w_m:.4}, err={err_m:.2}%"),
            },
            Check {
                name: "Heavy drag ~ weight",
                pass: err_h < 5.0,
                detail: format!("drag={drag_h:.4}, weight={w_h:.4}, err={err_h:.2}%"),
            },
        ];

        let _ = print_report("Fluid Drag (t=10s)", &checks);
    }
}
