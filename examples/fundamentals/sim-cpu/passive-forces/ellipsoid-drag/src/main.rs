//! Ellipsoid Drag — Per-Geom Advanced Fluid Model
//!
//! Three bodies with different shapes (capsule, sphere, flat cylinder) but
//! equal mass fall through a dense medium. Each uses `fluidshape="ellipsoid"`
//! which activates the 5-component advanced fluid model:
//!   1. Added mass (gyroscopic)
//!   2. Magnus lift
//!   3. Kutta lift
//!   4. Combined linear drag
//!   5. Combined angular drag
//!
//! Shape determines projected area and semi-axes, so drag differs. The
//! streamlined capsule (long axis aligned with fall) has low frontal area
//! and falls fastest. The flat cylinder (disc face into the flow) has the
//! highest frontal area and falls slowest.
//!
//! Run: `cargo run -p example-passive-ellipsoid-drag --release`

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

/// Three shapes, equal mass (2 kg), all with fluidshape="ellipsoid".
/// Dense medium (density=500, viscosity=0.01) — high density makes quadratic
/// drag dominate, so projected area drives the ordering.
///
/// Capsule: r=0.06, half-length=0.2 → semi-axes [0.06, 0.06, 0.26].
///   Frontal area (pi*0.06*0.06) = 0.011 m^2. Streamlined.
/// Sphere: r=0.15 → semi-axes [0.15, 0.15, 0.15].
///   Frontal area (pi*0.15*0.15) = 0.071 m^2. Moderate.
/// Cylinder: r=0.25, half-height=0.04 → semi-axes [0.25, 0.25, 0.04].
///   Frontal area (pi*0.25*0.25) = 0.196 m^2. High drag.
///
/// Expected ordering: v_capsule > v_sphere > v_cylinder
const MJCF: &str = r#"
<mujoco model="ellipsoid-drag">
  <option gravity="0 0 -9.81" timestep="0.002"
          density="500" viscosity="0.01"/>
  <worldbody>
    <body name="capsule" pos="-1 0 0">
      <freejoint name="cap_free"/>
      <geom name="cap_geom" type="capsule" size="0.06 0.2" mass="2.0"
            fluidshape="ellipsoid" rgba="0.2 0.7 0.3 1"/>
    </body>
    <body name="sphere" pos="0 0 0">
      <freejoint name="sph_free"/>
      <geom name="sph_geom" type="sphere" size="0.15" mass="2.0"
            fluidshape="ellipsoid" rgba="0.3 0.5 0.9 1"/>
    </body>
    <body name="cylinder" pos="1 0 0">
      <freejoint name="cyl_free"/>
      <geom name="cyl_geom" type="cylinder" size="0.25 0.04" mass="2.0"
            fluidshape="ellipsoid" rgba="0.9 0.4 0.2 1"/>
    </body>
  </worldbody>
</mujoco>
"#;

const REPORT_TIME: f64 = 10.0;

// ── Bevy app ──────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Ellipsoid Drag — Shape-Dependent Terminal Velocity ===");
    println!("  Three shapes (capsule/sphere/flat cylinder), equal mass, same medium.");
    println!("  Shape determines drag. Streamlined capsule falls fastest.\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "Ellipsoid Drag — Shape-Dependent Terminal Velocity".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .init_resource::<PhysicsHud>()
        .init_resource::<EllipsoidValidation>()
        .insert_resource(
            ValidationHarness::new()
                .report_at(REPORT_TIME)
                .print_every(1.0)
                .display(|m, d| {
                    let t = d.time;
                    let dof_c = m.body_dof_adr[m.body_id("capsule").unwrap()];
                    let dof_s = m.body_dof_adr[m.body_id("sphere").unwrap()];
                    let dof_y = m.body_dof_adr[m.body_id("cylinder").unwrap()];
                    let vc = -d.qvel[dof_c + 2];
                    let vs = -d.qvel[dof_s + 2];
                    let vy = -d.qvel[dof_y + 2];
                    format!("t={t:.1}s  cap={vc:.3}  sph={vs:.3}  cyl={vy:.3} m/s")
                }),
        )
        .add_systems(Startup, setup)
        .add_systems(Update, step_physics_realtime)
        .add_systems(
            PostUpdate,
            (
                sync_geom_transforms,
                validation_system,
                ellipsoid_diagnostics,
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

    let mat_cap = materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.2, 0.7, 0.3)));
    let mat_sph = materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.3, 0.5, 0.9)));
    let mat_cyl = materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.9, 0.4, 0.2)));

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[
            ("cap_geom", mat_cap),
            ("sph_geom", mat_sph),
            ("cyl_geom", mat_cyl),
        ],
    );

    // Camera: Bevy Y-up. Shapes start at MuJoCo z=0 -> Bevy y=0.
    // They fall in -Y. Camera looks slightly below start to see separation.
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

    hud.section("Ellipsoid Drag (5-component)");
    hud.raw(String::new());

    let shapes: [(&str, &str); 3] = [
        ("Capsule ", "capsule"),
        ("Sphere  ", "sphere"),
        ("Cylinder", "cylinder"),
    ];

    hud.raw("              speed    drag/wt".to_string());

    for (name, body_name) in &shapes {
        let bid = m.body_id(body_name).expect(body_name);
        let dof = m.body_dof_adr[bid];
        let vz = -d.qvel[dof + 2];
        let fz = d.qfrc_fluid[dof + 2];
        let weight = m.body_mass[bid] * 9.81;
        let pct = fz / weight * 100.0;

        hud.raw(format!("  {name}  {vz:6.3} m/s  {pct:5.1}%"));
    }

    hud.raw(String::new());
    hud.raw("  All shapes: 2.0 kg, same medium".to_string());
    hud.scalar("density (kg/m3)", m.density, 0);
    hud.scalar("viscosity (Pa s)", m.viscosity, 2);
    hud.scalar("time (s)", d.time, 1);
}

// ── Validation ────────────────────────────────────────────────────────────

#[derive(Resource, Default)]
struct EllipsoidValidation {
    reported: bool,
}

fn ellipsoid_diagnostics(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<EllipsoidValidation>,
) {
    if !harness.reported() || val.reported {
        return;
    }
    val.reported = true;

    let m = &model.0;
    let d = &data.0;

    let bid_cap = m.body_id("capsule").expect("capsule");
    let bid_sph = m.body_id("sphere").expect("sphere");
    let bid_cyl = m.body_id("cylinder").expect("cylinder");

    let dof_cap = m.body_dof_adr[bid_cap];
    let dof_sph = m.body_dof_adr[bid_sph];
    let dof_cyl = m.body_dof_adr[bid_cyl];

    let v_cap = -d.qvel[dof_cap + 2];
    let v_sph = -d.qvel[dof_sph + 2];
    let v_cyl = -d.qvel[dof_cyl + 2];

    // Drag-weight balance (all same mass)
    let drag_cap = d.qfrc_fluid[dof_cap + 2];
    let drag_sph = d.qfrc_fluid[dof_sph + 2];
    let drag_cyl = d.qfrc_fluid[dof_cyl + 2];
    let weight = m.body_mass[bid_cap] * 9.81;

    let err_cap = ((drag_cap - weight) / weight * 100.0).abs();
    let err_sph = ((drag_sph - weight) / weight * 100.0).abs();
    let err_cyl = ((drag_cyl - weight) / weight * 100.0).abs();

    let checks = vec![
        Check {
            name: "Velocity ordering: cap > sph > cyl",
            pass: v_cap > v_sph && v_sph > v_cyl,
            detail: format!("cap={v_cap:.3} > sph={v_sph:.3} > cyl={v_cyl:.3}"),
        },
        Check {
            name: "Capsule drag ~ weight",
            pass: err_cap < 5.0,
            detail: format!("drag={drag_cap:.4}, weight={weight:.4}, err={err_cap:.2}%"),
        },
        Check {
            name: "Sphere drag ~ weight",
            pass: err_sph < 5.0,
            detail: format!("drag={drag_sph:.4}, weight={weight:.4}, err={err_sph:.2}%"),
        },
        Check {
            name: "Cylinder drag ~ weight",
            pass: err_cyl < 5.0,
            detail: format!("drag={drag_cyl:.4}, weight={weight:.4}, err={err_cyl:.2}%"),
        },
    ];

    let _ = print_report("Ellipsoid Drag (t=10s)", &checks);
}
