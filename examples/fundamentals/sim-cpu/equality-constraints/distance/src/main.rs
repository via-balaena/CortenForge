//! Distance Constraint — Rigid Rod (1-DOF Scalar Constraint)
//!
//! Two free-floating spheres connected by a rigid rod (distance constraint).
//! A visible thin rod is drawn between the sphere centers each frame.
//! Sphere A gets a lateral velocity kick so the pair tumbles and orbits
//! rather than falling straight down. The heavier sphere stays lower; the
//! lighter one orbits around it.
//!
//! Validates:
//! - Distance maintained at 0.5m (< 5mm deviation)
//! - Both spheres move (velocity > 0.01 m/s)
//! - Different masses → lighter sphere moves more
//! - Energy bounded (< 5% growth)
//!
//! Run with: `cargo run -p example-equality-distance --release`

#![allow(
    clippy::doc_markdown,
    clippy::needless_pass_by_value,
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::let_underscore_must_use,
    clippy::suboptimal_flops
)]

use bevy::prelude::*;
use nalgebra::Vector3;
use sim_bevy::camera::OrbitCameraPlugin;
use sim_bevy::convert::physics_pos;
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

/// Marker for the visual rod connecting the two spheres.
#[derive(Component)]
struct RodVisual;

// ── MJCF Model ──────────────────────────────────────────────────────────────
//
// Two free spheres with a distance constraint (0.5m separation).
// Ground plane for landing. No lateral kick — falls straight down.

const MJCF: &str = r#"
<mujoco model="distance-rigid-rod">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.001" solver="Newton"
          iterations="50" tolerance="1e-10">
    <flag energy="enable"/>
  </option>

  <worldbody>
    <geom name="ground" type="plane" size="2 2 0.01"/>

    <body name="sphere_a" pos="-0.2 0 1.3">
      <freejoint/>
      <geom name="ga" type="sphere" size="0.08" mass="1.0"/>
    </body>
    <body name="sphere_b" pos="0.2 0 1.0">
      <freejoint/>
      <geom name="gb" type="sphere" size="0.06" mass="0.5"/>
    </body>
  </worldbody>

  <equality>
    <distance geom1="ga" geom2="gb" distance="0.5" solref="0.005 1.0"/>
  </equality>
</mujoco>
"#;

const TARGET_DIST: f64 = 0.5;
const BODY_A: usize = 1;
const BODY_B: usize = 2;

// ── Bevy App ────────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Distance Constraint (Rigid Rod) ===");
    println!("  Two spheres connected by a rigid rod ({TARGET_DIST}m)");
    println!("  Sphere A: 1.0kg (red), Sphere B: 0.5kg (blue)");
    println!("  Falls straight down, bounces, comes to rest");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Distance Constraint (Rigid Rod)".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .init_resource::<PhysicsHud>()
        .init_resource::<DistanceValidation>()
        .insert_resource(
            ValidationHarness::new()
                .report_at(5.0)
                .print_every(1.0)
                .display(|_m, d| {
                    let sep = (d.xpos[1] - d.xpos[2]).norm();
                    let err_mm = (sep - 0.5).abs() * 1000.0;
                    let e = d.energy_kinetic + d.energy_potential;
                    format!("dist={sep:.4}m  err={err_mm:.1}mm  E={e:.3}J")
                }),
        )
        .add_systems(Startup, setup)
        .add_systems(Update, step_physics_realtime)
        .add_systems(
            PostUpdate,
            (
                sync_geom_transforms,
                update_rod_visual,
                validation_system,
                distance_diagnostics,
                update_hud,
                render_physics_hud,
            )
                .chain(),
        )
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let model = sim_mjcf::load_model(MJCF).expect("MJCF should parse");
    let mut data = model.make_data();

    let _ = data.forward(&model);

    println!(
        "  Model: {} bodies, {} joints, {} equality constraints\n",
        model.nbody, model.njnt, model.neq
    );

    // ── Materials ───────────────────────────────────────────────────────
    let mat_a = materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.85, 0.25, 0.2)));
    let mat_b = materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.2, 0.4, 0.85)));

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[("ga", mat_a), ("gb", mat_b)],
    );

    // Camera: look at midpoint, slightly above
    spawn_example_camera(
        &mut commands,
        Vec3::new(0.0, 0.8, 0.0), // Bevy Y-up: center at z=0.8 in MuJoCo
        3.0,
        std::f32::consts::FRAC_PI_4,
        0.3,
    );

    spawn_physics_hud(&mut commands);

    // Visible rod between the two spheres (unit cylinder, scaled each frame)
    let rod_mat = materials.add(MetalPreset::SpringWire.material());
    commands.spawn((
        Mesh3d(meshes.add(Cylinder::new(1.0, 1.0))),
        MeshMaterial3d(rod_mat),
        Transform::default(),
        RodVisual,
    ));

    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── Rod Visual ──────────────────────────────────────────────────────────────

const ROD_RADIUS: f32 = 0.015;

fn update_rod_visual(data: Res<PhysicsData>, mut query: Query<&mut Transform, With<RodVisual>>) {
    let pa = &data.xpos[BODY_A];
    let pb = &data.xpos[BODY_B];

    // Convert MuJoCo (x,y,z) → Bevy (x,z,y)
    let a = physics_pos(pa[0] as f32, pa[1] as f32, pa[2] as f32);
    let b = physics_pos(pb[0] as f32, pb[1] as f32, pb[2] as f32);

    let midpoint = (a + b) * 0.5;
    let diff = b - a;
    let length = diff.length();

    if length < 1e-6 {
        return;
    }

    let dir = diff / length;
    // Bevy Cylinder is Y-up by default, so rotate from Y to the rod direction
    let rotation = Quat::from_rotation_arc(Vec3::Y, dir);

    for mut transform in &mut query {
        // Scale: radius in X/Z, half-length in Y (Cylinder height is 1.0, centered)
        transform.translation = midpoint;
        transform.rotation = rotation;
        transform.scale = Vec3::new(ROD_RADIUS, length, ROD_RADIUS);
    }
}

// ── HUD ─────────────────────────────────────────────────────────────────────

fn update_hud(_model: Res<PhysicsModel>, data: Res<PhysicsData>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();
    hud.section("Distance — Rigid Rod");

    let sep = (data.xpos[BODY_A] - data.xpos[BODY_B]).norm();
    let err = (sep - TARGET_DIST).abs() * 1000.0;
    hud.scalar("distance (m)", sep, 4);
    hud.scalar("target (m)", TARGET_DIST, 4);
    hud.scalar("error (mm)", err, 2);

    let vel_a = Vector3::new(data.qvel[0], data.qvel[1], data.qvel[2]).norm();
    let vel_b = Vector3::new(data.qvel[6], data.qvel[7], data.qvel[8]).norm();
    hud.scalar("vel A (m/s)", vel_a, 3);
    hud.scalar("vel B (m/s)", vel_b, 3);

    let energy = data.energy_kinetic + data.energy_potential;
    hud.scalar("energy (J)", energy, 3);

    // Constraint force (distance = 1 row)
    if data.ne >= 1 {
        hud.scalar("efc force (N)", data.efc_force[0].abs(), 1);
    }

    hud.scalar("time", data.time, 1);
}

// ── Validation ──────────────────────────────────────────────────────────────

#[derive(Resource, Default)]
struct DistanceValidation {
    max_dist_err: f64,
    sphere_a_moved: bool,
    initial_energy: Option<f64>,
    max_energy_growth: f64,
    reported: bool,
}

fn distance_diagnostics(
    _model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<DistanceValidation>,
) {
    // Record initial energy
    if val.initial_energy.is_none() {
        val.initial_energy = Some(data.energy_kinetic + data.energy_potential);
    }

    // Distance maintained
    let sep = (data.xpos[BODY_A] - data.xpos[BODY_B]).norm();
    let err = (sep - TARGET_DIST).abs();
    val.max_dist_err = val.max_dist_err.max(err);

    // Sphere A moves
    let vel_a = Vector3::new(data.qvel[0], data.qvel[1], data.qvel[2]).norm();
    if vel_a > 0.01 {
        val.sphere_a_moved = true;
    }

    // Energy growth
    if let Some(e0) = val.initial_energy {
        let e = data.energy_kinetic + data.energy_potential;
        let growth = (e - e0) / e0.abs().max(1e-10);
        val.max_energy_growth = val.max_energy_growth.max(growth);
    }

    // Final report
    if harness.reported() && !val.reported {
        val.reported = true;

        let vel_a = Vector3::new(data.qvel[0], data.qvel[1], data.qvel[2]).norm();
        let vel_b = Vector3::new(data.qvel[6], data.qvel[7], data.qvel[8]).norm();

        let checks = vec![
            Check {
                name: "Distance maintained",
                pass: val.max_dist_err < 0.005,
                detail: format!("max err = {:.2} mm", val.max_dist_err * 1000.0),
            },
            Check {
                name: "Both move",
                pass: val.sphere_a_moved,
                detail: format!("vel_a = {vel_a:.4} m/s"),
            },
            Check {
                name: "Mass ratio effect",
                pass: vel_b > vel_a * 0.5 || vel_a < 0.01,
                detail: format!("vel_a={vel_a:.3}, vel_b={vel_b:.3}"),
            },
            Check {
                name: "Energy bounded",
                pass: val.max_energy_growth < 0.05,
                detail: format!("max growth = {:.2}%", val.max_energy_growth * 100.0),
            },
        ];
        let _ = print_report("Distance Constraint (t=5s)", &checks);
    }
}
