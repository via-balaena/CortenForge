//! SDF Physics 04 — Rest
//!
//! First contact step. A `Solid::sphere(5.0)` placed just above a ground
//! plane. It should settle onto the ground and come to rest.
//!
//! Verifies:
//! - `sdf_plane_contact` generates correct contact normal
//! - Body settles at z ≈ 5.0 (sphere center, bottom touching ground)
//! - Velocity approaches zero within a few seconds
//! - No jitter or vibration at rest
//! - Equilibrium penetration is small (< 0.5 mm)
//!
//! New concept: `sdf_plane_contact`
//! Depends on: 03-freefall (`to_model()` + gravity works)
//!
//! Run with: `cargo run -p example-sdf-04-rest --release`

#![allow(
    clippy::needless_pass_by_value,
    clippy::expect_used,
    clippy::let_underscore_must_use,
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation
)]

use bevy::prelude::*;
use cf_design::{JointDef, JointKind, Material, Mechanism, Part, Solid};
use nalgebra::{Point3, Vector3};
use sim_bevy::camera::OrbitCameraPlugin;
use sim_bevy::model_data::{
    PhysicsData, PhysicsModel, spawn_model_geoms, step_model_data, sync_geom_transforms,
};
use sim_bevy::scene::ExampleScene;

const RADIUS: f64 = 5.0;

fn main() {
    let solid = Solid::sphere(RADIUS);
    let material = Material::new("PLA", 1250.0);

    let mechanism = Mechanism::builder("resting")
        .part(Part::new("ball", solid, material))
        .joint(JointDef::new(
            "free",
            "world",
            "ball",
            JointKind::Free,
            Point3::new(0.0, 0.0, RADIUS + 0.5), // bottom 0.5 mm above ground
            Vector3::z(),
        ))
        .build();

    let mut model = mechanism.to_model(1.0, 0.3);
    model.add_ground_plane();

    // Default solref=[0.02, 1.0] is tuned for m-scale. At mm-scale gravity
    // (9810), equilibrium penetration ≈ 0.46 mm — visible floor clipping.
    // Tighten to 0.005 (must be > 2×timestep = 0.004).
    for solref in &mut model.geom_solref {
        solref[0] = 0.005;
    }

    eprintln!();
    eprintln!("  Rest Diagnostics");
    eprintln!("  ----------------");
    eprintln!("  Bodies: {}, Geoms: {}", model.nbody, model.ngeom);
    eprintln!("  Initial z: {:.2} mm", model.qpos0[2]);
    eprintln!(
        "  Gravity: ({:.1}, {:.1}, {:.1}) mm/s²",
        model.gravity.x, model.gravity.y, model.gravity.z
    );
    eprintln!();

    let mut data = model.make_data();
    data.forward(&model).expect("forward kinematics");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "SDF Physics 04 — Rest".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .insert_resource(PhysicsModel(model))
        .insert_resource(PhysicsData(data))
        .insert_resource(RestTracker::default())
        .add_systems(Startup, setup)
        .add_systems(Update, (step_model_data, track_rest))
        .add_systems(PostUpdate, sync_geom_transforms)
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
) {
    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model.0,
        &data.0,
    );

    ExampleScene::new(30.0, 40.0)
        .with_target(Vec3::new(0.0, 3.0, 0.0))
        .with_angles(0.3, 0.4)
        .with_ground_y(-0.1) // slightly below physics z=0 to avoid z-fight
        .spawn(&mut commands, &mut meshes, &mut materials);
}

#[derive(Resource, Default)]
struct RestTracker {
    last_print: f64,
    settled: bool,
}

fn track_rest(data: Res<PhysicsData>, mut tracker: ResMut<RestTracker>) {
    let t = data.0.time;

    // Print every 0.5s for first 5s
    let interval = if t < 2.0 { 0.25 } else { 1.0 };
    if t - tracker.last_print < interval || t > 10.0 {
        return;
    }
    tracker.last_print = t;

    let z = data.0.qpos[2];
    let vz = data.0.qvel[2];
    let ncon = data.0.ncon;
    let penetration = RADIUS - z;

    eprintln!("  t={t:.2}s  z={z:.4}  pen={penetration:.4}  vz={vz:.4}  contacts={ncon}");

    // Check settling at t≈5s
    if t >= 5.0 && !tracker.settled {
        tracker.settled = true;
        eprintln!();
        check(
            &format!("penetration < 0.1 mm (got {penetration:.4})"),
            penetration.abs() < 0.1,
        );
        check(
            &format!("vertical velocity < 1.0 mm/s (got {vz:.4})"),
            vz.abs() < 1.0,
        );
        check(&format!("contact active (ncon = {ncon})"), ncon > 0);
        eprintln!();
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
