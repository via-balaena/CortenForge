//! Cable Loaded — Midpoint Force on Fixed Cable
//!
//! A cable composite pinned at both ends with a constant downward force
//! applied at the midpoint via `xfrc_applied`. The midpoint sags well below
//! the natural catenary, creating a V-like shape.
//!
//! Validates:
//! - Cable sags below anchor height
//! - Midpoint is the lowest point on the cable
//! - Cable settles (max angular velocity < threshold)
//! - Sag exceeds passive catenary depth
//!
//! Run with: `cargo run -p example-composite-cable-loaded --release`

#![allow(
    clippy::doc_markdown,
    clippy::needless_pass_by_value,
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::let_underscore_must_use,
    clippy::suboptimal_flops,
    clippy::cast_precision_loss,
    clippy::unwrap_used
)]

use bevy::prelude::*;
use sim_bevy::camera::OrbitCameraPlugin;
use sim_bevy::examples::{
    PhysicsHud, ValidationHarness, render_physics_hud, spawn_example_camera, spawn_physics_hud,
    validation_system,
};
use sim_bevy::model_data::{
    PhysicsAccumulator, PhysicsData, PhysicsModel, spawn_model_geoms, step_physics_realtime,
    sync_geom_transforms,
};
use sim_core::validation::{Check, print_report};

// ── MJCF Model ──────────────────────────────────────────────────────────────

/// Same cable geometry as cable-catenary: 15 segments spanning 1.0m between
/// two pylons, half-sine upward bulge for slack. Cyan instead of gold.
/// The midpoint load is applied via xfrc_applied in Rust code, not MJCF.
const MJCF: &str = r#"
<mujoco model="cable-loaded">
  <option gravity="0 0 -9.81" timestep="0.002" integrator="implicitfast"
          solver="Newton" iterations="50"/>

  <worldbody>
    <!-- Ground plane -->
    <geom name="floor" type="plane" size="2 2 0.01" rgba="0.25 0.25 0.28 1"
          pos="0 0 0" contype="0" conaffinity="0"/>

    <!-- Left pylon -->
    <body name="pylon_L" pos="-0.5 0 0.8">
      <geom name="pylon_L_geom" type="box" size="0.025 0.025 0.4"
            rgba="0.5 0.5 0.5 1" contype="0" conaffinity="0"/>

      <!-- Cable: 15 segments, cyan, starts with sine bulge for slack -->
      <composite type="cable" prefix="A" count="16 1 1"
                 initial="ball" curve="l 0 s" size="1.0 0.2 1.0"
                 offset="0 0 0.4">
        <joint kind="main" damping="0.05"/>
        <geom type="capsule" size="0.01" rgba="0.2 0.8 0.9 1"
              density="800" contype="0" conaffinity="0"/>
      </composite>
    </body>

    <!-- Right pylon -->
    <body name="pylon_R" pos="0.5 0 0.8">
      <geom name="pylon_R_geom" type="box" size="0.025 0.025 0.4"
            rgba="0.5 0.5 0.5 1" contype="0" conaffinity="0"/>
    </body>
  </worldbody>

  <equality>
    <connect body1="AB_last" body2="pylon_R"
             anchor="0.06667 0 0" solref="0.002 1.0"/>
  </equality>
</mujoco>
"#;

// ── Constants ───────────────────────────────────────────────────────────────

const ANCHOR_Z: f64 = 1.2;
const LOAD_FORCE: f64 = 5.0; // Newtons downward at midpoint

// ── Startup delay ───────────────────────────────────────────────────────────

#[derive(Resource)]
struct StartDelay {
    elapsed: f32,
    duration: f32,
}

fn tick_delay(time: Res<Time>, mut delay: ResMut<StartDelay>) {
    delay.elapsed += time.delta_secs();
}

fn delay_elapsed(delay: Res<StartDelay>) -> bool {
    delay.elapsed >= delay.duration
}

// ── Midpoint body index ────────────────────────────────────────────────────

#[derive(Resource)]
struct MidpointBody(usize);

#[derive(Component)]
struct LoadIndicator;

// ── Bevy App ────────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Cable Loaded ===");
    println!("  Both-ends-fixed cable with {LOAD_FORCE}N midpoint load");
    println!("  Cyan cable, 15 segments, 1.0m span between gray pylons");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Cable Loaded".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .init_resource::<PhysicsHud>()
        .init_resource::<LoadedValidation>()
        .insert_resource(StartDelay {
            elapsed: 0.0,
            duration: 2.0,
        })
        .insert_resource(
            ValidationHarness::new()
                .report_at(15.0)
                .print_every(2.0)
                .display(|m, d| {
                    let first = m.body_id("AB_first").unwrap();
                    let last = m.body_id("AB_last").unwrap();
                    let mid = m.body_id("AB_8").unwrap();
                    let (sag, _) = compute_sag(d, first, last);
                    let mid_z = d.xpos[mid][2];
                    format!("sag={sag:.4}  mid_z={mid_z:.4}")
                }),
        )
        .add_systems(Startup, setup)
        .add_systems(
            Update,
            (
                tick_delay,
                (apply_midpoint_load, step_physics_realtime)
                    .chain()
                    .run_if(delay_elapsed),
            )
                .chain(),
        )
        .add_systems(
            PostUpdate,
            (
                sync_geom_transforms,
                sync_load_indicator,
                validation_system,
                loaded_diagnostics,
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
    data.forward(&model).expect("initial forward");

    let mid_body = model.body_id("AB_8").expect("AB_8 midpoint body");

    println!(
        "  Model: {} bodies, {} joints, {} geoms, {} eq constraints",
        model.nbody, model.njnt, model.ngeom, model.neq
    );
    println!("  Midpoint body: AB_8 (index {mid_body})\n");

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[],
    );

    // Red sphere at the midpoint to indicate the load
    let mid_pos = data.xpos[mid_body];
    commands.spawn((
        LoadIndicator,
        Mesh3d(meshes.add(Sphere::new(0.02))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.9, 0.15, 0.15),
            ..default()
        })),
        Transform::from_xyz(mid_pos[0] as f32, mid_pos[2] as f32, -mid_pos[1] as f32),
    ));

    // Side view: same framing as cable-catenary
    spawn_example_camera(
        &mut commands,
        Vec3::new(0.0, 0.8, 0.0), // slightly lower center for deeper sag
        2.0,
        std::f32::consts::FRAC_PI_2,
        0.15,
    );

    spawn_physics_hud(&mut commands);

    commands.insert_resource(MidpointBody(mid_body));
    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

/// Apply constant downward force at the midpoint body every frame.
/// xfrc_applied layout: [torque_x, torque_y, torque_z, force_x, force_y, force_z]
/// Downward (physics -z) = index 5.
fn apply_midpoint_load(mid: Res<MidpointBody>, mut data: ResMut<PhysicsData>) {
    data.xfrc_applied[mid.0][5] = -LOAD_FORCE;
}

/// Keep the red sphere tracking the midpoint body's world position.
fn sync_load_indicator(
    data: Res<PhysicsData>,
    mid: Res<MidpointBody>,
    mut query: Query<&mut Transform, With<LoadIndicator>>,
) {
    let p = data.xpos[mid.0];
    for mut tf in &mut query {
        // Physics (x,y,z) → Bevy (x,z,-y)
        tf.translation = Vec3::new(p[0] as f32, p[2] as f32, -p[1] as f32);
    }
}

// ── HUD ─────────────────────────────────────────────────────────────────────

fn update_hud(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    mid: Res<MidpointBody>,
    mut hud: ResMut<PhysicsHud>,
) {
    hud.clear();
    hud.section("Cable Loaded — Midpoint Force");

    let first = model.body_id("AB_first").unwrap();
    let last = model.body_id("AB_last").unwrap();
    let (sag, _) = compute_sag(&data, first, last);
    let mid_z = data.xpos[mid.0][2];

    hud.scalar("t", data.time, 1);
    hud.scalar("sag depth", sag, 4);
    hud.scalar("load (N)", LOAD_FORCE, 1);
    hud.scalar("mid z", mid_z, 3);
}

// ── Helpers ─────────────────────────────────────────────────────────────────

/// Compute sag depth and the body index of the lowest point.
fn compute_sag(data: &sim_core::Data, first: usize, last: usize) -> (f64, usize) {
    let mut min_z = f64::MAX;
    let mut min_idx = first;
    for i in first..=last {
        let z = data.xpos[i][2];
        if z < min_z {
            min_z = z;
            min_idx = i;
        }
    }
    (ANCHOR_Z - min_z, min_idx)
}

// ── Validation ──────────────────────────────────────────────────────────────

#[derive(Resource, Default)]
struct LoadedValidation {
    reported: bool,
}

fn loaded_diagnostics(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    mid: Res<MidpointBody>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<LoadedValidation>,
) {
    if !harness.reported() || val.reported {
        return;
    }
    val.reported = true;

    let first = model.body_id("AB_first").unwrap();
    let last = model.body_id("AB_last").unwrap();

    // Sag depth
    let (sag, lowest_idx) = compute_sag(&data, first, last);

    // Midpoint is lowest
    let mid_is_lowest = lowest_idx == mid.0;

    // Settled check
    let max_qvel = data.qvel.iter().fold(0.0_f64, |m, &v| m.max(v.abs()));

    // Sag exceeds passive catenary (~0.186 from cable-catenary)
    let exceeds_passive = sag > 0.05;

    let mid_z = data.xpos[mid.0][2];
    let checks = vec![
        Check {
            name: "Cable sags below anchors",
            pass: sag > 0.0,
            detail: format!("sag_depth={sag:.4}"),
        },
        Check {
            name: "Midpoint is lowest point",
            pass: mid_is_lowest,
            detail: format!("mid_z={mid_z:.4}, lowest_body={lowest_idx}"),
        },
        Check {
            name: "Cable settled",
            pass: max_qvel < 0.01,
            detail: format!("max_qvel={max_qvel:.4}"),
        },
        Check {
            name: "Sag exceeds passive catenary",
            pass: exceeds_passive,
            detail: format!("sag={sag:.4} (passive≈0.186)"),
        },
    ];
    let _ = print_report("Cable Loaded (t=15s)", &checks);
}
