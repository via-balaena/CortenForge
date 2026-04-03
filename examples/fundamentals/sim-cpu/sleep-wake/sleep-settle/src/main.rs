//! Sleep Settle — Velocity Threshold and Sleep Transition
//!
//! Five boxes drop from staggered heights and turn blue one by one as they
//! settle below the velocity threshold and sleep.
//!
//! Bodies are color-coded: orange = awake, steel blue = asleep.
//!
//! Run: `cargo run -p example-sleep-wake-settle --release`

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
    GeomBodyId, PhysicsHud, SleepMaterials, ValidationHarness, render_physics_hud,
    spawn_example_camera, spawn_physics_hud, update_sleep_colors, validation_system,
};
use sim_bevy::materials::MetalPreset;
use sim_bevy::model_data::{
    PhysicsAccumulator, PhysicsData, PhysicsModel, spawn_model_geoms_with, step_physics_realtime,
    sync_geom_transforms,
};
use sim_core::SleepState;
use sim_core::validation::{Check, print_report};

// ── MJCF ──────────────────────────────────────────────────────────────────

const MJCF: &str = r#"
<mujoco model="sleep-settle">
  <option gravity="0 0 -9.81" timestep="0.002" sleep_tolerance="0.05">
    <flag sleep="enable"/>
  </option>
  <worldbody>
    <geom name="floor" type="plane" size="5 5 0.1" solref="0.005 1.5"/>
    <body name="box_0" pos="-2 0 1.0"><freejoint/>
      <geom name="g0" type="box" size="0.15 0.15 0.15" mass="1" solref="0.005 1.5"/></body>
    <body name="box_1" pos="-1 0 1.5"><freejoint/>
      <geom name="g1" type="box" size="0.15 0.15 0.15" mass="1" solref="0.005 1.5"/></body>
    <body name="box_2" pos="0 0 2.0"><freejoint/>
      <geom name="g2" type="box" size="0.15 0.15 0.15" mass="1" solref="0.005 1.5"/></body>
    <body name="box_3" pos="1 0 2.5"><freejoint/>
      <geom name="g3" type="box" size="0.15 0.15 0.15" mass="1" solref="0.005 1.5"/></body>
    <body name="box_4" pos="2 0 3.0"><freejoint/>
      <geom name="g4" type="box" size="0.15 0.15 0.15" mass="1" solref="0.005 1.5"/></body>
  </worldbody>
</mujoco>
"#;

const NUM_BOXES: usize = 5;
const REPORT_TIME: f64 = 15.0;

// ── Resources ─────────────────────────────────────────────────────────────

#[derive(Resource, Default)]
struct SettleValidation {
    all_awake_at_start: Option<bool>,
    all_asleep_by_6: Option<bool>,
    sleeping_qvel_zero: Option<bool>,
    sleeping_qacc_zero: Option<bool>,
    reported: bool,
}

// ── Bevy App ──────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Sleep Settle ===");
    println!("  5 boxes drop from staggered heights.");
    println!("  Watch them turn blue as they settle and sleep.\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "Sleep Settle — Velocity Threshold".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .init_resource::<PhysicsHud>()
        .init_resource::<SettleValidation>()
        .insert_resource(
            ValidationHarness::new()
                .report_at(REPORT_TIME)
                .print_every(1.0)
                .display(|_m, d| {
                    format!(
                        "t={:.1}s  awake={}/{}",
                        d.time,
                        d.nbody_awake().saturating_sub(1),
                        NUM_BOXES,
                    )
                }),
        )
        .add_systems(Startup, setup)
        .add_systems(Update, step_physics_realtime)
        .add_systems(
            PostUpdate,
            (
                sync_geom_transforms,
                update_sleep_colors,
                validation_system,
                track_validation,
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

    let awake_mat = materials.add(MetalPreset::Anodized(Color::srgb(0.95, 0.45, 0.15)).material());
    let asleep_mat = materials.add(MetalPreset::Anodized(Color::srgb(0.35, 0.50, 0.70)).material());
    let floor_mat = materials.add(MetalPreset::CastIron.material());

    commands.insert_resource(SleepMaterials {
        awake: awake_mat.clone(),
        asleep: asleep_mat,
    });

    let mut overrides: Vec<(&str, Handle<StandardMaterial>)> = vec![("floor", floor_mat)];
    let geom_names: Vec<String> = (0..NUM_BOXES).map(|i| format!("g{i}")).collect();
    for name in &geom_names {
        overrides.push((name.as_str(), awake_mat.clone()));
    }

    spawn_model_geoms_with(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &overrides,
        |cmds, geom_id, _name| {
            cmds.insert(GeomBodyId(model.geom_body[geom_id]));
        },
    );

    spawn_example_camera(
        &mut commands,
        Vec3::new(0.0, 0.0, 1.0),
        8.0,
        std::f32::consts::FRAC_PI_2, // 90°: looking along X axis
        0.35,
    );

    spawn_physics_hud(&mut commands);

    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── Validation ────────────────────────────────────────────────────────────

fn track_validation(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<SettleValidation>,
) {
    let t = data.0.time;

    if val.all_awake_at_start.is_none() && t >= 0.1 {
        val.all_awake_at_start = Some(data.0.nbody_awake() == NUM_BOXES + 1);
    }

    if val.all_asleep_by_6.is_none() && t >= 6.0 {
        val.all_asleep_by_6 = Some(data.0.nbody_awake() == 1);
    }

    if val.sleeping_qvel_zero.is_none() && t >= 6.5 {
        let mut qvel_ok = true;
        let mut qacc_ok = true;
        for body_id in 1..model.0.nbody {
            if data.0.body_sleep_state[body_id] == SleepState::Asleep {
                let dof_adr = model.0.body_dof_adr[body_id];
                let nv = model.0.body_dof_num[body_id];
                for i in 0..nv {
                    if data.0.qvel[dof_adr + i] != 0.0 {
                        qvel_ok = false;
                    }
                    if data.0.qacc[dof_adr + i] != 0.0 {
                        qacc_ok = false;
                    }
                }
            }
        }
        val.sleeping_qvel_zero = Some(qvel_ok);
        val.sleeping_qacc_zero = Some(qacc_ok);
    }

    if harness.reported() && !val.reported {
        val.reported = true;
        let checks = vec![
            Check {
                name: "All awake at start",
                pass: val.all_awake_at_start.unwrap_or(false),
                detail: format!("nbody_awake at t=0.1: expected {}", NUM_BOXES + 1),
            },
            Check {
                name: "All asleep by t=6",
                pass: val.all_asleep_by_6.unwrap_or(false),
                detail: "nbody_awake == 1 (world only)".into(),
            },
            Check {
                name: "Sleeping qvel = 0",
                pass: val.sleeping_qvel_zero.unwrap_or(false),
                detail: "bitwise zero for all sleeping DOFs at t=6.5".into(),
            },
            Check {
                name: "Sleeping qacc = 0",
                pass: val.sleeping_qacc_zero.unwrap_or(false),
                detail: "bitwise zero for all sleeping DOFs at t=6.5".into(),
            },
        ];
        let _ = print_report("Sleep Settle (t=15s)", &checks);
    }
}

// ── HUD ───────────────────────────────────────────────────────────────────

fn update_hud(model: Res<PhysicsModel>, data: Res<PhysicsData>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();
    hud.section("Sleep Settle");
    hud.scalar("time", data.0.time, 1);
    hud.raw(format!(
        "awake  {} / {}",
        data.0.nbody_awake().saturating_sub(1),
        NUM_BOXES
    ));
    hud.section("Per-Body State");
    for i in 0..NUM_BOXES {
        let body_name = format!("box_{i}");
        let body_id = model.0.body_id(&body_name).expect("body exists");
        let state = match data.0.body_sleep_state[body_id] {
            SleepState::Awake => "Awake",
            SleepState::Asleep => "Asleep",
            SleepState::Static => "Static",
        };
        hud.raw(format!("  {body_name}  {state}"));
    }
}
