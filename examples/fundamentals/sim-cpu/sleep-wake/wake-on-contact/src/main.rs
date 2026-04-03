//! Wake-on-Contact — Contact-Triggered Reactivation
//!
//! A box with sleep="init" sits on a plane, frozen asleep (blue). A ball
//! falls from above and strikes it — the contact wakes the box (turns orange).
//!
//! Run: `cargo run -p example-sleep-wake-contact --release`

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
<mujoco model="wake-on-contact">
  <option gravity="0 0 -9.81" timestep="0.002" sleep_tolerance="0.05">
    <flag sleep="enable"/>
  </option>
  <worldbody>
    <geom name="floor" type="plane" size="5 5 0.1" solref="0.005 1.5"/>
    <body name="box" pos="0 0 0.15" sleep="init">
      <freejoint/>
      <geom name="box_geom" type="box" size="0.15 0.15 0.15" mass="2" solref="0.005 1.5"/>
    </body>
    <body name="ball" pos="0 0 2.0">
      <freejoint/>
      <geom name="ball_geom" type="sphere" size="0.1" mass="0.5" solref="0.005 1.5"/>
    </body>
  </worldbody>
</mujoco>
"#;

const REPORT_TIME: f64 = 15.0;

// ── Resources ─────────────────────────────────────────────────────────────

#[derive(Resource)]
struct ContactValidation {
    box_id: usize,
    ball_id: usize,
    box_starts_asleep: Option<bool>,
    ball_starts_awake: Option<bool>,
    box_frozen: Option<bool>,
    box_woke: Option<bool>,
    both_resleep: Option<bool>,
    impact_time: Option<f64>,
    reported: bool,
}

// ── Bevy App ──────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Wake-on-Contact ===");
    println!("  A sleeping box (blue) is struck by a falling ball (orange).");
    println!("  The contact wakes the box.\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "Wake-on-Contact — Sleep Reactivation".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .init_resource::<PhysicsHud>()
        .insert_resource(
            ValidationHarness::new()
                .report_at(REPORT_TIME)
                .print_every(1.0)
                .display(|_m, d| {
                    format!(
                        "t={:.2}s  awake={}/2  contacts={}",
                        d.time,
                        d.nbody_awake().saturating_sub(1),
                        d.ncon,
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

    let box_id = model.body_id("box").expect("box");
    let ball_id = model.body_id("ball").expect("ball");

    let awake_mat = materials.add(MetalPreset::Anodized(Color::srgb(0.95, 0.45, 0.15)).material());
    let asleep_mat = materials.add(MetalPreset::Anodized(Color::srgb(0.35, 0.50, 0.70)).material());
    let floor_mat = materials.add(MetalPreset::CastIron.material());

    commands.insert_resource(SleepMaterials {
        awake: awake_mat.clone(),
        asleep: asleep_mat.clone(),
    });

    spawn_model_geoms_with(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[
            ("floor", floor_mat),
            ("box_geom", asleep_mat),
            ("ball_geom", awake_mat),
        ],
        |cmds, geom_id, _name| {
            cmds.insert(GeomBodyId(model.geom_body[geom_id]));
        },
    );

    spawn_example_camera(
        &mut commands,
        Vec3::new(0.0, 0.0, 0.5),
        4.0,
        0.52, // ~30 degrees
        0.35, // ~20 degrees
    );

    spawn_physics_hud(&mut commands);

    commands.insert_resource(ContactValidation {
        box_id,
        ball_id,
        box_starts_asleep: None,
        ball_starts_awake: None,
        box_frozen: None,
        box_woke: None,
        both_resleep: None,
        impact_time: None,
        reported: false,
    });
    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── Validation ────────────────────────────────────────────────────────────

fn track_validation(
    data: Res<PhysicsData>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<ContactValidation>,
) {
    let t = data.0.time;

    // Check initial states (first frame)
    if val.box_starts_asleep.is_none() && t > 0.0 {
        val.box_starts_asleep = Some(data.0.body_sleep_state[val.box_id] == SleepState::Asleep);
        val.ball_starts_awake = Some(data.0.body_sleep_state[val.ball_id] == SleepState::Awake);
    }

    // Check box is frozen at t=0.3 (before impact)
    if val.box_frozen.is_none() && t >= 0.3 {
        let box_z = data.0.xpos[val.box_id].z;
        val.box_frozen = Some((box_z - 0.15).abs() < 1e-12);
    }

    // Track impact: box transitions from asleep to awake
    if val.impact_time.is_none() && data.0.body_sleep_state[val.box_id] == SleepState::Awake {
        val.impact_time = Some(t);
        val.box_woke = Some(true);
    }

    // Check both re-sleep by t=14
    if val.both_resleep.is_none() && t >= 14.0 {
        val.both_resleep = Some(data.0.nbody_awake() == 1);
    }

    // Print report
    if harness.reported() && !val.reported {
        val.reported = true;
        let impact_str = val
            .impact_time
            .map_or_else(|| "never".to_string(), |t| format!("{t:.3}s"));
        let checks = vec![
            Check {
                name: "Box starts asleep",
                pass: val.box_starts_asleep.unwrap_or(false),
                detail: "sleep_state(box) == Asleep at t=0".into(),
            },
            Check {
                name: "Ball starts awake",
                pass: val.ball_starts_awake.unwrap_or(false),
                detail: "sleep_state(ball) == Awake at t=0".into(),
            },
            Check {
                name: "Box frozen while asleep",
                pass: val.box_frozen.unwrap_or(false),
                detail: "box z == 0.15 at t=0.3 (no gravity drift)".into(),
            },
            Check {
                name: "Box wakes on impact",
                pass: val.box_woke.unwrap_or(false),
                detail: format!("impact at t={impact_str}"),
            },
            Check {
                name: "Both re-sleep by t=14",
                pass: val.both_resleep.unwrap_or(false),
                detail: "nbody_awake == 1".into(),
            },
        ];
        let _ = print_report("Wake-on-Contact (t=15s)", &checks);
    }
}

// ── HUD ───────────────────────────────────────────────────────────────────

fn update_hud(data: Res<PhysicsData>, val: Res<ContactValidation>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();
    hud.section("Wake-on-Contact");
    hud.scalar("time", data.0.time, 2);
    let box_state = match data.0.body_sleep_state[val.box_id] {
        SleepState::Awake => "Awake",
        SleepState::Asleep => "Asleep",
        SleepState::Static => "Static",
    };
    let ball_state = match data.0.body_sleep_state[val.ball_id] {
        SleepState::Awake => "Awake",
        SleepState::Asleep => "Asleep",
        SleepState::Static => "Static",
    };
    hud.raw(format!("box    {box_state}"));
    hud.raw(format!("ball   {ball_state}"));
    hud.raw(format!("contacts  {}", data.0.ncon));
    if let Some(t) = val.impact_time {
        hud.raw(format!("impact  {t:.3}s"));
    }
}
