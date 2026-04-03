//! Island Groups — Independent Constraint Islands
//!
//! Two stacks of 3 boxes each, placed 3m apart. After settling and sleeping,
//! an impulse disturbs one stack — only that stack wakes (orange). The other
//! remains asleep (blue), demonstrating island independence.
//!
//! Run: `cargo run -p example-sleep-wake-islands --release`

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
    PhysicsAccumulator, PhysicsData, PhysicsModel, spawn_model_geoms_with, step_physics_realtime,
    sync_geom_transforms,
};
use sim_core::SleepState;
use sim_core::validation::{Check, print_report};

// ── MJCF ──────────────────────────────────────────────────────────────────

const MJCF: &str = r#"
<mujoco model="island-groups">
  <option gravity="0 0 -9.81" timestep="0.002" sleep_tolerance="0.05">
    <flag sleep="enable"/>
  </option>
  <worldbody>
    <geom name="floor" type="plane" size="10 10 0.1" solref="0.005 1.5"/>

    <!-- Stack A (left) — dropped from height for visible settling -->
    <body name="a1" pos="-1.5 0 1.0"><freejoint/>
      <geom name="ga1" type="box" size="0.15 0.15 0.15" mass="1" solref="0.005 1.5"/></body>
    <body name="a2" pos="-1.5 0 1.5"><freejoint/>
      <geom name="ga2" type="box" size="0.15 0.15 0.15" mass="1" solref="0.005 1.5"/></body>
    <body name="a3" pos="-1.5 0 2.0"><freejoint/>
      <geom name="ga3" type="box" size="0.15 0.15 0.15" mass="1" solref="0.005 1.5"/></body>

    <!-- Stack B (right) — same heights -->
    <body name="b1" pos="1.5 0 1.0"><freejoint/>
      <geom name="gb1" type="box" size="0.15 0.15 0.15" mass="1" solref="0.005 1.5"/></body>
    <body name="b2" pos="1.5 0 1.5"><freejoint/>
      <geom name="gb2" type="box" size="0.15 0.15 0.15" mass="1" solref="0.005 1.5"/></body>
    <body name="b3" pos="1.5 0 2.0"><freejoint/>
      <geom name="gb3" type="box" size="0.15 0.15 0.15" mass="1" solref="0.005 1.5"/></body>
  </worldbody>
</mujoco>
"#;

const NUM_BODIES: usize = 6;
const IMPULSE_TIME: f64 = 7.0;
const IMPULSE_DURATION: f64 = 0.05;
const IMPULSE_FORCE: f64 = 60.0;
const REPORT_TIME: f64 = 15.0;

// ── Components & Resources ────────────────────────────────────────────────

#[derive(Component)]
struct GeomBodyId(usize);

#[derive(Resource)]
struct SleepMaterials {
    awake: Handle<StandardMaterial>,
    asleep: Handle<StandardMaterial>,
}

#[derive(Resource)]
struct BodyIds {
    a: [usize; 3],
    b: [usize; 3],
}

#[derive(Resource, Default)]
struct IslandValidation {
    all_asleep_by_6: Option<bool>,
    stack_a_woke: Option<bool>,
    stack_b_stayed: Option<bool>,
    islands_zero_pre: Option<bool>,
    stack_a_resleep: Option<bool>,
    reported: bool,
}

// ── Bevy App ──────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Island Groups ===");
    println!("  Two stacks of 3 boxes. At t=7s, Stack A is disturbed.");
    println!("  Stack B stays asleep — islands are independent.\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "Island Groups — Independent Constraint Islands".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .init_resource::<PhysicsHud>()
        .init_resource::<IslandValidation>()
        .insert_resource(
            ValidationHarness::new()
                .report_at(REPORT_TIME)
                .print_every(1.0)
                .display(|_m, d| {
                    format!(
                        "t={:.1}s  awake={}/{}  islands={}",
                        d.time,
                        d.nbody_awake().saturating_sub(1),
                        NUM_BODIES,
                        d.nisland(),
                    )
                }),
        )
        .add_systems(Startup, setup)
        .add_systems(Update, (apply_impulse, step_physics_realtime).chain())
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

    let a = [
        model.body_id("a1").expect("a1"),
        model.body_id("a2").expect("a2"),
        model.body_id("a3").expect("a3"),
    ];
    let b = [
        model.body_id("b1").expect("b1"),
        model.body_id("b2").expect("b2"),
        model.body_id("b3").expect("b3"),
    ];

    let awake_mat = materials.add(MetalPreset::Anodized(Color::srgb(0.95, 0.45, 0.15)).material());
    let asleep_mat = materials.add(MetalPreset::Anodized(Color::srgb(0.35, 0.50, 0.70)).material());
    let floor_mat = materials.add(MetalPreset::CastIron.material());

    commands.insert_resource(SleepMaterials {
        awake: awake_mat.clone(),
        asleep: asleep_mat,
    });

    spawn_model_geoms_with(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[
            ("floor", floor_mat),
            ("ga1", awake_mat.clone()),
            ("ga2", awake_mat.clone()),
            ("ga3", awake_mat.clone()),
            ("gb1", awake_mat.clone()),
            ("gb2", awake_mat.clone()),
            ("gb3", awake_mat),
        ],
        |cmds, geom_id, _name| {
            cmds.insert(GeomBodyId(model.geom_body[geom_id]));
        },
    );

    spawn_example_camera(
        &mut commands,
        Vec3::new(0.0, 0.0, 0.5),
        7.0,
        std::f32::consts::FRAC_PI_2, // 90°: looking along X axis
        0.35,
    );

    spawn_physics_hud(&mut commands);

    commands.insert_resource(BodyIds { a, b });
    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── Impulse ───────────────────────────────────────────────────────────────

fn apply_impulse(mut data: ResMut<PhysicsData>, bodies: Res<BodyIds>) {
    let t = data.0.time;
    let a3 = bodies.a[2];
    if (IMPULSE_TIME..IMPULSE_TIME + IMPULSE_DURATION).contains(&t) {
        data.0.xfrc_applied[a3][5] = IMPULSE_FORCE; // upward force on top of Stack A
    } else {
        data.0.xfrc_applied[a3][5] = 0.0;
    }
}

// ── Sleep Color Coding ────────────────────────────────────────────────────

fn update_sleep_colors(
    data: Res<PhysicsData>,
    mats: Res<SleepMaterials>,
    mut query: Query<(&GeomBodyId, &mut MeshMaterial3d<StandardMaterial>)>,
) {
    for (body_id, mut mat_handle) in &mut query {
        match data.0.body_sleep_state[body_id.0] {
            SleepState::Awake => *mat_handle = MeshMaterial3d(mats.awake.clone()),
            SleepState::Asleep => *mat_handle = MeshMaterial3d(mats.asleep.clone()),
            SleepState::Static => {}
        }
    }
}

// ── Validation ────────────────────────────────────────────────────────────

fn track_validation(
    data: Res<PhysicsData>,
    bodies: Res<BodyIds>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<IslandValidation>,
) {
    let t = data.0.time;

    // All asleep by t=6
    if val.all_asleep_by_6.is_none() && t >= 6.0 {
        val.all_asleep_by_6 = Some(data.0.nbody_awake() == 1);
    }

    // Islands == 0 just before impulse (all sleeping)
    if val.islands_zero_pre.is_none() && t >= 6.5 {
        val.islands_zero_pre = Some(data.0.nisland() == 0);
    }

    // Stack A woke, Stack B stayed asleep at t=7.1
    if val.stack_a_woke.is_none() && t >= 7.1 {
        let a_awake = bodies
            .a
            .iter()
            .all(|&id| data.0.body_sleep_state[id] == SleepState::Awake);
        let b_asleep = bodies
            .b
            .iter()
            .all(|&id| data.0.body_sleep_state[id] == SleepState::Asleep);
        val.stack_a_woke = Some(a_awake);
        val.stack_b_stayed = Some(b_asleep);
    }

    // Stack A re-sleeps by t=14
    if val.stack_a_resleep.is_none() && t >= 14.0 {
        let a_asleep = bodies
            .a
            .iter()
            .all(|&id| data.0.body_sleep_state[id] == SleepState::Asleep);
        val.stack_a_resleep = Some(a_asleep);
    }

    // Print report
    if harness.reported() && !val.reported {
        val.reported = true;
        let checks = vec![
            Check {
                name: "All asleep by t=6",
                pass: val.all_asleep_by_6.unwrap_or(false),
                detail: "nbody_awake == 1 (world only)".into(),
            },
            Check {
                name: "Islands == 0 pre-disturb",
                pass: val.islands_zero_pre.unwrap_or(false),
                detail: "nisland == 0 when all sleeping".into(),
            },
            Check {
                name: "Stack A wakes",
                pass: val.stack_a_woke.unwrap_or(false),
                detail: "all 3 Stack A bodies Awake at t=7.1".into(),
            },
            Check {
                name: "Stack B stays asleep",
                pass: val.stack_b_stayed.unwrap_or(false),
                detail: "all 3 Stack B bodies Asleep at t=7.1".into(),
            },
            Check {
                name: "Stack A re-sleeps",
                pass: val.stack_a_resleep.unwrap_or(false),
                detail: "all 3 Stack A bodies Asleep by t=14".into(),
            },
        ];
        let _ = print_report("Island Groups (t=15s)", &checks);
    }
}

// ── HUD ───────────────────────────────────────────────────────────────────

fn update_hud(data: Res<PhysicsData>, bodies: Res<BodyIds>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();
    hud.section("Island Groups");
    hud.scalar("time", data.0.time, 1);
    hud.raw(format!(
        "awake  {} / {}",
        data.0.nbody_awake().saturating_sub(1),
        NUM_BODIES,
    ));
    hud.raw(format!("islands  {}", data.0.nisland()));

    let state_str = |id: usize| match data.0.body_sleep_state[id] {
        SleepState::Awake => "Awake",
        SleepState::Asleep => "Asleep",
        SleepState::Static => "Static",
    };

    hud.section("Stack A");
    for (i, &id) in bodies.a.iter().enumerate() {
        let marker = if i == 2 && data.0.time >= IMPULSE_TIME {
            " <-"
        } else {
            ""
        };
        hud.raw(format!("  a{}  {}{marker}", i + 1, state_str(id)));
    }
    hud.section("Stack B");
    for (i, &id) in bodies.b.iter().enumerate() {
        hud.raw(format!("  b{}  {}", i + 1, state_str(id)));
    }
}
