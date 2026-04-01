//! Exclude Pairs — Explicit Body-Pair Exclusion
//!
//! Side-by-side comparison: two identical mocap platforms, two falling spheres.
//! The left sphere has an `<exclude>` with its platform — it passes cleanly
//! through. The right sphere has no exclusion — it lands and rests on top.
//!
//! Run with: `cargo run -p example-contact-filtering-exclude-pairs --release`

#![allow(
    clippy::doc_markdown,
    clippy::needless_pass_by_value,
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::let_underscore_must_use,
    clippy::suboptimal_flops,
    clippy::cast_sign_loss,
    clippy::similar_names
)]

use bevy::prelude::*;
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

// ── MJCF Model ────────────────────────────────────────────────────────────

/// Two mocap platforms (fixed in space, no physics jitter).
/// Two spheres drop from above in slow gravity.
/// Left sphere is excluded from left platform → passes through.
/// Right sphere is NOT excluded → collides and rests on top.
const MJCF: &str = r#"
<mujoco model="exclude-pairs-demo">
  <compiler angle="radian"/>
  <option gravity="0 0 -2.0" timestep="0.002"/>

  <worldbody>
    <!-- Left platform: mocap (fixed), excluded from left_sphere -->
    <body name="plat_left" mocap="true" pos="-0.4 0 0">
      <geom name="pl_geom" type="box" size="0.2 0.2 0.04" mass="1.0"
            rgba="0.3 0.5 0.85 1"/>
    </body>

    <!-- Right platform: mocap (fixed), NOT excluded -->
    <body name="plat_right" mocap="true" pos="0.4 0 0">
      <geom name="pr_geom" type="box" size="0.2 0.2 0.04" mass="1.0"
            rgba="0.3 0.5 0.85 1"/>
    </body>

    <!-- Left sphere: excluded from left platform -->
    <body name="sphere_excluded" pos="-0.4 0 1.0">
      <freejoint name="ex_free"/>
      <geom name="ex_geom" type="sphere" size="0.08" mass="0.5"
            rgba="0.85 0.25 0.2 0.6"/>
    </body>

    <!-- Right sphere: normal collision with right platform -->
    <body name="sphere_normal" pos="0.4 0 1.0">
      <freejoint name="nm_free"/>
      <geom name="nm_geom" type="sphere" size="0.08" mass="0.5"
            rgba="0.85 0.7 0.2 1"/>
    </body>
  </worldbody>

  <contact>
    <exclude body1="plat_left" body2="sphere_excluded"/>
  </contact>
</mujoco>
"#;

// ── Bevy App ──────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Exclude Pairs — Body-Pair Exclusion ===");
    println!("  Left:  red sphere + blue platform = EXCLUDED (passes through)");
    println!("  Right: gold sphere + blue platform = normal (lands on top)");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Exclude Pairs (Contact Filtering)".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .init_resource::<PhysicsHud>()
        .insert_resource(
            ValidationHarness::new()
                .report_at(5.0)
                .print_every(0.5)
                .display(|m, d| {
                    let ex = d.xpos[m.body_id("sphere_excluded").unwrap()].z;
                    let nm = d.xpos[m.body_id("sphere_normal").unwrap()].z;
                    format!("excluded={ex:.3} normal={nm:.3} ncon={}", d.ncon)
                }),
        )
        .add_systems(Startup, setup)
        .add_systems(Update, step_physics_realtime)
        .add_systems(
            PostUpdate,
            (
                sync_geom_transforms,
                validation_system,
                diagnostics,
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

    let mat_platform =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.3, 0.5, 0.85)));
    let mat_excluded = materials.add(StandardMaterial {
        base_color: Color::srgba(0.85, 0.25, 0.2, 0.6),
        alpha_mode: AlphaMode::Blend,
        metallic: 0.6,
        perceptual_roughness: 0.35,
        ..default()
    });
    let mat_normal =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.85, 0.7, 0.2)));

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[
            ("pl_geom", mat_platform.clone()),
            ("pr_geom", mat_platform),
            ("ex_geom", mat_excluded),
            ("nm_geom", mat_normal),
        ],
    );

    spawn_example_camera(
        &mut commands,
        physics_pos(0.0, 0.0, 0.3),
        4.0,
        std::f32::consts::FRAC_PI_2,
        0.15,
    );
    spawn_physics_hud(&mut commands);

    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── HUD ───────────────────────────────────────────────────────────────────

fn update_hud(model: Res<PhysicsModel>, data: Res<PhysicsData>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();
    hud.section("Exclude Pairs — Body-Pair Exclusion");

    let pl = model.body_id("plat_left").expect("plat_left");
    let ex = model.body_id("sphere_excluded").expect("excluded");
    let pr = model.body_id("plat_right").expect("plat_right");
    let nm = model.body_id("sphere_normal").expect("normal");

    hud.raw(String::from("LEFT (excluded):"));
    hud.raw(format!("  sphere z={:.3}", data.xpos[ex].z));
    let ex_con = data.contacts_between_bodies(&model, pl, ex);
    hud.raw(format!("  contacts: {ex_con}"));

    hud.raw(String::from("RIGHT (normal):"));
    hud.raw(format!("  sphere z={:.3}", data.xpos[nm].z));
    let nm_con = data.contacts_between_bodies(&model, pr, nm);
    hud.raw(format!("  contacts: {nm_con}"));

    hud.scalar("time", data.time, 1);
}

// ── Validation ────────────────────────────────────────────────────────────

#[derive(Default)]
struct DiagState {
    max_excluded_contacts: usize,
    normal_ever_collided: bool,
    reported: bool,
}

fn diagnostics(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    harness: Res<ValidationHarness>,
    mut state: Local<DiagState>,
) {
    let pl = model.body_id("plat_left").expect("plat_left");
    let ex = model.body_id("sphere_excluded").expect("excluded");
    let pr = model.body_id("plat_right").expect("plat_right");
    let nm = model.body_id("sphere_normal").expect("normal");

    state.max_excluded_contacts = state
        .max_excluded_contacts
        .max(data.contacts_between_bodies(&model, pl, ex));

    if data.contacts_between_bodies(&model, pr, nm) > 0 {
        state.normal_ever_collided = true;
    }

    if !harness.reported() || state.reported {
        return;
    }
    state.reported = true;

    let ex_z = data.xpos[ex].z;
    let nm_z = data.xpos[nm].z;
    let plat_z = data.xpos[pl].z; // mocap, should be 0.0

    let checks = vec![
        Check {
            name: "Excluded sphere fell through platform",
            pass: ex_z < plat_z - 1.0,
            detail: format!("sphere z = {ex_z:.3}, platform z = {plat_z:.3}"),
        },
        Check {
            name: "Normal sphere resting on platform",
            pass: nm_z > plat_z,
            detail: format!("sphere z = {nm_z:.3}, platform z = {plat_z:.3}"),
        },
        Check {
            name: "Zero contacts between excluded pair (entire run)",
            pass: state.max_excluded_contacts == 0,
            detail: format!("max = {}", state.max_excluded_contacts),
        },
        Check {
            name: "Normal pair collided at least once",
            pass: state.normal_ever_collided,
            detail: format!("collided = {}", state.normal_ever_collided),
        },
    ];
    let _ = print_report("Exclude Pairs (t=5s)", &checks);
}
