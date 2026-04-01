//! Ghost Layers — Collision Layers via Bitmasks
//!
//! Two physics layers defined by contype/conaffinity bitmasks:
//! - Solid layer (contype=1, conaffinity=3): collides with everything.
//! - Ghost layer (contype=2, conaffinity=1): collides with solids, passes
//!   through other ghosts.
//!
//! Two solid boxes stack on the ground. Two ghost spheres rest on the solid
//! stack but pass through each other.
//!
//! Run with: `cargo run -p example-contact-filtering-ghost-layers --release`

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

/// Layer math:
///   Solid(ct=1,ca=3) vs Solid(ct=1,ca=3): (1&3)||(1&3) = true  ✓
///   Solid(ct=1,ca=3) vs Ghost(ct=2,ca=1): (1&1)||(2&3) = true  ✓
///   Ghost(ct=2,ca=1) vs Ghost(ct=2,ca=1): (2&1)||(2&1) = false ✓
///
/// Two solid boxes stacked at x=0.
/// Two ghost spheres dropped at x=-0.05 and x=+0.05 onto the solid stack.
/// Ghosts pass through each other but rest on the solid surface.
const MJCF: &str = r#"
<mujoco model="ghost-layers-demo">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.002"/>

  <worldbody>
    <geom name="ground" type="plane" size="5 5 0.1"
          contype="1" conaffinity="3" rgba="0.25 0.25 0.25 1"/>

    <!-- Solid layer: contype=1, conaffinity=3 -->
    <body name="solid_a" pos="0 0 0.15">
      <freejoint name="sa_free"/>
      <geom name="sa_geom" type="box" size="0.2 0.2 0.15" mass="2.0"
            contype="1" conaffinity="3" rgba="0.3 0.5 0.85 1"/>
    </body>

    <body name="solid_b" pos="0 0 0.5">
      <freejoint name="sb_free"/>
      <geom name="sb_geom" type="box" size="0.15 0.15 0.1" mass="1.0"
            contype="1" conaffinity="3" rgba="0.85 0.7 0.2 1"/>
    </body>

    <!-- Ghost layer: contype=2, conaffinity=1 -->
    <body name="ghost_a" pos="-0.05 0 1.0">
      <freejoint name="ga_free"/>
      <geom name="ga_geom" type="sphere" size="0.08" mass="0.5"
            contype="2" conaffinity="1" rgba="0.2 0.8 0.4 0.5"/>
    </body>

    <body name="ghost_b" pos="0.05 0 1.0">
      <freejoint name="gb_free"/>
      <geom name="gb_geom" type="sphere" size="0.08" mass="0.5"
            contype="2" conaffinity="1" rgba="0.2 0.8 0.4 0.5"/>
    </body>
  </worldbody>
</mujoco>
"#;

// ── Bevy App ──────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Ghost Layers — Collision Layers via Bitmasks ===");
    println!("  Solid layer (blue, gold): contype=1, conaffinity=3");
    println!("  Ghost layer (green):      contype=2, conaffinity=1");
    println!("  Solids stack. Ghosts rest on solids but pass through each other.");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Ghost Layers (Contact Filtering)".into(),
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
                    let gaz = d.xpos[m.body_id("ghost_a").unwrap()].z;
                    let gbz = d.xpos[m.body_id("ghost_b").unwrap()].z;
                    format!("ghost_a={gaz:.3} ghost_b={gbz:.3} ncon={}", d.ncon)
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

    let mat_sa = materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.3, 0.5, 0.85)));
    let mat_sb = materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.85, 0.7, 0.2)));
    let ghost_mat = materials.add(StandardMaterial {
        base_color: Color::srgba(0.2, 0.8, 0.4, 0.4),
        alpha_mode: AlphaMode::Blend,
        metallic: 0.3,
        perceptual_roughness: 0.5,
        ..default()
    });

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[
            ("sa_geom", mat_sa),
            ("sb_geom", mat_sb),
            ("ga_geom", ghost_mat.clone()),
            ("gb_geom", ghost_mat),
        ],
    );

    spawn_example_camera(
        &mut commands,
        physics_pos(0.0, 0.0, 0.5),
        3.0,
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
    hud.section("Ghost Layers — Collision Layers");

    let sa = model.body_id("solid_a").expect("solid_a");
    let sb = model.body_id("solid_b").expect("solid_b");
    let ga = model.body_id("ghost_a").expect("ghost_a");
    let gb = model.body_id("ghost_b").expect("ghost_b");

    hud.raw(format!("solid_a (blue): z={:.3}", data.xpos[sa].z));
    hud.raw(format!("solid_b (gold): z={:.3}", data.xpos[sb].z));
    hud.raw(format!("ghost_a (green): z={:.3}", data.xpos[ga].z));
    hud.raw(format!("ghost_b (green): z={:.3}", data.xpos[gb].z));

    let ga_geom = model.geom_id("ga_geom").expect("ga_geom");
    let gb_geom = model.geom_id("gb_geom").expect("gb_geom");
    let ghost_ghost = data.contacts_between_geoms(ga_geom, gb_geom);
    hud.raw(format!("ghost<>ghost: {ghost_ghost} contacts"));

    hud.scalar("ncon", data.ncon as f64, 0);
    hud.scalar("time", data.time, 1);
}

// ── Validation ────────────────────────────────────────────────────────────

#[derive(Default)]
struct DiagState {
    max_ghost_ghost_contacts: usize,
    reported: bool,
}

fn diagnostics(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    harness: Res<ValidationHarness>,
    mut state: Local<DiagState>,
) {
    let ga_geom = model.geom_id("ga_geom").expect("ga_geom");
    let gb_geom = model.geom_id("gb_geom").expect("gb_geom");
    let gg = data.contacts_between_geoms(ga_geom, gb_geom);
    state.max_ghost_ghost_contacts = state.max_ghost_ghost_contacts.max(gg);

    if !harness.reported() || state.reported {
        return;
    }
    state.reported = true;

    let sa = model.body_id("solid_a").expect("solid_a");
    let sb = model.body_id("solid_b").expect("solid_b");
    let ga = model.body_id("ghost_a").expect("ghost_a");
    let gb = model.body_id("ghost_b").expect("ghost_b");

    let sa_z = data.xpos[sa].z;
    let sb_z = data.xpos[sb].z;
    let ga_z = data.xpos[ga].z;
    let gb_z = data.xpos[gb].z;

    let checks = vec![
        Check {
            name: "Solid boxes stacked (solid_b above solid_a)",
            pass: sb_z > sa_z + 0.1,
            detail: format!("sa_z={sa_z:.3}, sb_z={sb_z:.3}"),
        },
        Check {
            name: "Ghost A resting on solid surface (not falling)",
            pass: ga_z > 0.2,
            detail: format!("ghost_a z={ga_z:.3}"),
        },
        Check {
            name: "Ghost B resting on solid surface (not falling)",
            pass: gb_z > 0.2,
            detail: format!("ghost_b z={gb_z:.3}"),
        },
        Check {
            name: "Ghosts at similar Z (passed through each other, not stacked)",
            pass: (ga_z - gb_z).abs() < 0.1,
            detail: format!("delta_z = {:.4}", (ga_z - gb_z).abs()),
        },
        Check {
            name: "Zero ghost-ghost contacts (entire run)",
            pass: state.max_ghost_ghost_contacts == 0,
            detail: format!("max ghost<>ghost = {}", state.max_ghost_ghost_contacts),
        },
    ];
    let _ = print_report("Ghost Layers (t=5s)", &checks);
}
