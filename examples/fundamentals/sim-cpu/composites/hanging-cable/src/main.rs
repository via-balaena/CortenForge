//! Hanging Cable — Pendulum Chain Under Gravity
//!
//! Three cable composites pinned at one end, hanging freely under gravity.
//! Different segment counts (5, 10, 20) show how resolution affects the
//! equilibrium shape. More segments produce a smoother curve that converges
//! toward the continuous catenary.
//!
//! Validates:
//! - All cable tips hang below anchor (gravity works)
//! - 10-seg tip closer to 20-seg tip than 5-seg (convergence)
//! - All cables settle (max angular velocity < threshold)
//!
//! Run with: `cargo run -p example-composite-hanging-cable --release`

#![allow(
    clippy::doc_markdown,
    clippy::needless_pass_by_value,
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::let_underscore_must_use,
    clippy::suboptimal_flops,
    clippy::cast_precision_loss,
    clippy::struct_field_names
)]

use bevy::prelude::*;
use sim_bevy::camera::OrbitCameraPlugin;
use sim_bevy::examples::{
    PhysicsDelay, PhysicsHud, ValidationHarness, physics_delay_elapsed, render_physics_hud,
    spawn_example_camera, spawn_physics_hud, tick_physics_delay, validation_system,
};
use sim_bevy::model_data::{
    PhysicsAccumulator, PhysicsData, PhysicsModel, spawn_model_geoms, step_physics_realtime,
    sync_geom_transforms,
};
use sim_core::validation::{Check, print_report};

// ── MJCF Model ──────────────────────────────────────────────────────────────

/// Three cables side by side: 5 segments (red), 10 segments (green), 20
/// segments (blue). All pinned at one end (`initial="none"`), same total
/// length (0.8m), same density, same damping. Capsule geoms for a rope-like
/// look. Ground plane for spatial reference.
const MJCF: &str = r#"
<mujoco model="hanging-cable">
  <option gravity="0 0 -9.81" timestep="0.002" integrator="implicitfast"/>

  <worldbody>
    <!-- Ground plane for spatial reference -->
    <geom name="floor" type="plane" size="2 2 0.01" rgba="0.25 0.25 0.28 1"
          pos="0 0 0" contype="0" conaffinity="0"/>

    <!-- 5-segment cable (red) — count=6 vertices → 5 bodies -->
    <body name="anchor_5" pos="-0.4 0 1.2">
      <composite type="cable" prefix="C5" count="6 1 1"
                 initial="none" curve="l 0 0" size="0.8">
        <joint kind="main" damping="0.4"/>
        <geom type="capsule" size="0.012" rgba="0.9 0.25 0.2 1"
              density="500" contype="0" conaffinity="0"/>
      </composite>
    </body>

    <!-- 10-segment cable (green) — count=11 vertices → 10 bodies -->
    <body name="anchor_10" pos="0 0 1.2">
      <composite type="cable" prefix="C10" count="11 1 1"
                 initial="none" curve="l 0 0" size="0.8">
        <joint kind="main" damping="0.4"/>
        <geom type="capsule" size="0.012" rgba="0.2 0.8 0.3 1"
              density="500" contype="0" conaffinity="0"/>
      </composite>
    </body>

    <!-- 20-segment cable (blue) — count=21 vertices → 20 bodies -->
    <body name="anchor_20" pos="0.4 0 1.2">
      <composite type="cable" prefix="C20" count="21 1 1"
                 initial="none" curve="l 0 0" size="0.8">
        <joint kind="main" damping="0.4"/>
        <geom type="capsule" size="0.012" rgba="0.2 0.4 0.9 1"
              density="500" contype="0" conaffinity="0"/>
      </composite>
    </body>

    <!-- Anchor bar for visual context -->
    <geom name="bar" type="box" size="0.55 0.015 0.015"
          pos="0 0 1.2" rgba="0.5 0.5 0.5 1"
          contype="0" conaffinity="0"/>
  </worldbody>
</mujoco>
"#;

// ── Cable body indices ──────────────────────────────────────────────────────

/// Body layout (from MJCF expansion):
///   0 = world
///   1 = anchor_5     →  2..6   = C5 cable (5 bodies)
///   7 = anchor_10    →  8..17  = C10 cable (10 bodies)
///  18 = anchor_20    → 19..38  = C20 cable (20 bodies)
///
/// We look up by name at startup to avoid hard-coding indices.
struct CableInfo {
    /// Body index of the last cable body (tip)
    tip_5: usize,
    tip_10: usize,
    tip_20: usize,
}

impl CableInfo {
    fn from_model(model: &sim_core::Model) -> Self {
        Self {
            tip_5: model.body_id("C5B_last").expect("C5B_last"),
            tip_10: model.body_id("C10B_last").expect("C10B_last"),
            tip_20: model.body_id("C20B_last").expect("C20B_last"),
        }
    }
}

// ── Bevy App ────────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Hanging Cable ===");
    println!("  Three cables pinned at one end, hanging under gravity");
    println!("  Red: 5 segments | Green: 10 segments | Blue: 20 segments");
    println!("  More segments = smoother curve, converging to catenary");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Hanging Cable".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .init_resource::<PhysicsHud>()
        .insert_resource(PhysicsDelay::new(2.0))
        .insert_resource(
            ValidationHarness::new()
                .report_at(15.0)
                .print_every(2.0)
                .display(|m, d| {
                    let info = CableInfo::from_model(m);
                    let t5 = d.xpos[info.tip_5];
                    let t10 = d.xpos[info.tip_10];
                    let t20 = d.xpos[info.tip_20];
                    format!(
                        "tip_5=({:.3},{:.3},{:.3})  tip_10=({:.3},{:.3},{:.3})  tip_20=({:.3},{:.3},{:.3})",
                        t5[0], t5[1], t5[2],
                        t10[0], t10[1], t10[2],
                        t20[0], t20[1], t20[2],
                    )
                }),
        )
        .add_systems(Startup, setup)
        .add_systems(Update, (tick_physics_delay, step_physics_realtime.run_if(physics_delay_elapsed)).chain())
        .add_systems(
            PostUpdate,
            (
                sync_geom_transforms,
                validation_system,
                hang_diagnostics,
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
    // Run forward kinematics so geoms have valid initial positions
    data.forward(&model).expect("initial forward");

    println!(
        "  Model: {} bodies, {} joints, {} geoms\n",
        model.nbody, model.njnt, model.ngeom
    );

    // Use MJCF rgba colors directly — no material overrides needed
    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[],
    );

    spawn_example_camera(
        &mut commands,
        Vec3::new(0.0, 0.7, 0.0), // center between bar (y=1.2) and tips (y≈0.2)
        2.5,                      // distance
        std::f32::consts::FRAC_PI_2, // azimuth: 90° — face the X-Z plane
        0.15,                     // slight elevation
    );

    spawn_physics_hud(&mut commands);

    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── HUD ─────────────────────────────────────────────────────────────────────

fn update_hud(model: Res<PhysicsModel>, data: Res<PhysicsData>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();
    hud.section("Hanging Cable — Resolution Comparison");

    let info = CableInfo::from_model(&model);
    let t5 = data.xpos[info.tip_5];
    let t10 = data.xpos[info.tip_10];
    let t20 = data.xpos[info.tip_20];

    hud.scalar("t", data.time, 1);
    hud.vec3("5-seg  tip", t5.as_slice(), 3);
    hud.vec3("10-seg tip", t10.as_slice(), 3);
    hud.vec3("20-seg tip", t20.as_slice(), 3);
}

// ── Validation ──────────────────────────────────────────────────────────────

const ANCHOR_Z: f64 = 1.2;

fn hang_diagnostics(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    mut harness: ResMut<ValidationHarness>,
) {
    if !harness.take_reported() {
        return;
    }

    let info = CableInfo::from_model(&model);

    // Check 1-3: each cable tip below anchor
    let tip_5_z = data.xpos[info.tip_5][2];
    let tip_10_z = data.xpos[info.tip_10][2];
    let tip_20_z = data.xpos[info.tip_20][2];

    // Check 4: convergence — 10 closer to 20 than 5 is
    let tip_5 = data.xpos[info.tip_5];
    let tip_10 = data.xpos[info.tip_10];
    let tip_20 = data.xpos[info.tip_20];
    let err_5v20 = (tip_5 - tip_20).norm();
    let err_10v20 = (tip_10 - tip_20).norm();

    // Check 5: all cables settled (max angular velocity < threshold)
    let max_qvel = data.qvel.iter().fold(0.0_f64, |m, &v| m.max(v.abs()));

    let checks = vec![
        Check {
            name: "5-seg tip below anchor",
            pass: tip_5_z < ANCHOR_Z,
            detail: format!("tip_z={tip_5_z:.3}, anchor_z={ANCHOR_Z}"),
        },
        Check {
            name: "10-seg tip below anchor",
            pass: tip_10_z < ANCHOR_Z,
            detail: format!("tip_z={tip_10_z:.3}, anchor_z={ANCHOR_Z}"),
        },
        Check {
            name: "20-seg tip below anchor",
            pass: tip_20_z < ANCHOR_Z,
            detail: format!("tip_z={tip_20_z:.3}, anchor_z={ANCHOR_Z}"),
        },
        Check {
            name: "Convergence: 10 vs 20",
            pass: err_10v20 < err_5v20,
            detail: format!("err_5v20={err_5v20:.4}, err_10v20={err_10v20:.4}"),
        },
        Check {
            name: "All cables settled",
            pass: max_qvel < 0.1,
            detail: format!("max_qvel={max_qvel:.4}"),
        },
    ];
    let _ = print_report("Hanging Cable (t=15s)", &checks);
}
