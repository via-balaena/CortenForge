//! Subtree Velocity Sensor
//!
//! A 3-link chain (free joint root + 2 hinges) free-falls under gravity.
//! SubtreeLinVel on the root body measures the velocity of the composite
//! center of mass. SubtreeCom tracks the COM position for context.
//!
//! In free fall the COM accelerates at exactly g, so v_z = −g×t regardless
//! of how the internal joints move. The links flex and tumble but the COM
//! velocity is a clean linear ramp.
//!
//! Validates:
//! - v_z = −g×t at t=2s (within 0.5%)
//! - v_xy ≈ 0 (no horizontal forces)
//! - COM descends (com_z drops over time)
//!
//! Run with: `cargo run -p example-sensor-adv-subtree-velocity --release`

#![allow(
    clippy::doc_markdown,
    clippy::needless_pass_by_value,
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::let_underscore_must_use,
    clippy::suboptimal_flops,
    clippy::cast_precision_loss
)]

use bevy::prelude::*;
use sim_bevy::camera::OrbitCameraPlugin;
use sim_bevy::examples::{
    PhysicsHud, ValidationHarness, render_physics_hud, spawn_example_camera, spawn_physics_hud,
    validation_system,
};
use sim_bevy::materials::MetalPreset;
use sim_bevy::model_data::{PhysicsData, PhysicsModel, spawn_model_geoms, sync_geom_transforms};
use sim_core::validation::{Check, print_report};

// ── MJCF Model ──────────────────────────────────────────────────────────────
//
// 3-link chain: free-joint root (link1) + two hinges (link2, link3).
// Different masses so the COM isn't trivially at the geometric center.
// No contacts — falls forever. Initial hinge offsets give the chain a
// bent shape so the links visibly flex during the fall.
//
const MJCF: &str = r#"
<mujoco model="subtree-velocity">
    <compiler angle="radian"/>
    <option gravity="0 0 -9.81" timestep="0.001">
        <flag energy="enable"/>
    </option>

    <default>
        <geom contype="0" conaffinity="0"/>
    </default>

    <worldbody>
        <body name="link1" pos="0 0 3">
            <freejoint name="root"/>
            <geom name="g1" type="capsule" size="0.04"
                  fromto="0 0 0 0 0 -0.4" mass="2.0" rgba="0.52 0.53 0.56 1"/>
            <body name="link2" pos="0 0 -0.4">
                <joint name="j2" type="hinge" axis="0 1 0" damping="0.02"/>
                <geom name="g2" type="capsule" size="0.035"
                      fromto="0 0 0 0 0 -0.35" mass="1.0" rgba="0.2 0.5 0.85 1"/>
                <body name="link3" pos="0 0 -0.35">
                    <joint name="j3" type="hinge" axis="0 1 0" damping="0.02"/>
                    <geom name="g3" type="capsule" size="0.03"
                          fromto="0 0 0 0 0 -0.3" mass="0.5" rgba="0.85 0.3 0.2 1"/>
                </body>
            </body>
        </body>
    </worldbody>

    <sensor>
        <subtreelinvel name="sub_vel" body="link1"/>
        <subtreecom name="sub_com" body="link1"/>
    </sensor>
</mujoco>
"#;

const G: f64 = 9.81;
const DROP_DELAY: f64 = 2.0; // seconds of hover before gravity kicks in

// ── Bevy App ────────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Subtree Velocity Sensor ===");
    println!("  3-link chain in free fall (masses 2.0, 1.0, 0.5 kg)");
    println!("  SubtreeLinVel tracks composite COM velocity");
    println!("  Expected: v_z = -g*t (linear ramp)");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Subtree Velocity".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsHud>()
        .init_resource::<SensorValidation>()
        .insert_resource(
            ValidationHarness::new()
                .report_at(5.0)
                .print_every(0.5)
                .display(|m, d| {
                    let v = sensor_vec3_or_zero(d, m, "sub_vel");
                    // Sim time = 0 during hold (physics not stepping), then ramps
                    let expected_vz = -G * d.time;
                    let phase = if d.time < 0.01 { "HOLD" } else { "FALL" };
                    format!(
                        "{phase}  v_z={:.4}  expected={expected_vz:.4}  err={:.3}%",
                        v[2],
                        if expected_vz.abs() > 0.1 {
                            (v[2] - expected_vz).abs() / expected_vz.abs() * 100.0
                        } else {
                            0.0
                        }
                    )
                }),
        )
        .add_systems(Startup, setup)
        .add_systems(Update, delayed_step)
        .add_systems(
            PostUpdate,
            (
                sync_geom_transforms,
                validation_system,
                sensor_diagnostics,
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

    // Bend the chain slightly so the links visibly flex during fall
    let j2 = model.joint_id("j2").expect("j2");
    let j3 = model.joint_id("j3").expect("j3");
    data.qpos[model.jnt_qpos_adr[j2]] = 0.3;
    data.qpos[model.jnt_qpos_adr[j3]] = -0.2;
    let _ = data.forward(&model);

    println!(
        "  Model: {} bodies, {} joints, {} sensors\n",
        model.nbody, model.njnt, model.nsensor
    );

    let mat_g1 = materials.add(MetalPreset::PolishedSteel.material());
    let mat_g2 = materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.2, 0.5, 0.85)));
    let mat_g3 = materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.85, 0.3, 0.2)));

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[("g1", mat_g1), ("g2", mat_g2), ("g3", mat_g3)],
    );

    // Camera far out and steeply tilted down to watch the full drop
    spawn_example_camera(
        &mut commands,
        Vec3::new(0.0, -2.0, 0.0), // Bevy Y-up: target below ground level
        14.0,
        std::f32::consts::FRAC_PI_4,
        1.0, // steep downward tilt
    );

    spawn_physics_hud(&mut commands);

    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── Delayed Step ────────────────────────────────────────────────────────────

/// Don't step physics for the first DROP_DELAY wall-clock seconds so the
/// viewer can see the chain hovering before the drop.
fn delayed_step(
    model: Res<PhysicsModel>,
    mut data: ResMut<PhysicsData>,
    time: Res<Time>,
    mut state: Local<(f64, f64)>, // (wall_time, physics_accumulator)
) {
    state.0 += time.delta_secs_f64();
    if state.0 < DROP_DELAY {
        return; // hold: don't step physics
    }
    let dt = model.0.timestep;
    state.1 += time.delta_secs_f64();
    let mut steps = 0;
    while state.1 >= dt && steps < 200 {
        let _ = data.0.step(&model.0);
        state.1 -= dt;
        steps += 1;
    }
}

// ── HUD ─────────────────────────────────────────────────────────────────────

fn update_hud(model: Res<PhysicsModel>, data: Res<PhysicsData>, mut hud: ResMut<PhysicsHud>) {
    let v = sensor_vec3_or_zero(&data, &model, "sub_vel");
    let com = sensor_vec3_or_zero(&data, &model, "sub_com");
    let expected_vz = -G * data.time;

    hud.clear();
    hud.section("Subtree Velocity (free fall)");
    hud.vec3("v_com", &v, 3);
    hud.scalar("v_z", v[2], 3);
    hud.scalar("expected v_z", expected_vz, 3);
    let err_pct = if expected_vz.abs() > 0.1 {
        (v[2] - expected_vz).abs() / expected_vz.abs() * 100.0
    } else {
        0.0
    };
    hud.raw(format!("  err          {err_pct:.3}%"));
    hud.raw(String::new());
    hud.vec3("COM", &com, 3);
    hud.scalar("t", data.time, 2);
}

// ── Sensor Validation ───────────────────────────────────────────────────────

#[derive(Resource, Default)]
struct SensorValidation {
    vz_at_2s: Option<f64>,
    max_vxy: f64,
    com_z_initial: Option<f64>,
    com_z_at_3s: Option<f64>,
    reported: bool,
}

fn sensor_diagnostics(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<SensorValidation>,
) {
    let v = sensor_vec3_or_zero(&data, &model, "sub_vel");
    let com = sensor_vec3_or_zero(&data, &model, "sub_com");

    // Sim time starts at 0 and only advances once physics steps

    // Record initial COM
    if val.com_z_initial.is_none() && data.time > 0.01 {
        val.com_z_initial = Some(com[2]);
    }

    // Track max horizontal velocity
    if data.time > 0.0 {
        val.max_vxy = val.max_vxy.max(v[0].abs().max(v[1].abs()));
    }

    // Sample at t=2s sim time
    if val.vz_at_2s.is_none() && data.time >= 2.0 {
        val.vz_at_2s = Some(v[2]);
    }

    // Sample COM at t=3s sim time
    if val.com_z_at_3s.is_none() && data.time >= 3.0 {
        val.com_z_at_3s = Some(com[2]);
    }

    if harness.reported() && !val.reported {
        val.reported = true;

        let vz_2 = val.vz_at_2s.unwrap_or(0.0);
        let expected_vz_2 = -G * 2.0; // -19.62
        let err_pct = (vz_2 - expected_vz_2).abs() / expected_vz_2.abs() * 100.0;

        let com_z0 = val.com_z_initial.unwrap_or(0.0);
        let com_z3 = val.com_z_at_3s.unwrap_or(0.0);

        let checks = vec![
            Check {
                name: "v_z = -g*t at t=2s",
                pass: err_pct < 0.5,
                detail: format!("v_z={vz_2:.4}, expected={expected_vz_2:.4}, err={err_pct:.3}%"),
            },
            Check {
                name: "v_xy ~ 0",
                pass: val.max_vxy < 0.01,
                detail: format!("max |v_xy|={:.2e}", val.max_vxy),
            },
            Check {
                name: "COM descends",
                pass: com_z3 < com_z0,
                detail: format!("com_z(0)={com_z0:.3}, com_z(3)={com_z3:.3}"),
            },
        ];
        let _ = print_report("Subtree Velocity (t=5s)", &checks);
    }
}

// ── Helpers ─────────────────────────────────────────────────────────────────

fn sensor_vec3_or_zero(
    data: &sim_core::types::Data,
    model: &sim_core::types::Model,
    name: &str,
) -> [f64; 3] {
    if let Some(id) = model.sensor_id(name) {
        let s = data.sensor_data(model, id);
        if s.len() >= 3 {
            return [s[0], s[1], s[2]];
        }
    }
    [0.0, 0.0, 0.0]
}
