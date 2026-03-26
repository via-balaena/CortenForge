//! JointPos + JointVel Sensors
//!
//! A hinge pendulum displaced 45° and released. JointPos reads `qpos` directly,
//! JointVel reads `qvel` directly — both are pass-throughs verified against the
//! backing state.
//!
//! Validates:
//! - JointPos sensor == data.joint_qpos (exact pass-through)
//! - JointVel sensor == data.joint_qvel (exact pass-through)
//! - JointPos range > 1.0 rad (pendulum actually swings)
//! - JointVel changes sign >= 10 times in 15s (oscillation)
//! - Energy conservation (undamped system)
//!
//! Run with: `cargo run -p example-sensor-joint-pos-vel --release`

#![allow(
    clippy::doc_markdown,
    clippy::needless_pass_by_value,
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::let_underscore_must_use,
    clippy::suboptimal_flops
)]

use bevy::prelude::*;
use sim_bevy::camera::OrbitCameraPlugin;
use sim_bevy::examples::{
    PhysicsHud, ValidationHarness, render_physics_hud, spawn_example_camera, spawn_physics_hud,
    validation_system,
};
use sim_bevy::materials::MetalPreset;
use sim_bevy::model_data::{
    PhysicsAccumulator, PhysicsData, PhysicsModel, spawn_model_geoms, step_physics_realtime,
    sync_geom_transforms,
};
use sim_core::ENABLE_ENERGY;
use sim_core::validation::{Check, print_report};

// ── MJCF Model ──────────────────────────────────────────────────────────────

const MJCF: &str = r#"
<mujoco model="joint_pos_vel">
    <compiler angle="radian"/>
    <option gravity="0 0 -9.81" timestep="0.001" integrator="RK4"/>

    <default>
        <geom contype="0" conaffinity="0"/>
    </default>

    <worldbody>
        <!-- Support frame -->
        <geom name="beam" type="capsule" size="0.022"
              fromto="-0.25 0 0  0.25 0 0" rgba="0.40 0.40 0.43 1"/>
        <geom name="post_l" type="capsule" size="0.022"
              fromto="-0.25 0 0  -0.25 0 0.30" rgba="0.40 0.40 0.43 1"/>
        <geom name="post_r" type="capsule" size="0.022"
              fromto="0.25 0 0  0.25 0 0.30" rgba="0.40 0.40 0.43 1"/>

        <body name="arm" pos="0 0 0">
            <joint name="hinge" type="hinge" axis="0 1 0" damping="0"/>
            <inertial pos="0 0 -0.25" mass="1.0" diaginertia="0.01 0.01 0.01"/>
            <geom name="rod" type="capsule" size="0.02"
                  fromto="0 0 0  0 0 -0.5" rgba="0.48 0.48 0.50 1"/>
            <geom name="tip" type="sphere" size="0.06"
                  pos="0 0 -0.5" rgba="0.2 0.5 0.85 1"/>
        </body>
    </worldbody>

    <sensor>
        <jointpos name="pos" joint="hinge"/>
        <jointvel name="vel" joint="hinge"/>
    </sensor>
</mujoco>
"#;

// ── Bevy App ────────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: JointPos + JointVel Sensors ===");
    println!("  Hinge pendulum, zero damping, RK4 integrator");
    println!("  Initial tilt: 45°");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — JointPos + JointVel".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .init_resource::<PhysicsHud>()
        .init_resource::<SensorValidation>()
        .insert_resource(
            ValidationHarness::new()
                .report_at(15.0)
                .print_every(1.0)
                .display(|m, d| {
                    let pos = d.sensor_data(m, 0)[0];
                    let vel = d.sensor_data(m, 1)[0];
                    let energy = d.energy_kinetic + d.energy_potential;
                    format!("pos={pos:+6.3} rad  vel={vel:+6.3} rad/s  E={energy:.4}J")
                })
                .track_energy(0.5),
        )
        .add_systems(Startup, setup)
        .add_systems(Update, step_physics_realtime)
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
    let mut model = sim_mjcf::load_model(MJCF).expect("MJCF should parse");
    model.enableflags |= ENABLE_ENERGY;
    let mut data = model.make_data();

    // Initial displacement: 45°
    let qpos_adr = model.jnt_qpos_adr[0];
    data.qpos[qpos_adr] = std::f64::consts::FRAC_PI_4;
    let _ = data.forward(&model);

    println!(
        "  Model: {} bodies, {} joints, {} sensors\n",
        model.nbody, model.njnt, model.nsensor
    );

    // ── Materials ────────────────────────────────────────────────────────
    let mat_frame = materials.add(MetalPreset::BrushedMetal.material());
    let mat_rod = materials.add(MetalPreset::BrushedMetal.material());
    let mat_tip = materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.2, 0.5, 0.85)));

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[
            ("beam", mat_frame.clone()),
            ("post_l", mat_frame.clone()),
            ("post_r", mat_frame),
            ("rod", mat_rod),
            ("tip", mat_tip),
        ],
    );

    spawn_example_camera(
        &mut commands,
        Vec3::new(0.0, -0.2, 0.0),
        1.8,
        std::f32::consts::FRAC_PI_4,
        0.35,
    );

    spawn_physics_hud(&mut commands);

    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── HUD ────────────────────────────────────────────────────────────────────

fn update_hud(model: Res<PhysicsModel>, data: Res<PhysicsData>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();
    hud.section("JointPos + JointVel");
    let pos = data.sensor_data(&model, 0)[0];
    let vel = data.sensor_data(&model, 1)[0];
    let energy = data.energy_kinetic + data.energy_potential;
    hud.scalar("pos", pos, 3);
    hud.scalar("vel", vel, 3);
    hud.scalar("E", energy, 4);
}

// ── Sensor Validation ───────────────────────────────────────────────────────

#[derive(Resource, Default)]
struct SensorValidation {
    pos_max_err: f64,
    vel_max_err: f64,
    pos_min: f64,
    pos_max: f64,
    vel_sign_changes: u32,
    last_vel_sign: i8,
    reported: bool,
}

fn sensor_diagnostics(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<SensorValidation>,
) {
    let jid = model.sensor_objid[0]; // joint index for the pos sensor

    // JointPos pipeline check
    let pos_sensor = data.sensor_data(&model, 0)[0];
    let pos_state = data.joint_qpos(&model, jid)[0];
    let pos_err = (pos_sensor - pos_state).abs();
    if pos_err > val.pos_max_err {
        val.pos_max_err = pos_err;
    }

    // JointVel pipeline check
    let vel_sensor = data.sensor_data(&model, 1)[0];
    let vel_state = data.joint_qvel(&model, jid)[0];
    let vel_err = (vel_sensor - vel_state).abs();
    if vel_err > val.vel_max_err {
        val.vel_max_err = vel_err;
    }

    // Range tracking
    if pos_sensor < val.pos_min {
        val.pos_min = pos_sensor;
    }
    if pos_sensor > val.pos_max {
        val.pos_max = pos_sensor;
    }

    // Velocity sign changes
    let sign: i8 = if vel_sensor > 0.0 {
        1
    } else if vel_sensor < 0.0 {
        -1
    } else {
        0
    };
    if sign != 0 && val.last_vel_sign != 0 && sign != val.last_vel_sign {
        val.vel_sign_changes += 1;
    }
    if sign != 0 {
        val.last_vel_sign = sign;
    }

    // Report
    if harness.reported() && !val.reported {
        val.reported = true;
        let pos_range = val.pos_max - val.pos_min;
        let checks = vec![
            Check {
                name: "JointPos == qpos",
                pass: val.pos_max_err < 1e-14,
                detail: format!("max error={:.2e}", val.pos_max_err),
            },
            Check {
                name: "JointVel == qvel",
                pass: val.vel_max_err < 1e-14,
                detail: format!("max error={:.2e}", val.vel_max_err),
            },
            Check {
                name: "Pos range > 1 rad",
                pass: pos_range > 1.0,
                detail: format!(
                    "range={pos_range:.3} rad [{:.3}, {:.3}]",
                    val.pos_min, val.pos_max
                ),
            },
            Check {
                name: "Vel sign changes",
                pass: val.vel_sign_changes >= 10,
                detail: format!("{} changes (need >= 10)", val.vel_sign_changes),
            },
        ];
        let _ = print_report("JointPos + JointVel (t=15s)", &checks);
    }
}
