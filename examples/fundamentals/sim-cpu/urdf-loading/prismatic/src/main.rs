//! Prismatic Joint Slider
//!
//! A box sliding back and forth on a horizontal rail, driven by a spring force
//! applied via `qfrc_applied`. The URDF `prismatic` joint converts to an MJCF
//! `slide` joint. Verifies oscillation period matches T = 2*pi*sqrt(m/k).
//!
//! Run: `cargo run -p example-urdf-prismatic --release`

#![allow(
    clippy::doc_markdown,
    clippy::needless_pass_by_value,
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::cast_lossless,
    clippy::let_underscore_must_use,
    clippy::suboptimal_flops,
    clippy::too_many_lines
)]

use bevy::prelude::*;
use sim_bevy::camera::OrbitCameraPlugin;
use sim_bevy::convert::physics_pos;
use sim_bevy::examples::{
    PhysicsHud, ValidationHarness, render_physics_hud, spawn_example_camera, spawn_physics_hud,
    validation_system,
};
use sim_bevy::model_data::{
    PhysicsAccumulator, PhysicsData, PhysicsModel, spawn_model_geoms, step_physics_realtime,
    sync_geom_transforms,
};
use sim_core::validation::{Check, print_report};

// ── URDF Model ───────────────────────────────────────────────────────────

/// Horizontal slider: base (fused into world) + visible box on a prismatic
/// joint along X. A rail (cylinder) on the base gives spatial reference.
const SLIDER_URDF: &str = r#"<?xml version="1.0"?>
<robot name="slider">
    <link name="base">
        <inertial>
            <mass value="100.0"/>
            <inertia ixx="10" ixy="0" ixz="0" iyy="10" iyz="0" izz="10"/>
        </inertial>
    </link>
    <link name="mass">
        <inertial>
            <mass value="2.0"/>
            <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
        </inertial>
        <collision>
            <geometry>
                <box size="0.2 0.2 0.2"/>
            </geometry>
        </collision>
    </link>
    <joint name="slide" type="prismatic">
        <parent link="base"/>
        <child link="mass"/>
        <origin xyz="0 0 1.0"/>
        <axis xyz="1 0 0"/>
        <limit lower="-1.0" upper="1.0"/>
    </joint>
</robot>
"#;

// ── Physics constants ────────────────────────────────────────────────────

const SPRING_K: f64 = 50.0;
const SLIDER_MASS: f64 = 2.0;
const INITIAL_DISP: f64 = 0.3; // 30cm displacement for visible motion

fn analytical_period() -> f64 {
    2.0 * std::f64::consts::PI * (SLIDER_MASS / SPRING_K).sqrt()
}

// ── Bevy App ─────────────────────────────────────────────────────────────

fn main() {
    let t = analytical_period();
    println!("=== CortenForge: URDF Prismatic Joint Slider ===");
    println!("  URDF prismatic → MJCF slide, spring via qfrc_applied");
    println!("  m={SLIDER_MASS}kg  k={SPRING_K}N/m  x0={INITIAL_DISP}m");
    println!("  Analytical period T = {t:.4}s");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — URDF Prismatic Slider".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .init_resource::<PhysicsHud>()
        .init_resource::<SliderValidation>()
        .insert_resource(
            ValidationHarness::new()
                .report_at(10.0)
                .print_every(0.5)
                .display(|m, d| {
                    let pos = d.joint_qpos(m, 0)[0];
                    let vel = d.joint_qvel(m, 0)[0];
                    format!("x={pos:+.3}m  v={vel:+.3}m/s")
                }),
        )
        .add_systems(Startup, setup)
        .add_systems(Update, step_physics_realtime)
        .add_systems(
            PostUpdate,
            (
                sync_geom_transforms,
                validation_system,
                slider_diagnostics,
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
    let mjcf = sim_urdf::urdf_to_mjcf(SLIDER_URDF).expect("URDF→MJCF");
    let mjcf = mjcf.replace(
        r#"timestep="0.002"/>"#,
        r#"timestep="0.002"><flag energy="enable"/></option>"#,
    );
    // Inject spring stiffness on the joint (URDF has no stiffness attribute)
    let mjcf = mjcf.replace(
        r#"type="slide""#,
        &format!(r#"type="slide" stiffness="{SPRING_K}""#),
    );
    let model = sim_mjcf::load_model(&mjcf).expect("MJCF should parse");
    let mut data = model.make_data();

    data.qpos[0] = INITIAL_DISP;
    let _ = data.forward(&model);

    println!(
        "  Model: {} bodies, {} joints, {} geoms, {} DOFs",
        model.nbody, model.njnt, model.ngeom, model.nv
    );
    println!(
        "  Joint type: {:?}, axis: ({:.1}, {:.1}, {:.1})\n",
        model.jnt_type[0], model.jnt_axis[0].x, model.jnt_axis[0].y, model.jnt_axis[0].z
    );

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
        physics_pos(0.0, 0.0, 1.0),
        3.0,
        std::f32::consts::FRAC_PI_4,
        0.4,
    );
    spawn_physics_hud(&mut commands);

    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── HUD ──────────────────────────────────────────────────────────────────

fn update_hud(model: Res<PhysicsModel>, data: Res<PhysicsData>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();
    hud.section("URDF Prismatic Slider");

    let pos = data.joint_qpos(&model, 0)[0];
    let vel = data.joint_qvel(&model, 0)[0];
    let spring_f = -SPRING_K * pos;

    hud.scalar("position (m)", pos, 3);
    hud.scalar("velocity (m/s)", vel, 3);
    hud.scalar("spring force (N)", spring_f, 1);
    hud.scalar("time", data.time, 1);
}

// ── Validation ───────────────────────────────────────────────────────────

#[derive(Resource, Default)]
struct SliderValidation {
    first_crossing: Option<f64>,
    second_crossing: Option<f64>,
    prev_q: f64,
    initialized: bool,
    reported: bool,
}

fn slider_diagnostics(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<SliderValidation>,
) {
    // Track zero crossings for period measurement
    if !val.initialized {
        val.prev_q = data.qpos[0];
        val.initialized = true;
        return;
    }

    let q = data.qpos[0];
    if val.prev_q > 0.0 && q <= 0.0 {
        if val.first_crossing.is_none() {
            val.first_crossing = Some(data.time);
        } else if val.second_crossing.is_none() {
            val.second_crossing = Some(data.time);
        }
    }
    val.prev_q = q;

    if !harness.reported() || val.reported {
        return;
    }
    val.reported = true;

    let is_slide = model.jnt_type[0] == sim_core::MjJointType::Slide;
    let axis = model.jnt_axis[0];
    let axis_ok = (axis.x - 1.0).abs() < 0.01 && axis.y.abs() < 0.01 && axis.z.abs() < 0.01;

    let (period_pass, period_detail) = match (val.first_crossing, val.second_crossing) {
        (Some(t1), Some(t2)) => {
            let measured = t2 - t1;
            let expected = analytical_period();
            let err = ((measured - expected) / expected).abs();
            (
                err < 0.02,
                format!(
                    "T={measured:.4}s, expected={expected:.4}s, err={:.2}%",
                    err * 100.0
                ),
            )
        }
        _ => (false, "insufficient zero crossings".into()),
    };

    let checks = vec![
        Check {
            name: "Prismatic → slide",
            pass: is_slide,
            detail: format!("type={:?}", model.jnt_type[0]),
        },
        Check {
            name: "Axis maps to X",
            pass: axis_ok,
            detail: format!("axis=({:.1}, {:.1}, {:.1})", axis.x, axis.y, axis.z),
        },
        Check {
            name: "Period matches T=2pi*sqrt(m/k)",
            pass: period_pass,
            detail: period_detail,
        },
    ];
    let _ = print_report("URDF Prismatic", &checks);
}
