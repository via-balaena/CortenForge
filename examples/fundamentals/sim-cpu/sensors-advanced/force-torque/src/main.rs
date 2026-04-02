//! Force and Torque Sensors
//!
//! A cantilevered beam held horizontal by a strong motor. Force and Torque
//! sensors at the base site measure the 3D reaction force and moment.
//!
//! For a static horizontal beam (m=1 kg, L=0.8 m, COM at L/2):
//! - Force = mg = 9.81 N (upward, supporting weight)
//! - Torque = mg × L/2 = 3.924 N·m (resisting gravity)
//!
//! Both sensors output in the **site frame**. The base site is aligned with
//! the body frame, which at θ=0 (horizontal) matches the world frame.
//!
//! Validates:
//! - |Force| ≈ mg (reaction supports weight)
//! - |Torque| ≈ mgL/2 (reaction resists gravity moment)
//! - Force XY ≈ 0 (no lateral forces)
//! - Torque XZ ≈ 0 (moment about hinge axis only)
//!
//! Run with: `cargo run -p example-sensor-adv-force-torque --release`

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
use sim_bevy::model_data::{
    PhysicsAccumulator, PhysicsData, PhysicsModel, spawn_model_geoms, step_physics_realtime,
    sync_geom_transforms,
};
use sim_core::validation::{Check, print_report};

// ── MJCF Model ──────────────────────────────────────────────────────────────
//
// Horizontal beam (capsule, L=0.8m, m=1kg) on a Y-axis hinge. Strong PD
// motor holds it horizontal. Base site at the hinge for Force/Torque sensors.
// Heavy joint damping + high kp for overdamped settling.
//
const MJCF: &str = r#"
<mujoco model="force-torque">
    <compiler angle="radian"/>
    <option gravity="0 0 -9.81" timestep="0.001">
        <flag energy="enable"/>
    </option>

    <default>
        <geom contype="0" conaffinity="0"/>
    </default>

    <worldbody>
        <body name="beam" pos="0 0 1.2">
            <joint name="hinge" type="hinge" axis="0 1 0" damping="50"/>
            <geom name="shaft" type="capsule" size="0.025"
                  fromto="0 0 0 0.8 0 0" mass="1.0" rgba="0.48 0.48 0.50 1"/>
            <geom name="tip_ball" type="sphere" size="0.04"
                  pos="0.8 0 0" rgba="0.2 0.7 0.3 1" mass="0.001"/>
            <site name="base" pos="0 0 0"/>
        </body>
    </worldbody>

    <actuator>
        <position name="motor" joint="hinge" kp="5000"/>
    </actuator>

    <sensor>
        <force name="base_force" site="base"/>
        <torque name="base_torque" site="base"/>
    </sensor>
</mujoco>
"#;

const MASS: f64 = 1.0;
const G: f64 = 9.81;
const HALF_L: f64 = 0.4; // COM at L/2 = 0.4m from hinge
const EXPECTED_FORCE: f64 = MASS * G; // 9.81 N
const EXPECTED_TORQUE: f64 = MASS * G * HALF_L; // 3.924 N·m

// ── Bevy App ────────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Force & Torque Sensors ===");
    println!("  Static cantilevered beam (m=1 kg, L=0.8 m)");
    println!("  Expected: |F| = {EXPECTED_FORCE:.3} N, |tau| = {EXPECTED_TORQUE:.3} N·m");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Force & Torque".into(),
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
                    let f = sensor_vec3_or_zero(d, m, "base_force");
                    let t = sensor_vec3_or_zero(d, m, "base_torque");
                    let f_mag = vec3_norm(&f);
                    let t_mag = vec3_norm(&t);
                    format!("|F|={f_mag:.4}  |tau|={t_mag:.4}")
                }),
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
    let model = sim_mjcf::load_model(MJCF).expect("MJCF should parse");
    let data = model.make_data();

    println!(
        "  Model: {} bodies, {} joints, {} sensors\n",
        model.nbody, model.njnt, model.nsensor
    );

    let mat_shaft = materials.add(MetalPreset::CastIron.material());
    let mat_tip = materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.2, 0.7, 0.3)));

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[("shaft", mat_shaft), ("tip_ball", mat_tip)],
    );

    // Camera: slightly above and to the side
    spawn_example_camera(
        &mut commands,
        Vec3::new(0.3, 1.2, 0.0), // Bevy Y-up
        3.0,
        std::f32::consts::FRAC_PI_4,
        0.35,
    );

    spawn_physics_hud(&mut commands);

    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── HUD ─────────────────────────────────────────────────────────────────────

fn update_hud(model: Res<PhysicsModel>, data: Res<PhysicsData>, mut hud: ResMut<PhysicsHud>) {
    let f = sensor_vec3_or_zero(&data, &model, "base_force");
    let t = sensor_vec3_or_zero(&data, &model, "base_torque");
    let f_mag = vec3_norm(&f);
    let t_mag = vec3_norm(&t);

    hud.clear();
    hud.section("Force & Torque (static beam)");
    hud.vec3("Force", &f, 4);
    hud.scalar("|F|", f_mag, 4);
    hud.scalar("expected (mg)", EXPECTED_FORCE, 4);
    hud.raw(format!(
        "  err          {:.3}%",
        (f_mag - EXPECTED_FORCE).abs() / EXPECTED_FORCE * 100.0
    ));
    hud.raw(String::new());
    hud.vec3("Torque", &t, 4);
    hud.scalar("|tau|", t_mag, 4);
    hud.scalar("expected (mgL/2)", EXPECTED_TORQUE, 4);
    hud.raw(format!(
        "  err          {:.3}%",
        (t_mag - EXPECTED_TORQUE).abs() / EXPECTED_TORQUE * 100.0
    ));
}

// ── Sensor Validation ───────────────────────────────────────────────────────

#[derive(Resource, Default)]
struct SensorValidation {
    // Accumulate after 2s settling
    sum_f_mag: f64,
    sum_t_mag: f64,
    max_f_xy: f64,
    max_t_xz: f64,
    sample_count: u64,
    reported: bool,
}

fn sensor_diagnostics(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<SensorValidation>,
) {
    // Skip first 2 seconds for settling
    if data.time < 2.0 {
        return;
    }

    let f = sensor_vec3_or_zero(&data, &model, "base_force");
    let t = sensor_vec3_or_zero(&data, &model, "base_torque");
    let f_mag = vec3_norm(&f);
    let t_mag = vec3_norm(&t);

    val.sum_f_mag += f_mag;
    val.sum_t_mag += t_mag;
    val.max_f_xy = val.max_f_xy.max(f[0].abs().max(f[1].abs()));
    val.max_t_xz = val.max_t_xz.max(t[0].abs().max(t[2].abs()));
    val.sample_count += 1;

    if harness.reported() && !val.reported {
        val.reported = true;
        let n = val.sample_count as f64;
        let mean_f = val.sum_f_mag / n;
        let mean_t = val.sum_t_mag / n;

        let checks = vec![
            Check {
                name: "|Force| = mg",
                pass: ((mean_f - EXPECTED_FORCE) / EXPECTED_FORCE).abs() < 0.01,
                detail: format!(
                    "mean |F|={mean_f:.4}, expected={EXPECTED_FORCE:.4}, err={:.3}%",
                    (mean_f - EXPECTED_FORCE).abs() / EXPECTED_FORCE * 100.0
                ),
            },
            Check {
                name: "Force XY ~ 0",
                pass: val.max_f_xy < 0.05,
                detail: format!("max |F_xy|={:.4}", val.max_f_xy),
            },
            Check {
                name: "|Torque| = mgL/2",
                pass: ((mean_t - EXPECTED_TORQUE) / EXPECTED_TORQUE).abs() < 0.01,
                detail: format!(
                    "mean |tau|={mean_t:.4}, expected={EXPECTED_TORQUE:.4}, err={:.3}%",
                    (mean_t - EXPECTED_TORQUE).abs() / EXPECTED_TORQUE * 100.0
                ),
            },
            Check {
                name: "Torque XZ ~ 0",
                pass: val.max_t_xz < 0.05,
                detail: format!("max |tau_xz|={:.4}", val.max_t_xz),
            },
        ];
        let _ = print_report("Force & Torque (t=15s)", &checks);
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

fn vec3_norm(v: &[f64; 3]) -> f64 {
    (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt()
}
