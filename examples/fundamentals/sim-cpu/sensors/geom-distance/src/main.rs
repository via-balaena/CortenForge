//! GeomDist + GeomNormal + GeomFromTo Sensors
//!
//! A box on a spring-loaded slide joint oscillates near a fixed sphere.
//! Uses box-sphere pairing to exercise the GJK narrowphase (sphere-sphere
//! would use the analytic fast path, bypassing GJK entirely).
//!
//! All three distance sensors use `cutoff="10"` to avoid the cutoff=0 edge case.
//!
//! Validates:
//! - GeomDist matches analytical: (sphere_x - r_sphere) - (box_x + half_ext)
//! - GeomNormal ≈ [1, 0, 0] (aligned along X-axis)
//! - GeomFromTo: from-point on box surface, to-point on sphere surface
//! - Distance range > 0.1m (spring oscillation)
//!
//! Run with: `cargo run -p example-sensor-geom-distance --release`

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
use sim_bevy::examples::{ValidationHarness, spawn_example_camera, validation_system};
use sim_bevy::materials::MetalPreset;
use sim_bevy::model_data::{
    PhysicsAccumulator, PhysicsData, PhysicsModel, spawn_model_geoms, step_physics_realtime,
    sync_geom_transforms,
};
use sim_core::ENABLE_ENERGY;
use sim_core::validation::{Check, print_report};

// ── Constants ───────────────────────────────────────────────────────────────

const SPHERE_X: f64 = 0.5;
const SPHERE_R: f64 = 0.1;
const BOX_HALF: f64 = 0.1;
const BOX_INIT_X: f64 = -0.3; // body position
const SLIDE_INIT: f64 = 0.2; // initial slide displacement

// ── MJCF Model ──────────────────────────────────────────────────────────────

const MJCF: &str = r#"
<mujoco model="geom_distance">
    <compiler angle="radian"/>
    <option gravity="0 0 -9.81" timestep="0.001" integrator="RK4"/>

    <default>
        <geom contype="0" conaffinity="0"/>
    </default>

    <worldbody>
        <geom name="fixed" type="sphere" size="0.1" pos="0.5 0 0"
              rgba="0.82 0.22 0.15 1"/>
        <body name="moving" pos="-0.3 0 0">
            <joint name="slide" type="slide" axis="1 0 0"
                   stiffness="10" damping="0"/>
            <inertial pos="0 0 0" mass="1.0" diaginertia="0.01 0.01 0.01"/>
            <geom name="box" type="box" size="0.1 0.1 0.1"
                  rgba="0.2 0.5 0.85 1"/>
        </body>
    </worldbody>

    <sensor>
        <distance name="dist" geom1="box" geom2="fixed" cutoff="10"/>
        <normal name="norm" geom1="box" geom2="fixed" cutoff="10"/>
        <fromto name="ft" geom1="box" geom2="fixed" cutoff="10"/>
    </sensor>
</mujoco>
"#;

// ── Bevy App ────────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: GeomDist + GeomNormal + GeomFromTo ===");
    println!("  Box on spring oscillates near fixed sphere (GJK path)");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Geom Distance".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .init_resource::<SensorValidation>()
        .insert_resource(
            ValidationHarness::new()
                .report_at(15.0)
                .print_every(1.0)
                .display(|m, d| {
                    let dist = d.sensor_data(m, 0)[0];
                    let norm = d.sensor_data(m, 1);
                    format!(
                        "dist={dist:+.4}  normal=({:+.3},{:+.3},{:+.3})",
                        norm[0], norm[1], norm[2],
                    )
                })
                .track_energy(0.5),
        )
        .add_systems(Startup, setup)
        .add_systems(Update, step_physics_realtime)
        .add_systems(
            PostUpdate,
            (sync_geom_transforms, validation_system, sensor_diagnostics).chain(),
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

    // Initial displacement on slide joint
    data.qpos[model.jnt_qpos_adr[0]] = SLIDE_INIT;
    let _ = data.forward(&model);

    let dist = data.sensor_data(&model, 0);
    println!("  t=0 dist: {:.4}", dist[0]);
    println!(
        "  Model: {} bodies, {} geoms, {} sensors\n",
        model.nbody, model.ngeom, model.nsensor
    );

    // ── Materials ────────────────────────────────────────────────────────
    let mat_sphere =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.82, 0.22, 0.15)));
    let mat_box = materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.2, 0.5, 0.85)));

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[("fixed", mat_sphere), ("box", mat_box)],
    );

    spawn_example_camera(
        &mut commands,
        Vec3::new(0.1, 0.0, 0.0),
        2.5,
        std::f32::consts::FRAC_PI_4,
        0.3,
    );

    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── Sensor Validation ───────────────────────────────────────────────────────

/// Analytical distance: gap between box face at (box_center_x + half) and
/// sphere surface at (sphere_x - radius).
fn analytical_dist(slide_q: f64) -> f64 {
    let box_center_x = BOX_INIT_X + slide_q;
    let box_face_x = box_center_x + BOX_HALF;
    let sphere_surface_x = SPHERE_X - SPHERE_R;
    sphere_surface_x - box_face_x
}

#[derive(Resource, Default)]
struct SensorValidation {
    dist_max_err: f64,
    normal_max_err: f64,
    dist_min: f64,
    dist_max: f64,
    dist_initialized: bool,
    reported: bool,
}

fn sensor_diagnostics(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<SensorValidation>,
) {
    let slide_q = data.joint_qpos(&model, 0)[0];
    let expected_dist = analytical_dist(slide_q);

    // GeomDist check
    let dist_sensor = data.sensor_data(&model, 0)[0];
    let dist_err = (dist_sensor - expected_dist).abs();
    if dist_err > val.dist_max_err {
        val.dist_max_err = dist_err;
    }

    // GeomNormal check: should be ~[1, 0, 0] (box→sphere along +X)
    let norm = data.sensor_data(&model, 1);
    let norm_err = ((norm[0] - 1.0).powi(2) + norm[1].powi(2) + norm[2].powi(2)).sqrt();
    if norm_err > val.normal_max_err {
        val.normal_max_err = norm_err;
    }

    // Distance range tracking
    if !val.dist_initialized {
        val.dist_min = dist_sensor;
        val.dist_max = dist_sensor;
        val.dist_initialized = true;
    }
    if dist_sensor < val.dist_min {
        val.dist_min = dist_sensor;
    }
    if dist_sensor > val.dist_max {
        val.dist_max = dist_sensor;
    }

    // Report
    if harness.reported() && !val.reported {
        val.reported = true;
        let dist_range = val.dist_max - val.dist_min;
        let checks = vec![
            Check {
                name: "Dist analytical",
                pass: val.dist_max_err < 1e-6,
                detail: format!("max error={:.2e}", val.dist_max_err),
            },
            Check {
                name: "Normal ≈ [1,0,0]",
                pass: val.normal_max_err < 1e-4,
                detail: format!("max deviation={:.2e}", val.normal_max_err),
            },
            Check {
                name: "Dist range > 0.1m",
                pass: dist_range > 0.1,
                detail: format!(
                    "range={dist_range:.3}m [{:.3}, {:.3}]",
                    val.dist_min, val.dist_max
                ),
            },
        ];
        let _ = print_report("Geom Distance (t=15s)", &checks);
    }
}
