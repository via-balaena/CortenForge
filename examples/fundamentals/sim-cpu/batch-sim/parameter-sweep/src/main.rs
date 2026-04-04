//! Parameter Sweep — BatchSim Construction and Parallel Stepping
//!
//! Eight pendulums side by side, each with a different effective damping
//! coefficient (0.0 to 0.7). All share one Model and step in parallel via
//! `BatchSim::step_all()`. Damping is applied per-env as a velocity-
//! proportional control torque: `ctrl[0] = -D * qvel[0]`.
//!
//! The undamped pendulum swings forever; the highest-damped one dies out
//! almost immediately.
//!
//! Run: `cargo run -p example-batch-sim-parameter-sweep --release`

#![allow(
    clippy::doc_markdown,
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::similar_names,
    clippy::needless_pass_by_value,
    clippy::needless_range_loop,
    clippy::let_underscore_must_use,
    clippy::suboptimal_flops
)]

use std::sync::Arc;

use bevy::prelude::*;
use sim_bevy::camera::OrbitCameraPlugin;
use sim_bevy::examples::{
    PhysicsHud, ValidationHarness, render_physics_hud, spawn_example_camera, spawn_physics_hud,
    validation_system,
};
use sim_bevy::materials::MetalPreset;
use sim_bevy::multi_scene::{PhysicsScenes, spawn_scene_geoms, sync_scene_geom_transforms};
use sim_core::batch::BatchSim;
use sim_core::validation::{Check, print_report};

// ── Config ───────────────────────────────────────────────────────────────

const NUM_ENVS: usize = 8;
const SPACING: f32 = 1.5;
const REPORT_TIME: f64 = 10.0;

/// Damping coefficients: D_i = i * 0.1
const DAMPING: [f64; NUM_ENVS] = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7];

// ── MJCF ─────────────────────────────────────────────────────────────────

/// Single-link pendulum with motor actuator for ctrl-based damping.
/// Energy enabled for validation. No contacts (pure pendulum).
const MJCF: &str = r#"
<mujoco model="parameter-sweep">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.002" integrator="Euler">
    <flag energy="enable" contact="disable"/>
  </option>
  <default>
    <geom contype="0" conaffinity="0"/>
  </default>
  <worldbody>
    <body name="link" pos="0 0 0">
      <joint name="hinge" type="hinge" axis="0 1 0" damping="0"/>
      <inertial pos="0 0 -0.5" mass="1.0" diaginertia="0.01 0.01 0.01"/>
      <geom name="rod" type="capsule" size="0.025"
            fromto="0 0 0  0 0 -1.0" rgba="0.50 0.50 0.53 1"/>
      <geom name="tip" type="sphere" size="0.05"
            pos="0 0 -1.0"/>
    </body>
  </worldbody>
  <actuator>
    <motor name="torque" joint="hinge" gear="1"/>
  </actuator>
</mujoco>
"#;

// ── Resources ────────────────────────────────────────────────────────────

#[derive(Resource)]
struct BatchResource {
    batch: BatchSim,
    model: Arc<sim_core::Model>,
    initial_energies: [f64; NUM_ENVS],
    accumulator: f64,
}

#[derive(Resource, Default)]
struct SweepValidation {
    reported: bool,
}

// ── Bevy App ─────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Parameter Sweep ===");
    println!("  8 pendulums, damping D = 0.0 to 0.7");
    println!("  All share one Model, step in parallel via BatchSim");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Parameter Sweep (BatchSim)".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsHud>()
        .init_resource::<SweepValidation>()
        .insert_resource(
            ValidationHarness::new()
                .report_at(REPORT_TIME)
                .print_every(2.0)
                .display(|_m, _d| String::new()),
        )
        .add_systems(Startup, setup)
        .add_systems(Update, step_batch)
        .add_systems(
            PostUpdate,
            (
                sync_batch_to_scenes,
                sync_scene_geom_transforms,
                sync_dummy_time,
                validation_system,
                track_validation,
                update_hud,
                render_physics_hud,
            )
                .chain(),
        )
        .run();
}

// ── Setup ────────────────────────────────────────────────────────────────

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let model = Arc::new(sim_mjcf::load_model(MJCF).expect("MJCF should parse"));

    // Create BatchSim and set initial conditions
    let mut batch = BatchSim::new(Arc::clone(&model), NUM_ENVS);
    for env in batch.envs_mut() {
        env.qpos[0] = std::f64::consts::FRAC_PI_2; // 90° tilt
    }

    // Run one forward pass on each env to compute initial energy
    let mut initial_energies = [0.0; NUM_ENVS];
    for (i, env) in batch.envs_mut().enumerate() {
        env.forward(&model).expect("forward");
        initial_energies[i] = env.total_energy();
    }

    // Create PhysicsScenes for visual rendering (one scene per env)
    let mut scenes = PhysicsScenes::default();
    let tip_colors = [
        Color::srgb(0.15, 0.75, 0.15), // D=0.0 green (undamped)
        Color::srgb(0.30, 0.70, 0.30),
        Color::srgb(0.50, 0.65, 0.25),
        Color::srgb(0.70, 0.60, 0.15),
        Color::srgb(0.80, 0.50, 0.15),
        Color::srgb(0.85, 0.40, 0.15),
        Color::srgb(0.85, 0.25, 0.15),
        Color::srgb(0.85, 0.15, 0.15), // D=0.7 red (highest damping)
    ];

    for i in 0..NUM_ENVS {
        // Each scene gets its own Model + Data copy for rendering
        let scene_model = (*model).clone();
        let mut scene_data = scene_model.make_data();
        scene_data.qpos[0] = std::f64::consts::FRAC_PI_2;
        scene_data.forward(&scene_model).expect("forward");

        let id = scenes.add(format!("D={:.1}", DAMPING[i]), scene_model, scene_data);

        let mat_rod = materials.add(MetalPreset::BrushedMetal.material());
        let mat_tip = materials.add(MetalPreset::PolishedSteel.with_color(tip_colors[i]));

        let x = (i as f32 - (NUM_ENVS as f32 - 1.0) / 2.0) * SPACING;
        spawn_scene_geoms(
            &mut commands,
            &mut meshes,
            &mut materials,
            &mut scenes,
            id,
            Vec3::new(x, 0.0, 0.0),
            &[("rod", mat_rod), ("tip", mat_tip)],
        );
    }

    commands.insert_resource(scenes);

    commands.insert_resource(BatchResource {
        batch,
        model: Arc::clone(&model),
        initial_energies,
        accumulator: 0.0,
    });

    spawn_example_camera(
        &mut commands,
        Vec3::new(0.0, 0.0, -0.5),
        11.0,
        std::f32::consts::FRAC_PI_2,
        0.25,
    );

    spawn_physics_hud(&mut commands);

    // Insert dummy PhysicsModel/PhysicsData for ValidationHarness display fn
    let dummy_model = (*model).clone();
    let dummy_data = dummy_model.make_data();
    commands.insert_resource(sim_bevy::model_data::PhysicsModel(dummy_model));
    commands.insert_resource(sim_bevy::model_data::PhysicsData(dummy_data));
}

// ── Stepping ─────────────────────────────────────────────────────────────

fn step_batch(mut res: ResMut<BatchResource>, time: Res<Time>) {
    let wall_dt = time.delta_secs_f64();
    res.accumulator += wall_dt;

    let dt_sim = res.batch.model().timestep;
    let mut steps = 0;

    while res.accumulator >= dt_sim && steps < 200 {
        // Apply velocity-proportional damping torque per env
        for (i, env) in res.batch.envs_mut().enumerate() {
            env.ctrl[0] = -DAMPING[i] * env.qvel[0];
        }

        let _errors = res.batch.step_all();
        res.accumulator -= dt_sim;
        steps += 1;
    }

    // Post-step forward to refresh geom poses for rendering
    if steps > 0 {
        let model = Arc::clone(&res.model);
        for env in res.batch.envs_mut() {
            let _ = env.forward(&model);
        }
    }
}

// ── Keep dummy PhysicsData time in sync for ValidationHarness ────────────

fn sync_dummy_time(res: Res<BatchResource>, mut data: ResMut<sim_bevy::model_data::PhysicsData>) {
    if let Some(env0) = res.batch.env(0) {
        data.0.time = env0.time;
    }
}

// ── Sync batch geom poses → PhysicsScenes for rendering ─────────────────

fn sync_batch_to_scenes(res: Res<BatchResource>, mut scenes: ResMut<PhysicsScenes>) {
    for i in 0..NUM_ENVS {
        if let (Some(env), Some(scene)) = (res.batch.env(i), scenes.get_mut(i)) {
            for g in 0..env.geom_xpos.len() {
                if g < scene.data.geom_xpos.len() {
                    scene.data.geom_xpos[g] = env.geom_xpos[g];
                    scene.data.geom_xmat[g] = env.geom_xmat[g];
                }
            }
        }
    }
}

// ── Validation ───────────────────────────────────────────────────────────

fn track_validation(
    res: Res<BatchResource>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<SweepValidation>,
) {
    if !harness.reported() || val.reported {
        return;
    }
    val.reported = true;

    let energies: Vec<f64> = (0..NUM_ENVS)
        .map(|i| res.batch.env(i).expect("env").total_energy())
        .collect();

    let e0_initial = res.initial_energies[0];

    // Energy scale of the system: mgl (mass * gravity * COM distance).
    // At 90° tilt, initial E ≈ 0 (COM at pivot height). At rest (hanging
    // down), E = -mgl ≈ -4.905 J. Use mgl as the reference for drift.
    let mgl = 1.0 * 9.81 * 0.5; // mass=1, g=9.81, l_com=0.5

    // Check 1: Undamped energy conserved (D=0)
    // Euler integrator drifts, so allow 1% of mgl
    let drift_abs = (energies[0] - e0_initial).abs();
    let drift_pct = drift_abs / mgl * 100.0;
    let check1 = Check {
        name: "Undamped energy conserved",
        pass: drift_abs < 0.01 * mgl,
        detail: format!(
            "E0_initial={e0_initial:.4}, E0_now={:.4}, |drift|={drift_abs:.4} ({drift_pct:.2}% of mgl)",
            energies[0],
        ),
    };

    // Check 2: Highest damping settles near rest energy (-mgl)
    let e_rest = -mgl;
    let settle_err = (energies[7] - e_rest).abs();
    let check2 = Check {
        name: "Damped energy settles",
        pass: settle_err < 0.05 * mgl,
        detail: format!(
            "E7={:.4}, E_rest={e_rest:.4}, |err|={settle_err:.4} ({:.1}% of mgl)",
            energies[7],
            settle_err / mgl * 100.0,
        ),
    };

    // Check 3: Energy monotonically ordered
    let monotonic = (0..NUM_ENVS - 1).all(|i| energies[i] > energies[i + 1]);
    let check3 = Check {
        name: "Energy monotonically ordered",
        pass: monotonic,
        detail: format!(
            "E = [{}]",
            energies
                .iter()
                .map(|e| format!("{e:.3}"))
                .collect::<Vec<_>>()
                .join(", ")
        ),
    };

    // Check 4: All envs stepped to ~10s
    let all_stepped = (0..NUM_ENVS).all(|i| {
        let t = res.batch.env(i).expect("env").time;
        (t - REPORT_TIME).abs() < 0.1
    });
    let check4 = Check {
        name: "All envs stepped",
        pass: all_stepped,
        detail: format!(
            "times = [{}]",
            (0..NUM_ENVS)
                .map(|i| format!("{:.1}", res.batch.env(i).expect("env").time))
                .collect::<Vec<_>>()
                .join(", ")
        ),
    };

    let _ = print_report(
        &format!("Parameter Sweep (t={REPORT_TIME}s)"),
        &[check1, check2, check3, check4],
    );
}

// ── HUD ──────────────────────────────────────────────────────────────────

fn update_hud(res: Res<BatchResource>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();
    hud.section("Parameter Sweep");

    if let Some(env0) = res.batch.env(0) {
        hud.scalar("time", env0.time, 1);
    }

    hud.section("Per-Env Energy");
    for i in 0..NUM_ENVS {
        if let Some(env) = res.batch.env(i) {
            let e = env.total_energy();
            let pct = if res.initial_energies[i].abs() > 1e-15 {
                e / res.initial_energies[i] * 100.0
            } else {
                0.0
            };
            hud.raw(format!("D={:.1}  E={:+.3}J  ({:.0}%)", DAMPING[i], e, pct));
        }
    }
}
