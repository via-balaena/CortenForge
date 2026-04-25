#![allow(
    missing_docs,
    clippy::doc_markdown,
    clippy::needless_pass_by_value,
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
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
use sim_bevy::model_data::{
    PhysicsAccumulator, PhysicsData, PhysicsModel, spawn_model_geoms, sync_geom_transforms,
    sync_rendering_data,
};
use sim_core::validation::{Check, print_report};
use sim_ml_chassis::{ActionSpace, Environment, ObservationSpace, SimEnv as ChassisSimEnv, Tensor};
use sim_ml_chassis_bevy::SimEnv;

// ── Constants ──────────────────────────────────────────────────────────────

const ANGLES: [f64; 4] = [0.5, 1.0, 1.5, 2.0];
const TRUNCATE_TIME: f64 = 2.0;
const REPORT_TIME: f64 = 15.0;

// ── Resources ──────────────────────────────────────────────────────────────

#[derive(Resource)]
struct OnResetState {
    episode: u32,
    last_start_angle: f64,
    angles_seen: Vec<f64>,
    sensor_matches: bool,
    reported: bool,
}

impl Default for OnResetState {
    fn default() -> Self {
        Self {
            episode: 1,
            last_start_angle: 0.0,
            angles_seen: Vec::new(),
            sensor_matches: true,
            reported: false,
        }
    }
}

// ── MJCF Model ─────────────────────────────────────────────────────────────

const MJCF: &str = r#"
<mujoco>
  <option timestep="0.002"/>
  <worldbody>
    <body name="pendulum" pos="0 0 1">
      <joint name="hinge" type="hinge" axis="0 1 0" damping="0.02"/>
      <geom name="rod" type="capsule" size="0.05"
            fromto="0 0 0 0 0 -0.5" mass="1"
            rgba="0.48 0.48 0.50 1"/>
      <geom name="tip" type="sphere" size="0.07"
            pos="0 0 -0.5" rgba="0.2 0.5 0.85 1"/>
    </body>
  </worldbody>
  <actuator>
    <motor joint="hinge" name="motor"/>
  </actuator>
  <sensor>
    <jointpos joint="hinge" name="angle"/>
    <jointvel joint="hinge" name="angvel"/>
  </sensor>
</mujoco>
"#;

// ── Bevy App ───────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Domain Randomization via on_reset ===");
    println!("  Angles: {ANGLES:?} (cycling)");
    println!("  Truncated: time > {TRUNCATE_TIME:.0} s");
    println!("  Action: zero (pure gravity)");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    // ── Build SimEnv outside Bevy ──

    let model_arc = Arc::new(sim_mjcf::load_model(MJCF).expect("MJCF parse"));

    let obs = ObservationSpace::builder()
        .all_qpos()
        .all_qvel()
        .build(&model_arc)
        .expect("obs build");
    let act = ActionSpace::builder()
        .all_ctrl()
        .build(&model_arc)
        .expect("act build");

    println!("  obs_dim={}, act_dim={}", obs.dim(), act.dim());

    let mut cycle_index = 0_usize;
    let truncate_time = TRUNCATE_TIME;

    let mut env = ChassisSimEnv::builder(model_arc.clone())
        .observation_space(obs)
        .action_space(act)
        .reward(|_m, _d| 0.0)
        .done(|_m, _d| false)
        .truncated(move |_m, d| d.time > truncate_time)
        .on_reset(move |_m, data| {
            data.qpos[0] = ANGLES[cycle_index % ANGLES.len()];
            cycle_index += 1;
        })
        .build()
        .expect("env build");

    // First reset invokes on_reset — starts at ANGLES[0] = 0.5
    env.reset().expect("reset");

    let model_owned = (*model_arc).clone();
    let data_owned = env.data().clone();

    // Record the first starting angle
    let first_angle = env.data().qpos[0];

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — on_reset".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .insert_resource(PhysicsModel(model_owned))
        .insert_resource(PhysicsData(data_owned))
        .insert_resource(SimEnv::from(env))
        .init_resource::<PhysicsAccumulator>()
        .init_resource::<PhysicsHud>()
        .insert_resource(OnResetState {
            episode: 1,
            last_start_angle: first_angle,
            angles_seen: vec![first_angle],
            sensor_matches: true,
            reported: false,
        })
        .insert_resource(
            ValidationHarness::new()
                .wall_clock()
                .report_at(REPORT_TIME)
                .print_every(1.0)
                .display(|m, d| {
                    let angle = d.sensor_scalar(m, "angle").unwrap_or(0.0);
                    let angvel = d.sensor_scalar(m, "angvel").unwrap_or(0.0);
                    format!("angle={angle:+.4}  angvel={angvel:+.4}")
                }),
        )
        .add_systems(Startup, setup)
        .add_systems(Update, step_env)
        .add_systems(
            PostUpdate,
            (
                sync_geom_transforms,
                validation_system,
                on_reset_diagnostics,
                update_hud,
                render_physics_hud,
            )
                .chain(),
        )
        .run();
}

// ── Setup ──────────────────────────────────────────────────────────────────

fn setup(
    mut commands: Commands,
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let mat_rod = materials.add(MetalPreset::BrushedMetal.material());
    let mat_tip = materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.2, 0.5, 0.85)));
    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[("rod", mat_rod), ("tip", mat_tip)],
    );

    spawn_example_camera(
        &mut commands,
        Vec3::new(0.0, 0.0, 0.75),
        2.0,
        std::f32::consts::FRAC_PI_2,
        0.0,
    );
    spawn_physics_hud(&mut commands);
}

// ── Stepping (Update) ─────────────────────────────────────────────────────

fn step_env(
    mut env: ResMut<SimEnv>,
    model: Res<PhysicsModel>,
    mut physics_data: ResMut<PhysicsData>,
    time: Res<Time>,
    mut acc: ResMut<PhysicsAccumulator>,
    mut state: ResMut<OnResetState>,
) {
    acc.0 += time.delta_secs_f64();
    let dt = model.0.timestep;
    let action = Tensor::zeros(&[1]); // zero action — pure gravity

    let chunk = dt;
    #[allow(clippy::cast_sign_loss)]
    let budget = ((acc.0 / chunk).max(0.0) as u32).min(200);
    for _ in 0..budget {
        let result = env.step(&action).expect("step");

        if result.truncated {
            state.episode += 1;
            let _obs = env.reset().expect("reset");

            // Check 4: sensor must match qpos after on_reset + forward()
            let qpos = env.data().qpos[0];
            let sensor = env
                .data()
                .sensor_scalar(&model, "angle")
                .unwrap_or(f64::NAN);
            if (qpos - sensor).abs() > 1e-10 {
                state.sensor_matches = false;
            }

            state.last_start_angle = qpos;
            state.angles_seen.push(qpos);
        }

        acc.0 -= chunk;
    }

    sync_rendering_data(env.data(), &mut physics_data.0);
}

// ── HUD (PostUpdate) ──────────────────────────────────────────────────────

fn update_hud(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    state: Res<OnResetState>,
    mut hud: ResMut<PhysicsHud>,
) {
    hud.clear();

    hud.section("On-Reset");
    hud.raw(format!(" episode:     {}", state.episode));
    hud.scalar(" start_angle", state.last_start_angle, 4);

    // Show sequence with bracket around current angle
    let idx = (state.episode as usize - 1) % ANGLES.len();
    let seq: Vec<String> = ANGLES
        .iter()
        .enumerate()
        .map(|(i, a)| {
            if i == idx {
                format!("[{a}]")
            } else {
                format!("{a}")
            }
        })
        .collect();
    hud.raw(format!(" sequence:    {}", seq.join(" -> ")));

    hud.section("State");
    let angle = data.sensor_scalar(&model, "angle").unwrap_or(0.0);
    let angvel = data.sensor_scalar(&model, "angvel").unwrap_or(0.0);
    hud.scalar("angle", angle, 4);
    hud.scalar("angvel", angvel, 4);
    hud.scalar("sim_time", data.time, 2);
}

// ── Validation (PostUpdate) ───────────────────────────────────────────────

fn on_reset_diagnostics(
    harness: Res<ValidationHarness>,
    env: Res<SimEnv>,
    mut state: ResMut<OnResetState>,
) {
    if !harness.reported() || state.reported {
        return;
    }
    state.reported = true;

    // Check 1: at least 4 episodes (one full cycle)
    let enough_episodes = state.episode >= 5;

    // Check 2: all 4 angles seen
    let all_seen = ANGLES.iter().all(|target| {
        state
            .angles_seen
            .iter()
            .any(|seen| (*seen - *target).abs() < 1e-10)
    });

    // Check 3: angles cycle in order
    let in_order = state.angles_seen.iter().enumerate().all(|(i, seen)| {
        let expected = ANGLES[i % ANGLES.len()];
        (*seen - expected).abs() < 1e-10
    });

    // Check 4: sensor matched qpos at every reset (tracked inline)
    let sensor_ok = state.sensor_matches;

    // Check 5: no NaN
    let obs = env.observe();
    let no_nan = obs.as_slice().iter().all(|v| v.is_finite());

    let checks = vec![
        Check {
            name: "At least 4 episodes",
            pass: enough_episodes,
            detail: format!("episodes={}", state.episode),
        },
        Check {
            name: "All 4 angles seen",
            pass: all_seen,
            detail: format!("seen={:?}", state.angles_seen),
        },
        Check {
            name: "Angles cycle in order",
            pass: in_order,
            detail: format!(
                "first_4={:?}",
                &state.angles_seen[..state.angles_seen.len().min(4)]
            ),
        },
        Check {
            name: "Sensor reflects start angle",
            pass: sensor_ok,
            detail: format!("all_matched={sensor_ok}"),
        },
        Check {
            name: "No NaN",
            pass: no_nan,
            detail: format!("finite={}", obs.as_slice().iter().all(|v| v.is_finite())),
        },
    ];
    let _ = print_report("On-Reset (Domain Randomization)", &checks);
}
