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
    PhysicsHud, ValidationHarness, insert_batch_validation_dummies, render_physics_hud,
    spawn_example_camera, spawn_physics_hud, validation_system,
};
use sim_bevy::materials::MetalPreset;
use sim_bevy::model_data::{PhysicsAccumulator, sync_rendering_data};
use sim_bevy::multi_scene::{PhysicsScenes, spawn_scene_geoms, sync_scene_geom_transforms};
use sim_core::validation::{Check, print_report};
use sim_ml_chassis::{ActionSpace, Environment, ObservationSpace, SimEnv, Tensor};

// ── Constants ──────────────────────────────────────────────────────────────

const REPORT_TIME: f64 = 15.0;
const SUB_STEPS_LEFT: usize = 1;
const SUB_STEPS_RIGHT: usize = 10;

// ── Resources ──────────────────────────────────────────────────────────────

#[derive(Resource)]
struct SubStepEnvs {
    left: SimEnv,
    right: SimEnv,
    left_act_steps: u64,
    right_act_steps: u64,
}

#[derive(Resource, Default)]
struct SubStepValidation {
    reported: bool,
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
            pos="0 0 -0.5"/>
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
    println!("=== CortenForge: Sub-Stepping (1 vs {SUB_STEPS_RIGHT} sub-steps) ===");
    println!("  Left:  sub_steps={SUB_STEPS_LEFT}  (500 Hz action rate)");
    println!("  Right: sub_steps={SUB_STEPS_RIGHT} ( 50 Hz action rate)");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    // ── Build two SimEnvs outside Bevy ──

    let model_arc = Arc::new(sim_mjcf::load_model(MJCF).expect("MJCF parse"));

    let build_env = |sub_steps: usize| -> SimEnv {
        let obs = ObservationSpace::builder()
            .all_qpos()
            .all_qvel()
            .build(&model_arc)
            .expect("obs build");
        let act = ActionSpace::builder()
            .all_ctrl()
            .build(&model_arc)
            .expect("act build");

        let mut env = SimEnv::builder(model_arc.clone())
            .observation_space(obs)
            .action_space(act)
            .reward(|_m, _d| 0.0)
            .done(|_m, _d| false)
            .truncated(|_m, _d| false)
            .sub_steps(sub_steps)
            .build()
            .expect("env build");

        env.reset().expect("reset");
        env
    };

    let left = build_env(SUB_STEPS_LEFT);
    let right = build_env(SUB_STEPS_RIGHT);

    println!(
        "  obs_dim={}, act_dim={}",
        left.observation_space().dim(),
        left.action_space().dim()
    );

    // ── Build PhysicsScenes for rendering ──

    let mut scenes = PhysicsScenes::default();
    let model_clone_l = left.model().clone();
    let data_clone_l = left.data().clone();
    let id_l = scenes.add("1 sub-step", model_clone_l, data_clone_l);

    let model_clone_r = right.model().clone();
    let data_clone_r = right.data().clone();
    let id_r = scenes.add("10 sub-steps", model_clone_r, data_clone_r);

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Sub-Stepping".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .insert_resource(SubStepEnvs {
            left,
            right,
            left_act_steps: 0,
            right_act_steps: 0,
        })
        .insert_resource(scenes)
        .init_resource::<PhysicsAccumulator>()
        .init_resource::<PhysicsHud>()
        .init_resource::<SubStepValidation>()
        .insert_resource(
            ValidationHarness::new()
                .wall_clock()
                .report_at(REPORT_TIME)
                .print_every(1.0),
        )
        .add_systems(
            Startup,
            move |mut commands: Commands,
                  mut meshes: ResMut<Assets<Mesh>>,
                  mut materials: ResMut<Assets<StandardMaterial>>,
                  mut scenes_res: ResMut<PhysicsScenes>| {
                setup(
                    &mut commands,
                    &mut meshes,
                    &mut materials,
                    &mut scenes_res,
                    id_l,
                    id_r,
                );
            },
        )
        .add_systems(Update, step_envs)
        .add_systems(
            PostUpdate,
            (
                sync_scene_geom_transforms,
                validation_system,
                sub_step_diagnostics,
                update_hud,
                render_physics_hud,
            )
                .chain(),
        )
        .run();
}

// ── Setup ──────────────────────────────────────────────────────────────────

fn setup(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    scenes: &mut PhysicsScenes,
    id_l: usize,
    id_r: usize,
) {
    // Materials — left tip blue, right tip orange, both rods brushed metal
    let mat_rod = materials.add(MetalPreset::BrushedMetal.material());
    let mat_tip_l =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.2, 0.5, 0.85)));
    let mat_tip_r =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.85, 0.5, 0.2)));

    spawn_scene_geoms(
        commands,
        meshes,
        materials,
        scenes,
        id_l,
        Vec3::new(-1.0, 0.0, 0.0),
        &[("rod", mat_rod.clone()), ("tip", mat_tip_l)],
    );
    spawn_scene_geoms(
        commands,
        meshes,
        materials,
        scenes,
        id_r,
        Vec3::new(1.0, 0.0, 0.0),
        &[("rod", mat_rod), ("tip", mat_tip_r)],
    );

    // Dummy PhysicsModel + PhysicsData for validation_system compatibility
    let model = scenes.get(id_l).expect("scene 0").model.clone();
    insert_batch_validation_dummies(commands, &model);

    spawn_example_camera(
        commands,
        Vec3::new(0.0, 0.0, 0.75),
        3.0,
        std::f32::consts::FRAC_PI_2,
        0.0,
    );
    spawn_physics_hud(commands);
}

// ── Stepping (Update) ─────────────────────────────────────────────────────

fn step_envs(
    mut envs: ResMut<SubStepEnvs>,
    mut scenes: ResMut<PhysicsScenes>,
    time: Res<Time>,
    mut acc: ResMut<PhysicsAccumulator>,
) {
    acc.0 += time.delta_secs_f64();
    let dt = envs.left.model().timestep; // 0.002
    let mut action = Tensor::zeros(&[1]); // pre-allocate, reuse

    // Both envs must consume the same physics time per outer iteration:
    //   Left:  10 × env.step(sub_steps=1)  = 10 physics steps
    //   Right:  1 × env.step(sub_steps=10) = 10 physics steps
    // Accumulator drains in chunks of 10*dt = 0.020s.

    let chunk = dt * SUB_STEPS_RIGHT as f64;
    #[allow(clippy::cast_sign_loss)]
    let mut budget = (acc.0 / chunk).max(0.0) as u32;
    budget = budget.min(200 / SUB_STEPS_RIGHT as u32);
    for _ in 0..budget {
        // Left: 10 individual steps, action recomputed each step (500 Hz).
        for _ in 0..SUB_STEPS_RIGHT {
            let t = envs.left.data().time;
            action.as_mut_slice()[0] = (t * 5.0).sin() as f32 * 10.0;
            envs.left.step(&action).expect("left step");
            envs.left_act_steps += 1;
        }

        // Right: 1 step with sub_steps=10, action computed once (50 Hz).
        let t = envs.right.data().time;
        action.as_mut_slice()[0] = (t * 5.0).sin() as f32 * 10.0;
        envs.right.step(&action).expect("right step");
        envs.right_act_steps += 1;

        acc.0 -= chunk;
    }

    // Sync to PhysicsScenes for rendering.
    if let Some(scene) = scenes.get_mut(0) {
        sync_rendering_data(envs.left.data(), &mut scene.data);
    }
    if let Some(scene) = scenes.get_mut(1) {
        sync_rendering_data(envs.right.data(), &mut scene.data);
    }
}

// ── HUD (PostUpdate) ──────────────────────────────────────────────────────

fn update_hud(envs: Res<SubStepEnvs>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();

    let model = envs.left.model();
    let dt = model.timestep;

    // Left env
    let ld = envs.left.data();
    let l_angle = ld.sensor_scalar(model, "angle").unwrap_or(0.0);
    let l_angvel = ld.sensor_scalar(model, "angvel").unwrap_or(0.0);
    let l_action = ld.ctrl.get(0).copied().unwrap_or(0.0);

    // Right env
    let rd = envs.right.data();
    let r_angle = rd.sensor_scalar(model, "angle").unwrap_or(0.0);
    let r_angvel = rd.sensor_scalar(model, "angvel").unwrap_or(0.0);
    let r_action = rd.ctrl.get(0).copied().unwrap_or(0.0);

    hud.section("Sub-Stepping");
    hud.raw(format!(" {:>14}    {:>14}", "1 sub-step", "10 sub-steps"));
    hud.raw(format!(" angle:  {l_angle:+.4}    angle:  {r_angle:+.4}"));
    hud.raw(format!(" angvel: {l_angvel:+.4}    angvel: {r_angvel:+.4}"));
    hud.raw(format!(" action: {l_action:+.4}    action: {r_action:+.4}"));
    hud.raw(format!(
        " act_steps: {:<8} act_steps: {}",
        envs.left_act_steps, envs.right_act_steps
    ));

    hud.section("Timing");
    hud.scalar("sim_time", ld.time, 2);
    #[allow(clippy::cast_sign_loss)]
    let physics_steps = (ld.time / dt) as u64;
    hud.raw(format!(" physics_steps: {physics_steps}"));
}

// ── Validation (PostUpdate) ───────────────────────────────────────────────

fn sub_step_diagnostics(
    harness: Res<ValidationHarness>,
    envs: Res<SubStepEnvs>,
    mut val: ResMut<SubStepValidation>,
) {
    if !harness.reported() || val.reported {
        return;
    }
    val.reported = true;

    let lt = envs.left.data().time;
    let rt = envs.right.data().time;
    let time_match = (lt - rt).abs() < 1e-12;

    let step_ratio_ok = envs.left_act_steps == SUB_STEPS_RIGHT as u64 * envs.right_act_steps;

    let lq = envs.left.data().qpos[0];
    let rq = envs.right.data().qpos[0];
    let diverged = (lq - rq).abs() > 0.01;

    let l_obs = envs.left.observe();
    let r_obs = envs.right.observe();
    let no_nan = l_obs.as_slice().iter().all(|v| v.is_finite())
        && r_obs.as_slice().iter().all(|v| v.is_finite());

    let checks = vec![
        Check {
            name: "Same sim time",
            pass: time_match,
            detail: format!("left={lt:.6}, right={rt:.6}"),
        },
        Check {
            name: "Left has 10x action steps",
            pass: step_ratio_ok,
            detail: format!(
                "left={}, right={}, ratio={}",
                envs.left_act_steps,
                envs.right_act_steps,
                if envs.right_act_steps > 0 {
                    envs.left_act_steps / envs.right_act_steps
                } else {
                    0
                }
            ),
        },
        Check {
            name: "Trajectories diverge",
            pass: diverged,
            detail: format!("|qpos_diff| = {:.6}", (lq - rq).abs()),
        },
        Check {
            name: "No NaN",
            pass: no_nan,
            detail: format!(
                "left_finite={}, right_finite={}",
                l_obs.as_slice().iter().all(|v| v.is_finite()),
                r_obs.as_slice().iter().all(|v| v.is_finite())
            ),
        },
    ];
    let _ = print_report("Sub-Stepping (1 vs 10)", &checks);
}
