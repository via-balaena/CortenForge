//! Soft Landing — Finding the Right Touch
//!
//! Twelve landers descend under gravity from 5 m, each with a different
//! constant thrust. Too little thrust → CRASH (slam into ground). Too much
//! → HOVER (never lands). Just right → SOFT LANDING.
//!
//! After each crash or hover, `BatchSim::reset_where(mask)` snaps the
//! lander back to 5 m and nudges its thrust toward the sweet spot
//! (`m·g ≈ 9.81 N`). Over ~10 s every lander converges and lands softly.
//!
//! Run: `cargo run -p example-batch-sim-reset-subset --release`

#![allow(
    clippy::doc_markdown,
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::cast_lossless,
    clippy::similar_names,
    clippy::needless_pass_by_value,
    clippy::needless_range_loop,
    clippy::let_underscore_must_use,
    clippy::suboptimal_flops,
    clippy::float_cmp
)]

use std::sync::Arc;

use bevy::prelude::*;
use sim_bevy::camera::OrbitCameraPlugin;
use sim_bevy::convert::physics_pos;
use sim_bevy::examples::{
    PhysicsHud, ValidationHarness, render_physics_hud, spawn_example_camera, spawn_physics_hud,
    validation_system,
};
use sim_bevy::materials::MetalPreset;
use sim_bevy::multi_scene::{PhysicsScenes, spawn_scene_geoms, sync_scene_geom_transforms};
use sim_core::batch::BatchSim;
use sim_core::validation::{Check, print_report};

// ── Config ───────────────────────────────────────────────────────────────

const NUM_ENVS: usize = 12;
const SPACING: f32 = 1.2;
const REPORT_TIME: f64 = 10.0;

/// Starting height (metres).
const START_HEIGHT: f64 = 3.0;

/// Soft-landing velocity threshold (m/s). Land slower than this → success.
/// v_max² = 2·(g − thrust/m)·h → landing window width ≈ 2.67 N.
const SOFT_VEL: f64 = 4.0;

/// Ground level — below this we evaluate landing.
const GROUND: f64 = 0.1;

/// Hover: if not descending at ≥ this speed after HOVER_TIME, reset.
const HOVER_VEL_THRESH: f64 = -0.3;
const HOVER_TIME: f64 = 1.0;

/// Hover thrust adaptation step (N).
const HOVER_NUDGE: f64 = 1.5;

/// Max crash thrust adaptation step (N).
const CRASH_NUDGE_MAX: f64 = 3.0;

/// Mass of the lander (kg) — must match MJCF.
const MASS: f64 = 1.0;

/// Initial thrust levels: 2 N to 15 N.
/// Equilibrium is m·g ≈ 9.81 N. Low end crashes, high end hovers.
fn initial_thrust(i: usize) -> f64 {
    let t = i as f64 / (NUM_ENVS - 1) as f64;
    2.0 + t * 13.0 // 2.0 → 15.0 N
}

// ── MJCF ─────────────────────────────────────────────────────────────────

/// Lander: capsule body + flat cylinder base on a vertical slide joint.
/// Motor actuator provides upward thrust via ctrl[0].
const MJCF: &str = r#"
<mujoco model="soft-landing">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.002" integrator="RK4">
    <flag energy="enable" contact="disable"/>
  </option>
  <default>
    <geom contype="0" conaffinity="0"/>
  </default>
  <worldbody>
    <body name="lander" pos="0 0 3">
      <joint name="slide" type="slide" axis="0 0 1" damping="0"/>
      <inertial pos="0 0 0" mass="1.0" diaginertia="0.01 0.01 0.01"/>
      <geom name="body" type="capsule" size="0.06"
            fromto="0 0 -0.15  0 0 0.15" rgba="0.50 0.50 0.53 1"/>
      <geom name="base" type="cylinder" size="0.10 0.02"
            pos="0 0 -0.17" rgba="0.35 0.33 0.32 1"/>
      <geom name="top" type="sphere" size="0.04"
            pos="0 0 0.17" rgba="0.85 0.85 0.85 1"/>
    </body>
  </worldbody>
  <actuator>
    <motor name="thrust" joint="slide" gear="1"/>
  </actuator>
</mujoco>
"#;

// ── Per-env state ───────────────────────────────────────────────────────

#[derive(Clone, Copy, PartialEq, Eq)]
enum LanderStatus {
    /// In flight — descending (or ascending).
    Flying,
    /// Hit the ground too fast.
    Crashed,
    /// Stuck above hover ceiling.
    Hovering,
    /// Touched down gently — success.
    Landed,
}

impl LanderStatus {
    const fn label(self) -> &'static str {
        match self {
            Self::Flying => "FLY",
            Self::Crashed => "CRASH",
            Self::Hovering => "HOVER",
            Self::Landed => "LAND",
        }
    }
}

// ── Resources ────────────────────────────────────────────────────────────

#[derive(Resource)]
struct BatchResource {
    batch: BatchSim,
    model: Arc<sim_core::Model>,
    accumulator: f64,
    sim_time: f64,
    /// Current thrust per env (adapted after each reset).
    thrusts: [f64; NUM_ENVS],
    /// Running status per env.
    status: [LanderStatus; NUM_ENVS],
    /// How many times each env has been reset.
    reset_counts: [u32; NUM_ENVS],
    /// Sim time at which each env last landed (for "untouched" check).
    landed_at: [f64; NUM_ENVS],
}

#[derive(Resource, Default)]
struct ResetValidation {
    reported: bool,
}

// ── Bevy App ─────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Soft Landing ===");
    println!("  12 landers, each finding the right thrust to land gently");
    println!("  Too little → crash | Too much → hover | Just right → soft landing");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Soft Landing (reset_where)".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsHud>()
        .init_resource::<ResetValidation>()
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

    let mut batch = BatchSim::new(Arc::clone(&model), NUM_ENVS);

    // All envs start at the model default (qpos0 = 0, meaning z = 5 from
    // the body pos). Nothing to set — the MJCF default is the start state.
    // Run forward to compute initial geom poses.
    for env in batch.envs_mut() {
        env.forward(&model).expect("forward");
    }

    // Initial thrusts
    let mut thrusts = [0.0; NUM_ENVS];
    for i in 0..NUM_ENVS {
        thrusts[i] = initial_thrust(i);
    }

    // ── Visual scenes ───────────────────────────────────────────────────
    let mut scenes = PhysicsScenes::default();

    for i in 0..NUM_ENVS {
        // Colour: red (low thrust / crashers) → blue (high thrust / hoverers)
        // Middle envs near equilibrium get a neutral gray that turns green on
        // landing (we'll handle the green shift via material swap or HUD).
        let t = i as f32 / (NUM_ENVS - 1) as f32;
        let body_color = Color::srgb(
            0.85 - 0.60 * t, // R: 0.85 → 0.25
            0.35,            // G: constant mid
            0.25 + 0.60 * t, // B: 0.25 → 0.85
        );

        let scene_model = (*model).clone();
        let mut scene_data = scene_model.make_data();
        scene_data.forward(&scene_model).expect("forward");

        let id = scenes.add(format!("{:.1}N", thrusts[i]), scene_model, scene_data);

        let mat_body = materials.add(MetalPreset::PolishedSteel.with_color(body_color));
        let mat_base =
            materials.add(MetalPreset::CastIron.with_color(Color::srgb(0.35, 0.33, 0.32)));
        let mat_top =
            materials.add(MetalPreset::BrushedMetal.with_color(Color::srgb(0.85, 0.85, 0.85)));

        // Space landers along physics Y-axis (side by side)
        let lane = (i as f32 - (NUM_ENVS as f32 - 1.0) / 2.0) * SPACING;
        spawn_scene_geoms(
            &mut commands,
            &mut meshes,
            &mut materials,
            &mut scenes,
            id,
            physics_pos(0.0, lane, 0.0),
            &[("body", mat_body), ("base", mat_base), ("top", mat_top)],
        );
    }

    // Ground plane at physics z=0 (Bevy y=0), facing Bevy Y-up
    commands.spawn((
        Mesh3d(meshes.add(Plane3d::new(Vec3::Y, Vec2::splat(50.0)))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.18, 0.20, 0.18),
            perceptual_roughness: 0.9,
            metallic: 0.0,
            ..default()
        })),
        Transform::from_translation(physics_pos(0.0, 0.0, 0.0)),
    ));

    commands.insert_resource(scenes);

    commands.insert_resource(BatchResource {
        batch,
        model: Arc::clone(&model),
        accumulator: 0.0,
        sim_time: 0.0,
        thrusts,
        status: [LanderStatus::Flying; NUM_ENVS],
        reset_counts: [0; NUM_ENVS],
        landed_at: [0.0; NUM_ENVS],
    });

    // Camera: side view showing height + all 12 lanes
    // Look at physics (0, 0, 2.5) = mid-height in Bevy Y-up
    spawn_example_camera(
        &mut commands,
        physics_pos(0.0, 0.0, 2.5),
        14.0,                        // distance
        std::f32::consts::FRAC_PI_2, // azimuth (side view)
        0.15,                        // elevation
    );

    spawn_physics_hud(&mut commands);

    // Dummy PhysicsModel/PhysicsData for ValidationHarness
    let dummy_model = (*model).clone();
    let dummy_data = dummy_model.make_data();
    commands.insert_resource(sim_bevy::model_data::PhysicsModel(dummy_model));
    commands.insert_resource(sim_bevy::model_data::PhysicsData(dummy_data));
}

// ── Stepping + Reset ─────────────────────────────────────────────────────

fn step_batch(mut res: ResMut<BatchResource>, time: Res<Time>) {
    let wall_dt = time.delta_secs_f64();
    res.accumulator += wall_dt;

    let dt_sim = res.batch.model().timestep;
    let mut steps = 0;

    while res.accumulator >= dt_sim && steps < 200 {
        // Snapshot per-env ctrl values before the mutable borrow.
        // Landed envs get exactly m·g to cancel gravity (net force = 0).
        let status_snap = res.status;
        let thrust_snap = res.thrusts;
        for (i, env) in res.batch.envs_mut().enumerate() {
            if status_snap[i] == LanderStatus::Landed {
                // Freeze: pin to ground, zero velocity, balance gravity
                env.qpos[0] = -START_HEIGHT; // body origin z minus start = ground
                env.qvel[0] = 0.0;
                env.ctrl[0] = MASS * 9.81;
            } else {
                env.ctrl[0] = thrust_snap[i];
            }
        }

        let _errors = res.batch.step_all();
        res.accumulator -= dt_sim;
        res.sim_time += dt_sim;
        steps += 1;

        // Re-freeze landed envs after step (prevents any drift)
        let status_post = res.status;
        for (i, env) in res.batch.envs_mut().enumerate() {
            if status_post[i] == LanderStatus::Landed {
                env.qpos[0] = -START_HEIGHT;
                env.qvel[0] = 0.0;
            }
        }

        // ── Evaluate landings / crashes / hovers ────────────────────
        let mut mask = [false; NUM_ENVS];
        let mut impact_vel = [0.0f64; NUM_ENVS]; // capture before reset

        for i in 0..NUM_ENVS {
            if res.status[i] == LanderStatus::Landed {
                continue; // already done
            }

            if let Some(env) = res.batch.env(i) {
                let z = env.qpos[0]; // slide displacement from body origin
                let height = START_HEIGHT + z; // actual height above ground
                let vel = env.qvel[0]; // downward is negative

                if height <= GROUND {
                    impact_vel[i] = vel;
                    if vel.abs() > SOFT_VEL {
                        // CRASH — too fast
                        res.status[i] = LanderStatus::Crashed;
                        mask[i] = true;
                    } else {
                        // SOFT LANDING — pin to ground immediately
                        res.status[i] = LanderStatus::Landed;
                        res.landed_at[i] = res.sim_time;
                        if let Some(env) = res.batch.env_mut(i) {
                            env.qpos[0] = -START_HEIGHT;
                            env.qvel[0] = 0.0;
                        }
                    }
                } else if env.time > HOVER_TIME && vel > HOVER_VEL_THRESH {
                    // HOVER — not descending meaningfully after HOVER_TIME.
                    // Uses env.time (resets to 0 on reset_where) so a
                    // freshly-reset env gets a full flight window.
                    res.status[i] = LanderStatus::Hovering;
                    mask[i] = true;
                }
            }
        }

        // Batch-reset all crashed / hovering envs
        if mask.iter().any(|&m| m) {
            // Adapt thrusts BEFORE reset (uses pre-reset state)
            for i in 0..NUM_ENVS {
                if mask[i] {
                    match res.status[i] {
                        LanderStatus::Crashed => {
                            // Proportional: harder crash → bigger nudge
                            let nudge = (impact_vel[i].abs() * 0.5).clamp(0.5, CRASH_NUDGE_MAX);
                            res.thrusts[i] += nudge;
                        }
                        LanderStatus::Hovering => {
                            res.thrusts[i] -= HOVER_NUDGE;
                        }
                        _ => {}
                    }
                }
            }

            res.batch.reset_where(&mask);

            for i in 0..NUM_ENVS {
                if mask[i] {
                    res.reset_counts[i] += 1;
                    res.status[i] = LanderStatus::Flying; // back in the air
                }
            }
        }
    }

    // Post-step forward to refresh geom poses
    if steps > 0 {
        let model = Arc::clone(&res.model);
        for env in res.batch.envs_mut() {
            let _ = env.forward(&model);
        }
    }
}

// ── Keep dummy PhysicsData time in sync for ValidationHarness ────────────

fn sync_dummy_time(res: Res<BatchResource>, mut data: ResMut<sim_bevy::model_data::PhysicsData>) {
    data.0.time = res.sim_time;
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
    mut val: ResMut<ResetValidation>,
) {
    if !harness.reported() || val.reported {
        return;
    }
    val.reported = true;

    // Check 1: Low-thrust envs crashed and were reset
    let low_reset = (0..2).all(|i| res.reset_counts[i] > 0);
    let check1 = Check {
        name: "Low-thrust envs crashed and reset",
        pass: low_reset,
        detail: format!(
            "reset_counts[0..2] = [{}, {}], thrusts = [{:.1}, {:.1}]",
            res.reset_counts[0], res.reset_counts[1], res.thrusts[0], res.thrusts[1],
        ),
    };

    // Check 2: High-thrust envs hovered and were reset
    let high_reset = (10..NUM_ENVS).all(|i| res.reset_counts[i] > 0);
    let check2 = Check {
        name: "High-thrust envs hovered and reset",
        pass: high_reset,
        detail: format!(
            "reset_counts[10..12] = [{}, {}], thrusts = [{:.1}, {:.1}]",
            res.reset_counts[10], res.reset_counts[11], res.thrusts[10], res.thrusts[11],
        ),
    };

    // Check 3: All envs eventually landed softly
    let all_landed = (0..NUM_ENVS).all(|i| res.status[i] == LanderStatus::Landed);
    let landed_count = (0..NUM_ENVS)
        .filter(|&i| res.status[i] == LanderStatus::Landed)
        .count();
    let check3 = Check {
        name: "All envs eventually landed",
        pass: all_landed,
        detail: format!(
            "{landed_count}/{NUM_ENVS} landed, status = [{}]",
            (0..NUM_ENVS)
                .map(|i| res.status[i].label())
                .collect::<Vec<_>>()
                .join(", ")
        ),
    };

    // Check 4: Landed envs were not reset after landing.
    // Once an env reaches Landed, we skip it in the reset mask, so its
    // env.time should exceed (REPORT_TIME - landed_at_global) — meaning
    // it has been stepping continuously since it landed.
    let landed_untouched = (0..NUM_ENVS)
        .filter(|&i| res.status[i] == LanderStatus::Landed)
        .all(|i| {
            // env.time must be > 0 (not freshly reset) and should have
            // been accumulating since at least the landing moment.
            res.batch.env(i).is_some_and(|e| e.time > 1.0)
        });
    let check4 = Check {
        name: "Landed envs untouched by reset_where",
        pass: landed_untouched,
        detail: format!(
            "env_times = [{}]",
            (0..NUM_ENVS)
                .filter(|&i| res.status[i] == LanderStatus::Landed)
                .map(|i| format!("{}:{:.1}s", i, res.batch.env(i).map_or(0.0, |e| e.time)))
                .collect::<Vec<_>>()
                .join(", ")
        ),
    };

    let _ = print_report(
        &format!("Soft Landing (t={REPORT_TIME}s)"),
        &[check1, check2, check3, check4],
    );
}

// ── HUD ──────────────────────────────────────────────────────────────────

fn update_hud(res: Res<BatchResource>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();
    hud.section("Soft Landing");
    hud.scalar("time", res.sim_time, 1);

    let landed = (0..NUM_ENVS)
        .filter(|&i| res.status[i] == LanderStatus::Landed)
        .count();
    let total_resets: u32 = res.reset_counts.iter().sum();
    hud.raw(format!("landed: {landed}/{NUM_ENVS}"));
    hud.raw(format!("total resets: {total_resets}"));

    hud.section("Per-Env");
    for i in 0..NUM_ENVS {
        if let Some(env) = res.batch.env(i) {
            let height = START_HEIGHT + env.qpos[0];
            let vel = env.qvel[0];
            hud.raw(format!(
                "{:>2} T={:>5.1}N  z={:>4.1}m  v={:>+5.1}  {}  #{}",
                i,
                res.thrusts[i],
                height,
                vel,
                res.status[i].label(),
                res.reset_counts[i],
            ));
        }
    }
}
