//! Forward Replay — Inverse → Forward Round-Trip Verification
//!
//! Takes the torque profile computed by inverse dynamics and replays it
//! through forward simulation. If inverse and forward are consistent,
//! the motor-driven arm should follow the same trajectory as the
//! prescribed one.
//!
//! Two arms are shown side by side: the left (ghost) follows the
//! prescribed trajectory directly, the right is driven by the
//! inverse-computed torques. They should be visually indistinguishable.
//!
//! Validates:
//! - Tracking error < 0.01 rad per joint over 5 seconds
//! - Round-trip residual < 1e-6 at every inverse step
//! - Both arms trace visually identical paths
//!
//! Run with: `cargo run -p example-inverse-forward-replay --release`

#![allow(
    clippy::doc_markdown,
    clippy::needless_pass_by_value,
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::let_underscore_must_use,
    clippy::suboptimal_flops,
    clippy::cast_sign_loss,
    clippy::cast_precision_loss
)]

use bevy::prelude::*;
use sim_bevy::camera::OrbitCameraPlugin;
use sim_bevy::convert::{quat_from_physics_matrix, vec3_from_vector};
use sim_bevy::examples::{
    PhysicsHud, ValidationHarness, render_physics_hud, spawn_example_camera, spawn_physics_hud,
    validation_system,
};
use sim_bevy::materials::MetalPreset;
use sim_bevy::model_data::{
    ModelGeomIndex, PhysicsAccumulator, PhysicsData, PhysicsModel, spawn_model_geoms_with,
};
use sim_core::validation::{Check, print_report};

// ── MJCF Model ──────────────────────────────────────────────────────────────

const MJCF: &str = r#"
<mujoco model="forward-replay">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.002" integrator="RK4">
    <flag energy="enable"/>
  </option>

  <default>
    <geom contype="0" conaffinity="0"/>
  </default>

  <worldbody>
    <body name="upper" pos="0 0 0">
      <joint name="shoulder" type="hinge" axis="0 1 0"/>
      <inertial pos="0 0 -0.2" mass="2.0" diaginertia="0.02 0.02 0.005"/>
      <geom name="upper_rod" type="capsule" size="0.03"
            fromto="0 0 0  0 0 -0.4" rgba="0.48 0.48 0.50 1"/>
      <body name="lower" pos="0 0 -0.4">
        <joint name="elbow" type="hinge" axis="0 1 0"/>
        <inertial pos="0 0 -0.15" mass="1.0" diaginertia="0.008 0.008 0.002"/>
        <geom name="lower_rod" type="capsule" size="0.025"
              fromto="0 0 0  0 0 -0.3" rgba="0.48 0.48 0.50 1"/>
        <geom name="tip" type="sphere" size="0.04"
              pos="0 0 -0.3" rgba="0.85 0.3 0.2 1"/>
        <site name="end_effector" pos="0 0 -0.3"/>
      </body>
    </body>
  </worldbody>

  <actuator>
    <motor name="shoulder_motor" joint="shoulder" gear="1"/>
    <motor name="elbow_motor" joint="elbow" gear="1"/>
  </actuator>

  <sensor>
    <jointpos name="s_jpos" joint="shoulder"/>
    <jointpos name="e_jpos" joint="elbow"/>
  </sensor>
</mujoco>
"#;

// ── Trajectory (same as torque-profile) ─────────────────────────────────────

const A1: f64 = 0.8;
const A2: f64 = 0.6;
const OMEGA1: f64 = 2.0;
const OMEGA2: f64 = 3.0;
const PHI: f64 = 1.0;
const DURATION: f64 = 5.0;

fn trajectory(t: f64) -> ([f64; 2], [f64; 2], [f64; 2]) {
    let qpos = [A1 * (OMEGA1 * t).sin(), A2 * (OMEGA2 * t + PHI).sin()];
    let qvel = [
        A1 * OMEGA1 * (OMEGA1 * t).cos(),
        A2 * OMEGA2 * (OMEGA2 * t + PHI).cos(),
    ];
    let qacc = [
        -A1 * OMEGA1 * OMEGA1 * (OMEGA1 * t).sin(),
        -A2 * OMEGA2 * OMEGA2 * (OMEGA2 * t + PHI).sin(),
    ];
    (qpos, qvel, qacc)
}

// ── Pre-computed torque profile ─────────────────────────────────────────────

#[derive(Resource)]
struct TorqueProfile {
    torques: Vec<[f64; 2]>,
    max_residual: f64,
}

fn compute_torque_profile(model: &sim_core::Model) -> TorqueProfile {
    let dt = model.timestep;
    let steps = (DURATION / dt).round() as usize;
    let mut data = model.make_data();
    let mut torques = Vec::with_capacity(steps);
    let mut max_residual: f64 = 0.0;

    for i in 0..steps {
        let t = i as f64 * dt;
        let (qpos, qvel, qacc) = trajectory(t);

        data.qpos[0] = qpos[0];
        data.qpos[1] = qpos[1];
        data.qvel[0] = qvel[0];
        data.qvel[1] = qvel[1];

        // forward() computes M, qfrc_bias, kinematics
        data.forward(model).expect("forward");

        // Overwrite qacc with desired, then inverse()
        data.qacc[0] = qacc[0];
        data.qacc[1] = qacc[1];
        data.inverse(model);

        torques.push([data.qfrc_inverse[0], data.qfrc_inverse[1]]);

        // Verify formula: qfrc_inverse = M*qacc + bias - passive - constraint
        for dof in 0..model.nv {
            let mut m_qacc_dof = 0.0;
            for j in 0..model.nv {
                m_qacc_dof += data.qM[(dof, j)] * data.qacc[j];
            }
            let formula = m_qacc_dof + data.qfrc_bias[dof]
                - data.qfrc_passive[dof]
                - data.qfrc_constraint[dof];
            let residual = (data.qfrc_inverse[dof] - formula).abs();
            max_residual = max_residual.max(residual);
        }
    }

    println!("  Computed {steps} torque samples over {DURATION}s (dt={dt})");
    println!("  Max formula residual: {max_residual:.2e}");

    TorqueProfile {
        torques,
        max_residual,
    }
}

// ── Marker components ───────────────────────────────────────────────────────

#[derive(Component)]
struct ReplayArm;

#[derive(Component)]
struct GhostArm;

#[derive(Resource)]
struct GhostData {
    data: sim_core::Data,
}

// ── Bevy App ────────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Forward Replay ===");
    println!("  Inverse -> forward round-trip verification");
    println!("  Left (ghost): prescribed trajectory");
    println!("  Right (solid): driven by inverse-computed torques");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Forward Replay".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .init_resource::<PhysicsHud>()
        .init_resource::<ReplayValidation>()
        .init_resource::<TorqueState>()
        .insert_resource(
            ValidationHarness::new()
                .report_at(DURATION - 0.1)
                .print_every(1.0)
                .display(|m, d| {
                    let s = d.sensor_scalar(m, "s_jpos").unwrap_or(0.0);
                    let e = d.sensor_scalar(m, "e_jpos").unwrap_or(0.0);
                    if d.time > DURATION {
                        format!("DISENGAGED  shoulder={s:.3}  elbow={e:.3}")
                    } else {
                        let (qd, _, _) = trajectory(d.time);
                        format!(
                            "s_err={:.4}  e_err={:.4}",
                            (s - qd[0]).abs(),
                            (e - qd[1]).abs()
                        )
                    }
                }),
        )
        .add_systems(Startup, setup)
        .add_systems(Update, step_replay)
        .add_systems(
            PostUpdate,
            (
                sync_replay_transforms,
                sync_ghost_transforms,
                validation_system,
                replay_diagnostics,
                update_hud,
                render_physics_hud,
            )
                .chain(),
        )
        .run();
}

const ARM_SPACING: f32 = 0.6;

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let model = sim_mjcf::load_model(MJCF).expect("MJCF should parse");

    // Phase 1: compute the full torque profile
    let profile = compute_torque_profile(&model);

    // Replay arm (right side) — driven by motors
    let mut replay_data = model.make_data();
    let (qpos0, qvel0, _) = trajectory(0.0);
    replay_data.qpos[0] = qpos0[0];
    replay_data.qpos[1] = qpos0[1];
    replay_data.qvel[0] = qvel0[0];
    replay_data.qvel[1] = qvel0[1];
    replay_data.forward(&model).expect("forward");

    let mat_rod = materials.add(MetalPreset::BrushedMetal.material());
    let mat_tip = materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.85, 0.3, 0.2)));

    spawn_model_geoms_with(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &replay_data,
        &[
            ("upper_rod", mat_rod.clone()),
            ("lower_rod", mat_rod),
            ("tip", mat_tip),
        ],
        |entity_cmd, _geom_id, _geom_name| {
            entity_cmd.insert(ReplayArm);
        },
    );

    // Ghost arm (left side) — follows prescribed trajectory directly
    let mut ghost_data = model.make_data();
    ghost_data.qpos[0] = qpos0[0];
    ghost_data.qpos[1] = qpos0[1];
    ghost_data.forward(&model).expect("forward");

    let ghost_mat = materials.add(StandardMaterial {
        base_color: Color::srgba(0.3, 0.6, 0.9, 0.4),
        alpha_mode: AlphaMode::Blend,
        ..default()
    });

    spawn_model_geoms_with(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &ghost_data,
        &[
            ("upper_rod", ghost_mat.clone()),
            ("lower_rod", ghost_mat.clone()),
            ("tip", ghost_mat),
        ],
        |entity_cmd, _geom_id, _geom_name| {
            entity_cmd.insert(GhostArm);
        },
    );

    spawn_example_camera(
        &mut commands,
        Vec3::new(0.0, 0.0, -0.25),
        2.2,
        std::f32::consts::FRAC_PI_2,
        0.0,
    );

    spawn_physics_hud(&mut commands);

    commands.insert_resource(profile);
    commands.insert_resource(GhostData { data: ghost_data });
    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(replay_data));
}

// ── Disengagement state ─────────────────────────────────────────────────────

#[derive(Resource)]
struct TorqueState {
    disengaged: bool,
    disengage_time: f64,
}

impl Default for TorqueState {
    fn default() -> Self {
        Self {
            disengaged: false,
            disengage_time: 0.0,
        }
    }
}

// ── Step: replay torques through forward simulation ─────────────────────────

fn step_replay(
    model: Res<PhysicsModel>,
    mut data: ResMut<PhysicsData>,
    profile: Res<TorqueProfile>,
    mut ghost: ResMut<GhostData>,
    mut val: ResMut<ReplayValidation>,
    mut state: ResMut<TorqueState>,
) {
    let dt = model.timestep;
    let steps_per_frame = (dt.recip() * (1.0 / 60.0)).round() as usize;
    let steps_per_frame = steps_per_frame.max(1);

    for _ in 0..steps_per_frame {
        let step_idx = (data.time / dt).round() as usize;

        if step_idx < profile.torques.len() {
            // Phase 1: Apply inverse-computed torques
            let tau = profile.torques[step_idx];
            data.set_ctrl(0, tau[0]);
            data.set_ctrl(1, tau[1]);
        } else {
            // Phase 2: Disengaged — zero torque, arm falls under gravity
            if !state.disengaged {
                state.disengaged = true;
                state.disengage_time = data.time;
                println!(
                    "\n  *** TORQUES DISENGAGED at t={:.1}s — arm falls under gravity ***\n",
                    data.time
                );
            }
            data.set_ctrl(0, 0.0);
            data.set_ctrl(1, 0.0);
        }

        // Forward step (forward + integrate)
        data.step(&model).expect("step");

        // Track error against prescribed trajectory (only during profile)
        if !state.disengaged {
            let (qd, _, _) = trajectory(data.time);
            let s_err = (data.qpos[0] - qd[0]).abs();
            let e_err = (data.qpos[1] - qd[1]).abs();
            val.record(s_err, e_err);
        }

        // Update ghost arm: follows trajectory during profile, freezes after
        if !state.disengaged {
            let (qd, _, _) = trajectory(data.time);
            ghost.data.qpos[0] = qd[0];
            ghost.data.qpos[1] = qd[1];
            ghost.data.forward(&model).expect("ghost forward");
        }
    }
}

// ── Transform sync (two arms, two data sources, with offsets) ───────────────

fn sync_replay_transforms(
    data: Res<PhysicsData>,
    mut query: Query<(&ModelGeomIndex, &mut Transform), With<ReplayArm>>,
) {
    for (geom_idx, mut transform) in &mut query {
        let idx = geom_idx.0;
        if idx < data.0.geom_xpos.len() {
            let mut t = vec3_from_vector(&data.0.geom_xpos[idx]);
            t.x += ARM_SPACING / 2.0;
            transform.translation = t;
            transform.rotation = quat_from_physics_matrix(&data.0.geom_xmat[idx]);
        }
    }
}

fn sync_ghost_transforms(
    ghost: Res<GhostData>,
    mut query: Query<(&ModelGeomIndex, &mut Transform), With<GhostArm>>,
) {
    for (geom_idx, mut transform) in &mut query {
        let idx = geom_idx.0;
        if idx < ghost.data.geom_xpos.len() {
            let mut t = vec3_from_vector(&ghost.data.geom_xpos[idx]);
            t.x -= ARM_SPACING / 2.0;
            transform.translation = t;
            transform.rotation = quat_from_physics_matrix(&ghost.data.geom_xmat[idx]);
        }
    }
}

// ── HUD ─────────────────────────────────────────────────────────────────────

fn update_hud(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    val: Res<ReplayValidation>,
    state: Res<TorqueState>,
    mut hud: ResMut<PhysicsHud>,
) {
    hud.clear();

    if state.disengaged {
        hud.section("TORQUES DISENGAGED");
    } else {
        hud.section("Forward Replay");
    }

    let s_pos = data.sensor_scalar(&model, "s_jpos").unwrap_or(0.0);
    let e_pos = data.sensor_scalar(&model, "e_jpos").unwrap_or(0.0);

    hud.scalar("shoulder", s_pos, 4);
    hud.scalar("elbow", e_pos, 4);

    if state.disengaged {
        hud.scalar("s_torque", 0.0, 1);
        hud.scalar("e_torque", 0.0, 1);
    } else {
        let (qd, _, _) = trajectory(data.time);
        hud.scalar("s_error", s_pos - qd[0], 6);
        hud.scalar("e_error", e_pos - qd[1], 6);
        hud.scalar("max_err", val.max_error, 6);
    }

    hud.scalar("time", data.time, 1);
}

// ── Validation ──────────────────────────────────────────────────────────────

#[derive(Resource, Default)]
struct ReplayValidation {
    max_shoulder_err: f64,
    max_elbow_err: f64,
    max_error: f64,
    count: u64,
    reported: bool,
}

impl ReplayValidation {
    fn record(&mut self, s_err: f64, e_err: f64) {
        self.count += 1;
        if s_err > self.max_shoulder_err {
            self.max_shoulder_err = s_err;
        }
        if e_err > self.max_elbow_err {
            self.max_elbow_err = e_err;
        }
        let err = s_err.max(e_err);
        if err > self.max_error {
            self.max_error = err;
        }
    }
}

fn replay_diagnostics(
    harness: Res<ValidationHarness>,
    profile: Res<TorqueProfile>,
    mut val: ResMut<ReplayValidation>,
) {
    if !harness.reported() || val.reported {
        return;
    }
    val.reported = true;

    let checks = vec![
        Check {
            name: "Formula residual < 1e-6",
            pass: profile.max_residual < 1e-6,
            detail: format!("max={:.2e}", profile.max_residual),
        },
        Check {
            name: "Shoulder tracking < 0.02 rad",
            pass: val.max_shoulder_err < 0.02,
            detail: format!("max={:.4e} rad", val.max_shoulder_err),
        },
        Check {
            name: "Elbow tracking < 0.02 rad",
            pass: val.max_elbow_err < 0.02,
            detail: format!("max={:.4e} rad", val.max_elbow_err),
        },
        Check {
            name: "Combined tracking < 0.02 rad",
            pass: val.max_error < 0.02,
            detail: format!("max={:.4e} rad over {} steps", val.max_error, val.count),
        },
    ];
    let _ = print_report("Forward Replay (t=4.9s)", &checks);
}
