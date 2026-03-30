//! Jacobian — End-Effector Velocity Mapping
//!
//! Demonstrates the analytical Jacobian: how joint velocities map to
//! Cartesian end-effector velocity via `v = J * qdot`.
//!
//! Two identical arms swing side by side under gravity:
//!   Left  = STALE Jacobian (computed once at t=0, never updated)
//!   Right = CORRECT Jacobian (recomputed every physics step)
//!
//! Each arm has two arrows at its end-effector:
//!   Green = Jacobian prediction (J * qdot)
//!   Red   = finite-difference actual velocity
//!
//! On the right arm (correct), green covers red perfectly.
//! On the left arm (stale), green diverges from red as the arm moves
//! away from its initial configuration — showing WHY the Jacobian must
//! be recomputed at each configuration.
//!
//! Validates:
//! - Correct Jacobian velocity prediction matches FD within 1%
//! - Jacobian dimensions are correct (3x2 for a 2-DOF arm)
//! - Analytical form at zero configuration matches known geometry
//!
//! Run with: `cargo run -p example-inverse-jacobian --release`

#![allow(
    clippy::doc_markdown,
    clippy::needless_pass_by_value,
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::let_underscore_must_use,
    clippy::suboptimal_flops,
    clippy::cast_sign_loss,
    clippy::cast_precision_loss,
    clippy::similar_names,
    clippy::too_many_arguments,
    clippy::missing_const_for_fn,
    clippy::needless_range_loop
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
use sim_core::mj_jac_site;
use sim_core::validation::{Check, print_report};

// ── MJCF Model ──────────────────────────────────────────────────────────────

const MJCF: &str = r#"
<mujoco model="jacobian">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.002" integrator="RK4">
    <flag energy="enable"/>
  </option>

  <default>
    <geom contype="0" conaffinity="0"/>
    <joint damping="0.3"/>
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

const L1: f64 = 0.4;
const L2: f64 = 0.3;
const SHOULDER_INIT: f64 = 1.0;
const ELBOW_INIT: f64 = -0.5;
const ARROW_SCALE: f32 = 0.3;
const ARM_SPACING: f32 = 0.8;

// ── Marker components ───────────────────────────────────────────────────────

#[derive(Component)]
struct StaleArm;

#[derive(Component)]
struct CorrectArm;

// ── Resources ───────────────────────────────────────────────────────────────

#[derive(Resource)]
struct JacobianConfig {
    site_id: usize,
    zero_config_err: f64,
    jacp_rows: usize,
    jacp_cols: usize,
    nv: usize,
    /// The stale Jacobian: computed once at t=0, never updated
    stale_jacp: Vec<Vec<f64>>, // 3 rows × nv cols (avoid nalgebra dep)
}

/// Second data instance for the stale arm (both arms have identical physics)
#[derive(Resource)]
struct StaleArmData {
    data: sim_core::Data,
}

/// Per-frame arrow data for both arms
#[derive(Resource, Default)]
struct ArrowState {
    // Correct arm (right)
    correct_ee: [f64; 3],
    correct_v_pred: [f64; 3],
    correct_v_actual: [f64; 3],
    // Stale arm (left)
    stale_ee: [f64; 3],
    stale_v_pred: [f64; 3],
    stale_v_actual: [f64; 3],
}

// ── Bevy App ────────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Jacobian Velocity Mapping ===");
    println!("  Left arm:  STALE Jacobian (computed once at t=0)");
    println!("  Right arm: CORRECT Jacobian (recomputed every step)");
    println!("  Green arrow: Jacobian prediction (J * qdot)");
    println!("  Red arrow:   finite-difference actual velocity");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Jacobian".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .init_resource::<PhysicsHud>()
        .init_resource::<JacobianValidation>()
        .init_resource::<ArrowState>()
        .insert_resource(
            ValidationHarness::new()
                .report_at(6.0)
                .print_every(1.0)
                .display(|m, d| {
                    let s = d.sensor_scalar(m, "s_jpos").unwrap_or(0.0);
                    let e = d.sensor_scalar(m, "e_jpos").unwrap_or(0.0);
                    format!("shoulder={s:.3}  elbow={e:.3}")
                }),
        )
        .add_systems(Startup, setup)
        .add_systems(Update, step_and_compute)
        .add_systems(
            PostUpdate,
            (
                sync_correct_transforms,
                sync_stale_transforms,
                draw_velocity_arrows,
                validation_system,
                jacobian_diagnostics,
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

    data.qpos[0] = SHOULDER_INIT;
    data.qpos[1] = ELBOW_INIT;
    data.forward(&model).expect("forward");

    let site_id = model.site_id("end_effector").expect("site should exist");

    // Compute stale Jacobian at initial configuration (frozen forever)
    let (stale_jacp_mat, _) = mj_jac_site(&model, &data, site_id);
    let mut stale_jacp = vec![vec![0.0; model.nv]; 3];
    for row in 0..3 {
        for col in 0..model.nv {
            stale_jacp[row][col] = stale_jacp_mat[(row, col)];
        }
    }

    // Check Jacobian at zero configuration (analytical verification)
    let mut zero_data = model.make_data();
    zero_data.forward(&model).expect("forward");
    let (jacp_zero, _) = mj_jac_site(&model, &zero_data, site_id);

    let expected_shoulder_x = -(L1 + L2);
    let actual_shoulder_x = jacp_zero[(0, 0)];
    let zero_config_err = (actual_shoulder_x - expected_shoulder_x).abs();

    println!("  Jacobian at zero config:");
    println!(
        "    jacp shoulder col: [{:.4}, {:.4}, {:.4}]",
        jacp_zero[(0, 0)],
        jacp_zero[(1, 0)],
        jacp_zero[(2, 0)]
    );
    println!("    expected:          [{expected_shoulder_x:.4}, 0, 0]");
    println!("    error: {zero_config_err:.2e}");
    println!(
        "    jacp dims: {}x{} (expect 3x{})\n",
        jacp_zero.nrows(),
        jacp_zero.ncols(),
        model.nv
    );

    // Correct arm (right side) — solid metal
    let mat_rod = materials.add(MetalPreset::BrushedMetal.material());
    let mat_tip = materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.85, 0.3, 0.2)));

    spawn_model_geoms_with(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[
            ("upper_rod", mat_rod.clone()),
            ("lower_rod", mat_rod),
            ("tip", mat_tip),
        ],
        |entity_cmd, _geom_id, _geom_name| {
            entity_cmd.insert(CorrectArm);
        },
    );

    // Stale arm (left side) — dimmer, slightly transparent
    let stale_mat = materials.add(StandardMaterial {
        base_color: Color::srgba(0.5, 0.5, 0.55, 0.7),
        alpha_mode: AlphaMode::Blend,
        ..default()
    });

    // Clone data for the stale arm (identical physics, different visuals)
    let stale_data = data.clone();

    spawn_model_geoms_with(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &stale_data,
        &[
            ("upper_rod", stale_mat.clone()),
            ("lower_rod", stale_mat.clone()),
            ("tip", stale_mat),
        ],
        |entity_cmd, _geom_id, _geom_name| {
            entity_cmd.insert(StaleArm);
        },
    );

    spawn_example_camera(
        &mut commands,
        Vec3::new(0.0, -0.35, 0.0),
        2.8,
        std::f32::consts::FRAC_PI_2,
        0.0,
    );

    spawn_physics_hud(&mut commands);

    commands.insert_resource(JacobianConfig {
        site_id,
        zero_config_err,
        jacp_rows: jacp_zero.nrows(),
        jacp_cols: jacp_zero.ncols(),
        nv: model.nv,
        stale_jacp,
    });
    commands.insert_resource(StaleArmData { data: stale_data });
    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── Step + Jacobian computation ─────────────────────────────────────────────

fn step_and_compute(
    model: Res<PhysicsModel>,
    mut data: ResMut<PhysicsData>,
    mut stale: ResMut<StaleArmData>,
    config: Res<JacobianConfig>,
    mut arrows: ResMut<ArrowState>,
    mut val: ResMut<JacobianValidation>,
    mut prev_correct: Local<Option<[f64; 3]>>,
    mut prev_stale: Local<Option<[f64; 3]>>,
) {
    let dt = model.timestep;
    let steps_per_frame = (dt.recip() * (1.0 / 60.0)).round() as usize;
    let steps_per_frame = steps_per_frame.max(1);

    let site_id = config.site_id;
    let nv = model.nv;

    for _ in 0..steps_per_frame {
        // ── Correct arm: recompute Jacobian each step ──
        let ee_before = data.site_xpos[site_id];
        let before_c = [ee_before.x, ee_before.y, ee_before.z];

        let (jacp, _) = mj_jac_site(&model, &data, site_id);
        let mut v_pred_c = [0.0; 3];
        for row in 0..3 {
            for col in 0..nv {
                v_pred_c[row] += jacp[(row, col)] * data.qvel[col];
            }
        }

        data.step(&model).expect("step");

        let ee_after = data.site_xpos[site_id];
        let after_c = [ee_after.x, ee_after.y, ee_after.z];

        // ── Stale arm: same physics, stale Jacobian ──
        let ee_before_s = stale.data.site_xpos[site_id];
        let before_s = [ee_before_s.x, ee_before_s.y, ee_before_s.z];

        // Use the frozen Jacobian from t=0
        let mut v_pred_s = [0.0; 3];
        for row in 0..3 {
            for col in 0..nv {
                v_pred_s[row] += config.stale_jacp[row][col] * stale.data.qvel[col];
            }
        }

        stale.data.step(&model).expect("stale step");

        let ee_after_s = stale.data.site_xpos[site_id];
        let after_s = [ee_after_s.x, ee_after_s.y, ee_after_s.z];

        // ── Central FD for correct arm ──
        if let Some(prev) = *prev_correct {
            let v_act = [
                (after_c[0] - prev[0]) / (2.0 * dt),
                (after_c[1] - prev[1]) / (2.0 * dt),
                (after_c[2] - prev[2]) / (2.0 * dt),
            ];

            let v_pred_mag =
                (v_pred_c[0] * v_pred_c[0] + v_pred_c[1] * v_pred_c[1] + v_pred_c[2] * v_pred_c[2])
                    .sqrt();
            let err_mag = ((v_pred_c[0] - v_act[0]).powi(2)
                + (v_pred_c[1] - v_act[1]).powi(2)
                + (v_pred_c[2] - v_act[2]).powi(2))
            .sqrt();

            if v_pred_mag > 0.05 {
                val.record(err_mag / v_pred_mag);
            }

            arrows.correct_v_actual = v_act;
        }

        // ── Central FD for stale arm ──
        if let Some(prev) = *prev_stale {
            let v_act = [
                (after_s[0] - prev[0]) / (2.0 * dt),
                (after_s[1] - prev[1]) / (2.0 * dt),
                (after_s[2] - prev[2]) / (2.0 * dt),
            ];
            arrows.stale_v_actual = v_act;
        }

        *prev_correct = Some(before_c);
        *prev_stale = Some(before_s);

        arrows.correct_ee = before_c;
        arrows.correct_v_pred = v_pred_c;
        arrows.stale_ee = before_s;
        arrows.stale_v_pred = v_pred_s;
    }
}

// ── Transform sync ──────────────────────────────────────────────────────────

fn sync_correct_transforms(
    data: Res<PhysicsData>,
    mut query: Query<(&ModelGeomIndex, &mut Transform), With<CorrectArm>>,
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

fn sync_stale_transforms(
    stale: Res<StaleArmData>,
    mut query: Query<(&ModelGeomIndex, &mut Transform), With<StaleArm>>,
) {
    for (geom_idx, mut transform) in &mut query {
        let idx = geom_idx.0;
        if idx < stale.data.geom_xpos.len() {
            let mut t = vec3_from_vector(&stale.data.geom_xpos[idx]);
            t.x -= ARM_SPACING / 2.0;
            transform.translation = t;
            transform.rotation = quat_from_physics_matrix(&stale.data.geom_xmat[idx]);
        }
    }
}

// ── Draw velocity arrows ────────────────────────────────────────────────────

/// Convert physics \[x,y,z\] (Z-up) to Bevy Vec3 (Y-up)
fn to_bevy(p: &[f64; 3]) -> Vec3 {
    Vec3::new(p[0] as f32, p[2] as f32, p[1] as f32)
}

fn draw_velocity_arrows(mut gizmos: Gizmos, arrows: Res<ArrowState>) {
    // ── Correct arm (right side) ──
    let ee_r = to_bevy(&arrows.correct_ee) + Vec3::new(ARM_SPACING / 2.0, 0.0, 0.0);
    let vp_r = to_bevy(&arrows.correct_v_pred) * ARROW_SCALE;
    let va_r = to_bevy(&arrows.correct_v_actual) * ARROW_SCALE;

    if va_r.length() > 0.001 {
        gizmos.arrow(ee_r, ee_r + va_r, Color::srgb(0.9, 0.2, 0.2));
    }
    if vp_r.length() > 0.001 {
        gizmos.arrow(ee_r, ee_r + vp_r, Color::srgb(0.2, 0.9, 0.2));
    }

    // ── Stale arm (left side) ──
    let ee_l = to_bevy(&arrows.stale_ee) - Vec3::new(ARM_SPACING / 2.0, 0.0, 0.0);
    let vp_l = to_bevy(&arrows.stale_v_pred) * ARROW_SCALE;
    let va_l = to_bevy(&arrows.stale_v_actual) * ARROW_SCALE;

    if va_l.length() > 0.001 {
        gizmos.arrow(ee_l, ee_l + va_l, Color::srgb(0.9, 0.2, 0.2));
    }
    if vp_l.length() > 0.001 {
        gizmos.arrow(ee_l, ee_l + vp_l, Color::srgb(0.2, 0.9, 0.2));
    }
}

// ── HUD ─────────────────────────────────────────────────────────────────────

fn update_hud(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    val: Res<JacobianValidation>,
    mut hud: ResMut<PhysicsHud>,
) {
    hud.clear();
    hud.section("Jacobian");

    let s_pos = data.sensor_scalar(&model, "s_jpos").unwrap_or(0.0);
    let e_pos = data.sensor_scalar(&model, "e_jpos").unwrap_or(0.0);

    hud.scalar("shoulder", s_pos, 3);
    hud.scalar("elbow", e_pos, 3);
    hud.scalar("correct_err%", val.max_rel_err * 100.0, 4);
    hud.scalar("time", data.time, 1);
}

// ── Validation ──────────────────────────────────────────────────────────────

#[derive(Resource, Default)]
struct JacobianValidation {
    max_rel_err: f64,
    count: u64,
    reported: bool,
}

impl JacobianValidation {
    fn record(&mut self, rel_err: f64) {
        self.count += 1;
        if rel_err > self.max_rel_err {
            self.max_rel_err = rel_err;
        }
    }
}

fn jacobian_diagnostics(
    harness: Res<ValidationHarness>,
    config: Res<JacobianConfig>,
    mut val: ResMut<JacobianValidation>,
) {
    if !harness.reported() || val.reported {
        return;
    }
    val.reported = true;

    let dims_ok = config.jacp_rows == 3 && config.jacp_cols == config.nv;

    let checks = vec![
        Check {
            name: "Jacobian dims (3 x nv)",
            pass: dims_ok,
            detail: format!(
                "{}x{} (expect 3x{})",
                config.jacp_rows, config.jacp_cols, config.nv
            ),
        },
        Check {
            name: "Zero-config analytical form",
            pass: config.zero_config_err < 1e-6,
            detail: format!("err={:.2e}", config.zero_config_err),
        },
        Check {
            name: "Correct J*qdot vs FD < 1%",
            pass: val.max_rel_err < 0.01,
            detail: format!(
                "max rel err={:.4}% over {} samples",
                val.max_rel_err * 100.0,
                val.count
            ),
        },
    ];
    let _ = print_report("Jacobian (t=6s)", &checks);
}
