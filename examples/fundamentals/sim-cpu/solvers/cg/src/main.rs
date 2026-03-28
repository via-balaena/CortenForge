//! CG Solver — Conjugate Gradient on a Two-Box Stack
//!
//! Two boxes stacked on a ground plane, solved with CG (primal-space,
//! M^-1 preconditioner). The HUD shows per-step iteration count and max
//! contact penetration. CG converges faster than PGS but slower than Newton.
//!
//! Validates:
//! - Stack stable: `box_a` z-drift < 1mm over 5s
//! - CG converges: avg iterations <= 50
//!
//! Run with: `cargo run -p example-solver-cg --release`

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

// ── What the engine computes (CG solver) ───────────────────────────────────
//
//   Preconditioned Conjugate Gradient — primal-space constraint optimizer:
//
//   Objective: minimize cost(qacc) where
//     cost = ½·(M·qacc - qfrc_smooth)·(qacc - qacc_smooth)
//          + Σᵢ constraint_cost_i(Jᵢ·qacc - aref_i)
//
//   Main loop (for iter = 0..max_iters):
//     1. Line search along search direction:
//          mv = M · search,  jv = J · search
//          α = exact_line_search(search, mv, jv)
//     2. Move: qacc ← qacc + α · search
//     3. Re-classify constraints (update active set, forces, states)
//     4. Gradient:  grad = M·qacc - qfrc_smooth - Jᵀ·efc_force
//     5. Precondition: mgrad = M⁻¹ · grad   (sparse LDL solve)
//     6. Polak-Ribière direction update:
//          β = grad·(mgrad - mgrad_old) / (grad_old · mgrad_old)
//          search = -mgrad + β · search
//     7. Convergence: if improvement < tol AND |grad| < tol, stop
//
//   Properties:
//   - Superlinear convergence for well-conditioned problems
//   - M⁻¹ preconditioning leverages existing mass-matrix factorization
//   - No Hessian assembly (cheaper per-iteration than Newton)
//   - Falls back to PGS if line search fails
//
// Source: sim/L0/core/src/constraint/solver/cg.rs

// ── MJCF Model ──────────────────────────────────────────────────────────────

const MJCF: &str = r#"
<mujoco model="solver-cg">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.002" solver="CG"
          iterations="100" tolerance="1e-10">
    <flag energy="enable"/>
  </option>

  <worldbody>
    <geom name="ground" type="plane" size="2 2 0.01"
          friction="0.5 0.005 0.001" rgba="0.35 0.35 0.38 1"/>

    <body name="box_a" pos="0 0 0.1">
      <joint type="free" name="jnt_a"/>
      <geom name="box_a" type="box" size="0.1 0.1 0.1" mass="1.0"
            friction="0.5 0.005 0.001" rgba="0.82 0.22 0.15 1"/>
    </body>

    <body name="box_b" pos="0 0 0.3">
      <joint type="free" name="jnt_b"/>
      <geom name="box_b" type="box" size="0.1 0.1 0.1" mass="0.5"
            friction="0.5 0.005 0.001" rgba="0.15 0.45 0.82 1"/>
    </body>
  </worldbody>
</mujoco>
"#;

const SOLVER_NAME: &str = "CG";

// ── Bevy App ────────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: {SOLVER_NAME} Solver ===");
    println!("  Conjugate Gradient — primal-space, M^-1 preconditioner");
    println!("  Two boxes stacked on ground, dt = 0.002");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: format!("CortenForge — {SOLVER_NAME} Solver"),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .init_resource::<PhysicsHud>()
        .init_resource::<SolverValidation>()
        .insert_resource(
            ValidationHarness::new()
                .report_at(5.0)
                .print_every(1.0)
                .display(|_m, d| format!("iter={} ncon={}", d.solver_niter, d.ncon)),
        )
        .add_systems(Startup, setup)
        .add_systems(Update, step_physics_realtime)
        .add_systems(
            PostUpdate,
            (
                sync_geom_transforms,
                validation_system,
                solver_diagnostics,
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

    data.forward(&model).expect("forward should succeed");

    println!(
        "  Model: {} bodies, {} joints, {} geoms\n",
        model.nbody, model.njnt, model.ngeom
    );

    let mat_ground = materials.add(MetalPreset::BrushedMetal.material());
    let mat_box_a =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.82, 0.22, 0.15)));
    let mat_box_b =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.15, 0.45, 0.82)));

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[
            ("ground", mat_ground),
            ("box_a", mat_box_a),
            ("box_b", mat_box_b),
        ],
    );

    spawn_example_camera(
        &mut commands,
        Vec3::new(0.0, 0.0, 0.15),
        1.2,
        std::f32::consts::FRAC_PI_4,
        0.3,
    );

    spawn_physics_hud(&mut commands);

    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── HUD ─────────────────────────────────────────────────────────────────────

fn update_hud(_model: Res<PhysicsModel>, data: Res<PhysicsData>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();
    hud.section(SOLVER_NAME);

    hud.raw(format!("iter: {}", data.solver_niter));
    hud.raw(format!("contacts: {}", data.ncon));

    let max_depth = data.contacts[..data.ncon]
        .iter()
        .map(|c| c.depth)
        .fold(0.0_f64, f64::max);
    hud.raw(format!("max depth: {:.4} mm", max_depth * 1000.0));

    hud.scalar("energy", data.total_energy(), 4);
    hud.scalar("time", data.time, 1);
}

// ── Validation ──────────────────────────────────────────────────────────────

#[derive(Resource, Default)]
struct SolverValidation {
    reported: bool,
    total_iter: usize,
    steps: usize,
    rest_z: Option<f64>,
    max_drift: f64,
    settled: bool,
}

fn solver_diagnostics(
    data: Res<PhysicsData>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<SolverValidation>,
) {
    val.total_iter += data.solver_niter;
    val.steps += 1;

    let box_a_z = data.qpos[2];

    if !val.settled && data.time >= 0.5 {
        val.settled = true;
        val.rest_z = Some(box_a_z);
    }

    if val.settled
        && let Some(rz) = val.rest_z
    {
        let drift = (box_a_z - rz).abs();
        if drift > val.max_drift {
            val.max_drift = drift;
        }
    }

    if harness.reported() && !val.reported {
        val.reported = true;

        let avg_iter = val.total_iter as f64 / val.steps as f64;
        let drift_mm = val.max_drift * 1000.0;

        let checks = vec![
            Check {
                name: "Stack stable: drift < 1mm",
                pass: drift_mm < 1.0,
                detail: format!("max drift = {drift_mm:.3} mm"),
            },
            Check {
                name: "CG converges: avg iter <= 50",
                pass: avg_iter <= 50.0,
                detail: format!("avg = {avg_iter:.1}"),
            },
        ];
        let _ = print_report(&format!("{SOLVER_NAME} Solver (t=5s)"), &checks);
    }
}
