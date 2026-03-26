//! Damper Actuator — Viscous Braking
//!
//! A velocity-dependent actuator that brakes a spinning arm. The damper
//! shortcut maps to `GainType::Affine(0, 0, -kv)`, `BiasType::None`,
//! `ActuatorDynamics::None`. Force = -kv * velocity * ctrl. The ctrl signal
//! scales damping (0 = off, 1 = full).
//!
//! The arm starts spinning at 10 rad/s in zero gravity. Unlike a constant-
//! force brake (linear slowdown), viscous braking pushes proportional to
//! speed — fast deceleration at high ω, then asymptotically approaching rest.
//! The result is textbook exponential decay: ω(t) = ω₀ · e^(-t/τ).
//!
//! Validates:
//! - Exponential velocity decay: ω(τ) / ω₀ ≈ e⁻¹ ≈ 0.368
//! - Time constant: velocity drops to 37% at t = τ = I_eff / kv
//! - Force proportional to velocity: force ≈ -kv * ω
//! - Affine gain path: gaintype is GainType::Affine (structural check)
//!
//! Run with: `cargo run -p example-actuator-damper --release`

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
use sim_bevy::examples::{
    PhysicsHud, ValidationHarness, render_physics_hud, spawn_example_camera, spawn_physics_hud,
    validation_system,
};
use sim_bevy::materials::MetalPreset;
use sim_bevy::model_data::{
    PhysicsAccumulator, PhysicsData, PhysicsModel, spawn_model_geoms, step_physics_realtime,
    sync_geom_transforms,
};
use sim_core::GainType;
use sim_core::validation::{Check, print_report};

// ── MJCF Model ──────────────────────────────────────────────────────────────

const MJCF: &str = r#"
<mujoco model="damper">
  <compiler angle="radian"/>
  <option gravity="0 0 0" timestep="0.001" integrator="RK4"/>

  <default>
    <geom contype="0" conaffinity="0"/>
  </default>

  <worldbody>
    <body name="arm" pos="0 0 0">
      <joint name="spin" type="hinge" axis="0 1 0" armature="0.01"/>
      <inertial pos="0 0 -0.25" mass="1.0" diaginertia="0.01 0.01 0.01"/>
      <geom name="rod" type="capsule" size="0.02"
            fromto="0 0 0  0 0 -0.5" rgba="0.48 0.48 0.50 1"/>
      <geom name="tip" type="sphere" size="0.05"
            pos="0 0 -0.5" rgba="0.85 0.3 0.2 1"/>
    </body>
  </worldbody>

  <actuator>
    <damper name="brake" joint="spin" kv="0.03" ctrlrange="0 1"/>
  </actuator>

  <sensor>
    <actuatorfrc name="act_force" actuator="brake"/>
    <jointvel name="jvel" joint="spin"/>
  </sensor>
</mujoco>
"#;

const CTRL_DAMPING: f64 = 1.0; // full damping

// I_eff = diaginertia_Y + m*d² + armature = 0.01 + 0.0625 + 0.01
const I_EFF: f64 = 0.0825;
const KV: f64 = 0.03;
const OMEGA_0: f64 = 10.0;
const TAU: f64 = I_EFF / KV; // 2.75 s

// ── Bevy App ────────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Damper Actuator ===");
    println!("  Viscous braking — velocity-dependent damping");
    println!("  ctrl = {CTRL_DAMPING} (full damping), kv = {KV}");
    println!("  I_eff = {I_EFF} kg·m², ω₀ = {OMEGA_0} rad/s, τ = {TAU:.2} s");
    println!("  Analytical: ω(t) = {OMEGA_0} · e^(-t/{TAU:.2})");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Damper Actuator".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .init_resource::<PhysicsHud>()
        .init_resource::<DamperValidation>()
        .insert_resource(
            ValidationHarness::new()
                .report_at(15.0)
                .print_every(1.0)
                .display(|m, d| {
                    let force = d.sensor_data(m, 0)[0];
                    let vel = d.sensor_data(m, 1)[0];
                    format!("force={force:.3}  ω={vel:.2}")
                }),
        )
        .add_systems(Startup, setup)
        .add_systems(Update, (apply_ctrl, step_physics_realtime).chain())
        .add_systems(
            PostUpdate,
            (
                sync_geom_transforms,
                validation_system,
                damper_diagnostics,
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

    // Set initial angular velocity: ω₀ = 10 rad/s
    let dof_adr = model.jnt_dof_adr[0];
    data.qvel[dof_adr] = OMEGA_0;
    let _ = data.forward(&model);

    println!(
        "  Model: {} bodies, {} joints, {} actuators, {} sensors",
        model.nbody, model.njnt, model.nu, model.nsensor
    );
    println!("  Initial ω = {} rad/s\n", data.qvel[dof_adr]);

    let mat_rod = materials.add(MetalPreset::BrushedMetal.material());
    let mat_tip = materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.85, 0.3, 0.2)));

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
        Vec3::new(0.0, -0.25, 0.0),  // arm midpoint in Bevy Y-up coords
        2.5,                         // distance — zoomed out for full rotation
        std::f32::consts::FRAC_PI_2, // azimuth: 90° — face the rotation plane
        0.0,                         // elevation: level
    );

    spawn_physics_hud(&mut commands);

    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── Control ─────────────────────────────────────────────────────────────────

fn apply_ctrl(mut data: ResMut<PhysicsData>) {
    if !data.ctrl.is_empty() {
        data.ctrl[0] = CTRL_DAMPING;
    }
}

// ── HUD ─────────────────────────────────────────────────────────────────────

fn update_hud(model: Res<PhysicsModel>, data: Res<PhysicsData>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();
    hud.section("Damper Actuator");

    let brake = data.sensor_data(&model, 0)[0];
    let vel = data.sensor_data(&model, 1)[0];
    let expected_vel = OMEGA_0 * (-data.time / TAU).exp();
    let decay_pct = (1.0 - vel / OMEGA_0) * 100.0;

    hud.scalar("kv (damping)", KV, 3);
    hud.scalar("omega (rad/s)", vel, 2);
    hud.scalar("expected (rad/s)", expected_vel, 2);
    hud.scalar("brake (Nm)", brake, 4);
    hud.scalar("decay (%)", decay_pct, 1);
    hud.scalar("time (s)", data.time, 2);
}

// ── Validation ──────────────────────────────────────────────────────────────

#[derive(Resource, Default)]
struct DamperValidation {
    /// Samples of (time, omega) for exponential decay check
    samples: Vec<(f64, f64)>,
    /// Samples of (force, omega) for proportionality check
    force_samples: Vec<(f64, f64)>,
    /// Closest sample to t = TAU
    tau_sample: Option<(f64, f64)>,
    reported: bool,
}

fn damper_diagnostics(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<DamperValidation>,
) {
    let time = data.time;
    let force_sensor = data.sensor_data(&model, 0)[0];
    let vel = data.sensor_data(&model, 1)[0];

    // Skip t=0 frame (actuator_force not yet computed before first step)
    if time < 1e-6 {
        return;
    }

    // Collect samples every ~50ms for the first 8s (covers ~3 time constants)
    if time <= 8.0 {
        let last_t = val.samples.last().map_or(0.0, |s| s.0);
        if time - last_t >= 0.049 {
            val.samples.push((time, vel));
            val.force_samples.push((force_sensor, vel));
        }
    }

    // Track the sample closest to t = TAU
    let dist_to_tau = (time - TAU).abs();
    let current_best = val.tau_sample.map_or(f64::MAX, |(t, _)| (t - TAU).abs());
    if dist_to_tau < current_best {
        val.tau_sample = Some((time, vel));
    }

    // Final report
    if harness.reported() && !val.reported {
        val.reported = true;

        // Check 1: Exponential decay — ω(τ) / ω₀ ≈ e⁻¹ ≈ 0.368
        let (tau_t, tau_omega) = val.tau_sample.unwrap_or((0.0, 0.0));
        let ratio = tau_omega / OMEGA_0;
        let expected_ratio = (-1.0_f64).exp(); // e⁻¹ ≈ 0.3679
        let decay_err_pct = ((ratio - expected_ratio) / expected_ratio).abs() * 100.0;

        // Check 2: Time constant — same measurement, phrased differently
        let tc_err_pct = decay_err_pct; // same data, same check

        // Check 3: Force proportional to velocity — force ≈ -kv * ω
        let mut max_prop_err_pct = 0.0_f64;
        for &(force, omega) in &val.force_samples {
            let expected_force = -KV * omega;
            if expected_force.abs() > 1e-6 {
                let err_pct = ((force - expected_force) / expected_force).abs() * 100.0;
                max_prop_err_pct = max_prop_err_pct.max(err_pct);
            }
        }

        // Check 4: Affine gain path (structural)
        let gaintype_ok = !model.actuator_gaintype.is_empty()
            && matches!(model.actuator_gaintype[0], GainType::Affine);

        let checks = vec![
            Check {
                name: "Exponential decay ω(τ)/ω₀ ≈ e⁻¹",
                pass: decay_err_pct < 5.0,
                detail: format!(
                    "at t={tau_t:.4}: ω/ω₀={ratio:.4} (expect {expected_ratio:.4}), err={decay_err_pct:.2}%"
                ),
            },
            Check {
                name: "Time constant τ = I/kv",
                pass: tc_err_pct < 5.0,
                detail: format!(
                    "ω drops to {:.1}% at t={tau_t:.4}s (expect 36.8% at t={TAU:.2}s)",
                    ratio * 100.0
                ),
            },
            Check {
                name: "Force ∝ velocity",
                pass: max_prop_err_pct < 5.0,
                detail: format!(
                    "max |force - (-kv·ω)| err={max_prop_err_pct:.2}% ({} samples)",
                    val.force_samples.len()
                ),
            },
            Check {
                name: "Affine gain path",
                pass: gaintype_ok,
                detail: format!(
                    "actuator_gaintype[0] = {:?}",
                    model.actuator_gaintype.first()
                ),
            },
        ];
        let _ = print_report("Damper Actuator (t=15s)", &checks);
    }
}
