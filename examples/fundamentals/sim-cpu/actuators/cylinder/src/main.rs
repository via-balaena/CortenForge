//! Cylinder Actuator — Double-Acting Pneumatic Piston
//!
//! A double-acting pneumatic cylinder on a horizontal rail. Positive pressure
//! extends the piston; vacuum (negative pressure) retracts it. The cylinder
//! shortcut maps to `GainType::Fixed(area)`, `BiasType::Affine(b0, b1, b2)`,
//! `ActuatorDynamics::Filter(tau)`. Force = `area * act + b0 + b1*length +
//! b2*velocity`. Filter dynamics is Euler-approximated (not FilterExact),
//! which is the key code-path difference from example 5.
//!
//! Pressure alternates between +1 and -1 every 3 seconds. The activation
//! filter delays the pressure response (tau=0.5s), and viscous damping
//! (bias[2]) smooths the motion. Joint limits model the end-of-stroke walls.
//!
//! Validates:
//! - Activation state exists (data.na == 1)
//! - Filter dynamics: act at t ~= tau is ~0.632 (within 5%)
//! - Force formula: force ~= area*act + b0 + b1*pos + b2*vel (within 2%)
//! - Area from diameter: model.actuator_gainprm[0][0] ~= pi/4 * d^2 (within 1e-10)
//!
//! Run with: `cargo run -p example-actuator-cylinder --release`

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
use sim_core::validation::{Check, print_report};

// ── MJCF Model ──────────────────────────────────────────────────────────────

const MJCF: &str = r#"
<mujoco model="cylinder">
  <compiler angle="radian"/>
  <option gravity="0 0 0" timestep="0.001" integrator="RK4"/>

  <default>
    <geom contype="0" conaffinity="0"/>
  </default>

  <worldbody>
    <!-- Moving piston: barrel + rod (housing spawned by Bevy as hollow tube) -->
    <body name="piston" pos="0 0 0">
      <joint name="slide" type="slide" axis="1 0 0" armature="0.01"
             range="-0.25 0.25" limited="true"/>
      <inertial pos="0 0 0" mass="1.0" diaginertia="0.001 0.001 0.001"/>
      <geom name="barrel" type="cylinder" size="0.06 0.08" euler="0 1.5708 0"
            rgba="0.5 0.7 0.3 1"/>
      <geom name="rod" type="cylinder" size="0.02 0.35" pos="0.43 0 0"
            euler="0 1.5708 0" rgba="0.7 0.7 0.7 1"/>
    </body>
  </worldbody>

  <actuator>
    <cylinder name="pneumatic" joint="slide" timeconst="0.5" diameter="0.5"
              bias="0 0 -0.5" ctrlrange="-1 1" ctrllimited="false"/>
  </actuator>

  <sensor>
    <actuatorfrc name="act_force" actuator="pneumatic"/>
    <jointpos name="jpos" joint="slide"/>
    <jointvel name="jvel" joint="slide"/>
  </sensor>
</mujoco>
"#;

// Area = pi/4 * diameter^2
const DIAMETER: f64 = 0.5;
const AREA: f64 = std::f64::consts::PI / 4.0 * DIAMETER * DIAMETER;
const TAU: f64 = 0.5;
const BIAS: [f64; 3] = [0.0, 0.0, -0.5]; // no spring, viscous damping only
const HALF_PERIOD: f64 = 3.0; // seconds per half-cycle

/// Read `data.act[0]`, returning 0.0 if the vector is empty.
fn read_act(data: &PhysicsData) -> f64 {
    if data.act.is_empty() {
        0.0
    } else {
        data.act[0]
    }
}

// ── Bevy App ────────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Cylinder Actuator ===");
    println!("  Double-acting pneumatic cylinder — pressure extends and retracts");
    println!(
        "  ctrl = +1/-1 square wave (period = {}s), tau = {TAU} s",
        2.0 * HALF_PERIOD
    );
    println!("  area = {AREA:.4} m^2 (d = {DIAMETER} m)");
    println!(
        "  bias = [{}, {}, {}] (no spring, viscous damping only)",
        BIAS[0], BIAS[1], BIAS[2]
    );
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge \u{2014} Cylinder Actuator".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .init_resource::<PhysicsHud>()
        .init_resource::<CylinderValidation>()
        .insert_resource(
            ValidationHarness::new()
                .report_at(15.0)
                .print_every(1.0)
                .display(|m, d| {
                    let force = d.sensor_data(m, 0)[0];
                    let pos = d.sensor_data(m, 1)[0];
                    let act = if d.act.is_empty() { 0.0 } else { d.act[0] };
                    format!("act={act:.3}  x={pos:.4}m  F={force:.4}")
                }),
        )
        .add_systems(Startup, setup)
        .add_systems(Update, (apply_ctrl, step_physics_realtime).chain())
        .add_systems(
            PostUpdate,
            (
                sync_geom_transforms,
                validation_system,
                cylinder_diagnostics,
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
        "  Model: {} bodies, {} joints, {} actuators, {} sensors, na={}",
        model.nbody, model.njnt, model.nu, model.nsensor, model.na
    );
    println!("  gainprm[0] = {:.10}\n", model.actuator_gainprm[0][0]);

    let mat_cap = materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.4, 0.4, 0.45)));
    let mat_housing = materials.add(StandardMaterial {
        base_color: Color::srgba(0.3, 0.3, 0.35, 0.5),
        alpha_mode: AlphaMode::Blend,
        double_sided: true,
        cull_mode: None,
        ..default()
    });
    let mat_barrel =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.5, 0.7, 0.3)));
    let mat_rod = materials.add(MetalPreset::BrushedMetal.material());

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[("barrel", mat_barrel), ("rod", mat_rod)],
    );

    // Rotation to align Bevy Z-axis extrusions with the X-axis (slide direction)
    let align_x = Quat::from_rotation_y(std::f32::consts::FRAC_PI_2);

    // ── Hollow cylinder housing ─────────────────────────────────────────
    commands.spawn((
        Mesh3d(meshes.add(Extrusion::new(Annulus::new(0.06, 0.08), 0.70))),
        MeshMaterial3d(mat_housing),
        Transform::from_xyz(0.0, 0.0, 0.0).with_rotation(align_x),
    ));

    // ── Left end cap (solid disc — seals the bore) ──────────────────────
    commands.spawn((
        Mesh3d(meshes.add(Extrusion::new(Circle::new(0.06), 0.005))),
        MeshMaterial3d(mat_cap.clone()),
        Transform::from_xyz(-0.35, 0.0, 0.0).with_rotation(align_x),
    ));

    // ── Right end cap (annulus — rod passes through seal) ───────────────
    commands.spawn((
        Mesh3d(meshes.add(Extrusion::new(Annulus::new(0.02, 0.06), 0.005))),
        MeshMaterial3d(mat_cap),
        Transform::from_xyz(0.35, 0.0, 0.0).with_rotation(align_x),
    ));

    spawn_example_camera(
        &mut commands,
        Vec3::new(0.15, 0.0, 0.0),   // center on assembly
        1.2,                         // distance
        std::f32::consts::FRAC_PI_2, // azimuth: side view
        0.15,                        // slight elevation for depth
    );

    spawn_physics_hud(&mut commands);

    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── Control ─────────────────────────────────────────────────────────────────

/// Square wave: +1 for 3s (extend), -1 for 3s (retract), repeat.
fn current_pressure(time: f64) -> f64 {
    let cycle = time % (2.0 * HALF_PERIOD);
    if cycle < HALF_PERIOD { 1.0 } else { -1.0 }
}

fn apply_ctrl(mut data: ResMut<PhysicsData>) {
    if !data.ctrl.is_empty() {
        data.ctrl[0] = current_pressure(data.time);
    }
}

// ── HUD ─────────────────────────────────────────────────────────────────────

fn update_hud(model: Res<PhysicsModel>, data: Res<PhysicsData>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();
    hud.section("Cylinder Actuator");

    let force = data.sensor_data(&model, 0)[0];
    let pos = data.sensor_data(&model, 1)[0];
    let act = read_act(&data);

    let pressure = current_pressure(data.time);
    hud.scalar("pressure (command)", pressure, 1);
    hud.scalar("activation (buildup)", act, 3);
    hud.scalar("position (m)", pos, 4);
    hud.scalar("force (N)", force, 4);
    hud.scalar("time (s)", data.time, 2);
}

// ── Validation ──────────────────────────────────────────────────────────────

#[derive(Resource, Default)]
struct CylinderValidation {
    tau_sample: Option<(f64, f64)>,
    force_samples: Vec<(f64, f64, f64, f64, f64)>,
    reported: bool,
}

fn cylinder_diagnostics(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<CylinderValidation>,
) {
    let time = data.time;
    let force_sensor = data.sensor_data(&model, 0)[0];
    let pos = data.sensor_data(&model, 1)[0];
    let vel = data.sensor_data(&model, 2)[0];
    let act = read_act(&data);

    if time < 1e-6 {
        return;
    }

    let dist_to_tau = (time - TAU).abs();
    let current_best = val.tau_sample.map_or(f64::MAX, |(t, _)| (t - TAU).abs());
    if dist_to_tau < current_best {
        val.tau_sample = Some((time, act));
    }

    if time <= 8.0 {
        let last_t = val.force_samples.last().map_or(0.0, |s| s.0);
        if time - last_t >= 0.049 {
            val.force_samples.push((time, force_sensor, act, pos, vel));
        }
    }

    if harness.reported() && !val.reported {
        val.reported = true;

        let na = model.na;
        let na_ok = na == 1;

        let (tau_t, tau_act) = val.tau_sample.unwrap_or((0.0, 0.0));
        let expected_act = 1.0 - (-1.0_f64).exp();
        let filter_err_pct = ((tau_act - expected_act) / expected_act).abs() * 100.0;

        let mut max_force_err_pct = 0.0_f64;
        let mut worst_detail = String::new();
        for &(_t, force, a, p, v) in &val.force_samples {
            let length = p;
            let expected_force = AREA * a + BIAS[0] + BIAS[1] * length + BIAS[2] * v;
            if expected_force.abs() > 1e-10 {
                let err_pct = ((force - expected_force) / expected_force).abs() * 100.0;
                if err_pct > max_force_err_pct {
                    max_force_err_pct = err_pct;
                    worst_detail = format!(
                        "f={force:.6} vs expected={expected_force:.6} (a={a:.4}, p={p:.6}, v={v:.6})"
                    );
                }
            }
        }

        let gainprm_area = model.actuator_gainprm[0][0];
        let area_err = (gainprm_area - AREA).abs();

        let checks = vec![
            Check {
                name: "Activation exists (na==1)",
                pass: na_ok,
                detail: format!("data.na = {na}"),
            },
            Check {
                name: "Filter dynamics act(tau) ~= 0.632",
                pass: filter_err_pct < 5.0,
                detail: format!(
                    "at t={tau_t:.4}: act={tau_act:.4} (expect {expected_act:.4}), err={filter_err_pct:.2}%"
                ),
            },
            Check {
                name: "Force formula (area*act + bias)",
                pass: max_force_err_pct < 2.0,
                detail: format!(
                    "max err={max_force_err_pct:.4}% ({} samples) worst: {worst_detail}",
                    val.force_samples.len()
                ),
            },
            Check {
                name: "Area from diameter",
                pass: area_err < 1e-10,
                detail: format!(
                    "gainprm[0]={gainprm_area:.12}, expected={AREA:.12}, err={area_err:.2e}"
                ),
            },
        ];
        let _ = print_report("Cylinder Actuator (t=15s)", &checks);
    }
}
