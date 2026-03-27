//! Slider-Crank — Mechanical Linkage Transmission
//!
//! A slider-crank mechanism that converts rotation into reciprocating linear
//! motion. A motor spins the crank, a servo drives the piston along the
//! kinematic curve, and the slider-crank transmission computes the varying
//! moment arm that would exist if a physical connecting rod linked them.
//!
//! The key insight: the moment arm varies with crank angle. At dead centers
//! (crank aligned with slider axis) the moment arm drops to zero — no torque
//! transmitted. At 90 deg (crank perpendicular) the moment arm peaks. Watch
//! the crank speed wobble as the transmission torque assists and resists the
//! motor each half-revolution.
//!
//! Validates:
//! - Actuator length varies over a full revolution
//! - Moment arm varies (zero at dead center, peak near 90 deg)
//! - Dead center reached (moment arm < 0.02)
//! - Transmission active (peak moment arm > 0.05)
//!
//! Run with: `cargo run -p example-actuator-slider-crank --release`

#![allow(
    clippy::doc_markdown,
    clippy::needless_pass_by_value,
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
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

/// Marker for the visual connecting rod entity.
#[derive(Component)]
struct ConnectingRod;

// ── MJCF Model ──────────────────────────────────────────────────────────────

const MJCF: &str = r#"
<mujoco model="slider-crank">
  <compiler angle="radian"/>
  <option gravity="0 0 0" timestep="0.001" integrator="RK4"/>

  <default>
    <geom contype="0" conaffinity="0"/>
  </default>

  <worldbody>
    <!-- Crank: rotates about Y. High armature stores flywheel energy. -->
    <body name="crank" pos="0 0 0">
      <joint name="crank_joint" type="hinge" axis="0 1 0"
             armature="0.05" damping="0.3"/>
      <inertial pos="0.05 0 0" mass="1.0" diaginertia="0.003 0.003 0.003"/>
      <geom name="crank_arm" type="capsule" size="0.015"
            fromto="0 0 0  0.1 0 0" rgba="0.48 0.48 0.50 1"/>
      <site name="crank_pin" pos="0.1 0 0"/>
    </body>

    <!-- Piston: translates along X, driven by servo to follow crank kinematics.
         Long piston rod keeps the wrist pin outside the cylinder housing. -->
    <body name="slider_body" pos="0.50 0 0">
      <joint name="slider_joint" type="slide" axis="1 0 0"
             armature="0.01" damping="0.5"/>
      <inertial pos="0 0 0" mass="1.0" diaginertia="0.001 0.001 0.001"/>
      <geom name="piston_rod" type="capsule" size="0.008"
            fromto="-0.25 0 0  0 0 0" rgba="0.48 0.48 0.50 1"/>
      <geom name="piston_head" type="cylinder" size="0.025"
            fromto="-0.015 0 0  0.015 0 0" rgba="0.7 0.5 0.2 1"/>
      <!-- Wrist pin at rod end — connecting rod attaches here, outside housing -->
      <site name="slider_pin" pos="-0.25 0 0" euler="0 1.5708 0"/>
    </body>
  </worldbody>

  <actuator>
    <!-- Motor keeps the crank spinning at steady speed -->
    <motor name="spin_motor" joint="crank_joint" gear="1"/>
    <!-- Servo drives piston along kinematic curve (simulates rod constraint) -->
    <position name="slider_servo" joint="slider_joint" kp="200" dampratio="1"/>
    <!-- Slider-crank transmission: computes moment arm + applies linkage force -->
    <general name="linkage" cranksite="crank_pin" slidersite="slider_pin"
             cranklength="0.2" gainprm="3" gear="1"/>
  </actuator>

  <sensor>
    <actuatorfrc name="linkage_force" actuator="linkage"/>
    <jointpos name="crank_angle" joint="crank_joint"/>
    <jointpos name="slider_pos" joint="slider_joint"/>
  </sensor>
</mujoco>
"#;

/// Motor torque to spin the crank (Nm).
const MOTOR_TORQUE: f64 = 2.0;

/// Linkage drive signal.
const LINKAGE_CTRL: f64 = 1.0;

/// Linkage gain (N per unit ctrl).
const LINKAGE_GAIN: f64 = 3.0;

/// Crank radius (m).
const CRANK_R: f64 = 0.1;

/// Connecting rod length (m).
const ROD_L: f64 = 0.2;

/// Slider body rest position along X (m).
const SLIDER_X0: f64 = 0.50;

/// Linkage actuator index (spin_motor=0, slider_servo=1, linkage=2).
const LINKAGE_IDX: usize = 2;

// ── Bevy App ────────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Slider-Crank ===");
    println!("  Crank (r={CRANK_R} m) + rod (L={ROD_L} m) + piston");
    println!("  Motor = {MOTOR_TORQUE} Nm, linkage = {LINKAGE_GAIN} N");
    println!("  Moment arm varies: zero at dead center, peak at 90 deg");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge \u{2014} Slider-Crank".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .init_resource::<PhysicsHud>()
        .init_resource::<SliderCrankValidation>()
        .insert_resource(
            ValidationHarness::new()
                .report_at(17.0)
                .print_every(1.0)
                .display(|m, d| {
                    let crank_angle = d.sensor_data(m, 1)[0];
                    let slider_pos = d.sensor_data(m, 2)[0];
                    let moment = if d.actuator_moment.len() > 2
                        && !d.actuator_moment[2].is_empty()
                    {
                        d.actuator_moment[2][0]
                    } else {
                        0.0
                    };
                    let crank_deg = ((crank_angle.to_degrees() % 360.0) + 360.0) % 360.0;
                    format!(
                        "crank={crank_deg:5.1} deg  slider={slider_pos:+.4} m  arm={moment:.4}",
                    )
                }),
        )
        .add_systems(Startup, setup)
        .add_systems(Update, (apply_ctrl, step_physics_realtime).chain())
        .add_systems(
            PostUpdate,
            (
                sync_geom_transforms,
                update_connecting_rod,
                validation_system,
                slider_crank_diagnostics,
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
        "  Model: {} bodies, {} joints, {} actuators, {} sensors\n",
        model.nbody, model.njnt, model.nu, model.nsensor
    );

    let mat_crank = materials.add(MetalPreset::BrushedMetal.material());
    let mat_piston =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.7, 0.5, 0.2)));

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[
            ("crank_arm", mat_crank.clone()),
            ("piston_rod", mat_crank),
            ("piston_head", mat_piston),
        ],
    );

    // Connecting rod: visual-only cylinder stretching between crank pin and piston
    let rod_mat = materials.add(MetalPreset::BrushedMetal.material());
    commands.spawn((
        Mesh3d(meshes.add(Cylinder::new(0.01, 1.0))),
        MeshMaterial3d(rod_mat),
        Transform::default(),
        ConnectingRod,
    ));

    // Cylinder housing: translucent shell the piston slides through.
    // Piston head travels roughly x=[0.35..0.55] in MuJoCo = Bevy X.
    let housing_mat = materials.add(StandardMaterial {
        base_color: Color::srgba(0.3, 0.3, 0.35, 0.3),
        alpha_mode: AlphaMode::Blend,
        ..default()
    });
    commands.spawn((
        Mesh3d(meshes.add(Cylinder::new(0.028, 0.20))),
        MeshMaterial3d(housing_mat),
        Transform {
            translation: Vec3::new(0.43, 0.0, 0.0),
            rotation: Quat::from_rotation_z(-std::f32::consts::FRAC_PI_2),
            ..default()
        },
    ));

    // Multi-body mechanism: angled view, zoomed out to see full mechanism
    spawn_example_camera(
        &mut commands,
        Vec3::new(0.25, 0.0, 0.0),
        1.3,
        std::f32::consts::FRAC_PI_3,
        0.2,
    );

    spawn_physics_hud(&mut commands);

    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── Control ─────────────────────────────────────────────────────────────────

/// Pin offset from slider body origin along X (m).
const PIN_OFFSET: f64 = -0.25;

/// Kinematic slider position for a slider-crank mechanism.
/// The pin is at body-local (PIN_OFFSET, 0, 0), so its world X = SLIDER_X0 + slider_q + PIN_OFFSET.
/// Solving: slider_q = sqrt(L^2 - r^2*sin^2(theta)) + r*cos(theta) - SLIDER_X0 - PIN_OFFSET
fn kinematic_slider_pos(theta: f64) -> f64 {
    let sin_t = theta.sin();
    let cos_t = theta.cos();
    (ROD_L * ROD_L - CRANK_R * CRANK_R * sin_t * sin_t).sqrt() + CRANK_R * cos_t
        - SLIDER_X0
        - PIN_OFFSET
}

fn apply_ctrl(mut data: ResMut<PhysicsData>) {
    if data.ctrl.len() < 3 {
        return;
    }
    // Actuator 0: motor torque to spin the crank
    data.ctrl[0] = MOTOR_TORQUE;

    // Actuator 1: servo drives piston along kinematic curve
    let theta = if data.qpos.is_empty() {
        0.0
    } else {
        data.qpos[0]
    };
    data.ctrl[1] = kinematic_slider_pos(theta);

    // Actuator 2: linkage force (creates speed wobble on crank)
    data.ctrl[2] = LINKAGE_CTRL;
}

// ── HUD ─────────────────────────────────────────────────────────────────────

fn update_hud(model: Res<PhysicsModel>, data: Res<PhysicsData>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();
    hud.section("Slider-Crank");

    let crank_angle = data.sensor_data(&model, 1)[0];
    let slider_pos = data.sensor_data(&model, 2)[0];
    let act_length = if data.actuator_length.len() > LINKAGE_IDX {
        data.actuator_length[LINKAGE_IDX]
    } else {
        0.0
    };
    let moment_crank = if data.actuator_moment.len() > LINKAGE_IDX
        && !data.actuator_moment[LINKAGE_IDX].is_empty()
    {
        data.actuator_moment[LINKAGE_IDX][0]
    } else {
        0.0
    };
    let crank_speed = if data.qvel.is_empty() {
        0.0
    } else {
        data.qvel[0]
    };

    let crank_deg = ((crank_angle.to_degrees() % 360.0) + 360.0) % 360.0;

    hud.scalar("motor torque (Nm)", MOTOR_TORQUE, 1);
    hud.scalar("linkage force (N)", LINKAGE_GAIN, 1);
    hud.scalar("crank angle (deg)", crank_deg, 1);
    hud.scalar("crank speed (rad/s)", crank_speed, 2);
    hud.scalar("moment arm (varies)", moment_crank, 4);
    hud.scalar("slider position (m)", slider_pos, 4);
    hud.scalar("act. length (m)", act_length, 4);
    hud.scalar("time (s)", data.time, 2);
}

// ── Connecting Rod Visual ────────────────────────────────────────────────────

fn update_connecting_rod(
    data: Res<PhysicsData>,
    mut query: Query<&mut Transform, With<ConnectingRod>>,
) {
    if data.site_xpos.len() < 2 {
        return;
    }
    // Site 0 = crank_pin, Site 1 = slider_pin (MuJoCo Z-up → Bevy Y-up)
    let cp = &data.site_xpos[0];
    let sp = &data.site_xpos[1];
    let crank = Vec3::new(cp[0] as f32, cp[2] as f32, cp[1] as f32);
    let slider = Vec3::new(sp[0] as f32, sp[2] as f32, sp[1] as f32);

    let midpoint = (crank + slider) / 2.0;
    let delta = slider - crank;
    let length = delta.length();
    if length < 1e-6 {
        return;
    }
    let direction = delta / length;

    for mut transform in &mut query {
        transform.translation = midpoint;
        transform.rotation = Quat::from_rotation_arc(Vec3::Y, direction);
        transform.scale = Vec3::new(1.0, length, 1.0);
    }
}

// ── Validation ──────────────────────────────────────────────────────────────

#[derive(Resource, Default)]
struct SliderCrankValidation {
    /// Samples of (crank_angle, slider_q, actuator_length).
    length_samples: Vec<(f64, f64, f64)>,
    /// Moment arm samples: (crank_angle, |moment_crank|).
    moment_samples: Vec<(f64, f64)>,
    reported: bool,
}

fn slider_crank_diagnostics(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<SliderCrankValidation>,
) {
    let time = data.time;
    let crank_angle = data.sensor_data(&model, 1)[0];
    let slider_q = data.sensor_data(&model, 2)[0];
    let act_length = if data.actuator_length.len() > LINKAGE_IDX {
        data.actuator_length[LINKAGE_IDX]
    } else {
        0.0
    };
    let moment_crank = if data.actuator_moment.len() > LINKAGE_IDX
        && !data.actuator_moment[LINKAGE_IDX].is_empty()
    {
        data.actuator_moment[LINKAGE_IDX][0]
    } else {
        0.0
    };

    if time < 3.0 {
        return;
    }

    // Collect samples every ~10ms
    let sample_interval = 0.009;
    let should_sample = val.length_samples.is_empty()
        || val.length_samples.len() < (((time - 3.0) / sample_interval) as usize);

    if should_sample {
        val.length_samples.push((crank_angle, slider_q, act_length));
        val.moment_samples.push((crank_angle, moment_crank.abs()));
    }

    // Final report
    if harness.reported() && !val.reported {
        val.reported = true;
        print_validation_report(&val);
    }
}

fn print_validation_report(val: &SliderCrankValidation) {
    // Check 1: actuator length varies over the revolution
    let mut min_len = f64::MAX;
    let mut max_len = f64::MIN;
    for &(_theta, _sq, measured_len) in &val.length_samples {
        min_len = min_len.min(measured_len);
        max_len = max_len.max(measured_len);
    }
    let length_range = max_len - min_len;
    let length_varies = length_range > 0.01;

    // Check 2: moment arm varies
    let mut min_moment = f64::MAX;
    let mut max_moment = 0.0_f64;
    for &(_theta, m) in &val.moment_samples {
        min_moment = min_moment.min(m);
        max_moment = max_moment.max(m);
    }
    let moment_range = max_moment - min_moment;
    let variable_moment = moment_range > 0.01;

    // Check 3: dead center reached (moment arm near zero)
    let near_zero_moment = min_moment < 0.02;

    // Check 4: transmission active (peak moment arm non-trivial)
    let transmission_active = max_moment > 0.05 && !val.length_samples.is_empty();

    let checks = vec![
        Check {
            name: "Length varies",
            pass: length_varies,
            detail: format!(
                "range={length_range:.4} (min={min_len:.4}, max={max_len:.4}, {} samples)",
                val.length_samples.len()
            ),
        },
        Check {
            name: "Variable moment arm",
            pass: variable_moment,
            detail: format!("min={min_moment:.4}, max={max_moment:.4}, range={moment_range:.4}"),
        },
        Check {
            name: "Dead center reached",
            pass: near_zero_moment,
            detail: format!("min |moment|={min_moment:.4} (threshold 0.02)"),
        },
        Check {
            name: "Transmission active",
            pass: transmission_active,
            detail: format!(
                "max moment={max_moment:.4}, {} samples",
                val.length_samples.len()
            ),
        },
    ];
    let _ = print_report("Slider-Crank (t=17s)", &checks);
}
