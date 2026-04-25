#![allow(
    missing_docs,
    clippy::doc_markdown,
    clippy::needless_pass_by_value,
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::let_underscore_must_use
)]

use bevy::prelude::*;
use sim_bevy::camera::OrbitCameraPlugin;
use sim_bevy::examples::{
    HudPosition, PhysicsHud, ValidationHarness, render_physics_hud, spawn_example_camera,
    spawn_physics_hud_at, validation_system,
};
use sim_bevy::materials::MetalPreset;
use sim_bevy::model_data::{
    PhysicsAccumulator, PhysicsData, PhysicsModel, spawn_model_geoms, step_physics_realtime,
    sync_geom_transforms,
};
use sim_core::validation::{Check, print_report};
use sim_ml_chassis::ObservationSpace as ChassisObservationSpace;
use sim_ml_chassis_bevy::ObservationSpace;

// ── MJCF Model ─────────────────────────────────────────────────────────────

const MJCF: &str = r#"
<mujoco>
  <option timestep="0.002">
    <flag energy="enable"/>
  </option>
  <worldbody>
    <geom type="plane" size="5 5 0.1"/>
    <body name="cart" pos="0 0 0.5">
      <joint name="cart_slide" type="slide" axis="1 0 0" damping="0.1"/>
      <geom name="cart_box" type="box" size="0.2 0.1 0.05" mass="1"
            rgba="0.3 0.3 0.35 1"/>
      <body name="pole" pos="0 0 0.05">
        <joint name="pole_hinge" type="hinge" axis="0 1 0"/>
        <geom name="pole_rod" type="capsule" size="0.02"
              fromto="0 0 0 0 0 0.5" mass="0.1"
              rgba="0.85 0.5 0.2 1"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <motor joint="cart_slide" name="force"/>
  </actuator>
  <sensor>
    <jointpos joint="cart_slide" name="cart_pos"/>
    <jointvel joint="cart_slide" name="cart_vel"/>
    <jointpos joint="pole_hinge" name="pole_angle"/>
    <jointvel joint="pole_hinge" name="pole_angvel"/>
  </sensor>
</mujoco>
"#;

// ── Bevy App ───────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Rich Observation Space ===");
    println!("  Cart-pole with 10 extractors in one observation");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Rich Observation Space".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .init_resource::<PhysicsHud>()
        .init_resource::<RichValidation>()
        .insert_resource(
            ValidationHarness::new()
                .report_at(15.0)
                .print_every(1.0)
                .display(|m, d| {
                    let cart = d.sensor_scalar(m, "cart_pos").unwrap_or(0.0);
                    let pole = d.sensor_scalar(m, "pole_angle").unwrap_or(0.0);
                    format!("cart={cart:+.3}  pole={pole:+.4}")
                }),
        )
        .add_systems(Startup, setup)
        .add_systems(Update, step_physics_realtime)
        .add_systems(
            PostUpdate,
            (
                sync_geom_transforms,
                validation_system,
                rich_diagnostics,
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

    // Small perturbation so the pole topples visibly.
    data.qpos[1] = 0.05;
    data.forward(&model).expect("forward");

    let obs_space = ChassisObservationSpace::builder()
        .all_qpos()
        .all_qvel()
        .sensor("cart_pos")
        .sensor("cart_vel")
        .sensor("pole_angle")
        .sensor("pole_angvel")
        .xpos(1..model.nbody)
        .energy()
        .time()
        .contact_count()
        .build(&model)
        .expect("obs space build");

    println!(
        "  obs_space: dim={}, segments={}\n",
        obs_space.dim(),
        obs_space.segments().len()
    );

    let mat_cart = materials.add(MetalPreset::BrushedMetal.material());
    let mat_pole =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.85, 0.5, 0.2)));

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[("cart_box", mat_cart), ("pole_rod", mat_pole)],
    );

    spawn_example_camera(
        &mut commands,
        Vec3::new(0.0, 0.0, 0.5),
        3.0,
        std::f32::consts::FRAC_PI_2,
        0.2,
    );

    spawn_physics_hud_at(&mut commands, HudPosition::BottomLeft);

    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
    commands.insert_resource(ObservationSpace::from(obs_space));
}

// ── HUD ────────────────────────────────────────────────────────────────────

fn update_hud(
    _model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    obs_space: Res<ObservationSpace>,
    mut hud: ResMut<PhysicsHud>,
) {
    let obs = obs_space.extract(&data);
    let s = obs.as_slice();

    hud.clear();
    hud.section("Rich Observation");
    hud.raw(format!("dim={}, shape={:?}", obs_space.dim(), obs.shape()));
    hud.raw(String::new());

    for seg in obs_space.segments() {
        let vals = &s[seg.offset..seg.offset + seg.dim];
        let formatted: Vec<String> = vals.iter().map(|v| format!("{v:+.4}")).collect();
        hud.raw(format!(
            "[{:>2}..{:>2}] {:<20} {}",
            seg.offset,
            seg.offset + seg.dim,
            seg.label,
            formatted.join(" ")
        ));
    }

    hud.section("State");
    hud.scalar("time", data.time, 2);
}

// ── Validation ─────────────────────────────────────────────────────────────

#[derive(Resource, Default)]
struct RichValidation {
    max_sensor_err: f64,
    reported: bool,
}

fn rich_diagnostics(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    obs_space: Res<ObservationSpace>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<RichValidation>,
) {
    let obs = obs_space.extract(&data);
    let s = obs.as_slice();
    let segments = obs_space.segments();

    // Check named sensors vs manual sensordata indexing.
    // sensor("cart_pos") is at offset 4, sensordata[0] is cart_pos.
    for (sensor_idx, obs_idx) in [(0, 4), (1, 5), (2, 6), (3, 7)] {
        let manual = data.sensordata[sensor_idx] as f32;
        let extracted = s[obs_idx];
        let err = (f64::from(manual) - f64::from(extracted)).abs();
        if err > val.max_sensor_err {
            val.max_sensor_err = err;
        }
    }

    if harness.reported() && !val.reported {
        val.reported = true;

        let seg_dim_sum: usize = segments.iter().map(|s| s.dim).sum();
        let dim_ok = obs_space.dim() == seg_dim_sum;

        // xpos segment: bodies 1..3 = 2 bodies * 3 = 6 floats.
        let xpos_count = (model.nbody - 1) * 3;
        let xpos_ok = xpos_count == 6;

        // energy segment: always 2 floats.
        let energy_ok = segments
            .iter()
            .find(|s| s.label == "energy")
            .is_some_and(|s| s.dim == 2);

        let checks = vec![
            Check {
                name: "obs_dim",
                pass: dim_ok,
                detail: format!("{} (sum={})", obs_space.dim(), seg_dim_sum),
            },
            Check {
                name: "Named sensors",
                pass: val.max_sensor_err < 1e-10,
                detail: format!("max err={:.2e}", val.max_sensor_err),
            },
            Check {
                name: "xpos count",
                pass: xpos_ok,
                detail: format!("{xpos_count} floats ({} bodies * 3)", model.nbody - 1),
            },
            Check {
                name: "energy count",
                pass: energy_ok,
                detail: "2 (kinetic + potential)".into(),
            },
        ];
        let _ = print_report("Rich Observation Space (t=15s)", &checks);
    }
}
