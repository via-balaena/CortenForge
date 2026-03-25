//! Cone Limit — Limited Ball Joint
//!
//! A pendulum on a ball joint with a 45° cone limit. Displaced 60° from
//! vertical (beyond the limit) and released — the constraint pushes it back
//! inside the cone, where it oscillates.
//!
//! Demonstrates: `limited="true"`, `range="0 0.7854"` (45° cone),
//! constraint solver enforcement, `solref`/`solimp` tuning.
//!
//! Validates:
//! - Cone limit enforced: rotation angle never exceeds 45° + solver margin
//! - Energy monotonically decreasing (damped system, limit dissipates)
//! - Quaternion norm preserved to machine precision
//!
//! Run with: `cargo run -p example-ball-joint-cone-limit --release`

#![allow(
    clippy::needless_pass_by_value,
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::let_underscore_must_use,
    clippy::suboptimal_flops
)]

use bevy::prelude::*;
use sim_bevy::camera::{OrbitCamera, OrbitCameraPlugin};
use sim_bevy::materials::{MetalPreset, override_geom_materials_by_name};
use sim_bevy::model_data::{
    ModelGeomIndex, PhysicsAccumulator, PhysicsData, PhysicsModel, spawn_model_geoms,
    step_physics_realtime, sync_geom_transforms,
};
use sim_core::ENABLE_ENERGY;
use sim_core::validation::{
    EnergyMonotonicityTracker, LimitTracker, QuaternionNormTracker, print_report,
};

// ── MJCF Model ──────────────────────────────────────────────────────────────
//
// Single ball-joint pendulum with a 45° cone limit.
// Light damping to stabilize the limit constraint and show energy dissipation.
//
const MJCF: &str = r#"
<mujoco model="cone_limit">
    <compiler angle="radian"/>
    <option gravity="0 0 -9.81" timestep="0.001" integrator="RK4"/>

    <default>
        <geom contype="0" conaffinity="0"/>
    </default>

    <worldbody>
        <!-- Support frame -->
        <geom name="beam" type="capsule" size="0.022"
              fromto="-0.3 0 0  0.3 0 0" rgba="0.40 0.40 0.43 1"/>
        <geom name="post_l" type="capsule" size="0.022"
              fromto="-0.3 0 0  -0.3 0 0.35" rgba="0.40 0.40 0.43 1"/>
        <geom name="post_r" type="capsule" size="0.022"
              fromto="0.3 0 0  0.3 0 0.35" rgba="0.40 0.40 0.43 1"/>

        <!-- Pendulum: limited ball joint (45° cone), light damping -->
        <body name="pendulum" pos="0 0 0">
            <joint name="ball" type="ball" damping="0.01"
                   limited="true" range="0 0.7854"/>
            <inertial pos="0 0 -0.5" mass="1.0" diaginertia="0.01 0.01 0.01"/>
            <geom name="socket" type="sphere" size="0.035"
                  pos="0 0 0" rgba="0.35 0.33 0.32 1"/>
            <geom name="rod" type="capsule" size="0.02"
                  fromto="0 0 0  0 0 -0.5" rgba="0.48 0.48 0.50 1"/>
            <geom name="tip" type="sphere" size="0.07"
                  pos="0 0 -0.5" rgba="0.82 0.22 0.15 1"/>
        </body>
    </worldbody>
</mujoco>
"#;

// ── Physics constants ───────────────────────────────────────────────────────

const CONE_LIMIT_RAD: f64 = std::f64::consts::FRAC_PI_4; // 45°

/// Compute axis-angle rotation angle from a quaternion [w, x, y, z].
/// Returns the angle in [0, pi].
fn quat_angle(w: f64, x: f64, y: f64, z: f64) -> f64 {
    let sin_half = (x * x + y * y + z * z).sqrt();
    2.0 * sin_half.atan2(w.abs())
}

// ── Bevy App ────────────────────────────────────────────────────────────────

fn main() {
    let limit_deg = CONE_LIMIT_RAD.to_degrees();
    println!("=== CortenForge: Cone Limit ===");
    println!("  Ball joint with {limit_deg:.0}° cone limit, light damping");
    println!("  Initial tilt: 60° (beyond limit — constraint pushes back)");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Cone Limit".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .init_resource::<DiagTimer>()
        .init_resource::<Validation>()
        .add_systems(Startup, setup)
        .add_systems(Update, (apply_materials, step_physics_realtime).chain())
        .add_systems(PostUpdate, (sync_geom_transforms, diagnostics))
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let mut model = sim_mjcf::load_model(MJCF).expect("MJCF should parse");
    model.enableflags |= ENABLE_ENERGY;
    let mut data = model.make_data();

    // ── Initial displacement: 60° about X axis (beyond 45° cone limit) ──
    let angle = 60.0_f64.to_radians();
    let half = angle / 2.0;
    let qpos_adr = model.jnt_qpos_adr[0];
    data.qpos[qpos_adr] = half.cos();
    data.qpos[qpos_adr + 1] = half.sin(); // pure X rotation
    data.qpos[qpos_adr + 2] = 0.0;
    data.qpos[qpos_adr + 3] = 0.0;

    let _ = data.forward(&model);

    println!(
        "  Model: {} bodies, {} joints, {} DOFs\n",
        model.nbody, model.njnt, model.nv
    );

    // ── Spawn MJCF geometry ─────────────────────────────────────────────
    spawn_model_geoms(&mut commands, &mut meshes, &mut materials, &model, &data);

    // ── Metallic materials ──────────────────────────────────────────────
    let mat_frame = materials.add(MetalPreset::BrushedMetal.material());
    let mat_socket = materials.add(MetalPreset::CastIron.material());
    let mat_rod = materials.add(MetalPreset::BrushedMetal.material());
    let mat_tip =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.82, 0.22, 0.15)));

    // ── Camera + lights ─────────────────────────────────────────────────
    let mut orbit = OrbitCamera::new()
        .with_target(Vec3::new(0.0, -0.2, 0.0))
        .with_angles(std::f32::consts::FRAC_PI_4, 0.35);
    orbit.max_distance = 20.0;
    orbit.distance = 1.8;
    let mut cam_transform = Transform::default();
    orbit.apply_to_transform(&mut cam_transform);
    commands.spawn((Camera3d::default(), orbit, cam_transform));

    commands.insert_resource(GlobalAmbientLight {
        color: Color::WHITE,
        brightness: 800.0,
        ..default()
    });
    commands.spawn((
        DirectionalLight {
            illuminance: 15_000.0,
            shadows_enabled: true,
            ..default()
        },
        Transform::from_xyz(30.0, 50.0, 50.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
    commands.spawn((
        DirectionalLight {
            illuminance: 5_000.0,
            shadows_enabled: false,
            ..default()
        },
        Transform::from_xyz(-20.0, 30.0, -30.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
    commands.insert_resource(MaterialOverrides {
        frame: mat_frame,
        socket: mat_socket,
        rod: mat_rod,
        tip: mat_tip,
    });
}

// ── Material override (runs once) ───────────────────────────────────────────

#[derive(Resource)]
struct MaterialOverrides {
    frame: Handle<StandardMaterial>,
    socket: Handle<StandardMaterial>,
    rod: Handle<StandardMaterial>,
    tip: Handle<StandardMaterial>,
}

fn apply_materials(
    mut commands: Commands,
    model: Option<Res<PhysicsModel>>,
    overrides: Option<Res<MaterialOverrides>>,
    mut query: Query<(&ModelGeomIndex, &mut MeshMaterial3d<StandardMaterial>)>,
) {
    let (Some(model), Some(ovr)) = (model, overrides) else {
        return;
    };

    override_geom_materials_by_name(
        &model,
        &[
            ("beam", ovr.frame.clone()),
            ("post_l", ovr.frame.clone()),
            ("post_r", ovr.frame.clone()),
            ("socket", ovr.socket.clone()),
            ("rod", ovr.rod.clone()),
            ("tip", ovr.tip.clone()),
        ],
        &mut query,
    );

    commands.remove_resource::<MaterialOverrides>();
}

// ── Diagnostics & Validation ────────────────────────────────────────────────

#[derive(Resource, Default)]
struct DiagTimer {
    last: f64,
}

#[derive(Resource)]
struct Validation {
    quat_norm: QuaternionNormTracker,
    energy: EnergyMonotonicityTracker,
    cone_limit: LimitTracker,
    reported: bool,
}

impl Default for Validation {
    fn default() -> Self {
        Self {
            quat_norm: QuaternionNormTracker::new(),
            energy: EnergyMonotonicityTracker::new(),
            cone_limit: LimitTracker::new(),
            reported: false,
        }
    }
}

fn diagnostics(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    mut timer: ResMut<DiagTimer>,
    mut val: ResMut<Validation>,
) {
    let time = data.time;

    // ── Quaternion norm ─────────────────────────────────────────────────
    let adr = model.jnt_qpos_adr[0];
    let (w, x, y, z) = (
        data.qpos[adr],
        data.qpos[adr + 1],
        data.qpos[adr + 2],
        data.qpos[adr + 3],
    );
    val.quat_norm.sample(w, x, y, z);

    // ── Energy monotonicity (damped + limited — energy should only decrease)
    // Skip the first 1s: the infeasible initial state (60° > 45° limit)
    // causes a transient as the constraint solver projects onto the feasible
    // set. After settling, energy should monotonically decrease.
    let energy = data.energy_kinetic + data.energy_potential;
    if time > 1.0 {
        val.energy.sample(energy);
    }

    // ── Cone limit: rotation angle vs 45° limit ────────────────────────
    // Same 1s settling window for the constraint transient.
    if time > 1.0 {
        let angle = quat_angle(w, x, y, z);
        val.cone_limit.sample(angle, 0.0, CONE_LIMIT_RAD);
    }

    // ── Per-second diagnostics ──────────────────────────────────────────
    if time - timer.last > 1.0 {
        timer.last = time;

        let angle = quat_angle(w, x, y, z);
        let angle_deg = angle.to_degrees();
        let limit_deg = CONE_LIMIT_RAD.to_degrees();

        let status = if angle > CONE_LIMIT_RAD {
            "OVER"
        } else {
            "ok  "
        };

        println!(
            "t={time:5.1}s  angle={angle_deg:5.1}° / {limit_deg:.0}° {status}  E={energy:.4}J"
        );
    }

    // ── Validation report at t=15s ──────────────────────────────────────
    if time > 15.0 && !val.reported {
        val.reported = true;

        let checks = vec![
            val.quat_norm.check(1e-10),
            // Constraint solver can inject tiny energy during limit enforcement.
            // 1e-3 J is well within acceptable solver tolerance.
            val.energy.check(1e-3),
            // Solver allows slight penetration (controlled by solref/solimp).
            // 0.02 rad ≈ 1.1° is typical for default solver parameters.
            val.cone_limit.check(0.02),
        ];
        let _ = print_report("Validation Report (t=15s)", &checks);
    }
}
