//! Vertical Slide Joint — Gravity-Shifted Spring-Mass Oscillator
//!
//! A block on a vertical rail, suspended by a spring from an overhead mount.
//! Gravity pulls it down; the spring pulls it back up. Released below
//! equilibrium, it oscillates as a damped harmonic oscillator around a
//! gravity-shifted rest position.
//!
//! Demonstrates: `type="slide"` on a vertical axis, gravity-shifted equilibrium,
//! joint stiffness as a restoring spring.
//!
//! Validates:
//! - Oscillation period matches `T = 2pi * sqrt((m + armature) / k)`
//! - Energy monotonically decreases (damping dissipates, never injects)
//! - Joint limits never exceeded
//! - Mean position converges to `x_eq = -mg/k` (gravity-shifted equilibrium)
//!
//! Run with: `cargo run -p example-slide-joint-vertical --release`

#![allow(
    clippy::needless_pass_by_value,
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::let_underscore_must_use,
    clippy::suboptimal_flops
)]

use bevy::asset::RenderAssetUsages;
use bevy::prelude::*;
use sim_bevy::camera::OrbitCameraPlugin;
use sim_bevy::convert::physics_pos;
use sim_bevy::examples::{DiagTimer, spawn_example_camera};
use sim_bevy::materials::{MetalPreset, override_geom_materials_by_name};
use sim_bevy::mesh::{SpringCoilParams, spring_coil};
use sim_bevy::model_data::{
    ModelGeomIndex, PhysicsAccumulator, PhysicsData, PhysicsModel, spawn_model_geoms,
    step_physics_realtime, sync_geom_transforms,
};
use sim_core::ENABLE_ENERGY;
use sim_core::validation::{
    EnergyMonotonicityTracker, EquilibriumTracker, LimitTracker, PeriodTracker, print_report,
};

// ── MJCF Model ──────────────────────────────────────────────────────────────
//
// A single slide joint along the Z axis (vertical in MuJoCo, maps to Y in Bevy).
//   mass = 1.0 kg, armature = 0.1 → effective mass = 1.1 kg
//   stiffness = 40 N/m, damping = 0.3 Ns/m
//   range = [-1.5, 1.0] m (asymmetric — block can drop further than rise)
//   Initial displacement: -0.5 m (below equilibrium)
//
// Visual: block on a vertical rail, spring above connecting to overhead mount.
// Gravity shifts equilibrium to x_eq = -mg/k = -0.2453 m.
//
const MJCF: &str = r#"
<mujoco model="vertical_slide_joint">
    <compiler angle="radian"/>
    <option gravity="0 0 -9.81" timestep="0.001" integrator="RK4"/>

    <default>
        <geom contype="0" conaffinity="0"/>
    </default>

    <worldbody>
        <!-- Vertical rail along Z axis -->
        <geom name="rail" type="capsule" size="0.02"
              fromto="0 0 -0.5  0 0 2.0" rgba="0.50 0.50 0.53 1"/>

        <!-- Overhead mount at top of rail -->
        <geom name="mount" type="box" size="0.10 0.10 0.02"
              pos="0 0 2.0" rgba="0.45 0.45 0.48 1"/>

        <!-- Ground plane (visual reference) -->
        <geom name="ground" type="plane" size="1.0 1.0 0.01"
              pos="0 0 -0.5" rgba="0.35 0.35 0.38 1"/>

        <!-- The sliding block -->
        <body name="block" pos="0 0 1.0">
            <joint name="slide" type="slide" axis="0 0 1"
                   stiffness="40" damping="0.3" armature="0.1"
                   limited="true" range="-1.5 1.0"/>
            <inertial pos="0 0 0" mass="1.0" diaginertia="0.01 0.01 0.01"/>
            <geom name="block" type="box" size="0.08 0.08 0.08"
                  rgba="0.15 0.45 0.82 1"
                  contype="1" conaffinity="1"/>
        </body>
    </worldbody>
</mujoco>
"#;

// ── Physics constants ───────────────────────────────────────────────────────

const MASS: f64 = 1.0;
const ARMATURE: f64 = 0.1;
const MASS_EFF: f64 = MASS + ARMATURE;
const STIFFNESS: f64 = 40.0;
const GRAVITY: f64 = 9.81;
const INITIAL_DISP: f64 = -0.5;

// MJCF positions (Z-up) — use physics_pos() to convert to Bevy (Y-up)
const BODY_ANCHOR_Z: f32 = 1.0; // MJCF: pos="0 0 1.0"
const MOUNT_Z: f32 = 2.0; // MJCF: pos="0 0 2.0"
const BLOCK_HALF: f32 = 0.08; // half-size of block geom

/// Analytical equilibrium: `x_eq = -mg/k`
fn analytical_equilibrium() -> f64 {
    -(MASS * GRAVITY) / STIFFNESS
}

/// Analytical period: `T = 2pi * sqrt(m_eff / k)`
fn analytical_period() -> f64 {
    2.0 * std::f64::consts::PI * (MASS_EFF / STIFFNESS).sqrt()
}

// ── Spring visual ───────────────────────────────────────────────────────────

const COIL_PARAMS: SpringCoilParams = SpringCoilParams {
    turns: 10,
    radius: 0.05,
    tube_radius: 0.006,
    segments_per_turn: 24,
    tube_segments: 8,
    min_length: 0.05,
};

/// Marker for the overhead spring.
#[derive(Component)]
struct OverheadSpring;

// ── Bevy App ────────────────────────────────────────────────────────────────

fn main() {
    let t = analytical_period();
    let x_eq = analytical_equilibrium();
    println!("=== CortenForge: Vertical Slide Joint ===");
    println!("  Mass on a vertical spring with gravity");
    println!("  m=1.0kg  armature=0.1  k=40 N/m  c=0.3 Ns/m");
    println!("  Effective mass = {MASS_EFF:.1} kg");
    println!("  Analytical period T = {t:.4} s");
    println!("  Equilibrium x_eq = {x_eq:.4} m (gravity shift)");
    println!("  Initial displacement = {INITIAL_DISP} m");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Vertical Slide Joint".into(),
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
        .add_systems(
            PostUpdate,
            (sync_geom_transforms, update_spring, diagnostics),
        )
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

    data.qpos[0] = INITIAL_DISP;
    let _ = data.forward(&model);

    println!(
        "  Model: {} bodies, {} joints, {} DOFs\n",
        model.nbody, model.njnt, model.nv
    );

    // ── Spawn MJCF geometry ─────────────────────────────────────────────
    spawn_model_geoms(&mut commands, &mut meshes, &mut materials, &model, &data);

    // ── Metallic materials ──────────────────────────────────────────────
    let mat_rail = materials.add(MetalPreset::PolishedSteel.material());
    let mat_mount = materials.add(MetalPreset::BrushedMetal.material());
    let mat_block =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.15, 0.45, 0.82)));
    let mat_spring = materials.add(MetalPreset::SpringWire.material());

    // ── Spring: mount → block top ───────────────────────────────────────
    // All positions in MJCF Z-up; physics_pos() converts to Bevy Y-up.
    let block_z = BODY_ANCHOR_Z + data.qpos[0] as f32;
    let spring_bottom_z = block_z + BLOCK_HALF;
    let spring_length = MOUNT_Z - spring_bottom_z;

    // Spring coil extends along +X; rotate -90° around Z to make it extend along +Y
    let rotation = Quat::from_rotation_z(std::f32::consts::FRAC_PI_2);
    commands.spawn((
        OverheadSpring,
        Mesh3d(meshes.add(spring_coil(
            &COIL_PARAMS,
            spring_length,
            RenderAssetUsages::default(),
        ))),
        MeshMaterial3d(mat_spring),
        Transform {
            translation: physics_pos(0.0, 0.0, spring_bottom_z),
            rotation,
            ..default()
        },
    ));

    // ── Camera + lights ─────────────────────────────────────────────────
    spawn_example_camera(
        &mut commands,
        physics_pos(0.0, 0.0, 1.0),
        4.0,
        std::f32::consts::FRAC_PI_2,
        0.15,
    );

    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
    commands.insert_resource(MaterialOverrides {
        rail: mat_rail,
        mount: mat_mount,
        block: mat_block,
    });
}

// ── Material override (runs once) ───────────────────────────────────────────

#[derive(Resource)]
struct MaterialOverrides {
    rail: Handle<StandardMaterial>,
    mount: Handle<StandardMaterial>,
    block: Handle<StandardMaterial>,
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
            ("rail", ovr.rail.clone()),
            ("mount", ovr.mount.clone()),
            ("block", ovr.block.clone()),
        ],
        &mut query,
    );

    commands.remove_resource::<MaterialOverrides>();
}

// ── Spring update ───────────────────────────────────────────────────────────

fn update_spring(
    data: Res<PhysicsData>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut query: Query<(&Mesh3d, &mut Transform), With<OverheadSpring>>,
) {
    let block_z = BODY_ANCHOR_Z + data.qpos[0] as f32;
    let spring_bottom_z = block_z + BLOCK_HALF;
    let spring_length = MOUNT_Z - spring_bottom_z;

    for (mesh_handle, mut transform) in &mut query {
        transform.translation = physics_pos(0.0, 0.0, spring_bottom_z);
        if let Some(mesh) = meshes.get_mut(&mesh_handle.0) {
            *mesh = spring_coil(&COIL_PARAMS, spring_length, RenderAssetUsages::default());
        }
    }
}

// ── Diagnostics & Validation ────────────────────────────────────────────────

#[derive(Resource)]
struct Validation {
    period: PeriodTracker,
    energy: EnergyMonotonicityTracker,
    limits: LimitTracker,
    equilibrium: EquilibriumTracker,
    reported: bool,
}

impl Default for Validation {
    fn default() -> Self {
        Self {
            period: PeriodTracker::new(),
            energy: EnergyMonotonicityTracker::new(),
            limits: LimitTracker::new(),
            equilibrium: EquilibriumTracker::new(5.0), // 5s trailing window
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
    let pos = data.qpos[0];
    let energy = data.energy_kinetic + data.energy_potential;
    let time = data.time;
    let x_eq = analytical_equilibrium();

    // Period tracker: sample displacement relative to equilibrium
    // (so zero crossings correspond to passing through equilibrium)
    val.period.sample(pos - x_eq, time);
    val.energy.sample(energy);
    val.equilibrium.sample(pos, time);

    if model.jnt_limited[0] {
        let (lo, hi) = model.jnt_range[0];
        val.limits.sample(pos, lo, hi);
    }

    if time - timer.last > 1.0 {
        timer.last = time;
        println!("t={time:5.1}s  pos={pos:+.4}m  x_eq={x_eq:+.4}m  E={energy:.4}J");
    }

    if time > 15.0 && !val.reported {
        val.reported = true;
        let checks = vec![
            val.period.check(analytical_period(), 2.0),
            val.energy.check(1e-6),
            val.limits.check(0.001),
            val.equilibrium.check(x_eq, 5.0),
        ];
        let _ = print_report("Validation Report (t=15s)", &checks);
    }
}
