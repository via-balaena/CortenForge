//! Primitive Geometry Shapes
//!
//! A 2-link arm with all three URDF collision primitives visible:
//! - Box (base, fused into worldbody — stationary platform)
//! - Sphere (link1 — elbow joint)
//! - Cylinder (link2 — forearm)
//!
//! The arm swings under gravity so you can see all three shapes in motion.
//! Verifies the size conversions: URDF full-extents → MJCF half-extents.
//!
//! Run: `cargo run -p example-urdf-geometry --release`

#![allow(
    clippy::doc_markdown,
    clippy::needless_pass_by_value,
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::cast_lossless,
    clippy::let_underscore_must_use,
    clippy::suboptimal_flops,
    clippy::too_many_lines
)]

use bevy::prelude::*;
use sim_bevy::camera::OrbitCameraPlugin;
use sim_bevy::convert::physics_pos;
use sim_bevy::examples::{
    PhysicsHud, ValidationHarness, render_physics_hud, spawn_example_camera, spawn_physics_hud,
    validation_system,
};
use sim_bevy::model_data::{
    PhysicsAccumulator, PhysicsData, PhysicsModel, spawn_model_geoms, step_physics_realtime,
    sync_geom_transforms,
};
use sim_core::GeomType;
use sim_core::validation::{Check, print_report};

// ── URDF Model ───────────────────────────────────────────────────────────

/// Three shapes on a single pendulum arm: box (shoulder), cylinder (rod),
/// sphere (tip). All on one body so they move together as a unit — no
/// wobbly second joint. The point is seeing the three primitives, not
/// articulation.
const GEOMETRY_URDF: &str = r#"<?xml version="1.0"?>
<robot name="geometry_test">
    <link name="base">
        <inertial>
            <mass value="5.0"/>
            <inertia ixx="0.5" ixy="0" ixz="0" iyy="0.5" iyz="0" izz="0.5"/>
        </inertial>
    </link>
    <link name="arm">
        <inertial>
            <origin xyz="0 0 -0.4"/>
            <mass value="2.0"/>
            <inertia ixx="0.02" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.002"/>
        </inertial>
        <collision>
            <origin xyz="0 0 -0.05"/>
            <geometry>
                <box size="0.2 0.4 0.1"/>
            </geometry>
        </collision>
        <collision>
            <origin xyz="0 0 -0.25"/>
            <geometry>
                <cylinder radius="0.08" length="0.3"/>
            </geometry>
        </collision>
        <collision>
            <origin xyz="0 0 -0.55"/>
            <geometry>
                <sphere radius="0.15"/>
            </geometry>
        </collision>
    </link>
    <joint name="j1" type="revolute">
        <parent link="base"/>
        <child link="arm"/>
        <origin xyz="0 0 2.0"/>
        <axis xyz="0 1 0"/>
        <limit lower="-1.57" upper="1.57"/>
    </joint>
</robot>
"#;

// ── Bevy App ─────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: URDF Primitive Geometry Shapes ===");
    println!("  Sphere (link1) + Cylinder + Box (link2)");
    println!("  URDF full-extents → MJCF half-extents");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — URDF Geometry Shapes".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .init_resource::<PhysicsHud>()
        .init_resource::<GeomValidation>()
        .insert_resource(
            ValidationHarness::new()
                .report_at(6.0)
                .print_every(2.0)
                .display(|m, d| {
                    let a1 = d.joint_qpos(m, 0)[0];
                    format!("angle={:+5.1}°", a1.to_degrees())
                }),
        )
        .add_systems(Startup, setup)
        .add_systems(Update, step_physics_realtime)
        .add_systems(
            PostUpdate,
            (
                sync_geom_transforms,
                validation_system,
                geom_diagnostics,
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
    let mjcf = sim_urdf::urdf_to_mjcf(GEOMETRY_URDF).expect("URDF→MJCF");
    let mjcf = mjcf.replace(
        r#"timestep="0.002"/>"#,
        r#"timestep="0.002"><flag energy="enable"/></option>"#,
    );
    let model = sim_mjcf::load_model(&mjcf).expect("MJCF should parse");
    let mut data = model.make_data();

    data.qpos[0] = 0.4; // offset for visible swing
    let _ = data.forward(&model);

    let types: Vec<GeomType> = (0..model.ngeom).map(|i| model.geom_type[i]).collect();
    println!("  Model: {} geoms, types: {types:?}\n", model.ngeom);

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[],
    );

    spawn_example_camera(
        &mut commands,
        physics_pos(0.0, 0.0, 1.5),
        3.5,
        std::f32::consts::FRAC_PI_4,
        0.3,
    );
    spawn_physics_hud(&mut commands);

    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── HUD ──────────────────────────────────────────────────────────────────

fn update_hud(model: Res<PhysicsModel>, data: Res<PhysicsData>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();
    hud.section("URDF Geometry Shapes");

    let types: Vec<&str> = (0..model.ngeom)
        .map(|i| match model.geom_type[i] {
            GeomType::Box => "Box",
            GeomType::Sphere => "Sphere",
            GeomType::Cylinder => "Cylinder",
            _ => "Other",
        })
        .collect();
    hud.raw(format!("geoms: {}", types.join(", ")));

    let a1 = data.joint_qpos(&model, 0)[0];
    hud.scalar("angle (deg)", a1.to_degrees(), 1);
    hud.scalar("time", data.time, 1);
}

// ── Validation ───────────────────────────────────────────────────────────

#[derive(Resource, Default)]
struct GeomValidation {
    reported: bool,
}

fn geom_diagnostics(
    model: Res<PhysicsModel>,
    _data: Res<PhysicsData>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<GeomValidation>,
) {
    if !harness.reported() || val.reported {
        return;
    }
    val.reported = true;

    let types: Vec<GeomType> = (0..model.ngeom).map(|i| model.geom_type[i]).collect();
    let has_sphere = types.contains(&GeomType::Sphere);
    let has_cyl = types.contains(&GeomType::Cylinder);
    let has_box = types.contains(&GeomType::Box);

    // Check sphere radius
    let sphere_ok = (0..model.ngeom)
        .filter(|&i| model.geom_type[i] == GeomType::Sphere)
        .any(|i| (model.geom_size[i].x - 0.15).abs() < 0.001);

    // Check cylinder size
    let cyl_ok = (0..model.ngeom)
        .filter(|&i| model.geom_type[i] == GeomType::Cylinder)
        .any(|i| {
            (model.geom_size[i].x - 0.08).abs() < 0.001
                && (model.geom_size[i].y - 0.15).abs() < 0.001
        });

    // Check box half-extents
    let box_ok = (0..model.ngeom)
        .filter(|&i| model.geom_type[i] == GeomType::Box)
        .any(|i| {
            (model.geom_size[i].x - 0.1).abs() < 0.001
                && (model.geom_size[i].y - 0.2).abs() < 0.001
                && (model.geom_size[i].z - 0.05).abs() < 0.001
        });

    let checks = vec![
        Check {
            name: "All 3 primitives present",
            pass: has_sphere && has_cyl && has_box,
            detail: format!("sphere={has_sphere} cyl={has_cyl} box={has_box}"),
        },
        Check {
            name: "Sphere radius = 0.15",
            pass: sphere_ok,
            detail: "checked".into(),
        },
        Check {
            name: "Cylinder (r=0.08, hl=0.15)",
            pass: cyl_ok,
            detail: "checked".into(),
        },
        Check {
            name: "Box half-extents (0.1, 0.2, 0.05)",
            pass: box_ok,
            detail: "checked".into(),
        },
    ];
    let _ = print_report("URDF Geometry", &checks);
}
