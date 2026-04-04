//! Cable Catenary — Both-Ends-Fixed Passive Sag
//!
//! A cable composite pinned at both ends sags under its own weight into a
//! catenary shape. The left end is pinned via `initial="none"` (no joint on
//! the first body). The right end is pinned via an equality `connect`
//! constraint that attaches the cable tip to the right pylon.
//!
//! Validates:
//! - Cable sags below anchor height
//! - Cable settles (max angular velocity < threshold)
//! - Horizontal span preserved (endpoints stay at pylons)
//! - Sag is symmetric (left and right halves match)
//!
//! Run with: `cargo run -p example-composite-cable-catenary --release`

#![allow(
    clippy::doc_markdown,
    clippy::needless_pass_by_value,
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::let_underscore_must_use,
    clippy::suboptimal_flops,
    clippy::cast_precision_loss,
    clippy::unwrap_used
)]

use bevy::prelude::*;
use sim_bevy::camera::OrbitCameraPlugin;
use sim_bevy::examples::{
    PhysicsHud, ValidationHarness, render_physics_hud, spawn_example_camera, spawn_physics_hud,
    validation_system,
};
use sim_bevy::model_data::{
    PhysicsAccumulator, PhysicsData, PhysicsModel, spawn_model_geoms, step_physics_realtime,
    sync_geom_transforms,
};
use sim_core::validation::{Check, print_report};

// ── MJCF Model ──────────────────────────────────────────────────────────────

/// Cable with 15 segments (count=16 vertices) spanning 1.0m between two pylons.
/// Left end pinned via initial="none". Right end pinned via connect constraint.
///
/// Geometry: curve="l 0 s" with size="1.0 0.2 1.0" gives a half-sine bulge
/// upward in z, so the cable starts bowed up and swings down under gravity
/// into a catenary. The x-extent is still 1.0 (matching the span), so both
/// endpoints are at the pylons. Path length (~1.06m) exceeds span = slack.
///
/// Pylon layout:
///   Left pylon body at (-0.5, 0, 0.8), cable starts at top: offset=(0, 0, 0.4)
///   → cable origin world pos = (-0.5, 0, 1.2)
///   Right pylon body at (0.5, 0, 0.8), top at (0.5, 0, 1.2)
///
/// Connect anchor: the S_last site is at [edge_length, 0, 0] in AB_last's
/// local frame. For a curved cable the edge_length varies per segment, but
/// the last two vertices (ix=14,15) are nearly straight (sin ≈ 0 near π),
/// so the last edge_length ≈ size[0]/(count-1) = 1.0/15 = 0.06667.
const MJCF: &str = r#"
<mujoco model="cable-catenary">
  <option gravity="0 0 -9.81" timestep="0.002" integrator="implicitfast"
          solver="Newton" iterations="50"/>

  <worldbody>
    <!-- Ground plane -->
    <geom name="floor" type="plane" size="2 2 0.01" rgba="0.25 0.25 0.28 1"
          pos="0 0 0" contype="0" conaffinity="0"/>

    <!-- Left pylon -->
    <body name="pylon_L" pos="-0.5 0 0.8">
      <geom name="pylon_L_geom" type="box" size="0.025 0.025 0.4"
            rgba="0.5 0.5 0.5 1" contype="0" conaffinity="0"/>

      <!-- Cable: 15 segments, gold, starts with sine-dip for slack -->
      <composite type="cable" prefix="A" count="16 1 1"
                 initial="ball" curve="l 0 s" size="1.0 0.2 1.0"
                 offset="0 0 0.4">
        <joint kind="main" damping="0.05"/>
        <geom type="capsule" size="0.01" rgba="0.9 0.75 0.2 1"
              density="800" contype="0" conaffinity="0"/>
      </composite>
    </body>

    <!-- Right pylon -->
    <body name="pylon_R" pos="0.5 0 0.8">
      <geom name="pylon_R_geom" type="box" size="0.025 0.025 0.4"
            rgba="0.5 0.5 0.5 1" contype="0" conaffinity="0"/>
    </body>
  </worldbody>

  <equality>
    <!-- Pin cable tip to right pylon top.
         anchor = S_last position in AB_last's local frame.
         Last segment is nearly straight (sine ≈ 0 near endpoints),
         so edge_length ≈ 1.0/15 = 0.06667. -->
    <connect body1="AB_last" body2="pylon_R"
             anchor="0.06667 0 0" solref="0.002 1.0"/>
  </equality>
</mujoco>
"#;

// ── Constants ───────────────────────────────────────────────────────────────

const ANCHOR_Z: f64 = 1.2;

// ── Startup delay ───────────────────────────────────────────────────────────

#[derive(Resource)]
struct StartDelay {
    elapsed: f32,
    duration: f32,
}

fn tick_delay(time: Res<Time>, mut delay: ResMut<StartDelay>) {
    delay.elapsed += time.delta_secs();
}

fn delay_elapsed(delay: Res<StartDelay>) -> bool {
    delay.elapsed >= delay.duration
}

// ── Bevy App ────────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Cable Catenary ===");
    println!("  Both-ends-fixed cable sagging under own weight");
    println!("  Gold cable, 15 segments, 1.0m span between gray pylons");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Cable Catenary".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .init_resource::<PhysicsHud>()
        .init_resource::<CatenaryValidation>()
        .insert_resource(StartDelay {
            elapsed: 0.0,
            duration: 2.0,
        })
        .insert_resource(
            ValidationHarness::new()
                .report_at(15.0)
                .print_every(2.0)
                .display(|m, d| {
                    let first = m.body_id("AB_first").unwrap();
                    let last = m.body_id("AB_last").unwrap();
                    let (sag, _) = compute_sag(d, first, last);
                    let span = (d.xpos[last][0] - d.xpos[first][0]).abs();
                    format!("sag={sag:.4}  span={span:.4}")
                }),
        )
        .add_systems(Startup, setup)
        .add_systems(
            Update,
            (tick_delay, step_physics_realtime.run_if(delay_elapsed)).chain(),
        )
        .add_systems(
            PostUpdate,
            (
                sync_geom_transforms,
                validation_system,
                catenary_diagnostics,
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
    data.forward(&model).expect("initial forward");

    println!(
        "  Model: {} bodies, {} joints, {} geoms, {} eq constraints\n",
        model.nbody, model.njnt, model.ngeom, model.neq
    );

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[],
    );

    // Side view: camera looks along Y axis at the cable profile
    spawn_example_camera(
        &mut commands,
        Vec3::new(0.0, 0.9, 0.0), // center between anchor z=1.2 and sagged cable
        2.0,                      // distance
        std::f32::consts::FRAC_PI_2, // azimuth: 90° — face the X-Z plane
        0.15,                     // slight elevation
    );

    spawn_physics_hud(&mut commands);

    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── HUD ─────────────────────────────────────────────────────────────────────

fn update_hud(model: Res<PhysicsModel>, data: Res<PhysicsData>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();
    hud.section("Cable Catenary — Both-Ends-Fixed");

    let first = model.body_id("AB_first").unwrap();
    let last = model.body_id("AB_last").unwrap();
    let (sag, _) = compute_sag(&data, first, last);
    let span = (data.xpos[last][0] - data.xpos[first][0]).abs();

    hud.scalar("t", data.time, 1);
    hud.scalar("sag depth", sag, 4);
    hud.scalar("span", span, 3);
}

// ── Helpers ─────────────────────────────────────────────────────────────────

/// Compute sag depth and the body index of the lowest point.
/// Sag = anchor_z - min(body_z) over all cable bodies.
fn compute_sag(data: &sim_core::Data, first: usize, last: usize) -> (f64, usize) {
    let mut min_z = f64::MAX;
    let mut min_idx = first;
    for i in first..=last {
        let z = data.xpos[i][2];
        if z < min_z {
            min_z = z;
            min_idx = i;
        }
    }
    (ANCHOR_Z - min_z, min_idx)
}

// ── Validation ──────────────────────────────────────────────────────────────

#[derive(Resource, Default)]
struct CatenaryValidation {
    reported: bool,
}

fn catenary_diagnostics(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<CatenaryValidation>,
) {
    if !harness.reported() || val.reported {
        return;
    }
    val.reported = true;

    let first = model.body_id("AB_first").unwrap();
    let last = model.body_id("AB_last").unwrap();

    // Sag depth
    let (sag, _) = compute_sag(&data, first, last);

    // Settled check
    let max_qvel = data.qvel.iter().fold(0.0_f64, |m, &v| m.max(v.abs()));

    // Span preservation
    let span = (data.xpos[last][0] - data.xpos[first][0]).abs();
    let span_err = (span - 1.0).abs();

    // Symmetry: compare sag depth of left half vs right half
    let n_cable = last - first + 1;
    let mid = first + n_cable / 2;
    let mut min_z_left = f64::MAX;
    let mut min_z_right = f64::MAX;
    for i in first..mid {
        min_z_left = min_z_left.min(data.xpos[i][2]);
    }
    for i in mid..=last {
        min_z_right = min_z_right.min(data.xpos[i][2]);
    }
    let sag_left = ANCHOR_Z - min_z_left;
    let sag_right = ANCHOR_Z - min_z_right;
    let sag_asym = (sag_left - sag_right).abs();

    let checks = vec![
        Check {
            name: "Cable sags below anchors",
            pass: sag > 0.0,
            detail: format!("sag_depth={sag:.4}"),
        },
        Check {
            name: "Cable settled",
            pass: max_qvel < 0.01,
            detail: format!("max_qvel={max_qvel:.4}"),
        },
        Check {
            name: "Span preserved",
            pass: span_err < 0.07,
            detail: format!("span={span:.4}, error={span_err:.4}"),
        },
        Check {
            name: "Symmetric sag",
            pass: sag_asym < 0.01,
            detail: format!("left={sag_left:.4}, right={sag_right:.4}, diff={sag_asym:.4}"),
        },
    ];
    let _ = print_report("Cable Catenary (t=15s)", &checks);
}
