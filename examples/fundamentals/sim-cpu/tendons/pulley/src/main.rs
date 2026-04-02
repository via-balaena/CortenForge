//! Pulley — Mechanical Advantage via Tendon Branch Scaling
//!
//! Two masses on parallel horizontal rails connected through a pulley.
//! A motor drives the top mass left and right. The bottom mass follows at
//! half the rate — the pulley's divisor=2 creates a 2:1 mechanical advantage.
//!
//! Run with: `cargo run -p example-tendon-pulley --release`

#![allow(
    clippy::doc_markdown,
    clippy::needless_pass_by_value,
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::let_underscore_must_use,
    clippy::suboptimal_flops,
    clippy::cast_sign_loss,
    clippy::similar_names
)]

use bevy::prelude::*;
use sim_bevy::camera::OrbitCameraPlugin;
use sim_bevy::convert::{physics_pos, vec3_from_vector};
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

// ── MJCF Model ────────────────────────────────────────────────────────────

/// Two masses on parallel horizontal rails. A pulley (divisor=2) connects
/// them through a central anchor. When the top mass moves ±X, the bottom
/// mass moves ±X/2. Zero gravity, stiff tendon spring for tight coupling.
/// A motor on the top mass drives it sinusoidally.
const MJCF: &str = r#"
<mujoco model="pulley">
  <option gravity="0 0 0" timestep="0.002"/>

  <worldbody>
    <!-- Top mass (blue, load) — vertical rail, spring-centered -->
    <body name="top" pos="0 0 1.1">
      <joint name="j_top" type="slide" axis="0 0 1" damping="1.0"
             stiffness="8" springref="0"/>
      <geom name="top_geom" type="box" size="0.05 0.05 0.04"
            rgba="0.3 0.5 0.85 1" mass="1.0"/>
      <site name="s1" pos="0 0 -0.04" size="0.012" rgba="1 1 0 1"/>
    </body>

    <!-- Pulley anchor (yellow, fixed, between the two masses) -->
    <body name="anchor" pos="0 0 0.7">
      <geom name="anchor_geom" type="sphere" size="0.035"
            rgba="0.9 0.85 0.2 1" contype="0" conaffinity="0"/>
      <site name="s2" pos="0 0 0.035" size="0.012" rgba="1 1 0 1"/>
      <site name="s3" pos="0 0 -0.035" size="0.012" rgba="1 1 0 1"/>
    </body>

    <!-- Bottom mass (red, effort/pull side) — vertical rail, spring-centered -->
    <body name="bottom" pos="0 0 0.3">
      <joint name="j_bot" type="slide" axis="0 0 1" damping="1.0"
             stiffness="8" springref="0"/>
      <geom name="bot_geom" type="box" size="0.05 0.05 0.04"
            rgba="0.85 0.25 0.2 1" mass="1.0"/>
      <site name="s4" pos="0 0 0.04" size="0.012" rgba="1 1 0 1"/>
    </body>
  </worldbody>

  <tendon>
    <spatial name="pulley_tendon" stiffness="500" damping="10">
      <site site="s1"/>
      <site site="s2"/>
      <pulley divisor="2"/>
      <site site="s3"/>
      <site site="s4"/>
    </spatial>
  </tendon>

  <actuator>
    <motor name="drive" joint="j_bot" gear="1"/>
  </actuator>

  <sensor>
    <tendonpos name="ten_pos" tendon="pulley_tendon"/>
  </sensor>
</mujoco>
"#;

// ── Bevy App ──────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Pulley — Tendon ===");
    println!("  Bottom (red): motor-driven, pulls the rope");
    println!("  Top (blue): the load — moves half as far (divisor=2)");
    println!("  Watch the top mass follow at half the rate");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge \u{2014} Pulley (Tendons)".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .init_resource::<PhysicsHud>()
        .init_resource::<RangeTracker>()
        .insert_resource(
            ValidationHarness::new()
                .report_at(10.0)
                .print_every(1.0)
                .display(|m, d| {
                    let jt = m.joint_id("j_top").unwrap();
                    let jb = m.joint_id("j_bot").unwrap();
                    let qt = d.qpos[m.jnt_qpos_adr[jt]];
                    let qb = d.qpos[m.jnt_qpos_adr[jb]];
                    format!("top={qt:+.3}  bot={qb:+.3}  ratio={:.2}", qt / qb.max(1e-6))
                }),
        )
        .add_systems(Startup, setup)
        .add_systems(Update, (apply_control, step_physics_realtime).chain())
        .add_systems(
            PostUpdate,
            (
                sync_geom_transforms,
                validation_system,
                diagnostics,
                draw_tendon_path,
                draw_ruler_ticks,
                update_hud,
                render_physics_hud,
            )
                .chain(),
        )
        .run();
}

/// Tracks the travel range of each mass.
#[derive(Resource)]
struct RangeTracker {
    top_min: f64,
    top_max: f64,
    bot_min: f64,
    bot_max: f64,
}

impl Default for RangeTracker {
    fn default() -> Self {
        Self {
            top_min: f64::MAX,
            top_max: f64::MIN,
            bot_min: f64::MAX,
            bot_max: f64::MIN,
        }
    }
}

// ── Setup ─────────────────────────────────────────────────────────────────

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let model = sim_mjcf::load_model(MJCF).expect("MJCF should parse");
    let mut data = model.make_data();
    data.forward(&model).expect("forward");

    let mat_top = materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.3, 0.5, 0.85)));
    let mat_bot =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.85, 0.25, 0.2)));
    let mat_anchor =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.9, 0.85, 0.2)));

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[
            ("top_geom", mat_top),
            ("anchor_geom", mat_anchor),
            ("bot_geom", mat_bot),
        ],
    );

    spawn_example_camera(
        &mut commands,
        physics_pos(0.0, 0.0, 0.7),
        1.8,
        std::f32::consts::FRAC_PI_2,
        0.15,
    );
    spawn_physics_hud(&mut commands);

    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── Control ───────────────────────────────────────────────────────────────

/// Slow sinusoidal drive on the top mass.
fn apply_control(model: Res<PhysicsModel>, mut data: ResMut<PhysicsData>) {
    let t = data.time;
    data.ctrl[0] = 1.5 * (2.0 * std::f64::consts::PI * 0.2 * t).sin();
    let _ = &model;
}

// ── Tendon Visualization ──────────────────────────────────────────────────

/// Self-calibrating min/max for each branch's segment length.
#[derive(Default)]
struct BandState {
    b1_min: f32,
    b1_max: f32,
    b2_min: f32,
    b2_max: f32,
    init: bool,
}

/// Draw a ribbon of `N` parallel lines between two points.
fn draw_ribbon(gizmos: &mut Gizmos, start: Vec3, end: Vec3, half_width: f32, color: Color) {
    const N: usize = 5;
    let dir = (end - start).normalize_or_zero();
    let mut perp = dir.cross(Vec3::Z);
    if perp.length_squared() < 0.001 {
        perp = dir.cross(Vec3::X);
    }
    perp = perp.normalize_or_zero();
    for s in 0..N {
        let t = s as f32 / (N - 1) as f32 - 0.5; // −0.5 … +0.5
        let offset = perp * (t * 2.0 * half_width);
        gizmos.line(start + offset, end + offset, color);
    }
}

/// Map a branch length to half-width: short → thick, long → thin.
fn band_half_width(len: f32, min_len: f32, max_len: f32) -> f32 {
    const MAX_HW: f32 = 0.018;
    const MIN_HW: f32 = 0.003;
    let range = max_len - min_len;
    if range < 1e-6 {
        return (MAX_HW + MIN_HW) * 0.5;
    }
    let t = ((len - min_len) / range).clamp(0.0, 1.0);
    MAX_HW + t * (MIN_HW - MAX_HW) // t=0 (shortest) → MAX, t=1 (longest) → MIN
}

/// Draw tendon branches as resistance-band ribbons + ruler tick marks.
///
/// Ribbon width is inversely proportional to branch length — thicker when
/// compressed, thinner when stretched.  Self-calibrates over the first cycle.
fn draw_tendon_path(
    mut gizmos: Gizmos,
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    mut band: Local<BandState>,
) {
    let tid = model.tendon_id("pulley_tendon").expect("tendon");
    let adr = data.ten_wrapadr[tid];
    let num = data.ten_wrapnum[tid];

    if num >= 2 {
        // -- Branch lengths for band width --
        let s1 = model.site_id("s1").expect("s1");
        let s2 = model.site_id("s2").expect("s2");
        let s3 = model.site_id("s3").expect("s3");
        let s4 = model.site_id("s4").expect("s4");
        let b1 = (data.site_xpos[s1] - data.site_xpos[s2]).norm() as f32;
        let b2 = (data.site_xpos[s3] - data.site_xpos[s4]).norm() as f32;
        if !band.init {
            band.b1_min = b1;
            band.b1_max = b1;
            band.b2_min = b2;
            band.b2_max = b2;
            band.init = true;
        }
        band.b1_min = band.b1_min.min(b1);
        band.b1_max = band.b1_max.max(b1);
        band.b2_min = band.b2_min.min(b2);
        band.b2_max = band.b2_max.max(b2);
        let b1_hw = band_half_width(b1, band.b1_min, band.b1_max);
        let b2_hw = band_half_width(b2, band.b2_min, band.b2_max);

        // -- 4-color scheme --
        let jt = model.joint_id("j_top").expect("j_top");
        let jb = model.joint_id("j_bot").expect("j_bot");
        let top_vel = data.qvel[model.jnt_dof_adr[jt]];
        let bot_vel = data.qvel[model.jnt_dof_adr[jb]];
        let top_mag = (data.qpos[model.jnt_qpos_adr[jt]].abs() / 0.15).clamp(0.1, 1.0) as f32;
        let bot_mag = (data.qpos[model.jnt_qpos_adr[jb]].abs() / 0.15).clamp(0.1, 1.0) as f32;

        let load_color = if top_vel > 0.005 {
            Color::srgb(0.95 * top_mag, 0.2 * top_mag, 0.1)
        } else if top_vel < -0.005 {
            Color::srgb(0.15, 0.3 * top_mag, 0.9 * top_mag)
        } else {
            Color::srgb(0.35, 0.35, 0.3)
        };
        let pull_color = if bot_vel < -0.005 {
            Color::srgb(0.95 * bot_mag, 0.5 * bot_mag, 0.1)
        } else if bot_vel > 0.005 {
            Color::srgb(0.1, 0.7 * bot_mag, 0.7 * bot_mag)
        } else {
            Color::srgb(0.35, 0.35, 0.3)
        };

        // Find pulley marker
        let mut pulley_idx = None;
        for i in 0..num {
            if data.wrap_obj[adr + i] == -2 {
                pulley_idx = Some(i);
                break;
            }
        }

        // Draw ribbon segments
        for i in 0..num - 1 {
            if data.wrap_obj[adr + i] == -2 || data.wrap_obj[adr + i + 1] == -2 {
                continue;
            }
            let is_branch1 = pulley_idx.is_some_and(|pi| i < pi);
            let color = if is_branch1 { load_color } else { pull_color };
            let hw = if is_branch1 { b1_hw } else { b2_hw };
            let start = vec3_from_vector(&data.wrap_xpos[adr + i]);
            let end = vec3_from_vector(&data.wrap_xpos[adr + i + 1]);
            draw_ribbon(&mut gizmos, start, end, hw, color);
        }

        // Site dots
        for i in 0..num {
            if data.wrap_obj[adr + i] != -2 {
                let color = if pulley_idx.is_some_and(|pi| i <= pi) {
                    load_color
                } else {
                    pull_color
                };
                let pos = vec3_from_vector(&data.wrap_xpos[adr + i]);
                gizmos.sphere(Isometry3d::from_translation(pos), 0.008, color);
            }
        }
    }
}

/// Ruler tick marks along each vertical rail (every 5 cm).
fn draw_ruler_ticks(mut gizmos: Gizmos) {
    let tick_color = Color::srgba(0.6, 0.6, 0.6, 0.5);
    let tick_half = 0.08;

    for &center_y in &[1.1_f32, 0.3_f32] {
        for i in -6..=6 {
            let y = center_y + i as f32 * 0.05;
            gizmos.line(
                Vec3::new(-tick_half, y, 0.0),
                Vec3::new(tick_half, y, 0.0),
                tick_color,
            );
        }
    }
}

// ── HUD ───────────────────────────────────────────────────────────────────

fn update_hud(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    mut hud: ResMut<PhysicsHud>,
    mut range: ResMut<RangeTracker>,
) {
    let jt = model.joint_id("j_top").expect("j_top");
    let jb = model.joint_id("j_bot").expect("j_bot");
    let qt = data.qpos[model.jnt_qpos_adr[jt]];
    let qb = data.qpos[model.jnt_qpos_adr[jb]];

    // Update range tracker
    range.top_min = range.top_min.min(qt);
    range.top_max = range.top_max.max(qt);
    range.bot_min = range.bot_min.min(qb);
    range.bot_max = range.bot_max.max(qb);

    let top_travel = range.top_max - range.top_min;
    let bot_travel = range.bot_max - range.bot_min;

    hud.clear();
    hud.section("Pulley \u{2014} pull 2, load moves 1");

    hud.raw(format!("Top (load):  {qt:+.4} m"));
    hud.raw(format!("Bot (pull):  {qb:+.4} m"));
    hud.raw(String::new());
    hud.raw(format!(
        "Top range:   {:.3} m  ({:+.3} to {:+.3})",
        top_travel, range.top_min, range.top_max
    ));
    hud.raw(format!(
        "Bot range:   {:.3} m  ({:+.3} to {:+.3})",
        bot_travel, range.bot_min, range.bot_max
    ));
    if top_travel > 0.005 {
        let ratio = bot_travel / top_travel;
        hud.raw(format!(
            "Pull/Load:   {ratio:.1}\u{00d7}  \u{2190} bot pulls {ratio:.1}\u{00d7} farther"
        ));
    } else {
        hud.raw("Pull/Load:   -- (warming up)".to_string());
    }
    hud.scalar("time", data.time, 1);
}

// ── Validation ────────────────────────────────────────────────────────────

#[derive(Default)]
struct DiagState {
    max_err: f64,
    reported: bool,
}

fn diagnostics(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    harness: Res<ValidationHarness>,
    mut state: Local<DiagState>,
) {
    let tid = model.tendon_id("pulley_tendon").expect("tendon");
    let length = data.ten_length[tid];

    let s1 = model.site_id("s1").expect("s1");
    let s2 = model.site_id("s2").expect("s2");
    let s3 = model.site_id("s3").expect("s3");
    let s4 = model.site_id("s4").expect("s4");
    let manual = (data.site_xpos[s1] - data.site_xpos[s2]).norm()
        + (data.site_xpos[s3] - data.site_xpos[s4]).norm() / 2.0;
    let err = (length - manual).abs();
    state.max_err = state.max_err.max(err);

    if !harness.reported() || state.reported {
        return;
    }
    state.reported = true;

    let checks = vec![Check {
        name: "L = branch1 + branch2/divisor (every frame)",
        pass: state.max_err < 1e-8,
        detail: format!("max |error| = {:.2e}", state.max_err),
    }];
    let _ = print_report("Pulley (t=10s)", &checks);
}
