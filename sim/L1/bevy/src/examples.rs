//! Example-specific utilities.
//!
//! Helpers shared across CortenForge example programs. These are convenience
//! functions for demos and tutorials — not for production simulation code.
//!
//! # Validation Harness
//!
//! [`ValidationHarness`] replaces per-example boilerplate for physics validation.
//! Declare trackers, display functions, and report timing via a builder API, then
//! add [`validation_system`] to `PostUpdate`.
//!
//! # Tendon Visualization
//!
//! Color helpers and path-drawing utilities for tendon examples:
//! - [`tendon_color_bipolar`] — blue (−1) → green (0) → red (+1)
//! - [`tendon_color_ramp`] — green (0) → yellow (0.5) → red (1)
//! - [`draw_tendon_segments`] — wrap path as line segments + dot markers
//! - [`draw_sphere_arc`] — great-circle arc for sphere wrapping
//! - [`draw_cylinder_arc`] — helical geodesic for cylinder wrapping

use bevy::prelude::*;
use nalgebra::{Matrix3, Vector3};
use sim_core::validation::{
    Check, EnergyConservationTracker, EnergyMonotonicityTracker, EquilibriumTracker, LimitTracker,
    PeriodTracker, QuaternionNormTracker, print_report,
};
use sim_core::{Data, Model};

use crate::camera::OrbitCamera;
use crate::convert::vec3_from_vector;
use crate::model_data::{PhysicsData, PhysicsModel};

/// Timer for periodic diagnostic printing in examples.
///
/// Tracks the last physics time at which diagnostics were printed.
/// Compare against `data.time` to print at a fixed interval:
///
/// ```ignore
/// if data.time - timer.last > 1.0 {
///     timer.last = data.time;
///     println!("t={:.1}s  ...", data.time);
/// }
/// ```
#[derive(Resource, Default)]
pub struct DiagTimer {
    /// Last physics time at which diagnostics were printed.
    pub last: f64,
}

// ── Physics Delay ─────────────────────────────────────────────────────────

/// Startup delay: freezes physics for a few seconds so the user can see the
/// initial configuration before simulation begins.
///
/// Insert as a resource with `PhysicsDelay::new(seconds)`, add
/// [`tick_physics_delay`] and gate your stepping with
/// [`physics_delay_elapsed`]:
///
/// ```ignore
/// .insert_resource(PhysicsDelay::new(2.0))
/// .add_systems(Update, (
///     tick_physics_delay,
///     step_physics_realtime.run_if(physics_delay_elapsed),
/// ).chain())
/// ```
#[derive(Resource)]
pub struct PhysicsDelay {
    elapsed: f32,
    duration: f32,
}

impl PhysicsDelay {
    /// Create a new delay that freezes physics for `duration` seconds.
    #[must_use]
    pub fn new(duration: f32) -> Self {
        Self {
            elapsed: 0.0,
            duration,
        }
    }
}

/// Bevy system: tick the [`PhysicsDelay`] timer each frame.
#[allow(clippy::needless_pass_by_value)]
pub fn tick_physics_delay(time: Res<Time>, mut delay: ResMut<PhysicsDelay>) {
    delay.elapsed += time.delta_secs();
}

/// Run condition: returns `true` once the [`PhysicsDelay`] has elapsed.
#[allow(clippy::needless_pass_by_value, clippy::must_use_candidate)]
pub fn physics_delay_elapsed(delay: Res<PhysicsDelay>) -> bool {
    delay.elapsed >= delay.duration
}

/// Spawn a standard example camera and two-point lighting rig.
///
/// Creates an [`OrbitCamera`] aimed at `target` with the given `distance`,
/// `azimuth`, and `elevation`, plus ambient light, a key directional light
/// (with shadows), and a fill directional light (no shadows).
///
/// `target` is in **Bevy coordinates (Y-up)**. Use [`physics_pos()`](crate::convert::physics_pos)
/// at the call site when converting from MJCF (Z-up) coordinates.
///
/// Does NOT spawn a ground plane — use [`ExampleScene`](crate::scene::ExampleScene)
/// if you need one.
pub fn spawn_example_camera(
    commands: &mut Commands,
    target: Vec3,
    distance: f32,
    azimuth: f32,
    elevation: f32,
) {
    let mut orbit = OrbitCamera::new()
        .with_target(target)
        .with_angles(azimuth, elevation);
    orbit.max_distance = 20.0;
    orbit.distance = distance;
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
}

// ── Validation Harness ─────────────────────────────────────────────────────

/// Enum wrapping the 6 tracker types from `sim_core::validation`.
///
/// Each variant stores the tracker, its sampling closure, and the
/// tolerance/expected values needed for the final check.
enum TrackerKind {
    Period {
        tracker: PeriodTracker,
        sampler: fn(&Model, &Data) -> (f64, f64),
        expected: f64,
        tolerance_pct: f64,
    },
    EnergyConservation {
        tracker: EnergyConservationTracker,
        tolerance_pct: f64,
    },
    EnergyMonotonicity {
        tracker: EnergyMonotonicityTracker,
        tolerance_j: f64,
    },
    Limit {
        tracker: LimitTracker,
        sampler: fn(&Model, &Data) -> (f64, f64, f64),
        tolerance: f64,
    },
    QuatNorm {
        tracker: QuaternionNormTracker,
        sampler: fn(&Model, &Data) -> (f64, f64, f64, f64),
        tolerance: f64,
    },
    Equilibrium {
        tracker: EquilibriumTracker,
        sampler: fn(&Model, &Data) -> (f64, f64),
        expected: f64,
        tolerance_pct: f64,
    },
}

/// A single validation entry: tracker + metadata.
struct ValidationEntry {
    name: &'static str,
    tracker: TrackerKind,
    skip_until: f64,
}

/// Resource: collects validation entries, handles timing, prints reports.
///
/// Builder API for declarative validation setup in examples:
///
/// ```ignore
/// use sim_bevy::examples::{ValidationHarness, validation_system};
///
/// ValidationHarness::new()
///     .report_at(15.0)
///     .print_every(1.0)
///     .display(|m, d| {
///         let angle = d.joint_qpos(m, 0)[0];
///         format!("angle={:+6.1}\u{00B0}  E={:.4}J",
///             angle.to_degrees(),
///             d.energy_kinetic + d.energy_potential)
///     })
///     .track_energy(0.5)
///     .track_period("Period", |m, d| (d.joint_qpos(m, 0)[0], d.time), 1.47, 2.0)
/// ```
#[derive(Resource)]
pub struct ValidationHarness {
    entries: Vec<ValidationEntry>,
    report_time: f64,
    print_interval: f64,
    reported: bool,
    custom_reported: bool,
    last_print: f64,
    display: Option<fn(&Model, &Data) -> String>,
}

impl ValidationHarness {
    /// Create a new empty harness with default settings.
    ///
    /// Defaults: report at 15.0s, print every 1.0s, no display function.
    #[must_use]
    pub fn new() -> Self {
        Self {
            entries: Vec::new(),
            report_time: 15.0,
            print_interval: 1.0,
            reported: false,
            custom_reported: false,
            last_print: 0.0,
            display: None,
        }
    }

    /// Set the simulation time at which the final validation report prints.
    #[must_use]
    pub fn report_at(mut self, time: f64) -> Self {
        self.report_time = time;
        self
    }

    /// Set the interval (sim seconds) between periodic console output lines.
    #[must_use]
    pub fn print_every(mut self, interval: f64) -> Self {
        self.print_interval = interval;
        self
    }

    /// Set the periodic console display function.
    ///
    /// Called once per `print_interval` to produce a status line. The harness
    /// prepends `t={time:5.1}s  ` automatically.
    #[must_use]
    pub fn display(mut self, f: fn(&Model, &Data) -> String) -> Self {
        self.display = Some(f);
        self
    }

    /// Skip sampling for the most recently added entry until `time` seconds.
    ///
    /// Useful for constraint-settling transients (e.g. skip first 1s after
    /// an infeasible initial state is projected onto the feasible set).
    #[must_use]
    pub fn skip_until(mut self, time: f64) -> Self {
        if let Some(entry) = self.entries.last_mut() {
            entry.skip_until = time;
        }
        self
    }

    /// Track oscillation period. `sampler` returns `(signal_value, sim_time)`.
    #[must_use]
    pub fn track_period(
        mut self,
        name: &'static str,
        sampler: fn(&Model, &Data) -> (f64, f64),
        expected: f64,
        tolerance_pct: f64,
    ) -> Self {
        self.entries.push(ValidationEntry {
            name,
            tracker: TrackerKind::Period {
                tracker: PeriodTracker::new(),
                sampler,
                expected,
                tolerance_pct,
            },
            skip_until: 0.0,
        });
        self
    }

    /// Track energy conservation (drift from initial value).
    ///
    /// Reads `data.energy_kinetic + data.energy_potential` automatically.
    #[must_use]
    pub fn track_energy(mut self, tolerance_pct: f64) -> Self {
        self.entries.push(ValidationEntry {
            name: "Energy conservation",
            tracker: TrackerKind::EnergyConservation {
                tracker: EnergyConservationTracker::new(),
                tolerance_pct,
            },
            skip_until: 0.0,
        });
        self
    }

    /// Track energy monotonic decrease (for damped systems).
    ///
    /// Reads `data.energy_kinetic + data.energy_potential` automatically.
    #[must_use]
    pub fn track_energy_monotonic(mut self, tolerance_j: f64) -> Self {
        self.entries.push(ValidationEntry {
            name: "Energy",
            tracker: TrackerKind::EnergyMonotonicity {
                tracker: EnergyMonotonicityTracker::new(),
                tolerance_j,
            },
            skip_until: 0.0,
        });
        self
    }

    /// Track joint/tendon limit enforcement. `sampler` returns `(value, lo, hi)`.
    #[must_use]
    pub fn track_limit(
        mut self,
        name: &'static str,
        sampler: fn(&Model, &Data) -> (f64, f64, f64),
        tolerance: f64,
    ) -> Self {
        self.entries.push(ValidationEntry {
            name,
            tracker: TrackerKind::Limit {
                tracker: LimitTracker::new(),
                sampler,
                tolerance,
            },
            skip_until: 0.0,
        });
        self
    }

    /// Track quaternion norm preservation. `sampler` returns `(w, x, y, z)`.
    #[must_use]
    pub fn track_quat_norm(
        mut self,
        name: &'static str,
        sampler: fn(&Model, &Data) -> (f64, f64, f64, f64),
        tolerance: f64,
    ) -> Self {
        self.entries.push(ValidationEntry {
            name,
            tracker: TrackerKind::QuatNorm {
                tracker: QuaternionNormTracker::new(),
                sampler,
                tolerance,
            },
            skip_until: 0.0,
        });
        self
    }

    /// Track equilibrium convergence. `sampler` returns `(value, sim_time)`.
    #[must_use]
    pub fn track_equilibrium(
        mut self,
        name: &'static str,
        sampler: fn(&Model, &Data) -> (f64, f64),
        expected: f64,
        tolerance_pct: f64,
        window: f64,
    ) -> Self {
        self.entries.push(ValidationEntry {
            name,
            tracker: TrackerKind::Equilibrium {
                tracker: EquilibriumTracker::new(window),
                sampler,
                expected,
                tolerance_pct,
            },
            skip_until: 0.0,
        });
        self
    }

    /// Whether the final validation report has been printed.
    #[must_use]
    pub fn reported(&self) -> bool {
        self.reported
    }

    /// Returns `true` exactly once after the harness report fires.
    ///
    /// Use this in custom diagnostics systems to run your own checks at
    /// report time without needing a separate `{ reported: bool }` resource:
    ///
    /// ```ignore
    /// fn my_diagnostics(mut harness: ResMut<ValidationHarness>, ...) {
    ///     if !harness.take_reported() { return; }
    ///     // ... custom checks, print_report, etc.
    /// }
    /// ```
    pub fn take_reported(&mut self) -> bool {
        if self.reported && !self.custom_reported {
            self.custom_reported = true;
            true
        } else {
            false
        }
    }
}

impl Default for ValidationHarness {
    fn default() -> Self {
        Self::new()
    }
}

/// Bevy system: sample all trackers, print periodic diagnostics, print report.
///
/// Add to `PostUpdate` in your example app:
///
/// ```ignore
/// .add_systems(PostUpdate, (sync_geom_transforms, validation_system))
/// ```
#[allow(clippy::needless_pass_by_value)]
pub fn validation_system(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    mut harness: ResMut<ValidationHarness>,
) {
    let time = data.time;
    let model_ref: &Model = &model;
    let data_ref: &Data = &data;

    // Phase 1: Sample each tracker
    for entry in &mut harness.entries {
        if time < entry.skip_until {
            continue;
        }
        match &mut entry.tracker {
            TrackerKind::Period {
                tracker, sampler, ..
            } => {
                let (val, t) = sampler(model_ref, data_ref);
                tracker.sample(val, t);
            }
            TrackerKind::EnergyConservation { tracker, .. } => {
                tracker.sample(data_ref.energy_kinetic + data_ref.energy_potential);
            }
            TrackerKind::EnergyMonotonicity { tracker, .. } => {
                tracker.sample(data_ref.energy_kinetic + data_ref.energy_potential);
            }
            TrackerKind::Limit {
                tracker, sampler, ..
            } => {
                let (val, lo, hi) = sampler(model_ref, data_ref);
                tracker.sample(val, lo, hi);
            }
            TrackerKind::QuatNorm {
                tracker, sampler, ..
            } => {
                let (qw, qx, qy, qz) = sampler(model_ref, data_ref);
                tracker.sample(qw, qx, qy, qz);
            }
            TrackerKind::Equilibrium {
                tracker, sampler, ..
            } => {
                let (val, t) = sampler(model_ref, data_ref);
                tracker.sample(val, t);
            }
        }
    }

    // Phase 2: Periodic console output
    if time - harness.last_print >= harness.print_interval {
        harness.last_print = time;
        if let Some(display_fn) = harness.display {
            println!("t={time:5.1}s  {}", display_fn(model_ref, data_ref));
        }
    }

    // Phase 3: Final validation report
    if time >= harness.report_time && !harness.reported {
        harness.reported = true;
        let report_time = harness.report_time;
        let checks: Vec<Check> = harness
            .entries
            .iter()
            .map(|entry| {
                let mut check = match &entry.tracker {
                    TrackerKind::Period {
                        tracker,
                        expected,
                        tolerance_pct,
                        ..
                    } => tracker.check(*expected, *tolerance_pct),
                    TrackerKind::EnergyConservation {
                        tracker,
                        tolerance_pct,
                    } => tracker.check(*tolerance_pct),
                    TrackerKind::EnergyMonotonicity {
                        tracker,
                        tolerance_j,
                    } => tracker.check(*tolerance_j),
                    TrackerKind::Limit {
                        tracker, tolerance, ..
                    } => tracker.check(*tolerance),
                    TrackerKind::QuatNorm {
                        tracker, tolerance, ..
                    } => tracker.check(*tolerance),
                    TrackerKind::Equilibrium {
                        tracker,
                        expected,
                        tolerance_pct,
                        ..
                    } => tracker.check(*expected, *tolerance_pct),
                };
                check.name = entry.name;
                check
            })
            .collect();
        if !checks.is_empty() {
            let title = format!("Validation Report (t={report_time:.0}s)");
            let _ = print_report(&title, &checks);
        }
    }
}

// ── Physics HUD ──────────────────────────────────────────────────────────────

/// A single line of HUD text.
pub struct HudLine {
    /// Label (left column).
    pub label: String,
    /// Formatted value (right column).
    pub value: String,
}

/// Resource: list of HUD lines to display this frame.
///
/// Clear at the start of your update system, then push lines. The
/// [`render_physics_hud`] system joins them into the on-screen overlay.
///
/// ```ignore
/// fn update_hud(model: Res<PhysicsModel>, data: Res<PhysicsData>, mut hud: ResMut<PhysicsHud>) {
///     hud.clear();
///     hud.section("Sensors");
///     hud.scalar("Clock", data.time, "{:.2}s");
/// }
/// ```
#[derive(Resource, Default)]
pub struct PhysicsHud {
    lines: Vec<HudLine>,
}

impl PhysicsHud {
    /// Clear all lines (call at start of each frame).
    pub fn clear(&mut self) {
        self.lines.clear();
    }

    /// Add a labeled scalar value.
    ///
    /// `fmt` is a format-style hint like `"{:.3}"` or `"{:.2}s"`. The value is
    /// formatted with the precision extracted from `fmt`, and the suffix (if any)
    /// is appended.
    pub fn scalar(&mut self, label: &str, value: f64, precision: usize) {
        self.lines.push(HudLine {
            label: label.to_string(),
            value: format!("{value:.precision$}"),
        });
    }

    /// Add a labeled 3D vector.
    pub fn vec3(&mut self, label: &str, v: &[f64], precision: usize) {
        let (x, y, z) = (v[0], v[1], v[2]);
        self.lines.push(HudLine {
            label: label.to_string(),
            value: format!("({x:+.precision$}, {y:+.precision$}, {z:+.precision$})"),
        });
    }

    /// Add a labeled quaternion `[w, x, y, z]`.
    pub fn quat(&mut self, label: &str, q: &[f64]) {
        self.lines.push(HudLine {
            label: label.to_string(),
            value: format!("({:+.4}, {:+.4}, {:+.4}, {:+.4})", q[0], q[1], q[2], q[3],),
        });
    }

    /// Add a section header.
    pub fn section(&mut self, title: &str) {
        self.lines.push(HudLine {
            label: format!("── {title} ──"),
            value: String::new(),
        });
    }

    /// Add a raw pre-formatted line.
    pub fn raw(&mut self, text: String) {
        self.lines.push(HudLine {
            label: text,
            value: String::new(),
        });
    }
}

/// Marker component for the HUD text entity.
#[derive(Component)]
pub struct HudText;

/// Spawn the HUD overlay: a dark semi-transparent panel with monospace text.
///
/// Call once in your `Startup` system. Pair with [`render_physics_hud`] in
/// `PostUpdate` and init [`PhysicsHud`] as a resource.
pub fn spawn_physics_hud(commands: &mut Commands) {
    commands
        .spawn((
            Node {
                position_type: PositionType::Absolute,
                top: Val::Px(10.0),
                left: Val::Px(10.0),
                padding: UiRect::all(Val::Px(8.0)),
                ..default()
            },
            BackgroundColor(Color::srgba(0.0, 0.0, 0.0, 0.7)),
            GlobalZIndex(999),
        ))
        .with_children(|parent| {
            parent.spawn((
                HudText,
                Text::new(""),
                TextFont {
                    font_size: 14.0,
                    ..default()
                },
                TextColor(Color::WHITE),
            ));
        });
}

/// Bevy system: render [`PhysicsHud`] lines as Node-based UI text.
///
/// Add to `PostUpdate`:
///
/// ```ignore
/// .add_systems(PostUpdate, (sync_geom_transforms, validation_system, render_physics_hud))
/// ```
#[allow(clippy::needless_pass_by_value)]
pub fn render_physics_hud(
    hud: Res<PhysicsHud>,
    config: Option<Res<crate::resources::ViewerConfig>>,
    mut query: Query<(&mut Text, &mut Visibility), With<HudText>>,
) {
    let show = config.as_ref().is_none_or(|c| c.show_hud);
    for (mut text, mut vis) in &mut query {
        if !show {
            *vis = Visibility::Hidden;
            continue;
        }
        *vis = Visibility::Inherited;
        let content: String = hud
            .lines
            .iter()
            .map(|line| {
                if line.value.is_empty() {
                    line.label.clone()
                } else {
                    format!("{}: {}", line.label, line.value)
                }
            })
            .collect::<Vec<_>>()
            .join("\n");
        **text = content;
    }
}

// ── Tendon Visualization Utilities ──────────────────────────────────────────

// ── Sensor Helpers ─────────────────────────────────────────────────────────

/// Read a 3D sensor by name, returning `[x, y, z]`.
///
/// Returns `[0, 0, 0]` if the sensor is not found or has fewer than 3
/// elements. Useful for `FrameLinVel`, `FrameAngVel`, `Force`, `Torque`,
/// `SubtreeLinVel`, `SubtreeAngMom`, and other 3D sensor types.
#[must_use]
pub fn sensor_vec3(data: &Data, model: &Model, name: &str) -> [f64; 3] {
    if let Some(id) = model.sensor_id(name) {
        let s = data.sensor_data(model, id);
        if s.len() >= 3 {
            return [s[0], s[1], s[2]];
        }
    }
    [0.0, 0.0, 0.0]
}

/// Euclidean magnitude of a 3-element array.
#[must_use]
pub fn vec3_magnitude(v: &[f64; 3]) -> f64 {
    v[0].hypot(v[1]).hypot(v[2])
}

// ── Tendon Visualization ──────────────────────────────────────────────────

/// Bipolar tendon color: blue (−1) → green (0) → red (+1).
///
/// Use for quantities that span positive and negative:
/// - Elongation from rest: `t = (length - rest) / saturation`
/// - Velocity: `t = velocity / saturation`
///
/// `t` is clamped to [−1, 1] internally.
#[must_use]
pub fn tendon_color_bipolar(t: f32) -> Color {
    let t = t.clamp(-1.0, 1.0);
    if t >= 0.0 {
        Color::srgb(t, 0.8 * (1.0 - t), 0.0)
    } else {
        let s = -t;
        Color::srgb(0.0, 0.8 * (1.0 - s), s)
    }
}

/// Ramp tendon color: green (0) → yellow (0.5) → red (1).
///
/// Use for quantities normalized to [0, 1]:
/// - Proximity to limit: `t = length / limit`
/// - Self-calibrating range: `t = (length - min) / (max - min)`
///
/// `t` is clamped to [0, 1] internally.
#[must_use]
pub fn tendon_color_ramp(t: f32) -> Color {
    let t = t.clamp(0.0, 1.0);
    if t < 0.5 {
        let s = t * 2.0;
        Color::srgb(s, 0.8, 0.0)
    } else {
        let s = (t - 0.5) * 2.0;
        Color::srgb(1.0, 0.8 * (1.0 - s), 0.0)
    }
}

/// Draw a spatial tendon's wrap path as line segments with dot markers.
///
/// Reads `wrap_xpos` for the given tendon and draws straight-line segments
/// between consecutive points, plus small spheres at each point.
/// Coordinates are converted from physics (Z-up) to Bevy (Y-up).
pub fn draw_tendon_segments(
    gizmos: &mut Gizmos,
    data: &Data,
    tendon_id: usize,
    color: Color,
    dot_radius: f32,
) {
    let adr = data.ten_wrapadr[tendon_id];
    let num = data.ten_wrapnum[tendon_id];
    if num < 2 {
        return;
    }
    for i in 0..num - 1 {
        let start = vec3_from_vector(&data.wrap_xpos[adr + i]);
        let end = vec3_from_vector(&data.wrap_xpos[adr + i + 1]);
        gizmos.line(start, end, color);
    }
    for i in 0..num {
        let pos = vec3_from_vector(&data.wrap_xpos[adr + i]);
        gizmos.sphere(Isometry3d::from_translation(pos), dot_radius, color);
    }
}

/// Draw a great-circle arc between two tangent points on a sphere surface.
///
/// Interpolates `arc_segments` steps via spherical linear interpolation (slerp)
/// from `from` to `to` at distance `radius` from `center`. All positions are
/// in physics coordinates (Z-up) and converted to Bevy (Y-up) for drawing.
#[allow(clippy::cast_precision_loss)]
pub fn draw_sphere_arc(
    gizmos: &mut Gizmos,
    center: &Vector3<f64>,
    radius: f64,
    from: &Vector3<f64>,
    to: &Vector3<f64>,
    arc_segments: usize,
    color: Color,
) {
    let da = (from - center).normalize();
    let db = (to - center).normalize();
    let dot = da.dot(&db).clamp(-1.0, 1.0);
    let angle = dot.acos();

    if angle.abs() < 1e-8 {
        gizmos.line(vec3_from_vector(from), vec3_from_vector(to), color);
        return;
    }

    let sin_angle = angle.sin();
    let mut prev = vec3_from_vector(from);
    for seg in 1..=arc_segments {
        let frac = seg as f64 / arc_segments as f64;
        let sa = ((1.0 - frac) * angle).sin() / sin_angle;
        let sb = (frac * angle).sin() / sin_angle;
        let dir = da * sa + db * sb;
        let point = center + dir * radius;
        let cur = vec3_from_vector(&point);
        gizmos.line(prev, cur, color);
        prev = cur;
    }
}

/// Draw a helical geodesic arc between two tangent points on a cylinder surface.
///
/// Interpolates `arc_segments` steps along the cylinder's surface in its local
/// frame (XY = cross-section, Z = axis). `rotation` is the cylinder's 3×3
/// orientation matrix (`geom_xmat`). All positions are in physics coordinates
/// (Z-up) and converted to Bevy (Y-up) for drawing.
#[allow(clippy::cast_precision_loss, clippy::too_many_arguments)]
pub fn draw_cylinder_arc(
    gizmos: &mut Gizmos,
    center: &Vector3<f64>,
    rotation: &Matrix3<f64>,
    radius: f64,
    from: &Vector3<f64>,
    to: &Vector3<f64>,
    arc_segments: usize,
    color: Color,
) {
    // Transform tangent points to cylinder-local frame
    let la = rotation.transpose() * (from - center);
    let lb = rotation.transpose() * (to - center);

    // In local frame: XY is the cross-section, Z is the axis
    let angle_a = la.y.atan2(la.x);
    let angle_b = lb.y.atan2(lb.x);

    // Choose the shorter arc direction
    let mut delta = angle_b - angle_a;
    if delta > std::f64::consts::PI {
        delta -= 2.0 * std::f64::consts::PI;
    } else if delta < -std::f64::consts::PI {
        delta += 2.0 * std::f64::consts::PI;
    }

    let mut prev = vec3_from_vector(from);
    for seg in 1..=arc_segments {
        let frac = seg as f64 / arc_segments as f64;
        let angle = angle_a + delta * frac;
        let z = la.z + (lb.z - la.z) * frac;
        let local = Vector3::new(radius * angle.cos(), radius * angle.sin(), z);
        let world = center + rotation * local;
        let cur = vec3_from_vector(&world);
        gizmos.line(prev, cur, color);
        prev = cur;
    }
}

// ── Sleep-State Color Coding ────────────────────────────────────────────────

/// Tags each geom entity with its owning body index.
///
/// Attach via [`spawn_model_geoms_with`](crate::model_data::spawn_model_geoms_with)
/// callback during setup:
///
/// ```ignore
/// spawn_model_geoms_with(
///     &mut commands, &mut meshes, &mut materials, &model, &data, &overrides,
///     |cmd, geom_id, _name| { cmd.insert(GeomBodyId(model.geom_body[geom_id])); },
/// );
/// ```
#[derive(Component)]
pub struct GeomBodyId(pub usize);

/// Pre-allocated material handles for sleep-state color coding.
///
/// Insert as a resource during setup. The [`update_sleep_colors`] system
/// reads these handles each frame to swap entity materials based on
/// `body_sleep_state`.
#[derive(Resource)]
pub struct SleepMaterials {
    /// Material for awake bodies.
    pub awake: Handle<StandardMaterial>,
    /// Material for sleeping bodies.
    pub asleep: Handle<StandardMaterial>,
}

/// Bevy system that color-codes geom entities by their body's sleep state.
///
/// Requires [`GeomBodyId`] on each geom entity and [`SleepMaterials`] as a
/// resource. Add to `PostUpdate` after `sync_geom_transforms`.
///
/// - `Awake` → `SleepMaterials::awake` (orange)
/// - `Asleep` → `SleepMaterials::asleep` (steel blue)
/// - `Static` → unchanged (keeps initial material, e.g. floor)
#[allow(clippy::needless_pass_by_value)] // Bevy system signature
pub fn update_sleep_colors(
    data: Res<PhysicsData>,
    mats: Res<SleepMaterials>,
    mut query: Query<(&GeomBodyId, &mut MeshMaterial3d<StandardMaterial>)>,
) {
    for (body_id, mut mat_handle) in &mut query {
        match data.0.body_sleep_state[body_id.0] {
            sim_core::SleepState::Awake => *mat_handle = MeshMaterial3d(mats.awake.clone()),
            sim_core::SleepState::Asleep => *mat_handle = MeshMaterial3d(mats.asleep.clone()),
            sim_core::SleepState::Static => {}
        }
    }
}

// ── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn harness_defaults() {
        let h = ValidationHarness::new();
        assert_eq!(h.entries.len(), 0);
        assert!((h.report_time - 15.0).abs() < f64::EPSILON);
        assert!((h.print_interval - 1.0).abs() < f64::EPSILON);
        assert!(!h.reported);
        assert!(h.display.is_none());
    }

    #[test]
    fn harness_builder_settings() {
        let h = ValidationHarness::new().report_at(30.0).print_every(2.0);
        assert!((h.report_time - 30.0).abs() < f64::EPSILON);
        assert!((h.print_interval - 2.0).abs() < f64::EPSILON);
    }

    #[test]
    fn harness_track_energy() {
        let h = ValidationHarness::new().track_energy(0.5);
        assert_eq!(h.entries.len(), 1);
        assert_eq!(h.entries[0].name, "Energy conservation");
        assert!(h.entries[0].skip_until.abs() < f64::EPSILON);
    }

    #[test]
    fn harness_track_energy_monotonic() {
        let h = ValidationHarness::new().track_energy_monotonic(1e-6);
        assert_eq!(h.entries.len(), 1);
        assert_eq!(h.entries[0].name, "Energy");
    }

    #[test]
    fn harness_skip_until() {
        let h = ValidationHarness::new()
            .track_energy(0.5)
            .track_energy_monotonic(1e-3)
            .skip_until(1.0);
        assert_eq!(h.entries.len(), 2);
        assert!(h.entries[0].skip_until.abs() < f64::EPSILON);
        assert!((h.entries[1].skip_until - 1.0).abs() < f64::EPSILON);
    }

    #[test]
    fn harness_multiple_trackers() {
        fn sampler_2(_m: &Model, _d: &Data) -> (f64, f64) {
            (0.0, 0.0)
        }
        fn sampler_3(_m: &Model, _d: &Data) -> (f64, f64, f64) {
            (0.0, 0.0, 0.0)
        }
        fn sampler_4(_m: &Model, _d: &Data) -> (f64, f64, f64, f64) {
            (0.0, 0.0, 0.0, 0.0)
        }

        let h = ValidationHarness::new()
            .track_period("Period", sampler_2, 1.0, 2.0)
            .track_energy(0.5)
            .track_energy_monotonic(1e-6)
            .track_limit("Limits", sampler_3, 0.001)
            .track_quat_norm("Quat norm", sampler_4, 1e-10)
            .track_equilibrium("Equilibrium", sampler_2, -0.245, 5.0, 5.0);

        assert_eq!(h.entries.len(), 6);
        assert_eq!(h.entries[0].name, "Period");
        assert_eq!(h.entries[1].name, "Energy conservation");
        assert_eq!(h.entries[2].name, "Energy");
        assert_eq!(h.entries[3].name, "Limits");
        assert_eq!(h.entries[4].name, "Quat norm");
        assert_eq!(h.entries[5].name, "Equilibrium");
    }

    #[test]
    fn harness_display_function() {
        fn my_display(_m: &Model, _d: &Data) -> String {
            "test".to_string()
        }
        let h = ValidationHarness::new().display(my_display);
        assert!(h.display.is_some());
    }

    #[test]
    fn harness_skip_until_no_entries() {
        // skip_until on empty harness is a no-op
        let h = ValidationHarness::new().skip_until(1.0);
        assert_eq!(h.entries.len(), 0);
    }

    // ── PhysicsHud tests ─────────────────────────────────────────────────

    #[test]
    fn hud_default_empty() {
        let hud = PhysicsHud::default();
        assert!(hud.lines.is_empty());
    }

    #[test]
    fn hud_scalar_formatting() {
        let mut hud = PhysicsHud::default();
        hud.scalar("time", 1.23456, 3);
        assert_eq!(hud.lines.len(), 1);
        assert_eq!(hud.lines[0].label, "time");
        assert_eq!(hud.lines[0].value, "1.235");
    }

    #[test]
    fn hud_vec3_formatting() {
        let mut hud = PhysicsHud::default();
        hud.vec3("accel", &[1.0, -2.5, 9.81], 2);
        assert_eq!(hud.lines.len(), 1);
        assert_eq!(hud.lines[0].label, "accel");
        assert_eq!(hud.lines[0].value, "(+1.00, -2.50, +9.81)");
    }

    #[test]
    fn hud_quat_formatting() {
        let mut hud = PhysicsHud::default();
        hud.quat("ori", &[1.0, 0.0, 0.0, 0.0]);
        assert_eq!(hud.lines.len(), 1);
        assert_eq!(hud.lines[0].label, "ori");
        assert_eq!(hud.lines[0].value, "(+1.0000, +0.0000, +0.0000, +0.0000)");
    }

    #[test]
    fn hud_section_header() {
        let mut hud = PhysicsHud::default();
        hud.section("Sensors");
        assert_eq!(hud.lines.len(), 1);
        assert_eq!(hud.lines[0].label, "── Sensors ──");
        assert!(hud.lines[0].value.is_empty());
    }

    #[test]
    fn hud_raw_line() {
        let mut hud = PhysicsHud::default();
        hud.raw("contact: yes".to_string());
        assert_eq!(hud.lines.len(), 1);
        assert_eq!(hud.lines[0].label, "contact: yes");
        assert!(hud.lines[0].value.is_empty());
    }

    #[test]
    fn hud_clear() {
        let mut hud = PhysicsHud::default();
        hud.scalar("a", 1.0, 1);
        hud.scalar("b", 2.0, 1);
        assert_eq!(hud.lines.len(), 2);
        hud.clear();
        assert!(hud.lines.is_empty());
    }

    #[test]
    fn hud_multi_line_composition() {
        let mut hud = PhysicsHud::default();
        hud.section("Test");
        hud.scalar("x", 1.5, 2);
        hud.raw("done".to_string());
        assert_eq!(hud.lines.len(), 3);
        // Section header has empty value
        assert!(hud.lines[0].value.is_empty());
        // Scalar has label + value
        assert_eq!(hud.lines[1].label, "x");
        assert_eq!(hud.lines[1].value, "1.50");
        // Raw has label only
        assert_eq!(hud.lines[2].label, "done");
    }

    // ── Tendon color tests ──────────────────────────────────────────────

    fn color_rgb(c: Color) -> (f32, f32, f32) {
        let srgba = c.to_srgba();
        (srgba.red, srgba.green, srgba.blue)
    }

    #[test]
    fn bipolar_green_at_zero() {
        let (r, g, b) = color_rgb(tendon_color_bipolar(0.0));
        assert!(r.abs() < 1e-4, "r={r}");
        assert!((g - 0.8).abs() < 1e-4, "g={g}");
        assert!(b.abs() < 1e-4, "b={b}");
    }

    #[test]
    fn bipolar_red_at_plus_one() {
        let (r, g, b) = color_rgb(tendon_color_bipolar(1.0));
        assert!((r - 1.0).abs() < 1e-4, "r={r}");
        assert!(g.abs() < 1e-4, "g={g}");
        assert!(b.abs() < 1e-4, "b={b}");
    }

    #[test]
    fn bipolar_blue_at_minus_one() {
        let (r, g, b) = color_rgb(tendon_color_bipolar(-1.0));
        assert!(r.abs() < 1e-4, "r={r}");
        assert!(g.abs() < 1e-4, "g={g}");
        assert!((b - 1.0).abs() < 1e-4, "b={b}");
    }

    #[test]
    fn bipolar_clamps_beyond_range() {
        let a = color_rgb(tendon_color_bipolar(5.0));
        let b = color_rgb(tendon_color_bipolar(1.0));
        assert!((a.0 - b.0).abs() < 1e-6);
        assert!((a.1 - b.1).abs() < 1e-6);
        assert!((a.2 - b.2).abs() < 1e-6);
    }

    #[test]
    fn ramp_green_at_zero() {
        let (r, g, b) = color_rgb(tendon_color_ramp(0.0));
        assert!(r.abs() < 1e-4, "r={r}");
        assert!((g - 0.8).abs() < 1e-4, "g={g}");
        assert!(b.abs() < 1e-4, "b={b}");
    }

    #[test]
    fn ramp_yellow_at_half() {
        let (r, g, b) = color_rgb(tendon_color_ramp(0.5));
        assert!((r - 1.0).abs() < 1e-4, "r={r}");
        assert!((g - 0.8).abs() < 1e-4, "g={g}");
        assert!(b.abs() < 1e-4, "b={b}");
    }

    #[test]
    fn ramp_red_at_one() {
        let (r, g, b) = color_rgb(tendon_color_ramp(1.0));
        assert!((r - 1.0).abs() < 1e-4, "r={r}");
        assert!(g.abs() < 1e-4, "g={g}");
        assert!(b.abs() < 1e-4, "b={b}");
    }

    #[test]
    fn ramp_clamps_beyond_range() {
        let a = color_rgb(tendon_color_ramp(-0.5));
        let b = color_rgb(tendon_color_ramp(0.0));
        assert!((a.0 - b.0).abs() < 1e-6);
        assert!((a.1 - b.1).abs() < 1e-6);
        assert!((a.2 - b.2).abs() < 1e-6);
    }
}
