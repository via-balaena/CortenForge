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

use bevy::prelude::*;
use sim_core::validation::{
    Check, EnergyConservationTracker, EnergyMonotonicityTracker, EquilibriumTracker, LimitTracker,
    PeriodTracker, QuaternionNormTracker, print_report,
};
use sim_core::{Data, Model};

use crate::camera::OrbitCamera;
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
        let title = format!("Validation Report (t={report_time:.0}s)");
        let _ = print_report(&title, &checks);
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
                TextColor(Color::srgb(0.0, 1.0, 0.0)),
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
}
