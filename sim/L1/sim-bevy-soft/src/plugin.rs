//! `SoftBodyVisualPlugin` — wires [`step_replay`] +
//! [`reset_replay_on_keypress`] into Bevy's `Update` schedule and inits
//! the [`UpAxis`] resource if no consumer-supplied value is present.
//!
//! Does NOT add [`OrbitCameraPlugin`] — consumers add the camera plugin
//! themselves, mirroring the cf-bevy-common precedent (separation of
//! up-axis convention, camera input, and viz systems). Each example's
//! `main.rs` adds both `SoftBodyVisualPlugin` + `OrbitCameraPlugin`
//! together.
//!
//! [`OrbitCameraPlugin`]: cf_bevy_common::camera::OrbitCameraPlugin

use bevy::app::{App, Plugin, Update};
use bevy::ecs::schedule::IntoScheduleConfigs;
use cf_bevy_common::axis::UpAxis;

use crate::trajectory::{reset_replay_on_keypress, step_replay};

/// Plugin that wires soft-body trajectory replay into Bevy's `Update`
/// schedule.
///
/// - Inits the [`UpAxis`] resource (default `PlusZ`) if no consumer-
///   supplied value was inserted before plugin add-time.
/// - Adds [`reset_replay_on_keypress`] to `Update` ordered
///   `.before(step_replay)` — `KeyR` press clears every entity's
///   [`ReplayEpoch`] so the next tick animates from frame 0 again
///   (same-frame reset; no stale-end-state glitch).
/// - Adds [`step_replay`] to `Update` — per-soft-body-entity frame
///   advancement + position/normal write.
///
/// Consumers wanting to interleave their own systems between trajectory
/// advancement and the position update can omit this plugin and wire
/// `step_replay` + `reset_replay_on_keypress` (re-exported via
/// `prelude`) manually. Consumers not running `DefaultPlugins` need to
/// add Bevy's `InputPlugin` themselves for the keyboard reset to fire.
///
/// [`reset_replay_on_keypress`]: crate::trajectory::reset_replay_on_keypress
/// [`ReplayEpoch`]: crate::trajectory::ReplayEpoch
pub struct SoftBodyVisualPlugin;

impl Plugin for SoftBodyVisualPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<UpAxis>()
            .add_systems(Update, (reset_replay_on_keypress, step_replay).chain());
    }
}
