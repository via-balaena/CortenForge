//! Frame-by-frame deformed positions captured from a headless sim-soft run.
//!
//! [`Trajectory`] is a Bevy `Component` carried by the soft-mesh entity
//! alongside `Mesh3d` (enforced via `#[require(Mesh3d)]`). Each
//! `frames[i]` is a flat `Vec<f64>` in vertex-major + xyz-inner DOF
//! layout — same packing as
//! [`sim_soft::NewtonStep::x_final`](sim_soft::NewtonStep). For an
//! `n_vertex` mesh, `frames[i].len() == 3 * n_vertices`.
//!
//! # Capture path
//!
//! Per Q5 trajectory-replay (β) lock: solver runs headless,
//! [`Solver::replay_step`](sim_soft::Solver::replay_step) returns
//! [`NewtonStep`](sim_soft::NewtonStep) per step, the example collects
//! `step.x_final` into [`Trajectory::frames`] and spawns the soft-mesh
//! entity with `(Mesh3d, MeshMaterial3d, Transform, Trajectory)`. The
//! solver is NEVER invoked from inside Bevy's update loop.
//!
//! # Multi-soft-body
//!
//! v1 known scope (PR2 rows 12, 13, 14, 18 + PR3 row 20) is single-
//! soft-body; the Component shape lifts to multi-body for free — adding a
//! second soft body in a future arc spawns a second entity with its own
//! `(Mesh3d, Trajectory, ...)` bundle. No migration required.

use bevy::asset::Assets;
use bevy::mesh::{Mesh, Mesh3d};
use bevy::prelude::{Component, Query, Res, ResMut};
use cf_bevy_common::axis::UpAxis;

use crate::mesh::apply_soft_positions;

/// Captured trajectory of deformed positions, replayed by [`step_replay`].
#[derive(Component, Debug, Clone)]
#[require(Mesh3d)]
pub struct Trajectory {
    /// Frame-by-frame deformed positions. Each `frames[i]` is the
    /// flat-stride-3 `Vec<f64>` layout produced by sim-soft's solver
    /// (vertex-major + xyz-inner; `frames[i].len() == 3 * n_vertices`).
    pub frames: Vec<Vec<f64>>,
    /// Wall-clock seconds between consecutive frames. Replay maps
    /// `Time<Real>::elapsed_secs_f64() → frame_index = floor(elapsed / dt)`,
    /// clamped to the last frame at end of trajectory.
    pub dt: f64,
}

impl Trajectory {
    /// Index of the frame to display at `elapsed_secs` of wall-clock time
    /// since trajectory start. Clamps at `frames.len() - 1` past
    /// trajectory duration (last frame holds — no looping for v1).
    ///
    /// Returns `0` for an empty trajectory (defensive — `frames` is
    /// expected non-empty at consumer-construction time).
    #[must_use]
    // f64 → usize cast saturates to 0 for negative/NaN and to usize::MAX
    // for huge inputs; the subsequent `.min(...)` clamps to last frame in
    // both cases, so cast-truncation / sign-loss / precision-loss are all
    // intentional and bounded.
    #[allow(
        clippy::cast_possible_truncation,
        clippy::cast_sign_loss,
        clippy::cast_precision_loss
    )]
    pub fn frame_index_at(&self, elapsed_secs: f64) -> usize {
        // `elapsed / dt` is f64; floor + as-usize saturates to 0 for
        // negative / NaN inputs and to usize::MAX for huge ones. The
        // subsequent `.min(...)` clamps to last frame either way.
        let raw = (elapsed_secs / self.dt).floor() as usize;
        raw.min(self.frames.len().saturating_sub(1))
    }

    /// Total wall-clock duration of the trajectory.
    #[must_use]
    // usize → f64 cast: PR2 trajectory lengths are well below 2^53, so
    // precision loss from the frame-count cast is not realizable.
    #[allow(clippy::cast_precision_loss)]
    pub fn duration_secs(&self) -> f64 {
        self.frames.len() as f64 * self.dt
    }
}

/// Bevy system: per-soft-body-entity, advance the frame index under
/// `Time<Real>` and update the entity's `Mesh3d` POSITION + NORMAL
/// attributes via [`apply_soft_positions`].
///
/// Wired by [`crate::plugin::SoftBodyVisualPlugin`] into Bevy's `Update`
/// schedule; consumers wanting to interleave their own systems can call
/// it directly via the `prelude` re-export. Pause / scrub / loop controls
/// are out of scope for v1; promote to a custom replay-clock Resource
/// when a row demands them.
///
/// Skips entities whose `Mesh3d` handle has no live asset (e.g., during
/// despawn) or whose computed frame index falls outside the trajectory
/// (defensive — [`Trajectory::frame_index_at`] already clamps).
#[allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value
pub fn step_replay(
    mut meshes: ResMut<Assets<Mesh>>,
    query: Query<(&Mesh3d, &Trajectory)>,
    time: Res<bevy::time::Time<bevy::time::Real>>,
    up: Res<UpAxis>,
) {
    for (mesh3d, trajectory) in &query {
        let frame_index = trajectory.frame_index_at(time.elapsed_secs_f64());
        let Some(frame) = trajectory.frames.get(frame_index) else {
            continue;
        };
        let Some(mesh) = meshes.get_mut(&mesh3d.0) else {
            continue;
        };
        apply_soft_positions(mesh, frame, *up);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn three_frame_trajectory(dt: f64) -> Trajectory {
        Trajectory {
            frames: vec![
                vec![0.0, 0.0, 0.0],
                vec![1.0, 0.0, 0.0],
                vec![2.0, 0.0, 0.0],
            ],
            dt,
        }
    }

    /// At `elapsed = 0`, frame 0 is shown.
    #[test]
    fn frame_index_at_zero_returns_first_frame() {
        let t = three_frame_trajectory(0.1);
        assert_eq!(t.frame_index_at(0.0), 0);
    }

    /// Floor semantics: `elapsed = 0.05, dt = 0.1` → frame 0.
    #[test]
    fn frame_index_at_floors_to_lower_frame() {
        let t = three_frame_trajectory(0.1);
        assert_eq!(t.frame_index_at(0.05), 0);
    }

    /// At `elapsed = dt`, frame 1 is shown.
    #[test]
    fn frame_index_at_dt_advances_one_frame() {
        let t = three_frame_trajectory(0.1);
        assert_eq!(t.frame_index_at(0.1), 1);
    }

    /// Past trajectory duration, the last frame holds (clamp-not-loop per
    /// Q4 v1 lock).
    #[test]
    fn frame_index_at_clamps_past_duration() {
        let t = three_frame_trajectory(0.1);
        // duration = 3 * 0.1 = 0.3; well past should clamp to 2 (last index).
        assert_eq!(t.frame_index_at(10.0), 2);
    }

    /// Negative elapsed (defensive — should not happen under `Time<Real>`
    /// but the `as usize` saturation handles it cleanly).
    #[test]
    fn frame_index_at_negative_elapsed_returns_first_frame() {
        let t = three_frame_trajectory(0.1);
        assert_eq!(t.frame_index_at(-1.0), 0);
    }

    /// `duration_secs = n_frames * dt`.
    #[test]
    fn duration_secs_matches_frame_count_times_dt() {
        let t = three_frame_trajectory(0.05);
        assert!((t.duration_secs() - 0.15).abs() < 1e-12);
    }
}
