//! Frame-by-frame deformed positions captured from a headless sim-soft run.
//!
//! [`Trajectory`] is a Bevy `Component` carried by the soft-mesh entity
//! alongside `Mesh3d` and [`ReplayEpoch`] (both auto-added via
//! `#[require(...)]`). Each `frames[i]` is a flat `Vec<f64>` in
//! vertex-major + xyz-inner DOF layout — same packing as
//! [`sim_soft::NewtonStep::x_final`](sim_soft::NewtonStep). For an
//! `n_vertex` mesh, `frames[i].len() == 3 * n_vertices`.
//!
//! # Capture path
//!
//! The headless solver harness builds the trajectory:
//! [`Solver::replay_step`](sim_soft::Solver::replay_step) returns
//! [`NewtonStep`](sim_soft::NewtonStep) per step, the example collects
//! `step.x_final` into [`Trajectory::frames`] and spawns the soft-mesh
//! entity with `(Mesh3d, MeshMaterial3d, Transform, Trajectory)`. The
//! solver is NEVER invoked from inside Bevy's update loop.
//!
//! # Replay clock
//!
//! [`step_replay`] computes frame index against `Time<Real>` elapsed —
//! but Bevy's `DefaultPlugins` startup (winit window + render-pipeline
//! init, ~1-2 s on first run) ticks `Time<Real>` continuously before the
//! first frame becomes visible. To prevent that startup window from
//! consuming the trajectory's playback budget, each entity carries a
//! [`ReplayEpoch`] component (auto-added via [`Trajectory`]'s
//! `#[require(...)]`) that captures the wall-clock at the first
//! [`step_replay`] tick and freezes it; subsequent ticks compute frame
//! index against `(now - epoch)` rather than `now`. Each soft-body
//! entity gets its own epoch — multi-body scenes spawning at different
//! times each animate from their own `t = 0` rather than a shared global
//! clock.
//!
//! # Multi-soft-body
//!
//! Per-entity Component shape supports any number of soft bodies — each
//! spawns its own `(Mesh3d, Trajectory, ...)` bundle and `step_replay`
//! iterates the query. There is no global "current soft body" Resource
//! to migrate around.

use bevy::asset::Assets;
use bevy::input::ButtonInput;
use bevy::input::keyboard::KeyCode;
use bevy::mesh::{Mesh, Mesh3d};
use bevy::prelude::{Component, Query, Res, ResMut};
use cf_bevy_common::axis::UpAxis;

use crate::mesh::apply_soft_positions;

/// Per-entity replay-clock epoch — wall-clock `Time<Real>` reading at the
/// first [`step_replay`] tick against this entity. `None` until the first
/// replay, then captured-and-frozen so subsequent frame-index lookups are
/// relative to "first replay tick" rather than "app start".
///
/// Auto-added on [`Trajectory`] spawn via `#[require(ReplayEpoch)]`.
/// Consumers do not construct or read this directly under normal use —
/// it's load-bearing only inside [`step_replay`].
///
/// See the module-level "Replay clock" section for the full motivation
/// (`DefaultPlugins` startup window not consuming playback budget).
#[derive(Component, Default, Debug, Clone)]
pub struct ReplayEpoch {
    /// Wall-clock seconds at first replay tick. `None` before first tick.
    pub epoch_secs: Option<f64>,
}

/// Captured trajectory of deformed positions, replayed by [`step_replay`].
#[derive(Component, Debug, Clone)]
#[require(Mesh3d, ReplayEpoch)]
pub struct Trajectory {
    /// Frame-by-frame deformed positions. Each `frames[i]` is the
    /// flat-stride-3 `Vec<f64>` layout produced by sim-soft's solver
    /// (vertex-major + xyz-inner; `frames[i].len() == 3 * n_vertices`).
    pub frames: Vec<Vec<f64>>,
    /// Wall-clock seconds between consecutive frames. Replay maps
    /// `(now - epoch) → frame_index = floor(elapsed / dt)`, clamped to
    /// the last frame at end of trajectory. The `epoch` is captured at
    /// first replay tick (see [`ReplayEpoch`]) so `Time<Real>` ticks
    /// during `DefaultPlugins` startup do not consume playback budget.
    pub dt: f64,
}

impl Trajectory {
    /// Index of the frame to display at `elapsed_secs` of wall-clock time
    /// since trajectory start. Clamps at `frames.len() - 1` past
    /// trajectory duration — last frame holds, no looping. Returns `0`
    /// for an empty trajectory (defensive; consumers are expected to
    /// construct with non-empty `frames`).
    #[must_use]
    // f64 → usize cast saturates to 0 for negative/NaN inputs and to
    // usize::MAX for huge ones; `.min(...)` then clamps to the last
    // frame in either case, so the bounded-and-intentional truncation /
    // sign-loss / precision-loss covers the full input domain.
    #[allow(
        clippy::cast_possible_truncation,
        clippy::cast_sign_loss,
        clippy::cast_precision_loss
    )]
    pub fn frame_index_at(&self, elapsed_secs: f64) -> usize {
        let raw = (elapsed_secs / self.dt).floor() as usize;
        raw.min(self.frames.len().saturating_sub(1))
    }

    /// Total wall-clock duration of the trajectory.
    #[must_use]
    // usize → f64 cast: realistic trajectory lengths are well below 2^53,
    // so precision loss from the frame-count cast is not realizable.
    #[allow(clippy::cast_precision_loss)]
    pub fn duration_secs(&self) -> f64 {
        self.frames.len() as f64 * self.dt
    }
}

/// Bevy system: on `KeyR` press, clear every entity's [`ReplayEpoch`] so
/// the next [`step_replay`] tick captures a fresh epoch and the
/// trajectory animates from frame 0 again.
///
/// Wired by [`crate::plugin::SoftBodyVisualPlugin`] into Bevy's `Update`
/// schedule, ordered `.before(step_replay)` so the reset takes effect
/// on the same frame the key is pressed (no one-frame stale-end-state
/// glitch). Reads keyboard via the standard [`ButtonInput<KeyCode>`]
/// resource populated by Bevy's `InputPlugin` (part of `DefaultPlugins`);
/// consumers running with `MinimalPlugins` would need to add
/// `InputPlugin` themselves for the system to receive events.
///
/// `KeyR` was chosen because the orbit camera ([`cf_bevy_common::camera::OrbitCameraPlugin`])
/// is mouse-only — there is no keybind conflict. Pause / scrub are
/// out of scope (defer to a future row that needs them; pause requires
/// a mutable elapsed-accumulator on `ReplayEpoch`, more invasive than
/// reset's `set epoch to None`).
#[allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value
pub fn reset_replay_on_keypress(
    keys: Res<ButtonInput<KeyCode>>,
    mut query: Query<&mut ReplayEpoch>,
) {
    if keys.just_pressed(KeyCode::KeyR) {
        for mut epoch in &mut query {
            epoch.epoch_secs = None;
        }
    }
}

/// Bevy system: per-soft-body-entity, advance the frame index under the
/// per-entity epoch-relative `Time<Real>` reading and update the
/// entity's `Mesh3d` POSITION + NORMAL attributes via
/// [`apply_soft_positions`].
///
/// Wired by [`crate::plugin::SoftBodyVisualPlugin`] into Bevy's `Update`
/// schedule; consumers wanting to interleave their own systems can call
/// it directly via the `prelude` re-export. Replay clamps at trajectory
/// end (no looping); reset is wired separately by
/// [`reset_replay_on_keypress`] (KeyR clears the per-entity epoch so
/// the next tick captures a fresh one and animates from frame 0).
/// Pause / scrub controls are out of scope (would extend
/// [`ReplayEpoch`] with a paused-elapsed accumulator — defer to a
/// future row that needs them).
///
/// On the first tick against an entity, the entity's [`ReplayEpoch`] is
/// `None`; this system captures the current `Time<Real>` reading into
/// it. Subsequent ticks compute `elapsed = now - epoch_secs`, isolating
/// the trajectory's playback budget from `DefaultPlugins` startup time
/// (see module-level "Replay clock" section).
///
/// Skips entities whose `Mesh3d` handle has no live asset (e.g., during
/// despawn) or whose computed frame index falls outside the trajectory
/// (defensive — [`Trajectory::frame_index_at`] already clamps).
#[allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value
pub fn step_replay(
    mut meshes: ResMut<Assets<Mesh>>,
    mut query: Query<(&Mesh3d, &Trajectory, &mut ReplayEpoch)>,
    time: Res<bevy::time::Time<bevy::time::Real>>,
    up: Res<UpAxis>,
) {
    let now = time.elapsed_secs_f64();
    for (mesh3d, trajectory, mut epoch) in &mut query {
        let epoch_secs = *epoch.epoch_secs.get_or_insert(now);
        let elapsed = (now - epoch_secs).max(0.0);
        let frame_index = trajectory.frame_index_at(elapsed);
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
    // Test code uses `.expect("entity exists")` to surface state-extraction
    // failures loudly; the crate-level deny is for production paths.
    #![allow(clippy::expect_used)]
    // Same reason, for the one helper that reads a mesh attribute back: a
    // missing POSITION buffer means the fixture is wrong, and failing loudly
    // beats returning a default that a later assertion would silently compare
    // against.
    #![allow(clippy::panic)]

    use super::*;
    use bevy::ecs::world::World;

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

    /// Past trajectory duration, the last frame holds (clamp, not loop).
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

    /// `ReplayEpoch::default()` starts as `None` (un-captured); the
    /// epoch-capture happens inside [`step_replay`] on first tick. This
    /// ensures `Trajectory`'s `#[require(ReplayEpoch)]` auto-add lands
    /// the entity in the pre-first-tick state, not pre-loaded.
    #[test]
    fn replay_epoch_defaults_to_none() {
        let epoch = ReplayEpoch::default();
        assert!(epoch.epoch_secs.is_none());
    }

    /// `reset_replay_on_keypress` clears `epoch_secs` to `None` on KeyR
    /// press; verified by driving the system in a minimal Bevy app and
    /// asserting the post-press state.
    #[test]
    fn reset_replay_on_keypress_clears_epoch() {
        use bevy::ecs::schedule::{IntoScheduleConfigs, Schedule};
        use bevy::ecs::world::World;

        let mut world = World::new();
        world.init_resource::<ButtonInput<KeyCode>>();

        let entity = world
            .spawn(ReplayEpoch {
                epoch_secs: Some(1.234),
            })
            .id();

        // Press KeyR.
        world
            .resource_mut::<ButtonInput<KeyCode>>()
            .press(KeyCode::KeyR);

        let mut schedule = Schedule::default();
        schedule.add_systems(reset_replay_on_keypress.into_configs());
        schedule.run(&mut world);

        let epoch = world.get::<ReplayEpoch>(entity).expect("entity exists");
        assert!(
            epoch.epoch_secs.is_none(),
            "KeyR press should clear epoch_secs to None",
        );
    }

    /// A world wired exactly as `step_replay` expects: mesh assets, a real
    /// clock, an up-axis, and one soft-body entity.
    ///
    /// `UpAxis::PlusY` is the identity mapping (pinned by
    /// `build_soft_mesh_under_plus_y_is_identity` next door), so a position
    /// read back here reflects the REPLAY choice and not an axis conversion —
    /// otherwise a frame-index bug and an axis bug would be indistinguishable.
    fn replay_world(trajectory: Trajectory) -> (World, bevy::ecs::entity::Entity) {
        use bevy::app::App;
        use bevy::asset::AssetPlugin;
        use bevy::prelude::AssetApp;

        // `Assets<Mesh>` needs the asset server behind it, so borrow the
        // resources a minimal App sets up rather than hand-rolling them.
        let mut app = App::new();
        app.add_plugins(AssetPlugin::default());
        app.init_asset::<Mesh>();
        let mut world = std::mem::take(app.world_mut());

        world.init_resource::<bevy::time::Time<bevy::time::Real>>();
        world.insert_resource(UpAxis::PlusY);

        let handle = world
            .resource_mut::<Assets<Mesh>>()
            .add(crate::mesh::build_soft_mesh(
                &tet_positions(),
                &tet_faces(),
                UpAxis::PlusY,
            ));

        let entity = world.spawn((Mesh3d(handle), trajectory)).id();
        (world, entity)
    }

    fn tet_positions() -> Vec<sim_soft::Vec3> {
        vec![
            sim_soft::Vec3::new(0.0, 0.0, 0.0),
            sim_soft::Vec3::new(1.0, 0.0, 0.0),
            sim_soft::Vec3::new(0.0, 1.0, 0.0),
            sim_soft::Vec3::new(0.0, 0.0, 1.0),
        ]
    }

    fn tet_faces() -> Vec<[sim_soft::VertexId; 3]> {
        vec![[1, 2, 3], [0, 3, 2], [0, 1, 3], [0, 2, 1]]
    }

    /// Frame `k` puts vertex 0 at `x = k`, so the x-coordinate written into
    /// the mesh names the frame `step_replay` chose.
    fn labelled_frames(n: usize) -> Vec<Vec<f64>> {
        (0..n)
            .map(|k| {
                #[allow(clippy::cast_precision_loss)]
                let x = k as f64;
                vec![x, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            })
            .collect()
    }

    fn run_step_replay(world: &mut World) {
        use bevy::ecs::schedule::{IntoScheduleConfigs, Schedule};
        let mut schedule = Schedule::default();
        schedule.add_systems(step_replay.into_configs());
        schedule.run(world);
    }

    /// Which frame `step_replay` last wrote, read back off the mesh.
    fn displayed_frame(world: &World, entity: bevy::ecs::entity::Entity) -> f64 {
        use bevy::mesh::VertexAttributeValues;
        let handle = &world.get::<Mesh3d>(entity).expect("entity exists").0;
        let mesh = world
            .resource::<Assets<Mesh>>()
            .get(handle)
            .expect("mesh asset exists");
        match mesh.attribute(Mesh::ATTRIBUTE_POSITION) {
            Some(VertexAttributeValues::Float32x3(v)) => f64::from(v[0][0]),
            _ => panic!("mesh has no position attribute"),
        }
    }

    fn advance(world: &mut World, secs: f64) {
        world
            .resource_mut::<bevy::time::Time<bevy::time::Real>>()
            .advance_by(std::time::Duration::from_secs_f64(secs));
    }

    /// First tick captures the epoch rather than leaving it `None`.
    #[test]
    fn step_replay_captures_the_epoch_on_first_tick() {
        let (mut world, entity) = replay_world(Trajectory {
            frames: labelled_frames(3),
            dt: 0.1,
        });
        advance(&mut world, 5.0);
        run_step_replay(&mut world);

        let epoch = world.get::<ReplayEpoch>(entity).expect("entity exists");
        assert_eq!(
            epoch.epoch_secs,
            Some(5.0),
            "the first tick must freeze the current clock reading, not stay None"
        );
    }

    /// ★★ The module's central claim: replay is measured from the FIRST TICK,
    /// not from app start.
    ///
    /// `DefaultPlugins` startup (winit + render init) ticks `Time<Real>` for
    /// ~1-2 s before anything is visible. Here the clock is already at 5 s —
    /// far past this trajectory's 0.3 s duration — when replay begins. Reading
    /// the clock absolutely would compute frame 50, clamp to the last frame,
    /// and the viewer would show a finished deformation having animated
    /// nothing. Epoch-relative, it must start at frame 0.
    #[test]
    fn step_replay_starts_at_frame_zero_however_late_the_first_tick_is() {
        let (mut world, entity) = replay_world(Trajectory {
            frames: labelled_frames(3),
            dt: 0.1,
        });
        advance(&mut world, 5.0); // startup burned 5 s of Time<Real>
        run_step_replay(&mut world);

        assert!(
            (displayed_frame(&world, entity) - 0.0).abs() < 1e-9,
            "a late first tick must still start playback at frame 0; reading the \
             clock absolutely would clamp straight to the last frame"
        );
    }

    /// Once the epoch is frozen, later ticks advance against it.
    #[test]
    fn step_replay_advances_frames_against_the_frozen_epoch() {
        let (mut world, entity) = replay_world(Trajectory {
            frames: labelled_frames(3),
            dt: 0.1,
        });
        advance(&mut world, 5.0);
        run_step_replay(&mut world);
        advance(&mut world, 0.2); // 0.2 s past the epoch ⇒ frame 2
        run_step_replay(&mut world);

        assert!(
            (displayed_frame(&world, entity) - 2.0).abs() < 1e-9,
            "elapsed must be measured from the epoch (0.2 s ⇒ frame 2), not from \
             the absolute clock (5.2 s)"
        );
    }

    /// Past the end, the last frame holds — replay clamps rather than looping.
    #[test]
    fn step_replay_holds_the_last_frame_past_the_end() {
        let (mut world, entity) = replay_world(Trajectory {
            frames: labelled_frames(3),
            dt: 0.1,
        });
        run_step_replay(&mut world);
        advance(&mut world, 60.0);
        run_step_replay(&mut world);

        assert!(
            (displayed_frame(&world, entity) - 2.0).abs() < 1e-9,
            "replay clamps at the final frame; a loop would wrap back to 0"
        );
    }

    /// An entity whose mesh asset is gone (mid-despawn) is skipped, not
    /// panicked on — the `meshes.get_mut` guard in `step_replay`.
    #[test]
    fn step_replay_skips_an_entity_whose_mesh_asset_is_gone() {
        let (mut world, entity) = replay_world(Trajectory {
            frames: labelled_frames(3),
            dt: 0.1,
        });
        let handle = world
            .get::<Mesh3d>(entity)
            .expect("entity exists")
            .0
            .clone();
        world.resource_mut::<Assets<Mesh>>().remove(&handle);

        run_step_replay(&mut world); // must not panic

        assert!(
            world
                .get::<ReplayEpoch>(entity)
                .expect("entity exists")
                .epoch_secs
                .is_some(),
            "the entity is still ticked (epoch captured) — only the mesh write is skipped"
        );
    }

    /// An empty trajectory is skipped rather than indexing out of bounds.
    /// `frame_index_at` returns 0 for it, and `frames.get(0)` is `None`.
    #[test]
    fn step_replay_skips_an_empty_trajectory() {
        let (mut world, entity) = replay_world(Trajectory {
            frames: Vec::new(),
            dt: 0.1,
        });
        run_step_replay(&mut world); // must not panic

        assert!(
            world
                .get::<ReplayEpoch>(entity)
                .expect("entity exists")
                .epoch_secs
                .is_some()
        );
    }

    /// Without KeyR press, `reset_replay_on_keypress` leaves the epoch
    /// untouched.
    #[test]
    fn reset_replay_on_keypress_without_press_is_noop() {
        use bevy::ecs::schedule::{IntoScheduleConfigs, Schedule};
        use bevy::ecs::world::World;

        let mut world = World::new();
        world.init_resource::<ButtonInput<KeyCode>>();

        let entity = world
            .spawn(ReplayEpoch {
                epoch_secs: Some(2.5),
            })
            .id();

        let mut schedule = Schedule::default();
        schedule.add_systems(reset_replay_on_keypress.into_configs());
        schedule.run(&mut world);

        let epoch = world.get::<ReplayEpoch>(entity).expect("entity exists");
        assert_eq!(epoch.epoch_secs, Some(2.5));
    }
}
