//! The on-demand FSU solve. `S` in Design dispatches the coupled-FSU assembly
//! ([`scene::build_from_meshes`]) to a background thread — it takes ~90 s in
//! release (tet-mesh + SDF grids + the moment-ramp capture) — so the window
//! stays responsive during the solve instead of showing the macOS beachball.
//! The poll picks up the result the frame it finishes and hands the replay +
//! overlay data to Simulate.

use bevy::prelude::*;
use bevy::tasks::{AsyncComputeTaskPool, Task, futures_lite::future};

use crate::overlays::Overlays;
use crate::render::{self, SourceMeshes};
use crate::replay::Flexion;
use crate::scene::{self, FsuScene};
use crate::state::StudioState;

/// The in-flight background solve, if any.
#[derive(Resource, Default)]
pub(crate) struct SolveTask {
    pending: Option<Task<Result<FsuScene, String>>>,
}

/// The most recent solve error (shown in the Design panel), if the last solve
/// failed.
#[derive(Resource, Default)]
pub(crate) struct SolveError(pub(crate) Option<String>);

/// `S` in Design dispatches the coupled-FSU assembly to the
/// `AsyncComputeTaskPool` and enters Solving. Ignored while a solve is already
/// in flight. The three source meshes are cloned into the task so the ECS
/// world isn't borrowed across the ~90 s compute.
#[allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value.
pub(crate) fn start_solve(
    keys: Res<ButtonInput<KeyCode>>,
    sources: Res<SourceMeshes>,
    mut task: ResMut<SolveTask>,
    mut error: ResMut<SolveError>,
    mut next: ResMut<NextState<StudioState>>,
) {
    if !keys.just_pressed(KeyCode::KeyS) || task.pending.is_some() {
        return;
    }
    error.0 = None;
    let (l4, l5, disc) = (sources.l4.clone(), sources.l5.clone(), sources.disc.clone());
    let pool = AsyncComputeTaskPool::get();
    task.pending =
        Some(pool.spawn(async move {
            scene::build_from_meshes(l4, l5, disc).map_err(|e| e.to_string())
        }));
    next.set(StudioState::Solving);
}

/// Poll the in-flight solve each frame while Solving. On success, insert the
/// replay + overlay resources and the disc surface, then enter Simulate; on
/// failure, surface the error and drop back to Design.
pub(crate) fn poll_solve(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut task: ResMut<SolveTask>,
    mut error: ResMut<SolveError>,
    mut next: ResMut<NextState<StudioState>>,
) {
    let Some(mut pending) = task.pending.take() else {
        return;
    };
    let Some(result) = future::block_on(future::poll_once(&mut pending)) else {
        task.pending = Some(pending); // still running
        return;
    };
    match result {
        Ok(fsu) => {
            let FsuScene {
                disc_surface,
                disc_node_weights,
                o4,
                o5,
                flexion,
                ligaments,
                warnings,
            } = fsu;
            let disc_rest = disc_surface.vertices.clone();
            commands.insert_resource(Flexion::new(flexion, disc_rest, disc_node_weights, o4, o5));
            commands.insert_resource(Overlays {
                ligaments,
                warnings,
            });
            // Spawn the disc here (not an OnEnter system) so it and its replay
            // resources land together — no cross-schedule timing gap.
            render::spawn_disc(&mut commands, &mut meshes, &mut materials, &disc_surface);
            next.set(StudioState::Simulate);
        }
        Err(msg) => {
            error.0 = Some(msg);
            next.set(StudioState::Design);
        }
    }
}
