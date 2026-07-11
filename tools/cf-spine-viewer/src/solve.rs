//! The on-demand FSU solve. `S` in Design dispatches the coupled-FSU assembly
//! ([`scene::build_from_meshes`]) to a background thread — it takes ~90 s in
//! release (tet-mesh + SDF grids + the moment-ramp capture) — so the window
//! stays responsive during the solve instead of showing the macOS beachball.
//! The poll picks up the result the frame it finishes and hands the replay +
//! overlay data to Simulate.

use bevy::prelude::*;
use bevy::tasks::{AsyncComputeTaskPool, Task, futures_lite::future};
use cf_mesh_paint::prelude::PaintBody;
use mesh_loft::{WallCorrespondence, assemble_bushing, extract_patch, finalize_patch, flip_patch};
use mesh_repair::{RepairParams, repair_mesh};
use mesh_types::IndexedMesh;

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

/// `S` in Design lofts the two painted endplate patches into the disc, then
/// dispatches the coupled-FSU assembly to the `AsyncComputeTaskPool` and enters
/// Solving. Ignored while a solve is in flight; if a patch is unpainted it
/// surfaces a message and stays in Design. The meshes are cloned into the task
/// so the ECS world isn't borrowed across the ~90 s compute.
#[allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value.
pub(crate) fn start_solve(
    keys: Res<ButtonInput<KeyCode>>,
    bodies: Query<&PaintBody>,
    sources: Res<SourceMeshes>,
    mut task: ResMut<SolveTask>,
    mut error: ResMut<SolveError>,
    mut next: ResMut<NextState<StudioState>>,
) {
    if !keys.just_pressed(KeyCode::KeyS) || task.pending.is_some() {
        return;
    }
    let disc = match loft_painted_disc(&bodies) {
        Ok(disc) => disc,
        Err(msg) => {
            error.0 = Some(msg);
            return; // stay in Design
        }
    };
    error.0 = None;
    let (l4, l5) = (sources.l4.clone(), sources.l5.clone());
    let pool = AsyncComputeTaskPool::get();
    task.pending = Some(pool.spawn(async move {
        // The FEM solve fail-closes with a PANIC on a disc geometry it rejects (a
        // validity violation — an over-distorted tet). Catch it here so a bad disc
        // routes back to Design; otherwise the panic re-raises on the main thread
        // when the task is polled and crashes the whole app.
        match std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            scene::build_from_meshes(l4, l5, disc)
        })) {
            Ok(result) => result.map_err(|e| e.to_string()),
            Err(_) => Err(
                "the solver rejected this disc (a validity violation — the geometry is \
                           too distorted); paint fuller, flatter endplate patches on a single face"
                    .to_string(),
            ),
        }
    }));
    next.set(StudioState::Solving);
}

/// Loft the two painted patches (L4 + L5) into the intervertebral disc, or an
/// error message if either is unpainted. `finalize_patch` keeps the largest
/// component and seals interior holes; L4's rim is flipped to face L5's, and
/// arc-length correspondence distributes the wall evenly around the convex rims.
fn loft_painted_disc(bodies: &Query<&PaintBody>) -> Result<IndexedMesh, String> {
    let mut l4 = None;
    let mut l5 = None;
    for body in bodies {
        if body.painted_count() < 3 {
            continue;
        }
        let faces: Vec<usize> = body.painted().iter().copied().collect();
        let patch = finalize_patch(&extract_patch(body.source(), &faces));
        match body.name() {
            "L4" => l4 = Some(patch),
            "L5" => l5 = Some(patch),
            _ => {}
        }
    }
    let (Some(l4), Some(l5)) = (l4, l5) else {
        return Err("paint a region on BOTH L4 and L5 first".to_string());
    };
    let top = flip_patch(&l4);
    let raw = assemble_bushing(&top, &l5, 1, WallCorrespondence::ArcLength).mesh;
    // Rebuild clean shared topology before tet-meshing. The old paint-faces flow
    // did this implicitly by round-tripping through an STL: STL is triangle soup
    // (each face gets its own 3 verts), so the reload's `repair_mesh` re-welds
    // ALL verts by position. Repairing the loft's *existing* topology in place
    // does NOT rebuild it, and the raw loft tet-meshes into a shattered surface —
    // so we explode to soup here, then weld, matching the STL round-trip exactly.
    let mut disc = explode_to_soup(&raw);
    let _ = repair_mesh(&mut disc, &RepairParams::for_scans());
    Ok(disc)
}

/// Explode a mesh into per-triangle soup (each face gets its own 3 vertices,
/// no sharing) — the STL representation. Welding this by position (via
/// `repair_mesh`) rebuilds clean shared topology from scratch.
fn explode_to_soup(mesh: &IndexedMesh) -> IndexedMesh {
    let mut vertices = Vec::with_capacity(mesh.faces.len() * 3);
    let mut faces = Vec::with_capacity(mesh.faces.len());
    for (i, &[a, b, c]) in mesh.faces.iter().enumerate() {
        vertices.push(mesh.vertices[a as usize]);
        vertices.push(mesh.vertices[b as usize]);
        vertices.push(mesh.vertices[c as usize]);
        let base = (i as u32) * 3;
        faces.push([base, base + 1, base + 2]);
    }
    IndexedMesh::from_parts(vertices, faces)
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
