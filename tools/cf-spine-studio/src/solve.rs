//! The two-phase on-demand FSU solve, split so a bad painting is caught cheaply.
//!
//! `Enter` in Design runs the **build phase** ([`scene::build_fsu`], ~5 s: tet-mesh + SDF
//! grids + `k_disc` probe + the disc-mesh fragmentation guards) on a background thread and,
//! on success, parks the built FSU in [`HeldBuildSlot`] and shows the conformed disc
//! (Preview). `S` in Preview then runs only the **capture phase** ([`scene::capture_scene`],
//! ~85 s: the moment ramp) on the held build. Splitting the phases means the expensive
//! ramp is only ever paid for a painting that already tet-meshed cleanly.
//!
//! Both phases run off the main thread so the window stays responsive (no macOS
//! beachball). [`CoupledFsu`](cf_fsu_model::CoupledFsu) is `Send` (so a build moves into
//! and out of a task freely) but **not `Sync`** (a `RefCell` scratch buffer), so a held
//! build cannot be a plain Bevy `Resource` — it lives in a `NonSend` slot instead.

use bevy::prelude::*;
use bevy::tasks::{AsyncComputeTaskPool, Task, futures_lite::future};
use cf_mesh_paint::prelude::PaintBody;
use mesh_loft::{WallCorrespondence, assemble_bushing, extract_patch, finalize_patch, flip_patch};
use mesh_repair::{RepairParams, repair_mesh};
use mesh_types::IndexedMesh;

use crate::overlays::Overlays;
use crate::render::{self, SourceMeshes};
use crate::replay::Flexion;
use crate::scene::{self, FsuScene, HeldBuild};
use crate::state::StudioState;

/// The in-flight build phase, if any (`Enter` → [`scene::build_fsu`]).
#[derive(Resource, Default)]
pub(crate) struct BuildTask {
    pending: Option<Task<Result<HeldBuild, String>>>,
}

/// The in-flight capture phase, if any (`S` → [`scene::capture_scene`]).
#[derive(Resource, Default)]
pub(crate) struct SolveTask {
    pending: Option<Task<Result<FsuScene, String>>>,
}

/// The built-but-not-captured FSU, held between the Design build (`Enter`) and the Preview
/// capture (`S`). A `NonSend` resource because [`HeldBuild`] holds a non-`Sync`
/// `CoupledFsu`; it is parked on the main thread and moved into the capture task on `S`.
#[derive(Default)]
pub(crate) struct HeldBuildSlot(pub(crate) Option<HeldBuild>);

/// The most recent failure (loft / build / capture), shown in the Design panel.
#[derive(Resource, Default)]
pub(crate) struct SolveError(pub(crate) Option<String>);

/// `Enter` in Design lofts the two painted endplate patches into the disc, then dispatches
/// the ~5 s build phase to the `AsyncComputeTaskPool` and enters Building. Ignored while a
/// build is in flight; if a patch is unpainted it surfaces a message and stays in Design.
/// The meshes are cloned into the task so the ECS world isn't borrowed across the compute.
#[allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value.
pub(crate) fn start_build(
    keys: Res<ButtonInput<KeyCode>>,
    bodies: Query<&PaintBody>,
    sources: Res<SourceMeshes>,
    mut task: ResMut<BuildTask>,
    mut error: ResMut<SolveError>,
    mut next: ResMut<NextState<StudioState>>,
) {
    if !keys.just_pressed(KeyCode::Enter) || task.pending.is_some() {
        return;
    }
    // Loft runs on the main thread (it reads the `PaintBody` query, which can't cross into
    // the task), so catch a panic here too — a malformed bushing must route back to Design,
    // not crash the app.
    let lofted =
        std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| loft_painted_disc(&bodies)));
    let disc = match lofted {
        Ok(Ok(disc)) => disc,
        Ok(Err(msg)) => {
            error.0 = Some(msg);
            return; // stay in Design
        }
        Err(_) => {
            error.0 = Some(
                "lofting the painted patches failed — the painted regions produced an invalid \
                 bushing; try repainting fuller, flatter patches"
                    .to_string(),
            );
            return;
        }
    };
    error.0 = None;
    let (l4, l5) = (sources.l4.clone(), sources.l5.clone());
    let pool = AsyncComputeTaskPool::get();
    task.pending = Some(pool.spawn(async move {
        // The build's `k_disc` probe drives one FEM solve, which fail-closes with a PANIC on
        // a disc geometry it rejects (an over-distorted tet). Catch it so a bad disc routes
        // back to Design instead of re-raising on the main thread when the task is polled.
        match std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            scene::build_fsu(&l4, &l5, disc)
        })) {
            Ok(result) => result.map_err(|e| e.to_string()),
            Err(_) => Err(
                "the solver rejected this disc (a validity violation — the geometry is too \
                 distorted); paint fuller, flatter endplate patches on a single face"
                    .to_string(),
            ),
        }
    }));
    next.set(StudioState::Building);
}

/// Poll the in-flight build each frame while Building. On success, park the built FSU in
/// the held-build slot and enter Preview (the conformed disc is spawned `OnEnter(Preview)`);
/// on failure (unpaintable disc, fragmentation guard, or panic), surface the error and drop
/// back to Design.
pub(crate) fn poll_build(
    mut task: ResMut<BuildTask>,
    mut slot: NonSendMut<HeldBuildSlot>,
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
        Ok(build) => {
            slot.0 = Some(build);
            next.set(StudioState::Preview);
        }
        Err(msg) => {
            error.0 = Some(msg);
            next.set(StudioState::Design);
        }
    }
}

/// `S` in Preview dispatches the ~85 s capture phase on the held build and enters Solving.
/// Ignored while a capture is in flight or if the slot is somehow empty. The build is moved
/// out of the slot into the task (it drives the disc FEM), so returning to Preview requires
/// a fresh build — matching the tweak loop (repaint → `Enter` → `S`).
#[allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value.
pub(crate) fn start_capture(
    keys: Res<ButtonInput<KeyCode>>,
    sources: Res<SourceMeshes>,
    mut slot: NonSendMut<HeldBuildSlot>,
    mut task: ResMut<SolveTask>,
    mut next: ResMut<NextState<StudioState>>,
) {
    if !keys.just_pressed(KeyCode::KeyS) || task.pending.is_some() {
        return;
    }
    let Some(build) = slot.0.take() else {
        return; // no held build to capture
    };
    let (l4, l5) = (sources.l4.clone(), sources.l5.clone());
    let pool = AsyncComputeTaskPool::get();
    task.pending = Some(pool.spawn(async move {
        // The incremental ramp drives the disc FEM to each equilibrium angle; it fail-closes
        // with a PANIC on a step it rejects (a validity violation). Catch it so a bad disc
        // routes back to Design rather than crashing the app when the task is polled.
        match std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            scene::capture_scene(build, &l4, &l5)
        })) {
            Ok(result) => result.map_err(|e| e.to_string()),
            Err(_) => Err(
                "the solver rejected this disc during the ramp (a validity violation — the \
                 geometry is too distorted); paint fuller, flatter endplate patches"
                    .to_string(),
            ),
        }
    }));
    next.set(StudioState::Solving);
}

/// Entering Design is a clean slate: drop any held build (the user chose to repaint over it,
/// or a failed capture left it) and abandon any in-flight build/capture task. In the normal
/// paths the slot + tasks are already empty (`poll_build`/`poll_solve` take their task before
/// routing here, and `start_capture` takes the build), so this only bites the pathological
/// same-frame `S`+`Esc` race in Preview — where it drops the orphaned capture task that would
/// otherwise wedge `start_capture`'s `pending.is_some()` guard forever.
pub(crate) fn discard_pending_work(
    mut slot: NonSendMut<HeldBuildSlot>,
    mut build: ResMut<BuildTask>,
    mut capture: ResMut<SolveTask>,
) {
    slot.0 = None;
    build.pending = None;
    capture.pending = None;
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

/// Poll the in-flight capture each frame while Solving. On success, insert the
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
