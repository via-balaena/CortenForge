//! The two-phase on-demand FSU solve, split so a bad painting is caught cheaply.
//!
//! `Enter` in Design runs the **build phase** ([`scene::build_fsu`], tet-mesh + SDF
//! grids + `k_disc` probe + the disc-mesh fragmentation guards) on a background thread and,
//! on success, parks the built FSU in [`HeldBuildSlot`] and shows the conformed disc
//! (Preview). `S` in Preview then runs only the **capture phase** ([`scene::capture_scene`],
//! the moment ramp) on the held build. Splitting the phases means the expensive
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
/// the build phase to the `AsyncComputeTaskPool` and enters Building. Ignored while a
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

/// `S` in Preview dispatches the capture phase on the held build and enters Solving.
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

/// Loft the two painted patches (L4 + L5) into the intervertebral disc.
/// `finalize_patch` keeps the largest component and seals interior holes; L4's
/// rim is flipped to face L5's, and arc-length correspondence distributes the
/// wall evenly around the convex rims.
///
/// # Errors
///
/// Three ways, all surfaced in the Design panel rather than panicking:
/// - either patch is unpainted;
/// - the patches loft to an empty surface;
/// - the explode-then-weld below merged **nothing**, which means the lofted
///   surface has no shared topology and would tet-mesh shattered.
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
    // Checked before the weld guard below so that guard's message can be taken
    // at face value: an empty loft also welds nothing, and reporting it as
    // "shattered" would blame the weld for a patch problem.
    if raw.faces.is_empty() {
        return Err(
            "the painted patches lofted to an empty disc surface — paint larger, \
             flatter regions on both bodies"
                .to_string(),
        );
    }
    // Rebuild clean shared topology before tet-meshing. The old paint-faces flow
    // did this implicitly by round-tripping through an STL: STL is triangle soup
    // (each face gets its own 3 verts), so the reload's `repair_mesh` re-welds
    // ALL verts by position. Repairing the loft's *existing* topology in place
    // does NOT rebuild it, and the raw loft tet-meshes into a shattered surface —
    // so we explode to soup here, then weld, matching the STL round-trip exactly.
    let mut disc = explode_to_soup(&raw);
    let summary = repair_mesh(&mut disc, &RepairParams::for_scans());

    // ★ The weld is load-bearing here, not hygiene, so its summary is checked
    // rather than discarded. `explode_to_soup` guarantees `vertices == 3 ·
    // faces`; if nothing merges, the surface has no shared topology at all and
    // tet-meshes into exactly the "shattered surface" the paragraph above says
    // this explode-then-weld exists to prevent — and it would reach the FSU
    // solve looking like an ordinary disc.
    //
    // ⚠ `vertices_welded == 0` is the unambiguous did-nothing case and cannot
    // false-fire: soup has three coincident copies of every shared vertex, so a
    // real surface always has something to merge. Deliberately not a ratio —
    // any threshold here would be invented rather than measured. The empty
    // loft, which also welds nothing, is rejected above so this message means
    // only what it says.
    //
    // ★ NOT redundant with `scene::build_fsu`'s fragmentation guards, which run
    // on the post-build tet boundary and look for an empty or blown-up surface.
    // An unwelded loft does not necessarily fragment: `cf-fsu-model` measured
    // the fixture's un-welded loft at ~40 % PHANTOM material
    // (`frame_fix_step0b_lofted_disc_phantom_diagnosis_fom`) and traced it to
    // topology rather than to the frame or a hole
    // (`frame_fix_step0c_lofted_fixture_vs_production_pipeline_fom`) — a seam
    // position that should carry one pseudo-normal instead carries several,
    // each accumulated from only some of its incident triangles. That inflates
    // the disc rather than shattering it, so it can pass both downstream checks
    // while being physically wrong — and the weld here is the only thing
    // standing between the Studio and that exact defect.
    if summary.vertices_welded == 0 {
        return Err(format!(
            "lofted disc failed to weld: {} vertices / {} faces went in as \
             triangle soup and none merged, so the surface has no shared \
             topology and would tet-mesh shattered",
            summary.initial_vertices, summary.initial_faces
        ));
    }
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

#[cfg(test)]
mod tests {
    use super::*;
    use mesh_types::Point3;

    /// A closed surface exploded to soup always has something to merge, so the
    /// weld guard in [`loft_painted_disc`] cannot false-fire on a real disc.
    /// This is the guard's *premise*, tested rather than assumed.
    #[test]
    fn exploding_a_closed_surface_to_soup_always_leaves_something_to_weld() {
        let tet = IndexedMesh::from_parts(
            vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
                Point3::new(0.0, 0.0, 1.0),
            ],
            vec![[0, 1, 2], [0, 1, 3], [0, 2, 3], [1, 2, 3]],
        );

        let mut soup = explode_to_soup(&tet);
        assert_eq!(
            soup.vertices.len(),
            3 * tet.faces.len(),
            "soup must carry one private vertex per face corner"
        );

        let summary = repair_mesh(&mut soup, &RepairParams::for_scans());
        assert!(
            summary.vertices_welded > 0,
            "a closed surface welded nothing — the guard's premise is false: {summary:?}"
        );
        assert!(
            summary.final_vertices < summary.initial_vertices,
            "welding did not reduce the vertex count: {summary:?}"
        );
    }

    /// ★ The negative control: the guard must be able to FIRE for the reason it
    /// claims. Two triangles that share no position are already "shattered" —
    /// welding merges nothing, which is exactly the state that would reach the
    /// FSU solve looking like an ordinary disc.
    #[test]
    fn a_surface_with_no_shared_positions_welds_nothing_and_would_trip_the_guard() {
        let disjoint = IndexedMesh::from_parts(
            vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
                Point3::new(50.0, 0.0, 0.0),
                Point3::new(51.0, 0.0, 0.0),
                Point3::new(50.0, 1.0, 0.0),
            ],
            vec![[0, 1, 2], [3, 4, 5]],
        );

        let mut soup = explode_to_soup(&disjoint);
        let summary = repair_mesh(&mut soup, &RepairParams::for_scans());
        assert_eq!(
            summary.vertices_welded, 0,
            "nothing should merge when no two corners coincide: {summary:?}"
        );
    }
}
