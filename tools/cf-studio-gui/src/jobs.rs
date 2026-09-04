//! Background work, and the pollers that land its results on the app.
//!
//! Two ops are too slow for the main thread, and `Studio::busy` gates every
//! control that could clobber one while it runs:
//!
//! - **Simplify** (step 2) decimates the working scan, ~10–40 s.
//! - **the print export** (step 6) copies the mold package, hundreds of
//!   megabytes at 0.5 mm.

use std::path::{Path, PathBuf};

use bevy::prelude::*;
use bevy::tasks::{AsyncComputeTaskPool, Task, futures_lite::future};
use cf_studio_engine::{PrintExportReport, export_print_package, run_simplify};
use cf_studio_gui::{format_simplify_done, format_simplify_started};
use mesh_types::IndexedMesh;

use crate::dialogs::{DialogKind, PendingDialog};
use crate::edit::{EditControls, land_edit};
use crate::save::{save_into, settle};
use crate::scan::{ActiveScan, ScanEdit};
use crate::state::{PendingSave, Studio};

/// The running print export, if any.
#[derive(Resource, Default)]
pub(crate) struct PrintJob(Option<Task<Result<PrintExportReport, String>>>);

/// Route a resolved file dialog to the work it was opened for.
pub(crate) fn poll_dialogs(
    mut dialog: ResMut<PendingDialog>,
    mut studio: ResMut<Studio>,
    mut job: ResMut<PrintJob>,
    mut scan: ResMut<ScanEdit>,
) {
    let Some((kind, picked)) = dialog.poll() else {
        return;
    };
    match kind {
        DialogKind::ScanFile => {
            let Some(path) = picked else { return };
            // Both reads run before either commit, so the project and the
            // viewport cannot end up disagreeing about which scan is loaded:
            // `ActiveScan::load` only reads, and `record_scan` records nothing
            // unless its own read of the same file succeeded.
            let outcome = match ActiveScan::load(&path) {
                Err(message) => Err(message),
                Ok(active) => {
                    let recorded = studio.record_scan(&path);
                    if recorded.is_ok() {
                        scan.set(active);
                    }
                    recorded
                }
            };
            studio.message = Some(outcome);
        }
        DialogKind::PrintDest => {
            // `None` is a cancel, which is a complete outcome: leave the app
            // exactly as it was, with no message.
            let Some(dest) = picked else { return };
            let Some(molds) = studio.project.molds().cloned() else {
                studio.message = Some(Err("Make the molds first (step 5).".to_string()));
                return;
            };
            studio.busy = true;
            studio.message = Some(Ok("Saving the printable files…".to_string()));
            job.0 = Some(spawn_export(molds, dest));
        }
        DialogKind::PrepDest => {
            let Some(smoothing) = studio.pending_save.as_ref().map(PendingSave::smoothing) else {
                return;
            };
            // ⚠ Unlike `PrintDest`, a cancel here must clear the pending Save:
            // it gates every control, so backing out silently leaves the app
            // inert with the question that gated it gone from the screen.
            let Some(dest) = picked else {
                settle(&mut studio, Ok("Save cancelled.".to_string()));
                return;
            };
            // ⚠ `&scan`, immutably — see the warning on `ScanEdit`.
            save_into(&scan, &mut studio, dest, smoothing);
        }
    }
}

/// Spawn the export off-thread.
///
/// The `catch_unwind` is carried over from the pre-port code deliberately: a
/// panic inside the copy would otherwise take the task down silently and leave
/// the app wedged on "Saving…" forever, with no way back.
fn spawn_export(
    molds: cf_studio_core::MoldOutputs,
    dest: PathBuf,
) -> Task<Result<PrintExportReport, String>> {
    AsyncComputeTaskPool::get().spawn(async move {
        let outcome = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            export_print_package(&molds, &dest)
        }));
        match outcome {
            Ok(Ok(report)) => Ok(report),
            Ok(Err(e)) => Err(e.to_string()),
            Err(_) => Err("internal error (panic) during export".to_string()),
        }
    })
}

/// Land a finished export: record it on the project, reveal the folder, and
/// report what happened.
pub(crate) fn poll_print_job(mut job: ResMut<PrintJob>, mut studio: ResMut<Studio>) {
    let Some(task) = job.0.as_mut() else { return };
    let Some(result) = future::block_on(future::poll_once(task)) else {
        return;
    };
    job.0 = None;
    studio.busy = false;
    studio.message = Some(match result {
        Ok(report) => {
            let dest = report.export.export_dir.clone();
            let stl_count = report.stl_count;
            let guide = if report.procedure_copied {
                " + the guide"
            } else {
                ""
            };
            match studio.project.set_print(report.export) {
                Ok(()) => {
                    reveal_in_file_manager(&dest);
                    Ok(format!(
                        "✔ Saved {stl_count} file(s){guide} to {} — opening it now. \
                         Print each piece, then click Next →.",
                        dest.display(),
                    ))
                }
                Err(e) => Err(format!("Copied the files, but couldn't record: {e}")),
            }
        }
        Err(msg) => Err(format!("Couldn't save the files: {msg}")),
    });
}

/// What a panicked decimation is reported as.
const SIMPLIFY_PANICKED: &str = "Simplify failed unexpectedly — try a higher target face count.";

/// The running Simplify, and the target it was started with.
#[derive(Resource, Default)]
pub(crate) struct SimplifyJob(Option<(usize, Task<Result<(IndexedMesh, f64), String>>)>);

/// Start a Simplify on the task pool.
///
/// ⚠ `&ScanEdit`, not `&mut` — and that is why a Simplify is not an
/// [`crate::edit::EditIntent`]. Marking the resource changed here would re-run
/// `show_scan` against the previous op's [`crate::scan::ViewUpdate`]: a full
/// rebuild of a 200 000-face mesh, and a camera jump if that op was the load.
pub(crate) fn start_simplify(
    target_faces: usize,
    scan: &ScanEdit,
    studio: &mut Studio,
    job: &mut SimplifyJob,
) {
    let Some(active) = scan.active() else { return };
    let working = active.session().working_clone();
    studio.busy = true;
    studio.message = Some(Ok(format_simplify_started(target_faces)));
    job.0 = Some((target_faces, spawn_simplify(working, target_faces)));
}

/// Decimate off-thread.
///
/// The `catch_unwind` is deliberate, as it was pre-port: a panic here would
/// otherwise leave `busy` stuck on, every control disabled, with no way back.
fn spawn_simplify(
    working: IndexedMesh,
    target_faces: usize,
) -> Task<Result<(IndexedMesh, f64), String>> {
    AsyncComputeTaskPool::get().spawn(async move {
        std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            run_simplify(&working, target_faces)
        }))
        .map_err(|_| SIMPLIFY_PANICKED.to_string())
    })
}

/// Land a finished Simplify: install the mesh, redraw, and report it.
pub(crate) fn poll_simplify_job(
    mut job: ResMut<SimplifyJob>,
    mut studio: ResMut<Studio>,
    mut scan: ResMut<ScanEdit>,
    mut controls: ResMut<EditControls>,
) {
    let Some((target_faces, task)) = job.0.as_mut() else {
        return;
    };
    let Some(result) = future::block_on(future::poll_once(task)) else {
        return;
    };
    let target_faces = *target_faces;
    job.0 = None;
    studio.busy = false;
    match result {
        Ok((mesh, secs)) => {
            // ⚠ Through `edit`, not around it: `apply_simplified` clears the
            // caps, so both the cached display mesh and the centerline overlay
            // are stale until that refresh runs.
            scan.edit(|session| session.apply_simplified(mesh, target_faces));
            land_edit(
                Ok(format_simplify_done(target_faces, secs)),
                &scan,
                &mut studio,
                &mut controls,
            );
        }
        Err(message) => studio.message = Some(Err(message)),
    }
}

/// Open `dir` in the OS file manager. Best-effort — a failure to spawn is
/// ignored, because it is a convenience and not part of the workflow.
pub(crate) fn reveal_in_file_manager(dir: &Path) {
    #[cfg(target_os = "macos")]
    let program = "open";
    #[cfg(target_os = "windows")]
    let program = "explorer";
    #[cfg(all(unix, not(target_os = "macos")))]
    let program = "xdg-open";
    let _ = std::process::Command::new(program).arg(dir).spawn();
}

#[cfg(test)]
mod tests {
    #![allow(clippy::expect_used)]

    use bevy::ecs::system::RunSystemOnce;
    use mesh_types::unit_cube;

    use super::*;
    use crate::scan::{ActiveScan, ViewUpdate};

    const TARGET: usize = 1_000;

    /// Enough app to own a task pool and the poller — no window, no renderer.
    fn app_with_a_loaded_scan() -> App {
        let mut app = App::new();
        app.add_plugins(TaskPoolPlugin::default())
            .init_resource::<Studio>()
            .init_resource::<EditControls>()
            .init_resource::<SimplifyJob>()
            .add_systems(Update, poll_simplify_job);
        let mut scan = ScanEdit::default();
        scan.set(ActiveScan::synthetic(unit_cube()));
        app.insert_resource(scan);
        app.update();
        app
    }

    /// Run frames until the app is handed back.
    ///
    /// ⚠ Wall clock, because these frames are nearly free and a fixed count
    /// gives no margin for a worker not yet scheduled — but keep it short: a
    /// mutant stopping the poller makes every caller sit out the whole
    /// deadline, and a long one turns that caught mutant into a timeout.
    fn run_until_idle(app: &mut App) {
        const DEADLINE: std::time::Duration = std::time::Duration::from_secs(2);
        let deadline = std::time::Instant::now() + DEADLINE;
        while app.world().resource::<Studio>().busy {
            assert!(
                std::time::Instant::now() < deadline,
                "the job never landed — `busy` was never cleared"
            );
            app.update();
        }
    }

    /// Enough app to own a task pool and [`poll_dialogs`], with `dialog`
    /// already answered and a step 2 ready to save into `dir`.
    fn app_with_a_resolved_dialog(dir: &std::path::Path, dialog: PendingDialog) -> App {
        let (scan, studio) = crate::save::tests::ready_to_save(dir);
        let mut app = App::new();
        app.add_plugins(TaskPoolPlugin::default())
            .init_resource::<PrintJob>()
            .add_systems(Update, poll_dialogs);
        app.insert_resource(studio);
        app.insert_resource(scan);
        app.insert_resource(dialog);
        app
    }

    /// ★ The routing that makes the picked folder safe. Sending it straight to
    /// `write_into` — what the pre-port code did — restores a silent overwrite,
    /// and every other gate in this crate stays green while it does.
    #[test]
    fn a_folder_the_picker_returns_is_checked_before_it_is_written_into() {
        let dir = crate::save::tests::temp_dir("routed-scan-folder");
        let picked = crate::save::tests::temp_dir("routed-picked-folder");
        std::fs::write(picked.join("base.prep.toml"), b"in the way").expect("a decoy");
        let mut app = app_with_a_resolved_dialog(
            &dir,
            PendingDialog::resolved(DialogKind::PrepDest, Some(picked.clone())),
        );
        // ⚠ Not 0. The smoothing has to survive the picker, and `0` is what a
        // dropped one would look like.
        app.world_mut().resource_mut::<Studio>().pending_save =
            Some(PendingSave::ChoosingFolder { smoothing: 3 });

        app.update();

        assert_eq!(
            app.world().resource::<Studio>().pending_save,
            Some(PendingSave::Confirming {
                dir: picked.clone(),
                smoothing: 3
            }),
            "the picked folder is asked about, not written into, and the \
             smoothing rides across the picker"
        );
        assert_eq!(
            std::fs::read(picked.join("base.prep.toml")).expect("still there"),
            b"in the way",
            "and the file that was in the way is untouched"
        );
        let _ = std::fs::remove_dir_all(&dir);
        let _ = std::fs::remove_dir_all(&picked);
    }

    /// ⚠ Unlike a cancelled print picker, this one must report and let go:
    /// `pending_save` gates every control, so backing out silently would leave
    /// the app inert with the question that gated it gone from the screen.
    #[test]
    fn cancelling_the_folder_picker_hands_the_app_back() {
        let dir = crate::save::tests::temp_dir("routed-cancel");
        let mut app =
            app_with_a_resolved_dialog(&dir, PendingDialog::resolved(DialogKind::PrepDest, None));
        app.world_mut().resource_mut::<Studio>().pending_save =
            Some(PendingSave::ChoosingFolder { smoothing: 0 });

        app.update();

        let studio = app.world().resource::<Studio>();
        assert!(studio.pending_save.is_none(), "the held save is let go");
        assert!(
            matches!(&studio.message, Some(Ok(text)) if text.contains("cancelled")),
            "and it says so: {:?}",
            studio.message
        );
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// ⚠ The `Remesh` assertion is the point of doing this off the intent
    /// path. The scan was just loaded, so its [`ViewUpdate`] still says
    /// `Reframe` — a Simplify landing outside `ScanEdit::edit` would leave it
    /// saying so, and the camera would snap to the front.
    #[test]
    fn a_simplify_runs_off_thread_and_lands_on_the_session() {
        let mut app = app_with_a_loaded_scan();

        let started = app.world_mut().run_system_once(
            |scan: Res<ScanEdit>, mut studio: ResMut<Studio>, mut job: ResMut<SimplifyJob>| {
                start_simplify(TARGET, &scan, &mut studio, &mut job);
            },
        );
        assert!(started.is_ok(), "the starter must run: {started:?}");
        assert!(
            app.world().resource::<Studio>().busy,
            "the app is held for the length of the run"
        );
        assert!(
            matches!(&app.world().resource::<Studio>().message,
                     Some(Ok(text)) if text.contains("Simplifying to 1000 faces")),
            "and says what it is doing: {:?}",
            app.world().resource::<Studio>().message
        );

        run_until_idle(&mut app);

        let world = app.world();
        assert!(!world.resource::<Studio>().busy, "and is handed back after");
        assert!(
            matches!(&world.resource::<Studio>().message,
                     Some(Ok(text)) if text.contains("✔ Simplified to 1000 faces")),
            "the landing must report itself: {:?}",
            world.resource::<Studio>().message
        );
        assert!(
            world
                .resource::<ScanEdit>()
                .active()
                .is_some_and(|a| a.session().simplify_applied()),
            "the mesh must reach the session, not just the message"
        );
        assert_eq!(
            world.resource::<ScanEdit>().view(),
            ViewUpdate::Remesh,
            "an edit re-meshes; only a new scan moves the camera"
        );
    }

    /// ⚠ `busy` is taken only after a scan is confirmed. Setting it first would
    /// wedge the app on a step with no scan: every control disabled, and no
    /// task running that could ever clear it.
    #[test]
    fn starting_a_simplify_with_no_scan_does_not_hold_the_app() {
        let mut app = App::new();
        app.add_plugins(TaskPoolPlugin::default())
            .init_resource::<Studio>()
            .init_resource::<ScanEdit>()
            .init_resource::<SimplifyJob>();

        let started = app.world_mut().run_system_once(
            |scan: Res<ScanEdit>, mut studio: ResMut<Studio>, mut job: ResMut<SimplifyJob>| {
                start_simplify(TARGET, &scan, &mut studio, &mut job);
            },
        );

        assert!(started.is_ok(), "the starter must run: {started:?}");
        assert!(
            !app.world().resource::<Studio>().busy,
            "nothing is running, so nothing may hold the app"
        );
        assert!(
            app.world().resource::<SimplifyJob>().0.is_none(),
            "and no job was left behind to poll"
        );
    }

    /// What the `catch_unwind` in [`spawn_simplify`] is for: a failed run must
    /// still hand the app back, or the wizard stays disabled for good.
    #[test]
    fn a_failed_simplify_hands_the_app_back_and_says_why() {
        let mut app = app_with_a_loaded_scan();
        // Stand in for the panic path: `start_simplify` would have taken `busy`.
        app.world_mut().resource_mut::<Studio>().busy = true;
        let task = AsyncComputeTaskPool::get()
            .spawn(async { Err::<(IndexedMesh, f64), String>(SIMPLIFY_PANICKED.to_string()) });
        app.world_mut()
            .insert_resource(SimplifyJob(Some((TARGET, task))));

        run_until_idle(&mut app);

        assert!(
            !app.world().resource::<Studio>().busy,
            "a failed run must still hand the app back"
        );
        assert!(
            matches!(&app.world().resource::<Studio>().message,
                     Some(Err(text)) if text.contains("higher target face count")),
            "and must say what to try instead: {:?}",
            app.world().resource::<Studio>().message
        );
    }
}
