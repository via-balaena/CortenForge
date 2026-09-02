//! Background work, and the pollers that land its results on the app.
//!
//! The print export copies the mold package — hundreds of megabytes at 0.5 mm —
//! so it runs off the main thread and `Studio::busy` gates the buttons that
//! could clobber it while it runs.

use std::path::{Path, PathBuf};

use bevy::prelude::*;
use bevy::tasks::{AsyncComputeTaskPool, Task, futures_lite::future};
use cf_studio_engine::{PrintExportReport, export_print_package};

use crate::dialogs::{DialogKind, PendingDialog};
use crate::state::Studio;

/// The running print export, if any.
#[derive(Resource, Default)]
pub(crate) struct PrintJob(Option<Task<Result<PrintExportReport, String>>>);

/// Route a resolved file dialog to the work it was opened for.
pub(crate) fn poll_dialogs(
    mut dialog: ResMut<PendingDialog>,
    mut studio: ResMut<Studio>,
    mut job: ResMut<PrintJob>,
) {
    let Some((kind, picked)) = dialog.poll() else {
        return;
    };
    match kind {
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
                        "✓ Saved {stl_count} file(s){guide} to {} — opening it now. \
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
