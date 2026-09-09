//! Background work, and the pollers that land its results on the app.
//!
//! Three ops are too slow for the main thread; `Studio::busy` gates every
//! control that could clobber one. Simplify (step 2), the cast (step 5), the
//! print export (step 6). The cast is measured in minutes — **408 s at 1.5 mm,
//! 2187 s at 0.5**, bonded — which is why it reports its own elapsed time.
//!
//! ⚠ Step 3's fit check is the one background op that does NOT take `busy`.
//! See [`start_plug_fit`].

use std::path::{Path, PathBuf};
use std::time::Instant;

use bevy::prelude::*;
use bevy::tasks::{AsyncComputeTaskPool, Task, futures_lite::future};
use cf_studio_core::{DesignDraft, MoldOutputs, RidgeOptions};
use cf_studio_engine::{
    PartSelection, PlugFit, PrintExportReport, export_print_package, generate_molds_for_design,
    plug_fit_preflight, run_simplify,
};
use cf_studio_gui::{
    CENDRILLON_CAST_MODE, FitQuestion, FitView, LayerStack, PourSession, apply_design, fit_view,
    format_inexact_design, format_molds_progress, format_simplify_done, format_simplify_started,
};
use mesh_types::IndexedMesh;

use crate::design::DesignControls;
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
    mut design: ResMut<DesignControls>,
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
        DialogKind::DesignFile => {
            let Some(path) = picked else { return };
            let outcome = apply_design(&mut studio.project, &path);
            let reported = match (outcome, studio.project.design()) {
                (Ok(message), Some(loaded)) => {
                    Ok(message + &show_loaded_design(loaded, &mut design))
                }
                (outcome, _) => outcome,
            };
            studio.message = Some(reported);
        }
    }
}

/// Put the design the project just took delivery of onto the rows that edit it,
/// and report anything the rows had to change to show it.
///
/// ⚠ The rows are what "Use this design" commits. Left on the previous stack
/// they do not merely look wrong — the next click replaces the file that just
/// landed with them.
///
/// ⚠ [`LayerStack::from_drafts`] returning `None` leaves the rows alone with
/// nothing said, and nothing here can reach it — measured, not assumed:
/// `load_design_toml` refuses an unknown silicone ("names unknown anchor key")
/// and a file with no layers ("missing field `layers`") before either can land
/// on the project. It stays a branch because
/// [`cf_studio_gui::Silicone::from_key`] is fallible, not because a file gets
/// here that way.
fn show_loaded_design(loaded: &DesignDraft, design: &mut DesignControls) -> String {
    let Some(rows) = LayerStack::from_drafts(&loaded.layers) else {
        return String::new();
    };
    let note = format_inexact_design(&rows, &loaded.layers).unwrap_or_default();
    design.layers = rows;
    note
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

/// A cast in flight: when it started, the last whole second put on screen, and
/// the task itself.
///
/// ⚠ `shown_secs` lives here, not in a system `Local`, so it dies with the run.
/// A `Local` would carry the previous cast's second into the next one and eat
/// that run's first status line.
struct MoldsRun {
    started: Instant,
    shown_secs: u64,
    task: Task<Result<MoldOutputs, String>>,
}

/// The running cast, if any.
///
/// Started by [`start_molds`], polled by [`poll_molds_job`].
#[derive(Resource, Default)]
pub(crate) struct MoldsJob(Option<MoldsRun>);

/// What the step-5 controls carried into a click.
///
/// Snapshotted at click time rather than re-read when the job starts, for the
/// same reason `Acted`'s other carried fields are: the executor must not see a
/// control the user has changed since. It also keeps [`start_molds`]
/// independent of the picker, which is why the poller could be gated before
/// any of this existed.
#[derive(Debug, Clone)]
pub(crate) struct MoldsStart {
    /// Marching-cubes cell size, from the quality picker's index.
    pub(crate) cell_size_m: f64,
    /// Which pieces to generate.
    pub(crate) selection: PartSelection,
}

/// What a panicked cast is reported as.
const MOLDS_PANICKED: &str = "internal error (panic) during mold generation";

/// What is missing when step 5 is somehow reached without its inputs.
const MOLDS_NO_INPUTS: &str = "Finish steps 2 and 4 first (clean the scan, choose a design).";

/// Start a cast on the task pool.
///
/// ⚠ `busy` is taken only once the inputs are confirmed, or the app wedges on a
/// step with no design. There is no "at least one part" check here — the button
/// is disabled instead, and one rule has one home.
pub(crate) fn start_molds(start: &MoldsStart, studio: &mut Studio, job: &mut MoldsJob) {
    // A run already in flight. `busy` disables the button, but a click queued
    // in the same frame still arrives, and a second cast into the same output
    // directory would race the first for the better part of an hour.
    if job.0.is_some() {
        return;
    }
    let inputs = studio
        .project
        .prep()
        .zip(studio.project.design())
        .map(|(prep, design)| {
            (
                prep.cleaned_stl.clone(),
                prep.prep_toml.clone(),
                design.clone(),
            )
        });
    let Some((cleaned_stl, prep_toml, draft)) = inputs else {
        studio.message = Some(Err(MOLDS_NO_INPUTS.to_string()));
        return;
    };
    // The ridges were committed with the plug on "Shape your piece". The one
    // field rides every offset, so the plug and every shell carry it alike.
    let ridges = studio
        .project
        .plug()
        .map(|plug| plug.ridges.clone())
        .unwrap_or_default();
    studio.busy = true;
    // `shown_secs` means "already on screen", so the seed and the opening line
    // have to be the same second or the poller suppresses the line it never drew.
    const OPENING: u64 = 0;
    studio.message = Some(Ok(format_molds_progress(OPENING)));
    job.0 = Some(MoldsRun {
        started: Instant::now(),
        shown_secs: OPENING,
        task: spawn_molds(cleaned_stl, prep_toml, draft, start.clone(), ridges),
    });
}

/// Run the cast off-thread.
///
/// The `catch_unwind` is the guard the other two jobs carry, and it earns more
/// here than anywhere: a panic half an hour in would otherwise leave `busy`
/// stuck on with no way back.
fn spawn_molds(
    cleaned_stl: PathBuf,
    prep_toml: PathBuf,
    draft: DesignDraft,
    start: MoldsStart,
    ridges: RidgeOptions,
) -> Task<Result<MoldOutputs, String>> {
    AsyncComputeTaskPool::get().spawn(async move {
        let outcome = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            generate_molds_for_design(
                &cleaned_stl,
                &prep_toml,
                &draft,
                start.cell_size_m,
                &ridges,
                &start.selection,
                CENDRILLON_CAST_MODE,
                None,
            )
        }));
        match outcome {
            Ok(Ok(outputs)) => Ok(outputs),
            Ok(Err(e)) => Err(e.to_string()),
            Err(_) => Err(MOLDS_PANICKED.to_string()),
        }
    })
}

/// The second to draw, or `None` to leave the line alone.
fn clock_to_draw(elapsed_secs: u64, shown: u64) -> Option<u64> {
    (shown != elapsed_secs).then_some(elapsed_secs)
}

/// Land a finished cast, or tick the clock on a running one.
pub(crate) fn poll_molds_job(mut job: ResMut<MoldsJob>, mut studio: ResMut<Studio>) {
    let Some(run) = job.0.as_mut() else { return };
    let Some(result) = future::block_on(future::poll_once(&mut run.task)) else {
        // Still running. The decision is `clock_to_draw`; this only carries
        // it out.
        if let Some(secs) = clock_to_draw(run.started.elapsed().as_secs(), run.shown_secs) {
            run.shown_secs = secs;
            studio.message = Some(Ok(format_molds_progress(secs)));
        }
        return;
    };
    job.0 = None;
    studio.busy = false;
    studio.message = Some(match result {
        Ok(outputs) => match studio.project.set_molds(outputs) {
            Ok(()) => {
                // ⚠ New molds mean a new pour. `set_molds` clears the project's
                // *record* of one, but the live session and its countdown are
                // held here, so step 7 would open on the previous cast's
                // progress. `Studio::record_scan` resets the same pair.
                studio.pour = PourSession::default();
                studio.pour_deadline = None;
                Ok("✔ Molds ready — click Next →.".to_string())
            }
            Err(e) => Err(format!("Molds made, but couldn't record them: {e}")),
        },
        Err(msg) => Err(format!("Mold generation failed: {msg}")),
    });
}

/// A fit check in flight: the question it is answering, when it started, and
/// the task.
///
/// ⚠ The question is carried so the answer can be matched back to it. Nothing
/// stops the user editing step 3 while this runs, and an answer about settings
/// they have moved on from is worse than no answer.
struct FitRun {
    question: FitQuestion,
    started: Instant,
    task: Task<Result<PlugFit, String>>,
}

/// Step 3's fit check: the run in flight, and the last answer with the question
/// it answers.
///
/// ⚠ Both halves live here rather than on [`Studio`], for two reasons.
/// `Studio::next`/`back` clear the step message, so a verdict kept there would
/// vanish on a trip to step 4 and back while the fields it describes had not
/// moved. And keeping the answer beside its question puts the staleness rule in
/// one place — [`PlugFitJob::view`].
#[derive(Resource, Default)]
pub(crate) struct PlugFitJob {
    running: Option<FitRun>,
    /// What the last check came back with, and the question it answers: the
    /// cast's verdict, or why it could not run at all.
    ///
    /// ⚠ The failure lives here too, rather than on `Studio::message`. This is
    /// the one job that leaves `Studio::busy` alone, so the operator can walk
    /// to another step while it runs — and the shared message would follow them
    /// there, over the top of a running cast's progress line.
    answered: Option<(FitQuestion, Result<PlugFit, String>)>,
}

impl PlugFitJob {
    /// Whether a check is in flight — for any question, which is why the button
    /// this disables needs no question of its own.
    pub(crate) const fn is_running(&self) -> bool {
        self.running.is_some()
    }

    /// What step 3 should show about `current`.
    pub(crate) fn view(&self, current: &FitQuestion) -> FitView<'_> {
        fit_view(
            self.running
                .as_ref()
                .map(|run| (&run.question, run.started.elapsed().as_secs())),
            self.answered.as_ref(),
            current,
        )
    }
}

/// The two states step 3 can be drawn against, built directly.
///
/// ⚠ Constructors, not a second implementation: [`PlugFitJob::view`] and the
/// panel's use of it are exactly the code under test. The in-flight task is a
/// future that never resolves, so the running state holds for as long as the
/// gate needs it.
#[cfg(test)]
impl PlugFitJob {
    pub(crate) fn in_flight(question: FitQuestion, task: Task<Result<PlugFit, String>>) -> Self {
        Self {
            running: Some(FitRun {
                question,
                started: Instant::now(),
                task,
            }),
            answered: None,
        }
    }

    pub(crate) fn running_for(question: FitQuestion) -> Self {
        Self::in_flight(
            question,
            AsyncComputeTaskPool::get_or_init(bevy::tasks::TaskPool::default)
                .spawn(std::future::pending::<Result<PlugFit, String>>()),
        )
    }

    pub(crate) fn answered_with(question: FitQuestion, outcome: Result<PlugFit, String>) -> Self {
        Self {
            running: None,
            answered: Some((question, outcome)),
        }
    }

    /// The question the check in flight is answering.
    pub(crate) fn asking(&self) -> Option<&FitQuestion> {
        self.running.as_ref().map(|run| &run.question)
    }
}

/// What a panicked fit check is reported as.
const FIT_PANICKED: &str = "internal error (panic) during the fit check";

/// What is missing when the check is somehow asked for without a cleaned scan.
const FIT_NO_PREP: &str = "Clean and save the scan first (step 2).";

/// Start a fit check for `question`.
///
/// ⚠⚠ `Studio::busy` is deliberately NOT taken, and this is the only job here
/// that leaves it alone. The check is opt-in and runs for minutes once ridges
/// are on; freezing every control behind it would make asking the question
/// worse than never asking. Nothing downstream reads a verdict, so a Continue
/// mid-run is safe — the answer lands into [`fit_view`]'s staleness drop.
///
/// ⚠ `studio` is borrowed immutably, and that is the other half of leaving
/// `busy` alone: the operator can walk to another step while this runs, so
/// nothing about the check may reach state their new screen is reading.
pub(crate) fn start_plug_fit(question: FitQuestion, studio: &Studio, job: &mut PlugFitJob) {
    // A click queued in the same frame the button disabled itself still
    // arrives, and a second check would race the first for minutes.
    if job.running.is_some() {
        return;
    }
    let Some(prep) = studio.project.prep() else {
        job.answered = Some((question, Err(FIT_NO_PREP.to_string())));
        return;
    };
    let (cleaned_stl, prep_toml) = (prep.cleaned_stl.clone(), prep.prep_toml.clone());
    job.running = Some(FitRun {
        task: spawn_plug_fit(cleaned_stl, prep_toml, question.clone()),
        started: Instant::now(),
        question,
    });
}

/// Run the pre-flight off-thread.
fn spawn_plug_fit(
    cleaned_stl: PathBuf,
    prep_toml: PathBuf,
    question: FitQuestion,
) -> Task<Result<PlugFit, String>> {
    AsyncComputeTaskPool::get().spawn(async move {
        let outcome = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            plug_fit_preflight(
                &cleaned_stl,
                &prep_toml,
                question.plug.cavity_inset_m,
                &question.plug.ridges,
                question.cell_size_m,
            )
        }));
        match outcome {
            Ok(Ok(fit)) => Ok(fit),
            Ok(Err(e)) => Err(e.to_string()),
            Err(_) => Err(FIT_PANICKED.to_string()),
        }
    })
}

/// Land a finished fit check.
///
/// ⚠ No clock to tick, unlike the cast: [`PlugFitJob::view`] reads the elapsed
/// time straight off the run, so the progress line needs no bookkeeping here.
///
/// ⚠ Takes no `Studio` at all. A verdict and a failure are told apart by
/// [`FitView`], not by which channel they arrive on — and neither may touch the
/// screen the operator walked to while this ran.
pub(crate) fn poll_plug_fit_job(mut job: ResMut<PlugFitJob>) {
    let Some(run) = job.running.as_mut() else {
        return;
    };
    let Some(result) = future::block_on(future::poll_once(&mut run.task)) else {
        return;
    };
    let question = run.question.clone();
    job.running = None;
    job.answered = Some((question, result));
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
///
/// ⚠ Knowingly ungated, and the one surviving mutant in this file (measured
/// 2026-09-06: 21 mutants, 6 caught, 14 unviable, this one missed). Gating it
/// means either opening a Finder window on every test run or injecting the
/// spawn — both worse than an untested convenience whose failure is already
/// defined as "nothing happens".
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
pub(crate) mod tests {
    #![allow(clippy::expect_used)]

    use std::time::Duration;

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
    fn run_until_idle(app: &mut App, what: &str) {
        const DEADLINE: Duration = Duration::from_secs(2);
        // ⚠ Zero frames is a pass otherwise, exactly as in
        // `run_plugin_update_until`: the caller's assertions then read the state
        // it seeded rather than the poller's output.
        assert!(
            app.world().resource::<Studio>().busy,
            "{what} was already idle before a frame ran"
        );
        let deadline = Instant::now() + DEADLINE;
        while app.world().resource::<Studio>().busy {
            assert!(
                Instant::now() < deadline,
                "{what} never landed — `busy` was never cleared"
            );
            app.update();
        }
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

        run_until_idle(&mut app, "the job");

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

        run_until_idle(&mut app, "the job");

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

    /// Run frames until the dialog has been polled, or fail.
    ///
    /// ⚠ Not one `update`. `PendingDialog::resolved` spawns its answer on the
    /// task pool, and a worker that has not been scheduled yet leaves `poll`
    /// returning `None` — so a single frame passes alone and fails in a full
    /// run. Same reason [`run_until_idle`] exists.
    fn run_until_answered(app: &mut App) {
        const DEADLINE: Duration = Duration::from_secs(2);
        let deadline = Instant::now() + DEADLINE;
        while app.world().resource::<PendingDialog>().is_open() {
            assert!(Instant::now() < deadline, "the dialog never resolved");
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
            .init_resource::<DesignControls>()
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

        run_until_answered(&mut app);

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

        run_until_answered(&mut app);

        let studio = app.world().resource::<Studio>();
        assert!(studio.pending_save.is_none(), "the held save is let go");
        assert!(
            matches!(&studio.message, Some(Ok(text)) if text.contains("cancelled")),
            "and it says so: {:?}",
            studio.message
        );
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// A minimal loadable scan, so the `ScanFile` arm has something real to put
    /// on the project and in the viewport.
    const ONE_TRIANGLE_STL: &str = "\
solid t
facet normal 0 0 1
  outer loop
    vertex 0 0 0
    vertex 1 0 0
    vertex 0 1 0
  endloop
endfacet
endsolid t
";

    /// ★ A sibling arm of the poller `PrepDest` was added to, and unreachable
    /// from any test until `PendingDialog::resolved` existed. Dropping its real
    /// work — a chosen scan that never reaches the viewport — passed the whole
    /// suite.
    #[test]
    fn a_chosen_scan_lands_on_the_project_and_in_the_viewport() {
        let dir = crate::save::tests::temp_dir("chosen-scan");
        let file = dir.join("pick.stl");
        std::fs::write(&file, ONE_TRIANGLE_STL).expect("a scan to pick");
        let mut app = App::new();
        app.add_plugins(TaskPoolPlugin::default())
            .init_resource::<Studio>()
            .init_resource::<ScanEdit>()
            .init_resource::<PrintJob>()
            .init_resource::<DesignControls>()
            .add_systems(Update, poll_dialogs);
        app.insert_resource(PendingDialog::resolved(DialogKind::ScanFile, Some(file)));

        run_until_answered(&mut app);

        assert!(
            app.world().resource::<ScanEdit>().active().is_some(),
            "the viewport has the scan"
        );
        let studio = app.world().resource::<Studio>();
        assert!(
            studio.project.scan().is_some(),
            "and the project recorded it: {:?}",
            studio.message
        );
        // ⚠ Landing it silently is its own failure.
        assert!(
            matches!(&studio.message, Some(Ok(text)) if !text.is_empty()),
            "and it said so: {:?}",
            studio.message
        );
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// ⚠ `busy` and the spawned job are one decision: the app tells the user it
    /// is saving, so something must actually be saving.
    #[test]
    fn a_chosen_export_folder_starts_the_job_that_holds_the_app() {
        let dir = crate::save::tests::temp_dir("chosen-export");
        let mut app = App::new();
        app.add_plugins(TaskPoolPlugin::default())
            .init_resource::<ScanEdit>()
            .init_resource::<PrintJob>()
            .init_resource::<DesignControls>()
            .add_systems(Update, poll_dialogs);
        app.insert_resource(Studio {
            project: crate::panel::tests::ready_to_pour(),
            ..Studio::default()
        });
        app.insert_resource(PendingDialog::resolved(
            DialogKind::PrintDest,
            Some(dir.clone()),
        ));

        run_until_answered(&mut app);

        assert!(app.world().resource::<Studio>().busy, "the app is held");
        assert!(
            app.world().resource::<PrintJob>().0.is_some(),
            "and a job is actually running to hold it for"
        );
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// A real `.design.toml` on disk, written by the writer the app itself
    /// uses — a hand-rolled literal would pass a loader that had stopped
    /// agreeing with the engine.
    fn a_design_file(dir: &std::path::Path) -> PathBuf {
        let path = dir.join("base.design.toml");
        cf_studio_engine::save_design_from_draft(
            Path::new("base.cleaned.stl"),
            &cf_studio_core::DesignDraft {
                cavity_inset_m: 0.005,
                layers: vec![cf_studio_core::LayerDraft {
                    thickness_m: 0.006,
                    material_key: "DRAGON_SKIN_20A".to_string(),
                    slacker_fraction: 0.1,
                }],
            },
            &path,
        )
        .expect("the engine writes its own format");
        path
    }

    /// The dialog app, walked on to the state a design can be set from.
    fn app_ready_for_a_design(dir: &std::path::Path, dialog: PendingDialog) -> App {
        let mut app = app_with_a_resolved_dialog(dir, dialog);
        let mut studio = app.world_mut().resource_mut::<Studio>();
        studio
            .project
            .set_prep(cf_studio_core::PrepInput {
                cleaned_stl: dir.join("base.cleaned.stl"),
                prep_toml: dir.join("base.prep.toml"),
            })
            .and_then(|()| {
                studio
                    .project
                    .set_plug(cf_studio_core::PlugDraft::default())
            })
            .expect("each artifact is set in workflow order");
        app
    }

    /// ★ "…or load a file" is the only way a design arrives from outside the
    /// editor, and the picked path has to reach `apply_design`. Routed
    /// nowhere, the picker opens, closes, and the screen says nothing —
    /// which every gate on the panel side passes.
    #[test]
    fn a_design_file_the_picker_returns_reaches_the_project() {
        let dir = crate::save::tests::temp_dir("routed-design-file");
        let picked = a_design_file(&dir);
        let mut app = app_ready_for_a_design(
            &dir,
            PendingDialog::resolved(DialogKind::DesignFile, Some(picked)),
        );

        run_until_answered(&mut app);

        let studio = app.world().resource::<Studio>();
        assert_eq!(
            studio.project.design().map(|design| design
                .layers
                .iter()
                .map(|layer| layer.material_key.as_str())
                .collect::<Vec<_>>()),
            Some(vec!["DRAGON_SKIN_20A"]),
            "the file's own stack, not the editor's: {:?}",
            studio.message
        );
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// ⚠ A cancel leaves the editor's stack standing — the only state this
    /// step has — so it must not clear the design or say anything.
    #[test]
    fn cancelling_the_design_picker_leaves_the_project_alone() {
        let dir = crate::save::tests::temp_dir("cancelled-design-file");
        let mut app =
            app_ready_for_a_design(&dir, PendingDialog::resolved(DialogKind::DesignFile, None));
        let before = app.world().resource::<Studio>().message.clone();

        run_until_answered(&mut app);

        let studio = app.world().resource::<Studio>();
        assert!(studio.project.design().is_none(), "no design was set");
        assert_eq!(studio.message, before, "and nothing new was reported");
        let _ = std::fs::remove_dir_all(&dir);
    }

    // ── the cast job ────────────────────────────────────────────────────────
    //
    // ⚠ The real cast runs for minutes and never runs here. Every test below
    // hands the poller a `Task` built by hand — the same trick the injected
    // `SimplifyJob` uses, except that here it is the only option.

    /// A pour step, as a real cast run produces one.
    fn pour_step(layer_index: usize, pot_life_minutes: u32) -> cf_studio_core::PourStep {
        cf_studio_core::PourStep {
            layer_index,
            material_display_name: "Ecoflex 00-30".to_string(),
            mass_g: 40.0,
            mix_ratio_a_to_b: "1:1".to_string(),
            pot_life_minutes,
            cure_time_hours: 4.0,
            slacker_fraction: None,
        }
    }

    /// What a finished cast hands back. `tag` lands in `out_dir` so a test can
    /// tell one run's outputs from another's.
    pub(crate) fn some_molds(tag: &str) -> MoldOutputs {
        MoldOutputs {
            out_dir: tag.into(),
            mold_stls: vec!["cup.stl".into()],
            plug_stls: vec!["plug.stl".into()],
            accessory_stls: Vec::new(),
            procedure_path: "procedure.md".into(),
            total_mass_g: 80.0,
            pour_plan: cf_studio_core::PourPlan {
                steps: vec![pour_step(0, 30), pour_step(1, 45)],
            },
        }
    }

    /// A project carried as far as step 4, which is what `set_molds` requires.
    fn studio_ready_for_molds() -> Studio {
        use cf_studio_core::{DesignDraft, LayerDraft, PlugDraft, PrepInput, ScanInput};

        let layer = || LayerDraft {
            thickness_m: 0.003,
            material_key: "ECOFLEX_00_30".to_string(),
            slacker_fraction: 0.0,
        };
        // ⚠ Absolute. `start_molds` spawns the real cast, and a relative path
        // makes the crate directory the cast's `base_dir`. The engine rejects
        // that now; this keeps the fixture on the right side of it without
        // depending on the error.
        let dir = crate::molds::tests::fixture_root().join(crate::molds::tests::test_label());
        let mut studio = Studio::default();
        studio.project.set_scan(ScanInput {
            source_path: dir.join("scan.stl"),
        });
        let built = [
            studio.project.set_prep(PrepInput {
                cleaned_stl: dir.join("scan.cleaned.stl"),
                prep_toml: dir.join("scan.prep.toml"),
            }),
            studio.project.set_plug(PlugDraft::default()),
            studio.project.set_design(DesignDraft {
                cavity_inset_m: 0.0,
                layers: vec![layer(), layer()],
            }),
        ];
        assert!(
            built.iter().all(Result::is_ok),
            "the fixture must reach DesignLayers: {built:?}"
        );
        studio
    }

    /// Enough app to own a task pool and the mold poller — no window.
    fn app_ready_for_molds() -> App {
        let mut app = App::new();
        app.add_plugins(TaskPoolPlugin::default())
            .init_resource::<MoldsJob>()
            .add_systems(Update, poll_molds_job);
        app.insert_resource(studio_ready_for_molds());
        app
    }

    /// How long a `backdated` reading stays valid.
    const CLOCK_MARGIN_MS: u64 = 1000;

    /// An `Instant` exactly `secs` in the past, so `elapsed().as_secs() == secs`.
    ///
    /// ⚠ No safety offset: `elapsed()` is read strictly after the `Instant::now()`
    /// this is built from, so the reading is already `secs` and holds for the
    /// whole next second. An offset would only spend that margin.
    ///
    /// ⛔ Not replaced by an injected clock, deliberately.
    /// `the_clock_follows_real_elapsed_time` exists to gate that the source IS
    /// the real clock — a seam would make it vacuous.
    fn backdated(secs: u64) -> Instant {
        Instant::now()
            .checked_sub(Duration::from_millis(secs * 1000))
            .expect("a few seconds of uptime")
    }

    /// Put a run into the job by hand: the task, when it started, and the
    /// second last shown.
    fn inject(
        app: &mut App,
        task: Task<Result<MoldOutputs, String>>,
        started: Instant,
        shown_secs: u64,
    ) {
        app.world_mut().resource_mut::<Studio>().busy = true;
        app.world_mut().resource_mut::<MoldsJob>().0 = Some(MoldsRun {
            started,
            shown_secs,
            task,
        });
    }

    fn finished(result: Result<MoldOutputs, String>) -> Task<Result<MoldOutputs, String>> {
        AsyncComputeTaskPool::get().spawn(async move { result })
    }

    fn never_finishes() -> Task<Result<MoldOutputs, String>> {
        AsyncComputeTaskPool::get().spawn(async { future::pending().await })
    }

    /// The second the running cast last drew.
    fn shown_second(app: &App) -> u64 {
        app.world()
            .resource::<MoldsJob>()
            .0
            .as_ref()
            .map(|run| run.shown_secs)
            .expect("a cast is in flight")
    }

    #[test]
    fn a_landed_cast_records_the_molds_and_hands_the_app_back() {
        let mut app = app_ready_for_molds();
        let task = finished(Ok(some_molds("out-a")));
        inject(&mut app, task, Instant::now(), 0);

        run_until_idle(&mut app, "the cast");

        let studio = app.world().resource::<Studio>();
        assert!(!studio.busy, "the app comes back");
        assert_eq!(
            studio.project.molds().map(|m| m.out_dir.clone()),
            Some("out-a".into()),
            "the outputs must reach the PROJECT, not just the message"
        );
        assert!(
            matches!(&studio.message, Some(Ok(text)) if text.contains("Molds ready")),
            "and it says so: {:?}",
            studio.message
        );
        assert!(
            app.world().resource::<MoldsJob>().0.is_none(),
            "the run is cleared, so a second cast can start"
        );
    }

    /// ⚠⚠ `Project::set_molds` clears the pour *record*, but the live session
    /// and its countdown are held on `Studio` — so without this reset a second
    /// cast opens step 7 on the FIRST cast's progress, part-way through a pour
    /// plan that no longer exists.
    #[test]
    fn new_molds_reset_the_pour_session_and_its_countdown() {
        let mut app = app_ready_for_molds();
        {
            let mut studio = app.world_mut().resource_mut::<Studio>();
            studio
                .project
                .set_molds(some_molds("out-first"))
                .expect("the fixture records a first cast");
            studio.mark_poured();
            studio.start_pour_timer();
            assert_eq!(studio.pour.current(), 1, "the fixture is mid-pour");
            assert!(studio.pour_deadline.is_some(), "with a clock running");
        }

        let task = finished(Ok(some_molds("out-second")));
        inject(&mut app, task, Instant::now(), 0);
        run_until_idle(&mut app, "the cast");

        let studio = app.world().resource::<Studio>();
        assert_eq!(
            studio.pour.current(),
            0,
            "a new cast starts its pour at layer 1"
        );
        assert!(
            studio.pour_deadline.is_none(),
            "and the previous layer's countdown is stopped, not left running"
        );
    }

    #[test]
    fn a_failed_cast_says_why_and_hands_the_app_back() {
        let mut app = app_ready_for_molds();
        let task = finished(Err("the mesher gave up".to_string()));
        inject(&mut app, task, Instant::now(), 0);

        run_until_idle(&mut app, "the cast");

        let studio = app.world().resource::<Studio>();
        assert!(!studio.busy, "a failure must hand the app back too");
        assert!(
            matches!(&studio.message, Some(Err(text)) if text.contains("the mesher gave up")),
            "and carry the reason, not just 'it failed': {:?}",
            studio.message
        );
        assert!(
            studio.project.molds().is_none(),
            "nothing is recorded from a failed run"
        );
    }

    #[test]
    fn the_clock_redraws_only_when_the_whole_second_changes() {
        // `start_molds` already drew 0:00, so the seed is a second already shown.
        assert_eq!(clock_to_draw(0, 0), None);
        assert_eq!(clock_to_draw(75, 0), Some(75));
        assert_eq!(clock_to_draw(75, 75), None);
        assert_eq!(clock_to_draw(76, 75), Some(76));
    }

    /// The clock source. Only a known duration catches a wrong scale: "it
    /// advanced" is satisfied by `as_millis()` in under a millisecond, while the
    /// app shows 18:20 for a one-second cast.
    #[test]
    fn the_clock_follows_real_elapsed_time() {
        const WAITED: u64 = 2;
        const LINE: &str =
            "Making molds… 0:02 elapsed (this can take a while — the window stays responsive)";

        let mut app = app_ready_for_molds();
        // ⚠ Backdated, not slept. A sleep plus a tolerance is what let a
        // constant source through: with ±1 s of slack a poller reading a fixed
        // `Duration::from_secs(3)` passed all 171. Taken after the app is
        // built, so construction does not eat the margin.
        let started = backdated(WAITED);
        // ⚠ `0` is what `start_molds` seeds. Seed a second that production
        // never seeds and a poller which draws once, then never redraws, passes
        // all 171.
        inject(&mut app, never_finishes(), started, 0);

        app.update();

        let shown = shown_second(&app);
        assert_eq!(
            shown, WAITED,
            "backdated {WAITED}s, clock says {shown} \
             (a stall over {CLOCK_MARGIN_MS} ms here reads as the next second)"
        );
        // ⚠ The literal, not `format_molds_progress(WAITED)`. An expectation
        // built by calling the subject is a mirror; the one excuse for it was
        // "lib.rs owns the wording", which round seven proved false.
        assert_eq!(
            app.world().resource::<Studio>().message,
            Some(Ok(LINE.to_string())),
            "and the line the run is on must reach the screen"
        );
    }

    /// The suppression at the caller. `clock_to_draw` returning `None` is half
    /// of it; the poller has to honour it.
    ///
    /// ⚠ Without this, replacing the whole `if let` with an unconditional write
    /// passed all 171 — a fresh `format!` every frame for the 36 minutes of a
    /// print-quality cast.
    #[test]
    fn the_line_is_left_alone_within_the_same_second() {
        const WAITED: u64 = 2;

        let mut app = app_ready_for_molds();
        // ⚠ The backdate has to land inside the seeded second: the case is
        // "already shown", so `elapsed().as_secs()` must equal `shown_secs`.
        let started = backdated(WAITED);
        // ⚠ The suite's only NONZERO `shown_secs`. With 0 injected everywhere,
        // hardcoding the poller's second argument to 0 — dropping the field read
        // outright — passed all 172.
        inject(&mut app, never_finishes(), started, WAITED);
        let sentinel = Some(Ok("SENTINEL".to_string()));
        app.world_mut().resource_mut::<Studio>().message = sentinel.clone();

        app.update();

        assert_eq!(
            app.world().resource::<Studio>().message,
            sentinel,
            "the whole second has not changed, so the line must not be rebuilt \
             (a stall over {CLOCK_MARGIN_MS} ms here reads as the next second)"
        );
    }

    /// The redraw, from a second that is NOT the seed.
    ///
    /// ⚠ The last cell of the decision's 2x2. Every other injection in the
    /// suite uses `shown_secs == 0`, so a poller that draws only while
    /// `shown_secs == 0` — one line off the seed, then never again — passed all
    /// 173 while a 36-minute cast sat at 0:01 the whole way.
    #[test]
    fn the_line_is_redrawn_from_a_second_that_is_not_the_seed() {
        const SHOWN: u64 = 1;
        const ELAPSED: u64 = 2;
        const LINE: &str =
            "Making molds… 0:02 elapsed (this can take a while — the window stays responsive)";

        let mut app = app_ready_for_molds();
        inject(&mut app, never_finishes(), backdated(ELAPSED), SHOWN);
        app.world_mut().resource_mut::<Studio>().message = Some(Ok("SENTINEL".to_string()));

        app.update();

        assert_eq!(
            shown_second(&app),
            ELAPSED,
            "the second on screen must advance off a nonzero seed"
        );
        assert_eq!(
            app.world().resource::<Studio>().message,
            Some(Ok(LINE.to_string())),
            "and the line must be redrawn \
             (a stall over {CLOCK_MARGIN_MS} ms here reads as the next second)"
        );
    }

    /// Both halves of the `OPENING` coupling.
    ///
    /// ⚠ `panel.rs` gates the line, but `MoldsJob.0` is private to this module,
    /// so a drift on the field alone — `shown_secs: OPENING + 1` — passed all
    /// 172. The poller would then measure suppression against a second that was
    /// never drawn and rebuild the identical line every frame.
    #[test]
    fn the_opening_line_and_the_seed_are_the_same_second() {
        // The pool `start_molds` spawns onto — the form `dialogs.rs` uses.
        AsyncComputeTaskPool::get_or_init(bevy::tasks::TaskPool::default);
        let mut studio = studio_ready_for_molds();
        let mut job = MoldsJob::default();

        start_molds(
            &MoldsStart {
                cell_size_m: 0.0015,
                selection: PartSelection::all(),
            },
            &mut studio,
            &mut job,
        );

        assert_eq!(
            job.0.as_ref().expect("a cast is in flight").shown_secs,
            0,
            "the seed is the second already on screen"
        );
        assert_eq!(
            studio.message,
            Some(Ok(
                "Making molds… 0:00 elapsed (this can take a while — the window stays responsive)"
                    .to_string()
            )),
            "and the opening line shows that same second"
        );
    }

    /// The step-3 question these gates ask: `base_mold`'s own saved inset, at
    /// the quality step 5 opens on.
    fn fit_question() -> FitQuestion {
        FitQuestion {
            plug: cf_studio_core::PlugDraft {
                cavity_inset_m: 0.011,
                ridges: RidgeOptions::default(),
            },
            cell_size_m: 0.0005,
            scan: None,
        }
    }

    fn fit_finished(result: Result<PlugFit, String>) -> Task<Result<PlugFit, String>> {
        AsyncComputeTaskPool::get().spawn(async move { result })
    }

    /// Enough app to own a task pool and the fit poller — no window, no cast.
    fn app_polling_fit() -> App {
        let mut app = App::new();
        app.add_plugins(TaskPoolPlugin::default())
            .init_resource::<PlugFitJob>()
            .add_systems(Update, poll_plug_fit_job);
        app.insert_resource(Studio {
            project: crate::shape::tests::ready_to_shape(),
            ..Studio::default()
        });
        app
    }

    /// Put a check into the job by hand.
    fn inject_fit(app: &mut App, question: FitQuestion, task: Task<Result<PlugFit, String>>) {
        *app.world_mut().resource_mut::<PlugFitJob>() = PlugFitJob::in_flight(question, task);
    }

    /// Run `Update` until the check lands.
    fn run_until_landed(app: &mut App) {
        let deadline = Instant::now() + Duration::from_secs(2);
        while app.world().resource::<PlugFitJob>().is_running() {
            assert!(
                Instant::now() < deadline,
                "the poller never landed the check"
            );
            app.world_mut().run_schedule(Update);
        }
    }

    /// ⚠⚠ Two outcomes that must not be confused — a refusal is the cast's
    /// verdict, a broken check is no verdict at all — told apart by [`FitView`]
    /// rather than by which channel they arrive on.
    ///
    /// ★★★ And NEITHER may reach `Studio::message`. Every other job here takes
    /// `Studio::busy`, which disables the nav and pins the operator on the step
    /// that started it; this one does not, so anything sent to the shared
    /// message would surface on whatever step they walked to — over the top of
    /// a running cast's progress line.
    #[test]
    fn a_refusal_and_a_broken_check_land_on_step_three_and_nowhere_else() {
        let refusal = PlugFit::WillNotCast {
            reason: "plug layer 0 came out in 3 pieces".to_string(),
        };

        let mut app = app_polling_fit();
        let task = fit_finished(Ok(refusal.clone()));
        inject_fit(&mut app, fit_question(), task);
        run_until_landed(&mut app);

        assert_eq!(
            app.world().resource::<PlugFitJob>().view(&fit_question()),
            FitView::Answered(&refusal),
            "the refusal is the cast's verdict, and step 3 can read it back"
        );
        assert!(
            app.world().resource::<Studio>().message.is_none(),
            "and nothing about it reached the shared message: {:?}",
            app.world().resource::<Studio>().message
        );

        let mut app = app_polling_fit();
        let task = fit_finished(Err("scan.cleaned.stl: no such file".to_string()));
        inject_fit(&mut app, fit_question(), task);
        run_until_landed(&mut app);

        assert_eq!(
            app.world().resource::<PlugFitJob>().view(&fit_question()),
            FitView::Failed("scan.cleaned.stl: no such file"),
            "a check that could not run is a failure, not a verdict"
        );
        assert!(
            app.world().resource::<Studio>().message.is_none(),
            "and it does not follow the operator to the step they walked to: {:?}",
            app.world().resource::<Studio>().message
        );
    }

    /// ⚠ A click queued in the same frame the button disabled itself still
    /// arrives. Without the guard a second check runs beside the first for
    /// minutes, and whichever lands last wins.
    #[test]
    fn a_second_check_is_refused_while_one_is_in_flight() {
        let first = fit_question();
        let mut job = PlugFitJob::running_for(first.clone());
        let studio = Studio {
            project: crate::shape::tests::ready_to_shape(),
            ..Studio::default()
        };

        start_plug_fit(
            FitQuestion {
                cell_size_m: 0.0015,
                ..first.clone()
            },
            &studio,
            &mut job,
        );

        assert_eq!(
            job.asking(),
            Some(&first),
            "the check already in flight is the one still running"
        );
    }

    /// ⚠ Names the step that produces the input, not the one that is missing
    /// it: step 3 is reachable only once step 2 has saved, so this is a
    /// should-not-happen that still has to say something useful.
    #[test]
    fn a_check_asked_for_without_a_cleaned_scan_says_so_instead_of_running() {
        let mut job = PlugFitJob::default();
        let studio = Studio::default();

        start_plug_fit(fit_question(), &studio, &mut job);

        assert!(!job.is_running(), "nothing was started");
        assert!(
            matches!(job.view(&fit_question()), FitView::Failed(text) if text.contains("step 2")),
            "and it says so on step 3, naming the step that produces the input"
        );
    }

    /// The fourth of the four, and the same hole. Every gate above inserts
    /// `PlugFitJob` by hand, so dropping the `init_resource` or the schedule
    /// entry leaves them green while a landed verdict never reaches the screen.
    #[test]
    fn the_plugin_registers_the_fit_job_and_runs_its_poller() {
        let mut app = app_from_the_plugin();
        let question = fit_question();

        // ⚠ `resource_mut`, not `insert_resource`: this line is the assertion
        // that the PLUGIN registered it.
        *app.world_mut().resource_mut::<PlugFitJob>() =
            PlugFitJob::in_flight(question.clone(), fit_finished(Ok(PlugFit::Casts)));

        run_plugin_update_until(&mut app, "the fit poller", |app| {
            !app.world().resource::<PlugFitJob>().is_running()
        });

        assert_eq!(
            app.world().resource::<PlugFitJob>().view(&question),
            FitView::Answered(&PlugFit::Casts),
            "the verdict landed through the plugin's own wiring"
        );
    }

    /// The plugin's own wiring, which nothing else reaches.
    ///
    /// ⚠ Every test above inserts `MoldsJob` by hand, so dropping the
    /// `init_resource` or the schedule entry left the whole suite green while a
    /// cast ran for fifteen minutes and never landed.
    #[test]
    fn the_plugin_registers_the_cast_job_and_runs_its_poller() {
        let mut app = app_from_the_plugin();
        app.insert_resource(studio_ready_for_molds());

        // ⚠ `resource_mut` and not `init_resource`: this line is the assertion
        // that the PLUGIN registered it. Adding it here would gate nothing.
        app.world_mut().resource_mut::<MoldsJob>().0 = Some(MoldsRun {
            started: Instant::now(),
            shown_secs: 0,
            task: finished(Ok(some_molds("out-plugin"))),
        });
        app.world_mut().resource_mut::<Studio>().busy = true;

        run_plugin_until_idle(&mut app, "the cast poller");

        assert_eq!(
            app.world()
                .resource::<Studio>()
                .project
                .molds()
                .map(|m| m.out_dir.clone()),
            Some("out-plugin".into()),
            "the cast landed through the plugin's own wiring"
        );
    }

    /// The same hole, for the print export — the only sibling that had it.
    ///
    /// ⚠ Driven through the FAILURE path: a landed export calls
    /// `reveal_in_file_manager`, and a gate that opens a Finder window is one
    /// people learn to skip.
    #[test]
    fn the_plugin_registers_the_print_job_and_runs_its_poller() {
        let mut app = app_from_the_plugin();

        app.world_mut().resource_mut::<PrintJob>().0 =
            Some(AsyncComputeTaskPool::get().spawn(async { Err("the copy failed".to_string()) }));
        app.world_mut().resource_mut::<Studio>().busy = true;

        run_plugin_until_idle(&mut app, "the print poller");

        assert!(
            matches!(&app.world().resource::<Studio>().message,
                     Some(Err(text)) if text.contains("the copy failed")),
            "the export landed through the plugin's own wiring: {:?}",
            app.world().resource::<Studio>().message
        );
    }

    /// An app wired by the plugin alone, with unrelated systems allowed to
    /// fall out.
    ///
    /// ⚠ `ignore` is load-bearing: `Update` also holds the scene and pointer
    /// systems, whose params want a renderer. In Bevy 0.18 a param that fails
    /// validation is an error the default handler PANICS on.
    fn app_from_the_plugin() -> App {
        use bevy::state::app::StatesPlugin;

        let mut app = App::new();
        app.set_error_handler(bevy::ecs::error::ignore);
        app.add_plugins((MinimalPlugins, StatesPlugin, crate::plugin::StudioPlugin));
        app
    }

    /// Run the plugin's `Update` until it hands the app back.
    fn run_plugin_until_idle(app: &mut App, what: &str) {
        run_plugin_update_until(app, what, |app| !app.world().resource::<Studio>().busy);
    }

    /// Run the plugin's `Update` until `done`, or fail saying which poller
    /// never ran.
    ///
    /// ⚠ The schedule by hand, not `app.update()`: `Startup` runs `setup_scene`,
    /// which wants an asset stack these gates have no business standing up.
    fn run_plugin_update_until(app: &mut App, what: &str, done: impl Fn(&App) -> bool) {
        // ⚠ Zero frames is a pass otherwise: the caller's assertion then reads
        // the state it seeded, not the plugin's output.
        assert!(!done(app), "{what} was already done before a frame ran");
        let deadline = Instant::now() + Duration::from_secs(2);
        while !done(app) {
            assert!(Instant::now() < deadline, "the plugin never ran {what}");
            app.world_mut().run_schedule(Update);
        }
    }

    /// The third of the four. Its resources are reached by the wizard gate, but
    /// its *place in the schedule* is not: dropping `poll_simplify_job` from
    /// `Update` leaves every other test green while a finished Simplify never
    /// lands and `busy` sticks on forever.
    #[test]
    fn the_plugin_runs_the_simplify_poller() {
        let mut app = app_from_the_plugin();
        app.world_mut().resource_mut::<SimplifyJob>().0 = Some((
            TARGET,
            AsyncComputeTaskPool::get().spawn(async { Err("the decimator gave up".to_string()) }),
        ));
        app.world_mut().resource_mut::<Studio>().busy = true;

        run_plugin_until_idle(&mut app, "the simplify poller");

        assert!(
            matches!(&app.world().resource::<Studio>().message,
                     Some(Err(text)) if text.contains("the decimator gave up")),
            "the Simplify landed through the plugin's own wiring: {:?}",
            app.world().resource::<Studio>().message
        );
    }

    /// The fourth. Dropping `poll_dialogs` from `Update` means every OS picker
    /// resolves into nothing — the file is chosen and silently discarded.
    ///
    /// ⚠ Driven with a print destination and NO molds recorded, because that
    /// branch reports and returns: it writes no files and spawns no export.
    #[test]
    fn the_plugin_runs_the_dialog_poller() {
        let mut app = app_from_the_plugin();
        app.insert_resource(PendingDialog::resolved(
            DialogKind::PrintDest,
            Some(PathBuf::from("somewhere")),
        ));

        run_plugin_update_until(&mut app, "the dialog poller", |app| {
            app.world().resource::<Studio>().message.is_some()
        });

        assert!(
            matches!(&app.world().resource::<Studio>().message,
                     Some(Err(text)) if text.contains("Make the molds first")),
            "the picked folder was routed through the plugin's own wiring: {:?}",
            app.world().resource::<Studio>().message
        );
    }

    #[test]
    fn the_poller_leaves_an_idle_app_alone() {
        let mut app = app_ready_for_molds();
        app.world_mut().resource_mut::<Studio>().message = Some(Ok("untouched".to_string()));
        app.update();

        let studio = app.world().resource::<Studio>();
        assert!(!studio.busy, "no run, so nothing holds the app");
        assert!(
            matches!(&studio.message, Some(Ok(text)) if text == "untouched"),
            "and nothing rewrites the message: {:?}",
            studio.message
        );
    }

    /// A `.design.toml` holding a stack the step-4 editor does not open on:
    /// **one** layer at 7 mm, against the three the screen starts with.
    const ONE_LAYER_DESIGN: &str = "\
[device_design]
tool_version = \"x\"
generated_at = \"2026-01-01T00:00:00Z\"
schema_version = 1
[scan_ref]
cleaned_stl = \"c.stl\"
[cavity]
inset_m = 0.005
visible = true
[[layers]]
thickness_m = 0.007
material_anchor_key = \"ECOFLEX_00_30\"
slacker_fraction = 0.25
visible = true
";

    /// A project standing on step 4 with everything before it done, so
    /// `set_design` is legal.
    fn ready_for_a_design() -> Studio {
        use cf_studio_core::{PlugDraft, PrepInput, ScanInput};
        let mut studio = Studio::default();
        studio.project.set_scan(ScanInput {
            source_path: "scan.stl".into(),
        });
        let staged = [
            studio.project.set_prep(PrepInput {
                cleaned_stl: "scan.cleaned.stl".into(),
                prep_toml: "scan.prep.toml".into(),
            }),
            // ⚠ 5 mm, matching every fixture design's `inset_m`. A plug that
            // disagreed would put the ignored-inset note on every message here
            // and hide whichever one a test was actually about.
            studio.project.set_plug(PlugDraft {
                cavity_inset_m: 0.005,
                ..PlugDraft::default()
            }),
        ];
        assert!(
            staged.iter().all(Result::is_ok),
            "the fixture must reach step 4: {staged:?}"
        );
        studio
    }

    /// ★ The two halves of "load a file" have to agree. `apply_design` puts the
    /// file's stack on the *project*; the rows are what the "Use this design"
    /// button commits. Rows left on the old stack do not merely look wrong —
    /// the next click replaces the file the user just loaded with them.
    #[test]
    fn a_loaded_design_file_reaches_the_rows_that_would_commit_it() {
        let app = app_after_loading("loaded-design", ONE_LAYER_DESIGN);
        let world = app.world();
        let loaded = world
            .resource::<Studio>()
            .project
            .design()
            .expect("the file must land on the project")
            .layers
            .clone();
        assert_eq!(loaded.len(), 1, "the fixture design is one layer");

        let rows = world.resource::<DesignControls>().layers.drafts();
        assert_eq!(
            rows, loaded,
            "the rows that would be committed must be the ones just loaded"
        );
    }

    /// ★★ `base_mold`'s validated stack, 17.5 / 7.5 / 5 mm — thicknesses this
    /// editor's whole-millimetre steppers cannot hold.
    const VALIDATED_DESIGN: &str = "\
[device_design]
tool_version = \"x\"
generated_at = \"2026-01-01T00:00:00Z\"
schema_version = 1
[scan_ref]
cleaned_stl = \"c.stl\"
[cavity]
inset_m = 0.005
visible = true
[[layers]]
thickness_m = 0.0175
material_anchor_key = \"ECOFLEX_00_30\"
slacker_fraction = 0.25
visible = true
[[layers]]
thickness_m = 0.0075
material_anchor_key = \"DRAGON_SKIN_10A\"
slacker_fraction = 0.0
visible = true
[[layers]]
thickness_m = 0.005
material_anchor_key = \"DRAGON_SKIN_20A\"
slacker_fraction = 0.0
visible = true
";

    /// What the app is reporting after a design load, failing on the error it
    /// carries rather than on a bare `None`.
    fn loaded_message(app: &App) -> String {
        app.world()
            .resource::<Studio>()
            .message
            .clone()
            .expect("the load must report something")
            .expect("and the design must have loaded")
    }

    /// Load `toml` through the design picker and hand back the finished app.
    fn app_after_loading(label: &str, toml: &str) -> App {
        let design = crate::save::tests::temp_dir(label).join("fixture.design.toml");
        std::fs::write(&design, toml).expect("the fixture must be writable");

        let mut app = App::new();
        app.add_plugins(TaskPoolPlugin::default())
            .insert_resource(ready_for_a_design())
            .init_resource::<PrintJob>()
            .init_resource::<ScanEdit>()
            .init_resource::<DesignControls>()
            .insert_resource(PendingDialog::resolved(
                DialogKind::DesignFile,
                Some(design),
            ))
            .add_systems(Update, poll_dialogs);
        run_until_answered(&mut app);
        app
    }

    /// ★★ The design the app is *for* is the one it cannot show. Both halves
    /// are the point: the rows follow the file as closely as the steppers
    /// allow, **and** the message says they had to round it — because the
    /// button beside them would then write 18 / 8 / 5 back over 17.5 / 7.5 / 5.
    #[test]
    fn a_design_the_steppers_cannot_hold_is_shown_clamped_and_reported() {
        let app = app_after_loading("validated-design", VALIDATED_DESIGN);
        let world = app.world();

        let shown: Vec<i32> = world
            .resource::<DesignControls>()
            .layers
            .rows()
            .iter()
            .map(|row| row.thickness_mm.value())
            .collect();
        assert_eq!(shown, vec![18, 8, 5], "the rows round to whole millimetres");

        let message = loaded_message(&app);
        assert!(
            message.contains("3 layer(s)") && message.contains("Not exactly this file"),
            "the message reports the load AND that the rows are not it: {message}"
        );
    }

    /// ★ The case that rewrote this message. `load_design_toml` accepts a
    /// 150 mm layer; the stepper's ceiling is 100. The note used to call that
    /// "rounded to the nearest millimetre" — a 50 mm discrepancy reported as a
    /// sub-millimetre one, by the only line standing between the user and the
    /// overwrite.
    #[test]
    fn a_layer_the_loader_accepts_but_the_steppers_cannot_hold_is_still_flagged() {
        let app = app_after_loading(
            "oversize-design",
            &ONE_LAYER_DESIGN.replace("0.007", "0.15"),
        );

        let shown: Vec<i32> = app
            .world()
            .resource::<DesignControls>()
            .layers
            .rows()
            .iter()
            .map(|row| row.thickness_mm.value())
            .collect();
        assert_eq!(shown, vec![100], "clamped to the stepper's ceiling");

        let message = loaded_message(&app);
        assert!(
            message.contains("Not exactly this file") && !message.contains("round"),
            "and reported without claiming a rounding: {message}"
        );
    }

    /// The other side of it: a design the steppers *can* hold is reported
    /// without a warning, or the warning stops meaning anything.
    #[test]
    fn a_design_the_steppers_can_hold_is_reported_without_a_warning() {
        let app = app_after_loading("exact-design", ONE_LAYER_DESIGN);
        let message = loaded_message(&app);
        assert_eq!(
            message, "✔ Design set: 1 layer(s), 5.0 mm cavity inset.",
            "nothing was changed to show it, so the whole message is the report",
        );
    }
}
