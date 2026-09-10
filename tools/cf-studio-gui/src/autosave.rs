//! The project file beside the scan: written as the session goes, and read back
//! when the same scan is picked again.
//!
//! ★★★ The writer and the reader are one piece, because a writer without a
//! reader is a data-loss machine: it puts this session's empty project over the
//! previous session's work the moment that scan is picked again. So a pick
//! *reads first*, and holds every write until the question that read raised has
//! been answered.

use std::path::{Path, PathBuf};

use bevy::prelude::*;
use cf_studio_core::{Project, Step};
use cf_studio_gui::{ResumeOffer, autosave_path, inspect_autosave, worth_keeping};

use crate::scan::{ActiveScan, ScanEdit};
use crate::state::Studio;

/// Whether the session is writing its project to disk — and, when it is not,
/// why not.
#[derive(Default)]
pub(crate) enum Saving {
    /// Nothing is holding the write. See [`Autosave::write`] for what it then
    /// takes to actually write.
    #[default]
    Yes,
    /// ⛔ Suspended: a saved session was found for this scan and the question is
    /// on screen. The file being offered back is the file a write would
    /// destroy, so nothing may be written until the user answers.
    Asking {
        /// The session on offer.
        project: Box<Project>,
        /// How far it got — the question's whole subject. Carried rather than
        /// re-derived, so "a question is up" cannot be answered by looking at
        /// what the project happens to contain.
        reached: Step,
    },
    /// ⛔ Off: a file this build cannot read sits where the project would go.
    /// Overwriting it would throw away work this build merely fails to
    /// understand.
    Refusing {
        /// Where that file is. Carried, because the note has to tell the user
        /// which file to move before saving can start — and the only other
        /// place to get it is [`Autosave::path`], where a `None` this variant
        /// already rules out would need an arm nothing can reach.
        file: String,
        /// The engine's reason, for the screen.
        why: String,
    },
}

/// Where this session's project is saved, and what it last wrote there.
#[derive(Resource, Default)]
pub(crate) struct Autosave {
    /// The file written to; `None` until a scan is picked.
    ///
    /// ⚠ Carried, not re-derived from the project each frame. A resumed project
    /// records the scan path it was *saved* with, and that scan may since have
    /// moved — re-deriving would write the file back to where the scan used to
    /// be. The same rule as [`crate::state::PendingSave`]'s `dir`.
    path: Option<PathBuf>,
    /// The project as of the last write attempt, whether it worked or not.
    ///
    /// ⚠ The dirty check, and it records failures on purpose: a folder that
    /// refuses the write would otherwise be hammered sixty times a second while
    /// the note on screen already says the work is not being saved.
    last_attempt: Option<Project>,
    state: Saving,
    /// What the last failed write reported. Cleared by the next one that works.
    failed: Option<String>,
}

impl Autosave {
    /// Point the autosave at the file for `scan`, and read what is already
    /// there.
    ///
    /// ★★★ The read is here, at the pick, and every write is held on its
    /// answer. Writing first would land this session's empty project on the
    /// resume candidate before the offer ever reached the screen.
    pub(crate) fn follow(&mut self, scan: &Path) {
        let path = autosave_path(scan);
        self.state = match inspect_autosave(&path) {
            ResumeOffer::Fresh => Saving::Yes,
            ResumeOffer::Resumable { project, reached } => Saving::Asking { project, reached },
            ResumeOffer::Unreadable(why) => Saving::Refusing {
                // ⚠ The whole path, not the filename: it is what the user has
                // to go and find, and every scan folder holds one of these.
                file: path.display().to_string(),
                why,
            },
        };
        self.path = Some(path);
        // A different file, which this session has never written.
        self.last_attempt = None;
        self.failed = None;
    }

    /// How far the saved session waiting for an answer got — `None` when none
    /// is waiting.
    pub(crate) const fn asking_about(&self) -> Option<Step> {
        match self.state {
            Saving::Asking { reached, .. } => Some(reached),
            _ => None,
        }
    }

    /// An [`Autosave`] with `project` offered back and the question still
    /// unanswered, so a test can prove a control is gated on one — and that the
    /// question reaches the screen — without standing a file up on disk.
    ///
    /// ⚠ `reached` is given rather than derived from `project`: deriving it
    /// would reproduce [`inspect_autosave`]'s own decision here, and then agree
    /// with it by construction.
    #[cfg(test)]
    pub(crate) fn asking(project: Project, reached: Step) -> Self {
        Self {
            state: Saving::Asking {
                project: Box::new(project),
                reached,
            },
            ..Self::default()
        }
    }

    /// What the screen has to say about saving; `None` when there is nothing to
    /// say.
    ///
    /// ⚠ Not [`Studio::message`]: that is wiped by the next step action, and
    /// "your work is not being saved" has to outlive the click after it.
    pub(crate) fn note(&self) -> Option<String> {
        match (&self.state, &self.failed) {
            (Saving::Refusing { file, why }, _) => Some(format!(
                "⚠ Your work isn't being saved: this version can't read \
                 {file}, so it won't be overwritten. Move or rename that file \
                 to start saving here. ({why})"
            )),
            (_, Some(why)) => Some(format!("⚠ Your work isn't being saved: {why}")),
            _ => None,
        }
    }

    /// Take the offered session and resume writing, if a question is up.
    ///
    /// ⚠ Puts the state back when there is no offer. `mem::take` leaves
    /// [`Saving::Yes`] behind, so a bare `take` would answer for a session that
    /// is *refusing* to write and set it saving over the file it is protecting.
    fn take_offer(&mut self) -> Option<Box<Project>> {
        match std::mem::take(&mut self.state) {
            Saving::Asking { project, .. } => Some(project),
            held => {
                self.state = held;
                None
            }
        }
    }

    /// Write `project`, unless writing is held, it is what was last written, or
    /// there is nothing in it yet worth a file.
    fn write(&mut self, project: &Project) {
        let (Saving::Yes, Some(path)) = (&self.state, &self.path) else {
            return;
        };
        // ★ The same bar the reader uses. A scan picked and looked at records
        // nothing the pick would not reproduce, and `~/scans` is flat — every
        // scan the user ever clicked would leave a file there for nothing.
        if worth_keeping(project).is_none() {
            return;
        }
        if self.last_attempt.as_ref() == Some(project) {
            return;
        }
        self.failed = project.save(path).err().map(|e| e.to_string());
        self.last_attempt = Some(project.clone());
    }
}

/// Save the project whenever it stops matching what is on disk.
pub(crate) fn drive_autosave(mut autosave: ResMut<Autosave>, studio: Res<Studio>) {
    autosave.write(&studio.project);
}

/// Take the offered session: it becomes this session's project, and the body it
/// was cut from comes back on screen.
pub(crate) fn resume(autosave: &mut Autosave, studio: &mut Studio, scan: &mut ScanEdit) {
    let Some(project) = autosave.take_offer() else {
        return;
    };
    // ⚠ The path is left as it is. It already names the file this project came
    // from, and that is not the same as the file its own scan path would give:
    // the scan may have moved since it was saved.
    studio.resume(*project);
    // The folder may have changed since this project was last open — parts
    // deleted, or a cast run from the CLI. Read it now rather than trust
    // anything the last session recorded.
    studio.refresh_folder_provenance();
    studio.say(match reload_body(&studio.project, scan) {
        None => Ok(format!(
            "✔ Picked up where you left off — step {} of {}.",
            studio.project.current_step().number(),
            Step::TOTAL
        )),
        // ⚠ Not a failed resume. The project is the work; the scan file is only
        // where the body came from, and it may have moved or been deleted since.
        Some(why) => Err(format!(
            "Picked up where you left off, but the scan couldn't be shown: {why}"
        )),
    });
}

/// Keep this session's own project, and let it replace the saved one.
///
/// The offer is dropped, and [`Autosave::take_offer`] resumes writing — which
/// is what makes this session's project replace the saved one on the next frame.
pub(crate) fn start_over(autosave: &mut Autosave) {
    autosave.take_offer();
}

/// Put the resumed project's body back on screen. Returns why it could not be,
/// when it could not.
///
/// ⚠ Synchronous, like step 1's own load: [`crate::jobs::poll_dialogs`] already
/// calls [`ActiveScan::load`] on the main thread for exactly this.
fn reload_body(project: &Project, scan: &mut ScanEdit) -> Option<String> {
    // ★ The cleaned scan, and only ever that: a session is offered back only
    // once it is past the pick, so step 2 is always done by the time this runs.
    // The raw scan is what step 1 shows, and it is not what this session was
    // looking at.
    let source = project.prep()?.cleaned_stl.clone();
    match ActiveScan::load(&source) {
        Ok(active) => {
            scan.set(active);
            None
        }
        Err(why) => Some(why),
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::expect_used)]

    use bevy::ecs::system::RunSystemOnce;
    use cf_studio_core::{PlugDraft, PrepInput, ScanInput};

    use super::*;
    use crate::save::tests::temp_dir;

    /// An ASCII STL of `faces` disjoint triangles — enough for
    /// [`ActiveScan::load`], and countable once it is loaded.
    fn stl(faces: usize) -> String {
        (0..faces)
            .map(|n| {
                #[allow(clippy::cast_precision_loss)] // A handful of triangles.
                let z = n as f32;
                format!(
                    "facet normal 0 0 1\n  outer loop\n    vertex 0 0 {z}\n    \
                     vertex 1 0 {z}\n    vertex 0 1 {z}\n  endloop\nendfacet\n"
                )
            })
            .fold("solid t\n".to_string(), |body, facet| body + &facet)
            + "endsolid t\n"
    }

    /// The raw scan's face count, and the cleaned scan's.
    ///
    /// ⚠ Different, and that is the whole point: which of the two a resume puts
    /// back on screen is a decision, and two identical files cannot tell.
    const RAW_FACES: usize = 1;
    const CLEANED_FACES: usize = 3;

    /// A folder with a scan and its cleaned twin, both real enough to load and
    /// told apart by their face counts.
    fn scans(name: &str) -> PathBuf {
        let dir = temp_dir(name);
        std::fs::write(dir.join("base.stl"), stl(RAW_FACES)).expect("a scan to load");
        std::fs::write(dir.join("base.cleaned.stl"), stl(CLEANED_FACES))
            .expect("a cleaned scan to load");
        dir
    }

    /// How many faces the viewport is showing.
    fn faces_on_screen(scan: &ScanEdit) -> Option<usize> {
        scan.active().map(|active| active.display().faces.len())
    }

    /// A project taken as far as `furthest`, pointing at the files in `dir`.
    fn walked_to(dir: &Path, furthest: Step) -> Project {
        let mut project = Project::new("resume gate");
        project.set_scan(ScanInput {
            source_path: dir.join("base.stl"),
        });
        if furthest >= Step::CleanScan {
            project
                .set_prep(PrepInput {
                    cleaned_stl: dir.join("base.cleaned.stl"),
                    prep_toml: dir.join("base.prep.toml"),
                })
                .expect("in workflow order");
        }
        if furthest >= Step::ShapePiece {
            project
                .set_plug(PlugDraft::default())
                .expect("in workflow order");
        }
        assert_eq!(
            project.furthest_completed(),
            Some(furthest),
            "the fixture must stop exactly where it says"
        );
        project
    }

    /// This session, one frame after the scan in `dir` was picked — in
    /// [`crate::jobs::poll_dialogs`]'s own order: record, then follow.
    fn just_picked(dir: &Path) -> (Autosave, Studio) {
        let scan = dir.join("base.stl");
        let mut studio = Studio::default();
        studio.record_scan(&scan).expect("the fixture scan loads");
        let mut autosave = Autosave::default();
        autosave.follow(&scan);
        (autosave, studio)
    }

    /// Record step 2's artifact — the first thing a session has that is worth a
    /// file.
    ///
    /// ⚠ Plain data: the setter gates on the previous step, not on anything
    /// being on disk.
    fn clean_the_scan(studio: &mut Studio, dir: &Path) {
        studio
            .project
            .set_prep(PrepInput {
                cleaned_stl: dir.join("base.cleaned.stl"),
                prep_toml: dir.join("base.prep.toml"),
            })
            .expect("in workflow order");
    }

    /// This session with the scan in `dir` picked *and* cleaned — the first
    /// point at which there is anything to save.
    fn picked_and_cleaned(dir: &Path) -> (Autosave, Studio) {
        let (autosave, mut studio) = just_picked(dir);
        clean_the_scan(&mut studio, dir);
        (autosave, studio)
    }

    /// Run the real autosave system `frames` times, and hand back what it left.
    fn drive(autosave: Autosave, studio: Studio, frames: usize) -> (Autosave, Studio) {
        let mut app = App::new();
        app.insert_resource(autosave).insert_resource(studio);
        for _ in 0..frames {
            app.world_mut()
                .run_system_once(drive_autosave)
                .expect("the autosave must run");
        }
        (
            app.world_mut()
                .remove_resource::<Autosave>()
                .expect("the autosave survives"),
            app.world_mut()
                .remove_resource::<Studio>()
                .expect("the studio survives"),
        )
    }

    /// ★★★ The whole reason the writer and the reader had to land together. A
    /// write on the frame the scan is picked lands this session's empty project
    /// on the previous session's file — and the offer then has nothing to
    /// offer, with no error anywhere and no way back.
    ///
    /// ⚠ Several frames, not one. The suspension has to hold for as long as the
    /// question is on screen, which is however long the user takes to read it.
    #[test]
    fn a_saved_session_is_untouched_until_the_question_is_answered() {
        let dir = scans("hazard");
        let path = autosave_path(&dir.join("base.stl"));
        walked_to(&dir, Step::ShapePiece)
            .save(&path)
            .expect("the previous session");
        let before = std::fs::read_to_string(&path).expect("a file to protect");

        // ⚠ Cleaned, not just picked. A session holding nothing yet is not
        // written at all, so the suspension would never be reached — and this
        // gate would pass with it deleted. What it protects is a named file,
        // whatever the session happens to hold.
        let (autosave, studio) = picked_and_cleaned(&dir);
        let (autosave, _) = drive(autosave, studio, 5);

        assert_eq!(
            autosave.asking_about(),
            Some(Step::ShapePiece),
            "the question is up, about the step the saved session reached"
        );
        assert_eq!(
            std::fs::read_to_string(&path).expect("still there"),
            before,
            "and the file it is about has not been written to"
        );
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// ★★ What resume is for, all four parts: the project, the screen it was
    /// left on, the body that was on it, and saving picking back up.
    #[test]
    fn resuming_takes_the_saved_session_whole_and_starts_saving_again() {
        let dir = scans("resume");
        let saved = walked_to(&dir, Step::CleanScan);
        let path = autosave_path(&dir.join("base.stl"));
        saved.save(&path).expect("the previous session");

        let (mut autosave, mut studio) = just_picked(&dir);
        let mut scan = ScanEdit::default();
        resume(&mut autosave, &mut studio, &mut scan);

        assert_eq!(studio.project, saved, "the project is the one on disk");
        assert_eq!(
            studio.cursor.viewed(),
            Step::CleanScan,
            "on the screen the file says it was left on"
        );
        assert_eq!(
            faces_on_screen(&scan),
            Some(CLEANED_FACES),
            "with the cleaned scan back in the viewport — step 2 is done, so the \
             raw one is not what this session was looking at"
        );
        assert!(
            studio.outcome().is_some_and(Result::is_ok),
            "and said so: {:?}",
            studio.outcome()
        );
        assert_eq!(autosave.asking_about(), None, "the question is answered");

        // ⚠ And saving resumes, where it read from. A resume that left writing
        // suspended would silently stop recording everything done from here on.
        studio.project.name = "carried on".to_string();
        let (autosave, studio) = drive(autosave, studio, 1);
        assert_eq!(
            Project::load(&path).expect("readable"),
            studio.project,
            "what happens after the resume is saved too: {:?}",
            autosave.note()
        );
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// ⚠ The other answer, and the one that destroys something. It has to
    /// actually replace the file — an answer that only dismissed the modal
    /// would leave the session unsaved with nothing on screen to say so.
    #[test]
    fn starting_over_lets_this_sessions_project_replace_the_saved_one() {
        let dir = scans("start-over");
        let path = autosave_path(&dir.join("base.stl"));
        walked_to(&dir, Step::ShapePiece)
            .save(&path)
            .expect("the previous session");

        let (mut autosave, studio) = picked_and_cleaned(&dir);
        start_over(&mut autosave);
        let (_, studio) = drive(autosave, studio, 1);

        let back = Project::load(&path).expect("the file is readable");
        assert_eq!(back, studio.project, "the file is this session's project");
        assert_eq!(
            back.furthest_completed(),
            Some(Step::CleanScan),
            "and the saved session is gone, which is what was asked for"
        );
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// ★ Both halves of the dirty check, and the sentinel is what makes the
    /// first half testable: a write nobody asked for is invisible unless
    /// something is put in the file to be destroyed.
    #[test]
    fn the_project_is_written_when_it_changes_and_not_when_it_has_not() {
        let dir = scans("dirty-check");
        let path = autosave_path(&dir.join("base.stl"));
        let (autosave, studio) = picked_and_cleaned(&dir);
        let (autosave, studio) = drive(autosave, studio, 1);
        assert!(
            path.is_file(),
            "the session is saved: {:?}",
            autosave.note()
        );

        const SENTINEL: &str = "not written by the autosave";
        std::fs::write(&path, SENTINEL).expect("a sentinel to destroy");
        let (autosave, mut studio) = drive(autosave, studio, 30);
        assert_eq!(
            std::fs::read_to_string(&path).expect("still there"),
            SENTINEL,
            "an unchanged project is not written 30 times over"
        );

        studio
            .project
            .set_plug(PlugDraft::default())
            .expect("in workflow order");
        let (_, studio) = drive(autosave, studio, 1);

        assert_eq!(
            Project::load(&path).expect("readable"),
            studio.project,
            "and a change does reach the file"
        );
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// ⚠ A folder that refuses the write must not be hammered at frame rate —
    /// and must be tried again the moment there is something new to save. The
    /// dirty check is what does both, which is why a failure records an attempt.
    #[test]
    fn a_failed_write_is_not_retried_until_there_is_something_new_to_save() {
        let dir = scans("failed-write");
        let path = autosave_path(&dir.join("base.stl"));
        let (autosave, studio) = picked_and_cleaned(&dir);
        // The folder goes out from under the write, which is what a read-only
        // or unmounted one does to it.
        std::fs::remove_dir_all(&dir).expect("take the folder away");

        let (autosave, studio) = drive(autosave, studio, 1);
        assert!(
            autosave
                .note()
                .is_some_and(|note| note.contains("isn't being saved")),
            "the screen says the work is not being saved: {:?}",
            autosave.note()
        );

        std::fs::create_dir_all(&dir).expect("and the folder comes back");
        let (autosave, mut studio) = drive(autosave, studio, 30);
        assert!(
            !path.exists(),
            "an unchanged project is not retried every frame"
        );

        studio.project.name = "renamed".to_string();
        let (_, studio) = drive(autosave, studio, 1);
        assert_eq!(
            Project::load(&path).expect("readable"),
            studio.project,
            "but the next real change tries again"
        );
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// ★★★ A file this build cannot read is somebody's work. Treating it as
    /// absent — which is what a `Fresh` here would mean — overwrites a session
    /// written by a newer Cendrillon with one this build can produce.
    ///
    /// ⚠ And it is NOT a question: there is nothing to offer back, so the app
    /// must not sit behind a modal with no answer that helps.
    #[test]
    fn a_project_file_this_build_cannot_read_is_never_written_over() {
        let dir = scans("unreadable");
        let path = autosave_path(&dir.join("base.stl"));
        const HAND_EDITED: &str = "{ not json at all";
        std::fs::write(&path, HAND_EDITED).expect("a file to protect");

        let (autosave, studio) = picked_and_cleaned(&dir);
        let (autosave, _) = drive(autosave, studio, 30);

        assert_eq!(
            std::fs::read_to_string(&path).expect("still there"),
            HAND_EDITED,
            "the file is left exactly as it was"
        );
        assert_eq!(autosave.asking_about(), None, "and no question is raised");
        // ⚠ Names the file, and says what to do about it. A note that only
        // reports the trouble leaves a non-technical user with no next step —
        // and every scan folder holds a file with this name.
        let note = autosave.note().unwrap_or_default();
        assert!(
            note.contains(&path.display().to_string()) && note.contains("Move or rename"),
            "the screen says which file, and how to get saving again: {note}"
        );
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// ⚠⚠ The answer to a question that was never asked. `mem::take` leaves
    /// `Saving::Yes` behind, so taking the offer without putting the state back
    /// turns a session that is *refusing* to write into one that writes — over
    /// the very file it was refusing to touch.
    ///
    /// ⚠ Both answers, because they take the same path and either one alone
    /// leaves the other free to do it.
    #[test]
    fn answering_a_question_that_was_never_asked_starts_nothing() {
        for (what, answer) in [
            (
                "resume",
                &resume as &dyn Fn(&mut Autosave, &mut Studio, &mut ScanEdit),
            ),
            (
                "start over",
                &|autosave: &mut Autosave, _: &mut Studio, _: &mut ScanEdit| start_over(autosave),
            ),
        ] {
            let dir = scans("never-asked");
            let path = autosave_path(&dir.join("base.stl"));
            const HAND_EDITED: &str = "{ not json at all";
            std::fs::write(&path, HAND_EDITED).expect("a file to protect");
            let (mut autosave, mut studio) = picked_and_cleaned(&dir);
            assert_eq!(autosave.asking_about(), None, "nothing is being asked");

            answer(&mut autosave, &mut studio, &mut ScanEdit::default());
            let (_, _) = drive(autosave, studio, 5);

            assert_eq!(
                std::fs::read_to_string(&path).expect("still there"),
                HAND_EDITED,
                "{what} must not talk a refusing session into writing"
            );
            let _ = std::fs::remove_dir_all(&dir);
        }
    }

    /// Before a scan is picked there is nowhere to write, and the session must
    /// not go looking for one.
    #[test]
    fn a_session_with_no_scan_yet_attempts_no_write_at_all() {
        let (autosave, _) = drive(Autosave::default(), Studio::default(), 30);

        assert!(autosave.path.is_none(), "no file is named");
        assert!(
            autosave.last_attempt.is_none(),
            "and no write was even attempted"
        );
        assert!(autosave.note().is_none(), "so there is nothing to report");
    }

    /// ★ A second pick moves the file — and the record of what was last written
    /// has to go with it. Left behind, the two sessions' projects compare equal
    /// on the frame after the pick and the second scan's file is never written.
    #[test]
    fn picking_a_second_scan_starts_saving_beside_that_one() {
        let first = scans("second-pick-first");
        let second = scans("second-pick-second");
        let (autosave, studio) = picked_and_cleaned(&first);
        let (mut autosave, mut studio) = drive(autosave, studio, 1);
        let first_file = autosave_path(&first.join("base.stl"));
        assert!(first_file.is_file(), "the first session is saved");

        let scan = second.join("base.stl");
        studio.record_scan(&scan).expect("the second scan loads");
        autosave.follow(&scan);
        clean_the_scan(&mut studio, &second);
        let (_, studio) = drive(autosave, studio, 1);

        let second_file = autosave_path(&scan);
        assert_eq!(
            Project::load(&second_file).expect("readable"),
            studio.project,
            "the session is saved beside the scan it is now working from"
        );
        assert_ne!(
            Project::load(&first_file).expect("readable"),
            studio.project,
            "and the first scan's file is left as the first session wrote it"
        );
        let _ = std::fs::remove_dir_all(&first);
        let _ = std::fs::remove_dir_all(&second);
    }

    /// ★ Redoing the same work on the same scan is how a user gets back a
    /// project file that has gone missing — deleted by hand, or on a drive that
    /// came back empty. The record of what was last written has to go with the
    /// pick, or the redone session compares equal to a file that is no longer
    /// there and is never written back.
    ///
    /// ⚠ The precondition is asserted, because it is the whole trap: the redone
    /// project has to be *identical* to the one already recorded, or the dirty
    /// check lets the write through for a reason that has nothing to do with
    /// the pick clearing it.
    #[test]
    fn redoing_the_work_writes_the_session_back_to_a_file_that_has_gone() {
        let dir = scans("re-pick");
        let scan = dir.join("base.stl");
        let path = autosave_path(&scan);
        let (autosave, studio) = picked_and_cleaned(&dir);
        let (mut autosave, mut studio) = drive(autosave, studio, 1);
        assert!(path.is_file(), "the first session is saved");
        let first = studio.project.clone();

        std::fs::remove_file(&path).expect("the file goes missing");
        studio
            .record_scan(&scan)
            .expect("the same scan, picked again");
        autosave.follow(&scan);
        clean_the_scan(&mut studio, &dir);
        assert_eq!(
            studio.project, first,
            "the redone session must be indistinguishable from the recorded one"
        );

        let (_, studio) = drive(autosave, studio, 1);

        assert_eq!(
            Project::load(&path).expect("the file is back"),
            studio.project,
            "redoing the work wrote it back"
        );
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// ★★ A scan picked and looked at is not work. Writing it would drop a
    /// project file beside every scan the user ever clicked — into `~/scans`,
    /// which is flat — and the reader would classify every one of them as
    /// nothing and ignore it.
    #[test]
    fn a_scan_picked_and_looked_at_leaves_no_file_behind() {
        let dir = scans("just-looking");
        let path = autosave_path(&dir.join("base.stl"));
        let (autosave, studio) = just_picked(&dir);

        let (autosave, mut studio) = drive(autosave, studio, 30);

        assert!(
            !path.exists(),
            "nothing is left beside the scan: {:?}",
            autosave.note()
        );

        // ...and the first real step is saved, or nothing ever would be.
        clean_the_scan(&mut studio, &dir);
        let (_, studio) = drive(autosave, studio, 1);
        assert_eq!(
            Project::load(&path).expect("readable"),
            studio.project,
            "the first step past the pick is what starts the file"
        );
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// ⚠⚠ The wiring no mutation sweep reaches, and the reason nothing about
    /// staleness is persisted: a project reopened days later must read the
    /// folder as it is NOW. If `resume` does not re-read, the panel is silent
    /// about a folder full of parts from three different casts.
    ///
    /// ⚠ The manifest is a LITERAL here, not built by the code that writes
    /// manifests — an oracle produced by the system under test agrees with it
    /// whatever either of them does.
    #[test]
    fn a_resumed_session_re_reads_what_the_output_folder_holds() {
        use cf_studio_core::{DesignDraft, LayerDraft, MoldOutputs, PourPlan};

        let dir = scans("resume-provenance");
        let stls = dir.join("out").join("stls");
        std::fs::create_dir_all(&stls).expect("an output folder");
        std::fs::write(stls.join("fresh.stl"), b"solid\n").expect("a fresh part");
        std::fs::write(stls.join("old.stl"), b"solid\n").expect("an older part");
        std::fs::write(
            stls.join("manifest.toml"),
            "latest_run = 2\n\n[[part]]\nfile = \"fresh.stl\"\nrun = 2\n\n\
             [[part]]\nfile = \"old.stl\"\nrun = 1\n",
        )
        .expect("a manifest");

        let mut saved = walked_to(&dir, Step::ShapePiece);
        saved
            .set_design(DesignDraft {
                cavity_inset_m: PlugDraft::default().cavity_inset_m,
                layers: vec![LayerDraft {
                    thickness_m: 0.0175,
                    material_key: "ECOFLEX_00_30".to_string(),
                    slacker_fraction: 0.25,
                }],
            })
            .expect("in workflow order");
        saved
            .set_molds(MoldOutputs {
                out_dir: dir.join("out"),
                mold_stls: vec![stls.join("fresh.stl")],
                plug_stls: Vec::new(),
                accessory_stls: Vec::new(),
                procedure_path: dir.join("out/procedure.md"),
                total_mass_g: 80.0,
                pour_plan: PourPlan { steps: Vec::new() },
            })
            .expect("in workflow order");
        saved
            .save(&autosave_path(&dir.join("base.stl")))
            .expect("the previous session");

        let (mut autosave, mut studio) = just_picked(&dir);
        assert!(
            studio.stale.is_none(),
            "nothing is known before the project is back"
        );

        resume(&mut autosave, &mut studio, &mut ScanEdit::default());

        let provenance = studio
            .stale
            .as_ref()
            .expect("the folder carries a manifest, so this is not unknown");
        assert_eq!(provenance.run, 2, "the manifest's latest run");
        assert_eq!(
            provenance
                .stale
                .iter()
                .map(|e| e.file.as_str())
                .collect::<Vec<_>>(),
            vec!["old.stl"],
            "the part that run did not write"
        );
    }

    /// ⚠⚠ The autosave keeps writing where it read from. A resumed project
    /// records the scan path it was *saved* with, and that scan may have moved
    /// since — re-deriving the file from it would write this session back to
    /// where the scan used to be, leaving the file it resumed from frozen.
    #[test]
    fn a_resumed_session_keeps_saving_where_it_was_read_from() {
        let here = scans("moved-scan-here");
        let gone = temp_dir("moved-scan-gone");
        // Saved when the scan lived in `gone`; picked up now that it is in `here`.
        let saved = walked_to(&gone, Step::ShapePiece);
        let path = autosave_path(&here.join("base.stl"));
        saved.save(&path).expect("the previous session");

        let (mut autosave, mut studio) = just_picked(&here);
        resume(&mut autosave, &mut studio, &mut ScanEdit::default());
        studio.project.name = "carried on".to_string();
        let (_, studio) = drive(autosave, studio, 1);

        assert_eq!(
            Project::load(&path).expect("readable"),
            studio.project,
            "the file it was read from is the file it goes on writing"
        );
        assert!(
            !autosave_path(&gone.join("base.stl")).exists(),
            "and nothing is written where the scan used to be"
        );
        let _ = std::fs::remove_dir_all(&here);
        let _ = std::fs::remove_dir_all(&gone);
    }

    /// ★★★ The whole workflow, one step at a time, through the real system:
    /// after every step the file beside the scan holds what the session holds,
    /// and a fresh session picking that scan back up gets all of it.
    ///
    /// ⚠ Every other gate here exercises one or two steps. This is the only one
    /// that asks whether a *late* artifact survives the round trip — a step
    /// whose artifact failed to serialize, or that the loader's invariants
    /// rejected, would stay invisible until a user lost a finished cast.
    #[test]
    fn every_step_of_a_whole_session_reaches_the_file_and_comes_back() {
        use cf_studio_core::{
            DesignDraft, LayerDraft, MoldOutputs, PourPlan, PourRecord, PourStep, PrintExport,
            RidgeOptions,
        };

        let dir = scans("whole-session");
        let scan = dir.join("base.stl");
        let path = autosave_path(&scan);
        let (autosave, studio) = just_picked(&dir);
        let (mut autosave, mut studio) = drive(autosave, studio, 1);
        assert!(!path.exists(), "the pick on its own writes nothing");

        let steps: [(&str, fn(&mut Project, &Path)); 6] = [
            ("clean the scan", |p, dir| {
                p.set_prep(PrepInput {
                    cleaned_stl: dir.join("base.cleaned.stl"),
                    prep_toml: dir.join("base.prep.toml"),
                })
                .expect("in workflow order");
            }),
            ("shape the piece", |p, _| {
                p.set_plug(PlugDraft {
                    cavity_inset_m: 0.005,
                    ridges: RidgeOptions::default(),
                })
                .expect("in workflow order");
            }),
            ("choose the layers", |p, _| {
                p.set_design(DesignDraft {
                    // ⚠ The plug's, or `Project::load` refuses the pair (#899).
                    cavity_inset_m: 0.005,
                    layers: vec![LayerDraft {
                        thickness_m: 0.0175,
                        material_key: "ECOFLEX_00_30".to_string(),
                        slacker_fraction: 0.25,
                    }],
                })
                .expect("in workflow order");
            }),
            ("make the molds", |p, dir| {
                p.set_molds(MoldOutputs {
                    out_dir: dir.join("out"),
                    mold_stls: vec![dir.join("out/mold.stl")],
                    plug_stls: vec![dir.join("out/plug.stl")],
                    accessory_stls: Vec::new(),
                    procedure_path: dir.join("out/procedure.md"),
                    total_mass_g: 842.0,
                    pour_plan: PourPlan {
                        steps: vec![PourStep {
                            layer_index: 0,
                            material_display_name: "Ecoflex 00-30".to_string(),
                            mass_g: 500.0,
                            mix_ratio_a_to_b: "1:1".to_string(),
                            pot_life_minutes: 25,
                            cure_time_hours: 4.0,
                            slacker_fraction: Some(0.25),
                        }],
                    },
                })
                .expect("in workflow order");
            }),
            ("print them", |p, dir| {
                p.set_print(PrintExport {
                    export_dir: dir.join("print"),
                })
                .expect("in workflow order");
            }),
            ("pour the silicone", |p, _| {
                p.set_pour(PourRecord { layers_poured: 1 })
                    .expect("in workflow order");
            }),
        ];

        for (what, apply) in steps {
            apply(&mut studio.project, &dir);
            let (back, on) = drive(autosave, studio, 1);
            (autosave, studio) = (back, on);
            assert_eq!(
                Project::load(&path).expect("the file is readable"),
                studio.project,
                "the file holds the session after: {what} ({:?})",
                autosave.note()
            );
        }
        assert!(studio.project.is_finished(), "the fixture ran the workflow");

        // ...and a fresh session picking the same scan back up gets all of it.
        let saved = studio.project.clone();
        let mut later = Studio::default();
        later
            .record_scan(&scan)
            .expect("the same scan, next launch");
        let mut autosave = Autosave::default();
        autosave.follow(&scan);
        assert_eq!(
            autosave.asking_about(),
            Some(Step::Pour),
            "the finished session is what is offered back"
        );

        resume(&mut autosave, &mut later, &mut ScanEdit::default());

        assert_eq!(later.project, saved, "every step of it came back");
        assert_eq!(
            later.cursor.viewed(),
            Step::Pour,
            "on the screen it was left on"
        );
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// The plugin's wiring: every gate above runs the system by hand. Left out
    /// of the schedule it never runs at all, nothing is ever saved, and all of
    /// them stay green.
    #[test]
    fn the_plugin_runs_the_autosave() {
        use bevy::state::app::StatesPlugin;

        let dir = scans("plugin-wiring");
        let path = autosave_path(&dir.join("base.stl"));
        let (autosave, studio) = picked_and_cleaned(&dir);
        let mut app = App::new();
        app.set_error_handler(bevy::ecs::error::ignore);
        app.add_plugins((MinimalPlugins, StatesPlugin, crate::plugin::StudioPlugin));
        app.insert_resource(autosave);
        app.insert_resource(studio);
        assert!(
            !path.exists(),
            "nothing is written before the schedule runs"
        );

        app.world_mut().run_schedule(Update);

        assert!(
            path.is_file(),
            "the plugin's own schedule saved the session: {:?}",
            app.world().resource::<Autosave>().note()
        );
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// ⚠ The project is the work; the scan file is only where the body came
    /// from. A session whose scan has been deleted since must still come back —
    /// refusing would lose every step after it over a file the user can re-pick.
    #[test]
    fn a_session_whose_scan_has_gone_still_comes_back_and_says_so() {
        let dir = scans("scan-deleted");
        let saved = walked_to(&dir, Step::CleanScan);
        saved
            .save(&autosave_path(&dir.join("base.stl")))
            .expect("the previous session");
        let (mut autosave, mut studio) = just_picked(&dir);
        std::fs::remove_file(dir.join("base.cleaned.stl")).expect("the body goes missing");

        let mut scan = ScanEdit::default();
        resume(&mut autosave, &mut studio, &mut scan);

        assert_eq!(studio.project, saved, "the work comes back regardless");
        assert!(scan.active().is_none(), "with no body to show");
        assert!(
            matches!(studio.outcome(), Some(Err(why)) if why.contains("Picked up")),
            "and the screen says both halves: {:?}",
            studio.outcome()
        );
        let _ = std::fs::remove_dir_all(&dir);
    }
}
