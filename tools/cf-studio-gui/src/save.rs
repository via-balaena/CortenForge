//! Step 2's Save: write the cleaned scan + its `.prep.toml`, then accept the
//! pair as the step's artifact.
//!
//! ⚠ Synchronous, unlike Simplify — measured, not inherited. On the
//! 200 000-face `base_mold` scan a save costs 87 ms at smoothing 0, 109 ms at
//! the default 8, and 147–184 ms through this path once the scan is trimmed
//! and its floor rebuilt: the same order as `weld` (68 ms) and `detect_caps`
//! (56 ms), which step 2 already runs on the main thread. So there is no task
//! to spawn, no poller, and no second `busy` path. The pre-port comment
//! calling this "the save freeze" was wrong.

use std::path::{Path, PathBuf};

use cf_studio_gui::{apply_prep, format_save_done};

use crate::scan::{ActiveScan, ScanEdit};
use crate::state::{PendingSave, Studio};

/// The unit the cleaned STL records itself in, as the pre-port save did.
const STL_UNITS: &str = "mm";

/// Used only if the scan's filename has no stem, which no picked file has.
const FALLBACK_STEM: &str = "scan";

/// The folder a Save defaults to, and the stem its two outputs are named
/// after — the scan's own folder and filename, so the cast's inputs land
/// beside the scan they came from.
///
/// `None` when no scan is recorded. The Save control is not offered then, so
/// this is the second guard, not the first.
fn scan_target(studio: &Studio) -> Option<(PathBuf, String)> {
    let source = &studio.project.scan()?.source_path;
    let dir = source
        .parent()
        .map_or_else(|| PathBuf::from("."), Path::to_path_buf);
    let stem = source
        .file_stem()
        .and_then(|s| s.to_str())
        .unwrap_or(FALLBACK_STEM)
        .to_owned();
    Some((dir, stem))
}

/// Whether saving `stem` into `dir` would overwrite something already there.
///
/// Either file counts: they are written and accepted as a pair, so a folder
/// holding one of them is a folder this save would change.
fn outputs_exist(dir: &Path, stem: &str) -> bool {
    dir.join(format!("{stem}.cleaned.stl")).exists()
        || dir.join(format!("{stem}.prep.toml")).exists()
}

/// Save into the scan's own folder — what the Save button asks for.
pub(crate) fn save_to_default(scan: &ScanEdit, studio: &mut Studio, smoothing: usize) {
    let Some((dir, _)) = scan_target(studio) else {
        settle(studio, Err("Add a scan in step 1 first.".to_owned()));
        return;
    };
    save_into(scan, studio, dir, smoothing);
}

/// The overwrite modal's question: which files, and which folder holds them.
pub(crate) fn overwrite_question(studio: &Studio, dir: &Path) -> String {
    let stem = scan_target(studio).map_or_else(|| FALLBACK_STEM.to_owned(), |(_, stem)| stem);
    format!(
        "{stem}.cleaned.stl / .prep.toml already exist in {}.\n\n\
         Overwrite them, or choose a different folder?",
        dir.display(),
    )
}

/// Save into `dir` — writing straight away, or raising the overwrite question.
///
/// ⚠ The single entry point for both the Save button and the folder the picker
/// returns, so a folder the user chose is checked exactly as the default one
/// is. The pre-port code checked only the default folder and then wrote into
/// the chosen one unconditionally, silently overwriting whatever was there.
pub(crate) fn save_into(scan: &ScanEdit, studio: &mut Studio, dir: PathBuf, smoothing: usize) {
    let Some((_, stem)) = scan_target(studio) else {
        settle(studio, Err("Add a scan in step 1 first.".to_owned()));
        return;
    };
    if outputs_exist(&dir, &stem) {
        studio.pending_save = Some(PendingSave::Confirming { dir, smoothing });
        return;
    }
    write_into(scan, studio, &dir, smoothing);
}

/// Write the pair into `dir` and accept it, whatever is already there.
///
/// Reached once the overwrite question is answered, or when there was none.
pub(crate) fn write_into(scan: &ScanEdit, studio: &mut Studio, dir: &Path, smoothing: usize) {
    let (Some((_, stem)), Some(session)) =
        (scan_target(studio), scan.active().map(ActiveScan::session))
    else {
        settle(studio, Err("Add a scan in step 1 first.".to_owned()));
        return;
    };
    let outcome = match session.save(dir, &stem, STL_UNITS, smoothing) {
        Err(e) => Err(format!("Save failed: {e}")),
        // Accepting the written pair is what completes step 2 and unblocks
        // Next — the files alone are not the artifact.
        Ok(report) => match apply_prep(&mut studio.project, &report.cleaned_stl, &report.prep_toml)
        {
            Ok(_) => Ok(format_save_done(&stem, report.face_count)),
            Err(e) => Err(format!("Saved the files, but they didn't validate: {e}")),
        },
    };
    settle(studio, outcome);
}

/// Report `outcome` and put the app back in a state that accepts actions.
///
/// ⚠ Every path out of a Save goes through here. `pending_save` gates every
/// control in the wizard, so a path that reported without clearing it would
/// leave the app inert with nothing on screen to explain why.
pub(crate) fn settle(studio: &mut Studio, outcome: cf_studio_gui::StepOutcome) {
    studio.pending_save = None;
    studio.message = Some(outcome);
}

#[cfg(test)]
pub(crate) mod tests {
    #![allow(clippy::expect_used)]

    use cf_studio_core::{Project, ScanInput};

    use super::*;
    use crate::edit::tests::open_tube;
    use crate::edit::{EditControls, EditIntent, apply_edit_intent};
    use crate::scan::ActiveScan;

    /// A folder of this test's own. Name- and PID-scoped like the engine's save
    /// gates, so concurrent runs cannot read each other's outputs.
    pub(crate) fn temp_dir(name: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!("cf-save-{name}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).expect("a temp dir");
        dir
    }

    /// Step 2 with a Save available: a scan recorded at `dir/base.stl`, and a
    /// session holding the centerline `EditSession::save` refuses to write
    /// without.
    ///
    /// ⚠ The scan file itself is never written. Nothing on the Save path reads
    /// it — the mesh is already in the session — and `source_path` is only ever
    /// asked for its folder and its stem.
    pub(crate) fn ready_to_save(dir: &Path) -> (ScanEdit, Studio) {
        let mut scan = ScanEdit::default();
        scan.set(ActiveScan::synthetic(open_tube()));
        let mut studio = Studio::default();
        let mut controls = EditControls::default();
        apply_edit_intent(EditIntent::FindFloor, &mut scan, &mut studio, &mut controls);
        assert!(
            scan.active()
                .map(ActiveScan::session)
                .is_some_and(|s| s.has_centerline()),
            "the fixture must stand up, or every save below fails for that reason \
             instead of the one it is testing; last message: {:?}",
            studio.message
        );
        studio.project = Project::new("save gate");
        studio.project.set_scan(ScanInput {
            source_path: dir.join("base.stl"),
        });
        (scan, studio)
    }

    /// ★ What step 2 is for: both files land beside the scan and the project
    /// accepts them, which is what unblocks Next.
    #[test]
    fn a_save_writes_both_files_and_completes_the_step() {
        let dir = temp_dir("roundtrip");
        let (scan, mut studio) = ready_to_save(&dir);

        save_to_default(&scan, &mut studio, 0);

        assert!(dir.join("base.cleaned.stl").is_file(), "the cleaned STL");
        assert!(dir.join("base.prep.toml").is_file(), "and its prep");
        assert!(
            studio.project.prep().is_some(),
            "accepted as step 2's artifact — the files alone are not the step: {:?}",
            studio.message
        );
        assert!(
            studio.message.as_ref().is_some_and(Result::is_ok),
            "and reported as a success: {:?}",
            studio.message
        );
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// ⚠ The modal's whole job is saying *which* files and *where*. An empty
    /// or fixed question is a modal asking the user to confirm nothing.
    #[test]
    fn the_overwrite_question_names_the_files_and_the_folder() {
        let dir = temp_dir("question");
        let (_scan, studio) = ready_to_save(&dir);

        let question = overwrite_question(&studio, &dir);

        assert!(question.contains("base.cleaned.stl"), "{question}");
        assert!(question.contains(&dir.display().to_string()), "{question}");
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// ⚠ Outputs already in the folder are a question, not a silent overwrite.
    /// Only the STL is planted, so this covers the first half of the check.
    #[test]
    fn outputs_already_there_are_asked_about_rather_than_overwritten() {
        let dir = temp_dir("collision");
        let (scan, mut studio) = ready_to_save(&dir);
        std::fs::write(dir.join("base.cleaned.stl"), b"not mine").expect("a decoy");

        save_to_default(&scan, &mut studio, 0);

        assert_eq!(
            studio.pending_save,
            Some(PendingSave::Confirming {
                dir: dir.clone(),
                smoothing: 0
            }),
            "the save is held on the question"
        );
        assert_eq!(
            std::fs::read(dir.join("base.cleaned.stl")).expect("still there"),
            b"not mine",
            "and nothing is written until it is answered"
        );
        assert!(
            studio.project.prep().is_none(),
            "so the step is not complete"
        );
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// Answering the question with Overwrite is what actually replaces them.
    #[test]
    fn overwriting_replaces_what_was_in_the_way() {
        let dir = temp_dir("overwrite");
        let (scan, mut studio) = ready_to_save(&dir);
        std::fs::write(dir.join("base.cleaned.stl"), b"not mine").expect("a decoy");
        save_to_default(&scan, &mut studio, 0);

        write_into(&scan, &mut studio, &dir, 0);

        assert_ne!(
            std::fs::read(dir.join("base.cleaned.stl")).expect("written"),
            b"not mine",
            "the decoy is gone"
        );
        assert!(studio.project.prep().is_some(), "and the step completes");
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// ★ The pre-port code checked the default folder and then wrote into a
    /// chosen one unconditionally. A folder picked *because* the first one
    /// collided is exactly where a second collision is likely.
    ///
    /// ⚠ Only the prep is planted here, so this covers the half of the check
    /// the collision gate above does not.
    #[test]
    fn a_folder_the_user_picked_is_checked_for_a_collision_too() {
        let dir = temp_dir("picked-scan-folder");
        let picked = temp_dir("picked-second-folder");
        let (scan, mut studio) = ready_to_save(&dir);
        std::fs::write(picked.join("base.prep.toml"), b"in the way").expect("a decoy");

        save_into(&scan, &mut studio, picked.clone(), 0);

        assert_eq!(
            studio.pending_save,
            Some(PendingSave::Confirming {
                dir: picked.clone(),
                smoothing: 0
            }),
            "asked about the picked folder, not the scan's own"
        );
        assert_eq!(
            std::fs::read(picked.join("base.prep.toml")).expect("still there"),
            b"in the way",
            "and it is still untouched"
        );
        let _ = std::fs::remove_dir_all(&dir);
        let _ = std::fs::remove_dir_all(&picked);
    }

    /// ⚠ `pending_save` gates every control in the wizard, so a Save that ends
    /// without clearing it leaves the app inert with no question on screen to
    /// explain why. Every way one can end, including the failures.
    #[test]
    fn every_way_a_save_ends_lets_go_of_the_app() {
        let dir = temp_dir("release");
        let (scan, mut studio) = ready_to_save(&dir);

        save_to_default(&scan, &mut studio, 0);
        assert!(studio.pending_save.is_none(), "a save that succeeded");

        // No scan at all — `write_into`'s own guard, before the engine.
        studio.pending_save = Some(PendingSave::ChoosingFolder { smoothing: 0 });
        write_into(&ScanEdit::default(), &mut studio, &dir, 0);
        assert!(
            studio.pending_save.is_none(),
            "a save with nothing to write"
        );
        assert!(
            matches!(&studio.message, Some(Err(text)) if text.contains("step 1")),
            "and it says which step is missing: {:?}",
            studio.message
        );

        // A scan that was never stood up — this one does reach the engine, and
        // it refuses, because the cast has no centerline to follow.
        let mut unlevelled = ScanEdit::default();
        unlevelled.set(ActiveScan::synthetic(open_tube()));
        studio.pending_save = Some(PendingSave::ChoosingFolder { smoothing: 0 });
        write_into(&unlevelled, &mut studio, &dir, 0);
        assert!(studio.pending_save.is_none(), "a save the engine refused");
        assert!(
            matches!(&studio.message, Some(Err(text)) if text.starts_with("Save failed")),
            "and it passes the engine's reason on: {:?}",
            studio.message
        );

        studio.pending_save = Some(PendingSave::Confirming {
            dir: dir.clone(),
            smoothing: 0,
        });
        settle(&mut studio, Ok("Save cancelled.".to_owned()));
        assert!(studio.pending_save.is_none(), "a save that was cancelled");

        // No scan on the project at all, through both entry points. The control
        // is not offered in this state, so these are the second guard — but a
        // second guard that returned without settling would wedge the app just
        // as hard as no guard at all.
        studio.project = Project::new("no scan recorded");
        for start in [
            &save_to_default as &dyn Fn(&ScanEdit, &mut Studio, usize),
            &|scan, studio, smoothing| save_into(scan, studio, dir.clone(), smoothing),
        ] {
            studio.pending_save = Some(PendingSave::ChoosingFolder { smoothing: 0 });
            start(&scan, &mut studio, 0);
            assert!(
                studio.pending_save.is_none(),
                "a save with no scan to write"
            );
            assert!(
                matches!(&studio.message, Some(Err(text)) if text.contains("step 1")),
                "and it says which step is missing: {:?}",
                studio.message
            );
        }
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// ⚠ Nothing else here would notice `smoothing` being dropped on the way to
    /// the engine: every other gate passes with it hard-coded.
    #[test]
    fn the_smoothing_field_reaches_the_mesh_that_is_written() {
        let smooth = temp_dir("smoothed");
        let sharp = temp_dir("unsmoothed");
        let (scan, mut studio) = ready_to_save(&sharp);
        save_to_default(&scan, &mut studio, 0);
        let (scan, mut studio) = ready_to_save(&smooth);

        save_to_default(&scan, &mut studio, 30);

        assert_ne!(
            std::fs::read(sharp.join("base.cleaned.stl")).expect("written"),
            std::fs::read(smooth.join("base.cleaned.stl")).expect("written"),
            "30 smoothing passes move the mesh; 0 and 30 cannot write the same file"
        );
        let _ = std::fs::remove_dir_all(&smooth);
        let _ = std::fs::remove_dir_all(&sharp);
    }
}
