//! Step 2's Save: write the cleaned scan + its `.prep.toml`, then accept the
//! pair as the step's artifact.
//!
//! ⚠ Synchronous, unlike Simplify — measured, not inherited. On the
//! 200 000-face `base_mold` scan a save costs 87 ms at smoothing 0, 109 ms at
//! the default 8, and 141 ms after a floor trim: the same order as `weld`
//! (68 ms) and `detect_caps` (56 ms), which step 2 already runs on the main
//! thread. So there is no task to spawn, no poller, and no second `busy` path.
//! The pre-port comment calling this "the save freeze" was wrong.

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
