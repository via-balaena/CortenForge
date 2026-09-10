//! The print-export boundary (workflow step "3D print the molds"): gather
//! the printable artifacts a mold run produced into one clean folder the
//! user hands to their slicer.
//!
//! The cast already wrote the STLs (under the run's `out/stls`) plus a
//! `procedure.md`. This copies them — flat — into a destination the user
//! picks, so they get a self-contained print package separate from the
//! cast's working directory.

use std::path::Path;

use cf_studio_core::{MoldOutputs, PrintExport};

use crate::error::{EngineError, Result};

/// What a print export copied, for the caller's confirmation message.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PrintExportReport {
    /// The recorded export ([`Step::Print`](cf_studio_core::Step::Print)'s
    /// artifact) — where the files landed.
    pub export: PrintExport,
    /// Number of `.stl` files copied (mold halves + plugs + accessories).
    pub stl_count: usize,
    /// Number of `.stl` files `dest_dir` holds **after** the copy, or `None`
    /// if the folder could not be read back.
    ///
    /// The destination is never cleared, so it ends up holding the union of
    /// what was already there and what this export wrote. That makes
    /// `Some(n)` with `n > stl_count` the exact test for "this folder holds a
    /// printable this export did not write" — equality means every file
    /// already present was overwritten.
    ///
    /// ⚠ `None`, not `0`, when the read fails: zero is a real answer (an
    /// export that copied nothing into an empty folder) and must not double
    /// as "couldn't look".
    pub dest_stl_count: Option<usize>,
    /// Whether the `procedure.md` was copied too (absent on a run that
    /// somehow didn't emit one).
    pub procedure_copied: bool,
}

/// Copy a mold run's printable artifacts into `dest_dir`: every mold-half,
/// plug, and accessory STL, plus the `procedure.md`, flattened into the one
/// folder. Creates `dest_dir` (and parents) if needed. Returns a report +
/// the [`PrintExport`] to record on the project.
///
/// Files are copied (not moved) so the cast's output directory stays
/// intact; re-exporting overwrites same-named files in `dest_dir` and
/// **leaves every other file where it is**. A selective cast reports only
/// the part it regenerated, so re-exporting after one copies that part into
/// a folder still holding the rest of the previous package.
/// [`PrintExportReport::dest_stl_count`] reports what the folder holds once
/// the copy is done; deciding what to do about it is the caller's.
///
/// # Errors
/// [`EngineError::ExportPrint`] if `dest_dir` can't be created, a source
/// path has no filename, or any copy fails.
pub fn export_print_package(molds: &MoldOutputs, dest_dir: &Path) -> Result<PrintExportReport> {
    std::fs::create_dir_all(dest_dir)
        .map_err(|e| EngineError::ExportPrint(format!("create {}: {e}", dest_dir.display())))?;
    let mut stl_count = 0;
    for src in molds
        .mold_stls
        .iter()
        .chain(&molds.plug_stls)
        .chain(&molds.accessory_stls)
    {
        copy_into(src, dest_dir)?;
        stl_count += 1;
    }

    // The procedure is a nicety, not load-bearing — copy it if the run
    // emitted one, but don't fail the export if it's missing.
    let procedure_copied = if molds.procedure_path.is_file() {
        copy_into(&molds.procedure_path, dest_dir)?;
        true
    } else {
        false
    };

    Ok(PrintExportReport {
        export: PrintExport {
            export_dir: dest_dir.to_path_buf(),
        },
        stl_count,
        dest_stl_count: count_stls(dest_dir),
        procedure_copied,
    })
}

/// How many `.stl` files `dir` holds, or `None` if it could not be read.
///
/// ⚠ This reads a directory that outlives the run, which the cast's own
/// export was once deleted for doing — but the claim is the opposite one.
/// That glob answered *"what did this run produce"*, which a directory cannot
/// know, and handed back orphans from earlier casts as the run's own output.
/// This answers *"what does this folder hold now"*, which is the only
/// question a directory can answer and the only way to tell someone what
/// their slicer is about to see.
///
/// A directory named `foo.stl` is not a printable file.
fn count_stls(dir: &Path) -> Option<usize> {
    Some(
        std::fs::read_dir(dir)
            .ok()?
            .flatten()
            .filter(|e| {
                let path = e.path();
                path.is_file()
                    && path
                        .extension()
                        .is_some_and(|x| x.eq_ignore_ascii_case("stl"))
            })
            .count(),
    )
}

/// Copy `src` into `dest_dir` under its filename. Skips the copy when the
/// destination already resolves to the same file as `src` — the user picked
/// the folder the file lives in (e.g. the cast's own `stls/`), or a symlink
/// at the destination points back at `src` — because `std::fs::copy` of a
/// file onto itself truncates it to zero bytes. Both paths are canonicalized
/// (resolving symlinks, `..`, and macOS `/Users` aliasing); `canonicalize` of
/// the destination only succeeds if it already exists, so a not-yet-present
/// destination simply copies.
fn copy_into(src: &Path, dest_dir: &Path) -> Result<()> {
    let name = src.file_name().ok_or_else(|| {
        EngineError::ExportPrint(format!("source has no filename: {}", src.display()))
    })?;
    let dest = dest_dir.join(name);
    if let (Ok(src_canon), Ok(dest_canon)) =
        (std::fs::canonicalize(src), std::fs::canonicalize(&dest))
        && src_canon == dest_canon
    {
        return Ok(()); // already in place — copying onto itself would zero it
    }
    std::fs::copy(src, &dest).map_err(|e| {
        EngineError::ExportPrint(format!("copy {} → {}: {e}", src.display(), dest.display()))
    })?;
    Ok(())
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use std::path::PathBuf;

    use cf_studio_core::PourPlan;

    use super::*;

    fn temp_dir(label: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!(
            "cf-studio-engine-print-test-{}-{label}",
            std::process::id()
        ));
        std::fs::create_dir_all(&dir).unwrap();
        dir
    }

    /// A directory guaranteed empty, for the tests that assert a COUNT: a
    /// run that panicked before its cleanup would otherwise leave files
    /// behind and move the number.
    fn fresh_dir(label: &str) -> PathBuf {
        let dir = temp_dir(label);
        std::fs::remove_dir_all(&dir).unwrap();
        std::fs::create_dir_all(&dir).unwrap();
        dir
    }

    /// Build a MoldOutputs whose STL/procedure paths point at real files in
    /// `src`, so export actually copies something.
    fn molds_with_files(src: &Path) -> MoldOutputs {
        let write = |name: &str| {
            let p = src.join(name);
            std::fs::write(&p, name.as_bytes()).unwrap();
            p
        };
        MoldOutputs {
            out_dir: src.to_path_buf(),
            mold_stls: vec![
                write("mold_layer_0_piece_0.stl"),
                write("mold_layer_0_piece_1.stl"),
            ],
            plug_stls: vec![write("plug_layer_0.stl")],
            accessory_stls: vec![write("platform.stl")],
            procedure_path: write("procedure.md"),
            total_mass_g: 100.0,
            pour_plan: PourPlan { steps: vec![] },
        }
    }

    #[test]
    fn export_copies_all_printables_plus_procedure() {
        let src = temp_dir("src");
        let dest = temp_dir("dest-all");
        let molds = molds_with_files(&src);

        let report = export_print_package(&molds, &dest).unwrap();
        assert_eq!(report.stl_count, 4, "2 molds + 1 plug + 1 accessory");
        assert!(report.procedure_copied);
        assert_eq!(report.export.export_dir, dest);
        for name in [
            "mold_layer_0_piece_0.stl",
            "mold_layer_0_piece_1.stl",
            "plug_layer_0.stl",
            "platform.stl",
            "procedure.md",
        ] {
            assert!(dest.join(name).is_file(), "{name} copied");
        }
        // Source untouched (copy, not move).
        assert!(src.join("mold_layer_0_piece_0.stl").is_file());

        let _ = std::fs::remove_dir_all(&src);
        let _ = std::fs::remove_dir_all(&dest);
    }

    #[test]
    fn export_creates_missing_destination() {
        let src = temp_dir("src2");
        let dest = temp_dir("dest2").join("nested/print-package");
        let molds = molds_with_files(&src);

        let report = export_print_package(&molds, &dest).unwrap();
        assert!(dest.is_dir(), "nested dest created");
        assert_eq!(report.stl_count, 4);

        let _ = std::fs::remove_dir_all(&src);
        let _ = std::fs::remove_dir_all(temp_dir("dest2"));
    }

    #[test]
    fn export_without_procedure_still_succeeds() {
        let src = temp_dir("src3");
        let dest = temp_dir("dest3");
        let mut molds = molds_with_files(&src);
        molds.procedure_path = PathBuf::from("/no/such/procedure.md");

        let report = export_print_package(&molds, &dest).unwrap();
        assert_eq!(report.stl_count, 4);
        assert!(!report.procedure_copied, "no procedure to copy");

        let _ = std::fs::remove_dir_all(&src);
        let _ = std::fs::remove_dir_all(&dest);
    }

    #[test]
    fn export_into_source_dir_is_a_safe_noop() {
        // The footgun: pick the folder the files already live in. Copying a
        // file onto itself truncates it to zero bytes — this must skip it.
        let src = temp_dir("selfcopy");
        let molds = molds_with_files(&src);

        let report = export_print_package(&molds, &src).unwrap();
        assert_eq!(report.stl_count, 4, "all still counted (already in place)");
        assert!(report.procedure_copied);
        // Every file must keep its content (was written non-empty), not be
        // zeroed by a self-copy.
        for name in [
            "mold_layer_0_piece_0.stl",
            "plug_layer_0.stl",
            "platform.stl",
            "procedure.md",
        ] {
            let bytes = std::fs::read(src.join(name)).unwrap();
            assert!(!bytes.is_empty(), "{name} must not be truncated to 0 bytes");
        }

        let _ = std::fs::remove_dir_all(&src);
    }

    /// The destination is never cleared, so the report has to say what the
    /// folder HOLDS — not only what this export put in it.
    ///
    /// ⚠ The two numbers answer different questions: `stl_count` is "what did
    /// I copy", `dest_stl_count` is "what will the slicer see". A selective
    /// cast makes them differ, and only the second one gets printed.
    #[test]
    fn the_report_counts_what_the_destination_holds_not_only_what_it_copied() {
        let src = fresh_dir("src-dest-count");
        let dest = fresh_dir("dest-dest-count");
        let molds = molds_with_files(&src);

        // Already there, from an export nothing in this run knows about.
        std::fs::write(dest.join("plug_layer_7.stl"), b"solid old\n").unwrap();
        // A DIRECTORY whose name ends in `.stl` is not a printable file —
        // the trap that got past two review rounds on the manifest.
        std::fs::create_dir_all(dest.join("not_a_part.stl")).unwrap();

        let report = export_print_package(&molds, &dest).unwrap();

        assert_eq!(report.stl_count, 4, "this export copied four parts");
        assert_eq!(
            report.dest_stl_count,
            Some(5),
            "the four copied plus the leftover — the directory is not a part, \
             and procedure.md is not a printable"
        );

        let _ = std::fs::remove_dir_all(&src);
        let _ = std::fs::remove_dir_all(&dest);
    }

    /// The negative control for the gate above.
    ///
    /// Equality is what tells the caller there is nothing else in the folder,
    /// so it has to actually hold on a clean export — a count wrong in the
    /// other direction would warn on every save, and a warning that always
    /// fires stops being read.
    #[test]
    fn a_clean_export_leaves_the_two_counts_equal() {
        let src = fresh_dir("src-clean-count");
        let dest = fresh_dir("dest-clean-count");
        let molds = molds_with_files(&src);

        let report = export_print_package(&molds, &dest).unwrap();

        assert_eq!(
            report.dest_stl_count,
            Some(report.stl_count),
            "nothing was in this folder that the export did not write"
        );

        let _ = std::fs::remove_dir_all(&src);
        let _ = std::fs::remove_dir_all(&dest);
    }

    #[test]
    fn export_errors_when_a_source_stl_is_missing() {
        let src = temp_dir("src4");
        let dest = temp_dir("dest4");
        let mut molds = molds_with_files(&src);
        molds.mold_stls.push(PathBuf::from("/no/such/missing.stl"));

        let err = export_print_package(&molds, &dest).unwrap_err();
        assert!(matches!(err, EngineError::ExportPrint(_)), "got: {err:?}");

        let _ = std::fs::remove_dir_all(&src);
        let _ = std::fs::remove_dir_all(&dest);
    }
}
