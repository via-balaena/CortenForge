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
/// intact; re-exporting overwrites same-named files in `dest_dir`.
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
        procedure_copied,
    })
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
    {
        if src_canon == dest_canon {
            return Ok(()); // already in place — copying onto itself would zero it
        }
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
