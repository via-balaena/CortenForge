//! The scan-clean acceptance boundary (interim P2c).
//!
//! Cap-plane + centerline *detection* lives only inside the
//! `cf-scan-prep` GUI today, so for now the wizard **accepts** an
//! existing cleaned scan + `.prep.toml` that the user produced there,
//! validating it against exactly what the cast pipeline requires:
//!
//! - the cleaned STL loads and has geometry, and
//! - the `.prep.toml` yields a **non-empty centerline** (a load-bearing
//!   precondition for `cf_cast::Ribbon`), and
//! - if a `[caps]` block is present, it parses.
//!
//! We deliberately reuse the cast pipeline's own parsers
//! ([`parse_centerline_from_prep_toml`] + [`parse_cap_planes`]) rather
//! than a divergent re-check, so "accepted here" means "the cast will
//! accept it too". A fully headless cleaning path (extracting the
//! detection out of the GUI into a `cf-scan-boundary` crate) is a
//! separate, later arc.

use std::fs;
use std::path::Path;

use cf_cap_planes::parse_cap_planes;
use cf_cast_cli::parse_centerline_from_prep_toml;
use cf_studio_core::PrepInput;

use crate::error::{EngineError, Result};
use crate::scan::load_scan;

/// Validate an existing cleaned scan + `.prep.toml` pair and produce the
/// spine's [`PrepInput`]. See the module docs for what's checked.
///
/// # Errors
/// - [`EngineError::ScanLoad`] / [`EngineError::EmptyScan`] if the cleaned
///   STL is missing, unreadable, or empty (reusing the scan-load checks).
/// - [`EngineError::PrepInvalid`] if the `.prep.toml` can't be read/parsed.
/// - [`EngineError::NoCenterline`] if it parses but has no centerline.
pub fn accept_prep(cleaned_stl: &Path, prep_toml: &Path) -> Result<PrepInput> {
    // 1. The cleaned scan must load and have geometry. Reuse the
    //    scan-load validation (its errors name the offending path).
    load_scan(cleaned_stl)?;

    // 2. The prep file must be readable.
    let text = fs::read_to_string(prep_toml).map_err(|e| EngineError::PrepInvalid {
        path: prep_toml.display().to_string(),
        reason: e.to_string(),
    })?;

    // 3. It must yield a non-empty centerline — the cast pipeline bails
    //    without one, so we catch it here at step 2 of the wizard.
    let centerline =
        parse_centerline_from_prep_toml(&text).map_err(|e| EngineError::PrepInvalid {
            path: prep_toml.display().to_string(),
            reason: format!("{e:#}"),
        })?;
    if centerline.is_empty() {
        return Err(EngineError::NoCenterline {
            path: prep_toml.display().to_string(),
        });
    }

    // 4. Cap planes are optional, but a present `[caps]` block must parse.
    parse_cap_planes(&text).map_err(|e| EngineError::PrepInvalid {
        path: prep_toml.display().to_string(),
        reason: format!("{e:#}"),
    })?;

    Ok(PrepInput {
        cleaned_stl: cleaned_stl.to_path_buf(),
        prep_toml: prep_toml.to_path_buf(),
    })
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use std::path::PathBuf;

    use super::*;

    /// A minimal valid ASCII STL — one triangle (stands in for a cleaned scan).
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

    /// A `.prep.toml` with a two-point centerline.
    const PREP_WITH_CENTERLINE: &str = "\
[centerline]
points_m = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.01]]
";

    /// A `.prep.toml` with an empty centerline block.
    const PREP_EMPTY_CENTERLINE: &str = "\
[centerline]
points_m = []
";

    fn temp_dir(label: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!(
            "cf-studio-engine-prep-test-{}-{label}",
            std::process::id()
        ));
        std::fs::create_dir_all(&dir).unwrap();
        dir
    }

    /// Write the cleaned STL + prep into `dir` and return their paths.
    fn write_pair(dir: &Path, prep_body: &str) -> (PathBuf, PathBuf) {
        let stl = dir.join("base_mold.cleaned.stl");
        let prep = dir.join("base_mold.prep.toml");
        std::fs::write(&stl, ONE_TRIANGLE_STL).unwrap();
        std::fs::write(&prep, prep_body).unwrap();
        (stl, prep)
    }

    #[test]
    fn accepts_a_valid_cleaned_scan_and_prep() {
        let dir = temp_dir("valid");
        let (stl, prep) = write_pair(&dir, PREP_WITH_CENTERLINE);

        let input = accept_prep(&stl, &prep).unwrap();
        assert_eq!(input.cleaned_stl, stl);
        assert_eq!(input.prep_toml, prep);

        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn empty_centerline_is_rejected() {
        let dir = temp_dir("empty-cl");
        let (stl, prep) = write_pair(&dir, PREP_EMPTY_CENTERLINE);

        let err = accept_prep(&stl, &prep).unwrap_err();
        assert!(
            matches!(err, EngineError::NoCenterline { .. }),
            "got: {err:?}"
        );

        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn missing_centerline_block_is_rejected() {
        let dir = temp_dir("no-cl");
        // A prep with no [centerline] block at all → empty → NoCenterline.
        let (stl, prep) = write_pair(&dir, "[scan_prep]\ntool_version = \"x\"\n");

        let err = accept_prep(&stl, &prep).unwrap_err();
        assert!(
            matches!(err, EngineError::NoCenterline { .. }),
            "got: {err:?}"
        );

        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn missing_prep_file_is_prep_invalid() {
        let dir = temp_dir("no-prep");
        let stl = dir.join("base_mold.cleaned.stl");
        std::fs::write(&stl, ONE_TRIANGLE_STL).unwrap();

        let err = accept_prep(&stl, &dir.join("absent.prep.toml")).unwrap_err();
        assert!(
            matches!(err, EngineError::PrepInvalid { .. }),
            "got: {err:?}"
        );

        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn missing_cleaned_scan_is_a_scan_error() {
        let dir = temp_dir("no-stl");
        let prep = dir.join("base_mold.prep.toml");
        std::fs::write(&prep, PREP_WITH_CENTERLINE).unwrap();

        let err = accept_prep(&dir.join("absent.cleaned.stl"), &prep).unwrap_err();
        assert!(matches!(err, EngineError::ScanLoad { .. }), "got: {err:?}");

        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn malformed_prep_toml_is_prep_invalid() {
        let dir = temp_dir("bad-toml");
        let (stl, prep) = write_pair(&dir, "not valid toml = = =");

        let err = accept_prep(&stl, &prep).unwrap_err();
        assert!(
            matches!(err, EngineError::PrepInvalid { .. }),
            "got: {err:?}"
        );

        let _ = std::fs::remove_dir_all(&dir);
    }
}
