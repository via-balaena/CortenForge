//! Coverage instrumentation scoped to the crate under measurement.
//!
//! `cargo llvm-cov` works by setting `RUSTFLAGS=-C instrument-coverage`, and
//! cargo applies `RUSTFLAGS` to *every* unit it builds: the crate under test,
//! its workspace siblings, and every registry dependency. The report was then
//! filtered back down to the crate's own files by
//! [`crate::coverage::production_coverage`] — so every instrumented dependency
//! line was computed at full price and then thrown away.
//!
//! For a crate whose tests reach a FEM solver, that is not a rounding error.
//! Measured on `cf-fsu-model`, 2026-08-13, same 16 `--lib` tests:
//!
//! | run | wall |
//! |---|---|
//! | instrumented, whole dependency tree | **3069 s** |
//! | uninstrumented | **7.6 s** |
//!
//! ~400×, and it is not one heavy test: dropping the single most expensive one
//! still left 1157 s for the remaining fifteen, which together run in 2.6 s
//! uninstrumented. The tax is uniform over everything that reaches the solver.
//! `nalgebra`'s small-matrix kernels are built from tiny generic functions that
//! optimise away to nothing; a counter in every basic block is not something an
//! FEM inner loop can absorb.
//!
//! ## Why scoping cannot change the number
//!
//! The criterion has only ever counted the measured crate's own files. Lines
//! belonging to a dependency were instrumented, exported, and then discarded by
//! the path filter. Removing instrumentation from those crates removes data
//! that was already being dropped — the numerator and denominator are built
//! from the same files either way. [`partition_export`] asserts exactly that on
//! every run rather than trusting it.
//!
//! ## Why a `RUSTC_WRAPPER`
//!
//! Cargo has no stable per-package `rustflags`: `profile-rustflags` is nightly
//! (checked on 1.96, which rejects it outright). `RUSTC_WRAPPER` is the one
//! stable interception point — cargo runs it in place of `rustc` for every
//! unit and passes the real `rustc` as the first argument, so the wrapper can
//! add `-C instrument-coverage` to exactly one compilation and leave the rest
//! of the dependency tree fully optimised.
//!
//! The wrapper is this same `xtask` binary re-entered through
//! [`WRAPPER_CRATE_ENV`]; a separate shell script would have to be written to
//! disk and quoted per platform.
//!
//! ## Cost of the approach
//!
//! Setting `RUSTC_WRAPPER` changes cargo's fingerprint for *every* unit, so a
//! coverage build cannot share `target/`'s cache without invalidating it for
//! ordinary builds and back again on the next one. Coverage therefore builds
//! into its own directory, which is what `cargo llvm-cov` does for the same
//! reason. The first build in a fresh tree is a full one; later runs are
//! incremental.

use std::ffi::OsString;
use std::path::{Path, PathBuf};
use std::process::Command;

use anyhow::{bail, Context, Result};
use xshell::{cmd, Shell};

/// Names the crate to instrument, and switches `xtask` into wrapper mode.
///
/// Set only on the cargo child process that builds the instrumented test
/// binary, so an ordinary `xtask` invocation never sees it.
pub(crate) const WRAPPER_CRATE_ENV: &str = "CF_COVERAGE_WRAPPER_CRATE";

/// Where instrumented builds live, relative to the workspace root.
///
/// Separate from `target/` because `RUSTC_WRAPPER` is part of every unit's
/// fingerprint — sharing one directory would make each coverage run invalidate
/// the ordinary build cache, and each ordinary build invalidate the coverage
/// one.
const COVERAGE_TARGET_DIR: &str = "target/cf-coverage";

/// What one instrumented `--lib` run produced.
pub(crate) struct LibCoverageRun {
    /// The llvm-cov JSON export, in the same shape `cargo llvm-cov --json`
    /// produced — [`crate::coverage::production_coverage`] reads it unchanged.
    pub json: serde_json::Value,
    /// Whether the instrumented `--lib` tests themselves passed.
    pub tests_passed: bool,
}

/// True when this `rustc` invocation compiles `target_crate`.
///
/// Cargo spells the flag `--crate-name foo`; `--crate-name=foo` is accepted by
/// `rustc` and handled too, so a future cargo cannot silently stop matching and
/// leave the whole tree uninstrumented — which would read as a coverage
/// collapse rather than as a wrapper bug.
///
/// The crate name here is the *compiler's* spelling: `cf-fsu-model` is compiled
/// as `cf_fsu_model`.
fn should_instrument(args: &[String], target_crate: &str) -> bool {
    let mut it = args.iter();
    while let Some(a) = it.next() {
        if a == "--crate-name" {
            return it.next().is_some_and(|n| n == target_crate);
        }
        if let Some(name) = a.strip_prefix("--crate-name=") {
            return name == target_crate;
        }
    }
    false
}

/// Stand in for `rustc`, adding instrumentation to one crate.
///
/// Cargo invokes a wrapper as `<wrapper> <rustc> <args…>`, including for the
/// version probes it runs before any compilation; those carry no
/// `--crate-name` and pass straight through.
///
/// Returns the exit code to propagate.
pub(crate) fn run_as_rustc_wrapper(target_crate: &str) -> Result<i32> {
    let mut argv = std::env::args_os().skip(1);
    let rustc: OsString = argv
        .next()
        .context("RUSTC_WRAPPER invoked without a rustc path")?;
    let rest: Vec<OsString> = argv.collect();

    let lossy: Vec<String> = rest
        .iter()
        .map(|a| a.to_string_lossy().into_owned())
        .collect();

    let mut cmd = Command::new(rustc);
    cmd.args(&rest);
    if should_instrument(&lossy, target_crate) {
        cmd.arg("-Cinstrument-coverage");
    }
    Ok(cmd
        .status()
        .context("failed to run rustc")?
        .code()
        .unwrap_or(1))
}

/// Path to one of the toolchain's LLVM tools.
///
/// Taken from the active toolchain rather than `PATH` so the tool's version
/// matches the `rustc` that wrote the coverage mapping; a mismatched
/// `llvm-profdata` rejects the profile format outright.
fn llvm_tool(sh: &Shell, name: &str) -> Result<PathBuf> {
    let libdir = cmd!(sh, "rustc --print target-libdir").read()?;
    let bin = Path::new(libdir.trim())
        .parent()
        .context("target-libdir has no parent")?
        .join("bin")
        .join(name);
    if !bin.is_file() {
        bail!(
            "{name} not found at {} — run `rustup component add llvm-tools-preview`",
            bin.display()
        );
    }
    Ok(bin)
}

/// Whether the toolchain's coverage tools are present.
///
/// Their absence is a missing rustup component, not a failing crate, so the
/// criterion degrades to Manual rather than reporting an F nobody can act on.
pub(crate) fn tools_available(sh: &Shell) -> bool {
    llvm_tool(sh, "llvm-profdata").is_ok() && llvm_tool(sh, "llvm-cov").is_ok()
}

/// The test executable cargo reported building, from `--message-format=json`.
///
/// Cargo emits one `compiler-artifact` line per unit; only the test binary
/// carries both `profile.test` and an `executable`.
fn test_executable(message_json: &str) -> Option<PathBuf> {
    let mut found = None;
    for line in message_json.lines() {
        let Ok(v) = serde_json::from_str::<serde_json::Value>(line) else {
            continue;
        };
        if v["profile"]["test"].as_bool() == Some(true) {
            if let Some(exe) = v["executable"].as_str() {
                found = Some(PathBuf::from(exe));
            }
        }
    }
    found
}

/// How the export's files divide into the measured crate's own and everything
/// else: `(own count, foreign paths)`.
///
/// Both halves are checked, and they catch opposite failures.
///
/// **Foreign files** falsify the scoping claim outright — the wrapper matched
/// something it should not have, and the run went back to paying the full tax.
///
/// **Zero own files** is the quieter one. If the wrapper matched *nothing*, the
/// export is empty, which has no foreign files either and would sail through a
/// one-sided check; `production_coverage` would then report no production lines
/// and the criterion would return N/A. A coverage gate that silently becomes
/// inapplicable is the exact failure this project keeps finding, so the empty
/// case is an error rather than a zero.
fn partition_export(json: &serde_json::Value, crate_path: &str) -> (usize, Vec<String>) {
    let Some(files) = json["data"][0]["files"].as_array() else {
        return (0, Vec::new());
    };
    let (mut own, mut foreign) = (0, Vec::new());
    for name in files.iter().filter_map(|f| f["filename"].as_str()) {
        if name.contains(crate_path) {
            own += 1;
        } else {
            foreign.push(name.to_string());
        }
    }
    (own, foreign)
}

/// Build and run `crate_name`'s `--lib` tests with only that crate instrumented,
/// and export the coverage report.
///
/// `quiet` silences the child processes rather than the measurement: the test
/// harness writes to stdout, and `grade --json` puts its report there too.
pub(crate) fn measure_lib_coverage(
    sh: &Shell,
    crate_name: &str,
    crate_path: &str,
    workspace_root: &Path,
    quiet: bool,
) -> Result<LibCoverageRun> {
    let profdata_tool = llvm_tool(sh, "llvm-profdata")?;
    let cov_tool = llvm_tool(sh, "llvm-cov")?;
    let xtask_exe = std::env::current_exe().context("cannot locate the running xtask binary")?;

    let target_dir = workspace_root.join(COVERAGE_TARGET_DIR);
    let profraw_dir = target_dir.join("profraw");
    // Stale profraw from an earlier run would be merged into this one's
    // profile, crediting lines this run never executed.
    if profraw_dir.exists() {
        std::fs::remove_dir_all(&profraw_dir).context("failed to clear profraw directory")?;
    }
    std::fs::create_dir_all(&profraw_dir).context("failed to create profraw directory")?;

    // The compiler spells `-` as `_` in crate names.
    let compiler_crate_name = crate_name.replace('-', "_");

    // `json-render-diagnostics` keeps the machine-readable artifact stream on
    // stdout while compiler errors still reach stderr in human form — a plain
    // `json` build shows the user nothing while it compiles.
    let mut build = cmd!(
        sh,
        "cargo test --release -p {crate_name} --lib --no-run --message-format=json-render-diagnostics"
    )
    .env("RUSTC_WRAPPER", &xtask_exe)
    .env(WRAPPER_CRATE_ENV, &compiler_crate_name)
    .env("CARGO_TARGET_DIR", &target_dir);
    if quiet {
        build = build.ignore_stderr();
    }
    let build_json = build.read().context("instrumented build failed")?;

    let exe = test_executable(&build_json)
        .context("cargo reported no test executable for the --lib target")?;

    // `%p`/`%m` keep one process's profile separate from another's; the test
    // harness is threaded but single-process, so this is one file in practice.
    let mut test_run = Command::new(&exe);
    test_run.env("LLVM_PROFILE_FILE", profraw_dir.join("cf-%p-%m.profraw"));
    let tests_passed = if quiet {
        test_run
            .output()
            .context("failed to run the instrumented test binary")?
            .status
            .success()
    } else {
        test_run
            .status()
            .context("failed to run the instrumented test binary")?
            .success()
    };

    let raw: Vec<PathBuf> = std::fs::read_dir(&profraw_dir)
        .context("failed to read profraw directory")?
        .filter_map(|e| e.ok().map(|e| e.path()))
        .filter(|p| p.extension().is_some_and(|x| x == "profraw"))
        .collect();
    if raw.is_empty() {
        bail!(
            "the instrumented test binary wrote no .profraw files — \
             instrumentation did not reach {compiler_crate_name}"
        );
    }

    let merged = target_dir.join("merged.profdata");
    cmd!(sh, "{profdata_tool} merge -sparse {raw...} -o {merged}")
        .run()
        .context("llvm-profdata merge failed")?;

    let export = cmd!(
        sh,
        "{cov_tool} export --format=text --instr-profile={merged} {exe}"
    )
    .read()
    .context("llvm-cov export failed")?;

    let json: serde_json::Value =
        serde_json::from_str(&export).context("llvm-cov export was not valid JSON")?;

    let (own, foreign) = partition_export(&json, crate_path);
    if !foreign.is_empty() {
        bail!(
            "instrumentation escaped {crate_name}: {} foreign file(s) in the export, first is {}",
            foreign.len(),
            foreign[0]
        );
    }
    if own == 0 {
        bail!(
            "the export names no file under {crate_path} — \
             instrumentation did not reach {compiler_crate_name}"
        );
    }

    Ok(LibCoverageRun { json, tests_passed })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The wrapper sees every `rustc` in the build. Instrumenting the wrong one
    /// costs the 400× tax this module exists to remove; instrumenting none
    /// yields an empty report that reads as 0 % coverage.
    #[test]
    fn only_the_named_crate_is_instrumented() {
        let args: Vec<String> = [
            "--crate-name",
            "cf_fsu_model",
            "--edition=2021",
            "src/lib.rs",
        ]
        .iter()
        .map(|s| s.to_string())
        .collect();
        assert!(should_instrument(&args, "cf_fsu_model"));
        assert!(!should_instrument(&args, "nalgebra"));
    }

    /// `rustc` accepts the joined spelling, so a future cargo could switch to
    /// it. Missing that would silently instrument nothing.
    #[test]
    fn the_joined_crate_name_spelling_is_matched_too() {
        let args = vec![
            "--crate-name=sim_soft".to_string(),
            "src/lib.rs".to_string(),
        ];
        assert!(should_instrument(&args, "sim_soft"));
        assert!(!should_instrument(&args, "sim_types"));
    }

    /// Cargo probes the wrapper with `rustc -vV` before compiling anything.
    /// Adding a codegen flag to a version query is not fatal, but matching on
    /// a value that merely *follows* some other flag would be.
    #[test]
    fn a_version_probe_carries_no_crate_name() {
        assert!(!should_instrument(&["-vV".to_string()], "cf_fsu_model"));
        let decoy = vec!["--emit=metadata".to_string(), "cf_fsu_model".to_string()];
        assert!(
            !should_instrument(&decoy, "cf_fsu_model"),
            "a bare argument that happens to equal the crate name is not --crate-name"
        );
    }

    /// Cargo emits an artifact line per unit; only the test binary has both
    /// `profile.test` and an `executable`.
    #[test]
    fn the_test_binary_is_picked_out_of_the_artifact_stream() {
        let stream = concat!(
            r#"{"reason":"compiler-artifact","profile":{"test":false},"executable":null}"#,
            "\n",
            r#"{"reason":"compiler-artifact","profile":{"test":false},"filenames":["libdep.rlib"]}"#,
            "\n",
            "not json at all\n",
            r#"{"reason":"compiler-artifact","profile":{"test":true},"executable":"/t/deps/cf_fsu_model-abc"}"#,
            "\n",
        );
        assert_eq!(
            test_executable(stream),
            Some(PathBuf::from("/t/deps/cf_fsu_model-abc"))
        );
        assert_eq!(test_executable("{}"), None);
    }

    /// The scoping check is the run's own negative control: it must actually
    /// fire when a dependency's file appears, not merely return empty.
    #[test]
    fn a_dependency_file_in_the_export_is_reported_as_escape() {
        let json = serde_json::json!({
            "data": [{"files": [
                {"filename": "/w/sim/L1/fsu-model/src/lib.rs"},
                {"filename": "/w/sim/L0/soft/src/solver.rs"},
            ]}]
        });
        assert_eq!(
            partition_export(&json, "sim/L1/fsu-model/"),
            (1, vec!["/w/sim/L0/soft/src/solver.rs".to_string()])
        );

        let clean = serde_json::json!({
            "data": [{"files": [{"filename": "/w/sim/L1/fsu-model/src/lib.rs"}]}]
        });
        assert_eq!(
            partition_export(&clean, "sim/L1/fsu-model/"),
            (1, Vec::new())
        );
    }

    /// An export with no files at all has no foreign files either. Checking
    /// only for escapes would pass it, and the criterion would then report
    /// "no production lines" — a coverage gate that had quietly stopped
    /// measuring anything.
    #[test]
    fn an_empty_export_is_not_mistaken_for_a_clean_one() {
        let empty = serde_json::json!({ "data": [{"files": []}] });
        assert_eq!(
            partition_export(&empty, "sim/L1/fsu-model/"),
            (0, Vec::new())
        );

        let malformed = serde_json::json!({});
        assert_eq!(
            partition_export(&malformed, "sim/L1/fsu-model/"),
            (0, Vec::new())
        );
    }
}
