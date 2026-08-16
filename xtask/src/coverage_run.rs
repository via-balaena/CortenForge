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
//! ## Why scoping cannot change the number, and the A/B that checked
//!
//! The criterion has only ever counted the measured crate's own files. Lines
//! belonging to a dependency were instrumented, exported, and then discarded by
//! the path filter. Removing instrumentation from those crates removes data
//! that was already being dropped — the numerator and denominator are built
//! from the same files either way.
//!
//! Argued that way it is still an argument, so it was measured. Same crate,
//! same commit, old pipeline then new:
//!
//! | crate | pipeline | coverage | grade wall |
//! |---|---|---|---|
//! | `sim-types` | whole-tree | 192/271 → 70.8 % | 30 s |
//! | `sim-types` | scoped | **192/271 → 70.8 %** | **11 s** |
//! | `cf-fsu-model` | whole-tree | 84.2 % | 3105 s |
//! | `cf-fsu-model` | scoped, cold tree | **585/695 → 84.2 %** | **83 s** |
//! | `cf-fsu-model` | scoped, warm tree | 84.2 % | **17 s** |
//!
//! `sim-types` agrees line for line. `cf-fsu-model` agrees to the three figures
//! the old run recorded — its exact counts were never printed, and re-running
//! the old pipeline to recover them costs 51 minutes for a digit.
//!
//! [`own_files_in_export`] re-checks the load-bearing half on every run.
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

/// What one instrumented run produced, across the crate's unit AND
/// integration test binaries.
pub(crate) struct LibCoverageRun {
    /// The llvm-cov JSON export, in the same shape `cargo llvm-cov --json`
    /// produced — [`crate::coverage::production_coverage`] reads it unchanged.
    pub json: serde_json::Value,
    /// Whether the `--lib` unit-test binary passed.
    ///
    /// ⚠ Integration binaries run and contribute coverage but do NOT gate this.
    /// Under `-C instrument-coverage` a wall-clock or memory-ceiling assertion
    /// fails because it is being measured, not because the code regressed; see
    /// the run loop in [`measure_lib_coverage`] for the measurement that
    /// settled it.
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

/// One instrumented test binary cargo built for the crate.
struct TestBinary {
    /// Path to the executable.
    path: PathBuf,
    /// Whether this is the `--lib` unit-test binary, as opposed to an
    /// integration binary from `tests/`. Only the lib one gates pass/fail.
    ///
    /// Taken from `target.kind`, which is exactly `["lib"]` for all 63 lib
    /// targets in this workspace — none declares a `crate-type`. A crate that
    /// later did (`cdylib`, `proc-macro`, …) would report a different kind and
    /// trip the "no --lib test binary" bail rather than silently handing the
    /// gate to an integration suite.
    is_lib: bool,
}

/// The test executable cargo reported building, from `--message-format=json`.
///
/// Cargo emits one `compiler-artifact` line per unit; only the test binary
/// carries both `profile.test` and an `executable`.
fn test_executables(message_json: &str) -> Vec<TestBinary> {
    let mut found = Vec::new();
    for line in message_json.lines() {
        let Ok(v) = serde_json::from_str::<serde_json::Value>(line) else {
            continue;
        };
        if v["profile"]["test"].as_bool() == Some(true) {
            if let Some(exe) = v["executable"].as_str() {
                let is_lib = v["target"]["kind"]
                    .as_array()
                    .is_some_and(|k| k.iter().any(|x| x.as_str() == Some("lib")));
                found.push(TestBinary {
                    path: PathBuf::from(exe),
                    is_lib,
                });
            }
        }
    }
    found
}

/// How many of the export's files belong to the measured crate.
///
/// Zero is an error, not a zero percent. If the wrapper ever stopped matching —
/// a future cargo spelling `--crate-name` differently, say — nothing would be
/// instrumented, the export would be empty, `production_coverage` would find no
/// production lines, and the criterion would quietly return N/A. A coverage gate
/// that silently stops applying is the failure this project keeps finding, so
/// the empty case is reported as the breakage it is.
///
/// ⚠ The symmetric check — "no *foreign* file may appear" — was written first
/// and removed, because it cannot fail for the reason it claimed.
/// [`should_instrument`] compares against exactly one crate name, so the wrapper
/// is incapable of instrumenting a second crate; the check could only ever fire
/// on something else. It did, immediately: `mesh-repair`'s export names
/// `tracing`'s `macros.rs`, because a macro's expansion carries the regions of
/// the file it was *written* in, not the file it was expanded into. Those lines
/// were in the old pipeline's export too, and `production_coverage` has always
/// dropped them by path. Instrumentation escaping to a real second crate is
/// prevented by construction above and by [`COVERAGE_TARGET_DIR`]'s reset below,
/// not by inspecting the export.
fn own_files_in_export(json: &serde_json::Value, crate_path: &str) -> usize {
    json["data"][0]["files"].as_array().map_or(0, |files| {
        files
            .iter()
            .filter_map(|f| f["filename"].as_str())
            .filter(|name| name.contains(crate_path))
            .count()
    })
}

/// Point the coverage build at a tree that holds only this crate's artifacts.
///
/// The wrapper decides what to instrument from an environment variable, and
/// cargo's fingerprint covers the wrapper's *path*, not that variable's value.
/// Cargo therefore cannot tell two coverage builds apart, and will reuse an
/// instrumented artifact built while its crate was the target. Grading
/// `sim-types` and then anything downstream of it would silently link the
/// instrumented `sim-types` — reintroducing the tax this module removes, on a
/// crate the run never intended to instrument.
///
/// Cargo cannot be taught the difference on stable, so the tree is reset when
/// the measured crate changes. Iterating on one crate — the loop that actually
/// happens — stays warm; moving to another pays one full build. The alternative,
/// a directory per crate, keeps every tree warm and grows `target/` without
/// bound.
///
/// ⚠ The reset is what keeps the fix from decaying: without it, each graded
/// crate leaves an instrumented artifact behind for every later crate that
/// depends on it, and the tax creeps back one dependency at a time. The price is
/// paid by `grade-all` *with* coverage, which pays one build per crate — an
/// unusual way to run it, and the reason CI passes `--skip-coverage`.
fn prepare_target_dir(workspace_root: &Path, compiler_crate_name: &str) -> Result<PathBuf> {
    let target_dir = workspace_root.join(COVERAGE_TARGET_DIR);
    let stamp = target_dir.join(".measured-crate");

    let previous = std::fs::read_to_string(&stamp).unwrap_or_default();
    if previous.trim() != compiler_crate_name && target_dir.exists() {
        std::fs::remove_dir_all(&target_dir)
            .context("failed to reset the coverage build directory")?;
    }
    std::fs::create_dir_all(&target_dir)
        .context("failed to create the coverage build directory")?;
    std::fs::write(&stamp, compiler_crate_name).context("failed to stamp the coverage build")?;
    Ok(target_dir)
}

/// Build and run `crate_name`'s unit AND integration tests with only that crate
/// instrumented, and export the coverage report.
///
/// ★ `--lib --tests`, not `--lib`. Measuring the lib target alone credits only
/// what a crate's own `#[cfg(test)]` modules execute, so a crate that keeps its
/// tests in `tests/` reads as barely covered while being thoroughly tested.
/// Measured on `cf-geometry`, whose 184 tests live in `tests/`: `mesh.rs` read
/// **7.27 %** under `--lib` and **100 %** with the integration binaries
/// included. The criterion is meant to assert "this production code is
/// exercised", and where the exercising test happens to live is not part of
/// that claim.
///
/// ⚠ This can only ever RAISE a crate's percentage: the denominator is the
/// crate's own production lines either way, and adding binaries can only cover
/// more of them. No crate can newly fall below threshold because of it.
///
/// ## Why integration binaries cannot inflate the denominator
///
/// The obvious worry is that `tests/*.rs` sits *under* the crate directory and
/// is not `cfg(test)`-gated, so counting it would pad both sides with
/// ~100 %-covered test source. It cannot happen: the [`WRAPPER_CRATE_ENV`]
/// wrapper instruments only the compilation whose `--crate-name` matches, and
/// an integration crate compiles under its own name (`mesh_tests`, …). Never
/// instrumented means no coverage mapping, which means it cannot appear in the
/// export — while its *execution* still fires the counters inside the
/// instrumented lib it links.
///
/// Audited on `cf-geometry` rather than argued: the export names **17 files, 0
/// of them under `tests/`** — every one a `src/*.rs` of the crate itself.
///
/// ## Known limit: doctests are still not counted
///
/// `--lib --tests` does not build doctests, and this pipeline structurally
/// cannot reach them: doctests are compiled by **rustdoc**, not `rustc`, so a
/// `RUSTC_WRAPPER` never sees them. They are not a rounding error — measured,
/// `mesh-repair` has 27, `cf-design` 19, `sim-core` 12 — and they do exercise
/// production code, so a crate leaning on doc examples still reads low here.
/// Same direction as everything above: counting them could only raise numbers.
///
/// ## What the extra binaries cost
///
/// Little, because instrumented time is dominated by the `--lib` binary that
/// already ran. Measured per binary:
///
/// | crate | lib (was already paid) | integration (added) |
/// |---|---|---|
/// | `cf-geometry` | 0.79 s | ~0.00 s across 7 binaries |
/// | `mesh-repair` | **392 s** | 0.06 s |
/// | `mesh-printability` | 41 s | **+37 s** (a stress suite) |
///
/// So the added cost is bounded by what a crate keeps in `tests/`, and the
/// instrumentation tax itself is unchanged — `mesh-repair`'s 117 lib tests run
/// in 0.32 s clean and 392 s instrumented, the ~100-400x this module documents.
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

    // The compiler spells `-` as `_` in crate names.
    let compiler_crate_name = crate_name.replace('-', "_");

    let target_dir = prepare_target_dir(workspace_root, &compiler_crate_name)?;
    let profraw_dir = target_dir.join("profraw");
    // Stale profraw from an earlier run would be merged into this one's
    // profile, crediting lines this run never executed.
    if profraw_dir.exists() {
        std::fs::remove_dir_all(&profraw_dir).context("failed to clear profraw directory")?;
    }
    std::fs::create_dir_all(&profraw_dir).context("failed to create profraw directory")?;

    // `json-render-diagnostics` keeps the machine-readable artifact stream on
    // stdout while compiler errors still reach stderr in human form — a plain
    // `json` build shows the user nothing while it compiles.
    let mut build = cmd!(
        sh,
        "cargo test --release -p {crate_name} --lib --tests --no-run --message-format=json-render-diagnostics"
    )
    .env("RUSTC_WRAPPER", &xtask_exe)
    .env(WRAPPER_CRATE_ENV, &compiler_crate_name)
    .env("CARGO_TARGET_DIR", &target_dir);
    if quiet {
        build = build.ignore_stderr();
    }
    let build_json = build.read().context("instrumented build failed")?;

    let exes = test_executables(&build_json);
    if exes.is_empty() {
        bail!("cargo reported no test executable for the --lib/--tests targets");
    }
    if !exes.iter().any(|b| b.is_lib) {
        bail!("cargo reported no --lib test binary; the pass/fail gate would be vacuous");
    }

    // `%p`/`%m` keep one process's profile separate from another's. Each test
    // binary is threaded but single-process, so this is one file per binary.
    // They all land in the same directory and merge together below — an
    // integration test that exercises production code the unit tests miss
    // credits those lines exactly as a unit test would.
    let mut tests_passed = true;
    for bin in &exes {
        let mut test_run = Command::new(&bin.path);
        test_run.env("LLVM_PROFILE_FILE", profraw_dir.join("cf-%p-%m.profraw"));
        let ok = if quiet {
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
        // ⚠ Only the lib binary gates. Integration suites still RUN — their
        // coverage is the whole point — but an instrumented failure there is
        // not evidence about the code. `-C instrument-coverage` costs this
        // workspace 100-400x, so any test asserting a wall-clock budget or a
        // memory ceiling fails BECAUSE it is being measured. Measured on
        // `mesh-printability`: `stress_c_5k_tri_perf_budget` and
        // `stress_h_voxel_grid_oom_safety` fail instrumented and pass clean
        // (45/45 in 0.23 s), which would have graded a healthy crate F.
        // Whether integration tests pass is CI's `tests` / `tests-release`
        // jobs' question, and they run them uninstrumented.
        if bin.is_lib {
            tests_passed &= ok;
        }
    }

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

    // llvm-cov takes the first binary positionally and every other one behind
    // its own `-object`. Passing only the first would report coverage for the
    // unit-test binary alone — the exact blindness this measures around.
    let (first_exe, other_exes) = exes.split_first().expect("emptiness checked above");
    let first_exe = &first_exe.path;
    let object_args: Vec<String> = other_exes
        .iter()
        .flat_map(|b| ["-object".to_string(), b.path.display().to_string()])
        .collect();
    let export = cmd!(
        sh,
        "{cov_tool} export --format=text --instr-profile={merged} {first_exe} {object_args...}"
    )
    .read()
    .context("llvm-cov export failed")?;

    let json: serde_json::Value =
        serde_json::from_str(&export).context("llvm-cov export was not valid JSON")?;

    if own_files_in_export(&json, crate_path) == 0 {
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

    /// Cargo emits an artifact line per unit; only test binaries have both
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
        let found = test_executables(stream);
        assert_eq!(found.len(), 1);
        assert_eq!(found[0].path, PathBuf::from("/t/deps/cf_fsu_model-abc"));
        assert!(test_executables("{}").is_empty());
    }

    /// ★ Every test binary must be collected, and the lib one distinguished.
    ///
    /// Two failure modes this pins, both silent. Returning one binary would
    /// measure only the unit suite — the blindness `--lib --tests` exists to
    /// remove. Mislabelling `is_lib` would hand the pass/fail gate to an
    /// integration suite, and instrumented perf-budget tests fail for reasons
    /// that are not about the code (see the run loop).
    #[test]
    fn integration_binaries_are_collected_and_the_lib_one_is_marked() {
        let stream = concat!(
            r#"{"reason":"compiler-artifact","profile":{"test":true},"target":{"kind":["lib"],"name":"cf_geometry"},"executable":"/t/deps/cf_geometry-1"}"#,
            "\n",
            r#"{"reason":"compiler-artifact","profile":{"test":true},"target":{"kind":["test"],"name":"mesh_tests"},"executable":"/t/deps/mesh_tests-2"}"#,
            "\n",
            r#"{"reason":"compiler-artifact","profile":{"test":true},"target":{"kind":["test"],"name":"aabb_tests"},"executable":"/t/deps/aabb_tests-3"}"#,
            "\n",
        );
        let found = test_executables(stream);
        assert_eq!(found.len(), 3, "every test binary must be measured");
        assert_eq!(
            found.iter().filter(|b| b.is_lib).count(),
            1,
            "exactly one lib binary gates pass/fail"
        );
        assert!(found[0].is_lib, "the lib target is the one marked");
        assert!(!found[1].is_lib && !found[2].is_lib);
    }

    /// A macro carries the regions of the file it was *written* in, not the one
    /// it expanded into, so a dependency's path in the export is ordinary and
    /// must not read as a scoping failure. `mesh-repair` really does export
    /// `tracing`'s `macros.rs`; the first version of this module errored on it.
    #[test]
    fn a_macro_source_from_a_dependency_is_neither_counted_nor_fatal() {
        let json = serde_json::json!({
            "data": [{"files": [
                {"filename": "/w/mesh/mesh-repair/src/lib.rs"},
                {"filename": "/home/u/.cargo/registry/src/x/tracing-0.1.44/src/macros.rs"},
            ]}]
        });
        assert_eq!(own_files_in_export(&json, "mesh/mesh-repair/"), 1);
    }

    /// If the wrapper ever stops matching, nothing is instrumented, the export
    /// is empty, `production_coverage` finds no production lines, and the
    /// criterion grades N/A. The measurement has to fail loudly rather than
    /// evaporate into "not applicable".
    #[test]
    fn an_export_naming_none_of_the_crates_files_is_a_breakage() {
        let empty = serde_json::json!({ "data": [{"files": []}] });
        assert_eq!(own_files_in_export(&empty, "sim/L1/fsu-model/"), 0);

        let malformed = serde_json::json!({});
        assert_eq!(own_files_in_export(&malformed, "sim/L1/fsu-model/"), 0);

        let only_foreign = serde_json::json!({
            "data": [{"files": [{"filename": "/reg/tracing/src/macros.rs"}]}]
        });
        assert_eq!(own_files_in_export(&only_foreign, "sim/L1/fsu-model/"), 0);
    }

    /// Cargo fingerprints the wrapper's path, not the environment variable that
    /// tells it what to instrument, so it cannot tell two coverage builds apart.
    /// The stamp is the whole defence against one crate's instrumented artifacts
    /// being linked into the next crate's measurement.
    #[test]
    fn the_build_tree_is_reset_when_the_measured_crate_changes() {
        let tmp = std::env::temp_dir().join(format!("cf-cov-stamp-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&tmp);
        std::fs::create_dir_all(&tmp).expect("temp dir");

        let dir = prepare_target_dir(&tmp, "sim_types").expect("first prepare");
        let artifact = dir.join("release").join("libsim_types.rlib");
        std::fs::create_dir_all(artifact.parent().expect("parent")).expect("mkdir");
        std::fs::write(&artifact, b"instrumented").expect("write");

        prepare_target_dir(&tmp, "sim_types").expect("second prepare");
        assert!(
            artifact.is_file(),
            "re-measuring one crate must keep its warm tree"
        );

        prepare_target_dir(&tmp, "mesh_repair").expect("third prepare");
        assert!(
            !artifact.exists(),
            "an artifact instrumented for sim_types outlived the switch to mesh_repair"
        );

        let _ = std::fs::remove_dir_all(&tmp);
    }
}
