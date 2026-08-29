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
//! ⚠ Re-running these today reproduces none of the coverage rows, and the
//! reason is worth keeping. The table predates the `--lib` → `--lib --tests`
//! change, so `cf-fsu-model`'s 84.2 % was expected to drift once its 4
//! integration tests began counting. `sim-types` was called out as the row
//! that would survive — no `tests/` directory, so nothing new to count — and
//! it drifted anyway: #776 added `#[cfg(test)]` units in `src/`, taking it to
//! **97.7 %** (measured 2026-08-17). The exception was reasoned about the one
//! axis that moves a figure and overtaken by another: somebody wrote tests.
//!
//! The A/B's *claim* — that scoping instrumentation cannot move the number —
//! is unaffected, because it rests on the two pipelines agreeing with each
//! other on one tree, not on either figure surviving the tree changing. Read
//! every absolute number here as of its moment.
//!
//! [`own_files_in_export`] re-checks the load-bearing half on every run.
//!
//! ## Why a `RUSTC_WRAPPER`
//!
//! Cargo has no stable per-package `rustflags`: `profile-rustflags` is nightly
//! (checked on 1.96, which rejects it outright). `RUSTC_WRAPPER` is the one
//! stable interception point — cargo runs it in place of `rustc` for every
//! unit and passes the real `rustc` as the first argument, so the wrapper can
//! add `-C instrument-coverage` to just the measured crate and its own test
//! targets, leaving the whole dependency tree fully optimised.
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
///
/// ⚠ Named `CoverageRun`, not `LibCoverageRun` as it once was: the run stopped
/// being `--lib`-only, and a type whose name says otherwise is how the old
/// scope kept being assumed.
pub(crate) struct CoverageRun {
    /// The llvm-cov JSON export, in the same shape `cargo llvm-cov --json`
    /// produced — [`crate::coverage::production_coverage`] reads it unchanged.
    pub json: serde_json::Value,
    /// Whether the `--lib` unit-test binary passed.
    ///
    /// ⚠ Integration binaries run and contribute coverage but do NOT gate this
    /// — under `-C instrument-coverage` a wall-clock or memory-ceiling
    /// assertion fails because it is being measured, not because the code
    /// regressed.
    ///
    /// That is not a hole: `grade`'s pass 2 runs the whole suite
    /// uninstrumented and gates on it, so an integration failure that is real
    /// still blocks the grade. This flag is only the instrumented `--lib`
    /// half. See the run loop in [`measure_coverage`] for the measurement.
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

/// True when this `rustc` invocation builds a test target (`--test`).
///
/// ★ Load-bearing, and not obvious. Instrumenting only the measured crate
/// leaves the integration binaries that LINK it without the profiling runtime,
/// and they then write incomplete profiles or none at all — measured on
/// `cf-geometry`, 7 profraw files for 8 binaries, and `aabb.rs` stuck at its
/// `--lib` figure of 49.5 % when the truth is 91.9 %. Instrumenting the test
/// targets as well makes every binary emit a profile.
///
/// Only this package's own targets are built with `--test`; dependencies are
/// not, so this stays scoped and cheap. Their own source is excluded from the
/// accounting by [`crate::coverage::is_own_production_file`].
fn is_test_target(args: &[String]) -> bool {
    args.iter().any(|a| a == "--test")
}

/// Stand in for `rustc`, adding instrumentation to the measured crate and its
/// test targets.
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
    if should_instrument(&lossy, target_crate) || is_test_target(&lossy) {
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

impl TestBinary {
    /// Filename-safe key identifying this binary's profiles on disk.
    ///
    /// Cargo already makes the file stem unique per target with its hash
    /// suffix, so this needs no disambiguation of its own. Used as the
    /// `LLVM_PROFILE_FILE` prefix so a later merge can select or exclude one
    /// binary's profiles by name.
    fn profile_key(&self) -> String {
        self.path.file_name().map_or_else(
            || "unknown".to_string(),
            |n| n.to_string_lossy().into_owned(),
        )
    }

    /// The target name a reader recognises — the key with cargo's hash suffix
    /// dropped, so `bonded_layer_indentation-1a2b3c` reads as
    /// `bonded_layer_indentation`.
    ///
    /// ⚠ Splits on the LAST `-` because that is the only one cargo puts there:
    /// a target name's own dashes are already `_` in the file stem.
    fn display_name(&self) -> String {
        let key = self.profile_key();
        key.rsplit_once('-')
            .map_or_else(|| key.clone(), |(head, _)| head.to_string())
    }
}

/// One binary's price in the coverage pass: what it cost, and what only it covered.
pub(crate) struct Contribution {
    /// Target name, hash suffix dropped.
    pub name: String,
    /// Instrumented wall seconds.
    pub seconds: f64,
    /// Production lines this binary covered that NOTHING else did. Zero means
    /// excluding it leaves the reported coverage bit-identical.
    pub marginal_lines: u64,
}

/// What one binary cost in the instrumented run, keyed to its profiles.
struct BinaryTiming {
    /// Matches the `LLVM_PROFILE_FILE` prefix its profiles were written under.
    key: String,
    /// The same binary as a reader names it — see [`TestBinary::display_name`].
    name: String,
    /// Wall seconds. Instrumented, so 7x-1226x the uninstrumented figure
    /// depending on how much arithmetic the binary does per line executed.
    seconds: f64,
}

/// Every test executable cargo reported building, from `--message-format=json`.
///
/// Cargo emits one `compiler-artifact` line per unit; only test binaries carry
/// both `profile.test` and an `executable`. A crate with a `tests/` directory
/// yields several — the lib one plus one per integration file.
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

/// How many of the export's files are the measured crate's own PRODUCTION
/// sources — see [`crate::coverage::is_own_production_file`], which excludes
/// its `tests/`.
///
/// Zero is an error, not a zero percent. If the wrapper ever stopped matching —
/// a future cargo spelling `--crate-name` differently, say — nothing would be
/// instrumented, the export would be empty, `production_coverage` would find no
/// production lines, and the criterion would quietly return N/A. A coverage gate
/// that silently stops applying is the failure this project keeps finding, so
/// the empty case is reported as the breakage it is.
///
/// ⚠ The symmetric check — "no *foreign* file may appear" — was written first
/// and removed, and it would be even more wrong today. Its premise was that
/// [`should_instrument`] compares against exactly one crate name, so nothing
/// else could be instrumented; [`is_test_target`] now instruments the crate's
/// test targets too, and their `tests/*.rs` source appears in the export by
/// design. Even before that the check could only fire on something else, and
/// it did, immediately: `mesh-repair`'s export names
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
            .filter(|name| crate::coverage::is_own_production_file(name, crate_path))
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

/// Build and run `crate_name`'s unit AND integration tests, instrumented and
/// scoped to that crate plus its own test targets, and export the report.
///
/// ★ `--lib --tests`, not `--lib`. Measuring the lib target alone credits only
/// what a crate's own `#[cfg(test)]` modules execute, so a crate that keeps its
/// tests in `tests/` reads as barely covered while being thoroughly tested.
/// Measured on `cf-geometry`, whose 186 tests live in `tests/`: `mesh.rs` read
/// **7.27 %** under `--lib` and **100 %** with the integration binaries
/// included. The criterion is meant to assert "this production code is
/// exercised", and where the exercising test happens to live is not part of
/// that claim.
///
/// ## The gating matrix, exercised rather than reasoned
///
/// | scenario | grade |
/// |---|---|
/// | lib test fails | **F** |
/// | integration test fails always | **F** — pass 2 catches it |
/// | integration test fails ONLY instrumented | **A** — perf/OOM asserts, see the run loop |
/// | all pass | **A** |
///
/// All four were run: a temporary failing test in `mesh-types`' lib and in
/// `mesh-io`'s `tests/` both produced F and reverted to A; `mesh-printability`
/// grades A+ while two of its stress tests fail under instrumentation.
///
/// ⚠ This can only ever RAISE a crate's percentage: the denominator is the
/// crate's own production lines either way, and adding binaries can only cover
/// more of them. No crate can newly fall below threshold because of it.
///
/// ## Why integration binaries do not inflate the denominator
///
/// `tests/*.rs` sits *under* the crate directory and is not `cfg(test)`-gated,
/// so counting it would pad both sides with ~100 %-covered test source. It is
/// excluded, but by the accounting rather than by absence — and that is worth
/// stating precisely, because the earlier design got there the other way.
///
/// Test targets ARE instrumented ([`is_test_target`]), because otherwise their
/// binaries emit no usable profile and the coverage they produce is lost. So
/// their source DOES reach the export: audited on `cf-geometry`, the export
/// names **24 files, 7 of them under `tests/`**.
/// [`crate::coverage::is_own_production_file`]
/// drops those 7, leaving the 17 `src/*.rs` files that are the crate itself.
///
/// ⚠ An earlier revision of this doc claimed the opposite mechanism — that
/// integration source "cannot appear in the export" because only the measured
/// crate is instrumented. That was true then and is false now; it stopped
/// being true the moment test targets had to be instrumented to emit profiles
/// at all. If the filter is ever removed on the strength of that old argument,
/// the denominator silently gains ~100 %-covered test bodies.
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
/// instrumentation tax itself is unchanged by this — but it is worth recording
/// that the tax is far wider than the ~400x the module header measured on the
/// whole-tree pipeline. Scoped, per lib suite: `mesh-printability` 0.25 s clean
/// vs 41 s instrumented (**164x**), `mesh-repair` 0.32 s vs 392 s (**1226x**).
/// It tracks how much of the workload is instrumented inner-loop code, so quote
/// it as a range that wide rather than a single figure.
///
/// `quiet` silences the child processes rather than the measurement: the test
/// harness writes to stdout, and `grade --json` puts its report there too.
pub(crate) fn measure_coverage(
    sh: &Shell,
    crate_name: &str,
    crate_path: &str,
    workspace_root: &Path,
    quiet: bool,
) -> Result<CoverageRun> {
    measure_inner(sh, crate_name, crate_path, workspace_root, quiet, None).map(|(run, _)| run)
}

/// Price every binary that costs more than `threshold_seconds`: what it took,
/// and what it uniquely covered.
///
/// ★ Exists because "exclude the slow tests from coverage" is only safe for a
/// binary whose MARGINAL contribution is zero — exclude one that solely covers
/// a line and the report calls that line untested, which is a lie in the
/// report rather than the silence the `integration-only` opt-in produced, but a
/// lie either way. This measures which is which instead of asserting it, and is
/// re-runnable, so the exclusion list stays derived rather than inherited.
///
/// ⚠ One full instrumented run — for `sim-soft` that is ~62 min. The subset
/// arithmetic afterwards is cheap (a merge and an export per candidate), which
/// is why the threshold matters: a binary too cheap to be worth excluding is
/// not worth an export either.
pub(crate) fn census_coverage(
    sh: &Shell,
    crate_name: &str,
    crate_path: &str,
    workspace_root: &Path,
    quiet: bool,
    threshold_seconds: f64,
) -> Result<Vec<Contribution>> {
    measure_inner(
        sh,
        crate_name,
        crate_path,
        workspace_root,
        quiet,
        Some(threshold_seconds),
    )
    .map(|(_, c)| c)
}

fn measure_inner(
    sh: &Shell,
    crate_name: &str,
    crate_path: &str,
    workspace_root: &Path,
    quiet: bool,
    census: Option<f64>,
) -> Result<(CoverageRun, Vec<Contribution>)> {
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
    let mut timings: Vec<BinaryTiming> = Vec::new();
    for bin in &exes {
        // ★ PREFIXED PER BINARY, not a single shared `cf-%p-%m`. The merge
        // below still globs the whole directory, so the measured number is
        // unchanged — but the profiles stay ATTRIBUTABLE, which is what lets
        // `coverage-census` merge every subset except one binary and price
        // that binary's unique contribution. Without the prefix the profiles
        // are an undifferentiated pile and the only answerable question is
        // "all or nothing".
        let key = bin.profile_key();
        let mut test_run = Command::new(&bin.path);
        test_run.env(
            "LLVM_PROFILE_FILE",
            profraw_dir.join(format!("{key}__cf-%p-%m.profraw")),
        );
        let started = std::time::Instant::now();
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
        // workspace 164-1226x on the suites measured here, so any test
        // asserting a wall-clock budget or a memory ceiling fails BECAUSE it is
        // being measured.
        //
        // Nothing is lost by not gating here: `grade`'s pass 2 already runs
        // `cargo test --release -p <crate>` UNINSTRUMENTED, which covers lib,
        // integration and doctests, and `heavy_passed` gates on it. Gating
        // pass 1 as well would be a second, WORSE detector of the same thing.
        // Measured on `mesh-printability` in one grade run:
        // `stress_c_5k_tri_perf_budget` and `stress_h_voxel_grid_oom_safety`
        // fail in pass 1 instrumented (43 passed / 2 failed, 36.95 s) and the
        // same suite passes in pass 2 (45/45, 0.21 s). Gating here would have
        // graded that crate F against evidence the same run already had.
        if bin.is_lib {
            tests_passed &= ok;
        }
        timings.push(BinaryTiming {
            key: bin.profile_key(),
            name: bin.display_name(),
            seconds: started.elapsed().as_secs_f64(),
        });
    }

    let raw: Vec<PathBuf> = std::fs::read_dir(&profraw_dir)
        .context("failed to read profraw directory")?
        .filter_map(|e| e.ok().map(|e| e.path()))
        .filter(|p| p.extension().is_some_and(|x| x == "profraw"))
        .collect();
    if raw.is_empty() {
        bail!(
            "the instrumented test binaries wrote no .profraw files — \
             instrumentation did not reach {compiler_crate_name}"
        );
    }

    let raw_all = &raw;
    let merged = target_dir.join("merged.profdata");
    cmd!(sh, "{profdata_tool} merge -sparse {raw_all...} -o {merged}")
        .run()
        .context("llvm-profdata merge failed")?;

    // llvm-cov takes one binary positionally and every other behind its own
    // `-object`; all are equal to it, and which lands first is just cargo's
    // artifact order. Passing only one would report that binary's view alone —
    // the exact blindness this measures around.
    let (first_exe, other_exes) = exes.split_first().expect("emptiness checked above");
    let first_exe = &first_exe.path;
    let object_args: Vec<String> = other_exes
        .iter()
        .flat_map(|b| ["-object".to_string(), b.path.display().to_string()])
        .collect();
    let objects = &object_args;
    let export = cmd!(
        sh,
        "{cov_tool} export --format=text --instr-profile={merged} {first_exe} {objects...}"
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

    let contributions = match census {
        None => Vec::new(),
        Some(threshold) => {
            let full = crate::coverage::production_coverage(&json, crate_path).covered;
            let mut out = Vec::new();
            for t in timings.iter().filter(|t| t.seconds >= threshold) {
                // Everything EXCEPT this binary's profiles. `covered` can only
                // fall, so the difference is exactly the lines nothing else
                // reached.
                let keep: Vec<PathBuf> = raw
                    .iter()
                    .filter(|p| {
                        !p.file_name()
                            .and_then(|n| n.to_str())
                            .is_some_and(|n| n.starts_with(&format!("{}__", t.key)))
                    })
                    .cloned()
                    .collect();
                let sans_covered = if keep.is_empty() {
                    0
                } else {
                    let sans = target_dir.join("sans.profdata");
                    let keep_ref = &keep;
                    cmd!(sh, "{profdata_tool} merge -sparse {keep_ref...} -o {sans}")
                        .run()
                        .context("llvm-profdata merge failed for the census subset")?;
                    let sans_export = cmd!(
                        sh,
                        "{cov_tool} export --format=text --instr-profile={sans} {first_exe} {objects...}"
                    )
                    .read()
                    .context("llvm-cov export failed for the census subset")?;
                    let sans_json: serde_json::Value = serde_json::from_str(&sans_export)
                        .context("census subset export was not valid JSON")?;
                    crate::coverage::production_coverage(&sans_json, crate_path).covered
                };
                out.push(Contribution {
                    name: t.name.clone(),
                    seconds: t.seconds,
                    marginal_lines: full.saturating_sub(sans_covered),
                });
            }
            // Cheapest exclusions first: zero marginal, most time saved.
            out.sort_by(|a, b| {
                a.marginal_lines
                    .cmp(&b.marginal_lines)
                    .then(b.seconds.total_cmp(&a.seconds))
            });
            out
        }
    };

    Ok((CoverageRun { json, tests_passed }, contributions))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The wrapper sees every `rustc` in the build. Instrumenting the wrong one
    /// costs the instrumentation tax this module exists to remove (164-1226x on
    /// the suites measured); instrumenting none yields an empty report that
    /// reads as 0 % coverage.
    ///
    /// ⚠ Scoped to [`should_instrument`] alone. The wrapper also instruments
    /// test targets ([`is_test_target`]), so "only the named crate" is not a
    /// module-level claim.
    #[test]
    fn should_instrument_matches_only_the_named_crate() {
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

    /// A `--test` invocation is what marks a target for instrumentation beyond
    /// the measured crate itself; a plain dependency build is not.
    #[test]
    fn only_test_targets_are_instrumented_beyond_the_measured_crate() {
        assert!(is_test_target(&["--test".to_string()]));
        assert!(!is_test_target(&[
            "--crate-name".to_string(),
            "nalgebra".to_string()
        ]));
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
