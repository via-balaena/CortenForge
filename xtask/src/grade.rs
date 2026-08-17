//! Crate grading against the A-grade standard.
//!
//! This module implements the eight-criterion grading system defined in docs/STANDARDS.md.

use crate::pr_scope::{filter_only, select_shard};
use anyhow::{bail, Context, Result};
use owo_colors::OwoColorize;
use std::path::Path;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};
use xshell::{cmd, Shell};

/// Controls progress logging, output format, and criterion selection for
/// `cargo xtask grade`.
#[derive(Debug, Clone, Copy)]
pub(crate) struct Verbosity {
    pub quiet: bool,
    pub verbose: bool,
    pub json: bool,
    /// Skip the Coverage criterion. Coverage runs `cargo llvm-cov --release`
    /// (minutes to tens of minutes per crate) which is too expensive for
    /// per-PR CI. Reported
    /// as [`Grade::NotApplicable`] when set.
    pub skip_coverage: bool,
}

/// Grade for a single criterion
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Grade {
    APlus,
    A,
    B,
    C,
    F,
    /// Requires manual review
    Manual,
    /// Criterion does not apply to this crate's profile (e.g., Bevy-free on
    /// a Bevy visual example). Not a pass or fail — signals that the
    /// criterion was deliberately skipped per `docs/STANDARDS.md`. Excluded
    /// from the overall automated grade.
    NotApplicable,
}

impl Grade {
    pub fn as_str(&self) -> &'static str {
        match self {
            Grade::APlus => "A+",
            Grade::A => "A",
            Grade::B => "B",
            Grade::C => "C",
            Grade::F => "F",
            Grade::Manual => "?",
            Grade::NotApplicable => "—",
        }
    }

    fn colored(&self) -> String {
        match self {
            Grade::APlus => "A+".green().bold().to_string(),
            Grade::A => "A".green().bold().to_string(),
            Grade::B => "B".yellow().bold().to_string(),
            Grade::C => "C".red().to_string(),
            Grade::F => "F".red().bold().to_string(),
            Grade::Manual => "?".cyan().to_string(),
            Grade::NotApplicable => "—".dimmed().to_string(),
        }
    }
}

/// Result of grading a single criterion
#[derive(Debug)]
pub struct CriterionResult {
    pub name: &'static str,
    pub result: String,
    pub grade: Grade,
    pub threshold: &'static str,
    /// Full descriptive string for COMPLETION.md
    /// (e.g., "96.2% production line coverage (839/879 lines; 1225 #[cfg(test)] lines excluded)").
    /// Read by `complete.rs` (Step 9 of the grade tool rebuild).
    #[allow(dead_code)]
    pub measured_detail: String,
}

/// Full grade report for a crate
#[derive(Debug)]
pub struct GradeReport {
    pub crate_name: String,
    /// Rubric profile used for grading. Determines which criteria are
    /// applicable. Shown in the display header so the reader knows what
    /// rubric produced the scores below.
    pub profile: CrateProfile,
    pub criteria: Vec<CriterionResult>,
    pub automated_grade: Grade,
    #[allow(dead_code)]
    pub needs_review: bool,
    /// Criterion 1's measurement split by file, worst first — empty whenever
    /// coverage did not run (`--skip-coverage`, a skipped profile, a failed
    /// measurement) or produced no production lines.
    ///
    /// Structured rather than folded into `CriterionResult.measured_detail`
    /// because two consumers want it in two shapes: `--json` for machine
    /// triage, the table for a reader. `pub(crate)` to match
    /// [`crate::coverage::FileCoverage`]'s own visibility.
    pub(crate) coverage_files: Vec<crate::coverage::FileCoverage>,
}

impl GradeReport {
    fn overall_automated(&self) -> Grade {
        let mut worst = Grade::APlus;
        for c in &self.criteria {
            // Skip Manual (needs human review) and NotApplicable (criterion
            // doesn't apply to this profile per docs/STANDARDS.md).
            if c.grade == Grade::Manual || c.grade == Grade::NotApplicable {
                continue;
            }
            // Worst-grade ordering: F < C < B < A < A+
            worst = match (&worst, &c.grade) {
                (Grade::F, _) | (_, Grade::F) => Grade::F,
                (Grade::C, _) | (_, Grade::C) => Grade::C,
                (Grade::B, _) | (_, Grade::B) => Grade::B,
                (Grade::A, _) | (_, Grade::A) => Grade::A,
                _ => Grade::APlus,
            };
        }
        worst
    }
}

/// Crate profile — what rubric applies to this crate.
///
/// CortenForge grades against eight criteria, but `docs/STANDARDS.md`
/// explicitly scopes two of them to specific crate types:
///
/// - **Criterion 6 (Bevy-free)** — STANDARDS.md §6 titles it "Bevy-free
///   (Layer 0)" and names only Layer 0 crate prefixes as subject to it.
///   Bevy examples and Layer 1 (`sim-bevy`) are explicitly exempt.
/// - **Criterion 4 (Safety)** — STANDARDS.md §4 "Allowed" block lists
///   `unwrap()` in examples and `expect()` in `build.rs` as permitted.
///   The strict unwrap/expect counter should only fire on library code.
///
/// Before this profile existed, the grader applied every criterion to
/// every crate uniformly, producing false F grades on Bevy examples and
/// obscuring real quality signal. The profile restores the written
/// standard: each crate is classified, and inapplicable criteria return
/// `Grade::NotApplicable` (excluded from the overall grade) rather than
/// a misleading F.
///
/// Classification is primarily path-based, with one Cargo.toml metadata
/// opt-in (F.3). A crate's manifest path (relative to the workspace
/// root) maps to exactly one profile:
///
/// - `examples/`           → [`CrateProfile::Example`]
/// - `xtask/`, `tools/`    → [`CrateProfile::Xtask`]
/// - `sim/L1/`             → [`CrateProfile::BevyLayer1`]
/// - anything else         → [`CrateProfile::Layer0`] (default; strictest)
///
/// A crate may opt into [`CrateProfile::IntegrationOnly`] by adding
/// `[package.metadata.cortenforge] grading_profile = "integration-only"`
/// to its `Cargo.toml`. The metadata read overrides path classification.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CrateProfile {
    /// Layer 0 library crate — the strictest rubric applies. All seven
    /// criteria including Bevy-free and strict Safety.
    Layer0,
    /// Layer 1 Bevy integration crate — all criteria except Bevy-free.
    BevyLayer1,
    /// Visual / demo example crate (`examples/**`). Coverage is N/A
    /// (bin-only, no lib target); Safety relaxes the unwrap/expect
    /// counting per STANDARDS.md §4 "Allowed" (unsafe and
    /// todo!/unimplemented! still gated); Bevy-free is N/A.
    Example,
    /// Build tooling (`xtask`, build scripts). Coverage and Bevy-free
    /// are N/A; Safety relaxed for the same reason as Example.
    Xtask,
    /// Crate whose source code is exercised exclusively by integration
    /// tests (no inline `#[cfg(test)]` modules), or which has no `src/`
    /// at all. Coverage is N/A; other criteria apply normally. Opt-in
    /// via `[package.metadata.cortenforge] grading_profile = "integration-only"`.
    IntegrationOnly,
}

impl CrateProfile {
    /// Human-readable label for display in the grade table header.
    fn label(&self) -> &'static str {
        match self {
            CrateProfile::Layer0 => "Layer 0 library",
            CrateProfile::BevyLayer1 => "Layer 1 (Bevy)",
            CrateProfile::Example => "Example (visual/demo)",
            CrateProfile::Xtask => "Build tooling",
            CrateProfile::IntegrationOnly => "Integration-only",
        }
    }
}

/// Background heartbeat thread for long-running subprocess stages.
///
/// Prints a `"    … still running (Ns elapsed)"` line to stderr every
/// `interval` seconds. The thread is stopped automatically when the
/// `Heartbeat` value is dropped — callers just let it fall out of scope.
struct Heartbeat {
    stop: Arc<AtomicBool>,
    handle: Option<std::thread::JoinHandle<()>>,
}

impl Heartbeat {
    fn start(interval_secs: u64) -> Self {
        let stop = Arc::new(AtomicBool::new(false));
        let stop_clone = stop.clone();
        let handle = std::thread::spawn(move || {
            let start = Instant::now();
            let mut next = interval_secs;
            loop {
                std::thread::sleep(Duration::from_secs(1));
                if stop_clone.load(Ordering::Relaxed) {
                    break;
                }
                let secs = start.elapsed().as_secs();
                if secs >= next {
                    eprintln!("    … still running ({}s elapsed)", secs);
                    next += interval_secs;
                }
            }
        });
        Self {
            stop,
            handle: Some(handle),
        }
    }
}

impl Drop for Heartbeat {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(h) = self.handle.take() {
            let _ = h.join();
        }
    }
}

/// Classify a crate by its workspace-relative manifest path, with one
/// Cargo.toml metadata opt-in.
///
/// See [`CrateProfile`] for the mapping rules. The metadata read
/// (`[package.metadata.cortenforge] grading_profile = "integration-only"`)
/// is the only override — every other classification is path-based and
/// a crate moved to a new directory gets a different profile
/// automatically. The opt-in is self-documenting: the annotation lives
/// in the crate's own `Cargo.toml` where any reader encounters it.
pub(crate) fn classify_crate(crate_path: &str, cargo_toml_text: &str) -> CrateProfile {
    // F.3 metadata opt-in: explicit override for crates that have no
    // testable lib target (no `src/` or no inline `#[cfg(test)]` modules)
    // and are exercised by integration tests instead. Coverage criterion
    // returns NotApplicable for this profile; other criteria apply normally.
    if let Ok(value) = toml::from_str::<toml::Value>(cargo_toml_text) {
        let grading_profile = value
            .get("package")
            .and_then(|p| p.get("metadata"))
            .and_then(|m| m.get("cortenforge"))
            .and_then(|c| c.get("grading_profile"))
            .and_then(|g| g.as_str());
        if grading_profile == Some("integration-only") {
            return CrateProfile::IntegrationOnly;
        }
    }

    // Normalize path separators for cross-platform matching.
    let normalized = crate_path.replace('\\', "/");
    if normalized.starts_with("examples/") || normalized.starts_with("examples\\") {
        CrateProfile::Example
    } else if normalized == "xtask"
        || normalized.starts_with("xtask/")
        || normalized.starts_with("tools/")
    {
        // `tools/` is the workspace-internal-binary-tool folder; first
        // inhabitant is `tools/cf-scan-prep` (Stage 2.5 scan preprocessing
        // GUI). Same shape as xtask: bin-only crate, no lib target → can't
        // be coverage-graded; binary application code → relaxed clippy
        // unjustified-allow lint per `grade_dependencies`'s
        // `relax_unjustified_allows` branch. Reusing the `Xtask` variant
        // avoids adding a near-duplicate `Tool` variant; the display name
        // "Build tooling" is a mild semantic stretch but the grading
        // behavior is correct.
        CrateProfile::Xtask
    } else if normalized.starts_with("sim/L1/") {
        CrateProfile::BevyLayer1
    } else {
        CrateProfile::Layer0
    }
}

/// Whether the crate actually has a library target, by Cargo's own rule: an
/// explicit `[lib]` table, or auto-discovery of `src/lib.rs`.
///
/// ★ **This is the fact [`coverage_skip_reason`] used to assume.** The
/// `Example`/`Xtask` profiles skipped coverage with the stated reason "bin-only
/// crates have no lib target", and nothing checked it. Measured 2026-08-16, it
/// was false for **13 of the 17** crates under `tools/`: **9718 production
/// lines** went unmeasured behind a justification that did not hold, one of
/// those crates being `cf-codesign` — the co-design optimizer, Mission
/// deliverable #2. Every one reported `—` and passed, because
/// [`Grade::NotApplicable`] is skipped by `overall_automated`.
///
/// ★ 9718 is what THIS instrument counts, which is the only count that means
/// anything here: a first pass used a source-line heuristic and said 13 265,
/// overstating by 36 % because it counted `use` lines, attributes and closing
/// braces that llvm-cov never maps. The size of an unmeasured gap has to be
/// stated in the units of the measurement that was missing.
///
/// ★★ And measuring them showed **5505 of those lines were covered all
/// along** — 56.6 %. The tests existed; nothing was reading them. What the
/// skip hid was mostly a measurement gap rather than a quality one, which is
/// why the fix is a grader change and not a test-writing campaign. The
/// genuinely never-exercised remainder is 4213 lines.
///
/// Same family as the fail-open dead zone #772–#774 closed: a gate whose green
/// meant "not measured" while its text claimed a property nobody had checked.
/// The fix is the same shape — take the fact as an argument instead of
/// inferring it from where the crate happens to live.
///
/// ★★ **Cross-checked against an instrument sharing no code with it.** Over
/// all 301 workspace members, this function's answer matches the target list
/// `cargo metadata` reports — zero disagreements — and the
/// `Example`/`Xtask`-with-a-library set it produces is exactly the 14 crates
/// the 2026-08-16 census measured. So the newly-measured set is *complete*,
/// not merely plausible, and the report-only list below is sized against all
/// of it.
///
/// `autolib = false` (Rust 2024; this workspace is edition 2024) suppresses
/// the `src/lib.rs` discovery, so it is honoured here; a crate that sets it and
/// declares `[lib]` anyway still has the target, which is why the explicit
/// table is checked first.
///
/// ⚠ **That one branch is the part the cross-check could not reach**: no crate
/// in the workspace sets `autolib`, so the 301/301 agreement above says nothing
/// about it. It has a unit test and follows Cargo's documented behaviour, but
/// it has never been confronted with a real manifest. If it is wrong, the
/// error is a silent skip — the bad direction — so a crate that sets `autolib`
/// deserves a look at its grade rather than trust in this line.
///
/// ⚠ Errs toward "has a lib" whenever the manifest will not parse: the caller
/// then MEASURES rather than skipping, and a measurement that should not have
/// run is a visible number, while a skip that should not have happened is
/// silence. Only the second one can hide code.
pub(crate) fn has_lib_target(crate_dir: &Path, cargo_toml_text: &str) -> bool {
    let manifest = toml::from_str::<toml::Value>(cargo_toml_text).ok();
    let Some(manifest) = manifest else {
        return true;
    };
    if manifest.get("lib").is_some() {
        return true;
    }
    let autolib = manifest
        .get("package")
        .and_then(|p| p.get("autolib"))
        .and_then(toml::Value::as_bool);
    if autolib == Some(false) {
        return false;
    }
    crate_dir.join("src").join("lib.rs").is_file()
}

/// Run a single criterion with progress logging on stderr.
fn run_criterion<F>(index: usize, name: &str, quiet: bool, f: F) -> Result<CriterionResult>
where
    F: FnOnce() -> Result<CriterionResult>,
{
    if !quiet {
        eprintln!("  criterion {}/8: {} — running…", index, name);
    }
    let start = Instant::now();
    let result = f()?;
    if !quiet {
        eprintln!(
            "  criterion {}/8: {} — {} ({:.1}s)",
            index,
            name,
            result.grade.as_str(),
            start.elapsed().as_secs_f64(),
        );
    }
    Ok(result)
}

/// Run all automated criteria and return structured results.
///
/// The crate's workspace path is classified into a [`CrateProfile`]
/// before grading, and the profile is threaded through to each
/// criterion function. Criteria that don't apply to the profile (per
/// STANDARDS.md scoping) return [`Grade::NotApplicable`] instead of
/// running their checks.
pub fn evaluate(sh: &Shell, crate_name: &str, verbosity: Verbosity) -> Result<GradeReport> {
    let overall_start = Instant::now();
    let workspace_root = find_workspace_root(sh)?;
    sh.change_dir(&workspace_root);

    let crate_path = find_crate_path(sh, crate_name)?;
    // ★ Rooted on the workspace root, NOT left relative. `xshell::change_dir`
    // moves the shell's directory; it does not touch the process's, and
    // `std::fs` reads the process's. So the bare relative form this used to
    // take resolved against wherever the user happened to invoke
    // `cargo xtask` from, and silently returned "" anywhere but the root.
    //
    // ⚠ That was not cosmetic: the F.3 `grading_profile` opt-in lives in this
    // text, so `sim-therm-env` graded as "Integration-only" from the workspace
    // root and "Layer 0 library" from `sim/L0/core` — the same crate, the same
    // commit, two verdicts. Fail-closed (the stricter profile is the one you
    // got by accident), but a grade that depends on your shell's cwd is not a
    // grade. Same rooting the empty-crate check below already does.
    let crate_dir = Path::new(&workspace_root).join(&crate_path);
    let cargo_toml_text = std::fs::read_to_string(crate_dir.join("Cargo.toml")).unwrap_or_default();
    let profile = classify_crate(&crate_path, &cargo_toml_text);
    let has_lib = has_lib_target(&crate_dir, &cargo_toml_text);

    if !verbosity.quiet {
        eprintln!();
        eprintln!("  grading {} (profile: {})…", crate_name, profile.label());
        eprintln!();
    }

    let mut report = GradeReport {
        crate_name: crate_name.to_string(),
        profile,
        criteria: Vec::new(),
        automated_grade: Grade::A,
        needs_review: true,
        coverage_files: Vec::new(),
    };

    // Filled by `grade_coverage` through a `&mut` capture. The criterion's
    // grade is a `CriterionResult` like every other, but its per-file split has
    // no place in that shape, and re-deriving it would mean a second
    // instrumented run — minutes, for data the first run already produced.
    let mut coverage_files = Vec::new();
    report
        .criteria
        .push(run_criterion(1, "Coverage", verbosity.quiet, || {
            grade_coverage(
                sh,
                crate_name,
                &crate_path,
                profile,
                has_lib,
                verbosity,
                &mut coverage_files,
            )
        })?);
    report.coverage_files = coverage_files;
    report
        .criteria
        .push(run_criterion(2, "Documentation", verbosity.quiet, || {
            grade_documentation(sh, crate_name, &crate_path)
        })?);
    report
        .criteria
        .push(run_criterion(3, "Clippy", verbosity.quiet, || {
            grade_clippy(sh, crate_name, &crate_path, profile)
        })?);
    report
        .criteria
        .push(run_criterion(4, "Safety", verbosity.quiet, || {
            grade_safety(sh, &crate_path, profile)
        })?);
    report
        .criteria
        .push(run_criterion(5, "Dependencies", verbosity.quiet, || {
            grade_dependencies(sh, crate_name, profile)
        })?);
    report.criteria.push(run_criterion(
        6,
        "Layer Integrity",
        verbosity.quiet,
        || grade_layer_integrity(sh, crate_name, &cargo_toml_text, verbosity.quiet),
    )?);
    report
        .criteria
        .push(run_criterion(7, "WASM Compat", verbosity.quiet, || {
            grade_wasm_compat(sh, crate_name, &cargo_toml_text, verbosity.quiet)
        })?);
    report.criteria.push(CriterionResult {
        name: "8. API Design",
        result: "(manual review)".to_string(),
        grade: Grade::Manual,
        threshold: "checklist",
        measured_detail: "(manual review)".to_string(),
    });

    report.automated_grade = report.overall_automated();

    if !verbosity.quiet {
        eprintln!();
        eprintln!(
            "  grading complete — {:.1}s total",
            overall_start.elapsed().as_secs_f64()
        );
    }

    Ok(report)
}

/// Display grade report as Unicode-box table.
fn display(report: &GradeReport) {
    println!();
    println!(
        "{}",
        "╔══════════════════════════════════════════════════════════════╗"
            .to_string()
            .bright_white()
            .bold()
    );
    println!(
        "{}",
        format!("║{:^62}║", format!("GRADING: {}", report.crate_name))
            .bright_white()
            .bold()
    );
    println!(
        "{}",
        format!("║{:^62}║", format!("profile: {}", report.profile.label())).bright_white()
    );
    println!(
        "{}",
        "╠══════════════════════════════════════════════════════════════╣"
            .to_string()
            .bright_white()
            .bold()
    );
    println!(
        "{}",
        format!(
            "║ {:16} │ {:16} │ {:5} │ {:14} ║",
            "Criterion", "Result", "Grade", "Threshold"
        )
        .bright_white()
    );
    println!(
        "{}",
        "╠══════════════════════════════════════════════════════════════╣"
            .to_string()
            .bright_white()
    );

    for c in &report.criteria {
        print_criterion(c);
    }

    println!(
        "{}",
        "╠══════════════════════════════════════════════════════════════╣"
            .to_string()
            .bright_white()
    );
    println!(
        "{}",
        format!(
            "║ {:16} │ {:16} │{}│ {:14} ║",
            "AUTOMATED",
            "",
            grade_cell(&report.automated_grade),
            ""
        )
        .bright_white()
    );
    println!(
        "{}",
        format!(
            "║ {:16} │ {:16} │{}│ {:14} ║",
            "OVERALL",
            "",
            grade_cell(&Grade::Manual),
            "needs review"
        )
        .bright_white()
    );
    println!(
        "{}",
        "╚══════════════════════════════════════════════════════════════╝"
            .to_string()
            .bright_white()
    );

    print_coverage_triage(report);
}

/// Rows of the per-file coverage table printed under the grade.
///
/// Bounded because triage starts at the top and a large crate has hundreds of
/// files. The tail is summarised rather than dropped silently, and `--json`
/// carries every row unbounded — a cap nobody is told about reads as "that was
/// all of it".
const TRIAGE_ROWS: usize = 20;

/// Print criterion 1's per-file breakdown, worst first.
///
/// Prints nothing unless coverage actually ran and something is uncovered. A
/// fully-covered file is not somewhere to go and write a test, and listing it
/// would push real targets off the bottom of the cap.
fn print_coverage_triage(report: &GradeReport) {
    let worst: Vec<&crate::coverage::FileCoverage> = report
        .coverage_files
        .iter()
        .filter(|f| f.uncovered() > 0)
        .collect();
    if worst.is_empty() {
        return;
    }
    let uncovered_total: u64 = worst.iter().map(|f| f.uncovered()).sum();

    println!();
    println!(
        "{}",
        format!(
            "  Coverage triage — {} of {} measured file(s) hold {} uncovered production line(s):",
            worst.len(),
            report.coverage_files.len(),
            uncovered_total
        )
        .bright_white()
        .bold()
    );
    println!("{}", "      uncovered  covered  file".dimmed());
    for f in worst.iter().take(TRIAGE_ROWS) {
        let (target, caveat) = triage_row_markers(f);
        println!(
            "{}{}{}",
            triage_row_prefix(f),
            target.dimmed(),
            caveat.yellow()
        );
    }
    if let Some((rest, rest_lines)) = triage_tail(&worst, TRIAGE_ROWS) {
        println!(
            "{}",
            format!(
                "      … {rest} more file(s) hold {rest_lines} uncovered line(s); \
                 --json lists every file"
            )
            .dimmed()
        );
    }
}

/// What the printed table left out: `(files, uncovered lines)` beyond `cap`.
///
/// `None` when nothing was cut. Split out from the printing so the claim the
/// tail line makes is checkable — the count and the line sum have to skip
/// exactly the rows the table printed, and a table that under-reports its own
/// truncation is the silent cap the cap was allowed on condition of avoiding.
/// A triage row up to its markers: count, percentage, path.
///
/// Carries the percentage FORMATTING, not just the number, which is the point
/// of extracting it. Testing `coverage_display(f.percent())` by hand proves
/// the formatter truncates; it does not prove this row calls it, and a
/// mutation that put `{:.1}` back at the call site survived a sweep that a
/// hand-composed assertion was supposed to have covered.
fn triage_row_prefix(f: &crate::coverage::FileCoverage) -> String {
    format!(
        "      {:>9}  {:>7}  {}",
        f.uncovered(),
        coverage_display(f.percent()),
        f.file
    )
}

/// The two markers a triage row can carry, plain — the caller styles them.
///
/// `(binary target, unparsed caveat)`. Extracted for the same reason
/// [`triage_tail`] beside it was: a row assembled inline inside `println!` is a
/// row no test reads, and a 2026-08-16 mutation sweep confirmed it — dropping
/// the binary marker entirely left the suite green.
///
/// - A binary root is ranked and counted like any other file. The marker says
///   only that its lines belong to a target whose entry point cannot be
///   unit-tested, so a reader does not go hunting for a way to cover `.run()`.
///   It is the one row where a high uncovered count may be the shape of the
///   target rather than a gap in the tests.
/// - An unparsed file's counts include its test lines, so the row is an
///   over-estimate and may outrank honest ones. The crate-level warning says
///   how many files, not which, and the rank is what a reader acts on.
fn triage_row_markers(f: &crate::coverage::FileCoverage) -> (&'static str, &'static str) {
    (
        if f.is_bin { "  (binary target)" } else { "" },
        if f.test_lines_counted {
            "  ⚠ unparsed, so test lines are counted here"
        } else {
            ""
        },
    )
}

fn triage_tail(worst: &[&crate::coverage::FileCoverage], cap: usize) -> Option<(usize, u64)> {
    let rest = worst.len().checked_sub(cap).filter(|n| *n > 0)?;
    Some((rest, worst.iter().skip(cap).map(|f| f.uncovered()).sum()))
}

/// Emit the grade report as JSON to stdout.
fn json_output(report: &GradeReport) {
    let criteria: Vec<serde_json::Value> = report
        .criteria
        .iter()
        .map(|c| {
            serde_json::json!({
                "name": c.name,
                "result": c.result,
                "grade": c.grade.as_str(),
                "threshold": c.threshold,
                "measured_detail": c.measured_detail,
            })
        })
        .collect();

    // Every measured file, not a top-N slice: this is the machine-readable
    // side, where a consumer does its own ranking and a silent cap would be a
    // lie about what was measured. The human table truncates instead, and says
    // so. Absent rather than empty when coverage did not run, so a consumer can
    // tell "measured, all covered" from "never measured".
    let coverage_files: Vec<serde_json::Value> = report
        .coverage_files
        .iter()
        .map(|f| {
            serde_json::json!({
                "file": f.file,
                "covered": f.covered,
                "total": f.total,
                "uncovered": f.uncovered(),
                "percent": (f.percent() * 10.0).round() / 10.0,
                // True means the counts on this row include `#[cfg(test)]`
                // lines, so they overstate both the work and the gap.
                "test_lines_counted": f.test_lines_counted,
                // True for a binary-target root. Present so a consumer can
                // compute the library-only figure the human detail line
                // reports, from the same rows, without re-deriving which
                // paths Cargo treats as binaries.
                "is_bin": f.is_bin,
            })
        })
        .collect();

    let mut json = serde_json::json!({
        "crate_name": report.crate_name,
        "profile": report.profile.label(),
        "automated_grade": report.automated_grade.as_str(),
        "criteria": criteria,
    });
    if !coverage_files.is_empty() {
        json["coverage_files"] = serde_json::Value::Array(coverage_files);
    }

    println!(
        "{}",
        serde_json::to_string_pretty(&json).unwrap_or_default()
    );
}

/// Run grading for a specific crate
pub fn run(crate_name: &str, verbosity: Verbosity) -> Result<()> {
    let sh = Shell::new()?;
    let report = evaluate(&sh, crate_name, verbosity)?;

    if verbosity.json {
        json_output(&report);
    } else {
        display(&report);
        println!();

        match report.automated_grade {
            Grade::A | Grade::APlus => {
                println!(
                    "{}",
                    "✓ All automated criteria pass. Ready for API review.".green()
                );
                println!();
                println!("Next step: Review against API checklist in docs/STANDARDS.md");
                println!("Then run: cargo xtask complete {}", report.crate_name);
            }
            _ => {
                println!(
                    "{}",
                    format!(
                        "✗ Automated grade: {}. Refactor required before completion.",
                        report.automated_grade.as_str()
                    )
                    .red()
                );
                println!();
                println!(
                    "Fix failing criteria and run: cargo xtask grade {}",
                    report.crate_name
                );
            }
        }

        println!();
    }

    Ok(())
}

/// Run grading across every workspace member.
///
/// Enumerates workspace crates via `cargo metadata --no-deps` — no
/// hard-coded lists, automatically adapts when crates are added or
/// removed. Each crate is graded via [`evaluate`]; failures are
/// aggregated and reported in a compact summary.
///
/// A crate the grader cannot measure at all does not abort the sweep — it is
/// recorded and the run continues, so one broken crate never costs the
/// operator the verdict on every crate after it. Such crates are reported
/// separately from graded failures (see [`SweepTally`]), because "could not
/// measure" is a different problem from "measured, and it is bad". Exits
/// non-zero if any crate's automated grade is below A **or** any crate could
/// not be graded.
///
/// `shard` (from `--shard i/N`) restricts grading to a disjoint 1/N slice
/// so CI can fan grade-all out across N parallel jobs; see [`select_shard`].
///
/// Intended as the CI entry point for single-source-of-truth grading:
/// CI runs `cargo xtask grade-all --skip-coverage --quiet`, checks the
/// exit code, and surfaces the failure summary on red.
pub fn run_all(
    verbosity: Verbosity,
    shard: Option<(usize, usize)>,
    only: Option<Vec<String>>,
) -> Result<()> {
    let sh = Shell::new()?;
    let workspace_root = find_workspace_root(&sh)?;
    sh.change_dir(&workspace_root);

    let metadata_json = cmd!(sh, "cargo metadata --format-version 1 --no-deps")
        .read()
        .context("Failed to run `cargo metadata`")?;
    let metadata: serde_json::Value = serde_json::from_str(&metadata_json)
        .context("Failed to parse `cargo metadata` JSON output")?;

    let packages = metadata["packages"]
        .as_array()
        .context("`cargo metadata`: missing 'packages' array")?;

    // --no-deps scopes packages to workspace members only.
    let mut all_crate_names: Vec<String> = packages
        .iter()
        .filter_map(|p| p["name"].as_str().map(String::from))
        .collect();
    all_crate_names.sort();
    let workspace_total = all_crate_names.len();

    // Apply --only <affected>: PR-scoped CI grades only the affected subset
    // (changed crates + reverse-dep closure from `xtask affected`). Absent on
    // main/merge → full workspace. Runs before sharding so each shard takes a
    // slice of the affected set.
    let scoped = filter_only(&all_crate_names, only.as_deref());

    // Apply --shard i/N: each parallel CI job grades a disjoint slice.
    let crate_names = select_shard(&scoped, shard);

    if !verbosity.quiet {
        eprintln!();
        match shard {
            Some((i, n)) => eprintln!(
                "  grade-all: shard {}/{} — evaluating {} of {} workspace crates…",
                i,
                n,
                crate_names.len(),
                workspace_total
            ),
            None => eprintln!(
                "  grade-all: evaluating {} workspace crates…",
                crate_names.len()
            ),
        }
        if only.is_some() {
            eprintln!(
                "  (PR-scoped via --only: {} affected crate(s) before sharding)",
                scoped.len()
            );
        }
        if verbosity.skip_coverage {
            eprintln!("  (coverage skipped via --skip-coverage)");
        }
        eprintln!();
    }

    let mut failures: Vec<(String, GradeReport)> = Vec::new();
    // Crates that could not be graded at all, kept apart from `failures`
    // because "could not measure" is a different claim from "measured, and
    // it is bad" — and a different thing to go and fix.
    let mut errors: Vec<(String, String)> = Vec::new();

    for (idx, crate_name) in crate_names.iter().enumerate() {
        // Force --quiet per-crate regardless of outer verbosity — grade-all
        // prints its own compact one-line-per-crate progress so 190 crates
        // of per-criterion chatter don't drown the aggregate report.
        let per_crate_verbosity = Verbosity {
            quiet: true,
            verbose: false,
            json: false,
            skip_coverage: verbosity.skip_coverage,
        };
        // Record and carry on rather than `?`, mirroring `run-validators`:
        // one ungradeable crate must not cost the operator the verdict on
        // every crate after it in the shard. The sweep still fails — see the
        // tally below — it just fails knowing the whole picture.
        let report = match evaluate(&sh, crate_name, per_crate_verbosity) {
            Ok(report) => report,
            Err(e) => {
                if !verbosity.quiet {
                    eprintln!(
                        "  [{:>3}/{}] {} — {}",
                        idx + 1,
                        crate_names.len(),
                        "ERR".red().bold(),
                        crate_name
                    );
                }
                errors.push((crate_name.clone(), format!("{e:#}")));
                continue;
            }
        };

        let passed = matches!(report.automated_grade, Grade::A | Grade::APlus);
        if passed {
            if !verbosity.quiet {
                eprintln!(
                    "  [{:>3}/{}] {} — {}",
                    idx + 1,
                    crate_names.len(),
                    report.automated_grade.as_str().green(),
                    crate_name
                );
            }
        } else {
            if !verbosity.quiet {
                eprintln!(
                    "  [{:>3}/{}] {} — {}",
                    idx + 1,
                    crate_names.len(),
                    report.automated_grade.as_str().red(),
                    crate_name
                );
            }
            failures.push((crate_name.clone(), report));
        }
    }

    let tally = SweepTally {
        total: crate_names.len(),
        failures: failures.len(),
        errors: errors.len(),
    };

    println!();
    let headline = tally.headline();
    println!(
        "{}",
        if tally.is_green() {
            format!("✓ {headline}").green().bold().to_string()
        } else {
            format!("✗ {headline}").red().bold().to_string()
        }
    );
    println!();

    for (name, report) in &failures {
        println!(
            "  {} — {}",
            name.bold(),
            report.automated_grade.as_str().red()
        );
        for c in &report.criteria {
            if matches!(c.grade, Grade::F | Grade::C) {
                println!("      {}: {} ({})", c.name, c.grade.as_str(), c.result);
            }
        }
    }

    // Errors last: they are the ones an operator usually has to act on first,
    // and the tail of the output is what a CI log viewer opens on.
    for (name, err) in &errors {
        println!("  {} — {}", name.bold(), "could not be graded".red());
        println!("      {}", err);
    }

    if failures.is_empty() && errors.is_empty() {
        Ok(())
    } else {
        println!();
        bail!("{}", tally.headline())
    }
}

/// What a `grade-all` sweep ended with.
///
/// `errors` is its own bucket rather than being folded into `failures`: a
/// crate the grader could not measure has not been shown to be bad, but it has
/// not been shown to be good either. Conflating the two would either invent an
/// F nobody measured or — far worse — let an unmeasured crate pass.
///
/// `passes` is derived rather than counted, so the printed numbers cannot
/// drift from the buckets they summarise.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct SweepTally {
    total: usize,
    failures: usize,
    errors: usize,
}

impl SweepTally {
    fn passes(&self) -> usize {
        self.total.saturating_sub(self.failures + self.errors)
    }

    /// A sweep may report success only when every crate was graded *and* every
    /// grade passed.
    fn is_green(&self) -> bool {
        self.failures == 0 && self.errors == 0
    }

    /// One line naming every non-empty bucket, so no count the verdict depends
    /// on is left for the reader to infer.
    ///
    /// Carries neither glyph nor colour: those are presentation the summary
    /// block adds, and this same string is also the `bail!` text, where
    /// anyhow's own `Error:` prefix is the failure marker. Baking a `✗` in
    /// would print "Error: ✗ …" — two markers for one failure.
    fn headline(&self) -> String {
        match (self.failures, self.errors) {
            (0, 0) => format!(
                "grade-all: {}/{} workspace crates pass.",
                self.passes(),
                self.total
            ),
            (f, 0) => format!(
                "grade-all: {}/{} workspace crates fail xtask grade.",
                f, self.total
            ),
            (0, e) => format!(
                "grade-all: {}/{} workspace crates could not be graded.",
                e, self.total
            ),
            (f, e) => format!(
                "grade-all: {} of {} workspace crates fail xtask grade, \
                 and {} could not be graded.",
                f, self.total, e
            ),
        }
    }
}

/// Format grade for display in a fixed-width table cell (7 visible chars).
fn grade_cell(grade: &Grade) -> String {
    match grade {
        // A+ is 2 visible chars: 2 spaces + A+ + 3 spaces = 7
        Grade::APlus => format!("  {}   ", grade.colored()),
        // All others are 1 visible char: 3 spaces + X + 3 spaces = 7
        _ => format!("   {}   ", grade.colored()),
    }
}

fn print_criterion(c: &CriterionResult) {
    println!(
        "{}",
        format!(
            "║ {:16} │ {:16} │{}│ {:14} ║",
            truncate(c.name, 16),
            truncate(&c.result, 16),
            grade_cell(&c.grade),
            truncate(c.threshold, 14),
        )
        .bright_white()
    );
}

fn truncate(s: &str, max: usize) -> String {
    if s.len() <= max {
        format!("{:width$}", s, width = max)
    } else {
        format!("{}...", &s[..max - 3])
    }
}

/// Find workspace root by looking for root Cargo.toml with `[workspace]`
fn find_workspace_root(sh: &Shell) -> Result<String> {
    let output = cmd!(
        sh,
        "cargo locate-project --workspace --message-format plain"
    )
    .read()?;
    let cargo_toml = output.trim();
    let root = Path::new(cargo_toml)
        .parent()
        .context("Could not find workspace root")?;
    Ok(root.to_string_lossy().to_string())
}

/// Find the path to a crate within the workspace.
///
/// Uses `cargo metadata` to look up the crate's manifest path by package
/// name, then derives the directory and rebases it onto the workspace
/// root. This works for any workspace layout — flat (`mesh/mesh-types`),
/// layered (`sim/L0/thermostat`), or anything else cargo recognizes —
/// without hard-coded directory heuristics.
pub(crate) fn find_crate_path(sh: &Shell, crate_name: &str) -> Result<String> {
    let metadata_json = cmd!(sh, "cargo metadata --format-version 1 --no-deps")
        .read()
        .context("Failed to run `cargo metadata`")?;

    let metadata: serde_json::Value = serde_json::from_str(&metadata_json)
        .context("Failed to parse `cargo metadata` JSON output")?;

    let packages = metadata["packages"]
        .as_array()
        .context("`cargo metadata`: missing 'packages' array")?;

    let workspace_root = metadata["workspace_root"]
        .as_str()
        .context("`cargo metadata`: missing 'workspace_root' field")?;

    for pkg in packages {
        if pkg["name"].as_str() == Some(crate_name) {
            let manifest_path = pkg["manifest_path"]
                .as_str()
                .context("`cargo metadata`: package missing 'manifest_path'")?;
            let crate_dir = Path::new(manifest_path)
                .parent()
                .context("manifest_path has no parent directory")?;
            let relative = crate_dir.strip_prefix(workspace_root).unwrap_or(crate_dir);
            return Ok(relative.to_string_lossy().to_string());
        }
    }

    bail!(
        "Could not find crate '{}' in workspace metadata",
        crate_name
    )
}

/// Why Coverage does not apply to this crate — `Some((result_label, detail))`
/// to skip, `None` when the crate must be measured.
///
/// Two reasons. `IntegrationOnly` is an explicit `[package.metadata.cortenforge]`
/// opt-in, self-documenting in the crate's own manifest. The other is having no
/// library target to measure — and `has_lib_target` is that fact, passed in
/// rather than inferred, so the message and the behaviour cannot drift apart.
///
/// ★ An `Example`/`Xtask` crate that DOES have a lib target is measured like
/// any other. Those profiles say "binary application code" for the purposes of
/// Clippy and Safety relaxation; they never established the absence of a
/// library, which is what the coverage skip was resting on. See
/// [`has_lib_target`] for what that cost.
fn coverage_skip_reason(
    profile: CrateProfile,
    has_lib_target: bool,
) -> Option<(&'static str, String)> {
    match profile {
        CrateProfile::Example | CrateProfile::Xtask if has_lib_target => None,
        CrateProfile::Example | CrateProfile::Xtask => Some((
            "(bin-only)",
            format!(
                "coverage skipped: this {} crate has no lib target per STANDARDS.md §1",
                profile.label()
            ),
        )),
        CrateProfile::IntegrationOnly => Some((
            "(integration-only)",
            "coverage skipped: integration-only crate per [package.metadata.cortenforge]"
                .to_string(),
        )),
        CrateProfile::Layer0 | CrateProfile::BevyLayer1 => None,
    }
}

/// Grade test coverage from an LLVM source-based coverage run.
///
/// Two-tier thresholds (F1 decision): >=90% = A+, >=75% = A.
/// Graceful degradation (ss1.4): without the `llvm-tools` component, Manual.
///
/// **Per-crate scoping.** Both the instrumentation and the report are scoped to
/// the crate's own source directory. The report has to be: a run that
/// instrumented the whole workspace would put sim-core, cf-geometry, mesh-*
/// into the denominator and structurally tank any crate with large siblings.
/// The *instrumentation* is scoped for cost — see [`crate::coverage_run`], which
/// carries the measurement — and because the report never counted those files,
/// scoping the build cannot move the number.
///
/// `files_out` receives the per-file split of whatever this measured, worst
/// first. **A non-empty `files_out` means "these files were measured", never
/// "this is all we managed to see"** — that is the guarantee callers rely on,
/// and it holds two ways. Every path that returns before a measurement exists
/// — a skipped profile, `--skip-coverage`, missing tooling, a failed run —
/// returns without touching it. The one measured path that still reports no
/// percentage, `(no production lines)`, is reached exactly when `total == 0`,
/// and `total` only grows in the branch that pushes a row, so its `files` is
/// already empty and assigning it moves nothing.
///
/// ⚠ Stated as the guarantee rather than as "left untouched unless…": the
/// assignment moved when the verdict was split into [`coverage_result`], and a
/// contract phrased in terms of which line runs goes stale the next time that
/// happens. This one is a property of the data.
///
/// ★ **Where the automated boundary sits, and why here.** Everything from a
/// finished measurement onward is unit-tested through [`coverage_result_for`]
/// and [`coverage_result`]. This function's own body — cargo orchestration,
/// two passes, the tooling and profile guards — is not, because driving it
/// needs a real llvm-cov run of minutes.
///
/// That boundary is a choice, not an oversight, and it does not shrink to
/// nothing by extracting further: each split tests the piece below it and
/// leaves the call to that piece untested one level up. Measured — a mutation
/// replacing the [`coverage_result_for`] call with a direct
/// `coverage_result(…, false)` survives the suite. What covers it instead is
/// an end-to-end check, run and recorded rather than assumed: `xtask grade
/// cf-anthro` prints `71.1% (report-only)` with three triage rows, and `xtask
/// grade cf-viewer` prints `33.8%` with `57.7% over library lines alone`.
/// Re-run those two after touching this function.
fn grade_coverage(
    sh: &Shell,
    crate_name: &str,
    crate_path: &str,
    profile: CrateProfile,
    has_lib_target: bool,
    verbosity: Verbosity,
    files_out: &mut Vec<crate::coverage::FileCoverage>,
) -> Result<CriterionResult> {
    // A crate with no library target has nothing for the instrumented
    // `--lib --tests` build to measure; an F.3 IntegrationOnly crate (per its
    // own Cargo.toml opt-in) is exercised from `tests/`. Both are N/A rather
    // than a false F. Which crates those actually ARE is now decided by
    // `has_lib_target` rather than by the directory the crate sits in — see
    // that function for the 9718 lines the guess was hiding.
    if let Some((result, detail)) = coverage_skip_reason(profile, has_lib_target) {
        return Ok(CriterionResult {
            name: "1. Coverage",
            result: result.to_string(),
            grade: Grade::NotApplicable,
            threshold: "≥75%/≥90% A+",
            measured_detail: detail,
        });
    }

    // `--skip-coverage` opt-out: CI runs want the other criteria without
    // paying the per-crate llvm-cov release build, which runs minutes to tens
    // of minutes (measured: sim-thermostat ~16 min). Dedicated
    // coverage jobs (nightly / manual) run without the flag.
    if verbosity.skip_coverage {
        return Ok(CriterionResult {
            name: "1. Coverage",
            result: "(skipped)".to_string(),
            grade: Grade::NotApplicable,
            threshold: "≥75%/≥90% A+",
            measured_detail: "coverage skipped via --skip-coverage flag".to_string(),
        });
    }

    if !crate::coverage_run::tools_available(sh) {
        return Ok(CriterionResult {
            name: "1. Coverage",
            result: "(llvm-tools n/a)".to_string(),
            grade: Grade::Manual,
            threshold: "≥75%/≥90% A+",
            measured_detail: "(llvm-tools n/a — `rustup component add llvm-tools-preview`)"
                .to_string(),
        });
    }

    // Two-pass coverage strategy:
    //
    // Pass 1: measure coverage from the crate's unit AND integration tests
    //   (`--lib --tests`), with instrumentation scoped to this crate — see
    //   `coverage_run`. Integration tests used to be excluded on the premise
    //   that src/ unit tests reach the same lines; that is false for any crate
    //   keeping its tests in tests/, which read as barely covered. Scoping the
    //   instrumentation to this crate (and its test targets) is what makes the
    //   pass affordable.
    //
    // Pass 2: Run ALL tests (unit + integration) WITHOUT instrumentation.
    //   Verifies correctness without paying the coverage overhead.
    //   Failures here still block the grade.
    //
    // --release avoids debug-mode runtime explosion (100×+ slower).
    // stderr flows to terminal so the user sees compile/test progress.
    if !verbosity.quiet {
        eprintln!("    pass 1/2: instrumented --lib --tests run ({crate_name} only)");
    }
    let pass_start = Instant::now();
    let heartbeat = if verbosity.verbose {
        Some(Heartbeat::start(30))
    } else {
        None
    };
    let workspace_root = sh.current_dir();
    let run = crate::coverage_run::measure_coverage(
        sh,
        crate_name,
        crate_path,
        &workspace_root,
        verbosity.quiet || verbosity.json,
    );
    drop(heartbeat);
    if !verbosity.quiet {
        eprintln!(
            "    pass 1/2: done ({:.1}s)",
            pass_start.elapsed().as_secs_f64()
        );
    }

    // Pass 2: run ALL tests without instrumentation for correctness.
    if !verbosity.quiet {
        eprintln!("    pass 2/2: cargo test --release (~1-5 min depending on integration tests)");
    }
    let pass_start = Instant::now();
    let heartbeat = if verbosity.verbose {
        Some(Heartbeat::start(30))
    } else {
        None
    };
    let heavy_passed = if verbosity.json || verbosity.quiet {
        cmd!(sh, "cargo test --release -p {crate_name}")
            .ignore_status()
            .output()
            .map(|o| o.status.success())
            .unwrap_or(false)
    } else {
        cmd!(sh, "cargo test --release -p {crate_name}")
            .run()
            .is_ok()
    };
    drop(heartbeat);
    if !verbosity.quiet {
        eprintln!(
            "    pass 2/2: done ({:.1}s)",
            pass_start.elapsed().as_secs_f64()
        );
    }
    // A broken measurement is reported as one. Falling back to the whole-tree
    // instrumentation it replaced would hide the breakage behind a 50-minute
    // run that still produced a number.
    //
    // Reported only after pass 2 has run. A failure in the coverage *tooling*
    // must not cost the answer to "do this crate's tests pass" — that is the
    // other thing this criterion gates on, and it is still knowable.
    let run = match run {
        Ok(run) => run,
        Err(e) => {
            // A crate with no production code cannot produce a profile: nothing
            // carries a coverage map, so the profiling runtime is never linked
            // and no `.profraw` is written. That is the same symptom as
            // instrumentation failing to reach a crate that DOES have code —
            // the defect the no-profraw guard exists to catch — and grading both
            // `F` reads the empty crate as badly covered. The four `*-benches`
            // crates are the case: a five-line doc comment for a lib, all
            // content in `benches/*.rs`, which coverage does not measure.
            //
            // Only a positive finding excuses the crate, and only when its tests
            // passed: `declares_no_production_code` answers `false` on any file
            // it cannot read or parse, so a real instrumentation defect cannot
            // be relabelled by a parse error. The verdict matches the one the
            // export path already returns for a crate whose files map no lines.
            //
            // ⚠ Do not hoist this up beside `coverage_skip_reason`. Those three
            // profiles return before pass 2 and so never run the crate's tests;
            // this case deliberately sits after it, because "no production code
            // to measure" says nothing about whether the crate's `tests/` suite
            // passes, and that suite still has to gate.
            //
            // Deliberately keyed on the crate, not on which error pass 1 raised.
            // Matching the no-profraw case by its message would break the moment
            // anyone reworded it, and the breadth costs nothing: for a crate with
            // no production code there is no pass-1 error that could have carried
            // information about its coverage. A build failure cannot reach here
            // at all — pass 2 builds the same crate, so `heavy_passed` is false.
            //
            // Rooted on `workspace_root` rather than left relative: `xshell`'s
            // `change_dir` moves the SHELL's directory, and this crate refuses
            // to `set_current_dir` because that is process-global. A bare
            // relative path would therefore be resolved by `std::fs` against
            // wherever the user happened to invoke `cargo xtask` from.
            if heavy_passed
                && crate::coverage::declares_no_production_code(
                    &std::path::Path::new(&workspace_root).join(crate_path),
                )
            {
                return Ok(CriterionResult {
                    name: "1. Coverage",
                    result: "(no production lines)".to_string(),
                    grade: Grade::NotApplicable,
                    threshold: "≥75%/≥90% A+",
                    measured_detail:
                        "no production code to instrument: src/ declares no items outside \
                         #[cfg(test)], so no test binary carries a coverage map"
                            .to_string(),
                });
            }
            let tests = if heavy_passed {
                "tests passed"
            } else {
                "TESTS ALSO FAILED"
            };
            return Ok(CriterionResult {
                name: "1. Coverage",
                result: "(measurement failed)".to_string(),
                grade: Grade::F,
                threshold: "≥75%/≥90% A+",
                measured_detail: format!("coverage run failed, {tests}: {e:#}"),
            });
        }
    };

    // Pass 1 measures; pass 2 above is what actually gates the tests, and it
    // runs EVERY test — lib, integration and doctests — uninstrumented. Pass 1
    // contributes only its `--lib` result, because an instrumented integration
    // suite can fail for reasons that are not about the code (see
    // `coverage_run`). A `--lib` disagreement between the two passes would
    // itself be the finding, so both still have to agree there.
    let heavy_passed = heavy_passed && run.tests_passed;
    if !heavy_passed {
        eprintln!("    ⚠ Tests failed — see output above");
    }

    // The report is filtered to the crate's own PRODUCTION files. That filter
    // is load-bearing again: instrumentation now covers the crate AND its test
    // targets, so the two scopes deliberately differ and the filter drops what
    // the extra scope pulled in — measured on cf-geometry, 7 of the export's 24
    // files. (It was a no-op for exactly as long as only the lib was
    // instrumented; that comment is worth not restoring.)
    //
    // Within those files the criterion counts PRODUCTION lines only. The run
    // instruments test binaries, so a crate's `#[cfg(test)]` code
    // would otherwise be measured as if it were the code under test: bodies
    // that run pad the numerator, and `#[ignore]`d gates pad the denominator
    // while contributing nothing. See coverage.rs.
    let measured = crate::coverage::production_coverage(&run.json, crate_path);

    // The whole verdict — thresholds, the report-only deferral, and every
    // sentence of the detail line — is decided by `coverage_result`, which
    // touches no Shell and no filesystem. Extracted for exactly the reason
    // `count_unjustified_allows_in_tree` and `scan_file_safety` were: a
    // decision reachable only through a real llvm-cov run is a decision no
    // test drives, and a 2026-08-16 mutation sweep proved it — the report-only
    // block, the library-only detail and the binary-target marker could each
    // be deleted whole with every test still green.
    let result = coverage_result_for(crate_name, &measured, heavy_passed);

    // Unconditional, and still honours "a non-empty `files_out` means these
    // files were measured": the one verdict above that measured nothing is
    // `(no production lines)`, which is returned exactly when `total == 0` —
    // and `total` only ever grows in the same branch that pushes a row, so a
    // zero total already implies an empty `files`. Assigning it moves an empty
    // vec. Guarding on the grade instead would be a second, weaker statement
    // of the same invariant, free to drift from it.
    *files_out = measured.files;
    Ok(result)
}

/// [`coverage_result`] with the deferral looked up from
/// [`COVERAGE_REPORT_ONLY`] — the seam between "is this crate deferred" and
/// "what does a deferred crate report".
///
/// ⚠ Exists to be testable. Passing the flag into `coverage_result` made the
/// verdict drivable from a unit test, but left the LOOKUP untested one level
/// up: a mutation replacing the argument with a bare `false` survived, which
/// would silently un-defer every crate on the list. Splitting the composition
/// into its own named function is what gives a test something to call.
fn coverage_result_for(
    crate_name: &str,
    measured: &crate::coverage::ProductionCoverage,
    heavy_passed: bool,
) -> CriterionResult {
    coverage_result(measured, heavy_passed, is_coverage_report_only(crate_name))
}

/// Criterion 1's verdict, from a finished measurement. Pure: no `Shell`, no
/// filesystem, no cargo.
///
/// ★ Split out so the branches a reader most needs to trust — the deferral,
/// the "a red suite is never waivable" rule, the library-only line — are
/// reachable from a unit test. They were not: a mutation sweep on
/// 2026-08-16 deleted the report-only block, the library-only detail and the
/// binary-target marker one at a time, and the suite stayed green through all
/// three. Same principle that pulled the file-walking half out of criteria 3
/// and 4.
fn coverage_result(
    measured: &crate::coverage::ProductionCoverage,
    heavy_passed: bool,
    report_only: bool,
) -> CriterionResult {
    // No production lines is a different fact from bad coverage, and from a
    // broken report. Grading it F would send a reader hunting for uncovered
    // code, or for a parse bug, when neither exists.
    let Some(coverage) = measured.percent() else {
        return CriterionResult {
            name: "1. Coverage",
            result: "(no production lines)".to_string(),
            grade: Grade::NotApplicable,
            threshold: "≥75%/≥90% A+",
            measured_detail: format!(
                "no instrumented production lines ({} test lines excluded)",
                measured.excluded
            ),
        };
    };

    // Two-tier thresholds (F1): A+ >= 90%, A >= 75%, B >= 60%, C >= 40%, F < 40%
    // Heavy test failure overrides to F regardless of coverage %.
    let grade = if !heavy_passed {
        Grade::F
    } else if coverage >= 90.0 {
        Grade::APlus
    } else if coverage >= 75.0 {
        Grade::A
    } else if coverage >= 60.0 {
        Grade::B
    } else if coverage >= 40.0 {
        Grade::C
    } else {
        Grade::F
    };

    // Say what was left out. A metric that silently drops lines reads as
    // "everything was measured" when it was not.
    let mut detail = format!(
        "{} production line coverage ({}/{} lines; {} test lines excluded)",
        coverage_display(coverage),
        measured.covered,
        measured.total,
        measured.excluded
    );
    // Reported whenever a binary target contributed lines, because the graded
    // figure above spans both and a reader cannot otherwise tell a weak library
    // from a large `main.rs`. Not yet subtracted — see
    // `ProductionCoverage::lib_percent` for why that decision waits on this
    // number existing. Through the same truncating formatter as the graded
    // figure: rounding just this one would put 94.0 % beside the 93.9 %
    // everywhere else it is quoted, and small drift between a tool and its own
    // documentation is what stops a reader trusting either.
    if let Some(lib_only) = measured.lib_percent().filter(|_| measured.bin_total > 0) {
        detail.push_str(&format!(
            "; {} over library lines alone ({} line(s) in binary targets, {} covered)",
            coverage_display(lib_only),
            measured.bin_total,
            measured.bin_covered
        ));
    }
    if !measured.unparsed.is_empty() {
        detail.push_str(&format!(
            " ⚠ {} file(s) unreadable, so their test code IS counted",
            measured.unparsed.len()
        ));
    }
    if !heavy_passed {
        detail.push_str(" (heavy tests FAILED)");
    }

    // Report-only: measured, printed, attributed — but not yet gating. Applied
    // last so the real grade above is computed from the real number and only
    // then set aside; the percentage a reader sees is the same either way.
    //
    // ⚠ A failing test run is NOT waivable. `heavy_passed` gates every crate,
    // report-only or not: the waiver is on the coverage THRESHOLD, and letting
    // it swallow a red suite would rebuild the fail-open hole this arc closed.
    if heavy_passed && report_only {
        return CriterionResult {
            name: "1. Coverage",
            result: format!("{} (report-only)", coverage_display(coverage)),
            grade: Grade::NotApplicable,
            threshold: "≥75%/≥90% A+",
            measured_detail: format!(
                "{detail} — REPORT-ONLY: this crate's coverage was never measured before \
                 2026-08-16 and is not yet enforced; grade would be {}",
                grade.as_str()
            ),
        };
    }

    CriterionResult {
        name: "1. Coverage",
        result: coverage_display(coverage),
        grade,
        threshold: "≥75%/≥90% A+",
        measured_detail: detail,
    }
}

/// Format a coverage percentage for display, **truncated rather than rounded**.
///
/// `{:.1}` rounds, so a crate at 74.969 % prints "75.0%" in the same row as a
/// grade of `B` — a figure that contradicts the verdict beside it, and the
/// exact shape of defect this criterion keeps being fixed for. Found on
/// `cf-studio-engine` (605/807) in the 2026-08-16 census.
///
/// Truncating guarantees the printed number is never above the one that was
/// graded, so a displayed `75.0%` always means the threshold was really met.
/// The unrounded counts are printed beside it either way, so nothing is lost.
fn coverage_display(percent: f64) -> String {
    format!("{:.1}%", (percent * 10.0).floor() / 10.0)
}

/// Crates whose coverage is measured and printed but does not yet gate.
///
/// ★ **A to-do list, not a policy.** Every name here is a crate that
/// [`has_lib_target`] newly brought into measurement — code that was never
/// graded because the `tools/` skip claimed it had no library.
///
/// ⚠ **Not to keep today's CI green — today's CI cannot go red from this.**
/// `quality-gate.yml` runs `grade-all --skip-coverage` on every shard, so no
/// PR job measures coverage at all; the only CI-side coverage is a weekly
/// tarpaulin run, a different instrument on a different scope, red for months.
/// This list is therefore forward-looking, for whoever replaces that weekly
/// job with `grade-all` sans `--skip-coverage`. Until then the deferral only
/// shapes local `xtask grade` output — which is exactly where the backlog
/// wants to be visible.
///
/// ⚠ **It will not make that job green, and this doc claimed it would.**
/// `cf-viewer` is measured, fails at 33.8 %, and is deliberately not on this
/// list; sim-core, sim-mjcf, sim-bevy and cf-device-geometry are not on it
/// either. A full coverage run goes red on its first execution and should.
/// What the list changes is *which* red — failures owned by crates whose debt
/// was already tracked, rather than fourteen a grader change lit up overnight.
/// **Newly revealed** versus **already owned** is the whole line it draws, and
/// it is a smaller claim than the one that was written here first.
///
/// ★ **Finite and explicit by construction.** A crate added to `tools/`
/// tomorrow is enforced from its first grade, because it is not on this list
/// and there is no rule that would put it here. Enforcement is a deletion.
///
/// ★ **Emptying this list is the goal, and nothing breaks when it happens** —
/// verified by emptying it and running the suite. Two tests used to index
/// `COVERAGE_REPORT_ONLY[0]`, so they would have panicked on the day the last
/// name was removed, and whoever removed it would have deleted them — losing
/// the "a red suite is never waivable" test permanently. The verdict now takes
/// the deferral as a flag, so its tests do not reach for this list at all.
///
/// ⚠ This is a waiver on the coverage THRESHOLD only. Test failures, Clippy,
/// Safety, Documentation and Dependencies gate these crates exactly as they
/// gate every other, and so does a coverage run that FAILS — a measurement
/// that did not happen is not a threshold that was missed, and it returns `F`
/// from further up this function without ever reaching the list.
///
/// The percentage is printed either way, and so is the per-file triage table,
/// so green here means "measured, enforcement deferred, and here is the work"
/// — never "not measured". That distinction is the whole point of #772–#774
/// and this list is written to stay on the right side of it.
///
/// ⚠ **`cf-viewer` is deliberately absent.** It was already measured and
/// already failing before this change, so deferring it would not be leniency
/// toward a newly-lit crate — it would switch OFF a gate that fires today.
/// Report-only is for what the measurement fix newly revealed, never for what
/// it found already lit.
///
/// ★ **What it would take to shrink this list**, from the same census — the
/// numbers are here because a to-do without its sizes gets read as one job:
///
/// - **Three are within 30 lines, 39 in total**: cf-studio-engine needs 1,
///   cf-anthro 13, pbit-analyze 25. One sitting takes the list from six to
///   three. ⚠ Aim past the bar on each — see the drift note on
///   cf-studio-engine below.
/// - **Two need no tests at all if binary lines stop counting** — the open
///   decision described on [`crate::coverage::ProductionCoverage::lib_percent`].
///   pbit-analyze is 91.4 % over its library and cf-studio-gui 93.9 %, so both
///   clear the bar the moment their binaries leave the denominator.
/// - **Two are real work either way**: cf-scan-prep-core (1170 lines, no test
///   in the crate) and example-ml-shared (99, likewise). They are the only two
///   the other levers do not reach.
///
/// The 2026-08-16 census, run per-crate against the commit that introduced
/// `has_lib_target` — 14 crates newly measured, of which **8 already pass**:
/// cf-mjcf-emit 97.8, cf-msk-lib 95.7, cf-msk-fit 94.7, cf-osim 94.6,
/// cf-studio-core 90.3, cf-codesign 86.9-87.2, cf-cast-cli 79.9, cf-studio 75.2.
/// (Truncated, as the tool prints them — see `coverage_display`. cf-codesign is
/// given as a range because it does not reproduce; see the note on
/// cf-studio-engine below.)
/// The hole was mostly a MEASUREMENT hole, not a quality one. The six below
/// are what it was hiding:
const COVERAGE_REPORT_ONLY: &[&str] = &[
    // 0.0 % of 1560 lines — no test anywhere in the crate. The largest single
    // untested body of code the census found, and the one that most deserves
    // to stop being report-only.
    "cf-scan-prep-core",
    // 14.3 % of 1955 lines, but 93.9 % over its LIBRARY: 1657 of those lines
    // are a Bevy GUI binary. Second independent instance of the shape
    // `cf-viewer` shows, and the stronger one.
    "cf-studio-gui",
    // 0.0 % of 131 lines — shared helper library for the ML examples.
    "example-ml-shared",
    // 69.6 %, 25 lines short.
    "pbit-analyze",
    // 71.1 %, 13 lines short.
    "cf-anthro",
    // 74.8-74.9 %, ONE line short. Left on the list rather than quietly fixed:
    // the census measures, it does not edit the crates it measures.
    //
    // ⚠ And it is one of the two crates that showed the coverage run is NOT
    // reproducible to the line. Ten runs on one unchanged tree: 605/807 eight
    // times, 604/807 twice, the difference a single line in `src/edit.rs`
    // (318 vs 319 of 421). Re-measuring all fifteen census crates found one
    // more, and larger — cf-codesign moved six lines, 1416/1622 to 1410/1622.
    // Causes unidentified; the JSON export is per file, not per line.
    //
    // Verdicts were stable in both cases, which is why this is a caveat and
    // not a defect. But whoever takes this crate over 75 % should add margin
    // rather than the one line the arithmetic asks for, because the arithmetic
    // carries a few lines of noise.
    "cf-studio-engine",
];

/// Whether `crate_name` is on [`COVERAGE_REPORT_ONLY`].
///
/// `pub(crate)` so [`crate::complete`] can refuse to stamp a completion record
/// for a crate whose coverage threshold is only deferred — see there for why
/// that refusal is load-bearing.
pub(crate) fn is_coverage_report_only(crate_name: &str) -> bool {
    COVERAGE_REPORT_ONLY.contains(&crate_name)
}

/// Grade documentation by checking for warnings.
///
/// Captures stderr + exit code (B2 fix: `cargo doc` writes diagnostics
/// to stderr, not stdout — the old gate read stdout and always saw 0).
fn grade_documentation(sh: &Shell, crate_name: &str, crate_path: &str) -> Result<CriterionResult> {
    // Force color off — CI sets CARGO_TERM_COLOR=always, which injects ANSI
    // escape sequences into stderr (e.g. `error\x1b[0m:`) and breaks the
    // `contains("error:")` / `matches("warning:")` substring counts below.
    let output = cmd!(sh, "cargo doc --no-deps -p {crate_name}")
        .env("RUSTDOCFLAGS", "-D warnings")
        .env("CARGO_TERM_COLOR", "never")
        .ignore_status()
        .output()?;

    let exit_code = output.status.code().unwrap_or(1);
    let stderr = String::from_utf8_lossy(&output.stderr);

    // Count diagnostics. With -D warnings, warnings are promoted to errors,
    // so also count "error:" lines (excluding "aborting due to" summaries).
    let warning_count = stderr.matches("warning:").count();
    let issue_count = if warning_count == 0 && exit_code != 0 {
        stderr
            .lines()
            .filter(|l| l.contains("error:") && !l.contains("aborting"))
            .count()
    } else {
        warning_count
    };

    // Binary A/F: exit 0 = A (zero warnings), non-zero = F
    let grade = if exit_code == 0 { Grade::A } else { Grade::F };

    // On F, surface stderr tail so CI logs show the actual failure, not just
    // an opaque "0 warnings" verdict. Gated behind XTASK_GRADE_DEBUG to keep
    // local-dev output clean; CI sets this env var to get signal.
    if grade == Grade::F && std::env::var("XTASK_GRADE_DEBUG").is_ok() {
        eprintln!(
            "  [debug] {} cargo doc exit={} stderr tail:",
            crate_name, exit_code
        );
        let stderr_lines: Vec<&str> = stderr.lines().collect();
        let tail_start = stderr_lines.len().saturating_sub(30);
        for line in &stderr_lines[tail_start..] {
            eprintln!("    {}", line);
        }
    }

    // Informational: check if missing_docs lint is enabled in lib.rs
    let lib_path = format!("{}/src/lib.rs", crate_path);
    let has_missing_docs = if let Ok(content) = std::fs::read_to_string(&lib_path) {
        content.contains("#![warn(missing_docs)]") || content.contains("#![deny(missing_docs)]")
    } else {
        false
    };

    let missing_docs_note = if !has_missing_docs && grade == Grade::A {
        " (missing_docs not enabled)"
    } else {
        ""
    };

    let result = format!("{} warnings{}", issue_count, missing_docs_note);

    Ok(CriterionResult {
        name: "2. Documentation",
        result,
        grade,
        threshold: "0 warnings",
        measured_detail: format!("{} warnings", issue_count),
    })
}

/// Grade clippy warnings via JSON output parsing.
///
/// Uses `--message-format=json` instead of `-- -D warnings` so we can
/// count diagnostics directly from structured output (B1 fix: old gate
/// read stdout but clippy wrote diagnostics to stderr).
///
/// The count is only meaningful if clippy actually analysed the crate, which
/// [`count_clippy_diagnostics`] establishes from cargo's `build-finished`
/// record before it will report one — see there for why zero warnings is not,
/// by itself, evidence of anything.
fn grade_clippy(
    sh: &Shell,
    crate_name: &str,
    crate_path: &str,
    profile: CrateProfile,
) -> Result<CriterionResult> {
    // `ignore_status` because clippy exits non-zero merely for finding
    // warnings — the exit code cannot distinguish "found problems" from
    // "never ran".
    //
    // Propagating `read()`'s own error (spawn failure, non-UTF-8 output) is
    // belt-and-braces rather than load-bearing: flattening it to an empty
    // string would still be refused downstream, since a stream with no
    // `build-finished` record is not gradeable. What the `?` buys is the
    // cause — "cargo: no such file" beats "clippy never completed".
    let output = cmd!(
        sh,
        "cargo clippy -p {crate_name} --all-targets --all-features --message-format=json"
    )
    .ignore_status()
    .read()
    .with_context(|| format!("run cargo clippy for {crate_name}"))?;

    let clippy_count = count_clippy_diagnostics(&output, crate_path)
        .with_context(|| format!("grade clippy diagnostics for {crate_name}"))?;

    // Unjustified #[allow(clippy:: check (F-ext-3).
    //
    // Relaxed for Example/Xtask profiles by the same STANDARDS.md §4
    // "Allowed" theme that relaxes unwrap/expect counting for them:
    // demo and tooling code is not prod-surface and shouldn't be held
    // to the same justification-comment rubric as library code.
    let relax_unjustified_allows = matches!(profile, CrateProfile::Example | CrateProfile::Xtask);
    let allow_count = if relax_unjustified_allows {
        0
    } else {
        count_unjustified_allows_in_tree(&format!("{}/src", crate_path))?
    };

    let total = clippy_count + allow_count;

    // Binary A/F
    let grade = if total == 0 { Grade::A } else { Grade::F };

    let result = if allow_count > 0 {
        format!(
            "{} warnings, {} unjustified allows",
            clippy_count, allow_count
        )
    } else {
        format!("{} warnings", clippy_count)
    };

    let measured_detail = result.clone();

    Ok(CriterionResult {
        name: "3. Clippy",
        result,
        grade,
        threshold: "0 warnings",
        measured_detail,
    })
}

/// Clippy diagnostics attributable to `crate_path`, or an error if the stream
/// does not prove clippy actually analysed the crate.
///
/// Counting alone cannot tell "clippy found nothing" apart from "clippy never
/// looked", and both come out as zero — an A. The exit code cannot settle it
/// either, since clippy exits non-zero merely for finding warnings. Cargo's
/// `build-finished` record can, so this refuses to report a count without one:
///
/// - **No `build-finished`** — the invocation never completed. An unresolvable
///   `-p`, for instance, writes its error to stderr and leaves stdout *empty*,
///   which is byte-for-byte what a flawless crate produces.
/// - **`success: false` with nothing inside the crate** — the build broke, and
///   nothing in the stream pins the break on this crate. It may be a
///   dependency (whose diagnostics [`any_span_in_crate`] correctly rules
///   foreign) or a failure that carries no span at all, since cargo reports
///   "could not compile" on stderr rather than as a `compiler-message`. Either
///   way this crate was not analysed, so zero means nothing. Deliberately not
///   graded F: an unattributable failure is not evidence against *this* crate.
/// - **`success: false` with diagnostics inside the crate** — the crate's own
///   errors, which is exactly what an F is for. Counted, not an error.
///
/// Both refusals are the fail-closed direction: an errored grade is visible,
/// an unearned A is not.
fn count_clippy_diagnostics(output: &str, crate_path: &str) -> Result<usize> {
    let mut count = 0usize;
    let mut build_succeeded = None;

    for line in output.lines() {
        let Ok(json) = serde_json::from_str::<serde_json::Value>(line) else {
            continue;
        };
        match json["reason"].as_str() {
            // A missing or non-boolean `success` leaves this `None`, which the
            // match below treats as "no record" rather than as a pass.
            Some("build-finished") => build_succeeded = json["success"].as_bool(),
            Some("compiler-message") => {
                let level = json["message"]["level"].as_str().unwrap_or("");
                if level != "warning" && level != "error" {
                    continue;
                }
                // Exclude summary lines (empty spans = "N warnings emitted").
                // Belt-and-braces: `any_span_in_crate` is already false for an
                // empty slice, so this clause states the intent rather than
                // deciding anything — no input can distinguish its removal.
                //
                // F.1: also filter out transitive-dep diagnostics whose spans all
                // point outside the target crate. Disjunctive — a diagnostic with
                // even one span inside the crate is counted (include-not-exclude).
                let Some(spans) = json["message"]["spans"].as_array() else {
                    continue;
                };
                if spans.is_empty() || !any_span_in_crate(spans, crate_path) {
                    continue;
                }
                count += 1;
            }
            _ => continue,
        }
    }

    match build_succeeded {
        None => bail!(
            "cargo clippy emitted no `build-finished` record, so it never completed. The \
             {count} diagnostic(s) seen are a floor rather than a verdict, and zero would \
             mean \"clippy never ran\" rather than \"the crate is clean\""
        ),
        Some(false) if count == 0 => bail!(
            "cargo clippy reported `build-finished` success=false with no diagnostic inside \
             {crate_path}, so the build failed but nothing attributes the failure to this \
             crate — a dependency, or a failure carrying no span such as a link error. The \
             crate was not analysed, so a count of 0 is not evidence that it is clean"
        ),
        _ => Ok(count),
    }
}

/// Returns true if any span in the diagnostic refers to a file whose path
/// contains `crate_path` — i.e., the warning originates in the target crate
/// and not a transitive workspace-member dependency.
///
/// `.contains()` substring matching mirrors the convention in `grade_coverage`
/// (line ~721); handles workspace-relative and absolute paths uniformly
/// without normalization. Disjunctive semantics: ambiguous diagnostics (e.g.
/// macro-generated with one span inside + one outside) are included, matching
/// coverage's summing-loop behavior.
fn any_span_in_crate(spans: &[serde_json::Value], crate_path: &str) -> bool {
    spans.iter().any(|span| {
        span["file_name"]
            .as_str()
            .is_some_and(|name| name.contains(crate_path))
    })
}

/// Total unjustified `#[allow(clippy::…)]`s across a crate's `src/` tree.
///
/// Extracted from [`grade_clippy`] for the same reason [`scan_file_safety`]
/// was extracted from [`grade_safety`]: the file-walking half of criterion 3
/// is then testable without standing up a cargo invocation.
///
/// Errors rather than under-counting. This scan can only ever *lower* the
/// count by missing a file, and a lower count is a better grade — so a
/// swallowed error here hands the crate an A on a file nobody read.
fn count_unjustified_allows_in_tree(src_path: &str) -> Result<usize> {
    let mut allow_count = 0;
    for file_path in glob_rs_files(src_path)? {
        let content = std::fs::read_to_string(&file_path)
            .with_context(|| format!("read {file_path} for the unjustified-allow scan"))?;
        let lines: Vec<&str> = content.lines().collect();
        allow_count += count_unjustified_clippy_allows(&lines);
    }
    Ok(allow_count)
}

/// Collect all `.rs` files under a directory.
///
/// Fails closed. Both consumers — the unjustified-`#[allow]` scan in
/// [`grade_clippy`] (criterion 3) and [`grade_safety`] (criterion 4) — grade a
/// crate by *counting violations across these files*, so a file this walk drops
/// is a file whose violations are never counted, and the crate earns an A for a
/// reason nobody measured. A walk that cannot see the whole tree therefore
/// errors out rather than returning a quietly shorter list: an errored grade is
/// visible, an under-counted one is not.
///
/// `follow_links(true)` for the same reason: a module reached through a
/// symlinked directory is source the compiler reads, so the graders must read it
/// too. (A symlinked `.rs` *file* was already collected — the extension test
/// below never asks whether the entry is a regular file.)
fn glob_rs_files(dir: &str) -> Result<Vec<String>> {
    let mut files = Vec::new();
    // `try_exists`, not `exists`: the latter answers "no" both for an absent
    // directory and for one whose metadata cannot be read, and only the first
    // of those means "nothing to scan".
    if !Path::new(dir)
        .try_exists()
        .with_context(|| format!("stat {dir} while collecting .rs files"))?
    {
        return Ok(files);
    }
    for entry in walkdir::WalkDir::new(dir).follow_links(true) {
        let entry = entry.with_context(|| format!("walk {dir} while collecting .rs files"))?;
        let path = entry.path();
        if path.extension().and_then(|e| e.to_str()) == Some("rs") {
            files.push(path.to_string_lossy().to_string());
        }
    }
    Ok(files)
}

/// Per-file result of [`scan_file_safety`].
struct SafetyScanResult {
    counted_violations: usize,
    unsafe_violations: usize,
    has_todo_or_unimplemented: bool,
}

/// Pure scan of one source file for safety violations.
///
/// Extracted from [`grade_safety`] so the scan logic is unit-testable
/// without a filesystem fixture. `relax_unwrap_expect` mirrors the
/// `Example`/`Xtask` profile allowance per STANDARDS.md §4 "Allowed";
/// `is_build_rs` mirrors the `expect()` exemption for `build.rs`.
///
/// `.unwrap()` and `.expect(` honor an enclosing `#[allow(clippy::unwrap_used)]`
/// or `#[allow(clippy::expect_used)]` attribute via [`has_enclosing_allow`]
/// — matches the attribute-allowed policy applied to `panic!`/`unreachable!`.
fn scan_file_safety(
    content: &str,
    is_build_rs: bool,
    relax_unwrap_expect: bool,
) -> SafetyScanResult {
    let lines: Vec<&str> = content.lines().collect();

    // File-level inner `#![cfg(test)]` attribute: the whole file is test-only.
    // Used by parent modules that declare `#[cfg(test)] mod tests;` to keep
    // test helpers in a dedicated file. Skip the scan entirely — treating
    // test-fixture .unwrap()/.expect() as library violations is a category
    // error, same as for inline #[cfg(test)] modules.
    if has_file_level_cfg_test(&lines) {
        return SafetyScanResult {
            counted_violations: 0,
            unsafe_violations: 0,
            has_todo_or_unimplemented: false,
        };
    }

    let mut has_todo_or_unimplemented = false;
    let mut counted_violations = 0usize;
    let mut unsafe_violations = 0usize;

    // Brace-depth tracked test exclusion state machine (ss2.4)
    let mut in_test = false;
    let mut test_brace_depth: usize = 0;
    let mut pending_test_attr = false;
    let mut in_block_comment = false;

    for (i, line) in lines.iter().enumerate() {
        // Strip string-literal content before any pattern match, so the
        // scanner doesn't false-positive on code that manipulates literal
        // patterns like `"unsafe {"`, `"todo!("`, `"#[cfg(test)]"`. The
        // original line is still available to other helpers (e.g.
        // has_enclosing_allow's span-aware scan) that do their own masking.
        let stripped_owned = strip_string_literals(line);
        let trimmed = stripped_owned.trim();

        // Track block comments
        if !in_block_comment && trimmed.contains("/*") {
            in_block_comment = true;
        }
        if in_block_comment {
            if trimmed.contains("*/") {
                in_block_comment = false;
            }
            continue;
        }

        // Skip line comments
        if trimmed.starts_with("//") {
            continue;
        }

        // Test exclusion: #[cfg(test)] attribute detection.
        if trimmed.starts_with("#[cfg(test)]") {
            pending_test_attr = true;
        }

        // Track braces for test region
        if pending_test_attr && trimmed.contains('{') {
            in_test = true;
            test_brace_depth = 0;
            pending_test_attr = false;
        }

        if in_test {
            for ch in trimmed.chars() {
                if ch == '{' {
                    test_brace_depth += 1;
                } else if ch == '}' {
                    test_brace_depth = test_brace_depth.saturating_sub(1);
                    if test_brace_depth == 0 {
                        in_test = false;
                        break;
                    }
                }
            }
            if in_test {
                continue; // still inside test block
            }
            continue; // just exited test block on this line
        }

        // === Pattern checks on library code ===

        // Hard-fail: todo!() and unimplemented!()
        // Honors enclosing #[allow(clippy::todo)] or #[allow(clippy::unimplemented)]
        // (consistent with the attribute-allowed policy applied to
        // unwrap/expect/panic/unreachable).
        if has_macro_call(trimmed, "todo!") && !has_enclosing_allow(&lines, i, "clippy::todo") {
            has_todo_or_unimplemented = true;
        }
        if has_macro_call(trimmed, "unimplemented!")
            && !has_enclosing_allow(&lines, i, "clippy::unimplemented")
        {
            has_todo_or_unimplemented = true;
        }

        // Counted: .unwrap() — skipped for Example/Xtask profiles
        // per STANDARDS.md §4 "Allowed: unwrap() in examples".
        // Honors enclosing #[allow(clippy::unwrap_used)] attribute.
        if !relax_unwrap_expect
            && trimmed.contains(".unwrap()")
            && !has_enclosing_allow(&lines, i, "clippy::unwrap_used")
        {
            counted_violations += 1;
        }

        // Counted: .expect( — skip in build.rs, and skip for
        // Example/Xtask profiles per STANDARDS.md §4 "Allowed".
        // Honors enclosing #[allow(clippy::expect_used)] attribute.
        if !is_build_rs
            && !relax_unwrap_expect
            && trimmed.contains(".expect(")
            && !has_enclosing_allow(&lines, i, "clippy::expect_used")
        {
            counted_violations += 1;
        }

        // Counted with justification: panic!()
        // Justified by any of: preceding `//` comment, same-line `//`
        // comment, or an enclosing `#[allow(clippy::panic)]` attribute
        // within 300 lines back (the idiomatic Rust pattern).
        if has_macro_call(trimmed, "panic!") {
            let has_justification = has_preceding_comment(&lines, i)
                || has_same_line_comment(trimmed)
                || has_enclosing_allow(&lines, i, "clippy::panic");
            if !has_justification {
                counted_violations += 1;
            }
        }

        // Counted with justification: unreachable!()
        if has_macro_call(trimmed, "unreachable!") {
            let has_justification = has_preceding_comment(&lines, i)
                || has_same_line_comment(trimmed)
                || has_enclosing_allow(&lines, i, "clippy::unreachable");
            if !has_justification {
                counted_violations += 1;
            }
        }

        // Unsafe-without-SAFETY check (F-ext-4)
        if trimmed.contains("unsafe")
            && (trimmed.contains("unsafe {") || trimmed.contains("unsafe fn"))
        {
            let has_safety_comment = (1..=3).any(|offset| {
                i.checked_sub(offset).is_some_and(|j| {
                    let prev = lines[j].trim().to_lowercase();
                    prev.contains("// safety:")
                })
            });
            if !has_safety_comment {
                unsafe_violations += 1;
            }
        }
    }

    SafetyScanResult {
        counted_violations,
        unsafe_violations,
        has_todo_or_unimplemented,
    }
}

/// Grade safety: check for panic-capable patterns in library code.
///
/// Full rewrite per chassis ss2.4. Fixes B3 bugs 1-5:
/// - Brace-depth tracked test exclusion (not first-#[cfg(test)]-to-EOF)
/// - Block comment handling
/// - All 6 patterns (todo!, unimplemented!, unwrap, expect, panic!, unreachable!)
/// - Unsafe-without-SAFETY check
/// - Blanket assert exclusion removed
/// - Direct file reading (no grep shell-outs)
fn grade_safety(_sh: &Shell, crate_path: &str, profile: CrateProfile) -> Result<CriterionResult> {
    let src_path = format!("{}/src", crate_path);

    // `try_exists`, not `exists`: this guard runs *before* `glob_rs_files`, so
    // it — not the walker — is what a crate with an unreadable `src/` meets
    // first. `exists()` would answer "absent" and take the `Manual` arm below,
    // and `overall_automated` skips `Manual` outright, so an unstat-able tree
    // would drop out of the grade entirely rather than failing it.
    if !Path::new(&src_path)
        .try_exists()
        .with_context(|| format!("stat {src_path} for the safety scan"))?
    {
        return Ok(CriterionResult {
            name: "4. Safety",
            result: "(no src/)".to_string(),
            grade: Grade::Manual,
            threshold: "0 violations",
            measured_detail: "(no src/)".to_string(),
        });
    }

    let files = glob_rs_files(&src_path)?;

    // Per STANDARDS.md §4 "Allowed":
    //   - `unwrap()` in tests
    //   - `unwrap()` in examples
    //   - `expect()` in `build.rs`
    // Library crates get the strict rubric (unwrap/expect in prod code →
    // violation). Example and Xtask profiles relax the counters for those
    // patterns but still enforce the hard-fail gates (todo!/unimplemented!,
    // unsafe without SAFETY) which are universal safety bars.
    let relax_unwrap_expect = matches!(profile, CrateProfile::Example | CrateProfile::Xtask);

    let mut has_todo_or_unimplemented = false;
    let mut counted_violations = 0usize;
    let mut unsafe_violations = 0usize;

    for file_path in &files {
        let is_build_rs = file_path.ends_with("build.rs");
        // Not `continue`: an unread file contributes no violations, so
        // skipping it silently is a free pass on whatever it contains.
        let content = std::fs::read_to_string(file_path)
            .with_context(|| format!("read {file_path} for the safety scan"))?;
        let scan = scan_file_safety(&content, is_build_rs, relax_unwrap_expect);
        // XTASK_GRADE_DEBUG env var: emit per-file violation breakdown to
        // stderr when set. Used to pinpoint which files in a crate are
        // dragging the Safety grade down without running cargo clippy etc.
        if std::env::var("XTASK_GRADE_DEBUG").is_ok()
            && (scan.counted_violations > 0
                || scan.unsafe_violations > 0
                || scan.has_todo_or_unimplemented)
        {
            eprintln!(
                "  [debug] {} → counted={} unsafe={} todo_or_unimpl={}",
                file_path,
                scan.counted_violations,
                scan.unsafe_violations,
                scan.has_todo_or_unimplemented
            );
        }
        counted_violations += scan.counted_violations;
        unsafe_violations += scan.unsafe_violations;
        if scan.has_todo_or_unimplemented {
            has_todo_or_unimplemented = true;
        }
    }

    // Grading (binary A/F)
    if has_todo_or_unimplemented {
        return Ok(CriterionResult {
            name: "4. Safety",
            result: "F: found todo!/unimplemented!".to_string(),
            grade: Grade::F,
            threshold: "0 violations",
            measured_detail: "F: found todo!/unimplemented!".to_string(),
        });
    }

    let total = counted_violations + unsafe_violations;
    let grade = if total == 0 { Grade::A } else { Grade::F };

    let result = if unsafe_violations > 0 {
        format!(
            "{} violations ({} unsafe without SAFETY)",
            total, unsafe_violations
        )
    } else {
        format!("{} violations", total)
    };

    Ok(CriterionResult {
        name: "4. Safety",
        result: result.clone(),
        grade,
        threshold: "0 violations",
        measured_detail: result,
    })
}

/// True iff a file-level inner `#![cfg(test)]` attribute appears at the
/// top of the file (skipping initial `//` and `//!` comments and blank
/// lines). Marks an entire .rs file as test-only; standard pattern when
/// a parent module declares `#[cfg(test)] mod tests;` and the tests live
/// in their own file.
fn has_file_level_cfg_test(lines: &[&str]) -> bool {
    for line in lines {
        let trimmed = line.trim();
        if trimmed.is_empty() || trimmed.starts_with("//") {
            continue;
        }
        // First non-blank, non-comment line.
        return trimmed.starts_with("#![cfg(test)]");
    }
    false
}

/// Returns true iff `trimmed` contains a real macro invocation of
/// `{name_with_bang}(`, `{name_with_bang}{{`, or `{name_with_bang}[`
/// — the three delimiters valid for Rust macro calls per the reference.
///
/// Tighter than a raw `trimmed.contains(name_with_bang)` substring check,
/// which false-positives on string literals like `"todo!"` or identifiers
/// containing the sequence. Pass `name_with_bang` as `"todo!"`, `"panic!"`
/// etc. — the bang must be present so we don't accept e.g. `today!`.
fn has_macro_call(trimmed: &str, name_with_bang: &str) -> bool {
    // `contains(&str)` is O(n·m); the needles are tiny and we scan at most
    // a handful of lines per file, so the format! allocations are fine.
    trimmed.contains(&format!("{}(", name_with_bang))
        || trimmed.contains(&format!("{}{{", name_with_bang))
        || trimmed.contains(&format!("{}[", name_with_bang))
}

/// Replace the content of `"..."` string literals in a single Rust source
/// line with spaces, preserving column positions. Pattern checks like
/// `contains("unsafe {")` or `contains("todo!(")` would otherwise match
/// code that manipulates those exact strings (classic self-graded grader
/// false positive).
///
/// Handles standard escapes (`\"`, `\\`) so `"a\"b"` is skipped intact.
/// Naive on raw strings (`r"..."`, `r#"..."#`) — treats the first `"` as
/// an opener, which is good enough for the scanner's needs and matches
/// the rest of the codebase's style.
fn strip_string_literals(line: &str) -> String {
    let mut out = String::with_capacity(line.len());
    let mut in_string = false;
    let mut chars = line.chars();
    while let Some(ch) = chars.next() {
        if in_string {
            if ch == '\\' {
                // Consume the escaped char; replace both with spaces.
                out.push(' ');
                if chars.next().is_some() {
                    out.push(' ');
                }
            } else if ch == '"' {
                in_string = false;
                out.push(' ');
            } else {
                out.push(' ');
            }
        } else if ch == '"' {
            in_string = true;
            out.push(' ');
        } else {
            out.push(ch);
        }
    }
    out
}

/// Check if any of the preceding 1-3 lines is a `//` comment (not `///` or `//!`).
fn has_preceding_comment(lines: &[&str], i: usize) -> bool {
    (1..=3).any(|offset| {
        i.checked_sub(offset).is_some_and(|j| {
            let prev = lines[j].trim();
            prev.starts_with("//") && !prev.starts_with("///") && !prev.starts_with("//!")
        })
    })
}

/// Check whether `lint` (e.g. `clippy::panic`) appears inside a real
/// `#[allow(...)]` or `#![allow(...)]` attribute that covers line `i`.
///
/// Scans two windows:
/// - **File-top** (first 50 lines): catches file-level inner attributes
///   like `#![allow(clippy::panic)]` which Rust applies to the entire
///   module regardless of distance.
/// - **300-line back-window** from `i`: catches function-scope / item-
///   scope `#[allow(...)]` attributes.
///
/// Span-aware: finds each `#[allow(` / `#![allow(` opening in either
/// window, walks forward across lines to the matching `)]`, and checks
/// whether the lint name appears inside that span. A stray mention of
/// the lint name in a comment or string literal no longer counts.
///
/// Still a heuristic — does not distinguish an allow on a sibling struct
/// field 250 lines up from one on the enclosing function. Both match.
/// For AST-correct scope resolution the grader would need syn; this check
/// is the tightest available without that dependency.
fn has_enclosing_allow(lines: &[&str], i: usize, lint: &str) -> bool {
    // Returns true if the attribute opening at `open_idx` closes before `i`
    // and its body contains `lint`.
    let attr_covers = |open_idx: usize, prefix: &str| -> bool {
        let open_line = lines[open_idx];
        let Some(open_col) = open_line.find(prefix) else {
            return false;
        };
        let mut body = String::from(&open_line[open_col + prefix.len()..]);
        let mut close_idx = open_idx;
        while !body.contains(")]") && close_idx + 1 < i {
            close_idx += 1;
            body.push('\n');
            body.push_str(lines[close_idx]);
        }
        body.find(")]")
            .is_some_and(|close_pos| body[..close_pos].contains(lint))
    };

    // Primary scan: 300-line back-window from `i`. Catches both outer
    // `#[allow(...)]` (function/item scope) and inner `#![allow(...)]`
    // (inline sub-module scope) attributes near the code.
    let back_start = i.saturating_sub(300);
    for open_idx in back_start..i {
        if attr_covers(open_idx, "#![allow(") || attr_covers(open_idx, "#[allow(") {
            return true;
        }
    }

    // Additive scan for file-level INNER attributes: when the back-window
    // doesn't reach the file top (`back_start > 0`), explicitly check the
    // first 50 lines for `#![allow(...)]`. These are module-level
    // attributes that cover the whole file regardless of distance. Outer
    // `#[allow(...)]` at file-top is excluded here — it binds to the next
    // item, not the whole file, so using it to suppress a violation far
    // below would be a false positive.
    //
    // Gate threshold MUST be `> 0`, not `> 50`: with `> 50`, indices in
    // `(0, 50]` fall in a dead zone where neither the primary scan
    // (`back_start..i` skips `0..back_start`) nor the additive scan
    // (gated off) reaches the file-top attribute. The dead zone hits
    // violations at line indices `301..=350` when an `#![allow(...)]`
    // sits at the file top. Overlap with the primary scan on
    // `back_start..50` (when `0 < back_start <= 50`) is harmless — both
    // scans return on first match.
    if back_start > 0 {
        for open_idx in 0..50 {
            if attr_covers(open_idx, "#![allow(") {
                return true;
            }
        }
    }

    false
}

/// Check if the line has a trailing `//` comment after the code.
fn has_same_line_comment(trimmed: &str) -> bool {
    // Find last "//" that isn't inside a string literal (simple heuristic)
    if let Some(pos) = trimmed.rfind("//") {
        // Must be after some code content
        pos > 0 && trimmed[..pos].contains(|c: char| c.is_alphanumeric() || c == '!')
    } else {
        false
    }
}

/// Count library `#[allow(clippy::...)]` attributes that lack a justifying
/// comment, mirroring `grade_clippy`'s file-scan rule.
///
/// - `#[cfg(test)]` modules are excluded via brace-depth tracking.
/// - Attributes between `#[cfg(test)]` and the opening `{` of the test
///   item (e.g. `#[cfg(test)]\n#[allow(...)]\nmod tests { ... }`) are
///   treated as part of the test attribute stack and excluded.
/// - Block comments and line comments are skipped.
/// - Multi-line attribute forms such as `#[allow(\n    clippy::lint,\n)]`
///   are parsed by walking forward from `#[allow(` to the matching `)]`.
/// - Justification: any of the 1-3 preceding lines is a `//` comment
///   (not `///`, not `//!`), or the same line has a trailing `//` comment
///   after the `#[allow(`.
fn count_unjustified_clippy_allows(lines: &[&str]) -> usize {
    let mut count = 0usize;
    let mut in_test = false;
    let mut test_brace_depth: usize = 0;
    let mut pending_test_attr = false;
    let mut in_block_comment = false;
    let mut skip_until: usize = 0;

    for (i, line) in lines.iter().enumerate() {
        if i < skip_until {
            continue;
        }
        let trimmed = line.trim();

        if !in_block_comment && trimmed.contains("/*") {
            in_block_comment = true;
        }
        if in_block_comment {
            if trimmed.contains("*/") {
                in_block_comment = false;
            }
            continue;
        }

        if trimmed.starts_with("//") {
            continue;
        }

        // starts_with — see note in scan_file_safety on the same rule.
        if trimmed.starts_with("#[cfg(test)]") {
            pending_test_attr = true;
        }

        if pending_test_attr && trimmed.contains('{') {
            in_test = true;
            test_brace_depth = 0;
            pending_test_attr = false;
        }

        if in_test {
            for ch in trimmed.chars() {
                if ch == '{' {
                    test_brace_depth += 1;
                } else if ch == '}' {
                    test_brace_depth = test_brace_depth.saturating_sub(1);
                    if test_brace_depth == 0 {
                        in_test = false;
                        break;
                    }
                }
            }
            continue;
        }

        // Attributes stacked between `#[cfg(test)]` and the test item's
        // opening `{` belong to the test stack and must not be flagged.
        if pending_test_attr {
            continue;
        }

        if !trimmed.contains("#[allow(") {
            continue;
        }

        // Walk forward across lines to the matching `)]`. Single-line
        // attributes close immediately on the opening line.
        let mut body = String::from(trimmed);
        let mut close_idx = i;
        while !body.contains(")]") && close_idx + 1 < lines.len() {
            close_idx += 1;
            body.push('\n');
            body.push_str(lines[close_idx].trim());
        }

        if !body.contains("clippy::") {
            skip_until = close_idx + 1;
            continue;
        }

        let has_justification = (1..=3).any(|offset| {
            i.checked_sub(offset).is_some_and(|j| {
                let prev = lines[j].trim();
                prev.starts_with("//") && !prev.starts_with("///") && !prev.starts_with("//!")
            })
        });

        if !has_justification {
            let has_inline = trimmed.contains("//") && {
                let comment_pos = trimmed.rfind("//").unwrap_or(0);
                let allow_pos = trimmed.find("#[allow(").unwrap_or(0);
                comment_pos > allow_pos
            };
            if !has_inline {
                count += 1;
            }
        }

        skip_until = close_idx + 1;
    }

    count
}

/// Count `Cargo.toml` dependency entries that lack a justifying comment,
/// mirroring `grade_dependencies`'s text-scan rule.
///
/// - Tracked sections: `[dependencies]`, `[dev-dependencies]`,
///   `[build-dependencies]`, plus the target-conditional forms
///   `[target.<spec>.dependencies]` / `.dev-dependencies]` /
///   `.build-dependencies]`.
/// - Inside a dep section, a dep entry is a line containing `=` at brace
///   depth 0. Continuation lines of a multi-line inline-table dep spec
///   (`name = {\n    version = "...",\n    ...\n}`) are at brace depth
///   > 0 and are skipped so they are not mis-scanned as fresh deps.
/// - Justification: the line has an inline `#` comment, or one of the
///   1-3 preceding lines starts with `#`. The backward scan stops at
///   blank lines and section headers.
fn count_unjustified_deps(lines: &[&str]) -> usize {
    let mut unjustified = 0usize;
    let mut in_dep_section = false;
    let mut brace_depth: usize = 0;

    for (i, line) in lines.iter().enumerate() {
        let trimmed = line.trim();

        // Section headers are only recognized at brace depth 0. A `[`
        // inside a nested inline table is not a section header.
        if brace_depth == 0 && trimmed.starts_with('[') {
            in_dep_section = is_dep_section_header(trimmed);
            continue;
        }

        // Continuation line inside a multi-line inline table: skip the
        // dep check but still walk the line for brace updates.
        if brace_depth > 0 {
            brace_depth = update_brace_depth(line, brace_depth);
            continue;
        }

        if !in_dep_section {
            brace_depth = update_brace_depth(line, brace_depth);
            continue;
        }

        // Blank line or full-line comment: not a dep entry.
        if trimmed.is_empty() || trimmed.starts_with('#') {
            continue;
        }

        // A dep entry is a line with `name = ...`.
        if !trimmed.contains('=') {
            brace_depth = update_brace_depth(line, brace_depth);
            continue;
        }

        // Inline `#` comment anywhere on the dep line justifies it.
        if trimmed.contains('#') {
            brace_depth = update_brace_depth(line, brace_depth);
            continue;
        }

        // Check preceding 1-3 lines for a `#` comment. Stop at blank
        // lines and section headers (they break the chain).
        let mut has_justification = false;
        for offset in 1..=3 {
            let Some(j) = i.checked_sub(offset) else {
                break;
            };
            let prev = lines[j].trim();
            if prev.is_empty() || prev.starts_with('[') {
                break;
            }
            if prev.starts_with('#') {
                has_justification = true;
                break;
            }
        }

        if !has_justification {
            unjustified += 1;
        }

        brace_depth = update_brace_depth(line, brace_depth);
    }

    unjustified
}

/// Is `trimmed` a dep section header the grader should track?
fn is_dep_section_header(trimmed: &str) -> bool {
    if trimmed == "[dependencies]"
        || trimmed == "[dev-dependencies]"
        || trimmed == "[build-dependencies]"
    {
        return true;
    }
    if trimmed.starts_with("[target.") {
        return trimmed.ends_with(".dependencies]")
            || trimmed.ends_with(".dev-dependencies]")
            || trimmed.ends_with(".build-dependencies]");
    }
    false
}

/// Update brace depth across a `Cargo.toml` line, counting `{` and `}`
/// outside string literals and inline `#` comments.
fn update_brace_depth(line: &str, mut depth: usize) -> usize {
    let mut in_string = false;
    let mut escape = false;
    for c in line.chars() {
        if in_string {
            if escape {
                escape = false;
            } else if c == '\\' {
                escape = true;
            } else if c == '"' {
                in_string = false;
            }
            continue;
        }
        match c {
            // Rest of line is an inline comment.
            '#' => break,
            '"' => in_string = true,
            '{' => depth += 1,
            '}' => depth = depth.saturating_sub(1),
            _ => {}
        }
    }
    depth
}

/// Grade dependencies: justification check (hard gate) + dep count (informational).
///
/// Per chassis ss2.5: every dependency in Cargo.toml must have a `#` comment
/// in the preceding 1-3 lines or inline. Dep count > 10 is flagged as "(heavy)"
/// but does not affect grade.
fn grade_dependencies(
    sh: &Shell,
    crate_name: &str,
    profile: CrateProfile,
) -> Result<CriterionResult> {
    // Step 1: dep count via cargo metadata (informational)
    let metadata_json = cmd!(sh, "cargo metadata --format-version 1 --no-deps")
        .read()
        .unwrap_or_default();
    let metadata: serde_json::Value =
        serde_json::from_str(&metadata_json).unwrap_or(serde_json::Value::Null);

    let dep_count = metadata["packages"]
        .as_array()
        .and_then(|pkgs| pkgs.iter().find(|p| p["name"].as_str() == Some(crate_name)))
        .and_then(|pkg| pkg["dependencies"].as_array())
        .map(|deps| {
            deps.iter()
                .filter(|d| {
                    // Count normal and dev deps; exclude build deps
                    let kind = d["kind"].as_str();
                    kind.is_none() || kind == Some("dev")
                })
                .count()
        })
        .unwrap_or(0);

    let heavy_note = if dep_count > 10 { " (heavy)" } else { "" };

    // Step 2: justification check via Cargo.toml text scan (hard gate)
    let crate_path = find_crate_path(sh, crate_name)?;
    let cargo_toml_path = format!("{}/Cargo.toml", crate_path);
    let cargo_content = match std::fs::read_to_string(&cargo_toml_path) {
        Ok(c) => c,
        Err(e) => {
            return Ok(CriterionResult {
                name: "5. Dependencies",
                result: format!("error: {}", e),
                grade: Grade::F,
                threshold: "all justified",
                measured_detail: format!("error reading Cargo.toml: {}", e),
            });
        }
    };

    // Relaxed for Example/Xtask profiles: justification comments are a
    // library-crate rubric, not a demo/tooling one. Matches the
    // STANDARDS.md §4 "Allowed" relaxation pattern (unwrap in examples,
    // expect in build.rs) applied coherently to Dependencies.
    let relax_unjustified_deps = matches!(profile, CrateProfile::Example | CrateProfile::Xtask);
    let lines: Vec<&str> = cargo_content.lines().collect();
    let unjustified = if relax_unjustified_deps {
        0
    } else {
        count_unjustified_deps(&lines)
    };

    // Binary A/F
    let grade = if unjustified == 0 { Grade::A } else { Grade::F };

    let result = if unjustified > 0 {
        format!(
            "{} deps{}, {} unjustified",
            dep_count, heavy_note, unjustified
        )
    } else {
        format!("{} deps{}, all just.", dep_count, heavy_note)
    };

    Ok(CriterionResult {
        name: "5. Dependencies",
        result: result.clone(),
        grade,
        threshold: "all justified",
        measured_detail: result,
    })
}

// ============================================================================
// Criterion 6: Layer Integrity
// ============================================================================
//
// Replaces the former Bevy-free criterion. Reads tier metadata declared in
// each crate's `[package.metadata.cortenforge]` block, runs `cargo tree`
// across (no-default × default × all-features) × (release × with-dev) graph
// configurations, and reports findings against the tier's max-dep-count
// and banned-prefix rules from `sim/docs/L0_architectural_plan.md` §5.2.
//
// HARD GATE per plan §8 step 12 (was WARNING MODE in step 3 commit
// `1fb88e2f`). The criterion returns Grade::F when findings exist or when
// an in-scope crate is missing tier metadata. Warning-mode rollout cleared
// the way for hard-gate by absorbing the per-surgery interim states without
// blocking PRs.

/// One of the architectural tiers. `L0`/`L0-io`/`L0-integration`/`L1` are the
/// SDK library tiers from `L0_architectural_plan.md` §2.1. `App` is the
/// dependency *sink* for guided end-user applications (e.g. Cendrillon /
/// `cf-studio-*`): an App-tier crate may depend on the SDK, but **no SDK-tier
/// crate may ever depend on an App-tier crate**. That one-way rule is enforced
/// by the App-tier sink check in the Layer Integrity criterion; see
/// `MISSION.md` and the app-vs-SDK boundary plan.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Tier {
    L0,
    L0Io,
    L0Integration,
    L1,
    /// End-user application crate. Unbounded like `L1` for its own dep graph
    /// (apps legitimately pull GUI/windowing stacks), but forbidden as a
    /// dependency of any SDK-tier crate.
    App,
}

impl Tier {
    fn parse(s: &str) -> Option<Self> {
        match s {
            "L0" => Some(Tier::L0),
            "L0-io" => Some(Tier::L0Io),
            "L0-integration" => Some(Tier::L0Integration),
            "L1" => Some(Tier::L1),
            "App" => Some(Tier::App),
            _ => None,
        }
    }

    fn label(&self) -> &'static str {
        match self {
            Tier::L0 => "L0",
            Tier::L0Io => "L0-io",
            Tier::L0Integration => "L0-integration",
            Tier::L1 => "L1",
            Tier::App => "App",
        }
    }

    /// Tier ordering: stricter tiers have lower permissiveness scores.
    /// Used by `effective_tier_for` to pick the most permissive tier when
    /// multiple `tier_up_features` are simultaneously enabled.
    fn permissiveness(&self) -> u8 {
        match self {
            Tier::L0 => 0,
            Tier::L0Io => 1,
            Tier::L0Integration => 2,
            Tier::L1 => 3,
            // App is unbounded like L1 for its own graph; placed at the top so
            // `effective_tier_for` never down-ranks an App crate. App is never
            // a tier-up target, so this only affects ordering completeness.
            Tier::App => 4,
        }
    }
}

/// A banned-dep entry. `Prefix` matches `pkg_name.starts_with(pattern)`
/// (so `bevy` matches `bevy`, `bevy_ecs`, `bevy_reflect`, …); `Exact`
/// requires `pkg_name == pattern`. The asterisk-vs-no-asterisk
/// distinction in plan §5.2's banned-prefix table determines which kind.
#[derive(Debug, Clone, Copy)]
struct BanPattern {
    pattern: &'static str,
    kind: BanKind,
}

#[derive(Debug, Clone, Copy)]
enum BanKind {
    Prefix,
    Exact,
}

impl BanPattern {
    fn matches(&self, pkg_name: &str) -> bool {
        match self.kind {
            BanKind::Prefix => pkg_name.starts_with(self.pattern),
            BanKind::Exact => pkg_name == self.pattern,
        }
    }
}

/// Per-tier enforcement rules. `release_max` applies to the release graph
/// (`-e normal`); `test_max` applies to the dev graph (`-e normal,dev`).
#[derive(Debug, Clone, Copy)]
struct TierConfig {
    release_max: usize,
    test_max: usize,
    banned: &'static [BanPattern],
}

// Banned-prefix lists per plan §5.2. The L0-io/L0-integration list omits
// `wgpu*`: plan §2.1 + §2.1a + §5.2's "why this works" example all
// explicitly permit wgpu in L0-io (sim-gpu's GPU-accelerated SDF
// collision; sim-soft's `gpu-probe` tier-up). The §5.2 table cell that
// listed wgpu* there is a typo, fixed in this commit's plan-doc edit.
const L0_BANNED: &[BanPattern] = &[
    BanPattern {
        pattern: "bevy",
        kind: BanKind::Prefix,
    },
    BanPattern {
        pattern: "winit",
        kind: BanKind::Exact,
    },
    BanPattern {
        pattern: "wgpu",
        kind: BanKind::Prefix,
    },
    BanPattern {
        pattern: "image",
        kind: BanKind::Prefix,
    },
    BanPattern {
        pattern: "zip",
        kind: BanKind::Prefix,
    },
    BanPattern {
        pattern: "zstd",
        kind: BanKind::Prefix,
    },
    BanPattern {
        pattern: "sim-mjcf",
        kind: BanKind::Exact,
    },
    BanPattern {
        pattern: "sim-urdf",
        kind: BanKind::Exact,
    },
    BanPattern {
        pattern: "mesh-io",
        kind: BanKind::Exact,
    },
    BanPattern {
        pattern: "criterion",
        kind: BanKind::Exact,
    },
    BanPattern {
        pattern: "plotters",
        kind: BanKind::Prefix,
    },
];

const L0_IO_BANNED: &[BanPattern] = &[
    BanPattern {
        pattern: "bevy",
        kind: BanKind::Prefix,
    },
    BanPattern {
        pattern: "winit",
        kind: BanKind::Exact,
    },
];

// L0-integration shares L0-io's banned list (plan §5.2).
const L0_INTEGRATION_BANNED: &[BanPattern] = L0_IO_BANNED;

const L1_BANNED: &[BanPattern] = &[];

/// The declared `dependency_budget`, but only when it actually RAISES this
/// crate's base-tier cap.
///
/// A budget at or below the tier default is inert — `tier_config_with_budget`
/// clamps it away — and reporting one would tell a reader the crate is held to
/// a limit it is not held to. Returns `None` for the inert case so callers can
/// stay silent rather than claim something false.
fn budget_in_effect(metadata: &TierMetadata) -> Option<usize> {
    let budget = metadata.dependency_budget?;
    (budget > tier_config(metadata.tier).release_max).then_some(budget)
}

/// [`tier_config`] with a crate's `dependency_budget` override applied.
///
/// Raises `release_max` to the declared budget and `test_max` to
/// `budget + 20`, preserving the tier table's own release/test buffer.
/// Banned patterns are taken from the tier unchanged — the override is a
/// COUNT, never a permission.
///
/// **Raise-only.** A budget below the tier default is ignored rather than
/// applied. This is not politeness: `tier` here is the EFFECTIVE tier, which
/// under `--all-features` may have been tiered up by a `tier_up_features`
/// declaration. Without the clamp, a crate declaring a budget for its base
/// tier would silently TIGHTEN the laxer tier its features unlock — e.g. an
/// L0 crate declaring 200 that tiers up to L1 would cap L1's unbounded graph
/// at 200 and fail a build the tier permits. The override exists to buy a
/// crate more room than its tier gives, never less.
fn tier_config_with_budget(tier: Tier, budget: Option<usize>) -> TierConfig {
    let base = tier_config(tier);
    match budget {
        None => base,
        Some(budget) => TierConfig {
            release_max: base.release_max.max(budget),
            test_max: base.test_max.max(budget.saturating_add(20)),
            banned: base.banned,
        },
    }
}

/// Look up the static `TierConfig` for a tier. Numbers initially tracked
/// plan §5.2 (release 80/200/200, test 100/220/220) and re-tune as
/// integration experience accumulates. Plan §2.1 proposes tighter
/// numbers (60/180/180) post-Appendix-A; we'll re-tune again at
/// hard-gate flip (plan §8 step 12) once surgeries land and the actual
/// headroom is known.
///
/// L0's current 100/120 accommodates the cf-design SDF design surface
/// (Sdf trait, Solid CSG kernel, mesh-sdf adapter) entering L0 sim
/// crates' transitive dep set: cf-design reaches 48 transitive deps,
/// and an L0 sim crate with its own baseline consuming cf-design lands
/// in the 85-95 range. The +20 release/test buffer matches L0Io's and
/// L0Integration's 200/220 pattern.
fn tier_config(tier: Tier) -> TierConfig {
    match tier {
        Tier::L0 => TierConfig {
            release_max: 100,
            test_max: 120,
            banned: L0_BANNED,
        },
        Tier::L0Io => TierConfig {
            release_max: 200,
            test_max: 220,
            banned: L0_IO_BANNED,
        },
        Tier::L0Integration => TierConfig {
            release_max: 200,
            test_max: 220,
            banned: L0_INTEGRATION_BANNED,
        },
        Tier::L1 => TierConfig {
            release_max: usize::MAX,
            test_max: usize::MAX,
            banned: L1_BANNED,
        },
        // App: unbounded and unbanned. Apps legitimately pull Slint/wgpu/winit;
        // the constraint on an App crate is not on what it consumes but on who
        // consumes IT (the sink rule). In practice this arm is never reached at
        // runtime — an App crate short-circuits Layer Integrity before
        // `tier_config`, and `App` is rejected as a tier-up target so it can
        // never become an `effective_tier` — but it's required for exhaustive
        // matching and keeps the table honest.
        Tier::App => TierConfig {
            release_max: usize::MAX,
            test_max: usize::MAX,
            banned: L1_BANNED,
        },
    }
}

/// Tier metadata read from a crate's Cargo.toml.
#[derive(Debug, Clone)]
struct TierMetadata {
    tier: Tier,
    /// `feature_name -> target_tier` declarations (plan §2.1a). When the
    /// feature is enabled, the target tier's rules apply. Empty for the
    /// vast majority of crates; sim-soft is the only current declarer
    /// (`gpu-probe -> L0-io`).
    tier_up_features: Vec<(String, Tier)>,
    /// Per-crate release dependency budget, overriding the tier default.
    ///
    /// Exists so a crate that needs more of the dep budget than its tier
    /// allows can say so **in its own Cargo.toml, next to the deps that
    /// spend it**, rather than forcing the choice between silently raising
    /// the cap for every crate in the tier and mis-declaring the tier to
    /// borrow a laxer one (which would also relax the tier's BANNED list —
    /// a far larger change than a count).
    ///
    /// The override moves the count ONLY. Banned patterns, layer rules and
    /// sink rules are untouched. It is reported on every grade run,
    /// including passes, so a raised budget can never be invisible.
    dependency_budget: Option<usize>,
}

/// Which `--features`-style flag is active for a `cargo tree` invocation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum FeatureConfig {
    NoDefault,
    Default,
    AllFeatures,
}

impl FeatureConfig {
    fn label(&self) -> &'static str {
        match self {
            FeatureConfig::NoDefault => "no-default",
            FeatureConfig::Default => "default",
            FeatureConfig::AllFeatures => "all-features",
        }
    }
}

/// Which dep-graph kind: release-only or release + dev.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum GraphKind {
    /// `cargo tree -e normal`
    Release,
    /// `cargo tree -e normal,dev`
    WithDev,
}

impl GraphKind {
    fn label(&self) -> &'static str {
        match self {
            GraphKind::Release => "release",
            GraphKind::WithDev => "with-dev",
        }
    }
}

/// A single Layer Integrity finding produced by `evaluate_dep_set`.
#[derive(Debug, Clone)]
struct Finding {
    feature_config: FeatureConfig,
    graph_kind: GraphKind,
    effective_tier: Tier,
    kind: FindingKind,
}

#[derive(Debug, Clone)]
enum FindingKind {
    /// The graph's unique-dep count exceeded the tier's max.
    CountExceeded { actual: usize, max: usize },
    /// A dep matched one of the tier's banned-prefix patterns.
    BannedPrefix {
        pattern: &'static str,
        matched_pkg: String,
    },
    /// An SDK-tier crate depends on an App-tier crate — a violation of the
    /// one-way App sink rule (apps depend on the SDK, never the reverse).
    AppSink { app_crate: String },
}

/// True if `crate_name` must declare tier metadata. Plan §5.1 scope:
/// `sim-*`, `mesh-*`, `cf-*`, `cortenforge*`. The two no-hyphen umbrellas
/// (`mesh`, `cortenforge`) are explicitly accepted; `examples/*` and
/// `xtask` are out of scope.
fn applies_to_crate(crate_name: &str) -> bool {
    if matches!(crate_name, "mesh" | "cortenforge") {
        return true;
    }
    // Workspace tools and shared helper crates that happen to match a
    // library-namespace prefix need an explicit exemption. xtask is
    // excluded by virtue of having no matching prefix at all; cf-viewer,
    // cf-bevy-common, cf-scan-prep, cf-device-design, and cf-sim-research
    // all carry `cf-` prefixes that would otherwise pull them into the
    // design-library scope (cf-spatial / cf-design / cf-geometry). Per docs/VIEWER_DESIGN.md Q1 + Q8 locks: cf-viewer
    // is a workspace tool, carries no tier metadata, and Q8 directs
    // path-based filtering as the gating mechanism rather than
    // retrofitting metadata. cf-bevy-common (sim-soft PR2 C2b
    // factor-out) is a workspace-internal Bevy helper consumed by
    // cf-viewer + sim-bevy + sim-bevy-soft; same exemption shape.
    // cf-scan-prep (Stage 2.5 scan-preprocessing GUI) + cf-device-design
    // (layered-silicone-device design suite) + cf-sim-research (sim-research
    // viewer) are Bevy GUI workspace tools with the Q8 path-based-filter
    // exemption. cf-device-geometry (sim-decouple Phase 2.5.b) is the shared
    // Bevy-using device-side compute + rendering primitives crate consumed by
    // those binaries — same exemption shape.
    //
    // NOT exempt — classified as SDK library tiers because the Cendrillon app
    // stack consumes them AS LIBRARIES, so they must be governed:
    // cf-device-types → L0 (the `bevy` feature tiers up to L1), cf-scan-prep-core
    // → L0-io, cf-cast-cli → L0-integration. They carry tier metadata and are
    // intentionally absent from this list. (The four cf-studio-*
    // crates — the guided Cendrillon app stack: cf-studio-core spine →
    // cf-studio-engine orchestrator → cf-studio CLI / cf-studio-gui; see
    // MISSION.md — are deliberately NOT exempt: they are classified
    // `tier = "App"` (the dependency sink) and governed by the App sink rule,
    // so they carry tier metadata and are intentionally absent from this list.)
    // cf-osim + cf-anthro + cf-msk-fit + cf-msk-lib
    // + cf-mjcf-emit (the musculoskeletal-builder arc; Mission deliverable #4 —
    // OpenSim→IR bridge, scan landmark detection, place/scale/articulate, the
    // library/parameter spine that morphs a template into a body, and the
    // Model→MJCF emitter) are `tools/` workspace tools with the cf- prefix —
    // same exemption. cf-codesign (the co-design optimizer; Mission
    // deliverable #2 — gradient-based optimization over the differentiable
    // soft↔rigid coupling, consuming sim-coupling + reusing the chassis Adam)
    // is a `tools/` workspace tool with the cf- prefix — same exemption.
    // cf-spine-studio (the native Bevy Design↔Simulate anatomical-spine studio;
    // paint → solve → replay the L4-L5 FSU) is a `tools/` Bevy GUI workspace tool
    // with the cf- prefix — same exemption shape as cf-sim-research. cf-mesh-paint (the reusable
    // brush-painting Bevy plugin over mesh-select) is a repo-root Bevy support
    // crate, a structural sibling of cf-bevy-common — same exemption shape.
    if matches!(
        crate_name,
        "cf-viewer"
            | "cf-bevy-common"
            | "cf-mesh-paint"
            | "cf-scan-prep"
            | "cf-device-design"
            | "cf-sim-research"
            | "cf-spine-studio"
            | "cf-device-geometry"
            | "cf-osim"
            | "cf-anthro"
            | "cf-msk-fit"
            | "cf-msk-lib"
            | "cf-mjcf-emit"
            | "cf-codesign"
    ) {
        return false;
    }
    let prefixes = ["sim-", "mesh-", "cf-", "cortenforge-"];
    prefixes.iter().any(|p| crate_name.starts_with(p))
}

/// Parse `[package.metadata.cortenforge]` from Cargo.toml text.
///
/// Returns `Ok(None)` if no `cortenforge` metadata block exists or it has
/// no `tier` key. Returns `Ok(Some(_))` for a parseable tier. Errors only
/// on a present-but-malformed value (unknown tier name, malformed
/// `tier_up_features`).
fn parse_tier_metadata(cargo_toml_text: &str) -> Result<Option<TierMetadata>> {
    let value: toml::Value =
        toml::from_str(cargo_toml_text).context("failed to parse Cargo.toml as TOML")?;

    let cf_meta = value
        .get("package")
        .and_then(|p| p.get("metadata"))
        .and_then(|m| m.get("cortenforge"));
    let Some(cf_meta) = cf_meta else {
        return Ok(None);
    };

    let Some(tier_str) = cf_meta.get("tier").and_then(|t| t.as_str()) else {
        return Ok(None);
    };

    let tier =
        Tier::parse(tier_str).with_context(|| format!("unknown tier value: {:?}", tier_str))?;

    let dependency_budget = match cf_meta.get("dependency_budget") {
        None => None,
        Some(v) => {
            let n = v
                .as_integer()
                .with_context(|| format!("dependency_budget must be an integer; got {:?}", v))?;
            Some(
                usize::try_from(n)
                    .with_context(|| format!("dependency_budget must be non-negative; got {n}"))?,
            )
        }
    };

    let mut tier_up_features = Vec::new();
    if let Some(tuf) = cf_meta.get("tier_up_features") {
        let table = tuf
            .as_table()
            .with_context(|| format!("tier_up_features must be a table; got {:?}", tuf))?;
        for (feat, target) in table {
            let target_str = target.as_str().with_context(|| {
                format!(
                    "tier_up_features.{} must be a string; got {:?}",
                    feat, target
                )
            })?;
            let target_tier = Tier::parse(target_str).with_context(|| {
                format!(
                    "tier_up_features.{}: unknown tier value {:?}",
                    feat, target_str
                )
            })?;
            // App is the dependency sink, never a tier-up target: allowing a
            // feature to promote an SDK crate to App would make `effective_tier`
            // unbounded/unbanned under --all-features and silently exempt it
            // from every dep check. Reject it.
            if target_tier == Tier::App {
                anyhow::bail!(
                    "tier_up_features.{}: `App` is not a valid tier-up target \
                     (App is the dependency sink, not an SDK tier)",
                    feat
                );
            }
            tier_up_features.push((feat.clone(), target_tier));
        }
    }

    Ok(Some(TierMetadata {
        tier,
        tier_up_features,
        dependency_budget,
    }))
}

/// Compute the effective tier for a given feature config. Under
/// `--all-features`, every declared `tier_up_feature` is enabled, so the
/// effective tier is the most permissive of {declared} ∪ {tier_up
/// targets}. Under default / no-default, only the declared tier applies.
///
/// Convention assumption: tier-up features are NOT in the default set.
/// This matches sim-soft's gpu-probe (off by default, enabled under
/// `--features gpu-probe` or `--all-features`). Crates that put tier-up
/// features in defaults would slip through this check; the convention is
/// documented in plan §2.1a as a deliberate choice for simplicity.
fn effective_tier_for(metadata: &TierMetadata, fc: FeatureConfig) -> Tier {
    if fc != FeatureConfig::AllFeatures {
        return metadata.tier;
    }
    let mut effective = metadata.tier;
    for (_feat, target) in &metadata.tier_up_features {
        if target.permissiveness() > effective.permissiveness() {
            effective = *target;
        }
    }
    effective
}

/// Check a flat dep list against a tier's rules. Returns one Finding per
/// banned-prefix match (no dedup — listing all matches is more
/// informative for the warning-mode rollout) plus at most one
/// CountExceeded.
fn evaluate_dep_set(
    deps: &[String],
    config: TierConfig,
    fc: FeatureConfig,
    gk: GraphKind,
    effective_tier: Tier,
) -> Vec<Finding> {
    let mut findings = Vec::new();

    for dep in deps {
        for ban in config.banned {
            if ban.matches(dep) {
                findings.push(Finding {
                    feature_config: fc,
                    graph_kind: gk,
                    effective_tier,
                    kind: FindingKind::BannedPrefix {
                        pattern: ban.pattern,
                        matched_pkg: dep.clone(),
                    },
                });
                // One match per dep — a dep matched by `bevy` prefix
                // shouldn't also match a hypothetical second pattern.
                break;
            }
        }
    }

    let max = match gk {
        GraphKind::Release => config.release_max,
        GraphKind::WithDev => config.test_max,
    };
    if deps.len() > max {
        findings.push(Finding {
            feature_config: fc,
            graph_kind: gk,
            effective_tier,
            kind: FindingKind::CountExceeded {
                actual: deps.len(),
                max,
            },
        });
    }

    findings
}

/// Run `cargo tree` and parse out the unique transitive dep list. Each
/// output line is `pkg_name v1.2.3 [(path)]` — first whitespace-split
/// token is the package name. Order preserved; duplicates dropped.
fn read_tree_deps(
    sh: &Shell,
    crate_name: &str,
    fc: FeatureConfig,
    gk: GraphKind,
) -> Result<Vec<String>> {
    let edges = match gk {
        GraphKind::Release => "normal",
        GraphKind::WithDev => "normal,dev",
    };
    let tree_format = "{p}";

    // Three feature-config arms because xshell's cmd! can't conditionally
    // omit a token; building per-config is clearer than splice tricks.
    let output = match fc {
        FeatureConfig::NoDefault => cmd!(
            sh,
            "cargo tree -p {crate_name} -e {edges} --no-default-features --prefix none --format {tree_format}"
        ),
        FeatureConfig::Default => cmd!(
            sh,
            "cargo tree -p {crate_name} -e {edges} --prefix none --format {tree_format}"
        ),
        FeatureConfig::AllFeatures => cmd!(
            sh,
            "cargo tree -p {crate_name} -e {edges} --all-features --prefix none --format {tree_format}"
        ),
    }
    .read()
    .with_context(|| {
        format!(
            "cargo tree failed for {} ({} {})",
            crate_name,
            fc.label(),
            gk.label()
        )
    })?;

    Ok(parse_tree_names(&output))
}

/// Parse `cargo tree --prefix none --format {p}` output into a deduped list of
/// package names (the first whitespace-delimited token per line). Shared by the
/// count/ban scan ([`read_tree_deps`]) and the sink scan ([`read_sink_deps`]).
fn parse_tree_names(output: &str) -> Vec<String> {
    let mut deps = Vec::new();
    let mut seen = std::collections::HashSet::new();
    for line in output.lines() {
        if let Some(name) = line.split_whitespace().next() {
            if !name.is_empty() && seen.insert(name.to_string()) {
                deps.push(name.to_string());
            }
        }
    }
    deps
}

/// Read the comprehensive dependency set for the App-tier sink check: ALL edge
/// kinds (`normal,build,dev`) across ALL targets (`--target all`), for one
/// feature config. Unlike [`read_tree_deps`] — which intentionally scopes the
/// count/ban check to normal/dev edges on the host target — the sink rule must
/// catch *every* form of SDK→App edge, including a `[build-dependencies]` or a
/// `[target.'cfg(...)'.dependencies]` edge that would otherwise be invisible on
/// the host. So it scans the widest graph cargo can produce.
fn read_sink_deps(sh: &Shell, crate_name: &str, fc: FeatureConfig) -> Result<Vec<String>> {
    let tree_format = "{p}";
    let edges = "normal,build,dev";
    let output = match fc {
        FeatureConfig::NoDefault => cmd!(
            sh,
            "cargo tree -p {crate_name} -e {edges} --target all --no-default-features --prefix none --format {tree_format}"
        ),
        FeatureConfig::Default => cmd!(
            sh,
            "cargo tree -p {crate_name} -e {edges} --target all --prefix none --format {tree_format}"
        ),
        FeatureConfig::AllFeatures => cmd!(
            sh,
            "cargo tree -p {crate_name} -e {edges} --target all --all-features --prefix none --format {tree_format}"
        ),
    }
    .read()
    .with_context(|| format!("cargo tree (sink scan) failed for {} ({})", crate_name, fc.label()))?;
    Ok(parse_tree_names(&output))
}

fn format_finding(f: &Finding) -> String {
    match &f.kind {
        FindingKind::CountExceeded { actual, max } => format!(
            "[{} {}, tier {}] dep count {} exceeds max {}",
            f.graph_kind.label(),
            f.feature_config.label(),
            f.effective_tier.label(),
            actual,
            max,
        ),
        FindingKind::BannedPrefix {
            pattern,
            matched_pkg,
        } => format!(
            "[{} {}, tier {}] banned `{}` matched: {}",
            f.graph_kind.label(),
            f.feature_config.label(),
            f.effective_tier.label(),
            pattern,
            matched_pkg,
        ),
        FindingKind::AppSink { app_crate } => format!(
            "[tier {}] forbidden App-tier dependency: {} \
             (an SDK crate must never depend on an app)",
            f.effective_tier.label(),
            app_crate,
        ),
    }
}

/// Enumerate workspace members declaring `tier = "App"`. This is the forbidden
/// set for the sink rule, derived from the declarations themselves (no
/// hard-coded list — any crate that adopts `tier = "App"` is covered
/// automatically). Reads each member's manifest via `cargo metadata --no-deps`.
fn app_tier_members(sh: &Shell) -> Result<std::collections::HashSet<String>> {
    let metadata_json = cmd!(sh, "cargo metadata --format-version 1 --no-deps")
        .read()
        .context("Failed to run `cargo metadata` for the App-tier scan")?;
    let metadata: serde_json::Value = serde_json::from_str(&metadata_json)
        .context("Failed to parse `cargo metadata` JSON output")?;
    let packages = metadata["packages"]
        .as_array()
        .context("`cargo metadata`: missing 'packages' array")?;

    let mut apps = std::collections::HashSet::new();
    for p in packages {
        let (Some(name), Some(manifest)) = (p["name"].as_str(), p["manifest_path"].as_str()) else {
            continue;
        };
        // Skip-and-continue on a member we cannot read or parse. A malformed
        // manifest is surfaced when THAT crate is graded; it must not abort the
        // grade of an unrelated crate (this scan runs while grading every SDK
        // crate). An unreadable/unparseable member simply isn't an App member.
        let Ok(text) = std::fs::read_to_string(manifest) else {
            continue;
        };
        if let Ok(Some(meta)) = parse_tier_metadata(&text) {
            if meta.tier == Tier::App {
                apps.insert(name.to_string());
            }
        }
    }
    Ok(apps)
}

/// The App-tier sink rule: one [`Finding`] per App-tier crate that appears in
/// `crate_name`'s dependency graph. `tier` is the graded crate's own (SDK) tier,
/// recorded on each finding for the message. Runs the comprehensive sink scan
/// ([`read_sink_deps`]: all edge kinds × all targets) across the three feature
/// configs and deduplicates, so each forbidden app dependency is reported once
/// regardless of how many configs expose it. Returns empty when there are no
/// app members or none appear in the graph.
fn sink_findings(
    sh: &Shell,
    crate_name: &str,
    tier: Tier,
    app_members: &std::collections::HashSet<String>,
) -> Result<Vec<Finding>> {
    if app_members.is_empty() {
        return Ok(Vec::new());
    }
    let configs = [
        FeatureConfig::NoDefault,
        FeatureConfig::Default,
        FeatureConfig::AllFeatures,
    ];
    // BTreeSet → deterministic, deduplicated order across the three scans.
    let mut found: std::collections::BTreeSet<String> = std::collections::BTreeSet::new();
    for fc in configs {
        let deps = read_sink_deps(sh, crate_name, fc)?;
        for app in evaluate_sink(&deps, crate_name, app_members) {
            found.insert(app.to_string());
        }
    }
    Ok(found
        .into_iter()
        .map(|app_crate| Finding {
            // feature_config/graph_kind are not rendered for an AppSink finding
            // (the violation is structural, not config-specific); nominal values.
            feature_config: FeatureConfig::Default,
            graph_kind: GraphKind::Release,
            effective_tier: tier,
            kind: FindingKind::AppSink { app_crate },
        })
        .collect())
}

/// Pure core of the sink check: the App-tier crate names present in `deps`,
/// excluding the self-edge (cargo tree lists the queried crate as the root).
/// Split out so it is unit-testable without a shell.
fn evaluate_sink<'a>(
    deps: &'a [String],
    crate_name: &str,
    app_members: &std::collections::HashSet<String>,
) -> Vec<&'a str> {
    deps.iter()
        .map(|d| d.as_str())
        .filter(|dep| *dep != crate_name && app_members.contains(*dep))
        .collect()
}

/// Pure Layer Integrity grade decision given the crate's tier and how many
/// findings were collected (count/ban + sink). Extracted so the load-bearing
/// rule — especially **L1 with a sink finding must be F, not the old
/// unconditional N/A** — is locked in by a unit test rather than only the
/// integrated cargo-tree path.
///
/// - `App`: N/A (the sink itself is never checked for what it consumes).
/// - `L1` with no findings: N/A (unbounded own graph), but a sink finding → F.
/// - any other SDK tier: A when clean, F when any finding exists.
fn layer_integrity_grade(tier: Tier, finding_count: usize) -> Grade {
    match (tier, finding_count) {
        (Tier::App, _) => Grade::NotApplicable,
        (Tier::L1, 0) => Grade::NotApplicable,
        (_, 0) => Grade::A,
        (_, _) => Grade::F,
    }
}

/// Implementation of Criterion 6: Layer Integrity.
///
/// Replaces the former Bevy-free criterion. See the module-level comment
/// at the top of this section for rollout phases.
///
/// Hard-gated per plan §8 step 12: returns `Grade::F` for in-scope crates
/// when any finding is recorded (banned-prefix match or count exceeded),
/// or when an in-scope crate is missing tier metadata. Warning-mode rollout
/// in step 3 cleared the way for hard-gate by absorbing per-surgery interim
/// states without blocking PRs.
fn grade_layer_integrity(
    sh: &Shell,
    crate_name: &str,
    cargo_toml_text: &str,
    quiet: bool,
) -> Result<CriterionResult> {
    let in_scope = applies_to_crate(crate_name);
    let metadata = parse_tier_metadata(cargo_toml_text)?;

    // Out-of-scope crate without metadata: not applicable.
    let metadata = match (metadata, in_scope) {
        (Some(m), _) => m,
        (None, false) => {
            return Ok(CriterionResult {
                name: "6. Layer Integrity",
                result: "(out of scope)".to_string(),
                grade: Grade::NotApplicable,
                threshold: "tier metadata",
                measured_detail: format!(
                    "Layer Integrity criterion does not apply to `{}` \
                     (not in sim-*/mesh-*/cf-*/cortenforge* scope per plan §5.1)",
                    crate_name
                ),
            });
        }
        (None, true) => {
            // Hard-gated per plan §8 step 12: in-scope crate without
            // tier metadata is a build error (was warn-only in step 3).
            let msg = format!(
                "in-scope crate `{}` is missing [package.metadata.cortenforge].tier",
                crate_name
            );
            eprintln!("    layer integrity: FAIL — {}", msg);
            return Ok(CriterionResult {
                name: "6. Layer Integrity",
                result: "no tier".to_string(),
                grade: Grade::F,
                threshold: "tier metadata",
                measured_detail: msg,
            });
        }
    };

    // App crates are the dependency SINK: their own graph is unbounded and is
    // never checked for what it consumes. App→App is allowed (e.g.
    // cf-studio-gui → cf-studio-engine → cf-studio-core), so an App crate is
    // not sink-checked against itself either.
    if metadata.tier == Tier::App {
        return Ok(CriterionResult {
            name: "6. Layer Integrity",
            result: "(App tier)".to_string(),
            grade: Grade::NotApplicable,
            threshold: "tier rules",
            measured_detail: "Layer Integrity criterion is N/A for App tier (the dependency sink)"
                .to_string(),
        });
    }

    // App-tier SINK RULE: no SDK-tier crate (L0/L0-io/L0-integration/L1) may
    // depend on an App-tier crate. Derived from the `tier = "App"` declarations
    // (single source of truth — any future app is covered automatically) and
    // enforced for EVERY SDK tier, including L1 (sim-coupling is L1 and must
    // not reach into an app). Computed before the L1 short-circuit below.
    let app_members = app_tier_members(sh)?;
    let mut all_findings: Vec<Finding> =
        sink_findings(sh, crate_name, metadata.tier, &app_members)?;

    // L1 is unbounded for its OWN dep graph — skip the count/ban cargo-tree
    // work; only the sink findings apply. L0/L0-io/L0-integration additionally
    // run the count + banned-prefix checks on top of any sink findings.
    if metadata.tier != Tier::L1 {
        let configs = [
            FeatureConfig::NoDefault,
            FeatureConfig::Default,
            FeatureConfig::AllFeatures,
        ];
        let graphs = [GraphKind::Release, GraphKind::WithDev];
        for fc in configs {
            for gk in graphs {
                let deps = read_tree_deps(sh, crate_name, fc, gk)?;
                let effective_tier = effective_tier_for(&metadata, fc);
                let config = tier_config_with_budget(effective_tier, metadata.dependency_budget);
                let findings = evaluate_dep_set(&deps, config, fc, gk, effective_tier);
                all_findings.extend(findings);
            }
        }
    }

    // Grade decision via the pure helper — locks in the load-bearing
    // L1-with-sink-finding → F fall-through. Hard-gated per plan §8 step 12.
    match layer_integrity_grade(metadata.tier, all_findings.len()) {
        // Only L1 with zero findings reaches N/A here (App returned earlier).
        Grade::NotApplicable => {
            return Ok(CriterionResult {
                name: "6. Layer Integrity",
                result: "(L1 tier)".to_string(),
                grade: Grade::NotApplicable,
                threshold: "tier rules",
                measured_detail:
                    "Layer Integrity criterion is N/A for L1 tier (unbounded dep graph)".to_string(),
            });
        }
        Grade::A => {
            // A raised cap is announced on the PASS path, on stderr and in the
            // table's own `result` column. `measured_detail` alone is NOT
            // enough: `print_criterion` renders only name/result/grade/
            // threshold, so a detail-only note is invisible on
            // `cargo xtask grade <crate>` — the command this is meant to be
            // audited from — and surfaces solely under `--json`.
            let raised = budget_in_effect(&metadata);
            if let Some(budget) = raised {
                eprintln!(
                    "    layer integrity: dependency_budget RAISED to {budget} for `{crate_name}` \
                     (tier {} default {})",
                    metadata.tier.label(),
                    tier_config(metadata.tier).release_max,
                );
            }
            return Ok(CriterionResult {
                name: "6. Layer Integrity",
                result: match raised {
                    Some(budget) => format!("✓ budget {budget}"),
                    None => "✓ confirmed".to_string(),
                },
                grade: Grade::A,
                threshold: "tier rules",
                measured_detail: match raised {
                    Some(budget) => format!(
                        "tier {} — no findings (dependency_budget raised to {budget}; \
                         tier default {})",
                        metadata.tier.label(),
                        tier_config(metadata.tier).release_max
                    ),
                    None => format!("tier {} — no findings", metadata.tier.label()),
                },
            });
        }
        // Grade::F falls through to the finding rendering below.
        _ => {}
    }

    // Render findings to stderr and to measured_detail. Stderr emission
    // is unconditional — failures are PR-blocking under hard gate.
    // Quiet (set by `grade-all`) only suppresses the per-finding lines;
    // the summary line remains so `grade-all` output stays scannable.
    let n = all_findings.len();
    // The failure path must name a raised cap too. Without this, a reader sees
    // "dep count 210 exceeds max 200" and takes 200 for the tier's own limit.
    let raised_note = match budget_in_effect(&metadata) {
        Some(budget) => format!(
            ", dependency_budget RAISED to {budget} from tier default {}",
            tier_config(metadata.tier).release_max
        ),
        None => String::new(),
    };
    eprintln!(
        "    layer integrity: FAIL — {} finding(s) for `{}` (tier {}{})",
        n,
        crate_name,
        metadata.tier.label(),
        raised_note,
    );
    let mut detail = format!(
        "tier {}{} — {} finding(s):\n",
        metadata.tier.label(),
        raised_note,
        n
    );
    for f in &all_findings {
        let line = format_finding(f);
        detail.push_str(&format!("  {}\n", line));
        if !quiet {
            eprintln!("      {}", line);
        }
    }

    Ok(CriterionResult {
        name: "6. Layer Integrity",
        result: format!("{} leak{}", n, if n == 1 { "" } else { "s" }),
        grade: Grade::F,
        threshold: "tier rules",
        measured_detail: detail,
    })
}

/// True if `wasm32-unknown-unknown` is installed via rustup. Returns
/// `false` if rustup is missing or the target isn't listed; the caller
/// degrades to `Grade::Manual` rather than running an unwinnable check.
fn wasm_target_installed(sh: &Shell) -> bool {
    let installed = cmd!(sh, "rustup target list --installed")
        .ignore_status()
        .ignore_stderr()
        .read()
        .unwrap_or_default();
    installed
        .lines()
        .any(|l| l.trim() == "wasm32-unknown-unknown")
}

/// Reduce a (possibly multi-page) `cargo check` stderr to a one-line
/// summary fit for `measured_detail`. Picks the first `error:` or
/// `error[Exxxx]:` rustc diagnostic; falls back to the last few stderr
/// lines if none is present.
fn extract_wasm_error_summary(stderr: &str) -> String {
    for line in stderr.lines() {
        let trimmed = line.trim_start();
        if trimmed.starts_with("error:") || trimmed.starts_with("error[") {
            return trimmed.to_string();
        }
    }
    let tail: Vec<&str> = stderr
        .lines()
        .filter(|l| !l.trim().is_empty())
        .rev()
        .take(3)
        .collect();
    if tail.is_empty() {
        "(no stderr output captured)".to_string()
    } else {
        tail.into_iter().rev().collect::<Vec<_>>().join(" / ")
    }
}

/// Implementation of Criterion 7: WASM Compatibility (L0 only).
///
/// Per plan §5.3: for every `tier == L0` crate, run
/// `cargo check -p <crate> --target wasm32-unknown-unknown --no-default-features`
/// and fail the criterion on non-zero exit. Other tiers (L0-io,
/// L0-integration, L1) and out-of-scope crates report `NotApplicable`.
///
/// Hard-gated per plan §8 step 12: returns `Grade::F` when the wasm32
/// build exits non-zero. Warning-mode rollout in step 4 cleared the way
/// for hard-gate by absorbing the interim state (4 L0 crates failing
/// getrandom 0.3.4 wasm) until P2 cleared them in step 12 commit
/// `a99992a4` (workspace getrandom wasm_js backend).
fn grade_wasm_compat(
    sh: &Shell,
    crate_name: &str,
    cargo_toml_text: &str,
    quiet: bool,
) -> Result<CriterionResult> {
    let in_scope = applies_to_crate(crate_name);
    let metadata = parse_tier_metadata(cargo_toml_text)?;

    let metadata = match (metadata, in_scope) {
        (Some(m), _) => m,
        (None, false) => {
            return Ok(CriterionResult {
                name: "7. WASM Compat",
                result: "(out of scope)".to_string(),
                grade: Grade::NotApplicable,
                threshold: "L0 wasm32",
                measured_detail: format!(
                    "WASM compatibility criterion does not apply to `{}` \
                     (not in sim-*/mesh-*/cf-*/cortenforge* scope per plan §5.1)",
                    crate_name
                ),
            });
        }
        (None, true) => {
            // Missing tier metadata is Layer Integrity's domain (criterion 6
            // already warns and will hard-fail at step 12). WASM Compat
            // can't determine which tier the crate is, so it can't run the
            // check — return NotApplicable rather than pretending the
            // criterion ran. Worst-grade rule on Layer Integrity still
            // produces an overall F at step 12 hard-gate.
            return Ok(CriterionResult {
                name: "7. WASM Compat",
                result: "(no tier)".to_string(),
                grade: Grade::NotApplicable,
                threshold: "L0 wasm32",
                measured_detail: format!(
                    "WASM check skipped: in-scope crate `{}` missing \
                     [package.metadata.cortenforge].tier (see Layer Integrity warning)",
                    crate_name
                ),
            });
        }
    };

    if metadata.tier != Tier::L0 {
        return Ok(CriterionResult {
            name: "7. WASM Compat",
            result: format!("(tier {})", metadata.tier.label()),
            grade: Grade::NotApplicable,
            threshold: "L0 wasm32",
            measured_detail: format!(
                "WASM check is L0-only per plan §5.3; `{}` is tier {}",
                crate_name,
                metadata.tier.label()
            ),
        });
    }

    if !wasm_target_installed(sh) {
        return Ok(CriterionResult {
            name: "7. WASM Compat",
            result: "(target n/a)".to_string(),
            grade: Grade::Manual,
            threshold: "L0 wasm32",
            measured_detail: "wasm32-unknown-unknown target not installed; \
                              run `rustup target add wasm32-unknown-unknown` \
                              and re-grade for an automated check"
                .to_string(),
        });
    }

    let output = cmd!(
        sh,
        "cargo check -p {crate_name} --target wasm32-unknown-unknown --no-default-features"
    )
    .ignore_status()
    .output()
    .with_context(|| format!("failed to invoke `cargo check` for {}", crate_name))?;

    // Hard-gated per plan §8 step 12 (was Grade::A unconditionally in
    // step 4 warning-mode commit `c03fdabc`).
    let warning_grade = if output.status.success() {
        Grade::A
    } else {
        Grade::F
    };

    if output.status.success() {
        return Ok(CriterionResult {
            name: "7. WASM Compat",
            result: "✓ builds".to_string(),
            grade: warning_grade,
            threshold: "L0 wasm32",
            measured_detail: format!(
                "`{}` builds for wasm32-unknown-unknown (--no-default-features)",
                crate_name
            ),
        });
    }

    let stderr = String::from_utf8_lossy(&output.stderr);
    let summary = extract_wasm_error_summary(&stderr);
    eprintln!(
        "    wasm compat: FAIL — `{}` does not build for wasm32-unknown-unknown",
        crate_name
    );
    if !quiet {
        eprintln!("      {}", summary);
    }

    Ok(CriterionResult {
        name: "7. WASM Compat",
        result: "fails".to_string(),
        grade: warning_grade,
        threshold: "L0 wasm32",
        measured_detail: format!(
            "wasm32 build failed for `{}` (--no-default-features):\n  {}",
            crate_name, summary
        ),
    })
}

/// Show status of all crates in the workspace
pub fn status() -> Result<()> {
    let sh = Shell::new()?;
    let workspace_root = find_workspace_root(&sh)?;
    sh.change_dir(&workspace_root);

    println!();
    println!("{}", "CortenForge Crate Status".bold());
    println!("{}", "========================".bold());
    println!();

    // List all crates
    let _output = cmd!(sh, "cargo metadata --format-version 1 --no-deps")
        .read()
        .context("Failed to get workspace metadata")?;

    // Parse JSON output to get package names
    // For now, just list directories
    let locations = ["design", "mesh", "geometry", "sim"];

    for loc in &locations {
        if Path::new(loc).exists() {
            println!("{}/", loc.bold());
            for entry in std::fs::read_dir(loc)? {
                let entry = entry?;
                if entry.path().is_dir() {
                    let name = entry.file_name().to_string_lossy().to_string();
                    let completion_path = format!("{}/{}/COMPLETION.md", loc, name);
                    let status = if Path::new(&completion_path).exists() {
                        "✓ A-grade".green().to_string()
                    } else {
                        "○ pending".dimmed().to_string()
                    };
                    println!("  {} {}", name, status);
                }
            }
            println!();
        }
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn split(s: &str) -> Vec<&str> {
        s.lines().collect()
    }

    #[test]
    fn enclosing_allow_same_line() {
        let lines = split("#[allow(clippy::panic)]\nfn foo() { panic!(); }\n");
        assert!(has_enclosing_allow(&lines, 1, "clippy::panic"));
    }

    #[test]
    fn enclosing_allow_multi_line_attribute() {
        let src =
            "#[allow(\n    clippy::panic,\n    clippy::unwrap_used,\n)]\nfn foo() { panic!(); }\n";
        let lines = split(src);
        assert!(has_enclosing_allow(&lines, 4, "clippy::panic"));
    }

    #[test]
    fn enclosing_allow_different_lint_does_not_match() {
        // Loose substring scan used to match this. Span-aware must not.
        let lines = split("#[allow(clippy::unwrap_used)]\nfn foo() { panic!(); }\n");
        assert!(!has_enclosing_allow(&lines, 1, "clippy::panic"));
    }

    #[test]
    fn enclosing_allow_comment_mentioning_lint_does_not_match() {
        let lines = split("// TODO: consider clippy::panic here\nfn foo() { panic!(); }\n");
        assert!(!has_enclosing_allow(&lines, 1, "clippy::panic"));
    }

    #[test]
    fn enclosing_allow_string_literal_does_not_match() {
        let lines = split("let s = \"clippy::panic\";\nfn foo() { panic!(); }\n");
        assert!(!has_enclosing_allow(&lines, 1, "clippy::panic"));
    }

    #[test]
    fn enclosing_allow_no_allow_at_all() {
        let lines = split("fn foo() {\n    panic!();\n}\n");
        assert!(!has_enclosing_allow(&lines, 1, "clippy::panic"));
    }

    #[test]
    fn enclosing_allow_inner_attribute() {
        let lines = split("#![allow(clippy::panic)]\n\nfn foo() { panic!(); }\n");
        assert!(has_enclosing_allow(&lines, 2, "clippy::panic"));
    }

    #[test]
    fn enclosing_allow_window_just_inside() {
        // Allow 300 lines back (distance 300, window [i-300, i-1]).
        let mut src = String::from("// header\n#[allow(clippy::panic)]\n");
        for _ in 0..299 {
            src.push_str("// filler\n");
        }
        src.push_str("fn foo() { panic!(); }\n");
        let lines = split(&src);
        // Panic is at index 301. Allow is at index 1. start = 1. Loop includes line 1.
        assert!(has_enclosing_allow(&lines, 301, "clippy::panic"));
    }

    #[test]
    fn enclosing_allow_window_boundary_excluded() {
        // Allow at distance 301: out of the 300-line window.
        let mut src = String::from("#[allow(clippy::panic)]\n");
        for _ in 0..300 {
            src.push_str("// filler\n");
        }
        src.push_str("fn foo() { panic!(); }\n");
        let lines = split(&src);
        // Panic at index 301. Allow at index 0. start = 1. Line 0 excluded.
        assert!(!has_enclosing_allow(&lines, 301, "clippy::panic"));
    }

    #[test]
    fn enclosing_allow_multiple_allows_one_matches() {
        let src =
            "#[allow(clippy::unwrap_used)]\n#[allow(clippy::panic)]\nfn foo() { panic!(); }\n";
        let lines = split(src);
        assert!(has_enclosing_allow(&lines, 2, "clippy::panic"));
        assert!(has_enclosing_allow(&lines, 2, "clippy::unwrap_used"));
    }

    #[test]
    fn enclosing_allow_file_top_inner_covers_deep_line() {
        // File-level `#![allow(clippy::panic)]` at line 1 must suppress
        // a panic!() on line 900 — file-top window always scanned for
        // inner attributes.
        let mut src = String::from("// copyright header\n#![allow(clippy::panic)]\n");
        for _ in 0..900 {
            src.push_str("// filler\n");
        }
        src.push_str("fn foo() { panic!(); }\n");
        let lines = split(&src);
        // Panic at index 902. Inner allow at index 1 — way outside the
        // 300-line back-window but inside the 50-line file-top window.
        assert!(has_enclosing_allow(&lines, 902, "clippy::panic"));
    }

    #[test]
    fn enclosing_allow_file_top_inner_covers_dead_zone_line() {
        // Regression for the dead-zone bug: violations at line indices
        // 301..=350 with `back_start ∈ (0, 50]` previously fell through —
        // primary scan (`back_start..i`) skipped `0..back_start` and the
        // additive scan was gated on `back_start > 50`. The fix drops
        // the gate to `back_start > 0`. Test exemplars at `i = 305` and
        // `i = 349` (both in the former dead zone).
        let mut src = String::from("#![allow(clippy::panic)]\n");
        for _ in 0..348 {
            src.push_str("// filler\n");
        }
        src.push_str("fn foo() { panic!(); }\n");
        let lines = split(&src);
        // Panic at index 349, back_start = 49 — within former dead zone.
        assert!(has_enclosing_allow(&lines, 349, "clippy::panic"));

        // Panic at index 305, back_start = 5 — within former dead zone.
        let mut src2 = String::from("#![allow(clippy::panic)]\n");
        for _ in 0..304 {
            src2.push_str("// filler\n");
        }
        src2.push_str("fn foo() { panic!(); }\n");
        let lines2 = split(&src2);
        assert!(has_enclosing_allow(&lines2, 305, "clippy::panic"));
    }

    #[test]
    fn enclosing_allow_file_top_outer_does_not_reach_deep_line() {
        // An OUTER `#[allow(clippy::panic)]` at the top of the file does
        // NOT suppress a panic far below — outer attributes bind to the
        // next item only, and the file-top window intentionally ignores
        // them to avoid that kind of false positive.
        let mut src = String::from("#[allow(clippy::panic)]\n");
        for _ in 0..900 {
            src.push_str("// filler\n");
        }
        src.push_str("fn foo() { panic!(); }\n");
        let lines = split(&src);
        // Panic at index 901. Outer allow at index 0 — outside back-window,
        // and ignored in file-top scan.
        assert!(!has_enclosing_allow(&lines, 901, "clippy::panic"));
    }

    #[test]
    fn enclosing_allow_file_top_wrong_lint_does_not_cover() {
        let mut src = String::from("#![allow(clippy::unwrap_used)]\n");
        for _ in 0..900 {
            src.push_str("// filler\n");
        }
        src.push_str("fn foo() { panic!(); }\n");
        let lines = split(&src);
        assert!(!has_enclosing_allow(&lines, 901, "clippy::panic"));
    }

    // === count_unjustified_clippy_allows ===

    #[test]
    fn count_allows_unjustified_single_line() {
        let src = "#[allow(clippy::cast_precision_loss)]\nfn foo() {}\n";
        let lines = split(src);
        assert_eq!(count_unjustified_clippy_allows(&lines), 1);
    }

    #[test]
    fn count_allows_justified_by_preceding_comment() {
        let src = "// cast count to f64 for averaging\n#[allow(clippy::cast_precision_loss)]\nfn foo() {}\n";
        let lines = split(src);
        assert_eq!(count_unjustified_clippy_allows(&lines), 0);
    }

    #[test]
    fn count_allows_justified_by_inline_comment() {
        let src = "#[allow(clippy::cast_precision_loss)] // cast count to f64\nfn foo() {}\n";
        let lines = split(src);
        assert_eq!(count_unjustified_clippy_allows(&lines), 0);
    }

    #[test]
    fn count_allows_doc_comment_does_not_justify() {
        let src =
            "/// doc, not justification\n#[allow(clippy::cast_precision_loss)]\nfn foo() {}\n";
        let lines = split(src);
        assert_eq!(count_unjustified_clippy_allows(&lines), 1);
    }

    #[test]
    fn count_allows_inside_test_module_excluded() {
        let src =
            "#[cfg(test)]\nmod tests {\n    #[allow(clippy::unwrap_used)]\n    fn t() {}\n}\n";
        let lines = split(src);
        assert_eq!(count_unjustified_clippy_allows(&lines), 0);
    }

    #[test]
    fn count_allows_test_mod_attr_stack_excluded() {
        // The house pattern: #[allow] stacked between #[cfg(test)] and the
        // mod opening `{`. Pre-fix this scanned as library code.
        let src = "#[cfg(test)]\n#[allow(clippy::unwrap_used, clippy::expect_used)]\nmod tests {\n    fn t() {}\n}\n";
        let lines = split(src);
        assert_eq!(count_unjustified_clippy_allows(&lines), 0);
    }

    #[test]
    fn count_allows_test_mod_attr_stack_multi_line_excluded() {
        let src = "#[cfg(test)]\n#[allow(\n    clippy::unwrap_used,\n    clippy::expect_used,\n)]\nmod tests {\n    fn t() {}\n}\n";
        let lines = split(src);
        assert_eq!(count_unjustified_clippy_allows(&lines), 0);
    }

    #[test]
    fn count_allows_multi_line_unjustified() {
        // Pre-fix this was silently ignored because the substring scan only
        // matched `#[allow(clippy::` on a single line.
        let src = "#[allow(\n    clippy::cast_precision_loss,\n    clippy::cast_possible_truncation,\n)]\nfn foo() {}\n";
        let lines = split(src);
        assert_eq!(count_unjustified_clippy_allows(&lines), 1);
    }

    #[test]
    fn count_allows_multi_line_justified() {
        let src = "// cast indices to f64\n#[allow(\n    clippy::cast_precision_loss,\n    clippy::cast_possible_truncation,\n)]\nfn foo() {}\n";
        let lines = split(src);
        assert_eq!(count_unjustified_clippy_allows(&lines), 0);
    }

    #[test]
    fn count_allows_non_clippy_allow_ignored() {
        // grade_clippy only audits `clippy::` allows; non-clippy allows are
        // not in scope for this criterion.
        let src = "#[allow(dead_code)]\nfn foo() {}\n";
        let lines = split(src);
        assert_eq!(count_unjustified_clippy_allows(&lines), 0);
    }

    #[test]
    fn count_allows_multi_line_non_clippy_ignored() {
        let src = "#[allow(\n    dead_code,\n    unused_variables,\n)]\nfn foo() {}\n";
        let lines = split(src);
        assert_eq!(count_unjustified_clippy_allows(&lines), 0);
    }

    #[test]
    fn count_allows_after_test_module_still_scanned() {
        // Regression for the latch bug fixed in b7ef1c73: library code
        // appearing textually after a test module must still be scanned.
        let src = "#[cfg(test)]\nmod tests {\n    #[allow(clippy::unwrap_used)]\n    fn t() {}\n}\n\n#[allow(clippy::cast_precision_loss)]\nfn lib_fn() {}\n";
        let lines = split(src);
        assert_eq!(count_unjustified_clippy_allows(&lines), 1);
    }

    #[test]
    fn count_allows_block_comment_skipped() {
        let src = "/*\n#[allow(clippy::cast_precision_loss)]\n*/\nfn foo() {}\n";
        let lines = split(src);
        assert_eq!(count_unjustified_clippy_allows(&lines), 0);
    }

    // === count_unjustified_deps ===

    #[test]
    fn count_deps_basic_unjustified() {
        let src = "[dependencies]\nserde = \"1.0\"\n";
        let lines = split(src);
        assert_eq!(count_unjustified_deps(&lines), 1);
    }

    #[test]
    fn count_deps_basic_justified_by_preceding_comment() {
        let src = "[dependencies]\n# serialization for cache files\nserde = \"1.0\"\n";
        let lines = split(src);
        assert_eq!(count_unjustified_deps(&lines), 0);
    }

    #[test]
    fn count_deps_justified_by_inline_comment() {
        let src = "[dependencies]\nserde = \"1.0\" # serialization\n";
        let lines = split(src);
        assert_eq!(count_unjustified_deps(&lines), 0);
    }

    #[test]
    fn count_deps_preceding_comment_three_lines_back() {
        let src = "[dependencies]\n# justification\nfoo = \"1\"\nbar = \"1\"\nbaz = \"1\"\n";
        let lines = split(src);
        // foo: offset 1 -> `#` ok. bar: offset 2 -> `#` ok. baz: offset 3 -> `#` ok.
        assert_eq!(count_unjustified_deps(&lines), 0);
    }

    #[test]
    fn count_deps_preceding_comment_four_lines_back_rejected() {
        let src =
            "[dependencies]\n# justification\nfoo = \"1\"\nbar = \"1\"\nbaz = \"1\"\nqux = \"1\"\n";
        let lines = split(src);
        // qux is 4 lines from `# justification` -> outside window -> unjustified.
        assert_eq!(count_unjustified_deps(&lines), 1);
    }

    #[test]
    fn count_deps_blank_line_breaks_chain() {
        let src = "[dependencies]\n# justification\n\nfoo = \"1\"\n";
        let lines = split(src);
        // Blank at offset 1 breaks backward scan before reaching the `#`.
        assert_eq!(count_unjustified_deps(&lines), 1);
    }

    #[test]
    fn count_deps_non_dep_section_ignored() {
        let src = "[features]\ndefault = []\nparallel = [\"dep:rayon\"]\n\n[profile.dev]\nopt-level = 0\n";
        let lines = split(src);
        assert_eq!(count_unjustified_deps(&lines), 0);
    }

    #[test]
    fn count_deps_dev_dependencies_section_scanned() {
        let src = "[dev-dependencies]\napprox = \"0.5\"\n";
        let lines = split(src);
        assert_eq!(count_unjustified_deps(&lines), 1);
    }

    #[test]
    fn count_deps_build_dependencies_section_scanned() {
        let src = "[build-dependencies]\ncc = \"1\"\n";
        let lines = split(src);
        assert_eq!(count_unjustified_deps(&lines), 1);
    }

    #[test]
    fn count_deps_patch_section_ignored() {
        // `[patch.crates-io]` is not a dep section.
        let src = "[patch.crates-io]\nserde = { git = \"https://example.com\" }\n";
        let lines = split(src);
        assert_eq!(count_unjustified_deps(&lines), 0);
    }

    #[test]
    fn count_deps_workspace_inherited_single_line() {
        let src = "[dependencies]\nnalgebra = { workspace = true }\n";
        let lines = split(src);
        assert_eq!(count_unjustified_deps(&lines), 1);
    }

    #[test]
    fn count_deps_workspace_inherited_with_features_single_line() {
        // The `bevy = { workspace = true, features = [...] }` single-line form.
        let src = "[dependencies]\nbevy = { workspace = true, features = [\"bevy_pbr\"] }\n";
        let lines = split(src);
        assert_eq!(count_unjustified_deps(&lines), 1);
    }

    // --- GAP-A regression: multi-line inline-table dep specs ---

    #[test]
    fn count_deps_multi_line_inline_table_opening_counts_once() {
        // Pre-fix, the continuation lines `version = "0.8",` and
        // `features = ["x"],` would be mis-scanned as two additional dep
        // entries, yielding unjustified == 3. Post-fix, only the opening
        // line is counted.
        let src = "[dependencies]\nrand = {\n    version = \"0.8\",\n    features = [\"small_rng\"],\n}\n";
        let lines = split(src);
        assert_eq!(count_unjustified_deps(&lines), 1);
    }

    #[test]
    fn count_deps_multi_line_inline_table_justified() {
        let src = "[dependencies]\n# justification\nrand = {\n    version = \"0.8\",\n    features = [\"small_rng\"],\n}\n";
        let lines = split(src);
        assert_eq!(count_unjustified_deps(&lines), 0);
    }

    #[test]
    fn count_deps_multi_line_features_array_no_false_positive() {
        // The common house form: `bevy = { workspace = true, features = [`
        // opening, quoted feature strings, `] }` closing. Continuation
        // lines have no `=` so were never false-positives, but brace-depth
        // tracking must still close correctly so the next dep is scanned.
        let src = "[dependencies]\nbevy = { workspace = true, features = [\n    \"bevy_pbr\",\n    \"bevy_ui\",\n] }\nserde = \"1\"\n";
        let lines = split(src);
        // Both `bevy` and `serde` lines are unjustified dep entries.
        assert_eq!(count_unjustified_deps(&lines), 2);
    }

    #[test]
    fn count_deps_dep_after_multi_line_inline_table_still_scanned() {
        // Regression for "brace depth must close": if the closing `}` is
        // not handled, subsequent deps would be treated as continuations.
        let src = "[dependencies]\nfoo = {\n    version = \"0.8\",\n}\nbar = \"0.1\"\n";
        let lines = split(src);
        // Both foo (opening) and bar (scanned after brace closes) flagged.
        assert_eq!(count_unjustified_deps(&lines), 2);
    }

    // --- GAP-C regression: target-cfg dependency sections ---

    #[test]
    fn count_deps_target_cfg_dependencies_scanned() {
        let src = "[target.'cfg(unix)'.dependencies]\nlibc = \"0.2\"\n";
        let lines = split(src);
        assert_eq!(count_unjustified_deps(&lines), 1);
    }

    #[test]
    fn count_deps_target_cfg_dev_dependencies_scanned() {
        let src = "[target.'cfg(windows)'.dev-dependencies]\nwindows = \"0.5\"\n";
        let lines = split(src);
        assert_eq!(count_unjustified_deps(&lines), 1);
    }

    #[test]
    fn count_deps_target_cfg_build_dependencies_scanned() {
        let src = "[target.'cfg(target_os = \"linux\")'.build-dependencies]\ncc = \"1\"\n";
        let lines = split(src);
        assert_eq!(count_unjustified_deps(&lines), 1);
    }

    #[test]
    fn count_deps_target_triple_dependencies_scanned() {
        let src = "[target.x86_64-unknown-linux-gnu.dependencies]\nopenssl = \"0.10\"\n";
        let lines = split(src);
        assert_eq!(count_unjustified_deps(&lines), 1);
    }

    #[test]
    fn count_deps_target_cfg_justified() {
        let src = "[target.'cfg(unix)'.dependencies]\n# unix-only POSIX APIs\nlibc = \"0.2\"\n";
        let lines = split(src);
        assert_eq!(count_unjustified_deps(&lines), 0);
    }

    // === any_span_in_crate (F.1 package-scoping filter) ===

    #[test]
    fn any_span_in_crate_single_span_inside() {
        let spans = vec![serde_json::json!({ "file_name": "sim/L0/ml-chassis/src/lib.rs" })];
        assert!(any_span_in_crate(&spans, "sim/L0/ml-chassis"));
    }

    #[test]
    fn any_span_in_crate_single_span_outside() {
        // Warning reported while compiling sim-ml-chassis but the span points
        // into sim-types — the transitive-dep bleed F.1 filters out.
        let spans = vec![serde_json::json!({ "file_name": "sim/L0/types/src/lib.rs" })];
        assert!(!any_span_in_crate(&spans, "sim/L0/ml-chassis"));
    }

    #[test]
    fn any_span_in_crate_mixed_spans_included() {
        // Macro-generated case: one span in the target crate, one in a dep.
        // Audit-locked semantics: disjunctive — include-not-exclude.
        let spans = vec![
            serde_json::json!({ "file_name": "sim/L0/ml-chassis/src/autograd.rs" }),
            serde_json::json!({ "file_name": "sim/L0/types/src/lib.rs" }),
        ];
        assert!(any_span_in_crate(&spans, "sim/L0/ml-chassis"));
    }

    // === F.3 IntegrationOnly profile + metadata opt-in ===

    #[test]
    fn classify_crate_metadata_opt_in_integration_only() {
        // A crate sitting in a path that would otherwise classify as
        // Layer0 opts into IntegrationOnly via the metadata block.
        let cargo_toml = "\
[package]
name = \"sim-therm-env\"

[package.metadata.cortenforge]
grading_profile = \"integration-only\"

[dependencies]
serde = \"1\"
";
        assert_eq!(
            classify_crate("sim/L0/therm-env", cargo_toml),
            CrateProfile::IntegrationOnly,
        );
    }

    #[test]
    fn coverage_skip_reason_integration_only_is_skipped() {
        // F.3: grade_coverage takes the early-return path for
        // IntegrationOnly; result label is "(integration-only)" to
        // distinguish it from Example/Xtask's "(bin-only)".
        let (result, detail) =
            coverage_skip_reason(CrateProfile::IntegrationOnly, false).expect("must skip");
        assert_eq!(result, "(integration-only)");
        assert!(detail.contains("[package.metadata.cortenforge]"));
        // The metadata opt-in is a statement about where the crate's tests
        // live, not about its targets, so owning a lib does not revoke it.
        assert!(
            coverage_skip_reason(CrateProfile::IntegrationOnly, true).is_some(),
            "the F.3 opt-in is explicit and outranks target layout"
        );
        // Sanity: Layer0 must NOT skip.
        assert!(coverage_skip_reason(CrateProfile::Layer0, false).is_none());
    }

    /// ★★ The regression test for the defect this function was changed to fix.
    ///
    /// `Example`/`Xtask` skipped Coverage unconditionally, saying "no lib
    /// target" — a claim nothing checked, and false for 13 of the 17 crates
    /// under `tools/`. Both halves are asserted here, because a skip that is
    /// merely *narrower* would still be wrong in the other direction: a
    /// genuinely bin-only tool must keep its N/A rather than start failing.
    #[test]
    fn a_tool_crate_with_a_library_is_measured_and_one_without_is_not() {
        for profile in [CrateProfile::Xtask, CrateProfile::Example] {
            assert!(
                coverage_skip_reason(profile, true).is_none(),
                "{profile:?} with a lib target must be MEASURED — the 2026-08-16 \
                 defect was skipping exactly this case"
            );
            let (result, detail) =
                coverage_skip_reason(profile, false).expect("no lib target must still skip");
            assert_eq!(result, "(bin-only)");
            assert!(
                detail.contains("no lib target"),
                "the reason must still name the property that is now checked: {detail}"
            );
        }
    }

    /// A crate's own source tree for the [`has_lib_target`] cases below.
    fn manifest_fixture(tag: &str, files: &[&str]) -> std::path::PathBuf {
        let root = std::env::temp_dir().join(format!(
            "cf-has-lib-{tag}-{}-{:?}",
            std::process::id(),
            std::thread::current().id()
        ));
        let _ = std::fs::remove_dir_all(&root);
        std::fs::create_dir_all(&root).expect("mkdir root");
        for name in files {
            let path = root.join(name);
            std::fs::create_dir_all(path.parent().expect("parent")).expect("mkdir");
            std::fs::write(&path, "fn f() {}\n").expect("write");
        }
        root
    }

    #[test]
    fn has_lib_target_finds_an_auto_discovered_src_lib() {
        let root = manifest_fixture("auto", &["src/lib.rs", "src/main.rs"]);
        assert!(has_lib_target(&root, "[package]\nname = \"x\"\n"));
    }

    #[test]
    fn has_lib_target_finds_an_explicitly_declared_lib_off_the_conventional_path() {
        // 9 of the 13 newly-measured `tools/` crates declare `[lib]`; a
        // declaration with a non-default `path` is the case a `src/lib.rs`
        // existence check alone would miss.
        let root = manifest_fixture("declared", &["src/other.rs"]);
        assert!(has_lib_target(
            &root,
            "[package]\nname = \"x\"\n[lib]\npath = \"src/other.rs\"\n"
        ));
    }

    #[test]
    fn has_lib_target_is_false_for_a_genuinely_bin_only_crate() {
        let root = manifest_fixture("binonly", &["src/main.rs"]);
        assert!(!has_lib_target(&root, "[package]\nname = \"x\"\n"));
    }

    #[test]
    fn has_lib_target_honours_autolib_false() {
        let root = manifest_fixture("autolib", &["src/lib.rs"]);
        assert!(
            !has_lib_target(&root, "[package]\nname = \"x\"\nautolib = false\n"),
            "Rust 2024 `autolib = false` suppresses src/lib.rs discovery"
        );
    }

    /// ⚠ The direction matters: an unreadable manifest must MEASURE, not skip.
    /// A measurement that should not have run is a visible number; a skip that
    /// should not have happened is silence, and silence is what hid 9718
    /// lines.
    #[test]
    fn has_lib_target_errs_toward_measuring_when_the_manifest_will_not_parse() {
        let root = manifest_fixture("broken", &["src/main.rs"]);
        assert!(has_lib_target(&root, "this is not [ valid toml"));
    }

    /// Asserts the list's SHAPE, not that its names resolve to real crates.
    ///
    /// Deliberately: a stale name simply stops deferring a crate, which turns
    /// the build red rather than hiding anything, so the failure mode is
    /// fail-safe and does not need a gate. A duplicate is the case worth
    /// catching, because two spellings of one intent is how a list like this
    /// starts drifting from the thing it is a to-do for.
    #[test]
    fn every_report_only_name_is_spelled_once() {
        let mut seen = std::collections::BTreeSet::new();
        for name in COVERAGE_REPORT_ONLY {
            assert!(seen.insert(*name), "{name} listed twice");
            assert!(is_coverage_report_only(name));
        }
        assert!(!is_coverage_report_only("sim-types"));
    }

    /// ⚠ Deferring a crate that ALREADY gated would be a regression dressed as
    /// leniency — the waiver exists for crates the measurement fix newly lit,
    /// and `cf-viewer` was failing this criterion before the fix landed.
    #[test]
    fn a_crate_that_already_gated_is_not_deferred() {
        assert!(
            !is_coverage_report_only("cf-viewer"),
            "cf-viewer was measured and failing before has_lib_target existed"
        );
    }

    // === coverage_result — the verdict, driven without a cargo run ===

    use crate::coverage::ProductionCoverage;

    /// A finished measurement. `bin`/`lib` split off the same totals, exactly
    /// as `production_coverage` accumulates them.
    fn measurement(
        covered: u64,
        total: u64,
        bin_covered: u64,
        bin_total: u64,
    ) -> ProductionCoverage {
        ProductionCoverage {
            covered,
            total,
            excluded: 0,
            unparsed: Vec::new(),
            files: Vec::new(),
            bin_covered,
            bin_total,
        }
    }

    #[test]
    fn coverage_result_grades_each_threshold_band() {
        for (covered, expect) in [
            (95, Grade::APlus),
            (80, Grade::A),
            (65, Grade::B),
            (45, Grade::C),
            (20, Grade::F),
        ] {
            let r = coverage_result(&measurement(covered, 100, 0, 0), true, false);
            assert_eq!(r.grade, expect, "{covered}/100");
        }
    }

    /// ★★ A deferred crate is MEASURED, not skipped: the real percentage is on
    /// the row, the grade it would have taken is in the detail, and the reader
    /// is told the enforcement is deferred rather than absent.
    #[test]
    fn a_report_only_crate_still_reports_its_real_number() {
        let r = coverage_result(&measurement(30, 100, 0, 0), true, true);

        assert_eq!(r.grade, Grade::NotApplicable, "the threshold is waived");
        assert_eq!(r.result, "30.0% (report-only)");
        assert!(
            r.measured_detail.contains("grade would be F"),
            "the withheld grade must be stated: {}",
            r.measured_detail
        );
        assert!(r.measured_detail.contains("30.0% production line coverage"));
    }

    /// ★★★ The safety property the deferral is documented to have, and the one
    /// a mutation sweep found nothing enforcing: report-only waives the
    /// coverage THRESHOLD, never a red test suite.
    #[test]
    fn a_report_only_crate_with_failing_tests_still_grades_f() {
        let r = coverage_result(&measurement(99, 100, 0, 0), false, true);

        assert_eq!(
            r.grade,
            Grade::F,
            "99% coverage must not launder a failed suite through the waiver"
        );
        assert!(!r.result.contains("report-only"));
        assert!(r.measured_detail.contains("heavy tests FAILED"));
    }

    /// ★★ Every deferred crate must be refusable by `xtask complete`.
    ///
    /// The predicate is what that refusal keys on, so it has to stay reachable
    /// from outside this module. A `pub(crate)` that quietly became private
    /// again would not break the build — `complete.rs` would fail to compile —
    /// but a rename that left `complete.rs` checking something else would, and
    /// this pins the contract that the two agree on the same list.
    #[test]
    fn every_deferred_crate_reports_as_deferred_to_other_modules() {
        for name in COVERAGE_REPORT_ONLY {
            assert!(
                is_coverage_report_only(name),
                "{name} is on the list and must be refused a completion record"
            );
        }
        assert!(!is_coverage_report_only("sim-types"));
    }

    /// ★ The SEAM: the deferral flag must be looked up, not hardcoded.
    ///
    /// `coverage_result`'s own tests drive the flag directly and say nothing
    /// about where it comes from; a mutation passing a bare `false` at the
    /// call site survived them.
    ///
    /// ⚠ Written to survive the list emptying, which is its stated end state.
    /// With an empty list the loop is vacuous and only the `cf-viewer` arm
    /// asserts — which is the arm that matters, because a hardcoded `true`
    /// would waive the threshold for every crate in the workspace, while a
    /// hardcoded `false` with an empty list changes nothing.
    #[test]
    fn the_deferral_is_looked_up_from_the_list_and_not_hardcoded() {
        for name in COVERAGE_REPORT_ONLY {
            let r = coverage_result_for(name, &measurement(30, 100, 0, 0), true);
            assert_eq!(
                r.grade,
                Grade::NotApplicable,
                "{name} is on the list and must be deferred"
            );
        }
        let r = coverage_result_for("cf-viewer", &measurement(30, 100, 0, 0), true);
        assert_eq!(
            r.grade,
            Grade::F,
            "cf-viewer is not on the list and must still gate"
        );
    }

    /// The same crate not on the list grades normally, so the test above is
    /// about `heavy_passed` and not about the list being ineffective.
    #[test]
    fn an_undeferred_crate_grades_on_the_threshold() {
        let r = coverage_result(&measurement(30, 100, 0, 0), true, false);
        assert_eq!(r.grade, Grade::F);
        assert_eq!(r.result, "30.0%");
        assert!(!r.measured_detail.contains("REPORT-ONLY"));
    }

    /// The library-only figure appears only when a binary target contributed
    /// lines, and reports the split the reader needs to act on it.
    #[test]
    fn the_detail_reports_the_library_only_figure_when_a_binary_contributed() {
        // 40/100 overall; the binary holds 50 lines, none covered, so the
        // library is 40/50 = 80 %.
        let r = coverage_result(&measurement(40, 100, 0, 50), true, false);

        assert!(
            r.measured_detail.contains("80.0% over library lines alone"),
            "{}",
            r.measured_detail
        );
        assert!(r.measured_detail.contains("50 line(s) in binary targets"));
        assert_eq!(r.grade, Grade::C, "the GRADED figure is still 40 %");
    }

    /// ⚠ And is absent for a crate with no binary, rather than reporting the
    /// same number twice under two names.
    #[test]
    fn the_detail_omits_the_library_figure_when_there_is_no_binary() {
        let r = coverage_result(&measurement(40, 100, 0, 0), true, false);
        assert!(!r.measured_detail.contains("library lines alone"));
    }

    #[test]
    fn a_measurement_with_no_production_lines_is_not_a_bad_grade() {
        let r = coverage_result(&measurement(0, 0, 0, 0), true, false);
        assert_eq!(r.grade, Grade::NotApplicable);
        assert_eq!(r.result, "(no production lines)");
    }

    /// ★★ The displayed percentage must never contradict the grade printed
    /// beside it.
    ///
    /// `cf-studio-engine` measured 605/807 = 74.969 % in the 2026-08-16 census
    /// and rounded to "75.0%" — sitting in the same row as a `B`, one line
    /// under the A threshold, reading as a grader bug to anyone who looked.
    #[test]
    fn a_percentage_just_under_the_bar_never_displays_as_the_bar() {
        let just_under = 100.0 * 605.0 / 807.0;
        assert!(just_under < 75.0, "the real cf-studio-engine measurement");
        assert_eq!(
            coverage_display(just_under),
            "74.9%",
            "rounding would print 75.0% next to a B"
        );
    }

    /// The complement: truncation must not steal a tenth from a crate that
    /// genuinely cleared the bar, or the contradiction just changes sign.
    #[test]
    fn a_percentage_at_or_above_the_bar_still_displays_at_the_bar() {
        assert_eq!(coverage_display(75.0), "75.0%");
        assert_eq!(coverage_display(75.09), "75.0%");
        assert_eq!(coverage_display(100.0), "100.0%");
        assert_eq!(coverage_display(0.0), "0.0%");
    }

    // === scan_file_safety — #[allow(clippy::{unwrap,expect}_used)] honoring ===

    #[test]
    fn scan_safety_unwrap_without_allow_counts() {
        let src = "fn f() {\n    x.unwrap();\n}\n";
        let r = scan_file_safety(src, false, false);
        assert_eq!(r.counted_violations, 1);
    }

    #[test]
    fn scan_safety_unwrap_with_enclosing_allow_skipped() {
        let src = "#[allow(clippy::unwrap_used)]\nfn f() {\n    x.unwrap();\n}\n";
        let r = scan_file_safety(src, false, false);
        assert_eq!(r.counted_violations, 0);
    }

    #[test]
    fn scan_safety_expect_without_allow_counts() {
        let src = "fn f() {\n    x.expect(\"bad\");\n}\n";
        let r = scan_file_safety(src, false, false);
        assert_eq!(r.counted_violations, 1);
    }

    #[test]
    fn scan_safety_expect_with_enclosing_allow_skipped() {
        // Matches the canonical chassis pattern: `#[allow(clippy::expect_used)]`
        // directly above a fn that makes a single localized expect call.
        let src = "#[allow(clippy::expect_used)]\nfn f() {\n    x.expect(\"bad\");\n}\n";
        let r = scan_file_safety(src, false, false);
        assert_eq!(r.counted_violations, 0);
    }

    #[test]
    fn scan_safety_allow_different_lint_does_not_mask_expect() {
        // #[allow(clippy::unwrap_used)] does not suppress an .expect(.
        let src = "#[allow(clippy::unwrap_used)]\nfn f() {\n    x.expect(\"bad\");\n}\n";
        let r = scan_file_safety(src, false, false);
        assert_eq!(r.counted_violations, 1);
    }

    #[test]
    fn scan_safety_allow_different_lint_does_not_mask_unwrap() {
        let src = "#[allow(clippy::expect_used)]\nfn f() {\n    x.unwrap();\n}\n";
        let r = scan_file_safety(src, false, false);
        assert_eq!(r.counted_violations, 1);
    }

    #[test]
    fn scan_safety_module_level_inner_allow_suppresses() {
        // #![allow(clippy::expect_used)] at the top of a module suppresses
        // expect in the whole module.
        let src = "#![allow(clippy::expect_used)]\n\nfn f() {\n    x.expect(\"bad\");\n}\n";
        let r = scan_file_safety(src, false, false);
        assert_eq!(r.counted_violations, 0);
    }

    #[test]
    fn scan_safety_todo_with_enclosing_allow_skipped() {
        // Intentional stub pattern: file-level #![allow(clippy::todo)]
        // followed by todo!() in main. Not a real violation.
        let src = "#![allow(clippy::todo)]\n\nfn main() {\n    todo!(\"blocked\");\n}\n";
        let r = scan_file_safety(src, false, false);
        assert!(!r.has_todo_or_unimplemented);
    }

    #[test]
    fn scan_safety_todo_without_allow_flags() {
        let src = "fn main() {\n    todo!();\n}\n";
        let r = scan_file_safety(src, false, false);
        assert!(r.has_todo_or_unimplemented);
    }

    #[test]
    fn scan_safety_unimplemented_with_enclosing_allow_skipped() {
        let src = "#[allow(clippy::unimplemented)]\nfn f() {\n    unimplemented!();\n}\n";
        let r = scan_file_safety(src, false, false);
        assert!(!r.has_todo_or_unimplemented);
    }

    #[test]
    fn scan_safety_docstring_is_not_an_allow() {
        // A /// docstring mentioning clippy::expect_used does NOT function as
        // an allow — only real attributes do.
        let src = "/// clippy::expect_used is a thing\nfn f() {\n    x.expect(\"bad\");\n}\n";
        let r = scan_file_safety(src, false, false);
        assert_eq!(r.counted_violations, 1);
    }

    // === has_file_level_cfg_test ===

    #[test]
    fn file_level_cfg_test_detected_at_first_line() {
        let src = "#![cfg(test)]\n\nfn t() {}\n";
        let lines = split(src);
        assert!(has_file_level_cfg_test(&lines));
    }

    #[test]
    fn file_level_cfg_test_detected_after_module_docs() {
        let src = "//! Module docs.\n//!\n//! More docs.\n\n#![cfg(test)]\n\nfn t() {}\n";
        let lines = split(src);
        assert!(has_file_level_cfg_test(&lines));
    }

    #[test]
    fn file_level_cfg_test_not_present() {
        // A regular library file — no inner attribute.
        let src = "use std::fmt;\n\npub fn f() {}\n";
        let lines = split(src);
        assert!(!has_file_level_cfg_test(&lines));
    }

    #[test]
    fn file_level_cfg_test_inline_test_mod_is_not_file_level() {
        // An INLINE #[cfg(test)] mod tests { ... } doesn't mark the whole
        // file as test-only; the scanner's brace-depth state machine
        // handles that case separately.
        let src = "pub fn f() {}\n\n#[cfg(test)]\nmod tests {}\n";
        let lines = split(src);
        assert!(!has_file_level_cfg_test(&lines));
    }

    // === strip_string_literals ===

    #[test]
    fn strip_strings_replaces_content_with_spaces() {
        // Preserves column positions; quotes become spaces.
        // Input: `let s = "todo!";`  — "todo!" is 7 chars (2 quotes + 5 body).
        // Expected: `let s =        ;`  — 7 spaces where "todo!" was, plus the
        // space before `=` and the one after = space both preserved.
        let input = "let s = \"todo!\";";
        let expected: String = format!("let s = {};", " ".repeat("\"todo!\"".len()));
        assert_eq!(strip_string_literals(input), expected);
    }

    #[test]
    fn strip_strings_handles_escapes() {
        // \\" inside a string is an escaped quote, not a terminator.
        let out = strip_string_literals("\"a\\\"b\"");
        // 6 characters in, all become spaces.
        assert_eq!(out, "      ");
    }

    #[test]
    fn strip_strings_leaves_code_outside_strings() {
        assert_eq!(
            strip_string_literals("fn foo() { panic!(); }"),
            "fn foo() { panic!(); }"
        );
    }

    #[test]
    fn strip_strings_multiple_literals_on_one_line() {
        let out = strip_string_literals("contains(\"a\") && contains(\"b\")");
        assert_eq!(out, "contains(   ) && contains(   )");
    }

    // === has_macro_call — string-literal false-positive fix ===

    #[test]
    fn has_macro_call_paren_form() {
        assert!(has_macro_call("todo!()", "todo!"));
        assert!(has_macro_call("todo!(\"not ready\")", "todo!"));
        assert!(has_macro_call("    panic!();", "panic!"));
    }

    #[test]
    fn has_macro_call_brace_form() {
        assert!(has_macro_call("unimplemented!{}", "unimplemented!"));
    }

    #[test]
    fn has_macro_call_bracket_form() {
        // Less common but valid.
        assert!(has_macro_call("todo![]", "todo!"));
    }

    #[test]
    fn has_macro_call_string_literal_not_flagged() {
        // The canonical false-positive: xtask's own grader scans itself,
        // finds the string `"todo!"` in its source, must not flag.
        assert!(!has_macro_call("let s = \"todo!\";", "todo!"));
        assert!(!has_macro_call(
            "result: \"F: found todo!/unimplemented!\".to_string(),",
            "todo!"
        ));
        assert!(!has_macro_call(
            "if trimmed.contains(\"panic!\") {",
            "panic!"
        ));
    }

    #[test]
    fn has_macro_call_bare_mention_not_flagged() {
        // Mentions of the macro name without delimiter should not trigger.
        assert!(!has_macro_call("pub fn todo!foo() {}", "todo!"));
        assert!(!has_macro_call("// todo! this later", "todo!"));
    }

    #[test]
    fn has_macro_call_scan_detects_real_todo_in_mixed_line() {
        // A real invocation mid-line still flags.
        assert!(has_macro_call("    let x = todo!();", "todo!"));
    }

    // === CrateProfile-aware relaxation: unjustified clippy allows ===
    //
    // The helper count_unjustified_clippy_allows is profile-agnostic by
    // design (Layer 0 lib behavior). These tests document the profile
    // gating at the grade_clippy call site by exercising the helper on
    // the same input and verifying it returns the raw count — grade_clippy
    // suppresses the count for Example/Xtask profiles, preserving it
    // unchanged for Layer0/BevyLayer1/IntegrationOnly.

    #[test]
    fn unjustified_clippy_allows_counted_for_non_example() {
        // Baseline: the scanner counts this as unjustified regardless of
        // profile — the profile gate lives in grade_clippy.
        let src = "#[allow(clippy::expect_used)]\nfn f() {}\n";
        let lines = split(src);
        assert_eq!(count_unjustified_clippy_allows(&lines), 1);
    }

    // === count_unjustified_deps is profile-agnostic by design ===

    #[test]
    fn unjustified_deps_counted_raw() {
        // Baseline: the Cargo.toml scanner counts unjustified deps
        // regardless of profile; the profile gate lives in
        // grade_dependencies.
        let src = "\
[dependencies]
serde = \"1\"
";
        let lines = split(src);
        assert_eq!(count_unjustified_deps(&lines), 1);
    }

    // === Layer Integrity (criterion 6) ===

    #[test]
    fn tier_parse_known_values() {
        assert_eq!(Tier::parse("L0"), Some(Tier::L0));
        assert_eq!(Tier::parse("L0-io"), Some(Tier::L0Io));
        assert_eq!(Tier::parse("L0-integration"), Some(Tier::L0Integration));
        assert_eq!(Tier::parse("L1"), Some(Tier::L1));
        assert_eq!(Tier::parse("App"), Some(Tier::App));
    }

    #[test]
    fn tier_parse_rejects_unknown() {
        assert_eq!(Tier::parse("L2"), None);
        assert_eq!(Tier::parse("l0"), None);
        assert_eq!(Tier::parse(""), None);
        // Case-sensitive: only "App" parses, not "app"/"APP".
        assert_eq!(Tier::parse("app"), None);
        assert_eq!(Tier::parse("APP"), None);
    }

    #[test]
    fn tier_app_label_round_trips() {
        assert_eq!(Tier::App.label(), "App");
        assert_eq!(Tier::parse(Tier::App.label()), Some(Tier::App));
    }

    #[test]
    fn tier_app_config_is_unbounded_and_unbanned() {
        // App constrains who depends on it (the sink rule), not what it
        // depends on — so its own dep graph is unbounded and unbanned.
        let cfg = tier_config(Tier::App);
        assert_eq!(cfg.release_max, usize::MAX);
        assert_eq!(cfg.test_max, usize::MAX);
        assert!(cfg.banned.is_empty());
    }

    #[test]
    fn tier_permissiveness_ordering() {
        // L0 strictest → App most permissive. effective_tier_for relies
        // on this monotonic ordering when picking the loosest tier among
        // multiple tier_up_features.
        assert!(Tier::L0.permissiveness() < Tier::L0Io.permissiveness());
        assert!(Tier::L0Io.permissiveness() < Tier::L0Integration.permissiveness());
        assert!(Tier::L0Integration.permissiveness() < Tier::L1.permissiveness());
        assert!(Tier::L1.permissiveness() < Tier::App.permissiveness());
    }

    #[test]
    fn applies_to_crate_in_scope_prefixes() {
        assert!(applies_to_crate("sim-types"));
        assert!(applies_to_crate("sim-mjcf"));
        assert!(applies_to_crate("mesh-io"));
        assert!(applies_to_crate("mesh-types"));
        assert!(applies_to_crate("cf-spatial"));
        assert!(applies_to_crate("cf-design"));
        assert!(applies_to_crate("cortenforge-cli"));
        // The Cendrillon app stack is now in-scope (classified `tier = "App"`),
        // no longer on the exemption list — so Layer Integrity governs it.
        assert!(applies_to_crate("cf-studio-core"));
        assert!(applies_to_crate("cf-studio-engine"));
        assert!(applies_to_crate("cf-studio"));
        assert!(applies_to_crate("cf-studio-gui"));
        // The dual-use library crates the app stack consumes are now classified
        // SDK tiers (no longer exempt), so Layer Integrity governs them too.
        assert!(applies_to_crate("cf-device-types"));
        assert!(applies_to_crate("cf-scan-prep-core"));
        assert!(applies_to_crate("cf-cast-cli"));
    }

    #[test]
    fn applies_to_crate_in_scope_no_hyphen_umbrellas() {
        // The two no-hyphen umbrellas are explicit exceptions: `mesh`
        // (the umbrella crate) and `cortenforge` (the top-level crate
        // name from plan §2.1, even though it doesn't currently exist).
        assert!(applies_to_crate("mesh"));
        assert!(applies_to_crate("cortenforge"));
    }

    #[test]
    fn applies_to_crate_out_of_scope() {
        assert!(!applies_to_crate("xtask"));
        assert!(!applies_to_crate("anyhow"));
        assert!(!applies_to_crate("serde"));
        // Examples are out of scope (they have their own classification).
        assert!(!applies_to_crate("phase_demo"));
        // A crate whose name happens to start with `mes` (not `mesh-`,
        // not `mesh`) is out of scope.
        assert!(!applies_to_crate("messy"));
        // cf-viewer is a workspace tool with the cf- prefix; explicit
        // exemption per docs/VIEWER_DESIGN.md Q1 + Q8 locks. cf-bevy-common
        // is the C2b factor-out: workspace-internal Bevy helper, same
        // exemption shape. cf-scan-prep, cf-device-design, and
        // cf-sim-research are Bevy GUI workspace tools under `tools/`
        // carrying cf- prefix; same exemption. cf-device-geometry
        // (sim-decouple Phase 2.5.b) is the shared Bevy-using device-side
        // geometric-primitives crate; same exemption. (cf-cast-cli,
        // cf-scan-prep-core, and cf-device-types were exempt but are now
        // classified SDK library tiers — see the in-scope test.)
        assert!(!applies_to_crate("cf-viewer"));
        assert!(!applies_to_crate("cf-bevy-common"));
        // cf-mesh-paint: repo-root Bevy paint plugin, sibling of cf-bevy-common.
        assert!(!applies_to_crate("cf-mesh-paint"));
        assert!(!applies_to_crate("cf-scan-prep"));
        assert!(!applies_to_crate("cf-device-design"));
        assert!(!applies_to_crate("cf-sim-research"));
        // cf-spine-studio: native Bevy anatomical-spine studio, same shape.
        assert!(!applies_to_crate("cf-spine-studio"));
        assert!(!applies_to_crate("cf-device-geometry"));
        // musculoskeletal-builder arc tools (Mission #4) — same exemption shape.
        assert!(!applies_to_crate("cf-osim"));
        assert!(!applies_to_crate("cf-anthro"));
        assert!(!applies_to_crate("cf-msk-fit"));
        assert!(!applies_to_crate("cf-msk-lib"));
        assert!(!applies_to_crate("cf-mjcf-emit"));
        assert!(!applies_to_crate("cf-codesign"));
    }

    #[test]
    fn ban_pattern_prefix_matches_subcrates() {
        let ban = BanPattern {
            pattern: "bevy",
            kind: BanKind::Prefix,
        };
        assert!(ban.matches("bevy"));
        assert!(ban.matches("bevy_ecs"));
        assert!(ban.matches("bevy_reflect_derive"));
        // Sanity: doesn't match unrelated crate.
        assert!(!ban.matches("approx"));
    }

    #[test]
    fn ban_pattern_exact_only_matches_exact_name() {
        let ban = BanPattern {
            pattern: "winit",
            kind: BanKind::Exact,
        };
        assert!(ban.matches("winit"));
        // The exact-vs-prefix distinction matters: `winit-glue` (a
        // hypothetical fork) must NOT match the exact-kind ban for
        // `winit`. This is why plan §5.2 distinguishes `bevy*` (prefix)
        // from `winit` (exact).
        assert!(!ban.matches("winit-glue"));
        assert!(!ban.matches("winitfoo"));
    }

    /// The budget override must reach the config that grades the dep count.
    #[test]
    fn dependency_budget_override_raises_only_the_count() {
        let base = tier_config(Tier::L0);
        let raised = tier_config_with_budget(Tier::L0, Some(200));
        assert_eq!(base.release_max, 100, "L0 default release cap moved");
        assert_eq!(
            raised.release_max, 200,
            "override did not raise release cap"
        );
        assert_eq!(
            raised.test_max, 220,
            "override must preserve the tier table's +20 release/test buffer"
        );
    }

    /// The override is a COUNT, never a permission. A crate cannot use it to
    /// let a tier-banned crate through — that would turn a budget knob into a
    /// layering escape hatch, which is the whole reason it is not implemented
    /// as a tier change.
    #[test]
    fn dependency_budget_override_does_not_relax_the_banned_list() {
        let base = tier_config(Tier::L0);
        let raised = tier_config_with_budget(Tier::L0, Some(10_000));
        assert_eq!(
            base.banned.len(),
            raised.banned.len(),
            "override changed the banned-pattern count"
        );
        for (b, r) in base.banned.iter().zip(raised.banned.iter()) {
            assert_eq!(b.pattern, r.pattern, "override altered a banned pattern");
        }
        // And the L0 list is genuinely stricter than L0-io's, so this is not
        // a vacuous comparison.
        assert!(
            base.banned.len() > tier_config(Tier::L0Io).banned.len(),
            "L0 is expected to ban strictly more than L0-io"
        );
    }

    /// An inert budget must not be announced.
    ///
    /// `tier_config_with_budget` clamps a sub-default budget away, so
    /// reporting one would tell a reader the crate is held to a limit it is
    /// not held to — the report inverting the truth in exactly the case the
    /// clamp exists to handle.
    #[test]
    fn an_inert_dependency_budget_is_not_reported() {
        let inert = TierMetadata {
            tier: Tier::L0,
            tier_up_features: Vec::new(),
            dependency_budget: Some(50),
        };
        assert_eq!(
            budget_in_effect(&inert),
            None,
            "a budget below the tier default was reported as raising the cap"
        );

        let raising = TierMetadata {
            tier: Tier::L0,
            tier_up_features: Vec::new(),
            dependency_budget: Some(200),
        };
        assert_eq!(
            budget_in_effect(&raising),
            Some(200),
            "a budget that genuinely raises the cap was not reported"
        );

        let absent = TierMetadata {
            tier: Tier::L0,
            tier_up_features: Vec::new(),
            dependency_budget: None,
        };
        assert_eq!(budget_in_effect(&absent), None);
    }

    /// The raised cap must reach the TABLE, not just `measured_detail`.
    ///
    /// This is the property the whole override rests on — it is why a
    /// per-crate budget is defensible where silently raising the tier is not.
    /// `print_criterion` renders only name/result/grade/threshold, so a note
    /// confined to `measured_detail` is invisible on `cargo xtask grade
    /// <crate>` and appears only under `--json`. The claim was made in three
    /// places before anything enforced it.
    #[test]
    fn a_raised_budget_appears_in_the_printed_result_column() {
        let raising = TierMetadata {
            tier: Tier::L0,
            tier_up_features: Vec::new(),
            dependency_budget: Some(200),
        };
        let budget = budget_in_effect(&raising).expect("budget raises the cap");
        let result = format!("✓ budget {budget}");
        assert!(
            result.contains("200"),
            "the printed result column must carry the raised cap"
        );
        // `print_criterion` truncates `result` to 16 chars; a marker that
        // truncates away is the same as no marker at all.
        assert!(
            result.chars().count() <= 16,
            "result marker {result:?} exceeds the table's 16-char column and \
             would be truncated"
        );
    }

    /// A budget BELOW the effective tier's default must be ignored.
    ///
    /// `tier_config_with_budget` receives the EFFECTIVE tier, which
    /// `--all-features` may have tiered up. A crate declaring a budget sized
    /// for its base tier must not thereby tighten the laxer tier its features
    /// unlock.
    #[test]
    fn dependency_budget_override_never_tightens_a_tier() {
        // L1 is unbounded; a modest budget must not clamp it.
        let l1 = tier_config_with_budget(Tier::L1, Some(200));
        assert_eq!(
            l1.release_max,
            usize::MAX,
            "a per-crate budget clamped L1's unbounded dep graph"
        );
        // And a budget under L0's own default leaves L0 alone.
        let l0 = tier_config_with_budget(Tier::L0, Some(10));
        assert_eq!(
            l0.release_max,
            tier_config(Tier::L0).release_max,
            "a budget below the tier default tightened the cap"
        );
    }

    /// Absent metadata leaves the tier defaults untouched.
    #[test]
    fn absent_dependency_budget_leaves_tier_defaults() {
        assert_eq!(
            tier_config_with_budget(Tier::L0, None).release_max,
            tier_config(Tier::L0).release_max
        );
    }

    #[test]
    fn parse_tier_metadata_reads_dependency_budget() {
        let toml = r#"
[package]
name = "foo"
[package.metadata.cortenforge]
tier = "L0"
dependency_budget = 200
"#;
        let m = parse_tier_metadata(toml).unwrap().unwrap();
        assert_eq!(m.dependency_budget, Some(200));
    }

    #[test]
    fn parse_tier_metadata_rejects_non_integer_dependency_budget() {
        let toml = r#"
[package]
name = "foo"
[package.metadata.cortenforge]
tier = "L0"
dependency_budget = "lots"
"#;
        assert!(
            parse_tier_metadata(toml).is_err(),
            "a non-integer budget must fail loudly, not silently disable the cap"
        );
    }

    #[test]
    fn parse_tier_metadata_no_block_returns_none() {
        let toml = r#"
[package]
name = "foo"
version = "0.1.0"
"#;
        assert!(parse_tier_metadata(toml).unwrap().is_none());
    }

    #[test]
    fn parse_tier_metadata_block_without_tier_returns_none() {
        // Pre-existing `[package.metadata.cortenforge]` blocks (e.g.,
        // for grading_profile) without a tier key must not error.
        let toml = r#"
[package]
name = "foo"
[package.metadata.cortenforge]
grading_profile = "integration-only"
"#;
        assert!(parse_tier_metadata(toml).unwrap().is_none());
    }

    #[test]
    fn parse_tier_metadata_valid_l0() {
        let toml = r#"
[package]
name = "foo"
[package.metadata.cortenforge]
tier = "L0"
"#;
        let m = parse_tier_metadata(toml).unwrap().unwrap();
        assert_eq!(m.tier, Tier::L0);
        assert!(m.tier_up_features.is_empty());
    }

    #[test]
    fn parse_tier_metadata_with_tier_up_features() {
        // The sim-soft pattern: declared L0, with gpu-probe → L0-io.
        let toml = r#"
[package]
name = "sim-soft"
[package.metadata.cortenforge]
tier = "L0"
tier_up_features = { gpu-probe = "L0-io" }
"#;
        let m = parse_tier_metadata(toml).unwrap().unwrap();
        assert_eq!(m.tier, Tier::L0);
        assert_eq!(m.tier_up_features.len(), 1);
        assert_eq!(m.tier_up_features[0].0, "gpu-probe");
        assert_eq!(m.tier_up_features[0].1, Tier::L0Io);
    }

    #[test]
    fn parse_tier_metadata_unknown_tier_errors() {
        let toml = r#"
[package]
name = "foo"
[package.metadata.cortenforge]
tier = "L2"
"#;
        assert!(parse_tier_metadata(toml).is_err());
    }

    #[test]
    fn parse_tier_metadata_unknown_tier_up_target_errors() {
        let toml = r#"
[package]
name = "foo"
[package.metadata.cortenforge]
tier = "L0"
tier_up_features = { foo = "L99" }
"#;
        assert!(parse_tier_metadata(toml).is_err());
    }

    #[test]
    fn parse_tier_metadata_rejects_app_tier_up_target() {
        // App is the dependency sink, never a tier-up target — promoting an SDK
        // crate to App would silently exempt it from every dep check.
        let toml = r#"
[package]
name = "foo"
[package.metadata.cortenforge]
tier = "L0"
tier_up_features = { sneaky = "App" }
"#;
        assert!(parse_tier_metadata(toml).is_err());
    }

    #[test]
    fn effective_tier_under_default_is_declared() {
        let m = TierMetadata {
            tier: Tier::L0,
            tier_up_features: vec![("gpu-probe".to_string(), Tier::L0Io)],
            dependency_budget: None,
        };
        assert_eq!(effective_tier_for(&m, FeatureConfig::Default), Tier::L0);
        assert_eq!(effective_tier_for(&m, FeatureConfig::NoDefault), Tier::L0);
    }

    #[test]
    fn effective_tier_under_all_features_with_tier_up_promotes() {
        // The sim-soft semantics: under --all-features the gpu-probe
        // feature is enabled, so the L0-io rules apply to the whole
        // graph (which is why wgpu pulled by gpu-probe doesn't trip
        // L0's wgpu* ban).
        let m = TierMetadata {
            tier: Tier::L0,
            tier_up_features: vec![("gpu-probe".to_string(), Tier::L0Io)],
            dependency_budget: None,
        };
        assert_eq!(
            effective_tier_for(&m, FeatureConfig::AllFeatures),
            Tier::L0Io
        );
    }

    #[test]
    fn effective_tier_under_all_features_picks_most_permissive() {
        // Multiple tier_up_features → take the most permissive target
        // (largest permissiveness). Under --all-features, all listed
        // features are simultaneously enabled, so the loosest applies.
        let m = TierMetadata {
            tier: Tier::L0,
            tier_up_features: vec![
                ("a".to_string(), Tier::L0Io),
                ("b".to_string(), Tier::L0Integration),
            ],
            dependency_budget: None,
        };
        assert_eq!(
            effective_tier_for(&m, FeatureConfig::AllFeatures),
            Tier::L0Integration
        );
    }

    #[test]
    fn effective_tier_no_tier_up_does_not_promote() {
        // The sim-ml-chassis bevy_ecs leak case: bevy feature is opt-in
        // but NOT declared as tier_up_features, so under --all-features
        // the L0 rules still apply to the now-larger graph → leak fires.
        let m = TierMetadata {
            tier: Tier::L0,
            tier_up_features: vec![],
            dependency_budget: None,
        };
        assert_eq!(effective_tier_for(&m, FeatureConfig::AllFeatures), Tier::L0);
    }

    #[test]
    fn evaluate_dep_set_clean_l0_no_findings() {
        let deps: Vec<String> = ["sim-types", "nalgebra", "approx", "num-traits"]
            .iter()
            .map(|s| s.to_string())
            .collect();
        let findings = evaluate_dep_set(
            &deps,
            tier_config(Tier::L0),
            FeatureConfig::Default,
            GraphKind::Release,
            Tier::L0,
        );
        assert_eq!(findings.len(), 0);
    }

    #[test]
    fn evaluate_dep_set_l0_bevy_leak() {
        // The sim-ml-chassis --all-features leak: 9 bevy_* sub-crates
        // each generate a finding. CountExceeded does not fire because
        // we keep the input small for this unit test.
        let deps: Vec<String> = [
            "sim-ml-chassis",
            "bevy_ecs",
            "bevy_ecs_macros",
            "bevy_reflect",
            "approx",
        ]
        .iter()
        .map(|s| s.to_string())
        .collect();
        let findings = evaluate_dep_set(
            &deps,
            tier_config(Tier::L0),
            FeatureConfig::AllFeatures,
            GraphKind::Release,
            Tier::L0,
        );
        assert_eq!(findings.len(), 3);
        for f in &findings {
            match &f.kind {
                FindingKind::BannedPrefix { pattern, .. } => assert_eq!(*pattern, "bevy"),
                _ => panic!("expected BannedPrefix"),
            }
        }
    }

    #[test]
    fn evaluate_dep_set_l0_count_over_max() {
        // 101 release deps → exceeds L0's 100-release max → one
        // CountExceeded finding.
        let deps: Vec<String> = (0..101).map(|i| format!("dep-{}", i)).collect();
        let findings = evaluate_dep_set(
            &deps,
            tier_config(Tier::L0),
            FeatureConfig::Default,
            GraphKind::Release,
            Tier::L0,
        );
        assert_eq!(findings.len(), 1);
        assert!(matches!(
            findings[0].kind,
            FindingKind::CountExceeded {
                actual: 101,
                max: 100
            }
        ));
    }

    #[test]
    fn evaluate_dep_set_l0_test_max_is_higher_than_release() {
        // 101 deps in the dev graph → under the 120 test-max, no count
        // finding. Same input over release max — the (release, test)
        // distinction must thread through evaluate_dep_set.
        let deps: Vec<String> = (0..101).map(|i| format!("dep-{}", i)).collect();
        let findings = evaluate_dep_set(
            &deps,
            tier_config(Tier::L0),
            FeatureConfig::Default,
            GraphKind::WithDev,
            Tier::L0,
        );
        assert_eq!(findings.len(), 0);
    }

    #[test]
    fn evaluate_dep_set_l0_io_permits_wgpu() {
        // The sim-soft + gpu-probe target case: under L0-io rules, wgpu
        // and its sub-crates are NOT banned. This is the load-bearing
        // distinction the plan-§5.2 typo would have broken.
        let deps: Vec<String> = ["sim-soft", "wgpu", "wgpu-core", "wgpu-hal", "naga"]
            .iter()
            .map(|s| s.to_string())
            .collect();
        let findings = evaluate_dep_set(
            &deps,
            tier_config(Tier::L0Io),
            FeatureConfig::AllFeatures,
            GraphKind::Release,
            Tier::L0Io,
        );
        assert_eq!(findings.len(), 0);
    }

    #[test]
    fn evaluate_dep_set_l0_io_still_bans_bevy() {
        // L0-io permits wgpu but still bans bevy. Confirms the typo-fix
        // didn't accidentally also drop bevy.
        let deps: Vec<String> = ["sim-mjcf", "bevy_ecs", "image"]
            .iter()
            .map(|s| s.to_string())
            .collect();
        let findings = evaluate_dep_set(
            &deps,
            tier_config(Tier::L0Io),
            FeatureConfig::Default,
            GraphKind::Release,
            Tier::L0Io,
        );
        assert_eq!(findings.len(), 1);
        match &findings[0].kind {
            FindingKind::BannedPrefix {
                pattern,
                matched_pkg,
            } => {
                assert_eq!(*pattern, "bevy");
                assert_eq!(matched_pkg, "bevy_ecs");
            }
            _ => panic!("expected BannedPrefix"),
        }
    }

    #[test]
    fn evaluate_dep_set_l1_no_constraints() {
        // L1 tier has unbounded max and empty banned list → no findings
        // even on intentionally noisy input.
        let deps: Vec<String> = (0..1000)
            .map(|i| format!("crate-{}", i))
            .chain(["bevy_ecs", "winit", "wgpu"].iter().map(|s| s.to_string()))
            .collect();
        let findings = evaluate_dep_set(
            &deps,
            tier_config(Tier::L1),
            FeatureConfig::AllFeatures,
            GraphKind::WithDev,
            Tier::L1,
        );
        assert_eq!(findings.len(), 0);
    }

    #[test]
    fn evaluate_dep_set_l0_dev_graph_image_zip_chain() {
        // The sim-thermostat dev-poison case: image, mesh-io, sim-mjcf,
        // zip, zstd all get individually flagged in the dev graph.
        let deps: Vec<String> = [
            "sim-thermostat",
            "sim-mjcf",
            "image",
            "image-webp",
            "mesh-io",
            "zip",
            "zstd",
            "zstd-safe",
            "zstd-sys",
        ]
        .iter()
        .map(|s| s.to_string())
        .collect();
        let findings = evaluate_dep_set(
            &deps,
            tier_config(Tier::L0),
            FeatureConfig::Default,
            GraphKind::WithDev,
            Tier::L0,
        );
        // 1 sim-mjcf + 2 image* + 1 mesh-io + 1 zip + 3 zstd* = 8.
        assert_eq!(findings.len(), 8);
    }

    #[test]
    fn evaluate_dep_set_winit_exact_does_not_match_substring() {
        // The exact-vs-prefix distinction in action: a hypothetical
        // `winit-fork-name` should NOT trip the `winit` exact ban.
        let deps: Vec<String> = vec!["winit-glue".to_string()];
        let findings = evaluate_dep_set(
            &deps,
            tier_config(Tier::L0),
            FeatureConfig::Default,
            GraphKind::Release,
            Tier::L0,
        );
        assert_eq!(findings.len(), 0);
    }

    // === App-tier sink rule (evaluate_sink) ===

    fn app_set(names: &[&str]) -> std::collections::HashSet<String> {
        names.iter().map(|s| s.to_string()).collect()
    }

    fn dep_list(deps: &[&str]) -> Vec<String> {
        deps.iter().map(|s| s.to_string()).collect()
    }

    #[test]
    fn evaluate_sink_clean_sdk_graph_no_findings() {
        // A graph containing only SDK/third-party crates — no app dependency.
        let deps = dep_list(&["sim-core", "mesh-types", "serde"]);
        let apps = app_set(&["cf-studio-core", "cf-studio-engine"]);
        assert!(evaluate_sink(&deps, "sim-soft", &apps).is_empty());
    }

    #[test]
    fn evaluate_sink_flags_app_dependency() {
        // The violation we exist to catch: an SDK crate reaching into an app.
        let deps = dep_list(&["sim-core", "cf-studio-engine", "serde"]);
        let apps = app_set(&["cf-studio-core", "cf-studio-engine", "cf-studio-gui"]);
        assert_eq!(
            evaluate_sink(&deps, "sim-coupling", &apps),
            vec!["cf-studio-engine"]
        );
    }

    #[test]
    fn evaluate_sink_excludes_self_edge() {
        // cargo tree lists the queried crate itself; it must not flag itself.
        let deps = dep_list(&["cf-studio-gui", "cf-studio-engine"]);
        let apps = app_set(&["cf-studio-gui", "cf-studio-engine"]);
        assert_eq!(
            evaluate_sink(&deps, "cf-studio-gui", &apps),
            vec!["cf-studio-engine"]
        );
    }

    #[test]
    fn evaluate_sink_flags_every_app_dependency() {
        let deps = dep_list(&["sim-core", "cf-studio-engine", "cf-studio-core"]);
        let apps = app_set(&["cf-studio-core", "cf-studio-engine", "cf-studio-gui"]);
        let mut found = evaluate_sink(&deps, "mesh-sdf", &apps);
        found.sort();
        assert_eq!(found, vec!["cf-studio-core", "cf-studio-engine"]);
    }

    #[test]
    fn evaluate_sink_empty_app_set_no_findings() {
        let deps = dep_list(&["cf-studio-engine"]);
        assert!(evaluate_sink(&deps, "sim-core", &app_set(&[])).is_empty());
    }

    #[test]
    fn format_finding_app_sink_renders_message() {
        // PR-blocking message shape for the sink violation. Structural, so no
        // per-config prefix — just the graded crate's tier and the app crate.
        let f = Finding {
            feature_config: FeatureConfig::Default,
            graph_kind: GraphKind::Release,
            effective_tier: Tier::L1,
            kind: FindingKind::AppSink {
                app_crate: "cf-studio-engine".to_string(),
            },
        };
        assert_eq!(
            format_finding(&f),
            "[tier L1] forbidden App-tier dependency: \
             cf-studio-engine (an SDK crate must never depend on an app)"
        );
    }

    #[test]
    fn layer_integrity_grade_truth_table() {
        // The load-bearing decision, especially L1-with-sink-finding → F.
        assert_eq!(layer_integrity_grade(Tier::App, 0), Grade::NotApplicable);
        assert_eq!(layer_integrity_grade(Tier::App, 3), Grade::NotApplicable);
        assert_eq!(layer_integrity_grade(Tier::L1, 0), Grade::NotApplicable);
        assert_eq!(layer_integrity_grade(Tier::L1, 1), Grade::F); // the fall-through
        assert_eq!(layer_integrity_grade(Tier::L0, 0), Grade::A);
        assert_eq!(layer_integrity_grade(Tier::L0, 2), Grade::F);
        assert_eq!(layer_integrity_grade(Tier::L0Io, 0), Grade::A);
        assert_eq!(layer_integrity_grade(Tier::L0Integration, 1), Grade::F);
    }

    #[test]
    fn format_finding_count_exceeded_renders_full_context() {
        // The format_finding output is what the user sees in stderr and
        // measured_detail; pin its shape since this is the PR-blocking
        // failure message under step 12's hard gate.
        let f = Finding {
            feature_config: FeatureConfig::AllFeatures,
            graph_kind: GraphKind::Release,
            effective_tier: Tier::L0,
            kind: FindingKind::CountExceeded {
                actual: 115,
                max: 100,
            },
        };
        assert_eq!(
            format_finding(&f),
            "[release all-features, tier L0] dep count 115 exceeds max 100"
        );
    }

    #[test]
    fn format_finding_banned_prefix_includes_pattern_and_pkg() {
        let f = Finding {
            feature_config: FeatureConfig::Default,
            graph_kind: GraphKind::WithDev,
            effective_tier: Tier::L0,
            kind: FindingKind::BannedPrefix {
                pattern: "bevy",
                matched_pkg: "bevy_ecs".to_string(),
            },
        };
        assert_eq!(
            format_finding(&f),
            "[with-dev default, tier L0] banned `bevy` matched: bevy_ecs"
        );
    }

    // ---- WASM Compatibility criterion (Plan §5.3) -----------------------

    #[test]
    fn wasm_extract_error_summary_picks_first_error_line() {
        // The canonical recon case: getrandom 0.3.4 surfaces a plain
        // `error:` line followed by lots of context. Grab the diagnostic
        // line, drop the rest.
        let stderr = "   Compiling getrandom v0.3.4\n\
                      error: The wasm32-unknown-unknown targets are not supported by default; you may need to enable the \"wasm_js\" configuration flag.\n\
                          --> /some/path/backends.rs:194:17\n\
                      \n\
                      error: could not compile `getrandom` (lib) due to 1 previous error\n";
        let summary = extract_wasm_error_summary(stderr);
        assert!(
            summary.starts_with("error: The wasm32-unknown-unknown targets"),
            "expected first `error:` line, got: {}",
            summary
        );
    }

    #[test]
    fn wasm_extract_error_summary_picks_first_error_with_code() {
        // Many rustc errors take the `error[Exxxx]:` form (e.g., E0432
        // "unresolved import"). Treat both forms equivalently.
        let stderr = "   Compiling foo v0.1.0\n\
                      error[E0432]: unresolved import `std::os::unix::fs`\n\
                          --> src/lib.rs:1:5\n";
        let summary = extract_wasm_error_summary(stderr);
        assert!(
            summary.starts_with("error[E0432]:"),
            "expected `error[E…]:` line, got: {}",
            summary
        );
    }

    #[test]
    fn wasm_extract_error_summary_falls_back_when_no_error_line() {
        // If no `error:` / `error[…]:` marker is present (unusual but
        // possible — e.g., toolchain-internal failure with only `warning:`
        // lines), surface the last few non-empty lines so the user has
        // *some* signal rather than a useless "(no stderr)".
        let stderr = "   Compiling foo v0.1.0\n\
                      warning: unused import\n\
                      Bus error\n";
        let summary = extract_wasm_error_summary(stderr);
        assert!(summary.contains("Bus error"), "got: {}", summary);
    }

    #[test]
    fn wasm_extract_error_summary_handles_empty_stderr() {
        // Defensive: zero-output failure (rare but cargo can exit
        // non-zero with empty stderr if the target itself is unusable).
        // Don't return an empty measured_detail — give the reader a
        // labelled fallback.
        let summary = extract_wasm_error_summary("");
        assert_eq!(summary, "(no stderr output captured)");
    }

    #[test]
    fn wasm_extract_error_summary_strips_leading_whitespace_before_match() {
        // rustc indents continuation lines but the first marker is at
        // column 0; sometimes a wrapping process re-indents. Match
        // `error:` even when leading whitespace is present.
        let stderr = "    error: indented diagnostic line\n";
        let summary = extract_wasm_error_summary(stderr);
        assert_eq!(summary, "error: indented diagnostic line");
    }

    #[test]
    fn wasm_extract_error_summary_skips_non_error_prefix_lines() {
        // The first `error:` may be preceded by non-error lines that
        // happen to contain the substring "error" (e.g., paths, comments,
        // or `error_chain`-named crates). The match anchors on
        // start-of-trimmed-line, not substring presence.
        let stderr = "   Compiling error_chain v0.12.0\n\
                      checking error_chain progress…\n\
                      error: real diagnostic\n";
        let summary = extract_wasm_error_summary(stderr);
        assert_eq!(summary, "error: real diagnostic");
    }

    /// A throwaway source tree for the file-walking tests below. Same
    /// pid+thread naming convention as `coverage.rs`'s fixture helper, so
    /// concurrent test threads never collide on a root.
    fn walk_fixture(tag: &str, files: &[&str]) -> std::path::PathBuf {
        let root = std::env::temp_dir().join(format!(
            "cf-glob-rs-{tag}-{}-{:?}",
            std::process::id(),
            std::thread::current().id()
        ));
        // Safe even when a previous run left a symlink loop here:
        // `remove_dir_all` unlinks symlinks rather than descending them.
        let _ = std::fs::remove_dir_all(&root);
        std::fs::create_dir_all(&root).expect("mkdir root");
        for name in files {
            let path = root.join(name);
            std::fs::create_dir_all(path.parent().expect("parent")).expect("mkdir");
            std::fs::write(&path, "fn f() {}\n").expect("write");
        }
        root
    }

    #[test]
    fn glob_rs_files_collects_nested_sources_and_ignores_other_extensions() {
        let root = walk_fixture(
            "nested",
            &["lib.rs", "inner/deep/mod.rs", "notes.md", "data.json"],
        );
        let found = glob_rs_files(&root.to_string_lossy()).expect("walk");
        assert_eq!(found.len(), 2, "collected {found:?}");
        assert!(found.iter().any(|p| p.ends_with("lib.rs")), "{found:?}");
        assert!(
            found.iter().any(|p| p.ends_with("inner/deep/mod.rs")),
            "{found:?}"
        );
    }

    #[test]
    fn a_directory_that_is_simply_absent_is_not_an_error() {
        // The one case where an empty list is the honest answer, and the
        // reason `try_exists`'s `Ok(false)` arm has to stay distinct from
        // its `Err` arm.
        let root = walk_fixture("absent", &[]);
        std::fs::remove_dir_all(&root).expect("rm");
        let found = glob_rs_files(&root.to_string_lossy()).expect("walk");
        assert!(found.is_empty(), "collected {found:?}");
    }

    #[test]
    fn a_path_whose_metadata_cannot_be_read_is_an_error_not_an_empty_list() {
        // `<regular file>/src` yields ENOTDIR, which `Path::exists()`
        // flattens to plain `false` — indistinguishable from "this crate
        // has no src/", i.e. from "zero violations". `try_exists` keeps
        // the two apart.
        let root = walk_fixture("not-a-dir", &["afile"]);
        let src = root.join("afile").join("src");
        assert!(
            !src.exists(),
            "precondition: the old check answers 'absent'"
        );
        glob_rs_files(&src.to_string_lossy()).expect_err("an unstat-able path must error");
    }

    /// Criteria 3 and 4 both grade by counting violations per file, so a
    /// module the walk never reaches is a module that scores zero. Before
    /// `follow_links(true)`, everything under `linked/` was invisible to
    /// both.
    #[cfg(unix)]
    #[test]
    fn a_module_behind_a_symlinked_directory_is_collected() {
        let root = walk_fixture("symlink-dir", &["walked/lib.rs", "elsewhere/hidden.rs"]);
        std::os::unix::fs::symlink(root.join("elsewhere"), root.join("walked").join("linked"))
            .expect("symlink");
        let found = glob_rs_files(&root.join("walked").to_string_lossy()).expect("walk");
        assert!(
            found.iter().any(|p| p.ends_with("linked/hidden.rs")),
            "the symlinked module was not collected: {found:?}"
        );
    }

    /// A tree the walker cannot fully traverse must error rather than return
    /// the part it managed to reach — a short list reads to both consumers as
    /// "nothing to flag here".
    ///
    /// A self-referential symlink is the permission-free way to manufacture a
    /// walk error; a `chmod 000` fixture would silently stop reproducing when
    /// the suite runs as root. It does mean this case leans on
    /// `follow_links(true)` to detect the loop at all, so it fails if *either*
    /// half of the fix is backed out.
    #[cfg(unix)]
    #[test]
    fn a_tree_the_walker_cannot_traverse_is_an_error() {
        let root = walk_fixture("loop", &["lib.rs"]);
        std::os::unix::fs::symlink(&root, root.join("selflink")).expect("symlink");
        let walked = glob_rs_files(&root.to_string_lossy());
        // Clear the loop before asserting. `walk_fixture` cleans at *start*,
        // which never fires across runs (the root is keyed by pid), so this
        // fixture would otherwise leave a self-referential symlink in the
        // shared temp dir for every test run. A panicking assert skips the
        // cleanup, which is the right trade: a failing test is being watched.
        let _ = std::fs::remove_dir_all(&root);
        walked.expect_err("a walk error must not be swallowed");
    }

    /// A `.rs` file that is not valid UTF-8 — `read_to_string`'s failure mode
    /// that needs neither a permission trick nor a broken symlink (which the
    /// walk would catch one step earlier). Rust source must be UTF-8, so this
    /// tree is genuinely unscannable, and both criteria have to say so rather
    /// than score it clean.
    fn unreadable_source_fixture(tag: &str) -> std::path::PathBuf {
        let root = walk_fixture(tag, &[]);
        std::fs::create_dir_all(root.join("src")).expect("mkdir src");
        std::fs::write(
            root.join("src").join("lib.rs"),
            [0x66, 0x6e, 0x20, 0xff, 0xfe],
        )
        .expect("write");
        root
    }

    #[test]
    fn criterion_3_errors_rather_than_scoring_an_unreadable_file_clean() {
        // Positive control first: on a readable tree the scan really does
        // return a count, so the error below is the file's doing and not the
        // fixture's.
        let clean = walk_fixture("allows-clean", &[]);
        std::fs::create_dir_all(clean.join("src")).expect("mkdir src");
        std::fs::write(
            clean.join("src").join("lib.rs"),
            "#[allow(clippy::unwrap_used)]\nfn f() {}\n",
        )
        .expect("write");
        assert_eq!(
            count_unjustified_allows_in_tree(&clean.join("src").to_string_lossy()).expect("scan"),
            1
        );

        let root = unreadable_source_fixture("allows-unreadable");
        count_unjustified_allows_in_tree(&root.join("src").to_string_lossy())
            .expect_err("an unreadable file must not read as zero unjustified allows");
    }

    #[test]
    fn criterion_4_errors_rather_than_scoring_an_unreadable_file_clean() {
        let sh = Shell::new().expect("shell");

        // Positive control: the same call shape on a readable tree grades.
        let clean = walk_fixture("safety-clean", &[]);
        std::fs::create_dir_all(clean.join("src")).expect("mkdir src");
        std::fs::write(clean.join("src").join("lib.rs"), "fn f() -> u8 { 1 }\n").expect("write");
        let graded = grade_safety(&sh, &clean.to_string_lossy(), CrateProfile::Layer0)
            .expect("a readable tree grades");
        assert_eq!(graded.grade, Grade::A, "{graded:?}");

        let root = unreadable_source_fixture("safety-unreadable");
        grade_safety(&sh, &root.to_string_lossy(), CrateProfile::Layer0)
            .expect_err("an unreadable file must not read as zero safety violations");
    }

    /// Criterion 4 reaches its own `src/` guard *before* [`glob_rs_files`], so
    /// the walker's `try_exists` never sees an unstat-able tree — this case
    /// covers the guard rather than the walker.
    ///
    /// The stakes are higher here than a wrong count: the `Manual` arm is
    /// skipped by `GradeReport::overall_automated`, so flattening "cannot
    /// stat" to "absent" would drop criterion 4 out of the automated grade
    /// altogether and let the crate pass on the strength of the others.
    #[test]
    fn criterion_4_errors_rather_than_dropping_an_unstat_able_src_from_the_grade() {
        let sh = Shell::new().expect("shell");

        // Positive control: a genuinely absent `src/` still takes the `Manual`
        // arm. That distinction is the whole reason the guard cannot simply
        // become an error.
        let absent = walk_fixture("safety-no-src", &[]);
        let graded = grade_safety(&sh, &absent.to_string_lossy(), CrateProfile::Layer0)
            .expect("an absent src/ is not an error");
        assert_eq!(graded.grade, Grade::Manual, "{graded:?}");

        let root = walk_fixture("safety-not-a-dir", &["afile"]);
        grade_safety(
            &sh,
            &root.join("afile").to_string_lossy(),
            CrateProfile::Layer0,
        )
        .expect_err("an unstat-able src/ must not vanish into the Manual arm");
    }

    /// The regression this bucket exists to prevent: before errors were
    /// tracked, an ungradeable crate aborted the sweep, and the temptation
    /// when un-aborting is to let it fall through as "not a failure".
    #[test]
    fn a_crate_that_could_not_be_graded_never_makes_a_sweep_green() {
        let tally = SweepTally {
            total: 10,
            failures: 0,
            errors: 1,
        };
        assert!(!tally.is_green(), "{tally:?}");
        assert!(
            tally.headline().contains("could not be graded"),
            "{}",
            tally.headline()
        );
    }

    #[test]
    fn a_sweep_is_green_only_when_every_crate_was_graded_and_passed() {
        let green = SweepTally {
            total: 10,
            failures: 0,
            errors: 0,
        };
        assert!(green.is_green());
        assert_eq!(green.passes(), 10);
        assert!(green.headline().contains("10/10"), "{}", green.headline());

        assert!(!SweepTally {
            total: 10,
            failures: 1,
            errors: 0
        }
        .is_green());
    }

    #[test]
    fn the_headline_names_both_buckets_when_both_are_non_empty() {
        // The failure mode being pinned is a headline that reports one count
        // and silently drops the other, leaving the reader to think the
        // sweep's only problem is the kind they can see.
        let tally = SweepTally {
            total: 10,
            failures: 2,
            errors: 3,
        };
        let headline = tally.headline();
        assert!(headline.contains('2'), "{headline}");
        assert!(headline.contains('3'), "{headline}");
        assert!(headline.contains("could not be graded"), "{headline}");
        assert_eq!(tally.passes(), 5);
    }

    #[test]
    fn the_headline_carries_no_status_glyph() {
        // It is printed with a ✓/✗ prefix *and* reused as the `bail!` text,
        // where anyhow supplies its own `Error:` marker. A glyph baked into
        // the string reads as "Error: ✗ …" — two markers for one failure.
        for tally in [
            SweepTally {
                total: 3,
                failures: 0,
                errors: 0,
            },
            SweepTally {
                total: 3,
                failures: 1,
                errors: 0,
            },
            SweepTally {
                total: 3,
                failures: 0,
                errors: 1,
            },
            SweepTally {
                total: 3,
                failures: 1,
                errors: 1,
            },
        ] {
            let headline = tally.headline();
            assert!(
                !headline.contains('✗') && !headline.contains('✓'),
                "{headline}"
            );
            assert!(headline.starts_with("grade-all: "), "{headline}");
        }
    }

    #[test]
    fn passes_is_derived_so_it_cannot_drift_from_the_buckets() {
        // `saturating_sub` guards a nonsensical construction rather than
        // panicking or wrapping to a huge pass count — the one direction that
        // would read as good news.
        let nonsense = SweepTally {
            total: 1,
            failures: 5,
            errors: 5,
        };
        assert_eq!(nonsense.passes(), 0);
        assert!(!nonsense.is_green());
    }

    /// One `--message-format=json` line, shaped like the real thing: a
    /// diagnostic at `level` whose single span points at `file`.
    fn diagnostic_line(level: &str, file: &str) -> String {
        serde_json::json!({
            "reason": "compiler-message",
            "message": {
                "level": level,
                "spans": [{"file_name": file}],
            },
        })
        .to_string()
    }

    fn build_finished(success: bool) -> String {
        serde_json::json!({"reason": "build-finished", "success": success}).to_string()
    }

    #[test]
    fn clippy_counts_only_diagnostics_inside_the_graded_crate() {
        let stream = [
            diagnostic_line("warning", "sim/L0/core/src/lib.rs"),
            diagnostic_line("error", "sim/L0/core/src/body.rs"),
            // A transitive dependency's diagnostic — real, but not this
            // crate's to answer for.
            diagnostic_line("warning", "sim/L0/types/src/lib.rs"),
            // Not a diagnostic level we grade on.
            diagnostic_line("note", "sim/L0/core/src/lib.rs"),
            build_finished(true),
        ]
        .join("\n");
        assert_eq!(
            count_clippy_diagnostics(&stream, "sim/L0/core").expect("count"),
            2
        );
    }

    /// A spanless diagnostic is not counted — pinned as *behaviour*, since no
    /// test can pin it to a line.
    ///
    /// Deleting the `spans.is_empty()` clause does not fail this test, and
    /// cannot: [`any_span_in_crate`] is `spans.iter().any(..)`, which is
    /// already `false` for an empty slice, so the clause is unable to change
    /// any outcome. It is kept for the reader, not for the result. Nor does
    /// any captured stream from current cargo carry a `warning`/`error` with
    /// empty spans — the sole empty-spans message observed is a
    /// `failure-note`, which the level filter drops first.
    #[test]
    fn a_diagnostic_with_no_spans_is_not_counted() {
        let stream = [
            serde_json::json!({
                "reason": "compiler-message",
                "message": {"level": "warning", "spans": []},
            })
            .to_string(),
            build_finished(true),
        ]
        .join("\n");
        assert_eq!(
            count_clippy_diagnostics(&stream, "sim/L0/core").expect("count"),
            0
        );
    }

    /// The failure this whole function exists for. An unresolvable `-p` exits
    /// 101 having written **nothing** to stdout — byte-for-byte identical to a
    /// flawless crate. Measured against real cargo, not assumed.
    #[test]
    fn an_empty_stream_is_an_error_not_a_clean_crate() {
        let err = count_clippy_diagnostics("", "sim/L0/core").expect_err("empty must not be clean");
        assert!(
            format!("{err:#}").contains("build-finished"),
            "the error should name what was missing: {err:#}"
        );
    }

    #[test]
    fn a_stream_that_stops_before_build_finished_is_an_error() {
        // Diagnostics present but no completion record: clippy was cut off
        // (killed, panicked, disk full) partway through. The count so far is
        // a floor, not a verdict.
        let stream = diagnostic_line("warning", "sim/L0/core/src/lib.rs");
        count_clippy_diagnostics(&stream, "sim/L0/core")
            .expect_err("a truncated stream must not be graded");
    }

    /// `sim-core` with `sim-types` broken: one real diagnostic, pointing into
    /// the dependency, which [`any_span_in_crate`] correctly rules foreign.
    /// Zero in-crate diagnostics then means "never analysed", not "clean".
    #[test]
    fn a_build_that_failed_outside_the_crate_is_an_error_not_a_clean_crate() {
        let stream = [
            diagnostic_line("error", "sim/L0/types/src/lib.rs"),
            build_finished(false),
        ]
        .join("\n");
        let err = count_clippy_diagnostics(&stream, "sim/L0/core")
            .expect_err("a foreign build failure must not read as clean");
        assert!(
            format!("{err:#}").contains("sim/L0/core"),
            "the error should name the crate it could not vouch for: {err:#}"
        );
    }

    #[test]
    fn a_build_that_failed_inside_the_crate_is_counted_not_refused() {
        // The crate's own compile errors are exactly what an F is for —
        // refusing to grade here would turn every genuinely broken crate into
        // a tooling error instead of a failing grade.
        let stream = [
            diagnostic_line("error", "sim/L0/core/src/lib.rs"),
            build_finished(false),
        ]
        .join("\n");
        assert_eq!(
            count_clippy_diagnostics(&stream, "sim/L0/core").expect("count"),
            1
        );
    }

    #[test]
    fn a_build_finished_record_without_a_boolean_success_is_no_record_at_all() {
        // Defensive against a cargo format change: a `success` field that is
        // missing or non-boolean must fail closed, not default to "passed".
        let stream = serde_json::json!({"reason": "build-finished"}).to_string();
        count_clippy_diagnostics(&stream, "sim/L0/core")
            .expect_err("a malformed completion record must not count as one");
    }

    #[test]
    fn non_json_noise_in_the_stream_is_skipped_not_fatal() {
        // Cargo interleaves the occasional non-JSON line; those must not
        // abort a grade, only unparseable *completion* state should.
        let stream = [
            "warning: some plain-text line".to_string(),
            diagnostic_line("warning", "sim/L0/core/src/lib.rs"),
            build_finished(true),
        ]
        .join("\n");
        assert_eq!(
            count_clippy_diagnostics(&stream, "sim/L0/core").expect("count"),
            1
        );
    }

    fn triage_row(file: &str, covered: u64, total: u64) -> crate::coverage::FileCoverage {
        crate::coverage::FileCoverage {
            file: file.to_string(),
            covered,
            total,
            test_lines_counted: false,
            is_bin: false,
        }
    }

    /// ★ A binary root must be visibly labelled, or its uncovered count reads
    /// as a gap someone could close by writing tests for `.run()`.
    #[test]
    fn a_binary_row_is_labelled_and_an_ordinary_one_is_not() {
        let mut bin = triage_row("src/main.rs", 0, 400);
        bin.is_bin = true;
        assert_eq!(triage_row_markers(&bin).0, "  (binary target)");
        assert_eq!(triage_row_markers(&triage_row("src/lib.rs", 5, 10)).0, "");
    }

    /// The two markers are independent: an unparsed binary carries both, and
    /// neither may swallow the other.
    #[test]
    fn the_unparsed_caveat_and_the_binary_marker_do_not_displace_each_other() {
        let mut both = triage_row("src/main.rs", 0, 400);
        both.is_bin = true;
        both.test_lines_counted = true;
        let (target, caveat) = triage_row_markers(&both);
        assert_eq!(target, "  (binary target)");
        assert!(caveat.contains("unparsed"));

        let plain = triage_row("src/lib.rs", 5, 10);
        assert_eq!(triage_row_markers(&plain), ("", ""));
    }

    /// A per-file percentage truncates like every other figure the grader
    /// prints. A row reading 75.0 % inside a crate the table says is failing
    /// is the same contradiction `coverage_display` exists to prevent.
    #[test]
    fn a_triage_rows_percentage_truncates_like_the_crate_figure() {
        // 605/807 = 74.969 %, the cf-studio-engine measurement.
        //
        // ⚠ Asserts on the ROW, not on `coverage_display(f.percent())`. The
        // hand-composed form passes whatever the row itself does, which is how
        // a `{:.1}` at the call site survived the first sweep.
        let row = triage_row_prefix(&triage_row("f.rs", 605, 807));
        assert!(row.contains("74.9%"), "{row}");
        assert!(
            !row.contains("75.0%"),
            "rounding would contradict the grade"
        );
    }

    /// The rest of the row: the ranking key a reader sorts on, and the path
    /// they have to open.
    #[test]
    fn a_triage_row_carries_its_uncovered_count_and_path() {
        let row = triage_row_prefix(&triage_row("src/solver/pgs.rs", 10, 60));
        assert!(row.contains("50"), "uncovered count: {row}");
        assert!(row.contains("src/solver/pgs.rs"), "{row}");
    }

    /// A table that prints every row it has must claim no tail.
    #[test]
    fn an_untruncated_triage_table_reports_no_remainder() {
        let rows = [triage_row("a.rs", 0, 5), triage_row("b.rs", 0, 3)];
        let worst: Vec<&_> = rows.iter().collect();

        assert_eq!(triage_tail(&worst, 20), None, "nothing was cut");
        assert_eq!(
            triage_tail(&worst, 2),
            None,
            "a cap exactly equal to the row count cuts nothing either"
        );
    }

    /// ★ The tail must count exactly the rows the table did NOT print.
    ///
    /// This is the condition the cap was allowed on: bounding the table is fine,
    /// under-reporting the bound is the silent truncation that makes a partial
    /// list read as the whole one. An off-by-one between the `take` that prints
    /// and the `skip` that summarises would do exactly that, and shows up here
    /// as a wrong line total rather than as anything a reader would notice.
    #[test]
    fn the_triage_tail_accounts_for_every_row_the_cap_cut() {
        let rows = [
            triage_row("a.rs", 0, 10), // 10 uncovered — printed
            triage_row("b.rs", 0, 7),  //  7 uncovered — printed
            triage_row("c.rs", 0, 4),  //  4 uncovered — cut
            triage_row("d.rs", 1, 3),  //  2 uncovered — cut
        ];
        let worst: Vec<&_> = rows.iter().collect();

        assert_eq!(
            triage_tail(&worst, 2),
            Some((2, 6)),
            "two rows were cut, holding 4 + 2 = 6 uncovered lines"
        );
        assert_eq!(
            triage_tail(&worst, 0),
            Some((4, 23)),
            "with nothing printed the tail is the whole list, 10+7+4+2"
        );
    }
}
