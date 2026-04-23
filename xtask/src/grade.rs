//! Crate grading against the A-grade standard.
//!
//! This module implements the seven-criterion grading system defined in docs/STANDARDS.md.

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
    /// (5-10 min per crate) which is too expensive for per-PR CI. Reported
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
    /// Full descriptive string for COMPLETION.md (e.g., "96.2% line coverage").
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
/// CortenForge grades against seven criteria, but `docs/STANDARDS.md`
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
/// - `xtask/`              → [`CrateProfile::Xtask`]
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
    } else if normalized == "xtask" || normalized.starts_with("xtask/") {
        CrateProfile::Xtask
    } else if normalized.starts_with("sim/L1/") {
        CrateProfile::BevyLayer1
    } else {
        CrateProfile::Layer0
    }
}

/// Run a single criterion with progress logging on stderr.
fn run_criterion<F>(index: usize, name: &str, quiet: bool, f: F) -> Result<CriterionResult>
where
    F: FnOnce() -> Result<CriterionResult>,
{
    if !quiet {
        eprintln!("  criterion {}/7: {} — running…", index, name);
    }
    let start = Instant::now();
    let result = f()?;
    if !quiet {
        eprintln!(
            "  criterion {}/7: {} — {} ({:.1}s)",
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
    // F.3: read the crate's Cargo.toml so classify_crate can honor the
    // [package.metadata.cortenforge] opt-in. Missing or unreadable falls
    // through to path-based classification — find_crate_path already
    // proved the crate exists in workspace metadata.
    let cargo_toml_text =
        std::fs::read_to_string(format!("{}/Cargo.toml", crate_path)).unwrap_or_default();
    let profile = classify_crate(&crate_path, &cargo_toml_text);

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
    };

    report
        .criteria
        .push(run_criterion(1, "Coverage", verbosity.quiet, || {
            grade_coverage(sh, crate_name, &crate_path, profile, verbosity)
        })?);
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
    report
        .criteria
        .push(run_criterion(6, "Bevy-free", verbosity.quiet, || {
            grade_bevy_free(sh, crate_name, profile)
        })?);
    report.criteria.push(CriterionResult {
        name: "7. API Design",
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

    let json = serde_json::json!({
        "crate_name": report.crate_name,
        "profile": report.profile.label(),
        "automated_grade": report.automated_grade.as_str(),
        "criteria": criteria,
    });

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
/// aggregated and reported in a compact summary. Exits non-zero if
/// any crate's automated grade is below A.
///
/// Intended as the CI entry point for single-source-of-truth grading:
/// CI runs `cargo xtask grade-all --skip-coverage --quiet`, checks the
/// exit code, and surfaces the failure summary on red.
pub fn run_all(verbosity: Verbosity) -> Result<()> {
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
    let mut crate_names: Vec<String> = packages
        .iter()
        .filter_map(|p| p["name"].as_str().map(String::from))
        .collect();
    crate_names.sort();

    if !verbosity.quiet {
        eprintln!();
        eprintln!(
            "  grade-all: evaluating {} workspace crates…",
            crate_names.len()
        );
        if verbosity.skip_coverage {
            eprintln!("  (coverage skipped via --skip-coverage)");
        }
        eprintln!();
    }

    let mut failures: Vec<(String, GradeReport)> = Vec::new();
    let mut passes = 0usize;

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
        let report = evaluate(&sh, crate_name, per_crate_verbosity)?;

        let passed = matches!(report.automated_grade, Grade::A | Grade::APlus);
        if passed {
            passes += 1;
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

    println!();
    if failures.is_empty() {
        println!(
            "{}",
            format!(
                "✓ grade-all: {}/{} workspace crates pass.",
                passes,
                crate_names.len()
            )
            .green()
            .bold()
        );
        println!();
        Ok(())
    } else {
        println!(
            "{}",
            format!(
                "✗ grade-all: {}/{} workspace crates fail xtask grade.",
                failures.len(),
                crate_names.len()
            )
            .red()
            .bold()
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
        println!();
        bail!("{} workspace crate(s) failed xtask grade", failures.len())
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
            c.name,
            truncate(&c.result, 16),
            grade_cell(&c.grade),
            c.threshold
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

/// Return `Some((result_label, detail))` if Coverage should skip for the
/// profile, or `None` if the crate must be measured.
///
/// Three profiles skip Coverage: Example and Xtask (no lib target per
/// STANDARDS.md §1) and IntegrationOnly (F.3 metadata opt-in for crates
/// exercised by integration tests). Layer0 and BevyLayer1 always run.
fn coverage_skip_reason(profile: CrateProfile) -> Option<(&'static str, String)> {
    match profile {
        CrateProfile::Example | CrateProfile::Xtask => Some((
            "(bin-only)",
            format!(
                "coverage skipped: {} crates have no lib target per STANDARDS.md §1",
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

/// Grade test coverage using cargo-llvm-cov (F2 decision: replaces tarpaulin).
///
/// Two-tier thresholds (F1 decision): >=90% = A+, >=75% = A.
/// Graceful degradation (ss1.4): if cargo-llvm-cov is not installed, reports Manual.
///
/// **Per-crate report filtering:** `cargo llvm-cov -p <crate>` scopes the
/// *tests* to the named package but includes all workspace-member source
/// files in the *report* (because workspace deps are compiled with
/// instrumentation). Without filtering, the denominator includes sim-core,
/// cf-geometry, mesh-*, etc. and structurally tanks any crate that depends
/// on large workspace siblings. We filter the JSON `data[0].files` array
/// to only sum lines from files whose path contains the crate's source
/// directory (e.g. `sim/L0/thermostat/`), giving the correct per-crate
/// coverage.
fn grade_coverage(
    sh: &Shell,
    crate_name: &str,
    crate_path: &str,
    profile: CrateProfile,
    verbosity: Verbosity,
) -> Result<CriterionResult> {
    // Per STANDARDS.md §1 "Exceptions: None for Layer 0 crates" — the
    // coverage gate is specifically scoped to Layer 0 library crates.
    // Example (bin-only) and Xtask (build tooling) crates have no lib
    // target, so `cargo llvm-cov --lib` errors out with "no library
    // targets found". F.3 IntegrationOnly crates (per Cargo.toml
    // metadata opt-in) are exercised by integration tests and have no
    // testable lib surface. Treat all three as N/A rather than a false F.
    if let Some((result, detail)) = coverage_skip_reason(profile) {
        return Ok(CriterionResult {
            name: "1. Coverage",
            result: result.to_string(),
            grade: Grade::NotApplicable,
            threshold: "≥75%/≥90% A+",
            measured_detail: detail,
        });
    }

    // `--skip-coverage` opt-out: CI runs want the other criteria without
    // paying the ~5-10 min per-crate llvm-cov release build. Dedicated
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

    // Check if cargo-llvm-cov is installed
    let version_check = cmd!(sh, "cargo llvm-cov --version")
        .ignore_status()
        .ignore_stderr()
        .read()
        .unwrap_or_default();

    if !version_check.contains("cargo-llvm-cov") {
        return Ok(CriterionResult {
            name: "1. Coverage",
            result: "(llvm-cov n/a)".to_string(),
            grade: Grade::Manual,
            threshold: "≥75%/≥90% A+",
            measured_detail: "(llvm-cov n/a)".to_string(),
        });
    }

    // Two-pass coverage strategy:
    //
    // Pass 1: Run coverage on unit tests only (fast — ~1 sec after compile).
    //   Integration tests (tests/*.rs) run millions of physics steps;
    //   llvm-cov instrumentation adds ~10× overhead even in release.
    //   Unit tests in src/ exercise the same source code paths.
    //
    // Pass 2: Run ALL tests (unit + integration) WITHOUT instrumentation.
    //   Verifies correctness without paying the coverage overhead.
    //   Failures here still block the grade.
    //
    // --release avoids debug-mode runtime explosion (100×+ slower).
    // stderr flows to terminal so the user sees compile/test progress.

    // Pass 1: coverage from unit tests only (--lib). Integration tests
    // run millions of physics steps; llvm-cov instrumentation adds ~10×
    // overhead making them impractical for coverage measurement.
    if !verbosity.quiet {
        eprintln!(
            "    pass 1/2: cargo llvm-cov --lib --release (~5-10 min for instrumented test runs)"
        );
    }
    let pass_start = Instant::now();
    let heartbeat = if verbosity.verbose {
        Some(Heartbeat::start(30))
    } else {
        None
    };
    let output = cmd!(sh, "cargo llvm-cov --json --release -p {crate_name} --lib")
        .ignore_status()
        .read()
        .unwrap_or_default();
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
    if !heavy_passed {
        eprintln!("    ⚠ Tests failed — see output above");
    }

    // Parse JSON and filter to files belonging to the target crate.
    // The llvm-cov report includes all instrumented workspace members;
    // we sum only files whose path contains the crate's directory
    // (e.g. "sim/L0/thermostat/") to get the correct per-crate number.
    let coverage = (|| -> Option<f64> {
        let json: serde_json::Value = serde_json::from_str(&output).ok()?;
        let files = json["data"][0]["files"].as_array()?;

        let mut covered: u64 = 0;
        let mut total: u64 = 0;
        for file in files {
            let filename = file["filename"].as_str().unwrap_or("");
            if !filename.contains(crate_path) {
                continue;
            }
            let lines = &file["summary"]["lines"];
            covered += lines["covered"].as_u64().unwrap_or(0);
            total += lines["count"].as_u64().unwrap_or(0);
        }

        if total == 0 {
            return None;
        }
        Some(100.0 * covered as f64 / total as f64)
    })();

    let Some(coverage) = coverage else {
        return Ok(CriterionResult {
            name: "1. Coverage",
            result: "(parse error)".to_string(),
            grade: Grade::F,
            threshold: "≥75%/≥90% A+",
            measured_detail: "(failed to parse llvm-cov JSON)".to_string(),
        });
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

    let detail = if !heavy_passed {
        format!("{:.1}% line coverage (heavy tests FAILED)", coverage)
    } else {
        format!("{:.1}% line coverage", coverage)
    };

    Ok(CriterionResult {
        name: "1. Coverage",
        result: format!("{:.1}%", coverage),
        grade,
        threshold: "≥75%/≥90% A+",
        measured_detail: detail,
    })
}

/// Grade documentation by checking for warnings.
///
/// Captures stderr + exit code (B2 fix: `cargo doc` writes diagnostics
/// to stderr, not stdout — the old gate read stdout and always saw 0).
fn grade_documentation(sh: &Shell, crate_name: &str, crate_path: &str) -> Result<CriterionResult> {
    let output = cmd!(sh, "cargo doc --no-deps -p {crate_name}")
        .env("RUSTDOCFLAGS", "-D warnings")
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
fn grade_clippy(
    sh: &Shell,
    crate_name: &str,
    crate_path: &str,
    profile: CrateProfile,
) -> Result<CriterionResult> {
    let output = cmd!(
        sh,
        "cargo clippy -p {crate_name} --all-targets --all-features --message-format=json"
    )
    .ignore_status()
    .read()
    .unwrap_or_default();

    // Parse JSON lines: filter compiler-message with warning/error level and non-empty spans
    let mut clippy_count = 0;
    for line in output.lines() {
        let Ok(json) = serde_json::from_str::<serde_json::Value>(line) else {
            continue;
        };
        if json["reason"].as_str() != Some("compiler-message") {
            continue;
        }
        let level = json["message"]["level"].as_str().unwrap_or("");
        if level != "warning" && level != "error" {
            continue;
        }
        // Exclude summary lines (empty spans = "N warnings emitted").
        // F.1: also filter out transitive-dep diagnostics whose spans all
        // point outside the target crate. Disjunctive — a diagnostic with
        // even one span inside the crate is counted (include-not-exclude).
        let Some(spans) = json["message"]["spans"].as_array() else {
            continue;
        };
        if spans.is_empty() || !any_span_in_crate(spans, crate_path) {
            continue;
        }
        clippy_count += 1;
    }

    // Unjustified #[allow(clippy:: check (F-ext-3).
    //
    // Relaxed for Example/Xtask profiles by the same STANDARDS.md §4
    // "Allowed" theme that relaxes unwrap/expect counting for them:
    // demo and tooling code is not prod-surface and shouldn't be held
    // to the same justification-comment rubric as library code.
    let relax_unjustified_allows = matches!(profile, CrateProfile::Example | CrateProfile::Xtask);
    let mut allow_count = 0;
    if !relax_unjustified_allows {
        let src_path = format!("{}/src", crate_path);
        if let Ok(entries) = glob_rs_files(&src_path) {
            for file_path in entries {
                if let Ok(content) = std::fs::read_to_string(&file_path) {
                    let lines: Vec<&str> = content.lines().collect();
                    allow_count += count_unjustified_clippy_allows(&lines);
                }
            }
        }
    }

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

/// Collect all `.rs` files under a directory.
fn glob_rs_files(dir: &str) -> Result<Vec<String>> {
    let mut files = Vec::new();
    if !Path::new(dir).exists() {
        return Ok(files);
    }
    for entry in walkdir::WalkDir::new(dir)
        .into_iter()
        .filter_map(|e| e.ok())
    {
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

    if !Path::new(&src_path).exists() {
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
        let content = match std::fs::read_to_string(file_path) {
            Ok(c) => c,
            Err(_) => continue,
        };
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

    // Additive scan for file-level INNER attributes: if `i` is deep enough
    // that the back-window doesn't reach the first 50 lines of the file,
    // explicitly check those top lines for `#![allow(...)]` only. These
    // are module-level attributes that cover the whole file regardless
    // of distance. Outer `#[allow(...)]` at file-top is excluded here —
    // it binds to the next item, not the whole file, so using it to
    // suppress a violation far below would be a false positive.
    if back_start > 50 {
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

/// Check that a crate has no Bevy dependencies.
///
/// Per STANDARDS.md §6 "Criterion 6: Bevy-free (Layer 0)" — this
/// criterion is explicitly and exclusively scoped to Layer 0 library
/// crates. STANDARDS.md §6 names the Layer 0 prefixes (`mesh-*`,
/// `cf-spatial`, `route-*`, `sensor-*`, `ml-*`, `sim-*` Layer 0) and
/// explicitly allows Bevy in Layer 1 (`cortenforge`, `sim-bevy`).
/// Example and Xtask crates are not part of the Layer 0 / Layer 1
/// dichotomy at all — they're tooling and demos, and applying the
/// Bevy-free rubric to them is a category error.
///
/// `wgpu` is allowed in Layer 0 — it's a standalone WebGPU
/// implementation used independently for compute shaders (F6 decision).
fn grade_bevy_free(sh: &Shell, crate_name: &str, profile: CrateProfile) -> Result<CriterionResult> {
    // Scope: Layer 0 only, per STANDARDS.md §6. Every other profile
    // gets N/A with a pointer to the standard.
    if !matches!(profile, CrateProfile::Layer0) {
        return Ok(CriterionResult {
            name: "6. Bevy-free",
            result: "(not layer 0)".to_string(),
            grade: Grade::NotApplicable,
            threshold: "no bevy/winit",
            measured_detail: format!(
                "Bevy-free criterion is Layer 0 only per STANDARDS.md §6; \
                 {} is exempt",
                profile.label()
            ),
        });
    }

    // --prefix none --format "{p}" outputs one "pkg_name vX.Y.Z" per line
    let tree_format = "{p}";
    let output = cmd!(
        sh,
        "cargo tree -p {crate_name} --prefix none --format {tree_format}"
    )
    .ignore_status()
    .ignore_stderr()
    .read()
    .unwrap_or_default();

    // Exact package-name prefix matching (not substring contains)
    let mut violations: Vec<String> = Vec::new();
    for line in output.lines() {
        let pkg_name = line.split_whitespace().next().unwrap_or("");
        if (pkg_name.starts_with("bevy") || pkg_name == "winit")
            && !violations.contains(&pkg_name.to_string())
        {
            violations.push(pkg_name.to_string());
        }
    }

    let grade = if violations.is_empty() {
        Grade::A
    } else {
        Grade::F
    };

    let result = if violations.is_empty() {
        "✓ confirmed".to_string()
    } else {
        format!("has: {}", violations.join(", "))
    };

    let measured_detail = result.clone();

    Ok(CriterionResult {
        name: "6. Bevy-free",
        result,
        grade,
        threshold: "no bevy/winit",
        measured_detail,
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
            coverage_skip_reason(CrateProfile::IntegrationOnly).expect("must skip");
        assert_eq!(result, "(integration-only)");
        assert!(detail.contains("[package.metadata.cortenforge]"));
        // Sanity: Layer0 must NOT skip.
        assert!(coverage_skip_reason(CrateProfile::Layer0).is_none());
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
}
