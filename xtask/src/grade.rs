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

/// One of the four architectural tiers from `L0_architectural_plan.md` §2.1.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Tier {
    L0,
    L0Io,
    L0Integration,
    L1,
}

impl Tier {
    fn parse(s: &str) -> Option<Self> {
        match s {
            "L0" => Some(Tier::L0),
            "L0-io" => Some(Tier::L0Io),
            "L0-integration" => Some(Tier::L0Integration),
            "L1" => Some(Tier::L1),
            _ => None,
        }
    }

    fn label(&self) -> &'static str {
        match self {
            Tier::L0 => "L0",
            Tier::L0Io => "L0-io",
            Tier::L0Integration => "L0-integration",
            Tier::L1 => "L1",
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

/// Look up the static `TierConfig` for a tier. Numbers track plan §5.2
/// (release 80/200/200, test 100/220/220). Plan §2.1 proposes tighter
/// numbers (60/180/180) post-Appendix-A; we'll re-tune at hard-gate flip
/// (plan §8 step 12) once surgeries land and the actual headroom is known.
fn tier_config(tier: Tier) -> TierConfig {
    match tier {
        Tier::L0 => TierConfig {
            release_max: 80,
            test_max: 100,
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
}

/// True if `crate_name` must declare tier metadata. Plan §5.1 scope:
/// `sim-*`, `mesh-*`, `cf-*`, `cortenforge*`. The two no-hyphen umbrellas
/// (`mesh`, `cortenforge`) are explicitly accepted; `examples/*` and
/// `xtask` are out of scope.
fn applies_to_crate(crate_name: &str) -> bool {
    if matches!(crate_name, "mesh" | "cortenforge") {
        return true;
    }
    // Workspace tools that happen to match a library-namespace prefix
    // need an explicit exemption. xtask is excluded by virtue of having
    // no matching prefix at all; cf-viewer's `cf-` prefix would
    // otherwise pull it into the design-library scope (cf-spatial /
    // cf-design / cf-geometry). Per docs/VIEWER_DESIGN.md Q1 + Q8
    // locks: cf-viewer is a workspace tool, carries no tier metadata,
    // and Q8 explicitly directs path-based filtering as the gating
    // mechanism rather than retrofitting metadata.
    if matches!(crate_name, "cf-viewer") {
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
            tier_up_features.push((feat.clone(), target_tier));
        }
    }

    Ok(Some(TierMetadata {
        tier,
        tier_up_features,
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

    let mut deps = Vec::new();
    let mut seen = std::collections::HashSet::new();
    for line in output.lines() {
        if let Some(name) = line.split_whitespace().next() {
            if !name.is_empty() && seen.insert(name.to_string()) {
                deps.push(name.to_string());
            }
        }
    }
    Ok(deps)
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

    // L1 is unbounded by definition — skip the cargo-tree work entirely.
    if metadata.tier == Tier::L1 {
        return Ok(CriterionResult {
            name: "6. Layer Integrity",
            result: "(L1 tier)".to_string(),
            grade: Grade::NotApplicable,
            threshold: "tier rules",
            measured_detail: "Layer Integrity criterion is N/A for L1 tier".to_string(),
        });
    }

    // Run the 6 (config × graph) cargo-tree invocations and aggregate.
    let configs = [
        FeatureConfig::NoDefault,
        FeatureConfig::Default,
        FeatureConfig::AllFeatures,
    ];
    let graphs = [GraphKind::Release, GraphKind::WithDev];

    let mut all_findings: Vec<Finding> = Vec::new();
    for fc in configs {
        for gk in graphs {
            let deps = read_tree_deps(sh, crate_name, fc, gk)?;
            let effective_tier = effective_tier_for(&metadata, fc);
            let config = tier_config(effective_tier);
            let findings = evaluate_dep_set(&deps, config, fc, gk, effective_tier);
            all_findings.extend(findings);
        }
    }

    // Hard-gated per plan §8 step 12 (was Grade::A unconditionally in
    // step 3 warning-mode commit `1fb88e2f`).
    let warning_grade = if all_findings.is_empty() {
        Grade::A
    } else {
        Grade::F
    };

    if all_findings.is_empty() {
        return Ok(CriterionResult {
            name: "6. Layer Integrity",
            result: "✓ confirmed".to_string(),
            grade: warning_grade,
            threshold: "tier rules",
            measured_detail: format!("tier {} — no findings", metadata.tier.label()),
        });
    }

    // Render findings to stderr and to measured_detail. Stderr emission
    // is unconditional — failures are PR-blocking under hard gate.
    // Quiet (set by `grade-all`) only suppresses the per-finding lines;
    // the summary line remains so `grade-all` output stays scannable.
    let n = all_findings.len();
    eprintln!(
        "    layer integrity: FAIL — {} finding(s) for `{}` (tier {})",
        n,
        crate_name,
        metadata.tier.label(),
    );
    let mut detail = format!("tier {} — {} finding(s):\n", metadata.tier.label(), n);
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
        grade: warning_grade,
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

    // === Layer Integrity (criterion 6) ===

    #[test]
    fn tier_parse_known_values() {
        assert_eq!(Tier::parse("L0"), Some(Tier::L0));
        assert_eq!(Tier::parse("L0-io"), Some(Tier::L0Io));
        assert_eq!(Tier::parse("L0-integration"), Some(Tier::L0Integration));
        assert_eq!(Tier::parse("L1"), Some(Tier::L1));
    }

    #[test]
    fn tier_parse_rejects_unknown() {
        assert_eq!(Tier::parse("L2"), None);
        assert_eq!(Tier::parse("l0"), None);
        assert_eq!(Tier::parse(""), None);
    }

    #[test]
    fn tier_permissiveness_ordering() {
        // L0 strictest → L1 most permissive. effective_tier_for relies
        // on this monotonic ordering when picking the loosest tier among
        // multiple tier_up_features.
        assert!(Tier::L0.permissiveness() < Tier::L0Io.permissiveness());
        assert!(Tier::L0Io.permissiveness() < Tier::L0Integration.permissiveness());
        assert!(Tier::L0Integration.permissiveness() < Tier::L1.permissiveness());
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
        // exemption per docs/VIEWER_DESIGN.md Q1 + Q8 locks.
        assert!(!applies_to_crate("cf-viewer"));
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
    fn effective_tier_under_default_is_declared() {
        let m = TierMetadata {
            tier: Tier::L0,
            tier_up_features: vec![("gpu-probe".to_string(), Tier::L0Io)],
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
        // 81 release deps → exceeds L0's 80-release max → one
        // CountExceeded finding.
        let deps: Vec<String> = (0..81).map(|i| format!("dep-{}", i)).collect();
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
                actual: 81,
                max: 80
            }
        ));
    }

    #[test]
    fn evaluate_dep_set_l0_test_max_is_higher_than_release() {
        // 81 deps in the dev graph → under the 100 test-max, no count
        // finding. Same input over release max — the (release, test)
        // distinction must thread through evaluate_dep_set.
        let deps: Vec<String> = (0..81).map(|i| format!("dep-{}", i)).collect();
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
                actual: 95,
                max: 80,
            },
        };
        assert_eq!(
            format_finding(&f),
            "[release all-features, tier L0] dep count 95 exceeds max 80"
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
}
