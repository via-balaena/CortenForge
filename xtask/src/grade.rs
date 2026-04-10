//! Crate grading against the A-grade standard.
//!
//! This module implements the seven-criterion grading system defined in docs/STANDARDS.md.

use anyhow::{bail, Context, Result};
use owo_colors::OwoColorize;
use regex::Regex;
use std::path::Path;
use xshell::{cmd, Shell};

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
    pub criteria: Vec<CriterionResult>,
    pub automated_grade: Grade,
    #[allow(dead_code)]
    pub needs_review: bool,
}

impl GradeReport {
    fn overall_automated(&self) -> Grade {
        let mut worst = Grade::APlus;
        for c in &self.criteria {
            if c.grade == Grade::Manual {
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

/// Run all automated criteria and return structured results.
pub fn evaluate(sh: &Shell, crate_name: &str) -> Result<GradeReport> {
    let workspace_root = find_workspace_root(sh)?;
    sh.change_dir(&workspace_root);

    let crate_path = find_crate_path(sh, crate_name)?;

    let mut report = GradeReport {
        crate_name: crate_name.to_string(),
        criteria: Vec::new(),
        automated_grade: Grade::A,
        needs_review: true,
    };

    // 1. Test Coverage
    report.criteria.push(grade_coverage(sh, crate_name)?);
    // 2. Documentation
    report
        .criteria
        .push(grade_documentation(sh, crate_name, &crate_path)?);
    // 3. Clippy
    report
        .criteria
        .push(grade_clippy(sh, crate_name, &crate_path)?);
    // 4. Safety
    report.criteria.push(grade_safety(sh, &crate_path)?);
    // 5. Dependencies
    report.criteria.push(grade_dependencies(sh, crate_name)?);
    // 6. Bevy-free
    report.criteria.push(grade_bevy_free(sh, crate_name)?);
    // 7. API Design (manual)
    report.criteria.push(CriterionResult {
        name: "7. API Design",
        result: "(manual review)".to_string(),
        grade: Grade::Manual,
        threshold: "checklist",
        measured_detail: "(manual review)".to_string(),
    });

    report.automated_grade = report.overall_automated();

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

/// Run grading for a specific crate
pub fn run(crate_name: &str, _format: &str) -> Result<()> {
    let sh = Shell::new()?;
    let report = evaluate(&sh, crate_name)?;
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

    Ok(())
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

/// Grade test coverage using cargo tarpaulin
fn grade_coverage(sh: &Shell, crate_name: &str) -> Result<CriterionResult> {
    // Check if tarpaulin is installed
    let tarpaulin_check = cmd!(sh, "cargo tarpaulin --version")
        .ignore_status()
        .ignore_stderr()
        .read()
        .unwrap_or_default();

    // If the output doesn't contain version info, tarpaulin isn't installed
    if !tarpaulin_check.contains("cargo-tarpaulin") {
        return Ok(CriterionResult {
            name: "1. Coverage",
            result: "(tarpaulin n/a)".to_string(),
            grade: Grade::Manual,
            threshold: "≥90%",
            measured_detail: "(tarpaulin n/a)".to_string(),
        });
    }

    // Run tarpaulin and parse output (stderr is captured separately)
    let output = cmd!(
        sh,
        "cargo tarpaulin -p {crate_name} --out Json --output-dir target/tarpaulin"
    )
    .ignore_status()
    .ignore_stderr()
    .read()
    .unwrap_or_default();

    // Parse coverage percentage from output
    // Look for pattern like "XX.XX% coverage"
    let coverage_regex = Regex::new(r"(\d+\.?\d*)% coverage")?;
    let coverage = coverage_regex
        .captures(&output)
        .and_then(|c| c.get(1))
        .and_then(|m| m.as_str().parse::<f64>().ok())
        .unwrap_or(0.0);

    let grade = if coverage >= 90.0 {
        Grade::A
    } else if coverage >= 80.0 {
        Grade::B
    } else if coverage >= 60.0 {
        Grade::C
    } else {
        Grade::F
    };

    Ok(CriterionResult {
        name: "1. Coverage",
        result: format!("{:.1}%", coverage),
        grade,
        threshold: "≥90%",
        measured_detail: format!("{:.1}% line coverage", coverage),
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
fn grade_clippy(sh: &Shell, crate_name: &str, crate_path: &str) -> Result<CriterionResult> {
    let output = cmd!(
        sh,
        "cargo clippy -p {crate_name} --all-targets --all-features --message-format=json"
    )
    .ignore_status()
    .ignore_stderr()
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
        // Exclude summary lines (empty spans = "N warnings emitted")
        let spans = &json["message"]["spans"];
        if !spans.is_array() || spans.as_array().is_none_or(|a| a.is_empty()) {
            continue;
        }
        clippy_count += 1;
    }

    // Unjustified #[allow(clippy:: check (F-ext-3)
    // Simple heuristic: skip lines after #[cfg(test)] to EOF (brace-depth tracker in Step 5)
    let mut allow_count = 0;
    let src_path = format!("{}/src", crate_path);
    if let Ok(entries) = glob_rs_files(&src_path) {
        for file_path in entries {
            if let Ok(content) = std::fs::read_to_string(&file_path) {
                let lines: Vec<&str> = content.lines().collect();
                let mut in_test = false;
                for (i, line) in lines.iter().enumerate() {
                    let trimmed_line = line.trim();
                    if !trimmed_line.starts_with("//") && trimmed_line.contains("#[cfg(test)]") {
                        in_test = true;
                    }
                    if in_test {
                        continue;
                    }
                    let trimmed = line.trim();
                    if !trimmed.contains("#[allow(clippy::") {
                        continue;
                    }
                    // Check preceding 1-3 lines for a justification comment
                    let has_justification = (1..=3).any(|offset| {
                        i.checked_sub(offset).is_some_and(|j| {
                            let prev = lines[j].trim();
                            prev.starts_with("//")
                                && !prev.starts_with("///")
                                && !prev.starts_with("//!")
                        })
                    });
                    if !has_justification {
                        // Also check same-line trailing comment
                        let has_inline = trimmed.contains("//") && {
                            let comment_pos = trimmed.rfind("//").unwrap_or(0);
                            comment_pos > trimmed.find("#[allow(clippy::").unwrap_or(0)
                        };
                        if !has_inline {
                            allow_count += 1;
                        }
                    }
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

/// Grade safety by checking for unwrap/expect in library code
fn grade_safety(sh: &Shell, crate_path: &str) -> Result<CriterionResult> {
    let src_path = format!("{}/src", crate_path);

    if !Path::new(&src_path).exists() {
        return Ok(CriterionResult {
            name: "4. Safety",
            result: "(no src/)".to_string(),
            grade: Grade::Manual,
            threshold: "0",
            measured_detail: "(no src/)".to_string(),
        });
    }

    // First, find where test sections start in each file
    let test_starts = cmd!(
        sh,
        "grep -rn '#\\[cfg(test)\\]' {src_path} --include='*.rs'"
    )
    .ignore_status()
    .ignore_stderr()
    .read()
    .unwrap_or_default();

    // Build a map of file -> test start line
    let mut test_start_lines: std::collections::HashMap<String, usize> =
        std::collections::HashMap::new();
    for line in test_starts.lines() {
        // Format: path/file.rs:123:#[cfg(test)]
        let parts: Vec<&str> = line.splitn(3, ':').collect();
        if parts.len() >= 2 {
            if let Ok(line_num) = parts[1].parse::<usize>() {
                let file = parts[0].to_string();
                test_start_lines.entry(file).or_insert(line_num);
            }
        }
    }

    // Count unwrap/expect occurrences (excluding tests)
    let patterns = [".unwrap()", ".expect("];
    let mut total_count = 0;

    for pattern in &patterns {
        let output = cmd!(sh, "grep -rn {pattern} {src_path} --include='*.rs'")
            .ignore_status()
            .ignore_stderr()
            .read()
            .unwrap_or_default();

        for line in output.lines() {
            if line.trim().is_empty() {
                continue;
            }

            // Parse file:line:content format
            let parts: Vec<&str> = line.splitn(3, ':').collect();
            if parts.len() < 3 {
                continue;
            }
            let file = parts[0];
            let line_num: usize = parts[1].parse().unwrap_or(0);
            let content = parts[2];

            // Skip if this line is in or after a #[cfg(test)] section
            if let Some(&test_start) = test_start_lines.get(file) {
                if line_num >= test_start {
                    continue;
                }
            }

            // Skip comments and doc comments
            let trimmed = content.trim();
            if trimmed.starts_with("//") || trimmed.starts_with("///") || trimmed.starts_with("//!")
            {
                continue;
            }

            // Skip assert macros (test-like code)
            if content.contains("assert") {
                continue;
            }

            total_count += 1;
        }
    }

    // B grade is acceptable for most crates (some legitimate uses in error paths)
    let grade = if total_count == 0 {
        Grade::A
    } else if total_count <= 5 {
        Grade::B
    } else if total_count <= 15 {
        Grade::C
    } else {
        Grade::F
    };

    Ok(CriterionResult {
        name: "4. Safety",
        result: format!("{} violations", total_count),
        grade,
        threshold: "≤5",
        measured_detail: format!("{} violations", total_count),
    })
}

/// Grade dependencies by counting and checking for heavy deps
fn grade_dependencies(sh: &Shell, crate_name: &str) -> Result<CriterionResult> {
    let output = cmd!(sh, "cargo tree -p {crate_name} --edges normal --depth 1")
        .ignore_status()
        .ignore_stderr()
        .read()
        .unwrap_or_default();

    let dep_count = output.lines().count().saturating_sub(1); // Subtract the crate itself

    // Thresholds: A ≤7 allows reasonable crates (mesh-types + error + logging + domain-specific)
    // GPU crates legitimately need wgpu + bytemuck + pollster + thiserror + tracing
    let grade = if dep_count <= 7 {
        Grade::A
    } else if dep_count <= 12 {
        Grade::B
    } else if dep_count <= 20 {
        Grade::C
    } else {
        Grade::F
    };

    Ok(CriterionResult {
        name: "5. Dependencies",
        result: format!("{} deps", dep_count),
        grade,
        threshold: "≤7",
        measured_detail: format!("{} deps", dep_count),
    })
}

/// Check that a crate has no Bevy dependencies.
///
/// Layer 0 crates should not depend on Bevy or windowing libraries.
/// `wgpu` is allowed — it's a standalone WebGPU implementation used
/// independently for compute shaders (F6 decision).
fn grade_bevy_free(sh: &Shell, crate_name: &str) -> Result<CriterionResult> {
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
