//! Crate grading against the A-grade standard.
//!
//! This module implements the seven-criterion grading system defined in docs/STANDARDS.md.

use anyhow::{bail, Context, Result};
use owo_colors::OwoColorize;
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

/// Grade test coverage using cargo-llvm-cov (F2 decision: replaces tarpaulin).
///
/// Two-tier thresholds (F1 decision): >=90% = A+, >=75% = A.
/// Graceful degradation (ss1.4): if cargo-llvm-cov is not installed, reports Manual.
fn grade_coverage(sh: &Shell, crate_name: &str) -> Result<CriterionResult> {
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

    // Run cargo llvm-cov --json -p <crate>
    let output = cmd!(sh, "cargo llvm-cov --json -p {crate_name}")
        .ignore_status()
        .ignore_stderr()
        .read()
        .unwrap_or_default();

    // Parse JSON: data[0].totals.lines.percent
    let coverage = (|| -> Option<f64> {
        let json: serde_json::Value = serde_json::from_str(&output).ok()?;
        json["data"][0]["totals"]["lines"]["percent"].as_f64()
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
    let grade = if coverage >= 90.0 {
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

    Ok(CriterionResult {
        name: "1. Coverage",
        result: format!("{:.1}%", coverage),
        grade,
        threshold: "≥75%/≥90% A+",
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

/// Grade safety: check for panic-capable patterns in library code.
///
/// Full rewrite per chassis ss2.4. Fixes B3 bugs 1-5:
/// - Brace-depth tracked test exclusion (not first-#[cfg(test)]-to-EOF)
/// - Block comment handling
/// - All 6 patterns (todo!, unimplemented!, unwrap, expect, panic!, unreachable!)
/// - Unsafe-without-SAFETY check
/// - Blanket assert exclusion removed
/// - Direct file reading (no grep shell-outs)
fn grade_safety(_sh: &Shell, crate_path: &str) -> Result<CriterionResult> {
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

    let mut has_todo_or_unimplemented = false;
    let mut counted_violations = 0usize;
    let mut unsafe_violations = 0usize;

    for file_path in &files {
        let is_build_rs = file_path.ends_with("build.rs");
        let content = match std::fs::read_to_string(file_path) {
            Ok(c) => c,
            Err(_) => continue,
        };
        let lines: Vec<&str> = content.lines().collect();

        // Brace-depth tracked test exclusion state machine (ss2.4)
        let mut in_test = false;
        let mut test_brace_depth: usize = 0;
        let mut pending_test_attr = false;
        let mut in_block_comment = false;

        for (i, line) in lines.iter().enumerate() {
            let trimmed = line.trim();

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

            // Test exclusion: #[cfg(test)] attribute detection
            if !trimmed.starts_with("//") && trimmed.contains("#[cfg(test)]") {
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
            if trimmed.contains("todo!") || trimmed.contains("unimplemented!") {
                has_todo_or_unimplemented = true;
            }

            // Counted: .unwrap()
            if trimmed.contains(".unwrap()") {
                counted_violations += 1;
            }

            // Counted: .expect( — skip in build.rs
            if !is_build_rs && trimmed.contains(".expect(") {
                counted_violations += 1;
            }

            // Counted with justification: panic!()
            if trimmed.contains("panic!") {
                let has_justification =
                    has_preceding_comment(&lines, i) || has_same_line_comment(trimmed);
                if !has_justification {
                    counted_violations += 1;
                }
            }

            // Counted with justification: unreachable!()
            if trimmed.contains("unreachable!") {
                let has_justification =
                    has_preceding_comment(&lines, i) || has_same_line_comment(trimmed);
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

/// Check if any of the preceding 1-3 lines is a `//` comment (not `///` or `//!`).
fn has_preceding_comment(lines: &[&str], i: usize) -> bool {
    (1..=3).any(|offset| {
        i.checked_sub(offset).is_some_and(|j| {
            let prev = lines[j].trim();
            prev.starts_with("//") && !prev.starts_with("///") && !prev.starts_with("//!")
        })
    })
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

/// Grade dependencies: justification check (hard gate) + dep count (informational).
///
/// Per chassis ss2.5: every dependency in Cargo.toml must have a `#` comment
/// in the preceding 1-3 lines or inline. Dep count > 10 is flagged as "(heavy)"
/// but does not affect grade.
fn grade_dependencies(sh: &Shell, crate_name: &str) -> Result<CriterionResult> {
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

    let mut unjustified = 0;
    let mut in_dep_section = false;
    let lines: Vec<&str> = cargo_content.lines().collect();

    for (i, line) in lines.iter().enumerate() {
        let trimmed = line.trim();

        // Track dependency sections
        if trimmed.starts_with('[') {
            in_dep_section = trimmed == "[dependencies]"
                || trimmed == "[dev-dependencies]"
                || trimmed == "[build-dependencies]";
            continue;
        }

        if !in_dep_section {
            continue;
        }

        // Skip blank lines and comments
        if trimmed.is_empty() || trimmed.starts_with('#') {
            continue;
        }

        // A dependency entry is a line with `name = ...` in a dep section
        if !trimmed.contains('=') {
            continue;
        }

        // Check for inline comment
        if trimmed.contains('#') {
            continue; // has inline justification
        }

        // Check preceding 1-3 lines for a # comment.
        // Stop at blank lines or section headers (they break the chain).
        let has_justification = {
            let mut found = false;
            for offset in 1..=3 {
                if let Some(j) = i.checked_sub(offset) {
                    let prev = lines[j].trim();
                    if prev.is_empty() || prev.starts_with('[') {
                        break;
                    }
                    if prev.starts_with('#') {
                        found = true;
                        break;
                    }
                }
            }
            found
        };

        if !has_justification {
            unjustified += 1;
        }
    }

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
