//! Crate grading against the A-grade standard.
//!
//! This module implements the seven-criterion grading system defined in STANDARDS.md.

use anyhow::{bail, Context, Result};
use owo_colors::OwoColorize;
use regex::Regex;
use std::path::Path;
use xshell::{cmd, Shell};

/// Grade for a single criterion
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Grade {
    A,
    B,
    C,
    F,
    /// Requires manual review
    Manual,
}

impl Grade {
    fn as_str(&self) -> &'static str {
        match self {
            Grade::A => "A",
            Grade::B => "B",
            Grade::C => "C",
            Grade::F => "F",
            Grade::Manual => "?",
        }
    }

    fn colored(&self) -> String {
        match self {
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
}

/// Full grade report for a crate
#[derive(Debug)]
pub struct GradeReport {
    pub crate_name: String,
    pub criteria: Vec<CriterionResult>,
    pub automated_grade: Grade,
    pub needs_review: bool,
}

impl GradeReport {
    fn overall_automated(&self) -> Grade {
        let mut worst = Grade::A;
        for c in &self.criteria {
            if c.grade == Grade::Manual {
                continue;
            }
            // ALL criteria must be A for A-grade
            worst = match (&worst, &c.grade) {
                (Grade::F, _) | (_, Grade::F) => Grade::F,
                (Grade::C, _) | (_, Grade::C) => Grade::C,
                (Grade::B, _) | (_, Grade::B) => Grade::B,
                _ => Grade::A,
            };
        }
        worst
    }
}

/// Run grading for a specific crate
pub fn run(crate_name: &str, _format: &str) -> Result<()> {
    let sh = Shell::new()?;

    // Find workspace root
    let workspace_root = find_workspace_root(&sh)?;
    sh.change_dir(&workspace_root);

    // Verify crate exists
    let crate_path = find_crate_path(&sh, crate_name)?;

    println!();
    println!(
        "{}",
        format!("╔══════════════════════════════════════════════════════════════╗")
            .bright_white()
            .bold()
    );
    println!(
        "{}",
        format!("║{:^62}║", format!("GRADING: {}", crate_name))
            .bright_white()
            .bold()
    );
    println!(
        "{}",
        format!("╠══════════════════════════════════════════════════════════════╣")
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
        format!("╠══════════════════════════════════════════════════════════════╣").bright_white()
    );

    let mut report = GradeReport {
        crate_name: crate_name.to_string(),
        criteria: Vec::new(),
        automated_grade: Grade::A,
        needs_review: true,
    };

    // 1. Test Coverage
    let coverage = grade_coverage(&sh, crate_name)?;
    print_criterion(&coverage);
    report.criteria.push(coverage);

    // 2. Documentation
    let docs = grade_documentation(&sh, crate_name)?;
    print_criterion(&docs);
    report.criteria.push(docs);

    // 3. Clippy
    let clippy = grade_clippy(&sh, crate_name)?;
    print_criterion(&clippy);
    report.criteria.push(clippy);

    // 4. Safety
    let safety = grade_safety(&sh, &crate_path)?;
    print_criterion(&safety);
    report.criteria.push(safety);

    // 5. Dependencies
    let deps = grade_dependencies(&sh, crate_name)?;
    print_criterion(&deps);
    report.criteria.push(deps);

    // 6. Bevy-free
    let bevy = grade_bevy_free(&sh, crate_name)?;
    print_criterion(&bevy);
    report.criteria.push(bevy);

    // 7. API Design (manual)
    let api = CriterionResult {
        name: "7. API Design",
        result: "(manual review)".to_string(),
        grade: Grade::Manual,
        threshold: "checklist",
    };
    print_criterion(&api);
    report.criteria.push(api);

    report.automated_grade = report.overall_automated();

    println!(
        "{}",
        format!("╠══════════════════════════════════════════════════════════════╣").bright_white()
    );
    println!(
        "{}",
        format!(
            "║ {:16} │ {:16} │   {}   │ {:14} ║",
            "AUTOMATED",
            "",
            report.automated_grade.colored(),
            ""
        )
        .bright_white()
    );
    println!(
        "{}",
        format!(
            "║ {:16} │ {:16} │   {}   │ {:14} ║",
            "OVERALL",
            "",
            Grade::Manual.colored(),
            "needs review"
        )
        .bright_white()
    );
    println!(
        "{}",
        format!("╚══════════════════════════════════════════════════════════════╝").bright_white()
    );
    println!();

    match report.automated_grade {
        Grade::A => {
            println!(
                "{}",
                "✓ All automated criteria pass. Ready for API review.".green()
            );
            println!();
            println!("Next step: Review against API checklist in STANDARDS.md");
            println!("Then run: cargo xtask complete {}", crate_name);
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
            println!("Fix failing criteria and run: cargo xtask grade {}", crate_name);
        }
    }

    println!();

    Ok(())
}

fn print_criterion(c: &CriterionResult) {
    println!(
        "{}",
        format!(
            "║ {:16} │ {:16} │   {}   │ {:14} ║",
            c.name,
            truncate(&c.result, 16),
            c.grade.colored(),
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

/// Find workspace root by looking for root Cargo.toml with [workspace]
fn find_workspace_root(sh: &Shell) -> Result<String> {
    let output = cmd!(sh, "cargo locate-project --workspace --message-format plain").read()?;
    let cargo_toml = output.trim();
    let root = Path::new(cargo_toml)
        .parent()
        .context("Could not find workspace root")?;
    Ok(root.to_string_lossy().to_string())
}

/// Find the path to a crate within the workspace
fn find_crate_path(_sh: &Shell, crate_name: &str) -> Result<String> {
    // Try common locations
    let locations = ["crates", "mesh", "geometry", "routing", "ml", "vision", "sim"];

    for loc in &locations {
        let path = format!("{}/{}", loc, crate_name);
        if Path::new(&path).exists() {
            return Ok(path);
        }
    }

    // Try direct path
    if Path::new(crate_name).exists() {
        return Ok(crate_name.to_string());
    }

    bail!(
        "Could not find crate '{}'. Looked in: {:?}",
        crate_name,
        locations
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
    })
}

/// Grade documentation by checking for warnings
fn grade_documentation(sh: &Shell, crate_name: &str) -> Result<CriterionResult> {
    let output = cmd!(sh, "cargo doc --no-deps -p {crate_name}")
        .env("RUSTDOCFLAGS", "-D warnings")
        .ignore_status()
        .ignore_stderr()
        .read()
        .unwrap_or_default();

    let warning_count = output.matches("warning:").count();
    let error_count = output.matches("error").count();

    let total_issues = warning_count + error_count;

    let grade = if total_issues == 0 {
        Grade::A
    } else if total_issues <= 5 {
        Grade::B
    } else if total_issues <= 20 {
        Grade::C
    } else {
        Grade::F
    };

    Ok(CriterionResult {
        name: "2. Documentation",
        result: format!("{} warnings", total_issues),
        grade,
        threshold: "0",
    })
}

/// Grade clippy warnings
fn grade_clippy(sh: &Shell, crate_name: &str) -> Result<CriterionResult> {
    let output = cmd!(
        sh,
        "cargo clippy -p {crate_name} --all-targets --all-features -- -D warnings"
    )
    .ignore_status()
    .ignore_stderr()
    .read()
    .unwrap_or_default();

    let warning_count = output.matches("warning:").count();
    let error_count = output.matches("error[").count();

    let total_issues = warning_count + error_count;

    let grade = if total_issues == 0 {
        Grade::A
    } else if total_issues <= 5 {
        Grade::B
    } else if total_issues <= 20 {
        Grade::C
    } else {
        Grade::F
    };

    Ok(CriterionResult {
        name: "3. Clippy",
        result: format!("{} warnings", total_issues),
        grade,
        threshold: "0",
    })
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
        });
    }

    // First, find where test sections start in each file
    let test_starts = cmd!(sh, "grep -rn '#\\[cfg(test)\\]' {src_path} --include='*.rs'")
        .ignore_status()
        .ignore_stderr()
        .read()
        .unwrap_or_default();

    // Build a map of file -> test start line
    let mut test_start_lines: std::collections::HashMap<String, usize> = std::collections::HashMap::new();
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
            if trimmed.starts_with("//") || trimmed.starts_with("///") || trimmed.starts_with("//!") {
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
    })
}

/// Check that a crate has no Bevy dependencies
///
/// Layer 0 crates should not depend on Bevy or windowing libraries.
/// Note: `wgpu` is NOT considered a Bevy dependency - it's a standalone
/// WebGPU implementation that can be used independently for compute shaders.
fn grade_bevy_free(sh: &Shell, crate_name: &str) -> Result<CriterionResult> {
    let output = cmd!(sh, "cargo tree -p {crate_name}")
        .ignore_status()
        .ignore_stderr()
        .read()
        .unwrap_or_default();

    // Only check for actual Bevy ecosystem dependencies
    // wgpu is NOT a Bevy dependency - it's a standalone GPU library
    let has_bevy = output.contains("bevy");
    let has_winit = output.contains("winit");

    let violations: Vec<&str> = [
        if has_bevy { Some("bevy") } else { None },
        if has_winit { Some("winit") } else { None },
    ]
    .iter()
    .filter_map(|&x| x)
    .collect();

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

    Ok(CriterionResult {
        name: "6. Bevy-free",
        result,
        grade,
        threshold: "no bevy",
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
    let locations = ["crates", "mesh", "geometry", "routing", "ml", "vision", "sim"];

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
