//! Completion recording for A-grade crates.
//!
//! This module handles recording when a crate achieves A-grade status.

use crate::grade;
use anyhow::{bail, Result};
use chrono::Utc;
use owo_colors::OwoColorize;
use std::fs;
use std::io::{self, Write};
use std::path::Path;
use xshell::Shell;

/// Record A-grade completion for a crate
pub fn run(crate_name: &str, force: bool) -> Result<()> {
    let sh = Shell::new()?;

    println!();
    println!("{}", "Recording A-Grade Completion".bold());
    println!("{}", "============================".bold());
    println!();

    // Use the shared cargo-metadata path lookup (ss5.2)
    let crate_path = grade::find_crate_path(&sh, crate_name)?;

    // Run the rebuilt grade tool internally (ss5.1)
    println!("{}", "Step 1: Verifying automated criteria...".cyan());
    let report = grade::evaluate(&sh, crate_name)?;

    // Check automated grade
    let passes = matches!(
        report.automated_grade,
        grade::Grade::A | grade::Grade::APlus
    );
    if !passes {
        println!();
        println!(
            "{}",
            format!(
                "✗ Automated grade: {}. Not all criteria pass.",
                report.automated_grade.as_str()
            )
            .red()
            .bold()
        );
        println!("Run `cargo xtask grade {}` to see details.", crate_name);
        std::process::exit(1);
    }

    println!(
        "  {} Automated criteria: {}",
        "✓".green(),
        report.automated_grade.as_str()
    );

    // API Review
    println!();
    println!("{}", "Step 2: API Design Review".cyan());

    if force {
        // --force bypass (ss5.4)
        println!("{}", "  ⚠ Bypassing manual API review (--force).".yellow());
        println!(
            "{}",
            "    Criterion 7 will be recorded as 'automated (forced)'.".yellow()
        );
        write_completion(&crate_path, crate_name, "automated (forced)", &report)?;
        update_completion_log(crate_name, "automated (forced)")?;
    } else {
        // Interactive review flow (ss5.5)
        println!();
        println!(
            "{}",
            "The API Design criterion requires manual review.".yellow()
        );
        println!();
        println!("Review checklist (from docs/STANDARDS.md):");
        println!("  □ Follows Rust API Guidelines");
        println!("  □ Naming consistent with stdlib and CortenForge");
        println!("  □ Types appropriately generic or concrete");
        println!("  □ Errors meaningful and actionable");
        println!("  □ #[must_use] on important return values");
        println!("  □ #[non_exhaustive] on extensible enums");
        println!("  □ Builder pattern for complex construction");
        println!("  □ No unnecessary allocations in hot paths");
        println!();

        print!("Have you reviewed the API against this checklist? [y/N]: ");
        io::stdout().flush()?;

        let mut input = String::new();
        io::stdin().read_line(&mut input)?;

        if !input.trim().eq_ignore_ascii_case("y") {
            println!();
            println!(
                "{}",
                "Completion cancelled. Review API and try again.".yellow()
            );
            return Ok(());
        }

        print!("Reviewer name (for the record): ");
        io::stdout().flush()?;

        let mut reviewer = String::new();
        io::stdin().read_line(&mut reviewer)?;
        let reviewer = reviewer.trim();

        if reviewer.is_empty() {
            bail!("Reviewer name required");
        }

        println!("  {} API Design reviewed by: {}", "✓".green(), reviewer);

        write_completion(&crate_path, crate_name, reviewer, &report)?;
        update_completion_log(crate_name, reviewer)?;
    }

    println!();
    println!("{}", "═══════════════════════════════════════".green());
    println!(
        "{}",
        format!("  ✓ {} is now A-GRADE", crate_name).green().bold()
    );
    println!("{}", "═══════════════════════════════════════".green());
    println!();

    Ok(())
}

/// Write COMPLETION.md with actual measured values from GradeReport (ss5.3).
fn write_completion(
    crate_path: &str,
    crate_name: &str,
    reviewer: &str,
    report: &grade::GradeReport,
) -> Result<()> {
    let now = Utc::now();
    let date = now.format("%Y-%m-%d").to_string();
    let timestamp = now.format("%Y-%m-%dT%H:%M:%SZ").to_string();

    // Build criteria table rows from actual measured values
    let mut criteria_rows = String::new();
    for c in &report.criteria {
        let status = if c.grade == grade::Grade::Manual {
            format!("✅ A | Reviewed by {}", reviewer)
        } else {
            format!("✅ {} | {}", c.grade.as_str(), c.measured_detail)
        };
        criteria_rows.push_str(&format!("| {} | {} |\n", c.name, status));
    }

    let content = format!(
        r#"# {crate_name} - A-Grade Completion

> This crate meets all seven A-grade criteria as defined in [STANDARDS.md](../../docs/STANDARDS.md).

## Completion Record

| Field | Value |
|-------|-------|
| **Crate** | `{crate_name}` |
| **Date** | {date} |
| **Reviewer** | {reviewer} |
| **Timestamp** | {timestamp} |

## Criteria Status

| Criterion | Status |
|-----------|--------|
{criteria_rows}
## Maintenance

This crate must maintain A-grade status. Any PR that causes a criterion to drop below A will be blocked by CI.

To verify current status:

```bash
cargo xtask grade {crate_name}
```

---

*Generated by `cargo xtask complete {crate_name}`*
"#
    );

    let completion_path = format!("{}/COMPLETION.md", crate_path);
    fs::write(&completion_path, content).context_msg("Failed to write COMPLETION.md")?;

    println!("  {} Created {}", "✓".green(), completion_path);

    Ok(())
}

fn update_completion_log(crate_name: &str, reviewer: &str) -> Result<()> {
    let log_path = "docs/archive/COMPLETION_LOG.md";
    let now = Utc::now();
    let date = now.format("%Y-%m-%d").to_string();

    let entry = format!("| {} | {} | {} |\n", date, crate_name, reviewer);

    if Path::new(log_path).exists() {
        // Append to existing log
        let content = fs::read_to_string(log_path)?;
        if content.contains(&format!("| {} |", crate_name)) {
            println!(
                "  {} {} already in completion log (updating)",
                "⚠".yellow(),
                crate_name
            );
        } else {
            let mut file = fs::OpenOptions::new().append(true).open(log_path)?;
            file.write_all(entry.as_bytes())?;
            println!("  {} Updated {}", "✓".green(), log_path);
        }
    } else {
        // Create new log
        let content = format!(
            r#"# CortenForge Completion Log

> Record of all crates that have achieved A-grade status.

See [STANDARDS.md](../STANDARDS.md) for the seven criteria.
See [CONTRIBUTING.md](../../CONTRIBUTING.md) for the workflow.

## A-Grade Crates

| Date | Crate | Reviewer |
|------|-------|----------|
{}"#,
            entry
        );

        fs::write(log_path, content)?;
        println!("  {} Created {}", "✓".green(), log_path);
    }

    Ok(())
}

/// Helper trait for adding context to fs errors.
trait ContextMsg<T> {
    fn context_msg(self, msg: &str) -> Result<T>;
}

impl<T> ContextMsg<T> for std::result::Result<T, std::io::Error> {
    fn context_msg(self, msg: &str) -> Result<T> {
        self.map_err(|e| anyhow::anyhow!("{}: {}", msg, e))
    }
}
