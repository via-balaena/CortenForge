//! Completion recording for A-grade crates.
//!
//! This module handles recording when a crate achieves A-grade status.

use anyhow::{bail, Context, Result};
use chrono::Utc;
use owo_colors::OwoColorize;
use std::fs;
use std::io::{self, Write};
use std::path::Path;
use xshell::{cmd, Shell};

/// Record A-grade completion for a crate
pub fn run(crate_name: &str, skip_review: bool) -> Result<()> {
    let sh = Shell::new()?;

    println!();
    println!("{}", "Recording A-Grade Completion".bold());
    println!("{}", "============================".bold());
    println!();

    // Find the crate path
    let crate_path = find_crate_path(crate_name)?;

    // First, verify automated criteria pass
    println!("{}", "Step 1: Verifying automated criteria...".cyan());

    let grade_output = cmd!(sh, "cargo xtask grade {crate_name}")
        .ignore_status()
        .read()?;

    // Check if we got automated A grade
    if !grade_output.contains("AUTOMATED") || grade_output.contains("Refactor required") {
        println!();
        println!(
            "{}",
            "✗ Automated criteria not all A-grade.".red().bold()
        );
        println!("Run `cargo xtask grade {}` to see details.", crate_name);
        std::process::exit(1);
    }

    println!("  {} Automated criteria: A", "✓".green());

    // API Review
    println!();
    println!("{}", "Step 2: API Design Review".cyan());

    if !skip_review {
        println!();
        println!("{}", "The API Design criterion requires manual review.".yellow());
        println!();
        println!("Review checklist (from STANDARDS.md):");
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

        // Write COMPLETION.md
        write_completion(&crate_path, crate_name, reviewer)?;

        // Update project-wide log
        update_completion_log(crate_name, reviewer)?;
    } else {
        println!(
            "  {} Skipping review (--skip-review)",
            "⚠".yellow()
        );
        write_completion(&crate_path, crate_name, "automated")?;
        update_completion_log(crate_name, "automated")?;
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

fn find_crate_path(crate_name: &str) -> Result<String> {
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

fn write_completion(crate_path: &str, crate_name: &str, reviewer: &str) -> Result<()> {
    let now = Utc::now();
    let date = now.format("%Y-%m-%d").to_string();
    let timestamp = now.format("%Y-%m-%dT%H:%M:%SZ").to_string();

    let content = format!(
        r#"# {crate_name} - A-Grade Completion

> This crate meets all seven A-grade criteria as defined in [STANDARDS.md](../../STANDARDS.md).

## Completion Record

| Field | Value |
|-------|-------|
| **Crate** | `{crate_name}` |
| **Date** | {date} |
| **Reviewer** | {reviewer} |
| **Timestamp** | {timestamp} |

## Criteria Status

| # | Criterion | Status | Notes |
|---|-----------|--------|-------|
| 1 | Test Coverage | ✅ A | ≥90% line coverage |
| 2 | Documentation | ✅ A | Zero warnings |
| 3 | Clippy | ✅ A | Zero warnings |
| 4 | Safety | ✅ A | Zero unwrap/expect |
| 5 | Dependencies | ✅ A | Minimal, justified |
| 6 | Bevy-free | ✅ A | No bevy in tree |
| 7 | API Design | ✅ A | Reviewed by {reviewer} |

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
    fs::write(&completion_path, content).context("Failed to write COMPLETION.md")?;

    println!("  {} Created {}", "✓".green(), completion_path);

    Ok(())
}

fn update_completion_log(crate_name: &str, reviewer: &str) -> Result<()> {
    let log_path = "COMPLETION_LOG.md";
    let now = Utc::now();
    let date = now.format("%Y-%m-%d").to_string();

    let entry = format!("| {} | {} | {} |\n", date, crate_name, reviewer);

    if Path::new(log_path).exists() {
        // Append to existing log
        let content = fs::read_to_string(log_path)?;
        if content.contains(&format!("| {} |", crate_name)) {
            // Already has an entry, update it
            println!(
                "  {} {} already in completion log (updating)",
                "⚠".yellow(),
                crate_name
            );
        } else {
            // Append new entry
            let mut file = fs::OpenOptions::new().append(true).open(log_path)?;
            file.write_all(entry.as_bytes())?;
            println!("  {} Updated {}", "✓".green(), log_path);
        }
    } else {
        // Create new log
        let content = format!(
            r#"# CortenForge Completion Log

> Record of all crates that have achieved A-grade status.

See [STANDARDS.md](./STANDARDS.md) for the seven criteria.
See [CONTRIBUTING.md](./CONTRIBUTING.md) for the workflow.

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
