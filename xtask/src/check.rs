//! Workspace-wide quality checks.
//!
//! This module runs all automated quality checks across the entire workspace.

use anyhow::{Context, Result};
use owo_colors::OwoColorize;
use xshell::{cmd, Shell};

/// Run all checks (non-CI mode - warnings are reported but don't fail)
pub fn run(ci_mode: bool) -> Result<()> {
    let sh = Shell::new()?;

    println!();
    println!("{}", "CortenForge Quality Check".bold());
    println!("{}", "=========================".bold());
    println!();

    let mut all_passed = true;

    // 1. Formatting
    println!("{}", "Checking formatting...".dimmed());
    let fmt_result = run_fmt_check(&sh);
    report_result("Formatting", &fmt_result);
    if fmt_result.is_err() {
        all_passed = false;
    }

    // 2. Clippy
    println!("{}", "Running clippy...".dimmed());
    let clippy_result = run_clippy(&sh);
    report_result("Clippy", &clippy_result);
    if clippy_result.is_err() {
        all_passed = false;
    }

    // 3. Tests
    println!("{}", "Running tests...".dimmed());
    let test_result = run_tests(&sh);
    report_result("Tests", &test_result);
    if test_result.is_err() {
        all_passed = false;
    }

    // 4. Documentation
    println!("{}", "Building documentation...".dimmed());
    let doc_result = run_doc_check(&sh);
    report_result("Documentation", &doc_result);
    if doc_result.is_err() {
        all_passed = false;
    }

    // 5. Safety scan
    println!("{}", "Scanning for safety violations...".dimmed());
    let safety_result = run_safety_scan(&sh);
    report_result("Safety", &safety_result);
    if safety_result.is_err() {
        all_passed = false;
    }

    println!();

    if all_passed {
        println!("{}", "✓ All checks passed!".green().bold());
        Ok(())
    } else if ci_mode {
        println!("{}", "✗ Some checks failed.".red().bold());
        std::process::exit(1);
    } else {
        println!("{}", "⚠ Some checks failed. Fix before committing.".yellow());
        Ok(())
    }
}

/// Run full CI suite
pub fn run_ci() -> Result<()> {
    let sh = Shell::new()?;

    println!();
    println!("{}", "CortenForge CI Suite".bold());
    println!("{}", "====================".bold());
    println!();
    println!(
        "{}",
        "Running the same checks as GitHub Actions...".dimmed()
    );
    println!();

    let mut failures = Vec::new();

    // 1. Formatting (must be exact)
    println!("{}", "Step 1/6: Checking formatting...".cyan());
    if let Err(e) = run_fmt_check(&sh) {
        failures.push(format!("Formatting: {}", e));
        println!("  {} Formatting check failed", "✗".red());
    } else {
        println!("  {} Formatting OK", "✓".green());
    }

    // 2. Clippy with all features
    println!("{}", "Step 2/6: Running clippy...".cyan());
    if let Err(e) = run_clippy(&sh) {
        failures.push(format!("Clippy: {}", e));
        println!("  {} Clippy failed", "✗".red());
    } else {
        println!("  {} Clippy OK", "✓".green());
    }

    // 3. Tests with all features
    println!("{}", "Step 3/6: Running tests...".cyan());
    if let Err(e) = run_tests(&sh) {
        failures.push(format!("Tests: {}", e));
        println!("  {} Tests failed", "✗".red());
    } else {
        println!("  {} Tests OK", "✓".green());
    }

    // 4. Documentation build
    println!("{}", "Step 4/6: Building docs...".cyan());
    if let Err(e) = run_doc_check(&sh) {
        failures.push(format!("Documentation: {}", e));
        println!("  {} Documentation failed", "✗".red());
    } else {
        println!("  {} Documentation OK", "✓".green());
    }

    // 5. Safety scan
    println!("{}", "Step 5/6: Safety scan...".cyan());
    if let Err(e) = run_safety_scan(&sh) {
        failures.push(format!("Safety: {}", e));
        println!("  {} Safety scan failed", "✗".red());
    } else {
        println!("  {} Safety scan OK", "✓".green());
    }

    // 6. Dependency check (cargo-deny)
    println!("{}", "Step 6/6: Checking dependencies...".cyan());
    if let Err(e) = run_deny(&sh) {
        // Don't fail on this if cargo-deny isn't installed
        println!("  {} Dependency check: {}", "⚠".yellow(), e);
    } else {
        println!("  {} Dependencies OK", "✓".green());
    }

    println!();

    if failures.is_empty() {
        println!("{}", "═══════════════════════════════════════".green());
        println!("{}", "  ✓ CI PASSED - Ready to push".green().bold());
        println!("{}", "═══════════════════════════════════════".green());
        Ok(())
    } else {
        println!("{}", "═══════════════════════════════════════".red());
        println!("{}", "  ✗ CI FAILED".red().bold());
        println!("{}", "═══════════════════════════════════════".red());
        println!();
        println!("Failures:");
        for f in &failures {
            println!("  - {}", f.red());
        }
        std::process::exit(1);
    }
}

fn report_result(name: &str, result: &Result<()>) {
    match result {
        Ok(()) => println!("  {} {}", "✓".green(), name),
        Err(e) => println!("  {} {} - {}", "✗".red(), name, e),
    }
}

fn run_fmt_check(sh: &Shell) -> Result<()> {
    cmd!(sh, "cargo fmt --all -- --check")
        .run()
        .context("Formatting check failed")?;
    Ok(())
}

fn run_clippy(sh: &Shell) -> Result<()> {
    cmd!(sh, "cargo clippy --all-targets --all-features -- -D warnings")
        .run()
        .context("Clippy check failed")?;
    Ok(())
}

fn run_tests(sh: &Shell) -> Result<()> {
    cmd!(sh, "cargo test --all-features")
        .run()
        .context("Tests failed")?;
    Ok(())
}

fn run_doc_check(sh: &Shell) -> Result<()> {
    cmd!(sh, "cargo doc --no-deps --all-features")
        .env("RUSTDOCFLAGS", "-D warnings")
        .run()
        .context("Documentation build failed")?;
    Ok(())
}

fn run_safety_scan(sh: &Shell) -> Result<()> {
    // Scan all src/ directories for unwrap/expect/panic in library code
    let output = cmd!(sh, "grep -rn --include='*.rs' -E '\\.(unwrap|expect)\\(' . 2>/dev/null")
        .ignore_status()
        .read()
        .unwrap_or_default();

    let mut violations = 0;
    for line in output.lines() {
        // Skip test files and test modules
        if line.contains("/tests/")
            || line.contains("_test.rs")
            || line.contains("#[test]")
            || line.contains("#[cfg(test)]")
            || line.contains("/examples/")
            || line.contains("/benches/")
            || line.contains("/xtask/") // Allow in dev tooling
        {
            continue;
        }

        // Skip comments
        let trimmed = line.split(':').last().unwrap_or("").trim();
        if trimmed.starts_with("//") || trimmed.starts_with("///") {
            continue;
        }

        violations += 1;
    }

    if violations > 0 {
        anyhow::bail!(
            "Found {} unwrap/expect calls in library code. See STANDARDS.md criterion 4.",
            violations
        );
    }

    Ok(())
}

fn run_deny(sh: &Shell) -> Result<()> {
    // Check if cargo-deny is installed
    let has_deny = cmd!(sh, "cargo deny --version")
        .ignore_status()
        .read()
        .is_ok();

    if !has_deny {
        anyhow::bail!("cargo-deny not installed. Run: cargo install cargo-deny");
    }

    cmd!(sh, "cargo deny check")
        .run()
        .context("Dependency check failed")?;

    Ok(())
}
