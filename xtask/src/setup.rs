//! Development environment setup
//!
//! This module handles:
//! - Installing git hooks for pre-commit quality checks
//! - Verifying required tools are installed
//! - Setting up development dependencies
//!
//! # Usage
//!
//! ```bash
//! cargo xtask setup
//! ```
//!
//! This will:
//! 1. Install pre-commit and commit-msg git hooks
//! 2. Verify cargo-audit, cargo-deny, cargo-tarpaulin are available
//! 3. Set up any additional development tooling

use anyhow::{Context, Result};
use owo_colors::OwoColorize;
use std::fs;
use std::path::Path;

/// Pre-commit hook content
///
/// This runs before every commit to catch issues early.
/// Faster than CI, provides immediate feedback.
const PRE_COMMIT_HOOK: &str = r#"#!/bin/sh
# CortenForge Pre-Commit Hook
# Installed by: cargo xtask setup
#
# This hook enforces quality standards before commits reach CI.
# See INFRASTRUCTURE.md for the full constraint specification.

set -e

echo "╔═══════════════════════════════════════════════════════════════╗"
echo "║                  CortenForge Pre-Commit Check                  ║"
echo "╚═══════════════════════════════════════════════════════════════╝"

# Format check (fast)
echo "→ Checking formatting..."
if ! cargo fmt --all -- --check 2>/dev/null; then
    echo "✗ Formatting check failed. Run: cargo fmt --all"
    exit 1
fi
echo "✓ Formatting OK"

# Clippy check (library code only - tests are checked in CI)
echo "→ Running clippy..."
if ! cargo clippy --all-features -- -D warnings 2>/dev/null; then
    echo "✗ Clippy check failed. Fix warnings before committing."
    exit 1
fi
echo "✓ Clippy OK"

# Note: unwrap/expect enforcement is handled by clippy via workspace lints
# (clippy::unwrap_used = "deny" in Cargo.toml)
# The grep-based scan was removed as it caught doc examples falsely.

echo ""
echo "╔═══════════════════════════════════════════════════════════════╗"
echo "║                    Pre-commit checks passed                    ║"
echo "╚═══════════════════════════════════════════════════════════════╝"
"#;

/// Commit message hook content
///
/// Enforces conventional commit format:
/// <type>(<scope>): <description>
const COMMIT_MSG_HOOK: &str = r#"#!/bin/sh
# CortenForge Commit Message Hook
# Installed by: cargo xtask setup
#
# Enforces conventional commit format for automated changelog generation.
# See INFRASTRUCTURE.md for details.

commit_msg=$(cat "$1")

# Allow merge commits
if echo "$commit_msg" | grep -qE "^Merge "; then
    exit 0
fi

# Allow revert commits
if echo "$commit_msg" | grep -qE "^Revert "; then
    exit 0
fi

# Conventional commit pattern:
# type(scope): description
# type: description
#
# Types: feat, fix, refactor, test, docs, chore, perf, ci, build, style
pattern="^(feat|fix|refactor|test|docs|chore|perf|ci|build|style)(\([a-z0-9-]+\))?: .+"

if ! echo "$commit_msg" | head -1 | grep -qE "$pattern"; then
    echo "✗ Commit message does not follow conventional commits format."
    echo ""
    echo "Expected format:"
    echo "  <type>(<scope>): <description>"
    echo ""
    echo "Types: feat, fix, refactor, test, docs, chore, perf, ci, build, style"
    echo ""
    echo "Examples:"
    echo "  feat(mesh-boolean): add GPU-accelerated union operation"
    echo "  fix(mesh-io): handle malformed STL headers gracefully"
    echo "  docs: update README with new examples"
    echo "  refactor(mesh-repair): extract hole-filling into separate module"
    echo ""
    echo "Your message:"
    echo "  $(head -1 "$1")"
    exit 1
fi
"#;

/// Run the setup command
pub fn run() -> Result<()> {
    println!(
        "{}",
        "╔═══════════════════════════════════════════════════════════════╗".bright_cyan()
    );
    println!(
        "{}",
        "║           CortenForge Development Environment Setup           ║".bright_cyan()
    );
    println!(
        "{}",
        "╚═══════════════════════════════════════════════════════════════╝".bright_cyan()
    );
    println!();

    // Step 1: Install git hooks
    install_git_hooks()?;

    // Step 2: Verify required tools
    verify_tools()?;

    // Step 3: Summary
    println!();
    println!("{}", "Setup complete!".bright_green().bold());
    println!();
    println!("Pre-commit hooks will now run before each commit:");
    println!("  • Formatting check (cargo fmt)");
    println!("  • Clippy lint check (library code)");
    println!();
    println!("Safety lint policy:");
    println!("  • Library code: unwrap/expect denied (via lib.rs attributes)");
    println!("  • Test code: unwrap/expect allowed (ecosystem standard)");
    println!();
    println!("Commit messages must follow conventional commits format:");
    println!("  • feat(scope): description");
    println!("  • fix(scope): description");
    println!("  • docs: description");
    println!();
    println!("{}", "Optional (Linux only):".bright_blue());
    println!("  cargo install cargo-tarpaulin");
    println!("  cargo tarpaulin -p <crate> --out Html");
    println!();
    println!("  This lets you check coverage locally before pushing.");
    println!("  CI enforces ≥90% coverage regardless - this is just for convenience.");
    println!();

    Ok(())
}

/// Install git hooks
fn install_git_hooks() -> Result<()> {
    println!("{}", "→ Installing git hooks...".bright_blue());

    let hooks_dir = Path::new(".git/hooks");

    if !hooks_dir.exists() {
        anyhow::bail!("Not in a git repository. Run this command from the repository root.");
    }

    // Install pre-commit hook
    let pre_commit_path = hooks_dir.join("pre-commit");
    fs::write(&pre_commit_path, PRE_COMMIT_HOOK).context("Failed to write pre-commit hook")?;
    make_executable(&pre_commit_path)?;
    println!("  ✓ Installed pre-commit hook");

    // Install commit-msg hook
    let commit_msg_path = hooks_dir.join("commit-msg");
    fs::write(&commit_msg_path, COMMIT_MSG_HOOK).context("Failed to write commit-msg hook")?;
    make_executable(&commit_msg_path)?;
    println!("  ✓ Installed commit-msg hook");

    Ok(())
}

/// Make a file executable on Unix systems
#[cfg(unix)]
fn make_executable(path: &Path) -> Result<()> {
    use std::os::unix::fs::PermissionsExt;
    let perms = fs::Permissions::from_mode(0o755);
    fs::set_permissions(path, perms).context("Failed to set executable permission")?;
    Ok(())
}

/// No-op on Windows (git handles hook execution)
#[cfg(not(unix))]
fn make_executable(_path: &Path) -> Result<()> {
    Ok(())
}

/// Verify required tools are installed
fn verify_tools() -> Result<()> {
    println!("{}", "→ Verifying required tools...".bright_blue());

    let tools = [
        ("cargo", "Rust package manager", true),
        ("rustfmt", "Rust formatter", true),
        ("clippy-driver", "Rust linter", true),
        (
            "cargo-audit",
            "Security scanner (cargo install cargo-audit)",
            false,
        ),
        (
            "cargo-deny",
            "Dependency policy (cargo install cargo-deny)",
            false,
        ),
        (
            "cargo-tarpaulin",
            "Coverage tool (cargo install cargo-tarpaulin)",
            false,
        ),
    ];

    let mut all_required_present = true;

    for (tool, description, required) in tools {
        let present = which::which(tool).is_ok()
            || std::process::Command::new("cargo")
                .args([tool.strip_prefix("cargo-").unwrap_or(tool), "--version"])
                .output()
                .map(|o| o.status.success())
                .unwrap_or(false);

        if present {
            println!("  ✓ {} - {}", tool.bright_green(), description);
        } else if required {
            println!(
                "  ✗ {} - {} {}",
                tool.bright_red(),
                description,
                "(REQUIRED)".red().bold()
            );
            all_required_present = false;
        } else {
            println!("  ⚠ {} - {}", tool.bright_yellow(), description);
        }
    }

    if !all_required_present {
        anyhow::bail!("Missing required tools. Install them and run setup again.");
    }

    Ok(())
}

/// Uninstall git hooks
pub fn uninstall() -> Result<()> {
    println!("{}", "→ Removing git hooks...".bright_blue());

    let hooks_dir = Path::new(".git/hooks");

    if !hooks_dir.exists() {
        println!("  No hooks directory found.");
        return Ok(());
    }

    // Remove pre-commit hook
    let pre_commit_path = hooks_dir.join("pre-commit");
    if pre_commit_path.exists() {
        fs::remove_file(&pre_commit_path)?;
        println!("  ✓ Removed pre-commit hook");
    }

    // Remove commit-msg hook
    let commit_msg_path = hooks_dir.join("commit-msg");
    if commit_msg_path.exists() {
        fs::remove_file(&commit_msg_path)?;
        println!("  ✓ Removed commit-msg hook");
    }

    println!("{}", "Hooks removed.".bright_green());
    Ok(())
}
