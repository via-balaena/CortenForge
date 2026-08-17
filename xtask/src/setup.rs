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
//! 2. Verify cargo-audit, cargo-deny, and the llvm-tools coverage component
//! 3. Set up any additional development tooling

use anyhow::{Context, Result};
use owo_colors::OwoColorize;
use std::fs;
use std::path::Path;

/// Pre-commit hook content (must match build.rs)
///
/// This runs before every commit to catch issues early.
/// Faster than CI, provides immediate feedback.
/// Only lints crates with staged Rust changes (not the full workspace).
const PRE_COMMIT_HOOK: &str = r#"#!/bin/sh
# CortenForge Pre-Commit Hook
# Installed by: cargo xtask setup
#
# This hook enforces quality standards before commits reach CI.
# See docs/INFRASTRUCTURE.md for the full constraint specification.
#
# Performance: only lints crates with staged changes (not the full workspace).

set -e

echo "╔═══════════════════════════════════════════════════════════════╗"
echo "║                  CortenForge Pre-Commit Check                  ║"
echo "╚═══════════════════════════════════════════════════════════════╝"

# Format check (fast — only checks already-formatted files, <1s)
echo "→ Checking formatting..."
if ! cargo fmt --all -- --check 2>/dev/null; then
    echo "✗ Formatting check failed. Run: cargo fmt --all"
    exit 1
fi
echo "✓ Formatting OK"

# Determine which crates have staged Rust or Cargo.toml changes.
# Pathspec `'*Cargo.toml'` matches nested manifests (sim/L0/**/Cargo.toml,
# examples/**/Cargo.toml, etc.) as well as the workspace root. A plain
# `'Cargo.toml'` pathspec would only match the workspace root.
staged_rs_files=$(git diff --cached --name-only --diff-filter=ACMR -- '*.rs' '*Cargo.toml')

if [ -z "$staged_rs_files" ]; then
    echo "→ No Rust/Cargo files staged — skipping clippy."
else
    # Extract crate names from staged file paths.
    # Walk up from each file to find nearest Cargo.toml, read [package] name.
    crates=""
    for file in $staged_rs_files; do
        dir=$(dirname "$file")
        while [ "$dir" != "." ]; do
            if [ -f "$dir/Cargo.toml" ] && grep -q '^\[package\]' "$dir/Cargo.toml"; then
                name=$(sed -n '/^\[package\]/,/^\[/{s/^name *= *"\(.*\)"/\1/p;}' "$dir/Cargo.toml")
                if [ -n "$name" ]; then
                    crates="$crates $name"
                fi
                break
            fi
            dir=$(dirname "$dir")
        done
    done

    # Deduplicate
    crates=$(echo "$crates" | tr ' ' '\n' | sort -u | tr '\n' ' ' | sed 's/ *$//')

    if [ -z "$crates" ]; then
        echo "→ Staged Rust files don't belong to a workspace crate — skipping clippy."
    else
        echo "→ Running clippy on changed crates: $crates"
        clippy_args=""
        for crate in $crates; do
            clippy_args="$clippy_args -p $crate"
        done
        if ! cargo clippy $clippy_args --all-targets --all-features -- -D warnings 2>/dev/null; then
            echo "✗ Clippy check failed. Fix errors before committing."
            exit 1
        fi
        echo "✓ Clippy OK"
    fi
fi

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
/// `<type>(<scope>): <description>`
const COMMIT_MSG_HOOK: &str = r#"#!/bin/sh
# CortenForge Commit Message Hook
# Installed by: cargo xtask setup
#
# Enforces conventional commit format for automated changelog generation.
# See docs/INFRASTRUCTURE.md for details.

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
    echo "  feat(mesh-repair): add hole-filling edge case detection"
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
    // ⚠ This block used to recommend `cargo tarpaulin` "to match CI threshold"
    // and told Mac/Windows users to rely on CI for coverage. Both were false:
    // no PR job measures coverage (`grade-all` runs `--skip-coverage` on every
    // shard), tarpaulin is a different instrument at workspace scope, and
    // `xtask grade` is cross-platform. Onboarding is the worst place to be
    // wrong about which command checks what — it is the first thing a new
    // contributor runs, and it was pointing them at a tool whose own weekly
    // job has failed every run since 2026-06-28.
    println!(
        "{}",
        "Coverage — check it locally, nothing else does:".bright_blue()
    );
    println!("  rustup component add llvm-tools-preview   # one-time");
    println!("  cargo xtask grade <crate>                 # criterion 1 reports the number");
    println!();
    println!("  Cross-platform — macOS and Windows included.");
    println!("  ⚠ PR CI does NOT measure coverage: `grade-all` runs --skip-coverage");
    println!("    on every shard. Run this on any crate you touch, before you push.");
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
        // ⚠ cargo-tarpaulin was checked here and is deliberately gone. The
        // coverage criterion is defined by `cargo xtask grade`, which uses
        // llvm-cov from the `llvm-tools` rustup COMPONENT — a different
        // instrument at a different scope, whose number does not agree with
        // the one that governs. Checking for tarpaulin taught every new
        // contributor to reach for the wrong tool.
        //
        // The component is checked after this list, not in it: it is not an
        // executable on PATH and not a `cargo <name>` subcommand, so neither
        // probe this loop uses would find it.
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

    // Last, and optional, like the two above it. Probed the way the grader
    // itself probes it — `llvm-profdata` and `llvm-cov` in the active
    // toolchain's sysroot, not `which` and not `cargo llvm-tools`, neither of
    // which finds a rustup COMPONENT. Reusing the grader's own detector is the
    // point: a looser check here would tell a contributor they are ready while
    // criterion 1 still reports "(llvm-tools n/a)".
    let coverage_desc = "Coverage component (rustup component add llvm-tools-preview)";
    match xshell::Shell::new() {
        Ok(sh) if crate::coverage_run::tools_available(&sh) => {
            println!("  ✓ {} - {}", "llvm-tools".bright_green(), coverage_desc);
        }
        Ok(_) => println!("  ⚠ {} - {}", "llvm-tools".bright_yellow(), coverage_desc),
        // A shell that will not start is not evidence the component is absent,
        // and saying so would be a verdict this never measured.
        Err(e) => println!(
            "  ⚠ {} - could not probe ({e})",
            "llvm-tools".bright_yellow()
        ),
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
