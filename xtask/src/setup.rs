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
use std::path::{Path, PathBuf};

/// Pre-commit hook content (must match build.rs)
///
/// This runs before every commit to catch issues early.
/// Faster than CI, provides immediate feedback.
/// Only lints crates with staged Rust changes (not the full workspace).
const PRE_COMMIT_HOOK: &str = include_str!("../hooks/pre-commit");

/// Commit message hook content
///
/// Enforces conventional commit format:
/// `<type>(<scope>): <description>`
const COMMIT_MSG_HOOK: &str = include_str!("../hooks/commit-msg");

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
    // tarpaulin was a different instrument at workspace scope, and `xtask
    // grade` is cross-platform. Onboarding is the worst place to be wrong
    // about which command checks what — it is the first thing a new
    // contributor runs.
    //
    // ⚠ Kept honest a second time when `scheduled.yml` started measuring
    // coverage: this used to read "nothing else does", which stopped being
    // true the moment a weekly job ran the same grader. The distinction that
    // matters to a contributor is not local-vs-CI but BEFORE-vs-AFTER their
    // merge — no PR shard measures coverage, so the weekly run cannot stop
    // them shipping a regression, only tell them about it afterwards.
    println!(
        "{}",
        "Coverage — check it locally; CI will not catch it before you merge:".bright_blue()
    );
    // ⚠ No `rustup component add` line here. The tool check above already
    // reports llvm-tools CONDITIONALLY — a ✓ when present, the install command
    // when not. Repeating it unconditionally told a reader to run something
    // the same output had just confirmed they did not need.
    println!("  • cargo xtask grade <crate>   — criterion 1 reports the number");
    println!("  • Cross-platform; macOS and Windows included");
    println!("  • PR CI does NOT measure coverage — `grade-all` runs");
    println!("    --skip-coverage on every shard, so run this on any crate");
    println!("    you touch, before you push");
    println!("  • The weekly `Coverage` job in scheduled.yml runs the same");
    println!("    grader without that flag — it reports a regression the");
    println!("    week AFTER it lands, which is not a substitute for the above");
    println!();

    Ok(())
}

/// The repository's `.git/hooks`, addressed from the workspace root.
///
/// ★ Both callers used a bare `.git/hooks`, which `std::fs` resolves against
/// the PROCESS working directory. Their symptoms differed, and the silent one
/// is why this exists:
///
/// - `install_git_hooks` bailed with "Not in a git repository. Run this command
///   from the repository root." — wrong about the cause, but honest and
///   actionable.
/// - `uninstall` printed "No hooks directory found." and returned **`Ok(())`**.
///   Run from a subdirectory it told the operator there was nothing to remove
///   and left the hooks installed — a no-op reported as success, the same shape
///   as the grader's cwd bugs and `complete`'s lost log entry.
///
/// Rooted, "no hooks directory" becomes a statement about the repository rather
/// than about where the operator happened to be standing.
fn git_hooks_dir() -> Result<PathBuf> {
    let sh = xshell::Shell::new()?;
    Ok(PathBuf::from(crate::grade::find_workspace_root(&sh)?).join(".git/hooks"))
}

/// Install git hooks
fn install_git_hooks() -> Result<()> {
    println!("{}", "→ Installing git hooks...".bright_blue());

    let hooks_dir = git_hooks_dir()?;

    if !hooks_dir.exists() {
        anyhow::bail!(
            "No {} — not a git repository (or a worktree, whose .git is a file).",
            hooks_dir.display()
        );
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

    let hooks_dir = git_hooks_dir()?;

    if !hooks_dir.exists() {
        println!("  No hooks directory found at {}.", hooks_dir.display());
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

#[cfg(test)]
mod hook_tests {
    use super::{COMMIT_MSG_HOOK, PRE_COMMIT_HOOK};

    /// The scan/mesh guard must actually be in the hook that gets installed.
    ///
    /// This is the gate that #709 needed and did not have. That PR is titled
    /// "make 'no scan or mesh binaries' a hard rule, **enforced twice**" and added
    /// the guard to `xtask/build.rs`'s copy of the hook only — `setup.rs` held a
    /// second, independent copy and never received it. So `cargo xtask setup`
    /// installed a hook with no guard, and the rule was enforced ONCE on every
    /// machine set up that way. Nothing failed, because nothing checked.
    ///
    /// Both consts now `include_str!` the same file, so this cannot drift again —
    /// but the assertion stays, because "they read the same file" is a property of
    /// today's code and this is a property of the SHIPPED ARTIFACT.
    #[test]
    fn the_installed_pre_commit_hook_carries_the_scan_mesh_guard() {
        for needle in ["CF_ALLOW_MESH", "*.stl", "*.obj", "*.ply", "*.3mf", "*.mtl"] {
            assert!(
                PRE_COMMIT_HOOK.contains(needle),
                "the pre-commit hook shipped to .git/hooks is missing {needle:?}. \
                 The repository is PUBLIC and the casting pipeline's inputs are \
                 anatomical scans of a real person; .gitignore cannot stop \
                 `git add -f`, which is the hole this guard exists to close."
            );
        }
    }

    /// The updater in `build.rs` recognises a hook as ours by its TITLE line, so
    /// that a hook written by either installer gets healed on the next build. If a
    /// title changes without `build.rs`'s marker changing with it, every existing
    /// checkout silently stops receiving hook updates — which is exactly the
    /// failure this arc fixed, and it is invisible until something else breaks.
    #[test]
    fn hook_titles_match_the_markers_the_updater_looks_for() {
        assert!(
            PRE_COMMIT_HOOK.contains("CortenForge Pre-Commit Hook"),
            "pre-commit title changed; update the marker in xtask/build.rs::main"
        );
        assert!(
            COMMIT_MSG_HOOK.contains("CortenForge Commit Message Hook"),
            "commit-msg title changed; update the marker in xtask/build.rs::main"
        );
    }

    /// Both hooks must be runnable `sh`, since git executes them directly.
    #[test]
    fn hooks_start_with_a_posix_sh_shebang() {
        assert!(PRE_COMMIT_HOOK.starts_with("#!/bin/sh\n"));
        assert!(COMMIT_MSG_HOOK.starts_with("#!/bin/sh\n"));
    }
}
