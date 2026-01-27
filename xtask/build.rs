//! Build script for xtask
//!
//! Automatically installs git hooks when xtask is built.
//! This ensures that anyone running `cargo xtask` commands
//! has the pre-commit and commit-msg hooks installed.

use std::fs;
use std::path::Path;

/// Pre-commit hook content (must match setup.rs)
const PRE_COMMIT_HOOK: &str = r#"#!/bin/sh
# CortenForge Pre-Commit Hook
# Auto-installed by: xtask build.rs
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
# Note: We use -W clippy::all instead of -D warnings because the workspace
# has pedantic/nursery lints enabled which produce many warnings.
# CI will catch all warnings; pre-commit just blocks obvious errors.
echo "→ Running clippy..."
if ! cargo clippy --all-features 2>/dev/null; then
    echo "✗ Clippy check failed. Fix errors before committing."
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

/// Commit message hook content (must match setup.rs)
const COMMIT_MSG_HOOK: &str = r#"#!/bin/sh
# CortenForge Commit Message Hook
# Auto-installed by: xtask build.rs
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

fn main() {
    // Skip in CI environments
    if std::env::var("CI").is_ok() || std::env::var("GITHUB_ACTIONS").is_ok() {
        return;
    }

    // Find the workspace root (where .git is)
    let manifest_dir = std::env::var("CARGO_MANIFEST_DIR").unwrap();
    let workspace_root = Path::new(&manifest_dir).parent().unwrap();
    let hooks_dir = workspace_root.join(".git/hooks");

    // Only proceed if we're in a git repo
    if !hooks_dir.exists() {
        return;
    }

    // Install pre-commit hook if missing or outdated
    let pre_commit_path = hooks_dir.join("pre-commit");
    install_hook_if_needed(&pre_commit_path, PRE_COMMIT_HOOK, "pre-commit");

    // Install commit-msg hook if missing or outdated
    let commit_msg_path = hooks_dir.join("commit-msg");
    install_hook_if_needed(&commit_msg_path, COMMIT_MSG_HOOK, "commit-msg");

    // Tell cargo to rerun if hooks are deleted
    println!("cargo:rerun-if-changed=../.git/hooks/pre-commit");
    println!("cargo:rerun-if-changed=../.git/hooks/commit-msg");
}

fn install_hook_if_needed(path: &Path, content: &str, name: &str) {
    let needs_install = if path.exists() {
        // Check if hook content matches (might be outdated)
        match fs::read_to_string(path) {
            Ok(existing) => !existing.contains("Auto-installed by: xtask build.rs"),
            Err(_) => true,
        }
    } else {
        true
    };

    if needs_install {
        if let Err(e) = fs::write(path, content) {
            // Don't fail the build, just warn
            println!("cargo:warning=Failed to install {name} hook: {e}");
            return;
        }

        #[cfg(unix)]
        {
            use std::os::unix::fs::PermissionsExt;
            let _ = fs::set_permissions(path, fs::Permissions::from_mode(0o755));
        }

        println!("cargo:warning=Installed {name} git hook");
    }
}
