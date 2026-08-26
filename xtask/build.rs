//! Build script for xtask
//!
//! Automatically installs git hooks when xtask is built.
//! This ensures that anyone running `cargo xtask` commands
//! has the pre-commit and commit-msg hooks installed.

use std::fs;
use std::path::Path;

// The installer's two pure decisions, shared VERBATIM with the xtask binary so
// they are unit-tested. A build script has no test target; while these lived only
// here, gutting the ownership check passed the whole suite.
include!("src/hook_install.rs");

/// Pre-commit hook content. Single source: `xtask/hooks/pre-commit`.
const PRE_COMMIT_HOOK: &str = include_str!("hooks/pre-commit");

/// Commit message hook content. Single source: `xtask/hooks/commit-msg`.
const COMMIT_MSG_HOOK: &str = include_str!("hooks/commit-msg");

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
    install_hook_if_needed(
        &pre_commit_path,
        PRE_COMMIT_HOOK,
        "pre-commit",
        title_of(PRE_COMMIT_HOOK),
    );

    // Install commit-msg hook if missing or outdated
    let commit_msg_path = hooks_dir.join("commit-msg");
    install_hook_if_needed(
        &commit_msg_path,
        COMMIT_MSG_HOOK,
        "commit-msg",
        title_of(COMMIT_MSG_HOOK),
    );

    // Tell cargo to rerun if hooks are deleted
    println!("cargo:rerun-if-changed=../.git/hooks/pre-commit");
    // The hook text itself is now a tracked file, so edits to it must
    // retrigger this build script or the change never reaches .git/hooks.
    println!("cargo:rerun-if-changed=hooks/pre-commit");
    println!("cargo:rerun-if-changed=hooks/commit-msg");
    println!("cargo:rerun-if-changed=../.git/hooks/commit-msg");
}

fn install_hook_if_needed(path: &Path, content: &str, name: &str, marker: &str) {
    // The DECISION is `should_replace` in src/hook_install.rs — pure, and unit-tested
    // there. Only the filesystem work lives here, because a build script cannot be
    // tested and this logic has already been wrong twice.
    let existing = if path.exists() {
        Some(fs::read_to_string(path).map_err(|_| ()))
    } else {
        None
    };
    if !should_replace(
        existing.as_ref().map(|r| r.as_deref().map_err(|_| ())),
        content,
        marker,
    ) {
        return;
    }

    if let Err(e) = fs::write(path, content) {
        // Don't fail the build, just warn.
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
