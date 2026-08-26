//! Build script for xtask
//!
//! Automatically installs git hooks when xtask is built.
//! This ensures that anyone running `cargo xtask` commands
//! has the pre-commit and commit-msg hooks installed.

use std::fs;
use std::path::Path;

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

/// The hook's own title line, used to recognise a hook as ours.
///
/// DERIVED, never written down twice. A hardcoded marker is a second copy of a
/// string that lives in the hook file, and the moment the two disagree the updater
/// silently stops recognising its own hooks — which is precisely the bug this arc
/// exists to fix, reintroduced one level up. Taking it from the text makes that
/// impossible.
fn title_of(hook: &str) -> &str {
    hook.lines()
        .nth(1)
        .expect("hook must have a title line")
        .trim_start_matches("# ")
}

fn install_hook_if_needed(path: &Path, content: &str, name: &str, marker: &str) {
    let needs_install = if path.exists() {
        match fs::read_to_string(path) {
            // Overwrite any hook that is OURS, by either installer's stamp.
            //
            // ⚠ This used to match only "Auto-installed by: xtask build.rs", and that
            // single line is why the scan/mesh guard never reached most checkouts.
            // `cargo xtask setup` wrote a hook stamped "Installed by: cargo xtask
            // setup"; this function then classified it as somebody else's file and
            // refused to touch it, FOREVER. #709 added the mesh guard to the build.rs
            // copy only, so every machine set up via `xtask setup` kept a hook with no
            // guard and no way to ever get one. Matching the title line instead heals
            // those checkouts on the next build, because both stamps carry it.
            Ok(existing) if existing.contains(marker) => existing != content,
            // Hook exists but isn't ours — don't overwrite.
            Ok(_) => false,
            // A hook we cannot READ is not a hook we may overwrite. This used to
            // return true, which would clobber a non-UTF-8 hook belonging to
            // somebody else — the exact thing the ownership check exists to prevent.
            Err(_) => false,
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
