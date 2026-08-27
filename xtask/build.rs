//! Build script for xtask
//!
//! Automatically installs git hooks when xtask is built.
//! This ensures that anyone running `cargo xtask` commands
//! has the pre-commit and commit-msg hooks installed.

use std::fs;
use std::path::Path;

// The installer's data and its pure decisions, shared VERBATIM with the xtask binary
// so they are unit-tested. A build script has no test target; while they lived only
// here, gutting the ownership check passed the whole suite, and so did crossing the
// two hooks' filenames.
include!("src/hook_install.rs");

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

    // ★ The filename comes from the same tuple as the text, so there is no pairing
    // to get wrong here. Writing it out per-hook is what let `build.rs` cross the
    // two consts while the `setup.rs` copy of the same list was under test.
    for (name, content) in HOOKS {
        install_hook_if_needed(&hooks_dir.join(name), content, name);
    }

    for (name, _) in HOOKS {
        // Re-run if the installed hook is deleted...
        println!("cargo:rerun-if-changed=../.git/hooks/{name}");
        // ...and if its tracked source changes, or the edit never reaches .git/hooks.
        println!("cargo:rerun-if-changed=hooks/{name}");
    }
}

/// Install `content` at `path` unless something we must not touch is already there.
///
/// The DECISION is `classify` in src/hook_install.rs — pure, and unit-tested there.
/// Only the filesystem work lives here, because a build script cannot be tested and
/// this logic has already been wrong twice.
fn install_hook_if_needed(path: &Path, content: &str, name: &str) {
    // DERIVED, not a parameter. A caller that can pass a marker can pass the OTHER
    // hook's marker — the mis-pairing class this arc keeps rediscovering. Taking it
    // from the content leaves nothing to cross.
    let marker = title_of(content);

    let existing = if path.exists() {
        Some(fs::read_to_string(path).map_err(|_| ()))
    } else {
        None
    };
    let state = classify(
        existing.as_ref().map(|r| r.as_deref().map_err(|_| ())),
        content,
        marker,
    );

    if !state.should_replace() {
        // ⚠ Declining to install is only allowed to be SILENT when the hook is
        // already ours and current. Otherwise the developer's scan/mesh guard is not
        // armed and they have no way to know — that silence is how the original bug
        // survived on real machines for two releases.
        match state {
            HookState::Foreign => println!(
                "cargo:warning=Left your existing {name} hook in place, so \
                 CortenForge's is NOT installed and the scan/mesh guard is not armed. \
                 Merge xtask/hooks/{name} into yours, or move yours aside and rebuild."
            ),
            HookState::Unreadable => println!(
                "cargo:warning=Could not read the existing {name} hook, so it was left \
                 alone. CortenForge's {name} hook is NOT installed and the scan/mesh \
                 guard is not armed."
            ),
            _ => {}
        }
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
        if let Err(e) = fs::set_permissions(path, fs::Permissions::from_mode(0o755)) {
            // git ignores a hook that is not executable, and says nothing. Discarding
            // this error leaves a hook that looks installed and never runs.
            println!(
                "cargo:warning=Installed {name} hook but could not make it \
                 executable, so git will ignore it: {e}"
            );
            return;
        }
    }

    println!("cargo:warning=Installed {name} git hook");
}
