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

    // ⚠ EMITTED BEFORE ANY EARLY RETURN. A build script that returns without any
    // `rerun-if-changed` falls back to cargo's package-mtime heuristic, so the
    // advice in the warnings below ("create it and rebuild") would not re-run this
    // script at all and the developer would conclude the fix did not work.
    for (name, _) in HOOKS {
        println!("cargo:rerun-if-changed=hooks/{name}");
    }

    let hooks_dir = match resolve_hooks_dir(workspace_root) {
        Some(HooksDir::Repo(dir)) => dir,
        Some(HooksDir::OtherRepo(_)) => {
            // A copy of this source nested inside SOMEBODY ELSE'S checkout: git
            // resolved their repository by walking up. Automatic installation only
            // ever targets our own checkout — writing our mesh guard and our
            // `cargo fmt` into an unrelated project is the harm containment exists
            // to prevent. Silent on purpose: a vendored copy has nothing to say.
            return;
        }
        Some(HooksDir::Shared(dir)) => {
            println!(
                "cargo:warning=git is configured to read hooks from {}, which is \
                 outside this repository and shared with every other repo on this \
                 machine. CortenForge's hooks were NOT installed there and the \
                 scan/mesh guard is not armed — merge xtask/hooks/* into it yourself.",
                dir.display()
            );
            return;
        }
        None => {
            // Only worth saying anything if this looks like a checkout. xtask can be
            // built from a vendored copy with no git present, where silence is right.
            if workspace_root.join(".git").exists() {
                println!(
                    "cargo:warning=Could not determine git's hooks directory, so \
                     CortenForge's hooks were NOT installed and the scan/mesh guard \
                     is not armed."
                );
            }
            return;
        }
    };

    if !hooks_dir.exists() {
        // ⚠ NOT silent, and NOT created: under core.hooksPath the path can be
        // anywhere, and making directories is not a build script's business.
        println!(
            "cargo:warning=git's hooks directory ({}) does not exist, so CortenForge's \
             hooks were NOT installed and the scan/mesh guard is not armed. Create it, \
             then `touch xtask/build.rs` or `cargo clean -p xtask` to retry — creating \
             a directory alone does not re-run this build script.",
            hooks_dir.display()
        );
        return;
    }

    // ★ The filename comes from the same tuple as the text, so there is no pairing
    // to get wrong here.
    for (name, content) in HOOKS {
        let path = hooks_dir.join(name);
        install_hook_if_needed(&path, content, name);
        // Re-run if the installed hook is deleted, at the directory git ACTUALLY uses.
        println!("cargo:rerun-if-changed={}", path.display());
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

    if !state.is_ours_to_manage() {
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

    let mut wrote = false;
    if state.should_replace() {
        if let Err(e) = write_hook_file(path, content) {
            // Don't fail the build, just warn.
            println!("cargo:warning=Failed to install {name} hook: {e}");
            return;
        }
        wrote = true;
    }

    // ⚠ Runs even when the text was already current. See `ensure_executable`.
    #[cfg(unix)]
    ensure_executable(path, name, wrote);

    if wrote {
        // Name the PATH. In a linked worktree this is the shared common dir, so a
        // build here rewrites the hooks the main checkout uses — that must be visible.
        println!(
            "cargo:warning=Installed {name} git hook at {}",
            path.display()
        );
    }
}

/// Make sure git will actually RUN the hook at `path`.
///
/// git silently ignores a hook without an executable bit — no error, no warning, the
/// guard simply never runs. The previous build script discarded this error entirely
/// (`let _ = fs::set_permissions(..)`), so a hook with the right text and mode 0644
/// is reachable on real machines, and checking only when we WRITE would never reach
/// it: its text is current, so there is nothing to rewrite. Repair it regardless.
#[cfg(unix)]
fn ensure_executable(path: &Path, name: &str, freshly_written: bool) {
    use std::os::unix::fs::PermissionsExt;
    let already_executable = fs::metadata(path)
        .map(|m| m.permissions().mode() & 0o111 != 0)
        .unwrap_or(false);
    if already_executable {
        return;
    }

    if let Err(e) = fs::set_permissions(path, fs::Permissions::from_mode(0o755)) {
        println!(
            "cargo:warning=The {name} hook is not executable and could not be made \
             one, so git will ignore it and its checks will not run: {e}"
        );
        return;
    }

    if !freshly_written {
        println!(
            "cargo:warning=Repaired the {name} hook's executable bit — git had been \
             ignoring it, so its checks were not running."
        );
    }
}
