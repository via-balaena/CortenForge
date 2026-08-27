//! Build script for xtask
//!
//! Automatically installs git hooks when xtask is built.
//! This ensures that anyone running `cargo xtask` commands
//! has the pre-commit and commit-msg hooks installed.

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

    // ⚠ EMITTED BEFORE EVERY RETURN BELOW — the `CI` skip above is the one exception,
    // and it is deliberate: CI does not commit. A build script that returns without any
    // `rerun-if-changed` falls back to cargo's package-mtime heuristic, so the
    // advice in the warnings below ("create it and rebuild") would not re-run this
    // script at all and the developer would conclude the fix did not work.
    for (name, _) in HOOKS {
        println!("cargo:rerun-if-changed=hooks/{name}");
    }

    // The DECISION is `build_outcome` in src/hook_install.rs — pure, and unit-tested
    // there. This renders it. Keeping the arms here is what left the guards a fresh
    // clone depends on with no gate at all.
    let hooks_dir = match build_outcome(
        resolve_hooks_dir(workspace_root),
        workspace_root.join(".git").exists(),
        std::env::var_os("GIT_DIR").is_some() || std::env::var_os("GIT_WORK_TREE").is_some(),
    ) {
        BuildOutcome::Install(dir) => dir,
        BuildOutcome::Silent => return,
        BuildOutcome::Warn(message) => {
            println!("cargo:warning={}", one_line(&message));
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
            one_line(&hooks_dir.display().to_string())
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

    let existing = read_existing_hook(path);
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
        // SHARED with `cargo xtask setup`. Spelling these out here is how the two
        // installers came to tell different stories about the same file: the empty-
        // hook wording was fixed in `setup.rs` only, so `cargo build` still told a
        // zero-byte hook to "merge ours into yours".
        // ⚠ ONE LINE. Cargo cuts a `cargo:warning=` at the first newline and drops
        // the remainder without even echoing it — measured. Only the empty-hook arm
        // interpolates a path, and it does so mid-sentence, so a hooks directory with
        // a newline in a component would lose the actionable half of the advice.
        println!(
            "cargo:warning={}",
            one_line(&describe_untouchable(
                state,
                name,
                path,
                Attempted::Install { retry: "rebuild" }
            ))
        );
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
            one_line(&path.display().to_string())
        );
    }
}

/// Flatten a message so cargo cannot truncate it.
///
/// Every `cargo:warning=` here interpolates a path, and a path may legally contain a
/// newline. Cargo is line-based and silently discards everything after the first one,
/// so the half of the sentence that tells the developer what to DO is the half that
/// disappears.
fn one_line(message: &str) -> String {
    message.replace(['\n', '\r'], " ")
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
    // Shared with `cargo xtask setup`, so the two installers cannot drift on what
    // "git will run this" means — the drift that started this arc.
    if is_executable(path) {
        return;
    }

    if let Err(e) = make_executable(path) {
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
