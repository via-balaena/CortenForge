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

/// Pre-commit hook content. Single source: `xtask/hooks/pre-commit`.
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
    use std::process::Command;

    /// Run the pre-commit hook in a throwaway git repo with `staged` created and
    /// `git add -f`'d. Returns (exited_zero, combined_output).
    ///
    /// The hook is executed for real. A `contains("CF_ALLOW_MESH")` assertion on the
    /// hook TEXT proves nothing — the string also appears in the header comment and
    /// in the help line, so the entire enforcing block can be deleted and a text
    /// assertion still passes. That was measured, not imagined: it is exactly what
    /// the first version of this test did.
    fn run_hook(staged: &str, allow_mesh: bool) -> (bool, String) {
        // ⚠ Per-INVOCATION, not per-filename. Two tests both stage "part.stl", so a
        // name derived from the filename collided and they raced each other's
        // temp dir — cargo runs tests in parallel. It passed when run alone and
        // failed in the suite, which is the flake that reaches CI and not you.
        static N: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
        let uniq = N.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        let dir = std::env::temp_dir().join(format!(
            "cf-hook-{}-{uniq}-{}",
            std::process::id(),
            staged.replace(['/', '.'], "_")
        ));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).expect("temp dir");
        let git = |args: &[&str]| {
            Command::new("git")
                .args(args)
                .current_dir(&dir)
                .output()
                .expect("git")
        };
        git(&["init", "-q", "."]);
        git(&["config", "user.email", "t@t"]);
        git(&["config", "user.name", "t"]);
        // A real, already-formatted cargo project. Without it `cargo fmt --all
        // --check` fails on EVERY invocation, the hook exits non-zero whatever the
        // guard does, and no assertion on the exit code can mean anything. This is
        // what makes `a_clean_stage_passes` — and therefore the whole suite's
        // pass/fail discrimination — possible.
        std::fs::write(
            dir.join("Cargo.toml"),
            b"[package]\nname = \"p\"\nversion = \"0.0.0\"\nedition = \"2021\"\n",
        )
        .expect("Cargo.toml");
        std::fs::create_dir_all(dir.join("src")).expect("src");
        std::fs::write(dir.join("src/main.rs"), b"fn main() {}\n").expect("main.rs");
        std::fs::write(dir.join(staged), b"solid x\n").expect("fixture");
        git(&["add", "-f", staged]);

        let hook = dir.join("hook.sh");
        std::fs::write(&hook, PRE_COMMIT_HOOK).expect("hook");
        let mut cmd = Command::new("sh");
        cmd.arg(&hook).current_dir(&dir);
        // env_remove is not optional. Without it the child INHERITS an ambient
        // CF_ALLOW_MESH, and a developer who exports it — i.e. exactly the person
        // who works with meshes — sees the two blocking tests fail on their machine
        // and nobody else's. Measured: `CF_ALLOW_MESH=1 cargo test -p xtask` failed
        // 2 of 5 before this line existed.
        if allow_mesh {
            cmd.env("CF_ALLOW_MESH", "1");
        } else {
            cmd.env_remove("CF_ALLOW_MESH");
        }
        let out = cmd.output().expect("run hook");
        let combined = format!(
            "{}{}",
            String::from_utf8_lossy(&out.stdout),
            String::from_utf8_lossy(&out.stderr)
        );
        let _ = std::fs::remove_dir_all(&dir);
        (out.status.success(), combined)
    }

    /// A staged mesh must actually BLOCK the commit — the behaviour, not the prose.
    ///
    /// This repository is PUBLIC and the casting pipeline's inputs are anatomical
    /// scans of a real person. `.gitignore` cannot stop `git add -f`; this hook is
    /// the only thing that can, and #709 shipped it to one of the two installers
    /// with nothing checking that it worked.
    #[test]
    fn every_guarded_extension_blocks_the_commit() {
        // ALL FIVE. Testing only .stl left four of the guard's extensions with zero
        // coverage, so dropping one from the pathspec was a change no test opposed.
        for name in ["part.stl", "part.obj", "part.ply", "part.3mf", "part.mtl"] {
            let (ok, out) = run_hook(name, false);
            // `!ok` is load-bearing now that the scratch repo is a valid, formatted
            // cargo project — `a_clean_stage_passes` proves the hook CAN exit 0.
            assert!(!ok, "{name} did not block the commit; output:\n{out}");
            assert!(
                out.contains("Refusing to commit mesh/scan binaries"),
                "{name} did not trip the mesh guard; output:\n{out}"
            );
            // ⚠ NOT `assert!(!ok)`. That is VACUOUS here: the throwaway repo has no
            // Cargo.toml, so `cargo fmt --all --check` fails and the hook exits
            // non-zero whatever the guard does. Assert instead that it exited AT the
            // guard — the guard `exit 1`s immediately, so the formatting step must
            // never be reached.
            assert!(
                !out.contains("Checking formatting"),
                "{name} tripped the guard but did not exit there; output:\n{out}"
            );
        }
    }

    /// ...including UPPERCASE extensions. Git pathspecs are case-sensitive, so the
    /// original `'*.stl'` did not match `PART.STL` and a scanner emitting uppercase
    /// walked straight through the guard. Measured before the `:(icase)` fix.
    #[test]
    fn an_uppercase_mesh_extension_blocks_too() {
        // ALL FIVE uppercased. Testing only SCAN.STL left four `:(icase)` entries
        // uncovered — dropping `:(icase)` from `*.mtl` alone was a mutant the suite
        // survived, measured.
        for name in ["SCAN.STL", "SCAN.OBJ", "SCAN.PLY", "SCAN.3MF", "SCAN.MTL"] {
            let (ok, out) = run_hook(name, false);
            assert!(!ok, "{name} did not block the commit; output:\n{out}");
            assert!(
                out.contains("Refusing to commit mesh/scan binaries"),
                "{name} was NOT blocked — is :(icase) still on every entry?:\n{out}"
            );
        }
        let (ok, out) = run_hook("SCAN.STL", false);
        assert!(!ok, "SCAN.STL did not block the commit");
        assert!(
            out.contains("Refusing to commit mesh/scan binaries"),
            "SCAN.STL was NOT blocked — the pathspec is case-sensitive again:\n{out}"
        );
        assert!(
            !out.contains("Checking formatting"),
            "did not exit at the guard:\n{out}"
        );
    }

    /// The documented override must work, and must ANNOUNCE itself. Printing
    /// "no scan/mesh binaries staged" while letting meshes through — which the
    /// hook used to do — hides the one commit anyone would want a record of.
    #[test]
    fn the_override_lets_it_through_and_says_so() {
        let (_, out) = run_hook("part.stl", true);
        assert!(
            !out.contains("Refusing to commit mesh/scan binaries"),
            "CF_ALLOW_MESH=1 did not suppress the refusal; output:\n{out}"
        );
        assert!(
            out.contains("allowing mesh/scan binaries through"),
            "override was silent — it must leave a record; output:\n{out}"
        );
        assert!(
            !out.contains("No scan/mesh binaries staged"),
            "override claimed nothing was staged, which is false; output:\n{out}"
        );
    }

    /// A mesh already in history, MODIFIED, must still be blocked.
    ///
    /// The guard's `--diff-filter=ACMR` covers additions, copies, modifications and
    /// renames. Every other test here stages a brand-new file, i.e. only `A` — so
    /// narrowing the filter to `A` alone was a mutation the whole suite survived.
    /// Measured, then closed with this test.
    #[test]
    fn modifying_a_mesh_already_in_history_is_blocked_too() {
        let dir = std::env::temp_dir().join(format!("cf-hook-mod-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).expect("temp dir");
        let git = |args: &[&str]| {
            Command::new("git")
                .args(args)
                .current_dir(&dir)
                .output()
                .expect("git")
        };
        git(&["init", "-q", "."]);
        git(&["config", "user.email", "t@t"]);
        git(&["config", "user.name", "t"]);
        // Land it in history first — the guard is not what put it there.
        std::fs::write(dir.join("old.stl"), b"solid v1\n").expect("v1");
        git(&["add", "-f", "old.stl"]);
        git(&["commit", "-q", "-m", "seed", "--no-verify"]);
        // Now MODIFY it: diff-filter M, not A.
        std::fs::write(dir.join("old.stl"), b"solid v2 much bigger\n").expect("v2");
        git(&["add", "old.stl"]);

        let hook = dir.join("hook.sh");
        std::fs::write(&hook, PRE_COMMIT_HOOK).expect("hook");
        let out = Command::new("sh")
            .arg(&hook)
            .current_dir(&dir)
            .env_remove("CF_ALLOW_MESH")
            .output()
            .expect("run hook");
        let combined = format!(
            "{}{}",
            String::from_utf8_lossy(&out.stdout),
            String::from_utf8_lossy(&out.stderr)
        );
        let _ = std::fs::remove_dir_all(&dir);
        assert!(
            combined.contains("Refusing to commit mesh/scan binaries"),
            "a MODIFIED mesh was not blocked — is --diff-filter still ACMR?; output:\n{combined}"
        );
    }

    /// POSITIVE CONTROL: the hook must be able to SUCCEED.
    ///
    /// Without this the suite cannot tell "the guard blocked it" from "the hook
    /// always fails", and it very nearly could not: the scratch repo had no
    /// `Cargo.toml`, so `cargo fmt --all --check` failed on every run and the hook
    /// exited non-zero unconditionally. Every `assert!(!ok)` was vacuous. A gate
    /// that cannot pass proves as little as one that cannot fail.
    #[test]
    fn a_clean_stage_passes() {
        let (ok, out) = run_hook("readme.txt", false);
        assert!(ok, "hook rejected a clean stage; output:\n{out}");
        assert!(
            out.contains("No scan/mesh binaries staged"),
            "clean stage did not reach the guard's happy path; output:\n{out}"
        );
    }

    /// `build.rs::title_of` takes the marker from line 2, so line 2 must actually
    /// BE a title. It only requires that a second line exists — a hook whose line 2
    /// were `set -e` would build fine and yield the marker "set -e", after which the
    /// updater silently stops recognising its own hooks. That is this arc's original
    /// bug, reintroduced one level up, and it would be invisible: no error, hooks
    /// just quietly stop updating. Assert the shape the derivation depends on.
    #[test]
    fn line_two_of_each_hook_is_the_title_the_marker_is_derived_from() {
        for (name, hook) in [
            ("pre-commit", PRE_COMMIT_HOOK),
            ("commit-msg", COMMIT_MSG_HOOK),
        ] {
            let line2 = hook.lines().nth(1).unwrap_or_else(|| {
                panic!("{name} has no line 2; build.rs::title_of would panic the BUILD")
            });
            assert!(
                line2.starts_with("# CortenForge"),
                "{name} line 2 is {line2:?}, not a `# CortenForge ...` title. \
                 build.rs::title_of would derive that as the ownership marker and \
                 the updater would stop recognising its own hooks, silently."
            );
        }
    }

    /// Both hooks must be runnable `sh`, since git executes them directly.
    #[test]
    fn hooks_start_with_a_posix_sh_shebang() {
        assert!(PRE_COMMIT_HOOK.starts_with("#!/bin/sh\n"));
        assert!(COMMIT_MSG_HOOK.starts_with("#!/bin/sh\n"));
    }

    /// No CR anywhere: `include_str!` embeds bytes verbatim (unlike a Rust string
    /// literal, which rustc normalises), so a CRLF checkout would ship `#!/bin/sh\r`
    /// and every hook would fail to execute. `.gitattributes` pins these to LF.
    ///
    /// ⚠ This does NOT prove the pin exists: it reads the bytes as checked out, so
    /// on any LF checkout it passes whether or not `.gitattributes` is there. It
    /// catches a CR committed into the file, and it fails on a CRLF checkout —
    /// which is where it matters. The pin itself is guarded by review, not by this.
    #[test]
    fn hooks_contain_no_carriage_returns() {
        assert!(
            !PRE_COMMIT_HOOK.contains('\r'),
            "pre-commit has CR — check .gitattributes"
        );
        assert!(
            !COMMIT_MSG_HOOK.contains('\r'),
            "commit-msg has CR — check .gitattributes"
        );
    }
}
