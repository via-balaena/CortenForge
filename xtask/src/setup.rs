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

use anyhow::Result;
use owo_colors::OwoColorize;
use std::fs;
use std::path::{Path, PathBuf};

// The hook texts and — critically — the filename each one is installed under come
// from `hook_install`, which `build.rs` also uses. Declaring them again here is what
// made the two installers separate code: `setup.rs` got a pairing test and `build.rs`
// kept its own uncovered copy, where crossing the consts still shipped green.
use crate::hook_install::HOOKS;

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
    //
    // ⚠ NOT `?`, for the same reason the install loop accumulates rather than
    // aborting. `?` here made ONE unrepairable hook cancel `verify_tools` and every
    // line of onboarding below — and that became reachable the moment the executable
    // bit stopped being repaired through a symlink, which turned a silent (wrong)
    // success into a legitimate failure. Report both, then fail.
    let hooks = install_git_hooks();

    // Step 2: Verify required tools
    let tools = verify_tools();

    // ⚠ Fail AFTER both have run and printed. Their messages are the actionable part;
    // the exit status only decides whether a script notices.
    //
    // ⚠ NOT GATED, and saying so is better than implying otherwise: `run()` does real
    // filesystem and process work, so the ordering is only observable end to end.
    // What IS gated is the contract underneath it — `install_git_hooks_into` returns
    // an error rather than aborting mid-loop, in
    // `a_hook_whose_bit_cannot_be_repaired_is_a_failure_not_a_success`.
    hooks?;
    tools?;

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

/// Where git will actually look for hooks — asked, never assumed.
///
/// Joining `.git/hooks` onto the workspace root is silently wrong under
/// `core.hooksPath`, in a linked worktree (whose `.git` is a FILE, so that path does
/// not exist at all), and with `--separate-git-dir`. It also resolved against the
/// PROCESS working directory, so running from a subdirectory made `uninstall` report
/// "No hooks directory found" and return `Ok(())` — a no-op reported as success.
///
/// Refuses a hooks directory outside this repository: a `core.hooksPath` pointing
/// out there MAY be shared with every other repo on the machine, and installing or
/// deleting there is not ours to do either way. ⚠ "May" is load-bearing — a
/// repo-local `core.hooksPath` out of the tree lands here too, so the refusal names
/// the location, never the owner. See [`crate::hook_install::HooksDir`].
///
/// # Errors
/// If git's hooks directory cannot be determined, or is not inside this checkout.
fn git_hooks_dir(attempted: crate::hook_install::Attempted) -> Result<PathBuf> {
    let sh = xshell::Shell::new()?;
    let root = PathBuf::from(crate::grade::find_workspace_root(&sh)?);
    hooks_dir_from(
        crate::hook_install::resolve_hooks_dir(&root),
        &root,
        std::env::var_os("GIT_DIR").is_some() || std::env::var_os("GIT_WORK_TREE").is_some(),
        attempted,
    )
}

/// Turn a resolution into a directory to write, or into the reason we will not.
///
/// Split from [`git_hooks_dir`] because THE REFUSALS ARE THE POINT and the version
/// that resolves the directory itself cannot be tested — it reads the process's cwd.
/// While the two were one function, mutating either refusal into `Ok(dir)` left the
/// whole suite green while `cargo xtask setup` wrote CortenForge's hooks into a
/// developer's global hooks directory, or into an unrelated repository.
fn hooks_dir_from(
    resolved: Option<crate::hook_install::HooksDir>,
    root: &Path,
    git_env_set: bool,
    attempted: crate::hook_install::Attempted,
) -> Result<PathBuf> {
    match resolved {
        Some(crate::hook_install::HooksDir::Repo(dir)) => Ok(dir),
        Some(crate::hook_install::HooksDir::OtherRepo(dir)) => anyhow::bail!(
            "git resolves this directory to a different repository — its hooks live \
             at {}. This looks like a copy of CortenForge nested inside another \
             checkout; refusing to install hooks into a repository that is not this \
             one.",
            dir.display()
        ),
        // ⚠ Says WHERE, not WHOSE. This fires for any hooks directory outside the
        // repository, which a repo-LOCAL `core.hooksPath` can also name — the old
        // wording told such a developer their setting was "shared with your other
        // repos", which was simply false. Classification is by location; so is this.
        Some(crate::hook_install::HooksDir::Shared(dir)) => anyhow::bail!(
            "git is configured to read hooks from {}, which is outside this \
             repository. Refusing to touch it — a directory out there may be shared \
             with your other repos. Merge xtask/hooks/* into it yourself, or point \
             core.hooksPath somewhere inside this checkout.",
            dir.display()
        ),
        // ⚠⚠ NOT A PERMISSION REFUSAL. This directory IS ours to write — and we
        // decline anyway, because git cannot exec a hook from the form it is spelled
        // in, so installing would make git refuse every commit in the checkout.
        // Both installers must say so, or `cargo build` and `cargo xtask setup`
        // disagree about a repository neither of them can help: the drift this whole
        // arc exists to end.
        // ⚠⚠ REMOVING FROM HERE IS EXACTLY WHAT THIS DEVELOPER NEEDS. Containment
        // has already passed, so the directory is inside the checkout; and the
        // population with an unrunnable `core.hooksPath` is precisely the one the
        // PREVIOUS code installed into — every commit in those checkouts is being
        // refused right now, by a hook we put there. Refusing to uninstall told them
        // to fix the setting and "re-run" a command they had asked to undo, which is
        // the mistake `describe_untouchable` already names: never answer an uninstall
        // with install advice.
        Some(crate::hook_install::HooksDir::GitCannotRun(dir))
            if attempted == crate::hook_install::Attempted::Uninstall =>
        {
            Ok(dir)
        }
        Some(crate::hook_install::HooksDir::GitCannotRun(dir)) => anyhow::bail!(
            "git resolves hooks to {}, but the path git would actually EXEC is not \
             the file we would write. core.hooksPath is a bare `.`/`./` (git \
             normalises it away and searches PATH), starts with `-` or `+` (the \
             hook's interpreter reads it as options), or is empty (git looks at the \
             FILESYSTEM root). Installing would either make git refuse EVERY commit \
             in this checkout or leave the hook silently unread. Naming that same \
             directory as an ABSOLUTE path works; so does a subdirectory such as \
             .githooks. Then re-run.",
            dir.display()
        ),
        // ⚠ NAME THE CAUSE WHEN WE KNOW IT. `build.rs` explained this case and
        // `setup` did not — the explanation lived in `build_outcome`, which is
        // build-script-only, so the developer got "could not determine" from one
        // installer and a usable sentence from the other about the same checkout.
        None if git_env_set => anyhow::bail!(
            "GIT_DIR/GIT_WORK_TREE are set in this environment. They are ignored when \
             locating hooks, because they let the environment install into a \
             different repository — so {} could not be resolved. Run this from the \
             working tree without them.",
            root.display()
        ),
        None => anyhow::bail!(
            "Could not determine git's hooks directory for {}.",
            root.display()
        ),
    }
}

/// Install git hooks
fn install_git_hooks() -> Result<()> {
    println!("{}", "→ Installing git hooks...".bright_blue());

    let hooks_dir = git_hooks_dir(crate::hook_install::Attempted::Install { retry: "re-run" })?;

    if !hooks_dir.exists() {
        // Worktrees and core.hooksPath now resolve correctly, so reaching here means
        // the directory genuinely is not there — not that we guessed the path wrong.
        anyhow::bail!(
            "git reports its hooks directory as {}, but that directory does not \
             exist. Create it (or unset core.hooksPath) and run setup again.",
            hooks_dir.display()
        );
    }

    install_git_hooks_into(&hooks_dir)
}

/// Write both hooks into `hooks_dir`.
///
/// Split out from [`install_git_hooks`] so a test can reach it: the version that
/// resolves the directory itself can only be exercised by changing the process's
/// cwd, which is global and races other tests. A mutation survey found the pairing
/// — which text goes to which filename — was covered by NOTHING, so crossing the two
/// shipped green. That is the same class as this arc's original bug: installer wiring
/// with no gate on it. The pairing now lives once, in `hook_install::HOOKS`.
fn install_git_hooks_into(hooks_dir: &Path) -> Result<()> {
    install_git_hooks_into_with(hooks_dir, &crate::hook_install::make_executable)
}

/// As above, with the executable-bit repair injected so a test can make it FAIL.
///
/// ⚠⚠ THE FAILURE CONTRACT HAD NO GATE AT ALL. Every test called this through
/// `.expect("install")`, so nothing ever asserted it returns `Err` — a mutation
/// survey deleted the `failures += 1`, and then the whole `if failures > 0 { bail! }`,
/// and both shipped green. The consequence is the exact false success this arc
/// exists to remove: `cargo xtask setup` exits 0 reporting success while a hook sits
/// at mode 0644, which git ignores WITHOUT A WORD. `uninstall_hooks_from` has the
/// symmetric test; install did not.
///
/// ⚠ A seam rather than a real refusal, and the ORIGINAL reason for it has since
/// been falsified: it said making `chmod` fail needs `chflags uchg` (macOS) or root
/// (Linux), which stopped being true when `make_executable` began refusing a
/// SYMLINK — portable, no privileges, and `repairing_the_bit_never_chmods_through_a_symlink`
/// does exactly that in this same suite.
///
/// ★ The seam still earns its place, for a better reason: it isolates the COUNTING
/// contract from the symlink policy. A test that made chmod fail by handing it a
/// symlink would fail for a second reason the moment that policy changed, and would
/// stop being a test of "a failure is counted, and the loop continues".
fn install_git_hooks_into_with(
    hooks_dir: &Path,
    #[cfg(unix)] repair_bit: &dyn Fn(&Path) -> std::io::Result<()>,
    #[cfg(not(unix))] _repair_bit: &dyn Fn(&Path) -> std::io::Result<()>,
) -> Result<()> {
    let mut failures = 0usize;
    // NAME IT. From a linked worktree this is the SHARED common directory, so this
    // command rewrites the hooks the main checkout uses; that blast radius must be
    // visible rather than inferred.
    println!("  hooks directory: {}", hooks_dir.display());
    for (name, content) in HOOKS {
        let path = hooks_dir.join(name);
        let marker = crate::hook_install::title_of(content);
        let existing = crate::hook_install::read_existing_hook(&path);
        let state = crate::hook_install::classify(
            existing.as_ref().map(|r| r.as_deref().map_err(|_| ())),
            content,
            marker,
        );

        // ⚠⚠ THE OWNERSHIP CHECK, which this command did not make. `build.rs` has
        // always classified first and left a foreign hook alone with a warning;
        // `setup` overwrote it and printed `✓ Installed`, indistinguishable from
        // the ordinary case. Two installers, one repository, opposite verdicts on
        // the same file — and the untested one was the one that destroyed data.
        //
        // Resolving the directory properly is what forced the issue: `setup` run
        // inside a linked worktree now reaches the MAIN checkout's hooks, so the
        // clobber stopped being local to the developer who typed the command.
        // Measured: it replaced a foreign `pre-commit` in the main checkout and
        // said nothing. An explicit command is not a licence to delete a file
        // nobody mentioned.
        if !state.is_ours_to_manage() {
            println!(
                "  {}",
                crate::hook_install::describe_untouchable(
                    state,
                    name,
                    &path,
                    crate::hook_install::Attempted::Install { retry: "re-run" },
                )
            );
            continue;
        }

        if state.should_replace() {
            // ⚠ NOT `?`, for the same reason the chmod branch below is not — and this
            // one was left with `?` in the very commit that fixed that one, 26 lines
            // apart in the same loop. Measured: with a stale-but-ours `pre-commit`
            // made immutable, `setup` aborted before `commit-msg` was even
            // classified — and before `verify_tools()` — while `build.rs`, facing the
            // identical directory, warned and installed `commit-msg` anyway. Two
            // installers, one directory, different filesystem outcomes: the drift
            // this arc exists to end.
            //
            // Atomic: git may be RUNNING this exact file. See `write_hook_file`.
            match crate::hook_install::write_hook_file(&path, content) {
                Ok(()) => println!("  ✓ Installed {name} hook"),
                Err(e) => {
                    println!("  ⚠ Failed to write the {name} hook: {e}");
                    failures += 1;
                }
            }
            continue;
        }

        // ⚠⚠ OURS AND CURRENT — DO NOT REWRITE IT. An earlier version did, reasoning
        // that laying the file down again was the cheapest way to guarantee the
        // executable bit. It is not equivalent: `write_hook_file` RENAMES over the
        // path, so a `.git/hooks/pre-commit` symlinked to `xtask/hooks/pre-commit`
        // became a regular file and the developer's link was gone — announced as
        // "already current". And it put the two installers back into disagreement on
        // `OursCurrent`, which is the asymmetry this all began with.
        //
        // ⚠ Be precise about what this did NOT fix. An earlier version of this note
        // also claimed it stopped tracked `.githooks/*` flipping 644 → 755. It does
        // not: the chmod below still does that, and MUST — git ignores a hook without
        // the bit. What went away is the gratuitous rewrite (the inode churned on
        // every run) and the destroyed symlink. Measured.
        //
        // Repair the bit IN PLACE instead, exactly as `build.rs` does.
        // ⚠ NOT `?`, for the same reason `uninstall_hooks_from` documents below: an
        // abort here leaves the REMAINING hooks unexamined while the command reports
        // an error, so a chmod refused on `pre-commit` (an immutable flag, a hook
        // owned by another user after a `sudo` build) silently decides the fate of
        // `commit-msg` too. Try every hook, then report.
        #[cfg(unix)]
        if !crate::hook_install::is_executable(&path) {
            match repair_bit(&path) {
                Ok(()) => println!(
                    "  ✓ Repaired the {name} hook's executable bit — git had been \
                     ignoring it, so its checks were not running."
                ),
                Err(e) => {
                    println!(
                        "  ⚠ The {name} hook is not executable and could not be made \
                         one, so git will ignore it and its checks will not run: {e}"
                    );
                    failures += 1;
                }
            }
            continue;
        }
        println!("  ✓ {name} hook already current");
    }

    if failures > 0 {
        anyhow::bail!("{failures} hook(s) could not be installed; see above.");
    }
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

    let hooks_dir = git_hooks_dir(crate::hook_install::Attempted::Uninstall)?;

    if !hooks_dir.exists() {
        println!("  No hooks directory found at {}.", hooks_dir.display());
        return Ok(());
    }

    // NAME IT, for the same reason as install: run from a linked worktree this is
    // the SHARED common directory, so removing hooks here disarms the guard for the
    // main checkout and every other worktree.
    println!("  hooks directory: {}", hooks_dir.display());
    uninstall_hooks_from(&hooks_dir)?;

    println!("{}", "Hooks removed.".bright_green());
    Ok(())
}

/// Remove OUR hooks from `hooks_dir`, and only ours.
///
/// Split out for the same reason as [`install_git_hooks_into`]: while the removal
/// lived inside the command that resolves its own directory, no test could reach it,
/// and it deleted files by name with no ownership check at all.
fn uninstall_hooks_from(hooks_dir: &Path) -> Result<()> {
    let mut failures = 0usize;
    // The same single source both installers read. Hardcoding the names here is
    // how a third hook would get installed by both of them and left behind by this
    // one — a live hook with no source, which is the drift shape (#709) this branch
    // exists to repair.
    for (name, content) in HOOKS {
        let path = hooks_dir.join(name);
        let marker = crate::hook_install::title_of(content);
        let existing = crate::hook_install::read_existing_hook(&path);
        let Some(existing) = existing else {
            continue;
        };
        let state = crate::hook_install::classify(
            Some(existing.as_deref().map_err(|_| ())),
            content,
            marker,
        );

        // ⚠⚠ DELETE ONLY WHAT WE INSTALLED. This removed `<dir>/<name>` by NAME,
        // so it destroyed a developer's own `pre-commit` — the file `build.rs`
        // deliberately preserves — and, once `core.hooksPath` inside the working
        // tree became a supported layout, it deleted TRACKED files out of the
        // checkout. Uninstalling CortenForge's hooks is not a licence to remove
        // anybody else's; the two halves of the installer now agree on ownership.
        if !state.is_ours_to_manage() {
            // ⚠ Uninstall wording, not install wording. The shared helper used to
            // tell an `uninstall` run to "merge xtask/hooks/<name> into yours … and
            // re-run" — advising an install in the middle of a removal.
            println!(
                "  {}",
                crate::hook_install::describe_untouchable(
                    state,
                    name,
                    &path,
                    crate::hook_install::Attempted::Uninstall,
                )
            );
            continue;
        }
        // ⚠ NOT `?`. Failing on `pre-commit` used to abort the loop, leaving
        // `commit-msg` installed while the command reported an error — a half-done
        // uninstall that neither the user nor the next run can reason about. Try
        // every hook, then report.
        if let Err(e) = fs::remove_file(&path) {
            println!("  ⚠ Could not remove the {name} hook: {e}");
            failures += 1;
            continue;
        }
        println!("  ✓ Removed {name} hook");
    }

    if failures > 0 {
        anyhow::bail!("{failures} hook(s) could not be removed; see above.");
    }
    Ok(())
}

#[cfg(test)]
mod hook_tests {
    use crate::hook_install::{COMMIT_MSG_HOOK, HOOKS, PRE_COMMIT_HOOK};
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
        // ⚠ PREMISE CHECK. Every call below is setup this test's CONCLUSION rests
        // on. A dropped exit status does not fail the test — it silently turns it
        // into a different, still-green one. If `git add` fails, nothing is staged,
        // the hook reports a clean stage and exits 0, and `a_clean_stage_passes` —
        // the positive control the whole suite's discrimination rests on — passes
        // having proved nothing about a stage.
        let git = |args: &[&str]| {
            let out = Command::new("git")
                .args(args)
                .current_dir(&dir)
                .output()
                .expect("git");
            assert!(
                out.status.success(),
                "scratch-repo setup failed at `git {}`:\n{}{}",
                args.join(" "),
                String::from_utf8_lossy(&out.stdout),
                String::from_utf8_lossy(&out.stderr)
            );
            out
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

    /// Run the hook with a real nested crate staged, and capture the argv it hands
    /// `cargo clippy`. Returns (exited_zero, hook output, cargo argv lines).
    ///
    /// ⚠⚠ THIS BLOCK WAS REACHED BY NOTHING. A mutation survey found that every
    /// staged shape the suite used — a mesh, a `.txt`, the fixture's own root-level
    /// `Cargo.toml` — exits before the crate walk, so four separate mutations of it
    /// all shipped green: dropping `tr -d '\r'` (no Rust commit passes on Windows),
    /// reverting to `for file in $staged_rs_files` (a crate under `my crate/`
    /// silently unlinted), dropping `core.quotePath=false` (same for `café/`), and
    /// re-adding `2>/dev/null` (a failure with no diagnostic). Untested installer
    /// wiring is the shape of #709 itself.
    ///
    /// `cargo` is stubbed on the PATH of the hook's CHILD PROCESS ONLY — per-child,
    /// so unlike a `set_var` it cannot race the parallel suite. It logs its argv and
    /// exits 0, which also keeps this fast: a real clippy per case would not.
    fn run_hook_with_crate(dir_name: &str, crlf_manifest: bool) -> (bool, String, Vec<String>) {
        run_hook_with_crate_full(dir_name, crlf_manifest, None, None)
    }

    /// As above, plus one extra file staged at the repo ROOT — the shape that lets a
    /// path beginning with `-` reach `dirname`/`grep` as an OPTION.
    fn run_hook_with_crate_and_extra(
        dir_name: &str,
        extra_staged: &str,
    ) -> (bool, String, Vec<String>) {
        run_hook_with_crate_full(dir_name, false, None, Some(extra_staged))
    }

    /// As above, but `cargo_failure` makes the stub fail one cargo SUBCOMMAND with
    /// that text on STDERR — which is where cargo puts every diagnostic it prints.
    fn run_hook_with_crate_and_cargo(
        dir_name: &str,
        crlf_manifest: bool,
        cargo_failure: Option<(&str, &str)>,
    ) -> (bool, String, Vec<String>) {
        run_hook_with_crate_full(dir_name, crlf_manifest, cargo_failure, None)
    }

    fn run_hook_with_crate_full(
        dir_name: &str,
        crlf_manifest: bool,
        cargo_failure: Option<(&str, &str)>,
        extra_staged: Option<&str>,
    ) -> (bool, String, Vec<String>) {
        static N: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
        let uniq = N.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        let dir = std::env::temp_dir().join(format!("cf-crate-{}-{uniq}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).expect("temp dir");
        let git = |args: &[&str]| {
            let out = Command::new("git")
                .args(args)
                .current_dir(&dir)
                .output()
                .expect("git");
            assert!(
                out.status.success(),
                "scratch-repo setup failed at `git {}`:\n{}{}",
                args.join(" "),
                String::from_utf8_lossy(&out.stdout),
                String::from_utf8_lossy(&out.stderr)
            );
        };
        git(&["init", "-q", "."]);
        git(&["config", "user.email", "t@t"]);
        git(&["config", "user.name", "t"]);

        let crate_dir = dir.join(dir_name);
        std::fs::create_dir_all(crate_dir.join("src")).expect("crate src");
        let manifest =
            "[package]\nname = \"demo-crate\"\nversion = \"0.0.0\"\nedition = \"2021\"\n";
        let manifest = if crlf_manifest {
            manifest.replace('\n', "\r\n")
        } else {
            manifest.to_string()
        };
        std::fs::write(crate_dir.join("Cargo.toml"), manifest).expect("manifest");
        std::fs::write(crate_dir.join("src/lib.rs"), b"pub fn a() {}\n").expect("lib.rs");
        if let Some(extra) = extra_staged {
            std::fs::write(dir.join(extra), b"fn x() {}\n").expect("extra");
        }
        git(&["add", "-A"]);

        // The stub. `"$@"` is logged verbatim so a mangled crate name is visible.
        let bin = dir.join("stubbin");
        std::fs::create_dir_all(&bin).expect("bin");
        let log = dir.join("cargo.log");
        let fail_clause = match cargo_failure {
            Some((subcommand, msg)) => {
                format!("case \"$1\" in {subcommand}) printf '%s\\n' '{msg}' >&2; exit 1;; esac\n")
            }
            None => String::new(),
        };
        std::fs::write(
            bin.join("cargo"),
            format!(
                "#!/bin/sh\nprintf '%s\\n' \"$*\" >> '{}'\n{fail_clause}exit 0\n",
                log.display()
            ),
        )
        .expect("stub");
        #[cfg(unix)]
        crate::hook_install::make_executable(&bin.join("cargo")).expect("chmod stub");

        let hook = dir.join("hook.sh");
        std::fs::write(&hook, PRE_COMMIT_HOOK).expect("hook");
        let path = format!(
            "{}:{}",
            bin.display(),
            std::env::var("PATH").unwrap_or_default()
        );
        let out = Command::new("sh")
            .arg(&hook)
            .current_dir(&dir)
            .env("PATH", path)
            .env_remove("CF_ALLOW_MESH")
            .output()
            .expect("run hook");
        let combined = format!(
            "{}{}",
            String::from_utf8_lossy(&out.stdout),
            String::from_utf8_lossy(&out.stderr)
        );
        let argv: Vec<String> = std::fs::read_to_string(&log)
            .unwrap_or_default()
            .lines()
            .map(str::to_owned)
            .collect();
        let _ = std::fs::remove_dir_all(&dir);
        (out.status.success(), combined, argv)
    }

    /// The crate walk must name every staged crate, whatever its path looks like.
    #[test]
    fn the_lint_step_finds_crates_through_awkward_paths_and_manifests() {
        // POSITIVE CONTROL FIRST. If the ordinary case did not reach clippy, every
        // assertion below would hold for a hook that simply never lints anything.
        let (ok, out, argv) = run_hook_with_crate("demo", false);
        assert!(ok, "the ordinary case must pass;\n{out}");
        assert!(
            argv.iter()
                .any(|a| a.contains("clippy") && a.contains("-p demo-crate")),
            "the crate walk never reached clippy, so nothing below discriminates; \
             argv = {argv:?}\n{out}"
        );

        for (label, dir_name, crlf) in [
            // A space: the default IFS split this into two words, so it matched no
            // Cargo.toml and the crate was dropped with a reassuring message.
            ("a space in the path", "my crate", false),
            // Non-ASCII: git C-quotes it to `"caf\303\251/..."` unless quotePath is
            // off — a name no file has, so the walk finds nothing.
            ("a non-ASCII path", "café", false),
            // A glob character, which must not be expanded on the way through.
            ("a glob character", "g[1]", false),
            // CRLF manifest: the captured name ended in a CR, and `cargo -p 'x<CR>'`
            // is rejected — so NO Rust commit passed the hook on a Windows checkout.
            ("a CRLF manifest", "demo", true),
            // ★ A crate at the REPOSITORY ROOT. The walk used to stop before `.`,
            // so any repo whose root is a crate had that crate silently unlinted —
            // and the message blamed a "virtual manifest", which was the wrong
            // cause: measured, a root manifest WITH a `[package]` behaved the same.
            ("a crate at the repository root", ".", false),
            // ★ A crate DIRECTORY starting with `-`. `dirname --` was not enough:
            // the very next line hands `"$dir/Cargo.toml"` to `grep`, which parses
            // it as options (`invalid option -- g`, exit 2) and drops the crate.
            // `--` is not portable there — BSD sed takes it as a FILENAME — so the
            // operand is prefixed `./` instead.
            ("a dash-prefixed crate directory", "-dash", false),
        ] {
            let (ok, out, argv) = run_hook_with_crate(dir_name, crlf);
            assert!(ok, "{label}: the hook failed;\n{out}");
            let clippy = argv
                .iter()
                .find(|a| a.contains("clippy"))
                .unwrap_or_else(|| panic!("{label}: clippy was never invoked — the crate was silently dropped from the lint set;\n{out}"));
            assert!(
                clippy.contains("-p demo-crate"),
                "{label}: clippy ran without the staged crate: {clippy:?}\n{out}"
            );
            assert!(
                !clippy.contains('\r'),
                "{label}: the crate name carries a carriage return, which cargo \
                 rejects as an invalid character in a package name: {clippy:?}"
            );
            assert!(
                !out.contains("don't belong to a workspace crate"),
                "{label}: the hook claimed the staged files belong to no crate;\n{out}"
            );
        }
    }

    /// An up-to-date hook is REPAIRED IN PLACE, never laid down again.
    ///
    /// ⚠⚠ These look equivalent and are not. `write_hook_file` renames over the
    /// path, so rewriting an already-current hook replaces a SYMLINK with a regular
    /// file — a developer whose `.git/hooks/pre-commit` links to `xtask/hooks/`
    /// loses the link silently, told only "already current". And it put the two
    /// installers back into disagreement on `OursCurrent`, which is the asymmetry the
    /// whole arc exists to end. `build.rs` never did this.
    ///
    /// ⚠ What the in-place repair does NOT change: a tracked hook at 644 still goes
    /// to 755 and still shows as modified. It has to — git ignores a hook without the
    /// bit. The inode assertion below is what separates the two behaviours.
    #[test]
    #[cfg(unix)]
    fn an_up_to_date_hook_is_repaired_in_place_and_a_symlinked_one_survives() {
        use std::os::unix::fs::{MetadataExt, PermissionsExt};

        let base = std::env::temp_dir().join(format!("cf-inplace-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&base);
        let hooks = base.join("hooks");
        let source = base.join("source");
        std::fs::create_dir_all(&hooks).expect("temp dir");
        std::fs::create_dir_all(&source).expect("temp dir");

        // pre-commit: OUR CURRENT TEXT, reached through a symlink — the shape a
        // developer creates to keep the hook tracking xtask/hooks/.
        let target = source.join("pre-commit");
        std::fs::write(&target, PRE_COMMIT_HOOK).expect("seed target");
        crate::hook_install::make_executable(&target).expect("chmod target");
        std::os::unix::fs::symlink(&target, hooks.join("pre-commit")).expect("symlink");

        // commit-msg: our current text, but mode 644 — git ignores it silently, so
        // the bit MUST be repaired. This is the positive control: an installer that
        // simply did nothing for `OursCurrent` would pass the symlink assertion.
        let cm = hooks.join("commit-msg");
        std::fs::write(&cm, COMMIT_MSG_HOOK).expect("seed commit-msg");
        std::fs::set_permissions(&cm, std::fs::Permissions::from_mode(0o644)).expect("chmod");
        let cm_ino_before = std::fs::metadata(&cm).expect("meta").ino();

        super::install_git_hooks_into(&hooks).expect("install");

        let still_symlink = hooks.join("pre-commit").is_symlink();
        let target_intact = std::fs::read_to_string(&target).expect("read target");
        let cm_meta = std::fs::metadata(&cm).expect("meta");
        let cm_mode = cm_meta.permissions().mode();
        let cm_ino_after = cm_meta.ino();
        let _ = std::fs::remove_dir_all(&base);

        assert!(
            still_symlink,
            "installing over an up-to-date hook replaced the developer's symlink \
             with a regular file — silently, and reported as 'already current'"
        );
        assert_eq!(
            target_intact, PRE_COMMIT_HOOK,
            "the symlink's target must be left exactly as it was"
        );
        assert!(
            cm_mode & 0o111 != 0,
            "POSITIVE CONTROL: an up-to-date hook at mode {cm_mode:o} is ignored by \
             git without a word — the bit has to be repaired, or doing nothing at \
             all would satisfy the assertion above"
        );
        assert_eq!(
            cm_ino_before, cm_ino_after,
            "the executable bit was repaired by REWRITING the file, not by chmod — \
             which is exactly what destroys a symlinked hook"
        );
    }

    /// Declining to install must say something the developer can act on.
    #[test]
    fn an_empty_hook_is_named_as_empty_rather_than_as_somebody_elses() {
        use crate::hook_install::HookState;

        let dir = std::env::temp_dir().join(format!("cf-empty-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).expect("temp dir");
        let empty = dir.join("pre-commit");
        let theirs = dir.join("commit-msg");
        std::fs::write(&empty, b"").expect("seed empty");
        std::fs::write(&theirs, b"#!/bin/sh\n# husky\n").expect("seed foreign");

        // A zero-byte file carries no marker, so it classifies Foreign — correctly.
        // But "merge xtask/hooks/pre-commit into yours" is nonsense advice for it,
        // and before the ownership check `setup` was the way to repair one. The
        // classification is right; the SENTENCE was the thing that had to change.
        use crate::hook_install::{describe_untouchable, Attempted};
        let installing = Attempted::Install { retry: "re-run" };
        let empty_msg = describe_untouchable(HookState::Foreign, "pre-commit", &empty, installing);
        let foreign_msg =
            describe_untouchable(HookState::Foreign, "commit-msg", &theirs, installing);
        let unreadable_msg =
            describe_untouchable(HookState::Unreadable, "pre-commit", &empty, installing);
        // ⚠ UNINSTALL MUST NOT ADVISE AN INSTALL. This helper began as install-only
        // wording that `uninstall` then borrowed, so removing hooks told the
        // developer to "merge xtask/hooks/pre-commit into yours … and re-run".
        let removing_msg = describe_untouchable(
            HookState::Foreign,
            "pre-commit",
            &theirs,
            Attempted::Uninstall,
        );
        let _ = std::fs::remove_dir_all(&dir);

        assert!(
            empty_msg.contains("EMPTY") && empty_msg.contains("Delete"),
            "an empty hook must be named, with the one action that works: {empty_msg}"
        );
        assert!(
            !empty_msg.contains("Merge"),
            "there is nothing to merge out of a zero-byte file: {empty_msg}"
        );
        assert!(
            foreign_msg.contains("Merge"),
            "POSITIVE CONTROL: a real foreign hook still gets the merge advice: \
             {foreign_msg}"
        );
        assert!(
            !removing_msg.contains("Merge") && !removing_msg.contains("re-run"),
            "uninstall told the developer to install the hook they asked to remove: \
             {removing_msg}"
        );
        assert!(
            unreadable_msg.contains("Could not read"),
            "unreadable must not be reported as somebody else's hook — a chmod-000 \
             hook that IS ours landed here: {unreadable_msg}"
        );
    }

    /// One hook that cannot be removed must not decide the fate of the other.
    ///
    /// ⚠ `uninstall_hooks_from` used `?`, so a failure on `pre-commit` aborted the
    /// loop and left `commit-msg` installed while the command reported an error — a
    /// half-done uninstall neither the developer nor the next run can reason about.
    /// The comment documenting that rule was there; the mutation that removed it
    /// shipped green, because nothing ever made a removal fail.
    #[test]
    #[cfg(unix)]
    fn a_hook_that_cannot_be_removed_does_not_strand_the_other_one() {
        use std::os::unix::fs::PermissionsExt;

        let dir = std::env::temp_dir().join(format!("cf-stuck-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).expect("temp dir");
        for (name, content) in HOOKS {
            std::fs::write(dir.join(name), content).expect("seed");
        }
        // A read-only DIRECTORY refuses unlink for everything inside it, with no
        // platform-specific flags. Both hooks are ours, so both would otherwise go.
        std::fs::set_permissions(&dir, std::fs::Permissions::from_mode(0o555)).expect("chmod");

        // ⚠ PRECONDITION, asserted rather than assumed: this does not hold as root,
        // and a test that silently stops refusing removals would pass having proved
        // nothing. Sampled before the call under test.
        let removal_is_refused = std::fs::remove_file(dir.join("pre-commit")).is_err();

        let result = super::uninstall_hooks_from(&dir);

        let _ = std::fs::set_permissions(&dir, std::fs::Permissions::from_mode(0o755));
        let _ = std::fs::remove_dir_all(&dir);

        assert!(
            removal_is_refused,
            "PRECONDITION BROKEN: removal succeeded in a read-only directory (running \
             as root?), so this test cannot show that the loop keeps going"
        );
        let err = result.expect_err("a refused removal must be reported");
        assert!(
            err.to_string().contains("2 hook(s)"),
            "`?` aborted the loop, so only the first hook was ever attempted: {err}"
        );
    }

    /// A staged path that git had to QUOTE takes its crate out of the lint set, and
    /// the hook has to say so.
    ///
    /// ⚠⚠ The previous attempt at this compared a NUL-delimited count against a line
    /// count and could NEVER fire — git puts a quoted path on exactly ONE line, so
    /// one path is always one line and one NUL. It shipped with no test, which is
    /// precisely why a check that cannot fail passed review. This asserts BOTH
    /// directions: the warning appears for a control-character path, and does not
    /// appear for ordinary ones.
    #[test]
    fn a_path_git_had_to_quote_is_announced_rather_than_dropped_in_silence() {
        let (ok, out, argv) = run_hook_with_crate_and_extra("demo", "we\nird.rs");
        assert!(ok, "the hook must still pass;\n{out}");
        assert!(
            argv.iter().any(|a| a.contains("clippy")),
            "PREMISE: clippy was never reached, so this proves nothing;\n{out}"
        );
        assert!(
            out.contains("has to quote"),
            "a path git had to quote was dropped from the lint set in silence — the \
             same reassuring output as having nothing to lint;\n{out}"
        );

        // NEGATIVE CONTROL: ordinary paths must NOT trip it, or the warning becomes
        // noise on every commit and stops meaning anything.
        let (_, plain, _) = run_hook_with_crate("demo", false);
        assert!(
            !plain.contains("has to quote"),
            "the warning fires for ordinary paths, so it says nothing;\n{plain}"
        );
    }

    /// A dash-prefixed staged FILE must not take every crate down with it.
    #[test]
    fn a_dash_prefixed_staged_path_does_not_silently_empty_the_lint_set() {
        let (ok, out, argv) = run_hook_with_crate_and_extra("demo", "-dash.rs");
        assert!(ok, "the hook must still pass;\n{out}");
        let clippy = argv
            .iter()
            .find(|a| a.contains("clippy"))
            .unwrap_or_else(|| {
                panic!(
                    "`dirname` parsed the staged `-dash.rs` as OPTIONS and `set -e` killed \
                 the crate walk mid-stream — every crate lost, announced as \
                 \"don't belong to a workspace crate\", exit 0;\n{out}"
                )
            });
        assert!(
            clippy.contains("-p demo-crate"),
            "the staged crate was dropped from the lint set: {clippy:?}\n{out}"
        );
    }

    /// A hook left non-executable is a FAILURE, not a success.
    ///
    /// ⚠⚠ git ignores a non-executable hook without a word, so `setup` exiting 0
    /// after failing to repair the bit is a guard reported as armed while it is not.
    /// Nothing covered this: every test called `install_git_hooks_into` through
    /// `.expect("install")`, so deleting the `failures += 1` — and then the entire
    /// `if failures > 0 { bail! }` — both shipped green. `uninstall_hooks_from` had
    /// the symmetric test all along, which is what makes this an asymmetry rather
    /// than an oversight.
    #[test]
    #[cfg(unix)]
    fn a_hook_whose_bit_cannot_be_repaired_is_a_failure_not_a_success() {
        use std::cell::RefCell;
        use std::os::unix::fs::PermissionsExt;

        let dir = std::env::temp_dir().join(format!("cf-norepair-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).expect("temp dir");
        // Ours and CURRENT, so the only work left is the executable bit — which is
        // exactly the path whose failure was uncounted.
        for (name, content) in HOOKS {
            let path = dir.join(name);
            std::fs::write(&path, content).expect("seed");
            std::fs::set_permissions(&path, std::fs::Permissions::from_mode(0o644))
                .expect("chmod 644");
        }

        let refused = RefCell::new(Vec::new());
        let result = super::install_git_hooks_into_with(&dir, &|p: &std::path::Path| {
            refused.borrow_mut().push(p.to_path_buf());
            Err(std::io::Error::new(
                std::io::ErrorKind::PermissionDenied,
                "refused by the test",
            ))
        });
        let attempted = refused.borrow().len();

        // POSITIVE CONTROL: the same fixture with a repair that WORKS must install
        // cleanly, or the assertion above would hold for a function that always fails.
        let ok_result = super::install_git_hooks_into_with(&dir, &|p: &std::path::Path| {
            crate::hook_install::make_executable(p)
        });
        let _ = std::fs::remove_dir_all(&dir);

        assert_eq!(
            attempted,
            HOOKS.len(),
            "a refusal on the first hook aborted the loop, so the rest were never \
             even attempted — the half-done install this counter exists to prevent"
        );
        let err = result.expect_err(
            "a hook left non-executable is one git IGNORES WITHOUT A WORD, so \
             reporting success here tells the developer their guard is armed when it \
             is not",
        );
        assert!(
            err.to_string().contains("2 hook(s)"),
            "every refusal must be counted, not just the last: {err}"
        );
        ok_result.expect("POSITIVE CONTROL: a repair that succeeds must install cleanly");
    }

    /// A failing lint must show the developer WHY, not just that.
    ///
    /// ⚠ cargo writes every diagnostic to stderr, so the `2>/dev/null` this call
    /// used to carry left `✗ Clippy check failed. Fix errors before committing.` and
    /// nothing else — no file, no lint, no line. It also hid the CRLF crate-name
    /// failure above for as long as that bug existed: the hook rejected every Rust
    /// commit on Windows and gave no clue that the reason was `invalid character in
    /// package name`. A verdict with no evidence is the thing this repo keeps
    /// refusing elsewhere; it does not get a pass because it is only a lint.
    #[test]
    fn a_failing_lint_shows_cargos_own_diagnostic() {
        let diagnostic = "error: length comparison to zero at src/lib.rs:1:5";
        let (ok, out, argv) =
            run_hook_with_crate_and_cargo("demo", false, Some(("clippy", diagnostic)));

        assert!(
            argv.iter().any(|a| a.contains("clippy")),
            "PREMISE: clippy was never reached, so this proves nothing;\n{out}"
        );
        assert!(!ok, "a failing lint must fail the commit;\n{out}");
        assert!(
            out.contains("Clippy check failed"),
            "the hook must say the lint failed;\n{out}"
        );
        assert!(
            out.contains(diagnostic),
            "cargo's own diagnostic was swallowed, leaving the developer a verdict \
             with no evidence;\n{out}"
        );
    }

    /// The formatting step must show its cause on failure and stay QUIET on success.
    ///
    /// ⚠ TWO-SIDED, because both one-sided versions of this shipped. First the step
    /// carried `2>/dev/null`, so a missing rustfmt component produced only "Run:
    /// cargo fmt --all" — advice that fixes nothing. Removing the redirect put 3552
    /// lines on EVERY commit: `rustfmt.toml` sets nightly-only options and cargo
    /// warns about each, per file. A gate on either half alone would have accepted
    /// the other failure.
    #[test]
    fn the_format_step_shows_its_cause_but_only_when_it_fails() {
        // ⚠ No apostrophes: the stub embeds this in a single-quoted shell literal,
        // and the first one ends the string. The mechanism was fine; the fixture was
        // not, which the assertion caught by showing the mangled text back.
        let cause = "error: rustfmt is not installed for the stable toolchain";
        let (ok, out, _) = run_hook_with_crate_and_cargo("demo", false, Some(("fmt", cause)));
        assert!(!ok, "a formatting failure must fail the commit;\n{out}");
        assert!(
            out.contains(cause),
            "the developer was told to run `cargo fmt --all`, which would not have \
             helped, and the actual cause was discarded;\n{out}"
        );

        // NEGATIVE CONTROL: on success the step must add no noise. The stub prints
        // nothing, so anything here would have come from an unredirected cargo.
        let (ok, quiet, _) = run_hook_with_crate("demo", false);
        assert!(ok, "the ordinary case must pass;\n{quiet}");
        assert!(
            !quiet.contains(cause) && quiet.lines().count() < 20,
            "the formatting step is noisy on the happy path ({} lines); captured \
             output must only be printed when it means something;\n{quiet}",
            quiet.lines().count()
        );
    }

    /// A staged mesh must actually BLOCK the commit — the behaviour, not the prose.
    ///
    /// This repository is PUBLIC and the casting pipeline's inputs are anatomical
    /// scans of a real person. `.gitignore` cannot stop `git add -f`; this hook is
    /// the only thing that can, and #709 shipped it to one of the two installers
    /// with nothing checking that it worked.
    #[test]
    fn every_guarded_extension_blocks_the_commit() {
        // EVERY extension. Testing only .stl left the others with zero coverage, so
        // dropping one from the pathspec was a change no test opposed.
        // ★ `.step`/`.stp` were in neither the guard nor .gitignore, while mesh-io
        // has read and written STEP as a first-class format all along.
        for name in [
            "part.stl",
            "part.obj",
            "part.ply",
            "part.3mf",
            "part.mtl",
            "part.step",
            "part.stp",
        ] {
            let (ok, out) = run_hook(name, false);
            // `!ok` is load-bearing now that the scratch repo is a valid, formatted
            // cargo project — `a_clean_stage_passes` proves the hook CAN exit 0.
            assert!(!ok, "{name} did not block the commit; output:\n{out}");
            assert!(
                out.contains("Refusing to commit mesh/scan binaries"),
                "{name} did not trip the mesh guard; output:\n{out}"
            );
            // AND that it exited AT the guard: the guard `exit 1`s immediately, so
            // the formatting step must never be reached. This is a separate claim
            // from `!ok` above — an exit code alone cannot say WHERE it exited.
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
        // ⚠ A second, identical `run_hook("SCAN.STL", ..)` used to sit below this
        // loop re-asserting the loop's own two predicates on the loop's own first
        // input. Only its third assertion was new, so it is folded in here.
        for name in [
            "SCAN.STL",
            "SCAN.OBJ",
            "SCAN.PLY",
            "SCAN.3MF",
            "SCAN.MTL",
            "SCAN.STEP",
            "SCAN.STP",
        ] {
            let (ok, out) = run_hook(name, false);
            assert!(!ok, "{name} did not block the commit; output:\n{out}");
            assert!(
                out.contains("Refusing to commit mesh/scan binaries"),
                "{name} was NOT blocked — is :(icase) still on every entry?:\n{out}"
            );
            assert!(
                !out.contains("Checking formatting"),
                "{name} tripped the guard but did not exit there:\n{out}"
            );
        }
    }

    /// The documented override must work, and must ANNOUNCE itself. Printing
    /// "no scan/mesh binaries staged" while letting meshes through — which the
    /// hook used to do — hides the one commit anyone would want a record of.
    #[test]
    fn the_override_lets_it_through_and_says_so() {
        let (ok, out) = run_hook("part.stl", true);
        // ⚠ THE EXIT STATUS WAS DISCARDED. Every assertion here was on stdout, so
        // the hook could print its notice and then die — which is exactly what the
        // CF_ALLOW_MESH escape did on the unreadable-index path, under `set -e`,
        // exit 129. The sibling test was upgraded to check this; this one was not,
        // and an `exit 1` appended to the allow branch survived all three checks.
        assert!(
            ok,
            "CF_ALLOW_MESH=1 printed its notice and then FAILED the commit anyway; \
             output:\n{out}"
        );
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
        // ⚠ PREMISE CHECK, and it is the whole test. This case differs from the ADD
        // case ONLY if the seed commit succeeds: if it fails, `old.stl` never reaches
        // HEAD, the later `git add` stages it as A rather than M, and the assertion
        // below passes while `--diff-filter=A` — the exact mutation this test exists
        // to kill — survives. Measured: forcing the commit to fail leaves it green.
        let git = |args: &[&str]| {
            let out = Command::new("git")
                .args(args)
                .current_dir(&dir)
                .output()
                .expect("git");
            assert!(
                out.status.success(),
                "scratch-repo setup failed at `git {}` — without a seed COMMIT this \
                 test silently degrades to the ADD case:\n{}{}",
                args.join(" "),
                String::from_utf8_lossy(&out.stdout),
                String::from_utf8_lossy(&out.stderr)
            );
            out
        };
        git(&["init", "-q", "."]);
        git(&["config", "user.email", "t@t"]);
        git(&["config", "user.name", "t"]);
        // Same reason as `env_remove("CF_ALLOW_MESH")` in `run_hook`: the scratch
        // repo inherits the developer's GLOBAL config, and a `commit.gpgsign=true`
        // there fails the seed commit on their machine and nobody else's.
        git(&["config", "commit.gpgsign", "false"]);
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

    /// The right hook text must land at the right FILENAME.
    ///
    /// Nothing covered this. A mutation survey found that swapping the two consts
    /// in `install_git_hooks_into` — writing the commit-msg script to `pre-commit`
    /// and vice versa — passed the entire suite. The scan/mesh guard lives only in
    /// the pre-commit text, so that swap silently disarms it.
    ///
    /// ⚠ ITS PAIR IS `each_hook_is_paired_with_the_filename_git_runs_it_under` in
    /// `hook_install.rs`, which asserts the same property on the `HOOKS` table.
    /// Neither is redundant: that one pins the DATA, this one drives the INSTALLER
    /// and would still catch a mispairing introduced in the write loop. An earlier
    /// version of both comments claimed to be the only gate.
    #[test]
    fn the_installer_writes_each_hook_to_its_own_filename() {
        let dir = std::env::temp_dir().join(format!("cf-install-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).expect("temp dir");

        super::install_git_hooks_into(&dir).expect("install");

        let pre = std::fs::read_to_string(dir.join("pre-commit")).expect("pre-commit written");
        let msg = std::fs::read_to_string(dir.join("commit-msg")).expect("commit-msg written");
        // git IGNORES a hook that is not executable, and says nothing when it does.
        // The chmod lives in `hook_install::write_hook_file`, whose `#[cfg(unix)]`
        // block is a no-op elsewhere, so a regression there disarms both hooks with
        // every content assertion still green.
        #[cfg(unix)]
        let modes: Vec<(&str, u32)> = {
            use std::os::unix::fs::PermissionsExt;
            HOOKS
                .iter()
                .map(|(name, _)| {
                    let m = std::fs::metadata(dir.join(name))
                        .expect("installed hook")
                        .permissions()
                        .mode();
                    (*name, m)
                })
                .collect()
        };
        let _ = std::fs::remove_dir_all(&dir);

        #[cfg(unix)]
        for (name, mode) in modes {
            assert!(
                mode & 0o111 != 0,
                "{name} was installed non-executable ({mode:o}); git will silently \
                 ignore it and the scan/mesh guard never runs"
            );
        }

        assert_eq!(
            pre, PRE_COMMIT_HOOK,
            "pre-commit did not receive PRE_COMMIT_HOOK"
        );
        assert_eq!(
            msg, COMMIT_MSG_HOOK,
            "commit-msg did not receive COMMIT_MSG_HOOK"
        );
        assert!(
            pre.contains("CF_ALLOW_MESH"),
            "the file installed as pre-commit has no scan/mesh guard — the consts are \
             crossed, and every other test would still pass"
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

    /// Every refusal, which nothing reached before.
    ///
    /// ⚠ These are the containment layer as the USER meets it. `classify_hooks_dir`
    /// was well covered, but its verdicts fed a `match` that no test called, so
    /// mutating either refusal arm to `Ok(dir)` left the whole suite green while
    /// `cargo xtask setup` wrote CortenForge's hooks into a developer's global hooks
    /// directory, or into an unrelated repository. A verdict nobody acts on is not a
    /// guard.
    ///
    /// ⚠ The name used to say "that is not ours", which stopped being true when
    /// `GitCannotRun` arrived: that directory IS ours, and is refused anyway because
    /// git would not run what we wrote there. A test name is read far more often
    /// than its body.
    #[test]
    fn setup_refuses_every_hooks_directory_it_must_not_write() {
        use crate::hook_install::{Attempted, HooksDir};
        use std::path::{Path, PathBuf};

        const INSTALL: Attempted = Attempted::Install { retry: "re-run" };
        let root = Path::new("/repo");
        let ours = super::hooks_dir_from(
            Some(HooksDir::Repo(PathBuf::from("/repo/.git/hooks"))),
            root,
            false,
            INSTALL,
        );
        assert_eq!(
            ours.expect("our own hooks dir must be accepted"),
            PathBuf::from("/repo/.git/hooks"),
            "POSITIVE CONTROL: a rule that refuses everything is not containment"
        );

        // Each refusal must NAME the directory — that string is the developer's only
        // route to fixing it, and an error that says only "refused" is a dead end.
        let shared = super::hooks_dir_from(
            Some(HooksDir::Shared(PathBuf::from("/home/dev/.githooks"))),
            root,
            false,
            INSTALL,
        )
        .expect_err("a hooks dir outside the repo is never ours to write");
        assert!(
            shared.to_string().contains("/home/dev/.githooks"),
            "the refusal must name the directory: {shared}"
        );
        assert!(
            !shared.to_string().contains("your other repos.")
                || shared.to_string().contains("may be shared"),
            "a repo-LOCAL core.hooksPath outside the tree lands here too, so this \
             must not assert whose directory it is: {shared}"
        );

        let other = super::hooks_dir_from(
            Some(HooksDir::OtherRepo(PathBuf::from("/outer/.git/hooks"))),
            root,
            false,
            INSTALL,
        )
        .expect_err("an ancestor repository is never ours to write");
        assert!(
            other.to_string().contains("/outer/.git/hooks"),
            "the refusal must name the directory: {other}"
        );

        // ★ OURS, AND STILL REFUSED. Every refusal above is "not our directory";
        // this one is "our directory, which git cannot run hooks from" — installing
        // would make git reject every commit in the checkout. Both installers must
        // reach the same verdict, so `build_outcome` gates the same case.
        let unrunnable = super::hooks_dir_from(
            Some(HooksDir::GitCannotRun(PathBuf::from("/repo"))),
            root,
            false,
            INSTALL,
        )
        .expect_err("core.hooksPath=. must not be installed into");
        assert!(
            unrunnable.to_string().contains("core.hooksPath")
                && unrunnable.to_string().contains("EVERY commit"),
            "name the setting to change and what installing would have done: \
             {unrunnable}"
        );

        // ⚠⚠ AND UNINSTALL MUST BE ABLE TO CLEAN IT. Containment already passed, so
        // this directory is inside the checkout — and the developers who have hooks
        // sitting in an unrunnable one are exactly those the PREVIOUS code installed
        // for. Every commit in those checkouts is refused right now by a hook we put
        // there; answering `cargo xtask uninstall` with "fix the setting, then
        // re-run" tells them to install the thing they asked to remove, and leaves
        // them no way out but deleting the files by hand.
        let removable = super::hooks_dir_from(
            Some(HooksDir::GitCannotRun(PathBuf::from("/repo/-hooks"))),
            root,
            false,
            Attempted::Uninstall,
        )
        .expect("uninstall must be able to remove hooks from a dir git cannot exec");
        assert_eq!(
            removable,
            PathBuf::from("/repo/-hooks"),
            "uninstall must clean the directory git named, not a guess"
        );
        // POSITIVE CONTROL, so "uninstall accepts anything" cannot pass: a directory
        // OUTSIDE the checkout is still not ours to delete from, whatever we are
        // attempting.
        super::hooks_dir_from(
            Some(HooksDir::Shared(PathBuf::from("/home/dev/.githooks"))),
            root,
            false,
            Attempted::Uninstall,
        )
        .expect_err("uninstall must never delete from a hooks dir outside the repo");

        let unknown = super::hooks_dir_from(None, root, false, INSTALL)
            .expect_err("an unknown hooks dir is not a licence");
        // The same checkout, with the environment that explains it — `build.rs` said
        // this and `setup` did not.
        let with_env = super::hooks_dir_from(None, root, true, INSTALL)
            .expect_err("an unknown hooks dir is not a licence");
        assert!(
            with_env.to_string().contains("GIT_DIR"),
            "name the variable the developer has to unset: {with_env}"
        );
        assert!(
            unknown.to_string().contains("/repo"),
            "the refusal must name the checkout: {unknown}"
        );
    }

    /// Resolve the hooks dir through the PRODUCTION path.
    ///
    /// ⚠ Deliberately not a local reimplementation. An earlier version spelled out
    /// `git rev-parse` here, which left the function the binary actually calls with
    /// no test reaching it — reverting that one to `root.join(".git/hooks")` kept the
    /// whole suite green. That is #709's shape inside the fix for #709.
    fn hooks_dir_of(repo: &std::path::Path) -> std::path::PathBuf {
        match crate::hook_install::resolve_hooks_dir(repo) {
            Some(crate::hook_install::HooksDir::Repo(dir)) => dir,
            other => panic!("expected a repo-owned hooks dir for {repo:?}, got {other:?}"),
        }
    }

    /// ★ END-TO-END: git ITSELF must run the guard on a real `git commit`.
    ///
    /// Every other test here runs `sh hook.sh` directly. That proves the hook SCRIPT
    /// works and says nothing about whether it ever reaches git — which is #709's bug
    /// exactly: a correct script that was never installed where git would run it.
    /// This is the only test in the suite that would have caught the original.
    ///
    /// It covers the whole chain in one pass: the installer picks the directory git
    /// actually uses, writes the right text under the right filename, makes it
    /// executable, and git invokes it on commit. A regression in ANY link fails here.
    #[test]
    fn git_itself_runs_the_installed_guard_on_a_real_commit() {
        let dir = std::env::temp_dir().join(format!("cf-e2e-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).expect("temp dir");
        let git = |args: &[&str]| {
            let out = Command::new("git")
                .args(args)
                .current_dir(&dir)
                .env_remove("CF_ALLOW_MESH")
                .output()
                .expect("git");
            let combined = format!(
                "{}{}",
                String::from_utf8_lossy(&out.stdout),
                String::from_utf8_lossy(&out.stderr)
            );
            (out.status.success(), combined)
        };
        let must = |args: &[&str]| {
            let (ok, out) = git(args);
            assert!(
                ok,
                "scratch-repo setup failed at `git {}`:\n{out}",
                args.join(" ")
            );
        };
        must(&["init", "-q", "."]);
        must(&["config", "user.email", "t@t"]);
        must(&["config", "user.name", "t"]);
        // The developer's global signing config would otherwise fail every commit
        // here on their machine and nobody else's.
        must(&["config", "commit.gpgsign", "false"]);
        // ⚠ Neutralise an inherited GLOBAL `core.hooksPath`. Without this the scratch
        // repo resolves to the DEVELOPER'S own hooks directory. Containment now
        // refuses to write there — exactly what that check is for — but the test
        // would then fail on their machine for a reason unrelated to what it tests.
        // Pinning to this repo's own default is what the default already is.
        let own_hooks = dir.join(".git/hooks");
        must(&["config", "core.hooksPath", &own_hooks.to_string_lossy()]);

        // Install through the REAL installer, into the directory git reports.
        super::install_git_hooks_into(&hooks_dir_of(&dir)).expect("install");

        // A valid, formatted cargo project so `cargo fmt --all --check` passes and
        // cannot confound the result. (Clippy is skipped here: the hook resolves a
        // staged file to its crate by walking UP from `dirname`, and a root-level
        // path yields ".", whose loop body never runs — a real gap in the hook's
        // clippy step, out of scope for this change.)
        std::fs::write(
            dir.join("Cargo.toml"),
            b"[workspace]\n[package]\nname = \"p\"\nversion = \"0.0.0\"\nedition = \"2021\"\n",
        )
        .expect("Cargo.toml");
        std::fs::create_dir_all(dir.join("src")).expect("src");
        std::fs::write(dir.join("src/main.rs"), b"fn main() {}\n").expect("main.rs");

        // THE CLAIM: a staged mesh cannot be committed, via git, not via `sh`.
        std::fs::write(dir.join("part.stl"), b"solid x\n").expect("fixture");
        must(&["add", "-f", "part.stl"]);
        let blocked = git(&["commit", "-m", "feat(x): try to sneak a mesh in"]);

        // POSITIVE CONTROL: the same installed hooks must let a clean commit through.
        // Without it, "the hook was never installed" and "the guard blocked it" are
        // the same result and the assertion above proves nothing.
        must(&["rm", "-q", "--cached", "part.stl"]);
        std::fs::remove_file(dir.join("part.stl")).expect("rm fixture");
        must(&["add", "Cargo.toml", "src/main.rs"]);
        let clean = git(&["commit", "-m", "feat(x): a clean commit"]);

        // ⚠ Sample everything, THEN clean up, THEN assert. Cleanup placed after an
        // assert never runs on failure, and this scratch repo holds whatever the
        // positive control's `cargo` invocations built — one leaked tree per failing
        // run, in a pid-named directory a later run will never reclaim.
        let _ = std::fs::remove_dir_all(&dir);

        let (blocked_ok, blocked_out) = blocked;
        assert!(
            !blocked_ok,
            "git COMMITTED a staged mesh — the guard never ran:\n{blocked_out}"
        );
        assert!(
            blocked_out.contains("Refusing to commit mesh/scan binaries"),
            "the commit failed, but not AT our guard — without this the test would \
             pass on any broken hook, so the message is the real assertion:\n{blocked_out}"
        );
        let (clean_ok, clean_out) = clean;
        assert!(
            clean_ok,
            "the installed hooks rejected a clean commit:\n{clean_out}"
        );
    }

    /// The worktree case, two-sided: the OLD assumption must be shown to fail, and
    /// ours to work. `<root>/.git/hooks` does not exist in a linked worktree — `.git`
    /// is a FILE — yet git runs hooks there from the common dir. Agents in this repo
    /// run with `isolation: "worktree"`, so this is a routine environment.
    #[test]
    fn a_worktree_resolves_to_the_common_hooks_dir_where_naive_joining_fails() {
        // `resolve_hooks_dir` spawns its own git, so an env var set on OUR Command
        // cannot reach it, and a GLOBAL `core.hooksPath` would redirect the answer.
        // Skipping loudly is the honest option here: the alternative — pinning the
        // config — is exactly what made this assertion a tautology before.
        let has_global_hookspath = Command::new("git")
            .args(["config", "--global", "--get", "core.hooksPath"])
            .output()
            .map(|o| o.status.success())
            .unwrap_or(false);
        if has_global_hookspath {
            eprintln!(
                "SKIPPED a_worktree_resolves_…: a global core.hooksPath is set, \
                 which redirects git's answer and cannot be isolated from here"
            );
            return;
        }

        let base = std::env::temp_dir().join(format!("cf-wt-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&base);
        let repo = base.join("repo");
        std::fs::create_dir_all(&repo).expect("temp dir");
        let git_in = |cwd: &std::path::Path, args: &[&str]| {
            let out = Command::new("git")
                .args(args)
                .current_dir(cwd)
                .output()
                .expect("git");
            assert!(
                out.status.success(),
                "git {} failed:\n{}",
                args.join(" "),
                String::from_utf8_lossy(&out.stderr)
            );
        };
        git_in(&repo, &["init", "-q", "."]);
        git_in(&repo, &["config", "user.email", "t@t"]);
        git_in(&repo, &["config", "user.name", "t"]);
        git_in(&repo, &["config", "commit.gpgsign", "false"]);
        // ⚠ NO `core.hooksPath` pin here. An earlier version pinned it to the value
        // this test then asserts git returns, which made the positive assertion a
        // TAUTOLOGY — it would have held even if worktree resolution were reverted
        // entirely. Measured: with hooksPath UNSET git already answers the common
        // dir, so the pin bought nothing and cost the test its discrimination.
        std::fs::write(repo.join("a.txt"), b"x\n").expect("seed");
        git_in(&repo, &["add", "a.txt"]);
        git_in(&repo, &["commit", "-q", "-m", "chore: seed", "--no-verify"]);

        let wt = base.join("wt");
        git_in(
            &repo,
            &["worktree", "add", "-q", wt.to_str().unwrap(), "-b", "wtb"],
        );

        // ⚠ Sample every fact BEFORE cleanup. `.exists()` on a deleted tree is
        // false for both the broken and the working path, which reads as a failure
        // of the thing under test rather than of the test's own ordering.
        let resolved = hooks_dir_of(&wt);
        let naive_exists = wt.join(".git/hooks").exists();
        let resolved_exists = resolved.exists();
        let dot_git_is_file = wt.join(".git").is_file();
        // The one right answer, computed independently of the code under test.
        let expected = std::fs::canonicalize(&repo)
            .expect("canonicalize repo")
            .join(".git")
            .join("hooks");
        let _ = std::fs::remove_dir_all(&base);

        assert!(dot_git_is_file, "a linked worktree's .git should be a FILE");
        // ⚠ WHICH directory, not merely "one that exists". `resolved.exists()` alone
        // is satisfied by returning the worktree root, or the toplevel, for every
        // input — the test named for common-dir resolution could not see the
        // difference it is named after.
        assert_eq!(
            resolved, expected,
            "a worktree's hooks must resolve to the MAIN checkout's common dir"
        );
        assert!(
            !naive_exists,
            "NEGATIVE CONTROL BROKEN: <worktree>/.git/hooks exists, so this test can \
             no longer show that naive joining fails"
        );
        assert!(
            resolved_exists,
            "asking git did not yield a usable hooks dir in a worktree ({})",
            resolved.display()
        );
    }

    /// Ask git, in a repository whose answer CANNOT be `.git/hooks`.
    ///
    /// ⚠⚠ THIS IS THE SUITE'S UNCONDITIONAL GUARD on the whole "ask git" path.
    /// Deleting the `rev-parse` block from `resolve_hooks_dir` — reverting precisely
    /// to the bug this arc exists to fix — was killed by exactly one test, the
    /// worktree one, which RETURNS EARLY whenever the developer has a global
    /// `core.hooksPath`. libtest captures `eprintln!` for a passing test and throws
    /// it away, so that skip is invisible: the run says `ok` and the only guard on
    /// the central claim silently did nothing.
    ///
    /// A repo-LOCAL `core.hooksPath` overrides any global one, so this cannot skip.
    /// And it is not the tautology the worktree test's old config pin was: that pin
    /// named `<dir>/.git/hooks`, which is exactly what the NAIVE join produces, so a
    /// reverted implementation passed it. This names a directory the naive join can
    /// never yield, which is what makes the assertion discriminating.
    #[test]
    fn a_repo_local_hooks_path_is_resolved_and_the_naive_join_cannot_reach_it() {
        let base = std::env::temp_dir().join(format!("cf-localhp-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&base);
        let repo = base.join("repo");
        std::fs::create_dir_all(&repo).expect("temp dir");
        let git_in = |args: &[&str]| {
            let out = Command::new("git")
                .args(args)
                .current_dir(&repo)
                .output()
                .expect("git");
            assert!(
                out.status.success(),
                "git {} failed:\n{}",
                args.join(" "),
                String::from_utf8_lossy(&out.stderr)
            );
        };
        git_in(&["init", "-q", "."]);
        // In-tree, versioned, and NOT `.git/hooks` — the widespread `.githooks`
        // convention. Local config beats global, so no environment can skip this.
        git_in(&["config", "core.hooksPath", ".githooks"]);
        std::fs::create_dir_all(repo.join(".githooks")).expect("hooks dir");

        let resolved = hooks_dir_of(&repo);
        let canonical = std::fs::canonicalize(&repo).expect("canonicalize");
        let naive = canonical.join(".git").join("hooks");
        let naive_exists = naive.exists();
        let _ = std::fs::remove_dir_all(&base);

        assert!(
            naive_exists,
            "NEGATIVE CONTROL BROKEN: <repo>/.git/hooks does not exist, so this test \
             can no longer show that the naive join returns a plausible WRONG answer \
             rather than an obviously missing one"
        );
        assert_eq!(
            resolved,
            canonical.join(".githooks"),
            "git reads hooks from .githooks here; anything else is a file git never \
             runs, installed with a success message"
        );
        assert_ne!(
            resolved, naive,
            "the naive join must not be able to produce this answer, or the test \
             cannot tell the two implementations apart"
        );
    }

    /// `GIT_DIR` in the environment must not steer resolution into another repo.
    ///
    /// ⚠ The containment check asks whether git's `--show-toplevel` equals the
    /// directory we asked about. With `GIT_DIR` set and no `GIT_WORK_TREE`, git
    /// skips discovery and calls OUR directory the top level — so that check passes
    /// BY CONSTRUCTION while the hooks path answers for the other repository. The
    /// verdict is `Repo`, pointing somewhere else entirely.
    ///
    /// Reachable, not hypothetical: git exports an absolute `GIT_DIR` to every hook
    /// it runs in a linked worktree, and to `git submodule foreach` — so a hook or a
    /// `foreach` that builds CortenForge elsewhere inherits one.
    ///
    /// ⚠ Asserted on the COMMAND, not end to end. The end-to-end version needs
    /// `set_var`, which is per-process; written that way it broke four unrelated git
    /// tests running in parallel (`fatal: not in a git directory`). The two halves of
    /// the claim are split accordingly: git's behaviour under `GIT_DIR` is measured
    /// and recorded here, and what this test pins is that we clear the variables.
    #[test]
    fn every_git_question_clears_the_variables_that_would_answer_for_another_repo() {
        use std::path::Path;

        let cmd = crate::hook_install::git_command(Path::new("/repo"), &["rev-parse"]);
        let cleared: Vec<&str> = cmd
            .get_envs()
            .filter(|(_, v)| v.is_none())
            .filter_map(|(k, _)| k.to_str())
            .collect();

        // Measured, git 2.50.1: with GIT_DIR set to repo B's git dir and the command
        // run in checkout A, `--show-toplevel` answers **A** — git skips discovery
        // and calls the working directory the top level — while --git-common-dir and
        // --git-path answer for **B**. So `toplevel == repo_root` holds by
        // construction, containment passes, and the verdict is Repo(B/.git/hooks).
        // The naive `.git/hooks` join this replaced could not be steered that way.
        // ⚠ HARDCODED, NOT ITERATED FROM `GIT_VARS_CLEARED`. Looping over the same
        // const the production code loops over made the oracle the SUT: emptying that
        // const would have satisfied the assertion vacuously. These four names are
        // written out here so the test can disagree with the code.
        for var in [
            "GIT_DIR",
            "GIT_COMMON_DIR",
            "GIT_WORK_TREE",
            "GIT_INDEX_FILE",
        ] {
            assert!(
                cleared.contains(&var),
                "{var} is inherited by our git call, so the environment can decide \
                 which repository we install into: cleared = {cleared:?}"
            );
        }
    }

    /// `setup` and `uninstall` must reach the same ownership verdict as `build.rs`.
    ///
    /// ⚠⚠ THE TWO INSTALLERS DISAGREED, and the untested one destroyed data.
    /// `build.rs` classified first and left a foreign hook alone with a warning;
    /// `setup` overwrote it and printed `✓ Installed`, indistinguishable from the
    /// ordinary case, and `uninstall` deleted it by NAME. Measured before the fix: a
    /// developer's `pre-commit` replaced, sha changed, no backup, three cheerful
    /// lines of output.
    ///
    /// Resolving the hooks directory properly is what made this urgent rather than
    /// merely wrong: `setup` run inside a linked worktree now reaches the MAIN
    /// checkout's hooks, so the clobber stopped being confined to the developer who
    /// typed the command — an agent working in a worktree could do it to them.
    #[test]
    fn neither_installing_nor_uninstalling_touches_a_hook_that_is_not_ours() {
        let dir = std::env::temp_dir().join(format!("cf-own-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).expect("temp dir");

        // Someone else's hook where OUR pre-commit would go; our own, stale, where
        // commit-msg goes — so one run exercises both verdicts.
        let theirs = "#!/bin/sh\n# husky\necho theirs\n";
        std::fs::write(dir.join("pre-commit"), theirs).expect("seed foreign");
        let stale = format!(
            "#!/bin/sh\n{}\n# an older CortenForge hook\n",
            COMMIT_MSG_HOOK.lines().nth(1).expect("title line")
        );
        std::fs::write(dir.join("commit-msg"), &stale).expect("seed stale");

        super::install_git_hooks_into(&dir).expect("install");

        let foreign_after_install =
            std::fs::read_to_string(dir.join("pre-commit")).expect("read foreign");
        let ours_after_install =
            std::fs::read_to_string(dir.join("commit-msg")).expect("read ours");

        super::uninstall_hooks_from(&dir).expect("uninstall");

        let foreign_survives = dir.join("pre-commit").exists();
        let foreign_final = std::fs::read_to_string(dir.join("pre-commit")).unwrap_or_default();
        let ours_removed = !dir.join("commit-msg").exists();
        let _ = std::fs::remove_dir_all(&dir);

        assert_eq!(
            foreign_after_install, theirs,
            "install overwrote a hook that was not ours — the file is gone and the \
             developer was told it was a successful install"
        );
        assert_eq!(
            ours_after_install, COMMIT_MSG_HOOK,
            "POSITIVE CONTROL: our own stale hook must still be healed, or this test \
             would pass just as well against an installer that writes nothing"
        );
        assert!(
            foreign_survives && foreign_final == theirs,
            "uninstall deleted a hook it never installed"
        );
        assert!(
            ours_removed,
            "POSITIVE CONTROL: uninstall must still remove OUR hook"
        );
    }

    /// Run the commit-msg hook against `message`. Returns (exited_zero, output).
    ///
    /// EXECUTED, not grepped. Asserting on `COMMIT_MSG_HOOK`'s text would prove
    /// nothing about what the hook does — the same mistake the pre-commit tests were
    /// built to avoid, and the reason this hook's regex went uncovered so long.
    fn run_commit_msg(message: &str) -> (bool, String) {
        static N: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
        let uniq = N.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        let dir = std::env::temp_dir().join(format!("cf-msg-{}-{uniq}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).expect("temp dir");

        let msg_path = dir.join("COMMIT_EDITMSG");
        std::fs::write(&msg_path, message).expect("message");
        let hook = dir.join("hook.sh");
        std::fs::write(&hook, COMMIT_MSG_HOOK).expect("hook");
        let out = Command::new("sh")
            .arg(&hook)
            .arg(&msg_path)
            .current_dir(&dir)
            .output()
            .expect("run hook");
        let combined = format!(
            "{}{}",
            String::from_utf8_lossy(&out.stdout),
            String::from_utf8_lossy(&out.stderr)
        );
        let _ = std::fs::remove_dir_all(&dir);
        (out.status.success(), combined)
    }

    /// The Merge/Revert allowances must read the SUBJECT, not the whole message.
    ///
    /// They used to grep everything, so any BODY line beginning "Merge " switched
    /// conventional-commit enforcement off for that commit. TWO-SIDED on purpose: a
    /// real merge subject must still be allowed, or this is not a fix, just a
    /// stricter hook that breaks `git merge`.
    #[test]
    fn merge_and_revert_are_allowed_only_in_the_subject() {
        for (label, msg) in [
            (
                "Merge in body",
                "garbage subject\n\nMerge the two config files\n",
            ),
            (
                "Revert in body",
                "garbage subject\n\nRevert that change later\n",
            ),
        ] {
            let (ok, out) = run_commit_msg(msg);
            assert!(
                !ok,
                "{label}: enforcement was disabled by a BODY line;\n{out}"
            );
        }

        // POSITIVE CONTROL: the allowances must still allow what they are for.
        for (label, msg) in [
            ("real merge", "Merge branch 'main' into feature\n"),
            ("real revert", "Revert \"feat(x): y\"\n"),
            (
                "valid subject, Merge in body",
                "feat(xtask): a thing\n\nMerge notes\n",
            ),
        ] {
            let (ok, out) = run_commit_msg(msg);
            assert!(ok, "{label}: rejected a message that must pass;\n{out}");
        }
    }

    /// Autosquash markers must pass, and the type list must be real.
    ///
    /// ⚠ SAFETY, not convenience. `git commit --fixup=<sha>` writes `fixup! <subject>`,
    /// which cannot match the conventional-commit pattern — so recording a fixup
    /// required `--no-verify`, and that ALSO disables the pre-commit hook. The cheap
    /// gate was pushing people onto the hammer that disarms the scan/mesh guard,
    /// whose miss is permanent and public. Those messages are rewritten out of
    /// history by `git rebase --autosquash` before anything reaches CI.
    ///
    /// ⚠ The type alternation had no coverage at all: only "garbage subject" (which
    /// fails everything) and one `feat(xtask):`. Deleting `perf` or `ci` from the
    /// pattern was a mutant nothing opposed.
    #[test]
    fn autosquash_markers_pass_and_every_declared_type_is_accepted() {
        for marker in [
            "fixup! feat(x): y",
            "squash! feat(x): y",
            "amend! feat(x): y",
        ] {
            let (ok, out) = run_commit_msg(&format!("{marker}\n"));
            assert!(
                ok,
                "{marker} was rejected, so recording a fixup needs --no-verify — \
                 which drops the scan/mesh guard too;\n{out}"
            );
        }

        // Every type the hook's own message promises. A type listed in the help text
        // but missing from the pattern is a documented lie the suite cannot see.
        for ty in [
            "feat", "fix", "refactor", "test", "docs", "chore", "perf", "ci", "build", "style",
        ] {
            let (ok, out) = run_commit_msg(&format!("{ty}: a description\n"));
            assert!(ok, "type `{ty}` is advertised but rejected;\n{out}");
            let (scoped, out2) = run_commit_msg(&format!("{ty}(cf-cast): a description\n"));
            assert!(scoped, "type `{ty}` with a scope was rejected;\n{out2}");
        }

        // NEGATIVE CONTROLS: a rule that accepts everything is not a rule.
        for (label, msg) in [
            ("unknown type", "feet: a description\n"),
            ("no description", "feat:\n"),
            ("no colon", "feat a description\n"),
            ("uppercase scope", "feat(CfCast): a description\n"),
            ("fixup without the bang", "fixup feat(x): y\n"),
        ] {
            let (ok, out) = run_commit_msg(msg);
            assert!(!ok, "{label}: accepted a malformed subject;\n{out}");
        }
    }

    /// Run the pre-commit hook OUTSIDE a git repo, which is the cheapest way to make
    /// `git diff --cached` fail for real rather than by injection.
    fn run_hook_outside_a_repo(allow_mesh: bool) -> (bool, String) {
        static N: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
        let uniq = N.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        let dir = std::env::temp_dir().join(format!("cf-nogit-{}-{uniq}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).expect("temp dir");
        // A valid, formatted cargo project, so the ONLY thing wrong here is that
        // git cannot answer. Without it the hook dies at `cargo fmt` and the test
        // cannot tell "the override let it through" from "it failed later anyway" —
        // which is exactly how a broken override passed review once already.
        std::fs::write(
            dir.join("Cargo.toml"),
            b"[workspace]\n[package]\nname = \"p\"\nversion = \"0.0.0\"\nedition = \"2021\"\n",
        )
        .expect("Cargo.toml");
        std::fs::create_dir_all(dir.join("src")).expect("src");
        std::fs::write(dir.join("src/main.rs"), b"fn main() {}\n").expect("main.rs");

        let hook = dir.join("hook.sh");
        std::fs::write(&hook, PRE_COMMIT_HOOK).expect("hook");
        let mut cmd = Command::new("sh");
        cmd.arg(&hook).current_dir(&dir);
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

    /// The documented override must work on the unreadable-index path too — and must
    /// STILL not claim a clean stage.
    ///
    /// Without it the only way past a malfunctioning guard is `git commit
    /// --no-verify`, which also disables the commit-message hook: the mitigation for
    /// one broken gate silently drops a second. ⚠ The second assertion is the
    /// load-bearing one. Letting this path fall through to "✓ No scan/mesh binaries
    /// staged" would print a clean bill of health for a check that never ran, which
    /// is the precise false statement this guard's whole history is made of.
    #[test]
    fn the_override_skips_the_unreadable_index_loudly_without_claiming_a_clean_stage() {
        let (ok, out) = run_hook_outside_a_repo(true);
        // ⚠ THE EXIT CODE, not merely the message. The first version of this override
        // printed its notice and then died anyway, because a LATER unguarded
        // `git diff` failed under `set -e` — so the escape hatch did not escape, and
        // a test that only grepped stdout could not see it. Assert it ran to the end.
        assert!(
            ok,
            "the override printed its notice and then refused anyway — it did not \
             let the commit proceed, which is the whole point of it;\n{out}"
        );
        assert!(
            out.contains("Checking formatting"),
            "the hook never reached the steps after the guard;\n{out}"
        );
        assert!(
            out.contains("SKIPPED ENTIRELY"),
            "the override did not leave a record that the guard never ran;\n{out}"
        );
        assert!(
            !out.contains("No scan/mesh binaries staged"),
            "the hook claimed a clean stage for a check that never ran;\n{out}"
        );
    }

    /// The scan/mesh guard must fail CLOSED when git cannot answer.
    ///
    /// It used to end in `|| true`, so any git failure produced an empty list and the
    /// hook printed "✓ No scan/mesh binaries staged" — false — and let the commit
    /// through with the scan attached.
    ///
    /// ⚠ The MESSAGE is the assertion, not just the exit code. Both are checked here,
    /// but the exit code alone would be weak: for most of this hook's history a
    /// later unguarded `git diff` aborted it under `set -e` anyway, so a non-zero
    /// exit proved nothing about the guard. What cannot be faked is that the hook
    /// must never CLAIM a clean stage it was unable to verify.
    #[test]
    fn the_guard_refuses_when_it_cannot_read_the_index() {
        let (ok, combined) = run_hook_outside_a_repo(false);

        assert!(
            !combined.contains("No scan/mesh binaries staged"),
            "the guard claimed a clean stage it could not verify; output:\n{combined}"
        );
        assert!(
            combined.contains("scan/mesh guard cannot run"),
            "the guard did not refuse for the stated reason; output:\n{combined}"
        );
        assert!(!ok, "hook passed; output:\n{combined}");
        assert!(
            combined.contains("CF_ALLOW_MESH=1 git commit"),
            "the refusal did not name the override, so the only way out a developer \
             can find is --no-verify, which drops the commit-message gate too;\n{combined}"
        );
    }

    /// `hook_install::title_of` takes the marker from line 2, so line 2 must actually
    /// BE a title. It only requires that a second line exists — a hook whose line 2
    /// were `set -e` would build fine and yield the marker "set -e", after which the
    /// updater silently stops recognising its own hooks. That is this arc's original
    /// bug, reintroduced one level up, and it would be invisible: no error, hooks
    /// just quietly stop updating. Assert the shape the derivation depends on.
    #[test]
    fn each_hooks_marker_finds_its_own_hook_and_only_its_own() {
        use crate::hook_install::title_of;

        // ⚠ THE ORACLE USED TO BE THE SUT. This test re-implemented `title_of`'s
        // own line-2 parse inline and asserted the result looked right — so it
        // agreed with the code under test by construction, and duplicated an
        // assertion `title_is_taken_from_line_two_without_its_comment_marker`
        // already makes exactly. What follows is the claim nothing else covers.
        let markers: Vec<(&str, &str)> = HOOKS
            .iter()
            .map(|(name, hook)| (*name, title_of(hook)))
            .collect();

        for (name, hook) in HOOKS {
            let marker = title_of(hook);
            // `classify` recognises a hook by `current.contains(marker)`. A marker
            // derived from a hook that is not FOUND in it means the installer stops
            // recognising its own work and never heals a stale hook again.
            assert!(
                hook.contains(marker),
                "{name}'s own marker {marker:?} is not present in its text"
            );
        }

        // ★ AND THEY MUST DIFFER. Identical titles would make each hook match the
        // other's marker, so `classify` would call a `commit-msg` sitting at the
        // `pre-commit` path "ours, stale" and overwrite it — the cross-pairing
        // failure this file's HOOKS table exists to prevent, arriving by a second
        // route that the pairing test cannot see.
        for (a_name, a) in &markers {
            for (b_name, b) in &markers {
                if a_name != b_name {
                    assert_ne!(
                        a, b,
                        "{a_name} and {b_name} share the ownership marker {a:?}, so \
                         each would recognise the other as its own"
                    );
                }
            }
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
