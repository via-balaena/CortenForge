// The git-hook installer's data and its pure decisions, in one place.
//
// # Why this file exists
//
// It is `include!`d by `xtask/build.rs` AND compiled into the xtask binary, so
// everything here has a test target. A build script has none, and while this logic
// lived only in `build.rs` NOTHING covered it — a mutation survey found that gutting
// the ownership check to `Ok(_) => true` (clobber every hook, including other
// people's) passed the entire suite. That is the same shape as the bug this whole
// arc exists to fix: installer logic with no gate on it.
//
// ★ The same argument applies to the PAIRING — which hook text goes to which
// filename. It was written out once in `build.rs` and again in `setup.rs`, so giving
// the `setup.rs` copy a test left the `build.rs` copy uncovered: crossing its two
// consts still shipped green, and that is the installer a fresh clone actually runs.
// `HOOKS` is now the single source of the pairing and BOTH installers iterate it.
//
// Keep everything here PURE. The filesystem work stays in the installers; everything
// decidable from strings is decided here, where a test can reach it.

/// Pre-commit hook text. Single source: `xtask/hooks/pre-commit`.
pub const PRE_COMMIT_HOOK: &str = include_str!("../hooks/pre-commit");

/// Commit message hook text. Single source: `xtask/hooks/commit-msg`.
pub const COMMIT_MSG_HOOK: &str = include_str!("../hooks/commit-msg");

/// Every hook we install, paired with the filename git must run it under.
///
/// ⚠ The PAIRING is the payload here. Both entries are well-formed hooks, so
/// crossing them yields two files that each look fine on their own — the only
/// symptom is the scan/mesh guard going missing from `pre-commit`, which no
/// assertion about the hook TEXT can see. Written once, iterated by both
/// installers, covered by `each_hook_is_paired_with_the_filename_git_runs_it_under`.
pub const HOOKS: [(&str, &str); 2] = [
    ("pre-commit", PRE_COMMIT_HOOK),
    ("commit-msg", COMMIT_MSG_HOOK),
];

/// Where git will look for hooks, and whether that place is ours to write.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum HooksDir {
    /// Inside THIS checkout. Ours to manage.
    Repo(std::path::PathBuf),
    /// Git resolves this checkout to a DIFFERENT repository — we are a copy nested
    /// inside someone else's working tree, and git found theirs by walking up.
    OtherRepo(std::path::PathBuf),
    /// A hooks directory outside this repository altogether: a global
    /// `core.hooksPath`, shared with every other repo on the machine.
    Shared(std::path::PathBuf),
}

/// The three answers one `git rev-parse` call gives about a checkout.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct GitPaths {
    /// The working tree root git resolves for us.
    pub toplevel: std::path::PathBuf,
    /// The shared git dir — the MAIN repo's `.git` when we are in a worktree.
    pub common_dir: std::path::PathBuf,
    /// Where git will actually look for hooks.
    pub hooks: std::path::PathBuf,
}

/// Parse `git rev-parse --path-format=absolute --show-toplevel --git-common-dir
/// --git-path hooks`, or `None` if git did not really answer.
///
/// ⚠⚠ `rev-parse` ECHOES an option it does not understand and still exits 0. This
/// has now bitten twice: once as `--git-path=hooks` written as a single token, and
/// once as `--path-format` on a git older than 2.31 (Ubuntu 20.04 ships 2.25). In
/// the second case git prints the flag verbatim AND the remaining paths RELATIVE,
/// so a naive read yields a plausible-looking relative path that `std::fs` then
/// resolves against the PROCESS working directory. Measured, both.
///
/// Hence two rules, and both are load-bearing:
/// - drop any line starting with `-` — that is an echoed option, not a path;
/// - require every remaining path to be ABSOLUTE, which is exactly what
///   `--path-format=absolute` guarantees and an old git silently does not.
#[must_use]
pub fn parse_git_paths(stdout: &str) -> Option<GitPaths> {
    let lines: Vec<&str> = stdout
        .lines()
        .map(str::trim)
        .filter(|l| !l.is_empty() && !l.starts_with('-'))
        .collect();
    let [toplevel, common_dir, hooks] = lines.as_slice() else {
        return None;
    };
    let paths = GitPaths {
        toplevel: std::path::PathBuf::from(toplevel),
        common_dir: std::path::PathBuf::from(common_dir),
        hooks: std::path::PathBuf::from(hooks),
    };
    if !paths.toplevel.is_absolute()
        || !paths.common_dir.is_absolute()
        || !paths.hooks.is_absolute()
    {
        return None;
    }
    Some(paths)
}

/// Decide whether the hooks directory git named is ours to write.
///
/// ⚠ CONTAINMENT, and it has two halves that are easy to conflate:
///
/// 1. **Is this even our checkout?** git finds a repository by walking UP. A copy of
///    this source with no `.git` of its own — vendored, extracted into someone's
///    tree — makes git answer with the ANCESTOR repository, whose hooks directory is
///    perfectly "inside its own git dir". Measured. Without the `toplevel` check we
///    would install our mesh guard and our `cargo fmt` into an unrelated project.
/// 2. **Is the hooks dir inside this repository?** A GLOBAL `core.hooksPath` points
///    at something shared with every repo on the machine; writing there is never
///    ours to do, and `uninstall` deleting there even less so.
///
/// ★ "Inside this repository" means inside the common git dir OR inside the working
/// tree. Both are required: a linked worktree's hooks live in the COMMON dir, which
/// is outside its working tree; and the widespread in-repo `core.hooksPath
/// .githooks` convention lives in the working tree, outside `.git`. An earlier
/// version checked only the git dir and refused `.githooks` with a message claiming
/// it was "shared with every other repo on this machine", which was simply false.
///
/// `repo_root` must already be canonicalised — see [`resolve_hooks_dir`].
#[must_use]
pub fn classify_hooks_dir(paths: &GitPaths, repo_root: &std::path::Path) -> HooksDir {
    if paths.toplevel != repo_root {
        return HooksDir::OtherRepo(paths.hooks.clone());
    }
    // Component-wise, so `/x/.github` never counts as inside `/x/.git`.
    if paths.hooks.starts_with(&paths.common_dir) || paths.hooks.starts_with(&paths.toplevel) {
        HooksDir::Repo(paths.hooks.clone())
    } else {
        HooksDir::Shared(paths.hooks.clone())
    }
}

/// Resolve the hooks directory for `repo_root`, or `None` if it cannot be known.
///
/// ONE implementation and ONE git invocation, used by `build.rs` and by the xtask
/// binary. Writing this out per-installer is what let the two drift, and asking git
/// twice let the two answers come from different config snapshots.
///
/// ⚠ FALLBACK. When git cannot answer but `<root>/.git` is a real directory, fall
/// back to `<root>/.git/hooks`. Installing nothing there is a regression: that path
/// is where git looks in the ordinary case, and a container with no `git` binary, a
/// repo tripping `dubious ownership`, or a git too old for `--path-format` is
/// exactly where the old code worked. A worktree's `.git` is a FILE, so it correctly
/// does not take this branch — there we genuinely do not know.
#[must_use]
pub fn resolve_hooks_dir(repo_root: &std::path::Path) -> Option<HooksDir> {
    // Canonicalise before comparing with git's answer: git resolves symlinks (macOS
    // `/var` → `/private/var`), so a raw comparison would call our own checkout a
    // different repository.
    let canonical = std::fs::canonicalize(repo_root).unwrap_or_else(|_| repo_root.to_path_buf());

    if let Some(out) = std::process::Command::new("git")
        .args([
            "rev-parse",
            "--path-format=absolute",
            "--show-toplevel",
            "--git-common-dir",
            "--git-path",
            "hooks",
        ])
        .current_dir(repo_root)
        .output()
        .ok()
        .filter(|o| o.status.success())
    {
        if let Some(paths) = String::from_utf8(out.stdout)
            .ok()
            .as_deref()
            .and_then(parse_git_paths)
        {
            return Some(classify_hooks_dir(&paths, &canonical));
        }
    }

    let dot_git = repo_root.join(".git");
    if dot_git.is_dir() {
        return Some(HooksDir::Repo(dot_git.join("hooks")));
    }
    None
}

/// Write `content` to `path` and make it executable, ATOMICALLY.
///
/// ⚠ Never `fs::write` a hook in place. git may be RUNNING that exact file: editing
/// `xtask/hooks/pre-commit` makes the pre-commit hook run clippy, which builds xtask,
/// whose build script then rewrites the hook the shell is still reading. `sh` reads a
/// script in blocks and seeks, so it resumes at a stale offset in new content — a
/// syntax error or, worse, a silently skipped check, blamed on the hook rather than
/// on the installer. Writing a sibling and renaming gives the running shell its own
/// inode and swaps the name in one step.
///
/// # Errors
/// If the temporary file cannot be written, made executable, or renamed into place.
pub fn write_hook_file(path: &std::path::Path, content: &str) -> std::io::Result<()> {
    // ⚠ UNIQUE per process and per call. A fixed name is not atomic against a
    // concurrent installer: rust-analyzer's `cargo check` uses a different target
    // dir than a terminal `cargo build`, so their build scripts take different cargo
    // locks and can run at the same time — one truncating the temp file the other is
    // mid-write, then renaming a half-written script over the live hook.
    static N: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
    let uniq = N.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    let tmp = path.with_extension(format!("cf-install-{}-{uniq}", std::process::id()));

    // Every failure past this point must take the temp file with it, or an orphan is
    // left in git's hooks directory that nothing ever cleans up — `uninstall`
    // iterates HOOKS by name and would not see it.
    let result = (|| -> std::io::Result<()> {
        std::fs::write(&tmp, content)?;
        #[cfg(unix)]
        {
            use std::os::unix::fs::PermissionsExt;
            // Mode on the TEMP file: the hook is then never visible under its real
            // name in a non-executable state, which git ignores without a word.
            std::fs::set_permissions(&tmp, std::fs::Permissions::from_mode(0o755))?;
        }
        std::fs::rename(&tmp, path)
    })();
    if result.is_err() {
        let _ = std::fs::remove_file(&tmp);
    }
    result
}

/// The hook's own title line, used to recognise a hook as ours.
///
/// DERIVED, never written down twice. A hardcoded marker is a second copy of a
/// string that lives in the hook file, and the moment the two disagree the installer
/// silently stops recognising its own hooks — which is this arc's own bug, one level
/// up.
///
/// # Panics
/// If the hook has no second line, or that line is not a `# CortenForge ...` title.
/// ⚠ VALIDATING is the point, not just extracting. An empty marker is the dangerous
/// case and it is silent: `existing.contains("")` is UNCONDITIONALLY TRUE, so a hook
/// whose line 2 was blank would make the installer treat EVERY hook as its own and
/// clobber a developer's foreign hook — the exact inverse of what the check is for.
/// A malformed hook file is a repository bug; failing loudly beats that outcome.
#[must_use]
pub fn title_of(hook: &str) -> &str {
    let title = hook
        .lines()
        .nth(1)
        .map(|l| l.trim_start_matches("# ").trim())
        .unwrap_or_default();
    assert!(
        title.starts_with("CortenForge"),
        "a hook in xtask/hooks/ has a malformed title line ({title:?}). Line 2 must \
         read `# CortenForge ...` — the installer derives its ownership marker from \
         it, and an empty marker matches every hook, including other people's."
    );
    title
}

/// What is at a hook path, classified against the hook we want there.
///
/// The installer needs the REASON, not just a yes/no. "Left alone because it is
/// someone else's" and "left alone because it is already ours and current" are the
/// same `false`, but they must not be the same SILENCE: the first means the
/// scan/mesh guard is not armed on that machine, and the developer has to hear it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HookState {
    /// Nothing at that path. Install ours.
    Missing,
    /// Ours by title, but out of date. Replacing this is what heals a checkout
    /// stranded by the old `cargo xtask setup`.
    OursStale,
    /// Ours and byte-identical. Nothing to do, and nothing to say.
    OursCurrent,
    /// Somebody else's hook. Never touch it.
    Foreign,
    /// A hook we cannot READ is not a hook we may overwrite.
    Unreadable,
}

impl HookState {
    /// Only the two states whose CONTENT is ours to write.
    #[must_use]
    pub fn should_replace(self) -> bool {
        matches!(self, HookState::Missing | HookState::OursStale)
    }

    /// Is this path ours to touch at all — including to REPAIR?
    ///
    /// Deliberately wider than [`Self::should_replace`]. A hook that is ours and
    /// already current still needs its executable bit checked: git ignores a
    /// non-executable hook and says nothing, and the previous build script discarded
    /// its `set_permissions` error — so "right text, mode 0644" is reachable in
    /// exactly the population this installer exists to heal, and `should_replace`
    /// alone would return before ever looking.
    #[must_use]
    pub fn is_ours_to_manage(self) -> bool {
        !matches!(self, HookState::Foreign | HookState::Unreadable)
    }
}

/// Classify whatever is sitting at the hook path.
///
/// `existing` is `None` when there is no hook, and `Some(Err(..))` when one is there
/// but its bytes are not UTF-8.
///
/// "Ours" is by TITLE line, which both historical stamps carry — that is what lets a
/// checkout stuck on the old `cargo xtask setup` hook heal itself on the next build.
#[must_use]
pub fn classify(existing: Option<Result<&str, ()>>, content: &str, marker: &str) -> HookState {
    match existing {
        None => HookState::Missing,
        Some(Ok(current)) if current.contains(marker) => {
            if current == content {
                HookState::OursCurrent
            } else {
                HookState::OursStale
            }
        }
        Some(Ok(_)) => HookState::Foreign,
        Some(Err(())) => HookState::Unreadable,
    }
}

#[cfg(test)]
mod tests {
    use super::{classify, title_of, HookState, COMMIT_MSG_HOOK, HOOKS, PRE_COMMIT_HOOK};

    const OURS: &str = "#!/bin/sh\n# CortenForge Pre-Commit Hook\necho hi\n";
    const MARKER: &str = "CortenForge Pre-Commit Hook";

    /// ⚠ THE PAIRING. Crossing the two entries of `HOOKS` installs the commit-msg
    /// script as `pre-commit`, which disarms the scan/mesh guard for everyone whose
    /// hooks come from `cargo build` — the majority path. Every other test in the
    /// workspace exercises `PRE_COMMIT_HOOK` directly rather than what the installer
    /// pairs it with, so this swap shipped green until this test existed. It was
    /// measured, not imagined.
    #[test]
    fn each_hook_is_paired_with_the_filename_git_runs_it_under() {
        let text = |name: &str| {
            HOOKS
                .iter()
                .find(|(n, _)| *n == name)
                .unwrap_or_else(|| panic!("HOOKS has no entry for {name}"))
                .1
        };
        let pre = text("pre-commit");
        let msg = text("commit-msg");

        assert!(
            pre.contains("CF_ALLOW_MESH"),
            "the text paired with `pre-commit` has no scan/mesh guard — the entries \
             are crossed, and the guard is silently disarmed on every fresh clone"
        );
        assert!(
            !msg.contains("CF_ALLOW_MESH"),
            "the text paired with `commit-msg` carries the guard — entries crossed"
        );
        assert_eq!(title_of(pre), "CortenForge Pre-Commit Hook");
        assert_eq!(title_of(msg), "CortenForge Commit Message Hook");
        assert_eq!(
            HOOKS.len(),
            2,
            "a hook was added or removed — give it a pairing assertion here too"
        );
    }

    /// The consts must be the ones the table names, not two independent copies.
    #[test]
    fn the_table_holds_the_hook_consts_themselves() {
        assert_eq!(HOOKS[0], ("pre-commit", PRE_COMMIT_HOOK));
        assert_eq!(HOOKS[1], ("commit-msg", COMMIT_MSG_HOOK));
    }

    #[test]
    fn title_is_taken_from_line_two_without_its_comment_marker() {
        assert_eq!(title_of(OURS), "CortenForge Pre-Commit Hook");
    }

    #[test]
    fn title_tolerates_trailing_space_and_a_carriage_return() {
        assert_eq!(
            title_of("#!/bin/sh\n# CortenForge Pre-Commit Hook  \r\nx\n"),
            "CortenForge Pre-Commit Hook"
        );
    }

    /// Every malformed shape must PANIC rather than yield a marker. The empty-string
    /// marker is the one that matters: `contains("")` is always true, so a silent
    /// empty marker turns the ownership check into "overwrite everything".
    #[test]
    fn every_malformed_title_panics_rather_than_yielding_a_marker() {
        for (label, hook) in [
            ("empty file", ""),
            ("one line only", "#!/bin/sh\n"),
            ("blank line 2", "#!/bin/sh\n\nx\n"),
            ("whitespace line 2", "#!/bin/sh\n   \nx\n"),
            ("comment marker only", "#!/bin/sh\n# \nx\n"),
            ("not a title", "#!/bin/sh\nset -e\nx\n"),
            ("wrong project", "#!/bin/sh\n# Somebody Else Hook\nx\n"),
        ] {
            assert!(
                std::panic::catch_unwind(|| title_of(hook)).is_err(),
                "{label}: title_of returned instead of panicking"
            );
        }
    }

    #[test]
    fn a_missing_hook_is_installed() {
        let state = classify(None, OURS, MARKER);
        assert_eq!(state, HookState::Missing);
        assert!(state.should_replace());
    }

    #[test]
    fn our_own_stale_hook_is_replaced_and_an_identical_one_is_not() {
        let stale = "#!/bin/sh\n# CortenForge Pre-Commit Hook\necho old\n";
        let stale = classify(Some(Ok(stale)), OURS, MARKER);
        assert_eq!(stale, HookState::OursStale);
        assert!(stale.should_replace(), "stale not replaced");

        let current = classify(Some(Ok(OURS)), OURS, MARKER);
        assert_eq!(current, HookState::OursCurrent);
        assert!(!current.should_replace(), "identical rewritten");
    }

    /// The legacy hook that #709 stranded: stamped by `cargo xtask setup`, missing
    /// the scan/mesh guard. It carries the same TITLE line, which is exactly why
    /// matching on the title heals it where matching on the old sentinel could not.
    #[test]
    fn the_legacy_setup_stamped_hook_is_recognised_and_healed() {
        let legacy =
            "#!/bin/sh\n# CortenForge Pre-Commit Hook\n# Installed by: cargo xtask setup\n";
        let state = classify(Some(Ok(legacy)), OURS, MARKER);
        assert_eq!(state, HookState::OursStale);
        assert!(state.should_replace());
    }

    /// A foreign hook must survive, whether or not we can read it. Both arms have
    /// been `true` at some point in this file's history, and each would silently
    /// destroy a developer's own hook.
    #[test]
    fn a_foreign_hook_is_never_touched_readable_or_not() {
        let theirs = "#!/bin/sh\n# husky\nnpm test\n";
        assert_eq!(classify(Some(Ok(theirs)), OURS, MARKER), HookState::Foreign);
        assert_eq!(classify(Some(Err(())), OURS, MARKER), HookState::Unreadable);
        assert!(!classify(Some(Ok(theirs)), OURS, MARKER).should_replace());
        assert!(!classify(Some(Err(())), OURS, MARKER).should_replace());
    }

    /// The two "we did not install" states must stay DISTINGUISHABLE from the
    /// up-to-date one. They are all `should_replace() == false`, but only the first
    /// two mean the guard is not armed, and the installer warns on exactly those.
    /// Collapsing them back into a bool is what made the omission silent.
    #[test]
    fn not_installed_is_distinguishable_from_already_current() {
        let theirs = "#!/bin/sh\n# husky\nnpm test\n";
        for state in [
            classify(Some(Ok(theirs)), OURS, MARKER),
            classify(Some(Err(())), OURS, MARKER),
        ] {
            assert!(!state.should_replace());
            assert_ne!(
                state,
                HookState::OursCurrent,
                "a hook we did not install reads as up to date — the warning is lost"
            );
        }
    }

    /// Parsing must reject what `rev-parse` does when it does NOT understand a flag.
    ///
    /// ⚠ It echoes the option and exits 0. Measured twice on real gits: once for
    /// `--git-path=hooks` as one token, once for `--path-format` on git < 2.31 —
    /// where the echoed line is followed by RELATIVE paths, which `std::fs` would
    /// resolve against the process working directory. Either shape must yield `None`
    /// so the caller falls back, not a plausible-looking wrong answer.
    #[test]
    fn a_rev_parse_that_echoed_an_option_or_answered_relatively_is_not_an_answer() {
        use super::parse_git_paths;

        let good = "/repo\n/repo/.git\n/repo/.git/hooks\n";
        assert!(
            parse_git_paths(good).is_some(),
            "the ordinary answer must parse"
        );

        assert!(
            parse_git_paths("--path-format=absolute\n/repo\n.git\n.git/hooks\n").is_none(),
            "an old git echoes the flag and answers RELATIVELY — measured; taking \
             those paths resolves them against the process cwd"
        );
        assert!(
            parse_git_paths("/repo\n.git\n.git/hooks\n").is_none(),
            "relative answers must be refused even with no echoed flag"
        );
        assert!(
            parse_git_paths("/repo\n/repo/.git\n").is_none(),
            "a short answer is not an answer"
        );
        assert!(parse_git_paths("").is_none(), "no output is not an answer");
    }

    /// CONTAINMENT has two independent halves, and each needs its own case.
    #[test]
    fn only_a_hooks_dir_inside_this_very_checkout_is_ours() {
        use super::{classify_hooks_dir, GitPaths, HooksDir};
        use std::path::{Path, PathBuf};

        let at = |top: &str, common: &str, hooks: &str| GitPaths {
            toplevel: PathBuf::from(top),
            common_dir: PathBuf::from(common),
            hooks: PathBuf::from(hooks),
        };
        let verdict = |p: &GitPaths, root: &str| classify_hooks_dir(p, Path::new(root));

        // The ordinary checkout.
        assert!(matches!(
            verdict(&at("/repo", "/repo/.git", "/repo/.git/hooks"), "/repo"),
            HooksDir::Repo(_)
        ));

        // ★ A LINKED WORKTREE: hooks live in the COMMON dir, which is NOT under this
        // working tree. This is the case the whole change exists for, and it is only
        // accepted because containment also allows the common dir.
        assert!(
            matches!(
                verdict(&at("/wt", "/main/.git", "/main/.git/hooks"), "/wt"),
                HooksDir::Repo(_)
            ),
            "a worktree's common-dir hooks must be accepted"
        );

        // ★ The in-repo `core.hooksPath .githooks` convention: inside the working
        // tree, OUTSIDE .git. Refusing it called a versioned, repo-private directory
        // "shared with every other repo on this machine", which was false.
        assert!(
            matches!(
                verdict(&at("/repo", "/repo/.git", "/repo/.githooks"), "/repo"),
                HooksDir::Repo(_)
            ),
            "an in-repo .githooks directory is ours"
        );

        // A GLOBAL core.hooksPath: neither in the git dir nor in the working tree.
        assert!(matches!(
            verdict(&at("/repo", "/repo/.git", "/home/dev/.githooks"), "/repo"),
            HooksDir::Shared(_)
        ));

        // ★ We are a copy nested in someone else's tree; git walked UP and answered
        // with THEIR repo, whose hooks are legitimately inside their own git dir.
        assert!(
            matches!(
                verdict(
                    &at("/outer", "/outer/.git", "/outer/.git/hooks"),
                    "/outer/vendor/cortenforge"
                ),
                HooksDir::OtherRepo(_)
            ),
            "an ancestor repository is not ours to install into"
        );

        // Component-wise comparison, not string prefix.
        assert!(matches!(
            verdict(&at("/repo", "/repo/.git", "/repo2/.git/hooks"), "/repo"),
            HooksDir::Shared(_)
        ));
    }

    /// The FALLBACK, both directions. When git cannot answer but `.git` is a real
    /// directory, installing nothing would be a regression: that path is where git
    /// looks in the ordinary case, and a container with no `git` binary or a repo
    /// tripping `dubious ownership` is exactly where the previous code worked.
    ///
    /// ⚠ And it must NOT guess when there is no `.git` at all — a vendored copy or a
    /// tarball has no hooks to install, and inventing a path would create junk.
    /// Measured: `git rev-parse` exits 128 in both of these directories.
    #[test]
    fn git_being_unable_to_answer_falls_back_only_when_dot_git_is_a_real_directory() {
        use super::{resolve_hooks_dir, HooksDir};

        let base = std::env::temp_dir().join(format!("cf-fallback-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&base);
        let with_git = base.join("with-git");
        let without = base.join("without-git");
        std::fs::create_dir_all(with_git.join(".git")).expect("temp dir");
        std::fs::create_dir_all(&without).expect("temp dir");

        let a = resolve_hooks_dir(&with_git);
        let b = resolve_hooks_dir(&without);
        let _ = std::fs::remove_dir_all(&base);

        assert_eq!(
            a,
            Some(HooksDir::Repo(with_git.join(".git").join("hooks"))),
            "a real .git directory git cannot speak for must still get hooks"
        );
        assert_eq!(b, None, "no .git at all must not be guessed at");
    }

    /// The write must be atomic and must leave nothing behind.
    ///
    /// ⚠ `fs::write` truncates in place, and git may be RUNNING that exact file:
    /// editing `xtask/hooks/pre-commit` makes the pre-commit hook run clippy, which
    /// builds xtask, whose build script rewrites the hook the shell is still reading.
    /// The race itself is not reproducible in a unit test, so this pins the two
    /// observable consequences of doing it right: the content lands, and no
    /// `.cf-install-tmp` sibling survives.
    #[test]
    fn writing_a_hook_replaces_it_and_leaves_no_temporary_behind() {
        use super::write_hook_file;

        let dir = std::env::temp_dir().join(format!("cf-atomic-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).expect("temp dir");
        let path = dir.join("pre-commit");

        std::fs::write(&path, "old\n").expect("seed");
        write_hook_file(&path, OURS).expect("write");

        let content = std::fs::read_to_string(&path).expect("read back");
        let leftovers: Vec<String> = std::fs::read_dir(&dir)
            .expect("read dir")
            .filter_map(|e| e.ok())
            .map(|e| e.file_name().to_string_lossy().into_owned())
            .filter(|n| n != "pre-commit")
            .collect();
        #[cfg(unix)]
        let mode = {
            use std::os::unix::fs::PermissionsExt;
            std::fs::metadata(&path).expect("meta").permissions().mode()
        };
        let _ = std::fs::remove_dir_all(&dir);

        assert_eq!(content, OURS, "the replacement content did not land");
        assert!(
            leftovers.is_empty(),
            "a temporary file survived the install: {leftovers:?}"
        );
        #[cfg(unix)]
        assert!(
            mode & 0o111 != 0,
            "the installed hook is not executable ({mode:o}); git ignores it silently"
        );
    }

    /// Ours-and-current is NOT ours-to-rewrite, but it IS ours to repair. If these
    /// two ever collapse back into one predicate, a correct-but-unexecutable hook
    /// stops being fixable and git goes on silently ignoring it.
    #[test]
    fn an_up_to_date_hook_is_still_ours_to_repair() {
        let current = classify(Some(Ok(OURS)), OURS, MARKER);
        assert!(!current.should_replace(), "current text rewritten");
        assert!(
            current.is_ours_to_manage(),
            "current hook is not repairable"
        );

        let theirs = "#!/bin/sh\n# husky\nnpm test\n";
        assert!(!classify(Some(Ok(theirs)), OURS, MARKER).is_ours_to_manage());
        assert!(!classify(Some(Err(())), OURS, MARKER).is_ours_to_manage());
        assert!(classify(None, OURS, MARKER).is_ours_to_manage());
    }

    /// The failure mode an empty marker would cause, pinned directly: if a marker of
    /// "" ever reached this function, EVERY hook would look like ours.
    #[test]
    fn an_empty_marker_would_match_a_foreign_hook_which_is_why_title_of_panics() {
        let theirs = "#!/bin/sh\n# husky\nnpm test\n";
        assert!(
            classify(Some(Ok(theirs)), OURS, "").should_replace(),
            "if this ever stops being true the empty-marker hazard changed shape"
        );
    }
}
