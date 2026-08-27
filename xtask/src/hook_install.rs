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
/// Hence ONE rule, and it is enough: every path must be ABSOLUTE, which is exactly
/// what `--path-format=absolute` guarantees and an old git silently does not. An
/// echoed flag is never an absolute path, and it also makes a fourth line, so both
/// measured shapes are refused twice over.
///
/// ⚠ There used to be a second rule — drop any line starting with `-` — described
/// here as belt and braces. It was not free: `core.hooksPath = -hooks` is a legal
/// setting whose answer that filter ate. Deleted. When a defensive rule cannot be
/// shown to catch anything the primary rule misses, it is not free insurance; it is
/// an untested code path with its own failure mode.
#[must_use]
pub fn parse_git_paths(stdout: &str) -> Option<GitPaths> {
    let [toplevel, common_dir, hooks] = three_answers(stdout)?;
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

/// The same three answers from a git too old to know `--path-format=absolute`.
///
/// Such a git returns the git dir and the hooks path RELATIVE to the directory it
/// ran in, so `base` — that same directory — is what they hang off. `Path::join`
/// returns an already-absolute argument unchanged, which is what `--show-toplevel`
/// gives even on old git, so one rule covers the mixed shape git actually emits.
///
/// ⚠ This exists so a `Shared` hooks directory is still RECOGNISED on old git.
/// Without it, resolution fell through to the `.git/hooks` guess, and a developer
/// on git 2.25 with a global `core.hooksPath` was told "Installed" while git read
/// its hooks somewhere else entirely — a false success, which is the failure mode
/// this whole arc exists to remove.
///
/// ⚠⚠ JOINING IS NOT ENOUGH — it must also NORMALISE. `--path-format=absolute`
/// resolves `..` and symlinks; a bare `base.join(..)` does neither, and
/// `Path::starts_with` is component-wise, so a surviving `..` walks straight
/// through containment. Measured: with `core.hooksPath = ../shared-hooks`, real git
/// answers `Shared(<lab>/shared-hooks)` and is refused, while the joined form
/// `<repo>/../shared-hooks` was accepted as `Repo` and written to — on exactly the
/// old-git population this function was added to serve. An in-repo symlink pointing
/// out of the tree escaped the same way.
#[must_use]
pub fn parse_git_paths_rooted_at(stdout: &str, base: &std::path::Path) -> Option<GitPaths> {
    let [toplevel, common_dir, hooks] = three_answers(stdout)?;
    Some(GitPaths {
        toplevel: absolutize(base, toplevel),
        common_dir: absolutize(base, common_dir),
        hooks: absolutize(base, hooks),
    })
}

/// Join `answer` onto `base` and resolve it the way `--path-format=absolute` would.
///
/// The hooks directory need not exist yet, so `canonicalize` can fail outright.
/// Resolve the longest prefix that DOES exist and re-attach the rest: that is enough
/// to defeat both escapes, because a `..` or a symlink can only take us somewhere
/// that already exists.
fn absolutize(base: &std::path::Path, answer: &str) -> std::path::PathBuf {
    let joined = base.join(answer);
    let mut suffix: Vec<std::ffi::OsString> = Vec::new();
    let mut probe = joined.clone();
    loop {
        if let Ok(real) = std::fs::canonicalize(&probe) {
            let mut out = real;
            for part in suffix.iter().rev() {
                out.push(part);
            }
            return out;
        }
        let Some(name) = probe.file_name().map(std::ffi::OsStr::to_os_string) else {
            // Ran out of path without finding anything real. Nothing to resolve
            // against, so hand back the lexical join and let containment judge it.
            return joined;
        };
        suffix.push(name);
        if !probe.pop() {
            return joined;
        }
    }
}

/// Exactly three answer lines, or nothing.
///
/// ⚠ Only `\r` is stripped, NOT surrounding whitespace: a directory whose name ends
/// in a space is legal on every filesystem we support, and trimming it made git's
/// answer differ from the path we asked about, so a repository was classified as
/// somebody ELSE's — the one build.rs arm that returns completely silently.
///
/// ⚠⚠ AND NOTHING IS DROPPED FOR STARTING WITH `-`. That filter was here to skip an
/// echoed option, and it cost more than it bought: `core.hooksPath = -hooks` is a
/// legal setting whose answer the filter ATE, leaving two lines, no answer, and the
/// `.git/hooks` fallback reporting a successful install into a directory git never
/// reads — the precise false success the second ask exists to prevent. It was also
/// unnecessary. An echoed option can only appear in the FIRST ask, which is the one
/// that requires every path to be absolute, and an echoed flag is never absolute; it
/// simply makes a fourth line, which the length check below refuses on its own. The
/// second ask passes no option old enough git could fail to know.
fn three_answers(stdout: &str) -> Option<[&str; 3]> {
    let lines: Vec<&str> = stdout
        .lines()
        .map(|l| l.trim_end_matches('\r'))
        .filter(|l| !l.is_empty())
        .collect();
    let [toplevel, common_dir, hooks] = lines.as_slice() else {
        return None;
    };
    Some([toplevel, common_dir, hooks])
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
    // `starts_with` is COMPONENT-wise, not textual, so a sibling whose name merely
    // begins with the same characters (`/x/.github` against `/x/.git`) is not
    // "inside" it. Both arms rely on that.
    //
    // ★ The working-tree arm accepts the repository ROOT itself (`core.hooksPath=.`,
    // where git's answer equals the toplevel). That is deliberate: it is where git
    // genuinely reads hooks from under that setting, and refusing would leave the
    // guard unarmed for someone who asked for it explicitly. It does mean the two
    // hooks appear as untracked files in the root — pinned by test, so a future
    // reader meets a decision rather than a surprise.
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
/// ⚠ FALLBACK, and it is the LAST resort, not the second. When git cannot answer at
/// all but `<root>/.git` is a real directory, fall back to `<root>/.git/hooks`.
/// Installing nothing there is a regression: that path is where git looks in the
/// ordinary case, and a container with no `git` binary or a repo tripping `dubious
/// ownership` is exactly where the old code worked. A worktree's `.git` is a FILE,
/// so it correctly does not take this branch — there we genuinely do not know.
///
/// ⚠⚠ The fallback must never override an answer git COULD have given. It used to:
/// on a git too old for `--path-format` the first ask returns unusable output, and
/// jumping straight to `.git/hooks` guessed WRONG for anyone with a `core.hooksPath`
/// — reporting "Installed" for a file git does not read. Hence the second ask, which
/// that git does understand. Only when both fail is the answer genuinely unknown.
#[must_use]
pub fn resolve_hooks_dir(repo_root: &std::path::Path) -> Option<HooksDir> {
    // Canonicalise before comparing with git's answer: git resolves symlinks (macOS
    // `/var` → `/private/var`), so a raw comparison would call our own checkout a
    // different repository.
    let canonical = std::fs::canonicalize(repo_root).unwrap_or_else(|_| repo_root.to_path_buf());

    // Modern git: every answer absolute, so a relative one is proof of a git that
    // did not understand the question.
    let modern = ask_git(
        repo_root,
        &[
            "rev-parse",
            "--path-format=absolute",
            "--show-toplevel",
            "--git-common-dir",
            "--git-path",
            "hooks",
        ],
    );
    // git < 2.31 (Ubuntu 20.04 ships 2.25): same question without the flag it lacks.
    // ⚠ Asked unconditionally rather than only when the first fails, so that the
    // ORDER between them is decided in one pure place instead of by control flow
    // nothing can test. The cost is one extra `rev-parse` — microseconds, once per
    // build — and it buys a decision a unit test can reach.
    let legacy = ask_git(
        repo_root,
        &[
            "rev-parse",
            "--show-toplevel",
            "--git-common-dir",
            "--git-path",
            "hooks",
        ],
    );

    resolve_from_answers(
        modern.as_deref(),
        legacy.as_deref(),
        &canonical,
        repo_root.join(".git").is_dir(),
    )
}

/// Pick between git's two answers and the last-resort guess. Pure, so it is tested.
///
/// ⚠⚠ THE ORDER OF THESE THREE RUNGS IS ITSELF A DECISION, and while it lived only
/// as control flow inside [`resolve_hooks_dir`] no test could see it: a mutation
/// survey deleted the entire `legacy` rung and the whole suite stayed green, because
/// on a modern git the first rung answers and the second never runs. Under a git-2.25
/// shim the same mutant returned `Repo(<root>/.git/hooks)` where the real answer is
/// `Shared(~/.githooks)` — a false "Installed" for a file git never reads.
///
/// The rungs, strongest first:
/// 1. a modern answer, every path absolute;
/// 2. an old-git answer, relative paths resolved against the repo root;
/// 3. `<root>/.git/hooks`, and ONLY when git could not be asked at all — a guess
///    that overrides a knowable answer is how the false success got here.
#[must_use]
pub fn resolve_from_answers(
    modern: Option<&str>,
    legacy: Option<&str>,
    repo_root: &std::path::Path,
    dot_git_is_dir: bool,
) -> Option<HooksDir> {
    if let Some(paths) = modern.and_then(parse_git_paths) {
        return Some(classify_hooks_dir(&paths, repo_root));
    }
    if let Some(paths) = legacy.and_then(|out| parse_git_paths_rooted_at(out, repo_root)) {
        return Some(classify_hooks_dir(&paths, repo_root));
    }
    if dot_git_is_dir {
        return Some(HooksDir::Repo(repo_root.join(".git").join("hooks")));
    }
    None
}

/// Run `git` in `repo_root` and return its stdout, or `None` if it did not succeed.
///
/// ⚠⚠ THE ENVIRONMENT IS PART OF THE QUESTION. With `GIT_DIR` set and no
/// `GIT_WORK_TREE`, git skips repository discovery entirely and calls the directory
/// we hand it the top level — so `--show-toplevel` echoes our own path back and the
/// "is this our checkout?" check in [`classify_hooks_dir`] passes BY CONSTRUCTION,
/// while `--git-common-dir` and `--git-path` answer for the OTHER repository. The
/// verdict is `Repo`, pointing at somebody else's hooks directory. Measured.
///
/// That shape is reachable: git exports an absolute `GIT_DIR` to every hook it runs
/// in a linked worktree, and `git submodule foreach` exports one too — so a hook or
/// a `foreach` that builds CortenForge elsewhere inherits it. Clearing these makes
/// the answer depend only on the directory, which is the whole premise of asking.
fn ask_git(repo_root: &std::path::Path, args: &[&str]) -> Option<String> {
    let out = git_command(repo_root, args)
        .output()
        .ok()
        .filter(|o| o.status.success())?;
    String::from_utf8(out.stdout).ok()
}

/// Every git invocation this module makes, including the environment it clears.
///
/// Split out so a test can inspect it. The end-to-end version of that test —
/// setting `GIT_DIR` and calling `resolve_hooks_dir` — cannot be written: the
/// environment is per-PROCESS, libtest runs these in parallel, and it broke four
/// unrelated git tests with `fatal: not in a git directory`. Asserting on the
/// command we build tests the same claim without a global side effect.
pub const GIT_VARS_CLEARED: [&str; 4] = [
    "GIT_DIR",
    "GIT_COMMON_DIR",
    "GIT_WORK_TREE",
    "GIT_INDEX_FILE",
];

#[must_use]
pub fn git_command(repo_root: &std::path::Path, args: &[&str]) -> std::process::Command {
    let mut cmd = std::process::Command::new("git");
    cmd.args(args).current_dir(repo_root);
    for var in GIT_VARS_CLEARED {
        cmd.env_remove(var);
    }
    cmd
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

/// Will git actually RUN the hook at `path`?
///
/// git ignores a non-executable hook without a word — no error, no warning, the
/// guard simply never runs. Both installers must therefore repair the bit even when
/// the TEXT is already current, and both must decide that the same way.
#[cfg(unix)]
#[must_use]
pub fn is_executable(path: &std::path::Path) -> bool {
    use std::os::unix::fs::PermissionsExt;
    std::fs::metadata(path).is_ok_and(|m| m.permissions().mode() & 0o111 != 0)
}

/// Repair the executable bit in place, WITHOUT rewriting the file.
///
/// ⚠ In place is the point. Laying the file down again looks equivalent and is not:
/// `write_hook_file` renames over the path, which replaces a SYMLINK with a regular
/// file. A developer whose `.git/hooks/pre-commit` is a link to `xtask/hooks/`
/// loses the link, silently, and the hook stops tracking its source. Measured.
///
/// # Errors
/// If the mode cannot be changed.
#[cfg(unix)]
pub fn make_executable(path: &std::path::Path) -> std::io::Result<()> {
    use std::os::unix::fs::PermissionsExt;
    std::fs::set_permissions(path, std::fs::Permissions::from_mode(0o755))
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

/// Read whatever is at `path`, in the shape [`classify`] wants.
///
/// `None` means nothing is there. `Some(Err(()))` means something IS there that we
/// cannot read as text, which [`HookState::Unreadable`] refuses to touch.
///
/// ⚠ PRESENCE IS `symlink_metadata`, NOT `exists()`. `Path::exists` FOLLOWS the
/// link, so a hook symlinked to a shared hooks repo on an unmounted volume — a
/// dangling link — answers `false`, classifies as `Missing`, and gets replaced by a
/// regular file. Measured: the developer's link is destroyed, silently, and the
/// installer reports `Installed`. A live symlink was already safe (the read follows
/// it and sees foreign text); only the dangling case bypassed the invariant that we
/// never touch a hook that is not ours.
#[must_use]
pub fn read_existing_hook(path: &std::path::Path) -> Option<Result<String, ()>> {
    path.symlink_metadata().ok()?;
    Some(std::fs::read_to_string(path).map_err(|_| ()))
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
        for state in [
            classify(Some(Ok(theirs)), OURS, MARKER),
            classify(Some(Err(())), OURS, MARKER),
        ] {
            assert!(!state.should_replace(), "its CONTENT is not ours to write");
            // The wider predicate, and the one that actually gates the installers:
            // `should_replace` alone would still let the chmod repair touch it.
            assert!(
                !state.is_ours_to_manage(),
                "a hook that is not ours must not be repaired either — that is a \
                 write to somebody else's file, just a smaller one"
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
            verdict(
                &at("/repo", "/repo/.git", "/nx-cf-elsewhere/.githooks"),
                "/repo"
            ),
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

        // ★ THE DEGENERATE SHAPE: `core.hooksPath=.`, where git's hooks answer IS
        // the working tree root. An earlier revision asserted this was refused; the
        // revision that widened containment to the working tree deleted that
        // assertion and replaced it with nothing, so the behaviour flipped with no
        // test either way. It is now a DECISION, pinned: accepted, because that is
        // genuinely where git reads hooks under that setting, and refusing leaves
        // the guard unarmed for someone who configured it deliberately. The visible
        // cost is two untracked files in the repository root.
        assert!(
            matches!(
                verdict(&at("/repo", "/repo/.git", "/repo"), "/repo"),
                HooksDir::Repo(_)
            ),
            "core.hooksPath=. puts hooks in the repo root, and git reads them there"
        );

        // ⚠ THE PAYLOAD, not just the variant. Every assertion above is
        // `matches!(.., Repo(_))`, which cannot see WHICH directory came back —
        // returning the toplevel, or the common dir, for every input survives all
        // of them. The verdict and the path are two separate claims.
        assert_eq!(
            verdict(&at("/wt", "/main/.git", "/main/.git/hooks"), "/wt"),
            HooksDir::Repo(PathBuf::from("/main/.git/hooks")),
            "the hooks path git named must be the path handed back, unaltered"
        );
        assert_eq!(
            verdict(
                &at("/repo", "/repo/.git", "/nx-cf-elsewhere/.githooks"),
                "/repo"
            ),
            HooksDir::Shared(PathBuf::from("/nx-cf-elsewhere/.githooks")),
            "a refusal still has to name the directory it is refusing"
        );
        assert_eq!(
            verdict(
                &at("/outer", "/outer/.git", "/outer/.git/hooks"),
                "/outer/vendor/cortenforge"
            ),
            HooksDir::OtherRepo(PathBuf::from("/outer/.git/hooks")),
        );
    }

    /// A git too old for `--path-format` answers RELATIVELY, and the second ask
    /// exists to use that answer rather than throw it away.
    ///
    /// ⚠ This is what stops the `.git/hooks` fallback from overriding a knowable
    /// answer. Without it, a developer on git 2.25 with a `core.hooksPath` was told
    /// "Installed" for a file git does not read — a false success, which is the
    /// failure mode the whole arc exists to remove.
    #[test]
    fn an_old_gits_relative_answer_is_usable_rather_than_discarded() {
        use super::{classify_hooks_dir, parse_git_paths_rooted_at, HooksDir};
        use std::path::{Path, PathBuf};

        let base = Path::new("/repo");

        // The exact mixed shape measured from a git without `--path-format`:
        // toplevel absolute, git dir and hooks relative to the directory it ran in.
        let paths = parse_git_paths_rooted_at("/repo\n.git\n.git/hooks\n", base)
            .expect("relative answers are usable once rooted");
        assert_eq!(paths.common_dir, PathBuf::from("/repo/.git"));
        assert_eq!(paths.hooks, PathBuf::from("/repo/.git/hooks"));
        assert_eq!(
            paths.toplevel,
            PathBuf::from("/repo"),
            "join must leave an already-absolute answer alone"
        );

        assert!(
            parse_git_paths_rooted_at("/repo\n.git\n", base).is_none(),
            "a short answer is not an answer here either"
        );

        // ★★ THE ESCAPE. `--path-format=absolute` resolves `..` and symlinks; a bare
        // join does neither, and `starts_with` is component-wise, so a surviving `..`
        // walked straight through containment. Measured on real git: with
        // `core.hooksPath = ../shared-hooks` the modern ask says `Shared` and refuses,
        // while the joined `<repo>/../shared-hooks` was accepted as `Repo` and WRITTEN
        // TO — on precisely the old-git population this parser was added to serve.
        //
        // Real directories, because resolving `..` past a symlink is a filesystem
        // question and a lexical answer is the wrong one.
        let lab = std::env::temp_dir().join(format!("cf-escape-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&lab);
        let repo = lab.join("repo");
        std::fs::create_dir_all(repo.join(".git")).expect("temp dir");
        std::fs::create_dir_all(lab.join("shared-hooks")).expect("temp dir");
        let repo = std::fs::canonicalize(&repo).expect("canonicalize");
        let outside = std::fs::canonicalize(lab.join("shared-hooks")).expect("canonicalize");

        let escaped = parse_git_paths_rooted_at(
            &format!("{}\n.git\n../shared-hooks\n", repo.display()),
            &repo,
        )
        .expect("old git emits exactly this shape");
        let verdict = classify_hooks_dir(&escaped, &repo);

        // POSITIVE CONTROL: a relative answer that stays INSIDE must still be ours,
        // or "refuses everything" would pass the assertion above.
        let inside =
            parse_git_paths_rooted_at(&format!("{}\n.git\n.git/hooks\n", repo.display()), &repo)
                .expect("old git emits exactly this shape");
        let inside_verdict = classify_hooks_dir(&inside, &repo);
        let _ = std::fs::remove_dir_all(&lab);

        assert_eq!(
            verdict,
            HooksDir::Shared(outside),
            "`..` in an old git's answer escaped the checkout and was accepted"
        );
        assert_eq!(
            inside_verdict,
            HooksDir::Repo(repo.join(".git").join("hooks")),
            "POSITIVE CONTROL: an old git's ordinary answer is still ours"
        );
    }

    /// The three rungs, in order, from the shapes each git actually emits.
    ///
    /// ⚠⚠ THE ORDERING HAD NO GATE. A mutation survey deleted the entire old-git
    /// rung and all 373 tests stayed green — on a modern git the first rung answers
    /// and the second never runs, so nothing could see it go. Under a git-2.25 shim
    /// the same mutant turned `Shared(~/.githooks)` into `Repo(.git/hooks)`: the
    /// false "Installed" for a file git never reads, which is this arc's whole
    /// subject. Deleting BOTH rungs was killed; deleting the second was not.
    #[test]
    fn the_strongest_available_answer_wins_and_the_guess_is_last() {
        use super::{resolve_from_answers, HooksDir};
        use std::path::{Path, PathBuf};

        let root = Path::new("/repo");
        let modern = "/repo\n/repo/.git\n/nx-cf-elsewhere/.githooks\n";
        // What a git < 2.31 emits for the SAME repository: the flag echoed back, and
        // the paths it does answer given relative. Measured.
        let legacy_echo = "--path-format=absolute\n/repo\n.git\n.git/hooks\n";
        let legacy = "/repo\n.git\n.git/hooks\n";

        // Rung 1 wins outright, even though rung 2 is available and disagrees.
        assert_eq!(
            resolve_from_answers(Some(modern), Some(legacy), root, true),
            Some(HooksDir::Shared(PathBuf::from(
                "/nx-cf-elsewhere/.githooks"
            ))),
            "a modern answer must not be overridden by anything below it"
        );

        // Rung 2 is used when rung 1 is unusable — the echoed-flag shape, which the
        // absolute rule refuses. THIS is the rung the survey deleted invisibly.
        assert_eq!(
            resolve_from_answers(Some(legacy_echo), Some(legacy), root, true),
            Some(HooksDir::Repo(PathBuf::from("/repo/.git/hooks"))),
            "an old git's answer must be used rather than discarded"
        );

        // ★ And the guess must NEVER override it. With `dot_git_is_dir` true, the
        // fallback would happily return `/repo/.git/hooks`; the old-git answer says
        // the hooks are elsewhere, and that answer has to win or the developer is
        // told "Installed" about a file git does not read.
        let legacy_elsewhere = "/repo\n.git\n/nx-cf-elsewhere/.githooks\n";
        assert_eq!(
            resolve_from_answers(Some(legacy_echo), Some(legacy_elsewhere), root, true),
            Some(HooksDir::Shared(PathBuf::from(
                "/nx-cf-elsewhere/.githooks"
            ))),
            "the .git/hooks guess overrode a knowable answer — the false success"
        );

        // Rung 3 only when git said nothing usable at all.
        assert_eq!(
            resolve_from_answers(None, None, root, true),
            Some(HooksDir::Repo(PathBuf::from("/repo/.git/hooks"))),
            "installing nothing when git cannot be asked is a regression"
        );
        assert_eq!(
            resolve_from_answers(None, None, root, false),
            None,
            "with no .git directory there is nothing to guess at"
        );
    }

    /// A directory whose name ends in a space is legal, and trimming broke it.
    #[test]
    fn a_path_is_not_trimmed_but_a_carriage_return_is() {
        use super::parse_git_paths;
        use std::path::PathBuf;

        let paths = parse_git_paths("/repo/trail \n/repo/trail /.git\n/repo/trail /.git/hooks\n")
            .expect("trailing spaces are part of the path");
        assert_eq!(
            paths.toplevel,
            PathBuf::from("/repo/trail "),
            "trimming made git's answer differ from the directory we asked about, \
             so the repository was classified as somebody else's — and that arm of \
             build.rs is the one that returns completely silently"
        );

        let crlf = parse_git_paths("/repo\r\n/repo/.git\r\n/repo/.git/hooks\r\n")
            .expect("a CR is line noise, not part of the path");
        assert_eq!(crlf.toplevel, PathBuf::from("/repo"));
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
        // Resolution works in canonicalised paths throughout, so the expectation
        // must too — on macOS the temp dir is reached through /var -> /private/var.
        let expected = std::fs::canonicalize(&with_git)
            .expect("canonicalize")
            .join(".git")
            .join("hooks");
        let _ = std::fs::remove_dir_all(&base);

        assert_eq!(
            a,
            Some(HooksDir::Repo(expected)),
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
        // ⚠ THE INODE IS THE CLAIM. Content, mode and no-leftovers are all equally
        // true of a plain `fs::write` in place — the very thing this function's doc
        // spends nine lines forbidding, because git may be executing that file and
        // `sh` resumes at a stale offset in new content. A rename gives the running
        // shell its own inode; an in-place write does not. Without this the whole
        // atomicity argument was untested prose.
        #[cfg(unix)]
        let ino_before = {
            use std::os::unix::fs::MetadataExt;
            std::fs::metadata(&path).expect("meta").ino()
        };
        write_hook_file(&path, OURS).expect("write");
        #[cfg(unix)]
        let ino_after = {
            use std::os::unix::fs::MetadataExt;
            std::fs::metadata(&path).expect("meta").ino()
        };

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
        #[cfg(unix)]
        assert_ne!(
            ino_before, ino_after,
            "the hook was written IN PLACE, not renamed over: a shell already \
             executing it would resume at a stale offset in the new bytes"
        );
    }

    /// Every failure past the temp write has to take the temp file with it.
    ///
    /// ⚠ An orphan here is invisible: `uninstall` iterates `HOOKS` by name and would
    /// never see `pre-commit.cf-install-1234-0`, so it sits in git's hooks directory
    /// forever. Nothing exercised this arm — `if false { .. }` around the cleanup
    /// survived the whole suite.
    #[test]
    fn a_failed_install_leaves_no_orphan_behind() {
        use super::write_hook_file;

        let dir = std::env::temp_dir().join(format!("cf-orphan-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).expect("temp dir");
        let path = dir.join("pre-commit");

        // A non-empty DIRECTORY where the hook belongs: the temp write and the chmod
        // both succeed, and `rename` then fails with ENOTEMPTY/EISDIR. Measured —
        // this is the one shape that reaches the cleanup without stubbing anything.
        std::fs::create_dir_all(path.join("occupied")).expect("blocker");

        let result = write_hook_file(&path, OURS);

        let leftovers: Vec<String> = std::fs::read_dir(&dir)
            .expect("read dir")
            .filter_map(|e| e.ok())
            .map(|e| e.file_name().to_string_lossy().into_owned())
            .filter(|n| n != "pre-commit")
            .collect();
        let _ = std::fs::remove_dir_all(&dir);

        assert!(
            result.is_err(),
            "renaming over a non-empty directory must fail, or this test proves nothing"
        );
        assert!(
            leftovers.is_empty(),
            "the temporary file outlived a failed install: {leftovers:?}"
        );
    }

    /// A dangling symlink is a hook that is PRESENT and not ours.
    #[test]
    #[cfg(unix)]
    fn a_dangling_symlink_is_never_mistaken_for_an_absent_hook() {
        use super::{classify, read_existing_hook, HookState};

        let dir = std::env::temp_dir().join(format!("cf-dangle-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).expect("temp dir");
        let path = dir.join("pre-commit");
        std::os::unix::fs::symlink(dir.join("no-such-volume/pre-commit"), &path).expect("symlink");

        let existing = read_existing_hook(&path);
        let state = classify(
            existing.as_ref().map(|r| r.as_deref().map_err(|_| ())),
            OURS,
            MARKER,
        );
        // POSITIVE CONTROL for the rig: the naive test really would have said absent.
        let exists_says_absent = !path.exists();
        let _ = std::fs::remove_file(&path);
        let _ = std::fs::remove_dir_all(&dir);

        assert!(
            exists_says_absent,
            "NEGATIVE CONTROL BROKEN: Path::exists no longer follows the link, so \
             this test can no longer show why symlink_metadata is required"
        );
        assert_eq!(
            state,
            HookState::Unreadable,
            "a developer's link to a shared hooks repo on an unmounted volume read \
             as Missing and was replaced by a regular file — silently, reported as \
             a successful install"
        );
        assert!(
            !state.should_replace(),
            "and so it must never be written over"
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
