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
//
// ⚠ `pub` HERE IS NOT PUBLIC API. This file is `include!`d into a build script (whose
// crate root it becomes) and `mod`'d privately into the xtask binary, so `pub` only
// ever means "reachable from `setup.rs`". Anything used solely inside this module —
// the parsers, `classify_hooks_dir`, `resolve_from_answers` — is private, and the
// child `mod tests` reaches it through `super::`. Adding `pub` to something only the
// tests use would be documenting a boundary that does not exist.

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
struct GitPaths {
    /// The working tree root git resolves for us.
    pub toplevel: std::path::PathBuf,
    /// The shared git dir — the MAIN repo's `.git` when we are in a worktree.
    pub common_dir: std::path::PathBuf,
    /// Where git will actually look for hooks.
    pub hooks: std::path::PathBuf,
}

/// Parse `git rev-parse --show-toplevel --git-common-dir --git-path hooks`, or
/// `None` if git did not really answer.
///
/// `--show-toplevel` is absolute on every git; the other two come back RELATIVE to
/// the directory the command ran in, which is `base`. `absolutize` resolves them the
/// way `--path-format=absolute` would.
///
/// ⚠ NO `--path-format=absolute`, and that is a deliberate simplification. Asking
/// for it made git's ANSWER easier and the CODE harder: a git older than 2.31
/// (Ubuntu 20.04 ships 2.25) does not know the flag, ECHOES it, and exits 0 — so
/// there had to be a second ask without it, a second parser, an absolute-path rule
/// to tell the two apart, and a documented order between their answers. Asking a
/// question every supported git understands removes all four.
///
/// ⚠ THE PRICE is that `absolutize` now resolves paths for EVERY git, where before
/// git's own `--path-format=absolute` did it whenever it could. That is not free:
/// review found two containment escapes in this walk that the two-ask design never
/// reached, because on a modern git it never ran. Both are fixed and gated
/// (`a_dangling_symlink_is_followed_the_way_git_follows_it`). Treat this walk as
/// load-bearing for everyone now, not as a legacy path.
///
/// ⚠ A differential over ~38 repository shapes found the two designs identical, but
/// that harness is not in the tree, so this comment is the only record of it — read
/// it as motivation, not as evidence. What is EVIDENCE is the tests below, whose
/// oracle is git's own `--path-format=absolute` answer recorded from real runs.
///
/// ⚠⚠ THE COUNT RULE IN [`three_answers`] IS THE WHOLE DEFENCE against a garbled
/// answer, and that is measured, not assumed. `rev-parse` echoes an option it does
/// not understand as an EXTRA LINE IN POSITION — it never replaces an answer — so
/// any echo makes four lines and is refused on count before anything inspects the
/// paths.
///
/// ⚠ There was briefly a `toplevel.is_absolute()` rule here as well, justified by a
/// shift that the count rule already catches. Nothing real reached it: its two
/// negative controls fed hand-built THREE-line strings that no git emits. That makes
/// it the fourth defensive rule on this arc to be deleted for the same reason —
/// after a `-` prefix filter (ate `core.hooksPath = -hooks`), an exact-match option
/// filter (ate `--git-path`), and an empty-line filter (defeated this very count
/// rule). ★ The pattern is worth naming: every one of them was added to catch a
/// shape someone REASONED about, and every one was measured afterwards to catch
/// nothing while three of the four ate a legal answer.
///
/// ⚠⚠ THE ECHO HAZARD IS NOT GONE, it is just no longer reachable by a flag WE
/// choose. `rev-parse` still echoes any option it does not understand and still
/// exits 0. `--git-common-dir` and `--git-path` shipped IN git 2.5 (2015), so only
/// something older echoes one — and measured, such a git also fails on the trailing
/// `hooks` operand and exits 128, so `ask_git` discards the output before it is
/// parsed at all. `core.hooksPath` did not exist before 2.9, so the `.git/hooks`
/// guess such a git falls back to is the right answer for it anyway.
///
/// ⚠ There used to be a rule dropping any line starting with `-`, described as belt
/// and braces. It was not free: `core.hooksPath = -hooks` is a legal setting whose
/// answer that filter ate. Deleted. A defensive rule that cannot be shown to catch
/// something the primary rule misses is not free insurance; it is an untested code
/// path with its own failure mode.
///
/// ⚠⚠ JOINING IS NOT ENOUGH — it must also NORMALISE. `Path::starts_with` is
/// component-wise, so a surviving `..` walks straight through containment. Measured:
/// with `core.hooksPath = ../shared-hooks`, git answers `Shared(<lab>/shared-hooks)`
/// and refuses it, while a bare join answered `<repo>/../shared-hooks` and was
/// accepted as `Repo`. An in-repo symlink pointing out of the tree escaped the same
/// way, as did a symlink CHAIN one hop further out.
#[must_use]
fn parse_git_paths(stdout: &str, base: &std::path::Path) -> Option<GitPaths> {
    let [toplevel, common_dir, hooks] = three_answers(stdout)?;
    Some(GitPaths {
        toplevel: absolutize(base, toplevel),
        common_dir: absolutize(base, common_dir),
        hooks: absolutize(base, hooks),
    })
}

/// Join `answer` onto `base` and resolve it the way git's own `--path-format=absolute`
/// would have — `..` applied to what is resolved so far, symlinks followed.
///
/// Walks the answer's components one at a time, resolving symlinks as it goes and
/// applying `..` to what it has resolved so far. The hooks directory need not exist
/// yet, so a component that is not on disk is simply carried lexically.
///
/// ⚠⚠ AN EARLIER VERSION WALKED UP FROM THE WHOLE JOINED PATH instead, canonicalising
/// the longest existing PREFIX and re-attaching the rest. That looks equivalent and
/// is not: `Path::file_name()` returns `None` for a `..` component, so the loop bailed
/// out to the un-normalised join whenever the nearest existing ancestor lay above a
/// `..`. Measured, `core.hooksPath = nope/../../shared-hooks`: git answers
/// `/lab/shared-hooks` (outside → refused) while the bail-out returned
/// `<repo>/nope/../../shared-hooks`, which `starts_with` accepts component-wise —
/// containment defeated. The shipped test used `../shared-hooks`, which happened to
/// work because `<repo>/..` exists: ONE component away from the failing class.
fn absolutize(base: &std::path::Path, answer: &str) -> std::path::PathBuf {
    let mut out = base.to_path_buf();
    // Shared across the whole walk, so a chain cannot be extended indefinitely by
    // splitting it across components.
    let mut budget = MAX_LINK_HOPS;
    resolve_into(&mut out, std::path::Path::new(answer), &mut budget);
    out
}

/// The bound that makes a symlink CYCLE terminate here.
///
/// ⚠ In the same spirit as the kernel's `SYMLOOP_MAX`, not equal to it — measured,
/// this platform's git gives up at 32 nested links. The number only has to be large
/// enough for real paths and small enough to end a cycle.
const MAX_LINK_HOPS: usize = 40;

/// Apply one path component to `out`, with the platform's own rules.
///
/// ⚠ `Prefix` ASSIGNS, `RootDir` PUSHES. `C:\repo` yields `Prefix("C:")` then
/// `RootDir`, and assigning on both discarded the drive: the second overwrote `C:`
/// with `MAIN_SEPARATOR_STR`, leaving a root-relative `\repo`. `PathBuf::push` of a
/// root replaces everything EXCEPT the prefix, which is exactly the rule wanted here.
fn push_component(out: &mut std::path::PathBuf, component: std::path::Component<'_>) {
    use std::path::Component;
    match component {
        Component::Prefix(_) => *out = std::path::PathBuf::from(component.as_os_str()),
        Component::RootDir => out.push(component.as_os_str()),
        Component::CurDir => {}
        Component::ParentDir => {
            out.pop();
        }
        Component::Normal(part) => out.push(part),
    }
}

/// Walk `path`'s components onto `out`, resolving each one as it lands.
///
/// ⚠⚠ A LINK TARGET IS A PATH, NOT A LIST OF NAMES, and that is why this recurses.
/// An earlier version pushed a target's components verbatim, so a component of the
/// TARGET that was itself a symlink never got followed — and a `..` after it then
/// popped the link rather than where the link lands. Measured against git 2.50 with
/// `lnk -> sub/../gone` where `sub -> <lab>/outside`: git resolves `<lab>/gone`,
/// OUTSIDE the checkout and refused, while pushing verbatim gave `<repo>/gone`,
/// INSIDE and accepted. A containment escape, in the direction that matters.
///
/// `budget` is shared with the caller so a cycle — or a chain spread across several
/// components — terminates. That is the ONLY guarantee here, and it is worth being
/// exact about what it is not.
///
/// ⚠ Do NOT justify it with "git refuses a cycle": without `--path-format=absolute`,
/// which is the ask we make, git resolves a cycle lexically and exits 0 — measured.
///
/// ⚠⚠ And do NOT claim the path we name cannot be OPENED. That held only for the one
/// shape the test used. Measured with `a -> b -> a` and `core.hooksPath = a/../hooks`:
/// the `..` pops the unresolved link and we answer `<repo>/hooks`, which EXISTS, so
/// the installers' `exists()` gate passes and both hooks are written — while git
/// cannot traverse the cycle at all (`fatal: More than 32 nested symlinks`) and runs
/// nothing. A wasted install reported as a success, not a wrong directory. The same
/// is true of an EMPTY symlink target, which macOS permits: git and we agree on
/// `<repo>`, and the kernel then refuses to traverse `empty/`, so the hook does not
/// run and the commit is NOT blocked — demonstrated end to end.
///
/// Both need a `core.hooksPath` pointing through a broken link, which git itself
/// cannot use either. Named in docs/INFRASTRUCTURE.md rather than worked around: any
/// detection would have to re-implement the kernel's traversal, which is the class of
/// reimplementation this arc has already paid for twice.
fn resolve_into(out: &mut std::path::PathBuf, path: &std::path::Path, budget: &mut usize) {
    for component in path.components() {
        push_component(out, component);
        if !matches!(component, std::path::Component::Normal(_)) {
            continue;
        }
        // Resolve as we go, so a symlinked component is followed BEFORE the next
        // `..` is applied — that is what makes `lnk/..` land where the kernel puts
        // it rather than where the link sits.
        if let Ok(real) = std::fs::canonicalize(&*out) {
            *out = real;
            continue;
        }
        // ⚠ `canonicalize` resolves a whole chain but REFUSES one that dangles, so a
        // dangling link has to be walked by hand. The kernel still follows it.
        if *budget == 0 {
            continue;
        }
        if let Ok(target) = std::fs::read_link(&*out) {
            *budget -= 1;
            out.pop();
            resolve_into(out, &target, budget);
        }
    }
}

/// Exactly three answer lines, or nothing.
///
/// ⚠ NOTHING IS TRIMMED. A directory whose name ends in a SPACE is legal on every
/// filesystem we support, and trimming it made git's answer differ from the path we
/// asked about, so a repository was classified as somebody ELSE's — the one build.rs
/// arm that returns completely silently.
///
/// ⚠ There was a `trim_end_matches('\r')` here for CRLF output. `str::lines` already
/// splits on `\r\n` and drops the `\r` — measured — so it fired only on a final line
/// ending in a bare CR with no newline, which git does not emit. A mutation survey
/// found deleting it changed nothing, which by this module's own standard makes it an
/// untested path rather than insurance. Removed; the CRLF property is gated below and
/// belongs to `lines`.
///
/// ⚠ KNOWN LIMIT, and unfixable line-by-line: a directory whose name ends in `\r`
/// would have that byte eaten by `lines` itself. Reading `-z` output would be the
/// only cure, and no supported git emits such a path here.
///
/// ⚠⚠ AND NO EMPTY-LINE FILTER, which was a LIVE BUG rather than dead weight.
/// `core.hooksPath` may end in a newline — git 2.50 stores it and genuinely runs
/// hooks out of the directory whose name ends in that newline, measured by putting
/// an executable hook in both candidates and committing. git's answer is then FOUR
/// lines with the last one empty, and dropping it left three, so the count check
/// below never fired and we installed into the TRUNCATED `<repo>/hooks` while git
/// read `<repo>/hooks\n`. Exactly what that check exists to prevent, reached by the
/// same mechanism with the newline at the END rather than the middle.
///
/// ⚠⚠ NOTHING IS DROPPED FOR LOOKING LIKE AN OPTION, and that has now been decided
/// twice, in both directions. A `-`-prefix filter ate the answer for
/// `core.hooksPath = -hooks`; an exact-match filter on the option names we pass ate
/// `core.hooksPath = --git-path`. Both are legal settings, and both failures were the
/// same one: a real answer removed, two lines left, no answer, and the `.git/hooks`
/// guess reporting a successful install into a directory git never reads.
///
/// ⚠ The BENEFIT was unreachable on every git that exists. A git that echoes one of
/// our options is, by construction, a git that cannot answer it — so the shape a
/// filter would rescue (an echo PLUS three good answers) cannot occur. Measured with
/// a faithful pre-2.5 shim: the trailing `hooks` becomes an unresolvable revision,
/// git exits 128, and `ask_git` discards the output before this function runs.
/// [`parse_git_paths`] rejects a shifted answer instead, on the one line that must be
/// a path whatever git you are on.
fn three_answers(stdout: &str) -> Option<[&str; 3]> {
    let lines: Vec<&str> = stdout.lines().collect();
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
fn classify_hooks_dir(paths: &GitPaths, repo_root: &std::path::Path) -> HooksDir {
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
/// ONE implementation, used by `build.rs` and by the xtask binary — writing this out
/// per-installer is what let the two drift.
///
/// ⚠ ONE `rev-parse` call, and the ORDER between its answer and the last-resort
/// guess is decided in the pure [`resolve_from_answers`], where a test can reach it,
/// rather than by control flow nothing could see. It briefly made TWO — one with
/// `--path-format=absolute` and one without, for the gits that do not know the flag
/// — and this paragraph outlived that design by a commit. See [`REV_PARSE_ASK`] for
/// why the flag is not asked for.
///
/// ⚠ FALLBACK, and it is the LAST resort, not the second. When git cannot answer at
/// all but `<root>/.git` is a real directory, fall back to `<root>/.git/hooks`.
/// Installing nothing there is a regression: that path is where git looks in the
/// ordinary case, and a container with no `git` binary or a repo tripping `dubious
/// ownership` is exactly where the old code worked. A worktree's `.git` is a FILE,
/// so it correctly does not take this branch — there we genuinely do not know.
///
/// ⚠⚠ The fallback must never override an answer git COULD have given. It used to:
/// the ask carried `--path-format=absolute`, which a git older than 2.31 does not
/// know, so its output was unusable and `.git/hooks` guessed WRONG for anyone with a
/// `core.hooksPath` — reporting "Installed" for a file git does not read. That was
/// first fixed with a SECOND ask; the flag is simply not asked for now, so there is
/// one ask, every supported git understands it, and the guess is reached only when
/// git could not be run at all.
#[must_use]
pub fn resolve_hooks_dir(repo_root: &std::path::Path) -> Option<HooksDir> {
    // Canonicalise before comparing with git's answer: git resolves symlinks (macOS
    // `/var` → `/private/var`), so a raw comparison would call our own checkout a
    // different repository.
    let canonical = std::fs::canonicalize(repo_root).unwrap_or_else(|_| repo_root.to_path_buf());

    let answer = ask_git(repo_root, &REV_PARSE_ASK);

    resolve_from_answers(
        answer.as_deref(),
        &canonical,
        repo_root.join(".git").is_dir(),
    )
}

/// Pick between git's answer and the last-resort guess. Pure, so it is tested.
///
/// ⚠⚠ THE ORDER IS ITSELF A DECISION, and while it lived only as control flow inside
/// [`resolve_hooks_dir`] no test could see it: a mutation survey deleted a whole rung
/// and the suite stayed green. Under a git-2.25 shim the same mutant returned
/// `Repo(<root>/.git/hooks)` where the real answer is `Shared(~/.githooks)` — a false
/// "Installed" for a file git never reads.
///
/// The rungs, strongest first:
/// 1. git's answer, with relative paths resolved against the repo root;
/// 2. `<root>/.git/hooks`, and ONLY when git could not be asked at all — a guess that
///    overrides a knowable answer is how the false success got here.
#[must_use]
fn resolve_from_answers(
    answer: Option<&str>,
    repo_root: &std::path::Path,
    dot_git_is_dir: bool,
) -> Option<HooksDir> {
    if let Some(paths) = answer.and_then(|out| parse_git_paths(out, repo_root)) {
        return Some(classify_hooks_dir(&paths, repo_root));
    }
    if dot_git_is_dir {
        return Some(HooksDir::Repo(repo_root.join(".git").join("hooks")));
    }
    None
}

/// The one question we ask git, lifted out so a test can inspect it.
///
/// ⚠⚠ NO `--path-format=absolute`, and that is the decision this const exists to
/// pin. Adding it looks free — on a modern git it changes nothing, which is exactly
/// why a mutation survey found nothing objecting. On a git older than 2.31 (Ubuntu
/// 20.04 ships 2.25) it is not free: that git does not know the flag, echoes it as
/// the FIRST line since it is the first option we pass, every answer shifts, and
/// `parse_git_paths` refuses on the toplevel rule. The result is safe but WRONG —
/// resolution falls back to `.git/hooks` and a `core.hooksPath` is silently lost.
///
/// Every option here shipped in git 2.5 (2015) or earlier.
const REV_PARSE_ASK: [&str; 5] = [
    "rev-parse",
    "--show-toplevel",
    "--git-common-dir",
    "--git-path",
    "hooks",
];

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

/// What the build script should DO about a resolution, and what it should say.
///
/// ⚠⚠ EXTRACTED FOR THE SAME REASON `hooks_dir_from` was, and it was overdue. This
/// branch's whole thesis is "a build script has no test target, so lift the decision
/// somewhere a test can reach it" — and that was applied to `cargo xtask setup`,
/// the installer a developer types, while `build.rs` (the one a FRESH CLONE runs)
/// kept its three directory-level arms as untestable control flow. Mutating its
/// `Shared` arm into an install shipped green. These are the guards that stop a
/// vendored copy writing into somebody else's repository.
///
/// ⚠ `dead_code` is allowed on THIS ITEM ONLY, and that is the point. The whole
/// module used to carry a blanket `#[allow(dead_code)]`, which outlived its reason:
/// `setup` began calling `title_of`/`classify`/`HookState`, so the attribute was
/// suppressing nothing while its explanation quietly went false. `build_outcome` and
/// this enum genuinely are build-script-only — `build.rs` reaches them through
/// `include!`, which the binary's dead-code pass cannot see — so the exemption is
/// scoped to them and says so.
#[derive(Debug, Clone, PartialEq, Eq)]
#[allow(dead_code)]
pub enum BuildOutcome {
    /// Install into this directory.
    Install(std::path::PathBuf),
    /// Do nothing, and say nothing. Reserved for cases that are not about this
    /// developer at all — a vendored copy, a tarball with no `.git`.
    Silent,
    /// Do nothing, and say exactly this.
    Warn(String),
}

/// Decide what a build script does with a resolution. Pure, so it is tested.
///
/// `dot_git_exists` distinguishes a checkout from an extracted tarball; `git_env_set`
/// is whether `GIT_DIR`/`GIT_WORK_TREE` are in the environment, which is the one
/// layout resolution deliberately refuses to follow.
#[must_use]
#[allow(dead_code)] // build-script-only; see `BuildOutcome`.
pub fn build_outcome(
    resolved: Option<HooksDir>,
    dot_git_exists: bool,
    git_env_set: bool,
) -> BuildOutcome {
    match resolved {
        Some(HooksDir::Repo(dir)) => BuildOutcome::Install(dir),
        // A copy of this source nested inside SOMEBODY ELSE'S checkout: git resolved
        // their repository by walking up. Automatic installation only ever targets
        // our own checkout — writing our mesh guard and our `cargo fmt` into an
        // unrelated project is the harm containment exists to prevent. Silent on
        // purpose: a vendored copy has nothing to say.
        Some(HooksDir::OtherRepo(_)) => BuildOutcome::Silent,
        // ⚠ Says WHERE, not WHOSE — a repo-LOCAL `core.hooksPath` pointing out of the
        // tree lands here too, and telling that developer the directory is "shared
        // with every other repo on this machine" is simply false.
        Some(HooksDir::Shared(dir)) => BuildOutcome::Warn(format!(
            "git is configured to read hooks from {}, which is outside this \
             repository — a directory out there may be shared with your other repos. \
             CortenForge's hooks were NOT installed and the scan/mesh guard is not \
             armed. Merge xtask/hooks/* into it yourself, or point core.hooksPath \
             somewhere inside this checkout.",
            dir.display()
        )),
        // ⚠ `GIT_DIR`/`GIT_WORK_TREE` are cleared before asking git, because
        // otherwise the environment decides which repository we install into. The
        // cost lands here: a bare repo checked out through those variables can no
        // longer be resolved, and its working tree has no `.git` for the next check
        // to find — so without this it is the one shape that gets no hooks AND no
        // explanation.
        None if git_env_set => BuildOutcome::Warn(
            "GIT_DIR/GIT_WORK_TREE are set in this environment. They are ignored \
             when locating hooks, because they let the environment install into a \
             different repository — so this checkout could not be resolved and \
             CortenForge's hooks were NOT installed. Build from the working tree \
             without them."
                .to_string(),
        ),
        // Only worth saying anything if this looks like a checkout. xtask can be
        // built from a vendored copy with no git present, where silence is right.
        None if dot_git_exists => BuildOutcome::Warn(
            "Could not determine git's hooks directory, so CortenForge's hooks were \
             NOT installed and the scan/mesh guard is not armed."
                .to_string(),
        ),
        None => BuildOutcome::Silent,
    }
}

/// What the caller was attempting, which is what makes the advice right or absurd.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Attempted {
    /// Installing. `retry` is how THIS caller is re-run — "rebuild" for the build
    /// script, "re-run" for `cargo xtask setup`.
    Install { retry: &'static str },
    /// Removing. There is nothing to merge and nothing to re-run.
    Uninstall,
}

/// Say precisely why a hook was left alone, and what to do about it.
///
/// ⚠⚠ SHARED BY BOTH INSTALLERS, and that is the entire point. This started life in
/// `setup.rs` alone, so `cargo build` and `cargo xtask setup` told DIFFERENT stories
/// about the same file — in the very commit that moved `is_executable` here to stop
/// exactly that, and that rewrote `build.rs`'s other message because "a fix that
/// lands in one installer and not the other is the drift this arc exists to end".
/// It landed in one installer and not the other.
///
/// Two cases were being told the wrong story:
/// - An EMPTY hook classifies `Foreign` — correctly, it carries no marker — but
///   "merge ours into yours" is nonsense for a zero-byte file, and before the
///   ownership check `setup` used to be the way to repair one. Name it, and give the
///   one action that works.
/// - Anything merely UNREADABLE was reported as somebody else's hook: a chmod-000
///   hook that IS ours, a directory, a dangling symlink.
#[must_use]
pub fn describe_untouchable(
    state: HookState,
    name: &str,
    path: &std::path::Path,
    attempted: Attempted,
) -> String {
    let empty = path.metadata().is_ok_and(|m| m.len() == 0);
    match (state, attempted) {
        // ⚠ Uninstalling says only what it DID. Advising a merge, or a re-run, in the
        // middle of `uninstall` tells the developer to install the thing they asked
        // to remove.
        //
        // ⚠⚠ But it still has to distinguish the two states. A `_` here threw the
        // distinction away and told the owner of a chmod-000 hook that IS ours — or
        // of a hook symlinked to `xtask/hooks/` whose relative target stopped
        // resolving — that their own file belonged to somebody else. That is the
        // exact bug the doc above says this function fixed, reintroduced on the
        // uninstall path one round later.
        (HookState::Unreadable, Attempted::Uninstall) => format!(
            "⚠ Could not read the {name} hook, so it was left in place — it may be a \
             directory, a broken symlink, or unreadable. If it is ours, remove it by \
             hand."
        ),
        (_, Attempted::Uninstall) => {
            format!("⚠ Left the {name} hook in place — it is not one we installed.")
        }
        (HookState::Foreign, Attempted::Install { retry }) if empty => format!(
            "⚠ The existing {name} hook is EMPTY, so it carries no mark of ours and \
             was left alone — CortenForge's {name} hook is NOT installed and its \
             checks will not run. Delete {} and {retry}.",
            path.display()
        ),
        (HookState::Foreign, Attempted::Install { retry }) => format!(
            "⚠ Left your existing {name} hook in place — CortenForge's is NOT \
             installed and its checks will not run. Merge xtask/hooks/{name} into \
             yours, or move yours aside and {retry}."
        ),
        (_, Attempted::Install { .. }) => format!(
            "⚠ Could not read the existing {name} hook, so it was left alone — it \
             may be a directory, a broken symlink, or unreadable. CortenForge's \
             {name} hook is NOT installed and its checks will not run."
        ),
    }
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
    use super::{classify, title_of, HookState, HOOKS};

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

    /// A SHIFTED answer is refused; a legitimate answer that looks like a flag is not.
    ///
    /// ⚠ `rev-parse` echoes any option it does not understand and still exits 0. If
    /// that ever happens every answer shifts and `toplevel` stops being a path — so
    /// the one line that must be absolute on every git is the line that is checked.
    /// Refusing sends the caller to the `.git/hooks` fallback (hooks installed,
    /// nothing hidden); letting a shift through yields `OtherRepo`, which `build.rs`
    /// handles by returning SILENTLY.
    ///
    /// ⚠⚠ TWO FILTERS DIED TO GET HERE, and both died the same way — eating a REAL
    /// answer. A `-`-prefix rule ate `core.hooksPath = -hooks`; an exact-match rule on
    /// our own option names ate `core.hooksPath = --git-path`. Both are legal
    /// settings. Both cases are asserted below, so a third attempt at filtering by
    /// what a line LOOKS like fails here.
    #[test]
    fn a_shifted_answer_is_refused_and_a_flag_shaped_answer_is_not() {
        use super::parse_git_paths;
        use std::path::{Path, PathBuf};

        let base = Path::new("/repo");

        let ordinary = parse_git_paths("/repo\n.git\n.git/hooks\n", base)
            .expect("toplevel absolute, the rest relative — what git actually emits");
        assert_eq!(ordinary.toplevel, PathBuf::from("/repo"));
        assert_eq!(ordinary.hooks, PathBuf::from("/repo/.git/hooks"));

        // ★ THE REAL ECHO SHAPE, measured on git 2.50: an unrecognised option is
        // echoed as an EXTRA LINE IN POSITION — it never replaces an answer — so it
        // makes FOUR lines and is refused on count. An earlier version of this test
        // fed a hand-built THREE-line string with the flag first, which no git
        // emits, and used it to justify a `toplevel.is_absolute()` rule that
        // therefore never fired. A fixture no git can produce tests nothing.
        assert!(
            parse_git_paths("/repo\n--some-option\n.git\n.git/hooks\n", base).is_none(),
            "an echoed option makes a fourth line; taking three of four silently \
             shifts which answer is which"
        );

        // ★★ AND THE COST SIDE. These are real answers that a filter would eat.
        // ⚠ `hooks` is included deliberately: it is the ARGUMENT we pass to
        // `--git-path`, not an option we ask for, and "the options we ask for" is
        // ambiguous enough that a filter listing it would eat a legal answer.
        for hooks_answer in ["-hooks", "--git-path", "--show-toplevel", "hooks"] {
            let paths = parse_git_paths(&format!("/repo\n.git\n{hooks_answer}\n"), base)
                .unwrap_or_else(|| {
                    panic!("`core.hooksPath = {hooks_answer}` is legal and its answer was eaten")
                });
            assert_eq!(
                paths.hooks,
                PathBuf::from(format!("/repo/{hooks_answer}")),
                "the answer for `core.hooksPath = {hooks_answer}` was not carried through"
            );
        }

        assert!(
            parse_git_paths("/repo\n.git\n", base).is_none(),
            "a short answer is not an answer"
        );
        assert!(
            parse_git_paths("", base).is_none(),
            "no output is not an answer"
        );
    }

    /// The ask must stay answerable by every git we support.
    ///
    /// ⚠ This pins a DESIGN decision that no behavioural test can see: re-adding
    /// `--path-format=absolute` changes nothing on a modern git, so a mutation
    /// survey found nothing objecting to it. Its cost lands only on git < 2.31,
    /// which echoes the flag — and the branch that removed it defended the removal
    /// with prose, measured by hand, with no gate.
    #[test]
    fn the_ask_carries_no_option_an_old_git_would_echo() {
        let ask = super::REV_PARSE_ASK;
        assert!(
            !ask.iter().any(|a| a.starts_with("--path-format")),
            "`--path-format` arrived in git 2.31; Ubuntu 20.04 ships 2.25, which \
             echoes it as the FIRST line, shifts every answer, and silently loses a \
             core.hooksPath to the .git/hooks fallback: {ask:?}"
        );
        // POSITIVE CONTROL: it must still ask the three questions resolution needs,
        // or "carries no bad option" would be satisfied by asking nothing.
        for required in ["--show-toplevel", "--git-common-dir", "--git-path"] {
            assert!(
                ask.contains(&required),
                "{required} is what containment is computed from: {ask:?}"
            );
        }
    }

    /// A fourth line means the answers cannot be trusted to line up.
    ///
    /// ⚠ Not hypothetical, and not only about echoed options: `core.hooksPath` may
    /// contain a NEWLINE — git 2.50 accepts it — and the answer then arrives split
    /// across two lines. Taking the first three would install into a TRUNCATED
    /// directory, which `setup` would happily create. The count check is what the
    /// module's own doc leans on, and nothing exercised it.
    #[test]
    fn more_answers_than_asked_for_is_not_an_answer() {
        use super::{resolve_from_answers, three_answers, HooksDir};
        use std::path::{Path, PathBuf};

        assert!(
            three_answers("/r\n.git\n.git/hooks\n/extra\n").is_none(),
            "a fourth line means something answered that we did not ask, so the \
             three we read may not be the three we wanted"
        );
        // ★★ AND AN EMPTY FOURTH LINE COUNTS. `core.hooksPath` may end in a newline;
        // git 2.50 stores it and RUNS hooks from the directory whose name ends in
        // that newline — measured by putting an executable hook in both candidates
        // and committing. Its answer is four lines, the last empty. An
        // `!l.is_empty()` filter dropped it, left three, and installed into the
        // TRUNCATED `<repo>/hooks` while git read `<repo>/hooks\n`.
        assert!(
            three_answers("/r\n.git\nhooks\n\n").is_none(),
            "an empty fourth line is still a fourth line; dropping it installs into \
             a truncated directory that git does not read"
        );
        // POSITIVE CONTROL: exactly three is still an answer.
        assert!(three_answers("/r\n.git\n.git/hooks\n").is_some());

        // `core.hooksPath = "ho\noks"` — one setting, two lines.
        assert_eq!(
            resolve_from_answers(Some("/r\n.git\n/r/ho\noks\n"), Path::new("/r"), true),
            Some(HooksDir::Repo(PathBuf::from("/r/.git/hooks"))),
            "a hooks path containing a newline splits git's answer; reading the \
             first three lines installs into a truncated directory"
        );
    }

    /// The hop budget is bounded at BOTH ends, and only one end was pinned.
    ///
    /// ⚠ `resolve_into` recurses once per hop, so this const is a stack-depth
    /// control as well as a loop bound. Measured on this platform: 10_000 terminates,
    /// 100_000 aborts the process with `fatal runtime error: stack overflow`. A build
    /// script runs this, so that abort is a failed build with no diagnostic at all —
    /// and no behavioural test can see the difference, because every value in between
    /// resolves the same paths. A mutation raising it to 1000 survived the suite.
    #[test]
    fn the_hop_budget_is_bounded_at_both_ends() {
        let budget = super::MAX_LINK_HOPS;
        // Lower: below the kernel's own limit we would truncate chains it WOULD
        // resolve, and name an inside path for a directory that is outside.
        assert!(
            budget >= 32,
            "this platform's kernel follows 32 nested links; a smaller budget \
             truncates chains git resolves fine: {budget}"
        );
        // Upper: no real path needs more than twice that, and the recursion is why
        // the ceiling matters.
        assert!(
            budget <= 64,
            "resolve_into recurses once per hop, and a large budget turns a crafted \
             symlink cycle into a stack overflow — which in a build script is a \
             failed build with no message: {budget}"
        );
    }

    /// A symlink CYCLE must terminate.
    ///
    /// ⚠ The ask carries no `--path-format=absolute`, and WITHOUT it git resolves a
    /// cycle lexically and exits 0 — it does NOT report the path as unresolvable, as
    /// an earlier version of `resolve_into`'s doc claimed.
    ///
    /// ⚠⚠ Nor does "whatever we name cannot be opened" hold — that was the SECOND
    /// false version of this claim, true only for the bare `a` this test started
    /// with. `a/../hooks` names a directory that exists. The assertion below is now
    /// the one thing that is actually guaranteed: it TERMINATES. The consequence of
    /// the rest is recorded in docs/INFRASTRUCTURE.md.
    #[test]
    #[cfg(unix)]
    fn a_symlink_cycle_terminates_and_names_nothing_that_exists() {
        let lab = std::env::temp_dir().join(format!("cf-cycle-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&lab);
        let repo = lab.join("repo");
        std::fs::create_dir_all(repo.join(".git")).expect("temp dir");
        let repo = std::fs::canonicalize(&repo).expect("canonicalize");
        std::os::unix::fs::symlink("b", repo.join("a")).expect("symlink");
        std::os::unix::fs::symlink("a", repo.join("b")).expect("symlink");

        let verdict = |hooks: &str| {
            super::parse_git_paths(&format!("{}\n.git\n{hooks}\n", repo.display()), &repo)
                .expect("three answers")
                .hooks
        };
        // Three shapes through the same cycle. Each must RETURN; none may hang.
        let bare = verdict("a");
        let up = verdict("a/..");
        let up_then_down = verdict("a/../hooks");
        let _ = std::fs::remove_dir_all(&lab);
        let resolved = bare;

        // Terminating at all is the claim: an unbounded walk would hang the build.
        assert!(
            resolved.starts_with(&repo),
            "a cycle should resolve to somewhere in the checkout, not wander: {resolved:?}"
        );
        // ⚠ NO assertion that these cannot be opened — `a/../hooks` names a real
        // directory, and asserting otherwise is what made this test agree with a
        // false doc for two rounds. What is pinned is that every shape terminates
        // and stays within the checkout rather than wandering.
        for (label, path) in [("a/..", &up), ("a/../hooks", &up_then_down)] {
            assert!(
                path.starts_with(&repo),
                "{label} resolved outside the checkout: {path:?}"
            );
        }
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

    /// git answers RELATIVELY, and those answers must be used rather than discarded.
    ///
    /// ⚠ This is what stops the `.git/hooks` fallback from overriding a knowable
    /// answer. Without it, a developer with a `core.hooksPath` was told "Installed"
    /// for a file git does not read — a false success, which is the failure mode the
    /// whole arc exists to remove.
    #[test]
    fn an_old_gits_relative_answer_is_usable_rather_than_discarded() {
        use super::{classify_hooks_dir, parse_git_paths, HooksDir};
        use std::path::{Path, PathBuf};

        let base = Path::new("/repo");

        // The exact mixed shape measured from a git without `--path-format`:
        // toplevel absolute, git dir and hooks relative to the directory it ran in.
        let paths = parse_git_paths("/repo\n.git\n.git/hooks\n", base)
            .expect("relative answers are usable once rooted");
        assert_eq!(paths.common_dir, PathBuf::from("/repo/.git"));
        assert_eq!(paths.hooks, PathBuf::from("/repo/.git/hooks"));
        assert_eq!(
            paths.toplevel,
            PathBuf::from("/repo"),
            "join must leave an already-absolute answer alone"
        );

        assert!(
            parse_git_paths("/repo\n.git\n", base).is_none(),
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

        let escaped = parse_git_paths(
            &format!("{}\n.git\n../shared-hooks\n", repo.display()),
            &repo,
        )
        .expect("old git emits exactly this shape");
        let verdict = classify_hooks_dir(&escaped, &repo);

        // ⚠⚠ THE SAME ESCAPE ONE COMPONENT DEEPER, which the `../shared-hooks` case
        // above does NOT cover. An earlier `absolutize` walked up from the whole
        // joined path, and `Path::file_name()` is `None` for `..`, so it bailed to
        // the un-normalised join whenever the nearest EXISTING ancestor lay above a
        // `..`. `../shared-hooks` passed only because `<repo>/..` exists. Here the
        // first component does not exist, so the old implementation gave up and
        // returned `<repo>/nope/../../shared-hooks` — accepted by a component-wise
        // `starts_with` as inside the repo.
        let deep = parse_git_paths(
            &format!("{}\n.git\nnope/../../shared-hooks\n", repo.display()),
            &repo,
        )
        .expect("old git emits exactly this shape");
        let deep_verdict = classify_hooks_dir(&deep, &repo);

        // ★ And through a SYMLINK that leaves the tree: `..` must apply to where the
        // link LANDS, not to where it sits, which is why components are resolved one
        // at a time rather than lexically.
        std::os::unix::fs::symlink(lab.join("shared-hooks"), repo.join("lnk")).ok();
        let via_link = parse_git_paths(&format!("{}\n.git\nlnk\n", repo.display()), &repo)
            .expect("old git emits exactly this shape");
        let link_verdict = classify_hooks_dir(&via_link, &repo);

        // POSITIVE CONTROL: a relative answer that stays INSIDE must still be ours,
        // or "refuses everything" would pass the assertion above.
        let inside = parse_git_paths(&format!("{}\n.git\n.git/hooks\n", repo.display()), &repo)
            .expect("old git emits exactly this shape");
        let inside_verdict = classify_hooks_dir(&inside, &repo);
        let _ = std::fs::remove_dir_all(&lab);

        assert_eq!(
            verdict,
            HooksDir::Shared(outside.clone()),
            "`..` in an old git's answer escaped the checkout and was accepted"
        );
        assert_eq!(
            inside_verdict,
            HooksDir::Repo(repo.join(".git").join("hooks")),
            "POSITIVE CONTROL: an old git's ordinary answer is still ours"
        );
        assert_eq!(
            deep_verdict,
            HooksDir::Shared(outside.clone()),
            "a `..` whose parent does not exist escaped the checkout — the shipped \
             test was one component away from seeing it"
        );
        assert_eq!(
            link_verdict,
            HooksDir::Shared(outside),
            "an in-repo symlink pointing out of the tree escaped containment"
        );
    }

    /// Both rungs, in order, from the shapes git actually emits.
    ///
    /// ⚠⚠ THE ORDERING HAD NO GATE. A mutation survey deleted a whole rung and every
    /// test stayed green — on a modern git the rung above answered and the one below
    /// never ran. Under a git-2.25 shim the same mutant turned `Shared(~/.githooks)`
    /// into `Repo(.git/hooks)`: the false "Installed" for a file git never reads,
    /// which is this arc's whole subject.
    ///
    /// ⚠ This used to have THREE rungs, because asking for `--path-format=absolute`
    /// forced a second ask for the gits that do not know that flag. Asking a question
    /// every supported git understands removed the middle one.
    #[test]
    fn gits_answer_wins_and_the_guess_is_last() {
        use super::{resolve_from_answers, HooksDir};
        use std::path::{Path, PathBuf};

        let root = Path::new("/nx-cf-rungs");
        let ordinary = "/nx-cf-rungs\n.git\n.git/hooks\n";
        let elsewhere = "/nx-cf-rungs\n.git\n/nx-cf-elsewhere/.githooks\n";

        assert_eq!(
            resolve_from_answers(Some(ordinary), root, true),
            Some(HooksDir::Repo(PathBuf::from("/nx-cf-rungs/.git/hooks"))),
            "git's own answer must be used"
        );

        // ★ THE POINT. `dot_git_is_dir` is true, so the guess is available and would
        // happily return `<root>/.git/hooks`. git says the hooks are elsewhere, and
        // that has to win — otherwise the developer is told "Installed" about a file
        // git does not read.
        assert_eq!(
            resolve_from_answers(Some(elsewhere), root, true),
            Some(HooksDir::Shared(PathBuf::from(
                "/nx-cf-elsewhere/.githooks"
            ))),
            "the `.git/hooks` guess overrode a knowable answer — the false success"
        );

        // An unusable answer falls through to the guess, but only then.
        assert_eq!(
            resolve_from_answers(Some("/nx-cf-rungs\n.git\n"), root, true),
            Some(HooksDir::Repo(PathBuf::from("/nx-cf-rungs/.git/hooks"))),
            "a truncated answer is not an answer, and the guess is what is left"
        );
        assert_eq!(
            resolve_from_answers(None, root, true),
            Some(HooksDir::Repo(PathBuf::from("/nx-cf-rungs/.git/hooks"))),
            "installing nothing when git cannot be asked is a regression"
        );
        assert_eq!(
            resolve_from_answers(None, root, false),
            None,
            "with no .git directory there is nothing to guess at"
        );
    }

    /// The build script's OWN three refusals, which nothing reached.
    ///
    /// ⚠⚠ `cargo xtask setup`'s refusals were extracted into `hooks_dir_from` and
    /// tested precisely so a mutation could not gut them — while `build.rs`, the
    /// installer a FRESH CLONE actually runs, kept the same three decisions as
    /// untestable control flow. Turning its `Shared` arm into an install shipped
    /// green. These arms are what stop a vendored copy writing our mesh guard and our
    /// `cargo fmt` into an unrelated project.
    #[test]
    fn the_build_script_installs_only_into_our_own_checkout() {
        use super::{build_outcome, BuildOutcome, HooksDir};
        use std::path::PathBuf;

        // POSITIVE CONTROL first: a rule that refuses everything is not containment.
        assert_eq!(
            build_outcome(
                Some(HooksDir::Repo(PathBuf::from("/repo/.git/hooks"))),
                true,
                false
            ),
            BuildOutcome::Install(PathBuf::from("/repo/.git/hooks")),
            "our own checkout must still be installed into"
        );

        // ★ SILENT, deliberately: a vendored copy has nothing to say to the host
        // repository's developer, and must certainly not write to it.
        assert_eq!(
            build_outcome(
                Some(HooksDir::OtherRepo(PathBuf::from("/outer/.git/hooks"))),
                true,
                false
            ),
            BuildOutcome::Silent,
            "a copy nested in someone else's checkout must write nothing, quietly"
        );

        // Outside the repo: never written, always announced — a guard that is not
        // armed must never be silent.
        let shared = build_outcome(
            Some(HooksDir::Shared(PathBuf::from("/home/dev/.githooks"))),
            true,
            false,
        );
        let BuildOutcome::Warn(message) = shared else {
            panic!("a hooks dir outside the repo must warn, not install: {shared:?}");
        };
        assert!(
            message.contains("/home/dev/.githooks"),
            "the warning must name the directory, or it is a dead end: {message}"
        );
        assert!(
            !message.contains("every other repo on this machine"),
            "a repo-LOCAL core.hooksPath lands here too, so this must not assert \
             whose directory it is: {message}"
        );

        // Unresolvable, with the environment that explains why.
        let env = build_outcome(None, false, true);
        let BuildOutcome::Warn(message) = env else {
            panic!("GIT_DIR/GIT_WORK_TREE must be explained, not silently ignored: {env:?}");
        };
        assert!(
            message.contains("GIT_DIR"),
            "name the variable the developer has to unset: {message}"
        );

        // Unresolvable in something that looks like a checkout: say so.
        assert!(
            matches!(build_outcome(None, true, false), BuildOutcome::Warn(_)),
            "a checkout we cannot resolve must not be left silently unguarded"
        );
        // ...but an extracted tarball has no hooks to install and nothing to say.
        assert_eq!(
            build_outcome(None, false, false),
            BuildOutcome::Silent,
            "no .git and no git env is not a checkout — silence is right"
        );
    }

    /// `./` is resolved AWAY, as `--path-format=absolute` resolves it.
    ///
    /// ⚠ ASSERTED ON BYTES, not on `PathBuf`. `PathBuf: PartialEq` compares
    /// `components()`, which normalises a non-leading `.` away — so
    /// `assert_eq!(hooks, PathBuf::from("/repo/.git/hooks"))` is TRUE for
    /// `/repo/./.git/hooks`, and every equality assertion in this module would have
    /// accepted a `CurDir` arm that pushed `"."`. Measured: a mutation survey found
    /// that arm changed no verdict anywhere, and replacing it with `panic!` left the
    /// whole suite green — live code, reached by nothing.
    ///
    /// The arm IS live: real git 2.50 answers `.` for `core.hooksPath = .`, `./` for
    /// `./`, and `./nested` for `././nested`.
    #[test]
    fn a_dot_component_is_resolved_away_rather_than_carried() {
        use super::parse_git_paths;
        use std::ffi::OsStr;
        use std::path::Path;

        let base = Path::new("/nx-cf-dot-repo");

        let nested =
            parse_git_paths("/nx-cf-dot-repo\n./.git\n./nested\n", base).expect("three answers");
        assert_eq!(
            nested.hooks.as_os_str(),
            OsStr::new("/nx-cf-dot-repo/nested"),
            "a `./` survived into the resolved hooks path: {:?}",
            nested.hooks
        );
        assert_eq!(
            nested.common_dir.as_os_str(),
            OsStr::new("/nx-cf-dot-repo/.git"),
            "and into the git dir: {:?}",
            nested.common_dir
        );

        // `core.hooksPath = .` — old git answers the bare `.`, meaning the repo root.
        let root = parse_git_paths("/nx-cf-dot-repo\n./.git\n.\n", base).expect("three answers");
        assert_eq!(
            root.hooks.as_os_str(),
            OsStr::new("/nx-cf-dot-repo"),
            "`core.hooksPath = .` must resolve to the root itself, not `<root>/.`: {:?}",
            root.hooks
        );

        // POSITIVE CONTROL: an answer with no `.` is carried through untouched, so
        // this is not satisfied by a function that mangles every path.
        let plain =
            parse_git_paths("/nx-cf-dot-repo\n.git\n.git/hooks\n", base).expect("three answers");
        assert_eq!(
            plain.hooks.as_os_str(),
            OsStr::new("/nx-cf-dot-repo/.git/hooks"),
            "POSITIVE CONTROL: an ordinary answer must be unchanged"
        );
    }

    /// A DANGLING symlink is still a symlink, and the kernel still follows it.
    ///
    /// ⚠ `canonicalize` refuses one, so the component stayed lexical and a following
    /// `..` popped the LINK rather than the link's TARGET. Measured against real git
    /// 2.50.1, `core.hooksPath = dangle/../hooks` where `dangle -> <lab>/outside/x`
    /// (target absent): git answers `<lab>/outside/hooks`, which is outside the
    /// checkout and refused, while resolution answered `<repo>/hooks` — and that path
    /// can EXIST, so the `hooks_dir.exists()` gate does not catch it and `setup`
    /// installs there and reports success.
    ///
    /// The oracle here is git's own answer, recorded from a real run, not a second
    /// implementation of the same walk.
    #[test]
    #[cfg(unix)]
    fn a_dangling_symlink_is_followed_the_way_git_follows_it() {
        use super::{classify_hooks_dir, parse_git_paths, HooksDir};

        let lab = std::env::temp_dir().join(format!("cf-dangle2-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&lab);
        let repo = lab.join("repo");
        std::fs::create_dir_all(repo.join(".git")).expect("temp dir");
        std::fs::create_dir_all(lab.join("outside")).expect("temp dir");
        let repo = std::fs::canonicalize(&repo).expect("canonicalize");
        let lab_real = std::fs::canonicalize(&lab).expect("canonicalize");
        // The target does NOT exist. That is the whole point.
        std::os::unix::fs::symlink(lab_real.join("outside/x"), repo.join("dangle"))
            .expect("symlink");

        let verdict = |hooks: &str| {
            let paths = parse_git_paths(&format!("{}\n.git\n{hooks}\n", repo.display()), &repo)
                .expect("old git emits exactly this shape");
            classify_hooks_dir(&paths, &repo)
        };
        let through_link = verdict("dangle/../hooks");
        let onto_link = verdict("dangle");
        // ★★ A CHAIN, not one hop. `l1 -> l2 -> <lab>/outside/gone`: `canonicalize`
        // refuses the whole thing because the end dangles, and following `read_link`
        // exactly ONCE stops at `<repo>/l2` — INSIDE the checkout, and accepted —
        // while git resolves the chain to `<lab>/outside/gone` and refuses it.
        // Measured against git 2.50; the one-hop version shipped in #835.
        std::os::unix::fs::symlink("l2", repo.join("l1")).expect("symlink");
        std::os::unix::fs::symlink(lab_real.join("outside/gone"), repo.join("l2"))
            .expect("symlink");
        let through_chain = verdict("l1");
        // ★★ AND `..` POPPING ONTO A SYMLINKED COMPONENT OF THE TARGET. `dangle`
        // points at `symdir/gone`; `symdir` is itself a link out of the tree. Walking
        // the target's components verbatim left `symdir` unresolved, so `..` popped
        // the LINK and `starts_with(toplevel)` accepted the in-repo SPELLING of an
        // out-of-repo directory. Measured against git 2.50: git says `<lab>/real`,
        // we said `Repo(<repo>/symdir)` — accepted, and installed into.
        std::fs::create_dir_all(lab_real.join("real")).expect("temp dir");
        std::os::unix::fs::symlink(lab_real.join("real"), repo.join("symdir")).expect("symlink");
        std::os::unix::fs::symlink("symdir/gone", repo.join("dangle2")).expect("symlink");
        let popped_onto_link = verdict("dangle2/..");
        // ★★ A LINK DEEPER IN THE ANSWER. Every symlink case above puts the link at
        // component ONE, so resolving only the first component passed them all —
        // measured, that mutant then installs into `<repo>/sub/x` (accepted as ours)
        // while git runs the hook from `<lab>/x`. The sibling shape `sub/lnk` is
        // worse: it names a directory that EXISTS outside the checkout, so the
        // installers' `exists()` gate does not catch it either.
        std::fs::create_dir_all(repo.join("sub")).expect("temp dir");
        std::fs::create_dir_all(lab_real.join("x")).expect("temp dir");
        std::os::unix::fs::symlink(lab_real.join("outside"), repo.join("sub/lnk"))
            .expect("symlink");
        let deep_link = verdict("sub/lnk");
        let deep_then_up = verdict("sub/lnk/../x");
        // POSITIVE CONTROL: an ordinary relative answer is still ours, so a rule that
        // simply refused everything would not pass this test.
        let ordinary = verdict(".git/hooks");
        let _ = std::fs::remove_dir_all(&lab);

        assert_eq!(
            through_link,
            HooksDir::Shared(lab_real.join("outside").join("hooks")),
            "`..` popped the dangling LINK instead of its TARGET, so a hooks \
             directory git resolves to `<lab>/outside/hooks` was accepted as ours"
        );
        assert_eq!(
            onto_link,
            HooksDir::Shared(lab_real.join("outside").join("x")),
            "a dangling link pointing out of the tree must still be followed out"
        );
        assert_eq!(
            ordinary,
            HooksDir::Repo(repo.join(".git").join("hooks")),
            "POSITIVE CONTROL: the ordinary answer is unaffected"
        );
        assert_eq!(
            through_chain,
            HooksDir::Shared(lab_real.join("outside").join("gone")),
            "a symlink CHAIN was followed only one hop, so a hooks directory git \
             resolves outside the checkout was accepted as ours"
        );
        assert_eq!(
            deep_link,
            HooksDir::Shared(lab_real.join("outside")),
            "a symlink at a LATER component of the answer was left unresolved, so a \
             directory outside the checkout was accepted as ours"
        );
        assert_eq!(
            deep_then_up,
            HooksDir::Shared(lab_real.join("x")),
            "`..` after a later symlinked component popped the LINK, not its target"
        );
        assert_eq!(
            popped_onto_link,
            HooksDir::Shared(lab_real.join("real")),
            "`..` popped a SYMLINKED component of the link target, so a hooks \
             directory git resolves outside the checkout was accepted as ours"
        );
    }

    /// A hooks directory whose name starts with `-` is an ANSWER, not an echoed flag.
    ///
    /// ⚠ This is the gate the commit that deleted the `-`-prefix filter did not
    /// write, against its own stated standard. Re-adding the filter survived every test in this module; measured on the pure functions, the filter turns
    /// `Repo(<repo>/-hooks)` into `Repo(<repo>/.git/hooks)` — a successful-looking
    /// install into a directory git never reads.
    #[test]
    fn a_dash_prefixed_hooks_path_is_an_answer_not_an_echoed_option() {
        use super::{resolve_from_answers, HooksDir};
        use std::path::Path;

        let root = Path::new("/nx-cf-dash-repo");
        let legacy = "/nx-cf-dash-repo\n/nx-cf-dash-repo/.git\n-hooks\n";
        assert_eq!(
            resolve_from_answers(Some(legacy), root, true),
            Some(HooksDir::Repo(root.join("-hooks"))),
            "`core.hooksPath = -hooks` is legal; dropping its answer for the leading \
             dash leaves two lines, no answer, and the `.git/hooks` guess reporting a \
             successful install into a directory git never reads"
        );

        // NEGATIVE CONTROL: an echoed option is still refused — on COUNT, the shape
        // git actually emits — without any rule inspecting what a line LOOKS like.
        // That is exactly what lets `-hooks` above survive.
        assert_eq!(
            resolve_from_answers(Some("/r\n--git-common-dir\n.git\n-hooks\n"), root, true),
            Some(HooksDir::Repo(root.join(".git").join("hooks"))),
            "a four-line answer must fall back, not be read as three"
        );
    }

    /// A directory whose name ends in a space is legal, and trimming broke it.
    #[test]
    fn a_path_is_not_trimmed_but_a_carriage_return_is() {
        use super::parse_git_paths;
        use std::path::PathBuf;

        let base = std::path::Path::new("/repo/trail ");
        let paths = parse_git_paths("/repo/trail \n.git\n.git/hooks\n", base)
            .expect("trailing spaces are part of the path");
        assert_eq!(
            paths.toplevel,
            PathBuf::from("/repo/trail "),
            "trimming made git's answer differ from the directory we asked about, \
             so the repository was classified as somebody else's — and that arm of \
             build.rs is the one that returns completely silently"
        );

        // ⚠ THIS IS `str::lines`' JOB, NOT OURS. It splits on `\r\n` and drops the
        // `\r`, so the `trim_end_matches('\r')` that used to sit here was dead for
        // every shape git emits — a survey deleted it with the suite still green —
        // and where it WAS live (a name ending in two CRs) it stripped more of a
        // legal directory name than git does. Gone. The property still deserves
        // pinning, because a Windows checkout really does produce this.
        let crlf = parse_git_paths(
            "/repo\r\n.git\r\n.git/hooks\r\n",
            std::path::Path::new("/repo"),
        )
        .expect("CRLF output must parse");
        assert_eq!(crlf.toplevel, PathBuf::from("/repo"));
        assert_eq!(
            crlf.hooks,
            PathBuf::from("/repo/.git/hooks"),
            "a CR survived into a resolved path, so every answer carries one"
        );
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
