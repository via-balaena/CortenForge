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
    /// A hooks directory outside this repository altogether. Often a GLOBAL
    /// `core.hooksPath`, which is shared with every other repo on the machine — but
    /// ⚠ classification is by LOCATION, not ownership, so a repo-LOCAL
    /// `core.hooksPath = ../hooks` lands here too. Say WHERE, never WHOSE: three
    /// other sites had to correct exactly that wording, and a test asserts the
    /// warning does not claim the directory is shared.
    Shared(std::path::PathBuf),
    /// Inside this checkout, but spelled so that GIT ITSELF will not exec the file
    /// we would write there — `core.hooksPath` of `.`, `./`, `""`, or anything
    /// starting with `-` or `+`. Installing is worse than installing nowhere: it
    /// either makes git refuse every commit, or leaves the hook silently unread.
    ///
    /// ⚠⚠ THE ONLY VARIANT THAT IS ABOUT GIT'S LIMIT RATHER THAN OUR PERMISSION.
    /// The other three answer "is this ours to write?"; this one answers "will the
    /// thing we write actually run?" — which is why it cannot be decided from the
    /// path in the payload: `.` and an absolute path to the same directory resolve
    /// identically and git runs one and not the other. Nor can it be decided from
    /// the directory ANSWER — `.` and `./.` give the same answer and differ in
    /// outcome. It is decided by asking git for the path it will exec; see
    /// [`git_will_exec_our_hook`].
    GitCannotRun(std::path::PathBuf),
}

/// The four answers one `git rev-parse` call gives about a checkout, plus the one
/// property that is read off them rather than stored.
#[derive(Debug, Clone, PartialEq, Eq)]
struct GitPaths {
    /// The working tree root git resolves for us.
    pub toplevel: std::path::PathBuf,
    /// The shared git dir — the MAIN repo's `.git` when we are in a worktree.
    pub common_dir: std::path::PathBuf,
    /// Where git will actually look for hooks.
    pub hooks: std::path::PathBuf,
    /// Whether git can EXEC a hook from that answer, which is not the same question
    /// as where the answer points. See [`git_will_exec_our_hook`].
    pub hooks_runnable: bool,
}

/// Parse `git rev-parse --show-toplevel --git-common-dir --git-path hooks --git-path
/// hooks/pre-commit` — see [`REV_PARSE_ASK`] — or `None` if git did not really answer.
///
/// `--show-toplevel` is absolute on every git. The other three are USUALLY relative to
/// the directory the command ran in, which is `base`, and `absolutize` resolves them
/// the way `--path-format=absolute` would.
///
/// ⚠ "Usually" is not "always", and the exception is the case this change exists for:
/// measured, a LINKED WORKTREE answers with an ABSOLUTE common dir and hooks path,
/// and so does an absolute `core.hooksPath`. Nothing here may assume relative —
/// `push_component` lets a `RootDir` replace what came before, which is what makes
/// both shapes land correctly. An earlier version of this sentence, and one in
/// docs/INFRASTRUCTURE.md, stated "relative … on every version" as a rule.
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
/// ⚠⚠ THE COUNT RULE IN [`four_answers`] IS THE WHOLE DEFENCE against a garbled
/// answer, and that is measured, not assumed. `rev-parse` echoes an option it does
/// not understand as an EXTRA LINE IN POSITION — it never replaces an answer — so
/// any echo makes a FIFTH line and is refused on count before anything inspects the
/// paths.
///
/// ⚠ A `toplevel.is_absolute()` rule lived here briefly and was deleted for catching
/// nothing. It is the FOURTH of five such rules on this arc; they are catalogued once,
/// in [`four_answers`], rather than retold at each site they touched.
///
/// ⚠⚠ THE ECHO HAZARD IS NOT GONE, it is just no longer reachable by a flag WE
/// choose. `rev-parse` still echoes any option it does not understand and still
/// exits 0. `--git-common-dir` and `--git-path` shipped IN git 2.5 (2015), so only
/// something older echoes one — and measured against a faithful shim, such a git
/// reads the trailing `hooks` as a REVISION. With no ref by that name it exits 128
/// and `ask_git` discards the output before it is parsed at all.
///
/// ⚠ THAT 128 IS NOT THE GUARANTEE, and leaning on it would be a mistake: in a
/// repository with a BRANCH named `hooks`, the same git exits 0 and emits an extra
/// line — the branch's SHA where an answer should be. Nothing is discarded, and what
/// refuses it is the COUNT rule in [`four_answers`]. A repo with a `hooks` branch is
/// not exotic. Measured against a pre-2.5 shim — and note the count that shim
/// produces moved when the fourth question was added, which is the whole reason this
/// paragraph names the RULE rather than a number.
///
/// ⚠ KNOWN GAP, one release wide. `core.hooksPath` arrived in 2.9, so the
/// `.git/hooks` guess is the right answer for anything older. But `rev-parse
/// --git-path` did not HONOUR `core.hooksPath` until 2.10 (upstream `9445b492`,
/// after v2.9.0 — its own message says `git_path()` was forgotten while
/// `run-command.c` was special-cased). On 2.9.x we therefore install into
/// `.git/hooks` and report success while git runs hooks from `core.hooksPath`.
/// Documented rather than worked around: the window is one 2016 minor release, and
/// closing it would mean a version check in a module that deliberately has none.
///
/// ⚠⚠ JOINING IS NOT ENOUGH — it must also NORMALISE. `Path::starts_with` is
/// component-wise, so a surviving `..` walks straight through containment. Measured:
/// with `core.hooksPath = ../shared-hooks`, git answers `Shared(<lab>/shared-hooks)`
/// and refuses it, while a bare join answered `<repo>/../shared-hooks` and was
/// accepted as `Repo`. An in-repo symlink pointing out of the tree escaped the same
/// way, as did a symlink CHAIN one hop further out.
#[must_use]
fn parse_git_paths(stdout: &str, base: &std::path::Path) -> Option<GitPaths> {
    let [toplevel, common_dir, hooks, exec] = four_answers(stdout)?;
    let hooks = absolutize(base, hooks);
    Some(GitPaths {
        toplevel: absolutize(base, toplevel),
        common_dir: absolutize(base, common_dir),
        // ⚠ FROM THE FOURTH ANSWER, not from `hooks`. The directory answer cannot
        // decide this — see `git_will_exec_our_hook`.
        hooks_runnable: git_will_exec_our_hook(exec, &hooks, base),
        hooks,
    })
}

/// Will git EXEC the very file we are about to write?
///
/// ⚠⚠ NOT THE SAME QUESTION AS "where do hooks live?", and installing where the
/// answer is no is worse than installing nowhere: a checkout that committed fine
/// beforehand refuses EVERY commit afterwards, with an error naming neither
/// CortenForge nor `core.hooksPath`.
///
/// ⚠⚠ THIS IS THE SECOND DESIGN. The first computed the property from the hooks
/// DIRECTORY answer, and that input cannot decide it — measured, `core.hooksPath`
/// of `.` and of `./.` both make `--git-path hooks` answer `.`, and git refuses the
/// commit under one and runs the hook under the other. git's own path cleanup strips
/// exactly one leading `./`, and it strips it from the directory answer and from the
/// exec path independently, so the directory answer is one cleanup behind the string
/// git actually runs. No function of it can be right.
///
/// ★ SO WE ASK GIT FOR THE STRING IT RUNS. `--git-path hooks/pre-commit` returns the
/// exact path git will exec, cleanup already applied — the same move this whole arc
/// is built on, one level down. Three properties of THAT answer decide it, and each
/// was measured against real commits rather than reasoned about:
///
/// 1. **It must contain a `/`.** `core.hooksPath = .` yields the bare word
///    `pre-commit`; `execvp` searches PATH and the commit dies with `error: cannot
///    run pre-commit: No such file or directory`.
/// 2. **It must not start with `-` or `+`.** `execve` succeeds into the KERNEL's
///    shebang handler, which passes the script path to the interpreter as its first
///    argument — so `/bin/sh` is handed `-hooks/pre-commit` and parses it as an
///    OPTION STRING (`/bin/sh: -/: invalid option`). POSIX shells take `+` the same
///    way (`set +x`), so `+hooks` fails identically. ⚠ It is the INTERPRETER that
///    rejects it, not a shell parsing a command line: git execs directly, and a
///    `core.hooksPath` containing `;`, spaces, `*`, `$` or a backtick all run fine.
///    A `#!/usr/bin/env python3` hook fails differently again — python reads `-h`,
///    prints usage, exits 0, and the guard is silently disarmed instead.
/// 3. **It must be the file we are about to write.** `core.hooksPath = ""` — which a
///    script setting it from an unset variable produces — makes git answer `./` for
///    the directory and `/pre-commit` for the exec path. We would install at the repo
///    root while git looks at the filesystem root: commits succeed, the hook never
///    runs, and "Installed" is a lie. Comparing the two answers is what catches it.
///
/// ⚠ THE COMPARISON IS ON THE PARENT, NOT THE WHOLE PATH, and that is deliberate:
/// [`absolutize`] canonicalises each component, so resolving the file itself would
/// follow a hook that is a SYMLINK to its target and report a mismatch for a setup
/// that works perfectly.
///
/// ★ VALIDATED AS A PREDICATE, not as three hunches — but read this as MOTIVATION,
/// not as evidence, on the same standard as the differential above: the sweep is not
/// in the tree. What is EVIDENCE is the nine shapes the live gate re-measures against
/// real git on every run. The sweep ran 20 `core.hooksPath` spellings against real
/// git — `.` `./` `./.` `././` `././.` `-hooks` `+hooks`
/// `.githooks` `./nested` `sub/-hooks` `sub/+hooks` `""` `hooks` `..` `" githooks"`
/// `"githooks "` `-` `+` `./-hooks` `./+hooks` — installing an always-passing hook
/// where git said the directory was, then committing. This function agrees with git
/// on all 20. The first design disagreed on SIX — it refused `./.`, `././` and
/// `././.`, which git runs, and accepted `+hooks`, `+` and `./+hooks`, which git
/// cannot exec. (An earlier version of this paragraph said "four", which is its own
/// small argument for keeping the sweep OUT of the prose and IN the gate.)
///
/// ⚠⚠ THIS IS NOT THE DELETED `-` FILTER COMING BACK (catalogue: [`four_answers`]).
/// That filter DROPPED the line,
/// so the answer was lost, two lines were left, and the `.git/hooks` guess reported a
/// successful install into a directory git does not read — a false success. This
/// keeps the answer, names the directory git really chose, and refuses it out loud.
#[must_use]
fn git_will_exec_our_hook(exec: &str, hooks_dir: &std::path::Path, base: &std::path::Path) -> bool {
    if !exec.contains('/') {
        return false;
    }
    if exec.starts_with('-') || exec.starts_with('+') {
        return false;
    }
    // ⚠ `Path::parent`, not `rsplit_once('/')`. The parent of `/pre-commit` is `/`,
    // and string surgery would call it "" — which resolves back to `base` and would
    // ACCEPT the empty-`core.hooksPath` case this condition exists to refuse.
    std::path::Path::new(exec)
        .parent()
        .and_then(|p| p.to_str())
        .is_some_and(|parent| absolutize(base, parent) == hooks_dir)
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

/// Exactly four answer lines, or nothing.
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
/// an executable hook in both candidates and committing. Re-measured under the
/// four-question ask, `core.hooksPath = "hooks\n"` answers SIX lines: the empty one
/// lands in position four, and the exec answer is itself split across lines five and
/// six (`hooks` / `/pre-commit`). Dropping empties leaves five, which is still
/// refused — but the historical bug was real: under the old three-question ask the
/// same filter left a well-formed THREE and installed into the TRUNCATED
/// `<repo>/hooks` while git read `<repo>/hooks\n`.
///
/// ⚠ The numbers in this paragraph moved when the fourth question was added, and an
/// earlier version of it kept the old ones. Quote the RULE, not a count.
///
/// ★★ THE CATALOGUE OF DELETED RULES LIVES HERE, and only here. FIVE defensive rules
/// have been added and deleted on this arc:
///
/// | rule | what it ate |
/// |---|---|
/// | `-` prefix filter | `core.hooksPath = -hooks` |
/// | exact-match option filter | `core.hooksPath = --git-path` |
/// | empty-line filter | defeated this very count rule |
/// | `toplevel.is_absolute()` in [`parse_git_paths`] | nothing |
/// | `file_name` check in [`git_will_exec_our_hook`] | nothing |
///
/// Every one was added to catch a shape someone REASONED about; every one was
/// measured afterwards to catch nothing, and three of the five ate a legal answer,
/// each the same way: a real answer removed, a short count, no answer, and the
/// `.git/hooks` guess reporting a successful install into a directory git never
/// reads. ⇒ **A defensive rule that cannot be shown to catch something the primary
/// rule misses is not free insurance; it is an untested code path with its own
/// failure mode.**
///
/// ⚠ The BENEFIT the option-shaped filters promised was unreachable on every git that
/// exists: a git that echoes one of our options is, by construction, a git that
/// cannot answer it, so the shape they would rescue (an echo PLUS a full set of good
/// answers) cannot occur. Measured with a faithful pre-2.5 shim.
///
/// ⇒ NOTHING IS DROPPED FOR LOOKING LIKE AN OPTION. If an echo ever did reach here,
/// THE COUNT RULE BELOW is what refuses it — nothing in this module inspects what a
/// line LOOKS like any more.
///
/// ⚠ The `trim_end_matches('\r')` above is deliberately NOT in that table, and the
/// distinction is worth holding: it was a NORMALISER, deleted for being redundant
/// with `str::lines`, not a rule added to refuse an answer. The five above all
/// REFUSED something; that is the class with the failure mode.
fn four_answers(stdout: &str) -> Option<[&str; 4]> {
    let lines: Vec<&str> = stdout.lines().collect();
    let [toplevel, common_dir, hooks, exec] = lines.as_slice() else {
        return None;
    };
    Some([toplevel, common_dir, hooks, exec])
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
    if !paths.hooks.starts_with(&paths.common_dir) && !paths.hooks.starts_with(&paths.toplevel) {
        return HooksDir::Shared(paths.hooks.clone());
    }
    // ⚠⚠ CONTAINMENT FIRST, THEN RUNNABILITY, AND THE ORDER IS A CLAIM. Five doc
    // sites said `GitCannotRun` means "ours, and git still will not run it" — and
    // with the runnable check first that was false: `core.hooksPath = -lnk`, where
    // `-lnk` is a symlink out of the tree, resolved OUTSIDE the checkout and still
    // came back `GitCannotRun`. Measured. Containment first makes the claim true by
    // construction rather than by comment.
    //
    // ★ It also hands that developer the more durable reason. "Outside this
    // repository" survives re-spelling the path; "git cannot exec this" does not, so
    // reporting runnability first would send them to fix the spelling and then meet
    // the containment refusal anyway — two round trips for one mistake.
    //
    // ⚠ The working-tree arm accepts the repository ROOT itself, and that used to be
    // pinned by a test whose comment called it a considered decision: "that is
    // genuinely where git reads hooks under that setting". It was false — git cannot
    // exec a hook from a relative `.` — and the runnable check below is what now
    // catches it. An ABSOLUTE `core.hooksPath` naming the top level does work, and is
    // still accepted; the two hooks then appear as untracked files in the root.
    if !paths.hooks_runnable {
        return HooksDir::GitCannotRun(paths.hooks.clone());
    }
    HooksDir::Repo(paths.hooks.clone())
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
/// 20.04 ships 2.25) it is not free: that git does not know the flag and echoes it
/// as an EXTRA line in position, so the answer is FIVE lines and the COUNT rule in
/// [`four_answers`] refuses it. (It is not a SHIFT — an echo never replaces an
/// answer. Saying "shift" here pointed at a `toplevel.is_absolute()` rule that no
/// longer exists, and was wrong about the mechanism even while it existed.) The
/// result is safe but WRONG — resolution falls back to `.git/hooks` and a
/// `core.hooksPath` is silently lost.
///
/// Every option here shipped in git 2.5 (2015) or earlier — and `--git-path` is
/// passed TWICE, which the sentence above does not cover on its own. Measured: git
/// parses options in a per-argument loop with no single-value slot, so repeats
/// cannot collapse; four `--git-path` operands come back as four lines in argument
/// order. ⚠ A hypothetical git that honoured only the FIRST would give three lines,
/// which [`four_answers`] refuses — safe, and a silent loss of `core.hooksPath` to
/// the `.git/hooks` guess.
const REV_PARSE_ASK: [&str; 7] = [
    "rev-parse",
    "--show-toplevel",
    "--git-common-dir",
    "--git-path",
    "hooks",
    // ⚠ THE FOURTH ANSWER IS NOT A LUXURY. It is the only thing that can tell
    // `core.hooksPath = .` (git refuses every commit) from `./.` (git runs the hook)
    // — they give the SAME directory answer. See `git_will_exec_our_hook`.
    //
    // ⚠ The hook NAME here is only a probe. Every name shares the `core.hooksPath`
    // prefix, and the properties `git_will_exec_our_hook` reads are functions of that
    // prefix alone, so any hook name gives the same verdict BY CONSTRUCTION. (A
    // 40-spelling sweep across three names agreed, but it is not in the tree; the
    // construction argument is the reason to believe it, not the sweep.) It must still name a hook we actually manage, which cannot be
    // expressed as a `const` (there is no const string concatenation) and is
    // therefore asserted against [`HOOKS`] in
    // `the_ask_carries_no_option_an_old_git_would_echo`.
    "--git-path",
    "hooks/pre-commit",
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
/// a `foreach` that builds CortenForge elsewhere inherits it.
///
/// ⚠⚠ CLEARING THESE DOES NOT MAKE THE ANSWER DEPEND ONLY ON THE DIRECTORY, and an
/// earlier version of this sentence claimed it did. Measured, in one directory:
/// `GIT_CONFIG_COUNT`/`GIT_CONFIG_KEY_0`/`GIT_CONFIG_VALUE_0`, `GIT_CONFIG_PARAMETERS`,
/// `GIT_CONFIG_GLOBAL` and `GIT_CONFIG_NOSYSTEM` each move the verdict — one of them
/// to an `Install` into a directory git does not read, and `GIT_CONFIG_GLOBAL=/dev/null`
/// hides a real global `core.hooksPath`. `GIT_CONFIG_PARAMETERS` is exported by git to
/// every hook whenever the outer command carried `-c`, so it is as reachable as
/// `GIT_DIR` was.
///
/// ★ They are NOT cleared, deliberately. `GIT_DIR` redirects us into ANOTHER
/// repository — a containment failure. These only change which config git reads for
/// THIS one, which is a developer configuring their own tools; clearing them would
/// mean overriding a setting someone deliberately passed. The narrower true claim is
/// the one that matters: the REPOSITORY we answer for depends only on the
/// directory.
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
        // ⚠⚠ THE ONE REFUSAL THAT IS NOT ABOUT OWNERSHIP. We are entitled to write
        // here — it is our own checkout — and installing anyway would break the
        // developer's repository rather than merely fail to protect it, so the
        // message has to say that plainly or it reads as bureaucracy.
        Some(HooksDir::GitCannotRun(dir)) => BuildOutcome::Warn(format!(
            "git resolves hooks to {}, but the path git would actually EXEC is not \
             the file we would write. core.hooksPath is a bare `.`/`./` (git \
             normalises it away and searches PATH), starts with `-` or `+` (the \
             hook's interpreter reads it as options), or is empty (git looks at the \
             FILESYSTEM root). Installing would either make git refuse EVERY commit \
             in this checkout or leave the hook silently unread, so CortenForge's \
             hooks were NOT installed and the scan/mesh guard is not armed. Naming \
             that same directory as an ABSOLUTE path works.",
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
///
/// ⚠⚠ `metadata` FOLLOWS a symlink, and that is REQUIRED, not incidental. git execs
/// through the link, so the mode that decides "will git run this?" is the TARGET's.
/// Measured: a link at mode 0644 whose target is 0755 — the hook RUNS; a link at 0777
/// whose target is 0644 — git skips it and the commit succeeds unguarded. Switching
/// to `symlink_metadata` would read the LINK's own mode, which macOS reports as
/// `120755`, so `& 0o111` is always true and every symlinked hook would be declared
/// fine — silently, which is the exact failure this function exists to detect. That
/// one-token change survived a whole mutation round; it is gated now.
///
/// ★ This is deliberately a different question from the one [`make_executable`] asks.
/// "Will git run it" is about the target; "is it ours to chmod" is about the path.
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
/// ⚠⚠ AND THAT IS EXACTLY WHY A LINK MUST NOT BE CHMODDED. `set_permissions` FOLLOWS
/// the link — there is no portable `lchmod` — so "repair the hook's bit" would change
/// the mode of whatever the link points AT: a file at an arbitrary path that
/// containment never looked at, because containment is computed for the hooks
/// DIRECTORY. Measured: with `.git/hooks/pre-commit` a link to a mode-0600 file
/// outside the repository, a plain `cargo build` widened it to 0755.
///
/// ⚠ BE EXACT ABOUT THE POPULATION, because an earlier version of this paragraph was
/// not. The chmod is only ever reached for [`HookState::OursCurrent`] — `Missing` and
/// `OursStale` go down the write path, `Foreign` and `Unreadable` are left alone — so
/// the target has to hold a byte-identical copy of our CURRENT hook text. "A
/// mode-0600 file outside the repository" overstated it; a copy of our own hook,
/// stored outside the repo and linked in, is the shape.
///
/// ⚠ AND ONE ESCAPE IS DELIBERATELY LEFT OPEN: a HARD link. `set_permissions` on it
/// changes a file with another name elsewhere, and no test refuses it. That is not
/// the same defect. Chmodding through a SYMLINK changes a file that is not the hook;
/// chmodding a hard link changes the hook, which merely has a second name — the
/// developer who made that link asked for exactly that. Refusing would punish an
/// intentional setup to defend a claim about names rather than about files. So the
/// honest scope of this guard is: no write here ever lands on a DIFFERENT file from
/// the one at the hook path.
///
/// ★ [`is_executable`] deliberately still FOLLOWS the link, and that is not an
/// inconsistency: the question it asks is "will git run this?", and git execs through
/// the link. The decision was always right — only the write was unbounded. So a
/// non-executable link is still reported rather than repaired; the old code did print
/// a repair message, it just never said whose file's mode it had changed.
///
/// # Errors
/// If the path cannot be stat-ed, if it is a symlink, or if the mode cannot be
/// changed.
#[cfg(unix)]
pub fn make_executable(path: &std::path::Path) -> std::io::Result<()> {
    use std::os::unix::fs::PermissionsExt;
    if path.symlink_metadata()?.file_type().is_symlink() {
        // ⚠ RESOLVED AGAINST THE HOOK'S OWN DIRECTORY, because `read_link` returns
        // the target exactly as written and these links are normally RELATIVE. The
        // raw form (`../../xtask/hooks/pre-commit`) is relative to the hooks dir,
        // while the developer reading this message is standing in the repo root,
        // where it points two levels above the checkout — at nothing. An unusable
        // path in a message whose only instruction is "chmod it yourself".
        // `absolutize` is the module's own resolver, so the path printed here is
        // normalised the same way every other path in this module is — no `..`
        // left in a sentence whose whole purpose is to be pasted after `chmod +x`.
        let target = std::fs::read_link(path).map(|t| {
            let parent = path.parent().unwrap_or(std::path::Path::new("."));
            t.to_str()
                .map_or_else(|| parent.join(&t), |s| absolutize(parent, s))
        });
        let said = match target {
            // ⚠ Says WHICH FILE, never WHOSE. An earlier version asserted the target
            // was "outside this repository" unconditionally, having checked nothing —
            // and the commonest link of all, to this repo's own tracked
            // `xtask/hooks/pre-commit`, made that simply false.
            Ok(t) => format!(
                "it is a symlink, so making it executable would change the \
                 permissions of {} instead — chmod that file yourself if you want \
                 git to run this hook",
                t.display()
            ),
            Err(e) => format!(
                "it is a symlink whose target could not be read ({e}), so making it \
                 executable would change the permissions of some other file"
            ),
        };
        return Err(std::io::Error::new(std::io::ErrorKind::InvalidInput, said));
    }
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

    /// git's four answers for a checkout whose hooks directory answer is `hooks`.
    ///
    /// ⚠ The fourth is `--git-path hooks/pre-commit`, and for an ORDINARY answer it
    /// really is the third with the hook name appended — measured across
    /// `.githooks`, `./nested`, `hooks`, `..`, `sub/-hooks` and others. This helper
    /// is only for those. The degenerate spellings, where git's cleanup makes the
    /// two answers disagree (`.` answers `pre-commit`, `""` answers `/pre-commit`),
    /// are written out literally at their own tests, because the disagreement IS the
    /// thing under test and a helper that manufactured it would be marking its own
    /// homework.
    fn ask(top: &str, common: &str, hooks: &str) -> String {
        format!("{top}\n{common}\n{hooks}\n{hooks}/pre-commit\n")
    }

    const OURS: &str = "#!/bin/sh\n# CortenForge Pre-Commit Hook\necho hi\n";
    const MARKER: &str = "CortenForge Pre-Commit Hook";

    /// ⚠ THE PAIRING. Crossing the two entries of `HOOKS` installs the commit-msg
    /// script as `pre-commit`, which disarms the scan/mesh guard for everyone whose
    /// hooks come from `cargo build` — the majority path. It was measured, not
    /// imagined: before the `HOOKS` table existed, `build.rs` wrote the pairing out
    /// itself and crossing its two consts shipped green.
    ///
    /// ⚠ NOT THE ONLY GATE, and an earlier version of this comment claimed it was.
    /// `the_installer_writes_each_hook_to_its_own_filename` in `setup.rs` drives the
    /// installer and would fail on the same swap. Both comments said "every other
    /// test exercises `PRE_COMMIT_HOOK` directly", so each implied it was load-bearing
    /// alone — delete either on the strength of its neighbour's prose and you would be
    /// wrong. This one is the cheaper, more direct assertion on the TABLE.
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

    /// An ECHOED option is refused; a legitimate answer that looks like a flag is not.
    ///
    /// ⚠ `rev-parse` echoes any option it does not understand and still exits 0 —
    /// as an EXTRA line IN POSITION, never in place of an answer, so what arrives is
    /// FIVE lines and the count rule in `four_answers` refuses it. That is the whole
    /// defence, and it is the only one: nothing inspects what a line looks like.
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
    fn an_echoed_option_is_refused_on_count_and_a_flag_shaped_answer_is_not() {
        use super::parse_git_paths;
        use std::path::{Path, PathBuf};

        let base = Path::new("/repo");

        let ordinary = parse_git_paths(&ask("/repo", ".git", ".git/hooks"), base)
            .expect("toplevel absolute, the rest relative — what git actually emits");
        assert_eq!(ordinary.toplevel, PathBuf::from("/repo"));
        assert_eq!(ordinary.hooks, PathBuf::from("/repo/.git/hooks"));

        // ★ THE REAL ECHO SHAPE, measured on git 2.50: an unrecognised option is
        // echoed as an EXTRA LINE IN POSITION — it never replaces an answer — so it
        // makes a FIFTH line and is refused on count. An earlier version of this test
        // fed a hand-built THREE-line string with the flag first, which no git
        // emits, and used it to justify a `toplevel.is_absolute()` rule that
        // therefore never fired. A fixture no git can produce tests nothing.
        assert!(
            parse_git_paths(
                &format!(
                    "/repo\n--some-option\n{}",
                    ask("", ".git", ".git/hooks").trim_start_matches('\n')
                ),
                base
            )
            .is_none(),
            "an echoed option makes a FIFTH line; taking four of five silently \
             shifts which answer is which"
        );

        // ★★ AND THE COST SIDE. These are real answers that a filter would eat.
        // ⚠ `hooks` is included deliberately: it is the ARGUMENT we pass to
        // `--git-path`, not an option we ask for, and "the options we ask for" is
        // ambiguous enough that a filter listing it would eat a legal answer.
        for hooks_answer in ["-hooks", "--git-path", "--show-toplevel", "hooks"] {
            let paths =
                parse_git_paths(&ask("/repo", ".git", hooks_answer), base).unwrap_or_else(|| {
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
             echoes it as an extra line, making FIVE, and silently loses a \
             core.hooksPath to the .git/hooks fallback: {ask:?}"
        );
        // POSITIVE CONTROL: it must still ask the questions resolution needs,
        // or "carries no bad option" would be satisfied by asking nothing.
        for required in ["--show-toplevel", "--git-common-dir", "--git-path"] {
            assert!(
                ask.contains(&required),
                "{required} is what containment is computed from: {ask:?}"
            );
        }
        // ★ AND THE PROBE, which is the only thing that can tell `core.hooksPath = .`
        // from `./.` — they give the same directory answer and git runs one of them.
        // Dropping it does not break containment, so nothing else here would notice.
        let probe = ask
            .iter()
            .find(|a| a.starts_with("hooks/"))
            .unwrap_or_else(|| panic!("the exec-path probe is gone from the ask: {ask:?}"));
        let probed_name = probe.trim_start_matches("hooks/");
        assert!(
            HOOKS.iter().any(|(name, _)| *name == probed_name),
            "the probe must name a hook this installer actually manages, or it \
             measures the runnability of a file git will never be asked for: \
             {probed_name:?}"
        );
    }

    /// EXACTLY four, which has a lower side as well as an upper one.
    ///
    /// ⚠ Not hypothetical, and not only about echoed options: `core.hooksPath` may
    /// contain a NEWLINE — git 2.50 accepts it — and the answer then arrives split
    /// across two lines. Taking the first four would install into a TRUNCATED
    /// directory, which `setup` would happily create. The count check is what the
    /// module's own doc leans on, and nothing exercised it.
    #[test]
    fn only_exactly_four_answers_is_an_answer() {
        use super::{four_answers, resolve_from_answers, HooksDir};
        use std::path::{Path, PathBuf};

        assert!(
            four_answers("/r\n.git\n.git/hooks\n.git/hooks/pre-commit\n/extra\n").is_none(),
            "a fifth line means something answered that we did not ask, so the \
             four we read may not be the four we wanted"
        );
        // ★★ AND THE SHAPE A NEWLINE IN `core.hooksPath` REALLY MAKES. git 2.50
        // stores `hooks\n` and RUNS hooks from the directory whose name ends in that
        // newline — measured by putting an executable hook in both candidates and
        // committing. The fixture below is that answer, re-measured under the
        // four-question ask: SIX lines, the empty one in position FOUR, and the exec
        // answer split across the last two. An `!l.is_empty()` filter dropped the
        // empty, leaving a well-formed count, and installed into the TRUNCATED
        // `<repo>/hooks` while git read `<repo>/hooks\n`.
        //
        // ⚠ The previous fixture here was `\"…hooks\\nhooks/pre-commit\\n\\n\"` — an
        // empty line LAST, which no git emits under this ask. A fixture no producer
        // can produce tests nothing, which this module says 100 lines above.
        //
        // ⚠ BE HONEST ABOUT WHAT THE COUNT RULE BUYS HERE. It prevents the TRUNCATED
        // install; it does not rescue the developer. Refusing sends resolution to the
        // `.git/hooks` guess, which is a different wrong directory — reported as
        // "Installed" while git reads the newline-named one. The guard moves the
        // failure from silent-and-truncated to silent-and-elsewhere.
        assert!(
            four_answers("/r\n.git\nhooks\n\nhooks\n/pre-commit\n").is_none(),
            "the real six-line shape must be refused; dropping the empty line \
             installs into a truncated directory that git does not read"
        );
        // ★ AND THE LOWER SIDE, which nothing reached: THREE lines. That is exactly
        // what the previous three-question ask produced, so it is the shape a git
        // that honoured only the first `--git-path` would emit — and the doc calls
        // this rule "the whole defence". Accepting three would read the hooks answer
        // as the exec path and decide runnability from the wrong string.
        assert!(
            four_answers("/r\n.git\n.git/hooks\n").is_none(),
            "three lines is the OLD ask's shape; accepting it reads the hooks answer \
             as the exec path"
        );
        // POSITIVE CONTROL: exactly four is still an answer.
        assert!(four_answers("/r\n.git\n.git/hooks\n.git/hooks/pre-commit\n").is_some());

        // `core.hooksPath = "ho\noks"` — one setting, two lines.
        assert_eq!(
            resolve_from_answers(
                Some("/r\n.git\n/r/ho\noks\n/r/ho\noks/pre-commit\n"),
                Path::new("/r"),
                true
            ),
            Some(HooksDir::Repo(PathBuf::from("/r/.git/hooks"))),
            "a hooks path containing a newline splits git's answer; reading the \
             first four lines installs into a truncated directory"
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
    fn a_symlink_cycle_terminates_and_stays_inside_the_checkout() {
        let lab = std::env::temp_dir().join(format!("cf-cycle-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&lab);
        let repo = lab.join("repo");
        std::fs::create_dir_all(repo.join(".git")).expect("temp dir");
        let repo = std::fs::canonicalize(&repo).expect("canonicalize");
        std::os::unix::fs::symlink("b", repo.join("a")).expect("symlink");
        std::os::unix::fs::symlink("a", repo.join("b")).expect("symlink");

        let verdict = |hooks: &str| {
            super::parse_git_paths(&ask(&repo.display().to_string(), ".git", hooks), &repo)
                .expect("four answers")
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

        // `hooks_runnable: true` throughout: every case here spells the hooks path
        // absolutely, which is the form git can exec from. The false case is not a
        // property of these paths at all — it comes from the RAW answer — so it is
        // gated where it is decided, in
        // `a_hooks_path_git_cannot_exec_from_is_refused_rather_than_installed_into`,
        // rather than smuggled in here as a magic path string.
        let at = |top: &str, common: &str, hooks: &str| GitPaths {
            toplevel: PathBuf::from(top),
            common_dir: PathBuf::from(common),
            hooks: PathBuf::from(hooks),
            hooks_runnable: true,
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

        // ★ THE DEGENERATE SHAPE: a hooks answer that IS the working tree root. An
        // earlier revision asserted this was refused; the revision that widened
        // containment to the working tree deleted that assertion and replaced it
        // with nothing, so the behaviour flipped with no test either way.
        //
        // ⚠⚠ THE COMMENT THAT REPLACED IT WAS WORSE THAN NO COMMENT. It read
        // "accepted, because that is genuinely where git reads hooks under that
        // setting", pinning `core.hooksPath = .` as a considered decision — and it
        // was measured false on this very branch: git cannot exec a hook from a
        // relative `.`, so the guard was never armed, and installing there makes git
        // refuse every commit. A confident justification is not evidence, and this
        // one outlived its own refutation by four commits.
        //
        // What is true is narrower, so it is asserted narrowly: the root is
        // acceptable when git names it in a form git can exec from — an ABSOLUTE
        // core.hooksPath. The relative spelling is refused in
        // `a_hooks_path_git_cannot_exec_from_is_refused_rather_than_installed_into`.
        assert!(
            matches!(
                verdict(&at("/repo", "/repo/.git", "/repo"), "/repo"),
                HooksDir::Repo(_)
            ),
            "an ABSOLUTE core.hooksPath naming the repo root is a directory git runs \
             hooks from"
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

    /// Four `core.hooksPath` spellings name a file git itself will not exec.
    ///
    /// ⚠⚠ THE PREMISE IS RUN, NOT ASSERTED. Two claims about git on this arc were
    /// written confidently in a comment and later measured false, so each shape here
    /// makes real git prove the harm before the decision that avoids it is gated. If
    /// a future git starts executing these, the test fails on the PREMISE and tells
    /// the reader the rule can be deleted — rather than passing forever on a stale
    /// belief.
    ///
    /// ★ EVERY REFUSAL IS PAIRED WITH THE SAME DIRECTORY SPELLED ABSOLUTELY, which
    /// git runs. Without that pairing, "git cannot exec from here" would be
    /// satisfied by a broken fixture — a hook that never worked anywhere — and the
    /// rule would look justified while condemning the directory rather than the
    /// spelling.
    #[test]
    #[cfg(unix)]
    fn a_hooks_path_git_cannot_exec_from_is_refused_rather_than_installed_into() {
        use super::{resolve_hooks_dir, HooksDir};

        let lab = std::env::temp_dir().join(format!("cf-dothooks-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&lab);
        std::fs::create_dir_all(&lab).expect("temp dir");
        let repo = std::fs::canonicalize(&lab)
            .expect("canonicalize")
            .join("repo");
        std::fs::create_dir_all(&repo).expect("temp dir");

        // Every invocation goes through `git_command`, so the ambient GIT_* variables
        // are cleared here exactly as they are in production.
        let git = |args: &[&str]| {
            super::git_command(&repo, args)
                .output()
                .expect("git must be installed to run this test")
        };
        assert!(git(&["init", "-q"]).status.success(), "git init");
        git(&["config", "user.email", "t@example.invalid"]);
        git(&["config", "user.name", "t"]);
        git(&["config", "commit.gpgsign", "false"]);
        std::fs::write(repo.join("a.txt"), "one\n").expect("seed");
        git(&["add", "a.txt"]);
        assert!(
            git(&["commit", "-qm", "feat: one"]).status.success(),
            "the baseline commit must succeed, or the fixture proves nothing"
        );

        // An ALWAYS-PASSING hook: any commit failure below is git failing to run it,
        // never the hook rejecting the commit.
        let hook = repo.join("pre-commit");
        std::fs::write(&hook, "#!/bin/sh\necho HOOK-RAN\nexit 0\n").expect("seed hook");
        super::make_executable(&hook).expect("chmod");

        let commit = |file: &str, msg: &str| {
            std::fs::write(repo.join(file), "x\n").expect("seed");
            git(&["add", file]);
            let out = git(&["commit", "-qm", msg]);
            // ⚠ BOTH STREAMS. git connects a hook's stdout to its own STDERR, so
            // reading stdout alone reported "the hook never ran" for a hook that ran
            // perfectly — caught here only because the positive control asserts the
            // hook DID run rather than just that the commit succeeded.
            let ran = String::from_utf8_lossy(&out.stdout).contains("HOOK-RAN")
                || String::from_utf8_lossy(&out.stderr).contains("HOOK-RAN");
            (out.status.success(), ran)
        };

        // ⚠ WRITTEN INTO `.git/config` DIRECTLY, because `git config core.hooksPath
        // -hooks` cannot express one of the shapes under test: git parses the value
        // as an option. `git config core.hooksPath -- -hooks` does not help either —
        // it stores the literal `--`, and the first version of this fixture did
        // exactly that and "passed" while testing a setting nobody has. A later
        // `[core]` section wins, so each call appends one, and the read-back below
        // is what stops a silently ineffective fixture a second time.
        let set_hooks_path = |value: &str| {
            let cfg = repo.join(".git").join("config");
            let mut text = std::fs::read_to_string(&cfg).expect("read .git/config");
            text.push_str(&format!("\n[core]\n\thooksPath = {value}\n"));
            std::fs::write(&cfg, text).expect("write .git/config");
            let got = git(&["config", "--get", "core.hooksPath"]);
            assert_eq!(
                String::from_utf8_lossy(&got.stdout).trim(),
                value,
                "the fixture did not actually set core.hooksPath, so everything below \
                 it would test the default"
            );
        };

        set_hooks_path(".");
        let under_dot = commit("b.txt", "feat: two");
        let dot_verdict = resolve_hooks_dir(&repo);

        // CONTROL: the identical file, the identical directory, spelled absolutely.
        set_hooks_path(&repo.display().to_string());
        let under_absolute = commit("c.txt", "feat: three");

        // CONTROL: an ordinary in-repo relative path keeps its directory part, so git
        // execs it — this is why the rule is narrowed to a path that normalises away
        // to nothing rather than to "relative".
        std::fs::create_dir_all(repo.join(".githooks")).expect("temp dir");
        std::fs::rename(&hook, repo.join(".githooks/pre-commit")).expect("move hook");
        set_hooks_path(".githooks");
        let under_subdir = commit("d.txt", "feat: four");
        let subdir_verdict = resolve_hooks_dir(&repo);

        // ★ THE SECOND SHAPE: a leading `-`. The command git execs starts with a
        // dash, so the shell reads it as OPTIONS and prints its option list instead
        // of running the hook.
        let dashed = repo.join("-hooks");
        std::fs::create_dir_all(&dashed).expect("temp dir");
        std::fs::rename(repo.join(".githooks/pre-commit"), dashed.join("pre-commit"))
            .expect("move hook");
        set_hooks_path("-hooks");
        let under_dash = commit("e.txt", "feat: five");
        let dash_verdict = resolve_hooks_dir(&repo);

        // CONTROL, and the one that identifies the CAUSE: the very same `-hooks`
        // directory, named from `/`, runs. So the fault is the leading character of
        // the command git execs, not the directory.
        set_hooks_path(&dashed.display().to_string());
        let under_dash_absolute = commit("f.txt", "feat: six");
        let dash_absolute_verdict = resolve_hooks_dir(&repo);

        // ★ THE THIRD SHAPE: `+`. POSIX shells take `+opts` as well as `-opts`, so an
        // interpreter handed `+hooks/pre-commit` rejects it exactly as it rejects the
        // dash. The first version of this rule tested only `-` and installed happily
        // into `+hooks`, which is the same brick one character to the right.
        let plussed = repo.join("+hooks");
        std::fs::create_dir_all(&plussed).expect("temp dir");
        std::fs::rename(dashed.join("pre-commit"), plussed.join("pre-commit")).expect("move hook");
        set_hooks_path("+hooks");
        let under_plus = commit("g.txt", "feat: seven");
        let plus_verdict = resolve_hooks_dir(&repo);

        // ★★ AND THE TWO SHAPES THAT MUST BE ACCEPTED, which the first rule REFUSED.
        // `./.` and `././` resolve to the same directory as `.` and give the SAME
        // `--git-path hooks` answer — and git RUNS the hook under them. That is the
        // measurement that killed reading the directory answer: no function of it can
        // be right, because one answer maps to both outcomes.
        std::fs::rename(plussed.join("pre-commit"), repo.join("pre-commit")).expect("move hook");
        set_hooks_path("./.");
        let under_dot_dot = commit("h.txt", "feat: eight");
        let dot_dot_verdict = resolve_hooks_dir(&repo);
        set_hooks_path("././");
        let under_dot_slash = commit("i.txt", "feat: nine");

        // ★ AND THE EMPTY VALUE, which a script setting core.hooksPath from an unset
        // variable produces. git answers `./` for the directory but `/pre-commit` for
        // the exec path: we would install at the repo root while git looks at the
        // FILESYSTEM root. Commits succeed and the hook never runs — a false success,
        // which is the failure mode this whole arc exists to remove.
        set_hooks_path("");
        let under_empty = commit("j.txt", "feat: ten");
        let empty_verdict = resolve_hooks_dir(&repo);
        let _ = std::fs::remove_dir_all(&lab);

        // THE PREMISE, from git itself.
        assert_eq!(
            under_dot,
            (false, false),
            "git is expected to REFUSE the commit and never run the hook under \
             core.hooksPath=. — if this now succeeds, git gained the ability to exec \
             from a relative `.` and the GitCannotRun rule can be deleted"
        );
        assert_eq!(
            under_absolute,
            (true, true),
            "the same file in the same directory, named absolutely, must run — \
             otherwise the assertion above is satisfied by a hook that never worked"
        );
        assert_eq!(
            under_subdir,
            (true, true),
            "a relative path with a real component still execs; the rule must not \
             widen to every relative answer"
        );
        assert_eq!(
            under_dash,
            (false, false),
            "git is expected to REFUSE the commit under core.hooksPath=-hooks — the \
             shell reads the command as options"
        );
        assert_eq!(
            under_dash_absolute,
            (true, true),
            "the SAME -hooks directory named absolutely must run, or the refusal \
             above is about the directory rather than the spelling"
        );
        assert_eq!(
            under_plus,
            (false, false),
            "git is expected to REFUSE the commit under core.hooksPath=+hooks — \
             POSIX shells read `+` as an option introducer too"
        );
        assert_eq!(
            under_dot_dot,
            (true, true),
            "git RUNS the hook under `./.`, which gives the same directory answer as \
             `.` — refusing it is a regression, and this is the leg that proves the \
             directory answer cannot decide runnability"
        );
        assert_eq!(
            under_dot_slash,
            (true, true),
            "and under `././`, which gives the same directory answer as `./`"
        );
        assert_eq!(
            under_empty,
            (true, false),
            "an empty core.hooksPath does not brick the checkout — it silently fails \
             to run the hook, which is why it must be refused rather than installed"
        );

        // THE DECISION, which exists only because of the premise above.
        assert_eq!(
            dot_verdict,
            Some(HooksDir::GitCannotRun(repo.clone())),
            "core.hooksPath=. must be refused by name, not installed into"
        );
        assert_eq!(
            subdir_verdict,
            Some(HooksDir::Repo(repo.join(".githooks"))),
            "the positive control: an in-repo relative hooks path is still ours"
        );
        // ⚠ THE PAYLOAD, not just the variant. The deleted `-`-prefix filter also
        // declined to install into `-hooks` — by LOSING the answer and installing
        // into `.git/hooks` while reporting success. Naming the directory git really
        // chose is what separates this refusal from that bug.
        assert_eq!(
            dash_verdict,
            Some(HooksDir::GitCannotRun(dashed.clone())),
            "the refusal must carry the directory git chose, not fall back to a guess"
        );
        assert_eq!(
            dash_absolute_verdict,
            Some(HooksDir::Repo(dashed)),
            "absolute is the spelling that works, and it must still be ours"
        );
        assert_eq!(
            plus_verdict,
            Some(HooksDir::GitCannotRun(plussed)),
            "`+hooks` must be refused by name, not installed into"
        );
        assert_eq!(
            dot_dot_verdict,
            Some(HooksDir::Repo(repo.clone())),
            "`./.` must be ACCEPTED — git runs hooks from it"
        );
        assert_eq!(
            empty_verdict,
            Some(HooksDir::GitCannotRun(repo)),
            "an empty core.hooksPath must be refused: git execs /pre-commit while we \
             would install at the repo root"
        );
    }

    /// Containment is decided BEFORE runnability, and a symlinked hook is still ours.
    ///
    /// ⚠⚠ TWO CHOICES THAT LOOK LIKE STYLE AND ARE NOT. Both survived a mutation
    /// round — the code could be reordered, or the comparison widened, with the whole
    /// suite green — while five doc sites rested on them.
    ///
    /// 1. `GitCannotRun` claims "this directory IS ours, and git still will not run
    ///    what we write there". With the runnable check first that was FALSE:
    ///    `core.hooksPath = -lnk`, where `-lnk` is a symlink pointing out of the
    ///    tree, resolves OUTSIDE the checkout and is unrunnable for its leading dash,
    ///    so it came back `GitCannotRun` naming somebody else's directory. Measured
    ///    against git 2.50.1.
    /// 2. [`git_will_exec_our_hook`] compares the exec path's PARENT with the hooks
    ///    directory rather than comparing whole paths. Widening it to the whole path
    ///    breaks the commonest bespoke setup there is — a hook that is a SYMLINK —
    ///    because `absolutize` canonicalises the final component and follows it to
    ///    the target, so the two sides disagree for a configuration that works
    ///    perfectly. A loud false refusal, and nothing objected.
    #[test]
    #[cfg(unix)]
    fn containment_is_decided_before_runnability_and_a_symlinked_hook_is_ours() {
        use super::{classify_hooks_dir, parse_git_paths, HooksDir};

        let lab = std::env::temp_dir().join(format!("cf-order-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&lab);
        let repo = lab.join("repo");
        std::fs::create_dir_all(repo.join(".git").join("hooks")).expect("temp dir");
        std::fs::create_dir_all(lab.join("outside")).expect("temp dir");
        let repo = std::fs::canonicalize(&repo).expect("canonicalize");
        let outside = std::fs::canonicalize(lab.join("outside")).expect("canonicalize");

        // A dash-named symlink OUT of the tree: unrunnable AND outside, so the two
        // rungs disagree and the order is observable.
        std::os::unix::fs::symlink(&outside, repo.join("-lnk")).expect("symlink");
        let both_wrong = parse_git_paths(
            &format!("{}\n.git\n-lnk\n-lnk/pre-commit\n", repo.display()),
            &repo,
        )
        .expect("git emits exactly this shape");
        let outside_verdict = classify_hooks_dir(&both_wrong, &repo);

        // POSITIVE CONTROL for the same rung: unrunnable but INSIDE must still be
        // `GitCannotRun`, or "containment first" would be satisfied by deleting the
        // runnable check altogether.
        std::fs::create_dir_all(repo.join("-hooks")).expect("temp dir");
        let inside_unrunnable = parse_git_paths(
            &format!("{}\n.git\n-hooks\n-hooks/pre-commit\n", repo.display()),
            &repo,
        )
        .expect("git emits exactly this shape");
        let inside_verdict = classify_hooks_dir(&inside_unrunnable, &repo);

        // ★ THE SYMLINKED HOOK. The FILE git execs is a link to somewhere else
        // entirely; the DIRECTORY is ours, and that is what decides.
        std::fs::write(lab.join("outside").join("shared-hook"), "hook\n").expect("seed");
        std::os::unix::fs::symlink(
            outside.join("shared-hook"),
            repo.join(".git").join("hooks").join("pre-commit"),
        )
        .expect("symlink");
        let linked = parse_git_paths(
            &format!(
                "{}\n.git\n.git/hooks\n.git/hooks/pre-commit\n",
                repo.display()
            ),
            &repo,
        )
        .expect("git emits exactly this shape");
        let linked_verdict = classify_hooks_dir(&linked, &repo);
        let _ = std::fs::remove_dir_all(&lab);

        assert_eq!(
            outside_verdict,
            HooksDir::Shared(outside),
            "a hooks path that is BOTH outside the checkout and unrunnable must be \
             refused for being outside — that reason survives re-spelling the path, \
             and `GitCannotRun` would be claiming a directory that is not ours"
        );
        assert_eq!(
            inside_verdict,
            HooksDir::GitCannotRun(repo.join("-hooks")),
            "POSITIVE CONTROL: unrunnable and INSIDE is still GitCannotRun"
        );
        assert_eq!(
            linked_verdict,
            HooksDir::Repo(repo.join(".git").join("hooks")),
            "a hook that is a SYMLINK must not make its own directory unrunnable — \
             comparing whole paths instead of parents follows the link to its target \
             and refuses a setup that works"
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
        let paths = parse_git_paths(&ask("/repo", ".git", ".git/hooks"), base)
            .expect("relative answers are usable once rooted");
        assert_eq!(paths.common_dir, PathBuf::from("/repo/.git"));
        assert_eq!(paths.hooks, PathBuf::from("/repo/.git/hooks"));
        assert_eq!(
            paths.toplevel,
            PathBuf::from("/repo"),
            "join must leave an already-absolute answer alone"
        );

        // ★ AND THE OPPOSITE SHAPE, which no test reached: git does NOT always answer
        // relatively. Measured, a LINKED WORKTREE returns an ABSOLUTE common dir and
        // hooks path — and that is the case this whole change exists for, so it must
        // not depend on a "relative on every version" belief that the docs and this
        // module both used to state as a rule. `push_component` handles it because a
        // `RootDir` REPLACES what came before; a `join`-and-hope would too, but only
        // by accident, and nothing was asserting either way.
        let worktree = parse_git_paths(
            &ask("/wt", "/main/.git", "/main/.git/hooks"),
            Path::new("/wt"),
        )
        .expect("a worktree's absolute answers are an answer");
        assert_eq!(
            worktree.common_dir,
            PathBuf::from("/main/.git"),
            "an absolute common dir must not be appended to the base"
        );
        assert_eq!(worktree.hooks, PathBuf::from("/main/.git/hooks"));

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
            &ask(&repo.display().to_string(), ".git", "../shared-hooks"),
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
            &ask(
                &repo.display().to_string(),
                ".git",
                "nope/../../shared-hooks",
            ),
            &repo,
        )
        .expect("old git emits exactly this shape");
        let deep_verdict = classify_hooks_dir(&deep, &repo);

        // ★ And through a SYMLINK that leaves the tree: `..` must apply to where the
        // link LANDS, not to where it sits, which is why components are resolved one
        // at a time rather than lexically.
        std::os::unix::fs::symlink(lab.join("shared-hooks"), repo.join("lnk")).ok();
        let via_link = parse_git_paths(&ask(&repo.display().to_string(), ".git", "lnk"), &repo)
            .expect("old git emits exactly this shape");
        let link_verdict = classify_hooks_dir(&via_link, &repo);

        // POSITIVE CONTROL: a relative answer that stays INSIDE must still be ours,
        // or "refuses everything" would pass the assertion above.
        let inside = parse_git_paths(
            &ask(&repo.display().to_string(), ".git", ".git/hooks"),
            &repo,
        )
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
        let ordinary = ask("/nx-cf-rungs", ".git", ".git/hooks");
        let elsewhere = ask("/nx-cf-rungs", ".git", "/nx-cf-elsewhere/.githooks");

        assert_eq!(
            resolve_from_answers(Some(&ordinary), root, true),
            Some(HooksDir::Repo(PathBuf::from("/nx-cf-rungs/.git/hooks"))),
            "git's own answer must be used"
        );

        // ★ THE POINT. `dot_git_is_dir` is true, so the guess is available and would
        // happily return `<root>/.git/hooks`. git says the hooks are elsewhere, and
        // that has to win — otherwise the developer is told "Installed" about a file
        // git does not read.
        assert_eq!(
            resolve_from_answers(Some(&elsewhere), root, true),
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

    /// The build script's OWN refusals, which nothing reached.
    ///
    /// ⚠⚠ `cargo xtask setup`'s refusals were extracted into `hooks_dir_from` and
    /// tested precisely so a mutation could not gut them — while `build.rs`, the
    /// installer a FRESH CLONE actually runs, kept the same decisions as
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
        // ★ AND THE FOURTH, which is not about ownership: the directory IS ours, and
        // we decline anyway because installing would make git refuse every commit.
        // Mutating this arm into an install is the shape that reintroduces the bug,
        // and it is indistinguishable from `Repo` by payload alone.
        let unrunnable = build_outcome(
            Some(HooksDir::GitCannotRun(PathBuf::from("/repo"))),
            true,
            false,
        );
        let BuildOutcome::Warn(said) = unrunnable else {
            panic!("core.hooksPath=. must warn, not install: {unrunnable:?}");
        };
        assert!(
            said.contains("core.hooksPath") && said.contains("EVERY commit"),
            "the developer has to be told which setting to change and what installing \
             would have done to their checkout: {said}"
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

        let nested = parse_git_paths(&ask("/nx-cf-dot-repo", "./.git", "./nested"), base)
            .expect("four answers");
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
        let root = parse_git_paths("/nx-cf-dot-repo\n./.git\n.\npre-commit\n", base)
            .expect("four answers");
        assert_eq!(
            root.hooks.as_os_str(),
            OsStr::new("/nx-cf-dot-repo"),
            "`core.hooksPath = .` must resolve to the root itself, not `<root>/.`: {:?}",
            root.hooks
        );

        // POSITIVE CONTROL: an answer with no `.` is carried through untouched, so
        // this is not satisfied by a function that mangles every path.
        let plain = parse_git_paths(&ask("/nx-cf-dot-repo", ".git", ".git/hooks"), base)
            .expect("four answers");
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
            let paths = parse_git_paths(&ask(&repo.display().to_string(), ".git", hooks), &repo)
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
    /// write, against its own stated standard. Re-adding the filter survived every
    /// test in this module; measured on the pure functions, the filter turns the
    /// answer into `Repo(<repo>/.git/hooks)` — a successful-looking install into a
    /// directory git never reads.
    ///
    /// ⚠⚠ THE VERDICT CHANGED AND THE CLAIM DID NOT. `-hooks` is now `GitCannotRun`
    /// rather than `Repo`, because git measurably cannot exec from it — but that is
    /// the OPPOSITE of what the deleted filter did, and this test exists to keep the
    /// two apart. The filter LOST the answer and installed somewhere else while
    /// reporting success; this names `<repo>/-hooks` — the directory git really
    /// chose — and refuses it out loud. So the assertion is on the PAYLOAD as much as
    /// the variant: if the guess ever reappears here, the filter is back under a new
    /// name.
    #[test]
    fn a_dash_prefixed_hooks_path_is_an_answer_not_an_echoed_option() {
        use super::{resolve_from_answers, HooksDir};
        use std::path::Path;

        let root = Path::new("/nx-cf-dash-repo");
        let legacy = ask("/nx-cf-dash-repo", "/nx-cf-dash-repo/.git", "-hooks");
        assert_eq!(
            resolve_from_answers(Some(&legacy), root, true),
            Some(HooksDir::GitCannotRun(root.join("-hooks"))),
            "`core.hooksPath = -hooks` is legal and must be carried through to the \
             directory git chose; dropping its answer for the leading dash leaves two \
             lines, no answer, and the `.git/hooks` guess reporting a successful \
             install into a directory git never reads"
        );
        // ★ AND THE SAME DIRECTORY, NAMED ABSOLUTELY, IS STILL OURS TO INSTALL INTO —
        // measured: git runs the hook. Without this the refusal above would be
        // satisfied by a rule that condemned the directory rather than the spelling.
        assert_eq!(
            resolve_from_answers(
                Some(&ask(
                    "/nx-cf-dash-repo",
                    "/nx-cf-dash-repo/.git",
                    "/nx-cf-dash-repo/-hooks",
                )),
                root,
                true
            ),
            Some(HooksDir::Repo(root.join("-hooks"))),
            "the fault is the leading character of the command git execs, not the \
             directory: absolute works"
        );

        // NEGATIVE CONTROL: an echoed option is still refused — on COUNT, the shape
        // git actually emits — without any rule inspecting what a line LOOKS like.
        // That is exactly what lets `-hooks` above survive.
        assert_eq!(
            resolve_from_answers(
                Some("/r\n--git-common-dir\n.git\n-hooks\n-hooks/pre-commit\n"),
                root,
                true
            ),
            Some(HooksDir::Repo(root.join(".git").join("hooks"))),
            "a five-line answer must fall back, not be read as four"
        );
    }

    /// A directory whose name ends in a space is legal, and trimming broke it.
    #[test]
    fn a_path_is_not_trimmed_but_a_carriage_return_is() {
        use super::parse_git_paths;
        use std::path::PathBuf;

        let base = std::path::Path::new("/repo/trail ");
        let paths = parse_git_paths(&ask("/repo/trail ", ".git", ".git/hooks"), base)
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
            "/repo\r\n.git\r\n.git/hooks\r\n.git/hooks/pre-commit\r\n",
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
    /// `pre-commit.cf-install-<pid>-<n>` sibling survives.
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

    /// Repairing the bit must stay INSIDE the hooks directory.
    ///
    /// ⚠⚠ `set_permissions` follows a symlink, so this was the one write in the
    /// module that containment could not bound: the hooks DIRECTORY was checked, and
    /// then the chmod landed on wherever a link pointed. Measured before the fix, with
    /// the real `build.rs`: a mode-0600 file outside the repository came out 0755
    /// after a plain `cargo build`.
    ///
    /// ★ TWO-SIDED, because a refusal that refuses everything proves nothing. The
    /// regular-file leg is the positive control: the same call, on the shape that IS
    /// ours, must still repair the bit — otherwise git goes on silently ignoring a
    /// correct hook, which is the failure this function exists to prevent.
    #[test]
    #[cfg(unix)]
    fn repairing_the_bit_never_chmods_through_a_symlink() {
        use std::os::unix::fs::PermissionsExt;

        let base = std::env::temp_dir().join(format!("cf-lchmod-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&base);
        std::fs::create_dir_all(&base).expect("temp dir");

        // The file a link points at: outside any hooks directory, owner-private.
        let outside = base.join("somebody-elses-file");
        std::fs::write(&outside, "not a hook\n").expect("seed");
        std::fs::set_permissions(&outside, std::fs::Permissions::from_mode(0o600)).expect("chmod");
        let link = base.join("pre-commit");
        std::os::unix::fs::symlink(&outside, &link).expect("symlink");

        let refused = super::make_executable(&link)
            .expect_err("a symlink must not be chmodded — the mode lands on its target");
        // The same escape written the ordinary way: a RELATIVE target.
        let nested = base.join("hooks");
        std::fs::create_dir_all(&nested).expect("temp dir");
        let rel_link = nested.join("pre-commit");
        std::os::unix::fs::symlink("../somebody-elses-file", &rel_link).expect("symlink");
        let said_relative = super::make_executable(&rel_link)
            .expect_err("a relative symlink is still a symlink")
            .to_string();
        let after = std::fs::metadata(&outside)
            .expect("meta")
            .permissions()
            .mode();

        // POSITIVE CONTROL: a regular file in the same directory still gets repaired.
        let ours = base.join("commit-msg");
        std::fs::write(&ours, "hook\n").expect("seed");
        std::fs::set_permissions(&ours, std::fs::Permissions::from_mode(0o644)).expect("chmod");
        let repaired = super::make_executable(&ours);
        let ours_mode = std::fs::metadata(&ours).expect("meta").permissions().mode();
        let _ = std::fs::remove_dir_all(&base);

        assert_eq!(
            after & 0o777,
            0o600,
            "the link's target was widened — this is the escape itself, not a proxy \
             for it"
        );
        let said = refused.to_string();
        assert!(
            said.contains("symlink") && said.contains("somebody-elses-file"),
            "the refusal is printed verbatim by both installers, so it has to name \
             the file the developer must chmod: {said}"
        );
        // ⚠ AND A RELATIVE LINK, which is how these are normally written. `read_link`
        // returns the target verbatim, so the message used to print a path relative
        // to the HOOKS directory to a developer standing in the repo root, where it
        // resolves to nothing. The instruction is "chmod that file yourself"; it has
        // to name a file they can actually reach.
        //
        // ⚠⚠ ASSERT THE WHOLE RESOLVED PATH, not "no `../` and the basename appears".
        // That weaker pair was the first version of this assertion, and a mutant that
        // resolved the target against the PROCESS WORKING DIRECTORY rather than the
        // hook's own directory satisfied both — reintroducing the unpasteable path
        // this leg exists to prevent, from a different base.
        assert!(
            said_relative.contains(&base.join("somebody-elses-file").display().to_string()),
            "the refusal must resolve a relative link target against the HOOK's own \
             directory, not the process working directory: {said_relative}"
        );
        repaired.expect("a regular hook file is still ours to repair");
        assert!(
            ours_mode & 0o111 != 0,
            "the positive control did not repair: a refusal that refuses everything \
             would satisfy the assertion above while git ignores every hook"
        );
    }

    /// "Will git run this?" is a question about the TARGET, and must stay one.
    ///
    /// ⚠⚠ A ONE-TOKEN MUTANT SURVIVED A WHOLE ROUND HERE. `metadata` →
    /// `symlink_metadata` reads the LINK's own mode, which macOS reports as `120755`,
    /// so `& 0o111` is true for EVERY symlinked hook: `ensure_executable` returns
    /// early, the refusal never prints, and git goes on silently ignoring a hook
    /// whose target is not executable. Silent, which is the one outcome this module
    /// treats as unacceptable — and the safety argument in `make_executable`'s doc
    /// rests on this behaviour, which nothing was measuring.
    #[test]
    #[cfg(unix)]
    fn whether_git_will_run_a_hook_is_read_through_the_symlink() {
        use std::os::unix::fs::PermissionsExt;

        let base = std::env::temp_dir().join(format!("cf-execlink-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&base);
        std::fs::create_dir_all(&base).expect("temp dir");

        let dull = base.join("target-0644");
        std::fs::write(&dull, "hook\n").expect("seed");
        std::fs::set_permissions(&dull, std::fs::Permissions::from_mode(0o644)).expect("chmod");
        let to_dull = base.join("pre-commit");
        std::os::unix::fs::symlink(&dull, &to_dull).expect("symlink");

        // POSITIVE CONTROL: the same link shape over an executable target.
        let live = base.join("target-0755");
        std::fs::write(&live, "hook\n").expect("seed");
        std::fs::set_permissions(&live, std::fs::Permissions::from_mode(0o755)).expect("chmod");
        let to_live = base.join("commit-msg");
        std::os::unix::fs::symlink(&live, &to_live).expect("symlink");

        let over_dull = super::is_executable(&to_dull);
        let over_live = super::is_executable(&to_live);
        // The link's OWN mode, which is what the mutant would read.
        let link_mode = to_dull
            .symlink_metadata()
            .expect("lstat")
            .permissions()
            .mode();
        let _ = std::fs::remove_dir_all(&base);

        assert!(
            !over_dull,
            "a link to a NON-executable target was called runnable — git execs \
             through the link, so this hook will be silently ignored"
        );
        assert!(
            over_live,
            "POSITIVE CONTROL: a link to an executable target must be runnable, or \
             the assertion above is satisfied by a function that always says no"
        );
        assert!(
            link_mode & 0o111 != 0,
            "this test is only meaningful while a symlink's own mode carries \
             execute bits ({link_mode:o}); if that changes, the mutant it exists to \
             kill can no longer hide"
        );
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
