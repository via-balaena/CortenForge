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
