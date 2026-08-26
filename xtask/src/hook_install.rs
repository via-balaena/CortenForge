// The two decisions the git-hook installer makes, as pure functions.
//
// # Why these live in their own file
//
// They are `include!`d by `xtask/build.rs` AND compiled into the xtask binary, so
// they can be unit-tested. A build script has no test target, and while they lived
// only in `build.rs` NOTHING covered them — a mutation survey found that gutting
// the ownership check to `Ok(_) => true` (clobber every hook, including other
// people's) or `Err(_) => true` passed the entire suite. That is the same shape as
// the bug this whole arc exists to fix: installer logic with no gate on it.
//
// Keep them PURE. The filesystem work stays in `build.rs`; everything decidable
// from strings is decided here, where a test can reach it.

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

/// Should the hook at this path be replaced by `content`?
///
/// `existing` is `None` when there is no hook, and `Some(Err(..))` when one is there
/// but its bytes are not UTF-8.
///
/// The rule: install if absent; replace only a hook that is OURS and out of date;
/// never touch anything else. "Ours" is by TITLE line, which both historical stamps
/// carry — that is what lets a checkout stuck on the old `cargo xtask setup` hook
/// heal itself on the next build.
#[must_use]
pub fn should_replace(existing: Option<Result<&str, ()>>, content: &str, marker: &str) -> bool {
    match existing {
        // No hook yet — install ours.
        None => true,
        // Ours, and stale. Replace.
        Some(Ok(current)) if current.contains(marker) => current != content,
        // Someone else's hook. Leave it alone.
        Some(Ok(_)) => false,
        // A hook we cannot READ is not a hook we may overwrite.
        Some(Err(())) => false,
    }
}

#[cfg(test)]
mod tests {
    use super::{should_replace, title_of};

    const OURS: &str = "#!/bin/sh\n# CortenForge Pre-Commit Hook\necho hi\n";

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
        assert!(should_replace(None, OURS, "CortenForge Pre-Commit Hook"));
    }

    #[test]
    fn our_own_stale_hook_is_replaced_and_an_identical_one_is_not() {
        let m = "CortenForge Pre-Commit Hook";
        let stale = "#!/bin/sh\n# CortenForge Pre-Commit Hook\necho old\n";
        assert!(
            should_replace(Some(Ok(stale)), OURS, m),
            "stale not replaced"
        );
        assert!(
            !should_replace(Some(Ok(OURS)), OURS, m),
            "identical rewritten"
        );
    }

    /// The legacy hook that #709 stranded: stamped by `cargo xtask setup`, missing
    /// the scan/mesh guard. It carries the same TITLE line, which is exactly why
    /// matching on the title heals it where matching on the old sentinel could not.
    #[test]
    fn the_legacy_setup_stamped_hook_is_recognised_and_healed() {
        let legacy =
            "#!/bin/sh\n# CortenForge Pre-Commit Hook\n# Installed by: cargo xtask setup\n";
        assert!(should_replace(
            Some(Ok(legacy)),
            OURS,
            "CortenForge Pre-Commit Hook"
        ));
    }

    /// A foreign hook must survive, whether or not we can read it. Both arms have
    /// been `true` at some point in this file's history, and each would silently
    /// destroy a developer's own hook.
    #[test]
    fn a_foreign_hook_is_never_touched_readable_or_not() {
        let m = "CortenForge Pre-Commit Hook";
        let theirs = "#!/bin/sh\n# husky\nnpm test\n";
        assert!(
            !should_replace(Some(Ok(theirs)), OURS, m),
            "clobbered a foreign hook"
        );
        assert!(
            !should_replace(Some(Err(())), OURS, m),
            "clobbered an unreadable hook"
        );
    }

    /// The failure mode an empty marker would cause, pinned directly: if a marker of
    /// "" ever reached this function, EVERY hook would look like ours.
    #[test]
    fn an_empty_marker_would_match_a_foreign_hook_which_is_why_title_of_panics() {
        let theirs = "#!/bin/sh\n# husky\nnpm test\n";
        assert!(
            should_replace(Some(Ok(theirs)), OURS, ""),
            "if this ever stops being true the empty-marker hazard changed shape"
        );
    }
}
