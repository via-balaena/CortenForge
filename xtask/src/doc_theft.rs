//! Detect doc comments stolen from the item below them.
//!
//! ## The failure mode
//!
//! Inserting a new item **above** an existing one, but **below** that item's doc
//! comment, silently reassigns the doc:
//!
//! ```text
//! /// Explains what `victim` does.        <-- belonged to `victim`
//! /// A brand-new doc for `thief`.        <-- inserted here
//! fn thief() {}                            <-- now wears BOTH comments
//!
//! fn victim() {}                           <-- lost its doc entirely
//! ```
//!
//! It has happened **ten times** in this repository, three of them on
//! 2026-08-31, and **two of those three occurred while fixing the first** — the
//! initial fix relocated the theft onto the next function instead of removing it.
//!
//! ## Why nothing else catches it
//!
//! - `missing_docs` does not fire on private items, and most victims are private
//!   helpers or `#[test]` functions.
//! - Everything still compiles, and every test still passes.
//! - The welded comment reads plausibly, so review misses it. On 2026-08-31 a
//!   plain read of the diff missed the same instance **twice**, including once
//!   after it had supposedly been fixed.
//!
//! ## What this checks
//!
//! Not "does every item have a doc" — plenty legitimately do not. The signal is
//! narrower and therefore trustworthy: **an item that HAD a doc comment before
//! the change and does NOT have one after.** That is not a style preference; it
//! is documentation that silently changed owner.
//!
//! ⚠ The parse is deliberately line-based rather than a real Rust parse. It only
//! needs to answer "is the line above this item a `///` line", which survives
//! macros, `cfg`, and generics that would defeat a naive AST walk for no gain.
//!
//! ## Known limits
//!
//! Both are **latent**: each was measured at **zero occurrences** in this
//! workspace, and both are shapes `rustfmt` normalises away. They are recorded
//! rather than fixed, because each additional parser rule has itself carried a
//! defect (see the `attribute_lines` and `bracket_delta` notes below).
//!
//! - A **blank line inside a multi-line attribute** ends the attribute scan
//!   early, so the item below reads as undocumented and adding such an
//!   attribute would report as theft.
//! - An **attribute and item on the same line** (`#[cfg(test)] fn a() {}`) is
//!   not recognised as an item at all, so a theft involving one is invisible.
//!
//! ## Coverage
//!
//! Measured over this workspace: **13 396 documented items tracked, 4 638 not
//! — 74 %.** The gap is entirely **struct fields (3 844)** and **enum variants
//! (794)**, which carry doc comments as readily as items do and can be robbed
//! the same way. Every other doc-bearing kind is covered: `macro` 2.0,
//! `extern crate` and trait aliases are all absent from this workspace, and
//! only 5 `impl` blocks carry docs.
//!
//! Fields and variants are deliberately **out of scope here**, not overlooked.
//! Recognising them means tracking brace depth to tell a field declaration
//! from a struct literal, and a variant from a match arm — a materially larger
//! parser than the one above, which has carried a defect into each of its last
//! four revisions. It belongs in its own change, reviewed against the same
//! real-history sweeps, rather than bolted onto this one.

use std::collections::BTreeMap;

/// Per item name, how many items carry that name and how many are documented.
///
/// Attributes (`#[test]`, `#[must_use]`, …) sit between a doc and its item, so
/// they are skipped when looking upward — otherwise every attributed item would
/// read as undocumented and the "lost a doc" signal would drown in noise.
#[must_use]
pub fn docs_by_item(source: &str) -> BTreeMap<String, ItemCounts> {
    let lines: Vec<&str> = source.lines().collect();
    let attr = attribute_lines(&lines);
    let mut out: BTreeMap<String, ItemCounts> = BTreeMap::new();

    for (i, line) in lines.iter().enumerate() {
        let Some(name) = item_name(line) else {
            continue;
        };

        // Walk up past everything Rust allows between a doc and its item.
        //
        // ⚠ That set is exactly three: attributes, plain `//` comments, and
        // BLANK LINES. Each was verified against `#![deny(missing_docs)]`,
        // which stays silent for all three — the doc still binds. Handling
        // only some of them is the same "1 of N sites" miss that left the
        // `//` case broken for 305 items.
        let mut j = i;
        while j > 0
            && (attr[j - 1] || is_plain_comment(lines[j - 1]) || lines[j - 1].trim().is_empty())
        {
            j -= 1;
        }
        let documented = j > 0 && lines[j - 1].trim_start().starts_with("///");

        let e = out.entry(name).or_default();
        e.total += 1;
        if documented {
            e.documented += 1;
        }
    }
    out
}

/// Which lines belong to an attribute, counting multi-line ones.
///
/// ⚠ An earlier version walked upward with `starts_with("#[")`. That stops at
/// the CLOSING line of a multi-line attribute (`)]`), so the item read as
/// undocumented and ADDING a multi-line `#[derive(…)]` to a documented item
/// reported as theft. Measured: 101 such attributes across 77 files here, two
/// already sitting directly under a doc comment.
fn attribute_lines(lines: &[&str]) -> Vec<bool> {
    let mut flags = vec![false; lines.len()];
    let mut depth: i32 = 0;
    for (i, line) in lines.iter().enumerate() {
        let t = line.trim_start();
        // ⚠ Bound the scan. A miscounted bracket used to leave `depth` stuck
        // above zero, marking every remaining line as attribute; the walk-up
        // then climbed to the top of the file and read EVERY item as
        // undocumented — which also makes a real theft invisible, since a file
        // already at zero documented can never show a decrease. An attribute
        // spans neither a blank line nor a doc comment, so either ends it.
        if t.is_empty() || t.starts_with("///") || t.starts_with("//!") {
            depth = 0;
            continue;
        }
        if depth == 0 && !t.starts_with("#[") && !t.starts_with("#![") {
            continue;
        }
        flags[i] = true;
        depth = (depth + bracket_delta(line)).max(0);
    }
    flags
}

/// Whether `line` is an ordinary `//` comment rather than a doc comment.
///
/// ⚠ Rust strips these before doc attributes bind, so a `//` note between a
/// `///` block and its item does NOT detach the doc — verified against
/// `#![deny(missing_docs)]`, which stays silent. Treating one as a separator
/// left **305 items in this workspace** reading as undocumented: a
/// false-positive whenever such a note is added, and a false NEGATIVE for all
/// 305, since an item already counted at zero can never show a decrease.
fn is_plain_comment(line: &str) -> bool {
    let t = line.trim_start();
    t.starts_with("//") && !t.starts_with("///") && !t.starts_with("//!")
}

/// Net `[` minus `]` on `line`, ignoring anything inside a string literal.
///
/// ⚠ `#[doc = "unmatched [ here"]` counts two opens and one close if the string
/// is included, leaving the scan permanently inside an attribute.
fn bracket_delta(line: &str) -> i32 {
    let mut depth = 0i32;
    let mut in_string = false;
    let mut escaped = false;
    for c in line.chars() {
        if in_string {
            if escaped {
                escaped = false;
            } else if c == '\\' {
                escaped = true;
            } else if c == '"' {
                in_string = false;
            }
            continue;
        }
        match c {
            '"' => in_string = true,
            '[' => depth += 1,
            ']' => depth -= 1,
            _ => {}
        }
    }
    depth
}

/// How many items share a name, and how many of those carry a doc comment.
///
/// ⚠ Counted, not reduced to a single bool. An earlier version kept only the
/// FIRST occurrence of a name, on the assumption that repeats were rare.
/// Measured, they are not: **102 of 1307 files** in this workspace declare the
/// same `fn` name twice (`new`, `default`, `from_str` across impl blocks), so a
/// theft on a repeated name was invisible in 8 % of the tree.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct ItemCounts {
    /// Items declared under this name.
    pub total: usize,
    /// How many of them have a doc comment directly above.
    pub documented: usize,
}

/// The item name declared on `line`, if it declares one.
///
/// Covers `fn`, `struct`, `enum`, `trait`, `type`, `union`, `mod`,
/// `macro_rules!`, `const` and `static`, at any visibility and with any of the
/// `async` / `unsafe` / `extern` / `const` qualifiers. Anything else — notably
/// `impl` blocks — yields `None` and is not tracked.
///
/// ⚠ `mod` and `macro_rules!` were missing until R8, which left **75
/// documented modules and 6 documented macros** in this workspace invisible: a
/// doc stolen from any of them could not be detected at all.
///
/// ⚠ `const` and `static` are checked BEFORE the qualifier strip, because
/// `const` is both an item kind (`const FOO: u8 = 1;`) and a qualifier
/// (`const fn f()`). An earlier version listed `const` only as a qualifier, so
/// every documented constant in the workspace was invisible to this check — and
/// its own doc comment claimed otherwise. The test that was meant to cover this
/// used `const fn`, which exercises the qualifier and not the kind.
fn item_name(line: &str) -> Option<String> {
    let mut t = line.trim_start();

    // Visibility first: it precedes both kinds and qualifiers.
    for vis in ["pub(crate) ", "pub(super) ", "pub(self) ", "pub "] {
        if let Some(rest) = t.strip_prefix(vis) {
            t = rest.trim_start();
            break;
        }
    }

    // `const NAME` / `static NAME` are ITEMS; `const fn` is a qualifier.
    for kw in ["const ", "static "] {
        if let Some(rest) = t.strip_prefix(kw) {
            let rest = rest.trim_start();
            if !rest.starts_with("fn ") {
                return name_from(rest.strip_prefix("mut ").unwrap_or(rest));
            }
        }
    }

    for prefix in [
        "default ", "async ", "unsafe ", "extern ", "const ", "static ",
    ] {
        // Repeat: `pub const fn` and `pub async unsafe fn` both occur.
        while let Some(rest) = t.strip_prefix(prefix) {
            t = rest.trim_start();
        }
    }

    // `macro_rules!` separates with `!`, not a space, so it cannot join the
    // list below.
    if let Some(rest) = t.strip_prefix("macro_rules!") {
        return name_from(rest.trim_start());
    }

    let kind = [
        "fn ", "struct ", "enum ", "trait ", "type ", "union ", "mod ",
    ]
    .into_iter()
    .find(|k| t.starts_with(k))?;

    name_from(t.strip_prefix(kind)?)
}

/// The leading identifier of `s`, if there is one.
fn name_from(s: &str) -> Option<String> {
    let name: String = s
        .chars()
        .take_while(|c| c.is_alphanumeric() || *c == '_')
        .collect();
    (!name.is_empty()).then_some(name)
}

/// Items that had a doc comment in `before` and have none in `after`.
///
/// Items added or removed by the change are ignored — only a name present on
/// both sides can have *lost* anything.
#[must_use]
pub fn items_that_lost_docs(before: &str, after: &str) -> Vec<String> {
    let b = docs_by_item(before);
    let a = docs_by_item(after);
    b.into_iter()
        .filter(|(name, before_counts)| {
            let Some(after_counts) = a.get(name) else {
                // Every item of that name is gone — a deletion, not a theft.
                return false;
            };
            // Fewer documented than before, while at least as many items exist:
            // something that had a doc no longer does, and it was not removed.
            //
            // ⚠ The `total` guard is what separates theft from deletion. Without
            // it, deleting one documented `new` out of two would report as a
            // loss — a false positive in 8 % of this workspace's files.
            after_counts.documented < before_counts.documented
                && after_counts.total >= before_counts.total
        })
        .map(|(name, _)| name)
        .collect()
}

/// Run the check over every `.rs` file changed between `base` and `HEAD`.
///
/// ⚠ Needs real history: `base...HEAD` requires the merge-base, so a shallow
/// checkout cannot run this. In CI it lives in the `affected` job, the one
/// already fetching `fetch-depth: 0` for exactly that reason.
///
/// # Errors
/// Returns an error if git cannot be run, or if any item lost a doc comment.
pub fn run(base: &str) -> anyhow::Result<()> {
    use anyhow::{bail, Context};
    use std::process::Command;

    let git = |args: &[&str]| -> anyhow::Result<String> {
        let out = Command::new("git")
            .args(args)
            .output()
            .with_context(|| format!("running git {}", args.join(" ")))?;
        // ⚠⚠ FAILS CLOSED. `.output()?` propagates only a SPAWN failure; a
        // non-zero git exit still returns Ok with empty stdout, and empty means
        // "no changed files" or "no before-text" — both of which report clean.
        // Same shape as the `.unwrap_or_default()` removed at the call sites.
        if !out.status.success() {
            bail!(
                "git {} failed ({}): {}",
                args.join(" "),
                out.status,
                String::from_utf8_lossy(&out.stderr).trim()
            );
        }
        Ok(String::from_utf8_lossy(&out.stdout).into_owned())
    };

    // ⚠ `git diff --name-only` yields REPO-RELATIVE paths, but the working-tree
    // reads below resolve against the CWD. Run from a subdirectory and every
    // path misses, each file counts as "deleted", and the check reports clean —
    // a theft would be invisible. Anchor on the repo root instead.
    let root = git(&["rev-parse", "--show-toplevel"])?.trim().to_string();

    // ⚠ NOT through the strict helper: `git merge-base` exits NON-ZERO when the
    // two commits share no ancestor, and git prints nothing to stderr for it.
    // Routing it through `git()` bailed with a bare "failed (exit status: 1):"
    // and buried the one message that tells a CI operator what to change. Empty
    // output is still fatal below, so this tolerates the status without
    // becoming fail-open.
    let merge_base = {
        let out = Command::new("git")
            .args(["merge-base", base, "HEAD"])
            .output()
            .context("running git merge-base")?;
        String::from_utf8_lossy(&out.stdout).trim().to_string()
    };
    if merge_base.is_empty() {
        bail!(
            "no merge-base between {base} and HEAD — this needs full history \
             (fetch-depth: 0), not a shallow checkout"
        );
    }

    // ⚠ No `HEAD` here, deliberately: `git diff --name-only <base>` includes
    // UNCOMMITTED work, and the content below is read from the working tree.
    // Listing committed changes while reading the working tree would disagree
    // locally — a theft you had not yet committed would be invisible to the very
    // check meant to catch it before you commit.
    let changed = git(&["diff", "--name-only", &merge_base])?;
    let files: Vec<&str> = changed.lines().filter(|f| f.ends_with(".rs")).collect();

    println!(
        "→ Checking {} changed Rust file(s) for stolen doc comments…",
        files.len()
    );

    // ⚠⚠ FAILS CLOSED. Both reads below used to be `.unwrap_or_default()`, which
    // turned any git or IO failure into "empty", and empty silently means "no
    // findings" — a broken input would report a clean check. That is the exact
    // shape the scan guard in `xtask/hooks/pre-commit` was fixed for (it used to
    // end in `|| true`). A guard that goes quiet when its inputs break is worse
    // than no guard, because it also stops anyone looking.
    //
    // The two legitimate "nothing to compare" cases are distinguished
    // explicitly, by asking git and the filesystem rather than by inferring
    // emptiness: a file ADDED by this change has no base version, and a file
    // DELETED by it has no working-tree version. Anything else is an error.
    let mut findings: Vec<(String, String)> = Vec::new();
    let mut skipped_added = 0usize;
    let mut skipped_deleted = 0usize;

    for file in &files {
        let blob = format!("{merge_base}:{file}");

        // Added by this change? Then there is no "before" and nothing can be lost.
        // ⚠ stderr silenced: a MISS here is the expected "file is new to this
        // change" path, and git prints `fatal: path ... exists on disk, but not
        // in <sha>` for it. A `fatal:` in a CI log reads as breakage, and a
        // check that cries wolf on its own happy path is one people learn to
        // scroll past. Only the exit status is consulted.
        let in_base = Command::new("git")
            .args(["cat-file", "-e", &blob])
            .stderr(std::process::Stdio::null())
            .status()
            .with_context(|| format!("git cat-file -e {blob}"))?
            .success();
        if !in_base {
            skipped_added += 1;
            continue;
        }

        // Deleted by this change? Then there is no "after" to compare against.
        let path = std::path::Path::new(&root).join(file);
        if !path.exists() {
            skipped_deleted += 1;
            continue;
        }

        let before = git(&["show", &blob])?;
        let after = std::fs::read_to_string(&path)
            .with_context(|| format!("reading {file} from the working tree"))?;

        for item in items_that_lost_docs(&before, &after) {
            findings.push(((*file).to_string(), item));
        }
    }

    // ★ Say what was NOT compared. A run that skipped everything and reported
    // nothing looks identical to a clean run otherwise — the "an empty result is
    // not evidence" failure this whole check exists to answer.
    let compared = files.len() - skipped_added - skipped_deleted;
    println!(
        "  compared {compared} file(s); skipped {skipped_added} added, \
         {skipped_deleted} deleted"
    );

    if findings.is_empty() {
        println!("✓ No doc comments changed owner");
        return Ok(());
    }

    eprintln!("\n✗ Doc comment(s) stolen from the item below them:\n");
    for (file, item) in &findings {
        eprintln!("    {file}: `{item}` LOST the doc comment it had before this change");
    }
    eprintln!(
        "\n  An item inserted above another, but below its doc comment, takes that\n  \
         comment with it. The only insertion point in a Rust item list that cannot\n  \
         steal a doc is between a `use` block and the next doc comment.\n"
    );
    bail!("{} doc comment(s) changed owner", findings.len())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn an_item_that_keeps_its_doc_is_not_reported() {
        let src = "/// Doc.\nfn a() {}\n";
        assert!(items_that_lost_docs(src, src).is_empty());
    }

    #[test]
    fn an_item_that_never_had_a_doc_is_not_reported() {
        // The check is about LOSS, not about coverage. Plenty of items are
        // legitimately undocumented and must not create noise.
        let before = "fn a() {}\n";
        let after = "fn a() {}\nfn b() {}\n";
        assert!(items_that_lost_docs(before, after).is_empty());
    }

    #[test]
    fn attributes_between_a_doc_and_its_item_do_not_hide_the_doc() {
        // ⚠ Without the attribute skip, every `#[test]` would read as
        // undocumented and real losses would drown in false positives.
        let src = "/// Doc.\n#[test]\n#[allow(clippy::all)]\nfn a() {}\n";
        assert_eq!(docs_by_item(src).get("a").map(|c| c.documented), Some(1));
    }

    #[test]
    fn the_real_2026_08_31_theft_is_detected() {
        // ★★★ The actual instance from PR #855, trimmed. A test inserted between
        // an existing test's doc and its `#[test]` left the original with none.
        // This gate exists because reading the diff missed it TWICE.
        let before = "\
    /// `ApexAxial` layout → NO separate funnel STL: the funnel is now
    /// integrated INTO the cup pieces. (Recon §7.8.)
    #[test]
    fn build_funnel_solid_apex_axial_is_none_now_integral() {}
";
        let after = "\
    /// `ApexAxial` layout → NO separate funnel STL: the funnel is now
    /// integrated INTO the cup pieces. (Recon §7.8.)
    /// ★★★ LITERAL expected values, deliberately NOT re-derived.
    #[test]
    fn nipple_outer_radius_is_dimensionally_pinned_and_asymmetric() {}

    #[test]
    fn build_funnel_solid_apex_axial_is_none_now_integral() {}
";
        assert_eq!(
            items_that_lost_docs(before, after),
            vec!["build_funnel_solid_apex_axial_is_none_now_integral".to_string()],
            "the gate must catch the instance that motivated it"
        );
    }

    #[test]
    fn the_fix_for_that_theft_reports_nothing() {
        // The same change done RIGHT: the new item goes somewhere that steals
        // nothing, and both items keep their own doc. A gate that fires on the
        // correct form too would just get switched off.
        let before = "\
    /// Victim doc.
    #[test]
    fn victim() {}
";
        let after = "\
    /// New item doc.
    #[test]
    fn newcomer() {}

    /// Victim doc.
    #[test]
    fn victim() {}
";
        assert!(items_that_lost_docs(before, after).is_empty());
    }

    #[test]
    fn relocating_the_theft_onto_the_next_item_is_still_reported() {
        // ⚠ The first attempt at fixing the real instance moved the new function
        // above a DIFFERENT function, stealing that one's doc instead. The gate
        // must not read a moved theft as a fix — it did not, which is how the
        // relocation was caught.
        let before = "/// A doc.\nfn a() {}\n\n/// B doc.\nfn b() {}\n";
        let after =
            "/// A doc.\nfn a() {}\n\n/// B doc.\n/// New doc.\nfn newcomer() {}\n\nfn b() {}\n";
        assert_eq!(items_that_lost_docs(before, after), vec!["b".to_string()]);
    }

    #[test]
    fn every_item_kind_is_tracked_not_just_fn() {
        for decl in [
            "fn a() {}",
            "struct a;",
            "enum a {}",
            "trait a {}",
            "type a = u8;",
            "union a { x: u8 }",
            "pub fn a() {}",
            "pub async unsafe fn a() {}",
            // ⚠ `const fn` exercises the QUALIFIER. The three below exercise
            // `const`/`static` as item KINDS, which is a different code path and
            // was silently untracked while this function's doc claimed otherwise
            // — every documented constant in the workspace was invisible.
            "pub(crate) const fn a() -> u8 { 0 }",
            "const a: u8 = 1;",
            "pub const a: u8 = 1;",
            "static mut a: u8 = 1;",
        ] {
            let before = format!("/// Doc.\n{decl}\n");
            let after = format!("{decl}\n");
            assert_eq!(
                items_that_lost_docs(&before, &after),
                vec!["a".to_string()],
                "kind not tracked: {decl}"
            );
        }
    }

    #[test]
    fn a_theft_on_a_repeated_name_is_caught() {
        // ★★★ The blind spot the counting model exists for. `new`, `default` and
        // `from_str` repeat across impl blocks in 102 of this workspace's 1307
        // files; keeping only the first occurrence made a theft on the SECOND one
        // invisible in all of them.
        let before = "\
/// A::new doc.
fn new() {}

/// B::new doc.
fn new() {}
";
        let after = "\
/// A::new doc.
fn new() {}

/// B::new doc.
/// Thief doc.
fn thief() {}

fn new() {}
";
        assert_eq!(
            items_that_lost_docs(before, after),
            vec!["new".to_string()],
            "the SECOND `new` lost its doc; only counting occurrences can see that"
        );
    }

    #[test]
    fn deleting_one_of_two_documented_namesakes_is_not_reported() {
        // ⚠ The false positive the `total` guard prevents. Without it, removing
        // one documented `new` of two reads as a loss — noise in 8 % of files,
        // which is how a gate gets switched off.
        let before = "/// One.\nfn new() {}\n\n/// Two.\nfn new() {}\n";
        let after = "/// One.\nfn new() {}\n";
        assert!(
            items_that_lost_docs(before, after).is_empty(),
            "a deletion is not a theft"
        );
    }

    #[test]
    fn a_deleted_item_is_not_a_loss() {
        // Deleting a documented item removes its doc too; that is not theft.
        let before = "/// Doc.\nfn gone() {}\n";
        assert!(items_that_lost_docs(before, "").is_empty());
    }

    #[test]
    fn a_multi_line_attribute_does_not_hide_the_doc_above_it() {
        // ⚠ The R5 false positive. Walking up with `starts_with("#[")` stops at
        // the CLOSING `)]` line, so the item reads as undocumented and merely
        // ADDING a multi-line derive reports as theft. Measured before the fix:
        // 101 multi-line attributes across 77 files, two already under a doc.
        let before = "/// Doc.\npub struct Widget;\n";
        let after = "/// Doc.\n#[derive(\n    Debug,\n)]\npub struct Widget;\n";
        assert!(
            items_that_lost_docs(before, after).is_empty(),
            "adding a multi-line attribute is not a theft"
        );
    }

    #[test]
    fn a_theft_under_a_multi_line_attribute_is_still_caught() {
        // The negative control for the fix above: widening the walk-up must not
        // blind the check. A real theft below a multi-line attribute still fails.
        let before = "/// Doc.\n#[derive(\n    Debug,\n)]\npub struct Widget;\n";
        let after =
            "/// Doc.\n#[derive(\n    Debug,\n)]\npub struct Thief;\n\npub struct Widget;\n";
        assert_eq!(
            items_that_lost_docs(before, after),
            vec!["Widget".to_string()],
            "a theft below a multi-line attribute must still be caught"
        );
    }

    #[test]
    fn an_unbalanced_bracket_in_an_attribute_does_not_poison_the_file() {
        // ⚠ The R6 runaway. `#[doc = "unmatched [ here"]` left the bracket depth
        // stuck above zero, so every later line read as attribute, the walk-up
        // climbed to the top of the file, and EVERY item read as undocumented.
        // That hides a real theft as well as inventing false ones: a file
        // already at zero documented can never show a decrease.
        let src = "use std::fmt;\n\
                   #[doc = \"unmatched [ here\"]\n\
                   pub struct A;\n\
                   \n\
                   /// Other.\n\
                   pub struct B;\n";
        let m = docs_by_item(src);
        assert_eq!(m["B"].documented, 1, "B's own doc must survive the runaway");
        assert_eq!(m["A"].documented, 0, "A genuinely has no doc comment");
    }

    #[test]
    fn a_blank_line_does_not_detach_a_doc() {
        // ⚠ Verified against `#![deny(missing_docs)]`: a blank line between a
        // doc and its item does NOT detach it. Zero occurrences here today,
        // but it is the third member of the separator set — the other two were
        // already handled, and a partial enumeration is how the `//` case was
        // missed.
        let before = "/// Doc.\npub const ITEM: u8 = 1;\n";
        let after = "/// Doc.\n\npub const ITEM: u8 = 1;\n";
        assert!(
            items_that_lost_docs(before, after).is_empty(),
            "a blank line between a doc and its item is not a theft"
        );
    }

    #[test]
    fn a_theft_across_a_blank_line_is_still_caught() {
        // Negative control: skipping blanks must not blind the check.
        let before = "/// Doc.\n\npub const ITEM: u8 = 1;\n";
        let after = "/// Doc.\n\npub const THIEF: u8 = 9;\n\npub const ITEM: u8 = 1;\n";
        assert_eq!(
            items_that_lost_docs(before, after),
            vec!["ITEM".to_string()],
            "a real theft across a blank line must still be caught"
        );
    }

    #[test]
    fn a_plain_comment_does_not_detach_a_doc() {
        // ⚠ The R8 false positive, found by sweeping 149 real files. Rust
        // strips `//` comments before doc attributes bind, so this item is
        // documented — confirmed against `#![deny(missing_docs)]`. 305 items
        // in this workspace sit in exactly this shape.
        let before = "/// Doc.\nconst INSET: f64 = 1.0;\n";
        let after = "/// Doc.\n// an explanatory note added later\nconst INSET: f64 = 1.0;\n";
        assert!(
            items_that_lost_docs(before, after).is_empty(),
            "a `//` note between a doc and its item is not a theft"
        );
    }

    #[test]
    fn a_theft_above_a_plain_comment_is_still_caught() {
        // Negative control: skipping `//` must not blind the check.
        let before = "/// Doc.\n// note\nconst INSET: f64 = 1.0;\n";
        let after = "/// Doc.\n// note\nconst THIEF: f64 = 2.0;\n\nconst INSET: f64 = 1.0;\n";
        assert_eq!(
            items_that_lost_docs(before, after),
            vec!["INSET".to_string()],
            "a real theft above a `//` note must still be caught"
        );
    }

    #[test]
    fn a_documented_module_is_tracked() {
        // ⚠ The R8 gap. `mod` was absent from the kind list, so 75 documented
        // modules in this workspace could lose a doc undetected.
        let before = "/// Module doc.\npub mod prelude {}\n";
        let after = "/// Module doc.\npub mod thief {}\n\npub mod prelude {}\n";
        assert_eq!(
            items_that_lost_docs(before, after),
            vec!["prelude".to_string()],
            "a theft from a documented `mod` must be caught"
        );
    }

    #[test]
    fn a_documented_macro_is_tracked() {
        // `macro_rules!` separates on `!`, so it needs its own branch.
        let before = "/// Macro doc.\nmacro_rules! shout {}\n";
        let after = "/// Macro doc.\nmacro_rules! thief {}\n\nmacro_rules! shout {}\n";
        assert_eq!(
            items_that_lost_docs(before, after),
            vec!["shout".to_string()],
            "a theft from a documented macro must be caught"
        );
    }

    #[test]
    fn brackets_inside_string_literals_do_not_count() {
        assert_eq!(bracket_delta(r#"#[doc = "see [link]"]"#), 0);
        assert_eq!(bracket_delta(r#"#[doc = "unmatched [ here"]"#), 0);
        assert_eq!(bracket_delta("#[derive("), 1);
        assert_eq!(bracket_delta(")]"), -1);
    }
}
